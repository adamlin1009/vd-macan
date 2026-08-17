#!/usr/bin/env python3
"""Day-1 analysis: Storm Stadium afternoon session, 2026-08-15.

One script, one pass over the raw session (racebox.csv + imu_sd/WIT38-41),
producing everything the day-1 write-up reports:

  1. run detection + calibrated GPS virtual gates -> run times
  2. IMU clock model (per-file linear fit from run-envelope
     cross-correlation against the RaceBox) -> synchronized IMU windows
  3. per-run metrics: vmax, peak lateral g, roll-rate RMS, roll-rate
     per unit lateral-acceleration rate, roll gradient (accel-minus-v*yaw
     gravity-leak method), grip ceiling raw + roll-corrected
  4. processed tables (CSV/JSON) into data/<session>/processed/
  5. standalone SVG figures into figures/day1/
  6. optionally the assembled log post (--post PATH), with the same
     figures inlined using the site's CSS color tokens

Only numpy is required. Colors validated (dataviz six checks, dark
surface #14171c): Normal #4d7fc9, Sport+ #a9631c.

Usage:
  python3 tools/day1_analysis.py                 # tables + figures
  python3 tools/day1_analysis.py --post PATH     # ...and the post
"""
import argparse
import datetime as dt
import json
import re
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from tap_check import parse_flag61, parse_standard, ACC_SCALE, GYR_SCALE  # noqa: E402

REPO = HERE.parent
SESSION = REPO / "data" / "20260815_afternoon"
PROCESSED = SESSION / "processed"
FIGURES = REPO / "figures" / "day1"
MPH2MPS = 0.44704
R_E = 6371000.0
G0 = 9.81
IMU_FILES = ["WIT38.TXT", "WIT39.TXT", "WIT40.TXT", "WIT41.TXT"]

# calibrated gate shifts (fit to owner-remembered official times, rms 0.11 s)
START_SHIFT_M = 2.0
FINISH_SHIFT_M = 23.0
GATE_HALF_WIDTH_M = 12.0

MODES = ["Normal", "Sport+", "Normal", "Sport+", "Normal", "Sport+"]
IDX_N = (0, 2, 4)
IDX_S = (1, 3, 5)
C_N = "#4d7fc9"          # Normal  (validated)
C_S = "#a9631c"          # Sport+  (validated; site ramp ember)

# Two palettes for the same figure code: the site's CSS tokens when the
# SVG is inlined in the post, concrete hex (the same token values, dark
# theme) when written as standalone files. Applied via use_palette().
PALETTES = {
    "site": dict(INK="var(--ink)", DIM="var(--ink-dim)", GRID="var(--grid)",
                 STRONG="var(--grid-strong)", SURF="var(--panel-deep)",
                 MONO="font-family:var(--font-data),monospace;", BG=None),
    "standalone": dict(INK="#e8ebee", DIM="#8b95a3", GRID="#2a313b",
                       STRONG="#3a434f", SURF="#171b21",
                       MONO="font-family:ui-monospace,SFMono-Regular,Menlo,"
                            "Consolas,monospace;", BG="#14171c"),
}
INK = DIM = GRID = STRONG = SURF = MONO = BG = None


def use_palette(name):
    globals().update(PALETTES[name])

# site speed ramp stops (app/lap-color.ts), eased t^1.25
RAMP = [(0.0, (0xA9, 0x63, 0x1C)), (0.5, (0xF2, 0xA3, 0x3C)),
        (0.85, (0xFC, 0xD9, 0xA0)), (1.0, (0xFF, 0xF6, 0xE8))]


def ramp_color(t):
    t = min(1.0, max(0.0, t)) ** 1.25
    for (t0, c0), (t1, c1) in zip(RAMP, RAMP[1:]):
        if t <= t1:
            f = (t - t0) / (t1 - t0)
            return "#%02x%02x%02x" % tuple(
                int(round(a + f * (b - a))) for a, b in zip(c0, c1))
    return "#fff6e8"


# ---------------------------------------------------------------- racebox
def load_racebox():
    lines = (SESSION / "racebox.csv").read_text(errors="replace").splitlines()
    hdr = next(i for i, l in enumerate(lines) if l.startswith("Record,"))
    rows = [l.split(",") for l in lines[hdr + 1:] if l.count(",") >= 12]
    t = [dt.datetime.fromisoformat(p[1]) for p in rows]
    d = {
        "t0": t[0],
        "ts": np.array([(x - t[0]).total_seconds() for x in t]),
        "lat": np.array([float(p[2]) for p in rows]),
        "lon": np.array([float(p[3]) for p in rows]),
        "v": np.array([float(p[5]) for p in rows]) * MPH2MPS,
        "gx": np.array([float(p[6]) for p in rows]),
        "gy": np.array([float(p[7]) for p in rows]),
        "yaw": np.array([float(p[12]) for p in rows]),
    }
    la0 = np.radians(d["lat"].mean())
    lo0 = np.radians(d["lon"].mean())
    d["E"] = R_E * np.cos(la0) * (np.radians(d["lon"]) - lo0)
    d["N"] = R_E * (np.radians(d["lat"]) - la0)
    return d


def find_runs(d):
    """Bouts + calibrated gates + crossing times. Mirrors split_runs.m."""
    ts, v, gx, E, N = d["ts"], d["v"], d["gx"], d["E"], d["N"]
    mov = v > 4
    edges = np.flatnonzero(np.diff(mov.astype(int)))
    bouts, start = [], None
    for e in edges:
        if mov[e + 1]:
            start = e + 1
        elif start is not None:
            if ts[e] - ts[start] > 20 and v[start:e].max() > 20:
                bouts.append((start, e))
            start = None

    sa, sd_, fa, fd_ = [], [], [], []
    for a, b in bouts:
        a0 = a
        while a0 > 0 and v[a0 - 1] > 0.5:
            a0 -= 1
        iv = np.flatnonzero(v[a0:b] >= 5.0)[0] + a0
        f = (5.0 - v[iv - 1]) / max(v[iv] - v[iv - 1], 1e-6)
        sa.append([E[iv - 1] + f * (E[iv] - E[iv - 1]),
                   N[iv - 1] + f * (N[iv] - N[iv - 1])])
        u = np.array([E[iv + 5] - E[iv - 1], N[iv + 5] - N[iv - 1]])
        sd_.append(u / np.linalg.norm(u))
        k = None
        for i in range(b - 8, a + 8, -1):
            if (gx[i:i + 8] < -0.45).all():
                w2 = np.flatnonzero((ts >= ts[i]) & (ts <= ts[i] + 6))
                if len(w2) and v[w2].min() < 6:
                    k = i
                    break
        if k is None:
            fa.append(None)
            continue
        while k > a and gx[k - 1] < -0.30:
            k -= 1
        j, dist = k, 0.0
        while j > 0 and dist < 5.0:
            dist += float(np.hypot(E[j] - E[j - 1], N[j] - N[j - 1]))
            j -= 1
        fa.append([E[j], N[j]])
        u = np.array([E[k] - E[k - 8], N[k] - N[k - 8]])
        fd_.append(u / np.linalg.norm(u))

    P0 = np.median(np.array(sa), axis=0)
    u0 = np.median(np.array(sd_), axis=0)
    u0 /= np.linalg.norm(u0)
    P1 = np.median(np.array([x for x in fa if x is not None]), axis=0)
    u1 = np.median(np.array(fd_), axis=0)
    u1 /= np.linalg.norm(u1)
    P0 = P0 + START_SHIFT_M * u0
    P1 = P1 - FINISH_SHIFT_M * u1

    def crossing(P, u, lo, hi, vmin, width=GATE_HALF_WIDTH_M):
        dE = E[lo:hi] - P[0]
        dN = N[lo:hi] - P[1]
        s = dE * u[0] + dN * u[1]
        r = -dE * u[1] + dN * u[0]
        ok = np.flatnonzero((s[:-1] < 0) & (s[1:] >= 0)
                            & (np.abs(r[:-1]) < width) & (v[lo:hi][:-1] > vmin))
        if not len(ok):
            return None
        i = ok[0] + lo
        den = (s[ok[0] + 1] - s[ok[0]]) or 1e-9
        return ts[i] + (-s[ok[0]] / den) * (ts[i + 1] - ts[i])

    runs = []
    for a, b in bouts:
        a0 = max(0, a - 100)
        t_s = crossing(P0, u0, a0, min(b, a0 + 1000), 1.0)
        t_f = crossing(P1, u1, a, min(len(v) - 1, b + 50), 5.0)
        runs.append({"a": a, "b": b, "t_s": t_s, "t_f": t_f,
                     "time": t_f - t_s})
    return runs, (P0, u0), (P1, u1)


# ------------------------------------------------------------------ IMU
def load_imu_file(name, t0):
    """One WITn.TXT -> device time [s since RaceBox t0], acc [g], gyr [deg/s].

    Frames are 28-byte 0x55 0x61 records with the device RTC appended;
    every frame is kept here (the fusion loop repeats values in ~48% of
    frames — dedupe where a spectrum or a clean sample count matters).
    """
    buf = (SESSION / "imu_sd" / name).read_bytes()
    rows = parse_flag61(buf)
    if rows is None:
        rows, _ = parse_standard(buf)
    rows = [r for r in rows if r["t"] is not None]
    t = np.array([(r["t"] - t0).total_seconds() for r in rows])
    acc = np.array([r["acc"] for r in rows], float) * ACC_SCALE
    gyr = np.array([r["gyr"] for r in rows], float) * GYR_SCALE
    return t, acc, gyr


def dedupe(acc, gyr):
    """Mask keeping only frames whose acc OR gyr changed vs the previous."""
    return np.concatenate(([True], (np.diff(acc, axis=0) != 0).any(axis=1)
                           | (np.diff(gyr, axis=0) != 0).any(axis=1)))


# ------------------------------------------------------------- svg utils
W = 760


def svg_open(h, label):
    """Inline form for the post; standalone files add xmlns + a background."""
    if BG is None:
        return (f'<svg viewBox="0 0 {W} {h}" role="img" aria-label="{label}" '
                f'style="display:block;width:100%;height:auto">')
    return (f'<svg xmlns="http://www.w3.org/2000/svg" viewBox="0 0 {W} {h}" '
            f'width="{W}" height="{h}" role="img" aria-label="{label}">'
            f'<rect width="{W}" height="{h}" fill="{BG}"/>')


def txt(x, y, s, size=10, fill=None, anchor="start", extra="", halo=False):
    fill = DIM if fill is None else fill
    h = (f' stroke="{SURF}" stroke-width="3" paint-order="stroke"'
         if halo else "")
    return (f'<text x="{x:.1f}" y="{y:.1f}" style="{MONO}font-size:{size}px;'
            f'letter-spacing:0.06em" fill="{fill}" '
            f'text-anchor="{anchor}"{h}{extra}>{s}</text>')


def legend(x, y, entries):
    out, dx = [], 0
    for color, name in entries:
        out.append(f'<rect x="{x+dx}" y="{y-8}" width="10" height="10" '
                   f'rx="2" fill="{color}"/>')
        out.append(txt(x + dx + 15, y + 1, name, 10, INK))
        dx += 15 + 8 * len(name) + 26
    return "".join(out)


def path_of(X, Y, step=1):
    pts = [f"{X[i]:.1f},{Y[i]:.1f}" for i in range(0, len(X), step)]
    return "M" + "L".join(pts)


# ------------------------------------------------------------------ figs
def fig_map(d, runs, gates0, gates1):
    r6 = runs[5]
    a, b = r6["a"], r6["b"]
    seg = np.flatnonzero((d["ts"] >= r6["t_s"] - 1) & (d["ts"] <= r6["t_f"] + 3))
    E, N, v = d["E"][seg], d["N"][seg], d["v"][seg]
    h = 560
    pad = 46
    minE, maxE = E.min(), E.max()
    minN, maxN = N.min(), N.max()
    sc = min((W - 2 * pad) / (maxE - minE), (h - 2 * pad) / (maxN - minN))
    X = pad + (E - minE) * sc + ((W - 2 * pad) - (maxE - minE) * sc) / 2
    Y = h - pad - (N - minN) * sc - ((h - 2 * pad) - (maxN - minN) * sc) / 2
    vmin, vmax = v.min(), v.max()
    parts = [svg_open(h, "Course map of run 6, colored by speed")]
    # speed-colored chunks (24 levels merged)
    lev = np.clip(((v - vmin) / (vmax - vmin) * 23).astype(int), 0, 23)
    i = 0
    while i < len(X) - 1:
        j = i
        while j < len(X) - 1 and lev[j] == lev[i]:
            j += 1
        col = ramp_color((lev[i] + 0.5) / 24)
        parts.append(f'<path d="{path_of(X[i:j+1], Y[i:j+1])}" fill="none" '
                     f'stroke="{col}" stroke-width="3" stroke-linecap="round" '
                     f'stroke-linejoin="round"/>')
        i = j
    # gates
    for (P, u), name in [(gates0, "START"), (gates1, "FINISH")]:
        gx_ = pad + (P[0] - minE) * sc + ((W - 2 * pad) - (maxE - minE) * sc) / 2
        gy_ = h - pad - (P[1] - minN) * sc - ((h - 2 * pad) - (maxN - minN) * sc) / 2
        px, py = -u[1], u[0]
        L = 14 * sc if 14 * sc > 18 else 18
        parts.append(f'<line x1="{gx_-px*L:.1f}" y1="{gy_+py*L:.1f}" '
                     f'x2="{gx_+px*L:.1f}" y2="{gy_-py*L:.1f}" '
                     f'stroke="{INK}" stroke-width="2"/>')
        parts.append(txt(gx_, gy_ + (26 if name == "START" else -18), name,
                         10, INK, "middle"))
    # scale bar 50 m
    bx = W - pad - 50 * sc
    parts.append(f'<line x1="{bx:.1f}" y1="{h-18}" x2="{W-pad}" y2="{h-18}" '
                 f'stroke="{DIM}" stroke-width="2"/>')
    parts.append(txt(W - pad - 25 * sc, h - 26, "50 M", 9, DIM, "middle"))
    # speed legend ramp
    for i in range(24):
        parts.append(f'<rect x="{pad+i*5}" y="{h-24}" width="5" height="6" '
                     f'fill="{ramp_color((i+0.5)/24)}"/>')
    parts.append(txt(pad, h - 30, f"{vmin/MPH2MPS:.0f} MPH", 9, DIM))
    parts.append(txt(pad + 120, h - 30, f"{vmax/MPH2MPS:.0f} MPH", 9, DIM))
    parts.append("</svg>")
    return "".join(parts)


def fig_times(runs):
    times = [r["time"] for r in runs]
    h = 300
    padl, padr, padt, padb = 64, 24, 30, 44
    y0, y1 = 50.8, 53.6
    def ymap(t):
        return padt + (y1 - t) / (y1 - y0) * (h - padt - padb)
    def xmap(i):
        return padl + (i + 0.5) * (W - padl - padr) / 6
    parts = [svg_open(h, "Run times by PASM mode")]
    for g in [51, 52, 53]:
        parts.append(f'<line x1="{padl}" y1="{ymap(g):.1f}" x2="{W-padr}" '
                     f'y2="{ymap(g):.1f}" stroke="{GRID}" stroke-width="1"/>')
        parts.append(txt(padl - 10, ymap(g) + 3, f"{g}.0", 10, DIM, "end"))
    for i, t in enumerate(times):
        c = C_N if MODES[i] == "Normal" else C_S
        x, y = xmap(i), ymap(t)
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="7" fill="{c}" '
                     f'stroke="{SURF}" stroke-width="2">'
                     f'<title>Run {i+1} · {MODES[i]} · {t:.2f} s</title></circle>')
        parts.append(txt(x, y - 14, f"{t:.2f}", 10, INK, "middle"))
        parts.append(txt(x, h - padb + 18, f"RUN {i+1}", 9, DIM, "middle"))
        parts.append(txt(x, h - padb + 31, MODES[i].upper(), 9,
                         C_N if MODES[i] == "Normal" else C_S, "middle"))
    parts.append(txt(8, padt - 12, "RUN TIME · S", 9, DIM))
    parts.append(legend(padl + 96, padt - 20, [(C_N, "NORMAL"), (C_S, "SPORT+")]))
    parts.append("</svg>")
    return "".join(parts)


def fig_speed(d, runs):
    h = 320
    padl, padr, padt, padb = 64, 24, 34, 40
    parts = [svg_open(h, "Speed against course distance, best Normal vs best Sport+ run")]
    vmax_all = 0
    series = []
    for idx, color, name in [(4, C_N, "RUN 5 · NORMAL"), (5, C_S, "RUN 6 · SPORT+")]:
        r = runs[idx]
        seg = np.flatnonzero((d["ts"] >= r["t_s"]) & (d["ts"] <= r["t_f"]))
        E, N, v = d["E"][seg], d["N"][seg], d["v"][seg] / MPH2MPS
        s = np.concatenate(([0], np.cumsum(np.hypot(np.diff(E), np.diff(N)))))
        series.append((s, v, color, name))
        vmax_all = max(vmax_all, v.max())
    smax = max(s[-1] for s, *_ in series)
    def xmap(x):
        return padl + x / smax * (W - padl - padr)
    def ymap(y):
        return padt + (vmax_all * 1.06 - y) / (vmax_all * 1.06) * (h - padt - padb)
    for g in [0, 20, 40, 60]:
        parts.append(f'<line x1="{padl}" y1="{ymap(g):.1f}" x2="{W-padr}" '
                     f'y2="{ymap(g):.1f}" stroke="{GRID}" stroke-width="1"/>')
        parts.append(txt(padl - 10, ymap(g) + 3, str(g), 10, DIM, "end"))
    for m in range(0, int(smax) + 1, 200):
        parts.append(txt(xmap(m), h - padb + 18, f"{m} M", 9, DIM, "middle"))
    for s, v, color, name in series:
        X = xmap(s)
        Y = ymap(v)
        parts.append(f'<path d="{path_of(X, Y, 2)}" fill="none" stroke="{color}" '
                     f'stroke-width="2" stroke-linecap="round" '
                     f'stroke-linejoin="round"/>')
    parts.append(txt(8, padt - 12, "SPEED · MPH", 9, DIM))
    parts.append(legend(padl + 96, padt - 20, [(C_N, "RUN 5 · NORMAL"), (C_S, "RUN 6 · SPORT+")]))
    parts.append("</svg>")
    return "".join(parts)


def fig_gg(d, runs):
    h = 430
    cx, cy = W / 2, (h - 26) / 2 + 4
    sc = 150
    parts = [svg_open(h, "Lateral versus longitudinal acceleration by PASM mode")]
    for g in [0.5, 1.0]:
        parts.append(f'<circle cx="{cx}" cy="{cy}" r="{g*sc:.1f}" fill="none" '
                     f'stroke="{GRID}" stroke-width="1"/>')
    parts.append(f'<line x1="{cx-1.25*sc}" y1="{cy}" x2="{cx+1.25*sc}" y2="{cy}" '
                 f'stroke="{GRID}" stroke-width="1"/>')
    parts.append(f'<line x1="{cx}" y1="{cy-1.3*sc}" x2="{cx}" y2="{cy+1.3*sc}" '
                 f'stroke="{GRID}" stroke-width="1"/>')
    # registered-prediction band 0.75-0.85 g
    parts.append(f'<circle cx="{cx}" cy="{cy}" r="{0.75*sc:.1f}" fill="none" '
                 f'stroke="{STRONG}" stroke-width="1"/>')
    parts.append(f'<circle cx="{cx}" cy="{cy}" r="{0.85*sc:.1f}" fill="none" '
                 f'stroke="{STRONG}" stroke-width="1"/>')
    parts.append(txt(cx, cy - 0.80 * sc - 6, "REGISTERED PREDICTION · 0.75–0.85 G",
                     9, DIM, "middle", halo=True))
    for want, color in [("Normal", C_N), ("Sport+", C_S)]:
        for i, r in enumerate(runs):
            if MODES[i] != want:
                continue
            seg = np.flatnonzero((d["ts"] >= r["t_s"]) & (d["ts"] <= r["t_f"]))
            X = cx + d["gy"][seg] * sc
            Y = cy + d["gx"][seg] * sc * -1
            parts.append(f'<path d="{path_of(X, Y, 2)}" fill="none" '
                         f'stroke="{color}" stroke-width="1.2" '
                         f'stroke-opacity="0.75" stroke-linejoin="round"/>')
    for g in [0.5, 1.0]:            # ring labels on the sparse diagonal, haloed
        parts.append(txt(cx + g * sc * 0.707 + 3, cy + g * sc * 0.707 + 10,
                         f"{g:.1f} G", 9, DIM, "start", halo=True))
    parts.append(txt(cx + 1.25 * sc + 8, cy + 3, "LAT G →", 9, DIM,
                     halo=True))
    parts.append(txt(cx + 8, cy - 1.3 * sc + 8, "ACCEL ↑", 9, DIM, halo=True))
    parts.append(txt(cx + 8, cy + 1.3 * sc - 2, "BRAKE ↓", 9, DIM, halo=True))
    parts.append(legend(24, 22, [(C_N, "NORMAL RUNS 1·3·5"), (C_S, "SPORT+ RUNS 2·4·6")]))
    parts.append("</svg>")
    return "".join(parts)


# Which SD file holds which runs. A new WITn.TXT opens at every IMU
# power-on; WIT39 (29.6 min) spans runs 1-4 and WIT40 spans runs 5-6 once
# their clocks are corrected. WIT38/WIT41 are short pre/post-session
# fragments (kept raw for completeness, no run inside them).
IMU_RUN_FILES = [("WIT39.TXT", [0, 1, 2, 3]), ("WIT40.TXT", [4, 5])]
OFF_SEARCH = np.arange(100, 220, 0.2)   # s; the device clock trailed GPS
EXPORT_PRE_S, EXPORT_POST_S = 3.0, 6.0  # per-run export window margins


def analyze_imu(d, runs):
    """Per-file clock fit (envelope xcorr vs RaceBox) + per-run metrics.

    The IMU's tick is a clean 5 ms RELATIVE, but absolute rate runs ~2%
    slow and the offset re-arms each power-on: fit OFF(t)=a+b*t per file
    from per-run cross-correlation, then window with the corrected clock
    (wall = device + OFF).
    """
    out = {"roll": [None] * 6, "corr_ay": [None] * 6, "xc": [None] * 6,
           "off": [None] * 6, "file": [None] * 6, "fits": [], "anchors": [],
           "windows": [None] * 6, "norm": [None] * 6}
    rb_env_t = d["ts"]
    rb_env = np.hypot(d["gx"], d["gy"])
    for name, runidx in IMU_RUN_FILES:
        pt, acc, gyr = load_imu_file(name, d["t0"])
        paxy = np.hypot(acc[:, 0], acc[:, 1])
        anchors = []
        for ri in runidx:
            s, f = runs[ri]["t_s"], runs[ri]["t_f"]
            grid = np.arange(s - 15, f + 15, 0.2)
            rbv = np.interp(grid, rb_env_t, rb_env)
            best, boff = -9, None
            for OFF in OFF_SEARCH:
                pv = np.interp(grid - OFF, pt, paxy, left=0, right=0)
                c = np.corrcoef(rbv, pv)[0, 1]
                if c > best:
                    best, boff = c, OFF
            anchors.append((0.5 * (s + f), boff))
            out["xc"][ri] = float(best)
            out["anchors"].append({"imu_file": name, "run": ri + 1,
                                   "t_mid_s": 0.5 * (s + f),
                                   "offset_s": float(boff),
                                   "xcorr": float(best)})
        x = np.array([a[0] for a in anchors])
        y = np.array([a[1] for a in anchors])
        b, a = np.polyfit(x, y, 1) if len(x) > 1 else (0.0, y[0])
        out["fits"].append({"imu_file": name, "runs": [r + 1 for r in runidx],
                            "intercept_s": float(a), "slope": float(b),
                            "n_anchors": len(x)})
        keep = dedupe(acc, gyr)
        for ri in runidx:
            s, f = runs[ri]["t_s"], runs[ri]["t_f"]
            OFF = a + b * 0.5 * (s + f)
            m = (pt >= s - OFF) & (pt <= f - OFF)
            out["roll"][ri] = float(np.sqrt(np.mean(gyr[m, 0] ** 2)))
            grid = np.arange(s + 1, f - 1, 0.2)
            pay = np.interp(grid, pt[m] + OFF, acc[m, 1])
            rgy = np.interp(grid, rb_env_t, d["gy"])
            out["corr_ay"][ri] = float(np.corrcoef(pay, rgy)[0, 1])
            out["off"][ri] = float(OFF)
            out["file"][ri] = name
            w = keep & (pt + OFF >= s - EXPORT_PRE_S) & (pt + OFF <= f + EXPORT_POST_S)
            out["windows"][ri] = (pt[w] + OFF, acc[w], gyr[w])
    # roll rate normalized by lateral-accel rate (RaceBox), per run
    for i, r in enumerate(runs):
        m = (d["ts"] >= r["t_s"]) & (d["ts"] <= r["t_f"])
        aydot = np.sqrt(np.mean(np.gradient(d["gy"][m], d["ts"][m]) ** 2))
        out["norm"][i] = out["roll"][i] / aydot
    out["off_first"], out["off_last"] = out["off"][0], out["off"][-1]
    out["norm_n"] = float(np.mean([out["norm"][i] for i in IDX_N]))
    out["norm_s"] = float(np.mean([out["norm"][i] for i in IDX_S]))
    return out


def fig_roll(roll_rms):
    h = 300
    padl, padr, padt, padb = 64, 24, 34, 44
    ymax = max(roll_rms) * 1.25
    def ymap(v):
        return padt + (ymax - v) / ymax * (h - padt - padb)
    def xmap(i):
        return padl + (i + 0.5) * (W - padl - padr) / 6
    parts = [svg_open(h, "Roll-rate RMS per run by PASM mode, from the console-mounted IMU")]
    for g in range(0, int(ymax) + 1, 2):
        parts.append(f'<line x1="{padl}" y1="{ymap(g):.1f}" x2="{W-padr}" '
                     f'y2="{ymap(g):.1f}" stroke="{GRID}" stroke-width="1"/>')
        parts.append(txt(padl - 10, ymap(g) + 3, f"{g}", 10, DIM, "end"))
    bw = 24
    for i, v in enumerate(roll_rms):
        c = C_N if MODES[i] == "Normal" else C_S
        x = xmap(i) - bw / 2
        y = ymap(v)
        parts.append(f'<path d="M{x:.1f},{ymap(0):.1f} L{x:.1f},{y+4:.1f} '
                     f'Q{x:.1f},{y:.1f} {x+4:.1f},{y:.1f} L{x+bw-4:.1f},{y:.1f} '
                     f'Q{x+bw:.1f},{y:.1f} {x+bw:.1f},{y+4:.1f} '
                     f'L{x+bw:.1f},{ymap(0):.1f} Z" fill="{c}">'
                     f'<title>Run {i+1} · {MODES[i]} · {v:.2f} deg/s RMS</title></path>')
        parts.append(txt(xmap(i), y - 8, f"{v:.2f}", 10, INK, "middle"))
        parts.append(txt(xmap(i), h - padb + 18, f"RUN {i+1}", 9, DIM, "middle"))
        parts.append(txt(xmap(i), h - padb + 31, MODES[i].upper(), 9,
                         C_N if MODES[i] == "Normal" else C_S, "middle"))
    parts.append(f'<line x1="{padl}" y1="{ymap(0):.1f}" x2="{W-padr}" '
                 f'y2="{ymap(0):.1f}" stroke="{STRONG}" stroke-width="1"/>')
    parts.append(txt(8, padt - 12, "ROLL-RATE RMS · °/S", 9, DIM))
    parts.append(legend(padl + 96, padt - 20, [(C_N, "NORMAL"), (C_S, "SPORT+")]))
    parts.append("</svg>")
    return "".join(parts)


def roll_gradient(d, runs):
    """Roll angle from the accelerometer/kinematic disagreement:
    phi ~ (ay_accel - v*yaw_rate)/g at quasi-steady cornering samples.
    Same-device (RaceBox) only — cross-device versions are biased by the
    +-130 ms clock residual. Returns per-run slopes + pooled points."""
    ay_kin = d["v"] * np.radians(d["yaw"]) / G0
    slopes, pts, samples = [], {"Normal": [], "Sport+": []}, []
    for ri, r in enumerate(runs):
        g = np.arange(r["t_s"] + 1, r["t_f"] - 1, 0.04)
        ayk = np.interp(g, d["ts"], ay_kin)
        aym = np.interp(g, d["ts"], d["gy"])
        v = np.interp(g, d["ts"], d["v"])
        gx = np.interp(g, d["ts"], d["gx"])
        day = np.gradient(ayk, g)
        qs = QS_MASK(ayk, day, v, gx)
        phi = np.degrees(aym[qs] - ayk[qs])
        slopes.append(float(np.polyfit(ayk[qs], phi, 1)[0]))
        pts[MODES[ri]] += list(zip(ayk[qs], phi))
        samples += [(ri + 1, MODES[ri], t, a, p)
                    for t, a, p in zip(g[qs], ayk[qs], phi)]
    return slopes, pts, samples


# quasi-steady cornering mask for the roll-gradient regression:
# cornering hard enough to matter, lateral acceleration not ramping,
# rolling, and not under significant braking/throttle.
QS_MASK_DEF = {"abs_ay_kin_g_min": 0.30, "abs_day_gps_max": 0.30,
               "speed_mps_min": 8.0, "abs_along_g_max": 0.25}


def QS_MASK(ayk, day, v, gx):
    m = QS_MASK_DEF
    return ((np.abs(ayk) > m["abs_ay_kin_g_min"])
            & (np.abs(day) < m["abs_day_gps_max"])
            & (v > m["speed_mps_min"]) & (np.abs(gx) < m["abs_along_g_max"]))


def fig_rollgrad(slopes, pts):
    h = 380
    padl, padr, padt, padb = 64, 24, 34, 44
    x0, x1, y0, y1 = -1.15, 1.15, -4.4, 4.4
    def xm(x):
        return padl + (x - x0) / (x1 - x0) * (W - padl - padr)
    def ym(y):
        return padt + (y1 - y) / (y1 - y0) * (h - padt - padb)
    parts = [svg_open(h, "Roll angle versus lateral acceleration by PASM mode")]
    parts.append(f'<line x1="{xm(x0)}" y1="{ym(0):.1f}" x2="{xm(x1)}" '
                 f'y2="{ym(0):.1f}" stroke="{GRID}" stroke-width="1"/>')
    parts.append(f'<line x1="{xm(0):.1f}" y1="{padt}" x2="{xm(0):.1f}" '
                 f'y2="{h-padb}" stroke="{GRID}" stroke-width="1"/>')
    for gv in [-1.0, -0.5, 0.5, 1.0]:
        parts.append(txt(xm(gv), h - padb + 16, f"{gv:+.1f} G", 9, DIM, "middle"))
    for dv in [-4, -2, 2, 4]:
        parts.append(txt(padl - 8, ym(dv) + 3, f"{dv:+d}°", 9, DIM, "end"))
    for mode, color in [("Normal", C_N), ("Sport+", C_S)]:
        p = pts[mode][:: max(1, len(pts[mode]) // 300)]
        for x, y in p:
            if x0 < x < x1 and y0 < y < y1:
                parts.append(f'<circle cx="{xm(x):.1f}" cy="{ym(y):.1f}" r="2" '
                             f'fill="{color}" fill-opacity="0.5"/>')
    for mode, color, idxs in [("Normal", C_N, (0, 2, 4)),
                              ("Sport+", C_S, (1, 3, 5))]:
        # binned means: the trend the fit follows, visible over the cloud
        arr = np.array(pts[mode])
        for b0 in np.arange(-1.1, 1.1, 0.2):
            mb = (arr[:, 0] >= b0) & (arr[:, 0] < b0 + 0.2)
            if mb.sum() >= 12 and y0 < arr[mb, 1].mean() < y1:
                parts.append(f'<circle cx="{xm(b0+0.1):.1f}" '
                             f'cy="{ym(arr[mb,1].mean()):.1f}" r="5" '
                             f'fill="{color}" stroke="{SURF}" stroke-width="2"/>')
        s = np.mean([slopes[i] for i in idxs])
        parts.append(f'<line x1="{xm(-1.1):.1f}" y1="{ym(-1.1*s):.1f}" '
                     f'x2="{xm(1.1):.1f}" y2="{ym(1.1*s):.1f}" stroke="{color}" '
                     f'stroke-width="2"/>')
        parts.append(txt(xm(1.1) - 4, ym(1.1 * s) + (14 if mode == "Sport+" else -8),
                         f"{s:+.1f}°/G {mode.upper()}", 9, INK, "end", halo=True))
    parts.append(txt(8, padt - 12, "ROLL ANGLE FROM ACCEL/KINEMATIC SPLIT · °", 9, DIM))
    parts.append(legend(padl + 250, padt - 20, [(C_N, "NORMAL"), (C_S, "SPORT+")]))
    parts.append("</svg>")
    return "".join(parts)


def figure(svg, num, title, note):
    return (f'<figure class="log-figure">{svg}<figcaption>'
            f'<span>FIG {num:02d}</span><span>{title}</span>'
            f'<span>{note}</span></figcaption></figure>\n')


FIGURE_SPECS = [
    # key, file stem, number, title, note
    ("MAP", "fig01_course_map", 1,
     "RUN 6 · GPS PATH, SPEED-COLORED · VIRTUAL GATES", "RACEBOX MINI S · 25 HZ"),
    ("TIMES", "fig02_run_times", 2,
     "RUN TIMES BY PASM MODE · GATES CALIBRATED TO OFFICIAL TIMES",
     "STORM STADIUM · 2026-08-15"),
    ("SPEED", "fig03_speed_distance", 3,
     "SPEED VS COURSE DISTANCE · BEST RUN PER MODE", "RACEBOX MINI S · 25 HZ"),
    ("GG", "fig04_gg_diagram", 4,
     "G-G DIAGRAM BY PASM MODE · ALL SIX RUNS",
     "ROOF-MOUNTED · UNCORRECTED FOR BODY ROLL"),
    ("ROLL", "fig05_roll_rate_rms", 5,
     "ROLL-RATE RMS PER RUN · CONSOLE-MOUNTED IMU",
     "WT901SDCL-BT50 · CLOCK SYNCED TO GPS"),
    ("RGRAD", "fig06_roll_gradient", 6,
     "ROLL ANGLE VS LATERAL G · QUASI-STEADY SAMPLES",
     "RACEBOX ACCEL MINUS V×YAW · SELF-CONSISTENT"),
]


def render_figures(R):
    """All six SVGs under the currently selected palette -> {key: svg}."""
    d, runs = R["d"], R["runs"]
    return {
        "MAP": fig_map(d, runs, R["gate_start"], R["gate_finish"]),
        "TIMES": fig_times(runs),
        "SPEED": fig_speed(d, runs),
        "GG": fig_gg(d, runs),
        "ROLL": fig_roll(R["imu"]["roll"]),
        "RGRAD": fig_rollgrad(R["rg_slopes"], R["rg_pts"]),
    }


# --------------------------------------------------------------- analysis
def analyze():
    """Everything the post reports, as one dict of arrays and scalars."""
    d = load_racebox()
    runs, gate_start, gate_finish = find_runs(d)
    imu = analyze_imu(d, runs)

    lat = [float(np.abs(d["gy"][r["a"]:r["b"]]).max()) for r in runs]
    brake = [float(-d["gx"][r["a"]:r["b"]].min()) for r in runs]
    vmax = [float(d["v"][r["a"]:r["b"]].max()) for r in runs]
    corner = (d["v"] > 8) & (np.abs(d["gy"]) > 0.3)
    in_runs = np.zeros(len(d["ts"]), bool)
    for r in runs:
        in_runs |= (d["ts"] >= r["t_s"]) & (d["ts"] <= r["t_f"])
    lat_p95 = float(np.percentile(np.abs(d["gy"][corner & in_runs]), 95))
    lat_max = float(np.abs(d["gy"][corner & in_runs]).max())

    rg_slopes, rg_pts, rg_samples = roll_gradient(d, runs)
    rgn = float(np.mean([rg_slopes[i] for i in IDX_N]))
    rgs = float(np.mean([rg_slopes[i] for i in IDX_S]))
    grad_rad = float(np.radians(0.5 * (rgn + rgs)))
    rn = float(np.mean([imu["roll"][i] for i in IDX_N]))
    rs = float(np.mean([imu["roll"][i] for i in IDX_S]))

    return {
        "d": d, "runs": runs, "gate_start": gate_start,
        "gate_finish": gate_finish, "imu": imu, "lat": lat, "brake": brake,
        "vmax": vmax, "lat_p95": lat_p95, "lat_max": lat_max,
        "rg_slopes": rg_slopes, "rg_pts": rg_pts, "rg_samples": rg_samples,
        "rgn": rgn, "rgs": rgs, "grad_rad": grad_rad, "roll_n": rn,
        "roll_s": rs, "lat_p95_corr": lat_p95 / (1 + grad_rad),
        "lat_max_corr": lat_max / (1 + grad_rad),
    }


def print_summary(R):
    runs, imu = R["runs"], R["imu"]
    print("runs:", [f"{r['time']:.2f}s" for r in runs])
    print("roll RMS deg/s:", [f"{x:.2f}" for x in imu["roll"]])
    print(f"clock offsets {imu['off_first']:.0f} -> {imu['off_last']:.0f} s; "
          f"xcorr {min(imu['xc']):.2f}-{max(imu['xc']):.2f}; "
          f"ay corr {min(imu['corr_ay']):+.2f}..{max(imu['corr_ay']):+.2f}; "
          f"norm N {imu['norm_n']:.2f} vs S+ {imu['norm_s']:.2f}")
    print(f"lat p95 {R['lat_p95']:.2f}, max {R['lat_max']:.2f}")
    print(f"roll gradient per run: {[f'{s:+.2f}' for s in R['rg_slopes']]}; "
          f"N {R['rgn']:+.2f} S+ {R['rgs']:+.2f}; corrected grip p95 "
          f"{R['lat_p95_corr']:.2f} max {R['lat_max_corr']:.2f}")


# -------------------------------------------------------- processed data
def _csv(path, header, rows, fmt=None):
    with open(path, "w") as fh:
        fh.write(",".join(header) + "\n")
        for row in rows:
            fh.write(",".join(fmt(v) if fmt else str(v) for v in row) + "\n")


def _num(v):
    if isinstance(v, (float, np.floating)):
        return f"{v:.6g}" if abs(v) >= 1e-3 or v == 0 else f"{v:.6e}"
    return str(v)


def enu_to_latlon(d, E, N):
    la0 = np.radians(d["lat"].mean())
    lo0 = np.radians(d["lon"].mean())
    return (np.degrees(la0 + N / R_E),
            np.degrees(lo0 + E / (R_E * np.cos(la0))))


def _rel(p):
    try:
        return p.relative_to(REPO)
    except ValueError:
        return p


def write_processed(R, out):
    out.mkdir(parents=True, exist_ok=True)
    d, runs, imu = R["d"], R["runs"], R["imu"]

    # runs.csv — one row per run, every per-run number the post uses
    rows = []
    for i, r in enumerate(runs):
        t_start_local = (d["t0"] + dt.timedelta(seconds=r["t_s"])).isoformat(
            timespec="milliseconds")
        rows.append([i + 1, MODES[i], t_start_local, r["t_s"], r["t_f"],
                     r["time"], R["vmax"][i] / MPH2MPS, R["lat"][i],
                     R["brake"][i], imu["roll"][i], imu["norm"][i],
                     R["rg_slopes"][i], imu["file"][i], imu["off"][i],
                     imu["xc"][i], imu["corr_ay"][i]])
    _csv(out / "runs.csv",
         ["run", "pasm_mode", "start_time_local", "t_start_s", "t_finish_s",
          "run_time_s", "vmax_mph", "peak_lat_g", "peak_brake_g",
          "roll_rate_rms_dps", "roll_rate_per_ay_rate_s",
          "roll_gradient_deg_per_g", "imu_file", "imu_clock_offset_s",
          "imu_envelope_xcorr", "imu_ay_vs_racebox_r"], rows, _num)

    # gates.csv — the calibrated virtual timing gates
    grows = []
    for name, (P, u), shift in [("start", R["gate_start"], START_SHIFT_M),
                                ("finish", R["gate_finish"], -FINISH_SHIFT_M)]:
        la, lo = enu_to_latlon(d, P[0], P[1])
        grows.append([name, f"{la:.7f}", f"{lo:.7f}", P[0], P[1], u[0], u[1],
                      shift, GATE_HALF_WIDTH_M])
    _csv(out / "gates.csv",
         ["gate", "lat_deg", "lon_deg", "east_m", "north_m",
          "course_dir_east", "course_dir_north", "calibration_shift_m",
          "half_width_m"], grows, _num)

    # imu_clock.csv — anchors + the per-file linear fit they produced
    fits = {f["imu_file"]: f for f in imu["fits"]}
    _csv(out / "imu_clock.csv",
         ["imu_file", "run", "t_mid_s", "offset_s", "envelope_xcorr",
          "fit_intercept_s", "fit_slope"],
         [[a["imu_file"], a["run"], a["t_mid_s"], a["offset_s"], a["xcorr"],
           fits[a["imu_file"]]["intercept_s"], fits[a["imu_file"]]["slope"]]
          for a in imu["anchors"]], _num)

    # per-run synchronized time series
    ay_kin = d["v"] * np.radians(d["yaw"]) / G0
    for i, r in enumerate(runs):
        lo, hi = r["t_s"] - EXPORT_PRE_S, r["t_f"] + EXPORT_POST_S
        m = (d["ts"] >= lo) & (d["ts"] <= hi)
        idx = np.flatnonzero(m)
        _csv(out / f"run{i+1}_racebox.csv",
             ["t_s", "time_local", "lat_deg", "lon_deg", "east_m", "north_m",
              "speed_mps", "a_long_g", "a_lat_g", "yaw_rate_dps",
              "a_lat_kin_g", "in_gates"],
             [[d["ts"][k],
               (d["t0"] + dt.timedelta(seconds=float(d["ts"][k]))).isoformat(
                   timespec="milliseconds"),
               f"{d['lat'][k]:.7f}", f"{d['lon'][k]:.7f}", d["E"][k],
               d["N"][k], d["v"][k], d["gx"][k], d["gy"][k], d["yaw"][k],
               ay_kin[k], int(r["t_s"] <= d["ts"][k] <= r["t_f"])]
              for k in idx], _num)
        t, acc, gyr = imu["windows"][i]
        _csv(out / f"run{i+1}_imu.csv",
             ["t_s", "ax_g", "ay_g", "az_g", "roll_rate_dps",
              "pitch_rate_dps", "yaw_rate_dps", "in_gates"],
             [[t[k], acc[k, 0], acc[k, 1], acc[k, 2], gyr[k, 0], gyr[k, 1],
               gyr[k, 2], int(r["t_s"] <= t[k] <= r["t_f"])]
              for k in range(len(t))], _num)

    # roll-gradient regression samples
    _csv(out / "roll_gradient_samples.csv",
         ["run", "pasm_mode", "t_s", "a_lat_kin_g", "roll_angle_deg"],
         R["rg_samples"], _num)

    # summary.json — every headline number, with definitions
    def mode_stats(vals):
        return {"normal_runs": [vals[i] for i in IDX_N],
                "sportplus_runs": [vals[i] for i in IDX_S],
                "normal_mean": float(np.mean([vals[i] for i in IDX_N])),
                "sportplus_mean": float(np.mean([vals[i] for i in IDX_S]))}
    summary = {
        "session": {
            "date": "2026-08-15", "venue": "Storm Stadium, Lake Elsinore, CA",
            "event": "SCCA Cal Club autocross (afternoon session)",
            "vehicle": "Porsche Macan S (95B), PASM adaptive dampers on steel springs, "
                       "Pirelli Scorpion Verde All Season",
            "racebox_t0_local": d["t0"].isoformat(timespec="milliseconds"),
            "pasm_by_run": MODES,
            "powertrain_mode": "Sport+ (constant)", "psm": "Sport",
            "enu_origin_lat_deg": float(d["lat"].mean()),
            "enu_origin_lon_deg": float(d["lon"].mean()),
        },
        "gates": {
            "start_shift_m": START_SHIFT_M, "finish_shift_m": -FINISH_SHIFT_M,
            "half_width_m": GATE_HALF_WIDTH_M,
            "calibration": "shifts fit to two owner-remembered official times "
                           "(run 5 ~52.0 s, run 6 ~51.1 s), rms 0.11 s",
        },
        "run_times_s": mode_stats([r["time"] for r in runs]),
        "grip_ceiling_g": {
            "definition": "|a_lat| from the roof RaceBox while cornering "
                          "(v > 8 m/s, |a_lat| > 0.3 g) inside the gates, all runs",
            "p95_raw": R["lat_p95"], "max_raw": R["lat_max"],
            "roll_leak_correction_deg_per_g": float(np.degrees(R["grad_rad"])),
            "p95_roll_corrected": R["lat_p95_corr"],
            "max_roll_corrected": R["lat_max_corr"],
            "registered_prediction_g": [0.75, 0.85],
        },
        "imu_clock": {
            "model": "wall_s = device_s + intercept + slope * t_mid_s, per SD file",
            "fits": imu["fits"], "offset_run1_s": imu["off_first"],
            "offset_run6_s": imu["off_last"],
            "envelope_xcorr_range": [min(imu["xc"]), max(imu["xc"])],
            "imu_ay_vs_racebox_r_range": [min(imu["corr_ay"]), max(imu["corr_ay"])],
        },
        "roll_rate_rms_dps": mode_stats(imu["roll"]),
        "roll_rate_per_ay_rate_s": mode_stats(imu["norm"]),
        "roll_gradient_deg_per_g": {
            **mode_stats(R["rg_slopes"]),
            "method": "roll = (a_lat_accel - v*yaw_rate/g) at quasi-steady samples, "
                      "RaceBox only; slope of a per-run linear fit vs v*yaw_rate/g",
            "quasi_steady_mask": QS_MASK_DEF,
            "n_samples": len(R["rg_samples"]),
        },
        "generator": "tools/day1_analysis.py",
    }
    (out / "summary.json").write_text(json.dumps(summary, indent=2) + "\n")
    print("processed ->", _rel(out))


def write_figures(R, out):
    out.mkdir(parents=True, exist_ok=True)
    use_palette("standalone")
    svgs = render_figures(R)
    for key, stem, num, title, note in FIGURE_SPECS:
        (out / f"{stem}.svg").write_text(
            f"<!-- FIG {num:02d} · {title} · {note} -->\n{svgs[key]}\n")
    print("figures ->", _rel(out))


# The post must link the public repository. The template carries the link
# in its intro and closing; if a template rewrite drops it, write_post()
# puts it back (before the first section, and as the closing sentence) and
# says so on stderr — a link-less post is never written silently.
REPO_URL = "https://github.com/adamlin1009/vd-macan"
REPO_INTRO = (
    "Everything below is reproducible. The raw session files, the\n"
    "processed per-run tables, and the analysis code are public at\n"
    f"[github.com/adamlin1009/vd-macan]({REPO_URL});\n"
    "one script regenerates every number and figure in this post from the\n"
    "raw logs.\n")
REPO_CLOSING = (
    "The raw session folder, the processed tables, and the code that\n"
    "produced this post are already public in the\n"
    f"[vd-macan repository]({REPO_URL}); the ride-block data joins it\n"
    "when that block runs.")
# Closing sentences seen in past template versions that should carry the
# link; the first that matches is replaced, otherwise REPO_CLOSING is
# appended as its own paragraph.
STALE_CLOSINGS = [
    re.compile(r"The raw session\s+folder\s+and the MATLAB\s+pipeline\s+publish"
               r"\s+with the\s+field report\."),
    re.compile(r"The raw session\s+folder\s+and the analysis code are public"
               r"\s+in the meantime\."),
]


def ensure_repo_link(post):
    if REPO_URL in post:
        return post
    print("WARNING: post template lost the repo link — injecting it",
          file=sys.stderr)
    i = post.index("\n## ")
    post = post[:i].rstrip("\n") + "\n\n" + REPO_INTRO + post[i:]
    for pat in STALE_CLOSINGS:
        if pat.search(post):
            return pat.sub(REPO_CLOSING, post)
    return post.rstrip("\n") + "\n\n" + REPO_CLOSING + "\n"


def write_post(R, path):
    d, runs, imu = R["d"], R["runs"], R["imu"]
    use_palette("site")
    svgs = render_figures(R)
    figs = {key: figure(svgs[key], num, title, note)
            for key, _, num, title, note in FIGURE_SPECS}
    rows = "\n".join(
        f"| {i+1} | {MODES[i]} | {runs[i]['time']:.2f} | "
        f"{R['vmax'][i]/MPH2MPS:.1f} | {R['lat'][i]:.2f} |"
        for i in range(6))
    rg = R["rg_slopes"]
    post = POST_TEMPLATE
    for key, val in {
        "@@TABLE@@": rows,
        "@@P95@@": f"{R['lat_p95']:.2f}",
        "@@MAX@@": f"{R['lat_max']:.2f}",
        "@@RN@@": f"{R['roll_n']:.2f}",
        "@@RS@@": f"{R['roll_s']:.2f}",
        "@@RGAIN@@": f"{100*(R['roll_s']/R['roll_n']-1):.0f}",
        "@@NRM_N@@": f"{imu['norm_n']:.2f}",
        "@@NRM_S@@": f"{imu['norm_s']:.2f}",
        "@@OFF1@@": f"{imu['off_first']:.0f}",
        "@@OFF6@@": f"{imu['off_last']:.0f}",
        "@@XCLO@@": f"{min(imu['xc']):.2f}",
        "@@XCHI@@": f"{max(imu['xc']):.2f}",
        "@@AYHI@@": f"{max(imu['corr_ay']):.2f}",
        "@@RGN@@": f"{R['rgn']:.2f}",
        "@@RGS@@": f"{R['rgs']:.2f}",
        "@@RGNLO@@": f"{min(rg[i] for i in IDX_N):.2f}",
        "@@RGNHI@@": f"{max(rg[i] for i in IDX_N):.2f}",
        "@@RGSLO@@": f"{min(rg[i] for i in IDX_S):.2f}",
        "@@RGSHI@@": f"{max(rg[i] for i in IDX_S):.2f}",
        "@@P95C@@": f"{R['lat_p95_corr']:.2f}",
        "@@MAXC@@": f"{R['lat_max_corr']:.2f}",
        **{f"@@FIG_{k}@@": v for k, v in figs.items()},
    }.items():
        post = post.replace(key, val)
    assert "@@" not in post, "unfilled template token"
    post = ensure_repo_link(post)
    Path(path).write_text(post)
    print("wrote", path, f"({len(post)/1024:.0f} KB)")


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--processed", default=str(PROCESSED),
                    help="output dir for processed tables (default: %(default)s)")
    ap.add_argument("--figures", default=str(FIGURES),
                    help="output dir for standalone SVG figures (default: %(default)s)")
    ap.add_argument("--post", default=None,
                    help="also write the assembled log post to this path")
    ap.add_argument("--no-write", action="store_true",
                    help="compute and print only")
    args = ap.parse_args()

    R = analyze()
    print_summary(R)
    if args.no_write:
        return
    write_processed(R, Path(args.processed))
    write_figures(R, Path(args.figures))
    if args.post:
        write_post(R, args.post)


POST_TEMPLATE = r"""---
title: "Six runs, two damper maps: first data from Storm Stadium"
date: "2026-08-16"
summary: "About $300 of loggers, one autocross, six runs alternating PASM Normal and Sport+. The data caught me being wrong three times before it told me anything about the car."
---

You know that moment when you finally sit down with data you've been
waiting weeks to collect? Saturday night I had two loggers' worth
from my first instrumented autocross, a cold drink, and big plans.

The data had other ideas. Before it told me one thing about the car,
it caught me being wrong three times. And honestly, that turned out
to be the best part.

So here's the whole story: six runs, two damper maps, a clock that
flat-out lied to me, and a grip prediction I got to watch die in
public.

## The setup

The experiment is almost embarrassingly simple. My Macan has a
button. Press it and PASM swaps between a Normal damper map and a
Sport+ one. Same springs, same anti-roll bars, same everything else.
Just software deciding how hard the dampers push back.

So I alternated it every single run. Normal, Sport+, Normal, Sport+,
Normal, Sport+. Six runs in the afternoon session at Storm Stadium
(SCCA Cal Club, Lake Elsinore), powertrain locked in Sport+, PSM in
Sport. The damper button was the only thing I touched all afternoon.

Housekeeping, because it matters later: cold pressures set to placard
37/40 with my reference gauge. The TPMS read 36/39 at the start, a
consistent 1 psi low on both axles, which is fine. Offsets cancel. By
the end of the session it read 40/42, and fuel went from about 5/8
tank to 1/2.

Two loggers rode along: the RaceBox on the roof catching every run at
25 Hz, and the AHRS IMU on the center console writing 200 Hz to its
own storage. Two independent loggers means two independent clocks,
and keeping them in step turned out to be its own small adventure.
Luckily each run writes its own sync signature: a launch spike and
then fifty-six seconds of unmistakable car dynamics. Why that
mattered so much is Wrong #3.

## Wrong #1: the "200 Hz" sensor isn't one

Quick detour on why sample rates matter, because this is the part
people skip. Sampling theory is blunt: you only get to see content
below half your sample rate,

$$f_\mathrm{visible} = \tfrac{1}{2} f_\mathrm{sample}.$$

Secondary ride, the busy patter that makes a car feel firm, lives at
roughly 4 to 25 Hz. The RaceBox samples at 25 Hz, so it tops out at
12.5 Hz. It literally cannot see the top half of the thing I care
about most. That's the whole reason the IMU exists in this project.

So before trusting the IMU, I characterized it. And found the
brochure number:

```
frames written to storage:      200 per second  (exact, clean)
distinct accelerometer values:  ~104 per second
distinct gyro values:           ~50 per second
```

Look at that middle line. The sensor writes 200 frames a second,
sure. But its fusion loop only produces about 104 new accelerometer
values a second, so nearly half those frames are copies of the one
before. It still clears my requirement, since 104 Hz covers the 4 to
25 Hz band four times over. But it's not what the listing says. **A
spec sheet tells you what a sensor writes, not what it measures.**
The ingest code drops the duplicates before any spectrum gets
computed.

## Timing runs without timing equipment

Here's a fun problem. The event has timing lights, but their numbers
live on a results sheet, not in my data. What I have is GPS. And GPS,
if you ask it the right way, is timing equipment.

Think about what a run looks like in the data. Every launch is a hard
acceleration from a dead stop. Every finish is one last big braking
event. So I built virtual gates out of the six trajectories
themselves: a start line where speed first crosses 5 m/s, and a
finish line just before the terminal braking. The six launch points
landed within 0.7 m of each other. Drivers stage more consistently
than I expected. That one genuinely surprised me.

Then I calibrated the pair against the two official times I
remembered. The fit barely moved the start line. But it pulled the
finish 23 m before the braking point, and once you see it, it's
obvious: at 22 m/s you cross the lights flat-out and don't touch the
brakes for another full second. After calibration the gates reproduce
the officials to about a tenth.

@@FIG_MAP@@

## The runs

| run | PASM | time [s] | vmax [mph] | peak lat [g] |
|---|---|---|---|---|
@@TABLE@@

Sport+ holds the day's best time, and the mode averages sit about
half a second apart. Run 4 was the slow one, and that one's on me.
Driving mistake. Not the dampers.

But look at the shape of this. The margin between modes is tenths on
a 52-second course, and from the seat they felt like completely
different cars. **"Feels transformed" and "barely faster" can both be
true at once.** That gap between what you feel and what the clock
says? Closing it is the whole reason this project exists.

@@FIG_TIMES@@

@@FIG_SPEED@@

## Wrong #2: the grip prediction is dead. Good.

Back before any data existed, I put a number on the record: a
2.2-ton-class SUV on touring all-seasons should top out around
0.75–0.85 g. It felt safe. Maybe even a little generous.

Measured, cornering hard across all six runs: 95th percentile
@@P95@@ g. Peak @@MAX@@ g.

Not missed. Demolished. And per this project's standing rules, the
dead prediction stays right here in the text where everyone can see
it. **Register your predictions before the data exists. Being wrong
on the record is the fastest calibration a person can get.**

Two caveats before anyone quotes those numbers at a tire shop. The
RaceBox sits on the roof, so body roll tilts it and leaks a slice of
gravity into the lateral channel. I'll measure and correct that
below, once I've actually measured the roll. And an autocross rewards
brief peaks, not skidpad steady-state.

@@FIG_GG@@

## Wrong #3: the clock that lied by two percent

My first pass at the IMU file said the sensor sat perfectly still
through every single run window. Flat. Nothing. For one genuinely bad
hour, the working theory was that it had spent the day in the paddock
while I drove.

It hadn't. It rode the console all session, and its own file proves
it: six unmistakable 56-second bursts of car dynamics, spaced exactly
like the run schedule. They just weren't where the timestamps said
they'd be.

Here's the failure mode, written out. File time relates to true time
through an offset that drifts,

$$t_\mathrm{GPS} = t_\mathrm{IMU} + \mathrm{OFF}(t), \qquad
\mathrm{OFF}(t) = a + b\,t,$$

and the fitted drift came out around \( b \approx 20 \) ms per
second. That's the two percent. The offset was @@OFF1@@ s at run 1
and @@OFF6@@ s by run 6. Fifty-one seconds of drift in one afternoon.
The IMU's tick is beautifully regular, 5.000 ms per frame relative to
itself. It's just that its whole sense of "now" runs slow, and every
power cycle resets the starting error. My analysis windows were
slicing through empty paddock time while the actual runs sat two
minutes away in file time.

The fix cost nothing. Cross-correlate the acceleration envelope
against the RaceBox, fit the \( \mathrm{OFF}(t) \) line per file,
done. Per-run correlation came out @@XCLO@@ to @@XCHI@@ with
residuals inside ±130 ms. And with the clock fixed, everything
snapped into place: IMU lateral tracks GPS lateral at
\( r = @@AYHI@@ \), longitudinal tracks longitudinal, yaw tracks yaw.

**Never trust a logger's clock you haven't measured against GPS. And
look for the sync signal already hiding in your data.** Both lessons
are on the per-session checklist now, written in the tone of someone
who learned them the hard way.

## Did the seat tell the truth? Rate isn't angle

What I remembered from the seat, written down the day after (the
per-run blind sheets fell to the event-day rush, so this is memory
and I'm labeling it as memory): Normal had way more body roll, pitch,
and yaw. Sport+ felt planted and reactive.

First channel that can referee: the roll-rate gyro.

@@FIG_ROLL@@

And at first glance the gyro says I'm wrong. Roll-rate RMS is
*higher* in Sport+, not lower: @@RS@@ °/s against @@RN@@ °/s, about
@@RGAIN@@% more. Normalize by how hard the car was actually being
driven, \( \mathrm{RMS}(p)\,/\,\mathrm{RMS}(\dot a_y) \), and it's a
dead wash: @@NRM_N@@ against @@NRM_S@@.

So was the seat lying? No. The cleanest way to see it is to just
write the roll dynamics down. One equation carries the whole
argument:

$$K_\varphi\,\varphi + C_\varphi\,\dot\varphi = m\,h_s\,a_y$$

where \( \varphi \) is the roll angle, \( K_\varphi \) is the roll
stiffness (springs and anti-roll bars, stock, fixed),
\( C_\varphi \) is the roll damping (the dampers, the one thing PASM
touches), and \( m\,h_s\,a_y \) is the overturning moment from
cornering. Now watch what happens in a steady corner. Steady means
\( \dot\varphi = 0 \), so

$$\varphi_\mathrm{ss} = \frac{m\,h_s}{K_\varphi}\,a_y$$

and \( C_\varphi \) just... vanishes. The damper term multiplies roll
*rate*, so once the car settles into a corner, the dampers have no
vote in how far it leans. **PASM can't change this car's lean. It can
only change how the lean happens.**

Which reframes everything. "Body roll" was two claims wearing one
phrase: roll angle, which is spring territory, and roll rate, which
is damper territory. "Planted and reactive" is a rate feeling, and a
tighter car makes *more* roll rate, not less. The gyro agreed with
the seat the whole time. I was just asking it the wrong question.

One real confound stays attached: I drove the Sport+ runs harder.
Separating "the car responds faster" from "the driver asked for more"
needs matched inputs, and that lands in the next-steps list.

## Wringing the dataset: five derivations, one survivor

With the clock solved I got greedy, honestly. Five derived analyses,
everything I could think to squeeze out of two loggers. One produced
a number I'll stand behind. Four died in ways worth writing down.

The survivor is my favorite trick of the whole weekend: a roll sensor
built out of disagreement. Here's the idea. An accelerometer bolted
to the body reads cornering force plus a slice of gravity leaking
through the roll angle. GPS speed times gyro yaw rate reads cornering
force alone, no gravity anywhere in it:

$$a_\mathrm{meas} \approx a_y + g\sin\varphi \approx a_y + g\,\varphi,
\qquad a_\mathrm{kin} = v\,\dot\psi.$$

Two channels that should agree, and the gap between them *is* the
roll angle:

$$\varphi \approx \frac{a_\mathrm{meas} - a_\mathrm{kin}}{g}.$$

The roll gradient is then just the slope \( G = d\varphi/da_y \) in
degrees per g. Both channels come from the same device, so the clock
saga can't touch this one. Sample it where cornering is quasi-steady,
regress run by run:

@@FIG_RGRAD@@

And the result genuinely surprised me. **Normal: @@RGN@@ °/g**
(per-run @@RGNLO@@ to @@RGNHI@@). **Sport+: @@RGS@@ °/g** (@@RGSLO@@
to @@RGSHI@@). The ranges don't even touch. Every Normal run leaned
more per g than every Sport+ run.

But wait. Didn't I just derive, two sections ago, that the
steady-state gradient can't split on stock springs? Exactly. And
that's the tell: an autocross never gives you truly steady samples.
Even the calmest windows carry some leftover \( \dot\varphi \), and
that's the term the dampers own. The ~0.8 °/g split is
\( C_\varphi \) bleeding into a measurement that was supposed to
exclude it. It's the seat's "Normal leans more," made visible. Two
fingerprints back that reading: loosen the steadiness filter and the
split shrinks toward overlap, and the binned means bow steeper at
high g, meaning the relation is progressive and a single slope is a
summary. The genuinely steady number needs a constant-radius test.
Next-steps list.

The gradient pays off the grip section's promise too. The roof
accelerometer over-reads lateral g through the same gravity-leak
mechanism, so with \( G \approx 2.5 \) °/g, or
\( G_\mathrm{rad} \approx 0.044 \),

$$a_\mathrm{true} \approx \frac{a_\mathrm{meas}}{1 + G_\mathrm{rad}}$$

which works out to about a 4% haircut: @@P95@@ g becomes **@@P95C@@
g** sustained, and @@MAX@@ g becomes **@@MAXC@@ g** peak. Still
comfortably past my prediction. It stays dead.

The graveyard, with causes of death:

- **Roll transfer function per mode.** Coherence between lateral
  input and roll rate never reached 0.6 at any frequency. On a
  course, the road excites roll as much as the driver does, and the
  estimator starves.
- **Launch and brake pitch transients.** Driver variance swamps the
  mode signal, and the roof lever-arm contaminates the accelerometer
  during pitch transients.
- **Dive and squat gradients.** Autocross braking is a two-second
  ramp, never quasi-steady. Correlations near zero.
- **Repeated-bump ringdowns.** Storm Stadium's lot is smooth. Three
  vertical events all session, none recurring.

**Negative results are results. Write them down.** Every one of these
failures points at the same missing ingredient, and it's not cleverer
math. It's controlled inputs.

## Steal these

If you're thinking about instrumenting your own car, and honestly you
should be, here's what a weekend of being wrong taught me:

- Characterize the instrument before the experiment.
- Register predictions before data exists.
- The sensor doesn't measure what it isn't bolted to.
- Never trust an unmeasured clock.
- When clever math dies, you need controlled inputs, not more math.

## Next steps and improvements

Here's the list, roughly ordered by value per effort:

- **More runs.** Statistics fix half of day one's weaknesses for
  free. Every future event day doubles as a data day.
- **Fill the blind rating sheets, per run, before looking at
  anything.** The subjective side deserves the same rigor as the
  channels, and this weekend it didn't get it.
- **A controlled-input session when lot space materializes.**
  Fixed-speed step-steers for the roll and yaw transients per mode,
  constant-radius or spiral ramps for the true steady-state gradient
  (the number FIG 06 only approximates), and repeated passes
  over one bump for a heave damping ratio per mode. Everything the
  graveyard wants lives here.
- **Re-sync the IMU clock at every power-on**, or keep trusting
  envelope correlation. Either works now that the failure mode is
  known and measured.

Then the quarter-car fit and the semi-active study, once
controlled-input data exists to feed them. And you don't have to take
my word for any of this: the raw session files, the processed
per-run tables, and the code that produced every number and figure in
this post are public in the
[vd-macan repository](https://github.com/adamlin1009/vd-macan).
One script regenerates the whole post from the raw logs.
"""


if __name__ == "__main__":
    main()
