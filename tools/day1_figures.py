#!/usr/bin/env python3
"""Build the day-1 log post: analysis + inline-SVG figures + assembled md.

Reads the staged afternoon session (racebox.csv + imu_sd/WIT38-41),
recomputes gates with the calibrated shifts, measures the IMU clock
offset against the RaceBox by envelope cross-correlation, computes
per-run per-mode metrics, and writes the complete post to the website
repo. All colors validated (dataviz six checks, dark surface #14171c):
Normal #4d7fc9, Sport+ #a9631c (the site speed-ramp ember anchor).

Usage: python3 day1_figures.py [--out <website>/content/log/<slug>.md]
"""
import argparse
import datetime as dt
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from tap_check import parse_flag61, parse_standard, ACC_SCALE, GYR_SCALE  # noqa: E402

SESSION = HERE.parent / "data" / "20260815_afternoon"
MPH2MPS = 0.44704
R_E = 6371000.0

# calibrated gate shifts (fit to owner-remembered official times, rms 0.11 s)
START_SHIFT_M = 2.0
FINISH_SHIFT_M = 23.0

MODES = ["Normal", "Sport+", "Normal", "Sport+", "Normal", "Sport+"]
C_N = "#4d7fc9"          # Normal  (validated)
C_S = "#a9631c"          # Sport+  (validated; site ramp ember)
INK = "var(--ink)"
DIM = "var(--ink-dim)"
GRID = "var(--grid)"
STRONG = "var(--grid-strong)"
SURF = "var(--panel-deep)"
MONO = "font-family:var(--font-data),monospace;"

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

    def crossing(P, u, lo, hi, vmin, width=12.0):
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
def load_imu():
    rows = []
    for name in ["WIT38.TXT", "WIT39.TXT", "WIT40.TXT", "WIT41.TXT"]:
        buf = (SESSION / "imu_sd" / name).read_bytes()
        r = parse_flag61(buf)
        if r is None:
            r, _ = parse_standard(buf)
        rows += [x for x in r if x["t"] is not None]
    rows.sort(key=lambda x: x["t"])
    acc = np.array([r["acc"] for r in rows], int)
    gyr = np.array([r["gyr"] for r in rows], int)
    keep = np.concatenate(([True], (np.diff(acc, axis=0) != 0).any(axis=1)
                           | (np.diff(gyr, axis=0) != 0).any(axis=1)))
    t = np.array([r["t"] for r in rows])[keep]
    return t, acc[keep] * ACC_SCALE, gyr[keep] * GYR_SCALE


# ------------------------------------------------------------- svg utils
W = 760


def svg_open(h, label):
    return (f'<svg viewBox="0 0 {W} {h}" role="img" aria-label="{label}" '
            f'style="display:block;width:100%;height:auto">')


def txt(x, y, s, size=10, fill=DIM, anchor="start", extra="", halo=False):
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


def analyze_imu(d, runs):
    """Per-file clock fit (envelope xcorr vs RaceBox) + per-run metrics.

    The IMU's tick is a clean 5 ms RELATIVE, but absolute rate runs ~2%
    slow and the offset re-arms each power-on: fit OFF(t)=a+b*t per file
    from per-run cross-correlation, then window with the corrected clock.
    """
    out = {"roll": [], "corr_ay": [], "off_first": None, "off_last": None,
           "xc": []}
    rb_env_t = d["ts"]
    rb_env = np.hypot(d["gx"], d["gy"])
    for name, runidx in [("WIT39.TXT", [0, 1, 2, 3]),
                         ("WIT40.TXT", [4, 5])]:
        rows = [r for r in parse_flag61(
            (SESSION / "imu_sd" / name).read_bytes()) if r["t"]]
        pt = np.array([(r["t"] - d["t0"]).total_seconds() for r in rows])
        acc = np.array([r["acc"] for r in rows], float) * ACC_SCALE
        gyr = np.array([r["gyr"] for r in rows], float) * GYR_SCALE
        paxy = np.hypot(acc[:, 0], acc[:, 1])
        anchors = []
        for ri in runidx:
            s, f = runs[ri]["t_s"], runs[ri]["t_f"]
            grid = np.arange(s - 15, f + 15, 0.2)
            rbv = np.interp(grid, rb_env_t, rb_env)
            best, boff = -9, None
            for OFF in np.arange(100, 220, 0.2):
                pv = np.interp(grid - OFF, pt, paxy, left=0, right=0)
                c = np.corrcoef(rbv, pv)[0, 1]
                if c > best:
                    best, boff = c, OFF
            anchors.append((0.5 * (s + f), boff))
            out["xc"].append(best)
        x = np.array([a[0] for a in anchors])
        y = np.array([a[1] for a in anchors])
        b, a = np.polyfit(x, y, 1) if len(x) > 1 else (0.0, y[0])
        for ri in runidx:
            s, f = runs[ri]["t_s"], runs[ri]["t_f"]
            OFF = a + b * 0.5 * (s + f)
            m = (pt >= s - OFF) & (pt <= f - OFF)
            out["roll"].append(float(np.sqrt(np.mean(gyr[m, 0] ** 2))))
            grid = np.arange(s + 1, f - 1, 0.2)
            pay = np.interp(grid, pt[m] + OFF, acc[m, 1])
            rgy = np.interp(grid, rb_env_t, d["gy"])
            out["corr_ay"].append(float(np.corrcoef(pay, rgy)[0, 1]))
            if out["off_first"] is None:
                out["off_first"] = OFF
            out["off_last"] = OFF
    # roll rate normalized by lateral-accel rate, per mode
    norm = []
    for i, r in enumerate(runs):
        m = (d["ts"] >= r["t_s"]) & (d["ts"] <= r["t_f"])
        aydot = np.sqrt(np.mean(np.gradient(d["gy"][m], d["ts"][m]) ** 2))
        norm.append(out["roll"][i] / aydot)
    out["norm_n"] = float(np.mean([norm[i] for i in (0, 2, 4)]))
    out["norm_s"] = float(np.mean([norm[i] for i in (1, 3, 5)]))
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
    ay_kin = d["v"] * np.radians(d["yaw"]) / 9.81
    slopes, pts = [], {"Normal": [], "Sport+": []}
    for ri, r in enumerate(runs):
        g = np.arange(r["t_s"] + 1, r["t_f"] - 1, 0.04)
        ayk = np.interp(g, d["ts"], ay_kin)
        aym = np.interp(g, d["ts"], d["gy"])
        v = np.interp(g, d["ts"], d["v"])
        gx = np.interp(g, d["ts"], d["gx"])
        day = np.gradient(ayk, g)
        qs = ((np.abs(ayk) > 0.30) & (np.abs(day) < 0.30)
              & (v > 8) & (np.abs(gx) < 0.25))
        phi = np.degrees(aym[qs] - ayk[qs])
        slopes.append(float(np.polyfit(ayk[qs], phi, 1)[0]))
        pts[MODES[ri]] += list(zip(ayk[qs], phi))
    return slopes, pts


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


# ------------------------------------------------------------------ main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default=str(Path.home() / "Documents/website/content/log/2026-08-16-six-runs-two-damper-maps.md"))
    args = ap.parse_args()

    d = load_racebox()
    runs, g0, g1 = find_runs(d)
    print("runs:", [f"{r['time']:.2f}s" for r in runs])

    pk = analyze_imu(d, runs)
    print("roll RMS deg/s:", [f"{x:.2f}" for x in pk["roll"]])
    print(f"clock offsets {pk['off_first']:.0f} -> {pk['off_last']:.0f} s; "
          f"xcorr {min(pk['xc']):.2f}-{max(pk['xc']):.2f}; "
          f"ay corr {min(pk['corr_ay']):+.2f}..{max(pk['corr_ay']):+.2f}; "
          f"norm N {pk['norm_n']:.2f} vs S+ {pk['norm_s']:.2f}")

    lat = [float(np.abs(d["gy"][r["a"]:r["b"]]).max()) for r in runs]
    corner = (d["v"] > 8) & (np.abs(d["gy"]) > 0.3)
    in_runs = np.zeros(len(d["ts"]), bool)
    for r in runs:
        in_runs |= (d["ts"] >= r["t_s"]) & (d["ts"] <= r["t_f"])
    lat_p95 = float(np.percentile(np.abs(d["gy"][corner & in_runs]), 95))
    lat_max = float(np.abs(d["gy"][corner & in_runs]).max())
    print(f"lat p95 {lat_p95:.2f}, max {lat_max:.2f}")

    rg_slopes, rg_pts = roll_gradient(d, runs)
    rgn = float(np.mean([rg_slopes[i] for i in (0, 2, 4)]))
    rgs = float(np.mean([rg_slopes[i] for i in (1, 3, 5)]))
    grad_rad = float(np.radians(0.5 * (rgn + rgs)))
    print(f"roll gradient per run: {[f'{s:+.2f}' for s in rg_slopes]}; "
          f"N {rgn:+.2f} S+ {rgs:+.2f}; corrected grip p95 "
          f"{lat_p95/(1+grad_rad):.2f} max {lat_max/(1+grad_rad):.2f}")

    figs = {
        "MAP": figure(fig_map(d, runs, g0, g1), 1,
                      "RUN 6 · GPS PATH, SPEED-COLORED · VIRTUAL GATES",
                      "RACEBOX MINI S · 25 HZ"),
        "TIMES": figure(fig_times(runs), 2,
                        "RUN TIMES BY PASM MODE · GATES CALIBRATED TO OFFICIAL TIMES",
                        "STORM STADIUM · 2026-08-15"),
        "SPEED": figure(fig_speed(d, runs), 3,
                        "SPEED VS COURSE DISTANCE · BEST RUN PER MODE",
                        "RACEBOX MINI S · 25 HZ"),
        "GG": figure(fig_gg(d, runs), 4,
                     "G-G DIAGRAM BY PASM MODE · ALL SIX RUNS",
                     "ROOF-MOUNTED · UNCORRECTED FOR BODY ROLL"),
        "ROLL": figure(fig_roll(pk["roll"]), 5,
                       "ROLL-RATE RMS PER RUN · CONSOLE-MOUNTED IMU",
                       "WT901SDCL-BT50 · CLOCK SYNCED TO GPS"),
        "RGRAD": figure(fig_rollgrad(rg_slopes, rg_pts), 6,
                        "ROLL ANGLE VS LATERAL G · QUASI-STEADY SAMPLES",
                        "RACEBOX ACCEL MINUS V×YAW · SELF-CONSISTENT"),
    }

    rows = "\n".join(
        f"| {i+1} | {MODES[i]} | {runs[i]['time']:.2f} | "
        f"{max(d['v'][runs[i]['a']:runs[i]['b']])/MPH2MPS:.1f} | {lat[i]:.2f} |"
        for i in range(6))

    rn = float(np.mean([pk["roll"][i] for i in (0, 2, 4)]))
    rs = float(np.mean([pk["roll"][i] for i in (1, 3, 5)]))
    post = POST_TEMPLATE
    for key, val in {
        "@@TABLE@@": rows,
        "@@P95@@": f"{lat_p95:.2f}",
        "@@MAX@@": f"{lat_max:.2f}",
        "@@RN@@": f"{rn:.2f}",
        "@@RS@@": f"{rs:.2f}",
        "@@RGAIN@@": f"{100*(rs/rn-1):.0f}",
        "@@NRM_N@@": f"{pk['norm_n']:.2f}",
        "@@NRM_S@@": f"{pk['norm_s']:.2f}",
        "@@OFF1@@": f"{pk['off_first']:.0f}",
        "@@OFF6@@": f"{pk['off_last']:.0f}",
        "@@XCLO@@": f"{min(pk['xc']):.2f}",
        "@@XCHI@@": f"{max(pk['xc']):.2f}",
        "@@AYHI@@": f"{max(pk['corr_ay']):.2f}",
        "@@RGN@@": f"{rgn:.2f}",
        "@@RGS@@": f"{rgs:.2f}",
        "@@RGNLO@@": f"{min(rg_slopes[i] for i in (0,2,4)):.2f}",
        "@@RGNHI@@": f"{max(rg_slopes[i] for i in (0,2,4)):.2f}",
        "@@RGSLO@@": f"{min(rg_slopes[i] for i in (1,3,5)):.2f}",
        "@@RGSHI@@": f"{max(rg_slopes[i] for i in (1,3,5)):.2f}",
        "@@P95C@@": f"{lat_p95/(1+grad_rad):.2f}",
        "@@MAXC@@": f"{lat_max/(1+grad_rad):.2f}",
        **{f"@@FIG_{k}@@": v for k, v in figs.items()},
    }.items():
        post = post.replace(key, val)
    assert "@@" not in post, "unfilled template token"
    Path(args.out).write_text(post)
    print("wrote", args.out, f"({len(post)/1024:.0f} KB)")


POST_TEMPLATE = """---
title: "Six runs, two damper maps: first data from Storm Stadium"
date: "2026-08-16"
summary: "Day one of the venue weekend put the instrumentation plan through first contact with reality: six autocross runs alternating PASM Normal and Sport+, a grip ceiling that cleared my registered prediction by a wide margin, and one instrumentation lesson learned the honest way."
---

The plan said: enter an autocross weekend, alternate the damper modes
run by run, and let the loggers ride along. Day one at Storm Stadium
(SCCA Cal Club, Lake Elsinore) did exactly that. Six runs in the
afternoon session, PASM alternating Normal / Sport+ starting with
Normal, powertrain fixed in Sport+ the whole time, PSM in Sport. Cold
pressures set to placard 37/40 with the reference dial gauge before the
session; the TPMS read 36/39 at the start (a consistent −1 psi against
the gauge on both axles) and climbed to 40/42 by the end. Fuel ran from
about 5/8 to 1/2 tank.

The RaceBox rode the roof at 25 Hz and caught every run; the AHRS IMU
rode the center console, on the cupholder perimeter — though getting
its data to admit it was aboard took a fight I'll get to. The plan's
brake-jab sync ritual didn't happen — the day moved fast — but it
turned out each run writes its own sync signature anyway: a launch
spike and fifty-six seconds of unmistakable car dynamics.

## What the instruments turned out to be

Shakedown honesty, because the numbers only mean something if the
instrument is characterized. The IMU writes frames at exactly 200 Hz
on a clean 5 ms clock — but underneath, its fusion loop updates the
accelerometer at about 104 Hz and the gyro at about 50 Hz, so roughly
half the frames repeat the previous values. That still clears the
plan's requirement (≥100 Hz to onboard storage, which covers the
4–25 Hz secondary-ride band twice over), but "200 Hz logger" would be
the brochure number, and this log doesn't do brochure numbers. The
ingest code de-duplicates to the true effective rate before any
spectrum gets computed. The clock needed the same treatment: the
frame-to-frame tick is a metronomic 5 ms, but the absolute rate runs
about two percent slow and the offset re-arms at every power-on — what
that nearly cost me is below.

## Timing the runs without timing equipment

The course is point-to-point and the event's timing lights aren't in my
data — but the GPS is. Each launch shows up as a hard acceleration from
standstill and each finish as the run's last sustained braking, so I
built virtual gates from the six trajectories: the start anchor is
where speed first crosses 5 m/s after launch (the six anchors landed
within 0.7 m of each other), and the finish anchor sits just before the
terminal braking (4.4 m spread). Two remembered official times
calibrated the pair of gates along the course — the fit put the start
beam almost exactly at my geometric anchor and the finish 23 m earlier
than the braking point, which is just the reminder that you cross the
lights flat-out and brake afterward. After calibration the gate times
reproduce the officials to about a tenth.

@@FIG_MAP@@

## The runs

| run | PASM | time [s] | vmax [mph] | peak lat [g] |
|---|---|---|---|---|
@@TABLE@@

Sport+ holds the day's best time and the mode averages sit about half
a second apart — though run 4 broke the pattern, coming in slower
than the Normal runs on either side of it, and three runs per mode is
thin statistics. Either way the margin is tenths on a 52-second
course, while from the seat the two modes felt like different cars.
That tension between "feels transformed" and "barely faster" is the
whole reason this project pairs subjective ratings with instruments.

@@FIG_TIMES@@

@@FIG_SPEED@@

## The grip ceiling prediction is dead — good

The plan registered a prediction before any data existed: a
2.2-ton-class SUV on touring all-seasons should top out around
0.75–0.85 g. Measured, while cornering hard across all six runs: 95th
percentile @@P95@@ g, peak @@MAX@@ g. The prediction wasn't just
missed, it was cleared with room to spare — and per this project's
standing rules, it stays in the text. Two honest caveats ride along:
the RaceBox sits on the roof, so body roll leaks a few percent of
gravity into its lateral channel (the IMU's roll angle will correct
that before any number gets called final), and an autocross course
rewards brief peaks, not skidpad steady-state.

@@FIG_GG@@

## The clock that lied by two percent

My first pass at the IMU's file said the sensor sat level and still
through every single run window — for one bad hour the working theory
was that it had spent the day in the paddock. It hadn't. It rode the
console the whole session, and its own file proves it: six
unmistakable ~56-second bursts of car dynamics, spaced exactly like
the run schedule — just not where the timestamps said they should be.
The IMU's clock ticks a clean 5 ms per frame relative to itself, but
its absolute rate runs about two percent slow, and every power-cycle
re-arms the offset: by run 1 the file clock trailed GPS time by
@@OFF1@@ seconds, and by run 6, @@OFF6@@. Cross-correlating the
acceleration envelope against the RaceBox pins each file's clock to
GPS (per-run correlation @@XCLO@@–@@XCHI@@, residuals inside
±130 ms), and with the clock fixed the mount checks out end to end —
IMU lateral tracks GPS lateral at r = @@AYHI@@, longitudinal tracks
longitudinal, yaw tracks yaw. Two lines joined the per-session card:
confirm the IMU is aboard and flashing, and never trust a logger's
clock you haven't measured against GPS. The plan's brake-jab ritual —
which never actually happened on day one — exists for exactly this
failure mode; course runs turned out to carry a fingerprint loud
enough to sync on without it. It stays in the protocol for recordings
that don't, like the ride block's repeated passes.

## Did the seat of the pants tell the truth? Not so fast

My recollection, written down the day after (the per-run blind sheets
fell to the event-day rush, so this is memory and labeled as such):
Normal had a lot more body roll, pitch, and yaw; Sport+ felt a lot
more planted and reactive. With the clock fixed, the first channel
that can referee is the IMU's roll-rate gyro, run by run:

@@FIG_ROLL@@

The result refuses to flatter the impression — and then rewards a
closer reading of it. Raw roll-rate RMS is higher in Sport+, not
lower: @@RS@@ °/s against @@RN@@ °/s, about @@RGAIN@@% more, and
normalizing by how hard the car was actually being driven (roll rate
per unit of lateral-acceleration rate) lands at a wash — @@NRM_N@@ in
Normal, @@NRM_S@@ in Sport+. But "body roll" was always two claims
wearing one phrase. The lean the seat complains about is roll *angle*
— how far the body heels over and how it settles — and a rate gyro
doesn't measure that, while a 6-axis IMU's fused angle can't honestly
separate lean from sustained lateral acceleration mid-corner. The
*rate* side is where "planted and reactive" lives, and there the
number makes physical sense — doubly so on this car. The steady-state
lean budget is set by springs, anti-roll bars, track width, and CG
height, all stock here and none of them touched by PASM; a damper
can't hold the body up, it can only decide how fast the lean develops
and whether it overshoots on the way. So the mode switch shouldn't
much change how far this car rolls once settled — it changes how the
roll *happens*, and a firmer map that makes the body track its inputs
faster generates *more* roll rate, not less. That is the roll version
of this plan's registered prediction: the modes should separate in
the transients, not the steady-state numbers. Higher rate RMS in
Sport+ is consistent with exactly that, tangled with the other honest
explanation that I drove the Sport+ runs harder. Separating "the car responds faster" from "the driver
asked for more" takes matched inputs — step-steers at fixed speed,
and the ride block, where the road does the asking — and the
angle-domain lean question waits for the same. Carrying a sensation
across to the right number is the discipline this project exists to
practice; this is what it looks like mid-translation.

## Wringing the dataset: five derivations, one survivor

With the clock solved I went hunting for everything else these two
loggers could be made to say. Five derived analyses; one produced a
number I'll stand behind, and four failed in ways worth recording.

The survivor is a roll sensor built out of disagreement. The
accelerometer measures true lateral acceleration *plus* gravity
leaking through the roll angle; speed times yaw rate measures true
lateral acceleration alone. Their difference, at quasi-steady
cornering samples, is the roll angle — no roll sensor involved, one
device, immune to the clock saga. Regressed against lateral g, run by
run:

@@FIG_RGRAD@@

The result surprised me. **Normal: @@RGN@@ °/g** (per-run
@@RGNLO@@–@@RGNHI@@); **Sport+: @@RGS@@ °/g** (@@RGSLO@@–@@RGSHI@@)
— the ranges don't touch: every Normal run leaned more per g than
every Sport+ run. Read carefully, though, because a true steady-state
gradient *cannot* split on stock springs and bars — that's the whole
argument above. What this measures is that an autocross never offers
truly steady samples: even the calmest windows carry residual roll
rate, and roll rate is exactly where the dampers act. The ~0.8 °/g
split is the damper's transient contribution bleeding into a
quasi-steady measurement — the seat's "Normal leans more" made
visible, in the least-transient data this course can provide — and
the split shrinks toward overlap when the steadiness filter is
loosened, which is the fingerprint of a transient effect rather than
a spring-rate one. The binned means also bow steeper at high g than
the straight fit — the relation is progressive, one more reminder
that a single slope is a summary. The genuinely steady version of
this number waits for the constant-radius work. Meanwhile the mean
gradient pays off a
promise from the grip section: at ~2.5 °/g, body roll inflates the
roof accelerometer's lateral channel by about 4%, so the honest grip
figures are **@@P95C@@ g** at the 95th percentile and **@@MAXC@@ g**
peak — the 0.75–0.85 g prediction stays busted after the correction.

The graveyard, with causes of death: a roll-rate *transfer function*
per mode (coherence between lateral input and roll rate never reached
0.6 at any frequency — on a course the road excites roll as much as
the driver does, so the estimator starves; it needs the ride block's
matched inputs). Launch and finish-brake pitch transients overlaid
per mode (driver variance swamps the mode signal, and the roof
lever-arm contaminates the accelerometer during pitch transients).
Brake-dive and squat gradients by the same disagreement trick
(autocross braking is a two-second ramp, never quasi-steady —
correlations near zero, no number). And repeated-bump ringdowns
(Storm Stadium's lot is smooth: three vertical events all session,
none recurring). Every one of those failures points at the same
place: controlled inputs. Which is what the ride block is.

## What's next

The 45-minute ride block (speed bump decays, slab-joint and
broken-surface passes per mode) is still owed — that's where the heave
damping-ratio headline number comes from. Then the quarter-car fit and
the semi-active study on top of it, per the plan. The raw session
folder and the MATLAB pipeline publish with the field report.
"""


if __name__ == "__main__":
    main()
