#!/usr/bin/env python3
"""imu_characterize.py — characterize a WT901SDCL-BT50 file.

Parses a WitMotion log and reports true sample rate, timestamp health,
and duplicate-sample fraction (bandwidth-resampling detector). Impulse observations are informational;
they are not an experiment or pass gate.

Accepted inputs, auto-detected:
  1. Raw SD file (WIT1.TXT etc.): binary stream of either
     a. standard 11-byte frames  0x55 [0x50..0x5A] d0..d7 cksum
     b. flag frames              0x55 0x61 + 18 data bytes (+ optional
                                 8 time bytes YY MM DD HH MN SS MSL MSH)
  2. App text export (.txt/.csv): either the headered 23-column export
     (time, DeviceName, AccX(g)..AngleZ(°), ...) or any numeric table
     whose first 9 numeric columns are acc/gyro/angle.

App exports are recognized as BLE receive-stamped (timestamps clump in
~30 ms bursts): they get a span-rate check only, with the reminder that
the SD-card file carries the authoritative timebase.

Usage:
  python3 imu_characterize.py WIT1.TXT [--rate 200] [--png report.png]

Exit code 0 = no file-health check failed, 1 = at least one FAIL.
"""
import argparse
import datetime as dt
import sys

import numpy as np

STD_TYPES = set(range(0x50, 0x5B))
ACC_SCALE = 16.0 / 32768.0      # g per LSB
GYR_SCALE = 2000.0 / 32768.0    # deg/s per LSB
ANG_SCALE = 180.0 / 32768.0     # deg per LSB


def _i16(lo, hi):
    v = (hi << 8) | lo
    return v - 65536 if v >= 32768 else v


def _decode_time(b):
    """8 bytes YY MM DD HH MN SS MSL MSH -> datetime or None."""
    yy, mm, dd, hh, mn, ss = b[0], b[1], b[2], b[3], b[4], b[5]
    ms = b[6] | (b[7] << 8)
    if not (1 <= mm <= 12 and 1 <= dd <= 31 and hh < 24 and mn < 60
            and ss < 60 and ms < 1000):
        return None
    try:
        return dt.datetime(2000 + yy, mm, dd, hh, mn, ss, ms * 1000)
    except ValueError:
        return None


def parse_standard(buf):
    """Standard checksummed 11-byte frames -> list of sample dicts."""
    i, n = 0, len(buf)
    rows, cur_t, bad = [], None, 0
    cur = None
    while i + 11 <= n:
        if buf[i] == 0x55 and buf[i + 1] in STD_TYPES:
            if (sum(buf[i:i + 10]) & 0xFF) == buf[i + 10]:
                typ, d = buf[i + 1], buf[i + 2:i + 10]
                if typ == 0x50:
                    cur_t = _decode_time(d)
                elif typ == 0x51:
                    cur = {"t": cur_t,
                           "acc": [_i16(d[0], d[1]), _i16(d[2], d[3]),
                                   _i16(d[4], d[5])]}
                    rows.append(cur)
                elif typ == 0x52 and cur is not None:
                    cur["gyr"] = [_i16(d[0], d[1]), _i16(d[2], d[3]),
                                  _i16(d[4], d[5])]
                elif typ == 0x53 and cur is not None:
                    cur["ang"] = [_i16(d[0], d[1]), _i16(d[2], d[3]),
                                  _i16(d[4], d[5])]
                i += 11
                continue
            bad += 1
        i += 1
    return rows, bad


def parse_flag61(buf):
    """0x55 0x61 stride frames (20 or 28 bytes) -> rows, or None."""
    idxs = [i for i in range(len(buf) - 1)
            if buf[i] == 0x55 and buf[i + 1] == 0x61]
    if len(idxs) < 10:
        return None
    gaps = np.diff(idxs)
    stride = int(np.median(gaps))
    if not (20 <= stride <= 64 and np.mean(gaps == stride) > 0.5):
        return None
    rows = []
    i = idxs[0]
    n = len(buf)
    while i + stride <= n:
        if buf[i] != 0x55 or buf[i + 1] != 0x61:
            j = buf.find(b"\x55\x61", i)
            if j < 0:
                break
            i = j
            continue
        d = buf[i + 2:i + 20]
        row = {"t": None,
               "acc": [_i16(d[0], d[1]), _i16(d[2], d[3]), _i16(d[4], d[5])],
               "gyr": [_i16(d[6], d[7]), _i16(d[8], d[9]),
                       _i16(d[10], d[11])],
               "ang": [_i16(d[12], d[13]), _i16(d[14], d[15]),
                       _i16(d[16], d[17])]}
        if stride >= 28:
            row["t"] = _decode_time(buf[i + 20:i + 28])
        rows.append(row)
        i += stride
    return rows if len(rows) >= 10 else None


def parse_text(text):
    """App export. Headered 23-column layout preferred; numeric fallback."""
    lines = text.splitlines()
    if not lines:
        return None
    rows = []
    hdr = [h.strip().lower() for h in lines[0].replace(",", "\t").split("\t")]

    def col(prefix):
        for i, h in enumerate(hdr):
            if h.startswith(prefix):
                return i
        return None

    ia, ig, an, it = col("accx"), col("asx"), col("anglex"), col("time")
    if ia is not None and ig is not None and an is not None:
        for line in lines[1:]:
            p = line.replace(",", "\t").split("\t")
            if len(p) <= max(ia + 2, ig + 2, an + 2):
                continue
            try:
                row = {"t": None,
                       "acc_g": [float(p[ia]), float(p[ia + 1]),
                                 float(p[ia + 2])],
                       "gyr_d": [float(p[ig]), float(p[ig + 1]),
                                 float(p[ig + 2])],
                       "ang_d": [float(p[an]), float(p[an + 1]),
                                 float(p[an + 2])]}
            except ValueError:
                continue
            if it is not None:
                try:
                    row["t"] = dt.datetime.fromisoformat(p[it])
                except ValueError:
                    pass
            rows.append(row)
        return rows if len(rows) >= 10 else None

    for line in lines:
        parts = line.replace(",", "\t").split("\t")
        nums = []
        for p in parts:
            try:
                nums.append(float(p))
            except ValueError:
                pass
        if len(nums) >= 9:
            rows.append({"t": None, "acc_g": nums[0:3], "gyr_d": nums[3:6],
                         "ang_d": nums[6:9]})
    return rows if len(rows) >= 10 else None


def load(path):
    buf = open(path, "rb").read()
    ble = buf.startswith(b"ADDR:")      # app .wplay = raw BLE capture
    head = buf[:4096]
    printable = sum(32 <= b < 127 or b in (9, 10, 13) for b in head)
    if not ble and head and printable / len(head) > 0.9:
        rows = parse_text(buf.decode("utf-8", "replace"))
        if rows:
            return rows, {"format": "text-export", "bad_frames": 0,
                          "ble_capture": False}
    std_rows, bad = parse_standard(buf)
    f61_rows = parse_flag61(buf)
    if f61_rows and len(f61_rows) > len(std_rows):
        return f61_rows, {"format": "flag61" + (" (.wplay)" if ble else ""),
                          "bad_frames": 0, "ble_capture": ble}
    if std_rows:
        return std_rows, {"format": "standard-11B", "bad_frames": bad,
                          "ble_capture": ble}
    sys.exit(f"FAIL: could not parse {path} as any known WitMotion format "
             f"(std={len(std_rows)} frames, flag61="
             f"{0 if not f61_rows else len(f61_rows)})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--rate", type=float, default=200.0,
                    help="configured record rate, Hz")
    ap.add_argument("--png", default="imu_characterization_report.png")
    args = ap.parse_args()

    rows, meta = load(args.file)
    n = len(rows)

    if "acc_g" in rows[0]:
        acc = np.array([r["acc_g"] for r in rows], float)
        raw = None
    else:
        raw = np.array([r["acc"] for r in rows], int)
        acc = raw * ACC_SCALE

    times = [r["t"] for r in rows]
    have_t = sum(t is not None for t in times)
    checks = []   # (name, PASS/WARN/FAIL, detail)
    dts = []

    shared = 0.0
    if have_t > n * 0.5:
        tv = [t for t in times if t is not None]
        shared = float(np.mean([a == b for a, b in zip(tv[1:], tv[:-1])]))

    # --- rate + timestamp health --------------------------------------
    if have_t > n * 0.5 and shared > 0.30:
        # BLE receive-stamped app export: bursts share stamps
        ts = np.array([(t - times[0]).total_seconds() if t else np.nan
                       for t in times])
        good = ~np.isnan(ts)
        span = np.nanmax(ts) - np.nanmin(ts)
        rate = (good.sum() - 1) / span if span > 1 else float("nan")
        err = abs(rate - args.rate) / args.rate
        dts = np.diff(ts[good])
        pos = dts[dts > 0]
        checks.append(("mean rate over span",
                       "PASS" if err <= 0.03 else "WARN",
                       f"{rate:.1f} Hz vs {args.rate:.0f} set "
                       f"({100*err:.1f}% off) — shortfall on an app export "
                       f"is usually BLE transport loss, not the sensor"))
        checks.append(("timestamps", "WARN",
                       f"receive-stamped bursts: {100*shared:.0f}% of rows "
                       f"share a stamp, burst gap median "
                       f"{1000*np.median(pos):.0f} ms, max "
                       f"{1000*np.max(dts):.0f} ms — app-export time is a "
                       f"transport artifact; the SD-card WITn.TXT carries "
                       f"the real timebase"))
        t_s = np.where(good, ts, np.nan)
    elif have_t > n * 0.5:
        ts = np.array([(t - times[0]).total_seconds() if t else np.nan
                       for t in times])
        good = ~np.isnan(ts)
        span = np.nanmax(ts) - np.nanmin(ts)
        rate = (good.sum() - 1) / span if span > 1 else float("nan")
        dts = np.diff(ts[good])
        nback = int(np.sum(dts < 0))
        maxgap = float(np.max(dts)) if len(dts) else float("nan")
        p99 = float(np.percentile(np.abs(dts - 1.0 / args.rate), 99))
        err = abs(rate - args.rate) / args.rate
        if err <= 0.03:
            rst, note = "PASS", ""
        elif meta.get("ble_capture"):
            rst = "WARN"
            note = (" — BLE capture: shortfall = radio drops (device tick "
                    "= dt median); the SD-card file is the complete record")
        else:
            rst, note = "FAIL", ""
        checks.append(("true sample rate", rst,
                       f"{rate:.1f} Hz vs {args.rate:.0f} set "
                       f"({100*err:.1f}% off, span {span:.1f} s){note}"))
        if nback == 0:
            checks.append(("timestamps monotonic", "PASS", ""))
        elif nback <= max(3, int(0.001 * len(dts))):
            checks.append(("timestamps monotonic", "WARN",
                           f"{nback} backward step(s) — RTC nudge(s); "
                           f"ingest sorts/segments these"))
        else:
            checks.append(("timestamps monotonic", "FAIL",
                           f"{nback} backward steps"))
        checks.append(("max gap",
                       "PASS" if maxgap <= 5 / args.rate else "WARN",
                       f"{1000*maxgap:.1f} ms (limit "
                       f"{5000/args.rate:.0f} ms)"))
        checks.append(("dt jitter p99",
                       "PASS" if p99 <= 1.0 / args.rate else "WARN",
                       f"|dt-{1000/args.rate:.0f}ms| p99 = "
                       f"{1000*p99:.1f} ms"))
        t_s = np.where(good, ts, np.nan)
    else:
        checks.append(("timestamps", "WARN",
                       f"only {have_t}/{n} samples carry time — using "
                       f"index/rate timebase; align against an independent "
                       f"reference before cross-device analysis"))
        t_s = np.arange(n) / args.rate

    # --- duplicate-sample fraction (bandwidth resampling detector) ----
    if raw is not None:
        dup = float(np.mean(np.all(np.diff(raw, axis=0) == 0, axis=1)))
        dup_note = ""
        if meta.get("ble_capture"):
            dup_note = (" — CAUTION: the app's .wplay writer repeats stale "
                        "values (time advances, data lags); cross-check the "
                        ".txt export, and trust only the SD file's number")
    else:
        d0 = np.all(np.diff(acc, axis=0) == 0, axis=1)
        dup = float(np.mean(d0))
        gyr = np.array([r.get("gyr_d", [0, 0, 0]) for r in rows], float)
        moving = ((np.abs(np.linalg.norm(acc, axis=1) - 1.0) > 0.05)
                  | (np.abs(gyr).max(axis=1) > 5))
        mi = moving[:-1] & moving[1:]
        if mi.sum() > 100:
            dup = float(np.mean(d0[mi]))
            dup_note = " (measured during motion; rounded text collides at rest)"
        else:
            dup_note = " (rounded text export, mostly at rest — inflated)"
    st = "PASS" if dup <= 0.20 else ("WARN" if dup <= 0.55 else "FAIL")
    checks.append(("duplicate consecutive samples", st,
                   f"{100*dup:.1f}%{dup_note} (>=50% means bandwidth is "
                   f"limiting: raise Band Width to 98 Hz+, or accept "
                   f"effective {args.rate/2:.0f} Hz)"))

    # --- optional impulse observations -------------------------------
    az = acc[:, 2] - np.median(acc[:, 2])
    mad = np.median(np.abs(az - np.median(az))) or 1e-4
    thr = max(0.5, 8 * 1.4826 * mad)
    above = np.abs(az) > thr
    peaks, i = [], 0
    min_sep = int(0.1 * args.rate)
    while i < n:
        if above[i]:
            j = min(i + min_sep, n)
            k = i + int(np.argmax(np.abs(az[i:j])))
            peaks.append(k)
            i = k + min_sep
        else:
            i += 1
    ring_ms = []
    for k in peaks:
        lim = 0.1 * abs(az[k])
        settled = np.where(np.abs(az[k:min(k + int(0.5 * args.rate), n)])
                           < lim)[0]
        ring_ms.append(1000 * settled[0] / args.rate if len(settled)
                       else float("inf"))
    if peaks:
        worst = max(ring_ms)
        checks.append(("impulse peaks (informational)", "INFO",
                       f"{len(peaks)} spikes > {thr:.2f} g"))
        checks.append(("impulse settling (informational)", "INFO",
                       f"worst 10% settling time {worst:.0f} ms; this can "
                       f"describe the recording or mount, but does not "
                       f"change the file-health verdict"))
    else:
        checks.append(("impulses (informational)", "INFO",
                       "none detected; no impulse event is required"))

    # --- report -------------------------------------------------------
    print(f"\n{args.file}: {n} samples, format {meta['format']}, "
          f"{meta['bad_frames']} bad frames")
    width = max(len(c[0]) for c in checks)
    fail = False
    for name, stt, detail in checks:
        fail |= stt == "FAIL"
        print(f"  [{stt:4s}] {name:<{width}}  {detail}")
    print("FILE CHARACTERIZATION:",
          "FAIL — do not trust spectral analysis yet" if fail else
          "PASS — file-health checks complete")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(10, 6))
        axes[0].plot(t_s, acc[:, 2], lw=0.5)
        if peaks:
            axes[0].plot(np.asarray(t_s)[peaks], acc[peaks, 2], "rx")
        axes[0].set(xlabel="t [s]", ylabel="az [g]", title="vertical accel")
        if len(dts):
            axes[1].hist(1000 * np.asarray(dts), bins=100)
            axes[1].set(xlabel="dt [ms]", ylabel="count",
                        title="sample intervals")
        fig.tight_layout()
        fig.savefig(args.png, dpi=120)
        print(f"plot: {args.png}")
    except Exception as e:                                   # noqa: BLE001
        print(f"(no plot: {e})")
    sys.exit(1 if fail else 0)


if __name__ == "__main__":
    main()
