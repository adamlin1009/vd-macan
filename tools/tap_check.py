#!/usr/bin/env python3
"""tap_check.py — WT901SDCL-BT50 shakedown verifier (no MATLAB needed).

Parses a WitMotion log and reports true sample rate, timestamp health,
duplicate-sample fraction (bandwidth-resampling detector), and tap-spike
crispness against the shakedown pass criteria from the project plan.

Accepted inputs, auto-detected:
  1. Raw SD file (WIT1.TXT etc.): binary stream of either
     a. standard 11-byte frames  0x55 [0x50..0x5A] d0..d7 cksum
     b. flag frames              0x55 0x61 + 18 data bytes (+ optional
                                 8 time bytes YY MM DD HH MN SS MSL MSH)
  2. App text export (.txt/.csv): numeric columns incl. ax ay az.

Usage:
  python3 tap_check.py WIT1.TXT [--rate 200] [--png report.png]

Exit code 0 = all PASS, 1 = any FAIL (WARNs don't fail the gate).
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
    """Standard checksummed 11-byte frames -> dict of sample arrays."""
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
            # resync
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
    """App export: numeric columns, expects ax ay az somewhere."""
    rows = []
    for line in text.splitlines():
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
    head = buf[:4096]
    printable = sum(32 <= b < 127 or b in (9, 10, 13) for b in head)
    if head and printable / len(head) > 0.9:
        rows = parse_text(buf.decode("utf-8", "replace"))
        if rows:
            return rows, {"format": "text", "bad_frames": 0}
    std_rows, bad = parse_standard(buf)
    f61_rows = parse_flag61(buf)
    if f61_rows and len(f61_rows) > len(std_rows):
        return f61_rows, {"format": "flag61", "bad_frames": 0}
    if std_rows:
        return std_rows, {"format": "standard-11B", "bad_frames": bad}
    sys.exit(f"FAIL: could not parse {path} as any known WitMotion format "
             f"(std={len(std_rows)} frames, flag61="
             f"{0 if not f61_rows else len(f61_rows)})")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("file")
    ap.add_argument("--rate", type=float, default=200.0,
                    help="configured record rate, Hz")
    ap.add_argument("--png", default="tap_check_report.png")
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

    # --- true rate + timestamp health ---------------------------------
    if have_t > n * 0.5:
        ts = np.array([(t - times[0]).total_seconds() if t else np.nan
                       for t in times])
        good = ~np.isnan(ts)
        span = np.nanmax(ts) - np.nanmin(ts)
        rate = (good.sum() - 1) / span if span > 1 else float("nan")
        dts = np.diff(ts[good])
        mono = bool(np.all(dts >= -1e-9))
        maxgap = float(np.max(dts)) if len(dts) else float("nan")
        p99 = float(np.percentile(np.abs(dts - 1.0 / args.rate), 99))
        err = abs(rate - args.rate) / args.rate
        checks.append(("true sample rate",
                       "PASS" if err <= 0.03 else "FAIL",
                       f"{rate:.1f} Hz vs {args.rate:.0f} set "
                       f"({100*err:.1f}% off, span {span:.1f} s)"))
        checks.append(("timestamps monotonic",
                       "PASS" if mono else "FAIL", ""))
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
                       f"index/rate timebase; rely on brake-jab sync"))
        t_s = np.arange(n) / args.rate

    # --- duplicate-sample fraction (bandwidth resampling detector) ----
    if raw is not None:
        dup = float(np.mean(np.all(np.diff(raw, axis=0) == 0, axis=1)))
    else:
        dup = float(np.mean(np.all(np.diff(acc, axis=0) == 0, axis=1)))
    st = "PASS" if dup <= 0.20 else ("WARN" if dup <= 0.55 else "FAIL")
    checks.append(("duplicate consecutive samples", st,
                   f"{100*dup:.1f}% (>=50% means bandwidth is limiting: "
                   f"raise Band Width to 98 Hz+, or accept effective "
                   f"{args.rate/2:.0f} Hz)"))

    # --- tap spikes ---------------------------------------------------
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
        checks.append(("tap spikes found", "PASS",
                       f"{len(peaks)} spikes > {thr:.2f} g"))
        checks.append(("tap ringdown < 50 ms",
                       "PASS" if worst <= 50 else "WARN",
                       f"worst {worst:.0f} ms — after VHB mounting, a slow "
                       f"ringdown means the mount (not the car) is ringing"))
    else:
        checks.append(("tap spikes", "WARN",
                       "none detected — did the recording include taps?"))

    # --- report -------------------------------------------------------
    print(f"\n{args.file}: {n} samples, format {meta['format']}, "
          f"{meta['bad_frames']} bad frames")
    width = max(len(c[0]) for c in checks)
    fail = False
    for name, st, detail in checks:
        fail |= st == "FAIL"
        print(f"  [{st:4s}] {name:<{width}}  {detail}")
    print("VERDICT:", "FAIL — do not trust spectra yet" if fail else
          "PASS — logger cleared for the ride block")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, axes = plt.subplots(2, 1, figsize=(10, 6))
        axes[0].plot(t_s, acc[:, 2], lw=0.5)
        if peaks:
            axes[0].plot(np.asarray(t_s)[peaks], acc[peaks, 2], "rx")
        axes[0].set(xlabel="t [s]", ylabel="az [g]", title="vertical accel")
        if have_t > n * 0.5 and len(dts):
            axes[1].hist(1000 * dts, bins=100)
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
