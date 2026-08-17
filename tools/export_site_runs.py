#!/usr/bin/env python3
"""Export the day-1 RaceBox runs for the website's measured-data trace.

Reads the staged afternoon session through day1_analysis (same run detection,
same calibrated gates), resamples each run onto a 5 m course-distance grid,
and writes a small JSON that the website turns into app/macan-data.ts
(`npm run data:macan` in the website repo).

RaceBox-only by design: GPS speed and the roof-mounted lateral g channel,
uncorrected for body roll. Nothing from the IMU is exported here.

Usage:
  python3 tools/export_site_runs.py [--out <website>/data/macan/day1-runs.json]
"""
import argparse
import json
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
import sys  # noqa: E402

sys.path.insert(0, str(HERE))
from day1_analysis import (  # noqa: E402
    FINISH_SHIFT_M,
    MODES,
    START_SHIFT_M,
    find_runs,
    load_racebox,
)

STEP_M = 5.0
MPS2MPH = 1.0 / 0.44704
DEFAULT_OUT = Path.home() / "Documents" / "website" / "data" / "macan" / "day1-runs.json"


def resample_run(d, run):
    """One run → samples on a 5 m grid from the start gate to the finish gate."""
    ts, v, gy, E, N = d["ts"], d["v"], d["gy"], d["E"], d["N"]
    t_s, t_f = run["t_s"], run["t_f"]
    inside = np.flatnonzero((ts >= t_s) & (ts <= t_f))
    lo = max(int(inside[0]) - 1, 0)
    hi = min(int(inside[-1]) + 2, len(ts))
    t = ts[lo:hi]
    step = np.hypot(np.diff(E[lo:hi]), np.diff(N[lo:hi]))
    s = np.concatenate([[0.0], np.cumsum(step)])
    s0 = float(np.interp(t_s, t, s))
    s1 = float(np.interp(t_f, t, s))
    course = s - s0
    length = s1 - s0
    grid = np.arange(0.0, length + 1e-9, STEP_M)
    speed_mph = np.interp(grid, course, v[lo:hi]) * MPS2MPH
    lat_g = np.interp(grid, course, gy[lo:hi])
    in_run = slice(int(inside[0]), int(inside[-1]) + 1)
    return {
        "lengthM": round(length, 1),
        "vmaxMph": round(float(v[in_run].max() * MPS2MPH), 1),
        "peakLatG": round(float(np.abs(gy[in_run]).max()), 2),
        "samples": [
            [round(float(x), 1), round(float(sp), 1), round(float(g), 3)]
            for x, sp, g in zip(grid, speed_mph, lat_g)
        ],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", type=Path, default=DEFAULT_OUT)
    args = ap.parse_args()

    d = load_racebox()
    runs, _gate0, _gate1 = find_runs(d)
    if len(runs) != len(MODES):
        sys.exit(f"expected {len(MODES)} runs, found {len(runs)}")

    exported = []
    for i, (run, mode) in enumerate(zip(runs, MODES), start=1):
        if run["t_s"] is None or run["t_f"] is None:
            sys.exit(f"run {i} has no gate crossing")
        rec = {"index": i, "pasmMode": mode, "timeS": round(float(run["time"]), 2)}
        rec.update(resample_run(d, run))
        exported.append(rec)

    payload = {
        "schema": "macan-runs/1",
        "session": {
            "date": "2026-08-15",
            "venue": "Storm Stadium, Lake Elsinore, CA",
            "event": "SCCA Cal Club autocross · afternoon session",
            "car": "Porsche Macan S · PASM",
            "logger": "RaceBox Mini S",
            "rateHz": 25,
            "mount": "roof",
            "gates": (
                "virtual GPS gates, calibrated to official times "
                f"(start +{START_SHIFT_M:g} m, finish −{FINISH_SHIFT_M:g} m)"
            ),
            "stepM": STEP_M,
            "channels": {
                "speedMph": "GPS speed",
                "latG": "GForceY, roof-mounted, uncorrected for body roll, signed",
            },
            "exporter": "vd-macan/tools/export_site_runs.py",
        },
        "columns": ["dM", "speedMph", "latG"],
        "runs": exported,
    }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, indent=1, sort_keys=True) + "\n")
    for r in exported:
        print(
            f"run {r['index']} {r['pasmMode']:<6} {r['timeS']:6.2f} s "
            f"{r['lengthM']:6.1f} m  vmax {r['vmaxMph']:4.1f} mph  "
            f"peak lat {r['peakLatG']:.2f} g  {len(r['samples'])} samples"
        )
    print("wrote", args.out)


if __name__ == "__main__":
    main()
