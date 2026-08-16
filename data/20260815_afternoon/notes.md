# 2026-08-15 afternoon session — SCCA Cal Club autocross, Storm Stadium (day 1)

**Owner directive (2026-08-16): this afternoon session is the analysis
dataset; the morning session (10:16) is excluded.**

## Configuration (owner-confirmed 2026-08-16)

- 6 course runs, PASM damper alternating:
  **run 1 Normal · run 2 Sport+ · run 3 Normal · run 4 Sport+ ·
  run 5 Normal · run 6 Sport+**
- Powertrain mode: **Sport+** (constant all runs)
- PSM: **Sport** (not fully off)
- Pressures cold/hot, fuel, ambient, surface: TODO owner
- Blind rating sheets: TODO owner (attach or transcribe per run)

## Runs (GPS virtual gates, from split_runs on racebox.csv)

Start gate 33.6525677, -117.3018512 (anchor spread 0.7 m over 6 runs);
finish gate 33.6527631, -117.3033518 (spread 4.4 m), 5 m before the
onset of each run's FINAL braking (owner: course is point-to-point,
finish before the big final brake; times owner-corroborated ~51-52 s).
Session t0 = 14:52:07.440 local.

| run | PASM | start t [s] | gate-to-gate [s] | vmax [mph] | lat g max | brake g max |
|----|--------|--------|-------|------|------|------|
| 1 | Normal | 590.73 | 54.82 | 53.3 | 1.05 | 0.98 |
| 2 | Sport+ | 1005.14 | 53.61 | 57.1 | 1.09 | 0.82 |
| 3 | Normal | 1423.38 | 53.76 | 55.1 | 1.06 | 0.87 |
| 4 | Sport+ | 1862.47 | 53.98 | 55.3 | 1.04 | 0.84 |
| 5 | Normal | 2222.56 | 53.62 | 53.5 | 1.06 | 0.79 |
| 6 | Sport+ | 2533.23 | 52.83 | 55.6 | 1.14 | 0.87 |

High-g activity clusters (|g|>0.45): 590.7–646.8, 1005.0–1060.4,
1423.3–1479.6, 1862.3–1918.8, 2222.4–2278.6, 2533.2–2589.0 s — the
gates bracket each cluster with the final braking just past the finish.

Brake-jab clusters (stationary long-g spikes) precede runs at t ≈ 569,
989, 1408, 2200, 2510 s (~6 spikes each) + a 13-spike cluster at 2674 s —
sync anchors for sync_runs. Grip-ceiling view (|lat g| while cornering):
p95 0.97, p99 1.02, max 1.14 — registered prediction was 0.75–0.85 g.

## Files

- `racebox.csv` — RaceBox Track Session 14:52 (25.00 Hz, 53 min, Speed
  in mph, GForceX=long/GForceY=lat/GyroZ=yaw verified)
- `puck_sd/WIT38–WIT41.TXT` — the puck's own storage covering the
  session (200 Hz frames, ~104 Hz effective acc; parse with
  ingest_puck, dedupe=true for spectra)
- `app_capture/` — quick-look BLE captures (lossy; see shakedown.md
  channel discipline)
- Full untouched card image in `../sd_dump_20260816/`
