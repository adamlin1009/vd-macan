# 2026-08-15 afternoon session — SCCA Cal Club autocross, Storm Stadium (day 1)

**Owner directive (2026-08-16): this afternoon session is the analysis
dataset; the morning session (10:16) is excluded.**

## Configuration (owner-confirmed 2026-08-16)

- 6 course runs, PASM damper alternating:
  **run 1 Normal · run 2 Sport+ · run 3 Normal · run 4 Sport+ ·
  run 5 Normal · run 6 Sport+**
- Powertrain mode: **Sport+** (constant all runs)
- PSM: **Sport** (not fully off)
- Pressures: set **37/40 F/R before the session** with the Jaco dial
  gauge (the project's reference gauge). TPMS read 36/39 at session
  start (≈ −1 psi vs gauge, both axles — consistent offset, deltas
  unaffected) and climbed progressively to **40/42 F/R** by session end
  (+4/+3 psi build).
- Fuel: ~5/8 tank at start → ~1/2 at end (within the ¼–¾ convention).
- Ambient/surface: TODO owner.

## Subjective impressions (owner, recalled 2026-08-16 — post-session,
## non-blind; per-run blind sheets were not filled on day 1)

- **Normal:** "a lot more body roll, pitch, and yaw — but the car still
  drove nicely."
- **Sport+:** "a lot more planted and reactive."

These are the claims the objective transients must test: Normal should
show larger roll/pitch rates (puck gyrX/gyrY) and larger yaw overshoot /
slower settling (RaceBox GyroZ via stepsteer_metrics on course
transients); "planted" should appear as higher damping in both.

## Runs (GPS virtual gates, from split_runs on racebox.csv)

Gates CALIBRATED to owner-remembered official times (run 5 ≈ 52.0,
run 6 ≈ 51.1; rms fit 0.11 s): geometric anchors (start = v-crossing
after launch, spread 0.7 m; finish = 5 m before terminal brake onset,
spread 4.4 m) shifted start +2 m downstream and finish 23 m earlier —
the car crosses the real finish flat-out ~1 s before braking. In
split_runs: start_shift_m=2.0, finish_shift_m=23.0 for this session.
Calibrated start gate 33.6525529, -117.3018635; finish 33.6527783,
-117.3035996. Session t0 = 14:52:07.440 local.

| run | PASM | start t [s] | run time [s] | vmax [mph] | lat g max | brake g max |
|----|--------|--------|-------|------|------|------|
| 1 | Normal | 591.06 | 53.11 | 53.3 | 1.05 | 0.98 |
| 2 | Sport+ | 1005.44 | 51.98 | 57.1 | 1.09 | 0.82 |
| 3 | Normal | 1423.68 | 52.12 | 55.1 | 1.06 | 0.87 |
| 4 | Sport+ | 1862.76 | 52.33 | 55.3 | 1.04 | 0.84 |
| 5 | Normal | 2222.85 | 51.90 | 53.5 | 1.06 | 0.79 |
| 6 | Sport+ | 2533.54 | 51.21 | 55.6 | 1.14 | 0.87 |

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
  ingest_puck, dedupe=true for spectra). **CAUTION: the puck was NOT
  aboard during the runs** — az_std ≤ 0.007 g and gyro ≤ 0.5 °/s in
  run windows 1–5, hand-scale wiggle in run 6; it logged the paddock.
  No roll/ride channel exists for this session.
- `app_capture/` — quick-look BLE captures (lossy; see shakedown.md
  channel discipline)
- Full untouched card image in `../sd_dump_20260816/`
