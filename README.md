# vd-macan

Data + MATLAB pipeline for the Macan S instrumentation project. The living
plan is the log post on the site (`content/log/2026-08-05-macan-instrumentation-plan.md`);
operational detail in the site repo's `docs/vd-project-plan.md`. This repo
holds what the plan calls the analysis side: raw run folders and the
MATLAB pipeline, both public when the posts ship.

## Layout

```
data/YYYYMMDD_run##_<config>/   one folder per continuous recording
    imu.bin        raw WitMotion SD file (WITn.TXT, renamed)
    racebox.csv     RaceBox session export
    notes.md        config, PASM mode, pressures cold/hot, fuel, ambient, surface
matlab/             pipeline (ingest -> sync -> metrics -> model)
tools/tap_check.py  shakedown verifier, runs anywhere with python3+numpy
docs/shakedown.md   device config + tap-test gate + in-car procedure
```

## Pipeline order

| stage | file | status |
|---|---|---|
| ingest IMU | `ingest_imu.m` | ready (mirrors verified `tap_check.py` parser) |
| ingest RaceBox | `ingest_racebox.m` | skeleton — harden on first real CSV |
| clock sync | `sync_runs.m` | ready (brake-jab cross-correlation + drift) |
| shakedown gate | `tap_test.m` / `tap_check.py` | ready |
| ride PSD + bands | `ride_psd.m` + `iso2631_weight.m` | ready |
| bump decay → ζ | `bump_logdec.m` | ready |
| understeer gradient | `spiral_usg.m` | ready (yaw-rate curvature path) |
| step-steer metrics | `stepsteer_metrics.m` | ready |
| quarter-car ID | `quarter_car_fit.m` | stub by design — waits on ride data |
| semi-active study | `semiactive_sim.m` | stub by design — waits on the fit |

Device constants baked into the ingest: WT901SDCL-BT50, ±16 g / ±2000 °/s /
±180°, scale = raw/32768 × full-scale, record rate 200 Hz, bandwidth 98 Hz.

## Axes (mounting contract)

Mount the IMU flat, label up, **printed X arrow toward the nose** (Y
arrow then points at the driver's door). That is ISO 8855 body axes:
X forward, Y left, Z up. Car mapping: `a_long=+accX`, `a_lat=+accY`
(left-positive per ISO — negate for SAE right-positive), `a_vert=+accZ`;
roll rate = gyrX, pitch rate = gyrY, yaw rate = gyrZ, and the device's
"Roll"/"Pitch" angle outputs read as car roll/pitch directly. If geometry
forces a rotated mount, photograph it and fix the rotation at ingest —
never in the spreadsheet.
