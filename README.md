# vd-macan — instrumenting a Porsche Macan S

Raw data, processed data, and the code behind the Macan S instrumentation
project: a road car (95B Macan S, PASM adaptive dampers on steel springs,
Pirelli Scorpion Verde All-Season) fitted with two small loggers to ask
one question well — **do the PASM Normal and Sport+ damper calibrations
produce measurable, mode-attributable differences in transient response,
and do those differences match what the driver reports?**

The write-ups live on the site's engineering log; this repository is
their evidence. Every number and figure in the day-1 post is regenerated
from the raw logs by one script (below). The living plan, with registered
predictions and a dated history of changes, is the plan post
([source](https://github.com/adamlin1009/website/blob/main/content/log/2026-08-05-macan-instrumentation-plan.md));
the first data post is *Six runs, two damper maps*
([source](https://github.com/adamlin1009/website/blob/main/content/log/2026-08-16-six-runs-two-damper-maps.md)).

## Reproduce

Python ≥ 3.9 and numpy. No MATLAB needed for anything reported so far.

```sh
git clone https://github.com/adamlin1009/vd-macan
cd vd-macan
pip install -r requirements.txt                    # numpy only
shasum -a 256 -c data/20260815_afternoon/SHA256SUMS # raw-file integrity
python3 tools/day1_analysis.py                     # ~3 s
```

Expected console output (these are the numbers in the post):

```
runs: ['53.11s', '51.98s', '52.12s', '52.33s', '51.90s', '51.21s']
roll RMS deg/s: ['4.32', '4.81', '4.28', '4.59', '4.55', '4.95']
clock offsets 131 -> 182 s; xcorr 0.84-0.90; ay corr +0.79..+0.90; norm N 3.24 vs S+ 3.28
lat p95 0.97, max 1.14
roll gradient per run: ['+2.65', '+1.94', '+3.03', '+2.26', '+3.12', '+2.09']; N +2.93 S+ +2.10; corrected grip p95 0.93 max 1.09
processed -> data/20260815_afternoon/processed
figures -> figures/day1
```

The script rewrites `data/20260815_afternoon/processed/` (per-run tables,
synchronized time series, `summary.json`) and `figures/day1/` (standalone
SVGs). Add `--post PATH` to also assemble the log post markdown, or
`--no-write` to only print. To characterize the IMU file itself (true
sample rate, timestamp health, duplicate fraction):

```sh
python3 tools/tap_check.py data/20260815_afternoon/imu_sd/WIT39.TXT
```

## Day 1 — Storm Stadium, 2026-08-15 (SCCA Cal Club autocross)

Six course runs in the afternoon session, PASM alternating
**Normal · Sport+ · Normal · Sport+ · Normal · Sport+**; powertrain mode
fixed in Sport+, PSM in Sport, cold pressures set to placard 37/40 psi
front/rear with the project's reference dial gauge (TPMS read 36/39 at
the start and 40/42 hot at the end), fuel 5/8 → 1/2 tank. Session notes,
including the driver's (non-blind, next-day) impressions, are in
[`data/20260815_afternoon/notes.md`](data/20260815_afternoon/notes.md).

| run | PASM | time [s] | vmax [mph] | peak lat [g] | roll-rate RMS [°/s] | roll gradient [°/g] |
|---|---|---|---|---|---|---|
| 1 | Normal | 53.11 | 53.3 | 1.05 | 4.32 | 2.65 |
| 2 | Sport+ | 51.98 | 57.1 | 1.09 | 4.81 | 1.94 |
| 3 | Normal | 52.12 | 55.1 | 1.06 | 4.28 | 3.03 |
| 4 | Sport+ | 52.33 | 55.3 | 1.04 | 4.59 | 2.26 |
| 5 | Normal | 51.90 | 53.5 | 1.06 | 4.55 | 3.12 |
| 6 | Sport+ | 51.21 | 55.6 | 1.14 | 4.95 | 2.09 |

Full precision in [`processed/runs.csv`](data/20260815_afternoon/processed/runs.csv);
definitions in [`processed/summary.json`](data/20260815_afternoon/processed/summary.json).

![Run 6 GPS path, speed-colored, with the calibrated virtual gates](figures/day1/fig01_course_map.svg)

**What the data says so far**

- **Run times.** Gate times reproduce the two remembered official times to
  ~0.1 s rms. Sport+ holds the best time and the mode means sit ~0.5 s
  apart, but run 4 (Sport+) was slower than both adjacent Normal runs and
  n = 3 per mode is thin. Not a result; a table.
- **Grip ceiling.** Registered prediction: 0.75–0.85 g. Measured while
  cornering (roof RaceBox, all runs): p95 0.97 g, peak 1.14 g raw;
  0.93 / 1.09 g after removing the gravity leak from body roll at
  ~2.5 °/g. Prediction busted upward; it stays in the text.
- **Roll rate.** IMU roll-rate RMS is *higher* in Sport+ (4.79 vs
  4.38 °/s, +9%); normalized by lateral-acceleration rate it is a wash
  (3.28 vs 3.24). Consistent with a firmer map making the body follow its
  inputs faster — or with the driver pushing harder in Sport+. Matched
  inputs (step-steers, the ride block) are needed to separate the two.
- **Roll gradient (quasi-steady).** From the RaceBox alone, roll angle =
  accelerometer lateral minus v·yaw-rate/g at quasi-steady cornering
  samples: Normal 2.93 °/g (2.65–3.12), Sport+ 2.10 °/g (1.94–2.26); the
  per-run ranges do not overlap. A true steady-state gradient cannot split
  on unchanged springs and bars, so this is read as the dampers' transient
  contribution bleeding into a not-quite-steady measurement — the split
  shrinks as the steadiness mask is loosened. The steady number waits for
  constant-radius testing.

![Roll angle vs lateral g by PASM mode, quasi-steady samples](figures/day1/fig06_roll_gradient.svg)

**Negatives worth recording** (all need controlled inputs, i.e. the ride
block): roll transfer function per mode (coherence < 0.6 everywhere —
road and driver excite roll together on a course); launch/brake pitch
transients per mode (driver variance, roof lever arm); dive/squat
gradients (autocross braking is never quasi-steady, r ≈ 0); repeated-bump
ringdowns (three vertical events all session, none recurring).

## Instruments, as characterized

| | RaceBox Mini S | WitMotion WT901SDCL-BT50 ("the AHRS IMU") |
|---|---|---|
| mount | roof, GNSS sky view | center console, cupholder perimeter, printed X arrow forward |
| record | 25 Hz GNSS + 6-axis IMU, app CSV export | 200 Hz frames to onboard storage (`WITn.TXT`), 28-byte `0x55 0x61` frames + device RTC |
| effective rate | 25.00 Hz | accel ≈ 104 Hz, gyro ≈ 50 Hz — the fusion loop repeats values in ~48% of frames; dedupe before spectra |
| clock | GNSS-disciplined; the session's reference | ticks a clean 5 ms *relative*, but ~2% slow absolute, and the offset re-arms at every power-on: 131 s behind GPS at run 1, 182 s at run 6. Corrected per file by a linear fit from run-envelope cross-correlation (r 0.84–0.90, residual ±130 ms) — see `processed/imu_clock.csv` |
| axes | GForceX + = accelerating; GForceY + and GyroZ + = left turn (ISO 8855 left-positive; GyroZ vs GPS heading rate r = 0.92, v·yaw vs GForceY r = 0.98) | ISO 8855 body axes: X forward, Y left, Z up. Verified in-car after the clock fix: ax↔GPS long, ay↔GPS lat (r ≤ 0.90), yaw↔yaw (r ≤ 0.91) |
| trust | lateral g carries a body-roll gravity leak (~4% at 2.5 °/g); no roll channel of its own | 6-axis fused Roll/Pitch angles are unusable under sustained lateral acceleration — rates and accelerations only |

Config, tap-test gate, and per-session procedure: [`docs/shakedown.md`](docs/shakedown.md).

## Method notes (short — the code is the reference)

- **Run detection and virtual gates** (`find_runs`): moving bouts (v > 4 m/s
  for > 20 s reaching > 20 m/s); start anchor = position where speed
  first crosses 5 m/s after launch (six anchors within 0.7 m); finish
  anchor = 5 m before the onset of the run's last sustained hard brake
  that terminates near standstill (4.4 m spread). Gate crossings are
  interpolated sign changes of the along-course coordinate within ±12 m
  cross-track. Both gates were then shifted along the course (start
  +2 m, finish −23 m) to fit two remembered official times (rms 0.11 s);
  the finish shift is the "you cross the lights flat-out and brake after"
  correction. Gate coordinates: `processed/gates.csv`.
- **IMU clock model** (`analyze_imu`): per SD file, offset(t) = a + b·t
  fit through per-run offsets found by maximizing the correlation between
  the RaceBox |g| envelope and the IMU |a_xy| envelope over a 100–220 s
  search; wall = device + offset. Deduped, clock-corrected per-run IMU
  windows are exported as `processed/runN_imu.csv`.
- **Roll-rate metrics**: RMS of IMU gyro X inside the gates; normalized
  version divides by the RMS time-derivative of RaceBox lateral g.
- **Roll gradient** (`roll_gradient`): φ ≈ (a_lat,accel − v·r/g), sampled
  on a 40 ms grid, masked to |v·r/g| > 0.30, |d(v·r/g)/dt| < 0.30 g/s,
  v > 8 m/s, |a_long| < 0.25 g; slope of a per-run least-squares line vs
  v·r/g. Samples: `processed/roll_gradient_samples.csv`. RaceBox-only on
  purpose — cross-device versions inherit the ±130 ms clock residual.
- **Grip ceiling**: |a_lat| while cornering (v > 8 m/s, |a_lat| > 0.3 g)
  inside the gates, all runs; roll correction divides by (1 + φ̄) with φ̄
  the mean roll gradient in radians per g.

## Limitations (read before quoting a number)

- n = 3 runs per mode, competition runs, one driver, one day.
- Mode-to-mode comparisons on course runs cannot separate "the car
  responded faster" from "the driver asked for more". Matched-input tests
  are the fix and have not run yet.
- The subjective impressions in `notes.md` were recalled the day after,
  non-blind. Blind per-run rating sheets are part of the protocol going
  forward.
- Official run times were not recorded in the data; two remembered
  officials calibrated the gates.
- The morning session was driven but is excluded from analysis by
  decision (its IMU files are still in the card image for completeness).
- Ambient temperature and surface notes for day 1 are missing.

## Repository layout

```
data/
  README.md                       data dictionary: every file, column, unit, sign
  20260815_afternoon/             the analyzed session
    racebox.csv                   RaceBox "Track Session" export, 25 Hz, 53 min
    imu_sd/WIT38..41.TXT          IMU onboard-storage files covering the session
    app_capture/                  quick-look BLE captures (lossy; not used)
    notes.md                      configuration, conditions, run labels, impressions
    SHA256SUMS                    checksums of the raw files above
    processed/                    generated by tools/day1_analysis.py
      runs.csv                    one row per run: gates, times, peaks, IMU metrics
      gates.csv                   calibrated virtual gates (lat/lon, heading, shifts)
      imu_clock.csv               clock-offset anchors and per-file linear fits
      runN_racebox.csv            per-run RaceBox series (gates −3 s … +6 s)
      runN_imu.csv                per-run IMU series, deduped, clock-corrected
      roll_gradient_samples.csv   quasi-steady samples behind FIG 06
      summary.json                every headline number with its definition
  sd_dump_20260816/               untouched image of the IMU card (all sessions
                                  + shakedown files + SET.TXT config), checksummed
figures/day1/                     standalone SVGs of the six post figures
tools/
  day1_analysis.py                the day-1 analysis: gates → clock → metrics → tables/figures/post
  tap_check.py                    IMU file parser + instrument characterization gate
  export_site_runs.py             RaceBox-only per-run export for the website's trace
matlab/                           planned pipeline (see status below)
docs/
  shakedown.md                    IMU config, tap-test gate, per-session card, protocol
  day1-thread.md                  plain-language thread version of the day-1 write-up
```

## MATLAB pipeline (status)

The plan names MATLAB as the analysis home for the ride block, the
quarter-car identification, and the semi-active study. Those stages have
not run yet, and **no `.m` file has been executed against this dataset in
this repository's history** (the analysis machine has no MATLAB or
Octave). The day-1 numbers come from `tools/day1_analysis.py`. Where the
two overlap, the MATLAB mirrors the Python: `split_runs.m` ↔ `find_runs`,
`ingest_imu.m` ↔ the `tap_check.py` parsers, `ingest_racebox.m` ↔
`load_racebox` (with unit validation added). Treat the rest as
written-not-tested until this section says otherwise.

| stage | file | status |
|---|---|---|
| ingest IMU | `ingest_imu.m` | written; mirrors the verified Python parser |
| ingest RaceBox | `ingest_racebox.m` | written against the real export format |
| run gates | `split_runs.m` | written; mirrors `find_runs` |
| clock sync | `sync_runs.m` | written (brake-jab cross-correlation + drift) |
| shakedown gate | `tap_test.m` / `tap_check.py` | Python version used and verified |
| ride PSD + bands | `ride_psd.m` + `iso2631_weight.m` | written; waits on the ride block |
| bump decay → ζ | `bump_logdec.m` | written; waits on the ride block |
| understeer gradient | `spiral_usg.m` | written; waits on constant-radius testing |
| step-steer metrics | `stepsteer_metrics.m` | written; waits on matched-input runs |
| quarter-car ID | `quarter_car_fit.m` | stub by design |
| semi-active study | `semiactive_sim.m` | stub by design |

Device constants baked into the ingest: WT901SDCL-BT50, ±16 g / ±2000 °/s
/ ±180°, scale = raw/32768 × full-scale, 200 Hz frames, 98 Hz bandwidth.

## Standing rules

Predictions are registered before data is collected. Claims are scoped to
what the instrument can support. Maneuvers happen on closed courses only.
Cold pressures are set and logged every session. When a result embarrasses
a prediction, the prediction stays in the text.

## License

Code (`tools/`, `matlab/`) is MIT. Data (`data/`, `figures/`) and
documentation are CC BY 4.0 — reuse with attribution to Adam Lin and a
link to this repository. See [`LICENSE`](LICENSE).
