# MATLAB code: unverified future work

Python is the authoritative analysis for this repository. Every reported
day-1 number comes from `tools/day1_analysis.py`.
None of these `.m` files has been executed against the dataset in this
repository, and MATLAB or Octave was not available for verification. Do
not cite these files as reproduced results.

The MATLAB directory is preserved for recoverability. Some files are
unexecuted sketches of Python analysis concepts; others describe future
studies that are not scheduled. Controlled-input ride, step-steer, and
constant-radius work can proceed only after a suitable venue exists. Tire
work is only a future-study idea and has no implementation here.

| file | limited status |
|---|---|
| `ingest_imu.m` | Unexecuted parser sketch for the IMU formats handled authoritatively by `tools/imu_characterize.py` and `tools/day1_analysis.py` |
| `ingest_racebox.m` | Unexecuted parser sketch for the observed RaceBox export |
| `split_runs.m` | Unexecuted sketch of GPS run detection and virtual gates |
| `sync_runs.m` | Unexecuted clock-alignment sketch; not the day-1 Python envelope implementation |
| `characterize_imu.m` | Unexecuted MATLAB file-characterization sketch |
| `ride_psd.m`, `iso2631_weight.m` | Unverified future ride study; no suitable venue or input data |
| `bump_logdec.m` | Unverified future ride study; no suitable venue or input data |
| `spiral_usg.m` | Unverified future constant-radius study; no suitable venue or input data |
| `stepsteer_metrics.m` | Unverified future controlled-input study; no suitable venue or input data |
| `quarter_car_fit.m` | Unimplemented future-study stub |
| `semiactive_sim.m` | Unimplemented future-study stub |

Do not infer validation from similarity to the Python code. If MATLAB work
ever resumes, it needs its own tests against the raw files and independent
comparison with the Python outputs before its status changes.
