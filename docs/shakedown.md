# WT901SDCL-BT50 configuration, characterization, and session procedure

This document records the configuration that produced the repository's
IMU files, what day 1 established about those files, and the procedure for
any later recording. Config values were checked against WitMotion's BT50
protocol V260506, user manual v25-04-18, and datasheet v23-1227.

## Configuration used

| setting | value | reason or observed consequence |
|---|---|---|
| Algorithm | **Axis 6** | A car is a poor magnetic environment. RaceBox GPS owns absolute heading. |
| Installation Direction | **Horizontal** | The IMU was mounted flat with its printed X arrow forward. |
| Return Rate | **200 Hz** | The SD file contains exact 5 ms frame ticks. |
| Band Width | **98 Hz** | The fusion loop still repeats values, but the accelerometer produces about 104 distinct values per second. |
| Acceleration range | **±16 g** | Saved in `data/sd_dump_20260816/SET.TXT`. |
| Gyro range | **±2000 °/s** | Saved in `SET.TXT`. |
| Gyro Auto Calibrate | **on** | Device setting used on day 1. |
| Time calibration | set before the event | The device RTC still ran about 2% slow and its offset reset at power-on, so its clock is not an analysis reference. |

The accelerometer was leveled while the device was flat and still. In
six-axis mode the fused Roll/Pitch angles are not usable under sustained
lateral acceleration. The analysis uses rates and accelerations instead.

## Day-1 mount: what actually ran

The AHRS IMU was on the center console at the cupholder perimeter, not on
the seat rail. It sat flat with the printed X arrow pointing toward the
nose: X forward, Y left, Z up. The in-car comparison after clock
correction supports that mapping: IMU longitudinal tracks RaceBox
longitudinal, lateral tracks lateral, and yaw tracks yaw.

That location is part of the day-1 measurement definition. It should not
be rewritten as a rigid chassis mount after the fact. Any future study
must record its actual mount and orientation before comparing results.

## Characterize each IMU file first

From the repository root, run:

```sh
python3 tools/imu_characterize.py data/20260815_afternoon/imu_sd/WIT39.TXT
```

The report covers frame rate, timestamp health, and consecutive duplicate
values. Those checks describe the file before spectral work. The day-1 SD
files established:

- 200 Hz frames on exact 5 ms device ticks;
- occasional isolated RTC back-steps, which ingest sorts or segments;
- about 48% repeated complete frames;
- about 104 distinct accelerometer values per second and about 50 distinct
  gyro values per second.

The repeated frames mean the configured frame rate is not the effective
sensor-update rate. `day1_analysis.py` deduplicates before using the IMU
channels. The roughly 104 Hz accelerometer update rate is sufficient for
the project's stated 4–25 Hz ride band, but no controlled-input ride
measurement has been collected.

Some archived desk and mount recordings contain deliberate impulses. The
characterization tool reports detected peaks and settling time only as
context. Those observations are informational, not a pass criterion and
not an experiment in the current plan.

## Day-1 clock and channel behavior

The IMU clock trailed the GPS reference by about 131 s at run 1 and 182 s
at run 6. Its relative ticks were regular, but the absolute clock ran
about 2% slow and the initial offset changed at power-on. Day 1 therefore
uses a per-file linear clock model fitted from cross-correlation of the
RaceBox and IMU acceleration envelopes. The driving itself is the sync
signal. Per-run envelope correlations were 0.84–0.90, with an estimated
residual within about ±0.13 s.

The SD-card `WITn.TXT` files are the authoritative IMU channel. USB-C
mounted the card as a drive. The app `.txt` export uses bursty phone
receive timestamps and loses samples; the `.wplay` capture repeats stale
values. Both are quick-look records only.

## Procedure for any later recording

Another autocross is not part of the current plan. If one is recorded as
additional data, use this card:

1. Set cold pressures to placard with the reference gauge and log them.
2. Record the IMU's actual mount, orientation, and PASM mode. Verify the
   storage LED before moving.
3. Start the RaceBox after GNSS fix. Keep it where it has sky view.
4. Keep both loggers running through recognizable driving. Align them in
   post by acceleration-envelope cross-correlation.
5. Alternate PASM mode only if the session is explicitly an A/B set. Keep
   powertrain and PSM settings fixed and record their cluster indication.
6. Record per-run subjective ratings before looking at times or plots.
7. Log hot pressures, fuel, ambient conditions, surface, and mistakes.
8. Offload without renaming device files, compute `SHA256SUMS`, then run
   `imu_characterize.py` before analysis.

One session folder contains `racebox.csv`, the original `imu_sd/WITn.TXT`
files, `notes.md`, and raw-file checksums. Column definitions are in
[`data/README.md`](../data/README.md).

## Future studies, venue required

There was no tire experiment, controlled-input ride measurement,
step-steer set, or constant-radius set on day 1. Those are not committed
current work. They may be designed as future studies only after a suitable
closed venue exists. Until then, this procedure does not claim damping
ratio, steady understeer gradient, or matched-input PASM results.
