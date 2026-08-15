# WT901SDCL-BT50 shakedown + in-car procedure

Source of truth for the puck's configuration and the per-session workflow.
Config values verified against WitMotion's official BT50 docs
(protocol V260506, user manual v25-04-18, datasheet v23-1227).

## One-time config (at home, in the WITMOTION app, sensor connected)

Open the sensor's **Configuration / Sensor Configuration** panel and set:

| setting | value | why |
|---|---|---|
| Algorithm | **Axis 6** | a car is a rolling magnetic disturbance; 9-axis lets the magnetometer yank yaw around. 6-axis yaw drifts slowly instead — RaceBox GPS owns absolute heading. Skip magnetic calibration entirely. |
| Installation Direction | **Horizontal** | puck mounts flat on the rail |
| Return Rate | **200 Hz** | ships at 10 Hz — the single most important change |
| Band Width | **98 Hz** | firmware pads duplicate samples whenever rate > bandwidth; at the default 20 Hz, "200 Hz" is really ~50 Hz repeated. 98 Hz makes 200 Hz real and still filters above the 4–25 Hz secondary-ride band. |
| Acceleration range | 16 g/s2 (default) | headroom for taps and curbs; still 0.5 mg resolution |
| Gyro range | 2000 deg/s (default) | fixed |
| Gyro Auto Calibrate | leave **on** | re-zeros gyro bias when stationary |
| Time calibration | **click it** | writes the Mac's clock into the RTC so SD timestamps land near real time; brake jabs do the fine alignment |
| Device Name | optional: `MACAN-PUCK` | avoids pairing the wrong "WIT" at a busy paddock |

Then **Calibrate → Acceleration**: puck flat and still on a hard level
surface, click, wait ~2 s. Accept when az reads ≈ 1.000 g and X/Y angles
≈ 0°. Do NOT "Reset Z-axis Angle" at home — yaw zero is per-session, and
in 6-axis mode it re-zeros itself at every power-on anyway.

Charge to full tonight (red LED off = charged; 2–3 h). Battery is good
for 10–20 h of recording — a full event day.

## Desk tap test (gate #1 — tonight)

1. In the app's Storage section tick **RecordStatus** (this is the SD
   record switch). Blue LED flashing = recording to internal storage.
2. Record ~2 min: 30 s still → 5 sharp taps on the TABLE next to the
   puck, ~1 s apart → 30 s still → 3 taps on the puck itself → still.
3. Untick RecordStatus. **Power-cycle the puck, watch the LED:** if the
   blue flash returns on its own, record-on-boot persists and the power
   switch is the whole track workflow. If it doesn't, RecordStatus must
   be ticked from the phone app at each session start — note which.
4. Offload the file (newest `WITn.TXT` via the app's File List → save, or
   USB-C if a drive/serial device appears when plugged into the Mac) and
   run: `python3 tools/tap_check.py WIT1.TXT`

Pass gate (the plan's "verify true sample rate and timestamp stability
before trusting spectra"):
- true rate within 3% of 200 Hz
- timestamps monotonic, no gap > 25 ms
- duplicate consecutive samples ≤ 20% (≥ 50% = bandwidth still limiting)
- taps visible, ringdown < 50 ms

If duplicates ≈ 50%: Band Width didn't take — set 98 Hz again, save,
re-test. If rate ≈ 10 Hz: Return Rate didn't save. Escalation if the gate
keeps failing (from the plan): Movella DOT-class replacement.

## Mounting (tonight — VHB cures overnight)

- Spot: driver's-side seat rail, bare metal, where nothing hits it at
  full seat travel — run the seat through its whole range FIRST.
- Isopropyl-wipe both surfaces. VHB square on the puck's flat base, press
  firm 30 s. Puck flat, label up, **printed Y arrow pointing at the nose**
  (then X = car right, Z = up; vertical ride channel = az).
- Photograph the mounted orientation once — it's the axes contract for
  every future session.
- Knuckle-tap the rail next to the puck tomorrow with a short recording
  running: gate #2 is the same tap criteria on the mount. Crisp on the
  desk but ringing on the rail = the MOUNT is resonating, not the car.

## Per-session card (track)

1. Cold pressures to placard, log them (same gauge, eye level).
2. Puck on ≥ 1 min before rolling (RTC + gyro settle). Confirm blue
   flash (or tick RecordStatus from the phone if step 3 above said so).
3. RaceBox on, wait for satellite fix, start recording.
4. **Three sharp stationary brake jabs** — the clock-sync signature.
5. Drive. Autocross: recordings restart per run, so jabs per run; ride
   loops: one recording per config, jabs at start and end.
6. End of recording: 15 s still → three jabs again → 5 s still → puck off.
7. notes.md: config name, PASM mode, hot pressures, fuel, ambient, surface.
8. Between sessions puck off; top up from a power bank if the day runs long.

## Offload (desk, per session)

One folder per recording: `data/YYYYMMDD_run##_<config>/` with `puck.bin`
(the WITn.TXT, renamed), `racebox.csv`, `notes.md`. Then
`tap_check.py puck.bin` as a quick health pass before anything else reads it.

## RaceBox (when it lands)

Firmware update in the RaceBox app, set 25 Hz recording, verify CSV
export columns against `ingest_racebox.m` (it fails loudly and prints
what it found — expected on first contact), dash mount with sky view,
never the spare well.
