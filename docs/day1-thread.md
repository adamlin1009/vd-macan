# Day-1 thread (X/Twitter), owner voice, post-ready

Style: human conversational voice, short and active, no em dashes.
All numbers verified against data/20260815_afternoon. Paste per
tweet. Swap [LINK] for the log-post URL. Post AFTER the site post is
live (tweet 9 says "publicly wrong," which the post makes true).

---

1/
I spent ~$300 on two data loggers, put them in a Porsche Macan S, and
entered an autocross.

The data caught me being wrong three times before it told me anything
about the car.

A thread about dampers, lying clocks, and measured reality:

2/
The experiment: PASM adaptive dampers have a Normal map and a Sport+
map.

Same springs. Same anti-roll bars. Only the damper software changes.

I alternated modes every run: N, S+, N, S+, N, S+. And logged
everything.

3/
The kit:

• RaceBox Mini S on the roof: GPS + gyro, 25 Hz
• WitMotion AHRS IMU on the console: 200 Hz to onboard storage

Total: about the price of one track tire.

4/
Wrong #1. The "200 Hz" sensor is not a 200 Hz sensor.

It writes 200 frames per second. The accelerometer inside only
updates ~104 times per second. Half the frames are copies.

Characterize your instrument before you trust it. Brochure numbers
are marketing.

5/
Wrong #2. My first analysis said the IMU never rode in the car. Flat
line during every single run.

It was on the console the whole day.

The real culprit: its clock.

6/
The IMU's clock runs 2% slow. Every power cycle resets its offset.

By run 1 it trailed GPS time by 131 seconds. By run 6, 182.

My analysis windows were slicing through empty paddock time.

Never trust a logger clock you haven't measured against GPS.

7/
The fix was free.

Each run is 56 seconds of unmistakable car motion. Cross-correlate
the acceleration envelope against GPS, and every file snaps to true
time with an estimated calibration residual of about 0.11 s.

Your data often carries its own sync signal. Find the fingerprint.

8/
No timing equipment? GPS is timing equipment.

Launch = speed rising from zero. Finish = the last hard brake.

Gates built from six trajectories landed within 0.7 m of each other.
Calibrated to two remembered official times, the GPS virtual-gate
estimates are shown to tenths.

9/
Wrong #3. Before the event I published a prediction: this SUV on
all-season tires tops out at 0.75–0.85 g.

Raw roof measurement: 0.97 g sustained, 1.14 g peak.

An exploratory roll correction gives 0.93 g sustained, 1.09 g peak.

Publicly wrong. That was the point of writing it down first.

10/
The seat said: Normal has way more body roll. Sport+ feels planted
and reactive.

The roll-rate gyro said: Sport+ has MORE roll rate. +9%.

Contradiction? No. The seat and the gyro answer different questions.

11/
Roll ANGLE is how far the car leans. Springs and bars set it. The
damper button cannot touch it.

Roll RATE is how fast the lean happens. Dampers own it.

"Planted and reactive" is a rate feeling. A tighter car makes MORE
rate. The gyro agreed with the seat.

12/
Favorite trick of the weekend: a roll sensor built from disagreement.

Accelerometer lateral = cornering force + gravity leaking through
lean.
Speed × yaw rate = cornering force only.

Subtract. The leftover IS the roll angle. No roll sensor involved.

13/
That trick measured: Normal leans 2.9°/g. Sport+ leans 2.1°/g. Every
Normal run above every Sport+ run.

On identical springs? Careful: autocross is never truly steady. The
damper's grip on lean-in-motion is what shows through.

Keeping the caveat attached beats polishing the number.

14/
I tried five derived analyses. One survived.

Roll transfer function: dead (road noise drowns the signal).
Pitch transients: dead (driver variance).
Dive gradient: dead (braking too brief).
Bump ringdowns: dead (the lot is smooth).

Negative results are results. Record them.

15/
If you instrument your own car, steal these:

• Characterize the instrument before the experiment
• Register predictions before data exists
• The sensor doesn't measure what it isn't bolted to
• Never trust an unmeasured clock
• When clever math dies, you need controlled inputs, not more math

16/
Current study: one autocross day, complete. No tire experiment or
controlled-input work happened.

Future study, only after a suitable venue exists: rating sheets,
step-steers, constant-radius ramps, and a repeatable bump.

Full write-up, all six figures, raw data: [LINK]

Everything gets published. Including the mistakes.
