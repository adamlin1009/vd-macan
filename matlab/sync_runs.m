function [lag_s, drift, info] = sync_runs(IMU, racebox)
%SYNC_RUNS Unexecuted sketch for aligning IMU and RaceBox motion envelopes.
%
%   [lag_s, drift, info] = SYNC_RUNS(IMU, racebox)
%
%   IMU    timetable from ingest_imu  (uses vector magnitude of acc)
%   racebox timetable from ingest_racebox (uses gx/gy magnitude if present,
%           else d(speed)/dt)
%
%   lag_s   seconds to ADD to IMU time so it lands on RaceBox (GPS) time
%   drift   fractional clock drift estimated from start/end event clusters
%           (NaN if only one cluster found); apply as
%           t_true = t0 + (t - t0)*(1+drift) + lag_s
%   info    struct: xcorr peak, event cluster times per logger
%
%   This file has not been run. It expects recognizable motion near the
%   start and end of a recording. The authoritative day-1 Python analysis
%   instead aligns each complete course-run acceleration envelope.

arguments
    IMU timetable
    racebox timetable
end

fs = 25;                                     % common grid = RaceBox rate

sp = event_signal_imu(IMU, fs);
sr = event_signal_racebox(racebox, fs);

% coarse RTC offset limits the search window to +-60 s
[c, lags] = xcorr(sr.x - mean(sr.x), sp.x - mean(sp.x), 60 * fs);
[pk, im] = max(c);
lag_s = lags(im) / fs + seconds(sr.t0 - sp.t0);

info.xcorr_peak = pk / (norm(sr.x - mean(sr.x)) * norm(sp.x - mean(sp.x)));
info.events_imu = find_clusters(sp);
info.events_racebox = find_clusters(sr);

drift = NaN;
if numel(info.events_imu) >= 2 && numel(info.events_racebox) >= 2
    dp = info.events_imu(end) - info.events_imu(1);
    dr = info.events_racebox(end) - info.events_racebox(1);
    drift = dr / dp - 1;
end
end

function s = event_signal_imu(IMU, fs)
a = vecnorm(IMU.acc, 2, 2);
a = abs(a - movmedian(a, 51));               % spikes over local baseline
t = seconds(IMU.t - IMU.t(1));
tg = (0:1/fs:t(end))';
s.x = interp1(t, a, tg, 'linear', 0);
s.t0 = IMU.t(1);
s.tg = tg;
end

function s = event_signal_racebox(rb, fs)
if all(ismember(["gx","gy"], rb.Properties.VariableNames))
    a = hypot(rb.gx, rb.gy);
else
    a = [0; abs(diff(rb.speed_mps))] * 25 / 9.81;
end
a = abs(a - movmedian(a, 51));
t = seconds(rb.t - rb.t(1));
tg = (0:1/fs:t(end))';
s.x = interp1(t, a, tg, 'linear', 0);
s.t0 = rb.t(1);
s.tg = tg;
end

function tc = find_clusters(s)
thr = max(0.15, 6 * mad(s.x, 1));
hits = s.tg(s.x > thr);
tc = [];
if isempty(hits), return, end
brk = [0; find(diff(hits) > 5); numel(hits)];
for i = 1:numel(brk) - 1
    tc(end+1) = mean(hits(brk(i)+1:brk(i+1)));           %#ok<AGROW>
end
end
