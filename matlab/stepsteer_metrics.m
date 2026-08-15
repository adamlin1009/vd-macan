function M = stepsteer_metrics(rb, t0, opts)
%STEPSTEER_METRICS Yaw-rate transient metrics for one step-steer event.
%
%   M = STEPSTEER_METRICS(rb, t0)   % t0 = event start [s from rec start]
%
%   From the RaceBox gyro: rise time (10-90% of steady yaw rate),
%   overshoot [%], settling time (into +-5% band), steady-state yaw rate.
%   Five events per PASM mode; the registered prediction is that the mode
%   differences live HERE, not in the steady-state gradient.
%
%   TODO on first venue data: event auto-detection from yaw-rate slope so
%   t0 can be approximate.

arguments
    rb timetable
    t0 (1,1) double
    opts.win (1,2) double = [-0.5 4]
end

t = seconds(rb.t - rb.t(1));
m = t >= t0 + opts.win(1) & t <= t0 + opts.win(2);
tt = t(m) - t0;
r = abs(rb.yaw_rate_dps(m));

rss = median(r(tt > 2 & tt < 4));        % steady state
i10 = find(r > 0.1 * rss, 1); i90 = find(r > 0.9 * rss, 1);
M.rise_time_s = tt(i90) - tt(i10);
M.overshoot_pct = 100 * (max(r) - rss) / rss;
outside = abs(r - rss) > 0.05 * rss;
last = find(outside & tt > 0, 1, 'last');
M.settle_time_s = tt(min(last + 1, numel(tt)));
M.yaw_ss_dps = rss;
end
