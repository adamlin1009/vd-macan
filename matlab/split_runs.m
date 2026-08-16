function [runs, gates] = split_runs(rb, opts)
%SPLIT_RUNS Time autocross runs through GPS virtual start/finish gates.
%
%   [runs, gates] = SPLIT_RUNS(rb)     % rb from ingest_racebox
%
%   Gate construction (validated on the 2026-08-15 afternoon session,
%   6 runs, start-anchor spread 0.7 m, finish spread 11.7 m):
%     start  = median position over runs where speed first crosses
%              v_start (5 m/s) after launch from standstill — a few
%              meters into the run, about where the timing lights sit;
%              gate direction = median course heading there.
%     finish = per run, take the LAST moment the car is fast
%              (> v_fast, 15 m/s), find the sustained hard brake there
%              (gx < -0.45 g for >= 0.32 s), back up 5 m along the path
%              (the lights sit before the braking zone); median position.
%   Crossings are sign changes of the along-gate coordinate,
%   interpolated between samples.
%
%   runs  table: i_start, i_finish, t_start, t_finish, run_time_s,
%         vmax_mps, lat_g_max, brake_g_max
%   gates struct: start/finish lat, lon, heading (ENU unit vector)

arguments
    rb timetable
    opts.v_start (1,1) double = 5.0
    opts.v_fast (1,1) double = 15.0
    opts.v_run (1,1) double = 20.0     % bout must exceed this to be a run
    opts.brake_g (1,1) double = 0.45
    opts.backup_m (1,1) double = 5.0
end

R = 6371000;
ts = seconds(rb.t - rb.t(1));
la0 = deg2rad(mean(rb.lat)); lo0 = deg2rad(mean(rb.lon));
E = R * cos(la0) * (deg2rad(rb.lon) - lo0);
N = R * (deg2rad(rb.lat) - la0);
v = rb.speed_mps;

% --- movement bouts that reach run speed ------------------------------
mov = v > 4;
d = diff(mov);
ups = find(d == 1) + 1; downs = find(d == -1);
bouts = [];
for u = ups'
    w = downs(downs > u);
    if isempty(w), w = numel(v); end
    b = w(1);
    if ts(b) - ts(u) > 20 && max(v(u:b)) > opts.v_run
        bouts(end+1, :) = [u b];                          %#ok<AGROW>
    end
end

sa = []; sd = []; fa = []; fd = [];
for r = 1:size(bouts, 1)
    [a, b] = deal(bouts(r, 1), bouts(r, 2));
    a0 = a; while a0 > 1 && v(a0-1) > 0.5, a0 = a0 - 1; end
    iv = find(v(a0:b) >= opts.v_start, 1) + a0 - 1;
    f = (opts.v_start - v(iv-1)) / max(v(iv) - v(iv-1), 1e-6);
    sa(end+1, :) = [E(iv-1) N(iv-1)] + f * [E(iv)-E(iv-1) N(iv)-N(iv-1)]; %#ok<AGROW>
    u = [E(iv+5)-E(iv-1) N(iv+5)-N(iv-1)];
    sd(end+1, :) = u / norm(u);                           %#ok<AGROW>

    ifast = find(v(a0:b) > opts.v_fast, 1, 'last') + a0 - 1;
    if isempty(ifast), continue, end
    w = find(ts >= ts(ifast) - 3 & ts <= ts(ifast) + 4);
    k = [];
    for i = 1:numel(w)-8
        if all(rb.gx(w(i:i+7)) < -opts.brake_g), k = w(i); break, end
    end
    if isempty(k), continue, end
    j = k; dist = 0;
    while j > 1 && dist < opts.backup_m
        dist = dist + hypot(E(j)-E(j-1), N(j)-N(j-1)); j = j - 1;
    end
    fa(end+1, :) = [E(j) N(j)];                           %#ok<AGROW>
    u = [E(k)-E(k-8) N(k)-N(k-8)];
    fd(end+1, :) = u / norm(u);                           %#ok<AGROW>
end

P0 = median(sa, 1); u0 = median(sd, 1); u0 = u0 / norm(u0);
P1 = median(fa, 1); u1 = median(fd, 1); u1 = u1 / norm(u1);
gates.start = to_ll(P0); gates.start_dir = u0;
gates.finish = to_ll(P1); gates.finish_dir = u1;
gates.start_spread_m = max(hypot(sa(:,1)-P0(1), sa(:,2)-P0(2)));
gates.finish_spread_m = max(hypot(fa(:,1)-P1(1), fa(:,2)-P1(2)));

runs = table();
for r = 1:size(bouts, 1)
    [a, b] = deal(bouts(r, 1), bouts(r, 2));
    t_s = cross_gate(P0, u0, max(1, a-100), min(b, a+1000), 1.0);
    t_f = cross_gate(P1, u1, a, min(numel(v), b+50), 8.0);
    if isnan(t_s) || isnan(t_f), continue, end
    seg = a:b;
    runs = [runs; table(a, b, rb.t(1)+seconds(t_s), rb.t(1)+seconds(t_f), ...
        t_f - t_s, max(v(seg)), max(abs(rb.gy(seg))), -min(rb.gx(seg)), ...
        'VariableNames', {'i_start','i_finish','t_start','t_finish', ...
        'run_time_s','vmax_mps','lat_g_max','brake_g_max'})];  %#ok<AGROW>
end

    function tc = cross_gate(P, u, lo_i, hi_i, vmin)
        s = (E(lo_i:hi_i) - P(1)) * u(1) + (N(lo_i:hi_i) - P(2)) * u(2);
        ok = find(s(1:end-1) < 0 & s(2:end) >= 0 & v(lo_i:hi_i-1) > vmin, 1);
        if isempty(ok), tc = NaN; return, end
        i = ok + lo_i - 1;
        den = s(ok+1) - s(ok); if den == 0, den = 1e-9; end
        tc = ts(i) + (-s(ok) / den) * (ts(i+1) - ts(i));
    end

    function ll = to_ll(P)
        ll = [rad2deg(P(2)/R + la0), rad2deg(P(1)/(R*cos(la0)) + lo0)];
    end
end
