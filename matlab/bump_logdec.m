function B = bump_logdec(D, t_bump, opts)
%BUMP_LOGDEC Effective heave damping ratio from a speed-bump transient.
%
%   B = BUMP_LOGDEC(D, t_bump)   % t_bump = approximate bump time [s from start]
%
%   Method (the headline number of the ride block): band-pass az around the
%   body heave mode (0.5-4 Hz), find the decay peaks after the bump, fit
%   log decrement delta = ln(x_n/x_{n+1}) -> zeta = delta/sqrt(4pi^2+delta^2).
%   Returns peaks used, per-pair zeta, zeta_mean, f_heave estimate.
%
%   Honest caveat carried from the plan: PASM is continuously variable and
%   nonlinear; this is an EQUIVALENT LINEAR damping ratio per mode — a
%   summary of behavior, not a copy of the valve map.

arguments
    D timetable
    t_bump (1,1) double
    opts.fs (1,1) double = 200
    opts.win (1,2) double = [-1 5]      % s around bump to analyze
end

t = seconds(D.t - D.t(1));
m = t >= t_bump + opts.win(1) & t <= t_bump + opts.win(2);
az = detrend(D.acc(m,3)) * 9.81;

[b, a] = butter(2, [0.5 4] / (opts.fs/2), 'bandpass');
x = filtfilt(b, a, az);
tt = t(m);

[pkv, pki] = findpeaks(abs(x), 'MinPeakDistance', round(0.2 * opts.fs), ...
    'MinPeakHeight', 0.1 * max(abs(x)));
assert(numel(pki) >= 3, "need >=3 decay peaks — check t_bump/window");

% keep the monotonically decaying tail after the largest peak
[~, i0] = max(pkv);
pkv = pkv(i0:end); pki = pki(i0:end);
keep = [true; diff(pkv) < 0]; pkv = pkv(keep); pki = pki(keep);

% successive |peaks| are half-cycles: x_n/x_{n+1} over one full cycle = skip 2
dlt = log(pkv(1:end-2) ./ pkv(3:end));
B.zeta = dlt ./ sqrt(4*pi^2 + dlt.^2);
B.zeta_mean = mean(B.zeta);
B.f_heave = 1 / (2 * mean(diff(tt(pki))));   % half-cycle spacing -> Hz
B.peaks_t = tt(pki); B.peaks_v = pkv;
end
