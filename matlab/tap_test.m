function pass = tap_test(path, opts)
%TAP_TEST Shakedown gate for the WT901SDCL-BT50 (MATLAB twin of tap_check.py).
%
%   pass = TAP_TEST('WIT1.TXT')          % rate assumed 200 Hz
%   pass = TAP_TEST(file, rate=200)
%
%   Pass criteria (from the plan: verify true sample rate and timestamp
%   stability before trusting spectra):
%     rate      true rate within 3% of configured
%     time      monotonic, max gap <= 5 sample periods
%     unique    <=20% exactly-duplicated consecutive samples (>=50% means
%               Band Width is limiting the fusion rate — raise it to 98 Hz)
%     taps      spikes present, ringdown to 10% within 50 ms
%
%   Run once at the desk (sensor on the table, tap the table then the
%   sensor) and once again after VHB mounting (knuckle-tap the seat rail):
%   a slow ringdown on the rail that was crisp on the desk is the MOUNT
%   ringing, and the mount must be fixed before the data can be believed.

arguments
    path (1,1) string
    opts.rate (1,1) double = 200
end

[D, meta] = ingest_puck(path, rate=opts.rate);
fprintf("\n%s: %d samples, format %s, %d bad frames\n", ...
    path, meta.n, meta.format, meta.bad_frames);

pass = true;
    function judge(ok, warnonly, name, detail)
        if ok, s = "PASS"; elseif warnonly, s = "WARN"; else, s = "FAIL"; end
        pass = pass && (ok || warnonly);
        fprintf("  [%s] %-30s %s\n", s, name, detail);
    end

err = abs(meta.rate_true - opts.rate) / opts.rate;
judge(err <= 0.03, false, "true sample rate", ...
    sprintf("%.1f Hz vs %.0f set (%.1f%% off)", meta.rate_true, opts.rate, 100*err));
judge(meta.gap_max_ms <= 5000 / opts.rate, true, "max gap", ...
    sprintf("%.1f ms", meta.gap_max_ms));
judge(meta.dt_p99_ms <= 1000 / opts.rate, true, "dt jitter p99", ...
    sprintf("%.1f ms", meta.dt_p99_ms));
judge(meta.dup_frac <= 0.20, meta.dup_frac <= 0.55, "duplicate samples", ...
    sprintf("%.1f%%", 100 * meta.dup_frac));

az = D.acc(:,3) - median(D.acc(:,3));
thr = max(0.5, 8 * 1.4826 * mad(az, 1));
[pkv, pki] = findpeaks(abs(az), 'MinPeakHeight', thr, ...
    'MinPeakDistance', round(0.1 * opts.rate));
if isempty(pki)
    judge(false, true, "tap spikes", "none found — recording without taps?");
else
    ring = nan(size(pki));
    for i = 1:numel(pki)
        w = az(pki(i):min(pki(i) + round(0.5*opts.rate), end));
        j = find(abs(w) < 0.1 * pkv(i), 1);
        if ~isempty(j), ring(i) = 1000 * j / opts.rate; end
    end
    judge(true, false, "tap spikes found", sprintf("%d > %.2f g", numel(pki), thr));
    judge(max(ring) <= 50, true, "tap ringdown < 50 ms", ...
        sprintf("worst %.0f ms", max(ring)));
end

figure('Name', 'tap test');
subplot(2,1,1); plot(seconds(D.t - D.t(1)), D.acc(:,3)); hold on
plot(seconds(D.t(pki) - D.t(1)), D.acc(pki,3), 'rx');
xlabel('t [s]'); ylabel('az [g]'); title('vertical accel');
subplot(2,1,2); histogram(1000 * diff(seconds(D.t - D.t(1))), 100);
xlabel('dt [ms]'); title('sample intervals');

if pass, fprintf("VERDICT: PASS — logger cleared for the ride block\n");
else,    fprintf("VERDICT: FAIL — do not trust spectra yet\n"); end
end
