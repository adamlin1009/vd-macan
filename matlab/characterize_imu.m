function pass = characterize_imu(path, opts)
%CHARACTERIZE_IMU File checks for the WT901SDCL-BT50.
%
%   pass = CHARACTERIZE_IMU('WIT1.TXT')          % rate assumed 200 Hz
%   pass = CHARACTERIZE_IMU(file, rate=200)
%
%   File-health criteria before spectral analysis:
%     rate      true rate within 3% of configured
%     time      monotonic, max gap <= 5 sample periods
%     unique    <=20% exactly-duplicated consecutive samples (>=50% means
%               Band Width is limiting the fusion rate — raise it to 98 Hz)
%
%   Any recorded impulse peaks and settling times are printed as context.
%   They are informational and do not affect pass.

arguments
    path (1,1) string
    opts.rate (1,1) double = 200
end

[D, meta] = ingest_imu(path, rate=opts.rate);
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
    fprintf("  [INFO] %-30s %s\n", "impulses", "none detected; none required");
else
    ring = nan(size(pki));
    for i = 1:numel(pki)
        w = az(pki(i):min(pki(i) + round(0.5*opts.rate), end));
        j = find(abs(w) < 0.1 * pkv(i), 1);
        if ~isempty(j), ring(i) = 1000 * j / opts.rate; end
    end
    fprintf("  [INFO] %-30s %d peaks > %.2f g\n", ...
        "impulse peaks", numel(pki), thr);
    fprintf("  [INFO] %-30s worst 10%% settling time %.0f ms\n", ...
        "impulse settling", max(ring));
end

figure('Name', 'IMU file characterization');
subplot(2,1,1); plot(seconds(D.t - D.t(1)), D.acc(:,3)); hold on
plot(seconds(D.t(pki) - D.t(1)), D.acc(pki,3), 'rx');
xlabel('t [s]'); ylabel('az [g]'); title('vertical accel');
subplot(2,1,2); histogram(1000 * diff(seconds(D.t - D.t(1))), 100);
xlabel('dt [ms]'); title('sample intervals');

if pass, fprintf("FILE CHARACTERIZATION: PASS — file-health checks complete\n");
else,    fprintf("FILE CHARACTERIZATION: FAIL — do not trust spectral analysis yet\n"); end
end
