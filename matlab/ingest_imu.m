function [D, meta] = ingest_imu(path, opts)
%INGEST_IMU Parse a WitMotion WT901SDCL-BT50 log into a timetable.
%
%   [D, meta] = INGEST_IMU(path)            % raw SD file WITn.TXT
%   [D, meta] = INGEST_IMU(path, rate=200)  % nominal rate if no timestamps
%
%   D    timetable: t (datetime or duration), acc [g], gyr [deg/s],
%        ang [deg], in device axes. Mounting contract: printed X arrow at
%        the car's nose, label up -> ISO 8855 body axes (X forward,
%        Y left, Z up). Car mapping: a_long=+accX, a_lat=+accY
%        (left-positive per ISO; negate for SAE), a_vert=+accZ; roll
%        rate=gyrX, pitch rate=gyrY, yaw rate=gyrZ; the device's
%        "Roll"(angX)/"Pitch"(angY) angles are car roll/pitch directly.
%   meta fields: format, n, rate_true, dt_med_ms, dt_p99_ms, gap_max_ms,
%        dup_frac (bandwidth-resampling detector), bad_frames.
%
%   Formats auto-detected: standard 11-byte checksummed frames
%   (0x55 [0x50..0x5A] d0..d7 ck), flag frames 0x55 0x61 +18B (+8B time),
%   or the app's text export. Scale: /32768 * {16 g, 2000 dps, 180 deg}.

arguments
    path (1,1) string
    opts.rate (1,1) double = 200
    opts.dedupe (1,1) logical = false   % drop repeated-value frames
end

fid = fopen(path, 'rb'); assert(fid > 0, "cannot open %s", path);
buf = fread(fid, inf, 'uint8=>uint8')'; fclose(fid);

head = double(buf(1:min(4096, numel(buf))));
if ~isempty(head) && mean((head >= 32 & head < 127) | ismember(head, [9 10 13])) > 0.9
    [D, meta] = parse_text(path, opts.rate);
else
    [rowsStd, bad] = parse_standard(buf);
    rows61 = parse_flag61(buf);
    if ~isempty(rows61) && height(rows61) > height(rowsStd)
        rows = rows61; meta.format = "flag61"; meta.bad_frames = 0;
    else
        assert(~isempty(rowsStd), "no WitMotion frames found in %s", path);
        rows = rowsStd; meta.format = "standard-11B"; meta.bad_frames = bad;
    end
    D = finish(rows, opts.rate);
end

meta.n = height(D);
t = seconds(D.t - D.t(1));
dts = diff(t);
meta.rate_true = (meta.n - 1) / max(t(end), eps);
meta.dt_med_ms = 1000 * median(dts);
meta.dt_p99_ms = 1000 * prctile(abs(dts - 1/opts.rate), 99);
meta.gap_max_ms = 1000 * max(dts);
meta.dup_frac = mean(all(diff(D.acc) == 0, 2));

% Measured 2026-08-16 on venue SD files: the BT50 writes frames at the
% configured 200 Hz on a clean 5 ms clock, but its fusion loop updates
% acc ~104 Hz and gyro ~50 Hz, so ~48% of consecutive frames repeat all
% values. dedupe=true keeps only frames where something changed — the
% honest effective-rate series to resample for spectra. Leave false for
% tap_test (it judges the frame stream itself).
if opts.dedupe
    keep = [true; any(diff(D.acc) ~= 0, 2) | any(diff(D.gyr) ~= 0, 2) ...
            | any(diff(D.ang) ~= 0, 2)];
    D = D(keep, :);
    meta.n_dedup_dropped = meta.n - height(D);
    meta.rate_effective = (height(D) - 1) / ...
        max(seconds(D.t(end) - D.t(1)), eps);
end
end

% ---------------------------------------------------------------------
function [rows, bad] = parse_standard(buf)
i = 1; n = numel(buf); bad = 0;
t = NaT; T = []; A = []; G = []; N = []; k = 0;
est = floor(n / 11);
T = NaT(est, 1); A = nan(est, 3); G = nan(est, 3); N = nan(est, 3);
while i + 10 <= n
    if buf(i) == 85 && buf(i+1) >= 80 && buf(i+1) <= 90   % 0x55, 0x50-0x5A
        if mod(sum(double(buf(i:i+9))), 256) == double(buf(i+10))
            d = double(buf(i+2:i+9)); typ = buf(i+1);
            switch typ
                case 80                                   % 0x50 time
                    t = mk_time(d);
                case 81                                   % 0x51 accel
                    k = k + 1; T(k) = t; A(k,:) = i16(d(1:6));
                case 82                                   % 0x52 gyro
                    if k > 0, G(k,:) = i16(d(1:6)); end
                case 83                                   % 0x53 angle
                    if k > 0, N(k,:) = i16(d(1:6)); end
            end
            i = i + 11; continue
        else
            bad = bad + 1;
        end
    end
    i = i + 1;
end
rows = table(T(1:k), A(1:k,:), G(1:k,:), N(1:k,:), ...
    'VariableNames', {'t','acc','gyr','ang'});
end

function rows = parse_flag61(buf)
idx = find(buf(1:end-1) == 85 & buf(2:end) == 97);        % 0x55 0x61
rows = table();
if numel(idx) < 10, return, end
stride = median(diff(idx));
if stride < 20 || stride > 64 || mean(diff(idx) == stride) < 0.5, return, end
i = idx(1); n = numel(buf); k = 0;
est = floor(n / stride);
T = NaT(est,1); A = nan(est,3); G = nan(est,3); N = nan(est,3);
while i + stride - 1 <= n
    if buf(i) ~= 85 || buf(i+1) ~= 97
        j = find(buf(i:end-1) == 85 & buf(i+1:end) == 97, 1);
        if isempty(j), break, end
        i = i + j - 1; continue
    end
    d = double(buf(i+2:i+19));
    k = k + 1;
    A(k,:) = i16(d(1:6)); G(k,:) = i16(d(7:12)); N(k,:) = i16(d(13:18));
    if stride >= 28, T(k) = mk_time(double(buf(i+20:i+27))); end
    i = i + stride;
end
if k >= 10
    rows = table(T(1:k), A(1:k,:), G(1:k,:), N(1:k,:), ...
        'VariableNames', {'t','acc','gyr','ang'});
end
end

function D = finish(rows, rate)
acc = rows.acc / 32768 * 16;
gyr = rows.gyr / 32768 * 2000;
ang = rows.ang / 32768 * 180;
if mean(~isnat(rows.t)) > 0.5
    t = rows.t;
    t = fillmissing(t, 'linear');                 % time frames are sparse
else
    t = rows.t(1) + seconds((0:height(rows)-1)' / rate);
    if isnat(t(1)), t = datetime(0,1,1) + seconds((0:height(rows)-1)'/rate); end
end
D = timetable(t, acc, gyr, ang);
D = sortrows(D);
end

function [D, meta] = parse_text(path, rate)
M = readmatrix(path);
M = M(all(isfinite(M(:, 1:min(9,end))), 2), :);
assert(size(M,2) >= 9, "text export needs >=9 numeric columns");
c = size(M,2) - 8;                                % assume trailing 9 = data
acc = M(:, c:c+2); gyr = M(:, c+3:c+5); ang = M(:, c+6:c+8);
t = datetime(0,1,1) + seconds((0:size(M,1)-1)' / rate);
D = timetable(t, acc, gyr, ang);
meta.format = "text-export"; meta.bad_frames = 0;
end

function v = i16(d)
v = d(1:2:end) + 256 * d(2:2:end);
v(v >= 32768) = v(v >= 32768) - 65536;
end

function t = mk_time(d)
ms = d(7) + 256 * d(8);
try
    t = datetime(2000 + d(1), d(2), d(3), d(4), d(5), d(6), ms);
catch
    t = NaT;
end
end
