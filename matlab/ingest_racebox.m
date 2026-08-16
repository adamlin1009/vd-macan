function [D, meta] = ingest_racebox(path)
%INGEST_RACEBOX Parse a RaceBox "Track Session" CSV export into a timetable.
%
%   [D, meta] = INGEST_RACEBOX('racebox.csv')
%
%   Hardened against the real export 2026-08-16 (RaceBox Mini S, app
%   "Track Session" CSV): ~11 Key,Value metadata lines, a blank line,
%   then the header row
%     Record,Time,Latitude,Longitude,Altitude,Speed,GForceX,GForceY,
%     GForceZ,Lap,GyroX,GyroY,GyroZ
%   Time = local ISO8601 with ms at 25 Hz. Speed unit varies by app
%   setting, so it is VALIDATED against GPS ground speed and snapped to
%   {m/s, km/h, mph, knots}. Axis semantics verified empirically on the
%   2026-08-15 session: GForceX = longitudinal (+ = accelerating),
%   GForceY = lateral, GForceZ ~ +1 g static, GyroZ = yaw rate [deg/s]
%   (corr 0.98 with v*r). No heading column; Lap is all-zero when no
%   track is defined in the app.
%
%   D    timetable: t, lat, lon, alt_m, speed_mps, gx, gy, gz [g],
%        yaw_rate_dps, lap
%   meta struct: preamble (session metadata), rate_true, speed_unit,
%        speed_ratio (column/GPS)

arguments
    path (1,1) string
end

lines = readlines(path);
hdr = find(startsWith(lines, "Record,"), 1);
assert(~isempty(hdr), "ingest_racebox: no 'Record,...' header row in %s", path);

meta.preamble = struct();
for i = 1:hdr-1
    kv = split(lines(i), ",");
    if numel(kv) >= 2 && strlength(strtrim(kv(1))) > 0
        key = matlab.lang.makeValidName(kv(1));
        meta.preamble.(char(key)) = strjoin(kv(2:end), ",");
    end
end

T = readtable(path, 'NumHeaderLines', hdr-1, 'VariableNamingRule', 'preserve');
req = ["Time" "Latitude" "Longitude" "Altitude" "Speed" "GForceX" ...
       "GForceY" "GForceZ" "GyroX" "GyroY" "GyroZ"];
have = string(T.Properties.VariableNames);
assert(all(ismember(req, have)), ...
    "ingest_racebox: format changed. Columns found:\n  %s", join(have, ", "));

t = T.Time;
if ~isdatetime(t)
    t = datetime(string(t), 'InputFormat', "uuuu-MM-dd'T'HH:mm:ss.SSS");
end

% --- speed units: snap column/GPS ratio to a known unit ---------------
R = 6371000;
la = deg2rad(T.Latitude); lo = deg2rad(T.Longitude);
dts = max(seconds(diff(t)), 1e-3);
vgps = R * hypot(diff(la), cos(la(1:end-1)) .* diff(lo)) ./ dts;
m = vgps > 3;
if nnz(m) > 100
    ratio = median(T.Speed([false; m]) ./ vgps(m));
else
    ratio = 1;                       % too little motion to tell
end
unit_names = ["m/s" "km/h" "mph" "kn"];
unit_scale = [1 3.6 2.23694 1.94384];
[~, k] = min(abs(unit_scale - ratio));
assert(abs(unit_scale(k) - ratio) < 0.25 * unit_scale(k) || nnz(m) <= 100, ...
    "ingest_racebox: speed/GPS ratio %.2f matches no known unit", ratio);
meta.speed_unit = unit_names(k);
meta.speed_ratio = ratio;

D = timetable(t, T.Latitude, T.Longitude, T.Altitude, ...
    T.Speed / unit_scale(k), T.GForceX, T.GForceY, T.GForceZ, T.GyroZ, ...
    'VariableNames', {'lat','lon','alt_m','speed_mps','gx','gy','gz', ...
                      'yaw_rate_dps'});
if ismember("Lap", have), D.lap = T.Lap; end

meta.rate_true = (height(D) - 1) / seconds(D.t(end) - D.t(1));
end
