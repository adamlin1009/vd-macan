function [D, meta] = ingest_racebox(path)
%INGEST_RACEBOX Parse a RaceBox Mini S session CSV into a timetable.
%
%   [D, meta] = INGEST_RACEBOX('racebox.csv')
%
%   Returns whatever channels the export contains, normalized to:
%   t (datetime), lat, lon, speed_mps, heading_deg, gx, gy, gz [g],
%   yaw_rate_dps (and any unmapped columns untouched, listed in meta).
%
%   The RaceBox app's CSV layout varies by app version; this reader finds
%   the header row, maps columns by keyword, and errors loudly with the
%   found column names if the essentials are missing. Harden it against
%   the real export when the box lands — treat failures here as expected
%   first-contact, not data loss.

arguments
    path (1,1) string
end

lines = readlines(path);
hdr = find(contains(lower(lines), "latitude"), 1);
assert(~isempty(hdr), "no header row containing 'Latitude' in %s", path);

T = readtable(path, 'NumHeaderLines', hdr - 1, 'VariableNamingRule', 'preserve');
names = string(T.Properties.VariableNames);
low = lower(names);

    function idx = pick(varargin)
        idx = [];
        for p = 1:nargin
            idx = find(contains(low, lower(string(varargin{p}))), 1);
            if ~isempty(idx), return, end
        end
    end

it   = pick("time");
ilat = pick("latitude");  ilon = pick("longitude");
ispd = pick("speed");     ihdg = pick("heading", "bearing", "course");
igx  = pick("gforcex", "g-force x", "gx", "accx", "lateral");
igy  = pick("gforcey", "g-force y", "gy", "accy", "longitudinal");
igz  = pick("gforcez", "g-force z", "gz", "accz", "vertical");
iyaw = pick("gyroz", "yaw", "rotation");

req = [it ilat ilon ispd];
assert(numel(req) == 4, ...
    "ingest_racebox: missing essentials. Columns found:\n  %s", ...
    join(names, newline + "  "));

t = T.(names(it));
if ~isdatetime(t)
    t = datetime(string(t), 'InputFormat', "HH:mm:ss.SSS");  % adjust on contact
end
D = timetable(t, T.(names(ilat)), T.(names(ilon)), ...
    'VariableNames', {'lat','lon'});

spd = T.(names(ispd));
if max(spd, [], 'omitnan') > 130, spd = spd / 3.6; end   % km/h -> m/s guess
D.speed_mps = spd;
if ~isempty(ihdg), D.heading_deg = T.(names(ihdg)); end
if ~isempty(igx),  D.gx = T.(names(igx)); end
if ~isempty(igy),  D.gy = T.(names(igy)); end
if ~isempty(igz),  D.gz = T.(names(igz)); end
if ~isempty(iyaw), D.yaw_rate_dps = T.(names(iyaw)); end

meta.columns_found = names;
meta.rate_true = (height(D) - 1) / seconds(D.t(end) - D.t(1));
meta.unmapped = names(setdiff(1:numel(names), ...
    [it ilat ilon ispd ihdg igx igy igz iyaw]));
end
