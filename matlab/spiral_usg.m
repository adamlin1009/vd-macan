function S = spiral_usg(rb, opts)
%SPIRAL_USG Understeer gradient from a constant-steer spiral (RaceBox data).
%
%   S = SPIRAL_USG(rb)   % rb from ingest_racebox, one spiral ramp
%
%   The plan's centerpiece, no steering sensor needed: with the wheel held
%   fixed, delta = L*kappa + K*ay. Plot L*kappa against ay: the slope is
%   -K (deg/g after unit bookkeeping) and the unknown fixed steer angle is
%   the intercept. Curvature kappa comes from GPS: kappa = yaw_rate/v
%   (preferred) or from path geometry.
%
%   Returns K_deg_per_g, delta0_deg (intercept), R2, and the (ay, Lk) pairs
%   used. Fit range limited to the linear band (ay 0.1-0.45 g by default).
%   Three ramps each direction, required to agree — repetition is the
%   cross-check (shaped after ISO 4138's intent, no compliance claim).

arguments
    rb timetable
    opts.L (1,1) double = 2.807          % Macan wheelbase [m]
    opts.ay_fit (1,2) double = [0.10 0.45]
    opts.steer_ratio (1,1) double = 15.4 % approx, only for road-wheel deg
end

v = rb.speed_mps;
assert(any(strcmp('yaw_rate_dps', rb.Properties.VariableNames)), ...
    "spiral_usg wants yaw_rate_dps; add curvature-from-path fallback on contact");
r = deg2rad(rb.yaw_rate_dps);
kappa = r ./ max(v, 1);                  % 1/m
ay = v .* r / 9.81;                      % g, from v*r (bias-free vs accel)

m = abs(ay) >= opts.ay_fit(1) & abs(ay) <= opts.ay_fit(2) & v > 3;
x = abs(ay(m)); y = rad2deg(opts.L * abs(kappa(m)));   % "Ackermann deg"

p = polyfit(x, y, 1);
S.K_deg_per_g = -p(1);                   % slope = -K
S.delta0_deg = p(2);                     % fixed steer angle (road-wheel deg)
yy = polyval(p, x);
S.R2 = 1 - sum((y - yy).^2) / sum((y - mean(y)).^2);
S.ay = x; S.Lk = y; S.n = nnz(m);
end
