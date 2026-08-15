function P = semiactive_sim(Q, opts)
%SEMIACTIVE_SIM Semi-active strategies on the identified model. (Phase 3 — stub.)
%
%   P = SEMIACTIVE_SIM(Q)   % Q from quarter_car_fit
%
%   Plan of record: implement skyhook, groundhook, acceleration-driven
%   damping, and clipped-optimal on the identified quarter-car; excite with
%   ISO 8608 class B/C profiles plus the measured speed-bump geometry; output
%   THE killer figure — comfort (ISO-weighted RMS) vs tire-load variation,
%   with Porsche's Normal and Sport as two fixed points and the semi-active
%   frontier drawn through the same axes.
%
%   TODO after quarter_car_fit lands.

arguments
    Q struct
    opts.road_class (1,1) string = "C"
    opts.v_kmh (1,1) double = 60
end

error("semiactive_sim: waits on quarter_car_fit (by design).");
end
