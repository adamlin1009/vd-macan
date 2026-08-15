function Q = quarter_car_fit(ride, bump, opts)
%QUARTER_CAR_FIT Grey-box quarter-car ID per PASM mode. (Phase 3 — stub.)
%
%   Q = QUARTER_CAR_FIT(ride, bump)
%     ride: struct array of ride_psd results (per segment, one PASM mode)
%     bump: struct array of bump_logdec results (same mode)
%
%   Plan of record:
%     - fixed/disclosed: corner sprung mass from published curb weight and
%       distribution; tire vertical rate estimated from pressure
%     - fitted: equivalent linear suspension stiffness + damping per mode,
%       by matching measured transmissibility peaks and the bump decay
%       (zeta, f_heave) — lsqnonlin over the 2-DOF quarter-car response
%     - output: ms, mu, ks, kt, cs_equiv per mode + fit residuals
%
%   The caveat ships with the number: PASM is continuously variable and
%   nonlinear; this identifies EQUIVALENT LINEAR damping per mode per band.
%
%   TODO after the ride block: implement once real PSDs exist — fitting
%   synthetic data first would just launder assumptions into "results".

arguments
    ride struct
    bump struct
    opts.corner_mass_kg (1,1) double = 520   % est. from curb + distribution
    opts.unsprung_kg (1,1) double = 55       % est.
    opts.kt_N_per_m (1,1) double = 250e3     % est. from pressure, disclose
end

error("quarter_car_fit: not implemented until ride-block data exists (by design).");
end
