function R = ride_psd(D, opts)
%RIDE_PSD Welch PSD + band splits of vertical acceleration, per segment.
%
%   R = RIDE_PSD(D)           % D from ingest_puck (uses acc z, in g)
%   R = RIDE_PSD(D, fs=200, band_primary=[0.5 4], band_secondary=[4 25])
%
%   Returns: f, Pzz [(m/s^2)^2/Hz], rms_total, rms_primary, rms_secondary,
%   rms_wk (ISO 2631 Wk-weighted, via iso2631_weight).
%   Primary 0.5-4 Hz = body motion; secondary 4-25 Hz = harshness —
%   "Sport is firmer" is not a finding; WHERE it's firmer is.

arguments
    D timetable
    opts.fs (1,1) double = 200
    opts.band_primary (1,2) double = [0.5 4]
    opts.band_secondary (1,2) double = [4 25]
end

az = (D.acc(:,3) - mean(D.acc(:,3))) * 9.81;          % m/s^2, detrended
nwin = 2^nextpow2(8 * opts.fs);                        % ~8 s windows
[R.Pzz, R.f] = pwelch(az, hann(nwin), nwin/2, nwin, opts.fs);

    function r = bandrms(b)
        m = R.f >= b(1) & R.f <= b(2);
        r = sqrt(trapz(R.f(m), R.Pzz(m)));
    end

R.rms_total = bandrms([0.5 80]);
R.rms_primary = bandrms(opts.band_primary);
R.rms_secondary = bandrms(opts.band_secondary);
R.rms_wk = sqrt(trapz(R.f, R.Pzz .* iso2631_weight(R.f).^2));
end
