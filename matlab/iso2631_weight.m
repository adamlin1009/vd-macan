function W = iso2631_weight(f)
%ISO2631_WEIGHT Wk frequency weighting (vertical, seated) magnitude at f [Hz].
%
%   W = ISO2631_WEIGHT(f) returns |Wk(f)|, the ISO 2631-1 vertical
%   whole-body weighting, computed from the standard's band-limit +
%   a-v transition + upward-step transfer functions (Annex A parameters:
%   f1=0.4, f2=100, f3=12.5, f4=12.5, Q4=0.63, f5=2.37, Q5=0.91,
%   f6=3.35, Q6=0.91).
%
%   Used by ride_psd for comfort-weighted RMS in the style of ISO 2631 —
%   styled after the standard, no compliance claim.

arguments
    f double
end

s = 1i * 2 * pi * f(:);
w1 = 2*pi*0.4; w2 = 2*pi*100;
Hh = (s.^2 ./ (s.^2 + sqrt(2)*w1*s + w1^2));            % high-pass
Hl = (w2^2 ./ (s.^2 + sqrt(2)*w2*s + w2^2));            % low-pass
w3 = 2*pi*12.5; w4 = 2*pi*12.5; Q4 = 0.63;
Ht = ((s + w3) .* w4^2 ./ (w3 * (s.^2 + w4/Q4*s + w4^2)));
w5 = 2*pi*2.37; Q5 = 0.91; w6 = 2*pi*3.35; Q6 = 0.91;
Hs = ((s.^2 + w5/Q5*s + w5^2) .* w6^2 ./ ...
      ((s.^2 + w6/Q6*s + w6^2) * w5^2));

W = reshape(abs(Hh .* Hl .* Ht .* Hs), size(f));
end
