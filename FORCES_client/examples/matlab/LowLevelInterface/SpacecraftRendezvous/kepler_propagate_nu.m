function nu = kepler_propagate_nu(M0, n, ecc, ecc2, dT)
%
%
% $Id: kepler_propagate_nu.m 2310 2014-10-25 17:17:35Z enh20 $
% (c) Ed Hartley, Cambridge University, UK.

M = M0 + n*dT; %Mean anomaly vector
E = M;

for k2 = 1:6
    err = E - ecc*sin(E) - M;
    errdash = 1 - ecc*cos(E);
    E = E - err/errdash;
end
nu = atan2( sin(E)*sqrt(1-ecc2), cos(E)-ecc);
