function M0 = kepler_nu2M(ecc, ecc2, nu0)
%
% $Id: kepler_nu2M.m 2310 2014-10-25 17:17:35Z enh20 $
% (c) Ed Hartley, Cambridge University, UK.

X1a1 = sqrt(1-ecc);            % Precomputed constants
X1a2 = sqrt(1+ecc);            % Precomputed constants
X2 = ecc*sqrt(1-ecc2);      % Precomputed constants
M0 = 2*atan2(X1a1*tan(nu0/2), X1a2) - (X2*sin(nu0))/(1 + ecc*cos(nu0));
