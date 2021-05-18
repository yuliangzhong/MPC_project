function n = kepler_M_rate(mu, a)
%
% $Id: kepler_M_rate.m 2310 2014-10-25 17:17:35Z enh20 $
% (c) Ed Hartley, Cambridge University, UK.

n = sqrt(mu/a^3);