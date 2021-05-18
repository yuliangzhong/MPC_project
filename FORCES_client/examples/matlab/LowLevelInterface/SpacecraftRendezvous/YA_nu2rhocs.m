function out = YA_nu2rhocs(cfg,nu)
% out = YA_nu2rhocs(cfg,nu)
%
%   Compute some re-usable parameters for use in the YA-STM
%
% $Id: YA_nu2rhocs.m 2311 2014-10-25 17:17:49Z enh20 $
%       Ed Hartley, Cambridge University, UK.

ecc = cfg.ecc;
ya_k2 = cfg.ya_k2;

out = struct;
out.rho = 1 + ecc*cos(nu);
out.rhoinv = 1/out.rho;
out.rhoinv2 = out.rhoinv*out.rhoinv;
out.c = out.rho*cos(nu);
out.sinnu =sin(nu);
out.s = out.rho*out.sinnu;
out.cprime = -( out.sinnu + ecc*sin(2*nu));
out.sprime = cos(nu) + ecc*cos(2*nu);


out.adc_adc_inv_vec = zeros(1,6);

out.adc_adc_inv_vec(1,1) = out.rho;
out.adc_adc_inv_vec(1,2) = 1./(ya_k2*out.rho);
out.adc_adc_inv_vec(1,3) = -ecc*out.sinnu;
oneoverad = 1/(out.adc_adc_inv_vec(1,1)*out.adc_adc_inv_vec(1,2));

out.adc_adc_inv_vec(1,4) = out.adc_adc_inv_vec(1,2)*oneoverad;
out.adc_adc_inv_vec(1,5) = out.adc_adc_inv_vec(1,1)*oneoverad;
out.adc_adc_inv_vec(1,6) = -out.adc_adc_inv_vec(1,3)*oneoverad;
