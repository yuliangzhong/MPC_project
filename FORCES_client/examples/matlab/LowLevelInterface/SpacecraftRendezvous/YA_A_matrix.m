function [A, nu1] = YA_A_matrix(cfg, nu0, dT)
% [A, nu1] = YA_A_matrix(cfg, nu0, dT)
%
%   Compute Yamanaka-Ankersen state transition matrix for relative position
%   in an elliptical Keplerian orbit.
%
%   Inputs:
%           cfg:        Orbital parameters and some pre-computed variables
%           nu0:        True anomaly at beginning of prediction
%           dT:         Prediction period
%
%   Outputs:
%           A:          State transition matrix
%           nu1:        True anomaly at end of prediction
%
%
% Example usage:
%
%       cfg = MSRE_constants;           % Get orbit parameters
%       nu0 = 0;                        % Initialise true anomaly
%       dT  = 200;                      % Predict over 200 s
%
%       [A, nu1] = YA_A_matrix(cfg, nu0, dT);   % Compute YA STM
%
%  Reference:
%           K. Yamanaka and F. Ankersen, ?New state transition matrix for
%           relative motion on an arbitrary elliptical orbit,? J. Guidance,
%           Control, and Dynamics, vol. 25, no. 1, pp. 60?66, 2002.
%
%
% $Id: YA_A_matrix.m 2309 2014-10-25 17:14:33Z enh20 $
% (c) Ed Hartley, Cambridge University, UK.

% Eccentricity
ecc = cfg.ecc;
ecc2 = cfg.ecc2;

% Compute functions of true-anomaly
nu_processed_k = YA_nu2rhocs(cfg,nu0);
rhoinv_k = nu_processed_k.rhoinv;
rhoinv2_k = nu_processed_k.rhoinv2;
c_k = nu_processed_k.c;
s_k = nu_processed_k.s;
rho_k = nu_processed_k.rho;
PHIinv_prev = 1/(1-ecc2)*...
    [ 1-ecc2,   0,  3*ecc*s_k*(rhoinv_k + rhoinv2_k),   -ecc*s_k*(1+rhoinv_k),  0,  -ecc*c_k+2
    0,          0,  0,                                  0,                      0,  0
    0,          0,  -3*s_k*(rhoinv_k + ecc2*rhoinv2_k), s_k*(1+rhoinv_k),       0,  c_k-2*ecc
    0,          0,  -3*(c_k*rhoinv_k + ecc),            c_k*(1+rhoinv_k)+ecc,   0,  -s_k
    0,          0,   0,                                 0,                      0,  0
    0,          0,   3*rho_k + ecc2 - 1,                -rho_k*rho_k,           0,  ecc*s_k];
PHIinv_prev(2,2) = 1;
PHIinv_prev(5,5) = 1;


% Propagate the true anomaly
Mk = kepler_nu2M(ecc, ecc2, nu0);           % Convert true anomaly to mean anomaly
n = kepler_M_rate(cfg.mu, cfg.a);           % Compute mean anomaly rate
nu1 = kepler_propagate_nu(Mk, n, ecc, ecc2, dT);  % Compute propagated true anomaly

% Compute functions of true anomaly
nu_processed_k1 = YA_nu2rhocs(cfg,nu1);
rhoinv_k1 = nu_processed_k1.rhoinv;
rhoinv2_k1 = nu_processed_k1.rhoinv2;
c_k1 = nu_processed_k1.c;
s_k1 = nu_processed_k1.s;
cprime_k1 = nu_processed_k1.cprime;
sprime_k1 = nu_processed_k1.sprime;
rho_k1 = nu_processed_k1.rho;
rho2_k1 = rho_k1^2;
J_k1 = cfg.ya_k2*dT;

% Compute functions of true anomaly difference
c_dk = cos( nu1 - nu0);
s_dk = sin( nu1 - nu0);


PHI_k1 = [1,    0,      -c_k1*(1+rhoinv_k1),    s_k1*(1+rhoinv_k1), 0,      3*rho2_k1*J_k1
    0,          c_dk,   0,                      0,                  s_dk,   0
    0,          0,      s_k1,                   c_k1,               0,      (2-3*ecc*s_k1*J_k1)
    0,          0,      2*s_k1,                 2*c_k1-ecc,         0,      3*(1-2*ecc*s_k1*J_k1)
    0,          -s_dk,  0,                      0,                  c_dk,   0
    0,          0,      sprime_k1,              cprime_k1,          0,      -3*ecc*(sprime_k1*J_k1 + s_k1*rhoinv2_k1)];




transk = [nu_processed_k.adc_adc_inv_vec(1,1)*eye(3) zeros(3); ...
    nu_processed_k.adc_adc_inv_vec(1,3)*eye(3) nu_processed_k.adc_adc_inv_vec(1,2)*eye(3)];

itransk1 = [nu_processed_k1.adc_adc_inv_vec(1,4)*eye(3) zeros(3); ...
    nu_processed_k1.adc_adc_inv_vec(1,6)*eye(3) nu_processed_k1.adc_adc_inv_vec(1,5)*eye(3)];


A = itransk1*PHI_k1*PHIinv_prev*transk;
