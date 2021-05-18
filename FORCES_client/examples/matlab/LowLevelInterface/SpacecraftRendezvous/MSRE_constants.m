function cfg = MSRE_constants()
%
% CONSTANTS FOR MSR ELLIPTICAL SCENARIO
%
% $Id: MSRE_constants.m 2311 2014-10-25 17:17:49Z enh20 $
% (c) Ed Hartley, Cambridge University, UK.

cfg = struct;


%
% Scenario constants
%
cfg.G = 6.673e-11;                      % Gravitational constant
cfg.M_mars = 6.4191e23;                 % Mass of central body
cfg.ecc = 0.204410;                     % Eccentricity
cfg.a = 4643e3;                         % Semimajor axis a


%
% Calculations
%
cfg.mu = cfg.G*cfg.M_mars;                  % Gravitational parameters
cfg.n = sqrt(cfg.mu/cfg.a^3);                       % Mean anomaly rate
cfg.ecc2 = cfg.ecc*cfg.ecc;
cfg.p = cfg.a*(1 - cfg.ecc2);               % Semi-latus rectum
cfg.rp = cfg.p/(1 + cfg.ecc);               % Periapsis
cfg.v0 = sqrt(2)*sqrt(cfg.mu)*sqrt(1/cfg.rp - 1/(2*cfg.a));  % Velocity at periapsis
cfg.h = cfg.rp*cfg.v0;                      % Angular momentum
cfg.ya_k2 = cfg.h/(cfg.p*cfg.p);            % Useful constant

cfg.X1a = zeros(1,2);
cfg.X1a(1) = sqrt(1-cfg.ecc);            % Precomputed constants
cfg.X1a(2) = sqrt(1+cfg.ecc);            % Precomputed constants
cfg.X2 = cfg.ecc*sqrt(1-cfg.ecc2);      % Precomputed constants
