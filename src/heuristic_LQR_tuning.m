% BRRIEF:
%   Template for tuning of Q and R matrices for LQR controller as described 
%   in task 6.
% INPUT:
%   n_samples:  Number of samples considered for the tuning.
%   T0:         Initial condition
%   T_sp:       Set point
%   scen:       Disturbance scenario
% OUTPUT:
%   Q, R: Describes stage cost of LQR controller (x^T Q x + u^T R u)

function [Q, R] = heuristic_LQR_tuning(n_samples, T0, T_sp, scen)

%figure(2); set(gcf, 'WindowStyle' ,'docked'); grid on; hold on
%xlabel('Energy consumption [kWh]'); 
%ylabel('Relative norm of steady state deviation');
%...

%for index = 1:n_samples
%...

% Q =
% R =
end
