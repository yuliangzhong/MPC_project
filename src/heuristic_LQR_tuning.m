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

% define a figure
figure; set(gcf, 'WindowStyle' ,'docked'); grid on; hold on
xlabel('Energy consumption [kWh]'); 
ylabel('Relative norm of steady state deviation');
% define R and bestQ
R = eye(3); Q = eye(3);
% define dT_rel
dT_rel = 999;

% Loop
for index = 1:n_samples
    Q_ind = diag([randi([1,10e6]),randi([1,10e6]),randi([1,10e6])]); % randomly sample Q
    clear controller_lqr;
    [T,p,~,~,T_v,p_v] = simulate_building(T0,@controller_lqr,Q_ind,R,scen,0);
    dT_relative = norm(T_sp-T(:,15))/norm(T_sp-T0);
    power_sum = sum(abs(p),'all')/1000/60;
    if T_v
        scatter(power_sum, dT_relative, 'red');
    elseif p_v
        scatter(power_sum, dT_relative, 'blue');
    else
        scatter(power_sum, dT_relative, 'green');
    end
    if power_sum<16
        if dT_relative<dT_rel
            dT_rel = dT_relative;
            Q = Q_ind;
        end
    end
end
end
