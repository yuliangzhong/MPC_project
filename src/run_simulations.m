%% Init
clear all
close all
addpath(genpath(cd));
rng(1234)
load('system/parameters_scenarios.mat')

%% Example
% figure; set(gcf, 'WindowStyle' ,'docked');
% clear persisten variables of function controller_example
% clear controller_example
% execute simulation
% [T,~,~,t] = simulate_building(T0_example,@controller_example);

%% Config here
param = compute_controller_base_parameters;
load('Q_sim.mat')
load('R_sim.mat')
T_sp = param.T_sp;
T0_1 = T_sp + [-2.25;1.75;0.75];
T0_2 = T_sp + [1.5;2.75;-0.25];

%% Open loop sim without control (Task 5)

% figure(5); set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T_sp);

%% Unconstrained optimal control
% disp('Unconstraint optimal control');
% [Q,R] = heuristic_LQR_tuning(2500, T0_1, T_sp, scen1);
% 
% figure(7); set(gcf, 'WindowStyle' ,'docked');
% clear controller_lqr;
% simulate_building(T0_1, @controller_lqr, Q, R, scen1); % Task7
% 
% figure(8); set(gcf, 'WindowStyle' ,'docked');
% clear controller_lqr;
% simulate_building(T0_2, @controller_lqr, Q, R, scen1); % Task8
%% From LQR to MPC
% disp('First MPC'); 
% [A_x, b_x] = compute_X_LQR(Q, R);
% % Task10
% [K,S,e] = dlqr(param.A, param.B, Q, R);
% J0_1 = (T0_1-T_sp)'*S*(T0_1-T_sp);
% J0_2 = (T0_2-T_sp)'*S*(T0_2-T_sp);
% disp(J0_1)
% disp(J0_2)
% Task11
figure(11); set(gcf, 'WindowStyle' ,'docked');
clear controller_mpc_1;
simulate_building(T0_1, @controller_mpc_1, Q, R, scen1); % Task11-1
simulate_building(T0_2, @controller_mpc_1, Q, R, scen1); % Task11-2
%% MPC with guarantees
disp('MPC with guarantees');




%% Soft-constrained MPC
disp('Soft-constrained MPC');




%% Offset-free MPC
disp('Offset-free MPC');




%% Comparison using forces
disp('MPC Implementation with FORCES Pro');
