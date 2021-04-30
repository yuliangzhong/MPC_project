%% Init
clear all
close all
addpath(genpath(cd));
rng(1234)
load('system/parameters_scenarios.mat')

param = compute_controller_base_parameters;
T_sp = param.T_sp;
T0_1 = T_sp + [-2.25;1.75;0.75];
% T_sp = ...
% dT0_exmaple = ...
% T0_example = ...
%% Open loop sim without control (Task 5)
% figure; set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T_sp);

%% Example
% figure; set(gcf, 'WindowStyle' ,'docked');
% clear persisten variables of function controller_example
% clear controller_example
% execute simulation
% [T,~,~,t] = simulate_building(T0_example,@controller_example);


%% Unconstrained optimal control
disp('Unconstraint optimal control');

% Uncontrolled system
%figure(1); set(gcf, 'WindowStyle' ,'docked');
%...

% Tuning of LQR on first initial condition
[Q,R] = heuristic_LQR_tuning(2500, T0_1, T_sp, scen1);

%% From LQR to MPC
disp('First MPC'); 

%% MPC with guarantees
disp('MPC with guarantees');




%% Soft-constrained MPC
disp('Soft-constrained MPC');




%% Offset-free MPC
disp('Offset-free MPC');




%% Comparison using forces
disp('MPC Implementation with FORCES Pro');
