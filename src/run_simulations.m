%% Init

% annotate below when testing Task 24!!!
clear all
close all
clear;
clc;
% annotate above when testing Task 24!!!

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
T0_1 = T_sp + [-2.25;1.75;0.75]; % initial state 1
T0_2 = T_sp + [1.5;2.75;-0.25];  % initial state 2

global ifplot9; % if plot in task 9

%% Open loop sim without control

% % Task 5 
% figure(5); set(gcf, 'WindowStyle' ,'docked');
% simulate_building(T_sp);

%% Unconstrained optimal control

% % Task 6
% disp('Unconstraint optimal control');
% [Q,R] = heuristic_LQR_tuning(2500, T0_1, T_sp, scen1);

% % Task 7
% figure(7); set(gcf, 'WindowStyle' ,'docked');
% clear controller_lqr;
% simulate_building(T0_1, @controller_lqr, Q, R, scen1);
% simulate_building(T0_1, @controller_lqr, 1000000*eye(3), R, scen1);

% % Task 8
% figure(8); set(gcf, 'WindowStyle' ,'docked');
% clear controller_lqr;
% simulate_building(T0_2, @controller_lqr, Q, R, scen1);

%% From LQR to MPC

% % Task 9
% figure(9);grid on; hold on;
% ifplot9 = 1;
% plot3(-2.25,1.75,0.75,'b.','MarkerSize',25);
% plot3(1.5,2.75,-0.25,'b.','MarkerSize',25);
% [A_x, b_x] = compute_X_LQR(Q, R);
% ifplot9 = 0;

% % Task 11
% figure(11); set(gcf, 'WindowStyle' ,'docked');
% clear controller_mpc_1;
% [~,~,J1t1] = simulate_building(T0_1, @controller_mpc_1, Q, R, scen1); % Task11-1
% [~,~,J1t2] = simulate_building(T0_2, @controller_mpc_1, Q, R, scen1); % Task11-2
% save("J1t1.mat", 'J1t1'); save("J1t2.mat", 'J1t2');

%% MPC with guarantees

% % Task 12
% figure(13); set(gcf, 'WindowStyle' ,'docked');
% clear controller_mpc_2;
% [~,~,J2t1] = simulate_building(T0_1, @controller_mpc_2, Q, R, scen1); % Task13-1
% [~,~,J2t2] = simulate_building(T0_2, @controller_mpc_2, Q, R, scen1); % Task13-2
% save("J2t1.mat", 'J2t1'); save("J2t2.mat", 'J2t2');

% % Task 14
% figure(14); set(gcf, 'WindowStyle' ,'docked');
% clear controller_mpc_3;
% [~,~,J3t1] = simulate_building(T0_1, @controller_mpc_3, Q, R, scen1); % Task14-1
% [~,~,J3t2] = simulate_building(T0_2, @controller_mpc_3, Q, R, scen1); % Task14-2
% save("J3t1.mat", 'J3t1'); save("J3t2.mat", 'J3t2');

% % Task15
% load('J1t1.mat');load('J1t2.mat');load('J2t1.mat');load('J2t2.mat');load('J3t1.mat');load('J3t2.mat');
% disp(['opt costs of T0_1 = ' num2str(sum(J1t1)) ',   ' num2str(sum(J2t1)) ',   ' num2str(sum(J3t1))])
% disp(['opt costs of T0_2 = ' num2str(sum(J1t2)) ',   ' num2str(sum(J2t2)) ',   ' num2str(sum(J3t2))])

%% Soft-constrained MPC

% % Task17
% figure(17); set(gcf, 'WindowStyle' ,'docked');
% clear controller_mpc_3;
% simulate_building(T0_1, @controller_mpc_3, Q, R, scen2);

% % Task 18
% figure(18); set(gcf, 'WindowStyle' ,'docked');
% clear controller_mpc_4;
% simulate_building(T0_1, @controller_mpc_4, Q, R, scen2);

% % Task 19
% figure(19); set(gcf, 'WindowStyle' ,'docked');
% clear controller_mpc_3;
% clear controller_mpc_4;
% simulate_building(T0_1, @controller_mpc_3, Q, R, scen1); % Task19-1
% simulate_building(T0_1, @controller_mpc_4, Q, R, scen1); % Task19-2

% % Task 20
% figure(20); set(gcf, 'WindowStyle' ,'docked');
% d = zeros(3,scen2.Nbar + 30);
% d(1,36:50) = -1e4;
% d(2,37:43) = 5e3;
% d(3,45:49) = 1.9e3;
% clear controller_mpc_5;
% simulate_building(T0_1, @controller_mpc_5, Q, R, scen2,1,30,d); % Task20

%% Offset-free MPC

% % Task 23
% disp('Offset-free MPC');
% figure(23); set(gcf, 'WindowStyle' ,'docked');
% clear controller_mpc_3; clear controller_mpc_6;
% simulate_building(T0_1, @controller_mpc_3, Q, R, scen3); % Task23-1
% simulate_building(T0_1, @controller_mpc_6, Q, R, scen3); % Task23-2

%% Comparison using forces

% % Task 24
% % DO NOT clear controller_mpc_1_forces !!
% [~,~,~,t_sim_forces] = simulate_building(T0_2, @controller_mpc_1_forces, Q, R, scen1);
% % DO NOT clear controller_mpc_1 !!
% [~,~,~,t_sim] = simulate_building(T0_2, @controller_mpc_1, Q, R, scen1);
% disp(['t_sim_forces = ' num2str(t_sim_forces) ', t_sim = ' num2str(t_sim)]);
