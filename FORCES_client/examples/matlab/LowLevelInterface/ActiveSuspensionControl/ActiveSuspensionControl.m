%% Active Suspension Control
%
% Model Predictive Control Designed with FORCESPRO
%
% (c) Gian Koenig, Embotech AG, Zurich, Switzerland, 2014.

clear all; clc; close all;

%% System
    
% output vector (3 outputs)
% - zb_ddot     : heave acceleration 
% - phi_ddot    : pitch acceleration 
% - theta_ddot  : roll acceleration

% REDUCED MODEL : state vector (6 states)
% - zb          : heave displacement 
% - phi         : pitch displacement
% - theta       : roll displacement
% - zb_dot      : heave velocity 
% - phi_dot     : pitch velocity
% - theta_dot   : roll velocity

load('Active_Suspension_Control_Model.mat')

%% Road Bump

n = 3600;
t=[0:.005:n*.005-.005]';
road1 = zeros(length(t),1);
road2 = zeros(length(t),1);

for i = 1:20
    k = i+160;
    road1(k,1) = i*.005;
end
for i = 1:20
    k = i+180;
    road1(k,1) = .1 - i*.005;
end

%% Simulate the passive model

sim('Active_Suspension_Control_NC')

% System Response: output_nocontrol
Y_NC = output_nocontrol.Data;
t_sim = output_nocontrol.Time;

% System Input: w_ii, w_dot_ii, ii = {fl, fr, rl, rr}
w = [w.signal1.Data, w.signal2.Data,...
    w.signal3.Data,w.signal4.Data...
    w_dot.signal1.Data,w_dot.signal2.Data,...
    w_dot.signal3.Data,w_dot.signal4.Data]';

%% Measurement data for control and for plotting
% Reduce Resolution
N=20;
kmax_pre = 720;
w_pre_temp = [w zeros(8,N)];
w_pre = [zeros(8,kmax_pre) zeros(8,N)];

t_pre = (0:0.025:kmax_pre*0.025-0.025)';
k=1;
for i = 1:kmax_pre
    w_pre(:,i) = w_pre_temp(:,k+1);
    k = 5*i;
end

%% MPC Setup
nx = 6;
nu = 4; 

N = 20;
Q = diag([50,50,50,50,50,50]);
R = eye(nu);
if( exist('dlqr','file') )
    [~,P] = dlqr(Ad,Bdu,Q,R);
else
    P = 20*Q;
end
umin = -.04;     umax = .04;

%% MPC without Preview
% FORCESPRO multistage form
% assume variable ordering zi = [ui; xi+1] for i=1...N-1

% Parameter
parameter = newParam('minusA_times_x0_minusBw_times_w',1,'eq.c');

stages = MultistageProblem(N);
for i = 1:N
    
        % dimension
        stages(i).dims.n = nx+nu; % number of stage variables
        stages(i).dims.r = nx;    % number of equality constraints        
        stages(i).dims.l = nu; % number of lower bounds
        stages(i).dims.u = nu; % number of upper bounds
        
        % cost
        if( i == N )
            stages(i).cost.H = blkdiag(R,P);
        else
            stages(i).cost.H = blkdiag(R,Q);
        end
        stages(i).cost.f = zeros(nx+nu,1);
        
        % lower bounds
        stages(i).ineq.b.lbidx = 1:nu; % lower bound acts on these indices
        stages(i).ineq.b.lb = umin*ones(4,1); % lower bound for the input signal
    
        % upper bounds
        stages(i).ineq.b.ubidx = 1:nu; % upper bound acts on these indices
        stages(i).ineq.b.ub = umax*ones(4,1); % upper bound for the input signal

        % equality constraints
        if( i < N )
            stages(i).eq.C =  [zeros(nx,nu), Ad];
        end
        if( i>1 )
            stages(i).eq.c = zeros(nx,1);
        end
        stages(i).eq.D = [Bdu, -eye(nx)];
        
end
  
% define outputs of the solver
outputs(1) = newOutput('u0',1,1:nu);

% solver settings
codeoptions = getOptions('VEHICLE_MPC_noPreview');

% generate code
generateCode(stages,parameter,codeoptions,outputs);

% simulate
x1 = zeros(6,1);
X_NP = zeros(nx,kmax_pre+1); X_NP(:,1) = x1;
Y_NP = zeros(3,kmax_pre);
U_NP = zeros(nu,kmax_pre);
problem.z1 = zeros(2*nx,1);
for k = 1:kmax_pre
    problem.minusA_times_x0_minusBw_times_w = -Ad*X_NP(:,k)-Bdw*[w_pre(:,k)];
    [solverout,exitflag,info] = VEHICLE_MPC_noPreview(problem);
    if( exitflag == 1 )
        U_NP(:,k) = solverout.u0;
    else
        info
        error('Some problem in solver');
    end
    X_NP(:,k+1) = Ad*X_NP(:,k) + [Bdu, Bdw]*[U_NP(:,k); w_pre(:,k)];
    Y_NP(:,k) = Cd*X_NP(:,k) + Dd*[U_NP(:,k); w_pre(:,k)];
end

N = 20;

%% MPC with Preview

% FORCESPRO multistage form
% assume variable ordering zi = [ui; xi+1] for i=1...N-1

% Parameters: First Eq. RHS
parameter(1) = newParam('minusA_times_x0_minusBw_times_w_pre',1,'eq.c');

stages = MultistageProblem(N);
for i = 1:N
    
        % dimension
        stages(i).dims.n = nx+nu; % number of stage variables
        stages(i).dims.r = nx;    % number of equality constraints        
        stages(i).dims.l = nu; % number of lower bounds
        stages(i).dims.u = nu; % number of upper bounds
        
        % cost
        if( i == N )
            stages(i).cost.H = blkdiag(R,P);
        else
            stages(i).cost.H = blkdiag(R,Q);
        end
        stages(i).cost.f = zeros(nx+nu,1);
        
        % lower bounds
        stages(i).ineq.b.lbidx = 1:nu; % lower bound acts on these indices
        stages(i).ineq.b.lb = umin*ones(4,1); % lower bound for the input signal
    
        % upper bounds
        stages(i).ineq.b.ubidx = 1:nu; % upper bound acts on these indices
        stages(i).ineq.b.ub = umax*ones(4,1); % upper bound for the input signal

        % equality constraints
        if( i < N )
            stages(i).eq.C =  [zeros(nx,nu), Ad];
        end
        stages(i).eq.D = [Bdu, -eye(nx)];
        
        % Parameters for Preview
        if (i < N)
            parameter(i+1)  = newParam(['pre',num2str(i+1),'_w'],i+1,'eq.c');
        end
        
end
  
% define outputs of the solver
outputs(1) = newOutput('u0',1,1:nu);

% solver settings
codeoptions = getOptions('VEHICLE_MPC_withPreview');

% generate code
generateCode(stages,parameter,codeoptions,outputs);

% simulate
x1 = zeros(6,1);
X_wP = zeros(nx,kmax_pre+1); X_wP(:,1) = x1;
Y_wP = zeros(3,kmax_pre);
U_wP = zeros(nu,kmax_pre);
problem.z1 = zeros(2*nx,1);
for k = 1:kmax_pre
    problem.minusA_times_x0_minusBw_times_w_pre = -Ad*X_wP(:,k)-Bdw*[w_pre(:,k)];
    problem.pre2_w  = -Bdw*w_pre(:,k+1);  problem.pre3_w  = -Bdw*w_pre(:,k+2);
    problem.pre4_w  = -Bdw*w_pre(:,k+3);  problem.pre5_w  = -Bdw*w_pre(:,k+4);
    problem.pre6_w  = -Bdw*w_pre(:,k+5);  problem.pre7_w  = -Bdw*w_pre(:,k+6);
    problem.pre8_w  = -Bdw*w_pre(:,k+7);  problem.pre9_w  = -Bdw*w_pre(:,k+8);
    problem.pre10_w = -Bdw*w_pre(:,k+9);  problem.pre11_w = -Bdw*w_pre(:,k+10);
    problem.pre12_w = -Bdw*w_pre(:,k+11); problem.pre13_w = -Bdw*w_pre(:,k+12);
    problem.pre14_w = -Bdw*w_pre(:,k+13); problem.pre15_w = -Bdw*w_pre(:,k+14);
    problem.pre16_w = -Bdw*w_pre(:,k+15); problem.pre17_w = -Bdw*w_pre(:,k+16);
    problem.pre18_w = -Bdw*w_pre(:,k+17); problem.pre19_w = -Bdw*w_pre(:,k+18);
    problem.pre20_w = -Bdw*w_pre(:,k+19);
    [solverout,exitflag,info] = VEHICLE_MPC_withPreview(problem);
    if( exitflag == 1 )
        U_wP(:,k) = solverout.u0;
    else
        info
        error('Some problem in solver');
    end
    X_wP(:,k+1) = Ad*X_wP(:,k) + [Bdu, Bdw]*[U_wP(:,k); w_pre(:,k)];
    Y_wP(:,k) = Cd*X_wP(:,k) + Dd*[U_wP(:,k); w_pre(:,k)];
end

%% Plots

% Road Disturbance
h = figure(1);
plot(t_sim, w(2,:)); grid on; 
title('Speed Bump: Front Right Wheel', 'FontSize', 16); 
ylim([0, .11]); xlim([0.3, 1.5])
h_xlabel = xlabel('Time'); h_ylabel = ylabel('Road w_{fr}');
set(h_xlabel, 'FontSize', 14); set(h_ylabel, 'FontSize', 14);
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 10 2]);

% Heave Acceleration
h = figure(2);
stairs(t_pre, Y_wP(1,:),'b'); hold on; stairs(t_pre, Y_NP(1,:),'r'); grid on; 
title('Heave Acceleration', 'FontSize', 16); 
ylim([-2, 2]); xlim([.3, 1.5])
h_xlabel = xlabel('Time'); h_ylabel = ylabel('Heave Acceleration [m/s^2]');
set(h_xlabel, 'FontSize', 14); set(h_ylabel, 'FontSize', 14);
h_legend=legend('heave acc with preview', 'heave acc without preview','Location','SouthWest'); grid on;
set(h_legend,'FontSize',14);

% Pitch Acceleration
h = figure(3);
stairs(t_pre, Y_wP(2,:),'b'); hold on; stairs(t_pre, Y_NP(2,:),'r'); grid on; 
title('Pitch Acceleration', 'FontSize', 16); 
ylim([-2, 2]); xlim([.3, 1.5])
h_xlabel = xlabel('Time'); h_ylabel = ylabel('Pitch Acceleration [m/s^2]');
set(h_xlabel, 'FontSize', 14); set(h_ylabel, 'FontSize', 14);
h_legend=legend('pitch acc with preview', 'pitch acc without preview','Location','SouthWest'); grid on;
set(h_legend,'FontSize',14);

% Roll Acceleration
h = figure(4); 
stairs(t_pre, Y_wP(3,:),'b'); hold on; stairs(t_pre, Y_NP(3,:),'r'); grid on; 
title('Roll Acceleration', 'FontSize', 16); 
ylim([-2, 2]); xlim([.3, 1.5])
h_xlabel = xlabel('Time'); h_ylabel = ylabel('Roll Acceleration [m/s^2]');
set(h_xlabel, 'FontSize', 14); set(h_ylabel, 'FontSize', 14);
h_legend=legend('roll acc with preview', 'roll acc without preview','Location','SouthWest'); grid on;
set(h_legend,'FontSize',14);

% Input FL
h = figure(5);
stairs(t_pre, U_wP(1,:),'b');  hold on;
plot([0 kmax_pre], [umax umax]', 'r--'); hold on;
plot([0 kmax_pre], [umin umin]', 'r--'); grid on;  
title('Input Front Left Actuator', 'FontSize', 16); 
ylim([-.045, .045]); xlim([.3, 1.5])
h_xlabel = xlabel('Time'); h_ylabel = ylabel('Actuator [m]');
set(h_xlabel, 'FontSize', 14); set(h_ylabel, 'FontSize', 14);
h_legend=legend('input fl','umax','umin', 'Location','SouthWest'); grid on;
set(h_legend,'FontSize',14);

% Input FR
h = figure(6);
stairs(t_pre, U_wP(2,:),'b');  hold on;
plot([0 kmax_pre], [umax umax]', 'r--'); hold on;
plot([0 kmax_pre], [umin umin]', 'r--'); grid on;  
title('Input Front Right Actuator', 'FontSize', 16); 
ylim([-.045, .045]); xlim([.3, 1.5])
h_xlabel = xlabel('Time'); h_ylabel = ylabel('Actuator [m]');
set(h_xlabel, 'FontSize', 14); set(h_ylabel, 'FontSize', 14);
h_legend=legend('input fr','umax','umin', 'Location','SouthWest'); grid on;
set(h_legend,'FontSize',14);

% Input RL
h = figure(7);
stairs(t_pre, U_wP(3,:),'b');  hold on;
plot([0 kmax_pre], [umax umax]', 'r--'); hold on;
plot([0 kmax_pre], [umin umin]', 'r--'); grid on;  
title('Input Rear Left Actuator', 'FontSize', 16); 
ylim([-.045, .045]); xlim([.3, 1.5])
h_xlabel = xlabel('Time'); h_ylabel = ylabel('Actuator [m]');
set(h_xlabel, 'FontSize', 14); set(h_ylabel, 'FontSize', 14);
h_legend=legend('input rl','umax','umin', 'Location','SouthWest'); grid on;
set(h_legend,'FontSize',14);

% Input RR
h = figure(8);
stairs(t_pre, U_wP(4,:),'b');  hold on;
plot([0 kmax_pre], [umax umax]', 'r--'); hold on;
plot([0 kmax_pre], [umin umin]', 'r--'); grid on; 
title('Input Rear Right Actuator', 'FontSize', 16); 
ylim([-.045, .045]); xlim([.3, 1.5])
h_xlabel = xlabel('Time'); h_ylabel = ylabel('Actuator [m]');
set(h_xlabel, 'FontSize', 14); set(h_ylabel, 'FontSize', 14);
h_legend=legend('input rr','umax','umin', 'Location','SouthWest'); grid on;
set(h_legend,'FontSize',14);
