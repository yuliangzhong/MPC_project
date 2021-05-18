% Input-rate constraints example
% 
%  min   xN'*P*xN + sum_{i=1}^{N-1} xi'*Q*xi + ui'*R*ui + di'*S*di
% xi,ui
%       s.t. x1 = x
%            x_i+1 = A*xi + B*ui            for i = 1...N-1
%            umin <= ui    <= umax          for i = 1...N
%            dumin <= ui-ui-1 <= dumax      for i = 1...N
%
% and P is solution of Ricatti eqn. from LQR problem
%
% (c) Embotech AG, Zurich, Switzerland, 2016.

clear all; clc; clf; close all;

%% System Model
A = [ 0.7115   -0.4345; ...
      0.4345    0.8853 ];
B = [ 0.2173; ...
      0.0573 ];  

%% MPC Setup

[nx,nu] = size(B);

N = 15;
Q = [10,0;0,15];
R = eye(nu);
R_sr = 5*eye(nu);
if( exist('dlqr','file') )
    [~,P] = dlqr(A,B,Q,R);
else
    P = 15*Q;
end

umin = -.5;
umax = 2;

dumin = -1; 
dumax = .5;

%% FORCESPRO multistage form 
%  Optimization variable: zi = [dui; xi+1; ui]

stages = MultistageProblem(N); % get stages struct of length N

for i = 1:N 

    % dimension 
    stages(i).dims.n = nu+nx+nu;     % number of stage variables 
    stages(i).dims.r = nx+nu;        % number of equality constraints 
    stages(i).dims.l = 2;            % number of lower bounds
    stages(i).dims.u = 2;            % number of upper bounds
    
    % lower 
    stages(i).ineq.b.lbidx = [1,4];  
    stages(i).ineq.b.lb = [dumin;umin];    
    
    % upper 
    stages(i).ineq.b.ubidx = [1,4];  
    stages(i).ineq.b.ub = [dumax;umax]; 
    
    % cost 
    if( i == N ) 
        stages(i).cost.H = blkdiag(R_sr,[P, zeros(2,1); zeros(1,2), 0]);
    else
        stages(i).cost.H = blkdiag(R_sr,[Q, zeros(2,1); zeros(1,2), R]);
    end 
    stages(i).cost.f = zeros(nx+2*nu,1); % linear cost terms

    % equality constraints 
    if( i < N ) 
         stages(i).eq.C = [zeros(nx+nu,nu), [A, B; zeros(1,2), 1]]; 
    end 
    if( i>1 ) 
         stages(i).eq.c = zeros(nx+nu,1); 
    end 
    stages(i).eq.D = [[B; 1], -eye(nx+nu)];   
end 

% RHS of first equality constraint is a parameter
parameter(1) = newParam('minusAhat_times_xhat0',1,'eq.c'); 

%% define outputs of the solver
output(1) = newOutput('uhat',1,1);

%% solver settings
codeoptions = getOptions('RateConstraints_Controller');

%% generate code
generateCode(stages,parameter,codeoptions,output);

%% Simulation
x0 = [-2; 6];
kmax = 30;
Xhat = zeros(nx+nu,kmax+1); Xhat(:,1) = [x0; 0];
Uhat = zeros(nu,kmax);
for k = 1:kmax
    problem.minusAhat_times_xhat0 = -[A, B; zeros(1,2), 1]*Xhat(:,k);
    [solverout,exitflag,info] = RateConstraints_Controller(problem);
    if( exitflag == 1 )
        Uhat(:,k) = solverout.uhat;
    else
        info;
        error('Some problem in solver');
    end     
    Xhat(:,k+1) = [A, B; zeros(1,2), 1]*Xhat(:,k) + [B; 1]*Uhat(:,k);
end

%% Figures
h = figure(1); clf;
grid on; title('states','FontSize', 16); hold on;
stairs(0:kmax,Xhat(1:2,1:end)'); h_legend = legend('x1','x2');
set(h_legend,'FontSize',14); ylim([-6 7]);
h_ylabel = ylabel('Magnitude [-]'); set(h_ylabel, 'FontSize', 14);
h_xlabel = xlabel('Simulation Step [-]'); set(h_xlabel, 'FontSize', 14);
hline = findobj(gcf, 'type', 'line');
set(hline,'LineWidth',1.15);
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 5 4])
print(h,'-depsc','RateConstraint1');
h = figure(2); clf;
grid on; title('input','FontSize', 16); hold on;
stairs(0:kmax,Xhat(3,1:end),'k'); hold on;
plot([0 kmax], [umin umin],'r--'); hold on;
plot([0 kmax], [umax umax],'r--'); h_legend = legend('u','umin','umax');
set(h_legend,'FontSize',14); ylim([umin-.2 umax+.2]);
h_xlabel = xlabel('Simulation Step [-]'); set(h_xlabel, 'FontSize', 14);
h_ylabel = ylabel('Magnitude [-]'); set(h_ylabel, 'FontSize', 14);
hline = findobj(gcf, 'type', 'line');
set(hline,'LineWidth',1.15);
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 5 4])
print(h,'-depsc','RateConstraint2');
h = figure(3); clf;
grid on; title('slew rate','FontSize',16); hold on;
stairs(1:kmax, Uhat(:,1:kmax)); hold on;
plot([0 kmax], [dumin dumin],'r--'); hold on;
plot([0 kmax], [dumax dumax],'r--'); hold on;
h_legend = legend('du','dumin','dumax');
set(h_legend,'FontSize',14); ylim([dumin-.2 dumax+.2]);
h_xlabel = xlabel('Simulation Step [-]'); set(h_xlabel, 'FontSize', 14);
h_ylabel = ylabel('Magnitude [-]'); set(h_ylabel, 'FontSize', 14);
hline = findobj(gcf, 'type', 'line');
set(hline,'LineWidth',1.15);
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 5 4])
print(h,'-depsc','RateConstraint3');