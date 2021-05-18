%% Time-Varying Model with FORCESPRO
%  
% (c) Gian Koenig, Embotech AG, Zurich, Switzerland, 2014.

clear all; clc; close all;

%% Systems
A1 = [0.7115, -.6; .6, 0.8853]; 
B1 = [0.02713; 0.0573];
C1 = [0, 1];
D1 = [0];

A2 = [.9, .5; .5, 1]; 
B2 = [0; 1/15];
C2 = [0, 1/5];
D2 = [0];

A3 = [0.7115, -.5; .5, 1]; 
B3 = [0.5; 0.01];
C3 = [0, 1];
D3 = [0];

A4 = [0, .9; -1, 0]; 
B4 = [0; 1/5];
C4 = [0, 1];
D4 = [0];
                
%% Input Constraints
u1min = -3;     u1max = 5;      
u2min = -5.5;   u2max = 5.5;
u3min = -3;     u3max = 5;
u4min = -.45;   u4max = 4.5;

%% Simulation Settings
% 'Switching Frequency' of the system: Linear system changes every s steps
s1 = 4;    s2 = 8;     s3 = 3;     s4 = 5;

% Initial condition
x1 = [1; 1];

% Prediction/Preview Horizon
N = 15;

% Simulation steps
n = 40;

%% Others
system{1,1} = A1; system{1,2} = A2; system{1,3} = A3; system{1,4} = A4;
system{2,1} = B1; system{2,2} = B2; system{2,3} = B3; system{2,4} = B4;
system{3,1} = C1; system{3,2} = C2; system{3,3} = C3; system{3,4} = C4;
system{4,1} = D1; system{4,2} = D2; system{4,3} = D3; system{4,4} = D4;

system_var1 = repmat(system(:,1),1,s1); system_var2 = repmat(system(:,2),1,s2);
system_var3 = repmat(system(:,3),1,s3); system_var4 = repmat(system(:,4),1,s4);

system_var = [system_var1, system_var2, system_var3, system_var4];
system_var = repmat(system_var,1, ceil((n+N)/length(system_var)));

constraints{1,1} = u1min; constraints{2,1} = u1max;
constraints{1,2} = u2min; constraints{2,2} = u2max;
constraints{1,3} = u3min; constraints{2,3} = u3max;
constraints{1,4} = u4min; constraints{2,4} = u4max;

constraints_var1 = repmat(constraints(:,1),1,s1);
constraints_var2 = repmat(constraints(:,2),1,s2);
constraints_var3 = repmat(constraints(:,3),1,s3);
constraints_var4 = repmat(constraints(:,4),1,s4);

constraints_var = [constraints_var1, constraints_var2, ...
                   constraints_var3, constraints_var4];
constraints_var = repmat(constraints_var,1,ceil((n+N)/length(constraints_var)));

%% MPC Setting
% Number of states, Number of inputs
[nx, nu] = size(B1); 

% Cost weights
R = 1; % on inputs
Q = 10*eye(2); % on states (including final stage)

%% Multistage Problem: No varying model in prediction horizon
stages = MultistageProblem(N); % get stages struct of length N

for i = 1:N
    % dimensions
    stages(i).dims.n = nx+nu;   % number of stage variables
    stages(i).dims.r = nx;      % number of equality constraints
    stages(i).dims.l = nu;      % number of lower bounds
    stages(i).dims.u = nu;      % number of upper bounds
    
    % lower bounds
    stages(i).ineq.b.lbidx = 1; % lower bound acts on these indices
    
    % upper bounds
    stages(i).ineq.b.ubidx = 1; % upper bound acts on these indices
    
    % cost
    stages(i).cost.H = blkdiag(R,Q);
    stages(i).cost.f = zeros(nx+nu,1);
    
    % inequality constraints
    if( i>1 )
        stages(i).eq.c = zeros(nx,1);
    end
end

% PARAMETERS
% Initial Equality
% c_1 = -A*x0
parameter(1) = newParam('minusA_times_x0',1,'eq.c');

% Inter-Stage Equality
parameter(2)  = newParam('D',1:N,'eq.D');
parameter(3)  = newParam('C',1:(N-1),'eq.C');

% Lower and upper bounds on the input
parameter(4)  = newParam('umin',1:N,'ineq.b.lb');
parameter(5)  = newParam('umax',1:N,'ineq.b.ub');

% define outputs of the solver
outputs(1) = newOutput('u0',1,1:nu);

% solver settings
codeoptions = getOptions('Time_Varying_Model');

% generate code
generateCode(stages,parameter,codeoptions,outputs);

% Simulation
X = zeros(nx,n+1); X(:,1) = x1;
X_NC = zeros(nx,n+1); X_NC(:,1) = x1;
U = zeros(nu,n);
problem.z1 = zeros(2*nx,1);
for k = 1:n
    
    problem.minusA_times_x0 = -system_var{1,k}*X(:,k);
    problem.D = [system_var{2,k}   , -eye(2)];
    problem.C = [zeros(nx,nu), system_var{1,k}];
    problem.umin  = constraints_var{1,k};
    problem.umax  = constraints_var{2,k};
    
    [solverout,exitflag,info] = Time_Varying_Model(problem);
    if( exitflag == 1 )
        U(:,k) = solverout.u0;
    else
        info
        error('Some problem in solver');
    end
    X_NC(:,k+1) = system_var{1,k}*X_NC(:,k);
    X(:,k+1) = system_var{1,k}*X(:,k) + system_var{2,k}*U(:,k);
end

%% Multistage Problem: Varying Model in prediction Horizon
stages = MultistageProblem(N); % get stages struct of length N

% Initial Equality
% c_1 = -A*x0
parameter(1) = newParam('minusA_times_x0',1,'eq.c');

for i = 1:N
    % dimension
    stages(i).dims.n = nx+nu;   % number of stage variables
    stages(i).dims.r = nx;      % number of equality constraints
    stages(i).dims.l = nu;      % number of lower bounds
    stages(i).dims.u = nu;      % number of upper bounds
    
    % lower bounds
    stages(i).ineq.b.lbidx = 1; % lower bound acts on these indices
    parameter(1+i) = newParam(['u',num2str(i),'min'],i,'ineq.b.lb');
    
    % upper bounds
    stages(i).ineq.b.ubidx = 1; % upper bound acts on these indices
    parameter(1+N+i) = newParam(['u',num2str(i),'max'],i,'ineq.b.ub');
    
    % cost
    stages(i).cost.H = blkdiag(R,Q);
    stages(i).cost.f = zeros(nx+nu,1);
    
    % Equality constraints
    if( i>1 )
        stages(i).eq.c = zeros(nx,1);
    end
    % Inter-Stage Equlity
    % D_i*z_i = [B_i -I]*z_i
    parameter(1+2*N+i)  = newParam(['D_',num2str(i)],i,'eq.D');
    if( i<N )
        % C_{i-1}*z_{i-1} = [0 A_i]*z_{i-1}
        parameter(1+3*N+i) = newParam(['C_',num2str(i)],i,'eq.C');
    end
end

% define outputs of the solver
outputs(1) = newOutput('u0',1,1);

% solver settings
codeoptions = getOptions('Time_Varying_Model_wP');
% generate code
generateCode(stages,parameter,codeoptions,outputs);

% Simulation
X_wP = zeros(nx,n+1); X_wP(:,1) = x1;
U_wP = zeros(nu,n);
problem.z1 = zeros(2*nx,1);
for k = 1:n
    % Initial Equality
    problem.minusA_times_x0 = -system_var{1,k}*X_wP(:,k);
    % Preview Information
    % Inter-Stage Constraints: D_i
    for i=1:N
        problem.(['D_',num2str(i)]) = [system_var{2,k-1+i}, -eye(2)]; 
    end     
    % Inter-Stage Constraints: C_{i-1}
    for i=1:N-1
        problem.(['C_',num2str(i)]) = [zeros(nx,nu), system_var{1,k+i}];
    end
    
    % Preview on Constraints
    for i=1:N
        problem.(['u',num2str(i),'min']) = constraints_var{1,k-1+i}; 
        problem.(['u',num2str(i),'max']) = constraints_var{2,k-1+i}; 
    end 
    
    [solverout,exitflag,info] = Time_Varying_Model_wP(problem);
    if( exitflag == 1 )
        U_wP(:,k) = solverout.u0;
    else
        info
        error('Some problem in solver');
    end
    X_wP(:,k+1) = system_var{1,k}*X_wP(:,k) + system_var{2,k}*U_wP(:,k);
end

%% Plots

input_lowerbound = nan(1, length(constraints_var));
for i = 1:length(constraints_var)
    input_lowerbound(i) = constraints_var{1,i};
end
input_upperbound = nan(1, length(constraints_var));
for i = 1:length(constraints_var)
    input_upperbound(i) = constraints_var{2,i};
end


% States Time-Varying Model vs Basic MPC
h = figure(1);
stairs(0:n,X(1,1:n+1)','m'); hold on;
stairs(0:n,X(2,1:n+1)','m--'); hold on;
stairs(0:n,X_wP(1,1:n+1)','b'); hold on;
stairs(0:n,X_wP(2,1:n+1)','b--'); hold on;
ylim([-6 6]);
h_legend=legend('state 1', 'state 2','state 1: Time-Varying', 'state 2: Time-Varying'); grid on;
set(h_legend,'FontSize',14);
title('States Time-Varying MPC vs Basic MPC', 'FontSize', 16); 
h_xlabel = xlabel('step [-]'); h_ylabel = ylabel('Magnitude [-]');
set(h_xlabel, 'FontSize', 14); set(h_ylabel, 'FontSize', 14);
hline = findobj(gcf, 'type', 'stair');
set(hline(1:4),'LineWidth',1.15);

% Input Time-Varying Model vs Basic MPC
h = figure(2);
stairs(0:n-1,U(:,1:n)','m'); hold on; stairs(0:n-1,U_wP(:,1:n)','b');
stairs(0:n, input_upperbound(1:n+1), 'r--'); stairs(0:n, input_lowerbound(1:n+1), 'r--');
grid on; ylim([-6., 6.]);
h_legend=legend('input u', 'input u: Time-Varying'); grid on;
set(h_legend,'FontSize',14);
title('Input Time-Varying MPC vs Basic MPC', 'FontSize', 16); 
h_xlabel = xlabel('step [-]'); h_ylabel = ylabel('Magnitude [-]');
set(h_xlabel, 'FontSize', 14); set(h_ylabel, 'FontSize', 14);
hline = findobj(gcf, 'type', 'stair');
set(hline(1:4),'LineWidth',1.15);
