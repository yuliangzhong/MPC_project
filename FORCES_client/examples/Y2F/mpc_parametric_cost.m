% Basic MPC example demonstrating the use of Yalmip to formulate MPC 
% problems and FORCES to solve them very quickly.
%
% In this example, we will make the cost weights parametric for later
% tuning.
%
% Simple MPC - double integrator example for use with FORCES PRO
% 
%  min   xN'*P*xN + sum_{i=0}^{N-1} xi'*Q*xi + ui'*R*ui
% xi,ui
%       s.t. x0 = x(t)
%            x_i+1 = A*xi + B*ui  for i = 0...N-1
%            xmin <= xi <= xmax   for i = 1...N
%            umin <= ui <= umax   for i = 0...N-1
%
% and P is solution of Ricatti eqn. from LQR problem. The matrices Q,R,P, 
% are parameters to the problem - the tuning of the cost function.
%
% Note: due to 1-based indexing in Matlab, we use 1...N+1 instead of 0...N
%       as indices for state and input trajectory
%
% This file is part of the y2f project: http://github.com/embotech/y2f, 
% a project maintained by embotech under the MIT open-source license.
%
% (c) Gian Ulli and embotech AG, Zurich, Switzerland, 2013-2020.

clear; clc;

modus = 'y2f'; % 'yalmip' or 'y2f'

%% MPC problem data

% system matrices
A = [1.1 1; 0 1];
B = [1; 0.5];
[nx,nu] = size(B);

% horizon
N = 10;

% constraints
umin = -0.5;     umax = 0.5;
xmin = [-5; -5]; xmax = [5; 5];

%% Build MPC problem in Yalmip

% Define variables
X = sdpvar(nx,N+1,'full'); % state trajectory: x0,x1,...,xN (columns of X)
U = sdpvar(nu,N,'full'); % input trajectory: u0,...,u_{N-1} (columns of U)

% Cost matrices - these will be parameters later
Q = sdpvar(nx);
R = sdpvar(nu);
P = sdpvar(nx);

% Initialize objective and constraints of the problem
cost = 0; const = [];

% Assemble MPC formulation
for i = 1:N        
    % cost
    if( i < N )
        cost = cost + 0.5*X(:,i+1)'*Q*X(:,i+1) + 0.5*U(:,i)'*R*U(:,i);
    else
        cost = cost + 0.5*X(:,N+1)'*P*X(:,N+1) + 0.5*U(:,N)'*R*U(:,N);
    end
    
    % model
    const = [const, X(:,i+1) == A*X(:,i) + B*U(:,i)];

    % bounds
    const = [const, umin <= U(:,i) <= umax];
    const = [const, xmin <= X(:,i+1) <= xmax];
end

%% Create controller object (generates code)
% for a complete list of codeoptions, see 
% https://www.embotech.com/FORCES-Pro/User-Manual/Low-level-Interface/Solver-Options
codeoptions = getOptions('parametricCost_solver'); % give solver a name
parameters     = { X(:,1),   Q,   R,   P  };
parameterNames = { 'xinit', 'Q', 'R', 'P' };
outputs     =      U(:,1)   ;
outputNames = {'controlInput'};

if( strcmpi(modus,'yalmip') )
    % standard yalmip optimizer
    controller = optimizer(const, cost, sdpsettings('solver','quadprog'), parameters, outputs);
    goodexitflag = 0; % indicates success of solve
else
    % y2f interface
    controller = optimizerFORCES(const, cost, codeoptions, parameters, outputs, parameterNames, outputNames);
    goodexitflag = 1; % indicates success of solve
end


%% Simulate
x1 = [-4; 2];
kmax = 30;
X = zeros(nx,kmax+1); X(:,1) = x1;
U = zeros(nu,kmax);
problem.z1 = zeros(2*nx,1);

% set cost matrices
Q = eye(2);
R = eye(1);
if exist('dlqr', 'file')
    [~,P] = dlqr(A,B,Q,R);
else
    fprintf('Did not find dlqr (part of the Control Systems Toolbox). Will use 10*Q for the terminal cost matrix.\n');
    P = 10*Q;
end


for k = 1:kmax
    
    % Evaluate controller function for parameters
    [U(:,k),exitflag,info] = controller{ {X(:,k), Q,R,P} };
    
    % Always check the exitflag in case something went wrong in the solver
    if( exitflag == goodexitflag )
        if( strcmpi(modus,'y2f') )
            fprintf('Time step %2d: FORCES took %2d iterations and %5.3f ', k,  info.it, info.solvetime*1000);
            fprintf('milliseconds to solve the problem.\n');
        end
    else
        disp(info);
        error('Some problem in solver');
    end
    
    % State update
    X(:,k+1) = A*X(:,k) + B*U(:,k);
end


%% plot
figure(1); clf;
subplot(2,1,1); grid on; title('states'); hold on;
plot([1 kmax], [xmax xmax]', 'r--'); plot([1 kmax], [xmin xmin]', 'r--');
ylim(1.1*[min(xmin),max(xmax)]); stairs(1:kmax,X(:,1:kmax)');
subplot(2,1,2);  grid on; title('input'); hold on;
plot([1 kmax], [umax umax]', 'r--'); plot([1 kmax], [umin umin]', 'r--');
ylim(1.1*[min(umin),max(umax)]); stairs(1:kmax,U(:,1:kmax)');
