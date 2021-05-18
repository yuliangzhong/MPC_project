% Regulation MPC - simple regulation controller with FORCESPRO
% 
%  min   xN'*P*xN + sum_{i=1}^{N-1} xi'*Q*xi + ui'*R*ui
% xi,ui
%       s.t. x1 = x
%            x_i+1 = A*xi + B*ui  for i = 1...N-1
%            xmin <= xi <= xmax   for i = 1...N
%            umin <= ui <= umax   for i = 1...N
%
% and P is solution of Ricatti eqn. from LQR problem
%
% (c) Embotech AG, Zurich, Switzerland, 2016.

clear;
close all;

%% system
A = [0.66 0.6; -1.2 0.6];
B = [1; 0.5];
[nx,nu] = size(B);

%% MPC setup

% fix the prediction horizon
N = 10;

% set the stage cost, here we put equal weight on state and input regulation
Q = eye(nx);
R = eye(nu);

% set the bounds on input and state variables
umin = -1; 
umax = 1;

xmin = [-3; -5]; 
% if the bounds are the same for all variables, we can provide a scalar
% instead of a vector
xmax = 5;

% now fill the required structures to generate the controller
system = struct('A', A, 'B', B);

% this assumes Q for all stages including the terminal stage
cost = struct('Q', Q, 'R', R); 
% Alternatively, you can specify that a terminal cost should be computed
% from the discrete-time algebraic Riccati equation for the system:
%     cost.computeTerminalCost = true;
% Or you can specify your own terminal cost:
%     cost.P = 10*eye(nx);

% For this simple MPC interface, only lower and upper bounds for states and
% control inputs are allowed:
constraints = struct('xmin', xmin, 'xmax', xmax, 'umin', umin, 'umax', umax);

% Let FORCESPRO generate an efficient MPC-controller for the specified
% system:
objective.computeTerminalCost = false; % switch to true for LQR terminal cost
controller = SimpleRegulationMPC(system, cost, constraints, N, 'ControlToOrigin_solver');


%% simulate
x1 = [2; -5];
kmax = 20;
X = zeros(nx,kmax+1); X(:,1) = x1;
U = zeros(nu,kmax);

for k = 1:kmax
    % call the controller
    [solverout,exitflag,info] = controller(X(:,k));
    % make sure everything worked out
    if( exitflag == 1 )
        U(:,k) = solverout.u0;
    else
        info
        error('Some problem in solver! exitflag = %i', exitflag);
    end
    
    % apply the optimal control input to the plant
    X(:,k+1) = A*X(:,k) + B*U(:,k);
end

%% plot

figure(1); clf;
% plot the state trajectory
subplot(2,1,1); grid on; title('states'); hold on;
plot([1 kmax], [xmax xmax]', 'k--'); plot([1 kmax], [xmin xmin]', 'k--');
ylim(1.1*[min(xmin),max(xmax)]); 
h = stairs(1:kmax,X(:,1:kmax)');
legend(h, {'State x_1', 'State x_2'});

% plot the input trajectory
subplot(2,1,2);  grid on; title('input'); hold on;
plot([1 kmax], [umax umax]', 'k--'); plot([1 kmax], [umin umin]', 'k--');
ylim(1.1*[min(umin),max(umax)]); stairs(1:kmax,U(:,1:kmax)');