% This example demonstrates how to use the SQP_NLP real-time iteration solver on a robotic manipulator
% with two links and two joints.
%
% The robotic manipulator must track a reference on its joint angles and their
% velocities while minimizing the input torques and satisfying state and input constraints.
%
% Variables are collected stage-wise into z = [dtau1 dtau2 theta1 dtheta1 theta2 dtheta2 tau1 tau2].
%
% (c) Embotech AG, Zurich, Switzerland, 2013-2021.
close all;
clc;

%% Problem dimensions
model.N     = 21;   % horizon length
model.nvar  = 8;    % number of variables
model.neq   = 6;    % number of equality constraints
model.nh    = 0;    % number of inequality constraint functions
model.npar  = 1;    % reference for angles

nx = 6;
nu = 2;
Tf = 2;

Tsim = 20;
Ns = Tsim/(Tf/(model.N-1));

%% Objective function
% A tracking cost is defined to minimize the distance of the angles to the
% reference 
model.LSobjective = @(z,p)[sqrt(1000) * (z(3)-p(1)*1.2); ...
                           sqrt(0.1) * z(4);...
                           sqrt(1000) * (z(5)+p(1)*1.2);...
                           sqrt(0.1) * z(6);...
                           sqrt(0.01) * z(7);...
                           sqrt(0.01) * z(8);...
                           sqrt(0.01) * z(1);...
                           sqrt(0.01) * z(2)];
Q = diag([1000 0.1 1000 0.1 0.01 0.01]);
R = diag([0.01 0.01]);
                       
%% Dynamics, i.e. equality constraints 
integrator_stepsize = Tf/(model.N-1);
model.continuous_dynamics = @dynamics;

% Indices on LHS of dynamical constraint - for efficiency reasons, make
% sure the matrix E has structure [0 I] where I is the identity matrix.
model.E = [zeros(nx,nu), eye(nx)];

%% Inequality constraints
% upper/lower variable bounds lb <= x <= ub
model.lb = [ -200,  -200,     -pi,  -100,  -pi,  -100,   -100,   -100  ];
model.ub = [  200,   200,      pi,   100,   pi,   100,    70,    70  ];

%% Initial and final conditions
model.xinit = [-0.4 0 0.4 0 0 0 ]';
model.xinitidx = 3:8;

%% Define solver options
codeoptions = getOptions('RobotArmSolver');
codeoptions.maxit = 200;                                    % Maximum number of iterations of inner QP solver
codeoptions.printlevel = 0;                                 % Use printlevel = 2 to print progress (but not for timings)
codeoptions.optlevel = 3; 
codeoptions.nlp.integrator.Ts = integrator_stepsize;
codeoptions.nlp.integrator.nodes = 5;
codeoptions.nlp.integrator.type = 'ERK4';
% Options for SQP solver
codeoptions.solvemethod = 'SQP_NLP'; 
codeoptions.nlp.hessian_approximation = 'gauss-newton';     % Gauss-Newton hessian approximation of nonlinear least-squares objective 

%% Generate real-time SQP solver
FORCES_NLP(model, codeoptions);

%% Call solver
% Set initial guess to start solver from:
x0i = model.lb+(model.ub-model.lb)/2;
x0 = repmat(x0i',model.N,1);
problem.x0 = x0; 

X = zeros(nx,Ns-model.N+1); 
X(:,1) = model.xinit;
U = zeros(nu,Ns-model.N);
% Set reference as run-time parameter
problem.all_parameters = ones(model.N,1);

% Closed-loop cost
cost = zeros(Ns,1);
ode45_intermediate_steps = 10;
cost_integration_grid = linspace(0,integrator_stepsize,ode45_intermediate_steps);
cost_integration_step_size = integrator_stepsize/ode45_intermediate_steps;

% Run simulation
sim.states = zeros(nx, 0);
sim.inputs = zeros(nu, 0);
sim.solvetime = zeros(1, 0);
sim.qptime = zeros(1, 0);
sim.rsnorm = zeros(1, 0);
sim.res_eq = zeros(1, 0);
sim.fevalstime = zeros(1, 0);
sim.closed_loop_obj = zeros(1, 0);
sim.time = [0:Ns-1] * integrator_stepsize;

for i = 1:Ns/2
    sim.states(:,end+1) = X(:,i);
    % Set initial condition
    problem.xinit = X(:,i);
    
    % Call SQP solver
    [output,exitflag,info] = RobotArmSolver(problem);
    if exitflag ~= 1
        error('SQP_NLP failed at %d.\n', i);
    end
    U(:,i) = output.x01(1:nu);
    sim.inputs(:,end+1) = U(:,i);
    sim.solvetime(end+1) = info.solvetime;
    sim.fevalstime(end+1) = info.fevalstime;
    sim.qptime(end+1) = info.QPtime;
    sim.rsnorm(end+1) = info.rsnorm;
    sim.res_eq(end+1) = info.res_eq;
    
    % Simulate dynamics
    [~,xtemp] = ode45( @(time, states) dynamics(states,U(:,i)), cost_integration_grid, X(:,i) );
    
    % Compute closed-loop cost
    for j = 1:length(xtemp)
        cost(i) = cost(i) + cost_integration_step_size*(xtemp(j,:)*Q*xtemp(j,:)' + U(:,i)'*R*U(:,i));
    end
    sim.closed_loop_obj(end+1) = cost(i);
    
    X(:,i+1) = xtemp(end,:);
end 

% Set reference
problem.all_parameters = -ones(model.N,1);
for i=Ns/2+1:Ns
    
    sim.states(:,end+1) = X(:,i);
    
    % Set initial condition
    problem.xinit = X(:,i);
    
    [output,exitflag,info] = RobotArmSolver(problem);
    if exitflag ~= 1
        error('SQP_NLP failed at %d.\n', i);
    end
    U(:,i) = output.x01(1:nu);
    
    sim.inputs(:,end+1) = U(:,i);
    sim.solvetime(end+1) = info.solvetime;
    sim.fevalstime(end+1) = info.fevalstime;
    sim.qptime(end+1) = info.QPtime;
    sim.rsnorm(end+1) = info.rsnorm;
    sim.res_eq(end+1) = info.res_eq;
    
    % Simulate dynamics
    [~,xtemp] = ode45( @(time, states) dynamics(states,U(:,i)), cost_integration_grid , X(:,i) );

    % Compute cost
    for j = 1:length(xtemp)
        cost(i) = cost(i) + cost_integration_step_size*(xtemp(j,:)*Q*xtemp(j,:)' + U(:,i)'*R*U(:,i));
    end
    sim.closed_loop_obj(end+1) = cost(i);
    
    X(:,i+1) = xtemp(end,:);
end 
    
%% Plot simulation results

% Input torque rates
figure(1);clc;
plot(sim.time, sim.inputs(1,:), 'b'); hold on;
plot(sim.time, sim.inputs(2,:), 'b-.'); hold on;
plot(sim.time, repmat([-200; 200], 1, length(sim.time)), 'r');
xlabel('Simulation time (seconds)'); ylabel('Input torque rates (Nm/s)');grid on;
ylim([-205, 205]);

% Joint angle
figure(2);clc;
plot(sim.time, sim.states(3,:), 'b'); hold on;
plot(sim.time, repmat([-1.2; 1.2], 1, length(sim.time)), 'r'); hold on;
xlabel('Simulation time (seconds)'); ylabel('Joint angle (rad)');grid on;

% Torques
figure(3);clc;
plot(sim.time, sim.states(5,:), 'b'); hold on;
plot(sim.time, sim.states(6,:), 'b-.'); hold on;
plot(sim.time, repmat(70, 1, length(sim.time)), 'r'); hold on;
xlabel('Simulation time (seconds)'); ylabel('Torques (Nm)');grid on;
ylim([-20,75]);

% Closed-loop objective
figure(4);clc;
plot(sim.time, sim.closed_loop_obj, 'b');
xlabel('Simulation time (seconds)'); ylabel('Closed-loop objective');grid on;
