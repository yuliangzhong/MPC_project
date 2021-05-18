% This example demonstrates how to use the SQP_NLP real-time iteration solver to control a DC motor
%
% The DC motor must track a reference on its angular speed.
%
% Variables are collected stage-wise into z = [u; x(1); x(2)].
%
% (c) Embotech AG, Zurich, Switzerland, 2013-2021.
close all;
clc;

%% populate model struct
% Problem dimensions
model.N = 20;               % horizon length
model.nvar = 3;             % number of variables
model.neq  = 2;             % number of equality constraints
model.nh = 0;               % number of inequality constraint functions
model.npar = 1;             % number of run-time parameters (in this case the 1-dimensional reference we are tracking)

% Objective function 
model.LSobjective = @(z,p) sqrt(100) * (z(3) - p);
model.LSobjectiveN = @(z,p) sqrt(100) * (z(3) - p);

% initial condition
model.xinit = zeros(model.neq,1);
model.xinitidx = 2:model.nvar;

% State and input bounds
model.lb = [1; -5; -10];
model.ub = [1.6; 5; 2.004];

% dynamics
model.continuous_dynamics = @(x,u,p) dynamics(x,u);
model.E = [zeros(model.neq,1), eye(model.neq)];

% initial value 
model.initidx = 2:model.nvar;

%% set codeoptions
codeoptions = getOptions('FORCESPROSolver');
codeoptions.solvemethod = 'SQP_NLP'; % generate SQP-RTI solver
codeoptions.BuildSimulinkBlock = 0;
codeoptions.nlp.integrator.type = 'ERK4';
integration_step = 0.01;
codeoptions.nlp.integrator.Ts = integration_step;
codeoptions.nlp.integrator.nodes = 1;
codeoptions.nlp.hessian_approximation = 'gauss-newton';
codeoptions.timing = 1;

%% generate FORCESPRO solver
FORCES_NLP(model, codeoptions);


%% run simulation
simLength = 500; % simulate 5 seconds

% populate run time parameters struct
params.all_parameters = repmat(2, model.N, 1);
params.xinit = zeros(model.neq, 1); % initial condition to ODE
params.x0 = repmat([1.2;zeros(2,1)], model.N, 1); % initial guess
params.reinitialie = 0;
x = params.xinit;

% collect simulation data
time = zeros(simLength, 1); % store time vector (simulation time)
solveTime = zeros(simLength, 1); % store solve time
fevalsTime = zeros(simLength, 1); % store function evaluations time
qpTime = zeros(simLength, 1); % store time it takes to solve quadratic approximation
obj = zeros(simLength, 1); % store closed-loop objective objective value 
referenceValue = zeros(simLength, 1); % store reference value which is tracked
angularSpeed = zeros(simLength, 1); % store angular speed

for k = 1:simLength
   
    % Solve optimization problem
    [output, exitflag, info] = FORCESPROSolver(params);
    
    % check quality of output
    assert(exitflag == 1, 'FORCESPROSolver failed to find good output')
    
    % extract control
    u = output.x01(1);
    
    % integrate ODE
    p = params.all_parameters(1);
    x = RK4( x, u, @dynamics, integration_step, p, codeoptions.nlp.integrator.nodes);
    
    % prepare params struct for next simulation step
    if k == 250
       params.all_parameters = repmat(-2, model.N, 1); % change reference half way through simulation to test robustness of controller
    end
    params.xinit = x;
    
    % collect simulation data
    time(k) = k*integration_step;
    solveTime(k) = info.solvetime;
    fevalsTime(k) = info.fevalstime;
    qpTime(k) = info.QPtime;
    obj(k) = 0.5 * (model.LSobjective([u;x], p)' * model.LSobjective([u;x], p));
    referenceValue(k) = p;
    angularSpeed(k) = x(2);
end


%% Plot simulation results

% Solve time
figure('Name','solvetime vs time');clc;
plot(time, solveTime, 'b', 'LineWidth', 2); hold on;
xlabel('Simulation time (s)'); ylabel('solvetime (s)');grid on;

% function evaluations time
figure('Name','fevalstime vs time');clc;
plot(time, fevalsTime, 'b', 'LineWidth', 2); hold on;
xlabel('Simulation time (s)'); ylabel('fevalstime (s)');grid on;

% QP time
figure('Name','QPtime vs time');clc;
plot(time, qpTime, 'b', 'LineWidth', 2); hold on;
xlabel('Simulation time (s)'); ylabel('QPtime (s)');grid on;

% Closed-loop objective
figure('Name','Closed-loop objective value vs time');clc;
plot(time, obj, 'b', 'LineWidth', 2); hold on;
xlabel('Simulation time (s)'); ylabel('Closed-loop objective value');grid on;

% Distance to reference
figure('Name','Angular speed and reference');clc;
plot(time, angularSpeed, 'b', 'LineWidth', 2); hold on;
plot(time, referenceValue, 'r', 'LineWidth', 2); hold on;
legend('Angular speed','reference value');
xlabel('Simulation time (s)');grid on;



function dx = dynamics(x,u,p)

%% model parameters
% Armature inductance (H)
La = 0.307;
% Armature resistance (Ohms)
Ra = 12.548;
% Motor constant (Nm/A^2)
km = 0.23576;
% Total moment of inertia (Nm.sec^2)
J = 0.00385;
% Total viscous damping (Nm.sec)
B = 0.00783;
% Load torque (Nm)
tauL = 1.47;
% Armature voltage (V)
ua = 60;


dx = [(-1/La)*(Ra*x(1) + km*x(2)*u(1) - ua);...
                       (-1/J)*(B*x(2) - km*x(1)*u(1) + tauL)];

end
