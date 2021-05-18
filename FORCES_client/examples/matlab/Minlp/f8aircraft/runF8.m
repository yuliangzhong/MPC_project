% F8 Crusader aircraft  
% The control objective is to drive the angle of attack to zero by changing 
% the tail deflection angle, which can be fully up or down (discrete control input).
%
%  min   xN'*P*xN + sum_{i=1}^{N-1} ui'*R*ui
% xi,ui
%       s.t. x1 = x
%            x_i+1 = f(x_i)       for i = 1...N-1 (aircraft dynamics)
%            xmin <= xi <= xmax   for i = 1...N
%            umin <= ui <= umax   for i = 1...N-1
%            u_i in {umin, umax}  for i = 1...N-1 (integer input)
%
% (c) Embotech AG, Zurich, Switzerland, 2019.

clear; clc;

% Whether or not to provide an guess for the incumbent
guessIncumbent = true;

solverName = 'F8aircraft';
Nstages = 100;

params.dimU = 1;
params.dimX = 3;
params.dimZ = params.dimU + params.dimX;

%% Model with stage variable (u', x')'
model.N = Nstages;
model.nvar = params.dimZ;
model.neq = params.dimX;

%% Boundary conditions
model.xinitidx = params.dimU+1:model.nvar;

%% Bounds
model.lb = [];
model.ub = [];
model.lbidx{1} = 1 : params.dimU;
model.ubidx{1} = 1 : params.dimU;
for i = 2 : model.N
    model.lbidx{i} = 1 : model.nvar;
    model.ubidx{i} = 1 : model.nvar;
end

%% Dynamics
wa = 0.05236;
wa2 = wa^2;
wa3 = wa^3;
continuous_dynamics = @(x, u) [ -0.877 * x(1) + x(3) - 0.088 * x(1) * x(3) + 0.47 * x(1) * x(1) - 0.019 * x(2) * x(2) - x(1) * x(1) * x(3)... 
                                + 3.846 * x(1) * x(1) * x(1) - 0.215 * wa * (2 * u(1) - 1) + 0.28 * x(1) * x(1) * wa * (2 * u(1) - 1) + 0.47 * x(1) * wa2 * (2 * u(1) - 1) * (2 * u(1) - 1)...
                                + 0.63 * wa3 * (2 * u(1) - 1) * (2 * u(1) - 1) * ( 2 * u(1) - 1);
                                x(3);
                                -4.208 * x(1) - 0.396 * x(3) - 0.47 * x(1) * x(1) - 3.564 * x(1) * x(1) * x(1) - 20.967 * wa * (2 * u(1) - 1) + 6.265 * x(1) * x(1) * wa * (2 * u(1) -1 )...
                                + 46.0 * x(1) * wa2 * (2 * u(1) - 1) * (2 * u(1) - 1) + 61.4 * wa3 * (2 * u(1) - 1) * (2 * u(1) - 1) * (2 * u(1) - 1)];
model.continuous_dynamics = continuous_dynamics; 
model.E = [zeros(params.dimX, params.dimU), eye(params.dimX)];

%% Objective
model.objective =  @(z) 0.0001 * z(params.dimU)^2;
model.objectiveN = @(z) 150 * z(params.dimU+1)^2 + 5 * z(params.dimU+2)^2 + 5 * z(params.dimU+3)^2;

%% Integer indices
for s = 1:model.N
    model.intidx{s} = [1];
end

%% Define outputs
outputs(1) = newOutput('TailAngle', 1:model.N, 1);
outputs(2) = newOutput('AngleAttack', 1:model.N, 2);
outputs(3) = newOutput('PitchAngle', 1:model.N, 3);
outputs(4) = newOutput('PitchAngleRate', 1:model.N, 4);

%% Set code-generation options
codeoptions = getOptions(solverName);
codeoptions.printlevel = 0;
codeoptions.maxit = 2000;
codeoptions.timing = 0;
codeoptions.nlp.integrator.type = 'IRK2';
codeoptions.nlp.integrator.Ts = 0.05;
codeoptions.nlp.integrator.nodes = 20;
% Specify maximum number of threads to parallelize minlp search
codeoptions.minlp.max_num_threads = 8;

%% Options for providing incumbent guess at runtime
if guessIncumbent
    codeoptions.minlp.int_guess = 1;
    codeoptions.minlp.round_root = 0;
    codeoptions.minlp.int_guess_stage_vars = [1];
end

%% Generate MINLP solver
FORCES_NLP(model, codeoptions, outputs);

%% Set run-time parameters
model.nu = params.dimU;
model.nx = params.dimX;
problem.(sprintf('lb%03d', 1)) = 0;
problem.(sprintf('ub%03d', 1)) = 1;
for s = 2:Nstages-1
    problem.(sprintf('lb%03d', s)) = [0, -1e1 * ones(1, model.nx)]';
    problem.(sprintf('ub%03d', s)) = [1, 1e1 * ones(1, model.nx)]';
end
problem.(sprintf('lb%03d', Nstages)) = [0, -1e1 * ones(1, model.nx)]';
problem.(sprintf('ub%03d', Nstages)) = [1, 1e1 * ones(1, model.nx)]';

problem.x0 = repmat([0; zeros(model.nx, 1)], Nstages, 1);
problem.xinit = zeros(model.nx, 1);
problem.xinit(1) = 0.4655;
% FORCESPRO integer search will run on 2 thread
problem.parallelStrategy = 0; % Default value
problem.numThreadsBnB = 2;

% Integer guess 
if guessIncumbent
    for s = 1:Nstages
        problem.(sprintf('int_guess%03d', s)) = [0];
    end
    for s = 1:2
        problem.(sprintf('int_guess%03d', s)) = [1];
    end
    problem.(sprintf('int_guess%03d', 39)) = [1];
    for s = 41:42
        problem.(sprintf('int_guess%03d', s)) = [1];
    end
    for s = 85:90
        problem.(sprintf('int_guess%03d', s)) = [1];
    end
end

%% Call MINLP solver
[sol,exitflag,info] = F8aircraft(problem);

%% plot
time = 0:codeoptions.nlp.integrator.Ts:4.95;
    
figure(1); clf;
stairs(time, 0.05236 * (2 * sol.TailAngle - 1), 'LineWidth', 3);
grid on; title('Tail deflection angle (rad)');
set(gca,'linewidth',1);

figure(2);clf;
subplot(3, 1, 1); grid on; title('Angle of attack (rad)'); hold on;
plot(time, sol.AngleAttack, 'r', 'LineWidth', 3);
set(gca,'linewidth',1.5);
subplot(3, 1, 2); grid on; title('Pitch angle (rad)'); hold on;
plot(time, sol.PitchAngle, 'b', 'LineWidth', 3);
set(gca,'linewidth',1.5);
subplot(3, 1, 3); grid on; title('Pitch angle rate (rad/sec)'); hold on;
plot(time, sol.PitchAngleRate, 'b', 'LineWidth', 3);
set(gca,'linewidth',1.5);


