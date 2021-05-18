%% Design nonlinear MPC Controller for landing a rocket
% This example shows how to create and test a nonlinear model predictive 
% controller for landing a rocket through the FORCESPRO plugin
% for the MPC Toolbox. It illustrates how to generate a FORCESPRO solver
% using a nlmpcMultistage object.
%
% This example requires Model Predictive Control Toolbox from MathWorks.
%
% Copyright 2020-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

% For further details on the used model, 
% see https://www.mathworks.com/help/mpc/ug/landing-rocket-with-mpc-example.html

clear
close all

%% initial condition
x0 = [20;80;0;0;0;0];
u0 = [0;0];
xf = [0;10;0;0;0;0];

%% Design nlmpcMultistage Controller
% Construct nlmpcMultistage object and set dynamics
Ts = 0.2;
pPlanner = 50;
planner = nlmpcMultistage(pPlanner,6,2);
planner.Ts = Ts;
planner.Model.StateFcn = 'RocketStateFcn';

% Limit thrusts between 0 and 8 Newton
planner.MV(1).Min = 0;
planner.MV(1).Max = 8; 
planner.MV(2).Min = 0;
planner.MV(2).Max = 8;

% Specify lower bound on y-axis to avoid crashing
planner.States(2).Min = 10;

% Set cost function
for ct=1:pPlanner
    planner.Stages(ct).CostFcn = 'RocketPlannerCostFcn';
end

% set terminal state
planner.Model.TerminalState = xf;

% set maximum number of solver iterations
planner.Optimization.SolverOptions.MaxIterations = 100;

% validate model functions
validateFcns(planner,rand(6,1),rand(2,1));

%% Generate FORCES NLP Solver
options = nlmpcMultistageToForcesOptions;
options.Server = 'https://forces.embotech.com/';
options.x0 = x0;
options.mv0 = u0;
options.UseOnlineConstraintMVMin = true;
options.UseOnlineConstraintMVMax = true;
options.UseOnlineConstraintStateMin = true;
[coredata, onlinedata, model] = nlmpcMultistageToForces(planner, options);

% initialize online data
onlinedata.MVMin = repmat([0 0],pPlanner,1);
onlinedata.MVMax = repmat([8 8],pPlanner,1);
onlinedata.StateMin = repmat([-inf 10 -inf -inf -inf -inf],pPlanner,1);

%% Run FORCES (MEX)
% use nlmpcmoveForcesMultistage to use non MEXed function.
tic;
[~, onlinedata, infoFORCES] = nlmpcmove_myForcesNLPSolver(x0,u0,onlinedata);
tFORCES=toc;

%% Run animation
plotobj = RocketAnimation(6,2);
for ct=1:pPlanner+1
    updatePlot(plotobj,(ct-1)*planner.Ts,infoFORCES.Xopt(ct,:),infoFORCES.MVopt(ct,:));
    pause(0.1);
end

%% Plot results
fprintf('FORCES: Calculation Time = %s; Objective cost = %s; ExitFlag = %s; Iterations = %s\n',num2str(tFORCES),num2str(infoFORCES.Cost),num2str(infoFORCES.ExitFlag),num2str(infoFORCES.Iterations));
figure;
subplot(3,1,1);plot(infoFORCES.Xopt(:,1),infoFORCES.Xopt(:,2));title('Optimal XY Trajectory');
subplot(3,1,2);plot(infoFORCES.Topt,infoFORCES.MVopt(:,1)),title('Optimal MV1 Trajectory');
subplot(3,1,3);plot(infoFORCES.Topt,infoFORCES.MVopt(:,2));title('Optimal MV2 Trajectory');

