%% Design nonlinear MPC Controller
% This example shows how to create and test a nonlinear model predictive 
% controller for an exothermic chemical reactor through the FORCESPRO plugin
% for the MPC Toolbox. It illustrates how to ude the generic MPC Toolbox to
% FORCESPRO interface as well as the generated mex function.
%
% This example requires Model Predictive Control Toolbox from MathWorks.
%
% Copyright 2020-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

% Set useMex = 0 for using generic MPC Toolbox to FORCESPRO interface. 
% Set useMex = 1 in order to use mex function (for speed-up). 
% Set useSQP = 0 to iseinterior point solverand set useSQP = 1 in order to
% use SQP solver. 
useMex = 1;
useSQP = 1;

%% Model plant using @nlmpc object
% The control objective in this example is to maintain the concentration of
% reagent A in the exitstream of the plant. As a consequence the output
% function of our plant model outputs the concentration of A.
nx = 2;
ny = 1;
nu = 3;
nlobj = nlmpc(nx,ny,'MV',1,'MD',[2 3]);
Ts = 0.5;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 6; 
nlobj.ControlHorizon = [2 2 2];
nlobj.MV.RateMin = -5;
nlobj.MV.RateMax = 5;
% See model functions below
nlobj.Model.StateFcn = 'exocstrStateFcnCT';
nlobj.Model.OutputFcn = 'exocstrOutputFcn';

%% Set options to use with FORCESPRO
options = nlmpcToForcesOptions();
options.SolverName = 'CstrSolver';
options.SolverType = 'InteriorPoint';
options.IntegrationNodes = 5;
options.IP_MaxIteration = 500;
options.x0 = [311.2639; 8.5698];
options.mv0 = 298.15;

if useSQP
    options.SolverType = 'SQP';
    options.SQP_MaxQPS = 5;
    options.SQP_MaxIteration = 500; 
end

%% Generate FORCESPRO NLP solver through MPC Toolbox interface
[coredata, onlinedata] = nlmpcToForces(nlobj,options);
onlinedata.md = [10 298.15];

%% Closed loop simulation
Tstop = 200; % Number of seconds to simulate
simulationLength = Tstop/nlobj.Ts; % Number of simulation steps
x = options.x0; % initial state vector
mv = options.mv0; % initial manipulated variable
md = onlinedata.md; % initial measured disturbance

% data to store simulation results
time = zeros(simulationLength,1);
recorded_concentration = zeros(simulationLength,1);
reference_concentration = zeros(simulationLength,1);
cost = zeros(simulationLength,1);
solvetime = zeros(simulationLength,1);

for k = 1:simulationLength
    
    % set reference trajectory
    onlinedata.ref = exocstrReferenceTrajectory( Tstop, k);
    
    % call generated FORCESPRO solver through MPC Toolbox interface or
    % call mex for speed-up
    if useMex
        [mv, onlinedata, info] = nlmpcmove_CstrSolver(x,mv,onlinedata); % generated mex function has the name strcat('nlmpcmove_', options.SolverName)
    else
        [mv, onlinedata, info] = nlmpcmoveForces(coredata,x,mv,onlinedata);
    end
    
    % always check that solve was successfull before applying result
    assert(info.ExitFlag == 1, 'FORCESPRO solver failed to find solution');
    
    % simulate dynamics
    x = RK4(x,[mv;onlinedata.md'],@(x,u) exocstrStateFcnCT(x,u),Ts);
    
    % store simulation data
    time(k) = k * Ts;
    recorded_concentration(k) = x(2);
    reference_concentration(k) = onlinedata.ref(2);
    cost(k) = info.Cost;
    solvetime(k) = info.SolveTime * 1e3; % store solvetime in ms
end


%% Plot results

% Cost
figure('Name','Cost');clc;
plot(time, cost, 'b', 'LineWidth', 2); hold on;
legend('cost');
xlabel('Simulation time (s)'); ylabel('Cost');grid on;

% Solve time
figure('Name','Solvetime');clc;
plot(time, solvetime, 'b', 'LineWidth', 2); hold on;
legend('solve time');
xlabel('Simulation time (s)'); ylabel('Solve time (ms)');grid on;

% Concentration of A
figure('Name','Concentration of A vs time');clc;
plot(time, recorded_concentration, 'b', 'LineWidth', 2); hold on;
plot(time, reference_concentration, 'r', 'LineWidth', 2); hold on;
legend('measured concentration','reference concentration');
xlabel('Simulation time (s)'); ylabel('Concentration of A');grid on;


function ref = exocstrReferenceTrajectory( Tstop, k )
% reference trajectory 
Ts = 0.5;
PredictionHorizon = 6;
yHigh = 8.5698;
yLow = 2;
time = (0:Ts:(Tstop+PredictionHorizon*Ts))';
len = length(time);
r = [yHigh*ones(5,1);linspace(yHigh,yLow,len-10)';yLow*ones(5,1)];
ref = r(k:k+PredictionHorizon-1);
end
