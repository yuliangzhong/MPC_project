%% Setting Targets for Manipulated Variables
% This example shows how to design a model predictive controller for a
% plant with two inputs and one output with target setpoint for a
% manipulated variable. 
% The FORCESPRO sparse solver is also demonstrated for setting input
% references.
%
% This example requires Model Predictive Control Toolbox from MathWorks.

% Copyright 2019-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

%% Define Plant Model
% The linear plant model has two inputs and two outputs.
N1 = [3 1];
D1 = [1 2*.3 1]; 
N2 = [2 1];
D2 = [1 2*.5 1]; 
plant = ss(tf({N1,N2},{D1,D2}));
A = plant.A;
B = plant.B;
C = plant.C;
D = plant.D;

%% Design MPC Controller
% Create MPC controller.
Ts = 0.4;                      % Sample time
mpcobj = mpc(plant,Ts,20,5);
%%
% Specify weights.
mpcobj.weights.manipulated = [0.3 0]; % weight difference MV#1 - Target#1
mpcobj.weights.manipulatedrate = [0 0];
mpcobj.weights.output = 1;
%%
% Define input specifications.
mpcobj.MV = struct('RateMin',{-0.5;-0.5},'RateMax',{0.5;0.5});

%% Generate FORCESPRO solver
% Create options structure for FORCESPRO solver using the Sparse QP
% formulation.
options = mpcToForcesOptions();
options.UseMVTarget = true; 
% Generate solver
[coredata, statedata, onlinedata] = mpcToForces(mpcobj, options);

%% Simulate
Tstop = 20;                     % seconds
Tf = round(Tstop/Ts);           % simulation iterations
% Discretize system
dSys = c2d(plant, Ts);
yref = zeros(Tf,1);
uref = [2*ones(Tf, 1), zeros(Tf, 1)];
x0 = zeros(size(plant.A,1),1);
[Y, U] = simulateMpcForces(mpcobj, dSys, Tf, x0, yref, 1, coredata, statedata, onlinedata, uref);

%% Plot results.
figure
plot(Ts:Ts:Tstop, Y, Ts:Ts:Tstop, yref);
grid on; xlabel('Time (sec)'); title('Output');
figure
plot(0:Ts:Tstop-Ts, U(1,:), 0:Ts:Tstop-Ts, uref(:,1));
grid on; xlabel('Time (sec)'); title('Input #1');

