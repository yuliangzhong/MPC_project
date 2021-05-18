%% Design Model Predictive Controller at Equilibrium Operating Point
% This example shows how to design a model predictive controller with
% nonzero nominal values. 
%
% The plant model is obtained by linearization of a nonlinear plant in
% Simulink(R) at a nonzero steady-state operating point.
%
% This example requires Model Predictive Control Toolbox from MathWorks and
% the FORCESPRO plugin.

% Copyright 2019-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

%% Load plant model linearized at its nominal operating point (x0, u0, y0)
load('nomConditionsLinearize.mat');

%% Design MPC Controller
% Create an MPC controller object with a specified sample time |Ts|,
% prediction horizon |p|, and control horizon |m|.
Ts = 0.1;
p = 20;
m = 3;
mpcobj = mpc(plant,Ts,p,m);

%%
% Set the nominal values in the controller.
mpcobj.Model.Nominal = struct('X',x0,'U',u0,'Y',y0);

%%
% Set the output measurement noise model (white noise, zero mean,
% variance = 0.01).
mpcobj.Model.Noise = 0.01;        

%%
% Set the manipulated variable constraint.
mpcobj.MV.Max = 0.2;

%% Simulate Using Simulink
% Specify the reference value for the output signal.
r0 = 1.5*y0;

%% Generate FORCESPRO QP solver
% Create options structure for the FORCESPRO sparse QP solver
options = mpcToForcesOptions();
% Generates the FORCESPRO QP solver
[coredata, statedata, onlinedata] = mpcToForces(mpcobj, options);

%% Simulate Using |sim| Command
% Simulate the controller.
Tf = round(10/Ts);
r = r0*ones(Tf,1);

dPlant = c2d(plant, Ts);
nx = size(dPlant.A, 1);
nu = size(dPlant.B, 2);
ny = size(dPlant.C, 1);

X = zeros(nx, Tf+1);
U = zeros(nu, Tf);
Y = zeros(ny, Tf);
X(:, 1) = x0;
for t = 1:Tf
    Y(:, t) = dPlant.C * (X(:, t) - x0) + dPlant.D * (U(:, t) - u0) + y0 + 0.01 * randn;
    % Prepare inputs of mpcmoveForces
    onlinedata.signals.ref = r(t:min(t+mpcobj.PredictionHorizon-1,Tf),:);
    onlinedata.signals.ym = Y(:, t);
    % Call FORCESPRO solver
    [mv, statedata, info] = mpcmoveForces(coredata, statedata, onlinedata);
    if info.ExitFlag < 0
        warning('Internal problem in FORCESPRO solver');
    end
    U(:, t) = mv;
    X(:, t+1) = dPlant.A * (X(:, t) - x0) + dPlant.B * (U(:, t) - u0) + x0;
end
%% Plot results
figure(1);clc;
plot(Y); hold on;
plot(r, 'r-.'); grid on;
