%% Control of a Multi-Input Single-Output Plant
% This example shows how to design model predictive controller with one
% measured output, one manipulated variable, one measured disturbance, and
% one unmeasured disturbance in a typical workflow. 
% It also shows how to solve the MPC problems with unmeasured and measured 
% disturbances using FORCESPRO.
%
% This example requires Model Predictive Control Toolbox from MathWorks.

% Copyright 2019-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

%% Define Plant Model
% The discrete-time linear open-loop dynamic model is defined below with
% sample time |Ts|.
sys = ss(tf({1,1,1},{[1 .5 1],[1 1],[.7 .5 1]}));
Ts = 0.2;
model = c2d(sys,Ts);

%% Design MPC Controller
% Define type of input signals: the first signal is a manipulated variable,
% the second signal is a measured disturbance, the third one is an
% unmeasured disturbance.
model = setmpcsignals(model,'MV',1,'MD',2,'UD',3);

%%
% Create the controller object with sampling period, prediction and control
% horizons.
mpcobj = mpc(model,Ts,10,3);

%%
% Define constraints on the manipulated variable.
mpcobj.MV = struct('Min',-0.8,'Max',0.8,'RateMin',-10,'RateMax',10);

%%
% For unmeasured input disturbances, its model is an integrator driven by
% white noise with variance = 1000.
mpcobj.Model.Disturbance = tf(sqrt(1000),[1 0]);

%% Simulate Controller Using |mpcmove|
% First, obtain the discrete-time state-space matrices of the plant.
[A, B, C, D] = ssdata(model);
Tstop = 30;                  % Simulation time

%%
% Create options structure for FORCESPRO solver using the Sparse QP
% formulation.
options = mpcToForcesOptions('dense');
% Generate FORCESPRO solver
[coredata, statedata, onlinedata] = mpcToForces(mpcobj, options);

%%
% Store the closed-loop MPC trajectories in arrays |YY|, |UU|, and |XX|.
YY=[];
UU=[];
XX=[];

%% Simulate model with measured and unmeasured disturbance
simuSteps = round(Tstop/Ts);
% Reference signal
yref = 0.5*ones(simuSteps, 1);
% Measured disturbance signal
mdis = zeros(simuSteps, 1);
mdis(50:end) = 1;
% Initial state of the plant
x = [0 0 0 0 0]';           
for t=1:simuSteps
    % Get online preview signals
    onlinedata.signals.ref = yref(t:min(t+mpcobj.p-1,simuSteps));
    onlinedata.signals.md = mdis(t:min(t+mpcobj.p,simuSteps));
    % Store states
    XX = [XX,x];
    % Define unmeasured disturbance signal
    d = 0;
    if t*Ts>=20
       d = -0.5;
    end
    % Plant equations: output update (no feedthrough from MV to Y)
    y = C*x + D(:,2)*mdis(t) + D(:,3)*d;
    YY = [YY,y];
    % Compute MPC action using FORCESPRO
    onlinedata.signals.ym = y;
    [mv, statedata] = mpcmoveForces(coredata, statedata, onlinedata);
    % Plant equations: state update
    x = A*x + B*[mv;mdis(t);d];
    UU = [UU,mv];
end

%% Plot results
figure
subplot(2,1,1);
plot(Ts:Ts:Tstop,YY,Ts:Ts:Tstop,yref);
grid on; xlabel('Time (sec)'); title('Output');
subplot(2,1,2);
plot(0:Ts:Tstop-Ts,UU,0:Ts:Tstop-Ts,-0.8*ones(1,simuSteps),0:Ts:Tstop-Ts,0.8*ones(1,simuSteps));
grid on; xlabel('Time (sec)'); title('Input');