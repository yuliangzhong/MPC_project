%% Improving Control Performance with Look-Ahead (Previewing)
% This example shows how to design a model predictive controller with
% look-ahead (previewing) on reference and measured disturbance
% trajectories. It demonstrates how to solve the MPC problems using the
% FORCESPRO dense QP solver.
%
% This example requires Model Predictive Control Toolbox from MathWorks.

% Copyright 2019-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

%% Define Plant Model
% Define the plant model as a linear time invariant system with two inputs
% (one manipulated variable and one measured disturbance) and one output.
plant = ss(tf({1,1},{[1 .5 1],[1 1]}),'min');

%%
% Get the state-space matrices of the plant model and specify the initial
% condition.
[A,B,C,D] = ssdata(plant);
Ts = 0.2; % Sample time
[Ad,Bd,Cd,Dd] = ssdata(c2d(plant,Ts));
x0 = [0;0;0]; 

%% Design Model Predictive Controller
% Define type of input signals.
plant = setmpcsignals(plant,'MV',1,'MD',2);

%%
% Create the MPC object.
p = 20;                       % prediction horizon 
m = 10;                       % control horizon 
mpcobj = mpc(plant,Ts,p,m);
% Specify MV constraints.
mpcobj.MV = struct('Min',0,'Max',2);
% Specify weights
mpcobj.Weights = struct('MV',0,'MVRate',0.1,'Output',1);

%% Generate FORCESPRO solver
% Create options for dense FORCESPRO solver
options = mpcToForcesOptions('dense');
% Turns MPC object into FORCESPRO solver
[coredata, statedata, onlinedata] = mpcToForces(mpcobj, options);

%% Simulate Using SIM Command
% Let us run closed-loop simulation in MATLAB. 
Tstop = 30;  % simulation time.
time = (0:Ts:(Tstop+p*Ts))'; % time vector
r = double(time>10); % reference signal
v = -double(time>20); % measured disturbance signal
%%
% Use MPCSIMOPT object to turn on previewing feature in the closed-loop
% simulation.
params = mpcsimopt(mpcobj);
params.MDLookAhead='on';
params.RefLookAhead='on';
%%
% Simulate in MATLAB with SIM command.
YY1 = sim(mpcobj,Tstop/Ts+1,r,v,params);

%% Simulate Using MPCMOVE Command
% Store the closed-loop MPC trajectories.
YY2 = [];
% Use MPCSTATE object to specify the initial state of MPC
x = x0;
%%
% Start simulation loop
SimSteps = round(Tstop/Ts)+1;
for ct=1:SimSteps
    % Plant equations: output update
    y = C*x + D(:,2)*v(ct);
    % Store signals
    YY2 = [YY2,y];
    % Prepare signals for mpcmoveForces
    onlinedata.signals.ref = r(ct:min(ct+mpcobj.PredictionHorizon-1,SimSteps),:);
    onlinedata.signals.md = v(ct:min(ct+mpcobj.PredictionHorizon,SimSteps),:);
    onlinedata.signals.ym = y;
    % Compute MPC law. Extracts references r(t+1),r(t+2),...,r(t+p) and
    % measured disturbances v(t),v(t+1),...,v(t+p) for previewing.
    [u, statedata, info] = mpcmoveForces(coredata, statedata, onlinedata);
    if info.ExitFlag < 0
        warning('Internal problem in FORCESPRO solver.');
    end
    % Plant equations: state update
    x = Ad*x+Bd(:,1)*u+Bd(:,2)*v(ct);
end
%% Plot results.
figure
t = 0:Ts:Tstop;
plot(t,r(1:length(t)),'c:',t,YY1,'r-',t,YY2,'bo');
xlabel('Time');
ylabel('Plant Output');
legend({'Reference';'From SIM command';'From mpcmoveForces command'},'Location','SouthEast');
grid
