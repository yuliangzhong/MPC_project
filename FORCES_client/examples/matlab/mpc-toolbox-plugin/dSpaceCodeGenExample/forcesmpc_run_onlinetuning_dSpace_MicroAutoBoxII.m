%% Tuning Controller Weights
% This example shows how to vary the weights on outputs, inputs, and ECR
% slack variable for soft constraints in real-time and deploy to dSpace
% MicroAutoBox II.
%
% The weights specified in the MPC object are overridden by the weights
% supplied to the MPC Controller block. If a weight signal is not connected
% to the MPC Controller block, then the corresponding weight is the one
% specified in the MPC object.
% 
% The Simulink block that is used as an MPC controller is the FORCESPRO
% solver block

% Copyright 1990-2021 The MathWorks, Inc and Embotech AG, Zurich, Switzerland  

%% Define Plant Model
% Define a multivariable discrete-time linear system with no direct I/O
% feedthrough, and assume input #4 is a measured disturbance and output #4
% is unmeasured.
Ts = 0.5; % sampling time
plant = tf({1,[1 1],5,2;3,[1 5],1,0;0,0,1,[1 1];2,[1 -1],0,0},...
    {[1 1 1],[1 3 4 5],[1 10],[1 5];
      [1 1],[1 2],[1 2 8],[1 1];
      [1 2 1],[1 3 1 1],[1 1],[1 2];
      [1 1],[1 3 10 10],[1 10],[1 1]});
plant = c2d(ss(plant),Ts);
plant.D = 0;

%% Design MPC Controller
% Specify input and output signal types.
plant = setmpcsignals(plant,'MD',4,'UO',4);
% Create the controller object with sampling period, prediction and control
% horizons:
p = 20;                                     % Prediction horizon
m = 3;                                      % Control horizon
mpcobj = mpc(plant,Ts,p,m); 
%%
% Specify MV constraints.
mpcobj.MV(1).Min = -6;
mpcobj.MV(1).Max = 6;
mpcobj.MV(2).Min = -6;
mpcobj.MV(2).Max = 6;
mpcobj.MV(3).Min = -6;
mpcobj.MV(3).Max = 6;

%% Generate FORCESPRO sparse QP solver
options = mpcToForcesOptions('sparse');
% For this example we need to specify that online weights on the outputs,
% the input rates and the ECR slacks are used
options.UseOnlineWeightOV = true;
options.UseOnlineWeightMVRate = true;
options.UseOnlineWeightECR = true;
options.ForcesTargetPlatform = 'dSPACE-MABII';

[coredata, statedata, onlinedata] = mpcToForces(mpcobj, options);

%% Simulate Using Simulink(R)
% To run this example, Simulink(R) is required.
if ~mpcchecktoolboxinstalled('simulink')
    disp('Simulink(R) is required to run this example.')
    return
end
% Define reference signal.
Tstop = 10;
ref = [1 0 3 1];            
r = struct('time',(0:Ts:Tstop)');
N = numel(r.time);
r.signals.values=ones(N,1)*ref;
%%
% Define measured disturbance.
v = 0.5;                                        
%%
% OV weights are linearly increasing with time, except for output #2
% that is not weighted.
ywt.time = r.time;
ywt.signals.values = (1:N)'*[.1 0 .1 .1];
%% 
% MVRate weights are decreasing linearly with time.
duwt.time = r.time;
duwt.signals.values = (1-(1:N)/2/N)'*[.1 .1 .1];
%%
% ECR weight increases exponentially with time.
ECRwt.time = r.time;
ECRwt.signals.values = 10.^(2+(1:N)'/N);
%% 
% Start Code Generation.
mdl = 'forcesmpc_onlinetuning_dSpace_MicroAutoBoxII';
open_system(mdl);                   % Open Simulink(R) Model
load_system(mdl);                   % Load Simulink(R) Model
rtwbuild(mdl);                      % Start Code Generation