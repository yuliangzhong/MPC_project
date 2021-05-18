% Simple MPC for an LTI system using the Mathworks MPC toolbox 
%
% This example requires Model Predictive Control Toolbox from MathWorks and
% the FORCESPRO plugin for the mpc toolbox

% Copyright 2019-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

%% Instantiate LTI system
A = [1.1 2; 0 0.95];
B = [0; 0.0787];
C = [-1 1];
D = 0;
Ts = 1;
sys = ss(A,B,C,D,Ts);
sys.InputName = {'u_1'};
sys.StateName = {'x_1', 'x_2'};
sys.OutputName = {'y_1'};
sys.InputGroup.MV = 1;
sys.OutputGroup.MO = 1;

%% Instantiate MPC object
Weights = struct('MV',0,'MVRate',0.1,'OV',10);
MV = struct('Min',{-1.0},'Max',{1.0});
MPCobj = mpc(sys,Ts,10,2,Weights,MV);

%% Instantiate FORCESPRO options using Sparse QP formulation
options = mpcToForcesOptions('sparse');

%% Generate FORCESPRO solver code from MPC object
[forcesCoredata, forcesStatedata, forcesOnlinedata] = mpcToForces(MPCobj, options);

%% Simulate MPC to bring the plant from a non-zero initial states to 0
SimuSteps = 40; % Number of simulation steps
xp0 = [0.5; -0.5]; % initial states at [0.5 -0.5]
X = zeros(length(xp0), SimuSteps+1);
U = zeros(size(sys.B, 2), SimuSteps);
Y = zeros(size(sys.C, 1), SimuSteps);
X(:, 1) = xp0;
% initialize state data
forcesStatedata.Plant = xp0;
for k = 1:SimuSteps
    Y(:,k) = sys.C * X(:,k);
    forcesOnlinedata.signals.ym = Y(:,k);
    [mv,forcesStatedata,info] = mpcmoveForces(forcesCoredata, forcesStatedata, forcesOnlinedata);
    if info.ExitFlag < 0
        warning('Internal problem in FORCESPRO QP solver');
    end
    U(:,k) = mv;
    X(:,k+1) = sys.A * X(:,k) + sys.B * U(:,k);
end

%% Plot results
timeVec = Ts:Ts:(SimuSteps*Ts);
figure(1);
plot(timeVec, Y(1,:));
xlabel('Time (sec)');ylabel('y');
grid on;
hold on

%% Instantiate FORCESPRO options using Dense QP formulation
options = mpcToForcesOptions('dense');

%% Generate FORCESPRO solver code from MPC object
[forcesCoredata, forcesStatedata, forcesOnlinedata] = mpcToForces(MPCobj, options);

%% Simulate MPC to bring the plant from a non-zero initial states to 0
X = zeros(length(xp0), SimuSteps+1);
U = zeros(size(sys.B, 2), SimuSteps);
Y = zeros(size(sys.C, 1), SimuSteps);
X(:, 1) = xp0;
% initialize state data
forcesStatedata.Plant = xp0;
for k = 1:SimuSteps
    Y(:,k) = sys.C * X(:,k);
    forcesOnlinedata.signals.ym = Y(:,k);
    [mv,forcesStatedata,info] = mpcmoveForces(forcesCoredata, forcesStatedata, forcesOnlinedata);
    if info.ExitFlag < 0
        warning('Internal problem in FORCESPRO QP solver');
    end
    U(:,k) = mv;
    X(:,k+1) = sys.A * X(:,k) + sys.B * U(:,k);
end

%% Plot results
timeVec = Ts:Ts:(SimuSteps*Ts);
plot(timeVec, Y(1,:), '*');
legend('Sparse QP','Dense QP')
