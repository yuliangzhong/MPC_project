%% Design MPC Controller at the Command Line
% This example shows how to create and test a model predictive controller
% from the command line. It also illustrates how to use the FORCESPRO
% plugin for the MPC toolbox. 
%
% This example requires Model Predictive Control Toolbox from MathWorks.

% Copyright 2019-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

%% Define Plant Model
% Create a state-space model of the plant and set model properties.
A = [-0.0285 -0.0014; -0.0371 -0.1476];
B = [-0.0850 0.0238; 0.0802 0.4462];
C = [0 1; 1 0]; 
D = zeros(2,2);
CSTR = ss(A,B,C,D);
%%
% The plant has one unmeasured disturbance and one unmeasured output.
CSTR.InputName = {'T_c','C_A_i'};
CSTR.OutputName = {'T','C_A'};
CSTR.StateName = {'C_A','T'};
CSTR.InputGroup.MV = 1;
CSTR.InputGroup.UD = 2;
CSTR.OutputGroup.MO = 1;
CSTR.OutputGroup.UO = 2;
nxp = size(A, 1);
nmv = 1;
ny = size(C, 1);

%% Create Controller
% To improve the clarity of the example, suppress Command Window messages from 
% the MPC controller.
old_status = mpcverbosity('off');

%% 
% Create a model predictive controller with a control interval, or sample 
% time, of |1| second, and with all other properties at their default values.
Ts = 1;
MPCobj = mpc(CSTR,Ts);

%% View and Modify Controller Properties
% Display a list of the controller properties and their current values.
get(MPCobj);

%% 
% The displayed |History| value will be different for your controller,
% since it depends on when the controller was created. For a description of
% the editable properties of an MPC controller, enter |mpcprops| at the
% command line.
% 
% Use dot notation to modify these properties. For example, change the
% prediction horizon to |15|.
MPCobj.PredictionHorizon = 15;

%% 
% You can abbreviate property names provided that the abbreviation is unambiguous.
% 
% Many of the controller properties are structures containing additional
% fields. Use dot notation to view and modify these field values. For
% example, you can set the measurement units for the controller output
% variables. The |OutputUnit| property is for display purposes only and is
% optional.
MPCobj.Model.Plant.OutputUnit = {'Deg C','kmol/m^3'};

%% 
% By default, the controller has no constraints on manipulated variables
% and output variables. You can view and modify these constraints using dot
% notation. For example, set constraints for the controller manipulated
% variable.
MPCobj.MV.Min = -10;
MPCobj.MV.Max = 10;
MPCobj.MV.RateMin = -3;
MPCobj.MV.RateMax = 3;

%% 
% You can also view and modify the controller tuning weights. For example,
% modify the weights for the manipulated variable rate and the output
% variables.  In this example, we let the unmeasured output float. 
MPCobj.W.ManipulatedVariablesRate = 0.3;
MPCobj.W.OutputVariables = [1 0];

%% Generate FORCESPRO solver
% Create options structure for the FORCESPRO sparse QP solver. This uses the
% multi-stage formulation of the QP problem.
forcesOptions = mpcToForcesOptions('sparse');
% Generates the FORCESPRO solver
[forcesCoredata, forcesStatedata, forcesOnlinedata] = mpcToForces(MPCobj, forcesOptions);

%% Perform Linear Simulations
% Simulate the closed-loop response of |MPCobj| for |100| control intervals.
% Specify setpoints of |2| and |0| for the reactor temperature and the
% residual concentration respectively. The setpoint for the residual
% concentration is ignored because the tuning weight for the second output
% is zero.
T = 100;
dCSTR = c2d(CSTR, Ts);
% Reference output
yref = zeros(T, ny);
for t = 1:T
     yref(t, :) = [2, 0];
end
x0 = zeros(nxp, 1);

% Nominal simulation
X = zeros(nxp, T+1);
U = zeros(nmv, T);
Y = zeros(ny, T);
X(:, 1) = x0;
xmpc = mpcstate(MPCobj);
for t = 1:T
    Y(:, t) = dCSTR.C * X(:,t) + dCSTR.D * [U(t); 0]; % no run-time disturbance
    % Prepare inputs of mpcmoveForces
    forcesOnlinedata.signals.ref = yref(t:min(t+14,T),:);
    forcesOnlinedata.signals.ym = Y(1, t);
    % Call FORCESPRO solver
    [mv, forcesStatedata, info] = mpcmoveForces(forcesCoredata, forcesStatedata, forcesOnlinedata);
    if info.ExitFlag < 0
        warning('Internal problem in FORCESPRO solver.');
    end
    U(t) = mv;
    X(:, t+1) = dCSTR.A * X(:, t) + dCSTR.B * [U(t); 0]; % no run-time disturbance
end
%% Plot results
% Plot OV trajectory
timeVec = Ts:Ts:T;
figure(1);
subplot(2,1,1);
plot(timeVec,Y(1,:));  grid on;
xlabel('Time (sec)'); ylabel('T (deg C)');
subplot(2,1,2);
plot(timeVec,Y(2,:));  grid on;
xlabel('Time (sec)'); ylabel('C_A (kmol/m^3)');
% Plot MV trajectory
timeVec = 0:Ts:(T-Ts);
figure(2);
plot(timeVec, U);  grid on;
xlabel('Time (sec)'); ylabel('Input T_c');

%% Generate custom mex function from "mpcmoveFORCES"
% Reference outputs need to be specified before codegen in order to get the
% right dimensions
T = 100;
yref = zeros(T, ny);
for t = 1:(T+15)
     yref(t, :) = [2, 0];
end
forcesOnlinedata.signals.ref = yref(1:15,:);

if ~mpcchecktoolboxinstalled('matlabcoder')
	error('Error. Coder needs to be installed in order to generate c code from the FORCESPRO plugin.');
end

mfunName = 'mpcmoveForces';
mexfunName = 'mpcmoveForcesGen';
Cfg = coder.config('mex');
Cfg.DynamicMemoryAllocation = 'off';
Cfg.IntegrityChecks = true;
Cfg.ResponsivenessChecks = true;
Cfg.SaturateOnIntegerOverflow = true;
Cfg.ConstantInputs = 'IgnoreValues';

codegen('-config',Cfg,mfunName,'-o',mexfunName,'-args',{coder.Constant(forcesCoredata), forcesStatedata, forcesOnlinedata});

%% Perform Linear Simulations
% Simulate the closed-loop response of |MPCobj| for |100| control intervals.
% Specify setpoints of |2| and |0| for the reactor temperature and the
% residual concentration respectively. The setpoint for the residual
% concentration is ignored because the tuning weight for the second output
% is zero.
T = 100;
dCSTR = c2d(CSTR, Ts);
x0 = zeros(nxp, 1);

% Nominal simulation
X = zeros(nxp, T+1);
U = zeros(nmv, T);
Y = zeros(ny, T);
X(:, 1) = x0;
xmpc = mpcstate(MPCobj);
for t = 1:T
    Y(:, t) = dCSTR.C * X(:,t) + dCSTR.D * [U(t); 0]; % no run-time disturbance
    % Prepare inputs of mpcmoveForces
    forcesOnlinedata.signals.ref = yref(t:(t+14),:);
    forcesOnlinedata.signals.ym = Y(1, t);
    % Call FORCESPRO solver
    [mv, forcesStatedata, info] = mpcmoveForcesGen(forcesCoredata, forcesStatedata, forcesOnlinedata);
    if info.ExitFlag < 0
        warning('Internal problem in FORCESPRO solver.');
    end
    U(t) = mv;
    X(:, t+1) = dCSTR.A * X(:, t) + dCSTR.B * [U(t); 0]; % no run-time disturbance
end
%% Plot results
% Plot OV trajectory
timeVec = Ts:Ts:T;
figure(3);
subplot(2,1,1);
plot(timeVec,Y(1,:));  grid on;
xlabel('Time (sec)'); ylabel('T (deg C)');
subplot(2,1,2);
plot(timeVec,Y(2,:));  grid on;
xlabel('Time (sec)'); ylabel('C_A (kmol/m^3)');
% Plot MV trajectory
timeVec = 0:Ts:(T-Ts);
figure(4);
plot(timeVec, U);  grid on;
xlabel('Time (sec)'); ylabel('Input T_c');

%% 
% Restore the |mpcverbosity| setting.
mpcverbosity(old_status);
