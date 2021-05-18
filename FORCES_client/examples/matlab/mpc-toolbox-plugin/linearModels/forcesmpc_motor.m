%% DC Servomotor with Constraint on Unmeasured Output
% This example shows how to design a model predictive controller for a DC
% servomechanism under voltage and shaft torque constraints. 
%
% This example requires Model Predictive Control Toolbox from MathWorks.

% Copyright 2019-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

%% Define DC-Servo Motor Model
% The linear open-loop dynamic model is defined in |plant|. Variable
% |tau| is the maximum admissible torque to be used as an output
% constraint.
[plant,tau] = mpcmotormodel;

%%
% Specify input and output signal types for the MPC controller. The
% second output, torque, is unmeasurable.
plant = setmpcsignals(plant,'MV',1,'MO',[1 2]);

%% Specify MV Constraints
% The manipulated variable is constrained between +/- 220 volts. Since the
% plant inputs and outputs are of different orders of magnitude, you also
% use scale factors to facilitate MPC tuning. Typical choices of scale
% factor are the upper/lower limit or the operating range.
umin = -220; % Lower bound on manipulated variable
umax = 220; % Upper bound on manipulated variable
MV = struct('Min',umin,'Max',umax,'ScaleFactor',440);

%% Specify OV Constraints
% Torque constraints are only imposed during the first three prediction
% steps.
OV = struct('Min',{-Inf, [-tau;-tau;-tau;-Inf]},...
            'Max',{Inf, [tau;tau;tau;Inf]},...
            'ScaleFactor',{2*pi, 2*tau});

%% Specify Tuning Weights
% The control task is to get zero tracking offset for the angular position.
% Since you only have one manipulated variable, the shaft torque is allowed
% to float within its constraint by setting its weight to zero.
Weights = struct('MV',0,'MVRate',0.1,'OV',[1.0 0]);

%% Create MPC controller
% Create an MPC controller with sample time |Ts|, prediction horizon |p|,
% and control horizon |m|.
Ts = 0.1;
p = 10;
m = 1;
mpcobj = mpc(plant,Ts,p,m,Weights,MV,OV);

%% Generate FORCESPRO solver
% Create options structure for FORCESPRO solver using the Sparse QP
% formulation
options = mpcToForcesOptions();
% Generate solver
[coredata, statedata, onlinedata] = mpcToForces(mpcobj, options);

%% Simulate Controller Using simulateMpcForces
% Use the |sim| function to simulate the closed-loop control of the linear
% plant model in MATLAB.
disp('Now simulating nominal closed-loop behavior');
Tstop = 8;                      % seconds
Tf = round(Tstop/Ts);           % simulation iterations
% Discretize system
dMotor = c2d(ss(plant.A, plant.B, plant.C, plant.D), Ts);
yref = [pi*ones(Tf,1) zeros(Tf,1)];
x0 = zeros(size(plant.A,1),1);
% Run nominal simulation using FORCESPRO solver
[Y, U] = simulateMpcForces(mpcobj, dMotor, Tf, x0, yref, [1 2], coredata, statedata, onlinedata, []);

%% Plot results.
timeVec = Ts:Ts:(Tf*Ts);
figure();
subplot(2,1,1);
plot(timeVec, Y(1,:), timeVec, yref);
xlabel('Time (sec)'); ylabel('y_1 (rad)'); grid on;
subplot(2,1,2);
plot(timeVec, Y(2,:));
xlabel('Time (sec)'); ylabel('y_2 (Nm)'); grid on;
figure();
plot(timeVec, U(1,:)); hold on;
plot(timeVec, umin * ones(1, length(timeVec)), 'r-.'); hold on;
plot(timeVec, umax * ones(1, length(timeVec)), 'r-.');
xlabel('Time (sec)'); ylabel('u_1'); grid on;



