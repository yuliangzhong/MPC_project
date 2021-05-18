%% Lane Following Using Nonlinear Model Predictive Control 
%% and the FORCESPRO nlmpc plugin running on a Speedgoat x86 hardware
% This example shows how to design a lane-following controller using the
% FORCESPRO Nonlinear Model Predictive Controller block, generate code for
% Speedgoat and deploy it using Simulink Real-Time.
%
% In this example, you: 
%
% # Design a nonlinear MPC controller (NLMPC) for lane following.
% # Run the model on Speedgoat
%
% This example requires Model Predictive Control Toolbox and Simulink Real-Time 
% from The MathWork.

% Copyright 2020-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

%% Introduction
% A lane-following system is a control system that keeps the vehicle
% traveling along the centerline of a highway lane, while maintaining a
% user-set velocity.
%
% A lane-following system manipulates both the longitudinal acceleration
% and front steering angle of the vehicle to:
%
% * Keep the lateral deviation $e_1$ and relative yaw angle $e_2$ small.
% * Keep the longitudinal velocity $V_x$ close to a driver set velocity.
% * Balance the above two goals when they cannot be met simultaneously.
%
  
%%
% In a separate example of lane keeping assist, it is assumed that the
% longitudinal velocity is constant. This restriction
% is relaxed in this example because the longitudinal acceleration varies
% in this MIMO control system.
%
% Another example augments a lane-following system with spacing control,
% where a safe distance from a detected lead car is also maintained.
%

%%
% This model contains four main components:
%
% # Vehicle Dynamics: Apply the _bicycle mode_ of lateral vehicle dynamics,
% and approximate the longitudinal dynamics using a time constant $\tau$.
% # Sensor Dynamics: Approximate a sensor such as a camera to calculate
% the lateral deviation and relative yaw angle. 
% # Lane Following Controller: Simulate nonlinear MPC. 
% # Curvature Previewer: Detect the curvature at the current time step and
% the curvature sequence over the prediction horizon of the MPC controller.
%
% The vehicle dynamics and sensor dynamics are discussed in more details in
% <docid:mpc_ug#mw_2025d6a5-c63c-4468-b4a2-e15164d4ccbd>. This example
% applies the same model for vehicle and sensor dynamics.

%% Parameters of Vehicle Dynamics and Road Curvature 
% The necessary Vehicle Dynamics and Road Curvature parameters are defined
% using the |LaneFollowingUsingNMPCData| script which is a |PreLoadFcn|
% callback of the model.

%% Design Nonlinear Model Predictive Controller 
% The continuous-time prediction model for NLMPC has the following state
% and output equations. The state equations are defined in
% |LaneFollowingStateFcn|. 
%
% <<../stateEqnImg.png>>
%
% <<../outputEqnImg.png>>
%
% The prediction model includes an unmeasured disturbance (UD) model. The
% UD model describes what type of unmeasured disturbance NLMPC expects to
% encounter and reject in the plant. In this example, the UD model is an
% integrator with its input assumed to be white noise. Its output is added
% to the relative yaw angle. Therefore, the controller expects a random
% step-like unmeasured disturbance occurring at the relative yaw angle
% output and is prepared to reject it when it happens.

%% 
% Create a nonlinear MPC controller with a prediction model that has seven
% states, three outputs, and two inputs. The model has two MV signals:
% acceleration and steering. The product of the road curvature and the
% longitudinal velocity is modeled as a measured disturbance, and the
% unmeasured disturbance is modeled by white noise.
nlobj = nlmpc(7,3,'MV',[1 2],'MD',3,'UD',4);

%%
% Specify the controller sample time, prediction horizon, and control
% horizon. 
nlobj.Ts = Ts;
nlobj.PredictionHorizon = 10;
nlobj.ControlHorizon = 2;

%% 
% Specify the state function for the nonlinear plant model. 
nlobj.Model.StateFcn = 'LaneFollowingStateFcn';

%% 
% Specify the output function for the nonlinear plant model. The output variables are:
%
% * Longitudinal velocity
% * Lateral deviation
% * Sum of the yaw angle and yaw angle output disturbance
%
nlobj.Model.OutputFcn = 'LaneFollowingOutputFcn';

%% 
% Set the constraints for manipulated variables.
nlobj.MV(1).Min = -3;      % Maximum acceleration 3 m/s^2
nlobj.MV(1).Max = 3;       % Minimum acceleration -3 m/s^2
nlobj.MV(2).Min = -1.13;   % Minimum steering angle -65 
nlobj.MV(2).Max = 1.13;    % Maximum steering angle 65

%% 
% Set the scale factors.
nlobj.OV(1).ScaleFactor = 15;   % Typical value of longitudinal velocity
nlobj.OV(2).ScaleFactor = 0.5;  % Range for lateral deviation
nlobj.OV(3).ScaleFactor = 0.5;  % Range for relative yaw angle
nlobj.MV(1).ScaleFactor = 6;    % Range of steering angle
nlobj.MV(2).ScaleFactor = 2.26; % Range of acceleration
nlobj.MD(1).ScaleFactor = 0.2;  % Range of Curvature

%% 
% Specify the weights in the standard MPC cost function. The third output,
% yaw angle, is allowed to float because there are only two manipulated
% variables to make it a square system. In this example, there is no
% steady-state error in the yaw angle as long as the second output, lateral
% deviation, reaches 0 at steady state.
nlobj.Weights.OutputVariables = [1 1 0];

%%
% Penalize acceleration change more for smooth driving experience.
nlobj.Weights.ManipulatedVariablesRate = [0.3 0.1];

%% 
% Validate prediction model functions at an arbitrary operating point using
% the |validateFcns| command. At this operating point:
%
% * |x0| contains the state values.
% * |u0| contains the input values.
% * |ref0| contains the output reference values.
% * |md0| contains the measured disturbance value.
%
x0 = [0.1 0.5 25 0.1 0.1 0.001 0.5];
u0 = [0.125 0.4];
ref0 = [22 0 0];
md0 = 0.1;
validateFcns(nlobj,x0,u0,md0,{},ref0);

%%
% Generate a FORCESPRO nonlinear interior point solver
%
options = nlmpcToForcesOptions();
% Set solver name
options.SolverName = 'LaneFollowSolver';
% Choose solver type 'InteriorPoint' or 'SQP'
options.SolverType = 'InteriorPoint';
% Choose Sppedgoat x86 platform to run FORCESPRO solver
options.ForcesTargetPlatform = 'Speedgoat-x86';
% x0 and u0 are used to create a primal initial guess 
options.x0 = x0;
options.mv0 = u0;
tm = tic;
[coredata, onlinedata] = nlmpcToForces(nlobj,options);
tBuild = toc(tm);

%% 
% Start code generation for Speedgoat x86
mdl = 'LaneFollowingNMPC_Speedgoat_x86';
open_system(mdl);                               % Open Simulink(R) Model
load_system(mdl);                               % Load Simulink(R) Model
rtwbuild(mdl);                                  % Start Code Generation

% Deploy application from the start
tg = slrt;
if(~strcmpi(tg.Application, 'loader'))
    tg.unload();
end
tg.load(mdl);

% Execute application
tg.start();
while(strcmpi(tg.Status, 'running'))
    pause(Ts);
end
scope1 = tg.getscope(1);
scope2 = tg.getscope(2);
scope3 = tg.getscope(3);

% Receive data from scopes
time1 = scope1.Time;
data1 = scope1.Data;
signals1 = scope1.getsignals();
time2 = scope2.Time;
data2 = scope2.Data;
signals2 = scope2.getsignals();
time3 = scope3.Time;
data3 = scope3.Data;
signals3 = scope3.getsignals();


% Find data size
signalNum1 = length(signals1);
signalNum2 = length(signals2);
signalNum3 = length(signals3);
signalNames1 = cell(1, signalNum1);
signalNames2 = cell(1, signalNum2);
signalNames3 = cell(1, signalNum3);
for i = 1:signalNum1
    signalNames1{i} = signals1(i).SigName;
end
for i = 1:signalNum2
    signalNames2{i} = signals2(i).SigName;
end
for i = 1:signalNum3
    signalNames3{i} = signals3(i).SigName;
end


tg.unload();

% Keep only necessary data
lastIndex1 = getLastTimeIndex(time1, Duration);
lastIndex2 = getLastTimeIndex(time2, Duration);
lastIndex3 = getLastTimeIndex(time3, Duration);
time1 = time1(1:lastIndex1);
time2 = time2(1:lastIndex2);
time3 = time3(1:lastIndex3);
data1 = data1(1:lastIndex1, :);
data2 = data2(1:lastIndex2, :);
data3 = data3(1:lastIndex3, :);


% Plot data
figure(1);clf;
grid on; title('Lateral deviation'); hold on;
for i = 1:signalNum1
    plot(time1, data1(:, i));
end
legend(signalNames1, 'Location', 'southoutside');
hold off;
figure(2);clf;
grid on; title('Relative yaw angle'); hold on;
for i = 1:signalNum2
    plot(time2, data2(:, i));
end
legend(signalNames2, 'Location', 'southoutside');
hold off;
figure(3);clf;
grid on; title('Longitudinal velocity'); hold on;
for i = 1:signalNum3
    plot(time3, data3(:, i));
end
legend(signalNames3, 'Location', 'southoutside');
hold off;


function stopIndex = getLastTimeIndex(timeData, Tstop)
    stopIndex = 1;
    currentTime = 0;
    for i = 1:length(timeData)
        if(timeData(i) < currentTime || timeData(i) > Tstop)
            break;
        end
        currentTime = timeData(i);
        stopIndex = i;
    end
end

