%% Simulate MPC Controller with a Custom QP Solver
% You can simulate the closed-loop response of an MPC controller with a
% custom quadratic programming (QP) solver in Simulink(R). This example
% shows how to do it with a FORCESPRO solver.
%
% This example uses an on-line monitoring application, first solving it
% using the Model Predictive Control Toolbox(TM) built-in solver, then
% using a custom solver that wraps the FORCESS-PRO dense QP solver.
%
% This example requires Model Predictive Control Toolbox from MathWorks.

% Copyright 2019-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

%% 
% In the on-line monitoring example, the |qp.status| output of the MPC
% Controller block returns a positive integer whenever the controller
% obtains a valid solution of the current run-time QP problem and sets the
% |mv| output. The |qp.status| value corresponds to the number of
% iterations used to solve this QP.
%
% If the QP is infeasible for a given control interval, the controller
% fails to find a solution. In that case, the |mv| outport stays at its
% most recent value and the |qp.status| outport returns |-1|. Similarly,
% if the maximum number of iterations is reached during optimization
% (rare), the |mv| outport also freezes and the |qp.status| outport returns
% |0|.
%
% Real-time MPC applications can detect whether the controller is in a
% "failure" mode (|0| or |-1|) by monitoring the |qp.status| outport. If a
% failure occurs, a backup control plan should be activated. This is
% essential if there is any chance that the QP could become infeasible,
% because the default action (freezing MVs) may lead to unacceptable system
% behavior, such as instability. Such a backup plan is, necessarily,
% application-specific.

%% MPC Application with Online Monitoring 
% The plant used in this example is a single-input, single-output system
% with hard limits on both the manipulated variable (MV) and the controlled
% output (OV). The control objective is to hold the OV at a setpoint of
% |0|. An unmeasured load disturbance is added to the OV.  This disturbance
% is initially a ramp increase. The controller response eventually
% saturates the MV at its hard limit. Once saturation occurs, the
% controller can do nothing more, and the disturbance eventually drives the
% OV above its specified hard upper limit. When the controller predicts
% that it is impossible to force the OV below this upper limit, the
% run-time QP becomes infeasible.
%%
% Define the plant as a first-order SISO system with unity gain.
Plant = tf(1,[2 1]);

%%
% Define the unmeasured load disturbance. The signal ramps up from |0| to
% |2| between |1| and |3| seconds, then ramps back down from |2| to |0|
% between |3| and |5| seconds.
LoadDist = [0 0; 1 0; 3 2; 5 0; 7 0];

%% Design MPC Controller
% Create an MPC object using the model of the test plant. The chosen
% control interval is about one tenth of the dominant plant time constant.
Ts = 0.2;
Obj = mpc(Plant, Ts);

%%
% Define hard constraints on the plant input (MV) and output (OV). By
% default, all the MV constraints are hard and OV constraints are soft.
Obj.MV.Min = -0.5;
Obj.MV.Max =  1;
Obj.OV.Min = -1;
Obj.OV.Max =  1;
Obj.OV.MinECR = 0; % change OV lower limit from soft to hard
Obj.OV.MaxECR = 0; % change OV upper limit from soft to hard

%%
% Generally, hard OV constraints are discouraged and are used here only to
% illustrate how to detect an infeasible QP. Hard OV constraints make
% infeasibility likely, in which case a backup control plan is essential.
% This example does not include a backup plan. However, as shown in the
% simulation, the default action of freezing the single MV is the best
% response in this simple case.

%% Simulate Using Simulink with Built-in QP Solver
% To run this example, Simulink and the Optimization Toolbox are
% required.
if ~mpcchecktoolboxinstalled('simulink')
    disp('Simulink is required to run this example.')
    return
end

%% 
% Build the control system in a Simulink model and enable the |qp.status|
% outport by selecting the *Optimization status* parameter of the MPC
% Controller block. Display the run-time |qp.status| value in the
% Controller Status scope.
mdl = 'mpc_onlinemonitoring';
open_system(mdl)

%%
% Simulate the closed-loop response using the default Model Predictive
% Control Toolbox QP solver.
open_system([mdl '/Controller Status'])
open_system([mdl '/Response'])
sim(mdl)

%% Explanation of the Closed-Loop Response
% As shown in the response scope, at 1.4 seconds, the increasing
% disturbance causes the MV to saturate at its lower bound of -0.5, which
% is the QP solution under these conditions (because the controller is
% trying to hold the OV at its setpoint of |0|).
%
% The OV continues to increase due to the ramp disturbance and, at 2.2
% seconds, exceeds the specified hard upper bound of |1.0|.  Since the QP
% is formulated in terms of predicted outputs, the controller still
% predicts that it can bring OV back below 1.0 in the next move and
% therefore the QP problem is still feasible.
%
% Finally, at t = 3.2 seconds, the controller predicts that it can no
% longer move the OV below 1.0 within the next control interval, and the QP
% problem becomes infeasible and |qp.status| changes to -1 at this time.
%
% After three seconds, the disturbance is decreasing. At 3.8 seconds, the
% QP becomes feasible again.  The OV is still well above its
% setpoint, however, and the MV remains saturated until 5.4 seconds,
% when the QP solution is to increase the MV as shown. From then on, the
% MV is not saturated, and the controller is able to drive the OV back to
% its setpoint.
%%
% When the QP is feasible, the built-in solver finds the solution in three
% iterations or less.

%% Generate FORCESPRO QP solver
% Create options structure for Dense QP formulation
options = mpcToForcesOptions('dense');
% Generate solver
[forcesCoredata, forcesStatedata, forcesOnlinedata] = mpcToForces(Obj, options);

%%
% When the FORCESPRO QP solver converges to an optimal solution, the QP
% status is 1. When it is run on an infeasible problem it outputs -7 or 0. 

%% Simulate with a Custom QP Solver
% To examine how the custom solver behaves under the same conditions,
% activate the custom solver option by setting the |Optimizer.CustomSolver|
% property of the MPC controller.
Obj.Optimizer.CustomSolver = true;
Obj.Optimizer.CustomSolverCodeGen = true;

%%
% Repeat the simulation.
set_param([mdl '/Controller Status'],'ymax','10');
sim(mdl)

%%
% The plant input and output signals are identical to those obtained using
% the built-in Model Predictive Control Toolbox solver. 
% FORCESPRO detects the same infeasibility time period.
