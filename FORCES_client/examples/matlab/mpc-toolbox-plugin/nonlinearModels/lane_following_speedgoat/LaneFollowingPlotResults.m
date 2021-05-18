function LaneFollowingPlotResults(logsout)
% Plot and compare results for nonlinear and adaptive mpc. 
% Copyright 2020-2021 The MathWorks, Inc. and Embotech AG, Zurich, Switzerland

%% Get the data from simulation
[e1_nmpc,e2_nmpc,vx_nmpc] = getData(logsout);

%% Plot results. 
figure; % lateral results
% lateral deviation
subplot(2,1,1);
hold on;
grid on;
plot(e1_nmpc.Values.time,e1_nmpc.Values.Data);
legend('Nonlinear MPC');
title('Lateral Deviation (e1) vs Time');
xlabel('Time(s)');
ylabel('Lateral Deviation(m)');
hold off;
% relative yaw angle
subplot(2,1,2);
hold on;
grid on;
plot(e2_nmpc.Values.Time,e2_nmpc.Values.Data);
legend('Nonlinear MPC');
title('Relative Yaw Angle (e2) vs Time');
xlabel('Time(s)');
ylabel('Relative Yaw Angle(radians)');
hold off;

figure; % longitudinal results
% longitudinal velocity
hold on;
grid on;
plot(vx_nmpc.Values.Time,vx_nmpc.Values.Data);
legend('Nonlinear MPC');
title('Velocity (Vy) vs Time');
xlabel('Time(s)');
ylabel('Velocity(m/s)');
hold off;

%% Local function: Get data from simulation
function [e1,e2,vx] = getData(logsout)
e1 = logsout.getElement('Lateral Deviation');    % lateral deviation
e2 = logsout.getElement('Relative Yaw Angle');   % relative yaw angle
vx = logsout.getElement('Longitudinal Velocity');% velocity of host car