clear all; clc; close all;

% Robust Kalman Filter
%
% Based on lecutre material of the course 'Recursive Estimation'  Spring 2014,
% Institute for Dynamic Systems and Control, ETH Zurich and the publication
% 'Real-Time Convex Optimization in Signal Processing' by J. Mattingley and
% S. Boyd, IEEE Signal Processing Magazine, 27(3):50-61, May 2010.
% 
% (c) Embotech AG, Zurich, Switzerland, 2013 - 2014. Email: info@embotech.com

% Set generate_code to 0 if no code generate is needed, i.e. a solver has
% already been generated.
generate_code = 1;

%% Simulation settings:
N = 1000;
lambda = 2; % tuning parameter

%% Simulate the real system
% Distribution parameters
x0 = 5*ones(3,1);
P0 = eye(3);
Q = diag([1/40, 1/10, 1/5]);
R = 0.5*eye(2);

% Draw samples of the process noise for all time steps:
v = (randn(N,3)*chol(Q))';
   
% Vector for full state history:
x = zeros(3,N+1);
% Draw random initial condition:
x(:,1) = x0 + chol(P0)*randn(3,1);

% Control input:
u = zeros(1,N);
u(1,1:500) = 5*ones(1,500);
u(1,501:end) = 2*ones(1,500);

% State dynamics 
alpha1 = 0.1;
alpha2 = 0.5;
alpha3 = 0.2;
    
B = [0.5;0.5;0];

A = [1-alpha1, 0, 0; 0, 1-alpha2, 0; alpha1, alpha2, 1-alpha3];
Ak = repmat(A, [1 1 N]);

% Measurements:
% Vector for full measurement history:
z = zeros(2,N+1);

% H matrix:
H = [1 0 0; 0 0 1];

% Draw samples of the measurement noise for all time steps:
w = (randn(N+1,2)*chol(R))';

% Additional noise term: Simulate sensor failures
numberOfOnes = N/20;
indexes = randperm(N);
y = zeros(N+1,2)';
for i = 1:numberOfOnes
    y(1,indexes(i)) = 25*randn;
end
indexes = randperm(N);
numberOfOnes = N/20;
indexes = randperm(N);
for i = 1:numberOfOnes
    y(2,indexes(i)) = 25*randn;
end

% Compute state and measurement history from Ak, B, u, v, H, w and y:
for k = 2:(N+1)
    x(:,k) = Ak(:,:,k-1)*x(:,k-1)+B*u(k-1)+v(:,k-1);
    z(:,k) = H*x(:,k)+w(:,k)+y(:,k); % with sensor failures
end

%% Run the Standard Kalman Filter.

% Initial mean and variance:
x0_hat = x0;
P0_hat = P0;

% Mean and variance history vectors:
x_hat = zeros(3,N+1);
P_hat = zeros(3,3,N+1);
P_p = zeros(3,3,N+1);

% Set initial mean and variance.
x_hat(:,1) = x0_hat;
P_hat(:,:,1) = P0_hat;

% Pre-compute the inverse of R because we need it all the time.
R_inv = inv(R);

% Execute the Kalman Filter equations for each time step:
for i = 2:(N+1)
    x_p = Ak(:,:,i-1)*x_hat(:,i-1)+B*u(i-1);
    P_p(:,:,i) = Ak(:,:,i-1)*P_hat(:,:,i-1)*Ak(:,:,i-1)' + Q;
    
    P_hat(:,:,i) = inv(inv(P_p(:,:,i)) + H'*R_inv*H);
    x_hat(:,i) = x_p + P_hat(:,:,i)*H'*R_inv*(z(:,i)-H*x_p);
end

% RMS Error Magnitude Standard Kalman Filter 
RMS_SKF = NaN(N+1,3);
for i = 1:3
    RMS_SKF(:,i) = sqrt((x(i,:)' - x_hat(i,:)').^2/N);
end

if (generate_code == 1)
%% Create FORCESPRO Solver for Measurement Update Step
% assume variable ordering z = [x,w,y,e]

% Create Multistage Structure
stages = MultistageProblem(1);

% dimension
[ny nx] = size(H);
nw = ny;
ne = ny;
stages(1).dims.n = nx+nw+ny+ne; % number of stage variables
stages(1).dims.r = ny;          % number of equality constraints        
stages(1).dims.p = 2*ne;        % number of polytopic constraints
    
% polytopic bounds
stages(1).ineq.p.A  = [zeros(ny,nx), zeros(ny,nw), lambda*eye(ny), -eye(ne); ...
                       zeros(ny,nx), zeros(ny,nw), -lambda*eye(ny), -eye(ne)]; 
stages(1).ineq.p.b  = zeros(2*ne,1); 
        
% equality constraints
stages(1).eq.D = [H, eye(nw), eye(ny), zeros(ne)];
        
% Parameters
params(1) = newParam('H_i',1,'cost.H');
params(2) = newParam('f_i',1,'cost.f');
params(3) = newParam('z_i',1,'eq.c');
    
% Output
outputs(1) = newOutput('x_hat_RKF',1,1:3);

% Code Generation
codeoptions = getOptions('Robust_KF');
generateCode(stages,params,codeoptions,outputs);
end

%% Run the Robust Kalman Filter with FORCESPRO
% We replace the standart measurement update with the solution of a
% minimization problem, that includes an l_1 term to handle the sparse
% noise.

% Initial mean and variance:
x0_hat = x0;
P0_hat = P0;

P_p_RKF = zeros(3,3,N+1);

P_hat_RKF = zeros(3,3,N+1);
P_hat_RKF(:,:,1) = P0_hat;

% Mean and variance history vectors:
x_hat_RKF = zeros(3,N+1);

% Set initial mean and variance.
x_hat_RKF(:,1) = x0_hat;

% Pre-compute the inverse of R because we need it all the time.
R_inv = inv(R);

% Safe Computation Time
solve_time = NaN(1,N);

% Execute the Kalman Filter equations for each time step
problem.z1 = zeros(nx+nw+ny+ne,1);
for i = 2:(N+1)
    
    % Prediction Step
    x_p_RKF = Ak(:,:,i-1)*x_hat_RKF(:,i-1)+B*u(i-1);
    P_p_RKF(:,:,i) = Ak(:,:,i-1)*P_hat_RKF(:,:,i-1)*Ak(:,:,i-1)' + Q;
  
    % Measurement Update Step - Optimization Problem
    problem.H_i = [2*inv(P_p_RKF(:,:,i)),zeros(nx,nw+ny+ne);...
                   zeros(ny,nx),2*R_inv,zeros(ny,ny+ne);...
                   zeros(ny+ne,nx+nw+ny+ne)];
    problem.f_i = [-2*(inv(P_p_RKF(:,:,i))*x_p_RKF);...
                   zeros(nw,1);...
                   zeros(ny,1);...
                   ones(ne,1)];
    problem.z_i = z(:,i);
    [solverout,exitflag,info] = Robust_KF(problem);
    solve_time(i-1) = info.solvetime;
    P_hat_RKF(:,:,i) = P_p_RKF(:,:,i);
    x_hat_RKF(:,i) = solverout.x_hat_RKF;
    
end

mst = mean(solve_time);

% Mean solve time
fprintf('\nThe mean solve time used by the FORCESPRO solve is %d seconds.\n',mst)

% RMS Error Magnitude Standard Kalman Filter
RMS_RKF = NaN(N+1,3);
for i = 1:3
    RMS_RKF(:,i) = sqrt((x(i,:)' - x_hat_RKF(i,:)').^2/N);
end

%% Improvement in RMS error in percentage

for i = 1:3
    imp(i) = 100-100/mean(RMS_SKF(:,i))*mean(RMS_RKF(:,i));
    fprintf('\nRMS error of state %i estimate is reduced by %.2f percent.\n',i,imp(i))
end


%% Plot results
for i = 1:3
    h=figure(i); clf;
    p1 = plot(0:N, x(i,:), 'k-', 0:N, x_hat(i,:), 'r',0:N, x_hat_RKF(i,:),'b'); hold on;
    p2 = plot(0:N, x_hat_RKF(i,:)'+sqrt(squeeze(P_hat_RKF(i,i,:))), 'g--',...
         0:N, x_hat_RKF(i,:)'-sqrt(squeeze(P_hat_RKF(i,i,:))), 'g--');

    h_legend = legend('True state', 'Estimated via SKF', 'Estimated via RKF','+/- 1 standard deviation');
    set(h_legend,'FontSize',14);
    %hline = findobj(gcf, 'type', 'line');
    set(p1,'LineWidth',1.2);
    set(p2,'LineWidth',1.1);
    xlim([250 500]);
    h_ylabel=ylabel(['Tank ' num2str(i) ' Level, x(',int2str(i),')']);
    h_xlabel=xlabel('Time step k');
    set(h_xlabel, 'FontSize', 14); set(h_ylabel, 'FontSize', 14);
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 15 5])
    print(h,'-depsc',['Estimated_Water_Tank_Levels_',int2str(i)]);
    
    h=figure(i+3); clf;
    plot(0:N, RMS_SKF(:,i),'r');
    title(['RMS Error state x(',int2str(i),')'], 'FontSize', 12);
    hold on; plot(0:N, RMS_RKF(:,i),'b');
    hline = findobj(gcf, 'type', 'line');
    set(hline,'LineWidth',1.2);
    xlim([250 500]);
    h_xlabel=xlabel('Time step k');
    h_ylabel=ylabel('RMS Error Maginute');
    set(h_xlabel, 'FontSize', 12); set(h_ylabel, 'FontSize', 12);
    h_legend = legend('SKF','RKF');
    set(h_legend,'FontSize',12);
    set(gcf,'PaperUnits','inches','PaperPosition',[0 0 3 3])
    print(h,'-depsc',['RMS_Error_Magniuted_Estimates_',int2str(i)]);
end

