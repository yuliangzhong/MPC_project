% Spacecraft Rendezvous maneouver
% 
%  min   sum_{i=1}^{N-1} ||Q*(xi-xtarget)||_2 + ||R*(ui-utarget||_1
% xi,ui
%       s.t. x1 = x
%            x_i+1 = A(i)*xi + B(i)*ui  for i = 1...N-1
%            umin <= ui <= umax     for i = 1...N
%            Hx*x_i <= hx           for i = 1...N
%
% (c)   Embotech AG, Zurich, Switzerland, 2014.
%       Ed Hartley, Cambridge University, UK.

clc
clear

%% system
cfg_MPC = MSRE_constants; % get constants
M_chs_MPC = 1;      % Input is an acceleration impulse
Ts_MPC = 200;       % control sampling time
nx = 6;         % velocities and acceleration in 3 axes
nu = 3;         % thrusters on 3 axis

%% MPC setup
N = 30;         
Q = diag([0.001 0.001 0.001 0 0 0]);
R = diag([15000 15000 15000]);
umin = -5;     umax = 5;
cone_angle = 20*pi/180;
Hx = [1 0 0 0 0 0];
hx = 0;                 % Must remain behind the target
Hx = [  Hx; ...
        cone_angle 0 1 0 0 0; ...       % Visibility cone
        cone_angle 0 -1 0 0 0];         % Visibility cone
hx = [hx;0;0];

xs = [-1000; 0; 0; 0; 0; 0];    % target state (1km away)
us = zeros(3,1);

%% FORCESPRO multistage form - 2-norm
% assume variable ordering zi = [ui, xi+1, eui] for i=1...N

stages = MultistageProblem(N);
params(1) = newParam('minusA_times_x0',1,'eq.c'); % RHS of first eq. constr. is a parameter: z1=-A*x0
for i = 1:N
    
        % dimension
        stages(i).dims.n = nx+2*nu; % number of stage variables
        stages(i).dims.r = nx;      % number of equality constraints        
        stages(i).dims.l = nu;      % number of lower bounds
        stages(i).dims.u = nu;      % number of upper bounds
        stages(i).dims.p = 3+2*nu;  % number of polytopic constraints
            
        % cost
        stages(i).cost.H = blkdiag(zeros(nu),Q,zeros(nu));
        stages(i).cost.f = [zeros(nu,1); -Q*xs; ones(nu,1)];
        
        % lower bounds
        stages(i).ineq.b.lbidx = 1:nu;      % lower bound acts on these indices
        stages(i).ineq.b.lb = umin*ones(nu,1); % lower bound for this stage variable
        
        % upper bounds
        stages(i).ineq.b.ubidx = 1:nu;      % upper bound acts on these indices
        stages(i).ineq.b.ub = umax*ones(nu,1); % upper bound for this stage variable
        
        % polytopic bounds
        stages(i).ineq.p.A  = [ zeros(3,nu), Hx, zeros(3,nu); ...
                                R, zeros(nu,nx), -eye(nu); ... 
                                -R, zeros(nu,nx), -eye(nu)];  
        stages(i).ineq.p.b  = [ hx; R*us; -R*us ];  

        % equality constraints
        if( i>1 )
            stages(i).eq.c = zeros(nx,1);
        end
        params(end+1) = newParam(['D_',num2str(i)],i,'eq.D');
        if( i < N)
            params(end+1) = newParam(['C_',num2str(i)],i,'eq.C');
        end
        
end

% define outputs of the solver
outputs(1) = newOutput('u0',1,1:nu);

% solver settings
codeoptions = getOptions('FORCESPro_spacecraft_controller');
codeoptions.maxit = 40;

% generate code
generateCode(stages,params,codeoptions,outputs);



%% simulate maneouver
x1 = [-15e3; 0; 0; 0; 0; 0];    % initial state (15km away)    
xs = [-1000; 0; 0; 0; 0; 0];    % target state (1km away)
us = zeros(3,1);
kmax = 6000;    % simulation samples
Ts_SIM = 1;     % simulation sampling time
X = zeros(nx,kmax+1); X(:,1) = x1;
U = zeros(nu,kmax);
nu0 = 0;

for k = 1:kmax

    if mod(k-1,Ts_MPC) == 0 % only run the controller every Ts_MPC seconds
        % get prediction model 
        nucopy = nu0;
        [A, nucopy] = YA_A_matrix(cfg_MPC, nucopy, Ts_MPC*Ts_SIM);        
        B = A*[zeros(3);eye(3)];
        problem.minusA_times_x0 = -A*X(:,k);
        problem = setfield(problem,'D_1', [B, -eye(nx), zeros(nx,nu)]); 
        for i = 2:N
            [A, nucopy] = YA_A_matrix(cfg_MPC, nucopy, Ts_MPC*Ts_SIM);
            B = A*[zeros(3);eye(3)];
            problem = setfield(problem,['D_',num2str(i)], [B, -eye(nx), zeros(nx,nu)]); 
            problem = setfield(problem,['C_',num2str(i-1)], [zeros(nx,nu), A, zeros(nx,nu)]); 
        end   
                
        [solverout,exitflag,info] = FORCESPro_spacecraft_controller(problem);   % solve
        if( exitflag == 1 )
            U(:,k) = solverout.u0;
            info.solvetime
        else
            info
            error('Some problem in solver');
        end
    else
        U(:,k) = zeros(3,1);
    end
    
    % simulate dynamics
    [A_SIM, nu0] = YA_A_matrix(cfg_MPC, nu0, Ts_SIM);
    B_SIM = A_SIM*[zeros(3);eye(3)];
    X(:,k+1) = A_SIM*X(:,k) + B_SIM*U(:,k);
end


%% Compuate some fuel usage stats
%
fprintf(1, 'Statistics Computing phase\n');
utotal_onethrust = sum(sqrt(sum(U.^2,1)));
utotal_xyzthrust = sum(sum(abs(U),2));
fprintf('Fuel usage using single thruster: %5.4f\n', utotal_onethrust);
fprintf('Fuel usage using 3D thrusters: %5.4f\n', utotal_xyzthrust);


%% Plot a nice graph
%
fprintf(1, 'Plotting phase\n');
figure;
clf
subplot(4,2,[1,3,5,7]);
line(-[0,15], -tan(20*pi/180)*[0,15], 'color', 'r', 'linewidth', 3); hold on;
line(-[0,15], tan(20*pi/180)*[0,15], 'color', 'r', 'linewidth', 3);
plot(X(1,:)/1e3, X(3,:)/1e3, 'b', 'linewidth', 2); hold on;
set(gca, 'Xdir', 'reverse');
set(gca, 'Ydir', 'reverse');
xlabel('x_{tgt} (km)');
ylabel('z_{tgt} (km)');
grid on;

subplot(4,2,2);
plot(X(1,:)/1e3); hold on;
set(gca, 'Ydir', 'reverse');
set(gca, 'xlim', [0 kmax]);
line([0 kmax], xs(1)/1e3*[1 1], 'color', 'k', 'linestyle', '--', 'linewidth', 2);
ylabel('x_{tgt} (km)');
xlabel('Time (s)');
grid on;

subplot(4,2,4);
plot(X(3,:)/1e3);
set(gca, 'Ydir', 'reverse');
set(gca, 'xlim', [0 kmax]);
line([0 kmax], xs(3), 'linestyle', '--')
line([0 kmax], xs(3)/1e3*[1 1], 'color', 'k', 'linestyle', '--', 'linewidth', 2);
ylabel('z_{tgt} (km)');
xlabel('Time (s)');
grid on;

subplot(4,2,6);
bar(U(1,:));
set(gca, 'Ydir', 'reverse');
set(gca, 'xlim', [0 kmax]);
xlabel('Time (s)');
ylabel('u_x (m/s)');
grid on;

subplot(4,2,8);
bar(U(3,:));
xlabel('Time (s)');
ylabel('u_z (m/s)');
grid on;
set(gca, 'Ydir', 'reverse');
set(gca, 'xlim', [0 kmax]);

