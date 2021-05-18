% HOW TO: Reference Tracking with FORCESPRO
%
% (c) Embotech AG, Zurich, Switzerland, 2013 - 2014. Email: info@embotech.com

clear all; clc; close all;

%% System
A = [1.1 1; 0 1];
B = [.8, 0.1;0.3,.8];
[nx,nu] = size(B);

%% MPC setup
N = 10;
Q = eye(nx);
R = eye(nu);
if( exist('dlqr','file') )
    [~,P] = dlqr(A,B,Q,R);
else
    P = 10*Q;
end
umin = [-.8;-.8];     umax = [1.3;1.3];
xmin = [-5; -5];      xmax = [5; 5];

%% FORCESPRO multistage form
% assume variable ordering zi = [ui; xi+1] for i=1...N-1

stages = MultistageProblem(N);
for i = 1:N
    
        % dimension
        stages(i).dims.n = nx+nu; % number of stage variables
        stages(i).dims.r = nx;    % number of equality constraints        
        stages(i).dims.l = nx+nu; % number of lower bounds
        stages(i).dims.u = nx+nu; % number of upper bounds
        
        % cost
        stages(i).cost.H = blkdiag(R,Q);
        
        % lower bounds
        stages(i).ineq.b.lbidx = 1:(nu+nx); % lower bound acts on these indices
        stages(i).ineq.b.lb = [umin; xmin]; % lower bound for this stage variable
        
        % upper bounds
        stages(i).ineq.b.ubidx = 1:(nu+nx); % upper bound acts on these indices
        stages(i).ineq.b.ub = [umax; xmax]; % upper bound for this stage variable
        
        % equality constraints
        if( i < N )
            stages(i).eq.C = [zeros(nx,nu), A];
        end
        if( i>1 )
            stages(i).eq.c = zeros(nx,1);
        end
        stages(i).eq.D = [B, -eye(nx)];
        
end
params(1) = newParam('Reference_Value',1:N,'cost.f'); % Reference Value on the states and inputs
params(2) = newParam('minusA_times_x0',1,'eq.c'); % RHS of first eq. constr. is a parameter   
 

%% Define outputs of the solver
outputs(1) = newOutput('u0',1,1:nu);

%% Solver settings
codeoptions = getOptions('FORCESPro_Reference_Tracking');

%% Generate code
generateCode(stages,params,codeoptions,outputs);

%% Simulate
x1 = [-5; 4];
kmax = 35;
X = zeros(nx,kmax+1); X(:,1) = x1;
X_ref = zeros(nx,kmax+1); X_ref(1,14:end) = 2; X_ref(2,22:end) = -1;
U = zeros(nu,kmax);
U_ref = zeros(nu,kmax);
for k = 1:kmax
    U_ref(:,k) = inv(B)*[eye(nx)-A]*X_ref(:,k);
    problem.Reference_Value = [-U_ref(:,k)'*R, -X_ref(:,k)'*Q]';
    problem.minusA_times_x0 = -A*X(:,k);
    [solverout,exitflag,info] = FORCESPro_Reference_Tracking(problem);
    if( exitflag == 1 )
        U(:,k) = solverout.u0;
    else
        info
        error('Some problem in solver');
    end
    X(:,k+1) = A*X(:,k) + B*U(:,k);
end

%% Plot
h = figure(1); clf;
grid on; h_title = title('States x'); set(h_title,'FontSize',14); hold on;
stairs(1:kmax,X(1,1:kmax)','g'); hold on; 
stairs(1:kmax,X(2,1:kmax)','b'); hold on;
stairs(1:kmax,X_ref(1,1:kmax),'k--'); hold on; 
stairs(1:kmax,X_ref(2,1:kmax),'k--'); hold on;
plot([1 kmax], [xmax xmax]', 'r--'); hold on;
plot([1 kmax], [xmin xmin]', 'r--'); hold on;
h_xlabel=xlabel('Time step k'); h_ylabel=ylabel('Maginute');
set(h_xlabel, 'FontSize', 12); set(h_ylabel, 'FontSize', 12);
h_legend = legend('x_1','x_2'); set(h_legend,'FontSize',14);
ylim(1.1*[min(xmin),max(xmax)]);
hline = findobj(gcf, 'type', 'line');
set(hline,'LineWidth',1.2);
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 7 4])
print(h,'-depsc','States_Reference_Tracking');

h = figure(2); clf;
grid on; h_title = title('Input Signal u'); set(h_title,'FontSize',14); hold on;
stairs(1:kmax,U(1,1:kmax)','g'); hold on;
stairs(1:kmax,U(2,1:kmax)','b'); hold on;
plot([1 kmax], [umax umax]', 'r--'); hold on;
plot([1 kmax], [umin umin]', 'r--');
h_xlabel=xlabel('Time step k'); h_ylabel=ylabel('Maginute');
set(h_xlabel, 'FontSize', 12); set(h_ylabel, 'FontSize', 12);
ylim(1.1*[min(umin),max(umax)]);
h_legend = legend('u_1','u_2'); set(h_legend,'FontSize',14);
hline = findobj(gcf, 'type', 'line');
set(hline,'LineWidth',1.2);
set(gcf,'PaperUnits','inches','PaperPosition',[0 0 7 4])
print(h,'-depsc','Input_Reference_Tracking');