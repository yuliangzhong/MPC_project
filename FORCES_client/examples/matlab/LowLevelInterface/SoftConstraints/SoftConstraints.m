% Soft constraints example
% 
%  min   xN'*P*xN + sum_{i=1}^{N-1} xi'*Q*xi + ui'*R*ui + di'*S*di
% xi,ui
%       s.t. x1 = x
%            x_i+1 = A*xi + B*ui            for i = 1...N-1
%       xmin - di <= xi(1) <= xmax + di     for i = 1...N
%            umin <= ui    <= umax          for i = 1...N
%                    di    >= 0             for i = 1...N
%
% and P is solution of Ricatti eqn. from LQR problem
%
% (c) Embotech AG, Zurich, Switzerland, 2015.

%% system
A = [0.71 -0.43; 0.43 0.88];
B = [0.2; 0.05];
[nx,nu] = size(B);

%% MPC setup
N = 10;
Q = eye(nx);
R = 10*eye(nu);
if( exist('dlqr','file') )
    [~,P] = dlqr(A,B,Q,R);
else
    P = 10*Q;
end
S = 10000;
umin = -1.2;    umax = 0.5;
xmax = 1.5;     xmin = -4; 


%% FORCESPRO multistage form
% assume variable ordering zi = [ui; xi+1; di+1] for i=1...N-1

stages = MultistageProblem(N);
for i = 1:N
    
        % dimension
        stages(i).dims.n = nx+nu+1; % number of stage variables
        stages(i).dims.r = nx;      % number of equality constraints        
        stages(i).dims.l = nu;      % number of lower bounds
        stages(i).dims.u = nu;      % number of upper bounds
        stages(i).dims.p = 3;       % number of polytopic bounds
        
        % cost
        if( i == N )
            stages(i).cost.H = blkdiag(R,P,S);
        else
            stages(i).cost.H = blkdiag(R,Q,S);
        end
        stages(i).cost.f = zeros(nx+nu+1,1);
        
        % lower bounds
        stages(i).ineq.b.lbidx = 1:nu;  % lower bound acts on these indices
        stages(i).ineq.b.lb = umin;     % lower bound for this stage variable
        
        % upper bounds
        stages(i).ineq.b.ubidx = 1:nu;  % upper bound acts on these indices
        stages(i).ineq.b.ub = umax;     % upper bound for this stage variable
        
        % polytopic constraints 
        stages(i).ineq.p.A = [zeros(1,nu), zeros(1,nx), -1; ...
                              zeros(1,nu), 1, 0,        -1; ...
                              zeros(1,nu), -1, 0,       -1];
        stages(i).ineq.p.b = [0; ...
                              xmax; ...
                              -xmin];
        
        % equality constraints
        if( i < N )
            stages(i).eq.C = [zeros(nx,nu), A, zeros(nx,1)];
        end
        if( i>1 )
            stages(i).eq.c = zeros(nx,1);
        end
        stages(i).eq.D = [B, -eye(nx), zeros(nx,1)];
        
end
params(1) = newParam('minusA_times_x0',1,'eq.c'); % RHS of first eq. constr. is a parameter: z1=-A*x0
    
 

%% define outputs of the solver
outputs(1) = newOutput('u0',1,1:nu);

%% solver settings
codeoptions = getOptions('myMPC_FORCESPro');

%% generate code
generateCode(stages,params,codeoptions,outputs);


%% simulate
x1 = [-4; 1];
kmax = 30;
X = zeros(nx,kmax+1); X(:,1) = x1;
U = zeros(nu,kmax);
problem.z1 = zeros(2*nx,1);
for k = 1:kmax
    problem.minusA_times_x0 = -A*X(:,k);
    [solverout,exitflag,info] = myMPC_FORCESPro(problem);
    if( exitflag == 1 )
        U(:,k) = solverout.u0;
    else
        info
        error('Some problem in solver');
    end
    X(:,k+1) = A*X(:,k) + B*U(:,k);
end

%% plot
figure(1); clf;
subplot(2,1,1); grid on; title('states'); hold on;
plot([1 kmax], [xmax xmax]', 'r--');
ylim(1.1*[-4.1 max(xmax)]); stairs(1:kmax,X(:,1:kmax)');
subplot(2,1,2);  grid on; title('input'); hold on;
plot([1 kmax], [umax umax]', 'r--'); plot([1 kmax], [umin umin]', 'r--');
ylim(1.1*[min(umin),max(umax)]); stairs(1:kmax,U(:,1:kmax)');