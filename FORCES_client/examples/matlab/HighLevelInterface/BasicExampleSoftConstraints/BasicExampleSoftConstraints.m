% Simple MPC - double integrator example with soft constraints
% on input variable u for use with FORCESPRO
% 
%  min   xN'*P*xN + sum_{i=1}^{N-1} xi'*Q*xi + ui'*R*ui
% xi,ui
%       s.t. x1 = x
%            x_i+1 = A*xi + B*ui  for i = 1...N-1
%            xmin <= xi <= xmax   for i = 1...N
%            umin <= ui <= umax   for i = 1...N
%
% and P is solution of Ricatti eqn. from LQR problem
%
% (c) Embotech AG, Zurich, Switzerland, 2015-2021.

%% system
A = [1.1 1; 0 1];
B = [1; 0.5];
[nx,nu] = size(B);
lambda = 8; % measure for penalty term

%% MPC setup
N = 10;
Q = eye(nx);
R = eye(nu);
if( exist('dlqr','file') )
    [~,P] = dlqr(A,B,Q,R);
else
    P = 10*Q;
end
umin = -0.5;     umax = 0.5;
xmin = [-5, -5]; xmax = [5, 5];

%% FORCESPRO multistage form
% assume variable ordering zi = [si; ui; xi] for i=1...N

% dimensions
model.N     = 11;   % horizon length
model.nvar  = 4;    % number of variables
model.neq   = 2;    % number of equality constraints
model.nh    = 2;    % number of inequality constraints

% objective with penalty term
model.objective = @(z)  z(2)*R*z(2) + [z(3);z(4)]'*Q*[z(3);z(4)] + lambda*z(1);
model.objectiveN = @(z) z(2)*R*z(2) + [z(3);z(4)]'*P*[z(3);z(4)] + lambda*z(1); 

% equalities
model.eq = @(z) [ A(1,:)*[z(3);z(4)] + B(1)*z(2);
                  A(2,:)*[z(3);z(4)] + B(2)*z(2)];
              
model.E = [zeros(2,2), eye(2)];

% initial state
model.xinitidx = 3:4;

% relaxed inequalities
model.ineq = @(z) [ z(2) - z(1);
                    z(2) + z(1)];
model.hu = [umax, +inf];
model.hl = [-inf, umin];


model.lb = [0,      -inf, xmin  ];
model.ub = [+inf,   +inf, xmax  ];


%% Generate FORCESPRO solver

% get options
codeoptions = getOptions('FORCESNLPsolver');
codeoptions.printlevel = 2;
codeoptions.nlp.ad_tool = 'casadi';
%codeoptions.nlp.ad_tool = 'symbolic-math-tbx';

% generate code
FORCES_NLP(model, codeoptions);


%% simulate
x1 = [-4; 2];
kmax = 30;
X = zeros(2,kmax+1); X(:,1) = x1;
U = zeros(1,kmax);
S = zeros(1,kmax);
problem.x0 = zeros(model.N*model.nvar,1); 
for k = 1:kmax
    
    problem.xinit = X(:,k);
    
    [solverout,exitflag,info] = FORCESNLPsolver(problem);

    if( exitflag == 1 )
        S(:,k) = solverout.x01(1);
        U(:,k) = solverout.x01(2);
        solvetime(k) = info.solvetime;
        iters(k) = info.it;
    else
        error('Some problem in solver');
    end
    
    %X(:,k+1) = A*X(:,k) + B*U(:,k);
    X(:,k+1) = model.eq( [S(:,k);U(:,k);X(:,k)] )';
end

%% plot
figure; clf;

subplot(2,1,1); grid on; title('states'); hold on;
plot([1 kmax], [5 5]', 'r--'); plot([1 kmax], [-5 -5]', 'r--');
ylim(1.1*[min(-5),max(5)]); stairs(1:kmax,X(:,1:kmax)');

subplot(2,1,2);  grid on; title('input'); hold on;
plot([1 kmax], [0.5 0.5]', 'r--'); plot([1 kmax], [-0.5 -0.5]', 'r--');
ylim(1.1*[min([U, -0.5]),max([U, 0.5])]); stairs(1:kmax,U(:,1:kmax)');