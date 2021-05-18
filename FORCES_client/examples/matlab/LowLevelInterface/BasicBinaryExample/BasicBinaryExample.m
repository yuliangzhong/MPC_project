% Binary Simple MPC - double integrator example with binary inputs for use
% with FORCESPRO.
%
% In this example, we have a system with 2 states (instable double
% integrator) and 2 inputs. Both inputs are discrete and can either be 
% umin or umax.
%
% Since FORCESPRO supports only binaries at the moment, we will formulate
% the problem in terms of variables delta, where
%
%       delta = 0  <==> u = umin   and   delta = 1  <==> u = umax
%
% This can be formulated by the equality constraint 
%
%       u = umin + diag(umax-umin)*delta
%
% To keep the number of variables at a minimum, we will directly insert 
% this equation into the dynamics:
%
%    x+ = Ax + Bu
%       = Ax + B*umin + B*diag(umax-umin)*delta
%       = Ax + Btilde*delta + Bconst,
%
% where Btilde = B*diag(umax-umin) and Bconst = B*umin.
%
% Similarly, for the cost function, 
% 
%   u'*R*u = (umin+diag(umax-umin)*delta)'*R*(umin+diag(umax-umin)*delta)
%          = delta'*diag(umax-umin)*R*diag(umax-umin)*delta + 2*umin'*diag(umax-umin)*R*delta + const.
%          = delta'*Rtilde*delta + ftilde'*delta + const.
%
% where Rtilde = diag(umax-umin)*R*diag(umax-umin) and
%       ftilde = 2*R*diag(umax-umin)*umin;
%
% Now we can formulate the finite horizon optimal control problem as follows:
%
%  min   xN'*P*xN + sum_{i=1}^{N-1} xi'*Q*xi + ui'*R*ui
% xi,ui
%       s.t. x1 = x
%            x_i+1 = A*xi + Btilde*delta_i + Bconst  for i = 1...N-1
%            xmin <= xi <= xmax   for i = 1...N
%            0 <= delta_i <= 1    for i = 1...N
%            delta \in {0, 1}
%
% and P is solution of Ricatti eqn. from LQR problem
%
% (c) Embotech AG, Zurich, Switzerland, 2015.

clear; close all; clc;

%% plot settings
doplot.trajectories = true;
doplot.timings = true;
doplot.exitflags = true;

%% system
A = [1.1 1;
    0 1];
B = [1 0;
    0.5 -0.5];
[nx,nu] = size(B);
binaryIndices = [1 2]; % these indices are binary

%% MPC setup
N = 5;                % horizon length
Q = 10*eye(nx);        % state cost 
R = eye(nu);           % input cost
if( exist('dlqr','file') )
    [~,P] = dlqr(A,B,Q,R); % terminal cost
else
    P = 5*Q;
end
umin = [-0.5; -0.5]; umax = -umin; % constraints on inputs
xmin = [-5; -5]; xmax = -xmin; % constraints on states

% precompute matrices for substitution of u = umin + diag(umax-umin)*delta
Btilde = B*diag(umax-umin);
Bconst = B*umin;
Rtilde = diag(umax-umin)*R*diag(umax-umin);
ftilde = 2*R*diag(umax-umin)*umin;


%% FORCESPRO multistage form
% assume variable ordering zi = [ui; xi+1] for i=1...N-1

stages = MultistageProblem(N);
for i = 1:N
    
    % dimension
    stages(i).dims.n = nx+nu;  % number of stage variables
    stages(i).dims.r = nx;     % number of equality constraints
    stages(i).dims.l = nx;     % number of lower bounds
    stages(i).dims.u = nx;     % number of upper bounds
    stages(i).bidx = binaryIndices;    % index of binary variable
    
    % cost
    if( i == N )
        stages(i).cost.H = blkdiag(Rtilde,P);
    else
        stages(i).cost.H = blkdiag(Rtilde,Q);
    end
    stages(i).cost.f = [ftilde; zeros(nx,1)];
    
    % lower bounds
    stages(i).ineq.b.lbidx = (nu+1):(nu+nx); % lower bound on states
    stages(i).ineq.b.lb = xmin;              % upper bound values 

    % upper bounds
    stages(i).ineq.b.ubidx = (nu+1):(nu+nx); % upper bound for this stage variable
    stages(i).ineq.b.ub = xmax;              % upper bound for this stage variable
    
    % equality constraints
    if( i < N )
        stages(i).eq.C = [zeros(nx,nu), A];
    end
    if( i>1 )
        stages(i).eq.c = -Bconst;
    end
    stages(i).eq.D = [Btilde, -eye(nx)];
    
end
params(1) = newParam('minusA_times_x0',1,'eq.c'); % RHS of first eq. constr. is a parameter: z1=-A*x0


%% define outputs of the solver
outputs(1) = newOutput('delta01', 1, 1:nu);

%% solver settings
codeoptions = getOptions('binary_simplempc_solver');
codeoptions.optlevel = 3;
codeoptions.printlevel = 2;

% miqp code options
codeoptions.mip.branchon = 'mostAmbiguous';  % options: 'mostAmbiguous', 'leastAmbiguous'
codeoptions.mip.stageinorder = 1;            % 1: on (earlier stages have precedence in branching)
codeoptions.mip.explore = 'bestFirst';       % options: 'best-first', 'depth-first', 'breadth-first'
codeoptions.mip.inttol = 1e-5;               % anything below this threshold is considered to be binary (either 0 or 1)
codeoptions.mip.timeout = 0.05;              % timeout in seconds
codeoptions.mip.mipgap = 0.00;               % relative suboptimality gap
codeoptions.mip.queuesize = 1000;

%% generate code
generateCode(stages,params,codeoptions,outputs);


%% simulate
x1 = [-5; 3]; % initial state
kmax = 30; % simulation steps
X = zeros(nx,kmax+1); X(:,1) = x1;
U = zeros(nu,kmax);

clcost = 0;
prepadzeros = num2str(ceil(log10(N)));

% set lower/upper bounds for stages with binary variables
for i = 1:N
    problem.(sprintf(['lb%0',prepadzeros,'d'],i)) = [0; 0; xmin];
    problem.(sprintf(['ub%0',prepadzeros,'d'],i)) = [1; 1; xmax];
end

for k = 1:kmax
    
    % set eq.c (initial state)
    problem.minusA_times_x0 = -A*X(:,k) - Bconst;
    
    % solve problem
    [optsol,exitflag(k),info] = binary_simplempc_solver(problem);
    if( exitflag(k) == -7 )
        error('Some problem in solver');
    end
    
    D(:,k) = optsol.delta01;
    U(:,k) = umin + diag(umax-umin)*D(:,k);
    
    % dynamics
    X(:,k+1) = A*X(:,k) + B*U(:,k);
    
    % collect statistics 
    solvetime(k) = info.solvetime;
    qpsolves(k) = info.it;
    it2opt(k) = info.it2opt;
    clcost = clcost + X(:,k+1)'*Q*X(:,k+1) + U(:,k)'*R*U(:,k);
    
    fprintf('Time step %d completed\n', k);
    
end

fprintf('Closed loop cost: %6.4e\n', clcost);


%% plot
if( doplot.trajectories )
    figure(1); clf;
    subplot(3,1,1); grid on; title('states'); hold on;
    plot([1 kmax], [xmax xmax]', 'r--'); plot([1 kmax], [xmin xmin]', 'r--');
    ylim(1.1*[min(xmin),max(xmax)]); stairs(1:kmax,X(:,1:kmax)');
    
    subplot(3,1,2);  grid on; title('true inputs u'); hold on;
    plot([1 kmax], [umax umax]', 'r--'); plot([1 kmax], [umin umin]', 'r--');
    ylim(1.1*[min(umin),max(umax)]); stairs(1:kmax,U(:,1:kmax)');
    
    subplot(3,1,3);  grid on; title('binary variables \delta'); hold on;
    plot([1 kmax], [1 1]', 'r--'); plot([1 kmax], [0 0]', 'r--');
    ylim([-0.1,1.1]); stairs(1:kmax,D(:,1:kmax)');
    
    print('-djpeg', 'binary_simplempc_traj.jpg');
end

%% plot timings
if( doplot.timings )
    figure(2); clf;
    subplot(2,1,1); bar([qpsolves; it2opt]'); grid on; ylim([0 1.5*max(qpsolves)]);
    title('number of QPs solved'); xlabel('time step');
    legend('# of QPs for optimality certificate', '# of QPs until finding opt. solution','Orientation','Horizontal','Location','Best');
    subplot(2,1,2); bar(solvetime.*1000); ylabel('milliseconds'); grid on; title('solve time'); xlabel('time step');
%     mytitle = codeoptions.mip.branchon;
%     if( codeoptions.mip.stageinorder )
%         mytitle = [mytitle, ' / ', 'SIO'];
%     end
%     mytitle = [mytitle, ' / ', codeoptions.mip.explore];
%     suptitle(mytitle)    
    
    print('-djpeg', 'binary_simplempc_timings.jpg');
end

%% plot exitflags
if( doplot.exitflags )
    figure(3); clf;
    histogram = hist(exitflag,[-2,-1,0,1,2]-0.5);
    h=bar([-2,-1,0,1,2],histogram/length(exitflag)*100);
    xlim([-2.5,2.5]); ylim([0 100]); grid on;
    set(gca,'Xtick',[-2,-1,0,1,2]);
    set(gca,'XTickLabel',{'no_mem', 'infeasible', 'timeout', 'optimal', 'suboptimal'});
    ylabel('% of problems');
    title('Solve status returned by FORCESPRO');
    print('-djpeg', 'binary_simplempc_exitflags.jpg');
end