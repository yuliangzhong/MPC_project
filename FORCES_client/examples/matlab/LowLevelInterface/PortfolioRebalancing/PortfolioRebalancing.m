% FORCESPRO Portfolio rebalancing optimization example.
%
% This examples solves the single period portfolio rebalancing problem
%
%             maximize  mu'*x - sum_{i=1}^n c(i)*|x(i)-x0(i)|
%           subject to  xmin <= x <= xmax
%                       x'*Sigma*x <= r
%
% where x is a vector of n assets to be traded, x0 is the current position,
% mu is a vector of expected return, c is a vector of transaction costs,
% i.e. for moving asset i from x0(i) to x(i) one is to pay amount c(i) times
% the absolute value of the difference. The vectors xmin and xmax are lower
% and upper bounds on the positions, the positive definite matrix Sigma is 
% the risk covariance matrix, and r represents an upper bound on the 
% variance of the portfolio.
%
% To solve this problem with FORCES, we reformulate it as
%
%            minimize  -mu'*x + c'*t
%          subject to  xmin <= x <= xmax
%                      x'*Sigma*x <= r
%                      -t <= x-x0 <= t
%                      t >= 0
%
% which requires the introduction of n new variables t. So the total number
% of variables is 2*n, sorted as z = [x; t]. The inequality constraints
%
%                    -t <= x-x0 <= t  with  t >= 0
%
% are used to express the absolute value on x-x0, which will be equal to t
% due to the minimization in case c is non-zero. Therefore the transaction
% costs in the objective value can be expressed as c'*t.
%
% IMPLEMENTATION:
%
% 1) We implement the affine inequalities -t <= x-x0 <= t as follows:
%
%                   -t-x <= -x0    and    x-t <= x0
%
%    With the variable ordering z = [x; t] we get therefore the inequality
%
%                 [-I -I; I -I]*[x; t] <= [-x0; x0]
%
%    which we write as A*z <= b, with b being a parameter to the problem.
%
% 2) Cost function: FORCESPRO supports the quadratic cost 0.5*z'*H*z + f'z, so
%    in this example H=0 and f = [-mu; c], with f being a parameter to the
%    problem.
%
% 3) Lower/upper bounds: we have [xmin; 0] <= [x; t] and x <= xmax.
%    Lower/upper bounds are parameters to the problem.
%
% (c) Embotech AG, Zurich, Switzerland, 2013-2021. All rights reserved.

clear all; clc; close all;

%% dimensions
n = 15; % number of assets

%% set up FORCESPRO formulation
stages = MultistageProblem(1); % single-period portfolio, indicated by 1

% dimensions
stages(1).dims.n = 2*n; % number of variables
stages(1).dims.r = 0;   % number of equality constraints
stages(1).dims.l = 2*n; % number of lower bounds
stages(1).dims.u = n;   % number of upper bounds
stages(1).dims.p = 2*n; % number of affine inequalities
stages(1).dims.q = 1;   % number of quadratic constraints

% cost: 0.5*z'*H*z + f'*z
stages(1).cost.H = zeros(stages(1).dims.n);
params(1) = newParam('minusMu_c', 1 , 'cost.f');

% inequalities - lower bounds lb <= z(lbidx)
stages(1).ineq.b.lbidx = 1:2*n;
params(end+1) = newParam('xmin_zeros', 1, 'ineq.b.lb');

% inequalities - upper bounds z(ubidx) <= ub
stages(1).ineq.b.ubidx = 1:n;
params(end+1) = newParam('xmax', 1, 'ineq.b.ub');

% inequalities - polytopic constraints A*z <= b
I = eye(n);
stages(1).ineq.p.A = [-I, -I; I -I];
params(end+1) = newParam('minusX0_X0', 1, 'ineq.p.b');

% inequalities - quadratic constraints z(qidx)'*Q*z(qidx) + l'*z(qidx) <= r
stages(1).ineq.q.idx = { 1:n };
stages(1).ineq.q.Q = {};
stages(1).ineq.q.l = { zeros(n,1) };
params(end+1) = newParam('Sigma', 1, 'ineq.q.Q', 1);
params(end+1) = newParam('r', 1, 'ineq.q.r');

%% generate solver
codeoptions = getOptions('rebalancer'); % this is the function name that will be generated

codeoptions.accuracy.ineq = 1e-7;  % infinity norm of residual for inequalities
codeoptions.accuracy.eq = 1e-7;    % infinity norm of residual for equalities
codeoptions.accuracy.mu = 1e-7;    % absolute duality gap
codeoptions.accuracy.rdgap = 1e-7; % relative duality gap := (pobj-dobj)/pobj
codeoptions.printlevel = 0;

% define which outputs you want - we get here both x and t
output(1) = newOutput('x', 1, 1:n);
output(2) = newOutput('t', 1, n+(1:n));

% call server to generate code
generateCode(stages, params, codeoptions, output);


%% Test portfolio optimizer
number_of_problems = 10; % number of random instances of the problem

% see whether there are other solvers on the system we can run against
solvewith.cvx_sedumi = exist('cvx_begin','file') && exist('cvx_end','file') && exist('sedumi','file');
solvewith.cvx_ecos = solvewith.cvx_sedumi && exist('ecos','file');
solvewith.yalmip_sedumi = exist('sdpvar','file') && exist('solvesdp','file');
solvewith.yalmip_cplex = solvewith.yalmip_sedumi && exist('cplexqcp','file');

for i = 1:number_of_problems
    
    %% generate random instance of problem
    mu = rand(n,1);         % expected return
    c = rand(n,1);          % transaction costs
    xmin = -ones(n,1);      % minimum positions
    xmax = ones(n,1);       % maximum positions
    x0 = rand(n,1);         % current portfolio
    Sigma = rand(n);  
    Sigma = Sigma'*Sigma;             
    Sigma = Sigma./norm(Sigma); % risk covariance matrix    
    r = rand(1);            % tolerated risk variance
    
    %% run FORCESPRO generated solver!
    % set up FORCESPRO problem - fill in parameters
    problem.minusMu_c = [-mu; c];
    problem.Sigma = Sigma;
    problem.r = r;
    problem.xmin_zeros = [xmin;  zeros(n,1)];
    problem.xmax = xmax;
    problem.minusX0_X0 = [-x0; x0];
    
    % type "help portfolioRebalancer" to get more info on how to use it
    [output, exitflag, info] = rebalancer(problem);
    
    % always check exitflag
    if( exitflag ~= 1)
        warning('Problem in solver, exiting');
        break;
    end
    
    % record solve time in seconds
    solvetime_FORCES(i) = info.solvetime;
    
    
    %% Solve with CVX, using Sedumi as a solver
    if( solvewith.cvx_sedumi )
        cvx_solver sedumi
        cvx_tic
        cvx_begin quiet
        variable x(n,1)
        maximize (mu'*x - c'*abs(x-x0))
        subject to
        xmin <= x <= xmax;
        quad_form(x, Sigma) <= r;
        cvx_end
        cvx_time_sedumi = cvx_toc;
        assert(~isempty(findstr(cvx_status,'Solved')),'Problem in CVX/SEDUMI');
        solvetime_cvx_sedumi(i) = cvx_time_sedumi(end);
    end
    
    
    %% Solve with CVX, using ECOS as solver
    if( solvewith.cvx_ecos )
        cvx_solver ecos
        cvx_tic
        cvx_begin quiet
        variable x(n,1)
        maximize (mu'*x - c'*abs(x-x0))
        subject to
        xmin <= x <= xmax;
        quad_form(x, Sigma) <= r;
        cvx_end
        cvx_time_ecos = cvx_toc;
        assert(~isempty(findstr(cvx_status,'Solved')),'Problem in CVX/ECOS');
        solvetime_cvx_ecos(i) = cvx_time_ecos(end);
    end
    
    
    %% run against cplex using YALMIP, with Sedumi as solver
    if( solvewith.yalmip_sedumi )
        x = sdpvar(n,1);
        obj = (-mu'*x + c'*abs(x-x0));
        con = [xmin <= x, x <= xmax, x'*Sigma*x <= r];
        opts = sdpsettings('solver','sedumi', 'verbose', 0);
        yalmipinfo=solvesdp(con,obj,opts);
        assert(yalmipinfo.problem == 0,'Problem in YALMIP solve');
        solvetime_yalmip_sedumi(i) = yalmipinfo.solvertime;
        assert(norm(double(obj)-info.pobj)<=1E-6,'Portfolio allocation error in objective too large w.r.t. YALMIP/SEDUMI: %6.4e\n', norm(double(obj)-info.pobj));
    end
    
    %% run against cplex using YALMIP, with CPLEX as solver
    if( solvewith.yalmip_cplex )
        x = sdpvar(n,1);
        obj = (-mu'*x + c'*abs(x-x0));
        con = [xmin <= x, x <= xmax, x'*Sigma*x <= r];
        opts = sdpsettings('solver','cplex', 'verbose', 0);
        opts.cplex.threads = 1;
        yalmipinfo=solvesdp(con,obj,opts);
        assert(yalmipinfo.problem == 0,'Problem in YALMIP solve');
        solvetime_yalmip_cplex(i) = yalmipinfo.solvertime;
        assert(norm(double(obj)-info.pobj)<=1E-6,'Portfolio allocation error in objective too large w.r.t. YALMIP/CPLEX: %6.4e\n', norm(double(obj)-info.pobj));
    end
    
end


%% Report timings
data = solvetime_FORCES; lgd = {'FORCES'};
if( solvewith.yalmip_cplex ), data = [data; solvetime_yalmip_cplex]; lgd = [lgd, {'YALMIP/CPLEX'}]; end
if( solvewith.yalmip_sedumi ), data = [data; solvetime_yalmip_sedumi]; lgd = [lgd, {'YALMIP/SEDUMI'}]; end
if( solvewith.cvx_sedumi ), data = [data; solvetime_cvx_sedumi]; lgd = [lgd, {'CVX/SEDUMI'}]; end
if( solvewith.cvx_ecos ), data = [data; solvetime_cvx_ecos];  lgd = [lgd, {'CVX/ECOS'}]; end

figure(1); clf; bar(data'*1000); legend(lgd); 
xlabel('problem instance'); ylabel('solve time [milliseconds]');
title('Solve times for portfolio rebalancing');