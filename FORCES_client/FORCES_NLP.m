% Interface to Embotech's FORCESPRO nonlinear programming (NLP) solver.
%
% [STAGES, OPTIONS] = FORCES_NLP(MODEL, OPTIONS) generates an efficient 
% solver for the following possibly nonlinear problem:
%
%   minimize    sum_{i=1}^N f(x,p)
%   subject to  x(xinitidx) == xinit                 (initial conditions)
%               E*x_{i+1} == c(x_i,p), for i = 1...N-1 (nonlinear equalities)
%               x(xfinalidx) == xfinal               (terminal conditions)
%               lb <= x_i <= ub,    for i = 1...N    (bounds on x)
%               x_i,j integer within lb <= x_i <= ub (integer variables)
%               hl <= h(z_i) <= hu  for i = 1...N    (nonlinear inequalities)
%
% The problem above is defined by the structure MODEL as follows:
% 
%    MODEL.N         - number of steps N
%         .nvar      - number of variables in each stage
%         .neq       - number of equalities in each stage (size of c(x))
%         .nh        - number of inequalities in each stage (size of h(x))
%         .npar      - number of parameters (length of p above) in each stage
%
%         .xinitidx  - indices on which the initial conditions are defined
%         .xfinalidx - indices on which the terminal conditions are defined
%         .E         - matrix multiplying the LHS of equalities
%
%         .lb        - lower bounds on x. If unconstrained, use -inf
%         .ub        - upper bounds on x. If unconstrained, use +inf
%         .hl        - lower bounds on h(x). If unconstrained, use -inf
%         .hu        - upper bounds on h(x). If unconstrained, use +inf
%
% Notes: - MODEL.nvar, .neq and .nh can be given per stage via cell arrays.
% ======   Otherwise it is assumed that they are the same for each stage.
%        - MODEL.npar allows for parameterizing the functions and is
%          optional (by default set to zero).
%        - MODEL.lb and .ub can be empty; in this case provide 
%          MODEL.lbidx and MODEL.ubidx to indicate on which variables lower
%          and upper bounds are present. The numerical values will then be
%          expected at runtime.
%
% The function may return:
%
%    STAGES  - an array of structs defining a multistage problem
%    OPTIONS - options as passed by the user but with automatic adjustments
%              for enhanced performance
%
% The generated code will have automatically the following parameters,
% which need to be supplied before calling the solver:
%
%    PROBLEM.x0 - initial guess, supplied as one column vector of length nvar*N.
% 
% If initial or final conditions are present, the following parameters have
% to be appropriately supplied:
%
%    PROBLEM.xinit  - initial condition
%    PROBLEM.xfinal - final condition
%
% If lower and/or upper bounds are parametric (see notes above):
%
%    PROBLEM.lb01 - lower bound for stage 1
%    PROBLEM.ub01 - upper bounds for stage 1
%        ...      - similarly for all stages
%
% (the number of leading zeros is adjusted to the log10 of the number of 
%  stages automatically)
%
% The default output names are:
%
%      OUTPUTS.x01
%      OUTPUTS.x02 etc. 
%
% You can type "help <solvername>" after calling FORCES_NLP for more detailed
% documentation of expected in- and outputs for the particular solver as 
% well as exitflags.
%
%
% [STAGES, OPTIONS] = FORCES_NLP(MODEL, OPTIONS, OUTPUTS) as above, but 
% with user defined outputs.
%
% [STAGES, OPTIONS, FORMULATION] = FORCES_NLP(MODEL, OPTIONS, OUTPUTS) as 
% above, but returning a struct FORMULATION containing a dump of the 
% problem formulation. This requires to set OPTIONS.dump_formulation = 1.
%
% See also MULTISTAGEPROBLEM GETOPTIONS NEWOUTPUT FORCESDUMPFORMULATION
%
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.
