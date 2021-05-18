function [mv, onlinedata, info] = nlmpcmoveForces(coredata,x,lastMV,onlinedata)
%% Interface between MPC Toolbox and FORCESPRO NLP Solver
%
% This function computes nonlinear MPC moves using FORCESPRO NLP solver.
%   It supports C/C++ code generation with MATLAB Coder.
%
%   The following Nonlinear MPC features are supported:  
%   * Prediction model in continuous time or discrete time
%   * Block control moves (i.e. control horizon can be a vector)
%   * Measured disturbances in the prediction model
%   * Unmeasured disturbances in the prediction model
%   * Output/MV/MVRate/ECR weights (uniform or time-varying)
%   * Output/MV/MVRate constraints (uniform or time-varying, hard or soft)
%   * Previewing of output references and measured disturbances
%   * Online update of Output/MV/MVRate/State bounds
%   * Online update of Output/MV/MVRate/ECR weights
%   * Single parameter used by the prediction model (must be a column vector)
%   * Double precision
%
%   The following features are not supported:
%   * Custom cost function
%   * Custom inequality and equality constraint functions
%   * Custom Jacobian functions (they are auto-generated from auto-diff tool CasADi) 
%   * Single precision
%
%   Syntax:
%
%   [mv, onlinedata, info] = nlmpcmoveForces(coredata, x, lastMV, onlinedata)
%
%   Inputs:
%
%       coredata:   a structure containing NLMPC settings.  It is generated
%                   by the "nlmpcToForces" command and used as a constant.  
%
%              x:   a nx-by-1 column vector, representing the current
%                   prediction model states.
%
%         lastMV:   a nmv-by-1 column vector, representing the control
%                   action applied to plant at the previous control
%                   interval. 
%
%     onlinedata:   a structure containing information such as references,
%                   measured disturbances, online constraints and weights, etc. 
%
%   Outputs:
%
%             mv:   Optimal control moves
%
%     onlinedata:   a structure prepared by "nlmpcmoveForces" for the next
%                   control interval.  The "X0" and "MV0" fields are
%                   populated as the initial guess to be used at the next
%                   control interval. 
%
%          info:    a structure containing extra optimization information
%                  MVopt: a p+1-by-nmv matrix for optimal MV trajectory from time k to k+p
%                   Xopt: a p+1-by-nx matrix for optimal state trajectory from time k to k+p
%                   Yopt: a p+1-by-ny matrix for optimal output trajectory from time k to k+p
%               ExitFlag: 1: optimum found
%                         0: maximum iteration reached (discard returned mv)
%                         negative value: failed to find a solution (discard returned mv)
%             Iterations: number of iterations used by the NLP solver
%                   Cost: optimal cost
%       EqualityResidual: residual of equality constraints
%     InequalityResidual: residual of inequality constraints (only available with interior-point solver)
%              SolveTime: total execution time by NLP solver (in seconds)
%            FcnEvalTime: total function evaluation time by the NLP solver
%                 QPTime: total QP solver time (only available with SQP_LTI solver)
%
% See also nlmpcToForces, nlmpcToForcesOptions.

%   Author(s): Rong Chen, MathWorks Inc.
%
%   Copyright 2019-2021 The MathWorks, Inc.
