%% Interface between MPC Toolbox and FORCESPRO QP Solver
%
%  This function computes optimal control moves using FORCESPRO QP solver.
%
%   [mv, statedata, info] = mpcmoveForces(coredata, statedata, onlinedata)
%
%   Inputs:
%
%       coredata:   a structure containing MPC settings.  It is generated
%                   by "mpcToForces" command and used as a constant here. 
%
%      statedata:   a structure containing plant states, disturbance model
%                   states, noise model states and the last optimal MV.
%
%                   When custom estimation is used, user needs to update
%                   the "Plant", "Disturbance" and "Noise" fields with
%                   estimated states x[k|k] but leave "LastMove" untouched.  
%
%                   When built-in estimation is used, no fields should be
%                   changed before passing it back to "mpcmoveForces" in
%                   the next control interval.
%
%     onlinedata:   a structure containing signals such as references,
%                   measured outputs, external MV, online constraints,
%                   online weights, etc.
%
%   Outputs:
%
%             mv:   Optimal control move for the current time
%
%      statedata:   a structure prepared by "mpcmoveForces" for the next
%                   control interval. 
%
%                   When custom estimation is used, only the "LastMove"
%                   field is updated with the new optimal "mv".
%
%                   When built-in estimation is used, all the fields are
%                   updated.  Do not change them before passing it back.
%
%          info:    a structure containing extra optimization information
%                   Uopt: a p-by-nmv matrix for optimal MV trajectory from time k to k+p-1
%                   Yopt: a p-by-ny matrix for optimal OV trajectory from time k+1 to k+p
%                   Xopt: a p-by-nx matrix for optimal state trajectory from time k+1 to k+p
%                  Slack: a p-by-1 vector of slack variables
%               ExitFlag: 1: optimum found, 
%                         0: maximum iteration reached
%                         negative value: failed to find a solution
%             Iterations: number of iterations 
%                   Cost: optimal cost
%
%  The following LTI MPC features are supported when using the FORCESPRO QP
%  solver:  
%
%       Plant model (continuous time or discrete time)
%       Block control moves 
%       Measured disturbances (MD)
%       Unmeasured disturbances (UD)
%       Output/MV/MVRate/ECR weights (uniform or time-varying)
%       Output/MV/MVRate bounds (uniform or time-varying)
%       Soft constraints           
%       Signal previewing on references and measured disturbances
%       Scale factors
%       Nominal values 
%       Online update of constraints and weights.
%       Build-in and custom state estimation
%
%  Unsupported MPC features include:
%       Adaptive MPC and LTV MPC where plant model changes online
%       Single precision
%       Mixed input and output constraints
%       Suboptimal solution when using the sparse QP formulation
%       Alternative cost function with off-diagonal weights
%
% See also mpcToForces, mpcToForcesOptions.

%   Author(s): Rong Chen, MathWorks Inc.
%
%   Copyright 2019-2021 The MathWorks, Inc.
