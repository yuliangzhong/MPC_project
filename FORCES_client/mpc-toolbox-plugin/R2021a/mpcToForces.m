function [coredata, statedata, onlinedata, stages] = mpcToForces(mpcobj, options)
%% Interface between MPC Toolbox and FORCESPRO QP Solver
%
%  This function generates customized FORCESPRO QP solver from @mpc object.
%
%   [coredata, statedata, onlinedata] = mpcToForces(mpcobj, options) 
%
%   Inputs:
%
%       mpcobj:     an MPC controller object created by the "mpc" command
%                   and designed with the MPC Toolbox from MathWorks
%
%       options:    specify solver generation settings created by the
%                   "mpcToForcesOptions" command.  You can choose either a
%                   "sparse" QP formulation or a "dense" QP formulation.  
%
%   Outputs:
%
%       coredata:   a structure of MPC settings (constant)
%
%       statedata:  a structure of plant states, disturbance model states, noise model states and the last optimal MV
%
%       onlinedata: a structure of signals (references, measured output, external MV, online constraints, online weights, etc.)
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
%       Signal previewing on output references and measured disturbances
%       Signal previewing on MV references (only available using sparse QP)
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
% See also mpcmoveForces, mpcToForcesOptions.

%   Author(s): Rong Chen, MathWorks Inc.
%
%   Copyright 2019-2021 The MathWorks, Inc.
