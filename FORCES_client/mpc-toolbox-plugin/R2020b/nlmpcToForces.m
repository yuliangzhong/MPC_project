function [coredata, onlinedata] = nlmpcToForces(nlobj, options)
%% Interface between MPC Toolbox and FORCESPRO NLP Solver
%
% This command generates a FORCESPRO NLP Solver from "nlmpc" object for
% simulation and code generation.
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
%   [coredata, onlinedata] = nlmpcToForces(nlobj)
%
%   [coredata, onlinedata] = nlmpcToForces(nlobj, options)
%   
%   Inputs:
%       nlobj   - nonlinear MPC (an @nlmpc object)
%       options - solver options (an @nlmpcToForcesOptions object).  If not
%                 provided, default settings will be used.
%
%   Outputs:
%       coredata    - a structure containing the constant NLMPC information
%                     used by "nlmpcmoveForces".
%       onlinedata  - a structure that allows you to specify online signals
%                     such as "x", "lastMV", "ref", "MVTarget", "md", as
%                     well as weights/bounds used by "nlmpcmoveForces". 
%
%   The command also generates a MEX file from "nlmpcmoveForces" such that
%   user can directly use it for simulation.
%
%   See also nlmpcToForcesOptions, nlmpcmoveForces

%   Author(s): Rong Chen, MathWorks Inc.
%
%   Copyright 2019-2021 The MathWorks, Inc.