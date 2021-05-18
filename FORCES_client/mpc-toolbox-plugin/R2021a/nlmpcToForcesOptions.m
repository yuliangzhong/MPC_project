    %% Interface between MPC Toolbox and FORCESPRO NLP Solver
    %
    %  This command generates options used by the "nlmpcToForces" command.
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
    %   options = nlmpcToForcesOptions();
    %
    %   You can adjust solver settings before using them with "nlmpcToForces".
    %
    % See also nlmpcToForces, nlmpcmoveForces.

%   Author(s): Rong Chen, MathWorks Inc.
%
%   Copyright 2019-2021 The MathWorks, Inc.
