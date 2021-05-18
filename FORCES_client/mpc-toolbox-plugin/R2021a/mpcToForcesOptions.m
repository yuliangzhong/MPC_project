function options = mpcToForcesOptions(qp)
%% Interface between MPC Toolbox and FORCESPRO QP Solver
%
%  This function allows you to choose Dense QP or Sparse QP formulation for
%  custom solver and specify corresponding MPC features and FORCESPRO QP
%  solver settings.
%
%  options = mpcToForcesOptions() or options = mpcToForcesOptions('sparse')
%  generate a Sparse QP problem where manipulated variables (MVs), outputs
%  (OVs) and states are decision variables.  Use Sparse QP for large MPC
%  problem with long horizons and large amount of constraints.  Sparse QP
%  also allows you to use long horizons even if the plant is unstable.
%
%  options = mpcToForcesOptions('dense') generates a Dense QP problem
%  where only manipulated variables (MVs) are used as decision variables.
%  Use Dense QP for small MPC problems with short horizons and small amount
%  of constraints. 
%
% See also mpcToForces, mpcmoveForces.

%   Author(s): Rong Chen, MathWorks Inc.
%
%   Copyright 2019-2021 The MathWorks, Inc.
