function [output,exitflag,SolverInfo] = forcesSparseSimulateMex(data,onlinedata,xk,yrefM,urefM,md,trueLastMV)
%% Interface between lineat MPC and FORCESPRO QP Solver (internal)
%
%   The method calls FORCESPRO QP solver MEX file when "mpcmoveForces" is used
%   in simulation, not code generation.

%   Author(s): Rong Chen, MathWorks Inc.
%
%   Copyright 2019-2021 The MathWorks, Inc.
