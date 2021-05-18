% Implicit Runge-Kutta integrator of order 4.
%
% [XNEXT, JAC] = IRK4( X, U, XIDX, UDIX, F, H ) implements implicit RK4 integrator
% equations to discretize (potentially nonlinear) continuous dynamics with 
% step size H. Note that H is the absolute time spent between X and XNEXT.
% F is a function handle to a function that takes the arguments (X, U)
% and returns dx/dt. XIDX and UIDX are the indices for the states and
% inputs used to construct the Jacobian JAC.
%
% [XNEXT, JAC] = IRK4( X, U, XIDX, UDIX, F, H, P ) as above, but 
% parameters P are passed to the function handle F to support parameters 
% in the dynamics, e.g. changing inertia etc.
%
% [XNEXT, JAC] = IRK4( X, U, XIDX, UDIX, F, H, P, M ) performs M-steps for 
% the integration, i.e. it places M-1 intermediate points between X and XNEXT.
% Use this to integrate systems more accurately without increasing the number
% of optimization variables in the NLP solver. If no parameters are present,
% use P = [].
%
% [XNEXT, JAC] = IRK4( X, U, XIDX, UDIX, F, H, P, M, OPTIONS ) performs M-steps
% for the integration, i.e. it places M-1 intermediate points between X and XNEXT
% and takes further codeoptions into accout. If no parameters are present,
% use P = [].
%
% See also ForwardEuler BackwardEuler RK2 RK3 RK4 IRK2 FORCES_NLP
%
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.
