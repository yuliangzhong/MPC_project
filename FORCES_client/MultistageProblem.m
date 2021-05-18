% Default initialization for a multistage problem.
%
%    STAGES = MULTISTAGEPROBLEM(N) returns an array of structs defining a 
%    multistage problem of with N stages:
%
%        min   sum_{i=1}^N z_i'*H*z_i + f_i'*z_i
%       {z_i}
% 
%     subject to:              
%                  lb_i <= z_i <= ub_i (bound constraints),      i=1,...,N
%                  Aineq_i*z_i <= b_i  (affine inequalities),    i=1,...,N
%                  z_i'*Q_{i,j}*z_i + g_{i,j}'*z_i <= r_{i,j}^2, i=1,...,N,
%                                      (quadratic inequalities)  j=1,...q
%                  D1*z1 = c1
%                  C_i*z_i + D_i+1*z_i+1 = c_i+1, (equality constraints) i=1,...,N-1.
%
%    Both H and Q are supposed to be positive definite.
%
%    The ith stage of the problem is accessed by STAGES(i), and has the
%    following fields:
%
% stages(i)
%       .dims.n - length of stage variable
%            .r - number of affine equalities (only for stages 1..N-1)
%            .l - vector in (1,n) indicating the number of lower bounds
%            .u - vector in (1,n) indicating the number of upper bounds
%            .p - number of linear inequalities g'*z <= h
%            .q - number of quadratic constraints z'*M*z + g'*z <= r
%            .h - number of general inequalities (possibly nonlinear, non-convex)
%
%       .cost.H - Hessian of cost
%            .f - linear part of cost
%
%         .eq.C - equality constraint matrix Ci
%            .D - equality constraint matrix Di
%            .c - vector containing the RHS of equality constraints ci
%
%    .ineq.b.lb - vector of lower bounds of stage variables
%           .lbidx - vector indicating to which stage variables the
%                    lower bounds belong to
%           .ub - vector of upper bounds of stage variables
%           .ubidx - vector indicating to which stage variables the
%                    upper bounds belong to
%            
%     .ineq.p.A - matrix defining the Jacobian of the polytopic inequalities A*z <= b
%           p.b - vector, RHS of polytopic inequalities
%  
%     .ineq.q.Q - cell array containing the Hessians of quadratic
%                 inequalities z'*Q*z + g'*z <= r
%            .l - linear part of quadratic inequalities ( N x q matrix )
%            .r - RHS of quadratic inequalities ( q x 1 vector )
%
%     .ineq.h.hl - lower bound for inequality hl <= h(z)
%            .hlidx - indicates on which indices of h(z) the lower bounds act
%            .hu - upper bound for inequality h(z) <= hu
%            .huidx - indicates on which indices of h(z) the upper bounds act
%
% See also FORCES_NLP NEWPARAM NEWOUTPUT
%
%
% This file is part of the FORCESPRO client software for Matlab.
% (c) embotech AG, 2013-2021, Zurich, Switzerland. All rights reserved.
