% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
%   N: MPC horizon length, dimension (1,1)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_mpc_2(Q, R, T, N, ~)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q, R, N);
end

% evaluate control action by solving MPC problem
% [u_mpc,errorcode] = yalmip_optimizer(...);
% if (errorcode ~= 0)
%     warning('MPC2 infeasible');
% end
% p = ...;
end

function [param, yalmip_optimizer] = init(Q, R, N)
% get basic controller parameters
% ...
% get terminal cost
% ...
% implement your MPC using Yalmip here
% nx = size(param.A,1);
% nu = size(param.B,2);
% U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
% X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
% T0 = sdpvar(nx,1,'full');
% objective = ...;
% constraints = [...];
% for k = 1:N-1
%     constraints = [constraints,...];
%     objective = objective + ...;
% end
% constraints = [constraints, ...];
% ops = sdpsettings('verbose',0,'solver','quadprog');
% yalmip_optimizer = optimizer(constraints,objective,ops,...,...);
end