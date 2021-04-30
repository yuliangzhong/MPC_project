% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   Q: State weighting matrix, dimension (3,3)
%   R: Input weighting matrix, dimension (3,3)
%   T: Measured system temperatures, dimension (3,1)
%   N: MPC horizon length, dimension (1,1)
%   d: Disturbance matrix, dimension (3,N)
% OUTPUT:
%   p: Heating and cooling power, dimension (3,1)

function p = controller_mpc_5(Q,R,T,N,d)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q,R,N);
end

% evaluate control action by solving MPC problem
% [u_mpc,errorcode] = yalmip_optimizer(...,...);
% if (errorcode ~= 0)
%     warning('MPC5 infeasible');
% end
% p = ...;
end

function [param, yalmip_optimizer] = init(Q,R,N)
% get basic controller parameters
% ...
% get terminal cost
% ...
% get terminal set
% ...
% implement your MPC using Yalmip here
% nx = size(param.A,1);
% nu = size(param.B,2);
% U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
% X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
% v = sdpvar(1,1,'full');
% T0 = sdpvar(nx,1,'full');
% d = ...;
% objective = 0;
% constraints = [...];
% for k = 1:N-1
%     constraints = [constraints,...];
%     objective = objective + ...;
% end
% constraints = [constraints, ...];
% objective = objective + ...;
% ops = sdpsettings('verbose',0,'solver','quadprog');
% yalmip_optimizer = optimizer(constraints,objective,ops,...,...);
end