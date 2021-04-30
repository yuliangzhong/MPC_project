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

function p = controller_mpc_6(Q,R,T,N,~)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q,R,T,N);
end

% evaluate control action by solving MPC problem
% [u_mpc,errorcode] = yalmip_optimizer(...);
% if (errorcode ~= 0)
%     warning('MPC6 infeasible');
% end
% p = ...;

% observer update
% ...

% set point update
% ...
end

function [param, yalmip_optimizer] = init(Q,R,T,N)
% get basic controller parameters
% ...
% get terminal cost
% ...
% get terminal set
% ...
% design disturbance observer
% ...
% init state and disturbance estimate variables
% ...
% implement your MPC using Yalmip here
% nx = size(param.A,1);
% nu = size(param.B,2);
% U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
% X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
% T0 = sdpvar(nx,1,'full');
% T_sp = ...;
% p_sp = ...;
% constraints = [...];
% objective = ...;
% for k = 1:N-1
%     constraints = [constraints, ...];
%     objective = objective + ...;
% end
% constraints = [constraints, ...];
% objective = objective + ...;
% ops = sdpsettings('verbose',0,'solver','quadprog');
% yalmip_optimizer = optimizer(constraints,objective,ops,...,...);
end