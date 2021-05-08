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

function p = controller_mpc_1(Q, R, T, N, ~)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q, R, N);
end

%% evaluate control action by solving MPC problem
[u_mpc,errorcode] = yalmip_optimizer(T - param.T_sp);
if (errorcode ~= 0)
    warning('MPC1 infeasible');
end
p = u_mpc + param.p_sp;
end

function [param, yalmip_optimizer] = init(Q, R, N)
% get basic controller parameters
param = compute_controller_base_parameters; 
% get terminal cost
[K, S, e] = dlqr(param.A, param.B, Q, R);

% implement your MPC using Yalmip here
nx = size(param.A,1);
nu = size(param.B,2);
U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
X = sdpvar(repmat(nx,1,N),ones(1,N),'full');

Ax=[eye(3);-eye(3)];
bx=[param.Xcons(:,2);-param.Xcons(:,1)];
Au=[eye(3);-eye(3)];
bu=[param.Ucons(:,2);-param.Ucons(:,1)];

objective = 0;
constraints = [];
for k = 1:N-1
    constraints = [constraints, X{k+1} == param.A*X{k} + param.B*U{k}];

    constraints = [constraints, Ax*X{k+1} <= bx];
    constraints = [constraints, Au*U{k} <= bu];
    objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k};
end
objective = objective + X{end}'*S*X{end};
x0 = sdpvar(3,1);
constraints = [constraints, X{1} == x0];
ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_optimizer = optimizer(constraints,objective,ops,x0,U{1});
end
