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

function p = controller_mpc_4(Q, R, T, N, ~)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q, R, N);
end
%% evaluate control action by solving MPC problem
[u_mpc,errorcode] = yalmip_optimizer(T-param.T_sp);
if (errorcode ~= 0)
    warning('MPC4 infeasible');
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
eps = sdpvar(repmat(nx,1,N),ones(1,N),'full');

v = 1;
E = eye(3); % punish T_F2 violation more ???

Ax=[eye(3);-eye(3)];
bx=[param.Xcons(:,2); -param.Xcons(:,1);];
Au=[eye(3);-eye(3)];
bu=[param.Ucons(:,2);-param.Ucons(:,1)];
[A_f,b_f] = compute_X_LQR(Q,R);

objective = 0;
constraints = [];
for k = 1:N-1
    constraints = [constraints, X{k+1} == param.A*X{k} + param.B*U{k}];
    constraints = [constraints, Ax*X{k+1} <= bx + [eps{k+1};eps{k+1}]];
    constraints = [constraints, eps{k} >= 0];
    constraints = [constraints, Au*U{k} <= bu];
    objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k}+ v * norm(eps{k},Inf) + eps{k}'*E*eps{k};
end
objective = objective + X{end}'*S*X{end} + v * norm(eps{end},Inf) + eps{end}'*E*eps{end};
constraints = [constraints, A_f*X{end} <= b_f];

x0 = sdpvar(3,1);
constraints = [constraints, X{1} == x0];

ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_optimizer = optimizer(constraints,objective,ops,x0,U{1});

end