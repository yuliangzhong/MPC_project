% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_4(T)
% controller variables
persistent param yalmip_optimizer

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init();
end

%% evaluate control action by solving MPC problem, e.g.
[u_mpc,errorcode] = yalmip_optimizer(T-param.T_sp);
if (errorcode ~= 0)
      warning('MPC infeasible');
end
p = u_mpc+param.p_sp;
end

function [param, yalmip_optimizer] = init()
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

param = compute_controller_base_parameters; % get basic controller parameters
[param.K, param.P] = dlqr(param.A, param.B, param.Q, param.R);
%% implement your MPC using Yalmip here, e.g.
N = 30;
nx = size(param.A,1);
nu = size(param.B,2);

U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
E = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');

v = 1e6;

Ax=[eye(3);-eye(3)];
bx=[param.Xcons(:,2);-param.Xcons(:,1)];
Au=[eye(2);-eye(2)];
bu=[param.Ucons(:,2);-param.Ucons(:,1)];
[Ax_t, bx_t] = compute_X_LQR;
***nx_end = size(Ax_t,1);
***E_end = sdpvar(nx_end,1);

objective = 0;
***constraints = [E{1} >= 0, E_end >= 0];
for i = 1:N-1
  constraints = [constraints,  X{i+1} == param.A * X{i} + param.B * U{i}];
  constraints = [constraints, Ax * X{i+1} - [E{i+1};E{i+1}] <= bx];
  constraints = [constraints, E{i+1} >= 0];
  constraints = [constraints, Au * U{i} <= bu];
  objective = objective +X{i}' * param.Q * X{i} + U{i}' * param.R * U{i} + v * norm(E{i},1) ;
end
constraints = [constraints, Ax_t * X{end}- E_end <= bx_t];
objective = objective + X{end}'*param.P*X{end} + v * norm(E_end,1);
% Initial condition
x0 = sdpvar(3,1);
constraints = [constraints, X{1} == x0];

ops = sdpsettings('verbose',0,'solver','quadprog');
fprintf('JMPC_dummy = %f',value(objective));
yalmip_optimizer = optimizer(constraints,objective,ops,x0,U{1} );
end