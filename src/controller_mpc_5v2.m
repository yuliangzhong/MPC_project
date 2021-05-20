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
[u_mpc,errorcode] = yalmip_optimizer(T-param.T_sp,num2cell(d,1));
if (errorcode ~= 0)
    warning('MPC5 infeasible');
end
p = value(u_mpc)+param.p_sp;
end

function [param, yalmip_optimizer] = init(Q,R,N)
% get basic controller parameters
    param = compute_controller_base_parameters;
% get terminal cost
	[~,P_inf,~] = dlqr(param.A,param.B,Q,R);
% get terminal set
    [A_x, b_x] = compute_X_LQR(Q, R);
% implement your MPC using Yalmip here
    nx = size(param.A,1);
    nu = size(param.B,2);
    U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
    X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
    eps = sdpvar(repmat(nx,1,N),ones(1,N),'full');%slack variables
    T0 = sdpvar(nx,1,'full');
    Ucons=param.Ucons;
    Xcons=param.Xcons;

    d = sdpvar(repmat(nx,1,N),ones(1,N),'full');
    v=1000000;%linear penalty for epsilon
    S=1000000*eye(3);%quadratic penalty for epsilon
    
    constraints=[X{1}==T0];
    objective = 0;

    for k = 1:N-1
	    constraints = [constraints,  X{k+1} == param.A*X{k} + param.B*U{k}+param.Bd*d{k}];
        constraints = [constraints, Ucons(:,1) <= U{k}<= Ucons(:,2), Xcons(:,1)-eps{k}<=X{k+1}<=Xcons(:,2)+eps{k},eps{k}>=0];
        objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k}+v*norm(eps{k},Inf)+eps{k}'*S*eps{k};
    end
    constraints=[constraints, A_x*X{N}<=b_x];
    objective = objective + X{N}'*P_inf*X{N}+v*norm(eps{N},1)+eps{N}'*S*eps{N};
    ops = sdpsettings('verbose',1,'solver','quadprog');
    yalmip_optimizer = optimizer(constraints,objective,ops,{T0,d{1:N}},U{1});

end