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
persistent param yalmip_optimizer x_hat d_hat Xs Us

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q,R,T,N);
    x_hat=T;
    d_hat=[0;0;0];
    Xs=param.T_sp;
    Us=param.p_sp;
end

% evaluate control action by solving MPC problem
[u_mpc,errorcode] = yalmip_optimizer(T-Xs,Xs,Us);
if (errorcode ~= 0)
    warning('MPC6 infeasible');
end
p = value(u_mpc)+Us;

% observer update
poles = [0.159077170263587;0.439403888885319;0.629329985950509;0.925058099643894;0.356987963341816;0.819133446743778];
L = place(param.A_aug', param.C_aug', poles)';
x_aug=param.A_aug*[x_hat;d_hat]+param.B_aug*p+L*(T-param.C_aug*[x_hat;d_hat]);
x_hat=x_aug(1:3);
d_hat=x_aug(4:6);

% set point update
x_aug_set=[param.A-eye(3),param.B;param.C_ref*eye(3),zeros(3)]\[-param.Bd*d_hat;param.b_ref];
Xs=x_aug_set(1:3);
Us=x_aug_set(4:6);
disp([x_hat,d_hat,Xs,Us])
end

function [param, yalmip_optimizer] = init(Q,R,T,N)
% get basic controller parameters
    param = compute_controller_base_parameters;
% get terminal cost
	[~,P_inf,~] = dlqr(param.A,param.B,Q,R);
% get terminal set
    [A_x, b_x] = compute_X_LQR(Q, R);
% implement your MPC using Yalmip here
    nx = size(param.A,1);
    nu = size(param.B,2);
    Tcons=param.Tcons;
    Pcons=param.Pcons;
    U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
    X = sdpvar(repmat(nx,1,N),ones(1,N),'full');    

    T0 = sdpvar(nx,1,'full');
    xs = sdpvar(nx,1);
    us = sdpvar(nu,1);
    
    constraints=[X{1}==T0];
    objective = 0;
    for k = 1:N-1
	    constraints = [constraints,  X{k+1} == param.A*X{k} + param.B*U{k}];
        constraints = [constraints, Pcons(:,1) <= U{k}+us <= Pcons(:,2), Tcons(:,1)<=X{k+1}+xs<=Tcons(:,2)];
        objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k};
    end
    constraints=[constraints, A_x*X{N}<=b_x];
    objective = objective + X{N}'*P_inf*X{N};
    ops = sdpsettings('verbose',1,'solver','quadprog');
    yalmip_optimizer = optimizer(constraints,objective,ops,{T0,xs,us},U{1});
end