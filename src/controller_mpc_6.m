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
persistent param yalmip_optimizer T_hat d_hat p_last
% here T_hat and d_hat means T_{k-1} and d_{k-1}

% initialize controller, if not done already
if isempty(param)
    [param, yalmip_optimizer] = init(Q,R,T,N);
    T_hat = T;          % initial state estimation
    d_hat = [1.7e3;-850;300]; % initial disturbance estimation
else
    A_aug = [param.A, param.Bd;
            zeros(3,3), eye(3)];
    B_aug = [param.B; 
            zeros(3,3)];
    C_aug = [eye(3), zeros(3,3)];
    %poles = [rand();rand();rand();rand();rand();rand()]; % tuned, abs(lambda)<1
    %save("poles",'poles');
    poles = [0.159077170263587;0.439403888885319;0.629329985950509;0.925058099643894;0.356987963341816;0.819133446743778];
    % pole placement by "K = place(A,B,p), return K, s.t. spec[A-BK] = p"
    % Find L, s.t. spec[A-LC] = p, ->i.e. find L', s.t. spec[A'-C'*L'] = p
    % i.e. L'= place(A',C',p)
    L = place(A_aug', C_aug', poles)';
    TD_k = A_aug*[T_hat;d_hat] + B_aug*p_last + L*(T - C_aug*[T_hat;d_hat]); % y(k) = T
    T_hat = TD_k(1:3);
    d_hat = TD_k(4:6);
end
% compute steady state for estimated state and disturbance
T_sp = param.T_sp;
p_sp = param.B\(-param.Bd*d_hat + (eye(3)-param.A)*T_sp);

% solve MPC problem by new equilibrum
input = [T_hat - T_sp; d_hat; T_sp; p_sp]; %% attention here!!
disp(input)
[u_mpc,errorcode] = yalmip_optimizer(input);
if (errorcode ~= 0)
      warning('MPC6 infeasible');
end
p = u_mpc + p_sp;
p_last = p;
end

function [param, yalmip_optimizer] = init(Q, R, T, N)
% get basic controller parameters
param = compute_controller_base_parameters; 
% get terminal cost
[K, S, e] = dlqr(param.A, param.B, Q, R);

% implement your MPC using Yalmip here
nx = size(param.A,1);
nu = size(param.B,2);
U = sdpvar(repmat(nu,1,N-1),ones(1,N-1),'full');
X = sdpvar(repmat(nx,1,N),ones(1,N),'full');
d = sdpvar(repmat(nx,1,N), ones(1,N),'full');

objective = 0;
constraints = [];

% Initial condition
x0 = sdpvar(12,1);
T_sp = x0(7:9);
p_sp = x0(10:12);
constraints = [constraints, X{1} == x0(1:3), d{1} == x0(4:6)];

for k = 1:N-1
    constraints = [constraints, X{k+1} == param.A*X{k} + param.B*U{k}];
    constraints = [constraints, d{k+1} == d{k}];
    constraints = [constraints, param.Tcons(:,1)-T_sp <= X{k+1} <= param.Tcons(:,2)-T_sp];
    constraints = [constraints, param.Pcons(:,1)-p_sp <= U{k} <= param.Pcons(:,2)-p_sp];
    objective = objective + X{k}'*Q*X{k} + U{k}'*R*U{k};
end
objective = objective + X{end}'*S*X{end};
[A_f,b_f] = compute_X_LQR(Q,R);
constraints = [constraints, A_f*X{end} <= b_f];

ops = sdpsettings('verbose',0,'solver','quadprog');
yalmip_optimizer = optimizer(constraints,objective,ops,x0,U{1});
end