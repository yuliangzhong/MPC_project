% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% INPUT:
%   Q, R: State and input weighting matrix, dimension (3,3)
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}

function [A_x, b_x] = compute_X_LQR(Q, R)
    % get basic controller parameters
    param = compute_controller_base_parameters;
    A = param.A;
    B = param.B;
    [K,S,e] = dlqr(A,B,Q,R);
    sys_LQR = LTISystem('A', A - B * K);
    Xmax = param.Xcons(:,2);
    Xmin = param.Xcons(:,1);
    Umax = param.Ucons(:,2);
    Umin = param.Ucons(:,1);
    Xp = Polyhedron('A',[eye(3);-eye(3);K;-K;],'b',[Xmax; -Xmin; Umax; -Umin;]);
    % figure(9);hold on; grid on;
    sys_LQR.x.with('setConstraint');
    sys_LQR.x.setConstraint = Xp;
    InvSetLQR = sys_LQR.invariantSet();
    % InvSetLQR.plot(), alpha(0.25), title('Invariant Set under LQR Control'), xlabel('DT_vc'), ylabel('DT_1'),zlabel('DT_2')
    A_x = InvSetLQR.A;
    b_x = InvSetLQR.b;
end