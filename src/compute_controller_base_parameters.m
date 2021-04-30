function param = compute_controller_base_parameters
    % load truck parameters
    load('system/parameters_building');
    
    % Task 1: continuous time dynamics in state space form
    Ac = [(-building.a_F1_VC - building.a_F2_VC - building.a_Env_VC)/building.m_VC, building.a_F1_VC/building.m_VC, building.a_F2_VC/building.m_VC;
           building.a_F1_VC/building.m_F1, (-building.a_F1_VC - building.a_F2_F1)/building.m_F1, building.a_F2_F1/building.m_F1;
           building.a_F2_VC/building.m_F2, building.a_F2_F1/building.m_F2,(-building.a_F2_VC - building.a_F2_F1)/building.m_F2];
    Bc = [building.b_11/building.m_VC, building.b_12/building.m_VC, building.b_13/building.m_VC;
          building.b_21/building.m_F1, building.b_22/building.m_F1, building.b_23/building.m_F1;
          building.b_31/building.m_F2, building.b_32/building.m_F2, building.b_33/building.m_F2];
    Bdc = diag([1/building.m_VC, 1/building.m_F1, 1/building.m_F2]);
    d = [building.d_VC+building.a_Env_VC*building.T_Env;
         building.d_F1;
         building.d_F2];
    
    % Task 2: discretization
    Ts = 60;
    A = eye(3)+Ac*Ts;
    B = Bc*Ts;
    Bd = Bdc*Ts;
    
    % Task 3: set point computation
    b_ref = [25;-42;-18.5];
    C_ref = eye(3);
    T_sp = b_ref;
    p_sp = B\(Bd*d - (eye(3)-A)*T_sp);
    
    % Task 4: constraints for delta formulation
    Pcons = building.InputConstraints;
    Tcons = building.StateConstraints;
    Ucons = Pcons - [p_sp,p_sp];
    Xcons = Tcons - [T_sp,T_sp];
    
    % put everything together
    param.A = A;
    param.B = B;
    param.Bd = Bd;
    param.d = d;
    param.b_ref = b_ref;
    param.C_ref = C_ref;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
end
