function param = compute_controller_base_parameters
    % load truck parameters
    load('system/parameters_truck');
    
    % (2) discretization
    Ts = 60;
    T0 = truck.To;
    
    w = truck.w;
    
    a10 = truck.a1o;
    a20 = truck.a2o;
    a30 = truck.a3o;
    a12 = truck.a12;
    a23 = truck.a23;
    m1 = truck.m1;
    m2 = truck.m2;
    m3 = truck.m3;
    
    A_c = [(-a10-a12)/m1, a12/m1, 0;
         a12/m2, (-a20-a12-a23)/m2, a23/m2;
         0, a23/m3, (-a30-a23)/m3];
    B_c = [1/m1, 0;
           0, 1/m2;
           0,  0];
    B_d = [60/m1 0 0;
            0 60/m2 0;
            0 0 60/m3];
    d_c = [a10*T0 + w(1);
           a20*T0 + w(2);
           a30*T0 + w(3)];
    sys = ss(A_c, B_c, eye(3), []);
    sysd = c2d(sys, Ts);
    A = sysd.A;
    B = sysd.B;
       
    % (3) set point computation    
    H = [1 0 0;
         0 1 0];
    A_sp = [eye(3) - A, -B; H, zeros(2, 2)];
    b_sp = [B_d * d_c; [-20; 0.25]];
    x_sp = A_sp \ b_sp;
    
    T_sp = x_sp(1:3);
    p_sp = x_sp(4:5);
    
    % (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = Pcons - p_sp;
    Xcons = Tcons - T_sp;
    
    % (5) LQR cost function
    Q = [3000 0 0;
         0 1500 0;
         0 0 0];
    R = [0.05 0;
         0 0.05];
     
    % add extra parameters
    [P,~,K] = dare(A, B, Q, R);
 
    % put everything together
    param.A = A;
    param.B = B;
    param.Q = Q;
    param.R = R;
    param.P = P;
    param.F = -K;
    param.T_sp = T_sp;
    param.p_sp = p_sp;
    param.Ucons = Ucons;
    param.Xcons = Xcons;
    param.Tcons = Tcons;
    param.Pcons = Pcons;
    param.counter = 0;
end

