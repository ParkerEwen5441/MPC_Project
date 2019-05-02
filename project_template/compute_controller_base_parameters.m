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
    
    A = [1+60*(-a10-a12)/m1, 60*a12/m1,              0;
        60*a12/m2,           1+60*(-a20-a12-a23)/m2, 60*a23/m2;
        0,                   60*a23/m3,              1+60*(-a30-a23)/m3];
    B = [60/m1  0;
         0      60/m2;
         0      0];
    
    % (3) set point computation
    T3sp = (15*a23/m3 + 60*(a30*T0-w(3))/m3) / (-60*(-a30-a23)/m3);
    
    p1sp = (m1/60)*((-60*(-a10-a12)/m1)*(-20) - (60*a12/m1)*(0.25) - 60*(a10*T0+w(1))/m1);
    p2sp = (m2/60)*((-60*a12/m2)*(-20) - (60*(-a20-a12-a23)/m2)*(0.25) - (60*a23/m2)*T3sp - 60*(a20*T0+w(2))/m2);
    
    T_sp = [-20; 0.25; T3sp];
    p_sp = [p1sp; p2sp];
    
    % (4) system constraints
    Pcons = truck.InputConstraints;
    Tcons = truck.StateConstraints;
    
    % (4) constraints for delta formulation
    Ucons = [-681.2 1818.8; -1373.8 626.25];
    Xcons = [-Inf 10; -0.25 4.75; -Inf Inf];
    
    % (5) LQR cost function
    Q = [60 0 0;
         0 50 0;
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
end

