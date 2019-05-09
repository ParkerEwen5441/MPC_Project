% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_5(T)
% controller variables
    persistent param yalmip_optimizer

    % initialize controller, if not done already
    if isempty(param)
        param.T0 = T;
        [param, yalmip_optimizer] = init(T);
    end

    % evaluate control action by solving MPC problem, e.g.
    [u_mpc,errorcode] = yalmip_optimizer(T);
    if (errorcode ~= 0)
          warning('MPC infeasible');
    end
    p = u_mpc{1}(:, 1);
end

function [param, yalmip_optimizer] = init(T)
% initializes the controller on first call and returns parameters and
% Yalmip optimizer object

    param = compute_controller_base_parameters; % get basic controller parameters

    % Model data
    A = param.A;
    B = param.B;
    C = eye(3);
    Cd = eye(3);
    Bd = param.Bd;
    T_sp = param.T_sp;
    p_sp = param.p_sp;
    Xcons = param.Xcons;
    Ucons = param.Ucons;
    nx = size(param.A,1);
    nu = size(param.B,2);

    % Terminal Set
    [A_x, b_x] = compute_X_LQR;
    
    % MPC data
    Q = param.Q;
    R = param.R;
    F = param.F;
    N = 30;
    Gx = [1 0 0; 0 1 0; 0 -1 0];
    hx = [Xcons(1, 2); Xcons(2, 2); -Xcons(2, 1)]; 
    Gu = [eye(2); -eye(2)];
    hu = [Ucons(:, 2); -Ucons(:, 1)];
    J = (T-param.T_sp)' * param.P * (T-param.T_sp);
    
    % Augmented system data
    A_aug = [A, Bd; zeros(3), eye(3)];
    B_aug = [B; zeros(3, 2)];
    C_aug = [eye(3), eye(3)];
    D_aug = zeros(1, 2);
%     Q_aug = [Q, zeros(3); zeros(3), ones(3)];
%     [~,~,K] = dare(A_aug, B_aug, Q_aug, R);
%     sysd = ss(A_aug, B_aug*(-K));    

    % Observer Design
%     P = pole(sysd) - 0.01;
    L = -(place(A_aug',C_aug',[0.25, 0.1, 1.25, 0.5, 2, 1]))';
    
    U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
    Y = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
    X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
    D = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');

    objective = 0;
    constraints = [];
    for k = 1:N-1
        if k == 1
            Y{k} = C*(X{k} - T_sp) + Cd*D{k};
            X{k+1} = A*(X{k} - T_sp) + Bd*D{k} + B*(U{k} - p_sp) + ...
                        L(1:3, :)*(-Y{k} + C*(X{k}-T_sp) + Cd*D{k});
            D{k+1} = eye(3)*D{k} + L(4:6, :)*(-Y{k} + C*(X{k}-T_sp) + eye(3)*D{k});
            constraints = [constraints, Gu*(U{k}-p_sp) <= hu, ...
                            Gx*(X{k}-T_sp) <= hx];
            objective = objective + ((X{k}-T_sp)'*Q*(X{k}-T_sp)) + ...
                        ((U{k}-p_sp)'*R*(U{k}-p_sp));
        elseif k == N-1
            Y{k} = C*X{k} + Cd*D{k};
            X{k+1} = A*X{k} + Bd*D{k} + B*(U{k} - p_sp) + ...
                        L(1:3, :)*(-Y{k} + C*X{k} + Cd*D{k});
            D{k+1} = eye(3)*D{k} + L(4:6, :)*(-Y{k} + C*X{k} + eye(3)*D{k});
            constraints = [constraints, Gu*(U{k}-p_sp) <= hu, ...
                            Gx*X{k} <= hx];
            constraints = [constraints, A_x * X{k+1} <= b_x];            
            objective = objective + (X{k}'*Q*X{k}) + ((U{k}-p_sp)'*R*(U{k}-p_sp));
        else
            Y{k} = C*X{k} + Cd*D{k};
            X{k+1} = A*X{k} + Bd*D{k} + B*(U{k} - p_sp) + ...
                        L(1:3, :)*(-Y{k} + C*X{k} + Cd*D{k});
            D{k+1} = eye(3)*D{k} + L(4:6, :)*(-Y{k} + C*X{k} + Cd*D{k});
            constraints = [constraints, Gu*(U{k}-p_sp) <= hu, ...
                            Gx*X{k} <= hx];
            objective = objective + (X{k}'*Q*X{k}) + ((U{k}-p_sp)'*R*(U{k}-p_sp));
        end
    end
    
    objective = objective + X{k}' * (Q * F'*R*F) * X{k};

    parameters_in = X{1};
    solutions_out = {[U{:}], [X{:}]};

    ops = sdpsettings('verbose',0,'solver','quadprog');
    fprintf('JMPC_dummy = %f',value(objective));
    yalmip_optimizer = optimizer(constraints, objective, ops, parameters_in, solutions_out);
end