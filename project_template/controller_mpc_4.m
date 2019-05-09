% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_4(T)
% controller variables
    persistent param yalmip_optimizer e_optimizer

    % initialize controller, if not done already
    if isempty(param)
        [param, yalmip_optimizer] = init(T);
        [e_optimizer] = init_soft_constraints();
    end

    % evaluate control action by solving MPC problem, e.g.
    [E, errorcode1] = e_optimizer(T);
    if (errorcode1 ~= 0)
          warning('Soft Constraint optimization infeasible');
    end
    [u_mpc,errorcode2] = yalmip_optimizer({T, E});
    if (errorcode2 ~= 0)
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

    U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
    X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
    E = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
    
    objective = 0;
    constraints = [];
    for k = 1:N-1
        if k == 1
            constraints = [constraints, Gu*(U{k}-p_sp) <= hu, ...
                            Gx*(X{k}-T_sp) <= hx + E{k}];
            objective = objective + ((X{k}-T_sp)'*Q*(X{k}-T_sp)) + ...
                        ((U{k}-p_sp)'*R*(U{k}-p_sp));
            X{k+1} = A*(X{k}-T_sp) + B*(U{k}-p_sp);
        elseif k == N-1
            X{k+1} = A*X{k} + B*(U{k}-p_sp);
            constraints = [constraints, Gu*(U{k}-p_sp) <= hu, ...
                            Gx*X{k} <= hx + E{k}];
            constraints = [constraints, A_x * X{k+1} <= b_x];            
            objective = objective + (X{k}'*Q*X{k}) + ((U{k}-p_sp)'*R*(U{k}-p_sp));
        else
            constraints = [constraints, Gu*(U{k}-p_sp) <= hu, ...
                            Gx*X{k} <= hx + E{k}];
            objective = objective + (X{k}'*Q*X{k}) + ((U{k}-p_sp)'*R*(U{k}-p_sp));
            X{k+1} = A*X{k} + B*(U{k}-p_sp);
        end
    end
    
    objective = objective + X{k}' * (Q * F'*R*F) * X{k};

    parameters_in = {X{1}, [E{:}]};
    solutions_out = {[U{:}], [X{:}]};

    ops = sdpsettings('verbose',0,'solver','quadprog');
    % fprintf('JMPC_dummy = %f',value(objective));
    yalmip_optimizer = optimizer(constraints, objective, ops, parameters_in, solutions_out);
end

function e_optimizer = init_soft_constraints(T)
% Initializes the optimizer for finding epsilon used in introducing 
% soft constraints into the controller and returns a Yalmip optimizer 
% object.

    param = compute_controller_base_parameters; % get basic controller parameters

    % Model data
    A = param.A;                % State-space A matrix
    B = param.B;                % State-space B matrix
    T_sp = param.T_sp;          % Steady-state temp.
    p_sp = param.p_sp;          % Steady-state inputs
    Xcons = param.Xcons;        % Constraints on state
    Ucons = param.Ucons;        % Constraints on input
    nc = 3;                     % Number of constraints
    nx = size(param.A,1);       % Number of states
    nu = size(param.B,2);       % Number of inputs

    % MPC data
    N = 30;
    Gx = [1 0 0; 0 1 0; 0 -1 0];
    hx = [Xcons(1, 2); Xcons(2, 2); -Xcons(2, 1)]; 
    Gu = [eye(2); -eye(2)];
    hu = [Ucons(:, 2); -Ucons(:, 1)];

    % Soft Constraints Parameters
    S = eye(3);
    v = [0.1; 0.1; 0.1];
    
    E = sdpvar(repmat(nc,1,N),repmat(1,1,N),'full');
    X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');
    U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');

    
    objective = 0;
    constraints = [];
    for k = 1:N-1
        if k == 1
            constraints = [constraints, Gu*(U{k}-p_sp) <= hu, ...
                            Gx*(X{k}-T_sp) <= hx + E{k}];
            constraints = [constraints, E{k} >= 0];
            objective = objective + E{k}'*S*E{k} + v'*E{k};
            X{k+1} = A*(X{k}-T_sp) + B*(U{k}-p_sp);
        elseif k == N-1
            X{k+1} = A*X{k} + B*(U{k}-p_sp);
            constraints = [constraints, Gu*(U{k}-p_sp) <= hu, ...
                            Gx*X{k} <= hx + E{k}, E{k} >= 0];
            constraints = [constraints, Gx*X{k+1} <= hx + E{k+1}, ...
                            E{k+1} >= 0];
            objective = objective + E{k}'*S*E{k} + v'*E{k};
            objective = objective + E{k+1}'*S*E{k+1} + v'*E{k+1};
        else
            constraints = [constraints, Gu*(U{k}-p_sp) <= hu, ...
                            Gx*X{k} <= hx + E{k}, E{k} >= 0];
            objective = objective + E{k}'*S*E{k} + v'*E{k};
            X{k+1} = A*X{k} + B*(U{k}-p_sp);
        end
    end

    parameters_in = X{1};
    solutions_out = {[E{:}]};

    ops = sdpsettings('verbose',0,'solver','quadprog');
    % fprintf('JMPC_dummy = %f',value(objective));
    e_optimizer = optimizer(constraints, objective, ops, parameters_in, solutions_out);
end