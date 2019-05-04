% BRIEF:
%   Controller function template. This function can be freely modified but
%   input and output dimension MUST NOT be changed.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_mpc_1(T)
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
    T_sp = param.T_sp;
    p_sp = param.p_sp;
    Xcons = param.Xcons;
    Ucons = param.Ucons;
    nx = size(param.A,1);
    nu = size(param.B,2);

    % MPC data
    Q = param.Q;
    R = param.R;
    J = (T-param.T_sp)' * param.P * (T-param.T_sp);
    N = 30;
    Gx = [1 0 0; 0 1 0; 0 -1 0];
    hx = [10; 4.75; 0.25]; 
    Gu = [eye(2); -eye(2)];
    hu = [Ucons(:, 2); -Ucons(:, 1)];

    U = sdpvar(repmat(nu,1,N-1),repmat(1,1,N-1),'full');
    X = sdpvar(repmat(nx,1,N),repmat(1,1,N),'full');

    objective = J;
    constraints = [];
    for k = 1:N-1
        if k == 1
            constraints = [constraints, Gu*(U{k}-p_sp) <= hu, ...
                            Gx*(X{k}-T_sp) <= hx];
            objective = objective + ((X{k}-T_sp)'*Q*(X{k}-T_sp)) + ...
                        ((U{k}-p_sp)'*R*(U{k}-p_sp));
            X{k+1} = A*(X{k}-T_sp) + B*(U{k}-p_sp);
        else
            constraints = [constraints, Gu*(U{k}-p_sp) <= hu, ...
                            Gx*X{k} <= hx];
            objective = objective + (X{k}'*Q*X{k}) + ((U{k}-p_sp)'*R*(U{k}-p_sp));
            X{k+1} = A*X{k} + B*(U{k}-p_sp);
        end
    end

    parameters_in = X{1};
    solutions_out = {[U{:}], [X{:}]};

    ops = sdpsettings('verbose',0,'solver','quadprog');
    fprintf('JMPC_dummy = %f',value(objective));
    yalmip_optimizer = optimizer(constraints, objective, ops, parameters_in, solutions_out);
end