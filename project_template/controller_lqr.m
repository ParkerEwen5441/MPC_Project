% BRIEF:
%   Controller function template. Input and output dimension MUST NOT be
%   modified.
% INPUT:
%   T: Measured system temperatures, dimension (3,1)
% OUTPUT:
%   p: Cooling power, dimension (2,1)
function p = controller_lqr(T)
    % controller variables
    persistent param;

    % initialize controller, if not done already
    if isempty(param)
        param = init();
    end
    
    % compute control action
    T_in = T - param.T_sp;
    p = (param.F * T_in) + param.p_sp;
    param.counter = param.counter + 1;
    
    % compute infinite-horizon cost
    if param.counter == 1
        param.J = (T-param.T_sp)' * param.P * (T-param.T_sp);
    end
    
    % check state constraint given in problem
    if (param.counter == 30) && ((norm(param.T_sp - T) <= (0.2 * norm([3, 1, 0]))))
        disp(" ")
        disp("-----------------------------")
        disp("           T(30)             ")
        disp("-----------------------------")
        disp(T)
        disp("Constraint is satisfied.")
    elseif (param.counter == 30)
        disp("Constraint is not satisfied!")
    end
    
end

function param = init()
    param = compute_controller_base_parameters;
end