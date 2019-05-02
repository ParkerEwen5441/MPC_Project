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
    
    %compute infinite-horizon cost
    J = [-17; 1.25; 6.1]' * param.P * [-17; 1.25; 6.1];
    
    % compute control action
    p = (param.F * T) + [-1818.8; -626.25];
    end

function param = init()
    param = compute_controller_base_parameters;
    % add additional parameters if necessary, e.g.
    [P,L,K] = dare(param.A, param.B, param.Q, param.R);
    param.P = P;
    param.F = -K;

end