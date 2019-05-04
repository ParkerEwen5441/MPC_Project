% BRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters0
    param = compute_controller_base_parameters;
   
    absxmin = param.Xcons(1:3, 1);
    absxmax = param.Xcons(1:3, 2);
    absumin = param.Ucons(1:2, 1);
    absumax = param.Ucons(1:2, 2);
       
    A_x = [eye(3); -eye(3); param.F; -param.F];
    b_x = [absxmax;-absxmin; absumax;-absumin];
    
    Xp = Polyhedron('A', A_x, 'b', b_x);
    figure(2)
    Xp.plot();
    alpha(0.25);
    title('Resulting State Constraints under LQR Control');
    xlabel('x_1');
    ylabel('x_2');
end

