% BRRIEF:
%   Template for explicit invariant set computation. You MUST NOT change
%   the output.
% OUTPUT:
%   A_x, b_x: Describes polytopic X_LQR = {x| A_x * x <= b_x}
function [A_x, b_x] = compute_X_LQR
    % get basic controller parameters
    param = compute_controller_base_parameters;
    
    A = param.A;
    b = param.B;
    K = param.F;
    
    absxmin = [-Inf; 0; -Inf];
    absxmax = [-10; 5; Inf];
    absumin = [-2500; -2000];
    absumax = [0; 0];
    
    system = LTISystem('A', A, 'B', A+b*K);
    Xp = Polyhedron('A',[eye(3); -eye(3); K; -K], 'b', [absxmax;-absxmin; absumax;-absumin]);
    system.x.with('setConstraint');
    system.x.setConstraint = Xp;
    % system.x.min = [-Inf; 0; -Inf];
    % system.x.max = [-10; 5; Inf];
    % system.u.min = [-2500; -2000];
    % system.u.max = [0; 0];
    InvSet = system.invariantSet()
    InvSet.plot()
    
    A_x = InvSet.A;
    b_x = InvSet.b;
end

