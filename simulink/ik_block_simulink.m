function [theta1, theta2] = ik_block_simulink(x_target, y_target)
% IK_BLOCK_SIMULINK  Drop this function INSIDE a Simulink MATLAB Function block.
%
%   COPY-PASTE THIS ENTIRE FILE into the MATLAB Function block editor.
%   Do NOT call it from outside - Simulink calls it automatically.
%
%   Inputs  (connected as Simulink input ports):
%     x_target   desired end-effector X position (m)
%     y_target   desired end-effector Y position (m)
%
%   Outputs (connected as Simulink output ports):
%     theta1     joint 1 angle in radians
%     theta2     joint 2 angle in radians
%
%   LINK LENGTHS - must match fk_block_simulink.m:
    L1 = 1.0;   % link 1 length (m)
    L2 = 0.8;   % link 2 length (m)

%   ALGORITHM: Closed-form geometric (Law of Cosines).
%   Elbow-up solution. No toolbox, no iteration.
%   Output always in [-pi, pi].

    % workspace clamp (avoids NaN in acos if target is slightly outside)
    r     = sqrt(x_target^2 + y_target^2);
    r_max = L1 + L2;
    r_min = abs(L1 - L2);

    % saturate r to valid workspace (prevents NaN - Simulink safe)
    r_safe = max(r_min + 1e-6, min(r_max - 1e-6, r));

    % scale target to lie inside workspace if it was outside
    if r > 1e-9
        x_safe = x_target * (r_safe / r);
        y_safe = y_target * (r_safe / r);
    else
        x_safe = r_safe;
        y_safe = 0;
    end

    % solve theta2 (law of cosines)
    cos_t2 = (x_safe^2 + y_safe^2 - L1^2 - L2^2) / (2 * L1 * L2);
    cos_t2 = max(-1, min(1, cos_t2));   % float clamp
    sin_t2 = sqrt(1 - cos_t2^2);        % elbow-up: positive sin
    theta2  = atan2(sin_t2, cos_t2);

    % solve theta1
    alpha  = atan2(y_safe, x_safe);
    beta   = atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));
    theta1 = atan2(sin(alpha - beta), cos(alpha - beta));   % wrapped

end
