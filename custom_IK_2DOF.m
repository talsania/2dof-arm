function [theta1, theta2, valid] = custom_IK_2DOF(x_target, y_target, L1, L2, elbow_up)
% CUSTOM_IK_2DOF  Inverse Kinematics — 2-DOF planar serial arm
%
%   USAGE:
%     [t1, t2, ok] = custom_IK_2DOF(x, y, L1, L2)        % elbow-up (default)
%     [t1, t2, ok] = custom_IK_2DOF(x, y, L1, L2, 0)     % elbow-down
%
%   INPUTS:
%     x_target, y_target  desired end-effector position (m)
%     L1, L2              link lengths (m)
%     elbow_up            1 = elbow-up (default), 0 = elbow-down
%
%   OUTPUTS:
%     theta1   joint 1 angle in radians, range [-pi, pi]
%     theta2   joint 2 angle in radians, range [-pi, pi]
%     valid    1 if target reachable, 0 if out of workspace
%
%   ALGORITHM: Closed-form geometric — Law of Cosines.
%   Zero toolbox dependencies.
%
%   DERIVATION SUMMARY:
%     r        = sqrt(x^2 + y^2)          distance base->target
%     cos(t2)  = (r^2 - L1^2 - L2^2) / (2*L1*L2)   law of cosines
%     t1       = atan2(y,x) - atan2(k2, k1)
%     k1       = L1 + L2*cos(t2)
%     k2       = L2*sin(t2)
%
%   BUG FIXED (v2): angle wrapping applied to theta1 via atan2(sin,cos)
%   identity to guarantee output always in [-pi, pi] for all quadrants.

    if nargin < 5
        elbow_up = 1;
    end

    % Safe defaults
    theta1 = 0;
    theta2 = 0;
    valid  = 0;

    % Workspace check
    r     = sqrt(x_target^2 + y_target^2);
    r_max = L1 + L2;        % fully extended arm
    r_min = abs(L1 - L2);   % fully folded arm

    if r > r_max + 1e-9 || r < r_min - 1e-9
        warning('IK: target (%.3f, %.3f) outside workspace. r=%.4f, limits=[%.4f, %.4f]', ...
                x_target, y_target, r, r_min, r_max);
        return;
    end

    % Step 1: Solve theta2 (law of cosines)
    cos_theta2 = (x_target^2 + y_target^2 - L1^2 - L2^2) / (2 * L1 * L2);
    cos_theta2 = max(-1, min(1, cos_theta2));   % clamp for float safety

    sin_theta2 = sqrt(1 - cos_theta2^2);        % magnitude only
    if elbow_up == 1
        theta2 = atan2( sin_theta2, cos_theta2);   % elbow-up  (+sin)
    else
        theta2 = atan2(-sin_theta2, cos_theta2);   % elbow-down (-sin)
    end

    % Step 2: Solve theta1
    alpha  = atan2(y_target, x_target);
    beta   = atan2(L2 * sin(theta2), L1 + L2 * cos(theta2));

    % wrap both to [-pi, pi]  (FIX: alpha-beta can exceed pi in Q2/Q3)
    theta1 = atan2(sin(alpha - beta), cos(alpha - beta));
    theta2 = atan2(sin(theta2),       cos(theta2));

    valid = 1;

end
