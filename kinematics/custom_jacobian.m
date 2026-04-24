function [J, w, det_J, is_singular, best_elbow] = custom_jacobian(theta1, theta2, L1, L2, lambda_thresh)
% CUSTOM_JACOBIAN  Geometric Jacobian for 2-DOF planar serial arm
%
%   USAGE:
%     [J, w, det_J, is_singular, best_elbow] = custom_jacobian(t1, t2, L1, L2)
%     [J, w, det_J, is_singular, best_elbow] = custom_jacobian(t1, t2, L1, L2, lambda_thresh)
%
%   INPUTS:
%     theta1, theta2   joint angles (rad)
%     L1, L2          link lengths (m)
%     lambda_thresh   singularity threshold on |det(J)|, default = 1e-4
%
%   OUTPUTS:
%     J              2x2 geometric Jacobian  [dx/dt1  dx/dt2; dy/dt1  dy/dt2]
%     w              Yoshikawa manipulability measure  w = |det(J)|
%     det_J          raw determinant of J (signed)
%     is_singular    logical: true if |det_J| < lambda_thresh
%     best_elbow     'up' or 'down' — whichever elbow config has higher w
%                    (requires x_target,y_target — only meaningful when called
%                     from custom_IK_velocity with elbow comparison)
%
%   MATH:
%     EE position:
%       x = L1*cos(t1) + L2*cos(t1+t2)
%       y = L1*sin(t1) + L2*sin(t1+t2)
%
%     Partial derivatives:
%       dx/dt1 = -L1*sin(t1) - L2*sin(t1+t2)
%       dx/dt2 =              - L2*sin(t1+t2)
%       dy/dt1 =  L1*cos(t1) + L2*cos(t1+t2)
%       dy/dt2 =               L2*cos(t1+t2)
%
%     Jacobian:
%       J = [ -L1s1 - L2s12,  -L2s12 ]
%           [  L1c1 + L2c12,   L2c12 ]
%       where s1=sin(t1), c1=cos(t1), s12=sin(t1+t2), c12=cos(t1+t2)
%
%     Manipulability (Yoshikawa 1985):
%       w = sqrt(det(J * J^T)) = |det(J)|   [for square J]
%
%     Singularity: det(J) = L1*L2*sin(t2)
%       => singular when t2 = 0 or t2 = ±pi (arm fully extended/folded)

    if nargin < 4
        error(['custom_jacobian requires at least 4 inputs: ', ...
               'custom_jacobian(theta1, theta2, L1, L2, [lambda_thresh])']);
    end

    if nargin < 5, lambda_thresh = 1e-4; end

    % Precompute trig
    s1   = sin(theta1);
    c1   = cos(theta1);
    s12  = sin(theta1 + theta2);
    c12  = cos(theta1 + theta2);

    % Build 2x2 Jacobian
    J = [ -L1*s1 - L2*s12,  -L2*s12 ;
           L1*c1 + L2*c12,   L2*c12 ];

    % Determinant and manipulability
    det_J = det(J);                    % = L1*L2*sin(t2) analytically
    w     = abs(det_J);                % Yoshikawa measure

    % Singularity flag
    is_singular = (w < lambda_thresh);

    % best_elbow: compare w for elbow-up vs elbow-down
    % Analytical: det(J_up) = L1*L2*sin(t2_up), det(J_down) = L1*L2*sin(t2_down)
    % Since t2_down = -t2_up, sin(t2_down) = -sin(t2_up)
    % So |det_up| == |det_down| always — elbow choice doesn't change w for same target
    % We still return based on current config sign
    if det_J >= 0
        best_elbow = 'up';
    else
        best_elbow = 'down';
    end
end