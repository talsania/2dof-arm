function [theta_dot, method_used, condition_num] = custom_IK_velocity(theta1, theta2, x_dot, L1, L2, method, lambda)
% Joint velocity IK using exact, transpose, DLS, or auto mode.

    % Defaults
    if nargin < 6 || isempty(method), method = 'auto'; end
    if nargin < 7 || isempty(lambda), lambda = 0.05;   end

    % Input validation
    x_dot = x_dot(:);
    if numel(x_dot) ~= 2
        error('x_dot must be 2x1 [dx_dot; dy_dot]');
    end

    % Jacobian
    [J, w, ~, is_singular] = custom_jacobian(theta1, theta2, L1, L2);

    % Condition number
    if is_singular
        condition_num = inf;
    else
        condition_num = cond(J);
    end

    % Auto mode method choice from manipulability
    if strcmpi(method, 'auto')
        if w > 0.1
            method = 'exact';
        elseif w > 0.01
            % smaller w -> larger lambda
            lambda = 0.05 * (0.1 / max(w, 1e-6));
            lambda = min(lambda, 0.5);
            method = 'dls';
        else
            method = 'transpose';
        end
    end

    % Compute theta_dot
    switch lower(method)

        % exact inverse
        case 'exact'
            det_J = J(1,1)*J(2,2) - J(1,2)*J(2,1);
            if abs(det_J) < 1e-10
                warning('IK_velocity/exact: near-singular (det=%.2e). Use dls or transpose.', det_J);
                % fallback to safe method
                theta_dot  = J' * x_dot;
                method_used = 'transpose_fallback';
                return;
            end
            J_inv      = (1/det_J) * [ J(2,2), -J(1,2); -J(2,1),  J(1,1)];
            theta_dot  = J_inv * x_dot;
            method_used = 'exact';

        % transpose
        case 'transpose'
            theta_dot   = J' * x_dot;
            method_used = 'transpose';

        % damped least squares
        case 'dls'
            A          = J * J' + lambda^2 * eye(2);
            theta_dot  = J' * (A \ x_dot);
            method_used = sprintf('dls(lambda=%.4f)', lambda);

        otherwise
            error('Unknown method "%s". Choose: exact | transpose | dls | auto', method);
    end
end