function [theta1, theta2, valid, iters, err_history] = custom_IK_newton(x_target, y_target, L1, L2, options)
% Newton-Raphson IK with DLS fallback near singularities.

    % Defaults
    if nargin < 5, options = struct(); end

    tol       = get_opt(options, 'tol',      1e-10);
    max_iter  = get_opt(options, 'max_iter', 50);
    init_mode = get_opt(options, 'init',     'closedform');
    elbow_up  = get_opt(options, 'elbow_up', 1);
    lambda    = get_opt(options, 'lambda',   0.01);
    verbose   = get_opt(options, 'verbose',  false);

    % Workspace check
    r     = sqrt(x_target^2 + y_target^2);
    r_max = L1 + L2;
    r_min = abs(L1 - L2);

    theta1      = 0;
    theta2      = 0;
    valid       = 0;
    iters       = 0;
    err_history = zeros(max_iter + 1, 1);

    if r > r_max + 1e-6 || r < r_min - 1e-6
        if verbose
            fprintf('NR-IK: target (%.3f, %.3f) outside workspace. r=%.4f\n', ...
                    x_target, y_target, r);
        end
        return;
    end

    % Initial guess
    switch lower(init_mode)
        case 'closedform'
            [t1_seed, t2_seed, cf_ok] = custom_IK_2DOF(x_target, y_target, L1, L2, elbow_up);
            if cf_ok
                theta1 = t1_seed;
                theta2 = t2_seed;
            else
                theta1 = 0; theta2 = pi/4;
            end

        case 'zero'
            theta1 = 0;
            theta2 = 0;

        case 'random'
            theta1 = (rand * 2 - 1) * pi;
            theta2 = (rand * 2 - 1) * pi;

        otherwise
            error('Unknown init mode "%s". Use: closedform | zero | random', init_mode);
    end

    if verbose
        fprintf('NR-IK: target=(%.4f, %.4f)  init=%s  seed=(%.4f, %.4f)\n', ...
                x_target, y_target, init_mode, theta1, theta2);
        fprintf('  Iter |  ||error||   |  t1(deg)   t2(deg)  | method\n');
    end

    % Newton loop
    for k = 1:max_iter

        % Forward kinematics at current angles
        [x_cur, y_cur] = custom_FK_2DOF(theta1, theta2, L1, L2);

        % Position error
        e    = [x_target - x_cur; y_target - y_cur];
        e_norm = norm(e);
        err_history(k) = e_norm;

        if verbose
            fprintf('  %4d | %11.4e | %9.4f  %9.4f | ', ...
                    k, e_norm, rad2deg(theta1), rad2deg(theta2));
        end

        % Convergence check
        if e_norm < tol
            if verbose, fprintf('CONVERGED\n'); end
            iters = k - 1;
            valid = 1;
            % Wrap to [-pi, pi]
            theta1 = atan2(sin(theta1), cos(theta1));
            theta2 = atan2(sin(theta2), cos(theta2));
            err_history = err_history(1:k);
            return;
        end

        % Jacobian at current angles
        [J, w] = custom_jacobian(theta1, theta2, L1, L2);

        % computation
        sing_thresh = 1e-4;
        if w > sing_thresh
            % Exact det_J  = J(1,1)*J(2,2) - J(1,2)*J(2,1);
            J_inv  = (1/det_J) * [J(2,2), -J(1,2); -J(2,1), J(1,1)];
            delta  = J_inv * e;
            if verbose, fprintf('exact\n'); end
        else
            % DLS near singularity
            A     = J*J' + lambda^2 * eye(2);
            delta = J' * (A \ e);
            if verbose, fprintf('DLS(w=%.2e)\n', w); end
        end

        % Update angles
        theta1 = theta1 + delta(1);
        theta2 = theta2 + delta(2);
    end

    % Reached max_iter
    [x_cur, y_cur] = custom_FK_2DOF(theta1, theta2, L1, L2);
    e_final = norm([x_target - x_cur; y_target - y_cur]);
    err_history(max_iter + 1) = e_final;
    err_history = err_history(1:max_iter + 1);

    % Accept if final error is within tolerance
    if e_final < tol
        valid = 1;
    end

    iters  = max_iter;
    theta1 = atan2(sin(theta1), cos(theta1));
    theta2 = atan2(sin(theta2), cos(theta2));

    if verbose && ~valid
        fprintf('NR-IK: did not converge after %d iterations. Final err=%.2e\n', ...
                max_iter, e_final);
    end
end

% Local helper
function val = get_opt(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end