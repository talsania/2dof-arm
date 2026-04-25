function [a_axis, b_axis, angle_deg, w] = plot_jacobian_ellipse(theta1, theta2, L1, L2, varargin)
% Plot velocity manipulability ellipse for a 2-DOF arm.

    % Parse optional args
    p_scale    = 0.25;
    p_draw_arm = true;
    p_color    = [0.2 0.6 1.0];
    p_alpha    = 0.35;
    p_n        = 200;
    p_label    = true;
    p_hold     = false;

    i = 1;
    while i <= length(varargin)
        key = lower(varargin{i});
        val = varargin{i+1};
        switch key
            case 'scale',    p_scale    = val;
            case 'draw_arm', p_draw_arm = val;
            case 'color',    p_color    = val;
            case 'alpha',    p_alpha    = val;
            case 'n_pts',    p_n        = val;
            case 'label',    p_label    = val;
            case 'hold_on',  p_hold     = val;
        end
        i = i + 2;
    end

    % Jacobian and SVD
    [J, w] = custom_jacobian(theta1, theta2, L1, L2);

    [U, S, ~] = svd(J);
    s1 = S(1,1);
    s2 = S(2,2);

    a_axis    = s1;
    b_axis    = s2;

    % major axis angle
    angle_deg = atan2(U(2,1), U(1,1)) * 180 / pi;

    % End-effector position
    [x_ee, y_ee, x1, y1] = custom_FK_2DOF(theta1, theta2, L1, L2);

    % Draw arm
    if ~p_hold, cla; end

    if p_draw_arm
        plot([0 x1], [0 y1], '-', 'Color', [0.2 0.2 0.8], 'LineWidth', 4);
        hold on;
        plot([x1 x_ee], [y1 y_ee], '-', 'Color', [0.8 0.2 0.2], 'LineWidth', 4);
        plot(0,    0,    'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
        plot(x1,   y1,   'ko', 'MarkerSize', 10, 'MarkerFaceColor', [0.3 0.3 0.3]);
        plot(x_ee, y_ee, 'go', 'MarkerSize', 10, 'MarkerFaceColor', [0 0.8 0]);
    else
        hold on;
    end

    % Ellipse points
    phi    = linspace(0, 2*pi, p_n);
    e_local = [cos(phi); sin(phi)];
    e_world = p_scale * U * S * e_local;

    ex = x_ee + e_world(1,:);
    ey = y_ee + e_world(2,:);

        % Draw ellipse
    fill(ex, ey, p_color, 'FaceAlpha', p_alpha, 'EdgeColor', p_color*0.6, ...
         'LineWidth', 1.5);

        % Major axis
    dx1 = p_scale * s1 * U(1,1);
    dy1 = p_scale * s1 * U(2,1);
    plot([x_ee - dx1, x_ee + dx1], [y_ee - dy1, y_ee + dy1], ...
         '-', 'Color', p_color*0.5, 'LineWidth', 2);

    % Minor axis
    dx2 = p_scale * s2 * U(1,2);
    dy2 = p_scale * s2 * U(2,2);
    plot([x_ee - dx2, x_ee + dx2], [y_ee - dy2, y_ee + dy2], ...
         '--', 'Color', p_color*0.5, 'LineWidth', 1.5);

    plot(x_ee, y_ee, '+', 'Color', p_color*0.4, 'MarkerSize', 8, 'LineWidth', 2);

    % Annotation
    if p_label
        txt = sprintf('w=%.3f\na=%.3f  b=%.3f\n\\phi=%.1f°', ...
                      w, a_axis, b_axis, angle_deg);
        text(x_ee + 0.06, y_ee + 0.06, txt, ...
             'FontSize', 8, 'Color', [0.1 0.1 0.5], ...
             'BackgroundColor', [1 1 1 0.7], 'EdgeColor', [0.7 0.7 0.9]);
    end

    % Axis formatting
    lim = (L1 + L2) * 1.25;
    xlim([-lim lim]);
    ylim([-lim lim]);
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('Velocity Manipulability Ellipse  |  \\theta_1=%.0f°  \\theta_2=%.0f°', ...
                  theta1*180/pi, theta2*180/pi));

    hold off;
end