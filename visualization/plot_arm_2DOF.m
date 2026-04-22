function plot_arm_2DOF(theta1, theta2, L1, L2, target_x, target_y)
% PLOT_ARM_2DOF  Draw the 2-DOF arm given joint angles.
%   Called from main_test_2DOF.m and animation loops.

    x0 = 0; y0 = 0;
    x1 = L1 * cos(theta1);
    y1 = L1 * sin(theta1);
    x2 = x1 + L2 * cos(theta1 + theta2);
    y2 = y1 + L2 * sin(theta1 + theta2);

    cla;

    % links
    plot([x0 x1], [y0 y1], 'b-', 'LineWidth', 4); hold on;
    plot([x1 x2], [y1 y2], 'r-', 'LineWidth', 4);

    % joints
    plot(x0, y0, 'ks', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
    plot(x1, y1, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', [0.3 0.3 0.3]);
    plot(x2, y2, 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

    % target
    if nargin == 6
        plot(target_x, target_y, 'rx', 'MarkerSize', 16, 'LineWidth', 3);
    end

    % angle arcs
    arc = 0.12;
    ta1 = linspace(0, theta1, 50);
    plot(arc*cos(ta1), arc*sin(ta1), 'm-', 'LineWidth', 1.5);
    text(arc*1.4*cos(theta1/2), arc*1.4*sin(theta1/2), ...
         sprintf('%.0f°', rad2deg(theta1)), 'FontSize', 8, 'Color', 'm');

    ta2 = linspace(theta1, theta1+theta2, 50);
    plot(x1+arc*cos(ta2), y1+arc*sin(ta2), 'c-', 'LineWidth', 1.5);
    text(x1+arc*1.4*cos(theta1+theta2/2), y1+arc*1.4*sin(theta1+theta2/2), ...
         sprintf('%.0f°', rad2deg(theta2)), 'FontSize', 8, 'Color', 'c');

    lim = (L1+L2)*1.3;
    xlim([-lim lim]); ylim([-lim lim]);
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)');
    title(sprintf('EE = (%.3f, %.3f)', x2, y2));
    hold off;
end
