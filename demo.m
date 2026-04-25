%% demo
% Jacobian manipulability ellipse demo.

% -- paths ----------------------------------------------------------
here     = fileparts(mfilename('fullpath'));
root_dir = here;
addpath(fullfile(root_dir, 'kinematics'));     % custom_jacobian, custom_FK_2DOF
addpath(fullfile(root_dir, 'visualization'));  % plot_jacobian_ellipse, ellipse_math
addpath(fullfile(root_dir, 'tests'));          % rs.m (if running tests alongside)

L1 = 1.0;
L2 = 0.8;

% Figure 1: key configurations
configs = [
%   t1(deg)  t2(deg)
    0        90        ;
    0        45        ;
    0        20        ;
    0         5        ;
    45       90        ;
    -30     120        ;
];
labels = {
    '\theta_2=90° (optimal,  w=max)'
    '\theta_2=45° (moderate)'
    '\theta_2=20° (near singular)'
    '\theta_2=5°  (very near singular)'
    '\theta_1=45°, \theta_2=90°'
    '\theta_1=-30°, \theta_2=120°'
};

figure('Name', 'Manipulability Ellipse - 6 Configurations', ...
       'Color', 'white', 'Position', [50 50 1200 700]);

for i = 1:6
    subplot(2, 3, i);
    t1 = deg2rad(configs(i,1));
    t2 = deg2rad(configs(i,2));

    plot_jacobian_ellipse(t1, t2, L1, L2, ...
        'scale',    0.28, ...
        'draw_arm', true, ...
        'color',    ellipse_color(i), ...
        'alpha',    0.35, ...
        'label',    true, ...
        'hold_on',  false);

    title(labels{i}, 'FontSize', 9, 'FontWeight', 'bold');
end
sgtitle('Velocity Manipulability Ellipse - Key Configurations', ...
        'FontSize', 13, 'FontWeight', 'bold');

% Figure 2: w and singular values vs t2 (t1 fixed)
t2_sweep  = linspace(5, 175, 35);           % degrees, avoid exact 0/180
w_vals    = zeros(1, length(t2_sweep));
a_vals    = zeros(1, length(t2_sweep));
b_vals    = zeros(1, length(t2_sweep));

for k = 1:length(t2_sweep)
    t2k = deg2rad(t2_sweep(k));
    [a_vals(k), b_vals(k), ~, w_vals(k)] = ellipse_math(0, t2k, L1, L2);
end

figure('Name', 'Manipulability vs theta2', ...
       'Color', 'white', 'Position', [100 400 900 380]);

subplot(1,2,1);
bar(t2_sweep, w_vals, 'FaceColor', [0.2 0.6 1.0], 'EdgeColor', 'none');
hold on;
plot([0 180], [L1*L2 L1*L2], 'r--', 'LineWidth', 1.5);
xlabel('\theta_2 (degrees)');
ylabel('w = |det(J)|');
title('Manipulability w vs \theta_2  (t_1=0°)');
legend('w', sprintf('max = L_1L_2 = %.2f', L1*L2), 'Location', 'north');
xlim([0 180]); grid on;

subplot(1,2,2);
plot(t2_sweep, a_vals, 'b-o', 'LineWidth', 2, 'MarkerSize', 4); hold on;
plot(t2_sweep, b_vals, 'r-s', 'LineWidth', 2, 'MarkerSize', 4);
xlabel('\theta_2 (degrees)');
ylabel('Singular value (m)');
title('Semi-axes a (major) and b (minor) vs \theta_2');
legend('a (major axis)', 'b (minor axis)', 'Location', 'northeast');
xlim([0 180]); grid on;

sgtitle('How the ellipse changes with elbow angle', ...
        'FontSize', 12, 'FontWeight', 'bold');

% Figure 3: animation while t2 sweeps
figure('Name', 'Ellipse Animation - elbow sweeping', ...
       'Color', 'white', 'Position', [700 50 560 520]);

t1_fixed = deg2rad(20);
t2_anim  = [linspace(10, 170, 60), linspace(170, 10, 60)];

fprintf('\nFigure 3: Animation running (sweeps elbow angle 10° -> 170° -> 10°)...\n');
fprintf('Close the figure window to stop.\n\n');

for k = 1:length(t2_anim)
    if ~ishandle(gcf), break; end

    t2k = deg2rad(t2_anim(k));
    [~, w_k] = custom_jacobian(t1_fixed, t2k, L1, L2);

    plot_jacobian_ellipse(t1_fixed, t2k, L1, L2, ...
        'scale',    0.30, ...
        'draw_arm', true, ...
        'color',    w_color(w_k, L1*L2), ...
        'alpha',    0.40, ...
        'label',    true, ...
        'hold_on',  false);

    % Manipulability bar
    hold on;
    fill([-1.8 -1.8+3.6*(w_k/(L1*L2)) -1.8+3.6*(w_k/(L1*L2)) -1.8], ...
         [-1.75 -1.75 -1.65 -1.65], [0.2 0.8 0.2], 'EdgeColor', 'none');
        patch('XData', [-1.8 1.8 1.8 -1.8], 'YData', [-1.75 -1.75 -1.65 -1.65], ...
            'FaceColor', 'none', 'EdgeColor', [0.4 0.4 0.4], 'LineWidth', 1);
    text(0, -1.60, sprintf('w = %.4f / %.4f', w_k, L1*L2), ...
         'HorizontalAlignment', 'center', 'FontSize', 9, 'Color', [0.1 0.4 0.1]);
    text(0, -1.83, sprintf('\\theta_2 = %.0f°', t2_anim(k)), ...
         'HorizontalAlignment', 'center', 'FontSize', 10, 'Color', [0.2 0.2 0.6]);
    hold off;

    drawnow;
    pause(0.04);
end

fprintf('Animation complete.\n');
fprintf('\n--- HOW TO USE plot_jacobian_ellipse MANUALLY ---\n');
fprintf('  plot_jacobian_ellipse(theta1, theta2, L1, L2)\n');
fprintf('  plot_jacobian_ellipse(pi/4, pi/2, 1.0, 0.8)\n');
fprintf('  plot_jacobian_ellipse(pi/4, pi/2, 1.0, 0.8, ''scale'', 0.4)\n');
fprintf('  [a,b,ang,w] = plot_jacobian_ellipse(pi/4, pi/2, 1.0, 0.8)\n\n');

% Helper colors
function c = ellipse_color(idx)
    palette = [0.20 0.60 1.00;   % blue
               0.20 0.80 0.40;   % green
               1.00 0.70 0.10;   % amber
               1.00 0.30 0.10;   % orange-red
               0.60 0.20 0.80;   % purple
               0.10 0.70 0.70];  % teal
    c = palette(mod(idx-1, size(palette,1))+1, :);
end

function c = w_color(w, w_max)
    % high w -> greener, low w -> redder
    t = max(0, min(1, w / w_max));
    c = [(1-t)*0.9 + t*0.1,  t*0.8 + (1-t)*0.1,  0.1];
end