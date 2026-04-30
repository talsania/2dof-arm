%% Demo pick and place
%  Figures show the trajectory, phase snapshots, and animation.
here     = fileparts(mfilename('fullpath'));
root_dir = here;
addpath(fullfile(root_dir, 'kinematics'));
addpath(fullfile(root_dir, 'visualization'));

% Parameters
pick_pt  = [0.8,  0.2];     % object to pick  (red X)
place_pt = [-0.7, 0.3];     % drop location   (blue X)

opts_pp.hover_height = 0.30;
opts_pp.home         = [0.6, 0.0];
opts_pp.n_steps      = 40;  % 7 segs → 280 total
opts_pp.elbow_up     = 1;

fprintf('\nPick & Place Demo\n');
fprintf('  pick  = (%.2f, %.2f)\n', pick_pt(1),  pick_pt(2));
fprintf('  place = (%.2f, %.2f)\n', place_pt(1), place_pt(2));
fprintf('  home  = (%.2f, %.2f)\n', opts_pp.home(1), opts_pp.home(2));
fprintf('  hover = %.2f m\n\n', opts_pp.hover_height);

% Plan trajectory
traj = custom_pick_place(pick_pt, place_pt, L1, L2, opts_pp);

fprintf(['Trajectory: %d waypoints  |  valid=%d  |  n_failed=%d\n' ...
     '  nr_fail_count=%d  |  cf_recovery_count=%d  |  both_fail_count=%d\n\n'], ...
    traj.total_pts, traj.valid, traj.n_failed, ...
    traj.nr_fail_count, traj.cf_recovery_count, traj.both_fail_count);

% Phase color map
phase_colors = [
    0.20 0.60 1.00;   % 1  home → above_pick
    0.10 0.80 0.30;   % 2  above_pick → pick
    0.80 0.50 0.10;   % 3  pick → above_pick
    0.70 0.20 0.80;   % 4  transit
    0.10 0.70 0.70;   % 5  above_place → place
    0.90 0.40 0.10;   % 6  place → above_place
    0.40 0.40 0.90;   % 7  above_place → home
];

figure('Name','Pick & Place - Trajectory Overview', ...
       'Color','white','Position',[50 50 1000 620]);

% Cartesian path by phase
ax1 = subplot(3,1,[1 2]);
hold on;

% Workspace boundary
th_ws = linspace(0,2*pi,300);
plot((L1+L2)*cos(th_ws),(L1+L2)*sin(th_ws),'--','Color',[0.88 0.88 0.88],'LineWidth',1);
r_min = abs(L1-L2);
plot(r_min*cos(th_ws), r_min*sin(th_ws), '--','Color',[0.88 0.88 0.88],'LineWidth',1);

% Draw each phase segment
for p = 1:7
    idx = find(traj.phase == p);
    plot(traj.x(idx), traj.y(idx), '-', 'Color', phase_colors(p,:), 'LineWidth', 2.5);
end

% Markers
plot(pick_pt(1),  pick_pt(2),  'rx','MarkerSize',16,'LineWidth',3);
plot(place_pt(1), place_pt(2), 'bx','MarkerSize',16,'LineWidth',3);
plot(traj.home(1),traj.home(2),'ks','MarkerSize',10,'MarkerFaceColor','k');

% Hover lines
plot([pick_pt(1)  pick_pt(1)],  [pick_pt(2)  pick_pt(2)+traj.hover_height],  'r:','LineWidth',1.5);
plot([place_pt(1) place_pt(1)], [place_pt(2) place_pt(2)+traj.hover_height], 'b:','LineWidth',1.5);

% Phase labels
for p = 1:7
    idx = find(traj.phase == p);
    mid = round(length(idx)/2);
    text(traj.x(idx(mid))+0.03, traj.y(idx(mid))+0.04, ...
         sprintf('%d',p), 'Color', phase_colors(p,:)*0.7, ...
         'FontSize', 8, 'FontWeight', 'bold');
end

text(pick_pt(1)+0.05,  pick_pt(2)-0.12,  'PICK',  'Color','r','FontSize',9,'FontWeight','bold');
text(place_pt(1)+0.05, place_pt(2)-0.12, 'PLACE', 'Color','b','FontSize',9,'FontWeight','bold');
text(traj.home(1)+0.05,traj.home(2)-0.12,'HOME',  'Color','k','FontSize',9);

lim = (L1+L2)*1.1;
xlim([-lim lim]); ylim([-lim lim]);
axis equal; grid on;
xlabel('X (m)'); ylabel('Y (m)');
title('Pick & Place Cartesian Path - 7 Phases (numbered)','FontSize',11,'FontWeight','bold');

% Manipulability strip
ax2 = subplot(3,1,3);
hold on;

for p = 1:7
    idx = find(traj.phase == p);
    plot(idx, traj.w(idx), '-', 'Color', phase_colors(p,:), 'LineWidth', 2);
end

yline(L1*L2, 'k--', 'LineWidth', 1, 'Label', 'w_{max}=L_1L_2');

% Phase boundaries
for p = 1:6
    idx_end = find(traj.phase == p, 1, 'last');
    xline(idx_end, ':', 'Color', [0.6 0.6 0.6], 'LineWidth', 1);
end

ylim([0 L1*L2*1.15]);
xlim([1 traj.total_pts]);
xlabel('Waypoint index'); ylabel('w = |det(J)|');
title('Manipulability w along trajectory','FontSize',10);
grid on;

sgtitle('Pick & Place - Full Trajectory Overview','FontSize',13,'FontWeight','bold');

figure('Name','Pick & Place - Phase Snapshots', ...
       'Color','white','Position',[100 50 1400 380]);

short_labels = {'Home→AbovePick','AbovePick→Pick','Pick→AbovePick', ...
                'AbovePick→AbovePlace','AbovePlace→Place', ...
                'Place→AbovePlace','AbovePlace→Home'};

for p = 1:7
    subplot(1,7,p);
    idx = find(traj.phase == p);
    mid = round(length(idx)/2);
    i_mid = idx(mid);

    t1m = traj.theta1(i_mid);
    t2m = traj.theta2(i_mid);
    x1m = L1*cos(t1m); y1m = L1*sin(t1m);
    xem = x1m + L2*cos(t1m+t2m);
    yem = y1m + L2*sin(t1m+t2m);

    % Arm links
    plot([0 x1m],[0 y1m],'-','Color',[0.2 0.3 0.9],'LineWidth',4); hold on;
    plot([x1m xem],[y1m yem],'-','Color',[0.9 0.2 0.2],'LineWidth',4);
    plot(0,0,'ks','MarkerSize',10,'MarkerFaceColor','k');
    plot(x1m,y1m,'ko','MarkerSize',8,'MarkerFaceColor',[0.4 0.4 0.4]);
    plot(xem,yem,'o','MarkerSize',9,'MarkerFaceColor',[0.1 0.85 0.1],'MarkerEdgeColor','k');

    % Small ellipse
    s1e=sin(t1m); c1e=cos(t1m); s12=sin(t1m+t2m); c12=cos(t1m+t2m);
    Jm = [-L1*s1e-L2*s12,-L2*s12; L1*c1e+L2*c12,L2*c12];
    [Um,Sm,~] = svd(Jm);
    phi_e = linspace(0,2*pi,60);
    em = 0.15 * Um * Sm * [cos(phi_e); sin(phi_e)];
    fill(xem+em(1,:), yem+em(2,:), phase_colors(p,:), ...
         'FaceAlpha',0.4,'EdgeColor',phase_colors(p,:)*0.6,'LineWidth',1);

    % Pick and place markers
    plot(pick_pt(1),pick_pt(2),'rx','MarkerSize',10,'LineWidth',2);
    plot(place_pt(1),place_pt(2),'bx','MarkerSize',10,'LineWidth',2);

    lim2 = (L1+L2)*1.05;
    xlim([-lim2 lim2]); ylim([-lim2 lim2]);
    axis equal; grid on;
    xticks([]); yticks([]);
    title({sprintf('Phase %d',p), short_labels{p}, ...
           sprintf('w=%.3f',traj.w(i_mid))}, ...
          'FontSize',7,'Color',phase_colors(p,:)*0.7,'FontWeight','bold');
end
sgtitle('Arm Pose at Mid-Point of Each Phase  (ellipse = manipulability)', ...
        'FontSize',11,'FontWeight','bold');

% Live animation
fprintf('Starting animation (Figure 6)...\n');
fprintf('Close the figure to stop early.\n\n');

anim_opts.pause_s       = 0.025;
anim_opts.show_ellipse  = true;
anim_opts.show_path     = true;
anim_opts.show_w_bar    = true;
anim_opts.ellipse_scale = 0.18;
anim_opts.show_gripper  = true;

animate_pick_place(traj, L1, L2, anim_opts);

fprintf('Pick & Place demo complete.\n');
fprintf('\nHow to use these functions\n');
fprintf('  traj = custom_pick_place([0.8 0.2], [-0.7 0.3], 1.0, 0.8)\n');
fprintf('  animate_pick_place(traj, 1.0, 0.8)\n');
fprintf('  traj.valid, traj.n_failed, traj.w  ← inspect trajectory\n');
fprintf('  traj.phase_labels{3}               ← phase name\n\n');