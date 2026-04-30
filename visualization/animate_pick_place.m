function animate_pick_place(traj, L1, L2, options)
% ANIMATE_PICK_PLACE  Animate a pick & place trajectory with live ellipse + w bar
%
%   USAGE:
%     animate_pick_place(traj, L1, L2)
%     animate_pick_place(traj, L1, L2, opts)
%
%   INPUTS:
%     traj     output struct from custom_pick_place()
%     L1, L2   link lengths (m)
%     options  struct:
%       .pause_s      pause between frames (default: 0.03 s)
%       .show_ellipse true/false - draw manipulability ellipse (default: true)
%       .show_path    true/false - draw EE path trace (default: true)
%       .show_w_bar   true/false - draw w bar at bottom (default: true)
%       .ellipse_scale scale factor for ellipse (default: 0.18)
%       .show_gripper  true/false - draw gripper open/close indicator (default: true)
%
%   WHAT YOU SEE:
%     Main axes:
%       Blue link 1, red link 2, joints drawn per frame
%       Green trace of EE path so far
%       Pick target (red X), Place target (blue X), Home (black square)
%       Velocity ellipse at EE (shrinks near singularity)
%       Phase label top-left
%     Bottom bar:  w(t) / w_max - green=high dexterity, red=low
%     Gripper indicator: OPEN (yellow) during transit, CLOSED (red) at pick/place

    if nargin < 4, options = struct(); end

    dt          = get_opt(options, 'pause_s',      0.03);
    show_ell    = get_opt(options, 'show_ellipse', true);
    show_path   = get_opt(options, 'show_path',    true);
    show_wbar   = get_opt(options, 'show_w_bar',   true);
    ell_scale   = get_opt(options, 'ellipse_scale',0.18);
    show_grip   = get_opt(options, 'show_gripper', true);

    w_max = L1 * L2;   % theoretical maximum manipulability
    N     = traj.total_pts;

    % Phase colour map - one colour per phase
    phase_colors = [
        0.20 0.60 1.00;   % 1 home→above_pick   blue
        0.10 0.80 0.30;   % 2 above_pick→pick    green
        0.80 0.50 0.10;   % 3 pick→above_pick    amber
        0.70 0.20 0.80;   % 4 transit            purple
        0.10 0.70 0.70;   % 5 above_place→place  teal
        0.90 0.40 0.10;   % 6 place→above_place  orange
        0.40 0.40 0.90;   % 7 above_place→home   indigo
    ];

    % Figure layout
    figure('Name','Pick & Place Animation','Color','white', ...
           'Position',[60 60 820 680]);

    ax_main = axes('Position',[0.08 0.22 0.88 0.72]);
    ax_wbar = axes('Position',[0.08 0.06 0.88 0.10]);

    % Manipulability bar
    axes(ax_wbar);
    fill([0 1 1 0],[0 0 1 1],[0.92 0.92 0.92],'EdgeColor','none'); hold on;
    plot([0 1],[0.5 0.5],'k--','LineWidth',0.5);
    xlim([0 1]); ylim([0 1]);
    axis off;
    w_patch = fill([0 0 0 0],[0 0 1 1],[0.2 0.8 0.2],'EdgeColor','none');
    text(0.5,-0.5,'Manipulability  w / w_{max}','HorizontalAlignment','center', ...
         'FontSize',9,'Units','data');

    % Main axes
    axes(ax_main);
    lim = (L1+L2)*1.15;
    xlim([-lim lim]); ylim([-lim lim]);
    axis equal; grid on;
    xlabel('X (m)'); ylabel('Y (m)');

    % Workspace boundary circles
    th = linspace(0,2*pi,300);
    plot((L1+L2)*cos(th),(L1+L2)*sin(th),'--','Color',[0.8 0.8 0.8],'LineWidth',1);
    hold on;
    r_min = abs(L1-L2);
    if r_min > 0.01
        plot(r_min*cos(th),r_min*sin(th),'--','Color',[0.85 0.85 0.85],'LineWidth',1);
    end

    % Fixed markers
    plot(traj.pick(1),  traj.pick(2),  'rx','MarkerSize',16,'LineWidth',3);
    plot(traj.place(1), traj.place(2), 'bx','MarkerSize',16,'LineWidth',3);
    plot(traj.home(1),  traj.home(2),  'ks','MarkerSize',10,'MarkerFaceColor','k');

    % Hover lines
    plot([traj.pick(1)  traj.pick(1)], ...
         [traj.pick(2)  traj.pick(2)+traj.hover_height], ...
         'r:','LineWidth',1.2);
    plot([traj.place(1) traj.place(1)], ...
         [traj.place(2) traj.place(2)+traj.hover_height], ...
         'b:','LineWidth',1.2);

    % Labels
    text(traj.pick(1)+0.05,  traj.pick(2)-0.12,  'PICK',  'Color','r','FontSize',9,'FontWeight','bold');
    text(traj.place(1)+0.05, traj.place(2)-0.12, 'PLACE', 'Color','b','FontSize',9,'FontWeight','bold');
    text(traj.home(1)+0.05,  traj.home(2)-0.12,  'HOME',  'Color','k','FontSize',9);

    % Animated handles
    h_link1   = plot([0 0],[0 0],'-','Color',[0.15 0.35 0.85],'LineWidth',5);
    h_link2   = plot([0 0],[0 0],'-','Color',[0.85 0.20 0.20],'LineWidth',5);
    h_base    = plot(0,0,'ks','MarkerSize',13,'MarkerFaceColor','k');
    h_elbow   = plot(0,0,'ko','MarkerSize',10,'MarkerFaceColor',[0.4 0.4 0.4]);
    h_ee      = plot(0,0,'o','MarkerSize',11,'MarkerFaceColor',[0.1 0.85 0.1],'MarkerEdgeColor','k');
    h_path    = plot(NaN,NaN,'-','LineWidth',2,'Color',[0.1 0.7 0.1]);
    h_ellipse = plot(NaN,NaN,'-','LineWidth',1.5,'Color',[0.3 0.3 0.9]);
    h_ell_fill= fill(NaN,NaN,[0.5 0.6 1.0],'FaceAlpha',0.25,'EdgeColor','none');
    h_phase   = text(-lim*0.95, lim*0.90,'','FontSize',10,'FontWeight','bold','Color',[0.2 0.2 0.6]);
    h_wtext   = text(-lim*0.95, lim*0.78,'','FontSize',9,'Color',[0.1 0.5 0.1]);

    % Gripper indicator
    if show_grip
        grip_box = fill([lim*0.60 lim*0.98 lim*0.98 lim*0.60], ...
                        [lim*0.82 lim*0.82 lim*0.98 lim*0.98], ...
                        [1 1 0.6],'EdgeColor',[0.5 0.5 0.5],'LineWidth',1.2);
        grip_txt = text(lim*0.79, lim*0.90,'OPEN','HorizontalAlignment','center', ...
                        'FontSize',10,'FontWeight','bold','Color',[0.4 0.4 0.0]);
    end

    % Gripper state per phase
    gripper_state = {'OPEN','CLOSING','HOLDING','HOLDING','RELEASE','OPEN','OPEN'};
    gripper_color = {[1 1 0.5],[1 0.6 0.1],[0.9 0.2 0.2], ...
                     [0.9 0.2 0.2],[0.2 0.8 0.2],[0.7 1.0 0.7],[0.7 1.0 0.7]};

    % Animation loop
    path_x = nan(N,1);
    path_y = nan(N,1);

    for i = 1:N
        if ~ishandle(ax_main), break; end

        t1  = traj.theta1(i);
        t2  = traj.theta2(i);
        w_i = traj.w(i);
        ph  = traj.phase(i);

        % FK for joint positions
        x1  = L1*cos(t1);
        y1  = L1*sin(t1);
        xee = x1 + L2*cos(t1+t2);
        yee = y1 + L2*sin(t1+t2);

        % Update arm links
        set(h_link1, 'XData',[0 x1],   'YData',[0 y1]);
        set(h_link2, 'XData',[x1 xee], 'YData',[y1 yee]);
        set(h_elbow, 'XData',x1,  'YData',y1);
        set(h_ee,    'XData',xee, 'YData',yee);

        % EE path trace
        if show_path
            path_x(i) = xee;
            path_y(i) = yee;
            set(h_path,'XData',path_x(1:i),'YData',path_y(1:i), ...
                'Color',phase_colors(ph,:));
        end

        % Manipulability ellipse
        if show_ell
            s1e=sin(t1); c1e=cos(t1); s12=sin(t1+t2); c12=cos(t1+t2);
            J   = [-L1*s1e-L2*s12, -L2*s12; L1*c1e+L2*c12, L2*c12];
            [U,S,~] = svd(J);
            phi = linspace(0,2*pi,80);
            e   = ell_scale * U * S * [cos(phi); sin(phi)];
            ex  = xee + e(1,:);
            ey  = yee + e(2,:);
            set(h_ellipse,  'XData',ex,'YData',ey,'Color',phase_colors(ph,:));
            set(h_ell_fill, 'XData',ex,'YData',ey,'FaceColor',phase_colors(ph,:));
        end

        % Phase label
        set(h_phase,'String', sprintf('Phase %d: %s', ph, traj.phase_labels{ph}), ...
                    'Color', phase_colors(ph,:));

        % w text
        set(h_wtext,'String', sprintf('w = %.4f  (%.0f%% of max)', ...
                    w_i, 100*w_i/w_max));

        % w bar
        if show_wbar
            axes(ax_wbar);
            frac = min(1, w_i/w_max);
            r_c  = (1-frac)*0.9;
            g_c  = frac*0.8;
            set(w_patch,'XData',[0 frac frac 0]);
            set(w_patch,'FaceColor',[r_c g_c 0.1]);
            axes(ax_main);
        end

        % Gripper
        if show_grip
            set(grip_box,'FaceColor',gripper_color{ph});
            set(grip_txt,'String',gripper_state{ph});
        end

        drawnow;
        pause(dt);
    end

    % Final frame hold
    set(h_phase,'String','COMPLETE - arm at home','Color',[0.1 0.5 0.1]);
    drawnow;
end

% Local helper
function val = get_opt(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end