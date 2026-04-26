function traj = custom_pick_place(pick, place, L1, L2, options)
% CUSTOM_PICK_PLACE  Pick and place trajectory planner for a 2-DOF planar arm
%
%   traj = custom_pick_place(pick, place, L1, L2)
%   traj = custom_pick_place(pick, place, L1, L2, options)
%
%   pick and place are [x, y] target positions in meters.
%   options fields: hover_height, home, n_steps, elbow_up.

    % Defaults
    if nargin < 5, options = struct(); end

    hover  = get_opt(options, 'hover_height', 0.3);
    home   = get_opt(options, 'home',         [0.6, 0.0]);
    n      = get_opt(options, 'n_steps',      30);
    eup    = get_opt(options, 'elbow_up',     1);

    % Key waypoints
    p_home        = home(:)';
    p_above_pick  = [pick(1),  pick(2)  + hover];
    p_pick        = [pick(1),  pick(2) ];
    p_above_place = [place(1), place(2) + hover];
    p_place       = [place(1), place(2)];

    % Segment endpoints
    seg_start = [p_home;        p_above_pick;  p_pick;        p_above_pick; ...
                 p_above_place; p_place;       p_above_place];
    seg_end   = [p_above_pick;  p_pick;        p_above_pick;  p_above_place; ...
                 p_place;       p_above_place; p_home];

    phase_labels = {
        'Home → Above Pick'
        'Above Pick → Pick'
        'Pick → Above Pick'
        'Above Pick → Above Place'
        'Above Place → Place'
        'Place → Above Place'
        'Above Place → Home'
    };

    % Output arrays
    total_pts = 7 * n;
    x_out  = zeros(total_pts, 1);
    y_out  = zeros(total_pts, 1);
    t1_out = zeros(total_pts, 1);
    t2_out = zeros(total_pts, 1);
    w_out  = zeros(total_pts, 1);
    ph_out = zeros(total_pts, 1);

    n_failed = 0;
    nr_fail_count = 0;
    cf_recovery_count = 0;
    both_fail_count = 0;

    % IK options
    ik_opts.init     = 'closedform';
    ik_opts.tol      = 1e-8;
    ik_opts.max_iter = 50;
    ik_opts.elbow_up = eup;

    % Seed the solver with the home position
    [t1_prev, t2_prev] = custom_IK_2DOF(p_home(1), p_home(2), L1, L2, eup);

    % Walk through the segments without duplicating junction points
    for seg = 1:7
        xs = seg_start(seg, 1);  ys = seg_start(seg, 2);
        xe = seg_end(seg,   1);  ye = seg_end(seg,   2);

        % Alpha grid
        alpha_all = linspace(0, 1, n + 1);
        if seg == 1
            alphas = alpha_all(1:n);      % [0 .. (n-1)/n]  — endpoint belongs to seg 2
        else
            alphas = alpha_all(2:n+1);    % [1/n .. 1]      — includes exact endpoint
        end

        for k = 1:length(alphas)
            a   = alphas(k);
            xw  = xs + a*(xe - xs);
            yw  = ys + a*(ye - ys);

            % Solve IK with Newton, then fall back if needed
            ik_opts.init = 'closedform';
            [t1, t2, ok_nr] = custom_IK_newton(xw, yw, L1, L2, ik_opts);
            ok = ok_nr;

            if ~ok_nr
                nr_fail_count = nr_fail_count + 1;

                % Fallback to closed-form IK
                [t1, t2, ok_cf] = custom_IK_2DOF(xw, yw, L1, L2, eup);
                ok = ok_cf;

                if ok_cf
                    cf_recovery_count = cf_recovery_count + 1;
                else
                    % Reuse the previous angles if both solvers fail
                    t1 = t1_prev;
                    t2 = t2_prev;
                    n_failed = n_failed + 1;
                    both_fail_count = both_fail_count + 1;
                end
            end

            % Manipulability
            [~, w] = custom_jacobian(t1, t2, L1, L2);

            % Store waypoint
            idx = (seg-1)*n + k;
            x_out(idx)  = xw;
            y_out(idx)  = yw;
            t1_out(idx) = t1;
            t2_out(idx) = t2;
            w_out(idx)  = w;
            ph_out(idx) = seg;

            % Warm start for the next waypoint
            t1_prev = t1;
            t2_prev = t2;
        end
    end

    % Pack output struct
    traj.x            = x_out;
    traj.y            = y_out;
    traj.theta1       = t1_out;
    traj.theta2       = t2_out;
    traj.w            = w_out;
    traj.phase        = ph_out;
    traj.phase_labels = phase_labels;
    traj.valid        = (n_failed == 0);
    traj.n_failed     = n_failed;
    traj.nr_fail_count = nr_fail_count;
    traj.cf_recovery_count = cf_recovery_count;
    traj.both_fail_count = both_fail_count;
    traj.pick         = pick(:)';
    traj.place        = place(:)';
    traj.home         = home(:)';
    traj.hover_height = hover;
    traj.n_steps      = n;
    traj.total_pts    = total_pts;
end

% Local helper
function val = get_opt(s, field, default)
    if isfield(s, field)
        val = s.(field);
    else
        val = default;
    end
end