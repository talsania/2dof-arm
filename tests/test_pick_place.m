%% TEST_PICK_PLACE.m
% Unit test suite for custom_pick_place().
%
%   cd tests
%   test_pick_place
%
%   Every assert() must not throw.
%
%   Tolerances:
%     TOL_POS   1e-9 m
%     TOL_FK    1e-7 m
%     TOL_W     1e-12
%     TOL_ANG   1e-9 rad

% Path setup
test_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(test_dir);
addpath(fullfile(root_dir, 'kinematics'));
addpath(fullfile(root_dir, 'visualization'));
addpath(fullfile(root_dir, 'tests'));

clc;
t_start = tic;

fprintf('[==========] test_pick_place.m\n');
fprintf('[----------] Arm: L1=1.0 m  L2=0.8 m  workspace r=[0.2, 1.8] m\n\n');

% Arm constants
L1    = 1.0;
L2    = 0.8;
R_MAX = L1 + L2;
R_MIN = abs(L1 - L2);

% Tolerances
TOL_POS = 1e-9;
TOL_FK  = 1e-7;
TOL_W   = 1e-12;
TOL_ANG = 1e-9;

% Reference inputs
PICK    = [0.8,  0.2];
PLACE   = [-0.7, 0.3];
HOME    = [0.6,  0.0];
HOVER   = 0.3;
N_STEPS = 30;

opts.hover_height = HOVER;
opts.home         = HOME;
opts.n_steps      = N_STEPS;
opts.elbow_up     = 1;

% Compute reference trajectory once
traj = custom_pick_place(PICK, PLACE, L1, L2, opts);

% Counters
n_pass = 0;
n_fail = 0;

% Section 1: Output contract
fprintf('[----------] 1. Output contract\n');

% 1.1  All required fields present
required = {'x','y','theta1','theta2','w','phase','phase_labels', ...
            'valid','n_failed','pick','place','home', ...
            'hover_height','n_steps','total_pts'};
cond = all(cellfun(@(f) isfield(traj,f), required));
try, assert(cond); fprintf('[ OK ]  struct_has_all_required_fields\n'); n_pass=n_pass+1;
catch, fprintf('[FAIL]  struct_has_all_required_fields\n'); n_fail=n_fail+1; end

% 1.2  total_pts == 7 * n_steps
cond = (traj.total_pts == 7*N_STEPS);
try, assert(cond, 'total_pts=%d expected=%d', traj.total_pts, 7*N_STEPS);
     fprintf('[ OK ]  total_pts_equals_7_x_n_steps  (%d)\n', traj.total_pts); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  total_pts_equals_7_x_n_steps  %s\n',ME.message); n_fail=n_fail+1; end

% 1.3  Every array field has length == total_pts
arr_fields = {'x','y','theta1','theta2','w','phase'};
all_len_ok = true;
for fi = 1:length(arr_fields)
    if length(traj.(arr_fields{fi})) ~= traj.total_pts
        all_len_ok = false;
        fprintf('       %s: len=%d expected=%d\n', arr_fields{fi}, ...
                length(traj.(arr_fields{fi})), traj.total_pts);
    end
end
try, assert(all_len_ok);
     fprintf('[ OK ]  all_array_fields_have_length_total_pts\n'); n_pass=n_pass+1;
catch, fprintf('[FAIL]  all_array_fields_have_length_total_pts\n'); n_fail=n_fail+1; end

% 1.4  Exactly 7 phase labels
cond = (length(traj.phase_labels) == 7);
try, assert(cond,'len=%d',length(traj.phase_labels));
     fprintf('[ OK ]  phase_labels_has_7_entries\n'); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  phase_labels_has_7_entries  %s\n',ME.message); n_fail=n_fail+1; end

% 1.5  Stored scalars match inputs
try, assert(norm(traj.pick  - PICK)  < TOL_POS);
     assert(norm(traj.place - PLACE) < TOL_POS);
     assert(norm(traj.home  - HOME)  < TOL_POS);
     assert(abs(traj.hover_height - HOVER) < TOL_POS);
     fprintf('[ OK ]  stored_inputs_match_pick_place_home_hover\n'); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  stored_inputs_match_pick_place_home_hover  %s\n',ME.message); n_fail=n_fail+1; end

fprintf('\n');

% Section 2: Phase structure
fprintf('[----------] 2. Phase structure\n');

% 2.1  Phase values are exactly 1:7
cond = isequal(unique(traj.phase)', 1:7);
try, assert(cond);
     fprintf('[ OK ]  phase_values_are_1_through_7\n'); n_pass=n_pass+1;
catch, fprintf('[FAIL]  phase_values_are_1_through_7\n'); n_fail=n_fail+1; end

% 2.2  Each phase has exactly n_steps waypoints
cnt_ok = true;
for p = 1:7
    if sum(traj.phase==p) ~= N_STEPS
        cnt_ok = false;
        fprintf('       phase %d: %d pts (expected %d)\n',p,sum(traj.phase==p),N_STEPS);
    end
end
try, assert(cnt_ok);
     fprintf('[ OK ]  each_phase_has_n_steps_waypoints  (%d)\n',N_STEPS); n_pass=n_pass+1;
catch, fprintf('[FAIL]  each_phase_has_n_steps_waypoints\n'); n_fail=n_fail+1; end

% 2.3  Phase indices non-decreasing (no backward jumps)
cond = all(diff(traj.phase) >= 0);
try, assert(cond);
     fprintf('[ OK ]  phase_sequence_monotone_non_decreasing\n'); n_pass=n_pass+1;
catch, fprintf('[FAIL]  phase_sequence_monotone_non_decreasing\n'); n_fail=n_fail+1; end

fprintf('\n');

% Section 3: Key waypoints
fprintf('[----------] 3. Key waypoints\n');

idx = @(p) find(traj.phase == p);
p2 = idx(2); p3 = idx(3); p4 = idx(4); p5 = idx(5);

% 3.1  Start at home
e = norm([traj.x(1)-HOME(1); traj.y(1)-HOME(2)]);
try, assert(e < TOL_POS,'err=%.2e m',e);
     fprintf('[ OK ]  trajectory_starts_at_home\n'); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  trajectory_starts_at_home  %s\n',ME.message); n_fail=n_fail+1; end

% 3.2  End at home
e = norm([traj.x(end)-HOME(1); traj.y(end)-HOME(2)]);
try, assert(e < TOL_POS,'err=%.2e m',e);
     fprintf('[ OK ]  trajectory_ends_at_home\n'); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  trajectory_ends_at_home  %s\n',ME.message); n_fail=n_fail+1; end

% 3.3  Pick position at end of phase 2
e = norm([traj.x(p2(end))-PICK(1); traj.y(p2(end))-PICK(2)]);
try, assert(e < TOL_POS,'err=%.2e m  EE=(%.4f,%.4f)',e,traj.x(p2(end)),traj.y(p2(end)));
     fprintf('[ OK ]  pick_position_exact_at_end_of_phase2  (err=%.2e m)\n',e); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  pick_position_exact_at_end_of_phase2  %s\n',ME.message); n_fail=n_fail+1; end

% 3.4  Place position at end of phase 5
e = norm([traj.x(p5(end))-PLACE(1); traj.y(p5(end))-PLACE(2)]);
try, assert(e < TOL_POS,'err=%.2e m  EE=(%.4f,%.4f)',e,traj.x(p5(end)),traj.y(p5(end)));
     fprintf('[ OK ]  place_position_exact_at_end_of_phase5  (err=%.2e m)\n',e); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  place_position_exact_at_end_of_phase5  %s\n',ME.message); n_fail=n_fail+1; end

% 3.5  Above-pick y == pick_y + hover  (end of phase 3)
e = abs(traj.y(p3(end)) - (PICK(2) + HOVER));
try, assert(e < TOL_POS,'err=%.2e m  y=%.4f want=%.4f',e,traj.y(p3(end)),PICK(2)+HOVER);
     fprintf('[ OK ]  above_pick_hover_height_correct  (err=%.2e m)\n',e); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  above_pick_hover_height_correct  %s\n',ME.message); n_fail=n_fail+1; end

% 3.6  Above-place y == place_y + hover  (end of phase 4)
e = abs(traj.y(p4(end)) - (PLACE(2) + HOVER));
try, assert(e < TOL_POS,'err=%.2e m  y=%.4f want=%.4f',e,traj.y(p4(end)),PLACE(2)+HOVER);
     fprintf('[ OK ]  above_place_hover_height_correct  (err=%.2e m)\n',e); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  above_place_hover_height_correct  %s\n',ME.message); n_fail=n_fail+1; end

fprintf('\n');

% Section 4: Cartesian geometry
fprintf('[----------] 4. Cartesian geometry\n');

% 4.1  Each phase is a straight line (collinearity via cross product)
straight_ok = true;
for p = 1:7
    ip = idx(p);
    if length(ip) < 3, continue; end
    dx = traj.x(ip(end)) - traj.x(ip(1));
    dy = traj.y(ip(end)) - traj.y(ip(1));
    for k = 2:length(ip)-1
        cv = abs(dx*(traj.y(ip(k))-traj.y(ip(1))) - dy*(traj.x(ip(k))-traj.x(ip(1))));
        if cv > 1e-9, straight_ok=false; break; end
    end
end
try, assert(straight_ok);
     fprintf('[ OK ]  all_phases_are_straight_lines_in_cartesian\n'); n_pass=n_pass+1;
catch, fprintf('[FAIL]  all_phases_are_straight_lines_in_cartesian\n'); n_fail=n_fail+1; end

% 4.2  Uniform step spacing within each phase
uniform_ok = true;
for p = 1:7
    ip = idx(p);
    if length(ip) < 3, continue; end
    step_d = sqrt(diff(traj.x(ip)).^2 + diff(traj.y(ip)).^2);
    if max(step_d)-min(step_d) > 1e-9
        uniform_ok = false;
        fprintf('       phase %d spacing spread=%.2e\n',p,max(step_d)-min(step_d));
    end
end
try, assert(uniform_ok);
     fprintf('[ OK ]  waypoint_spacing_uniform_within_each_phase\n'); n_pass=n_pass+1;
catch, fprintf('[FAIL]  waypoint_spacing_uniform_within_each_phase\n'); n_fail=n_fail+1; end

% 4.3  All waypoints inside workspace
r_vals = sqrt(traj.x.^2 + traj.y.^2);
cond = all(r_vals >= R_MIN-1e-6 & r_vals <= R_MAX+1e-6);
try, assert(cond,'r range [%.4f,%.4f] limits [%.4f,%.4f]',min(r_vals),max(r_vals),R_MIN,R_MAX);
     fprintf('[ OK ]  all_waypoints_inside_workspace  r=[%.4f,%.4f]\n',min(r_vals),max(r_vals)); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  all_waypoints_inside_workspace  %s\n',ME.message); n_fail=n_fail+1; end

fprintf('\n');

% Section 5: IK/FK consistency
fprintf('[----------] 5. IK/FK consistency\n');

% 5.1  FK(theta) == stored (x,y) at every waypoint
fk_errs = zeros(traj.total_pts,1);
for i = 1:traj.total_pts
    [xv,yv] = custom_FK_2DOF(traj.theta1(i), traj.theta2(i), L1, L2);
    fk_errs(i) = norm([xv-traj.x(i); yv-traj.y(i)]);
end
max_fk = max(fk_errs);
try, assert(max_fk < TOL_FK,'max_err=%.2e m tol=%.0e',max_fk,TOL_FK);
     fprintf('[ OK ]  fk_theta_matches_stored_xy_all_waypoints  (max=%.2e m)\n',max_fk); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  fk_theta_matches_stored_xy_all_waypoints  %s\n',ME.message); n_fail=n_fail+1; end

% 5.2  valid==true and n_failed==0 for standard case
try, assert(traj.valid && traj.n_failed==0,'valid=%d n_failed=%d',traj.valid,traj.n_failed);
     fprintf('[ OK ]  trajectory_valid_and_no_ik_failures\n'); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  trajectory_valid_and_no_ik_failures  %s\n',ME.message); n_fail=n_fail+1; end

% 5.3  All angles in [-pi, pi]
t1_ok = all(traj.theta1 >= -pi-TOL_ANG & traj.theta1 <= pi+TOL_ANG);
t2_ok = all(traj.theta2 >= -pi-TOL_ANG & traj.theta2 <= pi+TOL_ANG);
try, assert(t1_ok && t2_ok);
     fprintf('[ OK ]  joint_angles_bounded_within_minus_pi_to_pi\n'); n_pass=n_pass+1;
catch, fprintf('[FAIL]  joint_angles_bounded_within_minus_pi_to_pi\n'); n_fail=n_fail+1; end

% 5.4  IK at pick matches custom_IK_2DOF (closed-form ground truth)
[t1_cf,t2_cf,cf_ok] = custom_IK_2DOF(PICK(1),PICK(2),L1,L2,1);
if cf_ok
    [x_cf,y_cf]   = custom_FK_2DOF(t1_cf,t2_cf,L1,L2);
    [x_nr,y_nr]   = custom_FK_2DOF(traj.theta1(p2(end)),traj.theta2(p2(end)),L1,L2);
    err_cf = norm([x_nr-x_cf; y_nr-y_cf]);
    try, assert(err_cf < TOL_POS,'FK diff=%.2e m',err_cf);
         fprintf('[ OK ]  ik_at_pick_matches_closedform_ground_truth  (err=%.2e m)\n',err_cf); n_pass=n_pass+1;
    catch ME, fprintf('[FAIL]  ik_at_pick_matches_closedform_ground_truth  %s\n',ME.message); n_fail=n_fail+1; end
end

fprintf('\n');

% Section 6: Manipulability
fprintf('[----------] 6. Manipulability\n');

% 6.1  w in [0, L1*L2] everywhere
W_MAX = L1*L2;
cond = all(traj.w >= -1e-12 & traj.w <= W_MAX+1e-12);
try, assert(cond,'w range [%.5f,%.5f] limit [0,%.5f]',min(traj.w),max(traj.w),W_MAX);
     fprintf('[ OK ]  w_in_valid_range_0_to_L1L2  w=[%.4f,%.4f]\n',min(traj.w),max(traj.w)); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  w_in_valid_range_0_to_L1L2  %s\n',ME.message); n_fail=n_fail+1; end

% 6.2  Stored w == custom_jacobian w at every waypoint
w_errs = zeros(traj.total_pts,1);
for i = 1:traj.total_pts
    [~,wc] = custom_jacobian(traj.theta1(i),traj.theta2(i),L1,L2);
    w_errs(i) = abs(traj.w(i)-wc);
end
max_w = max(w_errs);
try, assert(max_w < TOL_W,'max_err=%.2e tol=%.0e',max_w,TOL_W);
     fprintf('[ OK ]  stored_w_matches_jacobian_all_waypoints  (max=%.2e)\n',max_w); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  stored_w_matches_jacobian_all_waypoints  %s\n',ME.message); n_fail=n_fail+1; end

% 6.3  w > 0 at pick and place (not singular at task targets)
w_pick  = traj.w(p2(end));
w_place = traj.w(p5(end));
try, assert(w_pick>0.01 && w_place>0.01,'w_pick=%.4f w_place=%.4f',w_pick,w_place);
     fprintf('[ OK ]  w_positive_at_pick_and_place  (%.4f  %.4f)\n',w_pick,w_place); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  w_positive_at_pick_and_place  %s\n',ME.message); n_fail=n_fail+1; end

fprintf('\n');

% Section 7: Options
fprintf('[----------] 7. Options\n');

% 7.1  hover_height=0.5
o = opts; o.hover_height = 0.5;
tr = custom_pick_place(PICK, PLACE, L1, L2, o);
p3t = find(tr.phase==3); p4t = find(tr.phase==4);
e1 = abs(tr.y(p3t(end))-(PICK(2)+0.5));
e2 = abs(tr.y(p4t(end))-(PLACE(2)+0.5));
try, assert(e1<TOL_POS && e2<TOL_POS,'e_pick=%.2e e_place=%.2e',e1,e2);
     fprintf('[ OK ]  option_hover_height_applied_correctly\n'); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  option_hover_height_applied_correctly  %s\n',ME.message); n_fail=n_fail+1; end

% 7.2  n_steps=20 -> total_pts=140
o = opts; o.n_steps = 20;
tr = custom_pick_place(PICK, PLACE, L1, L2, o);
try, assert(tr.total_pts==140,'total_pts=%d',tr.total_pts);
     fprintf('[ OK ]  option_n_steps_20_gives_total_pts_140\n'); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  option_n_steps_20_gives_total_pts_140  %s\n',ME.message); n_fail=n_fail+1; end

% 7.3  custom home [0.9, 0.1]
NEW_HOME = [0.9, 0.1];
o = opts; o.home = NEW_HOME;
tr = custom_pick_place(PICK, PLACE, L1, L2, o);
es = norm([tr.x(1)-NEW_HOME(1); tr.y(1)-NEW_HOME(2)]);
ee = norm([tr.x(end)-NEW_HOME(1); tr.y(end)-NEW_HOME(2)]);
try, assert(es<TOL_POS && ee<TOL_POS,'err_start=%.2e err_end=%.2e',es,ee);
     fprintf('[ OK ]  option_custom_home_trajectory_starts_and_ends_there\n'); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  option_custom_home_trajectory_starts_and_ends_there  %s\n',ME.message); n_fail=n_fail+1; end

% 7.4  elbow_up=0: different angles, identical EE positions
ou = opts; ou.elbow_up = 1;
od = opts; od.elbow_up = 0;
tru_ = custom_pick_place(PICK, PLACE, L1, L2, ou);
trd_ = custom_pick_place(PICK, PLACE, L1, L2, od);
ee_pos_diff   = max(sqrt((tru_.x-trd_.x).^2+(tru_.y-trd_.y).^2));
angle_diff    = max(abs(tru_.theta2-trd_.theta2));
try, assert(ee_pos_diff<1e-6,'max EE diff=%.2e m',ee_pos_diff);
     assert(angle_diff>0.5,'max t2 diff=%.4f rad',angle_diff);
     fprintf('[ OK ]  option_elbow_down_same_ee_different_angles\n'); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  option_elbow_down_same_ee_different_angles  %s\n',ME.message); n_fail=n_fail+1; end

fprintf('\n');

% Section 8: Multiple pick and place pairs
fprintf('[----------] 8. Multiple pick/place pairs\n');
%
%  Pairs chosen so NO segment crosses the inner workspace hole
%  (r < R_MIN = 0.2 m).  Pairs verified to have min-r > 0.25 m
%  along every segment including the return path.

pairs = {
    [0.8  0.2], [-0.7  0.3];
    [0.9  0.4], [-0.8  0.5];
    [0.5  0.5], [ 0.5 -0.5];
    [0.6  0.4], [-0.6  0.4];
    [0.7  0.3], [-0.5  0.6];
};

for pi_ = 1:size(pairs,1)
    pk = pairs{pi_,1};  pl = pairs{pi_,2};
    tr_i = custom_pick_place(pk, pl, L1, L2, opts);

    % valid
    try, assert(tr_i.valid && tr_i.n_failed==0, ...
                'pick=(%.1f,%.1f) place=(%.1f,%.1f) n_failed=%d', ...
                pk(1),pk(2),pl(1),pl(2),tr_i.n_failed);
         fprintf('[ OK ]  pair_%d_valid  pick=(%.1f,%.1f) place=(%.1f,%.1f)\n', ...
                 pi_,pk(1),pk(2),pl(1),pl(2)); n_pass=n_pass+1;
    catch ME, fprintf('[FAIL]  pair_%d_valid  %s\n',pi_,ME.message); n_fail=n_fail+1; end

    % FK consistency
    max_e=0;
    for i=1:tr_i.total_pts
        [xv,yv]=custom_FK_2DOF(tr_i.theta1(i),tr_i.theta2(i),L1,L2);
        e=norm([xv-tr_i.x(i);yv-tr_i.y(i)]); if e>max_e, max_e=e; end
    end
    try, assert(max_e<TOL_FK,'max_fk_err=%.2e m',max_e);
         fprintf('[ OK ]  pair_%d_fk_consistency  (max=%.2e m)\n',pi_,max_e); n_pass=n_pass+1;
    catch ME, fprintf('[FAIL]  pair_%d_fk_consistency  %s\n',pi_,ME.message); n_fail=n_fail+1; end

    % pick reached exactly
    ip2=find(tr_i.phase==2);
    ep=norm([tr_i.x(ip2(end))-pk(1); tr_i.y(ip2(end))-pk(2)]);
    try, assert(ep<TOL_POS,'err=%.2e m',ep);
         fprintf('[ OK ]  pair_%d_pick_exact\n',pi_); n_pass=n_pass+1;
    catch ME, fprintf('[FAIL]  pair_%d_pick_exact  %s\n',pi_,ME.message); n_fail=n_fail+1; end
end

fprintf('\n');

% Section 9: Robustness
fprintf('[----------] 9. Robustness\n');

% 9.1  pick == place: no crash, struct returned
try
    tr_same = custom_pick_place([0.8 0.3],[0.8 0.3],L1,L2,opts);
    assert(isstruct(tr_same) && isfield(tr_same,'x'));
    fprintf('[ OK ]  degenerate_pick_equals_place_no_crash\n'); n_pass=n_pass+1;
catch ME
    fprintf('[FAIL]  degenerate_pick_equals_place_no_crash  %s\n',ME.message); n_fail=n_fail+1;
end

% 9.2  n_failed is non-negative integer
try, assert(isnumeric(traj.n_failed) && traj.n_failed>=0 && floor(traj.n_failed)==traj.n_failed);
     fprintf('[ OK ]  n_failed_is_nonneg_integer  (%d)\n',traj.n_failed); n_pass=n_pass+1;
catch, fprintf('[FAIL]  n_failed_is_nonneg_integer\n'); n_fail=n_fail+1; end

% 9.3  n_steps=1: 7 total waypoints
o=opts; o.n_steps=1;
tr=custom_pick_place(PICK,PLACE,L1,L2,o);
try, assert(tr.total_pts==7,'total_pts=%d',tr.total_pts);
     fprintf('[ OK ]  n_steps_1_gives_7_total_waypoints\n'); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  n_steps_1_gives_7_total_waypoints  %s\n',ME.message); n_fail=n_fail+1; end

% 9.4  n_steps=100: 700 total waypoints
o=opts; o.n_steps=100;
tr=custom_pick_place(PICK,PLACE,L1,L2,o);
try, assert(tr.total_pts==700,'total_pts=%d',tr.total_pts);
     fprintf('[ OK ]  n_steps_100_gives_700_total_waypoints\n'); n_pass=n_pass+1;
catch ME, fprintf('[FAIL]  n_steps_100_gives_700_total_waypoints  %s\n',ME.message); n_fail=n_fail+1; end

% Summary
elapsed = toc(t_start);
fprintf('\n[==========] %d tests ran  (%.3f s)\n', n_pass+n_fail, elapsed);
if n_fail == 0
    fprintf('[  PASSED  ] %d tests\n', n_pass);
else
    fprintf('[  PASSED  ] %d tests\n', n_pass);
    fprintf('[  FAILED  ] %d tests\n', n_fail);
    error('test_pick_place: %d test(s) FAILED', n_fail);
end