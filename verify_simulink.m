%% VERIFY_SIMULINK.m
%  Run AFTER simulating arm_static.slx and arm_trajectory.slx
%    1. Open arm_static.slx → press Ctrl+T
%    2. Open arm_trajectory.slx → press Ctrl+T
%    3. Come back here → press F5
%    4. All checks should print PASS

fprintf('================================================\n');
fprintf(' Simulink Results Verification\n');
fprintf('================================================\n\n');

L1 = 1.0; L2 = 0.8;
pass = 0; fail = 0;

% V1: Static model — Display values
fprintf('--- V1: arm_static expected output ---\n');
fprintf('   Open arm_static.slx and check the Display blocks:\n');
fprintf('   Display_x  should show:  1.00000\n');
fprintf('   Display_y  should show:  0.80000\n');
fprintf('   Display_t1 should show:  0.00000  (rad, = 0 deg)\n');
fprintf('   Display_t2 should show:  1.57080  (rad, = 90 deg)\n\n');

% compute expected values manually to confirm
xt=1.0; yt=0.8;
c2=max(-1,min(1,(xt^2+yt^2-L1^2-L2^2)/(2*L1*L2)));
t2_exp=atan2(sqrt(1-c2^2),c2);
alp=atan2(yt,xt); bet=atan2(L2*sin(t2_exp),L1+L2*cos(t2_exp));
t1_exp=atan2(sin(alp-bet),cos(alp-bet));
x_exp=L1*cos(t1_exp)+L2*cos(t1_exp+t2_exp);
y_exp=L1*sin(t1_exp)+L2*sin(t1_exp+t2_exp);
fprintf('   Verification (computed here):\n');
fprintf('   t1 = %.5f rad  (%.2f deg)\n', t1_exp, rad2deg(t1_exp));
fprintf('   t2 = %.5f rad  (%.2f deg)\n', t2_exp, rad2deg(t2_exp));
fprintf('   x_ee = %.5f  y_ee = %.5f\n', x_exp, y_exp);
fprintf('   Error = %.2e\n\n', sqrt((x_exp-xt)^2+(y_exp-yt)^2));

% V2: Check ee_static in workspace
fprintf('--- V2: ee_static variable (from arm_static To Workspace block) ---\n');
if exist('ee_static','var') && ~isempty(ee_static)
    last_x = ee_static(end,1);
    ok = abs(last_x - 1.0) < 0.01;
    fprintf('  %s  x_ee final value = %.5f  (expected 1.0)\n', ...
            ifelse_str(ok,'PASS','FAIL'), last_x);
    if ok, pass=pass+1; else, fail=fail+1; end
else
    fprintf('  SKIP — ee_static not found. Run arm_static.slx first (Ctrl+T).\n');
end

% V3: Check ee_traj from trajectory model
fprintf('\n--- V3: ee_traj variable (from arm_trajectory To Workspace block) ---\n');
if exist('ee_traj','var') && ~isempty(ee_traj)
    x_vals = ee_traj(:,1);
    ok1 = (max(x_vals) > 1.0);
    ok2 = (min(x_vals) < 1.0);
    ok  = ok1 && ok2;
    fprintf('  %s  x_ee varies: min=%.3f  max=%.3f  (should span around 1.0)\n', ...
            ifelse_str(ok,'PASS','FAIL'), min(x_vals), max(x_vals));
    fprintf('        %d trajectory samples recorded\n', length(ee_traj));
    if ok, pass=pass+1; else, fail=fail+1; end
else
    fprintf('  SKIP — ee_traj not found. Run arm_trajectory.slx first (Ctrl+T).\n');
end

% V4: FK/IK algorithm precision (always runs)
fprintf('\n--- V4: FK/IK algorithm precision (independent of Simulink) ---\n');

tests = [1.0,0.8; 1.5,0.5; -0.8,0.6; 0.3,-1.1; -1.0,-0.8];
all_ok = 1;
for i = 1:size(tests,1)
    xt=tests(i,1); yt=tests(i,2);
    [t1,t2,ok] = custom_IK_2DOF(xt, yt, L1, L2, 1);
    if ok
        [xv,yv] = custom_FK_2DOF(t1, t2, L1, L2);
        err = sqrt((xv-xt)^2+(yv-yt)^2);
        ok2 = err < 1e-10;
        if ~ok2, all_ok=0; end
        fprintf('  %s  (% .2f,% .2f) => (%.5f,%.5f) err=%.1e\n', ...
                ifelse_str(ok2,'PASS','FAIL'), xt, yt, xv, yv, err);
        if ok2, pass=pass+1; else, fail=fail+1; end
    end
end

% V5: Singularity
fprintf('\n--- V5: Singularity (Jacobian det = 0 at boundary poses) ---\n');
t1=0; t2=0;
J=[-L1*sin(t1)-L2*sin(t1+t2),-L2*sin(t1+t2); ...
    L1*cos(t1)+L2*cos(t1+t2), L2*cos(t1+t2)];
d=det(J);
ok = abs(d) < 1e-9;
fprintf('  %s  Fully extended (t2=0): det(J)=%.5f  (expected 0)\n', ...
        ifelse_str(ok,'PASS','FAIL'), d);
if ok, pass=pass+1; else, fail=fail+1; end

t1=pi/4; t2=pi/2;
J=[-L1*sin(t1)-L2*sin(t1+t2),-L2*sin(t1+t2); ...
    L1*cos(t1)+L2*cos(t1+t2), L2*cos(t1+t2)];
d=det(J);
ok = abs(d) > 0.1;
fprintf('  %s  Normal pose (45,90): det(J)=%.5f  (expected ~0.8)\n', ...
        ifelse_str(ok,'PASS','FAIL'), d);
if ok, pass=pass+1; else, fail=fail+1; end

% Summary
fprintf('\n================================================\n');
fprintf(' RESULTS:  PASS = %d   FAIL = %d\n', pass, fail);
if fail == 0
    fprintf(' STATUS:   ALL VERIFIED\n');
else
    fprintf(' STATUS:   %d issue(s) found\n', fail);
end
fprintf('================================================\n');

function s = ifelse_str(cond, a, b)
    if cond, s = a; else, s = b; end
end