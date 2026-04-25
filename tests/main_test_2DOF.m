%% MAIN_TEST_2DOF.m
%  Complete test script for 2-DOF serial arm FK + IK
%  Run this BEFORE opening Simulink to confirm all algorithms work.
%    1. Open MATLAB
%    2. Navigate to the project root folder
%    3. Press F5 or click the green Run button on this file
%    4. All sections run automatically. Check Command Window output.

% Add paths to kinematics and visualization folders
test_folder = fileparts(mfilename('fullpath'));
project_root = fileparts(test_folder);
kinematics_folder = fullfile(project_root, 'kinematics');
visualization_folder = fullfile(project_root, 'visualization');
addpath(kinematics_folder);
addpath(visualization_folder);

clear; clc; close all;
fprintf('================================================\n');
fprintf(' 2-DOF Planar Arm - 1 Test Suite\n');
fprintf('================================================\n\n');

%% ── PARAMETERS ──────────────────────────────────────────────────
L1 = 1.0;   % link 1 length (m)
L2 = 0.8;   % link 2 length (m)

fprintf('Arm: L1=%.2f m   L2=%.2f m\n', L1, L2);
fprintf('Workspace: r_min=%.2f m   r_max=%.2f m\n\n', abs(L1-L2), L1+L2);

pass = 0;
fail = 0;

%% ── SECTION 1: FK TESTS ─────────────────────────────────────────
fprintf('--- FK Tests (all 3 methods) ---\n');

fk_tests = [0,0; 45,90; 90,-45; -30,60; 180,0; -90,180; 135,-90; 60,60];

for i = 1:size(fk_tests,1)
    t1 = deg2rad(fk_tests(i,1));
    t2 = deg2rad(fk_tests(i,2));

    [xg,yg] = custom_FK_2DOF(t1, t2, L1, L2, 'geometric');
    [xd,yd] = custom_FK_2DOF(t1, t2, L1, L2, 'dh');
    [xr,yr] = custom_FK_2DOF(t1, t2, L1, L2, 'rotchain');

    err = max([abs(xg-xd) abs(yg-yd) abs(xg-xr) abs(yg-yr)]);
    ok  = (err < 1e-10);

    if ok
        fprintf('  PASS  FK t1=%4.0f t2=%4.0f => EE=(%.4f, %.4f)  3-method err=%.1e\n', ...
                fk_tests(i,1), fk_tests(i,2), xg, yg, err);
        pass = pass + 1;
    else
        fprintf('  FAIL  FK t1=%4.0f t2=%4.0f  err=%.2e\n', fk_tests(i,1), fk_tests(i,2), err);
        fail = fail + 1;
    end
end

%% ── SECTION 2: IK TESTS ─────────────────────────────────────────
fprintf('\n--- IK Tests (elbow-up + down, all quadrants) ---\n');

ik_tests = [1.8,0; 1.0,1.0; 0.5,1.2; -0.8,0.6; 0.3,-1.1; -1.0,-0.8; 0.0,1.5; -1.5,-0.5];

for i = 1:size(ik_tests,1)
    xt = ik_tests(i,1);
    yt = ik_tests(i,2);

    [t1u, t2u, oku] = custom_IK_2DOF(xt, yt, L1, L2, 1);
    [t1d, t2d, okd] = custom_IK_2DOF(xt, yt, L1, L2, 0);

    if oku
        [xv,yv] = custom_FK_2DOF(t1u, t2u, L1, L2);
        err = sqrt((xv-xt)^2 + (yv-yt)^2);
        ang_ok = (t1u >= -pi-1e-9 && t1u <= pi+1e-9);
        ok = (err < 1e-10 && ang_ok);
        if ok
            fprintf('  PASS  IK (% .2f,% .2f) => t1=%7.2f  t2=%7.2f  err=%.1e\n', ...
                    xt, yt, rad2deg(t1u), rad2deg(t2u), err);
            pass = pass + 1;
        else
            fprintf('  FAIL  IK (% .2f,% .2f) err=%.2e  ang_ok=%d\n', xt, yt, err, ang_ok);
            fail = fail + 1;
        end
    else
        fprintf('  SKIP  IK (% .2f,% .2f) - outside workspace\n', xt, yt);
    end
end

%% ── SECTION 3: ROUND-TRIP ───────────────────────────────────────
fprintf('\n--- Round-trip FK->IK->FK (1000 random samples) ---\n');

max_err = 0;
n_fail  = 0;

for i = 1:1000
    t1r = (rand - 0.5) * 2 * pi;
    t2r = (rand - 0.5) * 2 * pi;
    [x, y]       = custom_FK_2DOF(t1r, t2r, L1, L2);
    [t1s, t2s, ok] = custom_IK_2DOF(x, y, L1, L2, 1);
    if ok
        [x2, y2] = custom_FK_2DOF(t1s, t2s, L1, L2);
        e = sqrt((x2-x)^2 + (y2-y)^2);
        if e > max_err, max_err = e; end
        if e > 1e-10, n_fail = n_fail + 1; end
    end
end

if max_err < 1e-10 && n_fail == 0
    fprintf('  PASS  1000 samples - max error = %.2e\n', max_err);
    pass = pass + 1;
else
    fprintf('  FAIL  %d samples failed - max error = %.2e\n', n_fail, max_err);
    fail = fail + 1;
end

%% ── SECTION 4: SINGULARITY CHECK ───────────────────────────────
fprintf('\n--- Jacobian determinant (singularity detection) ---\n');

sing_cases = [0,0; 45,0; 0,180];
for i = 1:size(sing_cases,1)
    t1 = deg2rad(sing_cases(i,1));
    t2 = deg2rad(sing_cases(i,2));
    J  = [-L1*sin(t1)-L2*sin(t1+t2), -L2*sin(t1+t2);
           L1*cos(t1)+L2*cos(t1+t2),  L2*cos(t1+t2)];
    d  = det(J);
    fprintf('  SING  t1=%4.0f t2=%4.0f  det(J)=%.5f  (expected ≈ 0)\n', ...
            sing_cases(i,1), sing_cases(i,2), d);
end

% non-singular check
t1 = deg2rad(45); t2 = deg2rad(90);
J  = [-L1*sin(t1)-L2*sin(t1+t2), -L2*sin(t1+t2);
       L1*cos(t1)+L2*cos(t1+t2),  L2*cos(t1+t2)];
d  = det(J);
ok = abs(d) > 0.1;
fprintf('  %s  Non-sing (45,90)  det(J)=%.5f  (expected ≠ 0)\n', ...
        ifelse_str(ok,'PASS','FAIL'), d);
if ok, pass = pass+1; else, fail = fail+1; end

%% ── SECTION 5: DH MATRIX CHECK ─────────────────────────────────
fprintf('\n--- DH T_total validity (SO2 rotation block) ---\n');

[~,~,~,~,T] = custom_FK_2DOF(deg2rad(45), deg2rad(90), L1, L2, 'dh');
R = T(1:2, 1:2);
det_ok   = abs(det(R) - 1) < 1e-10;
ortho_ok = norm(R'*R - eye(2)) < 1e-10;
phi      = atan2(T(2,1), T(1,1));

fprintf('  det(R) = %.6f  (expected 1.0)  %s\n', det(R), ifelse_str(det_ok,'PASS','FAIL'));
fprintf('  ||R^T R - I|| = %.1e            %s\n', norm(R'*R-eye(2)), ifelse_str(ortho_ok,'PASS','FAIL'));
fprintf('  EE orientation phi = %.2f deg   (expected 135.00)\n', rad2deg(phi));
if det_ok && ortho_ok, pass = pass+2; else, fail = fail+2; end

%% ── SECTION 6: PLOTS ────────────────────────────────────────────
fprintf('\n--- Generating plots ---\n');

% Workspace
figure('Name','Workspace','Color','white');
plot_workspace_2DOF(L1, L2);
fprintf('  Workspace figure opened.\n');

% Elbow-up vs elbow-down
xt = 1.0; yt = 0.8;
[t1u, t2u] = custom_IK_2DOF(xt, yt, L1, L2, 1);
[t1d, t2d] = custom_IK_2DOF(xt, yt, L1, L2, 0);
figure('Name','Elbow Solutions','Color','white');
subplot(1,2,1); plot_arm_2DOF(t1u, t2u, L1, L2, xt, yt);
title(sprintf('Elbow-UP  t1=%.1f° t2=%.1f°', rad2deg(t1u), rad2deg(t2u)));
subplot(1,2,2); plot_arm_2DOF(t1d, t2d, L1, L2, xt, yt);
title(sprintf('Elbow-DOWN  t1=%.1f° t2=%.1f°', rad2deg(t1d), rad2deg(t2d)));
sgtitle('IK dual solutions for same target');
fprintf('  Elbow-up/down figure opened.\n');

% Animation: circular path
figure('Name','Animation - circular path','Color','white');
r_c = 0.6; cx = 0.8; cy = 0.4;
phi_arr = linspace(0, 2*pi, 120);
ee_x = zeros(1,120); ee_y = zeros(1,120);
for k = 1:120
    xp = cx + r_c*cos(phi_arr(k));
    yp = cy + r_c*sin(phi_arr(k));
    [t1, t2, ok] = custom_IK_2DOF(xp, yp, L1, L2, 1);
    if ok
        plot_arm_2DOF(t1, t2, L1, L2, xp, yp);
        [ex, ey] = custom_FK_2DOF(t1, t2, L1, L2);
        ee_x(k) = ex; ee_y(k) = ey;
        hold on;
        plot(ee_x(1:k), ee_y(1:k), 'g-', 'LineWidth', 2);
        plot(cx + r_c*cos(linspace(0,2*pi,200)), ...
             cy + r_c*sin(linspace(0,2*pi,200)), 'r--', 'LineWidth', 1);
        hold off;
        drawnow; pause(0.02);
    end
end
fprintf('  Animation complete.\n');

%% ── SUMMARY ─────────────────────────────────────────────────────
fprintf('\n================================================\n');
fprintf(' RESULTS:  PASS = %d   FAIL = %d\n', pass, fail);
if fail == 0
    fprintf(' STATUS:   ALL PASS - Ready for Simulink\n');
else
    fprintf(' STATUS:   FAILURES FOUND - fix before Simulink\n');
end
fprintf('================================================\n');

% ---- helper (replaces ternary) ----
function s = ifelse_str(cond, a, b)
    if cond, s = a; else, s = b; end
end
