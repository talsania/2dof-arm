%% test_jacobian
% Unit tests for custom_jacobian and custom_IK_velocity.

% Path setup
test_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(test_dir);
addpath(fullfile(root_dir, 'kinematics'));
addpath(fullfile(root_dir, 'visualization'));
addpath(test_dir);

clc;
fprintf('==========================================================\n');
fprintf('  test_jacobian.m  —  Stage 1 & 2 Unit Tests\n');
fprintf('==========================================================\n\n');

L1  = 1.0;
L2  = 0.8;
TOL = 1e-10;
pass = 0;
fail = 0;

% Group A: Jacobian structure and math
fprintf('GROUP A — Jacobian structure & math\n');
fprintf('----------------------------------------------------------\n');

%% A1: Size and type
fprintf('A1: J is 2x2 double\n');
[J] = custom_jacobian(pi/4, pi/3, L1, L2);
ok  = isequal(size(J), [2 2]) && isa(J, 'double');
fprintf('    size=%dx%d  class=%s  => %s\n', size(J,1), size(J,2), class(J), rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

%% A2: Exact formula at 6 benchmark configs
fprintf('A2: J entries match analytical formula at 6 benchmark configs\n');
cfgs = [0 0; pi/4 pi/2; pi/2 -pi/4; pi pi/3; -pi/3 2*pi/3; pi/6 -pi/6];
a2ok = true;
for i = 1:size(cfgs,1)
    t1=cfgs(i,1); t2=cfgs(i,2);
    s1=sin(t1); c1=cos(t1); s12=sin(t1+t2); c12=cos(t1+t2);
    Je = [-L1*s1-L2*s12, -L2*s12;
           L1*c1+L2*c12,  L2*c12];
    [Jg] = custom_jacobian(t1, t2, L1, L2);
    err  = max(abs(Jg(:)-Je(:)));
    if err > TOL
        fprintf('    MISMATCH cfg %d  max_err=%.2e\n', i, err);
        a2ok = false;
    end
end
fprintf('    All 6 configs match (tol=1e-10)  => %s\n', rs(a2ok));
if a2ok, pass=pass+1; else, fail=fail+1; end

%% A3: Numerical finite-difference cross-check
fprintf('A3: Analytical J == Numerical FD Jacobian (h=1e-6, tol=1e-8)\n');
h    = 1e-6;
a3ok = true;
for i = 1:20
    t1 = rand*2*pi - pi;
    t2 = rand*2*pi - pi;
    [Ja] = custom_jacobian(t1, t2, L1, L2);
    [xp1,yp1] = custom_FK_2DOF(t1+h, t2,   L1, L2);
    [xm1,ym1] = custom_FK_2DOF(t1-h, t2,   L1, L2);
    [xp2,yp2] = custom_FK_2DOF(t1,   t2+h, L1, L2);
    [xm2,ym2] = custom_FK_2DOF(t1,   t2-h, L1, L2);
    Jn = [(xp1-xm1)/(2*h),  (xp2-xm2)/(2*h);
          (yp1-ym1)/(2*h),  (yp2-ym2)/(2*h)];
    if max(abs(Ja(:)-Jn(:))) > 1e-8
        a3ok = false;
        fprintf('    MISMATCH cfg %d\n', i);
    end
end
fprintf('    20 random configs within 1e-8  => %s\n', rs(a3ok));
if a3ok, pass=pass+1; else, fail=fail+1; end

%% A4: det(J) = L1*L2*sin(t2) identity
fprintf('A4: det(J) = L1*L2*sin(t2) (analytical identity)\n');
a4ok = true;
t2v  = linspace(-pi, pi, 100);
for k = 1:100
    t1 = rand*2*pi - pi;
    t2 = t2v(k);
    [~,~,dg] = custom_jacobian(t1, t2, L1, L2);
    de = L1 * L2 * sin(t2);
    if abs(dg-de) > TOL
        a4ok = false;
        fprintf('    FAIL at t2=%.3f\n', t2);
    end
end
fprintf('    100 angles verified  => %s\n', rs(a4ok));
if a4ok, pass=pass+1; else, fail=fail+1; end

%% A5: det independent of t1
fprintf('A5: det(J) independent of theta1 (only depends on theta2)\n');
t2_fixed = pi/3;
dets = zeros(1,20);
for i = 1:20
    t1 = rand*2*pi - pi;
    [~,~,dets(i)] = custom_jacobian(t1, t2_fixed, L1, L2);
end
spread = max(dets) - min(dets);
ok     = spread < TOL;
fprintf('    t2=pi/3  det_spread=%.2e (expect 0)  => %s\n', spread, rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

%% A6: Output count — all 5 outputs accessible
fprintf('A6: All 5 outputs returned correctly\n');
[J,w,det_J,is_sing,best_el] = custom_jacobian(pi/4, pi/2, L1, L2);
ok = isequal(size(J),[2 2]) && isscalar(w) && isscalar(det_J) ...
     && islogical(is_sing) && ischar(best_el);
fprintf('    J:2x2  w:scalar  det:scalar  is_sing:logical  best_el:char  => %s\n', rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n');

% Group B: Singularity and manipulability
fprintf('GROUP B — Singularity & manipulability\n');
fprintf('----------------------------------------------------------\n');

%% B1: Singular when t2=0 (fully extended)
fprintf('B1: Singular at t2=0 (arm fully extended)\n');
[~,w1,~,s1] = custom_jacobian(pi/4, 0, L1, L2);
ok = s1 && w1 < 1e-4;
fprintf('    w=%.6f  is_singular=%d  => %s\n', w1, s1, rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

%% B2: Singular when t2=±pi (fully folded)
fprintf('B2: Singular at t2=pi and t2=-pi (arm fully folded)\n');
[~,~,~,sp]  = custom_jacobian(0,  pi, L1, L2);
[~,~,~,sn]  = custom_jacobian(0, -pi, L1, L2);
ok = sp && sn;
fprintf('    t2=+pi: is_singular=%d  t2=-pi: is_singular=%d  => %s\n', sp, sn, rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

%% B3: Non-singular at t2=pi/2 (optimal config)
fprintf('B3: Non-singular at t2=pi/2\n');
[~,w3,~,s3] = custom_jacobian(pi/4, pi/2, L1, L2);
ok = ~s3 && w3 > 0.1;
fprintf('    w=%.5f  is_singular=%d  => %s\n', w3, s3, rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

%% B4: Manipulability w = |det(J)| for 50 random configs
fprintf('B4: w == |det(J)| for 50 random configs\n');
b4ok = true;
for i = 1:50
    t1=rand*2*pi-pi; t2=rand*2*pi-pi;
    [~,w,dj] = custom_jacobian(t1, t2, L1, L2);
    if abs(w - abs(dj)) > TOL
        b4ok = false;
    end
end
fprintf('    50 random configs  => %s\n', rs(b4ok));
if b4ok, pass=pass+1; else, fail=fail+1; end

%% B5: Max manipulability = L1*L2 at t2=pi/2
fprintf('B5: Max w = L1*L2 = %.4f occurs at t2=pi/2\n', L1*L2);
t2s = linspace(0.001, pi-0.001, 5000);
wv  = zeros(1, 5000);
for k = 1:5000
    [~,wv(k)] = custom_jacobian(0, t2s(k), L1, L2);
end
[w_peak, idx] = max(wv);
t2_peak = t2s(idx);
ok = abs(w_peak - L1*L2) < 1e-4 && abs(t2_peak - pi/2) < 0.005;
fprintf('    w_peak=%.6f at t2=%.4f rad  => %s\n', w_peak, t2_peak, rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n');

% Group C: Velocity IK exact inverse
fprintf('GROUP C — Velocity IK: exact inverse\n');
fprintf('----------------------------------------------------------\n');

cfgs2 = [pi/4 pi/2; pi/6 pi/3; -pi/3 2*pi/3; pi/3 pi/4; pi/5 3*pi/5];

%% C1: J*theta_dot == x_dot (exact reconstruction)
fprintf('C1: J*theta_dot == x_dot for 5 non-singular configs\n');
c1ok = true;
for i = 1:size(cfgs2,1)
    t1=cfgs2(i,1); t2=cfgs2(i,2);
    xd = [0.5; -0.3];
    [td] = custom_IK_velocity(t1, t2, xd, L1, L2, 'exact');
    [J ] = custom_jacobian(t1, t2, L1, L2);
    err  = norm(J*td - xd);
    if err > 1e-10
        fprintf('    FAIL cfg %d  err=%.2e\n', i, err);
        c1ok = false;
    end
end
fprintf('    All 5 within tol=1e-10  => %s\n', rs(c1ok));
if c1ok, pass=pass+1; else, fail=fail+1; end

%% C2: Fallback at singularity — no NaN/Inf
fprintf('C2: Safe fallback at singularity (t2=0, t2=pi)\n');
sing_pts = [pi/4 0; -pi/3 pi; pi/2 0; 0 -pi];
c2ok = true;
for i = 1:size(sing_pts,1)
    t1=sing_pts(i,1); t2=sing_pts(i,2);
    [td, meth] = custom_IK_velocity(t1, t2, [1;0], L1, L2, 'exact');
    if any(isnan(td)) || any(isinf(td))
        fprintf('    NaN/Inf at cfg %d (t1=%.2f t2=%.2f)\n', i, t1, t2);
        c2ok = false;
    end
end
fprintf('    4 singular configs: no NaN/Inf  => %s\n', rs(c2ok));
if c2ok, pass=pass+1; else, fail=fail+1; end

%% C3: Condition number — finite for normal, Inf for singular
fprintf('C3: Condition number finite for normal pose, Inf at singularity\n');
[~,~,cn_norm] = custom_IK_velocity(pi/4, pi/2, [0.1;0], L1, L2, 'exact');
[~,~,cn_sing] = custom_IK_velocity(0,    0,    [0.1;0], L1, L2, 'transpose');
ok = isfinite(cn_norm) && cn_norm > 1 && isinf(cn_sing);
fprintf('    cond(normal)=%.3f  cond(singular)=%s  => %s\n', ...
        cn_norm, num2str(cn_sing), rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n');

% Group D: Velocity IK transpose
fprintf('GROUP D — Velocity IK: transpose\n');
fprintf('----------------------------------------------------------\n');

%% D1: Never NaN/Inf — even at all singularities
fprintf('D1: transpose never produces NaN/Inf (10 singular + 10 random configs)\n');
all_pts = [0 0; 0 pi; 0 -pi; pi/4 0; -pi/3 pi; pi/2 0; pi 0; -pi/4 pi;
           0.1 pi-0.01; 0 pi/2];
xd_test = [1; 0.5];
d1ok = true;
for i = 1:size(all_pts,1)
    t1=all_pts(i,1); t2=all_pts(i,2);
    [td] = custom_IK_velocity(t1, t2, xd_test, L1, L2, 'transpose');
    if any(isnan(td)) || any(isinf(td))
        fprintf('    NaN/Inf at cfg %d\n', i);
        d1ok = false;
    end
end
for i = 1:10
    t1=rand*2*pi-pi; t2=rand*2*pi-pi;
    [td] = custom_IK_velocity(t1, t2, xd_test, L1, L2, 'transpose');
    if any(isnan(td)) || any(isinf(td))
        d1ok = false;
    end
end
fprintf('    20 configs all safe  => %s\n', rs(d1ok));
if d1ok, pass=pass+1; else, fail=fail+1; end

%% D2: Same directional half-space as exact
fprintf('D2: dot(td_transpose, td_exact) > 0 for non-singular configs\n');
d2ok = true;
for i = 1:size(cfgs2,1)
    t1=cfgs2(i,1); t2=cfgs2(i,2);
    xd = [0.4; 0.3];
    [tde] = custom_IK_velocity(t1, t2, xd, L1, L2, 'exact');
    [tdt] = custom_IK_velocity(t1, t2, xd, L1, L2, 'transpose');
    if dot(tde, tdt) <= 0
        fprintf('    FAIL cfg %d  dot=%.4f\n', i, dot(tde,tdt));
        d2ok = false;
    end
end
fprintf('    5 configs: correct half-space  => %s\n', rs(d2ok));
if d2ok, pass=pass+1; else, fail=fail+1; end

%% D3: td = J^T * xd (formula check)
fprintf('D3: transpose formula: theta_dot == J^T * x_dot\n');
d3ok = true;
for i = 1:size(cfgs2,1)
    t1=cfgs2(i,1); t2=cfgs2(i,2);
    xd = [0.6; -0.2];
    [td]     = custom_IK_velocity(t1, t2, xd, L1, L2, 'transpose');
    [J]      = custom_jacobian(t1, t2, L1, L2);
    td_check = J' * xd;
    if norm(td - td_check) > TOL
        fprintf('    FAIL cfg %d  err=%.2e\n', i, norm(td-td_check));
        d3ok = false;
    end
end
fprintf('    5 configs match J^T*xd formula  => %s\n', rs(d3ok));
if d3ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n');

% Group E: Velocity IK damped least squares
fprintf('GROUP E — Velocity IK: DLS\n');
fprintf('----------------------------------------------------------\n');

%% E1: DLS(lambda=0) == exact for non-singular configs
fprintf('E1: DLS(lambda=0) == exact for 5 configs (tol=1e-8)\n');
e1ok = true;
for i = 1:size(cfgs2,1)
    t1=cfgs2(i,1); t2=cfgs2(i,2);
    xd = [0.5; -0.3];
    [tde] = custom_IK_velocity(t1, t2, xd, L1, L2, 'exact');
    [tdd] = custom_IK_velocity(t1, t2, xd, L1, L2, 'dls', 0);
    if norm(tde-tdd) > 1e-8
        fprintf('    FAIL cfg %d  err=%.2e\n', i, norm(tde-tdd));
        e1ok = false;
    end
end
fprintf('    5 configs match  => %s\n', rs(e1ok));
if e1ok, pass=pass+1; else, fail=fail+1; end

%% E2: DLS damps joint velocity near singularity
fprintf('E2: DLS suppresses velocity spike near singularity\n');
near_sing_configs = [0 0.005; 0 0.01; pi/4 0.008; -pi/3 0.003];
e2ok = true;
for i = 1:size(near_sing_configs,1)
    t1=near_sing_configs(i,1); t2=near_sing_configs(i,2);
    xd = [1.0; 0.0];
    [tde] = custom_IK_velocity(t1, t2, xd, L1, L2, 'exact');
    [tdd] = custom_IK_velocity(t1, t2, xd, L1, L2, 'dls', 0.1);
    ne = norm(tde); nd = norm(tdd);
    ratio = nd / ne;
    if ratio >= 1.0
        fprintf('    FAIL cfg %d: ||exact||=%.1f  ||dls||=%.1f\n', i, ne, nd);
        e2ok = false;
    else
        fprintf('    cfg %d: ||exact||=%7.2f  ||dls||=%6.4f  ratio=%.4f\n', ...
                i, ne, nd, ratio);
    end
end
fprintf('    DLS always smaller than exact near singularity  => %s\n', rs(e2ok));
if e2ok, pass=pass+1; else, fail=fail+1; end

%% E3: DLS safe at all singularities
fprintf('E3: DLS safe (no NaN/Inf) at 5 fully singular configs\n');
e3ok = true;
for i = 1:size(all_pts,1)
    t1=all_pts(i,1); t2=all_pts(i,2);
    [td] = custom_IK_velocity(t1, t2, [1;0], L1, L2, 'dls', 0.05);
    if any(isnan(td)) || any(isinf(td))
        e3ok = false;
        fprintf('    NaN/Inf at cfg %d\n', i);
    end
end
fprintf('    All configs safe  => %s\n', rs(e3ok));
if e3ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n');

% Group F: Auto mode
fprintf('GROUP F — Velocity IK: AUTO mode selection\n');
fprintf('----------------------------------------------------------\n');

%% F1: Method selection by manipulability band
fprintf('F1: Correct method selected per manipulability band\n');
[~,w_hi] = custom_jacobian(pi/4, pi/2, L1, L2);
[~, m1 ] = custom_IK_velocity(pi/4, pi/2, [0.1;0.1], L1, L2, 'auto');
ok1 = strcmp(m1, 'exact');
fprintf('    w=%.4f  -> "%s"  (want exact)      %s\n', w_hi, m1, rs(ok1));

[~,w_mid] = custom_jacobian(0, 0.05, L1, L2);
[~, m2  ] = custom_IK_velocity(0, 0.05, [0.1;0], L1, L2, 'auto');
ok2 = ~isempty(strfind(m2,'dls')) || strcmp(m2,'transpose');
fprintf('    w=%.4f  -> "%s"  (want dls/transpose)  %s\n', w_mid, m2, rs(ok2));

[~,w_lo] = custom_jacobian(0, 0, L1, L2);
[~, m3 ] = custom_IK_velocity(0, 0, [0.1;0], L1, L2, 'auto');
ok3 = strcmp(m3,'transpose') || ~isempty(strfind(m3,'fallback')) ...
   || ~isempty(strfind(m3,'dls'));
fprintf('    w=%.4f  -> "%s"  (want transpose/dls)  %s\n', w_lo, m3, rs(ok3));

ok_f1 = ok1 && ok2 && ok3;
if ok_f1, pass=pass+1; else, fail=fail+1; end

%% F2: Auto never produces NaN/Inf across full workspace sweep
fprintf('F2: AUTO never NaN/Inf across 200-config workspace sweep\n');
f2ok = true;
for i = 1:200
    t1 = rand*2*pi - pi;
    t2 = rand*2*pi - pi;
    xd = [(rand-0.5)*2; (rand-0.5)*2];
    [td] = custom_IK_velocity(t1, t2, xd, L1, L2, 'auto');
    if any(isnan(td)) || any(isinf(td))
        f2ok = false;
        fprintf('    NaN/Inf at random cfg %d\n', i);
    end
end
fprintf('    200 random configs all safe  => %s\n', rs(f2ok));
if f2ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n');

% Group G: Integration and dynamics
fprintf('GROUP G — Integration & dynamics\n');
fprintf('----------------------------------------------------------\n');

%% G1: Integrate velocity IK — EE tracks desired velocity accurately
fprintf('G1: Euler integration tracks desired EE velocity (x-direction)\n');
t1 = pi/4; t2 = pi/2;
xd_des = [0.3; 0.0];
dt = 0.001; N = 80;
[x0,y0] = custom_FK_2DOF(t1, t2, L1, L2);
for k = 1:N
    [td] = custom_IK_velocity(t1, t2, xd_des, L1, L2, 'exact');
    t1 = t1 + td(1)*dt;
    t2 = t2 + td(2)*dt;
end
[xf,yf] = custom_FK_2DOF(t1, t2, L1, L2);
vx_actual = (xf - x0) / (N*dt);
vy_actual = (yf - y0) / (N*dt);
ok = abs(vx_actual - xd_des(1)) < 0.01 && abs(vy_actual - xd_des(2)) < 0.01;
fprintf('    desired=[%.2f, %.2f]  got=[%.4f, %.4f]  => %s\n', ...
        xd_des(1), xd_des(2), vx_actual, vy_actual, rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

%% G2: Circular velocity path — EE stays near expected arc
fprintf('G2: Circular velocity path: EE traces arc (radius test)\n');
t1 = pi/4; t2 = pi/2;
[x0,y0] = custom_FK_2DOF(t1, t2, L1, L2);
cx = x0 - 0.3;   % circle center to left of starting point
cy = y0;
r_target = 0.3;
dt2 = 0.001; N2 = 100;
radii = zeros(1, N2);
for k = 1:N2
    [xc,yc] = custom_FK_2DOF(t1, t2, L1, L2);
    rx = xc - cx; ry = yc - cy;
    speed = 0.3;
    xdv = -ry / norm([rx;ry]) * speed;
    ydv =  rx / norm([rx;ry]) * speed;
    [td] = custom_IK_velocity(t1, t2, [xdv;ydv], L1, L2, 'exact');
    t1 = t1 + td(1)*dt2;
    t2 = t2 + td(2)*dt2;
    [xe,ye] = custom_FK_2DOF(t1, t2, L1, L2);
    radii(k) = norm([xe-cx; ye-cy]);
end
r_drift = max(abs(radii - r_target));
ok = r_drift < 0.05;
fprintf('    target_r=%.3f  max_drift=%.4f (tol=0.05)  => %s\n', ...
        r_target, r_drift, rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

% Summary
fprintf('\n==========================================================\n');
fprintf('  RESULTS: PASS=%-2d  FAIL=%-2d  TOTAL=%d\n', pass, fail, pass+fail);
if fail == 0
    fprintf('  STATUS:  ALL PASS  — Stage 1 & 2 fully verified\n');
else
    fprintf('  STATUS:  %d FAILURE(S) found\n', fail);
    fprintf('  Check FAIL lines above for details.\n');
end
fprintf('==========================================================\n');

% rs helper is in tests/rs.m