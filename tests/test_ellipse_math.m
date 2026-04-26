% Headless math checks for ellipse_math.
test_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(test_dir);
addpath(fullfile(root_dir, 'kinematics'));
addpath(fullfile(root_dir, 'visualization'));
addpath(test_dir);

fprintf('[==========] test_ellipse_math.m\n');
fprintf('[----------] plot_jacobian_ellipse - math verification (headless)\n\n');

L1=1.0; L2=0.8; pass=0; fail=0;

% E1
fprintf('[E1] a_axis * b_axis == |det(J)|\n');
ok=true;
for i=1:50
    t1=rand*2*pi-pi; t2=rand*2*pi-pi;
    [a,b,~,w] = ellipse_math(t1,t2,L1,L2);
    s1e=sin(t1);c1e=cos(t1);s12e=sin(t1+t2);c12e=cos(t1+t2);
    Je=[-L1*s1e-L2*s12e,-L2*s12e;L1*c1e+L2*c12e,L2*c12e];
    det_J=abs(det(Je));
    if abs(a*b - det_J) > 1e-10
        ok=false;
        fprintf('  FAIL cfg %d: a*b=%.8f  det=%.8f\n',i,a*b,det_J);
    end
end
fprintf('  %s  a*b==|det(J)| for 50 random configs\n',rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

% E2
fprintf('\n[E2] Singularity (t2=0): b_axis~=0, w~=0 (line degenerate)\n');
[a,b,~,w] = ellipse_math(pi/4, 0, L1, L2);
ok = b < 1e-10 && w < 1e-10;
fprintf('  a=%.6f  b=%.2e  w=%.2e\n',a,b,w);
fprintf('  %s  b and w near zero\n',rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

% E3
fprintf('\n[E3] Optimal (t2=pi/2): w == L1*L2 == %.4f\n',L1*L2);
[a,b,~,w] = ellipse_math(0, pi/2, L1, L2);
ok = abs(w - L1*L2) < 1e-10 && a > 0 && b > 0;
fprintf('  a=%.5f  b=%.5f  w=%.6f  (target=%.6f)\n',a,b,w,L1*L2);
fprintf('  %s\n',rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

% E4
fprintf('\n[E4] a_axis >= b_axis for all configs (SVD ordering)\n');
ok=true;
for i=1:100
    t1=rand*2*pi-pi; t2=rand*2*pi-pi;
    [a,b]=ellipse_math(t1,t2,L1,L2);
    if a < b - 1e-12
        ok=false;
        fprintf('  FAIL: a=%.6f < b=%.6f\n',a,b);
    end
end
fprintf('  %s  a>=b for 100 random configs\n',rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

% E5
fprintf('\n[E5] Near-isotropic (L1=L2=1, t2=pi/2): both axes positive\n');
[a5,b5,~,~] = ellipse_math(0, pi/2, 1.0, 1.0);
ratio = a5/max(b5,1e-12);
ok = ratio < 20 && b5 > 0;
fprintf('  a=%.4f  b=%.4f  ratio=%.3f\n',a5,b5,ratio);
fprintf('  %s  Both axes positive\n',rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

% E6
fprintf('\n[E6] angle_deg always in [-180, 180]\n');
ok=true;
for i=1:100
    t1=rand*2*pi-pi; t2=rand*2*pi-pi;
    [~,~,ang]=ellipse_math(t1,t2,L1,L2);
    if ang < -180-1e-9 || ang > 180+1e-9
        ok=false;
        fprintf('  FAIL: angle=%.3f\n',ang);
    end
end
fprintf('  %s  angle in [-180,180] for 100 configs\n',rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

% E7
fprintf('\n[E7] ellipse_math w == custom_jacobian w (30 random configs)\n');
ok=true;
for i=1:30
    t1=rand*2*pi-pi; t2=rand*2*pi-pi;
    [~,~,~,we]=ellipse_math(t1,t2,L1,L2);
    [~,wj]=custom_jacobian(t1,t2,L1,L2);
    if abs(we-wj)>1e-10
        ok=false;
        fprintf('  FAIL cfg %d: we=%.10f  wj=%.10f\n',i,we,wj);
    end
end
fprintf('  %s  Consistent for 30 configs\n',rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

% E8
fprintf('\n[E8] a_axis, b_axis always >= 0\n');
ok=true;
for i=1:100
    t1=rand*2*pi-pi; t2=rand*2*pi-pi;
    [a,b]=ellipse_math(t1,t2,L1,L2);
    if a < 0 || b < 0
        ok=false;
        fprintf('  FAIL: a=%.6f b=%.6f\n',a,b);
    end
end
fprintf('  %s  a,b >= 0 for 100 random configs\n',rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

% E9
fprintf('\n[E9] Axis ratio a/b increases as t2 -> 0 (near singular)\n');
t2_vals = [pi/2, pi/4, pi/8, pi/16, 0.05];
prev_ratio = 1.0;
ok=true;
fprintf('  t2(deg)   a        b        a/b\n');
for k=1:length(t2_vals)
    t2v = t2_vals(k);
    [av,bv]=ellipse_math(0,t2v,L1,L2);
    ratio_v = av / max(bv, 1e-12);
    fprintf('  %6.2f   %.5f  %.5f  %.2f\n', t2v*180/pi, av, bv, ratio_v);
    if ratio_v < prev_ratio - 1e-6
        ok=false;
        fprintf('  FAIL: ratio decreased at t2=%.4f\n',t2v);
    end
    prev_ratio = ratio_v;
end
fprintf('  %s  Ratio monotonically increases toward singularity\n',rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n[==========] Results\n');
fprintf('PASS=%d  FAIL=%d  TOTAL=%d\n',pass,fail,pass+fail);
if fail==0
    fprintf('STATUS: ALL PASS - ellipse math verified\n');
else
    fprintf('STATUS: %d FAILURE(S)\n',fail);
end
fprintf('[==========]\n');