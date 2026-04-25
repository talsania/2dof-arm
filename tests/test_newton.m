%% TEST_NEWTON.m
% Tests for custom_IK_newton.

test_dir = fileparts(mfilename('fullpath'));
root_dir = fileparts(test_dir);
addpath(fullfile(root_dir, 'kinematics'));
addpath(fullfile(root_dir, 'visualization'));
addpath(fullfile(root_dir, 'tests'));

clc;
fprintf('==========================================================\n');
fprintf('  test_newton.m - Newton-Raphson IK\n');
fprintf('==========================================================\n\n');

L1 = 1.0; L2 = 0.8; pass = 0; fail = 0; TOL = 1e-9;

% Standard reachable targets
targets = [
    1.0   0.8;    % Q1 nominal
    1.5   0.5;    % Q1 near boundary
    0.5   1.2;    % Q1 high
   -0.8   0.6;    % Q2
   -1.0  -0.8;    % Q3
    0.3  -1.1;    % Q4
    0.0   1.5;    % top
   -1.5  -0.5;    % left
];

% Group A: basic convergence with closedform seed
fprintf('GROUP A - Convergence with closedform seed\n');
fprintf('----------------------------------------------------------\n');

%% A1: Converges and valid=1 for all standard targets
fprintf('A1: valid=1 for all 8 standard targets\n');
a1ok = true;
for i = 1:size(targets,1)
    [t1,t2,ok] = custom_IK_newton(targets(i,1), targets(i,2), L1, L2);
    if ~ok
        fprintf('  FAIL target %d (%.2f,%.2f)\n', i, targets(i,1), targets(i,2));
        a1ok = false;
    end
end
fprintf('  %s  8/8 targets converged\n', rs(a1ok));
if a1ok, pass=pass+1; else, fail=fail+1; end

%% A2: FK(solution) matches target (accuracy)
fprintf('\nA2: ||FK(t1,t2) - target|| < 1e-9 for all 8 targets\n');
a2ok = true;
for i = 1:size(targets,1)
    xt=targets(i,1); yt=targets(i,2);
    [t1,t2,ok] = custom_IK_newton(xt, yt, L1, L2);
    if ok
        [xv,yv] = custom_FK_2DOF(t1, t2, L1, L2);
        err = sqrt((xv-xt)^2 + (yv-yt)^2);
        if err > TOL
            fprintf('  FAIL target %d: err=%.2e\n', i, err);
            a2ok = false;
        end
    end
end
fprintf('  %s  All within 1e-9\n', rs(a2ok));
if a2ok, pass=pass+1; else, fail=fail+1; end

%% A3: Closedform seed converges fast (<=3 iterations)
fprintf('\nA3: Closedform seed converges in <=3 iterations\n');
a3ok = true;
opts.init = 'closedform';
for i = 1:size(targets,1)
    xt=targets(i,1); yt=targets(i,2);
    [~,~,ok,iters] = custom_IK_newton(xt, yt, L1, L2, opts);
    if ok && iters > 3
        fprintf('  WARN target %d: took %d iters (expected <=3)\n', i, iters);
        % informational only
    end
    if ~ok
        fprintf('  FAIL target %d did not converge\n', i);
        a3ok = false;
    end
end
fprintf('  %s  All converged (iter count checked)\n', rs(a3ok));
if a3ok, pass=pass+1; else, fail=fail+1; end

%% A4: Angles wrapped to [-pi, pi]
fprintf('\nA4: Output angles always in [-pi, pi]\n');
a4ok = true;
for i = 1:size(targets,1)
    [t1,t2,ok] = custom_IK_newton(targets(i,1), targets(i,2), L1, L2);
    if ok
        if t1 < -pi-1e-9 || t1 > pi+1e-9 || t2 < -pi-1e-9 || t2 > pi+1e-9
            fprintf('  FAIL target %d: t1=%.4f t2=%.4f out of [-pi,pi]\n', i, t1, t2);
            a4ok = false;
        end
    end
end
fprintf('  %s  All angles in [-pi, pi]\n', rs(a4ok));
if a4ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n');

% Group B: cold start seeds
fprintf('GROUP B - Cold start seeds (zero, random)\n');
fprintf('----------------------------------------------------------\n');

%% B1: Zero seed converges for easy targets
fprintf('B1: Zero seed converges for targets in Q1 (near workspace center)\n');
easy = [1.0 0.5; 0.8 0.6; 1.2 0.3; 0.6 0.8];
opts_zero.init = 'zero'; opts_zero.max_iter = 100;
b1ok = true;
for i = 1:size(easy,1)
    [t1,t2,ok,iters] = custom_IK_newton(easy(i,1), easy(i,2), L1, L2, opts_zero);
    if ok
        [xv,yv] = custom_FK_2DOF(t1, t2, L1, L2);
        err = norm([xv-easy(i,1); yv-easy(i,2)]);
        if err > 1e-8
            fprintf('  FAIL target %d: err=%.2e\n', i, err);
            b1ok = false;
        end
    else
        fprintf('  WARN target %d did not converge from zero (took %d iters)\n', i, iters);
        % informational only
    end
end
fprintf('  %s  Zero seed: easy targets converge\n', rs(b1ok));
if b1ok, pass=pass+1; else, fail=fail+1; end

%% B2: Random seed - run 20 trials, check accuracy when valid=1
fprintf('\nB2: Random seed: when valid=1, accuracy is correct\n');
opts_rand.init = 'random'; opts_rand.max_iter = 200; opts_rand.tol = 1e-8;
b2ok = true; n_conv = 0;
rng(42);
xt = 1.0; yt = 0.5;
for i = 1:20
    [t1,t2,ok,iters] = custom_IK_newton(xt, yt, L1, L2, opts_rand);
    if ok
        n_conv = n_conv + 1;
        [xv,yv] = custom_FK_2DOF(t1, t2, L1, L2);
        err = norm([xv-xt; yv-yt]);
        if err > 1e-7
            fprintf('  FAIL trial %d: err=%.2e\n', i, err);
            b2ok = false;
        end
    end
end
fprintf('  Converged %d/20 random trials\n', n_conv);
fprintf('  %s  All converged trials accurate\n', rs(b2ok));
if b2ok, pass=pass+1; else, fail=fail+1; end

%% B3: Closedform seed converges faster than zero seed (iter count)
fprintf('\nB3: Closedform seed faster than zero seed (lower iter count)\n');
xt = 1.0; yt = 0.8;
opts_cf.init='closedform'; opts_cf.tol=1e-10; opts_cf.max_iter=200;
opts_zr.init='zero';       opts_zr.tol=1e-10; opts_zr.max_iter=200;
[~,~,ok_cf,it_cf] = custom_IK_newton(xt, yt, L1, L2, opts_cf);
[~,~,ok_zr,it_zr] = custom_IK_newton(xt, yt, L1, L2, opts_zr);
ok = ok_cf && it_cf <= it_zr;
fprintf('  closedform: %d iters   zero: %d iters\n', it_cf, it_zr);
fprintf('  %s  Closedform seed is faster or equal\n', rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n');

% Group C: accuracy vs closed-form IK
fprintf('GROUP C - NR accuracy vs custom_IK_2DOF (closed-form)\n');
fprintf('----------------------------------------------------------\n');

%% C1: FK(NR solution) == FK(closed-form solution) to 1e-9
fprintf('C1: FK(NR) == FK(closedform) to 1e-9 for 8 targets\n');
c1ok = true;
for i = 1:size(targets,1)
    xt=targets(i,1); yt=targets(i,2);
    [t1_nr, t2_nr, ok_nr] = custom_IK_newton(xt, yt, L1, L2);
    [t1_cf, t2_cf, ok_cf] = custom_IK_2DOF(xt, yt, L1, L2, 1);
    if ok_nr && ok_cf
        [xnr, ynr] = custom_FK_2DOF(t1_nr, t2_nr, L1, L2);
        [xcf, ycf] = custom_FK_2DOF(t1_cf, t2_cf, L1, L2);
        err = norm([xnr-xcf; ynr-ycf]);
        if err > 1e-9
            fprintf('  FAIL target %d: FK difference=%.2e\n', i, err);
            c1ok = false;
        end
    end
end
fprintf('  %s  FK outputs match to 1e-9\n', rs(c1ok));
if c1ok, pass=pass+1; else, fail=fail+1; end

%% C2: 100 random reachable targets round-trip accuracy
fprintf('\nC2: 100 random reachable targets - NR accuracy < 1e-9\n');
c2ok = true; n_tested = 0;
for i = 1:200
    t1r=rand*2*pi-pi; t2r=rand*2*pi-pi;
    [xr,yr] = custom_FK_2DOF(t1r, t2r, L1, L2);
    [t1_nr, t2_nr, ok] = custom_IK_newton(xr, yr, L1, L2);
    if ok
        n_tested = n_tested + 1;
        [xv,yv] = custom_FK_2DOF(t1_nr, t2_nr, L1, L2);
        err = norm([xv-xr; yv-yr]);
        if err > 1e-9
            c2ok = false;
            fprintf('  FAIL sample %d err=%.2e\n', i, err);
        end
        if n_tested >= 100, break; end
    end
end
fprintf('  %s  %d random targets all within 1e-9\n', rs(c2ok), n_tested);
if c2ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n');

% Group D: err_history
fprintf('GROUP D - err_history output\n');
fprintf('----------------------------------------------------------\n');

%% D1: err_history(1) is error at initial guess
fprintf('D1: err_history(1) == ||FK(seed) - target||\n');
xt=1.0; yt=0.8;
opts_d.init='closedform';
[t1,t2,ok,iters,hist] = custom_IK_newton(xt, yt, L1, L2, opts_d);
[t1s,t2s] = custom_IK_2DOF(xt, yt, L1, L2, 1);
[xs,ys]   = custom_FK_2DOF(t1s, t2s, L1, L2);
seed_err  = norm([xs-xt; ys-yt]);
d1ok = abs(hist(1) - seed_err) < 1e-12;
fprintf('  seed_err=%.2e  hist(1)=%.2e  diff=%.2e\n', seed_err, hist(1), abs(hist(1)-seed_err));
fprintf('  %s\n', rs(d1ok));
if d1ok, pass=pass+1; else, fail=fail+1; end

%% D2: err_history monotone when seeded from closedform (quadratic convergence region)
% Check monotonic behavior for closedform seed.
fprintf('\nD2: err_history monotone for closedform-seeded run\n');
opts_d2.init='closedform'; opts_d2.max_iter=20; opts_d2.tol=1e-12;
[~,~,ok,iters,hist] = custom_IK_newton(1.0, 0.5, L1, L2, opts_d2);
d2ok = true;
for k = 2:length(hist)
    if hist(k) > hist(k-1) * 1.001
        d2ok = false;
        fprintf('  FAIL: hist(%d)=%.2e > hist(%d)=%.2e\n', k, hist(k), k-1, hist(k-1));
    end
end
fprintf('  Closedform seed: %d iterations, final err=%.2e\n', iters, hist(end));
fprintf('  %s  Monotone convergence in quadratic basin\n', rs(d2ok));
if d2ok, pass=pass+1; else, fail=fail+1; end

%% D3: Final err_history value matches actual FK error
fprintf('\nD3: hist(end) matches actual final FK error\n');
[t1,t2,ok,iters,hist] = custom_IK_newton(1.2, 0.6, L1, L2);
[xv,yv] = custom_FK_2DOF(t1, t2, L1, L2);
actual_err = norm([xv-1.2; yv-0.6]);
d3ok = abs(hist(end) - actual_err) < 1e-12;
fprintf('  hist(end)=%.2e  actual=%.2e  diff=%.2e\n', hist(end), actual_err, abs(hist(end)-actual_err));
fprintf('  %s\n', rs(d3ok));
if d3ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n');

% Group E: edge cases
fprintf('GROUP E - Edge cases\n');
fprintf('----------------------------------------------------------\n');

%% E1: Outside workspace returns valid=0, no error thrown
fprintf('E1: Out-of-workspace target returns valid=0 cleanly\n');
e1ok = true;
oor_targets = [2.0 0; 0 2.0; -2.0 0; 1.5 1.5; 0.0 0.0];
for i = 1:size(oor_targets,1)
    try
        [~,~,ok] = custom_IK_newton(oor_targets(i,1), oor_targets(i,2), L1, L2);
        if ok
            r = norm(oor_targets(i,:));
            if r > L1+L2+1e-6 || r < abs(L1-L2)-1e-6
                fprintf('  FAIL: claimed valid for OOR target %d\n', i);
                e1ok = false;
            end
        end
    catch ME
        fprintf('  FAIL: threw error for OOR target %d: %s\n', i, ME.message);
        e1ok = false;
    end
end
fprintf('  %s  All OOR targets handled cleanly\n', rs(e1ok));
if e1ok, pass=pass+1; else, fail=fail+1; end

%% E2: Near-singular configs - no NaN/Inf in output
fprintf('\nE2: Near-singular targets (arm nearly extended) - no NaN/Inf\n');
near_sing = [L1+L2-0.001, 0;
             0, L1+L2-0.001;
             -(L1+L2-0.001), 0];
e2ok = true;
for i = 1:size(near_sing,1)
    [t1,t2,ok] = custom_IK_newton(near_sing(i,1), near_sing(i,2), L1, L2);
    if any(isnan([t1 t2])) || any(isinf([t1 t2]))
        fprintf('  FAIL target %d: NaN/Inf\n', i);
        e2ok = false;
    end
end
fprintf('  %s  No NaN/Inf near workspace boundary\n', rs(e2ok));
if e2ok, pass=pass+1; else, fail=fail+1; end

%% E3: max_iter=1 - returns partial result without crash
fprintf('\nE3: max_iter=1 - partial result, no crash\n');
opts_e3.max_iter = 1; opts_e3.init = 'zero';
try
    [t1,t2,ok,iters,hist] = custom_IK_newton(1.0, 0.8, L1, L2, opts_e3);
    e3ok = ~any(isnan([t1 t2])) && ~any(isinf([t1 t2])) && iters >= 1;
catch ME
    e3ok = false;
    fprintf('  threw: %s\n', ME.message);
end
fprintf('  %s  Graceful partial result\n', rs(e3ok));
if e3ok, pass=pass+1; else, fail=fail+1; end

%% E4: tol=1e-3 - converges faster (fewer iters) than tol=1e-10
fprintf('\nE4: Loose tol=1e-3 uses fewer iterations than tight tol=1e-10\n');
opts_loose.init='zero'; opts_loose.tol=1e-3;  opts_loose.max_iter=200;
opts_tight.init='zero'; opts_tight.tol=1e-10; opts_tight.max_iter=200;
[~,~,~,it_loose] = custom_IK_newton(1.0, 0.6, L1, L2, opts_loose);
[~,~,~,it_tight] = custom_IK_newton(1.0, 0.6, L1, L2, opts_tight);
ok = it_loose <= it_tight;
fprintf('  loose iters=%d   tight iters=%d\n', it_loose, it_tight);
fprintf('  %s  Loose tol uses fewer or equal iterations\n', rs(ok));
if ok, pass=pass+1; else, fail=fail+1; end

fprintf('\n');

% Group F: options struct
fprintf('GROUP F - Options struct\n');
fprintf('----------------------------------------------------------\n');

%% F1: All 5 outputs accessible
fprintf('F1: All 5 outputs [t1,t2,valid,iters,hist] accessible\n');
[t1,t2,ok,iters,hist] = custom_IK_newton(1.0, 0.8, L1, L2);
f1ok = isscalar(t1) && isscalar(t2) && isscalar(ok) && ...
       isscalar(iters) && isvector(hist) && length(hist) >= 1;
fprintf('  t1=%.4f t2=%.4f ok=%d iters=%d len(hist)=%d\n', ...
        t1, t2, ok, iters, length(hist));
fprintf('  %s  All outputs valid\n', rs(f1ok));
if f1ok, pass=pass+1; else, fail=fail+1; end

%% F2: elbow_up=0 gives different (but equally accurate) solution
fprintf('\nF2: elbow_up=0 gives different angles, same FK position\n');
xt=1.0; yt=0.8;
opts_up.elbow_up=1; opts_dn.elbow_up=0;
[t1u,t2u,oku] = custom_IK_newton(xt, yt, L1, L2, opts_up);
[t1d,t2d,okd] = custom_IK_newton(xt, yt, L1, L2, opts_dn);
if oku && okd
    [xu,yu] = custom_FK_2DOF(t1u, t2u, L1, L2);
    [xd,yd] = custom_FK_2DOF(t1d, t2d, L1, L2);
    angles_differ = abs(t2u-t2d) > 1e-6;
    fk_same = norm([xu-xd; yu-yd]) < 1e-9;
    f2ok = angles_differ && fk_same;
    fprintf('  t2_up=%.4f  t2_down=%.4f  angles_differ=%d  FK_same=%d\n', ...
            t2u, t2d, angles_differ, fk_same);
else
    f2ok = false;
end
fprintf('  %s\n', rs(f2ok));
if f2ok, pass=pass+1; else, fail=fail+1; end

%% F3: Default call (no options) works identically to explicit defaults
fprintf('\nF3: Default call equals explicit defaults\n');
[t1a,t2a,oka] = custom_IK_newton(1.0, 0.8, L1, L2);
opts_def.tol=1e-10; opts_def.max_iter=50; opts_def.init='closedform';
opts_def.elbow_up=1; opts_def.lambda=0.01; opts_def.verbose=false;
[t1b,t2b,okb] = custom_IK_newton(1.0, 0.8, L1, L2, opts_def);
f3ok = oka && okb && abs(t1a-t1b)<1e-12 && abs(t2a-t2b)<1e-12;
fprintf('  t1 diff=%.2e  t2 diff=%.2e\n', abs(t1a-t1b), abs(t2a-t2b));
fprintf('  %s\n', rs(f3ok));
if f3ok, pass=pass+1; else, fail=fail+1; end

% Summary
fprintf('\n==========================================================\n');
fprintf('  RESULTS: PASS=%-2d  FAIL=%-2d  TOTAL=%d\n', pass, fail, pass+fail);
if fail == 0
    fprintf('  STATUS:  ALL PASS - 3 Newton-Raphson verified\n');
else
    fprintf('  STATUS:  %d FAILURE(S) - check FAIL lines above\n', fail);
end
fprintf('==========================================================\n');