%% BUILD_SIMULINK_MODEL.m
%  Builds TWO Simulink models automatically.
%  Fixed: code injection happens BEFORE wiring so ports exist.
%    1. Run setup_simulink.m first (just once per MATLAB session)
%    2. Press F5 on this file
%    3. Two Simulink windows open automatically
%    4. In EACH window press Ctrl+T to run the simulation
%    5. Then run tests/verify_simulink.m to check results

% Ensure we're in the simulink folder and save models here
simulink_folder = fileparts(mfilename('fullpath'));
original_folder = pwd;
cd(simulink_folder);

% Shared code strings for IK and FK blocks

ik_code = [...
'function [theta1, theta2] = IK_Block(x_target, y_target)', newline, ...
'L1 = 1.0; L2 = 0.8;', newline, ...
'r     = sqrt(x_target^2 + y_target^2);', newline, ...
'r_max = L1 + L2;', newline, ...
'r_min = abs(L1 - L2);', newline, ...
'r_s   = max(r_min + 1e-6, min(r_max - 1e-6, r));', newline, ...
'if r > 1e-9', newline, ...
'    xs = x_target * (r_s / r);', newline, ...
'    ys = y_target * (r_s / r);', newline, ...
'else', newline, ...
'    xs = r_s; ys = 0;', newline, ...
'end', newline, ...
'c2     = max(-1, min(1, (xs^2 + ys^2 - L1^2 - L2^2) / (2*L1*L2)));', newline, ...
'theta2 = atan2(sqrt(1 - c2^2), c2);', newline, ...
'alpha  = atan2(ys, xs);', newline, ...
'beta   = atan2(L2*sin(theta2), L1 + L2*cos(theta2));', newline, ...
'theta1 = atan2(sin(alpha - beta), cos(alpha - beta));', newline, ...
'end'];

fk_code = [...
'function [x_ee, y_ee] = FK_Block(theta1, theta2)', newline, ...
'L1 = 1.0; L2 = 0.8;', newline, ...
'x_ee = L1*cos(theta1) + L2*cos(theta1 + theta2);', newline, ...
'y_ee = L1*sin(theta1) + L2*sin(theta1 + theta2);', newline, ...
'end'];

% MODEL 1: Static test
% Constant target (x=1.0, y=0.8) → IK → FK → Display blocks
% Run time: 0.1 s
mdl1 = 'arm_static';
if bdIsLoaded(mdl1), close_system(mdl1, 0); end
if exist([mdl1 '.slx'], 'file'), delete([mdl1 '.slx']); end
new_system(mdl1);
open_system(mdl1);
fprintf('\n=== Building Model 1: %s ===\n', mdl1);

% A: Add all blocks
add_block('simulink/Sources/Constant', [mdl1 '/x_target'], ...
    'Value', '1.0', 'Position', [30 60 90 90]);

add_block('simulink/Sources/Constant', [mdl1 '/y_target'], ...
    'Value', '0.8', 'Position', [30 150 90 180]);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl1 '/IK_Block'], 'Position', [170 80 310 170]);

add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl1 '/FK_Block'], 'Position', [390 80 530 170]);

add_block('simulink/Sinks/Display', [mdl1 '/Display_x'], ...
    'Position', [610 70 720 100]);

add_block('simulink/Sinks/Display', [mdl1 '/Display_y'], ...
    'Position', [610 120 720 150]);

add_block('simulink/Sinks/Display', [mdl1 '/Display_t1'], ...
    'Position', [390 190 500 220]);

add_block('simulink/Sinks/Display', [mdl1 '/Display_t2'], ...
    'Position', [390 235 500 265]);

add_block('simulink/Sinks/To Workspace', [mdl1 '/ee_out'], ...
    'VariableName', 'ee_static', 'SaveFormat', 'Array', ...
    'Position', [610 165 720 195]);

fprintf('  Blocks added\n');

% B: Inject code BEFORE wiring (so ports exist)
try
    rt     = sfroot();
    m      = rt.find('-isa', 'Simulink.BlockDiagram', 'Name', mdl1);
    blk_ik = m.find('-isa', 'Stateflow.EMChart', 'Path', [mdl1 '/IK_Block']);
    blk_fk = m.find('-isa', 'Stateflow.EMChart', 'Path', [mdl1 '/FK_Block']);
    if ~isempty(blk_ik), blk_ik.Script = ik_code;
        fprintf('  IK code injected\n');
    end
    if ~isempty(blk_fk), blk_fk.Script = fk_code;
        fprintf('  FK code injected\n');
    end
catch ME
    fprintf('  WARNING: auto code inject failed (%s)\n', ME.message);
    fprintf('  --> Manually paste ik_block_simulink.m into IK_Block\n');
    fprintf('  --> Manually paste fk_block_simulink.m into FK_Block\n');
end

% C: save so ports refresh
save_system(mdl1);
pause(1);   % give Simulink a moment to register the new ports

% D: wire signals
try
    add_line(mdl1, 'x_target/1', 'IK_Block/1', 'autorouting', 'on');
    add_line(mdl1, 'y_target/1', 'IK_Block/2', 'autorouting', 'on');
    add_line(mdl1, 'IK_Block/1', 'FK_Block/1', 'autorouting', 'on');
    add_line(mdl1, 'IK_Block/2', 'FK_Block/2', 'autorouting', 'on');
    add_line(mdl1, 'FK_Block/1', 'Display_x/1', 'autorouting', 'on');
    add_line(mdl1, 'FK_Block/2', 'Display_y/1', 'autorouting', 'on');
    add_line(mdl1, 'IK_Block/1', 'Display_t1/1', 'autorouting', 'on');
    add_line(mdl1, 'IK_Block/2', 'Display_t2/1', 'autorouting', 'on');
    add_line(mdl1, 'FK_Block/1', 'ee_out/1', 'autorouting', 'on');
    fprintf('  Signals wired\n');
catch ME
    fprintf('  WARNING: auto-wiring failed: %s\n', ME.message);
    fprintf('  [WARN] Connect blocks manually in the Simulink canvas\n');
end

% E: Configure solver
set_param(mdl1, 'StopTime', '0.1', 'Solver', 'ode45', 'MaxStep', '0.01');

save_system(mdl1);
fprintf('  [OK] Saved: %s.slx\n', mdl1);
fprintf('  [INFO] Press Ctrl+T in this Simulink window to run\n');
fprintf('  [INFO] Display_x should show 1.00000\n');
fprintf('  [INFO] Display_y should show 0.80000\n');

% MODEL 2: Trajectory test
% Sine wave target → IK → Memory → FK → XY Graph + Scope
% Run time: 20 s
mdl2 = 'arm_trajectory';
if bdIsLoaded(mdl2), close_system(mdl2, 0); end
if exist([mdl2 '.slx'], 'file'), delete([mdl2 '.slx']); end
new_system(mdl2);
open_system(mdl2);
fprintf('\n=== Building Model 2: %s ===\n', mdl2);

% A: Add blocks
% x target: sine wave, moves between 0.6 and 1.4
add_block('simulink/Sources/Sine Wave', [mdl2 '/x_wave'], ...
    'Amplitude', '0.4', 'Bias', '1.0', 'Frequency', '0.5', ...
    '', '0', 'Position', [30 60 90 90]);

% y target: constant 0.6
add_block('simulink/Sources/Constant', [mdl2 '/y_const'], ...
    'Value', '0.6', 'Position', [30 140 90 170]);

% IK block
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl2 '/IK_Block'], 'Position', [160 75 300 155]);

% Memory blocks - hold angles one time (correct for continuous IK)
add_block('simulink/Discrete/Memory', [mdl2 '/Mem_t1'], ...
    'InitialCondition', '0', 'Position', [370 65 420 95]);
add_block('simulink/Discrete/Memory', [mdl2 '/Mem_t2'], ...
    'InitialCondition', '0', 'Position', [370 135 420 165]);

% FK block
add_block('simulink/User-Defined Functions/MATLAB Function', ...
    [mdl2 '/FK_Block'], 'Position', [480 80 620 160]);

% XY Graph - shows EE (x,y) path
add_block('simulink/Sinks/XY Graph', [mdl2 '/XY_EE'], ...
    'xmin', '-2', 'xmax', '2', 'ymin', '-2', 'ymax', '2', ...
    'Position', [690 60 760 130]);

% Scope - shows joint angles over time
add_block('simulink/Sinks/Scope', [mdl2 '/Scope_Angles'], ...
    'NumInputPorts', '2', 'Position', [690 150 740 200]);

% To Workspace
add_block('simulink/Sinks/To Workspace', [mdl2 '/ee_traj'], ...
    'VariableName', 'ee_traj', 'SaveFormat', 'Array', ...
    'Position', [690 215 790 245]);

fprintf('  Blocks added\n');

% B: Inject code BEFORE wiring
try
    rt     = sfroot();
    m      = rt.find('-isa', 'Simulink.BlockDiagram', 'Name', mdl2);
    blk_ik = m.find('-isa', 'Stateflow.EMChart', 'Path', [mdl2 '/IK_Block']);
    blk_fk = m.find('-isa', 'Stateflow.EMChart', 'Path', [mdl2 '/FK_Block']);
    if ~isempty(blk_ik), blk_ik.Script = ik_code;
        fprintf('  IK code injected\n');
    end
    if ~isempty(blk_fk), blk_fk.Script = fk_code;
        fprintf('  FK code injected\n');
    end
catch ME
    fprintf('  WARNING: auto code inject failed (%s)\n', ME.message);
    fprintf('  [WARN] Manually paste code into IK_Block and FK_Block\n');
end

% C: save so ports refresh
save_system(mdl2);
pause(1);

% D: wire signals
try
    add_line(mdl2, 'x_wave/1',     'IK_Block/1',     'autorouting', 'on');
    add_line(mdl2, 'y_const/1',    'IK_Block/2',     'autorouting', 'on');
    add_line(mdl2, 'IK_Block/1',   'Mem_t1/1',       'autorouting', 'on');
    add_line(mdl2, 'IK_Block/2',   'Mem_t2/1',       'autorouting', 'on');
    add_line(mdl2, 'Mem_t1/1',     'FK_Block/1',     'autorouting', 'on');
    add_line(mdl2, 'Mem_t2/1',     'FK_Block/2',     'autorouting', 'on');
    add_line(mdl2, 'FK_Block/1',   'XY_EE/1',        'autorouting', 'on');
    add_line(mdl2, 'FK_Block/2',   'XY_EE/2',        'autorouting', 'on');
    add_line(mdl2, 'Mem_t1/1',     'Scope_Angles/1', 'autorouting', 'on');
    add_line(mdl2, 'Mem_t2/1',     'Scope_Angles/2', 'autorouting', 'on');
    add_line(mdl2, 'FK_Block/1',   'ee_traj/1',      'autorouting', 'on');
    fprintf('  Signals wired\n');
catch ME
    fprintf('  WARNING: auto-wiring failed: %s\n', ME.message);
    fprintf('  --> Connect blocks manually in the Simulink canvas\n');
end

% E: Configure solver
set_param(mdl2, 'StopTime', '20', 'Solver', 'ode45', ...
    'MaxStep', '0.05');

save_system(mdl2);
fprintf('  [OK] Saved: %s.slx\n', mdl2);
fprintf('  [INFO] Press Ctrl+T in this Simulink window to run\n');
fprintf('  [INFO] XY Graph shows arm path (horizontal line)\n');
fprintf('  [INFO] Scope shows joint angles over time\n');

% Return to original folder
cd(original_folder);

% Final message
fprintf('\n[==========] Both models built\n\n');
fprintf('1. simulink/arm_static.slx     Press Ctrl+T\n');
fprintf('   Display_x = 1.00000\n');
fprintf('   Display_y = 0.80000\n\n');
fprintf('2. simulink/arm_trajectory.slx Press Ctrl+T\n');
fprintf('   XY Graph traces a path\n');
fprintf('   Scope shows two angle curves\n\n');
fprintf('3. Then run tests/verify_simulink.m\n');
fprintf('[==========]\n');