%% SETUP_SIMULINK.m
%  Run this script FIRST whenever you open MATLAB for this project.
%  It adds all files to MATLAB's path so Simulink can find them.
%
%  HOW TO USE:
%   1. Open MATLAB
%   2. In the top bar click the small folder icon → navigate to this folder
%   3. Press F5 to run this file
%   4. Command Window should print "Ready. Now open arm_2dof.slx"
%   5. Then open arm_2dof.slx

% Add simulink and kinematics folders to MATLAB path
simulink_folder = fileparts(mfilename('fullpath'));
project_root = fileparts(simulink_folder);
kinematics_folder = fullfile(project_root, 'kinematics');

addpath(simulink_folder);
addpath(kinematics_folder);
fprintf('Path added: %s\n', simulink_folder);
fprintf('Path added: %s\n', kinematics_folder);

% Quick self-check
L1 = 1.0; L2 = 0.8;
[t1, t2, ok] = custom_IK_2DOF(1.0, 0.8, L1, L2, 1);
[xv, yv]     = custom_FK_2DOF(t1, t2, L1, L2);
err          = sqrt((xv-1.0)^2 + (yv-0.8)^2);

if ok && err < 1e-10
    fprintf('Self-check PASSED  (FK/IK error = %.2e)\n', err);
    fprintf('\nReady. Now open arm_2dof.slx\n');
else
    fprintf('Self-check FAILED  (err=%.2e, ok=%d)\n', err, ok);
    fprintf('Make sure kinematics files are in the kinematics folder.\n');
end
