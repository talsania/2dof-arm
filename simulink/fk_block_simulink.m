function [x_ee, y_ee, x_elbow, y_elbow] = fk_block_simulink(theta1, theta2)
% FK_BLOCK_SIMULINK  Drop this function INSIDE a Simulink MATLAB Function block.
%
%   COPY-PASTE THIS ENTIRE FILE into the MATLAB Function block editor.
%   Do NOT call it from outside — Simulink calls it automatically.
%
%   Inputs  (connected as Simulink input ports):
%     theta1    joint 1 angle in radians
%     theta2    joint 2 angle in radians
%
%   Outputs (connected as Simulink output ports):
%     x_ee      end-effector X position (m)
%     y_ee      end-effector Y position (m)
%     x_elbow   elbow X position (m)
%     y_elbow   elbow Y position (m)
%
%   LINK LENGTHS — change these to match your arm:
    L1 = 1.0;   % link 1 length (m)
    L2 = 0.8;   % link 2 length (m)

%   METHOD: Geometric (direct trig) — no toolbox, no object overhead.
    x_elbow = L1 * cos(theta1);
    y_elbow = L1 * sin(theta1);
    x_ee    = x_elbow + L2 * cos(theta1 + theta2);
    y_ee    = y_elbow + L2 * sin(theta1 + theta2);

end
