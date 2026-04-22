function [x, y, x1, y1, T_total] = custom_FK_2DOF(theta1, theta2, L1, L2, method)
% CUSTOM_FK_2DOF  Forward Kinematics — 2-DOF planar serial arm
%
%   USAGE:
%     [x, y] = custom_FK_2DOF(t1, t2, L1, L2)              % geometric (default)
%     [x, y] = custom_FK_2DOF(t1, t2, L1, L2, 'dh')        % DH matrix
%     [x, y] = custom_FK_2DOF(t1, t2, L1, L2, 'rotchain')  % rotation chain
%     [x, y, x1, y1, T] = custom_FK_2DOF(...)               % all outputs
%
%   INPUTS:
%     theta1  joint 1 angle (rad)
%     theta2  joint 2 angle (rad), measured relative to link 1
%     L1, L2  link lengths (m)
%     method  'geometric' (default) | 'dh' | 'rotchain'
%
%   OUTPUTS:
%     x, y     end-effector position (m)
%     x1, y1   elbow (joint 2) position (m)
%     T_total  4x4 homogeneous transform base->EE (populated for 'dh' and 'rotchain')
%
%   THREE ALGORITHMS — NO TOOLBOX USED:
%
%   geometric  : direct trig chain. Fastest (4 trig ops). Default.
%   dh         : DH 4x4 matrix chain. Returns full pose + orientation.
%                Extends to 3-DOF by just adding one more T matrix.
%   rotchain   : explicit 2x2 rotation matrices. Clearest for teaching.
%
%   WHY FASTER THAN MATLAB BUILT-IN:
%     getTransform() in Robotics Toolbox allocates a rigidBodyTree object
%     and runs object-oriented overhead per call. Our functions are plain
%     trig — no object allocation, no toolbox license required.

    if nargin < 5, method = 'geometric'; end
    T_total = eye(4);
    x1 = 0; y1 = 0; x = 0; y = 0;

    switch lower(method)

        % METHOD 1: GEOMETRIC (default, fast)
        % x = L1*cos(t1) + L2*cos(t1+t2)
        % y = L1*sin(t1) + L2*sin(t1+t2)
        case 'geometric'
            x1 = L1 * cos(theta1);
            y1 = L1 * sin(theta1);
            x  = x1 + L2 * cos(theta1 + theta2);
            y  = y1 + L2 * sin(theta1 + theta2);

        % METHOD 2: DH HOMOGENEOUS TRANSFORM (4x4)
        % Standard Denavit-Hartenberg for planar arm
        % T_total = T1 * T2
        case 'dh'
            T1 = [cos(theta1) -sin(theta1) 0  L1*cos(theta1);
                  sin(theta1)  cos(theta1) 0  L1*sin(theta1);
                  0            0           1  0;
                  0            0           0  1];

            T2 = [cos(theta2) -sin(theta2) 0  L2*cos(theta2);
                  sin(theta2)  cos(theta2) 0  L2*sin(theta2);
                  0            0           1  0;
                  0            0           0  1];

            T_total = T1 * T2;

            x1 = T1(1,4);      y1 = T1(2,4);
            x  = T_total(1,4); y  = T_total(2,4);

        % METHOD 3: ROTATION MATRIX CHAIN (SE2)
        % p_EE = R1*[L1;0] + R1*R2*[L2;0]
        case 'rotchain'
            R1 = [cos(theta1) -sin(theta1); sin(theta1) cos(theta1)];
            R2 = [cos(theta2) -sin(theta2); sin(theta2) cos(theta2)];

            p1 = R1 * [L1; 0];
            p2 = p1 + R1 * R2 * [L2; 0];

            x1 = p1(1);  y1 = p1(2);
            x  = p2(1);  y  = p2(2);

            R_tot   = R1 * R2;
            T_total = [R_tot(1,1) R_tot(1,2) 0 x;
                       R_tot(2,1) R_tot(2,2) 0 y;
                       0          0          1 0;
                       0          0          0 1];

        otherwise
            error('Unknown method "%s". Choose: geometric | dh | rotchain', method);
    end

end
