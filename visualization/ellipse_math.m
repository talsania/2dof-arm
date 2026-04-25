function [a_axis, b_axis, angle_deg, w] = ellipse_math(t1, t2, L1, L2)
% Compute ellipse axes, angle, and manipulability from Jacobian.
    s1=sin(t1); c1=cos(t1); s12=sin(t1+t2); c12=cos(t1+t2);
    J = [-L1*s1-L2*s12, -L2*s12; L1*c1+L2*c12, L2*c12];
    w = abs(det(J));
    [U,S,~] = svd(J);
    a_axis    = S(1,1);
    b_axis    = S(2,2);
    angle_deg = atan2(U(2,1), U(1,1)) * 180/pi;
end