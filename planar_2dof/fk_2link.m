function [x1, y1, x2, y2] = fk_2link(theta1, theta2, params)
%FK_2LINK Compute forward kinematics for a planar two-link arm.

x1 = params.L1 * cos(theta1);
y1 = params.L1 * sin(theta1);

x2 = x1 + params.L2 * cos(theta1 + theta2);
y2 = y1 + params.L2 * sin(theta1 + theta2);
end
