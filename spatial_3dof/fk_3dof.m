function [x, y, z, r] = fk_3dof(q1, q2, q3, params)
%FK_3DOF Forward kinematics for yaw-pitch-pitch spatial arm.

L1 = params.link_lengths(1);
L2 = params.link_lengths(2);

r = L1 * cos(q2) + L2 * cos(q2 + q3);
x = r .* cos(q1);
y = r .* sin(q1);
z = params.base_height + L1 * sin(q2) + L2 * sin(q2 + q3);
end
