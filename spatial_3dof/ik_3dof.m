function q = ik_3dof(x, y, z, params, elbow_branch)
%IK_3DOF Inverse kinematics for yaw-pitch-pitch spatial arm.

if nargin < 5
    elbow_branch = 'positive';
end

x = x(:);
y = y(:);
z = z(:);

L1 = params.link_lengths(1);
L2 = params.link_lengths(2);

q1 = atan2(y, x);
r = sqrt(x.^2 + y.^2);
z_rel = z - params.base_height;

c3 = (r.^2 + z_rel.^2 - L1^2 - L2^2) / (2 * L1 * L2);
tol = 1e-10;
if any(c3 > 1 + tol) || any(c3 < -1 - tol)
    error('Spatial target is outside the reachable workspace.');
end
c3 = min(max(c3, -1), 1);

if strcmpi(elbow_branch, 'negative') || strcmpi(elbow_branch, 'elbow_up')
    q3 = -acos(c3);
else
    q3 = acos(c3);
end

q2 = atan2(z_rel, r) - atan2(L2 * sin(q3), L1 + L2 * cos(q3));
q = [q1, q2, q3];
end
