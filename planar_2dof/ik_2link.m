function q = ik_2link(x, y, params, elbow_branch)
%IK_2LINK Solve inverse kinematics for a planar two-link arm.

if nargin < 4
    elbow_branch = 'positive';
end

x = x(:);
y = y(:);

c2 = (x.^2 + y.^2 - params.L1^2 - params.L2^2) / (2 * params.L1 * params.L2);
tol = 1e-10;

if any(c2 > 1 + tol) || any(c2 < -1 - tol)
    error('Cartesian point is outside the reachable workspace.');
end

c2 = min(max(c2, -1), 1);

if strcmpi(elbow_branch, 'negative') || strcmpi(elbow_branch, 'elbow_up')
    theta2 = -acos(c2);
else
    theta2 = acos(c2);
end

theta1 = atan2(y, x) - atan2(params.L2 * sin(theta2), params.L1 + params.L2 * cos(theta2));

q = [theta1, theta2];
end
