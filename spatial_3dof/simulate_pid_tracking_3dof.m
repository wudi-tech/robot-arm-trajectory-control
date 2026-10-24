function result = simulate_pid_tracking_3dof(traj, params)
%SIMULATE_PID_TRACKING_3DOF Track a 3-joint trajectory with shared PID.

t = traj.t(:);
n = length(t);
q = zeros(n, 3);
dq = zeros(n, 3);
err = zeros(n, 3);
u = zeros(n, 3);
integral_err = [0, 0, 0];

q(1, :) = traj.q(1, :);

for i = 1:n-1
    dt_i = t(i+1) - t(i);
    err(i, :) = traj.q(i, :) - q(i, :);
    integral_err = integral_err + err(i, :) * dt_i;
    if i == 1
        derivative_err = [0, 0, 0];
    else
        derivative_err = (err(i, :) - err(i-1, :)) / dt_i;
    end
    u(i, :) = params.Kp .* err(i, :) + params.Ki .* integral_err + params.Kd .* derivative_err;
    ddq_actual = (u(i, :) - params.B .* dq(i, :)) ./ params.J;
    dq(i+1, :) = dq(i, :) + ddq_actual * dt_i;
    q(i+1, :) = q(i, :) + dq(i+1, :) * dt_i;
end

err(end, :) = traj.q(end, :) - q(end, :);
if n > 1
    u(end, :) = u(end-1, :);
end

[x_d, y_d, z_d] = fk_3dof(traj.q(:, 1), traj.q(:, 2), traj.q(:, 3), params);
[x, y, z] = fk_3dof(q(:, 1), q(:, 2), q(:, 3), params);
[x1_d, y1_d, z1_d] = elbow_position_3dof(traj.q(:, 1), traj.q(:, 2), params);
[x1, y1, z1] = elbow_position_3dof(q(:, 1), q(:, 2), params);

result.name = traj.name;
result.method_id = traj.method_id;
result.space = traj.space;
result.t = t;
result.q = q;
result.dq = dq;
result.error = err;
result.u = u;
result.x1_d = x1_d;
result.y1_d = y1_d;
result.z1_d = z1_d;
result.x_d = x_d;
result.y_d = y_d;
result.z_d = z_d;
result.x1 = x1;
result.y1 = y1;
result.z1 = z1;
result.x = x;
result.y = y;
result.z = z;
end

function [x1, y1, z1] = elbow_position_3dof(q1, q2, params)
L1 = params.link_lengths(1);
r1 = L1 * cos(q2);
x1 = r1 .* cos(q1);
y1 = r1 .* sin(q1);
z1 = params.base_height + L1 * sin(q2);
end
