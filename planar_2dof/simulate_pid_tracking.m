function result = simulate_pid_tracking(traj, params)
%SIMULATE_PID_TRACKING Track a desired joint trajectory with shared PID.

t = traj.t(:);
n = length(t);

q = zeros(n, 2);
dq = zeros(n, 2);
err = zeros(n, 2);
u = zeros(n, 2);
integral_err = [0, 0];

q(1, :) = traj.q(1, :);

for i = 1:n-1
    dt_i = t(i+1) - t(i);

    err(i, :) = traj.q(i, :) - q(i, :);
    integral_err = integral_err + err(i, :) * dt_i;

    if i == 1
        derivative_err = [0, 0];
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

[x1_d, y1_d, x2_d, y2_d] = fk_2link(traj.q(:, 1), traj.q(:, 2), params);
[x1, y1, x2, y2] = fk_2link(q(:, 1), q(:, 2), params);

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
result.x2_d = x2_d;
result.y2_d = y2_d;
result.x1 = x1;
result.y1 = y1;
result.x2 = x2;
result.y2 = y2;
end
