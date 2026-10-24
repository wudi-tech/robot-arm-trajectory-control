clear; clc;

params = init_params();
waypoints = init_waypoints();

assert(params.L1 == 1.0);
assert(params.L2 == 0.8);
assert(size(waypoints.rad, 1) == 4);
assert(size(waypoints.rad, 2) == 2);

traj = plan_quintic_joint(waypoints.rad, params);
assert(isfield(traj, 't'));
assert(isfield(traj, 'q'));
assert(isfield(traj, 'dq'));
assert(isfield(traj, 'ddq'));
assert(size(traj.q, 2) == 2);
assert(length(traj.t) == size(traj.q, 1));

traj_cubic = plan_cubic_joint(waypoints.rad, params);
assert(isfield(traj_cubic, 't'));
assert(isfield(traj_cubic, 'q'));
assert(isfield(traj_cubic, 'dq'));
assert(isfield(traj_cubic, 'ddq'));
assert(size(traj_cubic.q, 2) == 2);
assert(length(traj_cubic.t) == length(traj.t));

traj_trap = plan_trapezoid_joint(waypoints.rad, params);
assert(isfield(traj_trap, 't'));
assert(isfield(traj_trap, 'q'));
assert(isfield(traj_trap, 'dq'));
assert(isfield(traj_trap, 'ddq'));
assert(size(traj_trap.q, 2) == 2);
assert(length(traj_trap.t) == length(traj.t));

[~, ~, x_start, y_start] = fk_2link(waypoints.rad(1, 1), waypoints.rad(1, 2), params);
q_start_ik = ik_2link(x_start, y_start, params);
q_start_error = atan2(sin(q_start_ik(1, :) - waypoints.rad(1, :)), ...
    cos(q_start_ik(1, :) - waypoints.rad(1, :)));
assert(max(abs(q_start_error)) < 1e-10);

traj_cart = plan_cartesian_quintic(waypoints.rad, params);
assert(isfield(traj_cart, 't'));
assert(isfield(traj_cart, 'q'));
assert(isfield(traj_cart, 'dq'));
assert(isfield(traj_cart, 'ddq'));
assert(isfield(traj_cart, 'x_d'));
assert(isfield(traj_cart, 'y_d'));
assert(size(traj_cart.q, 2) == 2);
assert(length(traj_cart.t) == length(traj.t));

result = simulate_pid_tracking(traj, params);
assert(length(result.t) == length(traj.t));
assert(size(result.q, 2) == 2);
assert(size(result.u, 2) == 2);

metrics = compute_metrics(traj, result, params);
assert(isfield(metrics, 'joint1_rmse_deg'));
assert(isfield(metrics, 'joint2_rmse_deg'));
assert(isfield(metrics, 'ee_rmse_m'));
assert(metrics.joint1_rmse_deg >= 0);
assert(metrics.joint2_rmse_deg >= 0);
assert(metrics.ee_rmse_m >= 0);
assert(exist('animate_method_comparison', 'file') == 2);

disp('All advanced checks passed.');
