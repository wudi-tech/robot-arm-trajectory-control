clear; clc;

params = init_params_3dof();
waypoints = init_waypoints_3dof(params);

assert(params.base_height > 0);
assert(numel(params.link_lengths) == 2);
assert(size(waypoints.xyz, 2) == 3);
assert(size(waypoints.q, 2) == 3);
assert(min(waypoints.xyz(1:3, 3)) >= 0.75);

[x0, y0, z0] = fk_3dof(waypoints.q(1, 1), waypoints.q(1, 2), waypoints.q(1, 3), params);
q0 = ik_3dof(x0, y0, z0, params);
q0_error = atan2(sin(q0(1, :) - waypoints.q(1, :)), cos(q0(1, :) - waypoints.q(1, :)));
assert(max(abs(q0_error)) < 1e-9);

traj_quintic = plan_quintic_joint_3dof(waypoints.q, params);
traj_cubic = plan_cubic_joint_3dof(waypoints.q, params);
traj_trap = plan_trapezoid_joint_3dof(waypoints.q, params);
traj_cart = plan_cartesian_quintic_3dof(waypoints.xyz, params);

trajs = {traj_quintic, traj_cubic, traj_trap, traj_cart};
for i = 1:numel(trajs)
    assert(isfield(trajs{i}, 't'));
    assert(isfield(trajs{i}, 'q'));
    assert(isfield(trajs{i}, 'dq'));
    assert(isfield(trajs{i}, 'ddq'));
    assert(size(trajs{i}.q, 2) == 3);
    assert(length(trajs{i}.t) == size(trajs{i}.q, 1));
    assert(length(trajs{i}.t) == length(traj_quintic.t));
end

result = simulate_pid_tracking_3dof(traj_quintic, params);
assert(length(result.t) == length(traj_quintic.t));
assert(size(result.q, 2) == 3);
assert(size(result.u, 2) == 3);

metrics = compute_metrics_3dof(traj_quintic, result);
assert(metrics.ee_rmse_m >= 0);
assert(metrics.joint1_rmse_deg >= 0);
assert(metrics.joint2_rmse_deg >= 0);
assert(metrics.joint3_rmse_deg >= 0);
assert(exist('animate_robot_3dof', 'file') == 2);
assert(exist('animate_method_comparison_3dof', 'file') == 2);
assert(nargin('plot_comparison_3dof') == 4);

disp('All 3-DOF checks passed.');
