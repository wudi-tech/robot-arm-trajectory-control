clear; clc; close all;

% 多轨迹规划方法与规划空间对比的两自由度机械臂 PID 跟踪仿真

params = init_params();
waypoints = init_waypoints();

traj_cubic = plan_cubic_joint(waypoints.rad, params);
traj_quintic = plan_quintic_joint(waypoints.rad, params);
traj_trap = plan_trapezoid_joint(waypoints.rad, params);
traj_cart = plan_cartesian_quintic(waypoints.rad, params);

trajectories = {traj_cubic, traj_quintic, traj_trap, traj_cart};
results = cell(size(trajectories));
metrics = cell(size(trajectories));

for i = 1:numel(trajectories)
    results{i} = simulate_pid_tracking(trajectories{i}, params);
    metrics{i} = compute_metrics(trajectories{i}, results{i}, params);
end

metrics_table = struct2table([metrics{:}]);

disp('两自由度机械臂轨迹规划方法评价指标：');
disp(metrics_table);

plot_comparison(trajectories, results, waypoints, params);
animate_robot(results{1}, waypoints, params, ...
    'animation_2dof_cubic_joint.mp4', '三次多项式');
animate_robot(results{2}, waypoints, params, ...
    'animation_2dof_quintic_joint.mp4', '五次多项式');
animate_robot(results{3}, waypoints, params, ...
    'animation_2dof_trapezoid_joint.mp4', '梯形速度规划');
animate_method_comparison(results(1:3), waypoints, params, ...
    'animation_2dof_cubic_quintic_trap_comparison.mp4');

disp('2-DOF 三种规划方法动画已保存。');
close all;
