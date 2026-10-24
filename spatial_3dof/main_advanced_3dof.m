clear; clc; close all;

% 三自由度空间机械臂多轨迹规划方法与 PID 跟踪仿真

params = init_params_3dof();
waypoints = init_waypoints_3dof(params);

traj_cubic = plan_cubic_joint_3dof(waypoints.q, params);
traj_quintic = plan_quintic_joint_3dof(waypoints.q, params);
traj_trap = plan_trapezoid_joint_3dof(waypoints.q, params);
traj_cart = plan_cartesian_quintic_3dof(waypoints.xyz, params);

trajectories = {traj_cubic, traj_quintic, traj_trap, traj_cart};
results = cell(size(trajectories));
metrics = cell(size(trajectories));

for i = 1:numel(trajectories)
    results{i} = simulate_pid_tracking_3dof(trajectories{i}, params);
    metrics{i} = compute_metrics_3dof(trajectories{i}, results{i});
end

metrics_table = struct2table([metrics{:}]);

disp('三自由度空间机械臂轨迹规划方法评价指标：');
disp(metrics_table);

figure_output_dir = fullfile(pwd, 'matlab_comparison_figures');
plot_comparison_3dof(trajectories, results, waypoints, figure_output_dir);
export_pybullet_csv_3dof(traj_quintic, results{2}, 'quintic_3dof_pybullet.csv');
export_pybullet_csv_3dof(traj_cart, results{4}, 'cartesian_3dof_pybullet.csv');
animate_robot_3dof(results{1}, waypoints, params, ...
    'animation_3dof_cubic_joint.mp4', '三次多项式');
animate_robot_3dof(results{2}, waypoints, params, ...
    'animation_3dof_quintic_joint.mp4', '五次多项式');
animate_robot_3dof(results{3}, waypoints, params, ...
    'animation_3dof_trapezoid_joint.mp4', '梯形速度规划');
animate_method_comparison_3dof(results(1:3), waypoints, params, ...
    'animation_3dof_cubic_quintic_trap_comparison.mp4');

disp(['MATLAB 对比图已保存到 ', figure_output_dir]);
disp('PyBullet 轨迹文件已保存为 quintic_3dof_pybullet.csv');
disp('笛卡尔空间 PyBullet 轨迹文件已保存为 cartesian_3dof_pybullet.csv');
disp('3-DOF 三种规划方法动画已保存。');
close all;
