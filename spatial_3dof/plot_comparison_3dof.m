function plot_comparison_3dof(trajectories, results, waypoints, output_dir)
%PLOT_COMPARISON_3DOF Draw MATLAB comparison figures for 3-DOF project.

if nargin < 4
    output_dir = '';
end
if ~isempty(output_dir) && ~exist(output_dir, 'dir')
    mkdir(output_dir);
end

joint_idx = [];
for i = 1:numel(trajectories)
    if strcmp(trajectories{i}.space, 'joint')
        joint_idx(end+1) = i; %#ok<AGROW>
    end
end

fig = plot_joint_series_3dof(trajectories, joint_idx, 'q', 180/pi, '关节角度对比', '角度 / deg');
save_comparison_figure(fig, output_dir, '01_joint_angle_comparison');
fig = plot_joint_series_3dof(trajectories, joint_idx, 'dq', 180/pi, '关节角速度对比', '角速度 / (deg/s)');
save_comparison_figure(fig, output_dir, '02_joint_velocity_comparison');
fig = plot_joint_series_3dof(trajectories, joint_idx, 'ddq', 180/pi, '关节角加速度对比', '角加速度 / (deg/s^2)');
save_comparison_figure(fig, output_dir, '03_joint_acceleration_comparison');
fig = plot_error_series_3dof(results, joint_idx);
save_comparison_figure(fig, output_dir, '04_pid_tracking_error_comparison');
fig = plot_torque_series_3dof(results, joint_idx);
save_comparison_figure(fig, output_dir, '05_pid_control_torque_comparison');
fig = plot_3d_path_comparison(trajectories, results, waypoints);
save_comparison_figure(fig, output_dir, '06_end_effector_3d_path_comparison');
end

function fig = plot_joint_series_3dof(trajectories, joint_idx, field_name, scale, fig_name, y_label)
fig = figure('Name', fig_name);
colors = lines(numel(joint_idx));
for joint = 1:3
    subplot(3, 1, joint);
    hold on;
    names = cell(1, numel(joint_idx));
    for n = 1:numel(joint_idx)
        idx = joint_idx(n);
        data = trajectories{idx}.(field_name);
        plot(trajectories{idx}.t, data(:, joint) * scale, 'LineWidth', 1.5, 'Color', colors(n, :));
        names{n} = trajectories{idx}.name;
    end
    grid on;
    xlabel('时间 / s');
    ylabel(y_label);
    title(['关节 ', num2str(joint), ' ', fig_name]);
    legend(names, 'Location', 'best');
end
end

function fig = plot_error_series_3dof(results, joint_idx)
fig = figure('Name', '三关节 PID 跟踪误差对比');
colors = lines(numel(joint_idx));
for joint = 1:3
    subplot(3, 1, joint);
    hold on;
    names = cell(1, numel(joint_idx));
    for n = 1:numel(joint_idx)
        idx = joint_idx(n);
        plot(results{idx}.t, results{idx}.error(:, joint) * 180/pi, 'LineWidth', 1.5, 'Color', colors(n, :));
        names{n} = results{idx}.name;
    end
    grid on;
    xlabel('时间 / s');
    ylabel('误差 / deg');
    title(['关节 ', num2str(joint), ' PID 跟踪误差']);
    legend(names, 'Location', 'best');
end
end

function fig = plot_torque_series_3dof(results, joint_idx)
fig = figure('Name', '三关节 PID 控制力矩对比');
colors = lines(numel(joint_idx));
for joint = 1:3
    subplot(3, 1, joint);
    hold on;
    names = cell(1, numel(joint_idx));
    for n = 1:numel(joint_idx)
        idx = joint_idx(n);
        plot(results{idx}.t, results{idx}.u(:, joint), 'LineWidth', 1.5, 'Color', colors(n, :));
        names{n} = results{idx}.name;
    end
    grid on;
    xlabel('时间 / s');
    ylabel('力矩 / N*m');
    title(['关节 ', num2str(joint), ' PID 控制力矩']);
    legend(names, 'Location', 'best');
end
end

function fig = plot_3d_path_comparison(trajectories, results, waypoints)
fig = figure('Name', '三维末端轨迹对比');
hold on;
grid on;
axis equal;

quintic_idx = find_method_index_3dof(trajectories, 'quintic_joint_3dof');
cart_idx = find_method_index_3dof(trajectories, 'cartesian_quintic_3dof');

plot3(results{quintic_idx}.x_d, results{quintic_idx}.y_d, results{quintic_idx}.z_d, 'b-', 'LineWidth', 2);
plot3(results{quintic_idx}.x, results{quintic_idx}.y, results{quintic_idx}.z, 'b--', 'LineWidth', 1.5);
plot3(results{cart_idx}.x_d, results{cart_idx}.y_d, results{cart_idx}.z_d, 'r-', 'LineWidth', 2);
plot3(results{cart_idx}.x, results{cart_idx}.y, results{cart_idx}.z, 'r--', 'LineWidth', 1.5);
plot3(waypoints.xyz(:, 1), waypoints.xyz(:, 2), waypoints.xyz(:, 3), 'k:', 'LineWidth', 1.3);
plot3(waypoints.xyz(1:3, 1), waypoints.xyz(1:3, 2), waypoints.xyz(1:3, 3), 'ko', 'MarkerFaceColor', 'y', 'MarkerSize', 8);

for k = 1:3
    text(waypoints.xyz(k, 1), waypoints.xyz(k, 2), waypoints.xyz(k, 3) + 0.04, waypoints.labels{k}, 'FontSize', 10);
end

xlabel('X / m');
ylabel('Y / m');
zlabel('Z / m');
title('关节空间五次规划 vs 三维笛卡尔空间规划');
legend({'关节空间期望轨迹', '关节空间实际轨迹', '笛卡尔空间期望轨迹', '笛卡尔空间实际轨迹', '任务点连线', '任务点'}, 'Location', 'best');
view(45, 25);
end

function idx = find_method_index_3dof(trajectories, method_id)
idx = [];
for i = 1:numel(trajectories)
    if strcmp(trajectories{i}.method_id, method_id)
        idx = i;
        return;
    end
end
error(['Cannot find trajectory method: ', method_id]);
end

function save_comparison_figure(fig, output_dir, file_stem)
if isempty(output_dir)
    return;
end

fig_path = fullfile(output_dir, [file_stem, '.fig']);
png_path = fullfile(output_dir, [file_stem, '.png']);
savefig(fig, fig_path);
try
    exportgraphics(fig, png_path, 'Resolution', 200);
catch
    saveas(fig, png_path);
end
end
