function plot_comparison(trajectories, results, waypoints, params)
%PLOT_COMPARISON Draw planning and tracking comparison figures.

joint_idx = [];
for i = 1:numel(trajectories)
    if strcmp(trajectories{i}.space, 'joint')
        joint_idx(end+1) = i; %#ok<AGROW>
    end
end

plot_joint_quantity(trajectories, joint_idx, 'q', 180/pi, ...
    '关节角度对比', '角度 / deg');
plot_joint_quantity(trajectories, joint_idx, 'dq', 180/pi, ...
    '关节角速度对比', '角速度 / (deg/s)');
plot_joint_quantity(trajectories, joint_idx, 'ddq', 180/pi, ...
    '关节角加速度对比', '角加速度 / (deg/s^2)');
plot_tracking_errors(results, joint_idx);
plot_control_torques(results, joint_idx);
plot_workspace_paths(trajectories, results, waypoints, params);
end

function plot_joint_quantity(trajectories, joint_idx, field_name, scale, fig_name, y_label)
figure('Name', fig_name);
colors = lines(numel(joint_idx));
names = cell(1, numel(joint_idx));

for joint = 1:2
    subplot(2, 1, joint);
    hold on;
    for n = 1:numel(joint_idx)
        idx = joint_idx(n);
        data = trajectories{idx}.(field_name);
        plot(trajectories{idx}.t, data(:, joint) * scale, ...
            'LineWidth', 1.8, 'Color', colors(n, :));
        names{n} = trajectories{idx}.name;
    end
    grid on;
    xlabel('时间 / s');
    ylabel(y_label);
    title(['关节 ', num2str(joint), ' ', fig_name]);
    legend(names, 'Location', 'best');
end
end

function plot_tracking_errors(results, joint_idx)
figure('Name', 'PID 跟踪误差对比');
colors = lines(numel(joint_idx));
names = cell(1, numel(joint_idx));

for joint = 1:2
    subplot(2, 1, joint);
    hold on;
    for n = 1:numel(joint_idx)
        idx = joint_idx(n);
        plot(results{idx}.t, results{idx}.error(:, joint) * 180/pi, ...
            'LineWidth', 1.8, 'Color', colors(n, :));
        names{n} = results{idx}.name;
    end
    grid on;
    xlabel('时间 / s');
    ylabel('误差 / deg');
    title(['关节 ', num2str(joint), ' PID 跟踪误差']);
    legend(names, 'Location', 'best');
end
end

function plot_control_torques(results, joint_idx)
figure('Name', 'PID 控制力矩对比');
colors = lines(numel(joint_idx));
names = cell(1, numel(joint_idx));

for joint = 1:2
    subplot(2, 1, joint);
    hold on;
    for n = 1:numel(joint_idx)
        idx = joint_idx(n);
        plot(results{idx}.t, results{idx}.u(:, joint), ...
            'LineWidth', 1.8, 'Color', colors(n, :));
        names{n} = results{idx}.name;
    end
    grid on;
    xlabel('时间 / s');
    ylabel('控制力矩 / N*m');
    title(['关节 ', num2str(joint), ' PID 控制力矩']);
    legend(names, 'Location', 'best');
end
end

function plot_workspace_paths(trajectories, results, waypoints, params)
quintic_idx = find_method_index(trajectories, 'quintic_joint');
cart_idx = find_method_index(trajectories, 'cartesian_quintic');

figure('Name', '关节空间与笛卡尔空间末端轨迹对比');
hold on;

plot(results{quintic_idx}.x2_d, results{quintic_idx}.y2_d, ...
    'b-', 'LineWidth', 2);
plot(results{quintic_idx}.x2, results{quintic_idx}.y2, ...
    'b--', 'LineWidth', 1.6);

if isfield(trajectories{cart_idx}, 'x_d')
    plot(trajectories{cart_idx}.x_d, trajectories{cart_idx}.y_d, ...
        'r-', 'LineWidth', 2);
else
    plot(results{cart_idx}.x2_d, results{cart_idx}.y2_d, ...
        'r-', 'LineWidth', 2);
end
plot(results{cart_idx}.x2, results{cart_idx}.y2, ...
    'r--', 'LineWidth', 1.6);

[~, ~, waypoint_x, waypoint_y] = fk_2link(waypoints.rad(:, 1), waypoints.rad(:, 2), params);
plot(waypoint_x, waypoint_y, 'k:', 'LineWidth', 1.4);
plot(waypoint_x(1:3), waypoint_y(1:3), 'ko', ...
    'MarkerSize', 8, 'MarkerFaceColor', 'y');

for k = 1:3
    text(waypoint_x(k) + 0.03, waypoint_y(k) + 0.03, ...
        waypoints.labels{k}, 'FontSize', 10);
end

axis equal;
grid on;
xlabel('X / m');
ylabel('Y / m');
title('关节空间规划 vs 笛卡尔空间规划末端轨迹');
legend({'关节空间期望轨迹', '关节空间实际轨迹', ...
    '笛卡尔空间期望轨迹', '笛卡尔空间实际轨迹', ...
    '任务点直线连接', '任务点'}, 'Location', 'best');
end

function idx = find_method_index(trajectories, method_id)
idx = [];
for i = 1:numel(trajectories)
    if strcmp(trajectories{i}.method_id, method_id)
        idx = i;
        return;
    end
end

error(['Cannot find trajectory method: ', method_id]);
end
