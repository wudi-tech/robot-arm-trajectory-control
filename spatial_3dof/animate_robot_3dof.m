function animate_robot_3dof(result, waypoints, params, output_file, method_title)
%ANIMATE_ROBOT_3DOF Create one 3D MP4 animation for a tracked trajectory.

if nargin < 4
    output_file = 'three_dof_robot_animation.mp4';
end
if nargin < 5 || isempty(method_title)
    method_title = result.name;
end

fig = figure('Name', '三自由度空间机械臂动画', ...
    'Position', [100, 80, 1000, 760]);

video = VideoWriter(output_file, 'MPEG-4');
video.FrameRate = params.video_frame_rate;
open(video);

for i = 1:params.video_step:length(result.t)
    figure(fig);
    clf;
    draw_3dof_panel(result, waypoints, params, i, method_title, true);

    drawnow;
    frame = getframe(fig);
    writeVideo(video, frame);
end

close(video);
close(fig);
end

function draw_3dof_panel(result, waypoints, params, i, method_title, show_legend)
hold on;
grid on;
axis equal;
view(45, 25);

desired = [
    0, 0, 0;
    0, 0, params.base_height;
    result.x1_d(i), result.y1_d(i), result.z1_d(i);
    result.x_d(i), result.y_d(i), result.z_d(i)
];
actual = [
    0, 0, 0;
    0, 0, params.base_height;
    result.x1(i), result.y1(i), result.z1(i);
    result.x(i), result.y(i), result.z(i)
];

h_desired_arm = plot3(desired(:, 1), desired(:, 2), desired(:, 3), ...
    'b--o', 'LineWidth', 2, 'MarkerSize', 5, 'MarkerFaceColor', 'b');
h_actual_arm = plot3(actual(:, 1), actual(:, 2), actual(:, 3), ...
    'r-o', 'LineWidth', 3, 'MarkerSize', 6, 'MarkerFaceColor', 'r');
h_desired_trace = plot3(result.x_d(1:i), result.y_d(1:i), result.z_d(1:i), ...
    'b:', 'LineWidth', 1.4);
h_actual_trace = plot3(result.x(1:i), result.y(1:i), result.z(1:i), ...
    'r-', 'LineWidth', 1.4);
h_waypoints = plot3(waypoints.xyz(1:3, 1), waypoints.xyz(1:3, 2), waypoints.xyz(1:3, 3), ...
    'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
plot3(waypoints.xyz(:, 1), waypoints.xyz(:, 2), waypoints.xyz(:, 3), ...
    'k:', 'LineWidth', 1.1);

for k = 1:3
    text(waypoints.xyz(k, 1), waypoints.xyz(k, 2), waypoints.xyz(k, 3) + 0.05, ...
        waypoints.labels{k}, 'FontSize', 10);
end

plot3([0 0.35], [0 0], [params.base_height params.base_height], 'r-', 'LineWidth', 2);
plot3([0 0], [0 0.35], [params.base_height params.base_height], 'g-', 'LineWidth', 2);
plot3([0 0], [0 0], [0 params.base_height + 0.35], 'b-', 'LineWidth', 2);

title({method_title; ['t = ', num2str(result.t(i), '%.2f'), ' s']});
xlabel('X / m');
ylabel('Y / m');
zlabel('Z / m');
xlim([-1.2 1.3]);
ylim([-0.2 1.4]);
zlim([0 1.35]);

if show_legend
    legend([h_desired_arm, h_actual_arm, h_desired_trace, h_actual_trace, h_waypoints], ...
        {'理论机械臂', '实际机械臂', '理论末端轨迹', '实际末端轨迹', '任务点'}, ...
        'Location', 'northeastoutside');
end
end
