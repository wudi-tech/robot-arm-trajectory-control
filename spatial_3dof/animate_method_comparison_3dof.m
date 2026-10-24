function animate_method_comparison_3dof(results, waypoints, params, output_file)
%ANIMATE_METHOD_COMPARISON_3DOF Create a synchronized 3D method comparison MP4.

if nargin < 4
    output_file = 'animation_3dof_cubic_quintic_trap_comparison.mp4';
end

fig = figure('Name', '三自由度三种轨迹规划动画对比', ...
    'Position', [40, 80, 1600, 620]);

video = VideoWriter(output_file, 'MPEG-4');
video.FrameRate = params.video_frame_rate;
open(video);

frame_count = length(results{1}.t);
for i = 1:params.video_step:frame_count
    figure(fig);
    clf;

    for n = 1:numel(results)
        subplot(1, numel(results), n);
        draw_3dof_comparison_panel(results{n}, waypoints, params, i);
    end

    drawnow;
    frame = getframe(fig);
    writeVideo(video, frame);
end

close(video);
close(fig);
end

function draw_3dof_comparison_panel(result, waypoints, params, i)
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

plot3(desired(:, 1), desired(:, 2), desired(:, 3), ...
    'b--o', 'LineWidth', 1.7, 'MarkerSize', 4, 'MarkerFaceColor', 'b');
plot3(actual(:, 1), actual(:, 2), actual(:, 3), ...
    'r-o', 'LineWidth', 2.5, 'MarkerSize', 5, 'MarkerFaceColor', 'r');
plot3(result.x_d(1:i), result.y_d(1:i), result.z_d(1:i), ...
    'b:', 'LineWidth', 1.1);
plot3(result.x(1:i), result.y(1:i), result.z(1:i), ...
    'r-', 'LineWidth', 1.1);
plot3(waypoints.xyz(:, 1), waypoints.xyz(:, 2), waypoints.xyz(:, 3), ...
    'k:', 'LineWidth', 1.0);
plot3(waypoints.xyz(1:3, 1), waypoints.xyz(1:3, 2), waypoints.xyz(1:3, 3), ...
    'ks', 'MarkerSize', 7, 'MarkerFaceColor', 'y');

for k = 1:3
    text(waypoints.xyz(k, 1), waypoints.xyz(k, 2), waypoints.xyz(k, 3) + 0.05, ...
        waypoints.labels{k}, 'FontSize', 9);
end

plot3([0 0.35], [0 0], [params.base_height params.base_height], 'r-', 'LineWidth', 2);
plot3([0 0], [0 0.35], [params.base_height params.base_height], 'g-', 'LineWidth', 2);
plot3([0 0], [0 0], [0 params.base_height + 0.35], 'b-', 'LineWidth', 2);

title({result.name; ['t = ', num2str(result.t(i), '%.2f'), ' s']});
xlabel('X / m');
ylabel('Y / m');
zlabel('Z / m');
xlim([-1.2 1.3]);
ylim([-0.2 1.4]);
zlim([0 1.35]);
end
