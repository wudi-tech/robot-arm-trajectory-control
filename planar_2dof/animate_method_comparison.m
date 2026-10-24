function animate_method_comparison(results, waypoints, params, output_file)
%ANIMATE_METHOD_COMPARISON Create a synchronized 2-DOF method comparison MP4.

if nargin < 4
    output_file = 'animation_2dof_cubic_quintic_trap_comparison.mp4';
end

method_titles = cell(1, numel(results));
for n = 1:numel(results)
    method_titles{n} = results{n}.name;
end

fig = figure('Name', '两自由度三种轨迹规划动画对比', ...
    'Position', [50, 80, 1500, 560]);

video = VideoWriter(output_file, 'MPEG-4');
video.FrameRate = params.video_frame_rate;
open(video);

frame_count = length(results{1}.t);
for i = 1:params.video_step:frame_count
    figure(fig);
    clf;

    for n = 1:numel(results)
        subplot(1, numel(results), n);
        draw_2dof_panel(results{n}, waypoints, params, i, method_titles{n});
    end

    drawnow;
    frame = getframe(fig);
    writeVideo(video, frame);
end

close(video);
close(fig);
end

function draw_2dof_panel(result, waypoints, params, i, method_title)
hold on;

rectangle('Position', [0.4, 1.2, 0.5, 0.08], ...
    'FaceColor', [0.85 0.85 0.85], 'EdgeColor', 'k');
text(0.45, 1.33, '传送带取料区', 'FontSize', 9);

rectangle('Position', [0.0, 1.6, 0.45, 0.15], ...
    'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'k');
text(0.05, 1.82, '放料盒', 'FontSize', 9);

plot([0 result.x1_d(i)], [0 result.y1_d(i)], 'b--', 'LineWidth', 1.8);
plot([result.x1_d(i) result.x2_d(i)], ...
    [result.y1_d(i) result.y2_d(i)], 'b--', 'LineWidth', 1.8);

plot([0 result.x1(i)], [0 result.y1(i)], 'r-', 'LineWidth', 2.8);
plot([result.x1(i) result.x2(i)], ...
    [result.y1(i) result.y2(i)], 'r-', 'LineWidth', 2.8);

plot(0, 0, 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'k');
plot(result.x1(i), result.y1(i), 'ko', 'MarkerSize', 6, 'MarkerFaceColor', 'k');
plot(result.x2(i), result.y2(i), 'ro', 'MarkerSize', 7, 'MarkerFaceColor', 'r');

plot(result.x2_d(1:i), result.y2_d(1:i), 'b:', 'LineWidth', 1.2);
plot(result.x2(1:i), result.y2(1:i), 'r-', 'LineWidth', 1.2);

for k = 1:3
    [~, ~, px, py] = fk_2link(waypoints.rad(k, 1), waypoints.rad(k, 2), params);
    plot(px, py, 'ks', 'MarkerSize', 7, 'MarkerFaceColor', 'y');
    text(px + 0.03, py + 0.03, waypoints.labels{k}, 'FontSize', 9);
end

title({method_title; ['t = ', num2str(result.t(i), '%.2f'), ' s']});
xlabel('X / m');
ylabel('Y / m');
xlim([-0.5 2.0]);
ylim([-0.2 2.0]);
axis equal;
grid on;
end
