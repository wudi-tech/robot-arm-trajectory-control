function animate_robot(result, waypoints, params, output_file, method_title)
%ANIMATE_ROBOT Create an MP4 animation for one tracked trajectory result.

if nargin < 4
    output_file = 'two_link_robot_animation.mp4';
end
if nargin < 5 || isempty(method_title)
    method_title = result.name;
end

fig = figure('Name', '两自由度机械臂搬运分拣动画', ...
    'Position', [100, 100, 1000, 750]);

video = VideoWriter(output_file, 'MPEG-4');
video.FrameRate = params.video_frame_rate;
open(video);

for i = 1:params.video_step:length(result.t)
    figure(fig);
    cla;
    hold on;

    rectangle('Position', [0.4, 1.2, 0.5, 0.08], ...
        'FaceColor', [0.85 0.85 0.85], 'EdgeColor', 'k');
    text(0.45, 1.33, '传送带取料区', 'FontSize', 10);

    rectangle('Position', [0.0, 1.6, 0.45, 0.15], ...
        'FaceColor', [0.9 0.9 0.9], 'EdgeColor', 'k');
    text(0.05, 1.82, '放料盒', 'FontSize', 10);

    h1 = plot([0 result.x1_d(i)], [0 result.y1_d(i)], ...
        'b--', 'LineWidth', 2);
    plot([result.x1_d(i) result.x2_d(i)], ...
        [result.y1_d(i) result.y2_d(i)], 'b--', 'LineWidth', 2);

    h2 = plot([0 result.x1(i)], [0 result.y1(i)], ...
        'r-', 'LineWidth', 3);
    plot([result.x1(i) result.x2(i)], ...
        [result.y1(i) result.y2(i)], 'r-', 'LineWidth', 3);

    h3 = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    plot(result.x1(i), result.y1(i), 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'k');
    h4 = plot(result.x2(i), result.y2(i), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');

    h5 = plot(result.x2_d(1:i), result.y2_d(1:i), 'b:', 'LineWidth', 1.5);
    h6 = plot(result.x2(1:i), result.y2(1:i), 'r-', 'LineWidth', 1.5);

    for k = 1:3
        [~, ~, px, py] = fk_2link(waypoints.rad(k, 1), waypoints.rad(k, 2), params);
        plot(px, py, 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
        text(px + 0.03, py + 0.03, waypoints.labels{k}, 'FontSize', 10);
    end

    legend([h1, h2, h3, h4, h5, h6], ...
        {'理论机械臂', '实际机械臂', '基座/关节', '末端执行器', ...
        '理论末端轨迹', '实际末端轨迹'}, 'Location', 'northwest');

    title([method_title, '  t = ', num2str(result.t(i), '%.2f'), ' s']);
    xlabel('X / m');
    ylabel('Y / m');
    xlim([-0.5 2.0]);
    ylim([-0.2 2.0]);
    axis equal;
    grid on;

    drawnow;
    frame = getframe(fig);
    writeVideo(video, frame);
end

close(video);
close(fig);
end
