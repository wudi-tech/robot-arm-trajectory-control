clear; clc; close all;

%  基于五次多项式插值与PID控制的两自由度机械臂搬运分拣仿真
%  任务：等待点A -> 取料点B -> 放料点C -> 返回等待点A

%% 1. 机械臂结构参数
L1 = 1.0;        % 连杆1长度 / m
L2 = 0.8;        % 连杆2长度 / m

%% 2. 关节简化动力学参数
J1 = 0.02;       % 关节1转动惯量
J2 = 0.01;       % 关节2转动惯量
B1 = 0.10;       % 关节1阻尼系数
B2 = 0.08;       % 关节2阻尼系数

%% 3. PID参数
Kp1 = 25;
Ki1 = 4;
Kd1 = 0.7;

Kp2 = 22;
Ki2 = 3;
Kd2 = 0.6;

%% 4. 任务点设置，单位为角度
% A：等待点
% B：取料点
% C：放料点

A = [30, 20];      % [theta1, theta2]
B = [55, 75];
C = [85, 35];

% 运动顺序：A -> B -> C -> A
waypoints_deg = [
    A;
    B;
    C;
    A
];

waypoints = waypoints_deg * pi / 180;    % 转换为弧度

%% 5. 每段运动时间设置
Tseg = 3;          % 每一段运动时间 / s
dt = 0.001;        % 仿真步长 / s

%% 6. 生成完整期望轨迹
t_all = [];
theta1_d_all = [];
theta2_d_all = [];
dtheta1_d_all = [];
dtheta2_d_all = [];
ddtheta1_d_all = [];
ddtheta2_d_all = [];

current_time = 0;

for k = 1:size(waypoints, 1)-1
    
    theta_start = waypoints(k, :);
    theta_end = waypoints(k+1, :);
    
    t_seg = 0:dt:Tseg;
    s = t_seg / Tseg;
    
    % 五次多项式插值函数
    h = 10*s.^3 - 15*s.^4 + 6*s.^5;
    dh = 30*t_seg.^2/Tseg^3 - 60*t_seg.^3/Tseg^4 + 30*t_seg.^4/Tseg^5;
    ddh = 60*t_seg/Tseg^3 - 180*t_seg.^2/Tseg^4 + 120*t_seg.^3/Tseg^5;
    
    % 关节1
    theta1_d = theta_start(1) + (theta_end(1) - theta_start(1)) .* h;
    dtheta1_d = (theta_end(1) - theta_start(1)) .* dh;
    ddtheta1_d = (theta_end(1) - theta_start(1)) .* ddh;
    
    % 关节2
    theta2_d = theta_start(2) + (theta_end(2) - theta_start(2)) .* h;
    dtheta2_d = (theta_end(2) - theta_start(2)) .* dh;
    ddtheta2_d = (theta_end(2) - theta_start(2)) .* ddh;
    
    % 避免段与段之间时间点重复
    if k > 1
        t_seg = t_seg(2:end);
        theta1_d = theta1_d(2:end);
        theta2_d = theta2_d(2:end);
        dtheta1_d = dtheta1_d(2:end);
        dtheta2_d = dtheta2_d(2:end);
        ddtheta1_d = ddtheta1_d(2:end);
        ddtheta2_d = ddtheta2_d(2:end);
    end
    
    t_all = [t_all, current_time + t_seg];
    
    theta1_d_all = [theta1_d_all, theta1_d];
    theta2_d_all = [theta2_d_all, theta2_d];
    
    dtheta1_d_all = [dtheta1_d_all, dtheta1_d];
    dtheta2_d_all = [dtheta2_d_all, dtheta2_d];
    
    ddtheta1_d_all = [ddtheta1_d_all, ddtheta1_d];
    ddtheta2_d_all = [ddtheta2_d_all, ddtheta2_d];
    
    current_time = current_time + Tseg;
end

t = t_all;
theta1_d = theta1_d_all;
theta2_d = theta2_d_all;

dtheta1_d = dtheta1_d_all;
dtheta2_d = dtheta2_d_all;

ddtheta1_d = ddtheta1_d_all;
ddtheta2_d = ddtheta2_d_all;

%% 7. 初始化实际关节变量
theta1 = zeros(size(t));
theta2 = zeros(size(t));

dtheta1 = zeros(size(t));
dtheta2 = zeros(size(t));

error1 = zeros(size(t));
error2 = zeros(size(t));

u1 = zeros(size(t));
u2 = zeros(size(t));

integral_error1 = 0;
integral_error2 = 0;

% 实际初始位置设置为等待点A
theta1(1) = waypoints(1, 1);
theta2(1) = waypoints(1, 2);

%% 8. PID轨迹跟踪控制仿真
for i = 1:length(t)-1
    
    % -------------------------
    % 关节1 PID控制
    % -------------------------
    error1(i) = theta1_d(i) - theta1(i);
    integral_error1 = integral_error1 + error1(i) * dt;
    
    if i == 1
        derivative_error1 = 0;
    else
        derivative_error1 = (error1(i) - error1(i-1)) / dt;
    end
    
    u1(i) = Kp1 * error1(i) + Ki1 * integral_error1 + Kd1 * derivative_error1;
    
    % -------------------------
    % 关节2 PID控制
    % -------------------------
    error2(i) = theta2_d(i) - theta2(i);
    integral_error2 = integral_error2 + error2(i) * dt;
    
    if i == 1
        derivative_error2 = 0;
    else
        derivative_error2 = (error2(i) - error2(i-1)) / dt;
    end
    
    u2(i) = Kp2 * error2(i) + Ki2 * integral_error2 + Kd2 * derivative_error2;
    
    % -------------------------
    % 简化关节动力学模型
    % J*ddtheta + B*dtheta = u
    % -------------------------
    ddtheta1 = (u1(i) - B1 * dtheta1(i)) / J1;
    ddtheta2 = (u2(i) - B2 * dtheta2(i)) / J2;
    
    % -------------------------
    % 数值积分
    % -------------------------
    dtheta1(i+1) = dtheta1(i) + ddtheta1 * dt;
    theta1(i+1) = theta1(i) + dtheta1(i+1) * dt;
    
    dtheta2(i+1) = dtheta2(i) + ddtheta2 * dt;
    theta2(i+1) = theta2(i) + dtheta2(i+1) * dt;
end

error1(end) = theta1_d(end) - theta1(end);
error2(end) = theta2_d(end) - theta2(end);

%% 9. 正运动学计算末端轨迹

% 理论机械臂位置
x1_d = L1 * cos(theta1_d);
y1_d = L1 * sin(theta1_d);

x2_d = x1_d + L2 * cos(theta1_d + theta2_d);
y2_d = y1_d + L2 * sin(theta1_d + theta2_d);

% 实际机械臂位置
x1 = L1 * cos(theta1);
y1 = L1 * sin(theta1);

x2 = x1 + L2 * cos(theta1 + theta2);
y2 = y1 + L2 * sin(theta1 + theta2);

%% 10. 绘制关节1轨迹对比

figure(1);
plot(t, theta1_d * 180/pi, 'LineWidth', 2);
hold on;
plot(t, theta1 * 180/pi, '--', 'LineWidth', 2);
grid on;
xlabel('时间 / s');
ylabel('关节1角度 / °');
title('关节1理论轨迹与实际轨迹对比');
legend('关节1理论轨迹', '关节1实际轨迹');

%% 11. 绘制关节2轨迹对比

figure(2);
plot(t, theta2_d * 180/pi, 'LineWidth', 2);
hold on;
plot(t, theta2 * 180/pi, '--', 'LineWidth', 2);
grid on;
xlabel('时间 / s');
ylabel('关节2角度 / °');
title('关节2理论轨迹与实际轨迹对比');
legend('关节2理论轨迹', '关节2实际轨迹');

%% 12. 绘制误差曲线

figure(3);
plot(t, error1 * 180/pi, 'LineWidth', 2);
hold on;
plot(t, error2 * 180/pi, 'LineWidth', 2);
grid on;
xlabel('时间 / s');
ylabel('跟踪误差 / °');
title('两关节轨迹跟踪误差');
legend('关节1误差', '关节2误差');

%% 13. 绘制末端轨迹

figure(4);
plot(x2_d, y2_d, 'LineWidth', 2);
hold on;
plot(x2, y2, '--', 'LineWidth', 2);

% 标记等待点、取料点、放料点
for k = 1:size(waypoints, 1)-1
    th1 = waypoints(k, 1);
    th2 = waypoints(k, 2);
    px = L1*cos(th1) + L2*cos(th1 + th2);
    py = L1*sin(th1) + L2*sin(th1 + th2);
    
    plot(px, py, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    text(px + 0.03, py + 0.03, ['P', num2str(k)], 'FontSize', 12);
end

grid on;
axis equal;
xlabel('X / m');
ylabel('Y / m');
title('机械臂末端理论轨迹与实际轨迹对比');
legend('理论末端轨迹', '实际末端轨迹', '任务点');

%% 14. 绘制PID控制力矩

figure(5);
plot(t, u1, 'LineWidth', 2);
hold on;
plot(t, u2, 'LineWidth', 2);
grid on;
xlabel('时间 / s');
ylabel('控制力矩 / N·m');
title('两关节PID控制力矩');
legend('关节1控制力矩', '关节2控制力矩');

%% 15. 绘制理论速度和加速度

figure(6);
plot(t, dtheta1_d * 180/pi, 'LineWidth', 2);
hold on;
plot(t, dtheta2_d * 180/pi, 'LineWidth', 2);
grid on;
xlabel('时间 / s');
ylabel('角速度 / (°/s)');
title('五次多项式规划的关节角速度');
legend('关节1理论角速度', '关节2理论角速度');

figure(7);
plot(t, ddtheta1_d * 180/pi, 'LineWidth', 2);
hold on;
plot(t, ddtheta2_d * 180/pi, 'LineWidth', 2);
grid on;
xlabel('时间 / s');
ylabel('角加速度 / (°/s^2)');
title('五次多项式规划的关节角加速度');
legend('关节1理论角加速度', '关节2理论角加速度');

%% 16. 两自由度机械臂搬运动画并保存为视频

figure(8);
set(gcf, 'Position', [100, 100, 1000, 750]);   % 固定窗口大小，防止视频帧尺寸变化

axis equal;
grid on;
xlim([-0.5 2.0]);
ylim([-0.2 2.0]);
xlabel('X / m');
ylabel('Y / m');
title('两自由度机械臂搬运分拣动画');

% 创建视频文件
video = VideoWriter('two_link_robot_animation.mp4', 'MPEG-4');
video.FrameRate = 15;
open(video);

step = 60;

for i = 1:step:length(t)
    
    figure(8);
    cla;
    hold on;
    
    % 当前理论位置
    xd1 = x1_d(i);
    yd1 = y1_d(i);
    xd2 = x2_d(i);
    yd2 = y2_d(i);
    
    % 当前实际位置
    xa1 = x1(i);
    ya1 = y1(i);
    xa2 = x2(i);
    ya2 = y2(i);
    
    % 绘制传送带和料盒
    rectangle('Position', [0.4, 1.2, 0.5, 0.08], ...
              'FaceColor', [0.85 0.85 0.85], ...
              'EdgeColor', 'k');
    text(0.45, 1.33, '传送带取料区', 'FontSize', 10);
    
    rectangle('Position', [0.0, 1.6, 0.45, 0.15], ...
              'FaceColor', [0.9 0.9 0.9], ...
              'EdgeColor', 'k');
    text(0.05, 1.82, '放料盒', 'FontSize', 10);
    
    % 理论机械臂
    h1 = plot([0 xd1], [0 yd1], 'b--', 'LineWidth', 2);
    plot([xd1 xd2], [yd1 yd2], 'b--', 'LineWidth', 2);
    
    % 实际机械臂
    h2 = plot([0 xa1], [0 ya1], 'r-', 'LineWidth', 3);
    plot([xa1 xa2], [ya1 ya2], 'r-', 'LineWidth', 3);
    
    % 关节和末端
    h3 = plot(0, 0, 'ko', 'MarkerSize', 8, 'MarkerFaceColor', 'k');
    plot(xa1, ya1, 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'k');
    h4 = plot(xa2, ya2, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    
    % 末端轨迹
    h5 = plot(x2_d(1:i), y2_d(1:i), 'b:', 'LineWidth', 1.5);
    h6 = plot(x2(1:i), y2(1:i), 'r-', 'LineWidth', 1.5);
    
    % 标记任务点
    labels = {'A 等待点', 'B 取料点', 'C 放料点'};
    for k = 1:3
        th1 = waypoints(k, 1);
        th2 = waypoints(k, 2);
        px = L1*cos(th1) + L2*cos(th1 + th2);
        py = L1*sin(th1) + L2*sin(th1 + th2);
        
        plot(px, py, 'ks', 'MarkerSize', 8, 'MarkerFaceColor', 'y');
        text(px + 0.03, py + 0.03, labels{k}, 'FontSize', 10);
    end
    
    % 图例
    legend([h1, h2, h3, h4, h5, h6], ...
           {'理论机械臂', '实际机械臂', '基座/关节', '末端执行器', ...
            '理论末端轨迹', '实际末端轨迹'}, ...
           'Location', 'northwest');
    
    title(['两自由度机械臂搬运分拣动画  t = ', num2str(t(i), '%.2f'), ' s']);
    
    xlim([-0.5 2.0]);
    ylim([-0.2 2.0]);
    axis equal;
    grid on;
    
    % 保存当前帧
    drawnow;
    frame = getframe(gcf);
    writeVideo(video, frame);
end

% 关闭视频文件
close(video);

disp('动画已保存为 two_link_robot_animation.mp4');