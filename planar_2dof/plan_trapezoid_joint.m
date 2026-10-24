function traj = plan_trapezoid_joint(waypoints_rad, params)
%PLAN_TRAPEZOID_JOINT Generate joint-space trapezoidal velocity trajectories.

ta = params.trap_accel_ratio * params.Tseg;
if ta <= 0 || ta >= params.Tseg / 2
    error('trap_accel_ratio must make acceleration time between 0 and Tseg/2.');
end

t_all = [];
q_all = [];
dq_all = [];
ddq_all = [];

current_time = 0;

for k = 1:size(waypoints_rad, 1)-1
    q_start = waypoints_rad(k, :);
    q_end = waypoints_rad(k+1, :);
    delta_q = q_end - q_start;

    t_seg = (0:params.dt:params.Tseg)';
    [h, dh, ddh] = normalized_trapezoid_profile(t_seg, params.Tseg, ta);

    q_seg = q_start + h .* delta_q;
    dq_seg = dh .* delta_q;
    ddq_seg = ddh .* delta_q;

    if k > 1
        t_seg = t_seg(2:end);
        q_seg = q_seg(2:end, :);
        dq_seg = dq_seg(2:end, :);
        ddq_seg = ddq_seg(2:end, :);
    end

    t_all = [t_all; current_time + t_seg];
    q_all = [q_all; q_seg];
    dq_all = [dq_all; dq_seg];
    ddq_all = [ddq_all; ddq_seg];

    current_time = current_time + params.Tseg;
end

traj.name = '梯形速度规划';
traj.method_id = 'trapezoid_joint';
traj.space = 'joint';
traj.t = t_all;
traj.q = q_all;
traj.dq = dq_all;
traj.ddq = ddq_all;
end

function [h, dh, ddh] = normalized_trapezoid_profile(t, total_time, accel_time)
v = 1 / (total_time - accel_time);
a = v / accel_time;

h = zeros(size(t));
dh = zeros(size(t));
ddh = zeros(size(t));

idx_acc = t < accel_time;
idx_flat = t >= accel_time & t <= total_time - accel_time;
idx_dec = t > total_time - accel_time;

h(idx_acc) = 0.5 * a * t(idx_acc).^2;
dh(idx_acc) = a * t(idx_acc);
ddh(idx_acc) = a;

h(idx_flat) = 0.5 * a * accel_time^2 + v * (t(idx_flat) - accel_time);
dh(idx_flat) = v;
ddh(idx_flat) = 0;

t_remaining = total_time - t(idx_dec);
h(idx_dec) = 1 - 0.5 * a * t_remaining.^2;
dh(idx_dec) = a * t_remaining;
ddh(idx_dec) = -a;

h(end) = 1;
dh(end) = 0;
ddh(end) = -a;
end
