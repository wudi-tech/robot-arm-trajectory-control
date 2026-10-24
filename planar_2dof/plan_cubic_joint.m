function traj = plan_cubic_joint(waypoints_rad, params)
%PLAN_CUBIC_JOINT Generate joint-space cubic polynomial trajectories.

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
    s = t_seg / params.Tseg;

    h = 3*s.^2 - 2*s.^3;
    dh = 6*t_seg/params.Tseg^2 - 6*t_seg.^2/params.Tseg^3;
    ddh = 6/params.Tseg^2 - 12*t_seg/params.Tseg^3;

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

traj.name = '三次多项式';
traj.method_id = 'cubic_joint';
traj.space = 'joint';
traj.t = t_all;
traj.q = q_all;
traj.dq = dq_all;
traj.ddq = ddq_all;
end
