function traj = plan_cartesian_quintic(waypoints_rad, params)
%PLAN_CARTESIAN_QUINTIC Plan straight Cartesian paths with quintic timing.

[~, ~, waypoint_x, waypoint_y] = fk_2link(waypoints_rad(:, 1), waypoints_rad(:, 2), params);

t_all = [];
x_all = [];
y_all = [];

current_time = 0;

for k = 1:size(waypoints_rad, 1)-1
    x_start = waypoint_x(k);
    y_start = waypoint_y(k);
    x_end = waypoint_x(k+1);
    y_end = waypoint_y(k+1);

    t_seg = (0:params.dt:params.Tseg)';
    s = t_seg / params.Tseg;
    h = 10*s.^3 - 15*s.^4 + 6*s.^5;

    x_seg = x_start + (x_end - x_start) .* h;
    y_seg = y_start + (y_end - y_start) .* h;

    if k > 1
        t_seg = t_seg(2:end);
        x_seg = x_seg(2:end);
        y_seg = y_seg(2:end);
    end

    t_all = [t_all; current_time + t_seg];
    x_all = [x_all; x_seg];
    y_all = [y_all; y_seg];

    current_time = current_time + params.Tseg;
end

q_all = ik_2link(x_all, y_all, params, 'positive');
q_all = unwrap(q_all);

dq_all = zeros(size(q_all));
ddq_all = zeros(size(q_all));
for j = 1:2
    dq_all(:, j) = gradient(q_all(:, j), t_all);
    ddq_all(:, j) = gradient(dq_all(:, j), t_all);
end

traj.name = '笛卡尔空间规划';
traj.method_id = 'cartesian_quintic';
traj.space = 'cartesian';
traj.t = t_all;
traj.q = q_all;
traj.dq = dq_all;
traj.ddq = ddq_all;
traj.x_d = x_all;
traj.y_d = y_all;
end
