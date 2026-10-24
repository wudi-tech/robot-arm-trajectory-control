function traj = plan_cartesian_quintic_3dof(waypoints_xyz, params)
%PLAN_CARTESIAN_QUINTIC_3DOF Plan 3D straight paths with quintic timing.

t_all = [];
x_all = [];
y_all = [];
z_all = [];
current_time = 0;

for k = 1:size(waypoints_xyz, 1)-1
    p_start = waypoints_xyz(k, :);
    p_end = waypoints_xyz(k+1, :);
    delta_p = p_end - p_start;

    t_seg = (0:params.dt:params.Tseg)';
    s = t_seg / params.Tseg;
    h = 10*s.^3 - 15*s.^4 + 6*s.^5;

    xyz_seg = p_start + h .* delta_p;

    if k > 1
        t_seg = t_seg(2:end);
        xyz_seg = xyz_seg(2:end, :);
    end

    t_all = [t_all; current_time + t_seg];
    x_all = [x_all; xyz_seg(:, 1)];
    y_all = [y_all; xyz_seg(:, 2)];
    z_all = [z_all; xyz_seg(:, 3)];
    current_time = current_time + params.Tseg;
end

q_all = ik_3dof(x_all, y_all, z_all, params, 'positive');
q_all = unwrap(q_all);

dq_all = zeros(size(q_all));
ddq_all = zeros(size(q_all));
for j = 1:3
    dq_all(:, j) = gradient(q_all(:, j), t_all);
    ddq_all(:, j) = gradient(dq_all(:, j), t_all);
end

traj.name = '三维笛卡尔空间规划';
traj.method_id = 'cartesian_quintic_3dof';
traj.space = 'cartesian';
traj.t = t_all;
traj.q = q_all;
traj.dq = dq_all;
traj.ddq = ddq_all;
traj.x_d = x_all;
traj.y_d = y_all;
traj.z_d = z_all;
end
