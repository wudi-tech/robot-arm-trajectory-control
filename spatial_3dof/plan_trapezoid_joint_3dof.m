function traj = plan_trapezoid_joint_3dof(waypoints_q, params)
%PLAN_TRAPEZOID_JOINT_3DOF Generate joint-space trapezoidal velocity trajectories.

[t, q, dq, ddq] = build_segmented_joint_trajectory(waypoints_q, params, 'trapezoid');

traj.name = '梯形速度规划';
traj.method_id = 'trapezoid_joint_3dof';
traj.space = 'joint';
traj.t = t;
traj.q = q;
traj.dq = dq;
traj.ddq = ddq;
end
