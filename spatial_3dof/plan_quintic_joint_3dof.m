function traj = plan_quintic_joint_3dof(waypoints_q, params)
%PLAN_QUINTIC_JOINT_3DOF Generate joint-space quintic trajectories.

[t, q, dq, ddq] = build_segmented_joint_trajectory(waypoints_q, params, 'quintic');

traj.name = '五次多项式';
traj.method_id = 'quintic_joint_3dof';
traj.space = 'joint';
traj.t = t;
traj.q = q;
traj.dq = dq;
traj.ddq = ddq;
end
