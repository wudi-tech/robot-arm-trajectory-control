function traj = plan_cubic_joint_3dof(waypoints_q, params)
%PLAN_CUBIC_JOINT_3DOF Generate joint-space cubic trajectories.

[t, q, dq, ddq] = build_segmented_joint_trajectory(waypoints_q, params, 'cubic');

traj.name = '三次多项式';
traj.method_id = 'cubic_joint_3dof';
traj.space = 'joint';
traj.t = t;
traj.q = q;
traj.dq = dq;
traj.ddq = ddq;
end
