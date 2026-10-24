params = init_params();
waypoints = init_waypoints();

traj = plan_quintic_joint(waypoints.rad, params);
result = simulate_pid_tracking(traj, params);

T = table(result.t, ...
    traj.q(:,1), traj.q(:,2), ...
    result.q(:,1), result.q(:,2), ...
    'VariableNames', {'t','theta1_d','theta2_d','theta1','theta2'});

writetable(T, 'quintic_pybullet.csv');