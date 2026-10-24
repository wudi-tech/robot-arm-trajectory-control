function waypoints = init_waypoints_3dof(params)
%INIT_WAYPOINTS_3DOF Return spatial A-B-C-A task points and joint waypoints.

waypoints.A_xyz = [0.95, 0.35, 0.78];
waypoints.B_xyz = [0.55, 1.05, 1.05];
waypoints.C_xyz = [-0.65, 0.85, 0.88];

waypoints.xyz = [
    waypoints.A_xyz;
    waypoints.B_xyz;
    waypoints.C_xyz;
    waypoints.A_xyz
];

waypoints.q = ik_3dof(waypoints.xyz(:, 1), waypoints.xyz(:, 2), waypoints.xyz(:, 3), params);
waypoints.q = unwrap(waypoints.q);
waypoints.labels = {'A 等待点', 'B 取料点', 'C 放料点'};
end
