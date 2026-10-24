function waypoints = init_waypoints()
%INIT_WAYPOINTS Return A-B-C-A task waypoints in degrees and radians.

waypoints.A_deg = [30, 20];
waypoints.B_deg = [55, 75];
waypoints.C_deg = [85, 35];

waypoints.deg = [
    waypoints.A_deg;
    waypoints.B_deg;
    waypoints.C_deg;
    waypoints.A_deg
];

waypoints.rad = waypoints.deg * pi / 180;
waypoints.labels = {'A 等待点', 'B 取料点', 'C 放料点'};
end
