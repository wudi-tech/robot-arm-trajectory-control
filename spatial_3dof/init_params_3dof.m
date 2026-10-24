function params = init_params_3dof()
%INIT_PARAMS_3DOF Return parameters for the spatial 3-DOF robot arm.

params.base_height = 0.35;
params.link_lengths = [0.85, 0.65];

params.J = [0.030, 0.025, 0.015];
params.B = [0.120, 0.100, 0.080];

params.Kp = [35, 32, 24];
params.Ki = [3.5, 3.0, 2.0];
params.Kd = [0.9, 0.8, 0.55];

params.Tseg = 3;
params.dt = 0.001;
params.trap_accel_ratio = 0.25;
params.pybullet_stride = 20;
params.video_step = 60;
params.video_frame_rate = 15;
end
