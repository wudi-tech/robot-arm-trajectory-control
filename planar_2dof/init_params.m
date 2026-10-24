function params = init_params()
%INIT_PARAMS Return shared simulation parameters for the 2-DOF arm.

params.L1 = 1.0;
params.L2 = 0.8;

params.J = [0.02, 0.01];
params.B = [0.10, 0.08];

params.Kp = [25, 22];
params.Ki = [4, 3];
params.Kd = [0.7, 0.6];

params.Tseg = 3;
params.dt = 0.001;
params.trap_accel_ratio = 0.25;

params.video_step = 60;
params.video_frame_rate = 15;
end
