function metrics = compute_metrics(traj, result, params)
%COMPUTE_METRICS Compute trajectory tracking and smoothness indicators.

joint_err = result.error;
joint_err_deg = joint_err * 180/pi;
ee_err = sqrt((result.x2_d - result.x2).^2 + (result.y2_d - result.y2).^2);

metrics.method = string(traj.name);
metrics.joint1_max_error_deg = max(abs(joint_err_deg(:, 1)));
metrics.joint2_max_error_deg = max(abs(joint_err_deg(:, 2)));
metrics.joint1_rmse_deg = sqrt(mean(joint_err_deg(:, 1).^2));
metrics.joint2_rmse_deg = sqrt(mean(joint_err_deg(:, 2).^2));
metrics.ee_max_error_m = max(ee_err);
metrics.ee_rmse_m = sqrt(mean(ee_err.^2));
metrics.max_torque_Nm = max(abs(result.u(:)));
metrics.mean_torque_Nm = mean(abs(result.u(:)));
metrics.max_velocity_deg_s = max(abs(traj.dq(:))) * 180/pi;
metrics.max_acceleration_deg_s2 = max(abs(traj.ddq(:))) * 180/pi;
end
