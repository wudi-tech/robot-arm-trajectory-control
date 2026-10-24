function metrics = compute_metrics_3dof(traj, result)
%COMPUTE_METRICS_3DOF Compute tracking, torque, and smoothness metrics.

joint_err_deg = result.error * 180/pi;
ee_err = sqrt((result.x_d - result.x).^2 + (result.y_d - result.y).^2 + (result.z_d - result.z).^2);

metrics.method = string(traj.name);
metrics.joint1_rmse_deg = sqrt(mean(joint_err_deg(:, 1).^2));
metrics.joint2_rmse_deg = sqrt(mean(joint_err_deg(:, 2).^2));
metrics.joint3_rmse_deg = sqrt(mean(joint_err_deg(:, 3).^2));
metrics.joint1_max_error_deg = max(abs(joint_err_deg(:, 1)));
metrics.joint2_max_error_deg = max(abs(joint_err_deg(:, 2)));
metrics.joint3_max_error_deg = max(abs(joint_err_deg(:, 3)));
metrics.ee_rmse_m = sqrt(mean(ee_err.^2));
metrics.ee_max_error_m = max(ee_err);
metrics.max_torque_Nm = max(abs(result.u(:)));
metrics.mean_torque_Nm = mean(abs(result.u(:)));
metrics.max_velocity_deg_s = max(abs(traj.dq(:))) * 180/pi;
metrics.max_acceleration_deg_s2 = max(abs(traj.ddq(:))) * 180/pi;
end
