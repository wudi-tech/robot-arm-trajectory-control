function [t_all, q_all, dq_all, ddq_all] = build_segmented_joint_trajectory(waypoints_q, params, profile)
%BUILD_SEGMENTED_JOINT_TRAJECTORY Shared segmented joint trajectory builder.

t_all = [];
q_all = [];
dq_all = [];
ddq_all = [];
current_time = 0;

for k = 1:size(waypoints_q, 1)-1
    q_start = waypoints_q(k, :);
    q_end = waypoints_q(k+1, :);
    delta_q = q_end - q_start;

    t_seg = (0:params.dt:params.Tseg)';
    [h, dh, ddh] = time_profile(t_seg, params, profile);

    q_seg = q_start + h .* delta_q;
    dq_seg = dh .* delta_q;
    ddq_seg = ddh .* delta_q;

    if k > 1
        t_seg = t_seg(2:end);
        q_seg = q_seg(2:end, :);
        dq_seg = dq_seg(2:end, :);
        ddq_seg = ddq_seg(2:end, :);
    end

    t_all = [t_all; current_time + t_seg];
    q_all = [q_all; q_seg];
    dq_all = [dq_all; dq_seg];
    ddq_all = [ddq_all; ddq_seg];
    current_time = current_time + params.Tseg;
end
end

function [h, dh, ddh] = time_profile(t, params, profile)
s = t / params.Tseg;

switch lower(profile)
    case 'cubic'
        h = 3*s.^2 - 2*s.^3;
        dh = 6*t/params.Tseg^2 - 6*t.^2/params.Tseg^3;
        ddh = 6/params.Tseg^2 - 12*t/params.Tseg^3;
    case 'quintic'
        h = 10*s.^3 - 15*s.^4 + 6*s.^5;
        dh = 30*t.^2/params.Tseg^3 - 60*t.^3/params.Tseg^4 + 30*t.^4/params.Tseg^5;
        ddh = 60*t/params.Tseg^3 - 180*t.^2/params.Tseg^4 + 120*t.^3/params.Tseg^5;
    case 'trapezoid'
        ta = params.trap_accel_ratio * params.Tseg;
        if ta <= 0 || ta >= params.Tseg / 2
            error('trap_accel_ratio must produce 0 < ta < Tseg/2.');
        end
        v = 1 / (params.Tseg - ta);
        a = v / ta;
        h = zeros(size(t));
        dh = zeros(size(t));
        ddh = zeros(size(t));
        idx_acc = t < ta;
        idx_flat = t >= ta & t <= params.Tseg - ta;
        idx_dec = t > params.Tseg - ta;
        h(idx_acc) = 0.5 * a * t(idx_acc).^2;
        dh(idx_acc) = a * t(idx_acc);
        ddh(idx_acc) = a;
        h(idx_flat) = 0.5 * a * ta^2 + v * (t(idx_flat) - ta);
        dh(idx_flat) = v;
        ddh(idx_flat) = 0;
        t_remaining = params.Tseg - t(idx_dec);
        h(idx_dec) = 1 - 0.5 * a * t_remaining.^2;
        dh(idx_dec) = a * t_remaining;
        ddh(idx_dec) = -a;
        h(end) = 1;
        dh(end) = 0;
        ddh(end) = -a;
    otherwise
        error(['Unknown time profile: ', profile]);
end
end
