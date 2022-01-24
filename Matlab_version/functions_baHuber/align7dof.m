function [T_gps_ba, scale_gps_ba] = align7dof(t_gps, t_ba)


% Initial guess is identity
twist_guess = se3Log(eye(4));
scale_guess = 1;

x = [twist_guess; scale_guess];

% Error fuction. errors = p_G_camera - p_V_camera_aligned(x);
error = @(x) Align_Error(x, t_gps, t_ba);

options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'Display', 'iter');
% options = optimoptions(@lsqnonlin, 'Display', 'iter');

x_optim = lsqnonlin(error, x, [], [], options); % optimization by lsqnonlin func.

T_gps_ba = se3Exp(x_optim(1:6));
scale_gps_ba = x_optim(7);


% num_frames = size(t_ba, 2);
% t_ba_aligned = scale_gps_ba * T_gps_ba(1:3, 1:3) * t_ba + repmat(T_gps_ba(1:3, 4), [1, num_frames]);
% p_G_camera_aligned = (1/scale_gps_ba) * T_gps_ba(1:3, 1:3)' * t_gps + repmat(-(1/scale_gps_ba) * T_gps_ba(1:3, 1:3)' * T_gps_ba(1:3, 4), [1, num_frames]);
end