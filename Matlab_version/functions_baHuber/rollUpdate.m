function R_roll = rollUpdate(T_gps_init, T_ba_init, u)
% u is roll axis

% Initial guess is zero
x = 0;

% Error fuction. errors = p_G_camera - p_V_camera_aligned(x);
error = @(x) rollUpdate_Error(x, T_gps_init(1:3, 1:3), T_ba_init(1:3, 1:3), u);

% options = optimoptions(@lsqnonlin, 'Algorithm', 'levenberg-marquardt', 'Display', 'iter');
options = optimoptions(@lsqnonlin, 'Display', 'iter');

x_optim = lsqnonlin(error, x, [], [], options); % optimization by lsqnonlin func.

u_hat = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];
R_roll = cos(x_optim)*eye(3) + sin(x_optim)*u_hat + (1-cos(x_optim))*(u*u');




end