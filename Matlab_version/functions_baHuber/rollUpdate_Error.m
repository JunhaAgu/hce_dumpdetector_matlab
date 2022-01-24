function error = rollUpdate_Error(x, R_gps_init, R_ba_init, u)

u_hat = [0 -u(3) u(2); u(3) 0 -u(1); -u(2) u(1) 0];

R_roll = cos(x)*eye(3) + sin(x)*u_hat + (1-cos(x))*(u*u');


error = norm(R_gps_init - R_roll*R_ba_init);


end

