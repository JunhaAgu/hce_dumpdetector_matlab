function [points_ba_aligned, T_c0ck_ba_aligned] = alignGPS2BA(points_ba, T_c0ck_ba, T_c0ck)


% represented by T_c0ck{1}
num_frames = length(T_c0ck);
T_c0ck_init = T_c0ck{1};
T_cick_ = T_c0ck;
T_cick_ba_ = T_c0ck_ba;
for i = 1:num_frames
    T_cick_{i} = se3Inv(T_c0ck_init)*T_c0ck{i};
    T_cick_ba_{i} = se3Inv(T_c0ck_init)*T_c0ck_ba{i};    
end
points_ba_ = T_c0ck_init(1:3, 1:3)'*points_ba - T_c0ck_init(1:3, 1:3)'*T_c0ck_init(1:3, end);



% 7 dof alignment (Sim(3)) & initial pose fixed to gps
t_gps_InitFin = [T_cick_{1}(1:3, end), T_cick_{end}(1:3, end)];
t_ba_InitFin = [T_cick_ba_{1}(1:3, end), T_cick_ba_{end}(1:3, end)];

[T_gps_ba, scale_gps_ba] = align7dof(t_gps_InitFin, t_ba_InitFin);






% straight region --> roll data is not reliable
roll_axis = (t_gps_InitFin(:, end) - t_gps_InitFin(:, 1))/norm(t_gps_InitFin(:, end) - t_gps_InitFin(:, 1));
R_roll = rollUpdate(T_cick_{1}, T_gps_ba*T_cick_ba_{1}, roll_axis);



% update
points_ba_aligned = R_roll*(scale_gps_ba*T_gps_ba(1:3, 1:3)'*points_ba_ + repmat(T_gps_ba(1:3, 4), [1, size(points_ba_, 2)]));
T_c0ck_ba_aligned = cell(size(T_cick_ba_));
for i = 1:num_frames
    t_temp = R_roll*(scale_gps_ba*T_gps_ba(1:3, 1:3)*T_cick_ba_{i}(1:3, 4) + T_gps_ba(1:3, 4));
    T_c0ck_ba_aligned{i} = [R_roll*T_gps_ba(1:3, 1:3)*T_cick_ba_{i}(1:3, 1:3), t_temp; 0 0 0 1];
end




% represent by {c0}
for i = 1:num_frames
    T_c0ck_ba_aligned{i} = T_c0ck_init*T_c0ck_ba_aligned{i};    
end
points_ba_aligned = T_c0ck_init(1:3, 1:3)*points_ba_aligned + T_c0ck_init(1:3, end);



% for debug
% figure()
% plotCoordinateFrame(T_c0ck_ba_aligned{1}(1:3, 1:3), T_c0ck_ba_aligned{1}(1:3, end), 1);
% hold on
% plotCoordinateFrame(T_c0ck{1}(1:3, 1:3), T_c0ck{1}(1:3, end), 1);
% 
% hold off
% axis equal
% xlabel('X')
% ylabel('Y')
% zlabel('Z')
% grid on
end