function error = Align_Error(x, t_gps, t_ba)


T_gps_ba = se3Exp(x(1:6));
scale_gps_ba = x(7);
num_frames = size(t_ba, 2);

t_ba_aligned = scale_gps_ba * T_gps_ba(1:3, 1:3) * t_ba + repmat(T_gps_ba(1:3, 4), [1 num_frames]);

error = t_gps - t_ba_aligned;

error = error(:);

end

