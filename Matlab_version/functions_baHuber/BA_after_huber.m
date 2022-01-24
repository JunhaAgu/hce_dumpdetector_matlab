function [points_ba, T_c0ck_ba] = BA_after_huber(hidden_state_opt, observations, r, n_img_gap, T_wk0)

r_idx = zeros((length(observations) - 2 - n_img_gap) / 3, 1);
error_i = 1;
observation_i = 3;
for i = 1:n_img_gap
    ki = observations(observation_i); % ki: ith img ¾È¿¡¼­ ÂïÈù keypoints ¼ö
 
    r_idx(error_i:error_i+ki-1) = observations(observation_i+2*ki+1:observation_i+3*ki);
    
    error_i = error_i + ki;
    observation_i = observation_i + 1 + 3*ki;
end

r = sqrt(sum(reshape(r.^2, 2, [])));
idx_outlier = r >= 4*mean(r);
idx_outlier = unique(r_idx(idx_outlier));


points_ba = T_wk0(1:3, 1:3)*reshape(hidden_state_opt(n_img_gap*6+1:end), 3, []) + T_wk0(1:3,4);
% points_ba(:, idx_outlier) = NaN;
% points_ba = rmmissing(points_ba, 2);

twist_kk0 = reshape(hidden_state_opt(1:n_img_gap*6), 6, []); % twist coord extracted from hidden_state [6xn]

T_c0ck_ba = cell(1, n_img_gap);
for i = 1:n_img_gap
    T_c0ck_ba{i} = T_wk0*se3Inv(se3Exp(twist_kk0(:, i)));
end



end