function [hidden_state, observation, T_wk0] = BA_initialze_huber(data_base, T_0k, points_trimmed, idx_3d, n_img_gap, idx_tag_valid)

points3D_init = points_trimmed;
hidden_state = zeros(6*n_img_gap + 3*size(points3D_init, 2), 1);


T_wk0 = T_0k{1};
points3D_init = T_wk0(1:3,1:3)'*points3D_init - T_wk0(1:3,1:3)'*T_wk0(1:3,4);
%0(world coordi)기준이었던 것을 1기준으로 바꿔줌
for i = 1:n_img_gap
    hidden_state(6*i-5:6*i) = se3Log(se3Inv(T_0k{i})*T_wk0);
%     hidden_state(6*i-5:6*i) = se3Log(se3Inv(T_c0ck{i}));
end
hidden_state(1 + 6*n_img_gap:end) = points3D_init(:);


% make observation
db_ba = data_base.db(idx_3d);
observationEachFrame = cell(n_img_gap, 1);

z = 0;
for i = 1:n_img_gap
    
    j = 1;
    observationEachFrame_pts = [];
    observationEachFrame_idx = [];
    for ii = 1:size(points3D_init, 2)
        check = find(db_ba{ii}.id_imgs == idx_tag_valid(i));
        if ~isempty(check)
            observationEachFrame_pts(2*j-1:2*j, 1) = db_ba{ii}.pts(:, check);
            observationEachFrame_idx(j, 1) = ii;
            j = j+1;           
        end
    end

    observationEachFrame{i} = [j-1; observationEachFrame_pts; observationEachFrame_idx];
    z = z + size(observationEachFrame{i}, 1);
end


observation = zeros(2 + z, 1);
observation(1:2) = [n_img_gap; size(points3D_init, 2)];

j = 2;
for i = 1:n_img_gap
    range_obs = j+1:j+1+size(observationEachFrame{i}, 1)-1;
    observation(range_obs) = observationEachFrame{i};
    
    j = range_obs(end);
    
end
end
