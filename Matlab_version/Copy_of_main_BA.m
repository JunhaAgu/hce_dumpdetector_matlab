%% set the data_base for Bundle Adjustment

addpath('functions_Lie'); % Lie group (SE(3)), Lie algebra (se(3)), and Rotational matrix (SO(3)) 관련 함수들
addpath('functions_baHuber');               % BA 관련 함수들.

tag0_n_group = group_num(1);
tag1_n_group = group_num(2);
tag2_n_group = group_num(3);
tag3_n_group = group_num(4);
if isempty( find(Ap{1, 1}.frame_group==tag0_n_group) )
    tag0_n_group = tag0_n_group-1;
end
if isempty( find(Ap{1, 2}.frame_group==tag1_n_group) )
    tag1_n_group = tag1_n_group-1;
end
if isempty( find(Ap{1, 3}.frame_group==tag2_n_group) )
    tag2_n_group = tag2_n_group-1;
end
if isempty( find(Ap{1, 4}.frame_group==tag3_n_group) )
    tag3_n_group = tag3_n_group-1;
end
n_db = tag0_n_group + tag1_n_group + tag2_n_group + tag3_n_group ;
data_base.db = cell(4*n_db,1);

cnt_feat = 0;
if ~isempty(tag0_n_group)
    for i=1:tag0_n_group
        idx_0_tmp = find(Ap{1, 1}.frame_group==i);
        idx_0{1,i} = Ap{1, 1}.frame(idx_0_tmp);
        tmp_idx = length(idx_0{1,i});
        data_base.db{4*(i-1)+1,1}.pts(1:2,1:tmp_idx) = Ap{1,1}.P0(:,idx_0_tmp);
        data_base.db{4*(i-1)+2,1}.pts(1:2,1:tmp_idx) = Ap{1,1}.P1(:,idx_0_tmp);
        data_base.db{4*(i-1)+3,1}.pts(1:2,1:tmp_idx) = Ap{1,1}.P2(:,idx_0_tmp);
        data_base.db{4*(i-1)+4,1}.pts(1:2,1:tmp_idx) = Ap{1,1}.P3(:,idx_0_tmp);

        data_base.db{4*(i-1)+1,1}.pts_3D = Ap{1,1}.P0_3D(1,idx_0_tmp);
        data_base.db{4*(i-1)+2,1}.pts_3D = Ap{1,1}.P1_3D(1,idx_0_tmp);
        data_base.db{4*(i-1)+3,1}.pts_3D = Ap{1,1}.P2_3D(1,idx_0_tmp);
        data_base.db{4*(i-1)+4,1}.pts_3D = Ap{1,1}.P3_3D(1,idx_0_tmp);

        data_base.db{4*(i-1)+1,1}.id_imgs = idx_0{1,i};
        data_base.db{4*(i-1)+2,1}.id_imgs = idx_0{1,i};
        data_base.db{4*(i-1)+3,1}.id_imgs = idx_0{1,i};
        data_base.db{4*(i-1)+4,1}.id_imgs = idx_0{1,i};
    end
    cnt_feat = cnt_feat+4*tag0_n_group;
end

if ~isempty(tag1_n_group)
    for i=1:tag1_n_group
        idx_1_tmp = find(Ap{1, 2}.frame_group==i);
        idx_1{1,i} = Ap{1, 2}.frame(idx_1_tmp);
        tmp_idx = length(idx_1{1,i});
        data_base.db{cnt_feat+4*(i-1)+1,1}.pts(1:2,1:tmp_idx) = Ap{1,2}.P0(:,idx_1_tmp);
        data_base.db{cnt_feat+4*(i-1)+2,1}.pts(1:2,1:tmp_idx) = Ap{1,2}.P1(:,idx_1_tmp);
        data_base.db{cnt_feat+4*(i-1)+3,1}.pts(1:2,1:tmp_idx) = Ap{1,2}.P2(:,idx_1_tmp);
        data_base.db{cnt_feat+4*(i-1)+4,1}.pts(1:2,1:tmp_idx) = Ap{1,2}.P3(:,idx_1_tmp);

        data_base.db{cnt_feat+4*(i-1)+1,1}.pts_3D = Ap{1,2}.P0_3D(1,idx_1_tmp);
        data_base.db{cnt_feat+4*(i-1)+2,1}.pts_3D = Ap{1,2}.P1_3D(1,idx_1_tmp);
        data_base.db{cnt_feat+4*(i-1)+3,1}.pts_3D = Ap{1,2}.P2_3D(1,idx_1_tmp);
        data_base.db{cnt_feat+4*(i-1)+4,1}.pts_3D = Ap{1,2}.P3_3D(1,idx_1_tmp);

        data_base.db{cnt_feat+4*(i-1)+1,1}.id_imgs = idx_1{1,i};
        data_base.db{cnt_feat+4*(i-1)+2,1}.id_imgs = idx_1{1,i};
        data_base.db{cnt_feat+4*(i-1)+3,1}.id_imgs = idx_1{1,i};
        data_base.db{cnt_feat+4*(i-1)+4,1}.id_imgs = idx_1{1,i};
    end
    cnt_feat = cnt_feat+4*tag1_n_group;
end

if ~isempty(tag2_n_group)
    for i=1:tag2_n_group
        idx_2_tmp = find(Ap{1, 3}.frame_group==i);
        idx_2{1,i} = Ap{1, 3}.frame(idx_2_tmp);
        tmp_idx = length(idx_2{1,i});
        data_base.db{cnt_feat+4*(i-1)+1,1}.pts(1:2,1:tmp_idx) = Ap{1,3}.P0(:,idx_2_tmp);
        data_base.db{cnt_feat+4*(i-1)+2,1}.pts(1:2,1:tmp_idx) = Ap{1,3}.P1(:,idx_2_tmp);
        data_base.db{cnt_feat+4*(i-1)+3,1}.pts(1:2,1:tmp_idx) = Ap{1,3}.P2(:,idx_2_tmp);
        data_base.db{cnt_feat+4*(i-1)+4,1}.pts(1:2,1:tmp_idx) = Ap{1,3}.P3(:,idx_2_tmp);

        data_base.db{cnt_feat+4*(i-1)+1,1}.pts_3D = Ap{1,3}.P0_3D(1,idx_2_tmp);
        data_base.db{cnt_feat+4*(i-1)+2,1}.pts_3D = Ap{1,3}.P1_3D(1,idx_2_tmp);
        data_base.db{cnt_feat+4*(i-1)+3,1}.pts_3D = Ap{1,3}.P2_3D(1,idx_2_tmp);
        data_base.db{cnt_feat+4*(i-1)+4,1}.pts_3D = Ap{1,3}.P3_3D(1,idx_2_tmp);

        data_base.db{cnt_feat+4*(i-1)+1,1}.id_imgs = idx_2{1,i};
        data_base.db{cnt_feat+4*(i-1)+2,1}.id_imgs = idx_2{1,i};
        data_base.db{cnt_feat+4*(i-1)+3,1}.id_imgs = idx_2{1,i};
        data_base.db{cnt_feat+4*(i-1)+4,1}.id_imgs = idx_2{1,i};
    end
    cnt_feat = cnt_feat+4*tag2_n_group;
end


if ~isempty(tag3_n_group)
    for i=1:tag3_n_group
        idx_3_tmp = find(Ap{1, 4}.frame_group==i);
        idx_3{1,i} = Ap{1, 4}.frame(idx_3_tmp);
        tmp_idx = length(idx_3{1,i});
        data_base.db{cnt_feat+4*(i-1)+1,1}.pts(1:2,1:tmp_idx) = Ap{1,4}.P0(:,idx_3_tmp);
        data_base.db{cnt_feat+4*(i-1)+2,1}.pts(1:2,1:tmp_idx) = Ap{1,4}.P1(:,idx_3_tmp);
        data_base.db{cnt_feat+4*(i-1)+3,1}.pts(1:2,1:tmp_idx) = Ap{1,4}.P2(:,idx_3_tmp);
        data_base.db{cnt_feat+4*(i-1)+4,1}.pts(1:2,1:tmp_idx) = Ap{1,4}.P3(:,idx_3_tmp);

        data_base.db{cnt_feat+4*(i-1)+1,1}.pts_3D = Ap{1,4}.P0_3D(1,idx_3_tmp);
        data_base.db{cnt_feat+4*(i-1)+2,1}.pts_3D = Ap{1,4}.P1_3D(1,idx_3_tmp);
        data_base.db{cnt_feat+4*(i-1)+3,1}.pts_3D = Ap{1,4}.P2_3D(1,idx_3_tmp);
        data_base.db{cnt_feat+4*(i-1)+4,1}.pts_3D = Ap{1,4}.P3_3D(1,idx_3_tmp);

        data_base.db{cnt_feat+4*(i-1)+1,1}.id_imgs = idx_3{1,i};
        data_base.db{cnt_feat+4*(i-1)+2,1}.id_imgs = idx_3{1,i};
        data_base.db{cnt_feat+4*(i-1)+3,1}.id_imgs = idx_3{1,i};
        data_base.db{cnt_feat+4*(i-1)+4,1}.id_imgs = idx_3{1,i};
    end
    cnt_feat = cnt_feat+4*tag3_n_group;
end

%% 
idx_tag_valid=[];
idx_tag_dom=[];
for ii = valid_data_num
    vv_select=1e7;
    tag_fag = 0;
    non_tag_fag = 0;
    for j=1:4
        tt_tmp = find(Ap{1, j}.April_frame==ii);
        if ~isempty(tt_tmp)
            vv_tmp = norm(Ap{1, j}.T_cp{1, tt_tmp}(1:3,4));
            if  vv_select > vv_tmp
                vv_select = vv_tmp;
                tt_select = ii;
                tag_select = j;
            end
            tag_fag = 1;
        end
    end
    if tag_fag==0
        for j=1:4
            tt_tmp = find(Ap{1, j}.frame==ii);
            if ~isempty(tt_tmp)
                vv_tmp = norm(Ap{1, j}.T_cp{1, tt_tmp}(1:3,4));
                if  vv_select > vv_tmp
                    vv_select = vv_tmp;
                    tt_select = ii;
                    tag_select = j;
                end
                non_tag_fag = 1;
            end
        end
    end
    if or(tag_fag,non_tag_fag)
        idx_tag_valid = [idx_tag_valid, tt_select];
        idx_tag_dom = [idx_tag_dom, tag_select];
    end
end

disp(['**Base Tag id:',num2str(idx_tag_valid(1)-1)]);
% frame_order = find(Ap{1, idx_tag_dom(1)}.frame==idx_tag_valid(1));
% T_00 = Ap{1, idx_tag_dom(1)}.T_cp{1, frame_order};

T_bp = cell(1,length(idx_tag_valid));
for jj=1:length(idx_tag_valid)
%     idx_frame = idx_tag_valid(jj);
    frame_order = find(Ap{1, idx_tag_dom(jj)}.frame==idx_tag_valid(jj));
    T_tmp = Ap{1, idx_tag_dom(jj)}.T_cp{1, frame_order};
    if idx_tag_dom(1) == 1
        if idx_tag_dom(jj) == 1
            T_bp{1,jj} = T_tmp;
        elseif idx_tag_dom(jj) == 2
            T_bp{1,jj} = T_tmp*inv(T_01_f);
        elseif idx_tag_dom(jj) == 3
            T_bp{1,jj} = T_tmp*inv(T_02_f);
        elseif idx_tag_dom(jj) == 4
            T_bp{1,jj} = T_tmp*inv(T_03_f);
        end
    elseif idx_tag_dom(1) == 2
        if idx_tag_dom(jj) == 1
            T_bp{1,jj} = T_tmp*(T_01_f);
        elseif idx_tag_dom(jj) == 2
            T_bp{1,jj} = T_tmp;
        elseif idx_tag_dom(jj) == 3
            T_bp{1,jj} = T_tmp*inv(T_02_f)*(T_01_f);
        elseif idx_tag_dom(jj) == 4
            T_bp{1,jj} = T_tmp*inv(T_03_f)*(T_01_f);
        end
    elseif idx_tag_dom(1) == 3
        if idx_tag_dom(jj) == 1
            T_bp{1,jj} = T_tmp*(T_02_f);
        elseif idx_tag_dom(jj) == 2
            T_bp{1,jj} = T_tmp*inv(T_01_f)*(T_02_f);
        elseif idx_tag_dom(jj) == 3            
            T_bp{1,jj} = T_tmp;
        elseif idx_tag_dom(jj) == 4
            T_bp{1,jj} = T_tmp*inv(T_03_f)*(T_02_f);
        end
    elseif idx_tag_dom(1) == 4
        if idx_tag_dom(jj) == 1
            T_bp{1,jj} = T_tmp*(T_03_f);
        elseif idx_tag_dom(jj) == 2
            T_bp{1,jj} = T_tmp*inv(T_01_f)*(T_03_f);
        elseif idx_tag_dom(jj) == 3            
            T_bp{1,jj} = T_tmp*inv(T_02_f)*(T_03_f);
        elseif idx_tag_dom(jj) == 4
            T_bp{1,jj} = T_tmp;            
        end
    end
end

T_0k = cell(1,length(T_bp));
for jj=1:length(T_bp)
    T_0k{1,jj} = T_bp{1,1} * inv(T_bp{1,jj});
end
figure(22);
for i=1:length(T_bp)
    plot3(T_0k{1,i}(1,4),T_0k{1,i}(2,4),T_0k{1,i}(3,4),'r*'); hold on;
end
plot3(0,0,0,'rs');
axis equal;
xlabel('x[m]'); ylabel('y[m]'); zlabel('z [m]');
title('excavator trajectory');

figure(123);
for i=1:length(T_bp)
    plot3(T_bp{1,i}(1,4),T_bp{1,i}(2,4),T_bp{1,i}(3,4),'r*'); hold on;
end
plot3(0,0,0,'bs');
axis equal;
xlabel('x[m]'); ylabel('y[m]'); zlabel('z [m]');
title(['Base Tag: ',num2str(idx_tag_valid(1)-1)]);

%% 3D points
idx_3d = 1:length(data_base.db);
points_trimmed = mean3DPoints(T_0k, data_base, idx_tag_valid, idx_tag_dom);
n_img_gap = length(T_bp);
K = intrinsics.Intrinsics.IntrinsicMatrix';

[hidden_state, observation, T_wk0_huber] = BA_initialze_huber(data_base, T_0k, points_trimmed, idx_3d, n_img_gap, idx_tag_valid);
[hidden_state_optimal, residual] = runBA_huber(hidden_state, observation, K);
[points_ba, T_c0ck_ba] = BA_after_huber(hidden_state_optimal, observation, residual, n_img_gap, T_wk0_huber);
% [points_ba, T_c0ck_ba] = alignGPS2BA(points_ba, T_c0ck_ba, T_0k);

figure(22);
for i=1:length(T_c0ck_ba)
    plot3(T_c0ck_ba{1,i}(1,4),T_c0ck_ba{1,i}(2,4),T_c0ck_ba{1,i}(3,4),'b*'); hold on;
end
plot3(0,0,0,'bs');
axis equal;
xlabel('x[m]'); ylabel('y[m]'); zlabel('z [m]');


figure(11);
hold on;
for i=1:length(points_ba)
    plot3(points_ba(1,i),points_ba(2,i),points_ba(3,i),'b+','MarkerSize',20); hold on;
end
axis equal;
xlabel('x[m]'); ylabel('y[m]'); zlabel('z [m]');

fixed=[];
for i=1:length(T_0k)
    fixed(i,:) = T_0k{i}(1:3,4)';
end
moving=[];
for i=1:length(T_c0ck_ba)
    moving(i,:) = T_c0ck_ba{i}(1:3,4)';
end

fixed = pointCloud(fixed);
moving = pointCloud(moving);
% tform = pcregrigid(moving,fixed);

% moving_warped = tform.T'*[moving.Location'; ones(1,size(moving.Location,1))];
figure();
pcshow(fixed.Location,repmat([1 0 0],size(fixed.Location,1),1));
hold on;
pcshow(moving.Location,repmat([0 1 0],size(moving.Location,1),1));
% pcshow(moving_warped(1:3,:)', repmat([0 1 0],size(moving_warped,2),1));

% points_ba_icp = tform.T'*[points_ba; ones(1,size(points_ba,2))];
% 
% figure(11);
% hold on;
% for i=1:length(points_ba_icp)
%     plot3(points_ba_icp(1,i),points_ba_icp(2,i),points_ba_icp(3,i),'m*','MarkerSize',20); hold on;
% end
% axis equal;
% xlabel('x[m]'); ylabel('y[m]'); zlabel('z [m]');