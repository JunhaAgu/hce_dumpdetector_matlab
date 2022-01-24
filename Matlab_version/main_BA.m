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
data_base.db = cell(4*4,1);

if ~isempty(tag0_n_group)
        data_base.db{1,1}.pts = Ap{1,1}.P0;
        data_base.db{2,1}.pts = Ap{1,1}.P1;
        data_base.db{3,1}.pts = Ap{1,1}.P2;
        data_base.db{4,1}.pts = Ap{1,1}.P3;

        data_base.db{1,1}.pts_3D = Ap{1,1}.P0_3D;
        data_base.db{2,1}.pts_3D = Ap{1,1}.P1_3D;
        data_base.db{3,1}.pts_3D = Ap{1,1}.P2_3D;
        data_base.db{4,1}.pts_3D = Ap{1,1}.P3_3D;

        data_base.db{1,1}.id_imgs = Ap{1, 1}.frame;
        data_base.db{2,1}.id_imgs = Ap{1, 1}.frame;
        data_base.db{3,1}.id_imgs = Ap{1, 1}.frame;
        data_base.db{4,1}.id_imgs = Ap{1, 1}.frame;
end

if ~isempty(tag1_n_group)
        data_base.db{4+1,1}.pts = Ap{1,2}.P0;
        data_base.db{4+2,1}.pts = Ap{1,2}.P1;
        data_base.db{4+3,1}.pts = Ap{1,2}.P2;
        data_base.db{4+4,1}.pts = Ap{1,2}.P3;

        data_base.db{4+1,1}.pts_3D = Ap{1,2}.P0_3D;
        data_base.db{4+2,1}.pts_3D = Ap{1,2}.P1_3D;
        data_base.db{4+3,1}.pts_3D = Ap{1,2}.P2_3D;
        data_base.db{4+4,1}.pts_3D = Ap{1,2}.P3_3D;

        data_base.db{4+1,1}.id_imgs = Ap{1, 2}.frame;
        data_base.db{4+2,1}.id_imgs = Ap{1, 2}.frame;
        data_base.db{4+3,1}.id_imgs = Ap{1, 2}.frame;
        data_base.db{4+4,1}.id_imgs = Ap{1, 2}.frame;
end

if ~isempty(tag2_n_group)
        data_base.db{8+1,1}.pts = Ap{1,3}.P0;
        data_base.db{8+2,1}.pts = Ap{1,3}.P1;
        data_base.db{8+3,1}.pts = Ap{1,3}.P2;
        data_base.db{8+4,1}.pts = Ap{1,3}.P3;

        data_base.db{8+1,1}.pts_3D = Ap{1,3}.P0_3D;
        data_base.db{8+2,1}.pts_3D = Ap{1,3}.P1_3D;
        data_base.db{8+3,1}.pts_3D = Ap{1,3}.P2_3D;
        data_base.db{8+4,1}.pts_3D = Ap{1,3}.P3_3D;

        data_base.db{8+1,1}.id_imgs = Ap{1, 3}.frame;
        data_base.db{8+2,1}.id_imgs = Ap{1, 3}.frame;
        data_base.db{8+3,1}.id_imgs = Ap{1, 3}.frame;
        data_base.db{8+4,1}.id_imgs = Ap{1, 3}.frame;
end

if ~isempty(tag3_n_group)
        data_base.db{12+1,1}.pts = Ap{1,4}.P0;
        data_base.db{12+2,1}.pts = Ap{1,4}.P1;
        data_base.db{12+3,1}.pts = Ap{1,4}.P2;
        data_base.db{12+4,1}.pts = Ap{1,4}.P3;

        data_base.db{12+1,1}.pts_3D = Ap{1,4}.P0_3D;
        data_base.db{12+2,1}.pts_3D = Ap{1,4}.P1_3D;
        data_base.db{12+3,1}.pts_3D = Ap{1,4}.P2_3D;
        data_base.db{12+4,1}.pts_3D = Ap{1,4}.P3_3D;

        data_base.db{12+1,1}.id_imgs = Ap{1, 4}.frame;
        data_base.db{12+2,1}.id_imgs = Ap{1, 4}.frame;
        data_base.db{12+3,1}.id_imgs = Ap{1, 4}.frame;
        data_base.db{12+4,1}.id_imgs = Ap{1, 4}.frame;
end

%% 
idx_frame_valid = [];
idx_tag_valid = [];
for ii = valid_data_num
    short_dist = 1e7;
    tag_flag = 0;
    non_tag_flag = 0;
    for j=1:4
        tt_tmp = find(Ap{1, j}.April_frame==ii);
        if ~isempty(tt_tmp)
            cam2tag_dist = norm(Ap{1, j}.T_cp{1, tt_tmp}(1:3,4));
            if  short_dist > cam2tag_dist
                short_dist = cam2tag_dist;
                frame_selected = ii;
                tag_select = j;
            end
            tag_flag = 1;
        end
    end
    if tag_flag==0
        for j=1:4
            tt_tmp = find(Ap{1, j}.frame==ii);
            if ~isempty(tt_tmp)
                cam2tag_dist = norm(Ap{1, j}.T_cp{1, tt_tmp}(1:3,4));
                if  short_dist > cam2tag_dist
                    short_dist = cam2tag_dist;
                    frame_selected = ii;
                    tag_select = j;
                end
                non_tag_flag = 1;
            end
        end
    end
    if or(tag_flag,non_tag_flag)
        idx_frame_valid = [idx_frame_valid, frame_selected];
        idx_tag_valid = [idx_tag_valid, tag_select];
    end
end

disp(['**Base Tag id: ',num2str(idx_frame_valid(1)-1)]);

%% Unify the direction to the base tag (T_k_bt)

T_k_bt = cell(1,length(idx_frame_valid));
for jj = 1:length(idx_frame_valid)
    frame_order = find(Ap{1, idx_tag_valid(jj)}.frame==idx_frame_valid(jj));
    T_tmp = Ap{1, idx_tag_valid(jj)}.T_cp{1, frame_order};
    if idx_tag_valid(1) == 1
        if idx_tag_valid(jj) == 1
            T_k_bt{1,jj} = T_tmp;
        elseif idx_tag_valid(jj) == 2
            T_k_bt{1,jj} = T_tmp*inv(T_01_f);
        elseif idx_tag_valid(jj) == 3
            T_k_bt{1,jj} = T_tmp*inv(T_02_f);
        elseif idx_tag_valid(jj) == 4
            T_k_bt{1,jj} = T_tmp*inv(T_03_f);
        end
    elseif idx_tag_valid(1) == 2
        if idx_tag_valid(jj) == 1
            T_k_bt{1,jj} = T_tmp*(T_01_f);
        elseif idx_tag_valid(jj) == 2
            T_k_bt{1,jj} = T_tmp;
        elseif idx_tag_valid(jj) == 3
            T_k_bt{1,jj} = T_tmp*inv(T_02_f)*(T_01_f);
        elseif idx_tag_valid(jj) == 4
            T_k_bt{1,jj} = T_tmp*inv(T_03_f)*(T_01_f);
        end
    elseif idx_tag_valid(1) == 3
        if idx_tag_valid(jj) == 1
            T_k_bt{1,jj} = T_tmp*(T_02_f);
        elseif idx_tag_valid(jj) == 2
            T_k_bt{1,jj} = T_tmp*inv(T_01_f)*(T_02_f);
        elseif idx_tag_valid(jj) == 3            
            T_k_bt{1,jj} = T_tmp;
        elseif idx_tag_valid(jj) == 4
            T_k_bt{1,jj} = T_tmp*inv(T_03_f)*(T_02_f);
        end
    elseif idx_tag_valid(1) == 4
        if idx_tag_valid(jj) == 1
            T_k_bt{1,jj} = T_tmp*(T_03_f);
        elseif idx_tag_valid(jj) == 2
            T_k_bt{1,jj} = T_tmp*inv(T_01_f)*(T_03_f);
        elseif idx_tag_valid(jj) == 3            
            T_k_bt{1,jj} = T_tmp*inv(T_02_f)*(T_03_f);
        elseif idx_tag_valid(jj) == 4
            T_k_bt{1,jj} = T_tmp;            
        end
    end
end

T_0k = cell(1,length(T_k_bt));
for jj=1:length(T_k_bt)
    T_0k{1,jj} = T_k_bt{1,1} * inv(T_k_bt{1,jj});
end

figure(3);
for i=1:length(T_k_bt)
    plot3(T_0k{1,i}(1,4),T_0k{1,i}(2,4),T_0k{1,i}(3,4),'r*'); hold on;
end
plot3(0,0,0,'rs');
axis equal;
xlabel('x[m]'); ylabel('y[m]'); zlabel('z [m]');
title('excavator trajectory');

figure(4);
for i=1:length(T_k_bt)
    T_k_bt_inv = inv(T_k_bt{1,i});
    plot3(T_k_bt_inv(1,4),T_k_bt_inv(2,4),T_k_bt_inv(3,4),'r*'); hold on;
end
plot3(0,0,0,'bs');
base_tag_point_name = ['id: ',num2str(idx_frame_valid(1)-1)];
text(0,0,0,base_tag_point_name,'FontSize',20,...
    'Position',[0,0,-0.5]);
axis equal;
xlabel('x[m]'); ylabel('y[m]'); zlabel('z [m]');
title(['Base Tag: ',num2str(idx_frame_valid(1)-1)]);
set(gca,'Zdir','reverse'); set(gca,'Xdir','reverse'); 
view(-120, 30);

%% Bundle Adjustment (BA)
idx_3d = 1:length(data_base.db);
points_trimmed = mean3DPoints(T_0k, data_base, idx_frame_valid, idx_tag_valid);
n_img_gap = length(T_k_bt);
K = intrinsics.Intrinsics.IntrinsicMatrix';

[hidden_state, observation, T_wk0_huber] = BA_initialze_huber(data_base, T_0k, points_trimmed, idx_3d, n_img_gap, idx_frame_valid);
[hidden_state_optimal, residual] = runBA_huber(hidden_state, observation, K);
[points_ba, T_c0ck_ba] = BA_after_huber(hidden_state_optimal, observation, residual, n_img_gap, T_wk0_huber);
% [points_ba, T_c0ck_ba] = alignGPS2BA(points_ba, T_c0ck_ba, T_0k);

figure(3);
for i=1:length(T_c0ck_ba)
    plot3(T_c0ck_ba{1,i}(1,4),T_c0ck_ba{1,i}(2,4),T_c0ck_ba{1,i}(3,4),'b*'); hold on;
end
plot3(T_c0ck_ba{1,1}(1,4),T_c0ck_ba{1,1}(2,4),T_c0ck_ba{1,1}(3,4),'bs','MarkerSize',20);
axis equal;
xlabel('x[m]'); ylabel('y[m]'); zlabel('z[m]');


figure(500);
hold on;
for i=1:length(points_ba)
    plot3(points_ba(1,i),points_ba(2,i),points_ba(3,i),'g+','MarkerSize',20,'LineWidth',5); hold on;
    if i<5
        point_name = ['p_{0',num2str(i-1),'}'];
    elseif i<9
        point_name = ['p_{1',num2str(i-5),'}'];
    elseif i<13
        point_name = ['p_{2',num2str(i-9),'}'];
    elseif i<17
        point_name = ['p_{3',num2str(i-13),'}'];
    end
    text(points_ba(1,i),points_ba(2,i),points_ba(3,i),point_name,'Color','black','FontSize',10,'FontWeight','bold');
end
axis equal;
xlabel('x[m]'); ylabel('y[m]'); zlabel('z[m]');
set(gca,'Zdir','reverse'); set(gca,'Xdir','reverse'); 
view(-270, 90);

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

figure(5);
pcshow(fixed.Location,repmat([1 0 1],size(fixed.Location,1),1),'Markersize',50);
hold on;
pcshow(moving.Location,repmat([1 1 0],size(moving.Location,1),1),'Markersize',50);
plot3(fixed.Location(1,1),fixed.Location(1,2), fixed.Location(1,3),'ms','MarkerSize',20);
plot3(moving.Location(1,1),moving.Location(1,2), moving.Location(1,3),'ys','MarkerSize',20);
axis equal;
xlabel('x[m]'); ylabel('y[m]'); zlabel('z[m]');
title('Excavator trajectory','FontWeight','bold');
legend('before BA', 'after BA','start(before)','start(after)', 'TextColor','white','FontWeight','bold');

tag0 = points_ba(:,1:4);
tag1 = points_ba(:,5:8);
tag2 = points_ba(:,9:12);
tag3 = points_ba(:,13:16);

tag0_mid = mean(tag0,2);
tag0_x = mean(tag0(:,2:3),2) - tag0_mid ;
tag0_x = tag0_x./norm(tag0_x); %% x
tag0_y = mean(tag0(:,1:2),2) - tag0_mid ;
tag0_y = tag0_y./norm(tag0_y);
tag0_z = cross(tag0_x,tag0_y);
tag0_z = tag0_z./norm(tag0_z); %% z
tag0_y = cross(tag0_z,tag0_x); %% y

tag1_mid = mean(tag1,2);
tag1_x = mean(tag1(:,2:3),2) - tag1_mid ;
tag1_x = tag1_x./norm(tag1_x); %% x
tag1_y = mean(tag1(:,1:2),2) - tag1_mid ;
tag1_y = tag1_y./norm(tag1_y);
tag1_z = cross(tag1_x,tag1_y);
tag1_z = tag1_z./norm(tag1_z); %% z
tag1_y = cross(tag1_z,tag1_x); %% y

tag2_mid = mean(tag2,2);
tag2_x = mean(tag2(:,2:3),2) - tag2_mid ;
tag2_x = tag2_x./norm(tag2_x); %% x
tag2_y = mean(tag2(:,1:2),2) - tag2_mid ;
tag2_y = tag2_y./norm(tag2_y);
tag2_z = cross(tag2_x,tag2_y);
tag2_z = tag2_z./norm(tag2_z); %% z
tag2_y = cross(tag2_z,tag2_x); %% y

tag3_mid = mean(tag3,2);
tag3_x = mean(tag3(:,2:3),2) - tag3_mid ;
tag3_x = tag3_x./norm(tag3_x); %% x
tag3_y = mean(tag3(:,1:2),2) - tag3_mid ;
tag3_y = tag3_y./norm(tag3_y);
tag3_z = cross(tag3_x,tag3_y);
tag3_z = tag3_z./norm(tag3_z); %% z
tag3_y = cross(tag3_z,tag3_x); %% y

figure(500);
hold on;
plot3([tag0_mid(1) tag0_mid(1)+0.5*tag0_x(1)], [tag0_mid(2) tag0_mid(2)+0.5*tag0_x(2)],...
    [tag0_mid(3) tag0_mid(3)+0.5*tag0_x(3)],'r','LineWidth',3);
plot3([tag0_mid(1) tag0_mid(1)+0.5*tag0_y(1)], [tag0_mid(2) tag0_mid(2)+0.5*tag0_y(2)],...
    [tag0_mid(3) tag0_mid(3)+0.5*tag0_y(3)],'g','LineWidth',3);
plot3([tag0_mid(1) tag0_mid(1)+0.5*tag0_z(1)], [tag0_mid(2) tag0_mid(2)+0.5*tag0_z(2)],...
    [tag0_mid(3) tag0_mid(3)+0.5*tag0_z(3)],'b','LineWidth',3);

plot3([tag1_mid(1) tag1_mid(1)+0.5*tag1_x(1)], [tag1_mid(2) tag1_mid(2)+0.5*tag1_x(2)],...
    [tag1_mid(3) tag1_mid(3)+0.5*tag1_x(3)],'r','LineWidth',3);
plot3([tag1_mid(1) tag1_mid(1)+0.5*tag1_y(1)], [tag1_mid(2) tag1_mid(2)+0.5*tag1_y(2)],...
    [tag1_mid(3) tag1_mid(3)+0.5*tag1_y(3)],'g','LineWidth',3);
plot3([tag1_mid(1) tag1_mid(1)+0.5*tag1_z(1)], [tag1_mid(2) tag1_mid(2)+0.5*tag1_z(2)],...
    [tag1_mid(3) tag1_mid(3)+0.5*tag1_z(3)],'b','LineWidth',3);

plot3([tag2_mid(1) tag2_mid(1)+0.5*tag2_x(1)], [tag2_mid(2) tag2_mid(2)+0.5*tag2_x(2)],...
    [tag2_mid(3) tag2_mid(3)+0.5*tag2_x(3)],'r','LineWidth',3);
plot3([tag2_mid(1) tag2_mid(1)+0.5*tag2_y(1)], [tag2_mid(2) tag2_mid(2)+0.5*tag2_y(2)],...
    [tag2_mid(3) tag2_mid(3)+0.5*tag2_y(3)],'g','LineWidth',3);
plot3([tag2_mid(1) tag2_mid(1)+0.5*tag2_z(1)], [tag2_mid(2) tag2_mid(2)+0.5*tag2_z(2)],...
    [tag2_mid(3) tag2_mid(3)+0.5*tag2_z(3)],'b','LineWidth',3);

plot3([tag3_mid(1) tag3_mid(1)+0.5*tag3_x(1)], [tag3_mid(2) tag3_mid(2)+0.5*tag3_x(2)],...
    [tag3_mid(3) tag3_mid(3)+0.5*tag3_x(3)],'r','LineWidth',3);
plot3([tag3_mid(1) tag3_mid(1)+0.5*tag3_y(1)], [tag3_mid(2) tag3_mid(2)+0.5*tag3_y(2)],...
    [tag3_mid(3) tag3_mid(3)+0.5*tag3_y(3)],'g','LineWidth',3);
plot3([tag3_mid(1) tag3_mid(1)+0.5*tag3_z(1)], [tag3_mid(2) tag3_mid(2)+0.5*tag3_z(2)],...
    [tag3_mid(3) tag3_mid(3)+0.5*tag3_z(3)],'b','LineWidth',3);

T_w0_ba = [tag0_x, tag0_y, tag0_z, tag0_mid ; 0 0 0 1];
T_w1_ba = [tag1_x, tag1_y, tag1_z, tag1_mid ; 0 0 0 1];
T_w2_ba = [tag2_x, tag2_y, tag2_z, tag2_mid ; 0 0 0 1];
T_w3_ba = [tag3_x, tag3_y, tag3_z, tag3_mid ; 0 0 0 1];

T_00_ba = inv(T_w0_ba)*T_w0_ba;
T_01_ba = inv(T_w0_ba)*T_w1_ba;
T_02_ba = inv(T_w0_ba)*T_w2_ba;
T_03_ba = inv(T_w0_ba)*T_w3_ba;

figure(600);
plot3(0,0,0,'m*'); hold on;
text(0,0,0,'id0','FontSize',20,...
    'Position',[0,0,+0.2]);
plot3([0,0.5],[0,0],[0,0],'r','LineWidth',3);
plot3([0,0],[0,0.5],[0,0],'g','LineWidth',3);
plot3([0,0],[0,0],[0,0.5],'b','LineWidth',3);

plot3(T_01_ba(1,4),T_01_ba(2,4),T_01_ba(3,4),'m*');
text(T_01_ba(1,4),T_01_ba(2,4),T_01_ba(3,4),'id1','FontSize',20,...
    'Position',[T_01_ba(1,4),T_01_ba(2,4),T_01_ba(3,4)+0.2]);
plot3([T_01_ba(1,4),T_01_ba(1,4)+0.5*T_01_ba(1,1)],[T_01_ba(2,4),T_01_ba(2,4)+0.5*T_01_ba(2,1)],[T_01_ba(3,4),T_01_ba(3,4)+0.5*T_01_ba(3,1)],'r','LineWidth',3);
plot3([T_01_ba(1,4),T_01_ba(1,4)+0.5*T_01_ba(1,2)],[T_01_ba(2,4),T_01_ba(2,4)+0.5*T_01_ba(2,2)],[T_01_ba(3,4),T_01_ba(3,4)+0.5*T_01_ba(3,2)],'g','LineWidth',3);
plot3([T_01_ba(1,4),T_01_ba(1,4)+0.5*T_01_ba(1,3)],[T_01_ba(2,4),T_01_ba(2,4)+0.5*T_01_ba(2,3)],[T_01_ba(3,4),T_01_ba(3,4)+0.5*T_01_ba(3,3)],'b','LineWidth',3);

plot3(T_02_ba(1,4),T_02_ba(2,4),T_02_ba(3,4),'m*');
text(T_02_ba(1,4),T_02_ba(2,4),T_02_ba(3,4),'id2','FontSize',20,...
    'Position',[T_02_ba(1,4),T_02_ba(2,4),T_02_ba(3,4)+0.2]);
plot3([T_02_ba(1,4),T_02_ba(1,4)+0.5*T_02_ba(1,1)],[T_02_ba(2,4),T_02_ba(2,4)+0.5*T_02_ba(2,1)],[T_02_ba(3,4),T_02_ba(3,4)+0.5*T_02_ba(3,1)],'r','LineWidth',3);
plot3([T_02_ba(1,4),T_02_ba(1,4)+0.5*T_02_ba(1,2)],[T_02_ba(2,4),T_02_ba(2,4)+0.5*T_02_ba(2,2)],[T_02_ba(3,4),T_02_ba(3,4)+0.5*T_02_ba(3,2)],'g','LineWidth',3);
plot3([T_02_ba(1,4),T_02_ba(1,4)+0.5*T_02_ba(1,3)],[T_02_ba(2,4),T_02_ba(2,4)+0.5*T_02_ba(2,3)],[T_02_ba(3,4),T_02_ba(3,4)+0.5*T_02_ba(3,3)],'b','LineWidth',3);

plot3(T_03_ba(1,4),T_03_ba(2,4),T_03_ba(3,4),'m*');
text(T_03_ba(1,4),T_03_ba(2,4),T_03_ba(3,4),'id3','FontSize',20,...
    'Position',[T_03_ba(1,4),T_03_ba(2,4),T_03_ba(3,4)+0.2]);
plot3([T_03_ba(1,4),T_03_ba(1,4)+0.5*T_03_ba(1,1)],[T_03_ba(2,4),T_03_ba(2,4)+0.5*T_03_ba(2,1)],[T_03_ba(3,4),T_03_ba(3,4)+0.5*T_03_ba(3,1)],'r','LineWidth',3);
plot3([T_03_ba(1,4),T_03_ba(1,4)+0.5*T_03_ba(1,2)],[T_03_ba(2,4),T_03_ba(2,4)+0.5*T_03_ba(2,2)],[T_03_ba(3,4),T_03_ba(3,4)+0.5*T_03_ba(3,2)],'g','LineWidth',3);
plot3([T_03_ba(1,4),T_03_ba(1,4)+0.5*T_03_ba(1,3)],[T_03_ba(2,4),T_03_ba(2,4)+0.5*T_03_ba(2,3)],[T_03_ba(3,4),T_03_ba(3,4)+0.5*T_03_ba(3,3)],'b','LineWidth',3);
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
set(gca,'Zdir','reverse'); set(gca,'Xdir','reverse'); 
view(-120, 30);
% zlim([-0.5 0.5]);
title('After BA : relative poses between AprilTag 0 1 2 3');

quat_00 = rotm2quat(T_00_ba(1:3,1:3));
quat_01 = rotm2quat(T_01_ba(1:3,1:3));
quat_02 = rotm2quat(T_02_ba(1:3,1:3));
quat_03 = rotm2quat(T_03_ba(1:3,1:3));

% fprintf(['{id: 0, size: 0.24, x: %f, y: %f, z: %f, qw: %f, qx: %f, qy: %f, qz: %f},\n' ...
%     '{id: 1, size: 0.24, x: %f, y: %f, z: %f, qw: %f, qx: %f, qy: %f, qz: %f},\n' ...
%     '{id: 2, size: 0.24, x: %f, y: %f, z: %f, qw: %f, qx: %f, qy: %f, qz: %f},\n' ...
%     '{id: 3, size: 0.24, x: %f, y: %f, z: %f, qw: %f, qx: %f, qy: %f, qz: %f}\n'],...
%     T_00_ba(1,4),T_00_ba(2,4),T_00_ba(3,4),quat_00(1),quat_00(2),quat_00(3),quat_00(4),...
%     T_01_ba(1,4),T_01_ba(2,4),T_01_ba(3,4),quat_01(1),quat_01(2),quat_01(3),quat_01(4),...
%     T_02_ba(1,4),T_02_ba(2,4),T_02_ba(4,3),quat_02(1),quat_02(2),quat_02(3),quat_02(4),...
%     T_03_ba(1,4),T_03_ba(2,4),T_03_ba(3,4),quat_03(1),quat_03(2),quat_03(3),quat_03(4));

fprintf(['x_00: %.8f\n','y_00: %.8f\n','z_00: %.8f\n','qw_00: %.8f\n','qx_00: %.8f\n','qy_00: %.8f\n','qz_00: %.8f\n', ...
         'x_01: %.8f\n','y_01: %.8f\n','z_01: %.8f\n','qw_01: %.8f\n','qx_01: %.8f\n','qy_01: %.8f\n','qz_01: %.8f\n', ...
         'x_02: %.8f\n','y_02: %.8f\n','z_02: %.8f\n','qw_02: %.8f\n','qx_02: %.8f\n','qy_02: %.8f\n','qz_02: %.8f\n',...
         'x_03: %.8f\n','y_03: %.8f\n','z_03: %.8f\n','qw_03: %.8f\n','qx_03: %.8f\n','qy_03: %.8f\n','qz_03: %.8f\n'],...
    T_00_ba(1,4),T_00_ba(2,4),T_00_ba(3,4),quat_00(1),quat_00(2),quat_00(3),quat_00(4),...
    T_01_ba(1,4),T_01_ba(2,4),T_01_ba(3,4),quat_01(1),quat_01(2),quat_01(3),quat_01(4),...
    T_02_ba(1,4),T_02_ba(2,4),T_02_ba(4,3),quat_02(1),quat_02(2),quat_02(3),quat_02(4),...
    T_03_ba(1,4),T_03_ba(2,4),T_03_ba(3,4),quat_03(1),quat_03(2),quat_03(3),quat_03(4));

[1.00000000, 0.00000000, 0.00000000, 0.00000000
 0.00000000, 1.00000000, 0.00000000, 0.00000000
 0.00000000, 0.00000000, 1.00000000, 0.00000000
 0.00000000, 0.00000000, 0.00000000, 1.00000000]
[0.99997627, 0.00200334, 0.00659195, -0.00188738
 -0.00221700, 0.99946713, 0.03256606, 1.53553985
 -0.00652319, -0.03257991, 0.99944785, -0.01065190
 0.00000000, 0.00000000, 0.00000000, 1.00000000]
[0.99979205, 0.00206859, -0.02028735, 2.61801923
 -0.00213255, 0.99999282, -0.00313177, 0.00012741
 0.02028073, 0.00317438, 0.99978929, 0.00000000
 0.00000000, 0.00000000, 0.00000000, 1.00000000]
[0.99746084, -0.02525298, -0.06658944, 2.61658924
 0.02634912, 0.99953053, 0.01563448, 1.47980057
 0.06616336, -0.01734936, 0.99765796, 0.07332305
 0.00000000, 0.00000000, 0.00000000, 1.00000000]

