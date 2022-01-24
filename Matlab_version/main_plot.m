%% calculate relative poses between tag 0 1 2 3
% T_01_f, T_02_f, T_03_f 

for u = 1:length(Ap{1, 1}.April_frame)
    April0_idx(1,u) = find(Ap{1, 1}.frame == Ap{1, 1}.April_frame(u));
end
for u = 1:length(Ap{1, 2}.April_frame)
    April1_idx(1,u) = find(Ap{1, 2}.frame == Ap{1, 2}.April_frame(u));
end
for u = 1:length(Ap{1, 3}.April_frame)
    April2_idx(1,u) = find(Ap{1, 3}.frame == Ap{1, 3}.April_frame(u));
end
for u = 1:length(Ap{1, 4}.April_frame)
    April3_idx(1,u) = find(Ap{1, 4}.frame == Ap{1, 4}.April_frame(u));
end

for u = 1:length(Ap{1, 1}.T_cp)
    traj0_tmp = inv(Ap{1, 1}.T_cp{1, u});
    traj0(:,u) = traj0_tmp(1:3,4);
end
figure(100);
% subplot(2,2,2);
pcshow(traj0',repmat([1 0 0],size(traj0,2),1),'Markersize',50);
hold on;
pcshow(traj0(:,April0_idx)',repmat([1 1 0],size(traj0(:,April0_idx),2),1),'Markersize',50);
plot3(0,0,0,'y*');
plot3(traj0(1,1),traj0(2,1),traj0(3,1),'sw','MarkerSize',20);
plot3([0 1],[0 0],[0 0],'r');
plot3([0 0],[0 1],[0 0],'g');
plot3([0 0],[0 0],[0 1],'b');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('id0','FontSize',20,'FontWeight','bold','Color','yellow');
legend('all frames','AprilTag frame','origin','start point','x-axis','y-axis','z-axis','TextColor','white');
set(gca,'Zdir','reverse'); set(gca,'Xdir','reverse'); 
view(-120, 30);

for u = 1:length(Ap{1, 2}.T_cp)
    traj1_tmp = inv(Ap{1, 2}.T_cp{1, u});
    traj1(:,u) = traj1_tmp(1:3,4);
end
figure(101);
% subplot(2,2,1);
pcshow(traj1',repmat([1 0 0],size(traj1,2),1),'Markersize',50);
hold on;
pcshow(traj1(:,April1_idx)',repmat([1 1 0],size(traj1(:,April1_idx),2),1),'Markersize',50);
plot3(0,0,0,'y*');
plot3(traj1(1,1),traj1(2,1),traj1(3,1),'sw','MarkerSize',20);
plot3([0 1],[0 0],[0 0],'r');
plot3([0 0],[0 1],[0 0],'g');
plot3([0 0],[0 0],[0 1],'b');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('id1','FontSize',20,'FontWeight','bold','Color','yellow');
legend('all frames','AprilTag frame','origin','start point','x-axis','y-axis','z-axis','TextColor','white');
set(gca,'Zdir','reverse'); set(gca,'Xdir','reverse'); 
view(-120, 30);

for u = 1:length(Ap{1, 3}.T_cp)
    traj2_tmp = inv(Ap{1, 3}.T_cp{1, u});
    traj2(:,u) = traj2_tmp(1:3,4);
end
figure(102);
% subplot(2,2,4);
pcshow(traj2',repmat([1 0 0],size(traj2,2),1),'Markersize',50);
hold on;
pcshow(traj2(:,April2_idx)',repmat([1 1 0],size(traj2(:,April2_idx),2),1),'Markersize',50);
plot3(0,0,0,'y*');
plot3(traj2(1,1),traj2(2,1),traj2(3,1),'sw','MarkerSize',20);
plot3([0 1],[0 0],[0 0],'r');
plot3([0 0],[0 1],[0 0],'g');
plot3([0 0],[0 0],[0 1],'b');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('id2','FontSize',20,'FontWeight','bold','Color','yellow');
legend('all frames','AprilTag frame','origin','start point','x-axis','y-axis','z-axis','TextColor','white');
set(gca,'Zdir','reverse'); set(gca,'Xdir','reverse'); 
view(-120, 30);

for u = 1:length(Ap{1, 4}.T_cp)
    traj3_tmp = inv(Ap{1, 4}.T_cp{1, u});
    traj3(:,u) = traj3_tmp(1:3,4);
end
figure(103);
% subplot(2,2,3);
pcshow(traj3',repmat([1 0 0],size(traj3,2),1),'Markersize',50);
hold on;
pcshow(traj3(:,April3_idx)',repmat([1 1 0],size(traj3(:,April3_idx),2),1),'Markersize',50);
plot3(0,0,0,'y*');
plot3(traj3(1,1),traj3(2,1),traj3(3,1),'sw','MarkerSize',20);
plot3([0 1],[0 0],[0 0],'r');
plot3([0 0],[0 1],[0 0],'g');
plot3([0 0],[0 0],[0 1],'b');
xlabel('x [m]'); ylabel('y [m]'); zlabel('z [m]');
title('id3','FontSize',20,'FontWeight','bold','Color','yellow');
legend('all frames','AprilTag frame','origin','start point','x-axis','y-axis','z-axis','TextColor','white');
set(gca,'Zdir','reverse'); set(gca,'Xdir','reverse'); 
view(-120, 30);

%% T_01 only using AprilTag
idx01_0 = [];
idx01_1 = [];
cnt_share = 1;
for u = 1:length(Ap{1, 2}.April_frame)
    if ~isempty(find(Ap{1, 1}.April_frame == Ap{1, 2}.April_frame(u)))
        idx01_0(1,cnt_share) = find(Ap{1, 1}.April_frame == Ap{1, 2}.April_frame(u));
        idx01_1(1,cnt_share) = u;
        cnt_share = cnt_share + 1;
    end
end
frame_num_01 = Ap{1, 1}.April_frame(idx01_0);
T_01 = cell(1,length(idx01_0));
for i =1:length(idx01_0)
    T_01{i} = inv(Ap{1, 1}.T_April{1,idx01_0(i)}) * Ap{1, 2}.T_April{1,idx01_1(i)};
end
for i = 1: length(T_01)
    eul_01(i,:) = rotm2eul(T_01{1,i}(1:3,1:3));
    xyz_01(i,:) = T_01{1,i}(1:3,4);
end
eul_01_median = median(eul_01);
xyz_01_median = median(xyz_01);
T_01_median = [eul2rotm(eul_01_median), xyz_01_median' ; 0 0 0 1];

%% T_13 only using AprilTag
cnt_share = 1;
for u = 1:length(Ap{1, 4}.April_frame)
    if ~isempty(find(Ap{1, 2}.April_frame == Ap{1, 4}.April_frame(u)))
        idx13_1(1,cnt_share) = find(Ap{1, 2}.April_frame == Ap{1, 4}.April_frame(u));
        idx13_3(1,cnt_share) = u;
        cnt_share = cnt_share + 1;
    end
end
frame_num_13 = Ap{1, 2}.April_frame(idx13_1);
T_13 = cell(1,length(idx13_1));
for i =1:length(idx13_1)
    T_13{i} = inv(Ap{1, 2}.T_April{1,idx13_1(i)}) * Ap{1, 4}.T_April{1,idx13_3(i)};
end
for i = 1: length(T_13)
    eul_13(i,:) = rotm2eul(T_13{1,i}(1:3,1:3));
    xyz_13(i,:) = T_13{1,i}(1:3,4);
end
eul_13_median = median(eul_13);
xyz_13_median = median(xyz_13);
T_13_median = [eul2rotm(eul_13_median), xyz_13_median' ; 0 0 0 1];

%% T_32 only using AprilTag
cnt_share = 1;
for u = 1:length(Ap{1, 3}.April_frame)
    if ~isempty(find(Ap{1, 4}.April_frame == Ap{1, 3}.April_frame(u)))
        idx32_3(1,cnt_share) = find(Ap{1, 4}.April_frame == Ap{1, 3}.April_frame(u));
        idx32_2(1,cnt_share) = u;
        cnt_share = cnt_share + 1;
    end
end
frame_num_32 = Ap{1, 4}.April_frame(idx32_3);
T_32 = cell(1,length(idx32_3));
for i =1:length(idx32_3)
    T_32{i} = inv(Ap{1, 4}.T_April{1,idx32_3(i)}) * Ap{1, 3}.T_April{1,idx32_2(i)};
end
for i = 1: length(T_32)
    eul_32(i,:) = rotm2eul(T_32{1,i}(1:3,1:3));
    xyz_32(i,:) = T_32{1,i}(1:3,4);
end
eul_32_median = median(eul_32);
xyz_32_median = median(xyz_32);
T_32_median = [eul2rotm(eul_32_median), xyz_32_median' ; 0 0 0 1];

%%20
cnt_share = 1;
for u = 1:length(Ap{1, 1}.April_frame)
    if ~isempty(find(Ap{1, 3}.April_frame == Ap{1, 1}.April_frame(u)))
        idx20_2(1,cnt_share) = find(Ap{1, 3}.April_frame == Ap{1, 1}.April_frame(u));
        idx20_0(1,cnt_share) = u;
        cnt_share = cnt_share + 1;
    end
end
frame_num_20 = Ap{1, 3}.April_frame(idx20_2);
T_20 = cell(1,length(idx20_2));
for i =1:length(idx20_2)
    T_20{i} = inv(Ap{1, 3}.T_April{1,idx20_2(i)}) * Ap{1, 1}.T_April{1,idx20_0(i)};
end
for i = 1: length(T_20)
    eul_20(i,:) = rotm2eul(T_20{1,i}(1:3,1:3));
    xyz_20(i,:) = T_20{1,i}(1:3,4);
end
eul_20_median = median(eul_20);
xyz_20_median = median(xyz_20);
T_20_median = [eul2rotm(eul_20_median), xyz_20_median' ; 0 0 0 1];

T_01_f = T_01_median;
T_02_f = T_01_median*T_13_median*T_32_median;
T_03_f = T_01_median*T_13_median;

figure(2);
plot3(0,0,0,'m*'); hold on;
text(0,0,0,'id0','FontSize',20,...
    'Position',[0,0,-0.2]);
plot3([0,0.5],[0,0],[0,0],'r','LineWidth',3);
plot3([0,0],[0,0.5],[0,0],'g','LineWidth',3);
plot3([0,0],[0,0],[0,0.5],'b','LineWidth',3);

plot3(T_01_f(1,4),T_01_f(2,4),T_01_f(3,4),'m*');
text(T_01_f(1,4),T_01_f(2,4),T_01_f(3,4),'id1','FontSize',20,...
    'Position',[T_01_f(1,4),T_01_f(2,4),T_01_f(3,4)-0.2]);
plot3([T_01_f(1,4),T_01_f(1,4)+0.5*T_01_f(1,1)],[T_01_f(2,4),T_01_f(2,4)+0.5*T_01_f(2,1)],[T_01_f(3,4),T_01_f(3,4)+0.5*T_01_f(3,1)],'r','LineWidth',3);
plot3([T_01_f(1,4),T_01_f(1,4)+0.5*T_01_f(1,2)],[T_01_f(2,4),T_01_f(2,4)+0.5*T_01_f(2,2)],[T_01_f(3,4),T_01_f(3,4)+0.5*T_01_f(3,2)],'g','LineWidth',3);
plot3([T_01_f(1,4),T_01_f(1,4)+0.5*T_01_f(1,3)],[T_01_f(2,4),T_01_f(2,4)+0.5*T_01_f(2,3)],[T_01_f(3,4),T_01_f(3,4)+0.5*T_01_f(3,3)],'b','LineWidth',3);

plot3(T_02_f(1,4),T_02_f(2,4),T_02_f(3,4),'m*');
text(T_02_f(1,4),T_02_f(2,4),T_02_f(3,4),'id2','FontSize',20,...
    'Position',[T_02_f(1,4),T_02_f(2,4),T_02_f(3,4)-0.2]);
plot3([T_02_f(1,4),T_02_f(1,4)+0.5*T_02_f(1,1)],[T_02_f(2,4),T_02_f(2,4)+0.5*T_02_f(2,1)],[T_02_f(3,4),T_02_f(3,4)+0.5*T_02_f(3,1)],'r','LineWidth',3);
plot3([T_02_f(1,4),T_02_f(1,4)+0.5*T_02_f(1,2)],[T_02_f(2,4),T_02_f(2,4)+0.5*T_02_f(2,2)],[T_02_f(3,4),T_02_f(3,4)+0.5*T_02_f(3,2)],'g','LineWidth',3);
plot3([T_02_f(1,4),T_02_f(1,4)+0.5*T_02_f(1,3)],[T_02_f(2,4),T_02_f(2,4)+0.5*T_02_f(2,3)],[T_02_f(3,4),T_02_f(3,4)+0.5*T_02_f(3,3)],'b','LineWidth',3);

plot3(T_03_f(1,4),T_03_f(2,4),T_03_f(3,4),'m*');
text(T_03_f(1,4),T_03_f(2,4),T_03_f(3,4),'id3','FontSize',20,...
    'Position',[T_03_f(1,4),T_03_f(2,4),T_03_f(3,4)-0.2]);
plot3([T_03_f(1,4),T_03_f(1,4)+0.5*T_03_f(1,1)],[T_03_f(2,4),T_03_f(2,4)+0.5*T_03_f(2,1)],[T_03_f(3,4),T_03_f(3,4)+0.5*T_03_f(3,1)],'r','LineWidth',3);
plot3([T_03_f(1,4),T_03_f(1,4)+0.5*T_03_f(1,2)],[T_03_f(2,4),T_03_f(2,4)+0.5*T_03_f(2,2)],[T_03_f(3,4),T_03_f(3,4)+0.5*T_03_f(3,2)],'g','LineWidth',3);
plot3([T_03_f(1,4),T_03_f(1,4)+0.5*T_03_f(1,3)],[T_03_f(2,4),T_03_f(2,4)+0.5*T_03_f(2,3)],[T_03_f(3,4),T_03_f(3,4)+0.5*T_03_f(3,3)],'b','LineWidth',3);
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
set(gca,'Zdir','reverse'); set(gca,'Xdir','reverse'); 
view(-120, 30);
zlim([-0.5 0.5]);
title('Relative poses between AprilTag 0 1 2 3');