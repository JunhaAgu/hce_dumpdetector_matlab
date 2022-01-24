clc; close all; clear all;

%% 검증용으로 사용할 수 있음. 이미지 한개씩 쓰기위함.

%%
% data_folder = 'D:\★기업과제\굴삭기 2차년도\2021_08_11_experiment\20210811_HCE_exp\hce_data';
data_folder = 'D:\★기업과제\굴삭기 2차년도\2021_08_26_experiment';
% data_name = '2021-08-11_17_10_27';
% data_name = 'markers_swing1_exp120';
data_name = 'markers_boom1_exp120';
cam_num = 0;

for iii =4  %[2:35,41:149] %[2:93, 98:159] 
data_num = iii ;

I = imread([data_folder,'\',data_name, '\', 'cam', num2str(cam_num), '\', num2str(data_num),'.png']);
tagFamily = "tag36h11";

% figure(); imshow(I);
data = load(['intrinsic_cam', num2str(cam_num), '.mat']);
intrinsics = data.cameraParams;

% tagSize = 0.161;
tagSize = 0.233;

I = undistortImage(I,intrinsics,"OutputView","same");
[id,loc,pose] = readAprilTag(I,"tag36h11",intrinsics,tagSize);

worldPoints = [0 0 0; tagSize/2 0 0; 0 tagSize/2 0; 0 0 tagSize/2];

for i = 1:length(pose)
    % Get image coordinates for axes.
    imagePoints = worldToImage(intrinsics,pose(i).Rotation, ...
                  pose(i).Translation,worldPoints);

    % Draw colored axes.
    I = insertShape(I,"Line",[imagePoints(1,:) imagePoints(2,:); ...
        imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
        "Color",["red","green","blue"],"LineWidth",7);

%     I = insertText(I,loc(1,:,i),id(i),"BoxOpacity",1,"FontSize",10);
end
figure(1); imshow(I);
if length(id) ==4
   fprintf("wow \n"); 
end

if isempty(id)
   fprintf("wow \n"); 
end
end
%% projection
% load('pts_3D.mat'); %3d points
% idx = find(id==3);
% T_k = zeros(4,4);
% T_k(4,4) =1;
% T_k(1:3,1:3) = pose(idx).Rotation';
% T_k(1:3,4) = pose(idx).Translation';
% 
% P_k0 = T_k*[pts_3D ; ones(1,size(pts_3D,2))];
% uv = intrinsics.IntrinsicMatrix' * P_k0(1:3,:);
% uv = uv ./ repmat(uv(3,:),3,1);
% uv(3,:) = [];
% figure(1); hold on;
% plot(uv(1,:),uv(2,:),'*b');
% 
% l1 = line([uv(1,1), uv(1,2)], [uv(2,1), uv(2,2)] );
% l1.Color = 'green';
% l1.LineStyle = '--';
% l1.LineWidth = 3;
% 
% l2 = line([uv(1,2), uv(1,4)], [uv(2,2), uv(2,4)] );
% l2.Color = 'green';
% l2.LineStyle = '--';
% l2.LineWidth = 3;
% 
% l3 = line([uv(1,4), uv(1,3)], [uv(2,4), uv(2,3)] );
% l3.Color = 'green';
% l3.LineStyle = '--';
% l3.LineWidth = 3;
% 
% l4 = line([uv(1,3), uv(1,1)], [uv(2,3), uv(2,1)] );
% l4.Color = 'green';
% l4.LineStyle = '--';
% l4.LineWidth = 3;
% 
% T_30 = (pose(1, 4).T')^-1*(pose(1, 1).T');
% T_31 = (pose(1, 4).T')^-1*(pose(1, 2).T');
% T_32 = (pose(1, 4).T')^-1*(pose(1, 3).T');
% T_34 = (pose(1, 4).T')^-1*(pose(1, 5).T');
% tag0 = T_30(1:3,4)';
% tag1 = T_31(1:3,4)';
% tag2 = T_32(1:3,4)';
% tag4 = T_34(1:3,4)';

T_10 = (pose(1, 2).T')^-1*(pose(1, 1).T');
T_12 = (pose(1, 2).T')^-1*(pose(1, 3).T');
T_13 = (pose(1, 2).T')^-1*(pose(1, 4).T');

