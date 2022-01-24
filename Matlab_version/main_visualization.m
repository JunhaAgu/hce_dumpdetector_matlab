clc; close all; clear all;

%% video 저장용으로 연속된 이미지에서 ROI tracking 하는 script.

%%
data_folder = 'D:\★기업과제\굴삭기 2차년도\2021_08_26_experiment';
% data_name = 'markers_swing1_exp120';
data_name = 'markers_boom1_exp120';
cam_num = 0;

f = fopen([data_folder,'\',data_name ,'\association.txt'],'r');
fgets(f);
n_line = 0;
while(1)
    lines = fgets(f);
    if(lines==-1)
        break;
    else
        n_line = n_line +1;
    end
end
n_data = n_line;

data_path = 'D:\★기업과제\굴삭기 2차년도\Apriltag 인식2\mat_boom\';
load([data_path, 'pts_3D.mat']); %3d points

%% tracking ROI video

video_flag = 0;

if video_flag==1
    framerate = 10;
    vid_image = VideoWriter('0826_boom');
    vid_image.FrameRate = framerate;
    open(vid_image);
end

T_k1_all = {};
%% 
for data_num = [2:35,41:149]
% for data_num = [2:93, 98:159]
    
I = imread([data_folder,'\',data_name, '\', 'cam', num2str(cam_num), '\', num2str(data_num),'.png']);
tagFamily = "tag36h11";

% imshow(I);
data = load(['intrinsic_cam', num2str(cam_num), '.mat']);
intrinsics = data.cameraParams;

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

    I = insertText(I,loc(1,:,i),id(i),"BoxOpacity",1,"FontSize",10);
end

%% projection
load([data_path, 'T_10.mat']);
load([data_path, 'T_12.mat']);
load([data_path, 'T_13.mat']);

idx = find(id==1);
if isempty(idx)
    idx = find(id==0);
    if isempty(idx)
        idx = find(id==2);
        if isempty(idx)
            idx = find(id==3);
            if isempty(idx)
                imshow(I);
                T_k1 = zeros(4,4);
                T_k1_all = [T_k1_all, T_k1];
                continue;
            end
        end
    end
end

see_marker = id(idx);
T_kx = zeros(4,4);
T_kx(4,4) =1;
T_kx(1:3,1:3) = pose(idx).Rotation';
T_kx(1:3,4) = pose(idx).Translation';
if see_marker == 1
    T_k1 = T_kx;
elseif see_marker ==0
    T_k1 = T_kx * T_10^-1;
elseif see_marker ==2
    T_k1 = T_kx * T_12^-1;
elseif see_marker ==3
    T_k1 = T_kx * T_13^-1;
end

T_k1_all = [T_k1_all, T_k1];

P_k1 = T_k1*[pts_3D ; ones(1,size(pts_3D,2))];
uv = intrinsics.IntrinsicMatrix' * P_k1(1:3,:);
uv = uv ./ repmat(uv(3,:),3,1);
uv(3,:) = [];

% figure(1); 
% plot(uv(1,:),uv(2,:),'*c'); 

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

I = insertShape(I,"Line",[uv(:,1)' uv(:,2)'; ...
        uv(:,2)' uv(:,4)'; uv(:,4)' uv(:,3)'; uv(:,3)' uv(:,1)'], ...
        "Color",["green","green","green","green"],"LineWidth",5);
    
I = insertShape(I,"Circle",[uv(:,1)' 10; ...
        uv(:,2)' 10; uv(:,3)' 10; uv(:,4)' 10], ...
        "Color",["magenta"],"LineWidth",5);
imshow(I);

if video_flag==1
    writeVideo(vid_image,I);
end

end
if video_flag==1
    close (vid_image);
end