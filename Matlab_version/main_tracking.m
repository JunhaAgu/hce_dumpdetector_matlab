clc; clear all; close all;
dbstop if error;

%% 1. P1, P2, P3, P4 - Í∞ÅÍ∞Å ?èå?†§Ï§òÏïº?ï®. 
%% point_num ?ûà?äî Î∂?Î∂? Î∞îÍøî?ïº?ï®
%% cam_num - 0?ù∏Ïß? 1?ù∏Ïß?

%%
addpath('C:\dev\mexopencv')
addpath('C:\dev\mexopencv\opencv_contrib')
% %% at first time you should build %%%
% mexopencv.make('opencv_path','C:\dev\build\install','opencv_contrib',true);

data_folder = 'D:\°⁄±‚æ˜∞˙¡¶\±ºªË±‚ 2¬˜≥‚µµ\2021_08_26_experiment';
% data_name = 'markers_swing1_exp120';
data_name = 'markers_boom1_exp120';
cam_num = 0;

f = fopen([data_folder,'\',data_name ,'\association.txt'],'r');
fgets(f);
n_line = 0;
while(1)
    line = fgets(f);
    if(line==-1)
        break;
    else
        n_line = n_line +1;
    end
end
n_data = n_line+1;

%%
point_num = 1;
standard_tag_id = 1; %from 0 to ??

if point_num == 1
%     image_point_range = 2:28;
%     image_point{1,1} = [735 216]; %from image #2
    image_point_range = 2:34;
    image_point{1,1} = [535 235]; %from image #2
elseif point_num == 2
%     image_point_range = 2:28;
%     image_point{1,1} = [423 197]; %from image #2
    image_point_range = 41:148;
    image_point{1,1} = [691 153]; %from image #2
elseif point_num == 3
%     image_point_range = 77:92;
%     image_point{1,1} = [694 760]; %from image #77
    image_point_range = 2:34;
    image_point{1,1} = [525 578]; %from image #2
elseif point_num == 4
%     image_point_range = 75:92;
%     image_point{1,1} = [358 763]; %from image #75
    image_point_range = 41:148;
    image_point{1,1} = [658 663]; %from image #41

end

%%
tagFamily = "tag36h11";
data = load(['intrinsic_cam', num2str(cam_num), '.mat']);
intrinsics = data.cameraParams;
tagSize = 0.233;

point(:,1) = image_point{1,1}';
n_pts = 1;
frame(1,1) = image_point_range(n_pts) ;

for ii=image_point_range
    I = imread([data_folder,'\',data_name, '\', 'cam', num2str(cam_num), '\', num2str(ii),'.png']);
    
    I = undistortImage(I,intrinsics,"OutputView","same");
    [id,loc,pose] = readAprilTag(I,"tag36h11",intrinsics,tagSize);
    
    worldPoints = [0 0 0; tagSize/2 0 0; 0 tagSize/2 0; 0 0 tagSize/2];
    
    I_next = imread([data_folder,'\',data_name, '\', 'cam', num2str(cam_num), '\', num2str(ii+1),'.png']);
    I_next = undistortImage(I_next,intrinsics,"OutputView","same");
    
    dst0 = cv.buildPyramid(I);
    dst1 = cv.buildPyramid(I_next);

    nextPts = cv.calcOpticalFlowPyrLK(dst0{1}, dst1{1}, image_point,'WinSize',[31 31]);
    image_point = nextPts;
    
    n_pts= n_pts+1;
    point(:,n_pts) = nextPts{1,1}';
    frame(1,n_pts) = ii + 1 ;
    
    idx = find(id==standard_tag_id);
    T = zeros(4,4);
    T(4,4) =1;
    T(1:3,1:3) = pose(idx).Rotation';
    T(1:3,4) = pose(idx).Translation';
    
    T_k0{1,n_pts-1} = T;
    if ii == image_point_range(1,end)
        [id,loc,pose] = readAprilTag(I_next,"tag36h11",intrinsics,tagSize);
        idx = find(id==standard_tag_id);
        T = zeros(4,4);
        T(4,4) =1;
        T(1:3,1:3) = pose(idx).Rotation';
        T(1:3,4) = pose(idx).Translation';
        
        T_k0{1,n_pts} = T;
    end
    
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
%     figure(); imshow(I);
    figure(100); imshow(I_next); hold on; 
    plot(image_point{1,1}(1,1),image_point{1,1}(1,2),'*m');
    hold off;
    
end

P.frame = frame;
P.point = point;
P.T_k0 = T_k0;