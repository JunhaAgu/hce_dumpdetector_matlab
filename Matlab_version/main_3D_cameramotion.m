clc; clear all; close all;

data_path = 'D:\★기업과제\굴삭기 2차년도\Apriltag 인식2\mat_boom\';

load([data_path, 'pts_3D.mat']); %3d points
load([data_path, 'T_k1_all.mat']);

%% tracking ROI video

video_flag =1 ;

if video_flag==1
    framerate = 10;
    vid_image = VideoWriter('0826_boom_3D_sideview');
    vid_image.FrameRate = framerate;
    open(vid_image);
end

%%
figure(100);
plot3(0,0,0, 'ws'); hold on;
plot3(pts_3D(1,:), pts_3D(2,:), pts_3D(3,:),'g+', 'LineWidth',1, 'MarkerSize',10); hold on;
plot3(pts_3D(1,:), pts_3D(2,:), pts_3D(3,:),'gs', 'LineWidth',2, 'MarkerSize',10); hold on;
axis equal;
xlabel('x[m]'); ylabel('y[m]'); zlabel('z[m]');
title('Region of Interest' ,'Color', 'y');
grid on;
set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w');

line1_x = linspace(pts_3D(1,1),pts_3D(1,2),100);
line1_y = linspace(pts_3D(2,1),pts_3D(2,2),100);
line1_z = linspace(pts_3D(3,1),pts_3D(3,2),100);
plot3(line1_x, line1_y, line1_z, 'g--', 'LineWidth',2);

line2_x = linspace(pts_3D(1,2),pts_3D(1,4),100);
line2_y = linspace(pts_3D(2,2),pts_3D(2,4),100);
line2_z = linspace(pts_3D(3,2),pts_3D(3,4),100);
plot3(line2_x, line2_y, line2_z, 'g--', 'LineWidth',2);

line3_x = linspace(pts_3D(1,4),pts_3D(1,3),100);
line3_y = linspace(pts_3D(2,4),pts_3D(2,3),100);
line3_z = linspace(pts_3D(3,4),pts_3D(3,3),100);
plot3(line3_x, line3_y, line3_z, 'g--', 'LineWidth',2);

line4_x = linspace(pts_3D(1,3),pts_3D(1,1),100);
line4_y = linspace(pts_3D(2,3),pts_3D(2,1),100);
line4_z = linspace(pts_3D(3,3),pts_3D(3,1),100);
plot3(line4_x, line4_y, line4_z, 'g--', 'LineWidth',2);

xlim([-4 3]); ylim([-5 4]); zlim([-0.5 8]);
view(15,30);
% view(0,90);

for i=1: length(T_k1_all)
    if T_k1_all{1,i} == zeros(4,4)
        continue;
    else
        T_1k = T_k1_all{1,i}^-1;
        T_1k = [0 -1 0 0; -1 0 0 0; 0 0 -1 0; 0 0 0 1]*T_1k;
        R = T_1k(1:3,1:3);
        t = T_1k(1:3,4);
        pose = rigid3d(R',t');
        cam.Visible = 0;
        cam = plotCamera('AbsolutePose',pose,'Size',0.5,'Color',[0 1 1]);
        drawnow;
        pause(0.05);
        if video_flag==1
            curframe= getframe(figure(100));
            writeVideo(vid_image,curframe);
        end
    end
end
if video_flag==1
    close (vid_image);
end
