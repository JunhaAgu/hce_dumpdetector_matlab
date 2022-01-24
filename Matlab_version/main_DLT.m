clc; clear; close all;

%% 2. 최종적으로 진행 DLT

data_path = 'D:\★기업과제\굴삭기 2차년도\Apriltag 인식2\mat_boom\';

load([data_path, 'P1.mat']);P1 = P;
load([data_path, 'P2.mat']);P2 = P;
load([data_path, 'P3.mat']);P3 = P;
load([data_path, 'P4.mat']);P4 = P;
load('intrinsic_cam0.mat');

n_frame = length(P1.frame);

K = cameraParams.IntrinsicMatrix';
A = zeros(n_frame,4);
for i=1:n_frame
    M = K*P1.T_k0{1, i}(1:3,:);
    A(i*2-1,:) = P1.point(1,i)*M(3,:)-M(1,:);
    A(i*2,:) = P1.point(2,i)*M(3,:)-M(2,:);
end
[~,~,V] = svd(A);
X_temp = V(:,4);
pts_3D(:,1) = X_temp(1:3)./X_temp(4);

n_frame = length(P2.frame);
K = cameraParams.IntrinsicMatrix';
A = zeros(n_frame,4);
for i=1:n_frame
    M = K*P2.T_k0{1, i}(1:3,:);
    A(i*2-1,:) = P2.point(1,i)*M(3,:)-M(1,:);
    A(i*2,:) = P2.point(2,i)*M(3,:)-M(2,:);
end
[~,~,V] = svd(A);
X_temp = V(:,4);
pts_3D(:,2) = X_temp(1:3)./X_temp(4);

n_frame = length(P3.frame);
K = cameraParams.IntrinsicMatrix';
A = zeros(n_frame,4);
for i=1:n_frame
    M = K*P3.T_k0{1, i}(1:3,:);
    A(i*2-1,:) = P3.point(1,i)*M(3,:)-M(1,:);
    A(i*2,:) = P3.point(2,i)*M(3,:)-M(2,:);
end
[~,~,V] = svd(A);
X_temp = V(:,4);
pts_3D(:,3) = X_temp(1:3)./X_temp(4);

n_frame = length(P4.frame);
K = cameraParams.IntrinsicMatrix';
A = zeros(n_frame,4);
for i=1:n_frame
    M = K*P4.T_k0{1, i}(1:3,:);
    A(i*2-1,:) = P4.point(1,i)*M(3,:)-M(1,:);
    A(i*2,:) = P4.point(2,i)*M(3,:)-M(2,:);
end
[~,~,V] = svd(A);
X_temp = V(:,4);
pts_3D(:,4) = X_temp(1:3)./X_temp(4);

% load('tag0_3D.mat'); %tag0
% load('tag1_3D.mat'); %tag1
% load('tag2_3D.mat'); %tag2
% load('tag4_3D.mat'); %tag3

figure();
plot3(0,0,0, 'ws'); hold on;
plot3(pts_3D(1,:), pts_3D(2,:), pts_3D(3,:),'g+', 'LineWidth',1, 'MarkerSize',10); hold on;
plot3(pts_3D(1,:), pts_3D(2,:), pts_3D(3,:),'gs', 'LineWidth',2, 'MarkerSize',10); hold on;
% plot3(tag0(1,1),tag0(1,2),tag0(1,3) ,'ys'); plot3(tag1(1,1),tag1(1,2),tag1(1,3) ,'ys');
% plot3(tag2(1,1),tag2(1,2),tag2(1,3) ,'ys'); plot3(tag4(1,1),tag4(1,2),tag4(1,3) ,'ys');

axis equal;
xlabel('X[m]'); ylabel('Y[m]'); zlabel('Z[m]');
title('Region of Interest' ,'Color', 'y');
grid on;
set(gcf,'Color','k'); set(gca,'Color','k'); set(gca,'xcolor','w'); set(gca,'ycolor','w'); set(gca,'zcolor','w');

% % xyz=[0 0 0 ; tag0 ; tag1];
% p_A = det([1 tag0(2) tag0(3) ; 1 tag1(2) tag1(3) ; 1 tag4(2) tag4(3)]);
% p_B = det([tag0(1) 1 tag0(3) ; tag1(1) 1 tag1(3) ; tag4(1) 1 tag4(3)]);
% p_C = det([tag0(1) tag0(2) 1 ; tag1(1) tag1(2) 1 ; tag4(1) tag4(2) 1]);
% p_D = det([tag0(1) tag0(2) tag0(3) ; tag1(1) tag1(2) tag1(3) ; tag4(1) tag4(2) tag4(3)]);
% p_N = [p_A, p_B, p_C, p_D];
% x = -max(abs([tag0(1),tag1(1),tag2(1),tag4(1)]))*1.2:0.01:max(abs([tag0(1),tag1(1),tag2(1),tag4(1)]))*1.2;
% y = -max(abs([tag0(2),tag1(2),tag2(2),tag4(2)]))*1.2:0.01:max(abs([tag0(2),tag1(2),tag2(2),tag4(2)]))*1.2;
% [X,Y] = meshgrid(x,y);
% Z = (X.*p_A + Y.*p_B + p_D)./p_C;
% s= surf(X,Y,Z);
% s.EdgeColor = 'none';
% s.FaceColor = 'white';
% s.FaceAlpha = 0.5;

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

xlim([-2 4]); ylim([-4 2]); zlim([-0.5 0.5]);