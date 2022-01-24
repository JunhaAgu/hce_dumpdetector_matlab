clc; clear all; close all;

%% frame 두개로 검증해본 것. (아까워서 남기기)

%%
data_folder = 'D:\★기업과제\굴삭기 2차년도\2021_08_11_experiment\20210811_HCE_exp\hce_data';
data_name = '2021-08-11_17_10_27';
cam_num = 1;

data_num = 7;

I = imread([data_folder,'\',data_name, '\', 'cam', num2str(cam_num), '\', num2str(data_num),'.png']);
tagFamily = "tag36h11";

figure(); imshow(I);
data = load(['intrinsic_cam', num2str(cam_num), '.mat']);
intrinsics = data.cameraParams;

fu = intrinsics.IntrinsicMatrix(1,1);
fv = intrinsics.IntrinsicMatrix(2,2);
cu = intrinsics.IntrinsicMatrix(3,1);
cv = intrinsics.IntrinsicMatrix(3,2);

% (361, 1)      (1021, 131)
% (362, 680)    (1015, 550)

% (372, 70)     (1024, 203)
% (351, 750)    (1011, 621)

u1_1 = 366; v1_1 = 41;
% [u1_1, v1_1] = undistort(u1_1, v1_1, intrinsics.IntrinsicMatrix', intrinsics.RadialDistortion);
u2_1 = 1025; v2_1 = 170;
% [u2_1, v2_1] = undistort(u2_1, v2_1, intrinsics.IntrinsicMatrix', intrinsics.RadialDistortion);
u3_1 = 365; v3_1 = 644;
% [u3_1, v3_1] = undistort(u3_1, v3_1, intrinsics.IntrinsicMatrix', intrinsics.RadialDistortion);
u4_1 = 1024; v4_1 =512;
% [u4_1, v4_1] = undistort(u4_1, v4_1, intrinsics.IntrinsicMatrix', intrinsics.RadialDistortion);

u1_2 = 381; v1_2 = 145;
% [u1_2, v1_2] = undistort(u1_2, v1_2, intrinsics.IntrinsicMatrix', intrinsics.RadialDistortion);
u2_2 = 1030; v2_2 = 278;
% [u2_2, v2_2] = undistort(u2_2, v2_2, intrinsics.IntrinsicMatrix', intrinsics.RadialDistortion);
u3_2 = 347; v3_2 = 748;
% [u3_2, v3_2] = undistort(u3_2, v3_2, intrinsics.IntrinsicMatrix', intrinsics.RadialDistortion);
u4_2 = 1018; v4_2 = 621;
% [u4_2, v4_2] = undistort(u4_2, v4_2, intrinsics.IntrinsicMatrix', intrinsics.RadialDistortion);

A1 = (u1_1 - cu) / fu;
B1 = (v1_1 - cv) / fv;
C1 = (u1_2 - cu) / fu;
D1 = (v1_2 - cv) / fv;

A2 = (u2_1 - cu) / fu;
B2 = (v2_1 - cv) / fv;
C2 = (u2_2 - cu) / fu;
D2 = (v2_2 - cv) / fv;

A3 = (u3_1 - cu) / fu;
B3 = (v3_1 - cv) / fv;
C3 = (u3_2 - cu) / fu;
D3 = (v3_2 - cv) / fv;

A4 = (u4_1 - cu) / fu;
B4 = (v4_1 - cv) / fv;
C4 = (u4_2 - cu) / fu;
D4 = (v4_2 - cv) / fv;

load('T_10.mat');
T_10 = zeros(4,4);
T_10(4,4) = 1;
T_10(1:3,1:3) = T(1:3,1:3)';
T_10(1:3,4) = T(4,1:3)';

load('T_20.mat');
T_20 = zeros(4,4);
T_20(4,4) = 1;
T_20(1:3,1:3) = T(1:3,1:3)';
T_20(1:3,4) = T(4,1:3)';

T_21 = T_20*T_10^-1;

%% P1
z1_1 = (D1*T_21(3,4) - T_21(2,4)) /( A1*(T_21(2,1)-D1*T_21(3,1)) + B1*(T_21(2,2)-D1*T_21(3,2)) + (T_21(2,3)-D1*T_21(3,3)) ) ;
x1_1 = A1* z1_1;
y1_1 = B1* z1_1;

P1_0 = (T_10)^-1*[x1_1 y1_1 z1_1 1]';

%% P2
z2_1 = (D2*T_21(3,4) - T_21(2,4)) /( A2*(T_21(2,1)-D2*T_21(3,1)) + B2*(T_21(2,2)-D2*T_21(3,2)) + (T_21(2,3)-D2*T_21(3,3)) ) ;
x2_1 = A2* z2_1;
y2_1 = B2* z2_1;

P2_0 = (T_10)^-1*[x2_1 y2_1 z2_1 1]';

%% P3
z3_1 = (D3*T_21(3,4) - T_21(2,4)) /( A3*(T_21(2,1)-D3*T_21(3,1)) + B3*(T_21(2,2)-D3*T_21(3,2)) + (T_21(2,3)-D3*T_21(3,3)) ) ;
x3_1 = A3* z3_1;
y3_1 = B3* z3_1;

P3_0 = (T_10)^-1*[x3_1 y3_1 z3_1 1]';

%% P4
z4_1 = (D4*T_21(3,4) - T_21(2,4)) /( A4*(T_21(2,1)-D4*T_21(3,1)) + B4*(T_21(2,2)-D4*T_21(3,2)) + (T_21(2,3)-D4*T_21(3,3)) ) ;
x4_1 = A4* z4_1;
y4_1 = B4* z4_1;

P4_0 = (T_10)^-1*[x4_1 y4_1 z4_1 1]';

% z1_1_ = (D*T_21(3,4) - T_21(2,4)) /( A*(T_21(2,1)-D*T_21(3,1)) + B*(T_21(2,2)-D*T_21(3,2)) + (T_21(2,3)-D*T_21(3,3)) ) ;
% x1_1_ = A* z1_1_;
% y1_1_ = B* z1_1_;
% 
% P1_0_ = (T_10)^-1*[x1_1_ y1_1_ z1_1_ 1]';

P = [P1_0(1:3,1), P2_0(1:3,1), P3_0(1:3,1), P4_0(1:3,1)]; 

figure();
plot3(0,0,0, 'bs'); hold on;
plot3(P1_0(1,1), P1_0(2,1), P1_0(3,1),'r*'); hold on;
plot3(P2_0(1,1), P2_0(2,1), P2_0(3,1),'r*'); hold on;
plot3(P3_0(1,1), P3_0(2,1), P3_0(3,1),'r*'); hold on;
plot3(P4_0(1,1), P4_0(2,1), P4_0(3,1),'r*');
grid on; axis equal;