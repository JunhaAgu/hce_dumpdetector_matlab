clc; clear all; close all;

% F = imread('tets.png'); 
load("test_image.mat");
% F = I_next;
F = imread('58.png');
imshow(F);
T = F(200:300, 455:555);
figure(); imshow(T);
% T = imread('id0.png');
% T = rgb2gray(T);
% T = imresize(T,0.25);

% figure(); subplot(121); imshow(F); title('gg');
% subplot(122); imshow(T); title('coin template');
% 
% [corrScore, boundingBox] = corrMatching(F,T);
% 
% figure(); imagesc(abs(corrScore)); axis image; axis off; colorbar;
% title('dfdf');
% bY = [boundingBox(1), boundingBox(1)+boundingBox(3), boundingBox(1) + boundingBox(3), boundingBox(1), boundingBox(1)];
% bX = [boundingBox(2), boundingBox(2), boundingBox(2)+boundingBox(4), boundingBox(2) + boundingBox(4), boundingBox(2)];
% figure(); imshow(F); line(bX, bY); title('dfd');


dst = cv.cornerHarris(F);
J = im2uint8(dst);
figure(); imagesc(J);

C = detectHarrisFeatures(F);
figure();
imshow(F);
hold on;
plot(C.selectStrongest(100));