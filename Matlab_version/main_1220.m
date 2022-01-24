clc; clear all; close all;

visualization_flag = false;

%% video

video_flag = 0;

if video_flag==1
    framerate = 10;
    vid_image = VideoWriter('1229_data5');
    vid_image.FrameRate = framerate;
    open(vid_image);
end

%% at first time you should build %%
addpath('C:\dev\mexopencv')
addpath('C:\dev\mexopencv\opencv_contrib')
% mexopencv.make('opencv_path','C:\dev\build\install','opencv_contrib',true);

% data_folder = 'D:\★기업과제\굴삭기 2차년도\2021_08_26_experiment';
% data_date = '_0826';
% data_name = 'markers_swing1_exp120';
% data_name = 'markers_boom1_exp120';

% data_folder = 'D:\★기업과제\굴삭기 2차년도\2021_08_11_experiment';
% data_date = '_0811';
% data_name = '\20210811_HCE_exp\hce_data\2021-08-11_17_10_27';
% data_name = '\20210811_HCE_exp\hce_data\marker_detection3_vertical';

data_folder = 'D:\★기업과제\굴삭기 2차년도\2021_12_29_junha';
data_date = '_2022_0113'; %'_1229';
data_name = '\data5';
% data1: id1에서 traking 두번
% data2: tracking 없음
% data3: tracking 없음
% data4: id3에서 traking 한번 <-디버깅 필요
% data5: tracking 없음
cam_num = 0;

f = fopen([data_folder,'\',data_name ,'\association.txt'],'r');
fgets(f);
n_line = 0;
valid_data_num = zeros(1,10000);
while(1)
    line = fgets(f);
    if(line==-1)
        break;
    else
        tmp0 = strfind(line,'cam0/');
        tmp1 = strfind(line,'.png');
        img_num = line(tmp0+5:tmp1(1)-1);
        valid_data_num(n_line+1) =  str2double(img_num);
        n_line = n_line +1;
    end
end
a = find(valid_data_num==0);
valid_data_num = valid_data_num(1,1:a(1)-1);
n_data = length(valid_data_num);

tagFamily = "tag36h11";
data = load(['intrinsic_cam', num2str(cam_num), data_date ,'.mat']);
intrinsics = data.cameraParams;
tagSize = 0.233;

used_tag = [0 1 2 3];
n_tag = length(used_tag);
n_pts_per_tag = 4;

group_num = [1 1 1 1];
group_change_flag = false(1,4);

tag = struct('frame_group',[],'April_frame',[],'frame',[],...
    'P0',[],'P0_3D',[],'P1',[],'P1_3D',[],'P2',[],'P2_3D',[],'P3',[],'P3_3D',[],'T_cp',[],'T_April',[]);
Ap = cell(1,n_pts_per_tag);
for i=1:n_pts_per_tag
    Ap{i} = tag;
end

valid_data_num_idx = and(valid_data_num>0, valid_data_num<inf);

for ii = valid_data_num(valid_data_num_idx)
    if ii==42
        disp('haha');
    end

    I = imread([data_folder,'\',data_name, '\', 'cam', num2str(cam_num), '\', num2str(ii),'.png']);
    
    I = undistortImage(I,intrinsics,"OutputView","same");
    [id,loc,pose] = readAprilTag(I,"tag36h11",intrinsics,tagSize);

%     worldPoints = [0 0 0; tagSize/2 0 0; 0 tagSize/2 0; 0 0 tagSize/2];
%     for i = 1:length(pose)
%         % Get image coordinates for axes.
%         imagePoints = worldToImage(intrinsics,pose(i).Rotation, ...
%             pose(i).Translation,worldPoints);
% 
%         % Draw colored axes.
%         I = insertShape(I,"Line",[imagePoints(1,:) imagePoints(2,:); ...
%             imagePoints(1,:) imagePoints(3,:); imagePoints(1,:) imagePoints(4,:)], ...
%             "Color",["red","green","blue"],"LineWidth",7);
% 
%         I = insertText(I,loc(1,:,i),id(i),"BoxOpacity",1,"FontSize",1);
%     end
%     imshow(I)
    
    imshow_flag = false;
    detected_flag_first = false(1,n_tag);
    cnt_id = 1;
    for jj = used_tag
        k = find(used_tag==jj);
        if isempty(find(id==jj))

            if ~isempty(Ap{k}.P0) && ~isempty(Ap{k}.P1) && ~isempty(Ap{k}.P2) && ~isempty(Ap{k}.P3) && group_change_flag(k) ==false 

                dst0 = cv.buildPyramid(I_pre);
                dst1 = cv.buildPyramid(I);

                image_point{1,1} = Ap{k}.P0(:,end)';
                image_point{1,2} = Ap{k}.P1(:,end)';
                image_point{1,3} = Ap{k}.P2(:,end)';
                image_point{1,4} = Ap{k}.P3(:,end)';

                [nextPts,status,error] = cv.calcOpticalFlowPyrLK(dst0{1}, dst1{1}, image_point, 'WinSize',[31 31]);
                disp(error');
                image_point2 = nextPts;

                if sum(error)>50 || status(1)*status(2)*status(3)*status(4)==0
                    disp(">>>>> KLT tracker is failed");
                    group_num(k) = group_num(k) +1;
                    group_change_flag(k) = true;
                    continue;
                end

                tag_point_name0 = ['p_{',num2str(jj),'0}']; tag_point_name1 = ['p_{',num2str(jj),'1}'];
                tag_point_name2 = ['p_{',num2str(jj),'2}']; tag_point_name3 = ['p_{',num2str(jj),'3}'];

                if visualization_flag == true

                    figure(1);
                    if imshow_flag == false
                        imshow(I);
                    end
                    hold on;
                    plot(image_point{1}(1,1),image_point{1}(1,2),'*b');
                    plot(image_point{2}(1,1),image_point{2}(1,2),'*b');
                    plot(image_point{3}(1,1),image_point{3}(1,2),'*b');
                    plot(image_point{4}(1,1),image_point{4}(1,2),'*b');

                    plot(image_point2{1}(1,1),image_point2{1}(1,2),'sr'); plot(image_point2{1}(1,1),image_point2{1}(1,2),'+r');
                    text(image_point2{1}(1,1),image_point2{1}(1,2),tag_point_name0,'Color','green','FontSize',10,'FontWeight','bold',...
                        'Position',[image_point2{1}(1,1)-15,image_point2{1}(1,2)-20]);
                    plot(image_point2{2}(1,1),image_point2{2}(1,2),'sr'); plot(image_point2{2}(1,1),image_point2{2}(1,2),'+r');
                    text(image_point2{2}(1,1),image_point2{2}(1,2),tag_point_name1,'Color','green','FontSize',10,'FontWeight','bold',...
                        'Position',[image_point2{2}(1,1)-15,image_point2{2}(1,2)+15]);
                    plot(image_point2{3}(1,1),image_point2{3}(1,2),'sr'); plot(image_point2{3}(1,1),image_point2{3}(1,2),'+r');
                    text(image_point2{3}(1,1),image_point2{3}(1,2),tag_point_name2,'Color','green','FontSize',10,'FontWeight','bold',...
                        'Position',[image_point2{3}(1,1)-15,image_point2{3}(1,2)+15]);
                    plot(image_point2{4}(1,1),image_point2{4}(1,2),'sr'); plot(image_point2{4}(1,1),image_point2{4}(1,2),'+r');
                    text(image_point2{4}(1,1),image_point2{4}(1,2),tag_point_name3,'Color','green','FontSize',10,'FontWeight','bold',...
                        'Position',[image_point2{4}(1,1)-15,image_point2{4}(1,2)-20]);

                end

                imshow_flag = true;
                
                Ap{k}.frame = [Ap{k}.frame, ii];
                Ap{k}.frame_group = [Ap{k}.frame_group, group_num(k)];
                Ap{k}.P0 = [Ap{k}.P0, [image_point2{1}(1,1);image_point2{1}(1,2)]];
                Ap{k}.P1 = [Ap{k}.P1, [image_point2{2}(1,1);image_point2{2}(1,2)]];
                Ap{k}.P2 = [Ap{k}.P2, [image_point2{3}(1,1);image_point2{3}(1,2)]];
                Ap{k}.P3 = [Ap{k}.P3, [image_point2{4}(1,1);image_point2{4}(1,2)]];
                
                objectPoints{1,1} = Ap{k}.P0_3D{end}';
                objectPoints{1,2} = Ap{k}.P1_3D{end}';
                objectPoints{1,3} = Ap{k}.P2_3D{end}';
                objectPoints{1,4} = Ap{k}.P3_3D{end}';
                imagePoints = image_point2(1:4);

                [rvecs, tvecs, solutions] = cv.solvePnP( objectPoints(1,1:4), imagePoints, transpose(intrinsics.IntrinsicMatrix) );
%                 for q = 1:solutions
%                     rot = cv.Rodrigues(rvecs{q});
%                     T_p3p{q} = [rot, tvecs{q} ; 0 0 0 1];
%                     for qq=1:4
%                         A = [transpose(intrinsics.IntrinsicMatrix), [0;0;0]] * (T_p3p{q}) * [objectPoints{qq}' ; 1];
%                         B = A./A(end);
%                         pt_p3p_proj(:,qq) = B(1:2,1);
%                     end
%                     for qq=1:4
%                         reproj_err(:,qq) = norm(pt_p3p_proj(:,qq) - image_point2{qq}');
%                     end
%                     repro_err_sum(1,q) = sum(reproj_err);
%                 end
%                 [p3p_err, idx_p3p_sol] = min(repro_err_sum);
%                 T_est = T_p3p{idx_p3p_sol};
                if solutions == 0
                    disp('PnP algorithm fails');
                    return;
                else
                    rot = cv.Rodrigues(rvecs);
                    T_p3p = [rot, tvecs; 0 0 0 1];
                    T_est = T_p3p;

                    for qq=1:4
                        A = [transpose(intrinsics.IntrinsicMatrix), [0;0;0]] * (T_p3p) * [objectPoints{qq}' ; 1];
                        B = A./A(end);
                        pt_p3p_proj(:,qq) = B(1:2,1);
                    end
                end
                
                T = (T_est) * Ap{k}.T_cp{end};
                Ap{k}.T_cp{length(Ap{k}.frame)} = T;

                Ap{k}.P0_3D{length(Ap{k}.frame)} = T(1:3,1:3)*p00 + T(1:3,4);
                Ap{k}.P1_3D{length(Ap{k}.frame)} = T(1:3,1:3)*p01 + T(1:3,4);
                Ap{k}.P2_3D{length(Ap{k}.frame)} = T(1:3,1:3)*p02 + T(1:3,4);
                Ap{k}.P3_3D{length(Ap{k}.frame)} = T(1:3,1:3)*p03 + T(1:3,4);
            end
 
            if detected_flag_first(1,k) == false
                continue;
            end
            
        else
            group_change_flag(k) = false;
            detected_flag_first(1,k) = true;
            Ap{k}.April_frame = [Ap{k}.April_frame, ii];
            Ap{k}.frame = [Ap{k}.frame, ii];
            Ap{k}.frame_group = [Ap{k}.frame_group, group_num(k)];
            Ap{k}.P0 = [Ap{k}.P0, [loc(1,1,cnt_id);loc(1,2,cnt_id)]];
            Ap{k}.P1 = [Ap{k}.P1, [loc(2,1,cnt_id);loc(2,2,cnt_id)]];
            Ap{k}.P2 = [Ap{k}.P2, [loc(3,1,cnt_id);loc(3,2,cnt_id)]];
            Ap{k}.P3 = [Ap{k}.P3, [loc(4,1,cnt_id);loc(4,2,cnt_id)]];

            
            tag_point_name0 = ['p_{',num2str(jj),'0}'];
            tag_point_name1 = ['p_{',num2str(jj),'1}'];
            tag_point_name2 = ['p_{',num2str(jj),'2}'];
            tag_point_name3 = ['p_{',num2str(jj),'3}'];

            if visualization_flag==true

                figure(1);
                if imshow_flag == false
                    imshow(I);
                end
                hold on;
                plot(Ap{k}.P0(1,end),Ap{k}.P0(2,end),'sb'); plot(Ap{k}.P0(1,end),Ap{k}.P0(2,end),'+b');
                text(Ap{k}.P0(1,end),Ap{k}.P0(2,end),tag_point_name0,'Color','green','FontSize',10,'FontWeight','bold',...
                    'Position',[Ap{k}.P0(1,end)-15,Ap{k}.P0(2,end)-20]);

                plot(Ap{k}.P1(1,end),Ap{k}.P1(2,end),'sb'); plot(Ap{k}.P1(1,end),Ap{k}.P1(2,end),'+b');
                text(Ap{k}.P1(1,end),Ap{k}.P1(2,end),tag_point_name1,'Color','green','FontSize',10,'FontWeight','bold',...
                    'Position',[Ap{k}.P1(1,end)-15,Ap{k}.P1(2,end)+15]);

                plot(Ap{k}.P2(1,end),Ap{k}.P2(2,end),'sb'); plot(Ap{k}.P2(1,end),Ap{k}.P2(2,end),'+b');
                text(Ap{k}.P2(1,end),Ap{k}.P2(2,end),tag_point_name2,'Color','green','FontSize',10,'FontWeight','bold',...
                    'Position',[Ap{k}.P2(1,end)-15,Ap{k}.P2(2,end)+15]);

                plot(Ap{k}.P3(1,end),Ap{k}.P3(2,end),'sb'); plot(Ap{k}.P3(1,end),Ap{k}.P3(2,end),'+b');
                text(Ap{k}.P3(1,end),Ap{k}.P3(2,end),tag_point_name3,'Color','green','FontSize',10,'FontWeight','bold',...
                    'Position',[Ap{k}.P3(1,end)-15,Ap{k}.P3(2,end)-20]);

            end

            imshow_flag = true;

            T = zeros(4,4);
            T(4,4) =1;
            T(1:3,1:3) = pose(cnt_id).Rotation';
            T(1:3,4) = pose(cnt_id).Translation';

            Ap{k}.T_cp{length(Ap{k}.frame)} = T;
            p00 = [-tagSize/2 ; tagSize/2 ; 0];
            p01 = [tagSize/2 ; tagSize/2 ; 0];
            p02 = [tagSize/2 ; -tagSize/2 ; 0];
            p03 = [-tagSize/2 ; -tagSize/2 ; 0];
            
            Ap{k}.P0_3D{length(Ap{k}.frame)} = T(1:3,1:3)*p00 + T(1:3,4);
            Ap{k}.P1_3D{length(Ap{k}.frame)} = T(1:3,1:3)*p01 + T(1:3,4);
            Ap{k}.P2_3D{length(Ap{k}.frame)} = T(1:3,1:3)*p02 + T(1:3,4);
            Ap{k}.P3_3D{length(Ap{k}.frame)} = T(1:3,1:3)*p03 + T(1:3,4);
            Ap{k}.T_April{length(Ap{k}.April_frame)} = T;
        end
        cnt_id = cnt_id +1;
    end

    if imshow_flag==true && video_flag==1
        cur_frame = getframe(figure(1));
        writeVideo(vid_image,cur_frame);
    end

    I_pre = I;

end

if video_flag==1
    close (vid_image);
end