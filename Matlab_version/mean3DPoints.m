function points_trimmed = mean3DPoints(T_0k, data_base, idx_tag_valid, idx_tag_dom)
points_trimmed = [];
    for i= 1: length(data_base.db)
        id = data_base.db{i, 1}.id_imgs;
        
        pts = [];
        for j=1:length(data_base.db{i, 1}.pts_3D)
            frame_idx = find(idx_tag_valid==id(j));
            pts_tmp = T_0k{1, frame_idx}*[data_base.db{i, 1}.pts_3D{1,j} ; 1];
            pts = [pts, pts_tmp(1:3,1)];
        end
        pts_median = median(pts,2);
        figure(500); 
        plot3(pts(1,:),pts(2,:),pts(3,:),'r*'); axis equal; hold on;
        plot3(pts_median(1),pts_median(2),pts_median(3),'bs','MarkerSize',20);
        
        points_trimmed = [points_trimmed, pts_median];
    end
end