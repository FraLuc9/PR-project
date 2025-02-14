1;


% only working with homogeneous SE(3) matrices representing SE(2) movements
% only in need of transformations from a SE(2) state to SE(3) matrix and vice versa 

function T = v2t(p)
    T = eye(4);
    T(1:2,1:2) = get2DRotMat(p(3));
    T(1:2,4) = p(1:2);
endfunction


function p = t2v(T)
    p = zeros(3,1);
    p(1:2) = T(1:2,4);
    p(3) = atan2(T(2,1),T(1,1));
endfunction


function R = get2DRotMat(a)
    R = [cos(a) -sin(a);
         sin(a) cos(a)];
endfunction


function S = skew(t)
    S = [0    -t(3)  t(2);
         t(3)  0    -t(1);
        -t(2)  t(1)  0];
endfunction


function idx = pindex(p_index, num_poses, num_landmarks)
    global pose_dim landmark_dim;

    if (p_index > num_poses)
        idx = -1;
        return;
    endif
    idx = 1 + (p_index - 1) * pose_dim;
endfunction


function idx = lindex(l_index, num_poses, num_landmarks)
    global pose_dim landmark_dim;

    if(l_index > num_landmarks)
        idx = -1;
        return;
    endif
    idx = 1 + (num_poses) * pose_dim + (l_index - 1) * landmark_dim;
endfunction


function [XL, points_seen, points_triangulated] = triangulate(poses, measurements, num_landmarks)

    global landmark_dim K T;
    printf("Triangulating points...\n");

    A = buildLinearMatrix(poses, measurements, num_landmarks);

    XL = zeros(landmark_dim, num_landmarks);
    points_seen = 0;
    points_triangulated = 0;

    for i = 1:length(XL)
    
        % point has been seen at least once
        if(size(A{i},1)) > 0
            
            points_seen++;

            % at least 4 hits to triangulate, i.e. 8 equations
            if(size(A{i},1)) >= 8

                points_triangulated++;
                [~,~,V] = svd(A{i});    
                point = V(:,end);
                XL(:,i) = (point/point(end))(1:3);
            else
                XL(:,i) = [0;0;-1];
            end
        else
            XL(:,i) = [0;0;-1];
        endif
    endfor

    printf("done!\n%d landmarks have been seen at least once, of which %d have been triangulated\n", points_seen, points_triangulated);

endfunction




function A = buildLinearMatrix(poses, measurements, num_landmarks)
    
    global T K width height;

    A = cell(1, num_landmarks);

    % unused
    % margin = min(width, height) * 0.15;

    for i = 1:length(measurements)
        for k = keys(measurements(i))
            j = k{1}+1;
            
                Cam_in_world = poses(:,:,i)*T;
                World_in_image = K*inv(Cam_in_world)(1:3,:);
                p_j = measurements(i)(k{1});
                x = p_j(1);
                y = p_j(2);
                P1 = World_in_image(1,:);
                P2 = World_in_image(2,:);
                P3 = World_in_image(3,:);
                A{j} = [A{j}; x*P3-P1; y*P3-P2];

        endfor
    endfor

endfunction


function [odo_meas, gt_odo_meas] = calculateOdometryMeasurements(odometry, gt_odometry)
    
    odo_meas = zeros(size(odometry)(1), size(odometry)(2) - 1);
    gt_odo_meas = zeros(size(gt_odometry)(1), size(gt_odometry)(2) - 1);
    assert(length(odo_meas) == length(gt_odo_meas));
    
    for i = 1 : length(odo_meas)
        odo_meas(:,i) = t2v(inv(v2t(odometry(:,i)))*v2t(odometry(:,i+1)));
        gt_odo_meas(:,i) = t2v(inv(v2t(gt_odometry(:,i)))*v2t(gt_odometry(:,i+1)));
    endfor

endfunction


function [tot_err_rotation, tot_err_translation] = poseMSE(XR, XR_gt)
    tot_err_rotation = 0;
    tot_err_translation = 0;

    for i = 1 : length(XR) - 1

        T_0 = XR(:,:,i);
        T_1 = XR(:,:,i+1);
        rel_T = inv(T_0)*T_1;

        GT_0 = XR_gt(:,:,i);
        GT_1 = XR_gt(:,:,i+1);
        rel_GT = inv(GT_0)*GT_1;

        err_T = inv(rel_T)*rel_GT;

        err_rotation = sqrt(atan2(err_T(2,1),err_T(1,1))^2);
        err_translation = sqrt(err_T(1,4)^2 + err_T(2,4)^2);


        tot_err_rotation += err_rotation;
        tot_err_translation += err_translation;
    endfor

    tot_err_rotation *= 1./(length(XR)-1);
    tot_err_translation *= 1./(length(XR)-1);
  
endfunction


function tot_err = mapMSE(XL, XL_gt)
    tot_err = 0;

    for i = 1 : length(XL)
        
        % do not consider unseen landmarks
        if XL(:,i) == [0;0;-1]
            continue;
        endif

        err = sqrt((XL_gt(1,i) - XL(1,i))^2 + (XL_gt(2,i) - XL(2,i))^2 + (XL_gt(3,i) - XL(3,i))^2);
        tot_err += err;
    endfor;

    tot_err *= 1./length(XL);

endfunction


function err = mse(XL, XL_gt)
    err = sqrt((XL_gt(1)-XL(1))^2 + (XL_gt(2)-XL(2))^2 + (XL_gt(3)- XL(3))^2);    
endfunction
