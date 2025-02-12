1;


% 2D stuff
function T = v2t(p)
    T = eye(3);
    T(1:2,1:2) = get2DRotMat(p(3));
    T(1:2,3) = p(1:2);
endfunction

function p = t2v(T)
    p = zeros(3,1);
    p(1:2) = T(1:2,3);
    p(3) = atan2(T(2,1),T(1,1));
endfunction

function R = get2DRotMat(a)
    R = [cos(a) -sin(a);
         sin(a) cos(a)];
endfunction



% 3D
% robot in SE(3) flattened on the ground
% transforms a SE(2)->R3 point into a SE(3) homogeneous matrix
% z axis is left untouched
function T = v2t3D(p)
    T = eye(4);
    T(1:2,1:2) = get2DRotMat(p(3));
    T(1:2,4) = p(1:2);
endfunction

function p = t2v3D(T)
    p = zeros(3,1);
    p(1:2) = T(1:2,4);
    p(3) = atan2(T(2,1),T(1,1));
endfunction

function S = skew(t)
    S = [0    -t(3)  t(2);
         t(3)  0    -t(1);
        -t(2)  t(1)  0];
endfunction

function idx = pindex(p_index, pose_dim, landmark_dim, num_poses, num_landmarks)
    if (p_index > num_poses)
        idx = -1;
        return;
    endif
    idx = 1 + (p_index - 1) * pose_dim;
endfunction

function idx = lindex(l_index, pose_dim, landmark_dim, num_poses, num_landmarks)
    if(l_index > num_landmarks)
        idx = -1;
        return;
    endif
    idx = 1 + (num_poses) * pose_dim + (l_index - 1) * landmark_dim;
endfunction


function [XL, points_seen, points_triangulated] = triangulate(poses, measurements, num_landmarks)

    global landmark_dim;
    printf("Triangulating points...\n");

    A = buildLinearMatrix(poses, measurements, num_landmarks);

    XL = zeros(landmark_dim, num_landmarks);
    points_seen = 0;
    points_triangulated = 0;

    for i = 1:length(XL)
    
        % point has been seen at least once
        if(size(A{i},1)) > 0
            
            points_seen++;

            % at least 2 hits to triangulate, i.e. 4 equations
            if(size(A{i},1)) >= 4

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
    printf("done!\n%d landmarks have been seen, of which %d have been triangulated\n", points_seen, points_triangulated);
endfunction

function A = buildLinearMatrix(poses, measurements, num_landmarks)
    
    global T K;

    A = cell(1, num_landmarks);

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
    
    odo_meas = zeros(size(odometry)(1), size(odometry)(2));
    gt_odo_meas = zeros(size(gt_odometry)(1), size(gt_odometry)(2));
    odo_meas(:,1) = odometry(:,1);
    gt_odo_meas(:,1) = gt_odometry(:,1);
    assert(length(odo_meas) == length(gt_odo_meas));
    
    for i = 1 : length(odo_meas) - 1
        j = i + 1;
        odo_meas(:,j) = t2v(inv(v2t(odometry(:,i)))*v2t(odometry(:,j)));
        gt_odo_meas(:,j) = t2v(inv(v2t(gt_odometry(:,i)))*v2t(gt_odometry(:,j)));
    endfor

endfunction

