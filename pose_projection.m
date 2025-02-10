source "./utils.m"

% [K,T,z_near,z_far,width,height] = extractCamParams();

global K T z_near z_far width height;

# Xr robot pose homogeneous matrix (4x4 right now, actually SE(2))
# Xl landmark pose in world
# Z landmark measurement in image frame

function idx = pindex(p_index, pose_dim, landmark_dim, num_poses, num_landmarks)
    if (p_index > num_poses)
        idx = -1;
        return;
    end
    idx = 1 + (p_index - 1) * pose_dim;
endfunction

function idx = lindex(l_index, pose_dim, landmark_dim, num_poses, num_landmarks)
    if(l_index > num_landmarks)
        idx = -1;
        return;
    end
    idx = 1 + (num_poses) * pose_dim + (l_index - 1) * landmark_dim;
endfunction

function S = skew(t)
    S = [0    -t(3)  t(2);
         t(3)  0    -t(1);
        -t(2)  t(1)  0];
endfunction

function [point_in_image, p_im, im_z] = projectPoint(Xr, Xl)
    global K T z_near z_far width height;
    
    point_in_image = [-1;-1];
    p_im = [-1;-1];
    im_z = -1;

    world_in_cam = inv(Xr * T);
    point_in_cam = world_in_cam(1:3, 1:3)*Xl + world_in_cam(1:3,4);
    if (point_in_cam < z_near || point_in_cam > z_far)
        return;
    end

    p_im = K * point_in_cam;
    im_z = 1./p_im(3);
    p_im *= im_z;
    
    if (p_im(1) < 0 ||
        p_im(1) > width ||
        p_im(2) < 0 ||
        p_im(2) > height)
        return;
    end
    point_in_image = p_im(1:2);
endfunction

function [valid, e, Jr, Jl] = projectionErrorAndJacobian(Xr, Xl, Z)
    
    global K T;
    
    valid = false;
    e = [0; 0];
    
    # considering a robot SE(3) glued to the 2D plane, make it 2D (SE(2)) later 
    Jr = zeros(2, 6);
    Jl = zeros(2, 3);

    
    cam_in_world = Xr*T;
    # 3rd row/col always 0 0 1 as rotation around z axis
    R_world_in_cam = cam_in_world(1:3, 1:3)';
    
    J_wr = zeros(3, 6);
    J_wr(1:3,1:3) = -R_world_in_cam;
    J_wr(1:3, 4:6) = R_world_in_cam * skew(Xl);

    J_wl = R_world_in_cam;

    [Z_hat, p_im, im_z] = projectPoint(Xr, Xl);
    if (Z_hat < 0)
        return
    end
    e = Z_hat - Z;

    im_x = p_im(1);
    im_y = p_im(2);
    im_z2 = im_z * im_z;

    Jp = [1/im_z    0       -im_x/im_z2; 
          0         1/im_z  -im_y/im_z2];

    Jr = Jp * K * J_wr;
    Jl = Jp * K * J_wl;
    valid = true;
endfunction

function [H, b, chi_tot, inliers] = linearizeProjections(XR, XL, Zl, kernel_threshold)

    pose_dim = 6;
    landmark_dim = 3;
    num_poses = length(XR);
    num_landmarks = length(XL);
    system_size = pose_dim * num_poses + landmark_dim * num_landmarks;

    H = zeros(system_size, system_size);
    b = zeros(system_size, 1);
    chi_tot = 0;
    inliers = 0;
    assert(length(XR) == length(Zl));
    for i = 1 : length(Zl)
        for j = keys(Zl(i))

            Omega = eye(2);
            Omega *= 1e-1;

            # landmark ids are numbered starting from 0
            k = j{1};

            Xr = XR(:,:,i);
            Xl = XL(:,k+1);
            Z = Zl(i)(k);
            [valid, e, Jr, Jl] = projectionErrorAndJacobian(Xr, Xl, Z);
            if (!valid)
                continue;
            end

            chi = e' * Omega * e;
            if chi > kernel_threshold
                Omega *= sqrt(kernel_threshold / chi);
                chi = kernel_threshold;
            else
                inliers++;
            end

            chi_tot += chi;

            H_pose_pose = Jr' * Omega * Jr;
            H_pose_proj = Jr' * Omega * Jl;
            H_proj_pose = Jl' * Omega * Jr;
            H_proj_proj = Jl' * Omega * Jl;
            b_pose = Jr' * e;
            b_proj = Jl' * e;

            #H and b indexes
            pose_idx = 1 + (i - 1) * pose_dim;
            landmark_idx = 1 + num_poses * pose_dim + k * landmark_dim;

            H(pose_idx : pose_idx + pose_dim - 1, pose_idx : pose_idx + pose_dim - 1) += H_pose_pose;

            H(pose_idx : pose_idx + pose_dim - 1, landmark_idx : landmark_idx + landmark_dim - 1) += H_pose_proj;

            H(landmark_idx : landmark_idx + landmark_dim - 1, pose_idx : pose_idx + pose_dim - 1) += H_proj_pose;

            H(landmark_idx : landmark_idx + landmark_dim - 1, landmark_idx : landmark_idx + landmark_dim - 1) += H_proj_proj;

            b(pose_idx : pose_idx + pose_dim - 1) = b_pose;

            b(landmark_idx : landmark_idx + landmark_dim - 1) = b_proj;
            
        end
    end
 
endfunction