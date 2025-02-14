source "./utils.m"


# Xr robot pose homogeneous matrix (4x4, representing SE(2) motion)
# Xl landmark pose in world
# Z landmark measurement in image frame

function [valid, e, Jr, Jl] = projectionErrorAndJacobian(Xr, Xl, Z)
    
    global K T z_near z_far width height;

    global pose_dim landmark_dim;
    
    valid = false;
    e = [0; 0];
    

    Jr = zeros(2, pose_dim);
    Jl = zeros(2, landmark_dim);

    if(Xl == [0;0;-1])
        return;
    endif
    
    cam_in_world = Xr*T;

    world_in_cam = inv(cam_in_world);

    iR = world_in_cam(1:3,1:3);
    p_wc = world_in_cam(1:3,4);
    point_in_cam = iR*Xl + p_wc;

    if(point_in_cam(3) < z_near || point_in_cam(3) > z_far)
        return;
    endif
    
    J_wr = zeros(3,6);
    J_wr(1:3,1:3) = -iR;
    J_wr(1:3, 4:6) = iR * skew(Xl);

    if( pose_dim == 3)
        % SE(2) so only calculating dx dy and dalphaz components
        J_wr = J_wr(:, [1,2,6]);
    endif

    J_wl = iR;

    p_cam = K * point_in_cam;
    iz = 1./p_cam(3);
    Z_hat = p_cam(1:2)*iz;

    if (Z_hat(1) < 0 || Z_hat(1) > width || Z_hat(2) < 0 || Z_hat(2) > height)
        return;
    endif

    im_x = p_cam(1);
    im_y = p_cam(2);
    im_z2 = iz * iz;

    Jp = [iz    0       -im_x*im_z2; 
          0     iz      -im_y*im_z2];

    e = Z_hat - Z;
    Jr = Jp * K * J_wr;
    Jl = Jp * K * J_wl;
    valid = true;

endfunction

function [H, b, chi_tot, inliers] = linearizeProjections(XR, XL, Zl, kernel_threshold)

    global pose_dim landmark_dim;
    num_poses = length(XR);
    num_landmarks = length(XL);
    system_size = pose_dim * num_poses + landmark_dim * num_landmarks;

    H = zeros(system_size, system_size);
    b = zeros(system_size, 1);

    chi_tot = 0;
    inliers = 0;
    outliers = 0;

    assert(length(XR) == length(Zl));
    for i = 1 : length(Zl)
        for k = keys(Zl(i))

            Omega = eye(2);

            # landmark ids are numbered starting from 0 with the id being used as key
            j = k{1}+1;

            Xr = XR(:,:,i);
            Xl = XL(:,j);
            
            % observation
            Z = Zl(i)(k{1});

            [valid, e, Jr, Jl] = projectionErrorAndJacobian(Xr, Xl, Z);

            if (!valid)
                continue;
            endif
            
            chi = e' * Omega * e;
            if chi > kernel_threshold
                Omega *= sqrt(kernel_threshold / chi);
                chi = kernel_threshold;
            else
                inliers++;
            endif

            chi_tot += chi;

            H_pose_pose = Jr' * Omega * Jr;
            H_pose_proj = Jr' * Omega * Jl;
            H_proj_pose = Jl' * Omega * Jr;
            H_proj_proj = Jl' * Omega * Jl;
            b_pose = Jr' * Omega * e;
            b_proj = Jl' * Omega * e;
           
            #H and b indexes
            pose_idx = pindex(i, num_poses, num_landmarks);
            landmark_idx = lindex(j, num_poses, num_landmarks);

            H(pose_idx : pose_idx + pose_dim - 1, pose_idx : pose_idx + pose_dim - 1) += H_pose_pose;

            H(pose_idx : pose_idx + pose_dim - 1, landmark_idx : landmark_idx + landmark_dim - 1) += H_pose_proj;

            H(landmark_idx : landmark_idx + landmark_dim - 1, pose_idx : pose_idx + pose_dim - 1) += H_proj_pose;

            H(landmark_idx : landmark_idx + landmark_dim - 1, landmark_idx : landmark_idx + landmark_dim - 1) += H_proj_proj;

            b(pose_idx : pose_idx + pose_dim - 1) += b_pose;

            b(landmark_idx : landmark_idx + landmark_dim - 1) += b_proj;

        endfor
    endfor
    
endfunction