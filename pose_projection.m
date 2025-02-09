1;

% [K,T,z_near,z_far,width,height] = extractCamParams();

global K T z_near z_far width height;

# Xr robot pose homogeneous matrix (4x4 right now, actually SE(2))
# Xl landmark pose in world
# Z landmark measurement in image frame

function S = skew(t)
    S = [0    -t(3)  t(2);
         t(3)  0    -t(1);
        -t(2)  t(1)  0];
endfunction

function [point_in_image, p_im, im_z] = projectPoint(Xr, Xl)
    global K T z_near z_far width height;
    
    point_in_image = [-1;-1];

    world_in_cam = inv(Xr*T);
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

function [valid, e, Jr, Jl] = errorAndJacobian(Xr, Xl, Z)
    % INSERT FLAGS FOR WHEN POINT IS NOT VALID (I.E. OUTSIDE CAMERA BOUNDARIES OR NO PREDICTION)?
    global K T;
    
    valid = false;
    e = [0; 0];
    
    # considering a robot SE(3) glued to the 2D plane, make it 2D (SE(2)) later 
    Jr = zeros(3, 6);
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

function [H, b, chi_tot, inliers] = linearizeProjections(XR, XL, Zl, pose_dim, landmark_dim, kernel_threshold)


    system_size = pose_dim * num_poses + landmark_dim * num_landmarks;

    H = zeros(system_size, system_size);
    b = zeros(system_size, 1);
    chi_tot = 0;
    inliers = 0;
    assert(length(XR) == length(Zl));
    for i = 1 : length(XR)
        for j = 0 : length(XL)-1
            if (isKey(Zl(i),j))
                Xr = XR(:,:,i);
                Xl = XL(j);
                Z = Zl(i)(j);
                [valid, err, Jr, Jl] = errorAndJacobian(Xr, Xl, Z);
                if (!valid)
                    continue;
                end

                chi = err' * err;
                if chi > kernel_threshold
                    err *= sqrt(kernel_threshold / chi);
                    chi = kernel_threshold;
                else
                    inliers++;
                end

                chi_tot += chi;

                H_pose_pose = Jr' * Jr;
                H_pose_proj = Jr' * Jl;
                H_proj_proj = Jl' * Jl;
                b_pose = Jr' * err;
                b_proj = Jl' * err;

                #H and b indexes
            end
        end
    end

    
endfunction