source "./pose_projection.m"
source "./pose_pose.m"


function [XR, XL] = boxPlus(XR, XL, dx)
    global pose_dim landmark_dim;

    num_poses = length(XR);
    num_landmarks = length(XL);
    for i = 1 : num_poses
        pose_idx = pindex(i, num_poses, num_landmarks);
        dxr = dx(pose_idx : pose_idx + pose_dim - 1);
        XR(:,:,i) = v2t(dxr) * XR(:,:,i);
    endfor

    for j = 1 : num_landmarks
        landmark_idx = lindex(j, num_poses, num_landmarks);
        dxl = dx(landmark_idx : landmark_idx + landmark_dim - 1);
        XL(:, j) += 1.2*dxl;
    endfor
endfunction


function [XR, XL, tot_chi_proj, num_inliers_proj, tot_chi_pose, num_inliers_pose, H, b] = totalLS(XR, XL,
        Z_cam, Z_odom, damping, kernel_threshold, num_iterations)

    global pose_dim landmark_dim;


    num_poses = length(XR);
    num_landmarks = length(XL);

    tot_chi_proj = zeros(1, num_iterations);
    num_inliers_proj = zeros(1, num_iterations);
    tot_chi_pose = zeros(1, num_iterations);
    num_inliers_pose = zeros(1, num_iterations);

    system_size = pose_dim * num_poses + landmark_dim * num_landmarks;

    for i = 1 : num_iterations

        printf("iteration %d\n", i);

        H = zeros(system_size, system_size);
        b = zeros(system_size, 1);

        [H_proj, b_proj, chi_proj, inliers_proj] = linearizeProjections(XR, XL, Z_cam, kernel_threshold);
        [H_pose, b_pose, chi_pose, inliers_pose] = linearizePoses(XR, XL, Z_odom, kernel_threshold);

        tot_chi_proj(i) = chi_proj;
        num_inliers_proj(i) = inliers_proj;
        tot_chi_pose(i) = chi_pose;
        num_inliers_pose(i) = inliers_pose;

        H = H_pose + H_proj;
        b = b_pose + b_proj;

        H += eye(system_size) * damping;

        dx = zeros(system_size, 1);

        dx(pose_dim + 1 : end) = -(H(pose_dim + 1 : end, pose_dim + 1 : end) \ b(pose_dim + 1 : end, 1));
        [XR, XL] = boxPlus(XR, XL, dx);

        printf("projection chi value: ");
        disp(tot_chi_proj(i));
        printf("projection inliers: ");
        disp(num_inliers_proj(i));
        printf("pose chi value: ");
        disp(tot_chi_pose(i));
        printf("pose inliers: ");
        disp(num_inliers_pose(i));
        printf("+++++++++++++++++++++++++++++++\n")
    endfor
end