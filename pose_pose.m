source "./utils.m"



global dRz0 = [0 -1 0; 1 0 0; 0 0 0];


function [e, Ji, Jj] = poseErrorAndJacobian(Xi, Xj, Z)

    global dRx0 dRy0 dRz0;
    global pose_dim landmark_dim;

    % SE(2) rotation around z
    Ri = Xi(1:2,1:2);
    Rj = Xj(1:2,1:2);

    ti = Xi(1:2,4);
    tj = Xj(1:2,4);

    % SE(2) CHORDAL ERROR
    g =inv(Xi)*Xj;
    Zhat = [g(1:2,1);g(1:2,2);g(1:2,4)];
    e = Zhat - [Z(1:2,1);Z(1:2,2);Z(1:2,4)];

    % SE(2) CHORDAL JACOBIANS
    Ji = zeros(6,3);
    Jj = zeros(6,3);

    % SE(2) prediction derivative along the z axis
    dg_alphaz = [Ri'*dRz0(1:2,1:2)*Rj Ri'*dRz0(1:2,1:2)*tj];

    % SE(2) prediction derivative along the x and y directions
    dg_tx = [zeros(2,2) Ri'* [1; 0]];
    dg_ty = [zeros(2,2) Ri'* [0; 1]];

    Jj(:,1) = reshape(dg_tx, 6, 1);
    Jj(:,2) = reshape(dg_ty, 6, 1);
    Jj(:,3) = reshape(dg_alphaz, 6, 1);

    Ji = -Jj;

endfunction

function [H, b, chi_tot, inliers] = linearizePoses(XR, XL, Zr, kernel_threshold)

    global pose_dim landmark_dim;

    num_poses = length(XR);
    num_landmarks = length(XL);
    system_size = pose_dim * num_poses + landmark_dim * num_landmarks;
    
    H = zeros(system_size, system_size);
    b = zeros(system_size, 1);
    chi_tot = 0;
    inliers = 0;
    for i = 1 : length(Zr) -1
        
        Omega = eye(2 * pose_dim) * 1e3;
        
        % odometry measurement for pose i+1 seen by pose i
        Z = v2t3D(Zr(:,i+1));
        Xi = XR(:,:,i);
        Xj = XR(:,:,i+1);
        [e, Ji, Jj] = poseErrorAndJacobian(Xi, Xj, Z);

        chi = e' * Omega * e;
        if chi > kernel_threshold
            Omega *= sqrt(kernel_threshold/chi);
            chi = kernel_threshold;
        else
            inliers++;
        endif

        chi_tot += chi;

        H_ii = Ji' * Omega * Ji;
        H_ij = Ji' * Omega * Jj;
        H_ji = Jj' * Omega * Ji;
        H_jj = Jj' * Omega * Jj;
        b_i = Ji' * Omega * e;
        b_j = Jj' * Omega * e;

        pose_i_idx = pindex(i, pose_dim, landmark_dim, num_poses, num_landmarks);
        pose_j_idx = pindex(i+1, pose_dim, landmark_dim, num_poses, num_landmarks);

        H(pose_i_idx : pose_i_idx + pose_dim - 1, pose_i_idx : pose_i_idx + pose_dim - 1) += H_ii;

        H(pose_i_idx : pose_i_idx + pose_dim - 1, pose_j_idx : pose_j_idx + pose_dim - 1) += H_ij;

        H(pose_j_idx : pose_j_idx + pose_dim - 1, pose_i_idx : pose_i_idx + pose_dim - 1) += H_ji;

        H(pose_j_idx : pose_j_idx + pose_dim - 1, pose_j_idx : pose_j_idx + pose_dim - 1) += H_jj;

        b(pose_i_idx : pose_i_idx + pose_dim - 1, 1) += b_i;

        b(pose_j_idx : pose_j_idx + pose_dim - 1, 1) += b_j;

    endfor

endfunction
