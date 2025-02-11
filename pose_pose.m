source "./utils.m"


%% REWRITE FOR 2D POSES MADONNA

global dRx0 = [0 0 0; 0 0 -1; 0 1 0];
global dRy0 = [0 0 1; 0 0 0; -1 0 0];
global dRz0 = [0 -1 0; 1 0 0; 0 0 0];


function [e, Ji, Jj] = poseErrorAndJacobian(Xi, Xj, Z)

    global dRx0 dRy0 dRz0;

    Ri = Xi(1:3,1:3);
    Rj = Xj(1:3,1:3);
    ti = Xi(1:3,4);
    tj = Xj(1:3,4);

    # prediction
    g = inv(Xi)*Xj;

    # CHORDAL ERROR
    Zhat = flatten4(g);
    e = Zhat - flatten4(Z);
    

    rx = flatten3(Ri'*dRx0*Rj);
    ry = flatten3(Ri'*dRy0*Rj);
    rz = flatten3(Ri'*dRz0*Rj);
    #CHORDAL JACOBIANS
    Ji = zeros(12, 6);
    Jj = zeros(12, 6);

    Jj(1:9,4:6) = [rx ry rz];
    Jj(10:12,1:3) = Ri';
    Jj(10:12,4:6) = -Ri'*skew(tj);
    
    Ji = -Jj;

endfunction

function [H, b, chi_tot, inliers] = linearizePoses(XR, XL, Zr, kernel_threshold)

    num_poses = length(XR);
    num_landmarks = length(XL);
    pose_dim = 6;
    landmark_dim = 3;
    system_size = pose_dim * num_poses + landmark_dim * num_landmarks;
    
    H = zeros(system_size, system_size);
    b = zeros(system_size, 1);
    chi_tot = 0;
    inliers = 0;
    for i = 1 : length(Zr) -1
        
        Omega = eye(12);
        Omega(1:9, 1:9) *= 1e3;
        
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
        end
        chi_tot += chi;

        H_ii = Ji' * Omega * Ji;
        H_ij = Ji' * Omega * Jj;
        H_ji = Jj' * Omega * Ji;
        H_jj = Jj' * Omega * Jj;
        b_i = Ji' * Omega * e;
        b_j = Jj' * Omega * e;

        pose_i_idx = 1 + (i - 1) * pose_dim;
        pose_j_idx = 1 + i * pose_dim;

        H(pose_i_idx : pose_i_idx + pose_dim - 1, pose_i_idx : pose_i_idx + pose_dim - 1) += H_ii;

        H(pose_i_idx : pose_i_idx + pose_dim - 1, pose_j_idx : pose_j_idx + pose_dim - 1) += H_ij;

        H(pose_j_idx : pose_j_idx + pose_dim - 1, pose_i_idx : pose_i_idx + pose_dim - 1) += H_ji;

        H(pose_j_idx : pose_j_idx + pose_dim - 1, pose_j_idx : pose_j_idx + pose_dim - 1) += H_jj;

        b(pose_i_idx : pose_i_idx + pose_dim - 1, 1) += b_i;

        b(pose_j_idx : pose_j_idx + pose_dim - 1, 1) += b_j;

    endfor



endfunction
# testino

% X1 = [1 0 0 1;
%       0 1 0 1;
%       0 0 1 0;
%       0 0 0 1];
% X2 = [1 0 0 2;
%       0 1 0 2;
%       0 0 1 0;
%       0 0 0 1];

% Z = inv(X1)*X2;

% [e, j1, j2] = poseErrorAndJacobian(X1, X2, Z);
% j1
% j2