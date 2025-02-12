close all
clear all
clc

source "./dataExtractUtils.m"
source "./total_LS.m"

global K T z_near z_far width height;

global pose_dim landmark_dim;

[odom, odom_gt, XR, XR_gt] = readTrajectory();

pose_dim = 3;
landmark_dim = 3;

% assuming knowledge of total landmark number in order to save memory
num_landmarks = 1000;

[K,T,z_near,z_far,width,height] = extractCamParams();

Z_proj = extractMeasurements();

[Z_odom, Z_odom_gt] = calculateOdometryMeasurements(odom, odom_gt);

[XL, points_seen, points_triangulated] = triangulate(XR, Z_proj, num_landmarks);

XL_gt = extractWorld("data/world.dat");

kernel_threshold = 1e3;
damping = 1;
num_iterations = 20;

[XR_LS, XL_LS, tot_chi_proj, num_inliers_proj, tot_chi_pose, num_inliers_pose, H, b] = totalLS(XR, XL, Z_proj, Z_odom, damping, kernel_threshold, num_iterations);