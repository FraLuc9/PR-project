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

% measurements of pose i+1 seen by pose i
[Z_odom, Z_odom_gt] = calculateOdometryMeasurements(odom, odom_gt);

[XL, points_seen, points_triangulated] = triangulate(XR, Z_proj, num_landmarks);

XL_gt = extractWorld();




kernel_threshold = 1e1;
damping = 1;
num_iterations = 20;


% LEAST SQUARES
[XR_LS, XL_LS, chi_proj, num_inliers_proj, chi_pose, num_inliers_pose, H, b] = totalLS(XR, XL, Z_proj, Z_odom, damping, kernel_threshold, num_iterations);


% PRIOR ERRORS
map_err_ig = mapMSE(XL, XL_gt);
printf("landmark error initial guess: %d\n\n", map_err_ig);

[rot_err_ig, transl_err_ig] = poseMSE(XR, XR_gt);
printf("pose error initial guess: \nrot: %d\ntransl: %d\n\n", rot_err_ig, transl_err_ig);


% POST OPTIMIZATION ERRORS
map_err_ls = mapMSE(XL_LS, XL_gt);
printf("landmark error after BA: %d\n\n", map_err_ls);

[rot_err_ls, transl_err_ls] = poseMSE(XR_LS,XR_gt);
printf("pose error after BA: \nrot: %d\ntransl: %d\n\n", rot_err_ls, transl_err_ls);


% PLOTS
figure(1);
hold on;
grid;

subplot(1,2,1);
title("Landmark Initial Guess");
plot3(XL_gt(1,:), XL_gt(2,:), XL_gt(3,:), 'b*', "linewidth",2);
hold on;
plot3(XL(1,:), XL(2,:), XL(3,:), 'ro', "linewidth", 2);
legend("Landmark True", "Initial Guess");
grid;

subplot(1,2,2);
title("Landmark After Optimization");
plot3(XL_gt(1,:), XL_gt(2,:), XL_gt(3,:),'b*',"linewidth", 2);
hold on;
plot3(XL_LS(1,:), XL_LS(2,:), XL_LS(3,:), 'ro', "linewidth", 2);
legend("Landmark True", "Least Squares Guess");
grid;


figure(2);
hold on;
grid;

subplot(1,2,1);
title("2D Landmark Initial Guess");
plot(XL_gt(1,:), XL_gt(2,:), 'b*', "linewidth",2);
hold on;
plot(XL(1,:), XL(2,:), 'ro', "linewidth", 2);
legend("Landmark True", "Initial Guess");
grid;

subplot(1,2,2);
title("2D Landmark After Optimization");
plot(XL_gt(1,:), XL_gt(2,:), 'b*',"linewidth", 2);
hold on;
plot(XL_LS(1,:), XL_LS(2,:), 'ro', "linewidth", 2);
legend("Landmark True", "Least Squares Guess");
grid;


figure(3);
hold on;
grid;

subplot(1,2,1);
title("Poses Initial Guess");
plot(XR_gt(1,4,:), XR_gt(2,4,:),'b*',"linewidth", 2);
hold on;
plot(XR(1,4,:), XR(2,4,:),'ro',"linewidth", 2);
legend("Poses True", "Initial Guess");
grid;

subplot(1,2,2);
title("Poses After Optimization");
plot(XR_gt(1,4,:), XR_gt(2,4,:), 'b*', "linewidth", 2);
hold on;
plot(XR_LS(1,4,:), XR_LS(2,4,:), 'r*', "linewidth", 2);
legend("Poses True", "Least Squares Guess"); 
grid;


figure(4);
hold on;
grid;
title("chi evolution");

subplot(2,2,1);
plot(chi_pose, 'r-', "linewidth", 2);
legend("Chi Poses"); 
grid; 
xlabel("iterations");
subplot(2,2,2);
plot(num_inliers_pose, 'b-', "linewidth", 2);
legend("#inliers"); 
grid; 
xlabel("iterations");

subplot(2,2,3);
plot(chi_proj, 'r-', "linewidth", 2);
legend("Chi Proj"); 
grid;
xlabel("iterations");
subplot(2,2,4);
plot(num_inliers_proj, 'b-', "linewidth", 2);
legend("#inliers");
grid;
xlabel("iterations");


figure(5);
title("H matrix");
H_ =  H./H;                      # NaN and 1 element
H_(isnan(H_))=0;                 # Nan to Zero
H_ = abs(ones(size(H_)) - H_);   # switch zero and one
H_ = flipud(H_);                 # switch rows
colormap(gray(64));
hold on;
image([0.5, size(H_,2)-0.5], [0.5, size(H_,1)-0.5], H_*64);
hold off;
