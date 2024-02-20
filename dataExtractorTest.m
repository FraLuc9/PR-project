
source "./dataExtractUtils.m"

#camera.dat path
campath = "data/camera.dat";

[K,T,z_near,z_far,width,height] = extractCamParams(campath);

world = extractWorld("data/world.dat");

trajpath = "data/trajectory.dat";

#each column is (id, [xytheta_guess], [xytheta_true])
#odometry injects 0mean 0.001cov noise into the ground truth vals, both linear and angular
#angles are between pi and -pi
odometry = readTrajectory(trajpath);
od = odometry(1:3,:);
gt = odometry(4:6,:);



## VISUALIZATION STUFF

plot(od(1,:),od(2,:), "r", ...
    gt(1,:), gt(2,:), "b", ...
    world(1,:), world(2,:), "ro", "MarkerSize", 1.5);
title("dioca");
xlabel("x");
ylabel("y");
grid on;
grid minor;
saveas(gcf, "dioc.png");

#saveas("diocan.png")
% measurepath =  "data/meas-00190.dat";
% [seq,p,points] = extractMeasurements(measurepath);

% odometry(4:6, 191);
% p(4:6,1);