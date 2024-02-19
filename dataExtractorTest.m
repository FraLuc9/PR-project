
source "./dataExtractUtils.m"

#camera.dat path
campath = "data/camera.dat";

[K,T,z_near,z_far,width,height] = extractCamParams(campath);


trajpath = "data/trajectory.dat";

#each column is (id, [xytheta_guess], [xytheta_true])
#odometry injects 0mean 0.001cov noise into the ground truth vals, both linear and angular
#angles are between pi and -pi
odometry = readTrajectory(trajpath)
