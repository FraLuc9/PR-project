source "dataExtractUtils.m"
source "utils.m"


#butchery trials

odometry = readTrajectory("data/trajectory.dat");

l = length(odometry);

#3D poses

#GT
XR_true = zeros(4,4,l);

#Odometry
XR_odo = XR_true;


XR_true(:,:,1) = v2t3D(odometry(4:6,1));

XR_odo(:,:,1) = v2t3D(odometry(1:3,1));
for(i=2:l)
    dXr_o = v2t3D(odometry(1:3,i));
    dXr_t = v2t3D(odometry(4:6,i));
    XR_true(:,:,i) = XR_true(:,:,i-1)*dXr_t;
    XR_odo(:,:,i) = XR_odo(:,:,i-1)*dXr_o;
endfor


#trial stuff
campath = "data/camera.dat";

[K,T,z_near,z_far,width,height] = extractCamParams(campath);

XR_odo(:,:,1)*T;
XR_true(:,:,1)*T;


XR_true(:,:,200);
XR_odo(:,:,200);

rel_GT = inv(XR_odo(:,:,19))*XR_odo(:,:,20)
rel_T = inv(XR_true(:,:,19))*XR_true(:,:,20)

error_T = inv(rel_T)*rel_GT

#errors
error_translation = sqrt(error_T(1:3,4)'*error_T(1:3,4))
error_rotation = atan2(error_T(2,1), error_T(1,1))