close all
clear all
clc

source "./dataExtractUtils.m"
source "./triangulation.m"


[odometry,gt_odom] = readTrajectory();


[K,T,z_near,z_far,width,height] = extractCamParams();

meas = extractMeasurements();

% K,T,z_near,z_far,width,height = extractCamParams();

% ao = zeros(3,1);
% indexes = [];
% for i=1 : length(meas)-1
%     if isKey(meas(i),6) && isKey(meas(i+1), 6)
%         indexes(end+1) = i;
%         T1 = v2t3D(gt_odom(:,i))*T;
%         C1 = K * [T1(1:3,1:3) T1(1:3,4)];
%         T2 = v2t3D(gt_odom(:,i+1))*T;
%         C2 = K * [T2(1:3,1:3) T2(1:3,4)];
%         p = triangulate(meas(i)(6), meas(i+1)(6), C1, C2);

%         if (C1 * p)(3) > 0 && (C2 * p)(3) > 0 && (C1 * p)(3) < z_far && (C2 * p)(3) < z_far 
%             ao(:,end+1)= p(1:3);
%         end
% function A = col(meas,T,K, odom)
printf("building A...\n")
A = cell(1,1000);
for i = 0:999
    printf("triangulating landmark ")
    disp(i)
    printf("...\n")
    for j = 1:length(meas)-1
        if isKey(meas(j), i)
            T_j = v2t3D(odometry(:,j))*T;
            C_j = K * [T_j(1:3,1:3) T_j(1:3,4)];
            p_j = meas(j)(i);
            A{i+1} = [A{i+1}; p_j(2)*C_j(3,:)-C_j(2,:); p_j(1)*C_j(3,:)-C_j(1,:)];
        end
    end
end
% endfunction

printf("done!\n")


        
%     end
% end
% ao = ao(:,2:end);

% points = {};
% cam_poses = {};
% for i = 1 : length(meas)
%     if(isKey(meas(i),6))
%         T = v2t3D(gt_odom(:,i));
%         C = K * [T(1:3,1:3)]