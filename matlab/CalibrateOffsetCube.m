% Copyright 2024 Andrew Schneider, Alex Arbogast
%
% Licensed under the Apache License, Version 2.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
%
%     http://www.apache.org/licenses/LICENSE-2.0
%
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.

%% URDF Import for Tormach ZA6
% Using the designed calibration cube points and CMM Measured points
% Script calculates the Rotation and translation vectors between tool and
% CMM for 2 robots
% Created by Andrew Schneider, June 27, 2024
% Edited: Sep 2, 2024
clear, clc, close all, format compact
addpath('../robot_data/za_description/urdf')
addpath('../CMM_data/40PointCloudRaw')
addpath('../CMM_data/ToolCloud')

% % Rob1 to Rob1
% Q1 = deg2rad([0, 94.119, -6.812, 0, -87.307, 0]);
% Q2 = deg2rad([-11.278, 53.020, 17.195, -82.596, -122.65, 48.863]);
% 
% % Rob2 to Rob2
% Q1 = deg2rad([0.137, 38.415, 52.065, 1.298, -90.610, 14.454]);
% Q2 = deg2rad([26.406, 23.369, 9.66, 105.939, -67.422, -35.926]);

% Rob1 to Rob2
Q1 = deg2rad([0, 21, 45, 0, -66, 0]);
Q2 = deg2rad([0, 21, 45, 0, -66, 0]);

%% Configure Robot 1  
robot1 = importrobot('za.urdf');
config1 = homeConfiguration(robot1);
[config1.JointPosition] = deal([Q1(1)],[Q1(2)],[Q1(3)],[Q1(4)],[Q1(5)],[Q1(6)]);

%% Configure Robot 2 
robot2 = importrobot('za.urdf');
config2 = homeConfiguration(robot2);
[config2.JointPosition] = deal([Q2(1)],[Q2(2)],[Q2(3)],[Q2(4)],[Q2(5)],[Q2(6)]);

%% Designed tool points
% 3x20 tool frame coords
tool_cloud = readtable("tool_cloud.csv");
V1_J6 = transpose(tool_cloud{:,4:6}/1000);

%% Data import from CMM
cloud = readtable("robot_to_robot_091124.csv");
n = 40;
[W1_CMM,W2_CMM] = CMMCloudRead(cloud,n);

%% MatLab image toolbox point registration
V1_J6_cloud = pointCloud(transpose(V1_J6));
W1_CMM_cloud = pointCloud(transpose(W1_CMM));
W2_CMM_cloud = pointCloud(transpose(W2_CMM));

H_CMMJ6_cloud = pcregistericp(V1_J6_cloud,W1_CMM_cloud);
H_J6Base1 = getTransform(robot1,config1,"base","flange");
H_CMMB1 = H_CMMJ6_cloud.A*H_J6Base1; % rad, m

H_CMMJ6_cloud = pcregistericp(V1_J6_cloud,W2_CMM_cloud);
H_J6Base2 = getTransform(robot2,config2,"base","flange");
H_CMMB2 = H_CMMJ6_cloud.A*H_J6Base2; % rad, m

H_B1B2 = inv(H_CMMB1)*H_CMMB2;

%% Use Function UNUSED

% H_CMMJ61 = CloudReg(V1_J6,W1_CMM); % rad, m
% H_J6Base = getTransform(robot1,config1,"base","flange");
% H_CMMB1 = H_CMMJ61*H_J6Base;
% 
% H_CMMJ62 = CloudReg(V1_J6,W2_CMM); % rad, m
% H_J6Base = getTransform(robot2,config2,"base","flange");
% H_CMMB2 = H_CMMJ62*H_J6Base;

% H_B1B2 = inv(H_CMMB1)*H_CMMB2;

% H_W1W2 = inv(CloudReg(W2_CMM,W1_CMM))

%% Find and Convert to Frame at Midpoint Between Arms
T_CMMMiddle = [(H_CMMB1(1,4)+H_CMMB2(1,4))/2, (H_CMMB1(2,4)+H_CMMB2(2,4))/2,((H_CMMB1(3,4)+H_CMMB2(3,4))/2)];
H_CMMMiddle = [1 0 0 T_CMMMiddle(1);
               0 1 0 T_CMMMiddle(2);
               0 0 1 T_CMMMiddle(3);
               0 0 0     1         ;];


% alex output
% eul_alex = [pi/2, 0, 0];
% alexrotm = eul2rotm(eul_alex,'ZYX');
% H_CMMMid(1:3,1:3) = alexrotm;

H_MB1 = inv(H_CMMMiddle)*H_CMMB1;
H_B1M = inv(H_MB1);
T_B1M = H_B1M(1:3,4)*1000;
eul1 = rotm2eul(H_B1M(1:3,1:3));
eul1 = flip(rad2deg(eul1));

H_MB2 = inv(H_CMMMiddle)*H_CMMB2;
H_B2M = inv(H_MB2);
T_B2M = H_B2M(1:3,4)*1000;
eul2 = rotm2eul(H_B2M(1:3,1:3));
eul2 = flip(rad2deg(eul2));

%% Plot Stuff!

figure(1)
hold on
ax1 = plotTransforms(se3(H_CMMB1),'FrameAxisLabels',"on","FrameLabel","Robot 1","FrameSize",.5);
ax2 = plotTransforms(se3(H_CMMB2),'FrameAxisLabels',"on","FrameLabel","Robot 2","FrameSize",.5);
axis normal
grid on
view(160, 30)

%% Plot Stuff! UNUSED
close all
figure(2)
show(robot1,config1,Frames="on",Position=[H_CMMB1(1,4) H_CMMB1(2,4) H_CMMB1(3,4) deg2rad(eul1(3))]);
hold on
show(robot2,config2,Frames="on",Position=[H_CMMB2(1,4) H_CMMB2(2,4) H_CMMB2(3,4) deg2rad(eul2(3))]);

plot3([H_CMMB1(1,4) 0 H_CMMB2(1,4)],[H_CMMB1(2,4) 0 H_CMMB2(2,4)],[H_CMMB1(3,4) 0 H_CMMB2(3,4)],'-*r')
% xlim([-1 1])
% ylim([-.2 .8])
% zlim([-.25 1.5])
axis padded
view(-30, 30)
plot3([H_CMMB1(1,4) T_CMMMiddle(1) H_CMMB2(1,4)],[H_CMMB1(2,4) T_CMMMiddle(2) H_CMMB2(2,4)],[H_CMMB1(3,4) T_CMMMiddle(3) H_CMMB2(3,4)],'-*b')

text(0,0,0+.075, 'CMM origin')
text(T_CMMMiddle(1), T_CMMMiddle(2), T_CMMMiddle(3)+.075,'XY Midpoint')
text(H_CMMB1(1,4)-.15, H_CMMB1(2,4)-.11,H_CMMB1(3,4)-.15,'Robot 1')
text(H_CMMB2(1,4)-.15, H_CMMB2(2,4)-.11,H_CMMB2(3,4)-.15,'Robot 2')
xlabel('X'), ylabel('Y'), zlabel('Z');

%% Path Pilot Outputs
T_B1M = T_B1M
eul1 = eul1

T_B2M = T_B2M
eul2 = eul2

%% Self Register Output
% H_B1B2 = H_B1M*inv(H_B2M);