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
clear, clc, close all, format compact, format long g
addpath('../robot_data/za_description/urdf')
addpath('../CMM_data/40PointCloudRaw')
addpath('../CMM_data/ToolCloud')

addpath('/Users/andrewschneider/GaTech Dropbox/Andrew Schneider/calibration_data/IterativeMethod_1212')
cloud = readtable("InitialR2RPoints_1212.csv", "VariableNamingRule","preserve");

robot = importrobot('za.urdf');
% load("q.mat"), load("W.mat")


%% Select data set

% Rob1 to Rob2
% q1 = [0, 21, 45, 0, -66, 0];
% q2 = [15, 50, 30, -80, 90, -180];
% 
% q1 = deg2rad(q1);
% q2 = deg2rad(q2);

q1 = [-0.24789175391063686, 0.730726957311628, 0.6559345722258424, -1.4246836900715545, -1.6855932474137816, -0.15966585279311146];
q2 = [0.21151547133604728, 0.7422233223860509, 0.4045844078168699, 1.4762687683103108, -1.5276070833154838, -0.4220822751444625];

% q1 = q(1,:);
% q2 = q(2,:);


%% Configure Robot 1  
robot1 = robot;
config1 = homeConfiguration(robot1);
[config1.JointPosition] = deal([q1(1)],[q1(2)],[q1(3)],[q1(4)],[q1(5)],[q1(6)]);
%% Configure Robot 2 
robot2 = robot;
config2 = homeConfiguration(robot2);
[config2.JointPosition] = deal([q2(1)],[q2(2)],[q2(3)],[q2(4)],[q2(5)],[q2(6)]);

%% Designed tool points
% % 3x20 tool frame coords
% tool_cloud = readtable("tool_cloud.csv", "VariableNamingRule","preserve");
% V1_J6 = transpose(tool_cloud{:,4:6}/1000);

% 3 x 6 Flange bolt hole pattern in meters
tool_cloud = readtable("HexToolFramePoints_1212.csv", "VariableNamingRule","preserve");
%V1_J6 = transpose(tool_cloud{:,4:6}/1000);
V1_J6 = CMMCloudRead(tool_cloud)/1000;

%% Data import from CMM
n = 12;
W = CMMCloudRead(cloud);
% confirm simulated dataset
% W = [W_CMM(:,:,1),W_CMM(:,:,2)]*1000;

W1_CMM = [W(:,1:n/2)]/1000;
W2_CMM = [W(:,n/2+1:n)]/1000;



%% Bases expressed in Flange Frames
H_J6Base1 = getTransform(robot1,config1,"base","link_6");
H_J6Base2 = getTransform(robot2,config2,"base","link_6");

%% SVD Function Point Registration

H_CMMJ61 = CloudReg(V1_J6,W1_CMM); % rad, m
H_CMMB1 = H_CMMJ61*H_J6Base1;

H_CMMJ62 = CloudReg(V1_J6,W2_CMM); % rad, m
H_CMMB2 = H_CMMJ62*H_J6Base2;

H_B1B2 = inv(H_CMMB1)*H_CMMB2;

T_B1B2 = norm(H_B1B2(1:3,4))*1000;

% H_W1W2 = CloudReg(W2_CMM,W1_CMM)
%% Find and Convert to Frame at Midpoint Between Arms
T_CMMMiddle = (H_CMMB1(1:3,4)+H_CMMB2(1:3,4))/2;
H_CMMMiddle = [1 0 0 T_CMMMiddle(1);
               0 1 0 T_CMMMiddle(2);
               0 0 1 T_CMMMiddle(3);
               0 0 0     1         ;];


% alex output
eul_alex = [pi/2, 0, 0];
alexrotm = eul2rotm(eul_alex,'ZYX');
H_CMMMid(1:3,1:3) = alexrotm;
H_CMMMid = [H_CMMMid(1:3,1:3),T_CMMMiddle;0,0,0,1];


H_M1B1 = inv(H_CMMMid)*H_CMMB1;
H_B1M1 = inv(H_M1B1);
T_B1M1 = H_B1M1(1:3,4)'*1000
eul1 = rotm2eul(H_M1B1(1:3,1:3));
eul1 = flip(eul1)

H_M2B2 = inv(H_CMMMid)*H_CMMB2;
H_B2M2 = inv(H_M2B2);
T_B2M2 = H_B2M2(1:3,4)'*1000
eul2 = rotm2eul(H_M2B2(1:3,1:3));
eul2 = flip(eul2)

save("Base2Middle.mat","H_B1M1","H_B2M2")

%% Plot Stuff!

figure(1)
hold on
ax1 = plotTransforms(se3(H_CMMB1),'FrameAxisLabels',"on","FrameLabel","Robot 1","FrameSize",.5);
ax2 = plotTransforms(se3(H_CMMB2),'FrameAxisLabels',"on","FrameLabel","Robot 2","FrameSize",.5);
ax3 = plotTransforms(se3(H_CMMMid),'FrameAxisLabels',"off","FrameLabel","AlexWorld","FrameSize",.1);

axis normal
grid on
view(160, 30)

%% Plot Stuff! UNUSED
% close all
% figure(2)
% show(robot1,config1,Frames="on",Position=[H_CMMB1(1,4) H_CMMB1(2,4) H_CMMB1(3,4) deg2rad(eul1(3))]);
% hold on
% show(robot2,config2,Frames="on",Position=[H_CMMB2(1,4) H_CMMB2(2,4) H_CMMB2(3,4) deg2rad(eul2(3))]);
% 
% plot3([H_CMMB1(1,4) 0 H_CMMB2(1,4)],[H_CMMB1(2,4) 0 H_CMMB2(2,4)],[H_CMMB1(3,4) 0 H_CMMB2(3,4)],'-*r')
% % xlim([-1 1])
% % ylim([-.2 .8])
% % zlim([-.25 1.5])
% axis padded
% view(-30, 30)
% plot3([H_CMMB1(1,4) T_CMMMiddle(1) H_CMMB2(1,4)],[H_CMMB1(2,4) T_CMMMiddle(2) H_CMMB2(2,4)],[H_CMMB1(3,4) T_CMMMiddle(3) H_CMMB2(3,4)],'-*b')
% 
% text(0,0,0+.075, 'CMM origin')
% text(T_CMMMiddle(1), T_CMMMiddle(2), T_CMMMiddle(3)+.075,'XY Midpoint')
% text(H_CMMB1(1,4)-.15, H_CMMB1(2,4)-.11,H_CMMB1(3,4)-.15,'Robot 1')
% text(H_CMMB2(1,4)-.15, H_CMMB2(2,4)-.11,H_CMMB2(3,4)-.15,'Robot 2')
% xlabel('X'), ylabel('Y'), zlabel('Z');

%% Path Pilot Outputs
% T_B1M = T_B1M
% eul1 = eul1
% 
% T_B2M = T_B2M
% eul2 = eul2

%% Self Register Output
% H_B1B2 = H_B1M1*inv(H_B2M2);

