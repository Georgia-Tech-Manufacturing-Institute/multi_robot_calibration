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
%% Robot 1 Import
robot1 = importrobot('za.urdf');
config1 = homeConfiguration(robot1);
config1(1).JointPosition = deg2rad(0.137);
config1(2).JointPosition = deg2rad(38.415);
config1(3).JointPosition = deg2rad(52.065);
config1(4).JointPosition = deg2rad(1.298);
config1(5).JointPosition = deg2rad(-90.610);
config1(6).JointPosition = deg2rad(14.454);

%% Robot 2 Import

robot2 = importrobot('za.urdf');
config2 = homeConfiguration(robot2);
config2(1).JointPosition = deg2rad(26.406);
config2(2).JointPosition = deg2rad(23.369);
config2(3).JointPosition = deg2rad(9.660);
config2(4).JointPosition = deg2rad(105.939);
config2(5).JointPosition = deg2rad(-67.422);
config2(6).JointPosition = deg2rad(-35.926);

%% Designed tool points
%       X      Y      Z
% v1 = [-4.079; 19.427; 10.007];
% v2 = [-4.122; 19.510; 20.063];
% v3 = [5.950; 18.968; 10.149];
% v4 = [5.856; 19.036; 20.137];
% 
% v5 = [12.144; 12.003; 10.203];
% v6 = [12.067; 12.033; 20.247];
% v7 = [11.013; -13.005; 10.468];
% v8 = [10.916; -12.972; 20.413];
% 
% v9  = [4.103; -19.281; 10.399];
% v10 = [4.057; -19.205; 20.430];
% v11 = [-5.956; -18.801; 10.426];
% v12 = [-6.012; -18.705; 20.391];
% 
% v13 = [-12.311; -11.923; 10.357];
% v14 = [-12.324; -11.816; 20.325];
% v15 = [-11.146; 13.019; 10.140];
% v16 = [-11.196; 13.157; 20.113];
% 
% v17 = [-4.643; 12.833; 26.825];
% v18 = [5.487; 12.386; 26.861];
% v19 = [-5.740; -12.115; 27.011];
% v20 = [4.243; -12.584; 27.049];

v1 = [9.994; -18.997; -5.107];
v2 = [20.036; -19.042; -5.214];
v3 = [10.078; -19.136; 4.830];
v4 = [20.069; -19.153; 4.784];

v5 = [10.063; -12.564; 11.403];
v6 = [20.098; -12.544; 11.379];
v7 = [10.203; 12.431; 11.775];
v8 = [20.141; 12.434; 11.712];

v9  = [10.115; 19.048; 5.249];
v10 = [20.148; 19.032; 5.256];
v11 = [10.204; 19.165; -4.827];
v12 = [20.166; 19.129; -4.835];

v13 = [10.178; 12.751; -11.557];
v14 = [20.163; 12.708; -11.541];
v15 = [10.108; -12.215; -11.901];
v16 = [20.093; -12.285; -11.913];

v17 = [26.745; -12.322; -5.317];
v18 = [26.737; -12.485; 4.733];
v19 = [26.807; 12.634; -4.918];
v20 = [26.799; 12.515; 5.074];



% 3x20 matrix
V =[v1, v2, v3, v4, v5, v6, v7, v8, v9, v10, v11, v12, v13, v14, v15, v16, v17, v18, v19, v20;]/1000;

%% Align coordinate points frame to urdf frame
% Note: The flange frame used in the urdf and Tormach PathPilot differ
% The CMM built frame was aligned with PathPilot, this code snippet
% aligns the points with the flange frame used in the URDF.
% The CMM/PathPilot Flange frame is: Z out of End Effector face, X up.

% eul = [pi, pi/2, 0];
% rotmXYZ = eul2rotm(eul,'XYZ');
% V1 = rotmXYZ*V;
V1 = V;

%% Process Check
% Moves points to flange for qualitative check
% Do the points visibly match where they were when measured in real life?

% H_B6 = getTransform(robot1,config1,'flange','base_link');
% V = H_B6*[V1; ones(1,20)];
% V = V(1:3,1:20);
% figure(1)
% show(robot1,config1);
% hold on
% plot3([V(1,:)], [V(2,:)], [V(3,:)],'--*b')
% xlabel('X'), ylabel('Y'), zlabel('Z')
% plot3([V(1,1)], [V(2,1)], [V(3,1)],'or')
%% Data import from CMM
cloud = readtable("rob2_self_transform_092024.csv");
n = 40;
[W1,W2] = CMMCloudRead(cloud,n);

%% Use Function

H_CMMB1 = Cmm2Robot(V1,W1,robot1,config1);
H_CMMB2 = Cmm2Robot(V1,W2,robot2,config2);

%% Find and convert to world between bots
T_mid = [(H_CMMB1(1,4)+H_CMMB2(1,4))/2, (H_CMMB1(2,4)+H_CMMB2(2,4))/2,((H_CMMB1(3,4)+H_CMMB2(3,4))/2)];
H_CMMMid = [1 0 0 T_mid(1);
          0 1 0 T_mid(2);
          0 0 1 T_mid(3);
          0 0 0     1   ;];

eul_alex = [pi/2, 0, 0];
alexrotm = eul2rotm(eul_alex,'ZYX');

% H_CMMMid(1:3,1:3) = alexrotm;

H_MB1 = inv(H_CMMMid)*H_CMMB1;
H_B1M = inv(H_MB1);
T_B1M = H_B1M(1:3,4)*1000
eul1 = rotm2eul(H_B1M(1:3,1:3));
eul1 = flip(rad2deg(eul1))

H_MB2 = inv(H_CMMMid)*H_CMMB2;
H_B2M = inv(H_MB2);
T_B2M = H_B2M(1:3,4)*1000
eul2 = rotm2eul(H_B2M(1:3,1:3));
eul2 = flip(rad2deg(eul2))

HB1B2 = H_B1M*inv(H_B2M)

%% Plot Stuff!
close all
figure(1)
show(robot1,config1,Frames="on",Position=[H_CMMB1(1,4) H_CMMB1(2,4) H_CMMB1(3,4) deg2rad(eul1(3))]);
hold on
show(robot2,config2,Frames="on",Position=[H_CMMB2(1,4) H_CMMB2(2,4) H_CMMB2(3,4) deg2rad(eul2(3))]);

plot3([H_CMMB1(1,4) 0 H_CMMB2(1,4)],[H_CMMB1(2,4) 0 H_CMMB2(2,4)],[H_CMMB1(3,4) 0 H_CMMB2(3,4)],'-*r')
% xlim([-1 1])
% ylim([-.2 .8])
% zlim([-.25 1.5])
axis padded
view(-30, 30)
plot3([H_CMMB1(1,4) T_mid(1) H_CMMB2(1,4)],[H_CMMB1(2,4) T_mid(2) H_CMMB2(2,4)],[H_CMMB1(3,4) T_mid(3) H_CMMB2(3,4)],'-*b')

text(0,0,0+.075, 'CMM origin')
text(T_mid(1), T_mid(2), T_mid(3)+.075,'XY Midpoint')
text(H_CMMB1(1,4)-.15, H_CMMB1(2,4)-.11,H_CMMB1(3,4)-.15,'Robot 1')
text(H_CMMB2(1,4)-.15, H_CMMB2(2,4)-.11,H_CMMB2(3,4)-.15,'Robot 2')
xlabel('X'), ylabel('Y'), zlabel('Z');


