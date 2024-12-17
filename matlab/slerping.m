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

%% Slerping
% aka iteratively averaging toward a common World frame
clear, clc, close all, format compact
load("Base2Middle.mat")

%%
addpath('/Users/andrewschneider/GaTech Dropbox/Andrew Schneider/calibration_data/IterativeMethod_1212')
cloud = readtable("relative_trans_20241216.csv", "VariableNamingRule","preserve");
tool_cloud = readtable("HexToolFramePoints_1212.csv", "VariableNamingRule","preserve");
V1_J6 = CMMCloudRead(tool_cloud)/1000;

W = cloud{1:end,4:end}';
n = length(W);

A = [W(:,1:n/2)]/1000;
B = [W(:,n/2+1:n)]/1000;

error = sum(vecnorm(A-B))*1000
figure(1)
plot3(A(1,:)',A(2,:)',A(3,:)',"-.b")
hold on
plot3(B(1,:)',B(2,:)',B(3,:)',"-.r")

H_BA = CloudReg(A,B); % rad, m

T_BA = H_BA(1:3,4);

T_M1M1new = 0.5*T_BA;
T_M2M2new = -0.5*T_BA;

q0 = quaternion([0,0,0],"eulerd","XYZ","frame");
q_BA = quaternion(tform2quat(H_BA));

q_M1M1new = slerp(q0,q_BA,0.5);
q_M2M2new = slerp(q_BA,q0,0.5);

R_M1M1new = quat2rotm(q_M1M1new);
R_M2M2new = quat2rotm(q_M2M2new);

H_M1M1new = [R_M1M1new,T_M1M1new;0,0,0,1];
H_M2M2new = [R_M2M2new,T_M2M2new;0,0,0,1];

H_B1M1new = H_B1M1*H_M1M1new;
T_B1M1new = H_B1M1new(1:3,4)'*1000
eul1 = rotm2eul(H_B1M1new(1:3,1:3));
eul1 = flip(eul1)

H_B2M2new = H_B2M2*H_M2M2new;
T_B2M2new = H_B2M2new(1:3,4)'*1000
eul2 = rotm2eul(H_B2M2new(1:3,1:3));
eul2 = flip(eul2)

H_B1M1 = H_B1M1new;
H_B2M2 = H_B2M2new;

save("iteration.mat","H_B1M1","H_B2M2")

%% Plot Stuff!

figure(2)
hold on
ax1 = plotTransforms(se3(H_B1M1new),'FrameAxisLabels',"on","FrameLabel","Robot 1","FrameSize",.5);
ax2 = plotTransforms(se3(H_B2M2new),'FrameAxisLabels',"on","FrameLabel","Robot 2","FrameSize",.5);
ax3 = plotTransforms(se3(H_AlexRot),'FrameAxisLabels',"off","FrameLabel","AlexWorld","FrameSize",.1);

xlabel('x')
ylabel('y')
zlabel('z')

axis normal
grid on
view(160, 30)

