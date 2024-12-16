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
cloud = readtable("SlerpIter0_1212.csv", "VariableNamingRule","preserve");
tool_cloud = readtable("HexToolFramePoints_1212.csv", "VariableNamingRule","preserve");
V1_J6 = CMMCloudRead(tool_cloud)/1000;

n = 16;
m = 6
% W = CMMCloudRead(cloud);
for i = 1:n
    P1(:,:,i) = table2array(cloud((i-1)*m+1:i*m,4:6))'
    H_V1P1 = CloudReg(V1_J6,P1(:,:,i))
    Cubepoints((i-1)*3+1:i*3,1) = H_V1P1(1:3)
end

A = [W(:,1:n/2)]/1000;
B = [W(:,n/2+1:n)]/1000;

error = sum(vecnorm(A-B)) 
% figure(1)
% plot3(A(1,:)',A(2,:)',A(3,:)',"-.b")
% hold on
% plot3(B(1,:)',B(2,:)',B(3,:)',"-.r")


H_BA = CloudReg(A,B) % rad, m

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

% eul_alex = [pi/2, 0, 0];
% alexrotm = eul2rotm(eul_alex,'ZYX');
% H_CMMMid(1:3,1:3) = alexrotm;
% H_AlexRot = [H_CMMMid(1:3,1:3),zeros(3,1);0,0,0,1];

H_B1M1new = H_B1M1*H_M1M1new;
H_M1newB1 = inv(H_B1M1new)

T_M1newB1 = H_M1newB1(1:3,4)'*1000;
eul1 = rotm2eul(H_M1newB1(1:3,1:3));
eul1 = flip(eul1);



H_B2M2new = H_B2M2*H_M2M2new;
H_M2newB2 = inv(H_B2M2new)

T_M2newB2 = H_M2newB2(1:3,4)'*1000;
eul2 = rotm2eul(H_M2newB2(1:3,1:3));
eul2 = flip(eul2);

H_B1M1 = H_B1M1new;
H_B2M2 = H_B2M2new;
save("iteration.mat","H_B1M1","H_B2M2")
