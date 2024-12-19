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
% load("Base2Middle.mat")
% load("iteration0.mat")
% load("iteration1.mat")
load("iteration2.mat")
%%
addpath('/Users/andrewschneider/GaTech Dropbox/Andrew Schneider/calibration_data/IterativeMethod_1219')
% cloud = readtable("relative_trans0_20241216.csv", "VariableNamingRule","preserve");
% cloud = readtable("relative_trans1_20241219.csv", "VariableNamingRule","preserve");
% cloud = readtable("relative_trans2_20241219.csv", "VariableNamingRule","preserve");
cloud = readtable("relative_trans3_20241219.csv", "VariableNamingRule","preserve");


l = .2; w = .2; h = .2;
l2 = l/2; w2 = w/2; h2 = h/2;

V = [l2,  w2,  h2;
     l2, -w2,  h2;
    -l2, -w2,  h2;
    -l2,  w2,  h2;
    -l2,  w2, -h2;
     l2,  w2, -h2;
     l2, -w2, -h2;
    -l2, -w2, -h2;];
V = transpose(V+[0,0,0.2]);

W = cloud{1:end,4:end}';
n = length(W);

A_raw = [W(:,1:n/2)]/1000;
B_raw = [W(:,n/2+1:n)]/1000;

for i = 1:1:8
    A(:,i) = mean(A_raw(:,6*(i-1)+1:6*i)')';
    B(:,i) = mean(B_raw(:,6*(i-1)+1:6*i)')';
end

pointError = vecnorm(A-B);
pointAvgErro = mean(pointError)*1000
stddev = std(pointError)*1000
NormError = norm(vecnorm(A-B))*1000
SumError = sum(vecnorm(A-B))*1000

figure(1)
plot3(A(1,:)',A(2,:)',A(3,:)',"-.b")
hold on
plot3(B(1,:)',B(2,:)',B(3,:)',"-.r")

H_V1A = CloudReg(A,V); % rad, m
H_V2B = CloudReg(B,V); % rad, m

H_V1V2 = H_V1A*inv(H_V2B);
H_W1W2 = H_V1V2;

T_BA = H_W1W2(1:3,4);

T_W1Wnew = 0.5*T_BA;

q0 = quaternion([0,0,0],"eulerd","XYZ","frame");
q_BA = quaternion(tform2quat(H_W1W2));

q_W1Wnew = slerp(q0,q_BA,0.5);
q_W2Wnew = slerp(q_BA,q0,0.5);

R_W1Wnew = quat2rotm(q_W1Wnew);


H_W1Wnew = [R_W1Wnew,T_W1Wnew;0,0,0,1];
H_W2Wnew =  inv(H_W1W2)*H_W1Wnew;

H_B1Wnew = H_B1W*H_W1Wnew;

H_B2Wnew = H_B2W*H_W2Wnew;

H_B1W = H_B1Wnew;
H_B2W = H_B2Wnew;

H_B1B2 = H_B1W*inv(H_B2W);

[H_WB1,H_WB2] = FrameAlign(H_B1B2);

T_W1B1 = round(H_WB1(1:3,4)'*1000,6)
eul1 = rotm2eul(H_WB1(1:3,1:3));
eul1 = round(flip(eul1),6)

T_W2B2 = round(H_WB2(1:3,4)'*1000,6)
eul2 = rotm2eul(H_WB2(1:3,1:3));
eul2 = round(flip(eul2),6)

H_B1W = inv(H_WB1);
H_B2W = inv(H_WB2);
save("iteration3.mat","H_B1W","H_B2W")

%% Plot Stuff!

figure(2)
hold on
ax1 = plotTransforms(se3(inv(H_B1Wnew)),'FrameAxisLabels',"on","FrameLabel","Robot 1","FrameSize",.3);
ax2 = plotTransforms(se3(inv(H_B2Wnew)),'FrameAxisLabels',"on","FrameLabel","Robot 2","FrameSize",.3);
ax3 = plotTransforms(se3(inv(H_W1Wnew)),'FrameAxisLabels',"on","FrameLabel","World1","FrameSize",.1);
ax4 = plotTransforms(se3(inv(H_W2Wnew)),'FrameAxisLabels',"on","FrameLabel","World2","FrameSize",.1);



xlabel('x')
ylabel('y')
zlabel('z')

axis equal
grid on
view(160, 30)

