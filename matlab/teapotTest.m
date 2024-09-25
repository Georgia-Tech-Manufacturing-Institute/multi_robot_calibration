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

%% Teapot Test Environment
clear, clc, close all, format compact
load matlab.mat
V1_cloud = ptCloud;

eul1_true = [4,37,2];
trans1_true = [-15,-8,6];
eul2_true = [43,60,150];
trans2_true = [13,-10,3];

tform1 = rigidtform3d(eul1_true,trans1_true);
tform2 = rigidtform3d(eul2_true,trans2_true);
W1_cloud = pctransform(V1_cloud,tform1);
W2_cloud = pctransform(V1_cloud,tform2);

figure(1)
hold on
pcshow(V1_cloud)
pcshow(W1_cloud)
pcshow(W2_cloud)

%% pcregistericp method
H_W1V1_vision = pcregistericp(V1_cloud,W1_cloud);
H_W2V1_vision = pcregistericp(V1_cloud,W2_cloud);

%% Plot Stuff!
H1 = double(H_W1V1_vision.A);
H2 = double(H_W2V1_vision.A);

figure(2)
hold on
ax1 = plotTransforms(se3(eye(4,4)),'FrameAxisLabels',"on","FrameLabel","Origin","FrameSize",3);
ax2 = plotTransforms(se3(H1),'FrameAxisLabels',"on","FrameLabel","W1","FrameSize",3);
ax3 = plotTransforms(se3(H2),'FrameAxisLabels',"on","FrameLabel","W2","FrameSize",3);
axis padded
axis equal
grid on
view(160, 30)
title('Vision Point Registration')

%% SVD Method
V1_cloud = double(transpose(V1_cloud.Location(:,:)));
W1_cloud = double(transpose(W1_cloud.Location(:,:)));
W2_cloud = double(transpose(W2_cloud.Location(:,:)));

H_W1V1_SVD = CloudReg(V1_cloud,W1_cloud);
H_W2V1_SVD = CloudReg(V1_cloud,W2_cloud);

%% Plot Stuff!
figure(3)
hold on
ax1 = plotTransforms(se3(eye(4,4)),'FrameAxisLabels',"on","FrameLabel","Origin","FrameSize",3);
ax2 = plotTransforms(se3(H_W1V1_SVD),'FrameAxisLabels',"on","FrameLabel","W1","FrameSize",3);
ax3 = plotTransforms(se3(H_W2V1_SVD),'FrameAxisLabels',"on","FrameLabel","W2","FrameSize",3);
axis padded
axis equal
grid on
view(160, 30)
title('SVD Point Registration')

%% Confirm numbers
eul1_vision = flip(double(rad2deg(rotm2eul(H_W1V1_vision.A(1:3,1:3)))))
trans1_vision = double(H_W1V1_vision.A(1:3,4))
eul2_vision = flip(double(rad2deg(rotm2eul(H_W2V1_vision.A(1:3,1:3)))))
trans2_vision = double(H_W2V1_vision.A(1:3,4))

eul1_AMPF = flip(double(rad2deg(rotm2eul(H_W1V1_SVD(1:3,1:3)))))
trans1_AMPF = double(H_W1V1_SVD(1:3,4))
eul2_AMPF = flip(double(rad2deg(rotm2eul(H_W2V1_SVD(1:3,1:3)))))
trans2_AMPF = double(H_W2V1_SVD(1:3,4))

%% Easy Errors
eul1_vision_error = norm(eul1_true-eul1_vision)
trans1_vision_error = norm(trans1_true-transpose(trans1_vision))
eul2_vision_error = norm(eul2_true-eul2_vision)
trans2_vision_error = norm(trans2_true-transpose(trans2_vision))

eul1_AMPF_error = norm(eul1_true-eul1_AMPF)
trans1_AMPF_error = norm(trans1_true-transpose(trans1_AMPF))
eul2_AMPF_error = norm(eul2_true-eul2_AMPF)
trans2_AMPF_error = norm(trans2_true-transpose(trans2_AMPF))