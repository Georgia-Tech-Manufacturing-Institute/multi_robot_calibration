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

function [c] = centroidCost(init)
%UNTITLED7 Summary of this function goes here
%   Detailed explanation goes here

%% Real Data
% addpath('../robot_data/za_description/urdf')
% addpath('../CMM_data/40PointCloudRaw')
% addpath('../CMM_data/ToolCloud')
% addpath('/Users/andrewschneider/GaTech Dropbox/Andrew Schneider/calibration_data/rob1_kinematic_optimization_104')
% % 
% 
% % Joint Angles
% q(1,:) = [0, 45, 30, 0, -75, 0];
% q(2,:) = [25, 65, 30, 60, -65, 0];
% q(3,:) = [1.141, 35.480, 19.738, -84.379, -45.001, 40.004];
% q(4,:) = [-10.377, 121.931, -114.019, -45.710, -64.364, 20.006];
% q(5,:) = [50, 25, 50, 75, -65, 70];
% q(6,:) = [-25, 35, 40, -50, -112, 140];
% q(7,:) = [40, 35, 40, 85, -30, 150];
% q(8,:) = [165, -99, -70, 85, -118, 5];
% q(9,:) = [-7, 15, 60, -20, -55, -60];
% q(10,:) = [-15, 45, 20, 120, 100, 11];
% q(11,:) = [15, 45, 20, 170, 60, 88];
% 
% q = deg2rad(q);
% 
% % Measured CMM Points
% id = ["Rob1V1.csv";"Rob1V2.csv";"Rob1V3.csv";"Rob1V4.csv";"Rob1V5.csv";"Rob1V6.csv";"Rob1V7.csv";"Rob1V8.csv";"Rob1V9.csv";"Rob1V10.csv";"Rob1V11.csv"];
% 
% for i = 1:1:length(id)
%     W_CMM(:,:,i) = CMMCloudRead(readtable(id(i), "VariableNamingRule","preserve"))./1000;
% end

%% Simulated data
load("q.mat"), load("W.mat")

%% Designed tool points
% 3x20 tool frame coords
tool_cloud = readtable("tool_cloud.csv", "VariableNamingRule","preserve");
V1_J6 = transpose(tool_cloud{:,4:6}/1000);
%% Test environ
for i = 1:1:height(q)
    H_CMMJ6.A{i} = se3(CloudReg(V1_J6,W_CMM(:,:,i)));
    H_J6Base.A{i} = buildRobot(init,q(i,:));
    H_CMMB.A{i} = H_CMMJ6.A{i}*H_J6Base.A{i};
    t(i,:) = trvec(H_CMMB.A{i});
end

%% Cost function
t_mean = mean(t);
cost_t = t_mean-t;
c = norm(vecnorm(transpose(cost_t)));

end