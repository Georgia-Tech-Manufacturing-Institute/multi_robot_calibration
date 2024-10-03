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

addpath('../robot_data/za_description/urdf')
addpath('../CMM_data/40PointCloudRaw')
addpath('../CMM_data/ToolCloud')
addpath('/Users/andrewschneider/GaTech Dropbox/Andrew Schneider/calibration_data/robot_calib_data_925')
cloud = readtable("rob1_self_transform_092524_2.csv", "VariableNamingRule","preserve");

q1 = [0, 21, 45, 0, -66, 0];
q2 = [-15, 50, 30, 80, 90, -180];

q1 = deg2rad(q1);
q2 = deg2rad(q2);

%% Measured CMM Points
n = 40;
[W1_CMM,W2_CMM] = CMMCloudRead(cloud,n);
%% Designed tool points
% 3x20 tool frame coords
tool_cloud = readtable("tool_cloud.csv", "VariableNamingRule","preserve");
V1_J6 = transpose(tool_cloud{:,4:6}/1000);

%% Configure Robot 1  
robot1 = buildRobot(init);
config1 = homeConfiguration(robot1);
[config1.JointPosition] = deal([q1(1)],[q1(2)],[q1(3)],[q1(4)],[q1(5)],[q1(6)]);

%% Configure Robot 2 
robot2 = buildRobot(init);
config2 = homeConfiguration(robot2);
[config2.JointPosition] = deal([q2(1)],[q2(2)],[q2(3)],[q2(4)],[q2(5)],[q2(6)]);

%% Find base transforms
H_CMMJ61 = CloudReg(V1_J6,W1_CMM); % rad, m
H_J6Base1 = getTransform(robot1,config1,"base","link_6");
H_CMMB1 = H_CMMJ61*H_J6Base1;

H_CMMJ62 = CloudReg(V1_J6,W2_CMM); % rad, m
H_J6Base2 = getTransform(robot2,config2,"base","link_6");
H_CMMB2 = H_CMMJ62*H_J6Base2;

%% Cost function
c = norm(H_CMMB1(1:3,4)-H_CMMB2(1:3,4));
end