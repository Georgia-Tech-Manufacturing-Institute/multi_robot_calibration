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
%% Simulate CMM data of pose and end effector position
% confirmed validity by calculating an identity matrix for the base to base
% transformation of two poses
clear, clc, close all, format compact
%% Designed tool points
% 3x20 tool frame coords
addpath('../CMM_data/ToolCloud')
tool_cloud = readtable("tool_cloud.csv", "VariableNamingRule","preserve");
V_J6 = transpose(tool_cloud{:,4:6}/1000);
V_J6(4,:) = ones(1,length(V_J6));

H_CMMBase = [1 0 0 -.813;
             0 1 0 -.146;
             0 0 1    0 ;
             0 0 0    1 ;];
%% 
% only explicit offsets
% init = [.45, .025, -.001, .454, .035, .4195, .001, .1175];

% all offsets
init = [0, 0, .45, .025, 0, 0, 0, -.001, .454, 0, 0, .035, .4195, .001, 0, .1175, 0, 0];
error = .0015;
init_real = init+error;

%% random poses
n = 2000;
q = zeros(n,6);
for i= 1:1:n
    q(i,:) = rand(6,1)*2*pi;
    H_J6Base.A{i} = buildRobot(init_real,q(i,:));
    H_BaseJ6.A{i} = inv(H_J6Base.A{i});
    V_CMM = H_CMMBase*tform(H_BaseJ6.A{i})*V_J6;
    W_CMM(:,:,i) = V_CMM(1:3,:);
end



save("q.mat","q")
save("W.mat","W_CMM")
