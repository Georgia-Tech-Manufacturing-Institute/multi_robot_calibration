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
%% Start

clear, clc, close all, format short g, format compact
addpath('../robot_data/za_description/urdf')
addpath('../CMM_data/40PointCloudRaw')
addpath('../CMM_data/ToolCloud')
addpath('/Users/andrewschneider/GaTech Dropbox/Andrew Schneider/calibration_data/subchain_self_transformations/rob1_j6')

%% Function
init = [.45, .025, -.001, .454, .035, .4195, .001, .1175];

fun = @centroidCost;
options = optimset('Display','iter','PlotFcns',@optimplotfval);
[x,fval,exitflag,output] = fminsearch(fun,init,options)
link_1 = [0, 0, x(1)]
link_2 = [x(2), 0, 0]
link_3 = [0, x(3), x(4)]
link_4 = [0, 0, x(5)]
link_5 = [x(6), x(7), 0]
link_6 = [x(8), 0, 0]