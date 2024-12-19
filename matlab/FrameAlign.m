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

function [H_WB1,H_WB2] = FrameAlign(H_B1B2)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
vec = (H_B1B2(1:3,4));
vec = vec/norm(vec);
t_new_world = 0.5 * H_B1B2(1:3,4);

unit_y = -vec;
z = (0.5 * ([0;0;1]+H_B1B2(1:3,3)));
unit_z = z/norm(z);
z = (unit_z - dot(unit_z,vec)*vec);
unit_z = z/norm(z);
x = (cross(unit_y,unit_z)); 
unit_x = x/norm(x);


H_B1W1 = [unit_x,unit_y,unit_z,t_new_world; 0,0,0,1];
H_WB1 = inv(H_B1W1);
H_WB2 = H_WB1*H_B1B2;
H_B2W2 = inv(H_WB2);
end