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

function [H_CMMBase] = Cmm2Robot(V,W,robot,config)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%% Find Rigid Transformation matrix between tool and CMM
% Cite: cs.hunter.edu/~ioannis/registerpts_allen_notes.pdf
A = V; % tool data
B = W; % CMM data

centroid_A = [mean([A(1,:)]); mean([A(2,:)]); mean([A(3,:)])]; % centroid of A
centroid_B = [mean([B(1,:)]); mean([B(2,:)]); mean([B(3,:)])]; % cetroid of A

H = (A-centroid_A)*transpose(B-centroid_B);

[U,S,V] = svd(H);

R_AB = V*transpose(U);

% account for special reflection Citation: nghiaho.com Finding optimal
% Rotation and Translation Between Corresponding 3D points
if det(R_AB) < 0
    [U,S,V] = svd(H);
    V(1:3,3) = V(1:3,3)*-1;
    R_AB = V*transpose(U);
end
t_AB = centroid_B - R_AB*centroid_A;

H_J6CMM = [R_AB, t_AB; 0, 0, 0, 1];

H_CMMBase = H_J6CMM*getTransform(robot,config,"base","link_6");
end
