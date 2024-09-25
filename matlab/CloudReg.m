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

function [H_BA] = CloudReg(A,B)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% Inputs:
%       V: Tool points in the J6 frame of the Tormach ZA6
%       W: Tool points in the CMM frame
%       robot: rigidBodyTree of the tormach ZA6, used for configurations
%       config: joint angles of the given robot
% Outputs:
%       H_CMMBase: Homogeneous transformation matrix from the CMM to the
%       Robot base
%% Find Rigid Transformation matrix between tool and CMM
% Cite: cs.hunter.edu/~ioannis/registerpts_allen_notes.pdf
% A = V; % tool data
% B = W; % CMM data

centroid_A = [mean([A(1,:)]); mean([A(2,:)]); mean([A(3,:)])]; % centroid of A
centroid_B = [mean([B(1,:)]); mean([B(2,:)]); mean([B(3,:)])]; % cetroid of B

H = (A-centroid_A)*transpose(B-centroid_B); % A,B Covariance Matrix

[U,S,V] = svd(H);

R_BA = V*transpose(U);

% account for special reflection Citation: nghiaho.com Finding optimal
% Rotation and Translation Between Corresponding 3D points
if det(R_BA) < 0
    [U,S,V] = svd(H);
    V(1:3,3) = V(1:3,3)*-1;
    R_BA = V*transpose(U);
end
t_BA = centroid_B - R_BA*centroid_A;

H_BA = [R_BA, t_BA; 0, 0, 0, 1];
end
