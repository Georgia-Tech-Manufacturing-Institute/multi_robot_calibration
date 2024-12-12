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
load("Base2Middle.mat")
cloud = readtable(".csv", "VariableNamingRule","preserve");
n = 16;
W = CMMCloudRead(cloud);

A = [W(:,1:n/2)]/1000;
B = [W(:,n/2+1:n)]/1000;

H_BA = CloudReg(A,B); % rad, m

T_BA = H_BA(1:3,4);

T_M1M1new = 0.5*T_BA;
T_M2M2new = -0.5*T_BA;

q0 = quaternion([0,0,0],"eulerd","XYZ","frame");
q_BA = tform2quat(H_BA);

q_M1M1new = slerp(q0,q_BA,0.5);
q_M2M2new = slerp(q_BA,q0,0.5);

R_M1M1new = quat2rotm(q_M1M1new);
R_M2M2new = quat2rotm(q_M2M2new);

H_M1M1new = se3(R_M1M1new,T_M1M1new);
H_M2M2new = se3(R_M2M2new,T_M2M2new);

H_B1M1new = H_B1M1*H_M1M1new
H_B2M2new = H_B2M2*H_M2M2new
