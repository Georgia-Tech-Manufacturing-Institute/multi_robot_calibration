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
%% Testing environement, quick robot transforms
function [H60] = buildRobot(dh,q)
    % Assume errors may exist only where manufacturer states explicit
    % offsets exist
    % t1 = [0, 0, init(1)];
    % t2 = [init(2), 0, 0];
    % t3 = [0, init(3), init(4)];
    % t4 = [0, 0, init(5)];
    % t5 = [init(6), init(7), 0];
    % t6 = [init(8), 0, 0];
    % Assume errors may exist everywhere
    alpha = [dh(1,1),dh(2,1),dh(3,1),dh(4,1),dh(5,1),dh(6,1)];
    a =     [dh(1,2),dh(2,2),dh(3,2),dh(4,2),dh(5,2),dh(6,2)];
    theta = [dh(1,3),dh(2,3),dh(3,3),dh(4,3),dh(5,3),dh(6,3)];
    d =     [dh(1,4),dh(2,4),dh(3,4),dh(4,4),dh(5,4),dh(6,4)];
% dh method
for i = 1:1:height(dh)
    H(1:4,1:4,i) = [cos(theta(i)), -sin(theta(i))*cos(alpha(i)),  sin(theta(i))*sin(alpha(i)), a(i)*cos(theta(i));
                    sin(theta(i)),  cos(theta(i))*cos(alpha(i)), -cos(theta(i))*sin(alpha(i)), a(i)*sin(theta(i));
                            0    ,          sin(alpha(i))      ,          cos(alpha(i))      ,          d(i)     ;
                            0    ,                  0          ,                  0          ,            1      ;];
end

% mdh method
% for i = 2:1:height(dh)
%     H(1:4,1:4,i) = [                cos(theta(i)),                 -sin(theta(i)),                0,               a(i-1);
%                     sin(theta(i))*cos(alpha(i-1)),  cos(theta(i))*cos(alpha(i-1)), -sin(alpha(i-1)), -d(i)*sin(alpha(i-1));
%                     sin(theta(i))*sin(alpha(i-1)),  cos(theta(i))*sin(alpha(i-1)),  cos(alpha(i-1)),  d(i)*cos(alpha(i-1));
%                                                 0,                              0,                0,                     1;];
% end

H06 = H(:,:,2)*H(:,:,3)*H(:,:,4)*H(:,:,5)*H(:,:,6);
H06 = round(H06,3)
H60 = inv(H06)
end