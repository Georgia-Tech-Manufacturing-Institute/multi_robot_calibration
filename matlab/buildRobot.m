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
function [H60] = buildRobot(init,q)
    % Assume errors may exist only where manufacturer states explicit
    % offsets exist
    t1 = [0, 0, init(1)];
    t2 = [init(2), 0, 0];
    t3 = [0, init(3), init(4)];
    t4 = [0, 0, init(5)];
    t5 = [init(6), init(7), 0];
    t6 = [init(8), 0, 0];
    % Assume errors may exist everywhere
    % t1 = [init(1), init(2), init(3)];
    % t2 = [init(4), init(5), init(6)];
    % t3 = [init(7), init(8), init(9)];
    % t4 = [init(10), init(11), init(12)];
    % t5 = [init(13), init(14), init(15)];
    % t6 = [init(16), init(17), init(18)];
    
    eul1 = [0, 0, q(1)];
    eul2 = [0, q(2), 0];
    eul3 = [0, q(3), 0];
    eul4 = [q(4), 0, 0];
    eul5 = [0, q(5), 0];
    eul6 = [q(6), 0, 0];
    
    H01 = se3(eul1,"eul","XYZ",t1);
    H12 = se3(eul2,"eul","XYZ",t2);
    H23 = se3(eul3,"eul","XYZ",t3);
    H34 = se3(eul4,"eul","XYZ",t4);
    H45 = se3(eul5,"eul","XYZ",t5);
    H56 = se3(eul6,"eul","XYZ",t6);
    
    H60 = inv(H01*H12*H23*H34*H45*H56);

end