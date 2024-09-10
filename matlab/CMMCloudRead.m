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

function [W1,W2] = CMMCloudRead(cloud,n)
%   Automatically imports data from CMM
indices = find(cloud{:,2}==1);

Wx = zeros(n,1);
Wy = zeros(n,1);
Wz = zeros(n,1);

for i=1:1:n-1
    Wx(i) = mean(cloud{indices(i):1:(indices(i+1)-1),4});
    Wy(i) = mean(cloud{indices(i):1:(indices(i+1)-1),5});
    Wz(i) = mean(cloud{indices(i):1:(indices(i+1)-1),6});
end

if height(indices)==height(cloud)
    Wx(n) = mean(cloud{indices(n):1:indices(n),4});
    Wy(n) = mean(cloud{indices(n):1:indices(n),5});
    Wz(n) = mean(cloud{indices(n):1:indices(n),6});
else
    Wx(n) = mean(cloud{indices(n-1):1:indices(n),4});
    Wy(n) = mean(cloud{indices(n-1):1:indices(n),5});
    Wz(n) = mean(cloud{indices(n-1):1:indices(n),6});
end

W1 = [Wx(1:n/2)'; Wy(1:n/2)'; Wz(1:n/2)']/1000;
W2 = [Wx(n/2+1:n)'; Wy(n/2+1:n)'; Wz(n/2+1:n)']/1000;

end