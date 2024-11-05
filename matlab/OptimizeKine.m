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
%% Fminsearch
% % init = [.45, .025, -.001, .454, .035, .4195, .001, .1175];
% init = [0, 0, .45, .025, 0, 0, 0, -.001, .454, 0, 0, .035, .4195, .001, 0, .1175, 0, 0];
% init_g = init-0.001;
% 
% fun = @centroidCost;
% options = optimset('Display','iter','PlotFcns',@optimplotfval,'TolFun', 1.e-12);
% [x,fval,exitflag,output] = fminsearch(fun,init_g,options);
% init
% x

%% Particle Swarm
% init = [0, 0, .45, .025, 0, 0, 0, -.001, .454, 0, 0, .035, .4195, .001, 0, .1175, 0, 0];
% init = init-.001;
% lb = init-.005;
% ub = init+.005;
% 
% fun = @centroidCost;
% nvars = length(init);
% 
% options = optimoptions('particleswarm','Display','iter','PlotFcn', 'pswplotbestf','UseParallel',true);
% 
% [x,fval,exitflag,output,points] = particleswarm(fun,nvars,lb,ub,options);
% init
% x
%% FminCon
init = [0, 0, .45, .025, 0, 0, 0, -.001, .454, 0, 0, .035, .4195, .001, 0, .1175, 0, 0];
init_true = init+.0015;
lb = init-.005;
ub = init+.005;
% https://pmc.ncbi.nlm.nih.gov/articles/PMC5059574/ upper/lower bound
% selection citation

fun = @centroidCost;
options = optimoptions('fmincon','Display','iter','PlotFcn','optimplot','UseParallel',false,'ObjectiveLimit',1e-12,'OptimalityTolerance',1e-12,'StepTolerance',1e-30);
x = fmincon(fun,init,[],[],[],[],lb,ub,[],options);
init_true
x

save("linkLengths.mat","x")