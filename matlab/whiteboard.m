%% whiteboard HTM math
clear, clc, close all, format compact

eul1 = eul2rotm(deg2rad([-.245, .332, 90.030]),'XYZ');
t1 = [721.377, -32.889, -53.242]';

H_BU = [eul1,t1;0,0,0,1];
H_WB = [0.04185, 0.9991, -0.001767, 0;
  -0.9991, 0.04185, -0.002313,734.3;
  -0.002237, 0.001863, 1, 0;
   0, 0, 0, 1;];

H_WU = H_WB*H_BU;

t = H_WU(1:3,4)'/1000;
fprintf('%0.6f\n',t)

rot = rotm2eul(H_WU(1:3,1:3),"XYZ")