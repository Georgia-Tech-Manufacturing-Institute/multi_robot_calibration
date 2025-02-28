% Tool Frame Calibration Tool
clear, clc, close all, format compact
x0 = [-.087,.00087,.012];
% x0 = [-0.1583,   -0.0694,   -0.0255]
lb = x0-.05;
ub = x0+.05;

fun = @EE_poses;
% x = lsqnonlin(fun,x0)
% x = EE_poses(x0)
% 
options = optimset('Display','iter','PlotFcns',@optimplotfval);
[x_fminsearch,fval,exitflag,output] = fminsearch(fun,x0,options);

% % % options = optimoptions('fmincon','Display','iter','PlotFcn','optimplot');
% % % x_fmincon = fmincon(fun,x0,[],[],[],[],lb,ub,[],options);
x_fminsearch
% % x_fmincon