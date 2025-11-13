clear all
close all
clc

T = 20e-3;
ts = 1e-4;
Ns = floor(T/ts);
time = [0:ts:(Ns-1)*ts];
theta = 2*pi/T*time;

phi = 0;

uu = -sin(theta - phi); 
uv = -sin(theta - 2*pi/3 - phi); 
uw = -sin(theta + 2*pi/3 - phi); 

figure; plot(time,uu, time,uv,time,uw,'LineWidth',2); 
legend('uu','uv','uw');
grid on

Tuvw_ab = 2/3*[1, -1/2, -1/2; 0, sqrt(3)/2, -sqrt(3)/2];
pinv(Tuvw_ab)
Tab_dq = [cos(theta), sin(theta); -sin(theta), cos(theta)];

ualpha = 2/3*(uu -1/2*uv -1/2*uw);
ubeta = 2/3*(sqrt(3)/2*uv -sqrt(3)/2*uw);

figure; plot(time,ualpha, time,ubeta,'LineWidth',2); 
legend('ualpha','ubeta');
grid on

ud = cos(theta).*ualpha + sin(theta).*ubeta;
uq = -sin(theta).*ualpha + cos(theta).*ubeta;

figure; plot(time,ud, time,uq,'LineWidth',2);
legend('ud','uq');
grid on

ud_control = 0.3;
uq_control = -0.8;

ualpha_control = cos(theta).*ud_control - sin(theta).*uq_control;
ubeta_control = sin(theta).*ud_control + cos(theta).*uq_control;

figure; plot(time,ualpha_control, time,ubeta_control,'LineWidth',2); 
legend('ualpha_ctrl','ubeta_ctrl');
grid on