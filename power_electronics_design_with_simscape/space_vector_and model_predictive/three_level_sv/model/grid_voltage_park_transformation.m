clear all
close all
clc

T = 20e-3;
ts = 1e-4;
Ns = floor(T/ts);
time = [0:ts:(Ns-1)*ts];
theta = 2*pi/T*time;
phi = pi/4;
uu = cos(theta - phi); 
uv = cos(theta-2*pi/3 - phi); 
uw = cos(theta+2*pi/3 - phi); 

figure; plot(time,uu, time,uv,time,uw,'LineWidth',2); grid on

Tuvw_ab = 2/3*[1, -1/2, -1/2; 0, sqrt(3)/2, -sqrt(3)/2];
Tab_dq = [cos(theta), sin(theta); -sin(theta), cos(theta)];

ua = 2/3*(uu -1/2*uv -1/2*uw);
ub = 2/3*(sqrt(3)/2*uv -sqrt(3)/2*uw);

figure; plot(time,ua, time,ub,'LineWidth',2); grid on

ud = cos(theta).*ua + sin(theta).*ub;
uq = -sin(theta).*ua + cos(theta).*ub;

figure; plot(time,ud, time,uq,'LineWidth',2);
legend('ud','uq');
grid on



