clear all
close all
clc

ts = 1e-4;
T = 20e-3;
N = floor(T/ts);
time = [0:ts:(N-1)*ts];
theta = 2*pi/T*time;
t1_sv = zeros(1,N);
t2_sv = zeros(1,N);
t0_sv = zeros(1,N);
d1 = zeros(1,N);
d2 = zeros(1,N);
d3 = zeros(1,N);

alpha = sqrt(2)*cos(theta);
beta = sqrt(2)*sin(theta);
for i = 1:N
    [t1_sv(:,i), t2_sv(:,i), t0_sv(:,i), ...
        d1(:,i), d2(:,i), d3(:,i)] = ...
        three_level_sv(alpha(i), beta(i));
end

figure;
plot(time,alpha,time,beta);
grid on

figure;
plot(time, d1, time, d2, time, d3);
grid on