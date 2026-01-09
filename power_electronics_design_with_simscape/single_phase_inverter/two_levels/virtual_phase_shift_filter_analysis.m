clear all
close all
clc


ts = 1e-4;
time = [0:ts:40e-3-ts];
phase = 2*pi*50*time;

V1 = cos(phase);
V2 = cos(phase - 2*pi/3);
V3 = cos(phase - 4*pi/3);

Valpha = 2/3*(V1 -0.5*V2 -0.5*V3);
Vbeta = 2/3*(sqrt(3)/2*V2 -sqrt(3)/2*V3);

figure; plot(time, Valpha, time, Vbeta); grid on
legend('\alpha','\beta');

