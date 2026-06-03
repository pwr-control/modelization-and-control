clear all
close all
clc

options = bodeoptions;
options.FreqUnits = 'Hz';

J1 = 0.025;
J2 = J1/sqrt(3);
J3 = J1;
b1 = 0.01;
b2 = 0.01;
b3 = 0.01;
b_theta = 0;
k_theta = 1;

f0_msm = 1/2/pi*sqrt(k_theta*(J1+J2)/(J1*J2))

f0_msmsm = 1/2/pi/sqrt(2)/J1/J2/J3*sqrt(-J1^2*J2^2*J3*k_theta - 2*J1^2*J2*J3^2*k_theta ...
    -J1*J2^2*J3^2*k_theta - J1*J2*J3*sqrt(J1^2*J2^2-2*J1*J2^2*J3+4*J1^2*J3^2+J2^2*J3^2)*k_theta)

f1_msmsm = 1/2/pi/sqrt(2)/J1/J2/J3*sqrt(-J1^2*J2^2*J3*k_theta - 2*J1^2*J2*J3^2*k_theta ...
    -J1*J2^2*J3^2*k_theta + J1*J2*J3*sqrt(J1^2*J2^2-2*J1*J2^2*J3+4*J1^2*J3^2+J2^2*J3^2)*k_theta)

Ts = 1/1e3;
Tcontrol=Ts;
Tc = Ts;
simlength = 30;
Nc=floor(simlength/Tc);
load_torque = 10;

s=tf('s');
z=tf('z',Ts);

A = [-(b1+b_theta)/J1 b_theta/J1 0 -1/J1 0; ...
    b_theta/J2 -(b2+b_theta)/J2 b_theta/J2 1/J2 -1/J2;...
    0 b_theta/J3 -(b3+b_theta)/J3 0 1/J3;...
    k_theta -k_theta 0 0 0; ...
    0 k_theta -k_theta 0 0];
B = [1/J1 0 0 0 0]';
E = [0 0 1/J2 0 0]';
C = [1 0 0 0 0];

%% sf
Q=[Ts 0 0 0 0; 0 Ts 0 0 0; 0 0 Ts 0 0; ...
    0 0 0 1 0; 0 0 0 0 1];
R=Ts;
K = lqr(A,B,Q,R)





