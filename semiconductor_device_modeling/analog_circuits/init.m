clear all
close all
clc
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
beep off 
s = tf('s');

ts = 1e-6;
simlength = 1;
t_measure = simlength;
N = floor(t_measure/ts);

udc_m = 660;
uph_m = 230*sqrt(2);

Rbias_1 = 300;
Rbias_2 = 300;
u_dd_1 = 15;
u_ss_1 = -15;
u_dd_2 = 3.6;
u_ss_2 = -0.5;

R1 = 1.2e6;
R2 = 402;
C = 100e-9;
Hf1 = 1/(s*R2*C + 1);
Hf2 = R2/(s*R1*R2*C + R1 + R2);
figure; bode(Hf1,options); grid on
figure; bode(Hf2,options); grid on