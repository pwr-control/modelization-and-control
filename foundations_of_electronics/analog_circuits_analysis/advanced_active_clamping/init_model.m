clear all
close all
clc

tc = 1e-10;

%% double pulse test
delta_time_first_pulse = 100e-6;
delta_time_discharge = 20e-6;
delta_time_second_pulse = 35e-6;
deadtime = 3e-6;

t0_dpt = 5e-6;
t1_dpt = t0_dpt + delta_time_first_pulse;
t2_dpt = t1_dpt + deadtime;
t3_dpt = t2_dpt + delta_time_discharge;
t4_dpt = t3_dpt + deadtime;
t5_dpt = t4_dpt + delta_time_second_pulse;
t6_dpt = t5_dpt + deadtime;

Vgate_h = 15;
Vgate_l = -7;
% Vgate_h = 5;
% Vgate_l = 0;
% Vdc = 1070;
Vdc = 1400;

Rgate = 2.5;
Rgate_internal = 4;

Lstray_module = 7e-9;
RLstray_module = 1e-4;
Cgc = 100e-12;
Cge = 100e-12;
Cce = 100e-12;
RCgc = 1e-4;
RCge = 1e-4;
RCce = 1e-4;

simlength = 200e-6;

open_system('advanced_active_clamping.slx');

