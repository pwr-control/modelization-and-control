clear all
close all
clc

tc = 1e-10;

%% double pulse test
delta_time_first_pulse = 100e-6;
delta_time_discharge = 25e-6;
delta_time_second_pulse = 50e-6;
deadtime = 3e-6;

t0_dpt = 5e-6;
t1_dpt = t0_dpt + delta_time_first_pulse;
t2_dpt = t1_dpt + deadtime;
t3_dpt = t2_dpt + delta_time_discharge;
t4_dpt = t3_dpt + deadtime;
t5_dpt = t4_dpt + delta_time_second_pulse;
t6_dpt = t5_dpt + deadtime;

Vgate_h = 15;
Vgate_l = -10;
Vdc = 750;

%% parameters which match the first test
Rgate_on = 6.2/5;
Rgate_off = 18/5;
Rgate_internal = 4;
Cies = 54e-9;
Cres = 3e-9;
Coes = 10e-9;
Lstray_dclink = 10e-9;
Lstray_module = 10e-9;
Rc_module = 0.3e-4;
RLstray_module = 0.3e-4;
RLstray_dclink = 0.3e-4;

Cge = Cies-Cres;
Cgc = Cres;
Cce = Coes-Cres;

RCgc = 1e-4;
RCge = 1e-4;
RCce = 1e-4;

simlength = 200e-6;
Nc = floor(simlength/tc);

% open_system('gate_driver_circuit.slx');
open_system('gate_driver_circuit.slx');

