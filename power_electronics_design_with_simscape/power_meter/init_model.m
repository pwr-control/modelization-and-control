%[text] ## Settings for simulink model initialization and data analysis
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 1.25;
transmission_delay = 125e-6*2;
model = 'iec_power_meter';
load_step_time = 1.25;
%[text] ## Grid Emulator Settings
grid_emulator;
vp_xi_pu = 1;
vn_xi_pu = 0.0;
vn_eta_pu = 0.0;
ip_xi_pu = 1;
ip_eta_pu = 0.5;
in_xi_pu = 0.0;
in_eta_pu = 0.0;
%%
%[text] ## AFE Settings and Initialization
%[text] ### Switching frequencies, sampling time and deadtime
fPWM_AFE = 4e3;
tPWM_AFE = 1/fPWM_AFE;

dead_time_AFE = 3e-6;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;

ts_afe = 1/fPWM_AFE/2; % Sampling time of the control AFE as well as INVERTER
tc = ts_afe/200;

s=tf('s');
z_afe=tf('z',ts_afe);

t_misura = simlength - 0.025;
Nc = ceil(t_misura/tc);
Ns_afe = ceil(t_misura/ts_afe);

time_gain_afe_module_1 = 1.0;
time_gain_inv_module_1 = 1.0;
time_gain_afe_module_2 = 1.0;
time_gain_inv_module_2 = 1.0;
wnp = 0;
white_noise_power_afe_mod1 = wnp;
white_noise_power_inv_mod1 = wnp;
white_noise_power_afe_mod2 = wnp;
white_noise_power_inv_mod2 = wnp;

trgo_th_generator = 0.025;

afe_pwm_phase_shift_mod1 = 0;
white_noise_power_afe_pwm_phase_shift_mod1 = 0.0;
inv_pwm_phase_shift_mod1 = 0;
white_noise_power_inv_pwm_phase_shift_mod1 = 0.0;

afe_pwm_phase_shift_mod2 = 0;
white_noise_power_afe_pwm_phase_shift_mod2 = 0.0;
inv_pwm_phase_shift_mod2 = 0;
white_noise_power_inv_pwm_phase_shift_mod2 = 0.0;
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = rms_perios/f_grid/ts_afe;
rms_perios = 10;
n10 = rms_perios/f_grid/ts_afe;
%[text] ## C-Caller Settings
open_system(model);
Simulink.importExternalCTypes(model,'Names',{'dsmavgflt_output_t'});
%%
%[text] ### 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":22.3}
%---
