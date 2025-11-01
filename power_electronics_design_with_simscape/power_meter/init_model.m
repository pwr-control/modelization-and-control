%[text] ## Settings for simulink model initialization and data analysis
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 0.25;
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
Simulink.importExternalCTypes(model,'Names',{'mavgflts_output_t'}); %[output:627d9798] %[output:151a7add] %[output:7255b242] %[output:13f0e439] %[output:1f4050a0]
%%
%[text] ### 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":41.3}
%---
%[output:627d9798]
%   data: {"dataType":"warning","outputData":{"text":"Warning: The file containing block diagram '<a href=\"matlab:open_system ('three_phase_pwm_modulator')\">three_phase_pwm_modulator<\/a>' has been changed on disk since it was loaded. You should close it, and Simulink will reload it if necessary."}}
%---
%[output:151a7add]
%   data: {"dataType":"warning","outputData":{"text":"Warning: The file containing block diagram '<a href=\"matlab:open_system ('single_phase_pwm_modulator')\">single_phase_pwm_modulator<\/a>' has been changed on disk since it was loaded. You should close it, and Simulink will reload it if necessary."}}
%---
%[output:7255b242]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Error resolving Custom Code."}}
%---
%[output:13f0e439]
%   data: {"dataType":"warning","outputData":{"text":"Warning: C:\\Git\\GitHub\\modelization-and-control\\power_electronics_design_with_simscape\\power_meter\\include specified in custom include directory paths string does not exist in any of the following search directories:\n\""}}
%---
%[output:1f4050a0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: C:\\Git\\GitHub\\modelization-and-control\\power_electronics_design_with_simscape\\power_meter\\src specified in custom include directory paths string does not exist in any of the following search directories:\n\""}}
%---
