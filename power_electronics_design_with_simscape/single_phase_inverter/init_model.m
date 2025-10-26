%[text] ## General Settings
%[text] ### Simulink model initialization
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
simlength = 1;
transmission_delay = 125e-6*2;
s=tf('s');

rpi_enable = 0; % use RPI otherwise DQ PI
system_identification_enable = 0;
%[text] ### Voltage application
application400 = 1;
application690 = 0;
application480 = 0;
%[text] ### PWM and sampling time and data length storage
fPWM = 10e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

double_sampling = 0;

if double_sampling 
    ts_afe = 1/fPWM_AFE/2;
    ts_inv = 1/fPWM_INV/2;
    ts_dab = 1/fPWM_DAB/2;
else
    ts_afe = 1/fPWM_AFE;
    ts_inv = 1/fPWM_INV;
    ts_dab = 1/fPWM_DAB;
end

ts_battery = ts_dab;
tc = ts_inv/100;

z_dab = tf('z',ts_dab);
z_afe = tf('z',ts_afe);
z_inv = tf('z',ts_inv);

t_misura = simlength - 0.2;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

dead_time_INV = 1e-6;
delay_pwm = 0;
%[text] ## Grid Emulator Settings
grid_emulator;
%[text] ## Grid Voltage Rectifier
%[text] ### Nominal DClink voltage seting
if (application690 == 1)
    Vdc_nom = 1070; % DClink voltage reference
elseif (application480 == 1)
    Vdc_nom = 750; % DClink voltage reference
else
    Vdc_nom = 660; % DClink voltage reference
end
%[text] ### 
%[text] ### DClink, and dclink-brake parameters
Vdc_ref = Vdc_nom; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi_dc1 = 1400e-6*4;
RCFi_dc1_internal = 1e-3;
%[text] ### Load Transformer Parameters
m1_load_trafo = 50;
m2_load_trafo = 1;
m12_load_trafo = m1_load_trafo/m2_load_trafo;

ftr_nom_load_trafo = 50;
I0rms_load_trafo = 5;
V1rms_load_trafo = 330;
I1rms_load_trafo = 600;
V2rms_load_trafo = V1rms_load_trafo/m12_load_trafo %[output:4e4d9ab0]
I2rms_load_trafo = I1rms_load_trafo*m12_load_trafo %[output:617b30e4]
lm_load_trafo = V1rms_load_trafo/I0rms_load_trafo/2/pi/ftr_nom_load_trafo;
rfe_load_trafo = 2e3;
rd1_load_trafo = 1e-3;
ld1_load_trafo = 40e-6;
rd2_load_trafo = rd1_load_trafo/m12_load_trafo^2;
ld2_load_trafo = ld1_load_trafo/m12_load_trafo^2;
%[text] ### Current sensor endscale, and quantization
Pnom = 200e3;
margin_factor = 1.25;
adc12_quantization = 1/2^12;
adc11_quantization = 1/2^11;
adc16_quantization = 1/2^16;
adc15_quantization = 1/2^15;
Vdc_FS = Vdc_nom * margin_factor;
Idc_FS = Pnom/Vdc_nom * margin_factor;
Vac_FS = V1rms_load_trafo*sqrt(2) %[output:45db0fb3]
Iac_FS = I1rms_load_trafo*sqrt(2) %[output:19be7e26]
%[text] ### Single phase inverter control
iph_grid_pu_ref = 1;
%[text] ### DClink Lstray model
Lstray_dclink = 100e-9;
RLstray_dclink = 10e-3;
C_HF_Lstray_dclink = 15e-6;
R_HF_Lstray_dclink = 22000;
Z_HF_Lstray_dclink = 1/s/C_HF_Lstray_dclink + R_HF_Lstray_dclink;
Z_LF_Lstray_dclink = s*Lstray_dclink + RLstray_dclink;
Z_Lstray_dclink = Z_HF_Lstray_dclink*Z_LF_Lstray_dclink/(Z_HF_Lstray_dclink+Z_LF_Lstray_dclink);
ZCFi = 7/s/CFi_dc1;
sys_dclink = minreal(ZCFi/(ZCFi+Z_Lstray_dclink));
% figure; bode(sys_dclink,Z_Lstray_dclink,options); grid on
%[text] ### INV current control parameters
%[text] #### DQ PI
kp_inv = 1;
ki_inv = 45;
%[text] #### Phase shift filter for Q component derivation at 50Hz and 80Hz
if system_identification_enable
    freq = 300;
else
    freq = 80;
end
omega_set = 2*pi*freq;

flt_dq = 2/(s/omega_set+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
figure; bode(flt_dq_d, options); grid on %[output:700f1fc7]
[num, den]=tfdata(flt_dq_d,'v') %[output:97e357dd] %[output:9d1224cc]

%[text] #### Resonant PI
kp_rpi = 2;
ki_rpi = 45;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta*omega_set] %[output:26f707e8]
Aresd_nom = eye(2) + Ares_nom*ts_inv %[output:2214b3d6]
a11d = 1 %[output:6e27c0a7]
a12d = ts_inv %[output:7996ad92]
a21d = -omega_set^2*ts_inv %[output:9ad8d2a8]
a22d = 1 -2*delta*omega_set*ts_inv %[output:637faca4]

Bres = [0; 1];
Cres = [0 1];
Bresd = Bres*ts_inv;
Cresd = Cres;
%%
%[text] ### Grid Normalization Factors
Vgrid_phase_normalization_factor = Vphase2*sqrt(2);
pll_i1 = 80;
pll_p = 1;
pll_p_frt = 0.2;
Vmax_ff = 1.1;
Igrid_phase_normalization_factor = 250e3/Vphase2/3/0.9*sqrt(2);
ixi_pos_ref_lim = 1.6;
ieta_pos_ref_lim = 1.0;
ieta_neg_ref_lim = 0.5;
%%
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso_pll = [-1 -4]*omega_rso;
Lrso_pll = acker(Arso',Crso',polesrso_pll)';
Adrso_pll = eye(2) + Arso*ts_afe;
polesdrso_pll = exp(ts_afe*polesrso_pll);
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:61f9eea3]

%[text] ### PLL DDSRF
use_advanced_pll = 0;
use_dq_pll_ccaller = 0;
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:306c88de]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:23fb1bfd]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:11c23dfc]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:4b099645]
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;

kp_rc_pos_grid = 0.35;
ki_rc_pos_grid = 35;
kp_rc_neg_grid = 0.35;
ki_rc_neg_grid = 35;
%%
%[text] ## Settings for user functions: filters, moving average, rms
%[text] ### Low Pass Filters
%[text] #### LPF 50Hz in state space (for initialization)
fcut = 50;
fof = 1/(s/(2*pi*fcut)+1);
[nfof, dfof] = tfdata(fof,'v');
[nfofd, dfofd]=tfdata(c2d(fof,ts_afe),'v');
fof_z = tf(nfofd,dfofd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(nfofd,dfofd);
LVRT_flt_ss = ss(A,B,C,D,ts_afe);
[A,B,C,D] = tf2ss(nfof,dfof);
LVRT_flt_ss_c = ss(A,B,C,D);
%[text] #### LPF 161Hz
fcut_161Hz_flt = 161;
g0_161Hz = fcut_161Hz_flt * ts_afe * 2*pi;
g1_161Hz = 1 - g0_161Hz;
%%
%[text] #### LPF 500Hz
fcut_500Hz_flt = 500;
g0_500Hz = fcut_500Hz_flt * ts_afe * 2*pi;
g1_500Hz = 1 - g0_500Hz;
%%
%[text] #### LPF 75Hz
fcut_75Hz_flt = 75;
g0_75Hz = fcut_75Hz_flt * ts_afe * 2*pi;
g1_75Hz = 1 - g0_75Hz;
%%
%[text] #### LPF 50Hz
fcut_50Hz_flt = 50;
g0_50Hz = fcut_50Hz_flt * ts_afe * 2*pi;
g1_50Hz = 1 - g0_50Hz;
%%
%[text] #### LPF 10Hz
fcut_10Hz_flt = 10;
g0_10Hz = fcut_10Hz_flt * ts_afe * 2*pi;
g1_10Hz = 1 - g0_10Hz;
%%
%[text] #### LPF 4Hz
fcut_4Hz_flt = 4;
g0_4Hz = fcut_4Hz_flt * ts_afe * 2*pi;
g1_4Hz = 1 - g0_4Hz;
%%
%[text] #### LPF 1Hz
fcut_1Hz_flt = 1;
g0_1Hz = fcut_1Hz_flt * ts_afe * 2*pi;
g1_1Hz = 1 - g0_1Hz;
%%
%[text] #### LPF 0.2Hz
fcut_0Hz2_flt = 0.2;
g0_0Hz2 = fcut_0Hz2_flt * ts_afe * 2*pi;
g1_0Hz2 = 1 - g0_0Hz2;
%[text] #### 
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = rms_perios/f_grid/ts_afe;
rms_perios = 10;
n10 = rms_perios/f_grid/ts_afe;
%%
%[text] ### Online time domain sequence calculator
w_grid = 2*pi*f_grid;
apf = (s/w_grid-1)/(s/w_grid+1);
[napfd, dapfd]=tfdata(c2d(apf,ts_afe),'v');
apf_z = tf(napfd,dapfd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(napfd,dapfd);
ap_flt_ss = ss(A,B,C,D,ts_afe);
% figure;
% bode(ap_flt_ss,options);
% grid on
%%
%[text] ### Single phase pll
freq_pll = 50;
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:00aa870f]

freq_filter = freq_pll;
tau_f = 1/2/pi/freq_filter;
Hs = 1/(s*tau_f+1);
Hd = c2d(Hs,ts_inv);
%[text] ### Lithium Ion Battery
ubattery = 600;
Pbattery_nom = 750e3;
typical_cell_voltage = 3.6;
number_of_cells = floor(ubattery/typical_cell_voltage)-1; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 50; %Hr*A

Vbattery_nom = ubattery;
Ibattery_nom = Pbattery_nom/Vbattery_nom;
Rmax = Vbattery_nom^2/(Pbattery_nom*0.1);
Rmin = Vbattery_nom^2/(Pbattery_nom);

E_1 = -1.031;
E0 = 3.485;
E1 = 0.2156;
E2 = 0;
E3 = 0;
Elog = -0.05;
alpha = 35;

R0 = 0.0035;
R1 = 0.0035;
C1 = 0.5;
M = 125;

q1Kalman = ts_inv^2;
q2Kalman = ts_inv^1;
q3Kalman = 0;
rKalman = 1;

Zmodel = (0:1e-3:1);
ocv_model = E_1*exp(-Zmodel*alpha) + E0 + E1*Zmodel + E2*Zmodel.^2 +...
    E3*Zmodel.^3 + Elog*log(1-Zmodel+ts_inv);
figure;  %[output:2e5b507a]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:2e5b507a]
xlabel('state of charge [p.u.]'); %[output:2e5b507a]
ylabel('open circuit voltage [V]'); %[output:2e5b507a]
title('open circuit voltage(state of charge)'); %[output:2e5b507a]
grid on %[output:2e5b507a]

%[text] ## System Identification
%[text] ### Normalization
Ibez = I1rms_load_trafo*sqrt(2);
Ubez = V1rms_load_trafo*sqrt(2);
Xbez = Ubez/Ibez;
%[text] ## IGBT, MOSFET, DIODE and snubber data
%[text] ### Diode rectifier
Vf_diode_rectifier = 0.7;
Rdon_diode_rectifier = 1.85e-3;
%[text] ### HeatSink
weigth = 0.050; % kg
cp_al = 880; % J/K/kg
heat_capacity = cp_al*weigth % J/K %[output:6f3c261a]
thermal_conducibility = 204; % W/(m K)
Rth_mosfet_HA = 30/1000 % K/W %[output:7d5e9062]
Tambient = 40;
DThs_init = 0;
%[text] ### SKM1700MB20R4S2I4
Rds_on = 1.04e-3; % Rds [Ohm]
Vdon_diode = 4; % V
Vgamma = 4; % V
Rdon_diode = 1.85e-3; % V
Eon = 77e-3; % J @ Tj = 125°C
Eoff = 108e-3; % J @ Tj = 125°C
Eerr = 9.7e-3; % J @ Tj = 125°C
Voff_sw_losses = 1300; % V
Ion_sw_losses = 1000; % A

JunctionTermalMass = 2; % J/K
Rtim = 0.05;
Rth_mosfet_JC = 19/1000; % K/W
Rth_mosfet_CH = 6/1000; % K/W
Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH % K/W %[output:71a97270]
Lstray_module = 12e-9;

Irr = 475;
Csnubber = Irr^2*Lstray_module/Vdc_nom^2 %[output:5cd9d49f]
Rsnubber = 1/(Csnubber*fPWM_INV)/5 %[output:60ed39a0]
%[text] ### Load
uload = 3;
rload = uload/I2rms_load_trafo;
%[text] ### FF1000UXTR23T2M1

% Ron = 2.13e-3; % Rds [Ohm]
% Vgamma = 5.5; % V
% Eon = 540e-3; % J @ Tj = 125°C
% Eoff = 370e-3; % J @ Tj = 125°C
% Eerr = 0.075e-3; % J @ Tj = 125°C
% Voff_sw_losses = 1500; % V
% Ion_sw_losses = 2000; % A
% 
% JunctionTermalMass = 0.025; % J/K
% Rtim = 0.05;
% Rth_mosfet_JC = 20/1000; % K/W
% Rth_mosfet_CH = 5.8/1000; % K/W
% Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH % K/W
%[text] ## C-Caller Settings
model = 'single_phase_inverter';
open_system(model);
Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
Simulink.importExternalCTypes(model,'Names',{'rpi_output_t'});

%[text] ## Remove Scopes Opening Automatically
% open_scopes = find_system(model, 'BlockType', 'Scope');
% for i = 1:length(open_scopes)
%     set_param(open_scopes{i}, 'Open', 'off');
% end

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":51.6}
%---
%[output:4e4d9ab0]
%   data: {"dataType":"textualVariable","outputData":{"name":"V2rms_load_trafo","value":"   6.600000000000000"}}
%---
%[output:617b30e4]
%   data: {"dataType":"textualVariable","outputData":{"name":"I2rms_load_trafo","value":"       30000"}}
%---
%[output:45db0fb3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     4.666904755831214e+02"}}
%---
%[output:19be7e26]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     8.485281374238571e+02"}}
%---
%[output:700f1fc7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ9sltd973\/dut44alLbaUXjOMEkMWsqTYRaWV03KtuFlG2p0RW9mnm9SRQ5DNGwVIsZ\/pPoUrbGf1h9t1AySomH0O3ssLVIw5MmVkiHlHjcRoRYU8UGieMQ1w1LwW5alaw3W69+jzkvj1+\/r9\/znOfPOef3fl8pCn59\/n6+v+ecr885z\/O87xe\/+MUvCB8QAAEQAAEQAAEQ8IjA+2BgPFILTQUBEAABEAABEAgIwMAgEEAABEAABEAABLwjAAPjnWRoMAiAAAiAAAiAAAwMYgAEQAAEQAAEQMA7AjAw3kmGBoMACIAACIAACMDAIAZAAARAAARAAAS8IwAD451kaDAIgAAIgAAIgAAMDGIABEAABEAABEDAOwIwMN5JhgaDAAiAAAiAAAjAwCAGQAAEQAAEQAAEvCMAA+OdZGgwCIAACIAACIAADAxiAARAAARAAARAwDsCMDDeSYYGg0ByBC5evEhbtmyhmZmZfKGtra00MDBAVVVVZSu6du0adXd3Uy6Xo+bm5rLpz5w5Q+3t7QvSbdu2jbq6uihqWWUrQwIQAAHRBGBgRMuLzoHA0gTYwOzatYv27t1LjY2NkU1EVNPBBmZwcJCGh4eptrY2X19dXV1gYvABARAAAV0CMDC6pJAOBAQSKGdgwismaqWEMbAJOXjwIK1Zsyagwr\/jFZijR49ST09P\/rtCU1JoYDght6Gvr4++8pWvBEaKV3NWrVoVrOyMjY0FZalVIf63+p6\/e+edd6i3t5defvllGh8fp0uXLtGmTZto+fLlC1Z6RkZGaOXKldTZ2Umf+cxn6E\/\/9E+JTdNXv\/rVoC8TExPU399PbW1tAlVGl0BAJgEYGJm6olcgoEWg2BaSmsjD5qa+vj4wDi0tLYE5UKsoV65cCbag2AjwZ3R0NNh+UkajcGupmIG5evVqYCwef\/xxevbZZwMDc+eddwZl3HHHHaR+HzYqXAebjp07d9Lhw4cDA\/Pcc8\/lV3a4HrWlxaZqamqKtm7dSh0dHcH3bKy4D5yOV4O+853vBAZId+tMCy4SgQAIpEoABiZVvCgcBNwmUGoFRhkVZUj4PIwyAqpHhedW3njjjfzqi0oTXrXh73QNDJsMtT3FqzC8WnLgwIHA4HDbeKWklLFRZ3cKV4+UgeF2q9UiNjb8M6cN99Vt1dA6EAABJgADgzgAgQomUGhgGIUyKrw9FNXAKENQCqnuFhLn58O+4a0ftUJTzsCo1R\/+P6+oHD9+fMEKDAxMBQc8ui6KAAyMKDnRGRCIRmCpFZhPfOIT+QO+ultIaksnnD58rmSpQ7yPPfZY\/o4m7oUyT4VbRWqrp9T3ysCEz9LwCg5WYKLFBlKDgOsEYGBcVwjtA4EUCZS7jTqNQ7w6t1HzgVs+r8ImhdP\/5Cc\/WXS4t9ghXnWGRR0mZuPC5bzyyiuBGduxY0ewZYQtpBSDCkWDQEYEYGAyAo1qQAAEkiVQbDsq2RpQGgiAgMsEYGBcVgdtAwEQWEAgfJs2\/4LPyOg8QA8YQQAE5BGAgZGnKXoEAiAAAiAAAuIJwMCIlxgdBAEQAAEQAAF5BGBg5GmKHoEACIAACICAeAIwMOIlRgdBAARAAARAQB4BGBh5mqJHIAACIAACICCeAAyMeInRQRAAARAAARCQRwAGRp6m6BEIgAAIgAAIiCcAAyNeYnQQBEAABEAABOQRgIGRpyl6BAIgAAIgAALiCcDAiJcYHQQBEAABEAABeQRgYORpih6BAAiAAAiAgHgCMDDiJUYHQQAEQAAEQEAeARgYeZqiRyAAAiAAAiAgngAMjHiJ0UEQAAEQAAEQkEcABkaepugRCIAACIAACIgnAAMjXmJ0EARAAARAAATkEYCBkacpegQCIAACIAAC4gnAwIiXGB0EARAAARAAAXkEYGDkaYoegQAIgAAIgIB4AjAw4iVGB0EABEAABEBAHgEYGHmaokcgAAIgAAIgIJ4ADIx4idFBEAABEAABEJBHAAZGnqboEQiAAAiAAAiIJwADI15idBAEQAAEQAAE5BGAgZGnKXoEAiAAAiAAAuIJwMCIlxgdBAEQAAEQAAF5BGBgNDS9++67NVIhCQiAAAiAQJoEqqurafXq1TQ3N0fnzp1LsypRZU9OTorqj+oMDIyGrGxgsg6AJOqMWoZO+nJpSv0+yveFacvVqSFh5CRJ1Bm1DJ305dJE4cxQiqUH\/6XDZSkNov5Oh38pnSIHdcQM5WJNp7ioZZRLf\/nyZert7aVcLkfr1q0r2gRcAzewKBbluOpo6WIaGBgNVaSKr9F1J5KAv10ZwN8uf1sGxn6vF7eADczJkydp2bJlJQ1MGu32\/Rrwvf2lNIWB0Yh2qeJrdN2JJOBvVwbwt8sfBuYGfxgYs1iUeg3DwGjEg1TxNbruRJLXX3+dVqxY4URbKrER4G9fdWgwr4EtA+M7f6lzGAyMxtgkVXyNrjuRxPfBwwmIMRoB\/jHgJZQVGsDAxAklqXMYDIxGVNQ+\/l2NVEgCAvYI3FV7U9HK76pZ+P2d19PdVVu1IH3X+oaSjcfkaU9XVTM0gIGJE4UwMHHoeZ5Xqvi+yILBO5pSgyem8hkuXb22IPObV9+98bvZ+X9fCn1XqqawQWJTxEZImSD+3Z3XjRL\/u5SZitYLpA4TwDUAAxPnipA6h2EFRiMqpIqv0XUnkmDwtifDC6\/O0csX3qRrv3zLAlOkjNCl2XeLGiBlYj59T3Xe6PC\/H7y32l5nPK4Z1wAMTJzwlTqHwcBoRIVU8TW67kQSDN52ZYjKf\/Slt\/Km5sVXZ6mYyQlWampuok\/fWxN0bqktLLu9d6P2qBq40erkW4FDvGZMpc5hMDAa8SBVfI2uO5EEg7ddGZLkz9tVL742FxgcNjf8eeG1uXwH2dioVRus2NzQPUkN7EZTvNphYMz4SZ3DYGA04kGq+BpddyIJBm+7MmTBv9DYFJqa3AO3V\/RKTRYa2I0yvdphYPQ4FaaSOofBwFxX+syZM9Te3h781N\/fT21tbfkYkCq+2aWQfS4M3tkzD9domz8fSubVGmVq1CrNM7n77ILJsHbbGmTY1SWrgoExU0LqHAYDQ0RXr16lzs7O4B0b\/Onr66OhoSGqra0NfpYqvtmlkH0uDN7ZM3fJwITbwis1fMam0NDwCo3kLSdcA\/NRAANjNhZIncNgYIiIV18GBwdpeHiYqqqqqLu7O3hZWHNzMwyM2fWSaC4M3onijFyYy\/yLGZoH76kODgdLOhjssgaRAypGBhgYM3gwMGbcvMjFBmZ0dJQGBgaC9rKBaWlpyW8jsfjhz6lTp7zol5RGTk9PU319vZTueNcPX\/jPvPMe\/f35nwZ8D35v\/mBw3a3vp9aPfZD+4JN+377tiwZpB\/fs7CydPXuWampqqKmpKe3q8uX7yH\/t2rUL+ExOTmbGK6uKsAJzfQWmnIGRKH5WQRa3Hvz1GZdgvPy+8udn2PAdT4MnXg8A8MrM8UdXx4NhKbevGiSNCyswZkSxAmPGzYtc2EJyWyYM3nb1kcCfzczoSz8Mzs\/wIWA+M5N74KPePDVYggZJRDEMjBlFGBgzbl7kwiFet2XC4G1XH0n81ZkZNjP8b2VmXD8vI0mDONEMA2NGDwbGjFtquY4ePUo9PT0Lyi+8\/TlK5eHbqEdGRvIHeLkMqeJH4WMzLQZvm\/SJpPJXZkZtMXWtX+HsqoxUDaJGNgxMVGLz6aXOYd6dgVFGo5hZUaam0ICYSX4jl1Tx43LJKj8G76xIF6+nEvjz1tILr87mt5iUmbFL\/kbtlaCBDmsYGB1Ki9NIncO8MjC81TM2NkabN29eUsUjR46UTRMlDKSKH4WBzbQYvG3Sl7sCU4xqeFXGpe0lXAPzasHAmI0FUucwrwyMmXTxc0kVPz6ZbErA4J0N51K1VCJ\/17aXKlGDYvEIA2M2Fkidw7wzMBcvXqQtW7bQzMxM8Mh\/\/vBZmLq6Ojp8+DA1NjaaKbxELqniJw4qpQIxeKcEVrPYSufPrzJQh35tnZOpdA1UqMLAaF60BcmkzmFeGZhr167ln5K7cuVK6ujooE2bNgUPnOPzL+Pj48HD6Phpukl+pIqfJKM0y8LgnSbd8mWD\/zyjsJHhZ8rsz92X2W3Y0ABbSOWv1NIppM5hXhkYPgOzZ88e2r17d\/CeIn78\/5o1a4I7hgp\/F0fswrxSxU+SUZplYfBOk275ssF\/ISN+psyO584Ht2FnZWSgAQxM+SsVBiYOo9TzwsCkjtjJCjB425UF\/Evz3\/DMueAt2WkbGWgAAxNnFJD6RzhWYDSiQqr4Gl13IgkGb7sygP\/S\/MMrMvx0Xz4nw3cwJfmBBjAwceJJ6hzmnYHhcy8TExNFtVy1alXwRmneXkryI1X8JBmlWRYG7zTpli8b\/Msz4hRhI8MmJsmn+0IDGBi9KCyeSuoc5pWBiSNgnLxSxY\/DJMu8GLyzpL24LvCPxr\/wrqUkjAw0gIGJFoULU0udw7wyMHwGBiswccLYz7wYvO3qBv5m\/NnI8GsKknggHjSAgTGLwvlcMDBx6KWQl2+bnpqaoq6urqB0\/pk\/fEt10h+p4ifNKa3yMHinRVavXPDX41QsVeGTfU1fUQANYGDMoxAGJg67xPMWu2Uat1EnjtmZAjF425UC\/OPzLzQy+zfdRw\/eW61dMDSAgdEOliIJpf4R7tUWktJFPdCupaUlv+LCz4Thp\/PiQXZxwtzNvBi87eoC\/snyV7deRznoCw1gYOJEIQxMHHop5C08D5PWHUjcdKnipyBLKkVi8E4Fq3ah4K+NSjuhumOJM+QeuL3sHUvQAAZGO7iwAhMHlby8MDB2NcXgDf52CaRXe\/ig7ytPfqpkRbgGYGDiRKHUOcyrLSRedRkbG6PNmzcvqeWRI0fKpokSDFLFj8LAZloM3jbpE4F\/uvzD52NKPdEXGsDAxIlCqXOYVwaGBTxz5gy1t7cHb6IuvOOI70TiN1OPjIwE70dK6iNV\/KT4pF0OBu+0CS9dPvhnw5+NzI7R88GrCQrPx0ADGJg4USh1DvPOwCgRlVkJi1rM1MQRXeWVKn4SbLIoA4N3FpRL1wH+2fIPbyupu5WgAQxMnCiUOod5a2DiiBk1r1Txo3KwlR6Dty3y8\/WCf\/b8C7eVuh78IH16VWP2DXGsxsuXL9PJkydp2bJltG7dusxa5\/s1IHUOg4HRuASkiq\/RdSeS+D54OAExRiPAPwa8mFmX2laKWbSX2WFgzGSTOofBwGjEg1TxNbruRBJMoHZlAH+7\/Ln2nufO0cHvzQWvJYj6EDz7rU+uBTAwZiylzmEwMBrxIFV8ja47kQQTqF0ZwN8uf7WN98sfup1GX3oreL9S7oGP0jO5++w3LOMWwMCYAZc6h3lrYMIPsnv22Wfp+eefD26dbmxcep84fPg3\/PA7dXcTh0fhYWCp4ptdCtnnwgSaPfNwjeBvl78yMCtWrAgawttKG\/7yXPBvnYfg2W99ci2AgTFjKXUO89LAhF8lwC90XLNmTaDq6Ojokq8SuHjxIvX19dHQ0BDV1taSev3Arl276Mknn6Te3t6gnHAa\/lmq+GaXQva5MIFmzxwGxi7zwtqLXQPF7lZyq9XJtwYGxoyp1DnMSwMTfnHjoUOHAgOzcuVK2rNnD+3evTswJzofXnVh07Nx40b6i7\/4CxoeHqaqqirq7u6mXC6Xf5aMVPF1GLmQBgbGrgrgb5d\/4QpMuDWVdsgXBsYsFqXOYV4amGIrMKdPn478MkdegWloaKDly5fnV284PNjAhF8UKVV8s0sh+1yYQLNnjhUYu8x1VmDCaSplNQYGxiwupc5hXhoYljDuyxz5LAxvP3V1dQVP91XbT6UMTDhsTp06ZRZFyGVEYHp6murr643yIlN8AuAfn2HcEnQ0mHnnPfryyR\/R2R+8S2f\/sCFulU7mn52dpbNnz1JNTQ01NTVl1kYd\/pk1RrOitWvXLkg5OTmpmdOfZN4aGB3EfOZly5YtwcpMa2tr\/nyMWnlRryJgA8PfYQtJh2r2abACkz1zrMDYZR51BSacnt90zYd8+ZZraYd8sQJjFpdYgTHjlmiuwlWXwsLDdxWVqpiNCp+ZCb8ricvt7OzEId5E1UquMBiY5FialAT+JtSSzRNVg8In+R5\/dHWyDbJUGgyMGXgYGDNuqeUKbwFxJfwzfwpf8BhuQPhWafW9WpmZmJgIXhLJn8KXQUoVPzVxEi446uCdcPUVXxz42w8BUw14NWbHc+eDDkhYjYGBMYtFqXOYl1tI4buQ1B1Hxb4zk3pxLqniJ8Un7XJMB++021Up5YO\/faXjaqAO+T54TzX5vBoDA2MWi1LnMC8NTPguJLXiop7pMjAwENwKneRHqvhJMkqzrLiDd5ptq4Sywd++yklooB6Ax\/\/vWr+Cutb7d9AXBsYsFqXOYV4aGJYw7l1IUcJAqvhRGNhMm8TgbbP9vtcN\/vYVTFKD8GrM\/tx9wWFfXz4wMGZKSZ3DvDUwZjKa5ZIqvhmN7HMlOXhn33r\/awR\/+xomrYGvqzEwMGaxKHUO89LAlLobSecuJBP5pYpvwsJGnqQHbxt98LlO8LevXloa+LYaAwNjFotS5zAvDUwxCfkuJH6ibvj2aDOpF+eSKn5SfNIuJ63BO+12Sykf\/O0rmaYGPq3GwMCYxaLUOUyMgcFdSGaB7UOuNAdvH\/pvu43gb1sBoiw08OFOJRgYs1iEgTHjllmu8NN0dV\/mqNs4qeLr9t92uiwGb9t9dLl+8LevTlYahJ\/iu3\/TffTgvdX2Ox9qAQyMmRxS5zAvV2BKnYHp7+9f8kF2ZtITSRXflEfW+bIavLPuly\/1gb99pbLUgLeUBk+8TqMvveXc7dYwMGaxKHUO89LAmElonkuq+OZEss2Z5eCdbc\/8qA387etkQ4PwaswrT37KPgQigoExk0HqHOalgcGTeM2C2NdcNgZvX1ml0W7wT4NqtDJtacCrMTtGz9MLr805sRoDAxMtblRqGBgzbonmUk\/gHRsbK1pu+I3TSVYsVfwkGaVZlq3BO80++VQ2+NtXy7YGrhzwhYExi0Wpc5iYFRgzWfVySRVfr\/f2U9kevO0TsNsC8LfLn2t3QQN1uzW3x9YBXxgYs1iUOod5ZWDU1tFjjz1GO3fuJH6DdPiDB9mZBbfruVwYvF1nlGb7wD9Nunplu6TBo6PnrR3whYHRi5fCVDAwZtxE5JIqvi\/iuDR4+8IsyXaCf5I0zcpyTQO+Q4mNDL9H6fgXV2f2PiUYGLP4kTqHebUCE5aOn7zb09ODFRizePYql2uDt1fwEmgs+CcAMWYRLmqgDvhemn03sy0lGBizQIKBMeOWSi71HJiurq5UXh1QKctvqYiTQqEuDt4pdNPZIsHfvjQua7DhmXOZ3aUEA2MWizAwZtxSyZXmawOKNViq+KmIk0KhLg\/eKXTXuSLB374krmuQ1V1KMDBmsSh1DvN6C4mlbGtrM1M0Qi6p4kdAYDWp64O3VTgZVA7+GUAuU4UPGmTx4DsYGLNYlDqHeWlgSr1KAHchmQW367l8GLxdZxinfeAfh14yeX3RIHwuJo2n98LAmMUTDIwZNxG5pIrvizi+DN6+8IzaTvCPSiz59L5poLaUnsndR7kHPpoYEBgYM5RS5zBRKzAsbV1dHR0+fJgaGxvNlC6SS6r4iQFKuSDfBu+UcWRePPhnjnxRhT5qoG617lq\/grrWNyQCEQbGDKPUOcxLA8MS8m3UU1NTxHciqZ\/5\/8uXL6fR0VF6+umnzZSGgUmMW1IF+Th4J9V3F8oBf\/sq+KqBOhfz4D3VdPzR1bFBwsCYIYSBMeOWSq6lXubIT+ndt28fDEwq5O0U6uvgbYdW8rWCf\/JMo5boswbKxFz9378ZtduL0sPAmCGEgTHjlkou9VJH3i4Kr8CMj4\/Trl276Jvf\/Gb++1INuHjxYpB27969wXbTmTNnqL29PUje39+\/4O4mqeKnIk4Khfo8eKeAI\/MiwT9z5Isq9F0DPtx7\/1f+meKuxMDAmMWi1DnM2y2kwjuR+A6k\/fv3B4akpaVlydurlQE6e\/ZscF7mtttuo87OTurt7Q2io6+vj4aGhqi2tjb4War4ZpdC9rl8H7yzJ5ZsjeCfLE+T0iRokISJgYExiR65c5i3BsZMxvlcfH7m7bffJjYwbFquXLlCg4ODNDw8TFVVVdTd3U25XC7\/lF8YmDi04+eVMHjHp2CvBPC3x17VLEWDuCYGBsYsFqXOYd4aGDYcBw8eXKCmznNgeOvoyJEjtH37dnryySfzBoYP\/g4MDATlsYEJr+Kw+OHPqVOnzKIIuYwITE9PU319vVFeZIpPAPzjM4xbgiQNZt55j1qPTFPTHTfRNzZGu8V6dnY2+MOzpqaGmpqa4mLVzu8j\/7Vr1y7o3+TkpHZ\/fUnopYHh7SO15fPyyy8Hdx698cYbAfOlnszLW0dPPfUUbd68ecG2Ea\/AlDMwEsX3JUil\/PXpC+\/CdoK\/feUkalD7+Hcjn4nBCoxZLGIFxoxbKrnCdyFduHCBTp8+TVu3bqU9e\/bQ7t2782dXeLVly5YtNDMzQ62trfTII48EKy\/8s\/rwQeA\/+qM\/Cg7+YgspFbliFypx8I4NJcMCwD9D2CWqkqiBujspynNiYGDMYhEGxoxbKrl4JYVvlWbTwqsnyqTobCGpBoVXcXCINxWZEitU4uCdGJwMCgL\/DCCXqUKqBsrE6D6xFwbGLBZhYMy4pZZrYmKCbr755vwt0Dt37oz0BN6wgSm8jXpkZCR\/gJc7IFX81MRJuGCpg3fCmFIrDvxTQ6tdsHQNeDvp+BdX04P3Vi\/JBAZGO2QWJJQ6h3l5BsZMQvNcUsU3J5JtTumDd7Y0o9cG\/tGZJZ1DugYbnjlHl2bfpXIvgISBMYssqXMYDIxGPEgVX6PrTiSRPng7AXmJRoC\/fYUqQQN+0N1dNTct+coBGBizWJQ6h3llYAofXlcoZZQzMFHCQKr4URjYTFsJg7dNvuXqBv9yhNL\/fSVooM7DLLWVBANjFmtS5zCvDUzhWRUzacvnkip++Z67kaISBm83SBdvBfjbV6dSNBg8MUWDJ16nUu9NgoExi0Wpc5hXBiYsXbFXCfBt0Orx\/2YyF88lVfwkGaVZVqUM3mkyjFM2+Mehl0zeStKAt5JyD9xOXesbFsGDgTGLJ6lzmLcGplBGfhmjeh1A0iZGqvhml0L2uSpp8M6ebvkawb88o7RTVJIGoy+9RY+Ong8O9N5Ve9MCtDAwZpEmdQ7z1sCEH1LHkvKD6vhVAPwuo6Q\/UsVPmlNa5VXS4J0Wwzjlgn8cesnkrTQN+K4k\/hx\/dDUMTAIhJHUO88rAhLeN0jqwWyxWpIqfwHWRSRGVNnhnAjVCJeAfAVZKSStNA7UKU3igFyswZgEmdQ7z1sAUkzEtUyNVfLNLIftclTZ4Z0946RrB374ilagBn4X59D3VxE\/pVR8YGLNYlDqHeWVgzKSLn0uq+PHJZFNCJQ7e2ZDVqwX89TilmaoSNSh2FgYGxizKpM5hMDAa8SBVfI2uO5GkEgdvJ8BfbwT421ejUjUoXIWBgTGLRalzGAyMRjxIFV+j604kqdTB2wn4RAT+9pWoVA3UKox6LgwMjFksSp3DYGA04kGq+BpddyJJpQ7eTsCHgXFChkq+BsIveoSBMQtHqXMYDIxGPEgVX6PrTiSp5MHbBQHA374KlawB31J9Z+1NwWFeGBizWJQ6h8HAaMSDVPE1uu5EkkoevF0QAPztq1DJGoS3kWBgzGJR6hwGA6MRD1LF1+i6E0kqefB2QQDwt69CpWugtpEab\/kPOnnyJC1btozWrVuXmTC+85c6h8HAaFwCUsXX6LoTSXwfPJyAGKMR4B8DXkJZK10DdTfS\/\/rvtTAwBjEldQ6DgdEIBqnia3TdiSSVPnjbFgH8bSuAO8H43UgvvjZHJ7beDQNjEI5S5zAYGI1gkCq+RtedSIIJ1K4M4G+XP9de6RqoczDnuz4OA2MQjlLnMBgYjWCQKr5G151IUumDt20RwN+2AjAwrACfg4GBMYtFqXMYDIxGPEgVX6PrTiTBBGpXBvC3yx8rMPP8eRvp3y\/\/O\/2Pj0zjEG\/EkJQ6h8HAaASCVPE1uu5EEkygdmUAf7v8YWDm+attpK81zcDARAxJqXMYDIxGIEgVX6PrTiTBBGpXBvC3yx8GZqGB2fNrl+m+Oz+M26gjhKXUOaziDMy1a9eou7ubxsbGqK6ujg4fPkyNjY105swZam9vD0Kiv7+f2tra8uEhVfwI8W81KfhbxU\/gb5c\/1w4N5jXg26k\/c+tbtOHjt2RqYHzn73v7S12BFWdgBgcHqaGhITAobFpOnz5NW7dupc7OTurt7Q049fX10dDQENXW1gY\/SxXf\/rCs1wLw1+OUVirwT4usfrnQYJ4Vn4M5MfED+vo6goHRDx+xc1hFGZirV6\/Snj17aPfu3XlzwjHARoaNzfDwMFVVVQUrNLlcjpqbm60ZmCQGrKhl6KQvl6bU76N8X5i2XJ0RrmPtpEnUGbUMnfTl0kThXMqcg\/\/SYbKUBlF\/Vyy97nfawWyYsFys6RQbtYxy6flVAvyHJo\/PpZ7Ei2vghjKKRTmuOlq6mKYiDQwLEd5CunLlCo2OjtLAwECgERuYlpaW\/DYSi48PCIAACICAXQLV1dW0evVqmpubo3PnztltjGe1T05Oetbi8s2tKANz8eJF2rJlC331q18NVleOHj1K4+PjtHHjRjp27FhJA1MeI1KAAAiAAAiAAAhkSUC0gVGGZWZmhlpbW2nXrl305JNPBkuQ6uAubx1t376dDhw4UHILKUtBUBcIgAAIgAAIgEB5AqINTLHuhw\/xqhWYsLHhPIWHeMtjRAoQAAEQAAEQAIEsCVScgeGDvB0dHTQxMUGrVq0KVl34bqPwbdQjIyP5A7xZioG6QAAEQAAEQAAE9AhUnIGjKN++AAAgAElEQVTRw4JUIAACIAACIAACLhOAgXFZHbQNBEAABEAABECgKAEYGAQGCIAACIAACICAdwRgYLyTDA0GARAAARAAARCAgUEMgAAIgAAIgAAIeEcABsY7ydBgEAABEAABEAABGBjEAAiAAAiAAAiAgHcEYGC8kwwNBgEQAAEQAAEQgIFBDIAACIAACIAACHhHAAbGO8nQYBAAARAAARAAARgYxAAIgAAIgAAIgIB3BGBgvJMMDQYBEAABEAABEICBQQyAAAiAAAiAAAh4RwAGxjvJ0GAQAAEQAAEQAAEYGMQACIAACIAACICAdwRgYLyTDA0GARAAARAAARCAgbkeA2fOnKH29vbgp\/7+fmpra0N0gAAIgAAIgAAIOEoABoaIrl69Sp2dndTb2xvI1NfXR0NDQ1RbW+uobGgWCIAACIAACFQ2ARgYIuLVl8HBQRoeHqaqqirq7u6mXC5Hzc3NlR0d6D0IgAAIgAAIOEoABua6gRkdHaWBgYFAJjYwLS0t+W2k0ZfeKivfnTU3lU1TLMFdtdHzmeQxahwygQAIgAAIgICjBGBgNAxM7ePfdVS+aM36pZ\/9KFqG66l\/6WdXIuUrVk+xMlS6D1x6MVL5SAwCIFCZBKqrq2n16tU0NzdH586dq0wIhr2enJw0zOluNhgYjS2ku+++m8qJf+nqu0Yql8rX3p6jkZHRkmW+OVu+vj\/+4z+mP\/uzP8uXUa6NTz\/9NH3pS19aVOelq9fy333729+mz3\/+8yXbxb\/\/9d98eNHvx\/\/lVaqvr9duS6kKClef7rq+8nXn9ZWsu2qr8llVWl4de\/De6kj66GhersCoZeikL5em1O+jfF+Ytlyd5TiY\/D6JOqOWoZt+qXRRf1csve53Jlyj5NHlsVSZUcsol\/7y5cvBWUXe4l+3bl3RqqPEOhegw7tcu6Jw1U2bRJ2qjCTK0m13lulgYDQO8UoVP8tAM6nrhVfngmybHvnDvBErNGHKXL0ZMpCXZt+lcmaNyw2bITZCbIIKDVDugY+aNF1UHsS\/fTmhwbwGbGBOnjxJy5YtK2lg0lDLd\/6+t7+UpjAw18mEb6MeGRlZcIBXqvhpXOhplJkEf2Vo+P9q9SpsctgIKRO0lAFSpqfQ8PD3Jis9afBKuswk+CfdpkorDxrAwMSJeanxAwOjERVSxdfouhNJXn\/9dVqxYoWVthQanxs\/3zA8L7w2v1IU\/iij8+l7qvOrOl3rG6z0IW6lNvnHbbuU\/NDAroHxnb\/UOQwGRmOEkyq+RtedSOLT4MHbXrzCo4zOi6\/OBgwLTU6hwXHZ3PjE34mATaER0AAGJk5YSZ3DYGA0okKq+BpddyKJlME7b2pem1tgcAq3rNjc8BbVp++tIReMjRT+TgSzYSOgAQyMYegE2aTOYTAwGlEhVXyNrjuRpFIGbzY4L143N+pMTnjlho2N2pLK0thUCn8ngr1EI6ABDEyc+JQ6h8HAaESFVPE1uu5EkkofvMPGhrekSpkaNjdRbxfXEbjS+eswSjsNNICBiRNjUucwGBiNqJAqvkbXnUiCwXuxDGxqlLEpZmpyD9werNYkYWjA3\/5lAA1gYOJEodQ5DAZGIyqkiq\/RdSeSYPDWk6HUSg1vPbGhMd12An89\/mmmggYwMHHiS+ocBgOjERVSxdfouhNJMHibyVBqlUYZGt0VGvA3459kLmgAAxMnnqTOYTAwGlEhVXyNrjuRBIN3MjKEDc3gideDQnVWZ8A\/Gf5xSoEGMDBx4kfqHAYDoxEVUsXX6LoTSTB4pyMDGxp+03r4DM2D91Qvun0b\/NPhH6VUaAADEyVeCtNKncNgYDSiQqr4Gl13IgkG7\/RlUGaGb99mU6NWZ1558lME\/unzL1cDNICBKRcjS\/1e6hwGA6MRFVLF1+i6E0kweGcvAz9RmJ9Jo7aaeGUm9+u3E15umb0WXCOuARiYOJEndQ6rOAMTfmkjB0R\/fz+1tbVR+Hv1nQoYqeLHuSCyzIvBO0vai+v61gvn6btvUrAyE\/UAsN2Wy6kd1wAMTJxoljqHVZyBOXr0KE1NTVFXV1c+Hq5evUqdnZ3U29sbfNfX10dDQ0NUW1sb\/CxV\/DgXRJZ5MXhnSXtxXYq\/2mYafemHwTNodA4A2225nNpxDcDAxIlmqXNYxRmYwcFBamhoCFZd1IdXX\/j74eFhqqqqou7ubsrlctTc3AwDE+eqSSgvBu+EQBoWU4r\/4Imp\/BZT1\/oVwfaSekmlYVXIVoIArgEYmDgXBwxMHHqO5OWVlo6ODpqYmAhatGrVqsC0XLhwgUZHR2lgYCD4ng1MS0tL3uRIFd8RWco2A4N3WUSpJijHn41MeFWGD\/7ikyyBchokW5u7pV2+fJlOnjxJy5Yto3Xr1mXWUN\/5S53DKm4FJhzxvJ00Pj5OGzdupGPHji1pYML5Tp06ldmFg4qIpqenqb6+HigsEdDlP\/POe\/T3539KB783R3W3vp9aP\/ZB+oNPVltqtaxqdTWQ1evFvZmdnaWzZ89STU0NNTU1ZdZdH\/mvXbt2AZ\/JycnMeGVVkWgDc\/HiRdqyZQvNzMxQa2trYFB4i0h91NbR9u3b6cCBA9hCyirqItbj+18\/EbvrXPKo\/NVZGb6DCedkkpEzqgbJ1OpeKViBMdMEKzBm3JzKxVtIe\/bsod27dwcHdPncC3+2bt2KQ7xOKbWwMRi87YoTh394e4nPyZi+j8kuAfu1x9HAfuuTawEMjBlLGBgzbs7lCt8urc7AsJkJfz8yMpI\/wMsdkCq+c+KUaBAGb7tKJcGfV2V2jJ6nF16bI36mzP7cfTjwG0HWJDSIUJ2zSWFgzKSROoeJ3kIyk3pxLqniJ8Un7XIweKdNeOnyk+TPRmbDX54LbsOGkdHXNUkN9Gt1LyUMjJkmUucwGBiNeJAqvkbXnUiCwduuDGnw5yf97njufN7IHH90td1OOl57Gho43uWizYOBMVNN6hwGA6MRD1LF1+i6E0kweNuVIU3+hUYGW0vFtU5TA7vRFa12GJhovFRqqXOY8waGb3Xu6elZoFrho\/7NJNXPJVV8fQJ2U2Lwls8\/bGRw2Hex3rgG5pnAwJiNBVLnMGcNjDpUW8ysKFNTeNjWTNryuaSKX77nbqTA4G1Xhyz5q6f74vbrhZpnqYHdaFu6dhgYM3WkzmFOGhi+3XlsbIw2b968pFpHjhwpm8ZM7oW5pIqfBJssysDgnQXl0nXY4A8jAwNTLCJhYMzGAqlzmJMGxkyi9HJJFT89YsmWbGMCTbYHfpdmi3\/hA\/H2b7qPHry3Mp\/sa0sD1yIXBsZMEalzmNMGpvDdRWHp6urq6PDhw9TY2GimaIRcUsWPgMBqUgzeVvGTbf5sZPipvqMvvVWxt17b1sBuBN6oHQbGTAmpc5jTBoal4vMuU1NT1NXVFSjHP\/Nn+fLlwQsYn376aTNFI+SSKn4EBFaTYvC2it+6gVG9r+SDvrgG5qMABsZsLJA6hzltYAof\/c\/Sqe8ee+wx2rdvHwyMWTx7lQuDt125XOMfPh\/DdyzlHvioXUAZ1O6aBhl0uWgVMDBm5GFgzLjFynXt2jXq7u4m3i4Kr8DwG6R37dpF3\/zmN\/Pfx6qoTGap4qfJLMmyMXgnSTN6WS7yD5+PqYQn+rqoQfRIip8DBsaModQ5zOkVGLXi0tHRQRMTE4Fy\/P6i\/fv30969e6mlpYXa2trMFI2QS6r4ERBYTYrB2yp+Z7aQilEIv2NJ8vNjcA3Mqw8DYzYWSJ3DvDIwzz77LD3\/\/PPBrdNZHN5VoSJVfLNLIftcGLyzZx6u0Qf+fMD30dHzwQsiJW4r+aBBFlEKA2NGWeoc5rSBUVtIvNLCB3nXrFkTqMeHdwcGBqiqqqqsmoWHgMN3NrW2tubLCb+NuvDheVLFLwvPkQQYvO0K4Qt\/ydtKvmiQdqTCwJgRljqHOW1gwod4Dx06FBiYlStX0p49e2j37t1UW1u7pJqDg4N08OBB2rZtW\/6sDH\/X0NBAGzZsCM7X5HK5oMzOzk7q7e0Nyuvr66OhoaF8+VLFN7sUss+FwTt75r6twBQS2vDMOXrhtblgNaZrfYNdgAnUjmsAW0hxwkjqHOa0gSm2AnP69GmamZkpuwLDKy98qzWn5w8fAlarL\/zv5ubm\/C3abIzY2AwPDwerOsrYcBr+SBU\/zgWRZV4M3lnSXlyXr\/zD20q+PwTPVw2SjlyswJgRlTqHOW1gWKrCh9nxIV42GuVWX5TMbEzCBkattPAZGjY5fEfTxo0b6dixY4Ep4g8bmPABYRY\/\/Dl16pRZFCGXEYHp6Wmqr683yotM8Qn4zH\/mnffo78\/\/lA5+b46a7riJvrHRz1uufdYgfgTeKGF2dpbOnj1LNTU11NTUlGTRS5blI\/+1a9cu6NPk5GRmvLKqyHkDExdEUgZGovhx2WaVH399ZkW6eD0S+Pt+t5IEDZKIYqzAmFHECowZN6NcS71CgAssXIVRW038AsjCVwwUGhi+JRtbSEayWMuEwdsa+qBiSfzDD8HzaVtJkgZxohkGxoweDIwZt9i5Sr1KQPf5L2EDw43BId7YkmReAAbvzJEvqFAa\/\/DdSvwU32dy99kFrFG7NA00ulw0CQyMGTkYGDNusXIt9SoBnbuQlGHh\/6sn+YZXd8J3J4Vvox4ZGQkO+aqPVPFjiZNhZgzeGcIuUpVU\/urdStzl3AO3O323klQNokY2DExUYvPppc5hTp+BCd+FpFZceAVF5y4kM5mL55IqfpKM0iwLg3eadMuXLZ2\/D9tK0jUoH4XzKWBgdEktTCd1DnPawLAEce9CMpO7MsRPgk0WZWDwzoJy6Toqhb\/Lz46pFA3KRToMTDlClfVHuPMGxkyuZHNJda\/JUkqvNAze6bHVKbmS+Lu6rVRJGiwVkzAwOlfs4jRS5zAnDQyvuvAdRfzOo6U+R44cKZvGTG6swCTBLakyMHgnRdKsnErkH95WeuXJT5mBSzBXJWpQDB8MjFlQwcCYcTPOpQ7VFr6XiAvkO5N6enqo8LCtcWVlMkoVPy1eSZeLwTtpotHKq1T+Lj07plI1KIxUGJho165KLXUOc3IFJiyRMivh74qZGjNZ9XJJFV+v9\/ZTYfC2q0Gl83dhW6nSNVBXAAyM2VggdQ5z3sCYyZVsLqniJ0spvdIweKfHVqdk8J+nZPNuJWgwrwEMjM4VuziN1DkMBkYjHqSKr9F1J5Jg8LYrA\/jf4B9+CN6D91TT8UdXZyIONICBiRNoUucwGBiNqJAqvkbXnUiCwduuDOC\/mH\/W52OgAQxMnFFA6hwGA6MRFVLF1+i6E0kweNuVAfxL8x996S0aPPF6kCDNp\/lCAxiYOKOA1DnMeQMTfpDds88+S88\/\/3xw63RjY2McPSPllSp+JAgWE2Pwtghf2Msc0yIZPh+ThpHBNQADEyd2pc5hThuY8KsEpqamaM2aNYGGo6OjNDAwQFVVVXE01c4rVXxtAJYTYvC2KwD46\/EPn4+5q\/YmSvJt19AABkYvCounkjqHOW1gwi9zPHToUGBgVq5cSXv27CHdlzkWvs06\/NJGllrdkh3+vvA2banix7kgssyLwTtL2ovrAv9o\/NnI8LYSby8lZWSgAQxMtChcmFrqHOa0gSm2AnP69Gntlznyix8PHjxI4bdOFxoalpmNUmdnJ\/X29gaq9\/X10dDQENXW1gY\/SxU\/zgWRZV4M3lnShoFJinb4oC\/fsbQ\/d19gaEw+uAZgYEziRuWROoc5bWCUuejo6KCJiYlAi1WrVtHw8HDeXJQSlY3K8uXLiQ0Pf7q6uoL\/s6lpaGgg9XZr\/o5XX\/h7Lpe3pbq7uymXy1FzczMMTJyrJqG8GLwTAmlYDPgbgrueTT0Ijw1N1\/oV1LW+IXKB0AAGJnLQhDLAwMShFyOvWjHZunUrKSMT5RUCbEyUgSn1ZusLFy7kz9VwWjYwLS0teZMjVfwYsmSaFYN3prgXVQb+yfBXdyyZGBloAAMTJwqlzmFOr8DwFtJTTz0V3HX08ssvEx\/k3bhxI\/FLHJ944gmtQ7xhA1MYAGyOxsfHgzKPHTsWHAwuZWDCeU+dOhUnlpA3IoHp6Wmqr6+PmAvJkyIA\/kmRnC\/nG\/93jg5+by7497Zfr6Y\/+GR12QqgwTyi2dlZOnv2LNXU1FBTU1NZbkkl8JH\/2rVrF3R\/cnIyKRzOlOO0gSk8xMtbPw899NCiQ7zqrAy\/wbquro4OHz6cv816KQOjto62b99OBw4cwBaSM2G5sCH469OuMOCfDn916zWXXm5rCRpgBSZOFGIFJg49w7xqBeZzn\/tccBhXHbI1XYEJGyI+oKvMDW9P4RCvoUgZZMPgnQHkJaoA\/3T56xgZaAADEycKYWDi0IuRV93ezHcShY2G7oPsCldgwrdLhw8Eh78vPGMjVfwYsmSaFYN3prgXVQb+2fBfyshAAxiYOFEodQ5zegspjmBJ5pUqfpKM0iwLg3eadMuXDf7lGSWZotDI5B74KP3nj39IK1asSLIaL8vC26jNZJM6hzlvYNSzXMKy6d5KbSb14lxSxU+KT9rlYAJNm\/DS5YO\/Hf5sZEZf+iHxXUt82Hf7Z82fI2OnB8nXCgNjxlTqHOa0gQk\/YI7vQuLnurzxxhuBguHnuJhJqp9Lqvj6BOymxAQK\/nYJ2K2djcz\/+ec3aead9yjuA\/Hs9iR+7TAwZgylzmHOGxj12gB+Vgs\/lI7PwUR5lYCZ3AtzSRU\/CTZZlAEDkwXl0nWAv13+XDtr8DcX3pdfkUnqFQX2exatBTAw0Xip1FLnMKcNDN+FtG\/fvsC0XLlyhbZs2RK8RgBbSGZB7GsuTKB2lQN\/u\/yVgVFnYMJP9mUjw7dg8zmZSvjAwJipDANjxi12Ln6FwM033xw814XvFNq5c+eC57zErkCjAKnia3TdiSSYQO3KAP52+RcaGNWa8LuW2MjkHrjd6DUF9nun3wIYGH1W4ZRS5zCnV2DMpEo+l1TxkyeVTomYQNPhqlsq+OuSSi\/dUhoUvv1aspGBgTGLMalzmPMGhh\/339PTs0A1bCGZBbGvuTCB2lUO\/O3yL7UCU9gqNjLqfUtSV2RgYMxiEQbGjFusXOrli\/wmafVm6FgFGmaWKr4hjsyzYQLNHPmCCsHfLn9dAxPeWpJqZGBgzGJR6hzm9ApM4aP\/zaSLn0uq+PHJZFMCJtBsOJeqBfzt8o9qYIoZGf5OHfbl1RlfP+ff\/BF9+s\/\/hf7md\/6T1q1bl1k3fL8GpM5hThsYjk7eQuJPls99KbwqpIqf2dUfsyLfB4+Y3beeHfytSxDcRh3nSbzFnu7ro5GBgTGLRalzmJMGRm0d8R1IxT44A2MWxL7mijt4+9pvV9oN\/vaVSEqD8NN9fXwoHt9CvuEvz2EFJmJIwsBEBGY7efjwb9jwhM1Ra2srDQwMUFVVVXCLdnt7e9Ds\/v7+BSs+UsW3rZFu\/UkN3rr1Id1CAuBvPyKS1sBXIwMDYxaLUucwJ1dgWKKLFy\/mH1wXNho68nHevr4+GhoaotraWuL3KfED8Nis8IPxGhoaaMOGDdTd3U25XI5WrlxJnZ2d1NvbGxQfzss\/SxVfh6ULaZIevF3ok09tAH\/7aqWlQeFD8fZvuo8evLfafodLtIAPJz86eh4rMBEVkjqHOWlg+Am8ylzw3UdsQNh0mJ6D4dWV0dFR2rVrF+3YsYPUXU28SjM1NUVr1qwJ6hgeHg5WY8J1w8BEvFJSSJ7W4J1CU0UWCf72ZU1bg8KH4rlqZHjl6NB3X6WvryMc4o0QljAwEWDFTVp49xGvqJw4cSIwHyYfZYAeeuih\/EoLP9mXDcz4+Dht3LiRjh07FqzQ8IcNTEtLS94wSRXfhKWNPGkP3jb65FOd4G9fraw0cN3IwMCYxaLUOczJFZhiBubIkSP0xBNPBCskUT5qlYVXXcJvt45qYMJ1njp1KkoTkDYmgenpaaqvr49ZCrKbEgB\/U3LJ5ctaA37z9ZdP\/ojO\/uBdqrv1\/TS22Y3rj9v0\/Tfepu77f0ZNTU3JAS5TUtb8k+jY2rVrFxQzOTmZRLFOlSHCwKgtp7GxMaqrq8u\/K6lw66nwwXjYQnIqFks2Jqu\/Pv2gkX0rwT975oU12tIgvCLjwl1LG545R\/9vboY6P\/nfsIUUISyxAhMBVtykSdxGzeaFz7YUPsFXmRoc4o2rUnb5bQ3e2fXQ7ZrA374+tjUIH\/a1+UA8NjC3\/uxN+sLqm2FgIoQlDEwEWLaThm+JVm1RdzLxak1HRwfxM2a2bdsWHOjlTzjPyMjIAuMjVXzbOunWb3vw1m2n1HTgb19ZVzRQryjglRk2Ml3rGzKFU\/v4d+l3bv8JDExE6lLnMCe3kCJqk3pyqeKnDi6hClwZvBPqjnfFgL99yVzTQD3ZN8uXRrJpuv8r\/0yPrbwS3OqNVwnox6XUOQwGRiMGpIqv0XUnkrg2eDsBJcNGgH+GsEtU5aoGWRoZ9RA7GJjo8Sh1DoOB0YgFqeJrdN2JJK4O3k7AyaAR4J8B5DJVuKwBr4yE336d1jNklIH5WtMMLVu2DCswEcJS6hwGA6MRBFLF1+i6E0lcHrydAJRyI8A\/ZcAaxfugQeEdS8cfXa3RM\/0k6hkwe37tMgyMPrYgpdQ5DAZGIxCkiq\/RdSeS+DB4OwEqpUaAf0pgIxTrkwZsZPiFi0kf9OU7kC7\/++XgDAxWYCIEDwxMNFjSUsPA2FXUp8HbLql0agf\/dLhGKdVHDZI+H8MG5t5bfk5Nv\/wqDEyU4IGBiUhLWHIYGLuC+jh42yWWbO3gnyxPk9J81SDJ8zF8C\/UX7r8ZBsYggKTOYdhC0ggGqeJrdN2JJL4O3k7AS6AR4J8AxJhF+K5B+HyMyfNj1AHe810fp5MnT2IFJmI8SZ3DYGA0AkGq+BpddyKJ74O3ExBjNAL8Y8BLKKsUDdQTfRnLK09+SpvOo6Pn6c2r79Kh\/1kHA6NN7UZCqXMYDIxGMEgVX6PrTiSRMng7AdOgEeBvAC3hLNI0iHo+hrePeOXmC\/dXwcAYxJbUOQwGRiMYpIqv0XUnkkgbvJ2AGqER4B8BVkpJJWpQeD5GvWOpGEI2MMe\/uJoab\/kPGBiDGJM6h8HAaASDVPE1uu5EEomDtxNgNRsB\/pqgUkwmWYNCI5N74HbiVxTcWXMTvfjaHA2eeD3\/3qXLly\/DwBjEmdQ5DAZGIxikiq\/RdSeSSB68nQBcphHgb1+lStBAGZkXX52lF16bC6CzkQmflYGBMYtFqXOYWANz9OhR6unpCdRetWoVDQ8PU21t7YK3TvPv+vv7qa2tbcH36jsVKlLFN7sUss9VCYN39lT1awR\/fVZppYQG82RhYMwiTOocJtLAXLx4kfr6+mhoaCgwLYODgzQzM0MDAwN0\/Phxmpqaoq6urnwkXL16lTo7O6m3tzf4LpyXf5YqvtmlkH0uDN7ZMw\/XCP52+XPt0AAGJk4USp3DRBqYQqHPnDlDo6OjgYHZt28fNTQ0BKsu6sO\/Z5PDqzRVVVXU3d1NuVyOmpubgyRSxY9zQWSZF4N3lrQX1wX+dvnDwNzgjxUYs1iUOodVhIFhc8Km5aGHHqKOjg6amJgIokBtLV24cCFvcPh7NjAtLS15k8Pihz+nTp0yiyLkMiIwPT1N9fX1RnmRKT4B8I\/PMG4J0GCe4OzsLJ09e5ZqamqoqakpLlbt\/D7yX7t27YL+TU5OavfXl4TiDQyfhSncMlLi8O\/Gx8dp48aNdOzYsWCFppSBkSi+L0GKFQC7SoG\/Xf5YgcEKTNwIxApMXIIp5r927VqwajI2NkZ1dXV0+PBhamxsDLaFCreLws1QW0fbt2+nAwcOYAspRY3iFI0JNA69+HnBPz7DuCVAg3mC2EIyiyQYGDNu1nKxeVmzZk3+HAs3hA\/r7tmzh3bv3p0\/3Mvfb926FYd4rSlVvmIM3uUZpZkC\/NOkq1c2NICB0YuU4qlgYOLQyzgvr6y0t7cvqLW1tTXYIuLzL+p3pW6vHhkZWWB8pIqfsSzG1WHwNkaXSEbwTwRjrEKgAQxMnACSOoeJPwMTR3SVV6r4SbDJogzwz4Jy6TrA3y5\/rh0a2DUwvvP3vf2lrkAYGI2xSar4Gl13Ign425UB\/O3yh4G5wd\/WGRjfrwHf2w8DE2MMsiF+EnVGLUMnfbk0pX4f5fvCtOXqjCFtyaxJ1Bm1DJ305dJE4VxqYgT\/pSNqKQ2i\/q5Yet3v0oj7cJnlYk2n\/qhllEvPBoYfOMrP6Vq3bl3RJuAauIFFsSjHVUdLF9NgBUZDFRYfHxAAARAAAbsEqqurafXq1TQ3N0fnzp2z2xjPapf4KBAYGM+CEM0FARAAARAAARAggoFBFIAACIAACIAACHhHAAbGO8nQYBAAARAAARAAARgYgxjgJ\/\/+7d\/+Lf3bv\/0bPfLII7RixQqDUpAlDoH33nuPvv3tb9NnP\/vZ4L0o+GRLgN9J89d\/\/dfE74jZtGkT3X\/\/\/dk2oMJre+eddwL+rMPmzZvpjjvuqHAi9rp\/\/PhxuvPOO4OzOfhkSwAGxoD3P\/7jP9Jtt90WvK6AB5EvfOELwVus8cmGwI9\/\/GM6dOhQ8I6rP\/mTPwmeqoxPtgT4PWIf+9jHgv++8Y1v0O\/\/\/u\/DSGYowT\/8wz8Er0n58Ic\/HIxBX\/ziF+kDH\/hAhi1AVUyAXwTMY9HnP\/\/5BQ8\/BZ1sCMDAhDjzqwY6OzuD2\/TYnPCHB+qenp7g3+oJvTxg\/\/Zv\/3bguo8cOUL8lF9MovEDVpc\/r4D913\/9F33rW98C+\/jYF5Sgq4HKxCsAf\/VXf0Xbtm2jD37wgwm3pvKKi8KfV2FOnjxJv\/Irv0Kf+9zn6H3ve1\/lAUuhx7oa\/PSnP6W\/+7u\/o2XLlgWx39zcnEJrUORSBGBgrtO5ePEibdmyJfhJvQySv+vr66OhoaHAaY+OjgavI+D\/s2n5yEc+AgOT0PUVhb9a7YJ5TLTuifcAAAa\/SURBVAi+wTXAGrB52bdvX\/BMjpUrVybbmAosLeo18POf\/5z4FQO8GsMGEqvA8YNGV4P+\/n76p3\/6J\/rVX\/1Vevvtt4OKYWDi849aAgzM9Zc88jLgb\/3Wb9GXv\/xl2rt3b7ACw6sv4+PjgWnhv\/rV6sz3v\/99+vjHPx4s4fIk2tbWRrfeemtU9kh\/nQD\/xROFv1odg4FJLoSiavChD30o0Iy3T3H+Ir4OUfnzA914DOLzX1\/\/+tdpw4YN0CGmDFE02LlzJ50\/f57efPNN+sEPfhDo8KUvfQmrkDE1iJodBiZEjN33rl27FhgYPmfR1dUVvMm6o6Mj+DdvHfE2EgctT6YPP\/xwVO5IX4SALn\/1lw4MTPJhpKMBXyP8EDEeuPkMBpuZ3\/3d38XgnYAcOvx5DOIta\/4Di7cu+N+\/93u\/R+9\/\/\/sTaAGK0NVAjUP88mCswNiJGxgYAwODpcJ0gjXqwJFOKyq7VGhgV\/8o\/PkcGP8H45KsZlE0SLZmlBaVAAxMGQNTbAtJbWFEhY30SxMoNnCAf7ZRAw2y5V1YG\/jb5c+1QwP7Gui2AAZmCQNT6hAvDsvphle0dIUDB\/hH45dEamiQBEXzMsDfnF1SOaFBUiTTLwcGZgkDw79St1HX1dXl705KX5bKrKFw4AD\/7OMAGmTPPFwj+NvlX2wFBuOQfU1KtQAGxl1t0DIQAAEQAAEQAIESBGBgEBogAAIgAAIgAALeEYCB8U4yNBgEQAAEQAAEQAAGBjEAAiAAAiAAAiDgHQEYGO8kQ4NBAARAAARAAARgYBADIAACIAACIAAC3hGAgfFOMjQYBEAABEAABEAABgYxAAIgAAIgAAIg4B0BGBjvJEODQQAEQAAEQAAEYGAQAyAAAiAAAiAAAt4RgIHxTjI0GARAAARAAARAAAYGMQACFUbg2rVr1N3dTWNjYwt6vm3bNurq6vKaBr9L6MSJE9TR0RH0saWlhdra2oI+qX7ncjlqbm5e1E\/+\/b59+2jr1q1UW1vrNQc0HgQqgQAMTCWojD6CQIiAmsjDk7sEQGEDwm+Mj2pgmIEyQDt27JCABH0AAdEEYGBEy4vOgcBiAksZGH77+vj4OF26dIk2bdpEn\/jEJ2jLli00MzNDq1atouHh4WB14syZM9Te3k78lvbf+I3foFtuuSVYueCVD17F4RUOLmtqair4Wb3VnVujVnq4jIMHDwYNPH36NLW2ttLAwACx+RgcHMz\/bmRkJEgzOjoa\/J4\/bE4KV1K4vDfeeCNYcSnWx\/AKDNen6ubywnXv37+f1q9fT42NjQgfEAABhwnAwDgsDpoGAmkQKLaFpMzJd77zHXruuecCo8Kfzs5O6u3tDSZzZUjCRoXzsZlgI1PKwKxZs6ao+eDyd+7cSYcPH6bbbrstb374ezYw3IYrV65QX18fPfHEE\/S1r32Ndu\/enf9uaGhowVYP5+G62DyV2ibjstkQhbeQwvn499xP\/qitpzQ0QJkgAALxCcDAxGeIEkDAKwI6KzC80jE9PZ1ffVEdZMOyfft2OnDgQH41ppixCa\/ANDQ0UE9PzwJGvArDZkMZFbXlw6sqvIqiVm7CmZTR4O94BSV8Xof79NRTT9HmzZsDs1VuBUYZmELzwmXzSk5h+V4JjMaCQIUQgIGpEKHRTRBQBKIYGF79KFzp4AleGQ\/eTtIxMMUMSbgcHQOjjAX3Q620hPsU1cCUWmmBgcG1AgJ+EICB8UMntBIEEiOga2A4HZ9p4bMwvJ2izsfs2rWL+JArr4CEt5Aee+yx\/MHZDRs25LeW2GyoraL6+vp8muXLlxddgeGOhreQuL69e\/cS5+W7hP71X\/91kalSeQq3kErdhcSrPPwptk2ELaTEQg0FgUCqBGBgUsWLwkHAPQK6BoZXRfiuHN1DvOHDuuHDvUsd4i22haS2n9S2U39\/f95ohA8GF5INr5wstYX08MMPB1tgExMT+SLUGSDuc3glxz310CIQAAFFAAYGsQACIBCLwFKmIlbBBZmzeI4LbqNOUjGUBQLpEoCBSZcvSgcB8QSyMDBXr14NtrPuuuuu\/K3WxcDGOb9SeBBYvHDoIAh4TgAGxnMB0XwQAAEQAAEQqEQCMDCVqDr6DAIgAAIgAAKeE4CB8VxANB8EQAAEQAAEKpEADEwlqo4+gwAIgAAIgIDnBGBgPBcQzQcBEAABEACBSiQAA1OJqqPPIAACIAACIOA5ARgYzwVE80EABEAABECgEgnAwFSi6ugzCIAACIAACHhOAAbGcwHRfBAAARAAARCoRAIwMJWoOvoMAiAAAiAAAp4T+P+B1b78OoenFgAAAABJRU5ErkJggg==","height":337,"width":560}}
%---
%[output:97e357dd]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"num","rows":1,"type":"double","value":[["0","0.031366016786362","0.027661589854530"]]}}
%---
%[output:9d1224cc]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000","-1.656408362613720","0.685922165934166"]]}}
%---
%[output:26f707e8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"6","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000001000000000"],["-3.553057584392168","-0.000094247779608"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"2","name":"Aresd_nom","rows":2,"type":"double","value":[["0.010000000000000","0.000001000000000"],["-3.553057584392168","0.009905752220392"]]}}
%---
%[output:6e27c0a7]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:7996ad92]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:9ad8d2a8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"    -3.553057584392168e+02"}}
%---
%[output:637faca4]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.990575222039231"}}
%---
%[output:61f9eea3]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.149016195397013"],["36.521945502464568"]]}}
%---
%[output:306c88de]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:23fb1bfd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:11c23dfc]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:4b099645]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.155508836352695"],["27.166086113998457"]]}}
%---
%[output:00aa870f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.037191061070411"],["1.937144673860303"]]}}
%---
%[output:2e5b507a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10Xkd55x\/atERLCLaSUEcRxqkj09BQOc6GCmEw1Jt6264cDm1Xks+2rBCsW2I4u4lXkgnkNCTEktZOmyahuKnQprtY9rYkYJdCABMoqXAJTqyWEhIniu3IismHYxKIAw14z3OdUUZX92Nm7jzve+\/of8\/JcaT3mefO\/J55Z\/6az1ecPHnyJOEBARAAARAAARAAgQoReAUETIWihayCAAiAAAiAAAhEBCBgUBFAAARAAARAAAQqRwACpnIhQ4ZBAARAAARAAAQgYFAHQAAEQAAEQAAEKkcAAqZyIUOGQQAEQAAEQAAEIGBQB0AABEAABEAABCpHAAKmciFDhkEABEAABEAABCBgUAdAwBOBY8eOUW9vb+RtZGSEGhsbPXme6+bAgQPU09NDl1xyCQ0ODlJDQ4PYu3THtSyjSYEUhw9+8IPU2dlpkqTUNkNDQ7Rt27Yoj+vXr6f+\/n6r\/O7cuZM2bdpEmzdvLiUPLt\/evXvFvx9W0GBcWQIQMJUNHTJeNgK17NzTBAx3EKtWraK2tjYRPGlllH5vWmFcOkTuQL\/+9a9biQOXNLYB4HesW7duJlmIAiY0wWkbY9j7JQAB45cnvIFAXQicOHGCBgYGaPfu3bR9+\/aaCRge+anFe5Ogqs6wo6PDWIyoEQobceCSxqUSKAFjk7f4e8o+AqPq6eHDhzEK41JJkGYWAQgYVIhSEtCH0jmD+pC4PvqwfPlyuu6666IycEemT6eo0YKJiYk5n+s+Lr\/8cnrf+94X2TQ1NdHo6Ci1tLQkctGFQtw+PjrBn6sppdWrV9ONN95Ira2tUcOtd\/x5fngqSr133759Uf74UVNI11xzDX3sYx+LxIt64iz494qpLnBUp6nbq05Q+dI7VL2Mt9xyCw0PDye+d2pqKsrf9PT0TJ70GOocmfmtt95Kn\/rUp0iVj\/nHWSt2amouqbNWcdXfq8obL5eKdXNz84wIi\/PbtWtXNCWjHr1+xP3lCcd4fczylVUPs0ZqdCacZ5X3uCiKf790tsrHlVdeSXv27CH+\/qjY6WXm36l36LHN45JUD0vZCCFTpScAAVP6EM2vDMY7Lb30qhFO6qTiHQ\/7YfGgxEv886QONqvz58\/S8qY6m7POOmvWGhglYPQ8sFBIEhy6iIn78SVgkv7Cj3cm8Y4tjSv\/Pk3A9PX10YYNG+awzxIM\/Nk555xDTz75ZCTQkkQFv1PvaON5T6sX6r333Xdfohi54447Ztad6PVN76DjAibuS32eJmKy6iynOXToUKpQ0vMUFy9xkanEw9ve9jb6xje+MavxSBIhSd+vuABhm6Q88u\/Ve\/J861zKPko0v1rcapcWAqba8Qsu96qB1v8CVY0\/F1YffeC\/slXDqHcQemOr\/+Wpd3gsEtQIgfKh3h3\/S19BTvpcb4wvu+yyVAGj\/4Vq6ydPwPCoEz95UzlZI0Q8KvT000\/PYaKPGjCnZcuWzSqjyRRSfHpLsVfx5NGWeNw5L7weJGlkiFmuXbs2Kq8+YmOysNlkOihuE\/9ZMVFii\/Of925V95LKo37HQpfLnDaFpHNU9Ske0y9\/+cuREEoaUUnzGx+FU6NOuo+sd6sRGlX\/87j4mCoLruFDgZwIQMA4YUMiKQJpf52pDoAb7hUrViTuwNFtDh48mPhXNedb98F\/9asdQ3mLcPP+ckwTCHqDzu+39eNLwOjvZjHCj95hpnUsegf+\/ve\/31jAJI1YJb2X8xGfIksb4WBb7ohVPnS2Se+LT6VlCZi0qbN4mqzRlCTxGy+bmp6MCyEl2tKERl791OOr+0iLa3w0R7FSAiZt6lDfYafXZfW91KfvVDuhc0matpRqT+A3bAIQMGHHt3Klq7WA0bch53UQtsKD4Sdtqzb1o3fO8c6OfevbqE1GYNhGX\/jKP\/OW3fgIVLwDtRUwcY7xUZq4cPIlYFRlTxNOvDMrScDEp6LyRmCqIGCSRvxUXOP1L20ERveR9t2AgKlcExtUhiFgggpn9QvjOoUUn+pQawrS\/ppNGvLPEzBZUz\/6qABHgf9KTRMwpn54aD4uLtTUWlzAsEgwWRwZ79z1EYr4NBx3+HlTSDw6lCcA4j5cp5D02p02qhH\/BsTFSHw0QpVZH4lT5VF1J54maQop75snPYWkxK4auUoTMEkjV4pRfAQmbdF1fPoqawopiQumkPJqCz43JQABY0oKdjUhIL2IN0sA5AmYrLwlrQ9JEzB5fni4Xa1niUM3ETCcJmkXkvIV30miHwBns4hXTSXoafi97373u6PRoaSHOSWVz3QRL\/tUoi4unNIWuOppdBt+50033UTXX3\/9nAXHnCYuYPh3aQuCVVnzBHPS9EreCJjOMa2MWeJDFwwf+tCHUutWlg\/OQ9LiXtNFvDqXvBHImjQ0eEkQBCBggghjeIUw3Uatb4HO20adtDDYZgqJKWdNT+QtktVP5s3yw++Jb7n96Ec\/Svv3759ZtJo0AqN3bmkLkXXf8bU5SQJH78j1tPz\/SsAkvfe2226bdaIsH66nr7dx2UatCxG9Q03aYp+2fTvOVV+Twz6Z25YtW2jjxo0RDn0kTe0mS9uWnXd+S9Y2an6X6chE2toVHoVLEgdpo07MKGkLe9IoTpr45d\/HT\/5NW0ukfJiMFIbXoqFEEgQgYCSowqcogbwdH6Ivh\/PCBJKmquI7zdLO4dFf7nKQXeHMz1MHuuBUQi1pZ1IeHhxkl0cIn9sQCFLAcMPGZ1HwIVtJDWHWAWc28GBbHwIQMPXh7uutWVNoWVNfSe93uUrAVznmm5+kKSRmkHf4Y5LoDOXuqvlWB8pW3uAETN7iPvV5e3t7dNmZ+pm\/hLYXp5UtmPMlPxAw1Y90\/I8ILpGteOE0uFuntnUhPrVrI144pxCctY1X6G8LTsDwfC9\/SfhJG4GJB5X\/shgfH6\/prb6hVyyUDwRAAARAAAQkCQQlYPivumuvvZa6u7sjEQMBI1l14BsEQAAEQAAE6kcgKAHDIyn88ImQWWtgdNxqKLurqyuaUsIDAiAAAiAAAiBQfgLBCBieC7\/99tvp6quvJr6oz0TAqPUvHCb9FuN42H75l3+5\/JFEDkEABEAABEAggQDfKn7++ecHxyYYAcNTRnzWBJ8emrcLiaNoKl7YlgXM5ORkcMGvd4HAVS4CYAu2cgTkPKPeyrANlWsQAiZpR4OqBknX29vuPAo1+DJfFXOv4GrOytYSbG2JmduDrTkrW0uwtSVmZh8q1yAETDyEeSMwPFrDp1BmTRvpPkMNvlnVl7MCV7CVIyDnGfUWbOUIyHgOtc7OCwGjRlx4d9KyZcuiG4LVseCqumQdvR5q8GW+KuZewdWcla0l2NoSM7cHW3NWtpZga0vMzD5UrkEKGLOQmluFGnxzAjKWjz76aJALy2Ro2XkFWzteNtZga0PLzhZs7XiZWi+56M108DvfMjWvjB0EjEGoIGAMIDmYoLFygGaYBGwNQTmYga0DNMMkYGsIysJs7N6jdMXYA3TsxndapKqGKQSMQZwgYAwgOZigsXKAZpgEbA1BOZiBrQM0wyRgawjKwgwCxgJWiKYQMDJRRWMlw5W9gi3YyhGQ84x6658tBIx\/ppXyCAEjEy40VjJcIWDkuIIt2MoS8O8dAsY\/00p5hICRCRcEjAxXdLJyXMEWbGUJ+Pc+dNdBGrrrUayB8Y+2Gh4hYGTiBAEjwxWdrBxXsAVbWQL+vUPA+GdaKY8QMDLhgoCR4YpOVo4r2IKtLAH\/3iFg\/DOtlEcIGJlwQcDIcEUnK8cVbMFWloB\/7xAw\/plWyiMEjEy4IGBkuKKTleMKtmArS8C\/dwgY\/0wr5RECRiZcEDAyXNHJynEFW7CVJeDfOwSMf6aV8ggBIxMuCBgZruhk5biCLdjKEvDvnU\/h5a3UOInXP9tKeISAkQkTBIwMV3SyclzBFmxlCfj3DgHjn2mlPELAyIQLAkaGKzpZOa5gC7ayBPx7h4Dxz7RSHiFgZMIFASPDFZ2sHFewBVtZAv69Q8D4Z1opjxAwMuGCgJHhik5WjivYgq0sAf\/eIWD8M62URwgYmXBBwMhwRScrxxVswVaWgH\/va2+9n+555DgW8fpHWw2PEDAycYKAkeGKTlaOK9iCrSwB\/94hYPwzrZRHCBiZcEHAyHBFJyvHFWzBVpaAf+8QMP6ZVsojBIxMuCBgZLiik5XjCrZgK0vAv3cIGP9MS+PxwIED1NfXR8PDw9TS0pKYLwgYmXBBwMhwRScrxxVswVaWgH\/vy6\/\/Jh0+9gLWwPhHW1+PJ06coIGBAdq3bx+Njo5CwNQ4HBAwcsDBFmzlCMh5Rr31z7bxyrsjpziJ1z\/bunrcu3cvDQ0NRXnACEztQ4HGSo452IKtHAE5z6i3ftnyyAuPwEDA+OVad2\/Hjh2ja6+9lrq7uyMRAwFT+5CgsZJjDrZgK0dAzjPqrV+2EDB+eZbG286dO6O8rFixAmtg6hQVNFZy4MEWbOUIyHlGvfXL9p6Hj9PaT9yPERi\/WOvrjRfu3n777XT11VfT1NSUkYDRc7xnz576FiCQtzP75ubmQEpTrmKArVw8wBZs5Qj48bx69erI0U8Wv5WeX\/FeCBg\/WMvhhaeMVq1aRW1tbYRdSPWLCf7akmMPtmArR0DOM+qtX7ZDdx2kobsehYDxi7V+3njtS29vL01MTMzJxPbt2yNRE3+wjVomXmisZLiyV7AFWzkCcp5Rb\/2yVWfA\/NzzT9FTn\/x9v85L4O0VJ0+ePFmCfNQtCxiBqRt6dLKC6NERyMEFW7CVI+DXs9pCfdpTD9ITf\/1Hfp2XwBsEDA6yq1s1REcghx5swVaOgJxn1Ft\/bPUdSA3\/\/Gk68pW\/8ue8JJ7mvYAxiQOmkEwo2dugsbJnZpoCbE1J2duBrT0z0xRga0oq304XMGfcM0yHv\/WF\/EQVs4CAMQgYBIwBJAcTNFYO0AyTgK0hKAczsHWAZpgEbA1BGZip9S9suuCzvTQ5OWmQqlomEDAG8YKAMYDkYILGygGaYRKwNQTlYAa2DtAMk4CtIagcM330ZeXSBfSdrb8LAeMHbfW8QMDIxAyNlQxX9gq2YCtHQM4z6q0ftmP3HqUrxh6InN3afSFd3dkOAeMHbfW8QMDIxAyNlQxXCBg5rmALtrIEinvn0ZcNYw\/QPY8cp8WNp9P+j7yFQu3DMIVkUF9CDb5B0UVNIGDk8IIt2MoRkPOMelucrT76svGyJfTh3zofAqY41up6gICRiR0aKxmuGCWQ4wq2YCtLoJh3Hn3hu4\/4Xx59uaXrQlp5wQIImGJYq50aAkYmfhAwMlzRycpxBVuwlSVQzPvIPx6h\/\/mZhyIn\/WvOp\/41S6L\/D7UPwxSSQX0JNfgGRRc1gYCRwwu2YCtHQM4z6q07W33qSK19Ud5C7cMgYAzqS6jBNyi6qAkaKzm8YAu2cgTkPKPeurHVt03rU0cQMG48g0oFASMTTjRWMlwxzSHHFWzBVpaAvXd93Qun3vWBi6N1L\/oTah+GERiD+hJq8A2KLmoCASOHF2zBVo6AnGfUWzu2+sgLp9zwjtfRx9ZeMMdJqH0YBIxBfQk1+AZFFzVBYyWHF2zBVo6AnGfUW3O29zx8PNpxpJ4\/bGuiP\/vPb0h0EGofBgFjUF9CDb5B0UVN0FjJ4QVbsJUjIOcZ9TafLY+6fOa+79N1f\/\/y3UZ\/u76VfuMNjamJQ+3DIGDy60uwW9AMii5qgsZKDi\/Ygq0cATnPqLfZbHnUZcOOB6JzXvjhBbu8Xbr70kWZCSFg5Ops6T2HGvx6g0djJRcBsAVbOQJynlFv09kOf+kgDX7x0RkDFi+8YJf\/zXtC7cMwApMX+YAPATIouqgJGis5vGALtnIE5Dyj3s5myyMt9zz8DG3Y8b1ZH4y+5yK6vPUc40BAwBijCs8w1ODXO1JorOQiALZgK0dAzjPq7Sm2LFy++K9P0cCdB2bBthl10ROG2odhBMbguxhq8A2KLmqCxkoOL9iCrRwBOc+ot0Qf3PE9+vS3Hp8jXNS9Ri70Q+3DIGAMakOowTcouqgJGis5vGALtnIE5DzP13rLi3OH73qU7nnkuFfhopyF2odBwBh8F0MNvkHRRU3ma2MlCvUl52ArRxlswdYHARYtn93\/BH1q\/Mgcd0nXARR5Z6h9WDAC5sSJEzQwMEC7d++O4rx+\/Xrq7+9PjfnOnTtp06ZN0ecdHR00ODhIDQ0NifahBr\/IF8JHWnQEPigm+wBbsJUjIOc59HrLa1v+Zt\/36eNfePkMF0WTRctbly6ItkWb7CyyiUKofVgwAmZoaCiKJ4uWY8eOUW9vL3V1dVFnZ+ecOO\/du5fYfmRkJBItLHyamppSBU+owbf5AkjYht5YSTAz9Qm2pqTs7cDWnplpihDZ8kjLl777FN3ytccSMfgebUl6Sah9WDACJh40XdDEP+PRl\/Hx8ZlRl\/jPcftQg2\/aqEjZhdhYSbGy9Qu2tsTM7cHWnJWtZShs09a06KMtN3ddSK9vPN37aAsEjG2tK5m9GoHh0Zi2tjajEZj29vbE0RpODAEjE+BQGisZOsW8gm0xflmpwRZs4wR4auj\/\/tPjtHfy+JyFuLpo+fPOX6ElZzXURLToeQy1DwtuBIZHXrZt25a7ruXAgQPU09ND09PTtH379kShoyoAB19\/9uzZI\/cNnkeep6amqLm5eR6VuHZFBVs51mA7v9lOP\/tiBID\/\/ctvHad9R04d6x9\/ms48jc599Wm0\/tcXRP\/yz7V6Vq9ePedVk5Nz193UKj9S7wlOwChQLGRYnCQtzuUpox07dkRrYBobG6P1MPykLfoNVb1KVSpTv\/hL1pSUvR3Y2jMzTQG2pqTs7crKVq1j2f\/Yc6kjLFxaXs\/S+e8X0dsuWEgrL1hgD0AoRah9WLAChkdY+vr6aHh4mFpaWmaqhdqtpE8ZpdmqRKEGX+i7Yuy2rI2VcQFKbAi2csEB2\/DZsmD5zP3fp7sfPDZzcWJSqdXOoe5Lzy2VYInnNdQ+LFgBo+804lEW9UDAyDU+tp7REdgSM7cHW3NWtpZga0vM3L4ebFmsPPnDn9DoPx7JHF1RIyy81ZkFC4sX39udzUnZWULA2PHKtVYLbScmJnJtdYPW1la6884756TRp4GUSEnbGp00hZQ23cQvCjX4VuAFjOvRWAkUo5QuwVYuLGBbXbY2YkUJlnctfy29t\/28qNBVESwYgZGro5HnvJ1CSa9XoypJAiZ+kJ1+OJ36rLu7e2axrlrsy+\/BQXbCwU5xj45AjjvYgq0cATnPPusti5Vjz\/8b\/dU3pnJHVpQ4WbzwdOp76SC5qoqVpOiE+kd43aaQfAsYua8URmCk2PpsrKTyWFW\/YCsXObAtD1vevsxCg\/\/d+pVD9OiTz1uJld6VzXTWq36hUtNBLvQhYFyoBZIm1ODXOzzoCOQiALZgK0dAznNeveVRlQe\/\/yP63P4njISKPrKy+sKz6JLFZ5Z6sa0U2VD7sLqPwPAamLx7i6SCauo31OCbll\/KLq+xknrvfPALtnJRBltZtj\/\/mnOjEZVvH\/oBffV7x+jwMy9k7gTSc6PvCpoRL42ny2W4Ip5D7cPqJmBU3PW1KLzodnR0dNa25zLUj1CDX2+26AjkIgC2YCtHwI9nFin8fPXBY3THfd+3Fiq8XmXVskb69fNfE\/wUUFHiofZhdRcwKjDxXUllGpUJNfhFvxRF06OTLUowPT3Ygq0cATvPPO3Dzxe+8xT9y5Hsg+DintVCWrV1uUyHw9lRqK91qH1YaQSMHl79mP8yjMqEGvz6fqWI0MnKRQBswVaOwFzPPJrC\/z353E9odPxIZHDPI6eEi8mjhErrL51G1\/3umyJfVTpnxaSM9bQJtQ8rpYDRA512IF0tK0Oowa8lw6R3oZOViwDYgq1vAkqkTD71PP3Toz+gx469YCVSOD+RKFl4Ov3mG8+i5a87c+ZcFSVgUG99R+2Uv1D7sFIKGH0EhuHnXbYoE\/KXvYYafGluef7RWOURcv8cbN3Z5aUMnS1P+fzSmb9IN331MB1++oSTSGGGPO1z1WVLaPr4j41HU0Jnm1e3pD4PtQ8rjYDJOohOKqimfkMNvmn5pezQWEmRxfScHNlqs1VTMyxSnnn+3+iL\/\/qU00iKPpryX9qaqOk1r5wzmuISA7QJLtTy04Tah9VdwOi7kMow2pJUFUINfn61l7VAYyXHF2znN1u1w2fs3qP0jw8\/Y7XDRyenpnzaL1hAK5cujD6SXJuCeitTb0Ptw+omYPRdR3lH+cuE1NxrqME3JyBjicZKhit7Bduw2SqB8sDRH9GuiSecR1H0kRRek\/K+lefVdQEt6q1MvQ21D6ubgHn44YfpQx\/6EF1zzTUz9xPlhS7rLqS8tEU+DzX4RZj4SIvGygfFZB9gW122+jQPl2Ls3scjgWJzoFt8FIV\/XnnBQvqVRa+i5c2vFh1FKUIe9bYIvfS0ofZhdRMwuAtJpqJWySsaK7logW252aodPTwdc8vXDtP3Hv+Rs0DRR1F+9bwz6HcuOicqfBXPTEG9lam3EDCeucYPrjN139raSkm3UZumd7ELNfguLHymQWPlk+ZsX2Bbf7a8UPZ1LFDuPkwPHi0uUCJRsnQh\/ff\/sJiO\/uAnXhbNylFy84x668YtL1WofVjdRmDygJfp81CDX2\/GaKzkIgC2smz5vh5+fvziz+jmuw\/TwadOFB5BOTVqspCWntNAl77+NZF\/yQWzcoTcPaPeurPLShlqHwYBY1BfQg2+QdFFTdBYyeEF22Js1RTPl777FO1\/7LnImc3JsvG360fi\/8YbGunSJa+JFstWcZqnGNns1Ki3MnRD7cMgYAzqS6jBNyi6qAkaKzm8YJvOVomTpgWvpD\/9yiE69HSx0ZOZkZKFp0dTRpe3vjZaLKseJV7koh2OZ9RbmViG2odBwBjUl1CDb1B0URM0VnJ45ytbtb2Y\/\/3Gw8\/QN1+6j8fH6Akfgf+m886g5We\/SOcuOnfeTe\/I1daXPc\/XeivNNtQ+DALGoOaEGnyDoouaoLGSwxsa2\/jW4u8+\/kP6u39+svDUjho94X95\/UnzgldG\/+q\/j4+ghMZWrhbaewZbe2YmKULtwyBgDKIfavANii5qgsZKDm9V2CphoqZ1mMhzL7xIf\/cvTxY6+yQ+fcOjJxed92r67YvOnhEnrlM7VWErV7vkPIOtDNtQ+7BSCZidO3fSpk2bogjyBY6HDh2i8fFxGhwcpIaGhszIxu9SWr9+PfX396em4UPx1q1bF33OW7NHRkaosbEx0T7U4Mt8Vcy9orEyZ2VrWSa2+pknX\/ru07R\/6rnokkDXg9l0Fuqo+yVnN1DHr51DDb\/w8+Lbi8vE1rZelN0ebGUiFGofVhoBw3ciTU9PU19fH23YsCESHywsBgYGqKmpKVOMcMg5PT+cTp0x09XVRZ2dnXNqhLrtesuWLdEpwCycsoRSqMGX+aqYe0VjZc7K1rJWbPU1J\/ce\/AHd\/eCxKKtF1pzooyc8crLsl14VbS3+nTedM3PMPdu4jqDYsozb14pt0XxWMT3YykQt1D6sFAJGP5V32bJl1NvbGwkRFhfq+oCsEZKkkOuCJv45C5aDBw\/miiKVLtTgy3xVzL2isTJnZWtZlK2+5uQknaQvfOcp+s6RH3oXJ7xr5\/0rm+mHP\/6p+MiJLcM0+6JsfeUjRD9gKxPVUPuwIAVM1jUFaqqpvb09cXQmqfqEGnyZr4q5VzRW5qxsLdPYxhfDHn32x9GoSZG7dvS8qVERHjlZfNbp1PFrr6ULF70qqDNPUG9ta6O5Pdias7KxDLUPK4WA4UCoaRx9CkmNxqRNBaWNvGzbto3SbrhWAmbNmjV022230cTEBNbA2HwTPNqisfII8yVXar3J40cfp0efb6B7DjzjddSEnbE4eWPTGfQff\/VsOu3nXhEdxqamkuo1reOfZLpH1Fs52mArwxYCRobrLK\/6wlr1webNm41HSnRnak1NfAGwEjCHDx+eWbibZqv8cfD1Z8+ePTWgEf4rpqamqLm5OfyCeirhviMvRJ5++rOT9PcP\/oimn32RHn\/uxejfok\/TmadFLs599Wn0pkWvpPbXN0T\/rx71edH3hJAe9VYuimDrh+3q1avnOJqcnPTjvEReSjMC45sJL9Tl0Zzh4WFqaWmZcZ80hZRmqwuYEIPvm7mtP\/y1dYoYX\/rHz6LX\/CLdcvdjNPnk86d+\/9IhbLZcdXt9SudXm86gVcsW0hvPPaMUi2GLlKueaVFv5eiDrQxbjMDIcBXzmrX4l0dclixZMjOywwLmhhtuoK1btyZupQ41+GLwDR2H3lipaRXGMTp+hPYdejYi42P7MPuZESeNDfSmpjNo\/dubZ4TJY489Rm9tfVm4G4YEZgYEQq+3BgjETMBWBm2ofVgpRmDUoltej5L1ZJ3tou86UqMsaduv4+Ima8cS5yfU4Mt8Vcy9Vq2x0g9dU6X87P4n6CsPPC0jTF5aCLvxsiU09cyPrXbpVI2tea2pvyXYysUAbGXYhtqHlULAcMh4Ee+OHTtmHSinn+eydu3azDNh4gfZ6Yt41Wfd3d3R1mx+9PU2aQt+VVUKNfgyXxVzr2VprJKEybdeOtPE1+6cWSMmL1369wdtTfTiT09aCRNTumVha5rfKtmBrVy0wFaGbah9WCkETNa2Z3205KGHHooOrLvzzjtlopziNdTg1xRiwsukG6v4MfU85fKd6R\/S5z0dU68XSZ0Iy+eaXPy6M6PbiF+e4jm95qil2da8QCV6IdjKBQNsZdiG2odBwBjUl1CDb1B0UZOijZV+fw7vzPl\/+456O89EFVxfBPvONzTSpUteE30UCZbG2gsT04AUZWv6nvloB7ZyUQdbGbah9mGlEDAcsrwpJL4SQJ0Vc9NNN8lEGSMwNeWaddiaWgB71hm\/QJ\/42mN0yNPdOUnC5C1LF9DbtBuIyyxMTAOEjsCqqStJAAAgAElEQVSUlL0d2NozM00Btqak7OwgYOx4OVknnQPDlzqq+4puvvlmGh0dnbUt2ulFlolCDb4lBi\/m+sV+\/\/trD9K3H\/9Z5Nf7zpyFp9Mlrz+TetrPm3XIWgjixCQQ6AhMKLnZgK0bN5NUYGtCyd4m1D6sNCMw9iGpXYpQgy9BUAmUf\/vpz+gz9z\/h9dZhzm90Cuy5Z9AH3vG6eSlMTGOGjsCUlL0d2NozM00Btqak7OxC7cMgYAzqQajBNyj6HBP99uG\/ve\/70aFrRQ9c0xfAvuctTfSTF2V25riUt6pp0BHIRQ5swVaOgIznUPuw0ggYPkyup6eHpqen50SwtbV11vZqmRCnew01+Gkl1i\/8e+Doj2j3xBPOIkUXJ5e3vjbancP++f4cdARyNRlswVaOgJxn1FsZtqH2YaUQMPrx\/uq8Fz6zRV3m2N\/fP3N+i0x4s72GGny91Hyc\/Wfu\/z498oTdiIoSKMtfdyZdeO6r6K1LF8y4zVtvgsZKrjaDLdjKEZDzjHorwzbUPqwUAiZ+Dox+1D8v7B0bG6P4pYwyYU72GmLwx+49SmPfetxoZEUJkTVvPJuueGntCY+gFH3QWBUlmJ4ebMFWjoCcZ9RbGbYh9mFMqpQChrdLHzx4kHjkJetOI5lQz\/Va9eCrhbVj9z5OLFyyHhYrq1oW0u9fskj8rBM0VnI1GGzBVo6AnGfUWxm2Ve\/D0qiUQsBw5vT7iHTR8uUvf5nGx8cxAuNQr1m4bP3KQfo\/ex9PTK2mf27pvjD6PG\/KxyELmUnQWPkm+rI\/sAVbOQJynlFvZdhCwMhwnfGqr4PhQ+tY0Gzbto34QsZ6nP2iF7dqwef1LBt2PDCzzVgvC4sUvrX4j9\/+OuGI5rtHY5XPyNUCbF3J5acD23xGrhZg60ouO13V+jBTCqUZgTHNcD3sqhB8Hm356oPH6Mq\/eXAOIhYtt3RdKD4lZBsbNFa2xMztwdacla0l2NoSM7cHW3NWNpZV6MNsyqNsSyFgTC9zbGxsdClj4TRlDr5a37L2E\/fPKieLlnctfy29t\/28mk8NmQJHY2VKyt4ObO2ZmaYAW1NS9nZga8\/MJEWZ+zCT\/KfZQMAY0Ctr8JOmitRoi49dQgZoCpmgsSqELzMx2IKtHAE5z6i3MmzL2ocVLW1dBQzvNtq0aVNuGdavXx\/tSKrXU7bg86gL7yYauuvRGSRVEi4q02is5Go02IKtHAE5z6i3MmzL1of5KmVdBYwqRNYUkq+CFvFTtuAvv\/6bs+4BGn73MvrNN55VpIh1SYvGSg472IKtHAE5z6i3MmzL1of5KmUpBIyvwkj5KUvwecpIX+tSxVEXPUZorKRqLOGaBjm0YAu2ggRkXJelD\/NdOggYA6L1Dj5PGd313aeo\/44DM7n9656L6D+96RyD3JfXBAJGLjZgC7ZyBOQ8o97KsK13HyZTqjqexKumjSYmJnLLNp8vc2TxsvPbR2nzF0+td6n6qAtGYHKruxcDdAReMCY6AVuwlSMg4xkCRoZrJbzWK\/jxxbosXnZ94OLSbou2DSY6Alti5vZga87K1hJsbYmZ24OtOSsby3r1YTZ5dLENZgpJneS7e\/fuiIPpzqUDBw5QX18fDQ8PU0tLSyLDegX\/tnuOUP8dD82MvIQkXrhQaKxcvrJmacDWjJOLFdi6UDNLA7ZmnGyt6tWH2ebT1r5UAiZpW\/XmzZuJrxbIe\/S7lNT0VFdXV2ZaJXr27duXeV1BPYLP26SvGHsgWPECAZNXo4t9jo6gGL+s1GALtnIEZDzXow+TKclsr6URMCxeduzYQSMjI6RO3DUVIkmgdEGTBlJdGsmfl2kEZj6IFwgY2a83Olk5vmALtnIEZDxDwMhwjbz6vkrA5FwZtrn22mupu7s7ujiyLAKG173wOS\/q2f+RtwSz5iVehdARyH2pwBZs5QjIeUa9lWELASPD1buAUbdYd3R00ODgIDU0NCTmnEd8+FmxYoXRGhjdyZ49e0RoTD\/7Iv3JV56ifUdeiPz\/5bsX0SXnnS7yrjI4nZqaoubm5jJkJbg8gK1cSMEWbOUI+PG8evXqOY4mJyf9OC+Rl6CnkKanpxNFDC\/cvf322+nqq68mbozKsoh36K6DM9cDrFy6gHZdcXGJqor\/rOCvLf9MlUewBVs5AnKeUW9l2GIERobrLK9FFvHGs5e1u4hHaVatWkVtbW1Ull1I+tRRaNul06oOGiu5LxXYgq0cATnPqLcybCFgZLiKeVULdPVFwfyyrAP0tm\/fHoma+CMdfBYvG8YeoHseOR69mrdLV+E26aLBQ2NVlGB6erAFWzkCcp5Rb2XYSvdhMrnO91qKKaQiu41UEfVdR2p7dFNTU+4t1mUYgdF3Hc2HqSNMc+R\/MYtaoCMoShDiUI4g2NaaLQSMMPH49FHaaEhaNuIH2emLeNVnvOMoPsJSbwEzH6eOIGCEv0w4JFAUMMShHF6wlWELASPDNdGr2knEH\/IoyujoaOopubXIlmTwP\/b5SfqzPYeiYvSvOZ\/61yypRZFK8Q40VnJhAFuwlSMg5xn1VoatZB8mk2Mzr6WYQsrKKosZXs8SX8tiVjw\/VlLBj4++8Jkv8+lBYyUXbbAFWzkCcp5Rb2XYSvVhMrk191pKAaOPwNT7JmpGKRX8D3\/2YfrkPzwWRevW7gup+9JF5pELwBKNlVwQwRZs5QjIeUa9lWEr1YfJ5Nbca2kETNmmjXSEUsFvvPLu6DW8bXq+jb5wudFYmX9RbS3B1paYuT3YmrOytQRbW2Jm9lJ9mNnb5axKIWBMjv6XQ5DvWSL4w186SINffHTejr5AwOTXuyIW6AiK0MtOC7ZgK0dAxrNEHyaTUzuvpRAwdlmuvbXv4M\/3tS8qgugI5Ooy2IKtHAE5z6i3Mmx992EyubT3CgFjwMx38HUBM18OrUvCjMbKoPI5moCtIziDZGBrAMnRBGwdweUk892HyeTS3isEjAEzn8Fn8bL2E\/cT\/ztf175gBMag0hU0QUdQEGBGcrAFWzkCMp599mEyOXTzCgFjwM1n8PXRl\/\/1u8uo963nGeQgTBN0BHJxBVuwlSMg5xn1Voatzz5MJoduXkshYLIW8abdaeRWXLdUPoO\/9tb7592dR2nU0Vi51UeTVGBrQsnNBmzduJmkAlsTSvY2Pvsw+7fLpYCAMWDrK\/j66Mt8uvMIAsagknk2QUfgGajmDmzBVo6AjGdffZhM7ty91lXAxO8\/SivG+vXrcy9ldEeQn9JX8IfuOkhDd53aOj2fF+8q4ugI8uueqwXYupLLTwe2+YxcLcDWlVx2Ol99mEzu3L3WVcCobM+Xc2DU9NF8X7wLAeP+hTVNiY7AlJS9HdjaMzNNAbampOzsIGDseAVl7SP49zx8PNp9xA9fGcBXB8z3B42VXA0AW7CVIyDnGfVWhq2PPkwmZ8W8lmIEplgR5FP7CP6f\/N0j9OdfPRxlFtNHp2KGxkqu7oIt2MoRkPOMeivD1kcfJpOzYl7rJmD0aaNly5ZRb28vTUxMJJam3hc6+gj+fL\/3KCmwaKyKfXmzUoMt2MoRkPOMeivD1kcfJpOzYl7rJmCKZbu2qYsGX58+2vp7y6inff6e\/aJHDo2VXD0GW7CVIyDnGfVWhm3RPkwmV8W9QsAYMCwa\/Os+P0l\/uudQ9CZMH70MHI2VQeVzNAFbR3AGycDWAJKjCdg6gstJVrQPk8lVca+lEDBqOinUKSTsPkquqGisin+B0zyALdjKEZDzjHorwxYCRoZrplcWNldddRV9+MMfppaWljrk4NQriwRfnz764DsX07UdS+tWjrK9GI2VXETAFmzlCMh5Rr2VYVukD5PJkR+vpRiBySoKXyUwNjZGg4OD1NDQkGp64sQJGhgYoN27d0c2WYffxUd8Ojo6Mv0XCT4Or0uPLhorP1\/iJC9gC7ZyBOQ8o97KsC3Sh8nkyI\/XSgiYoaEhGhkZocbGxtRSsw0\/\/f39pARKV1cXdXZ2zkqjhE57e3v0mfq5qakp9bTfIsHH9BEEjJ+vqp0XdAR2vGyswdaGlp0t2NrxMrUu0oeZvqMedqUXMCxMpqenc0dg4vB0QZMHlq80GB8fT32Ha\/D1u4\/e3rKQPvvHy\/OyMq8+R2MlF26wBVs5AnKeUW9l2Lr2YTK58ee1FAImaxEvj4yMjo5arYGxvZpASsDo61\/45F0+gRfPywTQWMnVBrAFWzkCcp5Rb2XYQsDIcJ3xmja1o6Z6TF\/PIy\/btm2jvHUtyl\/WdJOycQ2+mj5iP9g+PTeCaKxMa7W9HdjaMzNNAbampOztwNaemUkK1z7MxHc9bUoxAsMAkqaKTMRFGjyTqSclmthH1iJhDr7+7NmzxyhmHbdP0fSzL1LTmafR7vc0G6WZT0ZTU1PU3AwuEjEHWwmqp3yCLdjKEfDjefXq1XMcTU5O+nFeIi+lEDBZUz6mu5DiTA8cOEB9fX00PDycOP1kKl7Yr4t61de\/4PLG5BqPv7bkWgKwBVs5AnKeUW9l2Lr0YTI58eu1EgLGZBdSHAsLn7R0JjuPdH8uwcf26fyKisYqn5GrBdi6kstPB7b5jFwtwNaVXHY6lz5MJid+vZZCwMTXv+hFzFtgq2z1XUd5AsVkeqmogNHXvxy78Z1+oxaINzRWcoEEW7CVIyDnGfVWhi0EjAzXGa88YrJx48ZZO454Gqinp4e2bNlCbW1tmTmIH2SnL+JVn3V3d1PazddZN167BB+3T+dXGDRW+YxcLcDWlVx+OrDNZ+RqAbau5DACI0POwiuLmHXr1s1KsX379lzxYvEKJ1NbAaOvf+lfcz71r1ni9N7QE6Gxkosw2IKtHAE5z6i3Mmxt+zCZXPj3WoopJP\/F8uvRNvj6+S\/YPp0eCzRWfuup7g1swVaOgJxn1FsZtrZ9mEwu\/HsthYDRp3jypor8I8j3aBt8ff3L\/o+8hRY3np7\/knlogcZKLuhgC7ZyBOQ8o97KsLXtw2Ry4d9rKQSM7cm5\/jFke7QN\/vLrv0k8jcTChQUMnmQCaKzkagbYgq0cATnPqLcybG37MJlc+PdaCgHDxTLdbeQfQb5Hm+Dr619WLl1Au664OP8F89QCjZVc4MEWbOUIyHlGvZVha9OHyeRAxmspBEzWXUhc7KwdQjJYZnu1Cb6+\/gULeLOjg8ZKrvaCLdjKEZDzjHorw9amD5PJgYzXUggYmaL582oTfBxgZ84djZU5K1tLsLUlZm4PtuasbC3B1paYmb1NH2bmsRxWEDAGcbAJvlrAi\/Uv+WDRWOUzcrUAW1dy+enANp+RqwXYupLLTmfTh8nkQMZr3QSMvnA37XA5VeSqTCFh\/YtdJUVjZcfLxhpsbWjZ2YKtHS8ba7C1oWVuCwFjzio4S9Pg4wA7u9CjsbLjZWMNtja07GzB1o6XjTXY2tAytzXtw8w9lsOybiMw8eLH70PKuh+p1uhMg48D7Owig8bKjpeNNdja0LKzBVs7XjbWYGtDy9zWtA8z91gOy9IImKQLFtU0U1dXF3V2dtaNmGnwP\/2to\/TBHQ9E+cQJvPnhQmOVz8jVAmxdyeWnA9t8Rq4WYOtKLjudaR8m83Y5r6UQMFkH2fH9SGNjYzQ4OEgNDQ1yJDI8mwYfC3jtwoPGyo6XjTXY2tCyswVbO1421mBrQ8vc1rQPM\/dYDstKCBgenRkZGaHGxsa6UDMNvrqBGgfYmYUJjZUZJxcrsHWhZpYGbM04uViBrQu1\/DSmfVi+p3JZlELAZK13KcMJvSbBxwJe+4qNxsqemWkKsDUlZW8HtvbMTFOArSkpOzuTPszOYzmsSyFgGAVPFW3cuJFGR0eppaUlonPgwAHq6emhLVu2UD0veTQJPhbw2ldoNFb2zExTgK0pKXs7sLVnZpoCbE1J2dmZ9GF2HsthXRoBo0TMunXrZpHZvn17XcULZ8Yk+PoJvLiB2qxyo7Ey4+RiBbYu1MzSgK0ZJxcrsHWhlp\/GpA\/L91I+i1IJmPLhOZUjk+BjAa999NBY2TMzTQG2pqTs7cDWnplpCrA1JWVnZ9KH2XkshzUEjEEcTIK\/\/PpvEq+DwQJeA6AvmaCxMmdlawm2tsTM7cHWnJWtJdjaEjOzN+nDzDyVywoCxiAeecHHFQIGEBNM0Fi5cTNJBbYmlNxswNaNm0kqsDWhZG+T14fZeyxHCggYgzjkBR87kAwgQsC4QXJMhY7AEZxBMrA1gORoAraO4HKS5fVhMm+V9zovBYzatr179+6I8Pr166m\/vz+Vdl7wsYDXraKisXLjZpIKbE0oudmArRs3k1Rga0LJ3iavD7P3WI4U81LA8MF4\/LBoMbmuIC\/4V4w9QGP3Ho18YgeSecVGY2XOytYSbG2JmduDrTkrW0uwtSVmZp\/Xh5l5KZ9VaQSMOvNlenp6DqXW1lbRk3h1QZMUorzgYweSW8VGY+XGzSQV2JpQcrMBWzduJqnA1oSSvU1eH2bvsRwpSiFg1JROU1NT5lSOBLKse5jU+\/KCjysE3CKDxsqNm0kqsDWh5GYDtm7cTFKBrQkle5u8PszeYzlSlELAmIgICVw88rJt2zbq6OjIvCySg68\/e\/bsmflx+tkXqeP2qejn9W9eQP\/t1xdIZDVIn1NTU9Tc3Bxk2epdKLCViwDYgq0cAT+eV69ePcfR5OSkH+cl8lIKAaNGYLq7u+ty6i4LGZ66SrvxOku94goB99qMv7bc2eWlBNs8Qu6fg607u7yUYJtHyO1zjMC4cTNOxXch1evWaV5\/09fXR8PDwzP3MOkZzwq+vgNp1wcuppUXYATGNOhorExJ2duBrT0z0xRga0rK3g5s7ZmZpICAMaHkaKOmkCYmJhI9SC\/izRNPWcG\/+rMP01\/8w2NRvrEDya4CoLGy42VjDbY2tOxswdaOl4012NrQMreFgDFnVXpLfdeRyQLirOBjB5J7uNFYubPLSwm2eYTcPwdbd3Z5KcE2j5Db5xAwbtxKmSp+kJ3JIt60BVC4A8k9xGis3NnlpQTbPELun4OtO7u8lGCbR8jtcwgYN25WqXbu3EmbNm2K0mzfvp0OHTpE4+PjmTuErF7gaJwVfGyhdoRKRGis3NnlpQTbPELun4OtO7u8lGCbR8jtcwgYN27GqdROIF5Mu2HDhug8GF77MjAwQPU4H0bPeFrw9R1I\/WvOp\/41S4zLC0MIGMk6gI5Aji7Ygq0cARnPEDAyXCOv+jkwy5Yto97e3kjAtLW1Ud4CW8Fszbg2ETDYgWQfCXQE9sxMU4CtKSl7O7C1Z2aaAmxNSdnZQcDY8bKyhoCxwhWMMRoruVCCLdjKEZDzjHorwxYCRobrjFde\/8LrXfQpJDUa09XVRZ2dncI5SHefFny1AykaRbrxnXXLX1VfjMZKLnJgC7ZyBOQ8o97KsIWAkeE6yytPF61bt27W7zZv3lxX8cKZyRMwixtPj86AwWNHAI2VHS8ba7C1oWVnC7Z2vGyswdaGlrktBIw5q+As04KPHUjFQo3Gqhi\/rNRgC7ZyBOQ8o97KsIWAkeFaCa9JwT987AXiM2D46b50Ed3afWElylKmTKKxkosG2IKtHAE5z6i3MmwhYGS4zvKqnwOjPuDzYHg3Uj2fpOBjC3XxiKCxKs4wzQPYgq0cATnPqLcybCFgZLjOeGXxsmPHDhoZGaHGxsbo92p3UhkX8eojMNhC7VY50Fi5cTNJBbYmlNxswNaNm0kqsDWhZG8DAWPPzDiFvo06PtpS1nNgcAu1cXhTDdFYFWeIERg5hmALtrUnIPNGCBgZrrNGWtThdfqryipgrhh7gMbuPXoq\/9hC7VQ7IGCcsBklAlsjTE5GYOuEzSgR2BphsjaCgLFGZpeAhcrGjRtpdHSUWlpaZgmbMk4h4RZqu\/gmWaOxKs4QowRyDMEWbGtPQOaNEDAyXGcJlYmJidy38P1Id955Z66dT4Ok4OMW6uKEIWCKM0QnK8cQbMG29gRk3ggBI8O1El7jwdcX8K5cuoB2XXFxJcpRtkxCwMhFBGzBVo6AnGfUWxm2EDAyXCvhNUvA4BZq9xCisXJnl5cSbPMIuX8Otu7s8lKCbR4ht88hYNy4WaVKOgemjFcJ6GfA8BUCfJUAHnsCaKzsmZmmAFtTUvZ2YGvPzDQF2JqSsrODgLHjZW1dpXNgePcR70LiB2fAWId6JgEaK3d2eSnBNo+Q++dg684uLyXY5hFy+xwCxo2bUaqqnQODM2CMwpprhMYqF5GzAdg6o8tNCLa5iJwNwNYZXWZCCBgZrpHXqgmYtZ+4n3gaKco7zoBxrhlorJzR5SYE21xEzgZg64wuNyHY5iJyMoCAccJmnqjoFJISQWordkdHBw0ODlJDQ0NiJvT1Nnm28eDjDBjzuGZZorHywzHJC9iCrRwBOc+otzJsIWBkuM7y6rqI98SJEzQwMEDt7e3U2dlJ6uempibi033jj366LwscTptmy2njwccZMH4qAxorPxwhYOQ4gi3Y1paAzNsgYGS4inllMTQ+Pp44ChP\/LMs2LmBwBoy\/kEHA+GMZ9wS2YCtHQM4z6q0MWwgYGa5iXrNESdIIjBq9ScqQHnxdwNz4+2+g\/\/qWJrEyhO4YjZVchMEWbOUIyHlGvZVhCwEjw1XEq1oPk3WH0oEDB6inp4emp6dp+\/btFL8FW8+YHnz9DBgcYlcsfGisivHLSg22YCtHQM4z6q0MWwgYGa7evar1L+w4bRFvfMHw0NBQlI+k9TL8ew6+en6y+K30\/Ir3Rj\/+5bsX0SXn4RA71yBOTU1Rc3Oza3KkyyAAtnLVA2zBVo6AH8+rV6+e42hyctKP8xJ5ecXJkydPlig\/hbJiIl7iC375hTwa09fXR8PDwzM3YaeNwOAMmEIhmpUYf235Yxn3BLZgK0dAzjPqrQxbjMDIcPXmNW\/nkXpRUQHDJ\/DySbz84BqBYuFDY1WMX1ZqsAVbOQJynlFvZdhCwMhw9eaVp4F4PUvW2S\/qZUlTSFlp9eDjDBhvISM0Vv5YYgRGjiXYgm3tCMi8CQJGhqsXr\/FD7JTT1tZWGhkZiQ6z47Neuru7ZxbrsuDZtm1bZGpzkB0EjJeQRU4gYPyxRCcrxxJswbZ2BGTeBAEjw7USXvXgN155d5TnlUsX0K4rLq5E\/suaSQgYuciALdjKEZDzjHorwxYCRoZrJbyq4OtnwHRfuohu7b6wEvkvaybRWMlFBmzBVo6AnGfUWxm2EDAyXCvhNUnA4AyY4qFDY1WcYZoHsAVbOQJynlFvZdhCwMhwrYRXFXz9EDsefeFRGDzuBNBYubPLSwm2eYTcPwdbd3Z5KcE2j5Db5xAwbtyCSJUkYHZ94GJaecGCIMpXr0KgsZIjD7ZgK0dAzjPqrQxbCBgZrpXwqoKPQ+z8hguNlV+eujewBVs5AnKeUW9l2ELAyHCthFcVfBxi5zdcaKz88oSAkeMJtmBbGwIyb4GAkeFaCa8q+DgDxm+4IGD88kQnK8cTbMG2NgRk3gIBI8O1El4hYGTCBAEjw5W9gi3YyhGQ84x6K8MWAkaGayW8quAvv\/6bxGfB4BA7P2FDY+WHY5IXsAVbOQJynlFvZdhCwMhwrYRXFXycwus3XGis\/PLENIccT7AF29oQkHkLBIwM10p45eB\/7dvfJR6B4QeH2PkJGwSMH44YgZHjCLZgW1sCMm+DgJHhWgmvHPy\/\/tJ9tPYT90PAeIwYBIxHmDFXYAu2cgTkPKPeyrCFgJHhWgmvcQGz\/yNvocWNp1ci72XOJBorueiALdjKEZDzjHorwxYCRoZrJbxy8D++c5z4HBh+cAqvn7ChsfLDEdMcchzBFmxrS0DmbRAwMlwr4ZWDv\/4vvkpDdz0KAeMxYhAwHmFiCkkOJtiCbc0IyLwIAkaGayW8cvB\/6+Ofp7F7j0b5xRSSn7BBwPjhiFECOY5gC7a1JSDzNggYGa6V8MrBv+iqz9A9jxyP1r6wgMFTnAAETHGGaR7AFmzlCMh5Rr2VYQsBI8O1El4hYGTChMZKhit7BVuwlSMg5xn1VoYtBIwM10p45eCf+d5P4xRez9FCY+UZqOYObMFWjoCcZ9RbGbYQMDJcK+GVg3\/8XSNRXnGNgL+QobHyxzLuCWzBVo6AnGfUWxm2EDAyXL15PXbsGPX29tLExETks6OjgwYHB6mhoSHxHXv37qV169ZFn7W2ttLIyAg1NjYm2i656M307G8ORZ91X7qIbu2+0Fu+57MjNFZy0QdbsJUjIOcZ9VaGLQSMDFcvXk+cOEEDAwPU3t5OnZ2dpH5uamqi\/v7+Oe84cOAA9fT00JYtW6itrY127txJ4+PjqYJHFzA4A8ZLyCInaKz8scQIjBxLsAXb2hGQeRMEjAxXMa9ZooQ\/O3jwYKK4ScrQ4jf\/Fv1wZV\/0EY++8CgMnuIEIGCKM0zzALZgK0dAzjPqrQxbCBgZrmJe0wRMfLTGJAPN7\/gDen7FeyNTjMCYEDOzQWNlxsnFCmxdqJmlAVszTi5WYOtCLT8NBEw+o9JYqPUwXV1d0ZSS\/igBs2bNGrrtttuiNTN5a2Cafvt\/0Au\/sjZyc8Y9w\/T1nZ8oTVmrnJGpqSlqbm6uchFKm3ewlQsN2IKtHAE\/nlevXj3H0eTkpB\/nJfLyipMnT54sUX4KZ0UJFHaUtIhXfX748OGZhbtDQ0M0PT2dugZGFzA4hbdwiGYc4K8tfyzjnsAWbOUIyHlGvZVhixEYGa5eveaJF35Z0hQSL+rt6+uj4eFhamlpmZOnRb93Pf1k8Vuj3x+78Z1e8zyfnaGxkos+2IKtHAE5z6i3MmwhYGS4evOat\/NIfxGPuCxZsmRmeokFzA033EBbt25N3Er92j\/8JL149htwjYC3aJ1yhMbKM1DNHdiCrRwBOc+otzJsIWBkuHrzmjcNpL+Iz4Bhe3X2C\/8\/P0lbrvn3EDDewjTLERorGa4Qh3JcwRZsZQnIeIeAkeHqxWv8EDvlVC3O5WRecsQAAAzMSURBVMPs+JyY7u7u6NwXfvSD7PIOvTv7j\/6GfvbvzsYpvF6i9bITCBjPQDECIwcUbMG2JgRkXgIBI8O1El4br7w7yieuEfAbLggYvzx1b2ALtnIE5Dyj3sqwhYCR4VoJr0rA4BoBv+FCY+WXJwSMHE+wBdvaEJB5CwSMDNdKeFUCpn\/N+dS\/Zkkl8lyFTELAyEUJbMFWjoCcZ9RbGbYQMDJcK+EVAkYmTGisZLiyV7AFWzkCcp5Rb2XYQsDIcK2EVyVgcA+S33ChsfLLE9MccjzBFmxrQ0DmLRAwMlwr4VUJGNyD5DdcEDB+eaKTleMJtmBbGwIyb4GAkeFaCa9KwOAaAb\/hgoDxyxOdrBxPsAXb2hCQeQsEjAzXSniFgJEJEwSMDFf2CrZgK0dAzjPqrQxbCBgZrpXwqgQM7kHyGy40Vn55YpRAjifYgm1tCMi8BQJGhmslvLKAWdx4OvEUEh5\/BCBg\/LGMewJbsJUjIOcZ9VaGLQSMDNdKeIWAkQkTGisZrphCkuMKtmArS0DGOwSMDNdKeGUBg2sE\/IcKAsY\/U+URbMFWjoCcZ9RbGbYQMDJcK+EVAkYmTGisZLhilECOK9iCrSwBGe8QMDJcK+GVBQzuQfIfKggY\/0wxAiPHFGzBVp6AzBsgYGS4VsIrCxjcg+Q\/VBAw\/pmik5VjCrZgK09A5g0QMDJcK+EVAkYmTBAwMlwxzSHHFWzBVpaAjHcIGBmulfD62j\/8JN30wXdF00h4\/BGAgPHHMu4JbMFWjoCcZ9RbGbYQMDJcK+E11ODXGz4aK7kIgC3YyhGQ84x6K8M21D7sFSdPnjwpgywcr6EGv94RQmMlFwGwBVs5AnKeUW9l2Ibah0HAGNSXUINvUHRREzRWcnjBFmzlCMh5Rr2VYRtqHxaMgDl27Bj19vbSxMREVAM6OjpocHCQGhoaMmvEgQMHqK+vj4aHh6mlpSXRNtTgy3xVzL2isTJnZWsJtrbEzO3B1pyVrSXY2hIzsw+1DwtCwJw4cYIGBgaovb2dOjs7Sf3c1NRE\/f39qRFWdvv27aPR0VEIGLPvgjerUL9U3gAVcAS2BeDlJAVbsJUjIOM51DobhIBJCvnOnTtpfHw8cxRm7969NDQ0FCXHCIzMFyfLa6hfqtqTnPtGsJWLAtiCrRwBGc+h1tl5K2B4yunaa6+l7u7uSMRAwMh8cSBgas+V3xhqg1UfmrPfCrZyUQBbGbahcg1SwKj1MF1dXdGUUtoIDf9+xYoVRmtgZKoVvIIACIAACICAPIHJyUn5l9T4DcEJGLWuhTmmLeLlhbu33347XX311TQ1NZUrYGocE7wOBEAABEAABEAgh0BQAsZEvDAPnjJatWoVtbW1kckuJNQiEAABEAABEACBchEIRsCY7jyKb7fWw7F9+\/ZI1OABARAAARAAARAoN4FgBAyPqkxPTxud\/aKHBCMw5a6gyB0IgAAIgAAIJBEIQsCkjaq0trbSyMhIdJgdnxPDO47iIywQMPhigAAIgAAIgED1CAQhYKqHHTkGARAAARAAARAoQgACpgg9pAUBEAABEAABEKgLAQiYDOy8rmbbtm2RBRb4utVPnqLr6emJ1ifl3U+l8+ZrILKud3DLTVipbNiqksev3QiLiJ\/S2HCNT1+jnciOgQ1b3RbtQbG6rb73Scsoinmub2oImBT+6poBXkPz0EMPRVuv+f8bGxvrG7EKvV3vLNeuXTvrvqp4MeJXP\/DPO3bsAPOUeNuw1V0w102bNtHmzZtTD3msUBXznlUbrvGdj1hPlx0OG7ZKGPJddrxuEe2Be1VX3Hfv3h3cH+IQMCn1Qt2RxF+gUNWr+1fCLGW8QWdRODY2ZrRTDJ1B\/l+y+i3qJmy5U7jqqqvo+PHjlHVKtVl0w7SyqbNse8MNN9DWrVvxh41BdbBlq9dvtAcGgBNM1CjWJZdcQocPH44uNw7pqBAImISgp91urW67dqtK8y+VPorFI1fxn7OIoMHKri8ubFmUX3rppfS5z31u5ub2+Vcr\/XE1uTAWfF8mYFNnk0Zg8i7nBeu5BI4cORL9knfi9vb2QsDMh0qSNOLCjf+SJUsw7G5RAeKjAjZ\/sbqe62ORvUqb2rJV12dceeWV0SWmEOPJ4bfhygLm4MGDkSOslcv\/OtmwZW\/61Mf69eujzhePG4G4IHTzUr5UGIHJGIHRFzxBwNhXXtsGS72BO4abb74Zi3gzkNuw5Y7g4x\/\/OL3nPe+h5ubmzLVI9lEOK4UNV7WeSC3c5bQbN25EvU2pEjZs1dTHli1boikPm9HbsGqkn9JAwPjhWAkvmELyEyabIWOIFzvmNmzZ9utf\/3r0Fyx2IWVztuEan0ICW7C1+xbXzhoCpnasS\/EmfcQFi3jdQhKfMspbaIqdBuacbdjq29P1N2BYfi5vG67x+ox2Irv+2rCFODRvC0wsIWBMKAVkg23UxYNps20Sw+92vG3Y6p4xSpDN2YZrvFPANIc\/tklTSJies2sjdGsIGHd2lU2Jg+yKhy7r4Cq1CJKnNtJGCXAwWHoMTNlCwNjVYxuu+kF2OGwtn7MNWxaE69ati5yCbT7bLAsImGL8kBoEQAAEQAAEQAAEvBHALiRvKOEIBEAABEAABECgVgQgYGpFGu8BARAAARAAARDwRgACxhtKOAIBEAABEAABEKgVAQiYWpHGe0AABEAABEAABLwRgIDxhhKOQAAEQAAEQAAEakUAAqZWpPEeEAABEAABEAABbwQgYLyhhCMQ8E\/gkUceoYULFxLf5m3y8HkPzzzzDC1dutTE3MlGndnT2tpKIyMjxnmryg3jqny1Onuk1u9zCjoSgUAJCUDAlDAoyBIIMAHbk11rcVhVERFSJG0tawQLCn5qeftxVdjUMg54FwjkEYCAySOEz0GgTgTKKGBs86Sjq0onDQFTpwqP14KAJQEIGEtgMAcBnwT0o+jZr5qWeeihh2aOUeffqysV4lcuqGmOs846i3p7e2liYiLKnrqoUd3ts3v37uj3JtM++jv0aRS++mHTpk0zxd+8eTN1dnbOwZFmpwTM2rVr6brrrovSxadp9OPjlWP1HmZ11VVX0dvf\/vYovSrL008\/TT09PTQ9PR0l+ehHP0q7du2i4eFhamlpiX6XVqakWMYFDP\/83HPPRf8pjlkXYcYvIuR3JP2uiuLOZ92HLxAoSgACpihBpAcBRwJJFyvqnWd8tCPthl5+\/eDgILE\/FjE89dHW1kZKHHV1dc0Ijawbv1V+lL+Ghoao47355ptpdHQ0EgN5IzBxe\/1SPhZZLDQuueSSKL\/K\/44dO6K1NCxE+vr6ZgkP3Z8SaYsXL55JHy+j+vnJJ5+M8tzc3EwDAwORUFJTQnkXhyYJmG3btpEupJizzlWvAhAwjl8IJAMBSwIQMJbAYA4CvggkCQzdd55YiP9lHxcwnH5sbGyms2f7rNuok6Z44vZZecq76Tp+wzDnJ29aSf9cCZi4IBsfH59VRl2g8DtuuOEG2rp166zFxlnTREkChkd3lOhin1kcIGB8fUPgBwSyCUDAoIaAQB0J6NMt8emdtE4yPs3S0dGROAITn8rRi5k0\/ZP2Pv3W8KyOO28RcZJYSfpdfFotPk2mRpi4PElCRPfJozrqRuN4mNOmgZIEDKfVF\/VmCS8ImDp+ofDqeUUAAmZehRuFLSsBvdPW18FwZ6q2KitBEl+XokYg4iMwnDY+cpBV\/jRxkjWtpfsrKmDYl1rLogRW0giMjYC57777SE1RmW5Fh4Ap67cE+QKB2QQgYFAjQKBEBHQRoEYYWMDwehFey9He3j5r4awuUuICJmu9S1KRazGFFF\/jor+TxUbWdJCaQtIFTNJohz6FxCMwGzdunFnDYxJqiSmkPDGZN5Vmkm\/YgMB8IwABM98ijvKWhkDSGhh9FERf1Jq0GFWNyKgpJC6YLnKUf17Qq6Y\/ktahKCC+FvHqIx76upgVK1bMWaQbFzB6WpVXzh8vyE0SMKaLeNmHWsOSt\/ao6CJeNcWndo6pcuiLl+OVEAKmNF9LZKRCBCBgKhQsZDU8AqpzU1uA9ekhfQs0T6lcdtlls7ZKs3C5\/PLL6ZprrpkZYYiLGjUqo7ZXM0HVsabRzNpybLqwOGm7tckamPi7t2zZEq1z4YW7qvz6CAyXIc4wvo06vpWc06RtAVejXvyvEn1J26j19Enl0tcfcZyWL19O+\/fvj0RUXGiqMsRHp8Kr7SgRCPglAAHjlye8gQAI1JkAC4qknUem2TJZA2Pqy9QOIzCmpGAHAi8TgIBBbQABEKgsgfg6HzXaop\/7Yls4CBhbYrAHgfoQgICpD3e8FQRAwBOB+OnEWafkmrwyfrniHXfcESWTuhsJlzmaRAU2IDCXwP8HbWE8g10v1boAAAAASUVORK5CYII=","height":337,"width":560}}
%---
%[output:6f3c261a]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"    44"}}
%---
%[output:7d5e9062]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.030000000000000"}}
%---
%[output:71a97270]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_JH","value":"   0.075000000000000"}}
%---
%[output:5cd9d49f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber","value":"     6.215564738292011e-09"}}
%---
%[output:60ed39a0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rsnubber","value":"     3.217728531855956e+03"}}
%---
