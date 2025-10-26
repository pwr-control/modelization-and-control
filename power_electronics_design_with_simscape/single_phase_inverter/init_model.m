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
ld1_load_trafo = 400e-6;
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
kp_inv = 0.25;
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
%   data: {"dataType":"textualVariable","outputData":{"name":"V2rms_load_trafo","value":"     6.600000000000000e+00"}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAArwAAAGmCAYAAACeH1cjAAAAAXNSR0IArs4c6QAAIABJREFUeF7tvQFwV8ed59neYm+YGZG1vclFAjkQo9mKj81oXIlxiHNMHKawLzOZo+wJxrqsyAG1JkRjjxUR71CU47gIVUayJlQULU4EZZgrwPicopaEs7lwc6PgOMKZZfBlyzMXkSETRYYlk5Aggtn1ra5+T+4\/T4\/\/\/\/9ev3+\/1\/3e\/\/OqXJb07\/519+f7e\/RXrX79bpienp5WXBCAAAQgAAEIQAACECgpgRswvCVVlmFBAAIQgAAEIAABCAQEMLwkAgQgAAEIQAACEIBAqQlgeEstL4ODAAQgAAEIQAACEMDwkgMQgAAEIAABCEAAAqUmgOEttbwMDgIQgAAEIAABCEAAw0sOQAACEIAABCAAAQiUmgCGt9TyMjgIQAACEIAABCAAAQwvOQABCEAAAhCAAAQgUGoCGN5Sy8vgIAABCEAAAhCAAAQwvOQABCAAAQhAAAIQgECpCWB4Sy0vg4MABCAAAQhAAAIQwPCSAxCAAAQgAAEIQAACpSaA4S21vAwOAhCAAAQgAAEIQADDSw5AAAIQgAAEIAABCJSaAIa31PIyOAhAoB6B4eFhNTAwMKvIvHnz1L59+1RnZ6cxvHPnzqnVq1errVu3qpUrVxrV7+3tVYcPH67bl2PHjqlt27apQ4cOqdbWVqP4FIYABCDQzAQwvM2sPmOHQJMTEMM7OjqqRkZGVEtLS0BDfnbw4MFUprJRwyvtDw4OVlSRvjzzzDMVA47hbfKEZfgQgEBqAhje1OioCAEIFJ1ANcN7+vRp1dPTo4aGhoxXeW0bXuErK79RI1x07vQfAhCAQN4EMLx5E6c9CEDAGwK1Vnijq77h7QarVq2atQorq64bN24MxrR06VI1OTlZ2dKgDfDExETw+a5du2pudahlbMN9\/O53vztrS0O4bYnf19enNm3aFLQ1NTWlNmzYoE6ePKlkm4b07dKlS8FqtsTZuXNnUE76Jls45Oru7g7KyBUep\/RNfi7\/Sbz29nb1la98Rf3pn\/5pUF++Z5uFN2lNRyAAgSoEMLykBQQg0LQEqu3hjZq3sOEUEyl7dNesWRMYy+iKro4nxvbDH\/5wYDiXL18elI1bOa5leMPbGF577bWK4RXR1q9fr7Zv3x6sREe3O1Tr9\/z58yuGV0y6NuDaHK9bty4w5NG+St+OHz8eGOPFixcH4xJjLyZXrjCTpk0mBg4BCHhNAMPrtTx0DgIQyJJAtRVeMY6bN2+eZe60aZW+1FtxDRvgd7\/73WrLli1q9+7dlQfMxDh2dHRUVmHDYzM1vNGH1sJ7j2U\/cthsx\/U7yjjMQMx0tG\/h77VZDjPKUjNiQwACEEhDAMObhhp1IACBUhCoZnjDBu6+++677tSFcB1Z8QxvfwivlAogvdUhDCu6JUJ\/ZrqlQZta2WIg1\/ve975gG0N41TV8WkQ9oy71w9s2FixYEMTU+5gxvKVIdwYBgaYmgOFtavkZPASam0Cc4ZU9rSYrpXErvPVoVzO82kDLVgQ5vaHW9gZZ7Q1\/ZrrCG93CUG1Lg\/RdnyDBCm9z3zeMHgJFJIDhLaJq9BkCELBCIG5Lg\/w5P8keXr2nVz9EVm0PrzbDumx0ANUMb71jyWQ\/b3Trhd5XKwY4bg9v+DxfMbhi7vv7+4M9vOE9u2xpsJJqBIEABBwTwPA6FoDmIQABdwSqPbQmvYmeplDvlAZtFuUEg9tuuy0YzCOPPBIYx+gpDbW2M0idai+eiD5AV20VV5\/C8LnPfU59\/etfr2xDCJ\/SIHHuvvtu9fd\/\/\/eVh9aiL7AIt\/\/EE08E5fSWCLY0uMtRWoYABOwQwPDa4UgUCEAAAl4TEHM\/Pj4+60g1rztM5yAAAQhYJIDhtQiTUBCAAAR8IRA+ESJuO4UvfaYfEIAABLIigOHNiixxIQABCDgkEN5qId2ot53CYTdpGgIQgEAuBDC8uWCmEQhAAAIQgAAEIAABVwQwvK7I0y4EIAABCEAAAhCAQC4EMLy5YKYRCEAAAhCAAAQgAAFXBDC8rsjTLgQgAAEIQAACEIBALgQwvLlgphEIQAACEIAABCAAAVcEMLyuyNMuBCAAAQhAAAIQgEAuBDC8uWCmEQhAAAIQgAAEIAABVwQwvK7I0y4EIAABCEAAAhCAQC4EMLy5YKYRCEAAAhCAAAQgAAFXBDC8rsjTLgQgAAEIQAACEIBALgQwvLlgphEIQAACEIAABCAAAVcEMLyuyNMuBCAAAQhAAAIQgEAuBDC8uWCmEQhAAAIQgAAEIAABVwQwvK7I0y4EIAABCEAAAhCAQC4EMLy5YKYRCEAAAhCAAAQgAAFXBDC8rsjTLgQgAAEIQAACEIBALgQwvLlgphEIQAACEIAABCAAAVcEMLyuyNMuBCAAAQhAAAIQgEAuBDC8uWCmEQhAAAIQgAAEIAABVwQwvK7I0y4EIAABCEAAAhCAQC4EMLy5YKYRCEAAAhCAAAQgAAFXBDC8rsjTLgQgAAEIQAACEIBALgQwvLlgphEIQAACEIAABCAAAVcEMLyuyNMuBCAAAQhAAAIQgEAuBDC8uWCmEQhAAAIQgAAEIAABVwQwvDmSn5iYyLE1moIABCAAAQhAoJkJtLe3N\/PwZ40dw5tTKojZ3bx5sxobG8upRZqBAAQgAAEIuCNw++23q7lz56qLFy+q119\/3V1HmrjlO++8U\/X39yuMr1IY3pxuhO9973uqq6srSLwFCxbk1GrtZsR479y502p\/GolpWjdJ+bgy9T43\/SyurbwFz6o\/jcQ1rZukfFwZUx1Fp1p14toqg8aNjNG0btLyceVsaRzXTt761svFRvrSyDhN677yyivqBz\/4gfrQhz6k3v\/+91ftdlzMNPrWYhfXViNc09TNqj867v33369eeOEFNTo6iuFVGN40OZqqjja8viSerDjLjfDII4+kGk+1So3ENK2bpHxcmXqfm37WDPqK5nFM6yWTad0k5ePKmOpYb4zNoHEcz7z1TZJztjT2Td8kY0\/zj3eeGh88eFCNj4+rFStWqGXLllXtblx\/0uhbi51vGseNPY2+4bHL6q4stPniO9KOx1Y9VnhtkYyJ49uNltOwm6YZ9C2\/1Ghcbo3R176+hw8fVpcvX1a33nprTcNrv9XaEZtN42Ybb1wuYXjjCFn6XBLvT\/7dV4MVVfbSWIKaQZhbbpqbKupPfzqzR3tmy8r1Dwm85+bkcU3KpuoslVIRYPJIha0wldDXvlQYXvtMTSKS07NpYXhNsqeBspJ4Hz90pYEIVIXANQL1THHUtEfLVqv7npt\/sxI8XF+XxYQrdfbsWfXss8+qrVu3qjlz5pCOJSOAvvYF9c3wNpvGGF4Mr\/27OkFEEi8BJA+K\/OPP30zVizfffFNduHBBtbW1VTVDpnF\/8ov6\/fjHn9f+5SnaVrW2w\/FN+xY2v2KOq5liMdDaOMvnZTDMovEbb7yhbrnlFgxvqrvE70roa18f3wxvs2mM78Dw2r+rE0Qk8RJAKnCRMv1DGjbA+uvZBvma2ZbPo2VqGWhteqsZ4bsW3xSo\/5GOG73NgjJp7C1khx1DX\/vwMbz2mZpExHdgeE3yJXHZ4eFhNTAwEJTv6+tTmzZtmlWXxEuMspAFmSyvl00bX\/m\/GObwqrQ2yjM\/v341W8yxXj2eWSGeWTF2aYjRuJC3ZuJOo29iVIkLYngTo8qkIL4Dw2s9sU6fPq16enrU0NBQEFt\/3dnZWWmLxLOO3auATJaNyRE2xy+f+UUQrJ4p1oZYG2BZIc7aDKNxYxr7Xht97SuE4bXP1CQivgPDa5IvicrK6q6cczcyMqJaWlpUb2+v6ujomLXKS+IlQlnYQkyW2UunDbA2xCfGLwaNvnxm5v9yZWmE0Th7jV22gL726WN47TM1iYjvwPCa5EuismJw5RocHAz+H\/1efkbiJUJZ2EJMlm6lEzMsxle2TVwzxrON8F2LbwwMcdrVYDR2q3HWraOvfcIYXvtMTSLiOzC8JvmSqGx0RVdWfOXtMtoAhw3v\/v37Z53D29ramqgNCvlNgMnST31mjPAvAhMsK8JjP56qdHTG\/F4zwR9a1FJ3EGjsp8a2eoW+tkhei3PkyJHgxRMLFy704sUTZdf43Llzs0SUN7nxprVrSDiH18I9bmJ4o811d3ertWvXWugFIVwSuHr1anAsmfwCwxmtLpWIb3vyV2+pv\/npm0r\/X76Wa\/47Zs7W\/aP3tagPts9VH1gw+2UhaBzPtsgl0Ne+eidOnFBiMuW4xiVLlthvwDBi2TXeu3ev2rdv33VUeLXwDBIMr+ENU624yZaGmTdxLaiEEYPEKq8FERyHuHLlijp\/\/nyweo\/hdSxGiubF\/D5\/6kKwEnzg1WurJGKCP3n7O4OVYDHAaJwCbkGqcA\/bF+ro0aNqampKLVq0SC1dutR+A4YRy66xrPCGV3nHxsbUzp07g2eMeMMrhtfwdqlePLqFgYfWrGAtVJCy\/6msUGJY6Gx4T7AYYH2KhBjgf7OsPfU+YAtdI0RGBLiH7YNlD699piYR2cM7mxYrvCbZU6Msx5JZgFjwEEyWBRcwpvtieP\/ylZ+oS5em1NdOXnsYTlZ+5Ti0B+9oK8Xb5MqtYv3RcQ\/bVx\/Da5+pSUQML4bXJF8Sl+XFE4lRlbIgk2UpZZ01qLDGsgXiwKtvBA\/C6WPR5CG4B+9oZfW3oKnAPWxfOAyvfaYmETG8GF6TfLFWlsSzhtLLQEyWXspitVP1NJ7Z+\/tGsP9Xvg6fB\/zYPe+12g+CZUOAe9g+VwyvfaYmEfEdGF6TfLFWlsSzhtLLQEyWXspitVNJNdbmt9rqL1sfrEpiNVhSfa02WvJgGF63AuM7MLxOMpDEc4I9t0aZLHND7ayhNBpr8xs+\/UFvfcD8OpOyasNp9PVrBP71BsPrVhN8B4bXSQaSeE6w59Yok2VuqJ01ZEPjmVXfX6inXjobjAPz60zO6xq2oa8\/o\/GjJxhetzrgOzC8TjKQxHOCPbdGmSxzQ+2sIdsa69VfzK8zSWc1bFtfP0blthcYXrf88R0YXicZSOI5wZ5bo0yWuaF21lCWGtcyvzzwlp\/cWeqb3yj8agnD61YPfAeG10kGknhOsOfWKJNlbqidNZSXxrW2PWB+s5U+L32zHYVf0TG8bvXAd2B4nWQgiecEe26NMlnmhtpZQy40fuqlf5j1umN50UXX0rbgvF8uuwRc6Gt3BP5Fw\/C61QTfgeF1koEknhPsuTXKZJkbamcNudQ4etQZL7mwnwYu9bU\/Gj8iYnjd6oDvKKHhnZqaUhs2bFAnT568LruWLl2qRkZGVEtLi9PMI\/Gc4s+8cSbLzBE7b8AXjau95EJWfDnmrLEU8UXfxkbhV20Mr1s98B0lMrynT59W3d3dwYj27dunOjs7r8uuY8eOqY0bN6p58+bVLJNHSpJ4eVB21waTpTv2ebXso8Y87GZPfR\/1tTc6N5EwvG6461bxHSUxvOfOnVM7duxQTz75ZKLVW1kFfvzxx9Xg4KCTDCTxnGDPrVEmy9xQO2vId41lv69+uxvn+5qnie\/6mo\/IfQ0Mr1sN8B0lMbxu08i8dRLPnFmRajBZFkmtdH0tisa1tjxwykN93Yuib7rsdVMLw+uGOyu81bnfMD09Pe1WkvStR\/fu7tq1KwgmWxjk8mX\/rvQFw5te5yLUZLIsgkqN9bGIGke3PMhe34903MQpD1VSoYj6NpbR2dfG8GbPuF4L+I4SrfD29vYGo5FtCnqv7qpVq4LvtRmeP3++s20MYdQkntsbP+vWmSyzJuw+fpE1ZtU3Pn+KrG\/86NyUwPC64c4Kb8lWeGUP7\/r169X27duDh9W0wV2+fLnatGlTMFp5qG3Lli1q9+7dqrXV7bmVGF63N37WrTNZZk3YffyyaFztQTfZ7tDsZ\/uWRV\/3d8q1HmB43aqB7yjJCm8tw7tu3Tq1cuVK64ZXnwhx6dKlILZeSdY4h4eH1cDAQPBtX19fxXTzm5bbGz6v1pks8yLtrp0yasyDbtfyqYz6urtbZlrG8LpVAMOL4TXOQDHXq1evVlu3bg3MtP5+zZo1gbEVM9zT06OGhoaC2Prr8DFpJJ4x9kJVYLIslFypOltmjVn1VarM+qZKeAuVMLwWIDYQAt+B4W0gfa5VDe8fltXd0dHRygsu5LOOjo5Zq7wknhXs3gZhsvRWGmsdaxaNZdX3wKvnglca6+PNmuGEh2bR19oNkSAQhjcBpAyL4DtKZHhl1XViYqJuurS3t6tDhw5Z38MbNrzhr6Uz0e\/lZzrx9u\/fr6RP+nK9tzjDe62pQjNZll\/uZtNYr\/o+fXzm31gxv2J8P3n7O0spdrPpm4eIR44cUZcvX1YLFy5Uy5Yty6PJum2UXWP563P4En\/U1dUVLMiFfYdzIRx1oNDHkjliFmxhkDe89ff3B1scoiu6suI7Pj4+63QIbXijfZY4a9eudTUU2rVE4OrVq+rChQvBL1Zz5syxFJUwPhFoZo2fGbuovvl3U2ryV2+p+e+Yo\/7ofS3qoTtv9EmehvvSzPo2DK9GgBMnTgRbRdra2tSSJUuyaiZx3LJrvHfv3uCNstELwztDBMOb+FaZKaj3737wgx+sGFoTwysmecGCBZVWxSCxymsogofFr1y5os6fPx\/8Fo3h9VAgC11CYxUY3udPXVBPvXQ2IPrI77equxbfGJztW\/QLfe0rePTo0eAEpUWLFgXn4ru+yq6x+JPwKu\/Y2JjauXMnK7xvJ15hDa82nllsaRADK3uP5Aq\/vKKa2ZUyJlsa+E3L9T952bRf9j+VZUOtWFHR+JpeZTzXF33t34\/s4bXP1CQie3hn0yqs4Y2KXu1BsejDZCaJEi0bPZkh\/Hl0CwMPrTVCuph1mSyLqZtJr9G4Oq3wCQ\/6IbcH72gL9vwW6UJf+2pheO0zNYmI4S2h4Y2eyauHaOvFE3FvbeNYMpNbsJxlmSzLqWt4VGhcX+NqR5vJyyyKcsID+tq\/hzG89pmaRMTwltDwypD0NoRdu3YFD5JFXzVskiTRstGXTujPw9sdePFEI4SLX5fJsvgaxo0AjeMIXftczK8+3qwoq77om1zfpCUxvElJZVMOw1tSwyvDChvTefPmBU8rhl\/+kE1KJYtK4iXjVNRSTJZFVS55v9E4OStdMrrqKw+4dS1t8\/I1xuhrrm9cDQxvHKFsP8d3lNjwZps6jUUn8Rrj53ttJkvfFWq8f2jcGEPfX2iBvo3pW602htc+U5OI+A4Mr0m+WCtL4llD6WUgJksvZbHaKTS2g9PXh9zQ146+4SgYXvtMTSLiO0pieOVBtR07dqgnn3xStbS0xOaAPHj2+OOPz3oZRGwliwVIPIswPQzFZOmhKJa7hMZ2gVZ7yE0ecJMH3Vxc6GufOobXPlOTiPiOkhheGYbesytf19qvqx9ec72nl8QzuU2LV5bJsniamfYYjU2JJS9\/YvyiOvDqG+rAq+eC48xcnO6Avsn1SloSw5uUVDbl8B0lMrx6KPrYsJMnT16XNeGTFLJJqWRRSbxknIpaismyqMol7zcaJ2eVtmR01fexexapvM70Rd+0qtWuh+G1z9QkIr6jhIbXJAFclSXxXJHPp10my3w4u2wFjfOj7+JNbuhrX18Mr32mJhHxHRhek3yxVpbEs4bSy0BMll7KYrVTaGwVZ+Jg1c70zeJlFuibWJLEBTG8iVFlUhDfgeHNJLHigpJ4cYSK\/TmTZbH1S9J7NE5CKbsy0e0Oep+vrVcYo6997TC89pmaRMR3YHhN8sVaWRLPGkovAzFZeimL1U6hsVWcDQXL4kxf9G1IkqqVMbz2mZpExHdgeE3yxVpZEs8aSi8DMVl6KYvVTqGxVZxWgtnc7oC+ViSZFQTDa5+pSUR8B4bXJF+slSXxrKH0MhCTpZeyWO0UGlvFaTWYje0O6GtVkiAYhtc+U5OI+I6SGt7w0WRyFNnDDz+svvSlL6ndu3er1lY3B5mHUZN4Jrdp8coyWRZPM9Meo7EpMTfl0253QF\/7emF47TM1iYjvKKHh1WZ3+fLlqqOjQ+3Zs0eNjIwEL6MYHR0Nvk7yNjaTRDItS+KZEitWeSbLYumVprdonIaauzqm2x3Q175WGF77TE0i4jtKaHjlNcPr169X27dvV+fPn68Y3jNnzqgtW7Z4scpL4pncpsUry2RZPM1Me4zGpsT8KJ90uwP62tcLw2ufqUlEfEcJDa8Mqbe3V01OTqoHHnhAPffcc8GWhs985jNqxYoVanBw0CRHMilL4mWC1ZugTJbeSJFZR9A4M7S5Ba633QF97cuA4bXP1CQivqOkhleGdezYMbVx48bKCPv6+tSmTZtM8iNRWWln27Zt6tChQ5X9wcPDw2pgYCCoX61dEi8R2sIWYrIsrHSJO47GiVF5X7DadodHfr9NvfHGG+qWW25Rc+bM8X4MRegghtetSviOEhvePFJLtk+sXr06aEob3tOnT6uenh41NDQU\/Fx\/3dnZWekSiZeHOu7awAy5Y59Xy2icF+n82olud\/i3S29U\/\/Zjv6Nu\/e9b8utEiVvC8LoVF9+B4W0oA2Ul95vf\/KaSB+W04ZWfhR+Ok+0V8vBceHWZxGsIu\/eVMUPeS9RwB9G4YYTeBhDj+5ev\/EQdePWcmvzVW0re3qbf5OZtpwvQMQyvW5HwHSUxvHqldWJiom5Gtbe3z9p60Ej6yUqurOJ+\/OMfV3\/xF39RiSsGVy69Vzj6vXymE2\/\/\/v1K+qQvH45Ma4QJdWcIYIbKnwloXG6Ntb7Tv\/0u9fTxn6jnT\/2sYnw\/t+KWcg8+o9EdOXJEXb58WS1cuFAtW7Yso1aShy37PSy+KHyJP+rq6goW5MK+IzmxcpW8YXp6eroMQ6q2qhpdeW10nNLGvffeG4QJ7+GNti3tjo+Pz3pYThveaB+6u7vV2rVrG+0a9R0TuHr1qrpw4UKwp5v9f47FyKh5NM4IrCdho\/rKSu+R16fU105eDHoo2x0+cVuLmv8O9vcmlezEiRPBYkBbW5tasmRJ0mqZlSv7Pbx3797gONboheGdIVIKwxs+liy8b1ZWZG0dSyYPqr344ouBiY0+tGZiePv7+9WCBQsq+SgGiVXezP59yy3wlStXgiPx5LdoDG9u2HNtCI1zxZ17Y7X0FeP7\/KkL6qmXzgZ9EsPb+7F21bW0Lfc+Fq3Bo0ePBtv\/Fi1apOSFUK6vst\/D4oXCq7xjY2Nq586drPC+nXilMLwyFjGdsl9o165dauXKlZUTG1atWmV8LJmOJXHlJpXtC48\/\/njwMJoY6mqGV8om2dLAb1qu\/8nLpv2y\/6ksG2rFiorGxdLLtLdJ9JU9vnK0mez5ZZ9vPGH28MYzyrIEe3hn0y2N4ZVhyYqubBG4dOmSmjdvXrC0H17xTZtY4bjhGLqNl19+edYWBh5aS0u6uPWSTJbFHR09FwJoXO48MNE3fLoDxrd2XmB43d4zGN4SG968Uiu6wsuxZHmR97cdk8nS31HQs3oE0Ljc+ZFG3+ixZo\/ds0g9eEdbsPrLpYK\/uspDa7feeisPrTlICAwvhrfhtOPFEw0jLF2ANJNl6SCUfEBoXG6BG9U3+ha3x+55b3C0WTNfGF636mN4MbxOMpDEc4I9t0YbnSxz6ygNpSaAxqnRFaKiLX1PjF8M9vm+fOZi0+\/zxfC6TX18RwkNb70zeW2ew9tI6pJ4jdDzv66tydL\/kTZvD9G43Nrb1pd9vmxpcH3H4DtKaHhrJZU+N1dObXB9kXiuFci2fduTZba9JXoaAmichlpx6mSlbzPv82WF123+4zuayPDaPIe30bQl8Rol6Hf9rCZLv0fdXL1D43LrnYe+zbbPF8Pr9p7BdzSR4ZU3nh08eNDaq4UbSV0SrxF6\/tfNY7L0n0K5e4jG6GuLgOzz7Tn4eunP88Xw2sqYdHHwHSU0vPX28OoXUaRLF3u1SDx7LH2MhBnyURW7fUJjuzx9i+ZCX9nuoFd9y3ieL4bXbZbjO0poeN2mVLLWSbxknIpaysVkWVRWRe03GhdVuWT9dqlvWR9ww\/Amy72sSuE7Smh4ZYV3\/fr1avv27bPerMYe3qxuI+JGCbicLFEjHwJonA9nV634oG\/ZHnDD8LrK5pl2MbwlMry1XvkbHuLSpUvVyMiIamlpcZp5JJ5T\/Jk37sNkmfkgm7wBNC53Avikb9T4ygss5EUWRXuDG4bX7T2D7yiR4dVDqbXC6zbVZrdO4vmkhv2++DRZ2h8dEYUAGpc7D3zVN3yyw12LbwyM70c6biyEGBhetzLhO0poeN2mVLLWSbxknIpaytfJsqg8few3Gvuoir0++a5v9GSHoTW3eW98Mbz28jNNJHxHSQyvPpnhl7\/8perv71fbtm1TExMT1+UEb1pLc5tQx5SA75Ol6Xgofz0BNC53VhRFX9nu8NkDr1deXSwrvrLlwccLw+tWFQxvSQyv2zQyb53EM2dWpBpFmSyLxNS3vqKxb4rY7U\/R9C2C8cXw2s1R02j4Dgyvac5YKU\/iWcHobZCiTZbegvS4Y2jssTgWulZUfX02vhheC4nZQAh8B4a3gfRJX5XES8+uCDWLOlkWga0vfURjX5TIph9F1zf6Egsf9vhieLPJ1aRR8R0lNbzHjh1TGzduvC4P2MOb9NagXCMEij5ZNjL2ZqmLxuVWuiz6Rld8XRpfDK\/bewbDW0LDqx9gW7Nmjdq0aVNmGTY8PKwGBgaC+NHzfcOf9fX1XdcPEi8zWbwIXJbJ0guYnnYCjT0VxlK3yqZv+FQHOc7sqw\/elvs5vhheS8mZMgy+o6SGt9qb1lLmSNVqsoK8efNmtW\/fvuBtbr29vUG5wcFBJS\/A6OnpUUNDQ8HP9NdSTl8knk01\/ItVtsnSP8Lue4TG7jXIsgdl1TdsfPN+gQWGN8uMjY+N7yih4ZUhyQrr+Ph4YECzuMTgdnR0VF1BlrZHR0crb3SrVpbEy0IVf2KWdbL0h7D7nqCxew2OkeoFAAAgAElEQVSy7EHZ9ZUXWDz10tkA4WP3LApeYJH1heHNmnD9+PiOEhpevaUhq3N4dfytW7eqlStXXpdh4dVe+TD6vfxMJ97+\/fuV7CvWV2urn+cnur1Ni9d62SfL4iliv8dobJ+pTxGbRV8xvk8fnwi2N8iK7+dW3JKZDEeOHFGXL19WCxcuVMuWLcusnaSBy66xeJXwJZ6oq6srWJAL+46kvMpW7obp6enpsg3K9nj0q4sfeOAB9fTTT6tLly7N2sMbXdGtttqsDW+0b93d3Wrt2rW2u0y8nAlcvXpVXbhwQckvMHPmzMm5dZrLgwAa50HZXRvNpO\/kr95SXzt5UR15fUrNf8cc9cQfvFN9YMFc6\/BPnDgRvJK7ra1NLVmyxHp804Bl13jv3r3BtsvoheGdIYLhTXDH6BXe+fPnB9sW5NqwYYOS72ULhYnhlbfCLViwoNKqGCRWeROI4HmRK1euqPPnzwe\/RWN4PRcrZffQOCW4glRrRn3F+D7y\/A+Dt7Y98vut1rc5HD16VE1NTalFixYFi0Sur7JrLF4lvMo7Njamdu7cyQrv24lXCsNbb0uDvsHmzZtXeeAs7qYTAyt7j+SSm3T79u3q05\/+tApvaZCH2OR1xocOHVI7duwIyur9w\/W2NPCbVhz9Yn5e9j+VFVMVu71GY7s8fYvWzPrq\/b2yzcHmMWbs4XWb5ezhnc2\/FIZXhlTtQbHw1gL9YNnBgweNM1B+Q5UV3XXr1lX28Irh3bNnT7DiK39CCD8wx0NrxogLX6GZJ8vCi5dwAGicEFRBizW7vuHze2091IbhdXszYHhLaHj1HltZiQ0fBSbHhW3ZskXt3r07+HOzfP2tb30rVQaGT2KQAGKAly9fHpzawLFkqZCWqlKzT5alErPGYNC43Cqj74y+4dXe\/7Dp9obO7sXwur1nMLwlNLx6hVdurl27dgWrsPrNa6tWrQq2GjSywquRhV8uoeNW+4wXT7i9yV20zmTpgnq+baJxvrzzbg19rxEPr\/bKCyvkNIc0F4Y3DTV7dTC8JTW8MixZaZVTD+QUhfCe3fBKr6sHxEg8ezexj5GYLH1UxW6f0NguT9+ioe\/1iujVXjG8YnxNLwyvKTG75fEdJTa8dlPFbjQSzy5P36IxWfqmiP3+oLF9pj5FRN\/qasib2v54+JSS1xMf+eztRpJheI1wWS+M78DwWk+qJAFJvCSUiluGybK42iXtORonJVXMcuhbWzdteuUUh7\/dmvwFEhhet\/cCvgPD6yQDSTwn2HNrlMkyN9TOGkJjZ+hzaRh962OWfb2y0itXUtOL4c0ldWs2gu8oqeHVD6lFlZcXAchZua727ur+kHhub\/ysW2eyzJqw+\/ho7F6DLHuAvvF0xfT+3rZXgpMbkpheDG880yxL4DtKaHj1iyceffRRJW926enpUYsXL551dFiWSZUkNomXhFJxyzBZFle7pD1H46SkilkOfZPppk1vkj29GN5kTLMqhe8oqeFdv3598EY0eZd0R0dH5XxcfQ4vK7xZ3VLEFQJMluXPAzQut8bom1zfpKYXw5ucaRYlMbwlNLz6TWjz589X9957r9q4caN6+umn1XPPPacmJyfZ0pDFnUTMWQSYLMufEGhcbo3R10xf\/SBbvXN6MbxmTG2XxvCW0PDqIX3hC19Q9913X\/BWNTG9S5cuDV7929LSYjuPjOOReMbIClWBybJQcqXqLBqnwlaYSuhrLpU+p1feyPaRjhuvC4DhNWdqswa+o8SG12ai2I5F4tkm6lc8Jku\/9MiiN2icBVV\/YqJvOi0+8dWZkxuqndGL4U3H1FYtfAeG11YuGcUh8YxwFa4wk2XhJDPuMBobIytUBfRNJ1e9rQ0Y3nRMbdXCd5TE8OqTGSYmJurmBseS2bp1iFOPAJNl+fMDjcutMfqm1\/ezB15XB149p34+ePesIBje9Ext1MTwltDw+mJq6yUoiWfj9vU3BpOlv9rY6hka2yLpZxz0bUyXm3v\/Sj12zyL12D3vrQTC8DbGtNHa+I6SGN7wMMIvnfDV\/JJ4jd66ftdnsvRbHxu9Q2MbFP2Ngb6NaVNtlRfD2xjTRmvjO0poeMND6u3tVXKTycUpDY3eLtRPSoDJMimp4pZD4+Jql6Tn6JuEUv0yssobPqYMw9s400YiYHhLbnj18PQeX\/meVws3cstQNwkBJssklIpdBo2LrV9c79E3jlD857LKKy+l0Cc2YHjjmWVZAsNbYsOrX0Bx8uTJYJSrVq1Sg4OD1vIpvHocjT08PKwGBgaCtvr6+oI3vYUvEs+aDF4GYrL0UharnUJjqzi9C4a+jUsiD66J6f3brcvUe26eG\/y19fLly+rWW29Vy5Yta7yBBiM0m8b4jhIa3npGtMH7o1JdDO3o6GjwIgsx1qtXr1Zr1qypvMK4p6dHDQ0NBeX1152dnZX6JJ4tJfyM02z\/kPqpQra9QuNs+bqOjr52FPi9ba+ouxbfGGxtwPDaYZo2Cr6jJIY3fCyZ7ZXcasklplouvWIc\/j5shuWtbvJZR0fHrFVeEi\/tLVuMekyWxdCpkV6icSP0\/K+LvnY0Cj+8huG1wzRtFHxHCQ1vvWSwdWpDtRXerVu3qpUrVwYGt5YZ1n3Tibd\/\/34lfdJXa2tr2lymnkcEmCw9EiOjrqBxRmA9CYu+doR4\/tTPgm0N3\/93d6jTL\/+fwZaGhQsXsqXBDt66UWQhMHzJewq6urqCv06HfUcOXfGyiRump6enveyZh53Sx5\/NmzdP7du3T+ktC9EVXTHH4+Pjs\/YPa8MbHVZ3d7dau3ath6OlSyYErl69qi5cuKDkF5g5c+aYVKVsQQigcUGEStlN9E0Jrkq1D3zlrHriD96pbvqnv1Xyi0RbW5tasmSJvQZSRiq7xnv37g28SfTC8M4QwfAmvHHE1E5OTgZ7eOXasGGDWr58ebBtwcTw9vf3qwULFlRaFYPEKm9CETwuduXKFXX+\/Pngt2gMr8dCNdA1NG4AXgGqoq89kT40cErdubBF3fPbPwyeeVm0aFFwTKjrq+waywpveJV3bGxM7dy5kxXetxMPw1vlDoye5bt9+3b16U9\/WuktDFJFVnu3bdsWHHm2Y8eOIEq1\/b06PHtpXP9Tl237\/Dk0W74+REdjH1TIrg\/oa4+tbGl4+cxF9cS\/Ps8pDfawGkfCd8xGhuFNkEL6Ablahvcb3\/jGrC0MPLSWAGrJijBZlkzQKsNB43JrjL729NXHk+1Z\/ksMrz2sxpEwvBhe46SRCtW2NMyfPz9Y1T19+nTlKDIpy7FkqRAXuhKTZaHlS9R5NE6EqbCF0NeedCfGL6o\/Hj6lvvj+8+rm\/+7\/4xxee2iNImF4MbxGCaMLx73UghdPpMJamkpMlqWRsuZA0LjcGqOvXX3lNcOfWnRR3fkvf43htYs2cTQML4Y3cbLYLEji2aTpXywmS\/80sd0jNLZN1K946GtXD3kBRdsNPw9ML29as8s2aTR8B4Y3aa5YLUfiWcXpXTAmS+8ksd4hNLaO1KuA6GtXDnlw7Uc\/+hGG1y5Wo2j4DgyvUcLYKkzi2SLpZxwmSz91sdkrNLZJ079Y6GtXEzG8L53+abCPlxVeu2yTRsN3YHiT5orVciSeVZzeBWOy9E4S6x1CY+tIvQqIvnbl0Cc1fOUDkxheu2gTR8N3YHgTJ4vNgiSeTZr+xWKy9E8T2z1CY9tE\/YqHvnb10IZXVng\/+D5eLWyXbrJo+A4Mb7JMsVyKxLMM1LNwTJaeCZJBd9A4A6gehURfu2KEjybD8NplmzQavgPDmzRXrJYj8azi9C4Yk6V3kljvEBpbR+pVQPS1L4c+muzBO1rVsmXL7DdgGLHZNMZ3YHgNbxE7xUk8Oxx9jdJs\/5D6qkOW\/ULjLOm6j42+9jXA8NpnahIR34HhNckXa2VJPGsovQzEZOmlLFY7hcZWcXoXDH3tSyJn8S75jQuq5yPvYoXXPt7YiPgODG9skmRRgMTLgqo\/MZks\/dEiq56gcVZk\/YiLvvZ1+JMv\/7W6fHlKPf6xmzG89vHGRsR3YHhjkySLAiReFlT9iclk6Y8WWfUEjbMi60dc9LWvgxjen\/ziivryvf8Cw2sfb2xEfAeGNzZJsihA4mVB1Z+YTJb+aJFVT9A4K7J+xEVf+zr8+e5j6pn\/9M\/Vtz45F8NrH29sRHwHhjc2SbIoQOJlQdWfmEyW\/miRVU\/QOCuyfsRFX\/s67Dn0TfUb\/\/WXvHjCPtpEEfEdGN5EiWK7EIlnm6hf8Zgs\/dIji96gcRZU\/YmJvva1OHz4sLp8+TKG1z7aRBHxHRjeRIliuxCJZ5uoX\/GYLP3SI4veoHEWVP2Jib72tcDw2mdqEhHfgeGNzZdjx46pPXv2qJGREdXS0lIp39vbq+QGlmvXrl1q5cqVlc+Gh4fVwMBA8H1fX5\/atGnTrHZIvFjshS5w9uxZ9eyzz6oNGzao9vb2Qo+FzlcngMblzgz0ta+vb4a32TTGd2B4697VYnY3btyoli5dOsvwys+3bdumDh06pF577bXK162trer06dOqp6dHDQ0NBbH1152dnZW2SDz7\/5j6FBF9fVIjm76gcTZcfYmKvvaV8M3wNpvGzTbeuAy+YXp6ejquULN8Liu4x48fD8zupUuXZhle+UyuwcFBNTU1FazkrVu3LljlldXd0dHRSnkp29HRMWuV17fEm5iYUC+88IK6\/\/77ra1INhLTtG6S8nFl6n1u+lkz6Cv5H8e03r8VpnWTlI8rY6pjvTE2g8ZxPPPWN0nO2dLYN32TjD3N3JynxgcPHlTj4+PqrrvuUnfffXfV7sb1J42+tdj5pnHc2NPoGx67\/LVx8+bNgT\/hL49KYXhDGSWruNUMrDa4y5cvD0xs9PuwGZZw0e\/lZ77daFn0p5GYpnWTlI8rU+9z08\/i2kr7D1faeln1p5G4pnWTlI8rY6pjvXs1rq20WqWtl0V\/GolpWjdp+bhytjSOayetTo3Uy6JPjcQ0rSvbvL7zne+oT33qUzUNb1zMNPrWuo\/j2mpEqzR1s+qPjtvf34\/hDQmD4a2SpdEV2+iKrja1ehU3uqIr9eW3WlkN1pdOwP3793vxm5b8ZtnV1aVs9qeRmKZ1k5SPK1Pvc9PP4tpK849hI3Wy6k8jcU3rJikfV8ZUR706Uu3eiGurEb3S1M2iP43ENK2btHxcOVsax7WTRqNG62TRp0Zimtb99re\/HRjej370o3VXeOvNRWn0rXUfm\/a\/Uf3i6mfVHx0XwztbAQxvToZXElD+tDA2NhZ3D\/A5BCAAAQhAoPAEFi1aFIzh4sWLwX9c+RO488471YEDB\/Jv2MMWm9LwykNm3d3dwT5duaqduBDek2tjS4P+jVOMLxcEIAABCEAAAhDImoDs3WX\/7gzlpjS8cQkW3dIg5cPbFqo9tBbewlDtobW4NvkcAhCAAAQgAAEIQCAbAhjeKlyrGd5GjyXLRj6iQgACEIAABCAAAQjEEcDwJjS8epU37Ysn4oTgcwhAAAIQgAAEIACBbAhgeLPhSlQIQAACEIAABCAAAU8IYHg9EYJuQAACEIAABCAAAQhkQwDDmw1XokIAAhCAAAQgAAEIeEIAw+uJEHQDAhCAAAQgAAEIQCAbAhjebLgSFQIQgAAEIAABCEDAEwIYXk+EoBsQgAAEIAABCEAAAtkQwPBmw5WoEIAABCAAAQhAAAKeEMDweiIE3YAABCAAAQhAAAIQyIYAhjcbrkSFAAQgAAEIQAACEPCEAIbXEyHoBgQgAAEIQAACEIBANgQwvNlwJSoEIAABCEAAAhCAgCcEMLyeCEE3IAABCEAAAhCAAASyIYDhzYYrUSEAAQhAAAIQgAAEPCGA4fVECLoBAQhAAAIQgAAEIJANAQxvNlyJCgEIQAACEIAABCDgCQEMrydC0A0IQAACEIAABCAAgWwIYHiz4UpUCEAAAhCAAAQgAAFPCGB4PRGCbkAAAhCAAAQgAAEIZEMAw5sNV6JCAAIQgAAEIAABCHhCAMPriRB0AwIQgAAEIAABCEAgGwIYXktch4eH1cDAQBCtr69Pbdq0yVJkwkAAAhCAAAQgAAEINEIAw9sIvbfrnj59WvX09KihoaHgJ\/rrzs5OC9EJAQEIQAACEIAABCDQCAEMbyP03q4rq7ujo6NqZGREtbS0qN7eXtXR0cEqrwW2hIAABCAAAQhAAAKNEsDwNkpQqcDgyjU4OBj8P\/q9bmJiYsJCa4SAAAQgAAEIQAAC8QTa29vjCzVJCQyvBaGjK7qy4js+Pl4xwNKEmN3PfeEpNTY2FtviP\/v1z2LLUAACEIAABCDgM4HW1lY1d+5c9eabb6pz58753NXS9u3OO+9U\/f39CuOrFIbXQponMbzf+9731McPXbHQWu0QtYzyP\/v1P82q9J6b5wbfiwmXm+CWm2a+D1+6jPws\/HX4+2g9MfM7d+4Mbq4FCxYYjdW0bpLycWXqfW76WVxbRjAsFM6qP43ENa2bpHxcGVMdBX2tOnFtWZDNKEQW\/WkkpmndpOXjytnSOK4dI3EsFc6iT43ENK37yiuvqB\/84AfqQx\/6kHr\/+99flUpczDT61rqP49qyJFviMFn1R8e9\/\/771QsvvBBsucTwYngTJ2a9gkm2NIjhXf3wF982g\/X\/xPCTX7xp3K9\/\/HltM\/2PP58dT77\/6U8n1MTET5X89hdtL1o+aWfEGEdNdC3j\/J6bfzMIq02zmPWuri514sVvJLoxhaeUr3cjx5Wp97npZ3FtJWVoq1xW\/WkkrmndJOXjypjqKPxr1Ylry5Z2SeNk0Z9GYprWTVo+rpwtjePaSaqLzXJZ9KmRmKZ1n332WfWd73xHfepTn1J33313VTRxMdPoW+s+jmvLpnZJYmXVHx1XFp82b96M4X1bDFZ4k2RlTJnoFoZqD61lldhpuy\/GVH7ze+SRR2JDVDPMupI2y1HDreuE614rG2\/ow0ZZTLH+Xv9fVq1P\/l\/fCvovP4uuQkv\/4sZY73PTz4qsb2wChArEMa0Xy7RukvJxZUx1rJc3zaBxHM+89c3qPq42Tt\/0TTJ2k3tXl81T44MHDwbb+1asWKGWLVtWtbtx\/UlzD9di55vGcWNPo2947LKgFbcwlLaNItbD8FpQLcmxZL7daBaG3XCIsBnWX1cz0PJZ+PN6K9Da+OqV47AZllVlbZ6rGeRGBoS+jdArRl00LoZOaXuJvmnJ1a53+PBhdfnyZXXrrbfWNLz2W60dsdk0brbxxuUShjeOUMLP4148QeIlBGlQLLqKLGZZrzQnMclhc1zNGH+k48bEvTl79qySP99t3bpVzZkzJ3E9ChaHABoXR6s0PUXfNNTq1\/HN8DabxviO2fmJ4bV\/j1eNSOLlBLpOM2GDXM0cz\/zs+u0WYobDK8N6pThsiOUp5DfeeEPdcsstGF73UmfSAzTOBKs3QdHXvhS+Gd5m0xjfgeG1f1cniEjiJYDkSZGwMX75zC+CXukV42qmWBviJf9SqZtuulHdtfgmZbI67Mmw6UYMgWabLJstIdDXvuIYXvtMTSLiOzC8JvlirSyJZw2lF4G0AdaG+P\/+u5+pt956S\/3NT2evEIsZvmvxjcFDdWKE5cIMeyGhcScwRMbIClUBfe3LheG1z9QkIr4Dw2uSL9bKknjWUHoZKDxZTv7qrWBFWJvhE+MX1ctnLs7qd9QIY4K9lHVWpzBE\/mvUSA\/RtxF61etieO0zNYmI78DwmuSLtbIknjWUXgZKMlnqrRIHXn0jGEPUCOutEWJ+2Rbhn8xJNPav1\/QoKQH0TUoqeTkMb3JWWZTEd2B4s8ir2JgkXiyiQhdoZLIMb4\/ABPubBo1o7O+o6JkmgL72cwHDa5+pSUR8B4bXJF+slSXxrKH0MpDtyTLOBD94R2vA4bF73usljzJ2yrbGZWRU5DGhr331MLz2mZpExHdgeE3yxVpZEs8aSi8D5TFZigmuth1CtkJggLNPizw0zn4UtFCLAPrazw0Mr32mJhHxHRhek3yxVpbEs4bSy0AuJsvwKvCBV89VzhDWBljOC9ZG2EtoBeuUC40LhqjQ3UVf+\/JheO0zNYmI78DwmuSLtbIknjWUXgbyYbKcORniYvC2uWoG+ME72oLj0bjSEfBB43Q9p1YSAuibhJJZGQyvGS\/bpfEdGF7bOZUoHomXCFNhC\/k4WeotEOEH4fTqL6dAmKeajxqbj4IatQigr\/3cwPDaZ2oSEd+B4TXJF2tlSTxrKL0MVITJUhtgvfrL3l+zVCqCxmYjonSYAPrazwcMr32mJhHxHRhek3yxVpbEs4bSy0BFmyyrrf7KG+HkDGBOfqieYkXT2MsbxeNOoa99cTC89pmaRMR3YHhN8sVaWRLPGkovAxV5ssT8JkupImucbITNXQp97euP4bXP1CQivgPDa5Iv1sqSeNZQehmoLJNl1Pyy5\/daupVFYy9vIA86hb72RcDw2mdqEhHfgeE1yRdrZUk8ayi9DFTGybLWnt9m3fJQRo29vJkcdQp97YPH8NpnahIR34HhNcmXStlz586p1atXq4mJicrP+vr61KZNm4Lvh4eH1cDAQPB1+Oe6MImXCnthKpV9stTm96mXzgaa6JXfZjK\/Zde4MDdbRh1FX\/tgMbz2mZpExHdgeE3ypVL29OnTasuWLWr37t2qtXXmta76ks96enrU0NBQ8CP9dWdnZ6UMiZcKe2EqNdNk+dRL\/6D0UWfa+DbDGb\/NpHFhbjyLHUVfizDfDoXhtc\/UJCK+A8Nrki+VsseOHVN79uxRIyMjqqWlZVYMWd0dHR2tfNbb26s6Ojoqq79SmMRLhb0wlZpxsoyu+sopD11L20r7drdm1LgwN6CFjqKvBYiREBhe+0xNIuI7MLwm+VIpGzW14SBicOUaHBwM\/h\/9Pmx49+\/fr9rb2yvVo6vFqTpHJecEmn2ylFXf50\/9LHi9sV71\/dyKW5zrYrMDza6xTZY+xkJf+6ocOXJEXb58WS1cuFAtW7bMfgOGEcuusWy9DF+yBbOrqytYkAv7DkNspSl+w\/T09HRpRpPhQMTEym+r+lq6dGnNFV0xx+Pj4xUDHDa80S52d3ertWvXZthzQudB4OrVq+rChQvBdpc5c+bk0aSXbUz+6i115PUp9bWTF9X8d8xRf\/S+FvWJ21qCr4t+oXHRFazff\/S1r++JEyeUmMy2tja1ZMkS+w0YRiy7xnv37lX79u27jgqGdwYJhjfBDTM1NaU2bNig5s+fH5jY6PfRLQz1DG9\/f79asGBBpVUxSKzyJhDB8yJXrlxR58+fD36LbmbDq2US4\/v8qQtKP+T2yO+3qqLv80Vjz2\/CBruHvg0CrFL96NGjwXy5aNEiJYtErq+yaywrvOFV3rGxMbVz505WeN9OPAxvlTswvJobXskNF5U9vdu2bVOHDh1SO3bsCD5KsqWB37Rc\/5OXTftl\/1NZI9Rku4N+nbHs85WTHeSNbkW70Lhoipn1F33NeCUpzR7eJJSyK8Me3tlsMbwpcy38EJv8CSG8hYGH1lJCLXA1Jst48cLGV\/b5ivF98I7ZJ57ER3FXAo3dsc+jZfS1TxnDa5+pSUQML4bXJF+CsvoM3q1bt6qVK1dWvl+zZk1wEgPHkhkjLV0FJsvkksqDbZ898Lp6+czF4AG3ohhfNE6ucRFLoq991TC89pmaRMTwYnhN8qVSNvriiVWrVs16KI0XT6TCWppKTJbmUorx1au+RTC+aGyucZFqoK99tTC89pmaRMTwYnhN8sVaWRLPGkovAzFZppelKCu+aJxe4yLURF\/7KmF47TM1iYjvwPCa5Iu1siSeNZReBmKybFyWqPEdWnObVw+3oXHjGvscAX3tq4Phtc\/UJCK+A8Nrki\/WypJ41lB6GYjJ0p4svhpfNLansY+R0Ne+Khhe+0xNIuI7MLwm+WKtLIlnDaWXgZgs7csixvePh08Fb2+T48y++uBtwUNuri40dkU+n3bR1z5nDK99piYR8R0YXpN8sVaWxLOG0stATJbZySJn+MrDbWJ8H7tnUXCqg4sLjV1Qz69N9LXPGsNrn6lJRHxHCQ2vfvPZyZMnr8uFWi+OMEkaG2VJPBsU\/Y3BZJm9NmJ65c1tssor5\/fmbXzROHuNXbaAvvbpY3jtMzWJiO8okeGV82+7u7uDEcnLHzo7O6\/LBXlBxMaNG9W8efNqljFJoLRlSby05IpRj8kyH51klffAq284Mb5onI\/GrlpBX\/vkMbz2mZpExHeUxPDKubjySt8nn3xStbS0xOaArAI\/\/vjjs87Oja1ksQCJZxGmh6GYLPMVJWp88zjRAY3z1Tjv1tDXPnEMr32mJhHxHSUxvCai+1CWxPNBhez6wGSZHdt6kcMnOmT9YBsau9E4r1bR1z5pDK99piYR8R0YXpN8sVaWxLOG0stATJZuZTkxflH1HHw90wfb0Nitxlm3jr72CWN47TM1iYjvKKHhjb72t1pCsIfX5DahrCkBJktTYtmUz\/LBNjTORjNfoqKvfSUwvPaZmkTE8JbQ8MqQent7VUdHh9q0aVNlhMPDw2p8fDzYtytfj46OqoMHD5rki7WyJJ41lF4GYrL0R5as9veisT8aZ9ET9LVPFcNrn6lJRHxHCQ2vrPCuX79ebd++fdZJDXKKw5YtW9Tu3bvV+fPng6+\/9a1vmeSLtbIknjWUXgZisvRPFtv7e9HYP41t9gh9bdKciYXhtc\/UJCK+o4SGV6\/wys21a9cutXLlSqWPI1u1ahUrvCZ3CGVTEWCyTIUtl0qyv1fe2CZXIy+uQONc5HLWCPraR4\/htc\/UJCKGt6SGV4alz+W9dOnSrHN3wyu9ra2tJvlirSyJZw2ll4GYLL2UZVanwvt70xxjhsb+a9xID9G3EXrV62J47TM1iYjvKLHhNUmEvMuSeHkTz7c9Jst8eadtrZFtDmiclnox6qGvfZ0wvPaZmkTEd2B4Y\/NFtqBpP6gAACAASURBVEPs2bNHjYyMzHqphTwYJzewXHrrhA4mD8UNDAwE3\/b19c16eE5+RuLFYi90ASbLYsmX5hgzNC6Wxqa9RV9TYvHlMbzxjLIsge8oqeGVN6lt2LBBnTx5Ui1dulQ9\/PDD6ktf+lLwwJrJNga991dihA2v\/Hzbtm3q0KFD6rXXXqt8LbFly0RPT48aGhoK6Oqvw686JvGyvK3dx2aydK9Bmh6YHGOGxmkIF6cO+trXCsNrn6lJRHxHCQ2vNrvLly8PjibTq7P79u0LjiKLrtTWShhZwT1+\/HhgmGUfcLiefCaXHHGm21u3bl3wgJw+8kyXr3ZEGolncpsWryyTZfE00z1OeowZGhdX4yQ9R98klMzKYHjNeNkuje8ooeENH0smx49pw3vmzJnKsWRJVnllFbeagQ0bajnnN\/p92AwL3uj38jMSz\/at7Fc8Jku\/9EjTm\/D+3mqnOaBxGqrFqYO+9rXC8NpnahIR31FCw6tN5uTkpHrggQfUc889F2xp+MxnPqNWrFgRrMqaXNEV2+iKrm5Pv+giuqIbfuGFblcn3v79+1V7e3ulO0mMuEnfKeuGAJOlG+5ZtCrbHJ4+PqHec\/Nc9eAdrepzK24JmkHjLGj7ExN97Wtx5MgRdfnyZbVw4UK1bNky+w0YRiy7xrL4F74mJiZUV1dX8JfusO8wxFaa4jdMT09Pl2U0ev+tHk+1h8eSjDVLwxttv7u7W61duzZJtyjjMYGrV6+qCxcuBPvF58yZ43FP6VoSApO\/eksdeX1Kfe3kRTX\/HXPUE3\/wTvWv33kDGieBV9Ay3MP2hTtx4kTwi2JbW5tasmSJ\/QYMI5Zd47179yrZyhm9MLwzREpleJPmfvi8XqlT7cSF8N5fm1sa+vv71YIFCypdFYPEKm9S5fwtd+XKleBtfvJbNIbXX51MeybG95Hnf6hePnNRfWDBXPXn\/+M8tfR\/WITGpiALUJ572L5IR48eDbYALlq0KHg2xvVVdo1lhTe8yjs2NqZ27tzJCu\/bideUhjfupouu8Er58LaFag+tjY+PV7ZO8NBaHOHyfV72P5WVTzGzEaU5xsysBUq7JsA9bF8B9vDaZ2oSkT28s2kV1vDKbzGrV69Wskel3iUrbnKUmMkqajXDy7FkJrdZ85Vlsiy\/5qLxE4dfD7Y56P29j93z3vIPvElGyD1sX2gMr32mJhExvCUxvFHRq62qVjOuSZKlVj1ePJGEXnOWYbIsv+5a4+nffpd6\/tQF9dRLZwPjm+Y1xeWnVbwRcg\/b1wzDa5+pSUQMbwkNb\/hYsvDLHmSv7pYtW4xfPmGSUEnLknhJSRWzHJNlMXUz6XVU40ZeU2zSLmXzIcA9bJ8zhtc+U5OI+I4SGl4Zkl591Q+g6RMbVq1aZXwsmUlCJS1L4iUlVcxyTJbF1M2k17U01m9rk1jVzu81aYOy7ghwD9tnj+G1z9QkIr6jpIZXhhU+fWHevHnB8RzhFV+TRLFdlsSzTdSveEyWfumRRW\/iNDZ5TXEW\/SNmYwTi9G0senPWxvC61R3fUWLD6za16rdO4vmsTuN9Y7JsnKHvEZJoHH1NsTzUJi+v4PKfQBJ9\/R+FXz3E8LrVA9+B4XWSgSSeE+y5NcpkmRtqZw2ZaBze38uDbc4kM2rYRF+jwE1cGMPrVnx8R0kMrzyotmPHDvXkk0+qlpaW2KySs3Mff\/xxZ\/t5SbxYiQpdgMmy0PIl6nwajcX4\/vHwKSX\/v2vxjeqrD94WnOzA5R+BNPr6Nwq\/eoThdasHvqMkhleGoffsyte19uvqh9dc7+kl8dze+Fm3zmSZNWH38RvROPziCoyvey2r9aARff0ckfteYXjdaoDvKJHh1UPRbz47efLkddklrzMcGRlJtAqcZWqSeFnSdR+bydK9Bln3wIbG8mDbgVfPBSu+nOiQtWJm8W3oa9Zi+UtjeN1qjO8ooeF1m1LJWifxknEqaikmy6Iql7zfNjXmKLPk3PMqaVPfvPrsezsYXrcK4TswvE4ykMRzgj23Rpksc0PtrKEsNMb4OpPzuoaz0Nef0bnpCYbXDXfdKr4Dw+skA0k8J9hza5TJMjfUzhrKSuPwUWYyONnq8OAdbTzclrPSWemb8zC8ag7D61YOfAeG10kGknhOsOfWKJNlbqidNZSHxtEVX4xvfnLnoW9+o\/GjJQyvWx3wHRheJxlI4jnBnlujTJa5oXbWUJ4aRx9uw\/hmL3ue+mY\/Gj9awPC61QHfgeF1koEknhPsuTXKZJkbamcNudA4bHzl\/F7e3Jad\/C70zW40fkTG8LrVAd9RUsMbPppMjiJ7+OGH1Ze+9CW1e\/du1drq\/tWeJJ7bGz\/r1pkssybsPr5LjcPn+IrxldcVi\/nlskfApb72RuFXJAyvWz3wHSU0vNrsLl++XHV0dKg9e\/YEZ+\/KyyhGR0c5h9ftPdcUrTNZll9mHzQOP+CG8bWbcz7oa3dE7qNheN1qgOEtoeGV1wyvX79ebd++XZ0\/f75ieM+cOaO2bNnixSovief2xs+6dSbLrAm7j++TxpzsYD8ffNLX\/ujcRMTwuuGuW8V3lNDwypB6e3vV5OSkeuCBB9Rzzz0XbGn4zGc+o1asWKEGBweNsk5eR6xXiVtaWoK6YqpXr16tJiYmKrH6+vrUpk2bgu+Hh4fVwMBA8HX45ySeEfrCFmayLKx0iTvuo8ba+Oq3t7Hqm1jO6wr6qG\/60fhRE8PrVgcMb0kNrwxLjOrGjRurGtKkaadjRF9JfPr06ZqrxfJZT0+PGhoaCprRX3d2dlaaJfGSKlDMckyWxdTNpNe+ayzmVz\/kpo0vpzskV9h3fZOPxJ+SGF63WuA7Smx4G00tWSU+fvy4ErN76dKlWXt\/q6366vZkdTe8V1jiyF5ivfor5Ui8RtXxuz6Tpd\/62OhdUTSObndg1TeZ+kXRN9lo\/CiF4XWrA74Dw1szA8XUrly5MtieEH3YrdrPdCAxuHLprRPR78OGd\/\/+\/aq9vb3SBx9OkHB7S5ajdSbLcuhYbxRF1FhOdzjw6hvq+VM\/C97cJqc73LX4JvWhRTNbtbiuESiivr7rd+TIEXX58mW1cOFCtWzZMufdLbvGsvUyfMkWzK6ursDPhH2HcyEcdeCG6enpaUdtW21WzOq2bdvUoUOH1De+8Y1gP+28efOCkxrCWwuSNFrN3IqJld9W9RXe8hBd0ZX64+Pjs\/YO69+0ou13d3ertWvXJukWZTwmcPXqVXXhwoXgCLw5c+Z43FO6lpZAkTWe\/NVb6sjrU+qbfzel5Ov575ij\/uh9LeoTt7UEX3MpVWR9fdXvxIkTSkxmW1ubWrJkifNull3jvXv3Bp4nemF4Z4iUwvDqY8nWrVunfvd3fzd4uGzr1q3BAKMPnyW546KGV8efP39+YGKj35sY3v7+frVgwYJKN8QgscqbRBW\/y1y5ciU4IUR+i8bw+q1V2t6VRWMxvM+fuqCeeulsgEIM7ydvf2fTn+tbFn3T5ncW9Y4ePRrMl4sWLQq2Crq+yq6xrPCGV3nHxsbUzp07WeF9O\/FKYXijx5LplV4xINWOJZOHzGRlVfbpyrVr165gK4O+6m1f0GXCK8o7duwIfpxkSwO\/abn+Jy+b9sv+p7JsqBUrahk1lofcZvb8zvwp9K7FN6qPdNzYlOa3jPq6vsPYw+tWAfbwzuZfCsMbXuGVrQR6\/23aF08kNbzhF1yEtzDw0Jrbm9xF60yWLqjn22aZNdYPusme35fPXGxK81tmffO9U661huF1RX6mXQxvCQ2vDEkfJ6b37crPqh0PliT9ooZXn8Er2yRkJVh\/v2bNmuAkBo4lS0K13GWYLMutr4yuWTSuZn7lYTd56K3MrzNuFn3zvFMxvHnSvr4tDG9JDa\/NtKq2wht98cSqVatmPZTGiydsKlC8WEyWxdPMtMfNqHE181vWY86aUV\/Te8C0PIbXlJjd8hheDK\/djEoYjcRLCKqgxZgsCyqcQbebXeN65rcML7hodn0NboXERTG8iVFlUhDfUVLDG33Lmh6mPDUvR5W5PgmBxMvkfvYmKJOlN1Jk1hE0voZWm1\/5iT7tIXzOrzz4VrQLfe0rhuG1z9QkIr6jhIZXbzd49NFHlRyDInt3Fy9erDZs2KCWL18+641nJslisyyJZ5Omf7GYLP3TxHaP0Lg2UW2A5bQH+Vquop34gL627xgVnF0vL5649dZbefGEfbyxEfEdJTW869evV9u3b1dy8LJ+ra88TFbtWLLYLMmgAImXAVSPQjJZeiRGRl1B42Rg6219kLe8+br6i77J9DUpheE1oWW\/LL6jhIY3\/CKIe++9V23cuFE9\/fTT6rnnnlOTk5NsabB\/HxExQoDJsvwpgcbmGov5lf9ePvOLytYHieLj9gf0Ndc3rgaGN45Qtp9jeEtoePWQvvCFL6j77rsveOOVmN7w63+zTav46CRePKMil2CyLLJ6yfqOxsk41StVb\/VX6rk89gx9G9c3GgHDa5+pSUR8R4kNr0ki5F2WxMubeL7tMVnmy9tFa2hsn7pP2x\/Q176+GF77TE0i4jswvCb5Yq0siWcNpZeBmCy9lMVqp9DYKs7rgoW3P4Qffstr+wP62tcXw2ufqUlEfAeG1yRfrJUl8ayh9DIQk6WXsljtFBpbxRkbrNrRZ1IpKwOMvrGSGBfA8Bojs1oB31FSw8s5vFbvE4IZEmCyNARWwOJo7Fa0Wmf\/3nLT3ODkh0ZPgEBf+\/pieO0zNYmI4S2h4dXn8K5Zs8aLM3erJSSJZ3KbFq8sk2XxNDPtMRqbEsu2fK39v2kNMPra12vgf\/s\/1PEf\/zf1+Mdu5hxe+3hjI+I7Smp49Tm8nZ2dsUngogCJ54J6fm0yWebH2lVLaOyKfLJ2axlgeQHGzDaItuD\/tS70TcbZpNSf7z6mnvlP\/1x965NzMbwm4CyVxXeU0PDKkGRLw4svvqgGBwctpYrdMCSeXZ6+RWOy9E0R+\/1BY\/tMs4xoegQa+tpXA8Nrn6lJRHxHSQyv3sYwMTFRV\/\/29nZePGFyh1A2FQEmy1TYClUJjQsl16zOhk+AODF+Ub185mLwefgBuA+2z1VvvPGGuuWWW9ScOXOKO1iPeo7hdSsGhrckhtdtGpm3TuKZMytSDcxQkdRK11c0TsfNx1ozb3+7qP7x51dmvQFu\/jvmqH+zrL3hB+B8HLOLPj3074+r4z+eVn\/5P\/82WxocCIDvKJnh7e3tVfIkqFy7du1SK1eudJBW8U2SePGMilwCM1Rk9ZL1HY2TcSpiKTHAf\/nKT9T\/O3lRHXl9atbqr3zj8g1wReSp+4zhdasevqNEhlfM7ve\/\/\/1gy4K8Tri7u1s99NBDqU9qCJvnefPmqX379qnwQ3D1zPXw8LAaGBgI6Pb19V3XBxLP7Y2fdeuYoawJu4+Pxu41yLIHYX0nf\/WWOvDqG6rW9gc5Bo0rngCGN55RliXwHSUxvLKHN3oygzy4tmfPHjUyMqJaWlqM8kgM6+joaKWufH\/w4MHK\/l+JvW3btuD71157rfJ1a2urOn36tOrp6VFDQ0NBm\/rrsFkm8YzkKFxhzFDhJDPuMBobIytUhVr6Vtv+oPf+svpbX2IMr9tbAN+B4U2UgWETK8ZVVnflklMgpqam1IYNG9S6deuCLRRRsyxlOzo6Zq3ykniJsBe2EGaosNIl7jgaJ0ZVyIJJ9a13+kPc0WeFBNNApzG8DcCzUBXfgeFNlEZhw7t48eLA4C5fvjwwsdrw6u\/DZliCR7+Xn+nE279\/v5KTI\/QlK8RcxSeQdLIs\/kibdwRoXG7t0+gbNr9jP5699\/eTt7+r7rm\/5aY5M7rPfv2v1X+c\/C9qcGULD63lILj85Tt8ySlWXV1dwV+vw74jh6542cQN09PT0172LKZTtrc0RJsT0zo5ORlscZArvKKrTa1exY2u6MqK7\/j4+KwzgbXhjbYj+47Xrl1bRAnoc4jA1atX1YULF5T8AsORRuVMDTQup656VDb0\/Zufvqm+P\/Gm+trJmWPP5NSHP3pfi\/rEbS3B1812PfL8D5Xsh37iQ\/9NLVmyxPnwbWjsfBB1OrB3797g2aPoheGdIVJow7t69WqVxTm8YlifeeaZykNr0S0MjRje\/v5+tWDBgko+ikFildfnf0KS9e3KlSvBg5PyWzSGNxmzopVC46IpZtZf2\/qK0Xv+1IXKsWdieD95+ztj3\/hm1mu\/S\/8v\/\/4VNXHxqupf8Vtq6dKlzjtrW2PnA4p0QBYCw6u8Y2NjaufOnazwvs2psIa3kUST7Qqysnrp0qUgTPg4s6jZlc+jWxga2dLAb1qNKOdv3TR\/DvV3NPSsGgE0LndeZKmv3vrw1EtnA4j6obeyH3f2J1\/+a\/WTX1xRX773X7ClwcHtwx7e2dCb0vDWyrvoyQzhcuFtC9UeWgtvYeChNQd3tuMms5wsHQ+N5t8mgMblToW89G0m84vhdXvPYHgxvFUzUI4d27x583Vn7+rCHEvm9sb1vfW8JkvfOZS5f2hcZnWVcqHvUy\/9g5oxwDMPGz14R6v6SMdNwf\/LcH1sxwl1+fIUK7yOxMTwYnirpl74pRLhAuHtDrx4wtFdW4BmXUyWBcBSqi6icankvG4wLvXVq75ifOXrsmx5EMP7G\/\/1l+rxj93MlgYHtw+GF8PrIO2uHUvGHl4n+DNv1OVkmfngaCAggMblTgRf9I1ueSjyqi+G1+09g+HF8DrJQBLPCfbcGvVlssxtwE3YEBqXW3Tf9C3Dqi+G1+09g+\/A8DrJQBLPCfbcGvVtssxt4E3UEBqXW2yf9Y2u+j52z6JCHG+G4XV7z+A7MLxOMpDEc4I9t0Z9nixzg1DyhtC43AIXQd+o8b1r8Y3qqw\/e5u0b3cTwtt3wc9XzkXexh9fB7YPvwPA6SDv28DqBnmOjRZgsc8RRyqbQuJSyVgZVNH3lhIfwQ25ypq9vpzv83rZX1JLfuIDhdXTrYHgxvE5Sj8Rzgj23Ros2WeYGpkQNoXGJxKwylKLqK6u+2vz6droDhtftPYPvwPA6yUASzwn23Bot6mSZG6ASNITGJRCxzhCKrm94u4MvxhfD6\/aewXdgeJ1kIInnBHtujRZ9sswNVIEbQuMCi5eg62XR1yfji+FNkHgZFsF3YHgzTK\/aoUk8J9hza7Qsk2VuwArYEBoXUDSDLpdN3+hWBxd7fG\/u\/Sv1qUUXg73Fy5YtM1Ajm6Jl0ziOEr4DwxuXI5l8TuJlgtWboM32D6k34HPsCBrnCNtBU2XVN2p8h9bcpj7ScWMuhDG8uWCu2Qi+A8PrJANJPCfYc2u0rJNlbgAL0BAaF0CkBrpYdn3F+H72wOvq5TMXVR7HmUl7sqXh4X\/1M3VPZzsrvA3kZtqq+A4Mb9rcaageidcQPu8rl32y9F6AHDqIxjlAdthEs+h7Yvyi6jn4uhJDKi+wkK0OWVwY3iyomsXEd2B4zTLGUmkSzxJIT8M0y2TpKf5cuoXGuWB21kiz6StHmT310tngpRWyx9a28RVj\/cfDp9QX339effB9C1nhdZDZ+A4Mr4O048UTTqDn2GizTZY5ovWmKTT2RopMOtKM+kZPdLC5vxfDm0maGgXF8GJ4jRLGVmESzxZJP+M042TppxLZ9QqNs2PrQ+Rm1je8v1dWe+V1xY1e8hY42TP8lQ9MqltvvZUV3kaBpqiP78DwpkibxquQeI0z9DlCM0+WPutis29obJOmf7HQVwWvKhaTamObg2yZ+PpfnQm2NGB43eQ7vgPDWzPzent71eHDh4PP582bp\/bt26c6OzuD78+dO6dWr16tJiYmKvX7+vrUpk2bgu+Hh4fVwMBA8HX457owiefmhs+rVSbLvEi7aweN3bHPo2X0naEc3ebwHzbdHhhg0wvDa0rMfnl8B4a3alaJYR0dHVUjIyOqpaUlMLAHDx5Uhw4dUq2trer06dNqy5Ytavfu3cH34Us+6+npUUNDQ8GP9dfaLMvPSDz7N7NPEZksfVIjm76gcTZcfYmKvrOVEOMrD52lPc1BVoq\/\/3c\/Vg\/\/q39ihddRkuM7MLyJUi9sYsW4Hjt2TO3Zs6diiMNBomZZVoo7Ojoqq78Y3kTIC12IybLQ8iXqPBonwlTYQuhbXbrwaQ4mD7V94qun1H\/+z+cxvA7vCAwvhjdR+kUNb9TUhoOIwZVrcHAw+H\/0ewxvIuSFLsRkWWj5EnUejRNhKmwh9K0tXfihtqRn98pb1v6ntkvq4\/MvscLr6K7A8GJ4E6WemNbJycnKim54f68EWLp06azPwiu6Yo7Hx8crBjhsePfv36\/a29srfYhuj0jUOQp5R4DJ0jtJrHcIja0j9Sog+sbLIau9Tx+fCPb0fvlPfqfmK4onf\/VW8Ja1P\/2dC+p35v0XtXAh5\/DG0228hDxrFL7kmaOurq5gu2bYdzTeUjEj3DA9PT1dzK5n12sxrM8880zlobWpqSm1YcMGNX\/+\/MDERr+PbmGoZ3ijve7u7lZr167NbjBEzoXA1atX1YULF4L93XPmzMmlTRrJlwAa58s779bQNxlxMbNPfPtn6m9++qb6wIK56ok\/eKea\/47Z\/+bJZ1LmscVnlfwi0dbWppYsWZKsgQxLlV3jvXv3Br4lemF4Z4g0peGV7QpiNC9duhRA2LVrl1q5cmXwddTs1rr3ZE\/vtm3bgofaduzYERRLsqWhv79fLViwoBJWDBKrvBn+C5dT6CtXrqjz588Hv0VjeHOCnnMzaJwz8JybQ18z4N87O6X+7H\/\/YfBQ2yO\/36oevKMtWPmV7+\/72v+j3vuuFvW\/tv1DsEC0aNGi4K+irq+yaywrvOFV3rGxMbVz505WeN9OvKY0vLVuuujJDPVuzvBDbPIbVXgLAw+tuf5nLf\/2+XNo\/szzbhGN8yaeb3vom463nN0rWx3E6GrDe9fiG4OXV\/zH0RfV5cuX2cObDm3DtdjDOxshhvdtHmJgN2\/ePOvsXY1Kn8G7devWYCVYf79mzZrgJAaOJWv4vix8ACbLwksYOwA0jkVU6ALo25h88irhl8\/8Qr3n5t9UYnjF\/Mq59hjexrg2UhvDi+Gtmj\/Rh9J0Ib3dIfriiVWrVs16KI0XTzRyWxa\/LpNl8TWMGwEaxxEq9ufoa18\/DK99piYRMbwYXpN8sVaWxLOG0stAZ8+eVc8++2zwcCNPw3opUcOdQuOGEXodAH3ty+Ob4W02jfEdGF77d3WCiCReAkgFLoK+BRYvYdfROCGoghZDX\/vC+WZ4m03jZhtvXAazhzeOkKXPSTxLID0Ng76eCmOxW2hsEaaHodDXvigYXvtMTSKS06zwmuSLtbK+JZ4cSP3CCy+o+++\/39qf4BuJaVo3Sfm4MvU+N\/2sGfSVmyGOab0bxrRukvJxZUx1rDfGZtA4jmfe+ibJOVsa+6ZvkrGnmaDy1PjgwYPBCUZ33XWXuvvuu6t2N64\/afStxc43jePGnkbf8Nhle508jM85vDMkWeFNm1GG9fSNFn3TmmEYa8X1G1hs9qeRmKZ1k5SPK1Pvc9PP4tqyJlzCQFn1p5G4pnWTlI8rY6qjnizk7UTReyOurYTSWCuWRX8aiWlaN2n5uHK2NI5rx5pwBoGy6FMjMU3rfvvb31bf+c531Ec\/+tG6hrfa\/aYxpdG31n1s2n8DqVIVzao\/Oq6c+4\/hvSYNhjdVmppXkgSUxJODoLkgAAEIQAACZSdw++23q7lz56qLFy+q119\/vezD9XJ8d955pzpw4ICXfcu7UxjeHImL6ZX\/uCAAAQhAAAIQgEDWBGRbAycHzVDG8GadbcSHAAQgAAEIQAACEHBKAMPrFD+NQwACEIAABCAAAQhkTQDDmzVh4kMAAhCAAAQgAAEIOCWA4XWKX6nwK4ujryt23DWat0zgC1\/4grrvvvtUZ2en5ciEc0ng9OnTqru7W126dElxD7tUIru2w\/9O9\/X1qU2bNmXXGJGdEjh27Jh68cUX1eDgoNN+0Lh9Ahhe+0yNIg4PDwflZcJ89NFHVU9PD4bIiKD\/haempoJXDstTyvv27UNf\/yUz6mH4Fxl+qTFCV5jC8u90R0eH+vCHP6wef\/xx9fnPf161trYWpv90NBkB\/YvNBz\/4QQxvMmSFKoXhtSyXNjfr1q1TK1eurESXfzAHBgaC78MrBOEJUv+jGq5nuXuEs0DAVOMf\/ehHQat79+5lhdcC\/6xDmOob7g+GN2t17MRPq7EYoh07dqgnn3xStbS02OkMUTIhkEbj3t5e9e53v1v9+te\/Vl\/84hcz6RdB3RHA8Fpkr2+wkydPql27dlUMr\/zJU1Zuh4aGgtb01\/KnbQyvRQFyCJVGY90tzFAOAjXYRCP68qfQBuHnVD2txnrrykMPPcSWhpy0SttMGo31\/bt27Vr1jW98A8ObFr7H9TC8lsTR\/xjedtttanJyUm3durVieGXlVl7tNzIyEqwKyG+R8ucx2QeG4bUkQA5h0mqM4c1BHAtNNKKv3OPyClX2\/VkQIsMQjWisu8Vf4jIUyELotBrLvHz48OFKD8KLVha6RQgPCGB4LYmg\/2z9W7\/1W2r16tWzDK\/cSHLpyTD8PXt4LQmQQ5i0GmN4cxDHQhNp9ZWVITG7PMhkQYSMQ6TVWP7Nvvfee4NFjPDXGXeX8CkIpNVYNyWGmRXeFOALUAXDa1kkvek9vMIbXtGV5sKrQTz9a1mAHMKZaozhzUEUi02Y6Ct7OeWBRNnGpC9WhiyKkVEoE41loYLTdDISIsOwphpjeDMUw5PQGF7LQqS9ySx3g3AZEkDjDOF6EBp9PRAh4y6gccaAPQiPxh6I4FkXMLyWBal1k0kz1bY0WG6ecDkQQOMcIDtsAn0dws+paTTOCbTDZtDYIXxPm8bwWham2k0WfaAlusXBchcIlzEBNM4YsOPw6OtYgByaR+McIDtuAo0dC+Bh8xhey6JUu8nqHUtmuXnC5UAAjXOA7LAJ9HUIOuzSbQAABk1JREFUP6em0Tgn0A6bQWOH8D1tGsNrWZhqN5k0UevFE5abJ1wOBNA4B8gOm0Bfh\/BzahqNcwLtsBk0dgjf06YxvJ4KQ7cgAAEIQAACEIAABOwQwPDa4UgUCEAAAhCAAAQgAAFPCWB4PRWGbkEAAhCAAAQgAAEI2CGA4bXDkSgQgAAEIAABCEAAAp4SwPB6KgzdggAEIAABCEAAAhCwQwDDa4cjUSAAAQhAAAIQgAAEPCWA4fVUGLoFAQhAAAIQgAAEIGCHAIbXDkeiQAACEIAABCAAAQh4SgDD66kwdAsCEIAABCAAAQhAwA4BDK8djkSBAAQgAAEIQAACEPCUAIbXU2HoFgQgAAEIQAACEICAHQIYXjsciQIBCEAAAhCAAAQg4CkBDK+nwtAtCEAAAhCAAAQgAAE7BDC8djgSBQIQKACBc+fOqdWrV6uJiYnrertr1y61cuXKAozCbhePHTumXnzxRTU4OKh6e3uD4PJ1+Kr182hPpNy9997blBztqkI0CEDANgEMr22ixIMABLwloA3v1q1bMWVKKeHxZ3\/2Z+rLX\/6yam1tbdjwRuN5mwh0DAIQaDoCGN6mk5wBQ6B5CWB4Z2s\/PDwc\/GDTpk3B\/xtd4ZUY0ZjNm22MHAIQ8IkAhtcnNegLBCCQKYE4wzs1NaU2bNig5s+frw4fPqxWrVoV\/Hn\/9OnTqru7W126dEnNmzdP7du3T3V2dgZ9DX+2YsWKoMzy5csDExk1kGIGR0dH1cjIiGppaQlWWMNbLPS2Ct0PaevkyZNBzPb2dnXo0KFgJTbaru6T\/HzLli1q9+7dQTkdZ926ddetaMtnjz76qOrp6amMJYnhlTLCJnxpTvIz2SKxZ8+eyhgzFZTgEIAABBISwPAmBEUxCECg+ARq7eHt6+sLDKo2iDLSqCnV2yDE0G3bti0wn3KJYV2zZk1QXz7buHGj0vHqGV6pK+Zam2MxzmI+h4aG1OLFi4PPJicng3bEHGsjLgY8atx1n5599tnA8GqDKzHDBjisYLXPqplZXSdsavXPwn3WvwBI39avX6+2b99eMdLFzxxGAAEIFJ0AhrfoCtJ\/CEAgMYGkK7zahFZbsQyvmsrn2vyGV1STrPCeOXPmOjMqhrOjoyNYTQ6bYWknbJ7Dpluv+GoIsoo8Pj4erEyHv45CqrYSm2SFV8epxTKOcWKxKAgBCEDAIgEMr0WYhIIABPwmEGfGtJkNG14xjQMDA9cNTFZxxZymNbzf\/e53g9Xg6CUrqU8++WRdwxvdGhGOEV653bFjR81TExoxvNU46T7U20bhd3bQOwhAoMwEMLxlVpexQQACswikMbz19qRGV1qjRrDeloZqK7xR0xg23klXeHUfPv7xj6ujR49WTmCwucJb75iyOMakJAQgAAEXBDC8LqjTJgQg4IRAnBmrtnIZraMfUuvv71cf\/vCHZ63ERvfwykrswYMHZ+3DlYHL\/mC5wtsWdDuyHzhuS0O1Pun9v7KXVq9KL126tObDY7X28Eq\/6p3DW291Weqyh9dJatMoBCAQQwDDS4pAAAJNQyCN4RU44ZMY5Hv9UJo2ePqkhdtuuy1g+Yd\/+IfBQ2zhh+TkJIWuri516tSpmqc06AfDqhnv6KpqtE\/hF2fozx566KHKkWNRkdOc0vD5z3++6os7wsaaUxqa5nZioBAoFAEMb6HkorMQgIDPBOrtbc2z30lXWbM4M1c\/eKfP9s1z3LQFAQhAoBYBDC+5AQEIQMASAV8Mb9JVVttvRrMdz5IshIEABCCgMLwkAQQgAAFLBHwwvLLCevz48Vkvx6g3PDHHL7744nX7dtMgkbbvvfdeXtucBh51IACBTAlgeDPFS3AIQAACEIAABCAAAdcEMLyuFaB9CEAAAhCAAAQgAIFMCWB4M8VLcAhAAAIQgAAEIAAB1wQwvK4VoH0IQAACEIAABCAAgUwJYHgzxUtwCEAAAhCAAAQgAAHXBDC8rhWgfQhAAAIQgAAEIACBTAlgeDPFS3AIQAACEIAABCAAAdcEMLyuFaB9CEAAAhCAAAQgAIFMCWB4M8VLcAhAAAIQgAAEIAAB1wQwvK4VoH0IQAACEIAABCAAgUwJYHgzxUtwCEAAAhCAAAQgAAHXBDC8rhWgfQhAAAIQgAAEIACBTAn8\/y\/3XIIvsPO6AAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:97e357dd]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"num","rows":1,"type":"double","value":[["0","2.443525700815186e-03","2.362998398567786e-03"]]}}
%---
%[output:9d1224cc]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000e+00","-1.901953846588630e+00","9.043571086383212e-01"]]}}
%---
%[output:26f707e8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ares_nom","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-2.526618726678876e+05","-2.513274122871834e+01"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000e+00","1.000000000000000e-04"],["-2.526618726678876e+01","9.974867258771282e-01"]]}}
%---
%[output:6e27c0a7]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:7996ad92]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:9ad8d2a8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"    -2.526618726678876e+01"}}
%---
%[output:637faca4]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"     9.974867258771282e-01"}}
%---
%[output:61f9eea3]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["1.490161953970131e-01"],["3.652194550246457e+01"]]}}
%---
%[output:306c88de]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:23fb1bfd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:11c23dfc]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","1.000000000000000e-04"],["-9.869604401089360e+00","9.984292036732051e-01"]]}}
%---
%[output:4b099645]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["1.555088363526948e-01"],["2.716608611399846e+01"]]}}
%---
%[output:00aa870f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["3.719106107041118e-02"],["1.937144673860303e+00"]]}}
%---
%[output:2e5b507a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAArwAAAGlCAYAAAAYiyWNAAAAAXNSR0IArs4c6QAAIABJREFUeF7svQuYFcW1NrxQxKCDgIKOMEYIqJl4gfGCGtSYg4HEqD96IiJRMAQFBz0nP4IYQmL0EPMRkEhUBCEGUAHnGMV7QP1MRlBBDKAxqEEhCQ4gKNeAoIbvWY217Wl6767uruvMW8\/jAzJVa61+39W136m9qqrJnj179hAaEAACQAAIAAEgAASAABBooAg0geBtoMzisYAAEAACQAAIAAEgAAQCBCB4kQhAAAgAASAABIAAEAACDRoBCN4GTS8eDggAASAABIAAEAACQACCFzkABIAAEAACQAAIAAEg0KARgOBt0PTi4YAAEAACQAAIAAEgAAQgeJEDQAAIAAEgAASAABAAAg0aAQjeBk0vHg4IAAEgAASAABAAAkAAghc5AASAABAAAkAACAABINCgEYDgbdD04uGAABAAAkAACAABIAAEIHiRA0AACAABIAAEgAAQAAINGgEI3gZNLx4OCAABIAAEgAAQAAJAAIIXOQAEgEBJBObPn09DhgyhFi1a0MyZM6lLly7GEdu+fTsNGjSIFi9eTL1796YJEyYUYuD4xowZQzU1NVReXm40tuXLl1P\/\/v1p27ZtNHnyZOrZs2fgX8R7zjnnUHV1tdGYkpwNGzaM5s6dWy\/epDHi5yqea9KkSVRbW0vTpk2jsrIyWdfK+oU5Y6MVFRW5cqdUbioL2oAhgUtlZaU1bgw8Jlw0YgQgeBsx+Xh0ICCDgMuCl8XT+PHjc4sWGRzi+sQJ3nXr1lGfPn1ozZo1NHz4cKcEr4i3ZcuWqUWeiucSYrtbt25WRFVYnAo+88bSUARv+DnCv7xlfTcwDgi4hgAEr2uMIB4gAASkEYDglYYq6Cjwiq6Sy1hpaII3CwZxODUUwZs3P2RyCH2AgE0EIHhtog\/f3iMgVj\/Fg8R97R9e1brsssvohhtuKDx3sRVAMUZ0jPaLfsh++9vfDsoOivUvBnRYxBQbG7fCG36mqqoqmjJlSjA8HKcQV8Ju9KvjYmI1jKlYaYo+76233loocQg\/W7HVumIri+Hnj65qyXAbXeHlWMI8iNjCtqPccp+4FbXoV+\/cZ+XKlbEr2jJf06d5Vo4pLAijWKR9rrg8i\/qQeYZSE0YSX1H7su9K3Li48hVRbiPzLkbfjei7w\/8v846Fc4lz\/7bbbqOrrroq9tuFpDmFfYpn5b\/bKl\/y\/kMBD+AsAhC8zlKDwFxHIE64xH2IluoX\/dCP+8pV2AwLkFL98nyQx\/kqJXjDHIXFfrFnDvcxKXiLlWWIf4+KcVlu0wreUnbDIqqYwIz75aFY3+gvX0kYxL1vIueSBG\/Sc5100kmFMo+wnyT7snXjMnxlEbyleBC\/3Mm8i2Fu48Su7Lwh8OjUqVPsL3xhbGXii65yq1jFd33eRnyNFwEI3sbLPZ48BwLhD8Lwqqb44C0m\/sIfMHF9xYdheHycUIl+yIoP1PCHeqnaxPD4sNiLiylJ8EZXn+OwCcclMMgjeMWmNdmShrgP8mJfRafhNk0Nb9zqWTgugUsxbsJxCc44hUW9cNz4cL4Vwypu9TsuD4uJIdnniq5aik1rSRgklR6k4StN+UE4LvEu8TOIzZOCA954J\/6Nfy7exbjniltlD8cUfmfDIl7mHYvOCWKM7JzCsafBJ8f0iaFAwAoCELxWYIdT3xGQ+YpcfOCIvtFVxKiA4N3+cScRhD+E4lZtosJWZmNQsdMF4k48KCV443a4x\/mP291vUvDGia1333039oSFNNymEbzRnI+u9AlhF7YZFTrRXHr99ddjT9CIW7ku9lxhYVVKXMqu\/hV7rmKCN2nlOekUhTR8pRF0xeKKnjJRTLAWe95wHkRXkOMEb6l3LPqzaO6kmVNEXDLzh+\/zN+JvnAhA8DZO3vHUORAo9aEZ97NiHyDRvjfeeGPs177hUJNW8WQ+ZLlPMcEbB0tSDW\/0eCnZD0zTgje6Erlw4cJ96mHTcptW8Jb6OjtO8EZre6OYPfLII8EzFGtxX4FHRW3cV\/1xpQSlBK\/McxXLzVJjeUypsoa0fKkQvFGsS9mMexdKlUnECd64b2pkRf4ll1wiPaeI55L91iTHFIqhQMAKAhC8VmCHU58RSPshC8Ebf9aqacEb5m3w4MG0dOnSfc71TcttGsEbFjpCxB1xxBH7lCSU+mVEh+AV72KcEAuvIBYTvLLPBcE7jcLfKjAe\/AvN17\/+9cI3OxC8Pn8yIHbXEYDgdZ0hxOckArIrLHzpQN6ShjgA0q4qRW0U+9pc\/Pu4ceMKlyhkXeGN2whWV1dXOH\/VtOBlDITP9u3b09atWwNYorvR03CbRvAK32FRE1fnqaKkIc0qZFx+xcVQTPDKPlcxwVusdED2xU\/DV5YVXiFMxaUiHO+IESMKeZPmXXzppZeCEpTwu5FUw1tqhTdrSUMpbOP4lOUC\/YCAywhA8LrMDmJzFoE0G2WK1UjKblqLE1VpPmTjbrMqtjEq\/PWy+Do9reCNwyZuA5D48GeSRa1q9PiqYseSpd20JhIp+vV9nJhIw20WwRt3UgXHp2rTWjFhWaq2mo\/UShLiSYI36bmKxRUn+ov1jZsQ0vCVRvDG5Sy\/S9H3NnxiQrRcJIp5OOej7xc\/m+wKb9wzp9m0VupbBNmSJGcnZwQGBIogAMGL1AACGRFIOopJrAiV6hcWOvz3YueVRj8M8wpetlfsmKaor7SCNyxW4qCNO1GiGAVJgrfUpp84m8VEQbSvLLdJv4wIu\/wcXL4griGOi03m3NuvfvWr9NZbb9VbISxVAxt3HFZ0VbBUTWlYxEax42dI+1zFNrTJPkOxPJHlK43gZV+lsMlST8\/+xakacc8iK3jjuGB74psLvuq62C+RYb\/RX\/jS4pNx6sQwIGAFAQheK7DDaUNBIPqBmHTxxH\/913\/RtddeS\/yBxE324onoypEKwVtMYEd9ZRG8bDsqYuKwiVtxDV\/OkSR4ox\/8STv6wyIm6YxXGW5LnXYRdxFI1KbsZRIi1riNdnG\/vJTCmvtHyzjiftGKwzIav8hf2eeK+gkLrmguJPETnUNk+Moi6OJ+MQy\/t2nfxag9ttW5c+d9TtuQWWmNfnsU3vha7IQPgVvciRylLidpKHM2nqPxIgDB23i5x5MbQkDmg8tQKHDjOQJ5dtBDzHhOvkT4skfHFTMVd6ayhFt0AQJeIADB+zlN\/EEyZ84cqqmpofLy8qLkxX2VVGyVzosMQJDaEYDg1Q5xg3IQFi3FNjclXcZQDBAhmLOOb1BAe\/ww4W9Pwp8\/eTcAIj88TgqEnogABG+olrFly5YlBa\/4IGrXrl1hp7n4jbhHjx4kNtIkoo4OjQoBCN5GRbeSh02q+46ezyvrVMxXSXOdrD30s4NAqfp7jqjULYvFIg4v5mTNLztowCsQkEOg0QveYrvV4+CLHkcj+siuDstRgl4NDQEI3obGqJnnidvIlbauNS5SkY8QNWZ41OWl2AbXrKv3QkRXVlYWFnR0xQ67QMAGAo1e8IprIquqquipp55KLGmII4ltTJkyZZ+NIDYIhU8gAASAABAAAkAACACB+gg0asEbXrHl3c8yNbzFVkyWLFmSSSwjIYEAEAACQAAIAAEgAAT0ItBoBa\/4Oqhv377Et2FlLUsQmwRkNq6tWbNGL5uwDgSAABAAAkAACACBzxHgja9oexFotIKX69ii15ymXeFNU\/PEYpevo1y0aBFyDwgAASAABIAAEAAC2hE4\/fTTia+Kh\/BtpII3bvNZ2hXeNGKXM\/qVV16hfv36BYnHt+GguY0A\/2IyceJE8OU2TfWiA2cekUUU\/PKPdwyc+YWAX9GKd6y2thaCt7Gu8CYd+ZNUniDKGNIc\/SIELxLPjwkDfPnBUzhKcOYXZ+DLL77CCzf4HPODO7xj9XlqtCUN0XSVXeEVYjft0S9IPD8mCBEl+PKLL3wYgy\/\/EPAvYsyLfnEGviB4YzNWRvDmuWQCiYeJwi8E\/IsW75hfnIEvv\/jCL5Xgyz8EIHilBW\/0fN2kUohSB7ljcvfrVVm9ejVNnz6dRo8eTU2bNvUr+EYaLTjzi3jw5RdfHC0484sz6A4IXisZi8SzAntmpx9\/\/DGtXbuWjjrqKAjezCiaHQjOzOKd1xv4youg+fHgzDzmeTxCd0Dw5smfzGOReJmhszIQE7sV2HM5BWe54DM+GHwZhzy3Q3CWG0KjBsbOW0W\/nvogLbnjKpzS0FhPaTCacZ87g+C1gXp2n5jYs2NnayQ4s4V8Nr\/gKxtuNkeBM5vop\/d94d1LaeG7m+n1YcdA8ELwpk+grCMgeLMiZ2ccJnY7uOfxCs7yoGd+LPgyj3lej+AsL4Jmx0Pw1scbx5IZyj8IXkNAK3KDiV0RkAbNgDODYCtwBb4UgGjYBDgzDHhOdxC8ELw5UyjbcAjebLjZGoWJ3Rby2f2Cs+zY2RgJvmygns8nOMuHn+nRELwQvKZzLvAHwWsF9sxOMbFnhs7aQHBmDfpMjsFXJtisDgJnVuFP7RyCF4I3ddKoGADBqwJFczYwsZvDWpUncKYKSTN2wJcZnFV6AWcq0dRvC4IXgld\/lsV4gOC1Antmp5jYM0NnbSA4swZ9JsfgKxNsVgeBM6vwp3bedczL9I+PPsYpDZ8jh01rqVMo2wAI3my42RqFid0W8tn9grPs2NkYCb5soJ7PJzjLh5\/p0RC8WOE1nXOBPwheK7BndoqJPTN01gaCM2vQZ3IMvjLBZnUQOLMKf2rnELwQvKmTRsUACF4VKJqzgYndHNaqPIEzVUiasQO+zOCs0gs4U4mmflsQvBC8+rMsxgMErxXYMzvFxJ4ZOmsDwZk16DM5Bl+ZYLM6CJxZhT+1cwheCN7USaNiAASvChTN2cDEbg5rVZ7AmSokzdgBX2ZwVukFnKlEU7+tQ4e9EDjB1cJ7scamNf05F3iA4DUEtCI3mNgVAWnQDDgzCLYCV+BLAYiGTYAzw4DndAfBixXenCmUbTgEbzbcbI3CxG4L+ex+wVl27GyMBF82UM\/nE5zlw8\/0aAheCF7TOYcVXiuI53OKiT0ffjZGgzMbqGf3Cb6yY2drJDizhXw2vxC8ELzZMifnKKzw5gTQ8HBM7IYBV+AOnCkA0aAJ8GUQbEWuwJkiIA2Y4QsneNMaN9Tw7gUcNbwGEo9dQPAaAlqRG0zsioA0aAacGQRbgSvwpQBEwybAmWHAc7iD4N0XPAjeHAmVZigEbxq07PfFxG6fg7QRgLO0iNntD77s4p\/FOzjLgpqdMRC8ELx2Mg8rvNZwz+oYE3tW5OyNA2f2sM\/iGXxlQc3uGHBmF\/803iF4IXjT5IvSvljhVQqndmOY2LVDrNwBOFMOqVaD4EsrvFqMgzMtsGoxCsELwaslsWSMQvDKoOROH0zs7nAhGwk4k0XKjX7gyw0e0kQBztKgZbcvBC8Er7UMhOC1Bn0mx5jYM8FmdRA4swp\/aufgKzVk1geAM+sUSAewYOVmumjS0qA\/TmnYCxs2rUmnT76OELz58DM9GhO7acTz+wNn+TE0aQF8mURbjS9wpgZHE1YgeLHCayLPYn1A8FqDPpNjTOyZYLM6CJxZhT+1c\/CVGjLrA8CZdQqkA4DgheCVThbZjpMmTaI5c+ZQTU0NlZeXFx0GwSuLqBv9MLG7wUOaKMBZGrTs9wVf9jlIGwE4S4uYvf5j562isfNWBwGgpGEvDyhpyJGPy5cvp\/79+1PLli0heHPg6OJQTOwuslI6JnDmF2fgyy++OFpw5g9nELxY4VWWrdu3b6dBgwbR4sWLqaKiAoJXGbJuGMLE7gYPaaIAZ2nQst8XfNnnIG0E4CwtYvb6D529gma\/ug4rvCEKsMKbMR+5lKG2tpaqqqroqaeeguDNiKOrwzCxu8pM8bjAmV+cgS+\/+MIKr198XXj3Ulr47mYIXgjefIk7f\/58GjFiBM2cOZMWLlyYqoZ31qxZwYqwaKXqfvNFidF5EMCHcR707IwFZ3Zwz+oVfGVFzt44cGYPexnP69btXdHldu1jGyB4I6BhhVcmi0J9OKH69OlDffv2perqakq7aS3qjmuABwwYkDIKdNeNwK5du2jDhg3BRsSmTZvqdgf7ChAAZwpANGgCfBkEW5ErcKYISE1mZsyYESzEcdvc+7cFL9i0thcKCN6UiTds2DCqq6ujadOmUVlZWWrBO27cOGrfvn3BKwsqrPKmJMFA9507d9L69euD1XgIXgOAK3ABzhSAaNAE+DIItiJX4EwRkJrM8IIc\/\/fPTR\/T4Gf3QPBGcIbgTZF44VKGLl26BCPTrvBy3W+4pCGFe3Q1iAC+ujMItiJX4EwRkIbMgC9DQCt0A84UgqnRVPhaYXaDFV6s8KZON17dnTt3btFxw4cPD8oc4hrO4U0Nt9UBmNitwp\/JOTjLBJu1QeDLGvSZHYOzzNAZHRg+kmy\/HRtp2egzsdCGkob8OYgV3vwYumgBE7uLrJSOCZz5xRn48osvjhac+cFZ+ISGphvfpj\/fdgEELwRv\/uSF4M2PoYsWMLG7yAoEr3+sFI8Y75h\/bIIzPzgLC94vvfU4Lb73BgheCN78yQvBmx9DFy1gYneRFQhe\/1iB4AVnDQkB958lWr9btuBX9NIjUyF4IXjNJS9qeM1hrcITBK8KFM3aAGdm8c7rDXzlRdD8eHBmHvO0Hvl2Nb5lTbRWc38YXJKFzfI4lixtLmXuD8GbGTorAzGxW4E9l1Nwlgs+44PBl3HIczsEZ7kh1G4gWr\/LK7wQvHthx7Fk2tNvrwMIXkNAK3KDiV0RkAbNgDODYCtwBb4UgGjYBDgzDHhKd9Fyhm+0XE\/LZ4yC4P0cRwjelAmVtTsEb1bk7IzDxG4H9zxewVke9MyPBV\/mMc\/rEZzlRVDv+Gg5w09O2UV3\/7QagheCV2\/iRa1D8JrFO683TOx5ETQ\/HpyZxzyPR\/CVBz07Y8GZHdxlvPLqLtfuLnx3c9C9e6dWxIK3X79+ELwQvDIppK4PBK86LE1YwsRuAmW1PsCZWjx1WwNfuhFWbx+cqcdUlcXo6u7j1VXUdONbELwhgFHSoCrbEuxA8BoCWpEbTOyKgDRoBpwZBFuBK\/ClAETDJsCZYcAl3UVXd7986JeC29WgO+oDCMErmVB5uyHx8iJodjwmdrN4q\/AGzlSgaM4G+DKHtSpP4EwVkmrtLFi5mS6atLRg9O7LK+ny08oheCMwQ\/Cqzbui1iB4DQGtyA0mdkVAGjQDzgyCrcAV+FIAomET4Mww4BLuoicziNVdHgrdgRVeiRRS3wWJpx5TnRYxsetEV49tcKYHV11WwZcuZPXZBWf6sM1iOVrKwDa4dveszq0Cc9AdELxZ8ir3GCRebgiNGsDEbhRuJc7AmRIYjRkBX8agVuYInCmDUomhqQvW0MhH\/lawxWUMXM4gGnQHBK+SREtrBImXFjG7\/TGx28U\/i3dwlgU1e2PAlz3ss3oGZ1mRUz8ueipDuJQBgjceb9Twqs\/DWIsQvIaAVuQGE7siIA2aAWcGwVbgCnwpANGwCXBmGPAi7sbOW0Vj560u\/JTFLpcy8J\/hBt2BFV4rGYvEswJ7ZqeY2DNDZ20gOLMGfSbH4CsTbFYHgTOr8AfO48TuXX0rC3W7ELzFOcIKr6H8heA1BLQiN5jYFQFp0Aw4Mwi2AlfgSwGIhk2AM8OAh9zxBrWn\/7KBRs1dWW9lt5jY5U7QHVjhtZKxSDwrsGd2iok9M3TWBoIza9Bncgy+MsFmdRA4swM\/i10+Z5f\/FI3LF0qJXQjefbnCCq+h\/IXgNQS0IjeY2BUBadAMODMItgJX4EsBiIZNgDPDgBNRdHMaR1CsZjcaHXQHVnjNZyy+WrCCeR6nmNjzoGdnLDizg3tWr+ArK3L2xoEzs9hfePdSWvju5npOZcUuVnixwms2W0Pe8JuWNegzOcbEngk2q4PAmVX4UzsHX6khsz4AnOmngMsWWOQOnb1iH2cTL\/sqXXn6kdJBQHdghVc6WVR2ROKpRFO\/LUzs+jFW7QGcqUZUrz3wpRdfHdbBmQ5U99pkocv\/XTdnRb1aXf4Zr+qO7NWR+GKJNA26A4I3Tb4o64vEUwalEUOY2I3ArNQJOFMKp3Zj4Es7xModgDPlkCYK3Vsv7EwXdWmbyTF0BwRvpsTJOwiJlxdBs+MxsZvFW4U3cKYCRXM2wJc5rFV5AmeqkNxrhzek8bm64dMXhIc0tbrFooLugOBVm7GS1pB4kkA50g0TuyNEpAgDnKUAy4Gu4MsBElKGAM5SAlak+\/2v1NF\/17wd+1OZ48Zko4DugOCVzRWl\/ZB4SuHUbgwTu3aIlTsAZ8oh1WoQfGmFV4txcJYNVlGfy6u50VMXwiu6SefqpvUO3QHBmzZnlPRH4imB0ZgRTOzGoFbmCJwpg9KIIfBlBGalTsBZOjhZ6D7z5kb68aN\/Kzow64Y0mUigOyB4ZfJEeR8knnJItRrExK4VXi3GwZkWWLUZBV\/aoNVmGJzJQbtg5eagNrfUau63j29D1d84KjiBQVeD7oDgLSAwbNgwmjt3buH\/J0+eTD179kzMvUmTJtH48eML\/YYPH07V1dUlxyHxEmF1qgMmdqfokAoGnEnB5Ewn8OUMFdKBgLN4qHgld8HKTcEmtGIil0fqXM2Niwy6A4I3QIDF7pIlS6impobKy8tp\/vz5NGTIEEoSryx2p0yZQjNnzqQuXbrQ8uXLqX\/\/\/jR48OCSoheJJz2nOtERE7sTNKQKApylgst6Z\/BlnYLUAYCzvZCJUxX+8OZGeuL1DYki93snH0H9z2indTUXgjc5nZvs2bNnT3K3htVDiNRx48bVW9FlEVxXV0fTpk2jsrKyfR563bp11KdPHzr11FNpwoQJhZ9HxTMSz\/98wcTuH4fgzC\/OwJdffHG0jZkzsfFs9qtrg5XcUo1Xcr9VeRj9f10Op7M6t7JGNBbasMJbNPkgeK29l845bswTu3NkSAYEziSBcqQb+HKEiBRhNCbOxCruto8\/pZse\/VvJVVyGkEXuWZ1aUd\/TjrQqcsN0QvBC8Ma+3qIuN6mOt1hJQ48ePeqt+kadiMSbNWsWVVRUFH7M5RRo7iHQmCZ299DPFhE4y4abrVHgyxby2f02dM7qtn4aCNtZi9cmClxGsd0hTYONZ187sswJkcvfQofbmjVrqF+\/flRbW1tPd2TPAL9HNsqShjBlonaX\/613794lRasYJ0oitm3bFvxTkkjmPkLwRtOF638HDBjgdxY1wOh37dpFGzZsCOq7mzZt2gCfsOE9Ejjzi1Pw5RdfHG1D4ozFLbe12z6lKYs202vvfyxFCIvcn5\/Xho5s0TQQvC61GTNmBPuLog2Cdy8ijV7wisTYvn07DRo0KKjhFRvZokkT10fU9bZr165o7W9Y8HLdcPv27QumWVBhldelKWNvLDt37qT169cHvxVD8LrHT1xE4MwPnkSU4MsvvhrCvMgnKbxZ9y968o0N0gKXSxUGnHEkVVWU0Rkd9t3b4xKLrEfCq7yLFi2iiRMnYoX3c5IgeEPZmnTiglgNjq7oFtsEF34RUEvj0rSQHEtD\/+ouGQH\/eoAzvzgDX37xxdH6xJnYZFbz2jpatXGnVIkCPyML3HOOaU19TikP\/q7znFzdGQDdUR9hJwXvli1b6J577iH+M2tr2bIl3XTTTamGywjeESNGFI4kE8bFKm\/fvn2LHk2GxEtFhfXOPk3s1sFyJABw5ggRkmGAL0mgHOrmKmdC3G7e+QlNqV0jLW6FwO3eqRVdftqR3gvcaKpAd3ggeIWA5ILrrI2\/iua6lbjGK7VxwrXYCq6wgRXerGz4N87Vid0\/JM1FDM7MYa3CE\/hSgaJZG65wxjeZ\/XvPHho3f3UqcSsELotbFrm+r+AmsQ\/B65HgHT16tNTNZ1HSWZiOGTOmqOAVtbg8Tpy5K1Z3Kysri9biqqjhRfF40ivqxs9dmdjdQMOPKMCZHzyJKMGXX3xxtDY449Xb51Z8SI8u+yCTuD2q9ZeC8oSObZo7cZKCSdYheD0QvLw7\/rrrrgv+O\/vss1Pnx4svvkh33XUXPfTQQyXHRq8Wjt6yFj2CTBiLjpM53QGJl5pGqwNsTOxWH7gBOAdnfpEIvvziS7fgFWUJtX\/bRC+\/tzm1uBWrt72+1oaGnntUAK7P9bcqsgO6wwPB+9lnnxH\/16xZMxWcO2EDiecEDdJB4MNYGipnOoIzZ6iQCgR8ScHkVCcVnAlhyw\/2+z+vp5UbdmQWtw219lYV6dAdHgheruHlw5KPOuqoYBMYX+W7\/\/77q8oBK3aQeFZgz+xUxcSe2TkGZkIAnGWCzdog8GUN+syO03AmbirjP9+o20ZPv7Exk7AVK7VndW5NfU\/1\/+SEzOBnGAjd4YHg5dMZ\/ud\/\/oeefPJJ2r17N7Vt25auuuoq4lMQWrdunYF2+0OQePY5SBNBmok9jV301YcAONOHrQ7L4EsHqnptxnEWFrbrtu6iGS\/X5RK2\/AQ\/7N6e2pQ1K2ws0\/tUDdc6dIcHgleEyJvEXnjhBbrvvvvo9ddfD\/75pJNOomuvvZbOPfdcr0oekHh+TSr4MPaLL44WnPnFGfjyiy8WtszZ6++tpd0HHEKzl6TfRBZ+YnFCwt5V2+YN\/sQEG2xDd3gkeMOh8ka2hx9+mKZPnx5c+VpWVkaXXnopDRw4sN7NZTaSSsYnEk8GJXf64MPYHS5kIwFnski50Q98ucFDOIrwai3\/+yvvbaY\/\/W1T5hVbtsHClk9KOPfY1nR6x4Z\/FJhLrEJ3eCp4Rdh79uyhd955h6ZOnUp\/+MMfaMeOHdSxY8dg1feiiy5ydtUXiefSNJAcCz6MkzFyrQc4c42R0vGAL3t8hYXtHtpDNUvW0+oP5W8ji4tcnIhw6SlH0DeOORQrtvboLXiG7vBc8IbD5\/rel19+mX76058G\/1xTU0Pl5eUOpNm+ISDxnKSlaFD4MPaLL44WnPnFGfjSy1d0tfbV1VtKaenxAAAgAElEQVTo\/779Ua7VWo643SFNqWnTpvTdE9vSd45vA2Grl8Zc1qE7GoDgFUKXyxtY8PIRZj179qTbbruN+EphFxsSz0VWiseED2O\/+ILgBV\/+IaAm4rCw\/cemnTR78Tr656aPSfx7Vi+iFOGcY1rTmV\/ZW4pw+EFEa9euDU5QYtGL5jYC0B2eCl4WtcuWLaPZs2cXShl8Or0Bief2xBCNDoLXL74geMGXfwjIRxwWtWu37KKZr9QpE7UcxVmdWtHpX2lFHQ8rvXkM86I8Zy70hO7wSPByve6aNWuCjWqPP\/44ffjhh0GN7vnnn0+DBw+mY489lpo0aeJCXiXGgMRLhMipDpjYnaJDKhhwJgWTM53A175ULFi5OVhJ\/b9vfUS\/X7peiahlL2K19uKqw+nYww8u3ECW9iYycObM6yMVCHSHB4J3586ddP\/999PMmTOprq4uELWVlZXBWbzf\/va3gxMafGtIPL8Yw8TuF19Y4QVfPiAQvmXs0WUf0Dvr\/6Vc1J7WoSUdc\/hBwckI4ugvVdhgXlSFpBk70B0eCF6+aa1Pnz60efNmr44eK5XCSDwzL7gqL5jYVSFpzg44M4e1Ck8NlS9RfvDMXzbS6+9vC2ppF767WQVkhZXZC05sS8e3K9MiaksF2lA5U0KOg0agOzwQvLzCu3r1aurUqZOzx4ylzW0kXlrE7PbHxG4X\/yzewVkW1OyN8ZUvsUrLq6e\/eeEf9PY6dau0zIYoPzjuiIOpd9fDA4JUr9RmZd1XzrI+r+\/joDs8ELxihXf06NHB6Qtp2\/z582nMmDFUW1ubdqi2\/kg8bdBqMYyJXQusWo2CM63wKjfuIl\/Ro7xWbdxBNa+pq6UVIApR261DS\/rmcYcGZQ3dO+09CcHl5iJnLuNlOzboDgheKzmIxLMCe2anmNgzQ2dtIDizBn0mx7b4Cq\/QLl69hWa8XBfEr6rsQKzI8p8sYrn8oMWXmmbeKJYJXE2DbHGm6XEavFnoDo8EL5\/QkLVVVFRghTcreBiHSww8zAF8GPtFmk6+hKh9ZdVm+tM7m7QJWt4YdsZXWgY3iwmh6\/oqbZ4s0clZnrgwNh4BCF4PBO+WLVvonnvuIf4za+MLKG666aasw5WPQ+Iph1SrQUzsWuHVYhycaYFVm9EsfEVLDmr\/tolefm\/vhjCVK7RCvLKgPeXoQ4jraXWceqANXE2Gs3CmKRSYlUAAusMDwSvBo3ddkHh+UYaJ3S++OFpw5hdncXxFBe2ClZsCIavi5rAoOqKO9ujDmlOfU46g9zfvCkoQhNj1C00z0eIdM4OzKi\/QHRC8qnIplR0kXiq4rHfGxG6dgtQBgLPUkFkZIMoN1ny4jZ5\/cz2t27m\/dkF72tGHUKe2BzWIOlorpH3uFO+YTfTT+4bugOBNnzUKRiDxFIBo0AQmdoNgK3IFzhQBmcEMi1heMQ1vCJtSuyY4h1bH6qxYheUyg6+WH0xdj2pBZ3duXYi8IdfRZqBH2RC8Y8qgNGIIugOC10iiRZ0g8azAntkpJvbM0FkbCM70Qh8uN9i4fTc999ZH9PcPdyqvnRVilv9kQdvr+MOoa8UhwcO5ch6tXqTdtY53zF1u4iKD7oDgtZKxSDwrsGd2iok9M3TWBoKzfNAvWLl389fb6\/9FS\/+5LRCzOldnP\/30Uzql\/Zfo1M5HUNVRXwjasODN90QYrRoBvGOqEdVrD7oDgldvhhWxjsSzAntmp5jYM0NnbSA42xd6sSrLPxF\/f+6tD+m1v2\/VKmbF6myXihbBFbhiM1hYzIIva69KZsfgLDN0VgZCd3gsePfs2UObNm2izz77jA499FDab7\/96JNPPvHi+mEknpX3PbNTTOyZobM2sLFyJupm\/\/7RTlq1cSctWrVFm5gVglUc0cVX3zY\/YP+A87TlBo2VL2sviALH4EwBiAZNQHd4KHhZ6D777LP005\/+lDZs2EB8qURNTQ0deOCB9MMf\/pBOPfVUGj58uNPCF4ln8C1X4AoTuwIQDZtoSJyJTWCizKCi9YE08f\/+g1Z+sEO7mBWrs2d1bkXdO9XfCKZyM1hD4stwqltzB86sQZ\/JMXSHh4L3T3\/6Ew0ePJhOPvlkOuaYY+iFF14IBG9ZWRn9+Mc\/pnnz5tHo0aOpf\/\/+qZJi2LBhNHfu3MKYyZMnU8+ePRNtzJ8\/n4YMGVLo161bN5o2bVoQT7GGxEuE1akOmNidokMqGJ84C59m8M76f9H8FR\/SirX\/MiZmzzmmNVW0\/pLVyxR84ksqARtBJ3DmF8nQHZ4J3l27dgXi8t\/\/\/jexIH3xxRdpzJgxgeAtLy+n3bt3EwvXjRs3JorO8KPzmCVLlhTsCBHLK8XV1dVFs3rSpEk0fvz4IBYWx+vWraM+ffpQu3btSvpH4vk1UWBi94svjtY2Z3FHcz269AN654N\/BfWzujaA8bOLlVcuNTipogV95\/g2AYHi31WuzKrKDNt8qXqOxmQHnPnFNnSHZ4JXCMprrrmGrrjiCmJhGha8\/DgPPPAA3XvvvQXxmpSSy5cvD1aDx40bV29Fl0VwXV1dUeEqYunbt289UcwxjRgxgmbOnEldunSJdY\/ES2LFrZ9jYneLD5loTHAWPprrjfe30dN\/2RiEpvpa2\/DzhsXs6R1bBhcoxG0Ck8HIpT4m+HLpeRtCLODMLxahOzwVvFdeeSVdffXVsYKXV10ffvhheuihh6ht27aZMzJJ8MaJbVlnSDxZpNzoh4ndDR7SRJGFs7hV2WdXfEjL1myj1Rv1HcslnissZi895Qj6SpuDnF+ZTcNJqb5Z+FLlG3ayIQDOsuFmaxR0h2eCV5Q07Nixg6ZOnUpMYHiFd82aNXTVVVfRUUcdFZQZ8Ea2LC1aqhBng\/vU1tbSbbfdFvhk39x69+5NEyZMKOkWiZeFFXtjMLHbwz6r52KchVdl+YzZucs+CFzoLDFg+yxmucTg6MOaB7eBXXRS26C0weUyg6zYZxmHdywLanbHgDO7+Kf1Dt3hmeDlcJcuXUpc0sAb1vi\/p59+mn70ox\/RypUr6fe\/\/31QxztlyhT6xje+kTYfghVjsQEtSbiKTW4tWrQolC+kreGdNWtWcMqEaFyHjOYeApjY3eNERFS39VNqd0jTetfYzn9rM\/2lbju9\/f4m2rDzizNndTwF++bGYrbz4QfRxV0Pp\/2aNAn+Le3RXDri88Um3jFfmPoiTnDmNmesR8KNF+X69esXLNSFdYfbT6EvuiZ7+MwvD9pLL70UHEu2atWqetFyCcMvfvELOu+883I9xfbt22nQoEFBDa\/YEBc1KARv9DQHIZpLnfIgftOK2uRa4gEDBuSKHYPVI8DfLPARePwLSdOmewUOmjkEXnv\/YzqyRVNicfvn9z+mJe9\/HDjnf9fZhJhl31XtDqRuRzUP3PH\/r92292YwNDUI4B1Tg6NJK+DMJNrpfc2YMSNYjIs2CN69iHgjeDlY1uZ8GsNf\/\/pX4hfv+OOPDwTJ\/vvvPfg8bxOb2fgItLiTGljwPv\/88\/tsTiu2mS0cjxC8vFGuffv2hR9x\/Fjlzcuc+vE7d+6k9evXB78VQ\/Dmx7d+rexO+vKhzWnGy3W0ePWWwDiLSRa3ulq4VrZjm+Z04QmH0gH77xe4458JoavLP+zuiwDeMf+yApy5zRlrkfAq76JFi2jixIlY4f2cNq8Er+5USxK8XMPLpRPR0xjSCF78pqWbRTX28dVdehz5kgQWj3zTV+3fNtHqD\/Vv+hKClf88\/egyOqzZp9Szy1HBLykcCwttvkABzT0E8I65x0lSROAsCSG3fo4a3vp8OC94t2zZQvfccw\/xn0mtTZs2QWnDCSecUHLVt9gxYkmlCcUEMY4lS2LGv5839oldbPRi5sTfN+34hP7w5kbtZ8qKbAmvylYeuXfTF1HxWtnGzplvbxn48o0x+2dd+4eY3YgheD0TvFxHed1119Gbb75JfFIDNxa23Li8Ia6dffbZdOedd9IhhxwS+3NRr8s\/FDekCTFbWVlZ8gKJaFmDWN3l641LndSAxLP74qf13tA\/jMVNX4zL2i276NXVW2jFOr03fcUJ2ePbldExhx9Exx1xcPDjPCcYNHTO0uaw6\/3Bl+sM7RsfOPOLM+gOzwQvh7t48WIaOnRocFnED37wg8IVvnw6w\/\/+7\/8Slxrcdddd9LWvfY0ef\/zx4Ngy7nvDDTeUzM7o1cLRW9aKlTCII8yE8aTTHbgfEs+vicLHiT18\/BajvWP3Z8GVtW9bELIsXFnEnvzlvb905hGyspnjI2eyz9YQ+4Ev\/1gFZ35xBt3hmeAVq7HHHnss3XLLLdTk8+N\/xGPwRrabb76Z\/v73vwfn8DZv3pzGjh0bCMxHH33UmexE4jlDhVQgrk3s4RXZ19\/fRn+t+xf9\/SMzNbJCsPIxXB0Oa05nd25Fn+0h5277co0zqURrxJ3Al3\/kgzO\/OIPu8EzwipIBXuG97LLLYrONb1i7++67C8eJPfLII3THHXcEOxNdaUg8V5iQi8PUxB4Wsls\/\/pSef+sjeme9mdKCsJDlFVi+HKHqqPorsuHVWTnk7PUyxZm9J2xYnsGXf3yCM784g+7wTPBu2rSJBg4cSJ06dQpuOGvWrFm9J+CyhlGjRtG7775L9913H7Vu3Tqo3+XLKZ555hlnshOJ5wwVUoFkndijV9WysxXrttPyNdvp74ZOLQgL2a+0aU6nHN2S+M+weBUlBlJgeNIpK2eePF6DCxN8+UcpOPOLM+gOzwQvh3vvvffS+PHj6dJLLw02sIlza3n1l2t358yZE9TrXnvttUG9749\/\/OPgjF4Wvq40JJ4rTMjFUWpi5+O3uL23cQet\/vDjYMMXt4Xv7v13nU1cV8t\/VrT+Ep3duXU9IRsWtTrjcNE2PoxdZKV4TODLL744WnDmF2fQHR4KXl7FZcH7u9\/9jj777LN6T8CXTvBGNt5wxi\/j1VdfHdyWxiKZT1xwpSHxXGFi3zjijuB6f9MOenrZ+\/TR7qb0z00fF47m0vUUYSF7xldaUcfDGv6KrGos8WGsGlG99sCXXnx1WAdnOlDVZxO6w0PBK0LmI8r4pjM+Qowbr+J+85vfLNxcxrfA8Kpvu3bt6MADD9SXRRksI\/EygKZ4iLgY4S912+mZNzfS6o36N32Fz5I9+5jWxBu\/uneqfxFCQywvUEydlDl8GEvB5Ewn8OUMFdKBgDNpqJzoCN3hseB1IoMyBoHEywhcimFiA9iyf26jeX\/de0azrjKDsJA9sX0ZnX8CX4rwxfFb0b+neAx0zYgAPowzAmdpGPiyBHwOt+AsB3gWhkJ3eCh4+eix9957j+bOnRt72cSuXbvo\/fffD+6MFvW9FnKrpEsknhpGhKh97R9b6bkVH2opN2Axe2SLpnTkwUSndT6cTqxoWRCzWI1Vw6MOK\/gw1oGqPpvgSx+2uiyDM13I6rEL3eGh4OUTF\/iSCK7ljWt8cgPfdMab1PiUBhcbEi89K1yC8Mp7m+lPf9ukbKVWCFYuK\/jO8W1o267PgjKD6MUImNjT82V7BDizzUA6\/+ArHV4u9AZnLrAgHwN0h2eC91\/\/+hcNHjw42Ij2m9\/8ho4++uhgY1pVVVVw+9rs2bNp+vTpwaUTJ554onwmGO6JxCsNOIvbp97YQFxfm6cMISxoTz26ZXBtbZZbvjCxG35BFLgDZwpANGgCfBkEW5ErcKYISENmoDs8E7zi4gk+kuz6668Pov\/5z39OH3zwQVDCwI1Xf3mT2rhx4\/a5ic1QXiW6QeJ9AZEoSxg7b1UmcSsE7DeOaU2XnlIeGOZ\/U1lugIk9MaWd6wDOnKOkZEDgyy++OFpw5hdn0B2eCt4f\/ehHdMkllwTRP\/DAA\/TEE0\/Q1KlT6ZBDDgn+\/8EHH6T777+f2rRp42RGNubEEwL38dc\/oGkL3pfmRwjYnl87jC466fBMK7XSziIdMbFnRc7eOHBmD\/ssnsFXFtTsjgFndvFP670x6444rJrs4R1hDretW7cGJQwnn3wyjRw5Moj0ueeeC1Z5Weh26NAhEL4sdmtqarBpzSEuuUxBdhVXiNvR53+Fyg85UPmKbVpYMLGnRcx+f3Bmn4M0EYCvNGi50RecucGDbBQQvJ6t8HK4t99+e3DpxI033kh9+\/YN6nmvuOKK4Oa173znO3TTTTcR63ZxtbBsMpjs1xgST6zk1ry2jh5YtLYkvOKihZG9OloXt3GBYmI3+Xao8QXO1OBoygr4MoW0Oj\/gTB2WJiw1Bt2RBkfnV3j5YXiVl+t3+U8Wta1ataIxY8YEm9VY6DZp0oRuvvlm6t+\/f5pnN9q3oSfe7FfXBau54VvLogCzyL2xZwc6q3NrpfW2OojExK4DVb02wZlefFVbB1+qEdVvD5zpx1ilh4auO9Ji5YXg5YdiYbtt2zZq0aJFIHD5iuGXXnqJXnzxRerVq1dQ8sD\/7mpriInH4valdzdT9ewVRWFnkXtX30onV3FL5QomdlffpOJxgTO\/OANffvHF0YIzvzhriLojDwPeCN5SD8nid8uWLdSyZUvaf\/\/98+ChbWxDS7wpL66hHz\/6t1i8hMg9q3P9K3S1gavBMCZ2DaBqNgnONAOs2Dz4UgyoAXPgzADICl00NN2RFxrnBa84lmz06NHUs2fP2Od96qmnaOzYsdi0ljcbEsbzii6fkTu0yIru5aeVk6jJ1RyKdvOY2LVDrNwBOFMOqVaD4EsrvFqMgzMtsGozCsFbH1onBS9fFczlCjt27KDNmzfTHXfcQRdffDF16dJln8TgvlzXy30feughatu2rbbkyWPY98RjsXvRpKX71Ojyau4FJ7ala86ucL4uNw1\/mNjToOVGX3DmBg+yUYAvWaTc6QfO3OFCJhLfdYfMM6bp46Tg5QeYOXMm3XLLLUHtblLjMoYRI0YEx5e5Wsfra+Kx0H12xYc04vfv7EPDgDPb0f\/f4+gGJXTFQ2JiT3rr3Ps5OHOPk1IRgS+\/+OJowZlfnPmqO3Sh7Kzg3b17d1CXu379ehoyZAj993\/\/N5199tn74NCsWTNq3bq1s0JXBOxj4pVa1X28uqpBCl0IXl1TjX67+DDWj7FKD+BLJZpmbIEzMzir8uKj7lD17HF2nBW8IljekPbRRx8FN6rx9cG+Nt8Sj48Zi9bqNoTNaLL5g4ldFil3+oEzd7iQiQR8yaDkVh9w5hYfSdH4pjuSnifvz50UvELk8p+yjcsaDj30UJzSIAtYiX6zFq+l6+a8Va8Hi92GvqobfmBM7AoSybAJcGYY8JzuwFdOAC0MB2cWQM\/hEoK3PnhOCl5xMsOaNWukqa6oqMApDdJoxXfkEobHl39AP3vi3Xod7ur7VerX7cic1v0ajondL744WnDmF2fgyy++8I75xxcErweCd+fOnbRw4ULiExhkG5c7dO\/enZo3by47xGg\/1xOPxe7sV9fS2HmrC7jwqi4fM8bHjTW2hg9j\/xgHZ35xBr784guC1z++XNcdphF1coXXNAgm\/LmceCx2r5uzghas3FxP7DamEoZoDuDD2MRbodYHOFOLp25r4Es3wurtgzP1mOq06LLu0PncxWx7JXg3bdpECxYsoNdee434dIaqqio644wzglMasrRhw4bR3LlzC0MnT55c9HKLYvYnTZpEc+bMSSyncDXxWOzOeXUd\/Z95qyB2QyRjYs\/yRtkdA87s4p\/WO\/hKi5j9\/uDMPgdpInBVd6R5BpV9vRC8vHnt3nvvpQkTJlB0IxtvVhs6dChVV1cHIli2sdhdsmRJQajOnz8\/OP5s+PDhgS2Ztnz5curfv39wpXFNTQ2Vlxf\/6t\/VxJu6YA2NfOSLK4Ib2+a0YjxjYpd5A9zqA87c4iMpGvCVhJB7Pwdn7nFSKiJXdYctFL0QvA8\/\/DCNHDmSzjnnHLrhhhuoQ4cOAV6rV6+m22+\/PbiVbdy4cXTRRRdJ4SiEKo8JX1fMIriuro6mTZtGZWVlJW1t376dBg0aRIsXLyaZDXMuJt7Yeav2qdltzGUMYcIxsUu9Sk51AmdO0ZEYDPhKhMi5DuDMOUpKBuSi7rCJoPOCVwhLFqB33nnnPpvSeIPb9ddfH6z8cklCnrN60wheLmWora0NyiqeeuopL1d4Dx32QiH3sLJb\/zXExG5zWsrmG5xlw83WKPBlC\/nsfsFZduxsjITgrY+684JXHFF2zTXX0BVXXBGbMw888EBQ8pBUVlAq4VjAjh8\/PhDN4VXfuDFc\/sBXGfP1x3yahG81vFy323XMyxC7JRICE7uN6TmfT3CWDz\/To8GXacTz+wNn+TE0aQGC1zPBu2HDBrrsssvo4osvDlZy4xqv\/D766KP00EMPUdu2bVPlk6jd5UG9e\/cO6oRLNSHA+\/btG9T6pt20NmvWrKAEQrRSdb+pHiRF54unvEEL3\/3iRIaJl3amy09rXOfsJsGFiT0JIfd+Ds7c46RURODLL744WnDmNmesT8KN7zLo169f8G10WHe4\/RT6onN+hfeTTz4hLjVYsWIFTZ06lTp27FgPjVWrVtHVV19NlZWVgVg94IADMqElSie4hrfUSnG07CGt4I0Gx5veBgwYkCnmLIMe++t2uvX5jYWhV53Skq7\/erZTLrL492UMnwHNv2zxLyRNmzb1JexGHSc484t+8OUXXxwtOHObsxkzZgTfPEcbBO9eRJwXvBzkG2+8QQMHDgxetm9961vBBRPcuJzg2WefDep2uaSB62nzNLGZbfDgwbEnNYRLGbp06RK4Sit4eaNc+\/btC2GyoDK1ysulDGeMX1rwzXW7rwzPh1kevF0ey7Xh69evD34rhuB1makvYgNnfvAkogRffvHF0YIztznjFd7wKu+iRYto4sSJWOH9nDYvBC\/H+vbbb9OoUaNo2bJltGfPnr1qvUkT6tq1K91222103HHH5c7EJMEbPbc36rDUkWa2a2lY7A6dvaJQyoBNaqXTBV\/d5X6djBsAZ8Yhz+UQfOWCz8pgcGYF9sxObeuOzIFrGuiN4BXPz6u8fAEFN75wIsupDHErtWxP1PPKbFwT8aRd4bX11UL0CLK7L69slFcGy75HmNhlkXKnHzhzhwuZSMCXDEpu9QFnbvGRFA0Eb32EnBe8XEd53333BWfsHnvsscQXTeRtol6X7Ygzd8XqLtcCy5zD65PgjTuVYdnoM\/PC2KDHY2L3j15w5hdn4MsvvjhacOYXZxC8HgpePqWBL5k47LDDAuF71VVXBbWVXNKQp0VLFKIlCbx6O2XKlKAIXNTsRv35sMJ74d1L65Uy3NW3ks7q3CoPdA1+LCZ2\/ygGZ35xBr784guC1z++IHg9E7wc7u7du+nll1+m6dOnB3\/y\/\/NpDeK4srRHkdlIW1uJt2DlZrpo0hcb1SZcehxddWY7GxB45RMfxl7RFQQLzvziDHz5xRfeMf\/4sqU7XEXK+ZKGKHBR8cvHlnEZAp\/icMEFF1CzZs2cxNpW4kVXd1HKIJce+DCWw8mlXuDMJTaSYwFfyRi51gOcucZI6Xhs6Q5XUfJO8IaB3Lp1K\/3ud78LVn5btGiR66Y13QTZSLzZr64LTmYQ7fHqKpQySBKNiV0SKIe6gTOHyJAIBXxJgORYF3DmGCEJ4djQHS4j5J3g5Q1nL7zwQiBumUxuJ510UrDCy2f0YoV3b7rxRjUuZeA\/ufExZFjdlX8VMbHLY+VKT3DmChNycYAvOZxc6gXOXGIjORYI3voYeSF4oyL3s88+C2p4f\/CDH9B3v\/vd4Hgy15vpxIuu7rLYZdGLJocAJnY5nFzqBc5cYiM5FvCVjJFrPcCZa4yUjse07nAdHecFL98a0qdPH+I7oXlz2pVXXkkXX3xxvdvKXAeZ4zOZeHGXTGB1N12WYGJPh5cLvcGZCyzIxwC+5LFypSc4c4UJuThM6g65iOz2cl7w8iUTs2fPpl69etFXvvKV3EeR2YLbZOJhdTc\/y5jY82No2gI4M414Pn\/gKx9+NkaDMxuoZ\/dpUndkj9LcSOcFrzko9HoymXg4mSE\/l5jY82No2gI4M414Pn\/gKx9+NkaDMxuoZ\/dpUndkj9LcSAheQ1ibSrzorWq4QjgbwZjYs+FmcxQ4s4l+et\/gKz1mtkeAM9sMpPNvSneki8pebwheQ9ibSjys7qohFBO7GhxNWgFnJtHO7wt85cfQtAVwZhrxfP5M6Y58UZobDcFrCGsTiRdd3R3ZqwON7NXR0BM2LDeY2P3jE5z5xRn48osvjhac+cWZCd3hEyIQvIbYMpF4M1+pox\/VvB08Ec7dzUcsJvZ8+NkYDc5soJ7dJ\/jKjp2tkeDMFvLZ\/JrQHdkiszPKecG7ZcsWuueee4Jrg0844YRYlF577TW64447aMKECcHRZS423YkXXd39Yff2NO4\/j3URCi9iwsTuBU31ggRnfnEGvvziCyu8\/vGlW3f4hojzglecwzt69Gjq2bPnPvjyJRTjx4+np556qlFfLbxg5ebgZjXRcI1wvlcRH8b58LMxGpzZQD27T\/CVHTtbI8GZLeSz+YXgrY+bk4J39+7dNGrUKHrkkUekWf6P\/\/gPuvPOO6l58+bSY0x21J142Kymlk1M7GrxNGENnJlAWZ0P8KUOS1OWwJkppNX40a071ERpzoqTgpcff9WqVTRjxgziiydeeOEF6tq1a+ztai1atKCqqio666yziP\/uatOZeNispp51TOzqMdVtEZzpRlitffClFk8T1sCZCZTV+dCpO9RFac6Ss4JXQCBTw2sOruyedCbe2HmraOy81UFw2KyWnaPwSEzsanA0aQWcmUQ7vy\/wlR9D0xbAmWnE8\/nTqTvyRWZntPOC1w4s6r3qTLxwOUP3Tq3oiaFV6h+gkVnExO4f4eDML87Al198cbTgzC\/OdOoOv5DYG62Tgles6nKA3\/\/+9+nBBx8k\/rdSrWXLlnTttdcS\/+li05V40XIGbFZTwz4mdjU4mrQCzkyind8X+MqPoQ5+at4AACAASURBVGkL4Mw04vn86dId+aKyN9pJwStOZmBYeCPa9ddfT2vWrCmJUkVFRaM8pQHlDHpeHkzsenDVaRWc6URXvW3wpR5T3RbBmW6E1dqH4K2Pp5OCVy3lbljTlXgoZ9DDLyZ2PbjqtArOdKKr3jb4Uo+pbovgTDfCau3r0h1qozRnDYLXENY6Eg\/lDPrIw8SuD1tdlsGZLmT12AVfenDVaRWc6URXvW0dukN9lOYsOi94RT0vanj3TQqUM+h7UTCx68NWl2VwpgtZPXbBlx5cdVoFZzrRVW8bgrc+ps4LXlHPixrefV8GlDOonyCERUzs+rDVZRmc6UJWj13wpQdXnVbBmU501duG4PVM8BZLgT179tDGjRtpzpw59Nhjj9Edd9xBJ5xwgvqMUWRRdeJFyxmWjT4zOIMXTQ0CmNjV4GjSCjgziXZ+X+ArP4amLYAz04jn86dad+SLxv5o51d4kyBi4XvzzTcHN7JNmDCBDjjggKQhVn6uOvFmv7qOhs5eETwLLptQTykmdvWY6rYIznQjrNY++FKLpwlr4MwEyup8qNYd6iKzY8l7wcuwPfTQQ3T33XenPpZs2LBhNHfu3ALykydPpp49e5ZkYvv27TRo0CBavHhxod\/w4cOpurq65DjViYdyBr0vDCZ2vfjqsA7OdKCqzyb40oetLsvgTBeyeuyq1h16ojRn1XvBu3v3bho1ahS9+eabdP\/991ObNm2k0GOxu2TJkoJInj9\/Pg0ZMoRKiVdRT9yuXTuaNm0alZWV0fLly6l\/\/\/7Uo0ePYIW5WFOZeNFyhpG9OtDIXh2lnhud5BDAxC6Hk0u9wJlLbCTHAr6SMXKtBzhzjZHS8ajUHX49eXy0zgvepFMa3nvvvUC4suj82c9+Rk2aNEnkRYjUcePG1VvRZRFcV1dXELNRQyyKR4wYQTNnzqQuXboUfjxp0qSglrimpobKy8tj\/atMvAUrN9NFk5YW\/OB2tUTKU3fAxJ4aMusDwJl1ClIFAL5SweVEZ3DmBA3SQajUHdJOHe7ovOBNOqWhWbNmdMkll9BNN91EhxxySC6okwRvMeMseKdMmbKPEA73V5l4\/af\/hZ58fUNgHvW7uSgvOhgTux5cdVoFZzrRVW8bfKnHVLdFcKYbYbX2VeoOtZHZsea84DUFC4vW8ePHk0wdbzSmaHlEXMwi8WbNmkV8DbJoxVaESz33qf\/nVeKyBm439KhAOYOGJMHErgFUzSbBmWaAFZsHX4oBNWAOnBkAOYcLXiAMNz7OtV+\/flRbW1tPd+Rw4fVQbwUvn86wbds2atGihVQZQzGWRO0u\/7x3794l63DjbMjU\/vI4IXijNrgUY8CAAdJJVLf1U7pwxppC\/3svKadT2uM4MmkAJTvu2rWLNmzYEJSoNG3aVHIUutlEAJzZRD+9b\/CVHjPbI8CZbQZK+58xY0bwTXO0QfDuRcQLwcsb0379618HG9N4JZY3i\/ExZAMHDgxqbn\/yk5\/QhRdemEv4itMX2F6pWtxwIola4MrKyqJ1v6K\/ELxcN9y+ffuCGRZUaVZ5b3\/+n3T783sFL5czvDK8yu030NPodu7cSevXrw9+K4bg9YNEcOYHTyJK8OUXXxwtOHObM17hDa\/yLlq0iCZOnIgV3s9p80Lw3nvvvUG5Adfqjh49OhC8\/OLNmzcvEJorV64MVmbPP\/\/8XNkoBOzgwYMTjxlLI3Y5KFW1NLxZjTetceveqRU9MRSCNxfpRQbjqzsdqOq1Cc704qvaOvhSjah+e+BMP8YqPajSHSpjsmnLecErVnKPOeYY+sUvfrHPxRK8+ss1tHzrmjgqLCugsoJXlDF069ZN2qeKxMNxZFmZTT8OE3t6zGyPAGe2GUjnH3ylw8uF3uDMBRbkY1ChO+S9ud\/TecErTmkYOnQoXXbZZbGIPvLII8HVwrKlCMWOFxNCttTGNdEnbb2visTDcWTmXihM7OawVuUJnKlC0owd8GUGZ5VewJlKNPXbUqE79EdpzoPzgpdXbq+88ko699xzaeTIkbHIjB07lv74xz9KXzwh6nXZWPQCiVL1uLKXTMQFqSLxxs5bRWPnrQ7M4zgyvS8JJna9+OqwDs50oKrPJvjSh60uy+BMF7J67KrQHXois2PVecHLpzHceuut9OSTT9KvfvWrQPiKyyX4Zyx0b7zxRrrgggukL54QUEevFo7eshY9XzfaP0pZqZVhFYmH64TNvSSY2M1hrcoTOFOFpBk74MsMziq9gDOVaOq3pUJ36I\/SnAfnBS9DwWfJXXPNNfTWW28Fl0vwUWTc+IgUXgH+6le\/SryxLXy+rTkI5TzlTTzU78rhrKoXJnZVSJqzA87MYa3CE\/hSgaJZG+DMLN55veXVHXn9uzbeC8HLoPGpDFyjy\/9t3bo1wJHFb58+fYL\/mjdv7hq29eLJm3hRwYvrhPXSjYldL746rIMzHajqswm+9GGryzI404WsHrt5dYeeqOxZdV7wctnCY489RscffzzxSQ2+tryJh\/pds8xjYjeLtwpv4EwFiuZsgC9zWKvyBM5UIWnGTl7dYSZKc16cF7x82xWfzvC9730v8Wxcc7Cl95Q38VC\/mx7zPCMwsedBz85YcGYH96xewVdW5OyNA2f2sM\/iOa\/uyOLT5THOC15xSgPfpFZdXe0yliVjy5t4Xce8TFzWwO3uyyvp8tPKvcXCh8AxsfvAUv0YwZlfnIEvv\/jiaMGZX5zl1R1+PW1ytM4LXn6E5557jn7+858HK73f\/e536aCDDtrnyfbff3869NBDif90seVJPNTvmmcUE7t5zPN6BGd5ETQ7HnyZxVuFN3CmAkVzNvLoDnNRmvPkvOAVF0\/wSQ2lGp\/QIHvxhDl4v\/CUJ\/GiF058NOGbNh6hUfnExO4f3eDML87Al198YYXXP77y6A7\/njY5YucFL5\/OsHDhwuAIslLtwAMPpO7duzt7WkOexMOGteREVt0DH8aqEdVvD5zpx1ilB\/ClEk0ztsCZGZxVecmjO1TF4JId5wWvS2DliSVP4oU3rHHtLtfwoulFABO7Xnx1WAdnOlDVZxN86cNWl2VwpgtZPXbz6A49Edm1CsFrCP+siRet3102+szgWmE0vQhgYteLrw7r4EwHqvpsgi992OqyDM50IavHblbdoSca+1adFLyibpfhufPOO+n6668Pblsr1RpqDW+0fhcXTph5aTCxm8FZpRdwphJN\/bbAl36MVXsAZ6oR1WsPgrc+vk4K3i1bttA999wTRPr973+fHnzwQeJ\/K9VatmxJ1157LfGfLrasiRcWvLyyyyu8aPoRwMSuH2PVHsCZakT12gNfevHVYR2c6UBVn82sukNfRHYtOyl47UKix3vWxMOFE3r4SLKKiT0JIfd+Ds7c46RURODLL744WnDmF2dZdYdfTykfrReCl68XfuaZZ6i2tpZ++tOf0sEHH0xbt24NLqI44IAD6KabbqLjjjtO\/qkt9MyaeGHBO7JXBxrZq6OF6BufS0zs\/nEOzvziDHz5xRcEr398ZdUd\/j2pXMReCN6nn36ahg0bRl26dKHJkydT69atgxKHX\/3qV\/TEE08QH0l27733UlVVldxTW+iVJfFw4YQFoj53iQ9je9hn9QzOsiJnZxz4soN7Hq\/gLA965sdm0R3mozTn0XnBu337dho0aFBwvi5vYCsrK6uHzqZNm2jIkCHB7Wsshln8utiyJB42rNljEhO7PeyzegZnWZGzMw582cE9j1dwlgc982Oz6A7zUZrz6LzgFSc2XHPNNXTFFVfEIvPAAw8EK7wN7aa12a+uo6GzVwTPjA1r5l4K9oSJ3SzeKryBMxUomrMBvsxhrcoTOFOFpBk7ELz1cXZe8G7YsIEuu+wyuvjii4PjyeLapEmT6OGHH6aHHnqI2rZtayaTUnrJknjYsJYSZIXdMbErBNOQKXBmCGhFbsCXIiANmgFnBsFW4CqL7lDg1lkTzgveTz75JKjfXbFiBU2dOpU6dqy\/aWvVqlV09dVXU2VlJU2YMCHYxOZiy5J42LBmj0lM7Pawz+oZnGVFzs448GUH9zxewVke9MyPzaI7zEdpzqPzgpeheOONN2jgwIG0bds2Ovnkk+noo48OENq4cSO9+OKL1KJFC7rvvvvoxBNPNIdcSk9ZEu\/QYS8UvODCiZSA5+yOiT0ngBaGgzMLoOdwCb5ygGdpKDizBHxGt1l0R0ZXXgzzQvAyku+\/\/z798pe\/pOeee452794dgNusWTM677zz6Mc\/\/jG1b9\/eacDTJl50wxquFDZLLyZ2s3ir8AbOVKBozgb4Moe1Kk\/gTBWSZuyk1R1morLnxRvBaw8iNZ7TJh42rKnBPasVTOxZkbM3DpzZwz6LZ\/CVBTW7Y8CZXfzTek+rO9La960\/BK8hxtImXs2SdTRkFk5oMETPPm4wsdtCPrtfcJYdOxsjwZcN1PP5BGf58DM9Oq3uMB2faX8QvIYQT5t44Q1r3zm+DT34Q3frkw1BaNQNJnajcCtxBs6UwGjMCPgyBrUyR+BMGZRGDKXVHUaCsugEgtcQ+GkTL7xhDVcKGyIp5AYTu3nM83oEZ3kRNDsefJnFW4U3cKYCRXM20uoOc5HZ8QTBawj3tImHExoMEVPEDSZ2u\/hn8Q7OsqBmbwz4sod9Vs\/gLCtydsal1R12ojTnFYI3A9Z8LvDcuXMLI\/lK4549e5a0lCbxcEJDBlIUD8HErhhQA+bAmQGQFboAXwrBNGQKnBkCWpGbNLpDkUunzUDwpqSHxe6SJUsK1xjPnz+fhgwZQsOHD6fq6uqi1tIk3th5q2jsvNWBLVwpnJIgRd0xsSsC0qAZcGYQbAWuwJcCEA2bAGeGAc\/pLo3uyOnKi+HeCF6+Ue0Pf\/gD\/eMf\/4gFtmXLlnTttdcS\/6mrLV++nPr370\/jxo2rt6LLIriuro6mTZtGZWVlse7TJB6uFNbFoLxdTOzyWLnSE5y5woRcHOBLDieXeoEzl9hIjiWN7ki25n8PLwTv008\/HVwvLC6ciIO9oqKisOpqmhadgve3\/Y+ni7sebvqRGr0\/TOz+pQA484sz8OUXXxwtOPOLMwje+nw5L3i3b99OgwYNog8++IB+85vf0PHHH09NmjRxJusmTZpE48ePp6Q6XpF4s2bNIhbnopWXl+\/zLIff+GLh3yZe2pkuP+1IZ563sQSCid0\/psGZX5yBL7\/4guB1n69169bVC3LNmjXUr18\/qq2trac73H8SPRE6L3iZwD59+tCVV15JV199tR4UMlgVtbs8tHfv3jRhwoSSVoTgjXbiEokBAwYU\/rlu66d04Yw1hf+\/95JyOqX9lzJEiCF5ENi1axdt2LCB+BeSpk2b5jGFsYYQAGeGgFbkBnwpAtKgGXBmEOwMrmbMmEEzZ87cZyQE715InBe8mzZtooEDB9L555\/vlOAVGSVWoLmGt6amJhBIcU0IXq7\/bd++faEL9w+PWbByE\/W5760vBPBtZ2ZIewzJi8DOnTtp\/fr1wW\/FELx50TQzHpyZwVmVF\/ClCklzdsCZOayzeOIFwvAq76JFi2jixIlY4f0cTOcFL8d577330jPPPBP82bZt2yx5oHWM2Mw2ePDgoic1yNbS4IQGrVRJG8fXrdJQOdMRnDlDhVQg4EsKJqc6gTOn6EgMRlZ3JBpqIB2cF7z8G+Uf\/\/hH+t3vfkdvvfUWnXPOOdSiRYt94DdxSkMxzlUK3qGzV9DsV\/fW4XTv1IqeGFrVQFLNr8fAxO4XXxwtOPOLM\/DlF194x\/zjC4K3PmfOC15Rw8vF16WaiVMauG53xIgRQY1Mly5dCuGIet5SG9dkEy98JBmuFLY3weDD2B72WT2Ds6zI2RkHvuzgnscrOMuDnvmxsrrDfGR2PDoveO3AEu9V1OvyT8WZu2J1t7KyUsk5vOErhSF47bGPid0e9lk9g7OsyNkZB77s4J7HKzjLg575sRC8nq3wmk+RZI\/Rq4WTblljizKJ94+PPqauY14uBPB4dRWd1blVckDooRwBTOzKIdVuEJxph1ipA\/ClFE4jxsCZEZiVOZHRHcqceWDImxXed999l8aMGUMvv\/wyHX744cGJCAcddBDddNNN9O1vf5suvPBCp87njXIvk3gLVm6miyYtLQxdNvrM4GphNPMIYGI3j3lej+AsL4Jmx4Mvs3ir8AbOVKBozoaM7jAXjX1PXgjeN954IziabL\/99qMOHTrQ2rVrA8F74IEHBv++YsWK4FKKnj172ke0SAQyiceb1XjTGjcWuix40ewggIndDu55vIKzPOiZHwu+zGOe1yM4y4ug2fEyusNsRHa9OS94P\/nkk+BaYd60xseSLV26NFjpFWfebt26NTifl1d7edMYi2AXm0zi4Ugyd5jDxO4OF7KRgDNZpNzoB77c4CFNFOAsDVr2+8roDvtRmovAecHLt11ddtll9L3vfS8445ZPRAgLXoZq6tSpdP\/995e8+MEcpPGeZBIvfEIDjiSzyxgmdrv4Z\/EOzrKgZm8M+LKHfVbP4CwrcnbGyegOO5HZ8eq84BXHkl1zzTV0xRVXxAreBx54IFj9LXXTmR14v\/Aqk3g4ksw2S1\/4x8TuDheykYAzWaTc6Ae+3OAhTRTgLA1a9vvK6A77UZqLwHnBK44Ca9OmDU2YMCG4hCKupOGAAw6gKVOm0MEHH2wOvRSekhIvekLD3ZdX0uWnxV9TnMItumZEABN7RuAsDgNnFsHP4Bp8ZQDN8hBwZpmAlO6TdEdKc953d17wMsJPP\/008dFfvMJ79NFH0z333BPU63Jd71133RVsWhs7dmxQ9uBqS0o8HEnmFnOY2N3iQyYacCaDkjt9wJc7XMhGAs5kkXKjX5LucCNKc1F4IXj37NkT3G42btw42rFjRz109t9\/\/2BTG5c88N9dbUmJhyPJ3GIOE7tbfMhEA85kUHKnD\/hyhwvZSMCZLFJu9EvSHW5EaS4KLwSvgINPZFi8eHHw3+7du+mUU06hs846i1q3bm0OsYyekhIPR5JlBFbTMEzsmoDVaBacaQRXg2nwpQFUzSbBmWaAFZtP0h2K3TlvzivB6zyaJQJMSrzwkWQ4ocE+05jY7XOQNgJwlhYxu\/3Bl138s3gHZ1lQszcmSXfYi8yOZ28EL5c1vPPOO\/T73\/+etm3bFqDFG9kuueQS6tixox30UnhNSjwcSZYCTANdMbEbAFmxC3CmGFDN5sCXZoA1mAdnGkDVaDJJd2h07aRpLwQvlzL85Cc\/CTavsfANtyZNmlDfvn3p5ptvpmbNmjkJMgeVlHhdx7xMvHGN28heHWhkL\/dFvLNgKwgME7sCEA2bAGeGAc\/pDnzlBNDCcHBmAfQcLpN0Rw7TXg51XvCywOXNavfddx8NHTqUBgwYQIccckgANgthvnSCz+C94YYbgo1rrrakxDt02AuF0B+vrqKzOrdy9VEaRVyY2P2jGZz5xRn48osvjhac+cVZku7w62nyR+u84N24cSNdeeWVdNppp9Ett9xCvKIbbiyIeXX3jTfeCESxqxvYSiUejiTLn8iqLWBiV42ofnvgTD\/GKj2AL5VomrEFzszgrMoLBG99JJ0XvOKmNV7d5SuG49oTTzwRrAL7etMajiRT9Xqrs4OJXR2WpiyBM1NIq\/EDvtTgaNIKODOJdn5fELyeCd5du3bR9ddfH9Tn8k1r0TpdPp5s1KhRQXnDnXfeSQceeGD+LNFgoVTi4UgyDYDnNImJPSeAFoaDMwug53AJvnKAZ2koOLMEfEa3ELyeCV4Ol29U4\/rco446im688Ubq0KED7bfffsSrv3zT2vz58+mXv\/wlnXDCCYWn40so2rZtmzFN1A8rlXgPLl5L1895K3D65UO\/RMtGn6k+AFhMhQAm9lRwOdEZnDlBg3QQ4EsaKmc6gjNnqJAKBILXM8ErShpY9KZpFRUVVFtbm2aI1r6lEg9HkmmFPpNxTOyZYLM6CJxZhT+1c\/CVGjLrA8CZdQpSBQDB65ng3blzJy1cuJC4tCFN49KG8847L80QrX1lBS+OJNNKg7RxTOzSUDnTEZw5Q4VUIOBLCianOoEzp+hIDAaC1zPBm8ioJx1KJR7O4HWPREzs7nGSFBE4S0LIrZ+DL7f4kIkGnMmg5E4fCF6PBe+mTZtowYIF9NprrwWb16qqquiMM85w9iiyMNTFEg9HkrkzOYQjwcTuJi+logJnfnEGvvzii6MFZ35xBsHroeD97LPPgssl+JQG\/nu48eY0PrKsurray5vWIHjdnEAwsbvJCwSvf7wUixjvmH9cgjO\/OIPg9VDwPvzwwzRy5Eg655xzghvV+JQGbqtXr6bbb7+dXnrppeAc3osuusjZbCyWeNEzeD+a8E1nn6ExBYaJ3T+2wZlfnIEvv\/jCCq9\/fEHweiZ4t2\/fToMGDaKysrLgnN3mzZvXewLe1Mbn9PLK7+TJk707h3fsvFU0dt7q4JlwJJk7Ewo+jN3hQjYScCaLlBv9wJcbPKSJApylQct+XwhezwSvOJaMz+G94oorYjPogQceCEoefLxpbeYrdfSjmreD5+reqRU9MbTK\/luCCFCr5mEO4MPYL9LAl198YYXXP74geD0TvBs2bAiuFL744ouDldy4xiu\/jz76KD300EPSl02IlePFixcXTA4fPjyoBU5qkyZNovHjx6caVyzxwmfwnntsa3pkSNck9\/i5AQTwYWwAZMUuwJliQDWbA1+aAdZgHpxpAFWjSQhezwTvJ598QsOGDaMVK1bQ1KlTqWPHjvWeYNWqVXT11VdTZWVlsKntgAMOSEwfsWrcrl07mjZtWlAusXz5curfvz\/16NEjsFOssdidMmUKzZw5k7p06VIYN3jw4JJiWUbw4gzeROqMdcDEbgxqZY7AmTIojRgCX0ZgVuoEnCmFU7sxCF7PBC+H+8Ybb9DAgQODyye+9a1vUffu3YOn4Aspnn322aBul0sa+JgymcZXEY8YMaIgWsUYFrNz5swpWhohhPKpp55aTxSzIF+yZEnJkopiiXfosBcKIUPwyrBnpg8mdjM4q\/QCzlSiqd8W+NKPsWoP4Ew1onrtQfB6KHg55LfffptGjRpFy5Ytoz179gRP0aRJE+ratSvddtttdNxxx+XOnOjqbdSgasGLI8lyU6bNACZ2bdBqMwzOtEGrxTD40gKrVqPgTCu8yo1D8HoqeEXYvMrLF1Bwa926tdJTGWRWaouVNCSVQsQlHgSv8vdbmUFM7MqgNGYInBmDWokj8KUERqNGwJlRuHM7g+D1XPDmzoAiBrjMYciQISSzcU3U+27bti2wxseh9ezZs2RoIvFmzZpFFRUVQd\/VH5fRRZOWFsZ98KuzdT0e7KZEABN7SsAc6A7OHCAhRQjgKwVYjnQFZ44QUSQM\/hY63NasWUP9+vWj2tragu5w+wn0Rtdkj6gP0OvHaetCwPLGN7GJLS5gcbJDXV1doV43bgNc3FgheMM\/2\/3l7rTj5IHBP7U7pCk9MWCvEEazjwB\/k8AnhJSXl1PTpk3tB4QIEhEAZ4kQOdUBfDlFh1Qw4EwKJmudZsyYEexNijYI3r2INHrBKyt2GSyxChxd0RU2+La3Yiu9QvByn\/bt2wfgP\/jmp\/Tgm58Ef+dLJ14ZLrfpztrb1Igc84Um69evD34rhuD1g3hw5gdPIkrw5RdfHC04c5szXoALr\/IuWrSIJk6ciBXez2lr1IJXCNhu3bqVXNkVKV7sdAexytu3b9+iR5PF1dIM\/\/07dN\/C9wPzuHTCrYkEX925xYdMNOBMBiV3+oAvd7iQjQScySLlRj\/U8NbnodEKXiF2e\/fuXfLc3TBcKlZ4w18thC+duPy0crr78ko33hJEgZvWPMwBfBj7RRr48osvjhac+cUZBC8Er\/QlE9HUVlHDGxa8Xce8THxSAzecwevWRIKJ3S0+ZKIBZzIoudMHfLnDhWwk4EwWKTf6QfBC8AY3t82dO7doRooa3WLn8kbHy6wSxyVe+NIJXt3lVV40NxDAxO4GD2miAGdp0LLfF3zZ5yBtBOAsLWJ2+0PwQvBaycBo4uEMXis0SDvFxC4NlTMdwZkzVEgFAr6kYHKqEzhzio7EYCB4IXgTk0RHhyTBu2z0mcFJDWhuIICJ3Q0e0kQBztKgZb8v+LLPQdoIwFlaxOz2h+CF4LWSgdHEW7Byc71LJz6a8E0rccFpPAKY2P3LDHDmF2fgyy++OFpw5hdnELwQvFYyNpp4Y+etorHzVgex8Mour\/CiuYMAJnZ3uJCNBJzJIuVGP\/DlBg9pogBnadCy3xeCF4LXShZC8FqBPbNTTOyZobM2EJxZgz6TY\/CVCTarg8CZVfhTO4fgheBNnTQqBkQTL3wGLy6dUIGwWhuY2NXiacIaODOBsjof4EsdlqYsgTNTSKvxA8ELwasmk1JaKSV4celESjANdMfEbgBkxS7AmWJANZsDX5oB1mAenGkAVaNJCF4IXo3pVdx0NPFw6YQVGqSdYmKXhsqZjuDMGSqkAgFfUjA51QmcOUVHYjAQvBC8iUmio0M08XDphA6U1dnExK4OS1OWwJkppNX4AV9qcDRpBZyZRDu\/LwheCN78WZTBQjjx\/n1QG+IVXtEer66iszq3ymAVQ3QhgIldF7L67IIzfdjqsAy+dKCq1yY404uvausQvBC8qnNKyh4ErxRMznTCxO4MFdKBgDNpqJzoCL6coCFVEOAsFVzWO0PwQvBaScJSgheXTlihpKRTTOzucZIUEThLQsitn4Mvt\/iQiQacyaDkTh8IXgheK9kYTrwX1zalobNXBHHg0gkrdCQ6xcSeCJFzHcCZc5Tgl0q\/KEmMFu9YIkROdYDgheC1kpDhxHvwzU9wy5oVFuSdYmKXx8qVnuDMFSbk4gBfcji51AucucRGciwQvBC8yVmioUc48X754jaa\/eq6wAsundAAtgKTmNgVgGjYBDgzDHhOd+ArJ4AWhoMzcq28WQAAHSNJREFUC6DncAnBC8GbI32yDw0n3rWPbaCF726G4M0Op\/aRmNi1Q6zcAThTDqlWg+BLK7xajIMzLbBqMwrBC8GrLblKGS4meEf26kAje3W0EhOcFkcAE7t\/2QHO\/OIMfPnFF0cLzvziDIIXgtdKxoYT74Lp\/6R\/fPRxEMfdl1cSXy2M5hYCmNjd4kMmGnAmg5I7fcCXO1zIRgLOZJFyox8ELwSvlUwMJ95JE\/5WiAGXTlihI9EpJvZEiJzrAM6co6RkQODLL76wwusfXxC8ELxWslYk3gNz5xOv8IoGwWuFjkSn+DBOhMi5DuDMOUogeP2iJDFavGOJEDnVAYIXgtdKQorE++XvHifetCbastFnBmfxormFACZ2t\/iQiQacyaDkTh\/w5Q4XspGAM1mk3OgHwQvBayUTiwle3LJmhY5Ep5jYEyFyrgM4c44SrPD6RUlitHjHEiFyqgMELwSvlYSMK2nALWtWqJByioldCianOoEzp+hIDAZ8JULkXAdw5hwlJQOC4IXgtZKxIvG++5OZxDetcYPgtUKFlFNM7FIwOdUJnDlFR2Iw4CsRIuc6gDPnKIHgTUFJkz179uxJ0R9dMyIgBO+J195LL67dP7CCW9YygmlgGCZ2AyArdgHOFAOq2Rz40gywBvPgTAOoGk1ihRcrvBrTq7hpkXit+k6k1R+XQfBaYUHeKSZ2eaxc6QnOXGFCLg7wJYeTS73AmUtsJMcCwQvBGyCwfft2GjRoEC1evLiAyPDhw6m6ujoxi+bPn09Dhgwp9OvWrRtNmzaNysr2Ctm4Fid4cctaItTWOmBitwZ9ZsfgLDN0VgaCLyuw53IKznLBZ3wwBC8EL61bt4769OlD7dq1KwjV5cuXU\/\/+\/alHjx40YcKEook5adIkGj9+PE2ePJl69uwZa6uU4N3acyz9+6A2QRcIXuPvv7RDTOzSUDnTEZw5Q4VUIOBLCianOoEzp+hIDAaCF4KXeIV2xIgRNHPmTOrSpUsBERazc+bMoZqaGiov3\/e6XyGU+\/btW28luJi9MNQi8Tb3\/m3hn3HpROL7aq0DJnZr0Gd2DM4yQ2dlIPiyAnsup+AsF3zGB0PwQvCWXL2dMmXKPkJYDGBhO2bMmKKCuFQ2c+L1HfRfxCu8okHwGn\/\/pR1iYpeGypmO4MwZKqQCAV9SMDnVCZw5RUdiMBC8ELxFk2TYsGG0ZMmSooKWV4Bra2vptttuo6uuuorWrFkT2Ordu3fJMgjuA8Gb+G461QETu1N0SAUDzqRgcqYT+HKGCulAwJk0VE50hOCF4I1NRLERrdTGNRbEc+fOpRYtWhRWgePqgeMccOL1+a9baPtZNxZ+\/ORVR1G3r3Vw4sVAEPURwMTuX0aAM784A19+8cXRgjO3OWM9Em68KNevX79goa6iosLt4A1Eh3N4iUhsWKusrCx52oIQvGLDmuBHiOXov4f5ixO8reb+MNgoN2DAAANUw0UaBHbt2kUbNmwIarmbNm2aZij6WkIAnFkCPqNb8JUROIvDwJlF8CVcz5gxI1iMizYI3r2INHrBKyt2GSwWvM8\/\/\/w+Nb7FNrNFBe8ltz5EH3\/1ouCf+Za1yec1CQRV3AY5idxGF40I7Ny5k9avXx\/8VgzBqxFohabBmUIwDZgCXwZAVuwCnCkGVLE51iLhVd5FixbRxIkTscL7Oc6NWvCKlVmZc3QZL67hjdvUllXwLht9puJ0hzlVCOCrO1VImrMDzsxhrcIT+FKBolkb4Mws3nm9oYa3PoKNVvAKsSuz4UxAJlaDBw8enOlYst7jn6PdX+4emMO1wnlfZb3jMbHrxVeHdXCmA1V9NsGXPmx1WQZnupDVYxeCF4K3ULObdMlEXApGyxrE6u6pp55a8qQGTryLJi2jT9scF5i9\/LRyuvvySj1ZDqu5EcDEnhtC4wbAmXHIczkEX7ngszIYnFmBPbNTCF4I3qAWl09bKNbE5rNiJQzitjUxXmaVOCp4ccta5nfYyEBM7EZgVuoEnCmFU7sx8KUdYuUOwJlySLUahOCF4NWaYMWMc+JdMP2fuFbYCvrpnWJiT4+Z7RHgzDYD6fyDr3R4udAbnLnAgnwMELwQvPLZorAnJ975NTsLFrmcgcsa0NxEABO7m7yUigqc+cUZ+PKLL44WnPnFGQQvBK+VjI0KXj6hgY8mQ3MTAUzsbvICwesfL8UixjvmH5fgzC\/OIHgheK1kbFTwPl5dRWd1bmUlFjhNRgATezJGrvUAZ64xUjoe8OUXX1jh9Y8vCF4IXitZixVeK7BndooP48zQWRsIzqxBn8kx+MoEm9VB4Mwq\/KmdQ\/BC8KZOGhUDooL3ownfVGEWNjQhgIldE7AazYIzjeBqMA2+NICq2SQ40wywYvMQvBC8ilNKzlxY8HLtLm5Zk8PNVi9M7LaQz+4XnGXHzsZI8GUD9Xw+wVk+\/EyPhuCF4DWdc4E\/CF4rsGd2iok9M3TWBoIza9Bncgy+MsFmdRA4swp\/aucQvBC8qZNGxYCw4MW1wioQ1WsDE7tefHVYB2c6UNVnE3zpw1aXZXCmC1k9diF4IXj1ZFaCVQheK7BndoqJPTN01gaCM2vQZ3IMvjLBZnUQOLMKf2rnELwQvKmTRsWAsODlCyf44gk0dxHAxO4uN8UiA2d+cQa+\/OKLowVnfnEGwQvBayVjw4J3ZK8ONLJXRytxwKkcApjY5XByqRc4c4mN5FjAVzJGrvUAZ64xUjoeCF4IXisZC8FrBfbMTjGxZ4bO2kBwZg36TI7BVybYrA4CZ1bhT+0cgheCN3XSqBgQFrxczsBlDWjuIoCJ3V1uikUGzvziDHz5xRdHC8784gyCF4LXSsaGBS+uFbZCQSqnmNhTweVEZ3DmBA3SQYAvaaic6QjOnKFCKhAIXgheqURR3QmCVzWieu1hYteLrw7r4EwHqvpsgi992OqyDM50IavHLgQvBK+ezEqwGha8fMsa37aG5i4CmNjd5aZYZODML87Al198cbTgzC\/OIHgheK1kbFjwfjThm1ZigFN5BDCxy2PlSk9w5goTcnGALzmcXOoFzlxiIzkWCF4I3uQs0dBDCF5e2eUVXjS3EcDE7jY\/cdGBM784A19+8YUVXv\/4guCF4LWStRC8VmDP7BQfxpmhszYQnFmDPpNj8JUJNquDwJlV+FM7h+CF4E2dNCoGCMHbvVMremJolQqTsKERAUzsGsHVZBqcaQJWk1nwpQlYjWbBmUZwNZiG4IXg1ZBWySY58S6Y\/k8688TOELzJcFnvgYndOgWpAwBnqSGzOgB8WYU\/k3Nwlgk2a4MgeCF4rSQfEs8K7JmdYmLPDJ21geDMGvSZHIOvTLBZHQTOrMKf2jl0BwRv6qRRMQCJpwJFczYwsZvDWpUncKYKSTN2wJcZnFV6AWcq0dRvC7oDgld\/lsV4QOJZgT2zU0zsmaGzNhCcWYM+k2PwlQk2q4PAmVX4UzuH7oDgTZ00KgYg8VSgaM4GJnZzWKvyBM5UIWnGDvgyg7NKL+BMJZr6bUF3QPAGCGzfvp0GDRpEixcvLiAyfPhwqq6uTpWFkyZNojlz5lBNTQ2Vl5cXHYvESwWr9c6rV6+m6dOnBzlSUVFhPR4EkIwAOEvGyKUe4MslNuRiAWdyOLnSC7oDgpfWrVtHffr0oXbt2tG0adOorKyMli9fTv3796cePXrQhAkTpPJVjGnZsiUErxRi\/nTCROEPVyJScOYXZ+DLL744WnDmF2fgC4KX5s+fTyNGjKCZM2dSly5dCojIrtZGV4h5BRArvH5NBEnRYqJIQsi9n4Mz9zgpFRH48osvCF7w5R8CELxFOWPBO2XKlH2EcNwA7ltbW0tVVVX01FNPQfD6\/iZE4seHsX+EgjO\/OANffvEFwQu+\/EMAgrcoZ8OGDaMlS5YkitfwCvHChQtT1fDOmjULNaEevDVr1qyhfv36EfjygKzPQwRn\/nDFkYIvv\/gCZ\/7yxYtz2ItC1GTPnj17\/KNRfcQsYocMGUJJG9dE\/W\/fvn2DDW6yZRA8uXMZxaJFi9QHD4tAAAgAASAABIAAEIggcPrpp9Ps2bOBC0HwBkkgNp9VVlYWNrEVyw5eBa6rqyv0kxW84rdjFr5oQAAIAAEgAASAABDQjQCv7GJ1dy\/KjX6FN43Yjdvslkbw6k5s2AcCQAAIAAEgAASAABDYF4FGLXhFGUO3bt0SV3YZOl7dnTt3btE8SiqHQAICASAABIAAEAACQAAImEeg0QpeIXZ79+4tfe5uHD1Y4TWftPAIBIAAEAACQAAIAIE0CDRKwZvlkolioELwpkk39AUCQAAIAAEgAASAgHkEGqXgTSpNmDx5MvXs2TM4gSHpXF4IXvNJC49AAAgAASAABIAAEEiDQKMUvGkAQl8gAASAABAAAkAACAABvxGA4PWbP0QPBIAAEAACQAAIAAEgkIAABC9SBAgAASAABIAAEAACQKBBIwDB26DpxcMBASAABIAAEAACQAAIQPBqzoHt27fToEGDaPHixYEnvvGkpqaGysvLNXuG+SQEwpsXW7RoQTNnzqQuXbqUHCaOsxOdwGcSyup+noWvsPfoteDqIoOlYghk4Sw6Z7JtsZEYSOtFIAtf4r0St4hiTtTLURbrzCu3CRMmZBneYMZA8GqkUkzc7dq1KyQaJ96SJUsgejXiLmM67opomRM5xo8fX+\/Dl+08\/\/zzUmJZJi70iUcgC19RS+LDHBfEmMmyLJwJ8cRz5rRp06isrCw4LSf63pl5gsblJQ9fp556Kj7jHE0X8f7kvXPA0cdLFRYEbyq40nWOO9YMq0zpMNTRW6zShleN4n45Cfsu9nPwqYOh+jaz8BWNKrwyD8HrLmdxxzwmvZv6n6bhe8j6jsV9xolz7gcPHkzV1dUNHzxHnzD6TQkELxEEr8Zkjf7GLFwV+3eNocB0CIFiZydnOVNZCN7wCgfAVotAXr4ER1xaxKuGffv2xQexWor2sZaFM\/EBfc4554AfzfxEzWfhi21A8BomStKdeJfq6upo+vTpNGrUKAp\/0yxppsF1g+DVRGmpVQmUNWgCXdJssV84ZC4aibrAaoYk6Dm65eEr\/B7eeOON1KdPHwjeHFzIDs3CmfjFZPTo0bRy5cqgjIGbbH29bGzoty8CWfhiK3G\/8KPMy60MwzckX\/ABwaspN0slWZaVRE1hNkqzxSZ3\/lpvxIgR0vW44d+isRFRXyrl4Sv8rnGEELz6eApbzsKZ+OVx27ZtFP76FTW8+jnLwleU77lz5wb\/1K1bt0L9tf7I4SEJAQheCN6kHMn9cwje3BBqM5B3cheBiU1Q2EGujarAcFa+hIAaN25ccFU46q318qRK8FZWVtYTTGIuZftiI5u5J2kcnrK+Y6L2N1wXj19Q3MoZCF4IXu0ZiZIG7RBndpD167u4FQ2I3cw0SA\/Mwlfc+wfBKw157o5ZOBO\/oPTo0WOf45PwrVhuSkoayMJXqV9EsE9FL19prEPwQvCmyZfMfbFpLTN0Wgdm3aDBQYV3vkLsaqWpYDwLX+Gvx+OixFmhernLwlmpDaAQvO7xhW8x9XKiyjoELwSvqlwqaSduksYqkxHoSzqJq9WVmRREnxUrVkjX+dp\/Wv8jyMpX9Mnx7pnLhaycxW3olXk3zT1Zw\/SUhS+s8PqRC3h\/IHiNZGrcIeo4ocEI9CWdxG02kzmhAbuP7XCXlS8IXjt8hb8J4WORxIZOmXcsrqxBZpy9J20YnrO+Y6jhdZ9\/CF4IXmNZGj38GV+lGoM+0ZHYdMYd444+Cv9ysn79eurfvz\/xDvK4hp3JiXDn7pCGr7iru7HCm5uC1AaycBa9qhbHkqWGPfOALHxFy4fAV2b4tQyE4IXg1ZJYMAoEgAAQAAJAAAgAASDgHgI4h9c9ThAREAACQAAIAAEgAASAgEIEIHgVgglTQAAIAAEgAASAABAAAu4hAMHrHieICAgAASAABIAAEAACQEAhAhC8CsGEKSAABIAAEAACQAAIAAH3EIDgdY8TRAQEgAAQAAJAAAgAASCgEAEIXoVgwhQQAAJAAAgAASAABICAewhA8LrHCSICAkAACAABIAAEgAAQUIgABK9CMGEKCAABIAAEgAAQAAJAwD0EIHjd4wQRAQEgAASAABAAAkAACChEAIJXIZgwBQSAQHoEPvvsM3rsscdo586d9P3vfz+1gffff5+mTZtG1dXV1LZt29Tj8wx47rnn6Oc\/\/znV1dVR+\/btac6cOcGfaZu4TvfUU0+lCRMmpB3uXP\/o9cDDhw8P+LHRolffTp48mXr27GkjFPgEAkDAIgIQvBbBh2sgAASIhDjq27dvJlE0adKkQGjW1NRQeXm5MUi3bt1KV199dSB2hwwZQkcccQR1796dmjdvnjqGhip4jzzySOrfvz8dc8wxdOyxx6bGRcWATZs20aJFi+jPf\/5z8IsRBK8KVGEDCPiHAASvf5whYiDQoBDwVfCqFKkqbbmQHC4+z\/z584NfTCB4XcgQxAAEzCMAwWsec3gEAo0GgT179tATTzxBt99+O61Zs4YOOOAA6tq1K91yyy103HHHUfTr5hYtWtDMmTOpS5cutH37drrrrrto7ty59MEHH1CTJk2ooqKCbrjhBrrwwguD\/x82bFjwc9F69+5dKAl4++236Wc\/+1mwssftpJNOohtvvJFOP\/30RPy5TOLOO++kJ598knbs2EHt2rWjH\/7wh0HJRbNmzUiIp7ChUl\/bMw7PPPMM\/frXv6b33nsvwOHMM8+k0aNHU6dOnQqr3FVVVdStWzf6zW9+Qxs2bAj8csziednf7t276cEHH6Tp06cHmLLtww8\/nAYOHEhXXXVVEB83sfL9gx\/8IPD773\/\/m2699Va65JJL6N133w3+\/tJLL9H+++9Pl19+OZ144okBLwJ\/tsFlJuznvvvuow8\/\/JAOO+ww+s\/\/\/E+67rrrqKysrCiOcYKX+Rw0aFDwTOeeey794he\/oI0bNwZ5MGrUqGB1nDkt1oqt5Muu8EPwJqY9OgCBBo0ABG+DphcPBwTsIvDUU08FAvUb3\/gG9erVizZv3ky\/\/e1vA2Eza9YsYoHLQmT8+PH09a9\/nb7zne8EgvTggw8OxOwf\/\/hH+t73vkf\/r73zCamiC8P4oY3RQjLa9k8QUloZQgsLrNxYIC5UkMKdERHpQrKVi4IiN7WKchFRIIEluBBEVChFMaVwkWBCoXshxIXo4uP3wns5zTcznuvfe\/U9EEHOnXPOMzf5zTPP+05FRYX7\/fu36+npEfB6\/fq1nHNmZkYADXBrbW1158+fdxcvXpRz8vmzZ8+6W7duiQgfPnxwCwsLAsQ1NTWJwgCRgCPAyd9nzpwRqB4fH3cNDQ0ChYDa6Oioe\/HihSsuLk59bA+Q4iqyx9LSUlkPQMh6Tpw44d6+fSvQybn5d2IAzc3NrqCgQECTdaAZGnCurq4u193dLRB85coV9\/fvXzkXIN3Z2SlrUeDlRqOoqMi1tLTIHFevXhXYZl9cC9ZSWFgoUAvcAtP+DQdgOz09nbkG3759c729vXLTAmjy2biRBrxzc3Oyhvr6etGW+Vg7kJ+WrTXg3d\/\/yza7KZDvChjw5vsVtPWbAjmsANCJWwqgKRyNjY0JjD558kQAJy7SgDtLkRNuol\/sBHDhtPJoWv89CkJkawE68rwAn2ZqgTnmJXML4MXBGkAJNFJEB2wCzwwK6wBWnFWFz9DH9ouLi66pqUkgEdhWB3ZyctLdv3\/fPXr0SGAf4MWFff\/+vTt37pzMq\/slK8yxgPadO3ckD4tDCzgydA7cYS16QxfW\/PTpU9fY2CjH6f5wm9+8eeNwlBk+5CvwAvk45JynsrIy8y1j3QA08+OoZwu8P378+Oemg4wt7jQ3QUnXRQE+LqttDm8O\/wKwpZkCOaSAAW8OXQxbiilw0BQAOIFdgA3nMa6LQjYZXj32xo0b7uHDhyJXFHi+f\/8uDunNmzflMbk\/cH5xjf3H9v7PAcrbt28LcL58+TIDlByDM8kecJyZOxR46eQAnBPPSHIw9VwAKK6xPtoPnUPjAjjGum50wQn395q2PyIcFHVxPPEPHHMcdFxenHgdOMOs8fLly4kdJdIcXs7DPH4kgrXyPQH2L1y4EPvfwBzeg\/bbwfZjCuytAga8e6u3zWYKHCoFcHcVnNg4WVNiC4CjuphpwLu2tiaPu4HNqakphzuMQ+tndaMgFJev9UUHJgHB69ev\/+9axAG1HhSFuFAYBeSA3STI5vxJ50r6d9xqtP3586dA6devX0UnYg8Kk3HAy2dwe\/mDYxy9GWhvb5d1kismb4vmSQNXGh2Jn0THZhneaOu1kHytAe+h+tVhmzUFdlwBA94dl9ROaAqYAr4CPEYHxj5+\/OgGBwczxWuapY0DXoCOx\/HkW4kTED+gvy3FbENDQ\/+4i0nAu5Vq\/N0A3jjwDAHEOBBGS4CUHC\/FdMeOHZPML\/17gVMc9DTg\/fPnjxTe0QIuBHjj3NiQb\/dWgPfu3buJNyLMacAborwdYwqYAkkKGPDad8MUMAX2VIH5+XnJ4FKwBJSS4SS\/6vfhHRkZcQAQ2c579+5lHn8rsFHY5mdV\/Wwn4EeGl1iDxh5CNxgSacCdBhZDHd6kSAPFaMQnqqurBULRIPriiegcGqtg\/48fP5aCNIbmYCl0SwPetP35YF5WVuYePHggXTTILVP8l81IA14y1Vx31qqDubm5Ya6kfr1Jx9DFglzxZn2YQ1zkbPZox5oCpkB+KWDAm1\/Xy1ZrCuSNAisrK66trU0cWmBFi8eWl5cFRk+ePJkIvBwPFL179y5TWIW7CdTQwqq2tjYReLVojawpnz916pRohmvMZ2lTRuGZRip8QTcrWgPKKGbjcX4o8CYVrfX390vLMboTkJkNAV6FNvK25Jh1EPUgJwyopgFvUtGaAjNOvF+0RocNIinkeDVXTISCwjlamfHzuLFZlwa\/IJCWc3wfiLtwzZNe3EEnCiD\/1atX0m2CsbS0JJ\/d2Ngw4M2b3wy2UFNgfxQw4N0f3W1WU+BQKEAnAKIJdDsAUhl9fX3iHGobKnUd+RmuLsVQs7OzAlXEGHB5gSA6J+De0skA2FOHV0EI8KH1GXMNDAxIRwYKozgPRVd8HlgDwnGYk3q+hrQlo9NCKPD6bclwZil6+\/XrlxRolZeXy80ARWchwIvDyz7RgH0RYfjy5Yu0YVtfX5ebgzTgRWNccpxl2pDRloxzAKDs++jRoxng5cYBiJ6YmBDARHMyw58\/f3bHjx+Xjgr0Rc4WeLmGFNf5czOX3zVCwd7vbax7pzMFnToY3NCwFoBdHV69LvT79YvjzOE9FL9ybJOmQKICBrz25TAFTIFdUwB3l7gBMANQMaIvGgAIgT\/AeHV1VXKc165dkxdWPH\/+XIrUyKrSdxcgpvMDvWe1hRWFWMAt\/WJ5mYMWUvE6WT4PPAOIwBmP6QFvetKmjbgXTxBjqKury7QVCwVe5om+eAKAB3AVyrMpWqPnMC3daN2mL\/Kg2AzHeHh4WPobnz59WkA62qVB98yLJ9Tt1hdPsCYiBX5xnb7849OnT9L\/mOtQVVXlOjo65GYkaaQ5vGjBtcQtx4UH+mlxxvdCRxzw8jN\/70Az3T+4rs+ePTPg3bX\/xXZiU+BgKGDAezCuo+3CFDAFTIFtKUBMgsJC\/qTBbMgk2bYlCznndo8xh3e7CtrnTYH8VsCAN7+vn63eFDAFTIFgBWjzRh73yJEj4pRrL1wiBbilxDyiPXKDT+4daMC7FdXsM6aAKbCbChjw7qa6dm5TwBQwBXJMASILRCIuXbok\/Ywp5gt97XLoVhR4aZlGV4uSkhJHppbevoydgOrQtZDvJd5CsSLzbqVdXehcdpwpYArkrgIGvLl7bWxlpoApYArsuALkqing441s5Kpxe+kSQQ6YorqkYr5sFqLAq7ltis8A3\/0AXgokmZuuIQwD3myupB1rChwcBf4DO29ZKsvizoQAAAAASUVORK5CYII=","height":337,"width":560}}
%---
%[output:6f3c261a]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"    44"}}
%---
%[output:7d5e9062]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"     3.000000000000000e-02"}}
%---
%[output:71a97270]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_JH","value":"     7.500000000000001e-02"}}
%---
%[output:5cd9d49f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber","value":"     6.215564738292011e-09"}}
%---
%[output:60ed39a0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rsnubber","value":"     3.217728531855956e+03"}}
%---
