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
system_identification_enable = 1;
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAnkAAAF9CAYAAAB4cH4qAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ1wVtd55590tbs0ljLYk25kSXiconRMm0Z2E6C1NXRbdjAlSUdr1zJosqIDmhgrqh0rIp5lNdhhMDNGQo0msoobwRh2FjCJs9om9thMGW+1wbZEujbOsjQboaiJIqCkMYnUYKbezc5z8Hl9dPV+3HPvPfeec8\/\/znisV+\/5ur\/\/c3X+nK\/7vl\/+8pe\/JFwgAAIgAAIgAAIgAAK5IvA+mLxc6YmbAQEQAAEQAAEQAAFBACYPgQACIAACIAACIAACOSQAk5dDUXFLIAACIAACIAACIACThxgAARAAARAAARAAgRwSgMnLoai4JRAAARAAARAAARCAyUMMgAAIgAAIgAAIgEAOCcDk5VBU3BIIgAAIgAAIgAAIwOQhBkAABEAABEAABEAghwRg8nIoKm4JBEAABEAABEAABGDyEAMgAAIgAAIgAAIgkEMCMHk5FBW3BAIgAAIgAAIgAAIweYgBEAABEAABEAABEMghAZi8HIqKWwIBEAABEAABEAABmDzEAAiAAAiAAAiAAAjkkABMXg5FxS2BgO0EhoeHqb+\/f0Eza2pq6PDhw9TU1KTd\/IsXL1Jrayv19vbSunXrtPJ3d3fT6Oho2bacOHGCdu\/eTcePH6fa2lqt8pEYBEAABLIiAJOXFXnUCwIeE2CTNzY2RiMjI1RdXS1I8O+OHTsWyUjFNXlc\/8DAQEERbsvTTz9dMJ0weR4HK24dBBwmAJPnsHhoOgi4SqCYyTtz5gx1dXXR0NCQ9mhe0iaPufIIX9D8ucob7QYBEPCTAEyen7rjrkEgUwKlRvKCo3vqVGpLS8uC0TYeXdu2bZu4j1WrVtHs7GxhulaavpmZGfH9\/v37S07jljJzahtfeeWVBdO1at1cfk9PD3V2doq65ufnqaOjgyYmJoinoLltc3NzYtSSyxkcHBTpuG08Pc1Xe3u7SMOXep\/cNv49\/8flNTQ00Fe+8hX6sz\/7M5GfP2MKOdNQRuUgYDUBmDyr5UHjQCCfBIqtyQsaFtVksXHiNXcbN24UZio4cifLYzN35513CpO1Zs0akbbSCGEpk6dO0b755psFk8eKbN26lfbs2SNGHINTucXaXVdXVzB5bEyl6ZSGcMuWLcKEBtvKbTt58qQwg8uXLxf3xWaWjR1fKpN8RgruCgRAIA4BmLw49JAXBEAgEoFiI3lslrZv377A0EijxpWUG1lTTd+HPvQh2rFjBx04cKCwSYLNUmNjY2G0TW20rskLbrxQ1xLy+kLVYFZqdxCeyoANZLBt6mdpEFVGkcRAJhAAgdwSgMnLrbS4MRCwl0Axk6ealnvuuWfRblk1D49sqVO76ogY37WcxlUJBKd75Xe607XSyPH0KV+33XabmKJVR9fUXb7lzCnnV6ek6+vrRZlyXSJMnr0xjJaBgAsEYPJcUAltBIGcEahk8niNms6IWKWRvHL4ipk8aRp5mpV33ZaauuVRPfU73ZG84PRsselabrvc+YuRvJw9CLgdEDBMACbPMGAUDwIgsJhApelanqoMsyZPrtGTGyGKrcmTBlCmDbammMkrd4QKr88LTivLdXJs+iqtyVPP22NTx4a2r69PrMlT1+BhuhZPDgiAQFwCMHlxCSI\/CICANoFiGy+4kOAu2HK7a6VB4p2nK1asEG14+OGHhVkK7q4tNVXLeYodhhzcBFJstE7unv3CF75AX\/3qVwtTrOruWi7nD\/7gD+h73\/teYeNF8FBltf7HH39cpJPTvZiu1Q4tZAABEFAIwOQhHEAABEDAIAE2tJOTkwuOfzFYHYoGARAAgQIBmDwEAwiAAAgkSEDdyVtpqjjBalEUCIAACCwiAJOHoAABEACBBAmo08hcbLmp4gSrRVEgAAIgAJOHGAABEAABEAABEAABHwhgJM8HlXGPIAACIAACIAAC3hGAyfNOctwwCIAACIAACICADwRg8nxQGfcIAiAAAiAAAiDgHQGYPO8kxw2DAAiAAAiAAAj4QAAmzweVcY8gAAIgAAIgAALeEYDJ805y3DAIgAAIgAAIgIAPBGDyfFAZ9wgCIAACIAACIOAdAZg87yTHDYMACIAACIAACPhAACbPB5VxjyAAAiAAAiAAAt4RgMnzTnLcMAiAAAiAAAiAgA8EYPJ8UBn3CAIgAAIgAAIg4B0BmDzvJMcNgwAIgAAIgAAI+EAAJs8HlXGPIAACIAACIAAC3hGAyfNOctwwCIAACIAACICADwRg8nxQGfcIAiAAAiAAAiDgHQGYPO8kxw2DAAiAAAiAAAj4QAAmzweVcY8gAAIgAAIgAALeEYDJ805y3DAIgAAIgAAIgIAPBGDyfFAZ9wgCIAACIAACIOAdAZg87yTHDYMACIAACIAACPhAACbPB5VxjyAAAiAAAiAAAt4RgMnzTnLcMAiAAAiAAAiAgA8EYPJ8UBn3CAIgAAIgAAIg4B0BmDzvJMcNgwAIgAAIgAAI+EAAJs8HlXGPIAACIAACIAAC3hGAyfNOctwwCIAACIAACICADwRg8nxQGfcIAiAAAiAAAiDgHQGYPO8kxw2DAAiAAAiAAAj4QAAmzweVcY8gAAIgAAIgAALeEYDJ805y3DAIgAAIgAAIgIAPBGDyfFAZ9wgCIAACIAACIOAdAZg87yTHDYMACIAACIAACPhAACYvAZVnZmYSKAVFgAAIgAAIgAAImCTQ0NBgsnjryobJiykJG7zt27fT+Ph4zJKQHQRAAARAwGcCK1asoNraWnr77bfp1Vdf9RmFsXtfvXo19fX1kS9mDyYvZii99tpr1NbWJoKmvr4+ZmnRsrPBHBwcTKQNUcvSyVcpbdTvS+Ur9vtKdURTInyupOuPUp5Onkppo37vkmasbqX7DB8B0crSqT9M2nJpdLQpxSZMG3SYRUmbZBuilhU23+nTp+mNN96glStX0u233170dqGZXhSovHhQhvvKsbExmDw9jP6mliYvy6DhwH3uuefo4Ycfji1E1LJ08lVKG\/X7UvmK\/T5r3Srdo66QUcrTyVMpbdTvXdKMNal0nzq6RSlLJ0+YtOXS6GhTik3Wz5kNmum04eWXX6ZTp07RRz\/6UWppaSkaTtBM5ylb+MzaEI96rY+fGiN5MRn6GDQxkVmRHbpZIYNWI6CZFi4rEkMzPRl4inZqaopuuOGGkiZPr0T91HnWLM\/3VkppmDz9Z2BBDh+DJiYyK7JPT0\/TM888Q729vVRVVWVFm9CI8gSgmXsRAs30NLPB5OVZMx\/7a5g8vWdwUWofgyYmMiuy88LmCxcu0LJly2DyrFCkciOgWWVGtqWAZnqK2GDy8qyZj\/01TJ7eM1jS5H1gy3+JWRKyp03gnXfecc7gLbtxSSKYbrkpXDmV0t1y068uak+xNqrlVCqz1A3mufNJRFQLC4FmeqLA5Onx0k0Nk6dLDOlJBs0n\/9NhamjIZnctZNAnwAZvbm6eamqqnTN6P\/zp2\/o3HMihU8aP3ipen04ZpRocNHzSIMrfq9\/XfaCK3nrrCn3s128WmvF3UQ1jbIAoIBQBmLxQmAqJYPL0eOmmhsnTJYb0BZOX5e5ayKBPAJ2PPrOwOYqZP\/V3qmn84U+vLihWpiuWvpyplGaPTaJq\/nikURrH5salYW8B6RIigOdMDyRMnh4v3dQwebrEkB4mz9EYQOfjnnBSs1\/e8GtiJE+aPjaN0izy7+TvT52\/sugmi5lBaQRhApOPCTxnekxh8vR46aaGydMlhvQweY7GADof94SLqplq+oJmMGgE2QSqo4F3Lb+RYP6ix0pUzaLX6HZOmDyz+sHkmeWby9J9DJo8CInOxz0VTWkWNIFyNFA1gNL8ScMH8xcufkxpFq5291LB5JnVzMf+GrtrQ8TU8PAw9ff3i5Q9PT3U2dlZyOVj0IRAZn0SdD7WS7SogVlo9p7he0tMA\/N\/QfO3aWUt8ZQv\/x\/XQgJZaOayBjB5ZtXzsb+GyasQU2fOnKGuri4aGhoSKeXPTU1N4rOPQWP2MUyndHQ+6XBOshabNGOzd\/T0BXF7R09fLKwD5BG\/u5YvFZs\/Nq282fvdvzZplmQsmioLJs8U2evl+thfw+RViCkexeOdsyMjI1RdXU3d3d3U2NhYGM3zMWjMPobplI7OJx3OSdZis2bqiN+3J68URvuum73rI3yP3v3hJHE4UZbNmtkIECbPrCo+9tdOm7z5+Xnq6OigiYmJRZGxatWqgjGLEzZs6vgaGBgQ\/w9+9jFo4vC0JS86H1uUCN8O1zSTo33FTJ8va\/pc0yx8NJpJCZNnhqss1cf+2kmTx1Oo7e3tQrfDhw+TnDpVw+PEiRO0bds2qqmpKZkmTDgFR+54ZG9ycrJg+mTQHDlyhBoaGgpF1tZifU4YvlmlQeeTFfno9bqsmTq9u+\/kTGEal0f52PD97q3V0cFYnNNlzbLAygMWU1NTtGTJEmppacmiCZQnzS5evLiA4czMDLW1tYnZObW\/zgR0SpU6Z\/JYtL1799KuXbvE9Gmli0f7du7cWTBlldIHvw9r8oL52IRu3rxZtzqkT4nAtWvX6PLly8RmnM9cw2U\/gTxpNvvzd+ib5+bpW383T\/wzv83jU7dVi\/9\/ekXlv2v2q3W9hXnSLA3mZ8+eFe\/UZpPX3NycRpWL6siTZocOHRKDPMELJi+T0LKz0rDTtX19fVRf\/95rzdg8YDTPTk25VVevXqVLly6Jf83B5Nmrk9qyvGrGJu9rr18ubOBgo3ffHR8UGziaG290Q5wSrcyrZqZE4ZG86elpMYCxYcMGU9WULTdPmvGgkDqaNz4+ToODgxjJyySyNCoNrsXbv3+\/yM3Ts3wltR6PywpOz2LjhYZQFifN05SExZgTbZoPmslpXbljV27ccHWnrg+aJRnkWJOXJM3FZWFNnlm+iZWujq7JtXe8foE3R0gDWFdXF3mKVm0ojlBJTDarCkLnY5UcoRrjm2bS8D350rTgIw2fS7t0fdMsVCCXSQSTF5dg+fwweWb5JlI6D71u3bqV9uzZIzZcSFO3Zs2awrEmbMx27NhBBw4cSGTKFIchJyKdVYWg87FKjlCN8VkzHtn79uRbYkrXpdE9nzULFdSBRDB5UaiFzwOTF55VZilLmbwtW7bQunXrRLuSNnnlbtbHoMlM\/AQrRueTIMyUioJmJA5d5kOYXRndg2Z6DwdMnh4v3dQ+9tdO7q4tNpIHk6cb7n6nR+fjnv7QbKFmfP4eGz51dM+2qVxopvecweTp8dJNDZOnSyyD9BjJywB6DqtE5+OeqNCsuGbB0b1H777VmleqQTO95wwmT4+XbmqYPF1iGaRnk9fa2kp8qGG5i4\/GOH78eCJr8jBdm4HQhqtE52MYsIHioVllqE++9IPCUSy8do9H9uRr1SrnTj4FNNNjCpOnx0s3NUyeLjGk9\/KFx3mQHZ2PeypCs\/Ca8RQuGz4e5ctyVy40C68Zp4TJ0+OlmxomT5cY0sPkORoD6HzcEw6a6WvGJk+O7mVh9qCZnmYweXq8dFPD5OkSyyA9pmszgJ7DKtH5uCcqNIuumbpuL02zB830NIPJ0+OlmxomT5eYBemDb6DgJvG5dvxuupGRkVDvt41zGz4GTRxetuRF52OLEuHbAc3CsyqVMm2zB830NIPJ0+Olm9rH\/tq5I1RUUYM7beV3OCdPN\/T9S4\/Oxz3NoVlymqVl9qCZnmYweXq8dFPD5OkSsyA9j+SNjo4Sv7+WD0MOvubMdBN9DBrTTNMoH51PGpSTrQOaJcuTSzNt9qCZnmYweXq8dFP72F87PZKnjty1t7fT3Nwc1dTU0OHDh8Urz9K4fAyaNLiargOdj2nCyZcPzZJnKks0ZfagmZ5mMHl6vHRT+9hf58Lk6QqdZHofgyZJflmVhc4nK\/LR64Vm0dmFzRk0e3HP2YNmYclfTweTp8dLN7WP\/TVMnm6UBNL7GDQxkVmRHZ2PFTJoNQKaaeGKlTh49MrQxhXU3LhUu0xopocMJk+Pl25qH\/tr50web7bYu3cv7dq1K9TO2fn5edq5cycNDAzoxkOo9D4GTSgwlidC52O5QEWaB83S14zN3ueOnqNT56\/QXcuX0lObVojDlcNe0CwsKYzk6ZGKltrH\/to5k8fS8u5ZXoPHV6n1d3IDhuk1ej4GTbTHy65c6Hzs0iNMa6BZGEpm0nx78gp1HTsnNmrwu3F5GjfMBc3CUHovDUby9Hjppvaxv3bS5ElheZSuo6ODJiYmFmm9atUqnJOn+wR4lB6dj3tiQ7PsNeO3Zzz50rRoSBizB830NIPJ0+OlmxomT5cY0uO1Zo7GADof94SDZvZoJs1epbdnQDM9zWDy9HjppobJ0yWG9DB5jsYAOh\/3hINmdmkW3IlbbHMGNNPTDCZPj5duapg8XWJID5PnaAyg83FPOGhmp2blNmdAMz3NYPL0eOmmhsnTJYb0MHmOxgA6H\/eEg2Z2a1ZscwY009MMJk+Pl25qmDxdYkgPk+doDKDzcU84aOaGZup6vfvu+CC1\/WYVLVu2jKqqqty4gQxbCZNnFj5Mnlm+uSzdx6DJg5AwDO6pCM3c0Uxdr1f3gSra\/5nfjnSYsjt3nExLYfKS4ViqFB\/7a6ePUGEh1WNU+NiUhx56iJ544gk6cOAA1dbWmo0YIozkGSdspgIYBjNcTZYKzUzSNVP2\/5m9Ig5T\/tsfvx3pMGUzrbK3VJg8s9rA5Jnlm3jp0uCtWbOGGhsb6eDBg+JsPD4geWxsDOfkJU48PwXCMLinJTRzV7O\/v1ZDn\/\/697UPU3bvjuO1GCYvHr9KuWHyKhGy7Ht+xdnWrVtpz549dOnSpYLJO3\/+PO3YsSP0aJ58g8bc3Jy4w5aWlgWvQRseHqb+\/n7xXU9PD3V2dhZI+Bg0loVBpObAMETClmkmaJYp\/kiVBzVT1+vxWzM2rTQ\/2xKp4RllgskzC97H\/tr56dru7m6anZ2l+++\/n5599lkxXfvggw\/S2rVrQ72vlo1ia2sr9fb20rp160h+3rhxozBzbAC7urpoaGhIRJ\/8uampSXz2MWjMPobplA7DkA7nJGuBZknSTKesYprFfR9uOi3PphaYPLPcfeyvnTd5HBLyPbUyPIKjbbphw8aRr4GBAeJRPHXql7\/jqWE5mudj0OjytDE9DIONqpRvEzTLl2ZR34frHoXwLYbJC88qSkof++tcmLwoYpfLo5o89WfOE\/zsY9AkzTuL8mAYsqAer05oFo9fFrnDaKZO4RZ7a0YW7c6qTpg8s+R97K9h8gIxJdfn9fX1ienb4Mgdj+xNTk4WpoJl0Bw5coQaGhoKpaWxs9fs45Dv0sN0Pvkm4N7dQbP8asZTuA9\/7ft06vwVsQt38L6PEL8X17drYmKCpqamaMmSJWJteBZXnp4zXn6lXjMzM9TW1iZm59T+OgvOadXpnMmTa+ZYrHIXC3j8+HGtY1Rk2Z\/4xCcKJi6syQu2pb29nTZv3pyWjqhHk8C1a9fo8uXLIj5wSKsmvIySQ7OMwMeoVlczPmrl8b\/+Cc3+\/B367Kql9MDqpTFqdy\/r2bNn6cKFC8LkNTc3Z3IDuppl0siQlR46dEicthG8YPJCArQhWdCEcZuC6+jUdnL60dFR8Ss+V4+PXKmuri5suFANHqcJO13LI3\/19fWFqtg8YDTPhggp3oarV6+KHdn8jwGYPHt1UlsGzdzQKQnNBv\/mAj350jTxQcpf\/pNGam680b2bj9BiHsmbnp4WfdKGDRsilBA\/S56eMx64UUfzxsfHaXBwECN58cMknRLUI1TkbleumadcdY5QCe6oVVsfnJ7Fxot0tDVdS56mJEyzsqV8aGaLEuHbEUczdRcuH7Xy1KYV4St2NCXW5JkVDmvyzPI1Urocmdu\/f79YQyd32gbPuitVuTxQua6uruiRKzhCxYhsmRcap\/PJvPGeNgCauSd8EpodPX1RvDWD1+ix2ePz9fJ6weSZVRYmzyxfY6WrhxnX1NSIOXh1ZK9cxcGDkGVadSoXhyEbky6zgpPofDJrvKcVQzP3hE9KM\/VduLwxg0f18rgxAybPbIzD5Jnlm8vSfQyaPAiZVOeTBxau3AM0c0Wp99qZtGZ5P1sPJs9sjPvYXzu3u9ZsCOiX7mPQ6FOyL0fSnY99d5i\/FkEz9zQ1pVlez9aDyTMb4z721zB5MWPKx6CJicyK7KY6HytuLqeNgGbuCWtSM3VjxqN335qLtXoweWZj3Mf+2mmTV+7MvCjn5EUJLx+DJgon2\/KY7Hxsu9e8tAeauadkGprlaVQPJs9sjPvYXztt8kqFA++4Xb9+vdhta\/ryMWhMM02j\/DQ6nzTuw6c6oJl7aqelWV6OW4HJMxvjPvbXuTR5uufkxQkrH4MmDi9b8qbV+dhyv3loBzRzT8W0NeONGX88\/LrYeevie3Bh8szGuI\/9dS5NHh95cuzYMe3XmkUJLx+DJgon2\/Kk3fnYdv8utgeauadaFpqpo3p83Mo3P3eHM+Bg8sxK5WN\/7bTJK7cmTx6ObDZkiHwMGtNM0yg\/i84njfvKcx3QzD11s9TMxVE9mDyzMe5jf+20yTMbDuFK9zFowpGxO1WWnY\/dZOxtHTSzV5tSLctas+AhyraP6sHkmY1xH\/trp01eUu+ujRNWPgZNHF625M2687GFg0vtgGYuqXW9rbZo5sqoHkye2Rj3sb920uSVehWZGh7qa8lMho2PQWOSZ1pl29L5pHW\/eagHmrmnok2auTCqB5NnNsZ97K+dNHkyDEqN5JkNk4Wl+xg0afI1VZdNnY+pe8xbudDMPUVt1MzmUT2YPLMx7mN\/7bTJMxsO4Ur3MWjCkbE7lY2dj93Esm8dNMteA90W2KqZrW\/LgMnTjTC99D72186ZPLmj9mc\/+xn19fXR7t27aWZmZpHSeOOFXvD7ltrWzsc3HXTuF5rp0LIjre2a2fa2DJg8s3ELk2eWby5L9zFo8iCk7Z1PHhgnfQ\/QLGmi5stzQTObRvVg8szGpI\/9tXMjeWZDQL90H4NGn5J9OVzofOyjlm2LoFm2\/KPU7pJmclQvywOUYfKiRFn4PD721zB54eOjaEofgyYmMiuyu9T5WAHMgkZAMwtE0GyCa5plvSkDJk8zwDST+9hfO2\/yTpw4Qdu2bVskNdbkaUa\/Z8ld63w8k6fo7UIz96LARc2ynL6FyTMb4zB5ZvkmXrrchLFx40bq7OxMvPwwBfoYNGG42J7Gxc7Hdqam2wfNTBNOvnyXNcti+hYmL\/kYVEv0sb92eiQP5+SZfSDyXLrLnU+edSl3b9DMPeVd1yzt6VuYPLMxDpNnlq+R0oeHh2lycpIGBgaMlF+pUB+DphITF753vfNxgXHSbYRmSRM1X14eNEtz+hYmz2xM+thfOz+S19rainPyzD4XuSw9D51PLoUpc1PQzD3F86SZnL7dtLKWntq0wogYMHlGsBYKhckzyzeXpfsYNHkQMk+dTx70CHMP0CwMJbvS5E0zOX1r6pgVmDyz8etjf+30SJ7ZcAhXuo9BE46M3any1vnYTTuZ1kGzZDimWUoeNVPX6f1V5x10y01LEkMKk5cYyqIF+dhfO23y5O7aYq81kwrX1NTQ4cOHqampKVT08JEs\/Kq048ePU21trcjD6\/76+\/vFzz09PQt28voYNKFAWp4oj52P5chjNw+axUaYegF51Uyu0\/vRW2\/T0MYV1Ny4NBG2MHmJYCxZiI\/9tdMmj5Xs7u6mxsbGBcZL3YzBP4+NjdGxY8cqRo80jZxQmrwzZ85QV1cXDQ0NifzyZ2kafQyaiiAdSJDXzscB9JGbCM0io8ssY941+\/RTr9Op81eIR\/SSMHoweWZD1cf+2mmTV+oIFTZmO3bsoAMHDtClS5fEz88\/\/3zF6GFD+K1vfYvm5+cLJk+axJGREaqurl5kKn0MmoogHUiQ987HAQm0mwjNtJFlnsEHzZI0ejB5ZkPWx\/7aaZMnR\/JGR0dp\/\/79tG7dOpJvwGhpaRHHqoQdyWNjyKN1GzZsoD\/\/8z8vmDweKeRLHtES\/Oxj0Jh9DNMp3YfOJx2S6dUCzdJjnVRNvmj2uaPn6Ojpi7FH9GDykoq84uX42F87b\/JYSjZo7e3tNDc3R+oaPHVET66vKxVCbN7Wr18vvlbX5AWng4Pn8smgOXLkCPGr1ORVqT6zoYzSKxHwpfOpxMGl76GZS2pdb6tPmj38te8Lozd4XyNtWnlzJLEmJiZoamqKlixZQjxQkcWVJ814tk+9eP1+W1ubWMKl9tdZcE6rzlyYvLiwePTvxRdfFKN1wY0XYU1esA1sOjdv3hy3achviMC1a9fo8uXLYnNNVVWVoVpQbJIEoFmSNNMpyzfNHv\/rn9A3z83TX95TSx+v1991e\/bsWbpw4YIwec3NzemIFKglT5odOnRIbLwMXjB5mYRWOpWyaePpXb5WrVolpmZ37twpNlTwZopiJo\/TVpqu7evro\/r6+sJNsHnAaF46mkap5erVq2K9Jv9rDiYvCsH080Cz9JnHrdFHzb7wX38gRvSOb7mNmhtv1ELII3nT09Ni\/TcvHcriypNmPJKnjuaNj4\/T4OAgRvKyCKys6lSnetU2yGnfU6dOLXhtWnBkz8c5\/qy0SrLePE1JJMnF5rKgmc3qFG+br5pF3YyBNXlmY9zH\/tr56Vq50SIYGjxCo551FzZ0giN5OEIlLDm30vna+bil0sLWQjP31PNZMzZ6uufoweSZjXGYPLN8Ey9dnmv3yCOP0AsvvCCmXJcvX04dHR20Zs2aBWfnha0chyGHJeV2Op87H1eVg2buKee7ZtLovdH7e6HEg8kLhSlyIpi8yOiyyaiek8cLLOWhyDq7auO23MegicvMhvy+dz42aKDbBmimSyz79L5rxm\/GuH33q7RpZS09tWlFRUFg8ioiipXAx\/7a6elaPrSYR+3q6urE8Sfbtm2jffv20bPPPkuzs7ORpmt1I8jHoNFlZGN63zsfGzWp1Catmf7fAAAgAElEQVRoVomQfd9DMyL5rtswb8WAyTMbwz72106bPBkOjz32GN1zzz1ityQbPd41K99QYTZkiHwMGtNM0ygfnU8alJOtA5olyzON0qDZdcphN2LA5JmNSh\/761yYPLNhUb50H4MmS95J1Y3OJymS6ZUDzdJjnVRN0Ow9kmz0+Prm5+4oiRcmL6nIK16Oj\/01TF7MmPIxaGIisyI7Oh8rZNBqBDTTwmVFYmj2ngxhpm1h8syGrY\/9tXMmT+6o5deTlLuiHqGiG2I+Bo0uIxvTo\/OxUZXybYJm0Mw9AgtbLKdtfzrwB0VvBSbPrMI+9tdOm7y0jFy5sPMxaMw+humUDsOQDucka4FmSdJMpyxotpjzTd0vl9xtC5NnNi597K+dM3lqCKgHIWdl+HwMGrOPYTqlo\/NJh3OStUCzJGmmUxY0W8yZX3n2uaPniM\/Ou+Wmhe+3hckzG5c+9tdOmzw1HILvpMXuWrMPi+ulo\/NxT0FoBs3cI1C8xXx23l3Lly46Ow8mz6zCMHlm+aZSulyzx5VFea2ZbiN9DBpdRjamh2GwUZXybYJm0Mw9AsVb\/ORLP6AnX5peNJoHk2dWYR\/761yM5MlDkScmJkSEtLS00MDAgNloebd0H4MmFbCGK4FhMAzYQPHQzABUw0VCs9KAeW3eo3ffSo\/e\/eFCIpg8swHpY3\/ttMlTp2jTNHZqGPoYNGYfw3RKR+eTDucka4FmSdJMpyxoVpozr8vj9XnqTluYPLNx6WN\/7ZzJU49QycrYweSZfRDTKB2dTxqUk60DmiXLM43SoFl5yjyax++05Xfb8gWTZzYqYfLM8k2kdJyTlwhG7wtB5+NeCEAzaOYegfIt5nPzeIctGz2YPPPqwuSZZ5y7GnwMmjyICMPgnorQDJq5R6B8i4PHqWAkz6zCPvbXzk3Xmg0B\/dJ9DBp9SvblgGGwT5NKLYJmlQjZ9z00q6yJOmULk1eZV5wUPvbXMHlxIoaIfAyamMisyI7OxwoZtBoBzbRwWZEYmlWWgTdg8JQt77KFyavMK04KH\/trmLw4EQOTF5NedtnR+WTHPmrN0CwquezyQbPK7NUp2x9\/73WampqiG264QRwFlsWVZ81g8rKIKMfr9DFoHJdMND\/Pf8jyoE+xe4Bm7ikLzcJpJqdsb33nBzB54ZBFSuVjf42RvEih8l4mH4MmJjIrsqPzsUIGrUZAMy1cViSGZuFkkK85+8ytV2DywiGLlMrH\/homL1KowOTFxJZ5dnQ+mUug3QBopo0s8wzQLJwE8jVnz9+3BCYvHLJIqWDyImHzO5OPQZMHxdH5uKciNINm7hEI12K5Lu8v1hK9c2UWa\/LCYdNO5WN\/jZE87TBZmMHHoImJzIrsMAxWyKDVCGimhcuKxNAsvAw8Zftb\/\/oybaibg8kLj00rpY\/9NUyeVogsTuxj0MREZkV2dD5WyKDVCGimhcuKxNAsvAx8lArvrOV1edhdG56bTkof+2uYvHcjZHh4mPr7+8WnVatW0cjICFVXV4vP6nc9PT3U2dlZiCsfg0bnobI17fT0ND3zzDPU0dFBDQ0NtjYT7VIIQDP3wgGahddMTtl+5ePZTtfmWTMf+2uYPCI6ceIEbd++nQ4fPkxNTU3U3d0tnsyBgQE6c+YMdXV10dDQkPid\/JnT8eVj0IT\/s2VvSuhmrzalWgbNoJl7BPRazEep8EjeH97yvszOycvzc5bneysVaTB5RMLUNTY2Lhihk8B4FG9sbKwwshdMa0PQzMzM0HPPPUf33ntv7FGpqGXp5KuUNur3pfIV+33WulW6R72ugShKeTp5KqWN+r1LmrEmle5TR7coZenkCZO2XBodbUqxyfo5s0EznTZ8+qnX6dyP\/pF23\/4T2rhxY9FwgmY6T9nCZ5bZtbW1iT7dlxkc703exYsXqbW1lXp7e2ndunWLokcd1eMvg5\/lH7EjR45kFjQycJNoQ9SydPJVShv1+1L5iv2+Uh16f0b0Uyddf5TydPJUShv1e5c0k501dxJZPWuVOKuRGCZtuTQ62pRiE6YN+k+PXo4k2xC1rLD5\/vt3\/jcNnzhH9\/z6P5c1eaViEJotjg2VCX8Lk6f3\/Difmk3e1q1b6f7776d9+\/bR3NzcgjV5wZE7HtmbnJwUU7nyDxtP9Y6PjzvPAjcAAiAAAiCQHYEVK1ZQbW2teCMPv8cWV\/IEVq9eTUePHk2+YEtLxEjeuyN5dXV1YkqWL16Mz5\/ZyFUyedLo8b8WcIEACIAACIAACNhLgKdpfZmqZRW8M3ls2kZHR0UE8i7aPXv20J\/+6Z8umK7ljRi7d++m48eP0969e0VaOXIXnK61N5TRMhAAARAAARAAAZ8JeGfygmLPz8+LkbstW7YU1uSxyTt48KAY2eMdt+r0bLlNGj4HEu4dBEAABEAABEDALgLemzyWQ91By5\/Z9K1Zs0bstq10hIpdcqI1IAACIAACIAACIHCdAEzeu5GgHnjc0tJSmJ6VJlAelBw8DBmBBAIgAAIgAAIgAAI2EoDJs1EVtAkEQAAEQAAEQAAEYhKAyYsJENlBAARAAARAAARAwEYCMHk2qoI2gQAIgAAIgAAIgEBMAjB5MQEiOwiAAAiAAAiAAAjYSAAmz0ZV0CYQAAEQAAEQAAEQiEkAJi8mQGQHARAAARAAARAAARsJwOTZqAraBAIgAAIgAAIgAAIxCcDkxQSI7CAAAiAAAiAAAiBgIwGYPBtVQZtAAARAAARAAARAICYBmLyYAJEdBEAABEAABEAABGwkAJNnoypoEwiAAAiAAAiAAAjEJACTFxMgsoMACIAACIAACICAjQRg8mxUBW0CARAAARAAARAAgZgEYPJiAkR2EAABEAABEAABELCRAEyejaqgTSAAAiAAAiAAAiAQkwBMXkyAyA4CIAACIAACIAACNhKAybNRFbQJBEAABEAABEAABGISgMkLAXB4eJj6+\/tFyp6eHurs7AyRC0lAAARAAARAAARAIDsCMHkV2J85c4a6urpoaGhIpJQ\/NzU1ZacaagYBEAABEAABEACBCgRg8ioA4lG8sbExGhkZoerqauru7qbGxkaM5uHRAgEQAAEQAAEQsJoATF4FedjU8TUwMCD+H\/xstbpoHAiAAAiAAAiAgLcEYPJCmDx15I5H9iYnJwumj7PPzMzQ\/7hQVbSkZTcuiRxct9wULe+v\/OInketERhAAARAAARDIK4GGhoa83lrR+4LJi2ny2OBt376dXrr5s84HThxz+Cu\/+MdI9x+mzlJlB\/Mu\/rywTWHqinQTyAQCIAACCRBYsWIF1dbW0ttvv02vvvpqAiWiiCCB1atXU19fH\/li9mDyQpg8TlJquva1116jtrY2ETT19fWLSvt\/7\/9g5KcsbN7x8ddocHDw3TY00I\/eejtSnT\/86VWamfkxPffcc3TvvfdSQ8Pi+ylV8CtvniduB+erdHEdMu0Pf7q4rT\/+8YxoBz+M8lLviY11WDal2lLOEKojqOpIrPy9+j3\/HGW0dnx8XNEsPOdS9xOlPJ08ldJG\/b5UvmK\/r1RHpbhL4vsk2xClLJ08YdKWS6OjDbOFZqUjLIwWnPv06dP0xhtv0MqVK+n2228vWiA003uSVV7cd3BfyevsYfL0OOY2dXB6NrjxQpq8LIOGA5eN2cMPPxxbh6hl6eSrlDbq96+8OSk6GtVosom8bhpnqO433vujyWlVM6uaTfVnaS6LmdFisKUBlMYvaAxvuelXRTY2mV\/\/+nO070uPUtRpebX+SsyKtVUnT6W0Ub8vla\/Y7\/GsXV8aEvZZD5O2XBodbTi+oFnpP79htODcL7\/8Mp06dYo++tGPUktLS9ECoZleN6fysuFviF7r46fGSF4FhpWOUPExaOKHXfYlxNGtmCF8zwxeLdwcp5Npw5hF1SDyz+8ZxF8tjBY2Ny7NHl5GLYijWUZN9r5aaKYXAjxFOzU1RTfccENJk6dXon7qPGuW53srpTRMXohnoNxhyD4GTQhk1ieZnp6mZ555hnp7e6mqqvimGZM3Ic0f\/58NIE+Vy0s1h6fOXynaDNUQSuPHo4Q8gqgaRJP3kHbZWWuW9v3moT5opqeiDSYvz5r52F\/D5Ok9g4tS+xg0MZFZkZ0XNl+4cIGWLVuWicnThVDKFH578roJLGYGpRG8a\/nSwqjgXctvdNYEuqaZrsZ5TA\/N9FS1weTlWTMf+2uYPL1nECYvJi9bsuf1D5kcBVRHB9kIXv+8cJOL3DQiRwLZANo8HZxXzWx5Jky0A5rpUYXJ0+OlmxomT5cY0pOPQZMH2X3tfIImsNhIoBwB3LSyVkhti\/nzVTOXnzdopqceTJ4eL93UPvbXGMnTjZJAeh+DJiYyK7Kj81ksgzSAp86\/Jb5kA6hOA8uRv7ZVN4s1hGmbP2hmxaOj1QhopoVLnI2X9caLPGvmY38Nk6f3DGK6NiYvW7Ln+Q9Z0oxV8\/fez9fXAqqjfrzxQ47+Jd0GLg+amaBqtkxopscXJk+Pl25qmDxdYkiP6VpHYwCdTzzh5Pq+o6cviLV+R09fLBSorvVLcrQPmsXTLIvc0EyPOkyeHi\/d1DB5usSQHibP0RhA55O8cOqInzrVq472xTF90Cx5zUyXCM30CMPk6fHSTQ2Tp0sM6WHyHI0BdD7pCHd9lK\/4aB9P7eqYPmiWjmZJ1gLN9GjC5Onx0k0Nk6dLDOlh8hyNAXQ+2QgnTV9wpE+e5ffo3R8u2TBolo1mcWqFZnr0YPL0eOmmhsnTJYb0MHmOxgA6HzuEK2X6eJRv08qbF7zXF5rZoZlOK6CZDi3C7lo9XNqpYfK0kSGDj0GTB9XR+dinojR83LInX5oWDeT1fHJa9xMNS5x6S4l9hNNvEZ4zPeYYydPjpZvax\/4aR6joRkkgvY9BExOZFdnR+VghQ9lGXJ\/SfWuB4VvfuIQ++4cfoV\/\/N9X23wBaiGNvNGMAJk8TmGZyH\/trmDwiunjxIrW2ttLMzEwhZHp6eqizs1N8Hh4epv7+fvGz+nv+7GPQaD5XViaHybNSlpKN4lG+\/\/zqj2jfyevPqBzhC07punVX+W8tnjM9jWHy9Hjppvaxv4bJI6IzZ87Qjh076MCBA1Rbe\/1VTvLi77q6umhoaEj8Sv7c1NQkPvsYNLoPlo3p0fnYqEr5NknNfnnDr9HXXr+8aEq33KYN9+42Hy3Gc6anI0yeHi\/d1D721zB5RHTixAk6ePAgjYyMUHX1wmkgHsUbGxsrfNfd3U2NjY2FUT4fg0b3wbIxPTofG1UJZ\/KWLVtGVVVVInFwSpfX7zU33mj0zRvukcuuxXjO9NjD5Onx0k3tY38Nk\/fudKxq5NTAYVPH18DAgPh\/8LOPQaP7YNmYHp2PjaromzyZQ27a4Ddv8M9yOheje9nqjOdMjz9Mnh4v3dQ+9tcwee8at9HR0UK8rFq1quTIHY\/sTU5OFkyfDJojR45QQ0NDoYzgtK9uMCK9WQLofMzyNVF6WM2k4VPX77HZu++OD5poFsosQyCsZoB4ncDExARNTU3RkiVLqKWlJRMsedKM19urF6+7b2trE7Nzan+dCeiUKvXe5M3Pz1NHRwfV1dUJ4xb8HJyeLWXygnq1t7fT5s2bU5IR1egSuHbtGl2+fFmswZRTf7plIH26BHQ1m\/35O\/TNc\/P0rb+bJ\/657gNV9KnbqumB1UvTbbjHtelq5jEqcetnz54VxwSxyWtubs4ER540O3ToEB0+fHgRR5i8TEIrnUrZtMlRO3XETq2d1+jt3r2bjh8\/Tnv37hVfVZqu7evro\/r6+kIxbB4wmpeOplFquXr1Kl26dEn8aw4mLwrB9PPE0YxN3r6TPyKezmWzx6N6mMo1r2Eczcy3zr4aeCRvenparA3fsGFDJg3Mk2Y8kqeO5o2Pj9Pg4CBG8jKJrBKVypE1Dv7gVcqkxW2\/uhGD\/xWgTs9i40Vcunbkz9OUhB1EzbciCc3kVK48bPmpTStIvlLN\/B34V0MSmvlEDWvyzKqNNXlm+WqVzkeX8JQnX2y05JElwRG3bdu2UU1NTck0lSqVZ+T19vbSunXrCmfmbdy4UeygxREqlQi6+T06H\/d0S1Kz4EYNNno8stfciKncJCMjSc2SbJetZcHkmVUGJs8s39Cls\/HiadJdu3YtOtKkWCE82rdz587ClGroit5NGDwMmRe8yulZToLDkHWJ2p8enY\/9GgVbaEozPoal69i5wq7coY0rYPYSCg9TmiXUPOuKgckzKwlMnlm+uSzdx6DJg5DofNxT0bRmQbPHI3t87h6u6ARMaxa9ZXbmhMkzq4uP\/bX3u2vjhpSPQROXmQ350fnYoIJeG9LSjKdyP3f0HJ06f0Wctwezp6eTmjotzaK30K6cMHlm9fCxv7be5BV7r2wwDOKsyYsbUj4GTVxmNuRH52ODCnptSFuzoNnDNK6eXpw6bc30W2hXDpg8s3r42F9bb\/JY8uCOVv6del6dfPXYsWPHzEZIkdJ9DJrUIRuoEJ2PAaiGi8xKM5i96MJmpVn0FmebEybPLH8f+2vrTR6P5G3dupX27NmzYIct73rdsWMHHThwQJx3xj8\/\/\/zzZiMEJi91vqYqROdjiqy5crPWTF2zx7tx+fgVns7FVZpA1pq5pg1MnlnFYPLM8o1cujzAeP\/+\/eKYEz7Hjo9OkbtgMZIXGa23GdH5uCe9LZqpZo83ZvCaPZi94vFki2auRDtMnlmlYPLM8o1Vujw3b25ubsG5eOqIXhZvmPAxaGIJaUlmdD6WCKHRDNs0e\/KlH5A8VPnRu2\/FGzSKaGmbZhrhlklSmDyz2H3sr62frjUrefzSfQya+NSyLwGdT\/Ya6LbAVs2k2ePRPDmyp3tveU1vq2a28obJM6uMj\/01TF7MmPIxaGIisyI7Oh8rZNBqhM2aqa9Lg9l7T1abNdMKvpQSw+SZBe1jf+2EyVPfX8vvq33ooYfoiSeeEJsuspiiVcPQx6Ax+ximUzo6n3Q4J1mLC5qx2eORvaOnL4p1er4fu+KCZknGaNyyYPLiEiyf38f+2nqTJw3emjVrqLGxkQ4ePEgjIyPiXbVjY2Pi5+rqarORUaZ0H4MmM9gJVozOJ0GYKRXlkmbqsSs8hcs7cX28XNLMBn1g8syq4GN\/bb3JU49Q4aNSpMk7f\/584QiVLEfzfAwas49hOqWj80mHc5K1uKiZ75szXNQsyZjVLQsmT5eYXnof+2vrTR5LyEeozM7O0v3330\/PPvusmK598MEHae3atTQwMKClMh+\/Io2iOgIoj2nhwuRRLbJgPqKlv79ffOzp6aHOzs5CnT4GjRZwSxOj87FUmDLNclkzXzdnuKxZFk8ITJ5Z6j72106YPJZdno0nQyBotsKEhiyD1\/Wp07z8+927d9Px48fpzTffLPzMI4R8REtXVxcNDQ2JKuTPTU1N4rOPQROGte1p0PnYrtDi9rmumY\/r9VzXLO2nBCbPLHEf+2tnTF5c6Xmk7uTJk8QGj8\/aU00ef8cXjwrKNYBbtmwRBy\/Lg5Zl+uAr1nwMmrha2JAfnY8NKui1IS+aqev18n6+Xl4004vU6Klh8qKzC5PTx\/7aG5PHo3XFTJu6sYOnYYOfVQPIQRT87GPQhHmYbE+Dzsd2hfI3khe8Ix+mcPGc6T1nMHl6vHRT+9hfW2nyeLNFa2srzczMlNWwoaFBTLHqbLwIjswFR+6kkeOdvGz6giN3nH9ycrKwFlAGzZEjR4jbIy+dNukGKtLHJ4DOJz7DtEvIo2byfL19J2fEkStf\/pOPUHPj0rTRGqsvj5oZg0VEExMTNDU1RUuWLBGv7cziypNm7CXUiz1FW1ubOJlD7a+z4JxWnVaavODNB40Wfx80a2GBmTJ5wfrb29tp8+bNYZuFdCkTuHbtGl2+fFn8A6Gqqirl2lFdFAJ51mz25+\/Q43\/9E\/rbH79NH69fQo\/\/uw9S3Qfcj8s8axYlhivlOXv2LF24cEGYvObm5krJjXyfJ80OHTokjlsLXjB5RkInWqHqESpyswOXVOqdteo7bjldsZ2y6vl6SU3X9vX1UX19feEm2TxgNC+a5mnkunr1KvGRPPyvOZi8NIjHr8MHzV6bnqfPf\/37xCN8D\/9+rfPvw\/VBs\/iR\/V4JPJI3PT0tzn7dsGFDkkWHLitPmrF\/UEfzxsfHaXBwECN5oaMhpYTyeBNp2OQuWR7O1j1CpdgIoDpSWGzjhTo9i40XKYluuJo8TUkYRmVN8T5plpf1ej5plsSDgjV5SVAsXQbW5JnlG6t0dYSupqZGDMGqI3thCy9m8nCESlh6+UmHzsc9LX3TLPg+XBdfkeabZnGfKpi8uATL54fJM8vXitJLreXDYchWyJNaI9D5pIY6sYp81czlI1d81Sxq0MPkRSUXLh9MXjhOSKUQ8DFo8hAA6HzcU9F3zVycwvVdM92nDCZPl5heeh\/7ayt31\/JCyb1799KuXbvEAtRKF6+j27lzp\/b6vErlhvnex6AJw8X2NOh8bFdocfugGYkNGUdPX6AnX5oWR67YPoULzfSeM5g8PV66qX3sr600eSycXIPHP5dafyc3YMRZo6cbJMH0PgZNXGY25EfnY4MKem2AZu\/xUqdw71q+lJ7atEKYPtsuaKanCEyeHi\/d1D7219aaPCme3O3KW8uDV\/AdtLqCJ5Hex6BJglvWZaDzyVoB\/fqh2WJm3568Ql3HzokRPhtfkQbN9OIcJk+Pl25qH\/tr602erohpp\/cxaNJmbKI+dD4mqJotE5qV5mvrej1opvdMwOTp8dJN7WN\/DZOnGyWB9D4GTUxkVmRH52OFDFqNgGblcQXX621amf1hytBMK8QJJk+Pl25qH\/trmDzdKIHJi0nMjuzofOzQQacV0CwcLTZ7PLJ39PTFzDdnQLNwmslUMHl6vHRTw+TpEkN68jFo8iA7Oh\/3VIRmeprZsDkDmulpBpOnx0s3tY\/9NUbydKMEI3kxidmRHZ2PHTrotAKa6dB6L22WmzOgmZ5mMHl6vHRTw+TpEkN6jOQ5GgPofNwTDprF00xuzuBS0tqJC830NIPJ0+OlmxomT5dYSunVY1T42JSHHnqInnjiCTpw4ADV1tam1Iri1fgYNJkCT6hydD4JgUyxGGgWH3bamzOgmZ5mMHl6vHRT+9hfWz9dKw3emjVrqLGxkQ4ePEgjIyPigOSxsTHxc5i3YugGQ9j0PgZNWDY2p0PnY7M6xdsGzZLTLC2zB830NIPJ0+Olm9rH\/tp6k8evONu6dSvt2bOHLl26VDB558+fpx07dmiP5vFbMqRRlOaQ62htbaWZmZlCzPT09FBnZ6f4PDw8TP39\/eJn9ff82ceg0X2wbEyPzsdGVcq3CZolr5lpswfN9DSDydPjpZvax\/7aepPHInZ3d9Ps7Czdf\/\/99Oyzz4rp2gcffJDWrl2r9b5a+Rq04Jsy+BVqpQwjf9fV1UVDQ0MinuTPTU1N4rOPQaP7YNmYHp2PjarA5GWlSvDYlUfv\/jDxOXtxLzxnegRh8vR46ab2sb92wuSxkNKgSVGDI2qVxGajePLkSWKDNzc3t2Cat9joniyPR\/HUaWEuh6eN5Sifj0FTibUL36PzcUGlhW2EZuY1S9rsQTM9zWDy9Hjppvaxv3bG5OmKGUzPRm7dunVi6jW4lq\/Y72R+NnV8DQwMiP8HP\/sYNHG1sCE\/Oh8bVNBrAzTT4xUndVJmD5rpqQCTp8dLN7WP\/bU3Jq\/UyJw0bqOjo4V4UadzgyN3bAgnJycLpk8GzZEjR6ihoaFQRta7fnWD37f06HzcUxyapa8Zm72Hv\/Z9OnX+inh7Bk\/j3nfHB0M3BJqFRiUSTkxM0NTUFC1ZsoRaWlr0MieUOk+a8Xp79eJ1921tbWKgR+2vE0JnZTFOmDwehdu9ezcdP36cvvGNb4hNEDU1NWKHrVwbF5ZucNRO7t6tq6sTxi34OazJC9bf3t5OmzdvDtsspEuZwLVr1+jy5cviCJ6qqqqUa0d1UQhAsyjUkskz+\/N36C8nrtA3z81T3Qeq6FO3VdMDq5dWLByaVUS0IMHZs2fpwoULwuQ1NzfrZU4odZ40O3TokPAJwQsmL6FgSaIYabq2bNlCH\/vYx8Qu2N7eXlF0cJcs\/443SrDB4nV3fO3fv19M08qr3NSsTKOayr1794pfV5qu7evro\/r6+kI9bB4wmpdEBJgp4+rVq2K3Nv9rDibPDOOkS4VmSRPVL4\/N3r6TPxLvxWWzx6N6PLpX6oJmeox5JG96elocC7Zhwwa9zAmlzpNmPJKnjuaNj4\/T4OAgRvISipVEigkeoSJH9LiDjnKESliTp57Hp07PYuNFIrJmXkiepiQyh5lSA6BZSqBDVBP26BVoFgKmkgRr8vR46abGmjxdYimkV0fy2GzJTRNRD0MOmjx5Rh6PDvKIn\/y8ceNGsYMWR6ikIHIGVaDzyQB6zCqhWUyABrJXMnvQTA86TJ4eL93UMHm6xFJKL49PkevwuNrgeXVhm1JsJC94GDIveJXTs1wuDkMOS9eddOh83NFKthSa2auZava4lfLduNBMTzOYPD1euqlh8nSJIT0OQ3Y0BtD5uCccNLNfs6DZ+8LaBvq3DUSrf+vDWPsaQj6YvBCQYiSByYsBz9esPgZNHrSGYXBPRWjmlmZPvvQDsUGDjd9dy5eKDRrNjZV35Lp1l8m2FiYvWZ7B0nzsr505QmXbtm2L1OedkXysSpa7WH0MGrOPYTqlwzCkwznJWqBZkjTTKYs1e3z0HL04+bYwe\/KsvSRemZbOHaRbC0yeWd4+9tfWmzy5Xu6RRx6hF154QazFW758OXV0dNCaNWsKrxczGxqlS\/cxaLJinWS9MAxJ0kynLGiWDucka1E14+NX5Ogemz02euWOX0myHa6UBZNnVikf+2snTN7WrVtpz549xAcbyvfG8q7XKEeoJB1CPgZN0gyzKA+GIQvq8eqEZvH4ZZG7mCS\/rU0AABY6SURBVGaVduRm0U5b6oTJM6uEj\/219SZPfQPF+vXriadt9+3bR88++yzNzs5iutbsM5Hb0mEY3JMWmuVLs2I7cjetvFlM6fp6weSZVR4mzyzfWKU\/9thjdM8994i3FLDRU98vG6vgmJl9DJqYyKzIDsNghQxajYBmWrisSBxGM2n25CYNn9ftweSZDVsf+2vrR\/LMSh6\/dB+DJj617EsI0\/lk30q0QCUAzdyLB13N2Ojxuj25ScO3dXsweWZj3Mf+GiYvZkz5GDQxkVmRXbfzsaLRnjcCmrkXAFE1Y5Pn4yaN5\/9mgoa+fZnu+8gvaUvrpzIRPKpmmTRWs1If+2uYPM0gCSb3MWhiIrMie57\/kFkB2EAjoJkBqIaLjKtZsU0aeV63xybvP\/y3f6Idv\/M29XzmjwyrU7z4uJpl0uiQlfrYXzth8uRrzYI64py8kJGNZIsI5PkPWV7lhmbuKZuUZsFNGnIaN2+bNA6f+J\/0+Rd\/Rv2\/+08YyTMQ7jB5BqDGLVKek7dx48bMz8Qrdi8+Bk1cTW3In1TnY8O9+NIGaOae0iY0U9+kkbdNGtLkHVzzM+J3qGdxmdAsi\/tAf32dgPUjeWzy5Dl5TU1NkWOlu7ubRkdHRf6amho6fPgwqeWp3+\/fv5\/WrVtXqGt4eJj6+\/vF556engVmEyYvsiSZZszzH7JMwRqsHJoZhGuoaJOafXvyCnUdO5erTRpf\/tYZsSav\/3fnYfIMxKSP\/bX1Jo915unaF198kQYGBiLJziZtbGyMRkZGqLq6mvjzsWPHCmfscfm7d+8Wn998883Cz\/y6ND50md+yMTQ0JOqWP0uD6GPQRBLBskwmOx\/LbjU3zYFm7kmZhmZ52qSx8xv\/i45MXITJMxTqPvbXVpo8OUU7MzNTVuqoa\/JU48ZmjUfx+GITKQ9f3rJlixjNCxpETivfusF5fAwaQ89fqsWm0fmkekMeVAbN3BM5Tc3y8CaNzx09R9\/5u7+n\/3jH2xjJMxDuPvbXVpo8A9ouKFI1ecH34EqTJ9+LqxpALiT42cegMa1PGuWn2fmkcT8+1AHN3FM5C81cNnuffup1+od\/uASTZyjUfeyvrTZ55dbJxYkBLpdficbTt3x1dHSQHLmTRk6O1gVH7nhkb3JysjB1LIPmyJEjxCOL8uKpXlz2Esii87GXhhstg2Zu6KS2MkvNpNnbd\/L6jNCjd99KX1i7zGqI\/\/7p79I\/v\/Vj6rgNI3lJCMWzgurFs4NtbW1i+ZbaXydRl61lWGvy2Fx95zvfEevk+FVm7e3t9MADD8TeYcsm7emnny5svAhOz0Y1eUGBub2bN2+2VXfv23Xt2jW6fPkysRmvqqrynocLAKCZCyotbKMNms3+\/B365rl5+suJK6Jxn121lD69oprqPmDfc\/\/pQzP0m\/\/qH+jTy35Bzc3NmQhug2ZJ3fihQ4dEXx+8YPKSIhyxnGI7anlzxMGDBwubJ0oVzVOxbLDm5uZEEnWnbNDg8ffB6dmo07V9fX1UX19faBabB4zmRQyAFLJdvXpV\/OOB\/zUHk5cC8ASqgGYJQEy5CNs0G\/ybC\/TkS9OCwsO\/X0u2Haxct+NVWl\/7c\/qTxv9LGzZsSFmt69XZplkcCOwl1NG88fFxGhwcxEheHKhJ5I1j8krVH9xRq6ZTp2SLbbxQp2ex8SIJhbMvI8tppOzv3s0WQDP3dLNVMz5rT5o9nsZ99O4PZw6Xp5dv3\/0qfebWK\/SHt7wPGy8MKII1eQagRikyaZPHo4Dbt29fdDaebBuOUImiktt5bO183KZqtvXQzCxfE6XbrplNZk+avId+4yd0e+2\/hMkzEJAweQagRikyaZOnbuBQ26NO5eIw5ChKuZvH9s7HXbLmWg7NzLE1VbIrmtlg9vhw5z8efp2+8vFZuuGGG2DyDAQlTJ4BqFGKNH1OXpQ2lcrjY9AkyS+rslzpfLLiY2O90MxGVcq3yTXNpNnj16XJ9+OmRZ3r\/urL5+lLv30JJs8QdB\/7a2t31xrSOPFifQyaxCFmUKBrnU8GiKyrEppZJ0nFBrmoWVbn7MmDkB\/6jX+EyasYWdES+Nhfw+RFi5VCLh+DJiYyK7K72PlYAS7DRkCzDOFHrNplzYJmjzdn8OieqYsPQn7nyqzYeIHpWjOUfeyvYfJixpKPQRMTmRXZXe58rACYQSOgWQbQY1aZB82C78Yd2riCmhuXxiSzOPtN3S\/TH908Rxvq5mDyEqd7vUAf+2uYvJjB5GPQxERmRfY8dD5WgEyxEdAsRdgJVZUnzdjs8ZTqqfNX6K7lS+mpTSuI1+4lccmdtTt+5226+X0\/hclLAmqRMnzsr2HyYgaTj0ETE5kV2fPU+VgBNIVGQLMUICdcRR41412wXcfOERuzpM7Yk2XyVC1MXsJBqBTnY38NkxcznnwMmpjIrMiex87HCrAGGwHNDMI1VHSeNUtyJy6XdfT0RfqLtURTU1MYyTMUjz721zB5MYPJx6CJicyK7HnufKwAbKAR0MwAVMNF5l2zpDZn8KYLnvrlkTyYPHNB6WN\/DZMXM558DJqYyKzInvfOxwrICTcCmiUMNIXifNEszuYMuR7vrzrvoH9x+RxMnsG49LG\/hsmLGVA+Bk1MZFZk96XzsQJ2Qo2AZgmBTLEY3zRTN2fwyFyYnbhyqvaN3t+jV199FSbPYHz62F\/D5MUMKB+DJiYyK7L71vlYAT1mI6BZTIAZZPdVM3VzBpu9Umfs8To83rHLo3h8LAtMntkg9bG\/hsmLGVM+Bk1MZFZk97XzsQJ+xEZAs4jgMszmu2Zs9nikjo9dka9KYznuWn4jnTr\/Fj350rQ4YJmPY+ELJs9ssPrYX3tj8rq7u2l0dFREUE1NDR0+fJiamprE52Lvyu3p6aHOzk7x\/fDwMPX394uf1d\/zZx+DxuxjmE7pvnc+6VBOthZolizPNEqDZtcpyw0abPrY8PFVbIQPJs9sVPrYX3th8tikjY2N0cjICFVXVwvTduzYMTp+\/DjV1tbSmTNnaMeOHXTgwAHxWb34u66uLhoaGhK\/lj9Lg+hj0Jh9DNMpHZ1POpyTrAWaJUkznbKg2WLObPikyQt+C5NnNi597K+9MHnBsFGNG5u1EydO0MGDBwsmUE0fNIg8ItjY2FgY5fMxaMw+humUjs4nHc5J1gLNkqSZTlnQTI8zTJ4eL93UPvbXMHlNTWJkTx3pUwOHTR1fAwMD4v\/Bzz4Gje6DZWN6dD42qlK+TdAMmrlHQK\/FMHl6vHRT+9hfe2ny2KjNzs4WRu7U9XocNKtWrVrwnTpyx4ZwcnKyYPpk0Bw5coQaGhoKMRec9tUNRqQ3SwCGwSxfE6VDMxNUzZYJzfT4TkxMiCNUlixZQi0tLXqZE0qdJ814vb16zczMUFtbmxjUUfvrhNBZWYx3Jo9N2tNPP13YeDE\/P08dHR1UV1cnjFvwc3B6tpTJC6rb3t5OmzdvtlJ0NIro2rVrdPnyZbEGs6qqCkgcIADNHBAp0ERopqfZ2bNn6cKFC8LkNTc362VOKHWeNDt06JDo64MXTF5CwZJFMbzejg3W3NycqH7\/\/v20bt068XPQ4JVqH6\/R2717t9iYsXfvXpGs0nRtX18f1dfXF4pk84DRvCwiIFydV69epUuXLol\/zcHkhWOWdSpolrUC+vVDMz1mPJI3PT0tNghu2LBBL3NCqfOkGY\/kqaN54+PjNDg4iJG8hGLFqmKCO2rLNU7diMH\/ClCnZ7HxwipZIzcmT1MSkSE4lhGaOSYYEUEzPc2wJk+Pl25qrMnTJeZIejZt27dvX3A2nmy6PCOvt7dXjPjJzxs3bhQ7aHGEiiMiazYTnY8mMAuSQzMLRNBsAjTTAwaTp8dLNzVMni4xR9IHN1bIZsup3OBhyLzgVU7PclochuyI0BrN5CmRZ555RqzH9GUBrgYeK5NCMytlKdsoaKanmQ0mL8+aweTpxSNS440XzsaAjw+7s2K923Bo5p6C0ExPMxtMXp41y\/O9lYo073bX6j1ylVPbEDS8Lfy5556je++9N\/aoVNSydPJVShv1+1L5iv0+a90q3WPlyFuYIkp5OnkqpY36vUuaMfFK96mjW5SydPKESVsujY42pdhk\/ZzZoJlOG15++WU6deqUOHCflwwVu6CZzlO28JnFESp67JBaGckLnpOXJhwZuEm0IWpZOvkqpY36fal8xX5fqQ7T+iVdf5TydPJUShv1e5c0k501n7OV1bNWibMat2HSlkujo00pNmHa4NKzFvV+wub77ne\/S1\/\/+tdpxYoVZU1eqRiEZoujSWXC3+KcPNNPXM7K5wDiTR28NRsXCIAACIAACEQlwOaOj97iDSs8dYsreQKrV6+mo0ePJl+wpSViujYBYdjo8X+4QAAEQAAEQAAE7CXAG+182mwHk2dvLKJlIAACIAACIAACIBCZAExeZHTICAIgAAIgAAIgAAL2EoDJs1cbtAwEQAAEQAAEQAAEIhOAyYuMrnJG9ZDl4AHLlXMjRZYEHnvsMbrnnnuoqakpy2ag7hAE1PdV4zkLAcyCJOrfxp6eHvF2IVzuEOC3SL344osLXhrgTuv9ailMnkG9+U0ZfLW3t9MjjzxCXV1dMA0GeSdR9Pz8vHgLxrlz54q+Bi+JOlBGsgRUQw5znixbU6Xx30Y+C+7OO++knTt30he\/+EWxqxSX\/QSkQf\/EJz4Bk2e\/XASTpyGSNABbtmwR77mVV6nXnqkdjvyjpubTqBpJIxLQ1WxqakrUdOjQIYzkRWQeN5uuZmp9MHlx6UfLH1UzNgx79+6lXbt2UXV1dbTKkSsygSi68WtCP\/ShD9EvfvEL+tKXvhS5bmRMhwBMXkjO8mGYmJgg+c5bzspTRTxCNzQ0JEqSP\/M0H0xeSLiGkkXRTDYFZsGQKBWKjaMZppDc0kxOsz\/wwAOYrs1AuijPmnzGNm\/eTN\/4xjdg8jLQTbdKmLwQxOQfIz6ocnZ2lnp7ewsjeTxCNzY2RiMjI+JfovyvHJ6G4DUmMHkh4BpKElUzmDxDgoQoNo5m\/BxOTk5i+igE5ySTxNFMtgOzHEkqEq6sqLpx\/zY6OlqoRB3wCFczUqVNACYvBHE5hff+97+fWltbF5g8Dnq+BgYGxP\/Vz1iTFwKuoSRRNYPJMyRIiGKjasajC2zwsHg\/BOSEk0TVjP9Orl+\/XvxjWf054eahuBIEouomi2OTiJE8N8ILJk9DJ7ngVB3JU0fuuCh1RAE7yDTgGkqqqxlMniEhNIrV0YzXcvFGGV5GIS+MLmjATiipjmb8D2KcPJAQ+JjF6OoGkxcTeAbZYfI0oEd9IDSqQNKECUCzhIGmUBw0SwFywlVAs4SBplQcdEsJdIbVwORpwC\/1QHARxaZrNYpGUkMEoJkhsAaLhWYG4RoqGpoZAmu4WOhmGLAFxcPkaYhQ7IEILvgOTt9qFI+kBghAMwNQDRcJzQwDNlA8NDMANYUioVsKkDOuAiZPQ4BiD0S5I1Q0ikZSQwSgmSGwBouFZgbhGioamhkCa7hY6GYYsAXFw+RpiFDsgeDspQ5D1igaSQ0RgGaGwBosFpoZhGuoaGhmCKzhYqGbYcAWFA+TZ4EIaAIIgAAIgAAIgAAIJE0AJi9poigPBEAABEAABEAABCwgAJNngQhoAgiAAAiAAAiAAAgkTQAmL2miKA8EQAAEQAAEQAAELCAAk2eBCGgCCIAACIAACIAACCRNACYvaaIoDwRAAARAAARAAAQsIACTZ4EIaAIIgAAIgAAIgAAIJE0AJi9poigPBEAABEAABEAABCwgAJNngQhoAgiAAAiAAAiAAAgkTQAmL2miKA8EQAAEQAAEQAAELCAAk2eBCGgCCIAACIAACIAACCRNACYvaaIoDwRAAARAAARAAAQsIACTZ4EIaAIIgAAIgAAIgAAIJE0AJi9poigPBEBgEYGLFy9Sa2srzczMLPpu\/\/79tG7dOu+onThxgl588UUaGBig7u5ucf\/8s3qV+n0QFqdbv369lxy9CxzcMAhoEIDJ04CFpCAAAtEISJPX29sLI0JEzOPzn\/88ffnLX6ba2trYJi9YXjSVkAsEQCBvBGDy8qYo7gcELCQAk7dQlOHhYfGLzs5O8f+4I3lcRrBMC8MATQIBEEiZAExeysBRHQj4SKCSyZufn6eOjg6qq6uj0dFRamlpEVOXZ86cofb2dpqbm6Oamho6fPgwNTU1CYTqd2vXrhVp1qxZI4xT0DSxARobG6ORkRGqrq4WI2nq9LGcMpbt4LomJiZEmQ0NDXT8+HEx4hasV7aJf79jxw46cOCASCfL2bJly6KRS\/7ukUceoa6ursK9hDF5nIbZqJfkxL\/j6d+DBw8W7tHHOMM9gwAILCQAk4eIAAEQME6g1Jq8np4eYcqkKeKGBI2YnOJlE7N7925huPhik7Zx40aRn7\/btm0byfLKmTzOy4ZSGkI2i2y4hoaGaPny5eK72dlZUQ8bQmk+2XQGzaps0zPPPCNMnjR1XKZq+lTAxb4rZuBkHtXIyd+pbZaml9u2detW2rNnT8E8GhcWFYAACFhNACbPannQOBDIB4GwI3nSeBUbmVJHx\/h7afjUkbMwI3nnz59fZMDYZDU2NopRQ9UAcj2qYVSNphzZkwrxaOHk5KQYgVR\/DipYbMQtzEieLKcUy0qM8xFJuAsQAAEdAjB5OrSQFgRAIBKBSgZEGjjV5LFR6u\/vX1Qfj9axIYtq8l555RUx6he8eMRs165dZU1ecNq31Ajd3r17S+52jWPyinGSbSg3RRxJNGQCARBwngBMnvMS4gZAwH4CUUxeuTVmwRG1oPkpN11bbCQvaJRUsxl2JE+2YcOGDfTCCy8Uds4mOZJX7kiVSoztjxK0EARAIGkCMHlJE0V5IAACiwhUMiDFRqiCeeRGi76+PrrzzjsXjLgF1+TxiNuxY8cWrKvjRvF6P77UKVlZD6\/vqzRdW6xNcj0fr42To4+rVq0quQGi1Jo8ble5c\/LKjSJyXqzJw4MHAiAQJACTh5gAARAwTiCKyeNGqTto+bPcWCFNjdwhu2LFCnEPn\/zkJ8VGDHWjB++AbWtro9dff73k7lq5uaGY2QyOngXbpB7mLL974IEHCsejBOFG2V37xS9+sehh0qqZxO5a42GMCkDAOQIwec5JhgaDAAgUM07BDRNZUAo7mmbiTDu5eUSevZfF\/aNOEAABuwjA5NmlB1oDAiAQgUC5DQkRioucJexoWtJvqEi6vMgAkBEEQMAqAjB5VsmBxoAACEQhYIPJ45G0kydPLjiwudy9qO+ujXLPah68uzYuQeQHgXwSgMnLp664KxAAARAAARAAAc8JwOR5HgC4fRAAARAAARAAgXwSgMnLp664KxAAARAAARAAAc8JwOR5HgC4fRAAARAAARAAgXwSgMnLp664KxAAARAAARAAAc8JwOR5HgC4fRAAARAAARAAgXwSgMnLp664KxAAARAAARAAAc8JwOR5HgC4fRAAARAAARAAgXwSgMnLp664KxAAARAAARAAAc8JwOR5HgC4fRAAARAAARAAgXwSgMnLp664KxAAARAAARAAAc8JwOR5HgC4fRAAARAAARAAgXwS+P\/ojr8rX6ge+QAAAABJRU5ErkJggg==","height":305,"width":507}}
%---
%[output:97e357dd]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"num","rows":1,"type":"double","value":[["0","3.136601678636231e-02","2.766158985453013e-02"]]}}
%---
%[output:9d1224cc]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000e+00","-1.656408362613720e+00","6.859221659341664e-01"]]}}
%---
%[output:26f707e8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ares_nom","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-3.553057584392169e+06","-9.424777960769380e+01"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000e+00","1.000000000000000e-04"],["-3.553057584392168e+02","9.905752220392307e-01"]]}}
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
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"     9.905752220392307e-01"}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAnkAAAF9CAYAAAB4cH4qAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQmYFsW19kFZgg6roCMMAUTRcQOMa0DcEI1b0BhEgqDITrwxyCbBqAnqjyBxZb8KiCKEIO5h1OgFEUENoFdBRSGKCA6yCwIX539OkZ40TX9fb1Xdp6fffh4elKk6dep9a3nn1FaprKysjPABASAABIAAEAACQAAIVCgEKkHkVSg+URkgAASAABAAAkAACCgEIPLQEIAAEAACQAAIAAEgUAERgMirgKSiSkAACAABIAAEgAAQgMhDGwACQAAIAAEgAASAQAVEACKvApKKKgEBIAAEgAAQAAJAACIPbQAIAAEgAASAABAAAhUQAYi8CkgqqgQEgAAQAAJAAAgAAYg8tAEgAASAABAAAkAACFRABCDyKiCpqFK6ESgpKaE+ffpQjRo1aNq0adSiRYvYK7Rjxw7q0aMHLVmyhDp06EBjxowp94H9GzFiBM2aNYsKCwtj9W358uXUtWtX2r59O40fP57at2+vyrf8bdu2LfXr1y9Wn7wKGzBgAM2dO\/cAf73yWD\/XUa+xY8fS\/PnzafLkyVRQUOC3aG3p7Jyx0aKiokhtJ1\/b1OZ0DIYsXIqLixPjJoZqooiEEYDIS5gAFA8EnAhIFnksGEaPHh15og7LupvIW79+PXXs2JHWrl1LAwcOFCXyLH9r1aoVWNjoqJclMM8888xEhIRdkFmcR\/Wloog8ez3sv7CE7RvIBwTcEIDIQ7sAAkDANwIQeb6hUgktvJzRUD9WKprIC4OBG04VReRFbR9+2hDSAAGIPLSBVCFgRbksp92WNO3Ri+uuu45uu+228jrmivRYeayEznTOieXSSy9VS6q50ucC1T5x58rrFsmz16lVq1Y0YcIEld3upyUoLLvOZbFcAs2OqRVRcNb3T3\/6U\/nyrb1uuaIyuSJI9vo7oxd+uHVG8tgXOw+Wb3bbTm45jVvkxLmsyGlWrVrlGrn0swQZpK7sk10EObEIWi+3duYsw08d8g0OXnw57fvtK2753Jbmra0Efvqis284+w7\/v58+Zm9L3PbvvfdeuvHGG12jyF5jCpdp1ZX\/O6mtGamaAOBsYAQg8gJDhgxJIeA2WbtNHPnSOSc6t+Uky6Z90s2XLsrk5VZWPpFnx94ucHPV2Z4mTpGXa8nZ+nenAPXLbVCRl8+uXTjkElVugjlXWucvHF4YuPUjq815iTyvep166qnlS9j2crzs+90H6oevMCIvHw\/WLzR++qKdWzeB53fcsPBo1qyZ6y85dmz9+OeMZuqI1iY1HqPcdCAAkZcOnjLvpX3wt0evrMkml+CxD6puaa0JwJ7fbXJ2TizWJGKfyPLtNbLntwscN5+8RJ4zyuiGjd0vC4MoIs86eOF3udZt8sq1zBaE2yB78tyiJHa\/LFxycWP3y+KMO6K1\/88tv7295cLKLcrp1g5zCQC\/9XJGp6yDF14YeC2rBuEryNKq3S+rL3EdrANAFgd8eMT6N\/651Rfd6uUWTbX7ZO+zduHqp485xwQrj98xhX0Pgk\/mJwEAEAoBiLxQsCFT3Aj4Wf6zBlkrrTNa5Jw0+ZSm2wlS+8Dr9tu5U8z52dye61So20nVfCLP7WSiW\/lupzLjFHluAuPzzz93PRkbhNsgIs\/ZRp0RHUvM2G06J3dnW\/rggw9cTz67RShz1csuJvIJKr9Rnlz1yiXyvCKMXqdfg\/AVRMTk8st5OjiXSMtVX3s7cEYK3URevj7m\/Jmz7QQZUyy\/\/IwfcY+3KK\/iIACRV3G4rLA1yTdRuP0s16DpTDt48GDXJS235a18PvgZpHOJPDfSvPbkOa\/C8FM+lxO3yHNGnBYuXHjQ\/rag3AYVefmW6txEnnOvnhOzOXPmqDrk+tyW95xCzm0Z022ZNJ\/I81OvXKInX17Ok2\/JNihfOkSeE+ugfTHfErCbyHOLyPsVttdcc43vMcWql9\/oeIUd4FExowhA5BmFF8Z1IBB0YoHIc78LLW6RZ+etd+\/etHTp0oPu3QvKbRCRZ5\/cLeFy1FFHHbTcmk+AmxB5Vp9wEx\/2SFEukee3XhB5k8kePWY8WMT\/\/Oc\/L4\/gQ+TpGKFhQzICEHmS2YFv5Qj4\/U2aL8KNulzrBnvQ6IHTRq4lQevfR40aVX6xb9hIntthhnXr1pXfjxa3yLNHDxs2bEjbtm1TsDhPEQbhNojIs+prn8jd9m3pWK4NEm1ya19uPuQSeX7rlUvk5VoW9TvcBOErTCTPEmPWRdfs76BBg8rbTZC++Pbbb6vldXvf8NqTly+SF3a5Nh+2bnz65QLpgIAXAhB5Xgjh5yIQCLLZO9eeJ78HL9yERJCJxe1VgVyb++1LZ9ZSYVCR54aN2yZ2a8JjQq29Z86rNnJdoRL04IXVaJxLk24TaBBuw4g8txPG7J+ugxe5xFS+vZJ8\/YeX+PQSeV71yuWXm9DNldat8wfhK4jIc2uz3Jec\/dZ+0tW5FO7E3N7mnf2L6+Y3kudW5yAHL\/JFi\/1utxAxEMOJ1CEAkZc6yrLrsNe1EdZv\/vnS2Sd3\/u9c94k5J4CoIo\/t5bpSwllWUJFnn6DdWofbSeBcrchL5OXbuO5mM9dE6Ezrl1svAW7Z5Xrw0qz1BJqbb37upTvhhBNo5cqVB0SC8u1pc7u6wxn9ybdHzC7cnNhxHYLWK9ehDL91yNVO\/PIVRORxWfmw8TrN7CZUuXzrNLRbXfyKPDcu2J4VoeZn9nL94mQv1\/lLTlB8sjv6o+ZhEYDIC4sc8iWCgHMS8LoM+b\/+67+ob9++6q1T\/vxehuyMEOgQeblEpdvbsM63a\/38tu+cuN2wcYus2S+M9hJ5zsnO6ySmfeL2uoPND7f5Tim7XU7ttOn3gmPLV7fDIm6CPR\/WnN65RO32y4Ublk7\/rfbrt17Ocuwiw9kWvPhxdng\/fIURMW6\/DNn7bdC+6LTHto499tiDTkn76WPOVQL74a1cJ7Mt3NxOUue7MDuRARaFVjgEIPIqHKWokJ\/BGigBAT8IRDn5iAncD8LpTuP3mptctXS78zDdiMB7aQhA5EljBP5ERgAiLzKEmTJgn6hzbdD3uiA4F2CWSAybP1NECK6sPUpuj9hFPcSC9iGY9AriGkReBSES1fgPAhB5aA1BEfDax+n21q2fMqxITa1atWjWrFlUWFjoJxvSCEMg335adjXfaze5qmJfdg7bvoTBBHcEIgCRJ5AUuBQNAYi8aPhlNbfbYYSg+9TcsLPaIybydLesXIe0wkZpLeFYXFxcfs1RuhGC9xIRgMiTyAp8AgJAAAgAASAABIBARAQg8iICiOxAAAgAASAABIAAEJCIAESeBlbWrl2rwQpMAAEgAASAABAAAiYR4MNVWfog8iKyzQKPn9xZvHhxREvIDgSAABAAAkAACJhE4KyzziJ+RjIrYg8iL2Jreuedd6hz586q0fDt5\/iSQ4CF9kMPPQQukqNAlQweEibg38WDB\/AgAwE5Xlh9Yv78+RB5cmiR7Ykl8rLUaKQyAi5kMAMewIMMBGR4gf4ggwf2IotcIJIXsf1lsdFEhMxYdnBhDNpAhsFDILiMJQYPxqANZBg8BILLaOIscgGRF7FJZbHRRITMWPY1a9bQlClTaPjw4VS5cmVj5cBwfgTAg4wWAh7AgwwE5HiRxfkaIi9i+8tio4kImbHsP\/zwA33zzTfUqFEjiDxjKHsbBg\/eGMWRAjzEgbJ3GeDBG6O4UmRxvobIi9i6sthoIkJmLDsGU2PQBjIMHgLBZSwxeDAGbSDD4CEQXMYSb9i2h0b8dQm9dE9XytIeeoi8iE0KIi8igBqzYzDVCGYEU+AhAngas4IHjWBGMAUeIoCnMWv\/GStoxrvrqfbcmyHyNOJa4U1B5MmhGIOpDC7AA3iQgYAML9AfZPAAkSeDh9R5AZEnhzIMpjK4AA\/gQQYCMrxAf5DBA0SeDB5S5wVEnhzKMJjK4AI8gAcZCMjwAv1BBg8QeTJ4SJ0XEHlyKMNgKoML8AAeZCAgwwv0Bxk8XPnYUlr4+RbsyZNBR3q8gMiTwxUGUxlcgAfwIAMBGV6gP8jgASJPBg+p8wIiTw5lGExlcAEewIMMBGR4gf4ggweIPBk8pM4LiDw5lGEwlcEFeAAPMhCQ4QX6gwweIPJk8JA6LyDy5FCGwVQGF+ABPMhAQIYX6A8yeIDIk8FD6ryAyJNDGQZTGVyAB\/AgAwEZXqA\/yOABIk8GD6nzAiJPDmUYTGVwAR7AgwwEZHiB\/iCDh5YjFtGXm37A6VoZdKTHC4g8OVxhMJXBBXgADzIQkOEF+oMMHiyRV7NkCL319zlUVFQkwzHDXuDt2ogAQ+RFBFBjdgymGsGMYAo8RABPY1bwoBHMCKbAQwTwNGZFJE8jmFkyBZEnh20MpjK4AA\/gQQYCMrxAf5DBA0SeDB5S5wVEnhzKMJjK4AI8gAcZCMjwAv1BBg8QeTJ4SJ0XEHlyKMNgKoML8AAeZCAgwwv0Bxk81B3whnKk9tybaf78+diTJ4MW+V5A5MnhCIOpDC7AA3iQgYAML9AfZPAAkSeDh9R5AZEnhzIMpjK4AA\/gQQYCMrxAf5DBA0SeDB5S5wVEnhzKMJjK4AI8gAcZCMjwAv1BBg8QeTJ4SJ0XEHlyKMNgKoML8AAeZCAgwwv0Bxk8QOTJ4CF1XkDkyaEMg6kMLsADeJCBgAwv0B+S54FfuuDTtfzh4EXyfIjxYOzYsfTMM8\/QrFmzqLCw0NUviDwxdBEGUxlcgAfwIAMBGV6gPyTPw1urttBVY5dC5CVPhRwPli9fTl27dqVatWpB5MmhJa8nGExlEAUewIMMBGR4gf6QPA8QeclzIMqDHTt2UI8ePWjJkiXqLh1E8kTRk9MZDKYyeAIP4EEGAjK8QH9InocZ766n\/jNWIJKXPBUyPOBlWr4ssVWrVvTSSy9B5MmgxdMLDKaeEMWSADzEArNnIeDBE6JYEoCHWGDOW8jIeatp5Lw1EHnJU5G8ByUlJTRo0CCaNm0aLVy40PeevKeffvqAG7Rz7eFLvoYV1wMMpjK4BQ\/gQQYCMrxAf0iOh\/Xr16vCn1i+h\/7y+r8g8pKjQkbJ3CA6duxInTp1on79+lGQgxfOGvB+vm7dusmoWEa82L17N5WWlqpDMpUrV85IreVVEzzI4AQ8gAcZCCTnxdSpU1XAZkebwfR\/9Y6HyEuOChklDxgwgNatW0eTJ0+mgoKCQCJv1KhR1LBhw\/KKsNBANC9eXnft2kUbNmxQEVWIvHixt5cGHpLDHjzIwB48yOCBAzf8p89rZcTXqPCHK1RkcBO7F\/Zl2hYtWqjyg0TysvTgcezk+CwQyyI+gTKcDDwYBtinefDgEyjDycCDYYA9zNvvyIPIS5aLREvnKN7cuXNz+jBw4EC1hOv8cE9eorQdUDgGUxlcgAfwIAMBGV6gPyTLg\/36FIi8ZLkQVzoieeIoyesQBlMZfIEH8CADARleoD8ky4P9+pRDdm6kmiVD1O0ZvK0nC1+lsrKysixUNEwdIfLCoJZcHgymyWFvLxk8gAcZCMjwAv0hWR6ufGwpLfx8i3Ki8sZPqOCt+yHykqVETukQeXK48OMJBlM\/KJlPAx7MY+ynBPDgByXzacCDeYxzleDcj3derQ20fOowiLzkKElfydiTJ4czDKYyuAAP4EEGAjK8QH9IjgfnfryXO1anzp07Q+QlR0n6SobIk8MZBlMZXIAH8CADARleoD8kxwM\/ZcZ78vj7ad2f0Ph2lSDykqMjnSVD5MnhDYOpDC7AA3iQgYAML9AfkuHBuVR7\/RmFdEPTrRB5ydCR3lIh8uRwh8FUBhfgATzIQECGF+gPyfBgP1XLHiwbfg6t+3QZRF4ydKS3VIg8OdxhMJXBBXgADzIQkOEF+kP8PHAU76qxS8tfuWjdrDa90L8VZXG+xhUqEdtfFhtNRMiMZcdgagzaQIbBQyC4jCUGD8agDWQYPASCS0vikfNW08h5a8ptPd+vFbU5tjZEnhZ0M2YEIk8O4RhMZXABHsCDDARkeIH+EC8Pzr14VhSPvcjifI1IXsT2l8VGExEyY9kxmBqDNpBh8BAILmOJwYMxaAMZBg+B4IqUmAUen6i1Lj\/mE7WPdipWUTyIvEjQZjczRJ4c7jGYyuACPIAHGQjI8AL9IT4enMu0D3Y8gbqefXS5A1mcrxHJi9j+sthoIkJmLDsGU2PQBjIMHgLBZSwxeDAGbSDD4CEQXKETO0\/TchSPT9TavyzO1xB5oZvU\/oxZbDQRITOWHYOpMWgDGQYPgeAylhg8GIM2kGHwEAiuUIndBB4ftmChB5FXVlYWClVkgsgT1gYwmMogBDyABxkIyPAC\/cEcD7wHb813u6jDuGXlhTj34UHkQeRFaoGI5EWCT2tmDKZa4QxtDDyEhk5rRvCgFc7QxsBDaOg8Mzr34OUTeGwsi\/M1lms9m1H+BFlsNBEhM5Ydg6kxaAMZBg+B4DKWGDwYgzaQYfAQCC5fiZ2XHXMmL4EHkecLWiRyIgCRJ6dNYDCVwQV4AA8yEJDhBfqDPh5Y3P3zy23UfdpHBxhlgee2Bw\/zNREieRHbH0ReRAA1ZsdgqhHMCKbAQwTwNGYFDxrBjGAKPEQAz5aVD1fw8iwLPfvXrvgImtXzVF+FZHG+hsjz1TRyJ8pio4kImbHsGEyNQRvIMHgIBJexxODBGLSBDIOHQHC5Jr7ysaXlFxxbCTh6N+SSpnT9GYW+C8jifA2R57t5uCfMYqOJCJmx7BhMjUEbyDB4CASXscTgwRi0gQyDh0BwHZDYTdxxArc78PyUksX5GiLPT8vIkyaLjSYiZMayYzA1Bm0gw+AhEFzGEoMHY9AGMgweAsFFb63aopZlrafJ7Ln9HK7IV1oW52uIvGDt76DUWWw0ESEzlh2DqTFoAxkGD4HgMpYYPBiDNpBh8OANF++zY1HntufOitzZ36D1tuieIovzdewib+vWrTRu3Djiv8N+tWrVoqFDh4bNrjVfFhuNVgA1GsNgqhHMCKbAQwTwNGYFDxrBjGAKPLiDx8KO\/8x49xviQxVuX9TIndNmFufr2EXe+vXrqWPHjrR27drQ3aaoqIjmz58fOr\/OjFlsNDrx02kLg6lONMPbAg\/hsdOZEzzoRDO8LfBwMHaz3l9PfZ5akRNU3eLOKiiL83ViIm\/48OHUvn37wD2npKSERowYAZEXGLmKnwGDqQyOwQN4kIGADC\/QH\/bzMO\/j72jInE8PugLFYomF3fnN69KAdo0PenNWF5MQebqQzGOntLSUfvvb36o\/5557buASFyxYQI8++ijNnDkzcF4TGbLYaEzgqMMmBlMdKEa3AR6iY6jDAnjQgWJ0G1nlgQ9QrNu6m\/o89XFeEK2oHf\/Nf0x+WZyvY4\/k7du3j\/hP1apVTXIZm+0sNprYwA1YUFYH04AwGU8OHoxD7KsA8OALJuOJssRDvpOxdqBZzN16UWO68Pi6xoWdvdwsztexizzek9e5c2dq1KgR9evXj04\/\/XQ69NBDjXc0UwVksdGYwjKq3SwNplGxMpkfPJhE179t8OAfK5MpKyoP1sGJ\/\/lsE73zxVbXK0+cwq7bOQ3oV62OilXYQeSVlZWZbOBO23yq9s9\/\/jO9+OKLtGfPHqpfvz7deOON1KlTJ6pTp06crmgpCyJPC4xajFTUwVQLODEaAQ8xgp2nKPAAHnQiYD0nNu\/jjfT88lJPUcdlc8Turiua0Wk\/rZmYsIPIi1nkWYDv2LGD3njjDXr88cfpgw8+UP986qmnUt++fen8889PzXIuRJ7OYSSaLUxq0fDTlRs86EIymh3wEA0\/XbnTzAMvv373\/R6a\/NbXvkVdozr7nxuLY49dUI6yOF\/HvlzrRgofxpg9ezZNmTKF+L8LCgro17\/+NXXv3p0aNmwYlMdY02ex0cQKcIDC0jyYBqim+KTgQQZF4AE8BEWARd3rK7+j9\/61zZeos6J1LOpY3LU5tnbQImNNn8X5WoTIs1jmoOKnn35KkyZNor\/\/\/e+0c+dOatq0qYruXXXVVSKje1lsNLH2ygCFYVILAJbBpODBILgBTIOHAGAZTCqVB15+Xb1xF41+dY1vQWeJumtaHUkXHn+EyGhdPiqzOF+LEnl2cni\/3qJFi+iOO+5Q\/zxr1iwqLCw02BXDmc5iowmHlPlcUgdT8zWXVQJ4kMEHeAAPjIC1l87rdYlcaPGy62Un16fLTq6XOlHnrFMW52txIs8Sd7x0yyKPr1vhS5Pvvfde4ufMpH1ZbDTSOLD8waQmgxnwAB5kICDDizj7g1PQ8Xuw1r\/5QcO6p65v20Z0UoMC8cuvfupkT5PF+VqEyGMht2zZMpoxY0b5Mm1aTt1msdEE7VhxpY9zMI2rTmksBzzIYA08VHweeA\/dD3v30UP\/+DLQkquFDIu6NsfWoU6nF6Y+SueH7SzO14mJPN5\/x+\/XcsTu+eefp++++07tubvsssuod+\/e1Lx5c6pUqZIf3hJNk8VGkyjgeQrHpCaDGfAAHmQgIMMLHf3BupOOI3NvrdocWtBJPvkaB1tZnK9jF3m7du2iJ598kqZNm0br1q1TQq64uFjdlXfppZeqk7Vp+rLYaKTyo2MwlVq3NPkFHmSwBR7SyQMLutn\/3EBvfLIplJjjWlvXlwxu31SBIP3Ua1xMZXG+jl3k8YsXHTt2pC1btqTmmpR8DTCLjSauDhm0HExqQREzkx48mME1qFXwEBQxM+mdPNj3zb3\/5TZ6bcV39NXmHwLtnbN7yoLuohPq0tUt978kYfr9VzMoxWM1i\/N17CKPI3lr1qyhZs2aibwSJWhTy2KjCYpRXOkxqcWFdP5ywAN4kIFA8l6woPt03WZaurqU5n+5L3RkzorO8XLrTT9vQEfWqAZBF4LeLM7XsYs8K5I3fPhwdWo26FdSUkIjRoyg+fPnB81qJH0WG40RIDUYhbjQAKIGE+BBA4gaTIAHDSD6NMEHIPh78cNS+mjdjshijm1dXHwE\/bLFkeWROUTofJKRJ1kW52uIvIjtJouNJiJkxrJjUjMGbSDD4CEQXMYSgwf90HJkjg8\/PL3km0hLrFZkjv++oHld+n27xmq5Fsut+jmzW8zifJ2YyOOTtWG\/oqIiRPLCgleB82FSk0EueAAPMhAI5oV9r9yK9Tto2VfblfBiURfla1Czssre5awG1Oa4\/a9E2EVeFNvIGwwBiLxgeIVKvXXrVho3bhzx32E\/vhR56NChYbNrzZfFRqMVQI3GIC40ghnBFHiIAJ7GrODBHUzrOhK+X+75D0rV015RhZwl2njP3M2tG1K9gqrlUTnwoLFRRzSVxfk69kheRI7EZc9ioxFHwr8dwmAqgxnwAB6SRsAScpt37qWJC\/avGukScmzrylPr0yUn1iuPyOXbL4f+kHRr+E\/5WZyvIfIitr8sNpqIkBnLjsHUGLSBDIOHQHAZS1xRebD2rlnLq3OWbqDXV26KvEfOIsISbK2b1aazmtaiY+odFmmvXEXlwVjDNWg4i\/M1RF7EBpXFRhMRMmPZMZgagzaQYfAQCC5jidPOgyXilq3dTvM+2qhlf5ybkDu9cU067sjDIwm5fCSmnQdjDTQBw1mcryHyIja0LDaaiJAZy47B1Bi0gQyDh0BwGUssnQdrSZUBKPl4Iy39aru2aBzbtCJyFx5fl65udSRVokrGhBxEnrFmrNVwFudriLyITSiLjSYiZMayS5\/UjFVcmGHwIIMQCTxY0biX\/3cjvfRhqVYRZwk5PuxwfvM6dFbT2gp4adeQSOBBRotM3ossztcQeRHbXRYbTUTIjGXHYGoM2kCGwUMguIwlNsmD\/boRrsALH5TSx9\/sMCLi2D7vj2vZqCYVF+5fVrVH6owBqMmwSR40uZgZM1mcr8WIvLKyMtq8eTPt27eP6tatS4cccgjt3btX\/NNnWWw0UkcEDKYymAEPFYMH+3Lqqyu+o39+uc2YiONoHB9yOL95XZHRuCiMoj9EQU9v3izO14mLPBZ3r776Kt1xxx1UWlpKfNHxrFmzqFq1anTzzTfT6aefTgMHDhQr9rLYaPR2O33WMJjqwzKKJfAQBT19eb14sJ7i+n7PPnrxg1Ja852e++KcNeDIm\/QlVX2oH2zJiweTZcP2gQhkcb5OXOT9z\/\/8D\/Xu3ZtOO+00Ou644+iNN95QIq+goIBuv\/12mjdvHvE7t127dhXZXrPYaEQSQUQYTGUwAx6S48FaRmUPvvh2B23cWEpLN1amD77eoZzScVecvXbW0qmbiEvTkqpJxtAfTKIbzHYW5+tERd7u3bupT58+9OOPP9L48eNpwYIFNGLECCXyCgsLac+ePTRgwADauHEjTZ48WQk\/aV8WG400Dix\/MJjKYAY8mOXBvoy6aedeevOTTfTZtzu1L6XaRdp+EVeXGtSupqJyadsXZ5aR\/NbRH5JEH5G8REXe+vXrqWPHjtSrVy\/q0qULlZSUHCDymJ7p06fTxIkTy4WfnOay3xOIPDmMYDCVwQV4iMaDJeJ+LCujV\/53I\/3vOv0HGiwPLbF23nF1qF3xEVTnsCrqR9JOqEZDNNnc6A\/J4m8vPYvztQiRd8MNN1DPnj1dRd7YsWNp9uzZNHPmTKpfv76c1vJvT7LYaMSR8G+HMJjKYAY8HMyDfRmV\/7tR3Z\/Qk++so8WrtxqJwLEHDWpWVo40rV9AZzetRedVwEMNMlo8Inlp4CGrQZlERZ61XLtz506aNGmSiorZl2vXrl1LN954IzVq1Egt5\/JhDGkfRJ4cRiAuZHCRVR6sCNyXm3ap6NuHX5uLwFnRNv6brxc5qUEBndqwRnkUjv\/jyMOIvvnmGzV+Vq68X\/Dhix+BrPaH+JH2LjGL83WiIo8pWbp0qVqu5UMX\/Ofll1+mW2+9lVatWkV\/+9vf1L68CRMm0HnnnefNYAIpsthoEoDZV5EYTH3BZDxRReTBOonK4L224jt638B1InZirGXUxnWr0y9b1KfmRx1eLuD3y7o3AAAgAElEQVSsn3kRWRF58KqzxJ+DBzmsZHG+TlzkMf1vv\/22ukJl9erVB7QGXp695557qF27dnJaicOTLDYaqWRgMJXBTFp4sC+hMnKfrP+eSlZ8RyvXf29sCZXLsZ9I5Se3zmhSq1zA2X8elc208BC1ntLzgwc5DGVxvhYh8rgJ8H15fIr2448\/Jl7GPemkk9QJ20MPPVROC3HxJIuNRiohGExlMCOFB\/sp1EVfbFH3wPG\/fbX5B\/W3ic8u4E5pWECnNKyR2GlUKTyYwDlNNsGDHLayOF+LEXlymkEwT7LYaIIhFF9qDKbxYZ2vpDh4sC+fzv9sM7GI40\/3PXD2etoFXIuiGnTpSfWMROB0sRgHD7p8rch2wIMcdrM4Xycq8rZu3Urjxo0j\/tvrq1evnlq2Pfnkk0VF97LYaLy4SurnGEyTQv7AcsPw4Dx9yoJqyZqttOCzzfTFxl1Go2\/svV3AXXvaUdSs\/mGiBZwfpsPw4Mcu0gRDADwEw8tk6izO14mKPH7G7Le\/\/S199NFHxCds+WMxxx8v3bp95557Lj3yyCNUs2ZNk23Bt+0sNhrf4MScEINpzIDnKM6NB\/vS6d59P9KCVZvp3TXblIW4om+N6\/6ETi2qQScevf9S9Yp+oS\/6g9z+IMOz7HmRxfk6UZHHTWzJkiXUv39\/9WzZTTfdVP6qBZ+q\/etf\/0p8T96jjz5KJ554Ij3\/\/PPqihVOe9ttt4looVlsNCKAd3ECk1oyzNgF3PK12+nvH21UT8yV7iJje9+c0bfiow+nK0+tT5WoUrl4s6dJBplkS0V\/SBZ\/q3TwIIMH9iKL83WiIm\/Hjh3Uo0cPat68Od19991UqVKlA1oDH8a488476V\/\/+pe6J6969eo0cuRIRdSzzz4rouVksdGIAB4izxgNzqVTLujtz7fQvzaZP7hgVcp61L7JEdXpohPqUr2CquXRN79XiBgDKCWGIS5kEAUeZPAAkZcAD9azZhzJu+6661w94JcuHnvssfJnzebMmUMPPvggzZ8\/PwGPDy4SIk8EDcoJDKb+uLBH3jZs300LV20x9vap0yP73rdWjWpQ8dEFB5w+zXr0zR+D\/lKhP\/jDyXQq8GAaYf\/2szhfJxrJ27x5M3Xv3p2aNWtG9957L1Wtuv+3devjJdthw4bR559\/To8\/\/jjVqVNH7cfjC5NfeeUV\/8waTJnFRmMQzkimMZjuXx7lP5t37qXXVn5HX5SaP7RgF2bq8fraVahJjR\/pjOYN1EsLFX3vW6RGazAz+oNBcAOYBg8BwDKcNIvzdaIij\/mcOHEijR49mn7961+rQxh8Nx5\/HOXjvXjPPPOM2n\/Xt29ftX\/v9ttvV3fosdiT8GWx0UjA3c2HijaYui2b8ksLn2743vh9bxa+9sgbnzhtV1yXalfP\/4h9ReNBanv38gs8eCEUz8\/BQzw4+ykli\/N14iKPo3Us8p544gnat2\/fATzxRch8GGPgwIFqKa5nz560bt06JQyLi4v9cGo8TRYbjXFQQxaQtsHUvmz69ZYf6O0vtiQTeav7Ezr2yMOosGY1LRf3po2HkM1NfDbwIIMi8CCDB\/Yii\/N14iLPop+vU3n99ddp+fLl6p84WnfBBRdQw4YN1f\/v2rVLRfcaNGhA1apVE9NqsthoxIDvcETSYGpF4fiR+lf+d6M6tGDypQU7FNahBRZuZzSpST+tU139OK5lU0k8SG2rcfgFHuJA2bsM8OCNUVwpsjhfixF5cZGsu5wsNhrdGOqyZ3IwdS6dvrriO\/rnl\/vveYtDvNmXTRsfUZ2ubnkkHXfk\/gt7rU\/KqVOTPOhqK1mwAx5ksAweZPDAXmRxvk5c5PE1KV988QXNnTvX9QJkfsf266+\/poceeqh8v56cJpPNRiMJf7svYQZT+5Ip2+I9b6+t+C4R8cYijYXbUZqWTZPiKQwPSflakcsFDzLYBQ8yeIDIS4gHPik7YMAA4r15bh+fuD399NPVQQs+XSvty+JvBtI4sPxxG0ytCNy8jzfSCx+UUllZ\/JG34wsPp\/OOq0N1DvvPgQX2WUrkTTefmNR0IxrOHngIh5vuXOBBN6Lh7WVxvk40kvf9999T79691WGKhx9+mBo3bqwOV7Rq1Uq9gjFjxgyaMmWKugj5lFNOCc+swZxZbDQG4fRt2hJv\/1i5if751TZaHcP7pnZhxleFnNiggNoXH0HVKh+i\/I5rz5tvkBJKiEktIeAdxYIH8CADATleZHG+TlTkWZch8\/Upt9xyi2oJd911F3377bdqeZY\/jvLxQYtRo0Yd9CKGhKaTxUZjGnf7EuqiL7bQ\/M82G9\/3Zt\/zxkumJzcsoOZHHg7xFoJsiIsQoBnIAh4MgBrCJHgIAZqhLFmcr0WIvFtvvZWuueYaRev06dPphRdeoEmTJlHNmjXV\/z\/11FP05JNPUr169XxTz+KQ9\/lZH0cD27dvnzc\/n+zld3G3b99+QLozzzyTJk+eXP6urv2HWWw0vknIk9AScl9u2kUz3l1vTMTZxRu\/rtDuhLp0WNVDy8VbRV0y1cFRWBuY1MIipzcfeNCLZ1hr4CEscvrzZXG+TlTkbdu2TS3PnnbaaTRkyBDF6GuvvaaieSzumjRposQeC7xZs2b5PnjBAu+9994rz1NSUkJ9+vRR9+3169cvZ8vhdIMGDaJp06ZRixYtfLWwLDYaX8D8O5G1rDqqZA2t+W4XLfx8S5DsedPaBRw\/Tn9MnUNpy5YtdFrzRge8tKCtQBjyhQAmNV8wGU8EHoxD7KsA8OALplgSZXG+TlTkMasPPPCAugh58ODB1KlTJ7U\/r0uXLuoFjF\/84hc0dOhQ4hO41rNmXi3Bisbx8q49csfCj23nisix3bFjx6o3cfOlcZafxUaTiwMWdFMXraMla7ZGFnPON07bn1jvgIMKbhE4DKZevSOen4OHeHD2KgU8eCEUz8\/BQzw4+ykli\/N14iKPo3m8H4\/\/ZiFXu3ZtGjFihDpwweKuUqVKdOedd6pl1CifH5HHafgbM2aM76Ky2GgYHBZ0HJV7esk3oQWddWlvi6IadOlJ+5fi+d\/CLqFiMPXdbI0mBA9G4fVtHDz4hspoQvBgFN5AxrM4Xycu8pghFnO8D65GjRpK1PHzZm+\/\/TYtWLCALrnkErWcy\/8e9uMIHT+dlm9fnnUIpKCggFauXFleVIcOHfKKviw1mrdWbaGR81YHFnUs2lo3q00dTy+kQytViiTk8rUBDKZhe4jefOBBL55hrYGHsMjpzQce9OIZxVqW5msLJxEiLx9pLPi2bt1KtWrVIn7LNshn7cXjPF5izVrmveiii8pFnSX8+Ck1r4MXTz\/9NBUVFZW7V1hYGMRVsWn\/unSj72hdg5qVVT2uaXUk3XjO\/ufowkblwgCCwTQMavrzgAf9mIaxCB7CoKY\/D3jQj6lfizyH27+1a9dS586d1bYs+3zt114a0yUq8iwRNXz48JwnX1966SUaOXJkoIMXTiJ27NhBPXr0UHvyghzgYDuWUMwVBbR+M3CWycvL3bp1S2OboPe\/\/oEmLN6i\/s73sag7s9FP6LLjC+joGpXJEnlJVZpfR+E3kFlgV668X3Diix8B8BA\/5m4lggfwIAOB5LyYOnWqOkjp\/CDyDHLCAw8vxe7cuVOdhHzwwQfp6quvdj3Nyml5nx6nnTlzJtWvXz+0Z1akji9fznfC1lmAVz5L5PFBj4YN90ev+GOhkbZo3uMLv6bhL32ZE2NrD91tFzVSEbqkRZ3T0V27dtGGDRvUb2gQeaG7SuSM4CEyhFoMgActMEY2Ah4iQxjaAAeS7NG8xYsXqzt4IfJCQ+ovIyvru+++W+3F8\/p4iZavNeGrVqLsy\/MSa7n88MpXEdb4x7z2Lxrx8heuEFj76a4\/42hqc2xtL7oS\/TmWRRKFv7xw8AAeZCAgwwv0Bxk8sBcVYb4OimYiy7X8Ti3vs+OoC99f97vf\/Y7OPffcg3znd2v5vdog4i7XXXdey665fu51d16aG834+Wtp2NzPcoq7RzsVixd2ducxmAbt\/mbSgwczuAa1Ch6CImYmPXgwg2sYq2mer8PUl\/MkIvIsZ\/lQxaZNm9TLFvx0mY7P2n\/HtqzDElY0rri4OOcBCrd9e26HMZw+pq3RWFef9J+x4iC4OWrX7ZwG9KtWR8V6YEIH72wDg6kuJKPZAQ\/R8NOVGzzoQjKaHfAQDT+dudM2X+uoe+wizxJ2\/Lffj5ds69atG+h0rfNZM+drF3ytyoQJEw563cIrX9pFXssRi9Qdd\/aPxd391zSn9ice4ZcSkekwmMqgBTyABxkIyPAC\/UEGD+wFRF4MXFgnavkos9+PN9IHPRXr13bUdGlpNPw+rFv07ncX\/pRu+nnDVEbunNxhMI3amvXkBw96cIxqBTxERVBPfvCgB0cdVtIyX+uoq2Uj9kgenzRauHAh8clZvx8v5bZu3ZqqV6\/uN0ts6aQ3Go7azf7nhoMOVnD07vl+rSqEuLPIxmAaW7PPWxB4AA8yEJDhBfqDDB4QyZPDQ6o8kSzyWOBdNXbpAcuzLO4e6XQCnXtsnVTh7MdZDKZ+UDKfBjyYx9hPCeDBD0rm04AH8xj7LUHyfO23DkHTxR7Jy+Xg5s2b6a233qL333+f+FRtq1at6Oyzz1anayV\/UhtNLoFX0aJ39raBwVRGTwEP4EEGAjK8QH+QwQMieQnxwAcwJk6cqJ4Scx7G4AMX\/fv3V5cXs\/CT+EkUeSzw+ICF\/auIy7PO9oDBVEYPAQ\/gQQYCMrxAf5DBA0ReQjzMnj2bhgwZQm3btqXbbruNmjRpojxZs2YNPfDAA+p1DH5N4qqrrkrIw\/zFShN5bgLvD79oSrddvB\/XivxhMJXBLngADzIQkOEF+oMMHiDyEuDBupuuoKCAHnnkkYMOVvAhjVtuuUVF+PjtWF136emsqiSR57ZEO7dvS2p7nOwlb118YDDVhWQ0O+AhGn66coMHXUhGswMeouGnM7ek+VpnvfLZSnRPnnWdSq9evahLly6ufk6fPl0t5+IKlfxNwk3gPXzdCdTlrKPjakuJl4PBNHEKlAPgATzIQECGF+gPMnhAJC8BHkpLS+m6666jq6++WkXs3D6O8D377LM0c+ZMql+\/fgJe5i9Swm8GLPD4DryFn28pd3bUr5rTza0bisPLpEMYTE2i6982ePCPlcmU4MEkuv5tgwf\/WJlOKWG+Nl1Hp\/1EI3l79+4lfmFixYoVNGnSJGratOkB\/q1evZp69uxJ\/BwZH8yoUqVK3Ph4lieh0Yz7n6\/oD8+tKveVo3ccxcvah8FUBuPgATzIQECGF+gPMnhAJC8hHj788EPq3r27uhz54osvVpce88cXJr\/66qtqHx4v1\/KVKhK\/pEWe8yULPkW7bPg5EqEy7hMGU+MQ+yoAPPiCyXgi8GAcYl8FgAdfMMWSKOn5OpZKOgpJNJJn+fLJJ5\/QsGHDaNmyZVRWVqb+uVKlStSyZUu699576fjjj08CG19lJtlonCdps3BNSj5SMJj6arLGE4EH4xD7KgA8+ILJeCLwYBxi3wUkOV\/7dlJzQhEiz6oTR\/P4UmT++BJkiadpnfgn2WiufGzpAfvwHru+mK4\/o1BzE0mPOQymMrgCD+BBBgIyvEB\/kMEDe5HkfJ0UComKPD548fjjj6s78Jo3b058+XHavqQajXOZ9vcXNaY7Lj8mbfBp9ReDqVY4QxsDD6Gh05oRPGiFM7Qx8BAaOu0Zk5qvtVckgMHERR6fruWLj4844ggl9m688UYqKipSy7Vp+JJoNG7LtFndh2dvIxhMZfQY8AAeZCAgwwv0Bxk8IJKXEA979uyhRYsW0ZQpU9Tf\/P98yta6WkXitSl2qJIQeb8cu4wWrNq\/rM0fCzzej5f1D4OpjBYAHsCDDARkeIH+IIMHiDwBPDgFH1+xwten8OnbK664QuT7tXGLvLdWbaGrxi4tZ+vWixrTHzO+TGuBgcFUQCfGZcgySAAP4EEMAnIciXu+llDzRJdr8wGwbds2euKJJ1SEr0aNGnjxgoiclx5n\/TSts\/1A5EkYUvDihQwWwAN4kIKAHD8g8hLmgt+yfeONN5SgYzL4O\/XUU1Ukj+\/Qq1q1asIeHlx8nI3Gedji+X6tqM2xtcVhkpRDEHlJIX9gueABPMhAQIYX6A8yeMBybUI8OIXdvn371J68m266iS6\/\/HJ1lYrkL06RZ78yJcuXHudqDxhMZfQU8AAeZCAgwwv0Bxk8QOQlwMP69eupY8eOtHbtWvUu7Q033KDesW3YMD1vrsYl8hDF826gGEy9MYojBXiIA2XvMsCDN0ZxpAAPcaDsr4y45mt\/3sSTKtE9eXzx8YwZM+iSSy6hY445JjXXptipiavRIIrn3SEwmHpjFEcK8BAHyt5lgAdvjOJIAR7iQNlfGXHN1\/68iSdVoiIvniqaLSWORuOM4uHKFHdOMZiabet+rYMHv0iZTQcezOLr1zp48IuU+XRxzNfmaxGsBIi8YHgdlDqORoMonj+SMJj6w8l0KvBgGmF\/9sGDP5xMpwIPphH2bz+O+dq\/N\/GkhMiLiLPpRuO8Fw9RvNyEYTCN2Jg1ZQcPmoCMaAY8RARQU3bwoAlIDWZMz9caXNRuAiIvIqSmGw2ieP4JwmDqHyuTKcGDSXT92wYP\/rEymRI8mEQ3mG3T83Uwb+JJDZEXEWeTjcb5Ri3uxctPFgbTiI1ZU3bwoAnIiGbAQ0QANWUHD5qA1GDG5HytwT0jJhIVeVu3bqVx48apJ8tOPvlk1wq+\/\/779OCDD9KYMWPUNSvSPpON5s1PN9E145erKuN1C2\/mMZh6YxRHCvAQB8reZYAHb4ziSAEe4kDZXxkm52t\/HsSfKlGRZ92TN3z4cGrfvv1BteeLkUePHk0vvfRSJp81azlikXrKjL+Li4+gmT1Pjb+FpKhEDKYyyAIP4EEGAjK8QH+QwQN7AZEXAxd79uyhYcOG0Zw5c3yXduGFF9IjjzxC1atX950nroSmGo3zwAWWar0ZxWDqjVEcKcBDHCh7lwEevDGKIwV4iANlf2WYmq\/9lZ5MqkQieatXr6apU6cSX4bMb9W2bNnS9ZWLGjVqUKtWrahNmzbE\/y3xM9VocOAiONsYTINjZiIHeDCBanCb4CE4ZiZygAcTqIazaWq+DudNPLkSEXlW1fzsyYsHhvClmGg0OHARjg8MpuFw050LPOhGNJw98BAON925wINuRMPbMzFfh\/cmnpyJirx4qmi2FBONBu\/UhuMMg2k43HTnAg+6EQ1nDzyEw013LvCgG9Hw9kzM1+G9iSdn7CLPit5x9X7zm9\/QU089Rfxv+b5atWpR3759if+W9ploNPal2tbNatML\/VtJq7ZIfzCYyqAFPIAHGQjI8AL9QQYP7IWJ+VpO7dw9iV3kWSdq2R0+THHLLbfQ2rVr8+JUVFSUmdO1zqXax64vpuvPKJTejkT4h8FUBA0EHsCDDARkeIH+IIMHiDw5PKTKE92\/GdiXavluPH7GDJ8\/BDCY+sPJdCrwYBphf\/bBgz+cTKcCD6YR9m9f93ztv+TkUsYeyUuuqmZK1t1osFQbnicMpuGx05kTPOhEM7wt8BAeO505wYNONKPZ0j1fR\/MmntyJijxrfx725O0nG0u10Ro9BtNo+OnKDR50IRnNDniIhp+u3OBBF5LR7UDkRccwkAVrfx725O2HzXmqdtOYCwLhmfXEGExltADwAB5kICDDC\/QHGTywFxB5QrgoKyujjRs30jPPPEPPPfecers219u2Sbuss9FgqTYamxhMo+GnKzd40IVkNDvgIRp+unKDB11IRrejc76O7k08FhJdrvWqIou9O++8U72MMWbMGKpSpYpXlth\/rrPR1B3wRrn\/Qy5pQkMuaRp7fdJcIAZTGeyBB\/AgAwEZXqA\/yOABkTw5PBzgycyZM+mxxx6r8FeoON+q5VO1fLoWn38EMJj6x8pkSvBgEl3\/tsGDf6xMpgQPJtENZltnUCZYycmlFh3J27NnDw0bNow++ugjevLJJ6levXrJIZWjZF2NZsicz2jSW\/vvC8TVKeFoxmAaDjfducCDbkTD2QMP4XDTnQs86EY0vD1d83V4D+LPmajI8zpd+8UXX9B7771HXbt2pT\/+8Y9UqVKl+BHyKFFXo7Hvx\/t9u8Z0x2XHiKurdIcwmMpgCDyABxkIyPAC\/UEGD+yFrvlaTo28PUlU5Hmdrq1atSpdc801NHToUKpZs6Z3bRJIoaPROK9Oeb5fK2pzbO0EapPuIjGYyuAPPIAHGQjI8AL9QQYPEHlyeEiVJzpE3sh5q2nkvDWq3liqDU8\/BtPw2OnMCR50ohneFngIj53OnOBBJ5rRbOmYr6N5EH\/uRCN5uarLp2q3b99ONWrUELlEa\/dbR6PB1Sl6Gj4GUz04RrUCHqIiqCc\/eNCDY1Qr4CEqgvry65iv9XkTj6XERR4frvjLX\/6iDleMHTuWCgoK1JUp3bt3p3Xr1tEf\/vAHuvLKK8WKPR2NBlen6GnsGEz14BjVCniIiqCe\/OBBD45RrYCHqAjqy69jvtbnTTyWEhd5EydOpNGjR6u9d8OHD1cib9euXTRv3jyaPHkyrVq1St2Rd9lll8WDSMBSojYa59Up2I8XkABbcgym4bHTmRM86EQzvC3wEB47nTnBg040o9mKOl9HKz2Z3ImKPCtid9xxx9E999xz0GXHHOUbMGCAev2CBR8LQGlf1EZjf8oM+\/GisYvBNBp+unKDB11IRrMDHqLhpys3eNCFZHQ7Uefr6B7EbyFRkWedru3fvz9dd911rrWfM2eOetZs1qxZVFhYGD9CHiVGbTTYj6ePUgym+rCMYgk8REFPX17woA\/LKJbAQxT09OaNOl\/r9SYea4mKPI7Q3XDDDXT++efTkCFDXGs8cuRIevPNNyvsZcjYj6evoWMw1YdlFEvgIQp6+vKCB31YRrEEHqKgpzcvRJ5ePD2t8SnaP\/3pT\/Tiiy\/S\/fffr8SedeEx\/4zF3eDBg+mKK66okJch4ykzzyYSKAEG00BwGUsMHoxBG8gweAgEl7HE4MEYtIENQ+QFhix6hrVr11KvXr1o5cqV6sJjvjaFv927d6u9eCeccALx4YyioqLohRmwEKXRYD+eXkIwmOrFM6w18BAWOb35wINePMNaAw9hkdOfL8p8rd+beCwmulxrVZFP0\/KeO\/6zbds29c8s+Dp27Kj+VK9ePR40QpQSpdFgP14IwPNkwWCqF8+w1sBDWOT05gMPevEMaw08hEVOf74o87V+b+KxmKjI4yXZ5557jk466STiE7Zp\/KI0mpYjFhE\/acbfkEua0JBLmqYRAjE+YzCVQQV4AA8yEJDhBfqDDB7YiyjztZxaBPMkUZFXWlqqTtVee+211K9fv2CeC0kdttHgvVr9BGIw1Y9pGIvgIQxq+vOAB\/2YhrEIHsKgZiZP2PnajDfxWE1U5Fmna\/lFi6yJPOehi01jLoiH8QpcCgZTGeSCB\/AgAwEZXqA\/yOABkbyEeHjttdforrvuUhG9yy+\/nA477LCDPDn00EOpbt26xH9L+8L+ZjBl0Toa8NdPVHVwCbIeVjGY6sExqhXwEBVBPfnBgx4co1oBD1ER1Jc\/7Hytz4P4LSUaybMuQ+YTtvk+Pllb0S5DxqEL\/Y0dg6l+TMNYBA9hUNOfBzzoxzSMRfAQBjUzeSDyzOCa0yqfql24cKG6LiXfV61aNWrdurXIU7ZhGo1zPx4OXehpeBhM9eAY1Qp4iIqgnvzgQQ+OUa2Ah6gI6ssfZr7WV3oylhKN5CVTZb2lhmk0zv14z\/drRW2Ora3XsQxaw2Aqg3TwAB5kICDDC\/QHGTywF2Hmazneh\/MEIi8cbuW5wjSakfNW08h5a5QN7MeLSIAtOwZTfVhGsQQeoqCnLy940IdlFEvgIQp6evOGma\/1ehC\/tdhFnrUPj6v6yCOP0C233EJZ25PXf8YK4tcu+GvdrDa90L9V\/MxXwBIxmMogFTyABxkIyPAC\/UEGD4jkxcTD1q1bady4caq03\/zmN\/TUU08R\/1u+r1atWtS3b1\/iv6V9YX4zsB+6wH48fYxiMNWHZRRL4CEKevryggd9WEaxBB6ioKc3b5j5Wq8H8VuLPZIXfxXNlhi00TgPXTx2fTFdf0ahWSczYh2DqQyiwQN4kIGADC\/QH2TwgEheQjzw02avvPIKzZ8\/n+644w46\/PDD1fu1fDlylSpVaOjQoXT88ccn5J13sUFFHg5deGMaNgUG07DI6c0HHvTiGdYaeAiLnN584EEvnlGsBZ2vo5QlJW\/ikbyXX36ZBgwYQC1atKDx48dTnTp11PLt\/fffTy+88ALx9SkTJ06kVq1k7lsL2mhw6MJc08dgag7bIJbBQxC0zKUFD+awDWIZPARBy2zaoPO1WW\/isZ6oyNuxYwf16NFD3X\/HhzAKCgoOqPXmzZupT58+6hUMFoAs+KR9QRuNXeTh0IVeNjGY6sUzrDXwEBY5vfnAg148w1oDD2GR058v6Hyt34P4LSYq8qyTtr169aIuXbq41n769OkqkldRXrzASxfmGjkGU3PYBrEMHoKgZS4teDCHbRDL4CEIWmbTQuSZxfcg66WlperN2quvvlpdpeL2jR07lmbPnk0zZ86k+vXrx+yhd3FBG03dAW+UG8XJWm98g6TAYBoELXNpwYM5bINYBg9B0DKXFjyYwzao5aDzdVD7EtMnGsnbu3ev2o+3YsUKmjRpEjVt2vQAjFavXk09e\/ak4uJiGjNmjDqIIe0L0micJ2vx0oVeNjGY6sUzrDXwEBY5vfnAg148w1oDD2GR058vyHytv\/RkLCYq8rjKH374IXXv3p22b99Op512GjVu3FghsXHjRlqwYAHVqFGDHn\/8cTrllFOSQcij1CCNxnmydtOYC0TWKa1OYTCVwRx4AA8yEJDhBfqDDB7YiyDztRyvo3mSuMhj93UPrXQAACAASURBVL\/++mu677776LXXXqM9e\/aoGlWtWpXatWtHt99+OzVs2DBaLQPk5sji3Llzy3PwgY\/27dvntBCk0fArF\/zaBX94ziwAKT6TYjD1CZThZODBMMA+zYMHn0AZTgYeDAMcwHyQ+TqAWdFJRYg8KQixwHvvvffKD3mUlJSo070DBw5U9\/a5fUEaDQ5dmGUag6lZfP1aBw9+kTKbDjyYxdevdfDgFynz6YLM1+a9iacEiLx\/47x8+XLq2rUrjRo16oDIHQu\/devW0eTJkw+64oWzBmk0eM7MbKPGYGoWX7\/WwYNfpMymAw9m8fVrHTz4Rcp8uiDztXlv4ikBIs8DZ10iD4cuzDdoDKbmMfZTAnjwg5L5NODBPMZ+SgAPflCKJw1EXjw4p6YUvr5l9OjR6iLmXPvyrEbz9NNPU1FRUXndCgsPfI\/2nTU76KqxS8t\/PqfXKdTm2NqpwSINjmIwlcESeAAPMhCQ4QX6Q3I88F289m\/t2rXUuXNn9Yyqfb5OzkPzJSOS54KxtRePf9ShQwd1fUuuzxJ5zp\/z0m+3bt3K\/\/n9r3+gXnP+0+Dev6WJeXYzVsLu3buJ715kgV25cuWM1V5OdcGDDC7AA3iQgUByXkydOpWmTZt2kAMQeclxIqpk69k13pOX68UNS+TxXj77KWAWGvZo3gOvf0UPvL5W1Y9P1r4zUOZbvKIICOjMrl27aMOGDeo3NIi8gOBpTA4eNIIZwRR4iACexqzgQSOYAU1xJM8ezVu8eDE99NBDiOQFxLFCJ7cOZPTu3dv1hK3fNX6crDXfTLAsYh5jPyWABz8omU8DHsxj7KcE8OAHpXjS+J2v4\/EmnlJELNfyyxZ\/\/\/vf6csvv3Stda1atahv377Ef8f9mRB5eM7MDIsYTM3gGtQqeAiKmJn04MEMrkGtgoegiJlLD5FnDtucll9++WX1tJl1CbJbQl5+y7Vcqstl3oc3aNAgtX7fokWLcrPW\/rxchy\/8NBrnydplw89RS7b49CKAwVQvnmGtgYewyOnNBx704hnWGngIi5z+fH7ma\/2lJmsx0Uieteft22+\/pYcffphOOukkqlSpUiKIWL5w4dadeFYUj9\/OjXJPHq5PiYdSDKbx4OxVCnjwQiien4OHeHD2KgU8eCEU388h8uLDWpXEGyI7duxIN9xwA\/Xs2TPm0t2Lcz5rlu+1C7bgp9Hgzdp4qMVgGg\/OXqWABy+E4vk5eIgHZ69SwIMXQvH93M98HZ838ZSUaCRv8+bN1L17d7rsssvEiLygsPtpNHizNiiq4dJjMA2Hm+5c4EE3ouHsgYdwuOnOBR50Ixrenp\/5Orx1mTkTFXkMycSJE+mVV15Rf9evX18mSnm88tNocLI2HloxmMaDs1cp4MELoXh+Dh7iwdmrFPDghVB8P\/czX8fnTTwlJSry+P6gN998k5544glauXIltW3blmrUqHFQzZM8XetFg59GgzdrvVDU83MMpnpwjGoFPERFUE9+8KAHx6hWwENUBPXl9zNf6ytNhqVERZ61J4+fGsn3xXG6NiwdfhpN3QFvlJvH9SlhkfbOh8HUG6M4UoCHOFD2LgM8eGMURwrwEAfK\/srwM1\/7s5SeVImKvPTAlNtTr0aDk7XxsYzBND6s85UEHsCDDARkeIH+IIMH9sJrvpbjqT5PIPIiYunVaJwna3FHXkTA82THYGoO2yCWwUMQtMylBQ\/msA1iGTwEQctsWq\/52mzpyVgXIfI+\/\/xzGjFiBC1atIiOPPJIdfHxYYcdRkOHDqVLL72UrrzyysTuz\/OixavR2EUeX4DMIg+fGQQwmJrBNahV8BAUMTPpwYMZXINaBQ9BETOX3mu+NldycpYTF3kffvihukblkEMOoSZNmtA333yjRF61atXUv69YsUJdlNy+ffvkUMpTslejGTlvNY2ct0ZZgMgzSyEGU7P4+rUOHvwiZTYdeDCLr1\/r4MEvUubTec3X5j2Iv4RERd7evXvVk2Z88IKvUFm6dKmK6FlPmG3btk3dn8dRPX5WjIWftM+r0eD6lPgYw2AaH9b5SgIP4EEGAjK8QH+QwQN74TVfy\/FUnyeJirzS0lK67rrr6Nprr6V+\/foRvxNrF3lczUmTJtGTTz5p\/O3asJB6NRpcnxIW2eD5MJgGx8xEDvBgAtXgNsFDcMxM5AAPJlANZ9Nrvg5nVXauREWedYVKr169qEuXLq4ib\/r06SrKZ0X3pMHp1Wjs16c8dn0xXX9GobQqVBh\/MJjKoBI8gAcZCMjwAv1BBg+I5CXAw44dO6hHjx5Ur149GjNmjLoY2W25tkqVKjRhwgQ6\/PDDE\/Ayf5H5RB6uT4mXLgym8eKdqzTwAB5kICDDC\/QHGTxA5CXEw8svv0wDBw5UkbzGjRvTuHHj1P473qf36KOPqoMXI0eOVEu6Er98Ig\/Xp8TLGAbTePGGyJOBN3gAD7IRkOOd18qbHE\/1eZLoci1Xo6ysjKZNm0ajRo2inTt3HlCzQw89VB3M4OVc\/m+JX5BI3qYxF0isQoXxCSJPBpXgATzIQECGF+gPMnhAJC9hHvgk7ZIlS9SfPXv20M9+9jNq06YN1alTJ2HP8hefT+Th+pR4qcNgGi\/eiCDJwBs8gAfZCMjxDpE8OVykxpN8jab\/jBU04931qi6tm9WmF\/q3Sk290ugoRJ4M1sADeJCBgAwv0B9k8IBIXoI88JLtp59+Sn\/7299o+\/btyhM+jHHNNddQ06ZNE\/TMu+h8Is9+fQqfquXTtfjMIYDB1By2QSyDhyBomUsLHsxhG8QyeAiCltm0iOSZxdfVOi\/T\/uEPfyA+gMFiz\/5VqlSJOnXqRHfeeSdVrVo1Ae+8i8zXaFqOWER8wpa\/IZc0oSGXyBas3rWVnQKDqQx+wAN4kIGADC\/QH2TwgEheAjywqOMDF48\/\/jj179+funXrRjVr1lSesPjji5D5jrzbbrtNHb6Q+OUSebg+JX62MJjGj7lbieABPMhAQIYX6A8yeIDIS4CHjRs30g033EBnnHEG3X333cSRO\/vHIpCjePy+LQtBiYcwIPISaDg5isRgKoML8AAeZCAgwwv0Bxk8QOQlwIP14gVH8fh5M7fvhRdeUNG+tL14gTvy4m9QGEzjxxyRPBmYgwfwIBcBOZ5hT17MXOzevZtuueUWtd+OX7xw7rvjq1SGDRumlm4feeQRqlatWsweehfnJ5L307o\/oWXDz\/E2hhSREIDIiwSftszgQRuUkQyBh0jwacsMHrRBGdkQRF5kCIMb4JcteL9do0aNaPDgwdSkSRM65JBDiKN8\/OJFSUkJ3XfffXTyySeXG+eLkevXrx+8MAM5cjUa3JFnAGwPkxhM48ccESQZmIMH8CAXATmeQeTFzIW1XMtCL8hXVFRE8+fPD5LFWNpcjcZ+fQruyDMG\/wGGIfLiwdmrFPDghVA8PwcP8eDsVQp48EIovp9D5MWHtSpp165dtHDhQuJl2yAfL9u2a9cuSBZjaf2IPFyfYgx+iLx4oA1UCia1QHAZSwwejEEbyDB4CASX0cQQeUbhrZjGczUa+x15fAkyX4aMzywCGEzN4uvXOnjwi5TZdODBLL5+rYMHv0iZTweRZx7jnCVs3ryZ3nrrLXr\/\/ffVAYxWrVrR2WefLfLaFHsl3BoN7shLpiFhME0Gd2ep4AE8yEBAhhfoDzJ4YC8g8hLgYt++ferCYz5dy\/9t\/\/iABV+v0q9fv1S9eAGRl0BDIiIMpsngDpEnA3fwAB5kIiDHK4i8BLiYPXs2DRkyhNq2batetuDTtfytWbOGHnjgAXr77bfVPXlXXXVVAt55F+nWaHBHnjduJlJA5JlANbhN8BAcMxM5wIMJVIPbBA\/BMTOVAyLPFLI57O7YsYN69OhBBQUF6h686tWrH5CSD2bwPXoc4Rs\/fnxq7smzizzckRdfo8JgGh\/W+UoCD+BBBgIyvEB\/kMEDewGRFzMX1hUqfE9ely5dXEufPn26Ws5N04sXuCMv5ob07+IwmCaDu7NU8AAeZCAgwwv0Bxk8QOQlwENpaal6zuzqq69WETu3jyN8zz77LM2cOVPMBch2P91+M+g\/YwXNeHe9SoY78uJrWBhM48MakTwZWIMH8CAfATkeIpIXMxd79+6lAQMG0IoVK2jSpEnUtGnTAzxYvXo19ezZk4qLi9XBjCpVqsTsoXdxbo3GfhEyX53CV6jgM48ARJ55jP2UAB78oGQ+DXgwj7GfEsCDH5TiSQORFw\/OB5Ty4YcfUvfu3dWFyBdffDG1bt1a\/ZwvSX711VfVPjxeruUrVSR+XiIPFyHHxxoG0\/iwRgRJBtbgATzIR0COhxB5CXHxySef0LBhw2jZsmVUVlamvKhUqRK1bNmS7r33Xjr++OMT8sy7WLdGU3fAG+UZcRGyN4a6UkDk6UIymh3wEA0\/XbnBgy4ko9kBD9Hw05kbIk8nmiFscTSPL0Xmr06dOiJP0zqr5Ww0uCMvBPGasmAw1QRkRDPgISKAmrKDB01ARjQDHiICqDE7RJ5GMLNiCiJPDtMYTGVwAR7AgwwEZHiB\/iCDB\/YCIk8OF6nxxNlocBFyctRhME0Oe3vJ4AE8yEBAhhfoDzJ4gMiTw0OqPMkn8nARcrxUYjCNF+9cpYEH8CADARleoD\/I4AEiTw4PqfLEKfJwEXJy9GEwTQ57RPJkYA8ewIM8BOR4hOVaOVykxhNno8FFyMlRB5GXHPYQFzKwBw\/gQR4CcjyCyJPDRWo8cTYaXIScHHUQeclhD3EhA3vwAB7kISDHI4g8OVykxpN8Ig8XIcdLI0RevHjnKg08gAcZCMjwAv1BBg\/sBUSeHC5S44mz0bQcsYj4rjz+IPLipRGDabx4Q+TJwBs8gAfZCMjxDiJPDhep8cTeaH48rB6xyLO+5\/u1ojbH1k5NXdLuKESeDAbBA3iQgYAML9AfZPCASJ4cHlLlCUSeHLowmMrgAjyABxkIyPAC\/UEGDxB5cnhIlSd2kbfmhwK6auzScv+XDT+H+K48fPEggME0Hpy9SgEPXgjF83PwEA\/OXqWABy+E4vs5lmvjw7rClJQvkrdpzAUVpp5pqAgGUxksgQfwIAMBGV6gP8jgAZE8OTykyhO7yHvqo700ct4a5T9eu4ifRgym8WPuViJ4AA8yEJDhBfqDDB4g8uTwkCpPIPLk0IXBVAYX4AE8yEBAhhfoDzJ4gMiTw0OqPLGLvPsWbKcZ765X\/rduVpte6N8qVXVJu7MYTGUwCB7AgwwEZHiB\/iCDB4g8OTykyhO7yOv7XCkt\/HyL8v\/6MwrpseuLU1WXtDuLwVQGg+ABPMhAQIYX6A8yeIDIk8NDqjzJJfJY4LHQwxcfAhhM48M6X0ngATzIQECGF+gPMniAyJPDQ6o8sYu8U8d8Vu47RF78NGIwjR9ztxLBA3iQgYAML9AfZPAAkSeHh1R5Yom86XNL6IopX5X7jtcu4qcRg2n8mEPkycAcPIAHuQjI8Qz35MnhIjWeQOTJoQoiTwYX4AE8yEBAhhfoDzJ4QCRPDg+p8sQSefc98TzxwQvrw2sX8dOIwTR+zBFBkoE5eAAPchGQ4xkieXK4SI0nuSJ5eO0ifgoh8uLHHOJCBubgATzIRUCOZxB5crhIjSdWo7n8D9OIX7zgD69dJEMfRF4yuDtLBQ\/gQQYCMrxAf5DBA3sBkSeHi9R4ApEnhyoMpjK4AA\/gQQYCMrxAf5DBA0SeHB5S5Ykl8k7pO5EWfHOo8h2vXSRDIQbTZHBHJE8G7uABPMhEQI5XiOTJ4SI1nliNpnanh2jNDwXKb7x2kQx9EHnJ4A5xIQN38AAeZCIgxyuIPDlcpMYTN5E35JImNOSSpqmpQ0VxFCJPBpPgATzIQECGF+gPMnhgLyDy5HCRGk+sRrOt\/Uj68bB6ym+IvGTow2CaDO6IIMnAHTyAB5kIyPEKIk8OF6nxxGo0Wzr8d7nPeNIsGfog8pLBHeJCBu7gATzIRECOVxB5crhIjSfcaDr1+C\/iSJ714UmzZOiDyEsGd4gLGbiDB\/AgEwE5XkHkyeEiNZ5A5MmhCiJPBhfgATzIQECGF+gPMnhgLyDy5HCRGk\/clmvxpFky9GEwTQZ3RJBk4A4ewINMBOR4BZEnh4vUeOIWycOTZsnQB5GXDO4QFzJwBw\/gQSYCcryCyJPDRWo8cUby8KRZctRB5CWHvb1k8AAeZCAgwwv0Bxk8YLlWDg+p8gQiTw5dGExlcAEewIMMBGR4gf4ggweIPDk8pMoTp8jDk2bJ0YfBNDnsEcmTgT14AA\/yEJDjEZZr5XARyZMdO3ZQjx49aMmSJeV2Bg4cSP369ctrd\/ny5dS1a1favn37AenOPPNMmjx5MhUU7H+2zP5B5EWiSmtmiDytcIY2Bh5CQ6c1I3jQCmdoY+AhNHTaM0LkaYc0foPr16+njh07UoMGDcqFmSXeLrroIhozZkxOp0pKSmjQoEE0bdo0atGihS\/nnSIPr134gs1IIgymRmANbBQ8BIbMSAbwYATWwEbBQ2DIjGWAyDMGbXyGcwm1sWPH0jPPPEOzZs2iwsJCV4c4zfz583NG7dwyQeTFx61XSRhMvRCK5+fgIR6cvUoBD14IxfNz8BAPzn5Kgcjzg1JK07CAmzBhQt4o3YABA1Tt8kX7nNV3ijw8aZZcA8Fgmhz29pLBA3iQgYAML9AfZPDAXkDkyeFCuycs4N57772ckTxrmZf33a1cubK8\/A4dOuQVfU6RhyfNtFPn2yAGU99QGU0IHozC69s4ePANldGE4MEovIGMQ+QFgis9iXkJt0+fPpTv8IXbvj23\/X1ekbxxv6xPfMI215JwelBLn6cYTGVwBh7AgwwEZHiB\/pAcDzyH27+1a9dS586d1basoqKi5ByLseRKZWVlZTGWF3tRlngrLi4OtNfOctQSiOPHj6f27dsf5L8zklezZAgdsnOjOqXbrVu32Oub5QJ3795NpaWlSmBXrlw5y1AkWnfwkCj85YWDB\/AgA4HkvJg6daraouX8IPKS40RryVEFHjtj2ejdu7frFSxOkfdyx+qqDiw0EM3TSqensV27dtGGDRvUb2gQeZ5wGUsAHoxBG8gweAgEl7HE4MEYtJ6GOZJnj+YtXryYHnroIUTyPJFLQQIrApfvjjs\/1Qgq8vBurR9UzaTBsogZXINaBQ9BETOTHjyYwTWoVfAQFDFz6bEnzxy2sVq2BJ7XoQm7U7mWZb3uzrNH8vBubaw0H1QYBtNk8bdKBw\/gQQYCMrxAf5DBA3sBkSeHi9Ce+L342FmA9UrGunXryk\/g+rEFkReaKu0ZMZhqhzSUQfAQCjbtmcCDdkhDGQQPoWAzkgkizwis8Rrlq1Lmzp2bs1DrAEWue\/Oc+b2eQ7OLPLxbGy\/XztIwmCaLPyJ5MvAHD+BBFgJyvIHIk8NFajyByJNDFUSeDC7AA3iQgYAML9AfZPDAXkDkyeEiNZ7YRR7erU2WNgymyeKPCJIM\/MEDeJCFgBxvIPLkcJEaT7LYaKSSA5EngxnwAB5kICDDC\/QHGTwgkieHh1R5ApEnhy4MpjK4AA\/gQQYCMrxAf5DBA0SeHB5S5QlEnhy6MJjK4AI8gAcZCMjwAv1BBg8QeXJ4SJUnEHly6MJgKoML8AAeZCAgwwv0Bxk8QOTJ4SFVnkDkyaFrzZo1NGXKFOrRo0dmHp+Wg\/5\/PAEPMlgBD+BBBgJyvMjifF2prKysTA4F6fMki41GKkvgQgYz4AE8yEBAhhfoDzJ4QCRPDg+p8gQdWA5d4EIGF+ABPMhAQIYX6A8yeIDIk8NDqjyxOvDTTz+NJcKEmVu7di117tyZwEWyRICHZPG3SgcP4EEGAnK8sPrE\/PnzMzNfY7k2YvvjRjNo0CBavHhxREvIDgSAABAAAkAACJhE4KyzzqIZM2aYLEKUbYg8DXSw0OM\/+IAAEAACQAAIAAG5CBQVFWUmiscsQOTJbYvwDAgAASAABIAAEAACoRGAyAsNHTICASAABIAAEAACQEAuAhB5crmBZ0AACAABIAAEgAAQCI0ARF5o6JARCAABIAAEgAAQAAJyEYDIk8sNPAMCQAAIAAEgAASAQGgEIPJCQ4eMQAAIAAEgAASAABCQiwBEnlxu4BkQAAJAAAgAASAABEIjAJEXGjpkBAJAAAgAASAABICAXAQg8uRyA8+AABAAAkAACAABIBAaAYi80NAR7dixg3r06EFLlixRVvgm7VmzZlFhYWEEq8iaC4EBAwbQ3Llz1Y9r1KhB06ZNoxYtWuQFrKSkhPr06VOeBhxFb19heLCXun79eurYsSN16tSJ+vXrF92hDFsIw4Vz3GL4xo8fT+3bt88wktGqHoYHqx9YryVhbIrGgd\/czBV\/Y8aM8Zsl1ekg8kLSZw2UDRo0KG8s3Hjee+89CL2QmObLxtiuW7eOJk+eTAUFBTR27FiaMGFCXqHHaUaPHn3ABMZ2Xn\/9dV8C0UA1Um8yDA\/OSlsT4sCBAyHyIrSIMFxYwoLHLXtfcvaTCG5lLmsUHk4\/\/XTMHzG2GGtO6NChA0RejLinsig3kYEIhRkqrWicPdrgJrLtpef6OTgKz1EYHpyl2SOrEHnxc8Hj1jPPPHPAL6JefSm8lxU\/Z9g+4TZ\/LF++nLp27Uq9e\/fGLz+am44zeg2RpxngimjO+dubVcdc\/14RMYirTm4TE5ed69\/z+WWJPPtv0HHVI+3lROXBwp63OHAUCcu14VtEGC6sia5t27YQEeGhPyBnGB6sscu5EgGRp4kUhxmr3fNK0JQpU2jYsGFkX4EzU6ocq1iuDcFFvt98sWQbAlCPLLmEs58lW6dpDKTh+YnCg73PDB48GHvywtOgcobhwhLZw4cPp1WrVqmtDPz53d8a0eUKmT0MDwyE2y+b2EpivolkMWoNkReiXeVrKGGiSyFcyFSWXAMpL5UMGjTI9\/46+290OCATvAlF4cHeL7hkHLwIjr89RxgurF9wtm\/fTvblKre9q9G8y07uMDw4ebQOk5155pnl+ySzg2C8NYXIixfv1JYGkRcvdVEHUstba8M\/ThKG4y8sD5a4GDVqlDrBiX2R4fDXJfKKi4sPEBPWeMb2rcMY0T3MhoWwfcLay2fflwqxbb7NQOSZx7hClIDl2nhpDLsk4vYbMwReeO7C8ODWVyDywnNg\/4XFftrc+vd8WxgssX3RRRcddLIQKxDhOInSJ9xENfZ0h+PBby6IPL9IIV3OPTHopPobR9jNzeyJ\/VQVBF40bsLwYF8idCsdd4OF4yQMF\/kOHUHkxccDVoLCYa0jF0SeDhQzYsNtUESEwgz5bnvv\/HRWK82KFSt879szU4OKYTUsD87ao59Ebw9huXA7GOanL0X3uGJaCMNDvuVxBAnMtpMstnUcvAjZptwuFcXJ2pBgemRzOzDh52QtTqvp5SMsDxB5enmwR6h5ydY6ROSnT7gt2frJp78GFcNi2D6BPXnJ8A+RlwzuqS3VecEilp7MUmkdnOBS3K59sIvsDRs2qItF+SSh24eTbOG5CsKD2xN\/iOSFx96ZMwwXzue0cIVKdD7C8ODcygAeovPgZQEizwsh\/BwIAAEgAASAABAAAkAgFQhguTYVNMFJIAAEgAAQAAJAAAgEQwAiLxheSA0EgAAQAAJAAAgAgVQgAJGXCprgJBAAAkAACAABIAAEgiEAkRcML6QGAkAACAABIAAEgEAqEIDISwVNcBIIAAEgAASAABAAAsEQgMgLhhdSAwEgAASAABAAAkAgFQhA5KWCJjgJBIAAEAACQAAIAIFgCEDkBcMLqYEAEAACQAAIAAEgkAoEIPJSQROcBAL6ENi3bx8999xztGvXLvrNb34T2PDXX39NkydPpn79+lH9+vUD54+S4bXXXqO77rqL+Dmvhg0b0jPPPKP+DvpZrz6cfvrpNGbMmKDZxaV3vmIxcOBAxU8Sn\/Mlh\/Hjx1P79u2TcAVlAoHMIwCRl\/kmAACyhkDUZ8X4rVMWV9abqXHht23bNurZs6cSeH369KGjjjqKWrduTdWrVw\/sQkUVeUcffbR6zu+4446j5s2bB8ZFR4bNmzfT4sWL6Z\/\/\/Kf6ZQAiTweqsAEEwiEAkRcON+QCAqlFIK0iT6cw02lLQkOQWJ+SkhIlxiHyJLQQ+JBVBCDysso86l0hESgrK6MXXniBHnjgAVq7di1VqVKFWrZsSXfffTcdf\/zxlO9RdH68+9FHH6W5c+fSt99+S5UqVaKioiK67bbb6Morr1T\/b3+InQHs0KFD+XLnJ598Qn\/84x9VBIe\/U089lQYPHkxnnXWWJ9a8BPzII4\/Qiy++SDt37qQGDRrQzTffrJaTq1atSpZgsBvKtyTJOLzyyiv0l7\/8hb744guFwznnnEPDhw+nZs2akSWKWrVqRWeeeSY9\/PDDVFpaqspln636cnl79uyhp556iqZMmaIwZdtHHnkkde\/enW688UblH39WhPOmm25S5f7444\/0pz\/9ia655hr6\/PPP1X+\/\/fbbdOihh9L1119Pp5xyiuJl2rRp1KJFC2WDl9C5nMcff5y+++47OuKII+hXv\/oV\/fa3v6WCgoKcOLqJPPtj7Oeffz7dc889tHHjRtUOhg0bpqKgzGmuL1fE1m8kFyLPs9kjARAwjgBEnnGIUQAQiA+Bl156SYmy8847jy655BLasmUL\/fd\/\/7eazJ9++mmqUaOGEkyjR4+mn\/\/85\/SLX\/xCibDDDz9cCbg333yTrr32WjrjjDNo9erVNGPGDCU2JkyYoGy+\/\/77SpSwWLn11lvphBNOoJ\/97GfKJudv0qQJdenSRVV4+vTptGrVKiUCL7vsspwgsHBiscQii\/9u3LixEpoLFy6kjh07KiHE4uSNN96gBx98kI455pi8S5Iswjh6xHUsLi5W\/rAIYn\/q1q1LTzzxhBJabJv\/nZc4u3XrRtWqVVPiiv1gzBgDtjVq1CiaNGmSEn5tZ5Df2gAAB\/JJREFU27alrVu3KlssHu+8807liyXyWFzXqVOHevXqpcq48MILlcDkejEX7EvNmjWVkGNBxwLSEnksyljMvffee+UcvPvuuzR79mwl1FlccV63L5\/IW7FihfLh17\/+tcKWy2PfWdjm2ysHkRdfv0VJQMAUAhB5ppCFXSCQAAIstDgqxqLEEgRvvfWWEmAjRoxQk7rbci1H4XijPkeN7Bv2WWRwRI2X3ax\/d07+vFeORUxhYaGKIFp75FjAcLm8h45FjZtAYRHFQokPgrDAYsHIHx8OYZHGETRLcPldkvzyyy+pc+fOShixwLQibe+88w7dcsstdPvttyuByyKPo21PPvkkNW3aVJVr1Zf3\/nFaFpe9e\/dW+9s4EsdiiT+rDI4CWgc3GBf2+b777qPrrrtOpbPqx1HFiRMnEkcO+bMLW0vksbDlSCjbadOmTXnrYb9ZNHL5HDkNKvKWLVt2gNDmPXMchWThn4sXS7S67b1EJC+Bjo0igUBIBCDyQgKHbEBAIgIssljgsUjhCJPb6dcge\/KstJdffjkNGTJEVdk5yS9dulRFwq644gq1BGj\/OMLH0UH7kqT95yyibrjhBiWyHnrooXIRxWk4AsV14Mgil+1X5PEJXBakvPScK1JlX67l6KC1bOm3DGsplCODlt+MC0c87XXNVz9enuaDCZyel7Y5MsqRUo7mccTV+jgCyD6ee+65OU8C54vksR0ux77cy75yO2GBe\/LJJ7s2ZUTyJPZw+AQEgiEAkRcML6QGAqIR4CieJRbYUd47xkuyLJasaFU+kbd79261lMcCa8mSJcRRQI7E2ffeOSd\/t\/1ydpBYQLH4adeu3UHYuYlIK5FTuPgVYCxeWODlEpZsP5etXP\/OUUnG9uOPP1ZCbMGCBQonXtK1BJSbyOM8HNXjPxwZdArgQYMGKT95n2CPHj0U5rk+jj4yjry07vy89uQ5r4nxs18OIk90V4dzQMAXAhB5vmBCIiCQHgR4iZAFyMyZM2nevHnlBzCsvXFuIo9FDC818n41XirlpVW+f44PBLz66qsHRJFyibwwpyhNiDw3seVHFLmJP8aSRRjvy+MDIYcddpjaw8f367Eg40hpPpG3Zs0adXikU6dOvkSeW9TNT8sLI\/L69u2bU3xzmRB5fpBHGiAgGwGIPNn8wDsgEBmBTz\/9VO2p4033LMR4TxbvR2PhYe2z+8c\/\/kE86fNerf79+5cv7VkihQ9n2Pee2fdqsdjhPXm8ZGst6fp12s9yLUchOQrmN5KXa7mWD1Tw0vDFF1+shBdj4LwM2VmGtWTM9f\/zn\/+sDlXwZ+1r48Ma+URevvrZxeiJJ55Iv\/vd79TpZ96HyAdYgnz5RB7vkWTe2Vfr47JZ0HNZue7Ty5WGTx\/zPkGvexL9RAuD1BFpgQAQCI4ARF5wzJADCIhEYPv27fT73\/9eReJ4grYOQGzatEkJsHr16uUUeZyehcDUqVPLDwdwFIsncr5u45e\/\/GVOkWcdvOC9Y5y\/UaNGCh+ODnJevlKFD09Yy8V28LwOXrAQ4QMZvFTpV+TlOnjx\/PPPq+tR+FQp74HzI\/IsocL753hfovXxMjYLZBZn+UReroMXlkjkiKv94AWfjObldt6XZ+0T5OVhPvzB167wz90+r9O19kMtfD0OtwdeymfOc10mzSeIWdiOGzdOnRLm76uvvlJ5\/+\/\/\/g8iT+QoAKeAwIEIQOShRQCBCoQAn+DkZVc+pcrCjL9nn31WRYisKzOs6BL\/jKN3vKH\/gw8+UEKCl2g5mscTP5945Sgdn0BlgWNF8qzJnyd7vqaFy3r55ZfVSVre3M92+OAA52eBwsKTI4m57mTzc4UKn5D1K\/LsV6hwBI4Pbnz22WfqkMFpp52mBDAfnPAj8jiSx\/VkDLhevDw7f\/58dWXM3r17lSDOJ\/IYY46GcgSRr0zhK1TYBosurvdPfvKTcpHHYpmF46JFi5SoYsx5D+CcOXOodu3a6iQs31sYVOQxh3xAxF42l2U\/7WuJWfvdg1bd+UQxn7Dmj0U8+8Ii1YrkWbzwHYP2Ax6I5FWggQVVSS0CEHmppQ6OA4GDEeAoHi+l8gTOIoI\/5+W3LIJY8LAY\/P7779W+rIsuukhdonz\/\/fergxa894zvxWMRyCd2+W4467oNPkzAgo7vc+MLhq3DAPyUFednwciiiAUJL0Gy2OQ74\/J9bpch8xLt1VdfXX4Fil+Rx+U4L0Nm0cqizhKiQQ5e8J2AfP0MXzNjXS7NByY4Mvj666+r+wd\/+tOfKvHoPF1r1ZkvQ7aimtZlyOwTL5faD4hYF1L\/7W9\/U\/cTMg8XXHABDR06NO8bvfkieYwFc8lRUY62stDl61i4XVifm8jjn9nrzkKRT20zr\/\/v\/\/0\/iDwMQEAgBQhA5KWAJLgIBIBAxUOAl4D5cAz\/4QhqlC\/oFSpRyvKbF5E8v0ghHRAwhwBEnjlsYRkIAIGMI8BX0vD+ukMOOURFRK276ni5lKNivITtvMMuDGQQeWFQQx4gUPERgMir+ByjhkAACCSIAC\/H8nLv2Wefre4b5AMpfp988+u2JfL4ehc+jXzcccepd3j57j3+dAhJv77wfj1euucDN1xumKt1\/JaFdEAACORHACIPLQQIAAEgYBAB3ifJh1D4ZQzeJ8lRPT7dy\/v6+GBIrgMpQVyyRJ61D5MPULDYS0Lk8SEfLptPe\/MHkReESaQFAnoR+P9S4aruZGTYdAAAAABJRU5ErkJggg==","height":305,"width":507}}
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
