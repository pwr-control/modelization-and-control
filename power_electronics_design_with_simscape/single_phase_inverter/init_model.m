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
ld1_load_trafo = 250e-6;
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
freq = 80;
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
freq = 50;
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:00aa870f]

freq_filter = 50;
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
%   data: {"layout":"onright","rightPanelPercent":52.2}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ9wVdd9539pSWJ5HUeSkyWWZSNsQ01mdwnROlEVNqQF101jMR3SqXi0s4SRCUvCOJOIQUJmltDGQmKiTENICbZVhhlXMk3LTK3MZEjALTNYpXEw0UxbHGPLgsiyvS5IcT3Fk9Jk53fxedz3eH\/OPefee879ve+b8Vh6On\/u+Xx\/95wv58+97\/rVr371K8IHBEAABEAABEAABDJE4F0wMBlSC5cKAiAAAiAAAiAQEICBQSCAAAiAAAiAAAhkjgAMTOYkwwWDAAiAAAiAAAjAwCAGQAAEQAAEQAAEMkcABiZzkuGCQQAEQAAEQAAEYGAQAyAAAiAAAiAAApkjAAOTOclwwSAAAiAAAiAAAjAwiAEQAAEQAAEQAIHMEYCByZxkuGAQAAEQAAEQAAEYGMQACIAACIAACIBA5gjAwGROMlwwCIAACIAACIAADAxiAARAAARAAARAIHMEYGAyJxkuGATiI3Du3DnasGEDzczM5Avt6OiggYEBqqurq1rR5cuXqbe3l3K5HLW1tVVNf+rUKVq3bl1Buk2bNlFPTw9FLatqZUgAAiAgmgAMjGh50TgQqEyADcy2bdtoz549tGjRosgmIqrpYAMzODhIw8PD1NjYmK+vqakpMDH4gAAIgIAuARgYXVJIBwICCVQzMOEZEzVTwhjYhBw4cIBWrFgRUOG\/8QzM4cOHafv27fnvik1JsYHhhHwN\/f399LWvfS0wUjybs3Tp0mBmZ2xsLChLzQrxz+p7\/u7NN9+kvr4+eu6552h8fJwuXLhAa9eupQULFhTM9IyMjNDixYupu7ubPvnJT9Kf\/umfEpumr3\/960FbJiYmaPfu3dTZ2SlQZTQJBGQSgIGRqStaBQJaBEotIamBPGxumpubA+PQ3t4emAM1i3Lx4sVgCYqNAH9GR0eD5SdlNIqXlkoZmEuXLgXG4itf+Qo9\/vjjgYG5\/fbbgzJuu+02Un8PGxWug03H1q1b6eDBg4GBefLJJ\/MzO1yPWtJiUzU1NUUbN26krq6u4Hs2VtwGTsezQT\/84Q8DA6S7dKYFF4lAAAQSJQADkyheFA4CfhMoNwOjjIoyJLwfRhkB1aLifSvnz5\/Pz76oNOFZG\/5O18CwyVDLUzwLw7Ml+\/fvDwwOXxvPlJQzNmrvTvHskTIwfN1qtoiNDf\/OacNt9Vs1XB0IgAATgIFBHIBADRMoNjCMQhkVXh6KamCUISiHVHcJifPzZt\/w0o+aoalmYNTsD\/+fZ1SeeuqpghkYGJgaDng0XRQBGBhRcqIxIBCNQKUZmI9+9KP5Db66S0hqSSecPryvpNIm3oceeih\/oolbocxT8VKRWuop970yMOG9NDyDgxmYaLGB1CDgOwEYGN8VwvWBQIIEqh2jTmITr84xat5wy\/tV2KRw+n\/7t3+7bnNvqU28ag+L2kzMxoXL+clPfhKYsS1btgRLRlhCSjCoUDQIpEQABiYl0KgGBEAgXgKllqPirQGlgQAI+EwABsZndXBtIAACBQTCx7T5D7xHRucBesAIAiAgjwAMjDxN0SIQAAEQAAEQEE8ABka8xGggCIAACIAACMgjAAMjT1O0CARAAARAAATEE4CBES8xGggCIAACIAAC8gjAwMjTFC0CARAAARAAAfEEYGDES4wGggAIgAAIgIA8AjAw8jRFi0AABEAABEBAPAEYGPESo4EgAAIgAAIgII8ADIw8TdEiEAABEAABEBBPAAZGvMRoIAiAAAiAAAjIIwADI09TtAgEQAAEQAAExBOAgREvMRoIAiAAAiAAAvIIwMDI0xQtAgEQAAEQAAHxBGBgxEuMBoIACIAACICAPAIwMPI0RYtAAARAAARAQDwBGBjxEqOBIAACIAACICCPAAyMPE3RIhAAARAAARAQTwAGRrzEaCAIgAAIgAAIyCMAAyNPU7QIBEAABEAABMQTgIERLzEaCAIgAAIgAALyCMDAaGh65513aqRCEhAAARAAgSQJ1NfX07Jly2hubo7OnDmTZFWiyp6cnBTVHtUYGBgNWdnApB0AcdQZtQyd9NXSlPt7lO+L01arU0PCyEniqDNqGTrpq6WJwpmhlEoP\/pXDpZIGUf+mw7+cTpGDOmKGarGmU1zUMqqlf\/3116mvr49yuRytWrWq5CXgHriGRbGoxlVHSx\/TwMBoqCJVfI2me5EE\/N3KAP5u+bsyMO5bff0VsIE5duwYzZ8\/v6yBSeK6s34PZP36y2kKA6MR7R\/6g6\/RZz\/7WY2U8pPc0ViXeCN77m8pqEPqzZc4yJgqAP+YQFoUAw2uwoOBMQsiqfEDA6MRD\/\/1f3+H2j7+cY2UtZHkwuzbqTb0ypUrNG\/evKDOC5fSrTvc0Dsabyho9x0N136\/\/Z2\/hQ2eSp+790Op8oq7spdffpkWLlwYd7EoLwIBaODWwGSdPwxMhJtNWlKp4mdFJ586j8GjU3lsFy5dzv\/8s5CxUgavmtlSBoeNULEBKp6FcqmVT\/xdcnBZNzSAgbGJP6ljGGZg3omKU6dO0bp164Lfdu\/eTZ2dnfl4kSq+zQ2RZl7JnTcbIjZCygCx+SllfMJm5xN3NwT4P3FXffD\/5Xdf\/X9SH8n8k2IWd7nQAAbGJqakjmEwMER06dIl6u7uDna386e\/v5+GhoaosbEx+F2q+DY3RJp5a7XzZiPD\/\/0sZGqU2Tn50lyBBNeWq27Nm5u4jE2t8k8zxqvVBQ1gYKrFSKW\/Sx3DYGCIiGdfBgcHaXh4mOrq6qi3tzc4ptfW1gYDY3PXxJQXnXdpkGqm5pmX5vKzNs+8OEthcxOHsQH\/mALZohhoAANjET5i\/xEOA\/OOgRkdHaWBgYEgRtjAtLe355eRpLpXmxsizbzovKPTVrM3bG74U8rY5O69lXT22oB\/dP5x54AGMDA2MSV1DIOB0TQw4eA5fvy4TSwhb0QC09PT1NzcHDEXkpciMPPmFTr9ytv06ptX6MevvB38zJ+mm+fRre+bR\/\/zthuotfkGar3t2gkr8HcfS9Dgqgazs7N0+vRpamhooNbW1tSEySL\/lStXFvBJ+2GsaYgDA4MlpDTizKoO\/OvTCl\/VzDxbM\/rsa0G6waMv59Pz8hPP0tz1Xy7THyxfUrUcJEiOAO4BzMDYRBdmYGzoeZ4Xm3j9Fgidtxt9+ITU6LOv5vfXKEOjs+zk5orl1op7AAbGJrphYGzoZSBv+Bj1yMhIfgMvX7pU8TMgS3CJ6LzdKvXMxDk6+dq7C2ZolJnhL2FoktcH9wAMjE2USR3DsISkERVSxddouhdJ0Hm7laGYv1pyCm8MxuxMshrhHoCBsYkwqWMYDIxGVEgVX6PpXiRB5+1Whkr8w6ed1P4ZZWb4QXtxPYvGLQH3teMegIGxiUKpYxgMjEZUSBVfo+leJEHn7VaGKPzV7IzaO4OZmXi0i6JBPDX6WYqrlzlmnb\/UMQwGRuM+lSq+RtO9SJL1zsMLiBYXYcq\/eKkJMzPmIphqYF6jnzlhYMx0kTqGwcBoxINU8TWa7kUSdN5uZYiDf7mZGSwz6WkbhwZ6NfmdCgbGTB+pYxgMjEY8SBVfo+leJEHn7VaGuPmXMzO5ez9E6tUHblvsX+1xa+BfC\/WuCAZGj1NxKqljmPcG5vDhw7R9+\/YCPYrfFm0mqX4uqeLrE3CbEp23bP7h581gz0xprXEPXOUCA2PWF0gdw7w1MOq5LKXMijI1xc9rMZO2ei6p4ldvuR8p0Hm71SEt\/mpmJnyaqef+hcQzM7X+SUsD3znDwJgpJHUM89LA8JNxx8bGaP369RXVOnToUNU0ZnIX5pIqfhxs0igDnXcalMvX4YL\/yRfngqcA8ysO1KxMLS8xudDAbdSVrh0GxkwVqWOYlwbGTKLkckkVPzli8ZaMzjtenlFLc8kfx7KvquVSg6jxkmR6GBgzulLHMG8NzLlz52jDhg00MzNDvIzEH94L09TURAcPHqRFixaZKWmQS6r4BiicZEHn7QR7vlJf+BcvMfGMzPK7G2piickXDdxGIvbAmPKXOoZ5aWAuX75Mvb29lMvlaPHixdTV1UVr166lzs5O4v0v4+PjNDAwQHV1daZ6RsonVfxIEBwmRuftEL6n\/\/pnM8N7ZWpliQn3wNV7ADMwZn2B1DHMSwPDe2B27dpFO3fupMbGRhocHKQVK1YEL1gs\/puZnNFySRU\/GgV3qdF5u2Pv+\/JFrSwx4R6AgbHpBaSOYTVnYMLHspcuXUrDw8OBSQq\/jbr45JNU8W1uiDTzovNOk\/b1dWWFf3iJSdrG36xokHSkYgbGjLDUMaymDAzvq+nv76ehoaH8zA7vsdm2bRvt2LGD+vr6gugIp+HfpYpvdiuknwudd\/rMwzVmjb\/EWZmsaZBUxMLAmJGVOoZ5a2B438vExERJtcIzJ2ZyXs3Fsy6jo6O0Zs0a+rM\/+7NgNob31aj9N7xkBQNjQzievOi84+FoWkqW+UuZlcmyBqZxVyofDIwZTRgYM25e5+K9NS0tLbRgwYLAyPDGYP6wgWlvbw82DSsDE27I8ePHvW6XtIubnp6m5uZmac3KTHuk8H\/0H+do7Pm3aObNK9R08zz6\/MfqqWPJTZnQQYoGtrBnZ2fp9OnT1NDQQK2trbbFaefPIv+VK1cWtG9yclK7vVlJWLMzMLwXZmpqinp6evIzMZUMjETxsxKk+NenW6Wk8S81K9Nzf4tbyFVql6aBKWzMwJiRwwyMGTfrXGGjwYXx7\/xRsyOVKgg\/S6ajoyN\/9FrNvKgyeCmJv8MSkrVciRSAzjsRrNqFSuVfvFdm+V31lPvYrV4+V0aqBtpB+E5CGJioxK6mh4Ex42aVq9SRadtj1OEj2eriuMzu7m5s4rVSK7nM6LyTY6tTci3wL\/VcGZ9mZWpBA51YhIHRoXR9GhgYM25WudQD7cL7UdiA8MkhkwfZhY9KqwtTMzO8YXjdunXB18UviZQqvpU4KWZG550i7BJV1RL\/4qf9qpdJ8rFsl59a0qASZxgYsyiUOoZ5uQcmLBHPjoRPJMV1AilKGEgVPwoDl2nRebukX7vv4eGn\/I7+6FU6+dJc\/oWSrmZlcA9cvQdgYMz6AqljmPcGxkyueHNJFT9eSsmVhs47ObY6Jdc6fx+OYte6BipOYWB07lgsIZlRiikXz7qMjY3R+vXrK5Z46NChqmniuCQYmDgompeBztucXRw5wf8qxVIPyFNLTHFwrlQGNMAMjE2MSR3DvJ2BUftVih\/rzyKq1wEU71WxEbhSXqniJ8Ur7nLRecdNNFp54H89r7SPYkMDGJhod21haqljmLcGRuEPv7tIfVfK1NiIWy2vVPGrtduXv6PzdqsE+JfnX2rTbxL7ZKABDIxNLyB1DPPewNiIFldeqeLHxSfpctB5J024cvngr8d\/8OgUjT77arDUpF4mGZeZgQYwMHpRWDqV1DEMBkYjKqSKr9F0L5Kg83YrA\/hH45\/E8hI0gIGJFoVYQrLhJSovDIxbOdF5g79bAma1F2\/65Q2\/pjMyuAdgYMyi8GouqWMYZmA0okKq+BpN9yIJOm+3MoC\/PX\/bJ\/1CAxgYmyiUOoZ5b2DCD7J7\/PHH6emnnw6OTi9atMhGz0h5pYofCYLDxOi8HcKn2n2QXRLUTZeXcA\/AwNjEo9QxzGsDE36VAL85esWKFYGGo6OjRq8SMA0AqeKb8kg7HzrvtIkX1gf+8fMvXl7K3fshyt17Ky2\/u75kZdAABsYmCqWOYV4bmPCLGx977LHAwCxevJh27dpFO3fupMbGRhtNtfNKFV8bgOOE6LzdCgD+yfLXWV6CBjAwNlEodQzz2sCUmoE5ceKE8cscwwFw7tw52rZtG+3ZsydYjgq\/6LH4OTNSxbe5IdLMi847TdrX1wX+6fCvtLwEDWBgbKJQ6hjmtYFhwZJ4maMyRqdPn6aDBw\/SLbfcQt3d3dTX1xfESH9\/Pw0NDeVneKSKb3NDpJkXnXeatGFg3NK+\/pUFvLy07sPz6BNL09v355pBufrxLiQzZaSOYd4bGDO5Kufip\/u+8cYbxAaGTcvFixdpcHCQhoeHqa6ujnp7eymXy1FbW1tQkFTxk2CbRJkwMElQ1S8T\/PVZxZ1SZ3kp7jp9Lg8GxkwdqWOYlwameNalWLKlS5cGZsNkDwwvHfFLIDdv3kw7duzIGxi1MZjrYgPT3t5OnZ2dMDBm90usuTCAxoozcmHgHxlZ7BmemThHJ197Nw0efTn2p\/zGfrEJFggDYwYXBsaMm3Uuni3hE0g9PT1BWfw7f5S5iFIBLx098sgjwTHs8LIRz8BUMzDheo4fPx6lWqS1JDA9PU3Nzc2WpSC7KQHwNyUXXz6lwcybV+h7Z9+iseffIv5508fq6YElN1HTzfPiq8zjkmZnZ4OZ84aGBmptbU3tSrN4D6xcubKAz+TkZGq80qrIyxkY1fjwKSQ121Lqu3KweLZlw4YNwabfjo4OevDBB4OZF\/5dfZqamujLX\/4yPfHEE1hCSivqItaDGYCIwGJODv4xAzUorpwGXxw9S6PPvhbMyvCTfnm\/jOQPZmDM1MUMjBk3q1zhU0hqxoX3qrABGRgYCParmH7YCKmNu9jEa0oxnXwYQNPhXK4W8HfLn2uvpIHpw\/Hctyr6FcDARGfGOWBgzLhZ50riFBJfVNjAFB+jHhkZyW\/glSy+tTgpFYABNCXQZaoBf7f8qxkYdXVhI8PfqRkZnp2R8oGBMVMSBsaMm4hcUsXPijgYQN0qBf5u+esamPBVDh6dotFnXyU2Ncvvqqfcx24VsbwEA2MWi1LHMO\/3wHR1ddHExESBajankEzklyq+CQsXeTCAuqB+rU7wd8vfxMCEZ2X45JLaJ8OvKzB9I7Z7CkQwMGYqSB3DvDYwpaTiU0gLFiwoWOIxk1Q\/l1Tx9Qm4TYkBFPzdEnBfu+09IGWfDAyMWSxKHcMyZ2CinEIyk\/r6XFLFj4tP0uXYdt5JX5\/08sHfvcJxasDLSzwrw5+s7ZOBgTGLRaljWOYMDL+zSD011+RBdibySxXfhIWLPHF23i6uP+t1gr97BZPQoHifzFNfXOa+oVWuAAbGTCKpY5jXBqbcE3mLX7ZoJql+Lqni6xNwmzKJzttti7JVO\/i71ytJDbL0ugIYGLNYlDqGeW1gzKSKP5dU8eMnlUyJSXbeyVyxrFLB372eaWiQBSMDA2MWi1LHMK8NjO2TeM2kvj6XVPHj4pN0OWl03km3Icvlg7979dLUwOcNvzAwZrEodQzz0sCoJ\/COjY2VVItfC2D7JN4oYSBV\/CgMXKZNs\/N22U5f6wZ\/98q40KDUg\/FcH8GGgTGLRaljmJcGRknk4sRRqfCQKr7ZrZB+Lhedd\/qt9LdG8HevjWsNik8uuTIyMDBmsSh1DPPSwCjj8tBDD9HWrVvxIDuzmBWTy3XnLQakYUPA3xBcjNl80cC1kYGBMQsqGBgzbiJySRU\/K+L40nlnhVfc1wn+cRONXp5vGoSPYPOzZNKakYGBiR47nEPqGOblDExYIn7y7vbt2wtUw6sEzII4q7l867yzytH0usHflFx8+XzVIG0jAwNjFlMwMGbcrHKp58D09PTE9uqA8AbhpqYmOnjwIBW\/jbr4OTNSxbcSJ8XMvnbeKSJwWhX4O8UfVO67Bmppid98neT7lmBgzGJR6hjm9QxMEpt4+Sm+LS0t1NnZSfxU3xMnTtDGjRupu7ub+vr6gujo7++noaEhUk\/6lSq+2a2Qfi7fO+\/0iaRbI\/iny7tUbVnRIGkjAwNjFotSxzCvDQxLxUtI\/GHDYfspZ4jCryeoq6uj3t5eyuVy+VkfqeLb8kwrf1Y677R4pF0P+KdN\/Pr6sqRB8XNk9q1dQsvvro8FIgyMGUapY5jXBqbcqwRM98AoA8MhwM+YUUtIFy9epNHR0eDZMvxhA9Pe3p43TSx++HP8+HGzKEIuIwLT09PU3NxslBeZ7AmAvz1D2xKyqMHMm1fo0R\/N0djZt6j1thvoq6s+QE03z7NCMTs7S6dPn6aGhgZqbW21KitK5izyX7lyZUETJycnozQ5E2m9NjBxEzx37hxt2LCBvv71rwezKzy7Mz4+TmvWrKEjR45UNDASxY+bb1LlZelfn0kxcFku+Lukf7XuLGvAMzJbRs\/SyZfmgrdf25xYwgyMWSxiBsaMm1WucjMwXGh4A265SpRhmZmZIX5677Zt22jHjh3BXhe1cZf3xGzevJn2799Pw8PDhCUkK8kSyZzlzjsRICkXCv4pAy9RnQQNwvtjfrLjN42gwsAYYcMxajNs9rl4lmRqaor4JBJ\/1J6YBQsWBMs+3\/zmNyNVEt7Eq2ZgwsaGC8Mm3khIE08sofNOHFKCFYB\/gnA1i5aige1sDAyMZsAUJcMMjBk3q1yVXubIT+ndu3dvZAMTntUJ76Xhjbzr1q0LrndkZKTg2LZU8a3ESTGzlM47RWSxVgX+seI0KkyaBmo2Zvld9fTUF5dpM4GB0UZVkFDqGOb1Hhj1zBZeLgrPwPC+FZ41eeKJJ\/Lfm8mql0uq+Hqtd59KWuftnmi0KwD\/aLySSC1RA56NWf3nZwJcuieVYGDMokvqGOa1gWGpivfB8KzJvn37aM+ePQUnhcxk1cslVXy91rtPJbHzdk9V\/wrAX59VUikla7D622eCDb5PfWFZ1ePWMDBmESZ1DPPewJjJFW8uqeLHSym50iR33slRi69k8I+PpWlJ0jVQS0qXvvFbFRHBwJhFkNQxzHsDw5tuDxw4UKCa6XNgzKSX+yIsUx5p55PeeafNM2p94B+VWPzpa0GD0Wdfoy+OnqVv55ZQ7t4PlYQIA2MWWzAwZtyscvHykXrE\/3PPPUd88uj8+fNBmXE8mVf34qSKr9t+1+lqofN2zbhS\/eDvXp1a0YANDBuZcstJMDBmsSh1DPN6BiZ8CumFF17Iv7do165dtHPnzvy7iswk1c8lVXx9Am5T1krn7ZZy+drB370ytaSBWk4qZWJgYMxiUeoY5rWB4VNIfFSaX7bIj\/vnp+jyQ+mwhGQWxFnNVUudt48agb97VWpNA97Ye2H2bSp+4B0MjFkswsCYcbPONTExQTfeeGP+yblbt26lgwcPBr+n9ZEqflr8bOuptc7bllfc+cE\/bqLRy6tFDT7ytX+g3L23Frx6AAYmeuxwDqljmNczMGZSxZ9Lqvjxk0qmxFrsvJMhaVYq+JtxizNXLWqgNvWGl5JgYMyiSuoYBgOjEQ9SxddouhdJarHz9gL8OxcB\/u7VqFUNeCmJP+ppvTAwZrEodQzz0sBUeokjy4c9MGZBnNVctdp5+6IX+LtXolY1OPniXPC0XjULAwNjFoswMGbcjHIVG5jidxMZFWqRSar4FkhSzVqrnXeqkCtUBv7ulahlDXgW5vbGG4Lnw8DAmMWi1DHMyxmYsESlXiUwPDxsfIT63LlzJU8zhV\/muHv37oLnzEgV3+xWSD9XLXfe6dO+vkbwd69CrWvQ+JW\/C04kvfc\/fk7Hjh2j+fPn06pVq1ITJuv8pY5h3huY4ghlo8FP5zUxMerlkO3t7YFB4XL4w8e01QPz+Pf+\/n4aGhrKmySp4qd291tWlPXOw7L5zrODv3MJqNY1YAPDMzC\/fce7YGAMwlHqGOa9gQnPmLBuHR0dNDAwQHV1dQYyUmBaWlpa8gaGf+Yn\/CpTxOX29vZSLpejtra2oA6p4hsBdJCp1jtvB8gLqgR\/1wpQzRsYfkIvf\/7vbzfCwBiEo9QxzEsDE142SmLDrnq\/ktpbw7M6o6OjgTHiDxsYNUsDA2Nwt8ScBQNozEAjFgf+EYElkBwaEPEszA8+fyf99PRJLCFFjDEYmIjAbJIndQpJldvT0xPMrqglpBUrVlQ1MOH2HD9+3KZ5yBuRwPT0NDU3N0fMheRxEQD\/uEialwMNiFq\/NUVfav11avr3n1JDQwO1traaA42YM4v8V65cWdDKycnJiK32P7mXMzBxYQsvP\/HS04MPPkjf+MY38vtb1H6azZs30\/79+4N9NVhCiot+fOXgX5\/xsTQpCfxNqMWbBxoQ8ZN5F930C\/r9D05jBiZieGEGJiIwH5MXz8AcPnyYxsfHadu2bbRjxw7q6+sLLhubeP1SD523Wz3A3y1\/rh0aEPE+mB8\/f54eWnwRBiZiSMLARATma3KdY9TFz52RKr6vGhVfFzpvt0qBv1v+MDBX+atXC3yrdQYGJmJISh3DRC8hRdS4bHKp4sfFJ+lyMIAmTbhy+eDvlj8MDAyMbQRKHcNgYDQiQ6r4Gk33IgkGULcygL9b\/jAw1\/jzSSTMwESPR6ljGAyMRixIFV+j6V4kwQDqVgbwd8sfBuYaf97I+8mbX6PVH34fnsQbISyljmEwMBpBIFV8jaZ7kQQDqFsZwN8tfxiYQgNz67suUffH3wsDEyEspY5hMDAaQSBVfI2me5EEA6hbGcDfLX8YmGv8+SQSP88EBiZaTEodw2BgNOJAqvgaTfciCQZQtzKAv1v+MDCFBoaPUj\/yv+ZhBiZCWEodw2BgNIJAqvgaTfciCQZQtzKAv1v+MDDX+A8enaLBoy\/TX\/3ef8LARAhLqWMYDIxGEEgVX6PpXiTBAOpWBvB3yx8G5hr\/119\/nZYM\/gsMTMSQlDqGwcBoBIJU8TWa7kUSDKBuZQB\/t\/xhYGBgbCNQ6hgGA6MRGVLF12i6F0kwgLqVAfzd8oeBgYGxjUCpYxgMjEZkSBVfo+leJMEA6lYG8HfLHwam0MAcO3YMrxKIGJJSxzAYGI1AkCq+RtO9SIIB1K0M4O+WPwwMDIxtBEodw2BgNCJDqvgaTfciCQZQtzKAv1v+MDAwMLYRKHUME29gDh8+TFNTU9TT0xPEwKVLl6irq4smJiaoo6ODBgYGqK6ujk6dOkXr1q0L0uzevZs6OzvzMSNVfNubIq384J8W6dL1gL9b\/lw7NLiqAZ9CcrGElHX+Wb\/+cnegaAMzODhIBw451+CRAAAgAElEQVQcoE2bNuUNDH\/X0tJCq1evpt7eXsrlcrR48WLq7u6mvr6+gFN\/fz8NDQ1RY2Nj8LtU8d13y3pXAP56nJJKBf5JkdUvFxrAwOhHy\/UppcaPWAPDMy8LFiygEydOBGryDIyafeGf29raSM3OrFixgtjYDA8PB7MxythwGlcGJo6Ai1qGTvpqacr9Pcr3xWmr1WlzY5fLG0edUcvQSV8tTRTO5WIb\/CtHVCUNov6tVHrd75KI+3CZ1WJNp\/6oZVRLzzMw\/A9N\/ofnqlWrSl4C7oFrWBSLalx1tPQxjVgDo2CzMQkbGDXTsmjRosDAjI+P05o1a+jIkSPBchJ\/2MC0t7fnl5FYfHxAAARAAATcEqivr6dly5bR3NwcnTlzxu3FZKx2foeUtA8MjIaBkSY62gMCIAACIAACWScgwsBcvnw5mDUZGxujpqYmOnjwIPEMC3+KZ2B4A2\/UJaSsi4zrBwEQAAEQAAFpBEQYmEqihA2MMjRRN\/FKEx3tAQEQAAEQAIGsE6g5AxM+Rh0+nRQ+Rj0yMhJs8sUHBEAABEAABEDATwLiDYyf2HFVIAACIAACIAACNgRgYGzoIS8IgAAIgAAIgIATAjAwTrCjUhAAARAAARAAARsCMDA29JAXBEAABEAABEDACQEYGCfYUSkIgAAIgAAIgIANARgYG3rICwIgAAIgAAIg4IQADIwT7KgUBEAABEAABEDAhgAMjA095AUBEAABEAABEHBCAAbGCXZUCgIgAAIgAAIgYEMABsaGHvKCAAiAAAiAAAg4IQAD4wQ7KgUBEAABEAABELAhAANjQw95QQAEQAAEQAAEnBCAgXGCHZWCAAiAAAiAAAjYEICBsaGHvCAAAiAAAiAAAk4IwMA4wY5KQQAEQAAEQAAEbAjAwNjQQ14QAAEQAAEQAAEnBGBg3sF+6tQpWrduXfDb7t27qbOz04kgqBQEQAAEQAAEQKA6ARgYIrp06RJ1d3dTX19fQKy\/v5+GhoaosbGxOkGkAAEQAAEQAAEQSJ0ADAwR8ezL4OAgDQ8PU11dHfX29lIul6O2trbUBUGFIAACIAACIAAC1QnAwLxjYEZHR2lgYCAgxgamvb0dy0jV4wcpQAAEQAAEQMAJARgYDQPzgf\/z3ari\/Nq\/X6yaJpzg1\/79XwvSl8vP6d5z4ZlIZSMxCIAACEgkUF9fT8uWLaO5uTk6c+aMxCYm1qbJycnEynZVMAyMxhJS0+99mb70pS9V1OjCpcuRNPzZpbevS39h9tp309PT9MsbPxCpTJX4jsYbgh+5jPb\/fne+jNvf+f6Oxrr8dyrt7Q030Lp1Obrwo+9XrPPOO++kSjdCub9H+b44bbU6jSBVyRRHnVHL0ElfLU0UzoygVHrwrxwclTSI+jcd\/uV0SiLuw2VWizWd+qOWUS3966+\/HuxV5CX+VatWlbwE3APXsCgW1bjqaOljGhgYjU28Pot\/8sW5IK5+FjI\/F4rMkTJXYdPEZqk4XakAVQaH\/3ZHww3EJqjYAOXu\/VCise0z\/0Qb7knh4O9eCGhwVQM2MMeOHaP58+eXNTBJqJV1\/lm\/\/nKawsC8QyZ8jHpkZKRgA69U8UsFBZsa\/k8ZorDJYSOkTFAlA6RMT7Hh4e95pof\/HzZG1TqcWuJfjYWLv4O\/C+qFdUIDGBibKJQaPzAwGlEhVXyNpmslUSZHGZ9rv181POXMjjIxn7irPj+r03N\/y3V1vvzyy7Rw4UKta0Gi+AmAf\/xMo5YIDdwamKzzlzqGwcBo9CRSxddoemJJRp99Lb+E9cyLs0E9J1+6uhymPsFMTcMN1PCeK\/ThOz5IpcxNYheIgvMEst55S5ASGsDA2MSx1DEMBkYjKqSKr9H01JOo2ZtnXprLG5xj\/\/QqnX6lcNOzMjefuLshuEaYm+SkwuCZHFvdkqEBDIxurJRKJ3UMg4HRiAqp4ms03Ysk4c6bDY4yN2pPTnjmho2NWpLi\/y+\/u96LNmT5IjB4ulcPGsDA2ESh1DEMBkYjKqSKr9F0L5LodN5hY8NLUjA18Umnwz++2lBSKQLQAAbG5s6QOobBwGhEhVTxNZruRRKbzpuPmfOMTSlTk7v31mC2BrM0lWW24e9FAAm4CGgAA2MTxlLHMBgYjaiQKr5G071IEmfnzTM1vIGYP2FTw0tPMDSl5Y6TvxcBlcGLgAYwMDZhK3UMg4HRiAqp4ms03YskSXbe6rk3xbM0MDTXpE+SvxcBloGLgAYwMDZhKnUMg4HRiAqp4ms03YskaXfeatlp8OjLQfuVmeGfa\/G0U9r8vQg6zy4CGsDA2ISk1DEMBkYjKqSKr9F0L5K47LzVklN4uWn5XfXEx7drxcy45O9FAHpwEdAABsYmDKWOYTAwGlEhVXyNpnuRxKfOe\/DoFPHxbbWPRs3OSDYzPvH3IiAdXAQ0gIGxCTupYxgMjEZUSBVfo+leJPGx8w7vnQkvNfXcv5CSfrll2qL4yD9tBq7rgwYwMDYxKHUMg4HRiAqp4ms03YskWei8ed\/M6LOvBjMzalaGjUyUl1Z6AbvERWSBv6\/s4rouaAADYxNLUsewmjMw4bdOc0Ds3r2bOjs7Kfy9+k4FjFTxbW6INPNmqfNWe2bYzPDPEpaYssQ\/zbhMsy5oAANjE29Sx7CaMzCHDx+mqakp6unpycfDpUuXqLu7m\/r6+oLv+vv7aWhoiBobG4PfpYpvc0OkmTernbcyM2qJSS0vZW1WJqv804zRpOuCBjAwNjEmdQyrOQMzODhILS0twayL+vDsC38\/PDxMdXV11NvbS7lcjtra2mBgbO6amPJK6Lx5aWn0R68GrzjI2qyMBP4xhaKzYqABDIxN8MHA2NDzJC\/PtHR1ddHExERwRUuXLg1MywsvvECjo6M0MDAQfM8Gpr29PW9yWPzw5\/jx4560qDYuY3p6mpqbm0U0dubNK\/S9s2\/RgR\/NUdPN86jjnpvogSU3BT\/7+pHE31fG1a4LGlwlNDs7S6dPn6aGhgZqbW2thi22v2eR\/8qVKwvaPzk5GRsPXwqquRmYMHheThofH6c1a9bQkSNHKhoYieL7EoTVrkPivz6ztLwkkX+1mPPt79AAMzA2MYkZGBt6jvKeO3eONmzYQDMzM9TR0REYFF4iUh+1dLR582bav38\/lpAc6VStWumdNz9bJrzpd9\/aJV69YFI6\/2rx58PfoQEMjE0cwsDY0PMkLy8h7dq1i3bu3Bls0OV9L\/zZuHEjNvF6olGpy6iVzpuPYm958mz+9JIvz5SpFf4e3wIEDWBgbOITBsaGnkd5w8el1R4YNjPh70dGRvIbePnSpYrvkSwVL6XWOm9eXuKTS+qZMq5nZGqNv4\/3BTSAgbGJS6ljWE3vgdENCKni67bfdbpa7bzZyGwZPZs\/ueTKyNQqf9dxH64fGsDA2MSj1DEMBkYjKqSKr9F0L5LUeuft2sjUOn8fbgJoAANjE4dSxzAYGI2okCq+RtO9SILO+6oMYSPDb8Tel1uSyqsKwN\/9bQANYGBsolDqGOa9geGjztu3by\/QrvhR\/zbC6uSVKr5O231Ig867UAU2Mqv\/\/ExgaHijb9JvwgZ\/93cBNICBsYlCqWOYtwZGbaotZVaUqSnebGsjcKW8UsVPilfc5aLzLk00fGopSSMD\/nFHdPTyoAEMTPSouZZD6hjmpYHh485jY2O0fv36ipodOnSoahob0VVeqeLHwSaNMtB5V6bMz5HhU0tJvaIA\/NOI8sp1QAMYGJsolDqGeWlgbIRKIq9U8ZNglUSZ6Lz1qIaNTJwnlsBfj3+SqaABDIxNfEkdw7w2MMXvLgoL2NTURAcPHqRFixbZ6KqVV6r4Wo33IBE6b30Rwq8oiGujL\/jr808qJTSAgbGJLaljmNcGhgXj\/S5TU1PU09MT6Me\/82fBggXBCxi\/+c1v2uiqlVeq+FqN9yAROu\/oIoRPLNnujwH\/6PzjzgENYGBsYkrqGOa1gSl+9D8LqL576KGHaO\/evTAwNlGdkbzovM2F4qf58v4Y\/uTuvdXoxBL4m\/OPKyc0gIGxiSUYGBt6hnkvX75Mvb29xMtF4RkYfoP0tm3b6Iknnsh\/b1iFVjap4ms13oNE6LztRbDZHwP+9vxtS4AGMDA2MSR1DPN6BkbNuHR1ddHExESgH7+\/aN++fbRnzx5qb2+nzs5OG1218koVX6vxHiRC5x2PCOF3LEXZHwP+8fC3KQUawMDYxI\/UMSxTBubxxx+np59+Ojg6ncbmXRUwUsW3uSHSzIvOO17a4QfhXfrGb1UtHPyrIko8ATSAgbEJMqljmNcGRi0h8UwLb+RdsWJFoCFv3h0YGKC6urqqmhZvAg6fbOro6MiXE34bdfHD86SKXxWeJwnQeScjRHhZiTf65u79UMmKwD8Z\/lFKhQYwMFHipTit1DHMawMT3sT72GOPBQZm8eLFtGvXLtq5cyc1NjZW1HRwcJAOHDhAmzZtyu+V4e9aWlpo9erVwf6aXC4XlNnd3U19fX1Bef39\/TQ0NJQvX6r4NjdEmnnReSdHO3zsmh+E99QXll33fiXwT46\/bsnQAAZGN1ZKpZM6hnltYErNwJw4cYJmZmaqzsDwzAsfteb0\/OFNwGr2hX9ua2vLH9FmY8TGZnh4OJjVUcaG0\/BHqvg2N0SaedF5J0+70rFr8E+ef7UaoAEMTLUYqfR3qWOY1waGBSl+mB1v4mWjUW32RYnJxiRsYNRMC++hYZPDJ5rWrFlDR44cCUwRf9jAhDcIs\/jhz\/Hjx21iCXkjEpienqbm5uaIuZDchMDpV96mzx95jZpunkcd99xEn\/94PYG\/Ccl480CDqzxnZ2fp9OnT1NDQQK2trfFCrlBaFvmvXLmyoEWTk5Op8UqrIu8NjC2IuAyMRPFt2aaVH\/\/6TIv0tXrC+2N2fKqe\/mD5kvQvAjXmCeAeuIri9ddfp2PHjtH8+fNp1apVqUVI1vljBia1ULl+1qW46uJZGLXUxC+ALH7FQLGB4SPZWEJKUcwYqsp65xEDAidFmB67dnKxwivFPQADYxPiMDA29CzylnuVgO7zX8IGhi8Dm3gtxHCUFZ23I\/DvVPvMxDn64ti\/Ehsa29cSuG1JdmvHPQADYxO9MDA29AzzVnqVgM4pJGVY+P\/qSb7hPTXh00nhY9QjIyPBJl\/1kSq+oSypZ0PnnTryggoV\/\/CykulrCdy2JLu14x6AgbGJXqljmNd7YMKnkNSMC8+g6JxCshG7OK9U8eNklGRZ6LyTpFu97DD\/4mPX+9YuoeV311cvBCmsCOAegIGxCSCpY5jXBoYFsz2FZCM6ZmDioGdfBjpve4Y2JZTiDyNjQzR6XtwDMDDRo+ZaDhgYG3oZzytV\/KzIgs7brVKV+IefH8NP8v12DqeVklAL9wAMjE1cSR3DvJyB4VkXPlHE7zyq9Dl06FDVNDaiYwYmDnr2ZaDztmdoU4IO\/5MvztGWJ89io68N6Ap5dTRIqGqvisUxajM5YGDMuBnnUptqi99LxAXyyaTt27dT8WZb48qqZJQqflK84i4XnXfcRKOVF4W\/2ujLNeDEUjTOlVJH0SC+Wv0rCQbGTBOpY5iXMzBhiZRZCX9XytSYyaqXS6r4eq13nwqdt1sNTPjjxFK8mploEO8V+FEaDIyZDlLHMO8NjJlc8eaSKn68lJIrDZ13cmx1SrbhDyOjQ7h6GhsNqpeenRQwMGZaSR3DYGA04kGq+BpN9yIJOm+3MtjyLz6xhGfIRNfTVoPoNfqZAwbGTBepYxgMjEY8SBVfo+leJEHn7VaGuPiHjQy3CHtk9HWNSwP9Gv1MCQNjpovUMQwGRiMepIqv0XQvkqDzditD3PxhZKLrGbcG0a\/AjxwwMGY6SB3DvDcw4QfZPf744\/T0008HR6cXLVpkpqRBLqniG6BwkgWdtxPs+UqT5M97ZEaffTV\/\/JqfJXNH4w1uG+xh7Ulq4GFzy14SDIyZWlLHMK8NTPhVAlNTU7RixYpAvdHRURoYGKC6ujozNSPmkip+RAzOkqPzdoY+qDgN\/uE3X7OB4eUlNjP4XCWQhgZZYA0DY6aS1DHMawMTfpnjY489FhiYxYsX065du0j3ZY5mchfmkip+HGzSKAOddxqUy9eRJn9s+C2tQ5oauI22yrXDwJipI3UM89rAlJqBOXHiRKSXOfJzZHj2Rr2NOvzWaQ4F9UyZ8PfFz5mRKr7ZrZB+LnTe6TMP1+iCf6l9MrW8vORCA7dRV7p2GBgzVaSOYV4bGJbK5mWO\/ObqAwcO0KZNm\/IGptjQqDq6u7upr68viI7+\/n4aGhqixsbG4Hep4pvdCunnQuedPnPXBiZcP++TeebFWTr50lywP6YWj2HjHrgaETAwZn2B1DHMewOjDMfGjRupq6uLJiYmtF4hwPkWLFhAPGPDHzUDw6ampaWFOjs785HAsy\/8\/fDwcLCvpre3l3K5HLW1tcHAmN0vseZC5x0rzsiF+cK\/1PLSJ+6qp+V310duU9Yy+KKBa24wMGYKwMCYcbPKxUtIjzzySHDq6LnnnguWgtasWUP8EseHH35YaxMvGxNlYMrN5rzwwgv5jcGclg1Me3t73uSw+OHP8ePHrdqFzNEITE9PU3Nzc7RMSB0bAR\/5P\/qPc\/TjV96m06+8TU03z6OOe26iB5bcFPws8eOjBi44z87O0unTp6mhoYFaW1tTu4Qs8l+5cmUBn8nJydR4pVWR1zMwxZt4eebkvvvui7SJN2xgiqHyLM34+Hhgio4cORKcbCpnYCSKn1aQ2daDf33aErTL7zN\/NSujjmJLXWLyWQO76IqWGzMw0Xip1JiBMeNmlUvNwDzwwAPBXha1R6V4BkZt9h0bG6OmpiY6ePBg\/jkxlQyMWjravHkz7d+\/H0tIVmollxmdd3JsdUrOCv\/ijb+SzExWNNCJJ5s0MDBm9GBgzLhZ51Kng3gjLu+DUZttdR9kV7yEFD6Crf4WLpcvGJt4rWWLtQB03rHijFxYFvmXMjM\/2fGbkdvuS4YsapAEOxgYM6owMGbcnOcqnoEJH5deunRpMOvCp43C34+MjOQ38HIDpIrvXBzNC0DnrQkqoWRZ53\/yxTl65qU5Gjz6ckBIzcxk6Vh21jWIKzRhYMxISh3DvN4Dw1Kpo9Bh2cLGw0zOaLmkih+NgrvU6LzdseeaJfHP6jKTJA1sohkGxoye1DHMawPDm3jVkhGfQuJj0efPnw8UDB+DNpNUP5dU8fUJuE2Jzhv8kyBQagMwH8u+o7GOeu5vSaJK4zJxD1xFBwNjFkJSxzDvDYzas8JHnfmZLrxfBa8SMAvirOZC5+1WuVrgr8yMemCeWmryZd9MLWigE+UwMDqUrk8DA2PGzSoXny7au3dvYFouXrxIGzZsCF4jgCUkK6yZy4zO261ktci\/1L6ZOxpuoE\/c3RC8ZDLtN2bXogaloh4GxqwvgIEx42adi5+8e+ONNwbHonmj7datWwuOSVtXoFGAVPE1mu5FEnTebmWodf5qdubCpcs0+uxrgRhqIzD\/nMZyU61roO4AGBizvkDqGOb1EpKZVPHnkip+\/KSSKRGddzJcdUsF\/0JS5Zab+B1NSb3aABpc1QAGRveuLUwndQzz3sDw03K3b99eoAaWkMyCOKu50Hm7VQ78y\/NnM8P\/8THt4v0zcRoaaAADY9MLwMDY0DPMq95dxC9iVC9WNCzKKptU8a2gpJgZnXeKsEtUBf7R+Kv9M+r1BuElJ9PlJmgAAxMtCjEDY8MrlrzhdyHxw+ZcfWBgXJG\/Wi86b\/B3S8C8djU7w\/9XD9ILGxrdJSfcAzAw5lEo92GsmVhCYuHSfO5LcaDAwNjcOvZ50XnbM7QpAfxt6BXmVftn+Nvwk4HVCadyhgYaXDMwSwb\/hf7q9\/6TVq1aFZ8wVUrKOn+pY5iXBkYtHfEJpFIf7IFJ7b71oqKsdx5eQLS4CPC3gFclazlDox6op45sQwMYGJsohIGxoZfxvFLFz4os6LzdKgX+6fEvd8Lp03ffQP\/jrluDZ9DU8odPIWEGJnoESB3DvJyBYXnOnTuXf3BdR0cHDQwMUF1dXXTlYsghVfwY0KRSBAbQVDCXrQT83fIfPDpFx\/7pVTr9ytvBhahn0Ojun3F79fHWDgNjxlPqGOalgeEn8Pb29lIulwtOH\/ELHVtaWiLtgwkfvw4vOYWXp8LGKPw26t27dxfUJVV8s1sh\/VwYQNNnHq4R\/N3y59pZg19\/\/635I9vFb9bmNKYnnNy3Tv8KYGD0WYVTSh3DvDQwxaePeDbm6NGjtGXLFi31OH1\/fz8NDQ0Rn15iA8SvIOBZHH41AZuh1atX503S4sWL8y+N5ArCefl3qeJrwfQgEQZQtyKAv1v+ysAsXLjwugvh2ZlSz5+RamZgYMxiUeoYlhkDc+jQIXr44YeNlpF4dmV0dJS2bdsWmCD1XBmepZmamqIVK1YEJmd4eDgoPzz7AwNjdsPEmQsDaJw0o5cF\/tGZxZ2jmgbhB+oVz864eHdT3O1X5cHAmJGFgTHjZpSr1AyMjYFRS1D33XdffqaF363EBmZ8fJzWrFlDR44cCWZo+MMGpr29Pb+MxOKHP8ePHzdqFzKZEZienqbm5mazzMhlTQD8rRFaFxBVg5k3r9D3zr5FY8+\/Rfxz083zqOOem6i1+QZqve0G6+txVcDs7CyteuLn9Gj7RWptbU3tMqLyT+3CKlS0cuXKgr9OTk76cFmxXoO3MzBdXV0UxzFqNcvCsy5sjLq7u6mvry94OWQUAyNR\/FgjKcHCqv3rM8GqUTQeJOhFDNjcA+pkk3oysNoEnMWZGczAmIUjZmDMuKWSS236HRsbo6ampvzbqos3\/xa\/mgBLSKnIY12JTedtXTkKwJOQPYiBOO8B3jdTbGaysmcGBsYsGGFgzLg5y8Xmhfe2FL9DSZkabOJ1Jk3kiuPsvCNXjgwwMB7EQBL3gJqZKd4z47OZgYExC0YYGDNuTnKFj0SrC1BHpnm2Ri1Pbdq0KdjQy59wnpGRkQLjI1V8J+IYVJpE521wGTWbBfzdS5+0BmEz4\/MSEwyMWSxKHcO83ANjJlFyuaSKnxyxeEtOuvOO92rllQb+7jVNS4Ny+2V8mZWBgTGLRaljGAyMRjxIFV+j6V4kSavz9qKxHl4E+LsXxYUGpWZlXBsZGBizWJQ6hsHAaMSDVPE1mu5FEhedtxcN9+QiwN+9EK41CG\/87bl\/YfBOJl5qSvsDA2NGXOoYBgOjEQ9SxddouhdJXHfeXkBweBHg7xD+O1X7okHYyCy\/q56e+uKyVOHAwJjhljqGwcBoxINU8TWa7kUSXzpvL2A4uAjwdwC9qErfNODlJT69NPrsa8FMzL61S2j53fWJg4KBMUMsdQyDgdGIB6niazTdiyS+dd5eQEnxIsA\/RdhlqvJVg7SNDAyMWSxKHcNgYDTiQar4Gk33IomvnbcXcFK4CPBPAXKVKnzXgI3MltGzdPKluWBG5ic7fjMRaLyE9djfvUjfWUW0atWqROooVajv\/KuBkDqGwcBUUx5vo9YglGySrHceydJJvnTwT55xtRqyokHYyPAemX25JbFu9oWBqRYppf8OA2PGTUQuqeJnRZysdN5Z4Rn1OsE\/KrH402dNg7CR4VNLcR2\/ZgNzZPx5euR\/zcMMTIQwkzqGYQZGIwikiq\/RdC+SZK3z9gJajBcB\/jHCNCwqqxqw4eDNvurpvrZG5oujZ+nHz5+HgYkYR1LHMBgYjUCQKr5G071IktXO2wt4MVwE+McA0bKIrGsQNjI2J5ZWf\/sMvf7\/XoeBiRhPUscwGBiNQJAqvkbTvUiS9c7bC4gWFwH+FvBiyipBg\/CJJX4Q3rdzSyLTYQPzH3Mz1P3x92IJKQI9qWMYDIxGEEgVX6PpXiSR0Hl7AdLwIsDfEFyM2SRpcPLFOdry5NmATu7eWyPtj\/nI1\/6B\/tt736DPLbsRBiZCfEkdw8QamMOHD9P27dsDiZcuXUrDw8PU2NhY8NZp\/tvu3bups7Oz4Hv1nYoPqeJHiH+nSSV13k5BGlYO\/obgYswmUQO1rBTltBIMjFlQSR3DRBqYc+fOUX9\/Pw0NDQWmZXBwkGZmZmhgYICeeuopmpqaop6ennwkXLp0ibq7u6mvry\/4LpyXf5cqvtmtkH4uiZ13+hTNawR\/c3Zx5ZSqQfi00qVv\/FZFXJyWDcy3Wmdo\/vz5mIGJEFxSxzCRBqZY11OnTtHo6GhgYPbu3UstLS3BrIv68N\/Z5PAsTV1dHfX29lIul6O2trYgiVTxI8S\/06RSO2+nUCNUDv4RYCWUVLoG\/EoCPmFU6bUEvPS0+s\/PwMAYxJjUMawmDAybEzYt9913H3V1ddHExEQQAmpp6YUXXsgbHP6eDUx7e3ve5LD44c\/x48cNQghZTAlMT09Tc3OzaXbksyQA\/pYAY8heCxrMvHmFvnf2LTrwozna9LF6+vzHC9+t9Og\/ztFfnXmDeu8+Tw0NDdTa2hoDWb0issh\/5cqVBY2bnJzUa2yGUok3MLwXpnjJSOnDfxsfH6c1a9bQkSNHghmacgZGovhZiVPp\/\/r0XQfwd69QLWmgNvnykpF6CB7\/zLMvi276Bf3+B6exhBQxJDEDExFYmskvX74czJqMjY1RU1MTHTx4kBYtWhQsCxUvF4WvSy0dbd68mfbv348lpDRFi1BXLXXeEbCklhT8U0NdtqJa1EBt8lVQeHnp6MY76dixYzAwEUMSBiYiMNfJ2bysWLEiv4+Fr4c36+7atYt27tyZ39zL32\/cuBGbeF0LVqH+Wuy8fZID\/N2rAQ2uasBvo4aBiR6PMDDRmTnLwTMr69atK6i\/o6MjWCLi\/S\/qb+WOV4+MjBQYH6niOxMoYsXovCMCizk5+McM1KA4aAADYxA2+SxSxzDxe2BsRFd5pYofB5s0ygD\/NCiXrwP83fLn2qGBWwOTdf5Zv\/5ydyAMjEbfJFV8jVmmHiQAAAlASURBVKZ7kQT83coA\/m75w8Bc4+9qCSnr90DWrx8GxqIPciF+HHVGLUMnfbU05f4e5fvitNXqtJC2bNY46oxahk76ammicC43MIJ\/5YiqpEHUv5VKr\/tdEnEfLrNarOnUH7WMaunZwPADR\/k5XatWrSp5CbgHrmFRLKpx1dHSxzSYgdFQhcXHBwRAAARAwC2B+vp6WrZsGc3NzdGZM2fcXkzGapf4KBAYmIwFIS4XBEAABEAABECACAYGUQACIAACIAACIJA5AjAwmZMMFwwCIAACIAACIAADYxAD\/OTf7373u\/TTn\/6UHnzwQVq4cKFBKchiQ+DKlSv0N3\/zN\/Q7v\/M7wXtR8EmXwOzsLP3lX\/4l8Tti1q5dSx\/5yEfSvYAar+3NN98M+LMO69evp9tuu63Gibhr\/lNPPUW33357sDcHn3QJwMAY8P7BD35At9xyS\/C6Au5EPve5zwVvscYnHQI\/\/\/nP6bHHHgvecfUnf\/InwVOV8UmXAL9H7J577gn+e\/TRR+mP\/\/iPYSRTlOD73\/9+8JqUD3zgA0Ef9IUvfIHe8573pHgFqIoJ8IuAuS\/67Gc\/W\/DwU9BJhwAMTIgzv2qgu7s7OKbH5oQ\/3FFv3749+Fk9oZc77E9\/+tOB6z506BDxU34xiNoHrC5\/ngH75S9\/SX\/9138N9vbYC0rQ1UBl4hmAv\/iLv6BNmzbRTTfdFPPV1F5xUfjzLAw\/Vv\/d7343PfDAA\/Sud72r9oAl0GJdDd566y3627\/92+C9TBz7bW1tCVwNiqxEAAbmHTrnzp2jDRs2BL+pl0Hyd\/39\/TQ0NBQ47dHR0eB1BPx\/Ni0f\/OAHYWBiur+i8FezXTCPMcE3uAdYAzYve\/fuDZ7JsXjx4ngvpgZLi3oP\/OIXvyB+xQDPxrCBxCywfdDoarB79276+7\/\/e\/qN3\/gNeuONN4KKYWDs+UctAQbmnZc88jTg7\/7u79JXv\/pV2rNnTzADw7Mv4+PjgWnhf\/Wr2Zl\/\/ud\/pg9\/+MPBFC4Pop2dnXTzzTdHZY\/07xDgf\/FE4a9mx2Bg4guhqBq8\/\/3vDzTj5VPsv7DXISp\/fqAb90G8\/+s73\/kOrV69GjpYyhBFg61bt9LZs2fpZz\/7Gb3yyiuBDl\/60pcwC2mpQdTsMDAhYuy+t23bVmBgeJ9FT09P8Cbrrq6u4GdeOuJlJA5aHkw\/85nPROWO9CUI6PJX\/9KBgYk\/jHQ04HuEHyLGHTfvwWAz84d\/+IfovGOQQ4c\/90G8ZM3\/wOKlC\/75j\/7oj2jevHkxXAGK0NVA9UP88mDMwLiJGxgYAwODqcJkgjVqx5HMVdR2qdDArf5R+PM+MP4PxiVezaJoEG\/NKC0qARiYKgam1BKSWsKIChvpKxMo1XGAf7pRAw3S5V1cG\/i75c+1QwP3GuheAQxMBQNTbhMvNsvphle0dMUdB\/hH4xdHamgQB0XzMsDfnF1cOaFBXCSTLwcGpoKB4T+pY9RNTU3500nJy1KbNRR3HOCffhxAg\/SZh2sEf7f8S83AoB9yr0m5K4CB8VcbXBkIgAAIgAAIgEAZAjAwCA0QAAEQAAEQAIHMEYCByZxkuGAQAAEQAAEQAAEYGMQACIAACIAACIBA5gjAwGROMlwwCIAACIAACIAADAxiAARAAARAAARAIHMEYGAyJxkuGARAAARAAARAAAYGMQACIAACIAACIJA5AjAwmZMMFwwCIAACIAACIAADgxgAARAAARAAARDIHAEYmMxJhgsGARAAARAAARCAgUEMgECNEbh8+TL19vbS2NhYQcs3bdpEPT09mabB7xI6evQodXV1BW1sb2+nzs7OoE2q3blcjtra2q5rJ\/997969tHHjRmpsbMw0B1w8CNQCARiYWlAZbQSBEAE1kIcHdwmAwgaE3xgf1cAwA2WAtmzZIgEJ2gACognAwIiWF40DgesJVDIw\/Pb18fFxunDhAq1du5Y++tGP0oYNG2hmZoaWLl1Kw8PDwezEqVOnaN26dcRvaf\/Upz5F73vf+4KZC5754FkcnuHgsqampoLf1Vvd+WrUTA+XceDAgeACT5w4QR0dHTQwMEBsPgYHB\/N\/GxkZCdKMjo4Gf+cPm5PimRQu7\/z588GMS6k2hmdguD5VN5cXrnvfvn10\/\/3306JFixA+IAACHhOAgfFYHFwaCCRBoNQSkjInP\/zhD+nJJ58MjAp\/uru7qa+vLxjMlSEJGxXOx2aCjUw5A7NixYqS5oPL37p1Kx08eJBuueWWvPnh79nA8DVcvHiR+vv76eGHH6ZvfetbtHPnzvx3Q0NDBUs9nIfrYvNUbpmMy2ZDFF5CCufjv3M7+aOWnpLQAGWCAAjYE4CBsWeIEkAgUwR0ZmB4pmN6ejo\/+6IayIZl8+bNtH\/\/\/vxsTCljE56BaWlpoe3btxcw4lkYNhvKqKglH55V4VkUNXMTzqSMBn\/HMyjh\/TrcpkceeYTWr18fmK1qMzDKwBSbFy6bZ3KKy8+UwLhYEKgRAjAwNSI0mgkCikAUA8OzH8UzHTzAK+PBy0k6BqaUIQmXo2NglLHgdqiZlnCbohqYcjMtMDC4V0AgGwRgYLKhE64SBGIjoGtgOB3vaeG9MLycovbHbNu2jXiTK8+AhJeQHnroofzG2dWrV+eXlthsqKWi5ubmfJoFCxaUnIHhhoaXkLi+PXv2EOflU0LPP\/\/8daZK5SleQip3ColnefhTapkIS0ixhRoKAoFECcDAJIoXhYOAfwR0DQzPivCpHN1NvOHNuuHNvZU28ZZaQlLLT2rZaffu3XmjEd4YXEw2PHNSaQnpM5\/5TLAENjExkS9C7QHiNodncvxTD1cEAiCgCMDAIBZAAASsCFQyFVYFF2VO4zkuOEYdp2IoCwSSJQADkyxflA4C4gmkYWAuXboULGfdcccd+aPWpcDa7F8p3ggsXjg0EAQyTgAGJuMC4vJBAARAAARAoBYJwMDUoupoMwiAAAiAAAhknAAMTMYFxOWDAAiAAAiAQC0SgIGpRdXRZhAAARAAARDIOAEYmIwLiMsHARAAARAAgVokAANTi6qjzSAAAiAAAiCQcQIwMBkXEJcPAiAAAiAAArVIAAamFlVHm0EABEAABEAg4wRgYDIuIC4fBEAABEAABGqRAAxMLaqONoMACIAACIBAxgn8f8SfUykZMHE4AAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:97e357dd]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"num","rows":1,"type":"double","value":[["0","0.002443525700815","0.002362998398568"]]}}
%---
%[output:9d1224cc]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000","-1.901953846588630","0.904357108638321"]]}}
%---
%[output:26f707e8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"5","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000010000000000"],["-2.526618726678876","-0.000251327412287"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-25.266187266788759","0.997486725877128"]]}}
%---
%[output:6e27c0a7]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:7996ad92]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:9ad8d2a8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":" -25.266187266788759"}}
%---
%[output:637faca4]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.997486725877128"}}
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
