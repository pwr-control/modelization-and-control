%[text] ## General Settings
%[text] ### Simulink model initialization
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
simlength = 0.5;
transmission_delay = 125e-6*2;
s=tf('s');

%[text] ### Voltage application
application400 = 1;
application690 = 0;
application480 = 0;
%[text] ### PWM and sampling time and data length storage
fPWM_AFE = 10e3; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = 10e3; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = 10e3; % PWM frequency 
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
tc = ts_dab/100;

z_dab = tf('z',ts_dab);
z_afe = tf('z',ts_afe);
z_inv = tf('z',ts_inv);

t_misura = simlength - 0.025;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

dead_time_INV = 0;
delay_pwm = 0;
%[text] ## Grid Emulator Settings
grid_emulator;
%[text] ## Grid Voltage Rectifier
%[text] ### Nominal DClink voltage seting
Vf_diode_rectifier = 0.7;

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
V2rms_load_trafo = V1rms_load_trafo/m12_load_trafo %[output:2a12e11d]
I2rms_load_trafo = I1rms_load_trafo*m12_load_trafo %[output:923f0700]
lm_load_trafo = V1rms_load_trafo/I0rms_load_trafo/2/pi/ftr_nom_load_trafo;
rfe_load_trafo = 2e3;
rd1_load_trafo = 1e-3;
ld1_load_trafo = 500e-6;
rd2_load_trafo = rd1_load_trafo/m12_load_trafo^2;
ld2_load_trafo = ld1_load_trafo/m12_load_trafo^2;
%[text] ### Current sensor endscale, and quantization
Pnom = 200e3;
margin_factor = 1.25;
adc_quantization = 1/2^11;
Vdc_FS = Vdc_nom * margin_factor;
Idc_FS = Pnom/Vdc_nom * margin_factor;
Vac_FS = V1rms_load_trafo*sqrt(2) %[output:4ca1a094]
Iac_FS = I1rms_load_trafo*sqrt(2) %[output:9bdb6f58]
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
kp_inv = 0.5;
ki_inv = 45;
%[text] #### Phase shift filter for Q component derivation at 50Hz and 80Hz
freq = 50;
temp = 2*pi*freq*ts_inv;
% a_phshift = (sin(temp) - cos(temp)) / (sin(temp) + cos(temp))
a_phshift = -0.951005;

% flt_dq = 2/(s/(2*pi*50)+1)^2;
% flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d, options); grid on
% [num50, den50]=tfdata(flt_dq_d,'v')
% 
% flt_dq_80Hz = 2/(s/(2*pi*80)+1)^2;
% flt_dq_80Hz_d = c2d(flt_dq_80Hz,ts_inv);
% figure; bode(flt_dq_80Hz_d, options); grid on
% [num80, den80]=tfdata(flt_dq_80Hz_d,'v')

% flt_dq = (s/(2*pi*50)-1)/(s/(2*pi*50)+1);
% flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d, options); grid on
% [num50, den50]=tfdata(flt_dq_d,'v')
% 
% flt_dq_80Hz = (s/(2*pi*80)-1)/(s/(2*pi*80)+1);
% flt_dq_80Hz_d = c2d(flt_dq_80Hz,ts_inv);
% figure; bode(flt_dq_80Hz_d, options); grid on
% [num80, den80]=tfdata(flt_dq_80Hz_d,'v')
% 
% flt_dq_d = (a_phshift + z_inv^-1)/(1 + a_phshift*z_inv^-1);
% figure; bode(flt_dq_d, options); grid on

%[text] #### Resonant PI
rpi_enable = 0;

omega_set = 2*pi*freq;
kp_rpi = 2;
ki_rpi = 45;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta*omega_set] %[output:4f337f7f]
Aresd_nom = eye(2) + Ares_nom*ts_inv %[output:37b63c7b]
a11d = 1 %[output:0f6f0f96]
a12d = ts_inv %[output:9dfdaf92]
a21d = -omega_set^2*ts_inv %[output:0b4e6f02]
a22d = 1 -2*delta*omega_set*ts_inv %[output:81251e8a]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:9a6d4274]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:686ca032]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:3e05b6e0]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:4e5d2adb]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:2a8ed490]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:0de50622]

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
figure;  %[output:42eb2e28]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:42eb2e28]
xlabel('state of charge [p.u.]'); %[output:42eb2e28]
ylabel('open circuit voltage [V]'); %[output:42eb2e28]
title('open circuit voltage(state of charge)'); %[output:42eb2e28]
grid on %[output:42eb2e28]

%[text] ## IGBT and snubber data
%[text] ### HeatSink
weigth = 0.050; % kg
cp_al = 880; % J/K/kg
heat_capacity = cp_al*weigth % J/K %[output:3da50f8a]
thermal_conducibility = 204; % W/(m K)
Rth_mosfet_HA = 30/1000 % K/W %[output:9479f5e4]
Tambient = 40;
DThs_init = 0;
%[text] ### SKM1700MB20R4S2I4
Ron = 1.04e-3; % Rds [Ohm]
Vdon_diode = 4; % V
Vgamma = 4; % V
Rdon_diode = 1.85e-3; % V
Eon = 77e-3; % J @ Tj = 125°C
Eoff = 108e-3; % J @ Tj = 125°C
Eerr = 9.7e-3; % J @ Tj = 125°C
Voff_sw_losses = 1500; % V
Ion_sw_losses = 2000; % A

JunctionTermalMass = 4; % J/K
Rtim = 0.05;
Rth_mosfet_JC = 19/1000; % K/W
Rth_mosfet_CH = 6/1000; % K/W
Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH % K/W %[output:2957f2e8]
Lstray_module = 12e-9;

Irr = 475;
Csnubber = Irr^2*Lstray_module/Vdc_nom^2 %[output:1f6650e6]
Rsnubber = 1/(Csnubber*fPWM_INV)/5 %[output:42540868]
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
%[output:2a12e11d]
%   data: {"dataType":"textualVariable","outputData":{"name":"V2rms_load_trafo","value":"     6.600000000000000e+00"}}
%---
%[output:923f0700]
%   data: {"dataType":"textualVariable","outputData":{"name":"I2rms_load_trafo","value":"       30000"}}
%---
%[output:4ca1a094]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     4.666904755831214e+02"}}
%---
%[output:9bdb6f58]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     8.485281374238571e+02"}}
%---
%[output:4f337f7f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ares_nom","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:37b63c7b]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000e+00","1.000000000000000e-04"],["-9.869604401089360e+00","9.984292036732051e-01"]]}}
%---
%[output:0f6f0f96]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:9dfdaf92]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:0b4e6f02]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"    -9.869604401089360e+00"}}
%---
%[output:81251e8a]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"     9.984292036732051e-01"}}
%---
%[output:9a6d4274]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["1.490161953970131e-01"],["3.652194550246457e+01"]]}}
%---
%[output:686ca032]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:3e05b6e0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:4e5d2adb]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","1.000000000000000e-04"],["-9.869604401089360e+00","9.984292036732051e-01"]]}}
%---
%[output:2a8ed490]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["1.555088363526948e-01"],["2.716608611399846e+01"]]}}
%---
%[output:0de50622]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["3.719106107041118e-02"],["1.937144673860303e+00"]]}}
%---
%[output:42eb2e28]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAArwAAAGlCAYAAAAYiyWNAAAAAXNSR0IArs4c6QAAIABJREFUeF7svQuYFcW1NrxQxKCDgIKOgBECaiZeYLygBjXmYCAx6o+eiEgUDEHBQc\/JjyCGkBg9xHwEJBIVQYgBVMA5RvEeUD+TEVQQA2gMalBIMg4gKNeAoIbvWY217Wl6767uruvMW8\/jAzJVa61+39W136m9qqrJnj179hAaEAACQAAIAAEgAASAABBooAg0geBtoMzisYAAEAACQAAIAAEgAAQCBCB4kQhAAAgAASAABIAAEAACDRoBCN4GTS8eDggAASAABIAAEAACQACCFzkABIAAEAACQAAIAAEg0KARgOBt0PTi4YAAEAACQAAIAAEgAAQgeJEDQAAIAAEgAASAABAAAg0aAQjeBk0vHg4IAAEgAASAABAAAkAAghc5AASAABAAAkAACAABINCgEYDgbdD04uGAABAAAkAACAABIAAEIHiRA0AACAABIAAEgAAQAAINGgEI3gZNLx4OCAABIAAEgAAQAAJAAIIXOQAEgEBJBBYsWEBDhw6lFi1a0KxZs6hr167GEdu+fTsNHjyYlixZQn369KGJEycWYuD4xo4dS9XV1VReXm40thUrVtCAAQNo27ZtNGXKFOrVq1fgX8R7zjnnUFVVldGYkpwNHz6c5s2bVy\/epDHi5yqea\/LkyVRTU0PTp0+nsrIyWdfK+oU5Y6MdOnTIlTulclNZ0AYMCVwqKiqscWPgMeGiESMAwduIycejAwEZBFwWvCyeJkyYkFu0yOAQ1ydO8K5bt4769u1LtbW1NGLECKcEr4i3ZcuWqUWeiucSYrt79+5WRFVYnAo+88bSUARv+DnCv7xlfTcwDgi4hgAEr2uMIB4gAASkEYDglYYq6Cjwiq6Sy1hpaII3CwZxODUUwZs3P2RyCH2AgE0EIHhtog\/f3iMgVj\/Fg8R97R9e1brsssvohhtuKDx3sRVAMUZ0jPaLfsh++9vfDsoOivUvBnRYxBQbG7fCG36myspKmjp1ajA8HKcQV8Ju9KvjYmI1jKlYaYo+76233loocQg\/W7HVumIri+Hnj65qyXAbXeHlWMI8iNjCtqPccp+4FbXoV+\/cZ9WqVbEr2jJf06d5Vo4pLAijWKR9rrg8i\/qQeYZSE0YSX1H7su9K3Li48hVRbiPzLkbfjei7w\/8v846Fc4lz\/7bbbqOrrroq9tuFpDmFfYpn5b\/bKl\/y\/kMBD+AsAhC8zlKDwFxHIE64xH2IluoX\/dCP+8pV2AwLkFL98nyQx\/kqJXjDHIXFfrFnDvcxKXiLlWWIf4+KcVlu0wreUnbDIqqYwIz75aFY3+gvX0kYxL1vIueSBG\/Sc5100kmFMo+wnyT7snXjMnxlEbyleBC\/3Mm8i2Fu48Su7Lwh8OjcuXPsL3xhbGXii65yq1jFd33eRnyNFwEI3sbLPZ48BwLhD8Lwqqb44C0m\/sIfMHF9xYdheHycUIl+yIoP1PCHeqnaxPD4sNiLiylJ8EZXn+OwCcclMMgjeMWmNdmShrgP8mJfRafhNk0Nb9zqWTgugUsxbsJxCc44hUW9cNz4cL4Vwypu9TsuD4uJIdnniq5aik1rSRgklR6k4StN+UE4LvEu8TOIzZOCA954J\/6Nfy7exbjniltlD8cUfmfDIl7mHYvOCWKM7JzCsafBJ8f0iaFAwAoCELxWYIdT3xGQ+YpcfOCIvtFVxKiA4N3+cScRhD+E4lZtosJWZmNQsdMF4k48KCV443a4x\/mP291vUvDGia1333039oSFNNymEbzRnI+u9AlhF7YZFTrRXHr99ddjT9CIW7ku9lxhYVVKXMqu\/hV7rmKCN2nlOekUhTR8pRF0xeKKnjJRTLAWe95wHkRXkOMEb6l3LPqzaO6kmVNEXDLzh+\/zN+JvnAhA8DZO3vHUORAo9aEZ97NiHyDRvjfeeGPs177hUJNW8WQ+ZLlPMcEbB0tSDW\/0eCnZD0zTgje6Erlo0aJ96mHTcptW8Jb6OjtO8EZre6OYPfLII8EzFGtxX4FHRW3cV\/1xpQSlBK\/McxXLzVJjeUypsoa0fKkQvFGsS9mMexdKlUnECd64b2pkRf4ll1wiPaeI55L91iTHFIqhQMAKAhC8VmCHU58RSPshC8Ebf9aqacEb5m3IkCG0bNmyfc71TcttGsEbFjpCxB1xxBH7lCSU+mVEh+AV72KcEAuvIBYTvLLPBcE7ncLfKjAe\/AvN17\/+9cI3OxC8Pn8yIHbXEYDgdZ0hxOckArIrLHzpQN6ShjgA0q4qRW0U+9pc\/Pv48eMLlyhkXeGN2whWV1dXOH\/VtOBlDITP9u3b09atWwNYorvR03CbRvAK32FRE1fnqaKkIc0qZFx+xcVQTPDKPlcxwVusdED2xU\/DV5YVXiFMxaUiHO\/IkSMLeZPmXXzppZeCEpTwu5FUw1tqhTdrSUMpbOP4lOUC\/YCAywhA8LrMDmJzFoE0G2WK1UjKblqLE1VpPmTjbrMqtjEq\/PWy+Do9reCNwyZuA5D48GeSRa1q9PiqYseSpd20JhIp+vV9nJhIw20WwRt3UgXHp2rTWjFhWaq2mo\/UShLiSYI36bmKxRUn+ov1jZsQ0vCVRvDG5Sy\/S9H3NnxiQrRcJIp5OOej7xc\/m+wKb9wzp9m0VupbBNmSJGcnZwQGBIogAMGL1AACGRFIOopJrAiV6hcWOvz3YueVRj8M8wpetlfsmKaor7SCNyxW4qCNO1GiGAVJgrfUpp84m8VEQbSvLLdJv4wIu\/wcXL4griGOi03m3NuvfvWr9NZbb9VbISxVAxt3HFZ0VbBUTWlYxEax42dI+1zFNrTJPkOxPJHlK43gZV+lsMlST8\/+xakacc8iK3jjuGB74psLvuq62C+RYb\/RX\/jS4pNx6sQwIGAFAQheK7DDaUNBIPqBmHTxxH\/913\/RtddeS\/yBxE324onoypEKwVtMYEd9ZRG8bDsqYuKwiVtxDV\/OkSR4ox\/8STv6wyIm6YxXGW5LnXYRdxFI1KbsZRIi1riNdnG\/vJTCmvtHyzjiftGKwzIav8hf2eeK+gkLrmguJPETnUNk+Moi6OJ+MQy\/t2nfxag9ttWlS5d9TtuQWWmNfnsU3vha7IQPgVvciRylLidpKHM2nqPxIgDB23i5x5MbQkDmg8tQKHDjOQJ5dtBDzHhOvkT4skfHFTMVd6ayhFt0AQJeIADB+zlN\/EEyd+5cqq6upvLy8qLkxX2VVGyVzosMQJDaEYDg1Q5xg3IQFi3FNjclXcZQDBAhmLOOb1BAe\/ww4W9Pwp8\/eTcAIj88TgqEnogABG+olrFly5YlBa\/4IGrXrl1hp7n4jbhnz54kNtIkoo4OjQoBCN5GRbeSh02q+46ezyvrVMxXSXOdrD30s4NAqfp7jqjULYvFIg4v5mTNLztowCsQkEOg0QveYrvV4+CLHkcj+siuDstRgl4NDQEI3obGqJnnidvIlbauNS5SkY8QNWZ41OWl2AbXrKv3QkRXVFQUFnR0xQ67QMAGAo1e8IprIisrK+mpp55KLGmII4ltTJ06dZ+NIDYIhU8gAASAABAAAkAACACB+gg0asEbXrHl3c8yNbzFVkyWLl2aSSwjIYEAEAACQAAIAAEgAAT0ItBoBa\/4Oqhfv37Et2FlLUsQmwRkNq7V1tbqZRPWgQAQAAJAAAgAASDwOQK88RVtLwKNVvByHVv0mtO0K7xpap5Y7PJ1lIsXL0buAQEgAASAABAAAkBAOwKnn3468VXxEL6NVPDGbT5Lu8KbRuxyRr\/yyivUv3\/\/IPH4Nhw0txHgX0wmTZoEvtymqV504MwjsoiCX\/7xjoEzvxDwK1rxjtXU1EDwNtYV3qQjf5LKE0QZQ5qjX4TgReL5MWGALz94CkcJzvziDHz5xVd44QafY35wh3esPk+NtqQhmq6yK7xC7KY9+gWJ58cEIaIEX37xhQ9j8OUfAv5FjHnRL87AFwRvbMbKCN48l0wg8TBR+IWAf9HiHfOLM\/DlF1\/4pRJ8+YcABK+04I2er5tUClHqIHdM7n69KmvWrKEZM2bQmDFjqGnTpn4F30ijBWd+EQ++\/OKLowVnfnEG3QHBayVjkXhWYM\/s9OOPP6a1a9fSUUcdBcGbGUWzA8GZWbzzegNfeRE0Px6cmcc8j0foDgjePPmTeSwSLzN0VgZiYrcCey6n4CwXfMYHgy\/jkOd2CM5yQ2jUwLj5q+nX0x6kpXdchVMaGuspDUYz7nNnELw2UM\/uExN7duxsjQRntpDP5hd8ZcPN5ihwZhP99L4vvHsZLXp3M70+\/BgIXgje9AmUdQQEb1bk7IzDxG4H9zxewVke9MyPBV\/mMc\/rEZzlRdDseAje+njjWDJD+QfBawhoRW4wsSsC0qAZcGYQbAWuwJcCEA2bAGeGAc\/pDoIXgjdnCmUbDsGbDTdbozCx20I+u19wlh07GyPBlw3U8\/kEZ\/nwMz0agheC13TOBf4geK3AntkpJvbM0FkbCM6sQZ\/JMfjKBJvVQeDMKvypnUPwQvCmThoVAyB4VaBozgYmdnNYq\/IEzlQhacYO+DKDs0ov4EwlmvptQfBC8OrPshgPELxWYM\/sFBN7ZuisDQRn1qDP5Bh8ZYLN6iBwZhX+1M67jX2Z\/vHRxzil4XPksGktdQplGwDBmw03W6MwsdtCPrtfcJYdOxsjwZcN1PP5BGf58DM9GoIXK7ymcy7wB8FrBfbMTjGxZ4bO2kBwZg36TI7BVybYrA4CZ1bhT+0cgheCN3XSqBgAwasCRXM2MLGbw1qVJ3CmCkkzdsCXGZxVegFnKtHUbwuCF4JXf5bFeIDgtQJ7ZqeY2DNDZ20gOLMGfSbH4CsTbFYHgTOr8Kd2DsELwZs6aVQMgOBVgaI5G5jYzWGtyhM4U4WkGTvgywzOKr2AM5Vo6rd16PAXAie4Wngv1ti0pj\/nAg8QvIaAVuQGE7siIA2aAWcGwVbgCnwpANGwCXBmGPCc7iB4scKbM4WyDYfgzYabrVGY2G0hn90vOMuOnY2R4MsG6vl8grN8+JkeDcELwWs657DCawXxfE4xsefDz8ZocGYD9ew+wVd27GyNBGe2kM\/mF4IXgjdb5uQchRXenAAaHo6J3TDgCtyBMwUgGjQBvgyCrcgVOFMEpAEzfOEEb1rjhhrevYCjhtdA4rELCF5DQCtyg4ldEZAGzYAzg2ArcAW+FIBo2AQ4Mwx4DncQvPuCB8GbI6HSDIXgTYOW\/b6Y2O1zkDYCcJYWMbv9wZdd\/LN4B2dZULMzBoIXgtdO5mGF1xruWR1jYs+KnL1x4Mwe9lk8g68sqNkdA87s4p\/GOwQvBG+afFHaFyu8SuHUbgwTu3aIlTsAZ8oh1WoQfGmFV4txcKYFVi1GIXgheLUkloxRCF4ZlNzpg4ndHS5kIwFnski50Q98ucFDmijAWRq07PaF4IXgtZaBELzWoM\/kGBN7JtisDgJnVuFP7Rx8pYbM+gBwZp0C6QAWrtpMF01eFvTHKQ17YcOmNen0ydcRgjcffqZHY2I3jXh+f+AsP4YmLYAvk2ir8QXO1OBowgoEL1Z4TeRZrA8IXmvQZ3KMiT0TbFYHgTOr8Kd2Dr5SQ2Z9ADizToF0ABC8ELzSySLbcfLkyTR37lyqrq6m8vLyosMgeGURdaMfJnY3eEgTBThLg5b9vuDLPgdpIwBnaRGz13\/c\/NU0bv6aIACUNOzlASUNOfJxxYoVNGDAAGrZsiUEbw4cXRyKid1FVkrHBM784gx8+cUXRwvO\/OEMghcrvMqydfv27TR48GBasmQJdejQAYJXGbJuGMLE7gYPaaIAZ2nQst8XfNnnIG0E4CwtYvb6D5uzkua8ug4rvCEKsMKbMR+5lKGmpoYqKyvpqaeeguDNiKOrwzCxu8pM8bjAmV+cgS+\/+MIKr198XXj3Mlr07mYIXgjefIm7YMECGjlyJM2aNYsWLVqUqoZ39uzZwYqwaKXqfvNFidF5EMCHcR707IwFZ3Zwz+oVfGVFzt44cGYPexnP69btXdHldu1jGyB4I6BhhVcmi0J9OKH69u1L\/fr1o6qqKkq7aS3qjmuABw4cmDIKdNeNwK5du2jDhg3BRsSmTZvqdgf7ChAAZwpANGgCfBkEW5ErcKYISE1mZs6cGSzEcdvc57cFL9i0thcKCN6UiTd8+HCqq6uj6dOnU1lZWWrBO378eGrfvn3BKwsqrPKmJMFA9507d9L69euD1XgIXgOAK3ABzhSAaNAE+DIItiJX4EwRkJrM8IIc\/\/fPTR\/TkGf3QPBGcIbgTZF44VKGrl27BiPTrvBy3W+4pCGFe3Q1iAC+ujMItiJX4EwRkIbMgC9DQCt0A84UgqnRVPhaYXaDFV6s8KZON17dnTdvXtFxI0aMCMoc4hrO4U0Nt9UBmNitwp\/JOTjLBJu1QeDLGvSZHYOzzNAZHRg+kmy\/HRtp+ZgzsdCGkob8OYgV3vwYumgBE7uLrJSOCZz5xRn48osvjhac+cFZ+ISGphvfpj\/fdgEELwRv\/uSF4M2PoYsWMLG7yAoEr3+sFI8Y75h\/bIIzPzgLC94vvfU4Lbn3BgheCN78yQvBmx9DFy1gYneRFQhe\/1iB4AVnDQkB958lWr9btvBX9NIj0yB4IXjNJS9qeM1hrcITBK8KFM3aAGdm8c7rDXzlRdD8eHBmHvO0Hvl2Nb5lTbRW834YXJKFzfI4lixtLmXuD8GbGTorAzGxW4E9l1Nwlgs+44PBl3HIczsEZ7kh1G4gWr\/LK7wQvHthx7Fk2tNvrwMIXkNAK3KDiV0RkAbNgDODYCtwBb4UgGjYBDgzDHhKd9Fyhm+0XE8rZo6G4P0cRwjelAmVtTsEb1bk7IzDxG4H9zxewVke9MyPBV\/mMc\/rEZzlRVDv+Gg5w09O2UV3\/7QKgheCV2\/iRa1D8JrFO683TOx5ETQ\/HpyZxzyPR\/CVBz07Y8GZHdxlvPLqLtfuLnp3c9C9R+dWxIK3f\/\/+ELwQvDIppK4PBK86LE1YwsRuAmW1PsCZWjx1WwNfuhFWbx+cqcdUlcXo6u7jVZXUdONbELwhgFHSoCrbEuxA8BoCWpEbTOyKgDRoBpwZBFuBK\/ClAETDJsCZYcAl3UVXd7986JeC29WgO+oDCMErmVB5uyHx8iJodjwmdrN4q\/AGzlSgaM4G+DKHtSpP4EwVkmrtLFy1mS6avKxg9O7LK+jy08oheCMwQ\/Cqzbui1iB4DQGtyA0mdkVAGjQDzgyCrcAV+FIAomET4Mww4BLuoicziNVdHgrdgRVeiRRS3wWJpx5TnRYxsetEV49tcKYHV11WwZcuZPXZBWf6sM1iOVrKwDa4dvesLq0Cc9AdELxZ8ir3GCRebgiNGsDEbhRuJc7AmRIYjRkBX8agVuYInCmDUomhaQtradQjfyvY4jIGLmcQDboDgldJoqU1gsRLi5jd\/pjY7eKfxTs4y4KavTHgyx72WT2Ds6zIqR8XPZUhXMoAwRuPN2p41edhrEUIXkNAK3KDiV0RkAbNgDODYCtwBb4UgGjYBDgzDHgRd+Pmr6Zx89cUfspil0sZ+M9wg+7ACq+VjEXiWYE9s1NM7JmhszYQnFmDPpNj8JUJNquDwJlV+APncWL3rn4VhbpdCN7iHGGF11D+QvAaAlqRG0zsioA0aAacGQRbgSvwpQBEwybAmWHAQ+54g9rTf9lAo+etqreyW0zscifoDqzwWslYJJ4V2DM7xcSeGTprA8GZNegzOQZfmWCzOgic2YGfxS6fs8t\/isblC6XELgTvvlxhhddQ\/kLwGgJakRtM7IqANGgGnBkEW4Er8KUARMMmwJlhwIkoujmNIyhWsxuNDroDK7zmMxZfLVjBPI9TTOx50LMzFpzZwT2rV\/CVFTl748CZWewvvHsZLXp3cz2nsmIXK7xY4TWbrSFv+E3LGvSZHGNizwSb1UHgzCr8qZ2Dr9SQWR8AzvRTwGULLHKHzVm5j7NJl32Vrjz9SOkgoDuwwiudLCo7IvFUoqnfFiZ2\/Rir9gDOVCOq1x740ouvDuvgTAeqe22y0OX\/rpu7sl6tLv+MV3VH9e5EfLFEmgbdAcGbJl+U9UXiKYPSiCFM7EZgVuoEnCmFU7sx8KUdYuUOwJlySBOF7q0XdqGLurbN5Bi6A4I3U+LkHYTEy4ug2fGY2M3ircIbOFOBojkb4Msc1qo8gTNVSO61wxvS+Fzd8OkLwkOaWt1iUUF3QPCqzVhJa0g8SaAc6YaJ3REiUoQBzlKA5UBX8OUACSlDAGcpASvS\/f5X6ui\/q9+O\/anMcWOyUUB3QPDK5orSfkg8pXBqN4aJXTvEyh2AM+WQajUIvrTCq8U4OMsGq6jP5dXc6KkL4RXdpHN103qH7oDgTZszSvoj8ZTAaMwIJnZjUCtzBM6UQWnEEPgyArNSJ+AsHZwsdJ95cyP9+NG\/FR2YdUOaTCTQHRC8MnmivA8STzmkWg1iYtcKrxbj4EwLrNqMgi9t0GozDM7koF24anNQm1tqNffbx7ehqm8cFZzAoKtBd0DwFhAYPnw4zZs3r\/D\/U6ZMoV69eiXm3uTJk2nChAmFfiNGjKCqqqqS45B4ibA61QETu1N0SAUDzqRgcqYT+HKGCulAwFk8VLySu3DVpmATWjGRyyN1rubGRQbdAcEbIMBid+nSpVRdXU3l5eW0YMECGjp0KCWJVxa7U6dOpVmzZlHXrl1pxYoVNGDAABoyZEhJ0YvEk55TneiIid0JGlIFAc5SwWW9M\/iyTkHqAMDZXsjEqQp\/eHMjPfH6hkSR+72Tj6ABZ7TTupoLwZuczk327NmzJ7lbw+ohROr48ePrreiyCK6rq6Pp06dTWVnZPg+9bt066tu3L5166qk0ceLEws+j4hmJ53++YGL3j0Nw5hdn4MsvvjjaxsyZ2Hg259W1wUpuqcYrud+qOIz+v66H01ldWlkjGgttWOEtmnwQvNbeS+ccN+aJ3TkyJAMCZ5JAOdINfDlCRIowGhNnYhV328ef0k2P\/q3kKi5DyCL3rM6tqN9pR1oVuWE6IXgheGNfb1GXm1THW6ykoWfPnvVWfaNOROLNnj2bOnToUPgxl1OguYdAY5rY3UM\/W0TgLBtutkaBL1vIZ\/fb0Dmr2\/ppIGxnL1mbKHAZxXaHNA02nn3tyDInRC5\/Cx1utbW11L9\/f6qpqamnO7JngN8jG2VJQ5gyUbvL\/9anT5+SolWMEyUR27ZtC\/4pSSRzHyF4o+nC9b8DBw70O4saYPS7du2iDRs2BPXdTZs2bYBP2PAeCZz5xSn48osvjrYhccbiltvabZ\/S1MWb6bX3P5YihEXuz89rQ0e2aBoIXpfazJkzg\/1F0QbBuxeRRi94RWJs376dBg8eHNTwio1s0aSJ6yPqetu1a1e09jcseLluuH379gXTLKiwyuvSlLE3lp07d9L69euD34oheN3jJy4icOYHTyJK8OUXXw1hXuSTFN6s+xc9+cYGaYHLpQoDzziSKjuU0Rkd993b4xKLrEfCq7yLFy+mSZMmYYX3c5IgeEPZmnTiglgNjq7oFtsEF34RUEvj0rSQHEtD\/+ouGQH\/eoAzvzgDX37xxdH6xJnYZFb92jpavXGnVIkCPyML3HOOaU19TykP\/q7znFzdGQDdUR9hJwXvli1b6J577iH+M2tr2bIl3XTTTamGywjekSNHFo4kE8bFKm+\/fv2KHk2GxEtFhfXOPk3s1sFyJABw5ggRkmGAL0mgHOrmKmdC3G7e+QlNramVFrdC4Pbo3IouP+1I7wVuNFWgOzwQvEJAcsF11sZfRXPdSlzjldo44VpsBVfYwApvVjb8G+fqxO4fkuYiBmfmsFbhCXypQNGsDVc445vM\/r1nD41fsCaVuBUCl8Uti1zfV3CT2Ifg9UjwjhkzRurmsyjpLEzHjh1bVPCKWlweJ87cFau7FRUVRWtxVdTwong86RV14+euTOxuoOFHFODMD55ElODLL744Whuc8ertcys\/pEeXf5BJ3B7V+ktBeUKnNs2dOEnBJOsQvB4IXt4df9111wX\/nX322anz48UXX6S77rqLHnrooZJjo1cLR29Zix5BJoxFx8mc7oDES02j1QE2JnarD9wAnIMzv0gEX37xpVvwirKEmr9topff25xa3IrV295fa0PDzj0qANfn+lsV2QHd4YHg\/eyzz4j\/a9asmQrOnbCBxHOCBukg8GEsDZUzHcGZM1RIBQK+pGByqpMKzoSw5Qf7\/Z\/X06oNOzKL24Zae6uKdOgODwQv1\/DyYclHHXVUsAmMr\/Ldf\/\/9VeWAFTtIPCuwZ3aqYmLP7BwDMyEAzjLBZm0Q+LIGfWbHaTgTN5Xxn2\/UbaOn39iYSdiKldqzurSmfqf6f3JCZvAzDITu8EDw8ukM\/\/M\/\/0NPPvkk7d69m9q2bUtXXXUV8SkIrVu3zkC7\/SFIPPscpIkgzcSexi766kMAnOnDVodl8KUDVb024zgLC9t1W3fRzJfrcglbfoIf9mhPbcqaFTaW6X2qhmsdusMDwStC5E1iL7zwAt133330+uuvB\/980kkn0bXXXkvnnnuuVyUPSDy\/JhV8GPvFF0cLzvziDHz5xRcLW+bs9ffW0u4DDqE5S9NvIgs\/sTghYe+qbfMGf2KCDbahOzwSvOFQeSPbww8\/TDNmzAiufC0rK6NLL72UBg0aVO\/mMhtJJeMTiSeDkjt98GHsDheykYAzWaTc6Ae+3OAhHEV4tZb\/\/ZX3NtOf\/rYp84ot22BhyyclnHtsazq9U8M\/CswlVqE7PBW8Iuw9e\/bQO++8Q9OmTaM\/\/OEPtGPHDurUqVOw6nvRRRc5u+qLxHNpGkiOBR\/GyRi51gOcucZI6XjAlz2+wsJ2D+2h6qXrac2H8reRxUUuTkS49JQj6BvHHIoVW3v0FjxDd3gueMPhc33vyy+\/TD\/96U+Df66urqby8nIH0mzfEJB4TtJSNCh8GPvFF0cLzvziDHzp5Su6Wvvqmi0auxYFAAAgAElEQVT0f9\/+KNdqLUfc7pCm1LRpU\/ruiW3pO8e3gbDVS2Mu69AdDUDwCqHL5Q0sePkIs169etFtt91GfKWwiw2J5yIrxWPCh7FffEHwgi\/\/EFATcVjY\/mPTTpqzZB39c9PHJP49qxdRinDOMa3pzK\/sLUU4\/CCitWvXBicosehFcxsB6A5PBS+L2uXLl9OcOXMKpQw+nd6AxHN7YohGB8HrF18QvODLPwTkIw6L2rVbdtGsV+qUiVqO4qzOrej0r7SiToeV3jyGeVGeMxd6Qnd4JHi5Xre2tjbYqPb444\/Thx9+GNTonn\/++TRkyBA69thjqUmTJi7kVWIMSLxEiJzqgIndKTqkggFnUjA50wl87UvFwlWbg5XU\/\/vWR\/T7ZeuViFr2IlZrL648nI49\/ODCDWRpbyIDZ868PlKBQHd4IHh37txJ999\/P82aNYvq6uoCUVtRURGcxfvtb387OKHBt4bE84sxTOx+8YUVXvDlAwLhW8YeXf4BvbP+X8pF7WkdW9Ixhx8UnIwgjv5ShQ3mRVVImrED3eGB4OWb1vr27UubN2\/26uixUimMxDPzgqvygoldFZLm7IAzc1ir8NRQ+RLlB8\/8ZSO9\/v62oJZ20bubVUBWWJm94MS2dHy7Mi2itlSgDZUzJeQ4aAS6wwPByyu8a9asoc6dOzt7zFja3EbipUXMbn9M7Hbxz+IdnGVBzd4YX\/kSq7S8evqbF\/5Bb69Tt0rLbIjyg+OOOJj6dDs8IEj1Sm1W1n3lLOvz+j4OusMDwStWeMeMGROcvpC2LViwgMaOHUs1NTVph2rrj8TTBq0Ww5jYtcCq1Sg40wqvcuMu8hU9ymv1xh1U\/Zq6WloBohC13Tu2pG8ed2hQ1tCj896TEFxuLnLmMl62Y4PugOC1koNIPCuwZ3aKiT0zdNYGgjNr0GdybIuv8ArtkjVbaObLdUH8qsoOxIos\/8kilssPWnypaeaNYpnA1TTIFmeaHqfBm4Xu8Ejw8gkNWVuHDh2wwpsVPIzDJQYe5gA+jP0iTSdfQtS+snoz\/emdTdoELW8MO+MrLYObxYTQdX2VNk+W6OQsT1wYG48ABK8HgnfLli10zz33EP+ZtfEFFDfddFPW4crHIfGUQ6rVICZ2rfBqMQ7OtMCqzWgWvqIlBzV\/20Qvv7d3Q5jKFVohXlnQnnL0IcT1tDpOPdAGribDWTjTFArMSiAA3eGB4JXg0bsuSDy\/KMPE7hdfHC0484uzOL6ignbhqk2BkFVxc1gUHVFHe\/RhzanvKUfQ+5t3BSUIQuz6haaZaPGOmcFZlRfoDgheVbmUyg4SLxVc1jtjYrdOQeoAwFlqyKwMEOUGtR9uo+ffXE\/rdu6vXdCedvQh1LntQQ2ijtYKaZ87xTtmE\/30vqE7IHjTZ42CEUg8BSAaNIGJ3SDYilyBM0VAZjDDIpZXTMMbwqbW1Abn0OpYnRWrsFxm8NXyg6nbUS3o7C6tC5E35DraDPQoG4J3TBmURgxBd0DwGkm0qBMknhXYMzvFxJ4ZOmsDwZle6MPlBhu376bn3vqI\/v7hTuW1s0LM8p8saHsffxh163BI8HCunEerF2l3reMdc5ebuMigOyB4rWQsEs8K7JmdYmLPDJ21geAsH\/QLV+3d\/PX2+n\/Rsn9uC8SsztXZTz\/9lE5p\/yU6tcsRVHnUF4I2LHjzPRFGq0YA75hqRPXag+6A4NWbYUWsI\/GswJ7ZKSb2zNBZGwjO9oVerMryT8Tfn3vrQ3rt71u1ilmxOtu1Q4vgClyxGSwsZsGXtVcls2Nwlhk6KwOhOzwWvHv27KFNmzbRZ599Roceeijtt99+9Mknn3hx\/TASz8r7ntkpJvbM0Fkb2Fg5E3Wzf\/9oJ63euJMWr96iTcwKwSqO6OKrb5sfsH\/Aedpyg8bKl7UXRIFjcKYARIMmoDs8FLwsdJ999ln66U9\/Shs2bCC+VKK6upoOPPBA+uEPf0innnoqjRgxwmnhi8Qz+JYrcIWJXQGIhk00JM7EJjBRZtCh9YE06f\/+g1Z9sEO7mBWrs2d1aUU9OtffCKZyM1hD4stwqltzB86sQZ\/JMXSHh4L3T3\/6Ew0ZMoROPvlkOuaYY+iFF14IBG9ZWRn9+Mc\/pvnz59OYMWNowIABqZJi+PDhNG\/evMKYKVOmUK9evRJtLFiwgIYOHVro1717d5o+fXoQT7GGxEuE1akOmNidokMqGJ84C59m8M76f9GClR\/SyrX\/MiZmzzmmNXVo\/SWrlyn4xJdUAjaCTuDML5KhOzwTvLt27QrE5b\/\/\/W9iQfriiy\/S2LFjA8FbXl5Ou3fvJhauGzduTBSd4UfnMUuXLi3YESKWV4qrqqqKZvXkyZNpwoQJQSwsjtetW0d9+\/aldu3alfSPxPNrosDE7hdfHK1tzuKO5np02Qf0zgf\/CupndW0A42cXK69canBShxb0nePbBASKf1e5MqsqM2zzpeo5GpMdcOYX29AdngleISivueYauuKKK4iFaVjw8uM88MADdO+99xbEa1JKrlixIlgNHj9+fL0VXRbBdXV1RYWriKVfv371RDHHNHLkSJo1axZ17do11j0SL4kVt36Oid0tPmSiMcFZ+GiuN97fRk\/\/ZWMQmuprbcPPGxazp3dqGVygELcJTAYjl\/qY4Mul520IsYAzv1iE7vBU8F555ZV09dVXxwpeXnV9+OGH6aGHHqK2bdtmzsgkwRsntmWdIfFkkXKjHyZ2N3hIE0UWzuJWZZ9d+SEtr91GazbqO5ZLPFdYzF56yhH0lTYHOb8ym4aTUn2z8KXKN+xkQwCcZcPN1ijoDs8Eryhp2LFjB02bNo2YwPAKb21tLV111VV01FFHBWUGvJEtS4uWKsTZ4D41NTV02223BT7ZN7c+ffrQxIkTS7pF4mVhxd4YTOz2sM\/quRhn4VVZPmN23vIPAhc6SwzYPotZLjE4+rDmwW1gF53UNihtcLnMICv2WcbhHcuCmt0x4Mwu\/mm9Q3d4Jng53GXLlhGXNPCGNf7v6aefph\/96Ee0atUq+v3vfx\/U8U6dOpW+8Y1vpM2HYMVYbEBLEq5ik1uLFi0K5Qtpa3hnz54dnDIhGtcho7mHACZ29zgREdVt\/ZTaHdK03jW2C97aTH+p205vv7+JNuz84sxZHU\/BvrmxmO1y+EF0cbfDab8mTYJ\/S3s0l474fLGJd8wXpr6IE5y5zRnrkXDjRbn+\/fsHC3Vh3eH2U+iLrskePvPLg\/bSSy8Fx5KtXr26XrRcwvCLX\/yCzjvvvFxPsX37dho8eHBQwys2xEUNCsEbPc1BiOZSpzyI37SiNrmWeODAgblix2D1CPA3C3wEHv9C0rTpXoGDZg6B197\/mI5s0ZRY3P75\/Y9p6fsfB87533U2IWbZd2W7A6n7Uc0Dd\/z\/a7ftvRkMTQ0CeMfU4GjSCjgziXZ6XzNnzgwW46INgncvIt4IXg6WtTmfxvDXv\/6V+MU7\/vjjA0Gy\/\/57Dz7P28RmNj4CLe6kBha8zz\/\/\/D6b04ptZgvHIwQvb5Rr37594UccP1Z58zKnfvzOnTtp\/fr1wW\/FELz58a1fK7uTvnxoc5r5ch0tWbMlMM5iksWtrhaule3UpjldeMKhdMD++wXu+GdC6OryD7v7IoB3zL+sAGduc8ZaJLzKu3jxYpo0aRJWeD+nzSvBqzvVkgQv1\/By6UT0NIY0ghe\/aelmUY19fHWXHke+JIHFI9\/0VfO3TbTmQ\/2bvoRg5T9PP7qMDmv2KfXqelTwSwrHwkKbL1BAcw8BvGPucZIUEThLQsitn6OGtz4fzgveLVu20D333EP8Z1Jr06ZNUNpwwgknlFz1LXaMWFJpQjFBjGPJkpjx7+eNfWIXG72YOfH3TTs+oT+8uVH7mbIiW8KrshVH7t30RVS8Vraxc+bbWwa+fGPM\/lnX\/iFmN2IIXs8EL9dRXnfddfTmm28Sn9TAjYUtNy5viGtnn3023XnnnXTIIYfE\/lzU6\/IPxQ1pQsxWVFSUvEAiWtYgVnf5euNSJzUg8ey++Gm9N\/QPY3HTF+OydssuenXNFlq5Tu9NX3FC9vh2ZXTM4QfRcUccHPw4zwkGDZ2ztDnsen\/w5TpD+8YHzvziDLrDM8HL4S5ZsoSGDRsWXBbxgx\/8oHCFL5\/O8L\/\/+7\/EpQZ33XUXfe1rX6PHH388OLaM+95www0lszN6tXD0lrViJQziCDNhPOl0B+6HxPNrovBxYg8fv8Vo79j9WXBl7dsWhCwLVxaxJ3957y+deYSsbOb4yJnsszXEfuDLP1bBmV+cQXd4JnjFauyxxx5Lt9xyCzX5\/Pgf8Ri8ke3mm2+mv\/\/978E5vM2bN6dx48YFAvPRRx91JjuReM5QIRWIaxN7eEX29fe30V\/r\/kV\/\/8hMjawQrHwMV8fDmtPZXVrRZ3vIudu+XONMKtEacSfw5R\/54MwvzqA7PBO8omSAV3gvu+yy2GzjG9buvvvuwnFijzzyCN1xxx3BzkRXGhLPFSbk4jA1sYeF7NaPP6Xn3\/qI3llvprQgLGR5BZYvR6g8qv6KbHh1Vg45e71McWbvCRuWZ\/DlH5\/gzC\/OoDs8E7ybNm2iQYMGUefOnYMbzpo1a1bvCbisYfTo0fTuu+\/SfffdR61btw7qd\/lyimeeecaZ7ETiOUOFVCBZJ\/boVbXsbOW67bSidjv93dCpBWEh+5U2zemUo1sS\/xkWr6LEQAoMTzpl5cyTx2twYYIv\/ygFZ35xBt3hmeDlcO+9916aMGECXXrppcEGNnFuLa\/+cu3u3Llzg3rda6+9Nqj3\/fGPfxyc0cvC15WGxHOFCbk4Sk3sfPwWt\/c27qA1H34cbPjitujdvf+us4nravnPDq2\/RGd3aV1PyIZFrc44XLSND2MXWSkeE\/jyiy+OFpz5xRl0h4eCl1dxWfD+7ne\/o88++6zeE\/ClE7yRjTec8ct49dVXB7elsUjmExdcaUg8V5jYN464I7je37SDnl7+Pn20uyn9c9PHhaO5dD1FWMie8ZVW1Omwhr8iqxpLfBirRlSvPfClF18d1sGZDlT12YTu8FDwipD5iDK+6YyPEOPGq7jf\/OY3CzeX8S0wvOrbrl07OvDAA\/VlUQbLSLwMoCkeIi5G+EvddnrmzY20ZqP+TV\/hs2TPPqY18cavHp3rX4TQEMsLFFMnZQ4fxlIwOdMJfDlDhXQg4EwaKic6Qnd4LHidyKCMQSDxMgKXYpjYALb8n9to\/l\/3ntGsq8wgLGRPbF9G55\/AlyJ8cfxW9O8pHgNdMyKAD+OMwFkaBr4sAZ\/DLTjLAZ6FodAdHgpePnrsvffeo3nz5sVeNrFr1y56\/\/33gzujRX2vhdwq6RKJp4YRIWpf+8dWem7lh1rKDVjMHtmiKR15MNFpXQ6nEzu0LIhZrMaq4VGHFXwY60BVn03wpQ9bXZbBmS5k9diF7vBQ8PKJC3xJBNfyxjU+uYFvOuNNanxKg4sNiZeeFS5BeOW9zfSnv21StlIrBCuXFXzn+Da0bddnQZlB9GIETOzp+bI9ApzZZiCdf\/CVDi8XeoMzF1iQjwG6wzPB+69\/\/YuGDBkSbET7zW9+Q0cffXSwMa2ysjK4fW3OnDk0Y8aM4NKJE088UT4TDPdE4pUGnMXtU29sIK6vzVOGEBa0px7dMri2NsstX5jYDb8gCtyBMwUgGjQBvgyCrcgVOFMEpCEz0B2eCV5x8QQfSXb99dcH0f\/85z+nDz74IChh4Marv7xJbfz48fvcxGYorxLdIPG+gEiUJYybvzqTuBUC9hvHtKZLTykPDPO\/qSw3wMSemNLOdQBnzlFSMiDw5RdfHC0484sz6A5PBe+PfvQjuuSSS4LoH3jgAXriiSdo2rRpdMghhwT\/\/+CDD9L9999Pbdq0cTIjG3PiCYH7+Osf0PSF70vzIwRsr68dRheddHimlVppZ5GOmNizImdvHDizh30Wz+ArC2p2x4Azu\/in9d6YdUccVk328I4wh9vWrVuDEoaTTz6ZRo0aFUT63HPPBau8LHQ7duwYCF8Wu9XV1di05hCXXKYgu4orxO2Y879C5YccqHzFNi0smNjTIma\/Pzizz0GaCMBXGrTc6AvO3OBBNgoIXs9WeDnc22+\/Pbh04sYbb6R+\/foF9bxXXHFFcPPad77zHbrpppuIdbu4Wlg2GUz2awyJJ1Zyq19bRw8sXlsSXnHRwqjenayL27hAMbGbfDvU+AJnanA0ZQV8mUJanR9wpg5LE5Yag+5Ig6PzK7z8MLzKy\/W7\/CeL2latWtHYsWODzWosdJs0aUI333wzDRgwIM2zG+3b0BNvzqvrgtXc8K1lUYBZ5N7YqyOd1aW10npbHURiYteBql6b4Ewvvqqtgy\/ViOq3B870Y6zSQ0PXHWmx8kLw8kOxsN22bRu1aNEiELh8xfBLL71EL774IvXu3TsoeeB\/d7U1xMRjcfvSu5upas7KorCzyL2rX4WTq7ilcgUTu6tvUvG4wJlfnIEvv\/jiaMGZX5w1RN2RhwFvBG+ph2Txu2XLFmrZsiXtv\/\/+efDQNrahJd7UF2vpx4\/+LRYvIXLP6lL\/Cl1t4GowjIldA6iaTYIzzQArNg++FANqwBw4MwCyQhcNTXfkhcZ5wSuOJRszZgz16tUr9nmfeuopGjduHDat5c2GhPG8ostn5A4rsqJ7+WnlJGpyNYei3Twmdu0QK3cAzpRDqtUg+NIKrxbj4EwLrNqMQvDWh9ZJwctXBXO5wo4dO2jz5s10xx130MUXX0xdu3bdJzG4L9f1ct+HHnqI2rZtqy158hj2PfFY7F40edk+Nbq8mnvBiW3pmrM7OF+Xm4Y\/TOxp0HKjLzhzgwfZKMCXLFLu9ANn7nAhE4nvukPmGdP0cVLw8gPMmjWLbrnllqB2N6lxGcPIkSOD48tcreP1NfFY6D678kMa+ft39qFh4Jnt6P\/veXSDErriITGxJ7117v0cnLnHSamIwJdffHG04MwvznzVHbpQdlbw7t69O6jLXb9+PQ0dOpT++7\/\/m84+++x9cGjWrBm1bt3aWaErAvYx8Uqt6j5eVdkghS4Er66pRr9dfBjrx1ilB\/ClEk0ztsCZGZxVefFRd6h69jg7zgpeESxvSPvoo4+CG9X4+mBfm2+Jx8eMRWt1G8JmNNn8wcQui5Q7\/cCZO1zIRAK+ZFByqw84c4uPpGh80x1Jz5P3504KXiFy+U\/ZxmUNhx56KE5pkAWsRL\/ZS9bSdXPfqteDxW5DX9UNPzAmdgWJZNgEODMMeE534CsngBaGgzMLoOdwCcFbHzwnBa84maG2tlaa6g4dOuCUBmm04jtyCcPjKz6gnz3xbr0Od\/X7KvXvfmRO634Nx8TuF18cLTjzizPw5RdfeMf84wuC1wPBu3PnTlq0aBHxCQyyjcsdevToQc2bN5cdYrSf64nHYnfOq2tp3Pw1BVx4VZePGePjxhpbw4exf4yDM784A19+8QXB6x9frusO04g6ucJrGgQT\/lxOPBa7181dSQtXba4ndhtTCUM0B\/BhbOKtUOsDnKnFU7c18KUbYfX2wZl6THVadFl36HzuYra9ErybNm2ihQsX0muvvUZ8OkNlZSWdccYZwSkNWdrw4cNp3rx5haFTpkwperlFMfuTJ0+muXPnJpZTuJp4LHbnvrqO\/s\/81RC7IZIxsWd5o+yOAWd28U\/rHXylRcx+f3Bmn4M0EbiqO9I8g8q+Xghe3rx277330sSJEym6kY03qw0bNoyqqqoCESzbWOwuXbq0IFQXLFgQHH82YsSIwJZMW7FiBQ0YMCC40ri6uprKy4t\/9e9q4k1bWEujHvniiuDGtjmtGM+Y2GXeALf6gDO3+EiKBnwlIeTez8GZe5yUishV3WELRS8E78MPP0yjRo2ic845h2644Qbq2LFjgNeaNWvo9ttvD25lGz9+PF100UVSOAqhymPC1xWzCK6rq6Pp06dTWVlZSVvbt2+nwYMH05IlS0hmw5yLiTdu\/up9anYbcxlDmHBM7FKvklOdwJlTdCQGA74SIXKuAzhzjpKSAbmoO2wi6LzgFcKSBeidd965z6Y03uB2\/fXXByu\/XJKQ56zeNIKXSxlqamqCsoqnnnrKyxXeQ4e\/UMg9rOzWfw0xsduclrL5BmfZcLM1CnzZQj67X3CWHTsbIyF466PuvOAVR5Rdc801dMUVV8TmzAMPPBCUPCSVFZRKOBawEyZMCERzeNU3bgyXP\/BVxnz9MZ8m4VsNL9ftdhv7MsRuiYTAxG5jes7nE5zlw8\/0aPBlGvH8\/sBZfgxNWoDg9UzwbtiwgS677DK6+OKLg5XcuMYrv48++ig99NBD1LZt21T5JGp3eVCfPn2COuFSTQjwfv36BbW+aTetzZ49OyiBEK1U3W+qB0nR+eKpb9Cid784kWHSpV3o8tMa1zm7SXBhYk9CyL2fgzP3OCkVEfjyiy+OFpy5zRnrk3Djuwz69+8ffBsd1h1uP4W+6Jxf4f3kk0+ISw1WrlxJ06ZNo06dOtVDY\/Xq1XT11VdTRUVFIFYPOOCATGiJ0gmu4S21Uhwte0greKPB8aa3gQMHZoo5y6DH\/rqdbn1+Y2HoVae0pOu\/nu2Uiyz+fRnDZ0DzL1v8C0nTpk19CbtRxwnO\/KIffPnFF0cLztzmbObMmcE3z9EGwbsXEecFLwf5xhtv0KBBg4KX7Vvf+lZwwQQ3Lid49tlng7pdLmngeto8TWxmGzJkSOxJDeFShq5duwau0gpe3ijXvn37QpgsqEyt8nIpwxkTlhV8c93uKyPyYZYHb5fHcm34+vXrg9+KIXhdZuqL2MCZHzyJKMGXX3xxtODMbc54hTe8yrt48WKaNGkSVng\/p80Lwcuxvv322zR69Ghavnw57dmzZ69ab9KEunXrRrfddhsdd9xxuTMxSfBGz+2NOix1pJntWhoWu8PmrCyUMmCTWul0wVd3uV8n4wbAmXHIczkEX7ngszIYnFmBPbNT27ojc+CaBnojeMXz8yovX0DBjS+cyHIqQ9xKLdsT9bwyG9dEPGlXeG19tRA9guzuyysa5ZXBsu8RJnZZpNzpB87c4UImEvAlg5JbfcCZW3wkRQPBWx8h5wUv11Hed999wRm7xx57LPFFE3mbqNdlO+LMXbG6y7XAMufw+iR4405lWD7mzLwwNujxmNj9oxec+cUZ+PKLL44WnPnFGQSvh4KXT2ngSyYOO+ywQPheddVVQW0llzTkadEShWhJAq\/eTp06NSgCFzW7UX8+rPBeePeyeqUMd\/WroLO6tMoDXYMfi4ndP4rBmV+cgS+\/+ILg9Y8vCF7PBC+Hu3v3bnr55ZdpxowZwZ\/8\/3xagziuLO1RZDbS1lbiLVy1mS6a\/MVGtYmXHkdXndnOBgRe+cSHsVd0BcGCM784A19+8YV3zD++bOkOV5FyvqQhClxU\/PKxZVyGwKc4XHDBBdSsWTMnsbaVeNHVXZQyyKUHPozlcHKpFzhziY3kWMBXMkau9QBnrjFSOh5busNVlLwTvGEgt27dSr\/73e+Cld8WLVrkumlNN0E2Em\/Oq+uCkxlEe7yqEqUMkkRjYpcEyqFu4MwhMiRCAV8SIDnWBZw5RkhCODZ0h8sIeSd4ecPZCy+8EIhbJpPbSSedFKzw8hm9WOHdm268UY1LGfhPbnwMGVZ35V9FTOzyWLnSE5y5woRcHOBLDieXeoEzl9hIjgWCtz5GXgjeqMj97LPPghreH\/zgB\/Td7343OJ7M9WY68aKruyx2WfSiySGAiV0OJ5d6gTOX2EiOBXwlY+RaD3DmGiOl4zGtO1xHx3nBy7eG9O3bl\/hOaN6cduWVV9LFF19c77Yy10Hm+EwmXtwlE1jdTZclmNjT4eVCb3DmAgvyMYAveaxc6QnOXGFCLg6TukMuIru9nBe8fMnEnDlzqHfv3vSVr3wl91FktuA2mXhY3c3PMib2\/BiatgDOTCOezx\/4yoefjdHgzAbq2X2a1B3ZozQ30nnBaw4KvZ5MJh5OZsjPJSb2\/BiatgDOTCOezx\/4yoefjdHgzAbq2X2a1B3ZozQ3EoLXENamEi96qxquEM5GMCb2bLjZHAXObKKf3jf4So+Z7RHgzDYD6fyb0h3porLXG4LXEPamEg+ru2oIxcSuBkeTVsCZSbTz+wJf+TE0bQGcmUY8nz9TuiNflOZGQ\/AawtpE4kVXd0f17kijency9IQNyw0mdv\/4BGd+cQa+\/OKLowVnfnFmQnf4hAgEryG2TCTerFfq6EfVbwdPhHN38xGLiT0ffjZGgzMbqGf3Cb6yY2drJDizhXw2vyZ0R7bI7IxyXvBu2bKF7rnnnuDa4BNOOCEWpddee43uuOMOmjhxYnB0mYtNd+JFV3d\/2KM9jf\/PY12EwouYMLF7QVO9IMGZX5yBL7\/4wgqvf3zp1h2+IeK84BXn8I4ZM4Z69eq1D758CcWECRPoqaeeatRXCy9ctTm4WU00XCOc71XEh3E+\/GyMBmc2UM\/uE3xlx87WSHBmC\/lsfiF46+PmpODdvXs3jR49mh555BFplv\/jP\/6D7rzzTmrevLn0GJMddSceNqupZRMTu1o8TVgDZyZQVucDfKnD0pQlcGYKaTV+dOsONVGas+Kk4OXHX716Nc2cOZP44okXXniBunXrFnu7WosWLaiyspLOOuss4r+72nQmHjarqWcdE7t6THVbBGe6EVZrH3ypxdOENXBmAmV1PnTqDnVRmrPkrOAVEMjU8JqDK7snnYk3bv5qGjd\/TRAcNqtl5yg8EhO7GhxNWgFnJtHO7wt85cfQtAVwZhrxfP506o58kdkZ7bzgtQOLeq86Ey9cztCjcyt6Ylil+gdoZBYxsftHODjzizPw5RdfHC0484sznbrDLyT2Ruuk4BWruhzg97\/\/fXrwwQeJ\/61Ua9myJV177bXEf7rYdCVetJwBm9XUsI+JXQ2OJq2AM5No5\/cFvvJjaPocoEMAACAASURBVNoCODONeD5\/unRHvqjsjXZS8IqTGRgW3oh2\/fXXU21tbUmUOnTo0ChPaUA5g56XBxO7Hlx1WgVnOtFVbxt8qcdUt0VwphthtfYheOvj6aTgVUu5G9Z0JR7KGfTwi4ldD646rYIzneiqtw2+1GOq2yI4042wWvu6dIfaKM1Zg+A1hLWOxEM5gz7yMLHrw1aXZXCmC1k9dsGXHlx1WgVnOtFVb1uH7lAfpTmLzgteUc+LGt59kwLlDPpeFEzs+rDVZRmc6UJWj13wpQdXnVbBmU501duG4K2PqfOCV9TzooZ335cB5QzqJwhhERO7Pmx1WQZnupDVYxd86cFVp1VwphNd9bYheD0TvMVSYM+ePbRx40aaO3cuPfbYY3THHXfQCSecoD5jFFlUnXjRcoblY84MzuBFU4MAJnY1OJq0As5Mop3fF\/jKj6FpC+DMNOL5\/KnWHfmisT\/a+RXeJIhY+N58883BjWwTJ06kAw44IGmIlZ+rTrw5r66jYXNWBs+CyybUU4qJXT2mui2CM90Iq7UPvtTiacIaODOBsjofqnWHusjsWPJe8DJsDz30EN19992pjyUbPnw4zZs3r4D8lClTqFevXiWZ2L59Ow0ePJiWLFlS6DdixAiqqqoqOU514qGcQe8Lg4ldL746rIMzHajqswm+9GGryzI404WsHruqdYeeKM1Z9V7w7t69m0aPHk1vvvkm3X\/\/\/dSmTRsp9FjsLl26tCCSFyxYQEOHDqVS4lXUE7dr146mT59OZWVltGLFChowYAD17NkzWGEu1lQmXrScYVTvjjSqdyep50YnOQQwscvh5FIvcOYSG8mxgK9kjFzrAc5cY6R0PCp1h19PHh+t84I36ZSG9957LxCuLDp\/9rOfUZMmTRJ5ESJ1\/Pjx9VZ0WQTX1dUVxGzUEIvikSNH0qxZs6hr166FH0+ePDmoJa6urqby8vJY\/yoTb+GqzXTR5GUFP7hdLZHy1B0wsaeGzPoAcGadglQBgK9UcDnRGZw5QYN0ECp1h7RThzs6L3iTTmlo1qwZXXLJJXTTTTfRIYcckgvqJMFbzDgL3qlTp+4jhMP9VSbegBl\/oSdf3xCYR\/1uLsqLDsbErgdXnVbBmU501dsGX+ox1W0RnOlGWK19lbpDbWR2rDkveE3BwqJ1woQJJFPHG40pWh4RF7NIvNmzZxNfgyxasRXhUs996v95lbisgdsNPTugnEFDkmBi1wCqZpPgTDPAis2DL8WAGjAHzgyAnMMFLxCGGx\/n2r9\/f6qpqamnO3K48Hqot4KXT2fYtm0btWjRQqqMoRhLonaXf96nT5+SdbhxNmRqf3mcELxRG1yKMXDgQOkkqtv6KV04s7bQ\/95LyumU9jiOTBpAyY67du2iDRs2BCUqTZs2lRyFbjYRAGc20U\/vG3ylx8z2CHBmm4HS\/mfOnBl80xxtELx7EfFC8PLGtF\/\/+tfBxjReieXNYnwM2aBBg4Ka25\/85Cd04YUX5hK+4vQFtleqFjecSKIWuKKiomjdr+gvBC\/XDbdv375ghgVVmlXe25\/\/J93+\/F7By+UMr4yodPsN9DS6nTt30vr164PfiiF4\/SARnPnBk4gSfPnFF0cLztzmjFd4w6u8ixcvpkmTJmGF93PavBC89957b1BuwLW6Y8aMCQQvv3jz588PhOaqVauCldnzzz8\/VzYKATtkyJDEY8bSiF0OSlUtDW9W401r3Hp0bkVPDIPgzUV6kcH46k4HqnptgjO9+Kq2Dr5UI6rfHjjTj7FKD6p0h8qYbNpyXvCKldxjjjmGfvGLX+xzsQSv\/nINLd+6Jo4KywqorOAVZQzdu3eX9qki8XAcWVZm04\/DxJ4eM9sjwJltBtL5B1\/p8HKhNzhzgQX5GFToDnlv7vd0XvCKUxqGDRtGl112WSyijzzySHC1sGwpQrHjxYSQLbVxTfRJW++rIvFwHJm5FwoTuzmsVXkCZ6qQNGMHfJnBWaUXcKYSTf22VOgO\/VGa8+C84OWV2yuvvJLOPfdcGjVqVCwy48aNoz\/+8Y\/SF0+Iel02Fr1AolQ9ruwlE3FBqki8cfNX07j5awLzOI5M70uCiV0vvjqsgzMdqOqzCb70YavLMjjThaweuyp0h57I7Fh1XvDyaQy33norPfnkk\/SrX\/0qEL7icgn+GQvdG2+8kS644ALpiycE1NGrhaO3rEXP1432j1JWamVYReLhOmFzLwkmdnNYq\/IEzlQhacYO+DKDs0ov4EwlmvptqdAd+qM058F5wctQ8Fly11xzDb311lvB5RJ8FBk3PiKFV4C\/+tWvEm9sC59vaw5COU95Ew\/1u3I4q+qFiV0VkubsgDNzWKvwBL5UoGjWBjgzi3deb3l1R17\/ro33QvAyaHwqA9fo8n9bt24NcGTx27dv3+C\/5s2bu4ZtvXjyJl5U8OI6Yb10Y2LXi68O6+BMB6r6bIIvfdjqsgzOdCGrx25e3aEnKntWnRe8XLbw2GOP0fHHH098UoOvLW\/ioX7XLPOY2M3ircIbOFOBojkb4Msc1qo8gTNVSJqxk1d3mInSnBfnBS\/fdsWnM3zve99LPBvXHGzpPeVNPNTvpsc8zwhM7HnQszMWnNnBPatX8JUVOXvjwJk97LN4zqs7svh0eYzzglec0sA3qVVVVbmMZcnY8iZet7EvE5c1cLv78gq6\/LRyb7HwIXBM7D6wVD9GcOYXZ+DLL744WnDmF2d5dYdfT5scrfOClx\/hueeeo5\/\/\/OfBSu93v\/tdOuigg\/Z5sv33358OPfRQ4j9dbHkSD\/W75hnFxG4e87wewVleBM2OB19m8VbhDZypQNGcjTy6w1yU5jw5L3jFxRN8UkOpxic0yF48YQ7eLzzlSbzohRMfTfymjUdoVD4xsftHNzjzizPw5RdfWOH1j688usO\/p02O2HnBy6czLFq0KDiCrFQ78MADqUePHs6e1pAn8bBhLTmRVffAh7FqRPXbA2f6MVbpAXypRNOMLXBmBmdVXvLoDlUxuGTHecHrElh5YsmTeOENa1y7yzW8aHoRwMSuF18d1sGZDlT12QRf+rDVZRmc6UJWj908ukNPRHatQvAawj9r4kXrd5ePOTO4VhhNLwKY2PXiq8M6ONOBqj6b4EsftrosgzNdyOqxm1V36InGvlUnBa+o22V47rzzTrr++uuD29ZKtYZawxut38WFE2ZeGkzsZnBW6QWcqURTvy3wpR9j1R7AmWpE9dqD4K2Pr5OCd8uWLXTPPfcEkX7\/+9+nBx98kPjfSrWWLVvStddeS\/yniy1r4oUFL6\/s8govmn4EMLHrx1i1B3CmGlG99sCXXnx1WAdnOlDVZzOr7tAXkV3LTgpeu5Do8Z418XDhhB4+kqxiYk9CyL2fgzP3OCkVEfjyiy+OFpz5xVlW3eHXU8pH64Xg5euFn3nmGaqpqaGf\/vSndPDBB9PWrVuDiygOOOAAuummm+i4446Tf2oLPbMmXljwjurdkUb17mQh+sbnEhO7f5yDM784A19+8QXB6x9fWXWHf08qF7EXgvfpp5+m4cOHU9euXWnKlCnUunXroMThV7\/6FT3xxBPER5Lde++9VFlZKffUFnplSTxcOGGBqM9d4sPYHvZZPYOzrMjZGQe+7OCexys4y4Oe+bFZdIf5KM15dF7wbt++nQYPHhycr8sb2MrKyuqhs2nTJho6dGhw+xqLYRa\/LrYsiYcNa\/aYxMRuD\/usnsFZVuTsjANfdnDP4xWc5UHP\/NgsusN8lOY8Oi94xYkN11xzDV1xxRWxyDzwwAPBCm9Du2ltzqvraNiclcEzY8OauZeCPWFiN4u3Cm\/gTAWK5myAL3NYq\/IEzlQhacYOBG99nJ0XvBs2bKDLLruMLr744uB4srg2efJkevjhh+mhhx6itm3bmsmklF6yJB42rKUEWWF3TOwKwTRkCpwZAlqRG\/ClCEiDZsCZQbAVuMqiOxS4ddaE84L3k08+Cep3V65cSdOmTaNOnepv2lq9ejVdffXVVFFRQRMnTgw2sbnYsiQeNqzZYxITuz3ss3oGZ1mRszMOfNnBPY9XcJYHPfNjs+gO81Ga8+i84GUo3njjDRo0aBBt27aNTj75ZDr66KMDhDZu3EgvvvgitWjRgu677z468cQTzSGX0lOWxDt0+AsFL7hwIiXgObtjYs8JoIXh4MwC6Dlcgq8c4FkaCs4sAZ\/RbRbdkdGVF8O8ELyM5Pvvv0+\/\/OUv6bnnnqPdu3cH4DZr1ozOO+88+vGPf0zt27d3GvC0iRfdsIYrhc3Si4ndLN4qvIEzFSiaswG+zGGtyhM4U4WkGTtpdYeZqOx58Ubw2oNIjee0iYcNa2pwz2oFE3tW5OyNA2f2sM\/iGXxlQc3uGHBmF\/+03tPqjrT2fesPwWuIsbSJV710HQ2djRMaDNGzjxtM7LaQz+4XnGXHzsZI8GUD9Xw+wVk+\/EyPTqs7TMdn2h8EryHE0yZeeMPad45vQw\/+0N36ZEMQGnWDid0o3EqcgTMlMBozAr6MQa3METhTBqURQ2l1h5GgLDqB4DUEftrEC29Yw5XChkgKucHEbh7zvB7BWV4EzY4HX2bxVuENnKlA0ZyNtLrDXGR2PEHwGsI9beLhhAZDxBRxg4ndLv5ZvIOzLKjZGwO+7GGf1TM4y4qcnXFpdYedKM15heDNgDWfCzxv3rzCSL7SuFevXiUtpUk8nNCQgRTFQzCxKwbUgDlwZgBkhS7Al0IwDZkCZ4aAVuQmje5Q5NJpMxC8Kelhsbt06dLCNcYLFiygoUOH0ogRI6iqqqqotTSJN27+aho3f01gC1cKpyRIUXdM7IqANGgGnBkEW4Er8KUARMMmwJlhwHO6S6M7crryYrg3gpdvVPvDH\/5A\/\/jHP2KBbdmyJV177bXEf+pqK1asoAEDBtD48ePrreiyCK6rq6Pp06dTWVlZrPs0iYcrhXUxKG8XE7s8Vq70BGeuMCEXB\/iSw8mlXuDMJTaSY0mjO5Kt+d\/DC8H79NNPB9cLiwsn4mDv0KFDYdXVNC06Be9vBxxPF3c73PQjNXp\/mNj9SwFw5hdn4MsvvjhacOYXZxC89flyXvBu376dBg8eTB988AH95je\/oeOPP56aNGniTNZNnjyZJkyYQEl1vCLxZs+eTSzORSsvL9\/nWQ6\/8cXCv026tAtdftqRzjxvYwkEE7t\/TIMzvzgDX37xBcHrPl\/r1q2rF2RtbS3179+fampq6ukO959ET4TOC14msG\/fvnTllVfS1VdfrQeFDFZF7S4P7dOnD02cOLGkFSF4o524RGLgwIGFf67b+ildOLO28P\/3XlJOp7T\/UoYIMSQPArt27aINGzYQ\/0LStGnTPKYw1hAC4MwQ0IrcgC9FQBo0A84Mgp3B1cyZM2nWrFn7jITg3QuJ84J306ZNNGjQIDr\/\/POdErwio8QKNNfwVldXBwIprgnBy\/W\/7du3L3Th\/uExC1dtor73vfWFAL7tzAxpjyF5Edi5cyetX78++K0YgjcvmmbGgzMzOKvyAr5UIWnODjgzh3UWT7xAGF7lXbx4MU2aNAkrvJ+D6bzg5TjvvfdeeuaZZ4I\/27ZtmyUPtI4Rm9mGDBlS9KQG2VoanNCglSpp4\/i6VRoqZzqCM2eokAoEfEnB5FQncOYUHYnByOqOREMNpIPzgpd\/o\/zjH\/9Iv\/vd7+itt96ic845h1q0aLEP\/CZOaSjGuUrBO2zOSprz6t46nB6dW9ETwyobSKr59RiY2P3ii6MFZ35xBr784gvvmH98QfDW58x5wStqeLn4ulQzcUoD1+2OHDkyqJHp2rVrIRxRz1tq45ps4oWPJMOVwvYmGHwY28M+q2dwlhU5O+PAlx3c83gFZ3nQMz9WVneYj8yOR+cFrx1Y4r2Kel3+qThzV6zuVlRUKDmHN3ylMASvPfYxsdvDPqtncJYVOTvjwJcd3PN4BWd50DM\/FoLXsxVe8ymS7DF6tXDSLWtsUSbx\/vHRx9Rt7MuFAB6vqqSzurRKDgg9lCOAiV05pNoNgjPtECt1AL6UwmnEGDgzArMyJzK6Q5kzDwx5s8L77rvv0tixY+nll1+mww8\/PDgR4aCDDqKbbrqJvv3tb9OFF17o1Pm8Ue5lEm\/hqs100eRlhaHLx5wZXC2MZh4BTOzmMc\/rEZzlRdDsePBlFm8V3sCZChTN2ZDRHeaise\/JC8H7xhtvBEeT7bffftSxY0dau3ZtIHgPPPDA4N9XrlwZXErRq1cv+4gWiUAm8XizGm9a48ZClwUvmh0EMLHbwT2PV3CWBz3zY8GXeczzegRneRE0O15Gd5iNyK435wXvJ598ElwrzJvW+FiyZcuWBSu94szbrVu3Bufz8movbxpjEexik0k8HEnmDnOY2N3hQjYScCaLlBv9wJcbPKSJApylQct+XxndYT9KcxE4L3j5tqvLLruMvve97wVn3PKJCGHBy1BNmzaN7r\/\/\/pIXP5iDNN6TTOKFT2jAkWR2GcPEbhf\/LN7BWRbU7I0BX\/awz+oZnGVFzs44Gd1hJzI7Xp0XvOJYsmuuuYauuOKKWMH7wAMPBKu\/pW46swPvF15lEg9Hktlm6Qv\/mNjd4UI2EnAmi5Qb\/cCXGzykiQKcpUHLfl8Z3WE\/SnMROC94xVFgbdq0oYkTJwaXUMSVNBxwwAE0depUOvjgg82hl8JTUuJFT2i4+\/IKuvy0+GuKU7hF14wIYGLPCJzFYeDMIvgZXIOvDKBZHgLOLBOQ0n2S7khpzvvuzgteRvjpp58mPvqLV3iPPvpouueee4J6Xa7rveuuu4JNa+PGjQvKHlxtSYmHI8ncYg4Tu1t8yEQDzmRQcqcP+HKHC9lIwJksUm70S9IdbkRpLgovBO+ePXuC283Gjx9PO3bsqIfO\/vvvH2xq45IH\/rurLSnxcCSZW8xhYneLD5lowJkMSu70AV\/ucCEbCTiTRcqNfkm6w40ozUXhheAVcPCJDEuWLAn+2717N51yyil01llnUevWrc0hltFTUuLhSLKMwGoaholdE7AazYIzjeBqMA2+NICq2SQ40wywYvNJukOxO+fNeSV4nUezRIBJiRc+kgwnNNhnGhO7fQ7SRgDO0iJmtz\/4sot\/Fu\/gLAtq9sYk6Q57kdnx7I3g5bKGd955h37\/+9\/Ttm3bArR4I9sll1xCnTp1soNeCq9JiYcjyVKAaaArJnYDICt2Ac4UA6rZHPjSDLAG8+BMA6gaTSbpDo2unTTtheDlUoaf\/OQnweY1Fr7h1qRJE+rXrx\/dfPPN1KxZMydB5qCSEq\/b2JeJN65xG9W7I43q7b6IdxZsBYFhYlcAomET4Mww4Dndga+cAFoYDs4sgJ7DZZLuyGHay6HOC14WuLxZ7b777qNhw4bRwIED6ZBDDgnAZiHMl07wGbw33HBDsHHN1ZaUeIcOf6EQ+uNVlXRWl1auPkqjiAsTu380gzO\/OANffvHF0YIzvzhL0h1+PU3+aJ0XvBs3bqQrr7ySTjvtNLrllluIV3TDjQUxr+6+8cYbgSh2dQNbqcTDkWT5E1m1BUzsqhHVbw+c6cdYpQfwpRJNM7bAmRmcVXmB4K2PpPOCV9y0xqu7fMVwXHviiSeCVWBfb1rDkWSqXm91djCxq8PSlCVwZgppNX7AlxocTVoBZybRzu8Lgtczwbtr1y66\/vrrg\/pcvmktWqfLx5ONHj06KG+488476cADD8yfJRoslEo8HEmmAfCcJjGx5wTQwnBwZgH0HC7BVw7wLA0FZ5aAz+gWgtczwcvh8o1qXJ971FFH0Y033kgdO3ak\/fbbj3j1l29aW7BgAf3yl7+kE044ofB0fAlF27ZtM6aJ+mGlEu\/BJWvp+rlvBU6\/fOiXaPmYM9UHAIupEMDEngouJzqDMydokA4CfElD5UxHcOYMFVKBQPB6JnhFSQOL3jStQ4cOVFNTk2aI1r6lEg9HkmmFPpNxTOyZYLM6CJxZhT+1c\/CVGjLrA8CZdQpSBQDB65ng3blzJy1atIi4tCFN49KG8847L80QrX1lBS+OJNNKg7RxTOzSUDnTEZw5Q4VUIOBLCianOoEzp+hIDAaC1zPBm8ioJx1KJR7O4HWPREzs7nGSFBE4S0LIrZ+DL7f4kIkGnMmg5E4fCF6PBe+mTZto4cKF9NprrwWb1yorK+mMM85w9iiyMNTFEg9HkrkzOYQjwcTuJi+logJnfnEGvvzii6MFZ35xBsHroeD97LPPgssl+JQG\/nu48eY0PrKsqqrKy5vWIHjdnEAwsbvJCwSvf7wUixjvmH9cgjO\/OIPg9VDwPvzwwzRq1Cg655xzghvV+JQGbmvWrKHbb7+dXnrppeAc3osuusjZbCyWeNEzeD+a+E1nn6ExBYaJ3T+2wZlfnIEvv\/jCCq9\/fEHweiZ4t2\/fToMHD6aysrLgnN3mzZvXewLe1Mbn9PLK75QpU7w7h3fc\/NU0bv6a4JlwJJk7Ewo+jN3hQjYScCaLlBv9wJcbPKSJApylQct+XwhezwSvOJaMz+G94oorYjPogQceCEoefLxpbdYrdfSj6reD5+rRuRU9MazS\/luCCFCr5mEO4MPYL9LAl198YYXXP74geD0TvBs2bAiuFL744ouDldy4xiu\/jz76KD300EPSl02IleMlS5YUTI4YMSKoBU5qkydPpgkTJqQaVyzxwmfwnntsa3pkaLck9\/i5AQTwYWwAZMUuwJliQDWbA1+aAdZgHpxpAFWjSQhezwTvJ598QsOHD6eVK1fStGnTqFOnTvWeYPXq1XT11VdTRUVFsKntgAMOSEwfsWrcrl07mj59elAusWLFChowYAD17NkzsFOssdidOnUqzZo1i7p27VoYN2TIkJJiWUbw4gzeROqMdcDEbgxqZY7AmTIojRgCX0ZgVuoEnCmFU7sxCF7PBC+H+8Ybb9CgQYOCyye+9a1vUY8ePYKn4Aspnn322aBul0sa+JgymcZXEY8cObIgWsUYFrNz584tWhohhPKpp55aTxSzIF+6dGnJkopiiXfo8BcKIUPwyrBnpg8mdjM4q\/QCzlSiqd8W+NKPsWoP4Ew1onrtQfB6KHg55LfffptGjx5Ny5cvpz179gRP0aRJE+rWrRvddtttdNxxx+XOnOjqbdSgasGLI8lyU6bNACZ2bdBqMwzOtEGrxTD40gKrVqPgTCu8yo1D8HoqeEXYvMrLF1Bwa926tdJTGWRWaouVNCSVQsQlHgSv8vdbmUFM7MqgNGYInBmDWokj8KUERqNGwJlRuHM7g+D1XPDmzoAiBrjMYejQoSSzcU3U+27bti2wxseh9erVq2RoIvFmz55NHTp0CPqu+biMLpq8rDDug1+drevxYDclApjYUwLmQHdw5gAJKUIAXynAcqQrOHOEiCJh8LfQ4VZbW0v9+\/enmpqagu5w+wn0Rtdkj6gP0OvHaetCwPLGN7GJLS5gcbJDXV1doV43bgNc3FgheMM\/2\/3lHrTj5EHBP7U7pCk9MXCvEEazjwB\/k8AnhJSXl1PTpk3tB4QIEhEAZ4kQOdUBfDlFh1Qw4EwKJmudZs6cGexNijYI3r2INHrBKyt2GSyxChxd0RU2+La3Yiu9QvByn\/bt2wfgP\/jmp\/Tgm58Ef+dLJ14ZIbfpztrb1Igc84Um69evD34rhuD1g3hw5gdPIkrw5RdfHC04c5szXoALr\/IuXryYJk2ahBXez2lr1IJXCNju3buXXNkVKV7sdAexytuvX7+iR5PF1dKM+P07dN+i9wPzuHTCrYkEX925xYdMNOBMBiV3+oAvd7iQjQScySLlRj\/U8NbnodEKXiF2+\/TpU\/Lc3TBcKlZ4w18thC+duPy0crr78go33hJEgZvWPMwBfBj7RRr48osvjhac+cUZBC8Er\/QlE9HUVlHDGxa83ca+THxSAzecwevWRIKJ3S0+ZKIBZzIoudMHfLnDhWwk4EwWKTf6QfBC8AY3t82bN69oRooa3WLn8kbHy6wSxyVe+NIJXt3lVV40NxDAxO4GD2miAGdp0LLfF3zZ5yBtBOAsLWJ2+0PwQvBaycBo4uEMXis0SDvFxC4NlTMdwZkzVEgFAr6kYHKqEzhzio7EYCB4IXgTk0RHhyTBu3zMmcFJDWhuIICJ3Q0e0kQBztKgZb8v+LLPQdoIwFlaxOz2h+CF4LWSgdHEW7hqc71LJz6a+E0rccFpPAKY2P3LDHDmF2fgyy++OFpw5hdnELwQvFYyNpp44+avpnHz1wSx8Mour\/CiuYMAJnZ3uJCNBJzJIuVGP\/DlBg9pogBnadCy3xeCF4LXShZC8FqBPbNTTOyZobM2EJxZgz6TY\/CVCTarg8CZVfhTO4fgheBNnTQqBkQTL3wGLy6dUIGwWhuY2NXiacIaODOBsjof4EsdlqYsgTNTSKvxA8ELwasmk1JaKSV4celESjANdMfEbgBkxS7AmWJANZsDX5oB1mAenGkAVaNJCF4IXo3pVdx0NPFw6YQVGqSdYmKXhsqZjuDMGSqkAgFfUjA51QmcOUVHYjAQvBC8iUmio0M08XDphA6U1dnExK4OS1OWwJkppNX4AV9qcDRpBZyZRDu\/LwheCN78WZTBQjjx\/n1QG+IVXtEer6qks7q0ymAVQ3QhgIldF7L67IIzfdjqsAy+dKCq1yY404uvausQvBC8qnNKyh4ErxRMznTCxO4MFdKBgDNpqJzoCL6coCFVEOAsFVzWO0PwQvBaScJSgheXTlihpKRTTOzucZIUEThLQsitn4Mvt\/iQiQacyaDkTh8IXgheK9kYTrwX1zalYXNWBnHg0gkrdCQ6xcSeCJFzHcCZc5Tgl0q\/KEmMFu9YIkROdYDgheC1kpDhxHvwzU9wy5oVFuSdYmKXx8qVnuDMFSbk4gBfcji51AucucRGciwQvBC8yVmioUc48X754jaa8+q6wAsundAAtgKTmNgVgGjYBDgzDHhOd+ArJ4AWhoMznD7VhQAAHSFJREFUC6DncAnBC8GbI32yDw0n3rWPbaBF726G4M0Op\/aRmNi1Q6zcAThTDqlWg+BLK7xajIMzLbBqMwrBC8GrLblKGS4meEf17kijeneyEhOcFkcAE7t\/2QHO\/OIMfPnFF0cLzvziDIIXgtdKxoYT74IZ\/6R\/fPRxEMfdl1cQXy2M5hYCmNjd4kMmGnAmg5I7fcCXO1zIRgLOZJFyox8ELwSvlUwMJ95JE\/9WiAGXTlihI9EpJvZEiJzrAM6co6RkQODLL76wwusfXxC8ELxWslYk3gPzFhCv8IoGwWuFjkSn+DBOhMi5DuDMOUogeP2iJDFavGOJEDnVAYIXgtdKQorE++XvHifetCba8jFnBmfxormFACZ2t\/iQiQacyaDkTh\/w5Q4XspGAM1mk3OgHwQvBayUTiwle3LJmhY5Ep5jYEyFyrgM4c44SrPD6RUlitHjHEiFyqgMELwSvlYSMK2nALWtWqJByioldCianOoEzp+hIDAZ8JULkXAdw5hwlJQOC4IXgtZKxIvG++5NZxDetcYPgtUKFlFNM7FIwOdUJnDlFR2Iw4CsRIuc6gDPnKIHgTUFJkz179uxJ0R9dMyIgBO+J195LL67dP7CCW9YygmlgGCZ2AyArdgHOFAOq2Rz40gywBvPgTAOoGk1ihRcrvBrTq7hpkXit+k2iNR+XQfBaYUHeKSZ2eaxc6QnOXGFCLg7wJYeTS73AmUtsJMcCwQvBGyCwfft2Gjx4MC1ZsqSAyIgRI6iqqioxixYsWEBDhw4t9OvevTtNnz6dysr2Ctm4Fid4cctaItTWOmBitwZ9ZsfgLDN0VgaCLyuw53IKznLBZ3wwBC8EL61bt4769u1L7dq1KwjVFStW0IABA6hnz540ceLEook5efJkmjBhAk2ZMoV69eoVa6uU4N3aaxz9+6A2QRcIXuPvv7RDTOzSUDnTEZw5Q4VUIOBLCianOoEzp+hIDAaCF4KXeIV25MiRNGvWLOratWsBERazc+fOperqaiov3\/e6XyGU+\/XrV28luJi9MNQi8Tb3+W3hn3HpROL7aq0DJnZr0Gd2DM4yQ2dlIPiyAnsup+AsF3zGB0PwQvCWXL2dOnXqPkJYDGBhO3bs2KKCuFQ2c+L1G\/xfxCu8okHwGn\/\/pR1iYpeGypmO4MwZKqQCAV9SMDnVCZw5RUdiMBC8ELxFk2T48OG0dOnSooKWV4Bramrotttuo6uuuopqa2sDW3369ClZBsF9IHgT302nOmBid4oOqWDAmRRMznQCX85QIR0IOJOGyomOELwQvLGJKDaildq4xoJ43rx51KJFi8IqcFw9cJwDTry+\/3ULbT\/rxsKPn7zqKOr+tY5OvBgIoj4CmNj9ywhw5hdn4MsvvjhacOY2Z6xHwo0X5fr37x8s1HXo0MHt4A1Eh3N4iUhsWKuoqCh52oIQvGLDmuBHiOXov4f5ixO8reb9MNgoN3DgQANUw0UaBHbt2kUbNmwIarmbNm2aZij6WkIAnFkCPqNb8JUROIvDwJlF8CVcz5w5M1iMizYI3r2INHrBKyt2GSwWvM8\/\/\/w+Nb7FNrNFBe8ltz5EH3\/1ouCf+Za1Kec1CQRV3AY5idxGF40I7Ny5k9avXx\/8VgzBqxFohabBmUIwDZgCXwZAVuwCnCkGVLE51iLhVd7FixfTpEmTsML7Oc6NWvCKlVmZc3QZL67hjdvUllXwLh9zpuJ0hzlVCOCrO1VImrMDzsxhrcIT+FKBolkb4Mws3nm9oYa3PoKNVvAKsSuz4UxAJlaDhwwZkulYsj4TnqPdX+4RmMO1wnlfZb3jMbHrxVeHdXCmA1V9NsGXPmx1WQZnupDVYxeCF4K3ULObdMlEXApGyxrE6u6pp55a8qQGTryLJi+nT9scF5i9\/LRyuvvyCj1ZDqu5EcDEnhtC4wbAmXHIczkEX7ngszIYnFmBPbNTCF4I3qAWl09bKNbE5rNiJQzitjUxXmaVOCp4ccta5nfYyEBM7EZgVuoEnCmFU7sx8KUdYuUOwJlySLUahOCF4NWaYMWMc+JdMOOfuFbYCvrpnWJiT4+Z7RHgzDYD6fyDr3R4udAbnLnAgnwMELwQvPLZorAnJ9751TsLFrmcgcsa0NxEABO7m7yUigqc+cUZ+PKLL44WnPnFGQQvBK+VjI0KXj6hgY8mQ3MTAUzsbvICwesfL8UixjvmH5fgzC\/OIHgheK1kbFTwPl5VSWd1aWUlFjhNRgATezJGrvUAZ64xUjoe8OUXX1jh9Y8vCF4IXitZixVeK7BndooP48zQWRsIzqxBn8kx+MoEm9VB4Mwq\/KmdQ\/BC8KZOGhUDooL3o4nfVGEWNjQhgIldE7AazYIzjeBqMA2+NICq2SQ40wywYvMQvBC8ilNKzlxY8HLtLm5Zk8PNVi9M7LaQz+4XnGXHzsZI8GUD9Xw+wVk+\/EyPhuCF4DWdc4E\/CF4rsGd2iok9M3TWBoIza9Bncgy+MsFmdRA4swp\/aucQvBC8qZNGxYCw4MW1wioQ1WsDE7tefHVYB2c6UNVnE3zpw1aXZXCmC1k9diF4IXj1ZFaCVQheK7BndoqJPTN01gaCM2vQZ3IMvjLBZnUQOLMKf2rnELwQvKmTRsWAsODlCyf44gk0dxHAxO4uN8UiA2d+cQa+\/OKLowVnfnEGwQvBayVjw4J3VO+ONKp3JytxwKkcApjY5XByqRc4c4mN5FjAVzJGrvUAZ64xUjoeCF4IXisZC8FrBfbMTjGxZ4bO2kBwZg36TI7BVybYrA4CZ1bhT+0cgheCN3XSqBgQFrxczsBlDWjuIoCJ3V1uikUGzvziDHz5xRdHC8784gyCF4LXSsaGBS+uFbZCQSqnmNhTweVEZ3DmBA3SQYAvaaic6QjOnKFCKhAIXgheqURR3QmCVzWieu1hYteLrw7r4EwHqvpsgi992OqyDM50IavHLgQvBK+ezEqwGha8fMsa37aG5i4CmNjd5aZYZODML87Al198cbTgzC\/OIHgheK1kbFjwfjTxm1ZigFN5BDCxy2PlSk9w5goTcnGALzmcXOoFzlxiIzkWCF4I3uQs0dBDCF5e2eUVXjS3EcDE7jY\/cdGBM784A19+8YUVXv\/4guCF4LWStRC8VmDP7BQfxpmhszYQnFmDPpNj8JUJNquDwJlV+FM7h+CF4E2dNCoGCMHbo3MremJYpQqTsKERAUzsGsHVZBqcaQJWk1nwpQlYjWbBmUZwNZiG4IXg1ZBWySY58S6Y8U8688QuELzJcFnvgYndOgWpAwBnqSGzOgB8WYU\/k3Nwlgk2a4MgeCF4rSQfEs8K7JmdYmLPDJ21geDMGvSZHIOvTLBZHQTOrMKf2jl0BwRv6qRRMQCJpwJFczYwsZvDWpUncKYKSTN2wJcZnFV6AWcq0dRvC7oDgld\/lsV4QOJZgT2zU0zsmaGzNhCcWYM+k2PwlQk2q4PAmVX4UzuH7oDgTZ00KgYg8VSgaM4GJnZzWKvyBM5UIWnGDvgyg7NKL+BMJZr6bUF3QPAGCGzfvp0GDx5MS5YsKSAyYsQIqqqqSpWFkydPprlz51J1dTWVl5cXHYvESwWr9c5r1qyhGTNmBDnSoUMH6\/EggGQEwFkyRi71AF8usSEXCziTw8mVXtAdELy0bt066tu3L7Vr146mT59OZWVltGLFChowYAD17NmTJk6cKJWvYkzLli0heKUQ86cTJgp\/uBKRgjO\/OANffvHF0YIzvzgDXxC8tGDBAho5ciTNmjWLunbtWkBEdrU2ukLMK4BY4fVrIkiKFhNFEkLu\/RycucdJqYjAl198QfCCL\/8QgOAtyhkL3qlTp+4jhOMGcN+amhqqrKykp556CoLX9zchEj8+jP0jFJz5xRn48osvCF7w5R8CELxFORs+fDgtXbo0UbyGV4gXLVqUqoZ39uzZqAn14K2pra2l\/v37E\/jygKzPQwRn\/nDFkYIvv\/gCZ\/7yxYtz2ItC1GTPnj17\/KNRfcQsYocOHUpJG9dE\/W+\/fv2CDW6yZRA8uXMZxeLFi9UHD4tAAAgAASAABIAAEIggcPrpp9OcOXOAC0HwBkkgNp9VVFQUNrEVyw5eBa6rqyv0kxW84rdjFr5oQAAIAAEgAASAABDQjQCv7GJ1dy\/KjX6FN43Yjdvslkbw6k5s2AcCQAAIAAEgAASAABDYF4FGLXhFGUP37t0TV3YZOl7dnTdvXtE8SiqHQAICASAABIAAEAACQAAImEeg0QpeIXb79Okjfe5uHD1Y4TWftPAIBIAAEAACQAAIAIE0CDRKwZvlkolioELwpkk39AUCQAAIAAEgAASAgHkEGqXgTSpNmDJlCvXq1Ss4gSHpXF4IXvNJC49AAAgAASAABIAAEEiDQKMUvGkAQl8gAASAABAAAkAACAABvxGA4PWbP0QPBIAAEAACQAAIAAEgkIAABC9SBAgAASAABIAAEAACQKBBIwDB26DpxcMBASAABIAAEAACQAAIQPBqzoHt27fT4MGDacmSJYEnvvGkurqaysvLNXuG+SQEwpsXW7RoQbNmzaKuXbuWHCaOsxOdwGcSyup+noWvsPfoteDqIoOlYghk4Sw6Z7JtsZEYSOtFIAtf4r0St4hiTtTLURbrzCu3iRMnZhneYMZA8GqkUkzc7dq1KyQaJ97SpUshejXiLmM67opomRM5JkyYUO\/Dl+08\/\/zzUmJZJi70iUcgC19RS+LDHBfEmMmyLJwJ8cRz5vTp06msrCw4LSf63pl5gsblJQ9fp556Kj7jHE0X8f7kvXPA0cdLFRYEbyq40nWOO9YMq0zpMNTRW6zShleN4n45Cfsu9nPwqYOh+jaz8BWNKrwyD8HrLmdxxzwmvZv6n6bhe8j6jsV9xolz7ocMGUJVVVUNHzxHnzD6TQkELxEEr8Zkjf7GLFwV+3eNocB0CIFiZydnOVNZCN7wCgfAVotAXr4ER1xaxKuG\/fr1wwexWor2sZaFM\/EBfc4554AfzfxEzWfhi21A8BomStKdeJfq6upoxowZNHr0aAp\/0yxppsF1g+DVRGmpVQmUNWgCXdJssV84ZC4aibrAaoYk6Dm65eEr\/B7eeOON1LdvXwjeHFzIDs3CmfjFZMyYMbRq1aqgjIGbbH29bGzoty8CWfhiK3G\/8KPMy60MwzckX\/ABwaspN0slWZaVRE1hNkqzxSZ3\/lpv5MiR0vW44d+isRFRXyrl4Sv8rnGEELz6eApbzsKZ+OVx27ZtFP76FTW8+jnLwleU73nz5gX\/1L1790L9tf7I4SEJAQheCN6kHMn9cwje3BBqM5B3cheBiU1Q2EGujarAcFa+hIAaP358cFU46q318qRK8FZUVNQTTGIuZftiI5u5J2kcnrK+Y6L2N1wXj19Q3MoZCF4IXu0ZiZIG7RBndpD167u4FQ2I3cw0SA\/Mwlfc+wfBKw157o5ZOBO\/oPTs2XOf45PwrVhuSkoayMJXqV9EsE9FL19prEPwQvCmyZfMfbFpLTN0Wgdm3aDBQYV3vkLsaqWpYDwLX+Gvx+OixFmhernLwlmpDaAQvO7xhW8x9XKiyjoELwSvqlwqaSduksYqkxHoSzqJq9WVmRREn5UrV0rX+dp\/Wv8jyMpX9Mnx7pnLhaycxW3olXk3zT1Zw\/SUhS+s8PqRC3h\/IHiNZGrcIeo4ocEI9CWdxG02kzmhAbuP7XCXlS8IXjt8hb8J4WORxIZOmXcsrqxBZpy9J20YnrO+Y6jhdZ9\/CF4IXmNZGj38GV+lGoM+0ZHYdMYd444+Cv9ysn79ehowYADxDvK4hp3JiXDn7pCGr7iru7HCm5uC1AaycBa9qhbHkqWGPfOALHxFy4fAV2b4tQyE4IXg1ZJYMAoEgAAQAAJAAAgAASDgHgI4h9c9ThAREAACQAAIAAEgAASAgEIEIHgVgglTQAAIAAEgAASAABAAAu4hAMHrHieICAgAASAABIAAEAACQEAhAhC8CsGEKSAABIAAEAACQAAIAAH3EIDgdY8TRAQEgAAQAAJAAAgAASCgEAEIXoVgwhQQAAJAAAgAASAABICAewhA8LrHCSICAkAACAABIAAEgAAQUIgABK9CMGEKCAABIAAEgAAQAAJAwD0EIHjd4wQRAYFGhcBnn31Gjz32GO3cuZO+\/\/3vp372999\/n6ZPn05VVVXUtm3b1OPzDHjuuefo5z\/\/OfEVuu3bt6e5c+cGf6Zt4naxU089lSZOnJh2uHP9o7eljRgxIuDHRoveBDZlyhTq1auXjVDgEwgAAYsIQPBaBB+ugQAQIMp75e\/kyZMDoVldXU1xVwrrwnjr1q109dVXB2J36NChdMQRR1CPHj2oefPmqV02VMF75JFHBldyH3PMMXTsscemxkXFgE2bNtHixYvpz3\/+c\/CLEQSvClRhAwj4hwAEr3+cIWIg0KAQ8FXwqhSpKm25kBwuPs+CBQuCX0wgeF3IEMQABMwjAMFrHnN4BAKNBoE9e\/bQE088QbfffjvV1tbSAQccQN26daNbbrmFjjvuOIp+3dyiRQuaNWsWde3albZv30533XUXzZs3jz744ANq0qQJdejQgW644Qa68MILg\/8fPnx48HPR+vTpUygJePvtt+lnP\/tZsLLH7aSTTqIbb7yRTj\/99ET8uUzizjvvpCeffJJ27NhB7dq1ox\/+8IdByUWzZs1IiKewoVJf2zMOzzzzDP3617+m9957L8DhzDPPpDFjxlDnzp0Lq9yVlZXUvXt3+s1vfkMbNmwI\/HLM4nnZ3+7du+nBBx+kGTNmBJiy7cMPP5wGDRpEV111VRAfN7Hy\/YMf\/CDw++9\/\/5tuvfVWuuSSS+jdd98N\/v7SSy\/R\/vvvT5dffjmdeOKJAS8Cf7bBZSbs57777qMPP\/yQDjvsMPrP\/\/xPuu6666isrKwojnGCl\/kcPHhw8Eznnnsu\/eIXv6CNGzcGeTB69OhgdZw5LdaKreTLrvBD8CamPToAgQaNAARvg6YXDwcE7CLw1FNPBQL1G9\/4BvXu3Zs2b95Mv\/3tbwNhM3v2bGKBy0JkwoQJ9PWvf52+853vBIL04IMPDsTsH\/\/4R\/re975Hp512Gq1evZrmzJkTCK+pU6cGNl977bVAoLFw+9GPfkRf\/X\/tnTtoFV0UhQ9pDBaiYqtGQTDByjQWKvhIo0KwMAaJ2CWIiFiIWlkkoGijVUALEW0CeYBFIEgiaETRgJIiglqI9oJIiqDFz7dhX47zz0zOzcPcm6wDIpi5c85Zcw3frFl7z+7dobW11c7J55uamkJXV5eJ8OTJk\/DlyxcD4mPHjhUKA0QCjgAnf2\/fvt2g+tWrV6Gjo8OgEFB7\/vx5uHv3bti5c2fpY3uAFFeRPTY3N9t6AELWs3nz5vDw4UODTs7NvxMDOHfuXFi3bp2BJutAMzTgXHfu3AkPHjwwCD548GD4+fOnnQuQvnHjhq3FgZcbjU2bNoXu7m6b4\/Dhwwbb7ItrwVo2bNhgUAvcAtPxDQdgOzU1VbkG7969C4ODg3bTAmjy2bxRBrwfP360NZw6dcq0ZT7WDuSXZWsFvCv7f1mzS4F6V0DAW+9XUOuXAjWsANCJWwqgORxNTk4ajPb19Rng5EUacGcpcsJNjIudAC6cVh5N+79nQYhsLUBHnhfg80wtMMe8ZG4BvDxYAyiBRorogE3gmUFhHcCKs+rwmfrY\/tu3b+HMmTMGicC2O7Bv3rwJFy9eDNevXzfYB3hxYR8\/fhx27Nhh8\/p+yQpzLKDd09NjeVgcWsCR4XPgDnvRG7qw5ps3b4bTp0\/bcb4\/3Ob79+8HHGVGDPkOvEA+Djnn2b9\/f+VbxroBaObHUa8WeD98+PDXTQcZW9xpboKKrosDfF5WWw5vDf8C0NKkQA0pIOCtoYuhpUiB1aYAwAnsAmw4j3ldFKrJ8Pqxx48fD1evXjW5ssDz\/v17c0hPnDhhj8njgfOLaxw\/to9\/DlCePXvWgPPevXsVoOQYnEn2gOPM3KnASycH4Jx4RpGD6ecCQHGN\/dF+6hweF8Ax9nWjC054vNey\/RHhoKiL44l\/4JjjoOPy4sT7wBlmjQcOHCjsKFHm8HIe5okjEayV7wmwv2fPntz\/BnJ4V9tvB+1HCvxbBQS8\/1ZvzSYF1pQCuLsOTmycrCmxBcDRXcwy4J2bm7PH3cDm27dvA+4wDm2c1c2CUF6+NhYdmAQEjx49+r9rkQfUflAW4lJhFJADdosgm\/MXnavo33Gr0XZmZsag9OXLl6YTsQeHyTzg5TO4vfzBMc7eDFy5csXWSa6YvC2aFw1caXQkfpId82V4s63XUvK1At419atDm5UCS66AgHfJJdUJpYAUiBXgMTowNjAwEMbGxirFa56lzQNegI7H8eRbiRMQP6C\/LcVsz549+8tdLALehVTjLwfw5oFnCiDmgTBaAqTkeCmmW79+vWV+6d8LnOKglwHv169frfCus7MzCXjz3NiUb\/dCgPf8+fOFNyLMKeBNUV7HSAEpUKSAgFffDSkgBf6pAp8+fbIMLgVLQCkZTvKrQJjncicmJgIARLbzwoULlcffDmwUtsVZ1TjbCfiR4SXW4LGH1A2mRBpwp3FHUx3eokgDxWjEJ9ra2gxC0SD74onsHB6rYP+9vb1WkMbwHCyFbmXAW7a\/GMxbWlrCpUuXrIsGuWWK\/6oZZcBLpprrzlp9MDc3N8xV1K+36Bi6WJArnq8Pc4qLXM0edawUkAL1pYCAt76ul1YrBepGgV+\/foXLly+bQwusePHYjx8\/DEa3bNlSCLwcDxQ9evSoUliFuwnU0MKqvb29EHi9aI2sKZ\/funWraYZrzGdpU0bhmUcqYkHnK1oDyihm43F+KvAWFa09ffrUWo7RnYDMbArwOrSRtyXH7IOoBzcLgGoZ8BYVrTkw48THRWt02CCSQo7Xc8VEKCico5UZP88b83VpiAsCaTnH94G4C9e86MUddKIA8vv7+63bBOP79+\/22T9\/\/gh46+Y3gxYqBVZGAQHvyuiuWaXAmlCATgBEE+h2AKQyRkZGzDn0NlTuOvIzXF2Koaanpw2qiDHg8gJBdE7AvaWTAbDnDq+DEOBD6zPmGh0dtY4MFEZxHoqu+DywBoTjMBf1fE1pS0anhVTgjduS4cxS9Pb582cr0Nq7d6\/dDFB0lgK8OLzsEw3YFxGGFy9eWBu2379\/281BGfCiMS45zjJtyGhLxjkAUPbd2NhYAV5uHIDo169fG2CiOZnh4eHhsHHjRuuoQF\/kaoGXa0hxXTw3c8VdIxzs497Gvnc6U9Cpg8ENDWsB2N3h9etCv9+4OE4O75r4laNNSoFCBQS8+nJIASmwbArg7hI3AGYAKkb2RQMAIfAHGM\/OzlqO88iRI\/bCitu3b1uRGllV+u4CxHR+oPest7CiEAu4pV8sL3PwQipeJ8vngWcAETjjMT3gTU\/aspH34gliDCdPnqy0FUsFXubJvngCgAdwHcqrKVqj5zAt3Wjd5i\/yoNgMx3h8fNz6G2\/bts1AOtulwffMiyfc7fYXT7AmIgVxcZ2\/\/GNoaMj6H3MdDh06FK5du2Y3I0WjzOFFC64lbjkuPNBPizO+Fz7ygJefxXsHmun+wXW9deuWgHfZ\/hfrxFJgdSgg4F0d11G7kAJSQAosSgFiEhQW8qcMZlMmqbYtWco5F3uMHN7FKqjPS4H6VkDAW9\/XT6uXAlJACiQrQJs38rgNDQ3mlHsvXCIFuKXEPLI9cpNPHh0o4F2IavqMFJACy6mAgHc51dW5pYAUkAI1pgCRBSIR+\/bts37GFPOlvnY5dSsOvLRMo6vFrl27AplaevsylgKqU9dCvpd4C8WKzLuQdnWpc+k4KSAFalcBAW\/tXhutTApIASmw5AqQq6aAjzeykavG7aVLBDlgiuqKivmqWYgDr+e2KT4DfFcCeCmQZG66hjAEvNVcSR0rBVaPAgLe1XMttRMpIAWkgBSQAlJACkiBHAX+AygwWSqLYRzsAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:3da50f8a]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"    44"}}
%---
%[output:9479f5e4]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"     3.000000000000000e-02"}}
%---
%[output:2957f2e8]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_JH","value":"     7.500000000000001e-02"}}
%---
%[output:1f6650e6]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber","value":"     6.215564738292011e-09"}}
%---
%[output:42540868]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rsnubber","value":"     3.217728531855956e+03"}}
%---
