%[text] ## dabGeneral Settings
%[text] ### Simulink model initialization
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 1.5;
transmission_delay = 125e-6*2;
s=tf('s');

%[text] ### Voltage application
application400 = 0;
application690 = 1;
application480 = 0;

% number of modules (electrical drives)
n_modules = 2;
%[text] ### PWM and sampling time and data length storage
fPWM = 5e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*2; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

TRGO_double_update = 0;
if TRGO_double_update
    ts_afe = 1/fPWM_AFE/2;
    ts_inv = 1/fPWM_INV/2;
    ts_dab = 1/fPWM_DAB/2;
else
    ts_afe = 1/fPWM_AFE;
    ts_inv = 1/fPWM_INV;
    ts_dab = 1/fPWM_DAB;
end
ts_battery = ts_dab;
tc = ts_dab/200;

z_dab=tf('z',ts_dab);
z_inv=tf('z',ts_inv);
z_afe=tf('z',ts_afe);

t_misura = simlength - 0.2;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

Pnom = 275e3;
ubattery = 1250;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 1500;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:0994022b]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:60f8fa48]
%[text] ### dead\_time and delays
dead_time_DAB = 0;
dead_time_AFE = 0;
dead_time_INV = 0;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;
%[text] ### Grid emulator initialization
grid_emulator;
%[text] ### Nominal DClink voltage seting as function of the voltage application
if (application690 == 1)
    Vdc_bez = 1070; % DClink voltage reference
elseif (application480 == 1)
    Vdc_bez = 750; % DClink voltage reference
else
    Vdc_bez = 660; % DClink voltage reference
end
%[text] ### ADC quantizations
adc12_quantization = 1/2^11;
adc16_quantization = 1/2^15;
%[text] ## HW design and settings
%[text] ### DAB
%[text] #### Input filter or inductance at stage 1
LFi_dc = 400e-6;
RLFi_dc = 5e-3;
%[text] #### DClink input stage or capacitor at stage 1
Vdc_ref = Vdc_bez; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi_dc1 = 900e-6*5;
RCFi_dc1_internal = 1e-3;
CFi_dc2 = 900e-6*5;
RCFi_dc2_internal = 1e-3;
%[text] #### Tank LC and HF-Transformer parameters
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/4) %[output:82d0ee5c]
f0 = fPWM_DAB/5;
Cs = 1/Ls/(2*pi*f0)^2 %[output:48d0ef44]

m1 = 12;
m2 = 12;
m12 = m1/m2;

Ls1 = Ls/2;
Cs1 = Cs*2;
Cs2 = Cs1/m12^2;
Ls2 = Ls1*m12^2;

lm_trafo = 5e-3;
rfe_trafo = 1e3;
rd1_trafo = 5e-3;
ld1_trafo = Ls1;
rd2_trafo = rd1_trafo/m12^2;
ld2_trafo = ld1_trafo/m12^2;
%[text] #### DClink Lstray model
Lstray_module = 100e-9;
RLstray_dclink = 10e-3;
C_HF_Lstray_dclink = 15e-6;
R_HF_Lstray_dclink = 22000;
Z_HF_Lstray_dclink = 1/s/C_HF_Lstray_dclink + R_HF_Lstray_dclink;
Z_LF_Lstray_dclink = s*Lstray_module + RLstray_dclink;
Z_Lstray_dclink = Z_HF_Lstray_dclink*Z_LF_Lstray_dclink/(Z_HF_Lstray_dclink+Z_LF_Lstray_dclink);
ZCFi = 7/s/CFi_dc1;
sys_dclink = minreal(ZCFi/(ZCFi+Z_Lstray_dclink));
% figure; bode(sys_dclink,Z_Lstray_dclink,options); grid on
%[text] ## Active Front End (AFE)
%[text] ### LCL switching filter
if (application690 == 1)
    LFu1_AFE = 0.5e-3;
    RLFu1_AFE = 157*0.05*LFu1_AFE;
    LFu1_AFE_0 = LFu1_AFE;
    RLFu1_AFE_0 = RLFu1_AFE/3;
    CFu_AFE = (100e-6*2);
    RCFu_AFE = (50e-3);
else
    LFu1_AFE = 0.33e-3;
    RLFu1_AFE = 157*0.05*LFu1_AFE;
    LFu1_AFE_0 = LFu1_AFE;
    RLFu1_AFE_0 = RLFu1_AFE/3;
    CFu_AFE = (185e-6*2);
    RCFu_AFE = (50e-3);
end
%%
%[text] ## Control system design and settings
%[text] ### Single phase inverter control
flt_dq = 2/(s/(2*pi*50)+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d); grid on
% [num50 den50]=tfdata(flt_dq_d,'v');
iph_grid_pu_ref = 2.5;
%[text] ### DAB Control parameters
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 1;
ki_v_dab = 45;
%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:7efb82ab]
Iac_FS = I_phase_normalization_factor %[output:2a7b0ed3]

kp_afe = 0.15;
ki_afe = 18;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);
res_min = s/(s^2 + 2*delta*omega_grid_min*s + (omega_grid_min)^2);
res_max = s/(s^2 + 2*delta*omega_grid_max*s + (omega_grid_max)^2);

Ares_nom = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Aresd_nom = eye(2) + Ares_nom*ts_afe %[output:1f9a2a65]
a11d = 1 %[output:99ce4091]
a12d = ts_inv %[output:89491fc7]
a21d = -omega_grid_nom^2*ts_inv %[output:8f588fa8]
a22d = 1 -2*delta*omega_grid_nom*ts_inv %[output:11c418e8]

Bres = [0; 1];
Cres = [0 1];

Ares_min = [0 1; -omega_grid_min^2 -2*delta*omega_grid_min];
Ares_max = [0 1; -omega_grid_max^2 -2*delta*omega_grid_max];

Aresd_min = eye(2) + Ares_min*ts_afe;
Aresd_max = eye(2) + Ares_max*ts_afe;
Bresd = Bres*ts_afe;
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:15b716c3]

freq_filter = 50;
tau_f = 1/2/pi/freq_filter;
Hs = 1/(s*tau_f+1);
Hd = c2d(Hs,ts_inv);
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso_pll = [-1 -4]*omega_rso;
Lrso_pll = acker(Arso',Crso',polesrso_pll)';
Adrso_pll = eye(2) + Arso*ts_afe;
polesdrso_pll = exp(ts_afe*polesrso_pll);
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:77afc900]

%[text] ### PLL DDSRF
use_advanced_pll = 0;
use_dq_pll_ccaller = 0;
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for voltager grid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:257cd48f]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:05e98ae0]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:609016e2]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:48d2ea22]
%%
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
%%
%[text] #### RMS filter
rms_perios = 1;
n1 = rms_perios/f_grid/ts_afe;
rms_perios = 10;
n10 = rms_perios/f_grid/ts_afe;
%%
%[text] #### Butterworth filter
omega_c = 2*pi*5;
P1 = s^2 + 0.7654*omega_c*s + omega_c^2;
P2 = s^2 + 1.8478*omega_c*s + omega_c^2; 
Hb_flt = omega_c^4/P1/P2; 
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
%[text] ### 
%[text] ## Lithium Ion Battery
typical_cell_voltage = 3.6;
number_of_cells = floor(ubattery/typical_cell_voltage)-1; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 50; %Hr*A

Vbattery_nom = ubattery;
Pbattery_nom = Pnom;
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
figure;  %[output:0b251a5d]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:0b251a5d]
xlabel('state of charge [p.u.]'); %[output:0b251a5d]
ylabel('open circuit voltage [V]'); %[output:0b251a5d]
title('open circuit voltage(state of charge)'); %[output:0b251a5d]
grid on %[output:0b251a5d]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink
weigth = 0.50; % kg
cp_al = 880; % J/K/kg
heat_capacity = cp_al*weigth % J/K %[output:43e486e9]
thermal_conducibility = 204; % W/(m K)
Rth_mosfet_HA = 30/1000 % K/W %[output:222f24c4]
Tambient = 40;
DThs_init = 0;
%[text] #### SKM1700MB20R4S2I4
Ron = 1.04e-3; % Rds [Ohm]
Vdon_diode = 4; % V
Vgamma = 4; % V
Rdon_diode = 1.85e-3; % V
Eon = 77e-3; % J @ Tj = 125°C
Eoff = 108e-3; % J @ Tj = 125°C
Eerr = 9.7e-3; % J @ Tj = 125°C
Voff_sw_losses = 1500; % V
Ion_sw_losses = 2000; % A

JunctionTermalMass = 5; % J/K
Rtim = 0.05;
Rth_mosfet_JC = 19/1000; % K/W
Rth_mosfet_CH = 6/1000; % K/W
Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH % K/W %[output:76ecdb95]
Lstray_module = 12e-9;

Irr = 475;
Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2 %[output:83a11c14]
Rsnubber = 1/(Csnubber*fPWM_DAB)/5 %[output:4ade6871]
%[text] #### FF1000UXTR23T2M1

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
model = 'single_phase_dab';
open_system(model);
% Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
%[text] ## 
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
% 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":34.1}
%---
%[output:0994022b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:60f8fa48]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:82d0ee5c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     7.102272727272727e-05"}}
%---
%[output:48d0ef44]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     8.916264160525726e-05"}}
%---
%[output:7efb82ab]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:2a7b0ed3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:1f9a2a65]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000200000000000"],["-19.739208802178720","0.996858407346410"]]}}
%---
%[output:99ce4091]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:89491fc7]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     2.000000000000000e-04"}}
%---
%[output:8f588fa8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":" -19.739208802178720"}}
%---
%[output:11c418e8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.996858407346410"}}
%---
%[output:15b716c3]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.073386376052051"],["3.802432508328568"]]}}
%---
%[output:77afc900]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.283130953403918"],["67.668222262819981"]]}}
%---
%[output:257cd48f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:05e98ae0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:609016e2]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000200000000000"],["-19.739208802178720","0.996858407346410"]]}}
%---
%[output:48d2ea22]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.311017672705390"],["54.332172227996914"]]}}
%---
%[output:0b251a5d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAQcAAACeCAYAAAA\/tc+YAAAAAXNSR0IArs4c6QAAHBNJREFUeF7tXX+QV9V1P3ZS60IhukBUiuuC2W1tYjWSH4q2i2MVOwnYYZIgdCYM7FAkItMJDLCARToBgbLOBLAN41Bmk5ZdZyOp8BdBKlS7ITWgZKJ2wC4LEvzFrooiTuq4nfPwfHP2fu977773ve\/eu\/s9b8ZR93t\/nPs593zuuef+uqS\/v78f5BMEBAFBQEHgEiEH6ROCgCCgQ0DIQfqFICAIaBEQcojpGIcOHYJZs2bBzp074ZZbbrHafdSyL1y4AJs3b4Z58+ZBbW2tlbqwzOXLl0dlrV+\/HmpqagDrPXnyJMyYMcNKHXGFPPHEE9DV1VWqN6myrVu3wpQpU6ChoSFVpixpUwv7NAHhtGfPHhg7dizs2LHDSBYdvqZ12kzX19cHzc3NsGzZMuv9VMjBpqZylrVhw4bIcLdv314YOZw+fRrmzJkDDz74YKHkQJ31vvvuS60HSWTLli1GBpklbRY1HD9+PMJl6tSpkYGZfqGQA8qL\/efMmTNGZGzaPkznlRxIMdgw\/GiUJuDPnTsX\/f3gwYNlrE6jL\/5+4403lgyL\/v7QQw9Ff8OyH3nkkdiOGicDH92xfByFSR7MgyPMuHHjor\/jqIPf\/Pnzow5GZZIhqp4C\/38cyVtaWqL86sil64AqkaRhiOUuXboUFi5cCEePHh0gJxpcXN1Yz7Zt2yKZJk+eDAcOHCgZMR9tsUCOr2rERBZUN6Xl+iPdNzY2RqOgKqcuLXpzXH40bvKQVAOIk0H9u64Mta2kY10f5f2QjBYxjOujWBb+TmUmYa7Kyj3aorxcb+SgGhDvVGR0hw8fjjrkqFGjok5TV1cXdQA+Ck6bNm2A+4wdC6cDHPC4UVkd5bjhHTt2rDStIHIgecgF5oxN9aLSUF4+SieRA3byJM8Bceno6IiIDj\/EAfPoSEiHIeZRMcNpBeK\/bt06aG1tLZVL+FJb0JAJX972OJyoLXwU42n37ds3wFNQiQTT1tfXR0ROhk9GoKblmBKpEC6cHEjH9JuqizTPIU7Hap\/AOlWdt7e3D8CevBOSgfoo5qW\/6TAneyBd7t69ewCOWby1QeE5xI0wqMRFixaVzZd5+iNHjpR1MjIgMmoaoZLcUVTIkiVLtG6tznMg5eD8PUkhWTyHNHKgsjZt2hTplcdBsmAYN63Qjb4Y\/0BviObfvB4iajI2joNK1IgTjYbqqIpt0ekmboTUEQkn\/TjXWhff4bEYwkU3rUjSMXkOp06d0hI3GSG1n3uWupEe08VhrhIP7xOoB5UAsxBAUlpvnoM6WvIGppHDrl27IneMf+SS9\/b2JhoQz5NGHGSINEpwclAJgJdrkxyoE2L7aISh2EQWDFVyoA6KRrF69WpYs2ZNVD56GUgO3PA4TtRRaSpI7cZREAOq3MNDclCnPZwkdJ4OGgh6C0lEqE7nSAYTAlKnaknkkKRjtRz8f+7VEelyXOK8F5Rf1SXHhvq0asg0AJLtkOeHuNv4vJFDllEPG5vkOXAgVFY2JQB1RcLUc9C5sjbJgY+wY8aMKU0pdCNvEsGq5MA7I+LLR9MsngPHPilIx+fu5E7rSCcuTpPmOcQZgw3PQafjJHJQBzeVOCr1HNS2DjnPwSTmQKMIzSmzxBzi5qocWBVUHVtjOTrPQWV7ZHeac951110DRhFyLUkmtXOkrVbw0ZcHokwwJG9AJQddWykgx2MO1Ja33367NM1IizmQ16GSTpIMfLpCxkX6p+AjX9lwGXOg9nAdq1MolQDUWAsGfokUdeTAYw4q5l5jDuo8L8kl0blueV0Y3ul5pJ6z8ogRIyI3U603bbXChBxQ7iyrFXxagf8dF8mmUZ1WAigqHUcOvC26fRXq\/JbvhTDBEKcK+NHKCpIAX8FA7NErwY9PWYpYreArAlx2dJHxI8xQ30hI5EmoaXnQEvNlWa3QEWzcUmbaagX1CZUcVL0gvmrAV9V1kKsVSA4458S5Z9ImnLR0CAh2uo0bN5ZtJFFBTiKZkNaQ8xLfYM6X1bPReWNFbMoZzJimyV4J5ugFmm46S5OD\/24t5kAGrS73UWVILIsXL4YVK1ak7kATcsiiQvtpVSLHGrLsFC2qs9pvaTgl5sW88B2SVAF3K7PChh0C3UAkBx0B8HV1W1uEs8oo6QUBQcAcgZLnoDJXln3maPhtbW2wYMECWLVqlZYc+Ho6ipe0a9FcfEkpCAgCRSEQO60wNWYklbVr18Ls2bOjnYEmU4ciXaGigJJyBYFqQ8Ao5pAUiFSj\/QhgmtdBXsqkSZMGnHmYMGFCteEv7RUEMiOwf\/9+GD9+fOZ8WTNoyUFd2kwzdpOgI3oi+NG+ft2qBpJDd3d31jZI+gIREJ0UCG7Ool3pJCKHvGSgtk1dkeCEkHSSj8px1eicOqnKbKKT8NTuSicROaDh4j++VxFcNTo8dYcr0YkTJ5y4sOEiEJ5kruyk5DnY2ARVKYyuGl2pnNWUX8ghPG27shPttCIJDpvbp9V6XDU6PHWHK5GQQ3i6cWUnRqsVruBx1WhX7RkK9Qg5hKdFV3Yi5BCe7oOSSMghKHVEwgg5hKeTqpRIyCE8tQs5hKeTqpRIyCE8tQs5hKeTqpRIyCE8tQs5hKeTqpRIyCE8tXsjB76TkW4MijtpaRs2V422LfdQLk\/IITzturKTAasV\/Kbj6dOnR8ewV65cCXhPfhE3zcg+h\/A6niqRkEN4OvJCDvz0JV6mSeSApGGyg7JSGF01ulI5qym\/kEN42nZlJ2X7HOhxkLlz50JnZ2d0gQs+paa7nts2bK4abVvuoVyekEN42nVlJ9pNUOrbhK5ubXLV6PDUHa5EQg7h6caVncgOyfB0H5REQg5BqSMSRsghPJ1UpURCDuGp3Qs5pD1uY3ojVF44XTU6r3zVmE\/IITytu7KTsmkF3t7U09MTPfFOH\/2tqakJ6FlxW491cuhdNTo8dYcrkZBDWLp57tV3Yfo\/dMBbP7q\/cMEGkEPcRbL0d3zDEB9ZTXsZK6\/UQg55kSsun5BDcdjmKbn9+TfggfZXoO\/RO\/Jkz5RHuwmKv1pFt0tPnDgR7r33XnjqqaeiZ9or8RxwuRQ\/7p24DLRkQqjKEws5hNUBvJEDwaAuZeJTaPjStcmbFGlQUtn04rBMK9IQ8\/u7kINf\/NXaN+ztgQ17T7j3HIqGgaYnDQ0NcP78efEcigbcQvlCDhZAtFjEkCUHnE5gUPPkyZNlQU+ZVljsQRaLEnKwCKaFojDegFML5zEHlF19Bo\/aU+nFsjidOHjwYOQt6FZEhBws9JwCihByKADUCoqc9tgL8Nz\/vuueHPijNLt27YpGeTxTgSN+fX39gKfrsrYPy9i2bduAbGrcAVcr8MPnvuQLA4HTp0\/DuHHjwhCmyqW488474dzdG+CTYaP9kAOdvty3b1\/J9U96KzOPvsRzyIOanzziOfjBPa7W2u89E\/3kfFrBH7i9+eabgd6zPHLkCHR0dMD27dutvIol5BBWh0uSRsghHF09duA1eGj3q37IIWKkvr7S3Q3oPbS0tETC4HImTjGK\/GQTVJHo5itbyCEfbrZz4c7Iaf\/0QlTs7314Fs7+8Fu2qygrT05lFg7x4K5AyMG\/\/jgxoDQjf7YMen7934ULlmn7dFHbpqmV4jkUru\/MFQg5ZIbMWoZTfR\/B0\/\/TC0t+cqxU5ourboXJX\/5T6O7utlZPXEHGb2XiZbOVbptOa42QQxpC7n8XcnCPOdb49Cu98O3HfzWg8t3f\/RLc\/vnL\/dznYHtVIiusQg5ZESs+vZBD8RhTDegpLGx\/JdrHwL+62ssAiQH\/jZ8rO5GYgzvdD8qahByKUxuSQffZC\/Dovp4yQsBakQy23nd95C3wzyk5pF3ygoJVukPSBGJXjTaRRdJcREDIwV5PQDL411+8Doe639WSARHCbdddDsumjC95CqoEruxEPAd7uh+SJQk5ZFcrkgB+G3\/WA6d6L8QSAZWMHsI3\/mwM\/O3t42IJwZvnkL35xeRwxYjFSD80SxVyiNcrkUDrvh44cTadBDgZ3P75K2Dp3fVGZBCM58CfwyOhXKxUuAy0DE0zLqZV1U4OSAD9APCD\/Sfh1bc+TPUCVC2gV0BEQNOGSjXlahCNfQ6P39JED93IUmalah18+Yc6OUTG398Pm595DY6\/eR5OvfMRkEeQRVtIAnVXXAYLmq6BL4z9w1wegWl9Xsgh7Q5J2QRlqr6hk24wkwMZ+b8ffQuefrk3Uoq6TJhFU0QAf9F4BXx74lVRVlpezFJOpWm9kAMKjV7Cnj17YMeOHYA3NtEdkji1UO98rLSRvuZStuUeyuWFTA7P97wHbYdej4J+eUd8rjsydJwGNN\/2RzBq+O97Mf60\/uSNHFAw9cIXeQ4vTV1D93eX5EAj\/S9OvAcHjvXBa30fWTF6PsKj6\/8nVw+HhZPrgjR8k57klRxMBCwijatGFyH7UC2zUnIgg+8++yE8\/Uof\/Or0+xW79yrWNOKj4X9r4lXQ1HiFN5ffRT9wZSeyz8GFNgdxHSo58GDdfx5\/Bw6deM+aW59k9H95\/Si496bPwSWfJvIx1w9FjV7IgXZK1tXVFX7ISge0q0aHouQQ5CBjR2M7ePwdePLIm9Bz9kIkmo15fJLBTxgzDL5585Ul976aDT5LX3BlJ2Weg26fg+6NiSyNMU3rqtGm8gzGdGjsaGSf9PfDjw69Dhi0szl312HC3fqGK4fBfV++Cq4c+QdD2rX32Tdc2YnRtAJvjsZVDFvXxMUB66rRPhWbpW4ydMzT1f0uPHnkrWgtvqhRnWTjxn7FpR\/D1xqvgm\/cMGbQBvCyYD4Y0rqyEyPPIe11bf5CVtxuStUj0R3kctVonx0ADf7tD34LPznyJrz0mw8KN3SsgBv7NbWXwdfGfxa+c8vYi3V\/6mnEYVJpQNIn1kO1bld2UnYTVHNzc4SpqZfAr7PHK8yXL18OkyZNKrvGnqfD\/RO6z1WjbXUaNKze8\/8HnYffgF97MHRsR8OVw+G7TdfAdWNqSjv7bM7dhRxs9RZ75biyE6NphWmzyDuYOXNm2WW0uJlq3bp10NraGnuDtatGJ7UHDb7jl2\/Ac8ffKXxU50aMy3DXjqqJAnS4FMcDhab4F5FOyKEIVCsr05WdWCMHmlrETStMNla5ajRe8b33pbMVbaVV1asaet2oGph9y9XwlfrPBmPoebqkkEMe1IrN48pOrJEDwYEk0NXVlbgUSkumuB2bX3dfRKPjrt5KUx+fp9eProGJ146EOxprS9lsuu5psvj8XcjBJ\/r6uouwE11N1snBZGWDP54zY8aMklzYaPwqeQ7vzLmPozIefvosHP7NxUs3dN\/YkZ+Bq0d8BhpHXwqzbhoZJcG\/yTcQAXkOL5wegc\/h0efs9mmqMM+pTDUPLnnipx7SQo8CPyQDjD\/Qa1o8OFkJI9IcHR\/+UI\/c0mm6rTOvl+W4jH1dPIeMgDlIXomdZBHPytX0cUuZnBDUpUzdYa68jdZNHZAQvn7DGJj\/52ZXb2UBrZrSCjmEp+28dpK1JUaP2mQtNG\/6PI1WXwNCUth1\/00wYXRNXjEkH0NAyCG87pDHTvK0wnrMIY8QlCdro3XEwO\/3r0QWyXsRASGH8HpCVjvJ24LStGLNmjWwaNEiWLJkCRw9erSsvNCupsepxE3f\/3lJTnoNKC8Qkk+PgJBDeD3DKTmE0nzTRgsxuNOYkIM7rE1rMrUT0\/Li0g26aYUafBSPodIukJxfyKFYfPOU7oUckl6+CmVaweMMt193Oex+4Et58JU8hggIORgC5TCZF3KIax\/uXWhqaio7L2EbD5NG137vmaha9XFR27JIeRKQDLUPmNiJDdmNphWuXt9OazSeiXho96tRu19cdatsaLLRA1LKEM\/BAcgZq0izk4zFxSY3IgeTLdE2BEpqNA9CoteA5CBf8QgIORSPcdYavJBDUsxh586dXqcVP33hLWj+8UsRjhKEzNqd8qcXcsiPXVE5vZBDUY0xLTep0TzWIF6DKaKVpxNyqBxD2yV4Iwf1UBSej+jo6DC+GaoSIOIa3f78G\/BA+ytR0Y\/NvB5mfuXiU2TyFY+AkEPxGGetwQs5xN3kZHJHQ9YG6tLHNXraYy9EF7NIrMEGytnKEHLIhpeL1F7IIc+RbZtg6BrNA5Gyr8Em2mZlCTmY4eQylRdywAail7Bly5bSQ7oUpMQbm3w8pMunFLJ86bILXqxLyME95mk1eiMHFIxe1j5z5kwkp8+HdGVKkdZViv1dyKFYfPOU7pUc8ghsI4+u0bRKIVMKGwhnL0PIITtmRefwQg6udkLGgac2mscbZG9D0V1OX76Qgx\/ck2r1Qg4oEJ6jqK+vL3uUxgVEaqNpSoF1S7zBhQbK6xBy8IN7cORQ5KlM9Q5J3Y5LlRzwMhd6rk02PvnppEIOfnAPjhyKhAFXQXp6eqIVj7izGio5ULzh7+68Fv7+6xevrZfPLQJCDm7xNqnN27TCRLhK0yA5tLe3lz18wxvN4w0ypagU8fz5hRzyY1dUTqfkQIHIou+Q5FOLtGmFxBuK6lrZyhVyyIaXi9ROycFFg3gdJs\/hyf4G11rR1yfkEIYeuBTeyMHFwau05\/B+\/NO9MLXtdITHrXU1sPXeK8PTUJVIJM\/hhaNor8\/hFXnwKstzePyeSDmF6bdziufgF39d7V48hyIPXmVZyuTnKWTzk9\/OKeTgF\/9gyAEFCeHgFQ9G9j16R3jaqSKJhBzCU7YXz4Fg8H3wSjY\/hdMhhRzC0QVJ4pUcfMGBjT7wy5dLz9zJYStfmvhdvUIO\/nWgSiDkAADLpoyHZVPqw9NOFUkk5BCesquWHNY+0VW6L1KCkf47ppCDfx2I5wAAyIjz\/\/k\/YMPeExEesm3af8cUcvCvAyGHT8nhi4uflMtkA+qPQg4BKeNTUbxNK\/BQ1KxZs8oQcfWQ7si5\/ybHtAPqj0IOASnDJznEnXlwBQ8y4rt\/vT2qTlYqXKGeXI+QQxh64FJ48Rx8XxNX\/8Wvwrm7N0Q4yEpFGJ1SyCEMPXgnBxSAX8riGpa6r\/4VfHD7UglGugY+oT4hh4CUEcK04ujRo15iDpwcZBkzjE4p5BCGHoLwHHxCcdU3vw+\/rbstEkHOVPjUxO\/qFnIIQw9VTw6f+84P4ePRfyxvYgbUH4UcAlKGz2kF1s2PVk+dOhWWLl0Kq1atghUrVkBDQ0OhSI2+vxM+GTZaVioKRTlb4UIO2fBykdrLagURw9ixY2H69OnQ1tYGK1euhN27d0NXV1fZhbC2gZDXrWwjWnl5Qg6VY2i7BC\/kwJcye3t7S+SApLFmzRpYvXo11NbW2m5rqTwih53NN8A9XxhdWD1SsDkCQg7mWLlK6YUcsHH44hU+oDt37lzo7OyEBQsWwMKFC8HFK9tEDrLHwVU3S69HyCEdI9cpvJEDNlTdQu3qlW0iB1nGdN3d4usTcghHFySJV3LwBYeQgy\/khRzCQz5eokFFDrirsqWlJWpN3AEt9YJZXToiB9njEE5XFc8hHF149xxUI0aBcElz\/fr1UFNTU4YU3je5bt06aG1tjYKVFLNQ02Owc\/HixYlLokgOdbWXRfc4yBcGAkIOYeiBS+HFc+BLmfjgLX70N\/zvOILggse9g6mSiA5yIYfwOqKQQ3g68UIONt6tQM+hvr4eZsyYMQBVPvXAH3RBTiQHOaodVmcUcghLHyiNF3LAinUjf5zBq7CZnuiMuzcCyeHSU\/8FP\/\/HvwlPI1UqkTyHF47ivT6HR0arO5VJEMUFHE0JhE9VJk2aNMDDQHKQPQ7hdEaXo1RYrQ5bGm+eQx5YkBiampqijVJxn8lbmUgOw478S+Q9yCcICALxCHR3dxcOzyX9\/f39ldSiu3OSVjfwTAZ+GH9QV0FcbayqpG2SVxCoZgTKyCHrUmY1gydtFwSGMgIDyEG3lImNj9u7MJSBkbYJAtWOwABysLGUWe2ASvsFgaGCQNm0Ar2EPXv2wI4dO6LLXejFbYwj0MYo243neyB27tyZGNi0XbeUdxEBHjuKiwepr6\/Pnz+\/sD4heolHwGS3sQ38tAFJkw1LNirHMvjOyWPHjkF7e7vRTkxb9Us5ALyzIR58OzzHJ273q2DoDgEiaKyRBvCiaq94taJSwZCI6JYpjHmknb+otD7JX44AGj16jNu3b4\/OzyxfvhxmzpxZ5sGZbnITjItBAEn88ccfh3vuuQcefvhh2LhxY6FXNwZBDj09PZF76vvFrWJUGn6p3CNAaZEc1A1q6ioWXiVY9MgVPnJ+JETvAe92FXLwg39V1WpCDiog3Nso8urAqlKEYWOrihxkWmHYKwpKZjqt4NW76qAFNXlQF+sKe+\/TCglI+u+nJgFJnFasXbsWZs+eHc1zeaxId8+H\/1YNXQmqhhxQhbQ6IvNYfx2aL2Xy5WT1TMycOXOiC4hFV\/50VVXk4A9mqVkQEATiEPA+rRDVCAKCQJgICDmEqReRShDwjoCQg3cViACCQJgICDmEqReRShDwjoCQg3cViACCQJgICDkUqBfTswgm1\/ZnFZOfoDQ96RrSwSpaWo27szQrHmp6Kj\/pTZZK6xjs+YUcCtSgT3LAzn\/w4MFMR6pDI4eiT+gigba1tcHKlSu1DzYV2DUGRdFCDhbUxDcQ0UiHx89nzZoVlU73Hqj3beKI3tjYCM3NzYA3flNeOhmJ92rglzTy8zJpFMSyqO64kZEfy6f7G4gcRowYEdWpjto8D98EhSc6X375ZXj22WdL75HwjW2TJ08GLJPuA9HJrO6yVIkK6xg+fDjs378\/wiruLgnVC0siPCGH5M4v5FAhOagXb\/AdhdxzUHe18e3H+DaE+qQgioXGhJ17yZIl2hOQNHXYtGlTZMh4mhKNlvLFjbzcYPgx+d7e3ohUiBjU8uhYNz17SDKq1wjyto4aNSoiP7yZHOXiv40bN26AzFwVOnKgS4ioTCxPvfFcyKHCDs2yCzlUiGXSrTxJ0wre+Tk5oDhoTNTx6ai07n4F1YD4Aaqki3Pi3hhRT1omyc9\/w\/KIKPDfaj7+\/+qZjLiRXUcOSXWQGoUcKuzQQg72AMSSePCPu\/GqkaARbdu2rVQ5pdWRg\/qwkO7qNtXQTA6xEdmo9zWgUKpBcvl1t5KTa6+STRJZqLeMYb26oKOOHPgzi3HEJeRgr2+L52APy6gkdZSki2zUUTnJczC9DasIz4FPRZJGfNVzSDLcOEySoE\/zHFQCivMckk6PSsxBYg6WzX9gcepIFRdz0N2ZgCXhy+VJMQceV9DNr\/GUZNaYg3o1H01jUB4TckAvgscRVM\/BNOaAR7\/jnj3QkQP+Da+yU6deXCO6OAzhrAY9hRyEHAolB\/IWWlpaonr4tIKi8uh+L1q0KAq+YVANg4YrVqyAzs5OaG1tLXV2\/A9+jyOtViS9DhYX+U9bluRTHHW1AgkLDYmP+PwdVZwGzJs3D\/bu3RuR2+bNm8teVqfpA6bFB2DPnz+vXa2I28egI4f3338fDhw4EB0Z55joYhyoD8QZSezFF1\/UkrCQg5BD4eQgFcQjkBTjyDqt4FMXG5gLOQg52OhHUkYGBNT9HHnet6AyyLPAW5dtkgOVLzsk4xUrAckMnV6SCgLVhMD\/AzuGyK2RG6heAAAAAElFTkSuQmCC","height":158,"width":263}}
%---
%[output:43e486e9]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"   440"}}
%---
%[output:222f24c4]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.030000000000000"}}
%---
%[output:76ecdb95]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_JH","value":"   0.075000000000000"}}
%---
%[output:83a11c14]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber","value":"     1.203333333333333e-09"}}
%---
%[output:4ade6871]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rsnubber","value":"     1.662049861495845e+04"}}
%---
