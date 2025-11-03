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

% model = 'single_phase_dab';
model = 'three_phase_dab';
use_thermal_model = 1;
%[text] ### Voltage application
application400 = 0;
application690 = 1;
application480 = 0;

% number of modules (electrical drives)
n_modules = 2;
%[text] ### PWM and sampling time and data length storage
fPWM = 12e3;
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
tc = ts_dab/100;

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
% single phase DAB
% Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/8)

% three phase DAB
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/2) %[output:82d0ee5c]

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
figure;  %[output:88456e2e]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:88456e2e]
xlabel('state of charge [p.u.]'); %[output:88456e2e]
ylabel('open circuit voltage [V]'); %[output:88456e2e]
title('open circuit voltage(state of charge)'); %[output:88456e2e]
grid on %[output:88456e2e]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink
heatsink_liquid_2kW; %[output:43e486e9] %[output:222f24c4] %[output:76ecdb95]
%[text] #### SKM1700MB20R4S2I4
danfoss_SKM1700MB20R4S2I4;
% Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2
% Rsnubber = 1/(Csnubber*fPWM_DAB)/5
%[text] ## C-Caller Settings
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
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);
% 

%[text] ## Enable/Disable Subsystems

if use_thermal_model
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
else
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":40.5}
%---
%[output:0994022b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:60f8fa48]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:82d0ee5c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     5.918560606060606e-05"}}
%---
%[output:48d0ef44]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     1.857555033442859e-05"}}
%---
%[output:7efb82ab]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:2a7b0ed3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:1f9a2a65]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000083333333333"],["-8.224670334241132","0.998691003061004"]]}}
%---
%[output:99ce4091]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:89491fc7]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     8.333333333333333e-05"}}
%---
%[output:8f588fa8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"  -8.224670334241132"}}
%---
%[output:11c418e8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.998691003061004"}}
%---
%[output:15b716c3]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.031062519151360"],["1.619345474049183"]]}}
%---
%[output:77afc900]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.125263346003807"],["30.829381225835562"]]}}
%---
%[output:257cd48f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:05e98ae0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:609016e2]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000083333333333"],["-8.224670334241132","0.998691003061004"]]}}
%---
%[output:48d2ea22]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.129590696960579"],["22.638405094998713"]]}}
%---
%[output:88456e2e]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAHgAAABJCAYAAAD2QuLaAAAAAXNSR0IArs4c6QAADM1JREFUeF7tXWuMVVcVXhPQFFpCGToQCRlmeExjGhxeUQQElSCJYcBMSHkkSngJaQM\/5D0QCJT3w+EpUCCEPzyikhSiEShaQmEwWgRihA7hNQJipjMdC3ZQkdFvN+u47r57n\/et586958\/MvffsfdZe33rttc\/eq6ClpaWFxHXp0iWaPHmy883hw4eprKyM5s2bR1VVVdSnTx95e\/7\/hHOgQAc44fTmyQvIgRSAWXuhtYMHDw7Ylfn25uZmWrx4sfpx\/fr11K5dO9q5cyeNHj06NmuwYcMGAu0HDhygwsJC9f+9e\/dowoQJsYxB7+TmzZu0cOFC2rhxo+sYMPbt27fTzJkzFV1uV9w0g8apU6dSCsDHjh2jJUuWpNFRXl7uMC8qx\/CMHTt20MGDBzMCcENDgxrYnDlzMgYwBOrhw4eOwNp4ogue7T4GI06aGcuC0tLSllmzZikpmzZtGl27do3GjBlDw4cPp61bt1LHjh3VYPAZGgiJ3Lt3r6K1oqJCfXf\/\/v0UpkpLAOFgDa6srFT38SUthUnTdQbhs\/5sWAS+b9OmTbRgwQK6evWqegSPa\/r06SnfLVq0SP0uBRr3om+mqbGxkWQ7\/t4EhomuEydOOMrSrVs3JdCXL19O+65z584pzwEdc+fOVTw7efKkonPdunVWYTU9G+PnOKqgpqamBR\/27NlDhw4dopqaGgV2U1MTXbhwgfr27Utt2rShVatW0ZkzZ+jo0aNKm6WmDBgwwBfAEAYM3KbBYDj3j4GBwXAVAESC7fabrsFoV1JSohjEgAIsMBbCBiHl\/hlgKZQ6zeh\/\/vz5jgWSwsx9siZKmtFu7dq1tGXLFgUaxlZcXGxUENmutrZWgWVym248cTSYgyxo0IwZMxTA8C379+9XIA8dOpRu375Nu3btUkyGNLIGsCbqUm3TYC+AuZ\/NmzerrnlgiOIl2Kx9LCjHjx93fLDJRJu0ET4a7ogZ5waUpAvtpIDKWYcOgslES6thsoCjRo1KGSvTPnHixBQt5u9ZAXSesLVI8cGnT5+m2bNnK22+ceMGVVdXE2ZR0CoTk+MGmM00hAgXB04m0KQvtwE8duxYx9SB+VJobEDJ+3SfCVN59+5dZTo5hmCa2ZyiDQNt0kSAumLFClq5cqXqXndxbA3hFuUF0y0Vy+QqJE+MALtJsclHxg2wlMKioiLHPHtJqw1gN9cRRIMlo92CRNZODkr37dvnCKn8H3GDnFnIGEbXYF3I+LMXTxyAa2trW+CLEGCdPXtWmWjpo\/A9nDakDj6YzRMHBzAR8Nm6v\/Tjz0xJE5ZMSLA0eWF8sGSWDFx0H6z\/5uWDpU+UgHfv3j0FOASkbIUk72CR0IctSI3VByOKhrbU19crc1xQUKD+8oX\/x40b50wJbJGsLSLVmcXSCgBN0SFbirq6urSpmXy2NFmSIawdMJlg4KBBg5Rw4gItGCcHQZmIojlihvBK\/wy3t3v3bqUsuAc8x4WAVad5+fLlKqgNGkVLnqQFWVB5+AUww2tSbjMb2fS97sOCzM8hULj0YDOJ48\/pVKW0CADHbb4pwfObyUoC4AUNDQ0tckJvIirOTFYSBp1LNOS0BucC0CkAm+Z0HOkhELBd8F8XL170zM3mAkOTNkYHYJlkkMGDV2Kdg5WBAwemANyzZ8+kjTXr6fln8VD6dMA0avzxt3yPxQHYFkW7RdcQijVr1lCvXr3oypUraQAjxZkN1507d6i0tDTxpB753SN688j1cABjdNBWmYZj7eSEvM4BmGZcPXr0oCNHjqQBjMRJNlyYmyNJkfTrgwdP6YfHH4UHGAPU14RtUweAj9WnpUuXqsm7CeC8BscrMv1W11Bd49NoAPslyfRyADR927Ztqgv44DzAfrlpvg9gtv9iG5p26I\/0\/q0m56ZQPjgKKUjJ5TU4CgdJaeb2X9dR7V\/\/ngKm3mu7P\/2cHvxqp++HpQRZchHabVqk954H2K6BxYUv0Mlr9XTmegPd\/aiZ6j5+qsAMcqGPiq8U0Vtjewe2jJ7zYH0d0i9hrclEAxAwGde1+0\/op5cf0dU\/P1afwwDmxkM8p7jTC7TgOyXUo3M757ncJihfPTNZ0E5E1\/zGYjYCLAFq\/tdzeq+2kd778GO6\/pcnaji3659Q27ZtA2uWX17wfSwkALCivIhmDuuunsnf++kvEsCmTJZc\/vJDQFhJc+ubmfCPZ8\/pt3f+Ru\/eaKQrdZ84TeLWIj\/jlKAAsF5d2tO48i70zbJOjqAEAc7PM8MEr2k+GJ0E1VYTcX4kDcBVv3uPbtV\/6hpY+B180PsYgGfPnlHPopdU86+WdqTXB3alsq4vBtauoM8Pc78fvsp+PU10GCLcJO1B01Pqu6ombLcp7aTJww\/wWV\/+0otKk\/79vMUxfV5mMFsyWZE0OBaOi050SQOTx\/7kD0Y\/x4EFtOa7fV+hb79a6GiPFzhx0J0HOAQXJcAACVkYGWyceKN\/oOAiBAm+m+QB9s2q\/90oAeYUG369tOhrVNa1fYgeM9ckJwAOs5rkxnIG+K1f3Kbqs\/fUrbdWD6NO7b+QOaRC9tyqAfbzyo5t0V++06QvSjDAhT\/6jWI7\/OyVZV8PCUFmm7VqgHnrStC3KmV6Estt+nZKALz\/l7+n8W9\/thEsSII8s3Cm996qAYYG43VZvPyNTVW8M0+yweulOywd8sYqfuUWADd970DitRcEtmqAo+7wZzNtMtEM8M9mlaupT1KvPMAeyHCKc8iQIc4OuJLXBtEnozeplh\/MKUkqtoqubHmjA7SOHDky0Dp7WqoyjInGg\/W3\/bv8YA89e+XVRAdXLHU5rcEAbsSIEWlndiDIOnfunNq+wTvd8D+f7cHR8zd6d6J33uiXaA3OaYDdomu3aRIDjIzVsN4v5wGOiQOxLzaEXQ\/OAxwTolo3oQHWjzmQ\/YY5VokBTvL8N++DIwghAE5y9koOLWd8sL4tUp56E3TPMAAe1utlOvFm\/whi8vk0zQmAeS47adKklIg57MYyALz\/+69RZf8unw9KEZ6SEwDHvZoEgF96fyO1\/ejDCKzPNzVxIMiGgrSjDOUZUKaTXPIszy4OpL2TJU+5wVD8HmuQXcPOHWoz9tJd7rAw2SP13B+cbPLz1HlxIEWD5cGdXg1tv8uzoZJm3uWL\/bY1bt1Fhd26E5Z\/Xu1ssx1bu9hWk\/AABGV89D8+6y8BeBGf6d8x5cNZk3y6LJ9CK59r2kiXabr89i+zjX6zi7H6YJm35tPb9Hm138HEfZ++Zm0DUgpB3DRE6Y9Pjx8\/frx680au3Ln1q87JivrKDj9AMg3f4cBN+RJAlAFGbaubNtMiir43K+y+rKi0urU3Lc26Ahz1lR2bectGgHVGhV1JSyzAUXPRrcFES3CSeGRhaA2OIxed7UEWHws1ZcoUVTAkbB4+kRocVy7a7Yj7TA7cT9\/Sx8qX+fk4KNR1kNOkVueD9SN187loP2KT7Hvyuehk4xOZuljnwZGpyXcQOwdiBdhvkiCOlChzQvpVP2nFoKm+2Dn+35PiJc1xp3M5BuIYI+sBDrppLkkAZyrLJ4+ZDAWwKdLkSiKQeNYkedyhqbwbS6\/b+9UmTcV3XPaWT6u3Rbyyb7ThIiH9+vVTh83oxUFMNHM1FRQKO3\/+vHo2Li4fh\/HKWoZe4zFl1XBSYIcOHVTftoUQafncBDUSwCbieIeDNNF6FkhubdHLzfFh4lyRBZXP9Oqn8txqbK\/h8nIo72OrbSzp4R2Qq1evVpXdcKEolewLQirPBGOauewOV32TyQZTCR6v8Zh4CGGRAg\/69KIfkQHWi0Oz9kiJcpMcNx9sWsnhymQyX23yz\/oz5cIBaiOZALbRqS86yOSMXseJaWaAmU59oYI\/czmcoOPRlcG0HRdYRAI4SApMLltJ8CWIpkPV2HQzobL0nAxm9GDJRBv3YQPYNh4deAkwm2JZpk5WA2WfqWe4dIBle+mydHfD\/ekCkzGAw9RNksShsiivt+pMcNNgr2AjExrMz5QAoxycrD2ha7ANEB3goOPRNdi2yCE12E0hrT7YzzRHN2lyl6FsLwEGQHrpO1nylZnK9+mVNtk8cSDj1wdLv88LB6gohsqqXgBLmtlEyzYYD3ykmw+2jcfmg2VRS5sPljywlZ01Auy2N0mP6qSvlr\/x93phZtyD+oanTp1SgQ0XTzZF0ba5rG7yZSFnW5BlasOAmACWxZolzexbpWbyWBF4AWxZs8JWgs\/NRHPhawSsMk9u0louAP348WMlqKgMK3kQKYrOxMQ\/2\/sMWurONk2C8Ac5p9vGNyvA+moLTs5ZtmwZVVVVqeWz\/PUZB\/QX8\/zUlpK80zNZpqImYXltzWTxQ2FyKisrnYIbMKf5oldh2f3\/b\/cf5nBks4N4MxwAAAAASUVORK5CYII=","height":316,"width":524}}
%---
%[output:43e486e9]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:222f24c4]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:76ecdb95]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
