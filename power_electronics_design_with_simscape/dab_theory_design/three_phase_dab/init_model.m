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
% danfoss_SKM1700MB20R4S2I4;
infineon_FF1000UXTR23T2M1;
Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2 %[output:08c57868]
Rsnubber = 1/(Csnubber*fPWM_DAB)/5 %[output:863cd5b9]
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAgwAAAE8CAYAAACsFOk\/AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10HtV55x92s62VEoLFR0AoYD5kQktrY4dWOC5OqlJOmpXJ0iaSvNtSR03cBsJujVeSS8I5BIIl1dBwgGxdjqLSbm1rtwsN2p6WsC5hIcJNMKBtUxoEwjhCuAGMCxTRlNZ7nnGufDWamTsz\/5l55533\/56TQ+T33mfu\/J5n7v2\/z\/2Y444cOXJE+CEBEiABEiABEiCBCALHUTAwPkiABEiABEiABFwEKBhchPg9CZAACZAACZCAUDAwCEiABEiABEiABJwEKBiciFiABEiABEiABEiAgoExQAIkQAIkQAIk4CRAweBExAIkQAIkQAIkQAIUDIyBhiZw6NAh6e3t9RiMjIxIc3NzbjympqZk48aNsnr1ahkcHJSmpqbcrmUbLvIe49yQ4fC5z31Ourq64lQpdZmhoSHZsWOH18ZNmzZJf39\/ovaOjY3J1q1bZdu2baXkofe3d+\/e3J+PRNBYuCYEKBhqgp0XLQuBIgfTMMGgHfK6deukvb09Fyxh95j3dcNuJs0ApAPWww8\/nGgwTlMnqQP0Ghs2bJivVkXBUDWBl9THLH+MAAUDo4EEakRgbm5OBgYGZHx8XHbu3FmYYNDMRhHXDcJqBp\/Ozs7Yg7\/5BZ5kME5TJ00YGMGQpG3+65Q9w2Di9MCBA8wypAmSCtWhYKiQM2t9K3ZqVttip1jtX9crV66Um266yWuuDhx2et78Gp6cnFz0vW3jiiuukF\/\/9V\/3yrS0tMjo6Ki0tbUFIrAHZn95\/69v\/d5MUXR0dMhtt90mK1as8DpKe6B12dGpDXPdffv2ee3Tj5mSuOGGG+SLX\/yiJxbMx89C\/90wtQWFGaTs8mbQMbbsAcy+xzvvvFOGh4cDrzszM+O1b3Z2dr5Ntg9tjsr8rrvukq9+9ati7k\/5+1kbdmaqJ2hwNH61r2vu139fxtetra3zosfP7\/777\/dS\/OZjx4ffnkuo+eMxylZUHEZlImwm2mbTdr8I8T9fNltjY\/PmzbJnzx7R58f4zr5n\/TdzDdu3Li5BcVjr\/obXL54ABUPxzCt3Rf8gYd+g6fSCBgV\/R692dLA2YsH\/fdCAFjXY6ndhbTOd+0knnbRgDYMRDHYbdGAOGuBt0eC3k5VgCPoF6++8\/QNJGFf99zDB0NfXJ9dcc80i9lEDtH53yimnyMsvv+wJoqBBXK9pD2z+tofFhbnuE088ETj433vvvfPrBux4swdEv2Dw2zLfh4mGqJjVOi+88EKoMLHb5BcLflFnBuuf\/dmflUceeWRB\/xA06Ac9X\/4BX8sEtVH\/3VzHZdvmUvYsSOU61ZLeEAVDSR1TT80yHaL9C8t0tnof9q9r\/RVpOiK7Q7Y7N\/uXlT3A6KBsfgEbG+ba\/l+yhl\/Q93bnd9lll4UKBvsXWFI7LsGgWRX9uKYGojIgmvV49dVXFzGxfxUrp+XLly+4xzhTEv7pEsPe+FOzCX6\/a1t0Pj8o86Es169f792vnZGIsxA0zvSCv4z\/b8PEiBttv+vaJvaC7sf8mwpLveewKQmbo4knv08ffPBBT3gEZQzC7PqzTCarYtuIurbJQJj4d3HJYuqlnvo0tjWYAAUDIwMmEPbrw3S42lGuWrUqcIeAXWb\/\/v2Bvxq1gbYN\/VVrdjS4Fi26fhmFDch2B6rXT2onK8FgX1sHf\/3YA1RYR24PmJ\/+9KdjC4agjEzQdbUd\/imXsF\/wWlYHPtMOm23Q9fxTM1GCIWwqxl8nKlsQJDb992amu\/zCw4iksIHdFZ+2f20bYX71ZysMKyMYwqai7B1Adiyb59KeDjIdgs0laBoM7jhooO4IUDDUncvK1+CiBYO9LdHVIScd6JVu0DbLuHbswdA\/uKhte1tlnAyDlrEXCurfuoXPn2HxD1hJBYOfoz8L4RcqWQkGE81hQkV3jgQJBv\/UhivDUA+CISijZfzqj7+wDINtI+zZoGAoXx9aLy2iYKgXT5W4nWmnJPypczMnHPZrLSiF7BIMUVMJ9q9exau\/wsIEQ1w7mur1D+ZmqsYvGHRQjrOYzD+Y2r\/A\/dM6OsC6piQ0++EacP020k5J2GEb9qvdH9r+wd\/\/a9vcs51pMvdjYsdfJ2hKwvVI5T0lYcSlycyECYagzIxh5M8whC1S9U+HRE1JBHHhlIQrWhrjewqGxvBzrneZ96LHqAHXJRii2hY0vx8mGFx2NH1r1iP4YccRDFonaJeEseVf6W4feJRk0aNJTdt19LpXXnmll\/0I+iinoPuLu+hRbRoR5RcqYQsC7Tp2Gb3m7bffLjfffPOiBZpaxy8Y9N\/CFlCae3UJ1KB0vSvDY3MMu8eowd4eoK+99trQ2IqyoW0IWgwZd9GjzcWVYcu1g6Hx0hCgYCiNK+q\/IXG3VdpbIl3bKoMWUiaZklCqUelu16JC++THKDt6Hf8WvC984Qvy1FNPzS\/yC8ow2INJ2MJN27Z\/bUWQoLAHTruu\/n8jGIKue\/fddy84sVAPk7LXS6TZVmkP\/PYAFrTlNmw7p5+rvaZCbSq37du3y5YtWzwcdqbI7HYJ26bpOj8halulXivuL++wtQeaZQoajMOyKsooaEtrUJYiTGzqv\/tPlgxbC2JsxMmE1X8PxjtwEaBgcBHi95kQcK1Iz+QiNJIbgaCpD\/9OmLBzMOxGpTm4KbebqrhhW+AZYRS0c8KFgQc3uQg1zvd1Lxi0A9I95HoYTVCHFXUQUOO4ufZ3SsFQex8gLYiakomaSgm6ZpqjoZG2N3LdoCkJ5eE67CxI5FXl3R+NHA\/ovde1YHAtojLfr1mzxnupi\/lbH5akL4hBQTd6fQqG+o8Av\/jWO0oqFrQO301QbCz4pwqTiAVtKQVesf4q89XqWjDoPJ8Gs37CMgx++Kq4JyYmCn1bYJkDgG0jARIgARIggTgE6lYw6K+dG2+8UXp6ejzRQMEQx90sQwIkQAIkQALpCNStYNBMgX70pLKoNQw2FpNS7e7uLuV759O5kLVIgARIgARIIH8CdSkYdA70nnvukeuvv170hURxBINZv6BI7bcj+hGfc845+VPnFUiABEiABBqCgL499Oyzz67EvdalYNApCN0jrqfauXZJqJfiigUtq4Jhenq6Es6t1U2QIU6eDMkQJ4BbYBySoU2g7gRD0Eptc0NBr6lNujOCDwgfEJwAboFxSIY4AdwC45AM61ow+N3nyjBoNkJPR4uahrBt8gHhA4ITwC0wDskQJ4BbYBySYaUFg8ko6O6J5cuXe28eNMfDmhuPOoKXDwgfEJwAboFxSIY4AdwC4xBj+Oizh+XKL+6W7\/\/hb2CGSlK77qYk8ubGBwQn\/Pzzz1dmkQ9OI50FMkzHza5FhmSIE8As7Pr2Qbl619Ny6LaPYIZKUpuCwecICgY8MtlRkyFOALfAOCRDnABmgYIB41f62hQMuIvYUZMhTgC3wDgkQ5wAZoGCAeNX+toUDLiL2FGTIU4At8A4JEOcAGaBggHjV\/raFAy4i9hRkyFOALfAOCRDnABmYeiB\/TL0wPNcw4BhLG9tCgbcN+yoyRAngFtgHJIhTgCzQMGA8St9bQoG3EXsqMkQJ4BbYBySIU4As0DBgPErfW0KBtxF7KjJECeAW2AckiFOALNAwYDxK31tCgbcReyoyRAngFtgHJIhTgCzQMGA8St9bQoG3EXsqMkQJ4BbYBySIU4As0DBgPErfW0KBtxF7KjJECeAW2AckiFOALOgpzzq1kqe9IhxLG1tCgbcNeyoyRAngFtgHJIhTgCzQMGA8St9bQoG3EXsqMkQJ4BbYBySIU4As0DBgPErfW0KBtxF7KjJECeAW2AckiFOALNAwYDxK31tCgbcReyoyRAngFtgHJIhTgCzQMGA8St9bQoG3EXsqMkQJ4BbYBySIU4As7D+rifl0ecOc9EjhrG8tSkYcN+woyZDnABugXFIhjgBzAIFA8av9LUpGHAXsaMmQ5wAboFxSIY4AcwCBQPGr6a1p6ampK+vT4aHh6WtrS2wLRQMuIvYUZMhTgC3wDgkQ5wAZoGCAeNXs9pzc3MyMDAg+\/btk9HRUQqGHD3BjhqHS4ZkiBPALTAOMYYrb35MDhx6m2sYMIzF1967d68MDQ15F2aGIV\/+7GRwvmRIhjgB3ALjEGPYvPkhzwBPesQ4Flr70KFDcuONN0pPT48nGigY8sXPTgbnS4ZkiBPALTAOMYYUDBi\/mtQeGxvzrrtq1SquYSjAA+xkcMhkSIY4AdwC4zA9w0efPSzrv\/IkMwzpERZfUxc63nPPPXL99dfLzMxMLMFgt3LPnj3FN7rOr6icW1tb6\/wuatt8MsT5kyEZ4gSSW+jo6PAqvXPy+fLm2j4KhuQIa1dDpyDWrVsn7e3twl0SxfiBv0pwzmRIhjgB3ALjMD1DZhjSs6tJTV270NvbK5OTk4uuv3PnTk9E+D\/cVom7ip0MGeIEcAuMQzLECaS3YLZU\/pu3XpFXfu8T6Q2VqOZxR44cOVKi9uTaFGYYcsU7b5wdNc6ZDMkQJ4BbYBymZ2i2VFIwpGdY05oUDMXgZyeDcyZDMsQJ4BYYh+kY6tkLKhj086PPfV1e+tq2dIZKVquhMgxx2HNKIg6l6DLsZMgQJ4BbYBySIU4gnYVd3z4o+qZK\/Rz\/6LAc+NafpzNUsloUDD6HUDDgEcqOmgxxArgFxiEZ4gTSWTDrF85sXiKvf\/U\/yvT0dDpDJatFwUDBkHlIsqPGkZIhGeIEcAuMw+QM7emIteeeKH9z6y9RMCTHWB81mGHA\/cROhgxxArgFxiEZ4gSSW\/iDiVnZ\/Cff9So+9flL5MMf\/HEKhuQY66MGBQPuJ3bUZIgTwC0wDskQJ5DMgp1d0OkIFQxVGlM4JcEpiWRPRIzS7KhjQHIUIUMyxAngFhiHyRh+8c+m5ct7XvAq3dVzgfRcfBoFQzKE9VW6SmqwVuTZyeDkyZAMcQK4BcZhfIZB2QWtXaUxhRkGZhjiPxExS7KTiQkqohgZkiFOALfAOIzHUMWCvmhK\/6sfnYrQKQkKhnj86rZUldRgrZzATgYnT4ZkiBPALTAO3QxVJFyz62l59LnDXuE\/\/tRPykcvPHm+YpXGFGYYmGFwPxEJS7CTSQgsoDgZkiFOALfAOIxm6BcLZqGjXYuCAY\/D0lqoknNrBZmdDE6eDMkQJ4BbYByGM\/RPQ6hYuP+zF81PRZiaVRpTmGFghgHvVXwW2MngSMmQDHECuAXGYThD83IpLRGUWaBgwOOv9BaqpAZrBZudDE6eDMkQJ4BbYBwuZKhZBX1PxNADz89\/EZZZoGDA46\/0FigYcBexkyFDnABugXFIhjiBYxYeffawXLP76fmdEPrNf\/\/UT8ovWgscg65XpTGFUxKcksjymfJssaPGkZIhGeIEcAuMQ\/EEgmYUNLNgPppVuLP7All73olOyBQMTkT1W6BKzq2VF9jJ4OTJkAxxAriFRo7DIKGgRH\/nl5bLZRectGhxYxjtKo0pzDAww4D3Kj4LjdzJZAWTDHGSZEiGaQjo1MPwA8\/Pn6tgZxWCdkG4rkHB4CJUx99Xybm1cgM7apw8GZIhTgC30EhxGLRGQQkmmX4IIl6lMYUZBmYY8F6FGQYyzJwAbrCRBjucVrCFKjM0xzj71yfYGQU94hn9UDCgBMH6c3NzMjAwIOPj456lTZs2SX9\/f6jVsbEx2bp1q\/d9Z2enDA4OSlNTU2D5KjkXxJy6epU7mdRQElYkw4TAAoqTIRkGEdBMwq5vv7RgEaMtEvp+YZmsPW9p7DUKLspVGlPqMsMwNDTk+UhFwqFDh6S3t1e6u7ulq6trke\/27t0rWn5kZMQTCSo0WlpaQgVGlZzrCuS8vmdHjZMlQzLECeAWqhCHrkyCUnKdpYCQrNKYUohgMIP65ORkIu4rVqyQ++67z1nHFhD+wppdmJiYmM8q+P\/2l6+Sc53gcipQhU4mJzSxzZJhbFShBcmwsRmGLV60swlxt0YiJKs0phQqGDQj0N7eHou9yQy4BIMRI2G2gzIMa9asCcxGaMOq5NxYoHMoxI4ah0qGZIgTwC3UUxxGTTUULRJs8lUaU+paMGhmYceOHc51CVNTU7Jx40aZnZ2VnTt3RooWda792bNnD\/7UNZiFmZkZaW1tbbC7zvZ2yRDnSYbVZrjvxbdl\/Ok3vf+FfVpOeJesPmOJfOanjx6wpH\/n\/eno6Fh0ienp6bwvW4j9QgRD3neiwkHFQNBiRp2C2L17t7eGobm52VvPoJ+wRZJVUoN5cw+zX0+\/SmrFyHVdMnQRcn9Phm5GrhJlYRhnHYKdRdj28Tb5qOPIZte9Z\/V9lcaUQgSDvYbBtaMhjZM0g9DX1yfDw8PS1tY2b8LsprCnIMLKmkpVcm4allnUKUsnk8W91MoGGeLkybB+Ger0whtvvyP\/7eHvLTpAyX9XumDxU2vOkFVnnhDrqGacSjILVRpTChEMBq+ZQtC\/dafC6OjoggE+mRuOlbbXKWgWwXwoGNISxeqxo8b4aW0yJEOcAG6hiDjU7MGz339LvrznBac40DtSgfDJ1afJf\/qZ0zPb+oiTCrdAwQDS9e+aSJp1sKcVjCgI2yoZNCURNn2ht1Ul54JuSl29iE4mdePqpCIZ4o4iw\/IxVHEw9f235PYE4uDMpUuk7\/KzPXGg\/6u3T5XGlEIzDEGOthckxs06+A9usg9jMt\/19PTML260Mxs8uCn\/x40dNc6YDMkQJ4BbSBOHKgp0YNdphcemD8sjU6\/Fzhxoiz+ztlX+\/U+d4jW+HgWCnzoFAx6HgRbCphZyulyg2So5t0hu9rXSdDK1amtZr0uGuGfIsBiGKgze37zEe2HTN5877L0O2vUxQuDXLjlDPnhWOdceuO4h7vdVGlNKlWFQB7i2PcZ1UtpyVXJuWgZoPXbUKEGuYcAJkmHWDFUI6P\/ue+r7MvX3\/xgra2CyBDqt8PmPnSM\/eOdI3U4tpOVZpTGlJoIhakohrVOyqlcl52bFJKkdCoakxBaXJ0MyxAkkt2CyA\/tfnZOxxw\/KM7OHRc87iPMxWYNL25Z6ixLXnnf07ING\/1RpTClUMNhrCcqQTQgK5Co5t1YPKgc7nDwZkiFOINqCTiVMzrwhD3znFTnw2tHsQdyPioOzmpvk9k+eLzOH\/6nhsgZxOWm5Ko0phQgGe1eEa9FhEkfkUbZKzs2DTxybHOziUIouQ4ZkiBIwAkD\/u\/vxg3Lg1bnY0wjm2noy4jmnHC+\/9fNnybmnvLsSixBRrknrV2lMKUQwPPvss3LttdfKDTfckPm7JJI6z1W+Ss513Wte33Oww8mSIRnGJWCEgR5y9J3ZNxOLAjOV8KFzT5Sei0\/3Lmu2MDIO43ohvFyVxpRCBIPrBVFBqOO+fAp350ILVXJu1mzi2mMnE5dUeDkyJEMlYG9R1EH81gf3y\/OvJM8UzIuApUvkYz91ivzE6cfHWmPAOMTjsEpjSqGCIa\/XW+MuPWahSs7NkksSW+xkktAKLkuGjcdQ1xQ89\/Jb8q39\/yDfO\/R24kyBLQouPON4+cULj51lkPY8A8YhHodVGlMKEQw48uIsVMm5xVFbeCV2Mjh5MqwOQ3+W4I+\/9ZI8pucVJFxoaIh40wVLl0jb+35M\/sPKU71\/zmtHAuMQj8MqjSkUDL54qJJz8VBPZ4GdTDpudi0yrE+GmiV4+uCb8tT33kidJbAzBWvblkr3B0+bn5pImylIS5NxmJZcNbPWFAwUDPgT4bPATgZHSoblYmiyBPrft37wL3LnN76XateBnSXQ\/68LDTs+cJKc+p4f8bIE5jr43WdjgXGIc6zSj1AKBgoG\/ImgYCDDzAngBpMOdma3wd++9KZ8\/elX5dm\/fyvVOgL\/1ME5p7xbNv\/8WTXLEiAkkzJErlXVuhQMVfVsxQ7ZqJWb2Mng5MkwW4bmWGNN6X\/jmUPyV8+nX1joFwTLTm6ST6x+nxwnx82fU1D01AFOK9gC4xAnS8GAMyythSo5t1aQ2cng5MkwHkN7qkBr6N9\/8Z1X5P\/NvCHTL78ps6+\/E89QQCmzuFAFwcVnvVfWLV\/qlaqKGIgDhnEYh1J0mSqNKTWbkhgbG5OtW7d6pPWFUy+88IJMTEzI4OCgNDU14V5KaaFKzk2JAK7GTgZGKGR4jKF9YuF3XnrTEwO67TDtLgN\/hmDF+98jn17bWpdTBnikRVtgHOKEqzSm1EQw6DslZmdnpa+vT6655hrp7++XFStWyMDAgLS0tHh\/1+pTJefWiiE7GZx8IzG0X3j0p099X579\/luZiIF33nnHO9Z43fJm+Zmz3+s5pYwLC\/Foyc9CI8VhXhSrNKYULhjsUx+XL18uvb29nkBob28Xc7rjyMiINDc35+W\/SLtVcm5NAApfK5wF96p01HZ2wEwVKJ9HnzsMYTLTBe9vXiLX\/tyZ0vTv\/q03VWDvMqgKQwgUWJkMQYAVWxdHweCLBwoG\/AFhJ9MYDO23G\/7BY7Py+P5\/8G48CzGgdlQA6BkEZzYfnaI0awfiriFgHDZGHOJ3ma+FKo0phQsGdY2uX9D1CvaUhMk2dHd3S1dXV6QH5+bmvOmL8fFxr9ymTZsipzE0c7FhwwavrE59RGUwquTcfB+DcOvsqHHytWRoLyQ8IiJ3PnRAvnvwH7MVA0uXiK4duPzHT\/bs5jFVUEuGeASUwwIZ4n6o0phSE8GgLrAHceOSbdu2OcWCltU1EPrRqQwzxREmNKampmTjxo2yfft2b9rDiJWwxZVVci4e6ukssJNJx82ulRdDe4rgzX96R\/73X7\/iHUCELiBckAFYukTOOqlJ\/usvLJvPDNTiQKK8GOLerR8LZIj7qkpjSs0EA+6GYxZsAeG3qwJh\/\/79sRdSVsm5WTJOYoudTBJawWXTMLTFwMzht+WRqdcy2U3gFwOrzzrBO51wwb83L8FvOmMLaRhm3IS6N0eGuAurNKbUvWCIenW2mbpYs2ZNrMyFhkaVnIuHejoL7GTScYvKMBgx8K9HjshXvvE9+buMpgj8YuBD5y31jis2\/x53vQB+x9lbYBziTMkQZ1ilMaVwwWAGeNerrl3rEtSNmlnYsWOHdHZ2Bp7fYATD5ZdfLnfffbfoNbmGAX8AXBbYybgIHfs+7IwB9NAhcwV7N8HHV54q57\/vxxY0rp4FgYsy49BFyP09GboZuUpQMLgIOb7XaYLdu3cvWHxor0VYv359ojMZzLkO\/nUJRjAcOHBg\/lphZU2T1bn2Z8+ePeDdNl71mZkZaW1tbbwb993xvhffltPf8y559tUfyMQLczJ96J\/lpTfegU4fNJdoOeFd3v9V+xec+iPyyZ88QQ6+efRUQ\/03\/ZgyjeoIxiHueTJMzrCjo2NRpenp6eSGSlijZhkGc\/aCzcQ+h+GZZ57xMgj33XefE5subNQdF8PDw9LW1jZfPmhKIqysLRiq4lwnuJwKVPlXib2V8J1\/PSJ3Pzoj33nxTY8kup3QuEN\/9Z\/SJLK85US56P0nyAdOO5oV8LIFJVwrkFMYwWarHIcwnJgGyDAmqIhizDAADKPWHKQVDFEHPqnoWLZs2fwaBhUMt9xyi9x6662Bh0NVybmAm6Cq9drJ2C8oGvnmi\/Lkgdc9DlnsIDADvvffpUvkwjOOl9+49P2LOBtBUK8MocDJuDIZ4kDJEGdYpTGl8AyD4ndNSeg5DGb74+23377IY\/auCJNFCDtS2i8monZU6IWq5Fw81NNZKFsnY68T+OZzh+Wbz76WmxDQ7YQfvfBkubDleOjdBGVjmC4SaluLDHH+ZIgzrNKYUhPBoC4IOodBX0Jlzkq44447ZHR0dMEUg3Gd\/+Ame9Gj+a6np8ez5b9W2AJJY7tKzsVDPZ2FIjuZR589esTwEwdel\/\/z9Ku5CYFzT3m3XHLOe6XlxKPbB5OeOpiUZJEMk7atXsqTIe4pMsQZVmlMqZlgwN2Qj4UqOTcfQm6rWXQyJisw9vhB7zwB\/eQxNaDvIvjgWe+VtlPfnbsIcJM7ViILhkmuV8WyZIh7lQxxhlUaUygYfPFQJefioZ7Ogr+TsY8a9gb+Q2\/LX7\/4hvzN7JuZHSy04Ff\/0iWiQuB9J\/yo\/NolLfMZgVqcNpiOIF\/glZabXY+DHU6RDHGGVRpTaiIYzHHN+opr\/8d1TgLuvmgLVXJu3qxs+zo1oGn651+Zkz9\/Yr\/87avZZQSMGNDFgioEPry8WU5\/74\/OCwFbLBR5z3leix01TpcMyRAngFuo0phSuGCwtzqa8xZ0vYH\/Vde4m9JZqJJz0xFYXGvhWwlflMf3v57p9IARAj9x+vHyU63vmRcC\/tcVZ3U\/9WCHgx3uJTIkQ5wAbqFKY0rhgsG\/rdLe9qgLIXft2hV4aiPutngWquTceHd8rJRmCeb++V\/kjr884P0jcq7A\/KLApUvkJ844Xj524SmezTzeSpj0PuuhPAc73EtkSIY4AdxClcaUmgsG++VQUecp4G6LZ6FKzg26Y5MtuOsbB+Tpl\/4xlSiwxcDPfeAkufKiU+e3EOo1\/+UfXpKzzz47HnCWCiTAwQ4PDDIkQ5wAbqFKY0rhgkHx22ch2CLhwQcflImJCWYY8Bidt6BZg\/\/x+EHZ\/+pcbHFgC4IrV71Pzjvl3fPZgThNY0cdh1J0GTIkQ5wAboFxiDOkYAAZ+o9sNi+R0sOXws5eAC8Zu3o9O9dkD4YeeF52ffug857Ni4muWHmqXHbB0dcV6wc9fpidjBO9swAZOhE5C5ChE5GzABk6ETkL1POY4r+5mmQYnIRrWKAenaviYNe3XorMIKgI0NcW919+dKoAFQVRLmIngwcwGZIhTgC3wDjEGdbjmBJ214ULhrjvkmhubsY9lcJCPTl3\/V1PhooEFQS9HzpDrlhxaq7iIAgxO5kUgeerQoZkiBPALTAOcYb1NKa47paCwUeozM7VKYdHnn1NPrf77xb51c4g5Jk9cAWUfs9OJg6l6DJkSIY4AdwC4xBnWOa6RcI5AAAgAElEQVQxJendFSYYdDfE1q1bne3btGmT6Kuva\/Upo3NVKPzfqdfk2rGFQsGIhJ6LT\/e2K5blw04G9wQZkiFOALfAOMQZlnFMSXtXhQkG08CoKYm0N5FlvbI5V3c5rP\/KkwtuUYXCnd0XeFMNtc4mBLFnJ4NHJBmSIU4At8A4xBmWbUxB7qhwwYA0toi6ZXGuZhWu2fX0gjUKKg7u\/+xFpRQJtm\/YyeCRSoZkiBPALTAOcYZlGVPwOxGhYPBRLINz\/VmFehEKBiU7GfzRJEMyxAngFhiHOMMyjCn4XRy1UIhgMNMQk5OTznY38sunNKvwF995RQbum5rnpGLhqc9f4uRWpgLsZHBvkCEZ4gRwC4xDnCEFA86wtBZq5VwVC\/aBS2adQpkWM8Z1GjuZuKTCy5EhGeIEcAuMQ5xhrcYUvOWLLRSSYcij4XnZrIVzVSzowkZzUmO9TUH4fcFOBo9OMiRDnABugXGIM6zFmIK3OthCzQRD0DbLbdu2SVdXl\/NezdHS4+PjXtm4WzGnpqakr69PhoeHpa2tLfA6RTu3amJBobKTcYawswAZOhE5C5ChE5GzABk6ETkLFD2mOBsEFKiJYFCxsHv3bhkZGRFzoqNZ59Dd3e0UDfbLq+LWMyJj3759ke+rKNK5\/p0Q9Z5ZMHHITgZ4In9YlQzJECeAW2Ac4gyLHFPw1kZbKFww5HE0tC0gwm7XvBVTvy9LhuHqXU\/PvySqHhc3hrFmJ4M\/tmRIhjgB3ALjEGdIwQAwzFowxDkISsvceOON0tPT471auwyCQV8YpYJBP1XJLDDDADwYvqrsqHGWZEiGOAHcAgUDyBCdkjCXN6\/F7uzslMHBQWlqagpsmV5PP6tWrSrFGgadilh582OVFAt6U+yowQeEDHGAZEiGmRDAjVAw4AwFWfTov7wKh9nZ2UDRoAsd77nnHrn++utlZmYmlmCw7e\/ZsyeDuz1mYvb1d2TTfQdF\/6uf37\/yNFl9xpJMr1FrY8q5tbW11s2o6+uTIe4+MiRDnEByCx0dHYsqTU9PJzdUwhqFr2HIg0HU7gcVE+vWrZP29nYpwy4Je93Clz5+nvzmpe\/PA0lNbTLDgOMnQzLECeAWGIc4Q2YYAIZxdzUkuYRZ0GjvutD6USdM7ty50xMR\/k+ezrWPfK7SIkc\/Q3YySaI3uCwZkiFOALfAOMQZ5jmm4K1LZqEmGQb\/dETY4B12K\/auCLNdsqWlxfla7FpmGOzzFur5FMc44cVOJg6l6DJkSIY4AdwC4xBnSMGAM5y3YBYu6j\/ooD86Ohp6qJKp5D+4yV70aL7THRH+DEItBcNffveQ\/PKOo+\/SuGn9eXL1h6s3FWH8w04Gf0DIkAxxArgFxiHOkIIBZxhoQcWDTi\/4pxZyulyg2Tyc698VUW8vk0rKn51MUmKLy5MhGeIEcAuMQ5xhHmMK3qp0FmoyJWE31c4w1PpNldquPJyrb5\/8\/UdmvNu+\/7MXST2+UCpJeLGTSUIruCwZkiFOALfAOMQZ5jGm4K1KZ6EmgiHNNES620teKw\/nNm9+yGtIlRc62qTZySSPO38NMiRDnABugXGIM8xjTMFblc5C4YIhzsmM6W4lm1pZO9feRtkI2QX1AjsZPBbJkAxxArgFxiHOMOsxBW9ReguFC4b0TS2mZpbObbS1C8ZD7GTwWCVDMsQJ4BYYhzjDLMcUvDWYBQoGH78snXvLnz8v2x\/c711BFzrqlEQjfNjJ4F4mQzLECeAWGIc4wyzHFLw1mAUKhpwEQ6NmFzglgT2QzNJkw49xmA1HCgacIwUDzrC0FrJyrv02yrt6LpCei08r7T1n3TB2MjhRMiRDnABugXGIM8xqTMFbglsoPMOQ9eutcQQLLWTl3PV3PSmPPne4YXZG2BTZyeBRSYZkiBPALTAOcYZZjSl4S3ALFAw5TEnY74z4Lx1nyQ0fOwf3VB1ZYCeDO4sMyRAngFtgHOIMKRhSMAx6nXWQmU2bNjnfCZHi8rGrZOFck13QizbKVkpmGGKHWKyC7KhjYYosRIZkiBPALWQxpuCtyMZCqTIM2dwSZgV1biMvdjTk2VFjMai1yZAMcQK4BcYhzhAdU\/AWZGehcMGQXdPzsYQ6156O6L\/8bOm\/fFk+DS2xVXYyuHPIkAxxArgFxiHOEB1T8BZkZ4GCwccSda692FGnIxrl7AUbIzsZ\/AElQzLECeAWGIc4Q3RMwVuQnYVCBIO9M2L58uXS29srk5NHX\/Xs\/9T6BVSoc817I9aee6Lcf\/VF2Xmqjiyxk8GdRYZkiBPALTAOcYbomIK3IDsLhQiG7JqbvyXEuY189gIzDNnGJjtqnCcZkiFOALeAjCn41bO1QMHg44k4t5HPXqBgyPbB5GCH8yRDMsQJ4BaQMQW\/erYWChcMZnqialMS9u6IRp6O0PBkR40\/pGRIhjgB3ALjEGdIwYAzXGRBhcR1110nv\/3bvy1tbW05XCGeybTOtQVDo+6OMITZycSLtahSZEiGOAHcAuMQZ5h2TMGvnL2FwjMMUbewd+9e2bVrlwwODkpTU1No0bm5ORkYGJDx8XGvTNRhT\/6MRmdnZ6T9tM61D2tqpDdTBjmJnQz+oJIhGeIEcAuMQ5xh2jEFv3L2FkonGIaGhmRkZESam5tD71bL6Ke\/v1+MIOju7paurq4FdYywWLNmjfed+bulpSX0NMk0zuVhTQtdxU4Gf1DJkAxxArgFxiHOMM2Ygl81HwulEgwqBGZnZ50ZBj8KW0C4MOkR1RMTE6HXSONcTkdQMLjiLun37KiTEltcngzJECeAW0gzpuBXzcdC4YIhatGj\/vIfHR1NtIYh6u2XQcjyEAxDD+yXoQee9y7X6NMRyoAdNf6wkiEZ4gRwC4xDnCEFA8gwbKrATB3ENa+ZhR07dohrXYKxFzV9Ycqoc+3Pnj17nM35zL0HZd+Lb0vLCe+S8ataneWrXmBmZkZaW8kB8TMZIvSO1iVDMsQJJLfQ0dGxqNL09HRyQyWsUXiGQRkETT3EGczD+MWZyjAiRW1ELapMqga5nXKxV\/irBH\/SyZAMcQK4BcYhzjDpmIJfMT8LhQuGqCmEuLsk\/Dimpqakr69PhoeHA6cz4ooFtZvUuXzZFAVDHo8nO2qcKhmSIU4At5B0TMGvmJ+F0gmGOLsk\/DhUaITVi7MzwraX1Ll\/9FcvyX8e+zvPhL5sau15J+bnrTqxzI4adxQZkiFOALfAOMQZJh1T8CvmZ6FwweBfv2DfmmtBoilr74pwCYI40xWIYOBx0Mww5PF4sqPGqZIhGeIEcAsUDCBDzQhs2bJlwY4InVbYuHGjbN++Xdrb2yOv4D+4yV70aL7r6emRsDdjRr0RM4lzuX4h2E3sqMEHhDtNcIBkSIaZEMCNJBlT8Kvla6HwDIO5HRUNGzZsWHB3O3fudIqFfHEkW8PA8xcoGPKKR4ounCwZkiFOALdAwYAzLK2FJM61z1\/g+oVjLmVHjYc3GZIhTgC3wDjEGSYZU\/Cr5Wuh8AyDPWXgmnrI99aDrSdxLtcvMMOQV4yyo8bJkiEZ4gRwC0nGFPxq+VooXDAkPZkx39tfbD2uc7l+Idwz7KjxqCVDMsQJ4BYYhzjDuGMKfqX8LRQuGPSW4u6GyP\/20wsG+\/wFTkcs5MhOBo9cMiRDnABugXGIM6RgABhGvUtCzUbtYAAuG7tqXOdy\/QIzDLGDKkVBdtQpoPmqkCEZ4gRwC3HHFPxK+VuoSYYh\/9tKf4W4zjXrF\/RKh277SPoLVrAmO2rcqWRIhjgB3ALjEGcYd0zBr5S\/BQoGH+O4zl1582Oi6xjObF7ivaGSn2ME2Mng0UCGZIgTwC0wDnGGcccU\/Er5WyhEMNgLHcMOUzK3Wg9TEvaCx0vblsqf\/ubK\/D1VR1dgJ4M7iwzJECeAW2Ac4gwpGHCGpbUQx7k8sCnafexk8PAmQzLECeAWGIc4wzhjCn6VYiwUkmHw34r\/fRJR75coBsOxq8Rxrr3gUacjdFqCH05JZBkD7KhxmmRIhjgB3EKcMQW\/SjEWaiIYgl4IZaYturu7paurq5i7D7hKHOdywSMzDHkHKAc7nDAZkiFOALcQZ0zBr1KMhcIFQ9TBTfp+iV27dsng4KA0NTUVQ8B3lTjO5YJHCoa8g5ODHU6YDMkQJ4BbiDOm4FcpxkLpBINmH0ZGRqS5ubkYAgkFA094dLuFHbWbkasEGboIub8nQzcjVwkydBFyf0\/B4GYUWiJqvUIZToB0OZcnPLqdz07GzchVggxdhNzfk6GbkasEGboIub93jSluC+UpUXiGQW9dpx62bNkio6Oj0tbW5tGYmpqSjRs3yvbt22v6imuXc3nCozt42cm4GblKkKGLkPt7MnQzcpUgQxch9\/euMcVtoTwlaiIYjGjYsGHDAhI7d+6sqVjQxrice92fPCOjEy967eYOieBAZieDP+BkSIY4AdwC4xBn6BpT8CsUZ6FmgqG4W0x2JZdz+UprN092Mm5GrhJk6CLk\/p4M3YxcJcjQRcj9vWtMcVsoT4nKCwazZmJ8fNyjvmnTJunv7w\/1gMu5zZsf8uquPfdEuf\/qi8rjyRK1hJ0M7gwyJEOcAG6BcYgzdI0p+BWKs1B5waC7LvSjIiHOWQ9RzuUJj\/ECk51MPE5RpciQDHECuAXGIc6QggFnWDMLtoAIakRcwXBXzwXSc\/FpNbuPMl+YnQzuHTIkQ5wAboFxiDOkYMAZ1sRC1KFRpkFRzuWR0PHcxk4mHidmGHBOZEiG+RLArVMw4AwLt6CZhR07dkhnZ2fkSZJRzuWR0PHcRsEQjxMHO5wTGZJhvgRw6xQMIENz5sLs7OwiS3m\/3jroPRZ2I9S59mfPnj3zf37m3oOy78W3peWEd8n4Va0ghepWn5mZkdZW8kE8TIYIvaN1yZAMcQLJLXR0dCyqND09ndxQCWsUvujR7FpoaWmJ3K2QFysVK319fTI8PDx\/aJRfMIQ5lzsk4nmFGYZ4nPjrGOdEhmSYLwHcOjMMAMM46wgA886qespk1PsqopxrBEP\/5WdL\/+XLnNdq1AIUDLjnyZAMcQK4BcYhzpCCAWBoMgw9PT2FnOpo74qIk90Icy7fIRHf6exk4rMKK0mGZIgTwC0wDnGGFAwgQ9evfND8gur+g5vSLnrkOyTie4WdTHxWFAw4KzIkw\/wI4JYpGACGZkpicnIy0Ereix5dTQ9z7nV\/8l0ZnTi6SJPvkIimSMHgijL392ToZuQqQYYuQu7vydDNyFWCgsFFqI6\/D3Mu3yER36nsZOKz4q9jnBUZkmF+BHDLFAw4w9JaCHPuypsfEz0amu+QcLuOgsHNyFWCDF2E3N+ToZuRqwQZugi5v6dgcDNylhgbG5OtW7d65fS11i+88IJMTExEHqrkNJpBgSDn2u+QoGBwQ2Yn42bkKkGGLkLu78nQzchVggxdhNzfUzC4GUWWMIcn6XkI11xzjXceg65dGBgYkFqdz2AaHORce4cEt1S6nc9Oxs3IVYIMXYTc35Ohm5GrBBm6CLm\/p2BwMwotYZ\/DsHz5cunt7fUEQ3t7uxS5eyKsgS7BcP9nL5K1550IEKh+VXYyuI\/JkAxxArgFxiHOkIIBYFiPgoEvnUrmcHYyyXgFlSZDMsQJ4BYYhzhDCgaQoa5f0PUK9pSEyTZ0d3dLV1cXeIX01YOce\/Wup2XXtw96Rg\/d9pH0xhukJjsZ3NFkSIY4AdwC4xBnSMGAM\/SmHzZs2LDA0rZt22oqFrQxQc7llspkDmcnk4wXMww4LzIkw3wI4FYpGHCGpbUQ5FxuqUzmLgqGZLw42OG8yJAM8yGAW6VgwBmW1oLfudxSmdxVFAzJmflrkCEZ4gRwC4xDnCEFA85Q7HMYjDk9j0F3S9TyEyUYuKUynmfYycTjFFWKDMkQJ4BbYBziDCkYQIYqFnbv3i0jIyPS3NzsWTO7J8q26JFvqUzubHYyyZkxw4AzI0MyzJ4AbpGCAWBob6v0ZxPKeA4D31KZ3NkUDMmZcbDDmZEhGWZPALdIwQAwrGfBwLdUxnM8BUM8TpySwDmRIRnmSwC3TsEAMtRMwpYtW2R0dFTa2tpKPSWx\/itPik5LeNMmPIMhlucpGGJhiixEhmSIE8AtMA5xhhQMAEOTYZicnHRa0fdL3Hfffc5yWRbwO5dnMCSny04mOTOm03FmZEiG2RPALVIw4AxLa8Hv3ObND3lt5Vsq47uMgiE+q7CSZEiGOAHcAuMQZ0jBgDOELPizFJ2dnZGvxba3cLrK2s61z2D41fYW+fInz4fa3SiV2cngniZDMsQJ4BYYhzhDCgacYeA5DHGOhp6bm\/Neg71mzRrvGGnzd9hrse2dF01NTc5XaIcJBp7BEN\/p7GTis2KGAWdFhmSYHwHcMgUDyDDrcxjMy6wGBwdFRYH98X8XVVbr2c61z2C4q+cC6bn4NPDOG6M6BQPuZzIkQ5wAboFxiDOkYAAY5rGtMkoEBGUYTHYi6DbCBMP9n71I1p53InDnjVOVnQzuazIkQ5wAboFxiDOkYAAYZi0Y4pwQOTU1JRs3bpTZ2VlxHT9tO9c+tIlnMMR3OjuZ+KyYTsdZkSEZ5kcAt0zBADLMakrCrF\/Q5gRNR+i\/+681NDTktb6\/vz\/wLtS55vPm2j555+SjCx33fW4ZeNeNU31mZkZaW1sb54ZzuFMyxKGSIRniBJJb6OjoWFRpeno6uaES1jjuyJEjR2rRrqCXT8VZ9GjaGkcs+BdIal3NNvT19cnw8PD8oVH2\/dtqkGcwpIsMZhjScbNrkSEZ4gRwC4xDnCEzDDhDyIJrZ4RfVNhrFpIIhpU3Pya6tZJnMCRzFzuZZLyCSpMhGeIEcAuMQ5whBQPOELKg0wq6HiFsGsI2HjQlEVXXONc+g4GCIZm72Mkk40XBgPMiQzLMhwBulYIBZ5jaQtjR0nqMtL4u25y10NPTI+ZtmCowduzY4V0z7sFNtmDgGQzJ3EXBkIwXBzucFxmSYT4EcKsUDDjD0loIyjBQMCRzFwVDMl4c7HBeZEiG+RDArVIw4AxLa8E41z60iWcwJHMXBUMyXhzscF5kSIb5EMCtUjDgDEtrwTjXPoOBgiGZuygYkvHiYIfzIkMyzIcAbpWCAWdYWgtBgoGHNiVzFwVDMl4c7HBeZEiG+RDArVIw4AxLa8E415zBoA09dNtHStveMjaMggH3ChmSIU4At8A4xBlSMOAMS2vBLxjObF4immHgJz4BdjLxWYWVJEMyxAngFhiHOEMKBpxhaS0Y5\/LQpvQuYieTnp2pSYZkiBPALTAOcYYUDDjD0lowzm3e\/JDXRh7alNxV7GSSM\/PXIEMyxAngFhiHOEMKBpxhaS2oc7\/x+N+KZhj087ufOF+uuqSltO0tY8PYyeBeIUMyxAngFhiHOEMKBpxhaS2oc\/\/w60\/I+q886bWRhzYldxU7meTMmGHAmZEhGWZPALdIwYAzLK0Fv2C4q+cC6bn4tNK2t4wNo2DAvUKGZIgTwC0wDnGGFAw4w9JaUOd+aWxCrt71tNdGHtqU3FXsZJIz469jnBkZkmH2BHCLFAw4w9Ja8AsGHtqU3FUUDMmZcbDDmZEhGWZPALdIwYAzLK0Fde5Hv\/RnsuvbB7028tCm5K6iYEjOjIMdzowMyTB7ArhFCgacYWktqHMvvO5\/yaPPHRYe2pTOTRQM6bjZtciQDHECuAXGIc6QggFnWFoLFAy4a9jJkCFOALfAOCRDnABugYIBZ1haC+rcEz71x3Lg0Ns8tCmll9hRpwRnVSNDMsQJ4BYYhzhDCgacYWktqHMPf3zEax9PeUznJnYy6bhxSgLnRoZkmC0B3BoFA86wtBaWXfjT8vovDHnt0\/MX9BwGfpIRoGBIxiuoNBmSIU4At8A4xBlSMOAMIQuHDh2S3t5emZyc9Ox0dnbK4OCgNDU1Bdrdu3evbNiwwftuxYoVMjIyIs3NzYFlbcHAUx7TuYmdTDpu\/HWMcyNDMsyWAG6NggFnmNrC3NycDAwMyJo1a6Srq0vM3y0tLdLf37\/I7tTUlGzcuFG2b98u7e3tMjY2JhMTE6ECwxYMPOUxnZsoGNJx42CHcyNDMsyWAG6NggFnmKmFKBGg3+3fvz9QTAQ1ovXDvyJvrfqU9xVPeUznJgqGdNw42OHcyJAMsyWAW6NgwBlmaiFMMPizEXEu2vKLvyVvf2A9BUMcWCFlKBgAeD+sSoZkiBPALTAOcYYUDDjDzCyY9Qzd3d3eFIX9MYLh8ssvl7vvvttb8+Baw2ALhhO+3i8PjY9l1tZGMTQzMyOtra2Ncru53CcZ4ljJkAxxAsktdHR0LKo0PT2d3FAJaxx35MiRIyVsV6wmGUGghYMWPZrvDxw4ML\/QcWhoSGZnZ0PXMJz2yzfLD878kHd9Hgsdyw2LCvFXSTpudi0yJEOcAG6BcYgzZIYBZwhbcIkFvUDQlIQuguzr65Ph4WFpa2tb1I5Tf\/X35J2Tz+ex0ICH2MkA8H5YlQzJECeAW2Ac4gwpGHCGkAXXzgjbuGYUli1bNj9doYLhlltukVtvvTVwayUFA+QarzI7GTLECeAWGIdkiBPALVAw4AwhC65pBdu4nsGg5c3ZC\/r\/9RO0BVP\/\/eTf+J\/yr+8+mac8Ah5iRw3AY4YBh0eGZJgZAdwQBQPOMLUF\/6FNxpBZzKiHN+k5DT09Pd65C\/qxD25yHfLUvPkhrw6PhU7tImYY0qObr0nRhUMkQzLECeAWKBhwhqW1YATD737ifLnqkpbStrPMDWNHjXuHDMkQJ4BbYBziDCkYcIaltWAEA4+FTu8idjLp2ZmaZEiGOAHcAuMQZ0jBgDMsrQUjGHgsdHoXsZNJz46CAWdHhmSYHQHcEgUDzrC0Foxg4LHQ6V1EwZCeHQc7nB0ZkmF2BHBLFAw4w9JaoGDAXUPBQIY4AdwC45AMcQK4BQoGnGFpLRjBwFMe07uIHXV6dvx1jLMjQzLMjgBuiYIBZ1haCxQMuGsoGMgQJ4BbYBySIU4At0DBgDMsrQUKBtw17KjJECeAW2AckiFOALdAwYAzLK0FFQxnNi+Rpz5\/SWnbWPaGsaPGPUSGZIgTwC0wDnGGFAw4w9JaoGDAXcNOhgxxArgFxiEZ4gRwCxQMOMPSWlDBwGOhMfewo8b4aW0yJEOcAG6BcYgzpGDAGZbWggqGnotPEz24iZ90BNjJpONm1yJDMsQJ4BYYhzhDCgacYWktqGDgsdCYe9jJYPyYYcD5kSEZZkMAt0LBgDMsrQUKBtw1FAxkiBPALTAOyRAngFugYMAZltaCCga+RwJzDztqjB9\/HeP8yJAMsyGAW6FgwBmW1kLrh39FZr7xR6VtXz00jIIB9xIZkiFOALfAOMQZUjDgDEtroUrOrRVkdjI4eTIkQ5wAboFxiDOs0phy3JEjR47gSKpjoUrOrZVX2Mng5MmQDHECuAXGIc6wSmNKXQqGQ4cOSW9vr0xOTnre7OzslMHBQWlqaor07tTUlPT19cnw8LC0tbUFlq2Sc\/FQT2eBnUw6bnYtMiRDnABugXGIM6zSmFJ3gmFubk4GBgZkzZo10tXVJebvlpYW6e\/vD\/WuKbdv3z4ZHR2lYMCfg1ALVXpAcsQUaZoMcfJkSIY4AdxCleKw7gRDkPvGxsZkYmIiMsuwd+9eGRoa8qozw4A\/BFEWqvSA5Esq3DoZ4uTJkAxxAriFKsVhQwgGncK48cYbpaenxxMNFAz4Q0DBQIb5EsCtV6mjxmmks0CG6bjZtarEsO4Fg1nP0N3d7U1RhGUg9N9XrVoVaw0DHiK0QAIkQAIkQAJHCUxPT1cCRV0LBrMuQT0RtuhRFzrec889cv3118vMzIxTMFTCq7wJEiABEiABEsiYQN0KhjhiQVnpFMS6deukvb1d4uySyJgvzZEACZAACZBAJQjUpWCIuzPCv\/3S9tjOnTs9EcEPCZAACZAACZCAm0BdCgbNGszOzsY6e8FGwAyDOyBYggRIgARIgASCCNSdYAjLGqxYsUJGRka8w5v0nAbdEeHPIFAw8CEgARIgARIggXQE6k4wpLtN1iIBEiABEiABEkAIUDAg9FiXBEiABEiABBqEAAXDDx2t6yJ27Njh\/cUFkdHRr1M7Gzdu9NaRuN7jYXPV47ujjuVukGfOu80kDA0X\/7HojcQr7F6TcPRPZ\/I5P0o1CUO7LJ\/neE+geW6DpsnjWShPKQoGETHHRusaiGeeecbbiqn\/v7m5uTyeKklL7EFr\/fr1C97r4W+i\/8hu\/Xv37t0NzzYJQ5up8tu6dats27Yt9JCykoRJIc1IwtG\/s4rrmY66KAlDI7j0nT26PozPszvMDd\/x8fFK\/BClYPjhWQ3qen0QqqQG3eGcvIS\/o1WxtWvXrlg7VthJH\/tFZ781NQ5D7ayvu+46OXz4sESdaprco\/VbI0ksatlbbrlFbr31Vv4QsFyelKEdt3ye42ViV69eLQcOHPDGl3rfyt\/wgiHs7ZfmbZj1253m03I7G6MZGP\/fUVdlB3OUThqGmvW6+OKL5Wtf+9r8m1rz8XD9WE3CMc4L6urnzrNraRKGQRkG10v\/smtp\/Vl68cUXvUbrzr3e3l4Khvpz4eIWB2UUtHNetmwZ074BDvb\/Gk7yyy3t+RlViDP7HpIyNMebb9682XuJGsXsMeFlZ7eiYlEFw\/79+72KXKt0LBqTxqKdYt+0aZM3CPITTcAvtOqZFzMMc3OLzm2gYAgP6aQdjLGkHfYdd9zBRY8\/zDDEHei0g\/7Sl74kV111lbS2tkauGannjihN25PEoln\/YRY6at0tW7Y0fDwmYWgWPG7fvt1LrSfJLqbxb1XqUDBUxZO+RT\/6tkuuRI92bpIUJsVCMMskDLXsww8\/vGB9DTMMxzIM9gLlqAHMPyXB55wMixrGKBiKIl3QdeyMAhc9RkP3p31dC\/a4knoxzyQM7W2ptiWmg49uB7QXMhsmUYkAAAWJSURBVEbFov87PudHoykJQ4qudAMSBUM6bqWtxW2V8V2TZBsW077BXJMwtC3wV\/FCnkk4+jttptOPskzCMGhKgtM67r6TgsHNqO5K8OCm+C6LOujFLC7TxVBhv455YE70YTk2QwoGd8Yr7BAxP0f74CYeOnSMa9znWWuo0NqwYYNXmQzj9ZkUDPE4sRQJkAAJkAAJkEBFCDT8LomK+JG3QQIkQAIkQAK5EqBgyBUvjZMACZAACZBANQhQMFTDj7wLEiABEiABEsiVAAVDrnhpnARIgARIgASqQYCCoRp+5F2QAAmQAAmQQK4EKBhyxUvjjUjgueeek6VLl8Z+K6Juu3rttdfk3HPPzQ2X2eK6YsWKRK8Xr5cXhpn7K2qrX9HXyy0waJgEEhCgYEgAi0VJwEUg6YFARezRRgZ9pK6LVZbf6wCunyJfhlQvbLLkTFuNTYCCobH9z7vPmEAZBUPSNtlI6mVQpGDIOJBpjgQCCFAwMCxIICEB+8RArWrS\/M8888z8KXj67+ZES\/+JlyZtftJJJ0lvb69MTk56LTDvh7BfIWzbb25uDm2pfQ07LW\/e0mgqbtu2LfC17WHljGBYv3693HTTTZ4Zf9rfPv3Pfx1ldd1118mll17q1TesXn31VTEnNGqdL3zhC3L\/\/ffL8PCwtLW1eWbC7ikIgl8w6N9vvPGG97\/x8fEFfIPq+9+ToGWC\/q0exVTC8GZxEgglQMHA4CCBBASC3udgD1b+X\/NhL+zRSw4ODnpn+ato0FS6vjLYiJHu7u75gT3qBV6mPcZeU1OTN9DZrxJ3ZRj85e13Bqio0YF99erVXnuN\/d27d3trIXTg7+vrWzDQ2\/aMKDrzzDPn6\/vv0fz98ssve6+bNq\/xVmFiphhc7yUJEgw7duwQI5CCuNpup2BI8BCwaMMSoGBoWNfzxtMQcA08rsHZ\/8vVLxiC3rgY9dKpoCkDf\/moNrleaOV\/4ZC23zVNYX9vBINfAE1MTMwLCLVpCwL9234LpfFT1LRDkGCYnZ1ddA37ddgUDGmeANZpZAIUDI3sfd57KgJ2+t6\/6yBscPan7Ts7OwMzDP6pAbuBQdMJYdezX7wUJRhciy6DxEHQv\/mnafzTLiaDovcTNPDbNjVrYV5w5HdQ2Gu9gwSD1rUXQUYJHWYYUj0KrNRgBCgYGszhvN3sCNiDpL2Owf4VawSAf12B+YXtzzBoXf8v46gWh4mBqGkS2x4qGNSWWYtgBE1QhiGJYHjiiSfETHlErduw74OCIbu4piUSCCNAwcDYIAGQgD3oml\/QOr+v8\/0DAwOyZs2aBQsNbVHgFwxR6xWCmlnElIR\/jYJ9TR3co6YXzJSELRiCfs3bUxKaYdiyZYu3nsEsgHS5KI8pCZd4c03NuNrM70mg3ghQMNSbx9jemhIIWsNg\/8q3FwEGLd4zGQczJaE3Y4sKY18XQJp0etA6AgMhq0WP9i96e13DqlWrFi1q9AsGu65pq7ZPB\/wgwRB30aPaMAstXWtH0EWPZsrI7Gwx92Ev9vQHHgVDTR9FXrwGBCgYagCdl6xvAmYw0akD\/djTDfaWSE3RX3bZZQu2TqpQuOKKK+SGG26Y\/wXtFxEm62C2W+o1zEAWRi5qC2LchZhbt26dNx80vWB+7fsHSv+1t2\/f7q1T0IWO5v7tDINexM\/Qv63Sv7VU64RtCTVZHf2vEVlB2yrt+kGDvb1+RP20cuVKeeqppzzR4hd25h782Zf6jmy2ngSiCVAwMEJIgARqTkAH8KCdEXEbFmcNQ1xbccsxwxCXFMtVhQAFQ1U8yfsggToh4F+nYbIJ9rkLSW+FgiEpMZYngeQEKBiSM2MNEiABkID\/9Muw7ZJxL+N\/GdS9997rVc3r3RJ8+VRcz7BclQj8f+bg\/pXUcEZpAAAAAElFTkSuQmCC","height":316,"width":524}}
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
%[output:08c57868]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber","value":"     5.377777777777778e-09"}}
%---
%[output:863cd5b9]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rsnubber","value":"     1.549586776859504e+03"}}
%---
