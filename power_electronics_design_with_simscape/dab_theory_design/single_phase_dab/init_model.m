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

model = 'single_phase_dab';
use_thermal_model = 0;
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
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/2) %[output:3409cce6]

f0 = fPWM_DAB/5;
Cs = 1/Ls/(2*pi*f0)^2 %[output:317c178c]

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
Vac_FS = V_phase_normalization_factor %[output:4b1b289a]
Iac_FS = I_phase_normalization_factor %[output:6c8d3181]

kp_afe = 0.15;
ki_afe = 18;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);
res_min = s/(s^2 + 2*delta*omega_grid_min*s + (omega_grid_min)^2);
res_max = s/(s^2 + 2*delta*omega_grid_max*s + (omega_grid_max)^2);

Ares_nom = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Aresd_nom = eye(2) + Ares_nom*ts_afe %[output:345f228b]
a11d = 1 %[output:9e69d693]
a12d = ts_inv %[output:20880618]
a21d = -omega_grid_nom^2*ts_inv %[output:266aaba1]
a22d = 1 -2*delta*omega_grid_nom*ts_inv %[output:7a235d1a]

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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:53464e72]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:2b42b309]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:19de2814]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:5159e62b]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:85434b31]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:613a9f65]
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
figure;  %[output:6480ce36]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:6480ce36]
xlabel('state of charge [p.u.]'); %[output:6480ce36]
ylabel('open circuit voltage [V]'); %[output:6480ce36]
title('open circuit voltage(state of charge)'); %[output:6480ce36]
grid on %[output:6480ce36]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink
heatsink_liquid_2kW; %[output:8982d8ac]
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
%   data: {"layout":"onright","rightPanelPercent":12.4}
%---
%[output:0994022b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"275"}}
%---
%[output:60f8fa48]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"1875"}}
%---
%[output:3409cce6]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"5.9186e-05"}}
%---
%[output:317c178c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"1.8576e-05"}}
%---
%[output:4b1b289a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"563.3826"}}
%---
%[output:6c8d3181]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"381.8377"}}
%---
%[output:345f228b]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.0000","0.0001"],["-8.2247","0.9987"]]}}
%---
%[output:9e69d693]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"1"}}
%---
%[output:20880618]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"8.3333e-05"}}
%---
%[output:266aaba1]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"-8.2247"}}
%---
%[output:7a235d1a]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"0.9987"}}
%---
%[output:53464e72]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.0311"],["1.6193"]]}}
%---
%[output:2b42b309]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.1253"],["30.8294"]]}}
%---
%[output:19de2814]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.0001"],["-9.8696","-0.0016"]]}}
%---
%[output:5159e62b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.0156"],["2.7166"]]}}
%---
%[output:85434b31]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.0000","0.0001"],["-8.2247","0.9987"]]}}
%---
%[output:613a9f65]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.1296"],["22.6384"]]}}
%---
%[output:6480ce36]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10XsV55x8aGluEECMQEpLjmIIMNNsK26U2hoQlhPp0W5lu0lSSe461rpJ6Fwjugi3JJWGXQrAlG7YuH1uXKqrcrT+SLiTotKc04RAaEPFJHKztpgRkim0sIaNgHJJgO6Hxnud6R4yu7sfMvPO8975X\/3uODljvzDNzf8+8M38983XGqVOnThEeEAABEAABEAABEKggAmdAwFSQt1BVEAABEAABEACBgAAEDBoCCIAACIAACIBAxRGAgKk4l6HCIAACIAACIAACEDBoAyAAAiAAAiAAAhVHAAKm4lyGCoMACIAACIAACEDAoA2AAAiAAAiAAAhUHAEImIpzGSoMAiAAAiAAAiAAAYM2AAIgAAIgAAIgUHEEIGAqzmWocF4JHD16lDo6OoLq9fX1UXV1tVhVR0ZGaPXq1bR48WLatGkTVVVViZWlGy7nO5q8kOLw2c9+llpaWkyy5DpNT08Pbdu2LajjmjVrqKury6q+u3fvpg0bNtDGjRtzyYPf71vf+pb498MKGhJXLAEImIp1HSqeNwLlHNzjBAwPENdeey0tXbpUBE\/cO0qXG\/cyLgMiD6BPP\/20lThwyWPrAC5j5cqVk9mKKGCKJjhtfYz0fglAwPjlCWsgkAmB48ePU3d3Nw0ODtKOHTvKJmA48lOOcqOgqsGwubnZWIyoCIWNOHDJ49IIlICxqVu4nLxHYFQ7PXToEKIwLo0EeaYQgIBBg8glAT2UzhXUQ+J69OGKK66gu+++O3gHHsj06RQVLRgeHp72uW7jxhtvpE9\/+tNBmvr6eurv76fGxsZILrpQCKcPRyf4czWldP3119P9999PTU1NQcetD\/xpdngqSpW7d+\/eoH78qCmkO++8k\/7kT\/4kEC\/qCbPg3yumusBRg6aeXg2CypY+oOrv+OCDD1Jvb29kuYcPHw7qNzY2Nlkn3Yc6R2b+0EMP0Re\/+EVS78f8w6wVOzU1FzVYK7\/q5ar3Db+X8vXcuXMnRViY3+OPPx5MyahHbx9he2nCMdwek2wltcOkSI3OhOus6h4WReHvl85W2bjtttvoySefJP7+KN\/p78y\/U2Xovk3jEtUOc9kJoVK5JwABk3sXzawKhgct\/e1VJxw1SIUHHrbD4kGJl\/DnUQNs0uDPn8XVTQ0255133pQ1MErA6HVgoRAlOHQRE7bjS8BE\/YUfHkzCA1scV\/59nIDp7OykW265ZRr7JMHAn9XU1NDExEQg0KJEBZepD7Thuse1C1Xud7\/73Ugx8uijj06uO9Hbmz5AhwVM2Jb6PE7EJLVZznPw4MFYoaTXKSxewiJTiYePfOQj9M1vfnNK5xElQqK+X2EBwmmi6si\/V+Wk2da55D1KNLN63Mp+WwiYyvZf4WqvOmj9L1DV+fPL6tEH\/itbdYz6AKF3tvpfnvqAxyJBRQiUDVV2+C99BTnqc70zvuGGG2IFjP4Xqq2dNAHDUSd+0qZykiJEHBV64403pjHRowbMacGCBVPe0WQKKTy9pdgrf3K0Jex3rguvB4mKDDHLFStWBO+rR2xMFjabTAeF04T\/rZgoscX1Tytbtb2o91G\/Y6HL7xw3haRzVO0p7NOvfe1rgRCKiqjE2Q1H4VTUSbeRVLaK0Kj2n8bFx1RZ4To+vJATAQgYJ2zIJEUg7q8zNQBwx71o0aLIHTh6mgMHDkT+Vc311m3wX\/1qx1DaIty0vxzjBILeoXP5tnZ8CRi9bBYj\/OgDZtzAog\/gn\/nMZ4wFTFTEKqpcrkd4iiwuwsFpeSBW9dDZRpUXnkpLEjBxU2fhPEnRlCjxG343NT0ZFkJKtMUJjbT2qftXtxHn13A0R7FSAiZu6lDfYae3ZfW91KfvVD+hc4matpTqT2C32AQgYIrt34p7u3ILGH0bctoAYSs8GH7UtmpTO\/rgHB7s2La+jdokAsNp9IWv\/G\/eshuOQIUHUFsBE+YYjtKEhZMvAaMae5xw4p1ZUQImPBWVFoGpBAETFfFTfg23v7gIjG4j7rsBAVNxXWyhKgwBUyh3Vv7LuE4hhac61JqCuL9mo0L+aQImaepHjwqwF\/iv1DgBY2qHQ\/NhcaGm1sIChkWCyeLI8OCuRyjC03A84KdNIXF0KE0AhG24TiHprTsuqhH+BoTFSDgaod5Zj8Sp91FtJ5wnagop7ZsnPYWkxK6KXMUJmKjIlWIUjsDELboOT18lTSFFccEUUlprweemBCBgTEkhXVkISC\/iTRIAaQImqW5R60PiBEyaHQ63q\/UsYegmAobzRO1CUrbCO0n0A+BsFvGqqQQ9D5f7iU98IogORT3MKer9TBfxsk0l6sLCKW6Bq55HT8Nlbt26le65555pC445T1jA8O\/iFgSrd00TzFHTK2kRMJ1j3DsmiQ9dMNx6662xbSvJBtchanGv6SJenUtaBLIsHQ0KKQQBCJhCuLF4L2G6jVrfAp22jTpqYbDNFBJTTpqeSFskq5\/Mm2SHywlvuf385z9P+\/btm1y0GhWB0Qe3uIXIuu3w2pwogaMP5Hpe\/n8lYKLKfeSRR6acKMuH6+nrbVy2UetCRB9Qo7bYx23fDnPV1+SwTea2ZcsWWrduXYBDj6Sp3WRx27LTzm9J2kbNZZlGJuLWrnAULkocxEWdmFHUFvaoKE6c+OXfh0\/+jVtLpGyYRAqL16PhjSQIQMBIUIVNUQJpOz5EC4fxkglETVWFd5rFncOjF+5ykF3JlZ+hBnTBqYRa1M6kNDw4yC6NED63IVAoAcMdGp9BwYdrRXWASQeb2UBD2mwJQMBky7\/U0pOm0JKmvqLKdblKoNT6z9T8UVNIzCLt8Mco0VmUu6tmalvIy3sXRsCkLepTny9btiy45Ez9m798them5cV5M7UeEDCV7\/nwHxP8RrbihfPgbp3ytoXw1K6NeOGaQnCW119FL60wAobnefnLwU9cBCbsTP6LYmhoqKy3+Ra9QeH9QAAEQAAEQKAcBAohYPivubvuuova2toCEeNTwPCR8\/yDBwRAAARAAAQqlQBfz8E\/RXoKIWA4ksIPnwSZtAZGd5wKYbe2tgZTSlEPC5f169fTnj17iuRzvAsIgAAIgMAMI7BkyRLavHlzoURMxQsYngMfGBigO+64I4iUmAgYtf6F269+e3G4Pavth+z0hoaGGdbcZV+XRSGfwQG2\/jmDrX+myiLYgq0cATnLqt2m3ZguVwMZyxUvYHjKiM+Y4FND03YhMUJT8cJplYApmtNlmpKdVbC142WTGmxtaNmlBVs7XjapwdaGll3aorKtaAETtZNBuTVKdNjuPCqq0+2avkxqsJXhCuEtxxVswVaWgJz1ova3FS1gwu5Oi8BwtIZPn0yaNtJtFtXpcl8Tc8t8WzRP\/bW3t9P8+fPNMyJlKgGwTUXknABsndGlZgTbVETOCYo6lhVawKiIC+9OWrBgQXAzsDoOXLWEpCPXi+p052+Bx4wnTpyg1157jS688EKaPXu2R8swBbZybQBswVaOgIzlQ0dP0IqHn6e3vvj7k\/eIyZRUfquFEjC+8UHA+Cb6rj0MBGArR0DOMtot2MoRkLHMAuaKe56jOV\/pgICRQZxPqxAwcn7BQAC2cgTkLKPdgq0cARnLEDAyXHNvFQJGzkUYCMBWjoCcZbRbsJUjIGP5mf3HgikkRGBk+ObWKgSMnGswEICtHAE5y2i3YCtHQMYyBIwM19xbhYCRcxEGArCVIyBnGe0WbOUIyFiGgJHhmnurEDByLsJAALZyBOQso92CrRwBGcsQMDJcc28VAkbORRgIwFaOgJxltFuwlSMgYxkCRoZr7q1CwMi5CAMB2MoRkLOMdgu2cgRkLEPAyHDNvVUIGDkXYSAAWzkCcpbRbsFWjoCM5Z3fHqebd76AXUgyePNrFQJGzjcYCMBWjoCcZbRbsJUjIGNZCZhz\/rGLdv3lnwUXHxflwUm8CZ6EgJFr5hgIwFaOgJxltFuwlSMgYxkCRoZr7q1CwMi5CAMB2MoRkLOMdgu2cgRkLGMKSYZr7q1CwMi5CAMB2MoRkLOMdgu2cgRkLEPAyHDNvVUIGDkXYSAAWzkCcpbRbsFWjoCM5Z4nXqGeJw5gEa8M3vxahYCR8w0GArCVIyBnGe0WbOUIyFiGgJHhmnurEDByLsJAALZyBOQso92CrRwBGcsQMDJcc28VAkbORRgIwFaOgJxltFuwlSMgYxkCRoZr7q1CwMi5CAMB2MoRkLOMdgu2cgRkLEPAyHDNvVUIGDkXYSAAWzkCcpbRbsFWjoCMZT6Fl3cizflKB+3YsQMH2clgzp9VCBg5n2AgAFs5AnKW0W7BVo6AjGUIGBmumVodGRmhzs5O6u3tpcbGxsi6QMDIuQgDAdjKEZCzjHYLtnIEZCw3P\/Q8PfvyMURgZPCW3+rx48epu7ub9u7dS\/39\/RAw5XcBYSCQgw62YCtHQM4y2q0MWwgYGa6ZWeXISk9PT1A+IjDZuAGdlRx3sAVbOQJyltFuZdhecc9zdOjoCcJljjJ8y2r16NGjdNddd1FbW1sgYkwEzNq1a2nJkiWT9ayrqyP+weNOgDur8fFxqq2tpaqqKndDyDmNANjKNQqwBVs5Av4sc9\/KPz8\/63z67b96NTCMRbz++GZmaffu3UHZixYtMl4DE65se3s7rVq1KrN3KELBJ0+epImJCaqpqaFZs2YV4ZVy8w5gK+cKsAVbOQL+LG\/fvp0GBgYCAfPWb5yebYCA8cc3E0u8cJedescdd9Dhw4eNBczmzZupoaEBERiPXuN1SEeOHAkiWbNnz\/ZoGabAVq4NgC3YyhHwZ1lFYHjx7hf2nv4DEQLGH99MLPGU0bXXXhvsg8cupExcMFko5rvl+IMt2MoRkLOMduufrbqJmi2f\/UwvfenP\/hvOgfGPWd4ir33p6Oig4eHhaYXFHe6DbdRyfkFnBbZyBOQso92CrRwB\/5bVDiREYPyzzdQiIjCZ4sc2akH8GGTl4IIt2MoR8G9ZCZhfePsHwS4knMTrn3EmFiFgMsGOKaQyYMcgKwcZbMFWjoBfy7x1mrdQ83PmD14MppAgYPwyzrU1TCHJuQcDAdjKEZCzjHYLtnIE\/FrW17\/csfgkPfT5myBg\/CLOtzUIGDn\/YCAAWzkCcpbRbsFWjoBfy2r6aF71bPrzj59BK1euhIDxizjf1iBg5PyDgQBs5QjIWUa7BVs5Av4s69NHV188hzgCAwHjj29FWIKAkXMTBgKwlSMgZxntFmzlCPiz\/NSLR+mT207vuN33uato7KV9EDD+8FaGJQgYOT9hIABbOQJyltFuwVaOgB\/LevSFp49YwBR1LDvj1KlTp\/xgK56Vojo9D57CQCDnBbAFWzkCcpbRbv2w3fQPr1DvPx4IjB29\/7rgv0UdyyBgEtpMUZ3u52tSmhV0VqXxS8oNtmArR0DOMtpt6Wyjoi8QMKVzrUgLEDBybkNnBbZyBOQso92CrRyB0iyzeFnx8PPE\/+XnobbLqe3KOkRgSsNaubkhYOR8h4EAbOUIyFlGuwVbOQLullm0PPDUIep7djQwwsKFBYx6ijqWYQopoc0U1enuXxN\/OTEQ+GMZtgS2YCtHQM4y2q0bWxYvf\/fPE3THV\/cHBtTCXd1aUccyCBgIGLdvTYm50FmVCDAhO9iCrRwBOctot\/ZsWbw8PfImrd39\/Vjxwh9AwNizrfgcRXV6HhyDzkrOC2ALtnIE5Cyj3dqxZfHS88QrxFcGqMjLg62X0zWXzJlmqKhjGSIwiMDYfWs8pUZn5QlkhBmwBVs5AnKW0W7t2PJFjWrBLk8bPX7TwmD6KOqBgLFjW4jURXV6HpyDzkrOC2ALtnIE5Cyj3aazZcHyjZeO0h996cXJxGniBVNI6VwLmQICRs6t6KzAVo6AnGW0W7CVI5Bs+Zn9x+iWXS9MRl04tb5VOil3UccyTCFhCimT7yMGAjnsYAu2cgTkLKPdxrNVN0urFBx16Vp+0eQ5L2legYBJI1TAz4vq9Dy4Cp2VnBfAFmzlCMhZRrudypani27e+QI9+\/KxKdNFNsJFZSzqWIYIDCIwcj1SgmV0VnLYwRZs5QjIWUa7pWB6SO0u0oULUzdZ6xLnHQgYuXabW8tFdXoegKOzkvMC2IKtHAE5yzO93YavAtCni5J2GJl4pKhjGSIwiMCYtH\/vaWZ6Z+UdqGYQbOXogi3Y+iTAC3P5LJdwtIXL4OsAeLoobmu0TT0gYGxolTHt8ePHqbu7mwYHB4NS16xZQ11dXbE12L17N23YsCH4vLm5mTZt2kRVVVWR6Yvq9DK6J7YoDARyXgBbsJUjIGd5prRbFi3\/+\/kjNPDc2DSYLFY+e908uuHy87wIF1VAUceyio\/A9PT0BD5i0XL06FHq6Oig1tZWamlpmdY42Imcvq+vLxAtLHzq6+tjBU9RnS7XBZlbnimdlTkRfynB1h\/LsCWwBVsXAixavrR3nP7Xntcis1998Zwg2hJ1iq5LeeE8RR3LKl7AhB2lC5rwZxx9GRoamoy6hP89U5zu4wtRqg0MBKUSjM8PtmArR0DOctHabdL0EFPkaAsf\/c\/\/9TFNlOQZCBi5duvNsorAcDRm6dKlRhGYZcuWRUZrOHNRne4NeAmGitZZlYDCe1aw9Y500iDYgm0cAV6E+9A3DtEjz4zGQiqnaNErUdSxrDARGI68bNu2LXVdy8jICK1evZrGxsZox44dkUJHOV45fe3atbRkyZLJ9lBXV0f8g8edAA8E4+PjVFtbG7sGyd36zM4JtnL+B1uwZQJjb70TbHf+4fGf0V8OjUcuwuV09eecSRe+\/0zqWj6f5lVXiUdalHe4b+Uf9YyOjtL69etTxzw578pYLoyAUXhYyLA4iVqcy1NGu3btCtbAVFdXB+th+Ilb9KsETBh9e3s7rVq1SsYjM8TqyZMnaWJigmpqamjWrFkz5K3L85pgK8cZbGcuWxYtO\/a9RS\/94Ke0d\/RELAgWLW1N59ClNe+lxQ3RlyvKUTxtefv27TQwMDCtmLQ\/2qXr5dt+4QQMR1g6Ozupt7eXGhsbJ3mp3Ur6lFFcWpVJCZjNmzdTQ0PDpC1EYEpvhuyPI0eOBJGs2bOz+ZKX\/hb5tAC2cn4B25nD9pn9b9KX9x6hLw+\/mfjSPC10zcVz6HcX1gQRFhYwWT\/hCMyePXto69atiMBk7Zi08vWdRhxlUU8pAqZoqjWNYTk+x1oCOcpgC7ZyBOQsZ9luecHtGz\/5Kf3lM6Ox00HqzVmk8K6htisvLMsCXB\/EsQbGB8X\/b0Mtth0eHray2tTURI899tiUPPo0kBIpcVujo6aQ4qabuJCiOt0KulDiLDsroVfKjVmwlXMF2FY+Wxuxwm\/LguXjl51Ht35sXvDy0juGJAgXdSzLZAopbbdQlANVZCUsYMIH2emH06nP2traJhfrqsW+XAYOspP4qpjZxEBgxsklFdi6UDPLA7ZmnFxSSbBlsfLWiXfofz79ampkRYmTD547mx5qu7xixUrc+Lly5UpMIbk0zHAenwLGR33ibBRVtUoyM7Ut0VmZll30dGAr52GwzR9b3g3EURH+7\/1fP0gvT7xtJVY6rmmg89\/33oqZDnLxQFHHskwiMBwZ4R99jYqLU6TzFNXp0txM7GMgMKHklgZs3biZ5AJbE0puaUzZclRl7Icn6a+\/NWYkVPTIyscuraYr539A7MRbtzeXz1XUsSwTAaOvgUm7u0jetfElFNXpWTJVZZt2Vnmoa6XVAWzlPAa25WH7+tsURFS+c\/CH9OT3j9Krb54I\/m3y6ItslXipxHUrJu9qmqaoY1kmAkZB19ej8MLb\/v7+KVufTZ0jla6oTpfiZWMXA4ENLbu0YGvHyyY12NrQSk\/LooR\/Rl5\/mx59\/gi9MvHj4JA4k4dFCa9X+WjjuXTVL80p9BSQCY+kNEUdyzIVMAp4eFdSXqIyRXV6qV8GH\/kxEPigGG0DbMFWjoCbZZ72CY7R\/8YheuG1nxhP\/agICv+30rYuu5GSyVXUsSwXAkZ3mX7UP\/8+yzNYiup0ma+InVUMsna8bFKDrQ0tu7RgG89LRVNOEVHvE69YTfuwVXUA3A0fvoD+6\/UfCiIz5bjo0K4FVGbqoo5luRMwevOIO5SuXE2oqE4vF7+kcjAQyHkBbMFWioBah\/LikZ\/QV\/a9HoiMZ18+ZlWcmvr5zX93Pv1qw\/snz1W54Cyi1157jS688EKczm1FND1xUcey3AkYRGDSG2MRUmCQlfMi2IJtKQRUJOWF8Z\/QvlffchYpatrn936tjt5zxhmp0RS021K8lpwXAkaObbCluru7mwYHB4NS0g6YE6zKFNNFdXq5+CECkw1pDARy3IvEltel8G3Kf\/5PhwNgtpEUzqOiKf\/l2g\/SObPPnIymuOz6KRJbuRboZrmoY1mmERh9FxK7Jcv1LlHNoqhOd\/sK+M2FzsovT90a2IKtmurh\/\/79\/52gfx79sfWaFEVRiZSWX6uj+edVlSRS8EeNXNtMslzUsSwTAaPvOspLtAUCprxfLAyycrzBdmaw5QgKPyff+Tn96ZMHS4qicGbekszTPRcJihQIGLm2CQFTJrb79++nW2+9le68887JO4rSio67CyktXymfF1W1lsLEV14Msr5ITrcDtsVhyyLlrPf+An1xaJQOvmG\/YFaPovD\/X39ZdbBw9rpLqychuUz3SBBGu5WgetpmUceyTCMwXV1dEDBybTbXltFZybkHbCuDrZrm4bt7Htv3Or3yg+MlTfOoKIo6Lp\/\/XUnbkNFu5dotBIxHtuGD60xNNzU1Ufg2atO8LumK6nQXFr7zoLPyTfRde2CbLVt1foma4nn+1bfo++M\/cdrNo7+JWovSeMFZ9ImFtcFHlSRQ0ryCdptGyP3zoo5lmURg3N1Q3pxFdXp5KUaXhs5KzgtgK8\/2Z7POnTyrZPD\/TNA\/fO8HQaEuO3lUbZVA4f+qBbP6Z3JvlQ\/LaLdyfijqWAYBk9Bmiup0ua+JuWV0VuasbFOCrS2x6PQcQamfM4seeOoQ7X\/9befpnbBAuej8Kvrkwlp6zy+kn43i500qwwrarZyfijqWQcBAwMh9axIso7OSww62yWz1W40fH36dnviXN7xET9gI7+T5pfOr6PYb5k8ehc+\/z8tCWblWV7pltNvSGcZZgICRY5tby0V1eh6Ao7OS88JMZquff\/L6j35KT37\/jUBIvPrm6VuPXR8lQJZ86GxacO4pal40j15\/m+iaS+a4mkS+EIGZ3G6lG0NRxzJEYBCBkf7uRNpHZyWHvYhswwtjv\/faj4PD2Q6+4b5zR5\/aUdGTedVV1HZlXfCREi169KSIbOVaop1lsLXjZZMaAsaGVkHSFtXpeXAPOis5L1QiW3X\/DlM5\/+xfpAe\/8aoXcaKECE\/tsBBZ\/svnU\/X7fjGA7xI9qUS2ci3Nr2Ww9ctTt1bUsSw3EZjdu3fThg0bAuZ8pcDBgwdpaGiINm3aRFVVVbGeDd+jtGbNGuLzZeIe5Uj+nLdl9\/X1UXX1u4c6zQSny31NzC2jszJnZZsyb2yVOGEBMXz4R\/TUi0dpxMOiWD1KcvXFc2jxh86hBRe8b8p6E99rT\/LG1rZt5Dk92Mp5BwJGji3xnUhjY2PU2dlJt9xySyBAWFzwBY\/19fWJgoTz8sN51Pkyra2t1NLSMq3G6qbrLVu2BAfosWhKEklFdbqgK41No7MyRmWdsJxs9TUno8dO0D+NvOllzYkuTliELPzgOdRxdUPmC2PLydba8RWeAWzlHFjUsSzzCIwSHSxAFixYQB0dHYEYYYGhrg9IipKEXa4LmvBnLFgOHDiQKIgQgZH7EumW0VnJcfbBVl9z8sHq2fTFZ0fpu4feCipdylkn6q1VZIQjJ3xybO05s8QuCfRJ2gdbn\/Upki2wlfMmBIwQW58CRrfFAkh\/1FTTsmXLIqMzUa9XVKcLudLKLDorK1xWiZPY6sKERcTeg2\/R1z3t1NHFiVpzcu2Calp60QcmIye+p3SswHhIjHbrAWKMCbCVY1vUsSzzCAy7TE3l6FNIKhoTNx0UFXnZtm0bxd1urQTM8uXL6ZFHHqHh4WHjNTBr166lJUuWTBZZV1dH\/IPHnQB3VuPj41RbW5u4xsm9hJmVc+ytd6j+nDOJ\/zvy2jH6xZPH6KnR99C+sdNbh31ETdg+Pxe+\/0xaUHsWffjCs4P\/8sO7d07\/d3ahwaPdyrkXbP2x5b6Vf9QzOjpK69evD9aXhv+491dq+S3lQsDwa+uLaxWGjRs3GkdLVB61nia8+FcJmEOHDk0u3I1Lq2xF1Yk\/a29vp1WrVpXfWwUq8eTJkzQxMUE1NTU0a9asAr2Z31fRhclrP3onEA\/\/8vpJeubA8UCs8O\/4v6U+uji5\/IJZ9NGLTgsSLo8f9Xmp5VR6frRbOQ+CrT+227dvp4GBgWkGIWD8MRaxxAt1OZLT29tLjY2Nk2VETSHFpQ0LmM2bN1NDQ8OkLURgSncd++PIkSNBJGv27GL\/1R5FS03lnN6hczyIYLzxk58Fa00OvHHcmzBRERE1pfPr8z9AHzr3vUGV+DMIE7u2PNPbrR0tu9Rga8crKXU4ArNnzx7aunUrIjD+EMtYSlr4yxGX+fPnT0Z1WMDce++9dN9990VupS7qvKEMeTurRZ3vnipMTgQi4af\/9nN68KlX6eWJtwNIPqZzlABRwuTD9WfTb\/9KTbDW5IKzKIhuLb70gzNSHNq1RLvURW23dhRkUoOtDFe2WtSxLPMpJLXwltekJD1x57vou45UlCVu63VY3CTtWCqy0+W+JuaWK7Wz0s80+fmpU8GBay+O\/8S7MGGDLE5+de77ac1H5k4eg8+CKG2dSaWyNW892aUEWzn2YCvHFgJGjm2wiHfXrl1TDpXTz3RZsWJF7Jkw4YPs9EW86rO2trbJhUv6upa4Bb\/qVYvqdEFXGpvOW2elnwQ799xZ9PA3XqUXBIXJxTVVdMvCKQ3sAAAgAElEQVS\/n0fjb\/00YGYiTEzh5o2tab0rIR3YynkJbOXYFnUsy00ERp39ortQj5i89NJLwYF3jz32mJyXQ5aL6vSyAUwoqFydVViYPPSNV+n7gsJk\/nlV9JmPNNBbx\/\/NuzAx9Vu52JrWp0jpwFbOm2Arx7aoYxkETEKbKarT5b4m5pZ9dFbP7D8WFFiOiAkLk\/90VT0d\/9nPMxMmpnR9sDUta6alA1s5j4OtHNuijmWZCxh2WdoUEl8LoM6K4ZXU5XqK6vRy8UsqJ+2wNbUYtn9olL5z8PQJsK++yTt2Tp9rUsqj78xhYdJ+VT2dqABhYvrOGAhMSdmnA1t7ZqY5wNaUlH26oo5luRAw7I6oM1fUnnUWLw888AD19\/dP2Rpt70a7HEV1uh0Ff6n1BbDf+dc36O+GX6OxH58Ktgz7FiYsUv7g6gY6\/tP8R0z8ET5tCQOBb6Lv2gNbsJUjIGe5qGNZbgSMnOvcLRfV6e5E0nPytE7DnFn04DcO0UtH3vYSNQmfZfL7v34h\/fzUu6e+pu3KSa91sVJgkJXzJ9iCrRwBOctFHcsgYBLaTFGd7vo10W8e5luHn\/vXYyULFF2c\/McrLqAFte+riEv9XBmWIx8GWTnKYAu2cgTkLBd1LMuFgOED5VavXk1jY2PTPNjU1DRle7Wci6dbLqrTTRhyJOXUqVO06zvjwfSOy+Fr+o3Dv\/ahD9D1l1UHtq65ZA6mOUyc4JgGg6wjOINsYGsAyTEJ2DqCM8hW1LEscwGjH\/Gvznvhc1vUZY5R26sN\/OUlSVGdrsNRURU+xv7Ox\/dbCRUlUD52aTUt\/+Xz6PILz540nTatg87KSxONNAK2YCtHQM4y2q0c26KOZZkLGHVgnRIq+nH\/DH3nzp0UvphRzs1TLRfR6Woh7Tf3v0lDLx8zEiwsRq6+eA5xFKXxgrO8HLqGzkquFYMt2MoRkLOMdivHtohjGdPKnYDhHUcHDhwgFjRJ9xrJufpdy0VxuhItPU+8kihYWKjw8fUfu6yaPrmwNgCRFklx9QM6K1dy6fnANp2RawqwdSWXng9s0xm5pijKWBZ+\/8wFDFdIv5NIFy1f+9rXaGhoCBEYx1bL61iSRAuLk49fdh79zhUXeImq2FQTnZUNLbu0YGvHyyY12NrQsksLtna8bFJDwNjQskyrr4PhQ+tY0Gzbto34UsZyn\/2iV70Snc7RFhYtO789Ps0Laiqoa\/lFotEVE\/ejszKh5JYGbN24meQCWxNKbmnA1o2bSa5KHMtM3isXERiTimaRppKczsLl5p0vTJsi0kWL1HSQi2\/QWblQM8sDtmacXFKBrQs1szxga8bJJVUljWU275e5gAkv4g1HQDga09fXR9XV1Tbv5SVtJTg9Sbg8ftNCsTUspQJGZ1Uqwfj8YAu2cgTkLKPdyrGthLHM5e0hYBKo5dnpLFye\/P4bdPvfvjTlDTjKkmfhoiqLzsrl62qWB2zNOLmkAlsXamZ5wNaMk0uqPI9lLu+j8mQmYHi30YYNG1LrvmbNmmBHUhZPXp3O4mXFw89PuT+oUoQLBIx8S8ZAIMcYbMFWjoCc5byOZaW+cWYCRlU8aQqp1JcrNX\/enH56O\/RxWvHwvslXY+HyYOvlwem2lfRgIJDzFtiCrRwBOctot3Js8zaW+XrTzAWMrxeRsJMnp7N42fnt16jniQOTr\/pg62W08tcvlHh1cZvorOQQgy3YyhGQs4x2K8c2T2OZz7eEgEmgmRenh6eMOOrCW6Hbrqzz2RbKagudlRxusAVbOQJyltFu5djmZSzz\/YaZCBg1bTQ8PJz6PjP9Msco8VIJi3TTHIvOKo2Q++dg684uLSfYphFy\/xxs3dml5YSASSNUwM+zdnqUeNn3uasKQRqdlZwbwRZs5QjIWUa7lWOb9Vgm9WaZRGB8vow6xXdwcDAwa7praWRkhDo7O6m3t5caGxsjq5Sl08PihS9THLx5oU90mdpCZyWHH2zBVo6AnGW0Wzm2WY5lcm+Vg8sc1ctFbaveuHEj8dUCSY9+j5KammptbU3Mp0TP3r17E68qyMrp4cPpeM1LUSIvypforOS+1mALtnIE5Cyj3cqxzWosk3uj05ZzEYFh8bJr164pJ+6aipEwIF3QxMFTF0by53mMwNz39YP0hb\/\/16D6RRQv\/F7orOS+2mALtnIE5Cyj3cqxhYARYuvzKgGTM2U4zV133UVtbW3BpZEmAmbt2rW0ZMmSSQJ1dXXEPxLPl5+foLVf3h+Yrj\/nTHr0D38lt9cBlPL+3FmNj49TbW0tVVVVlWIKeUMEwFauSYAt2MoR8GeZ+1b+Uc\/o6CitX7+eduzYQUuXLvVXUMaWMo\/A+BIw6gbr5uZm2rRpU+ygyNEefhYtWmS8Bibso\/b2dlq1apV314299Q41DxyeFC\/\/\/ePn0+KG2d7LyYPBkydP0sTEBNXU1NCsWbPyUKXC1AFs5VwJtmArR8Cf5e3bt9PAwMA0gxAw\/hhPWvI9hTQ2NhYpYnjhLjv1jjvuoMOHDxsLmM2bN1NDQ8NkfSUiMLzuZe2XR2jPwR8H5Wz91CX0qYU1ArTzYZLXIR05ciSIZM2eXUyRlhVpsJUjD7ZgK0fAn+VwBGbPnj20detWRGD8IZ5qyXURb7g+SbuLOEpz7bXXBiG0vO1C6nnilclTdou24yiqzWC+W+qbhPVFcmTBFmwlCcjZxhoYObZeLasFun19fVRdXT1pO+nwvLiwWjmdXn3bU0FdK+1SRlfnQcC4kkvPB7bpjFxTgK0rufR8YJvOyDVFOccy1zq65MvNGpi0rc9xL6fvOlLbo+vr61NvsM5TBKb5oefp2ZePBa\/I26VZxBT9QWcl52GwBVs5AnKW0W7l2ELAyLGl8PSRzUKj8EF2+iJe9RnvOAqvvM6LgNn57XG6eecLAd2ZMHWkmhE6K7kvFNiCrRwBOctot3JsIWDk2E6xrHYT8S85ktLf3x97Uq50lcrhdBV9mSlTRxAw0q0W6zQkCWOQlaMLtnJsyzGWydU+3nLmU0hJL81ihsGH17OUC5S003nn0RX3PBe8zmevm0d3NV9crlfLvBx0VnIuAFuwlSMgZxntVo6t9FgmV\/Nky7kTMHoEJsubqBmbpNP1u45mWvSF2aKzkvvKgy3YyhGQs4x2K8dWciyTq3W65VwImDxNG+nIJJ3+zP5jtOLh54Pi\/senLqX2q+rTvVWgFOis5JwJtmArR0DOMtqtHFvJsUyu1umWMxcwJsf\/p7+GTAopp+uXNc7E6AsiMDLtVVnFQCDHF2zBVo6AnGWpsUyuxmaWMxcwZtXMJpWU0\/WdR72faKRPXzM3mxfMsFQMBHLwwRZs5QjIWUa7lWMrNZbJ1djMMgRMAicpp8\/UnUc6anRWZl9Ql1Rg60LNLA\/YmnFySQW2LtTM8kiNZWaly6WCgCmzgNHXvlxzyRx6\/KaFct7NsWV0VnLOAVuwlSMgZxntVo4tBIwc29xalnD6TDx1N8rB6Kzkmj3Ygq0cATnLaLdybCXGMrnamlvOPAKTtIg37l4j89crLaVvp+vnvvDiXb42YKY+6KzkPA+2YCtHQM4y2q0cW99jmVxN7SxDwJRxCkkXMDPlzqM4vOis7L6oNqnB1oaWXVqwteNlkxpsbWjZpYWAseOVmjp8\/1FchjVr1qRezJhamGMC307XF+\/O5OgLuwOdlWOjNMgGtgaQHJOArSM4g2xgawDJMYnvscyxGt6z5ToC4\/1tLQ36dLoefZlJlzYiAmPZ6Dwkx0DgAWKMCbAFWzkCcpZ9jmVytbS3nLmAsa9y+XL4dHrPE69QzxMHgsrzziPegTSTHwwEct4HW7CVIyBnGe1Wjq3PsUyulvaWIWASmPl0OqaPpoJGZ2X\/ZTXNAbampOzTga09M9McYGtKyj6dz7HMvnS5HJkIGH3n0YIFC6ijo4OGh4cj3zLLCx19OV2fPmq7so4eartczqMVYhmdlZyjwBZs5QjIWUa7lWPrayyTq6Gb5UwEjFtVy5\/Ll9P16aOZvvtIeRGdlVx7BluwlSMgZxntVo6tr7FMroZuliFgyjCFhOmj6ZDRWbl9YU1yga0JJbc0YOvGzSQX2JpQcksDAePGLTWXmk4q6hQSpo+imwA6q9SvhnMCsHVGl5oRbFMROScAW2d0qRkhYFIR+U3Awub222+nP\/7jP6bGxka\/xg2t+XD6M\/vfpBUP7wtKxO6jd8GjszJshA7JwNYBmmEWsDUE5ZAMbB2gGWbxMZYZFlXWZLmeQmLoO3fupE2bNlFVVVUkmOPHj1N3dzcNDg4GnycdfBeO9jQ3Nyfa9uF0TB8hAlPWbzQOCRTFjUFWDi\/YyrH1MZbJ1c7dcu4FTE9PD\/X19VF1dXXkW\/Ln\/HR1dZESKK2trdTS0jIlvRI6y5YtCz5T\/66vr4896bdUp+PwuviGic7K\/UublhNs0wi5fw627uzScoJtGiH3z0sdy9xLls2ZawHD4mRsbCwxShLGowuaNHR8ncHQ0FCs\/VKdrgsY3jrNW6jxnCaAzkquJYAt2MoRkLOMdivHttSxTK5mpVnOXMAkLeLl6Eh\/f7\/xGpikm62jMJkKmLVr19KSJUsmTdTV1RH\/pD2f6nuBnn35WJDsW+sWEt9AjeddATM+Pk61tbWx04Ng5UaABwKwdWOXlgts0wi5fw627uzCOfn7zz\/qGR0dpfXr19OOHTto6dKl\/grK2FLmAobfP256R033mDDiyMu2bdsobV2LspU03aTSKNUaLr+9vZ1WrVqVWq3FD5y+OqD+nDNpsH1uavqZlODkyZM0MTFBNTU1NGvWrJn06uLvCrZyiMEWbOUI+LO8fft2GhgYmGYQAsYf40lLUVNFJgIjqiom005KMHH+pAXCSsBs3ryZGhoaJoszicDw9NHSLc8HeW6\/fi7dfv0HBchVrkn2wZEjR4JI1uzZiEz59CTY+qQ51RbYgq0cAX+WwxGYPXv20NatWxGB8Yf4tKWkaR+TXUjh+oyMjFBnZyf19vZGTj2Zihe2W8q8IS5vTG4pmO\/2\/U161x7Ygq0cATnLaLdybEsZy+RqVbrlzKeQ0gRM2i6kMAJ2VFwek51Hur1SnI7t0xAwpX893SxgIHDjZpILbE0ouaUBWzduJrlKGctM7GeVJnMBE17\/ooNIW2TLafVdR2kCxWR6yYeAwfbp9OaMziqdkWsKsHUll54PbNMZuaYAW1dy6fkgYNIZOadguOvWrZuy44inglavXk1btmxJXDUdPshOX8SrPmtra6O4W6+Tbrt2dfoz+4\/RiodPr3\/B9unoZoHOyvnrkpoRbFMROScAW2d0qRnBNhWRcwLXscy5wDJlzDwCo94zasdP1iumXZ2O9S\/prRedVToj1xRg60ouPR\/YpjNyTQG2ruTS87mOZemWs02RGwGTLYbo0l2djvUv6d5EZ5XOyDUF2LqSS88HtumMXFOArSu59HyuY1m65WxTZC5g9GmevB2w4+r06tueCrx69cVzaPDmhdl6OKelo7OScwzYgq0cATnLaLdybF3HMrka+bGcuYCxPT3Xz2ubWXFxur6At7\/9w3Rj0wVmhc2wVOis5BwOtmArR0DOMtqtHFuXsUyuNv4sZy5g+FVMdhv5e2VzSy5O1xfwPn7TQrrmkjnmBc6glOis5JwNtmArR0DOMtqtHFuXsUyuNv4sZy5gku5C4tdM2iXkD0O0JRenq\/UvbHHf567C\/UcxTkJnJdd6wRZs5QjIWUa7lWPrMpbJ1caf5cwFjL9X8W\/JxelYwGvmB3RWZpxcUoGtCzWzPGBrxsklFdi6UDPL4zKWmVnONhUETAJ\/W6fjADvzxozOypyVbUqwtSVmnh5szVnZpgRbW2Lm6W3HMnPL2abMRMDoC3fjDphTWCppCkkXMF3L51PX8ouy9W6OS0dnJeccsAVbOQJyltFu5dhCwMixza1lW6frB9hh\/UuyW9FZyTV7sAVbOQJyltFu5djajmVyNfFrOZMITPgVwvchJd2P5Pf1k63ZOl1fwHv0\/uvKWdWKKwudlZzLwBZs5QjIWUa7lWNrO5bJ1cSv5VwImKhLFtU0U2trK7W0tPh9a0Nrtk6\/4p7niKeR5lXPDnYg4YkngM5KrnWALdjKEZCzjHYrx9Z2LJOriV\/LmQuYpIPsGPrOnTtp06ZNVFVV5ffNDazZOB0LeA2AaknQWdnxskkNtja07NKCrR0vm9Rga0PLLq3NWGZnOdvUuRcwHJ3p6+uj6urqspOycTpuoLZzDzorO142qcHWhpZdWrC142WTGmxtaNmltRnL7CxnmzpzAZO03iXrE3ptnI4bqO0aMjorO142qcHWhpZdWrC142WTGmxtaNmltRnL7CxnmzpzAcOvz3DXrVtH\/f391NjYGBAZGRmh1atX05YtWyirSx5tnI4FvHYNGZ2VHS+b1GBrQ8suLdja8bJJDbY2tOzS2oxldpazTZ0LAaNEzMqVK6fQ2LFjR2biRa+TST1wAq9dQ0ZnZcfLJjXY2tCySwu2drxsUoOtDS27tBAwdrwKkdrG6dW3PRW889UXz6HBmxcW4v0lXwKdlRxdsAVbOQJyltFu5djajGVytfBvOTcRGP+vVrpFU6djAa89a3RW9sxMc4CtKSn7dGBrz8w0B9iakrJPZzqW2VvONgcETAJ\/U6frAubxmxbSNZfMydarFVA6Ois5J4Et2MoRkLOMdivH1nQsk6uBjOUZJWDUjqfBwcGA5po1a6irqyuWrKnT\/+q5Mbrtyy8GdiBgzBoqOiszTi6pwNaFmlkesDXj5JIKbF2omeUxHcvMrOUn1YwSMHymDD8sWkxO+jV1Ohbw2jdodFb2zExzgK0pKft0YGvPzDQH2JqSsk9nOpbZW842x4wSMGHUuqCJcoOp09UVAljAa96Y0VmZs7JNCba2xMzTg605K9uUYGtLzDy96VhmbjEfKXMhYNSZL2NjY9OoNDU1iZzEm3SFgaqEidP1KwQ+dmk1\/e2apnx4Nue1QGcl5yCwBVs5AnKW0W7l2JqMZXKly1nOXMCodSn19fWJ61F8IuDIy7Zt26i5uTnxniXl9LVr19KSJUsmq1BXV0f8w8+3DvyYPvEX\/xz8\/+3Xz6Xbr\/+gz6oW1hZ3VuPj41RbW5vJPVeFBUtEYCvnXbAFWzkC\/ixz38o\/6hkdHaX169eTyZlm\/mohbylzAWMSCZHCEHULtl6WEjDh8tvb22nVqlXBr\/eOnqA\/fPR0Q\/mLT9TR4obZUtUtlN2TJ0\/SxMQE1dTU0KxZswr1blm\/DNjKeQBswVaOgD\/L27dvp4GBgWkGIWD8MQ4sqQhMW1tb2U\/d5amrzs5O6u3tnbzCIErAbN68mRoaGiY\/0iMw9z35Kt335OHgs2+tW0jzqiFgTJoI+\/3IkSNBJGv2bDAzYWaaBmxNSdmnA1t7ZqY5wNaUVHq6cARmz549tHXrVkRg0tHZp+BIRxa3TqeVazJviB1I9v7mHJjvduNmkgtsTSi5pQFbN24mucDWhJJbGpOxzM1ytrlyM4U0PDwcScLnIl5915HJ2hsTp6sdSBx52fe5q7L1ZgWVjs5KzllgC7ZyBOQso93KsTUZy+RKl7OcuYCRe7XplsMH2Zku4o2bN9R3IGELtZ0n0VnZ8bJJDbY2tOzSgq0dL5vUYGtDyy4tBIwdr0KkTnO6foVA1\/L51LX8okK8dzleAp2VHGWwBVs5AnKW0W7l2KaNZXIly1rOTQRm9+7dtGHDhuBtOeJx8OBBGhoaStzmLIuGKM3puoDh6SMs4DX3CDorc1a2KcHWlph5erA1Z2WbEmxtiZmnTxvLzC3lK2UuBIzazsw7gm655ZbgPBhe+9Ld3U3lPB8m7Jo0p\/c88Qr1PHEgyIY7kOwaNjorO142qcHWhpZdWrC142WTGmxtaNmlTRvL7KzlJ3XmAkY\/B2bBggXU0dERCJilS5cGEZAsdicp96Q5\/ZPbhumpF48GyY\/ef11+vFoBNUFnJecksAVbOQJyltFu5dimjWVyJctahoBJ4JvmdGyhdm+c6Kzc2aXlBNs0Qu6fg607u7ScYJtGyP3ztLHM3XK2OTMXMPz6vP6F17voU0gqGtPa2kotLS2ZUEpzevVtTwX1wg4ke\/egs7JnZpoDbE1J2acDW3tmpjnA1pSUfbq0sczeYj5y5ELAMIqoY\/s3btyYmXjR6xS3jVoJmLYr6+ihtsvz4dEKqQU6KzlHgS3YyhGQs4x2K8cWAkaObW4tJzkdW6hLcxs6q9L4JeUGW7CVIyBnGe1Wji0EjBzb3Fo2FTDYgWTvQnRW9sxMc4CtKSn7dGBrz8w0B9iakrJPBwFjz8wqh34OjMqY9c2ZSU7HFmor905LjM6qNH6IwMjxA1uwzYaAXKkQMHJsg0W8u3btor6+Pqqurg5KUtur87qI9+adL9DOb4+friu2UFu3DggYa2TGGcDWGJV1QrC1RmacAWyNUVknhICxRmaWQT8Hhs9+0Z88nwODLdRm\/o1Lhc6qNH6IEsjxA1uwzYaAXKkQMEJsK1XAqFuosYXarWFAwLhxM8kFtiaU3NKArRs3k1xga0LJLQ0EjBs3o1wMd926ddTf30+NjY1BnjxPIeEWaiO3JiZCZ1U6Q0S35BiCLdiWn4BciRAwQmyVUBkeHk4tge9Heuyxx1LT+UoQ53RdwOAWajfaEDBu3Exyga0JJbc0YOvGzSQX2JpQcksDAePGraJzxTldPwOGD7Djg+zw2BFAZ2XHyyY12NrQsksLtna8bFKDrQ0tu7QQMHa8CpHaRMDgDBg3V6OzcuNmkgtsTSi5pQFbN24mucDWhJJbGggYN27GuaLOgcnrVQL6GTD7PncVzauebfyeSHiaADoruZYAtmArR0DOMtqtHFsIGDm2FXcOjNpCzUhwBoxbw0Bn5cbNJBfYmlBySwO2btxMcoGtCSW3NBAwbtxSc1XiNmqcAZPq1tQE6KxSETknAFtndKkZwTYVkXMCsHVGl5oRAiYVkVuCUgVMeBdTc3Mzbdq0iaqqqiIrpE9VpaWNc7q6hRpnwLj5nHOhs3Jnl5YTbNMIuX8Otu7s0nKCbRoh988hYNzZpeZ0vUrg+PHj1N3dTcuWLaOWlhZS\/66vr6eurq5p5eon+7LA4bxxaTlzlNP1LdS8+4h3IeGxJ4DOyp6ZaQ6wNSVlnw5s7ZmZ5gBbU1L26SBg7JlZ5fC1iJftDA0NRUZhwp8lpTURMDgDxsrFUxKjs3Jnl5YTbNMIuX8Otu7s0nKCbRoh988hYNzZlTVnkiiJisCo6E1UJdMiMDgDxt216Kzc2aXlBNs0Qu6fg607u7ScYJtGyP1zCBh3dmXLaXL9wMjICK1evZrGxsZox44dFL5AUq+scvratWtpyZIlwUd\/87136G++97Pg\/7\/0B5fRNZecW7b3K1JB3FmNj49TbW1t7HqlIr1vOd8FbOVogy3YyhHwZ5n7Vv5Rz+joKK1fvz51zPNXg\/JYOuPUqVOnylOUbClq\/QuXEreIN7zWpqenJ6hU1HoZ\/r0SMHrNT1y2gk5cdmPwq8H2uVR\/zpmyL1ZQ6ydPnqSJiQmqqamhWbNmFfQts3ktsJXjDrZgK0fAn+Xt27fTwMDANINpf7T7q0F5LBVCwJiIl\/CCX8bL0ZjOzk7q7e2dvERSx64EzObNm6mhoSH46KbHJ+jAibOD\/x+796ryeKmApbA\/jhw5QnV1dTR7Ng4C9OlisPVJc6otsAVbOQL+LIcjMHv27KGtW7ciAuMPsR9LaTuPVCmlCBhdteIMGD9+w3y3H45RVsAWbOUIyFlGu5VjizUwcmxLsszTQLyeJensF1VA1BRSUt4op19xz3PEW6lxBkxJbsM5MKXhS8yNgUAOLtiCrRwBOcsQMHJsnS2HD7FThpqamqivry9YHMpnvbS1tU0u1mXBs23btiCpy0F2OMTO2V1TMmIg8MMRERg5jmALtuUlIFcaBIwc29xaDjtdP8QOZ8CU5jYImNL4JeUGW7CVIyBnGe1Wji0EjBzb3FoOO\/2Z\/cdoxcPPB\/V9\/KaFdM0lc3Jb97xXDJ2VnIfAFmzlCMhZRruVYwsBI8c2t5YhYORcg84KbOUIyFlGuwVbOQJyliFg5Njm1nLY6T1PvEI9TxxABMaDxzAQeIAYYwJswVaOgJxltFs5thAwcmxzazlJwOz73FU0rxrnl7g6D52VK7n0fGCbzsg1Bdi6kkvPB7bpjFxTQMC4kqvgfGGnqzNg+JWO3n9dBb9Z9lVHZyXnA7AFWzkCcpbRbuXYQsDIsc2t5TgBw5EXjsDgcSeAzsqdXVpOsE0j5P452LqzS8sJtmmE3D+HgHFnV7E5w07HIXb+XInOyh\/LsCWwBVs5AnKW0W7l2ELAyLHNreWw03GInT9XobPyxxICRo4l2IJt+QjIlQQBI8c2t5Z1p9cvuII4AsPPH13\/Ibrzt34pt\/WuhIpBwMh5CWzBVo6AnGW0Wzm2EDBybHNrWXf6O+dfNnmIHU7hLd1l6KxKZxhnAWzBVo6AnGW0Wzm2EDBybHNrOU7APNR2ObVdWZfbeldCxdBZyXkJbMFWjoCcZbRbObYQMHJsc2tZd\/or75lPN+98IagrrhEo3WXorEpniAiMHEOwBdvyE5ArEQJGjm1uLUPAyLkGAgZs5QjIWUa7BVs5AnKWIWDk2ObWsu70v37lA7Tz2+NBXXEKb+kuw0BQOkNECeQYgi3Ylp+AXIkQMHJsc2tZd\/oX9s6iZ18+FlwfgEPsSncZBEzpDDHIyjEEW7AtPwG5EiFg5Njm1jIEjJxrIGDAVo6AnGW0W7CVIyBnGQJGjm1uLetO\/89fP0WHjp6gqy+eQ4M3L8xtnSulYhgI5DwFtmArR0DOMtqtHFsIGDm2ubWsO\/0\/fOl4UE8IGD\/uQmflh2OUFbAFWzkCcpbRbuXYQsDIsc2tZeX0noe+SGu+diqoJ5\/\/wufA4CmNADqr0r6Q2TwAABAuSURBVPgl5QZbsJUjIGcZ7VaOLQSMHNvcWo4SMDiF14+70Fn54YgIjBxHsAXb8hKQKw0CRo5tSZaPHj1KHR0dNDw8HNhpbm6mTZs2UVVVVaRd5Uj+sKmpifr6+qi6ujox7c13P0y8C4kfnMJbkrsmM0PA+OGIQVaOI9iCbXkJyJUGASPH1tny8ePHqbu7m5YtW0YtLS2k\/l1fX09dXV3T7I6MjNDq1atpy5YttHTpUtq9ezcNDQ3FCh7ldF3A4AwYZ3dNyQgB44cjBlk5jmALtuUlIFcaBIwcW6+Wk0QJf3bgwIFIcRNVCeX0pvZ76ekf1gZJcI2AH3dBwPjhiEFWjiPYgm15CciVBgEjx9ar5TgBE47WmBSqnH7+x2+i\/WcvDrJsu+EMuvLy+VRXh8scTRjGpWEBMz4+TrW1tbHTfaXYn8l5wVbO+2ALtnIE\/FnmvpV\/1DM6Okrr16+nHTt2BLMPRXnOOHXq1OntNQV41HqY1tbWYEpJf5SAWb58OT3yyCPBmhnTNTBvL\/oD+um8qwNzc77SQe3t7bRq1aoCEMvuFU6ePEkTExNUU1NDs2adXl+Exw8BsPXDMcoK2IKtHAF\/lrdv304DAwPTDELA+GPs1ZISKGw0ahGv+vzQoUOTC3d7enpobGwsdQ3Mj6\/ppHfOvzS4RuDPP35GEH1BBKY097E\/jhw5EnCcPXt2acaQe5pYB1uZRoF2K8OVrYKtP7bhCMyePXto69atiMD4Q+zPUpp4UV8OfcEv\/44X9XZ2dlJvby81NjZOq5CaQtIFDO5B8uM3rIHxwzHKCtiCrRwBOctot3JssQZGjm1JltN2HunGOeIyf\/78yeklFjD33nsv3XfffZFbqZXT3\/qNHvr5WefjFN6SPDU1MzorjzBDpsAWbOUIyFlGu5VjCwEjx7Yky2nTQLpxdiKnV2e\/8P\/zE7Xlmn+vnH7sd\/qCdLhGoCRXTcmMzsofy7AlsAVbOQJyltFu5dhCwMixdbYcPsROGVKLc\/kwO542amtrm1x5rR9kZ3LoXeunbyWOwARCZ\/l86lp+kXN9kfFdAuis5FoD2IKtHAE5y2i3cmwhYOTY5tYyOx0CRsY96KxkuLJVsAVbOQJyltFu5dhCwMixza1ldvrv3XoX8SJefnCNgD9XobPyxzJsCWzBVo6AnGW0Wzm2EDBybHNrmZ3+u90PEZ8Dww9O4fXnKnRW\/lhCwMixBFuwLR8BuZIgYOTY5tZyeBEvBIw\/V0HA+GOJQVaOJdiCbfkIyJUEASPHNreWwwIGFzn6cxUEjD+WGGTlWIIt2JaPgFxJEDBybHNrmZ3e1t5BP\/ytB4M6Hr3\/utzWtdIqBgEj5zGwBVs5AnKW0W7l2ELAyLHNrWV9DQxfI4BTeP25Cp2VP5aIEsixBFuwLR8BuZIgYOTY5tayPoUEAePXTRAwfnnq1sAWbOUIyFlGu5VjCwEjxza3lnUBg1N4\/boJnZVfnhAwcjzBFmzLQ0CuFAgYOba5tQwBI+caCBiwlSMgZxntFmzlCMhZhoCRY5tby7qAabuyLjjIDo8fAhgI\/HCMsgK2YCtHQM4y2q0cWwgYOba5tawLmD\/9vUtp1dL63Na10iqGzkrOY2ALtnIE5Cyj3cqxhYCRY5tby7qAwUWOft2EzsovT90a2IKtHAE5y2i3cmwhYOTY5tayLmBwD5JfN6Gz8ssTAkaOJ9iCbXkIyJUCASPHNreWdQGDawT8ugkCxi9PDLJyPMEWbMtDQK4UCBg5trm1DAEj5xoIGLCVIyBnGe0WbOUIyFmGgJFjm1vLyum\/+YW\/o67lFxEfZofHDwEMBH44RlkBW7CVIyBnGe1Wji0EjBzb3FouqtPzABydlZwXwBZs5QjIWUa7lWNb1LHsjFOnTp2Sw1bZlovq9Dx4BZ2VnBfAFmzlCMhZRruVY1vUsQwCJqHNFNXpcl8Tc8vorMxZ2aYEW1ti5unB1pyVbUqwtSVmnr6oY1nFC5ijR49SR0cHDQ8PB95sbm6mTZs2UVVVVaJ3R0ZGqLOzk3p7e6mxsTEybVGdbt7s5VIeOHCABgYGqL29nebPny9X0Ay0DLZyTgdbsJUjIGe5qGNZRQuY48ePU3d3Ny1btoxaWlpI\/bu+vp66urpiW4NKt3fvXurv74eAkfvexFou6hcqA5TTigRbOS+ALdjKEZCzXNR2W9ECJsrdu3fvpqGhocQoDDuzp6cnyI4IjNyXJslyUb9Q2dCcWirYynkBbMFWjoCc5aK22xknYHjK6a677qK2trZAxJgImLVr19KSJUvkWtcMtDw6Okrr168nsPXvfLD1z1RZBFuwlSMgZ1m12x07dtDSpUvlCiqz5UIJGLUeprW1NZhSiovQ8O8XLVqUugbm8OHDwSC7Z8+eMrsFxYEACIAACICAPwL8R\/jmzZtp7ty5\/oxmbKkwAkata2GecYt4eeEuLxy94447iMVJ2iJetsXp+AcPCIAACIAACFQqARYuRRIv7IdCCBgT8cIvy1NG1157bRBCM9mFVKkNFfUGARAAARAAgaITqHgBY7rzKLzdWnds0eYFi95o8X4gAAIgAAIgUPEChqMqY2NjRme\/6O5GBAaNHwRAAARAAAQql0BFC5i4qEpTUxP19fUFh9nxOTG84yi88hoCpnIbLWoOAiAAAiAAAhUtYOA+EAABEAABEACBmUkAAmZm+h1vDQIgAAIgAAIVTQACpqLdh8qDAAiAAAiAwMwkAAGT4HdeILxt27YgBXYquX1BeK3R6tWrg4XWaRdt6rz5Pquke6rcalOsXDZs1ZuH7w8rFhE\/b2PDNbwOD\/1Esg9s2Opp0R+U1rbV9z5qPWhplrPNDQETw1\/dl8SLgV966aXgDBn+\/+rq6mw9VkGl64PlihUrply8GX6N8B1W\/O9du3aBeYy\/bdjqJpjrhg0baOPGjbGnVVdQE\/NeVRuu4SMcsDEg2R02bJUw5Et5eQMG+gP3pq64Dw4OFu4PcQiYmHahLnvkL1BR1av7V8IsZ7hDZ1G4c+dOoy3vGAzS\/5LVT5I2YcuDwu23307Hjh2jpOs2zLxbzFQ2bZbT3nvvvXTffffhDxuD5mDLVm\/f6A8MAEckUVGsxYsX06FDh0gJQjdr+csFARPhk3CYHWF3t4arR7E4chX+d5JVdFjJzF3Ysii\/8sor6atf\/SotW7YMEZgIxDZcw1FDt2\/JzMllwzYqAjM0NGT0x8\/MIZr+pnyJIz98pEhHRwcETDqyyk8RFXHhzn\/+\/Pno9C3cG44K2PzF6npAoUX1KjqpLVt1D9htt90W3MYOARPtfhuuLGAOHDgQGMJaufSvkw1btqZPfaxZsyYYfPG4EQgLQjcr+cuFCExCBEZf8AQBY994bTssVQIPDA888AAW8SYgt2HLA8EXvvAFam9vDy5z48MdIWD8CBheT6QW7rJP1q1bh3Yb025t2qya+tiyZUuwBsYmemvfUxU\/BwRM8X08+YaYQvLjbJuQMcSLHXMbtpz26aefDv6CxXRoMmcbruEpJLAFW7tvcflSQ8CUj3UuStIjLljE6+aS8JRR2kJT7DQw52zDVt+erpeAsPx03jZcw+0Z\/URy+7VhC3Fo3heYpISAMaFUoDTYRl26M222TSL8bsfbhq1uGVGCZM42XMODAqY5\/LGNmkLC9JxdH6GnhoBxZ1exOXGQXemuSzq4Si2C5KmNuCgBDgaL94EpWwgYu3Zsw1U\/yA6HraVztmHLgnDlypWBUbBNZ5uUAgKmNH7IDQIgAAIgAAIgAALeCGAXkjeUMAQCIAACIAACIFAuAhAw5SKNckAABEAABEAABLwRgIDxhhKGQAAEQAAEQAAEykUAAqZcpFEOCIAACIAACICANwIQMN5QwhAIgAAIgAAIgEC5CEDAlIs0ygEBBwIvv\/wynXvuuca3HfN2yTfffJMuvvhih9LMsqgt701NTdTX12dct0q5oFO9X7m27pa7PDMvIxUI5J8ABEz+fYQazlACtgejleOsh1JESCl5y9kEWFDwU87LAyuFTTn9gLJAII0ABEwaIXwOAhkRyKOAsa2Tjq5SBmkImIwaPIoFAUsCEDCWwJAcBHwS0E9yZbtqWuall16aPIWUf69OJA6fWKymOc477zzq6Oig4eHhoHrqniN1NP7g4GDwe5NpH70MfRqFT07m25fVs3HjRmppaZmGIy6dEjArVqygu+++O8gXnqbRT18Nl8Osbr\/9dvroRz8a5Ffv8sYbb9Dq1atpbGwsyPL5z3+eHn\/8cert7aXGxsbgd3HvFOXLsIDhf\/\/oRz8KfhTHpHukwvf4cBlRv6tEceez7cMWCJRKAAKmVILIDwKOBKLuJdIHz3C0I+6COy5+06ZNwU3TLGJ46mPp0qWkxFFra+uk0Ei6MFPVR9mrqqoKBt4HHniA+vv7AzGQFoEJp9fvtGGRxUJj8eLFQX2V\/V27dgVraViIdHZ2ThEeuj0l0ubNmzeZP\/yO6t8TExNBnefOnUvd3d2BUFJTQmn3bkUJmG3btpESbFFc9SYAAeP4hUA2ELAkAAFjCQzJQcAXgbSBME0shP+yDwuYqNu\/ky5zjJriCadPqlPaRZHhC\/q4\/mnTSvrnSsCEBdnQ0NCkoGGbukDhf99777103333TVlsnDRNFCVgOLqjRJcqg9NFLWKGgPH1DYEdEEgmAAGDFgICGRLQp1vC0ztxYiE8zdLc3BwZgQlP5eivGTX9E1eefulmkoBJW0QcJVaifheeVgtPk6kIE79PlBDRbXJUR10IGHZz3DRQlIDhvPqi3iThBQGT4RcKRc8oAhAwM8rdeNm8EtAHbX0djP5XvhIk4XUpKgIRjsBw3nDkIOn948RJ0rSWbq9UAcO21FoWJbCiIjA2Aua73\/0uqSmq6upqI\/dDwBhhQiIQyJwABEzmLkAFQOBdAroIUBEGnqbg9SK8lmPZsmVTFs7qIiUsYJLWu0QxL8cUUniNi14mi42k6SA1haQLmKhohz6FxBGYdevWTa7hMWlrElNIaWIybSrNpN5IAwIzjQAEzEzzON43NwSi1sDoURB9UWvUYlQVkVFTSPxiushR9nlBr5r+iFqHooD4WsSrRzz0dTGLFi2atkg3LGD0vKquXD9ekBslYEwX8bINtYYlbe1RqYt41RSf2jmm3kNfvBxuhBAwuflaoiIVRAACpoKchaoWj4Aa3NQWYH16SN8CzVMqN9xww5St0ixcbrzxRrrzzjsnIwxhUaOiMmp7NRNUA2sczaQtx6YLi6O2W5usgQmXvWXLlmCdCy\/cVe+vR2D4HcIMw9uow1vJOU\/cFnAV9eL\/KtEXtY1azx\/1Xvr6I\/bTFVdcQfv27QtEVFhoqncIR6eK19rxRiDglwAEjF+esAYCIJAxARYUUTuPTKtlsgbG1JZpOkRgTEkhHQi8SwACBq0BBECgYgmE1\/moaIt+7ovty0HA2BJDehDIhgAETDbcUSoIgIAnAuHTiZNOyTUpMny54qOPPhpkk7obCZc5mngFaUBgOgEIGLQKEAABEAABEACBiiPw\/wAG5ZKg9K\/hfAAAAABJRU5ErkJggg==","height":104,"width":172}}
%---
%[output:8982d8ac]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Unrecognized function or variable 'heatsink_liquid_2kW'."}}
%---
