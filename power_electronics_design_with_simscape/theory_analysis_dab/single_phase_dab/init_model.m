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
use_thermal_model = 1;
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
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/8) %[output:4b1b289a]

f0 = fPWM_DAB/5;
Cs = 1/Ls/(2*pi*f0)^2 %[output:6c8d3181]

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
kp_i_dab = 0.1;
ki_i_dab = 5;
kp_v_dab = 1;
ki_v_dab = 45;
%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:1384c0dc]
Iac_FS = I_phase_normalization_factor %[output:2bf54a89]

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
heatsink_liquid_2kW; %[output:038a5c0b] %[output:7c8883f0] %[output:734836a0]
%[text] #### SKM1700MB20R4S2I4
danfoss_SKM1700MB20R4S2I4;
% Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2
% Rsnubber = 1/(Csnubber*fPWM_DAB)/5
Csnubber_zvs = 4.5e-9 %[output:545cfcb0]
Rsnubber_zvs = 5e-3;
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

% if use_thermal_model
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge ideal switch based', 'Commented', 'on');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge mosfet based with thermal model', 'Commented', 'off');
%      set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge ideal switch based', 'Commented', 'on');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge mosfet based with thermal model', 'Commented', 'off');
% else
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge ideal switch based', 'Commented', 'off');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge mosfet based with thermal model', 'Commented', 'on');
%      set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge ideal switch based', 'Commented', 'off');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge mosfet based with thermal model', 'Commented', 'on');
% end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":50.3}
%---
%[output:0994022b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:60f8fa48]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:4b1b289a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     3.551136363636363e-05"}}
%---
%[output:6c8d3181]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     1.783252832105145e-04"}}
%---
%[output:1384c0dc]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:2bf54a89]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:345f228b]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000200000000000"],["-19.739208802178720","0.996858407346410"]]}}
%---
%[output:9e69d693]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:20880618]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     2.000000000000000e-04"}}
%---
%[output:266aaba1]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":" -19.739208802178720"}}
%---
%[output:7a235d1a]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.996858407346410"}}
%---
%[output:53464e72]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.073386376052051"],["3.802432508328568"]]}}
%---
%[output:2b42b309]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.283130953403918"],["67.668222262819981"]]}}
%---
%[output:19de2814]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:5159e62b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:85434b31]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000200000000000"],["-19.739208802178720","0.996858407346410"]]}}
%---
%[output:613a9f65]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.311017672705390"],["54.332172227996914"]]}}
%---
%[output:6480ce36]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQt0nsV555+0bGMllNjiUiMEMReZkKaVMUuiGAeTVVOfbiuTTS+S3G1SRU3dBidnCz6SXBdOCQRLqk1LuSQuq2jd3Vr2toXG3uyGEIeQ+Cg+AQNq03IRCF+EcLgYAhST1MV7nteMGL16LzPzzfN97zv6v+dwjPQ987wzv2e+mb\/m+o7jx48fJzwgAAIgAAIgAAIgUCIC74CAKVG0kFUQAAEQAAEQAIGIAAQMKgIIgAAIgAAIgEDpCEDAlC5kyDAIgAAIgAAIgAAEDOoACIAACIAACIBA6QhAwJQuZMgwCIAACIAACIAABAzqAAiAAAiAAAiAQOkIQMCULmTIMAiAAAiAAAiAAAQM6gAIeCJw5MgR6u7ujrwNDQ1RfX29J8+z3YyPj1NXVxddcskl1N\/fT3V1dWLv0h1Xs4wmBVIcPve5z1F7e7tJkkLbDAwM0JYtW6I8rlmzhnp7e63yu2PHDlq\/fj1t3LixkDy4fHv37hX\/flhBg3FpCUDAlDZ0yHjRCFSzc08TMNxBrFixglpaWkTwpJVR+r1phXHpELkDvf\/++63EgUsa2wDwO1avXj2dLEQBE5rgtI0x7P0SgIDxyxPeQKAmBI4ePUp9fX20a9cu2rZtW9UEDI\/8VOO9SVBVZ9jW1mYsRtQIhY04cEnjUgmUgLHJW\/w9RR+BUfX04MGDGIVxqSRIM4MABAwqRCEJ6EPpnEF9SFwffViyZAndcMMNURm4I9OnU9RowdjY2KzPdR9XXnkl\/d7v\/V5k09DQQMPDw9TU1JTIRRcKcfv46AR\/rqaUWltb6eabb6bm5uao4dY7\/jw\/PBWl3rtv374of\/yoKaTrrruOvvCFL0TiRT1xFvx7xVQXOKrT1O1VJ6h86R2qXsbbbruNBgcHE987OTkZ5W9qamo6T3oMdY7M\/Pbbb6evfOUrpMrH\/OOsFTs1NZfUWau46u9V5Y2XS8W6sbFxWoTF+e3cuTOaklGPXj\/i\/vKEY7w+ZvnKqodZIzU6E86zyntcFMW\/Xzpb5ePqq6+m3bt3E39\/VOz0MvPv1Dv02OZxSaqHhWyEkKnCE4CAKXyI5lYG452WXnrVCCd1UvGOh\/2weFDiJf55Ugeb1fnzZ2l5U53NqaeeOmMNjBIweh5YKCQJDl3ExP34EjBJf+HHO5N4x5bGlX+fJmB6enpo7dq1s9hnCQb+7PTTT6fnn38+EmhJooLfqXe08byn1Qv13oceeihRjNx1113T6070+qZ30HEBE\/elPk8TMVl1ltMcOHAgVSjpeYqLl7jIVOLhIx\/5CH33u9+d0XgkiZCk71dcgLBNUh759+o9eb51LkUfJZpbLW65SwsBU+74BZd71UDrf4Gqxp8Lq48+8F\/ZqmHUOwi9sdX\/8tQ7PBYJaoRA+VDvjv+lryAnfa43xh\/72MdSBYz+F6qtnzwBw6NO\/ORN5WSNEPGo0IsvvjiLiT5qwJwWL148o4wmU0jx6S3FXsWTR1vicee88HqQpJEhZrlq1aqovPqIjcnCZpPpoLhN\/GfFRIktzn\/eu1XdSyqP+h0LXS5z2hSSzlHVp3hM77333kgIJY2opPmNj8KpUSfdR9a71QiNqv95XHxMlQXX8KFATgQgYJywIZEUgbS\/zlQHwA330qVLE3fg6Db79+9P\/Kua86374L\/61Y6hvEW4eX85pgkEvUHn99v68SVg9HezGOFH7zDTOha9A\/\/MZz5jLGCSRqyS3sv5iE+RpY1wsC13xCofOtuk98Wn0rIETNrUWTxN1mhKkviNl01NT8aFkBJtaUIjr37q8dV9pMU1PpqjWCkBkzZ1qO+w0+uy+l7q03eqndC5JE1bSrUn8Bs2AQiYsONbutJVW8Do25DzOghb4cHwk7ZVm\/rRO+d4Z8e+9W3UJiMwbKMvfOWfectufAQq3oHaCpg4x\/goTVw4+RIwqrKnCSfemZUkYOJTUXkjMGUQMEkjfiqu8fqXNgKj+0j7bkDAlK6JDSrDEDBBhbP8hXGdQopPdag1BWl\/zSYN+ecJmKypH31UgKPAf6WmCRhTPzw0HxcXamotLmBYJJgsjox37voIRXwajjv8vCkkHh3KEwBxH65TSHrtThvViH8D4mIkPhqhyqyPxKnyqLoTT5M0hZT3zZOeQlJiV41cpQmYpJErxSg+ApO26Do+fZU1hZTEBVNIebUFn5sSgIAxJQW7qhCQXsSbJQDyBExW3pLWh6QJmDw\/PNyu1rPEoZsIGE6TtAtJ+YrvJNEPgLNZxKumEvQ0\/N5PfOIT0ehQ0sOckspnuoiXfSpRFxdOaQtc9TS6Db\/zlltuoRtvvHHWgmNOExcw\/Lu0BcGqrHmCOWl6JW8ETOeYVsYs8aELhs9\/\/vOpdSvLB+chaXGv6SJenUveCGRVGhq8JAgCEDBBhDG8Qphuo9a3QOdto05aGGwzhcSUs6Yn8hbJ6ifzZvnh98S33F577bX0yCOPTC9aTRqB0Tu3tIXIuu\/42pwkgaN35Hpa\/n8lYJLee+edd844UZYP19PX27hso9aFiN6hJm2xT9u+Heeqr8lhn8xt06ZNtG7dugiHPpKmdpOlbcvOO78laxs1v8t0ZCJt7QqPwiWJg7RRJ2aUtIU9aRQnTfzy7+Mn\/6atJVI+TEYKw2vRUCIJAhAwElThU5RA3o4P0ZfDecUEkqaq4jvN0s7h0V\/ucpBdxZmfow50wamEWtLOpDw8OMgujxA+tyEQpIDhho3PouBDtpIawqwDzmzgwbY2BCBgasPd11uzptCypr6S3u9ylYCvcsw1P0lTSMwg7\/DHJNEZyt1Vc60OFK28wQmYvMV96vNly5ZFl52pn\/lLaHtxWtGCOVfyAwFT\/kjH\/4jgEtmKF06Du3WqWxfiU7s24oVzCsFZ3XiF\/rbgBAzP9\/KXhJ+0EZh4UPkvi9HR0are6ht6xUL5QAAEQAAEQECSQFAChv+qu\/7666mzszMSMRAwklUHvkEABEAABECgdgSCEjA8ksIPnwiZtQZGx62Gsjs6OqIpJTwgAAIgAAIgAALFJxCMgOG58K1bt9KGDRuIL+ozETBq\/QuHSb\/FOB628847r\/iRRA5BAARAAARAIIEA3yp+7rnnBscmGAHDU0Z81gSfHpq3C4mjaCpe2JYFzMTERHDBr3WBwFUuAmALtnIE5Dyj3sqwDZVrEAImaUeDqgZJ19vb7jwKNfgyXxVzr+BqzsrWEmxtiZnbg605K1tLsLUlZmYfKtcgBEw8hHkjMDxaw6dQZk0b6T5DDb5Z1ZezAlewlSMg5xn1FmzlCMh4DrXOzgkBo0ZceHfS4sWLoxuC1bHgqrpkHb0eavBlvirmXsHVnJWtJdjaEjO3B1tzVraWYGtLzMw+VK5BChizkJpbhRp8cwIylk8\/\/XSQC8tkaNl5BVs7XjbWYGtDy84WbO14mVqH2odBwBjUgFCDb1B0URM0VnJ4wRZs5QjIeUa9lWEbah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn1VoZtqH0YBIxBfQk1+AZFFzVBYyWHF2zBVo6AnGfUWxm2ofZhEDAG9SXU4BsUXdQEjZUcXrAFWzkCcp5Rb2XYhtqHQcAY1JdQg29QdFETNFZyeMEWbOUIyHlGvZVhG2ofBgFjUF9CDb5B0UVN0FjJ4QVbsJUjIOcZ9dY\/2z1Pvkwf33QvvfDl3\/TvvMYeIWAMAgABYwDJwQSNlQM0wyRgawjKwQxsHaAZJgFbQ1AWZiMPHKarRh6lIzd\/1CJVOUwhYAziBAFjAMnBBI2VAzTDJGBrCMrBDGwdoBkmAVtDUBZmEDAWsEI0hYCRiSoaKxmu7BVswVaOgJxn1Fv\/bAfu2U8D9zyNERj\/aMvhEQJGJk5orGS4QsDIcQVbsJUl4N87BIx\/pqXyCAEjEy4IGBmu6GTluIIt2MoS8O8dAsY\/01J5hICRCRcEjAxXdLJyXMEWbGUJ+PcOAeOfaak8QsDIhAsCRoYrOlk5rmALtrIE\/HuHgPHPtFQeIWBkwgUBI8MVnawcV7AFW1kC\/r3zFmreiYRt1P7ZlsIjBIxMmCBgZLiik5XjCrZgK0vAv3cIGP9MS+URAkYmXBAwMlzRycpxBVuwlSXg3zsEjH+mpfIIASMTLggYGa7oZOW4gi3YyhLw733V7Q\/TnqdexhSSf7Tl8AgBIxMnCBgZruhk5biCLdjKEvDvHQLGP9NSeYSAkQkXBIwMV3SyclzBFmxlCfj3DgHjn2lhPI6Pj1NPTw8NDg5SU1NTYr4gYGTCBQEjwxWdrBxXsAVbWQL+vUPA+GdaCI9Hjx6lvr4+2rdvHw0PD0PAVDkqEDBywMEWbOUIyHlGvfXPdsmN36ODR97AGhj\/aGvrce\/evTQwMBBlAiMw1Y8FGis55mALtnIE5Dyj3vpnW3\/1fZFTnAPjn23NPB45coSuv\/566uzsjEQMBEz1Q4HGSo452IKtHAE5z6i3ftnyyAuPwEDA+OVac287duyI8rB06VKsgalRNNBYyYEHW7CVIyDnGfXWL9s9T75Mq+54GALGL9baeuOFu1u3bqUNGzbQ5OSkkYDRc7x79+7aFiCQtzP7xsbGQEpTrGKArVw8wBZs5Qj48dza2ho5OnbahfTa8h4IGD9Yi+GFp4xWrFhBLS0thF1ItYsJ\/tqSYw+2YCtHQM4z6q1ftuoiR\/aKNTB+2dbEG6996e7uprGxsVnv37ZtWyRq4g+2UcuECo2VDFf2CrZgK0dAzjPqrV+2ags1BIxfroXxhhGY2oUCjZUce7AFWzkCcp5Rb\/2yVVuof+r1F+iFL\/+mX+cF8PaO48ePHy9APmqWBQiYmqHHKIEgenQEcnDBFmzlCPjzrO9AOum5f6bn\/tdaf84L4mnOCxiTOGAKyYSSvQ06AntmpinA1pSUvR3Y2jMzTQG2pqTy7XQBc\/KeQTr4\/f+Xn6hkFhAwBgGDgDGA5GCCxsoBmmESsDUE5WAGtg7QDJOArSEoAzN9\/csp3+il\/T\/4vkGqcplAwBjECwLGAJKDCRorB2iGScDWEJSDGdg6QDNMAraGoHLM9NGXc+rn0Stf+W2amJjw47xAXiBgDIIBAWMAycEEjZUDNMMkYGsIysEMbB2gGSYBW0NQOWYjDxymq0Yejax6V55LW\/7wP0HA+EFbPi8QMDIxQ2Mlw5W9gi3YyhGQ84x664etmj7i0Zedn72YrviP74eA8YO2fF4gYGRihsZKhisEjBxXsAVbWQKVe9dHX7qWnUWbf2MxhdqHYQrJoL6EGnyDoouaQMDI4QVbsJUjIOcZ9bYytrz2he8+4n959OW2joto+QXzIWAqw1ru1BAwMvFDYyXDFaMEclzBFmxlCVTmfdv3D9Pa7SfWvnReupBu77wo+v9Q+zCMwBjUl1CDb1B0URMIGDm8YAu2cgTkPKPeurPVp4549OWRP\/nwtLNQ+zAIGIP6EmrwDYouaoLGSg4v2IKtHAE5z6i3bmz1bdPsgRfu8tSRekLtwyBgDOpLqME3KLqoCRorObxgC7ZyBOQ8o97as9XXvXDqhza00KJT62Y4CrUPg4AxqC+hBt+g6KImaKzk8IIt2MoRkPOMemvHNi5efudDZ9It7e+b5STUPgwCxqC+hBp8g6KLmqCxksMLtmArR0DOM+qtOds9T74c7ThSj75oN+4l1D4MAsagvoQafIOii5qgsZLDC7ZgK0dAzjPqbT5bHnX5+j+\/QH13j08bf+WTP08fX3JGauJQ+zAImPz6EuwWNIOii5qgsZLDC7ZgK0dAzjPqbTbb+JQR7zbiqwJ49CXrgYCRq7OF9xxq8GsNHo2VXATAFmzlCMh5Rr1NZztwz34auOfpaQN1TQD\/m\/eE2odhBCYv8gEfAmRQdFETNFZyeMEWbOUIyHlGvZ3JlkdcHj70KnVt\/cGMD27teB\/99gfPNA4EBIwxqvAMQw1+rSOFxkouAmALtnIE5Dyj3p5gy8Jl92NH6Jq\/e3wGbJtRFz1hqH0YRmAMvouhBt+g6KImaKzk8IIt2MoRkPOMeku07u+eoK+MPjNLuKh7jVzoh9qHQcAY1IZQg29QdFETNFZyeMEWbOUIyHmeq\/WWt0QP3vM07XnqZa\/CRTkLtQ+DgDH4LoYafIOii5rM1cZKFOpbzsFWjjLYgq0PAjxNdMf9h+ivvjs5y51+k7SPd4XahwUjYI4ePUp9fX20a9euKN5r1qyh3t7e1Njv2LGD1q9fH33e1tZG\/f39VFc38\/jl0NWrjy9GJT7QEVRCLzst2IKtHAE5z6HXWxYtu\/7xebp255OJouWy8+dH26JNdhbZRAECxoZWDWwHBgait7JoOXLkCHV3d1NHRwe1t7fPys3evXuJ7YeGhiLRwsKnoaEhVfCEGvwahGnGK0NvrGrJF2zl6IMt2NoQ4Omh+x4\/Qn+++0BiMjXawv\/6Fi6h\/xEezAhMvGbogib+GY++jI6OTo+6xH+O20PA2HxdzW3REZizsrUEW1ti5vZga87K1jIUtmlrWhQPFip\/2f6+6NJFKdGisw+1DwtSwKgRGB6NaWlpMRqBWbZsWeJoDScONfi2jYtv+1AaK99cfPgDWx8Uk32ALdjGCfDU0N8+9EO6\/\/Ejsxbi6qLlL37rQjrvtHdVRbRAwMjVUzHPPPKyZcuW3HUt4+Pj1NXVRVNTU7Rt27ZEoaMPv+kZ3r17t1j+55LjyclJamxsnEtFrlpZwVYONdjObbZTrxyLALz4+r\/TraMv0b5n3kgE0nDKSdHv\/\/SXTqNLzso\/Ldcn1dbW1lnuJiYmfL6iEL6CHIFhsixkWJwkLc7lKaPt27dHa2Dq6+sjW37SFv1iBEamruIvWRmu7BVswVaOgJznotZbnhL61uNH6MH9P0odYWEqPB30X5acQa3vO5WWXzBfDpSl51D7sGAFDI+w9PT00ODgIDU1NU2HW+1W0qeM0mz1EZgQ1avld8C7eVEbK+8FrYFDsJWDDrbhs2XBcvcjz9Hux16MTsVNe1iw8M6hzkvPjMRLNdazuNCHgHGhVsM0+k4jHmVRDwRMDYMSezU6ArlYgC3YyhGQ81yLesti5dU3jtGX7j+UObqiRljKIFjiEYKA8Vxn1ULbsbExK8\/Nzc109913z0qjTwMpkZK2NTppCiltuolfFGrwrcALGNeisRIoRiFdgq1cWMC2vGxZrLz0+r\/Rnd+dzBUrSrD86i+cTms+cmKtXlFHWPIiEmofVrMppLydQkkBUaMqSQImfpCdfjid+qyzs3N6sa5a7MvvwUF2edVf5nN0BDJc2SvYgq0cATnPPusti5XXfnyM7vh2\/siKEifnLJhHHZcupOUXLCitWEmKDgSM5zrrW8B4zt4Md6EGX5KZiW+fjZXJ++aSDdjKRRtsi8OW16fwqAj\/e\/M3D9DE868bj6ywWPnUhxvo5055Z6HXr\/igHWofVrMRGB9BqZaPUINfLX5p70FHIBcBsAVbOQJynvPqLY+qPPX86\/T3D\/3QSKjoIytXXFhPH1z0nuDFCkZg5OrntGd9DUzevUVVyE7mKyBgZCKQ11jJvHVueAVbuTiDrSzbn37PmdGIykMHX6FvPvoiHXzpjcydQHpu9F1B0+KlvrpnsMjRcfccah9W8xEYfS0KL7odHh6ese3ZPWT+UoYafH+E3DyhI3DjZpIKbE0oudmArRu3eCq1Pfk74y\/R\/37wsLVQYX+rLz2Tlp0\/f06OqthEIdQ+rOYCRgUhviupSKMyoQbf5gsgYYuOQILqCZ9gC7ZyBOw887QPP9\/4lxfokUOvGk\/9qBEU\/reMW5ftKMlah9qHFUbA6OHTj\/kvwqhMqMGX\/crke0cnm8\/I1QJsXcnlpwPb2Yx4NIX\/e\/Fff0JDe56JDPY8dUK4mDxqe3Lzz51EN\/z6L0S+inwwnEmZimQTah9WSAGjBz7tQLpqVo5Qg19NhknvQkcgFwGwBVvfBJRIefqFo7T36Zfp0JE3rESKGlHh3T8fvbCeLn1rQa0+0oJ66ztqJ\/yF2ocVUsDoIzAMP++yRZmQv+011OBLc8vzj8Yqj5D752Drzi4vZehsecpn4Xt+hm7ZfZAOvHjUSaQwQ572+aNfei8d\/tFPjEdTQmebV7ekPg+1DyuMgMk6iE4qqKZ+Qw2+afml7NBYSZHFGhg5suVmq6ZmWKT86Ogx+r8\/eN5pJEUfTemI3QNUyWm1aBNkam6ofVjNBYy+C6kIoy1J1SfU4Mt8Vcy9orEyZ2VrCba2xMzty8BW7fDZ8eBh+u74S1Y7fHQS0TqUBfOo5bz5dHnTgugjybUpZWBrXlOKYxlqH1YzAaPvOso7yr\/W1SDU4NeaKxoruQiAbdhslUB57PC\/0lfHnnMeRdFHUprP\/ln6zPLGmi6gRb2Vqbeh9mE1EzBPPvkkff7zn6frrrtu+n6ivNBl3YWUl7aSz0MNfiVMfKRFY+WDYrIPsC0vW32ah0sx8sCzkUCxOdAtPorCPy8\/fwFduPBddPHZp4iOolRCHvW2EnrpaUPtw2omYHAXkkxFLZNXNFZy0QLbYrNVO3p4Oub2bx+iR599zVmg6KMo7284mX7tF06PCr\/8gvlyEIQ8o97KgIWA8cw1fnCdqfvm5mZKuo3aNL2LXajBd2HhMw0aK580Z\/oC29qz5YWyLFBuu+8g8VSP6wiKEij8L+\/s+W+t76UfvnJiZ4\/+mVyJq+cZ9VaGdah9WM1GYGTCJOM11ODL0DL3isbKnJWtJdjaEjO3Z7Z8Xw8\/Pzn2Jv3lfQdp\/wtHvQiU5RcsoPNOq4suHVTipJJdPealKoYl6q1MHELtwyBgDOpLqME3KLqoCRorObxgWxlbNcVz76Mv0sMHX4mc2ZwsG3+7EiE8gnLF4nr60LnviRbLlnGapzKy2alRb2XohtqHQcAY1JdQg29QdFETNFZyeME2na0SJw3z30l\/\/s0D0WFtlUzvTI+ULJhHZ9fPo1XNZ9BFC989nYG5NIJSaY1Gva2UYHL6UPswCBiD+hJq8A2KLmqCxkoO71xlq7YX8797nnyJRt+6j8fH6Amfh\/LzDSfT0tOO0ZlnnlnYnTxytUre81ytt9JkQ+3DIGAMak6owTcouqgJGis5vKGxjW8tfvTwa9HNxi738aRN7\/A2Yx6V0Q9sU6MreprQ2MrVQnvPYGvPzCRFqH0YBIxB9EMNvkHRRU3QWMnhLQtbJUzUtA4Tee3Hx2jXP5444t7H1E4kQt4aPfnVt7YYV3KabFnYytUuOc9gK8M21D6sUAJmx44dtH79+iiCfIHjgQMHaHR0lPr7+6muri4zsvG7lNasWUO9vb2pafhQvNWrV0ef89bsoaEhqq+vT7QPNfgyXxVzr2iszFnZWhaJrX7mSbQo9tCrdNDDuhM1OsLiZNFpddH5J+\/6mZ8W315cJLa29aLo9mArE6FQ+7DCCBi+E2lqaop6enpo7dq1kfhgYdHX10cNDQ2ZYoRDzun54XTqjJmOjg5qb2+fVSPUbdebNm2KTgFm4ZQllEINvsxXxdwrGitzVraW1WKrrzl58MCP6FuPHYmyWsmaE1VWdQ\/P4p97N513+gmBokZslHix5eLDvlpsfeS1bD7AViZiofZhhRAw+qm8ixcvpu7u7kiIsLhQ1wdkjZAkhVwXNPHPWbDs378\/VxSpdKEGX+arYu4VjZU5K1vLStnqa06O03H6+g9epH965lXv4oR37XRfdha9\/pM3xUdObBmm2VfK1lc+QvQDtjJRDbUPC1LAZF1ToKaali1bljg6k1R9Qg2+zFfF3CsaK3NWtpZpbOOLYX\/4yo\/pW48f8bLeRB8V4WkdHkH5tV88g95\/5ruDOvME9da2Nprbg605KxvLUPuwQggYDoSaxtGnkNRoTNpUUNrIy5YtWyjthmslYFauXEl33nknjY2NYQ2MzTfBoy0aK48w33Kl1pscPvws7T9aR9954iWvoyaRSFkwjy468930Kz9\/Gp300z8VHcamppLmwpknqLf+663yCLYybCFgZLjO8KovrFUfbNy40XikRHem1tTEFwArAXPw4MHphbtptsofB19\/du\/eXQUa4b9icnKSGhsbwy+opxLue+aNyNObx4m+9thrNPXKMXr21WPRv5U+DaecFLk482dPog8sfCdd9t666P\/Voz6v9D0hpEe9lYsi2Pph29raOsvRxMSEH+cF8lKYERjfTHihLo\/mDA4OUlNT07T7pCmkNFtdwIQYfN\/Mbf3hr60TxPjSv0g8vOdn6Nb7DtHE86+f+P1bh7DZctXtpy\/8WzCP+KbiFU0LosPYirAYtpJy1TIt6q0cfbCVYYsRGBmuYl6zFv\/yiMuiRYumR3ZYwNx00020efPmxK3UoQZfDL6h49AbKzWtwjj+x\/em6MH9P4rIVHq2icKr30b8gYaT6Q8uP3tamBw6dIgua35buBuGBGYGBEKvtwYIxEzAVgZtqH1YIUZg1KJbXo+S9WSd7aLvOlKjLGnbr+PiJmvHEucn1ODLfFXMvZatsdIPXVOl\/OrYc\/TY4X\/1tgiW\/eqjJvz\/6355EU2+9GOrXTplY2tea2pvCbZyMQBbGbah9mGFEDAcMl7Eu3379hkHyunnuaxatSrzTJj4QXb6Il71WWdnZ7Q1mx99vU3agl9VlUINvsxXxdxrURqrJGHCZ5rsfszf7py4MOHtw\/\/1Q2fSv7+pCZb6eebwciyLwtZbgQrkCGzlggG2MmxD7cMKIWCytj3royVPPPFEdGDd3XffLRPlFK+hBr+qEBNeJt1YxY+p59GMH0y9Rl\/7Jz\/H1OtFUoeusTBZcvYp0W3E+hRPtVlLs612eYr0PrCViwbYyrANtQ+DgDGoL6EG36DooiaVNlb6\/TlvHj9OOx487HUqJz5qcsWF9fTBRe+JmFRyl44o1LecV8q2Gnks6zvAVi5yYCvDNtQ+rBAChkOWN4XEVwKos2JuueUWmShjBKaqXLMOW1MLYE87+T\/Q7d8+RAfqAthSAAAgAElEQVQ83Z2jCqivM2k5b\/6MG4hDOMsEHYFcVQZbsJUjIOMZAkaG6wyvSefA8KWO6r6iW2+9lYaHh2dsi65CtrCI1yNk\/WK\/rd9+nB549s3Iu\/edOQvm0dL3nkKfXnbWjEPWQhAnJuFAJ2tCyc0GbN24maQCWxNK9jYQMPbMgkkRavAlAqQEyrE3j9PfPfRDr7cOR1M30SmwJ9NVV5zYMhz9rv7EsfV43iaAjkCuNoAt2MoRkPEcah9WmCkkmbD58Rpq8F3o6LcPs0DhQ9cqPXBNXwD7yZYG+rd\/P17TBbAuXIqWBp2sXETAFmzlCMh4DrUPK4yA4cPkurq6aGpqalYEm5ubZ2yvlglxutdQg59WYv3CPz7jZOfYc84iRRcnq5rPiHbnsH++PwcdgVxNBluwlSMg5xn1VoZtqH1YIQSMfry\/Ou+Fz2xRlzn29vZOn98iE95sr6EGXy81H2d\/18M\/pCefsxtRUQJlydk\/G03tXHb+\/Gm3edM6aKzkajPYgq0cATnPqLcybEPtwwohYOLnwOhH\/fPC3pGREYpfyigT5mSvIQZ\/5IHDNPL9Z41GVpQQ+eX3n0prrzhnegSl0higsaqUYHp6sAVbOQJynlFvZdiG2IcxqUIKGN4uvX\/\/fuKRl6w7jWRCPdtr2YOvFtaOPPAssXDJelisXN60gH7rkoXii2PRWMnVYLAFWzkCcp5Rb2XYlr0PS6NSCAHDmdPvI9JFy7333kujo6MYgXGo1yxcNn\/zAP3PvbPXFbE7Nf1zW+dFkfe8KR+HLGQmQWPlm+jb\/sAWbOUIyHlGvZVhCwEjw3Xaq74Ohg+tY0GzZcsW4gsZa3H2i17csgWf17Os3f7o9DZjvSwsUn7\/I4302RVnC0c03z0aq3xGrhZg60ouPx3Y5jNytQBbV3LZ6crWh5lSKMwIjGmGa2FXhuDzaMu3Hj9CV\/\/t47MQsWi5reMi8Skh29igsbIlZm4PtuasbC3B1paYuT3YmrOysSxDH2ZTHmVbCAFjepljfX29SxkrTlPk4Kv1LavueHhGOVm0XNl8BnVfdlbVp4ZMgaOxMiVlbwe29sxMU4CtKSl7O7C1Z2aSosh9mEn+02wgYAzoFTX4SVNFarSFz1kp+oPGSi5CYAu2cgTkPKPeyrAtah9WaWlrKmB4t9H69etzy7BmzZpoR1KtnqIFn0ddeDfRwD1PTyMpk3BRmUZjJVejwRZs5QjIeUa9lWFbtD7MVylrKmBUIbKmkHwVtBI\/RQv+khu\/N+MeoIFPLKaV7z+1kiLWJC0aKznsYAu2cgTkPKPeyrAtWh\/mq5SFEDC+CiPlpyjB5ykjfa1LGUdd9BihsZKqsYRrGuTQgi3YChKQcV2UPsx36SBgDIjWOvg8ZXTPv7xAvXeNT+f2r3\/3A\/Rrv3i6Qe6LawIBIxcbsAVbOQJynlFvZdjWug+TKVUNT+JV00ZjY2O5ZZvLlzmyeNnx4GHa+PUT613KPuqCEZjc6u7FAB2BF4yJTsAWbOUIyHiGgJHhWgqvtQp+fLEui5edn724sNuibYOJjsCWmLk92JqzsrUEW1ti5vZga87KxrJWfZhNHl1sg5lCUif57tq1K+JgunNpfHycenp6aHBwkJqamhIZ1ir4\/33PM9Rz1xPTIy8hiRcuFBorl6+sWRqwNePkYgW2LtTM0oCtGSdbq1r1Ybb5tLUvlIBJ2la9ceNG4qsF8h79LiU1PdXR0ZGZVomeffv2ZV5XUIvg8zbpq0YeDVa8QMDk1ejKPkdHUBm\/rNRgC7ZyBGQ816IPkynJTK+FETAsXrZv305DQ0OkTtw1FSJJoHRBkwZSXRrJnxdpBGYuiBcIGNmvNzpZOb5gC7ZyBGQ8Q8DIcI28+r5KwORcGba5\/vrrqbOzM7o4sigChte98Dkv6nnkTz4czJqXeBVCRyD3pQJbsJUjIOcZ9VaGLQSMDFfvAkbdYt3W1kb9\/f1UV1eXmHMe8eFn6dKlRmtgdCe7d+8WoTH1yjH602++QPueeSPy\/1efWEiXnDVP5F1FcDo5OUmNjY1FyEpweQBbuZCCLdjKEfDjubW1dZajiYkJP84L5CXoKaSpqalEEcMLd7du3UobNmwgboyKsoh34J7909cDLD9\/Pu286uICVRX\/WcFfW\/6ZKo9gC7ZyBOQ8o97KsMUIjAzXGV4rWcQbz17W7iIepVmxYgW1tLRQUXYh6VNHoW2XTqs6aKzkvlRgC7ZyBOQ8o97KsIWAkeEq5lUt0NUXBfPLsg7Q27ZtWyRq4o908Fm8rB15lPY89XL0at4uXYbbpCsNHhqrSgmmpwdbsJUjIOcZ9VaGrXQfJpPrfK+FmEKqZLeRKqK+60htj25oaMi9xboIIzD6rqO5MHWEaY78L2alFugIKiUIcShHEGyrzRYCRph4fPoobTQkLRvxg+z0RbzqM95xFB9hqbWAmYtTRxAwwl8mHBIoChjiUA4v2MqwhYCR4ZroVe0k4g95FGV4eDj1lNxqZEsy+F\/42gT9xe4DUTF6V55LvSsXVaNIhXgHGiu5MIAt2MoRkPOMeivDVrIPk8mxmddCTCFlZZXFDK9nia9lMSueHyup4MdHX\/jMl7n0oLGSizbYgq0cATnPqLcybKX6MJncmnstpIDRR2BqfRM1o5QK\/oZ\/eJK+9J1DUbRu77yIOi9daB65ACzRWMkFEWzBVo6AnGfUWxm2Un2YTG7NvRZGwBRt2khHKBX8+qvvi17D26bn2ugLlxuNlfkX1dYSbG2JmduDrTkrW0uwtSVmZi\/Vh5m9Xc6qEALG5Oh\/OQT5niWC\/6f\/5yn6y28dnLOjLxAw+fWuEgt0BJXQy04LtmArR0DGs0QfJpNTO6+FEDB2Wa6+te\/gz\/W1LyqC6Ajk6jLYgq0cATnPqLcybH33YTK5tPcKAWPAzHfwdQEzVw6tS8KMxsqg8jmagK0jOINkYGsAydEEbB3B5STz3YfJ5NLeKwSMATOfwWfxsuqOh4n\/natrXzACY1DpKjRBR1AhwIzkYAu2cgRkPPvsw2Ry6OYVAsaAm8\/g66Mvf\/bri6n7srMMchCmCToCubiCLdjKEZDzjHorw9ZnHyaTQzevhRAwWYt40+40ciuuWyqfwV91+8Nz7s6jNOporNzqo0kqsDWh5GYDtm7cTFKBrQklexuffZj92+VSQMAYsPUVfH30ZS7deQQBY1DJPJugI\/AMVHMHtmArR0DGs68+TCZ37l5rKmDi9x+lFWPNmjW5lzK6I8hP6Sv4A\/fsp4F7no5eOJcX7yri6Ajy656rBdi6kstPB7b5jFwtwNaVXHY6X32YTO7cvdZUwKhsz5VzYNT00VxfvAsB4\/6FNU2JjsCUlL0d2NozM00Btqak7OwgYOx4BWXtI\/h7nnw52n3ED18ZwFcHzPUHjZVcDQBbsJUjIOcZ9VaGrY8+TCZnlXktxAhMZUWQT+0j+PrJu5g+OhEzNFZydRdswVaOgJxn1FsZtj76MJmcVea1ZgJGnzZavHgxdXd309jYWGJpan2ho4\/gz\/V7j5ICi8aqsi9vVmqwBVs5AnKeUW9l2Prow2RyVpnXmgmYyrJd3dSVBl+fPtr8G4upa9ncPftFjxwaK7l6DLZgK0dAzjPqrQzbSvswmVxV7hUCxoBhpcG\/4WsT9Oe7D0RvwvTR28DRWBlUPkcTsHUEZ5AMbA0gOZqArSO4nGSV9mEyuarcayEEjJpOCnUKCbuPkisqGqvKv8BpHsAWbOUIyHlGvZVhCwEjwzXTKwuba665hv74j\/+YmpqaapCDE6+sJPj69NHnPnoOXd92fs3KUbQXo7GSiwjYgq0cATnPqLcybCvpw2Ry5MdrIUZgsorCVwmMjIxQf38\/1dXVpZoePXqU+vr6aNeuXZFN1uF38RGftra2TP+VBB+H16VHF42Vny9xkhewBVs5AnKeUW9l2FbSh8nkyI\/XUgiYgYEBGhoaovr6+tRSsw0\/vb29pARKR0cHtbe3z0ijhM6yZcuiz9TPDQ0Nqaf9VhJ8TB9BwPj5qtp5QUdgx8vGGmxtaNnZgq0dL1PrSvow03fUwq7wAoaFydTUVO4ITByeLmjywPKVBqOjo6nvcA2+fvfR5U0L6B\/+cEleVubU52is5MINtmArR0DOM+qtDFvXPkwmN\/68FkLAZC3i5ZGR4eFhqzUwtlcTSAkYff0Ln7zLJ\/DieZsAGiu52gC2YCtHQM4z6q0MWwgYGa7TXtOmdtRUj+nreeRly5YtlLeuRfnLmm5SNq7BV9NH7Afbp2dHEI2Vaa22twNbe2amKcDWlJS9HdjaMzNJ4dqHmfiupU0hRmAYQNJUkYm4SINnMvWkRBP7yFokzMHXn927dxvFrG3rJE29cowaTjmJdn2q0SjNXDKanJykxkZwkYg52EpQPeETbMFWjoAfz62trbMcTUxM+HFeIC+FEDBZUz6mu5DiTMfHx6mnp4cGBwcTp59MxQv7dVGv+voXXN6YXOPx15ZcSwC2YCtHQM4z6q0MW5c+TCYnfr2WQsCY7EKKY2Hhk5bOZOeR7s8l+Ng+nV9R0VjlM3K1AFtXcvnpwDafkasF2LqSy07n0ofJ5MSv10IImPj6F72IeQtsla2+6yhPoJhML1UqYPT1L0du\/qjfqAXiDY2VXCDBFmzlCMh5Rr2VYQsBI8N12iuPmKxbt27GjiOeBurq6qJNmzZRS0tLZg7iB9npi3jVZ52dnZR283XWjdcuwcft0\/kVBo1VPiNXC7B1JZefDmzzGblagK0rOYzAyJCz8MoiZvXq1TNSbNu2LVe8WLzCydRWwOjrX3pXnku9Kxc5vTf0RGis5CIMtmArR0DOM+qtDFvbPkwmF\/69FmIKyX+x\/Hq0Db5+\/gu2T6fHAo2V33qqewNbsJUjIOcZ9VaGrW0fJpML\/14LIWD0KZ68qSL\/CPI92gZfX\/\/yyJ98mM6pn5f\/kjlogcZKLuhgC7ZyBOQ8o97KsLXtw2Ry4d9rIQSM7cm5\/jFke7QN\/pIbv0c8jcTChQUMnmQCaKzkagbYgq0cATnPqLcybG37MJlc+PdaCAHDxTLdbeQfQb5Hm+Dr61+Wnz+fdl51cf4L5qgFGiu5wIMt2MoRkPOMeivD1qYPk8mBjNdCCJisu5C42Fk7hGSwzPRqE3x9\/QsW8GZHB42VXO0FW7CVIyDnGfVWhq1NHyaTAxmvhRAwMkXz59Um+DjAzpw7GitzVraWYGtLzNwebM1Z2VqCrS0xM3ubPszMYzGsIGAM4mATfLWAF+tf8sGiscpn5GoBtq7k8tOBbT4jVwuwdSWXnc6mD5PJgYzXmgkYfeFu2uFyqshlmULC+he7SorGyo6XjTXY2tCyswVbO1421mBrQ8vcFgLGnFVwlqbBxwF2dqFHY2XHy8YabG1o2dmCrR0vG2uwtaFlbmvah5l7LIZlzUZg4sWP34eUdT9StdGZBh8H2NlFBo2VHS8ba7C1oWVnC7Z2vGyswdaGlrmtaR9m7rEYloURMEkXLKpppo6ODmpvb68ZMdPg\/833D9Pntj8a5RMn8OaHC41VPiNXC7B1JZefDmzzGblagK0ruex0pn2YzNvlvBZCwGQdZMf3I42MjFB\/fz\/V1dXJkcjwbBp8LOC1Cw8aKzteNtZga0PLzhZs7XjZWIOtDS1zW9M+zNxjMSxLIWB4dGZoaIjq6+trQs00+OoGahxgZxYmNFZmnFyswNaFmlkasDXj5GIFti7U8tOY9mH5noplUQgBk7XepQgn9JoEHwt47Ss2Git7ZqYpwNaUlL0d2NozM00Btqak7OxM+jA7j8WwLoSAYRQ8VbRu3ToaHh6mpqamiM74+Dh1dXXRpk2bqJaXPJoEHwt47Ss0Git7ZqYpwNaUlL0d2NozM00Btqak7OxM+jA7j8WwLoyAUSJm9erVM8hs27atpuKFM2MSfP0EXtxAbVa50ViZcXKxAlsXamZpwNaMk4sV2LpQy09j0ofleymeRaEETPHwnMiRSfCxgNc+emis7JmZpgBbU1L2dmBrz8w0BdiakrKzM+nD7DwWwxoCxiAOJsFfcuP3iNfBYAGvAdC3TNBYmbOytQRbW2Lm9mBrzsrWEmxtiZnZm\/RhZp6KZQUBYxCPvODjCgEDiAkmaKzcuJmkAlsTSm42YOvGzSQV2JpQsrfJ68PsPRYjBQSMQRzygo8dSAYQIWDcIDmmQkfgCM4gGdgaQHI0AVtHcDnJ8vowmbfKe52TAkZt2961a1dEeM2aNdTb25tKOy\/4WMDrVlHRWLlxM0kFtiaU3GzA1o2bSSqwNaFkb5PXh9l7LEaKOSlg+GA8fli0mFxXkBf8q0YepZEHDkc+sQPJvGKjsTJnZWsJtrbEzO3B1pyVrSXY2hIzs8\/rw8y8FM+qMAJGnfkyNTU1i1Jzc7PoSby6oEkKUV7wsQPJrWKjsXLjZpIKbE0oudmArRs3k1Rga0LJ3iavD7P3WIwUhRAwakqnoaEhcypHAlnWPUzqfXnBxxUCbpFBY+XGzSQV2JpQcrMBWzduJqnA1oSSvU1eH2bvsRgpCiFgTESEBC4eedmyZQu1tbVlXhbJwdef3bt3T\/849coxats6Gf285oPz6fc\/NF8iq0H6nJycpMbGxiDLVutCga1cBMAWbOUI+PHc2to6y9HExIQf5wXyUggBo0ZgOjs7a3LqLgsZnrpKu\/E6S73iCgH32oy\/ttzZ5aUE2zxC7p+DrTu7vJRgm0fI7XOMwLhxM07FdyHV6tZpXn\/T09NDg4OD0\/cw6RnPCr6+A2nnZy+m5RdgBMY06GisTEnZ24GtPTPTFGBrSsreDmztmZmkgIAxoeRoo6aQxsbGEj1IL+LNE09Zwd\/wD0\/Sl75zKMo3diDZVQA0Vna8bKzB1oaWnS3Y2vGysQZbG1rmthAw5qwKb6nvOjJZQJwVfOxAcg83Git3dnkpwTaPkPvnYOvOLi8l2OYRcvscAsaNWyFTxQ+yM1nEm7YACncguYcYjZU7u7yUYJtHyP1zsHVnl5cSbPMIuX0OAePGzSrVjh07aP369VGabdu20YEDB2h0dDRzh5DVCxyNs4KPLdSOUIkIjZU7u7yUYJtHyP1zsHVnl5cSbPMIuX0OAePGzTiV2gnEi2nXrl0bnQfDa1\/6+vqoFufD6BlPC76+A6l35bnUu3KRcXlhCAEjWQfQEcjRBVuwlSMg4xkCRoZr5FU\/B2bx4sXU3d0dCZiWlhbKW2ArmK1p1yYCBjuQ7COBjsCemWkKsDUlZW8HtvbMTFOArSkpOzsIGDteVtYQMFa4gjFGYyUXSrAFWzkCcp5Rb2XYQsDIcJ32yutfeL2LPoWkRmM6Ojqovb1dOAfp7tOCr3YgRaNIN3+0Zvkr64vRWMlFDmzBVo6AnGfUWxm2EDAyXGd45emi1atXz\/jdxo0baypeODN5Auac+nnRGTB47AigsbLjZWMNtja07GzB1o6XjTXY2tAyt4WAMWcVnGVa8LEDqbJQo7GqjF9WarAFWzkCcp5Rb2XYQsDIcC2F16TgHzzyBvEZMPx0XrqQbu+8qBRlKVIm0VjJRQNswVaOgJxn1FsZthAwMlxneNXPgVEf8HkwvBuplk9S8LGFuvKIoLGqnGGaB7AFWzkCcp5Rb2XYQsDIcJ32yuJl+\/btNDQ0RPX19dHv1e6kIi7i1UdgsIXarXKgsXLjZpIKbE0oudmArRs3k1Rga0LJ3gYCxp6ZcQp9G3V8tKWo58DgFmrj8KYaorGqnCFGYOQYgi3YVp+AzBshYGS4zhhpUYfX6a8qqoC5auRRGnng8In8Ywu1U+2AgHHCZpQIbI0wORmBrRM2o0Rga4TJ2ggCxhqZXQIWKuvWraPh4WFqamqaIWyKOIWEW6jt4ptkjcaqcoYYJZBjCLZgW30CMm+EgJHhOkOojI2N5b6F70e6++67c+18GiQFH7dQV04YAqZyhuhk5RiCLdhWn4DMGyFgZLiWwms8+PoC3uXnz6edV11cinIULZMQMHIRAVuwlSMg5xn1VoYtBIwM11J4zRIwuIXaPYRorNzZ5aUE2zxC7p+DrTu7vJRgm0fI7XMIGDduVqmSzoEp4lUC+hkwfIUAXyWAx54AGit7ZqYpwNaUlL0d2NozM00Btqak7OwgYOx4WVuX6RwY3n3Eu5D4wRkw1qGeToDGyp1dXkqwzSPk\/jnYurPLSwm2eYTcPoeAceNmlKps58DgDBijsOYaobHKReRsALbO6HITgm0uImcDsHVGl5kQAkaGa+S1bAJm1R0PE08jRXnHGTDONQONlTO63IRgm4vI2QBsndHlJgTbXEROBhAwTtjME1U6haREkNqK3dbWRv39\/VRXV5eYCX29TZ5tPPg4A8Y8rlmWaKz8cEzyArZgK0dAzjPqrQxbCBgZrjO8ui7iPXr0KPX19dGyZcuovb2d1M8NDQ3Ep\/vGH\/10XxY4nDbNltPGg48zYPxUBjRWfjhCwMhxBFuwrS4BmbdBwMhwFfPKYmh0dDRxFCb+WZZtXMDgDBh\/IYOA8ccy7glswVaOgJxn1FsZthAwMlzFvGaJkqQRGDV6k5QhPfi6gLn5Ny+k3\/1wg1gZQneMxkouwmALtnIE5Dyj3sqwhYCR4SriVa2HybpDaXx8nLq6umhqaoq2bdtG8Vuw9YzpwdfPgMEhdpWFD41VZfyyUoMt2MoRkPOMeivDFgJGhqt3r2r9CztOW8QbXzA8MDAQ5SNpvQz\/noOvnp+ccxm9vvTT0Y9\/9YmFdMlZOMTONYiTk5PU2NjomhzpMgiArVz1AFuwlSPgx3Nra+ssRxMTE36cF8jLO44fP368QPmpKCsm4iW+4JdfyKMxPT09NDg4OH0TdtoIDM6AqShEMxLjry1\/LOOewBZs5QjIeUa9lWGLERgZrt685u08Ui+qVMBs\/PrT9Gff2B+5wzUClYUPjVVl\/LJSgy3YyhGQ84x6K8MWAkaGqzevPA3E61myzn5RL0uaQspKqwcfZ8B4CxmhsfLHEiMwcizBFmyrR0DmTRAwMly9eI0fYqecNjc309DQUHSYHZ\/10tnZOb1YlwXPli1bIlObg+wgYLyELHICAeOPJTpZOZZgC7bVIyDzJggYGa6l8KoHv\/7q+6I8Lz9\/Pu286uJS5L+omYSAkYsM2IKtHAE5z6i3MmwhYGS4lsKrCr5+BkznpQvp9s6LSpH\/omYSjZVcZMAWbOUIyHlGvZVhCwEjw7UUXpMEDM6AqTx0aKwqZ5jmAWzBVo6AnGfUWxm2EDAyXEvhVQVfP8SOR194FAaPOwE0Vu7s8lKCbR4h98\/B1p1dXkqwzSPk9jkEjBu3IFIlCZidn72Yll8wP4jy1aoQaKzkyIMt2MoRkPOMeivDFgJGhmspvKrg4xA7v+FCY+WXp+4NbMFWjoCcZ9RbGbYQMDJcS+FVBf+qkUdp5IHDUZ5xiF3loUNjVTnDNA9gC7ZyBOQ8o97KsIWAkeFaCq8q+DgDxm+40Fj55YkRGDmeYAu21SEg8xYIGBmupfAKASMTJggYGa7sFWzBVo6AnGfUWxm2EDAyXEvhVQV\/yY3fIz4LBofY+QkbGis\/HJO8gC3YyhGQ84x6K8MWAkaGaym8quDjFF6\/4UJj5ZcnpjnkeIIt2FaHgMxbIGBkuJbCKwf\/2w\/+C\/EIDD84xM5P2CBg\/HDECIwcR7AF2+oSkHkbBIwM11J45eD\/9TceolV3PAwB4zFiEDAeYcZcgS3YyhGQ84x6K8MWAkaGaym8xgUMtlD7CRsaKz8cMUogxxFswba6BGTeBgEjw7UUXjn4X9wxSnwODD84hddP2CBg\/HBEJyvHEWzBtroEZN4GASPDtRReOfhrvvQtGrjnaQgYjxGDgPEIE1NIcjDBFmyrRkDmRRAwMlxL4ZWD\/ytf\/BpO4fUcLQgYz0A1d2ALtnIE5Dyj3sqwhYCR4VoKrxz8D1zz97TnqZfpnPp50TUCeCongMaqcoZpHsAWbOUIyHlGvZVhCwEjw7UUXiFgZMKExkqGK3sFW7CVIyDnGfVWhi0EjAzXUnjl4J\/y6b\/BKbyeo4XGyjNQTCHJAQVbsK0KAZmXQMDIcC2FVw7+yx8fivKKawT8hQwCxh\/LuCewBVs5AnKeUW9l2ELAyHD15vXIkSPU3d1NY2Njkc+2tjbq7++nurq6xHfs3buXVq9eHX3W3NxMQ0NDVF9fn2i76AMfpFd+eSD6rPPShXR750Xe8j2XHaGxkos+2IKtHAE5z6i3MmwhYGS4evF69OhR6uvro2XLllF7ezupnxsaGqi3t3fWO8bHx6mrq4s2bdpELS0ttGPHDhodHU0VPLqAwRkwXkIWOUFj5Y8lRmDkWIIt2FaPgMybIGBkuIp5zRIl\/Nn+\/fsTxU1Shs754K\/Qa8t7oo949IVHYfBUTgACpnKGaR7AFmzlCMh5Rr2VYQsBI8NVzGuagImP1phkoPGK36HXl346MsUIjAkxMxs0VmacXKzA1oWaWRqwNePkYgW2LtTy00DA5DMqjIVaD9PR0RFNKemPEjArV66kO++8M1ozk7cGpuE\/\/xG98b5VkZuT9wzS\/TvuKExZy5yRyclJamxsLHMRCpt3sJULDdiCrRwBP55bW1tnOZqYmPDjvEBe3nH8+PHjBcpPxVlRAoUdJS3iVZ8fPHhweuHuwMAATU1Npa6B0QUMLnKsOETTDvDXlj+WcU9gC7ZyBOQ8o97KsMUIjAxXr17zxAu\/LGkKiRf19vT00ODgIDU1Nc3K08LfuJF+cs5l0e+P3PxRr3mey87QWMlFH2zBVo6AnGfUWxm2EDAyXL15zdt5pL+IR1wWLVo0Pb3EAuamm26izZs3J26lPuOTX6Zjp12IawS8ReuEIzRWnoFq7sAWbOUIyHlGvZVhCwEjw9Wb17xpIP1FfAYM26uzX\/j\/+Unacs2\/h4DxFqYZjtBYyXCFOJTjCrZgK0tAxjsEjAxXL17jh9gpp2pxLtDslvEAAAzOSURBVB9mx+fEdHZ2Rue+8KMfZJd36N1pf\/C39Oa7TsMpvF6i9bYTCBjPQDECIwcUbMG2KgRkXgIBI8O1FF7rr74vyieuEfAbLggYvzx1b2ALtnIE5Dyj3sqwhYCR4VoKr0rA4BoBv+FCY+WXJwSMHE+wBdvqEJB5CwSMDNdSeFUCpnfludS7clEp8lyGTELAyEUJbMFWjoCcZ9RbGbYQMDJcS+EVAkYmTGisZLiyV7AFWzkCcp5Rb2XYQsDIcC2FVyVgcA+S33ChsfLLE9MccjzBFmyrQ0DmLRAwMlxL4VUJGNyD5DdcEDB+eaKTleMJtmBbHQIyb4GAkeFaCq9KwOAaAb\/hgoDxyxOdrBxPsAXb6hCQeQsEjAzXUniFgJEJEwSMDFf2CrZgK0dAzjPqrQxbCBgZrqXwqgQM7kHyGy40Vn55YpRAjifYgm11CMi8BQJGhmspvLKAOad+HvEUEh5\/BCBg\/LGMewJbsJUjIOcZ9VaGLQSMDNdSeIWAkQkTGisZrphCkuMKtmArS0DGOwSMDNdSeGUBg2sE\/IcKAsY\/U+URbMFWjoCcZ9RbGbYQMDJcS+EVAkYmTGisZLhilECOK9iCrSwBGe8QMDJcS+GVBQzuQfIfKggY\/0wxAiPHFGzBVp6AzBsgYGS4lsIrCxjcg+Q\/VBAw\/pmik5VjCrZgK09A5g0QMDJcS+EVAkYmTBAwMlwxzSHHFWzBVpaAjHcIGBmupfB6xie\/TLd87uPRNBIefwQgYPyxjHsCW7CVIyDnGfVWhi0EjAzXUngNNfi1ho\/GSi4CYAu2cgTkPKPeyrANtQ97x\/Hjx4\/LIAvHa6jBr3WE0FjJRQBswVaOgJxn1FsZtqH2YRAwBvUl1OAbFF3UBI2VHF6wBVs5AnKeUW9l2IbahwUjYI4cOULd3d00NjYW1YC2tjbq7++nurq6zBoxPj5OPT09NDg4SE1NTYm2oQZf5qti7hWNlTkrW0uwtSVmbg+25qxsLcHWlpiZfah9WBAC5ujRo9TX10fLli2j9vZ2Uj83NDRQb29vaoSV3b59+2h4eBgCxuy74M0q1C+VN0AVOALbCuDlJAVbsJUjIOM51DobhIBJCvmOHTtodHQ0cxRm7969NDAwECXHCIzMFyfLa6hfquqTnP1GsJWLAtiCrRwBGc+h1tk5K2B4yun666+nzs7OSMRAwMh8cSBgqs+V3xhqg1UbmjPfCrZyUQBbGbahcg1SwKj1MB0dHdGUUtoIDf9+6dKlRmtgZKoVvIIACIAACICAPIGJiQn5l1T5DcEJGLWuhTmmLeLlhbtbt26lDRs20OTkZK6AqXJM8DoQAAEQAAEQAIEcAkEJGBPxwjx4ymjFihXU0tJCJruQUItAAARAAARAAASKRSAYAWO68yi+3VoPx7Zt2yJRgwcEQAAEQAAEQKDYBIIRMDyqMjU1ZXT2ix4SjMAUu4IidyAAAiAAAiCQRCAIAZM2qtLc3ExDQ0PRYXZ8TgzvOIqPsEDA4IsBAiAAAiAAAuUjEISAKR925BgEQAAEQAAEQKASAhAwldBDWhAAARAAARAAgZoQgIDJwM7rarZs2RJZYIGvW\/3kKbqurq5ofVLe\/VQ6b74GIut6B7fchJXKhq0qefzajbCI+CmNDdf49DXaiewY2LDVbdEeVFa31fc+aRlFZZ5rmxoCJoW\/umaA19A88cQT0dZr\/v\/6+vraRqxEb9c7y1WrVs24rypejPjVD\/zz9u3bwTwl3jZsdRfMdf369bRx48bUQx5LVMW8Z9WGa3znI9bTZYfDhq0ShnyXHa9bRHvgXtUV9127dgX3hzgETEq9UHck8RcoVPXq\/pUwSxlv0FkUjoyMGO0UQ2eQ\/5esfou6CVvuFK655hp6+eWXKeuUarPohmllU2fZ9qabbqLNmzfjDxuD6mDLVq\/faA8MACeYqFGsSy65hA4ePBhdbhzSUSEQMAlBT7vdWt127VaV5l4qfRSLR67iP2cRQYOVXV9c2LIov\/TSS+mrX\/3q9M3tc69W+uNqcmEs+L5NwKbOJo3A5F3OC9azCTzzzDPRL3knbnd3NwTMXKgkSSMu3PgvWrQIw+4WFSA+KmDzF6vruT4W2Su1qS1bdX3G1VdfHV1iCjGeHH4brixg9u\/fHznCWrn8r5MNW\/amT32sWbMm6nzxuBGIC0I3L8VLhRGYjBEYfcETBIx95bVtsNQbuGO49dZbsYg3A7kNW+4IvvjFL9KnPvUpamxszFyLZB\/lsFLYcFXridTCXU67bt061NuUKmHDVk19bNq0KZrysBm9DatG+ikNBIwfjqXwgikkP2GyGTKGeLFjbsOWbe+\/\/\/7oL1jsQsrmbMM1PoUEtmBr9y2unjUETPVYF+JN+ogLFvG6hSQ+ZZS30BQ7Dcw527DVt6frb8Cw\/GzeNlzj9RntRHb9tWELcWjeFphYQsCYUArIBtuoKw+mzbZJDL\/b8bZhq3vGKEE2Zxuu8U4B0xz+2CZNIWF6zq6N0K0hYNzZlTYlDrKrPHRZB1epRZA8tZE2SoCDwdJjYMoWAsauHttw1Q+yw2Fr+Zxt2LIgXL16deQUbPPZZllAwFTGD6lBAARAAARAAARAwBsB7ELyhhKOQAAEQAAEQAAEqkUAAqZapPEeEAABEAABEAABbwQgYLyhhCMQAAEQAAEQAIFqEYCAqRZpvAcEQAAEQAAEQMAbAQgYbyjhCARAAARAAARAoFoEIGCqRRrvAQEQAAEQAAEQ8EYAAsYbSjgCAf8EnnrqKVqwYAHxbd4mD5\/38NJLL9H5559vYu5ko87saW5upqGhIeO8leWGcVW+ap09Uu33OQUdiUCggAQgYAoYFGQJBJiA7cmu1TisqhIRUknaatYIFhT8VPP247KwqWYc8C4QyCMAAZNHCJ+DQI0IFFHA2OZJR1eWThoCpkYVHq8FAUsCEDCWwGAOAj4J6EfRs181LfPEE09MH6POv1dXKsSvXFDTHKeeeip1d3fT2NhYlD11UaO622fXrl3R702mffR36NMofPXD+vXrp4u\/ceNGam9vn4UjzU4JmFWrVtENN9wQpYtP0+jHxyvH6j3M6pprrqHLL788Sq\/K8uKLL1JXVxdNTU1FSa699lrauXMnDQ4OUlNTU\/S7tDIlxTIuYPjnV199NfpPccy6CDN+ESG\/I+l3ZRR3Pus+fIFApQQgYColiPQg4Egg6WJFvfOMj3ak3dDLr+\/v7yf2xyKGpz5aWlpIiaOOjo5poZF147fKj\/JXV1cXdby33norDQ8PR2IgbwQmbq9fyscii4XGJZdcEuVX+d++fXu0loaFSE9PzwzhoftTIu2cc86ZTh8vo\/r5+eefj\/Lc2NhIfX19kVBSU0J5F4cmCZgtW7aQLqSYs85VrwIQMI5fCCQDAUsCEDCWwGAOAr4IJAkM3XeeWIj\/ZR8XMJx+ZGRkurNn+6zbqJOmeOL2WXnKu+k6fsMw5ydvWkn\/XAmYuCAbHR2dUUZdoPA7brrpJtq8efOMxcZZ00RJAoZHd5ToYp9ZHCBgfH1D4AcEsglAwKCGgEANCejTLfHpnbROMj7N0tbWljgCE5\/K0YuZNP2T9j791vCsjjtvEXGSWEn6XXxaLT5NpkaYuDxJQkT3yaM66kbjeJjTpoGSBAyn1Rf1ZgkvCJgafqHw6jlFAAJmToUbhS0qAb3T1tfBcGeqtiorQRJfl6JGIOIjMJw2PnKQVf40cZI1raX7q1TAsC+1lkUJrKQRGBsB89BDD5GaojLdig4BU9RvCfIFAjMJQMCgRoBAgQjoIkCNMLCA4fUivJZj2bJlMxbO6iIlLmCy1rskFbkaU0jxNS76O1lsZE0HqSkkXcAkjXboU0g8ArNu3brpNTwmoZaYQsoTk3lTaSb5hg0IzDUCEDBzLeIob2EIJK2B0UdB9EWtSYtR1YiMmkLigukiR\/nnBb1q+iNpHYoC4msRrz7ioa+LWbp06axFunEBo6dVeeX88YLcJAFjuoiXfag1LHlrjypdxKum+NTOMVUOffFyvBJCwBTma4mMlIgABEyJgoWshkdAdW5qC7A+PaRvgeYplY997GMztkqzcLnyyivpuuuumx5hiIsaNSqjtlczQdWxptHM2nJsurA4abu1yRqY+Ls3bdoUrXPhhbuq\/PoIDJchzjC+jTq+lZzTpG0BV6Ne\/K8SfUnbqPX0SeXS1x9xnJYsWUKPPPJIJKLiQlOVIT46FV5tR4lAwC8BCBi\/POENBECgxgRYUCTtPDLNlskaGFNfpnYYgTElBTsQeJsABAxqAwiAQGkJxNf5qNEW\/dwX28JBwNgSgz0I1IYABExtuOOtIAACngjETyfOOiXX5JXxyxXvuuuuKJnU3Ui4zNEkKrABgdkE\/j9dHjKDmFx\/NgAAAABJRU5ErkJggg==","height":337,"width":560}}
%---
%[output:038a5c0b]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:7c8883f0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:734836a0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
%[output:545cfcb0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber_zvs","value":"     4.500000000000000e-09"}}
%---
