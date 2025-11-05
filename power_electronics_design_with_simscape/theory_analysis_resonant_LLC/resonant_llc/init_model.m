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

model = 'resonant_llc';
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
kp_i_dab = 0.25;
ki_i_dab = 18;
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
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000e+00","2.000000000000000e-04"],["-1.973920880217872e+01","9.968584073464102e-01"]]}}
%---
%[output:9e69d693]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:20880618]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     2.000000000000000e-04"}}
%---
%[output:266aaba1]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"    -1.973920880217872e+01"}}
%---
%[output:7a235d1a]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"     9.968584073464102e-01"}}
%---
%[output:53464e72]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["7.338637605205141e-02"],["3.802432508328568e+00"]]}}
%---
%[output:2b42b309]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["2.831309534039184e-01"],["6.766822226281998e+01"]]}}
%---
%[output:19de2814]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:5159e62b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:85434b31]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","2.000000000000000e-04"],["-1.973920880217872e+01","9.968584073464102e-01"]]}}
%---
%[output:613a9f65]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["3.110176727053895e-01"],["5.433217222799691e+01"]]}}
%---
%[output:6480ce36]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAW0AAADcCAYAAAC74PBGAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ2QF8WVf5qNZ8yCmGhYVsmZsH7lIqvhw0TNpiiuVismFpI7XMgFKOACInoqoFcEcx7ubUoXV4mrtyRACZcC3Mp5i8ld0IvxNBIRNYgxhybLyYUF3CxR+TiBxMjVG+x\/ent7pnvm3zP98X9TlYrsf+b1e7\/X\/Zs3r193n3Ds2LFjQBchQAgQAoSAFwicQKTthZ9ISUKAECAEIgSItKkjEAKEACHgEQJE2h45i1QlBAgBQoBIm\/oAIUAIEAIeIUCk7Ziz3njjDZg0aRL09PRAR0cHNDY2FqbhLbfcAl1dXTBhwgRoa2srtfv444\/DqlWrYMWKFVBdXZ27PnF6PPjgg9Dd3d1Pt9yVkTSAeMyZMwcWLFgAc+fO1Vbh0KFDMGvWLJgxY0ZqvxbtAzSK2ckMFPuFruHbtm2DqVOnRrevWbMG6uvrdR+1dh\/2taVLlxY+BnUMJtLWQanAe1wjbdZ5x44da5W044i8QNdETTHi3b59eyoCKsevNnzA68swTvuSYs\/5SNpM51NPPRU6Ozuhpqam6K4W2x6RtjOucFMRG4QhI2hXSJtFn2lfYj6TdrlffD6SNns5b9myxbloOwjSVkUF\/O\/33HMPPPzww4DOwEs2+FSfhTyBXHnlldGncppoRJQ\/aNCgUtQmG9x8p58\/fz6gDQcPHiylMZLkoV5xhCf+Pe7f\/OtEFm0lpTPwE1PEmN3P5Mb9jp\/jS5YsiVIKzF\/sGUYkDBvEI84HYv9AG\/ASdeMHKv7O+4XJZrqLOIg28c\/KdDzrrLNKERx7MbI2+GdFuXgPa1tH37hQIMkHYn9CGby+okxRD\/ydT6WIpL169eooDRcnV9SNb5v35R133BF9\/WEqkfWhHTt2RKkY1h+wn2zcuHFA2k8HOxsBi07o5j1pyzoYM5x1HBmp8+AkDSAZscgGEi8vKTIRB6g4UIcOHTogpy0b9PgctoM5XiQf8ZINfDEnaYq0mQ\/iBhfDI8kP\/LO8XkmkjVjxA1T2clH5ng12HMRsLkHEUqY\/7+O4\/sBs6u3tHaAn++2RRx5J9B9PcPxLaeLEiUp9ZQSg44OXX365XyCSRNpJ8lh\/40l78ODBsHv37n6q8S9tFZb4oMxP+CK77LLLYvsD\/yJJ0pn3q6xf65Bq3vd4Tdr825J3PE\/k6ISRI0eWHM2TA38f73QEnU2YyD6T+I4lG9BxEzZ8Z2H38PLxb7feemsiafN26sjDCcWskTbioBNtyL4OZB2eyYr7smCY6KZHxKhXxBJtl7XJvwQZnt\/85jcHRGNiKoRFcXH9g0XAPB6yv4l9Zv\/+\/aX+xuvG7pP5QIaRTupG1we66RyZPKYH8zPixV6ufP8Vn2X38XlkURYf1IjRP8Mkbown9S8Zdq6mdbwm7ThQdYiQvQ2TUh3iG1N0etJnPV99weTodIKk9Ah+8vGRgI48bDtv0ubJPW5g8D4RUwts8LLBdvfddw8gUFVOW4zQkgao+DK699574eabbx6QghG\/gjBixlRYHFnwkbBYVaIiQVn0F0fa2I4sZSTqK1ZppPEBH9EmfTmq\/IJyZC8i\/u\/8S1AcKyzNwV4APGnz\/SjJNp0vNxl2Kp\/lHVHHyfeatJM+X3hHyaJXGWnX1dVJP1XZvSoiUHVgnc+tNKStI68o0uZn2++\/\/3644YYb+pUtJg0A0Q5d0lalxvjUStyLAl+8LS0tMH369Ehf2cUII460ZTldJidragWfjyPtpFQOPifLxePf0\/hAh7STiJLHMS64EP8+YsSI2JeRirSTbIvjgiRf4wuPSDuH11KWSDtuAgkJmU0qJk26pCFB0WSdyFg1EcnXuerI4\/XlvwxkqQTZS0cnPYJtiPJwoimuPRORNo8Ta4ePQFUvWN4uPtJOKmvTfUny8xaynDkjYzE9h9G5TnqEtzNNGZ7tSFuMqMX+u2nTplLQJGIkI23+CyBLpK3Cjkg7B9KWDVxc\/FFuThs\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\/a1hu+0UUwTZpIA94kmnZkkQ\/t4G6yVRd5ZgBpiwfsykh7\/fr10NnZCTU1NSbxSSXLRTBTGaC4mQa8STTtyCIf2sHdZKsu8kxspF1bWwttbW0DIuukE9JNgqWS5SKYKp3T\/E4DPg1abt5LPnTTL2m0cpFnYnPac+bMgY6ODuju7i6lRzDHvXTp0ujvjY2NaWw3fq+LYJo0kga8STTtyCIf2sHdZKsu8oxW9QgDwaWjyFwE02RnoQFvEk07ssiHdnA32aqLPEOLa0x62KAsGvAGwbQkinxoCXiDza57\/g2Y\/w93wZbvzAc86NyFi0jbBS9IdKAB76hjUqhFPkwBlqO3fvmBrbBpx9vw8i3nuEva\/KIaFY42c9sufrao8ErzOw34NGi5eS\/50E2\/pNHKC9JGg3BxDU5ELliwoN8iGiz\/YxORQ4cOtbrQhkg7Tddz897QSS10+7BXhW6jF6TNFtewkj9xuPMlf1hNwi+8MUkN27Ztg3nz5kF7ezvU19cPEE2kbRJtO7JCH\/Ch21cJpH1R87PwmzeP+JEeaWpqki5Vx2ibLa555JFHSv9tcqENe3Fs3749WkpPpF1lh1VzbjV0UgvdPiLtnAdIjHjl4hpVpJ3H6kh8Maxbty5qmiLt4VBVRaRtZ3iU1yqRdnn4ufC0F5G2bk770ksvhVmzZkFcGiUr4JhPb25uhsWLF8Odd95JpD2cSDtrX7L9HJG2bQ+U3\/5HbnkyEuJ09QgzU1ZFgnWKuOcIbhyFhI0X7v6H\/zZxsTaRsHGik3Lae2E4kbaJrmVFBpG2FdiNNYq5bIy0vSFtY5anEISTnHjhnie6E5Fr166F0aNHp2jFj1tpwPvhpyQtyYd++3DzzkNw9YNbibTj3MjSImz3QF3SRnm4x\/e0adP87iGC9kePHoW+vr5oJ8VQc9qh2xi6fdhlQ7bxu092Q8crx+eTnE+PqBbYsDSJyYoRjLK7urqkxCvWi+NNrOSvtbUVRo0aZXWb2DzeFocPH4be3t5oFVaopB26jaHbh\/0+ZBu\/8p1X4NmdB\/0gbbEWG3f600lZmCQv3Ugb68Rd2RPApP30aW0STTuyyId2cDfRKp\/Pdj7SFg9BwLTFqlWrShOOWI7HSNwEOHEyiLSPwN69NBGZZx\/LWzaRdt4I5ycfN4q6ft32UgNOp0dE0kbyXLRoEaxcuTJKQYj\/zgs2Im0i7bz6VlFyibSLQtpsO2KUXbXvNfh5y5ec+aJXnhGJJD5z5kxoaWmJViYWRdoqN9AydhVC7v8eOqmFbh\/2sNBsRMLGCBt39mPX4Mdvg2c2PuIuaaOi\/FJ1jK4xx11XVxcta6eDfYshw9AGgwy10G0M3b7QSBsJG0v88P\/ZNW5IL2x9aFG0x5Irc2ex+2nzk5G4F8ikSZOgp6cnUtz2ob4IKEXaxbw88mwldFIL3b6QSFtMiaBtH\/\/IydDxlyfAlClT\/CDtPAerCdlE2iZQtCsjdFIL3b5QSJttv8qPBiTslxZ\/zsngkE6usctbsa3TgHfUMSnUIh+mAKvgW2W5a6bCA5MvgMljaqJ\/uhgcDiBtceJRxFLMdxeMdak5F8E0iQUNeJNo2pFFPrSDe1Kr33tuL9z48KvSWzC6fnTuxVFahF0u8gyRtnv9KtKIBryjjkmhFvkwBVg53YoR9aqf7YZv\/+Q3sS0gSbc3XQCX1w0ZcI+zpM0OHdiyZYsWdLJl5VoPGrzJRTANmkekbRJMS7KItIsHHkn6l3sOwYNP7epXtifThOWtk7R0kWdSR9rFu0HeootgmsSGBrxJNO3IIh\/mjzuS9Lef\/A289sb\/KUkatUGivvPqOvjyyDO0lHORZ2giUst1xd9EA754zE23SD40hyirncYIGiNpfvGLqhUk6u9\/vR5OqjqxX75a9Rz+7ixpq3b1E41zoVbbRTB1OoHuPTTgdZFy9z7yYTbfIEFv33sI2v9LneKIS3vMuOxMuHHcx7MpwD3lIs9QpF22W\/MRQAM+H1yLlEo+jEebRc7\/8twe2Pw\/+1NFzrxUjKKHn3YyfOuac2DwyVWpI2lVfyDSViGU4ncXwUyhvvJWGvBKiJy\/oZJ9yC8FX\/v8XtjU\/TbseutIvyXiaR3ICHrJ1XXw0Q9\/MHqcL89LK0\/nfhd5JjbSxo2h8ESYgwePbwKO16BBg2DNmjXRxlG2LxfBNIlJJQ94kzjalBW6D5GY0cbNr+6Gf3319\/DeMcgcMYvRM\/77a5+thb\/+zNBCyDmun7jIM1LSxj2058yZA2JpHy6sWbp0KXR0dEBjY6PN8eDkBIFJQEIf8IhV6Db6bh+Lln\/0y33ww5f7yo6UxfHBIuc5XxgOF9YePxw878g57Rj1grRZzXZtbW10Wo148RtJmTqFPS2QeL+LYGaxI+4Z3we8Dhah2+iyfYyQt\/UchO8+0wPHDEXJccSME4OjPj7YSWJO6qsu8oy0Tht39Gtqaoq2YhUvWsauQ0fl3+PygC\/fuuMSQrfRhn2MjN87dgy+\/\/NeeOpXb0VYpymRS+Pf2sFVMGxQFYwdcTr87efPgp63jkYrC1EP16LmNHaxe70gbYq0s7jW\/DM2Brx5K5Ilhm6jSfv4ib3\/3P476HrptxG45U7uJXmIkS5WZ1w6Ygh8deywAZGySRuL7n867XlB2mgI5bR13JnvPaEPBoq0j\/efZ7rfjiLSX\/\/2HfjZjrdhy879uUbGrNfyhHzhmdXwtUtq4cN\/9oFSp9aNkkPvp96QNnqOqkfyJWWV9NAHQ8ikzaLiw4cPw4YXd8FPd\/0x96iY7088IX\/x06fDVRceX7KtS8Sqvsn\/Hno\/9Yq00zjOxr0ugmkSh9AHg2+kzYj4j8eOwcZX9sF\/vLKvUCLmSRfTFTipN2l0DVRniI6pn+oj4CLP0IpIff8VeieRdr5w8zlizAs\/uq0P\/nvvocKJmCdjjITHn\/dRmHDxx+CE983PIzo2iWzo\/ZRI22BvcRFMg+YFX1mRR6TNiHj\/4Xfh0W2\/hc2vH88P5zlZF+dzJNt33303qqz43DlnwF99ZigMOaWYVXwm+6FKFpG2CiHzv8eW\/GGd9ooVK8BmLXaSuUTa5jtD0RKTBjyboEMiPuWkE6HrpT7YuuuANRKWpSeurj8DTq8+qQQbEjVf6hY6oeXx4i26D6rac5FnYkkbT17nLxdWQfL6uAimqgOk+T2kAc+nIhADTENg2dqvet+Jvij6DkNZe1KkwZW\/l5+ww\/\/GsrbLR5wGJ7yfmyg3NRGSD+MwDt1GF3lGmdNm5X+808aOHWs9CncRzKzkIXvO1cEgEvCBw+\/Cv7\/SF5Wu2UpFMPx4EkbivfSTQ+Caiz8GH\/rg8VK2ckk4rX9d9WFaO5LuD91GF3lGSdq8w9jCmz179kBnZyfU1Bw\/sdjG5SKYJnEoYjCwT3lGxH\/443vw4+1vwg9\/0WedgHmSxWoJvK741Efhyk+fDid94MQSCbu88q4IH5rsc1lkhW6jizyjJG22SRTvUN0zIvkoXWeHQFlUH3fggotgZun0pj87S6Vp7x2Dn7z2JmzddRD+93eHo8k4vMRI2aTOKlliOmLM8FOgbtDvoXbYMKiqqvKCiFU28r+HTmhoa+g2usgzWjltHcIVOzMS8MKFC0tbueK\/m5ubEyN0fEE8\/fTTWqkXF8FMM6BV9+JgePG1XTDsfUJ7efdBeGL7m9Dd946TUfBnP3EqXPEXp8ORP7wXpSHw5SA73bqSSC10QiPSVo3ifH6PJe3Ro0dLd\/nTVQN3A6yrqyttOsVSKzNmzIjd1hWfwUu2u6DYrq+kzW\/o88Nf7IPHfln8Ig0RSz7Xi6mIkWcNgnHnngbnDv1w6dY88sGhk1ro9hFp67Kh2fuU6RFTzTHSbmhokO4eqPrdJ9JGYt7z9hH4px+9Hqmd1w5rMt+IBPypYdXwhXNPg0+\/v18xPpMHAWfpJ6GTWuj2EWln6fXlP1MYaau2dMXDhWfOnAkHDhyA3bt3R5YlHSDsSqSNBH39uu25ETOfBx5xxikw5uzB8Pm603KNgMvvVnoSQie10O0j0tbr56bvyp20+cnFpAlMtkHV7NmzS5F4EtEz0l67di1gKqeoa\/POQ3DXY6+XRdK4BzFemIrAKHjiRUOh6gPHi4MZSdOAL8qj+bVDPswP2yIkYyCJ61WmTJkSzbVhEOnClTtpMyNV+3TLwEDQ4g5kYKSNz+FZltOmTcsFzz0H3o3k3vHjffDi7uMVGKqLkfLnz\/4QfPXiU6N9JNjfVM+y348ePQp9fX1RWSWrrNB91pf7QrcxdPuwn4Vs4+rVq6NCCrwqkrTRcJ0KEp5wkvLcjLRbW1th1KhRudSMP9P9Ftz0\/W5gxC0jQ4yMLxsxBP7uC8OM5opxW8\/e3t7o7R4qaYduY+j24XgI2UYMGjds2ADLli1zm7RZbrmlpUV66roqN42OjCPbJNKW\/cYi7cWLFw+oOMkzp4156qsf3CqtaWaHkT4w+QKjJC2+EOjT2pfvhXg9yYf++zBPnsmKjrTkDycEyyFtVAbJffny5aU67aRUB94v+z3pEOG8wMR89V2P7RyAJ5L1o3MvzpWo+UZpwGft0u48Rz50xxdZNcmLZ7Lqg89FpM0i4y1btmjJ0l0RKa6m5J\/Dicd58+ZBe3t7KaJnxM02q0ra48Q0mHHRddFkzRxAA16rKzp9E\/nQafdoKWeaZ7QaVdyUOtI20agJGSbBxM2OMB3CX7bImkjbRO9wQwaRtht+KEcLkzxTjh78s4VVj5hSmMkxBea659+I6qz5C8\/V+96MC02rnEoeDfhUcDl5M\/nQSbekUsoUz6RqVCfSZmkJPPgAc9nTp0+P6hPjrqRFLyaVS5JlAkxZhI15a9WeGUXYSAO+CJTzbYN8mC++RUg3wTOm9azYSBtz2Bc1P9sPT1cIG5WiAW+6qxcvj3xYPOamWyTSNohoOWDKJh1dImwibYMdxaIoIm2L4BtquhyeMaTCADEVGWn\/81O74BsbuktgbL7tEjh36Cl5YZxJLg34TLA59RD50Cl3ZFLGC9IWy+5klvqc0xbz2Fgl8tLiz2VyaJ4P0YDPE91iZJMPi8E5z1a8IO0kANIuQ3cNTHFHPlcJm9Ijefac4mQTaReHdV4teU\/aCEyagwryAhLlZgFTLO9zLY\/N40UDPs\/eU4xs8mExOOfZShaeyVMflJ06p62z90jeSmcl7Y\/c8mRJNZejbIq0i+hB+bdBpJ0\/xnm3QKRtEOG0YIq5bMxju3KCiwwWGvAGO4slUeRDS8AbbDYtzxhsOlZUqkibTVKWe36kCcPSgCnWZOPiGUyNuHzRgHfZO3q6kQ\/1cHL5rjQ8U5QdWqex88okbeJUlNJp0yNiLtv1KJvSI0X2pPzaItLOD9uiJHtB2gwMzF3jaQ0rVqyA6urq6ACDhQsXlrZaLQq0uHbSgPnlB7aWjgfDAwt+cL3bUTaRtu3eZaZ9Im0zONqUkoZnitJTmh4R98JmyrDzHjs6OgYcSlCUwqwdXTDFXLbLFSM8hjTgi+5R5tsjH5rHtGiJujxTpF4DSFt1lmPSwQRFKq4L5leWb4MnX3szUs31ihEi7SJ7UP5tEWnnj3HeLejyTN568PJjc9pNTU2lU9H5B3wq+RMnIG+74my47YpPFIlv5rZowGeGzpkHyYfOuCKzIkTamaEb+KAOmL6mRtBaGvAGO4slUeRDS8AbbFaHZww2pyVKmtOOS4GoUidaLRq6SQdMHycgGTw04A11FItiyIcWwTfUtA7PGGpKW4yUtPH8xqlTp8L48eOhra2tJCxuglK7NYM3qsAUUyO+TEASaRvsJJZFEWlbdoCB5lU8Y6CJ1CJiF9fIDvt1YXc\/ZqEKTB9rs3nv0YBP3Zede4B86JxLUiuk4pnUAg08EFs9MmPGDOtlfUn2qcD0OTWCdtOAN9C7LYsgH1p2gIHmVTxjoInUIlJXj6RuIacHVGDym0M9MPkCmDymJidN8hFLAz4fXIuUSj4sEu182lLxTD6tJkuNjbQbGhqkJX82lJS1mQSm7\/lsirRd6WXl6UGkXR5+LjztBWkjUDgROW\/ePGhvb4f6+noXsBugQxKYdz32Otz12M7oGZ8W1PBG0oB3stulUop8mAouJ2\/2grRDOG6Mz2df1zAc\/mlCnZMdIkkpGvDeuWyAwuRD\/33oBWn7AnMcmCGkRig94ksvTNaTSNt\/PxJpK3wolhlOmDChX504\/3gcmOIqyDfbxnnZc2jAe+m2fkqTD\/33IZF2gg\/F1Zbs33ETonFgdjy9CxZ1dXudz6ZI2\/\/BTj4Mw4fOkjbLY9fW1kJLSwtMnz4denp6YlHPY5ENTn4uWrQIVq5cCTU1x8vzcCvYVatWlfb01om0+Xw2lvlhuZ+PF0VpPnqtv87kQ\/996Cxpy6B14RAEJO3m5mbo7OwsETnTNQ5Mvj7bt6XrvB9owPs\/4MmH\/vvQG9J24RAEFv3HbRErA9O3w3uTujQNeP8HPPnQfx96QdqqnfzyPgSBn4xMSsMwMNeuXQt40DBe9zyxq1SfXTu4CvAsSF8vGvC+eu5PepMP\/fYhBo6YJp4yZUp09CLykQtX6mXsRR6CkHQuJSNtBBF3JJw2bRp8\/ZE34MXdRyJcR515Mnxnol9L1\/kOcfToUejr64vSQlVVVS70FeM6hG5j6PZhhwjZxtWrV0dn4uJFpK05\/JMqSBhpt7a2wqhRoyJyq130bEny\/PFnwfzxwzVbcu+2w4cPQ29vb\/R2D5W0Q7cxdPtw1IRsI0baGzZsgGXLlrlN2ugIG4cgyCYddUibvQHFRTWYGsEl7L5e9Gntq+coPeK\/5\/5kgRc5bVTXxiEIbOIR89Ps4IWkVIwIZiiLalg7yRn6AAAOlklEQVR3IdL2f+iTD\/33oTekjVDbOARB3PdEZyKSRdpP\/eotuKbjpaiX+LpJFN\/FacD7P+DJh\/770CvSdh1uEUx+UU3DOadB13UXuW5Con404L12X6Q8+dB\/HxJpG\/ShCOZFzc8C5rXx8vHQAxEaGvAGO4slUeRDS8AbbJZIO0cwQ1kJSTltg53EsigibcsOMNA8kbYBEJkIHsydR6rh6ge3lqT7XjlCn9YGO4pFUUTaFsE31DSRtiEgUQwP5k\/3VsH167ZH0kOYhCTSNthRLIoi0rYIvqGmibQNASmS9rd+ehDWPf9GJP2yEUPgB9dfbLAlO6JowNvB3WSr5EOTaNqRRaRtEHcezOs29MGmHW9H0m8e\/+dw+1WfNNiSHVE04O3gbrJV8qFJNO3I8oa0ZTXaPGR57Ked1iU8mF96aFepcuS2K86G2674RFpxzt1PA945l6RWiHyYGjLnHvCGtHEZ+wsvvCDdx9oVVBmY3+t6HJC02RXCJCTaQgPelZ6WXQ\/yYXbsXHnSC9JW7WPtIpgj235NpO2KY1LoETqphW5fJQQXRNopBrTqVgbmdfd0Ak5E4hVK5UglDIZKsJFIWzWK3f\/dC9JWHYLgCswMzG90\/Bss3PgmkbYrjkmhR+ikFrp9lfDi9YK00RG4TeqcOXOgo6MDGhsbUwzD4m5lYA5pWga4uAavUMr9KmEwVIKNRNrF8UFeLXlB2uJOezIwXKoe4Uk7hD1HGN404PMahsXJJR8Wh3VeLXlB2nkZb1ouA\/PtCStLoom0TaOcr7zQSS10+yrha4lI2yAHIJhNs26EA413laQ+OvdiuLxuiMFW7ImiAW8Pe1Mtkw9NIWlPjlekzadJxo4dC\/fddx\/cdNNN0NDQAHPnzrWH4vstI5iTbvxHOHT5rSVdQqnRroQIphJsJNK2ThNlK+ANafOnoG\/atCk61HLFihWwY8eO6OTz2bNnWyduMdIOqdyvEgitEmwk0i6bM60L8IK0xcN08ZxGRtrV1dUg\/tsWqgjmxCUPw5Hzr45UINK25Yns7YZOaqHbVwkvXi9IW1wRKSPt9evXW1\/ijmDiHtrvnn5+xBohlftVwmCoBBuJtLO\/0F150gvSFhfXiKSN+5Ls2bMnSpdg5G3rOk7aL8G7p58XqRDKRlEMTxrwtnqWuXbJh+awtCXJC9JGcPjFNd3d3aX0yJo1a2Dp0qVOLLpBMHGjqPdOOZ1I21aPLrPd0EktdPsq4WvJG9JGZ8gW2QwaNAiQuOvr68scruU\/jmB+sfNwSVBI5X6VMBgqwUYi7fLHuW0JXpG2bbBU7RNpqxBy\/\/fQSS10+yrhxUukbZBHRNJ+s22cQen2RdGAt++DcjUgH5aLoP3nvSJtWXoEF9nYnoBkbuRJO7Ryv0qIYCrBRiJt+6RbrgbekDa\/uIbPX2MlyfLly53IaxNpl9sd7T8fOqmFbl8lvHi9IG1W8jdjxgzptqxpSv7w3q6urhI7qLZ6ZVUrPJ3E7SjIk3ZoNdqVMBgqwUYibfuBQbkaeEHamBaZOXMmtLS0SKtEMNrWWVwjkvu2bduiJfCtra2xe3SnWW3Jk\/bkMTWAO\/yFdNGA99+b5EP\/fegFaYvL2EXYdSJtlg9fvHhxP4LGZ\/Fqa2uTelP1O\/8QT9ohbcnKbKQB7\/+AJx\/670MvSBthjju5BiPhchbXJJGy6mUhup9I2\/8BETqphW5fJaS4vCBtnZNrdHLOIqWoTnlnaZkDBw7A7t27o8eTTsjhSTu0hTWVMBgqwUYibf8DCy9IOw+YWRSNsuNKBlnOm9\/2NSl\/zpP2C38\/JtrlL6SLBrz\/3iQf+u1DDCR7enpgypQp0VYeGES6cJ1w7NixY3kqwggbN5nq7OyEmpoa7eaSonOetG8c\/BRMmzZNW64PNx49ehT6+voivKqqqnxQObWOodsYun3o8JBtXL16dVTejFfFkHY5hI1AJeW5GWljhN31N8NSvQxSs4uFB3bu3AnYabCSx5U3vGkYQrcxdPuwP4RsIwaNGzZsgGXLlvlB2ixdcfDgwdJYTbNhlE5KhCcBnPxsbm7uF43HVaHgc4y0T3xnH+AxY6Fd7LNs7dq1wZJ26DaGbh+OudBt9CY9wqpHFixY0O9YsTTVIzqlgTzRylIhSTIQzJFtv4aqfa9B9TN3h8bZZA8hQAg4gsAll1wC69atc0QbgAE5bfEQBFFTHTKWRelMDtu\/BM+bnDdvHrS3t5cW8YiVK6q9TpC48X90EQKEACGQFwKYnnQpRTmAtFWleborIvMCkOQSAoQAIVDJCOQSaVcyoGQ7IUAIEAJ5IiAt+TOR085TaZJNCBAChEClIhBbp11u9UilAkp2EwKEACGQJwK5L67JU3mSTQgQAoRApSFApF1pHid7CQFCwGsEiLS9dh8pTwgQApWGgHekzerIt2zZEvlKVcvtskPFeQNxMZOou1jHnrQLogt2p7WP1zlpNawLtjEdstjIFqmhjDSrjG3YncU+\/sQq1\/uoLqZoU11dXb\/FhrrPmr7PO9LmF\/cgGLNmzYLa2trYgxVMA2ZKnlgPrzrZR1Y\/73LNfFr7RFzZwFcdUWfKH1nkZLER7XrhhRdK2zWE5kPRHp3FeFmwL\/IZ1hdVQVVROnlF2khs4ipK2d+KAq+cdmRHqyUdt5Z2b5ZydDPxbFr7+DbR1jvvvBNwb\/Wk4+lM6FmOjLQ2yl7MLn9RpLVPtsGby\/apfM90379\/PwwePBgmT55MkbYKNPF3GXGpDiJO20ZR98tO8cFBvWjRIli5cqXWroUuD4is9rGX8O233x5tICYeWVeUf3TaSWujrP\/qtGPrnrT2hUba+NLq7u6GJUuWRF\/0DQ0NRNppO6PszZ\/2mLK0beZxf5zOab8akAQWLlwY7flbX1+fh6qZZGa1j39u4sSJMGnSJGdJO4uNrP9ee+21MH\/+\/AhbV3PaWexDe8T0iMvpH93O7RrHeJUeIdL+UzeTnfSj2wnzvq+cAY+bzePpRigjRNLGM1YnTJhQmoPBPr18+fJgXryMuNFOvHwuFGDjhEi7DMYg0j4OHiPs8ePHOzkBm4W0xa8Ml1M\/6IMsNib1X9cm07PYJ9shNO6Q8DJooPBHibTLgJxy2u4TNnNv2nwoXwYndhE+Mi2j+xh\/NIuN7Euiurq6pI9MjnFlMwhMa19czt5V+3QhIdLWRUpyXyVXj\/ARNn\/4cRlw5vpo2soDURnXI22WBhBJOKkCSDbR7Boh8H5I60Mi7VyHVEm4Vzlt1JrqtJucmMFWdc8sNcy8TB9IO4uNYtTpak4bfZHWPkqPqEaFmd+9I+1KWhHJD\/Ck9IErRf9il1Stpkv6bPaBtPmvH3aWqugLmY38ikFXq0eYL9P6UByfrtunQ6OufQ15R9o6INM9hAAhQAiEigCRdqieJbsIAUIgSASItIN0KxlFCBACoSJApB2qZ8kuQoAQCBIBIu0g3UpGEQKEQKgIEGmH6lmyixAgBIJEgEg7SLfKjcLyrfb2drj33nuBX5GnggAXTWzcuLGwJfN8eWOW1ZCyvcdVNrryO2+7jX07+HJEV0tJXfGVLT2ItG0hb6HdLBvSyxZM5Km6CcI1ISNPG5Nku7Arns\/42fJbke0SaReJtuW2fCBt1Qk+OhD6TDpE2joerux7iLQD8j\/bUY2ZxFajjRgxItrEnZ2rib+zY7zEcyfxN5aSEH\/jV7eJK994mUmQJskUV30mraZLWnnH2rjqqqtg69atJbtl5xUmtcnaOO+88+DJJ5+Enp6e0lajbOtY\/BvDDP9\/z5490dayLP0kylele0TSjrNFlTqJe3HpLJv3+aUX0HCONYVIOxAvy7bAVJ3XJxuc4j7dsvQIew63E2UEpbMFp2xLWUZq7CWiE2nL2ufJaOjQodFe3EioTC6zA93NdJZFtfwZjki87GXHn1Ups4Plgnkyxb898cQTpb2ymd6jR4+OnR+II220heWYZbaI3ZhIO5CBLTGDSDsQ3+p8VovpkbhnVJtyxUVr4qG1IrSy9IxIQDt27ICpU6cmng0p01t26o1IjvxzqBsSe1NT\/w24+JfGpZdeGpE2T\/T43zI7xRcJs0PckVH1cosjbdEW1cuNSDuQgU2kHa4j+Y194mb9k3La4sZALGJExPgT75OivKTjz5ImNHmi6u3tVZK2KjefhbD4qgmW6mGkzR9QEGeHiAseASc7kUaVeogjbfHlooras2DARodKx3BHkR+WUaTth5+0tJTlmXkCF8lOzC+zfGtSpC1rg1cuLg+dRDJpSNtkaoDPNzO90RYW6ctIO8kOHjckbXbklsx5cS\/WONIWDzhWVfUQaWsNGS9vItL20m16SrPokRGESNpx6Qwd0k57PJapSNsUaTNyFtMXsvRI1kh7\/fr10NnZCTU1NXoOkxyMG0e+FGlrQxrcjUTawbn0TwaJA1tGxmK+VszNiukR\/HdcLlyVV5e9JEzltFmuGas3WlpaYPr06QPy1XwuHtMwc+bMKU1UMtT4iVFZpJ02p93a2gqNjY0lp6hy0XnntMXJUVn3p\/SI26RApO22f7S1k00OipNesmoSvrqBEVJXV1e\/U7Tj0ip89YjO6fB5Vo\/wto4cOVI6ychjxCJt\/nBkPq+P1SJxpJ1UPcKX9IkEqfOVkFQ9IpZp8vjLOor4kmQYqQ4mINLWHnZWbiTStgJ7Po2KNcHYiqxUDU9Z4VMmSNLswr\/jxU+i8fXfYgmdrPY7yToxjy7WTqsiUSZbp05bnLwTX2zi5Cvqcv\/998MNN9wAWK2xZMmSfpOwvF2y+QD2e1tbW+lWU3Xa48aNK9WKo3Cx3lv20pSd8nTttdfCHXfcUSpDlL3sibTzGZ+mpBJpm0KS5FQ0AqaOpNKtHskTbCLtPNEtXzaRdvkYkoQKQ0CWm9dZaagDE5G2DkqVfQ+RdmX7n6zPgICs7FG2RD6D6GiSl5UKYq183KRqFtk6z9Aufzoo2b2HSNsu\/tQ6IUAIEAKpEPh\/73jLxDaB3iUAAAAASUVORK5CYII=","height":176,"width":292}}
%---
%[output:038a5c0b]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"     1.320000000000000e+01"}}
%---
%[output:7c8883f0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"     7.500000000000000e-03"}}
%---
%[output:734836a0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"     7.500000000000000e-03"}}
%---
