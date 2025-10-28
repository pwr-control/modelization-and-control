%[text] ## Settings for simulink model initialization and data analysis
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
simlength = 3.75;
% simlength = 1;
transmission_delay = 125e-6*2;
s=tf('s');

rpi_enable = 0;
rpi_ccaller = 1;
%[text] ### Settings voltage application
application400 = 0;
application690 = 1;
application480 = 0;

n_modules = 2;
%[text] ## Settings and initialization
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
z_afe=tf('z',ts_afe);

t_misura = simlength - 0.025;
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

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:1dd96712]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:174c5e41]
%[text] ### AFE simulation sampling time
dead_time_DAB = 0;
dead_time_AFE = 0;
dead_time_INV = 0;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;
%[text] ## Grid Emulator Settings
grid_emulator;
%[text] ### Nominal DClink voltage seting
if (application690 == 1)
    Vdc_bez = 1070; % DClink voltage reference
elseif (application480 == 1)
    Vdc_bez = 750; % DClink voltage reference
else
    Vdc_bez = 660; % DClink voltage reference
end
%[text] ### Current sensor endscale, and quantization
adc_quantization = 1/2^11;
%[text] ### 
%[text] ### DAB dimensioning
LFi_dc = 400e-6;
RLFi_dc = 50e-3;
%[text] #### DClink, and dclink-brake parameters
Vdc_ref = Vdc_bez; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi_dc1 = 900e-6*4;
RCFi_dc1_internal = 1e-3;
CFi_dc2 = 900e-6*4;
RCFi_dc2_internal = 1e-3;
%[text] #### Tank LC and HF-Transformer parameters
Ls = Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/8 %[output:31c2a36b]
f0 = fPWM_DAB/5;
Cs = 1/Ls/(2*pi*f0)^2 %[output:538b2e49]

m1 = 12;
m2 = 12;
m12 = m1/m2;

Ls1 = Ls/2;
Cs1 = Cs*2;
Cs2 = Cs1/m12^2;
Ls2 = Ls1*m12^2;

lm_trafo = 1e-3;
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
%[text] ### Single phase inverter control
flt_dq = 2/(s/(2*pi*50)+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d); grid on
% [num50 den50]=tfdata(flt_dq_d,'v');
iph_grid_pu_ref = 1.75;
%[text] ### DAB Control parameters
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 0.25;
ki_v_dab = 45;

%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:23e72b9a]
Iac_FS = I_phase_normalization_factor %[output:2e20a02c]

kp_rpi = 0.25;
ki_rpi = 18;
kp_afe = 0.25;
ki_afe = 18;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);
res_min = s/(s^2 + 2*delta*omega_grid_min*s + (omega_grid_min)^2);
res_max = s/(s^2 + 2*delta*omega_grid_max*s + (omega_grid_max)^2);

Ares_nom = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Ares_min = [0 1; -omega_grid_min^2 -2*delta*omega_grid_min];
Ares_max = [0 1; -omega_grid_max^2 -2*delta*omega_grid_max];
Bres = [0; 1];
Cres = [0 1];
Aresd_nom = eye(2) + Ares_nom*ts_afe;
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
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso_pll = [-1 -4]*omega_rso;
Lrso_pll = acker(Arso',Crso',polesrso_pll)';
Adrso_pll = eye(2) + Arso*ts_afe;
polesdrso_pll = exp(ts_afe*polesrso_pll);
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:0cc65d7c]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:8c1b6188]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:995db2c2]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:13ed32a5]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:7b651ad6]
%[text] ### 
%%
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;

kp_rc_pos_grid = 0.35;
ki_rc_pos_grid = 35;
kp_rc_neg_grid = 0.35;
ki_rc_neg_grid = 35;
%%
%[text] ### Settings for First Order Low Pass Filters
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:38b72f5b]

freq_filter = 50;
tau_f = 1/2/pi/freq_filter;
Hs = 1/(s*tau_f+1);
Hd = c2d(Hs,ts_inv);
%[text] ### Lithium Ion Battery
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
figure;  %[output:9482b399]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:9482b399]
xlabel('state of charge [p.u.]'); %[output:9482b399]
ylabel('open circuit voltage [V]'); %[output:9482b399]
title('open circuit voltage(state of charge)'); %[output:9482b399]
grid on %[output:9482b399]

%[text] ### IGBT and snubber data
%[text] #### HeatSink
weigth = 0.50; % kg
cp_al = 880; % J/K/kg
heat_capacity = cp_al*weigth % J/K %[output:619496e7]
thermal_conducibility = 204; % W/(m K)
Rth_mosfet_HA = 30/1000 % K/W %[output:5f4ae634]
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
Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH % K/W %[output:13430917]
Lstray_module = 12e-9;

Irr = 475;
Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2 %[output:104f42b6]
Rsnubber = 1/(Csnubber*fPWM_DAB)/5 %[output:71e550b8]
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
model = 'sst_single_phase_dab';
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

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":35.8}
%---
%[output:1dd96712]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:31c2a36b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     3.551136363636363e-05"}}
%---
%[output:538b2e49]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     1.783252832105145e-04"}}
%---
%[output:23e72b9a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:2e20a02c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:0cc65d7c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.283130953403918"],["67.668222262819981"]]}}
%---
%[output:8c1b6188]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:995db2c2]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:13ed32a5]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000200000000000"],["-19.739208802178720","0.996858407346410"]]}}
%---
%[output:7b651ad6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.311017672705390"],["54.332172227996914"]]}}
%---
%[output:38b72f5b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.073386376052051"],["3.802432508328568"]]}}
%---
%[output:9482b399]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARYAAACnCAYAAADUrHMrAAAAAXNSR0IArs4c6QAAH7JJREFUeF7tXX+QV9V1P2piWQkOLmCddVkXhVVTWxQSh1JbSIjFtAMmpC0\/MiMFhhAUtxNh+LHgIDMBF8JmMoBpGIdQ8sfuOozMCE4TQhkhGrRGUFqjDUTYJYQwUVAxKE3V7Zwn5+vZ+73vvft+3Hffdc+byUT2e+9953zuOZ937rm\/Lunp6ekBeQQBQUAQyBGBS4RYckRTmhIEBIEAASEWMQRBQBDIHQEhlhSQPvfcczBjxgxob2+HMWPGpGghvIra9nvvvQcbNmyAuXPnQm1tbS7vwjaXLl0atNXa2go1NTWA7+3u7oapU6fm8o6wRh577DE4cOBA5b1RL9u0aRNMnDgRRowYEStTkrKxjV0sQDjt2rUL6urqYOvWrUay6PA1fWee5c6ePQtz5syBJUuW5G6ncXIKscQh5Pj3tWvXBk6\/ZcsWa8Ry8uRJmDVrFtx\/\/\/1WiYUMfdq0abHvQQLauHGjkTMnKZukO48ePRrgMmnSpMA5TZ+yEAvKi\/Zz6tQpIyI31c+kXGmJhToVQcGHogPqtHPnzgV\/379\/f9XXhL76+PvIkSMrTkl\/f\/DBB4O\/YdsPP\/xwqJGHycCjCmwfv\/4kD9bBL1t9fX3wd\/za4TNv3rzAOKlNcmI1QuH\/xghi2bJlQX31i6kzXpWE4jDEdhcvXgwLFiyAw4cP95ITnTXs3fiezZs3BzKNHz8e9u3bVyEA\/pXHBjm+KgEQ0dC7qSzvP+r7pqam4Ouryqkri1Eklx+JgSIz1SnCZFD\/rmtD1ZX6WGej3A7J4RHDMBvFtvB3ajMKc1VWHknbjK6jCKaUxKI6HzdIctiDBw8Gxjxo0KDA4BoaGgLj4V\/fyZMn9wr50ShxCMM7KywaUL+u3GmPHDlSGQoRsZA8FLbzLwW9Fzsc5eXRQRSxoINERSyIS2dnZ0CS+CAOWEdHYDoMsY6KGQ6FEP81a9ZAW1tbpV3Cl3RBEiB8ue5hOJEu\/OvJy+7Zs6dXhKKSEJZtbGwMPgJEGuRAalmOKRES4cKdgfqYflP7Ii5iCetj1SbwnWqfd3R09MKeoiKSgWwU69LfdJiTP1Bf7ty5sxeOSaJEk0jEtEwpiSXsy4YG0NzcXJUf4OUPHTpUZaDkfEQI9GWMCqGxMxctWqQNxXURC3Us5iuiOjNJxBJHLNTW+vXrg\/7meZ8kGIYNhXRffcz3YBRG+Qb+HiJ5clSOg0ryiBN9hdWvOeqi65uwL7OOhPgHI2w4oMtn8dwT4aIbCkX1MUUsJ06c0JI+OSfpzyNaXYSB5cIwV0mL2wT2g0qepsSQtVwpiUX9SnNw4ohlx44dQQjJHxpGnDlzJtL5eJ040iEnpq8TJxaVPHi7eRILGTDqR182ysUkwVAlFjJudKiVK1fCqlWrgvYxukFi4U7LcSIjp+Er6Y1fX0w+88gSiUUdqnGC0UVY6FwYpUSRqDoEJRlMyEsdXkYRS1Qfq+3gv3k0SYTNcQmLmlB+tS85NmTTKhHQx5N8hyJOxL2Ip5TEkuRri0BFRSwcRPVrYEoe6syPacSiC7\/zJBb+ZR8yZEhlGKT74keRs0os3JARX\/4VTxKxcOyjEpo8V0FDAB1hheWl4iKWMEfKI2LR9XEUsagfRpV0skYsqq4SsTBETHIs9PWiMXSSHEvY2Jx3itohuq8EtqOLWNSvDH5VaIx955139vp6UThMMqmGFTcrxL\/6PGlngiFFISqx6HSl5CXPsZAur7\/+emVoFJdjoWhHJawoGfgQixyT+p8StXwGqcgcC+nD+1gd9qnkoeaWMElOhKojFp5jUTGXHEvC+Is7DJ8R4V+DAQMGBKGxGubGzQqZEAuKm2RWiA+F8L\/DZgwomqAZF8r+hxEL10W3bkYdz\/O1LiYY4vAGH5rBQgLhM0WIPUZD+PBhlo1ZIT7zwmXHsB4fwgz7G8mMIhi1LE\/wYr0ks0I6cg6bbo6bFSKbUIlF7RfEV02Oq33d52eFEDQ0zHXr1lUtJlI7QjfujeOfMq0RiJP1k\/h70ohKFwW6WLDlc19kwRyjT9MFiXlilGuOhZxenXolgXF4sXDhQmhpaTFawahTVIglz+5P3pb6ccAWkqxAdmXoyTUtT420mH9iVt6i0WCIisSiIw++PiKv5enl6X6RRBAQBAiB3CIWJI1t27bB\/PnzYcWKFVpi4esiUICoVa\/SRYKAIOAvAhViURcfRamk5kYwVFu9ejXMnDkzWFlqMtxxGab5210iuSDgBwK9iAUXQuGCqKhhChKCWk6dPUHV43aD0rhx7NixvfbqXH\/99X4gJ1IKAp4hsHfvXhg2bFghUuc2FDJJ0OJQCB\/aj6KbPUJiOXbsWCHKZ32JyJoVQX19wdV\/XKuGQqhSli366swPJ5Oona8EpRiV\/0aVVQOxgawIuifsXhGL6vhxwxkb6vtkVMePHy8stMyKtciaFUF9fZ9wLdK3IodCLmZxilQ+q6n5ZFQia9beFmJJgqBxjkWXtE3yItOyQiymSCUrJ8SSDC\/T0j7hWqRvhRKLOv1c1LCoSOVNjSesnE9GJbJm7W2JWJIgGLqOpSgiUYUVYknSfeZlhVjMsUpS0idci\/StCrFg4hb\/53qpfZHKJzEgXVmfjEpkzdrbErEkQTCXBXJJXhhXVoglDqF0vwuxpMMtrpZPuBbpW7ks6Y8DP8nvRSqfRC6JWLKiZV7fJ2f1SdYifct4VsjcLLKVLFL5bJIC+GRUImvW3pahUBIEhViSoKWUFWfNAF5EVcHVDq5FfrSFWDL0oThABvCEWOyAF9GqEIsnmxCFWOz4huBqB1chFiGW3C1LnDV3SIMGfcJViEWIJXcv8MkBRNbcuz9o0Dmx8F3OdB1E2HGTeUNQpPJZZRcHyIqg\/zMtPtlAkb5Vlbzl13ZOmTIlOMd2+fLlgJdNF3GNQJHKZ3ULn4xKZM3a2\/6TYJG+VUUsfBcz3u5GxIKEY3J0ZdbuK1L5rLKKs2ZF0H9n9ckGivQt7XQz3rqGF3vPnj0btm\/fHpy8v2DBgl53A9sxqWLHgVl18MmoRNasve0\/CTonFoSQX+2J\/y7qqo4ilc9qauKsWRH031l9soHGW26Hrpeft9NpSquyQC4DzD4ZlciaoaMjqvqCa8cvTsN9Ha\/C2e9+wQ4QQiz54eqLUaHGImt+\/c5b8gVX58QSd3FZXgdAYR4HH7wgnD8yFOrbDiAkaKf\/1+7ugrW7j7uNWPAQ7a6url5OT38bN24cdHR0QGtrK9TU1KRCgfI38+bNE2JJhWDySr58WYVYkvetSQ3nxBJ2aDb9vbm5GTZs2BB7Y2KYstTOiBEj4Pz580IsJlaRQxkhlhxA1DThC67OiYUWyB08eBC2bt0KSAB0hero0aPh7rvvhieeeCJ1xIJDIIx6uru7q6Ii7DcZCvVtB5CIxU7\/4zAIycV58ladbm5vb4empiajC9\/DoME29+\/fH0QpuuGWEIsdoxJnFVwnP\/IiPPPaW+6JxUZXYLSyefPmXk2reRaMWPDBC6zL\/pw8eRLq6+vLLmYgn8hqp5t8wHXChAlw7m\/XwodXDP5kEgvvWolY7Bh6WKu+5AIkusrfLk6cvQC3fvvZoGGnQyH1alVSdeTIkZkujBdiyd9oTFsUYjFFKlk5H3ClNSxOiQVnbRYuXAgtLS2wY8eOINE6ZswYwGFMY2MjTJ06NRnyCUtL8jYhYIbFfXAAUkVkNexUw2K1DzwVlLz03TfgjR\/8o2GtbMUidzfv2bOnMnMjdzdXAy0OkM34ZNhmBz9qFYdACzpeDZK2+HzmmXVw4vkf233pxdZDz2MZO3YsjBo1ChYvXgzr1q2DQ4cOQWdnZ25DoTDtJGKx0+9Cgn0LVySVyd9\/EfD\/8Wmo7Qfnfvh1OFbQ6YzaTYg8OsGoZdmyZYFwOOWMwyKbjxCLHXSFWPoGrkQklKwlUtl5720w\/nOfdUssdrrArFUhFjOckpYSYkmKmFn5MuHaffYC3HZx9oekx0gFSQX\/v0jfisyx8AviJcciORYzV8teqkzOGqeNa1nVPAonlE3TboY7hg+sqOCEWOJ2NaN0eLB2ls2HcZ2EvxepvIk8UWVcG1US+UXWJGiZl3WB64HX3oLWnxyvJGW5tDxCUbUo0reMIxZzqLOVLFL5bJLKGSdZ8Qur78JZ0+piW1aMSA52n4OtB36rJRLKoTRc1Q82Tb85GPKEPUX6lpwgl9ai5PCkDMhFV7XtrHkKnqesSCJP\/vfr8JOX3wglET7UmXBTLfzLF6+LJBOuqxNiMRkK5bnytgysmtXA8jSqrLLE1RdZ4xBK93saXJFAfv36u\/C9\/+iGE29eqEwJR0mAkchf3TAQlkwcZkwkpRoKpYM3v1pFsmpWqdMYVdZ3pq0vsqZFLl10heRx9YDLYdHjR+DEmfdiIxA1T4JDm\/X\/cCP0+\/SlqYlEiIUhIMRSrAPYeVu2Vn0gQVovsvWp\/4GDp3uMIw+VQPDf45tq4YEvmQ9p0qJbpG9pcyz8ilVSoogZIXxXkcqn7SCq54MDiKzpehmJ48OeHmjb0w3dZ95LRRw8H4IRyD+PvRY+d92VuUUgSTUr0rcir1jlB13TJWYy3fxxdwqxJDVts\/I2caVIY\/BnPg0rd70Gvzp9PhNpoEY0E3PHDQPhK7deDU1\/2t8ZeUQh7JRY4s68XblyJfCFc2amYl6qSOXNpdKXtOkAWWVT63\/SZSXCQL1\/9NwpeP742wEEpsnRKLwrxDH8Kpj42UEwsn5AUBz\/7hOuRfpW6BWru3btqjrzFodD6nUdeTtAkcpnld0no\/JVViKMd\/\/4AXS+cBoOdZ8Luo127GbtQyINHKr8\/Z8PgS\/fMrhCGiZt+4Rrkb4Vuo5FPexJrlitNjOfjKpMshJZYA7j319+I1i3kVd0wfMaQVRxVT+4blANzPuberiy36cSkYYQiwkC+jKyQC49dl6FwbaJhcjizXf\/D3a8+Ht48cRHkUUeQxHeRTzCaKitgWmfvyYYkkStOM3QxbFVbeMaK0CCAqWIWBLIm2vRIpXPKrhPRpVEVp6vqBv4J7Dpqd\/AU786Cz096aZV43DmZIFlx193GXxtbBNcV9svWDzmijTi5Mbfk+Bq0p7NMkX6lnav0Jw5c6ChocH6hkMdiEUqn7UTfTKqnx8+CkOHDoX3P+yBH7\/8Buz+Zf7Dj7DIYmhtP\/iLawdU8hfBECViT4tPuPoka5G+ZbyORXcdalbHFGJJjyBFFb9\/54\/w2Aung2lTG0OPMLK48Zr+Qd7i8ssu7dN5CyGWjDkWvGwM17Js2bJFppsvYpmHUfGjA1\/93flgqvSXp\/5gnSR41IAJzoZB\/eCfRl8DjYM+vo\/b1RAkD1zTU3aymj7JWsqIpa6urjL9rIOe35wYtkpXXdGr29RYpPLJTKi6tM6o+IzHz468Cf\/Z9Tb85uyFIIkZRBQXzyDN+m61vpqnuOma\/rDwzkb43\/c\/DIp+8PbvYNiwYXm\/1kp7PjmrT7IW6VuhORa0GNPohF8ZgjcDLl26FPAwbvWqEF4O74T2YShERFB\/VT\/43t5u2HfkLEBP\/rMdUUSBsx9furkWRjV8tBw8TULTJwcQWa3wdaHbZXKfbqaoZPr06VUHb+Pl8mvWrIG2trbQ4VSRrMojCDoHI+\/pUXXIgYnM24ZeCV+8qRY+dekluecnwkxSnNWOs\/qEa5G+lSux0HAobChksujOtvJhZ4QmMTsadgypAbh9+BC4b\/xQeP+DnsJIIomsVNYnBxBZ0\/RwfB3bvsUlyJVYqGEkkAMHDkROV9PBUrhFgF8pgsrjk+el8J2Hz8F3fnY2Fvm6Kz9amTn62n7wtVsGwKArLgv+TX9XG\/DhQnCSWWSN7f5UBXzAFS+Fp8fpvUKpEGaVTGaQaMik5mLyYtWoyIQiDjyNC0\/lSjv7IV\/WrJairy+42sE1L98ykc74MO2o6z\/U33BaGh91wyJGMvhgUhfzLXTLIk\/k5qH85EderNqkhuSx5Z4\/g9ENV5rgYlRGHMAIpsSFBNfEkBlVyMO3jF4EABViMTnzNuqwp7DpZk4m6nSzbmNjFuWf+fVbwbWS\/Jk5pg6+Zel0LnEAUzNLVk5wTYaXaeksvmX6DipnHLEkbTht+TTK47Dnv06+A\/f828uV10bdr5JWNrWeOEBeSPZuR3C1g2sa30oriZXkbVphsF4a5R995iQs2XE0eG0RhEL6iQNk6enwuoKrHVzT+FZaSXoNhVatWgXNzc2waNEiOHz4cFWbZbz+Y\/\/RN+Gr\/\/pS4aSCLxQHSGt20fUEVzu4OiEWO6okbzWJ8jynUmSkIhFL8n5NUkOIJQla5mWT+JZ5q\/qSXg+Fah94ykmkIsSS1ewkYrGLoL51p8QSNTtUpqEQzv5gxILPzntvgzuGDyy8r+TLagdywdUOrk6JJUwlXJsybty4qv0\/eUNgojzOAt367Wcr0cpLK\/4ybzGM2hMHMIIpcSHBNTFkRhVMfMuoIYNCxkOhqAVyBu8xLmKiPF8Ah6SSduWssVAhBcUBsiKory+42sHVxLfyerMxsZgs089DqDjlO35xGu7reDV41Tf+uh5av6o\/fiEPWeLaEAeIQyjd74JrOtziasX5Vlz9JL+Hnseim25ub293PhSiaMXFLJAKrDhAElMzLyu4mmOVpKRTYkkiqI2yUcrz3MoXmmrh8W+OtCGCcZviAMZQJSoouCaCy7iwc2JRNwjifp\/Ozk7jE+WMNdUUjFK+LLkVElscIEtPh9cVXO3g6pRYwk6AMzljJQ84wpQvy0wQ11EcII8er25DcLWDq1NiKeul8HyVrcuZICEWO0YvuNrH1SmxoHoYnWzcuLFyKj8tmsOT3lxdCs+Ttq7WrUjy1r7x4xskYrGDs3NiQbUwzzJr1iw4depUoKXrS+Fp+f4dNwyEnffdZgf5hK2KAyQEzLC44GoIVMJipSCWhDLnVlynPF+78sj0m2H656\/J7X1ZGhIHyIKeJG\/toBfeqlNiKWqFbZj6OuXLNhtEsgux2HENwdUOrk6JBVXCfUGNjY1VF47ZUbd3qzrlyzgMklyAPWsQYrGDrVNiKdvuZj7N7GoXc1g3iwPYcQDB1Q6uTonFjkrmrarK8\/yKEIs5jmpJcdb02EXV9AnXTyyxqKf06\/YeqcqXcZpZcix2nFRwtYurE2KhpK3NM29xfUxXV1ewFiZstzRXng+DyjTNLA5g1wF8igJ8ktUJsdg1lerWkVg6OjqqrmHlyvPVtmUbBkny1p7F+OSsPsnqnFhsbkLkw6G4odDa3V2wdvfxwILLsoyfu5NPRiWy2iFCn3B1SixFbUI0uRT+GztOw8HfXggs4uD9jXYsI0OrPlwITuqJrBk6OqKqD7iW4lL4ojYhmlwKj+faYp4FD3Uqy\/4giVjsOKjgah9XpxELqmdrE2KSS+F7JW6HXwU7773VPvIJ3+BTGCyyJuxcw+I+4eqcWBBTG5sQk0w3lz1xK8lbQ89LUcwnZ\/VJ1lIQSwp7yKUKKV\/mhXEy3ZxLV4c24pOz+iSrEMuxY8A3Hp797hfsWnLK1n0yKpE1ZSfHVPMJVyEWRixlTdzKUMiOowqu9nAVYjl2DMq6o1lmL+wZvgwx7WLb54ll3wuvVK5QLdPBTmq3+xQGi6x2nNYnXJ0TCy63nzFjRlVPFHUp\/I9+egjw0nd8yrjiVr6sdpxUcLWLq1NiCVsRa1flj1tH5bf99BDcfZFYyrhHSBzArjX4FAX4JKtzYlm1ahWsXLkSamtr7VqQpnVU\/paFj8Mzr70V\/FrWGSFJMtozDZ+c1SdZnRILmgs\/3sCe+ehb5sRS5hkhIRZ7luGTs\/okq1NiKcPRlG99ZUtgtWU8g0VmhewRigwx7WLrlFjsqhbfOipPxLJk4jBYMrF8u5rFAeL7MUsJn6IAn2Tt08TScPuX4Q93LA7sUogli3v2ruuTA4is+fU7b8k5sfDNgpMmTYLFixfDihUroKWlBUaMGGFH64utcmIp84yQ5FjsmYEQix1snRILkUpdXR1MmTIFtm3bBsuXL4edO3fCgQMHqo6SzBuCur\/7Fly4aXLQrBBLfuiKs+aHpa95NqfEwg96OnPmTIVYkHCKmIa++p4fwPuDbwz6rsxTzRKx2HFUwdUerk6JBdXCmxDxMvjZs2fD9u3bYf78+bBgwQIYM2ZMcMK+zYeIpexTzeIA9qxAois72DonFlRLXdb\/8MMPF3Ll6uBvbocPrxhc+qlmIRY7xi+42sO1FMRiT73oliu7mocPDHIsZX7ky2qndwRXO7gKsXgw1SxfVjvGL7jaw9U5sahn06KqOO3c2toKNTU19jQHqJzDUvYZIXEAe2YgEYsdbJ0SC59upkQt\/Q3VtU0uNBQSYsnXuMRZ88WTWvMJV6fEkvZeIdy4uGzZsgDvsHNb1EhIV46IpcznsPhoVD45gMhqhwSdEgvNCKn3KuMUdGNjo3ZmCK8KWbNmDbS1tQVHLdB0tRrdIGktXLgwcgUvEUvZ17DIUMiO8Quu9nB1SixRu5tJ5biT5MIufFcJSAchEosPa1jEAew5gEQsdrB1Six5qBQW3fDhEr5HtzZGiCWPHqhuQ5xVcPWaWEwPiQo7AhOJpeznsEiOxY6TCq52cXVOLGmnm6PyMCpkYZfCI7FcfuLn8Ox3vm4X5RxaP3nyJNTX1+fQkv0mRFY7GPuA64QJEyrKHzt2zA4QSquX9PT09PC\/6aab8fewhCzVxd\/HjRsX7CcKe0wuhUdiKfs5LKRfkV+ArNYgsmZFUF9fcNXjUkUsaaabddeF0II6PG4Bn6lTp4IaCYXlWK449MMgapFHEBAE8kXAWcRC0cmuXbtg69atwcFOOJsza9asYPWt7d3N+cIorQkCgoALBKoiFhLCZAbHhcDyTkFAECg\/AqHEUn7RRUJBQBAoKwJCLGXtGZFLEPAYgdIQCx96tbe3R84uFYk3T0yHHXbFy+BZwZSbKlJOfJeJrCST66t0k8iLM46bN28ORHdlGybY8lXrLu0gzO5MttTkZbOlIBa+1P\/IkSOg7lPKS9mk7fCOwLp8PxR3UL7\/CQmys7MTtmzZUugVtSaycv3JWV05qqm8fMEl2gkd7m77+A6OlamsiCk+OMGBchdx+LypTdMEDJYv4sNXCmLhnYBT0nEbFU3BzFoOv1JoLEgSaMhLly6F6dOnR0ZTJvuhssqlq59EViz75JNPwjvvvBOrjw1ZKVqJwxZtYfXq1TBz5kzr185E6WmKLSdB0xXotvBVifHRRx+Fu+66Cx566CFYt26ddTxLQyxdXV0B05chRKdO4Zsp8W9ILGPHjo08+9eVQZnKSuuU8K4oNLA4orRl+CbyErHQEMjVUMhEVsKJhvRFnRGdpH\/wo0f9bvt+MCGWiJ5JYlD0FXY1jDOVlVZI4w51kwgsieEmKWsiL31kpk2bFpA5jxzweI6iHhNZ1S0qZRsKIVZ9klhoPOrrUMhVpMKjq7ihRdiRGC7yLCbDC3JWiqqKdAxOWiayqolRV7JGkW2RMpUiYvE5eYsdyfdAFfUVVd9jmmCkeqrTFi23qbycsF1FLCayqhGLK1mFWBQEaGxatmk6Ps3Iv+xEJqNGjQq2O+AFb\/TEHYRly4njZMXhRFmIhYaOM2bMCETSYavuL3NpGybYln26uc9FLLYcTdoVBAQBNwiUYijkRnV5qyAgCNhCQIjFFrLSriDQhxEQYunDnS+qCwK2EBBisYWstCsI9GEEhFj6cOeL6oKALQSEWGwhG9Ou6YI6G3uPaEMaTpGbLo4LuyvKBXw09WtrWp\/aL+q+chcY2n6nEItthEPad0ks6Dj79+9PdMxo2YjF9tYJVzupHZlj7q8VYskd0t4N8oVV9IXFoyFoYdi8efMCB1cPJMdIoqmpCebMmQOHDx+u3IdNu6zxTGJ8oiIO3iZ9fbEtenfYF5mfjUOb6YhYBgwYELxTjRZ4Hb6QDbcZvPLKK\/D0009XLqjjiyHHjx8P2CadpayTWT0iQSU5fEf\/\/v1h7969AVaEqdq1avQXRZZCLNkcQ4glG36RtdX9I3zpv3rOCN91yjew4b016r3Y+FIio0WLFmnP16Dhzvr16wMSwA2H6PBUL+yLz52N79s6c+ZMQEhEKmp7tE+J7u4mGdVrY\/jqz0GDBgXEiVfGoFz8N7yvib+DA60jFjr8ndrE9tSraIRYLBq70rQQi0Wso07sihoKccfhxIKioiOS00Tt91Gdj+9diTpMK+zSOXXvS5T8\/Dd++BHKr9ZTzzDhhyOFRRQ6YiEi072DuliIxaKxC7EUBy6+iSdK+dBDdTB+\/CLWo7I6YsFwnz+6sz\/UbfsmGz3DbqfEd6nOzOXX3ZxJwxGVqKKIRr0ZAt+rS9DqiKWxsbFyTk4Y6QmxFGf7ErEUh3Wvr3XUrt2oiMX0dD0bEQsfPkVFGmrEEuX0aU5di4tYVPIKi1iizkyRHEs2xxBiyYZfZG31CxmWY9Gd94ENt7a2QlSOhedRdPkE3HWdNMfCnQ0jERp6oTwmxEJ1KG+iRiymORY84SzsWl8dseDf8AhRdbjIO0iXdyKc1QSxEEs2xxBiyYZfbG0e3vOhEM1+4JChubk5SFRiAhITrC0tLbB9+3Zoa2urOAr+Bz97l2aFoo5ADJthiZs65sMydVYIyQ6dkEca\/LgAHLrMnTsXdu\/eHRDjhg0bgEcslANZtmxZMMzBC8vPnz+vnRUKW6eiIxY8v3ffvn3B8RUcE11OB9+NOCNpvvTSS1oCF2KJNe3IAkIs2fCT2hkQiMrpRDUbl2PJIFKlqhBLNhSFWLLhJ7UTIqCu1wlbcxJHLDj1TRENnkCvRkUJxepVXFbeZkHvo7pCLNkxlBYEAUFAQeD\/AUidHPvE6YY8AAAAAElFTkSuQmCC","height":167,"width":278}}
%---
%[output:619496e7]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"   440"}}
%---
%[output:5f4ae634]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.030000000000000"}}
%---
%[output:13430917]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_JH","value":"   0.075000000000000"}}
%---
%[output:104f42b6]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber","value":"     1.203333333333333e-09"}}
%---
%[output:71e550b8]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rsnubber","value":"     1.662049861495845e+04"}}
%---
