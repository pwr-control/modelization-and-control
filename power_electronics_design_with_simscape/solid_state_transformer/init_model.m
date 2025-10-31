%[text] ## Settings for simulink model initialization and data analysis
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 1;
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

R0 = 0.035;
R1 = 0.035;
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
Rds_on = 1.04e-3; % Rds [Ohm]
Ron = Rds_on;
Vdon_diode = 4; % V
Vgamma = 4; % V
Rdon_diode = 1.85e-3; % V
Eon = 77e-3; % J @ Tj = 125°C
Eoff = 108e-3; % J @ Tj = 125°C
Eerr = 9.7e-3; % J @ Tj = 125°C
Voff_sw_losses = 1300; % V
Ion_sw_losses = 1000; % A

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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARcAAACoCAYAAADKOKrAAAAAAXNSR0IArs4c6QAAH9JJREFUeF7tXX+QV9V1P6Y0goo\/QGjEZbNol0jSkYhNpBumYBjETAdM6XT4YRoGKDJG3CRC+bFikU5B2Ej+QGxCLWFIp+wiKTOC0xk0jBAUbQwIyUQRfwFZgUbAn4hNjXTOC+fL2ft99737vvfd9+7dPW8mE\/nuvfed+7nnfN455\/664OzZs2dBHkFAEBAEckbgAiGXnBGV5gQBQSBCQMhFFEEQEAScICDkUgOszz33HEyZMgU2bNgAw4cPr6EFfRW17TNnzsCqVatg5syZ0KdPn1zehW0uWLAgamv58uXQq1cvwPcePnwYJk6cmMs7dI1s3LgRdu\/eXXlv0stWr14NY8eOhcbGxlSZspRNbexcAcJp69atMGDAAFi3bp2RLHH4mr4zz3KnTp2CGTNmwPz583PXUxM5hVxMUCqxzIoVKyLDX7t2rTNy6ejogGnTpsHdd9\/tlFxI2SdNmpT6HiShhx56yMigs5TNMpSvvPJKhMu4ceMiAzV9fCEXlBf15+jRo0Zkbto\/03LekgsNLAKDD3kJNHDvvfde9PvOnTurvir09ce\/Dx06tGKY9Pt9990X\/YZtP\/DAA1pF18nAvQtsH70Akgfr4Beurq4u+h2\/evjMmjUrUlBqkwxZ9VT4v9GTWLhwYVRf\/XLGKbBKRGkYYrvz5s2D2bNnw\/79+zvJiQareze+Z82aNZFMo0aNgh07dlRIgH\/tsUGOr0oCRDb0birLx4\/GfvDgwdFXWJUzrix6k1x+JAfy0FTD0Mmg\/h7XhtpXGuM4HeV6SEaPGOp0FNvCv1ObSZirsnKP2qWXnUYyXpKLaoBcKclo9+zZEyl03759I6Wrr6+PFIh\/hcePH9\/J\/UfFxHCGD5jOK1C\/stxwDx48WAmLiFxIHnLh+ReD3ouDjvJyLyGJXNBIkjwXxKW9vT0iSnwQB6wTR2JxGGIdFTMMixD\/ZcuWwcqVKyvtEr7UFyQCwpf3XYcT9YV\/RXnZJ598spOnohIRlm1oaIg+BEQcZERqWY4pkRLhwg2Cxpj+po5FmueiG2NVJ\/Cd6pi3tbV1wp68I5KBdBTr0m9xmJM90Fhu2bKlE45ZvMU0ssj6dy\/JRfeFQyVobm6uyhfw8nv37q1SUjJAIgX6Qia50zigc+fOjXXL4zwXGlzMXyQNaBbPJY1cqK0HH3wwGneeB8qCoS4sivv6Y\/4HvTHKP\/D3ENGTsXIcVKJHnOhrrH7VsS9xY6P7QscREf9o6EKDuPwWz0URLnFhUdIYk+dy5MiRWOInI6X+c882ztPAcjrMVeLiOoHjoBJoVoKwKe8luahfaw5QGrls3rw5cif5QyHFyZMnEw2Q10kjHjJk+kpxclEJhLebJ7mQEmP\/6AtHuZksGKrkQgqORrV48WJYsmRJ1D56OUgu3HA5TqToFMpSv\/ErjAlp7mEiuahhGyeZOE8LDQy9lSQiVcNRksGEwNRQM4lcksZYbQf\/zb1KIm2Oi857QvnVseTYkE6rJEAfULId8jwR96IeL8kly1cXwUryXDiQ6lfBlEDUGSFTzyXOFc+TXPgXvl+\/fpWQKO7Ln0TQKrlwZUZ8+dc8i+fCsU9KcvLcBYUDcaSly1OleS46Y8rDc4kb4yRyUT+OKvHYei5qX8VzURAxybnQV4xi6iw5F12szsVQByXua4HtxHku6tcGvy4Uc48ZM6bTV4xcY5JJVa602SL+9eeJPBMMyRtRySWur5TQ5DkX6stbb71VCZPSci7k9aiklSQDD7fIOGn8KXnLZ5aKzLlQf\/gYqyGgSiBqrgkT50SqceTCcy4q5pJzqcEP40bDZ0r4V6F3796Rm6y6vGmzRSbkgiJnmS3iYRH+t24mgbwKmomhWQEdufC+xK2rUeN7vhbGBEMMdfChmS0kET6DhNijV4QPD7lczBbxGRkuO7r4+BBmON5IaOTJqGV50hfrZZktiiNo3VR02mwR6YRKLuq4IL5qwlwda5ktOmeQqJytra1VC47UwYiLg9N4yKc1BGmydsW\/Z\/Ws4rzBshZ1hToeNpijF2q6aDFvfHLNuZDhq9OyJDSGGnPmzIGWlhajlY5xnRVyyVsFsrWnfiCwdpaVymUqe7ae+lO6Vsy71ApdVBx0V5Fc4giEr5\/Iaym7PyogkggCggBHIDfPBYlj\/fr1cOedd8KiRYtiyYWvm0AhklbHyjAJAoJA2AhUyEVdoJTULTVXgm7b0qVLYerUqdEKVJPQp2yXLexhE+kFAf8R6EQuuFgKF00lhSxICmo5dVYFu522i5TiyKampk57e6655hr\/URMJBYEAEdi+fTsMGjSoMMlzC4tMkrYYFuFD+1fiZpWQXF5\/\/fXCALB5kchqg56+ruDaNXCtCouwWzbb+9UZIU4oSTtmCU5RrK6hWDa9EB2wQc8f0u7kuajGnxbauIAgJMV64403CnUzbfAWWW3Q09cNCdeibSsxLCpjdqdoAGxULiTFElltRlrIpRb0jHMucYncWl6YVkfIJQ2h2v4u5FIbbmm1QsK1aNvSkos6NV1UiFQ0AGnKk\/T3kBRLZLUZafFcakFPu86lKDJRhRZyqWUY0+sIuaRjVEuJkHAt2rYq5ILJXPxf2cvyiwagFoWiOiEplshqM9LiudSCXi6L6Gp5sa6OkEueaJ5vS8hFcC3atnJZ\/p\/nsBUNgI3sYrA26HUNbyAkHSjatoxni9yoUXWrRQNg06+QFEtktRnprkGERduWkIuFzonBWoCXUFVwdYOrkEtAe4vECNwYgeDqBlchFyEXJ5olBusEVggJVyEXIRcnVhCSEYisTlQAhFyEXJxolhisE1jFc0mANTahy3dH01UTuqMr8x6yotnVRn4xWBv0usYMTEg6ULRtVZELvyJ0woQJ0bm49957L+AF10VcUVA0ADbmEZJiiaw2I901iLBo26oiF777GW+RI3JB0jE5BtN2CIsGwEZeMVgb9LqGwYakA0XbVmxYhLe74WXi06dPh02bNkUn+s+ePbvTXcRu1AoKTzrZ9CMkxRJZbUa6axChF+SCUPJrRPHfRV0DUjQANionBmuDXtcw2JB0oGjbkhW6FvYRkmKJrBYDnVA1FFzbnj8Od6\/dBSd++LdugIhpVcjFAupQFAu7KLJaDHQXIZe72l6CU9+\/2Q0QJuSSdjma60OkinbdbJAWg7VBT8IiN+jFt7pi2yFYse2NcskFRcODuQ8dOgTz58+vSEq\/jRw5Etra2mD58uXQq1evmvHBpDE+\/B34byGXmiFNrChE2L1x9YJcdAdx0+\/Nzc2watWq1JsZk4aSksWzZs0ScnGj81WtCrm4AToUXL0gF1pEt2fPHli3bh00NjYCXdd64403wm233QaPPfZYzZ4LkRS2e\/r0aSEXNzov5CK4dkJg3e43Yc5PDpYfFqFU6lT0hg0bYPDgwUaXzCeNK4ZDGFodPny4KvSSsMidRYTyhZXksxsdGP\/wC\/D0a+\/4QS4uuoiEtXPnzshbicvrELng\/+Ol2b4\/HR0dUFdX57uYkXwiq5thCgHX0aNHwwcj5sHHV36u65ILei1r1qzpNMpq3kUSum6MQDyX7o1rn3ueigAodSoaBVCvcaVhGTp0qNUl9Xx4kzyX119\/3Y0m5NyqGGzOgJ5rTnDNF1dcQIdrXEonF0y4zpkzB1paWmDz5s1RfmT48OGAXkdDQwNMnDgxl54LueQCo3EjYrDGUGUq6DuuR059BLPbXoryLZ\/68ES5K3T5VPSTTz5ZSbrKXdHVOue7YnGJRdZMnGFc2Hdcn3ntHRj38AtRf3oe\/C84+vj3jPtmW1B7nktTUxMMGzYM5s2bB62trbB3715ob2\/PLSzSCS45F9shja\/vuxEIEeY\/7jtePgUT1uyPGq7v0xPe+9HtUGTKIXZvkeq9LFy4MBIQp6MxRHL5CLm4QVfIpXvhSlPPRCxbvnUDjPrzz5dPLm6GwaxVIRcznLKWEnLJiphZed9w\/e6ml2H9s0crwqPHgsSC\/1+0bSWeRMcvpZeci+RczMzNvpRvBpvUIx9k5UlbLuuIay+HLXfdUPmpNHJJ2w2NEuJh3bYbFtNUr2gA0uTxXbFM5ffBCERWUwTSy+kIhYdB6K3wp2jbMvZc0rubT4miAbCRWgzWBj19XcG1GpskMiFCueXzfaF1wmAtsEXblhwWZWEfYgQW4CVU7e64IpH8x8+PwbOvvROtT9E96Jl85drLYf7YQVFOJe0pjVxMwqI8V+jqgCgagLQBkbDIBqHa6nYXckES+b\/ffwLfffRlOPL2R4D\/TnqIQB69YygM7n9RZnCLti3xXDIP0fkK3cUILCCqqWpXwpUIY9erb8PG548bkQiBhmQy4torYPXk62rCUa0k5CLXueaiSGojXclgnQBUY6OI6x9ddlVUe\/WOI3Dg2OlMBEL5Evz\/W4b0hda\/0edMahSxUs0LcuHXuZJkRcwU4buKBsBmwMRgbdDT1\/URV\/RA1uzqgF91vJ+ZPLgngv897vp+MHPEH47qMMmV5IVy0baVeJ0rP9+WLkqTqWgJi\/JSdl07RZILkgYa+GP7fgs\/PXAKDp88k5hETes7kcXIxivgnjENcEHBBJIkX+nkknaG7uLFi4EvrksDO+vfiwYgq3y8fJFGYCMn1u2OsiJxnAWAn+z9H9j58qkIwqTZFxOMiTzqr+gJt990FdT98fswcODAQj0QEznjyhRtW9rrXLdu3Vp1hi6GRupp\/bV2VFevaABs5O+OBmuDl2ndJFz5jEr7L47D06+8HTVrMtti8n4kDySOxj+5CGZ85Wq45MIeicQRkg4UbVva2SL1wCi5zrVaNUNSLN9lpfBkx8G3Yd3OV+Ht3\/XIjTB4bgOJo+HKXnD3zfVwYY9PWec9fMeVa6035GLC8i7KFA2ATR9CUqwyZOWEsWnPcfjNuXUctqEJHzMepgzq1wvGDOkL11\/d25o0TPWiDFxNZVPLFW1bss6l1pHqhnkMCknqrugZ7bz9xeF3I8LIKyTRkcZ1V10M\/3BLA5z53SeFkYapWgi56JGK3Vs0Y8YMqK+vd75J0Yekk6kSxZULSbHiZOX5ixePfQCP\/+oEHDl5xglZqKEJehnjr+8PjedWmvIp2dBxtdEpl3W98Fzi1rnE3Y7oAoiiAbDpg29GcN6zuBD+ddeb8NKxD+CNE+7IQiWML1x9Cfz1F\/vDZy690MrD8A3XJB0JSdaibcs4LMI7h3Cty9q1a2Uq+py2uVQs7lUce\/d\/Yesv34Jfdryf68xInNHwHMbAPj0Bd9reMPDSStEiFn25xNXmYxK69+oFucR5LgMGDKhMTceBzG9o1K3mVduN2whZNAA2ypbVCJAwPvr4E3jixZPwxK9POCcK7ln06wUwpO4K+MvGK+DLDZdZeRY2mJnUzYqrSZuuyoQka9G2pc254GCYein8OhK8gXDBggWAB3yr15DwcnhXdKg5F\/Iqfv\/uMTh45lLYsv+3lZkQF8lNwol7Ffjbtf0uihZy9e\/96VTPIiQjEFndUGHp5GLbLfJOJk+eXHWYN15ov2zZMli5cqU2tCoaALW\/RBw\/+Nlv4NdvfuAsuanmK67tfxF8YcAl8Pdfubqy9T7PEEQM1laz4+uHhGvRtmWcczEZGgqNdGGRycK8IgFo3ngADp2w20ui8ypuqL8UvnHTVdFCrTxJwmQcQs8NhGSwIclapG2hDuZKLqTUSCK7d+9OnMqmw6lwOwG\/rsQVAGnHBKYlN6+v6w2Tv\/QZ6N2zRyVfEZJiiay10nJyvZBwdWVbOoSckIvJzBKFT2puBgHAZ\/v27dbacPS9j+H+n56APW\/qT\/gacGkPuKp3D5jxpcth4GU9AP9t+nR0dADmmEJ4RFY3oxQCrqNHj650vtRL0WrZFa3WwSlrfNRNjujR4IOJXsy\/0G2OPLmbF7vyS6F46IJ7S1ZPHpJLqBLSV0tkdUMuIeGal22ZIlnxXEzO0E06MEo3Fc0JRZ2KjtsMaQvAj587Ct959OVO\/cecx+pJQ2DEn15uiotRuZAUS2Q1GtLMhULC1da2soLTZa4WwZzK+H95odMhx9OaroZvf7U+Fy9FkqRZVav28iEZbEiylk4utatEPjVrAUAlFn6FZT5SxbcSkmKJrG40ISRca7EtG9Q6hUVLliyB5uZmmDt3Luzfv7+qXR+vFlGJRb3C0gactLohKZbImjaatf09JFxLI5faoM2\/VhYAVGKZO+az0PK1P8w2FfGEpFgiqxuNCAnXLLaVB1pOpqJtBMsCwLc3HoB\/\/+9j0eswWbvlW+cv3baRwbRuSIolspqOarZyIeGaxbayoRBfWru3yPewCL2WL\/7zs1GvMMeyb9Ff5IFHpjZCUiyRNdPQGhcOCdfSyUWHKq5dGTlyZNV+IeNRMCxoCgASCx2jiB5LGUvsQ1IskdVQATMWCwlXU9vKCIG2uHFYpFtcl5cg1I4JAAeOn4am1p9HVTbdMRRGX9cnbzGM2gtJsURWoyHNXCgkXE1sKzMACRWMycVkSX8egqUB4EM4RP0MSbFE1jy0s7qNkHBNs628EcqUc9mwYUPpYdG\/Pf0mzNt8MMIBw6G8V91mATgkxRJZs4ysedmQcC2dXMxhdVMyDYA+9zwVvbisJC7vdUiKJbK60deQcE2zrbwRig2L1E2FuD+ovb3d+GQ6GyGTAHj61XeiJf4+eC0oQ0iKJbLaaKW+bki4lk4uupPkTM5oyWP4kgCgnc4+eC1CLnmMdnwbIRlsSLKWTi61HLmQp5olAUAhUZFL\/JP6FpJiiax5aun5tkLCtXRyQdjQS3nooYcqp\/3TcQx4YlxZF9HzoxRwwVwZ61pU9QxJsURWIRcvyAWHAfMu06ZNg6NHj0ajUvZF9L6FRBIWuTFWwdUdrt6Qi7suJrccBwBf2\/Kd0Z+Ff\/yr4jYnSlhUvCaIl+UG89LJpaiVuDr44gBoe\/443NX2kjezRCS7GIEbIxBc3eBaOrlgt3AfUUNDQ9WlZm663LnVOAB8DInEfXenDUIubrAtnVySztIt47AoHhL5Mksknosb5Rdc3eJaOrm47V566yoAfOHcw5OHRHcH+fLIF9bNSAiubnAVcrnmGuB3q6zYdghWbHvDu3yLhEVuDEBwdYdraeRCiVyXZ+iqV4vEbYRUAfA13yJG4M4IxHNxg21p5OKmO51bxcV5hw4dihbi6Y5wUAHwbVUu75EYgRutEVzd4OoFuRSxcRHJpa2treo+aQ4AT+b6lm8Rz8WNAQiu7nAtnVxcb1zkoVFaWPSzV96Gr\/9gX4S2L0v+xXNxp\/wyW+QW29LJpaiNizTljSES7lmiBwHABy+iv2Pz8egSebwcfutU\/y58D+EScsJVZHVjuCHg6s1F9DgERWxcJA+mqamp02I9zq4+51vEfXdjrIKrO1xL91yoay42LvJL6dW8DvdcaCqayGX+2EEwf2yDO9RrbFkSjzUCl1JNcHWDqzfk4qJ7WaaifTt1Lg4PMQIXWiIn\/LlBFaBLk4sJaAQAXzznYzJX3HeT0aytjJB2bbil1RJyObdC1+fFczKrkabGdn8XcrHDT1dbyOUcufAbFcu4qtVkeMUITFDKXkZwzY6ZSQ0hl3Pk4vtMkYRFJupcWxkhl9pwS6vlBbng6tkpU6ZUyVrUkQs\/fmJv5QoRX2eKhFzSVLn2vwu51I5dUs3SyUW3uM1Nd6tbRQA4ufiazBVycacRQi5usPWCXJYsWQKLFy+GPn2Kv+AdAfintmegeeOBCOGyr2xNGmYxAjdGILi6wbV0csFu8d3LbrqpbxUB+LM5\/wlPv\/aOF1e2CrkUrQGyzsUV4qWTiw\/HXAq55K9e4g3kj2looXHp5OJmCMxbRQDe+fraqIJvZ+aqvRCDNR\/XLCUF1yxomZcVcmHk4vNMUWhfLTFYcyPMUjIkXL0gF74HaNy4cTBv3jxYtGgRtLS0QGNjYxbsM5et\/\/LX4IMR86J6Qi6Z4dNWCMkIRNb8xp23VDq5ELEMGDAAJkyYAOvXr4d7770XtmzZArt37646OS5vGOpG\/R18OGx61KzP09DiueQ98ufbE3Jxg23p5MIPizp58mSFXJB0ipiirhv1Tfhw2DQhl5z1Sww2Z0DPNRcSrqWTC2KGNy7iBfTTp0+HTZs2wZ133gmzZ8+OTozDk+NcPv2\/+UP4+MrPRa849f2bXb7Kuu2QFEtktR7u2AZCwtULckEU1S0ADzzwQCHXuxK51PfpGYVFPj8hKZbI6kaTQsLVG3JxMxTprYawYZF6EZJiiazpuldLiZBwFXK556lojL9x01WwauJ1tYx3YXVCUiyR1Y1ahISrF+SiHkeJw4JT0suXL4devXq5GaVzrfp+bi7vfEiKJbK6UduQcC2dXPhUNCVv6TccHtcEQ+Ti84ZFCYvcGKrg6hbX0smlqHuLdDAKubhRsJC+sCKrGx0onVxopki9ahWnpxsaGrQzRriTeuHChREqukOl1HArrhyRi+8L6LCfYgRujEBwdYNr6eSStCuauqySAt5BtGzZMli5cmV0Bgytk1FDKGx7zpw5idsIiFx8X+Mi5OLGAARXd7iWTi55dE13ybxKQnHvQnIJYY2LGEEemhLfhngubrDtEuSiC6F46ITwxS3ME3Jxo1hisIKrF+RiMxVteoqd7qxeJBffz3GRWQ03hiq4usW1dHKJm4rGLuvyKByOtKQvL6u7iB7J5dNHnoFnv3e7W6RzaL2jowPq6upyaMl9EyKrG4xDwHX06NGVztM97G7Q6NzqBWfPnj3Lf6p1KhqJZeTIkdHmRt1jchE9kovv57hQ\/4r+EtgohMhqg56+ruCqx6aKXMhL2bp1K6xbty46HAoTsdOmTYtW6cbtio6754hW9OI5MPhMnDgR1HBLl3O5aO+PIu9FHkFAEMgXgVI9F+qKSfI1325La4KAINCVEIj1XLpSB6UvgoAgUA4CQi7l4C5vFQS6PAJCLl1+iKWDgkA5CHhDLjzHs2HDhsRZpyKh4slq3Wl8vAwebE6J8CLlxHeZyEoylX0neBZ5cSZyzZo1kehl6YYJtnzrTJl6oNM7k+03eeqsF+TCtwUcPHgQ1E2TeXY4S1t8MLAe3z\/FjZTvl0KSbG9vh7Vr1xZ617aJrLzvZLBlGaupvHxRJuoJ3Ubh+lwhjpWprIgpPjijinIXcVuGqT7TjC+WL+rj5wW58IHA6eq0zY2mgNqWw68VKgwSBSrzggULYPLkyYlelcn+KVu54upnkRXLPv744\/D++++n9seFrOS1pGGLurB06VKYOnWq8\/uykvppii0nQtOV6q7wVcnxkUcegVtvvRXuv\/9+aG1tLQRPb8jl0KFDEeP74K7TwPANmPgbkktTU1PiQeVlKZWprLRIEi+6QyVLI0tXym8iL5ELhUNlhUUmshJOFN4XdaB9lvHBDx+Nu+vLDVEuIZeE0cmiVPQ1LiukM5WVVlLjsRkmnlgW5c1S1kRe+tBMmjQpInTuQeDRHkU9JrKq21l8C4sQq25LLhSfhhoWleWxcC8rLczQndVTRt7FJNQggyXvqmjjyIotD+fLkjWJcIuWyQvPJeSELg4m3zNV1NdUfY9p0pHqqYZbtNym8nLSLstzMZFV9VzKklXIJQYBilV9m8LjU5D8C0+EMmzYsGjfFd5QSY\/umE\/XBpwmK4YWvpALhZFTpkyJRIrDVt2PVqZumGDr+1R0t\/RcXBudtC8ICALFI+BFWFR8t+WNgoAg4BoBIRfXCEv7gkA3RUDIpZsOvHRbEHCNgJCLa4SlfUGgmyIg5NJNB166LQi4RkDIxTXCmvZNF9252KtEm9hw+tx0AZ3uLqoy4KNpYVdT\/tQ+HdVa5CbJMvB09U4hF1fIprRbJrmg8ezcuTP2PGSd2L6Ri+ttFmXtwC5JHZ28VsjFCaznG+WLr+hLi8dK0OKxWbNmRUauHnKOHsXgwYNhxowZsH\/\/\/sr927Q7Gw9QxyfJ8+Bt0lcY26J3677M\/Gwd2oBH5NK7d+\/onarXwOvwxW64JeHFF1+EXbt2VS7B4wsmR40aBdgmHfweJ7PqOahEh++4+OKLYfv27RFWhKk6tKoXmESYQi72hiHkYo+htgX1cB6+TUA9p4TvVuWb3vBeHPUebnwhEdLcuXNjz+eg0OfBBx+MiAA3KaLRUz3dl58bHN\/ndfLkyYiUiFjU9mhfE90VTjKq913xVaJ9+\/aNyBOvo0G5+N\/wPij+Dg5yHLnQbRXUJranXnMj5OJQ2WOaFnJxiHfSyV9JYRE3Hk4uKCoaIxlO0v4g1QD5XpekA7l0F9upe2WS5Od\/4wcoofxqPfUMFH7Aks6ziCMXIrO4d9AQC7k4VHYhl2LBxbfx5CkPQ1Qj40c5Yj0qG0cu6PrzJ+7sEHXLv8nmUN0tmPgu1aC5\/HHX\/1JoopJVEtmo19nge+OStnHk0tDQUDlnR0d8Qi7F6r94LgXirTupTPUKkjwX01P6XHguPJRK8jhUzyXJ8Gs5vS3Nc1EJTOe5JJ25IjkXe8MQcrHHUNuC+qXU5VzizjbBRpcvXw5JOReeV4nLL+Bu7aw5F25w6JFQGIbymJAL1aE8iuq5mOZc8KQ03f3kceSCv+FxpGroyAcnLg9FOKtJYyEXe8MQcrHHMLEF7urzsIhmRTB8aG5ujpKXmJTEpGtLSwts2rQJVq5cWTEW\/A9+li\/NFiUdp6ibeUmbVuYhmjpbhISHhsg9Dn7UAIYxM2fOhG3btkXkuGrVKuCeC+VEFi5cGIU8eEn66dOnY2eLdOtY4sgFzwPesWNHdPQFxyQux4PvRpyROPft2xdL4kIu9oYh5GKPobRQIwJJOZ6kJtNyLjWK06makIs9ikIu9hhKCxkQUNfz6NakpJELTouTZ4Mn26veUQaRqoqSjLJC1wZFTw7otuuC1BYEBAEfEfh\/hK0q+MuUaHQAAAAASUVORK5CYII=","height":168,"width":279}}
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
