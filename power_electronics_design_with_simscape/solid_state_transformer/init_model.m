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
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"275"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"1875"}}
%---
%[output:31c2a36b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"3.5511e-05"}}
%---
%[output:538b2e49]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"1.7833e-04"}}
%---
%[output:23e72b9a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"563.3826"}}
%---
%[output:2e20a02c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"381.8377"}}
%---
%[output:0cc65d7c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.2831"],["67.6682"]]}}
%---
%[output:8c1b6188]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.0001"],["-9.8696","-0.0016"]]}}
%---
%[output:995db2c2]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.0156"],["2.7166"]]}}
%---
%[output:13ed32a5]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.0000","0.0002"],["-19.7392","0.9969"]]}}
%---
%[output:7b651ad6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.3110"],["54.3322"]]}}
%---
%[output:38b72f5b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.0734"],["3.8024"]]}}
%---
%[output:9482b399]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAcEAAAEPCAYAAAA6WX8sAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQuUVtWV5jZtGioxRgoJReGDaKqM05MUQhsQkxg742St7hR2mwcUMwNdqWRqujWyRh5FaWsvo4EqHmaIjzWVpKyB7qEg42hi9cwaQxzbqChRlOpMJzalEQgUkAqPqLFkxg6z9iWnOHX++9jn\/++9\/z33fnctlsK\/77nnfuc7+7t7n9dZp06dOkW4gAAQAAJAAAgUEIGzIIIFbHW8MhAAAkAACHgIQARBBCAABIAAECgsAhDBwjY9XhwIAAEgAAQgguAAEAACQAAIFBYBiGBhmz7dFz927Bi1tbV5D+3t7aXa2trEKjA0NEStra00e\/Zs6urqopqamsSepRec5jtKXkjh8NWvfpUWLFgguSXTNt3d3dTT0+PVsb29nTo6Oqzqu23bNurs7KQ1a9ZkEg9+v+eeey7x\/mEFWgGMIYIFaOQsvGKaAhEkguxkrrnmGpo7d24ikAS9Y9LPDXqZcpwqO+Enn3zSSmDKuce2AfgZixYtGrstjyKYt48W2zaulj1EsFrI47mpITA6OkqrVq2igYEB2rJlS2oiyBFoGs\/1A1I51ObmZrGgqUjJRmDKuaechlciaFM38zlZjwQVT\/fv349osBySlHkPRLBM4Kp5m54W4nro6R09Cpo5cybdddddXlXZGeqpQRW1DA4Olvyul3H99dfTl7\/8Zc+mvr6e+vr6qKGhwff1dbEx7c0oiX9X6dFPf\/rTdM8991BTU5PX+XXxiCqH06rqubt27fLqx5dKh95xxx30ta99zRNAdZlY8L8rTHWRVI5Xt1eOVJWlO2X9He+77z5au3at73MPHDjg1W94eHisTnob6jgy5vfffz89+OCDpN6P8TexVtipNLOfw1ftqj9Xva\/5XqqtL7jggjEhN\/F79NFHvfSiunR+mOVFfXyYfAwrK4yHYRGjjgnXWdXdFFazf+nYqjJuueUWevzxx4n7j2o7\/Z3539Qz9LaNwsWPh9X0NUV4NkTQoVY2HZ9eddWR\/Ryd6by4HBYgJYDm735OOkxA+LeguimHNXny5HFjgkoE9Tqw2PiJli6EZjlxiaBfpGE6JNM5BuHK\/x4kgitXrqSbbrqpBPsw0eHfpkyZQiMjI57I+wkTP1N31mbdg3ihnvviiy\/6CtrDDz88Ng6n80138qYImmWp34OEMIyzfM++ffsCxVavkymA5oeKEqBPfOIT9NRTT43r+X5C5te\/TBFjG7868r+r50SVreOS9WjVIXcpripEUAxV9Q1VJ9e\/hJUD4drpURB\/7avOpTsZvcPqX8C602ShUZGKKkM924w4FCp+v+sd+rrrrgsUQf1L2bacKBHk6JevqLRkWKTK0enRo0dLMNGjF8apsbFx3DtK0qFmqlZhr9qToz6z3bkuPD7mF6EylvPnz\/feV48cJZOFJKlN08b8u8JECTbXP+rZint+76P+jT+W+J2D0qE6jopPZptu377dE1O\/yC6oXDMboKJfvYywZ6tIUfE\/Cpc40r7V91Ru1QAi6FB7BX0lKifCnX\/WrFm+MyN1m7179\/p+3TMUehkcfaiZnFETW6K+YINERncK\/HzbcuISQf3ZLGh86U43yDnpIvCVr3xFLIJ+kbPfc\/UoW4laUKTFtuzMVT10bP2eZ6aFw0QwKA1s3hMW1fl9QJmpbJVqN8VUCX+QWEXxU29fvYygdjWjSoWVEsGgNLg+81nnsuqXeipauR4dF78UvEMuysmqQgQdara0RVBfYhDlZGzFi2H3WzIhLUd38KbD5LL1JRKSSJBt9Mkk\/Heejm9GwqYTthVBE0czWjTFNy4RVDQPEl+eMesngmZaNSoSdEEE\/TIPql1N\/gVFgnoZQX0DIuiGc4UIutFOXi3LTYeaaTs1xhL0Ve2XvooSwbA0ph6d8Hvw13KQCErL4TSTKVAqTWyKIAuNZMKBKRB6pGSmlFk0otKhHKVGiYhZRrnpUJ3GQdGVSXVT0MyoSL2znhFQ76O4Y97jlw6N6mJJp0PVB5OKoINE0C+CVhiZkWDQRCYzFRuWDvXDBenQKLbE\/ztEMH5MEysx6YkxYSISJYJhdfMbLwsSwahyOHWkxvdMoCUiyPf4zQ5VZZkz\/PRF5jYTY1RaTL+Hn3vDDTd4UarfxTj5vZ90YgyXqT4MTPENmjSi36Pb8DM3btxId999d8kkHr7HFEH+t6BJNupdoz66\/FKFUZG4jmPQO4YJmC46N998cyC3wsrgOvhNmJFOjNFxicqEJOZcClwwRNDBxpcukdCXN0QtkfCbbGOTDmUYw1JtURNP9B1kwsrh55jT6W+\/\/XbavXv32EQQv0hQd5BBk3v0ss2xSj+R1MVAv5f\/X4mg33O\/\/e1vj9v5hBfw6+OP5SyR0MVMd8p+y2eClmaYuOpjlFwm47Z+\/Xpavny5B4ce0atZvkFLLqLW94UtkeBnSSOkoLE8zgb4CUxQ9MsY+S1P8Ysmgz6g+N\/NHWqCxlZVGZKMhYMuK9NVzqQIchqGp5LzOiu\/NWl+TjKqk2W6FWKsXNRMvBgfhaISQMAv7WrOAA5ap6lXp5zF8gm8TiGK1P2R8kN+M0ajwMBi+SiEkvk9cyIoGc\/gr8Zly5bRrbfeGrhwOxm4sl8qRDD7bRRWw7B0cFga16\/McrZNcxu96tXeLx3KtYnaYMLvwyUve71WrzXsnpw5EeT0BHdevoIiQXb0q1evpg0bNiS6EbMdlNmwhghmox0qqYWZGuSybAWQ78FelJW0gv295jCFjQDy0\/DRYo95HHdkSgS58995553U0tLiESJIBFko+\/v7rU4I4F1Q+A8uIAAEgAAQSB4B3tmI\/2T9ypQIckqBL15YGjYmaKYewiY6cHksfitWrKCdO3dmvT1QPyAABIBALhCYM2cOrVu3LvNCmBkR5NTNpk2b6LbbbvNEK0wEOUrk2VtqQ2jz7yaD1AwwbpDp06fngmBxvAR\/FPA0eOAyHk3gUsouYOLf44BLOC5RG6fH4ccqLSMzIqifuRY1O9R86Sh7JYIuNEilDWpzP3DxRwu4lOICTMCVvPqWTIig30QABbhEuKImyqADowPntQPbvFcltuhD6EM2\/HGJL5kQQZvITk0hnzdvHvFuHurvPBOro6PDt51cahAbolVqyxtpcwp6yZIlNGPGjEqLy839wKW0KYGJP72Bi\/sfB06IoBI6njXKezaG7fHo1yQQQX+ivv3223To0CGaNm0aTZw4MTciVumLAJdSBIEJ+pBNv3LJ52ZSBG3Alti61CCS94nLBo4Njk3KJXAFXJFyhe1c8rkQQZuWzZktHBscm5TS4Aq4IuUKRNAGqZRsXfoqSQkS7zFwbHBsUr6BK+CKlCsQQRukUrKFCKID21ANDr8ULWCCPmTTh1zyuUiH2rRszmzh2ODYpJQGV8AVKVcQCdoglZKtS18lKUGCdGgI0HD4iASl\/RBc8UfKJZ+LSFDK9hzaoQPj615Ka3AFXJFyBZGgDVIp2br0VZISJIgEEQlaUQ0iCBG0IYxLPheRoE3L5swWjg2OTUppcAVckXIFkaANUinZuvRVkhIkiAQRCVpRDSIIEbQhjEs+F5GgTcvmzBaODY5NSmlwBVyRcgWRoA1SKdm69FWSEiSIBBEJWlENIggRtCGMSz4XkaBNy+bMFo4Njk1KaXAFXJFyBZGgDVIp2br0VZISJIgEEQlaUQ0iCBG0IYxLPheRoE3L5swWjg2OTUppcAVckXIFkaANUinZuvRVkhIkiAQRCVpRDSIIEbQhjEs+F5GgTcvmzBaODY5NSmlwBVyRcgWRoA1SKdm69FWSEiSIBBEJWlENIggRtCGMSz4XkaBNy+bMFo4Njk1KaXAFXJFyBZGgDVIp2br0VZISJIgEEQlaUQ0iCBG0IYxLPheRoE3L5swWjg2OTUppcAVckXIFkaANUinZuvRVkhIkiAQRCVpRDSIIEbQhjEs+F5GgTcvmzBaODY5NSmlwBVyRcgWRoA1SKdm69FWSEiSIBBEJWlENIggRtCGMSz4XkaBNy+bMFo4Njk1KaXAFXJFyBZGgDVIp2br0VZISJIgEEQlaUQ0iCBG0IYxLPheRoE3L5swWjg2OTUppcAVckXKlUJHgsWPHqK2tjQYHB23woaamJnrkkUes7qnE2KWvkkre0\/ZeODY4NilnwBVwRcqVQopgR0cHzZ07V4QRC1J3dzdEUIRWskZwbHBsUoaBK+CKlCsQwQikIII2VErWFo4Njk3KMHAFXJFypVAiODo6SvyntrbWBp\/UbZEORQe2IR0cfilawAR9yKYPueRzK5oYo48Jtre3E6dFs3i51CBp4gfHBscm5Ru4Aq5IuVKoSFCBwmN8PT093l\/r6+upr6+PGhoabDBL1BYiiA5sQzA4fESCUr6AK\/5IueRzK4oEzdc3Z4tmJTp0qUGknS8OO3RgfBxIeQSugCtSrrBd8\/0v0T9u+Bxt2bJFPGnSpvw4bWMVQb1iQ0ND1NraSsPDw94\/VxMMiCA6sE2ngcNHJCjlC7jij1TtLU\/Qed9rq6rfl7ZhYiKoV0DNCO3t7a3KJBqIIERQ2iHYDo4NIijlC7gCEQzkCiJBaTeqnh06MD4OpOwDV8AVKVf2H3ubZt79bDEjQV4usWrVKhoYGPDwam5upq6uLqqpqZHil4gdIkF0YBtiweEjEpTyBVwpRUqJ4Lk\/6KCt3\/lmMcYE9dmh1R7\/8yMvRBAiKHVqSIeCK+CKDQLBIvieFx+kh7puzLcI6rNBsxL1QQTlBMZXLBy+lC3gCrgi5Uqh0qGvvPIK3XzzzXTHHXeI1R7bpkmplLwdHBscm5Rl4Aq4IuXK06+coPkPvFSMMUEVCVZzA22egLNy5Upau3Zt4AJ9pEPRgaUdGOlQcAVcsUGg1LaQIlito5TURJxdu3aF7lIDEYRjs+nWiHpK0QIm6EPSPlQoEZSCkpSdSq1y+YgE7VGGY4Njk7IGXAFXpFzpf\/4w3dj\/s2KkQ6WgJGHHqdg777yTWlpavPMJIYL2KMOxwbFJWQOugCtSrnQ\/9hp1P7aXCrVEQgpOnHbbtm3zips1a5Z4THDp0qU0Z84cqqur8\/4U\/WLHdvjwYZo6dWrV13JmqS2Ai386FFwBLmH9lPnBf7qefoN+NHw2IsEknRpPhtm0aRPddtttdODAAbEIqjotWbKEFi9enGQVnSj75MmTNDIyQlOmTKEJEyY4Uec0KglcSlEGJv7MAy5ncNm8ebPnl9\/8+Ep65\/zLIIJJOitOf15zzTXesgyb2aHr1q2j6dOnIxL8XePwxKIjR454eEycODHJJnOqbOBS2lzAxJ\/CwOUMLhwFPv+zvdS+\/ZT3j+c8vZa++82\/Fi+fq5aTSGUD7ThfzjyuSS876KQKzA71bwGM8wAXad8EV8AVCVfUQnm2LewpEjxW19nZ6eHForRv3z7asWNHYnuI2kSC1TzOSUKgtG3g2ODYpJwDV8AVCVfUpJjCiiCnKfn8QF68ftNNNxEvom9qavI21eYT5\/nvcV8QwfIRhWODY5OyB1wBV6K4wlEgL4145tUT9K63fuXNDnUh8IgtHarvHtPY2EhtbW2e6PG4Hc4TjKJPdX6HY4NjkzIPXAFXoriiFsmz3bxzDtNP\/\/Y2iCBEMIo21f0djg2OTcpAcAVcCeOKHgVeVDuRbpt9kjpu\/FKxRJAB4vFAHv\/T06EqKly4cCEtWLBA2uditcPEGHRgG0LB4ZeiBUzQh8L60L5jo3TF3c95Jl+aN50+f8ExWrRoUfFEkAFQgqMDtmbNmqoJoF4nF\/LTNs66Uls4Njg2KYfAFXAliCtqmzT+naPAR\/\/yChres7u4IijtVGnaIRJEB7bhGxw+IkEpX4rOFV0AGTMWwI9\/6LyxYMiFwCO2iTFS0lTDDiIIEbThXdEdmx9WwAR9yERAnwjDvx2759oxE5d8bmwiGLaIXQevvb09kaUSYU7OpQaxcdaV2sKxwbFJOQSugCsKAZ4E0\/\/8IW+TbHWpCFD93SWfG5sI8svzxJitW7dSb28v1dbWengoceSJMfPnz090zWBQh3apQaROKQ47ODY4NimPwBVwhcWPr5l3PzsGBo8B3rfwci8Fql8u+dzYRDDslHl9neCePXu8o48eeeQRaf+r2M6lBqn4ZS0KgGODY5PSBVwpNlc49XnT1p+REkJGQ02C4f+al0s+FyIo9QI5tINjK7Zjs6E0uFI8rrDg8R8\/8fOL\/gofCUrSobxOUK0l3Lhxo00frMjWpa+Sil7U8mY4tuI5NkuKjJmDK8XhCkd9PObHMz\/1iyO+7hsa6fK693pRYNjlks+NLRJUgPitE1TTZFkA7733Xurr66OGhoZy+6P1fS41iPXLVXADHFtxHFsFNPFuBVfyzRWO+O57Yj9955mDJS\/KgvefvvhhuuT8mkjxM3UASyQq7Xkx3Q8RzHcHjokmiHpCgIQI5q8P6Vud+b1d0KQXSX9zyefGHglKAErbxqUGSRMbOLb8Obak+AOuuM8VFr1v\/u\/99MOXj46b4GKmPHm5Q1S6M4pnLvncWEWQjzVqbW31jlMyLz5SSV86EQVinL+71CBxvndUWXBs7ju2qDaO63dwxR2usNixiPF\/1\/1gLz31yvFQ0eM3Y+Hjq1LxK3Q6dHR01FsDOG\/evLH1gC0tLWQeqxRXp7QpByLoTge2adekbOHwS5EFJtnuQyx4u\/a\/Tg8+c9A7zy\/sYqH7wqyp9O\/m1scmeubzXPK5sUWC5jpBXgs4Y8YMb+NsBqS\/vz+x0+WjnKFLDRL1LnH+DseWbccWZ1tXWha4kh2u8OzNf\/7tKVq\/fW+k4Kno7vqmD9B1l0\/2RC+uaC+MUy753MREkGeC7t2719siDYfqVuqCkrkfji07ji2ZFo6vVHClOlzhCK\/3mYP04v7XxYJ34aSJtPSPLqKJ7\/69kp1c4mNEeEmFFEGGhKM\/vkzh2759u3fOYFdXF9XU1KTVDmPPcalB0gQHjq06ji3NNo7rWeBKclxRu7Dc+8R+evnwb0RipyI8Frw\/m\/kB+leXT\/YqmEaUJ+GUSz43tkiQgdHHBTkNyqLY09ND9fX1qa8N1BvKpQaRECwuGzi25BxbXG2UlXLAlcq5wmL36shb9NCLR7yJKlFjd\/oTWdxY8DYu+DANnziZWlqzXP655HNjFcFyAUv6PpcaJGks9PLh2Cp3bGm2VzWfBa7IuKK2Gtv6wmHad3TUSuj06O7fzplG0887PX6XlejOhn8u+dzYRFC6gbY6XcIG0EptXWqQSt\/V5n44Npljs8E0r7bgypmW5Ykpdef+Pn3j8X3WEZ0qRUV2N37qQjpnwtnOil0Q313yuRDBvHotwXvBsUEEBTTxTIrEFRY5vh5\/+Si9sO91+sXx0xtJ215K6D7ZMImuuuT0UUPmkUO2ZbpiXygR5FmgnZ2dkW1TjcN0VaVcapBIIGM0KJJjs4ENuJSilSdMVMry7\/cco52v\/bpskWOU6s89m6a972y67g8+QPM+lN4SBBs+V8PWJZ+bSiRYjUbQn+lSg6SJVZ4cW5y4ARe3RZBFbuiXb9EjL\/2S9h0brUjk1HjcNQ2TiCO6qedOGBfNgSv+Pc8lnxubCMbphOIuy6UGifvdw8pDB0Y6VMq3rHBFpSW\/9dQBGjzwhld9m1mW5vsqkeOZly1X1tFFtaeXcEnTllnBRdqOadm55HMhgmmxIoPPQQeGCEppmQZXlMD9t11HiFOVlQoc369E7upLz\/OE7epLJ3nje1KRi8InDVyi6pDF3wsjgmpG6ODgYGQ7YAPtSIhSN0AHhghKSVcuV9T4G4vRr0ffIV468A8H3qgoRanqrEdxH73gfdT+iQs8gVP\/nsbSgnJxkeLuql1hRNCVBnKpQdLEFB0YIijlWxBXlMg9OXSMnvv5r73iyp1NqddFF7jL6t5LX732IjpLM0hD4CTYoA\/5o+SSz0U6VML0nNqgA0MEg6itUpNPDh2n775w2DN7beRNGn79nYp7gy5wf3jxudQ6b7pXpjr+JysCJ3lR9CGIYAkCfksm1qxZ450mUa3Lpa+SNDFCBy6OCOppSX7r7T87St\/b\/cvYIjclYvxfnmTCQvYnH5lC504823tGXGNwafYPybPQhyCC4xBgAdy6deu4w3PVuOHChQurJoQQweI4e4njirJxzbGpqG33gTfosX\/81djC7kpmTZoYqfVwjfXn0ZwPvp8+1Vib+vhbVLtV43fXuJIWRi753NjSodg2LS16xfccdOBsfhyYu5N8f\/CX9IOfHo01ajMjt4trJ9K\/qD+HPvuRKeNSk8oOXMkmV+LzBvGWBBGcO3ccojhPMF6CxVUaHFu6jk2lJPmpPz30prfOjf8tjokk+pvoY278\/5\/96BT6g2nnVLQ0AFxJlytx9fFqlVNIEWSwkQ6tFuXKey4cW+WOTY\/anv35CfrR0PFEhE1FZDzexhfvXrLgD+u8\/1fLApKcUAKuVM6V8nqpm3cVVgSVEJp7iWJiTDaJDMfm3y57hk\/QP\/z8EH3kkjp6aPdxeu61+Kb+m0\/Uo7aPzXg\/feyD76f3\/v7vZe5UAXAFImjjxQotgjZApWXrUoOkhQk\/p0iOTUVsLx\/5Df1oz\/GxLbfiTkWqiI3\/y1HbJefXeEsAat\/7bqcnkhSJKzZ9ELj4o+WSz419Ykw1Z4EGkdelBrHpgJXautyB9fG1Xftepz2\/\/E1iaUhT2Hh25JUzSiM2fbeSStsma\/e7zJUksQQuEMFxCJhrBLds2UJzjUkySRISImiHblY6sLmG7cjrJ2nbC0fon478xnuhpKM1TklyxPaF2XWekL799ijV\/PObNPuyC2nixNNjcEW\/ssKVrLUDcIEIBnKyu7ubenp6vN\/r6+upr6+PGhoaqsJhRIL+sCfdgVUKkmdBPv7yMXp15K3ERM2M1i6eXEOLrqyj3546vROJGnuTRGtJ41KVTlDhQ4FJdfpQhc1Wtdtd8rmxpUPD0GZBZFB6e3uptrY29YZxqUHSBMfGsenR2ju\/PUXbf3qU\/u4nI6mJGovY5dPOofkfPb2OLcmNkm1wSbO9qvksYAIRtOGfSz43MRHUI8FqniDBDedSg9gQrVJbdmy7\/ukXNGXKFPrxL0aJp\/ir6C3O3UZUPfWZkPxvvDEyL87mVGScx9vEgcuhQ4do2rRpSIf+DkyIIETQpl+55HNjFcFKUqCjo6O0atUqGhgY8LBub2+njo4OX9xN2yh7lxrEhmhBtvratf\/xf0bof\/7kV4lFbLqwNU59D3380kk0++JzS6qW5Bq2ODDTy4DDL0UUmEAEbfqZSz43NhEM2zZNAh4LKF8sfFH7jfLvy5Yto1tvvVU0zuhSg0iwUqnJ4RMn6W92Dnu3xBW5mbuNXHXJeTRjcs3YDv953QgZIhjOPIggRFDim5SNSz43NhG0AUhiq4uiaT80NESrV6+mDRs2iMYYXWoQ812ffuUE9T9\/yEsXlit0urCxoM288H103eWTvXWC7z55nP7fhEle2s+laE3CoXJt4PARCUq5A674I+WSz82kCEZFlQxwf38\/dXV1UU1NTSRfVYMsXbqU5syZQ3V1dd6fLF18Tts777xDG5885ImduYlyUF15d3++pr3vbPpU4yRvh3++Pv6hSZGvxx348OHDNHXqVBGOkQXmxAC4+IsguAJcwro484P\/8HXw4EFasWIFZWWZXFi9MyeCalyxubk5UOTM9YhRE2+UCCoglixZQosXL66qy2bRYwH7zvMn6Ps\/jT6sVB1l829mnkvnTHgXzZ5e+fq1kydP0sjIiDcxZsKECVXFI0sPBy6lrQFM\/BkKXM7gsnnzZtq0adM4oCCCFXg2FsPh4WFfITR\/C7PlKigRXLduHU2fPr3qkeB\/\/fEhWvG9vYHocFryj\/\/l+fTnH5tCZ599tieWSVw8wejIkSMeHlgUfgZh4FLKNmDi3wOByxlc9Ehw586dtHHjxmJFgnGfJ8jjfitXrqS1a9dGTn6Jss1CfprH9rofe813XI9Fb8lV9fS5K6amOi6H8Qx\/xwZc\/NN+WDYCXKQf41nwudK6xpYOjVsEbc4gjJooU80G4bG9G\/t\/ViJ+LHwdn\/kgXX3peakKn04MOHuIoNRRgCvgipQrevatEOlQc3wuCKiwdX98jz4bVK0D5O3WzLWC6rd58+bRggULKMxW1aUaIsjit\/WFw9T1v14bBwmLHotfFpYawLHBsUkdG7gCrki5UjgRVMBEzeiMAtBcAK9PjFG\/tbS0eBtyh9n6PSdtEWQBnP\/AS+NmeLZcWeeJX5aWIcCxwbFF9Uv1O7gCrki5UlgRtAEobdu0RJDFjzeJ\/lzP4Ngrsug9+pdXZEr84NjCGQiHj7EvqY8CV\/yRSsvnStspzC62McE4KpNUGWk0CAsgL2rvfuzMrM+H2pvojy5Lf8NwKY7owPi6B1ekCIArNkil4XNt6pOYCOop0MbGRmpra6PBwTNRkP7gqLV8cb1QNdKh5uQXjv7uW3h5Jsb9wnCFCMKxSfsduAKuSLmCdKgNUinZJvlV4ieAWU1\/mnDDscGxSbsguAKuSLkCEbRBKiXbJEWQ1\/6pFGiWx\/\/8oIZjg2OTdkFwBVyRcqWwIqhSo0VKh\/Y\/f9hbA8iXawLIdYZjg2OTOjZwBVyRcqWwIhgEkO2xRzZAS22TiAQ5DTrz7medFUCIYDB74PBLsQEmEEGpv4UI+iBle+qDDdgS27hF0FwHeH\/L5cTrAF274Njg2KScBVfAFSlXIIIBIsg7wvT29orO\/7MBW2Ibtwhe\/8BueuqV496j\/3t7E12b4WUQYfjAscGxSfoPsgbIGkh5ouzi9rm2z7exT2WdYNQpDzYVLsc2zgYxxwF3\/9VV5VQpE\/dABCGCUiKCK+CKlCuFjQTDJsbwHqB9fX2Rp0HYgGxjG5cI6mlQV9YCIhK0YcppWzj8UsyACUTQpifF5XNtnlmubayRYNDm1mqz63IrWel9cTUIH4fEe4Kz1uf9AAAXM0lEQVTylfXdYCSYwbHBsUl4gg+DYJTQh\/yxicvnSvlZiV2sIuiX9lQR4sKFC71TH6pxxdEg5mxQl9Ogqg3QgSGC0v4IroArUq4UPh3KRx\/xSQ\/6lYfZoX898Crd+8R+77V4R5gsHIVkQ0o\/Wzg2ODYph8AVcEXKFYhggAi6PDs0j1EgUlxIcdk4NYggRNCGL3Fk32yeV4ltbOlQczxQrxQfvLtjxw7q6uqimpqaSupb1r2VNog+IzQvUSBEECJo05kgghBBG75U6nNtnlWpbWwiqELg5cuXj5sJOjQ0RK2trbR+\/fqSNGmllZfeX2mD1N7yhPconhGah7FAhRscGxybtA+BK+CKlCuFTYcqgJTg6IBt2bKlagJYaYPoqVBXd4YJIi8cGxyb1LGBK+CKlCuV+lyb58RhG2skGEeFkiijkkiw+f6X6JlXT+QuCkQ6FOlQm74GEYQI2vClEp9r85w4bGMTQTUm2NLSUtWozw+UchtEXxe45Kp6+sYXLosD88yUAccGxyYlI7gCrki5UthIUD9l3lwiYQNeErbliuCyh\/ZQ346DXpV4LJDHBPN0wbHBsUn5DK6AK1KuFFYE+cWrPQs0qJHKEcG8LovQMYJjg2OTOjZwBVyRcqWwIpi3Q3X1VGjHZ2ZQx2c+aMMBJ2zh2ODYpEQFV8AVKVcKK4I2AKVtW04kqCbEcF3ztDYQkWA0++DwSzECJhDB6J5zxqIcn2tTfpy2sU2MibNScZdl2yB6KvTqS8+jgRuviLtKmSgPjg2OTUpEcAVckXKlUJGgPhmmsbGR2traaHBw0BerpqYmZw7VLUIqlBsJjg2OTerYwBVwRcqVQomgDSjVtLWNBIuQCoUIBjMSDh\/pUKm\/Alf8kbL1uVK8k7CLNR2ah\/MEizArVBEJHRhf91KnAq6AK1KuFDoSzMN5groI5nVWKEQwvDvD4SMSlDp8cAWR4BgCYYvlXTpPsPux16j7sb3ee+VxgbxOWXRgfN3D2UsRAFdskCpkOjRKBF05TzDPe4WaJIYIwrFJHRu4Aq5IuVLYdGgezhMsytIIpEORDrVxaGwLEYQI2nCmkJGgUn+XzxPURTCvC+SRDo3uynD4GBOMZslpC3AFY4IlCLh8nqC+NCLv44HowMFuDo4NIggRlCIAEawMqZTulobmM+9+ljgazNsJ8kEww9kjxSXtguAKuCLlSmHHBG0ASttWIoJ6KrTlyjriU+TzfsGxwbFJOQ6ugCtSrkAEbZBKydZWBIswHoh0KNKhNt0PIggRtOGLxOfalJekbaw7xiRZ0UrKljRI0cYDIYIQQZs+BRGECNrwReJzbcpL0hYi+Dt0i7Q+UBEKjg2OTepcwBVwRcoVpENtkErJVvJVUnvLE15t8nx0kgk3HBscm7QLgivgipQrhRbBoaEham1tpeHh4RK8snyUkn50UlHGA5EORTrUxqlBBCGCNnyRBB425SVpG1s6VO0YU19fTx0dHUnW2brsqAZ56pXjdP0Du71yIYLW8ObuBjj80iYFJhBBm44e5XNtykraNjYRDNs7NO6XUII7MDDgFd3e3h4qvFENUsTxQESCiARt+iVEECJow5con2tTVtK2sYmgEqaWlhaaO3duovXmzbj54ohTie\/ChQtpwYIFvs+NapAijgdCBCGCNp0UIggRtOFLlM+1KStp29hEkCvKL16N0yJ0UfQDLKpBlAjm\/fxAExs4Njg2qYMBV8AVKVeUFixatIi2bNmSeFBkUy8\/29hEUEVkg4ODvnVKamKMJA2rRHDp0qU0Z84cqqur8\/54jbX3TbrhWz\/x\/v\/hf\/8RmjvjnEoxdeZ+dmyHDx+mqVOnUk1NjTP1TrqiwKUUYWASLILoQ6exYRz4D18HDx6kFStWFEsEk3ZMfuVzBNjT00PNzc3U1dUV6MjNTb2XLFlCixcv9or88S\/epr\/43umG+9YNdTR7+sRqvEpVnnny5EkaGRmhKVOm0IQJE6pShyw+FLiUtgow8WcqcDmDy+bNm2nTpk3jgCpUJFhNZ8ZiyMsygoRQieC6deto+vTp4yLBP+v5Ce3c96a3afZzy6+o5muk\/mwexz1y5IiHx8SJxRH\/KKCBSylCwMSfNcDlDC56JLhz507auHFjMSPBbdu2UWdnp4cMfwXs27ePduzYERqpRTmlqN95feLKlStp7dq11NDQUGIeNiZYtJMjdHAwzuPPLOBSigswAVei\/LD+e9Q8DJuykraNbUyQK6oiMhakm266yZu9yWOBq1atoiTXD0ZNyAlqkKKdJG+SCY4Njk3qYMAVcEXKFbYrpAjqE1QaGxupra3NE0FeLhElUjbgKrHl\/3L5kkX6QQ2i7xRTtJmhjB8cGxybtO+BK+CKlCsQwY4OSloEzcXy0okx5iBt92OvUfdje722LdJOMYrMcGxwbFLHBq6AK1KuFFYE+cV5PJDH\/\/R0qBLEsMXsNuCWYxsUCd687WX6252HvCJ3\/9VV3uSYIl1wbHBsUr6DK+CKlCuFFkH95XXA1qxZE7ibiw2w5doGiWBRt0tDJBjOJDj8UnyACUTQxv8WckzQBqC0bYMapKjbpUEEIYK2fRAiCBG04QxE0AatFGz9GkSfGVrESTEMOxwbHJu0+4Er4IqUK4VPh+rrBBVo1d41ACKIDmzTgeHwkQ6V8gVc8UeqsJEgC+DWrVupt7eXamtrPXQkpzxICVeunV+DFH1mKCLBYDbBsUEEpb4GXIEIjiEQtpF13OsEpQRVdn4i2PHwHvr20wc9kyLODIUIQgRt+hGcPbIpNnwpZCTomggWfWYoRBAiaOPUIIIQQRu+FFIE1WDo8uXLqa+vb2wPz6ymQ9WeoVdfeh4N3FisjbMVmeHY4Nikjg1cAVekXFFagPMEQxDj\/UQfeeQRG0wrsjW\/SvSZoZ9smETf+4uZFZXv6s1wbHBsUu6CK+CKlCuFFUEbgNK2NUWw6HuGIhIMZyAcfik+wAQiaOO3C5sOtQEpTdswESzinqEQQYigbf+DCEIEbThTaBH0WyeYtW3TsDziNJ3h2ODYpI4NXAFXpFwpdDrUlXWCN\/zn3fT3e457bXrsnmtt2jZXtnBscGxSQoMr4IqUK4UVQZeWSGB5BCLBsA4Nh48xQanDB1f8kSpkOtQlESz6xtkYE8SYoNTJgyvgii1XChsJ8ou7kA6tb5xJvEaQryVz6+kbX7ysnDbOxT34ikWKS0pkcAVckXKl0CKohLCzs3McXlmaGPPO+R+m+Q+85NWvqKdH4OseX\/c2Do1tIYIQQRvOFDIdagNQ2rZ6g+giWOTlEXBswSyEwy\/FBphABG38NkTQBq0UbPUGefLXU6n7sb3eU4u6cTYiQUSCtt0OIggRtOEMRNAGrRRs9Qb5m9feT\/3PH4YIIsUVyDw4fESCUrcErvgjBRGUMiglO71Bvr5rAj3z6gm6qHaiFwkW+UIHxte9lP\/gCrgi5QrbQQRt0ErBVm+Q\/\/DDU8QbaEMEMdkhiHpw+IgEpW4JXEEkKOVKVe10Efzj7456dSnyEUoYE8SYoG2HhLNHJGjDGUSCNmilYKsapPv+B6l9+ynviRs+fxm1zqtP4enZfQQcGxyblJ3gCrgi5QrSoTZIpWSrRPDGux4gHhPkq+hrBBkDODY4NmkXBFfAFSlXIII2SKVk6yeCRV8jCBEMJh8cfik2wAQiaOOukQ61QSsFW9Ugzf\/xHuIlEnxBBBEJBlEPDh8iKHVL4Io\/UhBBKYNSslMNUjv\/Dvr5uy72nlr0hfKIBBEJ2nQ\/OHtEgjZ8gQjaoJWCrWqQNz++kt45\/zIsj\/gd5nBscGzS7geugCtSrmBM0AaplGwhgujANlSDw0c6VMoXcAXpUClXqmqnRPDEn\/Z69cAawdPNgQ6MjwNpxwRXwBUpVxAJ2iCVki2L4MIv30yv\/+tu74ktV9bR\/S2Xp\/T07D4Gjg2OTcpOcAVckXIFImiDVEq2pgiyALIQFv2CY4Njk\/YBcAVckXIFImiDVEq2LIJfvPlO4okxfEEEkQ4Nox4cfik6wAQiaOOuMTvUBq0UbLlBPr\/qfnpr1pe8p2GNIEQQImjX8SCCEEEbxkAEbdBKwRYiiA5sQzM4fESCUr6AK\/5IQQSlDErJjhvkT9f\/kP7vRVd7T8RCeUSCiATtOh+cPT4kbRgDEbRBKwVbbpD5D+z2Fsrzdeyea1N4avYfAccGxyZlKbgCrki5wnYQQRu0UrDVRRCH6Z4BHI4Njk3a\/cAVcEXKFYigDVIp2bIIfva\/\/IJ++57zsVBewxyODY5N2gXBFXBFyhWIoA1Smu2xY8eora2NBgcHvX9tbm6mrq4uqqmpKSlxdHSUVq1aRQMDA2O\/tbe3U0dHh+\/TWQRxonwpNHBscGzS7gqugCtSrkAEbZD6na0StXnz5tGCBQtI\/b2+vt5X2Fgwly1bRrfeeis1NDREPlEXQRymi3RoFGHg8PHBFMUR9Tu44o8UxgSlDAqx27ZtG+3YscM3GhwaGqLVq1fThg0bqLa2NvJpughioTxEMIowcGwQwSiOQATDEYIIShlUpggywP39\/YHpUrNYfYkERBAiGEVPiCBEMIojEEGIoJQjZdmp8cGFCxd66VHz4iixs7Nz7J+bmpqot7c3MCrUI8HbZp+kz139Yaqrw96h7OwPHz5MU6dO9R17LavxcnATcPEXQXAFuIR1b+YH\/+Hr4MGDtGLFCtqyZQvNnTs3017hrFOnTp3KUg3VeCDXKWhiTHd3Nw0PD4\/9bv7dLxJUE2POeXottf3JXFq8eHGWXrsqdTl58iSNjIzQlClTaMKECVWpQxYfClxKWwWY+DMVuJzBZfPmzbRp06ZxQEEELT2cRAD9iuQxwpUrV9LatWt9J8rokWDPdWfRlZfPQCRIRHv37vVIu2TJEpoxY4Zla+XXHLiUti0w8ec7cDmDix4J7ty5kzZu3IhI0MZNRs0IDSsraqKMvlgeu8WcQdKlwWsbLlVqC1xKEQQm\/qwCLu7jkpl0aFRKU0Ftu5yC71OR4Lve+hX93Z9fWKmPzM39Km+\/dOlSmjNnTm7eq9IXAS6lCAITf1YBl3BckA4VeiNzoby6TU144QXzvDi+paXFG2Q1F8uHLaznsg4cOEAz736W3vXWUeIxQVxAAAgAASCQLAL8Yb1u3Tq64IILkn1QhaVnJhKs8D0ib2ch5D+4gAAQAAJAIHkEWPyyLoCMQmFEMPkmxxOAABAAAkDANQQggq61GOoLBIAAEAACsSEAEYwNShQEBIAAEAACriEAEXStxVBfIAAEgAAQiA0BiGBsUKIgIAAEgAAQcA0BiKBrLYb6AgEgAASAQGwIQARjgxIFAQEgAASAgGsI5F4EeSeanp4er11c2L0gbgLxlnKtra3ehuNRmwroWPGBxn19faJDi+Oucxrl2eCi6mPuVpRGPdN+hg0u5iYXee5fNrjotnnvR0H8VH1FbXCSNo9tnpdrEeTt0tix8zFLe\/bsGft\/yUG8NiBm1VZ32vPnz\/d23Zk3b17g8VT6IcZ8XNXWrVtDj6jK6ntH1csGF70sdYTXmjVrfDGMem7Wf7fBxdzrN2oT+6y\/e1j9bHBRHwYdHR3e7lZ57kdRAjgwMOBE4JFrEWQB5IsJ6dKXSVwOw3RMNocR59mplYMLO7dly5bRiRMnKOicy7jarVrl2OAStWl9td4hiefa4qKfaJPnfuSHtYqCZ8+eTfv37\/d8L84TTIKVgjKDNtoOioQERTpnokfCHP2afw97oTx33nJw4Q+qK6+8kr7\/\/e8HRtPOEcSosA0uHOHomQPX3z2s\/ja4+EWCRcGJMeQNxfni\/Z7b2toggtXsGH6RHzsyPjfP77T6atY1qWebkZ\/N17v0VI+k6p5kuba4MG587uItt9xCd955Z65FsL+\/f+yw6jC+sAjyWXp85X3M3ZYv+gb\/7e3tnhAU7TI\/BrL8\/rlNh0IETx8hJXVqOknZwd177725nRhjgwvz6Otf\/7p38DBvBhw2rprlji6pmw0uanxUTYbhe5cvX55LztjgotKB69ev99KANtkXSRu5YgMRzEBLIR16WgTVxCBpOjTvAsjUtMGFbZ988slx48p5Tanb4GKmQ\/M8cxa42Dt0iKA9Zoncoac\/izoxZvXq1bRhwwZSIqhHhiboRZnJZqb5wiYM6ctGdLzymOaywcXELM\/9ywaXIn0chDltiGAikmZfKJZIjI6l76KWSOQ5nWUyx2bKu35vnqMdfk8bXEwnl+e0nw0ufunQvKaJIYL2mlSVO7BYPnixvJrcwAP3QRFPXhdAhy1+1nEpkgjyu9rgoi+Wz\/uicBtc+INg0aJFHnXyjkuQU0ckWBW5w0OBABAAAkAACNghkNvZoXYwwBoIAAEgAASKiABEsIitjncGAkAACAABDwGIIIgABIAAEAAChUUAIljYpseLAwEgAASAAEQQHAACQAAIAIHCIgARLGzTu\/\/ir776Kk2aNMnbCEBy8bTt48eP06WXXioxL8tGLTVpamqyOobKlQ3L1fulNfU\/7eeV1ei4yWkEIIJON19xK2+7ODuNdUuVCFkl96bJAv14srSe6wo2aeGB58SLAEQwXjxRWkoIZFEEbeukQ+WKo4cIpkRwPCY1BCCCqUGNB9kioO9IwveqFOOePXvGduTgf1e72pi73qiU3eTJk72zzQYHB70qqH0\/9SNv9PLD0qv6M\/SUoDpVQb1j0OnzQXZKBHl7u7vuussrxkw56juRmM9Rh\/5+8pOf9O5XWB09epRaW1tpeHjYu+X222+nRx99lNauXUsNDQ3evwW9k197mSLIf3\/jjTe8P3ySuI6v3\/1+5xBGnU3oygeCLb9hnw0EIILZaAfUwkDAb59O3QGbUVfQxsVcbFdXl7cvpn7IpxJY\/ZT4sA3EVX1UeXxoqHniRlQkaNrr+0yyULNY8YncXF9V\/tatW72xRRYz\/cRyrodenhL6iy66aOx+8x3V30dGRrwjj9TRUCy26sy7qD1k\/USQzxNUou+Hq960EEF09awhABHMWougPh4CUc40SnCUSKhTvU0R9Ds5ImyDbL9oxLQPq1PU5tvmxstc\/6gISP9diaAp6uap5rrI8TP0U0YU9cJSnn4iyFGmEm4uIwwHiCA6eNYQgAhmrUVQnzEE9NShOdsyyNGaKcPm5mbfSNBMS+qw+6Uyg56nb7Yd5vyjJub4CZ7fv5kpYjPlyxEdH+bKl5+Y6WVydKk2ejZpF3RUlJ8I8r366elh4g0RRAfPGgIQway1COpTgoDu+PVxQf3AYCVq5jhdUCTI95oRTBj0QQIXlqLVy6tUBLksNbanRNovErQRwRdffJFUulW6zAQiiA6aNwQggnlr0Ry\/jy4kKtLh8TIeP1u1ahWZJ77rQmemQ20PEE4jHWqO+enPZMEKS22qdKgugn5Rl54O5UjQ9qy7JNKhUR8kUWnhHFMer5YCAhDBFEDGI+wR8BsT1KMxfaKI3wQPFRmqdCjXQBdKVT6nDlUqz29cTtU8rokxeuSljxPOmjWrZOKLKYL6vaquXD+e5OIngtKJMVyGGtOLGoutdGKMSlerGb3qPfQJQSZbIIL2\/Qd3yBGACMqxgmXKCOgHmfKj9VSnvryB04PXXXfduGUQLH7XX3893XHHHZ5I8HIAUxhVdKiWTvAzog4RDltOIJ2s09nZOYakX2pTLV0wnb\/57PXr13vjfjwZRr2\/HgnyQ0wMzSUS5jIRvidoeYeKvvm\/6sPBb4mEfr+fgOnjsdxOM2fOpN27d3tCbH6sqHcwo+SUqYjH5RgBiGCOGxevBgT8oiq\/GaFSpCRjgtKypHaIBKVIwa4cBCCC5aCGe4CAAwiY454q6tPXBdq+BkTQFjHYZx0BiGDWWwj1AwIVIGDuohO09EH6CHND64cffti7VV8iIS1LYocNtCUowaYSBCCClaCHe4EAEAACQMBpBP4\/BdA0hXialw4AAAAASUVORK5CYII=","height":271,"width":449}}
%---
%[output:619496e7]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"440"}}
%---
%[output:5f4ae634]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"0.0300"}}
%---
%[output:13430917]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_JH","value":"0.0750"}}
%---
%[output:104f42b6]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber","value":"1.2033e-09"}}
%---
%[output:71e550b8]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rsnubber","value":"1.6620e+04"}}
%---
