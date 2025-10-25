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

ts_afe = 1/fPWM_AFE;
ts_inv = 1/fPWM_INV;
ts_dab = 1/fPWM_DAB;
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
V2rms_load_trafo = V1rms_load_trafo/m12_load_trafo %[output:7a651282]
I2rms_load_trafo = I1rms_load_trafo*m12_load_trafo %[output:4b19a8b8]
lm_load_trafo = V1rms_load_trafo/I0rms_load_trafo/2/pi/ftr_nom_load_trafo;
rfe_load_trafo = 2e3;
rd1_load_trafo = 1e-3;
ld1_load_trafo = 50e-6;
rd2_load_trafo = rd1_load_trafo/m12_load_trafo^2;
ld2_load_trafo = ld1_load_trafo/m12_load_trafo^2;
%[text] ### Current sensor endscale, and quantization
Pnom = 200e3;
margin_factor = 1.25;
adc_quantization = 1/2^11;
Vdc_FS = Vdc_nom * margin_factor;
Idc_FS = Pnom/Vdc_nom * margin_factor;
Vac_FS = V1rms_load_trafo*sqrt(2) %[output:44b2d21e]
Iac_FS = I1rms_load_trafo*sqrt(2) %[output:76f4e179]
%[text] ### Single phase inverter control
flt_dq = 2/(s/(2*pi*50)+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d); grid on
% [num50 den50]=tfdata(flt_dq_d,'v');
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
%[text] #### Resonant PI
freq = 400;
omega_set = 2*pi*freq;
kp_inv = 1;
ki_inv = 45;
delta = 0.01;
res_nom = s/(s^2 + 2*delta*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta*omega_set] %[output:723f6bfb]
Aresd_nom = eye(2) + Ares_nom*ts_inv %[output:5e6b0628]
a11d = 1 %[output:8a55130b]
a12d = ts_inv %[output:41bcb827]
a21d = -omega_set^2*ts_inv %[output:4bafd65e]
a22d = 1 -2*delta*omega_set*ts_inv %[output:07ba8e5f]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:33a36eb7]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:847036c0]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:27dad7f7]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:6211d779]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:1d06662e]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:0b114241]

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
figure;  %[output:0336b779]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:0336b779]
xlabel('state of charge [p.u.]'); %[output:0336b779]
ylabel('open circuit voltage [V]'); %[output:0336b779]
title('open circuit voltage(state of charge)'); %[output:0336b779]
grid on %[output:0336b779]

%[text] ## IGBT and snubber data
%[text] ### HeatSink
weigth = 0.50; % kg
cp_al = 880; % J/K/kg
heat_capacity = cp_al*weigth % J/K %[output:68368d88]
thermal_conducibility = 204; % W/(m K)
Rth_mosfet_HA = 30/1000 % K/W %[output:693c913f]
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

JunctionTermalMass = 5; % J/K
Rtim = 0.05;
Rth_mosfet_JC = 19/1000; % K/W
Rth_mosfet_CH = 6/1000; % K/W
Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH % K/W %[output:5da90709]
Lstray_module = 12e-9;

Irr = 475;
Csnubber = Irr^2*Lstray_module/Vdc_nom^2 %[output:3ed5876a]
Rsnubber = 1/(Csnubber*fPWM_INV)/5 %[output:73b64db6]
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
%   data: {"layout":"onright","rightPanelPercent":42.6}
%---
%[output:7a651282]
%   data: {"dataType":"textualVariable","outputData":{"name":"V2rms_load_trafo","value":"   6.600000000000000"}}
%---
%[output:4b19a8b8]
%   data: {"dataType":"textualVariable","outputData":{"name":"I2rms_load_trafo","value":"       30000"}}
%---
%[output:44b2d21e]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     4.666904755831214e+02"}}
%---
%[output:76f4e179]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     8.485281374238571e+02"}}
%---
%[output:723f6bfb]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"6","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000001000000000"],["-6.316546816697190","-0.000050265482457"]]}}
%---
%[output:5e6b0628]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"2","name":"Aresd_nom","rows":2,"type":"double","value":[["0.010000000000000","0.000001000000000"],["-6.316546816697191","0.009949734517543"]]}}
%---
%[output:8a55130b]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:41bcb827]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:4bafd65e]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"    -6.316546816697190e+02"}}
%---
%[output:07ba8e5f]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.994973451754256"}}
%---
%[output:33a36eb7]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.149016195397013"],["36.521945502464568"]]}}
%---
%[output:847036c0]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:27dad7f7]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:6211d779]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:1d06662e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.155508836352695"],["27.166086113998457"]]}}
%---
%[output:0b114241]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.037191061070411"],["1.937144673860303"]]}}
%---
%[output:0336b779]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAigAAAFNCAYAAAAq3JTxAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQtwXtV17xcNk1iBEFs84igqNQ+Z0KZXYJfUcZ0Qqkk9bUcmJW0l+bZNXaV1byDcW\/CV5JIwJTwsaWxayqN1qaLSeyvbfcCNXW6HUichIcJDMKA2KWAZYTtCEB7CISRmMtz4zjpki62j89j7nLW+c77z\/b+ZTJC\/vdfZ57f2Wfv\/rf04Jxw\/fvw44QMCIAACIAACIAACJSJwAgRKibyBpoAACIAACIAACAQEIFDQEUAABEAABEAABEpHAAKldC5Bg0AABEAABEAABCBQ0AdAAARAAARAAARKRwACpXQuQYNAAARAAARAAAQgUNAHQAAEQAAEQAAESkcAAqV0LkGDQAAEQAAEQAAEIFDQB0AgI4HZ2Vnq7e0Nao+MjFBzc3NGS+nVJicnacOGDbRy5UoaHBykpqam9EoCJWp5jy7NNRw+85nPUFdXl0uVUpcZGhqi7du3B23cuHEj9ff3e7V3165dtHnzZtqyZUspefD97du3T\/358IKGwnVDAAKlblyFhpaNQC0H7ziBwgPAxRdfTKtWrVLBE3eP2teNu5ksAx4PkA888IDX4J+ljq8D+Brr16+fq1ZFgVI1QenrY5TPRwACJR8\/1AaBQggcO3aMBgYGaM+ePTQ2NlYzgcKZm1pcNwqqGew6OzudxYbJMPgM\/lnqZOkERqD4tC18nbJnUEw\/PXLkCLIoWTpJg9eBQGnwDlCW27dT3dwmO2VtZw8uuOACuv7664Nm80BlT3eYX\/sTExMLvrdtXHrppfSpT30qKNPS0kKjo6PU1tYWicIWAuHy4ewCf2+mfDo6Oujmm2+m9vb2IDDbA3uaHZ4qMtfdv39\/0D7+mCmea6+9lj7\/+c8H4sR8wiz43w1TW8CYQdEubwY5Y8seMO17vO2222h4eDjyutPT00H7ZmZm5tpk+9DmyMxvv\/12+sIXvkDm\/ph\/mLVhZ6bOogZj41f7uuZ+w\/dlfN3a2jonssL8du\/eHUyZmI\/dP8L20oRhuD8m2Urqh0mZFpsJt9m0PSx6ws+XzdbYuOqqq2jv3r3Ez4\/xnX3P\/G\/mGrZv07hE9cOyxB20o9wEIFDK7Z\/Kty48KNk3bIJs1CAUHljYDosDI07C30cNoEmDO38X1zYzmJx66qnz1qAYgWK3gYVAlKCwRUrYjpRAifqFHh4swgNXHFf+9ziB0tfXR1dcccUC9kmCgL87\/fTT6cUXXwwEWJRo4GvaA2m47XH9wlz30UcfjRQbd99999y6D7u\/2QNwWKCEbZnv40RKUp\/lOocPH44VQnabwuIkLCKNOPjwhz9MX\/va1+bFiyiREfV8hQUGl4lqI\/+7uU6abZtL2bM8lQ+ydXyDECh17LwqNN0EYPsXpAnufH929oB\/JZvAZw8AdjC1fznaAxqLAPML39gw1w7\/Ujdco763g+3HPvaxWIFi\/8L0tZMmUDhrxJ+0qZakDA9ndV5++eUFTOxf\/cxp+fLl8+7RZYonPP1k2Bt\/crYk7HduC6\/HiMrsMMt169YF92tnXFwWDrtM14TLhP82TIyY4vanXdv0vaj7Mf\/GQpbvOW6Kx+Zo+lPYp\/fff38gdKIyInF2w1k0kzWybSRd22RYTP9P4yIxlVWFWId78CcAgeLPDDUECcT9ujIBngPzihUrInew2GUOHToU+auYm2rb4F\/tZsdN2iLXtF9+cQLADth8fV87UgLFvjaLDf7YA2LcwGEP0L\/\/+7\/vLFCiMk5R1+V2hKew4jIUXJYHWtMOm23U9cJTXUkCJW5qK1wnKRsSJW7D92amD8NCx4iyOCGR1j9t\/9o24vwazsYYVkagxE3t2TvU7L5snkt7es2EBptL1LSiYAiBqQoTgECpsHPr4dZqLVDsbbppA4CvsGDeUduOXe3Yg294MGPb9jZjlwwKl7EXlvLfvKU1nEEKD5C+AiXMMZxlCQsjKYFi+necMOKdTVECJTxVlJZBqQeBEpWxM34N97+4DIptI+7ZgECph6hanTZCoFTHl3V5J1mneMJTEWZOP+7XaFRKPk2gJE3N2L\/qGTz\/yowTKK52OHUeFg9m6issUFgEuCw+DA\/edoYhPE3GA3raFA9nd9IG+LCNrFM8doeOy0qEO31YbISzCeae7UyauR\/Td8J1oqZ40h427SkeI2ZN5ilOoERlngyjcAYlblFzeHopaYonigumeNJ6C76PIwCBgr5RKAHtRbJJA3yaQElqW9T6jDiBkmaH0+FmPUnYGS4ChetE7eIxtsI7MewDznwWyZpUv12Hr3vZZZcF2Z2oD3OKuj\/XRbJs04i2sDCKW0Bq17HL8DVvueUWuuGGGxYs6OU6YYHC\/xa34Nbca5ogjpr+SMtg2Rzj7jFJXNiC4Morr4ztW0k2uA1Ri2ddF8naXNIyiIUGIFy81AQgUErtnsZpnOs2Y3uLcNo246iFtz5TPEw\/afogbRGqfbJskh2+TnhL6uc+9zl6\/PHH5xaFRmVQ7MErbqGvbTu8NiZKwNgDtV2X\/9sIlKjr3nnnnfNOROXD4+z1Llm2GdtCwx4wo7agx21vDnO118SwTea2detW2rRpU4DDzoSZ3Vhx25bTzi9J2mbM13LNLMStHeEsWtTgH5c1YkZRW7yjsjBx4pb\/PXxybdxaHmPDJdPXOJEOd+pDAALFhxbKFkIgbcdEIY3CRZ0JRE0lhXdqxZ1DY18ky0Ftzo1EwXkEbEFphFjUzp40bDioLY0Qvk8iUDmBwkGMz2XgA6Wigl7SYV7oKuUkAIFSTr+4tippiitpairKfpaj7l3biXLzCURN8XCJtMMNo0RlVd6dhD5SWwKVEihpC+nM96tXrw5erGX+5gfO9yVdtXVTY18NAqX+\/R\/+YcB35CtOuA7e7VLbvhCeevURJ9xSCMra+qtqV6uUQOG5V34g+BOXQQk7kH8ljI+P1\/QNsVXrRLgfEAABEAABEJAmUBmBwr\/QrrvuOurp6QlECgSKdFeBPRAAARAAARCoHYHKCBTOhPCHTzdMWoNiozVp5+7u7mDKBx8QAAEQAAEQAIFyEKiEQOF56bvuuouuueYa4pfCuQgUs\/6E3WC\/ETfslrPPPrscnkIrQAAEQAAEQCCGAL+J+qyzzqoUn0oIFJ7S4XMX+CTMtF087D1XccJlWaBMTU1VyulF3wyY6ngAXOW5gqk8U8RVMHUlUPcCJWp3gLn5qFeh++7cQYBy7Uru5cDUnZVPSXD1oeVWFkzdOPmWAldfYunlq8i07gVK2G1pGRTOtvCJiknTOrbNKjo9vavrlgBTHb7gKs8VTOWZIoMCpq4EKi9QTMaEd\/csX748eNusOcLaQEo6JhwByrUruZcDU3dWPiXB1YeWW1kwdePkWwpcfYmll68i08oJlHQ3+pWootP9CMiXfuaZZyq3mEuekr9FcPVnllYDTNMIZfseXLNxS6q17AMfpEPffFjecIEWIVBS4EOgyPdOBCd5pmwRXOW5gqk8U\/RVeaY7vvE8Xb7jCZq9+RJ54wVahECBQKl590PQ10EOrvJcwVSeKQSKPFMIFHmmdWERGRR5NyHoyzNF0AdTHQI6VhEDZLlCoMjyrBtrECjyrkJwkmcKgQKmOgR0rCIGyHKFQJHlWTfWIFDkXYXgJM8UAgVMdQjoWEUMkOU6dN8hGrrvGaxBkcVafmsQKPI+QnCSZwqBAqY6BHSsIgbIcoVAkeVZN9YgUORdheAkzxQCBUx1COhYRQyQ5QqBIsuzbqxBoMi7CsFJnikECpjqENCxihggyxUCRZZn3ViDQJF3FYKTPFMIFDDVIaBjFTFAlisEiizPurEGgSLvKgQneaYQKGCqQ0DHKmKALFcIFFmedWMNAkXeVQhO8kwhUMBUh4COVcQAWa58iixvNcZJsrJcS28NAkXeRQhO8kwhUMBUh4COVcQAWa4QKLI868YaBIq8qxCc5JlCoICpDgEdq4gBslwhUGR51o01CBR5VyE4yTOFQAFTHQI6VhEDZLlCoMjyrBtrECjyrkJwkmcKgQKmOgR0rCIGyHJdd\/tj9ODTR7EGRRZr+a1BoMj7CMFJnikECpjqENCxihggyxUCRZZn3ViDQJF3FYKTPFMIFDDVIaBjFTFAlisEiizPurEGgSLvKgQneaYQKGCqQ0DHKmKALFcIFFmedWMNAkXeVQhO8kwhUMBUh4COVcQAWa4QKLI8S2FtcnKS+vr6aHh4mNra2iLbBIEi7yoEJ3mmEChgqkNAxypigCzXC254iI7Mvo5FsrJYi7N27NgxGhgYoP3799Po6CgESg1dgeCkAxtc5bmCqTxTiGl5ps1XfTkwipNk5dkWYnHfvn00NDQUXBsZlNq6AEFfhze4ynMFU3mmECiyTDlzwhkUCBRZroVZm52dpeuuu456enoCkQKBUltXIOjr8AZXea5gKs8UAkWeKTIo8kwLs7hr167g2itWrMAalAK8gKCvAx1c5bmCqTxTCBRZpg8ePErr7ngMGRRZrMVY44Wxd911F11zzTU0PT3tJFDslu7du7eYhlfoqsy9tbW1QndUjlsBV3k\/gKk8U7YIrvm5dnR0BEZef\/+l9Pr710Gg5EdavAWe0rn44otp1apVhF08xfgDv0p1uIOrPFcwlWeKDIos0y98\/Vna9E8HIFBksdbeGq896e3tpYmJiQUXHxsbC0RL+INtxvJ+QtCXZ4qgD6Y6BHSsIgbIcTVnoPzED16il\/7yN+QMl8DSCcePHz9egnYU0gRkUArBTghOOtzBVZ4rmMozhZiWY2rv4Dnxpafohb\/9QznjJbAEgYKD2mreDRH0dZCDqzxXMJVnCoEix9ReILvoyd0083\/\/VM54CSw1tEBx4Y8pHhdKfmUQ9P14uZYGV1dS7uXA1J2VT0lw9aEVX3bHN56ny3c8ERQ4+cFhOvLwv8gYLokVCJQUR0CgyPdUBCd5pvhVCqY6BHSsIgbIcDXrT85sXkSvfuG\/0tTUlIzhkliBQIFAqXlXRHDSQQ6u8lzBVJ4pxLQcU3NA25pzFtM3t30CAkUObX1YQgZF3k8I+vJMEfTBVIeAjlXEgPxc\/\/xLR+hP\/vnpwNDuT19Iv\/NLKyBQ8mOtLwsQKPL+QnCSZwqBAqY6BHSsIgbk42rv3uHpncc\/+yGq4liFKR5M8eR7UjLURnDKAM2hCrg6QPIsAqaewByLg6sjqJhin793iv5s7+G57MmacxdDoORDWp+1q6hKi\/YEgpOOB8BVniuYyjNFti8f06jsCVus4liFDAoyKPmelgy1EfQzQHOoAq4OkDyLgKknMMfi4OoIKlSMxckVO56gB58+GnzDUzs8xQOBko1n3deqoiot2ikITjoeAFd5rmAqzxQZlOxM\/3TvYbr+3je3EvPOnd2XXzhnrIpjFTIoyKBkf1oy1kTQzwgupRq4ynMFU3mmECjZmNqHsnHWhHfumOwJMijZmNZ9rSqq0qKdgqCv4wFwlecKpvJMIVD8mY49\/BxdsfPJoGKUOIFA8WdaiRoQKPJuRNCXZ4qgD6Y6BHSsIga4c73tK9+ma3cfTBQnECjuPCtVEgJF3p0ITvJMIVDAVIeAjlXEgHSuvCB23R2PEf+\/yZz0rz2Lei5aGlm5imMV1qCk9JMqOj390dAtgeCkwxdc5bmCqTxTiOl0pvZ6EyNOwmtOwlaqOFZBoECgpD8twiUQ9IWB\/tgcuMpzBVN5phAoyUzNCwBNqbg1JxAoOn2zrqxWUZUW7QAEfR0PgKs8VzCVZwqBspApT+M8cvhV+tT\/+tbclyxMbus+n\/iUWJdPFccqZFCQQXHp+6JlEPRFcc4ZA1d5rmAqzxQC5S2mLEy+dvAV+syPd+j4Zk1s70Cg6PTVUlutotOLBo6gr+MBcJXnCqbyTCFQKFj4+i\/feok23zM5D7Bv1gQCRad\/1o1VCBR5VyHoyzNF0AdTHQI6VhsxBrAo4f\/t+MZzxItg7U8eYWLsVHGswhQPpnh0IlCC1UYMTrWADK7ylMFUnmmjiWmzTdjeMmxP5fisM0nyBgSKTl\/NbfXYsWM0MDBAe\/bsCWxt3LiR+vv7Y+3u2rWLNm\/eHHzf2dlJg4OD1NTUFFm+ik7PDTynAQT9nABjqoOrPFcwlWfaCALFiBL7xX7SGZOwZ6o4VlUigzI0NBT4ikXJ7Ows9fb2Und3N3V1dS14uvbt20dcfmRkJBAlLGxaWlpiBU0Vna4TctytIui7s\/IpCa4+tNzKgqkbJ99SVeTqIkrWf\/C91PdLy3xxOZWv4lhVCYES9p4tWMLfcfZkfHx8LmsS\/rsRVKlTb1csVMXgpIjL2TS4OqNyLgimzqi8ClaFK4uSr06+Qn\/\/yPP04NNHFzDgtSW\/cM5i6rnovc7bhb1AWoUhULKSq2E9k0HhbMqqVaucMiirV6+OzLZw5So6vYbuiLxUVYJT0RzD1wdXeY+AqTxTtljPXB88eDRyoashZUQJH0tvv21Yh+RbVqs4VlUqg8KZk+3bt6euK5mcnKQNGzbQzMwMjY2NRQoZ4\/YqOl37QUmzX8\/BKe3eivweXOXpg6k803oTKJwl+fv9z9NXD7wSmSXh+ylKlNjeqeJYVSmBYpzFQoXFR9TiV57S2blzZ7AGpbm5OViPwp+4RbXsdPuzd+9enSe2gaxOT09Ta2trA91xbW4VXOU5g6k8U7ZYZq77n32dDr78Q9p78AfE\/x33aTnlRPrdle+mT3zgXTqQUqx2dHQsKDE1NVVIW7QuWkmBwhmSvr4+Gh4epra2tjl2ZrePPaUTVxYZFK0uV9\/pXT0q+S3j135+hmELYCrPtGwZFJ6y+eLEC3T\/Ey\/PvTk46q45S7LmnMXUXYP1JFmoI4OShVpEHbNOZGJiwstie3s73XPPPal17J06nCUxHwiUVHQ1KYCgr4MZXOW5gqk80yIFCouRUxa9jT77xYOx0zXmjlmQfPjcJdT1c0uDKZxarifJQh0CJQu1BIESt5A16jJGdEQJFHuaxoiQuK3DUVM8cdNB3I4qOl3IjZnNIOhnRpdYEVzluYKpPNNaCBSz5Zf\/f\/i+Z1LFCLeJBchvrlxKH2lbUheCJOyZKo5VhUzxpO208RUo4YPa7MPXzHc9PT1zi2HNYlq+Dg5q0wlASVYR9HWYg6s8VzCVZ6ohUDgz8qPjx2nrvx5yFiNnLllE\/73jp+gdJ\/6E+hZgHYrzrUKg1IJyya5RRacXjRhBX8cD4CrPFUzlmeYVKCxGvnJglh5+5rvOYoSveflHz6Tzl55Ul9kRFy9UcawqNIPCa1DSjqV3cYxmmSo6XZOXi20EfRdK\/mXA1Z9ZWg0wTSOU7XsXrixE+MMv1\/v27OteYuTXLnwPdZzXXFkxEkW9imNVIQLFwLWnWnjNyOjo6LxdN9m6vmytKjpdlpC\/NZfg5G8VNcBVvg+AqTzTcAbFvOWXd9I89fz3nYQI2wgWri5ZRB8869300eWNJUYgUHT6ZaTV8K6eMmVVIFDkOwKCvjzTvGlznRbVv1X0VTkfmowIC5HHD72ceMZI+Kr2kfFz4qR5kVzjKmCpimNVoRmUqD5hn\/JahqxKFZ1e9LOIoK\/jAXCV5wqm\/kxZiByePUbjTx91npoxVzFZkc\/+6tn0wzeON9QUjT\/p+TWqOFaVTqDYyOPOM8nrSJ\/6VXS6z\/1rlEXQ16CKA\/A0qKKvLqTKUzIsJFiI\/Odzr9E\/\/\/uLdOSV1xMPOQtb4VNYzz79ZLr8kp+kk95+IoSIQOet4lhVOoFiZ1DYZ2nvyhHwa6KJKjpdm1mafQT9NELZvgfXbNySajUyU3OWyDdnXqN7\/+NF72zI3FTMkkV048fb6LvH3pgTIo3MVb6XvmmximNVKQRK0jkmWs50tVtFp7veu1Y5BCcdsuAqz7XqTI0IefL57wfHvbvulrFJmxNWf+GcxdRz0XuDr9JOXq06V\/memG6ximNVoQLF3sVThmxJVBeootPTu7puCQQnHb7gKs+1Kkx5OuaF7\/2Q\/mb82QDSg0+\/uYXX9WNECO+a4XfRGAGS9fj3qnB15VeLclUcqwoRKPaunbSTXGvh2KRrVNHpRTNFcNLxALjKc60XpixAWCz8n8dfoH\/jl955rgkx5Mwi1c7202ntT582lw2RJlsvXKXvW9NeFceqQgTKwYMH6corr6Rrr7127vj5NMclvYsnrW6e76vo9Dw8JOoiOElQXGgDXOW5lokpT8c89Z3vByIky1QM07EzIXyYWdsZ73zr32q4bbdMXOV7TTEWqzhWFSJQpN\/Fo9kdquh0TV4uthGcXCj5lwFXf2ZpNWrJ1JwTcu83X6RvPftariwI3xevCbnkvGZaeso7UteEpHGQ\/r6WXKXbXlZ7VRyrChUofNS9z6e9vZ2i3mbsY8O3bBWd7stAujyCkzTRN+2BqzxXKabmtFTOYPzNQzP0yKHvZhYgJhPC60Hev\/QkWtd+RnDjaQtT5elktyjFNXsLqlezimNVIQKlnrpGFZ1eNH8EJx0PgKs8V1emZjcM\/\/\/XDr5CDz19NLcAMVmQX\/nA6fTuphODN+6aM0jk77S2Fl251rZV9X21Ko5VECgpfbKKTi\/6MURw0vEAuMpzZaZve\/ebu1bMFMyef3+Rnngu+xSMyXYE\/79kEX1ixXvonNPfWXdZkDy00Vfz0IuuW8WxCgIFAkX+SUmxiOCkgxxc\/bnap6Jy7Xe+\/Sdo5OvPBotQs+6ECQsQ3hFz\/tKTg8ZxFgQfTEdq9AEIFA2qJbdZRacXjRwDqY4HwHU+VyM+zPQLf\/vXX3+WHj\/yalDQ9ywQ27p9ONmac5fQTy5ZVMhuGJ2epG8VfVWecRXHKmRQkEGRf1KQQak5U75gIwZ9W3x8\/emjxP878vKxXNkPkwHh6ZeWd\/4\/+o0PnR1sxzWfrIeTFdIpSnrRRuyr2q6AQNEmXEL7VXR60ZgRnHQ8UEWuRoAceOEHwUvppl78gYj4CETIkkW06ux302\/9fMvc4tOw+KgiU53e52cVXP14uZSu4lhVmgzKrl27aPPmzYEf+AWBhw8fpvHxcRocHKSmpiYX\/6iUqaLTVUB5GEVw8oDlUbTeuBrxcejlY\/TFiRdp8jvfzy0+7OzHT53aRJ\/5xTPphVd\/GFDMsv6j3ph6dJdCi4KrPP4qjlWlECj8Tp6ZmRnq6+ujK664gvr7+4nPPBkYGKCWlpbg76RP+GWDGzduTKzDp9KuX78+MMnXGRkZoebm5shLVNHp8o+Gn0UEJz9erqXLxNUcvf7Gj47TbV8+QgdfyJ\/5MOKD\/58PIfvVnz2dTll0Ymbx4cK1TExd2lsvZcBV3lNVHKsKFyj2qbLLly+n3t7eQFysWrWKzPH2SQKC3cwChz9cz9jr7u6mrq6uBb1gcnKSNmzYQFu3bg2uwZmbpExNFZ0u\/2j4WURw8uPlWroWXO0Dx159\/Q36q69N06GX8q\/5sMUHT71cesEZdN57TgpuvcgDyGrB1NW\/VSoHrvLerOJYVQmBEna1LVjC37EgOXToUGpWxtSrotPlHw0\/iwhOfrxcS0twNWd9fO\/1N+if\/+PF3NttTdvtd8D88gdOo59937sKFx8uXCWYulyn0cqAq7zHqzhWFS5Q2E0mi2FP8ZhsSlwmJM69Se\/5MVNBq1evjsyuRNmsotPlHw0\/iwhOfrxcS6dxNeIj2O1y8JXAbJ6ttlHi4xMr30PnnFadQ8fSmLr6BuXmEwBX+R5RxbGqFAKFXWWvCzGu27Jli7OQ4DqcOdm+fTt1dnZGLq41AmXt2rV05513Er8LCGtQ5B+UNIsITmmE3L+3j1j\/+reO0PSxt9NhgW22tvjgKZefbF5Ev3bhGbToxLc11Hkf6KvufdGnJLj60HIrC4HixqnwUmbRbXgHkBEoR44cmVsYG1fW3AQ73f7s3bu38Pur9wZMT09Ta2trvd+GevtnXn0juMZz33uD9j\/7Oj0y\/XrwN\/933k\/LKW8uLn3vu04k\/u+1y0+it7\/thOBv\/pjv816n3uujr+p4EFzzc+3o6FhgZGpqKr\/hElkoTQZFkgkvhOXpouHhYWpra5szHTXFE1fWFihVc7ok6yy2GvXXU9TJpl85MEuPHH5VbKEp+8Ne7\/EzLScHu13m\/Xvzoixua8g6jdpXtZ0NrvKEkUGRZzq364anW5I+aVuH7bpJu384Y7Js2bK5qSMWKDfddBNt27YtcqtxFZ2u4EYvk1UOTmZ77bdmXqN7v\/mSyKmmBq4tPJad1kR8xHrLu98xJz6+\/e1v0y+0vyXIvZyCwpEEqtxXi3Q5uMrTr+JYVYoMCi+S3blz57zzSOztwuvWrUs8E8XetWOyJHHnp4TFS9KOH+5CVXS6\/KPhZ7HegpO9zuNHx4\/Tvf\/xUvA2W\/5ILDK1sxt8vsfy95xEK888JbDPh4uZzEsa5XrjmnY\/ZfgeTHW8AK7yXKs4VhUuUJJ23dhi4sCBA8Ei2HvuuWeBZ8MHtdmLZM13PT09wbkn\/LEX5MYtqDUXqaLT5R8NP4tlCU72mR783\/\/25Mv06OE3XySX5022Ng0763H+e0+mzv\/y5nSLER9+5JJLl4Wr5D0VbQtMdTwArvJcqzhWVUKgyLv6LYtVdLomLxfbtQhOtvj4x0e\/Q195alZNePx0y8l0yXnNdP7Sk2Lf6eLCJW+ZWnDN28Z6qw+mOh4DV3muVRyrChco7Ka0KR4+EdaclXLLLbfIezbBYhWdXlOAERfLG5yM+Hj0yKv01He+L3aYGDfVznjw1loWHktPecfcv9tliuYYvn5ermW7nzK0B0x1vACu8lyrOFaVQqCwq6LOQeGXBprj6G+99VYaHR2dtytH3sULLVbR6bXglnSNqOBk1nlwvYemjtJXJ19RFR4fbz+Dmt7+tqCZPus8imbny7XM7a2HtmEg1fESuMpzreJYVRqBIu8uGYtVdLoMGXcrtvj4u4efoy9963lqWrRIdIEpHyZ25qlNdMa73k6\/+6GWeQtLTVbEvcX1WRJBX94WV9klAAAgAElEQVRvYCrPlC2CqzzXKo5VECgp\/aSKTpd+NIwAeeTwd2nvk7NB5kNid4s93fKR5UvofYsXBW+xtT+NIj5cfIag70LJrwyY+vFyLQ2urqTcy1VxrCqFQDFvGJ6ZmVngjbSj6N3dl61kFZ2ehYQ53+OWLx2hye98P5cA4VNKTzzxROKsx0+d2hTsbHnnj6db5kQJDhPzdhOCvjey1ApgmoooUwFwzYQtsVIVx6rCBYp9uqs574S3BJuXBfb3989tD5Z3abrFKjo96a45G3Lo5WO065HniV8qZ0\/PpNN6swSLDDPlctmFZ9C5p7\/58jjzHYKTK0m\/cuDqx8ulNJi6UPIvA67+zNJqVHGsKlyghM9BsU965YWzO3bsiHzxX5qzpL6votMNGxYfky\/8gG7Ze9grI2IEyFmnNdFvrFwamOOFpq4fBCdXUn7lwNWPl0tpMHWh5F8GXP2ZpdWo4lhVOoHC24kPHTpEnDlJOrI+zVlS31fF6WZr7quvv0F\/+cC3UwWJmWrhNR\/9a88SPd8DwUmqd863A67yXMFUnilbBFd5rlUZq2wyhQsUbox93LwtSu6\/\/34aHx9HBiVjXzaiZPi+ZxIFicmI9K09K7hS8LfiGhAEp4wOTakGrvJcwVSeKQSKDlMIFB2uFH7LMAuW7du3E79Pp4izT+zbrEenszAZuu8Z2vGN5yM9xuKDXzTX\/XNL1cVIVAMQ9HUeJHCV5wqm8kwhUHSY1uNYlUaiFBmUtEYW+X29ON1kS67Y+UTkwlYWJbs\/feFchqRIpgj6OvTBVZ4rmMozhUDRYVovY5XP3RcuUFxfFtjc3OxzX2Jly+50FiZffmqW\/ugfnlpwzyxKbus+32sBqxi4BEMI+jqUwVWeK5jKM4VA0WFa9rEqy11DoKRQK6vTTcZk3R2PzbsDFiU3XtpGP\/u+k1XXkWTpbKYOgn4eevF1wVWeK5jKM4VA0WFa1rEqz90WJlB4t87mzZtT275x48ZgR09RnzI6ncUJCxP7jBIzhaO5uFXKBwj6UiTn2wFXea5gKs8UAkWHaRnHqrx3WphAMQ1PmuLJe3MS9cvkdBYkXz34Cl2588m5WyvrNE4SewR9iZ650Aa4ynMFU3mmECg6TMs0VkndYeECRepGtOyUyekX3PDQvKzJn\/3mefTR5c2lncqJ8wmCvk5vBVd5rmAqzxQCRYdpmcYqqTuEQEkhWQanc+aExYn51GPWxMaMoC\/1+GKKR4fkW1bRV3UIg6s81zKMVdJ3VYhAMdM6ExMTqffT6C8L5LNMLt\/xxBynNecsptt6zq+7rAkESmpXz10AQT83wgUGwFSeKTIoOkwhUHS4ltpqkU4PixM+cr5\/7bJS83JpHIK+CyX\/MuDqzyytBpimEcr2Pbhm45ZUq8ixSv5u3rRYSAZF+mbMSbR79uwJTLvu\/JmcnKS+vj4aHh6mtra2yGYV5fQHDx4NduqYDx+y5vNCPmnGkvYQnCRpvmULXOW5gqk8U2RQdJgWNVbp3E3JBErUtuMtW7ZQV1dX6v3b7\/Ix00fd3d2JdY2o2b9\/f+Jx+kU43c6c1Pt6kyjnIeindulMBcA1E7bESmAqzxQCRYdpEWOVzp28ZbUUGRQWJzt37qSRkREyJ8a6Co0oQLZgiQNoXkrI35cpgxJeEFulzInxBYK+zmMNrvJcwVSeKQSKDlMIFAWu0kfdu5yrwmWuu+466unpCd6kXBaBEj6A7fae86nnoqUK1Is1iaCvwx9c5bmCqTxTCBQdphAoClwlBYp5C3JnZycNDg5SU1NTZIs5Y8OfFStWlGYNCouTK3c9SV+dfCVo24bVLbTt189TIF68SQR9HR+AqzxXMJVnCoGiwxQCRYcraUzxzMzMRIoUXhh711130TXXXEPT09NOAsW+7b1796pQ2PPEa\/Qn\/\/ZSYLvllBNpzydbVa5TBqPMvbW1uvdXFGNwlScPpvJM2SK45ufa0dGxwMjU1FR+wyWyUIo1KMwjzyLZMM+k3TmcZbn44otp1apVVJZdPPa6k3p6p07WfoxfpVnJJdcDV3muYCrPFBkUHabIoOhwFbdqFsDai275IkkHxI2NjQWiJfyphdPX3f4YPfj00eDSVVwUG2aKoC\/e5QOD4CrPFUzlmaKv6jCtxVil0\/J4q4VnUPLs1jG3Ze\/aMduHW1paUt+CXIYMip094VNid19+Ya37QM2vh6Cvgxxc5bmCqTxTCBQdphAoOlwXTO\/EZTPiLh8+qM1eJGu+4x074QxJ0QIlPLXz+Gc\/pES4XGYR9HX8Aa7yXMFUnikEig5TCBQdrvOsmp04\/I+cBRkdHY095bUGzSFNp\/M7dvhQNv5UdUtxlI8Q9HV6LrjKcwVTeaYQKDpMNccqnRanWy18iiepiSxWeD1JeC1J+m3JldByeqNmTxCc5Ppm2BIGU3m2YCrPFDFAh6nWWKXTWjerpRModgal6DcZM0Itpzdq9gTBye3BzFIKg2kWasl1wFSeKWKADlOtsUqntW5WSyFQyjatY6PTcHojZ08QnNwezCylMJhmoQaBIk8t3SL6ajoj3xIaY5VvG6TLFy5QXI6ml75pH3saTr9y55P0vx9+LmgGL4zls08a6YPgpONtcJXnCqbyTPEjRYepxlil01J3q4ULFPemFlNS2umNnj1BcNLrxxhM5dmCqTxTxAAdptJjlU4r\/axCoKTwknb60H2HaOi+Z4KrNtLOHRszgr7fQ+paGlxdSbmXA1N3Vj4lwdWHlltZ6bHK7aq6pSBQaihQkD15EzaCk85DDa7yXMFUniligA5TCBQdrqW2Kul0PvOEd+\/wp3\/tWdS\/dlmp712rcQj6OmTBVZ4rmMozhUDRYSo5Vum00N9q4RmUpEWyce\/U8b\/N7DUknW7eucOLYhvl1Ngo8gj62ftjUk1wlecKpvJMIVB0mEqOVTot9LcKgZLCTMrpDx48SuvueCy42qXtZ9DoJ3\/G31sVqYGgr+NIcJXnCqbyTCFQdJhKjVU6rctmtTCBsmvXLtq8eXNqqzdu3Jj60r9UIzkKSDl90z8eoC+MPxu0pBHeWIxf+jk6XcaqGEwzgkuoBqbyTCFQdJhKjVU6rctmtTCBYprbCOegYHHs\/M6JoJ\/tYU2rBa5phPy\/B1N\/Zi41wNWFkl8ZCBQ\/XpUoLeF0W6A0evYEv570HgsEfXm2YCrPFDFAh6nEWKXTsuxWC8+gZG96bWpKON0sjuUWQ6Bgm7FWz8VgKk8WTOWZQqDoMJUYq3Ralt1qIQLFntZZvnw59fb20sTERORdFP3CwLxOt7Mna85ZTLsvvzC7typSE0Ffx5HgKs8VTOWZQqDoMM07Vum0Kp\/VQgRKvibXtnZep+Psk4X+QtDX6cPgKs8VTOWZQqDoMM07Vum0Kp9VCJQUfnmdjukdCJR8j6h7bQym7qxcS4KpKym\/cuDqx8uldN6xyuUatS5TuEAx0z1VnOLB9E50d0Zw0nnMwVWeK5jKM0UGRYcpBIoO10irLFyuvvpq+uM\/\/mNqa2ur4ZXnXyqP0+3D2Rr1xYBRjkPQ1+nO4CrPFUzlmUKg6DDNM1bptCi\/1cIzKEm3wEfd79ixgwYHB6mpqSn\/3WawkMfpH\/+Lx+mrk68EV8XunbfgI+hn6IgOVcDVAZJnETD1BOZYHFwdQXkUyzNWeVympkVLL1CGhoZoZGSEmpubY8EcO3aMBgYGaM+ePUGZpNNnw1NKnZ2diQIoj9Obr\/py0J5Gf\/dO2HEITjrPOLjKcwVTeabIoOgwzTNW6bQov9VSCxQWJzMzM6kZFC7Hn\/7+fjICpLu7m7q6uuYRMkJm9erVwXfm75aWltjj9LM63Z7e+evf\/hm67MIz8nurIhYQ9HUcCa7yXMFUnikEig7TrGOVTmtkrBYuUJIWybJwGB0d9V6DYguWNEz8TqDx8fFYEZTV6fb2YkzvzPcCgn5ar8z2Pbhm45ZUC0zlmUKg6DDNOlbptEbGauEChW8jLrNhMh0+t+r7bh8tgWK2F2N6Z6H3EPR9erR7WXB1Z+VaEkxdSfmVA1c\/Xi6lIVBcKGUoEzWVkzRVE3cJtrN9+3ZKW1di6rtcg51uf\/bu3Zt6hzOvvkGdd00H5Va+bxH91WVLU+s0UoHp6WlqbW1tpFuuyb2CqzxmMJVnyhbBNT\/Xjo6OBUampqbyGy6RhcIzKEkZj6y7eFzWrpisDfsiaZdQFlVqn3\/Sv\/Ys6l+7rEQuL74p+PWk4wNwlecKpvJM2SK4ynPNMlbJt0LWYukFissunjCSyclJ6uvro+Hh4cj1K67ihO1mcfrQfYdo6L5ngmZh\/cnCDovgJPsQG2vgKs8VTOWZQqDoMM0yVum0RM5q4QIlvP7EvrW09SFxGDjzEidsXHbu2HazOB3rT5I7KIK+3ANsWwJXea5gKs8UAkWHaZaxSqclclYLFyh8KywoNm3aNG\/HDmdBNmzYQFu3bqVVq1Yl3rG9aydNgLhM\/+QVKOb8E7y9ONptCPpyDzAEig5LZKXAVZeAvHUIFHmmcxZZpKxfv37eFcbGxlLFCVcIH9RmL5I13\/X09NDy5cupt7eXwu\/9aW9vjz0Mztfp9vknmN6BQFF8ZBaYhvCTpw2m8kyRQdFh6jtW6bRC1mopMiiytyRrzdfpf7tvhv7H3z8VNAICBQJFtjcmW8NgKk8bTOWZQqDoMPUdq3RaIWu1cIFiZzjSpnJkb93Nmq\/Tsf4knSuCfjqjLCXANQs1iD55aukW0VfTGfmW8B2rfO0XUb5wgeJ7sFqtIfk43d5ejPUn8Z5CcNLpxeAqzxVM5Zkig6LD1Ges0mmBvNXCBQrfUtbdOvI4Flr0cbq9\/gTnn0Cg1KJ\/2tfAYCpPHEzlmUKg6DD1Gat0WiBvtXCBkvQuHr7dpAWs8jjkBArWn0Cg1KJ\/QqDoUoZA0eELrvJcIVDkmZbeoo\/TzfoTvqnZmy8p\/b0V1UAEJx3y4CrPFUzlmSKDosPUZ6zSaYG81cIzKPK3JGvRx+kX3PAQ8ToUvCAw2QcI+rJ91FgDV3muYCrPFAJFh6nPWKXTAnmrhQgUe2Fs3Nkk5lbrZYoHC2TdOyeCvjsrn5Lg6kPLrSyYunHyLQWuvsTSy0OgpDOqXAlXp+MFge6uR3ByZ+VTElx9aLmVBVM3Tr6lwNWXWHp517Eq3VJ5ShSSQQnffvh9PEnv56k1Olen4wWB7p5BcHJn5VMSXH1ouZUFUzdOvqXA1ZdYennXsSrdUnlKlEKgRL0fx0wDdXd3U1dXV2HEXJ2OA9rcXYTg5M7KpyS4+tByKwumbpx8S4GrL7H08q5jVbql8pQoXKAkHdTG7+fZsWMHDQ4OUlNTUyHUXJ1uFsjigLZ0NyE4pTPKUgJcs1BLrgOm8kzZIrjKc3Udq+SvrGex9AKFsysjIyPU3NysRyHBsqvTzRuMey5aSrf3nF9IW+vloghOOp4CV3muYCrPFAJFh6nrWKVzdR2rhQuUpPUmZThh1sXpeIOxX+dE0Pfj5VoaXF1JuZcDU3dWPiXB1YeWW1mXscrNUnlKFS5QGAVP5WzatIlGR0epra0toDM5OUkbNmygrVu3UpEvEXRxOhbI+nVoBCc\/Xq6lwdWVlHs5MHVn5VMSXH1ouZV1GavcLJWnVCkEihEp69evn0dmbGysUHHCjXFx+ro7HiPOovAHJ8imd24Ep3RGWUqAaxZqyXXAVJ4pWwRXea4uY5X8VXUtlkag6N5mdusuTscOHj++CE5+vFxLg6srKfdyYOrOyqckuPrQcivrMla5WSpPKQiUFF+kOR0nyPp3ZgQnf2YuNcDVhZJfGTD14+VaGlxdSbmXSxur3C2VpyQESk6BYi+Q7V97FvWvXVYe75a0JQhOOo4BV3muYCrPlC2CqzxXCBR5pqW3mOb0Hd94ni7f8URwH7s\/fSGtOXdx6e+p6AYiOOl4AFzluYKpPFMIFB2maWOVzlV1rTZcBsVsa96zZ09AduPGjdTf3x9LOc3p9g6exz\/7oeBNxvgkE0DQ1+kh4CrPFUzlmUKg6DBNG6t0rqprteEECh\/8xh8WJS7H6ac5HQtk\/Tsogr4\/M5ca4OpCya8MmPrxci0Nrq6k3MuljVXulspTshQCxZx5MjMzs4BMe3u76kmytmCJckua03HEvX9nRnDyZ+ZSA1xdKPmVAVM\/Xq6lwdWVlHu5tLHK3VJ5ShYuUMyUS0tLS+JUiwaypPcAmeslOR07eLJ5BcEpG7e0WuCaRsj\/ezD1Z+ZSA1xdKPmVgUDx4+VU2kUkOBnyLMSZk+3bt1NnZ2fiywjZ6fZn7969c3\/uf\/Z1+oO7nw\/+3vjBxfQHP48Fsi5umJ6eptbWVpeiKONBAFw9YDkWBVNHUJ7FwNUTWETxjo6OBf86NTWV33CJLJQmg9LT01PIqbEsVHhqKe6NyUmq1N7BgwWy7r0av57cWfmUBFcfWm5lwdSNk28pcPUlll4eGZR0RplK8Lt4inprMa9\/6evro+Hh4bn3ANk3keR0vIMnk7txBkI2bKm1EPRTEXkXAFNvZE4VwNUJk1chCBQvXG6FzRTPxMREZAXtRbJp4ijJ6djB4+bjcCkEp2zc0mqBaxoh\/+\/B1J+ZSw1wdaHkVwYCxY9XKUvbu3ZcFugmOd3s4OGzT3iKBx83AghObpx8S4GrL7H08mCazihLCXDNQi25DgSKPNOaWwwf1OaySDZq4RF28GR3HYJTdnZJNcFVniuYyjNli+AqzxUCRZ7pnMVdu3bR5s2bg7\/Hxsbo8OHDND4+nrjDRrE5c6bjnG4LFLyDx88TCE5+vFxLg6srKfdyYOrOyqckuPrQcisLgeLGybuU2UnDi1WvuOKK4DwUXnsyMDBARZyPYt9AnNPtlwTe3nM+9Vy01Pu+G7UCgpOO58FVniuYyjNFBkWHKQSKAlf7HJTly5dTb29vIFBWrVpFaQtYFZqzwGSc07GDJzt9BP3s7JJqgqs8VzCVZwqBosMUAkWBa70KlCt2PkljDz8XEJm9+RIFMtU1iaCv41twlecKpvJMIVB0mEKg6HAlXn\/C603sKR6TTenu7qauri6lK6ebjXM6thins4srgaCfnR0yKDrs0FfBtbYE5K8GgSLPdM4iT+esX79+3hW2bNlSqDjhxsQ5HS8JzN4ZIFCys4NA0WEHgQKutSUgfzUIFHmmpbcY5XR7B8\/FbUvonv92Qenvo0wNhEDR8Qa4ynMFU3mmbBFc5blCoMgzLb3FKKfbO3iwxdjfhQhO\/sxcaoCrCyW\/MmDqx8u1NLi6knIvB4Hizsq7pH0OiqnM56Hwbp4iP2kCBVuM\/b2D4OTPzKUGuLpQ8isDpn68XEuDqysp93IQKO6svEqyONm5cyeNjIxQc3NzUNfs7injIlk7g7L70xfSmnMXe91voxdGcNLpAeAqzxVM5ZmyRXCV5wqBIs90ToiYs0\/sS5T1HJTLdzxBO77xfNBUfgcPv4sHH3cCCE7urHxKgqsPLbeyYOrGybcUuPoSSy8PgZLOyLuEfQ5KeDqnrAIFW4y93TyvAoJTPn5xtcFVniuYyjNFBkWHKQSKDtfgxNhNmzbR6OgotbW1BVcp8xQP3mKcryMg6OfjB4Giwy\/KKvqqDmtwlecKgSLPdE6ITExMpFrn9\/Pcc889qeUkC4SdjrcY56eL4JSfIQZTHYZhq+irOpzBVZ4rBIo809JbTBIo2GKczX0ITtm4pdUC1zRC\/t+DqT8zlxrg6kLJrwwEih+vSpQOOx1noOR3K4JTfobIoOgwRAYFXGtDQP4qECjyTOcsRp2DUsaj7nn3Du\/i4Q928GTrEBAo2bil1QLXNEL+34OpPzOXGuDqQsmvDASKHy\/n0vV0DsrQfYdo6L5ngnvDGSjOLp5XEMEpG7e0WuCaRsj\/ezD1Z+ZSA1xdKPmVgUDx4+VUut62GdtnoMzefInTPaLQfAIITjo9AlzluYKpPFO2CK7yXCFQ5JnW3UFtOAMlfydAcMrPMMoCuMpzBVN5phAoOkwhUHS4Ut4pHpOFMVuVOzs7aXBwkJqamiJbbK93SSsbdro5A2XNOYtp9+UXKhGptlkEfR3\/gqs8VzCVZwqBosMUAkWHa2A16yLZY8eO0cDAAK1evZq6urrI\/N3S0kJ8fH74Y59OywKG68aV5bphpzdf9eXAJARK9s6AoJ+dXVJNcJXnCqbyTCFQdJhCoOhwFbfKYmd8fDwyixL+LqlsWKDYh7ThDJTsbkPQz84OAkWHXZxV9FUd3uAqzxUCRZ6pisUk0RGVQTHZl6jG2E63z0C5ved86rloqUr7q24UwUnHw+AqzxVM5Zkig6LDFAJFh6uoVZd3+ExOTtKGDRtoZmaGxsbGKPySQrtB7HTzeeO08+i1NX3Bn3912VJa+T68xTiL86anp6m1tTVLVdRJIACu8t0DTOWZskVwzc+1o6NjgZGpqan8hktk4YTjx48fL1F7cjXFrD9hI3GLZMMLcoeGhoJrRq1X4X+3VSnOQMnlnrnK+FUqwzFsBVzluYKpPFNkUHSYIoOiw1XEqos4CS+o5QtzNqWvr4+Gh4fn3qRsN8h2+v\/8pwM08vVng69ximx2tyHoZ2eXVBNc5bmCqTxTCBQdphAoOlxzW03buWMukFeg4AyU3K4KDCDoy3BEBkWHo20VfVWHMbjKc4VAkWcqYpGnaXg9SdLZJ+ZCUVM8SXVtp0OgiLgLAkUG4wIrCPryYMFUnil+pOgwhUDR4ZrLaviQNmOsvb2dRkZGgsPa+KyTnp6eucWwLGi2b98eFPU5qA1noORy1VxlBH0Zjsig6HBEBgVc9QnIXwECRZ5p6S0ap9tnoPzWz7+X\/rzr\/aVve1kbCIGi4xlwlecKpvJMkUHRYQqBosO11FajBAoOacvnMgT9fPziaoOrPFcwlWcKgaLDFAJFh2uprRqn24e0QaDkcxmCfj5+ECg6\/KKsoq\/qsAZXea4QKPJMS2\/ROH3HN56ny3c8EbR396cvpDXnLi5928vaQAQnHc+AqzxXMJVnigyKDlMIFB2upbYalUGBQMnnMgT9fPyQQdHhhwwKuNaOgPyVIFDkmZbeonE6Z084i8IfHNKWz20QKPn4QaDo8INAAdfaEZC\/EgSKPNPSWzRON2egcINnb76k9O0ucwMhUHS8A67yXMFUnimmeHSYQqDocC211bBAObN5UZBBwSc7AQT97OySaoKrPFcwlWcKgaLDFAJFh2uprRqnX3DDQ8Rnoaw5ZzHtvvzCUre57I1D0NfxELjKcwVTeaYQKDpMIVB0uJbaqnE6TpGVcxOCvhxL2xK4ynMFU3mmECg6TCFQdLiW2io7\/SuP\/CdxBoU\/OAMlv7sQ9PMzjLIArvJcwVSeKQSKDlMIFB2upbbKTv\/bf32U1t3xGASKkKcQ9IVAhsyAqzxXMJVnCoGiwxQCRYdrqa2GMyi395xPPRctLXWby944BH0dD4GrPFcwlWcKgaLDFAJFh2uprbLTb9w1jlNkBb2EoC8I0zIFrvJcwVSeKQSKDlMIFB2upbbKTt\/4F1+iofueCdqJU2TzuwtBPz\/DKAvgKs8VTOWZQqDoMIVA0eFaaqvs9F++8V6cIivoJQR9QZjIoOjA\/LFV9FUdvOAqzxUCRZ5p6S2y0z9w9T\/Rg08fJRzSJuMuBCcZjmEr4CrPFUzlmSKDosMUAkWHa6mtQqDIuwdBX54pgj6Y6hDQsYoYIM8VAkWeaektstNP+b2\/wymygp5CcBKEiSkeHZiY4gFXVQLyxiFQ5JmW3iI7\/ejHR4J24ph7GXdBoMhwxBSPDkfbKvqqDmNwlecKgSLPVMTi7Ows9fb20sTERGCvs7OTBgcHqampKdL+vn37aP369cF37e3tNDIyQs3NzZFll33gg\/TqLw0F3\/H5J3wOCj75CCA45eMXVxtc5bmCqTxTtgiu8lwhUOSZ5rZ47NgxGhgYoNWrV1NXVxeZv1taWqi\/v3+B\/cnJSdqwYQNt3bqVVq1aRbt27aLx8fFYQWNnUHDMfW53BQYQnGQ4IoOiwxEZFHDVJyB\/BQgUeaYqFpNEB3936NChSPES1ZgzP\/jL9NqavuArnCIr4y4IFBmOECg6HCFQwFWfgPwVIFDkmapYjBMo4WyLy8VbP\/rb9IMVvxcUxSFtLsTSy0CgpDPKUgJcs1BLrgOm8kzZIrjKc4VAkWcqbtGsR+nu7g6mfOyPEShr166lO++8M1izkrYGpeVX\/ohef\/+6wMzJDw7TA7vuEG9zoxmcnp6m1tbWRrtt9fsFV3nEYCrPlC2Ca36uHR0dC4xMTU3lN1wiCyccP378eInak6spRoCwkahFsub7I0eOzC2MHRoaopmZmdg1KLZAefyzHwoOa8MnHwH8esrHL642uMpzBVN5psig6DBFBkWHq4jVNHHCF4ma4uFFs319fTQ8PExtbW0L2rL016+nH565Jvj32ZsvEWlroxtB0NfpAeAqzxVM5ZlCoOgwhUDR4ZrbatrOHfsCnDFZtmzZ3PQPC5SbbrqJtm3bFrnV+Izf+Ut647TzcMx9bi+9ZQBBXxCmZQpc5bmCqTxTCBQdphAoOlxzW02bprEvwGegcHlz9gn\/N3+itiTzv0Og5HbPAgMI+vJMEfTBVIeAjlXEAHmuECjyTHNbDB\/SZgyaxa98WBufk9LT0xOce8If+6C2tEPdTvvDf6AfvfM0nCKb21PIoAgijDSFoC9PGEzlmUJM6zCFQNHhWmqrzVd9OWgfjrmXcxOCvhxL2xK4ynMFU3mmECg6TCFQdLiW2qoRKJ9a8z4avmx5qdtaL41D0NfxFLjKcwVTeaYQKDpMIVB0uJbaqhEoOOZezk0I+nIskUHRYWmsoq\/q8AVXea4QKPJMS28RAkXeRQhO8kzxqxRMdQjoWEUMkOcKgSLPtPQWjUDBe3jkXIXgJMcSGRQdlsiggKsuAXnrECjyTEtv0QgUvIdHzlUQKHIsIVB0WEKggKsuASXqtcgAAAvZSURBVHnrECjyTEtv0QgUHHMv5yoIFDmWECg6LCFQwFWXgLx1CBR5pqW3CIEi7yIIFHmmbBFc5bmCqTxT9FUdphAoOlxLbdUIFLyHR85NCPpyLJFB0WGJDAq46hKQtw6BIs+09BZZoPAbjHmKBx8ZAhAoMhzDVsBVniuYyjNFBkWHKQSKDtdSW4VAkXcPgr48UwR9MNUhoGMVMUCeKwSKPNPSW2SBgmPuZd2E4CTLE9MROjwh+sBVj4C8ZQgUeaaltwiBIu8iCBR5phhMwVSHgI5VxAB5rhAo8kxLb5EFSs9FS4kPasNHhgCCkwzHsBVwlecKpvJMIaZ1mEKg6HAttVUWKHgPj6yLEPRleWKKR4cnBlJw1SMgbxkCRZ5p6S1CoMi7CAJFnikGUzDVIaBjFTFAnisEijzT0ltkgYL38Mi6CcFJlicyKDo8IfrAVY+AvGUIFHmmpbdYRacXDR0CRccD4CrPFUzlmUL46TCt4lh1wvHjx4\/r4KqG1So6vWjPIOjreABc5bmCqTxTCBQdplUcqyBQUvpKFZ2u83i4W0XQd2flUxJcfWi5lQVTN06+pcDVl1h6+SqOVZUQKLOzs9Tb20sTExOBFzs7O2lwcJCampoSvTo5OUl9fX00PDxMbW1tkWWr6PT0rq5bAsFJhy+4ynMFU3mmyKDoMK3iWFX3AuXYsWM0MDBAq1evpq6uLjJ\/t7S0UH9\/f2xPMOX2799Po6OjECg6zwxEH7jWkID8paoY9OUp+VsEV39maTWqyLTuBUqU03bt2kXj4+OJWZR9+\/bR0NBQUB0ZlLSuL\/t9FR8kWULZrIFrNm5JtcBUnilbBFd5rlVk2pAChaeErrvuOurp6QlECgSK\/MOCoF9bpgj6OryrGPR1SPlZBVc\/Xi6lq8i0cgLFrEfp7u4OpnziMiz87ytWrHBag+LSOVAGBEAABEAABIokMDU1VeTlxa9dKYFi1pUwpbhFsrww9q677qJrrrmGpqenUwWKOHEYBAEQAAEQAAEQSCVQGYHiIk6YBk\/pXHzxxbRq1Spy2cWTShAFQAAEQAAEQAAExAlUQqC47twJb0e2aY6NjQWiBR8QAAEQAAEQAIHiCVRCoHBWZGZmxunsExs5MijFd0C0AARAAARAAASiCNS9QInLirS3t9PIyEhwWBufk8I7dsIZEggUPBQgAAIgAAIgUE4CdS9QyokVrQIBEAABEAABEMhDAAIlDz3UBQEQAAEQAAEQUCEAgRKDlde1bN++PfgWC2j9+x5Pn23YsCFYG5T2biSbNb+iIOnVA\/4tqU4NH6bmrsOvgqgODbk78eEanlJGbIj2gw9Tuyye\/+z92jzrUcsZslsttiYESgR\/cww+r2E5cOBAsDWZ\/7u5ublYb9XJ1e1Bcd26dfPelRS+hfBrCfjvnTt3gncIlA9Tuyrz3Lx5M23ZsiX24MI66VYqzfThGt4tiDVs0S7xYWoEH783jdcI4vnP1s0N8z179lTqBzUESkR\/MO\/o4Yemiqo02yPgXiscuFnw7dixw2mXFYJ+\/C9S+83bLkw5+F999dV09OhRSjpZ2d2z1Svp01e57E033UTbtm3Dj5WEruDL1O7XeP79nzGTgVq5ciUdOXIkeEluVY7MgEBJ+KVqvx3ZvC3Zv\/s0Xg07A8VZp\/DfSUQQoKLpZGHKQvuiiy6iL37xi3Nv+2683ph8xz5cXV5CCr604HlPev6jMihpL3oF4\/kEnn322eAfeMdqb28vBEqVO0hUxoQD\/bJly5Aid3R8+Ne9zy\/PrGfaODatbov5MjWvdLjqqquCF2NCYMcLPzu7l9RXWaAcOnQoMIT1afGPkm9ftacnNm7cGAyw+PgTCIs9fwvlq4EMSkwGxV5oBIHi13F9A5SxzgPArbfeikWyEbh9mHLAv\/HGG+mTn\/wktba2Jq4B8vNs9Ur7cDXreczCWK67adMm9NdQt\/BhaqYntm7dGkxL+GRbq9cb890RBEo+fnVRO7zrAbsg\/N3mkzaHOHHj68OUyz7wwAPBL1H032S+PlzDUzxgG5+VsjcWJIkOMHV7\/l1KQaC4UKpAGTtjgkWy\/g4Np8nTFnRi5X46Yx+m9rZt2zLS5ws5+3AN92PEhuh+68MUAiX92XctAYHiSqrOy2GbcT4H+mwzRJrcjbUPU9sifuUn8\/XhGh4AMB0RzdaHadQUD6bN3GJCuBQESjZudVkLB7Xlc1vSQU1msSFPQcT92scBWNG\/9uMOv7OZQqD49V3XvspW7YPacKhYPGcfpiz01q9fHxgDU7++a5eGQMnODjVBAARAAARAAARAwJkAdvE4o0JBEAABEAABEACBWhGAQKkVaVwHBEAABEAABEDAmQAEijMqFAQBEAABEAABEKgVAQiUWpHGdUAABEAABEAABJwJQKA4o0JBEAABEAABEACBWhGAQKkVaVwHBBwIPP3007RkyRLnt+Xy1sJXXnmFzjnnHAfr2YqYreDt7e00MjLi3LZ6efGjub9abXGt9fWyeR21QKB4AhAoxfsALQCBgIDvwV+1OPcgj8jIU7eWXYIFA39q+ZK6emFTSz\/gWiAQJgCBgj4BAiUhUEaB4tsmG2W9DMIQKCV5ANAMEAgRgEBBlwCBGhKwTyLly5ppkwMHDsydpsn\/bk7SDZ+0a6YhTj31VOrt7aWJiYmg9eY9O\/ar6237zc3NsXdpX8Oe5jBv7zUVt2zZQl1dXQvsxJUzAmXdunV0\/fXXB\/XC0yj2KaLh6zCrq6++mj7ykY8E9Q2rl19+mcyJulznc5\/7HO3evZuGh4epra0tMBN3T1EQwgKF\/\/7e974X\/G\/Pnj3z+EbVD79PhstE\/Vs9ircaPhq4FAgsIACBgk4BAjUiEPVeHHtwDGcr4l6kxs0dHBwM3lTMIoWnJvhV9Ub8dHd3zwmJpBcxmvYYe01NTcHAeuutt9Lo6Ggw2KdlUMLl7XersIhiIbFy5cqgvcb+zp07g7UsLDT6+vrmCQvbnhFhZ5555lz98D2av1988cWgza2trTQwMBAIITNlk\/a+pyiBsn37djKCLIqr3WUgUGr0AOEyDUcAAqXhXI4bLopA2kCXJgbCv8zDAiXqrdFJLwuMmoIJl09qU9qLCMMvguP2p0372N8bgRIWXOPj43OChW3aAoT\/vummm2jbtm3zFvMmTeNECZSZmZkF1+ByUYuEIVCKeqJw3aoTgECpuodxf6UiYE+HhHfFxImB8DRIZ2dnZAYlPNVi33jU9Ezc9ewXDyYJlLRFulFiJOrfwtNe4WkskyHi+4kSGrZNzsqYF8+FHW+mwcL\/HiVQuIy9aDZJWEGglOoRQ2MqRAACpULOxK3UDwF7ULbXodi\/0o3gCK8LMRmEcAaF64Z\/+ScRiRMfSdNOtr28AoVtmbUkRkBFZVB8BMqjjz5KZgopad2NfR8QKPXz3KCljUUAAqWx\/I27LRkBe5A3GQKeRuD1GryWYvXq1fMWptoiJCxQktabRN12LaZ4wmtM7GuymEiarjFTPLZAicpW2FM8nEHZtGnT3BoaF3drTPGkicW0qS6XdqMMCFSdAARK1T2M+ysNgag1KHYWw140GrXY02RUzBQP35gtYox9XjBrpiei1oEYIFKLZO2Mhb0uZcWKFQsWwYYFil3XtJXbxwteowSK6yJZtmEW5qat\/cm7SNZMwZmdV+Y+7MXB4U4IgVKaxxINKTEBCJQSOwdNqx4BM3jxVAx\/7Okbe4swT3l87GMfm7eVmIXJpZdeStdee+1chiAsWkxWxWw\/5muYgTOOZtKWXNeFu5s3b54zHzVdY7b\/hgfm8LW3bt0arDPhhbHm\/u0MCl8kzDC8zTi81ZrrxG2RNlkr\/n8j6qK2Gdv1o8SFvf6H\/XTBBRfQ448\/HoiksJA09xDOLlWvt+OOQCAfAQiUfPxQGwRAoGACLBiidu64NstlDYqrLddyyKC4kkK5RiYAgdLI3se9g0CdEQivszHZEvvcE99bgkDxJYbyIFAbAhAoteGMq4AACAgRCJ+uG7d92PVy4Zf33X333UFVrXfz4GWBrp5BuUYn8P8BIlW2gOv\/MQgAAAAASUVORK5CYII=","height":333,"width":552}}
%---
%[output:68368d88]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"   440"}}
%---
%[output:693c913f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.030000000000000"}}
%---
%[output:5da90709]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_JH","value":"   0.075000000000000"}}
%---
%[output:3ed5876a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber","value":"     6.215564738292011e-09"}}
%---
%[output:73b64db6]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rsnubber","value":"     3.217728531855956e+03"}}
%---
