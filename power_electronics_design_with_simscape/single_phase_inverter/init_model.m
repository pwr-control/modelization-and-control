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
fPWM_AFE = 20e3; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = 20e3; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = 20e3; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

double_sampling = 0;

if double_sampling 
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

z_dab = tf('z',ts_dab);
z_afe = tf('z',ts_afe);
z_inv = tf('z',ts_inv);

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
V2rms_load_trafo = V1rms_load_trafo/m12_load_trafo %[output:2a12e11d]
I2rms_load_trafo = I1rms_load_trafo*m12_load_trafo %[output:923f0700]
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
Vac_FS = V1rms_load_trafo*sqrt(2) %[output:4ca1a094]
Iac_FS = I1rms_load_trafo*sqrt(2) %[output:9bdb6f58]
%[text] ### Single phase inverter control
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
%[text] #### DQ PI
kp_inv = 0.5;
ki_inv = 45;
%[text] #### Phase shift filter for Q component derivation at 50Hz and 80Hz
freq = 400;
temp = 2*pi*freq*ts_inv;
% a_phshift = (sin(temp) - cos(temp)) / (sin(temp) + cos(temp))
a_phshift = -0.951005;

% flt_dq = 2/(s/(2*pi*50)+1)^2;
% flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d, options); grid on
% [num50, den50]=tfdata(flt_dq_d,'v')
% 
% flt_dq_80Hz = 2/(s/(2*pi*80)+1)^2;
% flt_dq_80Hz_d = c2d(flt_dq_80Hz,ts_inv);
% figure; bode(flt_dq_80Hz_d, options); grid on
% [num80, den80]=tfdata(flt_dq_80Hz_d,'v')

% flt_dq = (s/(2*pi*50)-1)/(s/(2*pi*50)+1);
% flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d, options); grid on
% [num50, den50]=tfdata(flt_dq_d,'v')
% 
% flt_dq_80Hz = (s/(2*pi*80)-1)/(s/(2*pi*80)+1);
% flt_dq_80Hz_d = c2d(flt_dq_80Hz,ts_inv);
% figure; bode(flt_dq_80Hz_d, options); grid on
% [num80, den80]=tfdata(flt_dq_80Hz_d,'v')
% 
% flt_dq_d = (a_phshift + z_inv^-1)/(1 + a_phshift*z_inv^-1);
% figure; bode(flt_dq_d, options); grid on

%[text] #### Resonant PI
rpi_enable = 1;

omega_set = 2*pi*freq;
kp_rpi = 2;
ki_rpi = 45;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta*omega_set] %[output:4f337f7f]
Aresd_nom = eye(2) + Ares_nom*ts_inv %[output:37b63c7b]
a11d = 1 %[output:0f6f0f96]
a12d = ts_inv %[output:9dfdaf92]
a21d = -omega_set^2*ts_inv %[output:0b4e6f02]
a22d = 1 -2*delta*omega_set*ts_inv %[output:81251e8a]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:9a6d4274]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:686ca032]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:3e05b6e0]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:4e5d2adb]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:2a8ed490]
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
%[text] #### 
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:0de50622]

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
figure;  %[output:42eb2e28]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:42eb2e28]
xlabel('state of charge [p.u.]'); %[output:42eb2e28]
ylabel('open circuit voltage [V]'); %[output:42eb2e28]
title('open circuit voltage(state of charge)'); %[output:42eb2e28]
grid on %[output:42eb2e28]

%[text] ## IGBT and snubber data
%[text] ### HeatSink
weigth = 0.50; % kg
cp_al = 880; % J/K/kg
heat_capacity = cp_al*weigth % J/K %[output:3da50f8a]
thermal_conducibility = 204; % W/(m K)
Rth_mosfet_HA = 30/1000 % K/W %[output:9479f5e4]
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
Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH % K/W %[output:2957f2e8]
Lstray_module = 12e-9;

Irr = 475;
Csnubber = Irr^2*Lstray_module/Vdc_nom^2 %[output:1f6650e6]
Rsnubber = 1/(Csnubber*fPWM_INV)/5 %[output:42540868]
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
%   data: {"layout":"onright","rightPanelPercent":52.2}
%---
%[output:2a12e11d]
%   data: {"dataType":"textualVariable","outputData":{"name":"V2rms_load_trafo","value":"   6.600000000000000"}}
%---
%[output:923f0700]
%   data: {"dataType":"textualVariable","outputData":{"name":"I2rms_load_trafo","value":"       30000"}}
%---
%[output:4ca1a094]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     4.666904755831214e+02"}}
%---
%[output:9bdb6f58]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     8.485281374238571e+02"}}
%---
%[output:4f337f7f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"6","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000001000000000"],["-6.316546816697190","-0.000125663706144"]]}}
%---
%[output:37b63c7b]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"2","name":"Aresd_nom","rows":2,"type":"double","value":[["0.010000000000000","0.000000500000000"],["-3.158273408348595","0.009937168146928"]]}}
%---
%[output:0f6f0f96]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:9dfdaf92]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     5.000000000000000e-05"}}
%---
%[output:0b4e6f02]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"    -3.158273408348595e+02"}}
%---
%[output:81251e8a]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.993716814692820"}}
%---
%[output:9a6d4274]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.076483869223994"],["18.982392004991411"]]}}
%---
%[output:686ca032]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:3e05b6e0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:4e5d2adb]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000050000000000"],["-4.934802200544680","0.999214601836603"]]}}
%---
%[output:2a8ed490]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.077754418176347"],["13.583043056999228"]]}}
%---
%[output:0de50622]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.018721899663332"],["0.977712707506129"]]}}
%---
%[output:42eb2e28]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10XsV55x9a2qCGEFtAaoTimBg5kJLKmJIa8eGyXupNz8rkJO2R5D2bVFFTt4FwtuAjyXHCKYFgSWuzS\/loHI6iOruV7JMNJPbShlKH0LDCDTGgNikBgbCNEE4MggDFJCHxnueaEaOr+zEz7zzve+\/of8\/hGOl95rkzv2femb\/m87ijR48eJTwgAAIgAAIgAAIgUCICx0HAlChayCoIgAAIgAAIgEBEAAIGFQEEQAAEQAAEQKB0BCBgShcyZBgEQAAEQAAEQAACBnUABEAABEAABECgdAQgYEoXMmQYBEAABEAABEAAAgZ1AARAAARAAARAoHQEIGBKFzJkGARAAARAAARAAAIGdQAEPBGYnp6mrq6uyNvg4CDV19d78jzXzfj4OHV2dtJ5551HfX19VFdXJ\/Yu3XE1y2hSIMXh05\/+NLW1tZkkKbRNf38\/bdu2Lcrj+vXrqaenxyq\/O3fupI0bN9LmzZsLyYPLt3fvXvHvhxU0GJeWAARMaUOHjBeNQDU79zQBwx3EqlWraOXKlSJ40soo\/d60wrh0iNyB3n\/\/\/VbiwCWNbQD4HevWrZtJFqKACU1w2sYY9n4JQMD45QlvIFATAkeOHKHe3l7avXs3DQ8PV03A8MhPNd6bBFV1hq2trcZiRI1Q2IgDlzQulUAJGJu8xd9T9BEYVU8PHjyIURiXSoI0swhAwKBCFJKAPpTOGdSHxPXRh+XLl9P1118flYE7Mn06RY0WjI2Nzflc93H55ZfTn\/zJn0Q2DQ0NNDQ0RE1NTYlcdKEQt4+PTvDnakpp9erVdNNNN1Fzc3PUcOsdf54fnopS7923b1+UP37UFNK1115Ln\/\/85yPxop44C\/69YqoLHNVp6vaqE1S+9A5VL+Ott95KAwMDie+dnJyM8jc1NTWTJz2GOkdmftttt9GXv\/xlUuVj\/nHWip2amkvqrFVc9feq8sbLpWLd2Ng4I8Li\/Hbt2hVNyahHrx9xf3nCMV4fs3xl1cOskRqdCedZ5T0uiuLfL52t8nH11VfTnj17iL8\/KnZ6mfl36h16bPO4JNXDQjZCyFThCUDAFD5E8yuD8U5LL71qhJM6qXjHw35YPCjxEv88qYPN6vz5s7S8qc7m5JNPnrUGRgkYPQ8sFJIEhy5i4n58CZikv\/DjnUm8Y0vjyr9PEzDd3d105ZVXzmGfJRj4s1NPPZUOHz4cCbQkUcHv1DvaeN7T6oV678MPP5woRu68886ZdSd6fdM76LiAiftSn6eJmKw6y2kOHDiQKpT0PMXFS1xkKvFw8cUX03e+851ZjUeSCEn6fsUFCNsk5ZF\/r96T51vnUvRRovnV4pa7tBAw5Y5fcLlXDbT+F6hq\/Lmw+ugD\/5WtGka9g9AbW\/0vT73DY5GgRgiUD\/Xu+F\/6CnLS53pjfNlll6UKGP0vVFs\/eQKGR534yZvKyRoh4lGhF154YQ4TfdSAOS1btmxWGU2mkOLTW4q9iiePtsTjznnh9SBJI0PMcu3atVF59REbk4XNJtNBcZv4z4qJEluc\/7x3q7qXVB71Oxa6XOa0KSSdo6pP8Zjee++9kRBKGlFJ8xsfhVOjTrqPrHerERpV\/\/O4+JgqC67hQ4GcCEDAOGFDIikCaX+dqQ6AG+4VK1Yk7sDRbfbv35\/4VzXnW\/fBf\/WrHUN5i3Dz\/nJMEwh6g87vt\/XjS8Do72Yxwo\/eYaZ1LHoH\/slPftJYwCSNWCW9l\/MRnyJLG+FgW+6IVT50tknvi0+lZQmYtKmzeJqs0ZQk8Rsvm5qejAshJdrShEZe\/dTjq\/tIi2t8NEexUgImbepQ32Gn12X1vdSn71Q7oXNJmraUak\/gN2wCEDBhx7d0pau2gNG3Ied1ELbCg+Enbas29aN3zvHOjn3r26hNRmDYRl\/4yj\/zlt34CFS8A7UVMHGO8VGauHDyJWBUZU8TTrwzK0nAxKei8kZgyiBgkkb8VFzj9S9tBEb3kfbdgIApXRMbVIYhYIIKZ\/kL4zqFFJ\/qUGsK0v6aTRryzxMwWVM\/+qgAR4H\/Sk0TMKZ+eGg+Li7U1FpcwLBIMFkcGe\/c9RGK+DQcd\/h5U0g8OpQnAOI+XKeQ9NqdNqoR\/wbExUh8NEKVWR+JU+VRdSeeJmkKKe+bJz2FpMSuGrlKEzBJI1eKUXwEJm3RdXz6KmsKKYkLppDyags+NyUAAWNKCnZVISC9iDdLAOQJmKy8Ja0PSRMweX54uF2tZ4lDNxEwnCZpF5LyFd9Joh8AZ7OIV00l6Gn4vR\/5yEei0aGkhzkllc90ES\/7VKIuLpzSFrjqaXQbfufNN99MN9xww5wFx5wmLmD4d2kLglVZ8wRz0vRK3giYzjGtjFniQxcMV111VWrdyvLBeUha3Gu6iFfnkjcCWZWGBi8JggAETBBhDK8Qptuo9S3QeduokxYG20whMeWs6Ym8RbL6ybxZfvg98S23n\/vc5+jRRx+dWbSaNAKjd25pC5F13\/G1OUkCR+\/I9bT8\/0rAJL33jjvumHWiLB+up6+3cdlGrQsRvUNN2mKftn07zlVfk8M+mduWLVtow4YNEQ59JE3tJkvblp13fkvWNmp+l+nIRNraFR6FSxIHaaNOzChpC3vSKE6a+OXfx0\/+TVtLpHyYjBSG16KhRBIEIGAkqMKnKIG8HR+iL4fzigkkTVXFd5qlncOjv9zlILuKMz9PHeiCUwm1pJ1JeXhwkF0eIXxuQyBIAcMNG59FwYdsJTWEWQec2cCDbW0IQMDUhruvt2ZNoWVNfSW93+UqAV\/lmG9+kqaQmEHe4Y9JojOUu6vmWx0oWnmDEzB5i\/vU5y0tLdFlZ+pn\/hLaXpxWtGDOl\/xAwJQ\/0vE\/IrhEtuKF0+BunerWhfjUro144ZxCcFY3XqG\/LTgBw\/O9\/CXhJ20EJh5U\/stidHS0qrf6hl6xUD4QAAEQAAEQkCQQlIDhv+quu+466ujoiEQMBIxk1YFvEAABEAABEKgdgaAEDI+k8MMnQmatgdFxq6Hs9vb2aEoJDwiAAAiAAAiAQPEJBCNgeC58+\/bttGnTJuKL+kwEjFr\/wmHSbzGOh+29731v8SOJHIIACIAACIBAAoH\/dus36Ko\/+EBwbIIRMDxlxGdN8OmhebuQOIqm4oVtWcBMTEwEF\/xaFwhc5SIAtmArR0DOM+qtf7YjDx2iK0Yeo+mbLvXvvMYegxAwSTsaFNek6+1tdx7hSyVTS8FVhitEtxxXsAVbWQL+vUPA+Gcq6jFvBIZHa\/gUyqxpIz2D6GhlwgWuMlzRycpxBVuwlSXg3zsEjH+moh7jAkaNuPDupGXLlkU3BKtjwVVGso5eR0crEy5wleGKTlaOK9iCrSwB\/94hYPwzLZVHdLQy4Xr66afpjDPOkHE+z72CrVwFAFuwlSPg3zMEjH+mpfIIASMTLnQEMlzZK9iCrRwBOc+ot\/7Z9t+zn\/rveRqLeP2jLYdHCBiZOKGxkuEKASPHFWzBVpaAf+8QMP6ZlsojBIxMuCBgZLiik5XjCrZgK0vAv3cIGP9MS+URAkYmXBAwMlzRycpxBVuwlSXg3zsEjH+mpfIIASMTLggYGa7oZOW4gi3YyhLw7x0Cxj\/TUnmEgJEJFwSMDFd0snJcwRZsZQn49w4B459pqTxCwMiECwJGhis6WTmuYAu2sgT8e4eA8c+0VB4hYGTCBQEjwxWdrBxXsAVbWQL+vfM9SHwWDO5C8s+2FB4hYGTCBAEjwxWdrBxXsAVbWQL+vUPA+GdaKo8QMDLhgoCR4YpOVo4r2IKtLAH\/3iFg\/DMtlUcIGJlwQcDIcEUnK8cVbMFWloB\/7xAw\/pmWyiMEjEy4IGBkuKKTleMKtmArS8C\/dwgY\/0xL5RECRiZcEDAyXNHJynEFW7CVJeDfOwSMf6al8ggBIxMuCBgZruhk5biCLdjKEvDvfe1tj9ADT72EXUj+0ZbDIwSMTJwgYGS4opOV4wq2YCtLwL93CBj\/TEvlEQJGJlwQMDJc0cnKcQVbsJUl4N87BIx\/pqXyCAEjEy4IGBmu6GTluIIt2MoS8O8dAsY\/01J5hICRCRcEjAxXdLJyXMEWbGUJ+PcOAeOfaak8QsDIhAsCRoYrOlk5rmALtrIE\/HuHgPHPtDAex8fHqbu7mwYGBqipqSkxXxAwMuGCgJHhik5WjivYgq0sAf\/el9\/wIB2cfh27kPyjra3HI0eOUG9vL+3bt4+GhoYgYKocDggYOeBgC7ZyBOQ8o976Z1t\/9X2RU1zm6J9tTT3u3buX+vv7ozxgBKb6oUBjJcccbMFWjoCcZ9Rbv2x55IVHYCBg\/HKtubfp6Wm67rrrqKOjIxIxEDDVDwkaKznmYAu2cgTkPKPe+meLERj\/TGvucefOnVEeVqxYgTUwNYoGGis58GALtnIE5Dyj3vpl+8CTL9Ha2x\/BCIxfrLX1xgt3t2\/fTps2baLJyUkjAaPneM+ePbUtQCBvZ\/aNjY2BlKZYxQBbuXiALdjKEfDjefXq1ZGj18+6nF4\/ay0EjB+sxfDCU0arVq2ilStXEnYh1S4m+GtLjj3Ygq0cATnPqLd+2f7vf36Ortr5QwgYv1hr543XvnR1ddHY2NicTAwPD0eiJv5gG7VMvNBYyXBlr2ALtnIE5Dyj3vplq86A+ZXXnqfnv\/hHfp0XwNtxR48ePVqAfNQsCxiBqRl6dLKC6NERyMEFW7CVI+DXs1rAe\/zzj9OPv\/Jnfp0XwBsEDA6yq1k1REcghx5swVaOgJxn1Ft\/bPUt1Cc+MEAHv\/v3\/pwXxNO8FzAmccAUkgklexs0VvbMTFOArSkpezuwtWdmmgJsTUnl2+k7kCBg8nkFawEBIxNaNFYyXNkr2IKtHAE5z6i3\/tiq9S+L60+gl7\/8X2hiYsKf84J4wgiMQSAgYAwgOZigsXKAZpgEbA1BOZiBrQM0wyRgawgqx0yfPrpo6QL6\/taPQsD4QVs+LxAwMjFDYyXDFSMwclzBFmxlCfjxPvLQIbpi5LHI2W0dZ9OmthYIGD9oy+cFAkYmZhAwMlzRycpxBVuwlSXgx7s+ffToZy+gUPswTCEZ1JdQg29QdFETCBg5vGALtnIE5Dyj3lbOVh996VlzBvWsWQIBUznW8nqAgJGJHRorGa4YJZDjCrZgK0ugMu+89oXvPuJ\/efHurk+dG\/0bah+GERiD+hJq8A2KLmoCASOHF2zBVo6AnGfU28rY9t41Tl\/6zmTkRI2+8P+H2odBwBjUl1CDb1B0URM0VnJ4wRZs5QjIeUa9dWerTx3xqAuvfVFPqH0YBIxBfQk1+AZFFzVBYyWHF2zBVo6AnGfUWze2+rZpFi+3tp9NF525AALGDWdYqSBgZOKJxkqGK3sFW7CVIyDnGfXWnq0uXjg1r3vRxQumkOyZBpUCAkYmnGisZLhCwMhxBVuwlSVg5z0uXnjkZd0HF81xEmofhikkg\/oSavANii5qAgEjhxdswVaOgJxn1FtztvpdR5zqExeeTls+uizRQah9GASMQX0JNfgGRRc1QWMlhxdswVaOgJxn1Nt8tjzqsvN7h2jzN5+eMf76ny+nS5oWpiYOtQ+DgMmvL8FuQTMouqgJGis5vGALtnIE5Dyj3maz5VGXK3c8Fp3zwk\/Sgt0kDxAwcnW28J5DDX6twaOxkosA2IKtHAE5z6i36WyvHHmMhh86NGOgH1SXF5FQ+zCMwORFPuBDgAyKLmqCxkoOL9iCrRwBOc+ot7PZ8kjLtx6fpqu\/+visD\/72Ex+gD51zinEgIGCMUYVnGGrwax0pNFZyEQBbsJUjIOcZ9fYYWxYud\/\/rYdr0jSdnwY4fUGcaiVD7MIzAGNSAUINvUHRREzRWcnjBFmzlCMh5Rr0lik8VMW3TtS5pkQm1D4OAMfguhhp8g6KLmqCxksMLtmArR0DO83ytt7w4d+Cep+mBp16aM+ISP1XXhX6ofRgEjEFtCDX4BkUXNZmvjZUo1Dedg60cZbAFWx8EWLTc+ciP6G8enJrjrtIRl7jDUPuwYATMkSNHqLe3l3bv3h3Fbv369dTT05Naz3bu3EkbN26MPm9tbaW+vj6qq6tLtA81+D6+hJX4QEdQCb3stGALtnIE5DyHXm95bcuOhw5R3z1vneGiaLJouXDpgugWaf5\/n0+ofVgwAqa\/vz+KN4uW6elp6urqovb2dmpra5tTD\/bu3UtsPzg4GIkWFj4NDQ2pgifU4Pv8grj4Cr2xcmHiKw3Y+iI51w\/Ygq0NAR5p+eYPnqfb738mMZnv0Zakl4TahwUjYOJB0wVN\/DMefRkdHZ0ZdYn\/PF+G32y+hBK26AgkqB7zCbZgK0dAznMo9TZtTYs+2sJrW1i8+B5tgYCRq59V8axGYHg0ZuXKlUYjMC0tLYmjNZw4VPValWBkvCSUxqrWHJPeD7ZyUQFbsI0T4Kmh7Q9O0UP7fzJnIa4uWm5pO4vec3JdVUSLnsdQ+7DgRmB45GXbtm2561rGx8eps7OTpqamaHh4OFHoqArAwdefPXv2yH2D55HnyclJamxsnEclrl5RwVaONdjOb7ZTL78RATj4k5\/Tlx\/6Ce179tix\/vGn4aTj6bR3HE\/rf3dB9C\/\/XK1n9erVc141MTFRrddX7T3BCRhFjoUMi5Okxbk8ZbRjx45oDUx9fX20HoaftEW\/oarXqtWylBfhL1m5CIAt2MoRkPNc1Hqr1rH8y+QrqSMsTIWng9Z98DRqee8CuujMBXKgLD2H2ocFK2B4hKW7u5sGBgaoqalpJtxqt5I+ZZRmqxKFGnzL74B386I2Vt4LWgOHYCsHHWzDZ8uC5WsP\/4i27527xVkvPQuWi85cSO2\/s6hq61lc6IfahwUrYPSdRjzKoh4IGJfqL5MGHYEMV\/YKtmArR0DOcy3qLYuVQy\/\/lL7y4FTm6IoaYeGtzh3nn1ZowRKPEASM5zqrFtqOjY1ZeW5ubqa77rprThp9GkiJlLSt0UlTSGnTTfyiUINvBV7AuBaNlUAxCukSbOXCArblZcti5cev\/Iz+ZvTZXLGiBMtHz\/1N+vgFDVGhq7FjSIJuqH1YzUZg8nYKJQVRjaokCZj4QXb64XTqs46OjpnFumqxL78HB9lJfGXyfaIjyGfkagG2ruTy04FtPiNXC59sWawcfvVnNPT\/zMXK4oUnUPebB8mVVawksYeAca2RKel8CxjP2ZvlLtTgSzIz8e2zsTJ533yyAVu5aINtcdjy9mUWGvzv1nv309PPHzEeWWGx8qcXN9KC3\/i1Uk0HudAPtQ+r2QiMSxBqlSbU4NeKp3ovOgK5CIAt2MoRkPOcV295VOWHh\/6ddo392EioqGkfFiuXvf9kOvfdJxVqd5AcydmeQ+3DaiZg9DUwefcWVSvIae8JNfi15prXWNU6f2V+P9jKRQ9sZdn+6jtPi0ZUvrv\/J\/Ttx6fp4IuvRz+bPOo+IV5kOyNePN8rZJKPotmE2ofVTMCoAOtrUXjR7dDQ0Kxtz0WoCKEGv9Zs0RHIRQBswVaOgB\/PSpTs+eE03fXIj6yFCo+qXPq+ejp\/yTuDnwKqlHiofVjNBYwKTHxXUpFGZUINfqVfikrTo5OtlGB6erAFWzkCdp552oefu79\/mH7w7KvGUz9qBIX\/VVuXi3Q4nB2F2lqH2ocVRsDo4dWP+S\/CqEyowa\/tVwpnlUjyh4CRowu2c9nyaAr\/96NXfkrbR6esRlN0odL8m8fT9R\/9QOSrWhcdytWU4ngOtQ8rpIDRw552IF01q0aowa8mw6R3oSOQiwDYgq1vAkqkPHX4tWh9yjPTr1uNpiihwlM\/F565MBpVUVuV1b+ot76jdsxfqH1YIQWMPgLD8PMuW5QJ+VteQw2+NLc8\/2is8gi5fw627uzyUobOlqd83vWOX6e\/uu8gHXzBbFuyzkyJERYoGy5bQs++9FPj0ZTQ2ebVLanPQ+3DCiNgsg6ikwqqqd9Qg29afik7NFZSZDE9J0e23GzV1AyLlOnXfk73\/OB5p5EUfTTlYxc00KKT3jZnNMUlBmgTXKjlpwm1D6u5gNF3IRVhtCWpKoQa\/PxqL2uBxkqOL9jOb7Zqh8\/wd5+j0adesl6TouhF61AWHruwsGXpsduVJdemoN7K1NtQ+7CaCRh911HeUf4yITX3GmrwzQnIWKKxkuHKXsE2bLZKoDx26FXaNXbYeRRFH0k5d\/FJ1HXh6TVdQIt6K1NvQ+3DaiZgnnzySbrqqqvo2muvnbmfKC90WXch5aWt5PNQg18JEx9p0Vj5oJjsA2zLy1af5uFSjDz0XCRQbA5000uv1qTwKMrZi95OzY3vEB1FqYQ86m0l9NLThtqH1UzA4C4kmYpaJq9orOSiBbbFZqt29LC4uOW+g\/T4oX93Fij6KMo5p59If3DOqVHhy3hmCuqtTL2FgPHMNX5wnan75uZmSrqN2jS9i12owXdh4TMNGiufNGf7Atvas+WFsu9+U6A84UGgRKJk6UL6i\/+4mJ77yc+8LJqVo+TmGfXWjVteqlD7sJqNwOQBL9LnoQa\/1ozRWMlFAGxl2fJ9Pfy8\/vNfRiMoB144UvEIyrFRk4V05ql19DvveWfkX3LBrBwhd8+ot+7sslKG2odBwBjUl1CDb1B0URM0VnJ4wbYytmqK555\/e57GnnklcvbAU8eOxHd59LNR\/sNZJ9P57zkpWixbxmkel\/KbpkG9NSVlZxdqHwYBY1APQg2+QdFFTdBYyeEF23S2Spw0LHgb3fSPB6LD2lwXyKq3qO3GPGX04eXvovf95ttnMqDEi1y0w\/GMeisTy1D7MAgYg\/oSavANii5qgsZKDu98Zau2F\/O\/\/zT+Iu2dODZq4mP0hM9D+UDjO+jck39OixadNu+md+Rq61ue52u9lWYbah8GAWNQc0INvkHRRU3QWMnhDY1tfGvxD557le7+l8MVixN2oG8zblzwtmgdiv77+AhKaGzlaqG9Z7C1Z2aSItQ+DALGIPqhBt+g6KImaKzk8JaFrRImalqHibz8+ht0978eO5zNx9ROJEZ49OT0d9CHzjllRpy4Tu2Uha1c7ZLzDLYybEPtwwolYHbu3EkbN26MIsgXOB44cIBGR0epr6+P6urqMiMbv0tp\/fr11NPTk5qGD8Vbt25d9DlvzR4cHKT6+vpE+1CDL\/NVMfeKxsqcla1lkdjqZ55Ei2InX\/Wy7kSNkrA4OeOUOmr97VPphF\/7VfHtxUVia1svim4PtjIRCrUPK4yA4TuRpqamqLu7m6688spIfLCw6O3tpYaGhkwxwiHn9PxwOnXGTHt7O7W1tc2pEeq26y1btkSnALNwyhJKoQZf5qti7hWNlTkrW8tqsdXXnHx3\/0\/o249PR1mtZM2JKqtaGLts0dujrcV8QJsasVHixZaLD\/tqsfWR17L5AFuZiIXahxVCwOin8i5btoy6uroiIcLiQl0fkDVCkhRyXdDEP2fBsn\/\/\/lxRpNKFGnyZr4q5VzRW5qxsLStlq685OUpH6e+\/\/zx9\/9lXvYsT3rXzyYsb6dXXfyE+cmLLMM2+Ura+8hGiH7CViWqofViQAibrmgI11dTS0pI4OpNUfUINvsxXxdwrGitzVraWaWzji2EPvfxTuu\/xaS\/rTfRREZ7WWXzyCbT2t99FZy16e1BnnqDe2tZGc3uwNWdlYxlqH1YIAcOBUNM4+hSSGo1JmwpKG3nZtm0bpd1wrQTMmjVr6I477qCxsTGsgbH5Jni0RWPlEeabrtR6k+eee46ePlJHD4y\/6HXUJBIpC0+g9zecSP\/pt06h43\/luOgwNjWV5Low1j8JOY+ot2ArR0DGMwSMDNdZXvWFteqDzZs3G4+U6M7Umpr4AmAlYA4ePDizcDfNVvnj4OvPnj17qkAj\/FdMTk5SY2Nj+AX1VMJ9z74eefrFL4\/S3z3+7zT18hv03CtvRP9W+jScdHzk4rR3HE+\/vehtdMF76qL\/V4\/6vNL3hJAe9VYuimDrh+3q1avnOJqYmPDjvEBeCjMC45sJL9Tl0ZyBgQFqamqacZ80hZRmqwuYEIPvm7mtP\/wle4wYX\/rHz6J3\/jrdct8z9PTh1479voKj61Us1IgIj5r8VsOJtGpZPb3\/tGNTOjOf1Z9gG7p5bY96Kxd+sJVhixEYGa5iXrMW\/\/KIy5IlS2ZGdljA3HjjjbR169bErdShBl8MvqHj0BsrNa3COL48+iw9fODliEylZ5vMESf1dfSB00+k9Rc3zgiTZ555hi5sfku4G4YEZgYEQq+3BgjETMBWBm2ofVghRmDUoltej5L1ZJ3tou86UqMsaduv4+Ima8cS5yfU4Mt8Vcy9lq2x0g9dU6W869Ef057HXpARJm8uhN1w2RKafPGnViMmZWNrXmtqbwm2cjEAWxm2ofZhhRAwHDJexLtjx45ZB8rp57msXbs280yY+EF2+iJe9VlHR0e0NZsffb1N2oJfVZVCDb7MV8Xca1EaqyRhwmea+Nydw1T06RzePvyxlQ30818ctRImpnSLwtY0v2WyA1u5aIGtDNtQ+7BCCJisbc\/6aMkTTzwRHVh31113yUQ5xWuowa8qxISXSTdW8WPqWUD867Ov0N99\/3lv24ZVsfTbiFcsPim6jbiWa0yk2da67tTy\/WArRx9sZdiG2odBwBjUl1CDb1B0UZNKGyv9\/pw3fnmUvrrvkIgwiUZPFp5Al55VT+e\/550Rk0iwFHjxa6VsRQNfcudgKxdAsJVhG2ofVggBYzKFxFcCqLNibr75ZpkoYwSmqlyzDltTC2BPPvHX6PZvP0MHXjjibfFrfDrngqUL6GLtBuIiCxPTAKEjMCVlbwe29sxMU4CtKSk7OwgYO15O1knnwPCljuq+oltuuYWGhoZmbYt2epFlolCDb4nBi7l+sd\/f3PdD+t6ho5Ff7ztzFp5Av7PkJPrjC06fdchaCOLEJBDoCEwoudmArRs3k1Rga0LJ3ibUPqwwIzD2IaleilCDL0HtBEJVAAAgAElEQVRQCZSf\/+KX9LVHfuz11uFo5OTNU2A\/terd81KYmMYMHYEpKXs7sLVnZpoCbE1J2dmF2odBwBjUg1CDb1D0OSb67cP\/5+Ef0cTh1yo+cE1fAPvHFzTQT9+Q2ZnjUt6ypkFHIBc5sAVbOQIynkPtwwojYPgwuc7OTpqampoTwebm5lnbq2VCnO411OCnlVi\/8O+xQ6\/S7rHDziJFFyeXN8++2A8dgVxNBluwlSMg5xn1VoZtqH1YIQSMfry\/Ou+Fz2xRlzn29PTMnN8iE95sr6EGXy81H2f\/tYd\/RE9ZjqgogbL83SfR2ae9nS5cumDGbd56EzRWcrUZbMFWjoCcZ9RbGbah9mGFEDDxc2D0o\/55Ye\/IyAjFL2WUCXOy1xCDP\/LQIRr57nNGIytKiKx5\/yl0xe8dW3vCNxBX+qCxqpRgenqwBVs5AnKeUW9l2IbYhzGpQgoY3i69f\/9+4pGXrDuNZEI912vZg68W1o489ByxcMl6WKysalpIf3TeIvGzTtBYydVgsAVbOQJynlFvZdiWvQ9Lo1IIAcOZ0+8j0kXLvffeS6OjoxiBcajXLFy23ruf\/tc\/P5eYWk3\/3NpxdvR53pSPQxYyk6Cx8k30LX9gC7ZyBOQ8o97KsIWAkeE641VfB8OH1rGg2bZtG\/GFjLU4+0UvbtmCz+tZrtzx2Mw2Y70sLFLWX9JIf37Ju4Ujmu8ejVU+I1cLsHUll58ObPMZuVqArSu57HRl68NMKRRmBMY0w7WwK0PwebTlW49P09VffXwOIhYtt7afLT4lZBsbNFa2xMztwdacla0l2NoSM7cHW3NWNpZl6MNsyqNsCyFgTC9zrK+vdyljxWmKHHy1vmXt7Y\/MKieLlg8vfxd9ouX0qk8NmQJHY2VKyt4ObO2ZmaYAW1NS9nZga8\/MJEWR+zCT\/KfZQMAY0Ctq8JOmitRoi49dQgZoKjJBY1URvszEYAu2cgTkPKPeyrAtah9WaWlrKmB4t9HGjRtzy7B+\/fpoR1KtnqIFn0ddeDdR\/z1PzyApk3BRmUZjJVejwRZs5QjIeUa9lWFbtD7MVylrKmBUIbKmkHwVtBI\/RQv+8hsenHUP0MBHltHvv\/\/kSopYk7RorOSwgy3YyhGQ84x6K8O2aH2Yr1IWQsD4KoyUn6IEn6eM9LUuZRx10WOExkqqxhKBLdjKEZDzjHorw7YofZjv0kHAGBCtdfB5yuieHzxPPXeNz+T2K53n0H\/+wKkGuS+uCRorudiALdjKEZDzjHorw7bWfZhMqWp4Eq+aNhobG8st23y+zJHFy87vHaLN3zy23qXsoy4Ygcmt7l4M0BF4wZjoBGzBVo6AjGcIGBmupfBaq+DHF+uyeNn1qXMLuy3aNpjoCGyJmduDrTkrW0uwtSVmbg+25qxsLGvVh9nk0cU2mCkkdZLv7t27Iw6mO5fGx8epu7ubBgYGqKmpKZFhrYJ\/xwPPUs+dT8yMvIQkXrhQaKxcvrJmacDWjJOLFdi6UDNLA7ZmnGytatWH2ebT1r5QAiZpW\/XmzZuJrxbIe\/S7lNT0VHt7e2ZaJXr27duXeV1BLYLP26SvGHksWPECAZNXoyv7HB1BZfyyUoMt2MoRkPFciz5MpiSzvRZGwLB42bFjBw0ODpI6cddUiCSB0gVNGkh1aSR\/XqQRmPkgXiBgZL\/e6GTl+IIt2MoRkPEMASPDNfLq+yoBk3Nl2Oa6666jjo6O6OLIoggYXvfC57yo59HPXhDMmpd4FUJHIPelAluwlSMg5xn1VoYtBIwMV+8CRt1i3draSn19fVRXV5eYcx7x4WfFihVGa2B0J3v27BGhMfXyG\/SX\/\/g87Xv29cj\/lz6yiM47\/QSRdxXB6eTkJDU2NhYhK8HlAWzlQgq2YCtHwI\/n1atXz3E0MTHhx3mBvAQ9hTQ1NZUoYnjh7vbt22nTpk3EjVFRFvH237N\/5nqAi5YuoF1XnFugquI\/K\/hryz9T5RFswVaOgJxn1FsZthiBkeE6y2sli3jj2cvaXcSjNKtWraKVK1dSUXYh6VNHoW2XTqs6aKzkvlRgC7ZyBOQ8o97KsIWAkeEq5lUt0NUXBfPLsg7QGx4ejkRN\/JEOPouXK0ceoweeeil6NW+XLsNt0pUGD41VpQTT04Mt2MoRkPOMeivDVroPk8l1vtdCTCFVsttIFVHfdaS2Rzc0NOTeYl2EERh919F8mDrCNEf+F7NSC3QElRKEOJQjCLbVZgsBI0w8Pn2UNhqSlo34QXb6Il71Ge84io+w1FrAzMepIwgY4S8TDgkUBQxxKIcXbGXYQsDIcE30qnYS8Yc8ijI0NJR6Sm41siUZ\/M\/fPUH\/c8+BqBg9a86gnjVLqlGkQrwDjZVcGMAWbOUIyHlGvZVhK9mHyeTYzGshppCysspihtezxNeymBXPj5VU8OOjL3zmy3x60FjJRRtswVaOgJxn1FsZtlJ9mExuzb0WUsDoIzC1vomaUUoF\/zNff5K++E\/PRNG6reNs6jh\/kXnkArBEYyUXRLAFWzkCcp5Rb2XYSvVhMrk191oYAVO0aSMdoVTw66++L3oNb5ueb6MvXG40VuZfVFtLsLUlZm4PtuasbC3B1paYmb1UH2b2djmrQggYk6P\/5RDke5YI\/sA\/7Ke+bz49b0dfIGDy610lFugIKqGXnRZswVaOgIxniT5MJqd2XgshYOyyXH1r38Gf72tfVATREcjVZbAFWzkCcp5Rb2XY+u7DZHJp7xUCxoCZ7+DrAma+HFqXhBmNlUHlczQBW0dwBsnA1gCSownYOoLLSea7D5PJpb1XCBgDZj6Dz+Jl7e2PEP87X9e+YATGoNJVaIKOoEKAGcnBFmzlCMh49tmHyeTQzSsEjAE3n8HXR1\/++0eXUdeFpxvkIEwTdARycQVbsJUjIOcZ9VaGrc8+TCaHbl4LIWCyFvGm3WnkVly3VD6Dv\/a2R+bdnUdp1NFYudVHk1Rga0LJzQZs3biZpAJbE0r2Nj77MPu3y6WAgDFg6yv4+ujLfLrzCALGoJJ5NkFH4Bmo5g5swVaOgIxnX32YTO7cvdZUwMTvP0orxvr163MvZXRHkJ\/SV\/D779lP\/fcc2zo9nxfvKuLoCPLrnqsF2LqSy08HtvmMXC3A1pVcdjpffZhM7ty91lTAqGzPl3Ng1PTRfF+8CwHj\/oU1TYmOwJSUvR3Y2jMzTQG2pqTs7CBg7HgFZe0j+A88+VK0+4gfvjKArw6Y7w8aK7kaALZgK0dAzjPqrQxbH32YTM4q81qIEZjKiiCf2kfw\/\/L\/PkV\/9a2DUWYxfXQsZmis5Oou2IKtHAE5z6i3Mmx99GEyOavMa80EjD5ttGzZMurq6qKxsbHE0tT6QkcfwZ\/v9x4lBRaNVWVf3qzUYAu2cgTkPKPeyrD10YfJ5KwyrzUTMJVlu7qpKw2+Pn209Q+XUWfL\/D37RY8cGiu5egy2YCtHQM4z6q0M20r7MJlcVe4VAsaAYaXBv\/7uCfofew5Eb8L00VvA0VgZVD5HE7B1BGeQDGwNIDmagK0juJxklfZhMrmq3GshBIyaTgp1Cgm7j5IrKhqryr\/AaR7AFmzlCMh5Rr2VYQsBI8M10ysLm2uuuYY+85nPUFNTUw1ycOyVlQRfnz769KWL6brWpTUrR9FejMZKLiJgC7ZyBOQ8o97KsK2kD5PJkR+vhRiBySoKXyUwMjJCfX19VFdXl2p65MgR6u3tpd27d0c2WYffxUd8WltbM\/1XEnwcXpceXTRWfr7ESV7AFmzlCMh5Rr2VYVtJHyaTIz9eSyFg+vv7aXBwkOrr61NLzTb89PT0kBIo7e3t1NbWNiuNEjotLS3RZ+rnhoaG1NN+Kwk+po8gYPx8Ve28oCOw42VjDbY2tOxswdaOl6l1JX2Y6TtqYVd4AcPCZGpqKncEJg5PFzR5YPlKg9HR0dR3uAZfv\/vokqaF9PU\/X56XlXn1ORoruXCDLdjKEZDzjHorw9a1D5PJjT+vhRAwWYt4eWRkaGjIag2M7dUEUgJGX\/\/CJ+\/yCbx43iKAxkquNoAt2MoRkPOMeivDFgJGhuuM17SpHTXVY\/p6HnnZtm0b5a1rUf6yppuUjWvw1fQR+8H26bkRRGNlWqvt7cDWnplpCrA1JWVvB7b2zExSuPZhJr5raVOIERgGkDRVZCIu0uCZTD0p0cQ+shYJc\/D1Z8+ePUYxa90+SVMvv0ENJx1Puz\/eaJRmPhlNTk5SYyO4SMQcbCWoHvMJtmArR8CP59WrV89xNDEx4cd5gbwUQsBkTfmY7kKKMx0fH6fu7m4aGBhInH4yFS\/s10W96utfcHljco3HX1tyLQHYgq0cATnPqLcybF36MJmc+PVaCgFjsgspjoWFT1o6k51Huj+X4GP7dH5FRWOVz8jVAmxdyeWnA9t8Rq4WYOtKLjudSx8mkxO\/XgshYOLrX\/Qi5i2wVbb6rqM8gWIyvVSpgNHXv0zfdKnfqAXiDY2VXCDBFmzlCMh5Rr2VYQsBI8N1xiuPmGzYsGHWjiOeBurs7KQtW7bQypUrM3MQP8hOX8SrPuvo6KC0m6+zbrx2CT5un86vMGis8hm5WoCtK7n8dGCbz8jVAmxdyWEERoachVcWMevWrZuVYnh4OFe8WLzCydRWwOjrX3rWnEE9a5Y4vTf0RGis5CIMtmArR0DOM+qtDFvbPkwmF\/69FmIKyX+x\/Hq0Db5+\/gu2T6fHAo2V33qqewNbsJUjIOcZ9VaGrW0fJpML\/14LIWD0KZ68qSL\/CPI92gZfX\/\/y6GcvoMX1J+S\/ZB5aoLGSCzrYgq0cATnPqLcybG37MJlc+PdaCAFje3KufwzZHm2Dv\/yGB4mnkVi4sIDBk0wAjZVczQBbsJUjIOcZ9VaGrW0fJpML\/14LIWC4WKa7jfwjyPdoE3x9\/ctFSxfQrivOzX\/BPLVAYyUXeLAFWzkCcp5Rb2XY2vRhMjmQ8VoIAZN1FxIXO2uHkAyW2V5tgq+vf8EC3uzooLGSq71gC7ZyBOQ8o97KsLXpw2RyIOO1EAJGpmj+vNoEHwfYmXNHY2XOytYSbG2JmduDrTkrW0uwtSVmZm\/Th5l5LIYVBIxBHGyCrxbwYv1LPlg0VvmMXC3A1pVcfjqwzWfkagG2ruSy09n0YTI5kPFaMwGjL9xNO1xOFbksU0hY\/2JXSdFY2fGysQZbG1p2tmBrx8vGGmxtaJnbQsCYswrO0jT4OMDOLvRorOx42ViDrQ0tO1uwteNlYw22NrTMbU37MHOPxbCs2QhMvPjx+5Cy7keqNjrT4OMAO7vIoLGy42VjDbY2tOxswdaOl4012NrQMrc17cPMPRbDsjACJumCRTXN1N7eTm1tbTUjZhr8v\/3uIfr0jseifOIE3vxwobHKZ+RqAbau5PLTgW0+I1cLsHUll53OtA+Tebuc10IImKyD7Ph+pJGREerr66O6ujo5EhmeTYOPBbx24UFjZcfLxhpsbWjZ2YKtHS8ba7C1oWVua9qHmXsshmUpBAyPzgwODlJ9fX1NqJkGX91AjQPszMKExsqMk4sV2LpQM0sDtmacXKzA1oVafhrTPizfU7EsCiFgsta7FOGEXpPgYwGvfcVGY2XPzDQF2JqSsrcDW3tmpinA1pSUnZ1JH2bnsRjWhRAwjIKnijZs2EBDQ0PU1NQU0RkfH6fOzk7asmUL1fKSR5PgYwGvfYVGY2XPzDQF2JqSsrcDW3tmpinA1pSUnZ1JH2bnsRjWhREwSsSsW7duFpnh4eGaihfOjEnw9RN4cQO1WeVGY2XGycUKbF2omaUBWzNOLlZg60ItP41JH5bvpXgWhRIwxcNzLEcmwccCXvvoobGyZ2aaAmxNSdnbga09M9MUYGtKys7OpA+z81gMawgYgziYBH\/5DQ8Sr4PBAl4DoG+aoLEyZ2VrCba2xMztwdacla0l2NoSM7M36cPMPBXLCgLGIB55wccVAgYQE0zQWLlxM0kFtiaU3GzA1o2bSSqwNaFkb5PXh9l7LEYKCBiDOOQFHzuQDCBCwLhBckyFjsARnEEysDWA5GgCto7gcpLl9WEyb5X3Oi8FjNq2vXv37ojw+vXrqaenJ5V2XvCxgNetoqKxcuNmkgpsTSi52YCtGzeTVGBrQsneJq8Ps\/dYjBTzUsDwwXj8sGgxua4gL\/hXjDxGIw8dinxiB5J5xUZjZc7K1hJsbYmZ24OtOStbS7C1JWZmn9eHmXkpnlVhBIw682VqamoOpebmZtGTeHVBkxSivOBjB5JbxUZj5cbNJBXYmlByswFbN24mqcDWhJK9TV4fZu+xGCkKIWDUlE5DQ0PmVI4Esqx7mNT78oKPKwTcIoPGyo2bSSqwNaHkZgO2btxMUoGtCSV7m7w+zN5jMVIUQsCYiAgJXDzysm3bNmptbc28LJKDrz979uyZ+XHq5Teodftk9PP6Dy6gP\/3dBRJZDdLn5OQkNTY2Blm2WhcKbOUiALZgK0fAj+fVq1fPcTQxMeHHeYG8FELAqBGYjo6Ompy6y0KGp67SbrzOUq+4QsC9NuOvLXd2eSnBNo+Q++dg684uLyXY5hFy+xwjMG7cjFPxXUi1unWa1990d3fTwMDAzD1Mesazgq\/vQNr1qXPpojMxAmMadDRWpqTs7cDWnplpCrA1JWVvB7b2zExSQMCYUHK0UVNIY2NjiR6kF\/Hmiaes4G\/6+pP01\/\/0TJRv7ECyqwBorOx42ViDrQ0tO1uwteNlYw22NrTMbSFgzFkV3lLfdWSygDgr+NiB5B5uNFbu7PJSgm0eIffPwdadXV5KsM0j5PY5BIwbt0Kmih9kZ7KIN20BFO5Acg8xGit3dnkpwTaPkPvnYOvOLi8l2OYRcvscAsaNm1WqnTt30saNG6M0w8PDdODAARodHc3cIWT1AkfjrOBjC7UjVCJCY+XOLi8l2OYRcv8cbN3Z5aUE2zxCbp9DwLhxM06ldgLxYtorr7wyOg+G17709vZSLc6H0TOeFnx9B1LPmjOoZ80S4\/LCEAJGsg6gI5CjC7ZgK0dAxjMEjAzXyKt+DsyyZcuoq6srEjArV66kvAW2gtmacW0iYLADyT4S6AjsmZmmAFtTUvZ2YGvPzDQF2JqSsrODgLHjZWUNAWOFKxhjNFZyoQRbsJUjIOcZ9VaGLQSMDNcZr7z+hde76FNIajSmvb2d2trahHOQ7j4t+GoHUjSKdNOlNctfWV+MxkoucmALtnIE5Dyj3sqwhYCR4TrLK08XrVu3btbvNm\/eXFPxwpnJEzCL60+IzoDBY0cAjZUdLxtrsLWhZWcLtna8bKzB1oaWuS0EjDmr4CzTgo8dSJWFGo1VZfyyUoMt2MoRkPOMeivDFgJGhmspvCYF\/+D068RnwPDTcf4iuq3j7FKUpUiZRGMlFw2wBVs5AnKeUW9l2ELAyHCd5VU\/B0Z9wOfB8G6kWj5JwccW6sojgsaqcoZpHsAWbOUIyHlGvZVhCwEjw3XGK4uXHTt20ODgINXX10e\/V7uTiriIVx+BwRZqt8qBxsqNm0kqsDWh5GYDtm7cTFKBrQklexsIGHtmxin0bdTx0ZaingODW6iNw5tqiMaqcoYYgZFjCLZgW30CMm+EgJHhOmukRR1ep7+qqALmipHHaOShQ8fyjy3UTrUDAsYJm1EisDXC5GQEtk7YjBKBrREmayMIGGtkdglYqGzYsIGGhoaoqalplrAp4hQSbqG2i2+SNRqryhlilECOIdiCbfUJyLwRAkaG6yyhMjY2lvsWvh\/prrvuyrXzaZAUfNxCXTlhCJjKGaKTlWMItmBbfQIyb4SAkeFaCq\/x4OsLeC9auoB2XXFuKcpRtExCwMhFBGzBVo6AnGfUWxm2EDAyXEvhNUvA4BZq9xCisXJnl5cSbPMIuX8Otu7s8lKCbR4ht88hYNy4WaVKOgemiFcJ6GfA8BUCfJUAHnsCaKzsmZmmAFtTUvZ2YGvPzDQF2JqSsrODgLHjZW1dpnNgePcR70LiB2fAWId6JgEaK3d2eSnBNo+Q++dg684uLyXY5hFy+xwCxo2bUaqynQODM2CMwpprhMYqF5GzAdg6o8tNCLa5iJwNwNYZXWZCCBgZrpHXsgmYtbc\/QjyNFOUdZ8A41ww0Vs7ochOCbS4iZwOwdUaXmxBscxE5GUDAOGEzT1TpFJISQWordmtrK\/X19VFdXV1iJvT1Nnm28eDjDBjzuGZZorHywzHJC9iCrRwBOc+otzJsIWBkuM7y6rqI98iRI9Tb20stLS3U1tZG6ueGhgbi033jj366LwscTptmy2njwccZMH4qAxorPxwhYOQ4gi3YVpeAzNsgYGS4inllMTQ6Opo4ChP\/LMs2LmBwBoy\/kEHA+GMZ9wS2YCtHQM4z6q0MWwgYGa5iXrNESdIIjBq9ScqQHnxdwNz0R++jP76gQawMoTtGYyUXYbAFWzkCcp5Rb2XYQsDIcBXxqtbDZN2hND4+Tp2dnTQ1NUXDw8MUvwVbz5gefP0MGBxiV1n40FhVxi8rNdiCrRwBOc+otzJsIWBkuHr3qta\/sOO0RbzxBcP9\/f1RPpLWy\/DvOfjq+dniC+m1FZ+IfvzSRxbReafjEDvXIE5OTlJjY6NrcqTLIAC2ctUDbMFWjoAfz6tXr57jaGJiwo\/zAnk57ujRo0cLlJ+KsmIiXuILfvmFPBrT3d1NAwMDMzdhp43A4AyYikI0KzH+2vLHMu4JbMFWjoCcZ9RbGbYYgZHh6s1r3s4j9aJKBQyfwMsn8fKDawQqCx8aq8r4ZaUGW7CVIyDnGfVWhi0EjAxXb155GojXs2Sd\/aJeljSFlJVWDz7OgPEWMkJj5Y8lRmDkWIIt2FaPgMybIGBkuHrxGj\/ETjltbm6mwcHB6DA7Puulo6NjZrEuC55t27ZFpjYH2UHAeAlZ5AQCxh9LdLJyLMEWbKtHQOZNEDAyXEvhVQ9+\/dX3RXm+aOkC2nXFuaXIf1EzCQEjFxmwBVs5AnKeUW9l2ELAyHAthVcVfP0MmI7zF9FtHWeXIv9FzSQaK7nIgC3YyhGQ84x6K8MWAkaGaym8JgkYnAFTeejQWFXOMM0D2IKtHAE5z6i3MmwhYGS4lsKrCr5+iB2PvvAoDB53Amis3NnlpQTbPELun4OtO7u8lGCbR8jtcwgYN25BpEoSMLs+dS5ddOaCIMpXq0KgsZIjD7ZgK0dAzjPqrQxbCBgZrqXwqoKPQ+z8hguNlV+eujewBVs5AnKeUW9l2ELAyHAthVcVfBxi5zdcaKz88oSAkeMJtmBbHQIyb4GAkeFaCq8q+DgDxm+4IGD88kQnK8cTbMG2OgRk3gIBI8O1FF4hYGTCBAEjw5W9gi3YyhGQ84x6K8MWAkaGaym8quAvv+FB4rNgcIidn7ChsfLDMckL2IKtHAE5z6i3MmwhYGS4lsKrCj5O4fUbLjRWfnlimkOOJ9iCbXUIyLwFAkaGaym8cvC\/\/b1\/Ix6B4QeH2PkJGwSMH44YgZHjCLZgW10CMm+DgJHhWgqvHPyv\/MPDtPb2RyBgPEYMAsYjzJgrsAVbOQJynlFvZdhCwMhwLYXXuIB59LMX0OL6E0qR9yJnEo2VXHTAFmzlCMh5Rr2VYQsBI8O1FF45+F\/YOUp8Dgw\/OIXXT9jQWPnhiGkOOY5gC7bVJSDzNggYGa6l8MrBX\/\/X36L+e56GgPEYMQgYjzAxhSQHE2zBtmoEZF4EASPDtRReOfgf+sLdNPLQoSi\/mELyEzYIGD8cMUogxxFswba6BGTeBgEjw7UUXjn451zzNXrgqZeitS8sYPBUTgACpnKGaR7AFmzlCMh5Rr2VYQsBI8O1FF4hYGTChMZKhit7BVuwlSMg5xn1VoYtBIwM11J45eCf9Im\/xSm8nqOFxsozUM0d2IKtHAE5z6i3MmwhYGS4lsIrB\/+lDw9GecU1Av5ChsbKH8u4J7AFWzkCcp5Rb2XYQsDIcPXmdXp6mrq6umhsbCzy2draSn19fVRXV5f4jr1799K6deuiz5qbm2lwcJDq6+sTbZec80F6+ff7o886zl9Et3Wc7S3f89kRGiu56IMt2MoRkPOMeivDFgJGhqsXr0eOHKHe3l5qaWmhtrY2Uj83NDRQT0\/PnHeMj49TZ2cnbdmyhVauXEk7d+6k0dHRVMGjCxicAeMlZJETNFb+WGIERo4l2IJt9QjIvAkCRoarmNcsUcKf7d+\/P1HcJGVo8Qc\/RK9e1B19xKMvPAqDp3ICEDCVM0zzALZgK0dAzjPqrQxbCBgZrmJe0wRMfLTGJAONv\/df6bUVn4hMMQJjQszMBo2VGScXK7B1oWaWBmzNOLlYga0Ltfw0EDD5jApjodbDtLe3R1NK+qMEzJo1a+iOO+6I1szkrYFp+IO\/oNfPWhu5OfGBAbp\/5+2FKWuZMzI5OUmNjY1lLkJh8w62cqEBW7CVI+DH8+rVq+c4mpiY8OO8QF6OO3r06NEC5afirCiBwo6SFvGqzw8ePDizcLe\/v5+mpqZS18DoAgan8FYcohkH+GvLH8u4J7AFWzkCcp5Rb2XYYgRGhqtXr3nihV+WNIXEi3q7u7tpYGCAmpqa5uRp0R\/eQD9bfGH0++mbLvWa5\/nsDI2VXPTBFmzlCMh5Rr2VYQsBI8PVm9e8nUf6i3jEZcmSJTPTSyxgbrzxRtq6dWviVup3feyL9MYp78M1At6idcwRGivPQDV3YAu2cgTkPKPeyrCFgJHh6s1r3jSQ\/iI+A4bt1dkv\/P\/8JG255t9DwHgL0yxHaKxkuEIcynEFW7CVJSDjHQJGhqsXr\/FD7JRTtTiXD7Pjc2I6Ojqic1\/40Q+yyzv07pQ\/+yr98jdOwSm8XqL1lhMIGM9AMQIjBxRswbYqBGReAgEjw7UUXuuvvi\/KJ64R8BsuCBi\/PHVvYAu2cgTkPKPeyrCFgJHhWgqvSsDgGgG\/4UJj5ZcnBIwcT7AF2+oQkHkLBIwM11J4VdWg1MwAAAxQSURBVAKmZ80Z1LNmSSnyXIZMQsDIRQlswVaOgJxn1FsZthAwMlxL4RUCRiZMaKxkuLJXsAVbOQJynlFvZdhCwMhwLYVXJWBwD5LfcKGx8ssT0xxyPMEWbKtDQOYtEDAyXEvhVQkY3IPkN1wQMH55opOV4wm2YFsdAjJvgYCR4VoKr0rA4BoBv+GCgPHLE52sHE+wBdvqEJB5CwSMDNdSeIWAkQkTBIwMV\/YKtmArR0DOM+qtDFsIGBmupfCqBAzuQfIbLjRWfnlilECOJ9iCbXUIyLwFAkaGaym8soBZXH8C8RQSHn8EIGD8sYx7AluwlSMg5xn1VoYtBIwM11J4hYCRCRMaKxmumEKS4wq2YCtLQMY7BIwM11J4ZQGDawT8hwoCxj9T5RFswVaOgJxn1FsZthAwMlxL4RUCRiZMaKxkuGKUQI4r2IKtLAEZ7xAwMlxL4ZUFDO5B8h8qCBj\/TDECI8cUbMFWnoDMGyBgZLiWwisLGNyD5D9UEDD+maKTlWMKtmArT0DmDRAwMlxL4RUCRiZMEDAyXDHNIccVbMFWloCMdwgYGa6l8Pquj32Rbv70h6NpJDz+CEDA+GMZ9wS2YCtHQM4z6q0MWwgYGa6l8Bpq8GsNH42VXATAFmzlCMh5Rr2VYRtqH3bc0aNHj8ogC8drqMGvdYTQWMlFAGzBVo6AnGfUWxm2ofZhEDAG9SXU4BsUXdQEjZUcXrAFWzkCcp5Rb2XYhtqHBSNgpqenqauri8bGxqIa0NraSn19fVRXV5dZI8bHx6m7u5sGBgaoqakp0TbU4Mt8Vcy9orEyZ2VrCba2xMztwdacla0l2NoSM7MPtQ8LQsAcOXKEent7qaWlhdra2kj93NDQQD09PakRVnb79u2joaEhCBiz74I3q1C\/VN4AVeAIbCuAl5MUbMFWjoCM51DrbBACJinkO3fupNHR0cxRmL1791J\/f3+UHCMwMl+cLK+hfqmqT3LuG8FWLgpgC7ZyBGQ8h1pn562A4Smn6667jjo6OiIRAwEj88WBgKk+V35jqA1WbWjOfivYykUBbGXYhso1SAGj1sO0t7dHU0ppIzT8+xUrVhitgZGpVvAKAiAAAiAAAvIEJiYm5F9S5TcEJ2DUuhbmmLaIlxfubt++nTZt2kSTk5O5AqbKMcHrQAAEQAAEQAAEcggEJWBMxAvz4CmjVatW0cqVK8lkFxJqEQiAAAiAAAiAQLEIBCNgTHcexbdb6+EYHh6ORA0eEAABEAABEACBYhMIRsDwqMrU1JTR2S96SDACU+wKityBAAiAAAiAQBKBIARM2qhKc3MzDQ4ORofZ8TkxvOMoPsICAYMvBgiAAAiAAAiUj0AQAqZ82JFjEAABEAABEACBSghAwFRCD2lBAARAAARAAARqQgACJgM7r6vZtm1bZIEFvm71k6foOjs7o\/VJefdT6bz5Gois6x3cchNWKhu2quTxazfCIuKnNDZc49PXaCeyY2DDVrdFe1BZ3Vbf+6RlFJV5rm1qCJgU\/uqaAV5D88QTT0Rbr\/n\/6+vraxuxEr1d7yzXrl07676qeDHiVz\/wzzt27ADzlHjbsNVdMNeNGzfS5s2bUw95LFEV855VG67xnY9YT5cdDhu2ShjyXXa8bhHtgXtVV9x3794d3B\/iEDAp9ULdkcRfoFDVq\/tXwixlvEFnUTgyMmK0UwydQf5fsvot6iZsuVO45ppr6KWXXqKsU6rNohumlU2dZdsbb7yRtm7dij9sDKqDLVu9fqM9MACcYKJGsc477zw6ePBgdLlxSEeFQMAkBD3tdmt127VbVZp\/qfRRLB65iv+cRQQNVnZ9cWHLovz888+nb3zjGzM3t8+\/WumPq8mFseD7FgGbOps0ApN3OS9YzyXw7LPPRr\/knbhdXV0QMPOhkiSNuHDjv2TJEgy7W1SA+KiAzV+sruf6WGSv1Ka2bNX1GVdffXV0iSnEeHL4bbiygNm\/f3\/kCGvl8r9ONmzZmz71sX79+qjzxeNGIC4I3bwULxVGYDJGYPQFTxAw9pXXtsFSb+CO4ZZbbsEi3gzkNmy5I\/jCF75AH\/\/4x6mxsTFzLZJ9lMNKYcNVrSdSC3c57YYNG1BvU6qEDVs19bFly5ZoysNm9DasGumnNBAwfjiWwgumkPyEyWbIGOLFjrkNW7a9\/\/77o79gsQspm7MN1\/gUEtiCrd23uHrWEDDVY12IN+kjLljE6xaS+JRR3kJT7DQw52zDVt+err8Bw\/JzedtwjddntBPZ9deGLcSheVtgYgkBY0IpIBtso648mDbbJjH8bsfbhq3uGaME2ZxtuMY7BUxz+GObNIWE6Tm7NkK3hoBxZ1falDjIrvLQZR1cpRZB8tRG2igBDgZLj4EpWwgYu3psw1U\/yA6HreVztmHLgnDdunWRU7DNZ5tlAQFTGT+kBgEQAAEQAAEQAAFvBLALyRtKOAIBEAABEAABEKgWAQiYapHGe0AABEAABEAABLwRgIDxhhKOQAAEQAAEQAAEqkUAAqZapPEeEAABEAABEAABbwQgYLyhhCMQAAEQAAEQAIFqEYCAqRZpvAcEQAAEQAAEQMAbAQgYbyjhCAT8E3jqqado4cKFxLd5mzx83sOLL75IS5cuNTF3slFn9jQ3N9Pg4KBx3spyw7gqX7XOHqn2+5yCjkQgUEACEDAFDAqyBAJMwPZk12ocVlWJCKkkbTVrBAsKfqp5+3FZ2FQzDngXCOQRgIDJI4TPQaBGBIooYGzzpKMrSycNAVOjCo\/XgoAlAQgYS2AwBwGfBPSj6NmvmpZ54oknZo5R59+rKxXiVy6oaY6TTz6Zurq6aGxsLMqeuqhR3e2ze\/fu6Pcm0z76O\/RpFL76YePGjTPF37x5M7W1tc3BkWanBMzatWvp+uuvj9LFp2n04+OVY\/UeZnXNNdfQJZdcEqVXZXnhhReos7OTpqamoiSf+9znaNeuXTQwMEBNTU3R79LKlBTLuIDhn1955ZXoP8Ux6yLM+EWE\/I6k35VR3Pms+\/AFApUSgICplCDSg4AjgaSLFfXOMz7akXZDL7++r6+P2B+LGJ76WLlyJSlx1N7ePiM0sm78VvlR\/urq6qKO95ZbbqGhoaFIDOSNwMTt9Uv5WGSx0DjvvPOi\/Cr\/O3bsiNbSsBDp7u6eJTx0f0qkLV68eCZ9vIzq58OHD0d5bmxspN7e3kgoqSmhvItDkwTMtm3bSBdSzFnnqlcBCBjHLwSSgYAlAQgYS2AwBwFfBJIEhu47TyzE\/7KPCxhOPzIyMtPZs33WbdRJUzxx+6w85d10Hb9hmPOTN62kf64ETFyQjY6OziqjLlD4HTfeeCNt3bp11mLjrGmiJAHDoztKdLHPLA4QML6+IfADAtkEIGBQQ0CghgT06Zb49E5aJxmfZmltbU0cgYlP5ejFTJr+SXuffmt4Vsedt4g4Sawk\/S4+rRafJlMjTFyeJCGi++RRHXWjcTzMadNASQKG0+qLerOEFwRMDb9QePW8IgABM6\/CjcIWlYDeaevrYLgzVVuVlSCJr0tRIxDxERhOGx85yCp\/mjjJmtbS\/VUqYNiXWsuiBFbSCIyNgHn44YdJTVGZbkWHgCnqtwT5AoHZBCBgUCNAoEAEdBGgRhhYwPB6EV7L0dLSMmvhrC5S4gIma71LUpGrMYUUX+Oiv5PFRtZ0kJpC0gVM0miHPoXEIzAbNmyYWcNjEmqJKaQ8MZk3lWaSb9iAwHwjAAEz3yKO8haGQNIaGH0URF\/UmrQYVY3IqCkkLpgucpR\/XtCrpj+S1qEoIL4W8eojHvq6mBUrVsxZpBsXMHpalVfOHy\/ITRIwpot42Ydaw5K39qjSRbxqik\/tHFPl0BcvxyshBExhvpbISIkIQMCUKFjIangEVOemtgDr00P6FmieUrnssstmbZVm4XL55ZfTtddeOzPCEBc1alRGba9mgqpjTaOZteXYdGFx0nZrkzUw8Xdv2bIlWufCC3dV+fURGC5DnGF8G3V8KzmnSdsCrka9+F8l+pK2Uevpk8qlrz\/iOC1fvpweffTRSETFhaYqQ3x0KrzajhKBgF8CEDB+ecIbCIBAjQmwoEjaeWSaLZM1MKa+TO0wAmNKCnYg8BYBCBjUBhAAgdISiK\/zUaMt+rkvtoWDgLElBnsQqA0BCJjacMdbQQAEPBGIn06cdUquySvjlyveeeedUTKpu5FwmaNJVGADAnMJ\/H84n0ODlpBOBwAAAABJRU5ErkJggg==","height":337,"width":560}}
%---
%[output:3da50f8a]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"   440"}}
%---
%[output:9479f5e4]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.030000000000000"}}
%---
%[output:2957f2e8]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_JH","value":"   0.075000000000000"}}
%---
%[output:1f6650e6]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber","value":"     6.215564738292011e-09"}}
%---
%[output:42540868]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rsnubber","value":"     1.608864265927978e+03"}}
%---
