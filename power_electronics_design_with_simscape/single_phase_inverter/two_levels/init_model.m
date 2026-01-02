%[text] ## General Settings
%[text] ### Simulink model initialization
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
simlength = 2;
transmission_delay = 125e-6*2;
s=tf('s');

rpi_enable = 0; % use RPI otherwise DQ PI
system_identification_enable = 0;
%[text] ### Voltage application
application400 = 1;
application690 = 0;
application480 = 0;
%[text] ### PWM and sampling time and data length storage
fPWM = 20e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;

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
tc = ts_inv/100;

z_dab = tf('z',ts_dab);
z_afe = tf('z',ts_afe);
z_inv = tf('z',ts_inv);

t_misura = simlength;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

dead_time_INV = 1e-6;
delay_pwm = 0;
%[text] ## Grid Emulator Settings
grid_emulator;
%[text] ## Grid Voltage Rectifier
%[text] ### Nominal DClink voltage seting
if (application690 == 1)
    Vdc_nom = 1070; % DClink voltage reference
elseif (application480 == 1)
    Vdc_nom = 750; % DClink voltage reference
else
    Vdc_nom = 660; % DClink voltage reference
end
%[text] ### Grid Filter
f5 = f_grid*5;
f7 = f_grid*7;
omega5 = 2*pi*f5;
omega7 = 2*pi*f7;

L5 = 1e-3;
C5 = 1/(omega5^2)/L5;
L7 = 1e-3;
C7 = 1/(omega7^2)/L7;
%[text] ### DClink
Vdc_ref = Vdc_nom; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
CFi_dc1 = 1400e-6*4;
RCFi_dc1 = 1e-3;
%[text] ### Load Transformer Parameters
m1_load_trafo = 50;
m2_load_trafo = 1;
m12_load_trafo = m1_load_trafo/m2_load_trafo;

ftr_nom_load_trafo = 50;
I0rms_load_trafo = 5;
V1rms_load_trafo = 330;
I1rms_load_trafo = 600;
V2rms_load_trafo = V1rms_load_trafo/m12_load_trafo %[output:4e4d9ab0]
I2rms_load_trafo = I1rms_load_trafo*m12_load_trafo %[output:617b30e4]
lm_load_trafo = V1rms_load_trafo/I0rms_load_trafo/2/pi/ftr_nom_load_trafo;
rfe_load_trafo = 2e3;
rd1_load_trafo = 1e-3;
ld1_load_trafo = 100e-6; % for 400Hz output 
% ld1_load_trafo = 400e-6; % for 50Hz output
rd2_load_trafo = rd1_load_trafo/m12_load_trafo^2;
ld2_load_trafo = ld1_load_trafo/m12_load_trafo^2;
%[text] ### Current sensor endscale, and quantization
Pnom = 200e3;
margin_factor = 1.25;
adc12_quantization = 1/2^12;
adc11_quantization = 1/2^11;
adc16_quantization = 1/2^16;
adc15_quantization = 1/2^15;
Vdc_FS = Vdc_nom * margin_factor;
Idc_FS = Pnom/Vdc_nom * margin_factor;
Vac_FS = V1rms_load_trafo*sqrt(2) %[output:45db0fb3]
Iac_FS = I1rms_load_trafo*sqrt(2) %[output:19be7e26]
%[text] ### Single phase inverter control
iph_grid_pu_ref_1 = 1/3;
iph_grid_pu_ref_2 = 1/3.;
iph_grid_pu_ref_3 = 1/3;
time_step_ref_1 = 0.025;
time_step_ref_2 = 0.5;
time_step_ref_3 = 1;
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
kp_inv = 0.25;
ki_inv = 45;
%[text] #### Phase shift filter for Q component derivation at 50Hz and 80Hz
if system_identification_enable
    freq = 300;
else
    freq = 50;
end
omega_set = 2*pi*freq;

flt_dq = 2/(s/omega_set+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
figure; bode(flt_dq_d, options); grid on %[output:647a9dbc]
[num, den]=tfdata(flt_dq_d,'v') %[output:97e357dd] %[output:9d1224cc]

%[text] #### Resonant PI
kp_rpi = 2;
ki_rpi = 45;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta*omega_set] %[output:26f707e8]
Aresd_nom = eye(2) + Ares_nom*ts_inv %[output:2214b3d6]
a11d = 1 %[output:6e27c0a7]
a12d = ts_inv %[output:7996ad92]
a21d = -omega_set^2*ts_inv %[output:9ad8d2a8]
a22d = 1 -2*delta*omega_set*ts_inv %[output:637faca4]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:61f9eea3]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:306c88de]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:23fb1bfd]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:11c23dfc]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:4b099645]
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
freq_pll = 50;
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:00aa870f]

freq_filter = freq_pll;
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
figure;  %[output:2e5b507a]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:2e5b507a]
xlabel('state of charge [p.u.]'); %[output:2e5b507a]
ylabel('open circuit voltage [V]'); %[output:2e5b507a]
title('open circuit voltage(state of charge)'); %[output:2e5b507a]
grid on %[output:2e5b507a]

%[text] ## System Identification
%[text] ### Normalization
Ibez = I1rms_load_trafo*sqrt(2);
Ubez = V1rms_load_trafo*sqrt(2);
Xbez = Ubez/Ibez;
%[text] ## IGBT, MOSFET, DIODE and snubber data
%[text] ### Diode rectifier
Vf_diode_rectifier = 0.35;
Rdon_diode_rectifier = 3.5e-3;
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:71a97270] %[output:5cd9d49f] %[output:60ed39a0]
%[text] #### DEVICES settings
%[text] ### SKM1700MB20R4S2I4
danfoss_SKM1700MB20R4S2I4;
mosfet.inv.Vth = Vth;                                            % [V]
mosfet.inv.Rds_on = Rds_on;                                      % [Ohm]
mosfet.inv.g_fs = g_fs;                                          % [A/V]
mosfet.inv.Vdon_diode = Vdon_diode;                              % [V]
mosfet.inv.Vgamma = Vgamma;                                      % [V]
mosfet.inv.Rdon_diode = Rdon_diode;                              % [Ohm]
mosfet.inv.Eon = Eon;                                            % [J] @ Tj = 125°C
mosfet.inv.Eoff = Eoff;                                          % [J] @ Tj = 125°C
mosfet.inv.Eerr = Eerr;                                          % [J] @ Tj = 125°C
mosfet.inv.Voff_sw_losses = Voff_sw_losses;                      % [V]
mosfet.inv.Ion_sw_losses = Ion_sw_losses;                        % [A]
mosfet.inv.JunctionTermalMass = JunctionTermalMass;              % [J/K]
mosfet.inv.Rtim = Rtim;                                          % [K/W]
mosfet.inv.Rth_mosfet_JC = Rth_mosfet_JC;                        % [K/W]
mosfet.inv.Rth_mosfet_CH = Rth_mosfet_CH;                        % [K/W]
mosfet.inv.Rth_mosfet_JH = Rth_mosfet_JH;                        % [K/W]
mosfet.inv.Lstray_module = Lstray_module;                        % [H]
mosfet.inv.Lstray_d = Lstray_d;                                  % [H]
mosfet.inv.RLd = RLd;                                            % [Ohm]
mosfet.inv.Lstray_s = Lstray_s;                                  % [H]
mosfet.inv.RLs = RLs;                                            % [Ohm]
mosfet.inv.Ciss = Ciss;                                          % [F]
mosfet.inv.Coss = Coss;                                          % [F]
mosfet.inv.Crss = Crss;                                          % [F]
mosfet.inv.Cgs = Cgs;                                            % [F]
mosfet.inv.Cgd = Cgd;                                            % [F]
mosfet.inv.Cds = Cds;                                            % [F]
mosfet.inv.Rgate_internal = Rgate_internal;                      % [Ohm]
mosfet.inv.Irr = Irr;                                            % [A]
mosfet.inv.Csnubber = Csnubber;                                  % [F]
mosfet.inv.Rsnubber = Rsnubber;                                  % [Ohm]
% ------------------------------------------------------------

%[text] ### Load
uload = 3;
rload = uload/I2rms_load_trafo;
lload = 1e-6/m12_load_trafo^2;

% rload = 0.86/m12_load_trafo^2;
% lload = 3e-3/m12_load_trafo^2;
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
%   data: {"layout":"onright","rightPanelPercent":5.1}
%---
%[output:4e4d9ab0]
%   data: {"dataType":"textualVariable","outputData":{"name":"V2rms_load_trafo","value":"   6.600000000000000"}}
%---
%[output:617b30e4]
%   data: {"dataType":"textualVariable","outputData":{"name":"I2rms_load_trafo","value":"       30000"}}
%---
%[output:45db0fb3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     4.666904755831214e+02"}}
%---
%[output:19be7e26]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     8.485281374238571e+02"}}
%---
%[output:647a9dbc]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAACsAAAAeCAYAAAC16ufeAAAAAXNSR0IArs4c6QAAAPdJREFUWEftlrEJhDAYRj9LW3ewFixdRFuxcAALgwOIjSPYa21p7xI2IuIKlpF4RDg5kOOI5uBPbeD9z5cQg3PO8SfLIFhFf4rMKhILMktmAcpAVQX3ml3XFYwxtG37NlAcx0jT9HLIW28DCet5Hnzfv4Q7f6ANbNM06Pse0zQhCAK4roswDLEsCxzHQVVVz2cgQbquQ13XO5RYSZIgyzLYtg0xyDiOz8B+ykCaLYoC8zwfVmUKYijtMpCweZ6jLEtYlnWkqyWsOIhRFO3tioMorWsJa5omhmF49oB9fVedNtxqlmB\/NaBqP2VAZunxraoBMktmXwY2fh4CJc6d3mwAAAAASUVORK5CYII=","height":0,"width":0}}
%---
%[output:97e357dd]
%   data: {"dataType":"matrix","outputData":{"columns":3,"exponent":"-3","name":"num","rows":1,"type":"double","value":[["0","0.244171410261656","0.241627792504512"]]}}
%---
%[output:9d1224cc]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000","-1.968829526703427","0.969072426304811"]]}}
%---
%[output:26f707e8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000050000000000"],["-4.934802200544680","0.999214601836603"]]}}
%---
%[output:6e27c0a7]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:7996ad92]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     5.000000000000000e-05"}}
%---
%[output:9ad8d2a8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"  -4.934802200544680"}}
%---
%[output:637faca4]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.999214601836603"}}
%---
%[output:61f9eea3]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.076483869223994"],["18.982392004991411"]]}}
%---
%[output:306c88de]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:23fb1bfd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:11c23dfc]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000050000000000"],["-4.934802200544680","0.999214601836603"]]}}
%---
%[output:4b099645]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.077754418176347"],["13.583043056999228"]]}}
%---
%[output:00aa870f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.018721899663332"],["0.977712707506129"]]}}
%---
%[output:2e5b507a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAACsAAAAeCAYAAAC16ufeAAAAAXNSR0IArs4c6QAAAuhJREFUWEftl7+PaVEQx0e5LdGotqGViEJ0WyipJEInIiq3eFhso9v18xVUCtFxNQr+gBedVmsbZLONUFLel5lk5FzrcZ3L5iXvnWaXe87M53zPfOceFk3TNJAYlUoFptMpdDod2Gw2EI\/HIZ1OQyQSkYh2eQnmsyiKQrDlchkeHh5AhLBarfS53W5TtGAw+GVerVaDXC4Hs9mM5qRSKUgmk5BIJHTf5fN5ej4YDKBYLB7mYuxerwc+nw+2261uHX\/\/\/v5OYlhUVdVUVSWFcGASXIjBRfBzz46VxXWPj4+kMsNhYpvNRklx0xyfYd1uNxQKBWJA4UajEbRaLeh2u3Ry2WwWLPP5XMMA9XqdJsZiMdqpy+XSgbMqHGA4HJ4tg1MqLZdLUpUVwzLifLwRLiVWE7lwHea17HY7DXfkcDgI9lwdokqXYEOhECk0Ho8JShSAk6JaTqeTcjEszzuu3re3N1gsFhTPggZjCLvdfigBVoZLwqiyHo9HZzYR6BplRWjmI1iW\/PPz83BEOFmmZgOBwKF8FEXRqSzW7PGzSzWLJ0Cw+\/2egq5WKzIadgEeYjdAp7OrxY1gF+GjR\/N4vV4olUoUAiHW6\/WhrZnqBrJ99nJn\/DqDT5BNJHoAa\/jcoD77nbBcWty38TMayMiLBDdqClYsEaNJZU6E10jDosv7\/T418I+PD3h+foZqtUot6V5DGlYEwiN6fX2FRqOhM+etoU3Dcikcl4H1xy\/TrNufT7oYpmExGrc+v99vyCyyu7gJLLsc\/3IflgU6t04aFg02mUwIjl\/N+D++nu81pGGPe+Zf3br+pB5fXK5p+KdisQ+i0ejhtEwpe5wEyyGTycDLyws9km1n4l2Y774Y76awqCq2MrwM8eVGVMZILaOizWYTwuEw\/ToQfXBzWH6rIRjexGTb2SnT\/tuwZsuAS+Xuyt7KYAh8d1hMIrYu0clGzCXO+RbYa6GumX9Tg12TWGbuf1gZ1Yys+Q2Sr+AqXKdbewAAAABJRU5ErkJggg==","height":0,"width":0}}
%---
%[output:71a97270]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:5cd9d49f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:60ed39a0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
