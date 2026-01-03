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
fPWM = 10e3;
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
    frequency_set = 300;
else
    frequency_set = 80;
end
omega_set = 2*pi*frequency_set;

a = 1 + 2*pi*frequency_set*ts_inv;
b = 1 - 2*pi*frequency_set*ts_inv;
phase_shift_filter_gain = 0.985-0.085/350*(frequency_set-50);
phase_shit_filter_d = phase_shift_filter_gain * (1-a*z_inv^-1)/(1-b*z_inv^-1);
flt_dq = 2/(s/omega_set+1)^2; %[output:851d9a6b]
flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d, options); grid on 
[num, den] = tfdata(flt_dq_d,'v')
figure; bode(phase_shit_filter_d,flt_dq_d, options); grid on

%[text] #### Resonant PI
kp_rpi = 0.25;
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
%   data: {"layout":"onright","rightPanelPercent":35.9}
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
%[output:851d9a6b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAYYAAADrCAYAAABtnTHVAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnX1sH0eZx59C4eKqtLFbBHXzYtMmtJx0aYvKGadHqBIIqLX\/KBLxL+VkohCstLkgNVH8UqQ0kNpxIEgNKcEkxopU7CAKUmN0KDRpiJSGih5prRMKJG3emrogmhdCj1Rcr5yecefX8Xr395vdnd3Z2f2uFCWxZ2dnPvPM891nZnbmin\/84x\/\/IFwgAAIgAAIg8A6BKyAMsAUQAAEQAAGVAIQB9gACIAACIDCJAIQBBgECIAACIABhgA2AAAiAAAgEE0DEAOsAARAAARBAxAAbAAEQAAEQQMQAGwABEAABENAkgKEkTVBIBgIgAAJFIQBhKEpLo54gAAIgoEkAwqAJCslAAARAoCgEIAxFaWnUEwRAAAQ0CUAYNEEhGQiAAAgUhQCEoSgtnaN6Hj9+nJYtW0bj4+PlWrW0tNCmTZuopqamak0vX75MXV1dVCqVqKmpqWr65557jpYuXTopXUdHB3V2dlLYvKo+DAlAIAMEIAwZaAQUIRwBFoZ169bR5s2bac6cOaGdc1hnzsLQ399Pg4ODVFdXV35efX29EAdcIJA3AhCGvLVoAepTTRjUN3z5Zs9Y2LkPDAzQggULBCX+HUcMP\/7xj6m7u7v8M6+z9woDJ+Qy9Pb20saNG4VAcfQxb948EYmMjo6KvGQUw\/+WP+efXbp0iXp6eujIkSN0+PBhOnPmDLW1tdHs2bMnRSbDw8M0d+5cWrNmDX3qU5+ib37zm8Ri9O1vf1vUZWxsjPr6+mjJkiUFaHVUMU0CEIY0aeNZRgj4DSVJB6mKxowZM4RDbm5uFk5XvvWfO3dODEWxg+VrZGREDENJB+4dYvIThvPnzwuH\/dBDD9HOnTuFMMycOVPkceONN5L8vSoA\/Ax25mvXrqWhoSEhDLt37y5HIvwcObTFYnXq1ClasWIFLV++XPycBYvrwOk4enn66aeFsOgOoRmBj0wKQQDCUIhmzlclgyIGKQDS0fN8g3SwkoB3XuD06dPlaEGmUaMM\/pmuMLDzlsNUHDXw2\/327duFcHDZ+M0+SDDk3Ig32pHCwOWW0Q0LBv+f06p1zVcrozY2CUAYbNLHsyMR8AoDZyIFgIeJwgqDdLRBhdEdSuL7eZJaHQKSEUU1YZDRCv\/NEcCePXsmRQwQhkimgpsiEoAwRASH2+wRqBQx3HHHHeWJad2hJDm0o6ZXx+0rTT6vXr26vMJJHZbyDhnJIZ+gn6vDWHKugiMORAz27KzIT4YwFLn1Ha17teWqSUw+6yxX5Yling9g58\/p\/\/rXv06ZlPabfJZzBHISnAWB83nxxReFyK1atUoMHWEoyVGDdbDYEAYHGw1FdpeA37CUu7VByfNKAMKQ15ZFvTJDQF0Oy4XiOQidD+syUwEUpHAEIAyFa3JUGARAAAQqE4AwwEJAAARAAAQmEYAwwCBAAARAAAQgDLABEAABEACBYAKIGGAdIAACIAACiBhgAyAAAiAAAogYYAMgAAIgAAKaBDCUpAkKyUAABECgKAQgDEVpadQTBEAABDQJQBg0QSEZCIAACBSFAIShKC2NeoIACICAJgEIgyYoJAMBEACBohCAMBSlpVFPEAABENAkAGHQBIVkIAACIFAUAhCGorQ06gkCIAACmgQgDJqgkAwEQAAEikIAwlCUlkY9QQAEQECTgFVh8J5sxWXu6+sj9SB2zXogGQiAAAiAgCECVoRBHtbuJwJSLHD8oaEWRjYgAAIgEJJA6sJw\/vx5Gh0dpfb29opF3bVrV9U0IeuK5CAAAiAAAhoEUhcGjTIlnuQjH\/lI4s\/AA0AABMwTuPvuu0WmBw4cMJ+5hRxPnDhh4anVH2lFGI4fP07Lli2j8fFxMafAV3d3N9XX19PQ0BDNmTOnesljpGBhyGqDxKiW9q1Frz+DyhuDkef\/KNr\/zPk33\/n7svj7lXf+L353YeJ3ajpto8lQwu9+fJzuv\/9+IyWybQe2nx8EMXVhuHz5MnV1dVGpVKK5c+fS8uXLqa2tTUw48\/zC4cOHadOmTVRTU2Ok4f0yyWpjJFZhT8YnT56kxsbGtB6XyedknYF08M++fFHwO\/TShbKjP\/TOz7xgZ9VNK\/9oVu3Ev2eqP6t7t09x2qbrLztnBz\/60Y9EvUwJg207yKovSl0YeI5hw4YNtH79eqqrq6P+\/n5asGABNTU1kfd3QR6F07GgjI2NUUtLyxQhkeLDcxl8dXR0UGdnZzm7rDZGWh7UdmdIq56VnpMlBiwC\/OeVC28KAeC3fNX5S4c\/\/6bpokp33VxLpTs\/HBtjlhjoVgbCoEsqXjonhYHFpKGhgVpbW8vRBwuLvDjyOHXqlBADKSIyKuE0EAZEDDadIosARwJeEWAB4Dd9fstn5z+zlv+eEIMkLpsMotYHwhCVXLj7nBMG6ejZ6bMYqCIQVHUpJPL7CAgDhCFtpyjFYOQ3r5WjARYCjgIeL90artcaSp02AxPFhjCYoFg9DyvCIIeB\/Io3b948GhwcFMNMfhcLw5o1a6inp0dMUlebl\/CmR8RA5KJDqG7K4VKkxYAnhaUYSCEwNRQUrsZTU6fFIG451fshDCZpBueVujDErVYYYVAnutWhJrlcdf\/+\/XGL4+T9Z8+epRkzZjhZdlOFTpLB+KW36OdH36CB31yk+muupI\/fOE38abn1alPFN5JPkgyMFNAnk3379omfLlq0yMgjbDJYuHChqEMWV0imLgzqxHHUiIEjjmpDSX6RgnwehpIwlJTE2zIPF\/XvPUkcJXB0ULrzBupc3GDEgSWRSRIMkignIoakqU7NP3VhUIvgnR\/g\/\/NVba+kapPP1VY3QRggDCadIgsCiwGLAgtC5+JGI6uGknYHJhkkXVaZP4aS0iFtTRj8nHc1hy6RqFGHXIqq3rtjxw4aGBiYRFDdlwnCAGEw4RRdFQTZMUwwSMdNvfsUCEM6xK0Jgxz\/b25uLkcIHAnw19D4wC3ZxnfRIZgmEpcBi8JtG3\/txJBRELu4DEy3iU5+EAYdSvHTWBMGLrp3vqHaiqT41Z3IAREDIoaoTlGNEs5\/Z2LfHlevqAxs1hfCkA59q8KQThWnPgXCAGGI4hRZFFq\/94IwqG1ttyb68VkafSMKgzTKVekZEIZ0WiB1YcjCttsQBghDGKeoRgk8sZzllUZh3EYYBmHyTTIthCFJuu\/mnbow8KNtH9QDYYAw6DpFKQojz79GL379k+n0ypSeossgpeJoPQbCoIUpdiIrwiBLbetoTwgDhEHHKapDR3seuF1MNOfp0mGQtfpCGNJpEavCkE4VMcfgJeCiQzBtK9UY8HcJD44cFWKQR1FgntUYmGZuIj8IgwmK1fOAMFRnlLsULjoE041QiYEUhTzNJ\/jxc9EOIAyme4J\/fhCGdDhn6ikuOgTTAIMY9O89Jb5gzrsoIGKYsCjbfSGrw9oQBtMex4H8bHeGLCDyY1AkUciCU4xiB4gYolALf49VYVA\/cNu5cyc988wz1N7ejjOfw7djqDsgDP5vinUPHShEpCCNxUU7gDCE6uqRE1sTBnVLDD5tjY\/35GtkZARbYkRuTr0bXXQIejXTT6Uy4EiBl6NmfTdU\/drppXTRDiAMem0bN5U1YfBuesfCMHfu3EnnQcetXND9WR3XS6q+3nxddAim2UgGRZlo9uPnoh1AGEz3BP\/8rAmDX8Rw8OBBbKKXQru76BBMY2EGz71eI5akFmGiGcLgb0G2+0JWX1KtCQM3EzbRM+3u9PKz3Rn0Splsqq3\/+d\/0yL7XCysKTNdFO0DEkGy\/kLlbFYZ0qjj1KVlV6bR4uOgQTLMp2kQzIgZEDGH6UOrCEPdozzCVwxxDNjuDiTaMmoc8R6HjE9Opr+32qNnk4j4XXxAQMaRjeqkLg1qtqEd7xkWDiKGYeyXJvY\/m3zSd1jZNo8bGxrim5PT9EAb7w2lZ9UXWhCHO0Z5xe2NWGyNuvXTvd9Eh6NYtKJ26IR7vklpEBl42LjJAxBC3J+jdb00YcLSnXgMlkcpFhxCHA4vCqpGjdObCm+Wts4vGAHMM2RxWzepLqjVh4GbCqqQ47i76vUVziq2PvyBEQd0ltWgMIAwQhjAew6owhCmoybRZVWmTdayUV5Gcovyq2XsUZ5EYBNmCiwwwlJSOl7AmDEGrk+bNm0eDg4NUV1cXi4A8JY4z6evroyVLlpTzgzAUY\/I5SBTYEFx0irE6hM\/NLjKAMJi2Av\/8rAmDX3F4ldLs2bOpqakpVu1ZdNasWUM9PT0in97eXtqyZUtZbCAM+ReGaltduOgUY3UKCIMvPtt2kFVflClh8FupFKUzcLTQ398vIo+amhrq6uqiUqlUFpysNkaUuka5x3ZniFLmMPdUEwVEDBM0XbQDRAxhekL0tJkSBtWhxxlK4nzkLq2MhoWhubm5PJzEwsDX\/v37o5Nz+M6zZ8\/SjBkzHK5BcNHHL71FLbvOEn\/A9tV\/nR6YMM8MdBvWRQb79u0T1Vu0aJFuNSums8lg4cKFomwnTpwwUheTmVgThqA5Bu98QJTK6ghDFhsjSl2j3OPim6JOPdUP2B4v3Vrxlrwy0OEk07jIABFDmBaOntaaMEQvcvU7MZRUmZGLDqFaq3s\/YKuWPo8MqtXZ+3sXGUAYwrZytPTWhCHJL58x+VwsYQgrCkzHRacYrYsH3+UiAwiDaSvwzy91YZBfPI+OjvqWqKWlxcgJbupy1eHh4UkrnTD5nK9VSX4fsFXrPi46xWp1Cvt7FxlAGMK2crT0qQuDLKapFUhRqg1hyJcw3Lbx15O+ataxCRedok69wqRxkQGEIUwLR0+bujBIQVi9ejWtXbuWxsbGJpXe1AdulZBAGNwXBnX\/I3WrC92u4KJT1K2bbjoXGUAYdFs3XrrUhSFecc3cDWFwWxhYFPr3nqRnX75I3q0udC3ERaeoWzfddC4ygDDotm68dFaFgb907u7uRsQQrw1D3+2iQ5CVZFHgD9hGnn8tsihwXi4zCN3gATe4yADCYKr1K+djTRjkdwydnZ2xt8AIiwoRg5sRgylRgDBM9BgIg30GWfVFVoVhw4YNtH79+tgb5kEYwhFw0SH4nakQrtaTU7vIIE59\/e51kQEiBtNW4J+fNWHg4vBQEl\/qzqdpVDurKp1G3V18U1S\/U4gy0ZwXp2jaPiAMiBiCbMqaMCS97XalTgRhcGcoKQlRcFEcTYuCqwwQMSRhCVPztCYM6VTP\/ykQBjeEIcoXzbp25eLbsm7ddNO5yADCoNu68dJZE4agiIGrU19fT0NDQzRnzpx4tQu4G8KQfWHgQ3Z4Ser579ydiA246BRNg3CRAYTBtBVkdI7h1KlTxCuT1DkHPqyHt81+7LHHEqEAYciuMKgrj0p33kCdixsSsQEXnaJpEC4ygDCYtoKMCUOlTfT4q+itW7dCGBKygaw6hKTmEzD57G9IWbWDSmYPYUjIKXiytTaUJDfT42EjNWI4fPgwrVu3jp544onyz02jQMSQrYhBRgk8dHTXTdNpz4O3m27yKfm56BRNQ3GRAYTBtBVkLGLg4njnGXifpG3bttHmzZsnnbhmGgWEITvCoEYJSQ4deW3IRadouh+4yADCYNoKMigM6VRx6lMgDPaFQe53xNtbdC5uTGwuIcjGXHSKpvuLiwwgDKatIIPC0N\/fTwMDA5NKht1Vk294mw5BHTaaVTeNXvz6J5OvsM8TbDKwUuGcMIAwpGM91uYY1FPWjhw5QrwS6fTp06LWSX8JjYgh\/YjBKwgcJZTu\/HA6Vp4Tp2galoviCGEwbQUZixjUVUnHjh2jgwcP0ooVKyiN\/ZMgDOkIA4sBb4098pvX6NDLF4kjhDTnESp1IRedommX4CIDCINpK8iYMPCqJF6SymJw7tw5WrZsGY2PjxOGkpJv+CQdAovBRHTwmhAFvubfNF0Iwl03T0++cppPSJKBZhGsJ3ORAYQhHbOxNpTE1ePT26666irxhTOf0cwnuiX5xbNEiojBbMQgI4NDL10QZyXwZXP+QKfruOgUdeoVJo2LDCAMYVo4elqrwhC92PHuhDCYEYYHR47SK+ffLA8TzaqdRqVP3CAiBBaGLF8uOkXTPF1kAGEwbQUZG0qKUz31+4eWlhbatGkT1dTUlLOUH8+Njo6Kn3V0dEz6WA7CUF0Y5JDQKxfeJI4E+JIiICMCFoL5N9cKIcjSMJGObbnoFHXqFSaNiwwgDGFaOHra1COGSpvncTV05hh4mWtDQwO1trZSV1cXlUqlSafA8TkPcg8m+by2trbyaqeiC8OTh47S\/\/5TrbCaCQG4XHb8Zy5MzBHIS775swjMrJtGd91cSzNr+e\/szBdEMX8XnWKUela6x0UGEAbTVpCRiMErDMPDw6GO9vQeCaqKQBAyKSRyGWxWhYF3FA26pPP2\/p7f4r0XO3fp9CuZker0OR07\/ll1Ne+sHrK3lDQN08+qDaRRd\/kMFxmYFgbbDGw\/P8jeUo8Y1IL4bYkxODhY8ahP9fsHnrRmYeD9lbzDSfI53vT887qHDqTZ\/yo+6z1\/e127LO\/527kpaf3ul+nk795\/5lntZyAhCGSZwN13T2zDfuBAdvpwXF4nTpyIm4Xx+60Kg7c2vDKJ3+4riUMYYZBzDd6hJuMUkSEIgAAI5IiAVWE4fvx4+fsFZlptIpl3YuUzGjZu3Cgmk5uamkTEoJ7pUClSyFG7oSogAAIgkBiB1IVBHT7SmWj2q3m1yWe\/sx4SI4iMQQAEQCBnBKwKgx9LHbFQxUUuRVXFYMeOHVM25+vr60t8D6ac2QaqAwIgUFACqQtDQTmj2iAAAiDgDAEIgzNNhYKCAAiAQDoEIAzpcMZTQAAEQMAZAhAGZ5oKBQUBEACBdAhAGNLhjKeAAAiAgDMEIAzONBUKCgIgAALpEIAwpMMZTwEBEAABZwhAGJxpKhQUBEAABNIhAGFIhzOeAgIgAALOEIAwONNUKCgIgAAIpEMAwpAOZzwFBEAABJwhAGFwpqlQUBAAARBIh0AhhSHooB6dQ3PUw3LU9N5DdKb9\/ql0WhBPAYECEcjbQT1ZPKSHzamQwjDj0\/9O3\/rWt6Z0J\/Ws46C+ph6x6T1WU\/dITZl30NGa\/Hv+XenOZI7XzOpxgmn6NzAgcpEBjvZMp5cUUhiy0iHkGc9SbKTQsMD4iZQqJPJ85s7FDaEtxcVD4ENXssoNYEDkIgPTwmCbQVZ8kbe7QBhMe5yE82MxYSFhEfETECke82+aTrPqanwjD9udIWFEWtmDAYSBDcW2HUAYtLprOomy2himas\/i8exLF6YIB4vGrNpp9M\/XEf3LTTckNlRlqh5J5mPbISRZN928XWSAiEG3deOlsx4x8JnN3d3dk2qR9GlreRcGr0nwsNSzL18Uw1McbRwbv0i\/ffVNkUxGGKU7byCOMu66eXo8i3Lkbhedomm0LjKAMJi2Av\/8rAnDc889R0uXLiU\/EZBiMTw8TE1NTcZJFE0YvADZIbz32hvEj0ee\/+M7f79WnteYmPie+H2UOQzjDZZAhi46RdMYXGQAYTBtBRkSBj6feXR0lNrb2yvWcteuXVPSsGicOnWKOjs7xb1SYGRGUmjUn3vFB8JwkhobG6ewn4goJqILHoo69PLFclSRt4jCRado2iW4yADCYNoKMiQMUavW399PAwMD1NHRURYGr1Bw3iw8a9asoZ6eHvGo3t5e2rJlC9XV1Yn\/Qxj8hcHbLn5CIecp9jx4e9RmzMR9LjpF0+BcZABhMG0FGRQGduDLly+nsbGxKaWrr6+noaEhmjNnjvgdC8Ds2bPp4MGDE0Mc70QMLBYNDQ20ZMmSch4cLfDPBwcHqaamhrq6uqhUKpWHpSAMesLgJxQ89KRGE3fdNJ3m31zr3JCTi07RtEtwkQGEwbQVZFAYpMNXh4ZYAPhiERgZGaHHHntsUsnZ4Uth8ArLvHnzhBgcO3ZM3Ltp0yaRloWhubm5LB4QhmjC4DUhuXSWxULOS7gyge2iUzTtElxkAGEwbQUZFAZ27Bs2bKD169eXh3nkz1avXk1bt26tKAzeKrGoHD58mO677z762c9+VlEY+N79+\/enQzljTzl79izNmDHDWKnGL71FPz\/6Bv3Xq2+K1U7111xJLbdcTV\/91+yucDLNwBjMFDNykcG+ffsEoUWLFhkhZZPBwoULRR2yuC2GtVVJDOTy5cvibZ6HjeTQkHTu69atoyeeeKL8c2kFasTgtQw5hLRy5Uravn07hpICuk6Sb4o8L8ERRP\/ekxOR3eJG8b2EXBZrpDcbyCRJBgaKl0oWLjJAxJCKadjfK8lvOGjbtm20efPmScM\/fsLgjTikaKxYsQKTzxXsJy2HcOili7Rq91Gx0omF4cWvfzIdq9Z4SloMNIpiLYmLDCAM6ZiL1YiBq6gKw86dO+mZZ54RS1TlpLMXgzdiUJelyjkGXn2k\/tz7PQTmGMzMMeiaKAsDRxByLkJGEbr3J5HORadomoOLDCAMpq3APz+rwiCHknhimCegFyxYIEopJ455RVESF4QhXWGQbZglgXDRKZruCy4ygDCYtoIMCoM6FLRjxw4hDHPnzp0yIW0aBYTBjjAECcS2tltT34rDRadouh+4yADCYNoKMigMfhEDf6cwPj4uVhQhYkjGCLLiEDiCWDVyVHxhzd9DbCvdmtokdVYYJNPCerm6yADCoNe2cVNZHUryzjHw\/9V5griVC7ofEYPdiMHbLuokNc8\/pLE\/k4tO0XR\/cJEBhMG0FWQwYkinilOfAmHIljDIFuIP5niSmlcwJT285KJTNN1fXGQAYTBtBRkShkpbYaQRNUAYsikM3PbqdxA8vJTUnkwuOkXTLsFFBhAG01aQIWFQi+LdBE9uiaHufWQaBYQhu8KgTlDL+YckhpdcdIqm+4GLDCAMpq0gg8JQaUsMdZsM0yggDNkXBu\/wkunJaRedoul+4CIDCINpK8igMKirkmSEwB+wYVVSso3vmkNQVy+Zih5cY5CERbjIAMKQhCVMzROrktLhnKmnuOgQGKA6Ob3ngdtjLW11lYFJQ3KRAYTBpAUE52VdGMJW0zsnoU5kt7S0lL9\/wAluwWRddAim5x5cZhC2zwSld5EBhMFU61fOx4owRD3a0+8EN3lQT2tra\/lAHv56Gie45VMY\/OYeoqxcctEpmnYJLjKAMJi2gozNMcg3eu95zFxMjgq6u7tJ3fzO7wQ3GS3wlt1NTU3iPrnnEk5wy7cwcO34w7jW770Q6bsHF52iaZfgIgMIg2kryJgwyOJIEVCL5ycW5TdFzwluMjLg3VjDHNSTxcMx0mlyIhcdQhCbqBPTeWIQ1W5cZABhiNra4e6zMpQUroiTU3uP9owqDJwrTnCL0xLZunf06Bv0yL7X6eM3TqMf3PfhqoWzeXJX1cKllMBFBjjBLR3jcF4Yli9fLk55w1CSvsG4+KaoUzsZPZy58GbVLTXyykCHk0zjIgNEDGFaOHpap4WBq43J5\/CN76JDCFNLuay10jcPeWegw8tFBhAGnZaNn8Z5YVCXq3Z0dJTPiMYJbsHG4aJDCGvq6sS03zcPRWBQjZmLDCAM1VrVzO+tC0PYoz1NVBtbYrizJUac9q40tOSiU4zDwu9eFxlAGExbgX9+VoUBR3um08jep7joEOKQ8htaKhoDCIO\/Bdm2g6y+pFoVBhztGcfdRb\/XdmeIXvLod3qHlv7vL69RY2Nj9AxzcKeLdoCIIR3DsyoMONoznUYuesQg669+8\/Db\/2iAMJx0b0gRwpCOz7AqDFxF76E9ONoz+YZ38U3RJBU5tJTkQUAmy5tUXi7aAYQhKWuYnK91YZDbWKxYsYL4m4SxsbFJW2EkgSGr43pJ1DUvY8um2Tx56Ch99Wd\/jLSdhumy2MoPwmB\/F4Cs+iKrwsBDSY8++ii1t7fTkSNHxD5H9913H+3atYsefvhhqqmpSaTPZLUxEqmsT6YuOgTTbJjBe6+9geQpcee\/c7fpR2Q+PxftABFDOmZlVRi8k88NDQ30mc98hjZs2EA4wS05A3DRIZimoTIYef6P9ODIUTJ9SpzpMpvOz0U7gDCYtgL\/\/KwKg4wY7r33XhoYGKCenh5RSkQMyTa+iw7BNBEvg6ib8ZkuV5r5uWgHEIZ0LMSqMHAV5RfK\/NUyzzOom+IlhQBDSe6tRjFtC0FOsUgT0xAGzDEE9SvrwmC6w+vkB2GAMFRyikWJHiAMEIbMCoM8lU0tYJglq+qeSJyHPMsBR3sGS6SLDkFH8MOk0WGgnjG9re1Wuuvm6WEekfm0OgyyVgkMJaXTIlYjBp58lkNHvCpp9uzZdPr0aVHzJUuWaBHwngHNN6n58v97e3tpy5YtVFdXJ\/JExICIQdcp5jl60GWg1RFTSgRhSAe0dWGQK5COHTtGBw8eFPMMYVYlyW23VSHhaAFHeyJiqNSFwjpFdUuN0p03UOfihnR6aIJPCcsgwaJoZw1h0EYVK6FVYeBVSVu3bhVicO7cOVq2bBmNj4+T7lBS0FfTLDIjIyO0adMmAaerq4uam5vLUQhHDHzhBLdYtuP0zVFOLxu\/9Bb9\/OgbNPCbi1R\/zZX0yKLrxYlxrl5RGNiuK05wS6cFrAoDV5G\/dL7qqquIz2zmN\/21a9fS0NCQ+H\/YC2c+6xFz8U1Rr2b6qeIwUIeXXP72IQ4DfdJmUyJiMMszKDfrwmCymnIIaeXKlbR9+3YaHBwUX09zxFAqlcTxn3xhjgFzDKac4m0bf00sFC4KhCkGJvtwtbwgDNUImfm9dWHgt\/zu7u5JtQkzlKTOR\/C8Al\/q9xD8f0w+TzYWFx2CGXN\/NxeTDHj+YdXuo0Ig+DjR0p0fFnswZf0yySCtukIY0iFtVRjkHEFnZ2f5bT5stdVlqaqg4GjPYJIuOoSwdlEtfRIMeHnryPOvCYFgYcj6EtckGFTjHvf3EIa4BPXuty4MYVYg6VWpeioMJWEoKUmnqEYQLBAyiqhumemmSJIOnHmIAAAKYUlEQVRBUjWBMCRFdnK+VoWBi8JDSXzpfrdgAguEAcKQhlPkyIE36Ovfe1JEEFlb5poGAxP9Vc0DwmCaqH9+VoTBu8zUWzTdOYaoiCAMEIY0naIUCHWYKQsikSaDqH3Vex+EwRTJyvlYEYZ0qhb8FAgDhMGmU1S32rApEDYZRPUBEIao5MLdZ00Yjh8\/Xv6graWlRXyMltTBPF4kEAYIg22nKKOIZ1+6QIdeviiGmmbVTqP5N9em9lW1bQbhXNVEaghDFGrh77EiDPzFs\/ptgd+2FuGron8HhAHCkCWn6BUJtmQ5JzH\/pumJbd6XJQa6vRfCoEsqXjorwqCe3MYb23H0sHfvXlq1alW82mjeDWGAMGTZKfKENYsFT1pLkZDRhEmhyDKDoK4MYdB0cjGTZUYYkj61TeUEYYAwuOIUZTRx5vxlscJJXjKi4P9HFQtXGKh9F8IQ0+Nr3g5h0ASVp2QuOgTT\/F1lIIWCecj5CRlV8N88mc2Xzu6vLjKAMJjuCf75WROG5cuXiw30\/C4sV0228V10CKaJ5I0Br3TyioUqGBxVzKqrEXMXM2uniXkLFxlAGEz3hAwJQxpVwwluwZRddAimbSbvDDiy4D\/PvnzRVzD4h7x1+JVXXilWQ83kVVF1NVqRhum2CJMfhCEMrehprUQM0YurdydOcKvMKe9OUcdKwICoe\/cLNL22lnj+4hUWkgsTYuK95IaAqoDINDpDVjrtoZsGwqBLKl66XAoDTnCrbBRFn3xnOmCgx0CukGJmUkDEvwNERFqeurssCwpfMirxpuHdaHUv08Jg2w5sPz+Ie26FQecEN11jRDoQAAF9An+fNZ\/evur68g1vX3XdlP\/zL9U0Qbm\/52+vl3\/1nr+do+nTp9O\/\/c\/TdODAAf0CZTzliRMnMlfCQgpD5loBBQIBEAgkICfWOWLh6\/HSraCVMIHcCgN\/TR10glvCTJE9CIAACDhNIJfCUG3y2ekWQ+FBAARAIGECuRQGZlbpBDcvU9676Sc\/+Qn94Q9\/oK985SvU2NiYMPZsZv\/WW2\/RT3\/6U\/rsZz9LtbW12SxkgqW6cOGC2KTt7Nmz1NbWRrfddluCT8tm1pcuXRIMmEV7ezvdeOON2SxoCqXas2cPzZw5k26\/\/fYUnpatR+RWGMJg\/uUvf0nXXXcdzZkzR3SKL3\/5y6nt9BqmnEmm\/ctf\/kI7duygU6dO0Te+8Q3iPayKdvGhUbfccov484Mf\/IC+9KUvFU4gf\/GLX1BDQwNdf\/31oi888MAD9P73v79opkDHjh0T\/eELX\/hC5GOHXYaWa2FQh5TY6fPFnb+7u1v8e3h4WDQ6O4HPf\/7z4u2A92zibcDz4hh1GXDU9Pbbb9OTTz6Zq\/pzO+sykB2Z35Z\/+MMfUkdHB1199dUu9+9y2cMw4Khh37599L73vY\/uvfdeuuKKKwrF4I033qCnnnqKPvShD4n2Zx9RtCu3wiDPe+AGHRoaEtEA\/6y3t5e2bNki3gjkklb+m8Xggx\/8YK6EIQwDeRZG3oQxLAMWha1bt1KpVKK5c+fmwh+EZfD3v\/9dbJfB0QOLY1rnpCQJW5dBX18f\/epXv6KPfvSj9Oc\/\/1kUCcKQZMukmDe\/HXEY+LnPfY4eeeQR2rx5sxAGjhYOHz4sDgXiN+Q1a9ZQT08P\/e53v6OPfexjIoRmx8jnT19zzTUpltj8o8IykBFVnoQhLINrr71W2A0PJeZlbD0sgz\/96U+iL\/Ac0\/e\/\/31qbW11nkUYBmvXrqWjR4\/SK6+8Qq+++qrg8LWvfS03kaOup8ltxMAA+C1h3bp1k4SBx9A7OzvF8AJv5Mf\/5iEkHk5iI2AHec899+jyy3w6XQbyrShPwiAbR4cB28kLL7wgnAGPr7NIfPGLX8yNQ9BhwH2Bh1D5BYqHUPjf999\/v9hPKQ+XLgPZF3gBCyKGPLS8pw5hDSGHCLTFMc\/hMuxA\/yWJ7YDnmvhPXgQhzAsCi2Oe+4KujytcxOA3lCSHUXShuZTOzymCgf+QIuygR0TMeb3QF\/RbtlDCEDT5nIfJtaAm93YGMAhehAA72JSLiWb0BX0BCEpZKGFgCHK5an19fXm1UnyM2c3BKwxgMHnZMuygm8CgOAx0PVWuhUEXAtKBAAiAAAi8SwDCAGsAARAAARCYRADCAIMAARAAARCAMMAGQAAEQAAEggkgYoB1gAAIgAAIIGKADYAACIAACCBigA2AAAiAAAhoEsBQkiYoJAMBEACBohCAMBSlpVFPEAABENAkAGHQBIVkIAACIFAUAhCGorQ06gkCIAACmgQgDJqgkMwuAT5Yqauri0ZHRycVhE8Y462SXb54P6u9e\/eK80G4js3NzeKwKL5kvflEOb\/toPn3fOLcihUrcnMcrcttmZeyQxjy0pI5r4d0kKrTzEOVVcfOu7uGFQZmIIVl1apVeUCCOmSAAIQhA42AIlQnUEkY5JGtZ86coba2Nrrjjjto2bJlND4+TvPmzaPBwUHxNs0nci1dulTsJvrpT3+aPvCBD4g3bXmSH7+Rc17ylD+5Ey+XTkYmnMfAwIAo8MGDB8VZ4XxULDv1\/v7+8u+Gh4dFGnmuOP+bnb73zZ\/zO336tIgQ\/OqoRgz8PPlszk999rZt22jx4sW5Pk+hupUghSkCEAZTJJFPogT8hpKk03\/66adp9+7dQgD4kmd5y3O+2dGrAsD3sZNmgQgShgULFvg6dc6fzwUeGhqi6667riwq\/HMWBi7DuXPnqLe3lx5++GH67ne\/S+vXry\/\/bMuWLZOGfPgefhaLUtBwGefNQqMOJan38e9ZxPiSQ1CJNgYyzz0BCEPumzgfFdSJGPjN\/ezZs+VoQdachWDlypW0ffv2cvQgI4MgYWhoaKDu7u5J8DhqYCcuBUAO\/XAUwG\/9MtJQb5IOXEYY6nwI1+nRRx+l9vZ28aZfLWKQwuAVBc6bIw+OKFyfb8mHtbpfCwiD+21YiBqEEQZ+W\/e+mbPjlA5dHnjvjSTUoSQWBj9Hr+ajIwzSYXMjychANlgUYQiKDCAMhegGqVUSwpAaajwoDgFdYeB0PGfAcw08rCLnH9atW0c8Octv1OpQ0urVq8sTvq2treUhJnbicshoxowZ5TSzZ8\/2jRi8Q0n8vM2bNxPfy6uGfv\/7308RK3mPdygpaFUSRyVBw0UYSopjXbjXSwDCAJtwgoCuMPBbPK\/S0Z18ZqFQj3uVk9LqzxmQOvnsN5Qkow05\/NTX11ce71cntL2w1Tf9SkNJ99xzjxgKGxsbK2ch51i4zuqQlBMNikJmmgCEIdPNg8IlRaCSszb5zDS+Q8ByVZMthryYAIQBdlBIAmkIw\/nz58Ww1qxZs8pLWv1gx5kf8M5TFLIxUWnjBCAMxpEiQxAAARBwmwCEwe32Q+lBAARAwDgBCINxpMgQBEAABNwmAGFwu\/1QehAAARAwTgDCYBwpMgQBEAABtwlAGNxuP5QeBEAABIwTgDAYR4oMQQAEQMBtAhAGt9sPpQcBEAAB4wQgDMaRIkMQAAEQcJsAhMHt9kPpQQAEQMA4gf8HPwhTPhvojl0AAAAASUVORK5CYII=","height":235,"width":390}}
%---
%[output:26f707e8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"5","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000010000000000"],["-2.526618726678876","-0.000251327412287"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-25.266187266788759","0.997486725877128"]]}}
%---
%[output:6e27c0a7]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:7996ad92]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:9ad8d2a8]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":" -25.266187266788759"}}
%---
%[output:637faca4]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.997486725877128"}}
%---
%[output:61f9eea3]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.149016195397013"],["36.521945502464568"]]}}
%---
%[output:306c88de]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:23fb1bfd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:11c23dfc]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:4b099645]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.155508836352695"],["27.166086113998457"]]}}
%---
%[output:00aa870f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.037191061070411"],["1.937144673860303"]]}}
%---
%[output:2e5b507a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAYYAAADrCAYAAABtnTHVAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQ+Q19V170kmr3VNtbCKJjsrLoYltU27aGqyEo2mBJP0Dfg6r+3u0nnykDCkwUxbWIHFOJZWYJdAXoyahCEbHm+m7DKdPBqY9oXabZ7RIpNElDTGVhQ2uNmoCKIxbtra2jlfPOvdy\/f\/7977vd97zncmE5ff93vvPZ97z+dz77n\/3vbGG2+8AfIIAoKAICAICAJvIvA2EQZpC4KAICAICAIqAiIM0h4EAUFAEBAEpiAgwsC4QZw+fRqWLVsWITA4OAjNzc1W0NizZw\/09fXB5s2boaurK8oD\/w0f+ttkxnF2TUxMwMaNG2HJkiXQ3t5uMrvEtI4ePQpLly6Fz3zmM7nsLFPGQ4cOwYMPPghr1661YhNheeTIkSj93bt3Q2dnZ+684uo+98eWXkSc161bBy0tLdZws1R0Z8mKMDiDWjIiBGyThS4MTU1NERE8+uijsHPnTmfCMDAwAEjceUSXyKpIGTHtxYsXw4oVK6wRHOWhinqRlmy7rouURX0Xy3Xvvfc6bQ9ly1rFdyIMhlFHMti+fXuUKvZIVCIiJ1m1ahWMjIwA9sL0d\/Qemur01AN9\/\/vfDxdccEHUe8Mny2kpX71MOoGeOnUq6uFSjxp7opR23jRw1KGTiUoOWAYcPdCzcOFC6O\/vByRvfIggT5w4MUmocaRJWIyPj0ffqTipdt13332wZcsW2L9\/\/2SeaNOiRYsisdD\/XR3BUF1iHW3duhXw75kzZ06WVy+DWg\/0G9pHvXm9bqnuW1tbE8ui4o4GEF7YdlAU6Ono6IjwwgdHgdTDzxINwpZwoHSwHvW81d9Ut0lrs2rdj46ORr6ht3lqL7otWAbCUW+TCxYsmLQTMbn55pvhk5\/85DmjUmprep5x9WOYCmqdnAiDwepTRUFNlobfuqNlOTX9ToSjExH9rjd6vWekErEqDhdddNGUUBIJA5EtpXv48OEpZJ6WRqPCgGkTToSbKogoImNjY5GAUTl1kUGyoxBZkjAQSalYqTjGkeLJkycBRTmtDHpdU93pBKzWfVIZL7\/88inkr7YH\/Tck7c997nNw++23T4qC3n70pp5UpqR6jxMGXRQoDxKkpDZPApdUl\/S93uaxbF\/+8pfhq1\/96hRRv\/766+Ghhx6K7cjECY6rMKpBenGalAiDIbipAc+YMWOyp0s9IXKCffv2RQRLf2PW1Gul3j\/2AnUyod4zETd+hyMRtacZF\/ulxo+ERiMXtQdHvS5MD3ubevrYSyuaRpYwYI88K7yg9+b090mA40gXcZgzZ84UwcsTSqI01e+x160TvV6X9DvhRCOKL37xi1HvmH6PGwmpTS9PKEkPHSX9ndR+9DkkvX0iToS1TuxJo1L9fZ1wH3jggSltXhXtuBBbUieA2jy2SSo3CRXVL4561NGgOuqMC4lhneM3LsOLhujGejIiDIYgjiM7nQzISVQSVxssFkXv3au9c\/xv7ClTr1V15Dhh0J2MwjVkclIoSU2\/aBomhEHFjXrT5ORY9rgJcxVHXfDShEHv0SKOOJLScU4ifr35IFlRmfX5gqSwEJYvTRiSwma6MCT1zpNGlKoY0oSybid1ZpKEIS6NuBFrlljpIw99RBHX5tUyxdU\/hdPU8qihtayyG6KGWiYjwmCo2uJ6JEnCkNSgk4QB\/z2JsPSwi2pOUVKnEUOjwqCLZNbfcVVA39x5553RaIZi9Uk976LCoJOC+ncjwqCGOpImkuPmoWj0p36Td4SQFbah9qOvJoprO66FQW9zFFrSQ3amhEGd0xJhSCY\/EQZDwmAjlKQXLY7o04RB7YXRiEIlm+XLl8fOMahOmDcNClep4S194jrp77gq0HvJ6oio0VCSPreihiLKhpKKhoXwfVUwaTJcFQaduPSwTVYoKatpmwwl6eFRsoPmp5JGDDSKpt\/1MulCgXVVJpQUh4UIgwhDlo8Y+d3W5HOeYXXS+vK48AKFFpImn1VhUAlMBSltRQ29lyUM+F7SShf8jfDU30mahCec9Di2SvyY7i233BJN0MaFGpIWCmAZ8kw+U+9dJ52kSdokHDEdfGiFW1w4RF3Ng+ncc889cPfdd59jl77yi9LKmnzGeH7WfFDeyecsYdAdMK3Nx5U7z+SzPnKSOQYRBiPEnycR08tVVVIsOmKg8sbF0TGskGeOISsN\/F0laiwvjkRuu+22c1aIEDmoZJImDGnr9PMuV6UJTpVEkXRvuOGGyRU\/SEK33norrFy5cjJkpQsTLlft7e1NXa6qEnBcaDGOROPmmzBvLCON6GhZ8\/333w9f+9rXgOZbVMHTxZ5ELw1fzCdtuao+qknajJg0P6DOgSUJgy7aiAcuk6ZJYSyDPt+D\/6bmqdanvolSnbNTf9NDZvr8Wx5fD\/mdykNJ6OBr1qyJ1prH7UjV1zinLc30vaKyel++l59L+VSy1JcK66OpJEyIeFCAbe1K5lIfup3UKcB\/j1ttl2c3vexjSG89lQpDniV6SKa45jwE5xJhqA+VJYUFszYTqhYW2flcH2SqL2lWWC7PkSfoi7Lz2dNQEo4G0HnwSRox4O9tbW25zpqpvsmml0CEwfcaeqt8cXHsrF3EST1bDEMVOV+oPihVV9K4eaa85zjJWUnZ9VbZiAFVf8OGDdDT0xOJQ5wwUAXOmzcvCGHIrg55QxAQBASB6hGoTBiw94zP1VdfnTjHEDdkLDKUrx5eKYEgIAgIAvVDoBJhwGHgrl274I477ojOvUmafKbhIg3F9b\/rB7eUWBAQBAQB\/xGoRBgwdITLBTHumrUqSYeQ5iTiJqOvuOIK\/xGXEgoCgoAg8CYCeMryrFmzvMPDuTAkrShAZPJMHqVNRqMwHDt2zDuQbRVI7LWFrD\/pSh37Uxc2SuJr\/ToXBh3ctBEDrVpSNynhJqOk0xB9BdlGg8I0xV5byPqTrtSxP3VhoyS+1q93woBiMDQ0NHkZir7BLW1U4SvINhqUCIMtVP1KV9q0X\/VhujS+1m\/lwmASaF9BNmmjmtbx48e9jE+KveYQkDo2h6WPKfnKWSIMPraWnGUS0sgJVI1fkzquceXlKLoIQw6QGn3FV5AbtSvpeyENW8j6k67UsT91YaMkvnKWjBhs1LajNIU0HAFdYTZSxxWC7yBrEQbGINsyXUjDFrL+pCt17E9d2CiJCIMNVLU0fQXZlulCGraQ9SddqWN\/6sJGSXzlLAkl2ahtR2kKaTgCusJspI4rBN9B1iIMjEG2ZbqQhi1k\/UlX6tifurBREhEGG6hKKEn2MThoV1VmIcJQJfr28xZhsI8xuyMihDQcNKqKs5A6rrgCLGcvwmAZYEzeV5BtmS6kYQtZf9KVOvanLmyUxFfOkslnG7XtKE0hDUdAV5iN1HGF4DvIWoSBMci2TBfSsIWsP+lKHftTFzZKIsJgA1WZfJbJZwftqsosRBiqRN9+3iIM9jGWOQYHGFeZBTeSRKy52czNXhEGB4ziK8i2TOfmRNzsFWGw5Tn+pOsrZ8nksz9tpHBJuBElN3tFGAq7RO0+CEIY0u5rTqqRjo4O2Lt3r5MK8xVkW8ZzI0pu9oow2PIcf9L1lbMKjRhQGFavXg3r16+H9vb2THTxPudNmzZFdzS7eHwF2Zbt3IiSm70iDLY8x590feWsQsLgD5zxJfEVZFu4cSNKbvaKMNjyHH\/S9ZWzCgkDhZIQ1sHBQWhubvYHYdn57FVd2CiMCIMNVP1Kk1sdByEM2IQmJiZg3bp1sH\/\/\/qhF7d69Gzo7O71oXb6CbAscbk7EzV4ZMdjyHH\/S9ZWzCo0YdDgPHToEixcvjv554cKF0N\/fD01NTZWh7ivItgDhRpTc7BVhsOU5\/qTrK2c1JAwErzqKaGlpiSab80xOm64eX0E2bSelx40oudkrwmDLc\/xJ11fOMiIMKsy0Emnbtm3O5yB8BdlWM+RGlNzsFWGw5Tn+pOsrZxkRBhkxVNPQuBElN3tFGKrxK5e5BikMMsfgsgmdmxc3ouRmrwhDtf5lO\/eh7z4HK4eehNOf\/4jtrAqnX3jEoK9K2rx5M3R1dRXO2MYHvqqvDVuFNGyh6le63MSQk73BCAPtYzh58mRlE8xpbivC4BepmS4NJ9KQBQamW49\/6QUjDP5BO7VEIgy+11Bj5RNhaAy\/OnzNqY4HDozCwIHj9Q8lyVlJfrkWJyfiGDrjaDOnNh2UMCxbtgyOHDmSmyHldNXcUBV+kZMTcSRJjjZzatPBCENh5nL8gYSSHAPuODtOpCFzDI4bVwXZiTA4Al2EwRHQFWUjwlAR8A6z5VTHIgyGGtbAwECU0tq1a2NTFGEwBLSnyXAiDRkxeNoIDRYL9zDgyqQg9jEYxKVQUrSZbsWKFSIMbyLHjSi52StzDIUoonYvizA0WGW4qW7jxo3wxBNPREd8y4jhLKDciJKbvVLHDRKH558HKwx79uyBvr6+CH68lwGfoaEh48dvYz74jI6OSihJaezciJKbvSIMnjN7g8VbdP9j8PAzZ8IKJWG8f3x8HO666y7YsGED9PT0RL35rHmAolji3glMH\/PZsWOHCIMIQ9EmVOv3uYkhJ3uDEwZ1o1tra2t0oxsJg+ljt1FobrjhhlyiI5PPtebAzMJzIg0Cg5vNnOxtXvWtqJqDmXxOEwacJEYyN3EnNIrMrl274I477ohuhssajaAw0DMyMpJJNHV\/YWxsDFCYuTzc7MV65WYzB3vnz58P\/3H+xfDKTWdXWQYjDGgMxv0PHjw4JZQ0Z84cwJ3R3d3dRk5cVecwVPLDa0Tvueeec\/hQRgxhSwSn3qSMGMJuyw8\/fQYWfemx8IQBLVLvY6BqtHkMd54Rw7Fjx8JuUTLHwKZ+0VBuYsjF3j\/762PwhZEfhSkMrj1UhGEq4lyciGvvWYTBNcO4y4\/mF97+2ovw4ld+z13GOXMqfFFPznQreU1CSZXA7ixTbkIowuCsaTnNSA0jnfdP+2D8b\/6X0\/zzZFZKGOjCnqxTVtN2KecpXNF3RBiKIlav90UY6lVfZUrLoY7n3v0InDj9c5jZfB6cGf4jGP3Bd8pAZfWbUsKAJcKJ4eHh4Smrj0gwaPI5K\/Rj2jIRBtOI+pUeB9LQEedmc+j2bn9oDPr2Ho2qefPvtMPAkg+Dj\/OipYSBBACPpsBNbeqjLlc9deoUbNq0KboG1MUjwuAC5eryCJ004pDlZnPI9tJpqljPOFp4\/LPXgq+cJcJQHc81nHPITiQkeRYBqeOG3aTyBDBs9KPTE3Dzlx6PyoKicF\/3lXDd7GlhCUPeUBLtdYjbc2CjtnxVXxu2CmnYQtWvdEUY\/KqPMqWhoy90UcC\/feWsUiMGAiduHwMepofhJfytt7c3CiO1t7eXwbPwN76CXNiQnB8IaeQEqsavSR3Xs\/JwlIAPTjTTgyOFfZ++Khox0OMrZzUkDL5Vma8g28JJSMMWsv6kK3XsT13kLQmKAu5qJnGgkYIuCsGOGPIC5eo9EQZXSFeTDzeSRJS52Vxne\/E2NrxjQX3U+YQ4r\/GVs0qPGOLCSGR4R0eHkUP0itKPryAXtSPv+3V2orw2qu9xs1eEoUwrcfsNjgpuG3oyuldBF4Qvdv0KfLh9emqBfOWsUsKAN6rhUdvz5s2DRYsWTR67bfoQvaJV7CvIRe3I+z43ouRmrwhDXk9w+97hE6\/An+5\/5hwxSAsZJZXQV84qJQzqsds4sYwb2dra2qITVXEkYeMWtzxV7yvIecpe5h1uRMnNXhGGMl5h\/pukUQHlhOGiG+c0wxd+\/72FM\/eVs4wIAy5LxWs3ccOb6Yt6iiDtK8hFbCjyLjei5GavCEMRbzD3Lp5ltOXA8dgRgSoG83+lGX5n7qXRfoSyj6+cVUoYEAT1uAt1lLBv377onob+\/v7och2Xj68g28KAG1Fys1eEwZbnvJUujgYGDhyHZ0\/\/PFMI8CvcmIYjBHXJaSOl9JWzSguDfiwGCsX27duhpaXF6d4FtVJ8BbmRhpP2LTei5GavCIM5z6Glo3lEAHMl4t\/6398Lcy4935gQ6Bb5ylmlhcFclZlLyVeQzVk4NSVuRMnNXhGGcp6DIrDrkXH47ujLqaMANXUUgg+9Zxqs\/dis6J9NjQiyLPCVs0oJgz75rBovcwxZTcHc79yIkpu9IgzJvoLkj\/\/7+mPPwzMvvJZbAIj0r589HW6\/qc2ZACRZwkYY1NNVm5ubzbFgjpR8BTlH0Uu9wo0oudnLXRgo\/PMX3\/kJPPLMGTjx0lkxyPtEcwHTz4Nbrm2BD7T9stORQN4y+spZhUYMaZvaVCBcX9BDefsKct5GUvQ9bkTJzd7QhUEl+byx\/zgfIQFYcm0LXHrhLxqdHC7qk0Xf95WzCgkDGZ0WSioKjMn3fQXZpI1qWtyIkpu9IQgDhXye+Mmr8NffP1m410\/tnWL+VcwD2PJfTNdXziolDDaBaiRtX0FuxKa0b7kRJTd7fRcGtcf\/hb\/\/ETz9\/GuliR9tRfKf0QTQ2T4Dll\/XOnn9pauJYFt+mpaur5xVSBjy3vUsZyW5aWLciJKbvVUKg0r6\/+8HL8L3f\/zTzLX+Wa2eCB7j\/n\/80cth9ozzJz+h37jVcRDCkFXxVf\/uK8i2cOHmRNzstSkMRPw\/GH8V\/uYfX4QTpyca6u1Tjz\/6\/+nnwR988N1w7RVndwQX6fFzq2NfOavQiMEWwZlK11eQTdmnp8PNibjZW1YYKK7\/+LOvwN\/+8FTUbPTTP8u0SZrkvWLG+XDbjZfBL7zj7RHpY35FyF\/Co28h4CtnNSQMeEZSX1\/flHrevHlzdJheFY+vINvCghtRcrNXFwbq5R994TX4q8dfgB+daryXT22TSH\/WxU2wqOMSaL\/kbJjHFOHn9QFudewrZ5UWBhSF4eHhKfcu0BxEd3d3JeLgK8h5naLoe9ycKFR7ifAfevoleOSZl42EdVTCp\/DOZc3nwYrrW2Ha+f+lEtLP075DreMk233lrFLCoJ+TpBotG9zyNH8z73BzojrZq07e\/uWjz8ODT52OKr3oJq20lkK9+cubm2DBlRfB3MsuiF5v5LRPMy2zfCp1quPyVgYaShJhMNEkGk+DmxP5YC8eyYyEjGGc4e89F63UMUn2avgGJ3FnT3sD\/vgTvxrF8etM+Hlbuw91nLesJt4LasSAgEgoyUSzaCwNbk5ky17q3b\/w03+FPd97Dv75uZ9Z690j2WNI58p3\/xIs+o0ZmSEdWzY31vLsfc3N3uCEgcRBJp\/tOUlWytycqIi9RPbjL\/8LfP3w81bIXu\/dt1\/6TljS+e7JGL76e1ZdJv1exOayefj0HTd7gxKGqieZkxqyryDbcjwuTkQk\/+yzz8Jll10G\/\/jjn8KBH56C0RfNrcpR60jdiIW9+w+9Z\/rk+Tuml2dmtQ0udUw4cLPXV84qNfmMlagfqLd7927o7OzMaudWf\/cVZFtG192JiPAn\/u3f4SvfHouOT8bHdMxe79m3XdwEv3n5hdE9vfS4XpaZt03UvY7z2inCUBQpu++XFga1WOp+BrnBzW6Fqan7RhrqSpx93z8JT\/7kVSuTszqZY9x+5kVN8LtXXwJXXHzuMQvuasR8Tr7VsXkLp6bIzV5fO7NGhEGtWrziE0cTg4ODIPcx2HUjm06kkjz+9ys\/fx3+7slT8LSjXj0id337dLi54xLASWHs0f\/7yz+BWbNmGd1pa7eGGk\/dZh03XjrzKXCzN2hhoPuesZnkOUBvYmIC1q1bB\/v3749aVtr9DXrIKm1E4ivI5t3nbIpFnYiOSkCSfeqF12DvY89HPXpb4Rs9hIPx+t96bzNc8+alKUXDOEXttYW7y3S52czNXl85q\/SIoZHwEQoJPmvXroWsiWzMZ3R0NHo36\/EV5Kxyl\/39H44cjSZj8Tn9s3+Lllo+Mf6qU6K\/6rIL4aZfvWjSBJuxem6kUUb8y7YlX77jVse+clYpYUjb4FamgalCoX+Pv7W1teU6YsNXkItiooZx\/s+hcfjO8Zetkb26Agfj9JdNPw+6fvNSePvb3hblSatw1N5\/UXtMvc+NNEQYTLUcf9PxlbNKCYNJmNNEhkJO8+bNC04YkPxfnngdtj80BidOTRg5\/VIP3eDfc2deCLd88N3RSZjqY7Nnb7J9qGmJMNhC1p90udWxCENM26O5iYULF0J\/fz80NTVNeSvuYqC001t9BBkF4MzE6\/DZvzpamvz1dfU3zJkOnbOmAa7rf+OdM1gclcCx98zRZhEGP0S68hEDwoACMT4+fo44HD16FJYuXQpbt26N9kjof+sQ+iAMKAQU\/sl7Bj4defzrrRdEp1\/Sk9Wr5+ZE3OwVYfCDJG2WwgfOirPPC2FAwl+zZg1s2bIF2tvbU+shbT4CQaZnZGTEZn1OSXv8ldfhT\/\/uRXj0x2dX+MQ9LRe+I\/rnj85+J\/zer18A9HcjhRwbG4PW1reEpJG06vAtN3uxTrjZzMHe+fPnT3G3Y8eOeed+pYQBQzyrV6+G9evXn0PkSPKbNm2Cbdu25d7HUOSo7rTJaNfqi8cy9Hz1+7GVir39337fDPjUh1utXXbCrQfNzV4ZMXjHl8YL5Jqz8hpgXBjykLza66cJZtyfoC9J1dPCv3t7e2Hnzp2xIwsXIGOo6Knnfwa\/v+NcQUAx2PfpqyLss8JAeSso7T1uRMnNXhEGE17idxouOKsMAoWEQd9slpRh2oY1\/Ebf4KZOPmMeQ0NDk\/MNRc5ksg0yisLcux+ZYjaJgQsh0PHmRpTc7BVhKENp9frGNmeVRaOQMFAmaaGksgUx8Z1NkAcOjMLAgeOTxaxSEKgQ3IiSm70iDCZYwe80bHJWI5aXEoZGMrT5rQ2QcZSw6EuPRefzUIjovu4rvVgiyo0oudkrwmCTLfxI2wZnmbCskDDQvoLly5fDjh074MiRI7FlyHNekonC62mYBlkPHfkwSlBt5kaU3OwVYbDBEn6laZqzTFlXSBhMZWorHZMg66Lw54tmw8obz55L5MvDjSi52SvC4Iun2SuHSc4yWUoRhhg0dVHY+4dz4Yb26SZxN5IWN6LkZq8IgxE38TqRoIQh7qgKFf06h5L0OYX\/+6m5cOMc\/0RBSMNrfzdWOG5iyM3eoIQhqdXjMtSNGzfCkiVLMncwG\/McJaFGQUZRuG3oyckzjVYvaIM7PjHLRlGNpMnNibjZK+JvxE28TqRRzrJlnPFQkr4PwVbB49JtFOSh7z4HK4eejJLGiebHP3uty+IXzosbUXKzV4ShsEvU7oNGOcuWwcaFocyRGKaMawRkdV7Bt9VHSfhwI0pu9oowmGIGf9NphLNsWmVcGJJOSrVpBKXdCMiL7n9sMoSEx1pcN3uaiyI3lAc3ouRmrwhDQ+5Ri48b4SybBpYShrTJ57Q7mW0agmmXBfnhp89Em9jwue4902DfyrPnHfn+cCNKbvaKMPjugY2XryxnNZ5zegqlhMF2ocqmXxZkGi3UJYRE+HAjSm72ijCUZYL6fFeWs2xbWFoY9Gs3005JtW1EI6EkdbSw\/hOzoHdBm6viNpwPN6LkZq8IQ8Mu4n0CwQlD3IU5VYtDGZDrOloQ0vDe540UkJsYcrO3DGcZaVgZiZQaMZi+qMeUoUVBVkcLt9\/UBn0f93fPQhxG3JyIm70i\/qaYwd90inKWK0tKC8OyZcuii3XwLmb1yXNRjy3jioJc59GCkIatVuRXutzEkJu9RTnLVessJQxYuD179sDw8DAMDg5OXuFJq5W6u7uhq6vLlQ2T+RQBWd23cHPHJbBzya85L2+jGXJzIm72ivg36iH+f1+Es1xaU1oYsJBxN7pt3ry5ElHA8hQB+eYvPQ4PPf1ShDXucK7iBrZGK5obUXKzV4ShUQ\/x\/\/sinOXSmoaEwWVB8+RVBOTmVd+KkqzD0RdJtnMjSm72ijDk8fp6v1OEs1xaWkoYaPVRT0\/POXMMLguv55UXZHXS+f6eK6HnmndVWezSeXMjSm72ijCUdo3afJiXs1wbVEoY6n7nc90nnamRcCNKbvaKMLimQ\/f5BSUMCB9OPo+OjkYrk3x58oCsTjrX6fiLOIy5ESU3e0UYfGEWe+XIw1n2ck9OufSIAZer1vHOZzWMVNdJZxkxVOEq1eTJTQy52RuUMFTjItm55gFZDSP5ft9ClsXcnIibvTJiyPKA+v+eh7OqsLLUiKGKgubJMwvkkMJIQhp5WkT93+EmhtzszeKsqlpwIWGgDWzLly+HHTt21C6UpIaR1n5sFqz9WH0OzJM5BgBupCHiXxUtuss3CGFwB1e5nLJAruNlPGlIcCNKbvaKMJTjgTp9lcVZVdlSaMSgFrKOx27PvfsRwHBSnTe1qXXAjSi52SvCUBUtuss3OGGo47HbtNv5Tz56Odz521e4q31LOXEjSm72ijBYchyPkg1KGOp47PbAgVEYOHA8ahJ1X6ZK7ZobUXKzV4TBIwa3VJTghKFux26r8wunP\/8RS9XsNlluRMnNXhEGt\/5URW5BCQMCWLdjtymMVPfdzjLHUK\/LlBolG25iyM3e4IQBG3xdjt1W9y+EsExVQkmN0m19vudGlNzsDVIYfHOvJJDV\/Qv7Pn0VXDd7mm9FL1Uebk7EzV4JJZVyi1p9JMLgoLqSQFbnF0KZeBbScNCgPMiCmxhys1eEoaSTqeGqlpYW2LlzJ7S3t8emliUMoexfkFBSycZUw8+4ESU3e0UYSjjl0aNHYc2aNbBly5ZIDOImvNVk40AO7XwkmXyWyecSrlSbT0QY\/Kiq0jufqyi+LhR6GbKEIaSJZwklVdEC3efJjSi52RvciAFJeunSpTA+Pn6Ot3R0dMDg4CA0Nzcb9SQcMRw8eBD6+\/uhqanpnLTjQA5xY5uEkow2K68T40aU3OwNShjonCSM+bu4wU0Vod27dyfeMx0Hckj3L+gMxs2JuNkro0KvNdtI4YIShqrufCaB2Lp1a6w4IMj0jIyMRP+5cNcYjL\/yOrRc+A7Yv6TVSGX6ksjS9snGAAAOdElEQVTY2Bi0toZlUxq23OxFLLjZzMHe+fPnT2nmx44d84VSJstRao6BRgw9PT2JvXdblsYd3kd56eqrTjwv+o0Z8L\/\/5\/tsFauSdLn1oLnZKyOGStzKaaZBjRgQuax4vw109aO+9Tx0kEO7mEdCScdh1ixZlWTDt3xJk5v4ByUMdJPbkSNHYtuTqclnfRUS7mno7e1N3Mugg6xOPIe041kmn32hMfvl4EaU3OwNShjsu8NbOejnMRWZfA7xRFUVe25OxM1eCSW5ZJpq8hJhcIC7DnLIK5KENBw0KA+y4CaG3OwNUhhwnqGvry9yH+zJ4zM0NJS4z8C2n6kgh7zjWUJJtluSP+lzI0pu9gYnDLg6CDe33XXXXbBhwwagFUppq4Zsu1uSMIS241mEwXZL8id9bkTJzd6ghEHdx4Dr6NetWzcpDDhhvGnTJti2bZvxnc9Z7qqCHOpR2zLHIKuSsvygzr+LMPhRe6X2MaQJA04W46jBxpEYWZCpwjD03edg5dCT0SchrkiSOYas1hDG79yIkpu9QY0Y0OVoH4MaSpozZw7gXdDd3d3Q1dXl3DNVkENfkSTC4Lx5VZIhN6LkZm9wwoBe4vPVnqGvSBJhqISnnWfKjSi52RukMDj3kowMVZCbV30revu690yDfSuv8q2oRsrDzYm42Svib8RNvE5EhMFB9RDI6lLVnUt+DW7uuMRB7u6z4EaU3OwVYXDvU65zDFIY1H0MBGjazmTboBPI6oqk+3uuhJ5r3mU760rS50aU3OwVYajErZxmGpwwxF2zSWcoVT35rK5Ievyz1wLe9Rziw40oudkrwhCi1061KShhIAHAS3o6OzunWOrDctWQb21TweZGlNzsFWEQYagKgdL7GHBZapww+LDBjcOKJCGNqlzGbb7cxJCbvUGNGNA14o7A9iWUNPfuRwAnoDGEhKGkUB9uTsTNXhH\/UD33LbuCEoas+xjU6sS7Gfbu3eukhhHk\/\/+9HwIKAz4hL1UV0nDSpCrPhJsYcrM3KGGo3FsSCqALQ6iH55H53JyIm70i\/r4yjblyiTCYwzIxJV0YQl6qKqThoEF5kAU3MeRmb5DCELePYfPmzZWck4Q+jCBv3HMw+MPzZMTgAWM7KgI3ouRmb3DC4Os+hhVf\/nsYOHA8ctuQ9zDIiMERM1ecDTei5GZvUMLg8z6G963+Ojz8zJnInU9\/\/iMVu7Xd7Lk5ETd7Rfzt+o8PqYswOKgFBJmEIfSlqkIaDhqUB1lwE0Nu9gYlDOgvvoaSLrz1L6I9DKEvVRVh8IC1HRSBG1Fyszc4YSBx6Ovrm+IeVU4+t73vA\/DKTQNReUQYHLCW4yy4kYaIv+MGVkF2QQpDBTimZokgn\/lvg9E7oe9hENLwrfXZKQ83MeRmrwiDHb+ZkurMD3wCXr1uTfRvoe9hEGFw0KA8yIIbUXKzV4TBgZO13vg\/4LWrb41y2vfpq+C62dMc5FpdFtyciJu9Iv7V+ZarnEUYHCA984P\/FV79UK8IgwOsq8hChKEK1N3mya2ORRgctK93\/e7d8K8zPxTlFPoeBulNOmhQHmTBjSi52SvC4MDJLrnlK\/D6xe8N\/rhtgpKbE3GzV8TfAWlUnIUIg4MKuPhTfwn\/cf7FIgwOsK4iCxGGKlB3mye3OhZhcNC+mld9K8qFwx4G6U06aFAeZMGNKLnZK8LgwMlIGHqueVe0XDX0h5sTcbNXxD90Dz57IvSxY8e8M7TUnc\/eWfFmgUgYOGxuE9LwtRWaLRc3MeRmrwiDWX+JTY2EgcPmNhEGBw3Kgyy4ESU3e0UYFCfT74xeuHAh9Pf3Q1NT0zmueOjQIVi8ePHkv7e0tMDOnTuhvb39nHdJGDhsbhNh8IC1HRSBG1Fys1eE4U0nmpiYgHXr1sG8efOim97obyT8tWvXnuNqeIrr6Oho7G\/6yyIMDpiqwiy4kYaIf4WNzVHWIgwpQCP5Hzx4MHbUMDAwAG1tbbmuCyVhCP3mNoKSG1Fys1eEwRE7V5iNCEMJYdBHF1n1h8LA4YIeEYaslhDO79zEkJu9IgwJvkrzDd3d3eeMCvS5CEwi7b4HEYZwCDHOEm6kISOGsNszWifCEFPHNCLAn+Imn48ePQpLly6FrVu3QmdnJ+h\/x80xvOPFf4ZfengLjIyMBN+qxsbGoLW1NXg7yUBu9qLd3GzmYO\/8+fOn+KzsY1DgyBKFJLbDOQd84iaqccTAZdez9CZ56CG3URI3e2XEECMKSSuR0lw+bTIahYHL5jafh6G2KNtXJ7Jlr9SxTWT9SNvXNl3Jzmck9\/Hx8cS9C1RluIcB3x0cHITm5mbAv3t7e1P3MYgw+NHgbZTCVyeyYSulyc1msddma8qftnNhiJtQxuJ2dHREAvDUU0\/B0NDQpGjoG9x2794dzTfEPThiOP\/w1+AXTvxDfgTkTUFAEBAEKkRA5hgqBF+yFgQEAUFAEMiHgPMRQ75iyVuCgCAgCAgCVSEgwlAV8pKvICAICAKeIiDC4GnFSLEEAUFAEKgKARGGqpCXfAUBQUAQ8BQBEQZPK0aKJQgIAoJAVQgEIQx4OmtfX1+EYdpZSlWB3Ei+uI9j+\/btURJpS3XV99LurGikLK6+zWszlafoYYuu7MibT1579aXeae0hb95VvZfXZjoGB\/c91b1dJ2Fd5ARpV\/VVe2HAhrNmzRrYsmVLhBn9d9xFPq5ANZWPusEP93eom\/3UPPRjy\/Hv4eHhyY2BpsrjIp28Nuv2Y8egjp2CvPbq95ao7b5ubT2vzSSEePwN7l2qc7tOEwXs+PnWdmsvDDop+qi+ZQlVPReKiKGnpydxgx\/lU2fSKGozksfq1avhzJkzEHdCb1nsXX2X116s002bNsG2bduiUwDq\/BSxWe3o1bld6\/VFojdt2rTop49\/\/OO57pxxVe+1Fwb9UL20Q\/ZcgWoin6Sb7ujmu7Q86upAZWzG+r7mmmvgG9\/4xuStgCbwd5FGEXvTLrNyUVZTeRSxOW7EkHShl6nyuUoHbXvppZei8Jh6o6Wr\/LPyCUIY1BveilwFmgVOlb\/HjRDyjobynkVVpX1xeRe1GQVw165dsGrVKtiwYUNthUEdBSbVMbVrxC3PnJNvdUvlKVrH9P7+\/fthxYoVua749dX2tDafp8Pn0i4RBpdoF8irqANR0kgg9957b+JBgwWK4PzVIjbjuxs3boQlS5ZEd1L42OvKArCIvbTAgiacsw6UzMq7qt+L2Kzfv6IfqlmVDSbz9XXhRBDCgBVF9zNwDiXVWRSwDouEGZAkHnzwwajefXWuLAIpYq8eShKbu7LgrcXvvtZj7YVBDx3lDbfUodWotmRNPoeyYiOvzepyR7Uu6xZuyGsvCqF66nBWe\/C5fee1ORQxTKsLEQZLLVWWq0LmPRWWoLeSbN6ljGrmvjpXHoDy2qtPxNY5rJLX5rhQUtp9LHnw9u0dX9tu7UcMWNEcN7ipI6Wk3nNdN0AlbX5KWljgq3PlJaG89qob3Oq+2Suvzep9LHW3WSaf83qEvCcICAKCgCDgHQJBjBi8Q1UKJAgIAoJAjREQYahx5UnRBQFBQBCwgYAIgw1UJU1BQBAQBGqMgAhDjStPii4ICAKCgA0ERBhsoCppCgKCgCBQYwREGGpceT4VXd+AlVY225uz1GWdCxcuhP7+fmhqasqES98rkPmB4xdoiWdHR4fVI9XV84mK4OcYDsnOIgIiDBbB5ZS0T8JQpCxqHdVBGLC8dPyL7fYVyomutnEKMX0RhhBr1ZJN+g1idPyEugmJerPYQ8eD7fBUTHrwMpJFixZN+Xe6oETtpeL7WT3VpI1P6mZHTCduk1+SHfTveCkMnWCq987VfDF99XfM++TJkzAyMgJHjhyJ8sbfVRzuvPNO2LdvX3SxFF6wU8Ru\/RywIieuxoleFvFn\/W6pmUmyHiAgwuBBJdShCFkHvqm9dLQHyRB3qlLvVj3gj05DpeOm43Yupx2GqJ8sGve3eq6Qim+aHQsWLIBly5bBzJkzo\/CTboeejyokaGfcIYbqEeiU3qOPPhqdfht3Kmya3XHCoN5cpx8hkTUayiL+rN\/r0G6ljOUQEGEohxu7r7KOncgK36hnWunCEPct3cy2fv36qGdNT1I5VNJMK0vaGUNletVqvjqRxtmgisupU6emHIyHNibZjb\/FCYN+cU2SsJSxTYSBnZtPGizCwLfuC1uuhlH0UE8SGatn4tBZN7ow6OEfKljc2ThJ8wDqOUppwpBGdnnJU72gHstKITU97bjrOFWBPHz4MGCPX3+SzgRKCiWpcw5J9uW1TS2LCENhFwnmAxGGYKrSnSEqMarzDGr4hgSBBGRsbAzo\/t44Ych7ZWOVwoA2LF26FMbHxyfnLtJGDHmEIa\/dSSOG0dHRKZPRIgzu\/CDknEQYQq5dy7bpxyeTMGC4Z\/Xq1aCGgbJCSUiwg4ODmRfdVxlKwkljnYgbDSXltVtCSZYbsyQ\/BQERBmkQuRDImiBWwzf4Lk7iYogDV\/hQLx9X7KiTrvrkszpZnTYXYHLyWSXc5cuXTyk3\/qb2wFEY1B4+hcCSQkmUNo4w1MlsffI5r91Jk8+08koVX3VeBstB9Ud5UZ3QRHvcPg8JJeVyjSBfEmEIslrtGKXH1tV5Bp38cWJ18eLFUUGQjLZu3RpNnnZ3d0NXV9fkHRpEqvoS0qxNXGnn9GdNhOt5kR26oOnCgH+rS0+x7G1tbTA8PByNdh544IEpwqESMi3bxeWq3\/72t2Hbtm3R6KiI3XHC8M1vfjPCGK85xUddnhs350GhMMQXhfDAgQORaGXZnmeDoJ1WJ6lWgYAIQxWoS55sEYibd8gLRp5VSXnTyvOejBjyoBTmOyIMYdarWOUBAnET5Wn7FLKKLMKQhZD8bgoBEQZTSEo6gkAMAvpOaQqdlQFLPyspLnRVJl39GzkryQSK9U7jPwFQOMjF0c0lcgAAAABJRU5ErkJggg==","height":235,"width":390}}
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
