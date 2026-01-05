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
use_current_controller_from_ccaller_mod1 = 1;
use_phase_shift_filter_from_ccaller_mod1 = 1;
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
% ld1_load_trafo = 100e-6; % for f > 400Hz 
ld1_load_trafo = 400e-6; % for f <= 80Hz output
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
    frequency_set = 50;
end
omega_set = 2*pi*frequency_set;

a = 1 + 2*pi*frequency_set*ts_inv;
b = 1 - 2*pi*frequency_set*ts_inv;
phase_shift_filter_gain = 0.985-0.06/350*(frequency_set-50);
phase_shit_filter_d = phase_shift_filter_gain * (1-a*z_inv^-1)/(1-b*z_inv^-1);
flt_dq = 2/(s/omega_set + 1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d, options); grid on 
[num, den] = tfdata(flt_dq_d,'v') %[output:26f707e8] %[output:2214b3d6]
figure; bode(phase_shit_filter_d,flt_dq_d, options); grid on %[output:2a03b5f5]

% 1. Define frequency range in Hz (10 Hz to 10 kHz)
freq_Hz = logspace(1, 3, 1000); 

% 2. Convert Hz to Rad/s for the bode function
w_rad = 2 * pi * freq_Hz; 

% 3. Calculate Bode data (Note: No 'options' argument here)
[mag, phase] = bode(phase_shit_filter_d, w_rad); 

% 4. Clean up dimensions (bode returns 3D arrays)
mag = squeeze(mag); 
phase = squeeze(phase);

% 5. (Optional) Convert Magnitude to dB if needed
mag_dB = 20 * log10(mag);

% 3. Plot Magnitude (Top Subplot)
figure('Color', 'w'); % Create white background figure %[output:7a92d0b3]
subplot 211; %[output:7a92d0b3]
semilogx(freq_Hz, mag_dB, 'Color', [0.25 0.25 0.25], ... %[output:7a92d0b3]
    'LineWidth', 2, 'LineStyle', '-'); %[output:7a92d0b3]
title('Bode Phase Shift Filter'); %[output:7a92d0b3]
ylabel('A/dB'); %[output:7a92d0b3]
grid on; %[output:7a92d0b3]
set(gca, 'FontSize', 12); % Adjust font size for axes %[output:7a92d0b3]
set(gca, 'xlim', [freq_Hz(1) freq_Hz(end)]); %[output:7a92d0b3]
% 4. Plot Phase (Bottom Subplot)
subplot 212; %[output:7a92d0b3]
semilogx(freq_Hz, phase, ... %[output:7a92d0b3]
    'Color', [0.25 0.25 0.25],'LineWidth', 2, 'LineStyle', '-'); %[output:7a92d0b3]
xlabel('f/Hz'); %[output:7a92d0b3]
ylabel('Phase (deg)'); %[output:7a92d0b3]
grid on; %[output:7a92d0b3]
set(gca, 'FontSize', 12); %[output:7a92d0b3]
set(gca, 'xlim', [freq_Hz(1) freq_Hz(end)]); %[output:7a92d0b3]
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('bode_phase_shit_filter','-depsc'); %[output:7a92d0b3]
%[text] #### Resonant PI
kp_rpi = 0.25;
ki_rpi = 45;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta*omega_set] %[output:8429c053]
Aresd_nom = eye(2) + Ares_nom*ts_inv %[output:09179d2f]
a11d = 1 %[output:154cc966]
a12d = ts_inv %[output:6187332d]
a21d = -omega_set^2*ts_inv %[output:3efa50f2]
a22d = 1 -2*delta*omega_set*ts_inv %[output:3a7c206f]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:507a9885]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:20e348d0]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:7026af53]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:7d496611]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:0988a301]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:1fc7b071]

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
figure;  %[output:4c5eea94]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:4c5eea94]
xlabel('state of charge [p.u.]'); %[output:4c5eea94]
ylabel('open circuit voltage [V]'); %[output:4c5eea94]
title('open circuit voltage(state of charge)'); %[output:4c5eea94]
grid on %[output:4c5eea94]

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
heatsink_liquid_2kW; %[output:40ce6ca5] %[output:380d5b63] %[output:4d5237fd]
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
Simulink.importExternalCTypes(model,'Names',{'dsmavgflt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'mavgflts_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
Simulink.importExternalCTypes(model,'Names',{'rpi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'phase_shift_flt_output_t'});

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
%   data: {"layout":"onright","rightPanelPercent":24.7}
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
%[output:26f707e8]
%   data: {"dataType":"matrix","outputData":{"columns":3,"exponent":"-3","name":"num","rows":1,"type":"double","value":[["0","0.966531084866133","0.946498544476604"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000","-1.938144852609621","0.939101367424292"]]}}
%---
%[output:2a03b5f5]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAMgAAAB4CAYAAAC3kr3rAAAAAXNSR0IArs4c6QAAEgpJREFUeF7tXWmMFVUWPjBtFASkG1CBbtaoGUgkIRNGmQmSOIgbTtwQdGLjwhhnWkxaERFNgwtbJFEhQUEUEhVEY4K4BDsuGNO2yYQRMyyKkO6epnHYVBiFBEJPvvv6vLnvdr2qeq+r6la9OvfP636v6i7fPd8959zl3G7t7e3tJEkQEAQcEegmBBHJEATyIyAEEekQBFwQEIKIeAgCQhCRAUGgOAREgxSHm7yVEgSEICnpaGlmcQgIQYrDTd5KCQJCkJR0tDSzOASEIMXhJm+lBAEhSEo6WppZHAJCkOJwk7dSgoAQJCUdLc0sDgGrBHnzzTdp7ty5OTVftGgR3XbbbcW1JqZv7dmzh+666y5qa2vL1nDKlCm0ePFi6tGjh2etT5w4QY8++ihNnz6dLrvsMs\/nGxsb6fbbb8957r777qM5c+ZQoXl5FlbiD1ghCHegExmYNG+88YYvYUhC\/4AgjzzyCC1dupQuuuiigoW0UKEGvkuWLKE1a9ZQRUVFtrxBgwYpkkjyj0DkBDl69Cht3ryZqqurXWu5bt06z2f8N9Puk14E0Ud8HulRYwj5Sy+9RFdccYVqAH6DBtE1r\/48t9IkCL5HHRYuXEhPP\/20Iiq00ZgxY5RmQn8gsVbD3\/w9vjt27Bg99thjtG3bNmpoaKCWlhaaNm0aDR06NEdTYVC7+OKL6aGHHqIJEybQU089RSDls88+q9qyfft2SpqFEDlB7IqqndKdTCwWFJ08lZWVSjDHjx+vhI+1wJEjR5SJBkFDWr9+vTLPWJBN08uJIBiYILi1tbX08ssvK4JUVVWpPAYPHkz8u04ElAGhfvjhh+nVV19VBNmwYUNWM6EcNvlA2qamJpo5cybdc8896ntoK7QBz0Gb1dfXK4L5NS3t9FZuqVYIogsMBAUJvghGG3QEzJBSSvk0CBOBBR7+CAsat9\/0G5qbmzv5baYW8UsQCDGbb9AiwH\/lypWKQKgbfMF8xGHfydR+TBDUm7UdiIP\/8aze1iT0ceQE0e1pqGOMNlDX6AwIR9JGGD+dbBIE7zARYD4VShAWuHxl+zWx8D6ced00Yg3jRRDWXviERnj33XdzNIgQxI9kODyDEWnBggVUV1enHEioYAgJRhvztyKLiN1rbhpk7NixWQfer4nFJo\/+vD7z5+akz5o1KzsjpptrpinFA1W+73Xzjn0ZaCDRIF0Uv7QSxG2aNwwn3c80L2twkADPHz9+vJPz7uSksw\/BkwUgBiyBr7\/+WpG9pqZGmVRiYhVBljQSpAiYYvOKk7kWm8pFUJHIfRAQBKMNRi2nBHXN8\/cRtF+KcEDAXMAtpTWpQjs8coIUWkF5XhCwiUDkBBENYrO7pexCEYicIHoFeaqTtz\/gf6RS24tVaKfI8\/FBwBpBnKZ0S3WaNz7dLTUpFAFrBOEFQ16QQsWxJoIdr0nailAo4PJ8shCwRhDAZPojMoOVLOFJQ22tEiQNAEsbk41A5ARJ43b3ZItIumsfOUEAd9oOTKVbxJLdeisEYcjScuQ22SKS7tpbJUi6oZfWJwEBIUgSeknqaA0BIYg16KXgJCAgBElCL0kdrSFglSD6QiHOQX\/yyScqkkmpnUm31rtScJcRsEYQfasJzlhzaJukHervcg9IBrFGwBpB9I2Jq1evVgTBEVD9vHqskZPKpQIBawRx0iBbt26VzYqpELvkNNIaQQBRVJsV9aAISYvslxxRKs2aWiVIFCvpeuAzdCHCby5btkyFHJIkCHghEDlBILBTp06lffv20ZkzZ6h79+45dUQYzP3796tgZn4imXs1UI\/KgWiAZpT0ESNGeGUhv4eEwLXXXks9e\/akDz\/8kH755ZdASoFcBZmsEISDV7sduQ0qeLUe7hLAcexbPtb7xfSBQeLpK69Bp3\/w9ZzTQ21lF2a\/PvCbC6mt7AI60PHdqj7uAcGLLjTEF2cPaFRBs88999xASrnyyivV4BtUipwgXPEwjtya92LA30AQ6OXLl9OXX36pir700ktVcDQmCDRIkIAG1TH58vlx44LsT6cPNdGpQ010+mAT4W8zlQ0Ypr7qMXoi8d\/lU+vCrqLv\/BG5\/ueff1ZR68877zzf77k9GHR\/WiMIjtZef\/319NNPP3Vq78CBA2nt2rUFLxiaGgkZf\/TRRzR79mwVa+vss8+me++9V4Xlv+qqq1S5QQMaSC8HkMnxz9bS6YPNKqcTOz9Tnyd3ZD45mQSKmjxCEI+OfuWVV9T9F4cOHVJPwv9AiM5Ro0apgM7PP\/98QaKCM+3Dhg3LiYoCgiCWLcpKE0H8AMfaCARy0kIgUNn5w+gsfKq\/h1LviTP8ZO3rGSGIC0xuJhYCLL\/wwgsFESTflPF3332XY2JdfvnldMMNNyTWxPIleQE8xBqIzTgv7YMiC9VAQhCXjoKJdd111ykTq1u3bupJzGqVl5fTa6+9Ru+9957ndWGmSaWTBL4Gop\/feuuttGrVKlcf5OOPPw5ApFKUxd5Gat\/3VabB\/Lm3MReA8kqiikqiEb9X33eb9GAngDBZg9krcdLzyI456kOgER0cNxHp4YCcXkewuY0bNypy8dVh0DowsaAh4GscPHiQ7r\/\/frV9JW0+iC26QtNkJg6aHX0fmGow06BtRIN49JK5mxfz4YcPH\/ZcyIPmwOwUtqYggSx79+6lW265RcXWggM+Y8YMNUMCgogPYosuncuF36PMt0NNVP+nF2UWK1\/XOO3F2rlzp\/IX3n\/\/fXUdmFcCGZggINvdd99N33zzjfoO2gjOPqZ0dRNLfBAvVKP7veVvw6mx1ziaPG+VTPOasPOIb07zIngcZpz8bAUxCYJLKnEJJc6T8HVuN910E73zzjs5l17q5humecUHiY4Uekn\/WVGtfJALatbJQqHZBX5385qXTOqXfJoEwb0j5s1G2EbPt8XKVhM7RMhXahhbTdS8QamspGO0x76rHTt2KAxHjx6t1kKwobBQDYL3eR0ETjrvueJ7u1EWkmxWjBdJ4l6byFfSdY3gBA7PSPE1w24A6hoEz+lOv341sr4FJahNkHHvWKlfMAhEThC5QCeYjpNcokHACkGwLoHVckzFtra25iwUVlVV0aZNm3yZWNFAJKWkGYHICcJgh7GbN80dKW0PBwFrBIEvgjUK3K3Nh6ba29upb9++vtdBwoFEchUE\/o+ANYJAgyAG1smTJ9UqOJJcoCOiGTcErBJEQvzETRykPiYCkRNEv0DH7VbboI7cenX539fvcn1kSEUPx9\/nTM6c1pNU2ghEThDAyesSHKBBhzjooA1e3Xf+nS+6PnKmZz+P3\/t7FZHze\/dfD2f\/7\/7rEcL\/+Dxn96aC8imFh8NYSQ9yFR0YWyEId24UYX90QuJvMy6WjSO3S7Zkzo+3HD1B\/z56Uv39xd7OR4+HVJyjfvvDyL4ETVZqWku2u7sMc24LhtjJq++56spo6RUXK66bFTfv+i8dOHZaNf0f+0\/SgeOnqa3jf8ZjUJ8yGti7jH43OEOkgX3KaMpve3UFrkjflQNTHnDnC\/uDsx7FnEl3Ks5PXKyg1XIUUmZqIScNhHqwFhpSfg5VdWikuGgj0SAeGsScxeLFw2LOpOcryisuVsOfz6ILL+iINYUjon5S+eDOT+GIaUdyOl7qJ9ugn1n11f\/NtjZNA0EbIZkaKejykR+0XL6Ere5\/rfinHLl1Aog3LcKcwhZ1JD7DgWO3OJfO33el47wIMv+PFfTgrMx5aafYUk5l40ipU0JkEE5+8uKwO3rkELxfaPCDruBj813RIB7oY4EQW9NBFgRs6N+\/Pz333HP01ltvuZ5J1517fXFR92t4V\/D27dvpiSeeoO+\/\/17t+UIwB2yF57CmNpx0HRYcQeXIIfg+X\/QQkKjHqIklRR4hiAtBir1AZ8+ePTlnOrDlHRFSFi9erEIFcdAGPg8yYMAAFQtY3xqPYA8jR45UtYurk87Qtdd3xAbzih4Cs6\/DzIuLieelncRJd0EIoz3Cf2Jk1xP2Yr399ttZAfYCmU0omGU1NTWOJwoff\/zx7CkznEl\/4IEHYqNBvNrn9rsZu8orBCm0EBIHgrNtyokGceldjPo33ngj3XHHHbRt2zaaMGECff755ypOFmaw\/ByYQvZ8inDSpElU7Jn0JM5i+SWWHoKU\/SL2ofIRyilv9pfwm0409X9HDOBCCScE8dAgXT1yq08T6+sdErTBL338P5c19fDKj62ZF3\/cn\/k8yv93fOrZ8uxeRxA53fxD0Abk0at2kwRtMLsCPghiVg0fPlxFWIQmOXXqlPIJ4CPwmfR8QRvMOLzsoEvQBv9CH8WTdw7KTDWP6XUi89k7s3MA6XhZJqL7X\/5VFdj9IMgvSIvA2lYTCH5tbS3dfPPNdOTIEWpoaFB7tMaNG0dLly51NbFADkQrMS\/YkaANUYh8usqwRhDzllvMPsGP8NoCb94Bgu7iKV2QDqF\/MLUrQRvSJchhtdYaQSDMzzzzjLojBM4ah+XBNvd58+b5dtLDAkbyFQSAQOQE0RfzcMQWi3f45HTJJZeo+wn9xMUKowtBXCxUfvvttyoANnwkSelFwCpBAHvc4lThwp1+\/fqp8KWvv\/66irzid8o5vWJUui2PnCA6lPA3YFLpCdEV8V3QGsScBkaZ+pYVJioCXV9zzTWE8EOoB\/yboOtSuuJUei2zRhBdYLFQiC3un376KX3wwQeBx8XC9hRc7YbE50z0LSu4hQqLk9iugk+QAltUhCClJ\/CFtsgqQXBvx65du+iHHzLXIk+ePJnKysroySefDGzUBhFXr15NV199Nc2fP19NIesLiSAF\/A5ehUecYFybgFk1EATbYfr06VMorvJ8iSAQOUF0Jx2jNEwb7OTFCI\/tJ2GF\/oHGwH4tnSBNTU1q75a+yAjTCmYWroIDkbCIKSm9CFglCIiBWSx9JiusWSy\/BDEXH9MrGtJyK9O8tmB3IghW700TC1pDkiDACESuQXTonaKaRGVi5XPSZUpXyKEjYI0g5ubCsLvF1CD6NG+QUVTCbofkHy0CVgnite8qWiikNEGgMwLWCMIjOD4xlSpJEIgjApETRG6YiqMYSJ3yIRA5QaQrBIEkIWCFILz1AwuDhVzamSRgpa6lgUDkBOEjtLhdCoty5tHZ0oBVWlEqCEROEPNuQmiTLVu2qJA9kgSBuCEQC4LIKcK4iYXUhxEQgogsCAIuCFghCAdWcKpXWFtNRAoEgWIQiJwgxVRS3hEEbCEgBAkJeT3gnV6EHo4opKJDz5YnVmAJIEj4+PHjs7shzFlKszL4HUHGZ86cGdihuDAbLAQJCV09en0pbaXRBRw7nwslCOBO0sylEMQCQfiioJaWFpo2bRqNHTvW8UQlB8nDbuOJEydS79691ciLkdsMsYr\/9eMDrKmQB+KOIW3dujVnYRZrUPwbTnYi8dl8\/M1XSOiHyJBfc3Oz0hhOg4CuQVAe54\/89EXhFStWqCPWcT9\/IwQJmSC4A4MTT0DU19fThg0baM2aNeonMyo9jgLrRMB7EFYQJR9BEIrVSbiRP87+I1gFwhkxufA9CII6IPTrwoULVcC+5cuXU11dXfa7ZcuW5ZhCetjXfGYk8jbDOZnhYkFmpLhrVyFIyATR7XMuijUITjO2trZmtYdOJAT2XrlypRJghB3iSPb5CIIgE3Pnzs1pDbQIiMNEYJMIuxigBfhMvv4SCzJrHP0aPI6GWV1drUZ+Lw3CmscpljI0ETRMENfshdSFKlshSEjouvkgJkEwepsjtX47r1+COAl8vlt+8xGEBRewmAHCiyFIPk0hBAlJ8JKSrV+CcMBt+CIwN\/SLTPnGLN3Ewg3A7Bjjfkc2vSDMbEpVVlZmn0G8MScNYppYHPEF72KWaffu3Z1Iy+8wcbw0CEiYz4wSEyspkhxSPf0SBGaPvrtZXyh1ctJ1Z1x33t2cdCeCwPzRnfpFixZl\/QHz\/nodIn3kdyMIwiXBRESkfdMHQ5sRuJxNtZC6IJBsxcQKBMbwM3ET2iBLj2IdQ6Z5g+wxyUshEAVB+LTnkCFDVDikfBFeuuI\/mH5M3LtXNEjce0jqZxUBIYhV+KXwuCMgBIl7D0n9rCIgBLEKvxQedwSEIHHvIamfVQSEIFbhl8LjjoAQJO49JPWzioAQxCr8UnjcERCCxL2HpH5WERCCWIVfCo87Av8D1WmyaypKEHAAAAAASUVORK5CYII=","height":120,"width":200}}
%---
%[output:7a92d0b3]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAMgAAAB4CAYAAAC3kr3rAAAAAXNSR0IArs4c6QAAElNJREFUeF7tXX2IVsUXnjWT+idoKUrbUiOjpLAE+zAisiijIoPQ\/KjYpIyiSI3yI0P\/aN1MLQoR01WKqJWigu0PCUKo6MuShAozKqGooAz6oIyC\/fGMv9nmve\/MnTP3nnvfe+fOhWX33Xdm7jnPnGfOfJ7pGh4eHhYpz19\/\/SWWLVsmhoaG0pLJ71544QVx0UUXyb937twpli9fbsyzdu1aMWfOHPndL7\/8IhYuXCj27dtnLV+Vm1am\/u5kQV9++aXo7e0V33\/\/fYuMthe+\/\/77Yt68eWLcuHFix44dYtKkSTJpspwzzzwzVfbrr79e9Pf3C2CYpqOOx2OPPSa2bNnSJlpSlmQCCo6LFi0SDz30kMyq3qNkPPbYY1vqbcqUKWJgYEDg\/6r+9fyqLlS67u5uqWfSVmz1YsPYVCcmWU3\/S9qHkpeCqUkfyNLFQRBb5ekGpRQ3AZYEFqCjIh944IE2o06W6TIck2ErEuclCMpJygNjHz9+vJFgpooy4aGMR8mnG6GrlTI1IiaMiiAIZEvKrpNfl70IgiQbCZ38LkwzE8RVIfH7iEDICDg9SMjKR90iAi4EIkFcCMXvG41AJEijqz8q70IgEsSFUPy+0QhEgjS6+qPyLgS6BgcHh9V6BWXKVC8QU5x9fX1iw4YNAvPg8YkIhIZA16xZs4axIAQDx1zw4OCgXCByGbyacwYglPSnn356aNgFoc\/o0aOlHmPGjJGLgvisfuqq4O7du9lElx5ErWqrBbvp06ePrHSb3qQvulAXsUCQr7\/+mk1wvaBvvvlGTJw40atsah5XOtv3pv8n\/+f72UtBLfGPP\/4o\/vjjD7Fp0yaBv\/FTxHPyySePFPvvv\/+Knp6elteMHTtWjBo1Sv5v8uTJ4sQTT5SkxMLq77\/\/Lv7++2\/5WX9gk\/gfyobc+L1nzx4xbdq0ET30NNjdcODAATb1ug4cODCstlKgVHiRd999V26RSAqL7xU5sEKKh+pxiiQIGxoVK8hFTpO4MKLnnntO\/PDDD+KTTz7x1kgZOX7DLq688koBo1Ok0kngXXgJGbjtrOvQoUPDenfKp5vlk5Zb8BKw7vgrKAT57rvv5BiQQgZl3Oedd5644YYbxFlnnTXSKndcWSYBuO0sCIJQDCmJPzWPK13ZXSy05NhH5SIEyDB16lRxyy23SNWTLb9LL5u9UvOlpcuDGeRK65rWmiBvvvkmUzvRjGLgHdCP\/\/nnn8X27dvF\/v37jYqfcMIJAj+33367\/N3k54orrmAd6wbhQUI0CHiKNWvWGEkBb4Af7Hiu+pig7Lph9yC+g3Rd4TgG4a9+jCdef\/31toL1LlMkhR13doJkmeZV4lWFINR+sQ4rNY8rXZ7+tMprG1egu3ThhReKBQsWsHsKl15xDHIEga5LLrlkWJ2a8zF4ZPZJz83sLMaeJY\/LkPIQBPP5Dz74oNFbPPHEE\/KEnu\/6DtWnufSKBPk\/QVxbTTBjgkcd1YxdLKoJ2tPBODGg1h90m+At7r\/\/\/pF\/ZzXi\/BLWtwTuhri0zYrcgtexCrHCi3PSSWJs3LhRYJU5+USC+Ncyt50FQZAshkTN40pH6WIhWMT8+fPbiLFkyRK5ZUI9rq0n\/uaS7sWydN9ceNh0oXRvKdtzUE6p6yCuoA0miJMH4NMiiqj83MzmNJaiysLge+7cuW3EwPiCMhNFNcai5K9judx25u1BQA5EG1ED++RnG6jcgle98kAMfVMgCEElBqUVrrr+nZKP2868CKJ2++LciD5oTxvIN82DLF68uGUbSBZiRIJkp1dHCaLOgIAcemwpeBGQJO1cCLfglD5tGszU7osrHb7Hruddu3aJZ599duSVWMPAQTTsh6KOMeIYpP3YAmVcoqfhtjMvD4IThJi3X7du3Ui0QVQ+pZvFLXhVCPLhhx+2TYHPnDlTzJgxo2UA7hpcUr7P3q6253QR3\/Yuar5gNiv6DNKrShBOw6GWhfEFulPJccaLL75ILcKZjmqMzoIalIC7IS7VgxS1m1ftevWxA2oeUzp4UH1nLbpT8Kz6TlpTvuT\/fD\/76OdKS9U\/WQ41X1o623cUzCBPGm7su3lD8CBZWlpqHj0dTuph9k49GIDfd9994uKLL26zR9++c+xita9vmDBx4dRRD1LVQbqrtcz7\/aFDh8RNN93UUgzGGabtN3nflXdsxfn+OpbVUYI0bZrXttDHOc5IM0Kql6ujIRclc0cJAqXUKrpaPafMYCEft+B5W9o047MNwE0LfZStJkpW1zSu63tOo8pKPmq+Rs5iqQqq2lYTaqW5SGUiBgbeL730ktU2I0HM0DSaIFlasiI9SBZ59Dw2j7Fq1SoZv6lTTxbid0rWqryX2868pnnzgMAteB5ZVF4bMWbNmiUuu+wy0oZCDjlsZUSC+KPLbWelxeblFtzVXUqDFvPoKhyOSocp2xtvvFHMnj27JavLSGMXK\/AuVpmxeTsZehTeAnulsGdKf0CMFStWiHPPPddY05EgrbC48LBNSFAaNMraEcpJm8zgboiDiM2b5i1M3Sikz7PL1t\/xZ8tBNcZspYeZi50gvmF\/6hCbNy36YB2IQWmFwzTv\/FqxEySE2LxoaY866ih5IMkUkhOkwP3diETva3yuVjyOQQIfg9SVIK4YtSAFgjTfdtttxtkol+FTiRQJEgli9XtlxcUCGT7\/\/HMxNDSUGrQZpMBuzuuuu67jU7T5OwvmzXsc5YZcRq27WK7t7gjSjNN5uOgFjy1Ys6rg0IM2U7eWh2zwvrqxb3f3HaTrAvt4EFzC8tZbb8mbjl555RV5wYvvbUe2oM3U7hJlqjFZIa6yYxcr8C5WWbF5L7\/8cnJjgOu78Bw+fFj8+uuvQn0mFxATNhoBzvW20mLzNrrGovK1RcC51YQrNm9tEYqCNxqB0jYrNhrlqHxtEYgEqW3VRcHLQCASpAyU4ztqi0AkSG2rLgpeBgKRIGWgHN9RWwQiQWpbdVHwMhCoDEEQc2vp0qXy8NKkSZPK0L3W71Axyvbt2yf1WLt2rZgzZ06tdSpaeBW2Cnv68FDutakEQRDzt7e3Vwqt7h0pGqy6l4\/1qQkTJkhSKPzWr1\/fEnW\/7jpyy4+tUQcPHpQB\/yg3EuD9bARBJfX19Qnc893d3d2iW1qYILSEW7duFYhUuHr16rbI8dwgVa28rLjpeqiWEZf26NdSVE1XLnk4MINNIgBgf3+\/3CBre1gIotw9XpK8I4R6I5UtcjwXqFUshwM36JVmMFXUO49MeTHTu1mldLF074ATezpBfEKVNo0gXLg1aezGhRkIaosznSRvLg\/iOp\/uE+y6SQThwq1JnoMLM0UA1XhPnz49dXIjF0F0tpnOhvhcuNMkgnDghjIQxmjlypWpfeg83Zmq5s1qa3v37pUqqYkN021prB6Eo6LVlG4kyMDI5IarYdm8ebPYtm2bPIKsP5Q+dVWN3keurATp6emRwTs6Ms2bVeimr3lE3HyocSRtmZhVpovlD1MYOcqs7DAQC4ggPoP0UCrPVw8TQSJu6SiWiVmhHsRnmtfXsEJJb6rsiJs\/QYrCrFCCQE01PTd+\/PhUrRGY4Z9\/\/pHz0zFIQyj074wevrefpUlZOEEUSW699VY58\/Lqq6+mBn\/ThUWYn6uvvlpuQ8GDz6bHFZonTx5X2THsj9m8OnnDFOfMHhtBXG2FKeId4mLB6J9\/\/nnx8ccfO4mj4mIhBm8THhc5m4CBr47skRV97kn3FVZP7yM4iLN7927xzDPPpL4ShEG59957bxChRpPKRoL4W5yPnVFK76gHoQiINCDMn3\/+KZ5++mlnbN4tW7aI4447zlk01fhc6WIXq3pdLNbAcVX0IGnWDbLgwazFU089ZSUMvAvGPeeff35tvYuLnM5WoIEJCvcgyZNqixYtkgdM9Kdq10DjTpDPPvtMTgLYnmnTpoklS5bUiiyRIP4ML5Qgihw333yz3NCV\/Axxqec7kqpxC54GHaaJcUmn8jbJtBdccIFYvHhx5ckSCVIxgpgWrfSjiTh5hc1e48aNa\/EqaeFJlYpFEsRmSIogkM9289T8+fPlfSK2x2WkcQzSoDGIydD1bQ+4wmDhwoWSHPrRTsr53k4QJFl1IAl0NHkWjFkef\/xxgR2f+hMJ0oqiCw+VupPrIIUM0m0HSPRu1tSpU4VpD32y22VqU4okiK8jVgRBN8tGFqy12BYmfd+XNT3VGLOWH2I+bjsbmeZtEkF0wwBBcAfJXXfdZbSXRx55RJx99tkdIUskiD+FI0EMmGUxJFsedMF27drV9hbb9dFxDNKQMUgZHsR1R6F\/e1FMDtyViGfdunVC\/a2\/Cesr55xzjsAdiUU+8Y5Cf3TZ7yjUFwrrOkj3h5GeA12wDz74QDz55JNkr0IvPT1lFs\/I9e66llNYFwuA1HWat4zKdA3s7777bnHppZeyjlUiQfxrtlCCqBkrTOFiKte2UDhv3ryRuKaUGSyoyS24Dl0WQ6LmMaUDWT799FPx6KOPGr0KIvaZ8iX\/5\/vZ31zsOaj6J0ug5gtumlcBUcetJtRKy0KqtLJBlG+\/\/VZs3LjROF2MhuSOO+4Yea0vIbLoRSVR1rKp+YIlCBVg33RFehBfWYpK\/8Ybb8go68kHM2Ag0dixY71eTTVGr0IDT8xtZ7XY7u6q0yyGRM3jSpf8Hl4FP+h+mWbA9OliX4\/iwsHne5detrKo+aIH8amNwMYglK4aDAR717766ivx8MMPG9HCtp0FCxaQu2CekKcmpxp6HIMMDw9zAm8ri9v1lSEz5zvgVRAq1LYIuWrVKjF58uSWV2Y1Yk6561YWt50F0cWqUyW6povRBcN61JgxY+ShsIkTJ9ZJvY7LGgliqIIsLS01jyud7XvKNO+ePXvE8ccf3zLTpauHlXoM7k899VR2w3PpFccgRxAIwoNkqWxqHle6PATR88KzvPPOO2LTpk1G24RnwSEw7Kjm2GXs0isSJCCCsDevFSjw7bffFthJbHvUbBi+5yBMBVRmESF2sVhgrEchaiYMESdxDsd2hFiPF6ZijdVDQ34pI0ECGoP4roO4jhAraCiEiV0sGjnjGMSBk8uQuMYgEMNFGJOohw8fFsuXLydFpbznnnvEGWecIbtkLr3iGCSOQWhNSAdT+RgxNV6Yrg6IctVVV43cUd9BVdleXYkuVtXiYrGhW7GCfAhiEx3EsUV1MeVRA\/5rr722ZZW\/YtBYxek4QaoYFyuLIVHzuNJ1uouVxXBBmjVr1oj9+\/d7ZQd5EHNsxowZ4s477xQ\/\/fST7K6ZJgYauRcrzyUl3MzWa9ZlxCYroOZxpasjQUzjHRj5F198IV577TXneCZt6llNO59yyili9uzZ4rTTTmtLngcz11iN2868Bul5rgbjFtyr6atpYhc5i1RL9wr4++WXX5YbL00B+LLIobpy6jfO+F9zzTVi1KhRudZ1uO3MiyCu64l37NghbLfWcguepVLqlqeTBKFiBfKcdNJJYvv27fKUpdruT82fNZ2+OKr\/jQXWjz76KGuxbfmCIEgWQ6LmcaXL011wTeu63p3HCrKWTc2X3EajjPi9996TO5p\/++03KX4RhMLdMlxPqQThEjqW0wwERo8eLRXFxMDRRx8t8PmYY46RO53Vo9Ko3\/h\/LQnSjCqNWoaGgJcHyTNIDw24qE8zEPAiSJ5p3mbAGbUMDQEvgkB5tYrOeRd1aKBGfcJBwJsgOkkUDIos4cASNYkIHEEgE0EieBGBpiAQCdKUmo56ZkIgEiQTbDFTUxCoDEEwhbx06VKxYsUK63aVplQKRc9kDGWEPMXNxPGxI6BmYYeGhmQiyti5EgTBHq\/e3l4pdNp+rlj5\/yGAMx4TJkyQpFD4rV+\/vuVy1YhXKwK43uPgwYPy5gLKxbOsg3RUUl9fn9iwYYPo7u5ukSztgBVawq1bt4qZM2eK1atXy1udbBseQ6zwrLjpWKiWce7cuY0gCAdmsElcU9Hf3y9DxNoeFg+i3D1eMjAw0EIQ6gEr207hEEmhdOLADWWlGUxo+OXFTO9mldLF0r3DlClTWgjis\/LeNIJw4daksRsXZmg0bNumkg1KLg+iBFZ3YgwODrYQxGfvVpMIwoVbkzwHF2aKALZLa1kJohdmut\/Q54BVkwjCgRvKQLT4lStXpvahQ+tiQZ+strZ3714Jh5rYQDA+15g3lwfhqGg1II8E+W\/s5mpYNm\/eLLZt2ybUdKWqB0qfOgTCZCVIT0+PWLZs2QhuFLwqQ5AQKi6LDlkru0kzfUlcy8TsfzQkidREcrITAAAAAElFTkSuQmCC","height":120,"width":200}}
%---
%[output:8429c053]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:09179d2f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:154cc966]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:6187332d]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:3efa50f2]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"  -9.869604401089360"}}
%---
%[output:3a7c206f]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.998429203673205"}}
%---
%[output:507a9885]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.149016195397013"],["36.521945502464568"]]}}
%---
%[output:20e348d0]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:7026af53]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:7d496611]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:0988a301]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.155508836352695"],["27.166086113998457"]]}}
%---
%[output:1fc7b071]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.037191061070411"],["1.937144673860303"]]}}
%---
%[output:4c5eea94]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAMgAAAB4CAYAAAC3kr3rAAAAAXNSR0IArs4c6QAAE4VJREFUeF7tXXtwVsUVP1SnEp8hgEpAJuERpaUCyWAjMk07qBnbMdNC1YSOMkxEWqnOFNKElDotHR4mQ+xYizVmAGmnPNT6INN2ZKDYqUarhodTBw0aQoSABWIEx6BS6ZyL52O\/zX3s6977JXfvP5Dv7u45e\/b89uzZPffsoDNnzpwB5tm3bx\/MnTsXioqK4MEHH4SsrCzo7u6GyspKp9SaNWsgJyeHrQKvvvoqzJ49GyZNmpR6v3nzZqitrYX58+fDvHnz0upjZbf2RNvZs2cPbNiwAYqLi1O0c3NzYd26dQ5fPP91dXXQ2NiYqpPG\/Jd\/9Pb2wuLFi6G5udn55dZbb031n2TS1dWVaoNkQrwUFBT06RNPl5VJTU1NSq7UxtChQx3e8cG+jB8\/vk\/\/jh8\/7sia5Y\/tD\/WjtbU11YZbf3leSPY4Xsgb9Xn48OGpMaX+UBm+Dsnk6NGjvrRZfqhNtj\/EG\/2G8sE+E12+P+zYrVy5Eu64446UbImXnTt3OvpI77ENln\/SUZZ35GNQfn6+AxA\/BQsCCK9cbAdQkXnl8QJIUDsIQFaJWToiAOEVnxc0DQz+zgoS\/6aB5OvQICLvPOjZOlhu5syZKQDw7aCc3PqH\/UIlpYEbNWqUpwxo0uCBSYrgBhR+oiFdePjhh2HZsmWAysk\/PEBY\/SFF9KrD\/85OPm4yYSdBL4Cwyu41Plu2bPEFyP333+8qVwcgpFw4Y6lYEGTKTblJ+DzAvAAS1A6+Z2du\/PuBBx6A3bt3A82Y+BtvQVgFYa0cL8ygGZAFENZlB8xtEnGj29TU5Fgzql9SUpI2O7JyxHFZvXo1rF27NtU\/HCM\/WWO7xCcLcjdFZN\/zcsWxwwdnbnxQbqtWrYKqqirnb1xJ0DgSiGi8\/eTkBlKeNquPrPL7AcQNJG5WycuCsFYTVwok+0H8EsutA\/a3aCTgtpSj35ADWnYFcUN1UEFw4O2jLgFPgKCQq6urob6+3lkLe822NHt4rYnVWUteTb8lpqx8cZmFFszNZ0yeZNV77AqQIEeP3k+bNs1xiOhvNEt2xlIfDKzJLzfwN1lwYB2yIvfdd58zRvZRk4ArQHDmwRkIHy8LwpPDdWdLS0tq50eNHVvLSiCzJNAHIDiDLV26FCoqKhyQWIBk1oBZbqKVQB+AoCXAp7Cw0NcHYdmkZUF5ebmnOR8zZky0PbPUEiuB7du3Q35+vpH+pwEE163r16+HJUuWwMGDB4UAQv4HckMHi26cIUDa29uNMC3biKUtKzH98gNF5mkAwSUV7svj4UzQLhaKUBQcWHagCExWdWy\/ZSWmX96kzFMAcds9IVbpAIhlXXbnyiTTsiK0tGUlpl8+Dpm\/9G4PlD26C7KfqzS2WlE+B0FrgyeOfssqVsxxCIzoW9r6Ci\/bQhwyjxUgZDFwd4tiq\/g4Hb8wjjgEZgGSLL8vUoDIzhhB5eMEyP79+43tagT1k39vactKTL38xtePwIKNe+HSrTXQ8Z\/X1BtiahqJxRJx6C1AjIyXVCNJAycBJBIfRHQkgsJSMmGZkzRFIZknrd8ZCRDRsBRrQUSnHHPlkgaQzu5TMHnZK9HsYokMk0xYigWIiETNlkkaQOpe6IC6F\/ZnDkBkwlL4UBMMB4jqwagA\/BIvjsfSDlfqM2bMSBHonXg7fDquNDMAIhuWYi1IuIri1nrSLEjZ6l3w0ns9mQEQ2bAUCxALkLAlkLNwh0Mi9l0s2bAUZNoCJGz16Nt+kiwIHRJmBED4obDnIN7KnyQlZaUQZb9x9wpjsPDf0TmD4cTaH4UfiyUz31mAWIDwEogKIAiKn27c6\/ge+DTd+TX4+azrMwsgImCySywRKZktE5WSxrlBgOceCBJ80Hrs\/uX1RpfzaaEmfFaNoDxEbP6joMQCFiBmlV+ktYEMkO1vd8Ntj59LakfgMO3v9vlgCgmwaTG9PqOlE3RMK4PpSTHjoV9WEwsQEZU2W2YgAqT22X3Q+K+DaYKaPi4bfl8+wbEgoQKEHx7KbOKWyofPYhKU1cQCxKzyi7Q2UADyzK7\/wm\/++l5qKUV9R0AgMBAg7GNS1zyjeWkrF8GBn+Dyj5sFoTxZboNnkmkR5YhrRyUuZzVOP8A0bd7x5ttHYGy5d0rKYvDvTeqaK0DcMm67CcEtVaaX8tpQE1lY65fvD2EuXSdOOx399bZj0HrorLPt9uReej4sv3k4XDvigrTXbKgJ+8JUghDf70H8PqvFJdWmTZvSUuMjg16ZFU2iWlZ1BspSo7\/3m3ab2G1Zrz6RP+FnKfwm40gA4nW+waceRUaDzkIsQGTVW798nBPDy3v2wd8PDII33z+ZOqPw6xECYvSQwfDb26+GscMv1Oq8SV3ztSCsn8FemmMBIj5+cSppFLQPHD8FDds6oONYrxAQUHJkHe4qzoUfFl7h6UuISzm9ZGgAYXetgtL6uC2x\/LKcmGRaVnBRKIoXT\/2dNi6L8A4yWRCQPMgyPH7n1+Gz018YB4Ob3E3qmu9BIXv4x2Y1oV0t\/galTM2s2N+VVHZCoPIi\/Sa\/YGfnCVj78iHo\/PBUn+1UEfoEhLpZBXDRV8+D\/310OLZEGZEBJOgkXfTmJhSwSaZFBowtI6Iosm2Klo+b9nmXjXBYfePACXii5ZDzf4pbEu0Dbw1uK7oSSgqG+FqDOPttUteUT9Jpixev5UKLYg8K3dUtbEVxlkAA8Ng\/34e3uj5WtgCsb4DO8i9uGQO52RcoL4nC7rcfuEMDCE806CS9o6ND+MIck0zLznxxDpYObQrffmjbAXjxnW6t2Z8HwIwJQ+EHky932iSnWVaufuV1+q3Lh0ldUzpJd9vFCuqUSaaDaPHv4xwsN9q07h9x2QVOCMWe909qzfzsEgj\/f8PYbJg55QoY\/OlxuGFS3+vzZOWnUj5OmZvUNaWTdAJIaWkp4K2tmILUL+1o0nwQBMD+Y73wZOsRaOvqgaO9oOT4sopJszwuf75VMARuL7oycOaPU0njpB06QGhgvE7SCSCdnZ1pJ+lB27zsgPe3rCYYEoHhDvu7P4dn3joJ7xz9DA6fPA0UKqEyy1IdbBefEZecD9+8Kgtuufoih5bu0x9CTXT7GGuoSVJO0nHG7\/38C2ebc+9hPUeXH3BU9DHDL4arcgbD974xHCbmXhw48+sqDdWPcxaPk3ZkFsTrJB0HAK1LXl5e6so1BNOKFSugoaEB2FN3GiyTTMsoECr\/Ey++DW8cxr0e9S1Onia75Jk+fghU3ZQHBz8892Vb0pV0QAJE5iSdB4\/fjleYPggC4OmdHzi7PKr7+15r\/YkjL4Z500dB\/rCsVEIAGXBi2TgVJam0TU7GWifp7EFhVJ\/cbnjtMGx6\/Yg0GNgZf\/JVl8Cim\/Lgo96zodZhbHNaCxLflROhAUR2dpQpr8N02wefQHHdv33JUajDhBEXwb0lo2HQoHOKn9SZNKn91tE1Xsm0kjZQY0Gh7ipLLDorwKwVbuv\/bxcMgYU35gnN\/klVlKT2OzSAsH6EyN3nqLhh3Q\/CpnOhZdBzP5kMeUOzZAyXUzapipLUfocGEF7zghxvLG\/6fhA2Sx4BQ+WrMrYvSVWUpPY7EoAEJW1ABTR9P0gY4LAWJF\/a4pqoECc4QweIaNIG2ftB\/L4T5jNZ3H3DSKifVWBirOwSy4gU5RoZ0AAhUfglbVC5H4QVMR9qghkt7nnmiFMET5+b55i78CYJIRdu6puEfmdkqAkOhun7QehuBzaFpNyc5V06ztnM0jY1iuLthL7EIla8Qk1M3w+y\/G\/t0LDtgEMWHXI+U564aNxLWiXVlaB8\/ThlHhpAZEJNWJHpnoOEaT2sk26ddHl4n6uhFWpCzegAZOve41De9GZo1sMCxALEGEB0Ggqq62X2wrYeFiAWIEG66fdeOdSE90NUgxUJIGH4HtTxONfDlraOeqrVjcQH8Qs14b9JD0oyh910Y3pW4x7Y8WUyArwZKKyoWqukaoqmUytOmYcGEF4gIqEmVEc27Q8eDFIg4vSx2bBlwRSd8fCtG+dgWdqhDatnw5EARCTUhOVQByCrKyZAxdQrQ5OkVdLQROvZcJwyDx0goqEmJB2RyF+eabw16O4\/veU0Eebyyjrp1knXmR6U7wchouR\/4N9BuXlZRscu+LNzYYrpsBI3YSQh5CKp\/c7YUBMcEFFw8E56mv8xLts5PQ\/zidPcW9phjqx726EvsYisX1YTkZ0rln2WaRYgYW7vEn2rpNEraZwyDw0gMqEmfpG+bsPBMr225RBUPd0Wif9hfRDrg+hMD0qhJgUFBVBZWemkHGUfv\/SjLEDKVu9yspKEEbnrJow4ZzNLW0c91eqGZkHU2BGrxTJNp+dhn3\/YJVZ8qXfinBgyAiDkgzQ3Nzt6GHTZDjHN+h81pflQU5onhjCNUnEOlqWtMXCKVTMCILIZUIjpl97tgbJHdzldj8JBtz6I9UEUceZU8z0HkWk4KCyFAPLEK12w8Kl3LEBkhKtYNqnWKyMsCDtmImEpxHTUDrq1INaCKM4vZiyIaFgKAgSfEzfXwRcXDoOikYPh8ZnhxV+xQrEn6ToqolY3KpnHepIuI5qgcxGyILSDNeOaofDUPdfKkFAum9SlRlL7nXFLLNTcoM9ukek\/bt0ZuYNul1h2iaU8s5p00v3CUpBBBMjyzS2wYONeh9+wI3hZoSR1Jk1qvzPCgsiEpRBA5v\/hH1D3wn4LEJ0pTaKuBYiEsDyKKm\/z8geFIt+kT6l+Fna0nb3vu\/uh7+hzL9hCUhUlqf3OCAsiqJupYsj0xEV\/iTQGi4gnVVGS2u\/QACKbqQQ\/s62trXX0UMSC9Hx\/jVM2qhgsCxAbiyU7kfPlU0ss2UwlrFOelZUFixcvhtzcXKipqXHlKW\/idc4ZCD74\/Tl+hx7Vk9SZNKn9Ds2C8Arrl4iBfxeUtIEFSFRBitaCWAuiOwn7Oul+Su9mQaZNm5a6N51nbPR1t8DH06udn8POYsLTTupMmtR+R2JBRDKV4OHg3LlzoaurCzZs2ADFxcWegM397s\/g1DVlzvtLt9bAjubNuuAWrh9V2IMbQ5a28DApFYwl1EQkGQNal02bNsGaNWsgJyfHuS8EHy8fZOSN86D32tlOmSgPCZFeUmfSpPY7VAsiAg7eoUclDAo1ufyux+D0sKsdgER5BmIBYkNNlEzTl5Vcv0n3243CejoAieo7dFYoSZ1Jk9rv0CxIUEQuq3RuSyz0RbySxw378VNOmHvUZyDWglgLYsSCeF2rRplK6KyjoqIi5YzTtyDIQNBBYdSJGqwFSa7vFZoF0UFaUF0CSNRnINaCWAsSpJt+79N8ENlQEzwLmT377M6UX04sfB\/FRTleHU3qWjyp\/Q7FgsiGmtAZyKpVq5wlV9BJOgEk6kNCa0GsBTFmQfiGgkJNOjo6PM89+LasBdEZJrW61oKoyY2tpRRq4rbNG8SKBUiQhMy\/twDRl6knQETuKCwtLYWmpiYnR6+oD4JhJl\/55Bhs375dn3vBFmy4h6CgDBaLSuYZGWpCFqSzszMt1MTvHIQsSNSn6NYHsT6IDu77WJCwQk0QIHGcoluAWIAYA4jMpTh4SJiXl5cKb8ddrRUrVkBDQ4MTvOjmpFuA6AyVfF3rg8jLjK+RZkFkQk34ND9B0bxoQeIIM7EWxFoQHZikAKISasIeFIqEmliA6AyVfF1rQeRl5mtBZE\/SqbGgUHcshxakfmYB3D19pD7Xki0kVVGS2u+MOEknHSW\/pbW1FdatWwfjx493VV8ESBxxWHaJZZdYkvNpWnGlg0K2BfJF8Lf6+npPgOAnt0\/+7lcwfVy2Dr9KdU3OKLIMWNqyEtMvb1LmWgDBJdnSpUsBQ+DRSfcDiEmmZUVoactKTL\/8QJG50kk6iQ9jtfApLCyE6upqCxAXvRooiiILmYHSb1eAiBwWomO+fv16WLJkCWBYgQhAZIVsy1sJqEqgvb1dtaq\/DyICDmwBl1QlJSVOqLvILpYRbm0jVgIRS0ApaYPXmQnyHpQfK+L+WXJWAloSUD5JZ6laC6I1BrZyBktA6ySd+mUBksEjbFnTkoDyBTpaVG1lK4F+IgELkH4yUJbNeCRgARKP3C3VfiKB0AHCJpcLa4eLzTIfFFXM8oMpVv3ix0TGUIY2tafyTb8bLzK0+Z1H3bGQoc2WNSFzv3Eh2bIJDkXG0atMqABhvxlpa2tzzk4oG7wO02xdVtnKysqcm6687ilxu\/SHzVAvy5MMbbZturpu5cqVnvepBPEiQ5v\/EE53U0WGNgETs\/5TeigdmYuAo7m52dhxQ6gAcbsq2hSyvXbQEJQbN270zBFscnuaVzQR2qgwixYtgp6eHigvL1cGiAztoK89g8DIv5elzUZZ6ILTi1eyUkVFRYC5EgiQsn3jy4cGEK9EdH63UKl0hv+ykf\/br03dwVKhjZPG1KlT4fnnn\/e0dCJykKEdlNRPhB5bRoa2mwVpaWkRmsBk+Dp06JBTHHNIV1ZW9h+A8Mmu2e\/YZQTgVZaftWVmS5lPjN3oy9Km+LWFCxc6UdA6k4UMbQQIJvnDp7Gx0flXxweRoY20aLLEpc\/8+fOFkw2q6AcPSJU22DqhW5BMBQgqzSOPPKLlpMsoCirJ8uXLYc6cOTBq1ChfX0lkUGVok89DoMC6VVVVyn2Xoc2nqJWx8CJy4MuYBsj\/AaNT9pmyJnlYAAAAAElFTkSuQmCC","height":120,"width":200}}
%---
%[output:40ce6ca5]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:380d5b63]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:4d5237fd]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
