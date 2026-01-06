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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANYAAACBCAYAAAC8aERhAAAAAXNSR0IArs4c6QAAE1BJREFUeF7tXWuMVdUVXmOnBjAgM3RknCePqCmIJAQj2oikliDxlRiLjD8EVDQEQitQeanDQx0h0hYVkZcDiQoWYlJAE5xoJWkRk0pEpKAWMozDYMUZECqQOGWab8+sy75nzr3nnHv3edxz1\/lzZ87dZ529v72\/u9Zee++1Cjo6OjpILkFAEDCKQIEQyyieIkwQUAgIsWQgCAI+ICDE8gFUESkICLFkDAgCPiAgxPIBVBEpCAixZAwIAj4gIMTyAVQRKQgIsWQMCAI+ICDE8gFUESkICLFkDAgCPiAgxPIBVBEpCAixZAwIAj4gIMTyAVS3Ir\/++muaMmUKtbS0JB65++676YUXXqCePXs6ijl\/\/jzNmzePampqaNSoUY7l9+7dSw8++GBSuccff5zmzp1LXmU5vizPCwixQhwAINaTTz5Jy5cvp2uuucbz4PZKBhBr2bJltGHDBiouLk68r6ysTJFLLnMIhEqst99+m+bPn5\/Umrq6OnrggQfMtTDCkpyIpWsY1ixoDsixZs0auu2221Tr8B00lo6nXp4hsBIL91GH559\/np599llFcGi\/4cOHK024Y8cO9ShrUfzN93HvzJkztGDBAtq3bx\/t2bOHmpqaaOLEiVRdXZ2kGd966y269tprafbs2TR69GhaunQpgcwvvviiasv+\/fspbv0eCrF4wNiByYMDneHGvIkwbxyrZmcKMiY66SoqKtSAvuWWW9SgZa3T2tqqTEkMUFybN29WZiQTwGoi2hGrra1NDfhZs2bR+vXrFbEqKyuVjPLycuLvdQLhHSDDnDlzqL6+XhFry5YtCU2I93DfoT8bGxtp6tSp9Mgjj6j70I5oA8pBezY0NChiujWBHYGNQIHAiYWOwi\/hpEmT0jZ\/06ZNjmUigF9WVUilsZhATBTMt3iA8gut86Jjx4510\/5WreWWWBj8bGair6BdVq9erYiHusGiSEU4nhtatS0TC\/Vm7QrC4X+U1duaFagReThwYkWk3ZGohpVYqBQTCGaeV2LxQE3VOLemIJ6Hk0M34VijORGLtSU+oYG2b9+epLGEWD4OPd0EgumDC3Mt\/DLCtMBEPh+udBprxIgRCceGW1OQTTO9vD5fTee8mDlzZsLDqJuVVpOPTbZU93UzlOdq6FfRWD6PaN2ThQkt7G5MeDEA8GsdN1s7HZxO7nY\/nBdu3O3cLyAPyp89e7abU8POecFzJJ4ng1Do388++0z9SMyYMUOZfmIK+kAy2OaLFy+m2tpa5fLFJBZmD8C2fufD60WkIQTszEpDomMhJvA5lhArd8eNdXkkHzy3mfaWECtT5OQ5QSANAqEQC3Y37He7CxNe3hkgPScI5CoCgRMrV4GSegsCXhAInFiYY4nG8tJFUjYXEQicWDpIvBjKG0DxP6582SuYiwNG6uwOgdCIZedaF3e7u06TUtFHIDRi8UIxb5EBVFjTwtmkOG3GjP4QkBr6gUBoxEJjrPMt8Qj60cUiMwwEQiVWGA2WdwoCQSAQOLHk2EgQ3SrvCBuBwImFBstBx7C7Xd7vNwKhEIsble9H8\/3uXJEfHgKhEiu8ZsubBQF\/ERBi+YuvSM9TBIRYedrx0mx\/ERBi+YuvSM9TBEIllr5AjAhAH374oYrMlC8xL\/J0zOVFs0Mjlr6lCdGFOPhk3MJg5cUokkZ2QyA0YukbbtetW6eIhSAmejwMU\/2lB2WJW8RVUxiJHLMIhEYsO421e\/du45tw9cCSgA7hlFesWKEC2cglCPiFQGjEQoOC2ISrRxNClFZrdo5Bgwb5ha3INYTAFVdcQePHj6dz587Re++9Z0jqJTFHjx41LjNUYhlvjY1APXwxvuYY6HyY8uhvC4Kohud3FJYMSHqm8KpL\/\/+86zsuUzSh1rP8XHrghx9+UEkgrrzySpUAwvSFH1fT5AqcWEEfzXciVvGsv5nuJ2PyHjuzKSHr6vZvE3+Xtf9H\/X31\/76lMu2+\/uKWwlI68bNSainsTycKS9VXa\/ukj5dvrOI+CVo04IDKfGL6uv3223OfWDooQRzNd2MKmv61Mt3xXuWd\/WgjtX93TD12\/l8fUft3jdR+sjFZA5YMIGjBnkPGqPtR1nqisTyMAD+O5lszFsIDOHbs2ESYY1TvuuuuU8H+2XnhhxngAYbAi144+BH9dLJREQ9kw9+4p18wMXsOHUP4jALhhFgehokfR\/OtGhDVAYEffvhh+vzzz1XtbrjhBnr99dfzllhOXXTqL4tTajoQjbVckIQTYjn1muV7DHBkEtSvp556ShEhkwsxMwYMGJAU5SkfTcFMsHN6hs1LOy3HThRouZLp9U6iPH8vxHIJmR8HHVO57r\/66qtumQ71ADb5Zgq67CLXxWBGnj+4O6Hl0pmVhVdVU+8xk13L5oJCLBeQ8dF8eHfsAndyQBk3WR\/TvY5TAt133330zjvvJKUQtRLrgw8+cFFzKeIJgVPNREc+oQ58Hv2E6Mje5MeLKogG30RUVEEFg24iGjwqpfgff\/xRZQHFepZ4BT31QmdhkAE5dt3kHtZzS3HyaU7TCVlsAk6bNk2l+UQ8+FQLxHHzCmYAfeCPYC4Hj6Wu4VLN4URjeeiedOtZpaWlhBzETrvcdWcF5C1cuJCam5vp4MGDNHjwYBozZgyBWJz8DNXLd6+ghy4KtKhuUp7auki9G0SD6XjZuN\/LArGX3rB68eAex5aVp59+mt59911auXJlSnFwVGA1Xk9gjXSfO3fuVM\/07dtXmX8jR44Ur6CXTolQWWg15TQ52UjbblwmOy\/c9M2RI0fo\/vvvp9OnT1NBQee2oo6ODsIcC5tkX3rppZTEYpMRm3ZxIfY7a0BrKk7smgcJxRR00yvRLNO4fhZd3PUnavjNa7KlyamLeB0LeWr1pAjIQYx8tW+88UbifipZIIxOrNmzZ9OCBQuUCSnOC6ceyJ3v4bzotXgY7bx1pTgvnLoNGgYkOH78uJoT4Ro6dChhflVYWKjmR05ZR0wRS5wXTr0V7vfivHCBP2squE9h+uGCKXjx4kX1CUK9+uqryoOHSy8P7VZfX59waliJBSdFJqagi2pLkRAR8PvYCJpm+sc18N3t3D\/IKjJhwgTlWDh58mRGR\/N1YkEu77y45557EueucCqZTUSUkYOOITIkj14dOLF48y2CxkDDwHmhX\/Dmbdu2TbnLnS4rsXQXvu4t1DfnSqZ3J1TlexMIBE4srnRQR\/NNgCQyBAGvCIRGLFQUm3CXLl2a5G6\/\/vrr1eKwxKTw2pVSPkoIhEYs67pTlECRuggC2SIQOLGCPpqfLUDyvCCQCQKBE0uvJLx1ra2ttHHjRnUbi7q4nNavMmmoPCMIBIlAaMRKtaXJi1cwSKDkXYKAFwRCI5YfR\/O9NFzKCgJ+IhAasTDXwlrWF198keQVRGPLy8uTdlj4CcD0zYe6ia8q7tz1Yb3mjkuO9ednvUR2biMQGrF4ToWECPomXNzHYUckR0h3bMQU7Fc99Fo3URd79XMUf7HXLxzL2BW47Nz36vZl51q7Pr9Xf\/c4\/NeM5OXDQ35vaTK9nQl9Ejix9KP51gQIvCsD56qeeOIJ2r59u+\/jJoyYF8t2NVJT23nVtm\/aLlDTqQvU1HbBRnP2UPd+NbgvQYtWFfegmhs7g2\/m0yWbcF32Nm8xQiiym2++OUljQVMhVNldd92lzmSZuNJlGwGxohjzYseh\/9KJM+2q+f88foFOnG2nlq7\/ca+sTyFd3buQRpZ3ku+xm\/qagCqSMiTmhcdusQt\/hrNYhw4dIj3gi0exScWdso2EobGyaQ8\/C62H6x\/\/PkV\/P3JpvyW0Gmu5VTW\/NPGq0GWIxgq9C7pXIN\/iCqYiHCMD4lUV9aBKfHY5aaLulBFieSQWx63QH+PwZ6b2CjolRchVjeURalWcSYf5Xbq5nS6bNSDugZB8gZjJ5ZI9qSbJKsTy0Nu6ibZv3z7lCTx2rDOQv8mdF07EMpbGB3Hy9KtY+7+ovPObrjIFY3\/nAanwi679pNPUbDnbOedTf2vzPb6HeaDdZVfWa6v+ULJX4gq6AU1PioBotQgMM3XqVOOpUoM0BTnuObefM3wg8UDink3mD\/6OQzUjPjpyYHXG2csseqybPsiVMqKxPPQUdl7AFPz0009VzIuioiIVacka98+DSNuiueS8YGKmywKCRkYpE0i2\/ePmeSGWG5S6yoBYOOWLuH\/4u6SkRMW6gAt+1apViZgXHkSmLJruBHFU3e12jelo6Iqz6BSyOcdMTac+Fne7E0La97opuG7dOhXzAvEprIvGdiKxC37+\/PnqK93ZoR9J0cNOO61j+bHy7gGKrIvqqXfskhLopmWQ6XeybliXANFYHpDM9Gg+YrbrAWFgTiIwDaLeYkEZaXy8BJOJs1fQmnonVWZHdBuTT\/0dkYRzPJyEWC6IpWsVPfwZHsX\/Xo\/ms9cPC8szZszIKPxZrmssF7CnLQKNx\/M6FLQjoFWAnnycE4\/rScdNakYhlose1omFWIL33nsv1dbWJlL6eI2ixCHPkBJVIuG66ABDRRLzPchDqh71ebzzs6350j279\/HSBJYkkMIHm1bTzAsxx+qoG03fVd1Kgx77s6EWXBITq+TeMAWfeeYZOnDgAB0+fFgdHYFHcNiwYbRkyRJXzgtrtpFMiZXvGsv4SE0j0Jp4HEVTzQuReByaDxrrm+kD6dSgX9OtS7YZr64f04HAd7czKlbnBeZGcLnjCIkeVzBVJFxrWtRskiIY7ykRmBUCD5Wdpv6X\/0TDe1+g0ssvLTr\/sf90lY3Gj8v0j2toxAJhMC\/6+OOPVdwLXAgv3b9\/f2Ua6knkrECCVPAiWhPUSSRcP4acyMwEgcCJpc+xWAVjPQu7LnRTLlVjdNc5l2HXOsjK6VclEm4mw0GeMYVAqMSya4TpTbimgBI5goAXBAInll65IHa3ewEDZaH1tm7dSl9++SU9+uijNHDgQK8ipLwgEPzRfN15waafn7vbvfbx+++\/T\/369VOpgt58802aPHmyKw+l1\/dI+XgjEJrGCmp3ux2ROWm4vjWK18\/Wrl1L48ePp8rKShVDHvM3U2fD4j2UpHU6AqERCyYXtiDBaQGv4JQpU9TWJD\/mWNgGBfm4OHGdvjUKx1YQawPbovAJMmFTsBBLyJIpAqERCxXev38\/9erVS5ld8PbNmTPHeDxBaEZs8r3jjjto0aJFtHz58qQcxSATSM5mKY6wDBkyRO05BLFw6LJPnz6Z4ivP5SkCoRIrSMyhobBuphOLYxrqi8swAWEOYrEahL\/zzjuDrKa8KyYIhEosfY7DePphCkK2W2JZF51j0s\/SjIARCI1YQefHsiPWnj171LxKNwXZsRFwP8jrYoZAqMRyc6jRFN5WYqVyXqTbSmWqLiIn\/giERixAG2Q+LCux+P04iVxWVmbcaRL\/oSMtTIdA4MSSjI4yIPMBgcCJlQ+gShsFgVCIxQu2WBDWg75IdwgCcUEgcGLxwcWamhp1nsp6YDEuwEo78huBwIml7xHEHjxor127dqlAMHIJAnFBIBLEwtahhQsXyi7yuIwqaUfwx0bsNJYQS0Zi3BAIRWPx8Xk7MP3a0hS3jpP2RBuBwIkVbTikdoKAGQSEWGZwFCmCQBICQiwZEIKADwgIsXwANZ1IPQCpXk4P1xZwlYy9jpdOMIeeN29eUoJ26\/ql9aX6ifI4hEIQYhkbVu4E6VlWTKaEdfd2\/0rpxMAJAa\/EQs3itKYpxPJvrNlKTkcs7PbHGbGmpiaaOHEijRgxwjYWCActxa58RA\/u3bu3ih0CTYEQ3djRose11w+UsmaEjDVr1qg6Ik2tvrVMD0uHIDu4OCYI\/gZpeOcMNxLykEMaPxZ2bdQ1Ft7H78bz+rtfeeUVGjdunDq9ncuXECvg3rMzBXmJoaGhgbZs2UIbNmxQtbImeUAoAZ1AeA6DHARLRSyE4rYjBeRzjBGEe2NS4j6IhTogyA9ykWHx\/uWXX1ahv\/neihUrkqJX6WG\/U5m7kG3NJmMNFx7kUSI\/u16I5Se6NrLdaCycam5ubk5oKxYDIk2bNo1Wr16tBj7mIqyZUhELQXE4+yXLgdYC4ZhAbLpBC0HrcCwQvfpMANZw0Ix8oU3PPfccTZo0SWkaJ43F4Q\/sYvBD80Gj6fID7iIjrxNiGYHRvRAvxNIzV+omFxPCLbHsiIIB7IVYPOBRD2tCikyIlUozCbHcjyUpqSHgllic4AFzLcxbeP6lZ67UTcGZM2cmHAacKhYmIkjAJl9FRUWiTHV1tS2xrKYgR7bCs4gDiVxmVjOQn2HCOWksaEVcds4bMQWFLhkh4JZYMM\/0c2v6Vi875wVMJ3ZS6E4N\/T4qrDsv7DQWOz7YfKyrq0sQQHeIWBuva5p0xEI4OZiyiCmpm7gwbdFm3aTMCOCIPCSmYEQ6ItNqpBvsmcq0ey6IdShxt5vsMZGVFQJBEIvjlFRVValwcakiWWUzP7LO07ICJQIPi8aKQCdIFeKHgBArfn0qLYoAAkKsCHSCVCF+CAix4ten0qIIICDEikAnSBXih4AQK359Ki2KAAJCrAh0glQhfggIseLXp9KiCCAgxIpAJ0gV4oeAECt+fSotigAC\/wdSCNtfMIhH9AAAAABJRU5ErkJggg==","height":129,"width":214}}
%---
%[output:7a92d0b3]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANYAAACBCAYAAAC8aERhAAAAAXNSR0IArs4c6QAAE+5JREFUeF7tXVuIVtUX35OWSlcmg5KRnGCEohqppJouUL7Ygw\/RRZMIBhGpUNPMpkQQAjXLgnyYSgfLIqfopYbQIBN6yCAQfSu1ErxFaUQEgUXz57f772\/2d2afc9bZlzP7nLMODDPzfWvvvdZvrd++nX3pGB0dHRWWz19\/\/SUGBgbEyMhIbg4LFiwQmzdvFidPnhT9\/f3i9OnTYtmyZeL555+XaT\/88EPxwgsvyL8\/+OADcccdd7R9tmnTJrFw4ULx22+\/iSVLlojDhw+3ZGfPnt36TM\/z5ZdfFm+99ZZQZU+bNm2cnkePHm3po8rNMuabb74RixcvFjNmzBA7d+4UPT09UjyZDz6DnG6PSovPlE6ffvqptDuZn9K9t7dXDA0NiXPnzrX0VFjo+Ot2J\/VXeWXZpZefhpvykdIJeCr\/m3yp5Do7O9v8pvRP0ycNY5O8SVfqZ7rPsjBN2g178p6OsoilB61OoqSCOuhJEpmMMZEwKZdFmFDE0slu0lsRC+TQK4osPNIIkiRlMg8KjpQKyYVYpko4zS8hiJWMOWUvBdMoiZXmdD2gVSCYgE46BLUgWrk1a9bIVk9Pk8wzL+BMLQ1aSh8tFvJJ6oNK49prrzW2eCYHm\/DQWz3oqbcKebWoqUIzYRSixYJuSd3TWq4QxEpWLnovJg\/T0omV50j+nhFoKgJOXcGmgsZ2MwJ5CDCx8hDi7xkBCwSYWBagcRJGIA8BJlYeQvw9I2CBABPLAjROwgjkIdAilj4VS5mm1jPGtPLGjRvF1q1bBeXlWZ5S\/D0jUHUEJLFAquHhYfmGH8RI\/p9lpHo\/ABmVPkv+uuuuqzpmjdZ\/8uTJ4rLLLhMXXnihxAH\/46fqz\/79+72a0HHu3LlRvPlftGiRXDKER72U7evra31mKlV\/sUZ9UQli\/fjjj16NaGJmP\/30k+ju7vZq+s8\/\/yzOnDkjdu3aJfA3fkI9V199dVvWyf\/xJXpOU6ZMEZdeeqms8K+88kpx+eWXi4svvlh+PmnSJC\/q3XvvveLIkSNe8lKZdBw5cmR07dq1YsuWLa11b\/gSrdbXX38t1\/eZ1tgpUuHtOR69xctrsUISyyXgqGkpcmky1M9NcvpnFB3S\/ADC\/PHHH2JwcFAcOnTIKqAUEfAbBMCayVtvvbXVkp04cULMnTs3N2+KHVTMVGFJ+TwsQ1T2HQcOHBjFEpZkN65Id7CIbAgjdO9RHJXmbWpailzoYKDooOwEkeBjKokUaa644grx5JNPiptvvlm2XqZWpQ5YhojJ2hErt4qsiUAWsahEAlHw88ADD0jyFCFOTWCUZjCx6uRNR1uSxPr+++\/Fm2++mdoqKdKsX79ejleaSiIT7LUh1r59+xzDKj059nt1dXVZ5U9NS5FLk6F+bpLTP8Pf58+fl2Q6e\/bsOHunT58uP8P4Wf1dFBSKnWl5UtNS5KiYKV2S8nlYzps3z\/uEmvXkhQ5oTGOsosFTNfmsbh5aoQcffFBglotbJLpng7RYLtPtSnUmFt2JtpKvv\/66wG7j5AMCrVy5Uu645scOgSDEUi+It23b1tpqXoQoMKWIfAgj7OCMP1Va6wQy3XTTTeLFF1+M34gKaBgiJslLmjBdi0edURFrV7DINHTS59S0FDmX6XYQavny5ePGThgroQIEsSg6uMa0SxnUtBQ5FyyBwYS8x3I588LGcSFqB10PiqPS9KampcjZBMPUqVNbB9AoHUEivGhdvXp1W4BQdLDxT12w1FelMLFcI6Gi6U+dOiUef\/zxNu1BKKyGmTlzptGqMohVUTgLqx2isi9920gIIwojGUkCdPkee+yxcYRCd3vOnDmZWjKx\/DkxREw6ESt5ug3lXL4QRlSx+wJC6Ytc9fETZVxQBrFcyqCmpcjZdKsr2xUEqXAEmTq0Mvl\/Wn0Smlj+6jH\/OYFIq1ataiMUunyYSi\/63okSkP4tqGeOIWLSqsVS20qwqlmfJcyaOVQuCWFEFdwNQumLYG0JpWxlYvnzeoiYtCKW2twIUukvJtFqmVbK6xCEMMIfxP5zAh579+5tZQxCoaXHFguXh4nlgl572hAxaUUsbMU37eGidAdDGBHjGAtkUi240m\/+\/Pni0UcfNW5QTBIlb4q4DGK5lEFNS5FrzBgrZmK51GMUJ6dNLKhy02b6du\/e7aKaTKvrR9XVpdAyyqDoF4JYoXtRtWuxKI4KIeNzYoKiXyxBT9E1dpkQvagJIVbdto3gRe53333Xih9MnT\/xxBPixhtvHBdT1C0QeVsdKNstXAPapQxqWoocFTNlbxTbRmyWNMU8eeFSk1PTKjksPsZ+KPVgYgK3WNx\/\/\/3yQB7TYS\/Ubg2PscaqBSpmKkXR8Wo0LVbTp9uPHTsmli5d2tYggFQ+xlHUVoZaCVDza7JcNMSCE9SqC7XagjIjiHQhjCgrKNImJmxe8LrqzMRyRXAsfYiYtBpjKZWasqTJhlChuy9lEMulDGpailxoLKMjlk2dEcIIXQ+Ko9L0TqZNIxS2cOSdmRc6GFzspPrNpQxqWopcaCxDxKRTi0V1UOh3BjZ6ZKX5999\/BQ4Y0R\/XJUi+daQEpO8y65ofEyuwZ3GmBMZLMRMqbeYrMDS1zj4oscq6bSSEEa5dweQCWeSX1UJRWovQ3ReKDq5scCmDmpYiFxrLEDE5IbeNxHB2O14iPvfcc+MO\/qd0+WIIBooOTKz\/EMh7JxiEWC7Hn1XttpGsM\/mwYxer9Yvui3INXtv0ZRDLVreqpQtCrLrdNmJyqqmrp7p7WedKxBwgTCx\/3glCrDreNvL333+nXlGDFgl7oXB4S7J1ogYrRS70uICig2vouZRBTUuRC40lE8sQKejeYXr8lVdeybwQACTK6+pRnJzWZ0+qFjoYqLq6kMulDGpailxoLGtDLJfV7bgA4JNPPhG4XcN0GYAKJKwwv\/POO8U999xjfSmAS1CGTktZFR5ah7rkH+RShBi7gur0oo8\/\/lj88MMPpAvT1F1Pea1SXYKBUtPXxdbQdgRpsWwnL3Rjbc5ux1WaH330kUDNa3PfLYiE8zZwb7I+VnIJOGpailzo7gtFB9eAdCmDmpYiFxrLIMRymW5XjitCrPvuu6+Qv\/\/55x+BH9yZi0kJ\/M0PI+AbAd\/vVlsviMu6bcQ3IJwfIxAjAqXfNhIjCKwTI+AbgdJXt\/s2gPNjBGJEgIkVo1dYp8ojwMSqvAvZgBgRYGLF6BXWqfIIMLEq70I2IEYEmFgxeoV1qjwC0RELh4E+++yz8kb4np6eygM8EQaoA1UPHz4si9+0aZNcocJPcQTUGZojIyMyMeVyRchFRSxcttDf3y8NUBfaFYeCU+CWk1mzZkkyKUxfffXVtiuXGCUaAlhVdPz4cbkzgnJNlcrVO7HgyI0bN4qtW7eKzs7ONu2zziFELbt9+3aBq242bNggL7Zueotli6UOuqpxcTWrfpcZLazqI+UDS8QvTjvevHmzmDZtWiY4XomluiAocWhoqI1Y1KtV064Iqo+LaZb4wBIlZQUUTZPqS7liqXcHS+8KZp1\/UeSsdybW2PHdCOne3t62SqoIljxe9YclfJF2GYip6vHSYilSYZCMZ3h4uC0YitxO0nRi+cKSW6oxUvmIS8S1qtT6+vpyJ4O8EEtnrGkLSZEbIJtOLB9YIo93331XrFu3LncsUP2OHs0C27g8ePCgLEBNBJmuCA7WYvkIBjVRwcQaQ9MmGAYHB8WOHTuEmh5WuVHHBrQwrZ6UDZaYme7q6hIDAwMtPKk4RtdiVc9l4TS2DYamz6aaPFI2lkyscLxwzrnsYHBWOOIMysayFGIVmbyI2Delq2YKBsbSzg1lY1kKsYpMEdvBVs9UpmBgLO18XTaWpRALUNherWoHYz1SpR3Sw1gW92\/ZWJZGLJ1cPFNFC4ys069srqmllVpPqbKx9E6sPLfkHX+mjjv75ZdfxOjoaF52\/D0j4AWBIMefedGMmMns2bPF22+\/LY+JPnToECmVOuU2edsiKXFNhSgHXdbUdO9mBTmwc7TkZiHNCJyGC6J9\/vnnJMKBbC+99JK45JJL+CRc76H2X4Yu5KWmpchV8iTcWIiVjA11fjvObscZ7nmtG4iG7Sp4uru7rUKN4mRqwIUOBqquVkD8P5FLGdS0FLnQWNa6xaIEgCIbNvJRiKa6jlW5pZGCgZKhBGSR\/Jos23himZwPsn3xxRdyNX3aA2I99dRTcuNkXUjGxPJXFTCxCFgi4N54443MFg03Oq5Zs4ZvdMzB04W81LQUuUp2BVesWDFq2radPJBk2bJlct+\/\/ti8SwlRO6TFB1qz06dPy8NpslozdBmr1pJRApJQD7GIECJETHZ0d3ePJpfCK1ItWrRI7kNJ\/g9vULfaJz0XwghKdFDGZzNmzJBndVSBZEwsitdpMiFi0kgs01tq\/YQaHKSBPSoIRL0Vw6QCnmTLppsXwggafGNSFJI988wz4vbbb4+WZEysol5Plw8Rkx133XXXaPJoLBNB9FXVeMm7ZMkSSSD95B\/K8VAhjNAhswk4EC1tphGtF7Z2T506tUUyShmhxwUUHVxDz6UMalqKXGgsQ8TkOGKl7evXu4O33HKLMG1RTnYPTY4NYYQrsVT6b7\/9VsycOVOsWrVKXt+afEAyjMeAUd67stDBQAlIJtZ\/CJiw0j8LEZO1I5ZrMKn0INaff\/4pli5daszyoYceEg8\/\/PCEdRXLIJYvLGPPh4k1QR4CybDU6p133kltxcqe8GBi+QuG2hBr3759\/lBJ5HTy5El5AIjNk5f27NmzMlssnfr999\/HFTF9+nTZRcbvtLyon5vk9M\/ydLWxP5nGpQxqWoocFTOlf1I+D8t58+YJ76vb6zZ54SOgKHmgFcPWlpUrVxpbMRxDPGXKlGBdRW6xKF6iyZTSYkGVuk+30+CmS+XNKoZ4Ac3EovsnT7I0YqkZQEylY0o97QXx4sWLW9eaUGYEYWAII\/KAK+t7EOzMmTNi9erVxlYMVxNdddVVXloxJpY\/r4aISeN7LKhc1SVNLgFHTUuRw9T9l19+Kfbu3Uua8EjmmTdFTNHBNfRcyqCmpciFfnURhFix7sdyDQqb9BQnI1+qHGTV+zCsxzQ9Tz\/9tLj77rtJrZhebhEdbLAoaqdtGZR0IYill8vEonghYhmQbNeuXWLPnj3GVgzrFC+44ILCJIvY5EqoxsQiuMmlJqempcjl1bIgWdYKD4zT5s6d22ZxlVosCkbUVjEPy2RYFO1WM7FqRCxlCgj266+\/ihUrVhitu+aaa8Rrr70mWzEmVjtEVMLljVeZWARiVVkEJMP1O6YJD9h12223yb1lSZJV2eYYdGdixeCFknTIejcGFbC6Y9u2baTxWEkqV7YYJhbBddS+vSkralqKHLWbovRIGxdQ9o6pVffIy+eaRYqdaS6hpqXI+cJS11XPk4nVQGLpJqttLVmnVIFYWJF\/ww03OJOMEvRMLDMCpR8xHaJ2IPCtdiIUkqkWDNP42O3NjxmBEDHJxKpotCVbE3QZjx07JtavX59pEVo07BifNGmSc4tWUejGqc3EIniyKd2XLDsp4zIFJYi2fPlyuYYzOUZrCpZMLCZW60iAIkFfhGiKXOvWrZNHECRfUhNcIEWo+lHkePKCgHqI2oFQbO1EKAGZZTTI9v7774vPPvuMhA1atjlz5mSewEXKKEKhEDHpNMaK\/cDOCH3oTSVXYiUVAdE6OzslcfLOxdfTqiuWMBN5\/vz51rjN57S\/N9BSMoqKWLEe2OkScNS0FLnQ3ReKDq4BqcpQaxpNJ1dllaHIhdX7CxYsEBdddJHVsd6hsYyGWC4XTIcwQneuS8BR01LkQgcDRQdfxDLlgw2d7733ntzYWaSFS+aFFSSTJ0+WhMMBqY888oicsaT4lIqxSS7KF8T64Z2xHdjpGkxVSV8GsWyxwHn5OLb81KlTTqQzla9aQdUFxbjv+uuvl62heop2Q0NU9lZjrKNHj0Z7YKdtMFQtXczEomD51VdfiQMHDsiNoOqHks6HjE48\/I1hDXTx+dSOWC4BR01LkaN2U5Qzk\/J53ReKDq6B4lIGNS1FDkccoLt54sQJaVIIIu7fv98Vrrb0E0IsrxZwZoxABgIdHR2ym4iLPDB2w3hOPepv\/K48sTgKGIEmIGDVYrlMXjQBVLaREbAilst0O0POCDQBAStiARi16kLdBkk9sLMJoLKNjIA1sXRyKRiTV64yvIxAUxFwIlZTQWO7GYE8BJhYeQjx94yABQJMLAvQOAkjkIdAdMTCVD7OzsPNHD09PXn68\/cGBJIXWuBy8oULFzJWFgioGfCRkRGZmjqPEBWxsAaxv79fGrBz504mlkUgIAlOcZo1a5Ykk8IU51zoC6Yts25cMtwVd\/z4cblPDTPfwHZoaEjuXct6vBMLjsRVojgZKFl41sZI1LLbt28X8+fPFxs2bBBbtmxpPLFssdQdrmpc3HbSZGL5wBLxu3v3boHbOrFEqjRiqS4ICkyymroxMm3lfNOqSh9YArOsgGoKpq5Y6t3B0ruCemvU29vbRqwiKzWYWGMv3xH4LljyeNUflvBF2lI+UwXlpSuoSIVBMp7h4eE2YhVZW9h0YvnCkluqMVL5iEvEtWog+vr6cieDvBBLZ6zpYvAiGyObTiwfWCIP3FqCI8zyxgJN6Q7axuXBgwclRGoiaO3ataTxPxMr4siyCYbBwUGxY8cOoaaHlXnUsUHEcDipZoMlZqa7urrEwMBAC08qjtERywm9miW2DQZ+\/zc+EMrGkokVMRnLDoaIoXBWrWwsSyFWkckLZwRrlIEpGBhLOweXjeX\/AIX129eDqKFgAAAAAElFTkSuQmCC","height":129,"width":214}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANYAAACBCAYAAAC8aERhAAAAAXNSR0IArs4c6QAAF5pJREFUeF7tXX2QVcWVP0Q3AclscEZIHAk7mPCR4EaBigFkV4xrMFUBDYnhIxUJzBKKiFNbyxTfFlIVR2CZuAHclShSk1Q5WGTZOPyRJUp2iGZky0CgdoUVDQwEJokwo6KItZvsbJ07npcz\/brv7e53+857886tsmRmuvue8+vz63P63P4Y0N3d3d3V1QW1tbWAz44dO6CysjL6tzyCgCDgh8AAJBZWvXz5MqxcuRL27t0btVRdXQ07d+6EUaNG+bUstQSBMkYgRywVg6effhpWrVqV+\/XDDz8Ms2fPLmOoRHVBwB4BI7F4Exgqrl+\/HtatW1d0YeLBgwdh3rx58NRTT8GkSZPsNbcoqbaNXn3Lli2waNGi1HCgSAHF2bBhAwwaNAjwvadPnw4+kOHg2dbWlntvHCTbtm2D6dOnW0UwLmUtuiEqwiMql2hKh6\/tOwsppyUWzbmOHj0qYSFDd+PGjZHRpzkPVTv+7NmzsGDBArj\/\/vuDEov6eM6cOYnvQQJu3brVamrgUtbFcF999dUIlxkzZsCKFSusq\/YpsZKIREp1dHRECpF3IKEvXrwY\/f7AgQN5czMa9fHvN954Y84o6fcPPPBA9DtsOy7cNMnAvQq2j\/NEkgfr4Dxx+PDhveaPixcvjjqH2iQjVj0U\/xk9CIXG6oip6zyVhEkYIj7Lly+HpUuXAg1oJCcPy9V343u2b98e4T5t2jRobW3NEUCdN3N8VQKoNkBlef9R348ePTpKdqly6spiFMHlR2KQZ1bZYZJB\/b2uDVVXwk5no9wOUQbC0GSj2BZiTG3GYU6yRh4LX47\/6bKBqvHxDiGDPXToUNSZVVVVEeAjRoyIwOOj78yZMyPjppAHOwVDOC6syRuooys32hMnTuRCQSIWyUOJFyyPxEWZ6L04OKC83DvEEQsNJM5jIS67du2KBgl8EAesoyOwDkPCRfVYiH9DQwM0Njbm2iV8SRckAeHLdTfhRLoQJhh+8rLPPvtsLw+lkhDL1tTURJ6OSEODrVqWY0qEJFzU6QbHTO2LJI9l6mPVJvCdap83Nzf3slfyiiQD2SjWpd\/pMCc+YLmIWHFzKA4UGuLq1avh3LlzMHXqVKirq+tFFuwgXv7w4cN5HUTGR4SgkTEuhEBl6uvrtaGIzmPx+UpcyOPisZKIRW1t3rw5shc+7zN5B2xTxdAUCupGfZzvYRaXsrdqX3FD5TiogxwRC0dldTRHXXR9o3oRE7HUwYgTAN9Lj24+ywdiwkUXCsb1MXmsM2fOaAc9ej\/3WkQk3fwdy5kw56TNEYu7dj6SvJ+Nj341YMAAGDduXPRvG2Lt2bMncqH8oVCms7Mz1vh4nSTSkRHT6MSJpZKHt5smsagDUT8a2WguphoXGYINsahz0aAweYRJJNIPicW9PMeJOpnCd9IbjQaTLzyyQANXQ1VOMJ2HReNCMsUNImoITjLYkNc099QRK66P1XbwZz7g0IDFcSGPaiKWCXOy6ZzH6mX5yg8uo22Sx+JNq0LbkkfN\/Nl6LF34kSax+Mg+dOjQXBioG\/FdiMVJifjyUdzFY3Hs4yb0fK7C53iUvOCjMoaCSf1om\/BJw2Pp+jiOWGq0oJKuYI8VRyybORaNXhRDu8yxTCEEl4kbIs5ZOAm559N5LGyHdy6Wpxj7jjvu6DV6UThAMqnAJhkJH\/V5+t8GQ\/JCaiio05Um73yORbqcP38+FxomzbFo5FUJGycDzaO5t6X+p0QFzyBmOccifXgfq2GvSh51bokJqrhQkM+xVMzz5lhxxMK\/cYPhWSk+GlRUVEShgermk7KCNsRSZcCfdcZvIpYpY0TehLJ9lP0xEYvrovtupsbzPBlkgyEmV\/ChFTBIIJ4pROzRG+LDw8wQWUGeeeOy45wYH8IMMUcyU2ZVLcsTHFjPJSuoG5xM6fakrCANXCqx1H5BfNXkkNrX1lnBJGKZ\/t5X3wh85e1v9Vw9qi4KwAgg7Q\/r\/Q3nuCmMKYqxWnkhxCpOU1FHae7JbSR2WXlh0145lLHFvCBilQOQoqMg4INAHrE4IynGX7t2bfT9Sla6+0AsdcoRgV7E4t9iZs2aBU1NTbBmzRpoaWmxXqxZjiCKzoKAikAvYvEVGJiyJGIh4bJY3X799ddLDwkCmSLwPyNugXcnLISu796W6nvzQkFacrJw4ULYvXs3LFmyJFoYqvv4lqokAIDEOnnyZNrNlkV7gp1fNze\/9Du4r\/l4eGKheOoq5aw2OYpx+BkH1jp16hSMHDnSv4EyrZkpsUJjjF4RH3VfjRDLH3khlh92\/YZYuqX4BIkQy884xGP547ZxXzts3HcqbCiobgVQxXXZEq1TlZIjmLa\/dOmSeCx\/e8irKR7LD8zMPBZ+jW9vb+9l9PS7W2+9FWhTGN9LY6sShoDYBu7GVd+BbYjHskUyv5wQyw+7zDyWLq1OngaX2eNWBZ9DZTAExK37tDrdRKz9+\/f7IVTmtXBFOq6ulscNgc+t2wd\/uGZM2FCQPhDz7d20+nfixIlw1113wTPPPGN1qo+qHt+hSX\/jW57FY7kZhFpaPJYffjMf\/RW88Os3wxKLRDMdILJs2bJUljbpwk0hlp9hUC0hljt+Z7reg5u+82JUMfgHYnfx3GsIsdwxS6ohxEpCKP\/vq3\/8Gjz289\/0H2KZIJDkhbtxiMfyw4x7qw+8ewEuPHaPX0OGWnlLmvhpQLyO7gCQVCWRrGBBcIrHsoev9UQXzHqs5zDaEZUD4eKTX099KV3eIlyaR+EJS5gap2O\/6Bw5e\/HdS4rHcsdMPJY9ZuilZv7TrwD\/T6TaNudTcO8XJoQnFqXb8dBGSolndXa7EMveSNSS4rHM2P3slS746vYeD0UPeiok1dRPDgny\/VS7H2vKlCkwYcKE6CCTTZs2AR68SQdthrw7S4glxPJH4E810SM1HeyAR547ndccEqrpmzfAjcMrcn8LYXd5cyzundBr0Wk8IW7zULUOoWAaHVUKbZSzx0Ii\/fRYJ7QcfT36JqV7kFBH1k6OwkD8N39C2F1RnXkRQsFSIEUaMpYLsZAY\/3HqLfjhwQ4jiXi498g9Y+C2MfE3lIawO+MOYh7yyRwrDdMP20Z\/IxYS6DdvvAcb\/+0UnHnjvVzCIQ5F9ESfH1sJf\/f5v8jzSnH1ghEraVU7ChV30GJaJhNCwbRkK\/Z2SpFYSB68p\/cfnzsNvz7\/bqIHUpMPI64eCP\/w1TEw6M8+4ESkLKYgVh4rK6MSYvkjXazEQvI8d7wTfnzkdWvPoxIIf751dCV872tj\/AGKqRnC7mSOFaSrsm+0L4iFpPm\/7m745wNn4fhv3\/EiDp8PoQeaP7kaPlvzkYI8kCv6wYhlEwrKygvX7sq2fJrEog+o1UM+BA0\/OQW\/bH+rINIgEpSJQ\/Ks+uJIuG7IwEzJ0ydzrGxNwPy2ECNHsegWWg4XYiFxTrx+CZ45ch5Od14umDScOLd8YggsvOU6GPrhD0Yqq6nt0Dj4tB\/C7iQU9OmJIqyDxLriI9fCyx3vwE9evgDtF9IhjOptvvSZoXDnuGtKgjC23ZQJsXSHvmeREUQQQihoC24xlqOPmacuXI5WEhw+3XOJum362UYn9CgYnn28ciB8ZfxH4ZPDrioZT2Ojn02ZEHZnPGKaH01mujfWRmiXMiEUdHl\/FmWJLP\/7x254tPUM\/Oy\/u1Ini+plvjLxo3Db6J6PpKUQmmXRD\/wdIezOKt2e9IGY7zg2eTfVE+qSISEUzKKTaLI\/+ENXQOOz7fBf594JTpYRVYPgyzcNg1Hve5g\/vvVbObDTs7ND2J32iGl+KzideWG6SQ9JR1tN6KpIXMSLN\/nxh5cz3VoSQkFPrHNf+ts7L8PuQ79PbZKvysOzZfi3u8cPg78ZW5UrZuthXJIXvpj013oh7E6bvFA3O9oeMU1eae7cuXm3BCJBGxoaoLGxEUwr5EMoqBoDeZemFzvgpfa3oj+bFm76GhInC85dJl8\/BP561NXOZHF5vxDLBa3eZUPYXWpZQQoHTaGgDVnTVBAJ9KPDv4fWV7pSIQ4ny5iPDYbbx1ZG2TG+ac6\/awuvKcTyxzBNuyMpUiMWNWhz\/SZ9kFbvvy1UwbaTb8K3nzputWBTneAjWb45uRoqBl5ZkhN8IVYRE4sMnm4N9zntFj0XZhHpVneduhQyqnMxJJbPgZ3f2vM7OHSuZ7u1+lT\/+ZVwbcWVsHTK1XDNVVcA\/twfHzmw069Xb7\/99qhi2tdHxV6VSqKqB2tyFdSMoekmEfRk+GBSA+dbtDuZJzJcPBaGYIfOXITaH7zcC1EM2RruHgU3VH+4JD2Pn3nINT6+uGE9F7uzfY9VKJjkhUzpdk4mNd2uS4jYKqgeCkJhHe4QLddHQkH\/nre1O5c3WHmsQm8ZsRXIRkHdSTst3x5fVt5Jh6cQy9bK8svZ2J1r63kfiGtra6M24uZIri+xLZ+kIJJqafPxXJYPwz4hVQ+6QixbK+sDYvmLlk7NJGLhAtO\/2vxS9DI6HCSdN5d+K0Is\/z5Msjuflq3mWD4N+9SJU5AfCSyeKh9dIZaPxfXUKWti0XUrCAQmKWyX+vjDXVo1hVj+\/VW2xOLeauonhkDLfeP9UeynNYVY\/h0bnFimVexJq9v9Vepd06SgeKtkhIVYyRiZSgQjls2ZF1lsdtQpyL3V9E9XQfPffsYfwX5cU4jl37nBiEUiZeWZXEaOZT96BXa2dURVZG5lNh4hVhETy1+0dGrqRo7Kv\/\/3qHFJr8djLMTyt8FgHos8VV1dHdTX18PRo72vPEGR++L4Mx4G4odgvHJFHj0CQix\/ywhGLH+R0q2pKjj3if+EfccuSBhoAbMQywIkQ5GyI5aEgfbGIsSyx0otGZxYcdnBrENBHgY+ee84uPumYf7IlUFNIZZ\/Jwcnlkk03GNF9xH7i59ckyv4wmtvRvfF4iPzq2TshFjJGJlK9BmxskrDcwXpo7BkA+0MRohlh5OuVJ8RK2mjo41K6kZH3dWrXEGaX8kSJht0ZduIHUr6UsGJFTfHKvQOYtxN3N7eDniAjImopCCfX33va2PhG5OuLQS3sqgrHsu\/m4MTy180t5pIrObmZtiwYQPwA2tIQZlfueGJpYVY7phRjUyIpR70gp5m165dqewo5uFgXCi4fM8JeOKFc5HeXd+9zR+xMqopxPLv7ODEMp1ka3NWoItaSecKSuLCBc2eskIsd8wy81hZbRuJO1fwh\/+6D2Y0nY10nnjdQPj+rI\/5I1ZGNeVcQb\/OzuxcQfROW7duhZ07dwKe+UfeZdKkSVHiwfexPVew9ZfH4KbvvBi9ZtucsTDvZklc2GAuHssGJX2Z4KEgvZZuGOno6NmuYXspQpxqtun2H\/z0sHwY9rARIZYHaO9XyYxY\/iIWVhMVfOjpNriv+XjUkOy\/ssdTiGWPlVoyOLGyWmFhggAV\/Mv6PfD8a29ERSQjaG8sQix7rDInFr4Q1wXW1NTkXRznL7Z9TSTWDcv+JTqQU5Yy2eOGJYVYbnjx0pl4LDwJty83Or55945IZ1nK5GYoQiw3vDIllr9o6dSsueFmuPiFjVFjT3xjHMwaL1tFbJEVYtkilV8uuMfyFy2dmiNu\/iK8M3V51Nijcz8Fcz8r37BskRVi2SKVIbGK5cwLTizZg+VmKEIsN7zKKhT8+JQvw6VJdZHOkmp3MxQhlhtemRMr5CLcJNWH3fsY\/OGaMVExSbUnodX770IsN7wyJVZWi3BNEBCxJNXubiRCLHfMqEbw5EVWi3BNEMiuYX\/jEGL5YxecWChaqEW4NmoTseZProZH7ukJCeWxQ0CIZYeTrlQmxMIXh1iEa6M2EWvF9JGwYnqNTRUp8z4CQix\/U8iMWP4iFlaTiCXfsNxxFGK5Y5bZHMtftHRqErEk1e6OpxDLHbOyI5Z8HHY3EiGWO2ZlRyzxWO5GIsRyx6zoiYWZxFWrVkVyms54V3cQ68phKCjfsPwMRIjlhxvWyiR5gWf+zZs3L09KE2Ewg9jQ0ACNjY1QWVkZ7efCLf3qmYH4jWzZsmWwevXq6CwN3SPE8jcOIZY\/dsGJZTqWzEVk02GcKgFNxJJ9WC5o\/6msEMsPt0w8Vhpb8007kHm4iMroDqhBjyXE8jMQIZYfbpkQC1\/Cz1h3FdW2rskzIrEW3zwEvvU5uRLVFXs5V9AVsZ7ymZwrWMjFcy5nZZgO7ERiSardz0DEY\/nhlpnH8hHP5mI6mwM7kVhXHX4SPnjmFz5iSB1BwBuBkydPetfVVRzQ3d3dXUiLuizijBkzoqxgS0tL1PTs2bNBTbencQhoIXJLXUEgJAJ5xOIEQIIsX74c1q5dG5smDymgtC0IlCICvYhFpKquroZZs2ZBU1MTrFmzJvI8bW1ted+mSlFhkVkQyAKBXsTi6fbOzs4csZBw69evh3Xr1kUfgeURBASBeATyQkFaObFw4ULYvXs3LFmyBJYuXQqF3jYSJwb\/xlXolaz9ucP5fNY0R1X30i1evLigW2L6M56km82qIFcctMkLNSERMtHAV2ScOHFCe4Wqq1L9sTzvfNSPLyPj+ppWvvRHTNLQiQYibIuurkqj3YKzgoUKwW+LxJAzaT1hoe8r1fr8QnS8t3nlypUwd+7cKJLgj+1H+lLFIU25cbB6\/PHH4c4774QHH3wQNm3aZFzH6vreoiBWe3t7FK6ksVbRFYBSKc89EcqMxJoyZUqvyyvUTxqYhEpzFC4VrFzlVI\/8c62vKx+bbqcK9F2K33CfxsuxDT7CCrHMqNoQS63NvZwknczYBicWT7fTtaj0OxRL3QqSBrkkFLRD0TYU5K2FMBg7aUurVAicjOl2PsKlserdBLUkL+yM0CZ5gYPgQw89BPPnz4\/mCnzQChFt2Ele\/KWCEwsh0GWVXBbY+sBI6XaZE8Sjx7O1\/LOEug5zwYIF0WZTwdPOGoMTK251O4lo2klsp4KUEgTKA4E+zwqWB8yiZbkhIMQqtx4XfTNBoM\/T7ZloKS8RBDJGwLi6ndLtKI\/p5KWMZZXXCQIlg0Cfp9uLASnbZUA2J0256sMXzdouQC6m9YCUqQyV1KL2Qy5ScO0zm\/La1e179+7NLYWhjkfFuBezabxUyvQlsdBwDhw44IRtsRGrubk5yOIBsh+0QdobWCrf47TJC5ujykqFNFxO\/h2IRlhcUU8HlNIWC3V1P3qS0aNHQ21tLRw9ejR32i8thsWBCJ84j8PbpNEX26J3m0Zk3he0y4CIVVFREb1T9Ra8Dv+WhSH9sWPH4Pnnn88dP8e\/IU6bNg2wTRpAdTKrhq2SHN8xePBg2L9\/f4SVaduK6v3jBot+Q6xSJE2SzOqeG\/5RlXss3R3MtHsajxhTT\/3F96IhomHU19drF72S19+8eXNEAlxAiwZP9UwjPjc2vvIfN6EiIYlUanto3Dt27MidTEwyqnNlrmtVVVU0cNC+O\/634cOH95JZHay4\/PgOinioTdRTXYUvxEqy2BL5e9xmtrhQkBs3JxaqjYZIRmO6vxnLqaMxX\/cXtwfNtOJFXVwbJz\/\/G7ZHJMP\/q\/X4z+pyKJNH0XmsuHfw8I4PUuKxSoRIOjF5ooCHXqqBoQFu37491wSV1RELwx3+6DaFqkZqsz7SdPaijqhcfnXrCJancEwlahzR1OkAtqNLUOiIVVNTk9vOYiK9eKwSJlKc6OroTHvCVG8Q57FsN2WG8Fg8\/IrzNKrHijN6EyZxOCZ5LJW8Jo8Vt2BY5lhFTEJ1hDTNsXTbM1At3DITN8fi8yjdfAIXxrrOsdQtNRR6ojw2xELvxedNqseynWPhSnnTt0wdsfB3OMdTw2VuHrp5J+GsJkiEWEVMLJpT0D1ePBSk7BeGTHV1ddFEHSfgmGDAa4fwUB28pogMBf+PhqNmBePOBjFl2JJS5zwsVbOCtD9Ot1mUspeLFi2Cffv2RQPDli1bgHssjgmGeXiO+aVLl7RZQdN3Kh2x3n77bWhtbY1W2HNMdHM67A\/EGQeAI0eOaAcwIVaRE0vEMyMQN6dzDQVV8haKuxCrUASlfqYIqN\/rfI5KozbIo+HhLGkSi9ov+ZUXmfasvEwQ6KcI\/D\/nofiz7fl7IQAAAABJRU5ErkJggg==","height":129,"width":214}}
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
