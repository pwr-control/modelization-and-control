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
use_current_controller_from_simulink_module_1 = 1;
use_current_controller_from_ccaller_module_1 = 0;
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
phase_shift_filter_gain = 0.985-0.085/350*(frequency_set-50);
phase_shit_filter_d = phase_shift_filter_gain * (1-a*z_inv^-1)/(1-b*z_inv^-1);
flt_dq = 2/(s/omega_set+1)^2;
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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:24ae7bfd]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:1588507c]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:434ec02a]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:62fb6716]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:1c8d79cd]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:2e5c863a]

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
figure;  %[output:0c2ff197]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:0c2ff197]
xlabel('state of charge [p.u.]'); %[output:0c2ff197]
ylabel('open circuit voltage [V]'); %[output:0c2ff197]
title('open circuit voltage(state of charge)'); %[output:0c2ff197]
grid on %[output:0c2ff197]

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
heatsink_liquid_2kW; %[output:7fb89984] %[output:684c5367] %[output:927c08f3]
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
%[output:24ae7bfd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.149016195397013"],["36.521945502464568"]]}}
%---
%[output:1588507c]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:434ec02a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:62fb6716]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:1c8d79cd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.155508836352695"],["27.166086113998457"]]}}
%---
%[output:2e5c863a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.037191061070411"],["1.937144673860303"]]}}
%---
%[output:0c2ff197]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAMgAAAB4CAYAAAC3kr3rAAAAAXNSR0IArs4c6QAAFtBJREFUeF7tXX2wVdV1XzRMIgqE8mXAx4cQMXSkIlpDqFaJIfxhQcvY8NFJnPeQUhqko6B8CpECAoITwVao4pOm9eHQmgrOVLA2UpQ6rb5gaDUV5eMF3yTiIwxYtMRI57fJuq677z7n7HPvPu\/ed+86\/8B7b5991l57\/fb62Guv3encuXPnPvroI1qwYAHt3LmT8PTv358aGxvpsssuMz\/roxyoVQ50AkDswT\/99NO0cOHC3K8feOABmjx5cq3ySMddwxxwAkTy48SJE3T\/\/ffTsmXLqGfPnmVl1auvvkrTpk2jp556ikaPHh2EFtae6Gz16tXUpUsXeuSRR2j8+PHBNOiaNWsItG\/ZssXwEP8\/evRoZovOwYMH6d5776W1a9fGjgFj37BhA82YMSNxbkPTDBrr6+uptbWVJkyYkON93KS65iqIEMR0UgAQAGL69On0xhtv1KS5Be25cePGoCamBEhbW5sRjDvvvDMzgOB7EDwGfNT828CNasfCHJJmtlLSLHZlAUhbW9s5FyB69eqVBxQ2s5jIU6dOGX7u2bMnz2exAcYM4NX\/vvvuMyspJjDKdLN9opkzZ9L8+fPNyssaBPRB0G688UZ66aWX6OqrrzYCgRVx8+bNhjZemY4dO5YnlLKfK6+80vhfeCZNmmTa8SMnzzU5toDhZ\/vb0Ejc7sEHH6R77rknt\/hgXFi9Jf95rKBBmrr4PfpmmqL47BJmF107duzImdHsczY3Nxf8zpYD0DFnzpw8nzXOBHd9G4sv5pEf1\/s8R2jD9NXV1ZnvStnD\/LFmlloJ73G\/3Bfz0Ob7DTfcYEjp3r27kSE87JN3OnPmjHHSpfnEzIcZYwsmC9Trr79uVllm4MCBA2np0qW0fPly8wF8CJPAqzFWTjCFBSBu9ZIrIDMTgoHHBggDA4IIgdq2bZthmFypR40a5QUQm2Y7SCH7By0QbOaRHE\/c32wNgvcGDx5stIlcVXkBAMgxByxo4IMEtYvP8+bNy2lA16LCmsDWbKtWraL169cbPmNsmFP0by8w8r2333470uyN40mcBmFBX7duXd5YWb5Y9kAna+OJEycaoZ46daqZE3wbQSfIqC17eE\/+nfuRiyybxJ2gQWwfw7b1AaC7776bDhw4QE8++aQRegYBCyZ+B3BgEDwBcqAs3IzqKFOGwTllypQCEyRusnmFx2oDgZKPvapGaZAkgLjGA4EdNmxYHlh49efF4Zlnnsn5IC4Ty6UN4KMgUGJrYPzM4HHxGe9JE1GuxLY541qkpNZyaeBx48bljTVqvuxF1uYJayuXiSUXIrlw8xy3tLSYRdBeiPCz1CKseRggtvblBYA1Bv6F7zZ79uzcwmcAIlU8Gn366afUqVMnQwD\/i2BXjx49aPv27ZEAueuuu4wJYT8AxaBBg\/JWmiiAxNm7cQBxTQjTEQogEoTom1cZl9DL8UUBhFc9rHS2howSdNnOxecjR47kVk5oQNtcxTssKC5NAFAgIINFkxdBqUFYG8NElo80DaWgSr9F8iQOIFHWhW3m4mfW4myqHj9+3GgN9G9bLzZAWPvLfu+44w6aNWuWcQGM\/LvCvC4Nwna6bUbFaRDJQLvPJA3CxEf1Ya+icQ5cKIDIVbBPnz65VSZptYwCSJzpl0aDSB7FBRlYO7Dd\/thjj+VALv+POeX5tk0sW4MUrIa\/+UUST5I0iCtQEgcQ+BEywin5UJIGcQHExwfhVY\/NC18fhFEcN5FRUR+phWyA2CqcfSMAjVcX6S+wwxtnz7s2SqUKl+ZBMT6IFDbp+EozCiu6\/bckHyRKUNjJZc2AgAZrwRdeeKFgxY0KcrSXD2L7Sgi\/I3TN9EsNwgEWvMN8ZW1iAwTvu3wUHq\/kS+Q+SFSURKK4W7duRl3LSEJSFMsHIGmiWHboMSqSFBURsoWNzYmoKJttB0sbWX5bmhxSoHh1xgKDCbnmmmuMSYMHtGBSeUxZRLFkloT0TzZt2kSPPvqoibChDTQkHtj6Ns1sRXDmhW8UyxWhiwrzxkWxXACRgQz8\/frrr6e9e\/cWmK68fyblNDaK5dIgUWoTvy9HLDqOnmr9m20WptmfASDx2MGKauVVMeOK0\/iyv8SddPvjCpBipqO4d6RGQg++KT++O+nFUVUdb9l7JtIKigSING2g\/hHyWrJkCS1atChY2kV1sFdHUSscyGkQGcKEw7N161ZavHix2ezbt29fYtpCrTBMx1lbHMgBRCYlwutngAA4lZKsWFtTo6OtBA7k+SCc4tHQ0GA2BLFhIncVQxM8ZMiQ0F1qfzXMgbMDf5\/OjGqgEw+NDcaFAiddhtfSOIbFUASAHDp0qJhXa+od5ZPfdDf958\/pu01vZQsQP1LCtNKJ9+Pj4cOH6dJLL\/VrXMOtMgWIvcFn87nUY7iI49vOvgLET5oVIH58yhQgIAFCjGQ3ucHEv8NuY1NTU1HRLI45y9R0fE8B4jfxChA\/Pq3ZdYTW7DqcjYkVdbSWf498IOSopD16iyjYypUraejQobR\/\/\/48gClA\/CZeAeLHp0w1CO+D8GEUJOrJlf+WW26hZ599NrUGgQbCg0RDWwMpQPwmXgHix6cFPzxIf7P3WDYahEmwo1h8IGju3Lmpd9QBMN5PQRKcCyAvvvii3+hruBUSKJGJq088B766bBd90vvybAESchLs8kHoGyksDz\/8sPmMahA\/bqsG8eNTz7t\/ZBpmug\/iR0r6VtBMamKl5xveUID48S1zgLhWfJAWlenoR\/b5VgqQNNzKb6sAieddy4mPaeSKfzeNfuvMB\/TBpj8untnWm3m5WOxn4IgowrpcHYKrbgT76m86UhPLj6MKkGg+SXCgVdeX11LLf\/yzH2M9WjmTFXH8kvdDsqysqADxmCE1sZxMAjBmN71FL797Mvf33X9xNX1r7Mig6UsF6e5jxowhFBPg0pU4XM+1pkKXHlWAKED8OPBZqwPvfUjfbjxAAAg\/A3teQDv+\/CrCv6FlKi9ZUWoLaBEuYJ2mPGSaAYceTJpvd6S2tWxiAQgb\/rWFntj3nnPKpl3bjx6Z8pXc30LLVOojtyEFK\/RgQtJWSX3VEkBcppM9F9AUT00fQV2\/0NloDfmElimnD2KXIc3qwFTowVSSUIekpRoBAiD84vRZ+svn3s3zI6L4BiD8XcMIuqJ\/11jWhpYpZ2VFmwLf8vRphSL0YNJ+v6O076gAYT\/huQPH6fn\/+sALCJgT1gpLbx5K1wzqXqAl4uYttEwlapAshSj0YLKktZx9VzpAAITvv3iU3nn\/jDcIpION\/zeMuYRuHdk3FRhccxJaptQHKafke367nACB8P\/q1+do07\/9jP7n5\/9LLb\/8OC+C5DkEI\/gDf\/sC+trQHvQn1\/Yzr9n+g29f7apBXMWrbQJC7KS3B9pDMLgS+8gKIBD+i7t\/nh54\/jA1Hz1VtPBLbQAQDLv4Iprz9YGZgaBdAZK2smJIAQqtDkPSVkl9pQUI2\/7\/3fohwf7\/2YmPSxZ+XvEBgKF9L6QZ19VR1y98LhMtUArvQ8tUnonlKpWflYMOJoQeTCmMreR3AZDPfbGfEUYI\/7\/8tI3+af\/7ROcoiOBLcwcAmHhlX\/rm7\/QqiwYodR5Cy5SzcJw8cut7310xAws9mGJoKPc7vNqfOftr2vLKeyXZ+VFjYft\/SJ8L6eYRvWnc8F4GaFn4AOXmZ2iZSoxiaS5W+iln4Xu95RQ9ua+VjrZ9ZDqReUPpe81\/g4UbK\/6AnheYFf+qAelCoqXSUInvZwYQDFbemSCP3PI9eaEZEnowoenj\/ljgj7Z9TLvf+oB2\/uR4UPNG0i0F\/yv94OwOoqNHW2jAgAFVueKHnrPQMlUQ5rXPhMRVFJfHc6N8FVmh3C4dFHowaZkNwX\/\/9Fn6h+Zf0JutH5rXiw1jxn1bCv3g3l1oyu99iep6XOAt8Gmd9LR8qKb2oWWq6H0QmF58foRvLkImMG5r5Yedfr551J6I0IORK\/7xD8\/SE6+8FyyCY9Nur\/T1Yy6h4V+6KBPbXgHiD+HQMlU0QCTJUUCQIHJdZxZiMD7JbT7stW36bwzvRX80sm9uU6ycDq0CxGcGz7cJIVPya3lOurwbG9du+TxsZrlMLLtCin0TKgZTTFWTd9rO0l3PvU+tpz5JJLF\/987Ur1tnqvtiZ5owvCtd3LWzeQe\/7yiPVjXxm6mbbrrJNAxZ7zlxH8QW6ihSXaVFXVpGmmFp0A5N8cszv6KxD71WQAKHMWf+wQAacUlXb9vej+3lb6UaxH8O0siUT6+JJha0ABxtXOYYd6LQp519d16aweBQvn2KrGn679Lwfhf5jLNDt1GA+E9fGpny6TVRg0QVrbb3R1wXRwI0e\/bsMbV+uTg2\/s83jfoMxj6UL49X+gywGtooQPxn0Uem\/HsjKvBB8HKStuAPuMK89o1UUVcj+zhUNjhW3vJlmnXDgDTjq4q2ChD\/acwMIP4khGsZNxiAY+Jf\/zhnVr22aDQN6e0XOAhHYWX0pADxn4eaAcjq5w\/T2t1HDGfuHDuQ7p8w1J9LVdZSAeI\/oTUDEC4jCZ9j\/5Kv+XOoClsqQPwntSYAMvGvfpxL7AM4yrlJ5z812bVUgPjzNjOAJF2gk\/biHJ8huQYjHfPrhvagHd+9yqerqm6jAPGf3uAA8Tlym9WhKddgpjz+E9r9ZpvhiGqP84KhACkjQPjIbZbnPqKGZwNEag\/1PT7jmgKkjACBBkFhONxBOG\/ePMJNUPbTXkUbXn7npAnt4kGt1eu+3MOfM1XcUgHiP7nBTaxKKtrAzrlqj3yBUIAoQAwHOLSrzrkCxB8S+S0z0yCcK1UuE0vNq2iRUA3iD5fMABJFAnKp+LYpfzL9WsrByL0P9T9Ug\/hJUGGrdgdIltEtORhOZ1f\/o3DSVYP4w6XdAeJzzsOffLe9KMO7KFn5vT+s3bwrFy8VIP4SlhlA4nyQUm6YkunudoUUHowEiG4Oqgbxh0MFmFilECuvfsa5ar73kAs4MEA2\/qiFlu1813xKAaIAKUXmMtMgIOrgwYN5Qoxz5qEu8ETfq1atovXr1+eO7vJgdP8jXiTUxPKHTGYAiSrdk1SMwYd0NrNcJtZLr72ZuwRe9z\/c3FSA+EjZ+TaZASTrbF4GoF3V5Ac\/3EUTth4zg\/veN3qb0jz65HNAy\/74SUTmZX+gLTZu3EiNjY0EP4EddxRZkBXf\/cgtbOWqavK3u5tz+Vfqf6gGKVa2+L3MNAh\/AL5CfX09tba2ml\/F1eZNGoxPVZMV216h2dt+qg56DDPVxEqStM\/+njlA\/Enxa5kU5r1i7j+a04O6QRjNTwWIn6yVxQfxJy19S6C9e8Pf5wo+1\/rZ8ygOKkD8ZStTDYLVfvDgwXkV2v1JS98Sgzl56xbz4s0j+tAP6q9I30kNvKEA8Z\/kzABSjmzewVdcS6e+ucaMfv74S2n++MH+nKihlgoQ\/8nODCD+JIRrKQGiGbzqg4SQrOAAKeeR27obv01nRjUYvihAFCAVCZByHrntd+sS+r8h5+90OPHQ2BD8qco+1MTyn9bgGkQCJMtcLNcQ+35nE33S+3IN8SbMvwKkAgCSZS5W1PB6\/9l2+vTC3qQ5WPECoACpAIBknYvlGqIWafCbeAWIH5\/QKlMTK+tcLHuYDJDvf+ty+s7o\/v5cqLGWChD\/Cc8UICAjZC5W0rAYILoHoiZWkqz4\/j1zgPgSEqIdA0RDvAqQEPKUuYkVikjffhggmuauAPGVmaR2ValBFCAKkCTB9\/171QFE09yTp16d9GQecYuKAgiiXgsXLjS0RVWAl+dB7CulYWIpQJInXwGSzKN2AYi81lmS5BJ+u0oJgIBTiKtXr6YuXc7fRhu1+ch9AyC6SZg8+QqQZB5lDhBOd8fZc5xBT\/vIGlgMEPQ5d+5cWrRokTnjbj8KED8uK0D8+JRpFKvUGryuw1a2Rpo5c2Ze8QcAZOa1PehPv6oX5cSJgFY18QNIu1Q1OXLkSOoKJvBFkt5zlf0BQHQPJHnyVYMk86jdTKy094OkOaZrl\/0BQC5sfoI+3\/KKPwe0pXIggQOHDh0KxqNOpZwHSbo7JKnsT7BRaEfKgYw4kAcQNoN27txJuPoZxaaXLFnidLJdES9+Z+3atcT3qsswr+2DZDQm7VY5EIwDOYAwOLBXMWnSJNq6dSstXryYduzYQfv27csL3wb7unakHKhwDuQAIqNYbW1tOYAAOLgmmjVChY9HyVMOBOVAnonFm30NDQ20fft2mjVrFs2ePdvsi4SozcuUS\/OslNKmQTlRAZ1JEzcqM8E+jqBma\/7EJW1Op53mAifd9i1CC7DcPASx9p0haQdQTe1luDwqOujakK0mHpQyFlnbrZRb0SQNJUWxihmMvPMQO+4LFiygqVOnFrV7X8z3K\/Ude58oCgg+e06VOsYs6QL\/NmzYQLfddhvNmzfPWDzFZITYNJYFIE1NTcbpxwOAyDtDsmRiJfdtmwauy1OlCYax2MmflTy+9qKt1JSpWIDYE4DGCN3KBMRSBypXRgXIZ9z0AYjN+yxvIC51nsv1fmYAkWFedsj5dxhsKJCoieUWHV8TS75t1zErl1BW0nczA0h7lf1RJz1anJKcdIBo5cqVdPvtt5vs6BD3R1aScIegJTOAgDiXY5gm18p3gDJSFira4PvtSm5nZzKw1gYQ8EyePDmv6oz6IIWzmRlA4q4\/YDKiYvOVLHRKm3KgFA60exSrFGL1XeVAe3MgMlmRCQkdxWrvAer3lAOlcMCZrCjTSlxnzUv5YDnf9d1kC+l3Sb\/CJy0kdKpEMfyWNIfOpGD\/s6MsvM5kxZ49e+b4WupR3GImKKt3ygGQtPyrJIBkleGA8DRni3P9gqzmvNR+C5IVcRaksbHRhBE5MQ5oD5msWCrRSe\/LhD6O9CBDedq0aeZVXsll2SJu19zcnCtlxKtn3FXWTIu9yYro3LBhw2j69OmEU5pRESfZN95BIATZBSNHjqQtW7aYSjFyFXfRXFdXZ945efIk7d27l9APHjleWXEmaTyuTUtkP3Tr1s30HVfiiS+BjQN6hwUImConAD+HVrFJwl3q312Tu2fPHgNwqUHsXWh5HFiaWHKvAcUT6uvrad26dQV5PtIUBSCQD4SFplevXpGVXSQ9XEZpxYoVhANneBDmlX0B5PgOgAMtzzTPmTPHAAQgxDhlqJMBx\/3J8z1R43HxEGCTCwb6sxdNybeqBUipAlru9+MmJs7Ecm3STZw4sSBXzOWf2N+Uu+Ljxo1zAiSKTntHPa50EtPMAOGcNns\/i39eunQpLV++PC\/3zWc89mJi10TjOVeAlFv6Pb8v93SkOSBB4Mo7Y9OLJ5oBArNTPraz7dqc4j6iABK1oWUDRwKETSlJD2hhgLDPYO+w2wBJGk+UicUblwoQT0HsCM3kagrzgssT2UIUp0GSnNUsNAh\/UwIE\/pE8\/mxrEH4nSYOkHY+tQaKSJKUGidvR7tA+SEcQ+jgabZNEVlaRIJAAgYDDmeaTk1E+CLebMmWKSfuQTzE+iPR7OPEQJtDjjz+eOyMTBRBJs61BfH2QqPFE+SCcFmSXb5ImFgcD4DvBb3GlEilAyowymeslTSz+PcySGTNm5CJMaIOfd+3aZRxjaBoU5XZFsaL2MlxRLAAuzodwvcNOtUuDwOHnqJikmX0LqRl4rHDc8c7+\/ftzGdlJlWZcANm8ebOZVQQ85B6GS2swOE6fPm2AjmieLEGrACkzQPTz+RyIWvGj+JTkg5TKXwVIqRzU90vigF3YIe2utb2TPmjQIOJToKVu7HXYnfSSZkRfVg5UKQf+H\/2Lx6kLOORPAAAAAElFTkSuQmCC","height":120,"width":200}}
%---
%[output:7fb89984]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:684c5367]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:927c08f3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
