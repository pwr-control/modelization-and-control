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
print('bode_phase_shit_filter','-depsc');
%[text] #### Resonant PI
kp_rpi = 0.25;
ki_rpi = 45;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta*omega_set] %[output:7b890019]
Aresd_nom = eye(2) + Ares_nom*ts_inv %[output:7392f24e]
a11d = 1 %[output:55fe855d]
a12d = ts_inv %[output:577850ee]
a21d = -omega_set^2*ts_inv %[output:88965eb4]
a22d = 1 -2*delta*omega_set*ts_inv %[output:8fc1ceb0]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:4d8cbfe6]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:8272fe3b]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:4b5e32e2]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:41d8c6b8]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:00130ac3]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:25f30ff8]

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
figure;  %[output:0bf428b5]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:0bf428b5]
xlabel('state of charge [p.u.]'); %[output:0bf428b5]
ylabel('open circuit voltage [V]'); %[output:0bf428b5]
title('open circuit voltage(state of charge)'); %[output:0bf428b5]
grid on %[output:0bf428b5]

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
heatsink_liquid_2kW; %[output:37f5c355] %[output:58a706d0] %[output:2e2041d5]
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
%[output:26f707e8]
%   data: {"dataType":"matrix","outputData":{"columns":3,"exponent":"-3","name":"num","rows":1,"type":"double","value":[["0","0.966531084866133","0.946498544476604"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000","-1.938144852609621","0.939101367424292"]]}}
%---
%[output:2a03b5f5]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVUAAADNCAYAAAAITuO0AAAAAXNSR0IArs4c6QAAH6ZJREFUeF7tnQ9sVVWex3\/O4i51VjPUQLSAbd2BTJSERLeBdDaWWWLwH52EzYo1m8UG2YaVMAkwtEB2ClH5t7KJiCEITWWCFHWHzdiNCRqQTmYayFbGruAijNAyWBW3MBEFJ8OGze+U3+P0ct979\/d63z333ve9iWl5755zz\/mc04+\/e+6559x09erVq4QDBEAABEAgFAI3QaqhcEQmIAACIGAIQKroCCAAAiAQIgFINUSYyAoEQAAEIFX0ARAAARAIkQCkGiJMZAUCIAACkCr6AAiAAAiESABSDREmsgIBEAABSBV9AARAAARCJACphggTWYEACIAApIo+AAIgAAIhEoBUQ4SJrEAABEAAUkUfAAEQAIEQCUCqIcJEVv4ETp48SY2NjTQwMJA5Yfbs2bR+\/XoqKyvLi+3y5cvU0tJCDQ0NNH369LznHzp0iJ588slh5zU1NVFzczNp88p7MZwAAh4CkCq6RNEJsFSXL19OGzdupEmTJqnFphUhS3XDhg3U1tZG5eXlmetVVFQYseIAgWISgFSLSRd5GwL5pGpHlhJRcjoW47Zt26iurs7kw99xpPr666\/TihUrMp95RemVqpRh7dq19Nxzzxm5c9Q7depUEwF3dnaavCR65t\/lc\/7sq6++opUrV9KRI0eou7ubzpw5Q0888QRVVlYOi4h3795NkydPpqVLl9IDDzxAzz77LLHIX3jhBVOX3t5eWrduHc2dOxc9I8UEINUUN25cquZ3+y9ysYU7YcIEI7Pa2lojLIk2BwcHzfABy4mPjo4OM3Qg8vMOC\/hJ9fz580Z2S5YsoR07dhipTpw40eQxfvx4ku9tefI1WITLli2j9vZ2I9U9e\/ZkImC+jgxHsOj7+vpowYIFNH\/+fPM5y57rwOdx1Pzuu+8aKQcd9ohL+6EcOgKQqo4Xzi6AQLZIVeQpkuTxVZGTXMY7Dtrf35+JUuUcO7rlz4JKlcUnQwscrXJUuXXrViNdLhtHlNlkK2PB3ihbpMrllqiaZcv\/5nPtuhaAEkkSQABSTUAjJb2IXqlyfUSefGuvlapIKhuXoLf\/nJ4faNm37RLJ5pOqRMn8kyPPt956a1ikCqkmvdcWXn5ItXB2SBmQQK5I9b777ss8xAp6+y+34\/b59jhlrgdVixcvzswksIcSvLf5cpue7XN76EHGZjnSRaQasFOk+DRINcWNG5eq5ZtSVYwHVUGmVPFDJR7\/ZHHy+RcvXrzhAZbfgyoZE5UHZixTzueDDz4w\/4NYtGiRud3H7X9cemC05YBUo+WNqyWMgN9QQsKqgOJGTABSjRg4Lhd\/AvaULS4tj7kGeekg\/jVDCaMgAKlGQRnXAAEQKBkCkGrJNDUqCgIgEAUBSDUKyrgGCIBAyRCAVEumqVFREACBKAhAqlFQxjVAAARKhgCkWjJNjYqCAAhEQQBSjYIyrgECIFAyBCDVkmlqVBQEQCAKApBqFJRxDRAAgZIhAKmWTFOjoiAAAlEQcCpV7+uAXGGsjB5Fs+MaIAACxSLgRKqyKpGfQEW0eN+6WE2OfEEABIpJIHKp8krqvMr6vHnzctZr586dec8pJhjkDQIgAAKFEIhcqoUUEmlAAARAICkEnEjVXrSYhwD44N0xebFf3mCNtzHGAQIgAAJJJBC5VO093GXldd7ul7fD4PFU7DaZxG6EMoMACAiByKXKY6pr1qyh1tZWKi8vN1v48uZvvAiw9zs0EwiAAAgkjQCkmrQWQ3lBAARiTQBSjXXzoHAgAAJJI+BEqrKDpR8s3u63ra3NDA3gAAEQAIGkEYhcqnEDdPfdd8etSCgPCMSGQE1NDVVWVlJPTw\/19fXFplzZCnLq1CnnZYxcqvwwKk6RKks1Dg3hvCegACDgQ+Dtt9+mo0eP0iOPPEJTpkyJPaM4\/D1HLlW7VXgKFf\/fr7m52XzM\/+aDp1dFdcShEaKqK64DAloCkKqWGJEzqfpNnyrWlCpZa4DxeNcbgFT1nQYpSocApKpva2dSlZcAamtrM5Epz1kdGBig9evXU1lZmb42PilY1EuXLqWVK1eab9euXUubNm3KPAiDVEPBjExSSgBS1TesM6lyUb3jq8V48s9RKsuaZxSwqFtaWqihocG8bMBH949vpgnjJ+jJOUxx5cvoHhiMGls1rKajxg39++axVSTfjXm81SENXLqYBCBVPV2nUtUXV5+CpdrR0WGiXz5YqnZ0jEg1GNMLb6wxJ4rQ\/3RN7FfO9WU+8+bE0mUJQ8DBGMfxLEhV3yqRSzXqpf\/ySbV8yXt6akiRk8D9f\/yA7v9jrznnziufU8WVL+jO\/+Ofn2fSDYy6w\/z+\/l9Mpc9G3UGf\/dkd1PndWSAbQwI\/HXuIpk2bRtXV1TEs3fUizZw50\/zD9WyeyKXKlY5ykep8t\/+IVKP9O+FIl6Pby8e6zIUvf3SQvj12cFghOMK9dcZThGGFaNvG72qIVPVt4ESqUswotlPBgyp9p3CVgoV78b2dvrIV0fKXZffW0eh7Z7gqZkldF1LVN7dTqeqLW1gKe0qVd5sWRKqFMY0qFYv28rGDdOVc\/7CoVsZry+6ZAckWsTEgVT3ckpBqLiyQqr7TuE5hR7QX3lydKY5Es4hkw2shSFXPElLFa6r6XhPDFBcPvuobzfLYLCRbeINBqnp2kCqkqu81CUjBD7\/4YZj9IAwPwPQNB6nqmTmVqj35f8eOHXTgwAGzg2qUe1Th9l\/faZKYApItrNUgVT03Z1K1X1PlRVV4SxU+ZKJ+WK+p5kMCqeYjlL7v7YdfZtjg2osMPKOAH3xhKtf1NodU9f3fmVTtxVO2b99upMobAdr7V+mro08BqeqZpS2F34MvHioou3eGeRW3lCULqep7uzOp+kWqXV1doS+okg8JpJqPUOl9L5LFeCwRpKrv\/86kykWNYkGVfEgg1XyE8H0pSxZS1fd\/p1LVFzf8FJBq+EzTnKP9mq2Mx9ovIqRtqABS1ffmyKWK7VT0jYQU8SWQbTw2LWsXQKr6vhe5VO0iYjsVfYMhRbwJpG2oAFLV9zdnUi3Gdir2O\/6MQrZOwXYq+o6BFCMnkIahAkhV3w+cSbUY26l4I195GIbtVPQdAynCJ5DEoQJIVd8PnEm1GE\/\/eduUqqqqYbuxYj1VfadAimgIJGGoAFLV9wWnUtUXN3uKbNOzTpw4ge1UwgSNvIpCwB4qkJW34vACAqSqb25nUs02CyCszf94KKC7u5vmzJlDe\/fuzblHFWPbv3+\/nh5SgECxCFw4S1d7fkF06jDRJ4eGrjJmAtFf\/x3d9OBPinXVG\/I9fPgwnT59GtupKIg7k6pfGVmElZWVmZ1Oc9Xj5MmT1NjYaN7Amj179g3bWstt\/8KFC2nr1q1Zd1PFPFVFb8GpzgjIgjBRz41FpKpv8lhJ1W9GQNAqedPy+CofCxYsIDyoCkoR5yWBQK6x2LDXjoVU9T0iVlK1HyqVl5era2NPnbKHEbCdiholEiSIgEjWHosNa3FuSFXfEZxJNduYqswt1VelsBS4\/S+MG1LFl8CFN9ZkFueWhblHjas0O9RqD0hVS4zImVT1RS1OCki1OFyRq3sC3mGCQvbwglT17ehMqsV4o0pffSJItRBqSJM0AtkEm28BGEhV39KRS1XepOrs7PQtrd+TfH21gqeAVIOzwpnpICCClZkEvONBxer3fCsHqerbPHKpShFH8qRfX83sKSDVMGkir6QRYLHyGCyLluU67pl2s9uBHJCqvkUjl6rIdPHixbRs2TLq7e0dVuqwJv8HRQGpBiWF89JMgOfBnnu50ciVo1YWLB+Qqr7VI5eqvojFTQGpFpcvck8WAY5aeWrW3W9ehVQLbDqnUuU3qFasWIFItcDGQzIQKAYBjloHVv\/IRKwH+i\/R0aNH6ZFHHqEpU6YU43Kh5hmHIMmZVGWeanNzc6DXUkMlb2UWh0YoVt2QLwiMhMCpv7+Jjj72Mh3\/4mtIVQHSqVSj3o7ajwukqugtOLWkCHC0evyLi9RT\/Tikqmh5Z1LlMvLtPx9z585VFDncUyHVcHkit3QR4Gj132s2QKqKZnUm1TCW\/vOu9G\/nac93xXYqih6BU0HAIsBS5Uj1nn\/8F4ypBuwZzqQasHxZT+NVqLZt20ZNTU3E47J8yMr\/9fX11NLSQg0NDTR58mSsUjVS2EhfsgTO\/HM1\/fefT6Kqp\/8NUg3YC5xJNddW1RUVFdTe3k6TJk3yrYasu9rV1WW+Z6l6H3xJFFtXV2dk29bWRmVlZRnZTp8+3aTF7X\/AnoLTSpLAly830odHP6Rxz7wKqQbsAc6kKmOqfX19mUhTxlh5oeqOjg568cUXc1ZD1kwVqcq6qSxjrPwfsAfgNBDIQeDqGz+lcx\/\/li79w3aqrq6ONauZM2ea8p06dcppOZ1JNdeCKvy21ebNmyOTqutGcNoDcHEQyEHgo58\/S6M7f0aXWj9EpBqwpziTqiyswrf6MiYq0eXy5ctp165d5nN7ARbvsIA3Up0\/f75Jw7f2uP0P2ANwGgjkIIDXVPXdw5lUuah+O6Bu2bKFNm7cSLW1tXmnWtlS5fzwoErfAZACBHIRgFT1\/cOpVPXFHZ7CK1Vb0vasAGynMlLSSF+qBCBVfcs7lapMi7KLjVWq9I2IFCBQLAKQqp6sM6lyVClP648cOWK2pu7v7zc1iPINK0yp0ncapCgdApCqvq2dSlXe\/T9x4gTxnFPeTjrq9QBYqjhAAAT8CdTU1JiAp6enh3j6YxIO17N5nEmVn+rztCkW6eDgIDU2NtLAwABFffufhE6CMoIACCSHgDOpMiJe9f+WW24xb07xwyTeCSDXm1TJwYqSggAIlCoBp1ItVeioNwiAQHoJQKrpbVvUDARAwAGByKWaayEVrj\/GVB30AlwSBEAgNALOpbp7926n26mERhIZgQAIgAARRS5Vm7rfa6q8RF95eTkaBwRAAAQSScCpVL3EeAaArH0KsSayP6HQIFDyBJxK9eTJk5n5qdwS9hYoJd8yAAACIJBIApFL1b7lx0OpRPYZFBoEQCAHAadS9SsXRIv+CgIgkGQCkUs1ybBQdhAAARDIR8CpVHl1\/hUrVgwr47p16yJdpSofIHwPAiAAAhoCTqQqi0b7CVREi\/mrmmbEuSAAAnEhELlU+UFVZ2cnzZs3LyeDnTt35j0nDIjlS94LI5useXzn0v\/e8N13Lg2az+zv5LPRx39Z1PIgcxDQEEja0n+ul\/1jtpFLVdOgUZwbxSLVG\/YNX4fyzPnLpmq\/P\/9tpopnLgz9fsb6LFv97yofbb66a8xomlg+mu4qL6PmWVVR4MI1SowAFqnWN7gzqeZaA8C7a6q+WsFTRCHV4KXJfWbHf30+TLosZxHzrz\/5ww2Jbfn+8PtjhkRcPpoaau4Iq0jIJ+UEIFV9AzuTKhdVtpG2t6jmz3ml8Y6ODnrxxRf1NVKmSJJUNVX79e\/+QL+5JlqRr1e8LFiOdkW4P\/yr79HffP97msvg3JQTgFT1DexMqhyperdOkc8WL15sdgWAVPUNGiQFDzGwcGWo4Te\/u0C2cG3ZYlghCNH0ngOp6tvWmVR5O5WWlhbiW307Uu3u7qbly5fTrl27Mp9nq5Y30rWHFOxXXu0tqr0zDtIaqeq7wlAKiXA5uuXhBjls0SKiLZRu8tJBqvo2cyZVLqrfKlVbtmyhjRs3Um1tbc75qrK9dVNTU0a+\/FlVVRXV19cbYTc0NNDkyZMzu7byNdeuXUubNm3KrIQFqebuNBzNSmTrF9E21NxJkKz+Dy8pKSBVfUvFRqo7duygAwcOmGlUvGdVroMjVB535R1Y+eBIVwTNv0+fPj0zXltXV5dZ+aqsrCwjWz6HD0hV32lYshLFbth3elg0C8nqecY5BaSqbx1nUpXbf45Ieetblh8f\/IBq\/fr1xALMd3Bkakt16dKltHLlSiNlFi8PJcyZM4f27t1r8uSDI1g7CoZU81HO\/72fZDEum59bEs6AVPWt5Eyq9oOq7du3G6nyrbr34VWuKoUlVb7G\/v379fSQwpfAwFdX6D\/\/52vq+fRbev\/Tofm3FbeNotk\/+Eu6f8Joun\/80DxbHPEncPjwYTp9+jRNmzaNqqurY13gmTNnmvK5fgHAmVT9IlW+nR8YGCg4Up0\/f74ZCsDtf7z6vkSy9pjs0HzZO82cWZlPG69SozRMAJGqvh84kyoXdaTbqdiRKueHB1X6DhB1imxDBSxYPjCFK+oWyX09SFXfHk6lqi\/u8BReqdqStmcF2FOqvAu1YEx1pK0wsvS5oljMKhgZ2zBSQ6p6ipFLNW5bVEOq+k5TrBTZpm\/JUAEi2WKRz54vpKpnHrlU7SJme0117ty5+poUmAJSLRBchMl4QRrveKy8XovhguI2BKSq5+tMqrleU21tbY1sm2pIVd9pXKaQN778XkTg4QJesWvoJ6\/ehVkGI20rSFVP0JlU7af\/EpnyGKnm6b++ujemgFTDoOguj1xvfHGpWKwiW0S1+naCVPXMnEmVizrSp\/9Bq4t3\/4OSStd5so4tR7W8Xq13rVp5QUHWpOXas4D5wGpdQ30BUtX\/TTiVqr64+hQsbnnTilPj3X89w7SmYOnKsoh+0pVI1\/wcMzSUYAtYhhfSvD4tpKrv\/ZFLNertVDhK5WGFtrY28+qrLLSCd\/\/1naWUUvDY7e+t6NZeELyQXRpsMYucbZ48Fuw94jBcAanqe33kUuUiRrnxH19L1hPga3vf\/d\/6t7caarNmzRqiN2ZCVoo3PfgTPWGkKGkCrxweviPDwMUrhge\/ymsfn137XD7zfh8lRH6lWI5vvvmG\/qn8t3hNVdEATqQq5Ytii+p8Uv2Ph8po2rUVq6RcV84N31PK5nnly+zfKbjTqLHX95QaNe767zdf+1y+H\/N4qyZbnAsCoRJApKrH6VSq+uLqU8Tx9v\/iwVfpyrn+TGVE1H+yhC1izyVxES9LmWV8\/d+V5t+j752hB4YUIGARgFT13SH1Uk3bg6pvjx00rcwCFjGzeEXI8r3dFVi2tnhHjaukW2c8pe8tSFFyBCBVfZOnXqqMpFTf\/WfB2vK9\/NFB4gjYjn5FuGX3DEW1ZffWIcLV\/x2lNgWkqm\/akpBqLiylOvmfxcqCvXxsaPcEFq4d5bJsy+6dYYYUIFr9H1ZaUkCq+pZ0KlV78r9mOxV9NbOnKFWp5mN44Y01vqLlYQM8PMtHLz3fQ6r6tnQm1TC2U9FX98YUkGowivJwzY5oJZod+0x7sExwVuIIQKr6JnMm1TC2U8EW1foGDysFDx9cfG\/nsGiWJYtINizC8cgHUtW3gzOpjnQ7FWxRrW\/sYqaAZItJ113ekKqevTOpclELXVAFW1TrGzrKFPZDMDNs8GXftQdeMwhDBVG2xMivBanqGTqVqty+L1iwgHjTvt7eXvJud5KrSmHtpup690V9syUrBaLYZLWXXVpIVd92zqTKt\/\/PP\/88zZs3j44cOUJ9fX00Z84c2rlzJ61atcosfpLvCEuqfB1sUZ2PdojfXzhLV3t+QcT\/XTg7tN5C+QSiu6cR1lcIkXMIWWGLaj1EZ1L1PqiqqqqiBx98kNasWUP2yv8y9trZ2UkVFRXU3t5OkyZNMjX1ShVbVOs7gOsUEsWanwdfNcXBAy\/XrXL9+ohU9W3hTKoSqT722GO0bds2WrlypSl9oZGqSJblXF9fn1nib\/LkyVhPVd8vnKXAUIEz9L4XhlT17eFMqlxUeX2Ut5PmcVVZTFoi0XzVwRbV+Qgl+3v7gZff\/Fi87VX89oVU9YydSlVf3PBTYPJ\/+EyLlaOsWeCdH2sPGfDveK02vBaAVPUsnUpV5praxZ46dapZpb+8vFxfmwJSQKoFQItREhbt5WO8UEy\/72u1vDoXLxaDV2sLazRIVc\/NmVTtJfn46X9lZSX19w+tMSq7q+qro08BqeqZJSGF\/VptvpW5INzsLQqp6nu7U6nKk\/4TJ05QV1eXGVf1Pv3XV0mXAlLV8Ury2XZUK2vQeteftRf+luUQS1m6kKq+xzuTKj\/937x5sxHp4OAgNTY20sDAAAW5\/be3YbHPt9\/Qmj17Nq1fv97Md8UW1fqOUUopZKzWXgoxm3Rl\/Nb89Oy4kMaxXEhV\/5fgTKpcVH6D6pZbbjHzTll8y5YtGzYP1a86J0+eHLbNNI\/LsoxZoCxpTKnSdwKkCE7A3gpHxOsdXpDcsm13w\/LlIwnb3UCqwfuGnOlUqvri3phCNvZbvnw5LVq0iJqbm4m3n5ZXYOvq6rBFdRigkYeaAK9Jy0cQ+Xqj3rgMOXz082dpdOfP6FLrhzRlyhQ1g6gTxGE4z6lU\/XZTDXL7bzcUR6ryNpY9z5Xz7u7uNq++7t2710SyfHi3qOZG4AOvqUbd\/XE9+uQQ0YVP6Sq\/qmv++5To\/LXfbTzyGu+Y8eaV3ihf5T339lYae\/BfqW\/hfqquro51o82cOdOUz\/VaHs6kKuOfElkW0lr2eqr2bAIeTtBI1XUjFFJ3pEk3ARnnleliXFvvljf8WbG3vUGkqu9nTqUa5El\/tnf\/JUKV6VdeSeP2X98ZkCIZBLzC9dtfTObnjvThWd+OJfQ1r8nQ8ivc\/gfsHs6kyuVj8fGhnZfKQuWxUh479RsKwLv\/AVsfp6WSQJj7i0Gq+i4SuVS9C1N7i5xvTNWeHiVpZfoUR7WyLiuvJ8BDC3yU6hbV+u6AFGkkkG1\/sSBb30Cq+h4RuVT1RSxuijg8LSxuDZE7CFwnYL8AceHN1eaLXEstfvlyI516\/1f03SW\/xO1\/wI7kRKo811Qm+9uT9AOWOdTTINVQcSKzhBGQpRbtbW\/sCHZg9Y\/o+BcXadwzr0KqAds2cqnKg6eGhgYzJup94BSw3KGdBqmGhhIZJZyA31q2XKWuHzRRzdxFkGrA9o1cqvaK\/7wSFUet+\/btMxP3XRwyT9XFtXFNEIg7gZqaGrPYUU9Pj9nyKAmH6ymSsZCqZrX\/JDQqyggCIFC6BCDV0m171BwEQKAIBJxIVaY9+dUn35SqIjBAliAAAiAQGoHIpRpayYuUET9Ie\/PNN+njjz+mp59+OvbvOxcJA7IFARAokACk6gH3zjvv0O23326WI3zttdfoqaeeMmuy4gABEACBIARKRqreBVcYjr1K1u7du80Ur1deeYUefvhhmjhxotkum+fRRrVfVpAGwzkgAALxJlASUpWXDbgp2tvbTRRqL3bN27l0dHSY5QH5J4t07NixkGq8+y5KBwKxJJB6qXKEun37dnrooYdo9erVtHHjRiNVWRqQRcrjqLIW67Fjx+iee+4xa7RypMqLvdx2222xbDwUCgRAIH4EUi9VQc6RKe8OYEuVJzPzoiv2soF8289DAGPGjDHyffTRR+PXaigRCIBAbAlAqh6pepcTjG3LoWAgAAKxJFDSUuXtVry3\/xyd4gABEACBQgmUrFSzPajC9KlCuxLSgQAIMIGSlSpXXqZUVVRU5N0aG90FBEAABIIQKBmpBoGBc0AABEBgpAQg1ZESRHoQAAEQsAhAqugOIAACIBAiAUg1RJjICgRAAAQgVfQBEAABEAiRAKQaIkxkBQIgAAKQKvoACIAACIRIAFINESayAgEQAAFIFX0ABEAABEIkAKmGCBNZgQAIgACkij4AAiAAAiESgFRDhImsrhPghb9bWlqos7NzGJampiazhm2SD16MZ9++fcS7AnMda2trzWLmfEi9GxoazPY83oO\/37x5My1YsADb9CS5E+QoO6Sa0oZ1XS2Riy0c12UK4\/q2FHlFM61UuQwi5UWLFoVRJOQRMwKQaswaJC3FySVV2crmzJkz9MQTT9B9991HjY2NNDAwQFOnTqW2tjYTxR06dIiefPJJ4lXEZsyYQbfeequJ8DhC5GiXI0HOS3ZwsDdylIiY89i2bZvB2tXVZfYf4zV0WYgbNmzIfMcbP\/Ihe5Xx7yxMb8TJ+fX395vI1K+OdqTK15Nrc372tbds2UKzZs0yu0vgSBcBSDVd7Rmb2vjd\/osw3333XdqzZ4+RJx+yP5jsHcaStOXJ6VhwLNdsUq2rq\/MVIue\/bNkys7Qjbz0uQubPWapchsHBQVq7di2tWrWKXnrpJWptbc18tmnTpmG36ZyGr8VCzzbEwXnL7rzSIHY6\/oz\/B8CHDBvEpuFQkBETgFRHjBAZ+BEIEqlyxHj27NlMlCr5sEQXLlxIW7duzUStEpFmkypv1LhixYphReFolQUo8pTbdY4+OdqUCNdOJPKTyNYe\/+U6Pf\/88zRv3jwTYeaLVGVM1StUzpsjXo5kkz6+jN5\/IwFIFb2iKAQ0UuUo0RsRsnREhjwUEESqfpK08wkiVZEdQ5GIVAAVItVsESmkWpRuF4tMIdVYNEP6ChFUqnwe35Lz2CrfCst4K+98yw9yOJKzb\/8XL16ceThUX1+fGRZgAcpt\/oQJEzLnVFZW+kaq3tt\/2WmX0\/LT+ePHj98geknjvf3P9vSfo+Fst\/i4\/U9fn5caQarpbVunNQsqVY4e+Wl40AdVLFl7Gxx5gGV\/zhW3H1T53f7LQy4ZMli3bl1mfNN++OWFaEeYuW7\/eWtzHr7o7e3NZCFjylxnexjBaUPh4qETgFRDR4oMi0Egl+jCvF4U80wxpSrMFotfXpBq\/NoEJfIhEIVUz58\/b4Yi7rrrrsy0K7\/GGMl4qHdcFo2dPgKQavraFDUCARBwSABSdQgflwYBEEgfAUg1fW2KGoEACDgkAKk6hI9LgwAIpI8ApJq+NkWNQAAEHBKAVB3Cx6VBAATSRwBSTV+bokYgAAIOCUCqDuHj0iAAAukjAKmmr01RIxAAAYcEIFWH8HFpEACB9BH4f1wmaxFmVj4BAAAAAElFTkSuQmCC","height":205,"width":341}}
%---
%[output:7a92d0b3]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVUAAADNCAYAAAAITuO0AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQfUFcX5\/wdEioBIVVoEExDBIAg2RBFBJBSNggKiGEAkSLFAECSHLiAtoRjyioASC6FGEDCICGjASlETFQ1IVVqQpjThf77jf+5v331nd2Z3Z+\/dvfvsORy4l9nZmc\/Mfu\/MM888k+\/cuXPnGF1EgAgQASJghEA+ElUjHCkTIkAEiAAnQKJKHYEIEAEiYJAAiapBmJQVESACRIBENU194Mcff2QDBgxgS5YscXxi69at2ZgxY1iRIkWMleqrr75inTt3Znv27GGvvPIKu\/76633nbc1Llom9\/O+99x677777WIUKFdisWbNYtWrVfD873Tf+\/e9\/ZwMHDsz1WFn7PPPMMywnJ4ep2s7a\/t27d2dPPvkkz1vWL9BOV111Fe8vDRo0YO3atXOsvmDsxmf06NE8D1lZncqP+q9bt854f0x3O2bieSSqaaKuI6ooCl6mGTNmsFKlShkpWTpF1V7+OIqqqp3s7RNUVO3ijR+gSZMmsZEjR7LNmzczIYhOnSEMUdWtk5EOmoWZkKimqVFVL6u1GEFHlNa80i2qeLYQgjiKqo5IWUeaQQVIiKpVrP\/3v\/+xrl27GhdVWVf3MnpN06sS+8eQqKapCZ2mf+LxVvGzj05k026Z8NqFG1PSLl26sJ49e+aZ\/tvz1Bkhuwm0VQiE6FhF9dlnn2UzZ85MmT9kz5NNue0srM8R7JzKbhdI1RQd+Yky2E0WVrbW51lFycoaeTlN8\/F9nz59pOagG264ga1fvz5Pr3T6ofXyw6US0P79+7NevXpxMbde1nqomFp\/JLp168bzs7NI0yuXsceQqKYJvRdRtb5AMqERRbZ2dpnY2Ksm8nUajalsn35F1QmxVeTc6imE1a2OdmF1yk\/142G9z8rXqQ5CqJz+X5Td3v5xFFUdpk5pTM6+0vTK+n4MiapvdN5u1J3+W4XGKmLWF9zacUVnlX1nFyGkrV69empqKfJUCb5sNO1We\/v0H2llAipEvHTp0qkyifpYyyTuxQjKvvAlG+HLvtOdUjsJt5MYW0VVJqCi7GAgFiplbRnW9F9lqlCNXsXCqS5Trz9K3t6ieKQmUU1TO+mKqmyUan+h3UY9TivwqCbyxiVbkRejV7eRnGr13y6e1hGxtV4qO6\/TFFOIKp7jVk6ZndI6tVeZAdzqaR\/NC1FyGimL7+HREWdR1WW6ePFi7jWhmvWk6bXLyGNIVNOEXVdUURw3FxhRXOsIY\/DgwWz48OHcXmmfstoFbPv27XlchawI3F4Glajan+1k75OJqtsCkWy0Zy2z3e6qmpKrTAAib51Rq5tLEsQlW0RVl+mbb77J+5cu4zS9fml9DIlqmnDrTLHto57p06c7+kBmWlR1bGS6oorpv\/ClFSNq4aeJHwr7yNJJ7ESZdAXAq9uazMQSFVHVGRnqTvXd0jm9LkJESVRpm2qaJDW3k7fTAohdVJ06aBjTfx0Qqmm7PQ9dURUmCevoRmZTlW2KkKUTU1CvoyWrWMtMBNbRtGo2YZ8uhz39D1tUnab\/9jbXTafT3+KahkaqaWo5L9N\/8ULv2rUrNYLzslDltFrutFAFBDr+lmGLqhilYteXVcAEDyd7nf3H6ODBgyluTqvvYkeTvflVo1yktwpYUkaqsoUq2eyLRJVGqmmSVPl2RKeH67pUWW2JXlyq3NyX3Kb1YYmq1SNBxkSIKl5i4RQvS2fl4SSOqhGdDkfVirp1UcyPTdVp66psi7FJP1Wx0m\/vH6K+OkxJVElUIyWqTi+8X+d\/vNAYkfXr10\/p\/K8SG4AKS1QhFvY6QiAvvfRSqaeC7OWW\/RjYF7+8mANkPzwyRmGMVMHaXnan7aphiKr9h8VqClExJVElUU2bqNKDiAARSAYBsqkmo52plkSACKSJAIlqmkDTY4gAEUgGARLVZLQz1ZIIEIE0ESBRTRNoegwRIALJIBBIVM+cOcOOHj3KSRUvXpwVKFDACDXryqvOqrT1oVhFHjVqFJswYYKxQM9GKkWZEAEikAgCWqIKF4vly5ezH374gf3mN79hlSpVYh9++CFD\/EXsJcdVvnx5HjCiZcuWLH\/+\/L7hQVDnzJmTin5v\/+yWsXAFQRqT0fN9V4ZuJAJEIHEElKK6ZcsW1qNHD7Zt2zYOByNHBPDAkQ8Q2ebNm\/Pv33jjDXbgwAE2bdo01rBhQ18ghSi2b98+dS6PcIT2claPrj\/iZZdd5qucdFP6CYhZUMGCBfkZXuIz\/jY1Q0p\/reiJYRB4++23w8hWO09XUcX0Hs7Nq1atYmPHjmVXXnklDx83YsQIVq9ePTZlyhR2ySWX8Id99913rHfv3uyXv\/wlj5iEzu\/1wtQdo188y3pInOoQMuGQDAdpXNaRrlsZIKpbt271WkxKbyFgiiH6D\/rS+PHj2e7du3l\/wp+wL9F\/xXPsn63fQ7zPP\/98dsEFF3BzF9Lmy5ePXXjhhfwz+vx5553na6Z28803s7Vr1xqtLsqXDoZGC23LTLcOONiyb9++\/O5Ii+qRI0f4cQg1atTg4bzQgXbu3Mn3Vd96662p7wQHCPBHH33EnnvuOVayZEnPrCGOyMM+dfdiAvCS1pQgOFUUo\/uqVat65oAbdO9VpXP6fy\/f29NaP6ueL6s8XvTZs2ezjRs3+n7phfjhb\/y5+OKL2U033cSKFi2a65Hor5UrV079+PttKy+8nNrPNEevHctPW4ln6N6rShcmx8aNG0dfVMV0HNsIRQAK2XdWUYUw+rVnZpuoeu30cUyveokgoC+99BL7+OOPtQVUCCVmK3fccUdqei9GLU6jyTjy8ypaca5j2GXv0KED72ORHqmSqIbdDeKfv11U0alffPFFbmNXXRDHOnXq8B9sMf1X3ZOt\/6\/6ccrWepusF4mqhGYmRqooxltvvWWybROVF8ITnjx5ki1YsIB98cUXjnUvU6YMw4wHtkP8m67cBMARXjV0+SeAY4IKFy4cj5EqRhNYhMJ1+PBhfrxu\/fr1U98JDFi42rRpk+\/pv9+FKmszkE01d6cMy4YFrxCEinO6MArFguaxY8f4aDTIFWQUp3uvKl1YHIXNXfX8IPxMmBh0y6dKFybHxx9\/nOtPLKb\/9nPA3RpY151JlkcQlyqRH4lqeKK6Y8eO1FTd3n4QUbi93XPPPcpFIa8CoXpR3fLTvVeVLkwxQPlVz\/fKTJY+yDN071WlC5NjLEQV07pPPvmET+90r0KFCrHatWsz\/O3ngihixDtr1izuVuVFJPE8L+nDXv33U\/+o3QNbJzwyMAKQCSk8QZo1axa1YseuPCoxil2FMlBg9FPY8iM9Us0AF\/5I1TZVwMMlOxKDRNVMq23YsCHl92fNESPSnj17pjZ4kBiY4U0cg3OMhaiePXuW21DPnTunXWP4spYoUcKXA7T2QwwlDHukGuRF0b1Xlc7rdAu75+wLTlhYgg39wQcf5FN7a56q55toqiDP0L1Xlc4rR1l6+3fE8f96hxe+ThxjIao65\/XYX5ogNtUgL6D9mAedI5TDFtUg9UnXvWLHjWyKDwH905\/+xIvi5BuqEqN01SPuzyGOwVswFqJqt6lixLpy5Ur2wQcfsPvvv5+fIYQLe\/7hm\/jtt9\/yuABNmjTxbVP1gxaCinOYhB3W\/tkpz6SLKgQVxn37VkYhpjpO9iQGfnps3nuIY3COL7zwAtehWNlUFy5cyAv89NNP8\/3O1guuM9jKihcRts50BbkQAVcQ6MVqY3Wzu4pyJ1VUsegEMbXbShEIB\/ZSLxeJgRdazmmJY3COsRipWqsp4gC0a9eOh\/eTXUuXLmX4tfC7998PVmGigKBaj\/B12khgfUbYohrkRdG9V5XO+v9wiYJd1C6mGOUjQA4usgXKe6EXmx9xdI53ESbH2I1UDx06xB5++GHWtm3bVFg+e\/ebOXMmj7uaTlF12jCgYwJIiqh+\/\/337IknnsjVXFh8GjRoUB7HfBJVElUZAdWPt7hHlY5E1UJXhAFct24dmzhxIrv88stzsf\/88895RKsWLVqwRx99NG3T\/yiLqp+Rt8l7EEpuyJAheUamWHzSsZfqlEX1EunkQWnS4\/yf7ZxjN1JFg+zbt49vUYUPI2KrXnfddbyd3n\/\/ffbZZ5\/x6TfiYZYrVy5t7RdlUQ0iOLr32tNh0QntM27cOFcx9TJiIFcgZ9EjjrlfdVW\/9cLLqxkllqKKSh4\/fpwHz\/jHP\/7BhRQXBBYRYjBKtcezDFtdg4oqyhdWQJUgQTJ07xXp4IGBWLZz587Nhfyiiy5iTz31VJ4gJk75y763f2f9rFvOIP0gyDN071Wl88ILdSWO8hYPk2ObNm24j3ykV\/9hR8UOpWuuuYbVrFmTH2MRtSvKC1XpYoVR6bJly\/KMTLF4FzSYiaoOqpGJ6n76\/58JEMfgPSEWI1W4K8Hva968eTziP1aI7777bnbjjTdym1yQA\/6CI\/w5hyS7VIkAElaWwqUtbDEVzyQxMNOTiWNwjrEQVVFNOP3v3buXR29\/7bXX2Jo1a7ifKqKyY8qf6VGs2E0ldlHprPyjbnFc\/XcKcOIkpmHasNI1wgoiOLr3qtIRR72jgTLJMVaiav8NOXXqFPv000\/57qoVK1ZEYhQbxW2qqg7m9tssW4CS7X6Ca5T1AEZ7niQG0RcD+nHythDotHAaa1G1vrhiFAtXK4Tdgh02JyeHlSpVKvh4PuQcwh6pmij+1KlT+cKgfYpft25dfvJspq8gPxyZLnuUnk8cg7dG1oiqQAHb5jvvvMOFFdtVy5Yt65uSKvSfW8bwBhg1ahSbMGGCUtijKqpuU\/xHHnmEnxoalYvEwExLEMfgHLNCVDFKxQLWq6++ymDPPHr0KAsapcoeD9VLfFThCYDm0TnRNWxR9fKiqIJBOznsq55B03+a\/ptYVFT1M91nhNkfMaDD\/v9Iu1Q5\/XZgdxUiVWFqClsmrvLlyzOMorB4Vbx4cV8\/O0GOU7HaVHWFPdOiCiH929\/+lscdCvBgK8XmCuzL92J7JZtqbgJxEAOyqZqxqcZSVPfv388WLVqUCvMnui+m2q1btw68NdXvwX9CUEePHs2LNGfOnEiMVGVi6DYiRXovYfd8\/XIZvklXtAw\/NuuyI47BmzQ2oopRKXbqIFjK6tWrGT5jM0CnTp1YrVq1eAg5e4Qov3gycUT11q1b\/RZX+75JkybxHWhOl04waO2HpTkhiYEZ4MQxOMdYiCpW8nG8Bqb6EFA4\/rdq1Sq1COW0m8kvnmwQVdVIVIxGhV+p+Kw7bfeaLkwbFk1bzUxbiaMZjrEQVSGaOAEAIf9uu+02VrFixdROqmwQVXRoP3v\/sdce53fB3cl+ppPsRwU2UvwgYaME\/q26VHvRxf2qdGHutUYZVM9X1VPn\/4M8Q\/deVTriuItVqlRJ2VyZ5HjnnXey0qVLR3uhCgf\/wc6JqatY3a9atSoXWCxI4Rjqbt26ZeX0XxwxAsHELjJcsmOaZb1MhNXD8c04rttUmD1lj85AApq2moFOHINzjMVI1VpN+KGuX7+evfzyy9wfFbbVypUr8\/ONnn\/+eU++k7IDBSHa+JWBQ\/vYsWNZtWrVUo+HWxU2F4wZM0YZ1MWLC9YNN9zAwxWiDvZzmnSbWAhm48aN+Q9NNguojAmJgW5PcU9HHINzjJ2oWqts9wLAeVS33HIL69KlCz\/K2O\/5VEFcqkT5vIgqhFDnEkKJv5s2bep4nIxXe6fbs3VfMlW6sG2qYbulBbU3qviINlClI47R9\/eNtaiKjgjzwJdffsldrOBqdfr0aXbttdeyadOmsZIlS+roVZ40EEXsZRcno3oRSWTmJT1GquIHACNvlB8bGLCpARe+o4sIEIF4EChWrFj0bapeUEKMcHwHdjMgKHKQvf+qbapuJ6V6EVUv9aO0RIAIEAEdAvnOiWGZTmpKQwSIABEgAq4ESFSpgxABIkAEDBIgUTUIk7IiAkSACJCoUh8gAkSACBgkQKJqECZlRQSIABEgUaU+QASIABEwSIBE1SBMyooIEAEiQKLq0geww6tv377c79a6bZa6jR4B+3ZkxLtt166d3s2UKkVAHMO+ZMkS\/p04NZgQ+Sfg5uvuP9ef7yRRdSCIQDIIiIJL7O4KCjtp96PjVqlShQup4Dl+\/Hgeb4EufQLY0PLNN9\/wwEVO4TH1c6OUIqh99+7dOVPTV1aLqtshgG5HWmOENX36dNa8eXM2dOjQPAFeTDdC1PPzy9FaLzHa6tChQ2JF1QRH9FucCacTXCjq\/cpv+YJwxLs9bNgwPvM8fvw4iaqXRnA7BBAdE2c\/iRGo\/bN4jtPxLl7KEfe0JjiCgZdTbuPOTFb+oBytJoAkT\/+DcsTsqVGjRmz79u2p0b\/p\/paVI1W3QwBF56xQoUKuXymZjSXpomqKY9Jt06Y44uU3HRjetKCEmV9Qjrh\/zZo1\/L23mlRMlznrRFV1CKBTp5TZqpIsqqY4Jn2EaoqjePHFoKBBgwaJWvQzwREmvZycnFwaGoZdNetE1UpMFrHKSShlJoAki6oJjsgDYSEHDRqkDC5uerQQxfz89scNGzbw6ogFP1kg9yjWN6wy+eVoD3wvFv9Ml5NE9f8TJVF17lp+OjFi6uJECOEGJHJPsj3QD0fY\/XE21IABA1Isk8wQ\/cgvRxJVAz8fJuAbKEbssyCOZpqQOCaDI41UXUaqZrpA\/HMhMTDThsQxGRwTJ6peFqrMdIH45yITA+LovV2Jo3dmsjuizjFxourFpcpMF4h\/LrJOTBy9tytx9M5MV1Sj1B8TJ6poJOGeIQz+Ts7\/ZrpA\/HNxOveLOHprW+LojZdT6qhzTKSoWoWVVqXVHd3tMEW37b7qnJOVgjiaae+oc8xqUTXThJQLESACRECfAImqPitKSQSIABFQEiBRVSKiBESACBABfQIkqvqsKCURIAJEQEmARFWJiBIQASJABPQJkKjqs6KURIAIEAElARJVJSJKQASIABHQJ0Ciqs+KUhIBIkAElARIVJWIKAERIAJEQJ8Aiao+K0pJBIgAEVASUIrq2bNn2eHDh9n+\/fvZvn37WLly5VjZsmVZiRIlWP78+ZUPoAREgAgQgSQRkIoqhPTLL7\/kR2EsX76cHT16NA+T4sWLszZt2vAjHqpXr87y5cuXJG5UVyJABIiAlEAeUd2xYwcbOXIkW7lyJatVqxZr0qQJq127NitZsiSrWLEi2717Nzt06BD78MMP+cmEX3zxBWvatCnr27cviSt1MiJABBJPICWqZ86c4SPTmTNnsgceeICPQjHNd7vOnTvH9u7dy+bPn8\/v\/f3vf8+6du2aeKgEgAgQgeQSSIkqRp8rVqxgLVq0YJjae71gIli2bFmijs31yojSEwEikP0ElAtV2Y+AakgEiAARMEeARNUcS8qJCBABIsCkoioOddu8ebMSUenSpdltt93G7am\/+MUvlOkpAREgAkQgmwlIRfXkyZPs448\/Zjk5OWzTpk2sVatW7Prrr+ccNm7cyP75z3+ywoULs2bNmnF3K3y+8MIL2bRp07gHAF1EgAgQgaQSkIoqVvXnzp3LFi1axCZMmMBdqawXNgE89thjrG3btuzuu+9mR44cYYMGDWKXXHIJ\/5suIkAEiEBSCUhFFSLZq1cv1rJlS8fVfBy+tXTpUjZ16lQ+SsVnuFbNmzcvqSyp3kSACBABd5tqp06d2F133SXFhFHs7Nmz2YwZM1ipUqXYqlWr2JQpU\/joli4iQASIQFIJSEeqp06dYoMHD2bffPMNmzx5Mt\/vb5\/+9+nTh1WpUoUNHz6cnX\/++XzEumXLFi6sdBEBIkAEkkrA0aUKAtmjRw+GOAD33HMP37KK69\/\/\/jef4p8+fZr99a9\/5QtTf\/nLX\/giFQQWsQDoIgJEgAgklYCrnyriAIwbN47vtIKI4ipQoADf6z9gwADuQgX765AhQ9ivf\/1r1rFjR1aoUKGksqR6EwEiQATkNlU7F5gDjh07xr8uVqwYK1iwIKEjAkSACBABCQHXkSqm\/l999RVbu3YtO++889i9997LBRW+qzAHFC1alKASASJABIiAhYCjqGJkCh9VRJ\/CddVVV\/GVfsRNffjhh7nIyhaxiC4RIAJEIMkEHEUVfqdYyR87diznA9sqRBVxVTds2MD69+\/Pfvvb33J\/VgpQneQuRHUnAkTASkAqqhilPvHEE3zxCaL5\/vvvs2eeeSblk4oMEHd19erVKed\/wkoEiAARIALM3fkfwnrTTTex9957L4+ovvPOO2zixIm5hJaAEgEiQASSTsB1m+ott9zCunTpkkdUERsAzv6ffvopF1Z4BNBFBIgAESACDiNViCYc+7GXH079CAUopv\/Y5\/\/mm29yP1WE+8MfsqlSVyICRIAI\/EzAdfUfTv2vv\/46jz713XffsV\/96lf8mOqDBw\/yDQBPP\/208hwrAk0EiAARSBIBVz9VxFXFbiqMWOGbeuLECVanTh0e8g9nWZnwU4WXwcCBA1PMu3fvzp588slcbWAPmi1LA7vvfffdl7rvlVdeScWATVKDUl2JABHILIGMHqci3LZmzZrFqlWrxs0MOI0VAbGFsIrv2rdvz+MK2D8DHwS1X79+TORj\/5xZxPR0IkAEkkQgJapwo0Jg6sOHD2vXv0SJEnyXlZ+Fqh9\/\/JHbZRs0aJArCItdECG8c+bMyeVlYPVGKFKkCM+nQoUKuUa4sAHjso96tStHCYkAESACPgikRNXLuVTiOWKXFeKpmrqwLRYbC7DpAKNXmTiKskIwESULo1v8Wxz5Ikavdt9aU2WkfIgAESACTgQcp\/+HDh1io0aNYldccQUP\/Ve8eHGeB0aYixcv5rutxowZwxo2bGiUrnVkKkah9tGs1QRw9dVX5xJhURgyARhtFsqMCBABTQJSUT1z5gwfIcJ9SrYNVfipfv3111xYIX4mLrtN1clEQKJqgjblQQSIQBgEXI+oFjuqZA+GVwB8WMVxKkELJwQUMVxFniSqQanS\/USACKSbgOve\/8svv5w9+uijPDC19cJIFqaBXbt2GdlRJRNUYWqQLWaZGqledtll6eZNzyMCRCBkAlu3bg35Ce7ZO9pUFy5cyI+bRpg\/uDJhAwAubAKA3fO5557joQHhrxrkchJUkWeYC1UQ1Uw3QBB2UbiXGJppBeKYPRwdRRWjUcRSxSq8OEpFVBsH\/eFgQIitfRTrBY1KUJFXmC5VWIR76623WP78+fMUW\/yIeKmPPe22bdtY1apVfWWhe68qndP\/e\/nentb6WfV8X5W33RTkGbr3qtJ54YXiy9ITRzkXJ15+OEbhx0np\/A8vgI0bN\/ID\/3Ah4v8111yT8gYI8tJgFIpVeje7rH3xysn5H7upxC4q3ZX\/xo0baxdfiCz+xtlczZo1Y6VLl07db0KEtQsToYQqMYpQUSNdFOJopnliIapmqpo3F\/ijdu7cme3Zs0f6COs207C2qXoRVa8chMhWrFiR3X777Tw2La5sE18SA689Q56eOJrhGClRhXBh8emBBx5gtWvX9hR5Ci5Wn3zyCd\/5NHr0aDN00pCLvQFgL7Ze6OiIG\/vtt9\/yr\/H\/9jRBimkd\/cIXF7Fr4ya8JAZBesD\/3UsczXCMlKhCGHHA39ChQxkK9rvf\/Y7Vr1\/f1QcVW1vXrVvHp++IXoV7GzVqZIZOGnIx1QBCaMFw5cqV\/LgZXPCOOHDgQKCaQHjxB\/bfO+64I4+5QfUyki1Qz65NHN27qYqPuFuVLuz+aOqdDvLS5rGpHj9+nAcmmT59Ot89Bbcquw0Vof\/Wr1\/PjfGwK3br1o117NjRSNSqIJXxem\/YDSA6kHV0i39DJJctW8aDfAcZ\/SIfLChee+21PEIXDmO0mxfC7sSql8hrm8jSB3mG7r2qdMQxHj9OYb\/TOv3ZcaEKggrhxOo7Fn6OHj2ayg9CWq9ePR5M5YYbbjC2o0qnwCbTRKEBRH3sZgWYY\/DDhgthF\/1cYpSLQOIIfhOGaUElRn7KncR7iKOZVo\/CO61c\/RdVPXv2LPvhhx\/4aDRbIv1HoQG8diUhvu+++y7717\/+FVhwH3roIT7b8LuARmLgtQXl6YmjGY5ReKe1RdVMlcPLxU+Q6rAbIMiLonuvzMSwc+dO7l7m17QgBBZmnw4dOnDbcOXKlVPCay2bbjmDtHyQZ+jeq0pH03+a\/uv24awQVb9BqsMWVd1GSFc6mBG+\/\/579tprrwU2KyAD2HKvu+46vrAZhmkhXVyi8ByVqEehjHEoQxTe6diLqtiV5SdIdRQaICodFaNaeC\/Mnj2bj3D92nFFfazuYohza\/UK8WtqiAqrMMpBomqGahTe6diLqjVgtdcg1VFoADNdKZxc7ItnsKnPmzfPiOhaS2wV4PLly\/PTIHDIpF2gw6llNHIlUTXTDlF4p2MvqvaTAkTT6GxVDbsBgrwouveq0qXLFigEeMuWLdxrRNhzTW6WsL921hGvVZjLlSvH3QCxACfswSqBjgpHM9Iiz0VVR7dn696rShd2fwz7ndZpH6WowsEfwgV\/SOy0gi8kHNrRcWWBSHQeajINiar7AkLYnVj1EqGtZcKKID3YKIGdeCJNmALstc\/ZTRTo\/5UqVcqVDbxgLrroIn5GG\/5AxMuWLcsuvvhihpgZVapUyZUeC4hY8BOX9bP4d5imEZ22cuKke68qXdj9MdKiik4EH9Xhw4fzKFXiPCpEpcJpALBh\/vGPf\/R16J\/XDu6WPsqiarKeUc1L9RKZKLdVbPGDvmbNGoYfe\/G9\/W8Tz6Q88hIwLfi6+eHHC2YhRMeDSyf+jR2G+Dcuaz6RFtVVq1bxANWPP\/44rwTip2I7Ko5YWbRoERs5ciTr27cv69SpU0b7X1BRReER\/i+MC1NP++hG9zm696rSOf2\/l+\/taa2fVc\/Xra9buiDPsN4r2zIM\/2ucIIwR86lTp\/guQns6fMYgQ4S5DLr12AQTyiMvAbTR7t27Mx4jWTr9FyvqmOLjlNLz1tcLAAANAklEQVSPPvqIn1klQvSJM6qwzXLixIkZHa1GeaEqyChO915VurCnW6rnm3j5gzxD915VOh2OYsSMkZMsvf078Rn32U0DutzwLuLPTz\/9xIUfPwz4g+3mR44c4T8SJ06c4N8hPgfMFEiLezACtO6UFM+UmWGQR+HChZXFkqWz5mf9cbJmJvverzno7bffVpYzzASuZ1T17t2b3XrrrXybqv24Z4xkcaKqqTOq\/FaSXKr8kjNzn0qMzDwl+3Mhjt7b2Cq6COwEPcIVSVGFkR3HqLRp04a1b99eKqozZ85kq1evZlOnTuUmgUxeYjeV1yDVUbC\/ZJKbiWeTGJig6BwR30zuychFxEeOpKiKI6oxvZ88eTK3UVhHqp9\/\/jlfrMIoFuaBIEeqmGrupG9TdeKoM2213utl2or70iGqQZ6he68qHXGM\/jZVbKnG6DWSooqXBfaXPn36cCM+HLE\/+OADduedd7L\/\/ve\/PHAz3EVycnJ8n8FkSkyD5BP2SFX1orqVXfdeVToSg+iLAf04eTu7ysk2jYHfG2+8EV1RRUPv37+fTZs2jS1YsCBl0IZbw1133cVgb8VRIXG+whbVOLPRLbtK1HXzSXo64hi8B8BD6dVXX422qIpqYqUQq4RYNURczig4\/QdvAsYDgdAR1cFIkhgE4yfuJo7BOcZipArbKkar2CUCu+l\/\/vMfPmotWLAgH61Wr149OIkM5hC2qAZ5UXTvVaWj6T9N\/00It6qf6T4jzP74wgsvsBdffDG6I1VU\/oknnuCCCl9UODzDI+Drr7\/m\/IoXL85Qibp162ZQFoM9mkS1ah6AtFAl71NhigHZVM3YVCMtqmL1H76oY8aM4UenvPTSS+zZZ5\/lfqlYpBo4cCDfJoZtrBi5xvEKW1TjyMRrmXVHMF7zTVp64hi8xSMtqtiJAZepFi1acD9V4WCPakNkixQpwreqIvZmUOd\/xBeAQIure\/fu3E3LeoldU5s3b+Zfy9JE0aUqeDeJfg4kBmbaiDgG5xhpURUiJnZUYQtd586d+YmdXbp04bVfunQpe\/755wOJKgQVuyBwemu1atWYeC7iogphFd9B3Nu1a5dKIz6jLFGN\/B\/kRdG9V5WOpq1kU9W1d7rJmqqf6T4jzP4IdyosVkXST1WMTOGfihHr66+\/zp566ik+MoUNFeaBUaNG8UUsjFxFtBgvvzXiGQhIDLEUl10gIbxz5szJJd7WbbMYNQ8YMIBHzbKOcAEXl33Uay1j2NN\/3Y4o46Z7rypdmJ2YbIFmbIHE0QzHSIsqGnnhwoVs0KBBPP7j3r17+VHU48eP5+\/\/2LFjudCNGDEilyB6EVWntPaoUzJxtAZRgQdC165duXhS5H8TLeAtD5Woe8stuamJY\/C2j7yoYjS6fPlyfipn\/fr1eYg\/eAJghDl48GBWo0YNdv\/997NChQoFp2HJwToyFaNQ+2jWahK4+uqrWf\/+\/bnQw4TgNOKVFTLskapRMBHNjMTATMMQx+AcIy+qwavoPQe7TdXJRBAXUQ3youjeq0oX9vQ\/HT9MqjrG3RaI8hNHue3bq4tfzZo1uT99Ji+t41QQi9F+YSSL84jq1KljJJ6qENAdO3ak7KfpENVMwqdnEwEiYJ5ApndJOooqAqpgcQq+qk6XOGKlVKlSgcjIBBUZhi2qgQpNNxMBIkAEJASkoiqc\/7HaD1cqiCeCFeCEStgtsYiFwCqjR4\/mq+6IIO522X1RW7dunfJ3dRJUkV+YC1XUI4gAESACpglIRVU4\/0NE4VKFCwKKk1Sxyo7gKjifqkmTJoFW\/1WCiueG6VJlGiblRwSIABHQOk5FiBuOLBA7quBShfPdxWc\/KDEKhc+p264s++KVfTMAnus38r+fMtM9RIAIEAE3Aq4j1ZYtW6ZGohCuCRMmcDNAyZIlua01yBlV8EeFaWHPnj3S8omjUfCfYW1Tpa5BBIgAETBNwNWmiu2pmPZDRCGCOLIan2vXrs3Pplq7dm1KZE0XjPIjAkSACMSRgOPqP9ylevTowR3+Ef0fx9NiOyhMAGXKlOHHqiCwCYQ2CmdUxRE+lZkIEIHsI+Dqp7pr1y6Gw\/+wIIXwfjhUC3ZQTP1x0ioCrmAUSxcRIAJEgAj8TEDp\/J9kULDlwssB\/rrWLbBJZuKl7nZbOExH1uA5XvJKclrhJbNkyRKOwbrekGQuQequE3DJb\/6uorp7924+3ceIVXbhvKp7773XyI4qvxUI6z6xkIb8RWjCsJ6Vrfmi4yKgOYRU8ERQHmvgm2ytu8l6wa3wm2++4e6M1ghtQTfdmCxjnPIS3kKyuMwm6uEoqu+++y7r2bNn6hRV2cNM7agyURFZHniREaIQXgv2DugW1BojrOnTp7PmzZuzoUOH5gnWElZ5o5qvX47W+ojRFs5mT6qomuCIfosTQ4O4Mka1n+mWKwhHvNvDhg3jM8\/jx4+7hgbVLY89nVRUxQsAdyeMNqpWrarcNeW3AGHdJ6aeyN\/uB6sb1NoehjCsskY5XxMcUT+3FyHK9TdVtqAcrSaAJE\/\/g3KEnjVq1Iht3749Nfo31cYiH1fnf4T7w6mpcbuso1D7aFp0Tp2g1kkXVVMck26bNsUR76E1lnDSRvxBOeL+NWvW8NGp1aRiWt+konrs2DF+kmrQbaimC6uTnwCPRRFc9lMDnDqlzFaVZFE1xTHpI1RTHEXfdwoypPNuxDmNCY4w6eXk5OTCEIZd1dGmiqAp8+fPZ3\/+859ZuXLlYtkesrgBTkJpNwmIKassAHYsYQQotF+OeCTOYccJEgg4nvTLL8cNGzZwdGLBL+l90i9HqwdPWkaqGJ3OnTuXHT58mDcgYqiuWLGCnThxgt1+++2sePHied6JqK\/+B4Wf5JGqtbH9cMSGERwMKdyARH5Jtgf64QjPk0qVKvGNN+RS9XMv8ssx7aJq9ynUGVVEffXfBHwdDtmehjiaaWHimAyOWe38T504GZ3YTC3Dz4X6oxnGUecoFVVM\/WEOwNbUYsWKmSGRgVxk8L0sVGWgyJF8JHE00yzEMRkcc4kqIv4j2v+kSZNSTv9NmzZlQ4YMYRUrVjRDJI25yDqxF5eqNBY10o8ijmaahzgmg2MuUV22bBl77LHHWK1atVjDhg3ZwYMH2YIFC1i9evW40CJiVZwuWSdG+SmotbdWJI7eeDmlJo7J4JgS1ZMnT\/LAITgqBds6xWo\/IlJhuypWIePmbOzUia3CKpo5yavSqq5OHFWE9P6fOOpxUqWKOseUqNqPLREV+\/bbb1nXrl3Zgw8+SBGGVK1N\/08EiEDiCShF1UlsE0+OABABIkAEJARIVKlbEAEiQAQMEiBRNQiTsiICRIAIkKhSHyACRIAIGCSQR1Q3b96snX3Ut6lqV4QSEgEiQAQMEUiJqj2gik7+UQ+oolMHSkMEiAARMEkgq\/f+mwRFeREBIkAEdAiQqOpQojREgAgQAU0CJKqaoChZ5gmcO3eOYSv1xIkT2bZt21iLFi34GWpFixZl2KTyhz\/8gcepwNW5c2fWu3dv6YYVEf8B6ZJ8gF7mWzQ7S0Cimp3tmpW12r9\/P3vooYdYmTJlWMeOHVnp0qV5nIoCBQrws4defvllLrgQWBLVrOwCsagUiWosmokKCQI4iUEmlhjBjhs3jl1wwQWsV69ejukERRqpUn8KkwCJaph0KW9jBBBEY+DAgbnyE0Fwjhw5wsX0kUce4UF\/nMTXSVQXL16cJ2+RFqfuIpiQ9SgOY5WijLKSAIlqVjZr9lVq586dbN26dWz8+PH82PSbb76Z1ahRg5sCPvvsMzZixAh+SGX58uU9i+qBAwf4OfDWC6YG2GtvvPFGNmzYsFgHa8++3hDtGpGoRrt9qHQWAk4jUIxiN27cyIYPH85PqxDp9uzZ48qvdevW0oUq+GxjwWv37t1s8uTJsT1NmDpPZgiQqGaGOz3VBwGZqOLon8GDB7O6deumVvpFuiuuuILVrFkzz5NOnz7NVq5cyfD\/9tV\/nH4xZcoUNmfOHDZjxgx25ZVX+igp3ZJkAiSqSW79mNVdJqowC\/Tr14+NHDkyZff0alMtUqQIJ4EFr\/nz53ORxqi3bdu2LF++fDGjRMXNNAES1Uy3AD1fm4BMLHEyBUaVcKUSh1T6FdUtW7awHj16sFatWnEfV7hq0UUEvBIgUfVKjNJnjIBdLO2uVKJgfkR13759rE+fPvwcttGjR9PCVMZaOf4PJlGNfxsmpgZ2sbS7UvkV1Z9++okvTG3atIlNmzaNVa9ePTFMqaLmCZCommdKOYZEwC6qdlcqP6IKW+zMmTP5acE44PK6667LU\/pLL72UVa5cOaRaUbbZRoBENdtaNIvrYxdVuyuVH1EVi1JLlixxJAdzQLt27bKYLFXNJAESVZM0KS8iQAQST4BENfFdgAAQASJgkgCJqkmalBcRIAKJJ0CimvguQACIABEwSeD\/ASCAckz5oC+ZAAAAAElFTkSuQmCC","height":205,"width":341}}
%---
%[output:7b890019]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:7392f24e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:55fe855d]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:577850ee]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:88965eb4]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"  -9.869604401089360"}}
%---
%[output:8fc1ceb0]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.998429203673205"}}
%---
%[output:4d8cbfe6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.149016195397013"],["36.521945502464568"]]}}
%---
%[output:8272fe3b]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:4b5e32e2]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:41d8c6b8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:00130ac3]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.155508836352695"],["27.166086113998457"]]}}
%---
%[output:25f30ff8]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.037191061070411"],["1.937144673860303"]]}}
%---
%[output:0bf428b5]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVUAAADNCAYAAAAITuO0AAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ10V8WVv7aclmiXaoBi2QBBGyxVFz+qjVQ3Wk5L221wd\/sB4ew2BzguWxF7tuFAEntOl1MgQKXKh22ppjn0I8G2Cyu020Ua2q4Vqa0oWsViCTHGQAtBBCv2Q9lzn86\/k8n7mHlv5r2ZyX3n9BTzn7kz93fv\/N6dr\/vOOnPmzBmghxAgBAgBQkALAmcRqWrBkYQQAoQAIRAgQKRKjkAIEAKEgEYEiFQ1gkmiCAFCgBAgUnXEB5555hmYM2cO1NbWwpIlS7T3+vjx4zBv3jwYP348rFy5EsrKyrS3wQtctWoVbN++Hdra2qCqqspoW6Lw06dPQ2NjI4wdO9YIliaV2bNnD8yePTtoYv78+Ur9LxJzERPmb7NmzYKZM2eahCx32USquUOersEiSBXb3LFjB9xyyy3pOh1TSxzgJtsSu8GIqb29HaqrqxN1U+3bvffeCxMmTJCSndg4V4C9DHp6eqC1tRXKy8tVqoNNpIodx\/6gLdLooqR4zoWJVHMG3JXmTJM4P8ARE5NRuIi5ymBWxQEJtampCWQJW8UffCNV1ZebClZFliVSfeONuXHjxsAOOCXkp6Rs8F9\/\/fXBQMGnpaVlwJQFy7D6OD1n02c2ILHuqVOngumuKF80PhuU7O9scIqDm5XDKSC2zaaCrFxfX9+AKaI4vccfcQrMoh78bzb9X7x4cRCd7tu3L5AxZcqUAdEEG9z4m6grvzzB4xKF69q1a2HZsmWD2mL9CesDax\/xxIdhwNuFnybzbTMcMEJlyyjsb2JbUX2I+vuBAwdKU3PWL2wjqi9hA1\/sC\/MnZi+mM\/53GHFH1cflHObLo0ePLuHNYybiyuPG\/OrKK68MfAYfjDB5nWtqaoK\/nzhxouQvoj\/yfVZ9YRVJlCptD3lSFadEYqTBiIE5X9jvbG1w5MiRATGxAcucBp0YHbC\/vz82IouTLUZzPKkychCdlA1m7PsHP\/jBAWumcaSKRNnb26vUV+zP+vXrSy8kGVwZbqJuImmLfeFxQsLHlwPKYjbi9cb1Oj4yZTZYuHBh6cUY9jt7OYiYqvQN\/SCuL+L0PenFh8TIvwiT6ou4ib4s2ojHgX\/J8v7AfBnbFvuLLyVc72UvYdHfRR\/Jex1fhRizlB3SpBoXtTBiDFv7Y1PVm266adDmTtwAjXOipKldVKTKv\/njpp5JAzZqEEVtjPH9ufXWW4PBziJX1IV\/ueDfRaxlpv9i1IURKWuLX1cMIy5+E4yfZmJfcODzERofUYvRX1Q0FdY3fLnFvRhxQy5uyhv2G\/839gKJWlMVcRCJIelFh+XFaJVFymEvWbE90Yd37tw5YCmEYcleaEk+n4XYiqw7pEk1bMCI5LNu3boBu9T87+I0mRmSTZvECCyOVJPe2jKkGrcRoZtUUTf2AkEyWbRoEbDBooprVKTKos8rrriiFDWHvcjCSJUtx\/CDC4kUN5BEUhWnqFiHkW5UpBrWtyhSjeqLuOsd9lLkdZsxY0ZspJq0nptEquzlgi8vEecwUhXbiyJVkeDYUhWRapHUb6htE5Eq31VxQPoUqaKebMBfd911cPDgwdLUXxVXkVRF3MKiYpVIlbdJUjTHiCLqxRjXN5lINc6Vi4xUJ02aNGDWxWYb7IidjkhV1J1I1RCxFS1W5e2dNGjYmmqUIyZFo+Kbn1+DilpTjVv456dbYpTD1rvYGpk4\/Q+bwou24qfA4plJGVyT1qJxUwTX83C2wG\/GpVlTFddv46agYWuL4jp5VN9EYkxamuAxTZpNqK6pijaMswkjVewPrv+zqXvc9D\/Nmip\/MiJpPBTNDWnbH9LTfwaazC41rhF+8YtfDKrE7f7zO+UqkSrri+ruf9QaoLj7z0eW+G+cAuOJhLDdf7ajz3CJO7HAyoTtRMvgyk5aiG3t3bs3WI\/Dh+0qjxgxIiBZfNjmFPaN2SZq9x\/Ls\/6FRdFxu95YV6VvjMhw04YREtvAYTaOO24Vt3svE9nJ7P4zzMWXOH9KAf0YgwPmH1GbrHwd0adwM0tcWuFtRLv\/aWmbq4fOiU\/YjSARfPEYj4bmU4mIW6dMJZAqaUdA9bwjH4mqHqDX3nmPBIYdtYtTT9VurkCVW6TKAIy6Woe\/d3R05HJFUsU4RKoqaOVTNuyYG3+cK6kX6Gu4sVbEFdmkvrn0u3hkEPsunvqI08fXl1supIpTkqVLlwb4Rt23xqlId3e30l3mPByQSDUPlNXaEKe4\/PReRpLLd\/9l9MuzjLhcxV9+iesHsyHd\/U9pLSSmysrKgDSjpv\/8+pvqIEnZLapGCBAChIB2BIxHqjhF2LRpE9x2223BLm4YqbLIYerUqcEtF5qeabczCSQECIGcEDBKqkiWy5cvh\/r6+iC9W9xGFa+vSLL8bxdccEFO0FAzhAAh4BoCnZ2dMHHixEK7bZRUw26qoLZJeSAZqdbV1Q1Kn4ak2tXVVShoOhv3TR\/EhnTS6SHmZJGdzGBrlFTFLkdFqrho3dDQAM3NzUFEi9N\/LBuWZ9E3R\/BNHyJVMwPVhFTyPROo5vyNKp5URSLlo9q4w+a+OcKhQ4cKn67odi3SSTeiZuT5aCcb+CHXSFWHa9gAmg49mAwfHZt00ukh5mT5aCcb+IFI1ZzPSkn20bFJJynTF17IRzsRqaZwKxtAS9HtyCo+OjbppNNDzMny0U428ANFquZ8Vkqyj45NOkmZvvBCPtqJSDWFW9kAWopuU6SqE7QCZPlIQD7qZAM\/UKRawADlm\/TRsUmngp1Ksnkf7WQ1qYYlrUiyFabr27p1a1KxTL\/bAFomBYTKPjo26aTTQ8zJ8tFONvBDZKQqniNNMi2eM12xYkWQTs3kYwNoOvXz0bFJJ50eYk6Wj3aygR9o+m\/OZ6Uk++jYpJOU6Qsv5KOdrCZVW78fYwNoOkeDj45NOun0EHOyfLSTDfwQG6mKCWjFbzOZM3e0ZBtA06m3j45NOun0EHOyfLSTDfwgPf3nk0gX+f0oG0DT6eY+OjbppNNDzMny0U428IM0qTLTiqcCktL46XYJG0DTqZOPjk066fQQc7J8tJMN\/KBMqryJ2Y7\/mjVrIK+vUtoAmk4399GxSSedHmJOlo92soEflEk17rvi5sz\/V8k2gKZTTx8dm3TS6SHmZPloJxv4QYpUxQz+cflOzbnA65JtAE2njj46Numk00PMyfLRTjbwQ+zhf\/yG9759+0pWbW9vH\/R5E3MmD5dsA2g6dfbRsUknnR5iTpaPdrKBHxJJtbq6GpYsWWLOsoqSbQBNscuxxX10bNJJp4eYk+WjnWzgB7qmas5npST76Nikk5TpCy\/ko52sJ1Vx+p\/kBZRQJQmhwb\/76Nikk7ofFFHDRztZTapFGFmmTRtAk+mnbBkfHZt0krV+seV8tJMN\/CC1+1+s6Qe2bgNoOvHw0bFJJ50eYk6Wb3b6+W9PwCcaN8CR73\/eHGgSkolUJUAyWcQ3x0asSCeTHqNPtm926vjlEVjQsR+Of\/kGfSClkESkmgI0nVV8c2wiVZ3eYVaWb75HpJrSX2j6nxK4HKv5NljpRZGj82Roikg1JXhEqimBy7EakWqOYGdoyjc7rdrRDat2HKLpv6pPEKmqIpZ\/ed8GK0Wq+ftQmhadI9XTp09DY2MjbN++HWpra2Hu3Llw5513Qp4ZqhBoItU07pZvHSLVfPFO25pvdnKKVBmhTp06FSZMmAAdHR2wcuVK2LZtG+zevTv4d1lZWVrbDqqHCbHxCbseS6SqDWZjgnwbrBSpGnMVrYKdIlX+y6r9\/f0lUu3t7Q2+oKozWt2zZw\/Mnj0bopJfE6lq9UMjwohUjcCqXahvdsLjVLhZ5cyRKowe+\/r64MYbb4T77rsvmP4vWLAgWArQlXAFyXvp0qWB82B6QYpUtY+jXAT6NlgpUs3FbTI3suj7B+Abu593h1RRYxZFMu11fwgQibuyshK6u7tp+p\/ZxYoTQKRaHPYqLftmpxl3PQo\/P3jCLVJVMZhqWUyEvWnTJrjttttg3bp1RKqqAFpU3rfBSpGqRc4V0xUiVQ4c3Ahbvnw51NfXQ1VVFSRtVGHVzs5ONyyd0Etcl66oqPBCF6YE6eSGOX2y0w21M+Hkh17f4HZiTVX8LlWYy2T5xIr4uRYmP2yzijaq7B+wFKnabyPfom9MpjLjK4+6Q6rYU7beOXPmzJLH3HvvvcH6J24o4b\/xeNXatWsze1RSpNrV1ZW5DVsEEAHZYon4fpCd7LYTu6LqVKTa0NAAzc3NwfScPfwnqvGoFR6vamtry4w+kWpmCAsVQARUKPzSjftkp8uWPQQ9x1+BN718DI597ZPSGJgoKJ2lColu48aNwD7+J54n1RmpxilK038TbqBXpk+DlSFDOun1EZ3S+Kn\/8AP\/A30\/+JJO8cqypEkVJfNrn\/waKh+xlpeXK3dCpQKRqgpaxZQlAioGd9VWfbATRqcYpeIzvnw4nNj8Wej+9cOqUGgtr0SqWltOKYxINSVwOVbzYbCKcJFOOTqQZFNIqLg5hf+Pz111k+G2mVOh6D0XIlVJA5oqRoPVFLJ65ZKd9OKZVZpIqPXXjIU7PnmRFQmXpElVvE3FQMEvqLa2toLpaT9rjyLVrO5ovj4RkHmMdbTgop2QTB\/qOgGfad9fgqDuqvODKBUfG\/hBilTZOdWFCxfCrl27gkP6eGAdUwFi5ir+mJUOY8fJsAE0nTq66NhJ+pNOSQjZ8btLdkIy\/c3v\/gAz7358AHgPLXkfXDTm7NLfbOAHaVJlR6q2bNkS3M9HIs1zg4oiVTsGokwvXBqsMvpgGdJJFim95ZBMv3R\/N3zn4cMDBOOm1GOfv2ZQY86QKsunijv+NTU1QWq+e+65J8hW1dPTQ9P\/DH5EgzUDeDlWJTvlCDYArNvVA\/\/5g4ODGkUy3Xbz5cFOf9jjDKmyzm\/YsAGmT58OeNAfiRXT\/ulOUJ1kOhtAS+qjyu80WFXQKq4s2cks9hiR4vel8GZU2BMVmYplbeAHqem\/WTjVpNsAmlqP40vTYNWJpjlZZCe92CKJrr6\/G9qFaT3fChLpd2+aApO4NdOkXtjAD0SqSVYy\/DsNVsMAaxJPdkoPJNtkWr+rJ8h3GvckTe+TeuEMqfKfU4m6+09HqpLMHf47DdZ0uOVdi+wkhzgS6Ld\/cRj2dJ1IJFCUiCTaOH0iTL3w3Mh1UrmWXy9lPalGpeTjlcx7XdUG0FSMnFSWBmsSQnb8TnYaaAckzx89eQx++PhRKfJkBDr+vOGwftZkmDAyfKMpq7Vt4Aep6X9UpJoVgDT1bQAtTb+j6tBg1YmmOVlD0U5InH9+9TVYu6sHuo+dliZPnkBXfXwSnPOWN2uJQmWsawM\/SJGqjDJ5lbEBNJ26DsXBqhO\/vGT5aKcH9z0D48aNgyMn\/wibHuqD546\/okScPHlWX3Au\/Mv73pkbeUbZ3QZ+iCRVmWz\/qBhdU802rH0crKRTNp\/QWRujzVdfOwN3dD4bRJs9L7xSSkCi0g6ufeLUfcEN42Hy+ecUTp5OkqoK4HmWteFNpFNfIiCdaJqTZaOdWHamx3tPwf37+5Wn6CJajDg\/fc1YuLry7cHPUYfszSGdTbIN\/EDT\/2w2zFzbxsGaVSnSKSuCUIomT7z8l+As51OHX0odZbLeMNIcP7IMPvuB8fD7w8\/D+6f89Use2XtdvATnSBWz+zc1NZWQa2lpyTWZCjZsA2g6XYcISCea5mTptBOLMPccehEeeOYFeLY\/\/bSc15iR5gWjz4ZbbxgHw978pthIU6dO5pBXk2wDP0hHqvg5FUz\/x9L8sTXX6urq4MN\/eT02gKZTVx8deyjqxIgSfePQsdPwvb2\/gx5NZMlPw99\/4bnw0UtGw6V\/+7bM03Mf7WQDP0iRKh3+10mjA2X56Ni+6YSE+dxzz8Fzf\/6bILLEXfK0Gz5hnsQizHHlw+EfLh0Nl4zNTpgyHuubnWyZyUqRKnaWIlUZN1Uv46Nju6ATiywP9Z+GB397IrgBhI9usgyizPOGw\/UXlcMnrhiTObpU97DoGi7YSVVfZyJVphitqaqaOLm8j45dlE74VU2M+l7+06vwzT2H4dfPn9JOlPxUHMny7yedB5+68vySoV3aLS\/KTsmjIn0J50g1var6atoAmj5tKPlxEpYsouw78Uf436eOwd5nTxonStwd\/9ilo+A97\/zrNNxHAvJRJxv4QXr6n+T8ef1uA2g6dfXRseN04jd0\/vux3wefyNC9Rsnsw6JGjChxvfLid74NPvZ3oweYTzayHGp20unjecqygR+IVPO0eEhbLg9WniBRtYe7X4SfHngBfvP8C1A2fLjW9ckoosTd8GvfdZ7x6bfLdopycR91coZU2fGp8ePH557pX3QIG0DTycO2OTZPlA8ePBF8uRKvN+KjcxOHx5CPKKvGnA0fv3wMVJz3ehYj\/A37JBtR6rQNL8s2O+nQ00edbOAH6Ug1LA0gHf7P7tqmHTvP6TZDY+yIYTBs2LBg1xun3ZePGwEfes9I49FkdmtESzBtJ5N9p0g1X3SlSTWsW3gaYPPmzfThvww2Ux2sjCSPvfQnuPdXv4P9h18yGkWyaDH4\/zdI8qoJb4cPvLt8EEmyiFJVpwzw5VaVdMoN6kwNOR+p4tdV29ragP8agIgIH+HGZbQSI+GosjaAlsnqXGV2qBzTr\/3+1J\/gu48cgacP\/yFXkrxw9NlB1vX3TXw9gQZPomn1JAJKi1y+9Xy0kw38IBWpsjVVNDm7pipjfvy09fLly6G+vj4gXoxsd+\/eHboui1dgOzo6EtdsbQBNRncWUX5rTx\/84tCLQZWk7\/PIyBXL8GuNGElWjTkHZr53DJw\/4q3aSFK1Xz4OVtJJ1QuKKW8DP0iRqi54MBpdsWIFrFmzBsRvWiHhdnd3J+YRsAE0hgcS5w+fOAo\/+vUxbZs4\/KYNtnP1xLfDP18+Bt721vyyp2e1NxFQVgTzqe+jnWzgh1xJNS5SxWuwGzduLHlTe3s7YLIW8SkKNCTQO378LBw8+rJyxMkT5aQx58CHLxkJk95xTqDaqy8ehokTJ1qxw61rKPs4WEknXd5hVk5R\/MBrlQup8l8RCCNLXCZobGyEqVOnBqkEcSlg0aJFoeu1CBo+nZ2dRq3Td\/IvcOqPr8GaB47DI8+\/EtsW7nbjUz2+DD59xQh481lnAftbUid7e3uhoqIiqZhTv5NObpjLJztNmzatBHpXV1ehBsiFVJmGjFwxVWBYFMrKiSTLI2T6TYQR6S0d+yOjUYw6P3zxKLi5ZpyWs5MUARXq\/9KNk52koSq0oGl+kFFOilR1pf6LI0u+s6xcXV3dIPI1BRqS6YyvPDro+z1IorPeez7MvtrMR81osMq4afFlyE7F20CmB6b4QaZtViaWVMMO\/IvCa2trI3fsRTJGeYsXL4bVq1cPOIYllsPpP66xhp00MAHajLseHRCZIpFOe\/fI4JMTpm\/y0GBVcdfiypKdisNepWUT\/KDSPpbNFKnKNCYSM1tTDSPcOXPmQF9fH8Sdf9UJGkanly17qKQGEmhb\/cXBDaC8HhqseSGdrR2yUzb88qqtkx\/S9lmKVNMKN1FPF2j3P9UPs+55vNTFay88FzbUTTYemYqY0GA14SX6ZZKd9GNqQqIufsjSt0hSZZtKR48ehdtvvz2Yju\/bt29QW3G3pLJ0LKpuVtAwOu345RFYteNQ0ARGpxtmTYZr33Wuie4myqTBmgiRFQXITlaYIbETWfkhsQGJAkMuUv36A73QuPWZEqFuu\/ny3KNT3i40WCW81IIiZCcLjCDRBSJVCZDEIllAwwh1Qcd+awgVO0KDNYUTFFCF7FQA6CmazMIPKZoLrSIVqfKH90Uprkz\/8ftFeGSKTfmLjlAZjjRYdbmyWTlkJ7P46pLuDKlGKYzrrDU1NbEH+XWBxeSkAU08g4qEWtQaqogHDVbdHmJGHtnJDK66pabhB919kIpUoxqNS5Ciu6NZSPWWzU9D+8OHAxHfmXcpfOTiUaa6pyyXBqsyZIVUIDsVArtyo86TqgtJqvmzqLjT\/9jnr1E2lMkKNFhNoqtPNtlJH5YmJTlDqnFrqlHZpEwBpwKaeI8fCdX0DSlVvWmwqiJWTHmyUzG4q7aqwg+qsmXLZ5r+yzais5wKaPzmVN1V58NddZN1dkWLLBqsWmA0LoTsZBxiLQ2o8IOWBkOESJOqeK00Ljeqqc6iXBXQ2J1+jE5t2e0XsaHBatJb9MkmO+nD0qQkFX4w1Q8pUo3KLiWbrV9n52VB49dS519XAS3\/VKWzG9pk0WDVBqVRQWQno\/BqEy7LD9oaTBup6kr9p0MRWdD4zFM2rqUyLGiw6vAK8zLITuYx1tGCLD\/oaCtKhlKkKuY3jUvRZ6rTMqDxUSomStm24HJT3ckslwZrZghzEUB2ygXmzI3I8EPmRhIESJEqysCpflNTE7DdfiTU2bNnQ0tLS\/AJlLweGdCW\/6gL1ux8NuiSzVEq9o8Ga16ek60dslM2\/PKqLcMPpvsiTarYkajcqKY7yctPAs32c6kiVjRY8\/Se9G2RndJjl2fNJH7Ioy9KpJpHh5LaSAKNP0a1ZPpEWDK9Mklkob\/TYC0UfunGyU7SUBVaMIkf8uicFKlGbVTl0UGxjSTQ+A0qm+74R2FFg7UIL1Jvk+ykjlkRNZL4IY8+SZEqdgSTp1RWVua6fhoGQBxork39UT8arHm4efY2yE7ZMcxDgjOk6krqP9em\/kSqeQwzPW0QqerB0bQUZ0jVNBAq8uNA+9TXH4cfP90fiHNh6k+kqmL5YssSqRaLv2zrRKqySHHl4kAr\/9xPgpI2ZqOiNdUUxraoCpGqRcaI6YrVpOrah\/\/4qf8dn7wI6q8Z64QX0GB1wky09u2GmZRyg5hSSXqjylQHVOVGvYlW7egufSHVlak\/Tf9VrV9ceXr5FYe9SstWR6qiIrZnqeIzUtmWiDrOKWiwqgyZ4sqSnYrDXqVlZ0jV9ixVLt31Fx2EBqvKkCmuLNmpOOxVWnaGVG3PUsWvp7o09afpv8pwKbYskWqx+Mu27gypskjV1ixVHb88Ags69ge4E6nKup+5ckRA5rDVKdlHOzlDqmhI01mq+GQtU6ZMgdbWVigvLx\/kQ2GgubqeSpGqToowK8tHAvJRJ6dIFV3WVJYqjISXL18O9fX1UFVVFRD47t27YeXKlVBWVjZgtISBxs6n2p47NWzY++jYpJNZgtcl3Uc7OUequoyZJAfJe8WKFbBmzZpB0WocqbqQlYo2qpKsb+fvPhKQjzoRqUaMH5VI1eVNKpr+20mgNKNwxy5iT4lUBUT4xC3sCwNJoOEGFW5U4WN7ln8arO4OVh+jOh91IlKNGGOMXJcsWQLV1dWD1lTxD52dncHf\/23LEXjk+Vdg7IhhsL2+wrlR29vbCxUV7vU7DmjSyQ039MlO06ZNK4He1dVVqAGsvKYaddkAkeLfRC4f+mdW9zFaIJ0KHdPSjftoJ4pU3zC\/eLkAN6oWL14Mq1evDk4D8E8Uqbq4SUVrqtLjv\/CCPhKQjzo5Rars66mid8edKVUZCbLHtXjQ+E0qF9dTiVRVPKTYsj4SkI86OUOqbI1z1qxZVn1O5esP9ELj1meC0ebaTSqa\/hdLkqqt+0hAPurkFKk2NDRAc3PzoOm4qnNmLc+D5vJNKiLVrJ6Qb30fCchHnZwhVXRfPDva3d0NuCNf5MODdtmyhwA3q1y8SUWkWqQXqbftIwH5qJMzpGrjh\/\/4nf+PXjIKvj33UvWRYkENHx2bdLLAsSS64KOdnCFVCfvkVoSB5uKXU8NA8tGxSafchkOmhny0E5FqCpdgoPGfT3F15592\/1M4QEFVfCQgH3VyilTZgfzt27dDbW0tzJ07F+68887QpCcm\/Z6B9q09h+Gz3306aIpI1STi6rJ9HKykk7ofFFHDGVLlbzhNmDABOjo6grR827Zti0zRZwpQBpoPO\/8UqZryEv1yiVT1Y2pCojOkyt946u\/vL5Eq3h2OStFnAjCUyUDzYeefSNWUl+iXS6SqH1MTEp0hVVR+1apV0NfXBzfeeCPcd999wfR\/wYIFwVJAnsesELSf\/uopQFLFx+XjVESqJoaVGZlEqmZw1S3VKVJF5cWrqi0tLbnfsBJJ1dWbVMyZaLDqHlZm5JGdzOCqW6pzpKobgDTyELRv3r8XZnzl0aA6kWoaFM3WIQIyi68u6T7aiUg1hXcgaPO\/ugtW7TgU1HZ555+m\/ykcoKAqPhKQjzoRqaYYIAha3Zfvh6\/+33NEqinwy6OKj4OVdMrDc7K34RSp8udUmerz58\/PdZMK20XQLmn4L\/j5wRMwvnx4EKm6\/NBgdcN6ZCc37OQMqTJCHTt2bIlE2d8Q6rBPSZsyAYI2Yu53nE+kwvChwWrKU\/TKJTvpxdOUNGdIVczMzwCJ+5S0KdAqL7kaTn5oVSC+puo82PqZy0w1lYtcGqy5wJy5EbJTZghzEeAMqSIaeJyK3aQqKysLAMKzq5WVlbkeq0LQTvxja9C+q59Q4b2LBmsuYy1zI2SnzBDmIsAZUo1L\/ccjhZ9W2bp1q1Hwxl\/9EXjp2sVBG3fVTYa6q8432p5p4TRYTSOsRz7ZSQ+OpqU4Q6qmgVCRX3H9v8LLV8wNqrh+RhV1oMGqYv3iypKdisNepWUiVRW03ig79qP\/Aa+8e0bwX66fUSVSTeEABVUhUi0IeMVmnSNV\/KRKU1NTSc0irqm+49Nfg7+Muijow\/Ev36AIuX3FabDaZ5OwHpGd3LCTU6SKm1K4WdXa2grl5eXA1lmrq6tzPavKSNWHM6oUqboxUMlO7tjJGVK16UjVqH\/\/Hrx29ijns1MxN6UIyI0BS3Zyw07OkCrCaUukWv65nwTWdT3lH5GqG4OU7OSWnZwiVYTWhjVVRqo+nFGlaaU7A5YiVTds5Ryp2gArkaoNVojvAxGQ\/Tby9YVOpJrC9xip+nAuLNYjAAAIHElEQVRG1VfHJlJN4dgFVPHRTkSqKRyJSDUFaDlX8XGwkk45O1HK5oYEqYqfYGlvbwc8hiU+mJxlzpw5wXew8MErr+z4Fl+WkaoPB\/8pUk05cgqoRqRaAOgpmvSeVPEo1tKlS+ELX\/hCcLYVCRZPEYSRZVjCljBMGan6cPCfSDXFqCmoCpFqQcArNus9qYp4sAsD+PVVMVrFkwXd3d2JFwmQVH05+E+kqjhiCixOpFog+ApNDzlSxSn+4sWLYfXq1VBVVTUAKoxgN27cWPpb1DIBkqovZ1SJVBVGS8FFiVQLNoBk80OKVNmXAqZOnToo\/6r4Gy4FLFq0CNra2gaRL5LqsGO\/gV8snS4Js93Fent7oaKiwu5OKvaOdFIErKDiPtlp2rRpJRS7uroKQvT1Zs86c+bMGdM9CPscS1ybcQSMpHpzzThYduO7THc7F\/kUAeUCc+ZGyE6ZIcxFwJCIVOMIMgplVqeurm7Q2iuSqi+3qVB\/G5xAt7eTTroRNSOP7GQGV6ORqiyhiglb4k4JIKmevfcb8JaeB80gQlIJAULAaQS8nv6LZ0+ZpXATatKkSdDQ0ADNzc3BuilfFr\/aGrae6rSlqfOEACEwJBAwGqkOCQRJSUKAECAEOASIVMkdCAFCgBDQiACRqkYwSRQhQAgQAkSq5AOEACFACGhEgEhVI5gkihAgBAgBZ0iVPx0wf\/78xBwBtplWpv\/sCNr27duD7tfW1sLKlSuhrKzMNnWC\/sjoxHc87pqyLQrK6sSXi8qo5ppOfEY5F8cYwxuvvFdWVg66uZmXPZwgVT4RCzpwY2MjhF13zQs01XZk+49JZfCZOXMmyJ7xVe2LrvKyOrH2mD6PPPKItcflZHUSEwPJJgPShb2KHFmd+BceXpt2bYzxhIo5RFpaWohU4xwFDb5ixQpYs2ZNkEIQnXj37t1WR3FihJam\/zbrqWoT1OWJJ56AJ598MjShjgpRmCorqxNGdD\/72c+cmC2p6MSn5cR\/44MZ5Vx4+Kvw2F+KVBOsJuZajbtxZaMDpO2\/zY6tohMb2DilRJ3CspTZYDdZnfAFcfToUejs7IR9+\/ZFJlR3SSfZiNYGnZL6QNP\/JITe+IorH5m6RqpixCnTf5kyEtAZK6KiEzp5TU0NjBw5MjL1o7GOKgiW1QnLrV+\/vrSMYfPLT1YnhIkRK74oolJvKsBZWFEiVQnoZSMICVGFFFHtf1zqw0IUCGlUVid+qmz7RpWsTuIaqs0vQFmdRB1sflEkjQEi1SSE3thlTrMmKSE6lyKy61rYGZsHaJp1YjH5OMqwNbeDrJ1kiSoX50poRFYnkURd8cMw9YlUJTzP9fUe2f675MiyOolEHPXlBwk3MF5EVic+qxrbKccXhY2bOrI6hUWq+BFOm4\/0RTkEkarkUJE9PygpLvdiUf3nHSAsqrP5rKqMTi6RKvZVVie+nM02UtEJlzWampoCk9l+9jZuABOp5k5v1CAhQAgQAuYQcOLwvzn1STIhQAgQAnoRIFLViydJIwQIgSGOAJHqEHcAUp8QIAT0IkCkqhdPkkYIEAJDHAEi1SHuAEWoz2fjUsmGZOOBdJbZyeTZW\/6kgcs3nYrwtSLaJFItAvUC21RJ0qJSVkWltOdxbSXVjo4O4+c54z7broI9lTWPAJGqeYytakGFKFXKqigp3kiSrUuk2gh1dXVQXV0tCxmVKwABItUCQDfdpJjsmk2x+STE7MB6b28vzJkzB\/D2DD5xZVHuvHnzgsxM+MRNRfnkHHxZvg9RU+aoz5UjqZ46dSr4HybyFuvzsrFNllOTXdUcMWJEUI\/1m79sgXpj\/dbW1iC9pOwn08UXhNjHuEP04ksi7iVGkarpUaNPPpGqPiytkcQn\/BAHIz9wscOYjJhFP2LCk7CyLDl4XHIUMcG2mCAmbvovJoDmy959990BKba1tUFVVVWQRpBdpcQ2GxoaoLm5OfiNr9ff3x+8OBYuXFhKXMz\/jl9WQBx6enoCUsUHXx547RSjwrj+hpEqJknmiTvquieRqjVDRmtHiFS1wmmHMDE1Hd+ruGhIJD++LEa0fFIblBl1HVAk3DCS5RMi8\/2LIzAVEsK+b968OSBJJFUx50BcApEDBw4Av04aFyWGkSpPonEvHxV9KFK1Y2zJ9IJIVQYlB8vw97j5abJIqvwUGKeq+LAk0nxZnPLPnj17EBJhu\/ciMaqQahzpx5EQi7rZ972uu+46OHnyZCiphn2qhu\/zzp07S3fgeYXDPtERRqpYhyVX4ZOvYATNP0SqDg4siS4TqUqA5HoRfpq8bdu20qdoMPrkI7i46X9YpBqFSxGRKpI+H\/2K0\/8skWqc\/SlSdX106O8\/kap+TAuXGBYBdXd3B9GTOKXHtcbbb789WDvEevyaZdyaKlv7nDVr1qAPrOlcU+UJesuWLQG2LAoUI+lFixYF660sHR9bIw2b\/qusqbJNK4aTuFzBLxWIGPIvNPGruPwSBVvXRdlh6fZo+l\/4sJLuAJGqNFTuFBR3\/\/kdaEYQo0ePDqbGuPmDGyv4bNiwIfhvtkEjlsUy\/O5\/3MH9qN1\/lJF0TpXfecfy\/KZPFKny039c7sANK9QFlzLwCcvjypY+sDzqhVF82O4\/1o\/6OmdUpIqELn6\/SlwK4DFifXjssccCUhU33ohU3Rl\/RKru2Ip6ahCBtGdnk9ZUdXWZSFUXkublEKmax5hasBCBsGNnabL3E6laaNyCu0SkWrABqPliEBCXJ9Jm7xfv\/ovrvjq0o7v\/OlDMT8b\/A4IC5dSAwtcmAAAAAElFTkSuQmCC","height":205,"width":341}}
%---
%[output:37f5c355]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:58a706d0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:2e2041d5]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
