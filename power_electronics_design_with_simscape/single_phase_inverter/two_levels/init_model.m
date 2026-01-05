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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnX9wXld555+0DkQZQrHAhiqKJAeksmS2pm47aEWncuwG6BJ7t06JLLc7xitcLzSTzCLX+pE\/HLeNbHnQzMZ1SU2sGs9QSU7Au0S7pWnqxOqCqlJI8VIDjYMsK8IUe22nCa2yxSU7z1XO66vr99U995x77zn3eb93JiNL7\/n5+T7vOd+cc+69N7z22muvES4QAAEQAAEQAAEQKBCBG2BgCqQWmgoCIAACIAACIBAQgIFBIIAACIAACIAACBSOAAxM4SRDg0EABEAABEAABGBgEAMgAAIgAAIgAAKFIwADUzjJ0GAQAAEQAAEQAAEYGMQACIAACIAACIBA4QjAwBROMjQYBEAABEAABEAABgYxAAIgAAIgAAIgUDgCMDCFkwwNBgEQAAEQAAEQgIFBDIAACIAACIAACBSOAAxM4SRDg0EABEAABEAABGBgEAMgAAIgAAIgAAKFIwADUzjJ0GAQSI\/AmTNnaNu2bXT+\/PlSoRs2bKB9+\/ZRTU1NbEXz8\/PU29tLnZ2d1NraGpt+amqKtmzZsijdjh07qKenh5KWFVsZEoAACIgmAAMjWl50DgSWJsAGZteuXbR\/\/35qbm5ObCKSmg42MIODgzQ8PEy1tbWl+urq6gITgwsEQAAEdAnAwOiSQjoQEEggzsCEV0zUSgljYBNy6NAham9vD6jwZ7wCc+zYMerr6yv9LWpKogaGE3IbBgYG6Pd\/\/\/cDI8WrOatXrw5WdsbHx4Oy1KoQ\/1v9nf\/28ssvU39\/Pz333HM0OTlJs7OztHnzZmpsbFy00jMyMkItLS3U3d1Nv\/zLv0y\/93u\/R2yaPvWpTwV9OXXqFO3du5c6OjoEqowugYBMAjAwMnVFr0BAi0C5LSQ1kYfNTX19fWAc2traAnOgVlEuXboUbEGxEeBrdHQ02H5SRiO6tVTOwFy+fDkwFp\/85Cfp8OHDgYG57bbbgjJuvfVWUp+HjQrXwaZj586ddOTIkcDAjI2NlVZ2uB61pcWmamZmhrZv305dXV3B39lYcR84Ha8GPf3004EB0t0604KLRCAAApkSgIHJFC8KBwG\/CVRagVFGRRkSPg+jjIDqUfTcyrlz50qrLypNeNWG\/6ZrYNhkqO0pXoXh1ZJHH300MDjcNl4pqWRs1Nmd6OqRMjDcbrVaxMaGf+e04b76rRpaBwIgwARgYBAHIFDFBKIGhlEoo8LbQ0kNjDIElZDqbiFxfj7sG976USs0cQZGrf7wT15RefLJJxetwMDAVHHAo+uiCMDAiJITnQGBZASWWoFZs2ZN6YCv7haS2tIJpw+fK1nqEO\/9999fuqOJe6HMU3SrSG31VPq7MjDhszS8goMVmGSxgdQg4DsBGBjfFUL7QCBDAnG3UWdxiFfnNmo+cMvnVdikcPpXXnnlusO95Q7xqjMs6jAxGxcu5xvf+EZgxu67775gywhbSBkGFYoGgZwIwMDkBBrVgAAIpEug3HZUujWgNBAAAZ8JwMD4rA7aBgIgsIhA+DZt\/oDPyOg8QA8YQQAE5BGAgZGnKXoEAiAAAiAAAuIJwMCIlxgdBAEQAAEQAAF5BGBg5GmKHoEACIAACICAeAIwMOIlRgdBAARAAARAQB4BGBh5mqJHIAACIAACICCeAAyMeInRQRAAARAAARCQRwAGRp6m6BEIgAAIgAAIiCcAAyNeYnQQBEAABEAABOQRgIGRpyl6BAIgAAIgAALiCcDAiJcYHQQBEAABEAABeQRgYORpih6BAAiAAAiAgHgCMDDiJUYHQQAEQAAEQEAeARgYeZqiRyAAAiAAAiAgngAMjHiJ0UEQAAEQAAEQkEcABkaepugRCIAACIAACIgnAAMjXmJ0EARAAARAAATkEYCBkacpegQCIAACIAAC4gnAwIiXGB0EARAAARAAAXkEYGDkaYoegQAIgAAIgIB4AjAw4iVGB0EABEAABEBAHgEYGHmaokcgAAIgAAIgIJ4ADIx4idFBEAABEAABEJBHAAZGnqboEQiAAAiAAAiIJwADI15idBAEQAAEQAAE5BGAgZGnKXoEAiAAAiAAAuIJwMCIlxgdBAEQAAEQAAF5BGBgNDS9\/fbbNVIhCQiAAAiAQJYEVqxYQe3t7XTx4kWamJjIsipRZU9PT4vqj+oMDIyGrGxg8g6ANOpMWoZO+rg0lT5P8vdo2rg6NSRMnCSNOpOWoZM+Lk0SzgylXHrwXzpcltIg6Wc6\/CvplDioE2aIizWd4pKWEZd+dnaWDh48SGvWrKHNmzeXbQK+A9ewKBZxXHW09DENDIyGKlLF1+i6F0nA360M4O+WvysD477X17eADczY2Bg1NDRUNDBZtLvo34Git7+SpjAwGtEuVXyNrnuRBPzdygD+bvnDwFzjDwNjFotSv8MwMBrxIFV8ja57keTs2bO0atUqL9pSjY0Af\/eqQ4MFDVwZmKLzlzqHwcBojE1SxdfouhdJij54eAHRohHgbwEvpazQAAbGJpSkzmEwMBpRIVV8ja57kQSDt1sZwN8tf64dGsDA2ESh1DkMBkYjKqSKr9F1L5Jg8HYrA\/i75Q8Dc40\/tpDMYlHqHAYDoxEPUsXX6LoXSTCBupUB\/N3yh4GBgbGNQKlzGAyMRmRIFV+j614kwQTqVgbwd8sfBgYGxjYCpc5hMDAakfHfP1RD72tt1UiZf5IbVzTlX6lFjcs02rv83t2LasAEagE8hazgnwJEyyKgwQJAbCGZBRIMjBk3EbkeXXcL\/fo9v+5lX350ccbLdlVq1NULydt7NcM+Rg3VspXXDKEyh+E0UXNVKPiGjcXkaQguxWzQAAbGJpxgYGzoFSDv1NQUbdmyJWjp3r17qaOjo9RqqeIXQJagiVkN3q+c\/CxdvXBuEQZllsLGUJmuOCOljA6boLD5KbrpyYp\/UeLPh3ZCAxgYmziUOodhC4mILl++TN3d3dTf3x\/EyMDAAA0NDVFtbW3wu1Txbb4QeeYtwuAdNkNsdJQBYvNTzviEzU7Ne9YGOGvuaA9+3nTHwu++XEXg7wurrNoBDWBgbGJL6hwGA0NEvPoyODhIw8PDVFNTQ729vdTZ2Umtr597kSq+zRciz7wSBm9lYtjQsLlRKz\/z3zoZoHz19MJPdSmDU3PHWuJ\/L1vZGKzquDA3EvjnGa9Z1AUNYGBs4krqHAYD87qBGR0dpX379gUxwgamra2ttI30tv\/yxKLYecc73mETS8ibkMDVq\/9Ky5b9ZMJcfiY\/\/\/JVrYb91stHg3Q\/\/+op+vn\/941Fec4vewd9\/yffQV+\/aXXw8\/yyt9PX3\/herXKRCASKTuB3VkzRypUrad26dbl1ZW5ujurr63OrL42K1q9fv6iY6enpNIr1qgwYGA0DU\/fv\/ys98MADXglXTY25t+U1vAspJPiVx\/cQr9yUW7W5Ze1Hg5S8HZXWag3+79\/9tw0aYAXGJgqxAmNDz\/O82ELyWyAM3pX1UVtTrzy7sGITNTa8\/WRrasDf\/fcDGsDA2EQhDIwNPc\/z4hCv3wJh8E6uDxub+dMng7M2YVOzcJ6mifjgsO4qDfgn5592DmgAA2MTUzAwNvQs8h47doz6+voWlRC9zdmi+FLW8G3UIyMjpQO8nECq+Glwy6MMDN7pUGZTo1ZqrjzxUKlQtUpTydCAfzr8bUqBBjAwNvEjdQ7z9gyMMhTlzIoyNVGjYSPwUnmlip8Vr7TLxeCdNtFr5fE5mvnTE9et0vC2k3p+Dfhnx1+3ZGgAA6MbK+XSSZ3DvDQwvKUzPj5OW7duXVKzo0ePxqaxEV3llSp+GmzyKAODdx6UKXheDd\/mHTU0tLyelv\/Kx7S3nPJpbXXVgu8ADIxNxEudw7w0MDZCZZFXqvhZsMqiTAzeWVCNL1MZmvNf+R+07BtfLD2QL27LKb5kpEhKAN8BGJikMRNOL3UO89bAnDlzhrZt20bnz58PHu3PF5+FqauroyNHjlBzc7ONnonyShU\/EQSHiTF4O4QfepVD+GCwOkPDZkY9bK\/or0xwS3np2vEdgIGxiU+pc5iXBmZ+fr70NNyWlhbq6uqizZs3Bw+W4\/Mvk5OTwUPn+Km5eVxSxc+DXRp1YPBOg6J5GZX4q0PB0buc+PyM7h1O5q2qrpz4DsDA2ES81DnMSwPDZ2D27NlDu3fvDt5HxI\/5b29vD+4Min5mI6puXqni6\/bfdToM3m4VSMKfH7IXvBfq4szrr0BYuGUbqzN2GibRwK4mv3PPzs7S2NgYNTQ0BP9Tm9dVdP5S5zAYGI1vgFTxNbruRZKiDx5eQLRohAl\/rM5YAC+T1USDdFvgR2kwMGY6SJ3DYGA04kGq+Bpd9yIJBm+3MqTBXxma8NkZ3mril1SqJwW77aXftaehgd891GsdDIwep2gqqXOYtwaGz72cOnWqrFqrV68O3hzN20t5XFLFz4NdGnVg8E6DonkZafMvtzrD723CVlNljdLWwDwa3OaEgTHjL3UO89LAmEmUXS6p4mdHLN2SMXinyzNpaVnz53MzfEVXZ3Bu5ppSWWuQNCZcpYeBMSMvdQ7z0sDwQV2swJgFqsRcGLzdqpon\/0pbTdV+V1OeGriNtqVrh4ExUwcGxoybdS6+bXpmZoZ6enqCsvh3vviW6rwuqeLnxc+2HgzetgTt8rvir8xM+K6maj0340oDu8hJPzcMjBlTqXOYlyswSqJyt0zjNmqzAC5yLgzebtXzgX+1n5vxQQO3UbhQOwyMmQowMGbcrHKpB9q1tbWVVlz4mTD8dF48yM4KbaEyY\/B2K5eP\/PncTGBqTn42gKNebyD13IyPGriIShgYM+owMGbcrHNFz8PkfQcSd0Cq+Nbi5FQABu+cQFeoxnf+6o3a0UPAt9y5NTA2Ei7fNciLMQyMGWmpc5jXW0hmUqWfS6r46ZPKpkQM3tlw1S21aPx5dSZqZop+CLhoGujGVtJ0MDBJiS2klzqHeWlgeNVlfHyctm7duqRaR48ejU1jJvfiXFLFT4NNHmVg8M6DcuU6isq\/0iHgIpqZomqQduTCwJgRlTqHeWlgWKKpqSnasmVL8Cbq6B1HfCcSv5l6ZGQkeD9S1pdU8bPmllb5GLzTImlWjhT+vDKjXjypzswU5UnAUjQwi8BruWBgzAhKncO8NTBKJmVWwrKVMzVmsurlkiq+Xu\/dp8Lg7VYDafyjdzSxmam5Y21wXsbXQ8DSNDCNaBgYM3JS5zDvDYyZXOnmkip+upSyKw2Dd3ZsdUqWzL8ot2dL1kAnBlUaGJgktK6llTqHwcBoxINU8TW67kUSDN5uZagW\/srM+Hh7drVoEBfpMDBxhMp\/LnUOg4HRiAep4mt03YskGLzdylCt\/Cvdnu1im6laNYhGPgyM2VggdQ6DgdGIB6nia3TdiyQYvN3KAP5Erp81Aw0WvgMwMGZjgdQ5zHsDE36Q3eHDh+mZZ54Jbp1ubm42U9Igl1TxDVA4yYLB2wn2UqXgv5h\/pRdOZvngPGgAA2MzCkidw7w2MOFXCfALHdvb2wMNR0dH8SoBm2guWF4M3m4FA\/\/K\/MuZmYZPn01dMGgAA2MTVDAwNvQM84Zf3PjYY48FBqalpYX27NlDu3fvptraWsOSk2WTKn4yCu5SY\/B2x55rBn89\/lk+OA8awMDoRWH5VFLnsMKtwExMTOBljjaRXMC8GLzdigb+yflXMjOmB4ChAQxM8ii8lgMGxoaeRV68zNECnpCsGLzdCgn+dvzTWJmBBjAwNlEIA2NDr+B5pYpfFFkweLtVCvzT42+6MgMNYGBsolDqHOblFlJ01SUq3OrVq2l4eBhnYGwiukB5MXi7FQv8s+Ff7pUGt6z9KJV72SQ0gIGxiUIYGBt6Fnn5XUh8B1JPT09QCv\/OV\/QFjxZVxGaVKn5sxz1JgMHbrRDgnz3\/ODMDDWBgbKJQ6hzm5QqMEip8F5K646jc35IIq95yrfKoF0OG\/x59WaRU8ZNwc5kWg7dL+rgLKW\/65czM1ff+B6p7\/3+km+5Ym3dzvKoPD7Izk0PqHOa1gQk\/B0atuAwODlrdhRRd0eFwYFPU3d1N\/f39QXQMDAzQ0NBQaYtKqvhmX4X8c8HA5M88XCP4u+V\/5fE9dOW5PyP67lTwxmzeZlq2sjH4WW0XDIyZ4lLnMK8NjDIXXV1ddOrUqUA52\/MvbICampoWbUHx6gv\/nc\/V1NTUUG9vL3V2dlJra2tQp1Txzb4K+efCBJo\/cxgYt8yjtfN34LY33UCvPHuU5r91Mni1QTWaGRgYs7iUOod5b2DM5Cqfq9It2c8\/\/3zp6b6ckw1MW1tbyeRIFT9NtlmWBQOTJd34ssE\/nlHWKaIaVDozI31lBgbGLNKkzmFeG5hKdyPZrsKoEODtpMnJSdq0aRMdP348eD1BJQMTDpsTJ06YRRFyGRGYm5uj+vp6o7zIZE8A\/O0Z2pawpAZX5ui1r32BaPqvg20mWl5P9Av30A23v4\/onQuryFKuCxcuBO\/DW7lyJa1bty63bhXxO7B+\/fpFfKanp3PjlVdFXhuYchDYdDQ2Npa2d5YCdebMGdq2bVtwZmbDhg3XvT9JbR19\/OMfp0cffRRbSHlFXcJ6sAKQEFjKycE\/ZaAGxelqEHc3k0HVXmXBCoyZHFiBMeOWei6bu5CiefncC1\/bt2\/HId7UlUqvQN3BO70aUVKYAPi7jwcTDSqZGdPXGbinQAQDY6YCDIwZt9RzhQ\/cmrzMMXy7dHgrKvz3kZGRRSs8UsVPXZyMCjQZvDNqSlUWC\/7uZbfVoJKZueXOrcFh4KJcMDBmSkmdw7zeQqp0Bib6nBYzSfVzSRVfn4DblLaDt9vWF7928HevYZoamL7OwD0FrMCYaiB1DvPawJiKlXY+qeKnzSmr8tIcvLNqo+Rywd+9ullpUDQzgxUYs1iUOod5bWCyeBKvifxSxTdh4SJPVoO3i74UsU7wd69aHhooM3PliYeCDqvnzPh0ZgYGxiwWpc5hXhoY9QTe8fHxsmqVu6PITFa9XFLF1+u9+1R5DN7ue+lvC8DfvTZ5axA1M\/wKg1vWbnX+9F8YGLNYlDqHeWlglEQ2dxyZyVw+l1Tx02SUZVl5D95Z9qWIZYO\/e9VcavDKyc\/Sq6cniH+qVRlXh39hYMxiUeoc5qWBUcbl\/vvvp507d5ZeI6CkS+tBdrqhIFV83f67Tudy8Hbddx\/qB3\/3KvigQbnzMg2fPpsrHBgYM9xS5zAvDYyZRNnlkip+dsTSLdmHwTvdHhWrNPB3r5dvGkS3mJZ\/5CGquaM987dlw8CYxaLUOcx7A8NP3u3r61ukGlZgzIK4qLl8G7yLytG03eBvSi69fD5rUG6LKauDvzAwZjEFA2PGzSqXeg5MT0+P1qsDrCpbIrNU8bPilXa5Pg\/eaffVx\/LA370qRdCg3KpM2mdlYGDMYlHqHOb1CgwO8ZoFq7RcRRi8pTEP9wf83atbNA1ePX2SLj+xh\/gnH\/xN66wMDIxZLMLAmHGzzsVbSHx1dHRYl2VagFTxTXnkna9og3fefLKuD\/yzJhxfflE1CK\/KpPFcGRiY+Fgpl0LqHOb9CkxXVxfuQjKLWTG5ijp4SxEA\/N0rWXQNokbGdEUGBsYsFmFgzLiJyCVV\/KKIU\/TBuyicK7UT\/N0rKEWDqJFZ+dtHEt25BANjFotS57BCrsCwhHV1dXTkyBFqbm42UzRBLqniJ0DgNKmUwdspRIvKwd8CXkpZpWkQNTJ1e57Veis2DIxZQEmdw7w2MCwVn4GZmZkhvhNJ\/c4\/GxsbaXR0lB555BEzRRPkkip+AgROk0obvJ3CNKgc\/A2gpZxFqgZsZM7vvpP45+1PvBZLDQYmFlHZBFLnMK8NzFIvc+Sn9B44cAAGxiyeC5VL6uBdFBHA371S0jW48vge4pdI8juX6h56tiJwGBizWISBMeNmlUu91JG3i8IrMJOTk7Rr1y763Oc+V\/q7VUUxmaWKnyWzNMuWPninySqLssA\/C6rJyqwGDdRqDJOpdDYGBiZZ3KjUUucwr1dgGL56mN2pU6cCLfgpvAcPHqT9+\/dTW1tbLrdXSxXf7KuQf65qGLzzp6pfI\/jrs8oqZbVowCbmwh9uo6sXZso+OwYGxizCpM5h3hsYM7nSzSVV\/HQpZVdatQze2RG0Kxn87filkbvaNDj\/0J3BQ\/B4O4m3ldQFA2MWTVLnMO8NzODgIB06dGiRangXklkQFzVXtQ3evukE\/u4VqUYN2MREV2JgYMxiEQbGjJtVLt4+6u7upv7+fnruueeCO4\/OnTsXlJnnk3mlim8lTo6Zq3HwzhFvbFXgH4so8wTVqsHsJ1bRspVNpYO9MDBmoSZ1DvN6BSZ8F9Lzzz9PExMTtH37dtqzZw\/t3r2bamtrzdRMmEuq+AkxOEterYO3M+CRisHfvRLVqgFvI\/FKjNpKgoExi0Wpc5jXBobvQuJbpdm0XLp0ibZt20bnz58PDvIODw\/DwJjFcuFyVevg7YtQ4O9eiWrW4OIfbqP50yeDQ70wMGaxCANjxs06F999dPPNNwdP3J2amqKdO3fm9gRe1Xip4luLk1MB1Tx454R4yWrA370K1awB35nEW0krfvsIXbl9HY2NjVFDQwNt3rw5N2GKzl\/qHOb1Ckxu0RlTkVTxfeEb146iDx5x\/fP9c\/B3r1C1a8CrMHzNb9gDA2MQjlLnMBgYjWCQKr5G171IUu2Dt2sRwN+1AkTVrsErJz9LbGKWDZ2DgTEIR6lzmJcGJvrwuqheOANjEMEFzlLtg7dr6cDftQIwMKwAbyP9sPU\/0\/\/8hzdiCylhSMLAJARmkzxqYEZGRqi1tdWmSKu8UsW3gpJjZkygOcIuUxX4u+XPtUMDClZgZl+cpRO1H4SBSRiSUucwL1dgwtqUe5VAnncgcVukip\/wO+AsOQZvZ+iDisHfLX9osMBfbSN9\/hcHYWAShqTUOcx7AxPVie9E4qfz5mlipIqf8DvgLDkmUGfoYWDcoi\/Vju\/AAorpj9xAf\/qzvfS25tW4CylBbEqdw7w3MGfOnCk9\/4X12rBhA+3bt49qamoSyGeXVKr4dlTyy43BOz\/W5WoCf7f8sQJzjT8bmK+tupd+vObXYGAShKXUOcxLAxPeNrI9sHvs2DGamZmhnp6eQO5w2WEzxCs7W7ZsCdLs3bt30asKpIqfIP6dJsUE6hQ\/tpDc4scqWIg\/H+T9P29oDg7z4jkw+oEpdQ7z3sCUk0jX1KgXQe7YsaNkYPhvTU1NtHHjRurt7aXOzk5qaWkpvXOJ6xsYGKChoaHSk36liq8f\/m5TwsCAv1sC7mvHd2BBAz7I+82\/+yZdaN8JA5MgLKXOYV4amAS6VEzKKy\/88kd+fxJfvAKjVl\/433xXk1qdaW9vL52r4a0pZWzUnU9SxU+Dcx5lYPDOg3LlOsDfLX+uHRpcMzDTX\/9LOvuBh2FgEoSl1DlMrIFR2vKKS9jAqLdb86sJ2MBMTk7Spk2b6Pjx48HZGr7YwLS1tZW2kaSKnyD+nSbF4O0UPyZPt\/iD2vEdWBDhyuN76HtPPkJ\/d\/enYWASxKXUOQwGRtPAhGPlxIkTCUIHSW0JzM3NUX19vW0xyG9IAPwNwaWYDRq8DvNrX6DXHv8devZDh2ndunUpEl66qCLyX79+\/aJOTU9P58Yrr4pEGBh+azWvmoyPj1NdXd2ilz1GV2C6urqC7SRsIeUVYvb14P8+7RnalAD+NvTSyQsNFjjy26ivdjfSV+8ZxQpMgtDCCkwCWD4lDRsYbhcO8fqkjl5bMHjrccoqFfhnRVa\/XGgAA6MfLdenhIGxoecwb9TAhG+jDt+dFL6NOvrqAqniO5QlUdUYvBPhSj0x+KeONHGB0AAGJnHQhDJIncNEbCHZCKuTV6r4On33IQ0Gb7cqgL9b\/lw7NICBsYlCqXMYDIxGVEgVX6PrXiTB4O1WBvB3yx8G5hp\/PgMzNjaGdyElDEmpcxgMjEYgSBVfo+teJMEE6lYG8HfLHwYGBsY2AqXOYTAwGpEhVXyNrnuRBBOoWxnA3y1\/GBgYGNsIlDqHwcBoRIZU8TW67kUSTKBuZQB\/t\/xhYGBgbCNQ6hwGA6MRGVLF1+i6F0kwgbqVAfzd8oeBgYGxjUCpcxgMjEZkSBVfo+teJMEE6lYG8HfLHwYGBsY2AqXOYTAwGpEhVXyNrnuRBBOoWxnA3y1\/GBgYGNsIlDqHwcBoRIZU8TW67kUS8HcrA\/i75c+1Q4MFDVzdRl10\/kVvf6VvIAyMxtgkVXyNrnuRBPzdygD+bvnDwLhfgSn6d6Do7YeBsRiDXIifRp1Jy9BJH5em0udJ\/h5NG1enhbQVs6ZRZ9IydNLHpUnCudLECP5LR9RSGiT9rFx63b9lEffhMuNiTaf+pGXEpecVmIMHD9KaNWsqvswR34FryigWcVx1tPQxDVZgNFRh8XGBAAiAAAi4JbBixQpqb2+nixcv0sTEhNvGFKz26enpgrU4vrkwMPGMkAIEQAAEQAAEQMAzAjAwngmC5oAACIAACIAACMQTgIGJZ4QUIAACIAACIAACnhGAgfFMEDQHBEAABEAABEAgngAMTDwjpAABEAABEAABEPCMAAyMZ4KgOSAAAiAAAiAAAvEEYGDiGSEFCIAACIAACICAZwRgYDwTBM0BARAAARAAARCIJwADE88IKUAABEAABEAABDwjAAPjmSBoDgiAAAiAAAiAQDwBGJh4RkgBAiAAAiAAAiDgGQEYGM8EQXNAAARAAARAAATiCcDAxDNCChAAARAAARAAAc8IwMB4JgiaAwIgAAIgAAIgEE8ABiaeEVKAAAiAAAiAAAh4RgAGxjNB0BwQAAEQAAEQAIF4AjAw8YyQAgRAAARAAARAwDMCMDCeCYLmgAAIgAAIgAAIxBOAgYlnhBQgAAIgAAIgAAKeEYCB8UwQNAcEQAAEQAAEQCCeAAxMPCOkAAEQAAEQAAEQ8IwADIxngqA5IACutCJCAAAgAElEQVQCIAACIAAC8QRgYOIZIQUIgAAIgAAIgIBnBGBgPBMEzQEBEAABEAABEIgnAAMTz4hqP\/msRqprSX7in\/9vxfQ\/8c+XrvusXHqV7qbvfDFR3UgMAiAAAlIJrFixgtrb2+nixYs0MTEhtZup92t6ejr1Mn0oEAZGQ4Xbb7+dkgTA4FMzFUudvTx\/3WcvXn71ur9NfvMFqq+vL\/19tkwajaZTQ+1Ni5I1LF\/4\/bbQ3xtqa6jng02k08+4NJU+T\/L3aNq4OnU4JE2TRp1Jy9BJH5cmCWdmUi49+C8dLUtpkPQzHf6VdEoa00nTx8WaTnlJy4hLPzs7SwcPHqQ1a9bQ5s2byzYB34FrWBSLOK46WvqYBgZGQ5Wiif\/lF16iF68smKKo8VEGKmyaZiukjaIJmyE2QsoEsQHiiz+\/bflN9EvveosGVf0kReOv37NipAR\/9zpBgwUN2MCMjY1RQ0NDRQOThVpF51\/09lfSFAZGI9qliq\/RdWIzxBcborAZYiOkTBAboKVWiNjYhFd+lOF5\/zsXjE6c4alm\/joaZZ0G\/LMmHF8+NICBiY+Syimkxg8MjEZUSBVfo+tGSdjM8H+VTE8lw6OMDq\/sKJPDW1tnz56lVatWGbUFmewJgL89Q9sSoIFbA1N0\/lLnMBgYjZFFqvgaXc8tyejf\/ENgetTKTjmTowzO+9+1PGgXmxtc2RMo+uCdPaHsa4AGMDA2USZ1DoOB0YgKqeJrdN15EjY1X\/3WWfrRG5cvMjhf\/u7C1pa6wuYGxiZd2TB5psvTpDRoAANjEjcqj9Q5DAZGIyqkiq\/RdS+SVBq81bmbr3z3pcDcfOWFK0F7w+aGjQ2fteEtKf4Zd97Giw571ghMnu4FgQYwMDZRKHUOg4HRiAqp4mt03YskJoN3YGhCxiZqavhQMW9FwdTES2zCP75UpEhCABrAwCSJl2haqXMYDIxGVEgVX6PrXiRJc\/BmY8PnbfgafOpsqX\/hLSiYmsWyp8nfi4AqYCOgAQyMTdhKncNgYDSiQqr4Gl33IkmWg7e6Y4pXa3gLKrpS0\/mLP131h4Wz5O9FgBWgEdAABsYmTKXOYTAwGlEhVXyNrnuRJO\/BO7xKEzY1vErDhqbaVmjy5u9F0HnWCGgAA2MTklLnMBgYjaiQKr5G171I4nrwDq\/SjP7N90sP7asWQ+OavxdB6LgR0AAGxiYEpc5hMDAaUSFVfI2ue5HEt8E7fEBYnaORfBu3b\/y9CMqcGwENYGBsQk7qHAYDoxEVUsXX6LoXSXwfvNWWk9TtJt\/5exGkGTcCGsDA2ISY1DkMBkYjKqSKr9F1L5IUcfDmN5Kr7Sa11bTw8x1eME3SiCLyT9K\/IqSFBjAwNnEqdQ4Tb2COHTtGMzMz1NPTE+h\/+fJl6urqolOnTtGGDRto3759VFNTQ1NTU7Rly5Ygzd69e6mjo6MUL1LFt\/lC5Jm3yIN3udWZX3rnW4Jn0BTlicFF5p9nnGZZFzSAgbGJL6lzmGgDMzg4SIcOHaIdO3aUDAz\/rampiTZu3Ei9vb3U2dlJLS0t1N3dTf39\/UGMDAwM0NDQENXW1ga\/SxXf5guRZ15JgzevzPD7ntSzaNTqjM9mRhL\/POM2zbqgAQyMTTxJncPEGhheeWlsbKSJiYlAd16BUasv\/O\/W1lZSqzPt7e3ExmZ4eDhYjVHGhtPAwNh8bdLJK3Xw\/vILLwVPCw4fBObbtHmbiY2NL5dU\/r7w1WkHNICB0YmTSmlgYGzoOczLxiRsYNRKS3Nzc2BgJicnadOmTXT8+PFgO4kvNjBtbW2lbSQWP3ydOHHCYY+qr+q5uTmqr68X3\/HP\/PVLdOirCy+prHvzMtrw7jfR3f\/mTcG\/XV7Vwt8l47i6ocECoQsXLtAzzzxDK1eupHXr1sVhS+3zIvJfv379ov5PT0+nxsOXgsSuwCjAaRkYieL7EoRx7ai2\/\/tU52Z8WZmpNv5x8ejic2iwQH12dpbGxsaooaGBNm\/enJsUReePFZjcQiXdiqIGhg\/wYgspXcZZl1b0wcOGjw9mppr522iXZl5oAANjE08wMDb0HOYNGxhuBg7xOhTDsGoM3q8P3q+\/iDLvlRnwNwzcFLNBAxgYm3CCgbGh5zBv1MCEb6MO350Uvo16ZGQkOOSrLqniO5QlUdUYvK\/HlefKDPgnCtdMEkMDGBibwJI6h4k\/A2MjOgxMGvTsy8DgvTTDrM0M+NvHsG0J0AAGxiaGYGBs6BU8r1TxiyILBm99pSqZGZs3aIO\/Pv+sUkIDGBib2JI6h2EFRiMqpIqv0XUvkmDwNpNBmZnoKw2SmhnwN+OfZi5oAANjE09S5zDvDQw\/q6Wvr2+RdtFH\/dsIq5NXqvg6ffchDQZvexWirzTgB+WxkWmorYl9pQH42\/O3LQEawMDYxJDUOcxbA6MO1ZYzK8rURA\/b2gi8VF6p4mfFK+1yMXinS3Spt2eXewow+KfL36Q0aAADYxI3Ko\/UOcxLA8N3Co2Pj9PWrVuX1Ozo0aOxaWxEly5+GmzyKAODd3aU2czwf+VeacC18juawD87\/rolQwMYGN1YKZcOBsaGXsHzShW\/KLJg8M5PqXKrM\/wqg\/\/0724Ltpx+6V1vya8xqKlEAN8BGBibr4PUOczLFRglVPiZLVHx6urq6MiRI8TvNMr6kip+1tzSKh+Dd1okk5WjVmf+19fO0pdeWFip4Uu9QRuGJhlPm9T4DsDA2MSP1DnMawPDgqk3RvPj\/9Xv\/JPfND06OkqPPPKIja5aeaWKr9V5DxJh8HYrguKvVme4NeppwDA0+WiD7wAMjE2kSZ3DvDYwvAKzZ88e2r17N9XW1gb6qb\/df\/\/9dODAARgYm6guSF4M3m6FWor\/4FMzQePUrdphQ8P\/xipNOtrhOwADYxNJMDA29Azzzs\/PU29vL\/F2UXgFZnJyknbt2kWf+9znSn83rEIrm1TxtTrvQSIM3m5F0OUfPhD8lReu0Je\/+1Kp4WrbiQ8F40pOQFeD5CUXKwfeRm2ml9Q5zOsVGLXiwm+QPnXqVKDc6tWr6eDBg7R\/\/35qa2ujjo4OM0UT5JIqfgIETpNi8HaK3+oupPC2UzlTo55Fg5WapTXGdwArMDajgNQ5rFAG5vDhw\/TMM88Et07ncXhXBYxU8W2+EHnmxeCdJ+3r60qbv7ptO\/hZZqWGWwBjs1iHtDVwG1HmtWMFxoyd1DnMawOjtpB4pWVmZoba29sD9fjw7r59+6impsZMzYS5pIqfEIOz5Bi8naEPKs6L\/5dfeCl4Hg1fUWPDf+NtqIblN9H737U8SFNNqzZ5aeA20uJrh4GJZ1QuhdQ5zGsDEz7E+9hjjwUGpqWl5bqDvWaS6ueSKr4+AbcpMXhXN\/\/wis3s5Xl68fKri87XKHMT\/Hzd4Eg7a4PvwMJ3AAbGbCyQOod5bWDKrcBMTEzQ+fPnsQJjFseFzIXB261sPvMPHxwOJrgKBidqcm7j1ZzahRVcXtm5bflNXj+kz2cN8oxOGBgz2jAwZtysc0UfZseHeIeHh0u3VVtX8HoB6t1L\/Gv0\/UtSxU+LXdblYPDOmvDS5ReZvzI4L1659iA+ZXK41+E7paIU2NioVR02PAtm55rp4d\/53VF5XEXWIE0+MDBmNKXOYV6vwJhJlTwXm6Tu7m7q7+8PMg8MDNDQ0FDJJEkVPzkpNzkweLvhrmqtFv7lzA4zCBue2ZARWkoVZX6UAeKfUROk8qu0S5mhatEgLtJhYOIIlf9c6hzmpYFZ6hUCLE\/aqzC8+jI4OBis7PDBYH72TGdnJ7W2tgbRIFV8s69C\/rkweOfPPFwj+JvxVw\/5YwPEF5\/dCQzRlYWfC+bo2r91awmbo5IJWr6wQqQuZZbCf1OrR9F6wuXltaKk29doOhgYM3JS5zAvDUxYokqvEkjz+S9sYNSdTVw3G5jwM2akim\/2Vcg\/FybQ\/JnDwLhlHq39wJ9+k1asWFHR9CiTpPIpsxQuJ2ycbAyULZlyBuw6UxUxZIv68eKL1Pvef6bNmzfbNkU7f9HHIKlzmNcGZqlXCYRfL6AdhRUSxhmY6Y\/ckKyK5fWV09eW+Wz5rYvTl8l\/w10PJGuDoNRzc3NUX78EU0F99bEr4O9eFSkajH\/7h\/T9l6\/GAj3\/Svk0\/\/RP\/0QXLlygre98hdatWxdbTloJish\/\/fr1i7o\/PT2dFg5vyvHawITvQlIrLrzVk\/ZdSFlsIV15fE9Zka9eXHh3TPj6UZm\/Xb2wOF25fDpRtGzF4ke3L1u58PuNr\/89\/PmylY10y9qP6hSba5qi\/99PrrAyqAz8M4CasEhosAAMW0gJA+f15FiBMeNmnSuPu5CkHuJ95eRn6eqFc4s0UEYobJrYLOkYpMVmp+k6E8QGqGSMVjZR1DyZBgMGb1Ny6eQD\/3Q42pQCDWBgbOIHBsaGXgHyhm+jHhkZKR3g5aZLFT+JLFEzxIZHmaClDJAyMbzyw+ZG\/b783t3a1WPw1kaVSULwzwRrokKhAQxMooCJJJY6h3m5hcQrIuPj48E7j5a6jh49GpvGRnSVV6r4abBZqoyw6QkbnldPn7wuW9jo1LxnbfC5WtH5\/s2NtGrVqqybi\/IrEMDk6T40oAEMjE0USp3DvDQwLJRaEYk+VI4\/4zuT+vr6KLpSYiPwUnmlip8VL91y2dTw6g2v5KitLmV0KpkcXslRBqfmjna66Y4Fs4MrOwKYPLNjq1syNICB0Y2VcumkzmHeGhglgjIrYVHKmRobcePyShU\/rt8+fM6G5sVTf0Vvufx80Jz5by2s3kQNDq\/g1NyxNtiiYmPDRietMzg+cHDZBkyeLukv1A0NYGBsolDqHOa9gbERLa28UsVPi0\/W5egM3nzXF5ubcsZG3VmFFRszpXT4m5WMXLoEoAEMjG6sYAXGhpTAvDAwbkVNOnirO6peefZoadUmbGyiqzXYhlpa36T83UaLzNqhAQyMTWRLncOwAqMRFVLF1+i6F0nSGrzZ2MyfPhmct4mu1rCpUSs1Se6Q8gJQxo1Ii3\/GzRRdPDSAgbEJcKlzGAyMRlRIFV+j614kyXLwrmRq2NCoA8PVvvWUJX8vAqwAjYAGMDA2YSp1DvPewIQfZHf48GF65plnglunm5ubbfRMlFeq+IkgOEyc9+DNpoa3n8KrNNVsaPLm7zDUvK0aGsDA2ASn1DnMawMTfpXAzMwMtbe3BxqqFy\/ym6PzuKSKnwe7NOpwPXgrQ8N9ufLEQ0GXwudopG85ueafRgwVvQxoAANjE8NS5zCvDUz4ZY6PPfZYYGBaWlpoz549lObLHOMCQ6r4cf325XPfBu9KKzR8hkbidpNv\/H2JyzzbAQ1gYGziTeoc5rWBKbcCMzExkfrLHOMCQ6r4cf325XPfB29laIInD1+cCVZn1PkZCaszvvP3JU6zbAc0gIGxiS+pc5jXBoYFy+NljnGBIVX8uH778nmRBu+lVmduuXNrIR+uVyT+vsRs2u2ABjAwNjEldQ7z3sDwk3j5\/Mv27dupq6uLTp06ldsrBFTASBXf5guRZ96iD978kL3w2ZmibTUVnX+esZpVXdAABsYmtqTOYV4bGN5Cevjhh4O7jp577rnAyGzatIn4JY4PPvgg4RCvTUgXJ6+UwbvcVlMRzIwU\/sWJ+OtbCg1gYGziFwbGhp5h3ugh3qamJrrrrrtwiNeQZ1GzSRy8o1tN6kF6Ph4Clsi\/aN8FaAADYxOzMDA29AzzqhWYu+++mw4dOkT9\/f1BSViBMQRa0GzSB2\/fzYx0\/kX4WkADGBibOIWBsaFnkXdqaoq2bNlCO3bsCM7BdHd3B0ZG90F26gxNT09P0IrwoeANGzbQvn37gq0oVQ+nib7tWqr4FrLkmrWaBm8fzUw18c81sBNUBg1gYBKEy3VJpc5hXp+BsRGM8w4ODgYrN2x+lIHhv\/FW1MaNG6m3t5c6OzuDZ8soY8T5BgYGaGhoiGpra4MmSBXflm9e+at18K5kZvK+Nbta+ecV3zr1QAMYGJ04qZRG6hzmvYFRJiQszOrVq2l4eLhkMMqJxisvjY2NxM+N4YsNjFp94X+3traSWp3hB+RxPVwmr8YoY8NpYGBsvjbp5MXgTcHzZcKvN8jzzAz4pxPHNqVAAxgYm\/iBgbGhZ5iXDYdaGeG7kNiQnDt3Liito6NDq1Q2JmEDE96CYgMzOTkZ3Nl0\/PjxYDuJLzYwbW1tpTqkiq8F0INEGLwXi1DpbqasVmbA3\/2XABrAwNhEodQ5zOsVmPBdSM8\/\/3ywmsLnYJK8SiAtAxMOnhMnTtjEEvImJDA3N0f19fUJc1VJ8itz9NrXvkDE\/12ZI1peT\/QL99ANt7+P6J0LK4i2F\/jbErTPDw0WGF64cCF4oe\/KlStp3bp19mA1Sygi\/\/Xr1y\/q3fT0tGZvi5PMawPDdyEdOHAgMC2XLl2ibdu2Ba8RiG4hqVcOjI+PU11dHR05cqR0yDdqYPhheNhCKk6Ackvxf596emX1nBnw1+OfZSpogBUYm\/jCCowNPYu8\/OTdm2++OTAkfKfQzp07FxmUuKLDBobT4hBvHDH\/PsfgnVyTNM0M+Cfnn3YOaAADYxNTMDA29BzmjRqY8G3U4buTwrdRj4yMBId81SVVfIeyJKoag3ciXGUT8+sM5r91kl49fTJ4HxM\/AXjZysbgZ9wF\/nGEsv8cGsDA2ESZ1DnM6y0kFowP2vb19S3STucuJBuxo3mlip8moyzLwuCdHt1yL5u86Y61VPOetVTpEDD4p8fftCRoAANjGjucT+oc5rWBid72bCOgTV6p4tswyTMvBu\/saPPKTGBqTn42qKTc7dngnx1\/3ZKhAQyMbqyUSyd1DvPewCS548hG4KXyShU\/K15pl4vBO22i5ctTqzPRN2dfeekK3f5b\/y2fRqCWsgTwHYCBsflqSJ3DvDYwLBhvIfGl+9wXG5Er5ZUqfhassigTg3cWVOPLDJ+bCa\/OZPW8mfgWVW8KfAdgYGyiX+oc5qWBCR+0LScazsDYhHLx8mLwdqvZ2ef+N\/30G\/+V5k9PBFtNvFITNjQ+vkHbLbH0a8d3AAbGJqpgYGzoFTyvVPGLIgsGb7dKRfmrrSZuldpugqHJViN8B2BgbCJM6hzm5QoMC3XmzJnSg+vCb422EdE0r1TxTXnknQ+Dd97EF9cXxz\/O0HBp2Hay0zBOA7vSi5N7dnaWxsbGqKGhgTZv3pxbw4vOX+oc5qWBUU\/W5TdF8\/NY1MPnXJ2DkSp+bt9+y4qKPnhYdt959qT82dDMnz5JVy+cKz17Rq3QLFvZFNyyjW2nZLIm1SBZ6cVJDQNjppXUOcxLAxN+B1JtbW2wGvPUU0\/RfffdZ6aeZS6p4ltiyS07Bu\/cUJetKA3+\/AA9PkMTvmUbpkZf1zQ00K\/N35QwMGbaSJ3DCmNgjh49Sg8++CDV1NSYKWiRS6r4FkhyzYrBO1fc11WWBX82Mlcv8ErNxKJVGmVq+Kd6SjBWa\/A+MBWUMDBmY4HUOQwGRiMepIqv0XUvkmQxgXrRsYI0Ii\/+YVPDaNSrDxQmfsie2oKqtjM1eWnge0jCwJgpJHUO89bA8Fuj+UWO5S7cRm0WxEXNhcHbrXKu+YeNDf\/7Rxdngnc6hY0N\/7vmjrXBk4QlmhvXGriNwGu1w8CYKQEDY8ZNRC6p4hdFHAzebpXylX\/cik14O4rNzYrfPuIWpEXtvmpg0SWjrDAwRtjwLiQzbDJywcC41RGDN\/gnIaAetKfO2KjtKP5dfVZuWyowPCsb6cYVTcQvuPTpwndgQQ0YGLOolDqHebmFZCZRdrmkip8dsXRLxuCdLs+kpUnjr1ZueCuKb\/Uuty0VNThsahbO4CwYHD6Lw7\/ndUnTwJQbDIwZOalzGAyMRjxIFV+j614kweDtVoZq41\/J4FRawWF11Pkb\/ncWZ3CqTYNKEQ8DYzYWSJ3DYGA04kGq+Bpd9yIJBm+3MoB\/ef7Be6EunAs+5Dum+AofLubf1SqN7d1T0GBBAzYwV7sb6av3jOJJvAmGBalzmFgDw2+x7uvrCyQO37UUflFk+BUFU1NTtGXLliD93r17F739Wqr4CeLfaVIM3k7xE\/gn41\/uDI7ONhU\/oTgwPWXO4UADGJhkUbg4tdQ5TKSB4Sf3DgwM0NDQEPGTfPlVBOfPn6d9+\/bRgQMHqKmpiTZu3Ei9vb3ErytoaWmh7u5u6u\/vD1QP5+XfpYpv84XIMy8G7zxpX18X+KfPP3oHVSWDo1Zwrjb+PC1f9W+9PWScPqHyJWIFxoy01DlMpIGJSsyrK6Ojo7Rr167gdQQ9PT3BO5Z4lWZmZoba29sDkzM8PBw86VcZG04DA2P2hUkzFybQNGkmLwv8kzOzyVFuBefKc39Gy17+h0V3USlzo87f+HoHlQ2LaF4YGDOaMDBm3LzIpV4Gedddd5VWWpqbmwMDMzk5SZs2baLjx48HKzR8sYFpa2srbSOx+OHrxIkTXvSrWhoxNzdH9fX11dJd7\/oJ\/u4lqaTBa08\/QjT910TfnVrcyOX1RLX1RLe\/j27gfy+\/leidC\/9DVuTrwoULtOJTrfTshw7TunXrcutKEb8D69evX8Rneno6N155VSR+BUatsvCqC59\/UVtFSQ2MRPHzCjLberACYEvQLj\/42\/FLI7eOBmrlptybwFUbeNWmyO+YwgqMWTRhBcaMWy655ufng1WT8fFxqquroyNHjhAbFLXy0tHREbRDHeDFFlIusqRWic7gnVplKOg6AuDvPihsNFDG5pVnjwYdKfeOqaI8qRgGxiwWYWDMuDnLxeaFz7aocyyqIcrU4BCvM2kSV2wzeCeuDBlgYDyMgSy+A2xsKq3WqJUa394EDgNjFpwwMGbcnOQK3xKtGqBumebVGvWiyB07dgQHevkK5xkZGVlkfKSK70Qcg0qzGLwNmlG1WcDfvfR5acCmptxKTXjrKYsH9ekShoHRJbU4ndQ5TPwZGDO5q0P8NNjkUUZeg3cefSliHeDvXjXXGlx5fM+irSdlaPI2MzAwZrEIA2PGTUQuqeIXRRzXg3dROGXVTvDPiqx+ub5oEH5+TfgsjTI0t9y5NdN3RMHA6MdMOKXUOQwrMBrxIFV8ja57kcSXwdsLGA4aAf4OoEeq9FWD8DmaK088FLQ6y\/MzMDBmsSh1DoOB0YgHqeJrdN2LJL4O3l7AyaER4J8D5JgqiqKBOkOjVmfS3mqCgTGLRalzGAyMRjxIFV+j614kKcrg7QWsDBoB\/hlATVhkETWoZGZs7mziF2ieO\/xJ+ru7P42XOSaIIalzGAyMRhBIFV+j614kKeLg7QW4lBoB\/imBtCim6BpUMjNJz8zwYeLvPfkIDEzCWJI6h8HAaASCVPE1uu5FkqIP3l5AtGgE+FvASymrNA3YiPBqChubJNtMF\/9wG01\/\/S\/p7AcexgpMgtiSOofBwGgEgVTxNbruRRJpg7cXUBM0AvwTwMooqVQN1MqMOgC8\/CMP0VK3Zp9\/6E56cXYWBiZhnEmdw2BgNAJBqvgaXfciidTB2wu4Go0Afw1IGSepFg14hYVXZnhVpuHTZ6+jOv2RG+hrq+6lH6\/5NazAJIg5qXMYDIxGEEgVX6PrXiQBf7cygL9b\/lx7NWkQXZVhM8PvauLrB3\/1RfrTn+2lhoaGXA1M0fkXvf2VvoEwMBpjk1TxNbruRRLwdysD+LvlX20GJkpbPQX4xhVNNL9hD42NjcHAJAxJqd9hGBiNQHAhfhp1Ji1DJ31cmkqfJ\/l7NG1cnRoSJk6SRp1Jy9BJH5cmCedKEyP4Lx0uS2mQ9LNy6XX\/ljioE2aIizWd4pKWEZeenwNz8OBBWrNmTcUVGHwHrimjWMRx1dHSxzQwMBqqsPi4QAAEQAAE3BJYsWIFtbe308WLF2liYsJtYwpW+\/T0dMFaHN9cGJh4RkgBAiAAAiAAAiDgGQEYGM8EQXNAAARAAARAAATiCcDAxDNCChAAARAAARAAAc8IwMAYCDI\/P09PPPEE\/f3f\/z197GMfo1WrVhmUgiw2BK5evUpf+MIX6AMf+AAtX77cpijkNSBw5coV+pM\/+ROam5sLDlO+973vNSgFWUwJvPzyywF\/1mHr1q106623mhaFfJYEnnzySbrtttvo537u5yxLQvakBGBgkhIjoj\/\/8z+nt771rdTc3BwMIh\/96EeppqbGoCRkMSHwj\/\/4j\/TYY4\/RzMwM\/e7v\/i7V1taaFIM8FgSOHTtG7373u4P\/PvOZz9Bv\/uZvwkha8Eya9Utf+hI1NTXR2972tmAM+sQnPkFveMMbkhaD9JYEnn\/++WAsuueee6i1tdWyNGRPSgAGJkTs8uXL1N3dTf39\/YE54YsH6r6+vuDfIyMjQZDygP2rv\/qrges+evQobdiwAZNo0sgrk16XP6+A\/fjHP6bPf\/7zYJ8C93ARuhqoPLwC8Md\/\/Me0Y8cOetOb3pRya6qvuCT8eRXmL\/7iL+jGG2+ku+++m2644YbqA5ZBj3U1+OEPf0hf\/OIX6e1vf3sQ+zAwGYgRUyQMzOuAzpw5Q9u2bQt+O3LkSGBg+G8DAwM0NDRE7LRHR0dp3759wU82LXxLHwxMOkGbhL9a7QL7dNirUpJqwOblwIED1NnZSS0tLek2pgpLS8r\/X\/7lX4hfMcCrMWwgsQpsHzS6Guzdu5dOnjxJP\/MzPxPc0s0XDIw9\/6QlwMAQETtuXgb80Ic+RA899BDt378\/MDC8+jI5ORmYFv6\/frU6c\/r0aXrPe94TLOHyJNrR0UFvfvObk7JH+tcJJOWvVsdgYNILoaQa\/NRP\/VTwneHtU5y\/sNchKf8f\/OAHwRjE57\/+6I\/+iDZu3AgdLGVIosHOnTvp29\/+Nr344ov0ve99L9DhgQcewCqkpQZJs8PAhIix+961a9ciA8PnLHp6egKT09XVFfybt454G4mDlifTD3\/4w0m5I30ZArr81f\/pwMCkH9xn5pUAAAWxSURBVEY6GvB35G\/\/9m+DgZvPYLCZuffeezF4pyCHDn8eg\/jcF\/8PFm9d8L9\/4zd+g5YtW5ZCC1CErgZqHJqamsIKjKOwgYExMDBYKswmWpMOHNm0orpLhQZu9U\/Cn8+B8X8wLulqlkSDdGtGaUkJwMDEGJhyW0hqCyMpbKRfmkC5gQP8840aaJAv72ht4O+WP9cODdxroNsCGJglDEylQ7w4LKcbXsnSRQcO8E\/GL43U0CANiuZlgL85u7RyQoO0SGZfDgzMEgaGP1K3UdfV1ZXuTspeluqsITpwgH\/+cQAN8mcerhH83fIvtwKDcci9JpVaAAPjrzZoGQiAAAiAAAiAQAUCMDAIDRAAARAAARAAgcIRgIEpnGRoMAiAAAiAAAiAAAwMYgAEQAAEQAAEQKBwBGBgCicZGgwCIAACIAACIAADgxgAARAAARAAARAoHAEYmMJJhgaDAAiAAAiAAAjAwCAGQAAEQAAEQAAECkcABqZwkqHBIAACIAACIAACMDCIARAAARAAARAAgcIRgIEpnGRoMAiAAAiAAAiAAAwMYgAEqozA\/Pw89fb20vj4+KKe79ixg3p6egpNg98l9NRTT1FXV1fQx7a2Nuro6Aj6pPrd2dlJra2t1\/WTPz9w4ABt376damtrC80BjQeBaiAAA1MNKqOPIBAioCby8OQuAVDYgPAb45MaGGagDNB9990nAQn6AAKiCcDAiJYXnQOB6wksZWD47euTk5M0OztLmzdvpjVr1tC2bdvo\/PnztHr1ahoeHg5WJ6ampmjLli3Eb2lfu3Yt3XLLLcHKBa988CoOr3BwWTMzM8Hv6q3u3Bq10sNlHDp0KGjgxMQEbdiwgfbt20dsPgYHB0ufjYyMBGlGR0eDz\/licxJdSeHyzp07F6y4lOtjeAWG61N1c3nhug8ePEgf\/OAHqbm5GeEDAiDgMQEYGI\/FQdNAIAsC5baQlDl5+umnaWxsLDAqfHV3d1N\/f38wmStDEjYqnI\/NBBuZSgamvb29rPng8nfu3ElHjhyht771rSXzw39nA8NtuHTpEg0MDNCDDz5If\/AHf0C7d+8u\/W1oaGjRVg\/n4brYPFXaJuOy2RCFt5DC+fhz7idfauspCw1QJgiAgD0BGBh7higBBApFQGcFhlc65ubmSqsvqoNsWD7+8Y\/To48+WlqNKWdswiswTU1N1NfXt4gRr8Kw2VBGRW358KoKr6KolZtwJmU0+G+8ghI+r8N9evjhh2nr1q2B2YpbgVEGJmpeuGxeyYmWXyiB0VgQqBICMDBVIjS6CQKKQBIDw6sf0ZUOnuCV8eDtJB0DU86QhMvRMTDKWHA\/1EpLuE9JDUyllRYYGHxXQKAYBGBgiqETWgkCqRHQNTCcjs+08FkY3k5R52N27dpFfMiVV0DCW0j3339\/6eDsxo0bS1tLbDbUVlF9fX0pTWNjY9kVGO5oeAuJ69u\/fz9xXr5L6Dvf+c51pkrliW4hVboLiVd5+Cq3TYQtpNRCDQWBQKYEYGAyxYvCQcA\/AroGhldF+K4c3UO84cO64cO9Sx3iLbeFpLaf1LbT3r17S0YjfDA4Sja8crLUFtKHP\/zhYAvs1KlTpSLUGSDuc3glxz\/10CIQAAFFAAYGsQACIGBFYClTYVVwJHMez3HBbdRpKoayQCBbAjAw2fJF6SAgnkAeBuby5cvBdlZDQ0PpVutyYG3Or0QPAosXDh0EgYITgIEpuIBoPgiAAAiAAAhUIwEYmGpUHX0GARAAARAAgYITgIEpuIBoPgiAAAiAAAhUIwEYmGpUHX0GARAAARAAgYITgIEpuIBoPgiAAAiAAAhUIwEYmGpUHX0GARAAARAAgYITgIEpuIBoPgiAAAiAAAhUIwEYmGpUHX0GARAAARAAgYITgIEpuIBoPgiAAAiAAAhUIwEYmGpUHX0GARAAARAAgYIT+P++NmQLvggLjwAAAABJRU5ErkJggg==","height":120,"width":200}}
%---
%[output:7a92d0b3]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQvYl1O6\/+\/IIYqt0kwpNJRTI4eQcho5tKVtjEyJ0U4bQw5DSWlfsTGdlFPsNqlktpQzOYXkmCiHnAYRRjIoLoeRlPyve81e7\/95n\/d3WPdvrfX81rN+3+e6XOrtXve6n8+9nmd933V6Gv38888\/Ey4QAAEQAAEQAAEQyBGBRhAwOcoWQgUBEAABEAABEFAEIGDQEEAABEAABEAABHJHAAImdylDwCAAAiAAAiAAAhAwaAMgAAIgAAIgAAK5IwABk7uUIWAfBFavXk3Dhw+nOXPmFHXfu3dvGjt2LDVp0sRZCEuXLqWBAwfSihUraObMmdS1a9eKfSd9FXKSjn\/hwoXUv39\/atOmDU2fPp06dOhQcd1ZF5w9ezaNGDGiXrWF8jNu3Di64YYbqFzukvk\/\/fTT6cILL1S+C7ULzlPnzp1Ve+nWrRv17du36O1rxqX4jBkzRvkoFGux+Pn+FyxY4Lw9Zp1H1AcCNgQgYGzooWw0BEwEDN8sd1xTp06l5s2bO7n3LAVMOv48CphyeUrnx1bApIUSi71rrrmGLr\/8clqyZAlp8VGsMfgQMKb35KSBwgkIBEwAAibg5CC07AiU6xiTkdiOlCR9ZS1guG7d6eZRwJgIguQIim1nrwVMUhh9+eWXNGjQIOcCplBrl4zKZPe0oCYQCIMABEwYeUAUVSZQbApBh5UUGunfugtN3RQSOWmRxNMap5xyCg0ePLjBFFLap8nITykxlOx0dQefFDDXX389TZs2rW4KrVB9haZt0iyS9Wh2xWJPi5Fy0zzsT8eQnvZKsk3WlxQASdbsq9hUEf\/8nHPOKTiluP\/++9Pzzz\/foLUWE7USkVhOrAwbNozOOussJZySV\/I+yjFNCrJTTz1V+UuzqPKjiOpBwJgABIwxKhjGTEAiYJKdVaFOXXNKdiyFOvY0T+232ChDubUqlQqYYnlNCopS96lFTKl7TIuYYv7KCbVkuSTfYvegRUGxf9exp\/OfRwFjwrSYjctRxZjfE7i3sAhAwISVD0RTJQKmU0jJTj0pGJKdabKT0B1DoZ+lO3y27dixY930hPZZTlwVGiUqhTE9hcS2hcSKFkwtWrSoi0nfTzImXZZHBtKLgguNXBX6mem0TDGRVEz4JAVMIbGiY2cGehF3oVz6mkIqN91VblRGLyo3ZSoVgFV6HFEtCBgRgIAxwgSj2AmYCphCoy\/pzrPUb\/PFdgIxX\/bNV6GdQXpUptQIRbldSGmhkhzpSd5XuXU5xaYptIDhekrFWWhdCZfRPy83lVTqPtOjVFoAFBsB0j\/nnWV5FjCmTO+\/\/361e6vcaF7szzvuLw4CEDBx5BF3YUnAVMBwNaW2veowkr85jxo1ii699FK1viQ97ZEWCx999FGD7cHJWyvV8ZQTMOm6i63PKCRgSi2eLTSKkYw5vU6m3LROuWkk7dtkNKbUNmTuyGMRMKZMH3vsMdW+TBlbPlYoDgJeCUDAeMUL53khYDJNk\/5tfsqUKUXPGKm2gDFZ02AqYHgKSZ9Vo0eK9DkoLMrSIybFhIWOybSzlW5VLzRNF4qAMRnxMJ0uKmVX7HnTggUCJi9vJMRpQgACxoQSbKInUImAKdYZ+JhCMklAuamftA9TAaOntZK\/tRdaA1PogL9CdnoaQzoKkBRGhaaZkqNE5UbJ0lMuvqeQfAuYYlNI6Zyb2pm0N9iAQLUJQMBUOwOoPwgCkikk3XkuX768bmRCsoi32K6dYot4GZDJeSa+BYwefeHTgpNiQfMotr4iPXK1atWqOm7FdgHpk3DTjaPc6A3bJ8VCrYzAFFrEW0iUQ8AE8bpBEI4IQMA4Agk3+SYgETCm26iTaz8k26hLbVkuNTXkS8Akd0YVyrIWMMxQH\/BWyC7Jo5gQKTdSYcKx3M4ejs1mBKbY5wUKfQbC5TkwesdRun3o+zVhCgGT7\/cUoq9PAAIGLQIEinzzJg2mWOda6UF2PIXCIw1Dhw4te5BduY6dY\/UlYLhjTt8ji5Htttuu4I6pQh1pIeGVXhgsmVIqJPIKMfIxAsOs07EX+6SADwGTFnHJ6bRyTCFg8LqLiUCuBUzyJWbygpf85hRTknEvIAACIAACIBAbgdwKGBYvs2bNqvuwXvrv6URp8cI\/10Ox+rcVkx0bsSUe9wMCIAACIAACeSaQSwGjh1D79etX9yl7LVCKfd6eh8D5WyLjx4+nDh061OWMh5hXrFiBz9LnuRUjdhAAARAAgZojkEsBU0yM8CjMggULRGKkkjI110pwwyAAAiAAAiAQGIFcChie+uGRk6lTp1LysKty00jFppV4\/UyxbZuB5QvhgAAIgAAIgAAIEFFNCxisgcEzAAIgAAIgAAL5JFCzAkZvC+UtiBh9yWfjRdQgAAIgAAK1S6AmBYxUvPzqV7+q3RaCOwcBEAABEMg9gWXLluX+HtI3kEsBY7OIVypeGBgLmBiTL2nNoTLIMi4fdbnwaeNDWtaHvdSnpN3myTZUDlnG5aMuFz5tfEjL+rCX+szLc5NLAVPJNmpOSCXiBQImL00ZceaVQKwv17zmA3HHRyDWZyyXAoabF+84mjRpEk2fPl2d61JuB5IWPXwsunTNS6zJlzymH3zwAbVv315SJBPbLOPyUZcLnzY+pGV92OP5+uejImWbyQOWcVw+GLjwaeNDWtaHfazPWG4FjBYxI0aMUM9xse+g8L+xYCn1gbxy32CJNfmSF6D0oZL4trHNMi4fdbnwaeNDWtaHvdSnTXsJuWyoHLKMy0ddLnza+JCW9WEfax+WawGT1cso1uRnxQ\/1gEApAtIXNmiCAAjICMTah0HAGLSDWJNvcOswAQHvBCBgvCNGBTVOINY+DALGoGHHmnyDW68zCbWTyTIuH3W58GnjQ1rWh73Up6Td5sk2VA5ZxuWjLhc+bXxIy\/qwj7UPg4AxeMPFmnyDW4cJCHgnIH1hew8IFYBAZARi7cMgYAwaaqzJN7h1mICAdwIQMN4Ro4IaJxBrHwYBY9CwY02+wa3DBAS8E4CA8Y4YFdQ4gVj7MAgYg4Yda\/INbh1rYBKQfHS0Lnza+JCW9WEv9Slpt3myDZVDlnH5qMuFTxsf0rI+7GPtwyBgDN5wsSbf4NYhYCBg6jUTHy9XqU9Ju82TbagcsozLR10ufNr4kJb1YR9rHwYBY\/CGizX5BrcOExDwTkD6wvYeECoAgcgIxNqHQcAYNNRYk29w6zABAe8EIGC8I0YFNU4g1j4MAsagYceafINbxxQSppAwhSR5UCxsQxVyWcbloy4XPm18SMv6sI+1D4OAMXjhxJp8g1uHCQh4JyB9YXsPCBWAQGQEYu3DIGAMGmqsyTe4dZiAgHcCEDDeEaOCGibw97\/\/nQ466CB69913o6MAAWOQUggYA0gwAYEKCUDAVAguwmLc2f7yl7+sd2f8M770z\/Xf+WdJ+0J2LVu2pHXr1tGyZcvop59+os8++4y+++47WrNmjfL5ww8\/qP\/zz3\/++eeSRJP1lkMvsS3ny9W\/z58\/35WrYPxAwBikAgKGKNROJsu4fNTlwqeND2lZH\/ZSnwaPbC5NQuVQLC4tHvj\/+s8sAt5\/\/33629\/+Rl999ZUSC+nOPC1AcpmsHAYNAZPDpLkIGQIGAobbkY8OxoVPGx\/Ssj7spT5dPNMh+vDNodDIxerVq+mZZ55RIxArVqyow6JtQxxJ8Jm79OiProtHcRo3blxXdTG7QrHxKM\/222\/f4J822mgj2nTTTdUIUZMmTWjDDTekb7\/9Vv3\/F7\/4hRodat68ObVp04a+\/PJLatq0KbVq1Yo+\/fRTJQ632WYbWr9+vRKIbJO8OL6ksOzfvz+99dZbPtFVxTdGYAywQ8AYQIIJCFRIwHfHXWFYQRdLi5Hnnnuu3miH7rxCuYl0h1\/o740aNaJ\/+Zd\/oY033pjat29Pm2++eZ1o0B1y+n4K+Sk25aR9SMRHKPxs44i1D4OAMWgZsSbf4NZhAgLeCdS6gElOxWjYvEbjrrvuoo8\/\/lj9KAtBkuzYk3\/m3\/Tbtm1LPGKwww47qHiSgkLbpteo1KJQ8P6wVFhBrH0YBIxBg4g1+Qa3XmcSaieTZVw+6nLh08aHtKwPe6lPSbsNyVaLEF5QytM2fL366qtOQ0yLidatW9Muu+xC7dq1qyc80nalgsgyPz7qcuHTxoe0rA\/7WPswCBiD10esyTe4dZiAgHcC0he294AqrEBPXbAomTt3rhOBkhQa\/OcOHToQv4\/0KEgyVIx4VJi4GigWax+WawEze\/ZsGjFihGp+vIhp+vTp6gE3uZYuXUqjR4+miRMnqoVSpa5Yk2\/CCTYg4JtAngSMHkXhBa+PPfZYxVM7aWFy4IEH0mabbaZQ77HHHr6Rw3+NEYi1D8utgGHxMmvWLJo6daoSIOm\/l2qfvKJ70KBBykSXh4CpsScatxsMgRAFjB5NufXWW2n58uXiqZ6kQGFxkh41wWhJMM2vJgKBgAkozVqA9OvXj\/r27asi4+2Aw4cPp27dutX9rFDICxcuJN5Sxlfnzp0hYAzzGmInw6FnGZePulz4tPEhLevDXurTsMmWNdOLZ3nKhxfLPvHEEyKhokUIj5gcccQRxLto+GeVipNqcSgHKsu4fNTlwqeND2lZH\/YQMOVaeYb\/ztM\/w4YNo\/Hjx9ebMuJRmAULFtDYsWPVvvr0pcXLmDFj1D8lR3AwAlM6gdKHKqvmkGVcPupy4dPGh7SsD3upT5u2xWLl9ddfp5dfftlYrGhR0qdPH7W1V7IAVhJrlhxCjcsHAxc+bXxIy\/qwh4CRtHjPtixExo0b12D0RDKNJLGNNfme0wT3IGBEQPrCNnL6fzt8Pv\/8c3r44YeNxIoWKvvvvz917NjRajTFNEbYgUAWBGLtw3K5BgYCJosmjzpAIBsCtgImeY7KjBkz6JFHHikbuBYrPPXDW41tpn7KVgYDEKgyAQiYKicgWT0ETPbJsO1kfEWcZVw+6nLh08aHtKwPe6nP5Cm0PBJb7iwVLU4GDBigmmGoYkXKwdczlfabZVw+6nLh08aHtKwPewiYrJ4Wg3qqIWCSYc2bN88gSpiAAAiYEOBdPnzSa6lr5cqVtGrVKrrvvvvo7bffLmnLXyDu1KkT7bfffrTTTjuZhAAbEIiOQI8ePerdEx+gGNuVyRQSf6yKP1LFV7Nmzep9FKsSoJUu4k3WhTUwlZBHGRBwT6DYb5z6QLhSU0J6Qe0555yjFu7jDBX3+YHH\/BPACIxBDnl7My+Y+\/777+lf\/\/Vf1W9VixYtUjuGPvroI+WB55t5u3OvXr1ogw02MPDa0MRmG7X2BgFTEXoUAgHnBJIChsUKn2JbbFpIC5YLLrhAHV5Z6ZZl5zcBhyAQMAEImDLJeffdd+mMM85Q53LwxS+XUaNG0TXXXKMETc+ePdXP+QXFw8GTJ0+mAw44oOKUswCZNGlS3em7EkHClUrsY02+BL50Xlbi28Y2y7h81OXCp40PaVnX9ixU7r777rpvAxVqCyxS+MwnnhKKWbBI2do8N5KyWcbloy4XPm18SMv6sI+1D3MyhcRTRLyYjg+C4rNZeP555syZdNlll9Hee++thEbyS6Vnn322+qrppZdeqj6dXulV7lMCHBNfF154YYMqIGBk1KUPlcx75dZZxuWjLhc+bXxIy9ra66P4eZSl2NQQvyt4KujII4+sqSkhKdvKnxpZySzj8lGXC582PqRlfdhDwJRo89988w2dddZZtPPOO6tvE\/GJlHyy5cCBA+nQQw+t+5l2wcJi8eLFdOONN9JWW20le5qqYB1r8quAElXWIAEtWniLc6mpoaOPPpp44WHMoyw1mH7ccgAEYu3DnIzA6DUpXbt2rRvtKPSzpIDhnUQm3yEKIPfqOyYxruAOgS1iiJOAqWjh0dG1a9fSPvvsEycI3BUIBEAg1j4MAsagccWafINbrzORDmtKfNvYZhmXj7pc+LTxIS1bzp6FS7GzWZLnsSR3C5XzadM+8lQ2VA5ZxuWjLhc+bXxIy\/qwj7UPg4AxeMPFmnyDW4cJCJQlUGq7czHRknQqfWGXDQgGIAAC9QjE2oc5FTD8WxUv0OXr66+\/Jj6boUuXLnU\/00R5US+\/9DCFhKcMBPJLgHcP8bOcvkxECwRMfvOOyPNHAAKmRM70epclS5YYZ7Zz584QMMa0YAgC1SfA00N6tKXQYlwWLldddZV4ES5GYKqfW0QQNwEImBL5XbNmDb322mvE\/ze9NtlkE9p9992J\/x\/6FWvyJdxD7WSyjMtHXS582vgoV7bctmcWLbwQt9j3hcr55zZoYiNpq3m1DZVDlnH5qMuFTxsf0rI+7GPtw5xMIeX1hWEad6zJN73\/kDsZ6cMuuee0rY+6XPi08VGq7IoVK2jIkCGU\/HgiM2GxcuCBB9Lvfve7sqMtJrGZ2NjkLS9lQ+WQZVw+6nLh08aHtKwP+1j7MCcCZv369WrNy88\/\/2z8ruCzYrbccsuKPydgXJEDw1iT7wANXERGgKeGip3XokdbXH9vSPrCjgw5bgcEvBOItQ9zImCwBsZ7+0MFIOCNgF7bwsKl0GjL4MGDaccddyw72lJpgBAwlZJDORAwIwABU4JTeg0Mj8Q8\/vjj9OKLL9JJJ51E2223nSrN30Dil+Snn36qvpPEp25iDYxZA6y2VaidTJZx+ajLhc9KfbBYue666+i5555r0LyKjbZI6zKxN7GpdvvPov5QOWQZl4+6XPi08SEt68MeAkbwBPP2yvnz59Of\/\/xn2mKLLeqV\/O6779SnBfQLsnHjxgLP1TGNNfnVoYlaq02g2DQRP5Pnn38+tWvXzttoS6F7l76wq80P9YNA3gjE2oc5mUJKJlN\/F6lv377Uq1evgnl+8MEH6eabb8a3kPL2FCDe3BLg0RYeEeVtzunL19oWU1gQMKakYAcClRGAgDHk9tVXX9Fpp51Gffr0IRYxha5p06bRww8\/DAFjyBRmIFApARYuPG2b\/vKz\/uLzgAEDMh1twQhMpZlEORConAAEjCG7devWqW+hLFiwgK688kraaaed6pX861\/\/qr5cfdRRR9G5555LmEIyBFtls1B\/S84yLh91ufBZyEepaSI+t0XvJJLW78Ne6rPKj4K36kPlkGVcPupy4dPGh7SsD3sIGMFj+\/nnn6vPCLz88svUqVMn2m+\/\/VTpF154gd544w3ir1ZPmDCBWrVqJfBaPdNYk189oqjZB4Fyu4mSwsVH\/ZX6lL6wK60H5UCgVgnE2oc5XwOjG8g\/\/vEPuuuuu+jee+9VooUvFjMnnHCCGn3ZfPPNc9OWYk1+bhKAQEsSYOHCIuCiiy6qZ8fTRPzLQ79+\/ao+TVTqBiBg0MBBwC+BWPswJwKG173Mnj2b9tlnH9p1112pSZMmfrORsfdYky\/BGGonk2VcPuqy9Tlx4kR64IEHGgiX008\/nQ455JCyKZbW78Ne6rPsTeXUIFQOWcbloy4XPm18SMv6sI+1D3MiYFavXq0WCt5xxx308ccf0957762OGe\/evbv6zW+DDTbI6Svln2HHmnxJUqQPlcS3jW2WcfmoqxKfputbTLhK6\/dhL\/Vpcl95tAmVQ5Zx+ajLhU8bH9KyPuxj7cOcCBj9suAD7D777DN66aWX6L777qOnnnpKnQPzb\/\/2b2raKK+jM7EmP48v+VqO2aVwCYmj9IUdUuyIBQTyQCDWPsypgEkn8scff6TXX39dncr76KOP5nZ0Jtbk5+HBQ4xELFx4Z1+hY\/75XBce5czzBQGT5+wh9jwQiLUP8ypgkonVozO8vZrPpOB1MzfccAM1b9480\/wvXLiQ+vfvX1fnzJkz1a6oUlesyZeAD7WTyTIuH3UV86l3FLFwSV4sVrp06UInnnhinXCxiUta1oe91Kek3ebJNlQOWcbloy4XPm18SMv6sI+1D8tMwOgXCa+XeeaZZ5SI4U8KbL311pm9Y1i8DB06lKZPn04dOnSg9N+LBRJr8iXgpQ+VxLeNbZZx+airkE8W+SNHjmwgXIp9VNEmLmlZH\/ZSnzbtJeSyoXLIMi4fdbnwaeNDWtaHfax9WCYChkdfeHHvbbfdRjzi8e2331Lnzp1p6tSpmY3AsHAaPnw4tWnThvg8DH3p33CTP0u\/5GJNfsgv81qM7S9\/+QvxKdXpEZdQz29xlSPpC9tVvfADArVCINY+zKuA4VN5+fsr\/MVbHu3gq3Xr1nTmmWeqhb3NmjXLrP18+eWXNGjQICVeklNGHBeLmFJiKtbkZwYfFRUlUOqo\/9iFi4YCAYMHBAT8Eoi1D\/MiYL744gu655571NbqTz\/9tC4zfGZF7969q\/L5gKVLl9KwYcNo\/PjxavpIXybTSLEmX\/LIhNrJZBmXy7pYuLBw5gW6tiMuNnFJy\/qwl\/qUtNs82YbKIcu4fNTlwqeND2lZH\/ax9mHOBAyPtixevFgNgT\/55JPEf+eD7U4++WTabbfd6Lzzzmsw+pHlywUCxo629KGyq828dJZx2dbFokWPuLgQLi5GMKT35MNe6tO8deTLMlQOWcbloy4XPm18SMv6sIeAKfEu4B1FZ5xxhpouYrHCh9gdffTRdQt0i03fZPl6sRUwyVjnzZuXZeioK+cEVq5cSatWrVLTlPzn5LXzzjvTKaecQi1btsz5XVYe\/vLly6lt27aVO0BJEACBBgR69OhR72fLli2LjpKTERgtUNasWUN9+vShww8\/nLbZZpu6E3hjEDAxJj+61hzgDRU6w4W3QvPXoAcMGJD7M1xcIJf+xumiTvgAgVoigBGYEtlev3498QgHf7hR7zJq3769EjO8WHeTTTahU089tapTSFjEa\/e4htrJZBmXaV08TcSCt9BWaB6dPP744+uSYeqzVPZsfEjL+rCX+rRryeGWDpVDlnH5qMuFTxsf0rI+7CFgDJ973q78\/PPP06233qrOe+G1MO3atVNz\/zfddBMdeOCBhp7cmmEbtR1P6UNlV5t56SzjKlcXt3E+dZqnipIXj7jwAnLegZc+NbecTxMSNj6kZX3YS32aMMmjTagcsozLR10ufNr4kJb1YQ8BU8EbIb0bqXHjxuoLuTznz6eJ8t+zvPQpvPr0XZMdSBxfrMnPkn3sdfGOIj6cMS1camUrtE1+pS9sm7pQFgRqkUCsfZiTNTDlGgRPMb3zzjtqWzVvr167di3tu+++NHnyZNpqq63KFXf67\/iUgFOcNetMf5fI1VbomgVJRBAwtZx93HsWBCBgHFHmU3iffvppmj9\/Pl100UWZncRrE36syZcwCbWTyTIurqtJkyZqOrTYxxWlIy4u4rfxIS3rw17qU9Ju82QbKocs4\/JRlwufNj6kZX3Yx9qHZTICk6eXSKFYY02+JC\/Sh0ri28Y2y7h4iij9cUWOvWfPnhXvKHIRv40PaVkf9lKfNu0l5LKhcsgyLh91ufBp40Na1od9rH0YBIzBGy3W5BvcOkyIqNg3in7729\/SwQcfjK3Qlq1E+sK2rA7FQaDmCMTah0HAGDTlWJNvcOs1a8Lnt\/CarWIn5vJuovSOopqFZXnjEDCWAFEcBMoQiLUPg4AxaPqxJt\/g1utMQu1kXMfF61v4sxd6ka4GwGKFD57j6SKXl4v4bXxIy\/qwl\/p0yT8kX6FyyDIuH3W58GnjQ1rWh32sfRgEjMEbLNbkG9x6zQiYhx56iK644ooGSPSJufx\/6YvFhK8LnzY+pGV92Et9mnDNo02oHLKMy0ddLnza+JCW9WEfax8GAWPwpos1+Qa3Hq1JuQ8rHnPMMerMIkwT+W8C0he2\/4hQAwjERSDWPgwCxqCdxpp8g1uPzkQLF1fboKMDVIUbgoCpAnRUWVMEYu3DIGAMmnGsyTe49aimkAqdlss3yKMsV111VdnRFh8drQufNj6kZX3YS31K2m2ebEPlkGVcPupy4dPGh7SsD\/tY+zAIGIM3XKzJN7j13AuYQl+D1qJl0KBB1KlTp7LCRUOQvlhM+LrwaeNDWtaHvdSnCdc82oTKIcu4fNTlwqeND2lZH\/ax9mEQMAZvuliTb3DruTThaSLeAp3+NpEWLj52E+USVCBBS1\/YgYSNMEAgNwRi7cMgYAyaYKzJN7j13JiwaFm8eLH6CnqhLdDdu3enPn36GI+25ObGIwgUAiaCJOIWgiYQax8GAWPQ7GJNvsGtBz2FxEKFp4jmzp3b4MA5Pdoi\/TZRKSY+OloXPm18SMv6sJf6lLTbPNmGyiHLuHzU5cKnjQ9pWR\/2sfZhEDAGb7hYk29w60EKmHJTRHwQ3bbbbut8tEX6YjHh68KnjQ9pWR\/2Up8mXPNoEyqHLOPyUZcLnzY+pGV92Mfah0HAGLzpYk2+wa0HZXLNNdfQvffe2yAm3kXUpUsX6tGjB\/GBc7jyRUD6ws7X3SFaEKg+gVj7MAgYg7YVa\/INbr2qJjzSwtNDN998c8E4WLi4nCKq6s3WcOUQMDWcfNx6JgRi7cMgYAyaT6zJN7j1zKeQWLTwsf78BehCV\/rMliw7Px91ufBp40Na1oe91Kek3ebJNlQOWcbloy4XPm18SMv6sI+1D4OAMXjDxZp8g1vPRMDwQtz58+fT\/fffX3KkpdAXoKUPu+Se07Y+6nLh08aHtKwPe6lPmxyGXDZUDlnG5aMuFz5tfEjL+rCPtQ+DgDF4o8WafINb92bCooXPauH\/FxtpueCCC6hNmzbOF+N6uyk4roiA9IVdUSUoBAI1TCDWPgwCxqBRx5p8g1t3arJgwQK64447SooWXtNSaKTFaSBwFhQBCJig0oFgIiQQax+WawEze\/ZsGjFihGpu\/Jv69OnTqUOHDkbNb+nSpTR69GiaOHEiNW\/evGSZWJNvBOr\/jCrpZHg9C39\/qNQoC4sVPhm3UtFSSVyS+07a+qjLhU8bH9KyPuylPivNX+jlQuWQZVw+6nLh08aHtKwP+1j7sNwKGBYvs2bNoqlTpyoBkv57qZfszNCrAAAgAElEQVTVl19+SfwdHL50+VL2sSZf8kIv91Dp029vv\/12ev755xuchqvrcr3luVxcknssZ+ujLhc+bXxIy\/qwl\/osl6e8\/nuoHLKMy0ddLnza+JCW9WEfax+WSwGjBUi\/fv2ob9++6n21evVqGj58OHXr1q3uZ4VeZAsXLqT+\/furf+rcuTMEjMXbnkdW5s2bR8uXLy86ysLu9XbnSkdZLEJE0RwQkL6wc3BLCBEEgiIAARNQOnj6Z9iwYTR+\/Ph6U0Y8CsPrLMaOHUtNmjRpELEWL2PGjFH\/lhzBwQhM+QSzYHnqqacKHiaXLM1CpVevXupLzzhYrjzXWreAgKn1FoD7900AAsY3YYF\/FiK8tiI9\/SOZRpLYxpr8Ysj1dND3339PkyZNKjm6okdY9t13X\/rNb35T8VoWQfrrmWbZ+fmoy4VPGx\/Ssj7spT4rbSuhlwuVQ5Zx+ajLhU8bH9KyPuxj7cNyOYUEAeP2VcwjK61ataIrrriirFjRguWwww5Toyz6724jMvcmfdjNPTe09FGXC582PqRlfdhLfdrkMOSyoXLIMi4fdbnwaeNDWtaHPQRMQE8+BEzlyWCx0qhRI3U8f7HdQenpIJ4S4u8MtW3bFlNClaNHySIEpC9sgAQBEJARgICR8fJqXQ0Bk7whXrga8rVy5cq68O677z5atWoVvf3222VDbtmypbI58MADqWPHjtSiRQvSPytbGAYgUCEBXgTO4hgXCICAOwL8S2fyWrZsmTvngXgKegpJ7zZasmRJHa6ZM2eqjrWSRbxJ5jGsgdFrVXjh8jPPPKO2LuuflWtfPKrC10EHHUTHHnus+rP+WaGyof6WnGVcPupy4dPGh7SsD3upz3JtO6\/\/HiqHLOPyUZcLnzY+pGV92GMEJqC3gs02an0beRAwSTHyxBNP0KJFi0QiRd8rCxPeDXTkkUfWCZVSYgUCpnBjl75YTB4ZFz5tfEjL+rCX+jThmkebUDlkGZePulz4tPEhLevDHgImsDcCCxDeIaNP35UIEr4Vib3P5OtRkxdeeEFN80hGUZIp0YJkzz33pCOOOEL9E7YwB9ZoEU5BAj6fLyAHARAgivUZC3oKqVzDK\/cpAd5qzRd\/Xyd9+RIwWoCwoOA\/v\/\/++\/Tee+\/VCRPTKZ5C965FCguT7t27U9OmTTPftlwuJ\/h3EJASiPXlKuUAexDwRSDWZyzXAsZXstN+eUErH5H\/i1\/8gp5++mn68MMP69aaVDpiUkqgsFDhUZTWrVvXrUuRTvm4ZhPqA5BlXD7qcuHTxoe0rA97qU\/XbTsUf6FyyDIuH3W58GnjQ1rWh73UZyjPRLk4IGDKESJSB7TZXuvWrVMu+P\/82YP169erPyd\/blsHyoMACIAACIBAIQLYhYR2AQIgAAIgAAIgAAIBEMAITABJQAggAAIgAAIgAAIyAhAwMl6wBgEQAAEQAAEQCIAABEwASUAIIAACIAACIAACMgIQMDJesAYBEAABEAABEAiAAARMAElACCAAAiAAAiAAAjICEDAyXrAGARAAARAAARAIgAAETABJQAggAAIgAAIgAAIyAhAwMl4FrfnjkkOGDKGLLrqIOnTo4MAjXIAACDCB9Bfpx4wZQ3379gUcEAABRwT4YNXhw4fTnDlzlMeZM2dS165dHXn36wYCxpLv0qVLaeDAgcqL\/rCkpUsUBwEQ+D8C\/D2z7bffXokW\/axNmDAhNy9YJBIEQifA3wXkz+PwNwMXLlxI\/MxNnTqVmjdvHnroBAFDpF6Mo0ePpokTJzZIGie0f\/\/+dYlMqlP+7XDKlCnUs2dPuuSSS2j8+PEYgQm+ySPAahCo9BlLxqp\/UzzhhBMgYKqRRNQZNAEXzxj3d7fddhuNHTuWmjRpEvT9cnA1L2D0EDXDSKtOTubQoUPrRlbSf9fZ5YYzbNgwCJjgmzsCrAYBF88Yx13qBV2N+0KdIBAKAdtnLDmNhCmkULJaJo7k6Ernzp3rCRid0DZt2qihNX3x8BpfyZ9BwOQk4QgzcwKunjGsM8s8dagwJwRcPWN8u1oIcf+Wh3UwNTsCo5POiwL5mjVrVj0BUyyRheYIIWBy8qQjzEwJuHrGMPKSadpQWY4IuHrG9C3rX9y7deuWi8XyNStgkm2UFzGlBUwxUVJoGgkCJkdPPEKtCoFKnzEOdsaMGTRy5MhczMlXBS4qBQEiqvQZe\/nllxU\/vVA+T8shIGAsEq+3TEPA4P0BAqUJVPJynTx5Mt1000112zt1DXmao0e7AIGsCFTyjPHO2bZt22IbdVZJ8lFPpYnHmS8+sgGfMRLAMxZjVnFPIRGoxWcMIzAORmBCasSIBQRCJFCLL9cQ84CY4iVQi88YBEwRASNZxBvvI4E7AwE3BAq9XPGMuWELLyDABGrxGYOAKZJ4yTZqPD4gAALyNTB4xtBqQMAdgUICJvZnDAKmiIDhZqW3qOlFg8UOsnPXBOEJBOIkUOjlimcszlzjrqpDoBafMQiYEgIm+YLVTRI7IKrzcKLWfBMo9nLFM5bvvCL6cAjU4jMGARNO+0MkIAACIAACIAAChgQgYAxBwQwEQAAEQAAEQCAcAhAw4eQCkYAACIAACIAACBgSgIAxBAUzEAABEAABEACBcAhAwISTC0QCAiAAAiAAAiBgSAACxhAUzEAABEAABEAABMIhAAETTi4QCQiAAAiAAAiAgCEBCBhDUDADARAAARAAARAIhwAETDi5QCQgAAIgAAIgAAKGBCBgDEHBDARAAARAAARAIBwCEDDh5AKRgAAIgAAIgAAIGBKAgDEEBTMQAAEQAAEQAIFwCEDAhJMLRAICIAACIAACIGBIAALGEBTMQAAEQAAEQAAEwiEAARNOLhAJCIAACIAACICAIQEIGENQMAMBEAABEAABEAiHAARMOLlAJCAAAiAAAiAAAoYEIGAMQcEMBEAABEAABEAgHAIQMOHkApGAAAiAAAiAAAgYEoCAMQQFMxAAARAAARAAgXAIQMCEkwtEAgIgAAIgAAIgYEgAAsYQFMxAAARAAARAAATCIQABE04uEAkIgAAIgAAIgIAhAQgYQ1AwAwEQAAEQAAEQCIcABEw4uUAkIAACIAACIAAChgQgYAxBwQwEQAAEQAAEQCAcAhAw4eQCkYAACIAACIAACBgSgIAxBAUzEAABEAABEACBcAhAwISTC0QCAiAAAiAAAiBgSAACxhAUzEAABEAABEAABMIhAAETTi4QCQiAAAiAAAiAgCEBCBhDUDADARAAARAAARAIh0DuBMzq1atp+PDhdMIJJ1DXrl3rkRw3bhzdcMMN9X7Wu3dvGjt2LDVp0kT9fOHChdS\/f\/86m5kzZzbwE056EAkIgAAIgAAIgEAhArkTMFqkpIWHFjbdunWjvn37Fsw2i5ehQ4fS9OnTqUOHDkrMJP+OJgICIAACIAACIJAPArkRMF9++SUNGjSIlixZosimBYz+9wsvvLDgiIoWOG3atCG20RcLIr6SP0un7je\/+U3ZbK5bt45++OEH+vrrr4n\/jAsEQAAEQAAEQiGwbNmyUEJxFkcuBIwWJ9tuuy2dcsopNHjwYJowYUI9obJ06VIaPXo0TZw4kZo3b94AUDGBw6MwLGKmTp1asBw72nXXXZXvp556il599VVj+L\/85S+V7cknn0ytW7cm\/rv+mbGTQAx\/9atfUYgPQJZx+ajLhU8bH9KyPuylPgN5JJyHESqHLOPyUZcLnzY+pGV92Et9Om\/cnhzmQsAk752FysCBAxsImNmzZ9OIESPqYUquf+Fyw4YNo\/Hjx6vpI32ZTCOlk\/\/3v\/9dFef\/89qa\/\/mf\/xELG16\/o6e68ipqPLVJuK0xArG+XGssjbjdgAnE+oxFI2B4FGXOnDl161v0lBG3KV7Eu3z5cmcCplQ7ZVHD\/33xxRf00EMPGQsbPTozYMCAIEdqPvjgA2rfvn1wj2iWcfmoy4VPGx\/Ssj7sY325Sh8WKVup\/0rts4zLR10ufNr4kJb1YR\/rM+ZdwKxfv16tC+EO\/fPPP6dWrVrR1ltvTVtuuSVtsMEG4meq2AhMIUdJ2xYtWlgJmKT\/efPmieJeuXKlsl+1ahU9++yz9NxzzxmVb9mypZp6+sMf\/qDs+e\/VulgAtm3btlrVF603y7h81OXCp40PaVkf9lKfwTVCRwGFyiHLuHzU5cKnjQ9pWVf2PXr0qNcyQ1wCYPvoeBEwLFreeecdmjFjBj388MP07bffNoizWbNmdNxxx6lplI4dO1KjRo2M7kUiYPS6l379+tFee+1lJWBcJz85DcULk3ltjcn6Gh6pOfbYYxWzPK+pMUo2jGqCgPQ3zpqAgpsEAYcEMAJjCPNvf\/sbXX755fT444\/TbrvtRqwCd999d9pqq61om222oU8++YS++uorWrRokVoU+\/bbb9Nhhx1GQ4YMMRIylQqYww8\/XO1iSu9SMlnEm2XyWdiwkOHRmsWLF5cVNVrE8H1hLY1hI4VZUAQgYIJKB4KJkECWfViW+JyNwPDWYR5xmTZtmpry4NEVnioqdf3888\/02Wef0Z133qnK\/vGPf1Qio9RVSMAU2yKdXLjL0x98AF4l26irmfzkSA0zMh2lOffcc2n77bd3JmpC7WSyjMtHXS582viQlvVhL\/WZ5Qsyy7pC5ZBlXD7qcuHTxoe0rA\/7avZhPp8hZwKGR1UeffRROuqoo4inh6QXTzPxotdih9Bpf8VGYPQJu\/p8GD19xLt99BkvaRuTHUhcb4jJZ2Hz4osv0vz588uKmuS00x577CFNDexBwCsB6QvbazBwDgIREgixD3OB2ZmAcRGMiY9SU0jpzwScfvrpDQ6oq+RTAnlIvt79ZDJKw4KmV69e1KlTJ4KgMWl1sPFJAALGJ134BoEwfwl3kZfcCRgXNy31kQcBk74nLWh4yolFTamLBU2fPn2oe\/fuzqacpIxhX7sEIGBqN\/e482wI5LEPMyHjRcCkj\/0vFQhvb+YFtrz+hU\/aDfGKJfksZt566y2aMmVKWUEzatQotfBaLwwOtZPJMi4fdbnwaeNDWtaHvdRniO8IFzGFyiHLuHzU5cKnjQ9pWR\/2sfRh6efMi4BZs2YNvfTSS+rL0NxpHn300XXH\/r\/yyis0d+5c2nTTTemII45QW6z571tssQVNnjxZ7UQK7Yox+Xq3E5\/RwycJlxuh4QP2evbsGVpqSPqw29yAj7pc+LTxIS3rw17q0yaHIZcNlUOWcfmoy4VPGx\/Ssj7sY+zD+Fn2ImB4d9Htt99O99xzj\/o2EW+fTl58oN2f\/vQnNW3xu9\/9jr755hsaOXKk+m2f\/x\/aFWvyk5z1lBOfSXPzzTcXTQHniHeZ8bk62LYdWkvNZzzSF3Y+7xJRg0D1CMTah3kRMCxIzjrrLLVQtNiuIv520YMPPkjXXXedGn3hv\/N26jvuuKN6WS5Sc6zJLwXadFEwi5irrroKYia4VpufgCBg8pMrRJpPArH2YV4EjF4Dw19h5lNjC108OnPLLbfUfQX6iSeeoEmTJqlRm9CuWJNvypnFzMcff0x83g5P8+mzadLlWcyceuqpdOihh5q6trbLsvPzUZcLnzY+pGV92Et9WjeaQB2EyiHLuHzU5cKnjQ9pWR\/2sfZhXgTMjz\/+SLwI9MMPP6Rrr71Wff8oPYV0zjnnqIPWLr30Utpoo43USMy7776rRExoV6zJr5SzXj\/Da5dKHay33377qalCTDVVSro2yklf2LVBBXcJAu4IxNqHeREwjJ3FyBlnnEH8XaTjjz9efVaArzfffFNNE61du1YtHuVFu\/\/93\/+tfrNnMVPuIDt3KTX3FGvyzQmUtmQR8\/777ysRWuxiEcMHCuLcGVfU4\/EDARNPLnEnYRKItQ\/zJmA4jfxdpCuuuEKd0MuCha\/GjRurbx\/xsf68bZrXy1x88cX061\/\/mk488UTaZJNNgmsBsSbfB2iTtTNazOBjlD4ykD+fEDD5yxkizheBWPswrwJGp5inlL777jv116ZNm9LGG2+cq+zHmnxJEirpZEzFDH\/8c\/PNN69oqqmSuCT3nbT1UZcLnzY+pGV92Et9Vpq\/0MuFyiHLuHzU5cKnjQ9pWR\/2sfZhXgUMTx\/x0f9PP\/00bbjhhvT73\/9eiReecuApJe608nDFmnwJe+lDlfZtcjIwj8jw2qj27dsbixnbuLJkUKguF\/Hb+JCW9WEv9SnJWZ5sQ+WQZVw+6nLh08aHtKwP+1j7MG8Chkdc+AwYfYx9586d1Y6jRo0a0WmnnaYETaEFviG+cGJNfjVZs6DhnWfFTgWuRMxU835Qd+UEpC\/symtCSRCoTQKx9mHeBAyf68I7isaPH69aDK+FYQHDx9O\/\/PLLNGzYMPrtb3+rzothURPyFWvyQ2HOYuaFF16gq6++umBILGaGDh2qDkTEjqZQsuYuDggYdyzhCQQKEYi1D\/MiYHj05fzzz1cLc1mgcOc0bty4ujNfGPC0adPoySefrDvILuRmF2vyJcyz6mRYzLzzzjt0ySWXFBUzvJtJLwDOKi4OxkddLnza+JCW9WEv9Slpt3myDZVDlnH5qMuFTxsf0rI+7GPtw7wIGH2QHYuYAw88kBYuXNhAwDzzzDN05ZVX1hM1ob5sYk1+qLx1XCxmXnvtNRozZkxJMYOt2aFnsnR80hd2vu8W0YNA9gRi7cO8CBj9KYFDDjmETjnllAYChr+VxGeGvP7660rE8M6kkK9Ykx8y83RsLGaeffZZuv7664uKGXzSIE8Z\/f+xQsDkM2+IOj8EYu3DvAgYFih8SB1\/24gPqOMRGT2FxN89euyxx9Q5MH\/84x\/Vf1gDk58HIYRIWczw4vBHHnmkYDgjRoxQpz9jZCaEbJWPAQKmPCNYgIANAQgYIT1eB8MH1D3wwANqvQJ3OjvuuCPxl6hXrVqlDrP785\/\/TFtvvbXQc\/bmsSZfQjLETobb1OOPP04vvfRSwU8acLvjNVg77LCDk8W\/Phi48GnjQ1rWh73Up6Td5sk2VA5ZxuWjLhc+bXxIy\/qwj7UP8zICo18aa9asUafw8kgMn\/3yww8\/qN+K+\/TpQ0cddRTOgcnR21X6UGV1azoufc4Mj\/QV+tiki08Z+GDgwqeND2lZH\/ZSn1m1razrCZVDlnH5qMuFTxsf0rI+7CFgsn6aA6ov1uQHhNhpKCxg+EOiPJVU6HIhZpwGXOPOpC\/sGseF2wcBMYFY+zBnIzA8ZXT77bfT119\/bQx3yy23VKfzYhGvMTIYCgmwmOG1MvpAxWRxFjLnnXce7bvvvkKvMHdJAALGJU34AoGGBCBgyrQKvXV6yZIlxu1Hn87bvHlz4zKrV69WC4BPOOEE6tq1a71y6RhOP\/109QXk5MVbuvv371\/3o5kzZzbwkw4m1uQbQ\/d0Boqk\/mK2pp1fue8ymYzKmNYluS8XPm18SMv6sJf6lPDNk22oHLKMy0ddLnza+JCW9WEfax\/mbAQm\/aL46quvaPTo0bTLLrvQ8ccfT82aNVMmLEDuv\/9+dUrv2LFj6YADDhC9Y3iNww033EBp4aHFS79+\/ahv375q59OgQYNI\/50rYfHCJ7pOnz6dOnTo0ODvxQKJNfki8BEZazHDoy+Frp49e9KAAQOcLPyNCJu3W5G+sL0FAscgECmBWPswLwJm3bp1ats0b5ku9KkAfQ7Me++9p0RMkyZNyjab9OhKWsDwpwtmzZpV72C85AF6XAeP3LRp06beqAzHyVd6pCYZUKzJLwu9BgxYzMydO5duvvnmBnerR2X0qb81gKMqtwgBUxXsqLSGCMTah3kRMOmTeAu1E96dxGfE8PeRyk0haX\/bbrutOhhv8ODBNGHChHpTP4WEiC7H4qRjx45qRIb\/nJx6KnRKcDreWJNfQ89v2VvNYhdT2SBq1AACpkYTj9vOjECsfZgXAaO\/hbTTTjvRueeeS40bN66XKB6h4eml5cuXi0\/iXbp0KQ0cOLCegNHrYrp166amj\/SVnEbaa6+91Ack+eOSPH2kr\/S0UqEWFWvyJU9PqJ2Mj7hYzDz33HPqtOj05for2S7it\/EhLevDXupT0m7zZBsqhyzj8lGXC582PqRlfdjH2od5ETD80rj77rtp5MiRdNpppylRob8izJ0DT\/fceOONNHHiRHUejOSCgJHQcmcrfajc1Vzak++4+Pwi3sHE\/y8kZnhEz+bEXxfx2\/iQlvVhL\/WZVdvKup5QOWQZl4+6XPi08SEt68MeAkb4NPMoC7\/4ecRj7dq19UpvtNFGNGrUKCVs0qMz5aqploBJxjVv3rxyYeLfIyOwcuVKdYK0nqpM3l7Lli3puOOOUyf+8p9xyQjwSGzbtm1lhWANAiBQkkCPHj3q\/fuyZcuiI+ZtBEaT4t1Ir7zyCr355pvqR7vtthvts88+dbuSpESrJWBiTL6UPez\/SaDUt5hMtmODY30C0t84wQ8EQEBGACMwMl7erAsJGK4Mi3i9IVeOQ+1ksowrXVep7dgsZHj6lI8R0NOnhTLkIn4bH9KyPuylPv229Op5D5VDlnH5qMuFTxsf0rI+7CFgyjzXvGCWF+b+4Q9\/oN133130hWneVv3aa6+pbdBjxowpWVMxAYNt1NV78aLmf47K3HLLLfTwww83wIFRmdItRPrCRnsDARCQEYCAKcOLRcjTTz9Nl1xyCTGsf\/\/3f6cuXbqUPOOFdystWLBAbaXmr1Rz2YMPPrgiAaN3HPEWaV5YWewgOz6FV58hY7IDiYOJNfmyRwDWJgRKbcdmIcPHAPz617\/GIXkJmBAwJi0LNiBQOYFY+zDna2D+8Y9\/qJNup0yZok7d5a3U6TUvvBjy+eefV9MSLVq0oFNPPZVOPPFEo69TFxuB4dTiUwKVN3CUdE8Aa2XMmELAmHGCFQhUSgACRkiOxQuLFJ7a4ZGOb7\/9ts4Di5a9995bfchx\/\/33NzqJV1i9U\/NYky+BFGonk2VcldZValSGdy2dffbZ6qDFUmtlSuWq0rjYp7SsD3upT0m7zZNtqByyjMtHXS582viQlvVhH2sf5nwEptgLY\/369fT999+rUZZGjRrl6b2CKaQKOrqsEix92G3iclGXj7UyNnFJy\/qwl\/q0yWHIZUPlkGVcPupy4dPGh7SsD3sImJCffM+xxZp8z9jgvgSBcjuYzjzzTHVidKWjMnmCL31h5+neECsIhEAg1j4ssxGYEJJYaQyxJr9SHijnlkCtr5WBgHHbnuANBNIEYu3DIGAM2nqsyTe49TqTUDuZLOPyUVfSZ7lRGf6I6Y477thgVMYmLmlZH\/ZSn5J2myfbUDlkGZePulz4tPEhLevDPtY+DALG4A0Xa\/INbh0mVSJQS6My0hd2lVKCakEgtwRi7cMgYAyaZKzJN7h1mFSZQLlRGZPTfqt8C2Wrh4ApiwgGIGBFINY+zLuA4cPq+OwW\/rgjn9C74YYbEn8Yr1WrVrTBBhtYJSWrwrEmPyt+qMcNgVhHZSBg3LQPeAGBYgRi7cO8CRgWLHwGzKWXXqq+Rt25c2d14i5\/ffqss86iNm3a0H\/+539S06ZNg291sSZfAj7UTibLuHzUVYnPcqMyfDDkoYceapReaf0+7KU+jW4sh0ahcsgyLh91ufBp40Na1od9rH2YNwHzxBNP0LnnnkvnnXcetW7dmm688UYlYLbYYgu655576PLLL6chQ4bQySefHPyrJtbkBw8eAZYlwGLmL3\/5Cz300EMNbHkL9lVXXRX8VmzpC7ssFBiAAAjUIxBrH+ZFwPApvMOHD1fTRPxdosWLF6uvRbOAad68OfF3k6677jp6\/fXX6corrwx+FCbW5OMZj4dAqdN++S4HDhxIRxxxRJBiBgImnnaIOwmTQKx9mBcBo79JxEek81A2f0ogKWA4xTxCM2nSpDpRE2ba\/xlVrMmXMA+1k8kyLh91ufCZ9sFiZu7cuXTzzTcXHJXhXyp4dIb\/k9bvw17qU9Ju82QbKocs4\/JRlwufNj6kZX3Yx9qHeREwX331FfHuiOOOO4769etXUMBMmzaNnnzySTUSw9NKIV+xJl\/CXPpQSXzb2GYZl4+6XPgs5qPcqAwLmAEDBlDPnj2NUyCN18TexMY4wBwbhsohy7h81OXCp40PaVkf9rH2YV4EDC\/g5REXniK69tpradmyZfVGYP7617+qhbw8OsO\/DfLC3pCvWJMfMnPE5pYAixn+heGGG24oOCpTze3Y0he2WzLwBgLxE4i1D\/MiYLg5fP7553TOOefQ119\/rU4PffHFF+mYY46h999\/n5555hnafvvt1cu0ffv2wbeeWJMfPHgE6IXAq6++SjNmzCD+f\/qqxsJfCBgvaYZTEKgjEGsf5k3AMLkvvviCJk+eTHfddRd9++23CuZGG21Exx57LPH6mG222SYXTSzW5Evgh9rJZBmXj7pc+KzUB4\/KaDHDfy4kZniEdI899qj7J2ldJvYmNpK2mlfbUDlkGZePulz4tPEhLevDPtY+zKuA0S8S3nXEAuann36iLbfcMjcH2On4Y02+5EUvfagkvm1ss4zLR10ufNr40GXLLfw9\/vjjqVu3bsQ7DCWjpiaxmdjYtJG8lA2VQ5Zx+ajLhU8bH9KyPuxj7cO8ChheC8OjMFtvvbVa5\/LWW2+p0ZiNN95YjcJ07NgxF++WWJOfC\/gIMlMCeuFvqSmmYcOGqbOdeLrJxSV9YbuoEz5AoJYIxNqHeRMw\/FI6\/\/zzlXjhs1748wG8UPC9995T7aZZs2Zqm+eee+4ZfDuKNfnBg0eAVSVgMsXk4qA8CJiqphmV1wCBWPswLwJG70Lis17Gjh1Le++9N\/3v\/\/4vXX\/99ercF17AO2LECNp8883VpwZ4RCbkK9bkS5iH2slkGZePulz4tPFhWpbFzJtvvqlO0C50FVv8a+LfxEbSVvNqGyqHLOPyUZcLnzY+pGV92Mfah3kRMN98843aJn3UUUepc2D0ybz8YmFB06RJE4fdYE4AABWrSURBVPU5gVtuuQUH2eXkbSt9qLK6rSzj8lGXC582PqRl2Z6fX55ieuSRRwqm+YADDlBnQPHiXxP\/JjZZtadq1hMqhyzj8lGXC582PqRlfdhDwAie7PRJvB9\/\/LE6yrx\/\/\/50yimnKE8PPvgg3XTTTU4FDJ89kz7nonfv3nWiievlU4E5Dn3NnDmTunbtWvLuYk2+IKUwBYEGBPQUE5\/6W2xLdq9eveiwww4ruV5G+sJGKkAABGQEYu3DvIzA6BEXPv+FR2IeeOABuuiii9SIC6954Smm0aNHqwW+PCLDU0m2l66Td0r07du3oDsWL0OHDqXp06dThw4dlJhJ\/r1YDLEm35Y5yoOAJmCy+LeYmIGAQTsCAb8EYu3DvAgYTsXdd99NI0eOpHbt2tFnn31G+++\/P02YMEFlafz48TRr1iy67LLLiooNaTr1qA+fW1FoREULnDZt2qjTf\/XFozZ8JX+WrjvW5EsYh9rJZBmXj7pc+LTxIS1ras8jMvxsFTtfhqeXDz\/88Iq+xyRpt3myNWWb9T1lGZePulz4tPEhLevDPtY+zJuA4VGWhx9+mHiKpkuXLnTyySerHUksJEaNGkU777wznXTSSbTJJps4eR6XLl2qRnUmTpyovnidvooJnEIfmoSAaZgS6UPlJKkGTrKMy0ddLnza+JCWldovWrRIHV5ZSszssMMO1KdPn3oH5hmkPjoTKdusAGQZl4+6XPi08SEt68MeAiarp6XCembPnq12NiWv5PoXFjh8fgWP\/vD0kb5MppFiTX6FqFEMBCoiUO7jkuyUF\/7yByaTp\/9WVBkKgQAI1BGItQ\/zNgKjyX333Xf0448\/NmhKPELz7rvvqhdV06ZNrZsa\/4Y3Z86cuvUt6Z1Py5cvh4CxpgwHIOCGgImY4a3ZPLXL\/3d1aJ6b6OEFBPJFAAJGmC\/+mCMv3OWzYIpdnTt3droLKV0Pj7rw7idee9OiRQsrAZP0PW\/ePCGN\/JuzAGzbtm1wN5JlXD7qcuHTxoe0rA\/7N954Q00z3XffffT2228XbGMtW7ZUzzJPQ\/OfY7ykbLNikGVcPupy4dPGh7SsK\/sePXrUayLLli3LqslkVo+XERh9kB3vOuKXDguVG2+8kfbZZx81fcMLfPmFNWbMGOJFtY0aNfJyw3rdC59Fs9dee1kJmBiTL4EunZeV+LaxzTIuH3W58GnjQ1rWh33SZ7ndTNxWeDSGdzR16tQpqqkmKVub50ZSNsu4fNTlwqeND2lZH\/YYgRG0eH2QHQsW3kbNF4uVDTfcUA0J84cdhwwZQqwQi215FlRX1DQpYHjHw6BBg1T9yV1KWMTrgjR8gEDlBIq9sLWYef3112natGlFK8BUU+XsUbI2CEDACPKcPsiOi\/Ii2wULFtQdKsfbqJ9\/\/vl6h8wJqqhnWmyLdHLhLk9\/DB8+XI34YBt1paRRDgTcEzD9jZMFDR+Kyd9WK7Q9W4\/O8CnAfBow1s24zxU85pMABIwgb3oEhod59QgLj3TwFmeeStpqq63U2phJkyY5WwOjT9jVJ+tqEcWjLVqwpG1MdiDxbceafEFKjY6El\/hzZWva+bmoz0ddLnza+JCW9WEv9cm5NJlq0oKGPzip\/+yiHfjyUQkHX7Ek\/WYZl4+6XPi08SEt68M+1j7M6xoY\/m2Jp45YsPBoyLnnnqv+vvvuu9N1111HTz\/9dJ2gcfEgpj8TcPrppzc4oA6fEqiMtPShqqwWeaks4\/JRlwufNj6kZX3YS32mW4kejXnttdfU+6XYxSMy7du3p9\/\/\/vdBrp2x5SB\/esxKZBmXj7pc+LTxIS3rwx4Cxqyt11nxFukzzjhD7RqYPHkybbrppmoKh6eReBfB+++\/TywwWNQ0btxY6D1b81iTny1F1AYChQlIX9jlOEpGZ7p3764O0cN0Uzmq+Pc8E4i1D\/MyAqMTzdvBeAEeL9bdeOON1bAvn9fC00c8T3322Wer0ZnQr1iTHzp3xFcbBFwLmEIjNJ9++qn6Fluhj05qexYxxxxzjDolHGfP1Ebbq5W7jLUP8ypgYmkcsSZfkh\/fnYwklqRtlnH5qMuFTxsf0rI+7KU+K20rXE5PN7311ls0ZcqUoouB2ZZFDB+0eeSRR2YiaLLkIGGYZVw+6nLh08aHtKwP+1j7MK8C5pNPPlFTRjwSU+jacsst1Xy0i5N4JQ+k1DbW5Es4SB8qiW8b2yzj8lGXC582PqRlfdhLfdq0l0KjM\/wzXo\/HU92lLj0qo08HdhkH+6omh1L3kmVcPupy4dPGh7SsD\/tY+zBvAubZZ5+lwYMHqzNfil2+T+J19YKJNfmu+MAPCNgQkL6wbeoyKcujNE899RTde++9JUdo9CgNnzj+008\/Bbkw2OR+YRM\/gVj7MC8CRp\/LsmLFCrXmhVf++zptN4umF2vys2CHOkCgHIHQBEwyXj3lxJsO7rzzzpJraLSg4ZEa\/iAl1tGUyzz+PSsCsfZhXgSMPoPl5JNPpmOPPTarHHmrJ9bkS4CF2slkGZePulz4tPEhLevDXupT0m5d22pBw\/+fMWNGWUGjRc0hhxyiFgjrvxeKK1QOWcbloy4XPm18SMv6sI+1D\/MiYPgL1Oeff773TwW4fjkV8xdr8iX8pA+VxLeNbZZx+ajLhU8bH9KyPuylPm3ai4+yets2LwxetGiRsajhQzYPPvjgupGaUDlkGZePulz4tPEhLevDPtY+zIuA4ZcEf7CRh1yvvvpqatWqlY\/3RmY+Y01+ZgBREQiUICB9YecBJosa3rI9d+5cI0GjR2a6dOmifvHD9FMespyfGGPtw5wJGB51uf322+nrr79WWf3xxx\/p0UcfpR9++EFtQ2zWrFmDbGMXUn4eAEQKAr4IxChg0qwqmXrSoobFzJlnnln3DsWhe75aYrx+IWDK5Fave1myZIlxK8AuJGNUVTcMtZPJMi4fdbnwaeNDWtaHvdRn1R8GhwHokRp+bz7yyCPGnvUIzUknnUTbbLON15OEs8yPj7pc+LTxIS3rwx4CxvjRis8w1uRLMiV9qCS+bWyzjMtHXS582viQlvVhL\/Vp015CLsscmjRpUhcib+Xmb7eVOj04fT9a2PDnEXbYYQcnwibL\/Pioy4VPGx\/Ssj7sY+3DnE0hJR8knj7iKSX+fEDoh9SZvNBiTb7JvcMGBHwTkL6wfccTov\/kFNTjjz9OfEioVNjwfWGNTYjZ9R9TrH2YUwGzbt069b2Ra665pu4Au8MOO4wuvvhiNcyZ1yvW5Oc1H4g7LgIQMHb5TE5D6T9LPOo1Nfvssw\/179+\/rijW2kgohm0bax\/mVMA89NBD9Kc\/\/Yl22203OuCAA2jVqlV011130d57761EDX+ZOo9XrMmX5CLUTibLuHzU5cKnjQ9pWR\/2Up+SdpsnW5cckiM2H330ET355JOiERvNjUUM\/2Latm1b9fHdzTbbzKvAcclAB+rCp40PaVkf9rH2Yc4EzJo1a4iP1OZPB0ycOLFuxTx\/eZo\/KTB9+nTicw\/yeMWafEkupA+VxLeNbZZx+ajLhU8bH9KyPuylPm3aS8hls+CQFDbM4rHHHiM+MV0yHZVkqEdp+P+9e\/em5s2bW20B98HAhU8bH9KyPuxj7cOcCRi9C4lFCn\/sTF\/8GftBgwapo7X79u0b8vujaGyxJj+XyUDQ0RGQvrCjAxDIDbG4YSHCYob\/zJ+E4Y9c6oP6Kg1TLyzmZQS9evUi\/mWXL5x1UylReblY+zDvAqaYsJGnoHolYk1+9YiiZhD4\/wQgYPLTGpJi5rnnnqP33nvPWuBoMaP\/361bN9p\/\/\/1p5cqVCswee+yRH0CBRhprHwYBY9DgYk2+wa3XmYTayWQZl4+6XPi08SEt68Ne6lPSbvNkGyoH07j09BQz5z+zuGGRw1elU1SF8qdHbjbaaCPq1KkT7b777vW2i6cXH5vGX6qt2PiQlvVhH2sfBgFj8IaLNfkGtw4Bk4AkfbGY8HXh08aHtKwPe6lPE655tAmVg+u49CgOC4133nlHnXWjf5YUQS5ymFyEzH\/edttt6aCDDqIvvvhCNLpjw0Ba1od9rH0YBIzBUxJr8g1uHSYg4J2A9IXtPSBUUFUCei1OUuh8+OGHaj0Or6l0PaKTvtnkwmT+t3bt2tF2222nDgbkKz3Ck4ft5rH2Yc4FTOifEmC1nzzrYObMmWV3R8Wa\/Kq+pVA5CPwfAQgYNAUbAumpK\/77+vXr6Y033lCCx3YRsiS2pJjRf+bFy9yH7LfffmrkpxoCKNY+zJmASX\/M0STpWX\/MkcXL0KFD1ZbuDh06qKHL5N+LxRxr8k1ypG1C7WSyjMtHXS582viQlvVhL\/Upabd5sg2VQ5Zx+agr6TM5usNtQ4sfPuvmzTffrCd2shQ+xUaB0iM+bdq0UYfC7rvvvuqUe45x7dq1apRI3xsLJP1n7TfWPsyZgAn9RcFbAocPH07cAJLbvMeNG6dCT\/4sfS+xJl+SMx8vFkn9xWyzjMtHXS582viQlvVhL\/Xpot2E6CNUDlnG5aMuFz4XLVpEfFJx+hwdbkf8izh\/t+qzzz5r8O+u1\/TYtNv58+fbFA+ybM0IGL2dm4VK8kA9HoVhETN16lR1CFOhCwImyLaLoCIhgOcrkkTiNooSSI\/8JEd\/+M88AsQHCvLOLR5RSQslF0IIAibHDXTp0qU0bNgwGj9+vJo+0pfJNBJesDlOPEIPngCer+BThAADIqDFjJ4q0qGlRQ4Loe+\/\/554BOq6666jxYsXB3QXbkKpmREYCBi7BhNqJ5NlXD7qcuHTxoe0rA97qU+7lhxu6VA5ZBmXj7pc+LTxIS3rw17qM9ynpH5kEDAGC3k5+bhAAARAAARAIK8Eli1bltfQi8YNAWMgYKLLOm4IBEAABEAABHJOoGYEjM0i3pznGOGDAAiAAAiAQHQEakbA2Gyjji7ruCEQAAEQAAEQyDmBmhEwnCd9Cq8+fddkB1LO84vwQQAEQAAEQCBKAjUlYJIiRmfT5FMCUWYeNwUCIAACIAACOSZQcwImx7lC6CAAAiAAAiAAAv9HAAIGTQEEQAAEQAAEQCB3BCBgcpcyBAwCIAACIAACIAAB46AN8BbtIUOG0EUXXVTvMwUOXMMFCNQ0AX38wZIlSxSHMWPGUN++fWuaCW4eBFwS0Dt058yZo9zmaV0oBIxlS+BPFAwcOFB5mT59OgSMJU8UB4EkAf7Q6vbbb69Ei37WJkyYUO+DrCAGAiBQOYHZs2fThx9+SPyhY5OPG1dek\/uSEDBE6sU4evRomjhxYoMvUuut1xp9Up3yb4dTpkyhnj170iWXXNLgQ5Hu0wWPIJBPApU+Y8m71b8pnnDCCRAw+WwGiNojARfPGPd3t912G40dO5aaNGniMVo3rmtewOghasY5derUegImfU5MsXNjin0o0k2K4AUE8k3AxTPGBEq9oPNNCNGDgB0B22csOY2EKSS7XGRWOjm60rlz53oCRnJyLwRMZilDRTkj4OoZwzqznCUe4WZGwNUzxgEX++ROZjcjrKhmR2B00nlRIF+zZs2qJ2Ak306CgBG2OpjXBAFXzxhGXmqiueAmKyDg6hnTVetf3Lt165aLxfI1K2CSbYUXMaUFTDFRUmgaCQKmgicPRWqKQKXPGEOaMWMGjRw5Mhdz8jWVVNxsUAQqfcZefvlldR96ofywYcNys54TAoaIKk18hw4dVOIhYIJ6jhFMgAQqecYmT55MN910E+ntnfq28jRHH2AqEFKkBCp5xnjnbNu2bWn48OF1z1meni8IGAcCJtLnAbcFAs4IVPpy1b8kOAsEjkAgUgK1+IxBwEDARPo447ZCIlCLL9eQ+COW+AnU4jMGAVNEwEgW8cb\/aOAOQcCOQKGXK54xO6YoDQJJArX4jEHAFBEwkm3UeIxAAARKEyj0csUzhlYDAu4I1OIzBgFTRMBws9Jb1PSipmIH2blrgvAEAnESKPRyxTMWZ65xV9UhUIvPGARMCQGTfMHqJpmnFdrVeYxQKwg0JFDs5YpnDK0FBNwQqMVnDALGTduBFxAAARAAARAAgQwJQMBkCBtVgQAIgAAIgAAIuCEAAeOGI7yAAAiAAAiAAAhkSAACJkPYqAoEQAAEQAAEQMANAQgYNxzhBQRAAARAAARAIEMCEDAZwkZVIAACIAACIAACbghAwLjhCC8gAAIgAAIgAAIZEoCAyRA2qgIBEAABEAABEHBDAALGDUd4AQEQAAEQAAEQyJAABEyGsFEVCMRM4Oeff6aHHnqIrrzySvrggw\/oqKOOonHjxtHmm29On376KV1wwQV08cUXKwQDBw6ks88+m\/r27dsAif5GEv\/D2LFjqUmTJjFjw72BAAhUSAACpkJwKAYCIFCfwBdffEH\/8R\/\/QS1btqQTTzyRWrRoQbvtths1btyYnnrqKbr11luVuGExAwGD1gMCIGBLAALGliDKgwAIKAJLly4tKEx4ZOaKK66gzTbbjM4666yidhojRmDQoEAABEwIQMCYUIINCIBASQL8IbkRI0bUs9EfPv3mm2+UcDnzzDOpa9euYgFz\/\/33N\/CtK2rTpg1Nnz6dOnTogAyBAAjUGAEImBpLOG4XBHwQ+Pjjj2nBggU0YcIEOvbYY+mggw6inXfeWU0nvfHGG3TZZZfR1VdfTa1btxYLmJUrV9JHH31UL2yeruL1Nd27d6f\/+q\/\/oqZNm\/q4LfgEARAImAAETMDJQWggkCcCxaaQeHTmlVdeoUsvvZQ23njjOgGzYsWKkrfXu3fvgot4v\/vuO7UY+JNPPqFrr72WWrVqlSdMiBUEQMARAQgYRyDhBgRqnUAhAfPjjz\/SqFGjaM8996zbcaTtdtllF9p1110bYFu7di09\/vjjxP+e3oW0bt06mjRpEs2aNYumTp1KnTp1qnXsuH8QqFkCEDA1m3rcOAi4JVBIwPDU0tChQ+nyyy+vW6dSbKRGR1NsES8vBr7zzjuVIOLRnD59+lCjRo3c3gS8gQAI5IYABExuUoVAQSBsAoWEyRNPPKFGS3j7tF6nUqmAeffdd+mMM86go48+Wp0hw9uzcYEACNQuAQiY2s097hwEnBJIC5P09mldWSUC5vPPP6dzzjmHtt56axozZgwW7TrNHJyBQD4JQMDkM2+IGgSCI5AWJunt05UKmJ9++kkt2n311Vdp8uTJ1LFjx+DuHQGBAAhkTwACJnvmqBEEoiSQFjDp7dOVCBheOzNt2jS65ppraPDgwbTffvs1YLfddttRu3btomSKmwIBEChOAAIGrQMEQMAJgbSASW+frkTA6AW7c+bMKRojTykV+qaSk5uCExAAgWAJQMAEmxoEBgIgAAIgAAIgUIwABAzaBgiAAAiAAAiAQO4IQMDkLmUIGARAAARAAARAAAIGbQAEQAAEQAAEQCB3BP4fwYxy7G8KkQ8AAAAASUVORK5CYII=","height":120,"width":200}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10Xkd55x\/atERLCLaSUEcRxqkj09BQOc6GCmEw1Jt6264cDm1Xks+2rBCsW2I4u4lXkgnkNCTEktZOmyahuKnQprtY9rYkYJdCABMoqXAJTqyWEhIniu3IismHYxKIAw14z3OdUUZX92Nm7jzve+\/of8\/JcaT3mefO\/J55Z\/6az1ecPHnyJOEBARAAARAAARAAgQoReAUETIWihayCAAiAAAiAAAhEBCBgUBFAAARAAARAAAQqRwACpnIhQ4ZBAARAAARAAAQgYFAHQAAEQAAEQAAEKkcAAqZyIUOGQQAEQAAEQAAEIGBQB0AABEAABEAABCpHAAKmciFDhkEABEAABEAABCBgUAdAwBOBY8eOUW9vb+RtZGSEGhsbPXme6+bAgQPU09NDl1xyCQ0ODlJDQ4PYu3THtSyjSYEUhw9+8IPU2dlpkqTUNkNDQ7Rt27Yoj+vXr6f+\/n6r\/O7cuZM2bdpEmzdvLiUPLt\/evXvFvx9W0GBcWQIQMJUNHTJeNgK17NzTBAx3EKtWraK2tjYRPGlllH5vWmFcOkTuQL\/+9a9biQOXNLYB4HesW7duJlmIAiY0wWkbY9j7JQAB45cnvIFAXQicOHGCBgYGaPfu3bR9+\/aaCRge+anFe5Ogqs6wo6PDWIyoEQobceCSxqUSKAFjk7f4e8o+AqPq6eHDhzEK41JJkGYWAQgYVIhSEtCH0jmD+pC4PvqwfPlyuu6666IycEemT6eo0YKJiYk5n+s+Lr\/8cnrf+94X2TQ1NdHo6Ci1tLQkctGFQtw+PjrBn6sppdWrV9ONN95Ira2tUcOtd\/x5fngqSr133759Uf74UVNI11xzDX3sYx+LxIt64iz494qpLnBUp6nbq05Q+dI7VL2Mt9xyCw0PDye+d2pqKsrf9PT0TJ70GOocmfmtt95Kn\/rUp0iVj\/nHWSt2amouqbNWcdXfq8obL5eKdXNz84wIi\/PbtWtXNCWjHr1+xP3lCcd4fczylVUPs0ZqdCacZ5X3uCiKf790tsrHlVdeSXv27CH+\/qjY6WXm36l36LHN45JUD0vZCCFTpScAAVP6EM2vDMY7Lb30qhFO6qTiHQ\/7YfGgxEv886QONqvz58\/S8qY6m7POOmvWGhglYPQ8sFBIEhy6iIn78SVgkv7Cj3cm8Y4tjSv\/Pk3A9PX10YYNG+awzxIM\/Nk555xDTz75ZCTQkkQFv1PvaON5T6sX6r333Xdfohi54447Ztad6PVN76DjAibuS32eJmKy6iynOXToUKpQ0vMUFy9xkanEw9ve9jb6xje+MavxSBIhSd+vuABhm6Q88u\/Ve\/J861zKPko0v1rcapcWAqba8Qsu96qB1v8CVY0\/F1YffeC\/slXDqHcQemOr\/+Wpd3gsEtQIgfKh3h3\/S19BTvpcb4wvu+yyVAGj\/4Vq6ydPwPCoEz95UzlZI0Q8KvT000\/PYaKPGjCnZcuWzSqjyRRSfHpLsVfx5NGWeNw5L7weJGlkiFmuXbs2Kq8+YmOysNlkOihuE\/9ZMVFii\/Of925V95LKo37HQpfLnDaFpHNU9Ske0y9\/+cuREEoaUUnzGx+FU6NOuo+sd6sRGlX\/87j4mCoLruFDgZwIQMA4YUMiKQJpf52pDoAb7hUrViTuwNFtDh48mPhXNedb98F\/9asdQ3mLcPP+ckwTCHqDzu+39eNLwOjvZjHCj95hpnUsegf+\/ve\/31jAJI1YJb2X8xGfIksb4WBb7ohVPnS2Se+LT6VlCZi0qbN4mqzRlCTxGy+bmp6MCyEl2tKERl791OOr+0iLa3w0R7FSAiZt6lDfYafXZfW91KfvVDuhc0matpRqT+A3bAIQMGHHt3Klq7WA0bch53UQtsKD4Sdtqzb1o3fO8c6OfevbqE1GYNhGX\/jKP\/OW3fgIVLwDtRUwcY7xUZq4cPIlYFRlTxNOvDMrScDEp6LyRmCqIGCSRvxUXOP1L20ERveR9t2AgKlcExtUhiFgggpn9QvjOoUUn+pQawrS\/ppNGvLPEzBZUz\/6qABHgf9KTRMwpn54aD4uLtTUWlzAsEgwWRwZ79z1EYr4NBx3+HlTSDw6lCcA4j5cp5D02p02qhH\/BsTFSHw0QpVZH4lT5VF1J54maQop75snPYWkxK4auUoTMEkjV4pRfAQmbdF1fPoqawopiQumkPJqCz43JQABY0oKdjUhIL2IN0sA5AmYrLwlrQ9JEzB5fni4Xa1niUM3ETCcJmkXkvIV30miHwBns4hXTSXoafi97373u6PRoaSHOSWVz3QRL\/tUoi4unNIWuOppdBt+50033UTXX3\/9nAXHnCYuYPh3aQuCVVnzBHPS9EreCJjOMa2MWeJDFwwf+tCHUutWlg\/OQ9LiXtNFvDqXvBHImjQ0eEkQBCBggghjeIUw3Uatb4HO20adtDDYZgqJKWdNT+QtktVP5s3yw++Jb7n96Ec\/Svv3759ZtJo0AqN3bmkLkXXf8bU5SQJH78j1tPz\/SsAkvfe2226bdaIsH66nr7dx2UatCxG9Q03aYp+2fTvOVV+Twz6Z25YtW2jjxo0RDn0kTe0mS9uWnXd+S9Y2an6X6chE2toVHoVLEgdpo07MKGkLe9IoTpr45d\/HT\/5NW0ukfJiMFIbXoqFEEgQgYCSowqcogbwdH6Ivh\/PCBJKmquI7zdLO4dFf7nKQXeHMz1MHuuBUQi1pZ1IeHhxkl0cIn9sQCFLAcMPGZ1HwIVtJDWHWAWc28GBbHwIQMPXh7uutWVNoWVNfSe93uUrAVznmm5+kKSRmkHf4Y5LoDOXuqvlWB8pW3uAETN7iPvV5e3t7dNmZ+pm\/hLYXp5UtmPMlPxAw1Y90\/I8ILpGteOE0uFuntnUhPrVrI144pxCctY1X6G8LTsDwfC9\/SfhJG4GJB5X\/shgfH6\/prb6hVyyUDwRAAARAAAQkCQQlYPivumuvvZa6u7sjEQMBI1l14BsEQAAEQAAE6kcgKAHDIyn88ImQWWtgdNxqKLurqyuaUsIDAiAAAiAAAiBQfgLBCBieC7\/99tvp6quvJr6oz0TAqPUvHCb9FuN42H75l3+5\/JFEDkEABEAABEAggQDfKn7++ecHxyYYAcNTRnzWBJ8emrcLiaNoKl7YlgXM5ORkcMGvd4HAVS4CYAu2cgTkPKPeyrANlWsQAiZpR4OqBknX29vuPAo1+DJfFXOv4GrOytYSbG2JmduDrTkrW0uwtSVmZh8q1yAETDyEeSMwPFrDp1BmTRvpPkMNvlnVl7MCV7CVIyDnGfUWbOUIyHgOtc7OCwGjRlx4d9KyZcuiG4LVseCqumQdvR5q8GW+KuZewdWcla0l2NoSM7cHW3NWtpZga0vMzD5UrkEKGLOQmluFGnxzAjKWjz76aJALy2Ro2XkFWzteNtZga0PLzhZs7XiZWi+56M108DvfMjWvjB0EjEGoIGAMIDmYoLFygGaYBGwNQTmYga0DNMMkYGsIysJs7N6jdMXYA3TsxndapKqGKQSMQZwgYAwgOZigsXKAZpgEbA1BOZiBrQM0wyRgawjKwgwCxgJWiKYQMDJRRWMlw5W9gi3YyhGQ84x6658tBIx\/ppXyCAEjEy40VjJcIWDkuIIt2MoS8O8dAsY\/00p5hICRCRcEjAxXdLJyXMEWbGUJ+Pc+dNdBGrrrUayB8Y+2Gh4hYGTiBAEjwxWdrBxXsAVbWQL+vUPA+GdaKY8QMDLhgoCR4YpOVo4r2IKtLAH\/3iFg\/DOtlEcIGJlwQcDIcEUnK8cVbMFWloB\/7xAw\/plWyiMEjEy4IGBkuKKTleMKtmArS8C\/dwgY\/0wr5RECRiZcEDAyXNHJynEFW7CVJeDfOwSMf6aV8ggBIxMuCBgZruhk5biCLdjKEvDvnU\/h5a3UOInXP9tKeISAkQkTBIwMV3SyclzBFmxlCfj3DgHjn2mlPELAyIQLAkaGKzpZOa5gC7ayBPx7h4Dxz7RSHiFgZMIFASPDFZ2sHFewBVtZAv69Q8D4Z1opjxAwMuGCgJHhik5WjivYgq0sAf\/eIWD8M62URwgYmXBBwMhwRScrxxVswVaWgH\/va2+9n+555DgW8fpHWw2PEDAycYKAkeGKTlaOK9iCrSwB\/94hYPwzrZRHCBiZcEHAyHBFJyvHFWzBVpaAf+8QMP6ZVsojBIxMuCBgZLiik5XjCrZgK0vAv3cIGP9MS+PxwIED1NfXR8PDw9TS0pKYLwgYmXBBwMhwRScrxxVswVaWgH\/vy6\/\/Jh0+9gLWwPhHW1+PJ06coIGBAdq3bx+Njo5CwNQ4HBAwcsDBFmzlCMh5Rr31z7bxyrsjpziJ1z\/bunrcu3cvDQ0NRXnACEztQ4HGSo452IKtHAE5z6i3ftnyyAuPwEDA+OVad2\/Hjh2ja6+9lrq7uyMRAwFT+5CgsZJjDrZgK0dAzjPqrV+2EDB+eZbG286dO6O8rFixAmtg6hQVNFZy4MEWbOUIyHlGvfXL9p6Hj9PaT9yPERi\/WOvrjRfu3n777XT11VfT1NSUkYDRc7xnz576FiCQtzP75ubmQEpTrmKArVw8wBZs5Qj48bx69erI0U8Wv5WeX\/FeCBg\/WMvhhaeMVq1aRW1tbYRdSPWLCf7akmMPtmArR0DOM+qtX7ZDdx2kobsehYDxi7V+3njtS29vL01MTMzJxPbt2yNRE3+wjVomXmisZLiyV7AFWzkCcp5Rb\/2yVWfA\/NzzT9FTn\/x9v85L4O0VJ0+ePFmCfNQtCxiBqRt6dLKC6NERyMEFW7CVI+DXs9pCfdpTD9ITf\/1Hfp2XwBsEDA6yq1s1REcghx5swVaOgJxn1Ft\/bPUdSA3\/\/Gk68pW\/8ue8JJ7mvYAxiQOmkEwo2dugsbJnZpoCbE1J2duBrT0z0xRga0oq304XMGfcM0yHv\/WF\/EQVs4CAMQgYBIwBJAcTNFYO0AyTgK0hKAczsHWAZpgEbA1BGZip9S9suuCzvTQ5OWmQqlomEDAG8YKAMYDkYILGygGaYRKwNQTlYAa2DtAMk4CtIagcM330ZeXSBfSdrb8LAeMHbfW8QMDIxAyNlQxX9gq2YCtHQM4z6q0ftmP3HqUrxh6InN3afSFd3dkOAeMHbfW8QMDIxAyNlQxXCBg5rmALtrIEinvn0ZcNYw\/QPY8cp8WNp9P+j7yFQu3DMIVkUF9CDb5B0UVNIGDk8IIt2MoRkPOMelucrT76svGyJfTh3zofAqY41up6gICRiR0aKxmuGCWQ4wq2YCtLoJh3Hn3hu4\/4Xx59uaXrQlp5wQIImGJYq50aAkYmfhAwMlzRycpxBVuwlSVQzPvIPx6h\/\/mZhyIn\/WvOp\/41S6L\/D7UPwxSSQX0JNfgGRRc1gYCRwwu2YCtHQM4z6q07W33qSK19Ud5C7cMgYAzqS6jBNyi6qAkaKzm8YAu2cgTkPKPeurHVt03rU0cQMG48g0oFASMTTjRWMlwxzSHHFWzBVpaAvXd93Qun3vWBi6N1L\/oTah+GERiD+hJq8A2KLmoCASOHF2zBVo6AnGfUWzu2+sgLp9zwjtfRx9ZeMMdJqH0YBIxBfQk1+AZFFzVBYyWHF2zBVo6AnGfUW3O29zx8PNpxpJ4\/bGuiP\/vPb0h0EGofBgFjUF9CDb5B0UVN0FjJ4QVbsJUjIOcZ9TafLY+6fOa+79N1f\/\/y3UZ\/u76VfuMNjamJQ+3DIGDy60uwW9AMii5qgsZKDi\/Ygq0cATnPqLfZbHnUZcOOB6JzXvjhBbu8Xbr70kWZCSFg5Ops6T2HGvx6g0djJRcBsAVbOQJynlFv09kOf+kgDX7x0RkDFi+8YJf\/zXtC7cMwApMX+YAPATIouqgJGis5vGALtnIE5Dyj3s5myyMt9zz8DG3Y8b1ZH4y+5yK6vPUc40BAwBijCs8w1ODXO1JorOQiALZgK0dAzjPq7Sm2LFy++K9P0cCdB2bBthl10ROG2odhBMbguxhq8A2KLmqCxkoOL9iCrRwBOc+ot0Qf3PE9+vS3Hp8jXNS9Ri70Q+3DIGAMakOowTcouqgJGis5vGALtnIE5DzP13rLi3OH73qU7nnkuFfhopyF2odBwBh8F0MNvkHRRU3ma2MlCvUl52ArRxlswdYHARYtn93\/BH1q\/Mgcd0nXARR5Z6h9WDAC5sSJEzQwMEC7d++O4rx+\/Xrq7+9PjfnOnTtp06ZN0ecdHR00ODhIDQ0NifahBr\/IF8JHWnQEPigm+wBbsJUjIOc59HrLa1v+Zt\/36eNfePkMF0WTRctbly6ItkWb7CyyiUKofVgwAmZoaCiKJ4uWY8eOUW9vL3V1dVFnZ+ecOO\/du5fYfmRkJBItLHyamppSBU+owbf5AkjYht5YSTAz9Qm2pqTs7cDWnplpihDZ8kjLl777FN3ytccSMfgebUl6Sah9WDACJh40XdDEP+PRl\/Hx8ZlRl\/jPcftQg2\/aqEjZhdhYSbGy9Qu2tsTM7cHWnJWtZShs09a06KMtN3ddSK9vPN37aAsEjG2tK5m9GoHh0Zi2tjajEZj29vbE0RpODAEjE+BQGisZOsW8gm0xflmpwRZs4wR4auj\/\/tPjtHfy+JyFuLpo+fPOX6ElZzXURLToeQy1DwtuBIZHXrZt25a7ruXAgQPU09ND09PTtH379kShoyoAB19\/9uzZI\/cNnkeep6amqLm5eR6VuHZFBVs51mA7v9lOP\/tiBID\/\/ctvHad9R04d6x9\/ms48jc599Wm0\/tcXRP\/yz7V6Vq9ePedVk5Nz193UKj9S7wlOwChQLGRYnCQtzuUpox07dkRrYBobG6P1MPykLfoNVb1KVSpTv\/hL1pSUvR3Y2jMzTQG2pqTs7crKVq1j2f\/Yc6kjLFxaXs\/S+e8X0dsuWEgrL1hgD0AoRah9WLAChkdY+vr6aHh4mFpaWmaqhdqtpE8ZpdmqRKEGX+i7Yuy2rI2VcQFKbAi2csEB2\/DZsmD5zP3fp7sfPDZzcWJSqdXOoe5Lzy2VYInnNdQ+LFgBo+804lEW9UDAyDU+tp7REdgSM7cHW3NWtpZga0vM3L4ebFmsPPnDn9DoPx7JHF1RIyy81ZkFC4sX39udzUnZWULA2PHKtVYLbScmJnJtdYPW1la6884756TRp4GUSEnbGp00hZQ23cQvCjX4VuAFjOvRWAkUo5QuwVYuLGBbXbY2YkUJlnctfy29t\/28qNBVESwYgZGro5HnvJ1CSa9XoypJAiZ+kJ1+OJ36rLu7e2axrlrsy+\/BQXbCwU5xj45AjjvYgq0cATnPPusti5Vjz\/8b\/dU3pnJHVpQ4WbzwdOp76SC5qoqVpOiE+kd43aaQfAsYua8URmCk2PpsrKTyWFW\/YCsXObAtD1vevsxCg\/\/d+pVD9OiTz1uJld6VzXTWq36hUtNBLvQhYFyoBZIm1ODXOzzoCOQiALZgK0dAznNeveVRlQe\/\/yP63P4njISKPrKy+sKz6JLFZ5Z6sa0U2VD7sLqPwPAamLx7i6SCauo31OCbll\/KLq+xknrvfPALtnJRBltZtj\/\/mnOjEZVvH\/oBffV7x+jwMy9k7gTSc6PvCpoRL42ny2W4Ip5D7cPqJmBU3PW1KLzodnR0dNa25zLUj1CDX2+26AjkIgC2YCtHwI9nFin8fPXBY3THfd+3Fiq8XmXVskb69fNfE\/wUUFHiofZhdRcwKjDxXUllGpUJNfhFvxRF06OTLUowPT3Ygq0cATvPPO3Dzxe+8xT9y5Hsg+DintVCWrV1uUyHw9lRqK91qH1YaQSMHl79mP8yjMqEGvz6fqWI0MnKRQBswVaOwFzPPJrC\/z353E9odPxIZHDPI6eEi8mjhErrL51G1\/3umyJfVTpnxaSM9bQJtQ8rpYDRA512IF0tK0Oowa8lw6R3oZOViwDYgq1vAkqkTD71PP3Toz+gx469YCVSOD+RKFl4Ov3mG8+i5a87c+ZcFSVgUG99R+2Uv1D7sFIKGH0EhuHnXbYoE\/KXvYYafGluef7RWOURcv8cbN3Z5aUMnS1P+fzSmb9IN331MB1++oSTSGGGPO1z1WVLaPr4j41HU0Jnm1e3pD4PtQ8rjYDJOohOKqimfkMNvmn5pezQWEmRxfScHNlqs1VTMyxSnnn+3+iL\/\/qU00iKPpryX9qaqOk1r5wzmuISA7QJLtTy04Tah9VdwOi7kMow2pJUFUINfn61l7VAYyXHF2znN1u1w2fs3qP0jw8\/Y7XDRyenpnzaL1hAK5cujD6SXJuCeitTb0Ptw+omYPRdR3lH+cuE1NxrqME3JyBjicZKhit7Bduw2SqB8sDRH9GuiSecR1H0kRRek\/K+lefVdQEt6q1MvQ21D6ubgHn44YfpQx\/6EF1zzTUz9xPlhS7rLqS8tEU+DzX4RZj4SIvGygfFZB9gW122+jQPl2Ls3scjgWJzoFt8FIV\/XnnBQvqVRa+i5c2vFh1FKUIe9bYIvfS0ofZhdRMwuAtJpqJWySsaK7logW252aodPTwdc8vXDtP3Hv+Rs0DRR1F+9bwz6HcuOicqfBXPTEG9lam3EDCeucYPrjN139raSkm3UZumd7ELNfguLHymQWPlk+ZsX2Bbf7a8UPZ1LFDuPkwPHi0uUCJRsnQh\/ff\/sJiO\/uAnXhbNylFy84x668YtL1WofVjdRmDygJfp81CDX2\/GaKzkIgC2smz5vh5+fvziz+jmuw\/TwadOFB5BOTVqspCWntNAl77+NZF\/yQWzcoTcPaPeurPLShlqHwYBY1BfQg2+QdFFTdBYyeEF22Js1RTPl777FO1\/7LnImc3JsvG360fi\/8YbGunSJa+JFstWcZqnGNns1Ki3MnRD7cMgYAzqS6jBNyi6qAkaKzm8YJvOVomTpgWvpD\/9yiE69HSx0ZOZkZKFp0dTRpe3vjZaLKseJV7koh2OZ9RbmViG2odBwBjUl1CDb1B0URM0VnJ45ytbtb2Y\/\/3Gw8\/QN1+6j8fH6Akfgf+m886g5We\/SOcuOnfeTe\/I1daXPc\/XeivNNtQ+DALGoOaEGnyDoouaoLGSwxsa2\/jW4u8+\/kP6u39+svDUjho94X95\/UnzgldG\/+q\/j4+ghMZWrhbaewZbe2YmKULtwyBgDKIfavANii5qgsZKDm9V2CphoqZ1mMhzL7xIf\/cvTxY6+yQ+fcOjJxed92r67YvOnhEnrlM7VWErV7vkPIOtDNtQ+7BSCZidO3fSpk2bogjyBY6HDh2i8fFxGhwcpIaGhszIxu9SWr9+PfX396em4UPx1q1bF33OW7NHRkaosbEx0T7U4Mt8Vcy9orEyZ2VrWSa2+pknX\/ru07R\/6rnokkDXg9l0Fuqo+yVnN1DHr51DDb\/w8+Lbi8vE1rZelN0ebGUiFGofVhoBw3ciTU9PU19fH23YsCESHywsBgYGqKmpKVOMcMg5PT+cTp0x09XVRZ2dnXNqhLrtesuWLdEpwCycsoRSqMGX+aqYe0VjZc7K1rJWbPU1J\/ce\/AHd\/eCxKKtF1pzooyc8crLsl14VbS3+nTedM3PMPdu4jqDYsozb14pt0XxWMT3YykQt1D6sFAJGP5V32bJl1NvbGwkRFhfq+oCsEZKkkOuCJv45C5aDBw\/miiKVLtTgy3xVzL2isTJnZWtZlK2+5uQknaQvfOcp+s6RH3oXJ7xr5\/0rm+mHP\/6p+MiJLcM0+6JsfeUjRD9gKxPVUPuwIAVM1jUFaqqpvb09cXQmqfqEGnyZr4q5VzRW5qxsLdPYxhfDHn32x9GoSZG7dvS8qVERHjlZfNbp1PFrr6ULF70qqDNPUG9ta6O5Pdias7KxDLUPK4WA4UCoaRx9CkmNxqRNBaWNvGzbto3SbrhWAmbNmjV022230cTEBNbA2HwTPNqisfII8yVXar3J40cfp0efb6B7DjzjddSEnbE4eWPTGfQff\/VsOu3nXhEdxqamkuo1reOfZLpH1Fs52mArwxYCRobrLK\/6wlr1webNm41HSnRnak1NfAGwEjCHDx+eWbibZqv8cfD1Z8+ePTWgEf4rpqamqLm5OfyCeirhviMvRJ5++rOT9PcP\/oimn32RHn\/uxejfok\/TmadFLs599Wn0pkWvpPbXN0T\/rx71edH3hJAe9VYuimDrh+3q1avnOJqcnPTjvEReSjMC45sJL9Tl0Zzh4WFqaWmZcZ80hZRmqwuYEIPvm7mtP\/y1dYoYX\/rHz6LX\/CLdcvdjNPnk86d+\/9IhbLZcdXt9SudXm86gVcsW0hvPPaMUi2GLlKueaVFv5eiDrQxbjMDIcBXzmrX4l0dclixZMjOywwLmhhtuoK1btyZupQ41+GLwDR2H3lipaRXGMTp+hPYdejYi42P7MPuZESeNDfSmpjNo\/dubZ4TJY489Rm9tfVm4G4YEZgYEQq+3BgjETMBWBm2ofVgpRmDUoltej5L1ZJ3tou86UqMsaduv4+Ima8cS5yfU4Mt8Vcy9Vq2x0g9dU6X87P4n6CsPPC0jTF5aCLvxsiU09cyPrXbpVI2tea2pvyXYysUAbGXYhtqHlULAcMh4Ee+OHTtmHSinn+eydu3azDNh4gfZ6Yt41Wfd3d3R1mx+9PU2aQt+VVUKNfgyXxVzr2VprJKEybdeOtPE1+6cWSMmL1369wdtTfTiT09aCRNTumVha5rfKtmBrVy0wFaGbah9WCkETNa2Z3205KGHHooOrLvzzjtlopziNdTg1xRiwsukG6v4MfU85fKd6R\/S5z0dU68XSZ0Iy+eaXPy6M6PbiF+e4jm95qil2da8QCV6IdjKBQNsZdiG2odBwBjUl1CDb1B0UZOijZV+fw7vzPl\/+456O89EFVxfBPvONzTSpUteE30UCZbG2gsT04AUZWv6nvloB7ZyUQdbGbah9mGlEDAcsrwpJL4SQJ0Vc9NNN8lEGSMwNeWaddiaWgB71hm\/QJ\/42mN0yNPdOUnC5C1LF9DbtBuIyyxMTAOEjsCqqStJAAAgAElEQVSUlL0d2NozM00Btqak7OwgYOx4OVknnQPDlzqq+4puvvlmGh0dnbUt2ulFlolCDb4lBi\/m+sV+\/\/trD9K3H\/9Z5Nf7zpyFp9Mlrz+TetrPm3XIWgjixCQQ6AhMKLnZgK0bN5NUYGtCyd4m1D6sNCMw9iGpXYpQgy9BUAmUf\/vpz+gz9z\/h9dZhzm90Cuy5Z9AH3vG6eSlMTGOGjsCUlL0d2NozM00Btqak7OxC7cMgYAzqQajBNyj6HBP99uG\/ve\/70aFrRQ9c0xfAvuctTfSTF2V25riUt6pp0BHIRQ5swVaOgIznUPuw0ggYPkyup6eHpqen50SwtbV11vZqmRCnew01+Gkl1i\/8e+Doj2j3xBPOIkUXJ5e3vjbancP++f4cdARyNRlswVaOgJxn1FsZtqH2YaUQMPrx\/uq8Fz6zRV3m2N\/fP3N+i0x4s72GGny91Hyc\/Wfu\/z498oTdiIoSKMtfdyZdeO6r6K1LF8y4zVtvgsZKrjaDLdjKEZDzjHorwzbUPqwUAiZ+Dox+1D8v7B0bG6P4pYwyYU72GmLwx+49SmPfetxoZEUJkTVvPJuueGntCY+gFH3QWBUlmJ4ebMFWjoCcZ9RbGbYh9mFMqpQChrdLHzx4kHjkJetOI5lQz\/Va9eCrhbVj9z5OLFyyHhYrq1oW0u9fskj8rBM0VnI1GGzBVo6AnGfUWxm2Ve\/D0qiUQsBw5vT7iHTR8uUvf5nGx8cxAuNQr1m4bP3KQfo\/ex9PTK2mf27pvjD6PG\/KxyELmUnQWPkm+rI\/sAVbOQJynlFvZdhCwMhwnfGqr4PhQ+tY0Gzbto34QsZ6nP2iF7dqwef1LBt2PDCzzVgvC4sUvrX4j9\/+OuGI5rtHY5XPyNUCbF3J5acD23xGrhZg60ouO13V+jBTCqUZgTHNcD3sqhB8Hm356oPH6Mq\/eXAOIhYtt3RdKD4lZBsbNFa2xMztwdacla0l2NoSM7cHW3NWNpZV6MNsyqNsSyFgTC9zbGxsdClj4TRlDr5a37L2E\/fPKieLlnctfy29t\/28mk8NmQJHY2VKyt4ObO2ZmaYAW1NS9nZga8\/MJEWZ+zCT\/KfZQMAY0Ctr8JOmitRoi49dQgZoCpmgsSqELzMx2IKtHAE5z6i3MmzL2ocVLW1dBQzvNtq0aVNuGdavXx\/tSKrXU7bg86gL7yYauuvRGSRVEi4q02is5Go02IKtHAE5z6i3MmzL1of5KmVdBYwqRNYUkq+CFvFTtuAvv\/6bs+4BGn73MvrNN55VpIh1SYvGSg472IKtHAE5z6i3MmzL1of5KmUpBIyvwkj5KUvwecpIX+tSxVEXPUZorKRqLOGaBjm0YAu2ggRkXJelD\/NdOggYA6L1Dj5PGd313aeo\/44DM7n9656L6D+96RyD3JfXBAJGLjZgC7ZyBOQ8o97KsK13HyZTqjqexKumjSYmJnLLNp8vc2TxsvPbR2nzF0+td6n6qAtGYHKruxcDdAReMCY6AVuwlSMg4xkCRoZrJbzWK\/jxxbosXnZ94OLSbou2DSY6Alti5vZga87K1hJsbYmZ24OtOSsby3r1YTZ5dLENZgpJneS7e\/fuiIPpzqUDBw5QX18fDQ8PU0tLSyLDegX\/tnuOUP8dD82MvIQkXrhQaKxcvrJmacDWjJOLFdi6UDNLA7ZmnGyt6tWH2ebT1r5UAiZpW\/XmzZuJrxbIe\/S7lNT0VFdXV2ZaJXr27duXeV1BPYLP26SvGHsgWPECAZNXo4t9jo6gGL+s1GALtnIEZDzXow+TKclsr6URMCxeduzYQSMjI6RO3DUVIkmgdEGTBlJdGsmfl2kEZj6IFwgY2a83Olk5vmALtnIEZDxDwMhwjbz6vkrA5FwZtrn22mupu7s7ujiyLAKG173wOS\/q2f+RtwSz5iVehdARyH2pwBZs5QjIeUa9lWELASPD1buAUbdYd3R00ODgIDU0NCTmnEd8+FmxYoXRGhjdyZ49e0RoTD\/7Iv3JV56ifUdeiPz\/5bsX0SXnnS7yrjI4nZqaoubm5jJkJbg8gK1cSMEWbOUI+PG8evXqOY4mJyf9OC+Rl6CnkKanpxNFDC\/cvf322+nqq68mbozKsoh36K6DM9cDrFy6gHZdcXGJqor\/rOCvLf9MlUewBVs5AnKeUW9l2GIERobrLK9FFvHGs5e1u4hHaVatWkVtbW1Ull1I+tRRaNul06oOGiu5LxXYgq0cATnPqLcybCFgZLiKeVULdPVFwfyyrAP0tm\/fHoma+CMdfBYvG8YeoHseOR69mrdLV+E26aLBQ2NVlGB6erAFWzkCcp5Rb2XYSvdhMrnO91qKKaQiu41UEfVdR2p7dFNTU+4t1mUYgdF3Hc2HqSNMc+R\/MYtaoCMoShDiUI4g2NaaLQSMMPH49FHaaEhaNuIH2emLeNVnvOMoPsJSbwEzH6eOIGCEv0w4JFAUMMShHF6wlWELASPDNdGr2knEH\/IoyujoaOopubXIlmTwP\/b5SfqzPYeiYvSvOZ\/61yypRZFK8Q40VnJhAFuwlSMg5xn1VoatZB8mk2Mzr6WYQsrKKosZXs8SX8tiVjw\/VlLBj4++8Jkv8+lBYyUXbbAFWzkCcp5Rb2XYSvVhMrk191pKAaOPwNT7JmpGKRX8D3\/2YfrkPzwWRevW7gup+9JF5pELwBKNlVwQwRZs5QjIeUa9lWEr1YfJ5Nbca2kETNmmjXSEUsFvvPLu6DW8bXq+jb5wudFYmX9RbS3B1paYuT3YmrOytQRbW2Jm9lJ9mNnb5axKIWBMjv6XQ5DvWSL4w186SINffHTejr5AwOTXuyIW6AiK0MtOC7ZgK0dAxrNEHyaTUzuvpRAwdlmuvbXv4M\/3tS8qgugI5Ooy2IKtHAE5z6i3Mmx992EyubT3CgFjwMx38HUBM18OrUvCjMbKoPI5moCtIziDZGBrAMnRBGwdweUk892HyeTS3isEjAEzn8Fn8bL2E\/cT\/ztf175gBMag0hU0QUdQEGBGcrAFWzkCMp599mEyOXTzCgFjwM1n8PXRl\/\/1u8uo963nGeQgTBN0BHJxBVuwlSMg5xn1Voatzz5MJoduXkshYLIW8abdaeRWXLdUPoO\/9tb7592dR2nU0Vi51UeTVGBrQsnNBmzduJmkAlsTSvY2Pvsw+7fLpYCAMWDrK\/j66Mt8uvMIAsagknk2QUfgGajmDmzBVo6AjGdffZhM7ty91lXAxO8\/SivG+vXrcy9ldEeQn9JX8IfuOkhDd53aOj2fF+8q4ugI8uueqwXYupLLTwe2+YxcLcDWlVx2Ol99mEzu3L3WVcCobM+Xc2DU9NF8X7wLAeP+hTVNiY7AlJS9HdjaMzNNAbampOzsIGDseAVl7SP49zx8PNp9xA9fGcBXB8z3B42VXA0AW7CVIyDnGfVWhq2PPkwmZ8W8lmIEplgR5FP7CP6f\/N0j9OdfPRxlFtNHp2KGxkqu7oIt2MoRkPOMeivD1kcfJpOzYl7rJmD0aaNly5ZRb28vTUxMJJam3hc6+gj+fL\/3KCmwaKyKfXmzUoMt2MoRkPOMeivD1kcfJpOzYl7rJmCKZbu2qYsGX58+2vp7y6inff6e\/aJHDo2VXD0GW7CVIyDnGfVWhm3RPkwmV8W9QsAYMCwa\/Os+P0l\/uudQ9CZMH70MHI2VQeVzNAFbR3AGycDWAJKjCdg6gstJVrQPk8lVca+lEDBqOinUKSTsPkquqGisin+B0zyALdjKEZDzjHorwxYCRoZrplcWNldddRV9+MMfppaWljrk4NQriwRfnz764DsX07UdS+tWjrK9GI2VXETAFmzlCMh5Rr2VYVukD5PJkR+vpRiBySoKXyUwNjZGg4OD1NDQkGp64sQJGhgYoN27d0c2WYffxUd8Ojo6Mv0XCT4Or0uPLhorP1\/iJC9gC7ZyBOQ8o97KsC3Sh8nkyI\/XSgiYoaEhGhkZocbGxtRSsw0\/\/f39pARKV1cXdXZ2zkqjhE57e3v0mfq5qakp9bTfIsHH9BEEjJ+vqp0XdAR2vGyswdaGlp0t2NrxMrUu0oeZvqMedqUXMCxMpqenc0dg4vB0QZMHlq80GB8fT32Ha\/D1u4\/e3rKQPvvHy\/OyMq8+R2MlF26wBVs5AnKeUW9l2Lr2YTK58ee1FAImaxEvj4yMjo5arYGxvZpASsDo61\/45F0+gRfPywTQWMnVBrAFWzkCcp5Rb2XYQsDIcJ3xmja1o6Z6TF\/PIy\/btm2jvHUtyl\/WdJOycQ2+mj5iP9g+PTeCaKxMa7W9HdjaMzNNAbampOztwNaemUkK1z7MxHc9bUoxAsMAkqaKTMRFGjyTqSclmthH1iJhDr7+7NmzxyhmHbdP0fSzL1LTmafR7vc0G6WZT0ZTU1PU3AwuEjEHWwmqp3yCLdjKEfDjefXq1XMcTU5O+nFeIi+lEDBZUz6mu5DiTA8cOEB9fX00PDycOP1kKl7Yr4t61de\/4PLG5BqPv7bkWgKwBVs5AnKeUW9l2Lr0YTI58eu1EgLGZBdSHAsLn7R0JjuPdH8uwcf26fyKisYqn5GrBdi6kstPB7b5jFwtwNaVXHY6lz5MJid+vZZCwMTXv+hFzFtgq2z1XUd5AsVkeqmogNHXvxy78Z1+oxaINzRWcoEEW7CVIyDnGfVWhi0EjAzXGa88YrJx48ZZO454Gqinp4e2bNlCbW1tmTmIH2SnL+JVn3V3d1PazddZN167BB+3T+dXGDRW+YxcLcDWlVx+OrDNZ+RqAbau5DACI0POwiuLmHXr1s1KsX379lzxYvEKJ1NbAaOvf+lfcz71r1ni9N7QE6Gxkosw2IKtHAE5z6i3Mmxt+zCZXPj3WoopJP\/F8uvRNvj6+S\/YPp0eCzRWfuup7g1swVaOgJxn1FsZtrZ9mEwu\/HsthYDRp3jypor8I8j3aBt8ff3L\/o+8hRY3np7\/knlogcZKLuhgC7ZyBOQ8o97KsLXtw2Ry4d9rKQSM7cm5\/jFke7QN\/vLrv0k8jcTChQUMnmQCaKzkagbYgq0cATnPqLcybG37MJlc+PdaCgHDxTLdbeQfQb5Hm+Dr619WLl1Au664OP8F89QCjZVc4MEWbOUIyHlGvZVha9OHyeRAxmspBEzWXUhc7KwdQjJYZnu1Cb6+\/gULeLOjg8ZKrvaCLdjKEZDzjHorw9amD5PJgYzXUggYmaL582oTfBxgZ84djZU5K1tLsLUlZm4PtuasbC3B1paYmb1NH2bmsRxWEDAGcbAJvlrAi\/Uv+WDRWOUzcrUAW1dy+enANp+RqwXYupLLTmfTh8nkQMZr3QSMvnA37XA5VeSqTCFh\/YtdJUVjZcfLxhpsbWjZ2YKtHS8ba7C1oWVuCwFjzio4S9Pg4wA7u9CjsbLjZWMNtja07GzB1o6XjTXY2tAytzXtw8w9lsOybiMw8eLH70PKuh+p1uhMg48D7Owig8bKjpeNNdja0LKzBVs7XjbWYGtDy9zWtA8z91gOy9IImKQLFtU0U1dXF3V2dtaNmGnwP\/2to\/TBHQ9E+cQJvPnhQmOVz8jVAmxdyeWnA9t8Rq4WYOtKLjudaR8m83Y5r6UQMFkH2fH9SGNjYzQ4OEgNDQ1yJDI8mwYfC3jtwoPGyo6XjTXY2tCyswVbO1421mBrQ8vc1rQPM\/dYDstKCBgenRkZGaHGxsa6UDMNvrqBGgfYmYUJjZUZJxcrsHWhZpYGbM04uViBrQu1\/DSmfVi+p3JZlELAZK13KcMJvSbBxwJe+4qNxsqemWkKsDUlZW8HtvbMTFOArSkpOzuTPszOYzmsSyFgGAVPFW3cuJFGR0eppaUlonPgwAHq6emhLVu2UD0veTQJPhbw2ldoNFb2zExTgK0pKXs7sLVnZpoCbE1J2dmZ9GF2HsthXRoBo0TMunXrZpHZvn17XcULZ8Yk+PoJvLiB2qxyo7Ey4+RiBbYu1MzSgK0ZJxcrsHWhlp\/GpA\/L91I+i1IJmPLhOZUjk+BjAa999NBY2TMzTQG2pqTs7cDWnplpCrA1JWVnZ9KH2XkshzUEjEEcTIK\/\/PpvEq+DwQJeA6AvmaCxMmdlawm2tsTM7cHWnJWtJdjaEjOzN+nDzDyVywoCxiAeecHHFQIGEBNM0Fi5cTNJBbYmlNxswNaNm0kqsDWhZG+T14fZeyxHCggYgzjkBR87kAwgQsC4QXJMhY7AEZxBMrA1gORoAraO4HKS5fVhMm+V9zovBYzatr179+6I8Pr166m\/vz+Vdl7wsYDXraKisXLjZpIKbE0oudmArRs3k1Rga0LJ3iavD7P3WI4U81LA8MF4\/LBoMbmuIC\/4V4w9QGP3Ho18YgeSecVGY2XOytYSbG2JmduDrTkrW0uwtSVmZp\/Xh5l5KZ9VaQSMOvNlenp6DqXW1lbRk3h1QZMUorzgYweSW8VGY+XGzSQV2JpQcrMBWzduJqnA1oSSvU1eH2bvsRwpSiFg1JROU1NT5lSOBLKse5jU+\/KCjysE3CKDxsqNm0kqsDWh5GYDtm7cTFKBrQkle5u8PszeYzlSlELAmIgICVw88rJt2zbq6OjIvCySg68\/e\/bsmflx+tkXqeP2qejn9W9eQP\/t1xdIZDVIn1NTU9Tc3Bxk2epdKLCViwDYgq0cAT+eV69ePcfR5OSkH+cl8lIKAaNGYLq7u+ty6i4LGZ66SrvxOku94goB99qMv7bc2eWlBNs8Qu6fg607u7yUYJtHyO1zjMC4cTNOxXch1evWaV5\/09fXR8PDwzP3MOkZzwq+vgNp1wcuppUXYATGNOhorExJ2duBrT0z0xRga0rK3g5s7ZmZpICAMaHkaKOmkCYmJhI9SC\/izRNPWcG\/+rMP01\/8w2NRvrEDya4CoLGy42VjDbY2tOxswdaOl4012NrQMreFgDFnVXpLfdeRyQLirOBjB5J7uNFYubPLSwm2eYTcPwdbd3Z5KcE2j5Db5xAwbtxKmSp+kJ3JIt60BVC4A8k9xGis3NnlpQTbPELun4OtO7u8lGCbR8jtcwgYN25WqXbu3EmbNm2K0mzfvp0OHTpE4+PjmTuErF7gaJwVfGyhdoRKRGis3NnlpQTbPELun4OtO7u8lGCbR8jtcwgYN27GqdROIF5Mu2HDhug8GF77MjAwQPU4H0bPeFrw9R1I\/WvOp\/41S4zLC0MIGMk6gI5Aji7Ygq0cARnPEDAyXCOv+jkwy5Yto97e3kjAtLW1Ud4CW8Fszbg2ETDYgWQfCXQE9sxMU4CtKSl7O7C1Z2aaAmxNSdnZQcDY8bKyhoCxwhWMMRoruVCCLdjKEZDzjHorwxYCRobrjFde\/8LrXfQpJDUa09XVRZ2dncI5SHefFny1AykaRbrxnXXLX1VfjMZKLnJgC7ZyBOQ8o97KsIWAkeE6yytPF61bt27W7zZv3lxX8cKZyRMwixtPj86AwWNHAI2VHS8ba7C1oWVnC7Z2vGyswdaGlrktBIw5q+As04KPHUjFQo3Gqhi\/rNRgC7ZyBOQ8o97KsIWAkeFaCa9JwT987AXiM2D46b50Ed3afWElylKmTKKxkosG2IKtHAE5z6i3MmwhYGS4zvKqnwOjPuDzYHg3Uj2fpOBjC3XxiKCxKs4wzQPYgq0cATnPqLcybCFgZLjOeGXxsmPHDhoZGaHGxsbo92p3UhkX8eojMNhC7VY50Fi5cTNJBbYmlNxswNaNm0kqsDWhZG8DAWPPzDiFvo06PtpS1nNgcAu1cXhTDdFYFWeIERg5hmALtrUnIPNGCBgZrrNGWtThdfqryipgrhh7gMbuPXoq\/9hC7VQ7IGCcsBklAlsjTE5GYOuEzSgR2BphsjaCgLFGZpeAhcrGjRtpdHSUWlpaZgmbMk4h4RZqu\/gmWaOxKs4QowRyDMEWbGtPQOaNEDAyXGcJlYmJidy38P1Id955Z66dT4Ok4OMW6uKEIWCKM0QnK8cQbMG29gRk3ggBI8O1El7jwdcX8K5cuoB2XXFxJcpRtkxCwMhFBGzBVo6AnGfUWxm2EDAyXCvhNUvA4BZq9xCisXJnl5cSbPMIuX8Otu7s8lKCbR4ht88hYNy4WaVKOgemjFcJ6GfA8BUCfJUAHnsCaKzsmZmmAFtTUvZ2YGvPzDQF2JqSsrODgLHjZW1dpXNgePcR70LiB2fAWId6JgEaK3d2eSnBNo+Q++dg684uLyXY5hFy+xwCxo2bUaqqnQODM2CMwpprhMYqF5GzAdg6o8tNCLa5iJwNwNYZXWZCCBgZrpHXqgmYtZ+4n3gaKco7zoBxrhlorJzR5SYE21xEzgZg64wuNyHY5iJyMoCAccJmnqjoFJISQWordkdHBw0ODlJDQ0NiJvT1Nnm28eDjDBjzuGZZorHywzHJC9iCrRwBOc+otzJsIWBkuM7y6rqI98SJEzQwMEDt7e3U2dlJ6uempibi033jj366LwscTptmy2njwccZMH4qAxorPxwhYOQ4gi3Y1paAzNsgYGS4inllMTQ+Pp44ChP\/LMs2LmBwBoy\/kEHA+GMZ9wS2YCtHQM4z6q0MWwgYGa5iXrNESdIIjBq9ScqQHnxdwNz4+2+g\/\/qWJrEyhO4YjZVchMEWbOUIyHlGvZVhCwEjw1XEq1oPk3WH0oEDB6inp4emp6dp+\/btFL8FW8+YHnz9DBgcYlcsfGisivHLSg22YCtHQM4z6q0MWwgYGa7evar1L+w4bRFvfMHw0NBQlI+k9TL8ew6+en6y+K30\/Ir3Rj\/+5bsX0SXn4RA71yBOTU1Rc3Oza3KkyyAAtnLVA2zBVo6AH8+rV6+e42hyctKP8xJ5ecXJkydPlig\/hbJiIl7iC375hTwa09fXR8PDwzM3YaeNwOAMmEIhmpUYf235Yxn3BLZgK0dAzjPqrQxbjMDIcPXmNW\/nkXpRUQHDJ\/DySbz84BqBYuFDY1WMX1ZqsAVbOQJynlFvZdhCwMhw9eaVp4F4PUvW2S\/qZUlTSFlp9eDjDBhvISM0Vv5YYgRGjiXYgm3tCMi8CQJGhqsXr\/FD7JTT1tZWGhkZiQ6z47Neuru7ZxbrsuDZtm1bZGpzkB0EjJeQRU4gYPyxRCcrxxJswbZ2BGTeBAEjw7USXvXgN155d5TnlUsX0K4rLq5E\/suaSQgYuciALdjKEZDzjHorwxYCRoZrJbyq4OtnwHRfuohu7b6wEvkvaybRWMlFBmzBVo6AnGfUWxm2EDAyXCvhNUnA4AyY4qFDY1WcYZoHsAVbOQJynlFvZdhCwMhwrYRXFXz9EDsefeFRGDzuBNBYubPLSwm2eYTcPwdbd3Z5KcE2j5Db5xAwbtyCSJUkYHZ94GJaecGCIMpXr0KgsZIjD7ZgK0dAzjPqrQxbCBgZrpXwqoKPQ+z8hguNlV+eujewBVs5AnKeUW9l2ELAyHCthFcVfBxi5zdcaKz88oSAkeMJtmBbGwIyb4GAkeFaCa8q+DgDxm+4IGD88kQnK8cTbMG2NgRk3gIBI8O1El4hYGTCBAEjw5W9gi3YyhGQ84x6K8MWAkaGayW8quAvv\/6bxGfB4BA7P2FDY+WHY5IXsAVbOQJynlFvZdhCwMhwrYRXFXycwus3XGis\/PLENIccT7AF29oQkHkLBIwM10p45eB\/7dvfJR6B4QeH2PkJGwSMH44YgZHjCLZgW1sCMm+DgJHhWgmvHPy\/\/tJ9tPYT90PAeIwYBIxHmDFXYAu2cgTkPKPeyrCFgJHhWgmvcQGz\/yNvocWNp1ci72XOJBorueiALdjKEZDzjHorwxYCRoZrJbxy8D++c5z4HBh+cAqvn7ChsfLDEdMcchzBFmxrS0DmbRAwMlwr4ZWDv\/4vvkpDdz0KAeMxYhAwHmFiCkkOJtiCbc0IyLwIAkaGayW8cvB\/6+Ofp7F7j0b5xRSSn7BBwPjhiFECOY5gC7a1JSDzNggYGa6V8MrBv+iqz9A9jxyP1r6wgMFTnAAETHGGaR7AFmzlCMh5Rr2VYQsBI8O1El4hYGTChMZKhit7BVuwlSMg5xn1VoYtBIwM10p45eCf+d5P4xRez9FCY+UZqOYObMFWjoCcZ9RbGbYQMDJcK+GVg3\/8XSNRXnGNgL+QobHyxzLuCWzBVo6AnGfUWxm2EDAyXL15PXbsGPX29tLExETks6OjgwYHB6mhoSHxHXv37qV169ZFn7W2ttLIyAg1NjYm2i656M307G8ORZ91X7qIbu2+0Fu+57MjNFZy0QdbsJUjIOcZ9VaGLQSMDFcvXk+cOEEDAwPU3t5OnZ2dpH5uamqi\/v7+Oe84cOAA9fT00JYtW6itrY127txJ4+PjqYJHFzA4A8ZLyCInaKz8scQIjBxLsAXb2hGQeRMEjAxXMa9ZooQ\/O3jwYKK4ScrQ4jf\/Fv1wZV\/0EY++8CgMnuIEIGCKM0zzALZgK0dAzjPqrQxbCBgZrmJe0wRMfLTGJAPN7\/gDen7FeyNTjMCYEDOzQWNlxsnFCmxdqJmlAVszTi5WYOtCLT8NBEw+o9JYqPUwXV1d0ZSS\/igBs2bNGrrtttuiNTN5a2Cafvt\/0Au\/sjZyc8Y9w\/T1nZ8oTVmrnJGpqSlqbm6uchFKm3ewlQsN2IKtHAE\/nlevXj3H0eTkpB\/nJfLyipMnT54sUX4KZ0UJFHaUtIhXfX748OGZhbtDQ0M0PT2dugZGFzA4hbdwiGYc4K8tfyzjnsAWbOUIyHlGvZVhixEYGa5eveaJF35Z0hQSL+rt6+uj4eFhamlpmZOnRb93Pf1k8Vuj3x+78Z1e8zyfnaGxkos+2IKtHAE5z6i3MmwhYGS4evOat\/NIfxGPuCxZsmRmeokFzA033EBbt25N3Er92j\/8JL149htwjYC3aJ1yhMbKM1DNHdiCrRwBOc+otzJsIWBkuHrzmjcNpL+Iz4Bhe3X2C\/8\/P0lbrvn3EDDewjTLERorGa4Qh3JcwRZsZQnIeIeAkeHqxWv8EDvlVC3O5WRecsQAAAzMSURBVMPs+JyY7u7u6NwXfvSD7PIOvTv7j\/6GfvbvzsYpvF6i9bITCBjPQDECIwcUbMG2JgRkXgIBI8O1El4br7w7yieuEfAbLggYvzx1b2ALtnIE5Dyj3sqwhYCR4VoJr0rA4BoBv+FCY+WXJwSMHE+wBdvaEJB5CwSMDNdKeFUCpn\/N+dS\/Zkkl8lyFTELAyEUJbMFWjoCcZ9RbGbYQMDJcK+EVAkYmTGisZLiyV7AFWzkCcp5Rb2XYQsDIcK2EVyVgcA+S33ChsfLLE9MccjzBFmxrQ0DmLRAwMlwr4VUJGNyD5DdcEDB+eaKTleMJtmBbGwIyb4GAkeFaCa9KwOAaAb\/hgoDxyxOdrBxPsAXb2hCQeQsEjAzXSniFgJEJEwSMDFf2CrZgK0dAzjPqrQxbCBgZrpXwqgQM7kHyGy40Vn55YpRAjifYgm1tCMi8BQJGhmslvLKAWdx4OvEUEh5\/BCBg\/LGMewJbsJUjIOcZ9VaGLQSMDNdKeIWAkQkTGisZrphCkuMKtmArS0DGOwSMDNdKeGUBg2sE\/IcKAsY\/U+URbMFWjoCcZ9RbGbYQMDJcK+EVAkYmTGisZLhilECOK9iCrSwBGe8QMDJcK+GVBQzuQfIfKggY\/0wxAiPHFGzBVp6AzBsgYGS4VsIrCxjcg+Q\/VBAw\/pmik5VjCrZgK09A5g0QMDJcK+EVAkYmTBAwMlwxzSHHFWzBVpaAjHcIGBmulfD62j\/8JN30wXdF00h4\/BGAgPHHMu4JbMFWjoCcZ9RbGbYQMDJcK+E11ODXGz4aK7kIgC3YyhGQ84x6K8M21D7sFSdPnjwpgywcr6EGv94RQmMlFwGwBVs5AnKeUW9l2Ibah0HAGNSXUINvUHRREzRWcnjBFmzlCMh5Rr2VYRtqHxaMgDl27Bj19vbSxMREVAM6OjpocHCQGhoaMmvEgQMHqK+vj4aHh6mlpSXRNtTgy3xVzL2isTJnZWsJtrbEzO3B1pyVrSXY2hIzsw+1DwtCwJw4cYIGBgaovb2dOjs7Sf3c1NRE\/f39qRFWdvv27aPR0VEIGLPvgjerUL9U3gAVcAS2BeDlJAVbsJUjIOM51DobhIBJCvnOnTtpfHw8cxRm7969NDQ0FCXHCIzMFyfLa6hfqtqTnPtGsJWLAtiCrRwBGc+h1tl5K2B4yunaa6+l7u7uSMRAwMh8cSBgas+V3xhqg1UfmrPfCrZyUQBbGbahcg1SwKj1MF1dXdGUUtoIDf9+xYoVRmtgZKoVvIIACIAACICAPIHJyUn5l9T4DcEJGLWuhTmmLeLlhbu33347XX311TQ1NZUrYGocE7wOBEAABEAABEAgh0BQAsZEvDAPnjJatWoVtbW1kckuJNQiEAABEAABEACBchEIRsCY7jyKb7fWw7F9+\/ZI1OABARAAARAAARAoN4FgBAyPqkxPTxud\/aKHBCMw5a6gyB0IgAAIgAAIJBEIQsCkjaq0trbSyMhIdJgdnxPDO47iIywQMPhigAAIgAAIgED1CAQhYKqHHTkGARAAARAAARAoQgACpgg9pAUBEAABEAABEKgLAQiYDOy8rmbbtm2RBRb4utVPnqLr6emJ1ifl3U+l8+ZrILKud3DLTVipbNiqksev3QiLiJ\/S2HCNT1+jnciOgQ1b3RbtQbG6rb73Scsoinmub2oImBT+6poBXkPz0EMPRVuv+f8bGxvrG7EKvV3vLNeuXTvrvqp4MeJXP\/DPO3bsAPOUeNuw1V0w102bNtHmzZtTD3msUBXznlUbrvGdj1hPlx0OG7ZKGPJddrxuEe2Be1VX3Hfv3h3cH+IQMCn1Qt2RxF+gUNWr+1fCLGW8QWdRODY2ZrRTDJ1B\/l+y+i3qJmy5U7jqqqvo+PHjlHVKtVl0w7SyqbNse8MNN9DWrVvxh41BdbBlq9dvtAcGgBNM1CjWJZdcQocPH44uNw7pqBAImISgp91urW67dqtK8y+VPorFI1fxn7OIoMHKri8ubFmUX3rppfS5z31u5ub2+Vcr\/XE1uTAWfF8mYFNnk0Zg8i7nBeu5BI4cORL9knfi9vb2QsDMh0qSNOLCjf+SJUsw7G5RAeKjAjZ\/sbqe62ORvUqb2rJV12dceeWV0SWmEOPJ4bfhygLm4MGDkSOslcv\/OtmwZW\/61Mf69eujzhePG4G4IHTzUr5UGIHJGIHRFzxBwNhXXtsGS72BO4abb74Zi3gzkNuw5Y7g4x\/\/OL3nPe+h5ubmzLVI9lEOK4UNV7WeSC3c5bQbN25EvU2pEjZs1dTHli1boikPm9HbsGqkn9JAwPjhWAkvmELyEyabIWOIFzvmNmzZ9utf\/3r0Fyx2IWVztuEan0ICW7C1+xbXzhoCpnasS\/EmfcQFi3jdQhKfMspbaIqdBuacbdjq29P1N2BYfi5vG67x+ox2Irv+2rCFODRvC0wsIWBMKAVkg23UxYNps20Sw+92vG3Y6p4xSpDN2YZrvFPANIc\/tklTSJies2sjdGsIGHd2lU2Jg+yKhy7r4Cq1CJKnNtJGCXAwWHoMTNlCwNjVYxuu+kF2OGwtn7MNWxaE69ati5yCbT7bLAsImGL8kBoEQAAEQAAEQAAEvBHALiRvKOEIBEAABEAABECgVgQgYGpFGu8BARAAARAAARDwRgACxhtKOAIBEAABEAABEKgVAQiYWpHGe0AABEAABEAABLwRgIDxhhKOQAAEQAAEQAAEakUAAqZWpPEeEAABEAABEAABbwQgYLyhhCMQ8E\/gkUceoYULFxLf5m3y8HkPzzzzDC1dutTE3MlGndnT2tpKIyMjxnmryg3jqny1Onuk1u9zCjoSgUAJCUDAlDAoyBIIMAHbk11rcVhVERFSJG0tawQLCn5qeftxVdjUMg54FwjkEYCAySOEz0GgTgTKKGBs86Sjq0onDQFTpwqP14KAJQEIGEtgMAcBnwT0o+jZr5qWeeihh2aOUeffqysV4lcuqGmOs846i3p7e2liYiLKnrqoUd3ts3v37uj3JtM++jv0aRS++mHTpk0zxd+8eTN1dnbOwZFmpwTM2rVr6brrrovSxadp9OPjlWP1HmZ11VVX0dvf\/vYovSrL008\/TT09PTQ9PR0l+ehHP0q7du2i4eFhamlpiX6XVqakWMYFDP\/83HPPRf8pjlkXYcYvIuR3JP2uiuLOZ92HLxAoSgACpihBpAcBRwJJFyvqnWd8tCPthl5+\/eDgILE\/FjE89dHW1kZKHHV1dc0Ijawbv1V+lL+Ghoao47355ptpdHQ0EgN5IzBxe\/1SPhZZLDQuueSSKL\/K\/44dO6K1NCxE+vr6ZgkP3Z8SaYsXL55JHy+j+vnJJ5+M8tzc3EwDAwORUFJTQnkXhyYJmG3btpEupJizzlWvAhAwjl8IJAMBSwIQMJbAYA4CvggkCQzdd55YiP9lHxcwnH5sbGyms2f7rNuok6Z44vZZecq76Tp+wzDnJ29aSf9cCZi4IBsfH59VRl2g8DtuuOEG2rp166zFxlnTREkChkd3lOhin1kcIGB8fUPgBwSyCUDAoIaAQB0J6NMt8emdtE4yPs3S0dGROAITn8rRi5k0\/ZP2Pv3W8KyOO28RcZJYSfpdfFotPk2mRpi4PElCRPfJozrqRuN4mNOmgZIEDKfVF\/VmCS8ImDp+ofDqeUUAAmZehRuFLSsBvdPW18FwZ6q2KitBEl+XokYg4iMwnDY+cpBV\/jRxkjWtpfsrKmDYl1rLogRW0giMjYC57777SE1RmW5Fh4Ap67cE+QKB2QQgYFAjQKBEBHQRoEYYWMDwehFey9He3j5r4awuUuICJmu9S1KRazGFFF\/jor+TxUbWdJCaQtIFTNJohz6FxCMwGzdunFnDYxJqiSmkPDGZN5Vmkm\/YgMB8IwABM98ijvKWhkDSGhh9FERf1Jq0GFWNyKgpJC6YLnKUf17Qq6Y\/ktahKCC+FvHqIx76upgVK1bMWaQbFzB6WpVXzh8vyE0SMKaLeNmHWsOSt\/ao6CJeNcWndo6pcuiLl+OVEAKmNF9LZKRCBCBgKhQsZDU8AqpzU1uA9ekhfQs0T6lcdtlls7ZKs3C5\/PLL6ZprrpkZYYiLGjUqo7ZXM0HVsabRzNpybLqwOGm7tckamPi7t2zZEq1z4YW7qvz6CAyXIc4wvo06vpWc06RtAVejXvyvEn1J26j19Enl0tcfcZyWL19O+\/fvj0RUXGiqMsRHp8Kr7SgRCPglAAHjlye8gQAI1JkAC4qknUem2TJZA2Pqy9QOIzCmpGAHAi8TgIBBbQABEKgsgfg6HzXaop\/7Yls4CBhbYrAHgfoQgICpD3e8FQRAwBOB+OnEWafkmrwyfrniHXfcESWTuhsJlzmaRAU2IDCXwP8HbWE8g10v1boAAAAASUVORK5CYII=","height":120,"width":200}}
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
