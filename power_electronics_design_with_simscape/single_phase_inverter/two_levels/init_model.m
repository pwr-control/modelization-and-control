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
%   data: {"dataType":"textualVariable","outputData":{"name":"V2rms_load_trafo","value":"6.6000"}}
%---
%[output:617b30e4]
%   data: {"dataType":"textualVariable","outputData":{"name":"I2rms_load_trafo","value":"30000"}}
%---
%[output:45db0fb3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"466.6905"}}
%---
%[output:19be7e26]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"848.5281"}}
%---
%[output:26f707e8]
%   data: {"dataType":"matrix","outputData":{"columns":3,"exponent":"-3","name":"num","rows":1,"type":"double","value":[["0","0.9665","0.9465"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.0000","-1.9381","0.9391"]]}}
%---
%[output:2a03b5f5]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAakAAAEACAYAAAAJP4l9AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQtwFse15w9ZESQwxoB5SbKR7UB87bsXx7GvFHIT4Vt5OGVDpSAxoGQRKuCyJixUwlO46gJOjHiEqoU45RBDBJSD7LVhK9bW5jpkbajNsrBxnKh8cRLIQ2BJCAhgG7AglsPWadEfo9HMN4+vZ6an5z9VKknzdfd0\/86Z\/n+np6e737Vr164RDhAAARAAARDQkEA\/iJSGVkGVQAAEQAAEBAGIFBwBBEAABEBAWwIQKW1Ng4qBAAiAAAhApOADIAACIAAC2hKASGlrGlQMBEAABEAAIgUfAAEQAAEQ0JYAREpb06BiIAACIAACECn4AAiAAAiAgLYEIFLamgYVAwEQAAEQgEjBB0AABEAABLQlAJHS1jSoGAiAAAiAAEQKPgACIAACIKAtAYiUtqZBxaImcPz4caqrq6OOjo7cpSZPnkzr16+nkpISz8t3dXXRypUraebMmVRVVeWZ\/vDhw1RTU9Mr3fz582nFihUUtCzPiyEBCBhCACJliCHRjOAEWKSWL19OGzdupHHjxgUWiqDCwiK1YcMG2rFjBw0bNix3vdLSUiFUOEAABPoSgEjBKzJLwEukrJGPjHgYFgvNtm3bqLq6WrDjzziSeuGFF6i+vj53zi48dpHihFyHdevW0Xe+8x0hlhyVTZgwQURozc3NoiwZ3fHf8jyfe++992jVqlX0xhtv0KFDh+jkyZM0Y8YMGjt2bK+Ibc+ePTR+\/HhasmQJffazn6Vvf\/vbxML43e9+V7SlpaWFGhoaaPr06Zn1BTRcXwIQKX1tg5pFTMBpuE921lYBKy8vF+IwceJEIQAyGjp37pwYLuTOno+mpiYxVCjFxD4M6CRS58+fF+LxrW99i7Zv3y5E6rbbbhNllJWVkfzcKkZ8DRaWpUuXUmNjoxCp559\/Pheh8XXk8CMLZ2trK82bN4\/mzJkjzrN4chs4HUd1+\/fvFyLnd5gzYrOgeBDoRQAiBYfILAG3SEqKkRQdfj4lO3sJy\/4c6cSJE7koSqaxRl98zq9IsZDIoUSOpjjqeeaZZ4SIcd044nETL\/kszR4FSpHiesuoj8WL\/+e01rZm1iHQcC0JQKS0NAsqFQcBu0jxNaUY8VBeUJGSnb5b3f0O93F+nmBhHaaTkZaXSMkojn9zZPTyyy\/3iqQgUnF4Fq6hkgBESiVNlJUqAvkiqfvvvz83qcLvcJ8cfrOmtz7nyTdxYtGiRbmZgtahQ\/uwnhyWcztvHWqUz7Y4EkMklSrXRGUtBCBScIfMEvCagh7FxAk\/U9B5kgM\/P2Ih4vQXL17sM6HCaeKEfKYkJ3CwOHE5v\/nNb4TgLly4UAzvYbgvsy6fyoZDpFJpNlQ66wSchg6zzgTtN5MARMpMu6JVBhKwTnHn5vEzKz8vERuIAk3KEAGIVIaMjaaCAAiAQNoIQKTSZjHUFwRAAAQyRAAilSFjo6kgAAIgkDYCEKm0WQz1BQEQAIEMEYBIZcjYaCoIgAAIpI0ARCptFkN9QQAEQCBDBCBSGTI2mgoCIAACaSMAkUqbxVBfEAABEMgQAYhUhoyNpoIACIBA2ghApNJmMdQXBEAABDJEACKVIWOjqSAAAiCQNgIQqbRZDPUFARAAgQwRgEhlyNhoKgiAAAikjQBEKm0WQ31BAARAIEMEIFIZMjaaCgIgAAJpIwCRSpvFUF8QAAEQyBABiFSGjI2mggAIgEDaCECk0mYx1BcEQAAEMkQAIpUhY6OpIAACIJA2ApkXqba2NuIfHCAAAiCQNQLl5eXEPzofmRYpFqdly5bRkSNHdLYR6gYCIBAzgYEDB9KDDz5Ily9fptdffz3mq8d3ucrKStq0aZPWQpVpkTp8+DDV1NQII5WVlSnxjPb2dhGZsfH9Hl558n3u9pnTefu5fP+zcG\/ZskV7Nsw4KB8d2Xi1w8mXvPzGq0y\/vuPFy\/p5FH7j1Q63+8yLT77P33rrLTp58iTdeuut9MADD+Qu4cXCqa5R8\/Fqp5vvcP+3d+9e2rNnD1VVVfntrmJPl6hIvfDCC1RfX9+r0Q0NDTR9+vRYQEiR0t1IscCwXQRs3KmDjfls3n33Xdq2bRsNGTKE5s+fr+wW1Ml3dKpLPsCJiJSE4yRIUrjiEI60GEnZHRKgoNbWVtq1axfV1tZSRUVFgJzmJwUbdxubwiYqkdKJT1r6v9hF6vz589Tc3Cw6v3yH7CCj7PLSYqQoGbiVfeXKFTp16hSNGTOGiouLk6iCttcEG3fTmMImKpHSiU9a+r\/YRUqnnictRkqCmU43UxLtz3dNsIFIhfVJnXwnLf1fIiJ1\/Phxqquro46ODuIhPz742VRpaSk1NjbSuHHjwvpAoHxpMVKgRilKrNPNpKhJyooBG4hUWGfSyXfS0v\/FLlJdXV20cuVKmjlzJo0fP57mzJlDM2bMEJMl+HnUoUOHaP369VRSUhLWD\/rkk8bgD6zPwfj83xo+Q+Vler8noAyEpaCikd7Pma50dVFR\/\/5UVFRE\/Ue4py9y+axo5NjcFe1p+l+\/vlveKNqsqkydOhpVbVJVjilsMNynyiMKLyd2keJnUmvXrqXVq1fTsGHDaMOGDVRdXS2mQNo\/K7x5JMpcsmQJrVq1ShS3bt062rx5s7g2i9S\/LfkSTfvKV4wXqu6zrYFwdnd\/QF1XrlBx17m8+brP9C036LX4AlaxkgLKwijPs+DJv0vunRSoLaoTm9IRq+bC5ZnCBiIVhXeEK9N4kWIhYiHcsWOHiM5kFMeimJZwN5xpC8ulsrOxi9YH14VNnu8+cyJXWXnug+uiyiLI51ig7OVYRcsuZlEKmUo2hVlJv9ymsIFI6eNbmRCppqYmMYTIB4vUxIkTxfAii9SXv\/tzqqysovJyNS\/z6mPawmty6dIlMbOPh\/t0OFikWNBYwEq7O0WVxnzYKf4e8+Hp3DlZVxYujso4IusoGkW\/GnAf\/WrABCVNOdbxjuCiCxsljVJUCAuVCWze6\/gDfXXMaZo7d64iMj2RZmdnJ40aNUrpI40gFeTr8w+\/BMwr7sTxuk+Q+tnTJiJS\/ByqpaXFsd4TJkwQUQ8Px6k4WIjyidSjO9\/OXWb06NEqLmlEGd3d3aIdYTrhMYOTE7XSDztpTHcnffJqj3998spveuzxpxtLX3UU9dj5fwz8ovj9q+IJRHf6f+P+ww+7qbv7QxowYIASW5feXEQd7\/XwlgefS+PBfnP16lUaNGhQGqufqzMvh3TmzBn6yuhOmjx5srK2MJuzZ8\/SiBEjlPlP0Mrt3r1bvAMpD4hUUIKK0\/sZ7pPLIrFIQah6DMATXE6fPi14GPOe1Pk2+uDYL0Q0xlHZxQM7e3vb0HIq+vinReRVfO8kchsyNJKNovvOFDY83Ldz506x4sTs2bMV0dHjvpKRlFzCCiJlMy9PZIgzkvKaOMFr9+luJGV3SICCTHm24NVk+Zzr4ms93yy73jpAV44eyGXjIcPBk2YTT9zg33xkhY0XO6fPTWGDZ1JhrB9NntiH+6zN4CnnvEzIihUrxGn+nw\/Va\/dZp6BbBQkTJ9ydypTOJsxtYxUuJ9HiCOti\/yE04uH\/TINvvzvMJYzNY4rfQKT0cdHERMppunkUU9DzoYZIQaT83opiePB6tMXDhFLIONJi0RIR10O1vabS+y3bpHQQqfzW1IlPWvq\/xERKvtQrZ9qxaXmqOK9CofplXje3SYuRkujEdLqZkmh\/vmsKNm\/9km45\/3ui8+29hgjl8CDnH\/rYat2qHnl9TPEbRFKRu4rvCyQmUlxD+\/Mp1TP7vChApBBJefmI0+dOHTFHVl1HD4hJGfZIi59lFd9b7ToRI0wddM0DkUIkpdo3ExUp1Y0JWh5ECiIV1Gc4vZ+O2Do8eOHFNbnLyEjLVNHywyYM87jzIJKKm7j79WIXKWzVoY\/xPYe0sFWHI6IwHbEULetEDBOHBsOw0fGOgEjpY5XYRYqbjk0P9XEAt5qY0tlEQbpQNixYvDTUlaMHjRsaLJRNFPYKUyZEKgy1aPIkIlKyKdg+PhqjqijVlM5GBQt7GarZmBRlqWYThf38lAmR8kMpnjSJilQ8TXS\/Cp5J4ZlUGB+MuiPmCRgcZTk9y9J9mnvUbMLYK0weiFQYatHkgUhhxQllz12icVH9So2zI84XZek4+SJONlF6BkQqSrrByoZIQaQgUsHuGV+z+wIW6Su5dZq7jLLkSu8l90zS4r0siFR+U+rEJy0jSRApiBREypdE3EikS0cjoyyd3svShU1Ak\/ZJjkiqUILq8icqUtaXebdv306vvvoq1dbW0rhx49S1ME9JafkmEQsM20VM6WyiYKcjG12GBXVkE8YHIFJhqEWTJzGRsi6LxIvM8hbyfMi9n3gX3agPiJQ7YVM6myh8KA1snCZfiO1HIh4WTAMbPz4BkfJDKZ40iYmUdTHZZ599VojU+PHjae3atbR69Wplmx7mwwiRgkiFuc3S1hHni7JUzxZMGxs3+0OkwtwZ0eRJTKScIqmDBw9igdlo7By4VFM6m8AN95EhzWyiHhZMMxur6SFSPm6EmJIkJlLcPiwwG5OVQ1zGlM4mRNM9s5jCxrryharZgqawgUh53gaxJUhUpGJrpcuFMNyH4b4wPmhKR2xve77Zgn6HBU1hA5EKc2dEkyd2kYp7+3g8kwrnOKZ0NuFanz9XFtiEHRY0hQ1EKoo7J1yZsYuUtZpxbR\/vhgaRFCKpMLeNKR2x37Y7bTsiV3AvGjmWeL8seZjCBiLl1zuiT5eYSEWxfbwUHYmtoaGBpk+fnlt1nc\/Lc\/w3RAoiFeYWM6UjDtN2ziNFy7q2oJzeXjJlBZ0yYIsXiFRY71CfLzGRimL7eHtkxrhYDJcsWUKrVq0S9NatW0ebN28WU9whUhCpMLdU1kXKyszpORYNLafBk2pp6OfnEkdcaTwgUvpYLTGRkgIyZ84camlpEUQK3T5+w4YNVFFRIaInebAQ8fkdO3YQvyC8cuVKmjlzJlVVVUGk8vghOmIIeNBuigXrwv7tdLHl50R\/OiKyp3UnYohUUOtHlz5RkVLZLLfp7MeOHcutYsHXY5GaOHFir2HAxYsXU2VlJY0ePVr84OjZIr2zs5NGjRolxB3HDQJgk1\/A2W+Gf+QKdR9+XiTsat7QR7D63VWltUtdvnyZeKm2QYMG0dy5c5XVVQffYfvwT3t7Oy1btoz27NkjvrTreiQmUm6z\/AqNpiRoHvo7dOgQTZ06lfbt20fr168XHzmJlMzD6wbOmjVLV1vFWq+rV6\/S2bNnacSIETRgwIBYr637xcDG3UKubC600bXX9xLt39KTeWg50QPTxJ\/9Pr9YO5OzSDU3NwuRmjx5srL66eA7u3fvpl27duXaBJEKYF4WlrFjx\/pS9ePHj1NdXZ1YoYKdiEXI+o1fDvM9\/vjj9Mwzz+Qd7tu0aROVlZUhkrLYip8Znj59WjApLi4OYEXzk4KNu419sTnfRl2\/+DFZV2+nOyup5F7ebmSNFg7Ew307d+6kIUOG0OzZN2YvFlo5X3wKvYhHfhlJHTlyhLZs2YJIKghvpxl\/fvPb8\/JzKD7mzZuHiRN+IVrS4ZmUOzSwUccm3\/tYQx9bHcJz1WTBMyk1HFWUkthwn1PlrZMcePZd0MM6Bd06bGg9bw1tMbtPXWcT1FZpTg+RisZvdBIsiJQ+d2hiIuX2TMr6HlPUmCBS0XQ2Udst6fIhUtH7TZwrtzu1BiKV9F124\/qJiZQOCCBS0Xc2OthZdR0gUvH6TRIRFkRK9V0TvrzERCqKFSeCYoBIxdvZBLWPrukhUsn5TVyCBZHS5+6LXaTkShM8vdPpcJqpFxUuiFRynU1UNo2jXIiUHn7jJFhDv7qGiu+tFjMFCzkgUoXQU5s3dpGS1S9kJp8qBBApPTobVfaMqxyIlH5+Y1+eSa50EXaGIEQqrrvJ+zqxi5QUp0WLFtHSpUtzSyLJqqp6mde76VhgNh8jdMT6dcR+fDrpNDr4jX0BXF78duhXVweKriBSSXvSjevHLlL6NB0iBZEK5406dMThah59Lp3YOEVXHFlZtxZxIwKRit5X\/F4hUZHiFSbq6+t71RWRlF\/TRZtOp84m2pYGLx1s0hdldh09QGe\/Xye2GeGhQC+xgkgFvy+iypGYSMn3pFasWOFrGaQoAOCZVPo6myj8IGiZEKn0+g2L1Jnv19GVoweIhwFHfqPRcTsRiFTQuyK69ImK1Nq1a2n16tVib6ckDohUejubJPxFXhMilX6\/sUZWI77R2GcIECKV5B3W+9qJiRRXg4f7+LDu\/xQnGohU+jubOP0FIuVNO20CfuG\/rSXeYZifU7FYyQMi5W3ruFIkJlJRb9XhByBECiLlx0\/sadLWEYdpY9g8aWTDUdWpNQ+J4b\/SNa+JpkOkwnqA+nyJiZT6pgQvESIFkQruNT0bQp46dYrGjBmDbUxsANPKRgqVjKggUmHujGjyJCZSbpEUN7O0tJQaGxtp3Lhx0bT6eqkQKYhUGAdLa0ccpq1B86SZjRSqMWteo7+Wf4K2bdsm9pOaP39+UAyu6XXik5b+LzGRYivyM6nW1lbiGX7yf\/7NGx82NTWJDbmiPNJipCgZuJWt082URPvzXRNszP1y07HmITHzb\/j2dyBSmtx4iYlUvgVmeTWKrVu3QqQSdBJ0xOZ2xFG6Vdr9hqeon1xwB33061toz++vIJKK0ll8lp2YSMmFZnlozxpJHTp0iJYvX07PPfdc7rzPtgROhkgKHXFgpyE8kzI9yuSXfnlr+5ce3ACRCnODKM6TmEhxO+zPpXi1iaeffpo2btxIEydOjHxqOkQKIhXmfkp7tBCmzX7zmMCGBYqF6n\/+w0rqP7ICz6T8Gj+idImKVERt8l0sRAoi5dtZLAlN6IjDtNtPHlPY\/Omr\/eiXdzxG79z1zxApP4aPME2iIrVhwwbxcNJ6BFm7zz7xwhqZWfelkmLE17FuTw+RgkiFubdM6YjDtN0rjyls+LnUH64Np+P3zYNIeRk94s8TEykWlCVLltCqVavojTfeEDP6Tpw4IZrrZwUKKXA8PVQ+0+JzFRUVNGXKFFq5ciXNnDmTxo8fn7sOl71u3TravHmzWIoJIgWRCnN\/mdIRh2m7Vx5T2PBwX+f\/\/Qkd+kwDRMrL6BF\/nqhIybX7jh07RgcPHqR58+aRn\/X8OIJiUeM8fLBI2ReslVFWdXU1sXjt2LGDSkpKcuJVVVUFkcrjXKZ0NlHcP2Bj\/pcb+Vxq\/+d+AJGK4iYKUGZiIsWz+3iaOQvTuXPnqK6ujjo6OijIcB+Lj1WkZGTGLwGzSPFMwalTp9K+ffto\/fr1Ii1HWHJShoykFi9eTJWVlTR69Gjxg6NnBltnZyeNGjVKiDuOGwTAJr9ImeA3Xb94ji5sm0cHP\/UkzVywTJn76+A7bB\/+aW9vp2XLltGePXsS24nCD9jERIor19LSQgMHDhQrS7Bg8E69QVaaUCVSElRtbS3NmjXLDzfj01y9epXOnj1LI0aMoAEDBhjf3iANBBt3Wsaw+eNhurathv7ffYupsmZxEPfIm1YHPrt376Zdu3bl6gmRKtC88n2q5ubmPssl2UVqzpw5YuiPh\/KCDPdt2rSJysrKEElZbMXcT58+LZgUFxcXaEWzsoONuz1NYcNr911eOJqOTZhLk5Z+T5kD68BHRlJHjhwRCyZApJSZt29BVpHiTzFxQh1sPHfJP6SFBWad+ZjiNyxS5+beQu1\/N40+8+RLym4snfikZeJY7MN9+RaWZU8I+0yK81rLts76s05Bt35rSIuRlN0hAQrS6WYKUO1YkoKN+QLOIvW\/\/\/UrYqHZqcs2KvMrnXwnLf1f4iKVZKiZFiMpu0MCFKTTzRSg2rEkBZtsiBRWQY\/ldvK8SOwiZa2R07JIPFU8ru3kIVLmdzaed0CIBBAp8\/0G+0mFuDEiypKoSNnbxKIh32mKQ6ggUuZ3NlHcNxAp8\/0GIhXFnROuzERF6vjx47n3o4I+jwrX3N65IFLuFHmfL56mytPyeRUPHDcIgI35fhOVSOnkO2np\/2IXKesQX5BJElF0kmkxUhRt9yoTbNwJgY35bKISKZ18R6e65OuPEhUpp4rFKVxRGKmtrY327t1L06ZNo\/Lyci8tEJ975cn3udtnTuft5\/L9z5\/V1NQofYfCq51OsPzkCcpHRzZ+\/MDOJwo2TvXw4sV5pM9H4Tdh2PjJk4\/fhQsXxDtEI0eOpAULFuTQe7Hgez4t9xW3j+2W5OQ1Px1k7CLlp1JxpbEvi6TiunKpEbnUkp8yvfLk+9ztM6fz9nP5\/uebjZdMCdIOr7Z6tdMpv588QfnoyIbb7qetVkZ+0gdl41QPL15WX4nCb8Kw8ZMnHxsWqddee42uXbsmvnDKw4sFL6+WpvuK2wWR8uq5Evycv\/FwR8xvXuMAARAAASsBXhKMD14ezNSDRZVX3PE76pMEh0xHUnJIgMUKBwiAAAhkjQCLk84CxfbIvEhlzSnRXhAAARBIEwGIVJqshbqCAAiAQMYIQKQyZnA0FwRAAATSRAAilSZroa4gAAIgkDECEKmMGRzNBQEQAIE0EYBIpclaqCsIgAAIZIwARCpjBkdzQQAEQCBNBCBSabIW6goCIAACGSMAkcqYwdFcEAABEEgTAYhUmqyFuoIACIBAxghApDJmcDQXBEAABNJEACKVJmuhriAAAiCQMQIQqYwZHM0FARAAgTQRgEilyVqoKwiAAAhkjEDmRYq36cBWHRnzejQXBEBAEMBWHZo7AovTf9nxCzpy5HCfmn7k\/XOutf\/I+39x\/Mwpj1tazdGgeiCQaQIDBw6kQYMG0eXLl+n99983lgU2PdTctLx9\/KM7385t+nXy\/JXIa2wXLbuw3T6sWNThtqE9v\/lob2+jjmMtxA418R8+Js7JdHx+y5YtYnfNsrKyXB7ebdh+3n4u3\/8s4E7lFgLIqU5e5fnJky+NHw5ch6TZONUhCTZ+WNjTWH0lCr8Jw8ZPnnx+8+abb9Lx48fFPcX3nTz8+FOa7ituF7aP97rTEvycRaqmpsa3kfKJmPyMBYVv1MrKKnr7Qm\/RO3m+q09rZT572TKvX+GUoiUFjv\/fu3cvffVLk6i8vIxuH1ZCLJAvvbSXvjnva0LsuJ6cZtq0aUKorf\/z30HY+DGj\/Xqq8uQr1+kzP+fiZsMsgvLxkz4oG6d6ePHiPNKPovCbMGz85MnH5ujRo\/Tiiy\/SyJEjacGCBTlX9WJhv490vq\/4SyjbDSLlpydKKE1QkUqomrnLWgVL\/s1iZhU\/u+j1fO4cIVqjNv5b\/v\/pu4aK6K2+bor2DpyETdLmN3EyMoXNu+++S9u2baMhQ4bQ\/PnzlSHUiY9OdckHONMTJ9JiJBV3iLN49UR2v\/jDO+L3\/\/ljz2\/rYRWyf\/rYLeIjFjGrqKmoX5rKaG1tpV27dlFtbS1VVFSkqeqR19UUNlGJlE580tL\/QaQCDPdFfodrcgEWtGMdF+itk3+hi9eKqeO9bhGNuYkYPz9jAeMhRfm3Jk2JpBpXrlyhU6dO0ZgxY6i4+Mazw0gulrJCTWETlUjpxAcilYKbKy1GSgJlvpuJBatHtC7kIjG7gHGk9em7WLj491AhYqYcOnU0ujE1hQ1ESh\/PQiSFSMrRG8N0Nl7ixYI188HRuSHDtApXGDb63PLR1sQUNhCpaP0kSOkQKYiUMpFyczwWr6ZfnhIfN\/2yU0Rh9mddaYq2TOmIg3QUftOawgYi5dfi0aeDSEGkIhcp+wWsERdP2rAOFcpoS2fRMqUjjqJ7MYUNRCoK7whXZqpF6oUXXiCeLbNixQrR+vPnz9OcOXOopaWFJk+eTOvXr6eSkhKSz544TUNDA02fPl2kxzMpd6eJu7OR0ZabaM18cEwu+grn6upyxc1GXc2jL8kUNhCp6H3F7xUSFSkWmfr6+l51tYpIvkZs2LBBvMfA7zBIkeJzPCV4ypQptHLlSpo5cyaNHz+elixZQqtWrRLFrVu3jjZv3kzDhg2DSOUBnHRn4yRaMsriWYTy2ZZfR1eZLmk2KtuiuixT2ECkVHtG+PISESkZwTgJkhSufG9Bc5qxY8fSwYMHRctZpGQUxX9XVVWRjLKqq6uJxWvHjh0iqpLixWkQSekTSeVzYevw4IZXWnNJpWjFHWWZ0hGH7zbS4TeFtA8iVQg9tXljFykWk+bmZvEiZL5DvizpFU1ZRUpGTOPGjRMidejQIZo6dSrt27dPDP3xwSI1ceJEMeQnRWrx4sVifa7Ro0eLHxxE3BF3dnbSqFGjhLjrdFijrF+196ymYX2W9UB5tO8u6cwmaTuZwoYXlt2+fbtYZHbu3LnKsOrAh+9r\/mlvb6dly5Zpv6pM7CKlzNpEIkJSIVKyTiycs2bNUlnF1JZ19epVOnv2LI0YMYIGDBigbTv4RePm316iUxd7fvNRenMRPXr3TcRi9cky9YKVFjZJGM0UNixS\/GWaRYqfb6s6dOCze\/dusWKKPLB2n4t1rZMc7ElKS0upsbGROCIKEknxpIkww31yBXFEUjdod3V10enTp0VkmaZVFV5v63nJWA4LsmBVjr1JRForvniHkr4mrWyUNN6jEFPY8HDfzp07xdp9s2fPVoZOBz4ykpKrtUOk8pjXPjuP\/+eDnzc1NTWJrSL8ihSnw8QJZfeSGO5L+9I\/clhQvpvFdOSwYCGCZQIbdZ7SuyRT2OCZVFQeErzcxIb7OJJau3YtrV69Wsy040OeW7RoEW3dujWwSFmjM+usP+sUdOu3BkyccHcYUzob2cJ8swWDvpNlGpvg3Yb5fgORUukVhZWVmEhx2MuTGHhoT04hl5Mdli9fTs8991zufGFNdM8NkTK\/s3FqYaGCBZEy328gUlH1usHLTUykZOQkX77l\/ydMmEBPP\/00bdy4MTcDL3iT\/OeASJnf2fjxhp4XiG88x7IOCzpNb4dIme83ECk\/d048abQRKZ5SNv0XAAAW5klEQVTu+eqrr4qp6V4TJlShgUiZ39kE9ZV8URaXxc+yIFLm+w1EKuidE136xERKDvfxO0u8tBG\/dMsHT5iQyxlF1+yekiFS5nc2hfiQ3EOLdz62v0T88MeK6XN\/P5o+9\/djCrmEcXlNEXCIlD6umZhIWSdOPPvss0KkeAkj+2SKKFFBpCBSQfxLRlnd3d20+X+15bJatyCJe\/WLIPWPIy1EKj9lnfikpf9LTKScIile5qijowORVBy9icc1dLqZNMDRqwqSzQcDhtJ\/f7Nn40d7pCV3KA46c1C3tgatjyl+g0gqqOWjS5+YSHGT7C\/08sQJXmNPTkmPrtkY7vNia0pn49XOMJ+7sfHagkQKFy+QK\/8Oc32d85jiNxApfbwsUZFKGkNawt0kOJnS2UTBLggb63Mt+zYkXDceKvz0XbeI3yaIVxA2UdhGVZkQKVUkCy8ndpHKtxwSNyfOaAoi5e5ApnQ2hd8ifUsolI014uLS3cSLP5MC1vP3UFGZf\/rYLVE0S0mZhbJRUgkFhUCkFEBUVETsImWtt9uySHJTQkVtxKaHIUCa0tmEaLpnlqjYsHjxwTsV84zCG2L2Tp86ceTFBw8b9kRhPf\/LaMx6zrNBChNExUZhFX0VBZHyhSmWRImJVL5lkaxLJRVKga+DTQ+DUzSlswnecu8cSbGRovX2hStCxPjgKEwKm1vNrYLWI2Q3RM0qbDJ\/IQKXFBtvqwVLAZEKxivK1ImJlHV2n4yceIFY1bP7eEgPmx4GdyFTOpvgLffOoTMbGY3xb6uYcaukyPHfPZ\/1RG5ehxQ5TseRm1XM7H\/z9PyLFy\/R4ME30Z0jB+fSWvM55c93zqt+UXwOkYqCargyExMprm4cs\/tYpOQLwnxN+6aH35z9GPGmh1VVlURDy7Hp4XU\/0mFztnAuHX0u09hYxUpGaPIci5n1sIqg9Tzv58UHi9SZ99XbwCqUTqU7iaA9nVcZ9s\/\/\/fCrVDm8C5seqjdnoBITFalANQ2Z2EukRm7+lP+Sh5a7px1a1vezYbb0Dvn7Wc9Zy5B5813Tf80Dp9Rhc7bAlY4pA9i4g3ZiwxtT2g8patbzTun4c7fzMq9TWfIzr7xuZfCmh3x887Zj2PQwpvvK7TKxi5TK7eP9sPMa7tswfypt2riRRg3oeyPJ8rvPnMh7qe6zrX0+\/8B2rvtM7zROeTzb4yBoRSMrRLb+I3p+y\/+L5P8jKqj\/9TSe5VsS6LA5W5D6xpkWbNxpm8IGmx7GeUflv1bsIsXVkVO\/GxoayD6Tj2f81dfXk6rdItMyccIqWh9YBE2etwqlPCeFUApgPuHLiZZF1OS54nt71k0suXdSzlt0fu6S9O0DNu4WMIUNnkklfZfduH4iIiUvLwXJisNJuArFlbVND3MidqaV7CLH\/0txu3L0gCNaFi+OyK6UDKfBt91NRUX9iYWMIzIpbIXaJM35TemIo7CBKWwgUlF4R7gyExWpcFVWlwsv8\/awFMJ1XdBkxHbl1HHq6vgD0Z+O9AEuRazknp7IiwXMGoWps5CeJZnSEUdB1xQ2EKkovCNcmRCpmhplQ4vhTKBnLmtnU3SxU1SShezK0YPi7663DpA9EsuKeJnSEUfheaawgUhF4R3hyoRIQaQcPcdvZyOHE7uOHiCOwtzEa\/Ck2cZEXX7ZhLsl053LFDYQKX38ECIFkSpIpNxcWQ4hcuRlFy5rxJXGoUJTOuIouiFT2ECkovCOcGUmKlLWl3mxfXw4A0aVK4rOxku4ONpKg2hFwSYqO8ZdrilsIFJxe4779RITKWwfr48TONUkrs6Ghevia7scoy1dRSsuNnp7iHPtTGEDkdLH+xITKWwfr48TJClS9ms7iZZ1eHDoY6sTB2dKRxwFSFPYQKSi8I5wZSYmUtg+PpzB4sqlS2cjRevigZ25d75YtJKMsnRhE5cvBLmOKWwgUkGsHm3axESKmxXHArP58OE9KXc6OnY2blFW3IKlI5touwn\/pZvCBiLl3+ZRp0xUpOSmh\/PmzaM5c+ZQS0tLoHeW7JsmWkVv8uTJtH79eiopKcGmhyG8KA2djRStCy+uES2Ma1gwDWxCmFxJFlPYQKSUuIOSQhITKR7ue+qpp6i2tpbeeOMNam1tpalTp9KuXbvoiSeeEOKS7+A9orZt20bz58+nFStWiKR8rqKigqZMmSK25Jg5cyaNHz8emx6GcJW0dTb5hgVVP8dKG5sQ5g+dxRQ2EKnQLqA8Y2IiZZ84weLy+c9\/ntauXUteO\/NyBDV27Fg6eLBn9QMWKRlF8d9VVVUko6zq6mpsehjCbdLc2eQbFhz8UG3B6w+mmU0IVwiUxRQ2EKlAZo80cWIiJSOpRx99VEREq1atEg31G0nJyMkqUnKb+HHjxgmROnTokIjO9u3bJ4b++LBvelhTUyM2PaysrBQbHvIPDiLubDo7O2nUqFGeUa3OvHgpp0sHdhKviCHXIbROvOh3V1Xg6pvCJnDDfWQwhQ3vJ8Xvbg4aNAibHvqwe5RJEhMpbpScuMBDdvxcyioyfhrNw3sqREpei4ceZ82a5efSxqcxcmO\/C2107fW9Pbbbv6XnN+\/R9cA06ndnJZFPwTKSjSKPNoUNi1Rzc7MQKX6+rerQgc\/u3btFMCAPVdsiqWJkLydRkfLTKDlVnR2mtLSUGhsbiSMlPuwixZMvwgz3bdq0icrKyhBJWQxiyuZ1+Xzs2h8PiwVz5cQLIVhDy8SK7kMf65mM4XRkgY2fe9NkNtj0MKwHqM+XqEjJyQ\/WZk2YMIF27NhBw4YN82ytVaSkaGHihCc2XwlMebbgq7HXtyvhlS\/8vI+VNTZ+GXI6U9jgmVQQq0ebNjGRsu6Yy7P7eCLEiRM927Tbd+t1Q2AXKesUdOusv6xteqjCZUzpbMKw8Fr1omTKCjp16hSNGTOGiouLw1zC2Dym+A1ESh8XTVSk5Ey+Y8eOiZl6\/FzKz+w+VfjwMq87SVM6GxW+4jS9nYcGB0+qFbsWq57irqLOSZVhit9ApJLyoL7XTUykeFx\/69atQpjOnTtHdXV11NHRQUGG+wrFCJGCSAX1IRasC\/u308VLl25Mvrj+IrHcM0vFNPeg9dIlPUQqvyV04pOW\/i8xkWJT8goTAwcOFBMhGNjSpUt7TYyI+sZLi5Gi5uBUvk43UxLtz3dNKxvetZifZfGRm4BhW\/0iDduPqGJsit8gklLlEYWXk6hIFV79wkqASCGSCuNB+TpijrTcdinm97N45iD\/NlW4IFKIpMLcU\/nyJCpS\/MJtfX19r\/phuE+1icOVZ0pnE671ajsa+UyLS823SzF\/nnbxMsVvEElFceeEKzMxkbIvYxSu+oXlQiSFSCqMB6noiPMJF9fJuliuFK\/+IysKXtIpTHuD5FHBJsj1okoLkYqKbPByExWpOGfyOaGBSEGkgt8y0b0LxMLFyzjxC8ZOUZesq5OA8Wc8lJj0AZFSG4VHac+09H+JiRTD5+E+Pvy+F6XaYGkxkup2+ynPlM7GT1uDpkmCjZOAdZ9pzW0EaRUwEYmNrKCSe3pEq2jkWBGBxSFiSbAJaj8\/6RFJ+aEUT5rYRcq+0aG9mXgmFY\/hva5iSmfj1c4wn+vGRgoY\/+4+c0II1wfi774iJkRrRIUQsf78e0SFQMDPwlREY7qxCWNfzgORCktOfb7YRUpFE6wTLqyihk0PVdDtKcOUzkYdkRslpY2NXcS4JTyBQ9iZV4d3OMIKWdrYuPkHi9S\/PfUv9NfyT9B\/WrRSmRvpxCctI0mJiNTx48dzL+9ad9D14wmcd926dbR582axvh8vjcQvAfNWHPxyMNbu80PRO41ON5N3beNNYSIbJyGTEZlfIeNhRV6J4y8335X6JaNYpE5+4w56585\/ps88+ZIyB9PJdyBSLmaVq5rzrrm8OaHcTTfscykG3dTURMuXL6eFCxeGWgUd+0n1NRbfTCbsJ6Wsd7EUlFU2PKlDDCleH0oUEZmMxP50pA9qpwke\/Xil+WHlUZhFaZm8VUfH4nH0l9s\/S5\/61+eVla2D7\/B9zT\/t7e20bNkywlYdNvNad+TlSIgjo1deeUUITJhDihzv6ht200N5XewndcMCOux7E8Yf4sgDNi6UL7QRs7n85qt0c\/d79B+Kioj+eJjoQjvRhbbema5viyL38Aqyn1ccNmaRGrj2P9KfPzaF7vyX\/6rskjr4DvaT8jCnk0gF2Y3XWrzcIl5uHx9WpLCfVF+jYc8kd0cGm5BszrfRtQtt+afYW\/b04skdcj1EZSrhsyAe7ru8cDS1\/d00+sdVz\/nM5Z1MB9+RkdSRI0doy5YtiKTsZgsqUm6bHtqHCe0vB0sBq66uFkOKvEdVSUmJ2D5eDjWmZUzW2\/XVp9Bp7Fx96worEWzc+YVh4\/V+mHXYMK4VOVikzs29hY7dN5cefuLZwhzGkjsMH2UXtxWUlv4v9okTKqags+iw+PAzLeshhWvKlCk5MRo\/fnxuGJDTWiddpMVIUTlpvnJ1upmSaD\/YhKOu0m+s4uW0nFSUq85DpMLZP4pcsYtUoY2wbmAoy5IzBDnq4i3keXV1bHpYGGmVnU1hNdEvN9iojaSCWNi6nJR9F2W5eK+K\/b34OicX3IFIKohxIkqbOpFSyQGRVHKdjUo7xl0WREofv7FGW3bR4kgr7PCgFKnf\/OMymrpsozIX08l30tL\/QaRqarR\/cKjsDglQkE43U4Bqx5IUbPQRKXtNrJGW3N+Ln2mxYPF7XH4nYkCkYrmVfF0EIgWRcnQUdMT6dsS+7uyEEunmN1K07ILlFWHx+1+n1jxEiKQSciTLZSFSECmIVMD7ULeOOGD1I02uMxs3wRr8UG2fLVB46PDs9+to\/+d+IJ5vqzp04oPhPlVWjbCctBgpQgSuRbe2thK\/v8YvOPNSUzhuEAAbd29ICxuvCIsF6s1\/f5OO3zdPqUjpxCct\/R8iKURSjj1OWhw4CfEEG3fqaWTDQ3uXDuwijp744GdYLGJvlX6O2u\/5ilKR0omPTnXJdx9DpGpqSK7dp6rD4ze5y8vLqayszHeRXnnyfe72mdN5+zm3\/7nivK5XGthwXYPy0ZGNVzucnMnLb7zK9Os7Xrzk51H5jVc73G40Lz7Wz7vP9mxzwlucDBw0kH52brDYDeDLX\/5yr+K9WDjVNWo+Xu104rN3717iH6zd57ubjj9hW1ub6IjZwDhAAARAwEqAFwzg4+DBnp2STTwqKyuJl4XjL9W6HpmOpNgoLFT8gwMEQAAEskaAxUlngWJ7ZF6ksuaUaC8IgAAIpIkARCpN1kJdQQAEQCBjBCBSNoPz+n8vvvgi\/f73v6e5c+fSHXfckTGXyN\/c7u5u8bD1C1\/4Ag0dOhRsLAQuXLhAP\/7xj8Xw8YwZM+i+++4Dn+sE3nvvPcGGGfFrDUEmFWUF4ssvv0y33XYbfeITn8hKk321EyJlw\/Szn\/2Mhg8fTuPGjRM31ezZs8UWHziIeGXoZ599lvhdjyeffJJ400ocNwjw9jB33323+PnhD39IX\/\/61yHk1\/H89Kc\/Fe\/b3XrrreK+WrBgAX30ox+F+1wncOzYMXFvTZs2rc\/uDlmHlBmR4i1CrJsisuG5U6mvrxc+IKdhcufypS99SXyj4ZdZeYV10ztjv2w4yvzb3\/5GL730Uia4yM7BLx+ZnqOFH\/3oR+L9mptuusnoPiYIG46mfv7zn1P\/\/v3p0UcfpX79+oFNVRVdunSJfvKTn9CoUaOEv9i3IDIako\/GZUKkeIv6uro6gaOxsVFESXxO7i3F32Kamppo\/fr14jcL04gRIzIhUkHYyIgyK+LN\/hKUDwvU1q1bxcaavJeZyUdQNn\/961\/pz3\/+M3FUxQJu8giFXzYNDQ104MAB+vjHP05nz54V7gKR6n3XGC9S\/E2Pw+iHH36Y1qxZQxs3bhQixVHUoUOHhDBxhCCjrKNHj9I999wjhia4M54+fTrdfPPNRvY1QdkwNz6yIlJB+QwZMkT4Gg8Rm\/7MJSib06dPi\/uKn2P+4Ac\/IN6Y1FRGQdgsXbqUfvvb39Lbb79N7e3tgg+\/QG96BB6kQzVepCQM\/mazfPnyXiLFz1ZWrFhB1q3neZiPh\/zYWbhTfuSRR4LwTGVav2zkN7ysiFQQ32Hf+vWvfy06Gn7uwoL12GOPGd\/Z+PUdHjLnL4bc+fLfX\/va16ioqCiV94vfSvtlI+8rXqYIkVRfuhApm0hlMdQOejP5vUlNSQc+7pYMwoafZ\/KP6eIU5MsNf0nOYp8TpG\/ItEg5DffJIa0gENOe1qmjAZsbVgWfYCIF3+nhBb9R0zNmVqTcJk6Y\/DDXzWXsNxPY9CYFPv5FCr7j\/uUGbMKJVmZFinHJKeilpaW5WX\/hMKY7l70TBpv8IgU+7h0x2ICN6t4wMyKlGhzKAwEQAAEQiJ4ARCp6xrgCCIAACIBASAIQqZDgkA0EQAAEQCB6AhCp6BnjCiAAAiAAAiEJQKRCgkM2EAABEACB6AlApKJnjCuAAAiAAAiEJACRCgkO2UAABEAABKInAJGKnjGuAAIgAAIgEJIARCokOGQDARAAARCIngBEKnrGuAIIgAAIgEBIAhCpkOCQLZ0EeO+wlStXUnNzc68G8CZ8vCJ1mg9e3uqVV16hOXPmiDZOnDhR7IfGh2w3b8botOo2f86bNc6bN8\/4najTbOMs1h0ilUWrZ7jNsrO2duAm4LCKDC+SHFSkmIEUuYULF5qABG0whABEyhBDohn+COQTKblb88mTJ2nGjBl0\/\/33U11dHXV0dNCECRNox44dIsrgzelqamqIFyaeNGkSDR48WEQgHMHI\/YG4LLmpplzImGsoIzYuY9u2baLSBw8epMmTJ4tdollgNmzYkPtsz549Ik1TU5P4nA8WIHtExOWdOHFCRE5ObbRGUnw9eW0uz3rtp59+mr74xS+KDT9xgIAOBCBSOlgBdYiNgNNwnxSg\/fv30\/PPPy\/EiI8lS5bQqlWrRIctRccqRpyPBYPFyk2kqqurHQWGy+etwxsbG2n48OE5gePzLFJch3PnztG6devoiSeeoO9973u0evXq3LnNmzf3GpbjPHwtHspzG9Lksln0rMN91nz8ObeTDzlMGJthcCEQcCEAkYJrZIqAn0iKI5a2trZcFCUBsSg9\/vjj9Mwzz+SiKifxYhGQ5ysqKqi+vr4XY46mWFCkGMnhOY6OOBqSEZg1kxQTGXlZn59xm5566imqra0VguoVSUmRsgsUl80RGUdaaX8+lymnNryxECnDDYzm9SYQRKQ4irFHLNyJS3HhoT8\/IuUkOtZy\/IiUFA9ujYyYZMvCiJRbxASRwh2jGwGIlG4WQX0iJeBXpDgdP2PiZ1M89CWfVy1fvpx4YgFHGtbhvkWLFuUmK0yZMiU3DMiCIof1ysvLc2nGjh3rGEnZh\/v4ehs3biTOy7Pvfve73\/URTpnHPtznNruPozW3IT0M90Xqfig8BAGIVAhoyJJeAn5FiqMbnu3md+IEi5Z1p2c5ocJ6nqlZJ044DffJoUI5RNjQ0JB7PmSdjGG3gDUCyjfc98gjj4jhypaWllwR8pkct9k6bJheK6PmJhGASJlkTbQldgL5hENlZeJ4zwlT0FVaDGWpIgCRUkUS5WSSQBwidf78eTH0ePvtt+emqTvBLuR5kv25ViaNiUZrSQAipaVZUCkQAAEQAAEmAJGCH4AACIAACGhLACKlrWlQMRAAARAAAYgUfAAEQAAEQEBbAhApbU2DioEACIAACECk4AMgAAIgAALaEoBIaWsaVAwEQAAEQAAiBR8AARAAARDQlgBESlvToGIgAAIgAAIQKfgACIAACICAtgT+P6WWEzpQeItmAAAAAElFTkSuQmCC","height":256,"width":425}}
%---
%[output:7a92d0b3]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAakAAAEACAYAAAAJP4l9AAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQncV8P6fyRLSJRCsitLZFeyRVxuuPdak1zJnqXQouXekFRUSAht5CKU7hXZs0eWyHJFruVSSbaSnfp\/vvO\/83Pe855zZubM2eb8nvl83k+973nmmZnvM2e+Z2aeeWaVlStXriROjAAjwAgwAoxAARFYhUmqgFbhKjECjAAjwAgIBJikuCMwAowAI8AIFBYBJqnCmoYrxggwAowAI8AkVYV94IcffqC+ffvS9OnTQ1t\/5JFH0rBhw6hevXqJITR\/\/nzq2rUrLVy4kO68805q06ZNbN1eXUFK\/PV\/8cUX6cQTT6SmTZvSxIkTqXnz5rHLzjrj3XffTf369atRbJB9rrzySrr55ptJZTuv\/c866yy6+OKLhe6gfgE77bzzzqK\/tG3bljp27BjafIlxFD5Dhw4VOoLqGlZ\/tH\/WrFmJ98es7cjlxUOASSoebk7n0iEpNBCD0\/jx46lhw4aJtDdLkvLX30WSUtnJbx9bkvKTIQh91KhRNHjwYJo7dy5JggnrDGmQlG6bEumgrKSQCDBJFdIs6VZKNfh5S7ed8Xh1ZU1SKFsOrC6SlM6g750J2Q7okqS85PfVV1\/RaaedljhJBfVwk9lVum8Iay8SAkxSRbJGRnUJW+6RxXvJxP\/1HLTMFkRkfiLEEtSpp55K5557bq3lPr9OnRlcFOF5B1Y5iHtJ6oYbbqAJEyZUljuDygtaYvNj4S1HYhdWdz\/hqJbkoE\/Wwb9E6cXWW553kPdiDV1hy3r4e\/fu3QOXf\/fee2964YUXavXKsA8Xkw8BFSH16dOHzjvvPEGO3uRthwpTL+meccYZQp8fi4xeOS7GAgEmKQvwXM1qQlLeASlo4JYYeAePoMHbj5XUGzZbUO0dxSWpMJt5SSOqnZKootroJ6owfSoy9ubz4hvWBjnwhz2Xdffb30WS0sE0TCbJ1QFXxwCX6s0k5ZK1Eqqr7nKfd+D2koJ3wPQOBPLlD\/qbf1CHbIsWLSpLSVKnikCDZntRsPiX+yAbREiSFBs1alSpk2yPt04yL77w\/Y4YQTPQoL\/pLqGFEWEYuXlJKoiQZN2BgXScCbJlWst9qqVJ1exKOvLoYmpK8gm9XqwmYQSYpBIG1AV1uiQVNIvyD5BRX+VhHnbACLqRgjzu5Owqaqah8u7zk5F3xuZtl2qfLGxJSZIUyomqZ9A+D\/LIv6uW\/aLa6Z9tykE+bCYn\/w6PTZdJShfT+++\/X3hFqmblLryz1VxHJqkqtL4uSQGaKJdhCZ33C3jgwIE0aNAgsd\/jX6LyE8LHH39cy7Xaa46owUVFUv6yw\/ZLgkgqymEhaDbirbN\/30q1BKda8pO6dWZVUS7cGKzLQlK6mD722GOif+liXIVDgRNNZpJywkzJVlJnSc3\/VT527NjQMzh5k5TOHoMuSWG5T57lkjM+eU4IxOuf+YSRh6yT7oBq6uYftKRaFJLSmbnoLu1FyYW9FZKUmKSSHTfy0sYklRfyOZYbh6TCXvg0lvt0oFEt0\/l16JKUXIL0fn0H7UkFHXIOkpNLTqZf817yC1oS9M72VLNd\/\/JY2st9aZNU2HKf3+a6cjr9jWXyQ4BJKj\/scyvZZLlPDpCffvppZYZh4jgR5g0X5jgBUHTO+6RNUnIWhagYXkKQeITtd\/hnoF9++WUFtzDvOhnxwd8hVLMwyHsJoVpmUkGOE0EfXkxSuQ0xiRbMJJUonG4oMyEpXRd0716MiQt6lLt31DJeWiTl9TgMsqYkKWAoD7kGyXnxCCMb1YxDB0eVxxzqZjOTCguVFBTSKslzUtKTz98\/ZHt1MGWScmM8UtWSSUqFUAmf65BU2AAa9zAvlrswY+jVq5fyMK9q8IZJ0iIpDL7+NoJwNt9880BPxKDBMohc\/c4YJst\/QUQehFEaMylg7a97WHikNEjKT9TepU8VpkxS5Ri8mKTKYUduBSPACDACpUSASaqUZuVGMQKMACNQDgSYpMphR24FI8AIMAKlRIBJqpRm5UYxAowAI1AOBJikymFHbgUjwAgwAqVEgEmqlGblRjECjAAjUA4EqoKkcBAVP5wYAUaAEWAEfkegWbNmhJ8ip9KTFMipd+\/eNHv27CLbgevGCFQtAnXr1g1s+6qrrkprrLEGrbbaauSVkf8Py1e1QMZs+O23315ooio9SckDf8OHD6dNNtkkphnzzYZo4fPmzaP27dvXeFnTrFUaZdrqNM1vIq8rq5JTPcfH0qhRo6jM\/XHx4sWia65cuZKeeOIJsYrx2Wefib+BWFwjl4022qjWq\/brr7+Kv0XNQvz5kOenn34iBBNeffXVxQ8I2PuDQ9qrrLIKLViwQLzz2223XQ28fvvttxp1+eijj+jbb78VctAjEwge5eM59Oy44461cH\/++edp6tSpNGnSJAqKIJLm+GKiu2pISidStglwWcr++OOPtGjRItp4441pzTXXzKToNMq01Wma30ReV1Ylp3ouP5pc64+SZND5HnnkEXGtPAbFL774okJAaXbMDTbYoEJy+I8kALwTa621lhiAW7VqJSKDYIAOSl7SwOC9ZMkSaty4cY13KoiQgnSp7GybR1e\/Si7quSt9MXeS8oZ80QmH4zU+wtcMGTKERo4cKb5OgpIrhkjzBWfdxUEAg+Ntt91GXbp0oS222KIwFZMkNGfOHELEe6TXX389sfrJwV\/+u9VWW4mbmTfccMMK4XjJJ7GCWVEoAq6MjbmSFAhq8uTJNH78eEEy\/t+j+peM6QUZmZ9Jit\/IoiOg+vJNs\/6SiJ577jnCUg9+986QTMr2kg7+j+Wm1q1bV1TozkhMymTZZBFgklLgKUnmhBNOoI4dOwppGfi0bdu2lb9FEQ+eqQJ1umKIKLjyGNjSKNNWp2l+E3ldWZWc7fOkhiGQz7333ksffPCB8YzIS0Bbbrkl7b\/\/\/qJa8u\/rrbee88vPKjup7BAnv0keXVmVHC\/3qSwZ8RxLdX369KGrrrqKmjdvXpHEbGrWrFkkQ\/X7VUjSQSRmJO9MjGdSFgbhrJkgoBpUTCsBMsK7dN999xmRkSScXXbZRSw9eknItA4s7yYCrnzA57bcB4BwtYB\/qc5kyU9H1hVDuNnNudamCNiQFAgJHnP4sNPdLwIZ4QdXXGBJXf5uWm+WLx8CroyNVUNSPXr0EGvm\/JKW72VzqUUgKZANHAaCrqD3tgVyM2bMEDMlFSlJ77c\/\/vGPdNBBB9VwRnAJH65r+gjIvUi4ueMMadE9TauGpKTpsbRx8sknp98TEiwBZyukuywON2aR0ijTVqdpfhN5XVmVnM1zuHNPnz6dnn322UgTS0I65ZRTqEmTJkJW\/s3VvqGqtwpXVX7\/c1t9cfKb5NGVVckFPce5KHiYysQkFdJ7sl7uk4cnXZxJwaEEByRR96zOSaVRpq1O0\/wm8rqyKjmT59988w29++67NG3aNPFvWILdd91118oMKUtCCqqTqo2mhKEjn3SZtvri5DfJoyurkgt6LmdS8mA5k1RID4zrOOFVx3tSOq83yxQJAZyTglOQipQOOeQQ6tChAy\/bFcl4JatL6fakENID4TeQ6tevbx3axMYFXfaVaiEpm832uO9VGmXa6jTNbyKvK6uSC3qO0DM4mxS2r4SZ0h577EGdO3d2gpRUGMTtc1H5ki7TVl+c\/CZ5dGVVcqV0QQd5PPTQQ\/T9998TNmERm+rll18W7uKIS4aEUCR9+\/alww8\/nOrUqRO7T4JkRo8eTRMnThRu6DqkU40zKVVHjG2AiIxplGmr0zS\/ibyurEoOz5988kl66623hNNDUMJyHd4r7I+6uPyswsCF\/mjbhjj5TfLoyqrkSkdS7733HnXr1o0+\/PBD0c8QpmjgwIEiICZI67DDDhN\/f\/jhh0XMrjFjxtC+++5r1SdVYZHgpo508cUX1ypHh9RcmdJagciZc0cAs6THH3+cHnzwwcC6gIzatWsnDp\/Dsy\/LOIy5g8MVKCQCroyNFe8+LOeBEGbOnCnOYSBqLjbULr\/8ctp9993FjEceAMTG2\/nnn09bb701DRo0SETzLWpyxRBFxY\/rFY4A3oN\/\/vOfYgUgKOF96dWrl3h\/ZFJ9+TLejEBWCLgyNlZIatmyZXTeeeeJGFz9+vUT4eI\/+eQT6tq1q\/Aqkn+TAILQXnnlFbrlllto\/fXXzwpX43JcMURUw\/IY2NIo01anaX4TeV1ZyCEC+I033ihWE\/wJS3lYGt9pp50CPTF1yzHu6BlmyKMNSZdpqy9OfpM8urIquVIt90lHBtwrIpfWgv7mJSkQQFRw1wzfm9CimKTiWUHV+eNotdVpmt9EXiWLWRM+zIKcHzBjuuaaawQkqrh2qnLi4Jp1njzakHSZtvri5DfJoyurkmOSYpLKenzg8jJCQEYHx6FH7MH6E4jpuOOOIwRDNon4rRpUMmoeF8MIkCsf8JXlPp5Jca9lBP4fARDUhRdeWOsaC5AR9pdOOukkI2Ly4sokxb2sKAg4S1KIigynCKSlS5dS9+7dxRkO+TcJMBwpsPTBy33pd7k8BrY0yrTVaZrfRB6HbMeNGyfuWQqaNWEJHO+GSqft8\/R7k30Jqjbal1BbQ9Jl2uqLk98kj66sSq6Uy31z587V7mOqu5y0FaUo6MrXQhQEqo6YBnxplGmr0zS\/jjw+tLDX5L\/8D7MmxMY79NBDa8Cr0mn7PA1bJq1T1caky4O+pMu01Rcnv0keXVmVXKlICoEI33jjDcK\/ugnBTlu1akVZBT3VrZdXrgwkFafdnCccARASjlcgmGvQrAlOECb7TCZYqwYVE10sywjYIODK2JhbFHQbcE3yumIIkzaxbDwEMGuCI4TfQw+EdM4554ioJ2mRk6wxk1Q823Gu5BFwZWyskNSKFSvEHhQuVdNNOEvVoEEDq9BIumXFlXPFEFHty2NgS6NMW52m+SH\/5ptv0m+\/\/SaipgQt6enuNemSjKqOqudx+3mW+fJoQ9Jl2uqLk98kj66sSq5Uy33Su4\/3pLJ83fXKUnVEPS1mUmmUaavTJD8I6d577xXXqntTWDBXXd0qOdvnZlbKR1rVxjRqlXSZtvri5DfJoyurkisVSfn3pDCjQiyyl156Sbjcbr755qLv4ZQ9lkwWLVok4vq1b98+8z0pOTuSL0PUfShlmEml8dKXVWfUkp4\/RFEeGKgGlTzqxGVWJwKujI2he1L4AkU05yuuuILWXXfdGlZcvny5CJOEr1Isl9StWzczKwNYDDYycrr\/d39FXDFEZgCWsCC5jBd2tkku6RWh6UxSRbAC1wEIuDI2BpKUjOPXsWNHcR1HUEK051tvvTXT2H24ZRJXhCA6uzcqelSkdFcMEfXa5DGwpVGmrc6g\/CCoTp061YIPZ5qGDh0qZvw6Ecd166aSs33uwvCpamMabUi6TFt9cfKb5NGVVcmVarnP27G+\/vprOvPMM+nYY48lEFVQmjBhgrh3KssAs3LfDASFGIMyhV1F79LXApOUemjzvnC4qwkHyr0JM\/sDDzyQ\/vSnP4lZvuoF9ubVlVXJ2T5Xo5C\/hKqNadQw6TJt9cXJb5JHV1YlV1qSktd2zJo1i66++mradttta\/S7d955R0RMx\/XWPXr0yGy5L+zK+aglvzLMpNJ46V3ViVmzP5aeN8Br2i7ktripBhVb\/ZyfEdBFwJWxMXRP6vPPPxchkebMmSPulmrdurVo++zZs8Wto5jJjBgxgpo0aaKLibUck5Q1hE4qiHKGSPPgbRpgMUmlgSrrjIOA8ySFRn\/33Xc0depUcbEbiAkJhIU9AMyi1l577TjYxM5jQ1KY8YFoXb2uG3svuNG1Xr16sfEzyYjBNOkyTXWiz40cObLW+Sbc2dSzZ08RSy8qmZSnK6uSs31uYqO8ZFVtTKNeSZdpqy9OfpM8urIquaDneK\/xs2DBAurdu7eIvuLdPknDfjY6KzMp7EPhhtE999yTdthhh8wGQ5PK25CULKdLly508sknmxSbuyyOByxZsoQaN26cmbt\/GmXq6MQRBzjuDB48uAbuICZcyLnXXnuJyBA6obh0ypOF6Mqq5Gyf597ZNCqgaqOGCmORpMu01Rcnv0keXVmVXNDzSZMmiWNEMjlDUvCcQ8VxABI38uJKgqOPPpr22WcfMfuoU6eOccdKOoON48Tw4cNpk002cXImlTSORdSHLzt4iz799NM1qoe+d\/bZZ1eWm4tYd5M64T1bvHix6IdrrrmmSVaWZQQSQUDOpLB1g0gszpCUbD0O8eIlevXVV+lf\/\/qXGDRwTgoeU1jiy3OWVa0u6In0zAIqkeebgm68lWfwVEt6BWxWZJV4T8o1i5W3vqXYk4J5fv75ZxH\/DNEnHn300dxnWX5gq+Ewbx4DWxplenV+8803oRcLhjlDmNbJRF5XViVn+9yFIVHVxjTakHSZtvri5DfJoyurkot6XhqS8nY4OcuCazrcgLGPdfPNN1PDhg3T6JehOqstLJKqI6YBfhplQicimYwdO7ZWlQ877LAaB7SD2mRaJxN5XVmVnO3zNGyZtE5VG5MuD\/qSLtNWX5z8Jnl0ZVVyVUdSsvNh2e3ZZ58VRIXwSNjQL2py5WuhqPglUS8s64Ut6Z1++unUsmXL1K\/ISKIdSehQDSpJlME6GAEdBFwZG7Xvk8IsCg4Vd911l9ho+\/bbb4lv5tXpCtUrg48YOOMEXZHh2vmmpKzIJJUUkqzHFoHSkBSiTyAS+vXXXy8CEiIhFhouiYMzRf369W2xSjW\/K4aIAiGPgS1umSCkadOm0T333FOrSXAjP\/XUU0XoojiebaZ1MpHXlVXJ2T5P9WVISLmqjQkVU0NN0mXa6ouT3ySPrqxKrtTLfTiXg8FGXsshewwOVx555JGZhUKy7fBMUvEQVHV+r1aVlx6uesGRhvXWW0874GtQrU3qhPwm8rqyKjnb5\/GslW0uVRvTqE3SZdrqi5PfJI+urEqudCSFWdMrr7xCCB771FNPEX7H4V4cfsW+Aa5C8Ad3TaNDJqmzDCSVJB5J67rpppvEIXB\/KqsLuS1+qkHFVj\/nZwR0EXBlbKwRcaJbt25iaQ+EhIO8RxxxRMUpIuwgrS4gecm5Yoi88IlTbpQjxB577EGdO3euGkcIU\/yYpEwRY\/m0EHBlbKyQlCQhhNHAFR2HHHKIiNAgI00wSaXVVdR68xjY\/GWGERNqrztrsm2HaX4TeV1ZlZztc3VvyF9C1cY0aph0mbb64uQ3yaMrq5Ir1XLfihUrCLHxEExWeu9tueWWgrDgIIFYaWeccQYv96XxBip0qjpiGlWSZU6fPl3sTQYt5yFg7wUXXKBdvG07TPObyOvKquRsn2uDmaOgqo1pVC3pMm31xclvkkdXViVXKpLydiycg3rhhRfojjvuEOehsDe16aabClficePG0X777ZdGP0xFpytT2lQaH0MpbIzbbK+66qparuNy1lSt7uMx4KyVRTWoJFEG62AEdBBwZWxUnpPye\/nVrVuX2rVrJ1yJsf+A34ucXDFE3hjiWowrrriCiSllQzBJpQwwq9dGwJWxUUlSssVYDnz33XeFSzqWf3755RdxbcKYMWNo\/fXX1wYma0FXDBGFS1oD25QpU+iGG24ILBpnmrp27SoCCm+xxRaJmM22Hab5TeR1ZVVyts8TATplJao2plF80mXa6ouT3ySPrqxKrrTLfapOhmgTzzzzDD355JPUv3\/\/zGP3qernfc4k9TsaWMq7\/fbbacaMGYEQwgECxw123XVX6zNNQQWoXiiVXU3zm8jryqrkbJ+rMCjCc1Ub06hj0mXa6ouT3ySPrqxKrmpJKslOiDM2iP+H1LRpU5o4caK41E4nwdFjyJAh4vbWsCC3ZSApHSzCZObMmSOICVewByUQ09\/\/\/neBH\/7PKV0EVINKuqWzdkbgdwRcGRu1l\/vSMC4IavLkyTR+\/HgxSPp\/jypTusRDRuYPknfFEEnhi9kSlmQRNy8sgYzgwSkv30uqbNajRoBJSo0RS2SDgCtjY24kJUnmhBNOoI4dOwqryEsN27ZtW\/lbFPHgmSrIrSuGiOqWUQObLinh8sBDDz2UdC8RTGMwtdVpmt9EXldWJWf7PJvhya4UVRvttAfnTrpMW31x8pvk0ZVVyfFyn0VvxFJdnz59hKuzd3kPsyncVzVs2DCqV69erRIk6QwdOlQ8887EyjqT8nY0XBYYdO2Fv+2YLfXq1UvEzIuTVJ0\/D52mdTKR15VVydk+j4Nr1nlUbUyjPkmXaasvTn6TPLqyKjkmKYveCLLBYOtfqjNZ8tORdXUmhRkSfnAbMs4the0pSROAlDArxQFb3luy6JgpZ1UNKikXz+oZgQoCroyNuS33ZU1SPXr0qAzgRRvEZRRxkC7+ryIkuIcjDRgwQHjhFa09PA6EIwCSgo033HDDwJUCxo4RSBsB+QG8YMEC6t27t9ifbtOmTdrFxtZfNSQlEerSpYtws84jffHFF6JYuO9juRPnznQSiAghqhD0F\/EUs0iI4YiD3Lh1GSGxkki2Ok3zm8jryqrkbJ8ngXPaOlRtTKP8pMu01Rcnv0keXVmVXNDzSZMmCecqmZikQnps1jOp4cOHiwEes440Zx5yVoRo8jNnzqQvv\/ySJDnpvLyoGyJ5HHDAAaKemDXBoUR64sW5LFCnXL9MGmXa6jTNbyKvK6uSs30ex1ZZ51G1MY36JF2mrb44+U3y6Mqq5IKey5nU7NmzadSoUTyTkl58c+fOrcHcjRo1iuU44X0B8tiTkiT0wQcf0L333iuqo1qeC3ppJVkedNBBIi5i2uSZxsDBOs0R+Oijj8RXLGb0SUXyMK8F52AESNy0fuKJJzJJhXUGGxd0qTMpkpLEA70gU5CO\/NrwPjPt2HLGBrdvODVg2SzOLO7TTz+lqVOn0jHHHEPNmjUzrUYs+TTKtNVpmt9EXldWJad67srAENVpVG2M1eEUmZIu01ZfnPwmeXRlVXJRz13pi7ntSaFPgmRGjx5diTKhQzpxZlIIhouLHBEM14Z0\/O8RosMjySjxuOwPCZviSSa5wSmdP5LUHaYrjTJtdZrmN5HXlVXJ2T7Pwra2ZajaaKs\/KH\/SZdrqi5PfJI+urEou6jk7Tmj2VFVYJLipI+Haen\/SITV8SVx22WU0b948kR3eVZJcpD7\/Pg9kkEBqkMXFj8uXL6eVK1fWqIJfj2aTWYwRYAQYgUIggCMr2K\/PaoUmTqNznUnFqXCcPCAq\/HBiBBgBRoAR+B0BkFORCQo1rQqS4k7JCDACjAAj4CYCTFJu2o1rzQgwAoxAVSDAJFUVZuZGMgKMACPgJgJMUm7ajWvNCDACjEBVIMAkVRVm5kYyAowAI+AmAkxSbtqNa80IMAKMQFUgwCT1PzMjAkbPnj2pf\/\/+2tfXV0UP4UZmhoA\/hBjuTJMXgmZWCS6IEfgfAvIS2unTp4u\/5BWIlkmKSEQk79q1qzDExIkTmaT4Nc0FARxcRzw\/EJPskyNGjCj0NQq5AMWFZoIAgiUg1iQCKYQFBM+iIqUhKbzUQ4YMoZEjR1LDhg1rYCdjVMk\/er8I8PU6duxYOuyww+jSSy+tdVNwFkbgMsqFQNy+6EVBfsV26tSJSapc3SPz1iTRHzGG3nXXXaE3pqfZqFKQlFwmAVD+m34BLq5RlzMk\/+8S3LDr7NMEn3WXD4Ek+iJQiRpYyocatygtBGz7o3fJj5f7YlrJO0vaeeeda5CUBLhp06Y1Yv8FxQNkkoppAM5WQSCpvsj7o9ypkkAgqf6Iukiyw9Jf1rf4Oj2TkkbABjPS5MmTa5BUGLBB66tMUkm8FtWrI6m+yDOo6u1DSbY8qf4o6yQ\/+Nu2bZu5M4\/TJOU1alBE9DDiCVryY5JK8hWpbl1x+yJQw4WIAwYMoHr16lU3iNz6xBCI2x\/nzJkj6iAdefr06ZPLnj2TVPPmwhBMUom9E1WvKM6gMGbMGBo3bhxJd18JYl77AFVvxBIBEKc\/Yg8f0dH79u1b6ZN59UUmqf+RVIn6JDclZwTiDgrNuS\/mbLlyFu96f2SS4oGhnG9mjq1yfVDIETouOgUEXO+PpSYpE8eJFPoGq6xSBIIGBe6LVdoZCtBs1\/tjqUnKxAW9AH2Jq1ASBIIGBe6LJTGug81wvT+WmqTQn6Qrptz0CzvM62Df4yoXFIGgQYH7YkGNVQXVcr0\/lp6kvIMDe0xVwRtZgCaGDQrcFwtgnCqsguv9sTQkVYV9j5vMCDACjEDpEWCSKr2JuYGMACPACLiLAJOUu7bjmjMCjAAjUHoEmKRKb2JuICPACDAC7iLAJOWu7bjmjAAjwAiUHgEmqdKbmBvICDACjIC7CDBJuWs7rjkjwAgwAqVHgEmq9CbmBjICjAAj4C4CTFLu2o5rzggwAoxA6RFgkiq9ibmBjAAjwAi4iwCTlLu245ozAowAI1B6BJikSm9ibiAjwAgwAu4iwCTlru245owAI8AIlB4BJqnSm5gbyAgwAoyAuwgwSblrO645I8AIMAKlR4BJqvQm5gYyAowAI+AuAkxS7tqOa84IMAKMQOkRYJIqvYm5gYwAI8AIuIsAk5S7tuOaMwKMACNQegSYpEpvYm4gI8AIMALuIsAk5a7tuOaMACPACJQeAS2SWrFiBS1dupSWLFlCn3\/+OTVp0oQaN25MDRo0oDp16pQeJG4gI8AIMAKMQD4IhJIUiOndd9+l2267jR566CH69ttva9Wwfv36dMwxx1DHjh2pRYsWtMoqq+TTCi6VEWAEGAFGoJQIBJLUf\/\/7Xxo8eDA9\/vjj1LJlS2rfvj21atWK1l9\/fdpkk01owYIF9PXXX9PLL79MTz\/9NM2bN48OPvhg6tmzJ5NVKbsJN4oRYAQYgXwQqEFSv\/76q5g5TZgwgf7617+KWRKW9aLSypUrafHixTRlyhSR9+yzz6bTTjstn9ZwqYwAI8AIMAKlQqAGSWF29Oijj1KHDh0IS3mmCUuCM2aTRJUnAAAfEUlEQVTMEMt\/nBgBRoARYAQYAVsEtBwnbAvh\/IwAI8AIMAKMQBwEmKTioMZ5GAFGgBFgBDJBIJSkvvrqK7G3NHfuXGVFGjVqRIcccojYj9pss82U8izACDACjAAjwAjoIBBKUj\/99BO9+uqrdPPNN9Prr79ORxxxBLVp00bofO211+iRRx6hNddck\/7whz8I93T8vu6669KYMWOEhx8nRoARYAQYAUbAFoFQkoLX3j333EPTpk2jkSNHCtdzb8Kh3gsuuICOPfZYOvroo2nZsmU0YMAA2mijjcS\/nBgBRoARYAQYAVsEQkkKpHPeeefR4YcfHuqtd\/fdd9ODDz5I119\/vZhF4Xe4ot9777229eL8jAAjwAgwAowAKfekTj75ZDrqqKMCocIsa9KkSTR+\/Hhq2LAhzZw5k0aPHi1mX5wYAUaAEWAEGAFbBEJJ6ueff6aBAwfSRx99RNddd52I1+df7uvevTttscUWNGjQIFpttdXEjOq9994TRMWJEWAEGAFGgBGwRSDSBR2E061bN0Icv+OOO06ESEJ6++23xZLeL7\/8QjfddJNwlLjxxhuF0wQIiw\/z2pqF8zMCjAAjwAgAAeU5KcTxGz58uIhEAVJCqlu3rojV17dvX+Fyjv2rSy65hHbaaSfq3LkzrbHGGowuI8AIMAKMACNgjYCSpGQJWP5bvny5+HWdddah1Vdf3bpwVsAIMAKMACPACEQhoCQpLPXNnz+fnnnmGVp11VXp+OOPFwSFs1NY\/lt77bULj\/Cnn35K+OHECDACjAAj8DsCzZo1I\/wUOUWSFGZOOCOF6OZIO++8s\/Dkw71RZ555piCtIKeKIjUY5NSvX7\/AyBmI+h6U8PewZ0VqG9eFEWAEGAEbBFq3bi22c4pMVJEkhXNP8NS76qqrBA5oDEgK90rNmTOH+vTpQ3\/5y1\/EeaqiXnj44osvEtzo\/YeRdQwryWqPPfYQe3AybbjhhjrZE5OZPXs2jRo1SuAfpx1xKpJGmbY6TfObyOvKquRsn8exVdZ5VG1Moz5Jl2mrL05+kzy6siq5qOfy2Z133lmJJpSG7Wx1hpIUZlEXXXSRcIYACaFBV155ZeVMFArGvVNPPfVU5TCvbWXSyA+SOvHEE+maa64hfDUgffbZZ2J\/berUqeJ3LF2aJkTWwE+vXr3EjBL\/TyvhGABms126dBEu\/1mkNMq01Wma30ReV1Ylp3ou+2PRB4aoPqZqYxr9M+kybfXFyW+SR1dWJRf13JW+qDzMC6Lab7\/9CA3yk9Szzz5LV199dQ3iSqOD2ug0NQQIDOn222+nhQsXGhOYJC9cGtm0adNUycsGF86bDwKqQSWfWnGp1YiA6diYF0bKsEjt2rWjU089tRZJIbYfDu+++eabgqjg8VfElIQhJHGhfT\/88IPYh8PfvH9XtR3kdeCBB9Jee+0liCvNmZeqLvw8PwR+\/PFHWrRoEW288cYiQDMnRiAvBJIYG7Ooe2SAWRzURSw+HNLF1R1yJoU4fY899pg4J4XrOfBT5D0pLPeltbziJStEgsfvusuHkqwuvvhiYesw4spjYEujTFudpvlN5HVlVXK2z7N46W3LULXRVn9Q\/qTLtNUXJ79JHl1ZlVzUc+dJCh0F+zY4pPvAAw+IARQD8DbbbEOIgP7ll18KZ4IrrriCGjdunEa\/TERnnoYAXrjy5Nprr41FXDzbSqQLFEqJalApVGW5MqVGIM+x0QRY5TkpDLKINoEZFWYIeMl22WUXcUVHhw4dCn9OqkiG8C4PAlPcy6Uz6wJZ7bbbboR9LiYuk+5dPFkmqeLZpFprVKSxMcoGSpJy3YBFN4QkLvz71ltviYsmdYkLRwCwt8HE5U4vZZJyx1Zlr2nRx0aJfw2SwvIeLjpcunSptn0aNGggolCU2XFCG4yEBUFccD1\/+OGHlZpBVOeccw7Vr19fzHRtUxqDqa1O0\/wm8rqyKjnb57Z2yyK\/qo1p1CHpMm31xclvkkdXViVXuj0pOEecdtppgdEZwjqejEKB+6SKmFz5WojCDh0NXpTY+1uyZIlwAlHNtkBa++yzj1iWRTKdbak6fxxb2+o0zW8iryurkrN9HgfXrPOo2phGfZIu01ZfnPwmeXRlVXKlIyl\/5\/r6669pyJAhtP3224urOvCVjgQ37Pvvv19Eoxg2bBjtu+++afTLRHSWgaTCgJCehfAqVM22QFJt27YVdjQlrEQMwUoEAqpBhWFiBLJCwJWxMXRPCiGB4HIOd\/OgsEfynNT7778viKpevXpZYWtUjiuGMGpUhDBmWPPmzaObb745UiWI6uijjxYHtZm0kkJfrYdJSo0RS2SDgCtjo3bEiSDY4KGGM1Ty+vhsoDUrxRVDRLUq7sAmnTIQZxERNKIOH4OoLrzwQnHYOK0v\/rjtkNiY5jeR15VVydk+N+vd+Uir2phGrZIu01ZfnPwmeXRlVXKlXu6Tsfu23XZb6tGjh7jo0Jsw08JSIKKMlz3iRBovXR46JUm98MILImpGVAJp4ZAxR8dI1lKqQSXZ0lgbIxCOgCsf8JEu6Pfddx8NGDBAXMuBK+HlshAGO0RIv+WWW8RVHjgvVdTkiiHywE\/uaWGJUF7HElQP2P2UU04hRH9PwnMwj7YWpUwmqaJYguvhytgYSVKYLWHwwlUd8up4adrVVluNBg4cKMjLP8tSmR+OFwip1KlTp1oh4v0ehmeddZb4ovcmCa78W1TII1cMocIsi+eStGDzKO9BkBaiyiPxfpaZZZikzPBi6fQQcGVs1DrMCy8\/REd4++23BWK4kXfPPfesePuZwgiHDGzs+8lFEtQJJ5wgyM\/\/O8oBsLgeY+LEidS8efNav\/vr4oohojDMY2BDtG44x8ydO1fcZRWWQFLnn38+bbXVVkrCsm2HaX4TeV1ZlZztc9N3KQ95VRvTqFPSZdrqi5PfJI+urEqu1HtSaXQ0\/yzJT1JYQpw8eXINRwzvFSHwIMQMDFdgeGdXID0k\/4xLklqaAWbTwMmvU9UR06iDv0zdWRaWA+VeVtLtMMXBRF5XViVn+zwNWyatU9XGpMuDvqTLtNUXJ79JHl1ZlVzpSAokAmcIxIhr1aqVUWRzfHW\/8cYbgmSGDh1aq59Kgtpss83E1R\/nnnsujRgxosZyXxDZyHwY+Fq0aCEOG+P\/bdq0qZQRdNeVfFiGmVQaL72NTumAgTBOCDAcNcuCrXgf63eEVIOKjV04LyNggoArY2ON5T4QzTPPPEOXXnqpWL7BZjmuTo86AwUvwFmzZonZD6KjI+8BBxwQidX8+fOpa9euNUhK7lPhwCmW+mTyLvkhyCri1WGPDEt9XiLyLgF6C3fFECadq2iyOrMsGbYJdqvmfSwmqaL13uqtjytjY+Ce1HfffSf2fMaOHSuiS8AN3b8Hhas64Mr84YcfUqNGjeiMM86gzp07a0VFZ5IyezHyGNhsyoTTRVQUDJAUzmPhEkiTWZZpnUzkdWVVcrbPzXpGPtKqNqZRq6TLtNUXJ79JHl1ZlVzplvv8nQsEBSLCXhFY99tvv62IgJh23313EVx27733Noo4kQdJ4axX69at+dxPGiOIQidCNqmcL4YPH04bbLBBDrXLtkgMGph5wp2\/qFFaskWES8saAbnysWDBAurdu3dqF8Im1S4t7z5Z2IoVK+j7778XsyWbm3jzICnZhi5dutDJJ5+cFH6sxwCBL774QkhjuVb+358dRIUlXaQykhbuZ0OQYAQLXmONNQzQY1FGIBkEJk2aVONcZFq3lidTWyIjkkqq0CCSgu40HSfwpb7JJpvwTCopI1rqAUnhi+6OO+4IPZMFkho0aJCYcZRlHwurE4sXLxbtWXPNNS1R5OyMgDkCciY1e\/ZsscLBJBWAYRhJsQt6cIdTrTubd1N1jjTKDNOpE80dgzquHYHzDC561BngTdqgK6uSs32utkz+Eqo2plHDpMu01Rcnv0keXVmVXOn3pNLobNAZRlLSkw\/u5XBdDjvM6z335D\/c66+zKx4sUVirOmIadkqjTF2dIK0nnniCxo0bF9g0EBb6wKabbhrpeKFbHgrRlVXJ2T5Pw5ZJ61S1MenyTOyjW7ZtG+LkN8mjK6uSY5LS7RE+uTCSghiHRYoJakmzgbD+\/e9\/0+WXXx7awl133VXsM5p4CuYFl2pQyateXG71IeDKB3wue1JZdgdXDJElJi6XhfBccLwIu3ak6HEFmaRc7n3lqrsrY6MWSeHALmY\/CDiLSBSrrrqq8M5q0qQJ1alTp9CWc8UQvNyn7kbeAf6bb74RGeBsExYMF4R10UUXCU+6LbbYIrIAXfJQydk+V6OQv4SqjWnUMOkybfXFyW+SR1dWJVf65T6QEpwZ4GGFKOg777yziCyBqOe4rRcx9P72t7\/ROuusk0a\/TERnGUgqESBKrEQ34gVCcW2zzTa5egqqBpUSm4mbVjAEXBkbI2dSM2fOFBce4sZWeFTh\/iiQFK6UnzZtGg0ePJh69uxZ6HNHrhiiYP3X2ero3JGFGVb79u3piCOOyJywmKSc7Vqlq7grY2MoSclYeljSg6fdK6+8IpZW5FXxiPN3\/fXX05tvvsk385au+5anQSAt9F1czhmWsryFmEmqPH3L9ZY4T1LSyw53BR100EEiLJKXpGAgzLRGjx5d42qNohnOFUPwnpS655gO8H55OctCP45yvEAUf5zDCjpArKqD7XM1CvlLqNqYRg2TLtNWX5z8Jnl0ZVVypd6TwkWHuDb+mGOOIVxCGERSEyZMoKeeekrMqLAEWMTEJBXPKqrOH0errU7T\/Cp5OFxE3UIMkkJ8ypNOOqlCWCqdts\/j4Jp1HlUb06hP0mXa6ouT3ySPrqxKrtQkBacJfHFiOe+6666jDz74oMZM6p133hHOE5hlYTnQ9Ar5NDpykM4ykFRWWFVrOXJW9fjjj4tVgahlwWuuuUY8jhumSTWoVKsNuN3ZI+DK2BjpOIH7obp3705Lly4VXlEvvfQS\/fnPf6b\/\/Oc\/9Oyzzwq3XlwDv+WWW2aPsGaJrhhCszkslgECIK2FCxcS4j1GLQvi8DACFpsQFpNUBgbkIrQQcGVsVJ6TQsTmMWPG0NSpUytXday22mp01FFHEfarELS1yMkVQ0RhmMfAlkaZtjpN85vIh8lKkoo6jyVnVvB0RVBcOBWFxRc0qVNR36s82pB0mbb64uQ3yaMrq5Ir9XKf\/wXBi4f7pH777Tdq0KBB4Q\/xyvqXgaSKOlhVY71AWh999JGIHh02y5KkFbQ0qBpUqhFTbnM+CLgyNipnUtibkvffYN8JcdQwq1p99dXFbKpFixb5IKxZqiuG0GwOixUIAZ0zWZKwpJs7SCpqplWg5nFVSo6AK2NjJEnhangZVubqq68WoZDg8ff+++8L89WvX59uvfVWQoDPoiZXDFFU\/LheZgioPAahDUuCiN5y+umnG+1nmdWEpRmBaARcGRtDSUp69+Es1LBhw4Qr7j\/+8Q+64YYbhAcUnCb69esnbulF2CTMrIqYXDFEFHZ5LBGlUaatTtP8JvK6sio5+Rw3V+Mdee+998Seblh8QTnTgvPR8ccfz5HcQ14EFe6mY4+tvjj5TfLoyqrkSr0ntWzZMuFi3qFDB3FOSkagQGcAaeG2VIRGwlXEMgqFaUfJQp5JKh7Kqs4fR6utTtP8JvK6siq5sOdy\/wqkhQPwWJUIS1galMuDksTi4J1WHhUGaZSbdJm2+uLkN8mjK6uSKzVJ+SNOfPLJJ9S1a1dx2dypp54q+uGDDz4oLqZjkkrjtWSdZUTAO9NCVAs4YSCIc9RMSxLVOeecQ82bNxewmLi9lxFHbpM9Aq58wCtj9+F8FGZUDzzwAPXv31\/MnLAHheXAIUOGCKcKzKyw7FfE5Iohiogd1yl5BFQzrU8\/\/ZTuuOMOLdLyxhxMvqassewIuDI2RjpO3HfffTRgwABxTffixYtp7733phEjRgjb4eK5yZMnixtTO3bsWFh7umKIKABVU\/o0wE+jTFudpvlN5HVlVXK2z7221LmCxCsP0tpss82oU6dOYqaV1mxL1UYX+qNtG+LkN8mjK6uSK\/VyHzoaZksPPfQQ3XnnnbTHHnuIKzlwgRz2pwYOHEjbbbediGu2xhprpNEvE9FZBpJKBAhWUggEVIOKqpIgLnjd3nPPPcrZllwWLPL+lqq9\/Dw9BFwZG5XnpNKDKBvNrhgiGzS4lLwRsCUpf\/3lbGvevHk0e\/ZsbeKCHtwTh5lXWjOuvLHm8qMRcGVs1CIpXB\/\/888\/12oxZlrwVkIMs6LezuuKIfiFqg4EkiapINKSf8PxkZdfflmLuLyzLiwX4khJmkuG1WHtYrfSlbFRGWAWzhLo7GFJXinfsGHDQlrEFUNEgZf2wBZUdhpl2uo0zW8iryurkrN9ntZLJGdcCBqNJXyVN6G3HpKssPeMs19oY9ZRM1S4muJmqy9OfpM8urIquVLvScnDvPDmg+s5yAjXx++5557CDRZOFQg0iwvimjZtSji8WMTEJBXPKqrOH0errU7T\/CbyurIqOdvncXC1zQMCw3u+aNEiI\/Lyzr7OOOMMwodqWkuHKlxNMbDVFye\/SR5dWZVcqUlKHuYFKcEFHQmEtOqqq4r7oxBsFlGf27dvz959pm8Iy1ctAqpBpQjAgLRANvgXZ7hwlstk5iXbIGdg8AqWMT55CbEIFv7\/OrjyAR+63Oc\/zItGocPOmjWrEnECLugvvPBC5ffiwP97TVwxRBGx4zolj4ALJBXWam\/UdywbTpw4URBZVDT4KAQlYbVu3ZratWsnRNOaiSVvSfc1ujI2hpKUnEkdfvjhlZkSGjVy5Eix7Lf++uuLvSqEeOGIE+l22DwGtjTKtNVpmt9EXldWJWf7PN2elIz2oDZKosK\/cLTCTQlIcWZg3pmYJK6WLVuKu+sw7mBfzJbQVHZSIRUnv0keXVmVXKmX++SeFMIhYZkPnWP+\/PnUo0cP8XurVq3o+uuvp2eeeaZCWirD5vHcla+FPLDhMrNHQDWoZF+j7EqURPbmm2\/SjBkzrEksiMy23nprOuywwyrexjwzC7evK2NjpHcf3Mu7desmDvAikjNijfXt21cs+eG6AVwjf9ZZZwniwl1TWSUJriwPh43btGkTWLwrhsgKOy4nXwSqmaTCkPcuF+L\/K1asoJdeeoneffddq+XEsPIkceFfP6nZztDy7V1mpbsyNirPSSGWGL584CCBsxPoRLhKG0t9xxxzjLhCHrOsrBKA7dWrl1gPh5eh\/3d\/PVwxRFb4cTn5IsAkZY+\/JDV4Iz766KMVIou7N6ZTIy+xQX6rrbYSP7vttlstz2ZXZm+ujI1KktIxYFYy8roQuLzDw1AmkCaS92\/ymSuGiMIwj4EtjTJtdZrmN5HXlVXJ2T7P6l2yKUfVRhvdYXlNygwiK1yNgq0JbFkg4eM76rqUpNsgiQvbKPg\/Vp4QE7VBgwaE\/TZE\/qhTp06lWLRX5yyaCpdS70lJtBYsWCCW92DUoASQcVlbFhEnpMchyMi7vAciAlEFOXAwScV73VSdP45WW52m+U3kdWVVcqrncOe+7bbbqEuXLhUHgDhY5plH1cY06pZ0mWH6vAT35Zdf0tNPP10hNq9zSBptNNXpJT6QnncGt\/HGG4ub0\/G3Ro0aUbNmzWittdaqUQTCaPXu3VuckQvbLjGtUxrykTOp5557js4991xxJiosZRlxAl9Bffr0ERHY5b06qFfUkl8ZSCoNw7POfBDg\/pgP7mmU6p+xyfNlsix82D\/\/\/PMiQoef4NJcmjRtKxzhnCQpubS2cOFCMUvB9dZ5R5WoVpJCZ4dLL\/YA8UWURUqjTFudpvlN5HVlVXKq52UgKVUb0+ifSZdpqy9O\/rA8QYT1yiuvVN553ECBhLNpr732mrg2CUuByAed+JFu+XHIz1mSkktruJ7jqKOOSqPfGeu0ISl4IOLQoIsJS66YlmfZhjTKtNVpmt9EXldWJWf73IX+qWpjGm1IukxbfXHym+TRlVXJvfrqqzRq1CjxgYvZ0oYbblgxj3zm7HIfDuRddNFFhQp7FIek8JWBAR7rr5wYAUaAEWAEfkcAH+7Dhw\/PbIUmDvbKm3mnTJlC1157LTVp0iSO\/kTzxHGcQAXklDjRyrAyRoARYAQcRwDbB1ltIcSFqgZJYfaEGz+XLl0q9OEOKZxDwMbfoYceKrxF\/ClL7744LuhxgeF8jAAjwAgwAvkjUIOk5Exl7ty52jXL0rsPlfJvPKsO82o3hAUZAUaAEWAECoeAU4d5JXomYZEKhzhXiBFgBBgBRkAbgVCSwlIflv8QCimLg7raNWZBRoARYAQYgapBoBZJIWwHXBLhtigP8R588MF0ySWXiFD5nBgBRoARYAQYgawQqEVSCKF\/wQUXiHhS++67LyE0CA6S7r777oK4EBGdEyPACDACjAAjkAUCNUjqp59+ov79+4sZFC43lN58iHiO8EiIPF7k8Bk2gMFppGfPnqL93pBLNjo5LyNggoDfcQmRADp27GiigmUZgcQQkN7U06dPFzqjrkRKrNAARYHefSAib0RxhMQ\/7bTTRFDMMr40OCTctWtXAY+8AiRN0Fk3IxCEAMKPIbwN3jHZJ0eMGFHaD0PuBcVG4O677yYERAYXRAXxTrsVWiQlv\/D85JV25Uz046UeMmSImAE2bNiwRtYob0C0bezYseI2z0svvbRW8FqTOrAsIwAE4vZFL3ryK7ZTp05MUtytrBBIoj9iDL3rrrto2LBhVK9ePav6mGYuBUlJEkXj\/dd16F6SGBZyyRRQlq9uBJLoiyqiq26EufUmCNj2R++SX6GX+4o8k\/LOkvwHi00iVDBJmXR9lg1CIKm+yPuj3L+SQCCp\/oi6hIWkS6KeKh1Oz6SkEbDBjDR58uQaMymTWH9MUqquws+jEEiqL0YtzbAFGAFdBJLqj7I8+cHftm3bzP0SnAuLFGYkbPL5ScokajqTlG73ZzkVAnH7IvTi1t4BAwZkvu6vahM\/dxeBuP1xzpw5otHSkSfowtksUIkMMKtTgSwDzEbVJ64hpLs5k5SOtVlGB4E4fXHMmDE0btw4ku6+spy89gF02skybiAQpz\/CyxnR0fv27Vvpk3n1RSdj9wV1jbiG4DNRbrxoLtWS+6JL1ip\/XV3vj0xSzZuXv5dyCzNFwPVBIVOwuLDUEXC9P5aapEwcJ1LvKVxA1SAQNChwX6wa8xeuoa73x1KTlIkLeuF6FlfIWQSCBgXui86a0\/mKu94fS01S6F18SaLz75hzDQgaFLgvOmfG0lTY9f5YepLyDg7sMVWa967QDQkbFLgvFtpspa2c6\/2xNCRV2h7GDWMEGAFGoIoRYJKqYuNz0xkBRoARKDoCTFJFtxDXjxFgBBiBKkaASaqKjc9NZwQYAUag6AgwSRXdQlw\/RoARYASqGAEmqSo2PjedEWAEGIGiI8AkVXQLcf0YAUaAEahiBJikqtj43HRGgBFgBIqOAJNU0S3E9WMEGAFGoIoRYJKqYuNz0xkBRoARKDoCTFJFtxDXr5AIrFy5kmbMmEFXX301ffjhh9ShQwe68sorae2116ZFixZR79696ZJLLhF179q1K51\/\/vmB127LwLOQGzZsGN\/IW0hrc6XyRIBJKk\/0uWxnEViyZAmdfvrptMEGG1Dnzp2pUaNG1LJlS6pbty49\/fTTdMcddwgCA2ExSTlrZq54ARBgkiqAEbgK7iEwf\/78QPLBDGv48OG01lpr0XnnnUdhcrLFPJNyz\/Zc42wRYJLKFm8urQQIIKp0v379arTkzjvvpDZt2tCyZcsEOZ1zzjnid1OSuv\/++2vplgU1bdqUJk6cSM35NukS9CJugi4CTFK6SLEcI\/A\/BD755BOaNWsWjRgxgo466ijaf\/\/9abvtthNLf2+99RZdfvnldO2119LGG29sTFJffPEFffzxxzWwxtIi9rv22Wcfuuyyy2idddZhWzACVYMAk1TVmJobmiQCYTMkzLJee+01GjRoEK2++uoVklq4cGFk8UceeWSg48Ty5cuFA8aCBQvouuuuoyZNmiTZDNbFCBQeASapwpuIK1hEBIJI6ueff6aBAwfSrrvuWvHkk3Lbb7897bDDDrWa8ssvv9Djjz9OeO737vv1119p9OjRNHnyZBo\/fjztuOOORYSC68QIpIoAk1Sq8LLysiIQRFJYBuzVqxcNHjy4sm9kuidVr149ARkcMKZMmSJID7OyY489llZZZZWywsntYgRCEWCS4s7BCMRAIIh8Zs6cKWY9cD2X+0ZxSeq9996jbt260RFHHCHOWMG1nRMjUI0IMElVo9W5zdYI+MnH73ouC4hDUp9\/\/jl1796dGjduTEOHDmVHCWtrsQKXEWCSctl6XPfcEPCTj9\/1PC5J\/fbbb8JR4vXXX6cxY8ZQixYtcmsjF8wIFAEBJqkiWIHr4BwCfpLyu57HISnsZU2YMIFGjRpF5557LrVu3boWLptvvjltuummzuHFFWYE4iLAJBUXOc5X1Qj4Scrveh6HpKSTxPTp00OxxfJfx44dqxp7bnx1IcAkVV325tYyAowAI+AUAkxSTpmLK8sIMAKMQHUhwCRVXfbm1jICjAAj4BQCTFJOmYsrywgwAoxAdSHwfzW6MMHjRRUCAAAAAElFTkSuQmCC","height":256,"width":425}}
%---
%[output:8429c053]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.0001"],["-9.8696","-0.0016"]]}}
%---
%[output:09179d2f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.0000","0.0001"],["-9.8696","0.9984"]]}}
%---
%[output:154cc966]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"1"}}
%---
%[output:6187332d]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"1.0000e-04"}}
%---
%[output:3efa50f2]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"-9.8696"}}
%---
%[output:3a7c206f]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"0.9984"}}
%---
%[output:24ae7bfd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.1490"],["36.5219"]]}}
%---
%[output:1588507c]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.0001"],["-9.8696","-0.0016"]]}}
%---
%[output:434ec02a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.0156"],["2.7166"]]}}
%---
%[output:62fb6716]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.0000","0.0001"],["-9.8696","0.9984"]]}}
%---
%[output:1c8d79cd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.1555"],["27.1661"]]}}
%---
%[output:2e5c863a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.0372"],["1.9371"]]}}
%---
%[output:0c2ff197]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAakAAAEACAYAAAAJP4l9AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQtwVtd17xdOGqQkdohsBUlgLMcRtqdx5WeRiVOnjh0nTcBt6oke00IJocwEJ3TMQwLH18NcA+KhzmAbu8SWFei9SCStPYE2DXVxg51giIuN5t6MbUiKgkEIKzxCHAuuH9xZR15if1vn\/Z3zncf+nxnG\/nTO2Wfvtfdev73WXnvvMefOnTtHuCABSAASgAQggRRKYAwglcJaQZYgAUgAEoAELAkAUmgIkAAkAAlAAqmVACCV2qpBxiABSAASgAQAKbSBoiVw4sQJmj17tpVOZ2cnVVRUFJ2mXQJbtmyhJUuW0MqVK6mxsdF6hP\/Gl\/yO8sN25RoaGqLly5fTzJkzqa6uLsrPOaZ14MABmjVrFn3rW9\/yVc4wedy9ezft3LmTWltbYymTyLK3t9dKf\/PmzdTQ0OD7W3Z17\/vlmB5kObe1tVFNTU1scosp65lKFpDKVHUhs6oE4lZcOqTKy8stpbR3717q6uoqGaRWrVpFDBE\/AwBRnEHyyGm3tLTQ3LlzY1O28g11gBGkNcdd10HyorfBhx9+uKTtIWxes\/oeIJXSmmPFtGHDBit3PFJTlaJ02HvvvZd27NhBPDrVn9FHrqoCkpH5DTfcQBdeeKE1quXLS4HId\/U86cr8+PHj1shfLA0eoUvaftNga0xXbKqi4jywVSXXtGnTqL29nRgkfImyPnTo0Ihyt1PgIov+\/n7rPVVOarkeeeQRWr16NW3btm3km1ym6dOnW+DS\/65adlKXXEdr164l\/j1p0qSR\/Op5UOtB7nH5xMrR61bqfuLEiY55UeXOBRB5cdthQMlVX19vyYsvto7F8vECmMhW5CDpcD3q31bvqd3Prc2qdd\/X12f1Db3NS3vRy8J5EDnqbfKOO+4YKSfL5K677qJvfOMbo6x1aWv6N+3qJ6UqJbPZAqRSWHUqoNTsiYtE7\/ReCkbui\/LTlaLc1zugPmJUoaCC6uKLLy5w9wmkRPFLui+99FIBWNzSKBZSnLbISeSmwpmBdvjwYQumkk8deKx4xY3pBClRmKqsVDnaKejBwUHiAYJbHvS6lrrTYaDWvVMeL7vssgIQqe1Bv8cAWbNmDS1atGgEUHr70buMU56c6t0OUjqg5BsCR6c2L7B1qkt5X2\/znLfHHnuMnnjiiYIBxmc\/+1l6\/vnnbQdVdvArlas7hWqqZFkCpEoman8fks5UWVk5YgHICFE65NatWy1lL785ZRnNi1XEo2NdsYlVIRDh99hCU0fgdnMF0hFZuYpFp45sZTTK6fEoXE+fR69B0\/CCFFsqXi4gfZSrPy+DATsAsBwmT55cAF8\/7j5JU32frREdOnpdyn2Rk1haDz30kGU1yH07C1FtWX7cfbp7z+m3U\/vR5xz19slyElnrkHGy1vXndeX\/zDPPFLR5dQBh5wZ1GpBIm+c2KfkWaEr9sjWoWsmqNW7ntuQ653dK6QL2p03y8RQglbJ6tFO8umKSDqsCRe08XCTd6lGtFv5\/tiBkNK8qFTtI6R1eXGoiOid3n5p+0DSigJQqN7EyROFw3u2CPVQ56vB1g5Q+0mc5soWpy9kJQnozZMUpedbnl5xcd5w\/N0g5uTZ1SDlZLU6WtgpmCYbQyykDKydI2aVhZ8l7gVO3yHRLy67Nq3myq39xear5Ud2fXnlPmYrJXHYAqZRVmd1IzQlSTp3LCVL8dyflqbvGVLEEBYxYUsVCSge212+7qpR37r\/\/fsvKk7kdJ4skKKR0BaX+LgZSqjvKKQjCbt5SrGL1Hb+Wk5drTdqPHpVn13ZKDSm9zYn7T3erRgUpdQ4UkIpXiQJS8co3cOpxuPv0TNhBxw1S6uhULC1V8c2ZM8d2TkpVCH7TEJei6oLUgy6cftsJW7ceVEuxWHefPhenuovCuvuCuu74eRXeEsihQkpXorprzcvd59WIo3T36S5sKYfMZzpZUuJdkPt6nnRocV2FcffZyQKQ8mohxd0HpIqTXyxvxxU44cf14bR+xc4FJO4fp8AJFVKqMlWF5haZJs95QYqfc4oY43siT\/0ZpwASkZM+76FCiNOdMWOGFVxg5w5yCnLhPPgJnBCrRleATgEGTnLkdPiSSFE7l5UaFcfprFu3jh588MFR5dIjKCUtr8AJnv\/xmj\/0GzjhBSm9Q7q1ebt8+wmc0C1KzEnFogZHEk0dpFhxLF682Ar3tVssqYeZukWkxSu6eFOPOgRdVdBBLSkpqd28C7t+\/MxJeaXB91VocH7ZQrvnnntGRVqJolIVmxuk3NYB+Q1Bl8l5tf0xAG699daRyDlWiF\/\/+tdp3rx5I25FHZIcgr5w4ULXEHQVBnbuXzuFbjc\/yd\/mPIqlK0sV1q9fT08++STJ\/JwKX33gIQB2ky9\/xy0EXbf2nBZeO80nqXOmTpDSBxAsD176IAENnAd9fpD\/pn5TrU99wbg6x6ve092a+nxtvFrCjNRTBSk\/kUmsKDnsN66V8Vmodq9RaRbKYEIeVcWth\/\/rVqaTPEQJ8mDA5DYfR3uRAQqnbRe16mcXE6yTiqNmCtNMFaR41MgWBF9OlhTfr62t9bU9TPziS+YLgFQycg\/zVSfXrdfCafVbQXacCJNHU9\/xcp362faK+yJ2nIi3BaUGUtxgli1bRs3NzRao7CAlI1N+Jsi+X\/GKsPSpA1Kll3nYL9rNe3jt3qB\/S0br7Co0ud2HrQO39+zmJf3uK4i9++KokdFppgZSrHj5uv766x3npOxGPkFGpKURKb4CCUACkAAkEJUEUgEpHs1s3LiR7rvvPmurGqfACX1E6TXC5LT4Hy5IABKABCCB0RLgpR78L81XKiDF7j2OkmJXhld0ny5MmcPSJ5UZTrz\/2J49e9Isf+QNEoAEIIHEJDBlyhRrr8Y0gypxSDlNXnKt+fENOwVSSOguV8CECRMSawRJf5ghzeteIAfIgdsi2sNwj4QczsvAj55NUo8lDim98G6WlET\/qWtWeM2J3caOAqm0V0DclQ85DEsYcoAc1L6G9pCdPpF6SHFj6u7uHjl7R1\/M6wQhNMLhLslryni+j0+S5dB9Uy\/IAe1BbftoD4BU4roQkBqugjNnztDRo0epurqaysrKEq+XpDIAOaA9qG0P7QGQSkoXjXwXkIJSglIa3Q2hnNEvpFVkRUemzt0XFd2yUgFRldcpHSglKCXAGrC20w9Z0ZGAVNyUSDh9QAqQAqQAKUAqYUWc5QqIW3SAFCAFSAFSWdaRsKTipkTC6QNSgBQgBUgBUgkr4ixXQNyiA6QAKUAKkMqyjoQlFTclEk4fkAKkAClACpBKWBFnuQLiFh0gBUgBUoBUlnUkLKm4KZFw+oAUIAVIAVKAVMKKOMsVELfoAClACpACpLKsI2FJxU2JhNMHpAApQAqQAqQSVsRZroC4RQdIAVKAFCCVZR0JSypuSiScPiAFSAFSgBQglbAiznIFxC06QAqQAqQAqSzrSFhScVMi4fQBKUAKkAKkAKmEFXGWKyBu0QFSgBQgBUhlWUfCkoqbEgmnD0gBUoAUIAVIJayIs1wBcYsOkAKkAClAKss6EpZU3JRIOH1ACpACpAApQCphRZzlCohbdIAUIAVIAVJZ1pGwpOKmRMLpA1KAFCAFSAFSCSviLFdA3KIDpAApQAqQyrKOhCUVNyUSTh+QAqQAKUAKkEpYEWe5AuIWHSAFSAFSgFSWdSQsqbgpkXD6gBQgBUgBUoBUwoo4yxUQt+gAKUAKkAKksqwjYUnFTYmE0wekAClACpAyFlInTpyg2bNnU29vr29VXF9fT08\/\/bTv58M+uHv3bmppaaHNmzdTQ0ND2GQy\/x4gBUgBUoCU0ZBasGABLV26lOrq6jwV+oEDB2jFihXU1dXl+WyxDwBSUM5QzlDOTnoEgzci1pF3t62nf2qfl+qBPNx9xdIw5e+jMwLWgDVgbaemul8coHndr9CPvlaeX0iJu29wcNCyjvxYU6XS6bCkoJyhnKGcYUk5a1wjIMXFHxoaora2Ntq2bZsljZUrV1JjY2OpWOT4HUAKkAKkAClAylkVr9p+kFZt78u3JaUXX8DAf582bRq1t7dTeXl5IsACpAApQAqQAqQAKVsJqNZVTU1NIq5AQAqQAqQAKUAKkPK0kiSir6OjgyoqKjyfj+oBQAqQAqQAKUAKkIIlFRVVY0oH0X2ANWANWNupF8xJYU4qJuwESxaQAqQAKUDKTmtw+DlH+OU6BJ0Ljui+YNAo9dOAFCAFSAFSxkIq6XVSq1atsmTf2to6qg4wJwXlDOUM5Yw5KedhsTGWVKktA\/meQGju3LmAlEslwJICrAFrwNpoSyqJvfvYxbh8+XL6xS9+YW3nAUvKmVKAFCAFSAFSdhpi2vqX6We\/OpXvOamkdkHfsmWLJfO+vj64+zxMWUAKkAKkACljIZWEq4\/BuGzZMnrggQfo8ccfB6QAKV\/NELAGrAHrwq5y7YMv0KETZ\/JtSfnSDhE\/xMESt956q+Xm8xM4MX\/+fJoyZQpVVVVZ\/0y7WDkPDAzQ+PHjE9uiKg0yhxzOQwrtgcjk9sD133\/6HfrK9163GkXuQ9BLqYB494qNGzfSfffdZylcP5CS\/M2cOZNmzJhRyuym4ltnz54l3qW+srKSxo4dm4o8JZEJyGFY6pAD5LBp0ybq+sG\/0ukvDEdHA1IRaiSei1qyZMmoFHkz23Xr1hX8XaL\/1qxZQxMmTDDWkuIgk2PHjlnlLysri7A2spUU5DBcX5AD5MCW1D\/\/7FVavnd40ApIxajL\/FhSOD7+DB09epSqq6uNhhTmpM67+9Aeht19JsvhyV1HaOE\/7TcPUqqlw3Dgq7u7O7YjOwAp7xGA6Z1RJAQ5AFJqbzG9PUj4+QVv\/Yb+5W8uze\/JvGqlMzD6+\/utqDuOvmtubvYMbvBWseGfwI4TUEpQSqP7j+nKGYMWsiL6OLKPr7L9P6KnvvOX+YcUh4XLot6JEydaJ\/UKpHBUR3jQRvEmlBJgDVgD1qoExIriv130763U88RDZkOKLRq2sjo7O3GeVBTUCZgGIAVIAVKAlEjgp788RdMffdn6ya4+hlTa5+3HnDt37lxAvWf7OM9H7dq1q8DdN3nyZJo9ezY1NTVRY2NjFJ\/xnQbcfVDOUM5Qzk4Kw8TBG7v5GFD8X77uu+Esrb\/\/m+ZAigstYFAbxsqVK0sOKDUvaR8l+KZuyAdN7Ix2ooIcMGgxedDCYOJdz3mvPr7uvf0yur3iDWppaTELUiH1aCyvwZKCUjJZKcGCcFcrpg1aVm\/vo\/btBy2hTKooo33fuXnEqEj7QD4yd18spCkiUUAKkAKk4O4zHda6i48BtfWb11mgyoqOjARSfndDdzr7qQgWOb6alQqIo+xQzlDOpitnr36Vd0uK4SSAElmIBSW\/s6IjI4EUF5oDJ3p6egqi+AReEjjhtvjWq1EFvZ+VCgharqDP570z+pUH5ADL2pTBG0fw3dPzykiAhOriU2WQFR0ZCaQERnz4IO9OrgtCQtCPHz9OK1asoK6uLr+6JfRzWamA0AX0+SKUM5SzKcrZZ5ewHstbvxDLadX2gyPBEU5wMtKSAqSCdI\/SPpu3zhhWepADYJ1XWP\/6+BDd9di+AsuJyzprag3Nv+0ya\/7J7srKQD4SS8qvu0\/WUuk7lodVPG7vZaUC4ih7XjtjMbICpACpPPULPaRcysZAuuVT4+iRpqs9u0tWdGRkkGKJ2K2TkvBGvrdw4ULL1VdXV+cpwGIfyEoFFFtOr\/ehnKGc86Scvdq73\/tZ7BfdLw7Q5p8fLXDnqXD6+7uvpNuuqvArArOi+3xLpYQPAlJQzlDOoztcFpVzHGojC3LgAIgfvHSM\/nF3v60I2Gr6zBXjqPXOyx1dennwNkVqScXRmMKmCUgBUoAUIOWkP9IIKXbhrXv217T\/2Fu21hKXRcDUfFO15dYr5sqKjowMUnauPhFgfX09NpgtpjUV8W4aO2MRxQn9KuSAQUvaBi1O80p6I2cwzfvcJJpzy4TQ7d\/uRaMgxUdS8\/EcU6dOpenTp48c1YENZiNtU6ESg3KGck6bcg7VkCN+qdT9goEk4eGywatTkRhKt11ZQX\/3eefIvCjEYRSk1POkOCiC10XV1tZaG8uyIOI8ndepsrJSAVE0Nrc0St0Z4y5P2PQhB8C6FLDmeaRDJ4aIgxxkM1e3Nivh4d++bRLdftXFoeaWwvaJrOjISNx9OqQ41Lyvr494cS8OPQzbhKJ5D8oZyrkUyjma1lq6VIrtF2wNvXn2XWp9aj+9fnJ4CyI\/F0Ppc5MrrF3IndYv+UknimeMghQLTN3ySLWetm7dap0z1d7eTuXl5VHI1lcaWakAX4Up4qFiO2MRn07Vq5ADYB0G1gyf3sO\/o+8+fzgwjPh7LX9cTU03ViUOJLvOmBUdGYklxQLQd51gaG3YsIFqampKtjZKrYisVEDcmhzKGco5jHKOu10mnb7aL954i2jo7XfpoWcPWRaRHzed5F+sIQkF578nbSH5lW1WdGRkkPIrmFI9l5UKiFsegBQgZTqkZF+7\/cd+T0\/ve8MSRxAQqeD58qcr6UufvsRKg2GUFSAZb0npc1KqQDAnFTeG3NMHpAApEyDFIDpHRKu3H7SsoSDzRKp8GDqXfryMFn2hlmovHp6eyDKI3LRDVgbykVhSbpBiQcgu6BUV\/rfsKFa1Z6UCii2n1\/uAFCCVB0gJhJ54\/jDtO\/y7oiDE8qi+8IP0+U+OpRvrauiTn\/hormHkpCOyoiOLgpTbAl5VMKU87FC+m5UK8IJMsfcBKUAqzZBSo+LY+uG96YLOC+l9RKwh\/i9bRBeMGTPKGkK\/OL\/XqhHHx7tZUsUq2bDvA1JQzmlWzmHbdbHvlVo5y3xQ1cc+ROt2HKK+40OhrSApuxqsMLOhhqo+NjawS67Ucii23uJ4Pys6sihLKg7BRZVmViogqvI6pYPOCFjHBWuxgs68\/R499J+HiM81CjsXpOZRLKGrqj5CX7mmkj5wwXlLKKr5IfQLQywpCTvv7e111bXYuy9uFDmnj84ISIWBlADov359mp599Tj9uohgBB1A\/JuDE26\/+mK6YdJFI\/NBUQHIT29DvzAEUn4aQ1LPwJKCcg6jnJNqr6X4LoNnf\/9JOnXqFJV95CLa\/trvLPcbX0FDsu3yK5BhAP1J3cep8caqEQCVonxBvgFIAVJB2ksszwJSgJRJkBLL5933ztH6n7xOrx37fWTw4YTUeaCbPznOgpD691g6cYyJAlKGQor37FuyZElB01q5cqW10WypL0AKkMoDpAQ+v3zjLfrJ\/hP08uu\/iwU+bP3UT7yQZt5cQ2M\/eEFBdy2lG65UegKQMhBSDKienp6Cc6NkzqqpqankoAKkAKm0Qkoi3jh\/vObnlaNvFrUA1cv1dtnF5fRXU6qp5mNjiZXzH5w9SdXV1VRWVlYqJqTuO4CUYZDS9+1TWyQW8ybbP9EZSwdrsXpO\/P5t6tx1hPp+MzzfE0XEm7Qidd6H\/\/+vG2os+Ph1vaE9lK49JNvzvb+elYF8JCHogJR3g0jqCSil8EpJXWj6znvnqPvnR2n3wd9GDh4VMOx2a7j8Y\/TZuo9bi1DVe1G43dAewreHpPpwXN81ClIsRLj74mpKxaULpTRaKfGu13y9\/e579A\/PHaZXB4aDDKK0eHTwMGC++IeXWPM+fq2e4mre\/m20B0BKWoZxkBJQIXAiDtUSPk1TlJJYPb8deoe+v3eA9r0fYBA3eK6o\/DDdff34kQqKwtoJX9veb5rSHrwkATkYOieVRICEU2PMyijBqzMVez+LnVENLODyHzt9ln6y\/2QkW+rYyVOd5+H7fBTDlz99CY0ZM8YKaJD7aQeQn7aSxfbgp1xBn4EcDIMUNxB9s9mkNy0EpNLn1hBr5+Rbb1PXrn761eCw3y1qa8fO1XZl5Vi6ofJdqqystKLa8gCcoIqZn4dyTl+\/CFOPUbyTFR0ZSeCELjB1vRRO5o2iOYVPI06lpAYWvPDfp+j5Aycj2z7Hy+JhyPzZpy+hayb4m+OJUw7ha6f0b0IOgJS0OqMhpXY9PkuKhdHZ2Uk4Tyq9SkmAI\/89eHyIdv\/3KcvdVQprh79x7aUX0ucmV1DdJz48SlDFWj5QzlDOaqNCezDQ3aeDacOGDdaf\/FhSQ0ND1NbWRtu2bbPecTt\/SncrOqWflVFC3Nja33+KBgcHLTfXv716mvgI7WJOLvXKr76OZ\/ofVdLV1cOHyvFVLGy8vu90H0oJkAKkCntHVnRkZO4+fUukIHNSbG3x1draSl67VPB3+vr6rGfdrqxUQFilO7JX27lz9L92H6Wf9w2v34lio1A1Typ0Lr+knK6Z8FG6uqoQOkmBJ4jsAClACpAyGFJeYAmiTPhZFVr6u3yvtrbWc5ulPEBKLJ72Hx+MFEAqeDiE+vNXVdDHP\/wHBZZOFsATpF0BUoAUIGUwpIIoC69n3XavELdgc3MzNTQ05MaSYhj980vH6NnXToS2hHQ329\/cPIHGX\/Qh7NX2fisBpAApQAqQ8uKP5322kngua9q0adTe3k7l5eUF79gdsui0y7pYUvPnz6cpU6ZQVVWV9S8N1+6+N6ljx+u+gVRz0QetbFdf+EG67coK+svrx9OhE0M0qWJYPm5WDyvngYEBGj9+\/Ch5pkEWpcoD5HAeUmgPw6H4psqBy83\/jhw5QosWLaIgUzOl6q\/qdyKbk4oy8wyr\/v7+UaA6cOAAzZo1i9auXWtZUvpvNQ96gMXMmTNpxowZUWYzUFpP\/tdvaf0LJ13fYRgxiL505UdoyqXlJHAK9CHt4bNnz44ETowdO7wRqYkX5DBc65AD5LBp0ybauHHjiBoApEJoRIbP4sWLafXq1VRXV+eagtP8lUBqzZo1NGHChEQsKbZ2vvrd\/0v9p98ZVQa2fngz0dV31dIHLhgTW9Qbu0iPHTtmld\/koxkgh+EmCDlADmJJ7dmzh9atW2eGJcVuuAULFtDSpUtHQYWBs2LFCuro6PC9TirI8R5OgRRJBU7w\/NK\/\/p9Buu+Hv7QF07JpV9B1l14UG5T0j2Iu5ryb6+jRozhH6cwZghyw8wb3iqR0ZFC7JRJ3nxuk\/ABHtYYkOILXP+lh5npa\/HvhwoXU1dU1Co6lrgCG0+kz79CfrH2xoA7YYvofX\/4kffW685uQBq2kYp4HpAAptf2gPaA9SHsotY4Mq8eKgpQ+7+OUCbfFueKCUBfzqoET\/I3u7u6R+Sm\/ewSWsgJ++stTdE\/PKyO7M3CZGE5bv3ldySwmJ9lDKUEpAVKjewf6BSypsNCM7L1SQIqtp3fee49uXLFnJN8Mp0earqZbPjUusrIUkxA6IyAFSAFSdjqkFDqyGN0l7xZlSUWRgbjSiLsCGFCrth+k7hcHRorQOeMP6S+u\/URcRQqVLiAFSAFSgJSxkJJ1S3PmzKHHH3+cent7bRVpfX19rjaYZUBNf\/TlEfde2qwnKCUoJbh\/3cd0GLwZ5u4LNcSP+aW4LCk7QKVh7glKCUrJT5eCcoZlLe0kLh3ppx0GeQbuvgDSYkDN635lZKcItqD2fefmACmU\/lEoJSglWNawrI1190nB7bYrUoWSF3ffvT94jb73Qr9VtCwAivMJSAFSgBQgZTyknOwDXvO0fPly4i2JvHaOiNrGiNqU5QAJtqIEUGl28UEpQSnB\/Qv3r5dOjVpHen0v7P3Y3X36OqewGQ36XpQVwG6+ax98IXOAgiV1vtXAooRFicFboRaNUkcG1c9Bno8dUmG2RQpSAKdno6wABpQcMshzUFk6awnKGcoZyhmWNdx9LlRx2tE8ChC5pREVpP73z4\/St3petT618I5aWvqly+POeqTpA1KAFCAFSBkPKbfACd6Dz25vvUg1sU1iUUBKd\/OlPZLPTqaAFCAFSAFSxkMqbuCEST8KSP35Y\/vouQPDZ0Ctb76amm9Kx6GJQeQBSAFSgBQgBUi9f04NbxI7depUamxstM6t4d92u5kHUbJhny0WUrxpLO8qwVdWws1hSTm3FsAasAasC\/tHsToyrG4O+l5kgRN2hw8mCapiK2Da+pdHFu1mLVgCnREjZydFAFgD1tI2itWRQWET9vlIIBX1oYdhC6O+V0wFqFbUZ64YR9vmXRdFlhJJA0oJSgmDFgxajHf3SeAEH1LY0NBQIA8\/hx7Gob2LgdRfPLaPdr4\/F5VlK4rlCkgBUoAUIGU8pFgAW7ZsoZ6enoLdzgVeTU1N1jxVKa+wkFIj+rJuRQFS51scYA1YA9aGz0lx8e1O6l25cmXJAaXmZfPmzaOsOzdYrnv2EC37l19Zj2TdigKkACm9rQPWgLW0ibAD+VIaG\/ytSOakSp1pP98LUwF5WBcFpWTfOqCcoZxhSRlsSUkUX3NzcyCrxQ9swj4TBlJqwETrnbXUeme2dpewkxWUM5QzlPPonoF+Ydihh27RfWEhU+x7YSClhp3zLue3fGpcsdlI\/H10RkAKkAKk7BRRGB2ZhEKLzN3HgRN9fX3EEX5puMJUQMW9\/2llPQ8BE1IHgBQgBUgBUsZDKg+HHqrzUXkImACkCrslYA1YA9YGz0mlwXLS8xDUkhJXX5a3QMKclHNLBKQAKUAKkEoVq4JCKo+uPq4QKGcoZyhnuPuMdfeJm2\/OnDn0+OOPU29vry2o6uvrCxb5loJmQSClRvXlydUHSJ1vaYA1YA1Yw5IqBXt8fyMIpPLq6gOkACm9wwDWgLW0iSA60rfijeHByKL7ZK1U1o7qyNs2SFBK9r0EyhnKGZaU4ZZUVo\/qUF19WT3Y0G3wAuUM5QzljDkpY+ekpOBZPqrjiZ8epsVPHbCKkpcFvFBKUEpOAxcMWjBoMdLdl+WjOvI8H4U5KcxJwf0L96\/TgMW4OamsHtWR19BzaZgYOWPkDMsalrXx7j7dfFQFkuajOrpfHKB53a9Y2c1b6DkgVdgtAWvAGrA2PHDCbfI+iXt+TFl1Q1kkBHYYAAARdElEQVRAKolaKt03ASlACpACpEqncXx8KQik8rYVEjoj3DtOXQSwBqx1z1fQg2F9qN9IH4lsnVSkuYogMT+Qyvt8FIsRSglKCYMWDFowJxUBVKJOwgtSed4KCUoJSgmWlLtGweDNsEMPowZMFOl5QWrT7n76u++\/Zn0qj+ujRIbojLCkMGjBoAWWFBEdOHCAZs2aRf39\/aPkkcYNZvO+PgqQKmyGgDVgDVgX9gmvgXwUxkIUaUQyJyX79tXU1MR+Mq8IlgvP3+vq6qK6urpRsnCrgLzv14fOiJEz3H1w93kBwihIuW2L5CWoIPfZWlu8eDGtXr3aApPdAmJJz60C8r5fHyAFSAFSgJSXbjUKUmJJNTc3U0NDg5dsIruvQ0tN2K0C1EW8eZ6PYnnAzQU3FwYtGLRgTorIsmp27dpF7e3tVF5eHhmI3BJy+6YbpExYxIs5KcxJ2fUdDFowaPHjbSqJAvf5kUjmpGSD2VKdzKsGaTgtRPMDqTwv4gWkAClAylkLAtYIQffJyOIeE1itXbt2lJtRIDV\/\/nyaMmUKVVVVWf\/6T79DN7a\/aH14ymUfpafnXlNcJlL+NnfGgYEBGj9+fMks3DSKBHI4b0GgPQy7wU2VA5eb\/x05coQWLVpE2HEiZo1ld9gif1KNAuTfM2fOpBkzZtDeI2fob58asHL13a9W0Q0TymLOYbLJnz17lgYHB6myspLGjh2bbGYS\/DrkMCx8yAFy2LRpE23cuHGkNxoFKZ4jWrJkiVV4Ljhf3d3dsc1T6UfWqzpQILVmzRqaMGHCiCXVseN16thx2Hr0qb+9hhpqP5qg6oz\/0yyjY8eOWeUvK8s3kN2kCTkMSwdygBzEktqzZw+tW7fOHEuKLRpeyPvAAw\/QsmXLSCL9nCydMOpZj+ZjEC1cuNB2rZTTnBQfzcHRfXzldedzVbbwvQ9LA3KAHNAvCrWuUSHo6jqpiRMnUltb2wikGCwrVqygjo4OqqioCMOmgnd0N17QwAlTdpoQoUE5QzlDOY9WO+gXhgVOuEGKocLWVGdnZySQ8ks5p1GCCTufQylBKTn1EyhnDFqkbRhlSXGhZc2S6u6bPHkyzZ49m5qamqixsdEvXyJ5zgtSrXfWUuudl0fyrTQnAqUEpYRBCwYtdjrKOEixEHRXHP8tTcfHm7QdEtx9hd0SsAasAWuD56TSaEnYjRJUSOV9OyRACpCy65eANWBtrLsvbaCyg5Qa2Xfi7\/80bVmOJT9QSlBKsCDg7oO7730JqOukRChJLRSzg5RpkX1cB4AUIAVIAVKA1PuBEz09PQVRfLKnX1oCJ0yL7AOkzndNwBqwBqwNnpMSGLW2ttruoZeWEHSBlCmRfYAUIKWPoAFrwNrIOSk3SEW9mNfvxI3u7lODJgApv1LMz3NQzlDOsKQMtqS46HZbFKXJ3bdq+0Fatb3PqiVTIvtgScGSgiVlP9DCoMXAHSd40a7TeVJqM6mvr6enn3469iG6bkmpkDJhzz4RMDojLAhYEAicQOBE7MgJ\/gEdUiZG9sGSgiUFSwqWlJP2NHLHieAoie8NvQKuffAFOnTiDJlwGi9Gzhg5O\/UsWNawrKVtGAkpu3VSadgWqWbytcSQ4uszV4yjbfOui4+OKUsZSglKCYMWDFrg7kv5OikVUo80XUUtf1ydMpTElx1ACpACpAAp4yGV9nVS71xyFU1\/9GWrnkwKP8ecFOakMCeFOSnMSRFR2iF18AO1xPv28WVS+DkgBUgBUoAUIPW+BHg+Kq3bIu387fiRNVImhZ8DUoAUIAVIAVKKBNIaOLF871j62a9OWTk1ZfdzqRbMSWFOCnNSmJMyfk7KidRJ\/l0NrxRImRZ+DksKlhQsKVhSsKSSJJHLt1VI\/dn3h6wnTQs\/B6QAKUAKkAKkUg6pVeufpLnPnLNy2XxTFa1vvjqlOY4nW3D3wd0Hdx\/cfXD3xaNfi0pVLKl5\/\/NRYncfXwwoBpVJFyAFSAFSgBQglUKtD0hBOUM5Qzk7qSYM3gzbBT2FjLKODmlpaSHVkjJtjRTmpDAnhTkpzElhTiqNhHr\/fCuGVP3MFcTrpPgybY0UIAVIAVKAFCCVcki9ectieueSK61cmrZGCpACpAApQAqQygikTFwjBUgBUoAUIAVIpRxSp7+wit778CVGrpECpAApQAqQAqRSDKmmb3ybGFJ8mbiQF5ACpAApQAqQygiktsz5I7rj6otTmtv4soVQ22HZQg6Qg9rL0B4Qgh6f1vWZMoegf+3by4gDJ\/gy7RwpERM6I5QzlPNopYF+AUj5REl8jzGk7m5bT29d\/3XrIyaukYIFAXcf3H1w98HdFx9nikpZh5SJa6QAKUAKkAKkAKmiUBLfywypP1\/7H\/T\/Jn3G+gggVU1lZWXxCTzlKcO9A7cn3J6FnVQ9KaKhoSG1PXjMuXPnhrcIz9nFFTD90X1GL+SFJQVLCpYULClYUimFmwopUxfyAlKAFCAFSAFSKYbUV773utELeQEpQAqQAqQAqRRDyuQTeaVaMBeDuRjMxYxWUugXCEEPhK4TJ07Q7Nmzqbe313pv2rRp1N7eTuXl5aPSkck+uVFTU0NdXV1UV1dX8Cw\/J5DquHsyzZo6IVCe8vIwOiMgBUgBUnb6DIETPrX80NAQtbW10dSpU6mxsZHkN8OntbV1VCpbtmyhvr4+23vqwyqkTF3IC3cf3H1w98HdB3efTxgFeYxBtGvXLltratWqVVRbW2sBze1SIWXisfFw9xW2DliUsChhURb2CVhSQaikPesEKbGympubySuuX4WUqbtNwJKCJQVLCpYULKkiYGT3qsxPNTU1jbKW9Lkrfn\/lypW2VhVDSqL7AKmjVF2NxbxHj0IOsChhUYrehSUVAl5iKfGrdoETBw4coFmzZtHatWstS0r\/7TQn9aOvlVNVVZX1z7SLldLAwACNHz\/eNhDFFHlADueVM9rD8K74psqBy83\/jhw5QosWLaLNmzd7eqaS1BOp2XHCC1BOQuI5Kr70IAtx913w1m\/oon9vpZkzZ9KMGTOSlHUi3z579iwNDg5SZWUljR07NpE8pOGjkMNwLUAOkMOmTZto48aNI90SkPKhobwi+tyScAqkkB0naibX0z\/cPsZYS4ojIblBMqQ54MTUC3IYrnnIAXIQS2rPnj20bt06WFJ+lCKDpr+\/33FtlOpD5Wc7OzupoqKCGEQLFy50XCfFc1I3X\/Mp2jbvOj\/ZyOUzWfE7xy18yGFYwpAD5KDq05aWFkDKS\/nYBUPwO\/X19RaM9u\/fT93d3SMA0xfzOpmq4u6rLXuTHp1e6ZWN3N4Xv\/P8+fNpypQpuS2nV8Egh2EJQQ6Qg\/QVzEl5aY2Y7x8+fJimP\/oyvbHr+\/ShQz+L+WtIHhKABCCB7EmAB65r1qyhiRMnpjbzqQmciENCDCr+hwsSgAQgAUhgtAQYTmkGFOc415BCo4QEIAFIABLItgQAqWzXH3IPCUACkECuJQBI5bp6UThIABKABLItAUAq2\/WH3EMCkAAkkGsJAFK5rl4UDhKABCCBbEsgl5DiXdSXLFli1YzTBrTZrrbRuedFzhs2bLBuuG1zoj7ndGBklmXjVw5SRv08syyXXc27Xzno6xTTvkVO0PrxKwfZB5Q3Fchjv3CTm9\/jj4LKPqrncwcpbmyLFy+m1atXWzKS\/9dP7o1KgGlIhxcuy04cvPhZ3ZVDzZ9+BAr\/7unpGdnBIw1lKSYPfuWgy4QHNHkazPiVg74dmdp38tBf\/MpBQM37f\/LG1XnrF16A4sFtmtt\/7iClK+K0jxKKUcryrrrJbpAzt\/KmlILKgZXTggUL6NSpU2R3NEwUdZNEGn7lwPW\/YsUK6ujosLYZy9sVRA7qYDZv\/cKuXgXM48aNs25\/8Ytf9DxINqn2kTtI6buiO+2SnpTAo\/6u7q4K4r7KU2cMIwduGzfddBP98Ic\/pKlTp6a2kwZpM0Hk4HYCdpBvpvHZIHKws6ScTgZPY1nD5InLfPLkScu12dbWlur2n0tIqcfLc0fknZ\/1ozzCVGwa37GznPxaj3439k1jufU8BZUDA5p3h7\/33ntp2bJlqe6kQeQfRA7SNzh9P\/OZQfKR9LNB5MB5lee3bdtGc+fOza2+cOo3aR6kAVJJ96Yivx+0M8rnWEE9\/PDDtjvIF5mlRF4PIgd+dvny5dbxJbwlTNpHkkEEGkQOEmAkwRJupwoEyUMang0iB\/3wVHUuK49uULV+gnhekqrXXEKKhSmWE9x9o5tW3gCljoRlROjW+VgJ7dy502ojWeikQZRDEDeX7u7LkywgB3+tJgt1njtI6e49v64vf1WazqfUMnoFTuQ5csmvHNSwZLVG8+Lm8SsHhrV6DI5X20ln63fOlV855BnWXnUGSHlJKIb7CEF3DkHPkzvHrun4DTnOmrsjaDfxKwc9YCBvbi6\/crBz9zkdphq0LtL+PCCVUA1hMe9ma70HX6pl6WRB5GkBp9PiTacAmix00jDdyK8c1MW8eVzE6lcO6mGqeZSDUxvKQvvPnbsvTIfGO5AAJAAJQALplAAglc56Qa4gAUgAEoAEcOgh2gAkAAlAApBAmiUASyrNtYO8QQKQACRguAQAKcMbAIoPCUACkECaJQBIpbl2kDdIABKABAyXACBleANIW\/H1BaZu+Yt78akanj1t2jRqb2+n8vJyT5Hp6488XyjxAxKWXV9fH+sxLep+eEHkV2Jx4HMplwAglfIKMi17aYJUkLyo9ZQFSHF+S7Xpcp53WzetfyZRXkAqCakb\/k39NFjZjkhdUCmjfLZceANY3p1aLj6gbfr06QV\/l0Pb1NE7P+81gndaxKkuCOd07BY8O5VD\/s4LqmV3cd1qUb\/L6av3+duDg4O0Y8cO6u3ttb7N91U53H\/\/\/bR161brcE8+oDBIufX9LIPshm4HYC8Ied03vDug+B4SAKTQREoqAa+NP1XrhTPGipl3AJBRv7o5ruxg3tzcbO2wYbd63m2DYX2bKLvf6t52qqDcynHHHXfQ7NmzadKkSZaLUC+H\/h0ValxOuw2A1WNVJL29e\/dau9jb7eTuVm47SKmnE+vbBHlZiV4Q8rpf0gaIj2VOAoBU5qos2xn22obFy8Wm7s2oQ8ruXTl9d+nSpZbFIZdTPlQF7pYXt33uwlgb6nd1pW5XBhV0x48fL9golsvoVG6+Zwcp\/ZA\/J8iFKRsgle0+m3TuAamka8DA76uuLt0d5wQGdQ822VtNh5TuohPR2u3F5jRvpO7x5wYpN8XrV5GLxdLf329lVdyeetp2x7yrsH7ppZeILSH9ctqDzsndp85ROZXPb9nUvABSBnbyCIsMSEUoTCQVTAKqklbnpVQXm8BJYHb48GFavHixNRdjBym\/x34nCSkuw6xZs4jhJHNdbpaUH0j5LbeTJaWfXg1IBWvLeDo+CQBS8ckWKfuUgH6kgkCKXXILFiwg1VXn5e5jZd\/Z2UleJ6om6e7jgAcdCsW6+\/yWG+4+n40Sj6VGAoBUaqrCjIx4BTeoLjZ+lgMQ2A3FgRFi\/XDkmxowoAdOqIEWbnNHUQZOqMp\/zpw5Bfnme6plwpBSLR9xUzq5+yRttrzUQAw9cMJvuZ0CJ8SqUwcC6jwe50PqT74ldSJBInbryODuM6Nvx1VKQCouySJdRwnoczHqvJQOIg4KaGlpsdJixbh27Vpr4r+pqYkaGxst5c\/zMaLg9bBwrwWrbucIeQVx6N+Scuhw1SHFv9Vwcs57bW0t9fT0WFbgM888UwAxFQ4Sis8h6M899xx1dHRYVmOQcttB6sc\/\/rEl4507d1r\/VUPu7ebIxF3J8mUob9++3QKoV9n9LIZG14EEVAkAUmgPkEAGJWA3T+W3GH6i+\/ym5ec5WFJ+pIRnnCQASKFtQAIpl4BdkIfbOiiv4gBSXhLC\/TRJAJBKU20gL5CAgwT0HSrEvRlGYPrefXbuxTDp6u9g774opIg0\/j8Qz\/p2059PjQAAAABJRU5ErkJggg==","height":256,"width":425}}
%---
%[output:7fb89984]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"13.2000"}}
%---
%[output:684c5367]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"0.0075"}}
%---
%[output:927c08f3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"0.0075"}}
%---
