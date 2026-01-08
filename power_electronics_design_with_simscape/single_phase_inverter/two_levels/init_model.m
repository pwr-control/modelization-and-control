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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAQwAAAChCAYAAAAyeYEXAAAAAXNSR0IArs4c6QAAGI9JREFUeF7tXWuMVVWWXvTgjAUiFnbRWhRUqQ0qJJow9kjoGatmCMEnGrpV0KRLBHSYIGaEhgJ+VKG8JDKtqIMISJP4QHFQQUMUQTAtwbQSMCkj0DBUCdhtDy8ZupxIZPLtct3edercuufce573fichde+5++zHt\/b5WGvttdfudu7cuXPCiwgQASLgAYFuJAwPKLEIESACBgESBicCESACnhEgYXiGigWJABEgYXAOEAEi4BkBEoZnqFiQCBABEgbnABEgAp4RIGF4hooFiQARIGFwDhABIuAZARKGZ6hYkAgQARIG5wARIAKeESBheIaKBYkAESBhcA4QASLgGQEShmeo0llw\/\/79Mn78eDl69GhmALfddpssWrRIysrKcg6qra1NGhoaZNy4cTJs2LCc5Xfu3Cn33HNPh3IPPvigzJw5U\/zWlbMxFogcARJG5JBH2yAIY8aMGbJ48WIZOHCg75fW70sOwnj88cdl1apV0qdPn0x7lZWVhjR4pRsBEka65Zez97kIw9YIVBNApXjply9fLrW1taYN\/AYN49VXX5VZs2Zl7jlJwEkYKIg+LFiwQObNm2eIC9rKtddeazSXjRs3mrpU68FnvY9733zzjcyePVt27dolO3bskNbWVhk7dqxUV1d30GRefvllGTRokEybNk1uuOEGeeyxxwQk9cQTT5ix7NmzRxYuXCh33313TsxYIDsCJIwinx1uJom+ODaZVFVVmRd1+PDh5mVULeHYsWPGpMGLh+uVV14x5oy+2E5TxY0wjh8\/bl7kRx55RFauXGkIo3\/\/\/qaOfv36if5uEwPawEs+ffp0Wb16tSGMtWvXZjQXtKMmEkjs0KFDMmnSJJkwYYK5DyLDGFAO2s7mzZsN4Xg1xYp8WuQ9PBJG3tCl48FsGoYSgxIA\/Bn64unInH6HlpaWjHahZWytBPe8EgZeajV3oGVAG1i2bJkhFPQNmkA2IlHfi1M7UsJAv1UbApHgO8raY02H9JLXSxJG8mQSaI+chIHKlRhgbvglDH0Bs3XSq0mC5+EctU0J1UByEYZqN\/gLjWHDhg0dNAwSRqBTqENlJIzwsE1EzV1pGEOHDs04RL2aJGoi2OVtv0BXTs+pU6dmVlxs88ZpeqjpkO2+bQ6pLwQaCjWM8KccCSN8jGNtIdeyahhOTy\/LqnBQwt8AUkD506dPd3KGujk91QehzlcQBerZvXu3Ib8pU6YYE4QmSTjTjoQRDq6sNQAE3MybAKplFQUgECth2Et0OgYufRUgzSJ41Dkn4OPwEjBWBENPxRBiIQxVg93IQScMJ0oq5g87WWIIRE4YWCrDMlp9fX2XUK9ZsyZnmRKTFYdLBGJHIHLCiH3E7AARIAJ5IxALYdiee5gluBBuDI83ovqw54EXESACyUMgcsKwNzPp0hr2BmAtH\/4Lhu8mb5KwR0RAEYicMODDmDt3rjQ2NprdjIj3R8QhPOHO3ygmIkAEkoUACSNZ8mBviECiESBhJFo87BwRSBYCsRCGhgS7QYG9AZp8JVlQsTdEgAhEThiEnAgQgfQiEDlhwLFJDSO9E4Y9L20EIicMG27Ny6Bp3vAdF9Oolfak5OiTi0BshOG2hMpl1eROFPaMCACB2AhDA7g0uxI6g5gMpMNn3kVOTiKQTARiIwzA4fRncIUkmZOEvSICikCshEExEAEikC4EIicMbm9P1wRhb4mAjUDkhIHGmUCHk5AIpBOBWAhDoWKKvnROGva6dBGIlTBKF3aOnAikE4GSIAw7lb4zj+jll1+eTsmx14lC4Oabb5YePXrIpk2b5MyZM5H37eDBg5G0WfSEYR+3B0RxKPCSJUtMLg5cIIyowI5EomwkFgRwcPWpU6fModW9e\/eOvA9RzeOiJwz7bAucyYkDh+0DhJf9Sy\/55S9+GbmA0WD3ihpf7Zbf1eirPAtHhwAJIwKs7cAtHMK7detWkyk8yJye9iG8GJKeUK77Vd64sUyuHzYsgtF2buLs14fybvfsn\/N7Vkmqe98aOa+iJkNa3ftWS6+6+\/LuT6k\/SMIIeQbYoeE44Bdp+nAFfcJ2LsLo88gHIY80OdU\/8M2aTGcuPftHqTz7J\/P97\/9vd6dOHu1+iXz1N5fI0e4\/ka9++Lyx56jkDCaBPfl1xU7B8Y49e\/aMvHcjRoyIxLSOzSSxN5qtWLHCEAaSAtv5PoNAPZdJEpXtF8RYoqzjxGtzTXNtn2+Tb5u3dWgaWgo0lLLBdULNpB0aahghz043DWP79u2Bbz6j0zM4QcIMamveJme\/bulEJGrqqFlTav4WEkZw8yxrTVFtPrOXVZ1HMFLDKFzQIJLTH7SbO7ZGUkokQsIofB6logYSRjhi6opEYM5UNhWX74iEEc486rSl3dlM1FvcSRghCTpLteobObGuKVMCmkjZkDqzYpNWU4aEEcE8SkKKPhJGBILO0QRIxOlcBXnAH5IWpyoJI+R5FEaKPttXge5rGHiu0HBGeoYsbB\/V5zJlsDJTNqRWzh9S56PW8IuSMELGOIwUfU6NBUPgKknIgoygeiWRbFpIEswYEkYEEyHoVRLkBK2pqemQdZxxGBEIMqYmnKaM7QuJWgshYcQ0CfJtNhv57Nu3LxM9irqdoeH0YeSLeHKegwaCMPu25u1ZfSFhEwgJI+T5kO1Ao6BWSWCe7NixQ8aMGSPr1683mcizEcaWLVtCHi2rjwOBc5ufam\/24MciB3a2fy6vErnuF+Zjt5EPB9atjRs3mm3tDA0PDNLcFeElr66ulmEeNoPt379fxo8fbyJDISTn0QRqikyePFmWLVtmzmt1261KDSO3XIqphJsZo2HuhWgh1DBimCWFHGTkfBb+DFyTJk2SadOmyezZs8135sOIQbAJbdLNmWrvk\/HjTCVhxCBk20GpCW78dMNePrVNG4aG+0GxtMs6A8s0HsRYM13kIyFhhDxvsvkwnCn0Qu4GM26FDXCK63fTQBD\/0auuvlPuEBJGigXtp+v0YfhBq7TLQvswJLLttyaMfcB\/\/ncGEBJGyHMjjEjPfLpMwsgHNT4D8sB+GCUOEkZIc0IjPLEM5Xa5rXiE1BVTLQkjTHSLu25oG18\/O97EgGy6toFJgMMUdyErItovZyi47RexiYd7ScKUJOs+2vTP8mVrq2y\/6kFmDQ96OihRTJ06VaZPny579uzp0ITXwC0sm0INRFr3mTNnmjo0NHz06NGZ7OBI+8dl1aClyPqcCBy8s5u8\/rPHSRhJnBoa4IWUfrhAGKpd4DMCv1T7QK5QEAkDt5IoyeLpU+u\/XSY7L\/gHGTXneZ5LEpZY3c5W9aphqEZhE4ZqEjimgKHhYUmN9boh8L\/\/cbt8\/d158pMpa5g1PIwp4tQI8mlDozlVw8iXMJgPIx\/0+YyNwKcNI4zz86rf7KKGEcbU8Or0tFdVKisrZfXq1ZmDjpyEMWHCBGOe0CQJQ2KssysEuKwawfyA2YBLTyHz26RNGHR6+kWP5YNEgIQRJJoudUHDuP322+XIkSMdfu3Xr5+89dZbmcOSu+qGkzDsZVV79STXXpKQh8rqSwCBuE9vB8RRmNaxnHymL7DbvhF1hDrPDymBOcchEoHEIxA5YUALQJQnAqvgc3DGYQAx+CruuOMOE6fBiwgQgeQgEDlh2EPPdswAkujgUOannvohY1Jy8GJPiEBJIxAbYRw4cEDuvPNOOXnyZCcB9O3bV66++mqzIsKLCBCB5CAQG2FguXTcuHHSrVs3eeONNwwiiKMAkUycONFoF5s3b04OUuwJESACEhthwJcxZ84cOXz4sDQ3NxtRDBkyRC655BL5\/vvv5dixY\/Lmm29SRESACCQIgdgIQwOy4ODUzWMazj1jxgx58cUXM\/cThBe7QgRKGoHYCAOoNzU1yZo1a4xZguvcuXMyePBgGTBggNTV1eUd0FXSEuXgiUCICMRGGDBJHnroIbPLdO\/evWYJ9cMPP5Thw4cLtr7zIgJEIHkIxEYYOE\/krrvukvvvv9\/EYkC7uPXWW+XRRx+V1157zcRi8CICRCBZCMRKGPX19fLcc8\/JihUr5P333zeOzosuukhef\/11ueKKK5KFFHtDBIhAfKskcHpiv8d1110n+FxRUSFPPvmk+f7ss8+aU8qiuB5\/91CnZmaOqomiabZBBFKHQGwaBpAK+vT2fND\/8b+u6\/Kx73v8OJ9qXZ\/50V\/+J3P\/R385Zj7rPfv737Z+FFibrCgaBOLefBbFxjMgGSthRCPKrluJKmv4K7\/\/o7Qe\/zbTmdbjbebzlz\/caz3xbYffnb0e0Od8GVB+vvTH3z7t2hc1oSTMoPY+cHt7SLJAJCeS9MIMcbtgimzYsMGTD6OUsobDdALJKMH87kDHkHollJ\/\/tNzA+vMrLpJ\/\/OlFIUmR1ToRIGGENCdsMwQRnciJoZvM\/CTUYdbwjgKyCcUmExAJyANaCUkkpElNDSM8YLXmbKHhVVVVMn\/+\/C4T6DBruDf5qBn00R9OiJNExv3sUpo03mD0VIoahieY8i+koeEI1NIUfQ8\/\/LAJ3nrnnXc8xWEElQR4y5Yt+Q8khU8+\/\/FJ+eTIt\/LpkXafSuWF3eXSXt3lun7nywPX04zJR6TI8XLmzBmT56Vnz575VFHQMyNGjCjejFuKzCeffCL33nuvfPfdd+YWYjDefvttT2SB8kERRlQe5oJmREgPwxELTcTg+e5fDxeGKQMthP4Qb8BTw\/CGk+9Stg\/Dzxkkbg0xa7hv+HM+AALBv48OnBTblLGdqlyd6QwjCSPn1MqvAFZJ7rvvvk7Jf7U2JRGoeIgE7erymjUcqQB3795tqrryyisF+UL79Oljvke1rJofWsl5SgPcnCSCHlIT4bJqqDM1qCTAXrKGQ6PBfpXPPvvMjOmaa66RF154gYQRgITdSESrdS7zFrtWQg0jgAmVqwq3oxLdMonnqqer30FOPFu1EAT9Pfu7P5w05gwujRtxxozgNxCK+ftDMFr7vbLUrtyQMPzNk8SWBmEgofCiRYtMHxsaGswWel2ZoUkSrehAKF9aUa12MFquaFe7p0o4GY2mvJ2A7AtRsc5Lo2Sd9wvVgEgY0c6j0FrLRRgH72xP3uP7Kq\/q\/Egfx73yfn8tY5XvNvJh383xgc4IYHlYr6Onz3YqcPSbzvdQ6CuXspl6sjzjBf9fV+zksqoXoJJcJmyT5MRrczPDP\/vnjjtfv7O+46BevZzlnPh1r2jfLdu9b42cV1Ej+r38rsYkQ13SfaOGUSTih9NTT3XHkBYsWCBLlixJvNMTRARiUdL5tnlbJ4mASEAqZYPrzG8klPgmLQkjAuztmIyVK1fK1q1bzVLqwIEDPbXudfMZVkXmzZtn6pw0aZLMmjUrU39afRint\/1Wzn7dkiEVm1BUI+lVdx9JxNNMKrwQCaNwDLuswQ4NP3TokNTW1pry6qDMlUCHm886wwuNBKZPW\/N282Pb59vESSQgEWojwU9uEkbwmHaoEdrF3LlzpbGx0aToA2EMGjQoc08Dq9y6wc1n\/oQDImlr3mY0EptEbJOmbEitnD+k3bTh5R8BEoZ\/zHw94aZhbN++XZAcGEuguTQMNMa9JL4g71AYJHL6gzXm3ol1TZnfQCI0ZfzjSsLwj5nvJwpN0RcUYZTabtWsgvrkv+TcicMiBz8WObCzvRiWg7FcfPn1wuXg7FOcu1V9v\/7+H1CnJRyR2O+B4wawz2PYsGGZylQTgUBw9AAOaFanKDef+cfc6xO69AstxM0XUjakziz30pRpR5QahteZlWc5EAES5WBVZNeuXQLH55gxY8xJaDhz1a9JoiZKTU2NSQGIiE4c9gy\/SBqXVfOENdTHsvlC0GipL\/GSMEKdeu0Zw22nJ170kSNHenZ6YmkUhx\/17dtXNm3aZOIq7HyhONcEuUFBPMW4rBqyeHxVj5UYrMxo3IgzZsRJJt37Vhs\/STFdJIyQpakaBk47A9izZ882LebSMPbv398h+ApmiTpKly5dKtQwQhacx+qdS7zZyES1E\/PXimxNWxAaCcPjxCikmG5zx4FG8GOo6eA1cAtt614RnPg+ZcoUc+I7fCDqH8FyLXerFiKl8J\/V8Ppcka3ak2yh83FqLiSM8OdJIC2ADNScsQkHhLFjxw7jF1m\/fj13qwaCdvyVOPfudBU6n017wf2gnbWfNowwQXNX\/WaX9O7dO3KgoopYjvUgI43WtNH1k7bPDg2394xAQ\/FDGFxWjXx+R9OgLhOjNSwXnzgichx\/D3dsP4Cl4z89U2\/qv+CRt5gEOAzp2i84Vkmqq6ulpaXFNKW5KvA527KqahZaVmM6aJKEIa3irBPOWWgoXe3JgV+lsumDnABQw8gJUWEF7FWSffv2CaI84cfQlZOuQsNBFvBN2PEa6I2SCJdVC5MNn25HAOZPtv04TqfsF\/8+VE6dOkWTJKzJA80BqxogiWPHjsn48ePNakcuk0QdpXa\/cBYEwslRpwaAwZEKbQOX\/YwzMCwq2y8sHFlvdAjoDmHnfhwsESO8\/vN+I+Wf5q6jDyMskSCys0ePHiZyEy\/19OnTO0RyhtWuXS8IgxcRyBeBX1WelGsvaJPLy\/9OXur\/gIkJwoFGcVxRnK8Tq9MzDlDZJhEgAvkjECthuGUNz2WS5D9UPkkEiEChCMRGGM5VjUIHwueJABEIH4FYCcPLikj4EHRsAY7TdevWyd69e2XixIly2WWXRd0FtkcEEotAbIQBRGCS4LLjLuJG6r333pOLL77YOGJfeuklc6yjl52zcfeb7ROBKBCInDCcSXOcgwzLh+GMBFXC0oTAutz6\/PPPy0033ST9+\/c3G+GwZNtVTEgUQmIbRCApCEROGHEMHDtcEeeBSxPw2LteETimyYfxFyRRUVFBwohDWGwz0QjEQhj6AiNQS4OuwlL7oVkgyfCNN94oTU1NsnjxYmNu6F4TDfjSjWvNzc0yePBgs6ENGgbMpQsvvDDRQmTniEBUCEROGLo3BNmwENrt3BMS1sBBUtgCbxMGsnwhGtResYEpArOkvLzcEMstt9wSVpdYLxFIHQKRE4a9hwS+AbzI7777rsllEebllTCc+1PC7BPrJgJpQyARhJEry1YQoLoRBvJlOE0SP8l7gugX6yACaUKgZAkjm9MzLF9KmiYF+0oEsiEQC2HojlK3ToW1rOrUMNC2hqY7jy\/gdCECRMAdgcgJg4IgAkQgvQiQMNIrO\/acCESOAAkjcsjZIBFILwIkjPTKjj0nApEjQMKIHHI2SATSiwAJI72yY8+JQOQIkDAih5wNEoH0IkDCSK\/s2HMiEDkCJIzIIWeDRCC9CJAw0is79pwIRI4ACSNyyNkgEUgvAiSM9Moua8\/t82jtQvZpcGkdtqZDwH6khoYGGT58eCYnrDPXinOM9ml7TLuY3wwgYeSHW6Kf0hfHfpkS3WGPnbNfeOwq9ksYaCaq\/Cseh5S6YiSM1Iksd4e7IgxNTdja2ipjx46VoUOHup5rq+fRYidvXV2d9OrVy5yDi\/\/ZkaUMiYZQl2Ytsw+lUk0GdSxfvtx0GIdt2+kYkWlNf0MCZlyaVxWfQQaalU1HjPpaWlqMRuE2RlvDQHtaP563237mmWdk1KhRJqMaL38IkDD84ZWK0m4miaYN2Lx5s6xdu1ZWrVplxqK5TDXPKQjAJgY8h5cXxJGNMGpra11fdtSv5+Xi6AYlG9wHYaAPOIh7wYIFMmfOHHn66aelsbExc2\/JkiUdMrbjGbQFsspmdqFu54Hb9nP4PYnHW6RiYokICSMtkvLRTy8aBjKNHT58OKNdaPUgiMmTJ8uyZcvMCw1bXzWJbISBhMl6XIPWAy0DL7cSg5oQ0BqgJahmYg9LX2zVSKDJ6IUxzZ8\/X+rr641mkEvD0FSLTrJAfdBUoIHY9fuAt6SLkjCKUPx+CAP\/uzv\/J8cLpS+6V8JwIwC7Hi+EoS8yRKKaRCGEkU2TIGHkP+lJGPljl9gnvRIGysFMgC8DfgH1byC7OpIy439g2ySZOnVqxtE4evTojKmCl1tNj6qqqkyZ6upqVw3DaZJoNnc8u3TpUvniiy86kZg+4zRJsq2SQIvB5XaqHk2S\/KcuCSN\/7BL7pFfCwP\/69hkxdnpEN6cnCMROa6jOUPs+QLGdnm4miTpM1YxZuHBh5sW2HalOgG3NoCuTBEdDwKTas2dPpgodG8ZsmzaJFWJCO0bCSKhgktStrl7iIPsZRRwFl1ULkxgJozD8SuLpKAhDD5MaMGCAOfohW\/b2QvwPTsdpSQgv4EGSMAIGlNURgWJGgIRRzNLl2IhAwAiQMAIGlNURgWJGgIRRzNLl2IhAwAiQMAIGlNURgWJGgIRRzNLl2IhAwAiQMAIGlNURgWJGgIRRzNLl2IhAwAiQMAIGlNURgWJGgIRRzNLl2IhAwAj8PwwRS0poW2QUAAAAAElFTkSuQmCC","height":161,"width":268}}
%---
%[output:7a92d0b3]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAQwAAAChCAYAAAAyeYEXAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQnYTcUbH8qaylLKHkUIkaxRlhZCGylbi49UZIlCeqKPkOSJkr6UpY0QZUkpUoqsyVZpQbaEytKC5P\/85t\/c5s53ljnrPffed57H87n3njNn5jczv\/PO+77zvjlOnTp1ilEhBAgBQkADgRxEGBoo0SWEACHAESDCoIlACBAC2ggQYWhDRRcSAoRAShPGn3\/+yQYMGMDmzZtnOtKtWrViI0eOZPny5fNtNnz77bfs7rvvZnv27GFvvPEGq1u3ruu65bqMKlHb\/\/nnn7P27duz4sWLs8mTJ7Py5cu7fnbYN7755pts4MCBcY81Gp8nn3ySZWVlMbuxk8e\/W7durH\/\/\/rxuo3mBcbr00kv5fKlfvz677bbbTLsvMLbCZ8SIEbwOo7aatR\/9X758ue\/z0c9xTHvCAJiYKC+\/\/DIrXLiwL9iGSRhq+5ORMOyIXR0fr4ShEhPIdezYsWzYsGHsyy+\/ZGKxm02GIAhDt0++TFAPlRBh\/AueV0lAHoOwCQPPFpM8GQlDZwHKEoLXxSUIQyaiX375hWVkZPhOGEZr04nU4WFtB3Jr2hCGPOEEkvLCVt8qRlsBI1JR344Qkzt37sy6d++ebUui1qkj2ViRjzzJRf9kwhg\/fjybNGlSbEtm9DyjbYCKhfwcgZ1Z29XFb7dtQH2iDeo2SsZWfp684GSsUZfZ1gPf9+zZ03CLWq9ePbZixYpsC8zsJeKElO3I4eGHH2Y9evTgRCUXuR92mMoE2LVrV16fioVf7EGEYaBrMFpEAnB5II0WkjowYtKZvUXtdA1uCcNsgsgL2KqfgjSs+qiShll9dsQo32dE7GpfxCI066Nou6rDSEbC0MHU7Bo\/pWaBddoQhhXDyotIXqDy5JUHRQyE0XfqAsO1FSpUiIm7ok4zhZzaTjulp7he3ZLgeyNyEARVpEiRWJtEf+Q2iXvx5lOVqEaSmdF3umK+GSmZEY1MGEbkINoODITS22gsg9qS2G2f7KQOoYTXxdQp4XqRNogwGIuzZBjtbwGw1dvKzFKB+7AYUYwsF0LqsHoD6xCG\/HxZkpHfMHZ6FTOxVxAG+mDVTjPcxPd2WxOrfqpSmFhwZhKO+B6Wr2QmDF1M586dy61LdtKqF6JIqITx999\/syNHjvA2nHnmmez000\/3oy\/Z6rDTvss3WJnBxHXym+Gxxx5jmZmZXD+gitHq4tyxY0c2c6H8bKuBtiMM9dlm+2sjwrBSNhq9pY3wUrExG0i7bYm4T0fasDJLYuGkCmHYbb1EPz\/44AM+v3Qx9rLYApUwMPgLFy5kf\/zxB2vevDkrWbIkW716NYOiB4sIpVixYvwt0KJFC5YzZ04vfbEkDLO9sfq2mjhxoqmNP9GEobMn1SUMbEmEr4iQhIQfAkhQlQjMFrJok+7kdmq6Ntr2RYUwdN7outsPq+vsSDglCGPr1q3svvvuY9u2beP9Bbh4K8PeDQJp1qwZ\/\/69995jBw4cYBMmTGANGjRIOGGYgR\/ElkSns3ZbCbUOXcIQ2yT5rWSkwzByaDO6TojFTt9yMhEZbVtkKchOClRF+KC3JEEThtmWRB1z3et05pvdNYFIGNhygDGXLFnCRo0axapUqcL38kOHDmU1a9Zkzz77LDv\/\/PN523766Sf2wAMPsAsvvJCL+Llz57Zrs\/bvTrYkYrLu2rUr9uZ1ovQ0syqYKT3RCR1\/gqAJQ0gX8EaVF6fAw2x\/rEpmBw8ejOFmZqUQnpbqANpJJ+KFIzxX00XCMFJ6GinLk54wDh8+zG3BFStW5HurHDlysJ07d\/IJ1aRJk9h38v53zZo17MUXX2SFChXSJgS7C50Qhizu65gb8WwnZlWrOq22GkERhmy5McJREAYwFA5NRtfJPhtmC9\/uTayDo53lAW3zImGYuYsbufX76YchLCLq\/BD91cE06QlDTACALd4qRt\/JhIFB8NM9G3XrEIbZZHbruAWRHH3u16+freOW3UJCH4IiDIyN2kcs\/jJlyhhadIwmrhHRqYpUJ1sUI1I1wigICQNYq203cxEPgjBU0jSzfKGddtYhp3oiuxev\/HsgW5KoEIYTIOhaQoAQsEeACMMeI7qCECAE\/kUgUMKoXr06V2iiHDp0iPvyX3755bHvxChACbp+\/XrftyQ0yoQAIeAvAoEShnqgxqrpTva6\/kJAtREChIAuAoEQxrFjx9iGDRsY\/uqWPHnysGrVqjH8pUIIEALRRCAQwohmV6lVhAAh4BWBQAjjn3\/+4ToLJxkM4Ktx9tlna7mHq+YvHZdpr0DR\/YQAIRBQ1HAdRxwVfF0dBsgCPg7C60\/9bDao5cqVo\/FOMALikCE8gfPmzcvOOOOM2MFD8VtQBxET3PVQHv\/RRx8F\/pxAJAxVhwFJ48MPP2SrVq1iHTt25M5BKDhDMnXqVLZ3715+zqRp06aWOgzhiAVnHtnNGI48KGaux\/gNhLF48eLAAU3HB2AcUTA+OIuDz3D5\/+233xICxznnnJPtuUbf4aJzzz2XS8K5cuXixxKgQ8O1IDQ4QJ122mncU\/m8885j8GDG9aK\/uA7\/F39RX6K+Q8DhH374IXC8AyEMtdWzZ89mYL8nnniCnXXWWXE\/Hz16lLuK42wJFrzVG0ZILrhOdtmFlAHSsPIUBWGEAWjgI5aAB2Dxo6xbt44TAj6L77w2R5wpQj34P\/5hjsD8XqpUqVj1+B7PlK\/3+uxUuj+s+R04YYhzJWBAHGE3KgsWLGBTpkyxPUsCV2YcjceBNjl8vs62JCxA0T+c0C1btqyn+eikDp1rra7BbzjZicX46quvcmKAX4zbgnqw7UA4g0qVKvEYp+KtHNSC18HAqj9O79e53g5zozlido\/6vfo5rPkdOGH8+uuv7J577mFt2rQxzfWAQLWIm2F3+CxZCMPtQgvjPrylf\/75Z64DciMpiAWPvy1btmSXXHIJb7ZKBDoLKoz+psszUoYwxFF3JGgZM2YMu\/jii+PG8KuvvuInW6+\/\/nrWq1cvyy0JEYbe9BfbBZAw\/GGcSgtia1C6dGnWrl07Q0KwawkRhh1C\/v6eMoQBWPBGg1s4RF3ExqhTpw5Ha+XKlWzTpk1cHzF69GhWtGhRSxS9EkZYSk\/E1IA47qXo1rF582YGHRFCHgqxX+e5QgmIEATXXHMN35IYKQaN2qHznW77ddqqc43X5zm9X+d6q2vMftP9Xr0OBoMwdHSBb0nEYP\/+++\/srbfeYm+\/\/TYnCRSQB95gkC5gYrMryaL09OPtKvQKwGTLli08dqjbLQQkBnjRIkyikQ7B6V7b6Hq7Pbbd2Hr93SvmTu\/Xud4prsAgLXUY0FsgtkGtWrVY5cqVfctb6tWsGgYDO5n4IABs2ebPn8+++eYbV4Qg6w9w2O+OO+7gcVITXXQWVKLbmErPT+otCRY2\/CtmzpzJI20hLN8tt9zCrrjiCv6G8xLsV3h5ygmCZEcus0kQBqBCdwBpCjobbMFQ3EgGaj+EZABSQOQyeNMGZXHwYyERYfiBon4dYcxvtCbQLQkcYvbt28fWrl3L3nnnHfbxxx9zG\/sNN9zAtyFupQ83ruEAFItY2PPFmxmLuWDBggy6AAQuhnSEIMW7d++OjZYgAr98D4ymgWx9uOmmm1j+\/Pm5H4IOKegsTjfisZmITFsSPdO5G8zTcktixovHjx9nGzdu5F6fixYt8l36sOLjxo0b69O1T1diuyEK\/g8PWEgf4oyN\/LtPj6Rq0hiBMLbcgUoYVmMnpA+89ZFqAG\/2rKws7o5LhRAgBKKJQMIIQ8ABfceyZcs4acBFHL76VAgBQiCaCCSEMCBdQBk6bdo0nq8EPgS6p1WjCSO1ihBIDwRCJQzs2XFi9bnnnuMh3VFgArz\/\/vu5IhR5VqkQAoRAdBEIhTD279\/P5syZEzvKLuB4+umneQ5PioEQ3QlCLSMEZAQCIwxIE8hmhoNlS5cu5Q5KcOSCYxEOLPXp04cfZzfKLEVDRAgQAtFEIBDCgMUDiZix\/QA5wGkLJxuFQtPMxTuaEFGrCAFCQCAQCGEIQoDfAY6143BTiRIlYh6eRBg0AQmB5EQgEMKA2zJOluKgmbCCIFgIyAPKTYRB69q1a6S2JCCxvn37skceeSQuOE9yDmu0W63GfDXLYRrtXiRP69Qcw16CZgdCGDKUaOyKFSvY66+\/zv0toMuAyzPcrF966SXWsGHDhCMvkhKjISK4cMIblcINQDjFCy64gAdUEtgjvAHps4IZdBwE3b59O39B64SztGpF4IQhP1y1lsA60qhRIx7CDTEcvVhLMPGGDx\/OYHlRvUWtzp7gbTdx4kTWrFkzNmTIkGzh\/4IZwtSo1S3m6gtlwIABPMwBEYb1vPADb6wF+D+NHDnS1SnyUAlDwIEtC45z40QrzK0nTpxgtWvXZhMmTGCFChVyvJqEiIsb1UDAumkJzILzOG5MmtzgB+aAymoRpAmUWt30ire8LYn0lsQODXh5fvLJJzyqOPQHTs+SyNKD6i3qJH4GEYbdSP33u1+Yk95ID3O\/8MbTvBocEiJh6MFkf5UAEkozlOnTp8dJGE4idBFh2OONK\/zCnCSLcPEWTxMv0fr165sG5Y6MDkMPIndXQbGjEoaTGKBEGM5xd4s5noTt6KBBg1zto523NDXucIu3COQklMxGqTp0EUpqCUPupFswRX4TIgzdKfPfdW4wh54K1jHEKJWLl32185Yn5x1u8IbVDwGpoVgWmHvBmgijfPnknD0RaLXbCSwnoYpAN5KmCVHAmwiDCMP1gonCBHbd+CS8MQp4pzRhOFF6JuH8SXiTjSYwYR7csEQB75QmDCdm1eCGOXVrNprAhHlw4x0FvFOaMGQzoNO0BMENe+rUbDSBCfPgxjcKeKc8YcgTWAylFy1xcNMh+Wo2m8CEeTBjGQW8U4YwghkiqpUQIARkBLQIA2c\/Dh06xHB4DImVkTQZwXDOPvtsT1nMaCgIAUIguRAwJQz5gNjChQt5ZG+1IGhv69atuYtphQoVWI4cOZKr99RaQoAQcISAIWH8+OOPbNiwYTxDGULsIZU8sn\/jJCkiZyGNIMLwrV69mqc\/\/Prrr9nVV1\/NA9AQcTjCny4mBJIKgTjCQHAb+PgjcG+nTp249GCXWEhkMJs1axa\/995772UZGRlJBQI1lhAgBPQQiCMMSA3IeYpEyW5yhGDb8u6777o6BafXXLqKECAEEomAltIzkQ2kZxMChEB0ECDCiM5YUEsIgcgjYEoYamRnq54UKVKEpxKA\/qJ06dKR7zQ1kBAgBNwhYEoYyCmydu1alpWVxdavX88TEYkgrV988QV7\/\/33Wd68edm1117LTa74fNZZZ\/G4nLCUUCEECIHUQ8CUMGD9mDFjBg\/Si0jcMKfKBQ5cvXv35rlGkNns8OHDPILS+eefz\/9SIQQIgdRDwJQwQAA9evRgLVq0MLV6wLd9wYIFPBs7pAt8hnl15syZqYcU9YgQIASYrQ4DyZNvvvlmQ6ggfbzyyiuxwLtLlixhzz77LJdKqBAChEDqIWBKGMePH2ePPfYYz5g0btw4fn5E3ZL07NmTZ7DKzMxkuXLl4pLG1q1bOWlQIQQIgdRDwNKsisWPLOw4V3LrrbdyN3GUzZs3820HEhC98MILXMn5\/PPPc4UnyANnS6gQAoRA6iFg64eBcyVPPfUU9wAFQaAgpSHOjiASMcyo0HcMHjyYVa1alXXo0IEnW6ZCCBACqYeALWGILmOLcvToUf6xQIECLHfu3KmHBvWIECAELBGwJQxsR5CzA+kMTzvtNNa2bVtOFvDNwBbljDPOIIgJAUIgTRCwJAxIFPDBwClUFJG7FHEv7rnnHk4gRgrRNMGOukkIpB0CloQBvwpYPEaNGsWBgS4D2dERFwPp15By7aabbuL+GhQ8J+3mDnU4DREwJQxIFw8++CBXZIIQVq5cyZ588sm4ZMeIm7F06dKY41Ya4kddJgTSCgFbxy2QRsOGDXnWbpUwli1bxsaMGRNHImmFHnWWEEgzBGxdwxs1asQ6d+6cjTBw1gSOWhs3buSkAcsJFUKAEEhtBCwPn8EpC2dD4JCF4+5CwsC5kQ8++ID7YeBIO\/6RDiO1Jwr1jhAAArZWEjhkzZ8\/n59C\/emnn9hFF13EUw0cPHiQO2898cQTtnE\/CWpCgBBIDQRs\/TAQFwNenpA04Hvx119\/serVq\/Nj7Yj9SX4YqTERqBeEgA4CtoShUwldQwgQAumBQBxhwJSKoDnIcqZbkP0M3p+k9NRFjK4jBJIXgTjCcBLHU3RZeH8WLlw4eVGglhMChIAWApZbEuQpGT58OKtUqRI\/3i5ylfz5559s7ty53At05MiRrEGDBloPo4sIAUIguREwJQxkQYMZFSZUI9dv4Yfx3XffcdLIly9fciNBrScECAFbBLQ9PY1qgvUEPho4X0JbElus6QJCIOkRsD1LcvHFF7NevXrxoDlygQSC7cquXbvI0zPppwF1gBDQQ8BShzF79myeMgBH2RF2D85bKHDgwknWF198kR9\/hz8GFUKAEEh9BCwJQ2Rzx\/F2EZ5PQIKgvwgSDCJRpY\/Uh416SAikJwJajluwliDbGYL\/oiDSVq1atVxleE9PmKnXhEBqIKBFGKnRVeoFIUAIeEUgm+MWFJmdOnVi1apVc3QCFWbWDRs2sOnTp7MRI0Z4bRfdTwgQAhFEII4wsOgR7HfIkCGsXLly7K677mKXX365pY8F3MmXL1\/OTas4xYp7r7rqqgh2lZpECBACXhEw3JL8\/vvvbPLkyWzixIkMXp0wrao6CxxvX7FiBdu2bRsrUqQI69q1K89JQqdXvQ4J3U8IRBcBSx0GyAKkABMqQvQdOXIk1hOQRM2aNfnBs3r16jny9ES9CL7Trl07Vrdu3Th01PMs3bp1Y\/3794+7Bm1p37597Ls33ngjWz3RhZxaRggkLwKOlJ7IUfLHH39wKcJLhC24nGdlZTF1oQuyuP3227m5Vv0MmEEW\/fr14xJQ+fLls31O3qGglhMC0UfAEWF47Y4qPaiEAUkGSlPZ1VwOPozzKpBMihcvHid1gIBQVEnEa3vpfkKAEIhHIDTCEGSBXKwIKty9e3c2evTouK2E0cIX94EMkPQ5IyODE4O8lTGKaK4OdOPGjQMfezi6oYi\/kMYQoQzKZPn7wBtCD0hLBH744YfA+x0aYcg9QerFu+++O44whF6jfv36cdnf5W3JZZddxpMnwfMU2xFR1G2KEWogG1iAUODivnv3boaTtnBGg24G7u4o6t8gR0C42uMvtnhQGkMvhDaIGKrimiDbEUTdUIaXLVs2iKqpTgMEYNUkwmAsTo\/hhTD8AFRdyPgMa9Fnn33GvvrqqxjhCNLxc2bL5ILgyy1atIgRi5\/P8asuIgy\/kNSrx4\/5rfOktJEwwgIUoIvFIghGllzwf0gTCIUo3ggIruxXAbHg3wUXXMBuvPFG\/hftgARn9sa3WtxGv+l8FzZheH2e0\/t1rneKqzx31Pmg1qV+Dmt+axEGnLOwjcDeHB6gSMJ84MABVrRoUZYzZ07Hcz0RWxIAunjxYsdtTdQNwPecc87huEOCgVMcvsM\/rwX1ouBMEJzsEI8V34lneq0f9yPsQcmSJf2oiurQQKBp06aJ35KAIGC5yMzM5KdVRfxOnE5FFC5YKx599FHHAYCNCAOY6Co9W7VqxYYNGxaDEW2YN2+eZSCfsBhYY2w9X2IkuWzZsoVjgOKHxCJvgWrXrs2JBS8HXZ2KzhvYMxBUQQyBsOa3pYSxZMkSHjynT58+rFixYjz+BUyeCNs3Z84cvmj79u3L7rjjDkdDZ0YYOmbVLl268PSMb731VswPA\/E6EJMDoQLNSliAWomVTkBysuDMrpV1Ljt37mR58uTh\/it+kQrqEQRSsGBBHvu1efPm\/BSz1RbICQ661zrBy6hOp\/frXJ9WWxJhtcC2A2bMNWvWxCVj9pJb1YwwhEUEJlM8U3XcQptAGPA+FT4csJCAMDBRhYRiNCHSkTBkHKwm7+rVq7nrv9B14GUAgsFnvxW4suQCnQpy92KOyVYhWYIiwvg\/ApHXYYjF+sADD7AmTZoYZm+HBILI4U5jepoRBoCxcg0Xv6X7lkR3EQV1HRY0xgI6IShugyAWte0y0eA3bIdBNHXq1GHIjQNFsiAdWfIJCoOo1RvWC9F0S4KgOXhzt27dmsFV28g5atKkSWzp0qU8izu2KUEXEI1bP4ywAA0ag2SpH29EEUkef7HdDNr07AQbVRcjfwYZQbF\/4YUX8rzBVatWZVD8y6QUNXIKa37bphmAvmDcuHH8TSKytyNCOAYfik9IH9g+hBGmL1kIQ2d\/aze5ndShc62b\/bSZPiZIs6rYAmFBQnn76aefsu+\/\/z4GVxjSjN3YBP27kWIZBgh5jeGzbIXCCx1b9aCLpdITpryePXvy1InI2r5q1Spu28cALlu2jNv4cYgsLI8+r4QRNJhUf3QQkBcXJANIDHnz5uV\/1Zeb+BzGSy9IhD766KMgq+d12\/ph7N+\/n+cegVVCHG9HAOCbb76ZQb9RokSJwBspHuCFMEJrJD2IEEhhBGwJQ\/QdVhEQxsmTJ7mSyY3Dllcc5YNoTg+feX023U8IEAIaEgb2SpAyoPyByAYHIUgbuXPn5lIGDnWFVYSpl463h4U4PYcQiEfAUsKAcuvBBx\/kZDFmzBjuOgzLCU55osBBZ8qUKaxGjRqh4Sqibcl+GHJAndAaQg8iBNIQAVsrCXwt4EGJY9evvfYaGz9+PPe7gMJz4MCBPPoWXMchcYRVKERfWEjTcwgBTQnj8OHD3GwKl2v4YYjtAG4X2drhEfjKK684dtyiQSAECIHkRMA2e7vw9ISrMILeIPguImahLFiwgL300ktEGMk59tRqQsAxAqaEISQK+F9A0pg\/fz575JFHuEQBnYXI3g6FKCQOSi\/gGHu6gRBIOgS0sreXKlWK7du3j6cTQBxOFITJQ8DeoUOHxoXUSzoE\/m0wTLY4eQtSlMP\/JWt\/otxu9bwQMuUhSjyVYBAQL38R\/sBLWg7b7O0LFy7kJ0ORAQ3H2GExQQOQub1ixYqsY8eO\/Nh0MhdxGA59EOkLkrk\/UW87jhhAaQ6SsDqIGPV+JEv7cI5n+\/bt\/AiHTsBsq35pO25FHRxMPOSFffrppxnOusjFyqqCtx0yvDVr1oyneVQDDEe934lsn1vM5TZbJbVKZN+i+Gw\/8MZamDZtWsxw4bSfWoSBk3rHjx\/PVjf0GFu3bmXVq1d3HHXLaUOtrhciLq5Rj9rrJj4yczv3s52pVJcfmAMPq0WQSnh57YtXvOVtSWBbEhw+w54evhhmRYTtU9\/qXgHSvV+WHtS2OPEMJcLQRfz\/2edEqkovmJPeSA9zv\/DG08yOV+i1xMI1HNID9pqwisCciomBEH2IzASl4OzZsxkOoUFhBVdtL6kTdRurXieARBtQ1KxpTs6eEGHojYJfmJNkES7e4mlm+X\/0WmNBGMJxCwQBsyoKFiaOB0N5goNosCogWnEUNNxG8UCdnG4lwtCdMv9d5xZz1DB16lQ2aNAgR0m8nbcwte5wi\/e6des4EELJbBSEShcpbcctVIgGL1++PKYwwRsdQTuE56fuQ4O4zi2YwoRKhOF8VNxgjlAJcPYTJj7xVC\/7auctT8473OANqx8C7SAncaBmVSFhIMOWkCAgjsIKga1JoUKFuG7DTUzPIIbLLZjkc+F+NAhz99i5uTMKeNsePoNLOLYiIAi8hZF2AJ+R0AixPJGvVBCIGxD8uicKYPrVl2SphzAPd6SigLelWRUm0\/vuu487a0GURIgziDbYliBTFkL1devWjZNIosObGYHpROkZ7tCnxtMI83DHMQp42\/phIOUdAgFDuYkj7AjCCusJtiOIKI7DaZA+El2MwHRiVk10+5Px+YR5uKMWBbxtCSNcSNw\/zQhM1EYBd9xjancnYW6HkL+\/RwFvW8LYvXs334JA0jAqiO\/Ztm3bhHp6ol1mYMqkQRr5cCYwYe4vzqK2KMxxS8JAToju3bvHooUbwZBoT89ghoZqJQQIASMEbONh7Nmzh+sskHskEd6cNGyEACEQHQRsHbdwpB3RwakQAoQAIWBKGDihiojhUXH9pqEiBAiBxCNgG3Fr1qxZ7JlnnuGZsqkQAoRAeiMQRxiQKmbMmMFzqaIgBsaiRYvYX3\/9xa677jqeh0QtUbGSpPcwUu8JgXAQiCMMNdaiThPISqKDEl1DCKQGArZ+GKnRTeoFIUAI+IGAKWFgO4ItCtzBCxQo4MezqA5CgBBIcgSyEQYibSHK1tixY2MOW1dffTUbPHgwK1GiRJJ3l5rvFIFTp06xd999l+fWRa5dZMKDXw7y0Ozdu5c99NBDfG6gIDIbzhYZBVQyypzntC10feIRyEYYmBy9e\/dml1xyCWvQoAE7ePAgz9aO3KogEZxcpZI+CCBRVZcuXfjp5A4dOrAiRYrwuYHTyR9\/\/DF7\/fXXOZmAPIgwUn9exBHGsWPHeNBfhN9DoBxhFcHJVLiII3pP3bp1Ux8V6mEMAZE3RJUcIHk89dRTLH\/+\/DyEo9l1oiKSMFJjUhlaSUAKiNspCt4eGRkZ7M4774xE\/M7UgD76vcBhp4EDB8Y1VITSExHZ7r\/\/fv4ScUoYc+fOzVa3eBCCSlNCqWjODy3CEOZWlUii2SVqlV8IINoaTiojPSaOB1x55ZU82x22J5s2beJpMuHUV6xYMceEceDAAbZjx464pmL7A\/3IFVdcwR5\/\/HFStvs1kD7WQ4ThI5ipWJWZ5ADp44svvmCZmZnckiauw2FFq9KqVSvDoNGwyEF5inAK48aNI8\/iiE4mIoyIDkxUmmVEGDC5I7dujRo1YltUcV2lSpVY5cqVszX\/xIkT7MMPP2T4XY0yD8scgkmLvDJVqlSJSvepHQoCRBg0JSwRMCIMbFX69evHhg0bFst071SHkS+KsRwLAAABC0lEQVRfPv5cKE9xXgkEBGmlTZs2FEYhwnOSXMMjPDhRaJoREcBqBmkA5lTh1OeWMESg6ZYtW3IfjkQHk44C5lFug+XhM52G0+EzHZSS9xqVCFRzquiZG8JA7t6ePXty3x6kriCP4ujPEzpLEv0xSmgLVSJQzaluCePkyZNcybl+\/XqewqJChQoJ7Sc9XA8BIgw9nNL2KpUwVHOqG8KA7mPSpEnccxgOgXXq1MmGb5kyZVipUqXSFveodpwII6ojE5F2qYShmlPdEIZQcKr5VeUuY4sShSTfERmGyDSDCCMyQ0ENIQSijwARRvTHiFpICEQGASKMyAwFNYQQiD4CRBjRHyNqISEQGQT+B+65i+BFbGHKAAAAAElFTkSuQmCC","height":161,"width":268}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAQwAAAChCAYAAAAyeYEXAAAAAXNSR0IArs4c6QAAHNFJREFUeF7tXX+Q1dV1P\/gjYSUoLpIaum4WlI02ToiSOIaaWVLHYNsBU6YdfnRaBhiGoMi0BYEFHGQmrEDZ\/AFoJHa7JW13cTAkwtQMoTQgulqtyDatGlRYcEWNggohOG0TOueL5815993v93u\/793vvXffO98ZR913f5zzOfd+7rnn\/hp0\/vz58yCfICAICAIGCAwSwjBASZIIAoJAhIAQhjQEQUAQMEZACCMFqueeew5mzJgBXV1dcOuttxoDa5JQLfvcuXOwceNGmDt3LtTX15sUkZoGy1y2bFmUbu3atVBXVwdY77Fjx2Dq1Kmp+StJ8Nhjj0FPT0+h3qSyNm\/eDBMnToQxY8akVpklbWphnyQgnHbt2gUjR46Ezs5OI1l0+JrWaTPdqVOnYM6cObB06VLr7ZTLKYRh02oVlrVu3bqoM3d0dORGGP39\/TBr1iy49957cyUMasDTpk1LrQeJZdOmTUadNEvaLOZ47bXXIlwmTZoUdTrTLxTCQHmx\/Zw4ccKIoE31U9MFQRhkLFQWPxrNyRinT5+O\/r5\/\/\/4S9qdRGn8fO3ZsobPR3++\/\/\/7ob1j2gw8+GNt442TgXgCWj6M1yYN5cCRqaGiI\/o6jE37z5s2LGh2VSZ1T9Sj4\/+OI39raGuVXRzhdo1TJJQ1DLHfJkiWwYMEC6O3tLZITO2Fc3VjPli1bIpkmTJgA+\/btK3RsPipjgRxftWMTgVDdlJbbj2zf3NwcjZaqnLq06PVx+bHDkyelNvY4GdS\/68pQdSUb69oob4fUkRHDuDaKZeHvVGYS5qqs3PPN0xsmLL0ThtqpeEOjjvjiiy9GjXT48OFRQ2psbIwaBR8tJ0+eXOR6Y2PDqQQ3QtzorY6GvDMePny4MCUhwiB5yH3mzE71oiFRXj6aJxEGNvwkDwNx2bZtW0R++CEOmEdHTDoMMY+KGU5JEP+2tjZob28vlEv4ki7YuQlfrnscTqQLH+142j179hR5FCq5YNqmpqaI3IkMqGOoaTmmRDSECycMsjH9ptoizcOIs7HaJrBO1ebd3d1F2JMXQzJQG8W89Dcd5tQfyJY7d+4swjGLVzdgPYy4kQgNu3DhwpL5N09\/8ODBkoZHnYo6Oo1kSa4sGmnx4sVal1jnYZDBMB6QZKQsHkYaYVBZGzZsiGzN4ypZMIybkuhGaYynoNdE83leD5E3dUCOg0reiBONmuroi7robBM3kurIhQ8EcW65Ll7EYzuEi25KkmRj8jCOHz+uJXPqmKQ\/90B1HgGmi8NcJSPeJtAOKimWSwpJ+bx7GOqoypVOI4wdO3ZErhz\/yJ0\/efJkYqfiedLIhDonjSacMFRS4OXaJAxqmKgfjUQU68iCoUoY1Gixo6xatQpWr14dlY\/eCBIG74wcJ2q8NI0kvXG0xKAt9wSRMNQpEycOnUeEnQa9iiRyVKeCJIMJKanTvCTCSLKxWg7+P\/f+iIg5LnFeDsqv2pJjQ21a7dA0KFLfIQ8Rcbf9eSeMLKMjApDkYXBwVPY2JQV1JcTUw9C5wTYJg4\/EI0aMKExHdCN0EumqhMEbKOLLR90sHgbHPikQyGMB5IrriCgu7pPmYcR1EBsehs7GSYShDngqmVTqYai61oSHYRLDoNGG5qhZYhhxc18Otgq0jtWxHJ2HoY4KOArQHPaOO+4oGm3ILSWZ1AaTtkrCR2ke7DLBkLwGlTB0ulLQj8cwSJf33nuvMEVJi2GQd6ISUZIMfKpDHY7sTwFOvqLiMoZB+nAbq9MvlRTU2A0Gl4kodYTBYxgq5hLD+KTX8o7AVwg4ew8dOjRyUVV3M22VxIQwUIwsqyR8SoL\/HRdBp9GfViAoGh5HGFwX3b4Pdb7M92qYYIjTDPxoRQeJga+cIPboveDHpzt5rJLwlQguO7rX+BFmaG8kKfI41LQ8MIr5sqyS6Eg3blk1bZWE2oRKGKpdEF81qKzaumZWSRAcbIDr168v2fSiAq6bZ6ouVkhr3LbnggOhvKwekM5ry3sj0UDAMYuMlWCO3qLpRrksMvG0UQxDjUgnFRbX0alzq0uOVBbWsWjRIli+fLnRDjo+cqsjernKSr5sCKgkj7mz7Hh10YCzaRR+6nIxd7rTEyvD6DhGyZO2JCelw8aBriMSho4U+Hq\/rW3P4ZtfJBQEqgsBK6skSAZbt26F+fPnw8qVK7WEwdf5EcKkXZfVBbFoIwhUDwJFUxIe7DJVEV2oNWvWwMyZM6OdjSbTDlfuk6kOkk4QEATMECh4GOrcyfTEnrq6gNWm5aW6xo8fX3S2Y\/To0WZSSypBQBAoQmDv3r0watSo3FGJnZKUM4VICmxiefjR+QXdagoSxpEjR3JXWirIjoDYJjtmLnO4so9RDCNLUJRPSThJJJ1sJGBdKe3SkNVSl9gmbEu6so+WMNRl1rQphi0oXSltS95aKufo0aNOXN5awtSmrq76jnYfhiuCUAFzpbRNQ9VKWUIYYVvaVd+JCAOnC\/iP7\/0RrpQO2\/RhSieEEaZdXE\/nCx5GpRu3bMAphGEDxXzKEMLIB1dbpbrqO9a2httQ3JXSNmSttTKEMMK2uKu+Y7RK4goqV0q70qea6hHCCNuarvqOEEbY7SAY6YQwgjGFVhAhjLDtU3PSCWGEbXIhjLDtU3PSCWGEbXIhjLDtU3PSCWGEbXIhjLDtU3PSCWGEbXJvhMHPfNCdj3F3XNiG0JXStuWuhfKEMMK2squ+U7RKwt++mDJlSnQpzooVKwBfWMr7rkA0hyulwzZ9mNIJYYRpF5LKVd8pIgx+KhWvUifCQCIx2QlaKaSulK5UzlrML4QRttVd9Z2SfRj01Nzs2bNh+\/bt0bV7+ICv7hEX2xC6Utq23LVQnhBG2FZ21Xe0G7fUV7Jd3b\/pSumwTR+mdEIYYdrF65TENyRCGL4tEF+\/EEa4tul+4R24p\/sVOPXdb+QupGwNzx3i6qhACCNcO3ojjLQHjfK+WEc8jHAbpRBGuLbxRhgICd7D2dfXB\/jEHX30t5aWFuju7gZ8ozOPp+SFMMJtlEIY4dpm3e4+WLf7qPspSdxlv\/R3fE1748aNqS+kpUGLKzH4cVLC\/xfCSEPO3+9CGP6wT6vZG2Ho3keld0fGjRsHd911FzzxxBMVeRi0AkNP3nMwhDDSmoa\/34Uw\/GGfVrM3wiDB1GVVfIC3ubnZ6FWzJOXIUxkzZgycPXtWPIy0lhDQ70IYARlDEaXrhXdgQTWukuBUBOMgx44dK4mTyJQk3AaJkglhhGsfXFLFwGdVLaui17J\/\/\/7Iq9AFVoUwwm2QQhhh22byQy\/B02986Icw1CcSCaqxY8dCR0dH2U8RoHexZcuWIuTVOAbGMPDDdyLlCwuB\/v5+aGhoCEsokQZuv\/12+PBbHRESzj0M\/jbqjh07oukDniHBzt7U1FT0cHIlthIPoxL0\/OSVKYkf3NNqPX7qY\/jyd571Rxh0KnXPnj2FOIPp26ppytHvQhimSIWTTggjHFtwSVb8+HX43lNv+iEMWlYdP3483HzzzUAvrB88eBC2bdtW0ZTEBG5ZVjVByU8aIQw\/uCfV+vTrH8Lkh1+Kklz06\/fh\/Uf+LHchS86ScG8CvYzW1tZICFxaxelJnp8QRp7oVla2EEZl+NnOzaciWPblP10Kff\/1vO1qSsqTw2e5Q1wdFQhhhGFHJAr8hzwLlOqN73wdxt3YDEeOHMldyNgbt\/jDzLZjGHFaiYeRu73LrkAIo2zorGUkosB\/03dg8VfhiyM\/4+xYhfHbqnghcF6Hzkh5IQxrbct6QUIY1iE1LvDh\/W\/CyideL0rfWD8Ydt59E+C\/8XPVd4w8DGPNKkzoSukKxazJ7EIYbs2+5idHoH3PsZJKkSDW\/UkzTPzi8KLfXPUdiWG4bQcDtjYhjPxMh1OM\/zj2EfxDz4lox6buUz0KNY1Twki7OAeFq3SnpwncrpQ2kUXSFCMghGGnRSA5nPvf38B9jx+OJQeqCUmia86X4DOfvrgw9YiTwlXfEQ\/DTjuo+lKEMLKZmAKTG\/b0Qd\/751LJAUtHgviDL9TDX93++VSC8OphZIMiv9SuWDI\/Daq3ZCGMUtsSKTzf9xH84NkTcPyDC0ueJh8FKzdNvR4+P7wuM0EEQxj8qUQSysUKCdYlhGHS1PykqVXCQAL47fnz8L39\/fDK27\/KRAp8atF45WDYOO16uGjQoIrJQdcCXPWd2KcS+fV59LiRLKv66awh1FqNhIFkgCP9oTfPwKNP98ObuCkqg5fA7UIew9evuxLu+2ZT9BP9zYX9vBBG2p2eq1atKvt4uwlorpQ2kUXSDPygJxLCsVPnYPuL70ZxhHLJQPUUbvjcELh7QiMMckwKSW3SVd\/RPpW4a9cu6OzsBLxKj+70xGmJemmv7U7lSmnbctdCeaF4GBQjwJWGf\/r3t6H3zTMR\/HHLkVlsgx4BTh1GXVUH3265Bm64ekiW7F7Tuuo72lUS9RIdeSrRa1sIovK8CYOI4PC7Z+Fffv4+vPHeryv2CHRTht+\/dhj86c2\/A9eOuMz5tCFPQ3oljDwVC8Gt8qXfQK63HMIgEuj\/4GN4ove9KGhoyxvg04QoXnDlYBg9og7ubmmEwZdeVFVkYNJuhDBMUJI0zhBAwrj4is9FgTwkgn\/7xSn4z\/4z8Pov7XoCOiK4ZdQVMPUrV8OnL6k9IjA1sBfCoB2fjY2NuR800wHhSmlTI1R7OvIAkAT2vnoKfnzol3Ds5LlI7UoDhDrsaNUAvYFr6gfD3NsaoH7IpTXnDeTRrlz1nZIYhm4fhu7RoYGsdB6yh1Am3zR08Php+NdXT8HxHAkgmgp8cloSSQA3IN094RoY8qmLhQQcNwhvhKHTE58IwL0YldwaboKfK6VNZPGdhvYI4L\/fPf0\/sOPQu\/Dfb12IAeQx+uumAo3D6+D6q4fA5C+NgN989HZhSuIbG6m\/FAFXfcfIw0h7tZ2\/lBa3K1T1XHSH2Vwp7brB8ZH\/irpL4PsH+uHAax\/k3vlVD2D0iMtgXOPl0NJ8ZWYPoJygp2uca7k+V32n5D6MOXPmRLibehP8aQJ8t2LZsmWAlwhPnTq1yH48He7v0H2ulLbRsGjL8JM\/fx9effdsYWNQ5AEYnifIKgffOXhhVeAymN\/SAHWXXpgCcILIWnZaeiGMNIT8\/u6q71g9rUpexPTp00suDMYNYG1tbdDe3h67W9SV0nGmJRJ45Kl+ePmE\/SVAtV4+\/x81og6+NmoYjL92WGElwuXW4rTmLoSRhpDf3131HWuEQdOSuCmJyWYwV0o\/d\/QjaHvyiNVYgLoC8OWGoXDH7w2PDhvlOfK7aqZCGK6QLq8eV33HGmGQmkgMPT09icuytHyLW8350wV5KI1ew+afHYe\/e+atTJbgBNB0VR3cfn093DX2s4XpRkijfybFykwshFEmcI6y5dF3dKJbJwyTFRX+YBKPdaDS+Nl4W3XDU6egu\/d0rLlGXn5J9NttTXXwFzddEf03\/c2RjQdUNfK2apjmwrdV6RsQzwyoJ1xx+RU\/9aAaeh74IUFgPINeVeMBUBsseU\/3K9D9wjsl1kWPYNmdo2DaV64O0\/KBSyUeRtgGstF3TDS08sxA3LIqJwl1WVV3oK0SpfmzcaR42sWpJgBJmgsICGGE3RIq6TtZNBvwzwxgjAI9inW7jxb0ji5FWfm1LDhI2hQEhDDCbiJeCMM3JFmVRrJY0P1K4S4EJIrN026A264b5luVqqtfCCNsk2btO+VqU5iSrF69GhYuXAiLFy+G3t7ekvJCfGZg8kMvFZEFfwmqXEAknx4BIYywW4ZTwggFiixKd\/a8BYsePxyJLrGK\/C0ohJE\/xpXUkKXvVFKP9WXVSoQxVZo\/dS9kUQni5nmFMMyx8pHStO9UKpv2LEnIUxKVLCS4WWkTMMsvhGGGk69UXggjTlncW9HS0lJyPsQ2OCZKt\/7oNdhyoD+qGsmi1nZc2sbctDwhDFOk\/KQz6Ts2JDOaksQ9P2BDAF5GmtLiXdhG3Lw8IQxzrHykTOs7tmQyIgyT7d42BEpTmq+KiHdhA3HzMoQwzLHykTKt79iSyTiG0dXV5X1KUv83P4v0vu3aYbDznptsYSDlGCAghGEAksckXgjDo75R1UlKt\/3kKOBL2BK78GMlIQw\/uJvW6o0w1INheB5k27ZtxjdwmSqoS5ekNHkXsu27EoTLzyuEUT52LnJ6IYy4G7NM7riwAUqc0nhWBE+h4vfQ9Btg+lflxKkNvLOUIYSRBS33ab0QRqiPMVOwU7wL9w2RahTC8Ie9Sc1eCAMFQ29i06ZNhceY6XYsvBnL12PMEuw0aTL5phHCyBffSkv3RhgoOL3YfuLEiUgPn48x83suZCm10mZVfn4hjPKxc5HTK2G4UNA06CnTEV\/WKK5XCCMMO8RJ4YUwXO3oNFWa7+yUvRd+G6wQhl\/802r3QhgoFJ4baWpqKnmIKE1gG7+rSvPpyNKJo2DpxCYb1UgZZSAghFEGaA6zeCEMCnCGclqVL6fi5Thyk5bDFqhUJYThD3uTmr0Qholg5aZRLwHWbTVXlZb4Rblo288nhGEfU5slVh1h4HJtX19ftDQbd5iNK10Uv7huGKCHIZ8\/BIQw\/GFvUrNTwqBgp6s7PZEwuru7S15H40pL\/MKkmbhLI4ThDutyanJKGOUIWE4ePi1Jm5J09pyARY\/\/IqpG4hfloG03jxCGXTxtl+aNMFwcPjN5W1XiF7abVGXlCWFUhl\/eub0QhqvDZ2lvq\/7jj3bDpK0XruEb97uD4ftT5LBZ3g0urXx5WzUNIT+\/D7i3VU1hyvK2Kg94yulUU4TzTSceRr74Vlq6Fw8Dhc7r8FmWZVUe8JT4RaVNyU5+IQw7OOZVijfCQIV8Hz7reOYtuO+HFx4pEsLIq4llK1cIIxterlN7JQzXylJ9pLQEPH1ZIL5eIYzwbMIlqmnCkPsvwmucQhjh2UQIY\/RoOHLkCBBhyIGzcBqpEEY4ttBJUrMexg9+ehAmP\/ySxC8Ca59CGIEZRBGnZgljzWM9hQt\/5YatcBqpEEY4tgjKw8BzHjNmzCiRaezYsbk\/NYAs+cdtT8I\/P\/92VL8QRjiNVAgjHFsEQxhxW7ZdQYWEceOiH8LTb3wYPbIsL7O7Qj69HiGMdIx8pvAyJQnhir4Pv9UR4S5X8vlsfqV1C2GEZQ9VGi+EgULweytcQ9R04y1w+pvromrxsSLcFi5fGAgIYYRhhzgpvBCG7yv6OGHIkmpYDVQIIyx7BONh+ISl8ZY\/hF\/dtiQSQbaE+7SETEnCQj9dGi8eRrpY+aYY+Ud\/DR9fPzmqRFZI8sU6a+niYWRFzG16b4TBT5VOmjQJlixZAitXroTly5fDmDFjckXhs3\/5CPzfVV+I6jj13W\/kWpcUng0BIYxseLlO7YUwiCxGjhwJU6ZMga1bt8KKFStg586d0NPTU3IHp21QiDBkSdU2spWXJ4RROYZ5luCFMPiy6smTJwuEgUSyevVqWLVqFdTX1+em91Xf3g6\/vewqWVLNDeHyCxbCKB87Fzm9EAYqhi+f4SPMs2fPhu3bt8P8+fNhwYIF4OL1djml6qJplVeHEEZ5uLnK5Y0wUEF1e7ir19vllKqr5pW9HiGM7Ji5zOGVMFwqyusiwpAlVV8WiK9XCCM8m3CJhDCuGxa2hWpMOiGMsA3ujTDUy3oRJlxeXbt2LdTV1WlRw+3kra2t0W9xp1rVcnXpyMOQPRjhNU4hjPBs4t3D4Muq+AYqfvQ3\/G8daeCFwW1tbdDe3h6toFDQVE2LKzCLFi1K3M9BhCF7MMJrnEIY4dnEO2HEnVbNcoo17t1UlVh08CNhyB6MMBumEEaYdiGpvE1JdB0evYampiaYOnVqKmpxafm0BQvRrbwIYaTC6y2BEIY36I0q9kIYSadVSeqkm7dMj8bHXdSDhCH3YBi1D+eJhDCcQ56pQi+EkUlCJXEWLyTubVUkjE8dfwae\/ds\/r0QUyZsDAvK2ag6gWijS69uq5cqPZNHS0hLtBo37TN5WRcKQezDKtUK++VyNYPlqUb2lu7LPoPPnz5\/nMGZdVtVdGkzLsHhoDT+MfajlxsUwLjv495GXIZ8gIAhkQwDf9Mn7KyIM3bIqChC3VJq3cFK+ICAIhIVAEWHYWFYNSz2RRhAQBGwiUDIlQW9i165d0NnZGV2YQy+54zSDNnPZFEDKEgQEgYGDQAlhoOgmeyYGjooiqSAgCNhCQEsYtgo3LYcTVFdXV+Jqi2mZkq48BHgQO+5aA\/I68d4U\/ObNmyfeZ3lwW8tlcvTCRmXeCYNvGT98+DB0d3fnfhWgDeCqsQze6FA\/fkaI6xu3\/b8aMRkIOhGBo6wUSshLbu+Egd4F3ReKqzRpB9TyAkLKvXBxEsawOjo6opPJy5Ytg+nTp5d4fKY7egXT\/BFAkn\/00UfhzjvvhAceeADWr1+f62XdQRBGX19f5NL6fts1f\/OGXQP3HFBSJIzx48cXnSFS99PghdF5j2phoxaGdOhl4A3\/Qhhh2KMmpDAhDBUI7pXkeUF0TRigAiVrijBkSlJBS7GY1XRKwqt01VAtqlmVRbmyg\/cpiQQ9w2m\/JkFPnJKsWbMGZs6cGc2VeQwq7ka2cDSsXklqhjDQhLSsKvNh\/w2aL6vyJW718OCsWbOi5yjEZv5thhLUFGGEAblIIQgIAmkIeJ+SpAkovwsCgkA4CAhhhGMLkUQQCB4BIYzgTSQCCgLhICCEEY4tRBJBIHgEhDAcmMh0K7XJUwxZxeUHxUwP9oV0VoRWbZIun86KCU9P5ac91lVJHdWUVwjDgTV9EgZ2iP3792c6TRoaYeR9IBFJdevWrbBixYrY1\/0cNJMBUYUQhkUz8T0MNCLiCdwZM2ZEtdAxcPUeVBz5m5ubYc6cOdDb21t4bpIOgOGFRvgleQi8TBotsSyqO24E5VcL0HF2IoyhQ4dGdaqjO8\/D92HgwbWXX34ZDhw4UHh3hu+xmTBhAmCZdBGTTmZ185dKXljHkCFDYO\/evRFWcUfrVW8tiQSFMMw7gRCGOVaJKdX7CPhGJ+5hqBts+E5JvMpffXYSK8UOhg1+8eLF2oNeNO3YsGFD1Lnx0Bh2ZMoXN0LzTsRPCp88eTIiGiILtTw60UpPY5KM6t2vXNfhw4dHhIg3y6Nc\/LeGhoYimdUpA5ef3whHZWJ56o31QhiWGrZSjBCGJVyTLjBJmpLwTssJA8XCDkadgU6J6o6bq6MnPxOSdMdI3Fsy6oGyJPn5b1gekQf+W83H\/1\/dUh7nAeg8jKQ6yJxCGJYathBGPkBiqTzAyKcAasfBjrVly5aCIJRWRxjodvNPdwuW2vlMzufEPSaFdamdlMuve4aCpgUqASURiHoNJNarC2zqCIM\/2xlHZkIY+bRz8TDywbVodOWNWh29kzwM08uE8vAw+DQgyTNQPYykzqyWQ\/egJJkgzcNQSSnOw0g6JCcxDPNOIIRhjlViSnVEi4th6I6QY8Fr166FpBgGj1Po5ut4GCxrDEO97YymQCiPCWGgt8HjEqqHYRrDwFOvcW\/f6AgD\/4a3gqnTNm4gXVyHcFYDq0IY5p1ACMMcq9SU3M3mUxJaDUDXfeHChVGAD1c+MDC5fPly2L59O7S3txc6AP4HvyaPVkniLuWlaYRuRSRtiZRPj9RVEiQx7FzcM+APduMUYu7cubB79+6I8DZu3Ajcw6A4RmtrazTdwHdAz549q10lidtnoSOMM2fOwL59+6LTshwTXcwE60ackdgOHTqkJWYhjNSmXUgghGGOlaSsAIGkmEnWKYlKShWIFWUVwjBHUAjDHCtJmREBdb9JOc8RqDs98cJbm4QhOz2zGVUIIxtekloQqGkE\/h8UjxezSoBkUgAAAABJRU5ErkJggg==","height":161,"width":268}}
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
