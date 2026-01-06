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
ld1_load_trafo = 100e-6; % for f > 400Hz 
% ld1_load_trafo = 400e-6; % for f <= 80Hz output
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
    frequency_set = 400;
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
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"num","rows":1,"type":"double","value":[["0","0.053515965322714","0.045258443518672"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000","-1.555535358343578","0.604922562764271"]]}}
%---
%[output:2a03b5f5]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANYAAACBCAYAAAC8aERhAAAAAXNSR0IArs4c6QAAE3dJREFUeF7tXXuMFtUVPxCMuyDILsVaWGCBShpsRJBEpS1saxCIj\/ogCBgLRJFswNWA4eEjQHysGvhDod0KPkp9gJDYFBob3CCQVMSkEjeGivJecXkUdoVFQLt1m9\/dPV\/vDjPfzHx7Z+bufGcSsux+d+7jd+7vO+eee+85nZqbm5tJHkFAEDCKQCchllE8pTJBQCEgxJKJIAhEgIAQKwJQpUpBQIglc0AQiAABIVYEoEqVgoAQS+aAIBABAkKsCECVKgUBIZbMAUEgAgSEWBGAKlUKAkIsmQOCQAQICLEiAFWqFASEWDIHBIEIEBBiRQBq0Cr37t1LM2bMoLq6uswrt912Gz333HNUWFjoW8358+dp4cKFNGXKFLrhhht8y+\/cuZOmTp3aptysWbNowYIFFLYu38byvICVxHrnnXdo0aJFbURTWVlJ99xzT6rEBWLNnz+fXnjhBbrqqqtCT+6wZACxnn\/+eXr11VepuLg4016fPn0UueQxh4BVxOJvVDcSMdnefvvtQN\/O5iCKriY\/YukahjULegNyvPzyyzRmzBjVOXwGjaV\/IenleQROYuHv6MOzzz5LTz\/9tCI4tN+wYcOUJty0aZN6lbUo\/s9\/x9\/OnDlDjz32GO3atYt27NhBtbW1NHnyZBowYEAbzQiZDRkyhObNm0ejR4+mp556ikDmZcuWqbHU1NRQ2r44rSFWfX29EuS0adOyzuQ1a9b4lomOCmZrdjMFeYLppCspKVETetSoUWrSstY5deqUMiUxQfGsXbtWmZFMAKeJ6EYs4I4JP3fuXHrllVcUsfr166fq6Nu3L\/HnOoHQBsjw6KOP0uuvv66ItW7duowmRDtsmoLshw4dopkzZ9L999+v\/g7tiDGgHLRndXW1ImZQE9isFKKpzRpiRTM8u2v10lhMICYK1ls8QXlEznXR4cOHLzKfnVorKLEw+dnMxJcdtEtVVZUiHvoGk9yLcLw2dGpbJhb6zdoVhMPvKKuP1W6pBeudVcTSv8HxzY0Hay0IFt+MWIek6XESC2NjAsHMC0ssnqheGAU1BfE+nBy6CccazY9YrC3xExpo48aNbTSWECvmGawvxGGPw2yAvY5vR0y2tJkKvL5xc15g8o4YMSLj2AhqCrJpppfXHT7ZnBcVFRUZD6NuVjpNPpaD1991M5TXavhiFI0VM6G4OZgWS5cupcWLFyuPFWxwfGvDbHB+llAXjTfr526PwnkRxN3OX2wgD8o3NjZe5NRwc17wGomdKCAUviA\/\/fRT9SUxZ84cZfqJKWh8KnlXmI\/EihFe4025mZXGG+nAFVqzxhJi2T+LnPuLadr6MI2+VcSC2QDzw+2Bvc4bm6ZBkPoEAdMIWEMs0wOT+gSBJBGwhlgwBUVjJTkVpG2TCFhDLH1QvJfD59fwO560nRU0KUipyy4ErCOWm2s9re52u6aC9MYkAtYRizeKeYcfg8WeFq5WpOksmUkhSl32IWAdsQCRc70lHkH7Jo70KDsCVhJLhCYIdHQErCFWPl4b6eiTR\/rvjYA1xEIX8+2io0zM9CJgFbEY5ny5mp\/eaSUjs5JYIhZBoKMjIMTq6BKU\/luJgBDLSrFIpzo6AkKsji5B6b+VCFhJLH2DGAFMPvjgAxWZKW0xL6ycEdIpIwhYRyz9SBOCo3DsvLRF8TEiPanEWgSsI5Z+4Hb16tWKWIjBoMfDsBZN6Zgg0IqAdcRy01jbt29v1yFcPShL2iKuyky2EwHriAWYTB7C1QNLom6EU16+fLmKBCWPIBAVAlYSy+Rg9WhCiNLqzM6x47eXmGwu9rqOf9clVJvHvv9\/+ePft4z9z3U9Q9URd+Fu3brRhAkT6Ny5c\/Tee+8Zb\/7AgQPG67SGWFFdzdfDFwM9joHOt5GX\/LKYHq542DiwcVXY9O9DgZv6j0vZphOHqLbhAvVpOuZaT5fepdTlilK6pHcp1XX5Ma3q0RJb\/6v6C4HaRd0mntOnT9PCnx5WCSBMP4MGDSLT5LKGWDpYuV7N188Y8h2uL7\/8klasWEEfffSRauKaa65RwSeZWMVzt5qWk9X19ekRXMM92HkL\/eS\/LYS77rsaov0724ytrsuV9Ldu49Tfjl4\/23XcfboHb88LuG+\/\/ZYOHjpIY3s3qMwnpp+bbrop\/cTK9Wo+p6Ph9RPfOh47diw9\/vjjKnTapZdeSg888IBKI3PzzTcr+UTxbWVa8DbW17B+qerW+X9towu7t6n\/Q7sVXl2mfhZNWmys29BWSFt0+eWXi8bKFVVTV\/PZBASxkILmtddeE2LlKpQA78Ekbdy65iKidS+brt5uD9GEWAEEEKSICa8gNFZpaanKJ6WbgjfeeCPdfvvtGVNQNFYQiYQvk41ohVePoYKrywJXKsQKDFVLwajuXelrND\/nBYi1ZcuWkD2X4qER2L+Tmg98TFT9YsurRSVEI++mTmP9HUdYYyFPF7yDssbKgny2m8Iw2ZC2s7m5mTp16pSpxRlQRs\/UoSfEhqaCW33r1q0q9SeyHj755JO0b98+VR+cF\/AMcsZB0VihKWLkBazReH2GNRlMRi9zUTRWAMhziW0BDQSTzi8zPEiFMu+++y598sknKlld586dadKkSSqEGh4EAV2\/fj0NHjxY\/S7ECiC0CIs4TUY3kgmxIhJAkICd+rEl7kZZWZk6FQ8vIO9TYI310EMPicaKSFbtqZZJ1rBhifIs9v\/DQVWdECsEqtk2hJ2pUYPmYoJ5iOTf5eXl9MQTTyhvIExBZ9JrPRiorLFCCC3Gos1Yi+FfUQmdq\/i7rLHCYO+2EYyFKvIu7d+\/v80ay+\/wLNz0OAF\/\/Phx2rZtm3Ktr1y5ki677LKsG8RiCoaRWLxlocFO\/H6G2ifb\/rNZ9H3JcNnH8hNBto1g5MN96aWX6MUXWz1ILpU5nRfY+L333nvp7NmzmdIgFfLyYtNYNoj9JGLv53VLfk31+2row19VCrH8xMQbwTD79Kwi0DZNTU2EBNXIR4wnyBpLL4N35s2bp0zBgwcPKnLJBrGfROz9XNZYIWXjXGf17NmTGhoaVC1wt8Ojx4\/uUndrhomFzzZu3OhpCrptEIfsthSPGYGoT7djOKk6hOsW2+KOO+6gN954Q2mrMHem2DRctmxZJiv7jh076K677lLud2QqweM83R7zHJHm8gSBxE63O28KDx8+XHn0CgoK6OTJk\/TZZ5+1EYHfBvH8+fMznkAEnWFPIjyEVVVVao3ldh8rT+Qsw4wZgcSIFUVsCz4fiCsh8DhCY+mEA7ZygzjmGZanzSVGrGyxLUaOHJlxXLBcguTI0k1Lvby+eQxXvt8JjjydCzJsgwgkRiz29ukJvUEGrJHgxYOnUAhgUNJSVawIJEqsbN69sM6LWFGTxgQBHwRiJ1aQ2BZwrcPFytfnRYqCQEdDIHZi6QB5HWnC3ZuampqsXsGOBrT0N78QSIxYuca2yC\/xyGg7KgKJEcsrtgXitdfV1V20jwWAnafeOyro0u\/0I5AYsby8gtjIra6uJhBMP0OI8rjsiCsg2Q7npl9kMsKOgECixArqFWSzMcip97Cgz177edhXjJbvX1zoWt+CcaVG25HK4kUgdmL5Xc1nExHlcGYQj36K4s0338xoMhNQXfG7P5qopl11\/NC1l+f7P3T9Uc51dz53Ur3b+dyp1p8n1f8L9vw15zqTeDHqQ7imD+ACo9iJhUazBZPhyE36BURsHOPSIoLD6Ld\/TQg5zRcdn9\/cEn66tv68+omw0Aj5XOsSHrp\/cYEq84vBPYm1qC1aU66NhJzpUYU\/c3YjWxofuZpPtOnzs3T0TJOC7Z9fX6CjjU1U1\/o7\/sZhqa\/rW0AIGf3g9fEmUZDwZyGJ5VUch2kRUlh\/gpwVdKvPL41PmjWWIXEQa74P9zXQP\/Z\/06ZaaLr+RQXUDz+LCykKLScay4AkdSLs2rVLeQIPHz6sas7lJIZfGh8hVm5Cgzn54f5vMmYlSOdmZrKJqZMPLYYhoBArNxm1eUvfOEamEGRznDlzZs6pUv0i4Vb9pjtNvHuigZ6HrwIhvrye9sQ6D9+T6N7Q13mc+sdrnaf3ggmJv4GUtV99ReUDj0rMi1xFBa8gAsmATAhdNmPGDLVhnKsp6Eesv4wvpGHXXptrd9v\/Xv2R7HU0+HzuycySlk+KW38W9W0J6wyPVYCwzu0fWPtrWPVxi9m599hpOnHiBE288piEmA4Cq9vVfATYROa+rl27Et8ExjUSRLXF72GffDQFG7f9iZpOtJjPnJiOk85xyh0dR9acUaTgCSsvt\/JiCoZA0e2iI17n4Jq4Rm\/iEeeFP4p6ritkeNSzRHJGx8KhLdlBkjBRhVj+MsyUaM\/VfLfMjQg8o2tAPaoTJ1pA4zAxFy1alOmHuNs9hAYTtP5IS4YQ\/L\/h67YZHWFWwswcdH3k5qW420MQyy2YDLKCwJbWs4xwlbzGwrpLj1vBmRsRhQlrM+TEQv4rTuI9ZMiQTIxB1OWMeSFewRBCazUtz+\/epkxNaDaYmLp5GYWGE40VTkZtNAxehfmH9RWIFTQ2BTsnEDRmzpw5mSv9fNdrzJgxKtOIV5QmIVZIoXkUB8myEQ6v5ZpKVYgVUkY8+WGecewLEApaxhkLA8RwizPIkZmQEpWj38LJwecL\/eIKiikYUmg5FldJDvDAtIQnVPd2tnorPRPRNRyh5srRtHvoNPr5dHO5jXkoqUruDVPwmWeeUWl2sBGMayIgAWILIhm37rzwyjai30DWnRRhiRXFIcwc51\/evQYz8vzu7WrceqJwBoI9lutKyyW5d5DZ4XReYG0ErYOMIffddx898sgjav8KDy444v\/6RUc9hiDKsOOCozuFMQWD9FfKJIdA1KfblSI9cMDoABM53a6+nVo11q233qrOBc6ePVtFsv3iiy9UOlPsW3mFmAapsHZyhkdjsoVxXhhFUyoTBFoRSIxYaJ9Pnc+aNYsmTpxId955JzU2Nmb1CuKY09SpU9sIkF3rICuvzVAn30CWgJ0y3+NGIFFixT1YaU8QiAuBRIll8nqIKcCg9TZs2KBMUiSzGzhwoKmqpZ48QiAxYpm+HmJKZu+\/\/z716tVLnUt86623aPr06W08lKbakXrSjUCixIIHEKGkTVwP8ROT0x2P8vrRKN6QXrVqFU2YMIH69eunXP9Yv4XJ0+XXD\/k8PxBIjFimr4dkExcnpUMZPiWPv\/HxJhCbD\/\/iJ8jUu3dvIVZ+cCCSUSZGLIwGYaRNXQ\/xQgeaavXq1TR+\/HhasmSJCkijbyDjjCFIzqc2du\/eTUOHDlVnDqGxcGu5R48ekYAvlaYXgUSJFSes0FA4T6gTi4OC6pvLMAFhDhYVFSkC3nLLLXF2U9pKCQKJEsstSlOuN4X95BGUWJKTyw9J+TwIAokRy3kEKUhn21PGjVhIpeo0BXO5pdyefsm76UQgUWKxVzAOr5uTWF7OC1M3l9M5XWRUQRFIjFjs7sbPXMKaBR0gl3MSi9vHbWLJYhIWTSnvh0DsxAqS0dHr7pXfYORzQcAWBGInli0Dl34IAlEikAixeMMWd6z0oC9RDlTqFgTiRCB2YnEQmSlTpqj7VM4Li3EOXtoSBKJCIHZiOXMPQ3tt3rxZBYKRRxBICwJWEMstzkVaAJZx5CcCQqz8lLuMOmIEEiGWHtrMOb6ojjRFjKNULwi0QSB2Ygn+gkA+ICDEygcpyxhjR0CIFTvk0mA+ICDEilnKvI+3adOmNi3r4dpi7pKx5njrBGtoJKUYNWpU5hyoc\/\/S2ah+ozyOQ9nGBu1RkRAraoQd9etZVuI4fBzX8HRi4IZAWGKhn2na0xRixTXzWtvJRixO5FBbW0uTJ0+mESNGuKaK5QCkOJVfVlZG3bt3V3m\/oCmcIbbxu36hlDUj6kAEYjzI86wfLdPD0iHIDh6OCYL\/c4ok\/VIo6kMSdnxZuI1R11hoj9tGfXrbK1eupHHjxuWUvTNmUWZtTogVszTcTEHeYqiurqZ169aplEN4nNlTEEpAJxDewyQHwbyIhVDcbqRA\/ZyCFuHemJT4O6c94lxkSFKxYsUKFVHLmZ+M4dPDfnuZuyjrTM\/kDBeOLwE8HV2bC7ESIpa+\/uAusMbCreYjR45ktBV\/DiKVl5dTVVWVIh\/WIs5USE6NhaA4egZL1AWt5ZU3DFqHY4Ho0DABWMNx+G78rmeOwQ1sP43Fms4tBj80HzSaXn\/MIjLSnBDLCIzBKwliCjKxnNkn0YozpVEQYrkRxSvpuRexeMKjD86EFLkQy0szCbGCzyUpqSEQlFic4AFrLZhFrM30zJW6KVhRUZFxGHC2FZiIIAGbfCUlJZkyAwYMcM106TQFObIV3kUq2j179tDy5csvCmLqZgp6eQVBXi9zT0xBoUtOCAQlFjxr+r01\/aiXm\/NCd1LoTo1szguvFLK6s6OysjKz3tET\/TkHr2uabKYgwsnBlEVMSd3E5VS2nIywowf1EVMwJ3rY81K2yW6yl3HsQ4m73aTEpK52IRAHsThOSf\/+\/VW4OK9IVu1ZHznXae0CxYKXRWNZIATpQvoQEGKlT6YyIgsQEGJZIATpQvoQEGKlT6YyIgsQEGJZIATpQvoQEGKlT6YyIgsQEGJZIATpQvoQEGKlT6YyIgsQEGJZIATpQvoQEGKlT6YyIgsQ+B\/+nhxucgSzIwAAAABJRU5ErkJggg==","height":129,"width":214}}
%---
%[output:7a92d0b3]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANYAAACBCAYAAAC8aERhAAAAAXNSR0IArs4c6QAAFOJJREFUeF7tXWmoVkUYnlsGSv8sWszKG3lbIG5YtmgRUYFEEuiPSpMIIYvS3JeC8E96K5fA4GZqVlDeCqK8UCKVtKDRcs1fmi3aQgt0o6ioaLnxHJnP+c43c+Y9s90z55sDce1878y888z7nNneeadjaGhoiBk+f\/zxB1u+fDnr7+\/X5jB16lTW09PDvvnmG3b77bezb7\/9ls2ZM4ctW7YsS\/v888+zFStWZP9+7rnn2GWXXdb0bvXq1eymm25iP\/30E5s9ezbbt29fQ7arq6vxTszzoYceYhs3bmS87FGjRrXo+emnnzb04eUWVea9995jM2bMYGPGjGFbt25l48ePz8Tz+eAd5MT68LR4x3Xavn17Vu98flz37u5utmXLFjY4ONjQk2Mh4i\/WO68\/z6uoXmL5Ktx4G3GdgCdvf1lbcrnRo0c3tRvXX6WPCmOZvExX6juxzYowzdcb9dE9HaGIJRqtSKK8giLoeRLJKiMjYV6uiDC+iCWSXaY3JxbIIX4oivBQESRPynweFBwpHyQbYsk+wqp28UGsvM3x+lIwrSSxVI0uGjQ3BBnQ+QbBVxC93OLFi7NeT0yTz1NncLKeBj2lix4L+eT1wUfjzDPPlPZ4sgaW4SH2etBT7BV0X1HZB02GkY8eC7rldVf1XD6Ilf+4iKMYHabBiaVryPR7QqBdEbAaCrYraKneCQEdAolYOoTS7wkBAwQSsQxAS0kSAjoEErF0CKXfEwIGCDghFla\/Vq1axdauXcsoa\/wGeqYkCYGoELAmFl\/GRK2xkZmIFVX7J2U9IWBFLHH9n7qfctZZZ3mqSso2IaBH4LjjjmMnnXQSGzFiRJPwrl279IlLSBgTi5MKm3x4+vr6SD0WiPXFF1+UUDGJyhA4dOgQ6+zsTOAQEPj+++8ZNr0\/\/vjjFulTTjmF7d27lx04cICQE13EmFhiEdiZrgqxbAyOmpYip5KhvpfJie8oOtDNQC5pUwY1LUWOihmvBZeHZ87MmTOlZLrjjjvY1Vdfnf3m42OfiCXATmlkiFPkTI0hbxyiVSRiNXNEhXFvby974YUXmoTRM02YMIHNmjWL4d9i2toQ64033rD9mLZ9epwSGDt2bNvjIALw5JNPsnfffbcJkxNPPJEtXbqU4a\/queaaa5xPT2rXY7WLpVF6zXbB4pFHHmGvvvpqSw+1fv36rHfSPbXpsXwuXtgYHDUtRS4NBWmLKzZYPv7449mZPfEBkRYuXMgmTpzYeK2bryZi6T49bfQ7xSDrCgeGwZgr5QlF7aHyuHgllnhWh3KOSTzf8t9\/\/7GOjo5MX5wSxklf1eOjEnU1oKJ6tSOxsGy+YMEChr\/8QQ9lSiiehw+bzOZY+eVyyvI53JgwKXz44YfZwMBAZZbb24Vk7UYsEErch3JBKK\/EGhwcHMLR8JtvvrnR0\/BTu5MmTVL2PtggxqZbWTcmH18HkUw2BkdNS5FLcyw3c6xt27axJ554oqmHwgnyCy+8ULntkcd+WOZYBw8eHOI9Dw+Mglqg19q9e3c2tJMFYcHvhw8fbgSDofYUiVhygytrDBRyU9tEJWdTBjWtSg7DvVtuuaVJtenTp7N77rmncFECP5bF0odNduzZs2dI1vMUDQdV0Zl00XdQaR+VsDWgGNNTDTfGuvkc9snw8GGTRsTiCxdnnHFGo0fjgVPmzp2bFi8CWHPdiIUe6pdffmF33nmn04UJSlNUhlgqZSmLHqiET88LG48EalqKnEqG+l4mJ76j6EAxqiIZmzKoaSE3cuTIbBHsxx9\/bKhz7rnnZotjeKiY8cR5eR2WXjwvTIaCqsbAggbCkomBLPOyPr4OtgYUY\/q69FhPPfUUe\/rpp6WLE6HaxYdNdpguXsgqnYgVyhRojsDhtClfUn5xAsvnK1euZOecc075zCxTeCGWyXI7n0+tWbMmCwXNH91KYlq8sLQAIXnMPZZscQLL6sP1eCEW3yDesGFDYwhHmSthJfH1119nn332WeZ1gRDw+KuLf+6jEmKD2BgcNS1FLu1jtW4r4EAh\/Pj4I\/Prk5HLN5Y+bLLh3a5zaQKR8PBLDDDsw2Gx3377rYEF4l3gyyPuh8U0x6IQBvWhyrn+AovlhtDBVRkyVyRs8MKWKN7nPoglto1XYpUxAr6PBZ9CTjSkz5NPlqePSpTRvS6yrozeJx6yI\/EgEvec8Fl2mbx92KTReSy+jwWAxDkWxc3JRyXSULCMGdFlbcj7yiuvsEcffbRp2IcPL\/Y+y7aXjx5LzNOHTRoRS3TAFYd9VVgVtDEGalqKnG9joOhAp5Bc0qQMmSsSeinV4gSlDN9YJmLZWkqN0lMMMmR1VUc6qjbsCzU9qV2PFdKYhrOsqhBLNY+yPSMVEtva9FjJpak1CExZNxyqy5CNgerKgBuSGI8PAVtwyPWiiy5SuiHl9dGVAfkoXZpMrkpNixf6s0a+5wUheixZGargl9dffz1bsmRJgzdU\/ShyvrGsTI+Vlttt+gE3aSkG6aakI7nI5lB4P2XKFHbbbbeR9qNc6uMyL6\/E0m0Q5yuyc+fOzMWfe1zgLx64q8ybN09Zbx+VcAlyLHmFIta+ffvY\/PnzW2BBD8WDX8aCmUpPHzZpFfPirrvuagp0qHNnQsV8VEIEzMbgqGkpcr6HLxQdTA1eNdzDsvm0adPYlVdeqe2hqPpR5Hxj6cMmO0yccNFglM3gUEubiVimFDqaDmR65pln2GuvvdaSmUngFgphUBBFLkpimR4bqWrMC3sTiyMHikHqagIy4SzUjh07pGSCP1\/s8ycdBr5GUUZH81PMC0pz+ZUxJdaePXuyCwNkV9pAY5PeyW9N\/efuZShocoK4yjEvTA2OOiyhyvkevlDqiR7p7bffZiCTikhFZKKUoTJ7alqKnG8sK0MsFZiUc1w+KpHmWEcQwOUAuBOqiEScSNjEvfXWWwsXIShGn4glR8BoKKgCk+qE69Pzwv\/AYXhK4IFW0Pvs378\/21f6+eefScrAI+L8889nN9xwQ+F1NqTMaijkJZiM6eKFDF8qsXzeNhJDu4MU\/IDfP\/\/8w9555x324YcfZr0NHvwuxicvUyfki\/\/WrVvHfvjhB+2yeJm86yrrYxRltNxuE\/Oiq6srG\/f7er7++mt2+umnS7OXnVbFxva\/\/\/7L\/v77b3bw4EF2\/PHHM8whBwcHs7+Idffnn39mv3Njx\/8jZBd\/RBKYEqIMHqgHCHn55ZdnocGPOeYYLwRKQ8EyrdIs29ggNol5gR6Kx27nl33rNon5va\/mKtc3JciCB3\/x319\/\/ZWRmr+rb82Hv2auR1HGMS8ABU6Ebty4sYGKjlTDD1\/SICEQBgGj81hhVEulJATiRSARK962S5pXGIFErAo3TlItXgQSseJtu6R5hRFIxKpw4yTV4kUgESvetkuaVxiBRKwKN05SLV4EKkcseDssWrSI3XfffYUx4OOF3L\/m\/PQBjtXjoVxh61+rOEvIH5Gi7tVWiljcVQpNUHR5XZxNFE5rbNyPGzcuC0Wmcj8Lp03cJYkHesucmndOLDTkqlWr2Nq1axluHxEf7vbE34nsx1d206ZNWdQfXECGmHVFt5bE3Vw07U2xFHPnX1zcQC\/G2adpUB8pF1jCfhEqu6enh40aNaoQHKfE4kMQlMh9CHnpec93lSe8Ki58fZqYVhMXWKKkIoOiaRK\/lC2W4nAw+FBQ7I26u7ubiFUmDmEi1pFAPTNmzMgs2gbLNF91hyXaQhWoVvbpcdJjcUPAJBlPX19fE7HKRM5td2K5wjL1VEdJ5cIuYde8g5g0aVI2fy16nBBLLEB2PL\/MtT\/tTiwXWCIPRF+6\/\/77tXOB+Ad6tBqY2uXAwEBWAF8IWrp0KWn+n4hFa5dhkTIxht7eXrZ582bW39\/fpDN1bjAsFQ1QqAmWWJkeO3YsW758eQNPKo6VI1YAjKMpwtQY2n01VdbAobFMxKowzUIbQ4WhsFYtNJZBiFVm8cIawRplIDOGhKVZA4fGMgixyiy3m8FWz1QyY0hYmrV1aCyDEAtQ5IPNUEKlmUFYn1SqAKgJy\/JtHBrLYMQSycVhoa6wlIexHimKIgsXuYfVo\/ZuaxEaS+fE0sFRpfBnPNwYdOb\/\/v3337OwY\/wiPfE3Xd3S7\/Ei4Dz82bx584ZkDpr50GaAbOrUqU0OiCZfzaKoozBuBMr88ssv2eeff84QVhlx9XiEWN5sPChmiOCYMlPhgT\/x99prr2UTJkxgHR0dWdBMMcqtTzOzCabpU68Y8\/YSCbezs3MoPySjuG5QnWrzQPuohFhG3uBEEsLwsaq2d+\/e7CZ28TcfJOXhnq+66qqGZzki9U6cOLHF\/vJ6y4gjvgtBLJsyqGkpclHeNiIjls7Z0GZlKjSxynxBZQ3ICYeh4fvvv8\/eeuutzGcMFxLYkJH3epMnT2bTp0\/PiC4SLhHraMtFSazJkycPrVmzpumsjs6B02YvxTexyhDJtSwn2ieffMJefvnlLHvdlTqqoeZpp53GFi5cqIzLTvnSu65fXfPzYZMdMmJhBWXFihVNOIrzqzJOtaGHgjE0\/q5du9j27dtLkY73cA888AA777zzSHf3xoBFFXQMRiwsXMCJkx+P50M\/gIDTkxi2yLx8KXtTPipRNMcq03DUXoAiV3b48sEHH2S3pOCE6ldffUUiHcg2ZswYtmTJknTbiNDQZeerPmxS2mPJjFGMnXDCCSdYEStdPEejO79s7tdff2UvvvgiO3DggDYhLplDMJ6TTz5ZK5sEjiDg5eI52VBQBjifV+E+JiwvV7XHagdjwVwOK5s7d+7U9mzo1dALpkeNwLD2WCKxrrvuOjZ79my2bNmypkUPShQbH5VoR6ORbStgCK+7xHvmzJns4osv9jJ0jLUdfNhky1BQtZQuLljww18Y34Nc\/EHD4hHfhV68oMx\/VAZATUuRKzvHKjsvKNKBr05SiLZ+\/XolySj1rAOWQYgFoPJOnry3QvgsThpTR1AflajD4oVLYuWNHUTDfO3BBx9U7r1hyJgnWSKWeR+sXLzIuyvNmTOnpSdy7dJkXo32S2lq9JTeTEayOiPs42Mf3AnXRyXq3Oi2Qy0dNiBa0ZDx7rvvZldccUWt52Q+bLJ2xDL9ksMAqWkpcsM5x9KRSfa7ridDLwYPnWOPPZZEMgpGVMx9Y+mFWCrv9nxg\/TQUNDFXf2mohmuiAUjGezKZP2TdhopeiFXkhIs9K8RTE5faeaDCqnq3mxhSjGl8EkvEA8SCJ4hspRcEQ4x+rA7H\/AQjluy0pbhHhYDwiLVWxeX2mBu4jO6hiJUnGQKB7tixo0XVSy65hC1YsIA0TCxTzxCyXogl87yQ7UeJHu1dXV2V3SC2MThqWoqc73kBRQdbo1SVgV4Mh1JnzZrVUgQfJmI\/tLOzU6sCpR6+sQxCLNUhx3ZwaaI0MnXCrbUoAwFRP6quBsU0klDKAMnQU+XnYi6HiT6IJeKSiGVjJTVLSzH6kFUGsXbv3s02bNjQUixu57zgggsqO0xMxCJYio3BUdNS5Hx8ZavYY+WbhPdciKOSf0499VS2bt26JoJVAcvaEMvnsRGcFYMvo8lDTUuRU8lQ38vkxHcUHUwwENPYlIG0I0eOzG7m4MdfeN442oLTEfhLKYOKGc8\/L6\/DMtixkZgXL2yNKZb0lC99leqyePFi9tFHHzWpVJX9sCA9FmqelturZJJyXWIjFmqBYaJsuX64CRaMWHlvdtUGMa7z5KHTKMfyAa6PSoimZ2Nw1LQUuXadY\/G2KMIIBEOATFyMl38ee+wxhhPqPMYHfveNpQ+bVHq3x+rSRDF6VX9ETUuR820MFB1s+12bMihpQTAczORTD1Hf+fPns0svvTQjmG8svRBrSIylbNsShPQ+KkEotnYiFMONqdKcYLL9sKLDmC7q6MMma+fd7gLoGPKoG7E45kUbzr4IlohFsHgbg6Ompcj5Hr5QdCDAVShiUwY1rUqOstBBxVgmJ74LSqzhuBTB1hCKJrqUvG2NgbKI4soYqLpS6m0755Slp+qnkwPBEPrtpZdeainm3nvvzYIZiQsdMhuoDLFivhTBxpBiSqszyJjqQtEVBPvuu++ysNv5B6csnn32WUo2UplgPVbMlyIYoxtZwnYjVr554DLlaqEjGLFivhTBxuCoaSly1CEfN5i8vG74QtHB9lthUwY1LUVOJYOw3G+++ab0fBgO6d54443K5fphmWPFfCkCpaFs5xSUMhKxDgU7j8XDu82dO7elaTH\/wvAxfyfZsBAr5ksRbL\/SsaSnkDuWurjSk9+mKRsmogyV61SwoaCsoulSBFfN7yYfile4m5LizAUe9fj49Pb2tlQAXvXTpk1jZ599duZh78W7nep5EcsJYpsvOTUtRS4NBcMNBfPMEbFHL4braeHTKotrj14M7\/fv3+\/0C0L2vHB5KYLTGqTMEgJEBEaMGJFJ4ooj\/m+eFJcBunxaiOX7UgSXyqe8EgJVRUDaY\/m8FKGqQCS9EgIuEVAOBX1diuBS+ZRXQqCqCJDnWFWtQNIrIVBFBBKxqtgqSafoEUjEir4JUwWqiEAiVhVbJekUPQKJWNE3YapAFRGoHLGwEb1o0SKGsMTjx4+vImaV1ykfCGj16tXZdUzpKY8A39ft7+\/PEvOoZLqcKkUs7o8Ipbdu3ZqIpWs9xe9woh43blxGJtHHE6dt01MOAZz0OHz4cHY\/mHiV1ejRowszck6sorNcRZeB4yu7adMmNmXKFLZy5cosNHG791imWIotzr+48PhuZ2K5wBL2u23bNtbT08NwR1zR45RYfAiCArds2cJEVlNvgAQAiOvd7sRygSXaQXdotdz3O05pWyzF4WDwoaDYG3V3dzcRS+V\/KIsRn4jFsiEHogzjscEyzVfdYYm20IWsED87TnosbgiYJOPp6+trIpZKIdmYtd2J5QrL1FMdJZULu4RdU4IscXI5IZbIVNmFCiqyyOK9tzuxXGCJPHD5AGKj6+YCcQ7uymttapcDAwNZYXwhiDpNScQq30bBUpgYA07Mbt68mfHlYa4sdW4QrHKBCzLBEivTuGsNF9kP+3K7aQXafQVQZmcJS3fsC41l5Xosd1DGn1NoY4gfMXUNQmMZhFhlFi\/q3Lhl6yYzhoRlWRSPyIfG8n8V7o71uhmregAAAABJRU5ErkJggg==","height":129,"width":214}}
%---
%[output:8429c053]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"6","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000001000000000"],["-6.316546816697190","-0.000125663706144"]]}}
%---
%[output:09179d2f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"2","name":"Aresd_nom","rows":2,"type":"double","value":[["0.010000000000000","0.000001000000000"],["-6.316546816697191","0.009874336293856"]]}}
%---
%[output:154cc966]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:6187332d]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:3efa50f2]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"    -6.316546816697190e+02"}}
%---
%[output:3a7c206f]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.987433629385641"}}
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
