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
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"num","rows":1,"type":"double","value":[["0","0.053515965322714","0.045258443518672"]]}}
%---
%[output:2214b3d6]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000","-1.555535358343578","0.604922562764271"]]}}
%---
%[output:2a03b5f5]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAMgAAAB4CAYAAAC3kr3rAAAAAXNSR0IArs4c6QAAEaxJREFUeF7tXW1sFVUafgvVUEyRli3W0tYi60fQQELWqGwiJC7B70TXReoPBRENS2Ujstb6kYKCfAR+oLgofiDrBwiryeoPgw2sJLsEEyCCslsV2bbLVhShaxukq126eU77Xk+nM3fm3jtzz7l33klIuffOnDnnOeeZ933PnPO8Bb29vb0khyAgCLgiUCAEkZEhCHgjIASR0SEIJEFACCLDQxAQgsgYEATSQ0AsSHq4yVUxQUAIEpOOlmamh4AQJD3c5KqYICAEiUlHSzPTQ0AIkh5uclVMEBCCxKSjpZnpISAESQ83uSomCFhJkLfeeosaGhoGdMHy5cvpjjvuiEm3SDNtQcAqguzZs4fuvPNOciMDk+bNN9+kq666yhb8AtXjiy++oNmzZ1N7e3vi\/JtvvplWrFhBRUVFvmWcPn2aHnnkEaqtrQ3UdsZRL\/j++++n+vp6SrUs38rl+QnWEOTkyZP03nvv0d13350U8k2bNvmeY1ufgSAPP\/wwrVq1ii666KKUB2mqgxoEWblyJb388stUWlqauF9FRYUiiRzBEbCGIMGrnHtn+hFEf+Lzkx6txCB\/4YUXaMqUKarR+A3WU3dB9fMZGSdB8D3q8PTTT9PSpUsVUWGNJk6cqCwTHkw42Krh\/\/w9vuvs7KRHH32U9u\/fT7t376a2tjaaOXMmXXDBBcri8wHrfvHFF9NDDz1E11xzDT311FMEUq5evVq15cCBA67egc09ahVBdFcEbhYOxCIAeePGjerpm4uHm4vFbqROnsrKSjUwJ0+erAYfW4ETJ04oFw0DDcfmzZuVe8YD2el6uREEFhoDd+HChfTSSy8pglRVVakyxowZQ\/y7TgTcA4N60aJFCn8QZMuWLQnLhPuwuwvStrS00Ny5c2nOnDnqe1grtAHnwZo1NTUpggV1LW3oa2sIorsReAoBZDylEJgD\/FwDVu9cLwvCROABj3iEBxpf74wbWltbB01gOK1IUIJgEDPusCJ4EK1fv14RCHUD9l7E4djJaf2YIKg3WzsQB59xrt5WGwjgVwdrCIKOWLJkCTU2Niq\/GU8euBYA2fmbX6Ns+91JENSPiYA2pkoQHnBe7QzqYuF6uEi6a8QWxo8gbL3wFxbh3XffHWBBhCAhj8I4EYSf2hiEkyZNSgTwQV0sdnn08\/Up8GRB+oIFCxIzYrq75nSl2GJ7fa+7dxzLwAKJBQmZGFxcvhMk2TRvFEG6HjwDY7dpXnZlQQKc39XVNSh4dwvSOYbgyQIQAy7xxx9\/rMheV1enXCpxsUIkCwgCkNFZbgeeUjxtGeJtpSgfBNzctTiBZk0MEifQbW+rcyVDLr6cDQtjawgiFiSsLpVywkTAGoLojeIZHn7ri884ZC1WmF0vZQVBwDqCuE3p5vo0b5COkHPsRMA6guhToGwx8E4EC\/1y6Q2snd0ttUoVAesIggY44xGZwUq1W+X8sBCwkiBhNU7KEQQyRcAaguTzcvdMO0muN4eANQQBBPm6Ycpc98qdM0XAKoJwY2TLbabdKteHhYCVBAmrcVKOIJApAkKQTBGU6\/MaASFIXnevNC5TBIQgmSIo1+c1AlYSRH9RiO2fO3fuVEomubonPa9HUJ43zjqC6EtNsLWUFT1ybS9zno+b2DTPOoLoCxNffPFFRRDsfNP3q8emd6ShxhGwjiBuFmTXrl2yWNH4UIlnBawjCLpBFivGczDa2GorCRI2ULoogohgh41ufpdnlCBuS0rOnDlDQ4YMGYR6ukvedeEzFAr5zTVr1ijtLTkEAT8EjBAkyKLEm266iZ555hlV\/0y23OqqHFADdKqkH\/lNgR9GOfn7sR8KB9X76\/\/2fce\/ff3DWfTH9pHG2nfDDTfQ8OHD6f3336dTp06FUo8jR46EUg4XknWC+C1r51msSy+9lObNm6fqmcmWW13uEmWx9i3vVixd+JdQAbWpsPs6Nw2ozvk9x9Tnip6v1d\/z\/3eMKvq\/009sLyynr4aW075hE2nDiORq+5m29\/dle5Ro9jnnnJNpUer6a6+9lsIkSdYJ4odC2Ftu\/Qhy4YUXhgqoX\/ty4feOrUuo53gL\/Xi8hboPfZiocmFZDRVdNpXwt2RGY8ZNgXL9d999p0Ttzj333IzLQwFh96cxgiST+SkvL6cRI0bQ559\/rkBLN\/7AtX4uVtiAhtLLFhbS9eGr1PNNK53++4eDSFM8dRYVXTaFhl02NaWaC0F84PKS94H0P96cr127NiXA+WRdLHrUqFEJWUz8fskllyixZg7SQZAdO3akdZ\/YX7T3bertOEp05COiL\/f0wVFSSfSLX1PBtN\/5wgNFecQe4mK5QOUl7\/PYY48plfDm5mYqKPgpgA5qRdhF27dvn8ppAYLcc889dPDgQVWLCRMm0CuvvDKAIGH6rL6jIo9PgDt2+tCuAVYG7hgsjJtLJhYkyWDggaynBeM8IBBAfv311xPpwvA9rEqQ3IQ49\/jx4wSCIBkMks9wIhq3WSxxsaJhLGKYnm9aFGGUe3a8RcUu1X\/4Z+KGQhAf7N3emK9bt06lCOP8FCgi6CwWXCvkMMTs1+OPP54giDMjk162uFjREGRQqR1HqXfv20T413GUClYdUanfxMVKgn\/QZe1BFMZhkZYtW6aWxcOtQroxtiB+BBEXK0sk6b8NZsk6ti2mpl89L7NYXtB7LWuHBYCK4ieffDLgUucSET3vH4K8e++9V1kOPdUy3LcHH3xQuWtInSAuVnaJkOxucLnafjuW\/nTFSpnmdQMqymXt+vIS3ZqgHs6lJnCx5DCDQBRv0tGSMD0CY+9BvJa1YwZr6NCh9OSTTyZmmoLGINzNOkGwC1FfrBjnXBdmaJDbdzVGEA6+9axSI0eOpI6OjgHTuwwv5\/Dm7Kq5DbvUPlcQMEoQP9dLVtzmyjDK33pmnSB+maTwpruwsJAOHTo0APWgLwrzt6ukZSYQyDpB9EZKJikTXS73TAUBYwRJlkkK1mLp0qViQVLpSTk3EgSMEcRrWTtmsfAuo6GhIdDSkkhQkUIFgX4EjBHEbRYLlmP16tVqJ2FjY6Nsi5VhahwBowTxan0mW2yNIyoVyCsEsk6QIFtu9XcjjLbMYuXVuMuZxmSdIEAmiGhDtt54z9\/8D6OdVV1a5Hr\/+uk1RuslN+9DwAhBGHw32R8sU8cy6AMHDgzqIyw+xCaoVEWsk+lijb7reeNj4czwUZ51ODP8ZynXb8j336prhnx\/ov\/vt+r\/w5r\/nHJZUV4QxVqsMNdhGSdIshgEs1n19fXqFI5J0tmK66eLlY\/7QTZ89B+FW3tXD7V39qj\/f6X9X8e9YkQhnV9cSPhbUdwnC3TfldmRApL9IGk8fpK9H1mwYIGa4Uplr7qINiTvhJXbW6jt5Gn618luauvopraT3QMuqC4dpj7\/ctxIYncQ39VeUZ5G7w68RHYUpgFhKltxgxQvsj9BUPI+BwTC8bfDHervX7\/ss05uB5OpuqSPVDiq+gkGcjnjKiFImn0TdCtukOL9CJJTyopQDNGPUu1zyZi+X0oqAymKBMEu1XPYtcN1unuHz3Dx1Pf9Lh+XLcJxSVAOuuXWWYQe3OvTv3p5vDwewf4TTzxBhw8fVsvooWoCdUUWgMg10QZsV+UDu\/L4gMgbDggl6N\/r2EE0AUfh6BoqGt+nYRWGAFyqROLzxYIkQS5ZJqnRo0erLbL6wUSASom+KxCKJViasmLFChWf1NTU0C233JLQ4C0rK6MZM2YoZRMcCPy3bt1K48aNU5\/zMUh3hZ01rPAjtKw6\/v2TlhVfwBZq3JVZsUQSpPtYD84apWeSgi5WZ2cnLV68mPbv36\/kflpbW1VJrKerF8suFKSC6urqFAFgHXilMDJUYeqYp\/+uvvpqeuCBB3LWgqT7tA5yHVsnqCe6WSJYoDCtj1iQFC0IMknpW24hPYrv5s6d65mCDZYBVmPatGkJJRO8J2GNrdtuu43eeecdZWFwOMWrc83FCjLQoziHyeOm2cv3YwKdVVZDZfM3+lZDCOIDkVsw\/txzz9Frr72mSAF3avbs2cqFcltqou8nce5DT4UgIj3qO5YDndDbtNbbfVMBTyURJhb6JxROHdxJ3xSPpfPqNom6uxvCPMBBBl5\/hSUm2HeOvBGwBLAgkPPp7u4m\/U06Ww52u5hsbi6Wn7JioN6XkzJC4K6Kvunh887+kcrP7pvR2vHzOaHnB0G5Yb5NN7bURBd6Q6wB1wruEHSxEIckE2fAgEds4ZQiZdLoQToy5LKIHMCTDFMZjfPYXWyMIE5dLI4j\/NI96+uquLd4ShekY0uEnBO8VEVkf2I3rkNrsDGCsAVBqjUEa\/Pnz1ezTZ999pmr7I8sdw+tz6WgFBAwRhDUkZ\/seNrffvvtdOutt1JXV5ciSLaWuzuxAnG3bdumiAo507Fjx6YAp5yabwgYJYgXmCbzpH\/wwQdK\/BoTBG+88QbNmjUraTyUbwNC2jMQAaMEQVAN90o\/3FypIOrufh3rnAbG+fqSFbZYGzZsoOuvv56qqqrUhAHiGxGw80M3f383RhB9wDrfmE+aNCnx\/gPQZyo7ykrwKIs3XOE7ntHCC0lOkYC\/uB+WqAhB8nfgB22ZUYLwjBUG6Pbt22nv3r306aef0uWXX64GZxhPbhARS1muu+46tXwFyXn0N+14w464g6eCoeg4fvx49XYedcB7FiQUlSOeCBgjCAYlFhfyG3Mkvjl27Bj19vaqntDzE+JzprNYemJPJgjvWtRfMsK1gptVUlKiiHTjjTfGc2RIq\/vGYS+PSAOAYCk6vzFHnLFo0aK09pwHqXpQggTJgxjkfnJOfiBglCDZhNCNILt371aLGHUXK1VBiGy2Qe6VfQSMEsRN1SRTV8oLQidBvIJ0yT+S\/UFo8x2NEcS5uDBqkJwE0ad505UTirrOUr55BIwSxG\/dlXl4pAZxR8AYQfgJjr9uOwXj3jHSfjsQyDpB\/DJMRRWD2AG31CLXEMg6QXINIKlvvBEwQhBe+oGttJkuI4l390nro0Yg6wRhuZ\/a2lq1I9C5dTbqBkv5gkAqCGSdIE7tXVgTrMOCZI8cgoBtCFhBkCD70G0DTuoTDwSEIPHoZ2llmggYIYhbijWuv0zzptmTclkkCGSdIJG0QgoVBCJCQAgSEbA8WweBZv3Q5YgiunXkxfLECjwBp5Src5bSWRl9H1AYG+KibqwQJCKEdfX6fFpKow9wrHxOlSCAO5dmLoUgBgjCusFtbW00c+ZM0vfg6zEYyyJhtfHUqVOpuLhY7cDEk9spsYrP+vYBtlQog4UxIOOqv5jVRTMgWoGD9+bj\/xj8\/L6KYUJ5UNsH6d0eAroFwf10UQ793uvWraPp06ennJA1ou7yLFYIEhHibi4WD\/6mpibasmVLIgcK74fXtwLrRMB1GKwgihdBIMXqNrjRPN6pCTkjJhe+Z81izrkCyddnn32WGhsblXC4m0yrLvvq5UaibKeumVMulhOz2m5dhSARE2Ty5MmDViuzBcFuxqNHjw5QcEF1QAgIdq9fv16RCL66U+jbaUEgMtHQ0DAo3gFx3MS7YQX0TMJ8IQ9kfIYFYPlWfNb1lEFmPwvC25fdtJRhiZzlR9QVGRUrBMkIPu+Lk8UgToK4PamdWmBBCOI24L2y\/HoRhAcuWuYUCE+HIF6WQggS0cDLlWKDEoQFtxGLwN1g8ugZs3QXC6mwOTBmFXu4XhjM7EpVVlYmzkGGLjcL4nSxcD9IIuFaqM00NzfTmjVrBkkvublYupXUY5BkmcHExcqVkRxRPYMSBDNB+upmvyBdD8b14D1ZkO6VH0UP6pcvX55wBfXERE549Cd\/MhcLcklwEaFcwwe3DW1etmwZQerJdpEMcbEiIkjYxSYbtGHeKxvvMWSaN8wek7IUAtkgCO\/2rK6uVnJIXgovmcQPzjjG9u4VC2J7D0n9jCIgBDEKv9zcdgSEILb3kNTPKAJCEKPwy81tR0AIYnsPSf2MIiAEMQq\/3Nx2BIQgtveQ1M8oAkIQo\/DLzW1HQAhiew9J\/YwiIAQxCr\/c3HYE\/g+Sz4xrZyzNjwAAAABJRU5ErkJggg==","height":120,"width":200}}
%---
%[output:7a92d0b3]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAMgAAAB4CAYAAAC3kr3rAAAAAXNSR0IArs4c6QAAEuhJREFUeF7tXWmoVkUYngsWRf9asNQ0I4UguBUYZUh72SIE0eaPQISsoEINV4zW26ZFBUlotNBi9Kuu0YIiREkLXIuCXCJvZRq2\/GmlxRvPifdrvvlmzrxzznvOd5YZkM\/7fbO888z7zMw7yzsDY2NjYyol\/P7772rZsmVqeHg4LVry24svvqhOP\/305P8vv\/yyWr58uTXNfffdp66++urkt59++knNnz9fffLJJ878Kd+0PPWyzYx27dql5s2bp\/bu3dslo6vA999\/X82dO1dNmDBBPf3002ratGlJVDOf6dOnp8o+Z84cdf\/99ytgmFZHHY8HHnhAPfnkkz2imbKYETg4LliwQC1dujRJSuWQjIceemhXuw0ODqqnnnpK4Xtqfz09tQXFO\/zww5N6mrriahcXxrY2sclq+87UD5KXg6mtPpBlQIIgrsbTFYoqbgPMBBagoyFvu+22HqU28\/Qpjk2xicR5CYJ8THmg7FOmTLESzNZQNjxIeUg+XQl9vZStE7FhVARBIJspu05+XfYiCGJ2Ejr5fZhmJoivQeLvEYEmI+AdQZpc+Vi3iIAPgUgQH0Lx91YjEAnS6uaPlfchkIsgMFCHhobUmjVrFFYxYogINA2BzAShFQMAguXASJCmqUasD2uZ1waTvmQWsgQZIY8I1A2B4BGEyIH1bYQNGzawRpDjjz++bthEeSuOwLhx49T48eMVPvWwZcsWMcmDCaKXjM2VEIJ8+eWXYoLrGe3evVtNnTo1KG9uGl881++2783vQv8OqqAnsq9eruTcdGnx8mAGuW688Ua1ffv2HhGPPvpoNTIyonbs2CEGVSMIIoZGxTLiKmPFxC5MnGuvvVZ99913XfmDFNddd526+OKLk+8xU5HsiCNBCmvO\/BlHgqiEECCGGUCMRx55ROFTD7UmyObNm\/NrjSWHPXv2qEmTJgXlzU3ji+f63fa9+V3o30EV9ET21cuVnJsuLR4Hsx9++EEtWbKkS4wjjzxSXXbZZckoobe3nt95550XRxBJRalyXm0cQWwjhmu0sLVdrUcQyblhlRVbSrY2EQTEWLhwYZeNEUIMwlycIBs2bBijexuco+P6keIDBw6ogYGBRDbce6A7HmUwW0oJq5xPWwgCYnz88cedpshCjMIIcvnll4\/RTjhn2RbHSzA3fPDBB5MltbjMa19ijsu87qV3wuaVV15RTzzxRIcYsDEWL16sjjvuuI7xHYpjISMI9fx0cWnmzJnO0QAbhbhsE3q8RFpwvefP0tNy0\/ji5VnTb+s+yEcffdRjgF9yySXq7LPPVjNmzOga1PtOkJ07d47RlVJIhlFk69atyZSJrmHqEuP30dHRztVN7hSlSIJwZahbPB8561YfyGvuZWA69dJLL4lVRVrPBn788ccx\/aBh2jTLdT\/dda2yyPVpMUQrnFGTCPLBBx8k99Up5LEz0pqsrwQhA33y5MmdEYbuZN988819M9KzKBI3jS9enGLZ1ZVwsa1OzZo1S9111109CTnTKSRKm5r2lSAu5nKMe2nBK9zxi4nmI6dYQQVl9Mwzz6hnn3228FGjyJlK0BTLhSMMd3gg0V3kmHFBkKJ20gtq375ny9217rugmgDYAUfQd8GxOnXrrbeqiRMnFi6q+E56qJFuqyGXIHGjMEw\/6jiC2PY0JI1wH4LSM5UBbBRyl3nJ3li9enXHQRwE9q18IY604DpQWRSJm8YXL9og\/7XEvn37El9gFDBqPP744z2HCW02BKWppA1y5plnjtHUiGNLYA9k06ZN6osvvkh20eGYEZ9png0jQf5TgdB9Dx85fb1p2u9Z87alsy3dLlq0qGdPI40ILuJwSKPHke6IkxEk7agJCIFALisxnbr++uvVL7\/80sEfy8QYRvX9FJsNEqdYYSqdVYnDSske29wJx9It9OTkk0\/OnmnOlOIE8bke1eWlfRCc2SLC4HeTRLY6Sgsep1j5NCkr+ZAOG8jmHQ0cEcFRdN8oUbspVghBaB8E5ND923KOn0SCNGOKdeWVVypaqYKyuzb80giYx27zTVWl9SzoRqF+UFGfTsVVrHy9uSt11l6+CGn6saeRpR6RIFlQq2maKhBk\/\/79PSckLr300mTfq4qh1gQpaqMwy4YaN40vHuf6KClS6BVbX9l5FNSXt+vKKxwknHTSSd6i8165dWGG79NwFN8oDLFBqjrFytLTctP44uWZT4cu+3q1MiCCS27flVcfHtFInz8\/WcGqkpEeoBe1ispVRolKYT\/r3HPP7cqqqBO3EvK68ujrFKuqy7xFAt7PvMsgyGuvvZa4z9FDHYlB8osTxLdRaCrI22+\/rW644YbODjo9cYgzOLfccotTn6QF1wvKokjcNL54dZxiYRp155139ngn5BDDh0fjplhZ7qTD9aO+K+47ZgLQIkH6vw8C\/wG2B0I5xOAoPrfTytOpoIw0201az4IOK0I4zqagbRiRFryfU5+yyub21mnyvPvuu2rVqlU9UUCKU045JXHbaXonLKt+RZQjrWcDocfd4530IprVnmcWgmD69Pzzz6vXX3\/dmmnIaFFeTeVKEidIE+6kZ1EkbhpfvDzTBYllXhACU6dXX33VqWUgxd13361OOOGEThxfvVyZcdM15qhJCEHinfRetSmTICADFkm2bdvW5WjNpsy+kYKr6Gbe3HStJIirV+HcI4lXbnnTCOxg\/\/HHH+qtt95Se\/fuTQxSTsAFpfPPP1+deuqpCv9vaxDfSQ8ZQVygx8OKbnVEr4\/eHJ8\/\/\/yz2rhxo\/r666+TBPjOfO+Co9jIj+5eNMnA5tTdF0fcBgk10m0Ccggyffp0tXPnTusDKLoSkTLp5dB3rs9vvvkmub1my8eW31FHHaXg3e\/EE09U3377rfrrr7+Sz19\/\/VV9\/\/336p9\/\/lG\/\/fZbkh9680MOOaQjjq7QWZTb18D67xgJ4Ob\/mmuuUVOmTBFdbeJOlVo\/xSrrTvo555wTohuNjvv3338n9cMn\/uGEwp9\/\/tn5rtGVL6FykjdXB7LcSceIQb556VFPzmZhCdjEIiICoggE30lH6bhiq+\/IRnKItknMrEIIBN0orJDcUZSIQCkIRIKUAnMspK4IRILUteWi3KUgEAlSCsyxkLoiEAlS15aLcpeCQCRIKTDHQuqKQGUIgoOQ8M63YsWKVBemdQVaWm79tWHkzXnlS1qGuuVnvpDG2Z6oBEHIazwAT3tjpG4NUqS82IvCa7DwzO\/yul9k+XXMW7\/LxL34J0YQNNLQ0JBas2aN0t88BJC0206g6sxFT7hu3To1e\/ZsdccddyTPS6c5wa5jw6TJnBU3PU\/qGeEvV\/c20zSsqD4SmEEn4XDd9VgtlSVCEBrukan5PLR5kNF1sNHlc6upjYx6SeCGfNIUpmn45cVMn2aVMsXSR4fBwcEugoS4CWobQaRwa5PtJoWZ3jmZPt7MDiXXCEICw0BEwNVPfQQJ8QbfJoJI4damkUMKMyIAdd4zZ85MfZ05F0F0ttluFYa4Km0TQSRwQx54QXblypXJex1tCll1bWRkJIGJFjbw0KjP5o0E6bNmZWnstWvXqvXr16vh4eEu6Tlz6j5XV6T4LJhhdRSXz5YtW9bBjYNXZQgiglwNM8na2G1a6TObtUzMIkH6TKoyG7vPVRUrvkzMCiVIiJEuhl7NMrI1dsQtvRHLxKxQgoQs89ZMr8XEtTV2xC2cIEVh1kMQ8zotRJ0zZ07XjqNtZ\/yrr77qWeZFWooLrxxVCOQwAbLg\/\/BogietDxw4kPw9bty45DOG+iJAxjfH246vll0E4awNu3bGr7rqKrVly5aenXQiCZwkv\/POOx0fUboLHzypdfDBByfeAkdHRxP\/UfhOD2W62zFBI99T+Lzooos674Dj7zI9K\/oaM+T3Jrv90XHgrFSl4dZFENfclzLIM4xJO\/RKq5TpH2v79u1q69atic8rIlpWp22ucolEp512mrriiivU5MmTO366QhRXj5tVibOW14R00nrWRRDfzmwe41Fa8KIaUyfX559\/rj788MOOB8Q8juJAoDPOOENhpOV6Q4wECW9laT3rIggMxuXLl3dJpdsfITvjZtWkBc\/b03KVzxZPJxE8MMKh9I4dO7wOpW3NDbJceOGFat68eakPw4SrSnoKbv3NXLjpGuO8Wn\/lFgY6dmfpTgZNqQASjgXDLrBtz3OMoSIJIq08UvmBSPv27VPPPfdcEHlAmgsuuEDBG+XUqVOlxGlFPtJ65l3m1S\/jHHHEEZEgAmoG4mClDHdnsDDhCzQlw2Ob3OmZL8+m\/l46QcjugANluNbPM4Js3ry5qe2Su1549gDOqnEa+r333vPmh7hoC4Q2P3dgAiX+\/IE+xbK1ik4QDPvzK\/hOOndenMVu8eVd9DLvQQcd5B1pMKrgjXPYi9wRxlcvF0O56Rpng7iWcHXDnE5DTpgwQeGiCQXYLgj6d0020jlEsylI6JNrrgUC4J02NQNJcJ00LXAVPRrp9NC5tutNmys0euCeMym\/6c2dY6ADZOm5oXcO0oAIHCVGB3b77bc7H+Khp9gAB3d0qTN00nrWY6Sbx0gWLFjQMzKkOWFwgSsteJ0bkSs7hyCUF+3RpI0uvncLuXJVOZ60nnlXsaTAkBacM82RmGL4lLRoGyTLMi+RBcvLb7zxhhUGvLL12GOPBY8qPjyosMbZIFJEiCOIHJJcZfSVmPZ2OkaVVatWJa6amjAFk+6Ie0YQ02NfnGL51K+436UIok\/DQBbcZbcZ+U2YghVKEH1JFxfbzb8BNNfPVZmrWMWpaH9zliaIXhuahsHZnBlAlHvuuUcddthhtRtVCiWI7fKO7qIR3jNw6T0u8\/6vUlW0QTi01uVOm4Idc8wx6uGHH+4QhUvaRtogtv0M\/QQvnnKOG4Xd6tcEgpgrYa5RBUddsF\/GWThoHEFcl6Ukj5pIPs\/L6SXrHofbWxdRT4wqCxcutL5rjxEFI0sVQ2FTrDoTJIsicdP44jVpBLEpPIiCafajjz7a8\/NDDz2U+JqyrX7FEWTatA5gnN10aWbHfZB8\/beP+GbudAcGR\/DNMGvWLHXTTTd1ESUSJANB4mneMKXG\/Rv00FULOHkMl5341AOdMO7n6eJCT\/PW1UivmgJJyRPay0uVy80Ho8qiRYuSS2F66Od+ivRMpefKremhPS7z7k5dtWm6DWKSxXXCGJuPb775JosoeTBDAWmnogsliHl617VROHfuXBXqe0ha8GiDcPt5e7yso1NaOowomzZtSi59pY0otSUIKhWPmuRTPMnUWZVYUoYseYEon332mbr33nt7iLJ69Wo1ceLELNmy0kh3xI04zctCroaR6koQgpqcVsBOMQMeXcIjpNIHJCNBLIqeRZG4aXzx8kwX0ubStrm2JMd99XKVxU1nHmWhjUcz37POOit5vNVVX1t5fbNBIGRW37y+11WlmR1tkHx04So6x0i3SeLKH6eIMfUyl4gxkmCkmTFjRld2lSJIHt+8vvfNiyRIPlWpbuqsSlzdGv0vWdpRljzujaT1rJG+eeugIBwZm0wQ3U554YUX1MaNG3sgwWbkscceG2SnFEqQuvrmzaJI3DS+eNEGsVM99KgJRpRPP\/1UDQ0N9WRobjz2zQaJvnl7GzsSpBsTHx4UO5QgZKTTi72uk8Rwgbt\/\/\/4uW0Uvq9ARJPrm5Ux8yovDVcbyJCq3JLoebO7QQwoct8dmtRkKJYit+tE3b7lKkXd1rn\/SFlsyznvhBIct4Jbr4OBgYquUThDJC1NFnebNcuqVm8YXz\/W77Xvzu9C\/JVXQVy9XWdx0afHyYIblYXiN3LZtW4+IOEWMHXy86yIVvDvpkr55pYSO+UQE8JYkwvjx45N3JfWApwClQocgRfvmlRI45hMRKBOBrhHE9Lsr6Zu3zErFsiICUgiU5ptXSuCYT0SgTAS8NkiZwsSyIgJVQyASpGotEuWpFAKRIJVqjihM1RCIBKlai0R5KoVAJEilmiMKUzUEKkMQLCkvXrxYrVixQk3TfG5VDbCqyGP6DsAVVnjkj8GNAO31DQ8PJ5HI8UgaZpUgCJ33gqC+i1dRAf5DAAdLcacbpNDPy\/ludrYZP5xWHx0dTZ4U1N1Z4fEgVxAjSNpdkrQ3DdETrlu3Ts2ePTu5m4xLMm0aQbLipjco9Yzwyt4GgkhgBp3EmS4cn6cj9jaSiBCEhnsUAJ9IOiO5D+7oz023hSASuAFz30W3Jo0aeTHTp1mlTLH00QFHjnWCuM532Vycto0gUri1yXaTwgwdhv7uTdqom2sEIYFhICKYbktdQtjmf20iiBRubRo5pDCj0ZTjoARxcxFEH7ptz7e5lN72XEKbCCKBG\/KAP9yVK1emzqGbNL2iumTVtZGRkSQLWthYsmSJ1+aNBOmzBmVp7LVr16r169crWq6kKnDm1H2urkjxWTDD6iieksDtw74s82YVui0GuUszIm7hnCkTs38BBmEh8iaFpBkAAAAASUVORK5CYII=","height":120,"width":200}}
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
