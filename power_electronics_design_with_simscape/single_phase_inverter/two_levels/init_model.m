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

rpi_enable = 1; % use RPI otherwise DQ PI
system_identification_enable = 0;
use_current_controller_from_ccaller_mod1 = 1;
use_phase_shift_filter_from_ccaller_mod1 = 1;
%[text] ### Voltage application
application400 = 1;
application690 = 0;
application480 = 0;
%[text] ### PWM and sampling time and data length storage
fPWM = 20e3;
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
tc = ts_inv/50;

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
    Vdc_nom = 690*1.35; % DClink voltage reference
elseif (application480 == 1)
    Vdc_nom = 480*1.35; % DClink voltage reference
else
    Vdc_nom = 400*1.35; % DClink voltage reference
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
%[text] #### Phase shift filter for Q component derivation at 50Hz and 80Hz
if system_identification_enable
    frequency_set = 300;
else
    frequency_set = 50;
end
omega_set = 2*pi*frequency_set;

a = 1 + 2*pi*frequency_set*ts_inv;
b = 1 - 2*pi*frequency_set*ts_inv;
phase_shift_filter_gain = -(1-0.05/750*(frequency_set-50));
phase_shit_filter_d = phase_shift_filter_gain * (1-a*z_inv^-1)/(1-b*z_inv^-1);
flt_dq = 2/(s/omega_set + 1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d, options); grid on 
[num, den] = tfdata(flt_dq_d,'v') %[output:3b365bc2] %[output:1da0ea6e]
figure; bode(phase_shit_filter_d,flt_dq_d, options); grid on %[output:4f9bd5ad]

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
figure('Color', 'w'); % Create white background figure %[output:29bd839f]
subplot 211; %[output:29bd839f]
semilogx(freq_Hz, mag_dB, 'Color', [0.25 0.25 0.25], ... %[output:29bd839f]
    'LineWidth', 2, 'LineStyle', '-'); %[output:29bd839f]
title('Bode Phase Shift Filter'); %[output:29bd839f]
ylabel('A/dB'); %[output:29bd839f]
grid on; %[output:29bd839f]
set(gca, 'FontSize', 12); % Adjust font size for axes %[output:29bd839f]
set(gca, 'xlim', [freq_Hz(1) freq_Hz(end)]); %[output:29bd839f]
% 4. Plot Phase (Bottom Subplot)
subplot 212; %[output:29bd839f]
semilogx(freq_Hz, phase, ... %[output:29bd839f]
    'Color', [0.25 0.25 0.25],'LineWidth', 2, 'LineStyle', '-'); %[output:29bd839f]
xlabel('f/Hz'); %[output:29bd839f]
ylabel('Phase (deg)'); %[output:29bd839f]
grid on; %[output:29bd839f]
set(gca, 'FontSize', 12); %[output:29bd839f]
set(gca, 'xlim', [freq_Hz(1) freq_Hz(end)]); %[output:29bd839f]
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('bode_phase_shit_filter','-depsc'); %[output:29bd839f]
%[text] ### Load Transformer Parameters
m1_load_trafo = 50;
m2_load_trafo = 1;
m12_load_trafo = m1_load_trafo/m2_load_trafo;

ftr_nom_load_trafo = 50;
I0rms_load_trafo = 5;
V1rms_load_trafo = 330;
I1rms_load_trafo = 600;
V2rms_load_trafo = V1rms_load_trafo/m12_load_trafo %[output:189fc40f]
I2rms_load_trafo = I1rms_load_trafo*m12_load_trafo %[output:103fb8d1]
lm_load_trafo = V1rms_load_trafo/I0rms_load_trafo/2/pi/ftr_nom_load_trafo;
rfe_load_trafo = 2e3;
rd1_load_trafo = 1e-3;
if frequency_set < 100
    ld1_load_trafo = 400e-6; % for f <= 80Hz output
else
    ld1_load_trafo = 100e-6; % for f > 400Hz 
end
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
Vac_FS = V1rms_load_trafo*sqrt(2) %[output:278a1c18]
Iac_FS = I1rms_load_trafo*sqrt(2) %[output:511b73df]
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
%[text] #### Resonant PI
kp_rpi = 0.25;
ki_rpi = 45;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta*omega_set] %[output:74d80938]
Aresd_nom = eye(2) + Ares_nom*ts_inv %[output:9c1cedb7]
a11d = 1 %[output:987b375e]
a12d = ts_inv %[output:81e6790f]
a21d = -omega_set^2*ts_inv %[output:6c239720]
a22d = 1 -2*delta*omega_set*ts_inv %[output:4d4af98a]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:900457b4]

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
omega_fht0 = 2*pi*50;
delta_fht0 = 0.05;
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:8c43ce75]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:261d01fa]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:516bf83b]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:73bc0081]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = 2*pi*frequency_set;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:11e11d95]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:33f191ae]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:5e9b3e14]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:2df27274]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:6036ed4b]

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
figure;  %[output:3a89dea5]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:3a89dea5]
xlabel('state of charge [p.u.]'); %[output:3a89dea5]
ylabel('open circuit voltage [V]'); %[output:3a89dea5]
title('open circuit voltage(state of charge)'); %[output:3a89dea5]
grid on %[output:3a89dea5]

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
heatsink_liquid_2kW; %[output:9437bd2f] %[output:86f2bfce] %[output:534048a4]
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
Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
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
%   data: {"layout":"onright","rightPanelPercent":35.5}
%---
%[output:3b365bc2]
%   data: {"dataType":"matrix","outputData":{"columns":3,"exponent":"-3","name":"num","rows":1,"type":"double","value":[["0","0.244171410261656","0.241627792504512"]]}}
%---
%[output:1da0ea6e]
%   data: {"dataType":"matrix","outputData":{"columns":3,"name":"den","rows":1,"type":"double","value":[["1.000000000000000","-1.968829526703427","0.969072426304811"]]}}
%---
%[output:4f9bd5ad]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAewAAAEoCAYAAACaU8LCAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ9wV1e94L99jzqkI1VSW2uaNgEhOu06aMWBxR0TH69WaWF38D0h6A7NIMPWZXDGMJCks0NRGwIjs1usWxEi0q2ktYprM2+cWmlh55nhPdYqa3nW4ksTGmNtF7ClY6qysvO96flxc\/P7c+\/v9\/3d372\/3+c30yn55dxzz\/mck\/O533PPveeKS5cuXRI+EIAABCAAAQgkmsAVCDvR7UPhIAABCEAAAh4BhE1HgAAEIAABCKSAAMJOQSNRRAhAAAIQgADCpg9AAAIQgAAEUkAAYaegkSgiBCAAAQhAAGHTByAAAQhAAAIpIICwU9BIFBECEIAABCCAsOkDEIAABCAAgRQQQNgpaCSKCAEIQAACEEDY9AEIQAACEIBACggg7BQ0EkWEAAQgAAEIIGz6AAQgAAEIQCAFBBB2ChqJIlYXgdOnT0tHR4eMj49nKrZ8+XLp6+uTurq6gpWdmJiQrq4uaW9vl8WLFxdMf\/z4cVmzZs2UdBs2bJCtW7dK1LwKnowEEIBA2Qgg7LKhJWMIZCegwt6yZYvs2rVL5s+fH1maUSWrwt65c6f09\/dLfX195nwNDQ2etPlAAALpIICw09FOlLKKCBQStj8idpGwVl+lu3fvXmltbfVo6O80wn700Uelu7s7811QwkFha0ItQ29vr3z5y1\/2Lhw0Wl+wYIEXuQ8ODnp5uahf\/+2+1+9ee+016enpkWeeeUaGhobkzJkzsnr1amlqapoSyR86dEhaWlqks7NTPvKRj8iXvvQl0YuEr3zlK15dTp48KTt27JBVq1ZVUetSFQiUjwDCLh9bcoZAVgLZpsSduPwyb2xs9ES5ZMkST4YuSj579qw3pa7i08\/AwIA3ne7EGpwqzybsc+fOeSL9whe+IPv37\/eEfeONN3p53HDDDeJ+7xeznkMlu3nzZjlw4IAn7EceeSQTuet53BS9XkSMjIzI+vXrZd26dd73eiGhddB0Gu0\/+eSTnvDD3gqgO0Gg1gkg7FrvAdQ\/dgK5ImwnZidgvZ\/txOcKGbzvPDo6momuXRp\/VK7fhRW2StVNt2uUrdHwgw8+6Aldy6aRcC6Ru3vvwdkBJ2wtt5sNUJHrz5rWX9fYG4ITQiBlBBB2yhqM4qafQFDYWiMnZp3ujipsJ8BcZMJOievxujjNP5XtIvBCwnbRvf5fI+bHH398SoSNsNPfb6lB5Qkg7Mq3ASWoMQL5Iuxbb701syAt7JS4m6L2p\/ffF8636GzTpk2ZFef+6fXg1Lebus71vX863t0L1widCLvGOjfVLSsBhF1WvGQOgekECj3WVY5FZ2Ee69IFYnq\/WaWs6S9cuDBtMVq2RWfuHrRb\/Kai1nx+\/vOfexcfGzdu9KbAmRLnrwECpRFA2KXx42gI1BSBbNPrNQWAykKgggQQdgXhc2oIpIGA\/7ExLa\/e4w7zwpY01I0yQiBNBBB2mlqLskIAAhCAQM0SQNg12\/RUHAIQgAAE0kQAYaeptSgrBCAAAQjULAGEXbNNT8UhAAEIQCBNBBB2mlqLskIAAhCAQM0SQNg12\/RUHAIQgAAE0kQAYaeptSgrBCAAAQjULAGEXbNNT8UhAAEIQCBNBBB2mlqLskIAAhCAQM0SQNg12\/RUHAIQgAAE0kQAYaeptSgrBCAAAQjULAGEXbNNT8UhAAEIQCBNBBB2mlqLskIAAhCAQM0SQNg12\/RUHAIQgAAE0kQAYaeptSgrBCAAAQjULAGEXbNNT8UhAAEIQCBNBBB2mlqLskIAAhCAQM0SQNg12\/RUHAIQgAAE0kQAYaeptSgrBCAAAQjULAGEXbNNT8UhAAEIQCBNBGpe2HPnzk1Te1FWCEAAAoki0NzcLAsXLpTR0VE5ceJEosoWLMzw8HCiy1eocAh77lxJeyMWauQ4f68XQPC0JQ5TeNoSsM3t2WeflYceekja2tpk2bJltpkb55b2vyWEjWBM\/yTS\/gdhCsMoM5gagXwzG3ja8kTYtjzz5YawEbZpb3vhhRdkzpw5pnnWemYwte0B8LTlqcIeHByUBQsWEGHbop2WG8JG2KZdjMHQFKeXGUxtmcLTlifCtuVJhJ2HANNjtp2NwdCWJ8KGpz0B2xwRti1PhI2wY+tRCNseNUxtmcLTlifCtuWJsBF2bD2KwdAeNUxtmcLTlifCtuVZ88I+fvy4rFmzxuOwY8cOWbVqVYbJ0L+\/UhpvaIyPeAxnuvjKSAxnKf8pZlzbPO0kM667\/N2Vvt\/70864rsk7blbbXeUvZAxnQDC2kOFpyxNh2\/KsaWGfO3dOOjs7paenx+PQ29sru3fvlvr6eu9n7mHbdjarwfDC0W\/JxZdHpxXOfzHyZ9+FycWXJy9SCl2sTBV7s6j03XezP7XNFoZRblZMjYqT+mzgaduECNuWZ00LW6PrnTt3Sn9\/v9TV1UlXV5e0t7fL4sWLEXYZ+lkaBsPz39mekbtKX2WfTfRO5BrVO7Fr9K7\/nnlLWxnoZc8yDUxjg2FwIngaQPRlgbBteSZW2I8++qh0d3dPKV9wyrpUFCrsgYEB6evr87JSYS9ZsiQzLf6O\/\/RY5hTXX399qaer2uPHX7tYtXXzV6zh4kvej+\/6fy\/JB\/94cvLfF1+Shou\/kw\/+8efTGIzPuF5++9fXy09nLvB+942r19YEJyoJAT+BL9T\/RObNmyeLFi1KJJilS5dmypXmNzFW5Dlsd085m5ydxA8dOpSJgkvpAYWEzZR4KXSnH1sr0csbp47KxKljHoCJfznq\/V+\/cx+NzjUyr7u5TUqdaq8VprY9MXdu8LQlTYRtyzNfbrELW+8p61tx1q7NH4kcPHiwYJowmJgSD0PJLk2tD4Y6ta5T7H6ZlyryWmdq1zsnc4KnLVGEbcszUcKOr2qTZ2LRWbzEGQyz8\/aLXCPyXBKvu6V12v1xmNr2YXja8kTYtjwTJ+zTp09LR0eHjI+Pe49Z6UfvZTc0NMiBAwdk\/vz5pgT8j3UFp9qZEjdFTfQSAadf4vpvXRnvn1Kvu6XNW8H++\/oWmfO37RFyJmk+Agjbtn8gbFueiRL2xMREZqV2S0uLrFu3TlavXu0tAtP710NDQ94CMV3RHccHYdtSZjAsnae7P54tEtdny7NF4aWftXZyoI\/atjXCtuWZKGHrFPX27dtl27Zt3rPQ+shVa2urt8As+Ls4MCBsW8oMhrY8NTfHVB9H855Pf\/P5c42+3cthSl3YZl\/q5OZIH7VtG4RtyxNh5yGAsG07G4OhLU+\/sF3Oue6Hq8DdNDoCz90O9FHbPoqwbXkibIQdW49iMLRHHYapdw\/86YPeI2ZuQZtf4LM+ujbzRjf7EqYrxzA801WjypYWYcfHvyKPdel965MnJ19KEfzoJuj6VjL36tByoyDCtiXMYGjLM1uEHeYMhQReyxE4fTRMDwqfBmGHZ1VqytiFXWqBrY9H2LZEGQxteRYr7GApcglc74HXmrzpo7Z9FGHb8kzclDgRdnwNHPeZGAztiZeDaT6BV\/sq9HLwtG\/19OSIsONrq4pG2PoY18jIiGzdutWrsf6sH\/\/2l+VGQYRtS5jB0JanVYSdr1T5FrFV42Nk9FHbPoqwbXkmKsJ2hcn2CBePdcXX8OU6E4OhPdm4mfoF7h4js3w3uj2haDnGzTNa6dKXGmHH12YVi7DdC1T8O2fpM9n69jNenBJfB7A+E4OhNdHKv\/vaTZ9rzc4\/dq9XwTQ\/A04fte2jCNuWZyIjbC2URtT++9lxrxDXMjAlbtvZGAxtecYxJR61xGm\/\/00fjdri+dMjbFueiRV2fNXMfSaEbdsKDIa2PJMo7GAN07YCnT5q20cRti3PRAk77u01C6FE2IUIRfs9g2E0XmFSp4mpynvi1FG5+PLolOnzJL2BLU08w\/SPSqdB2PG1QEXuYbvds3SnruCKcF0prjt3BXfVKhcShG1LlsHQlmcaIux8NU7i9Dl91LaPImxbnomKsP2FcXL2f5dN4uXEgbBt6TIY2vJMu7CTOH1OH7XtowjblmdihR1fNXOfCWHbtgKDoS3PahO2n06lVp\/TR237KMK25Ymw8xBA2LadjcHQlmc1CztK9G359jX6qG0fRdi2PBE2wo6tRzEY2qOuVaa669jEqWNT9gCfeUub1N3cJqXsPlarPO175mSOCLtcZKfnW5FFZ\/FVr\/CZiLALM4qSgsEwCq1waWEqYjl9Ds9w\/S5sKoQdllTp6SoqbP+LU\/bv3y9PPfWUrF27VubPn196zULmgLBDggqZjMEwJKgIyWCaHdb572yftv93mN3H4Bmh84VIirBDQDJKUjFh+19NqhuAtLa2elUaGBjg1aRGjVuJbBgM7anDND9T\/7vP\/a9OnXFdszd9Htw+FJ62fRRh2\/LMl1vFhO3f6GPfvn2esFtaWmT79u2ybds2qa+vj0zBPd\/tDnSPiPm\/Dz42RoQdGXPeAxgMbXlqbjCNxrTQs9+\/vapJ5syZEy1TUuckgLDj6xwVE3a2CPvYsWMlbf4R3K5TMeqFQWdnp\/T09HhUe3t7Zffu3ZkLAoRt29mQiy1PhF06z7S9OrX0GsebA8KOj3fFhO1karn5h+721dzcPOXtaRpd6\/f9\/f1SV1cnXV1d0t7eLosXL\/YoI2zbzoawbXkibFueKu8z3\/9vMvvts6ti5zFbOsXlhrCL41bMURUVdjEFznVMrp2\/nn\/++cx9cT1Whe3f0hNhW7YC07e2NCdz4yLIlqqfZ6Hpc32MjE9+Agg7vh4Su7CDYg1W1WqLTZ0eHxoakpUrV8rhw4e9hWy5hO3KcOTIkfjIV+mZxsbGpLGxsUprV5lqwdSWe06e58dEzo3JpeF\/Enny\/smTzm4Uefci7\/9X3PZ524JUSW56AaRj7bx582TRokWJrNXSpUsz5RoeHk5kGcMUKnZh+wsVvOesP+snuCFItoqcPn1aOjo6vHvey5cvn7ay3E2F33333fLggw8yJR6mNxikIRo0gBjIAqa2TMPyJPoOx50IOxwni1QVE7Z\/lbhbEZ7tu7CVDB6r9631s379ehadhYVokC7sYGhwqprJAqa2TV0sTyfwC0e\/5b3IZca1zaKPjjXc+7RtAVOWG8KOr8EqJmz\/KnEXUatkNWLW6WtdIBb14398yz+17v8+uG0n97CjUs6fvtjB0LYU1ZUbTG3b04Knk7f3\/6Pf8gqoAtcXt1i+99y25uXJDWGXh2u2XCsmbC1MroVixTyDXSwyhF0suezHWQyGtiVKf24wtW3DcvDMN31eynvPbWtentwQdnm4Jk7Y8VUz95kQtm0rlGMwtC1h+nKDqW2blZtnvveeB9+6ZluzyuSGsOPjXrEIO9dqcatV4mERIuywpMKlK\/dgGK4U1ZUKprbtGTfPao++EbZt\/8yXW8WEna1Qukq8qakp81KTODAgbFvKcQ+GtqVPZm4wtW2XSvJUeU+cOioXXx6d9uKWGdc1effA0\/ZB2PG1WKKEXcoq8WKRIexiyWU\/rpKDoW1NkpMbTG3bIkk8qyH6Rti2\/TM1Ebb\/NaJxLTxD2LadLUmDoW3NKpcbTG3ZJ5VnvnvfSV55jrBt+2cihZ3rHnZwN61yo0DYtoSTOhja1jLe3GBqyztNPP17fif1uW+Ebds\/Eyns+KqY\/0wI27Yl0jQY2ta8fLnB1JZtGnnme+670o+NIWzb\/plIYVu\/6axYZAi7WHLcw7Yllzu3NAomLjbFnKcaeDqBn3\/sXg9BJV\/agrCL6YXFHRP7ojP3hrPBwcGsJc72XvDiqhbuKIQdjlPYVNUwGIata1zpYGpLutp4BheuOXnHteocYdv2z9RE2PFV+\/KZELYt9WobDG3pFJcbTIvjluuoauaZTd51t7TJzFtay\/bIGMK27Z+JErabCt+0aZNs3rxZTp48OaV8vDglvsYvx5mqeTAsB68wecI0DKXwaWqFZ\/C+d7kib4Qdvu+VmjL2KfFSC2x9PBG2LdFaGQxtqeXPDaa2tGuRZ65pc4tXpSJs2\/6ZqAjbXxh9s1l3dzcRdnztXfYz1eJgWG6oMLUlDE8RfVzMv02ovmGtWHkjbNv+mUhhu+ewt27dGuurSIMwiLBtOxuDoS1PzQ2mtkzheZlncLX5zFva5Lr\/fMBbdR72g7DDkio9XcWmxCvxGtJsuBB26Z3InwODoS1PhA1PewLZcwxG3SpuFXihD8IuRMju9xUTtlZBp8T1s2rVKrsaRcwJYUcEViA5wrblibDhaU8gf44adTt5a6R9039\/Ie8BCDu+FqqYsNleM75GjvNMCNueNkxtmcIzHE8V98tf65A3Th2V2X9\/b8573Ag7HE+LVBUTtkXhLfIgwrageDkPBkNbnkTY8LQnEC1HFfb4vR\/17mtni7YRdjSepaSumLBzRdhamYaGBjlw4IDMnz+\/lLqFOhZhh8IUOhHCDo0qdEKYhkYVKiE8Q2GalkilrfJuuPfpKfe2EXZxPIs5qmLC1sLqPeyRkRHRleLuZ\/1\/U1OTDAwMyP33319MnSIdg7Aj4SqYmMGwIKLICWAaGVneA+BZPE+9t63vL5\/72KVMJgi7eJ5Rj6yYsPNt\/qFvQduzZw\/CjtqaCUjPYGjfCDC1ZQrP0ng6abtIG2GXxjPK0RUTttsERKe\/\/RH20NCQbNmyRR5++OHM97kqFIzQ\/dPs\/k1Ejh8\/LmvWrPGyCe63TYQdpbsUTstgWJhR1BQwjUosf3p4ls5Tp8cvvjzi3dNG2KXzDJtDxYStBQzex9b3iD\/wwAOya9cuWbJkSd7HvXbu3Cl79+6VDRs2ZMSu3zU3N8uKFSukq6tL2tvbpaWlRTo7O6Wnp8dj0tvbK7t375b6+nrvZ4QdtquES8dgGI5TlFQwjUKrcFp4FmYUJsWZz80R3Vjkd62dorsv6vi9bNmyMIdWLE3ax\/uKCrvYVtPIWu9zHzt2zMtCI\/Tgm9Nc9N3a2ioq8v7+fqmrq8uIfPHixQi72AbIcxyDoT1UmNoyhacNT3216Stf65A\/bPsFwrZBWjCXigrbRcn+UkbZrUuP9wvbRdK6ulyFrdPrK1eulMOHD0tfX5+XViNvf\/SuV1zuc+TIkYLASJCfwNjYmDQ2NoLJkABMDWGKCDzteF7aMldGPvQ5GfrTu2TevHmyaNEiu8wNc1q6dGkmt+HhYcOc482qYsLWiNgJ9plnnvEi5tHRUa\/2Yd98ZiXsNDdgvN2l8NmIXgozipoCplGJ5U8PTzueGmH\/4tlfyD\/d9EmmxO2w5syposLevn27bNu2TZ5\/\/nlvenv9+vXivnP3mN3iNL1HEnw+OyjsdevWedPjOt3NlHgMvSfLKRgM7bnD1JYpPO14umnxR2\/tRdh2WJMnbBWxPrqlkj579qx0dHTI+Pi41+h6v9kJOx8Dv7A1HYvOYugxBU7BYGjfBjC1ZQpPW57Df3+F\/HPT38nVf9PBojNbtNNyq1iErSU5efKkXHXVVd4bzfTRq82bN0d6w1lQ2P5V5\/7V4\/7Hug4dOjRlO8+0rxosc\/+InD2DYWRkBQ+AaUFEkRLAMxKugol1tfjQVQsRdkFSpSeoqLBLL37pOSDs0hn6c2AwtOWpucHUlik8bXnqM9m\/fOk1+eOKLxFh26JNVoRd5rqFyh5hh8IUOhGDYWhUoRPCNDSqUAnhGQpT6ES68Oxff\/q\/5NzffRVhh6ZWXMLYI+x8m35oFaLcwy6uylOPQtgWFC\/nwWBoy5MIG572BGxz1FeVjv3gfnnx0w8jbFu0lY+wg8IO3lMuc32nZY+wbYkjbFueCBue9gRsc3QrxZ\/r+AeEbYu28sL2lyDbq0nDrhC34oKwrUhO5oOwbXnCFJ72BGxz1HeJX7X9fYKwbblmyy32KfF8VdLV3O41omEe67LAg7AtKF7OA2Hb8kTY8LQnYJsjwrblmS+3igr79OnTmeevtZD+HbbiQoCwbUkjbFueCBue9gRsc0TYtjwTJWz\/NHjcC8yygUDYtp0NYdvyRNjwtCdgmyPCtuWZWGFnK1jcEkfYtp0NYdvyRNjwtCdgmyPCtuWZKGHHV7VwZ0LY4TiFTYWww5IKnw6m4VmFSQnPMJTCp1Fhsx92eF6lpKzoPexSCm51LMK2IjmZD4OhLU+YwtOegG2OCNuWJxF2HgII27azIWxbnggbnvYEbHNE2LY8ETbCjq1HIWx71DC1ZQpPW54I25YnwkbYsfUoBkN71DC1ZQpPW54I25YnwkbYsfUoBkN71DC1ZQpPW54I25YnwkbYsfUoBkN71DC1ZQpPW54I25YnwkbYsfUoFvHZo4apLVN42vJUYT\/00EPS1tbG5h+2aKflxmNdc+fK8PBwmTHXTvYMhvZtDVNbpvC05YmwbXkSYReIsOPDzZkgAAEIVBeB5uZmWbhwoYyOjsqJEycSX7k0B2g1H2EnvndRQAhAAAIQgICIIGy6AQQgAAEIQCAFBBB2ChqJIkIAAhCAAAQQNn0AAhCAAAQgkAICCDsFjUQRIQABCEAAAgibPgABCEAAAhBIAQGEnYJGoogQgAAEIAABhE0fgAAEIAABCKSAAMJOQSNRRAhAAAIQgADCpg9AAAIQgAAEUkAAYaegkSgiBCAAAQhAAGHTByAAAQhAAAIpIICwU9BIFBECEIAABCCAsOkDEIAABCAAgRQQQNgpaCSKCAEIQAACEEDY9AEIQAACEIBACggg7BQ0EkWEAAQgAAEIIGz6AAQgAAEIQCAFBBB2ChqJIkIAAhCAAAQQNn0AAhCAAAQgkAICCDsFjUQRIQABCEAAAjUv7PovPB17L\/irP\/zfrOf8qz+cnfa9P22h37\/lzE9irwsnhAAEaptAc3OzLFy4UEZHR+XEiROJhjE8PJzo8hUqXM0Le+7cuRJnIw6ceEnOnHsja7ucOTcx5fsXs6Q7c\/7ysbnyydfoN9XPzPz6ptmT\/77xze9uqq+7\/Lv6mXLjm7\/\/d\/PeXqgfZX4fN8\/QBUtxQpjaNh48bXk+++yz8tBDD0lbW5ssW7bMNnPj3NLe9gg7ZmEb97+C2f3jr38vL+aQvLtAcBcG7mKg0IWAk74KX2XvF\/1Xd\/wXGTv6PwqWiwThCaR9kAlf03hSwtOWM8K25ZkvN4Rd5cK26EoqcP3Pid8JXYXvl3020eeS+4ffPRm1R4neLeqSxjxeeOEFmTNnThqLnsgyw9O2WVTYg4ODsmDBAiJsW7TTckPYCNu0i+lg+Ndve1dG8EG5axQfFLtf6h+eN9srz9bbm03LlebMEIxt68HTlifCtuVJhJ2HANNjtp0t6mCoU\/Y\/+dffe4VwEfs\/vvmzK5kKXaffncw1Oq+lyDwqU9sWrb7c4GnbpgjblifCRtix9SirwdBF4SrzySn5yel3v8xdZK4C1\/vo1SpyK6axdYKEnwietg2EsG15ImyEHVuPimMwdPfUXWT+k1+fnyZyF5Gr1HW1e5oj8jiYxtZBEnAieNo2AsK25YmwEXZsPaqSg6Ff5MGI3EXj7R96V+ruj1eSaWwdJ8YTwdMWNsK25YmwEXZsPSppg6GbWtfn3\/Xjj8b998aTvMgtaUxj60xlOhE8bcEibFueCBthx9aj0jIYqsgnX2Iz4f3ffVTiGoXrJyn3xNPCNLZOVuKJ4FkiwMDhCNuWJ8JG2LH1qLQOhv7p9FxReKUEnlamsXW6iCeCZ0RgBZIjbFueCBthx9ajqm0w3PnEiMdu5xMvZI3C45hKrzamsXXGHCeCp20LIGxbnggbYcfWo6p5MNQo3D1mFozC3aNl5RB4NTONrWP6TgRPW+oI25YnwkbYsfWoWhoM45pGryWmcXRUeNpSRti2PBE2wo6tR9X6YOgWs+WbRo96L7zWmVp3XnjaEkXYtjwRNsKOrUcxGE5H7bZULXYxG0xtuy88bXkibFueCBthx9ajGAwLo84Xhbs3tPmjcJgWZholBTyj0CqcFmEXZmSVgt262K3Lqi95+TAYRseZ61645qTPhV9bJ\/K3\/yZZz4ZHr2VyjqCP2rYFwrblmdgI+9FHH5Xu7u4p5duxY4esWrUqNgLs1mWLmsHQhqdf4r8\/f15OnZVp70t3K9Or4X3pNtTC5UIfDccpbCqEHZZU6ekqEmEfP35c1qxZI9nk7CR+6NAhWbx4cdYa+kWvm6b39\/dLfX29nDt3TtatWycnT56U5cuXS19fn9TV1Yk7n2YWPCfCLr0T+XNgMLTlGZy1yPVomTtrtq1I9Xdp3vzEmih91JYowrblmagIW6U6ODgoa9euzVvLgwcPZk1z+vRp6e3tld27d3uS3rlzp4yPj3ty3rNnjzQ3N8uKFSukq6tL2tvbpaWlRTo7O6Wnp8c7n\/9Y\/Rlh23Y2BkNbnkFh58rdL\/JC+4rfqPuL19d5WUVdsW5fu\/hzpI\/aMkfYtjwTJWzrqmn0PDAwIFu2bJGNGzfK1q1bvchco\/CRkRFpbW31pK5RuEbbTuQuekfYti3CYGjLM6yw84lcf+de+OL9+9fn5cx53WP8jSmHuR3NdOGbX+puyt2L3utn2lcw5hzpo7bAEbYtz0QK2z99HSxgQ0ODHDhwQObPn1+QhMpYo+rbbrstE0nrcSrsoaEhWblypRw+fNiLwPWjwl6yZEnmPrkK232OHDlS8HwkyE9gbGxMGhsbwWRIoNxMx1+7KL+9cFF+OjYp8PELF8V9p\/\/P9mm4eob39btmzRD374ZZb3735u\/0+w\/ekDzBl5unYdOnIiu9ANKxdt68ebJo0aJElnnp0qWZcg0PDyeyjGEKVZF72K5gLgrWqFg\/+rN+mpqavKj5\/vvvz1sH\/\/F6AeCmvqMKO80NGKaR40xD9GJPO2lM3XPlWlM3\/e79O0vUHqThj9A1ktePRvPu46bq9efLe5hfbwo1aTxNK1eBzIiw44NeMWGrYLdv3y7btm3z7kXrx323adMm7350PmG7yNqtKHcRO1Pi8XWebGdiMLTnn2ambrW7UnkxIHSVvfe9b2pepT95ITAzp1PvAAAXEElEQVR1uj4s1WxT9u7CwOUx8cYb0tLw9mlZ+i8Wsl1otH\/I9sIhbJ2Sng5hx9dCFRP2xMSENz2t09\/+CFunVvR+9MMPP5z5PohDZa33poOryJ3EWXQWXwcKninNcqkctfxnhmm0lnE7rLmj3IWB+\/n111+X83+anL4PftwFw7Tvi7yAiFby+FJbrkV49dVXvYJ\/7aN\/kWXLlsVXiSLOlPY1SxUTtrIO3sfWR7QeeOAB2bVr15T7zP528T+i5b53j3DpRYB7rGvDhg0Z4fuPCT4ulvYGLKLPlvUQ5GKPF6a2TOEZnmfw4ifbkSrsX\/zi\/8jd\/\/YahB0ebVEpEyPs\/fv3y1NPPeU9yhVmsVlRtc1yEMK2IjmZD4OhLU+YwtOegG2OTInb8syXW8WE7abEdcW2e\/xKC6qLzdwLT+LAgLBtKSNsW54IG572BGxzRNi2PBMpbP+is3379nn3pPUlJ8GFaOVGgbBtCSNsW54IG572BGxzRNi2PBMp7GwR9rFjxzJvLdOXnMTxQdi2lBG2LU+EDU97ArY5ImxbnokUthYq26Iz917wuBAgbFvSCNuWJ8KGpz0B2xwRti3PxAo7vmrmPhPCtm0FhG3LE2HD056AbY4I25ZnooSd75WkWlD\/7ltxYEDYtpQRti1PhA1PewK2OSJsW56JEra\/MLleTcp+2PF1AOszIWxrojwqZ02UPmpLFGHb8kyksPO9mtT\/utJyoyDCtiXMYGjLkwgbnvYEbHNE2LY8Eyls\/ypxF1H797ZmlXh8ncDyTAjbkuZkXjC1ZQpPW54I25ZnIoWthWKVeHwNHdeZGAztScPUlik8bXkibFueiRV2fNXMfSamxG1bgcHQlicRNjztCdjmiLBteSZK2BpVDw4Oeu8Mz\/c5ePBgwTQWmBC2BcXLeSBsW54IG572BGxzRNi2PBMlbC2M2z1rx44dElwRrivHu7u7JbirVrmQIGxbsgjblifChqc9AdscEbYtz8QJ2xXIydlfwGwSLycOhG1LF2Hb8kTY8LQnYJsjwrblmVhhx1fN3GdC2LatgLBteSJseNoTsM0RYdvyRNh5CCBs286GsG15Imx42hOwzRFh2\/JE2Ag7th6FsO1Rw9SWKTxteSJsW54IG2HH1qMYDO1Rw9SWKTxteSJsW56JFbb\/xSn79++Xp556ynuUa\/78+bERYErcFjWDoS1PpsThaU\/ANkeEbcszkcL2v5p0ZGREWltbvXIODAxIX1+f8GrS+DqB5ZkQtiXNybxgassUnrY8EbYtz0QK27\/5x759+zxht7S0yPbt24XNP+LrANZnYjC0JoqwrYnSR22JImxbnokUdrYI+9ixYzI+Pm4eYbsXtSiI4HPeTInbdjYGQ1ueRNjwtCdgmyPCtuWZSGFroeLY\/EPP0dnZKT09PR6H3t5e2b17t9TX13s\/I2zbzoawbXkibHjaE7DNEWHb8kyssPVNZ3r\/ev369bJu3To5efKk+StJNbrWbTv7+\/u9++JdXV3S3t4uixcvRthl6GcI2x4qTG2ZwtOWJ8K25ZlIYeuU+H333eetCn\/mmWc8ca9cuVJ004977rnHbNGZCtstZFMQKuwlS5Zk3mH+4N\/MyvC5\/fbbL7Oa3Ri6Fa647fOh01Z7wrGxMWlsDM+u2nlY1A+mFhQv5wFPW556ATQ0NCTz5s2TRYsW2WZulNvSpUszOQ0PDxvlGn82V1y6dOlS\/KednA53C8x00Vlzc7Pcdttt5ovOCgn7+x+vk0VvRttBDhdfHimI5uIrhdMUzCRPghnXNk\/77Yzrpn53pS9NMP2M65pkVttdpRQh0rFEL5FwhUoM01CYQieCZ2hUoRISYYfCZJKoYsJ2Efadd94pe\/fuzdxjLkeEneYp8fPf2T6tof0XCX8OXDD4LzLCXkz4Je8uBtxFgPvd7E9tC9XhGAxDYYqUCKaRcBVMDM+CiCIlQNiRcJWUuGLC1lK71dsbNmzw7mO7xWGWL05h0Vn2\/vHGqaPiZH\/x5dFMIif5y78bkXzid0JX0avkL7x+QWbPeZ+XX90tk8\/W6++yzRSU1HNr6GAEY9vY8LTlibBteebLraLCjqua\/se6gvtss0q8+Fa4cPRb4mSvUlfJv\/HGGzLjtZeySt4v97qb27wTh43ciy9l+o9EMLZtCE9bngjblmdiha1T1Tod7v8sWLDAW9HtHrsqNwqEbUs432Dopvcn\/uWod1KN8oMflbpG5E7oGqXXeoSOYOLro7Znqo3cEHZ87VyxCNs\/Va2rxJuammR0dHJqdtWqVbERQNi2qIuRi0bneu994tQxrzCZaD0g9Gwyn3nLZKRezZ9imFYzj1LrBs9SCU49HmHb8kxkhO1fJf7888+LvuVM72PzatL4Gr8cZ7IeDN3984lTR73p92wy94tcV8XrvfRqErk103K0e5ryhKdtayFsW56JFLauEt+zZ48n6bNnz0pHR4f3WlKmxONr\/HKcKc7BUOXtF7n3b9+qeRW5e6RNp9bTKvE4mZajTyQtT3jatgjCtuWZSGFrofTNZldddZW3naYuDNu8ebMcOHCA7TXja3\/zM1V6MHTCvvD0Qa9uer\/cf688jRKvNFPzTlLhDOFp2wAI25ZnYoUdXzVzn4l72LatkNTB0B+N+yXuVq5rJJ7UFetJZWrbc+LLDZ62rBG2Lc\/EClvfJd7d3T2lfEyJx9f45ThTmgZDlXi2SFwlXndLm\/fseBIkniam5ehT1nnC05YowrblmUhhu526tm7dmtmII75qXz4TEbYt9TQPhv7V6sEo3D1qVol74Wlmatu7bHKDpw1HlwvCtuWZWGHHvSI8GwiEbdvZqm0w9Efh5x+7NwPL3QuPQ+DVxtS2x0XPDZ7RmeU7AmHb8kyksLVQOiWunzifuw7CQNi2na3aB8N898LLdR+82pna9sDCucGzMKMoKRB2FFqlpY39xSluKlxXiGf7cA+7tAat9NG1NhiGmUYv9T54rTEtdx+Gpy1hhG3LM7ERdnzVzH0mImzbVmAwnHxTmy5my\/dIWRSJw5Q+akvANjeEbcszccI+ffp05kUpy5cvl76+Pqmrq4uv1r4zIWxb7MhlOs980+hhFrPBlD5qS8A2N4RtyzNRwtY3nHV1dUl7e7u3Olw3AGlubq7YfWyEbdvZkEt4nroZSrYo3D1S5ha0wTQ80zAp4RmGUvg0CDs8q1JTVuQetn91uEbbTzzxhGzcuLHUuhR1PMIuClvOgxgMi+OZ6164l9vsRpnZMM\/bwawa35VeHLHij6KPFs8u25EI25ZnoiJs\/6YfuoWmCvvgwYNyzz33VGRaHGHbdjYGQzueTuLjP\/mfMnP85LTtSP0veGEb0vDc6aPhWYVJibDDULJJk4gIG2HbNGYScmEwtG8FP9NC70rXs\/tFrj\/H8ay4fa3LlyN91JYtwrblmbgIe926dd7GH9k+PNYVX+OX40wMhvZUwzANbkOq98Z1j\/Hg7mWe0K9r9qbXa1XmYXjat2L15oiw42vb2CPs+KoW7kxMiYfjFDYVg2FYUuHTWTLVhW7Z9hR3pXF7i+ue4m5jFI3Qnejdd+FLn7yUljyTV7v4S4Sw42OOsOfOleHh4fiIV\/mZGAztG7jcTP0L3rT0TujBCN0vdSdwJ\/Yoz5XbE4qWY7l5RitN+lMj7PjaEGEjbNPexmBoitPLLGlMNUr3i13\/7d9zPEjAReU6Fa8ff\/Q+Kf4m77uZt0xO05f7kzSe5a5vufNH2OUmfDl\/hI2wTXsbg6EpzkQKO0oNLxz9llx8eTQjeP3Hn18Z8e6vO+nnyy+b7D3Jl7D1KX00SgsWTouwCzOySpFKYfv30fYvUvO\/p9z\/BrXjx4\/LmjVrPGY7duyY8pIW7mFbdaXJfBgMbXnWElM3Ne+k7gk9i+wnvw8nfCd37\/9vRvhv1F0js946qyTp27dyenNE2PG1XeqErc9t9\/b2yu7du0Wf49Y3pY2Pj3uvN92zZ4\/31rQVK1Zk3qbW0tIinZ2d0tPT41H1H6s\/I2zbzoawbXnWkrAtyAWl74Tvj+YvvPiczJw5M7T0C03pp+n+vQXjYB4q7Ku2v0+e6\/gHWbZsWTlOYZZn2sf71Ak72HIaPQ8MDMiWLVu8t6Vt3brVe+WpRuEjIyPS2trqSb2\/v997MYv\/tagI2+zvIJMRwoapPQHbHAv1UXc\/XqfuC0X5\/sfm\/KX0S95\/z74a5Y6wbftnvtxSL2z3LvLbbrstE0nPnz\/fE\/bQ0JCsXLlSDh8+7EXg+lFhL1myJDMtrldc7nPkyJH4yFfpmcbGxqSxsbFKa1eZasHUlntcPC89eb\/I+TGR87+ZrMA5\/ffY9MrMfvPvpb5RZO4i7\/dX6HezbxB592LbypchN70Aan5wqfzzJwdk0aLJ8ifts3Tp0kyR0vxUUKqF7aJojar1\/rWb+o4q7DQ3YNL+MApFL0krbxrKA1PbVqo0TxeV6334yQV4lxfl6c\/ZVty75+PdC2+SFKkTYdv2z1RH2G53r8HBQWloaJADBw6ICjm4y5dbcMaUeHydJ9uZKj0YVrb25Tk7TG25pomnyttJPdcLb4Iyj\/tVtAjbtn+mWtjZCq+y1nvTeq\/a\/3ESZ9FZfB0oeKY0DYaVoxTtzDCNxqtQ6mrgGXwVbTaZ+0VeTokj7EI9zu73qZsS9z+i5TC4R7g0GnfvKd+wYYO3AE0\/\/mMOHTo0RfRpXzVo1xVscqqGwdCGhF0uMLVjqTlVO0+V98QpfZf8qPfWOn0W3n3KIXGEbds\/qy7CtsSDsC1pVv9gaEsrXG7VLphwFOxS1SJP\/+tns0l8VttdHuBi7o0jbLu+WSin1EXYhSoU9fcIOyqx\/OlrcTC0JTg9N5jaEobnZZ56j3zi1DHR3d3cYjeNwqMIHGHb9k8i7DwEELZtZ2MwtOVZC1O49sS4qCyWqRd9P30wq8BzRd8Iu1ja0Y8jwuZd4tF7DRdApswKZcZFZSFC0X4Pz2i8dLMX753wr4x4r3N1+6k7gevvz3z\/v8pv\/uMh3nQWDW3k1AgbYUfuNPkOYDA0xellBlNbpvAsnqfbT92\/kE1z+9+zPyxv\/Q89CLt4tKGORNi+N52FIkYiCEAAAhDIEND9GxYuXCijo6Ny4sSJxJNJ84uyal7Yie9dFBACEIAABCCgr6y9dOnSJUhAAAIQgAAEIJBsAgg72e1D6SAAAQhAAAIeAYQd6Aj6trTHHntMfvWrX8lnP\/tZmTNnDl3FgMDFixfle9\/7nnzsYx+T2bNnG+RYu1mcP39evv3tb4vuOrV69Wp5\/\/vfX7swjGr+2muveUyV7dq1a+WGG24wyplsHn\/8cbnxxhvlAx\/4ADBKJICwAwB\/9KMfyTXXXONtMKJ\/wHfddZe3jzaf4gm8+uqrsm\/fPm9\/8i9+8YtSX19ffGYc6W0d+973vtf77xvf+IZ85jOf4SKoxH7xwx\/+UHTx1Dve8Q7v7\/5zn\/ucvOUtbykxVw5\/\/vnnvb\/9T37yk9P2foBOdAI1I+zg9puKSge+7u5uj5p7x7gOgJ\/4xCe8K8KDBw+KvqccwWTvWGGZ6qzFX\/7yF\/nud78Lzzx\/o2F5uiw0GvzmN78p+t78t771rdH\/+mvgiChMNcr+8Y9\/LFdeeaXceeedcsUVV9QAoehVDMv09ddflx\/84Afyzne+0+ufwc2aop+ZI2pC2KdPn5aOjg6vtd32nPpdb2+v7N69W\/QqcGBgQPr6+rz\/q6SvvfZahJ3n7yMKUzdDwQVQbqBReaqs9+zZI+3t7dLS0sJIloVAVKZ\/+tOfvI1BNNrWiyBm1qZDDct0x44dcvToUXnPe94jr7zyipcRwi79z7Tqha1Xgzol8\/GPf1zuvfde2bVrlzfdrdH10NCQJ2mNADs7O6Wnp0dOnTolN998szc9poJZtWqVXH311aWTrqIcojJV3vpB2LlnKqL00be97W1en9bbNdxrtWH6u9\/9zvu71\/UVX\/\/610W36IXtVLZR\/u43b94sv\/zlL+XFF1+U3\/zmNx7Xz3\/+88wEleiBqhe246NXhlu2bJkibL2nqltwakfUbTn13zoVrtPi2sFUNHfccUeJiKv38LBM3ZU1ws7fF8Lw1D78s5\/9zBsE9X6ryvtTn\/oUA2EOtGGY6t+93vbSi3idutV\/f\/rTn5YZM2ZU7x9vCTULy9T93ev2xkTYJQD3HYqwA8Jm2iZ8x4r6hxs+59pMCU\/7do\/CVNdZ6H+IuvQLS70IYiy17881LexsU+Ju+tYedfXlmG0whGnx7QzP4tnlOhKmMLUnULkca1bYuRadsdAkfGcMDoYwDc8uW0p4lsYPpvb8YBoP07BnqVlhKyD3WFdDQ0Nm9XhYcKQTCQoGpqX1CniWxi+MXOijpTOmn5bOsNgcakbYxQLiOAhAAAIQgEASCCDsJLQCZYAABCAAAQgUIICw6SIQgAAEIACBFBBA2CloJIoIAQhAAAIQQNj0AQhAAAIQgEAKCCDsFDQSRYQABCAAAQggbPoABCAAAQhAIAUEEHYKGokiQgACEIAABBA2fQACEIAABCCQAgIIOwWNRBEhAAEIQAACCJs+AAEIQAACEEgBAYSdgkaiiOknMDExIV1dXTI4ODilMhs2bPD2YU\/zR98t\/cQTT3h7ymsdlyxZIqtWrfKq5Ord3t6edbtF\/f2ePXtk\/fr13j7UfCAAgdwEEDa9AwIxEHDi8ssshtOW\/RR+4epOd1GFrQV0wt+4cWPZy8sJIJBmAgg7za1H2VNDIJ+wddc43Uf8zJkzsnr1arn11lulo6NDxsfHZcGCBdLf3+9Fn8ePH5c1a9aI7i7X1tYms2bN8iJTjWw1Sl+8eLG3A93IyIj3s9uNTiG5SF7z2Lt3r8ft2LFjsnz5cunr6xOV7c6dOzO\/O3TokJdmYGDA+71+VMbBSFnzGx0d9SLqbHX0R9h6Pnduzc9\/7gceeEBuv\/12YT\/61HRpCloBAgi7AtA5Ze0RyDYl7mT85JNPyiOPPOKJWT+dnZ3S09PjycsJ2C9mPU7lqeLOJezW1tasstX8N2\/e7G0ne80112Rkr9+rsLUMZ8+eld7eXrnnnnvkq1\/9qmzbti3z3e7du6dMXesxei69WMg17a956wWApnEf\/3H6ndZTP24qvfZ6CDWGQGECCLswI1JAoGQCYSJsjWTHxsYy0bU7qQr67rvvlgcffDATbWcTuT\/Cbm5ulu7u7inl1ihb5erE7KawNWrWKNlF5v6DnFj1O42Q\/ffbtU733XefrF271ru4KBRhO2EHZa15a6QezL9k6GQAgSojgLCrrEGpTjIJRBG2RrfBSFaF5kSr0+NhhJ1NwP58wgjbiVSpukjaES5G2LkiaYSdzH5LqZJFAGEnqz0oTZUSCCtsTaf3pPVetk4Pu\/vbW7ZsEV2UpRGuf0p806ZNmYVeK1asyEyVq1zd1HdjY2MmTVNTU9YIW7H7p8T1fLt27RI9VldxP\/fcc9MuItwxwSnxXKvENYrPNe3NlHiVdnyqZUoAYZviJDMIZCcQVtga9eqq6bCLzvyLy\/yL0fItOss2Je6m0900+o4dOzL3k\/0L2YK180fG+abE77jjDm9K\/+TJk5ks3D18rbN\/ap0+BAEIZCeAsOkZEEghgXwStaxOHM9R81iXZYuRVzUTQNjV3LrUrWoJxCHsc+fOedPzN910U+bRr2xAS7n\/HLwPXrUNRsUgYEAAYRtAJAsIQAACEIBAuQkg7HITJn8IQAACEICAAQGEbQCRLCAAAQhAAALlJoCwy02Y\/CEAAQhAAAIGBBC2AUSygAAEIAABCJSbAMIuN2HyhwAEIAABCBgQQNgGEMkCAhCAAAQgUG4CCLvchMkfAhCAAAQgYEAAYRtAJAsIQAACEIBAuQkg7HITJn8IQAACEICAAYH\/DxyOm\/A5XxLtAAAAAElFTkSuQmCC","height":296,"width":492}}
%---
%[output:29bd839f]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAewAAAEoCAYAAACaU8LCAAAAAXNSR0IArs4c6QAAIABJREFUeF7tvQu0FcWVPr5BEBlARkAmIAoaQQ2OqOiIqImKRoLiIxpRfA0SNUZBo8gzgw4SeQgqasIYBdSJCD5HQRAVRZ0BjQYlGkVI8AUY3wvfb\/5rV351\/n37dp+q6q46XXX667VYl3vu3rt2fbu6vrPr2WTz5s2bCQ8QAAJAAAgAASDgNQJNQNhexwfOAQEgAASAABAQCICw0RCAABAAAkAACASAAAg7gCDBRSAABIAAEAACIGy0gcIQ+Pzzz2n06NG0YMGCVB8GDhxIkydPppYtW1rzc+3atTRkyBDauHEjzZ07l\/r06ZPZdtRWkpG4\/0899RQNHjyYOnfuTHPmzKHu3btnLrvWivPnz6cxY8Y0KDYpPlOmTKEbbriBVLGLxv+cc86hUaNGCdtJ7YLj1KtXL9Fe+vbtS4MGDUqtvsS4Gj6TJk0SNpJ8TfOf6798+XLr7bHWcUR54SIAwg43dsF7rkPYXEnuqGfNmkXt2rWzUudaEnbc\/xAJWxWneHzyEnb8iwF\/uZkxYwZNnDiRVq1aRZJs0xqDC8LWrZOVBgojQCAFARA2mkZhCKiIIOpY3kw4aqvWhM1lS5IJkbB1CDCaIeclN0nY0S8CH3zwAQ0dOtQ6YSc1fpOsu7CXBwWXEgEQdinD7kel04ZEpXdRYo1nVUlD0UmkHv9SwMO0Z555Jp133nmNhsTjNnUy+2rkHyUZSWhRwv7tb39Ls2fPrkwJJJWXNAwdxyJajsQuzfc4+aqGrdme9CE+jB\/FNlpelPCiWLOttKFv\/nz48OGJUyT7778\/rVixolGjTfsSZ\/KlSEXOI0eOpPPPP198UYg+0XqoMI1+ATnrrLOEvTgWfryR8MJ3BEDYvkeojv0zIexo55xEYhKmaEeaRGRxOKXdtCxSNdeclbDTwhol0Gr1lKRdrY5x0k6zp\/piEtWL4ptWB0mCaX+XvsfjHyJh62CaJmNz1KiOuwlULYIACBvNoTAEdIfEoyQWJcgoeUQ7RdkRJn0WJziW7dGjR2W4VdpUfZlIGgWoBmR8SJxlk8hZfkFo3759xSdZn6hPUpczv\/gitqSRiaTPdIeZ074UpBF9lLCTyFn6zhjIRYdJsXQ1JK4avldl3XIRpC6mpl94CnshUbD3CICwvQ9R\/TqoS9hJ2XWcLKpla2krtRlZts1P0sptmXVXy0BVq8TjxBzN5KP1Us2rpw27SsLmcqr5mTQvzDryc9XQeLV6xkchJOGlZfjyc175HzJh62J6\/\/33i9X1qtGa+n3TUTNbCICwbSEJO8YI6BI2G662DUcWHM2Mxo8fTxMmTBDzw\/Fh3Dg5vv766422K0UrU62jVRF2vOy0+dUkwq622CspS436HJ\/nVg1Tq4bFpW2dbLvatigmrnohbF1MH374YdG+dDE2fpGgUBoEQNilCbV\/FdUZdo5nazfeeGPqHt+iCVtnTlKXsHlIXO4VlyMBch8yfwmJZ8RpRCp90iUX061zSdMOvhC2TkarO\/xdTS7tzZIEDcL2r+8J1SMQdqiRqwO\/sxB2WufnYkhcB2LVUHbchi5hy2H6aFaWNIeddKBMkpwcljXN8qJfBJKGzaOjAKpRkPgQsushcdeEnTYkHo+5rpxOe4NMuREAYZc7\/oXW3mRIXJLF+vXrK5mnyaKztFXVaYvOGBid\/cSuCVtm13waW5QcJR5p86PxkYn333+\/glvaKm150li8Uaiyc5aPkmNZMuykRWdJX0JB2IV2M3VVOAi7rsIZVmVMCFt3W1d07tZkW1e1LVTVhrpdEXZ05XpSVCVhM4byQJEkuSgeacSrykR1cFStvGbf8mTYaceVJh0ra3MftlwRHm8fsr46mIKww+qXfPYWhO1zdOrcNx3CTiOTrAen8JAwZ5IjRoxQHpyiIjIOjyvCZiKK15HJt2vXrokr2pOII+mLRnwhm8kQedKXmiSMXGTYjHXc97QjSl0QdvxLS3R6QIUpCLvOO7IaVg+EXUOwURQQAAJAAAgAgawIgLCzIgc9IAAEgAAQAAI1RACEXUOwURQQAAJAAAgAgawIgLCzIgc9IAAEgAAQAAI1RACEXUOwURQQAAJAAAgAgawIgLCzIgc9IAAEgAAQAAI1RACEXUOwURQQAAJAAAgAgawIlJ6wd9ppp6zYQQ8IAAEgUBWBZs2a0TfffEP8M\/r\/7777jrbccktq0aKF0Oe\/ySft\/4DaDgKPPfaYHUMFWAFh77QTrVu3rgDoUSQQKA4B\/qKKdq\/G\/+9\/\/zt973vfI\/4p\/\/\/SSy\/Rs88+S2+99ZYwwJ9Hf6qt+ivBdU17qv2NdZo2bUodO3YU6vxlRH5R2Wqrrah169bEP\/lymS+++EL8f4cddqBNmzbR5s2bqXnz5kJPYh3\/v\/Sp2t+lfzIeUjb684c\/\/CGtWbPG3wAoPANhl6DjevXVV2nHHXcstJG69sGm\/Ty2THVdyOvYLANhV8Ph+eefF+SwYcMGWr58eeXLiyTmWr8scTKM\/s6nyXHW3aFDB+rUqRPxTW5NmjRp8EVizz33FF8c5M8o4UWJLEpe8TrqtJs0XLLomujoyqrkQm\/3IOwSEHatOx+U5z8Cqo7N\/xokexjNdp955hnibJgfJmjbT5RU5f+ZUPlf9+7dqVu3bpXsW\/49niHa9gn2qiMAwg68hYQewMDhh\/sFIRAqYUcJma9a3bhxY2W4Og+UUUJlO0y4hx9+OH366aeVodqkrDVPmdCtPQKh9\/fIsJFh1\/6tQYmFI+A7YUtivu+++2j16tWZM+QoETMJH3XUUcQXefAjh49Vc7OFBwsOWEMAhG0NymIMhR5AHdR86Jxd+2DTfh5bprou5HVs6sjotK28MnLOeMmSJSJTNh26lmTL5MsLiqJrNfg2uKLXbjA+rrG2ZT+PnSy6Jjq6siq50Pt7ZNglyLBVjThvp6uj79oHm\/bz2DLVdSGvY1NHRieuJjJMyLzA68knnzQiZiZl\/sekfMQRR1RWa6sy4yLqmISHaz9s2c9jJ4uuiY6urEoOhG3yxnooG3oAPYQULgWAgKpjy1sFJueVK1cSzzPrZs2SlPv16ydWRTNB4wECNhEIvb9Hhl2CDNtmg4et+kDANmEzKfOw9oMPPqgESGbMJ554IrVs2RLErEQMArYQAGHbQrIgO6EHUAc2252zTplxGdc+2LSfx5aprgt5HZs6MmlxlgvCpkyZosyeJTmffPLJ4jCNWmbNeeqYpY2n6bj2w5b9PHay6Jro6Mqq5ELv75FhI8O22TfBViAIqDq2eDU4g77lllu0CJpXYvfs2VPMO6vmmQOBC27WCQIg7MADGXoAA4cf7heEQDXClhn0bbfdRgsXLkz1UBLyqFGjhAzIuaBgolhtBELv75FhI8PWbuwQrB8E4oStM8wtCfqMM86o6dB2\/aCOmhSNAAi76AjkLD\/0AOpU33T4U8emqYxrH2zaz2PLVNeFvI5NKSOzaEnY8bgySY8bN06cYx1aBq2Dg2k7ziLv2g9b9vPYyaJroqMrq5ILvb\/3NsOeP38+jRkzRrwffPj9nDlzxHGBOs\/atWvpiiuuoOnTp4vbYao9oQdQBw9VI9axkVfGtQ827eexZarrQj7NpiTl22+\/ne6\/\/\/7EkDIp77XXXnT66acHP8xtim3eNp6m79oPW\/bz2Mmia6KjK6uSC72\/95KwmaznzZtHs2bNEoQb\/73ai8XHDg4dOlSISP2yE7arjgh2w0CAifrOO++ke+65J5Wkjz32WBo0aFAYFYKXQCAjAiDsjMClqUnCPemkkyodCB8xOHr0aOrbt2\/VTuWpp56iwYMHC9O9evUCYVuODcyFgwCTdLVMmoe4x44dK7JpPECgLAiAsC1HmoezR44cSVOnTm0wBM5ZNh9rOHnyZHHYQvyRZD1p0iTxp2iGXvYMWzVMZDmEieZc+2DTfh5bpro25Zmkv\/vuO7rgggvovffeaxQHHu7mFd3805dztl22PVNsXfni2g9b9vPYyaJroqMrq5IDYVtu5Uy8fCBDfDjbZFjcRDb0AFqGH+YCRKDaHmkm56uuuoqaNGnSYNGYqmMLEAa4DASUCITe33s3hw3CVrY5CAABcQHGH\/7wB3rggQcSM+nzzjuPdt5559SV3SBsNKIyIgDCthx1ELZlQGGubhCotlc6epuVzvGfIOy6aRaoiAECpSPsb775hj7++GMBUZs2bcStOjafIghb+r906VKbVfHG1vr166lLly6F+uPaB5v289gy1dWR5\/noRYsW0bJlyxrFkBeP8a6IXXbZpfI3HZs6MoU2GAuF+1JH137Ysp\/HThZdEx1d2TQ5vgFOPuvWrbPQuooxkTokzqu1Fy9eTJ999hn95Cc\/ER3+M888IxaEvf7668LbTp06idXbRx55JDVt2tRKDbIuOosWjjnshqHwIZty7YNN+3lsmepWk+eMOumCDc6m+Z388Y9\/nDjkreODjoyVF7pAI77U0bUftuznsZNF10RHV1YlV5cZ9po1a+jcc88lrjw\/fHDJ+PHjacaMGYLA+\/fvLz7nq\/T42\/\/MmTPpwAMPtPJq5tnWJR0AYVsJBYwUgACT9IYNG2jEiBGNSpervHWGvFWuqzo2lT7+DgRCRKDuCJuHvPlb\/aOPPiq2Vu2+++40d+5cuvzyy6l379503XXXVb7Vc+cybNgw+v73v08TJkwQ1+fZeJhwuRx5upkJAXP5JvKhB9AG3rDhBwK8mnvBggUNnGGS3nvvvem0006zejQoCNuPmMOL2iIQen\/faEj8o48+ovPPP5923XVXcTQobwd58803aciQIXTooYdWPpMwM7k\/++yz9Pvf\/5622WYba+irjiblcvmRNwVFCwZhNwyDD52zax9s2s9jy0SXv\/DylqwlS5Y0urYyLZvWsW9LxtrLXJAhHRxq4ZprP2zZz2Mni66Jjq6sSq7uCFsOSffp06dChkmfRQmbF4rpHANai5fDtIzQA2haX8j7gQBPJ\/H90vFLN2wOe1erqapj8wMleAEE7CIQen\/fKMMGYdttILAGBKIIcEb9q1\/9qhEovC6Er62s1Y1YIGy0yzIiAMKeMoWQYZex6aPOughwFs3\/4kTN5Pyzn\/1MnJFfK6KWPoOwdaMHuXpCoG4Jm1ei8oIyfjZt2kTDhw+nffbZp\/KZDCIvDuOsAUPi\/jZrHzpn1z7YtJ\/HVlSXSZrnp2+++eYGjSM67G1alo68LRl\/W7SeZzo46FnKJ+XaD1v289jJomuioyurkqtbwl61apV2K9W9GUvbYA0FQw+gDlSqRqxjI6+Max9s2s9jS+qmrfjmRZLRbVmmZenI25LJG\/Oi9XVwqIWPrv2wZT+PnSy6Jjq6siq50Pv7RnPYX375Jf35z38m\/qn7tGjRgvbYYw\/in6E9oQcwNLzr3V\/evcALyqJPrRaSmWCr6thMbEEWCISCQOj9vXeXf9Q68KEHsNZ4obzGCFQ7kezqq6+u+fy0ToxA2DooQabeEAi9v29E2HyfLs9Zb968WTtWvFe7bdu21o4n1S7YgmDoAdSBwIfO2bUPNu3r2qpG1PGh77Q46ZYl9XXkbcnotC2fZXRwqIX\/rv2wZT+PnSy6Jjq6siq50Pv71G1dmMOuxauMMkJFgFd882LL+NC3rxl1HGdVxxZqXOA3EKiGQN0RdnwOmzPtRx55hP74xz\/SqaeeSl27dhV48BnifPDDW2+9Jc4Z59tQMIeNl6XeEUgj6ttvvz2oqoOwgwoXnLWEQN0RdhyXe+65hx577DH6zW9+Q1tvvXWDP3\/yySfiqFK5qMb2VZuWYlTVTOgBrAVGZS8jbei71oed2IwDCNsmmrAVCgKh9\/dVF53Jc8UHDRokrtBMeh544AGxz9T2WeK1agChB1AHJx86Z9c+2LQvbWUhalM\/XMjr2NSR0WlbPsv4UkfXftiyn8dOFl0THV1ZlVzo\/X1Vwv7www\/p7LPPphNOOIGYtJOe2bNni3uzQdj+dl2qRlwLz137YNM+27r22msbzVHrZNSmfriQ17GpI1OLduGyDF\/q6NoPW\/bz2Mmia6KjK6uSq2vClldtLl++nPggiF122aXB+\/Xyyy+Lm70GDBhAF1xwAWFI3GX3A9u1QCBpjpqJOulWuFr446oMVcfmqlzYBQJFIlDXhM3AvvPOO+JY0pUrV4q7sffbbz+B99NPP00vvvgi8a1e06ZNo44dOxYZh8xlhx7AzBWHYgWBLEPfocMHwg49gvA\/CwKh9\/daB6d8+umndPfdd9P\/\/M\/\/CJLmh8n75JNPFtl1q1atsmDnhU7oAdQB0YfO2bUPWeynEfUBBxwgRo6yXMhh6ocLeR2bOjI6bctnGV\/q6NoPW\/bz2Mmia6KjK6uSC72\/b0TYPG89f\/582nfffekHP\/gBtWzZ0ud3MrdvoQcwNwAlNRD6Puq8YVN1bHntQx8I+IhA6P19I8L+\/PPPxf7qO++8k958803q3bs3\/fSnPyXOOjjjaNq0qY9xyOxT6AHMXPGSKtbLPuq84QNh50UQ+iEiEHp\/nzokzgemvP322\/SnP\/2J7rvvPnr88cfFPuyjjz5aDIPXS\/YdegB1XhofOmfXPlSzb3rWdx5fTXVdyOvY1JHRaVs+y\/hSR9d+2LKfx04WXRMdXVmVXOj9vdYcNr+UX331Fb3wwgvi1LOHHnooV\/bNQ+584Ao\/nTt3pjlz5lD37t2rvvsffPABDR06lOSRqeecc06Dlbvxv0tjkyZNSt2SxjKhB1Cnw1Q1Yh0beWVc+5Bk35SoZR3z+Gqq60Jex6aOTN6YF63vSx1d+2HLfh47WXRNdHRlVXKh9\/fahB19+WT2zdu9+CpBnve+4YYbqF27dsp3lMl63rx5NGvWLCEf\/z3JgCTjk046SZBv\/HfWWbt2LY0cOZKmTp2qJP9oGaEHUAl4CQX4jG+e1omf9a2zj7oscKk6trLggHqWC4HQ+\/tMhC1DzPPdTz75pCBtzpi33XZbrSxZEi8Ls43Ro0dT3759UzPhJFJ\/6qmniO8elsQf\/123GYYeQN161rscZ9P8cJsAUaujDcJWYwSJ+kMg9P7emLA5u+bFaHzZwdy5c+njjz+mXr16VYizWojTsmAmZM7WJ0+enLgqnTthfqKHV8gsmz\/jveBs47XXXjM+4CL0AOq8Uj50zq58YKLmf3zWPV9IE32yZtR5fDXVdSGvY1NHRqdt+SzjSx1d+2HLfh47WXRNdHRlVXKh9\/fahM2nnvGNXddffz1xNstPp06d6Je\/\/KVYiNamTRvlu5uWBVcbFk\/LwKPD4lw+Z+kLFixo4INq\/pqFQw+gEnQiUjViHRt5ZVz4wJk0f5mT2TX7yDsZeGcD3yyXZR8128jjq6muC3kdmzoyeWNetL4vdXTthy37eexk0TXR0ZVVyYXe3ysJ+91336V77723cpWmfAmnT59OAwcONDqO1BVhH3744WJB2g477FDJ0jmbHzJkCA0bNqz0i86K7jhtli\/JmbdnxYn6kEMOEV8esxK1TT99t6Xq2Hz3H\/4BgSwI1CVhczb97LPPEl\/ssWzZMuLf+SCV008\/nXr27EncWcqhaBPQXBF22sUkOgvaOIDyWbp0qUl1IFtDBHi4m6dirrvuugaldujQQXwx22233WroTfhFrV+\/nrp06RJ+RVADIKCBQL9+\/SpS69at09DwUyTxpLNzzz1XDH8zOfOhKUcddVRlQVl87tikWrUmbC5vxIgRVbeNhf6NSwd\/H7KpPD7MmDFDHIsbfTiL5l0Be+21l\/g4j\/04hnlsmeq6kNexqSOj07Z8lvGljq79sGU\/j50suiY6urIqudD7+0aELQn5yy+\/FNdq8nDzdtttVznhLA9hu1x0ltRxgLD\/gYqqEdei0zXxQS4kS9qaxUTNozt77rlnA7dN7Kvqm8eWqa4LeR2bOjIqnHz\/uy91dO2HLft57GTRNdHRlVXJ1R1hf\/fdd2JPM2c0chX4jjvuKMib5wdbtGhBZ511VqYh8aT90za2db3\/\/vtiWJRvDeMV4\/JRrT5nudAD6Hunaepf0kIytsFEffXVV2N+2hTQFHlVx2apGJgBAl4hEHp\/X3XRGZPpihUr6LbbbhP7rXkue\/vttxeLfW666SY66KCDjIPBJMrzkPJ0M515Zkn0TMacXSURP68W5ow6ui978ODB4ktHlMTjDoceQOMAeKjA7WnDhg1i+iL+cCbNMcdCMruBA2HbxRPWwkAg9P5euUpchiG+WrxZs2Z08MEH05lnnkn77LOP0Wpx1dGk1fZdpx1Nyn6yHp+4Jh8VWZclw\/ahc07yIe1EMiZnHs3hVd+6RG2zjnlsmeq6kNexqSMTRhec7qUvdXTthy37eexk0TXR0ZVVyZWGsOVrwUPmr7zyitjmxdu9vv76a\/q3f\/s3mjlzJm2zzTbBveOhB1AHcFUj1rGRV0b6kLYtSw57J81P65Rts455bJnqupDXsakjo4O7zzK+1NG1H7bs57GTRddER1dWJRd6f6+dYSe9mHzK2RNPPEGPPfYYjR07Vusscd9e8NAD6Bueaf5wNv3www\/TokWLGomkLSQLpW4h+qnq2EKsE3wGAioEQu\/vcxG2CpwQ\/h56AH3H+LnnnhMXskQPOZHZ9JFHHilOJMNTewRA2LXHHCUWj0Do\/T0Ie6edKOSN9DqvQC0752qXcOQd9q5WV5t1zGPLVNeFvI5NHRmdtuWzjC91dO2HLft57GTRNdHRlVXJgbB9fmM1fAs9gBpVdL4PW5I07yZYuHBh4pD3scceW\/WIWJ16gLD19tSrOi3GUUcmb0yK1veljq79sGU\/j50suiY6urIqudD7e2TYJciwXXWcTNS8Ej9+6YrMpHlL1hFHHNHokBNX\/sCuPgKqjk3fEiSBQDgIgLDDiVWip6EHsNbwM0nzmeu8Dz\/pwQKyWkckW3kg7Gy4QStsBELv75FhlyDDzts58wpvPlue70BPI2k+15uvW03bN53XB1U3YdN+Hlumui7kdWzqyKgw9\/3vvtTRtR+27Oexk0XXREdXViUHwvb9rVX4F3oAdeBXNeIkG0zSS5YsoQcffDCVpCdOnEitWrXSOtwkiw86dZMyNu3nsWWq60Jex6aOjAn+Psr6UkfXftiyn8dOFl0THV1ZlVzo\/T0y7BJk2KrOlIe5OTPmLVi33norMVmnZdJ8fChfBqN7ApmqbPy9GARUHVsxXqFUIOAWARC2W3ydWw89gFkBkiu7maAXL16cakbOSfNPkHRWtP3TA2H7FxN45B6B0Pt7ZNglyLBl58yZM1\/iwvvOq2XRLlZ3uyYIm\/bz2DLVdSGvY1NHxn336bYEX+ro2g9b9vPYyaJroqMrq5IDYbt955xbDz2A1QCS2674utTVq1dXzaInT54srk51lUWrXqS8gbZpP48tU10X8jo2dWTyxqRofV\/q6NoPW\/bz2Mmia6KjK6uSC72\/R4ZdRxl22u1X8Y6TSfnwww+nAQMGOCPoojtrlF8dAVXHBvyAQD0iAMIOPKohBlDOP\/ONaUzS8XO6kwj6gAMOoAMPPBCHmATeXm25D8K2hSTshIRAiP19FF9k2J5n2EzIfG3pNddckzrvHA2oHNI+5phjaNdddxUE7UPn7NoHm\/bz2DLVdSGvY1NHJqSOOMlXX+ro2g9b9vPYyaJroqMrq5IDYQf+VvsSQM6SN27cKK6g5P+nLQpLyp7lIrG0ldyqRlyLELr2wab9PLZMdV3I69jUkalFu3BZhi91dO2HLft57GTRNdHRlVXJ+dLfZ233yLBrmGEzCTOpzp8\/n1577TVtUubgysz5kEMOoaOPPlrE29UCsayNCXrhIKDq2MKpCTwFAvoIgLD1sXIu+dRTT9HgwYMr5fDFFH369Klarq0ARueRly1bRk8\/\/bQoVzdTjg9rc9Z82GGHiUNKQM7Om07pCgBhly7kqDAR2erviwKzbjJsJms+hWvOnDnUvXt3iv+eBrAqgJKI5WlgTzzxBK1YsUKY489UC77SypXD10zITMz8MEm7eHzonF37YNN+Hlumui7kdWzqyLhoi7W06UsdXfthy34eO1l0TXR0ZVVyqv6+lu0zS1l1Qdiff\/45jR49mjp37kyjRo2q4DBlyhTx\/+hncZB69epFp556qpg\/zkvC8SxZZsZyjln+vdZD2apGnKXhmOq49sGm\/Ty2THVdyOvY1JExjbFv8r7U0bUftuznsZNF10RHV1YlB8L24C394IMPaOjQoYKYo0PgnGUzac+aNYvatWuX6CnPCZs+knD5J39J4P3M7du3r5ipNSGb+g95IKDq2IAQEKhHBEDYHkSVT\/Li6x2nTp0qhsPlozMsLgk7SsKsz4Hdf\/\/9qVmzZg0Wd4GMPQg4XMiNQOgdV24AYKCUCITe7utiSDwPYYceQJ23zodsyrUPNu3nsWWq60Jexybavc6bY0dGJx55SrJlP4+dLLomOrqyKrnQ2z0Ie6ed8rwr0AUCQAAIAIGAEODLj0J9Sk\/YoQYOfgMBIAAEgEC5EKgLws6z6Kxc4UZtgQAQAAJAIFQE6oKw82zrCjVw8BsIAAEgAATKhUBdEDaHTJ5yJk8301khXq5Qo7ZAAAgAASAQMgJ1Q9hR0pYB0TmaNOTgwXcgAASAABAoDwJ1RdjlCRtqCgSAABAAAmVDAIRdtoijvkAACAABIBAkAiDsIMMGp4EAEAACQKBsCICwFRHnLWMXX3wxjR07tsGxp2VrKKhvORCQWyRXrVolKjxp0iQaNGhQOSqPWpYaAbnbaMGCBQIHH9dAgbCrNFE+8nTIkCFCQl7bWeoWjcrXPQJ8WU63bt0EScv2P23aNOW98nUPDCpY9wjMnz+fXnvtNXGJlM7FUUUAUveEzZ3OFVdcQdOnT290Y5fcCiaBj36j4kzjxhtvpP79+9Nll13W6GKRIoKFMoGALgJZ233Uvsw4Tj75ZBC2LvCQKxwBG22fueH222+nyZMnU8uWLQuvk3SgrglbDu9TnGYDAAAgAElEQVRxZeNXbMb3aaft2067WMSbCMIRIBBDwEa7Z5PVOj6ADgR8RCBv248Oi2NIvIYRjmbPvXr1akDYJiejgbBrGDQUlRsBW+0eazdyhwIGaoyArbbPbqcdd13jKjUqri4zbBk4XjDDz7x58xoQtsnZ4yDsopsoytdFwFa7R2atizjkfEHAVtuX9ZFJXd++fb1adFmXhB1tRLyQIE7YJvdng7B9eSXhhwkCWds9l3HLLbfQuHHjvJq7M6k7ZMuNQNa2v3LlSgGcXHA5cuRI79YugbC7d6+07qR5bBB2uV\/+UGufpdOaOXMm3XTTTSS3tci6+ziXF2pc4Ld7BLK0fd4F1KVLFxo9enSl\/fvY7kHYCsJ237xQAhCwj0DWTqt75H2w7xUsAgH3CNRz2wdhg7Ddv0EooeYI1HOnVXMwUWBQCNRz2y8lYZssOguqpcJZIPD\/EEjqtNDu0TzKgEA9t\/1SErbJtq4yNHDUsf4QSOq00O7rL86oUWME6rntl5KwOcRyG4BcWJB2cApeCCAQIgJJnRbafYiRhM+mCNRz2y8tYUc7L9kgfFwVaNpYIQ8EGIG0TgvtHu2j3hGo57Zf94Rd740T9QMCQAAIAIFyIADCLkecUUsgAASAABAIHAEQduABhPtAAAgAASBQDgRA2OWIM2oJBIAAEAACgSMAwg48gHAfCAABIAAEyoEACLsccUYtgQAQAAJAIHAEQNiBBxDuAwEgAASAQDkQAGGXI86oJRAAAkAACASOAAg78ADCfSAABIAAECgHAiDscsQZtQQCQAAIAIHAEQBhBx5AuA8EgAAQAALlQACEXY44o5ZAAAgAASAQOAIg7MADCPeBABAAAkCgHAiAsMsRZ9QSCAABIAAEAkcAhB14AOE+EAACQAAIlAMBEHY54oxaAgEgAASAQOAIgLADDyDcBwJAAAgAgXIgAMIuR5xRSyAABIAAEAgcARB24AGE+0AACAABIFAOBEDY5YgzagkEgAAQAAKBIwDCDjyAcB8IAAEgAATKgQAIuxxxRi2BABAAAkAgcARA2IEHEO4DASAABIBAORAAYZcjzqglEAACQAAIBI4ACDvwAMJ9IAAEgAAQKAcCIOxyxBm1BAJAAAgAgcARAGEHHkC4DwSAABAAAuVAAIRdjjijlkAACAABIBA4AiDswAMI94EAEAACQKAcCOQi7O+++442bdpE7777Lr3zzjvUsWNH2nbbbalt27bUtGnTciCIWgIBIAAEgAAQqAECxoTNJP3KK6\/QLbfcQosXL6aPP\/64kZtt2rSh448\/ngYNGkQ9evSgJk2a1KAqKAIIAAEgAASAQP0iYETYb7zxBk2cOJEeeeQR6tmzJ\/Xr14\/22GMP2mabbWi77bajDRs20IcffkjPPPMMPf7447R69Wo67LDD6OKLLwZx128bQs2AABAAAkCgBghoEfY333wjMurZs2fTaaedJrJnHvqu9mzevJnefvttuuuuu4TuL37xCxo6dGgNqoQigAAQAAJAAAjUHwJahM1Z80MPPUQDBgwgHu42fXjYfNGiRWKIHA8QAAJAAAgAASBgjoAWYZubhQYQAAJAAAgAASBgEwEQtk00YQsIAAEgAASAgCMEjAn7gw8+EHPRq1atUrrUvn17Ovzww8X89Q477KCUL0LgkEMOURbLc\/j8j4f2v\/rqKyHPv+MBAkAACACBsBBYt25dWA5HvDUm7C+\/\/JL+9Kc\/0Q033EDPP\/88HXXUUdSnTx9h8rnnnqMlS5bQVlttRT\/+8Y8FwfHvW2+9Nc2cOVOsFPftYZ9+\/\/vf03333Ud\/\/\/vfxT\/d53vf+x516tSJhg0bRi1bthRq\/BkeIOA7AjvttBOF3HH5ji\/88xOB0Nu9MWHz6u877riD7r33Xpo+fbrYzhV9+ACVCy+8kE444QT66U9\/Sh999BGNGzdOEBn\/9O1JCmCUuJcuXUrr168XX050H65r79696dRTT\/WCxF999VXacccddd13IufaB5v289gy1XUhr2Mz9I5Lp5Hq4KBjJ6+Maz9s2c9jJ4uuiY6urEou9HZvTNhMwOeffz4deeSRqau+58+fTw888ABdf\/31Irvm33l715133pm37VvXNw2gJHMeOeD\/6xI5kzj\/GzVqlBckbh1IGAwKAVXHFlRl4CwQ0ETAtL\/XNFszMWPClnPYp59+Oh133HGJjnL2feutt9KsWbOoXbt29Oijj9J1110nsnLfHhsBlMPo3377LU2bNg0k7luQ4U8jBEDYaBRlRMBGf18kbsaEzYuuxo8fT6+99hpde+214vzw6MND4sOHD6du3brRhAkTqHnz5iLTXrNmjSBt3x6XAZREztk4Z+I62Thn4QMHDqRDDz0UmbhvjaWO\/AFh11EwURVtBFz299pO5BA0Jmwui8n33HPPJT5X\/Gc\/+5k4ppSfv\/zlL2LY++uvv6b\/+q\/\/EovMfve734kFZ0zePh6cUssASgLnny+++KJYvKcicTmUfsYZZ9Cee+6ZKdQ+dM6ufbBpP48tU10X8jo2dWQyNTaPlHypo2s\/bNnPYyeLromOrqxKrpb9vYtXIRNhsyN8rviVV14pTkBjguanWbNm4uzw0aNHi21cPN996aWX0r\/+67\/SKaecQi1atHBRh1w2iw5glMQ5E3\/wwQeV9WESP+ecc+if\/\/mftUhc1YiVBVoQcO2DTft5bJnqupDXsakjYyHshZrwpY6u\/bBlP4+dLLomOrqyKrmi+\/u8L0RmwpYF8xD5J598In5t3bo1bbnllnl9qqm+jwFkEmdS5ktU5s6dq5WF89743XffHdvKatp6wi1M1bGFWzN4DgTSEfCxvzeJV2bC5uHwtWvX0hNPPEFbbLEFnXjiiYKseYiXh8hbtWpl4kdhsqEEUGbiU6ZM0SJwXhDIUxJZh9ELCwgKrgkCIOyawIxCPEMglP4+DbZMhM0ZNe\/B5lu4+OnVq5dYEc73Xp999tmCwJMWpHkWO+FOyAHkL0f8pYnXCVR7OnToQLyqf9999y0sA3dNEDbt57FlqutCXsemjoyP76uJT77U0bUftuznsZNF10RHV1YlF3J\/z20\/E2Hzvmpe8T116lTx\/vBcNhM234u9cuVKGjlyJB177LFivzaTuM9P6AGU2Mr94bwgkBf5VXt4uJ2\/cDVt2rQwAve5TZTBN1XHVgYMUMfyIRB6f29M2JxdX3TRRWIhGRPy008\/TTxMK\/dccxPge7OXLVtWOTjF52YRegDTsJUE\/sorr4gV+2mPPF6VM3AMn\/vcUu36BsK2iyeshYFA6P29MWHLg1OYtA866CB66qmnGhH2k08+SVdddVUDEq8WTrYxePDgisikSZMabQGLXzrCq6TlqWFSMW6HF2zJc87Tyg89gLqvie4cOBP4eeedRzvvvDOyb11wA5QDYQcYNLicG4HQ+3tjwpZHkx588MF05plnNiJsPmucD0p54YUXBGnzyvFqD8\/BDhkyRJwQxuQqf+cLNeS+bUnWJ510kvgs\/jvbZ7IeMWIEzZkzh7p3797o9zITdlrnzHPgvA4hbS+4jT3gEnfXBGHTfh5bprou5HVs6sjk7h0LNuBLHV37Yct+HjtZdE10dGVVcqUjbCZkHmLls8F5rpTJUw6J87nhDz\/8sNiHzVdq8j\/VHDbrbty4kSZPnly58YrnyJcvX175jH+fN29eg4w9mtnzTVlcZufOnRtk3Wybn3gmHu1HQg+gTp+oasRsg0n75ZdfFjeXVRs+ZywlkeuUDcKujpJObKIWdORtyZjE10dZHRxq4bdrP2zZz2Mni66Jjq6sSi70\/t44w+YGzvPYfCDKwoULRefNw608hMrHkr7\/\/vvi8JTf\/OY3tO2222Z6H+IknkS8MstmAuHtS7wPmf8fHQJPGq6POxR6ADMBrFCS89+q7Ju\/kO2yyy4YOncRBMc2VR2b4+JhHggUgkDo\/X0mwmak+V5sPuWMM23Ozr744guxaImv1RwwYEDmfdjxIfLPP\/9cZM99+\/ZtMK8dHRbfe++9xcp0XrXOw+HyiQ+TJ7WQ0APoutVHbydLO4Utev457gN3HRE79kHYdnCElbAQCL2\/z0zYtsMUXVTGl1\/IIXIQdn6kbXbOTOCPP\/546srztHlvmz4kIWLTfh5bprou5HVs6sjkb3nFWvCljq79sGU\/j50suiY6urIquVIQNg+B33HHHbRp0ybtN7Bt27bi9DPVorMkg9E5azk\/jQxbG\/qaCTJ5861tM2bMENMiSQ+PuuS5uKRmlSlZQaqOrWRwoLolQaAUhB3fUqUTW3n6Gd+HbfpEh7uPPvpo50Pi0r+lS5eaugr5\/4fAe++9J9Yv8H58\/n\/Ss9tuuxHHk+e98RSLwPr166lLly7FOoHSgUCNEOjXr1+lpHXr1tWoVPvFZBoS\/\/DDD+mKK64g7oD5es02bdoIz3j4+v777xenoPGQ9oEHHpjJ4\/i2LSw6ywRjYUo6e773339\/cW865ryLCRMy7GJwR6nFIlCKDDsK8TfffCO2cfEWrqSjR+U+7L\/+9a8NtmolhUnOT8e3Y8UXnmFbV75GXmTnLBet3XDDDbR69epGFWHC5lPW9tprr1zkbbOOeWyZ6rqQ17GpI5Ov1RWv7UsdXfthy34eO1l0TXR0ZVVypSPs+ElnSa8lrx7nPdrR40rTXl95Opk8lUySOMvLhWeyTN6yxVu30g5O4dPSpB2dFeJcRugB1OkWVY1Yx0ZeGfaB1yMwgfMXvqQ5byZv3mVwwAEHGJO3zTrmsWWq60Jex6aOTN6YF63vSx1d+2HLfh47WXRNdHRlVXKh9\/fGQ+LyLHGeh7zggguoWbNmDd5LzsB5uJznyHROOmPl+JGiSceO4mjSors\/u+XLzLsaeV944YXUtWtXY\/K262l9WlN1bPVZa9Sq7AiUjrA54Pfccw+NGzdOXKXJR4XKeUjuhHn4mk\/L4tugeD+270\/oAfQdXx3\/dMj76quvFqYw562DqFoGhK3GCBL1h0Do\/b1xhs0h5CyaT8Hig0q+\/vrrBlFt3rw5jR8\/XhB5PPv2MfyhB1AHUx86Z10fmLxffPFFcVJe0sOEzeQdJ25d+67xMvXDhbyOTR0ZHax8lvGljq79sGU\/j50suiY6urIqudD7+0yELV9SXi3+3HPP0V\/+8hfxUc+ePWnfffetrBr3+WWWvoUewBAwzuojk\/eSJUvo5ptvTjTRv39\/sccbWbc5wqqOzdwiNICA\/wiE3t\/nImz\/w6P2MPQAqmtYHxJ8\/C2Td9LxqEzYY8eOFWfXg7z14g3C1sMJUvWFQOj9vRZh84IvXkh22mmn0R577KG8gSsaYt7m9ec\/\/1nctsX3XPv2hB5A3\/B07Y\/ufDeIu3okQNiuWyrs+4hA6P29FmEz6T7xxBN02WWXiW1Q\/\/7v\/0777LNP5TrMpMDwanK+IpO3dvEtXqz7ox\/9yLsYhh5AHUB96Jxd+CCPRh0zZkwiDGnz3SrM8vhqqutCXsemjowKJ9\/\/7ksdXfthy34eO1l0TXR0ZVVyoff3WoQtX8xPP\/2U5syZQzfeeKM41Yy3dsXnrPl4yhUrVhAD1759ezrrrLPolFNOyXx7l+tOIfQA6uCjasQ6NvLKuPbhmWeeIT5ykO9qjz9M3LwFsVu3blpD5nl8NdV1Ia9jU0cmb8yL1veljq79sGU\/j50suiY6urIqudD7eyPCli8gkzWTMm\/h4j3UH3\/8ceXdZJLu3bu3uPiDj5\/kwzJ8fkIPoM\/YFuGb6i7vrFl3EXVxWaaqY3NZNmwDgaIQCL2\/z0TYcbC\/++47+uyzz0QW3aRJk6Jikanc0AOYqdIlUWLyfuutt+iiiy6yOmReD\/CBsOshiqiDKQKh9\/dWCNsUNJ\/kQw+gDpY+dM6ufVDZZ\/J+8skn6Xe\/+13ikPmwYcPE+gzOwFW2qmFuqutCXsemjoxO2\/JZxpc6uvbDlv08drLomujoyqrkQu\/vQdg77STmPvGUAwEMmf8jzqqOrRytAbUsGwIg7MAjHnoAA4e\/UPeZvPnM+0suuSTRj5\/85CfiJrF63CIGwi606aHwghAIvb9Hho0Mu6BXx69imbxvu+02WrhwYeKQ+ZVXXimO2q0X8gZh+9X+4E1tEABh1wZnZ6WEHkAdYHzonF37YMs+EzefqsZn5addAZp0lrmMg6kfLuR1bOrI6LQtn2V8qaNrP2zZz2Mni66Jjq6sSi70\/j5Xhs2Ho6xdu1ZcBsInoG2xxRb03nvvUceOHalp06Y+v8sV30IPoA7IqkasYyOvjGsfbNqXtpiwX375ZZowYUJi9fmMgUMPPbRB1m3qhwt5HZs6MnljXrS+L3V07Yct+3nsZNE10dGVVcmF3t9nImwmaN6DzR0Z39bVq1cvcaIZDxmef\/751LlzZ\/r1r39NrVu3LvqdVZYfegCVFYSAFQSYvDnrTjvLnK+T5S+poQyZqzo2K6DBCBDwDIHQ+\/tMhP3oo4+Kk6N+9atfUadOncT910zYW2+9Nd177700ceJEuvjii8WCHZ2HyT96vOQ555xDo0aNaqA6ZcoUuuGGGxp8NnDgQJo8eXLlcBY+xGXw4MEVmblz51KfPn2quhB6AHXwhYw9BHTOMue2u+eee9or1IElELYDUGHSewRC7++NCZtPORs9erQY9uaO6dlnnyUmUybsdu3aEZ87fv3119MLL7xAV111lTLLZrK+7rrrxJGn3bt3J75oZOjQoYJoJWnLMvv27Svu2U56mKxHjBhRsRP\/Pa0lhR5AnTfEh87ZtQ827evaYvJeuXIl8YK0pIfb8WGHHVY169YtS9rXkbclo9O2fJbRwaEW\/rv2w5b9PHay6Jro6Mqq5ELv740JWxIqHzTB83dMjFHC5heAM3AmYUniaS9FGhHHyVaWyQSelDFLOzwUH83M2S9+4tl61J\/QA1iLDgdlqBFQDZnzTXVbbbWVN0Pmqo5NXWNIAIHwEAi9vzcm7A8\/\/JDOPvtsOv744+mkk05KJOzZs2fTsmXLRKbNw+SmDy9kGzlyJE2dOlVk3fw7X+\/J84ScxcefNEJP+jIR1w09gKbYQt49ArzKnL8sZlll7t67f5QAwq4V0ijHJwRC7++NCZsXnHFnxEPe1157rTglLJph86paXnjG2TdntrwQzfThYXK+P1tm6PE5brYXnb+OE7wsT2dYPPQA6mDrQ+fs2geb9vPYiuoyYa9Zs4YuvfTSxDDxArUzzjiD+vfvrxNGbaLV8V9HRtspTwV9qaNrP2zZz2Mni66Jjq6sSi70\/t6YsPnd5Puthw8fTps2baKdd96Z\/vjHP9IxxxxDf\/vb38R5zXyNIS8Q23HHHY1f5aQ5bP5CsGDBgsr8tBwCZ+O86IxPq4pm5CDshrCrGrFxkDIouPbBpv08ttJ0mbyXLl1KN910UyJ6PGp1yCGHKIfMdXyzJZMhzF6p6OBQC4dd+2HLfh47WXRNdHRlVXKlJGxu5O+++y7NnDmT7r777sr1ms2bN6fjjjuOeH57u+22M34XJBG\/8cYbyvlvzqqHDBlC06ZNE\/du5yFs6Sh3qHiAgGsEXnnlFbrvvvto9erVjYrq0KED\/fznPxdTP\/x\/Vw9\/ye3SpYsr87ALBLxCoF+\/fhV\/Qr47IlOGHY0Erwrn+7C\/\/fZbatu2beYDU0zImsuXmTjPo++99965CDvkAHr1VsAZIwSK3CKmykSMKgJhIBAIAqXNsHkum7PsbbfdVsxTv\/TSSyLb3nLLLUWW3aNHD+0QmpJ1nLAPP\/xwsRUsvooci87+EQIfOmfXPti0n8eWqa6UZ\/LeuHGjOL8g6eH5bm7ffFDRvvvuW\/Xd0vFBR0b7BfZU0Jc6uvbDlv08drLomujoyqrkSknYDMpFF10kyJr3WvNxpDwH99e\/\/lW8um3atKGbb76Z9tprL+WrrCLrtC1b0YVmPLTHe8OxrSsZblUjVgbJgoBrH2zaz2PLVDdJnsmbF3Xyzohq5M0knnSymo4POjIWwl6oCV\/q6NoPW\/bz2Mmia6KjK6uSKx1hy1XivNeaF3z17t2b\/vCHP9Bvf\/tbMe\/MC8741LJWrVqJo0s546728IIyzoSr7dmWJ5jJk8uSFqbFZXRWiLNfoQew0B4RhTtHgMn7scceE6cJZiHvNAdVHZvziqEAIFAAAqH398Zz2B999JHYtjVgwACxDzu+Yrtly5bieNJbb71Ve+EYDwUmPdGjRePHjiYdX4qjSQt4A1BkTRHgkSs+0zyNvPkmMX5UZ5qDsGsaNhTmCQKlI+z4SWdvvvmmWK3NZ3ifeeaZIiwPPPCA2L6iOunMhxiGHkAdDH3onF37YNN+HlumulnlOfPmi0iqkTePgLVo0SLzsLlO2\/JZxhRbV3Vx7Yct+3nsZNE10dGVVcmF3t8bZ9gyo+b915xpL1y4kMaOHSsyap6z5iFznnvjBWncYfDQuM9P6AHUwVbViHVs5JVx7YNN+3lsmerakNchb35Xv\/\/971fI27TcvPEvQt+XOrr2w5b9PHay6Jro6Mqq5ELv740Jm1+8e+65h8aNG0fbb789vf3227T\/\/vuL\/dD88HGifErZ5ZdfnnpRRxEvb1qZoQfQJyzhS\/EIMHn\/3\/\/9nzgWOG3Y\/KCDDhJrN0xOWCu+ZvAACORHIPT+PhNhcxa9ePFi4jnmffbZR1yjySvGOfseP3487brrrnTqqaeK4Tjfn9AD6Du+8K84BJi8ebslf3lOe+R2sbQV58V5j5KBgH0EQu\/vMxG2fRiLsxh6AHWQUw0T6djIK+PaB5v289gy1XUhn2RTdUgLx5dJ+xe\/+AXtsssuykVredtDLfRNsXXlk2s\/bNnPYyeLromOrqxKLvT+Phdhf\/LJJ\/TVV181auecgfOlB3vuuafyPmxXL4mu3dADqFNPVSPWsZFXxrUPNu3nsWWq60JeZZPJm28UW7JkifiZ9vD7y4e1qFac520brvRVOLgqN27XtR+27Oexk0XXREdXViUXen+fibD58g9eaMZ7sdOeXr16YZV4rXoElAMEDBGQHZu8AnT58uXiDvu0h0n7kksuEccA4wECoSJQOsKWB6fwqnDezsXEzIc68HGJfHc1L0jjS0AmTZokTh5r0qSJ17ENPYBegwvnvEUgLRORQ+e8Xaxa9s0Ezqcc8vsdagbubXDgmDMEQu\/vjTNseXAKEzRvFeGHyXmLLbYQw2d8EQifh8y3owwaNMgZ8LYMhx5AHRxUw0Q6NvLKuPbBpv08tkx1Xcjr2NSRkeTNx6TOnj27avYt7\/bmYXRfHp061sJX137Ysp\/HThZdEx1dWZVc6P29MWHHD07hBj9\/\/nziITXed80nnfG2rhUrVlR+r8VLkbWM0AOoU29VI9axkVfGtQ827eexZarrQl7Hpo5MPOYm2Tdv+eT7BvgpKgPPUse87TxJ37UftuznsZNF10RHV1YlF3p\/b0zYMsM+8sgjKxk0Hwk6ffp0MTS+zTbbiLltng\/DSWcuXn\/YBAL5EVB1bDolmBC4jxm4Th0hU18IlI6w5Rw2H0nKQ+FM0Hxz1gUXXCB+32OPPcShDU888USFwH0OeegB9Blb+OYvAjYIO1o7uXiN7d5xxx1V579l1s1nNnD\/UVQG7m904JkrBELv740zbAaSt2yde+654rCUmTNn0lZbbSWut+Rh8Q4dOtDf\/vY34ss5mMT5rmyfn9ADqIOt7c5Zp8y4jGsfbNrPY8tU14W8jk0dmSxxljqSwN966y1xbHG1BWySwPkEtr59+woCt0Hiruuoi49rP2zZz2Mni66Jjq6sSi70\/j4TYXNDXb9+vbizlxeX8RWa\/ILyVZk8HH788cfTsGHDxLdn35\/QA6iDr6oR69jIK+PaB5v289gy1XUhr2NTRyZvzOP6ukPoUo9J+z\/+4z\/EWQ9ZSLyIOiZh5toPW\/bz2Mmia6KjK6uSC72\/z0zYtl\/mouyFHsCicEO5YSOg6thqVTvOvFetWkV8bajqkVn3r3\/9a\/r666\/FwUx4gIAJAqH395kJe8OGDWIInDPtpKdt27Z04oknap10xqvMx4wZUzGTdNe1XJ3OLzc\/uA\/bpJlCFgg0RMAXwo56JYfR+adqH3g0C+f\/19ORqmir7hAoJWH\/7\/\/+L5133nliz3Xao3vSGZM1ryifM2eOOHhFEnOfPn3Evm5+5GcnnXSSWJke\/51leKX6iBEjKnbiv6f5GXoAdZq2D52zax9s2s9jy1TXhbyOTR0ZnbblWkaS+Lp16+jOO+9UzoVHiZyvE5VnRdiYE89aV9dY27Kfx04WXRMdXVmVXOj9vXGGLe\/D3rhxo5iz3nHHHTOfZiZt8UKT6CErcbJlUue93dFtYizD5fNnvPebF73xyWqS5Pnl4r\/zE\/0s\/tKFHkCdTkTViHVs5JVx7YNN+3lsmeq6kNexqSOTN+au9E1XpMezce5veIFbrYbUXWNty34eO1l0TXR0ZVVyoff3xoQts1u+UvO4445z8k7yNrGRI0eKu7U5604iXukHk3GPHj1o6NChgpg5M5dPlNTbtWuX6GvoAXQSABitewRUHVtoAGQZTo8T+cEHH0z77bdfZYV6kVl5aPiH4m\/o\/b0xYfMNXXyCkcujR6MZtcye41l4dFicLySIEnyUsKPD5EmNKvQAhvKiwE+\/EKg3wk5DVxL5yy+\/TPfff7\/2kHqUzJm4OTnhxIAfELlfbdnEm9D7e2PCZnD4go+77rqLrrnmGurYsaMJXkrZ+Bx22rA5CFsJZUXAh87ZtQ827eexZarrQl7Hpo6MfgvzUzKtjtFsnBfNLl261JjIJXEzef\/oRz9qMLIXJ3TXWNuyn8dOFl0THV1ZlVwpCJuzaj69aNOmTeLN5H2RDz30EH3xxRd0xBFHUJs2bRq9sSarxKWyJOc33nijMl8Nws7fGaoacf4S1BZc+2DTfh5bprou5HVs6sioo+q3hGkdmciZbHmrGSci\/\/3f\/y3Ol1Ad+pKEgiTtTrMRPNkAAAo8SURBVJ06iWF2JvVo1m4TOdN6ppWdx04WXRMdXVmVXCkIO76lSqex6a4Sr0bW\/LdaELb0gb9p4wECZUCAM8suXbqUoapW6vjee+8JO++\/\/75IVJ599lnx\/9WrV2e2z6dC8r9\/+Zd\/oQEDBlQW7\/JneOwiwFO48uEdB6E+mYbEbVc2KbOOloFFZ7YRh72yI6DKRMqOj2n9o8PsfALkypUrRXYuPze1F83G5SlvPI\/+2WefYVFcVjCJqBQZdhQfHg7nIXI+jrR169Y5oPuHqoqsWQbbuvLB7EPn7NoHm\/bz2DLVdSGvY1NHJl+rK17blzo+88wzxNeN8sMEzncx8PXD\/GQZbk9CVg7B88+dd96ZjjnmGOJRgejnqojkwSuLromOrqxKrjSEzbd08SH+M2bMqByYcthhh9Gll15K2223naotpP6ds2feflXtKs74QrS0g1MGDx5Mc+fOFQtAcHDK\/w+5qhFnDp6BomsfbNrPY8tU14W8jk0dGYPweinqSx1VfsgsXM6fM5ivvfYaPf7441ZJPZq18\/8lmfPUCA\/JcyLWvHlzI5KXNlV1TGogJjq6siq50hD2okWL6MILL6SePXvSgQceKOZv7r77burdu7cgcb65y\/Th\/dZDhgwhPoQl6ZHky3\/D0aSm6EIeCKQjoOrYgJ2fCESH3tnDd955hxYvXlzJ3vMOwVerdTxb54OqOJOPDtNz+XwgjVzA5xuKpSDsL7\/8ksaOHSsy6+nTp1dWhfPNXHxEKR8rGj2wxLcgVfMn9ACGhDV89QcBtHt\/YuHCk6SsnXf58MhjlPRdEny8XnHC5995wR1PF+y2227UtGnTCtFL4reNTejtXmvRWdL53gwk33XLJ4ydccYZDY4WtQ2yS3uhB1AHGx+yKdc+2LSfx5aprgt5HZto9zpvjh0ZnXjkKcmW\/fhcO\/vESRpf8hTP7GtJ9EnYxPe6y99btWoltuTxF4CuXbvS1ltv3SDbP\/TQQ+mll17KA3ehurkIO43IC62RYeHcceEBAkAACACBbAg0a9aMeI0T\/5QP\/79JkyZicfIWW2wh5sb5kTJR2WylZtd67LHHsisXrFl6wi4YfxQPBIAAEAACQEALARC2FkwQAgJAAAgAASBQLAIg7GLxR+lAAAgAASAABLQQMCLsVatWaRllIdOjSbUNQxAIAAEgAASAQAkR0CLs+OUfOjhlufxDxy5kgAAQAAJAAAiUEQEtwi4jMKgzEAACQAAIAAGfEABh+xQN+AIEgAAQAAJAIAUBEDaaBhAAAkAACACBABAAYSuCxIfDXHzxxeJo1u7duwcQUrgIBLIjED+zf9KkScGeYpgdBWiWEQF5c+SCBQtE9aN3WfiCBwi7SiTk5SQswuelg7B9abbwwxUCfHtet27dBEnL9j9t2rRg7wpwhRPs1h8CfI0z35I2atQoceY6vwvVbpEsAoG6J2zudK644gpxaUm7du0aYMxB4Ss55RO\/HezGG2+k\/v3702WXXUZTp04FYRfRQlFmJgSytvtoYTLjOPnkk0HYmaIApSIQsNH2mRtuv\/12mjx5MrVs2bKIaiSWWdeELYf3uObxb0rx+7LT7s\/m4I8cORKE7U2ThSMqBGy0ey6jWsen8gF\/BwJFIJC37UeHxTEkXsMIRrPn+CEuMih8nysPf8iHh0D4iX4Gwq5h0FBUbgRstXus3cgdChioMQK22j67LYmfucCnq6PrMsOWgeMFM\/zMmzevQYadFoykeQsQdo3fOhSXGQFb7R6ZdeYQQLEgBGy1fem+TOr69u3r1aLLuiTsaJvhhQRxwk4j4aRhcRB2QW8gis2FQNZ2z4XecsstNG7cOK\/m7nKBAeVSIZC17a9cuVLgJBdc+jgVCsKObNUCYZfqva7rymbptGbOnEk33XQTyW0tEiAf5\/LqOnioXC4EsrR93gXUpUsXGj16dKX9+9juQdgKws7VcqAMBApCIGunha2LBQUMxVpDoJ7bPggbhG3tRYEhfxCo507LH5ThiY8I1HPbLyVhmyw687FBwicgoEIgqdNCu1ehhr\/XAwL13PZLSdgm27rqoQGjDuVDIKnTQrsvXzsoY43rue2XkrC5EcttAHJhQdrBKWVs8Khz+AgkdVpo9+HHFTVQI1DPbb+0hB3tvGQT8HFVoLp5QgIINEYgrdNCu0drqXcE6rnt1z1h13vjRP2AABAAAkCgHAiAsMsRZ9QSCAABIAAEAkcAhB14AOE+EAACQAAIlAMBEHY54oxaAgEgAASAQOAIgLADDyDcBwJAAAgAgXIgAMIuR5xRSyAABIAAEAgcARB24AGE+0AACAABIFAOBEDY5YgzagkEgAAQAAKBIwDCDjyAcB8IAAEgAATKgQAIuxxxRi3rFIHNmzfTokWL6KqrrqJXX32VBgwYQFOmTKFWrVrRW2+9RZdccgldeumlovZDhgyhYcOG0aBBgxqhIc8Z5z9MnjyZWrZsWaeIoVpAIFwEQNjhxg6eAwF699136ec\/\/zl16NCBTjnlFGrfvj317NmTmjVrRo8\/\/jjddtttgsyZvEHYaDBAIGwEQNhhxw\/elxyBtWvXJhIxZ95XXnkl\/dM\/\/ROdf\/75lCYn4UOGXfKGhOoHgQAIO4gwwUkg0BgBvuRgzJgxDf4gL7D56KOPBFH\/8pe\/pD59+hgT9v3339\/Itiyoc+fONGfOHOrevTvCAgSAQA0RAGHXEGwUBQRsIvDmm2\/S8uXLadq0aXTcccfRD3\/4Q9p1113F8PiLL75Il19+OV1zzTXUqVMnY8J+77336PXXX2\/gLg+\/8\/z4AQccQP\/5n\/9JrVu3tlkd2AICQECBAAgbTQQIBIxA2lA3Z9\/PPfccTZgwgbbccssKYW\/cuLFqbQcOHJi46OyTTz4Ri9c2bNhA1157LXXs2DFg1OA6EAgTARB2mHGD10BAIJBE2F999RWNHz+e9tprr8qKcCm322670Q9+8ING6H399df0yCOPEP89vkr8m2++oeuuu47mzZtHs2bNot133x3oAwEgUAACIOwCQEeRQMAWAkmEzUPlI0aMoIkTJ1bmmbMuOuPFa3fddZf4AsDZ+gknnEBNmjSx5T7sAAEgYIAACNsALIgCAd8QSCLiRx99VGTDvJ1LzjNnJew1a9bQueeeS0cddZTYw83bxfAAASBQDAIg7GJwR6lAwAoCcSKOb+eShWQh7HfeeYeGDx9O2267LU2aNAmLzKxEDEaAQHYEQNjZsYMmECgcgTgRx7dzZSXsb7\/9Viwye\/7552nmzJnUo0ePwusKB4BA2REAYZe9BaD+QSMQJ+z4dq4shM1z37Nnz6YZM2bQeeedR\/vtt18jjLp27Urbb7990NjBeSAQGgIg7NAiBn+BQASBOGHHt3NlIWy5wGzBggWpWPMQedKZ5AgOEAAC7hAAYbvDFpaBABAAAkAACFhDAIRtDUoYAgJAAAgAASDgDgEQtjtsYRkIAAEgAASAgDUEQNjWoIQhIAAEgAAQAALuEPj\/AJlqxu+pMLPoAAAAAElFTkSuQmCC","height":296,"width":492}}
%---
%[output:189fc40f]
%   data: {"dataType":"textualVariable","outputData":{"name":"V2rms_load_trafo","value":"   6.600000000000000"}}
%---
%[output:103fb8d1]
%   data: {"dataType":"textualVariable","outputData":{"name":"I2rms_load_trafo","value":"       30000"}}
%---
%[output:278a1c18]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     4.666904755831214e+02"}}
%---
%[output:511b73df]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     8.485281374238571e+02"}}
%---
%[output:74d80938]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:9c1cedb7]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000050000000000"],["-4.934802200544680","0.999214601836603"]]}}
%---
%[output:987b375e]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:81e6790f]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     5.000000000000000e-05"}}
%---
%[output:6c239720]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"  -4.934802200544680"}}
%---
%[output:4d4af98a]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.999214601836603"}}
%---
%[output:900457b4]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.076483869223994"],["18.982392004991411"]]}}
%---
%[output:8c43ce75]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:261d01fa]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:516bf83b]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000050000000000"],["-4.934802200544680","0.999214601836603"]]}}
%---
%[output:73bc0081]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.075698471060596","12.858521001586354"]]}}
%---
%[output:11e11d95]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:33f191ae]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:5e9b3e14]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000050000000000"],["-4.934802200544680","0.999214601836603"]]}}
%---
%[output:2df27274]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.075698471060596","12.858521001586354"]]}}
%---
%[output:6036ed4b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.018721899663332"],["0.977712707506129"]]}}
%---
%[output:3a89dea5]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAewAAAEoCAYAAACaU8LCAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ2QXmWV5w+KSoMg6RAMbYxJoAOIMx3IoG0GZDCD6OwmTLm7pjszq5VttbcksA7JppMgVCGYr0qoYZAZs06bZV3THWsFTXZnC5mMMmLIyABpP0DTkC9D8xHTRFHDbDmVrXPD03n69v14\/ve99773ed\/\/W2VJ+j333uf+znnP\/57nPh+nnThx4oTwQwIkQAIkQAIkUGkCp1GwK+0fNo4ESIAESIAEAgIUbAYCCZAACZAACXhAgILtgZPYRBIgARIgARKgYDMGSIAESIAESMADAhRsD5zEJsYTGB0dlZ6ensCgv79fWltbC8M1PDwsS5Yskblz58q6deukpaWlsGvZJy7zHl1uyHC46aabZNGiRS6HVNpm\/fr1snnz5qCNvb290tfXB7V327ZtsmrVKlm7dm0leej97d69u\/DfBwSNxpkIULAzYeNBVSFQppjFCbYmxGuuuUY6OzsLwRJ3j0VfN+5msgiACsYjjzwCiWGWY1AH6DUWL148dlgjCnajPWChPm4kewp2I3mT91IqgePHj8vKlStlx44dsnXr1tIEWyv7Mq4bBdMk\/wULFjiLr6lAETHMckwW5xvBRtoWvk7VK2wTp4cOHWKVnSVIKnQMBbtCzqh3U+yuQW2L3cVnV5dz5syRO++8M2iuJm67e9hUg0NDQxO+t89xww03yCc\/+cnApq2tTbZs2SLt7e2RCGxhDNuHq0\/93nSRz58\/X+6++27p6OgIEpUtdGnn0a51c90nnngiaJ9+TJf47bffLp\/\/\/OcDsTafMAv9u2FqC7oRCdveJH1zLltA7Hv84he\/KBs2bIi87uHDh4P2jYyMjLXJ9qHNUZnfd9998pWvfEXM\/Sn\/MGvDzrxqiBIn41f7uuZ+w\/dlfD1t2rSxh44wv+3btwddzOZjx0f4fGkPSuF4TDpXUhwmVeI2E22zaXv4ISD8+7LZmnPccsstsnPnTtHfj\/Gdfc\/6N3MN27dpXKLisN75htfHCVCwcWYNd0Q4Sds3aJJOVFIOJ1o9j4qlEevw91GCkiR2+l1c20xynTx58rh32Eaw7TaoMEYJrC3a4fPkJdhRFVw4eYYTeRxX\/XucYK9YsUKWLl06gX2SQOp3U6ZMkSNHjgQPJFEiqte0hSXc9ri4MNd98sknI8X3gQceGHtvbMebLUhhwQ6fy3wfJ9pJMavHHDx4MPbBwG5TWKzDD1VGLK+++mr53ve+Ny4\/RIlu1O8rLLhqE9VG\/bu5Ttq5bS5V7wVouKRa0A1RsAsC69NpTUKyKwyT7PQ+7OpSqyiTCOyEaCcXu7KwE7yKoqkAzTnMtcOVnOEX9b2dfK677rpYwbYrEPQ8aYKtvQr6SeuaTuoB0Kr\/6NGjE5jYVaFymj179rh7dOkSD3fXG\/bGn1pNh\/2ubdH3uVGVv7JcuHBhcL92Re4yEM+leztsE\/63YWIeLrT9adc2sRd1P+Zv+mCn9xzXJW5zNPEU9unDDz8cCH9UxRx33nAvi+lVsM+RdG1TgZv4T+OSR9e\/TzmtUdtKwW5UzwL3Fff0bRKeJqorrrgicoS0bXPgwIHIqkmbYp9Dqzozojtt0FhaZRAniHYC0+uj58lLsO1rq\/jqxxaIuERqC9anPvUpZ8GO6pGIuq62I9zlH1fBqq0Kj2mHzTbqeuFXA0mCHfcqIHxMUrUc9bAXvjfzuiUs\/OYhJU5Y0+LT9q99jji\/hqt1w8oIdtyrEHsGhB3L5ndpv44wP32bS9RrGCBF0LQiBCjYFXFEPZtRtmDb06LSEiIqtMoxapqX63lsMQondz23Pa3LpcJWG3uglv5bpxCFexjCgoEKdphjuAoPPyjkJdgmbuMeFHTkfJRgh7vW0ypsHwQ7qkfH+DUcf3EVtn2OuN8GBbue2bK+16Zg15d\/Ja6etUs83HVr3gnGVStRXZhpgp3UlW1XfQpSq5A4wXY9j3Y1hsXUvCoIC7aKostgnrCY2RVo+LWCClxal7hW\/2mCFz5H1i5xO0DjqtZwEIfFN1xtmnu2e1rM\/ZjYCR8T1SWe9uMpukvcPNyZnok4wY7qmTCMwhV23CDBcHd8Upd4FBd2iadFix\/fU7D98FOhrSx60FmS4KUJdlLbot7vxgl22nm0+9C8jw7DdhFsPSZqlLg5V3ikr73gCDLozHSN2sfodT\/60Y8G1X\/URzlF3Z\/roDM9p3mICT8oxA3Iso+xbfSa99xzj9x1110TBsjpMWHB1r\/FDWAz95r2gBjVXZzWw2FzjLvHJLG1BfLmm2+Oja2kc2gbogajuQ46s7mk9TAVmmB48twIULBzQ+n\/iVynddlTstKmdUUNZEO6xJVqUndr2qAue+WzpPPodcJTgG677TbZs2fP2CCrqArbTuZxA+fsc4ffrUcJui1c9rH630awo6775S9\/edyKXbqYi\/2+PMu0Llt4bQGJmvIXN50szNV+p67nVG4bN26U5cuXBzjsnhIz2j9umlja\/OmkaV16LdfKM+7ds\/ayRIlhXK+CMoqaUhdVpcc97OnfwyurxY0FMOdw6QnyP4M1\/h1QsBvfx7ncYdqI3FwuwpMURiCq6z08EyBuHrzdqCwLpxR2Uw1+YvsByzyYRI0cT8PAhVPSCPnzvXeCrQlD55zq4hFRCSaqikp7CvfHXfVrKQW7fuzzuHLSK4Gkrvyoa2dZmjSPe2jGc0R1iSuHtMWGoh6yGmXt92aMA3PPXgm2y6AX7QJbtmyZrF69OnblrGZ2eNZ7p2BnJVed48Ldw9oyVKz1GK5NXa5Pw6+qELHWlvIBq1x\/FXk1rwRb3wtp8OknrsLWZLJmzRrZtGlToTs3FekUnpsESIAESIAEwgS8EWytDu644w7p7u4ORDtOsI2oF73VIkOJBEiABEiABMok4I1g67sc\/ejKPknvsMPvfJJG7pYJmtciARIgARIggVoIeCHY2s19\/\/33y6233iq6gUSSYGv1rVMnzA5S4X+HYc2aNasWfjyWBEiABEjAEwK6E9rMmTM9ae3EZnoh2Cq6OqdUV4FKGyUevsU0exXsffv2eevAshtOXhhx8iIvjABmzfhqLl6VF+yoka3GRWl74apd2iA0BnxzBTx2t7VbM74whuRFXhgBzNr3+Kq8YCMVs5n2NW\/ePNGlH82\/dRpEX19fpGd9dyAWrrVbkxfGkLzICyOAWTO+mouX94JtRFlHj2uXedJGC1GuZcBjAb9\/\/36v3wFhd1u7NXlhDMmLvDACmLXv+d47wcbck27tuwPT7zBfCyZUjCd5kRdGALNmfGG8fM\/3FGwOOoMingkCwiXkRV4YAcya8YXxomBjvCpn7bsDywbKBIERJy\/ywghg1owvjJfv+Z4VNitsKOKZICBcrLAxXORFXiABzJyCjfGqnLXvDiwbKAUbI05e5IURwKwZXxgv3\/M9K2xW2FDEM0FAuFgxYrjIi7xAApg5BRvjVTlr3x1YNlAKNkacvMgLI4BZM74wXr7ne1bYrLChiGeCgHCxYsRwkRd5gQQwcwo2xqty1r47sGygFGyMOHmRF0YAs2Z8Ybx8z\/essFlhQxHPBAHhYsWI4SIv8gIJYOYUbIxX5ax9d2DZQCnYGHHyIi+MAGbN+MJ4+Z7vWWGzwoYingkCwsWKEcNFXuQFEsDMKdgYr8pZ++7AsoFSsDHi5EVeGAHMmvGF8fI937PCZoUNRTwTBISLFSOGi7zICySAmVOwMV6Vs\/bdgWUDpWBjxMmLvDACmDXjC+Ple75nhc0KG4p4JggIFytGDBd5kRdIADOnYGO8KmftuwPLBkrBxoiTF3lhBDBrxhfGy\/d8zwqbFTYU8UwQEC5WjBgu8iIvkABmTsHGeFXO2ncHlg2Ugo0RJy\/ywghg1owvjJfv+Z4VNitsKOKZICBcrBgxXORFXiABzJyCjfGqnLXvDiwbKAUbI05e5IURwKwZXxgv3\/M9K2xW2FDEM0FAuFgxYrjIi7xAApg5BRvjVTlr3x1YNlAKNkacvMgLI4BZM74wXr7ne1bYrLChiGeCgHCxYsRwkRd5gQQwcwo2xqty1r47sGygFGyMOHmRF0YAs2Z8Ybx8z\/essFlhQxHPBAHhYsWI4SIv8gIJYOYUbIxX5ax9d2DZQCnYGHHyIi+MAGbN+MJ4+Z7vWWGzwoYingkCwsWKEcNFXuQFEsDMKdgYr8pZ++7AsoFSsDHi5EVeGAHMmvGF8fI937PCZoUNRTwTBISLFSOGi7zICySAmVOwMV6Vs\/bdgWUDpWBjxMmLvDACmDXjy53Xo88ek49+flBe\/h\/\/2f2gilmywmaFDYUkEwSEixUjhou8yAsk4G4+8PiLcuPAMzJ697XuB1XMkoJNwYZCkoIN4aIAYbjIi7xAAu7mFGx3VpW1ZJc45hoKNnlhBDBrxhd5YQTcrSnY7qwqa0nBxlzDhEpeGAHMmvFFXhgBd2sKtjurylpSsDHXMKGSF0YAs2Z8kRdGwN16\/UMHZP1D+\/kO2x1Z9Swp2JhPmFDJCyOAWTO+yAsj4G5NwXZnVVlLCjbmGiZU8sIIYNaML\/LCCLhbU7DdWVXWkoKNuYYJlbwwApg144u8MALu1hRsd1aVtaRgY65hQiUvjABmzfgiL4yAuzUFO4bV6Oio9PT0yNDQkDtNEeno6JAHH3wQOqZWYwo2RpAJlbwwApg144u8MALu1hTsFMHu6+uTzs5OJ6K7d++W9evXU7CdaNXPiAkVY09e5IURwKwZX+68dJUzndrFlc5CzEyFTcF2DyZfLJkgME+RF3lhBDBrxpc7Lwq2O6vKWrJLHHMNEwR5YQQwa8YXeWEE3K0p2Cld4voOu7e3V7TSruqHgo15hgmVvDACmDXji7wwAu7WFOwUVvpOevPmzYFVW1ubbNmyRdrb290Jl2BJwcYgM6GSF0YAs2Z8kRdGwN2agu3IKjxqvEpVNwXb0YmvmzGhkhdGALNmfJEXRsDdeuF9T8mjzx3joDN3ZCLDw8OyZMkSGRkZqUTVTcFGvCfc\/hDDRV7kBRLAzPmA486Lgu3OKtLSTOXq7++X1tbWGs828XB9OFixYoVs2LAhtiuego1hZ4IgL4wAZs34Ii+MgLs1Bdud1ZilXWHrH7du3eo8Vxu53PHjx2XlypXyxBNPJL47p2AjVFlhY7TIi7xQApg9H3Dcec256zE5NPoau8TTkBnx3LFjR2C6YMECWbdunbS0tKQdmvl7U73rCVhhZ8Y44UAmCIwleZEXRgCzZny582q95TuBMRdOiWFmjxIvspoOX14Hud1xxx3S3d0drJ5GwXYP6jRLJog0QuO\/Jy\/ywghg1owvd14U7BhW9qjwMqrpcDO2bdsW\/OmKK65weodtjt+5c6e795vU8vDhwzJt2rQmvXv8tskLY0Ze5IURSLeeP3++\/O68i+XXV61ghR2F69lnn5Wbb75Zbr\/9duf303mtJa7vyO+\/\/3659dZbRX\/8HHSWHtCIBZ\/oEVp8h43RIi\/yQgm42esa4joPWz\/sEg8xq+da4toFfs011wQPChwl7hbMiBUFG6FFAcJokRd5oQTc7M1OXRTsCF712l4z6bpxo9E5Stwt4I0VBZu8MAKYNeOLvDACbtZmStcbfvsL+cWX\/oPbQRW0Ou3EiRMnKtiuXJrECjsXjONOwoSKMSUv8sIIYNaMLzdeZsDZm176kbz0tZvdDqqgFQV71izZt29fBV1TzSYxQWB+IS\/ywghg1oyvdF6PPntMFv71U4HhOd\/ukwM\/\/kH6QRW1aGjBdmHOLnEXSqdsmCDICyOAWTO+yAsjkGytC6UsHXgmWEN8eusZ8quv\/JnXBRoFmxU29PtgQoVwcS1xDBd5kRdIINncHh3ee\/U02bbsegp2roRLPhkrbAw4BZu8MAKYNeOLvDAC8dZaXWtXuP6\/VtfbP3O5\/NEfvJuCnRfgepyHgo1RZ0IlL4wAZs34Ii+MQLx13wN75cuPPh8Y\/NcPzZBVH54pvuf70rrEdfWxVatWBfB0itXBgwdl165dha8pnuZ83x2Ydn95f8+EihElL\/LCCGDWjK9oXvZAM62u93zu\/YGh7\/m+FMHWxUx0\/2tddWzp0qXS19cnHR0dwW5abW1twb\/r9fHdgWVzY4LAiJMXeWEEMGvG10Re9ntrFesvdl0qV110LgXbJbTsVc9mz54tPT09gUDrSmRF74ft0j4KtgulUzZMEOSFEcCsGV\/khRE4Za3vqr+7d1Q++\/WfBX8MizUrbAeyFGwHSB6ZMKFiziIv8sIIYNaMr1O8zGpmcWJNwXaMLX1\/re+r7S5xU213dXXJokWLHM+UvxkrbIwpEwR5YQQwa8YXeWEEJBgFPueux8YOMyPC9f\/DH9\/zfSnvsBWadn8vXrx4HL+1a9fWVawb4YkLDe5a7ZlQMYLkRV4YAcy6WeNLRTpYFGXwmeD\/zSdJrBsh35cm2FgYlmft+xNXeaROXqlZE0RWzuSFkSMv8kojoCPAo4TaHlwWdw7f8z0Fmyudpf0+xn3PhArh4gMOhou8yCuSgFbRB0ePy02DP51QUS++8gJZcf0MJ3IU7BRMrltt9vb21mV6l+8OdIrSHI0o2BhM8iIvjABm3cjxZbq6zVrgNhnt+l7w+1PkzoUXQcB8z\/elVNg66GxwcFD6+\/ultbU1AGyEXAedLVy4sG5zsn13IBStORg3coLIAc+EU5AXRpW8mptXkkgrGRXq\/\/XpDrno\/DMxUK9b+57vCxdse1qXzr22P\/Y87L1794ousPLggw9mckTWg3x3YNb7znocEypGjrzICyOAWTdKfNm7aoUJqEhPn3SGrLh+5tgCKBilU9a+53sKNt9hQ7HfKAkCuukajMkLg0dezcFLB44N\/vMLsvUHL8becNTCJxididYUbAeCaV3iOg\/bzNW+5557HM6Yn4nvDsyPhNuZmFDdOBkr8iIvjABm7Ut8aQW98eED8j\/\/6YVEgTaVdFBVR8yjxuhQsDPzipqHrZuAaDe5ivW9994rW7Zskfb29szXyHIgBRuj5kuCwO6qOGvywtiSV2Pw0gp6w0P75dHnjiXekIry6o\/MlI\/NnYrdeEZr3\/N94V3iGbmWdpjvDiwN1OsXYkLFiJMXeWEEMOt6x5cZJLbp7w\/KI3tHx025iroTFehPvL9N\/mL+u7Abzcna93xPweY7bOinUO8EATW2AsbkhTmBvKrLS6tmFVydZnXolZMrjSV9TJf2zddOlz++dHIhXdwYLW6v6cRreHhYlixZEmyxGf7oNpv2dC+nE+Zo5PsTV44onE7FhOqEacyIvMgLI4BZFxFfRoi\/NfSyPPz00dRubdNiFeh\/83tTpPfqacGfingHjdGZaO17vi+8wj5+\/Hgwx3revHlj8627u7slvNVmrY7IerzvDsx631mPKyJBZG2LD8eRF+Yl8iqXl1bN\/3Tgl\/LIz0YhYdZWfviy8+Qz17yzsuIcRdL3fF+4YIfnYetc6xkzZgSbfuhAtIGBAVm3bp20tLRgkZqTte8OzAmD82mYUJ1RBYbkRV4YAczaJb5Mxbz+of3y89HXnIXZVMk6cvvPOy+QzpnneiXOFGwslgLrsGDriPADBw4Ey5DaC6eYFdAyXKKmQyjYGD6XBIGdsbGtyQvzL3ll46WVsn72HP6VfPsnR53eMdtXMouTXDz1LLmh4\/ygO7uKXdoYnYnWvuf7witsRaZVtX7CIv3www8H+2Szwq41DMs7ngkVY01e5IURiLc2g77+9vvPy55Dv4JF2a6YdaT2v7vi7cHFtAJvRHFmhZ0x8uz32NoVrgK+efNmaWtrq8vca\/s2fH\/iyuiSzIdRgDB05EVergRM1\/WzR34r33jyJbj72lzHiO9HLjsvGAQ2JtQFLETiem9VsfM935dSYVfFWY34xFU2WwoQRpy8yMsQMIKs\/\/+\/f3REnh75daYq2Qiw\/n\/H20+XT\/7RRXLaaacF62w3U7WMRdZJawp2CjXXzT\/4DjtL+JV\/DAUIY05ezcHLnpOs\/61Ton724m8yC\/JYVTzpDLHfK9tirf\/N+MLii4JNwcYixnNrJgjMgeTVOLyMKO85\/Ko89JNfBF3WLguIxBEwXdd\/eOG58qF3nyeXv\/PsMVPXd8qMLyy+KNgxvHQ0+KpVq1Jp9vb2BoPR6vXx3YFlc2OCwIiTlx+87Ar5r\/7hkOx9qbbq2K6EVZCvbp8k82ad6rJ2FeQ0eoyvNELjv\/c93xf+DjupSxxDXYy17w4shkr8WZkgMOLkVQ1eRpC3D70s3376aNCotI0p0lpupkK9s\/UM+be\/N0Uua3srXCGnXSPte8ZXGiEKNkao4tYUbMxBTBDkhRHArNH4MlOdDhw9Lt\/c87I8+\/Jva+qmNq01FbAuGnLtJa1y5bveNjb1Ka\/qGCMTbY3yyuOaPp\/D93xfeIVddef67sCy+TJBYMTJKzsvrYr1f6\/97l\/lwadervmdcZQYX9r2VrnRWl7Tt1HWjC8svnzP94UItukGHxoaSqXJzT9SEVXKgAkCcwd5TeRlquJXX\/ud\/LdHD8v+I8dz6aLWk9iV8bvb3hp0VY\/9rQHnITO+sN8jBRvjVTlr3x1YNlAmCIx4s\/CyB20dHD0uDz19VH7481dzE2JbjK+6aJJ0znybzJh8cv+BZp5\/3Czxhf3q4q19z\/eFVNh5wS3jPL47sAxG9jWYIDDivvOyF\/t4x7lvka8\/8ZJ8\/9lXChFifV98\/hn\/Kt1\/OEvazz9z3CIgVXpvjEVAsda+x1exdCae3fd8X5pgR03zWrt2bbBrVz0\/vjuwbHZMEBjxKvMy74jffPpp8tXdL8jBo8dzGbBlCNnd0++a3CIfvLhVppz95sSquMq8MM+XY01eGGff830pgq1iPTg4KP39\/WJWNDPvubu6uuoq2r47EAvX2q2ZIDCGZfMyIqxi+Y2nXpLv\/HQ0aHAtC3yE79gW4plTzpSPzX27nDhx0qrW98Vl88K8WT1r8sJ84nu+L1ywuTQpFlBVt2aCwDyUFy8zUOvHI7+W\/\/OjI7mNmLbvxhbiC6ecKR9+z3ly1pvfWLMII8Ty4oVc02db8sK8R8FO4UXBxgKq6tZMEJiHkniZavhtLadL\/\/efl31Hfpt7NWyqXn0\/rJ8PzNYBW+cGImymMFXp\/TDjK7\/4ws7UHNYUbAc\/s0vcAZInJkyoyY4yItx61pvkvz\/2vDy1\/6i0nHFGrl3S47qeJ50h0ye3yHWXTpbJZ71prFu6SiKMhDbjC6HFzT8wWtyty5kXB505o6q0YbMlVHuU9JlvfoPs+OEReeLgrwqrhAMxnnSG6HKXv\/+Os4PlLsPi66sYuwR2s8WXC5MkG\/LCCLLCxnhVztp3B5YNtBEShD1nWN8J\/98f\/yIYIa2fWteXDvuj7ZzT5fTTTx8T4TnvPFsunXpShKvYJV12PIWv1wjxVSZD8sJo+57vSxt0Vu\/R4HFu9d2BWLjWbl3VBGEGZekdfnX3iIz88l8KGZgV7o7W6UqXTz9bZp9\/1hjcYFOI11fVqiqv2iOhmDOQF8aVvDBevuf7wgVbcYa7w7du3SqdnZ0Y6YKsfXdgQVhiT1tmgjDvg896yxvl\/sdGChuUFRbh2W8\/S66ZPUkmnXnqnbBtgzAvkxfSrqrakhfmGfLCePme70sRbBvp+vXrZfPmzcGf2traZMuWLdLe3o5Rz9HadwfmiMLpVFkThN0Nrf+99+XfyJOHXpVDOS\/WYd9EeL7wH1\/SKh3Tzg5M7C5ppxvPaJSVV8bLeX8YeWEuJC+Ml+\/5vnTBDov37t27xy2oguGv3dp3B9ZOADtDOEHY2xv+\/TNHZc\/r60fnuVCHaaEtwDooa+o5bwkqYV1T2t5lqUqDsphQa4sv7Ojms2Z8YT73Pd+XLth2hV3vnbrU1b47EAtXN2u7Gn7gqZdl+OXflPI+eNaUM+VPO6bI6W98w7iR0VUSYDeCp6yYUDFi5EVeGAHM2vd8X4pg19oNfvz4cVm5cqXs2LEj8E5vb6\/09fVFeipsm2bvuwOxcD1pbapifS\/8+IFf5jpH2K6C9Vpz33WOfLyzTZ4\/9i\/BtZttZyUKEBah5EVeGAHM2vd8X7hgJ6105opaBV8\/KtJpa5Dr98uWLZPVq1c7vRv33YFxDFWUd+8\/Jv+495VcBNkI8ZQWkQ9eNlW6r5wqh19pThF2jVu1owAhtMgLo0VeKC\/f833hgo0CdbG3BTxsPzw8LGvWrJFNmzaNbTSSdE7fHajC\/Oprv5O\/eeTnmeYQ2xXx+2a+Tf5jZ1uAK25QFgXIJUJP2ZAXeWEEMGvGF8bL93zvnWCnVew6iE0F3d4ZrJEEWwV64PEXZODxF50i1Qiybm344cvOkzNf38wh63thJggn7GNG5EVeGAHMmvGF8aJgY7xqsjbvwhcsWCDr1q2TlpaWCecLz\/lOG9imDjSfnTt31tS+og7+5k9elb\/72W\/kiedfi72Erqh1wdmny1UzWuSyt79F5r7j5GYPeX8OHz4s06ZNy\/u0DXs+8sJcS17khRFIt54\/f\/44o3379qUfVFEL7yps5ajCPTIyEina4e+SbPVcVXzi0lHaB0ePy02DPw2mK0V9tEL+7AffJR+8pDX4OmvFjMYln+gxYuRFXhgBzJrxhfGqYr5H7qBwwS5ie019T71ixQrZsGFD6sCyNNuqOfDGgWciu7tVkD911bRg4Q8daV2vDxMERp68yAsjgFkzvjBeVcv3WOtFvBRs5D112iC0qjhw4X1PTRg0piLd9QdTZfF7Lyitgk4LICaINELjvycv8sIIYNaML4xXVfI91upT1oUJdtR2mlGNTJpTbeztUeFmnrUuaxqei22+mzdvnixatEiSbM256+lA7e7+7t5R+ezXfzYOjQr1F7surWslHRdQTBDYT428yAsjgFkzvjBe9cz3WEujrQsTbHO5tFHdLjcRXgzFHnRmvuvu7g42FEmyjbpWvRyoo72XDj4z7h11lYXasGOCcInYUzbkRV4YAcxh42SnAAAa0klEQVSa8YXxqle+x1oZb124YOfV0KLOUw8HPvT0Uen+2x+O3ZIPQk3BzhaBTKgYN\/IiL4wAZl2PfI+1MNmagj1rlpQ1zF+7wJcOPDPuXfU3ejvk2otPjvT24cOEinmJvMgLI4BZM74wXhTsCF52N\/js2bOlp6dHhoaGIsmmzZPG3IFbl+VAFeuFf\/3UWBe4VtXbP3N5ZQaTuZJjgnAlddKOvMgLI4BZM74wXmXle6xV7tassEuosBtFrClA7j8sY8mEijEjL\/LCCGDWFGyMV+Wsi3ZglFjv+dz7K8fBtUFMqK6kWGFjpMiLvLIQwI4pOt9jrcGtC6+wTfd4M3aJh99Zaze4z2LNChv\/gfEBB2NGXuSFEcCsKdgYrzFrdBvMjJdJPaxIB9qrljWCWFOwU8NpggEFCGNGXuSFEcCsi8z3WEuyWRdeYSc1S1csGxgYiN3II9stYUcV5UCdZ62DzPTj6wCzKJJMqFh8kRd5YQQwa8YXxquofI+1Irt13QUb2Qoz+23GH1mEA7UrfM5dj42JdVVXLcvCkwkCo0Ze5IURwKwZXxivIvI91oLarOsq2Gk7adV2a25H5+3A8HtrnbpVz8063Ci4WzFBuLNSS\/IiL4wAZs34wnjlne+xq9duXbhgJw060\/XAt2zZkrrjVu23WV6FPfD4i6LvrvXTfeVUua\/70iKbX\/q5mSAw5ORFXhgBzJrxhfGiYDvwituUw2zS4XCKwkzydGC4K9zHhVHSQDNBpBEa\/z15kRdGALNmfGG88sz32JXzsS68wtZmRnV9m8q7q6sr2FmrXp88HWhvkanTt3SwWaN9mCAwj5IXeWEEMGvGF8Yrz3yPXTkf68IFO2m3rkYaJR6urn2fbx0XXkwQ2A+PvMgLI4BZM74wXhTsFF5pgt0oo8Tt6rrRBprZLmaCwBIEeZEXRgCzZnxhvCjYKbzC769t823btsmuXbu8n4dtz7m+6sJzZfuNl2NR5JE1EwTmLPIiL4wAZs34wnhRsB14adf38uXLx40IHx4eliVLlsjGjRuls7PT4SzFmOThwGaprtUDTBBYHJIXeWEEMGvGF8Yrj3yPXTFf68LfYZvmqmgvXrx4XOu3bt1aV7HWxtTqQPvddaNX1xRs\/MfHhIoxIy\/ywghg1rXme+xq+VuXJtj5Nz2fM9bqwGaqrinYeMxRgDBm5EVeGAHMutZ8j10tf+vCBdu8w+7u7q57NR2FrxYHNlt1TcHGf4AUIIwZeZEXRgCzriXfY1cqxrpwwU4aJV7MLWFnrcWBX939gvyXr\/80uGCjzrsO02RCxeKLvMgLI4BZM74wXrXke+xKxVgXLtja7CqMBo\/Dl9WBzTLvmoJd2w+PCRXjR17khRHArLPme+wqxVkXLthJa4nrbXV0dEh\/f7+0trYWd5cJZ87qQHsqV9\/1M6Xv+hl1aX\/ZF2VCxYiTF3lhBDBrxhfGK2u+x65SnHXhgl1c0\/M5c1YHNsMypFGEmSCwuCMv8sIIYNaML4xX1nyPXaU4awr2rFmyb98+iHAzDjYzgJggoFDhvHUMF3mRF0gAM6dgR\/CyB5rNnj1benp6ZGhoKJKsj13idne4bp+p22g2y4eCjXmavMgLI4BZM74wXhRsjFflrLM40HSH625cjbrJR5yjmCCwECYv8sIIYNaML4xXlnyPXaFY61K6xBttP+zWW74TeKUZVjYLhx8TBPaDJC\/ywghg1owvjBcF24FXI+2HPfD4i3LjwDPBXTfyrlyssB0C28GECdUBkmVCXuSFEcCsKdgpvNK21xwYGPBqt65m7g5XVzOhYgmCvMgLI4BZM74wXhTsGgXbp\/2wm3l0uHEzEwSWIMiLvDACmDXjC+NFwU7h1Uj7YduC3UyLpdguZoLAEgR5kRdGALNmfGG8KNgOvBplP+xmXSyFgu0Q5DEmTKgYO\/IiL4wAZk3BduTVCPthz7nrMdEquxmnc7FL3DHQQ2YUIIwbeZEXRgCzpmBjvCpn7epAuztcF0rRBVOa8cOEinmdvMgLI4BZM74wXq75HjtredalzMMu73bwK7k60F7drBmnc7HCxmNLj2BCxbiRF3lhBDBr13yPnbU8awq241ri9vvr0buvLc9DFbsSEyrmEPIiL4wAZs34wnhRsDFelbN2cSCnc51yGxMEFsLkRV4YAcya8YXxcsn32BnLtWaF7VBhczoXBTvrz5IJFSNHXuSFEcCsKdgYr8pZuzhw\/UMHZP1D+4O262YfOkq8WT9MqJjnyYu8MAKYNeML4+WS77EzlmtdSoU9PDwsS5YskZGRkQl358P2mnx\/zQo768+SCRUjR17khRHArCnYKbzMSmdtbW3S19eH0S3B2sWBZneuZp5\/bVzBhIoFJXmRF0YAs2Z8Ybxc8j12xnKtC6+wkzb\/KPdWo6+W5kD7\/fVffuxi+XhnWxWaXbc2MEFg6MmLvDACmDXjC+OVlu+xs5VvXbhgmwq7u7tbOjs7y7\/DlCumOdAW7Gaef80KO1voMqFi3MiLvDACmHVavsfOVr514YKtt6TLktZ7V644tGkOtAecUbC5EAj6E6UAYcTIi7wwAph1Wr7Hzla+deGCbbrEh4aGIu+u6oPOmn3\/67DTmFCxHyl5kRdGALNmfGG8KNgYr8pZJzmQC6ZMdBcTBBbC5EVeGAHMmvGF8aJgY7wqZ53kQHv9cN3sQzf9aPYPEwQWAeRFXhgBzJrxhfGiYDvy2rZtm6xatSqw3rp1qxw8eFB27dol69atk5aWFsezpJuZQW47duwIjHt7exOnkyU5kO+vWWGnR1yyBRMqRpC8yAsjgFlTsB146YAzXTRlxYoVsnTp0kBA9d31ypUrJe\/52Xot\/eg1zPvzrq4uWbRoUWRLkxzIBVMo2A7hnWhCAcIIkhd5YQQwawp2Ci97Hvbs2bOlp6cnEFOd4lXG6HFbwKOa6iLYXDDlFDkmVCxBkBd5YQQwa8YXxouCXWHBdlm0Jc6BHHAW7VgmCCxBkBd5YQQwa8YXxouC7cBL31\/r+2q7S9xU20nd1Q6njjXRynrz5s2yYMGCxPfk6kDz2blz59h\/P\/H8a\/LpB14M\/t373nPl0+87t5bmNMyxhw8flmnTpjXM\/RR9I+SFESYv8sIIpFvPnz9\/nNG+ffvSD6qoReHzsM19a\/f34sWLx2FYu3Zt7LvlvHiZ9+dxg9vinri4Qxcr7DxikBUQRpG8yAsjgFmzwsZ4lW6tO4VpZb9hwwZpb2+fcH0KNuYSJlTywghg1owv8sIIYNYUbIxX6dZpA9viHMgVzlhh5xGsFCCMInmRF0YAs6ZgO\/Ky52GbQ3Q+dt4bgtijwl229oxzoNlS86oLz5XtN17ueJeNb8aEivmYvMgLI4BZM74wXhRsB14q1oODg9Lf3y+tra3BES5zpB1OPcEkvHCKy6Cz8CAEe4T4vV2XyJ+994IsTWnIY5ggMLeSF3lhBDBrxhfGi4KdwitpalVadzXmimzWUQ7kkqTxLJkgsDgjL\/LCCGDWjC+MFwW7wQWbW2qOdzATBJYgyIu8MAKYNeML40XBduCllfTy5ctly5YtYyO1i+oSd2jOOJMoB3JJUlbYaBzF2TOhYiTJi7wwApg1Bduxwo7bD9s+XNcXf\/DBBzEP1GidJNhcknQiXCZULODIi7wwApg14wvjRcHGeFXOOsqBHCHOCjuvQGVCxUiSF3lhBDBrCjbGq3LWYQfaI8T7rp8pfdfPqFyb69kgJlSMPnmRF0YAs2Z8Ybwo2I68ouZhl7E0aVrzwg60R4hzwBm7xNPiJ+17JtQ0QuO\/Jy\/ywghg1hRsB15lzsN2aM44k7AD7TXEKdgUbDSewvYUIIwgeZEXRgCzpmCn8PJtHvbqbw7Ll\/7xcHBXez73ftGBZ\/ycIsCEikUDeZEXRgCzZnxhvCjYDSbYXEM82aFMEFiCIC\/ywghg1owvjBcF24GXT13ic+56THTgGdcQj3YsE4RDwFsm5EVeGAHMmvGF8aJgO\/LyYdCZPUKcgk3BdgztRDMmVIwieZEXRgCzpmBjvCpnbTuQU7rS3cOEms7ItiAv8sIIYNaML4wXBRvjVTnrOMG+r\/tS6b5yauXaW+8GMUFgHiAv8sIIYNaML4wXBRvjVTlr24Gc0pXuHiaIdEassDFG5EVe2QlgR1KwMV6Vs7YdeOPAMzLw+ItBGzmlK9pVFGwshMmLvDACmDXjC+NFwcZ4Vc7adiCndKW7hwkinRErRowReZFXdgLYkRRsjFflrG0Hmild3KUr3k0UbCyEyYu8MAKYNeML40XBxnhVzto4kFO63FzDBOHGyViRF3lhBDBrxhfGi4KN8aqcdZRg\/9WiS+TP33dB5dpahQYxQWBeIC\/ywghg1owvjBcFG+NVOWvjQHuXLk7pYpd4XoHKhIqRJC\/ywghg1hRsjFflrI0DOaXLzTVMqG6c2CWOcSIv8spGADuKgo3xqpy1ceDXfvCC3DT406B9nNLFCjuvQOUDDkaSvMgLI4BZU7AxXpWzNg40U7q0gaN3X1u5dlalQUyomCfIi7wwApg14wvjRcHGeFXOOizYnNKV7CImCCyEyYu8MAKYNeML40XBxnhVzto4kNtqurmGCcKNk7EiL\/LCCGDWjC+MFwUb41U5a3Xgd\/\/5aVHB1g+31WSFnWeQMqFiNMmLvDACmDUFG+NVOeuwYPddP1P6rp9RuXZWpUFMqJgnyIu8MAKYNeML40XBxnhVzjos2Ns\/c7lcddG5lWtnVRrEBIF5grzICyOAWTO+MF4UbIxX5azVgV\/Ytkt0py79ULDZJZ5nkDKhYjTJi7wwApg1BRvjVTlrdWDv3\/yDrH9oPwXbwTtMqA6QLBPyIi+MAGbN+MJ4UbAxXpWzVgf+yZq\/k60\/eCFoG+dgs8LOM0iZUDGa5EVeGAHMmoKN8aqctTrwPcu+IY8+d0w4BzvdPUyo6YxsC\/IiL4wAZs34wnhRsDFelbNWB57zn74mur0mBTvdPUwQ6Ywo2Bgj8iKv7ASwIynYGK\/KWasDj\/1pf9AuzsFOdw8FO50RBQhjRF7klZ0AdiQFG+NVOesZ73mv\/OpD64N23XTtdLljwYWVa2OVGkTBxrxBXuSFEcCsGV8YLwo2xqty1tPf+xH59VUrgnZx0ZR09zBBpDNixYgxIi\/yyk4AO5KCjfGqnLUt2Pd1XyrdV06tXBur1CAKNuYN8iIvjABmzfjCeFGwMV6Vs7YFm4umpLuHCSKdEStGjBF5kVd2AtiRFGyMV+Ws2\/7kL+S1SxYG7aJgp7uHgp3OiAKEMSIv8spOADuSgo3xqpz11H9\/l\/y\/6X8YtIuLpqS7h4KdzogChDEiL\/LKTgA7koKN8aqc9fkf\/5L87ryLOQfb0TMUbEdQr5uRF3lhBDBrxhfGi4KN8aqcNQUbcwkTBHlhBDBrxhd5YQQwawo2xqty1q23fCdoExdNcXMNE6obJ2NFXuSFEcCsGV8YLwo2xqty1kawdTqXTuviJ5kAEwQWIeRFXhgBzJrxhfGiYGO8KmdtBJuLpri5hgnCjRMrbIwTeZFXNgLYURRsjFflrCnYmEso2OSFEcCsGV\/khRHArCnYGK\/KWRvB5hxsN9cwobpxYsWIcSIv8spGADuKgo3xymQ9OjoqPT09MjQ0FBy\/YMECWbdunbS0tEw43\/Hjx2XlypWyY8eOse96e3ulr68v8toUbMwlFGzywghg1owv8sIIYNYUbIwXbG0EeN68ebJo0SIx\/25ra4sUYRX3ZcuWyerVq6W9vT31ekaw93zu\/cFcbH6SCTChYhFCXuSFEcCsGV8YLwo2xisX623btsmuXbsiq+zh4WFZs2aNbNq0SVpbW1OvZwSbq5ylogoMmCDcOBkr8iIvjABmzfjCeFGwMV65WCcJ9u7du2X9+vXS39\/vLNhaWWuFzU86ASaIdEa2BXmRF0YAs2Z8Ybwo2Bivmq3N++yurq6gizz8UTFftWrV2J87OjoSxVsr7Df89hdyzrf7ZOfOnTW3r9FPcPjwYZk2bVqj32Zu90deGEryIi+MQLr1\/Pnzxxnt27cv\/aCKWpx24sSJExVt24RmmffX+kXcoDOtrkdGRsa+D\/87fFIVbK5y5h4BfKJ3Z6WW5EVeGAHMmvGF8WKFjfHKbO0i1lEn13faK1askA0bNkQOQqNgYy5hgiAvjABmzfgiL4wAZk3Bxnhlsk4bGZ500rRBaCrYXOXM3S2+B7z7neZjSV4YR\/IiL4wAZu17fHnRJZ7WrW1chk4B0+Mo2M0V8Njd1m7te4KonQB2BvIiL4wAZu17fFVesMOLphj3mMFkuniKLpTS3d0tnZ2dY\/O0zcIpSYusGME+88mvyJsPfR\/zPK1JgARIgAS8I8BBZ965jA0mARIgARIgAb8IVL7C9gsnW0sCJEACJEACxRCgYBfDlWclARIgARIggVwJULBzxcmTkQAJkAAJkEAxBCjYxXDlWUmABEiABEggVwIU7Fxx8mQkQAIkQAIkUAwBCnYxXHlWEiABEiABEsiVQNMKti7Gsnnz5gDm1q1bgznc\/IjoynBLliwJ1mNPm8NuM9T9ybds2eK0B3kjcUZ4mfsOL\/DTSDxc7gVhFl6HoRl\/qwgv27ZZf5NJMWh+e2bdDpd4rZJNUwq2vQXn3r17oe04q+S8vNtiC8nChQuDBWnmzZsXuyuavSe57pI2ODjovK1p3m2vx\/kQXnb7zI5ya9eujWRbj3sp65oIs\/CSxGn7ApR1D2VeB+FlHm76+vqCAqQZf5MuYq2Lavn64NeUgq2VoX40sH1\/4sozeYQToj7YDAwMxO6MZl+7GZNpFl6aVJctWybHjh2TuC1i8\/Rp1c6FMEvbB6Bq91ZEe1Be9kZHzfibjPOB6XmYO3euHDp0KMj9PvaqNp1gx603HldJFvEjrOo57Z6H1tZWCf87qd3NmByy8NKHxSuvvFK+9a1vxfZeVDU+8mgXwkwrRLsXJ4\/r+3YOhFdUhd3s\/Iy\/n3\/++eA\/dSnrnp4eCrYvP4SoilqT6IwZM5quezLss3BFjVQ4rhu0+BInLu1EeSnP+++\/X2655Ra54447mlaw7V6bpBhTwT5w4EDgimYdb4LGmMlv2u3b29sbCBM\/pwiEH2p8Y9O0FbY96ICCfTJs0eRggl0T67333tt0g84QXppIv\/CFL8gnPvEJmTZtWuL4AN+SCNJehJl512\/eN+qxy5cvb6o4Q3iZbt+NGzcG3b1IDxniQ59tKdieeY9d4vEOQ7rfml2szQOOPuz19\/dL2isEZfvII4+MGzfRjK9hkBgLd4k34+h68spXYCjY+fIs5Wx2Rc1BZ6eQh7sn0wadNfsoVISXPQXODvJm67ZEmIXjrxl\/qwgvPuCkywcFO51R5Sw4rSvaJcgUkmbsngxTQ3jZxzZjpWjuH2EWTq7N2MWL8IrqEm+2VwhpYkPBTiNU0e+5cEq0Y5IWaTCDgHQgS1zF6Ov8xqxh6sqLgj2+JyducR47xvQIe+GUZl0IBIkxfahZvHhxALtZeSX9linYWTMdjyMBEiABEiABEnAm0HSjxJ3J0JAESIAESIAEKkSAgl0hZ7ApJEACJEACJBBHgILN2CABEiABEiABDwhQsD1wEptIAiRAAiRAAhRsxgAJkAAJkAAJeECAgu2Bk9jEcgk899xzMmnSpGD1MpePThV55ZVX5MILL3Qxz2RjptF1dHRAW5j6simLub+ypiKVfb1MTudBJBAiQMFmSJCARQBdnKOMeZ21iG4tx5YZGPaWt2Vd1xc2ZfHgdapPgIJdfR+xhSUSqKJgo22ycfkiShTsEoOcl\/KWAAXbW9ex4VkJ2Ktn6TlMN\/PevXvHVonSv5tV28Kruplu28mTJwd76w4NDQVNMeuC21sc2udP6mK3r2F3C5sdq8y9rl27NnIb2Dg7I9gLFy6UO++8MzhNuNvZXh0rfB1ltWzZMvnABz4QHG9YHT16VMxqZXrMbbfdJtu3b5cNGzZIe3t7cJq4e4ryW1iw9d+vvvpq8D\/dKtLmG3V81N7Zaftp+\/IwkzXOeVzjEaBgN55PeUcJBKLW8bbFIlzNxm2ooJdYt26d6PlUtHW5Vt3S0DwMdHV1jQlr0iYppj3mfC0tLRLerjStwg7b22tK60OFCuvcuXOD9przDw4OBu\/CVXhXrFgxTmjt85mHkunTp48dH75H8+8jR44EW1+a7UP1wcDsx5y29nyUYOse2OYBJYqr7WYKNn\/2zUCAgt0MXuY9jhFIS\/xp4qgnssUhLNhRO5wlbfYRVeWF7ZPalLaRSHhDCG1\/WmVpf28EO\/wAsmvXrjEB13Pagqz\/XrNmjWzatGncwL2kbu8owR4ZGZlwDXs7Uwo2f9jNRoCC3Wwe5\/0Ggrtq1aqARHjUdZw4hruNFyxYEFlhh7umbdxR3dlx17M3wUgS7LRBb1HiHPW38GuCcLe\/6UEwXd36\/6Z6Dj8EaNVuNqAIh1vcdqJRgp10DdPtbs7PCps\/7GYgQMFuBi\/zHiMJ2CJlv8e2qzgjwOH3yqbCDFfYemy4MkzCHyfGSd309vlqFWw9l3kXbR4ooipsRLCffPJJMV3urlPjKNj8kZJAOgEKdjojWjQ4AVv0TAWp73f1fe\/KlStl3rx54wZ62aIcFuyk99VRGMvoEg+\/o7avqeKa1L1tusRtwY6qZu0uca2w0X2Yi+gST3t4Sns10OBhz9vzkAAF20OnscnZCUS9w7arXHsQVtTgKVNxmy5xbYkt6ub8OgDNdBlHvUc2d5DXoDO7orXfa19xxRUTBpWFBds+1rRV26cDyKIE23XQmZ7DDHRLGztQ66Az88rCjOw392EPtgtHDQU7+++IR9aHAAW7Ptx51ToSMMlcu671Y3d321OytIv4uuuuGzd1S4X6hhtukNtvvz0QNH2XGhZxU3Wb6V56DSMkcbedNAXKdSCceS+v14jq3jbvfcNCFb72xo0bgylZOtDM3L9dYev5wwzD07rCU9vsNkUxcJnWZY8BiBJbe\/yA+mnOnDmyZ8+e4KEh\/GBl7iHc+1DHsOSlSSCVAAU7FRENSIAE0giogEaNDE87znzv8g7b9VyudqywXUnRrioEKNhV8QTbQQKeEAi\/pzfVtD3vGr0VCjZKjPbNSICC3Yxe5z2TQI0Ewqu\/xU3Xcr1MeDOOBx54IDjUnjrmei4XO27+4UKJNlUj8P8BU9Ushuf94d4AAAAASUVORK5CYII=","height":296,"width":492}}
%---
%[output:9437bd2f]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:86f2bfce]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:534048a4]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
