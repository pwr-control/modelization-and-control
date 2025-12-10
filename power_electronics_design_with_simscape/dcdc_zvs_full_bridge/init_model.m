%[text] ## dabGeneral Settings
%[text] ### Simulink model initialization
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 1.5;
transmission_delay = 125e-6*2;
s=tf('s');

model = 'dcdc_zvs_full_bridge';
use_thermal_model = 1;

% simulation voltages
number_of_cells_1_sim = floor(900/3.6) %[output:0994022b]
number_of_cells_2_sim = floor(360/3.6) %[output:60f8fa48]
%[text] ### Voltage application
application400 = 0;
application690 = 0;
application480 = 1;

% number of modules (electrical drives)
n_modules = 1;
%[text] ### PWM and sampling time and data length storage
fPWM = 12e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

TRGO_double_update = 0;
if TRGO_double_update
    ts_afe = 1/fPWM_AFE/2;
    ts_inv = 1/fPWM_INV/2;
    ts_dab = 1/fPWM_DAB/2;
else
    ts_afe = 1/fPWM_AFE;
    ts_inv = 1/fPWM_INV;
    ts_dab = 1/fPWM_DAB;
end
ts_battery = ts_dab;
tc = ts_dab/200;

z_dab=tf('z',ts_dab);
z_inv=tf('z',ts_inv);
z_afe=tf('z',ts_afe);

% t_misura = simlength - 0.98;
t_misura = 0.6;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

Pnom = 175e3;
ubattery1 = 750;
ubattery2 = 540;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery1;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = ubattery2;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:6c7481ff]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:6f1d1799]
%[text] ### dead\_time and delays
dead_time_DAB = 400e-9;
dead_time_AFE = 0;
dead_time_INV = 0;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;
%[text] ### Grid emulator initialization
grid_emulator;
%[text] ### Nominal DClink voltage seting as function of the voltage application
if (application690 == 1)
    Vdc_bez = 1070; % DClink voltage reference
elseif (application480 == 1)
    Vdc_bez = 750; % DClink voltage reference
else
    Vdc_bez = 660; % DClink voltage reference
end
%[text] ### ADC quantizations
adc12_quantization = 1/2^11;
adc16_quantization = 1/2^15;
%[text] ## HW design and settings
%[text] ### DAB
%[text] #### Input filter or inductance at stage 1
LFi_dc = 400e-6;
RLFi_dc = 5e-3;
%[text] #### DClink input stage or capacitor at stage 1
Vdc_ref = Vdc_bez; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = Pnom;
Rbrake = 4;
CFi_dc1 = 900e-6*4;
RCFi_dc1_internal = 1e-3;
CFi_dc2 = 900e-6*4;
RCFi_dc2_internal = 1e-3;
%[text] #### Tank LC and HF-Transformer parameters
magnetics_design_transformer; %[output:6f08adf0]
m1 = n1;
m2 = n2;
m12 = m1/m2;

% Ls_dab = L_d_eff;
Ls_dab = 12e-6;

f0 = fPWM_DAB/5;
Cs_dab = 1/Ls_dab/(2*pi*f0)^2 %[output:6c8d3181]

Ls1_dab = Ls_dab;
Cs1_dab = Cs_dab*2;
Cs2_dab = Cs1_dab/m12^2;
Ls2_dab = Ls1_dab*m12^2;

mu0 = 1.256637e-6;
mur = 5000;
lm_trafo = mu0*mur*n1^2*S_Fe/L_core_length * 1e-2;
rfe_trafo = V1^2/P_Fe;
rd1_trafo = P_Cu/I1^2/2;
ld1_trafo = Ls1_dab;
rd2_trafo = rd1_trafo/m12^2;
ld2_trafo = Ls2_dab;
%[text] #### DClink Lstray model
Lstray_module = 100e-9;
RLstray_dclink = 10e-3;
C_HF_Lstray_dclink = 15e-6;
R_HF_Lstray_dclink = 22000;
Z_HF_Lstray_dclink = 1/s/C_HF_Lstray_dclink + R_HF_Lstray_dclink;
Z_LF_Lstray_dclink = s*Lstray_module + RLstray_dclink;
Z_Lstray_dclink = Z_HF_Lstray_dclink*Z_LF_Lstray_dclink/(Z_HF_Lstray_dclink+Z_LF_Lstray_dclink);
ZCFi = 7/s/CFi_dc1;
sys_dclink = minreal(ZCFi/(ZCFi+Z_Lstray_dclink));
% figure; bode(sys_dclink,Z_Lstray_dclink,options); grid on
%[text] ## Active Front End (AFE)
%[text] ### LCL switching filter
if (application690 == 1)
    LFu1_AFE = 0.5e-3;
    RLFu1_AFE = 157*0.05*LFu1_AFE;
    LFu1_AFE_0 = LFu1_AFE;
    RLFu1_AFE_0 = RLFu1_AFE/3;
    CFu_AFE = (100e-6*2);
    RCFu_AFE = (50e-3);
else
    LFu1_AFE = 0.33e-3;
    RLFu1_AFE = 157*0.05*LFu1_AFE;
    LFu1_AFE_0 = LFu1_AFE;
    RLFu1_AFE_0 = RLFu1_AFE/3;
    CFu_AFE = (185e-6*2);
    RCFu_AFE = (50e-3);
end
%%
%[text] ## Control system design and settings
%[text] ### Single phase inverter control
flt_dq = 2/(s/(2*pi*50)+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d); grid on
% [num50 den50]=tfdata(flt_dq_d,'v');
iph_grid_pu_ref = 2.5;
%[text] ### DAB Control parameters
kp_i_dab = 0.2;
ki_i_dab = 5;
kp_v_dab = 1;
ki_v_dab = 45;
%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:6789dfdf]
Iac_FS = I_phase_normalization_factor %[output:3f404ad4]

kp_afe = 0.15;
ki_afe = 18;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);
res_min = s/(s^2 + 2*delta*omega_grid_min*s + (omega_grid_min)^2);
res_max = s/(s^2 + 2*delta*omega_grid_max*s + (omega_grid_max)^2);

Ares_nom = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Aresd_nom = eye(2) + Ares_nom*ts_afe %[output:63740541]
a11d = 1 %[output:28fc6eb0]
a12d = ts_inv %[output:5c0f6b69]
a21d = -omega_grid_nom^2*ts_inv %[output:1b40e88d]
a22d = 1 -2*delta*omega_grid_nom*ts_inv %[output:94cef1af]

Bres = [0; 1];
Cres = [0 1];

Ares_min = [0 1; -omega_grid_min^2 -2*delta*omega_grid_min];
Ares_max = [0 1; -omega_grid_max^2 -2*delta*omega_grid_max];

Aresd_min = eye(2) + Ares_min*ts_afe;
Aresd_max = eye(2) + Ares_max*ts_afe;
Bresd = Bres*ts_afe;
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
%[text] ### Single phase pll
freq = 50;
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:2df16908]

freq_filter = 50;
tau_f = 1/2/pi/freq_filter;
Hs = 1/(s*tau_f+1);
Hd = c2d(Hs,ts_inv);
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso_pll = [-1 -4]*omega_rso;
Lrso_pll = acker(Arso',Crso',polesrso_pll)';
Adrso_pll = eye(2) + Arso*ts_afe;
polesdrso_pll = exp(ts_afe*polesrso_pll);
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:179dae94]

%[text] ### PLL DDSRF
use_advanced_pll = 0;
use_dq_pll_ccaller = 0;
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for voltager grid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:56f14b27]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:71d3202f]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:8d48ca25]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:5494b171]
%%
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
%%
%[text] #### RMS filter
rms_perios = 1;
n1 = rms_perios/f_grid/ts_afe;
rms_perios = 10;
n10 = rms_perios/f_grid/ts_afe;
%%
%[text] #### Butterworth filter
omega_c = 2*pi*5;
P1 = s^2 + 0.7654*omega_c*s + omega_c^2;
P2 = s^2 + 1.8478*omega_c*s + omega_c^2; 
Hb_flt = omega_c^4/P1/P2; 
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
%[text] ### 
%[text] ## Lithium Ion Battery 1
typical_cell_voltage = 3.6;
number_of_cells_1 = floor(ubattery1/typical_cell_voltage)-1; % nominal is 100
number_of_cells_2 = floor(ubattery2/typical_cell_voltage)-1; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 50; %Hr*A

Vbattery1_nom = ubattery1;
Pbattery1_nom = Pnom;
Ibattery1_nom = Pbattery1_nom/Vbattery1_nom;
Rmax_battery1 = Vbattery1_nom^2/(Pbattery1_nom*0.1);
Rmin_battery1 = Vbattery1_nom^2/(Pbattery1_nom);

Vbattery2_nom = ubattery2;
Pbattery2_nom = Pnom;
Ibattery2_nom = Pbattery2_nom/Vbattery2_nom;
Rmax_battery2 = Vbattery2_nom^2/(Pbattery2_nom*0.1);
Rmin_battery2 = Vbattery2_nom^2/(Pbattery2_nom);

E_1 = -1.031;
E0 = 3.485;
E1 = 0.2156;
E2 = 0;
E3 = 0;
Elog = -0.05;
alpha = 35;

R0 = 0.035;
R1 = 0.035;
C1 = 0.5;
M = 125;

q1Kalman = ts_inv^2;
q2Kalman = ts_inv^1;
q3Kalman = 0;
rKalman = 1;

Zmodel = (0:1e-3:1);
ocv_model = E_1*exp(-Zmodel*alpha) + E0 + E1*Zmodel + E2*Zmodel.^2 +...
    E3*Zmodel.^3 + Elog*log(1-Zmodel+ts_inv);
figure;  %[output:5bf03e7d]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:5bf03e7d]
xlabel('state of charge [p.u.]'); %[output:5bf03e7d]
ylabel('open circuit voltage [V]'); %[output:5bf03e7d]
title('open circuit voltage(state of charge)'); %[output:5bf03e7d]
grid on %[output:5bf03e7d]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink
heatsink_liquid_2kW; %[output:78eda2d1] %[output:9a166e04] %[output:1ec537e3]
%[text] #### SKM1700MB20R4S2I4
danfoss_SKM1700MB20R4S2I4; % SiC-Mosfet full leg
dab_mosfet.Vth = Vth;                                  % [V]
dab_mosfet.Rds_on = Rds_on;                            % [Ohm]
dab_mosfet.Vdon_diode = Vdon_diode;                    % [V]
dab_mosfet.Vgamma = Vgamma;                            % [V]
dab_mosfet.Rdon_diode = Rdon_diode;                    % [Ohm]
dab_mosfet.Eon = Eon;                                  % [J] @ Tj = 125°C
dab_mosfet.Eoff = Eoff;                                % [J] @ Tj = 125°C
dab_mosfet.Eerr = Eerr;                                % [J] @ Tj = 125°C
dab_mosfet.Voff_sw_losses = Voff_sw_losses;            % [V]
dab_mosfet.Ion_sw_losses = Ion_sw_losses;              % [A]
dab_mosfet.JunctionTermalMass = JunctionTermalMass;    % [J/K]
dab_mosfet.Rtim = Rtim;                                % [K/W]
dab_mosfet.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
dab_mosfet.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
dab_mosfet.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
dab_mosfet.Lstray_module = Lstray_module;              % [H]
dab_mosfet.Irr = Irr;                                  % [A]
dab_mosfet.Csnubber = Csnubber;                        % [F]
dab_mosfet.Rsnubber = Rsnubber;                        % [Ohm]
dab_mosfet.Csnubber_zvs = 4.5e-9;                      % [F]
dab_mosfet.Rsnubber_zvs = 5e-3;                        % [Ohm]

danfoss_SKM1700MB20R4S2I4; % SiC-Mosfet full leg
mosfet.dab.Vth = Vth;                                  % [V]
mosfet.dab.Rds_on = Rds_on;                            % [V]
mosfet.dab.g_fs = g_fs;                                % [A/V]
mosfet.dab.Vdon_diode = Vdon_diode;                    % [V]
mosfet.dab.Rdon_diode = Rdon_diode;                    % [Ohm]
mosfet.dab.Eon = Eon;                                  % [J] @ Tj = 125°C
mosfet.dab.Eoff = Eoff;                                % [J] @ Tj = 125°C
mosfet.dab.Erec = Eerr;                                % [J] @ Tj = 125°C
mosfet.dab.Voff_sw_losses = Voff_sw_losses;            % [V]
mosfet.dab.Ion_sw_losses = Ion_sw_losses;              % [A]
mosfet.dab.JunctionTermalMass = JunctionTermalMass;    % [J/K]
mosfet.dab.Rtim = Rtim;                                % [K/W]
mosfet.dab.Rth_switch_JC = Rth_mosfet_JC;              % [K/W]
mosfet.dab.Rth_switch_CH = Rth_mosfet_CH;              % [K/W]
mosfet.dab.Rth_switch_JH = Rth_mosfet_JH;              % [K/W]
mosfet.dab.Lstray_module = Lstray_module;              % [H]
mosfet.dab.Ls = Ls;                                    % [H]
mosfet.dab.Ld = Ld;                                    % [H]
mosfet.dab.RLs = RLs;                                  % [Ohm]
mosfet.dab.RLd = RLd;                                  % [Ohm]
mosfet.dab.Irr = Irr;                                  % [A]
mosfet.dab.Ciss = Ciss;                                % [F]
mosfet.dab.Coss = Coss;                                % [F]
mosfet.dab.Crss = Crss;                                % [F]
mosfet.dab.Cgd = Cgd;                                  % [F]
mosfet.dab.Cgs = Cgs;                                  % [F]
mosfet.dab.Cds = Cds;                                  % [F]
mosfet.dab.Rgate_internal = Rgate_internal;            % [Ohm]
mosfet.dab.Csnubber = 2*Eon/Voff_sw_losses^2;          % [F]
mosfet.dab.Rsnubber = 1;                               % [Ohm]
% Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2
% Rsnubber = 1/(Csnubber*fPWM_DAB)/5


%[text] ## ZVS constraints 
idab_zvs_min = 2*mosfet.dab.Coss*Vdab1_dc_nom/dead_time_DAB        % [A] %[output:5e70c90c]
%[text] ## C-Caller Settings
open_system(model);
% Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
%[text] ## 
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
% 

%[text] ## Enable/Disable Subsystems

% if use_thermal_model
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge ideal switch based', 'Commented', 'on');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge mosfet based with thermal model', 'Commented', 'off');
%      set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge ideal switch based', 'Commented', 'on');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge mosfet based with thermal model', 'Commented', 'off');
% else
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge ideal switch based', 'Commented', 'off');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge mosfet based with thermal model', 'Commented', 'on');
%      set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge ideal switch based', 'Commented', 'off');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge mosfet based with thermal model', 'Commented', 'on');
% end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":6.2}
%---
%[output:0994022b]
%   data: {"dataType":"textualVariable","outputData":{"name":"number_of_cells_1_sim","value":"   250"}}
%---
%[output:60f8fa48]
%   data: {"dataType":"textualVariable","outputData":{"name":"number_of_cells_2_sim","value":"   100"}}
%---
%[output:6c7481ff]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.050925925925926e+02"}}
%---
%[output:6f1d1799]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:6f08adf0]
%   data: {"dataType":"text","outputData":{"text":"--- INITIAL ELECTRICAL PARAMETERS ---\nNominal Power (Sn): 347.82 kVA\nNominal Primary Voltage (Vn): 682.00 V\nNominal Primary Current (I1n): 510.00 V\nNominal Secondary Current (I2n): 194.00 V\nNominal Frequency: 12.00 kHz\n----------------------------------------------------\nCore Section Area (S_Fe): 85.3353 cm^2\nPrimary Turns (n1): 3\nSecondary Turns (n2): 9.090909e+00\nPrimary Copper Area (A_Cu1): 170.00 mm^2\nPrimary Copper Band Length: 17.00 cm\nSecondary Copper Band Length (L_b2): 12.93 cm\nCore Height (AM-NC-412 AMMET): 29.00 cm\nCore Width (AM-NC-412 AMMET): 6.00 cm\nCore Length (AM-NC-412 AMMET): 95.00 cm\nCore Dept (AM-NC-412 AMMET): 14.22 cm\nSpecific Core Loss (AM-NC-412 AMMET): 16.00 W\/kg\n----------------------------------------------------\n--- LOSS ESTIMATION ---\nCore Mass (M_Fe): 63.23 kg\nCore Loss (P_Fe): 1011.74 W\nCopper Loss (P_Cu): 68.74 W\nTotal Losses per Phase (P_tot): 1080.47 W\n----------------------------------------------------\nEstimated Efficiency (Eta, cos(phi)=0.95): 99.67 %\n----------------------------------------------------\n--- LEAKAGE INDUCTANCE AND REACTANCE ESTIMATION ---\nCalculated Leakage Inductance (Ld_calc): 0.000009 H (8.87 uH)\nEffective Leakage Inductance (Ld_eff): 0.000012 H (11.53 uH)\nLeakage Reactance (Xd): 0.869 Ohm\nEstimated Short Circuit Voltage (Vcc): 65.02 %\n----------------------------------------------------\n","truncated":false}}
%---
%[output:6c8d3181]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs_dab","value":"     3.664684014841500e-04"}}
%---
%[output:6789dfdf]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:3f404ad4]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:63740541]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000083333333333"],["-11.843525281307226","0.998429203673205"]]}}
%---
%[output:28fc6eb0]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:5c0f6b69]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     8.333333333333333e-05"}}
%---
%[output:1b40e88d]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":" -11.843525281307226"}}
%---
%[output:94cef1af]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.998429203673205"}}
%---
%[output:2df16908]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.031062519151360"],["1.619345474049183"]]}}
%---
%[output:179dae94]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.125263346003807"],["30.829381225835562"]]}}
%---
%[output:56f14b27]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:71d3202f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:8d48ca25]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000083333333333"],["-8.224670334241132","0.998691003061004"]]}}
%---
%[output:5494b171]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.129590696960579"],["22.638405094998713"]]}}
%---
%[output:5bf03e7d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAB4AAAAeCAYAAAA7MK6iAAAAAXNSR0IArs4c6QAAAjVJREFUSEvtVi2PKkEQbCQW\/gAGTUIQBPcEEiwBRwhBgXiET4PjM5xBIQgOgsHwAy44LBYUCY4sFrmX6qQ3s\/t4u8vC3SWXGwUzPVVT3dUz69N1XSebMRgMaLfb0Ww2I03TKJ\/PU7lcpkwmY7fNcc2naZoeCAQIBNPplDekUinq9\/vk9\/t5HsSj0YhqtRrt93uOKZVKVCwWqVAomOYajQavr1YrarVaRiywF4sFxeNxul6v5INiVRUiAYYAgNgpxlooFGL1QgTwYDDImYEAwRDiSCRCzWaTWLFKJKedTCY0n89pvV7bphqnV1WD+HQ6sVpRiIzlcjn+L4fyHQ4H3Vo3nN6JOJ1O88k3mw0DYgg4iGV\/OBzmg8uaxHlWHI1GTUZTwV0p9lrjZDJpeKFSqZjUqzW2rhk1lnZSXQ3HijtVc8Hlkl4YJxaLUafT4TQD8HK5GK3mytWOTfdgwPF4NJVB9QxqjsHt9CCuq3A1g9jQ6\/VMl85TxCq4FdjpdJ6J4eLlcsk33Pl8pnq9TsPhkCSVn0asAqOm3W6XxuMx4fp1MzwrFnBJtzXVgb\/v\/\/Bf3\/4Yc08TA+l2u3GbJRIJ16\/WS4hBDuUY0v9O6fZMDHNtt1smkocCv\/GquRmeiUWlvOFf1k7\/UyWPxb1LQ93zlGIrOVJerVap3W7zkl2LvZQYamEyfJ\/Jg5LNZu\/W\/eXEcptBsV2L\/Rzib0n1t5kLdVXbSb4y77XeS2vs5saSmF\/iR7L1VOwH+FUrwd61FWEAAAAASUVORK5CYII=","height":30,"width":30}}
%---
%[output:78eda2d1]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:9a166e04]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:1ec537e3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
%[output:5e70c90c]
%   data: {"dataType":"textualVariable","outputData":{"name":"idab_zvs_min","value":"  12.299999999999999"}}
%---
