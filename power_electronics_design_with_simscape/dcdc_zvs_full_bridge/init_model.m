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

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:573defed]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:9c485e2a]
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
magnetics_design_transformer; %[output:95f45fb2]
m1 = n1;
m2 = n2;
m12 = m1/m2;

% Ls_dab = L_d_eff/2;
Ls_dab = 12e-6;

f0 = fPWM_DAB/5;
Cs_dab = 1/Ls_dab/(2*pi*f0)^2 %[output:3bd175ac]

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
Vac_FS = V_phase_normalization_factor %[output:1dee7c80]
Iac_FS = I_phase_normalization_factor %[output:3e86cd16]

kp_afe = 0.15;
ki_afe = 18;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);
res_min = s/(s^2 + 2*delta*omega_grid_min*s + (omega_grid_min)^2);
res_max = s/(s^2 + 2*delta*omega_grid_max*s + (omega_grid_max)^2);

Ares_nom = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Aresd_nom = eye(2) + Ares_nom*ts_afe %[output:3afa214b]
a11d = 1 %[output:0f70846b]
a12d = ts_inv %[output:4d2d82be]
a21d = -omega_grid_nom^2*ts_inv %[output:7d759bac]
a22d = 1 -2*delta*omega_grid_nom*ts_inv %[output:8963de89]

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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:67ef2d7d]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:90bbebb9]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:89c004ba]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:54943e0d]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:3f18a767]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:93c5b888]
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
figure;  %[output:226c4982]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:226c4982]
xlabel('state of charge [p.u.]'); %[output:226c4982]
ylabel('open circuit voltage [V]'); %[output:226c4982]
title('open circuit voltage(state of charge)'); %[output:226c4982]
grid on %[output:226c4982]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink
heatsink_liquid_2kW; %[output:85a19b40] %[output:4142069f] %[output:851aa2d0]
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
idab_zvs_min = 2*mosfet.dab.Coss*Vdab1_dc_nom/dead_time_DAB        % [A] %[output:3b26f23c]
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
%[text] ### Simulation settings
% scenario = 0; nominal conditions
% scenario = 1; voltage input at minimum, voltage output at maximum
% scenario = 2; voltage input at maximum, voltage output at minimum
% maximum input current = 270A
% maximum output current = 270A

scenario = 0;

if scenario == 1 %[output:group:98e69b1b]
    number_of_cells_1_sim = floor(500/3.6)
    number_of_cells_2_sim = floor(860/3.6)
    i_in_dab_pu_ref = 0.65;
    i_in_dab_ref = i_in_dab_pu_ref * Idc_FS;
elseif scenario == 2
    number_of_cells_1_sim = floor(900/3.6)
    number_of_cells_2_sim = floor(360/3.6)
    i_in_dab_pu_ref = 0.54;
    i_in_dab_ref = i_in_dab_pu_ref * Idc_FS;
else
    number_of_cells_1_sim = floor(750/3.6) %[output:69c3b6d1]
    number_of_cells_2_sim = floor(560/3.6) %[output:76434bd2]
    i_in_dab_pu_ref = 0.65;
    i_in_dab_ref = i_in_dab_pu_ref * Idc_FS;
end %[output:group:98e69b1b]
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
%   data: {"layout":"onright","rightPanelPercent":49.9}
%---
%[output:573defed]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.050925925925926e+02"}}
%---
%[output:9c485e2a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:95f45fb2]
%   data: {"dataType":"text","outputData":{"text":"--- INITIAL ELECTRICAL PARAMETERS ---\nNominal Power (Sn): 347.82 kVA\nNominal Primary Voltage (Vn): 682.00 V\nNominal Primary Current (I1n): 510.00 V\nNominal Secondary Current (I2n): 194.00 V\nNominal Frequency: 12.00 kHz\n----------------------------------------------------\nCore Section Area (S_Fe): 42.6677 cm^2\nPrimary Turns (n1): 6\nSecondary Turns (n2): 12\nPrimary Copper Area (A_Cu1): 170.00 mm^2\nPrimary Copper Band Length: 17.00 cm\nSecondary Copper Band Length (L_b2): 12.93 cm\nCore Height (AM-NC-412 AMMET): 29.00 cm\nCore Width (AM-NC-412 AMMET): 6.00 cm\nCore Length (AM-NC-412 AMMET): 95.00 cm\nCore Dept (AM-NC-412 AMMET): 7.11 cm\nSpecific Core Loss (AM-NC-412 AMMET): 16.00 W\/kg\n----------------------------------------------------\n--- LOSS ESTIMATION ---\nCore Mass (M_Fe): 31.62 kg\nCore Loss (P_Fe): 505.87 W\nCopper Loss (P_Cu): 72.90 W\nTotal Losses per Phase (P_tot): 578.77 W\n----------------------------------------------------\nEstimated Efficiency (Eta, cos(phi)=0.95): 99.83 %\n----------------------------------------------------\n--- LEAKAGE INDUCTANCE AND REACTANCE ESTIMATION ---\nCalculated Leakage Inductance (Ld_calc): 0.000035 H (35.48 uH)\nEffective Leakage Inductance (Ld_eff): 0.000046 H (46.13 uH)\nLeakage Reactance (Xd): 3.478 Ohm\nEstimated Short Circuit Voltage (Vcc): 260.07 %\n----------------------------------------------------\n","truncated":false}}
%---
%[output:3bd175ac]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs_dab","value":"     3.664684014841500e-04"}}
%---
%[output:1dee7c80]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:3e86cd16]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:3afa214b]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000e+00","8.333333333333333e-05"],["-1.184352528130723e+01","9.984292036732051e-01"]]}}
%---
%[output:0f70846b]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:4d2d82be]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     8.333333333333333e-05"}}
%---
%[output:7d759bac]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"    -1.184352528130723e+01"}}
%---
%[output:8963de89]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"     9.984292036732051e-01"}}
%---
%[output:67ef2d7d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["3.106251915135997e-02"],["1.619345474049183e+00"]]}}
%---
%[output:90bbebb9]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["1.252633460038073e-01"],["3.082938122583556e+01"]]}}
%---
%[output:89c004ba]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:54943e0d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:3f18a767]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","8.333333333333333e-05"],["-8.224670334241132e+00","9.986910030610042e-01"]]}}
%---
%[output:93c5b888]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["1.295906969605790e-01"],["2.263840509499871e+01"]]}}
%---
%[output:226c4982]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAArwAAAGlCAYAAAAYiyWNAAAAAXNSR0IArs4c6QAAIABJREFUeF7svQuYFcW1NrxQxKCDgIIZYYwQ8DLxAkTEGNSYQwKJUX\/0JIhEwRAUHPR8\/giihMSYQ8xHQCJRkVsMoHKZYxQvaED9TUZQQTyAxqAGhRgcQFCuAUEN\/7Maa6en6b27uruuM289jw\/IVK21+n1X136n9qqqRvv3799PaEAACAABIAAEgAAQAAJAoJ4i0AiCt54yi8cCAkAACAABIAAEgAAQCBCA4EUiAAEgAASAABAAAkAACNRrBCB46zW9eDggAASAABAAAkAACAABCF7kABAAAkAACAABIAAEgEC9RgCCt17Ti4cDAkAACAABIAAEgAAQgOBFDgABIAAEgAAQAAJAAAjUawQgeOs1vXg4IAAEgAAQAAJAAAgAAQhe5AAQAAJAAAgAASAABIBAvUYAgrde04uHAwJAAAgAASAABIAAEIDgRQ4AASAABIAAEAACQAAI1GsEIHjrNb14OCAABIAAEAACQAAIAAEIXuQAEAACJRFYtGgRDRkyhJo1a0azZs2iTp06GUds165dNGjQIFq2bBn17t2bJkyYUIiB4xszZgxVV1dTeXm50dhWrVpF\/fv3p507d9LkyZOpZ8+egX8R7\/nnn09VVVVGY0pyNmzYMJo\/f36deJPGiJ+reK5JkyZRTU0NTZ8+ncrKymRdK+sX5oyNVlRU5MqdUrmpLGgDhgQulZWV1rgx8Jhw0YARgOBtwOTj0YGADAIuC14WT+PHj88tWmRwiOsTJ3g3btxIffr0ofXr19Pw4cOdErwi3ubNm6cWeSqeS4jtbt26WRFVYXEq+MwbS30RvOHnCP\/ylvXdwDgg4BoCELyuMYJ4gAAQkEYAglcaqqCjwCu6Si5jpb4J3iwYxOFUXwRv3vyQySH0AQI2EYDgtYk+fHuPgFj9FA8S97V\/eFXr8ssvp5tuuqnw3MVWAMUY0THaL\/oh+53vfCcoOyjWvxjQYRFTbGzcCm\/4mbp06UJTpkwJhofjFOJK2I1+dVxMrIYxFStN0ef9xS9+UShxCD9bsdW6YiuL4eePrmrJcBtd4eVYwjyI2MK2o9xyn7gVtehX79xnzZo1sSvaMl\/Tp3lWjiksCKNYpH2uuDyL+pB5hlITRhJfUfuy70rcuLjyFVFuI\/MuRt+N6LvD\/y\/zjoVziXP\/jjvuoKuvvjr224WkOYV9imflv9sqX\/L+QwEP4CwCELzOUoPAXEcgTrjEfYiW6hf90I\/7ylXYDAuQUv3yfJDH+SoleMMchcV+sWcO9zEpeIuVZYh\/j4pxWW7TCt5SdsMiqpjAjPvloVjf6C9fSRjEvW8i55IEb9JznXHGGYUyj7CfJPuydeMyfGURvKV4EL\/cybyLYW7jxK7svCHw6NChQ+wvfGFsZeKLrnKrWMV3fd5GfA0XAQjehss9njwHAuEPwvCqpvjgLSb+wh8wcX3Fh2F4fJxQiX7Iig\/U8Id6qdrE8Piw2IuLKUnwRlef47AJxyUwyCN4xaY12ZKGuA\/yYl9Fp+E2TQ1v3OpZOC6BSzFuwnEJzjiFRb1w3PhwvhXDKm71Oy4Pi4kh2eeKrlqKTWtJGCSVHqThK035QTgu8S7xM4jNk4ID3ngn\/o1\/Lt7FuOeKW2UPxxR+Z8MiXuYdi84JYozsnMKxp8Enx\/SJoUDACgIQvFZgh1PfEZD5ilx84Ii+0VXEqIDg3f5xJxGEP4TiVm2iwlZmY1Cx0wXiTjwoJXjjdrjH+Y\/b3W9S8MaJrXfeeSf2hIU03KYRvNGcj670CWEXthkVOtFceu2112JP0IhbuS72XGFhVUpcyq7+FXuuYoI3aeU56RSFNHylEXTF4oqeMlFMsBZ73nAeRFeQ4wRvqXcs+rNo7qSZU0RcMvOH7\/M34m+YCEDwNkze8dQ5ECj1oRn3s2IfING+N998c+zXvuFQk1bxZD5kuU8xwRsHS1INb\/R4KdkPTNOCN7oSuWTJkoPqYdNym1bwlvo6O07wRmt7o5g98sgjwTMUa3FfgUdFbdxX\/XGlBKUEr8xzFcvNUmN5TKmyhrR8qRC8UaxL2Yx7F0qVScQJ3rhvamRF\/mWXXSY9p4jnkv3WJMcUiqFAwAoCELxWYIdTnxFI+yELwRt\/1qppwRvmbfDgwbRixYqDzvVNy20awRsWOkLEffGLXzyoJKHULyM6BK94F+OEWHgFsZjglX0uCN7pFP5WgfHgX2i+\/vWvF77ZgeD1+ZMBsbuOAASv6wwhPicRkF1h4UsH8pY0xAGQdlUpaqPY1+bi38eNG1e4RCHrCm\/cRrDa2trC+aumBS9jIHy2bduWduzYEcAS3Y2ehts0glf4DouauDpPFSUNaVYh4\/IrLoZiglf2uYoJ3mKlA7Ivfhq+sqzwCmEqLhXheEeMGFHImzTv4osvvhiUoITfjaQa3lIrvFlLGkphG8enLBfoBwRcRgCC12V2EJuzCKTZKFOsRlJ201qcqErzIRt3m1WxjVHhr5fF1+lpBW8cNnEbgMSHP5MsalWjx1cVO5Ys7aY1kUjRr+\/jxEQabrMI3riTKjg+VZvWignLUrXVfKRWkhBPErxJz1UsrjjRX6xv3ISQhq80gjcuZ\/ldir634RMTouUiUczDOR99v\/jZZFd44545zaa1Ut8iyJYkOTs5IzAgUAQBCF6kBhDIiEDSUUxiRahUv7DQ4b8XO680+mGYV\/CyvWLHNEV9pRW8YbESB23ciRLFKEgSvKU2\/cTZLCYKon1luU36ZUTY5efg8gVxDXFcbDLn3p5yyin05ptv1lkhLFUDG3ccVnRVsFRNaVjERrHjZ0j7XMU2tMk+Q7E8keUrjeBlX6WwyVJPz\/7FqRpxzyIreOO4YHvimwu+6rrYL5Fhv9Ff+NLik3HqxDAgYAUBCF4rsMNpfUEg+oGYdPHEf\/3Xf9F1111H\/IHETfbiiejKkQrBW0xgR31lEbxsOypi4rCJW3ENX86RJHijH\/xJO\/rDIibpjFcZbkuddhF3EUjUpuxlEiLWuI12cb+8lMKa+0fLOOJ+0YrDMhq\/yF\/Z54r6CQuuaC4k8ROdQ2T4yiLo4n4xDL+3ad\/FqD221bFjx4NO25BZaY1+exTe+FrshA+BW9yJHKUuJ6kvczaeo+EiAMHbcLnHkxtCQOaDy1AocOM5Anl20EPMeE6+RPiyR8cVMxV3prKEW3QBAl4gAMH7OU38QTJ37lyqrq6m8vLyouTFfZVUbJXOiwxAkNoRgODVDnG9chAWLcU2NyVdxlAMECGYs46vV0B7\/DDhb0\/Cnz95NwAiPzxOCoSeiAAEb6iWsXnz5iUFr\/ggatOmTWGnufiNuEePHiQ20iSijg4NCgEI3gZFt5KHTar7jp7PK+tUzFdJc52sPfSzg0Cp+nuOqNQti8UiDi\/mZM0vO2jAKxCQQ6DBC95iu9Xj4IseRyP6yK4Oy1GCXvUNAQje+saomeeJ28iVtq41LlKRjxA1ZnjU5aXYBtesq\/dCRFdWVhYWdHTFDrtAwAYCDV7wimsiu3TpQgsWLEgsaYgjiW1MmTLloI0gNgiFTyAABIAAEAACQAAIAIG6CDRowRteseXdzzI1vMVWTJYvX55JLCMhgQAQAAJAAAgAASAABPQi0GAFr\/g6qG\/fvsS3YWUtSxCbBGQ2rq1fv14vm7AOBIAAEAACQAAIAIHPEeCNr2gHEGiwgpfr2KLXnKZd4U1T88Ril6+jXLp0KXIPCAABIAAEgAAQAALaETj77LOJr4qH8G2ggjdu81naFd40Ypcz+uWXX6Z+\/foFice34aC5jQD\/YjJx4kTw5TZNdaIDZx6RRRT88o93DJz5hYBf0Yp3rKamBoK3oa7wJh35k1SeIMoY0hz9IgQvEs+PCQN8+cFTOEpw5hdn4MsvvsILN\/gc84M7vGN1eWqwJQ3RdJVd4RViN+3RL0g8PyYIESX48osvfBiDL\/8Q8C9izIt+cQa+IHhjM1ZG8Oa5ZAKJh4nCLwT8ixbvmF+cgS+\/+MIvleDLPwQgeKUFb\/R83aRSiFIHuWNy9+tVWbduHc2YMYNGjx5NjRs39iv4BhotOPOLePDlF18cLTjzi7N5f\/4L3XTbWHr5wV+hhreh1vDaSFkIXhuoZ\/f58ccf04YNG+j444+H4M0Oo9GR4Mwo3Lmdga\/cEBo3AM6MQ57L4cX3rqAl72yj14adCMELwZsrl1INhuBNBZf1zpjYrVOQOgBwlhoyqwPAl1X4MzkHZ5lgszYIghclDVaSD4LXCuyZnWJizwydtYHgzBr0mRyDr0ywWR0EzqzCn9o5BC8Eb+qkUTEAglcFiuZsYGI3h7UqT+BMFZJm7IAvMzir9ALOVKKp3xYELwSv\/iyL8QDBawX2zE4xsWeGztpAcGYN+kyOwVcm2KwOAmdW4U\/tHIIXgjd10qgYAMGrAkVzNjCxm8NalSdwpgpJM3bAlxmcVXoBZyrR1G8LgheCV3+WYYXXCsYqnWJiV4mmGVvgzAzOqryAL1VImrMDzsxhrcJT5zEv0XsffYxTGj4HEzetqcgqCRtY4ZUAyaEumNgdIkMyFHAmCZQj3cCXI0SkCAOcpQDLga4QvFjhtZKGELxWYM\/sFBN7ZuisDQRn1qDP5Bh8ZYLN6iBwZhX+1M4heCF4UyeNigEQvCpQNGcDE7s5rFV5AmeqkDRjB3yZwVmlF3CmEk39tiB4IXj1Z1mMBwheK7BndoqJPTN01gaCM2vQZ3IMvjLBZnUQOLMKf2rnELwQvKmTRsUACF4VKJqzgYndHNaqPIEzVUiasQO+zOCs0gs4U4mmflsQvBC8+rMMK7xWMFbpFBO7SjTN2AJnZnBW5QV8qULSnB1wZg5rFZ6OHvZ8YOa1YSdSRUWFCpNe28ApDYbowwqvIaAVucHErghIg2bAmUGwFbgCXwpANGwCnBkGPKc7CF6s8OZMoWzDIXiz4WZrFCZ2W8hn9wvOsmNnYyT4soF6Pp\/gLB9+pkdD8ELwms65wB8ErxXYMzvFxJ4ZOmsDwZk16DM5Bl+ZYLM6CJxZhT+1cwheCN7USaNiAASvChTN2cDEbg5rVZ7AmSokzdgBX2ZwVukFnKlEU78tCF4IXv1ZFuMBgtcK7JmdYmLPDJ21geDMGvSZHIOvTLBZHQTOrMKfyjlfKcynNHDDprUD0GHTWqoUyt4Zgjc7djZGYmK3gXo+n+AsH36mR4Mv04jn9wfO8mNoygIE78FIQ\/Aayj4IXkNAK3KDiV0RkAbNgDODYCtwBb4UgGjYBDgzDHgOdxC8ELw50iffUAjefPiZHo2J3TTi+f2Bs\/wYmrQAvkyircYXOFODowkrELwQvCbyLNYHBK816DM5xsSeCTarg8CZVfhTOwdfqSGzPgCcWadAOgAIXghe6WRR3RGCVzWieu1hYteLrw7r4EwHqvpsgi992OqyDM50IaveLgQvBK\/6rJK0CMErCZQj3TCxO0JEijDAWQqwHOgKvhwgIWUI4CwlYBa7Q\/BC8FpLPwhea9BncoyJPRNsVgeBM6vwp3YOvlJDZn0AOLNOgXQAi9dso0smrQj641iyA7DhlAbp9MnXEYI3H36mR2NiN414fn\/gLD+GJi2AL5Noq\/EFztTgaMLKnFc20tA5qyF4Q2BD8ObMvEmTJtHcuXOpurqaysvLi1qD4M0JtOHhmNgNA67AHThTAKJBE+DLINiKXIEzRUAaMDN24Voau3AdBC8Er5psW7VqFfXv35+aN28OwasGUmesYGJ3hgrpQMCZNFROdARfTtCQKghwlgouq50heA+GHyu8GVNy165dNGjQIFq2bBlVVFRA8GbE0dVhmNhdZaZ4XODML87Al198cbTgzB\/OuJyByxq4oYb3AG8QvBnzl0sZampqqEuXLrRgwQII3ow4ujoME7urzEDw+sdMfMR4x\/xjEpz5w9nF966gJe9sg+ANUQbBmyF\/Fy1aRCNGjKBZs2bRkiVLUtXwzp49O1gRFq1U3W+G0DBEEQKY2BUBadAMODMItgJX4EsBiIZNgDPDgKd0t3HjgRVdbtc9thmCN4IfBG+GhOrTpw\/17duXqqqqKO2mtag7rgEeMGBAyijQXTcCe\/fupc2bNwcbERs3bqzbHewrQACcKQDRoAnwZRBsRa7AmSIgNZmZOXNmsBDHbVvv3xW8oKThABQQvCkTb9iwYVRbW0vTp0+nsrKy1IJ33Lhx1LZt24JXFlRY5U1JgoHue\/bsoU2bNgWr8RC8BgBX4AKcKQDRoAnwZRBsRa7AmSIgNZnhFV7+7x9bP6bBz+yH4I3gDMGbIvHCpQydOnUKRqZd4eW633BJQwr36GoQAXx1ZxBsRa7AmSIgDZkBX4aAVugGnCkEU6Op8KUT7AYrvFjhTZ1uvLo7f\/78ouOGDx8elDnENZzDmxpuqwMwsVuFP5NzcJYJNmuDwJc16DM7BmeZoTM6MHwk2SG7t9DK0edgoQ0lDflzECu8+TF00QImdhdZKR0TOPOLM\/DlF18cLTjzg7PwCQ2Nt7xF\/3vHRRC8ELz5kxeCNz+GLlrAxO4iKxC8\/rFSPGK8Y\/6xCc784CwseL\/w5uO0bOpNELwQvPmTF4I3P4YuWsDE7iIrELz+sQLBC87qEwLuP8t7H31Mnce8VAi0bPGv6cVHpkHwQvCaS17U8JrDWoUnCF4VKJq1Ac7M4p3XG\/jKi6D58eDMPOZpPfLtanzLGjeu3z1q0cjgkixslsexZGlzKXN\/CN7M0FkZiIndCuy5nIKzXPAZHwy+jEOe2yE4yw2hdgPR+l1e4YXgPQA7jiXTnn4HHEDwGgJakRtM7IqANGgGnBkEW4Er8KUARMMmwJlhwFO6i5YzfKP5Jlo1cxQE7+c4QvCmTKis3SF4syJnZxwmdju45\/EKzvKgZ34s+DKPeV6P4CwvgnrHh8sZ2NNPztxL9\/60CoIXgldv4kWtQ\/CaxTuvN0zseRE0Px6cmcc8j0fwlQc9O2PBmR3cZbzy6i7X7i55Z1vQvXuHFoHg7devHwQvBK9MCqnrA8GrDksTljCxm0BZrQ9wphZP3dbAl26E1dsHZ+oxVWUxurr7eFUXarzlTQjeEMAoaVCVbQl2IHgNAa3IDSZ2RUAaNAPODIKtwBX4UgCiYRPgzDDgku6iq7tfOvoLwe1q0B11AYTglUyovN2QeHkRNDseE7tZvFV4A2cqUDRnA3yZw1qVJ3CmCkm1dhav2UaXTFpRMHrvFZV0xVnlELwRmCF41eZdUWsQvIaAVuQGE7siIA2aAWcGwVbgCnwpANGwCXBmGHAJd9GTGcTqLg+F7sAKr0QKqe+CxFOPqU6LmNh1oqvHNjjTg6suq+BLF7L67IIzfdhmsRxXynBP30o6t2OLwBx0BwRvlrzKPQaJlxtCowYwsRuFW4kzcKYERmNGwJcxqJU5AmfKoFRiaMoL6+nWR\/9WsMVlDFzOIBp0BwSvkkRLawSJlxYxu\/0xsdvFP4t3cJYFNXtjwJc97LN6BmdZkVM\/LnoqQ7iUAYI3Hm\/U8KrPw1iLELyGgFbkBhO7IiANmgFnBsFW4Ap8KQDRsAlwZhjwIu7GLlxLYxeuK\/yUxS4fQ8Z\/hht0B1Z4rWQsEs8K7JmdYmLPDJ21geDMGvSZHIOvTLBZHQTOrMIfOI8Tu+G6XQje4hxhhddQ\/kLwGgJakRtM7IqANGgGnBkEW4Er8KUARMMmwJlhwEPueIPak69vptGPramzsltM7HIn6A6s8FrJWCSeFdgzO8XEnhk6awPBmTXoMzkGX5lgszoInNmBn8Uun7PLf4rG5QulxC4E78FcYYXXUP5C8BoCWpEbTOyKgDRoBpwZBFuBK\/ClAETDJsCZYcCJKLo5jSMoVrMbjQ66Ayu85jMWXy1YwTyPU0zsedCzMxac2cE9q1fwlRU5e+PAmVnsL753BS15Z1sdp7JiFyu8WOE1m60hb\/hNyxr0mRxjYs8Em9VB4Mwq\/Kmdg6\/UkFkfAM70U8BlCyxyh85ZfZCz315+Cl159nHSQUB3YIVXOllUdkTiqURTvy1M7PoxVu0BnKlGVK898KUXXx3WwZkOVA\/YZKHL\/10\/d3WdWl3+mUy9blxk0B0QvPoytoRlJJ4V2DM7xcSeGTprA8GZNegzOQZfmWCzOgicqYc\/SeiO+X860kWnt87kGLoDgjdT4uQdhMTLi6DZ8ZjYzeKtwhs4U4GiORvgyxzWqjyBM1VIHrDDG9L4XN3w6QvCQ5pa3WJRQXdA8KrNWElrSDxJoBzphondESJShAHOUoDlQFfw5QAJKUMAZykBK9J91su1dGP1W7E\/zVq+EGcMugOCV03GprSCxEsJmOXumNgtE5DBPTjLAJrFIeDLIvgZXYOzbMCJsgVezY2euhBe0U06Vzetd+gOCN60OaOkPxJPCYzGjGBiNwa1MkfgTBmURgyBLyMwK3UCztLByUL36b9soVvn\/63oQF7RHdmrPV1xVnk64xK9oTsgeCXSRH0XJJ56THVaxMSuE109tsGZHlx1WQVfupDVZxecyWG7eM22oDa31Grud09tRdd94\/jgBAZdDboDgreAwLBhw2j+\/PmF\/588eTL17NkzMfcmTZpE48ePL\/QbPnw4VVVVlRyHxEuE1akOmNidokMqGHAmBZMzncCXM1RIBwLO4qHildzFa7YGm9CKiVweqXM1Ny4y6A4I3gABFrvLly+n6upqKi8vp0WLFtGQIUMoSbyy2J0yZQrNmjWLOnXqRKtWraL+\/fvT4MGDS4peJJ70nOpER0zsTtCQKghwlgou653Bl3UKUgcAzg5AJk5V4HKF+2r+EXvKggCXRe4Pziynq84+TutqLgRvcjo32r9\/\/\/7kbvWrhxCp48aNq7OiyyK4traWpk+fTmVlZQc99MaNG6lPnz7UtWtXmjBhQuHnUfGMxPM\/XzCx+8chOPOLM\/DlF18cbUPmTGw8m\/PKhmAlt1RjkdvrK63o4jNa07kdW1gjGgttWOEtmnwQvNbeS+ccN+SJ3TkyJAMCZ5JAOdINfDlCRIowGhpnLHJ3fPwp3fro30qWKjCELHLP7diS+nYttypyw3RC8ELwxr7eoi43qY63WElDjx496qz6Rp2IxJs9ezZVVFQUfszlFGjuIdDQJnb3GEgfEThLj5nNEeDLJvrZfNd3zmp3fEqL39lKc5aVrsUV6LU5qjFdf8GX6JTyI50QufwtdLitX7+e+vXrRzU1NXV0Rzb2\/R\/VIEsawrSJ2l3+t969e5cUrWKcKInYuXNn8E9JIpn7CMEbTRmu\/x0wYID\/mVTPnmDv3r20efPmoL67cePG9ezp6ufjgDO\/eAVffvHF0dYnzljcctuw81OasnQbvfr+x1KEsMj9+bda0XHNGhP\/3aU2c+bMYH9RtEHwHkCkwQtekRi7du2iQYMGBTW8YiNbNGni+oi63jZt2hSt\/Q0LXq4bbtu2bcE0Cyqs8ro0ZRyIZc+ePbRp06bgt2IIXvf4iYsInPnBk4gSfPnFV32YF\/kkhb\/U7qIFr2+RFrhcqnD1OW2oc9sj6WvtDt7b4xKLrEfCq7xLly6liRMnYoX3c5IgeEPZmnTiglgNjq7oFtsEF34RUEvj0rSQHEt9\/+ouGQH\/eoAzvzgDX37xxdH6xJnYZFb96kZau2VPYg2uYIMF7vkntqQ+Z5YHdbk6z8nVnQHQHXURdlLwbt++ne677z7iP7O25s2b0y233JJquIzgHTFiROFIMmFcrPL27du36NFkSLxUVFjv7NPEbh0sRwIAZ44QIRkG+JIEyqFurnImxO3W3Z\/Q1BfWS4tbhpYFbfcOLeiKsw4cG+azwI2mCnSHB4JXCEguuM7a+KtorluJa7xSGydci63gChtY4c3Khn\/jXJ3Y\/UPSXMTgzBzWKjyBLxUomrXhCmd8k9m\/9u+ncYvWpRK3QuD2O+s4+nqHFvVO4ELwln4fnFzhFYJ39OjRUjefRR+RhemYMWOKCl5Ri8vjxJm7YnW3srKyaC2uihpeFI+bnaCzenNlYs8af0McB878Yh18+cUXR2uDM169fWb1hzR\/5QeZxO3xLb9Al3ctp3bHNHXiJAWTrGOF14MVXt4df\/311wf\/nXfeeanz44UXXqB77rmH5s2bV3Js9Grh6C1r0SPIhLHoOJnTHZB4qWm0OsDGxG71geuBc3DmF4ngyy++dAteUZbw579tpZff3ZZa3IrV2++c2oqqvnF8AG59Kk\/Iki3QHR4I3s8++4z4vyZNmmTh2MkxSDwnaSkaFD6M\/eJL94exf2i4HzHeMfc5ikaogjMhbNn2w\/+7id7ZvDuzuD23Q0vqe5b\/m8t0ZQJ0hweCl0sa+LDk448\/PtgExlf5HnroobpywohdJJ4RmJU5UTGxKwsGhqQQAGdSMDnTCXw5Q4V0IGk4Y2HLjf98\/f2d9NRftmQStmKlVtxiVt82lkmDn6EjdIcHgpdPZ\/jv\/\/5vevLJJ2nfvn3UunVruvrqq4lPQWjZsmUG2u0PQeLZ5yBNBGkm9jR20VcfAuBMH7Y6LIMvHajqtRnHWVjYbtixl2a9VJtL2PITDOpeQceUHRacntDQyxLyMArd4YHgFSHyJrHnn3+e7r\/\/fnrttdeCfz7jjDPouuuuowsuuMCrkgckXp7X1vxYfBibxzyvR3CWF0Gz48GXWbzzemNhy5y99u4G2nvYUTR3efpNZOEYxEpt365cktC03p+YkBf\/LOOhOzwSvOFQeSPbww8\/TDNmzAiufC0rK6Mf\/OAHNHDgwDo3l2VJChNjkHgmUFbnAx\/G6rA0ZQmcmUJajR\/wpQZHlVbCq7Vs96V3t1HN37ZmXrFlGyxs+aSEC046ms5u3xzCViVhCbagOzwVvCLs\/fv309tvv03Tpk2jP\/7xj7R7925q3759sOp7ySWXOLvqi8Qz+JYrcIUPYwUgGjYBzgwDntMd+MoJYI7hYWG7n\/ZT9fJNtO5D+dvI4lyL0gO+oYxvKkOtbQ6CFA2F7vBc8IbD5\/rel156iX7CvxS3AAAgAElEQVT6058G\/1xdXU3l5eWKUkWtGSSeWjx1W8OHsW6E1dsHZ+ox1WkRfOlE98BmMW7iz2XrttPzb32Ua7WW7bU5qjE1btyYLjq9NfERYBC2ennMYx26ox4IXiF0ubyBBS8fYdazZ0+64447iK8UdrEh8VxkpXhM+DD2iy+OFpz5xRn4UsNXWNi+99EemvPKRvrH1o8LQjerF1GKwKu153z5wOaxY48g2rBhQ3CCEoteNLcRgO7wVPCyqF25ciXNmTOnUMrg0+kNSDy3J4ZodPgw9osvCF7w5R8C8hHXOQlh+16a9XKtMlHLUZzboQV97cstgtvISq3YYl6U58yFntAdHglertddv359sFHt8ccfpw8\/\/DCo0b3wwgtp8ODBdNJJJ1GjRo1cyKvEGJB4iRA51QETu1N0SAUDzqRgcqYT+DqYisVrtgWC87k3P6JHVmxSImrZi1itvazLsXTisUcWjvpKe+QXOHPm9ZEKBLrDA8G7Z88eeuCBB2jWrFlUW1sbiNrKysrgLN7vfOc7wQkNvjUknl+MYWL3iy+s8IIvHxAI3zL26MpN9Pam3cpFbbd2zanjsUcEJyOorq\/FvOhDlv07RugODwQv37TWp08f2rZtm1dHj5V6FZB4fk0UmNj94guCF3y5goAoP3j6L1votfd3BrW0S97ZpiQ8sSJ70Rmt6dTjyrSI2lKBYl5UQqMxI9AdHgheXuFdt24ddejQwdljxtJmLBIvLWJ2+2Nit4t\/Fu\/gLAtq9sb4ypdYpWXx+dvn36O3Nv5T2SptuPzg5C8eSb07HxsQpHqlNivrvnKW9Xl9Hwfd4YHgFSu8o0ePDk5fSNsWLVpEY8aMoZqamrRDtfVH4mmDVothTOxaYNVqFJxphVe5cRf5ih7ltXbLbqp+VV0trQBR1NRy+cE3Tz46EMw+XKPrImfKE7MeGYTugOC1ks5IPCuwZ3aKiT0zdNYGgjNr0GdybIuv8Art0rXbgxMPuKkqOxArsvwni1g+r7bZFxpn3iiWCVxNg2xxpulx6r1Z6A6PBC+f0JC1VVRUYIU3K3gYhzNdPcwBfBj7RZpOvoSoffndbfTnv23VJmh5Y9jXvtycvnHi0YEPV0oPdGWCTs50xdyQ7ULweiB4t2\/fTvfddx\/xn1kbX0Bxyy23ZB2ufBwSTzmkWg1iYtcKrxbj4EwLrNqMZuErWnJQ87et9NK7BzaEqVyhFeKVBe2ZJxxFXE+r49QDbeBqMpyFM02hwKwEAtAdHgheCR6964LE84syTOx+8cXRgjO\/OIvjKypoF6\/ZGghZFTeHRdERdbQnHNOU+pz5RXp\/296gBEGIXb\/QNBMt3jEzOKvyAt0Bwasql1LZQeKlgst6Z0zs1ilIHQA4Sw2ZlQGi3OAfH+6k\/++NTbRxz6HaBe1ZJxxFHVofUS\/qaK2Q9rlTvGM20U\/vG7oDgjd91igYgcRTAKJBE5jYDYKtyBU4UwRkBjMsYnnFNLwhbErN+uAcWh2rs2IVlssMKsuPpM7HN6NzO7YsRJ72BrEMj9wgh+Ad84t26A4IXisZi8SzAntmp5jYM0NnbSA40wt9uNxgy6599OybH9HfP9yjvHZWiFn+kwXtd05tRZ0qmgUPV983hellML91vGP5MTRpAboDgtdkvhV8IfGswJ7ZKSb2zNBZGwjO8kG\/eM2BzV9vbfonrfjHzkDM6lyd\/fTTT+nMtl+grh2\/SF2OP6ogaMOCN98TYbRqBPCOqUZUrz3oDghevRlWxDoSzwrsmZ1iYs8MnbWB4Oxg6MWqLP9E\/P3Z1R\/Sq+\/t0Cpm2R+vzvLK7KltygqbwcJiFnxZe1UyOwZnmaGzMhC6w2PBu3\/\/ftq6dSt99tlndPTRR9MhhxxCn3zyiRfXDyPxrLzvmZ1iYs8MnbWBDZUzUTf794\/20Note4gvU9C1MisEqziii6++bXrYoYXV2TS1sw2VL2sviALH4EwBiAZNQHd4KHhZ6D7zzDP005\/+lDZv3kx8qUR1dTUdfvjh9OMf\/5i6du1Kw4cPd1r4IvEMvuUKXGFiVwCiYRP1iTOxCUyUGVS0PJwmPvcerdm8W7uYFauzvAlMHNMlhG4aQZtEf33iK+lZ68vPwZlfTEJ3eCh4\/\/znP9PgwYPpq1\/9Kp144on0\/PPPB4K3rKyMbr31Vlq4cCGNHj2a+vfvnyobhw0bRvPnzy+MmTx5MvXs2TPRxqJFi2jIkCGFft26daPp06cH8RRrSLxEWJ3qgIndKTqkgvGJs\/BpBm9\/8E9a9NcPafWGfxoTs+ef2JIqWn7B6mUKPvEllYANoBM484tk6A7PBO\/evXsDcfmvf\/2LWJC+8MILNGbMmEDwlpeX0759+4iF65YtWxJFZ\/jReczy5csLdoSI5ZXiqqqqolk9adIkGj9+fBALi+ONGzdSnz59qE2bNiX9I\/H8migwsfvFF0drm7O4o7keXfkBvb3pn0H9rO4yA7Eyy3WzfLKBWJUN\/+kSq7b5cgkLX2IBZ74wdSBO6A7PBK8QlNdeey1deeWVxMI0LHj5cR588EGaOnVqQbwmpeSqVauC1eBx48bVWdFlEVxbW1tUuIpY+vbtW0cUc0wjRoygWbNmUadOnWLdI\/GSWHHr55jY3eJDJhoTnIWP5uIzZp\/+y5YgNNXX2oafV5QRcN3s2e2bBxcoREsNZPBxrY8Jvlx7Zt\/jAWd+MQjd4angveqqq+iaa66JFby86vrwww\/TvHnzqHXr1pkzMknwxoltWWdIPFmk3OiHid0NHtJEkYWzuFXZZ1Z\/SCv\/sZPWaTyWSzxXWMzy9bbtWx0R\/Ej8u8qa2TRYmuibhS8TccFHcQTAmV\/ZAd3hmeAVJQ27d++madOmBUv04RXe9evX09VXX03HH398UGbAG9mytGipQpwN7lNTU0N33HFH4JN9c+vduzdNmDChpFskXhZW7I3BxG4P+6yei3EWXpXlM2bnr\/wgcKGzxECIVl6VPeGYpsFtYBef0ToobWgIYlaGQ7xjMii51QecucVHUjTQHZ4JXg53xYoVxCUNvGGN\/3vqqafoxhtvpDVr1tAf\/vCHoI53ypQp9I1vfCOJ\/4N+Ht6AliRcxSa3Zs2aFcoX0tbwzp49OzhlQjSuQ0ZzDwFM7O5xIiKq3fEptTmqcZ1rbBe9uY3+8v5Oeqt2G23e8+8zZ3U8BfvmxmK247FH0KWdj6VDGjUK\/g03gckjjndMHitXeoIzV5iIj4P1SLjxoly\/fv2Chbqw7nD7KfRF12g\/n\/nlQXvxxReDY8nWrl1bJ1ouYfjlL39J3\/rWt3I9xa5du2jQoEFBDa\/YEBc1KARv9DQHIZpLnfIgftOK2uRa4gEDBuSKHYPVI8DfLPARePwLSePGBwQOmjkEXn3\/YzquWWNicfu\/tR\/T8vUfB87533U2IWbZd5c2h1O345sG7vj\/N+w8cDMYmhoE8I6pwdGkFXBmEu30vmbOnBksxkUbBO8BRLwRvBwsa3M+jeGvf\/0r8Yt36qmnBoLk0EMPHHyet4nNbHwEWtxJDSx4n3vuuYM2pxXbzBaORwhe3ijXtm3bwo84fqzy5mVO\/fg9e\/bQpk2bgt+KIXjz41u3VnYPfenopjTzpVpatm57YJzFJItbXS1cK9u+VVO6+LSj6bBDDwnc8c+E0NXlH3YPRgDvmH9ZAc7c5oy1SHiVd+nSpTRx4kSs8H5Om1eCV3eqJQleruHl0onoaQxpBC9+09LNohr7+OouPY58SQKLR77pq+ZvW41s+hKClf88+4QyOqbJp9Sz0\/HBLykcCwvtczu2SP8wGKEdAbxj2iFW7gCcKYdUq0HU8NaF13nBu337drrvvvuI\/0xqrVq1CkobTjvttJKrvsWOEUsqTSgmiHEsWRIz\/v28oU\/sYqMXMyf+vnX3J\/THN7ZoP1NWZEt4VfYrx5UFm77CP4ueYNDQOfPtLQNfvjFm\/6xr\/xCzGzEEr2eCl+sor7\/+enrjjTeIT2rgxsKWG5c3xLXzzjuP7r77bjrqqKNify7qdfmH4oY0IWYrKytLXiARLWsQq7t8vXGpkxqQeHZf\/LTe6\/uHsbjpi3HZsH0vvbJuO63eqPemrzghe2qbMjrx2CPo5C8eGfw4zwkG9Z2ztDnsen\/w5TpDB8cHzvziDLrDM8HL4S5btoyGDh0aXBbxox\/9qHCFL5\/O8D\/\/8z\/EpQb33HMPfeUrX6HHH388OLaM+950000lszN6tXD0lrViJQziCDNhPOl0B+6HxPNrovBxYg8fv8Vo7973GS1a\/SG9ZUHIsnBlEfvVLx34pTOPkJXNHB85k322+tgPfPnHKjjzizPoDs8Er1iNPemkk+j222+nRp8f\/yMegzey3XbbbfT3v\/89OIe3adOmNHbs2EBgPvroo85kJxLPGSqkAnFtYg+vyL62fif9dcM\/6e8f7dF+lmx4VZaP4Wp3TFM678QW9Nm\/yLnbvlzjTCrRGnAn8OUf+eDML86gOzwTvKJkgFd4L7\/88ths4xvW7r333sJxYo888gjdddddwc5EVxoSzxUm5OIwNbGHheyOjz+l5978iN7eZKa0QKy8spDlFdhTyo+kLsfXXZENr87KIWevlynO7D1h\/fIMvvzjE5z5xRl0h2eCd+vWrTRw4EDq0KFDcMNZkyZN6jwBlzWMGjWK3nnnHbr\/\/vupZcuWQf0uX07x9NNPO5OdSDxnqJAKJOvEHr2qlp2t3riLVq3fRX83cFVtdEX2y62a0pknNCf+Myxe6+OVtVk5k0oIdFKOAPhSDql2g+BMO8RKHUB3eCZ4OdypU6fS+PHj6Qc\/+EGwgU2cW8urv1y7O3fu3KBe97rrrgvqfW+99dbgjF4Wvq40JJ4rTMjFUWpi5+O3uL27ZTet+\/DjYMMXtyXvHPh3nY2FqliRrWj5BTqvY8s6QjYsanXG4aJtfBi7yErxmMCXX3xxtODML86gOzwUvLyKy4L397\/\/PX322Wd1noAvneCNbLzhjF\/Ga665JrgtjUUyn7jgSkPiucLEwXHEHcH1\/tbd9NTK9+mjfY2N1MmGhezXvtyC2h9T\/1dkVWcEPoxVI6rXHvjSi68O6+BMB6r6bEJ3eCh4Rch8RBnfdMZHiHHjVdxvfvObhZvL+BYYXvVt06YNHX744fqyKINlJF4G0BQPERcj\/KV2Fz39xhZat0X\/pq\/wWbLnndgyWJ3t3qHuRQj1sbxAMXVS5vBhLAWTM53AlzNUSAcCzqShcqIjdIfHgteJDMoYBBIvI3AphokNYCv+sYMW\/fXDYKSuMoOwkD29bTO68LQDZ0OHxSuEbAryFHTFh7ECEA2aAF8GwVbkCpwpAtKQGegODwUvHz327rvv0vz582Mvm9i7dy+9\/\/77wZ3Ror7XUD5Ju0HiSUNVsqMQta++t4OeXf2hlnIDFqrHNWtMxx1JdFbHY+n0iuYFMQsRq4ZHHVbwYawDVX02wZc+bHVZBme6kNVjF7rDQ8HLJy7wJRFcyxvX+OQGvumMN6nxKQ0uNiReela4BOGld7dRzd+2KlupFYKVywq+e1or2vnxZ4VNYOEVWkzs6fmyPQKc2WYgnX\/wlQ4vF3qDMxdYkI8BusMzwfvPf\/6TBg8eHGxE++1vf0snnHBCsDGtS5cuwe1rc+bMoRkzZgSXTpx++unymWC4JxKvNOAsbhe8vpm4vjZPGUJY0HY9oXlwbW2WW74wsRt+QRS4A2cKQDRoAnwZBFuRK3CmCEhDZqA7PBO84uIJPpLshhtuCKL\/+c9\/Th988EFQwsCNV395k9q4ceMOuonNUF4lukHi\/RsiUZYwduHaTOJWCNhvnNiSfnBmeWCY\/01luQEm9sSUdq4DOHOOkpIBgS+\/+OJowZlfnEF3eCp4b7zxRrrsssuC6B988EF64oknaNq0aXTUUUcF\/\/\/QQw\/RAw88QK1aHdg85FpryIknBO7jr31A0xe\/L02NELC9vtKKLj6jdaaVWmlnkY6Y2LMiZ28cOLOHfRbP4CsLanbHgDO7+Kf13pB1RxxWjfbzjjCH244dO4IShq9+9as0cuTIINJnn302WOVloduuXbtA+LLYra6uxqY1h7jkMgXZVVwhbkdf+GUqP+pw5Su2aWHBxJ4WMfv9wZl9DtJEAL7SoOVGX3DmBg+yUUDw1kXKecHL4d55553BpRM333wz9e3bN6jnvfLKK4Ob17773e\/SLbfcQqzbxdXCsslgsl9DSDyxklv96kZ6cOmGkvCKixZG9mpvXdzGBYqJ3eTbocYXOFODoykr4MsU0ur8gDN1WJqw1BB0RxocvRC8vMrL9bv8J4vaFi1a0JgxY4LNaix0GzVqRLfddhv1798\/zbMb7VvfE2\/OKxuD1dzwrWVRgFnk3tyzHZ3bsaXSelsdRGJi14GqXpvgTC++qq2DL9WI6rcHzvRjrNJDfdcdabHyQvDyQ7Gw3blzJzVr1iwQuHzF8IsvvkgvvPAC9erVKyh54H93tdXHxGNx++K726hq9uqisLPIvadvpZOruKVyBRO7q29S8bjAmV+cgS+\/+OJowZlfnNVH3ZGHAW8Eb6mHZPG7fft2at68OR166KF58NA2tr4l3pSa9XTr\/L\/F4iVE7rkd616hqw1cDYYxsWsAVbNJcKYZYMXmwZdiQA2YA2cGQFboor7pjrzQOC94xbFko0ePpp49e8Y+74IFC2js2LHYtJY3GxLG84oun5E7dE78iu4VZ5WTqMnVHIp285jYtUOs3AE4Uw6pVoPgSyu8WoyDMy2wajMKwVsXWicFL18VzOUKu3fvpm3bttFdd91Fl156KXXq1OmgxOC+XNfLfefNm0etW7fWljx5DPueeCx2L5m04qAaXV7Nvej01nTteRXO1+Wm4Q8Texq03OgLztzgQTYK8CWLlDv9wJk7XMhE4rvukHnGNH2cFLz8ALNmzaLbb789qN1NalzGMGLEiOD4MlfreH1NPBa6z6z+kEb84e2DaBhwThv6f3ucUK+ErnhITOxJb517Pwdn7nFSKiLw5RdfHC0484szX3WHLpSdFbz79u0L6nI3bdpEQ4YMof\/zf\/4PnXfeeQfh0KRJE2rZsqWzQlcE7GPilVrVfbyqS70UuhC8uqYa\/XbxYawfY5UewJdKNM3YAmdmcFblxUfdoerZ4+w4K3hFsLwh7aOPPgpuVOPrg31tviUeHzMWrdWtD5vRZPMHE7ssUu70A2fucCETCfiSQcmtPuDMLT6SovFNdyQ9T96fOyl4hcjlP2UblzUcffTROKVBFrAS\/WYv20DXz32zTg8Wu\/V9VTf8wJjYFSSSYRPgzDDgOd2Br5wAWhgOziyAnsMlBG9d8JwUvOJkhvXr10tTXVFRgVMapNGK78glDI+v+oB+9sQ7dTrc0\/cU6tftuJzW\/RqOid0vvjhacOYXZ+DLL77wjvnHFwSvB4J3z549tGTJEuITGGQblzt0796dmjZtKjvEaD\/XE4\/F7pxXNtDYhesKuPCqLh8zxseNNbSGD2P\/GAdnfnEGvvziC4LXP75c1x2mEXVyhdc0CCb8uZx4LHa5XpfP2BWtoZUwRHMAH8Ym3gq1PsCZWjx1WwNfuhFWbx+cqcdUp0WXdYfO5y5m2yvBu3XrVlq8eDG9+uqrxKczdOnShb72ta8FpzRkacOGDaP58+cXhk6ePLno5RbF7E+aNInmzp2bWE7hauKx2J37ykb6vwvXQuyGSMbEnuWNsjsGnNnFP6138JUWMfv9wZl9DtJE4KruSPMMKvt6IXh589rUqVNpwoQJFN3IxpvVhg4dSlVVVYEIlm0sdpcvX14QqosWLQqOPxs+fHhgS6atWrWK+vfvH1xpXF1dTeXlxb\/6dzXxpi1eTyMf+fcVwQ19ZVfwjold5g1wqw84c4uPpGjAVxJC7v0cnLnHSamIXNUdtlD0QvA+\/PDDNHLkSDr\/\/PPppptuonbt2gV4rVu3ju68887gVrZx48bRJZdcIoWjEKo8JnxdMYvg2tpamj59OpWVlZW0tWvXLho0aBAtW7aMZDbMuZh4YxeuPahmtyGdxFCKYEzsUq+SU53AmVN0JAYDvhIhcq4DOHOOkpIBuag7bCLovOAVwpIF6N13333QpjTe4HbDDTcEK79ckpDnrN40gpdLGWpqaoKyigULFni5wnv0sOcLuYeV3bqvISZ2m9NSNt\/gLBtutkaBL1vIZ\/cLzrJjZ2MkBG9d1J0XvOKIsmuvvZauvPLK2Jx58MEHg5KHpLKCUgnHAnb8+PGBaA6v+saN4fIHvsqYrz\/m0yR8q+Hlut3OY16C2C2REJjYbUzP+XyCs3z4mR4Nvkwjnt8fOMuPoUkLELyeCd7NmzfT5ZdfTpdeemmwkhvXeOX30UcfpXnz5lHr1q1T5ZOo3eVBvXv3DuqESzUhwPv27RvU+qbdtDZ79uygBEK0UnW\/qR4kRedLp7xe50SGiT\/oSFec1bDO2U2CCxN7EkLu\/RycucdJqYjAl198cbTgzG3OWJ+EG99l0K9fv+Db6LDucPsp9EXn\/ArvJ598QlxqsHr1apo2bRq1b9++Dhpr166la665hiorKwOxethhh2VCS5ROcA1vqZXiaNlDWsEbDY43vQ0YMCBTzFkGPfbXXfSL57YUhl59ZnO64evZTrnI4t+XMXwGNP+yxb+QNG7c2JewG3Sc4Mwv+sGXX3xxtODMbc5mzpwZfPMcbRC8BxBxXvBykK+\/\/joNHDgweNm+\/e1vBxdMcONygmeeeSao2+WSBq6nzdPEZrbBgwfHntQQLmXo1KlT4Cqt4OWNcm3bti2EyYLK1CovlzJ8bfyKgm+u2315eD7M8uDt8liuDd+0aVPwWzEEr8tM\/Ts2cOYHTyJK8OUXXxwtOHObM17hDa\/yLl26lCZOnIgV3s9p80LwcqxvvfUWjRo1ilauXEn79+8\/oNYbNaLOnTvTHXfcQSeffHLuTEwSvNFze6MOSx1pZruWJnq5BDaplU4XfHWX+3UybgCcGYc8l0PwlQs+K4PBmRXYMzu1rTsyB65poDeCVzw\/r\/LyBRTc+MKJLKcyxK3Usj1RzyuzcU3Ek3aF19ZXC9EjyO69orJBXhks+x5hYpdFyp1+4MwdLmQiAV8yKLnVB5y5xUdSNBC8dRFyXvByHeX9998fnLF70kknEV80kbeJel22I87cFau7XAsscw6vT4I37lSGlaPPyQtjvR6Pid0\/esGZX5yBL7\/44mjBmV+cQfB6KHj5lAa+ZOKYY44JhO\/VV18d1FZySUOeFi1RiJYk8OrtlClTgiJwUbMb9efDCu\/F964onMrApQz39K2kczu2yANdvR+Lid0\/isGZX5yBL7\/4guD1jy8IXs8EL4e7b98+eumll2jGjBnBn\/z\/fFqDOK4s7VFkNtLWVuItXrONLpn0741qE35wMl19ThsbEHjlEx\/GXtEVBAvO\/OIMfPnFF94x\/\/iypTtcRcr5koYocFHxy8eWcRkCn+Jw0UUXUZMmTZzE2lbiRVd3Ucoglx74MJbDyaVe4MwlNpJjAV\/JGLnWA5y5xkjpeGzpDldR8k7whoHcsWMH\/f73vw9Wfps1a5brpjXdBNlIvDmvbKShc1YXHu3xqi4oZZAkGhO7JFAOdQNnDpEhEQr4kgDJsS7gzDFCEsKxoTtcRsg7wcsbzp5\/\/vlA3DKZ3M4444xghZfP6MUK74F0441qXMrAf3Lj2l2s7sq\/ipjY5bFypSc4c4UJuTjAlxxOLvUCZy6xkRwLBG9djLwQvFGR+9lnnwU1vD\/60Y\/oe9\/7XnA8mevNdOJFV3dZ7LLoRZNDABO7HE4u9QJnLrGRHAv4SsbItR7gzDVGSsdjWne4jo7zgpdvDenTpw\/xndC8Oe2qq66iSy+9tM5tZa6DzPGZTLy4SyawupsuSzCxp8PLhd7gzAUW5GMAX\/JYudITnLnChFwcJnWHXER2ezkvePmSiTlz5lCvXr3oy1\/+cu6jyGzBbTLxsLqbn2VM7PkxNG0BnJlGPJ8\/8JUPPxujwZkN1LP7NKk7skdpbqTzgtccFHo9mUw8nMyQn0tM7PkxNG0BnJlGPJ8\/8JUPPxujwZkN1LP7NKk7skdpbiQEryGsTSVe9FY1XCGcjWBM7NlwszkKnNlEP71v8JUeM9sjwJltBtL5N6U70kVlrzcEryHsTSUeVnfVEIqJXQ2OJq2AM5No5\/cFvvJjaNoCODONeD5\/pnRHvijNjYbgNYS1icSLru6O7NWORvZqb+gJ65cbTOz+8QnO\/OIMfPnFF0cLzvzizITu8AkRCF5DbJlIvFkv19KN1W8FT4Rzd\/MRi4k9H342RoMzG6hn9wm+smNnayQ4s4V8Nr8mdEe2yOyMcl7wbt++ne67777g2uDTTjstFqVXX32V7rrrLpowYUJwdJmLTXfiRVd3f9y9LY37z5NchMKLmDCxe0FTnSDBmV+cgS+\/+MIKr3986dYdviHivOAV5\/COHj2aevbseRC+fAnF+PHjacGCBQ36auHFa7YFN6uJhmuE872K+DDOh5+N0eDMBurZfYKv7NjZGgnObCGfzS8Eb13cnBS8+\/bto1GjRtEjjzwizfJ\/\/Md\/0N13301NmzaVHmOyo+7Ew2Y1tWxiYleLpwlr4MwEyup8gC91WJqyBM5MIa3Gj27doSZKc1acFLz8+GvXrqWZM2cSXzzx\/PPPU+fOnWNvV2vWrBl16dKFzj33XOK\/u9p0Jh42q6lnHRO7ekx1WwRnuhFWax98qcXThDVwZgJldT506g51UZqz5KzgFRDI1PCagyu7J52JN3bhWhq7cF0QHDarZecoPBITuxocTVoBZybRzu8LfOXH0LQFcGYa8Xz+dOqOfJHZGe284LUDi3qvOhMvXM7QvUMLemJoF\/UP0MAsYmL3j3Bw5hdn4MsvvjhacOYXZzp1h19IHIjWScErVnU5wB\/+8If00EMPEf9bqda8eXO67rrriP90selKvGg5AzarqWEfE9YyyxEAACAASURBVLsaHE1aAWcm0c7vC3zlx9C0BXBmGvF8\/nTpjnxR2RvtpOAVJzMwLLwR7YYbbqD169eXRKmioqJBntKAcgY9Lw8mdj246rQKznSiq942+FKPqW6L4Ew3wmrtQ\/DWxdNJwauWcjes6Uo8lDPo4RcTux5cdVoFZzrRVW8bfKnHVLdFcKYbYbX2dekOtVGaswbBawhrHYmHcgZ95GFi14etLsvgTBeyeuyCLz246rQKznSiq962Dt2hPkpzFp0XvKKeFzW8BycFyhn0vSiY2PVhq8syONOFrB674EsPrjqtgjOd6Kq3DcFbF1PnBa+o50UN78EvA8oZ1E8QwiImdn3Y6rIMznQhq8cu+NKDq06r4EwnuuptQ\/B6JniLpcD+\/ftpy5YtNHfuXHrsscforrvuotNOO019xiiyqDrxouUMK0efE5zBi6YGAUzsanA0aQWcmUQ7vy\/wlR9D0xbAmWnE8\/lTrTvyRWN\/tPMrvEkQsfC97bbbghvZJkyYQIcddljSECs\/V514c17ZSEPnrA6eBZdNqKcUE7t6THVbBGe6EVZrH3ypxdOENXBmAmV1PlTrDnWR2bHkveBl2ObNm0f33ntv6mPJhg0bRvPnzy8gP3nyZOrZs2dJJnbt2kWDBg2iZcuWFfoNHz6cqqqqSo5TnXgoZ9D7wmBi14uvDuvgTAeq+myCL33Y6rIMznQhq8euat2hJ0pzVr0XvPv27aNRo0bRG2+8QQ888AC1atVKCj0Wu8uXLy+I5EWLFtGQIUOolHgV9cRt2rSh6dOnU1lZGa1atYr69+9PPXr0CFaYizWViRctZxjZqx2N7NVe6rnRSQ4BTOxyOLnUC5y5xEZyLOArGSPXeoAz1xgpHY9K3eHXk8dH67zgTTql4d133w2EK4vOn\/3sZ9SoUaNEXoRIHTduXJ0VXRbBtbW1BTEbNcSieMSIETRr1izq1KlT4ceTJk0Kaomrq6upvLw81r\/KxFu8ZhtdMmlFwQ9uV0ukPHUHTOypIbM+AJxZpyBVAOArFVxOdAZnTtAgHYRK3SHt1OGOzgvepFMamjRpQpdddhndcsstdNRRR+WCOknwFjPOgnfKlCkHCeFwf5WJ13\/GX+jJ1zYH5lG\/m4vyooMxsevBVadVcKYTXfW2wZd6THVbBGe6EVZrX6XuUBuZHWvOC15TsLBoHT9+PMnU8UZjipZHxMUsEm\/27NnE1yCLVmxFuNRzd\/2\/rxCXNXC7qUcFyhk0JAkmdg2gajYJzjQDrNg8+FIMqAFz4MwAyDlc8AJhuPFxrv369aOampo6uiOHC6+Heit4+XSGnTt3UrNmzaTKGIqxJGp3+ee9e\/cuWYcbZ0Om9pfHCcEbtcGlGAMGDJBOotodn9LFM9cX+k+9rJzObIvjyKQBlOy4d+9e2rx5c1Ci0rhxY8lR6GYTAXBmE\/30vsFXesxsjwBnthko7X\/mzJnBN83RBsF7ABEvBC9vTPvNb34TbEzjlVjeLMbHkA0cODCouf3JT35CF198cS7hK05fYHulanHDiSRqgSsrK4vW\/Yr+QvBy3XDbtm0LZlhQpVnlvfO5f9Cdzx0QvFzO8PLwLm6\/gZ5Gt2fPHtq0aVPwWzEErx8kgjM\/eBJRgi+\/+OJowZnbnPEKb3iVd+nSpTRx4kSs8H5OmxeCd+rUqUG5Adfqjh49OhC8\/OItXLgwEJpr1qwJVmYvvPDCXNkoBOzgwYMTjxlLI3Y5KFW1NLxZjTetceveoQU9MRSCNxfpRQbjqzsdqOq1Cc704qvaOvhSjah+e+BMP8YqPajSHSpjsmnLecErVnJPPPFE+uUvf3nQxRK8+ss1tHzrmjgqLCugsoJXlDF069ZN2qeKxMNxZFmZTT8OE3t6zGyPAGe2GUjnH3ylw8uF3uDMBRbkY1ChO+S9ud\/TecErTmkYOnQoXX755bGIPvLII8HVwrKlCMWOFxNCttTGNdEnbb2visTDcWTmXihM7OawVuUJnKlC0owd8GUGZ5VewJlKNPXbUqE79EdpzoPzgpdXbq+66iq64IILaOTIkbHIjB07lv70pz9JXzwh6nXZWPQCiVL1uLKXTMQFqSLxxi5cS2MXrgvM4zgyvS8JJna9+OqwDs50oKrPJvjSh60uy+BMF7J67KrQHXois2PVecHLpzH84he\/oCeffJJ+\/etfB8JXXC7BP2Ohe\/PNN9NFF10kffGEgDp6tXD0lrXo+brR\/lHKSq0Mq0g8XCds7iXBxG4Oa1WewJkqJM3YAV9mcFbpBZypRFO\/LRW6Q3+U5jw4L3gZCj5L7tprr6U333wzuFyCjyLjxkek8ArwKaecQryxLXy+rTkI5TzlTTzU78rhrKoXJnZVSJqzA87MYa3CE\/hSgaJZG+DMLN55veXVHXn9uzbeC8HLoPGpDFyjy\/\/t2LEjwJHFb58+fYL\/mjZt6hq2deLJm3hRwYvrhPXSjYldL746rIMzHajqswm+9GGryzI404WsHrt5dYeeqOxZdV7wctnCY489RqeeeirxSQ2+tryJh\/pds8xjYjeLtwpv4EwFiuZsgC9zWKvyBM5UIWnGTl7dYSZKc16cF7x82xWfzvD9738\/8Wxcc7Cl95Q38VC\/mx7zPCMwsedBz85YcGYH96xewVdW5OyNA2f2sM\/iOa\/uyOLT5THOC15xSgPfpFZVVeUyliVjy5t4nce8RFzWwO3eKyrpirPKvcXCh8AxsfvAUt0YwZlfnIEvv\/jiaMGZX5zl1R1+PW1ytM4LXn6EZ599ln7+858HK73f+9736IgjjjjoyQ499FA6+uijif90seVJPNTvmmcUE7t5zPN6BGd5ETQ7HnyZxVuFN3CmAkVzNvLoDnNRmvPkvOAVF0\/wSQ2lGp\/QIHvxhDl4\/+0pT+JFL5z4aMI3bTxCg\/KJid0\/usGZX5yBL7\/4wgqvf3zl0R3+PW1yxM4LXj6dYcmSJcERZKXa4YcfTt27d3f2tIY8iYcNa8mJrLoHPoxVI6rfHjjTj7FKD+BLJZpmbIEzMzir8pJHd6iKwSU7zgtel8DKE0uexAtvWOPaXa7hRdOLACZ2vfjqsA7OdKCqzyb40oetLsvgTBeyeuzm0R16IrJrFYLXEP5ZEy9av7ty9DnBtcJoehHAxK4XXx3WwZkOVPXZBF\/6sNVlGZzpQlaP3ay6Q0809q06KXhF3S7Dc\/fdd9MNN9wQ3LZWqtXXGt5o\/S4unDDz0mBiN4OzSi\/gTCWa+m2BL\/0Yq\/YAzlQjqtceBG9dfJ0UvNu3b6f77rsviPSHP\/whPfTQQ8T\/Vqo1b96crrvuOuI\/XWxZEy8seHlll1d40fQjgIldP8aqPYAz1YjqtQe+9OKrwzo404GqPptZdYe+iOxadlLw2oVEj\/esiYcLJ\/TwkWQVE3sSQu79HJy5x0mpiMCXX3xxtODML86y6g6\/nlI+Wi8EL18v\/PTTT1NNTQ399Kc\/pSOPPJJ27NgRXERx2GGH0S233EInn3yy\/FNb6Jk18cKCd2SvdjSyV3sL0Tc8l5jY\/eMcnPnFGfjyiy8IXv\/4yqo7\/HtSuYi9ELxPPfUUDRs2jDp16kSTJ0+mli1bBiUOv\/71r+mJJ54gPpJs6tSp1KVLF7mnttArS+LhwgkLRH3uEh\/G9rDP6hmcZUXOzjjwZQf3PF7BWR70zI\/NojvMR2nOo\/OCd9euXTRo0KDgfF3ewFZWVlYHna1bt9KQIUOC29dYDLP4dbFlSTxsWLPHJCZ2e9hn9QzOsiJnZxz4soN7Hq\/gLA965sdm0R3mozTn0XnBK05suPbaa+nKK6+MRebBBx8MVnjr201rc17ZSEPnrA6eGRvWzL0U7AkTu1m8VXgDZypQNGcDfJnDWpUncKYKSTN2IHjr4uy84N28eTNdfvnldOmllwbHk8W1SZMm0cMPP0zz5s2j1q1bm8mklF6yJB42rKUEWWF3TOwKwTRkCpwZAlqRG\/ClCEiDZsCZQbAVuMqiOxS4ddaE84L3k08+Cep3V69eTdOmTaP27etu2lq7di1dc801VFlZSRMmTAg2sbnYsiQeNqzZYxITuz3ss3oGZ1mRszMOfNnBPY9XcJYHPfNjs+gO81Ga8+i84GUoXn\/9dRo4cCDt3LmTvvrVr9IJJ5wQILRlyxZ64YUXqFmzZnT\/\/ffT6aefbg65lJ6yJN7Rw54veMGFEykBz9kdE3tOAC0MB2cWQM\/hEnzlAM\/SUHBmCfiMbrPojoyuvBjmheBlJN9\/\/3361a9+Rc8++yzt27cvALdJkyb0rW99i2699VZq27at04CnTbzohjVcKWyWXkzsZvFW4Q2cqUDRnA3wZQ5rVZ7AmSokzdhJqzvMRGXPizeC1x5EajynTTxsWFODe1YrmNizImdvHDizh30Wz+ArC2p2x4Azu\/in9Z5Wd6S171t\/CF5DjKVNvOrlG2nIbJzQYIieg9xgYreFfHa\/4Cw7djZGgi8bqOfzCc7y4Wd6dFrdYTo+0\/4geA0hnjbxwhvWvntqK3rox+7WJxuC0KgbTOxG4VbiDJwpgdGYEfBlDGpljsCZMiiNGEqrO4wEZdEJBK8h8NMmXnjDGq4UNkRSyA0mdvOY5\/UIzvIiaHY8+DKLtwpv4EwFiuZspNUd5iKz4wmC1xDuaRMPJzQYIqaIG0zsdvHP4h2cZUHN3hjwZQ\/7rJ7BWVbk7IxLqzvsRGnOKwRvBqz5XOD58+cXRvKVxj179ixpKU3i4YSGDKQoHoKJXTGgBsyBMwMgK3QBvhSCacgUODMEtCI3aXSHIpdOm4HgTUkPi93ly5cXrjFetGgRDRkyhIYPH05VVVVFraVJvLEL19LYhesCW7hSOCVBirpjYlcEpEEz4Mwg2ApcgS8FIBo2Ac4MA57TXRrdkdOVF8O9Ebx8o9of\/\/hHeu+992KBbd68OV133XXEf+pqq1atov79+9O4cePqrOiyCK6traXp06dTWVlZrPs0iYcrhXUxKG8XE7s8Vq70BGeuMCEXB\/iSw8mlXuDMJTaSY0mjO5Kt+d\/DC8H71FNPBdcLiwsn4mCvqKgorLqapkWn4P1d\/1Pp0s7Hmn6kBu8PE7t\/KQDO\/OIMfPnFF0cLzvziDIK3Ll\/OC95du3bRoEGD6IMPPqDf\/va3dOqpp1KjRo2cybpJkybR+PHjKamOVyTe7NmzicW5aOXl5Qc9y7E3v1D4t4k\/6EhXnHWcM8\/bUALBxO4f0+DML87Al198QfC6z9fGjRvrBLl+\/Xrq168f1dTU1NEd7j+JngidF7xMYJ8+feiqq66ia665Rg8KGayK2l0e2rt3b5owYUJJK0LwRjtxicSAAQMK\/1y741O6eOb6wv9Pvayczmz7hQwRYkgeBPbu3UubN28m\/oWkcePGeUxhrCEEwJkhoBW5AV+KgDRoBpwZBDuDq5kzZ9KsWbMOGgnBewAS5wXv1q1baeDAgXThhRc6JXhFRokVaK7hra6uDgRSXBOCl+t\/27ZtW+jC\/cNjFq\/ZSn3uf\/PfAviOczKkPYbkRWDPnj20adOm4LdiCN68aJoZD87M4KzKC\/hShaQ5O+DMHNZZPPECYXiVd+nSpTRx4kSs8H4OpvOCl+OcOnUqPf3008GfrVu3zpIHWseIzWyDBw8uelKDbC0NTmjQSpW0cXzdKg2VMx3BmTNUSAUCvqRgcqoTOHOKjsRgZHVHoqF60sF5wcu\/Uf7pT3+i3\/\/+9\/Tmm2\/S+eefT82aNTsIfhOnNBTjXKXgHTpnNc155UAdTvcOLeiJoV3qSar59RiY2P3ii6MFZ35xBr784gvvmH98QfDW5cx5wStqeLn4ulQzcUoD1+2OGDEiqJHp1KlTIRxRz1tq45ps4oWPJMOVwvYmGHwY28M+q2dwlhU5O+PAlx3c83gFZ3nQMz9WVneYj8yOR+cFrx1Y4r2Kel3+qThzV6zuVlZWKjmHN3ylMASvPfYxsdvDPqtncJYVOTvjwJcd3PN4BWd50DM\/FoLXsxVe8ymS7DF6tXDSLWtsUSbx3vvoY+o85qVCAI9XdaFzO7ZIDgg9lCOAiV05pNoNgjPtECt1AL6UwmnEGDgzArMyJzK6Q5kzDwx5s8L7zjvv0JgxY+ill16iY489NjgR4YgjjqBbbrmFvvOd79DFF1\/s1Pm8Ue5lEm\/xmm10yaQVhaErR58TXC2MZh4BTOzmMc\/rEZzlRdDsePBlFm8V3sCZChTN2ZDRHeaise\/JC8H7+uuvB0eTHXLIIdSuXTvasGFDIHgPP\/zw4N9Xr14dXErRs2dP+4gWiUAm8XizGm9a48ZClwUvmh0EMLHbwT2PV3CWBz3zY8GXeczzegRneRE0O15Gd5iNyK435wXvJ598ElwrzJvW+FiyFStWBCu94szbHTt2BOfz8movbxpjEexik0k8HEnmDnOY2N3hQjYScCaLlBv9wJcbPKSJApylQct+XxndYT9KcxE4L3j5tqvLL7+cvv\/97wdn3PKJCGHBy1BNmzaNHnjggZIXP5iDNN6TTOKFT2jAkWR2GcPEbhf\/LN7BWRbU7I0BX\/awz+oZnGVFzs44Gd1hJzI7Xp0XvOJYsmuvvZauvPLKWMH74IMPBqu\/pW46swPvv73KJB6OJLPN0r\/9Y2J3hwvZSMCZLFJu9ANfbvCQJgpwlgYt+31ldIf9KM1F4LzgFUeBtWrViiZMmBBcQhFX0nDYYYfRlClT6MgjjzSHXgpPSYkXPaHh3isq6Yqz4q8pTuEWXTMigIk9I3AWh4Ezi+BncA2+MoBmeQg4s0xASvdJuiOlOe+7Oy94GeGnnnqK+OgvXuE94YQT6L777gvqdbmu95577gk2rY0dOzYoe3C1JSUejiRzizlM7G7xIRMNOJNByZ0+4MsdLmQjAWeySLnRL0l3uBGluSi8ELz79+8PbjcbN24c7d69uw46hx56aLCpjUse+O+utqTEw5FkbjGHid0tPmSiAWcyKLnTB3y5w4VsJOBMFik3+iXpDjeiNBeFF4JXwMEnMixbtiz4b9++fXTmmWfSueeeSy1btjSHWEZPSYmHI8kyAqtpGCZ2TcBqNAvONIKrwTT40gCqZpPgTDPAis0n6Q7F7pw355XgdR7NEgEmJV74SDKc0GCfaUzs9jlIGwE4S4uY3f7gyy7+WbyDsyyo2RuTpDvsRWbHszeCl8sa3n77bfrDH\/5AO3fuDNDijWyXXXYZtW\/f3g56KbwmJR6OJEsBpoGumNgNgKzYBThTDKhmc+BLM8AazIMzDaBqNJmkOzS6dtK0F4KXSxl+8pOfBJvXWPiGW6NGjahv37502223UZMmTZwEmYNKSrzOY14i3rjGbWSvdjSyl\/si3lmwFQSGiV0BiIZNgDPDgOd0B75yAmhhODizAHoOl0m6I4dpL4c6L3hZ4PJmtfvvv5+GDh1KAwYMoKOOOioAm4UwXzrBZ\/DedNNNwcY1V1tS4h097PlC6I9XdaFzO7Zw9VEaRFyY2P2jGZz5xRn48osvjhac+cVZku7w62nyR+u84N2yZQtdddVVdNZZZ9Htt99OvKIbbiyIeXX39ddfD0SxqxvYSiUejiTLn8iqLWBiV42ofnvgTD\/GKj2AL5VomrEFzszgrMoLBG9dJJ0XvOKmNV7d5SuG49oTTzwRrAL7etMajiRT9Xqrs4OJXR2WpiyBM1NIq\/EDvtTgaNIKODOJdn5fELyeCd69e\/fSDTfcENTn8k1r0TpdPp5s1KhRQXnD3XffTYcffnj+LNFgoVTi4UgyDYDnNImJPSeAFoaDMwug53AJvnKAZ2koOLMEfEa3ELyeCV4Ol29U4\/rc448\/nm6++WZq164dHXLIIcSrv3zT2qJFi+hXv\/oVnXbaaYWn40soWrdunTFN1A8rlXgPLdtAN8x9M3D6paO\/QCtHn6M+AFhMhQAm9lRwOdEZnDlBg3QQ4EsaKmc6gjNnqJAKBILXM8ErShpY9KZpFRUVVFNTk2aI1r6lEg9HkmmFPpNxTOyZYLM6CJxZhT+1c\/CVGjLrA8CZdQpSBQDB65ng3bNnDy1ZsoS4tCFN49KGb33rW2mGaO0rK3hxJJlWGqSNY2KXhsqZjuDMGSqkAgFfUjA51QmcOUVHYjAQvJ4J3kRGPelQKvFwBq97JGJid4+TpIjAWRJCbv0cfLnFh0w04EwGJXf6QPB6LHi3bt1KixcvpldffTXYvNalSxf62te+5uxRZGGoiyUejiRzZ3IIR4KJ3U1eSkUFzvziDHz5xRdHC8784gyC10PB+9lnnwWXS\/ApDfz3cOPNaXxkWVVVlZc3rUHwujmBYGJ3kxcIXv94KRYx3jH\/uARnfnEGweuh4H344Ydp5MiRdP755wc3qvEpDdzWrVtHd955J7344ovBObyXXHKJs9lYLPGiZ\/B+NOGbzj5DQwoME7t\/bIMzvzgDX37xhRVe\/\/iC4PVM8O7atYsGDRpEZWVlwTm7TZs2rfMEvKmNz+nlld\/Jkyd7dw7v2IVraezCdcEz4UgydyYUfBi7w4VsJOBMFik3+oEvN3hIEwU4S4OW\/b4QvJ4JXnEsGZ\/De+WVV8Zm0IMPPhiUPPh409qsl2vpxuq3gufq3qEFPTG0i\/23BBGgVs3DHMCHsV+kgS+\/+MIKr398QfB6Jng3b94cXCl86aWXBiu5cY1Xfh999FGaN2+e9GUTYuV42bJlBZPDhw8PaoGT2qRJk2j8+PGpxhVLvPAZvBec1JIeGdI5yT1+bgABfBgbAFmxC3CmGFDN5sCXZoA1mAdnGkDVaBKC1zPB+8knn9CwYcNo9erVNG3aNGrfvn2dJ1i7di1dc801VFlZGWxqO+ywwxLTR6wat2nThqZPnx6US6xatYr69+9PPXr0COwUayx2p0yZQrNmzaJOnToVxg0ePLikWJYRvDiDN5E6Yx0wsRuDWpkjcKYMSiOGwJcRmJU6AWdK4dRuDILXM8HL4b7++us0cODA4PKJb3\/729S9e\/fgKfhCimeeeSao2+WSBj6mTKbxVcQjRowoiFYxhsXs3Llzi5ZGCKHctWvXOqKYBfny5ctLllQUS7yjhz1fCBmCV4Y9M30wsZvBWaUXcKYSTf22wJd+jFV7AGeqEdVrD4LXQ8HLIb\/11ls0atQoWrlyJe3fvz94ikaNGlHnzp3pjjvuoJNPPjl35kRXb6MGVQteHEmWmzJtBjCxa4NWm2Fwpg1aLYbBlxZYtRoFZ1rhVW4cgtdTwSvC5lVevoCCW8uWLZWeyiCzUluspCGpFCIu8SB4lb\/fygxiYlcGpTFD4MwY1EocgS8lMBo1As6Mwp3bGQSv54I3dwYUMcBlDkOGDCGZjWui3nfnzp2BNT4OrWfPniVDE4k3e\/ZsqqioCPqu+7iMLpm0ojDug1+fp+vxYDclApjYUwLmQHdw5gAJKUIAXynAcqQrOHOEiCJh8LfQ4bZ+\/Xrq168f1dTUFHSH20+gN7pG+0V9gF4\/TlsXApY3volNbHEBi5MdamtrC\/W6cRvg4sYKwRv+2b4vdafdXx0Y\/FOboxrTEwMOCGE0+wjwNwl8Qkh5eTk1btzYfkCIIBEBcJYIkVMdwJdTdEgFA86kYLLWaebMmcHepGiD4D2ASIMXvLJil8ESq8DRFV1hg297K7bSKwQv92nbtm0A\/kNvfEoPvfFJ8He+dOLl4XKb7qy9TQ3IMV9osmnTpuC3YgheP4gHZ37wJKIEX37xxdGCM7c54wW48Crv0qVLaeLEiVjh\/Zy2Bi14hYDt1q1byZVdkeLFTncQq7x9+\/YtejRZXC3N8D+8TfcveT8wj0sn3JpI8NWdW3zIRAPOZFBypw\/4cocL2UjAmSxSbvRDDW9dHhqs4BVit3fv3iXP3Q3DpWKFN\/zVQvjSiSvOKqd7r6h04y1BFLhpzcMcwIexX6SBL7\/44mjBmV+cQfBC8EpfMhFNbRU1vGHB23nMS8QnNXDDGbxuTSSY2N3iQyYacCaDkjt9wJc7XMhGAs5kkXKjHwQvBG9wc9v8+fOLZqSo0S12Lm90vMwqcVzihS+d4NVdXuVFcwMBTOxu8JAmCnCWBi37fcGXfQ7SRgDO0iJmtz8ELwSvlQyMJh7O4LVCg7RTTOzSUDnTEZw5Q4VUIOBLCianOoEzp+hIDAaCF4I3MUl0dEgSvCtHnxOc1IDmBgKY2N3gIU0U4CwNWvb7gi\/7HKSNAJylRcxufwheCF4rGRhNvMVrttW5dOKjCd+0EhecxiOAid2\/zABnfnEGvvzii6MFZ35xBsELwWslY6OJN3bhWhq7cF0QC6\/s8govmjsIYGJ3hwvZSMCZLFJu9ANfbvCQJgpwlgYt+30heCF4rWQhBK8V2DM7xcSeGTprA8GZNegzOQZfmWCzOgicWYU\/tXMIXgje1EmjYkA08cJn8OLSCRUIq7WBiV0tniasgTMTKKvzAb7UYWnKEjgzhbQaPxC8ELxqMimllVKCF5dOpATTQHdM7AZAVuwCnCkGVLM58KUZYA3mwZkGUDWahOCF4NWYXsVNRxMPl05YoUHaKSZ2aaic6QjOnKFCKhDwJQWTU53AmVN0JAYDwQvBm5gkOjpEEw+XTuhAWZ1NTOzqsDRlCZyZQlqNH\/ClBkeTVsCZSbTz+4LgheDNn0UZLIQT719HtCJe4RXt8aoudG7HFhmsYoguBDCx60JWn11wpg9bHZbBlw5U9doEZ3rxVW0dgheCV3VOSdmD4JWCyZlOmNidoUI6EHAmDZUTHcGXEzSkCgKcpYLLemcIXgheK0lYSvDi0gkrlJR0iondPU6SIgJnSQi59XPw5RYfMtGAMxmU3OkDwQvBayUbw4n3wobGNHTO6iAOXDphhY5Ep5jYEyFyrgM4c44S\/FLpFyWJ0eIdS4TIqQ4QvBC8VhIynHgPvfEJblmzwoK8U0zs8li50hOcucKEXBzgSw4nl3qBM5fYSI4FgheCNzlLNPQIJ96vXthJc17ZGHjBpRMawFZgEhO7AhANuMZMKgAAHTBJREFUmwBnhgHP6Q585QTQwnBwZgH0HC4heCF4c6RP9qHhxLvusc205J1tELzZ4dQ+EhO7doiVOwBnyiHVahB8aYVXi3FwpgVWbUYheCF4tSVXKcPFBO\/IXu1oZK\/2VmKC0+IIYGL3LzvAmV+cgS+\/+OJowZlfnEHwQvBaydhw4l004x\/03kcfB3Hce0Ul8dXCaG4hgIndLT5kogFnMii50wd8ucOFbCTgTBYpN\/pB8ELwWsnEcOKdMeFvhRhw6YQVOhKdYmJPhMi5DuDMOUpKBgS+\/OILK7z+8QXBC8FrJWtF4j04fxHxCq9oELxW6Eh0ig\/jRIic6wDOnKMEgtcvShKjxTuWCJFTHSB4IXitJKRIvF\/9\/nHiTWuirRx9TnAWL5pbCGBid4sPmWjAmQxK7vQBX+5wIRsJOJNFyo1+ELwQvFYysZjgxS1rVuhIdIqJPREi5zqAM+cowQqvX5QkRot3LBEipzpA8ELwWknIuJIG3LJmhQopp5jYpWByqhM4c4qOxGDAVyJEznUAZ85RUjIgCF4IXisZKxLvez+ZRXzTGjcIXitUSDnFxC4Fk1OdwJlTdCQGA74SIXKuAzhzjhII3hSUNNq\/f\/\/+FP3RNSMCQvCeft1UemHDoYEV3LKWEUwDwzCxGwBZsQtwphhQzebAl2aANZgHZxpA1WgSK7xY4dWYXsVNi8Rr0Xcirfu4DILXCgvyTjGxy2PlSk9w5goTcnGALzmcXOoFzlxiIzkWCF4I3gCBXbt20aBBg2jZsmUFRIYPH05VVVWJWbRo0SIaMmRIoV+3bt1o+vTpVFZ2QMjGtTjBi1vWEqG21gETuzXoMzsGZ5mhszIQfFmBPZdTcJYLPuODIXgheGnjxo3Up08fatOmTUGorlq1ivr37089evSgCRMmFE3MSZMm0fjx42ny5MnUs2fPWFulBO+OnmPpX0e0CrpA8Bp\/\/6UdYmKXhsqZjuDMGSqkAgFfUjA51QmcOUVHYjAQvBC8xCu0I0aMoFmzZlGnTp0KiLCYnTt3LlVXV1N5+cHX\/Qqh3Ldv3zorwcXshaEWibet9+8K\/4xLJxLfV2sdMLFbgz6zY3CWGTorA8GXFdhzOQVnueAzPhiCF4K35OrtlClTDhLCYgAL2zFjxhQVxKWymROv76D\/Il7hFQ2C1\/j7L+0QE7s0VM50BGfOUCEVCPiSgsmpTuDMKToSg4HgheAtmiTDhg2j5cuXFxW0vAJcU1NDd9xxB1199dW0fv36wFbv3r1LlkFwHwjexHfTqQ6Y2J2iQyoYcCYFkzOdwJczVEgHAs6koXKiIwQvBG9sIoqNaKU2rrEgnj9\/PjVr1qywChxXDxzngBOvz3\/dTrvOvbnw4yevPp66faWdEy8GgqiLACZ2\/zICnPnFGfjyiy+OFpy5zRnrkXDjRbl+\/foFC3UVFRVuB28gOpzDS0Riw1plZWXJ0xaE4BUb1gQ\/QixH\/z3MX5zgbTH\/x8FGuQEDBhigGi7SILB3717avHlzUMvduHHjNEPR1xIC4MwS8Bndgq+MwFkcBs4sgi\/heubMmcFiXLRB8B5ApMELXlmxy2Cx4H3uuecOqvEttpktKngv+8U8+viUS4J\/5lvWJn+rUSCo4jbISeQ2umhEYM+ePbRp06bgt2IIXo1AKzQNzhSCacAU+DIAsmIX4EwxoIrNsRYJr\/IuXbqUJk6ciBXez3Fu0IJXrMzKnKPLeHENb9ymtqyCd+XocxSnO8ypQgBf3alC0pwdcGYOaxWewJcKFM3aAGdm8c7rDTW8dRFssIJXiF2ZDWcCMrEaPHjw4EzHkvUe\/yzt+1L3wByuFc77Kusdj4ldL746rIMzHajqswm+9GGryzI404WsHrsQvBC8hZrdpEsm4lIwWtYgVne7du1a8qQGTrxLJq2kT1udHJi94qxyuveKSj1ZDqu5EcDEnhtC4wbAmXHIczkEX7ngszIYnFmBPbNTCF4I3qAWl09bKNbE5rNiJQzitjUxXmaVOCp4ccta5nfYyEBM7EZgVuoEnCmFU7sx8KUdYuUOwJlySLUahOCF4NWaYMWMc+JdNOMfuFbYCvrpnWJiT4+Z7RHgzDYD6fyDr3R4udAbnLnAgnwMELwQvPLZorAnJ96F1XsKFrmcgcsa0NxEABO7m7yUigqc+cUZ+PKLL44WnPnFGQQvBK+VjI0KXj6hgY8mQ3MTAUzsbvICwesfL8UixjvmH5fgzC\/OIHgheK1kbFTwPl7Vhc7t2MJKLHCajAAm9mSMXOsBzlxjpHQ84MsvvrDC6x9fELwQvFayFiu8VmDP7BQfxpmhszYQnFmDPpNj8JUJNquDwJlV+FM7h+CF4E2dNCoGRAXvRxO+qcIsbGhCABO7JmA1mgVnGsHVYBp8aQBVs0lwphlgxeYheCF4FaeUnLmw4OXaXdyyJoebrV6Y2G0hn90vOMuOnY2R4MsG6vl8grN8+JkeDcELwWs65wJ\/ELxWYM\/sFBN7ZuisDQRn1qDP5Bh8ZYLN6iBwZhX+1M4heCF4UyeNigFhwYtrhVUgqtcGJna9+OqwDs50oKrPJvjSh60uy+BMF7J67ELwQvDqyawEqxC8VmDP7BQTe2borA0EZ9agz+QYfGWCzeogcGYV\/tTOIXgheFMnjYoBYcHLF07wxRNo7iKAid1dbopFBs784gx8+cUXRwvO\/OIMgheC10rGhgXvyF7taGSv9lbigFM5BDCxy+HkUi9w5hIbybGAr2SMXOsBzlxjpHQ8ELwQvFYyFoLXCuyZnWJizwydtYHgzBr0mRyDr0ywWR0EzqzCn9o5BC8Eb+qkUTEgLHi5nIHLGtDcRQATu7vcFIsMnPnFGfjyiy+OFpz5xRkELwSvlYwNC15cK2yFglROMbGngsuJzuDMCRqkgwBf0lA50xGcOUOFVCAQvBC8UomiuhMEr2pE9drDxK4XXx3WwZkOVPXZBF\/6sNVlGZzpQlaPXQheCF49mZVgNSx4+ZY1vm0NzV0EMLG7y02xyMCZX5yBL7\/44mjBmV+cQfBC8FrJ2LDg\/WjCN63EAKfyCGBil8fKlZ7gzBUm5OIAX3I4udQLnLnERnIsELwQvMlZoqGHELy8sssrvGhuI4CJ3W1+4qIDZ35xBr784gsrvP7xBcELwWslayF4rcCe2Sk+jDNDZ20gOLMGfSbH4CsTbFYHgTOr8Kd2DsELwZs6aVQMEIK3e4cW9MTQLipMwoZGBDCxawRXk2lwpglYTWbBlyZgNZoFZxrB1WAagheCV0NaJZvkxLtoxj\/onNM7QvAmw2W9ByZ26xSkDgCcpYbM6gDwZRX+TM7BWSbYrA2C4IXgtZJ8SDwrsGd2iok9M3TWBoIza9Bncgy+MsFmdRA4swp\/aufQHRC8qZNGxQAkngoUzdnAxG4Oa1WewJkqJM3YAV9mcFbpBZypRFO\/LegOCF79WRbjAYlnBfbMTjGxZ4bO2kBwZg36TI7BVybYrA4CZ1bhT+0cugOCN3XSqBiAxFOBojkbmNjNYa3KEzhThaQZO+DLDM4qvYAzlWjqtwXdAcEbILBr1y4aNGgQLVu2rIDI8OHDqaqqKlUWTpo0iebOnUvV1dVUXl5edCwSLxWs1juvW7eOZsyYEeRIRUWF9XgQQDIC4CwZI5d6gC+X2JCLBZzJ4eRKL+gOCF7auHEj9enTh9q0aUPTp0+nsrIyWrVqFfXv35969OhBEyZMkMpXMaZ58+YQvFKI+dMJE4U\/XIlIwZlfnIEvv\/jiaMGZX5yBLwheWrRoEY0YMYJmzZpFnTp1KiAiu1obXSHmFUCs8Po1ESRFi4kiCSH3fg7O3OOkVETgyy++IHjBl38IQPAW5YwF75QpUw4SwnEDuG9NTQ116dKFFixYAMHr+5sQiR8fxv4RCs784gx8+cUXBC\/48g8BCN6inA0bNoyWL1+eKF7DK8RLlixJVcM7e\/Zs1IR68NasX7+e+vXrR+DLA7I+DxGc+cMVRwq+\/OILnPnLFy\/OYS8KUaP9+\/fv949G9RGziB0yZAglbVwT9b99+\/YNNrjJlkHw5M5lFEuXLlUfPCwCASAABIAAEAACQCCCwNlnn01z5swBLgTBGySB2HxWWVlZ2MRWLDt4Fbi2trbQT1bwit+OWfiiAQEgAASAABAAAkBANwK8sovV3QMoN\/gV3jRiN26zWxrBqzuxYR8IAAEgAASAABAAAkDgYAQatOAVZQzdunVLXNll6Hh1d\/78+UXzKKkcAgkIBIAAEAACQAAIAAEgYB6BBit4hdjt3bu39Lm7cfRghdd80sIjEAACQAAIAAEgAATSINAgBW+WSyaKgQrBmybd0BcIAAEgAASAABAAAuYRaJCCN6k0YfLkydSzZ8\/gBIakc3kheM0nLTwCASAABIAAEAACQCANAg1S8KYBCH2BABAAAkAACAABIAAE\/EYAgtdv\/hA9EAACQAAIAAEgAASAQAICELxIESAABIAAEAACQAAIAIF6jQAEb72mFw8HBIAAEAACQAAIAAEgAMGrOQd27dpFgwYNomXLlgWe+MaT6upqKi8v1+wZ5pMQCG9ebNasGc2aNYs6depUcpg4zk50Ap9JKKv7eRa+wt6j14KriwyWiiGQhbPonMm2xUZiIK0XgSx8ifdK3CKKOVEvR1msM6\/cJkyYkGV4vRkDwauRSjFxt2nTppBonHjLly+H6NWIu4zpuCuiZU7kGD9+fJ0PX7bz3HPPSYllmbjQJx6BLHxFLYkPc1wQYybLsnAmxBPPmdOnT6eysrLgtJzoe2fmCRqWlzx8de3aFZ9xjqaLeH\/y3jng6OOlCguCNxVc6TrHHWuGVaZ0GOroLVZpw6tGcb+chH0X+zn41MFQXZtZ+IpGFV6Zh+B1l7O4Yx6T3k39T1P\/PWR9x+I+48Q594MHD6aqqqr6D56jTxj9pgSClwiCV2OyRn9jFq6K\/bvGUGA6hECxs5OznKksBG94hQNgq0UgL1+CIy4t4lXDvn374oNYLUUHWcvCmfiAPv\/888GPZn6i5rPwxTYgeA0TJelOvEu1tbU0Y8YMGjVqFIW\/aZY0U++6QfBqorTUqgTKGjSBLmm22C8cMheNRF1gNUMS9Bzd8vAVfg9vvvlm6tOnDwRvDi5kh2bhTPxiMnr0aFqzZk1QxsBNtr5eNjb0OxiBLHyxlbhf+FHm5VaG4RuSf\/MBwaspN0slWZaVRE1hNkizxSZ3\/lpvxIgR0vW44d+isRFRXyrl4Sv8rnGEELz6eApbzsKZ+OVx586dFP76FTW8+jnLwleU7\/nz5wf\/1K1bt0L9tf7I4SEJAQheCN6kHMn9cwje3BBqM5B3cheBiU1Q2EGujarAcFa+hIAaN25ccFU46q318qRK8FZWVtYRTGIuZftiI5u5J2kYnrK+Y6L2N1wXj19Q3MoZCF4IXu0ZiZIG7RBndpD167u4FQ2I3cw0SA\/Mwlfc+wfBKw157o5ZOBO\/oPTo0eOg45PwrVhuSkoayMJXqV9EsE9FL19prEPwQvCmyZfMfbFpLTN0Wgdm3aDBQYV3vkLsaqWpYDwLX+Gvx+OixFmhernLwlmpDaAQvO7xhW8x9XKiyjoELwSvqlwqaSduksYqkxHoSzqJq9WVmRREn9WrV0vX+dp\/Wv8jyMpX9Mnx7pnLhaycxW3olXk3zT1Z\/fSUhS+s8PqRC3h\/IHiNZGrcIeo4ocEI9CWdxG02kzmhAbuP7XCXlS8IXjt8hb8J4WORxIZOmXcsrqxBZpy9J60fnrO+Y6jhdZ9\/CF4IXmNZGj38GV+lGoM+0ZHYdMYd444+Cv9ysmnTJurfvz\/xDvK4hp3JiXDn7pCGr7iru7HCm5uC1AaycBa9qhbHkqWGPfOALHxFy4fAV2b4tQyE4IXg1ZJYMAoEgAAQAAJAAAgAASDgHgI4h9c9ThAREAACQAAIAAEgAASAgEIEIHgVgglTQAAIAAEgAASAABAAAu4hAMHrHieICAgAASAABIAAEAACQEAhAhC8CsGEKSAABIAAEAACQAAIAAH3EIDgdY8TRAQEgAAQAAJAAAgAASCgEAEIXoVgwhQQAAJAAAgAASAABICAewhA8LrHCSICAkAACAABIAAEgAAQUIgABK9CMGEKCAABIAAEgAAQAAJAwD0EIHjd4wQRAYEGhcBnn31Gjz32GO3Zs4d++MMfpn72999\/n6ZPn05VVVXUunXr1OPzDHj22Wfp5z\/\/OfEVum3btqW5c+cGf6Zt4naxrl270oQJE9IOd65\/9La04cOHB\/zYaNGbwCZPnkw9e\/a0EQp8AgEgYBEBCF6L4MM1EAACRHmv\/J00aVIgNKurqynuSmFdGO\/YsYOuueaaQOwOGTKEvvjFL1L37t2padOmqV3WV8F73HHHBVdyn3jiiXTSSSelxkXFgK1bt9LSpUvpf\/\/3f4NfjCB4VaAKG0DAPwQgeP3jDBEDgXqFgK+CV6VIVWnLheRw8XkWLVoU\/GICwetChiAGIGAeAQhe85jDIxBoMAjs37+fnnjiCbrzzjtp\/fr1dNhhh1Hnzp3p9ttvp5NPPpmiXzc3a9aMZs2aRZ06daJdu3bRPffcQ\/Pnz6cPPviAGjVqRBUVFXTTTTfRxRdfHPz\/sGHDgp+L1rt370JJwFtvvUU\/+9nPgpU9bmeccQbdfPPNdPbZZyfiz2USd999Nz355JO0e\/duatOmDf34xz8OSi6aNGlCQjyFDZX62p5xePrpp+k3v\/kNvfvuuwEO55xzDo0ePZo6dOhQWOXu0qULdevWjX7729\/S5s2bA78cs3he9rdv3z566KGHaMaMGQGmbPvYY4+lgQMH0tVXXx3Ex02sfP\/oRz8K\/P7rX\/+iX\/ziF3TZZZfRO++8E\/z9xRdfpEMPPZSuuOIKOv300wNeBP5sg8tM2M\/9999PH374IR1zzDH0n\/\/5n3T99ddTWVlZURzjBC\/zOWjQoOCZLrjgAvrlL39JW7ZsCfJg1KhRweo4c1qsFVvJl13hh+BNTHt0AAL1GgEI3npNLx4OCNhFYMGCBYFA\/cY3vkG9evWibdu20e9+97tA2MyePZtY4LIQGT9+PH3961+n7373u4EgPfLIIwMx+6c\/\/Ym+\/\/3v01lnnUVr166lOXPmBMJrypQpgc1XX301EGgs3G688UY65ZRT\/v\/2zh8kqy+M4weXwkEqWs0MGoqmWhwy0HKpQBq0iMKtiIhwiHJqSEhs0SmoISIXwQwcgogK+kNSQdFQUA2hexDiEDX8+DzwvJzu797ref2T76vfAxHkfe8553vf5HO\/9\/s8N+zbt8\/Oyee3b98eTp06ZSKMjY2Fb9++GRAfPny4UBggEnAEOPm7paXFoPrVq1eht7fXoBBQe\/bsWRgZGQk7duwofWwPkOIqssddu3bZegBC1rNly5Zw584dg07Ozb8TA+jr6wsbNmww0GQdaIYGnOvGjRvh9u3bBsEHDhwIP3\/+tHMB0levXrW1OPByo7F58+Zw5swZm6Ozs9Ngm31xLVhLU1OTQS1wC0zHNxyA7bt37yrX4O3bt2FiYsJuWgBNPps3yoD38+fPtoaenh7TlvlYO5Bflq0V8K7u\/2XNLgXqXQEBb71fQa1fCtSwAkAnbimA5nD08uVLg9HBwUEDnLxIA+4sRU64iXGxE8CF08qjaf\/3LAiRrQXoyPMCfJ6pBeaYl8wtgJcHawAl0EgRHbAJPDMorANYcVYdPlMf28\/MzISTJ08aJALb7sBOT0+HCxcuhIGBAYN9gBcX9t69e6G1tdXm9f2SFeZYQPvs2bOWh8WhBRwZPgfusBe9oQtrvn79ejh+\/Lgd5\/vDbb5161bAUWbEkO\/AC+TjkHOe\/fv3V75lrBuAZn4c9WqB98OHD3\/ddJCxxZ3mJqjoujjA52W15fDW8C8ALU0K1JACAt4auhhaihRYawoAnMAuwIbzmNdFoZoMrx975MiRcPnyZZMrCzzv3783h\/To0aP2mDweOL+4xvFj+\/jnAOXp06cNOEdHRytAyTE4k+wBx5m5U4GXTg7AOfGMIgfTzwWA4hr7o\/3UOTwugGPs60YXnPB4r2X7I8JBURfHE\/\/AMcdBx+XFifeBM8wa29vbCztKlDm8nId54kgEa+V7Auzv2bMn97+BHN619ttB+5EC\/1YBAe+\/1VuzSYF1pQDuroMTGydrSmwBcHQXswx4f\/36ZY+7gc03b94E3GEc2jirmwWhvHxtLDowCQgeOnTof9ciD6j9oCzEpcIoIAfsFkE25y86V9G\/41aj7adPnwxKX7x4YToRe3CYzANePoPbyx8c4+zNwKVLl2yd5IrJ26J50cCVRkfiJ9mxUIY323otJV8r4F1Xvzq0WSmw7AoIeJddUp1QCkiBWAEeowNj4+Pj4dGjR5XiNc\/S5gEvQMfjePKtxAmIH9DflmK2x48f\/+UuFgHvYqrxVwJ488AzBRDzQBgtAVJyvBTTNTY2WuaX\/r3AKQ56GfB+\/\/7dCu9OnDiRBLx5bmzKt3sxwHvu3LnCGxHmFPCmKK9jpIAUKFJAwKvvhhSQAv9UgS9fvlgGl4IloJQMJ\/lVIMxzuU+fPg0AENnO8+fPVx5\/O7BR2BZnVeNsJ+BHhpdYg8ceUjeYEmnAncYdTXV4iyINFKMRn+jq6jIIRYPsiyeyc3isgv1fu3bNCtIYnoOl0K0MeMv2F4P57t27w8WLF62LBrlliv+qGWXAS6aa685afTA3NzfMVdSvt+gYuliQK16oD3OKi1zNHnWsFJAC9aWAgLe+rpdWKwXqRoG5ubnQ399vDi2w4sVjP378MBjdunVrIfByPFB09+7dSmEV7iZQQwur7u7uQuD1ojWypny+ubnZNMM15rO0KaPwzCMVsaALFa0BZRSz8Tg\/FXiLitampqas5RjdCcjMpgCvQxt5W3LMPoh6cLMAqJYBb1HRmgMzTnxctEaHDSIp5Hg9V0yEgsI5Wpnx87yxUJeGuCCQlnN8H4i7cM2LXtxBJwog\/+bNm9ZtgjE7O2uf\/fPnj4C3bn4zaKFSYHUUEPCuju6aVQqsCwXoBEA0gW4HQCrjwYMH5hx6Gyp3HfkZri7FUB8\/fjSoIsaAywsE0TkB95ZOBsCeO7wOQoAPrc+Y6+HDh9aRgcIozkPRFZ8H1oBwHOainq8pbcnotJAKvHFbMpxZit6+fv1qBVp79+61mwGKzlKAF4eXfaIB+yLC8Pz5c2vD9vv3b7s5KANeNMYlx1mmDRltyTgHAMq+N27cWAFebhyA6NevXxtgojmZ4cnJybBp0ybrqEBf5GqBl2tIcV08N3PFXSMc7OPexr53OlPQqYPBDQ1rAdjd4fXrQr\/fuDhODu+6+JWjTUqBQgUEvPpySAEpsGIK4O4SNwBmACpG9kUDACHwBxjPz89bjvPgwYP2worh4WErUiOrSt9dgJjOD\/Se9RZWFGIBt\/SL5WUOXkjF62T5PPAMIAJnPKYHvOlJWzbyXjxBjOHYsWOVtmKpwMs82RdPAPAArkN5NUVr9BympRut2\/xFHhSb4Rg\/efLE+htv27bNQDrbpcH3zIsn3O32F0+wJiIFcXGdv\/zj\/v371v+Y69DR0RGuXLliNyNFo8zhRQuuJW45LjzQT4szvhc+8oCXn8V7B5rp\/sF1HRoaEvCu2P9inVgKrA0FBLxr4zpqF1JACkiBJSlATILCQv6UwWzKJNW2JUs551KPkcO7VAX1eSlQ3woIeOv7+mn1UkAKSIFkBWjzRh63oaHBnHLvhUukALeUmEe2R27yyaMDBbyLUU2fkQJSYCUVEPCupLo6txSQAlKgxhQgskAkoq2tzfoZU8yX+trl1K048NIyja4WO3fuDGRq6e3LWA6oTl0L+V7iLRQrMu9i2tWlzqXjpIAUqF0FBLy1e220MikgBaTAsitArpoCPt7IRq4at5cuEeSAKaorKuarZiEOvJ7bpvgM8F0N4KVAkrnpGsIQ8FZzJXWsFFg7Cgh418611E6kgBSQAlJACkgBKSAFchT4D8LkWCrziQJlAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:85a19b40]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"     1.320000000000000e+01"}}
%---
%[output:4142069f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"     7.500000000000000e-03"}}
%---
%[output:851aa2d0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"     7.500000000000000e-03"}}
%---
%[output:3b26f23c]
%   data: {"dataType":"textualVariable","outputData":{"name":"idab_zvs_min","value":"     1.230000000000000e+01"}}
%---
%[output:69c3b6d1]
%   data: {"dataType":"textualVariable","outputData":{"name":"number_of_cells_1_sim","value":"   208"}}
%---
%[output:76434bd2]
%   data: {"dataType":"textualVariable","outputData":{"name":"number_of_cells_2_sim","value":"   155"}}
%---
