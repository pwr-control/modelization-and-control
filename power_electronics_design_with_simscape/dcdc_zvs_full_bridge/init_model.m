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
fPWM = 5e3;
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
dead_time_DAB = 3e-6;
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
m3 = n2;
m12 = m1/m2;
m13 = m1/m3;

Ls_dab = L_d_eff;

f0 = fPWM_DAB/5;
Cs_dab = 1/Ls_dab/(2*pi*f0)^2 %[output:3bd175ac]

Ls1_dab = Ls_dab;
Ls2_dab = Ls1_dab;

mu0 = 1.256637e-6;
mur = 5000;
lm_trafo = mu0*mur*n1^2*S_Fe/L_core_length * 1e-2;
rfe_trafo = V1^2/P_Fe;
rd1_trafo = P_Cu/I1^2/2;
ld1_trafo = Ls1_dab;
rd2_trafo = rd1_trafo/m12^2;
rd3_trafo = rd1_trafo/m12^2;
ld2_trafo = 0;
ld3_trafo = 0;
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

LFu_dc2 = 1e-3;
RLFu_dc2 = 0.5e-3;
CFu_dc2 = 3.6e-3;
RCFu_dc2_internal = 1e-3;
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
%[text] #### Semiconductor device: SiC MOSFET SKM1700MB20R4S2I4
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
mosfet.dab.Lstray_s = Lstray_s;                        % [H]
mosfet.dab.Lstray_d = Lstray_d;                        % [H]
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

%[text] #### Semiconductor device: Si IGBT FF650F1700IE4
infineon_FF650R17IE4; % two IGBT modules in parallel
par_modules = 2; % number of paralleled modules

dab.igbt.Vth = Vth;
dab.igbt.Rce_on = Rce_on/par_modules;
dab.igbt.Vce_sat = Vce_sat/par_modules;
dab.igbt.Vdon_diode = Vdon_diode/par_modules;
dab.igbt.Rdon_diode = Rdon_diode/par_modules;
dab.igbt.Eon = Eon/par_modules;
dab.igbt.Eoff = Eoff/par_modules;
dab.igbt.Erec = Erec/par_modules;
dab.igbt.Voff_sw_losses = Voff_sw_losses;
dab.igbt.Ion_sw_losses = Ion_sw_losses;
dab.igbt.JunctionTermalMass = JunctionTermalMass*10;
dab.igbt.Rth_switch_JC = Rth_switch_JC/par_modules;
dab.igbt.Rth_switch_CH = Rth_switch_CH/par_modules;
dab.igbt.Rth_switch_JH = Rth_switch_JH/par_modules;
dab.igbt.Rth_diode_JC = Rth_diode_JC/par_modules;
dab.igbt.Rth_diode_CH = Rth_diode_CH/par_modules;
dab.igbt.Rth_diode_JH = Rth_diode_JH/par_modules;
dab.igbt.Lstray_module = Lstray_module/par_modules;
dab.igbt.Irr = Irr;
dab.igbt.Cies = Cies*par_modules;
dab.igbt.Cres = Cres*par_modules;
dab.igbt.Rgate_internal = Rgate_internal;
dab.igbt.td_on = td_on;
dab.igbt.trise = trise;
dab.igbt.td_off = td_off;
dab.igbt.tfall = tfall;
dab.igbt.Csnubber = Csnubber*100;
dab.igbt.Rsnubber = Rsnubber/100;

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
% scenario = 1; nominal conditions
% scenario = 2; voltage input at minimum, voltage output at maximum
% scenario = 3; voltage input at maximum, voltage output at minimum
% maximum input current = 270A
% maximum output current = 270A

scenario = 2;

if scenario == 2
    number_of_cells_1_sim = floor(500/3.6);
    number_of_cells_2_sim = floor(860/3.6);
    i_in_dab_pu_ref = 0.65;
    i_in_dab_ref = i_in_dab_pu_ref * Idc_FS;
elseif scenario == 3
    number_of_cells_1_sim = floor(900/3.6);
    number_of_cells_2_sim = floor(360/3.6);
    i_in_dab_pu_ref = 0.54;
    i_in_dab_ref = i_in_dab_pu_ref * Idc_FS;
else
    number_of_cells_1_sim = floor(750/3.6);
    number_of_cells_2_sim = floor(560/3.6);
    i_in_dab_pu_ref = 0.65;
    i_in_dab_ref = i_in_dab_pu_ref * Idc_FS;
end
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
%   data: {"layout":"onright","rightPanelPercent":38.1}
%---
%[output:573defed]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"405.0926"}}
%---
%[output:9c485e2a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"937.5000"}}
%---
%[output:95f45fb2]
%   data: {"dataType":"text","outputData":{"text":"--- INITIAL ELECTRICAL PARAMETERS ---\nNominal Power (Sn): 347.82 kVA\nNominal Primary Voltage (Vn): 682.00 V\nNominal Primary Current (I1n): 510.00 V\nNominal Secondary Current (I2n): 194.00 V\nNominal Frequency: 5.00 kHz\n----------------------------------------------------\nCore Section Area (S_Fe): 153.6036 cm^2\nPrimary Turns (n1): 4\nSecondary Turns (n2): 8\nPrimary Copper Area (A_Cu1): 170.00 mm^2\nPrimary Copper Band Length: 17.00 cm\nSecondary Copper Band Length (L_b2): 12.93 cm\nCore Height (AM-NC-412 AMMET): 29.00 cm\nCore Width (AM-NC-412 AMMET): 6.00 cm\nCore Length (AM-NC-412 AMMET): 95.00 cm\nCore Dept (AM-NC-412 AMMET): 25.60 cm\nSpecific Core Loss (AM-NC-412 AMMET): 6.67 W\/kg\n----------------------------------------------------\n--- LOSS ESTIMATION ---\nCore Mass (M_Fe): 113.82 kg\nCore Loss (P_Fe): 758.80 W\nCopper Loss (P_Cu): 117.14 W\nTotal Losses per Phase (P_tot): 875.94 W\n----------------------------------------------------\nEstimated Efficiency (Eta, cos(phi)=0.95): 99.74 %\n----------------------------------------------------\n--- LEAKAGE INDUCTANCE AND REACTANCE ESTIMATION ---\nCalculated Leakage Inductance (Ld_calc): 0.000016 H (15.77 uH)\nEffective Leakage Inductance (Ld_eff): 0.000021 H (20.50 uH)\nLeakage Reactance (Xd): 0.644 Ohm\nEstimated Short Circuit Voltage (Vcc): 48.16 %\n----------------------------------------------------\n","truncated":false}}
%---
%[output:3bd175ac]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs_dab","value":"0.0012"}}
%---
%[output:1dee7c80]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"391.9184"}}
%---
%[output:3e86cd16]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"509.1169"}}
%---
%[output:3afa214b]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.0000","0.0002"],["-28.4245","0.9962"]]}}
%---
%[output:0f70846b]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"1"}}
%---
%[output:4d2d82be]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"2.0000e-04"}}
%---
%[output:7d759bac]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"-28.4245"}}
%---
%[output:8963de89]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"0.9962"}}
%---
%[output:67ef2d7d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.0734"],["3.8024"]]}}
%---
%[output:90bbebb9]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.2831"],["67.6682"]]}}
%---
%[output:89c004ba]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.0001"],["-9.8696","-0.0016"]]}}
%---
%[output:54943e0d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.0156"],["2.7166"]]}}
%---
%[output:3f18a767]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.0000","0.0002"],["-19.7392","0.9969"]]}}
%---
%[output:93c5b888]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.3110"],["54.3322"]]}}
%---
%[output:226c4982]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAhUAAAFBCAYAAADAE10CAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQt0XdV55z8STyxBQo1ASMhATEBOmKyMjR3GivNwUoa4nVZOQ5JaYk2scZyMM4HgFbBluQSmhIdl2TB1CUxcqnjktn60s6DBTduUpClNEFGJg5WkdYJ42MaSZQTGBRLjLhrN+o7ZYuv4PPY553\/vPffof9bSsqW793f2+X377P2\/336dNj4+Pi68SIAESIAESIAESCAjgdMoKjISZHYSIAESIAESIAGPAEUFKwIJkAAJkAAJkACEAEUFBCONkAAJkAAJkAAJUFSwDpAACZAACZAACUAIUFRAMNIICZAACZAACZAARQXrAAmQAAmQAAmQAIQARQUEI42QAAmQAAmQAAlQVLAOkEAAgaNHj8qKFSu8T3p7e6Wurq5knIaGhmT58uUyf\/586e7ultra2pLdyzZczmd0eSDD4Ytf\/KIsXbrUJUuu02zYsEG2bNnilXHlypWydu3aROXdtWuXrFu3TtavX59LHvp8P\/jBD0r+fiSCxsQVJ0BRUXEXsAB5JFDODjdMVGijvWjRImlpaSkJorBnLPV9wx4mTSelndrDDz+cqMNOkyepA\/QeV1999US2IoqKoonApD5m+mACFBWsGSSQMwLHjx+Xrq4u2b17t2zfvr1sokIjJOW4bxBu00G1trY6CwTzTT5Jh50mT5rqYURFkrL575P3SIWppwcPHmS0Ik0lKWgeioqCOjZPj2WHgbVcdjjX\/pY+d+5cufXWW72ia+diDwWYb9WDg4OnfG7b+NjHPiaf\/exnvTRNTU2ydetWaW5uDsRhd97+9P5v8fq5GQ654oor5K677pI5c+Z4jandGcfZ0WEUc989e\/Z45dPLDH\/cfPPN8pWvfMUTFObys9C\/G6a26DAdmZ3edEzGlt3J2c\/41a9+VXp6egLve+jQIa98IyMjE2WyfWhzVOb33HOPfP3rXxfzfMrfz9qwM8NKQR2o8at9X\/O8\/ucyvj7\/\/PMnhJGf34MPPugNJ5jLrh9+e3Fizl8fo2xF1cOoiIbNRMtsyu4XKv73y2ZrbFx\/\/fXyne98R\/T9Mb6zn1n\/Zu5h+zaOS1A9zFPbw7KUnwBFRfmZT5k7+jsS+8FNwxjUcfg7A7WjHboRFP7Pgzq9qA5ZPwsrm+kAzj777ElzKoyosMugnXeQCLCFhd8OSlQEfRP2N\/D+ziaMq\/49TFR0dnbKtddeewr7qE5cP6uvr5exsTFPNAV19HpPu\/Pzlz2sXpj7\/uhHPwoUCPfff\/\/EPAa7vtmdpl9U+G2Zz8OERVSd1TwHDhwIFS92mfyCwi\/8TIf+wQ9+UL73ve9NajeChEHQ++UXBZomqIz6d3OfONs2l7xHU6ZMY5ujB6WoyJEzilYU02ja39RMg6zPan9L12+jprGyG227AbS\/odmdkHbc5pu0sWHu7f9GbBgHfW43kFdeeWWoqLC\/ySW1EycqNDqjV9wwRFQkRaMnL7zwwilM7G\/Xymn27NmTntFl+MM\/NGPYG39qVMLvdy2Lzi8IiqAoyyVLlnjPa0c2XCavugxl+NP4fzdMjADS8sfd29S9oOcxf1Pxqc8cNvxhczT1ye\/Thx56yBMnQZGHMLv+aJWJztg2ou5tIhmm\/sdxQQzzFK3dm+rPQ1Ex1WtACZ8\/7FuMaZS1MZ03b17gygc7zf79+wO\/fWrRbRv67dis1IibaBn3DSus07YbWb1\/UjsoUWHfWwWCXnYnFtbY253q5z73OWdRERTZCbqvlsM\/vBMWCdC02jmacthsg+7nHwaKEhVhwz7+PFFRhyBB6n82M7TmFydGSIV1\/nH10\/avbSPMr\/6oh2FlREXYsJe9ssmuy+a9tIeeTFNhcwkacithk0LTVUCAoqIKnFStRSy3qLCXZMY12knFgPogaImpqx27w\/R3QGrbXlLqEqnQNPbkRv1dly\/6IzX+Ti2pqPBz9Ecz\/GIGJSpMnQ8TM7oiJkhU+IdR4iIV1SAqgiJjxq\/++hcWqbBthL0bFBXV2tLmq9wUFfnyR6FKk3b4wx+mN2PUYd\/6gsLVcaIiatjC\/vasDtFvc2GiwtWOhpX9Hb4ZFvKLCu24XSbA+Ttc+5u8fwhJO+G44Q+NosR1yn4baYc\/7Ioe9u3f\/zL4BYL\/W7t5ZjtiZZ7H1B1\/nqDhj7iXsNTDH0aAmghPmKgIivAYRv5IRdjEWv\/QS9TwRxAXDn\/E1Zap9zlFxdTzedmeuNQTNaM65ThREVW2oPkGYaIizo6Gis38CD94F1GheYJWfxhb\/hn89qZRSSZqmjC4nUfve9VVV3lRlKBLOQU9n+tETbVphJZfzIRNYrTz2Gn0nps3b5bbbrvtlEmlmscvKvRvYZM+zbPGidigoYG4SJHNMewZowSB3Ylfd911oXUryoaWIWgCp+tETZtLXKSubI0Nb5QbAhQVuXFFcQviuqTUXg4at6Q0aPJnkuEPpR0VWo+bCGnvsBllR+\/jX3540003yd69eycmJgZFKuwOJ2yyqW3bP9cjSHTYnaudV\/9vREXQfe+7775JO0Pqhlz2\/I00S0ptcWB3ckHLjcOWsvq52nM81KZy27Rpk6xevdrDYUeczCqesCWqcftLRC0p1Xu5foMPmwuh0aqgDjssOqOMgpbzBkU7wgSp\/t2\/g2fY3BRjwyWiVtyWjU8WRKDqRYWGPHXZm66xD9qPIGp\/A1aJyhOIm2lf+RKyBFEEgoZZ\/Ct8wvYJse2m2fyKnklHwBaBRjwFrQiJs87Nr+IITc3Pq1pUxI3Fms8XLlzo7Z1vfldFn3Qf\/qlZPUr\/1BQVpWdcyjtEDf9EDdsElSnNNt2lfLYi2w4a\/tDnjdswLkgIFuWsliL7u5zPVtWiQkOB2hDpFRap8MPUl6m\/v7+sBzeV06HVdi+Kimrz2Knl9UcDNUVSQaF5eJZEeeuCf1gyiaDQklIEltdf1XK3qhUV2pDdcsst0t7e7lVuiopqqXIsJwmQAAmQQFEJVK2o0IiDXrpJS9ScCttx5htVW1tb5FHCuu2z\/vAiARIgARIggXIQ0O3s9afar6oUFRom7evrkxtvvNHr\/F1EhRn7VYfZB1X5Haj21qxZIwMDA9XuW5afBEiABEigSggsWLBANm7cWPXCoipFhQ536LI23dAnbvWH1idXQaFpzZItde7MmTOrpDrms5gqzHTvALLM7h+yzM5QLZAjhiNZ4jjaLONOx8XetTTWqk5UBE0KM2iCHJJ0xYcRFUVwbmmqjLtVsnRnFZeSLOMIuX1Ojm6cXFKRpQsltzRFYll1osLvorhIhUY1dGOYqCEP22aRnOtWnUuXiixxbMkSw5IcMRztqC6\/gGVnWqR6WThRYSITuirEHO1sds8zro\/aobBIzs1e1bNZ0NNFde5LR0eHzJo1K5uxKZ6bLDEVgBwxHNUKWeJYFqnfqXpRgXPrSUtFci6aTVJ7r776qhw+fFjOO+88qampSZqd6S0CZImpDuSI4ahWyBLHskj9DkWFr14Uybm4Kp\/OEhuddNyCcpElhiU5YjhSVOA4Fu3LLEUFRQX27eC365LwZGeIwUqOGI4UFTiOFBVYlrmzxkgFziVswMkSRwBjiXUSw5GiAseRogLLMnfWKCpwLmEDTpY4AhhLrJMYjhQVOI4UFViWubNGUYFzCRtwssQRwFhincRwpKjAcVRLX+z9vnzz9mVShOW5nFPhqxsUFbiXhQ04WeIIYCyxTmI4UlTgOKqluuu\/KzP+cgVFBRZrPqxRVOD8wAacLHEEMJZYJzEcKSpwHCkqsCxzZ42iAucSNuBkiSOAscQ6ieFIUYHjSFGBZZk7axQVOJewASdLHAGMJdZJDEeKChzHg0dflbm3PcrhDxzSfFmiqMD5gw04WeIIYCyxTmI4UlTgOFJU4Fjm0hJFBc4tbMDJEkcAY4l1EsORogLH0YiKM\/9urez84z+UlpYWnPEKWOLqDx90igpcLWQDTpY4AhhLrJMYjhQVOI5GVLz1+z3y53\/4vygqcGjzYYmiAucHNuBkiSOAscQ6ieFIUYHj+P0nj8mSex+X03\/0dfl\/3ddQVODQ5sMSRQXOD2zAyRJHAGOJdRLDkaICx9GICu5TgWOaK0sUFTh3sAEnSxwBjCXWSQxHigocR4oKHMtcWqKowLmFDThZ4ghgLLFOYjhSVOA4UlTgWObSEkUFzi1swMkSRwBjiXUSw5GiAsdxx2Ojcs2OfdynAoc0X5YoKnD+YANOljgCGEuskxiOFBU4joxU4FiW3dLQ0JB0dnZKT0+PNDc3B96fogLnFjbgZIkjgLHEOonhSFGB47jhW8\/Ihm\/tZ6QCh7Q8lo4fPy5dXV2yZ88e2bp1K0VFGbCzAcdBJksMS3LEcKSowHGkqMCxLKsljUBs2LDBuycjFeVBzwYcx5ksMSzJEcORogLHUedT6LwK7qiJY1pyS0ePHpVbbrlF2tvbPWHhIipWrVolCxYs8MrW2Njo\/fBKRkAb8NHRUWloaJDa2tpkmZl6EgGyxFQIcsRwNKKC73d6nspOf27fM10eeeoYhz\/Soyx\/zl27dnk3nTdvnvOcCruUHR0dsmzZsvIXvMrveOLECRkbG5P6+nqZPn16lT9NZYtPlhj+5IjhqFbIMhvLbdu2SV9fn7z00Q3yq9PPoajIhrN8uXVypjruxhtvlEOHDjmLio0bN8rMmTMZqcjgKp3HcuTIES\/KU1NTk8ESs5Ilpg6QI4ajWiHLbCw1SvHYvv2y8qFxzxB31MzGs2y5dbhj0aJF3n7qXP1RNuzejTh+jeNNlhiW5IjhyPcbw9EsJ1VrnFOBYVpSKzqXYsWKFTI4OHjKfbZv3x54cAuXlOJcwgacLHEEMJZYJzEcKSowHM3KD7XGU0oxTMtqhZGKsuJmpAKIm50hBiY5YjhSVGA4fvxre+XhJ170jDFSgWFaVisUFWXFTVEBxM3OEAOTHDEcKSowHOfe9qgcPPqqvOmXz3uiIiyCjrlbeaycNj4+fnKWCC+PAIc\/cBWBDThZ4ghgLLFOYjhSVGTnqGJCRYVeV579nDzWu46iIjvW\/FmgqMD5hA04WeIIYCyxTmI4UlRk52jPp7hx\/gm556YvUFRkx5o\/CxQVOJ+wASdLHAGMJdZJDEeKimwc7SjFhXU18rX\/cppcffXVFBXZsOYzN0UFzi9swMkSRwBjiXUSw5GiIhtHc9y5Wmm\/vFE+fdG\/UlRkQ5rf3BQVON+wASdLHAGMJdZJDEeKivQcNUqx5N7HvQmaGqV48AuXycgTeykq0iPNd06KCpx\/2ICTJY4AxhLrJIYjRUV6jnaUYs1HZ8m637ioUAsEuPrDVzcoKtK\/LP6cbMDJEkcAY4l1EsORoiIdR3sHTY1S7P3y+zxDRep3KCooKtK9HQ652IA7QHJMQpaOoGKSkSOGI0VFOo5mXwrNfU\/7pd58CoqKdCyrJleRFGOlobMBx3mALDEsyRHDkaIiGUd7HoXm\/B8fPF+6P948YaRI\/Q4jFYxUJHs7EqRmA54AFr9h42BFWGKdxGEmSzeWfkFhD3sYCxQVbiyrMlWRnFtpB7DRwXmALDEsyRHDkZEKN44ugoLDH24sqzYVRQXOdWzAyRJHAGOJdRLDkaIinqO9ykNTB0UoGKmI51j1KSgqcC5kA06WOAIYS6yTGI4UFeEcNTpxzY598shTxyYSbV76Lvn0gvNCMxWp3+GcCp+bi+RcXPORzhIb8HTcgnKRJYYlOWI4UlQEc7SXjJroxFfbLpUPXDIjEnyR+h2KCooKXCvjs8QGHIeWLDEsyRHDkaJiMkcVE9fu3Oftkmkus1um\/ht3UVTEEariz4vk3Eq7gQ04zgNkiWFJjhiOFBUnOYaJCZfohO2JIvU7jFQwUoFrZRipIMuSEcAYpqjAcJzqouKbPxmTT2\/96SSYGpH4w6Xvkg81n5UYMkVFYmTVk6FIzq00dTbgOA+QJYYlOWI4TjVRYYY1\/BMwlYOKiY2fmC3vbDjD+3+aq0j9TlVGKo4fPy5dXV2ye\/duz38rV66UtWvXhvpy165dsm7dOu\/z1tZW6e7ultra2sD0RXJumsqNzMMGHEeTLDEsyRHDcSqICiMk7n\/8iHzlm0+fAk4FRNJhjjD6Rep3qlJUbNiwwfONComjR4\/KihUrpK2tTZYuXXqKz9RZmr63t9cTEipGmpqaQkVIkZyLaz7SWWIDno5bUC6yxLAkRwzHIosKFRO9jwzL3d89GAjr\/RfPkLWLL4pd0ZGEdJH6naoUFX5n2SLD\/5lGKfr7+yeiE\/7f\/emL5NwklboUadmA46iSJYYlOWI4FklUmIjEH3zngPzfR0cCASGjEkE3KFK\/U\/WiwkQqNGrR0tLiFKlYuHBhYFRDMxfJubjmI50lNuDpuDFSgePmt8Q6iWNb7Sx1x8vt\/3R40iZVNh0VEssXzpSPzz039VwJV9pF6nfKJipM5z84OOjK2Us3Z84ceeCBBwLzaIRiy5YtsfMkhoaGZPny5TIyMiLbt28PFB\/mBsa5q1atkgULFnh\/bmxs9H54JSOgjc7o6Kg0NDSEzmFJZnHqpiZLjO\/JEcPRRCqq5f0eeek1ee2112Tzw4dFxUTY1XTmNLnzqkvkonNqSy4klJ3+6DU8PCxr1qyJ7Z9w3iudpbKLirCIQtAjmvkQYaLC5FFxoYIhaAKmDnfs3LnTm1NRV1fnza\/QK2xipxEVdnk6Ojpk2bJlpfNCQS2fOHFCxsbGpL6+XqZPn17QpyzPY5ElhjM5YjiqlbyyVAGh4mDP8Kvyt0\/8Qu7\/6cuRIuK8t02TlQtmiP6r+cp1bdu2Tfr6+ibdLu5Lb7nKluU+hRAVGono7OyUnp4eaW5+44x6s0rEHu4IS2sgGlGxceNGmTlzpvdnRirSVTHlf+TIEY9fTU26pVbp7ly8XGSJ8Sk5YjiqlTyx1HkR4yJy7Y59npiIunRY43PvP09+893nlFVE+MtkRyoGBgZk8+bNjFQkqZ5aAfVHowXoy17hYdvPIiqKoBjRnJPaq\/Yx16TPW8r0ZImhS44Yjmqlkix1J8sHB5+TfaO\/CJ0TYZ5URcTtH2uWX6udBl2xgSNZrLl8ZY9U6JyKuH0l4pxlD2EY4RC2TDRo+CNsqETvW6QJM3EcS\/15JRudUj9bue2TJYY4OWI4lkNUaPRBBYEKiL\/+6Zj8ZPgVJwGhZeu5arac\/pY351ZE+L1QpH6nbKLCQDSTK\/V3FQJbt26dNGThUuX9m1\/ZG1qZz9rb2ycmZNr35OZXLoQxadiAYziWowHHlTTfllgncf4pBUsVEI8+fUz+cejFWAGhT6KiQ7fF\/t35jVUjIII8QFEBqJf+1SBZoxeAInkmiuRcFJO0dkrR6KQtS7XnI0uMB8kRwzGr0DV7Q\/T83X458MJxZwFxwVk10vbeRvlg81klX52BIxVvqUj9TtkjFUF47SWf+nkl5zMUybnxVbm0KdiA4\/iSJYYlOWI4uooKFQ\/60\/\/0MfmeY\/TBRCBUQHxqfoN8ePbJeXhpz9XAPXHpLBWp38mFqLBdFTbpsnTunGy5SM4tF7Ow+7ABx3mALDEsyRHD0S8qnvvlSbt\/ufc5eWjfC06RB1MSFQtm6+uii4cw+kXqd3IhKhipwL3oebLEBhznDbLEsCTHbBzNsMWfDozIo0\/\/a2LxoNGHaz9yoZzxljd7kYciRx+SkKaoSEIrJG3UZEuA+dQmiuTc1BBAGdmAg0BWePke7ikqb4l1Mt4HZsjigroa6fnWM97wxSNPHYvP+HoKFQoqHi4593T50hVvn8hHARGOsEj9TtkjFfZKDEVcyfkTQS4uknOdW4ESJWQDjgNLlhiW5PgGRxN1+OPvH5LHn305kXBQK7r7pO5C+d53nCO\/9Z56Rh4yVNEi9TtlExX2ao+4ZZ0ZfJM5a5GcmxlGRgNswDMCtLKTJYblVONoog6Dh16Wv\/3n5+XZF09OnExymcjDFe+qk6sua\/Cy6t+mGsskzJKmLVK\/UzZR8eSTT8p1110nN998c+SBXrYzXM\/+SOrAqPRFci6SSxpbbHTSUAvOQ5YYlkXkaISD7u2gezxkEQ7vbDhDrvv1Cz3hETfnoYgsMbUsuZUi9TtlExVxR5SHDUXocEncgWLJXRieo0jORXJJY4uNThpqFBU4aqdaqsY6aSIL+q\/rzpJBDM2cBl1poXs9nHbaaZk2jKpGlqWsW1lsF6nfKbuoQB59nsWJYXmL5NxS8Elik41OElrRackSwzKvHI1w+Jt\/fl7+6sdj3sMmmRxp6NjC4VPzG2Xam7IJhyjqeWWJqSnltVKkfqdsoqK8Lkp\/tyI5Nz0FTE42OhiOaoUsMSwrxdEMUdSd8R\/k3n84KAcSrqiwn95eXfEJa45DuVdXVIolpibky0qR+h2KCl\/dKpJzK\/3asNHBeYAsMSxLyVHPrXjm+ePyg2eOeXMS0sxt0Kc04kCXZX5m4Uypf9tbvIePm+OAIeRupZQs3UtRjJRF6ncoKigqSvZWstHBoSVLDMu0HM3wxM9HfyF\/9dMxeXrseGrRYASCiobZDWfIx+eemzvB4EI7LUsX21MtDUVFgT1eJOdW2k1sdHAeIEsMyzCOGmXQSICKhr8cfC7xhk\/+0pkhinc2niGt76mXN7\/ptKoUDlHUWScxdVKtFKnfYaSCkQrcm+GzxEYHh5Ys07O0V0\/8\/c9fkO\/97DkZOy6J92uwS2BEw9vPrpHfmXuu1Ew7ue20PXyRvsTVkZN1Eucnigocy9xZKpJzKw2XjQ7OA2QZztJMgnztV+PyF3tGM81nMHeZmAxZf7p8Yp4OT9TynAp+acC90AX+MlvRSMWuXbtk3bp1Hl7drvvAgQPS398v3d3dUltbWzIHRhmmqMBhZ0dIllkJmGGJf\/v3X8n\/efhZeeLILzPNZdDy6PbS06ZN886n+Oh\/PFsuu+DMiQhDuVdQZOVTyfx8v3H0i9TvVExU6KZWIyMj0tnZKddee62sXbtW5syZI11dXdLU1OT9XomrSM6tBD\/7nmx0cB4oIkszLHHg6HHZ\/eMx2Xf4F5kFgxl+UMEw6+xab3fI6dPeNOGIc08XOXz4sJx33nlSU3NyuIJXOgJFrJPpSGTPVaR+pyKiwt5dc\/bs2bJixQpPRLS0tHgTVlRw9Pb2Sl1dXXZvJbRQJOcmfHR4cjY6OKTVyFKjDOPj4\/Lgj8fkZ6M4waBUdVfIeReeKbqttBESLlGGauSIq0VYS2SJ41mkfqcqRYX\/2PSVK1dGRjaMw7QKaDQkSrAUybm4Kp\/OEhuddNyCcuWJpZnDcO6Zb5G7\/\/6g7H8h2\/JK87z2\/gzvajxDvvgRtzMoklDOE8ck5c5jWrLEeaVI\/U5FRIW6QudT6PwJe\/jDRC3a2tpk6dKloR7TSIZeGt0wUY+wPENDQ7J8+XLZtGmTFwkx9w2bt1Ek5+KqfDpLbHTScaukqDBzGHTHxz\/\/4WhJBINGGK689GzvMT9wyYyJw6twtMItsU7iKJMljmWR+p2KiQp1hx1BMO5Zv359pKAIcqMtMvyfq4jYv3+\/8xyNIjkXV+XTWWKjk45bKUSFfdz1nw0clv6nj3m3SXO+hL98doSh5aJfk0Wz6yatlHAZlsCRirbEOokjTZY4lkXqdyoqKhAuiTr91AyTLFy40FmoFMm5CL5ZbLDRyUJvct4olia68C+HX5Fv7zsqPz+Cmb+gJbAPqJp7wdvk0sa3egXL25bRrqRZJ11Jxacjy3hGrimK1O9UtajQCMWWLVuktbU1cBmqERWLFy+W++67T\/SEVNc5FatWrZIFCxZ4daKxsdH74ZWMgDY6o6Oj0tDQULElwslKnK\/UIy+95g0N6BLI7Xuel0eeGPOWQg4ceCVzQdWmXue9bZq3SuLS886Q335PvRw8erzQezKwTmauOhMGyDIbS20b9Uev4eFhWbNmjbe1gg7TV\/NVEVFhogtxx6DHTcA04M3yVP88CSMqDh48ODE5MyytsRU0JNPR0SHLli2rZj9XpOwnTpyQsbExqa+vl+nTp1ekDHm7qQoF7dD3DL\/qdeiPHDguPxs7Ic8ee00Ov\/ya6OdZL1swzJ9ZI+89\/+TSSb2f+SzrPao1P+skznNkmY3ltm3bpK+vb5IRiooMTHWuw86dOyetxLAnXS5ZssR5zwqdjKkTPnt6eqS5uXmiVEHDH2Fp\/aJi48aNMnPmTO\/PjFSkc7TyP3LkiMdvquwJ8P0nX\/S+6Y++dEL+dOCwd2olUiyYTZveUV8rH26eIe9uqPGiF3pNdcHgUkunYp104ZImDVmmofZGHjtSMTAwIJs3b2akIi3SqHkQ9j4VTzzxhLdnxQMPPBB5q6i9LTT\/rFmzJuZUqKi444475M477wzcB6NIY1tp\/YPKV6QxV7OM8qzTp8nX\/vEQbFWEYW3PXWg8c7p0vK9pYlWEflYklqj6lcYOOaahFpyHLHEsi9TvVHT4w2x4ZbvGRVTYqz1MNCJsF06\/4IhaKaLlKJJzcVU+naW8Nzr2QVOvnHhN\/vqnz3uRBb0QqyLUjr0y4r1vP1OueNfJpZR66XJK1yvvLF2fo9LpyBHnAbLEsSxSv1MRUaGuiBv+0H0qzJ4SGhayL\/\/mV\/ZETfNZe3v7xIQXe55E2KROY79IzsVV+XSWKtXo2GLh9Le8Sbb94LA8NfbLkomF2Q1nyHua3iq\/\/q66SdGFdNT4rRDJzW+rUnWylM9UKdtkiSNfpH6nYqLCjgrYrjETVVRQ3H333bJ169ZJ8yRwbgy2VCTnlppVnH10o2PvtfDTkVfkoX0vyJPPZT9gyn4OO7Jwcf3p8hvvPlveOv3knIVKLqNEs4zzXVE\/J0ecZ8kSx7JI\/U5FRQXOJThLRXIujko6Sy6NjgoF7azNXgu6x8Lf\/+yoqGh49sVXvW\/+iMsWC7POqZXffs85csZbpuV2kyZ+w0Z4\/VQbLnWyNHcunlWyxPm0SP0ORYWvXhTJubgqn86SNjp7fv6st6RUV39s7R\/gHu59AAAgAElEQVSWHx54yTNWijkLl9SfLl\/48AXeqZRGrJgIQ7onyE8uNuAYX5AjhqNaIUscyyL1OxUTFeZMDj3+3H\/FbVCFc+Wplork3FJyUtv23IVv\/mQMGl2wIwv6\/y9d8XYZfenfvEcq93kRpeboYp8NuAul+DTkGM\/INQVZupKKT1ekfqciosLeP8LsR6ETK\/3HoMe7Ap+iSM7NSsceevjf3zkAmb9gi4XZDad7qyHOrDk5DJGnMyKyskPnZwOOIUqOGI6MVOA4qqUi9TsVERX+fSrsvSQU7o4dOwK33ca6MdhakZzrwsue0\/Ctf3le9j77cuqhCVswNJ97unz+\/Q3y46cPy396x3ne8AdFg4tHgtOwM0zPzs5JjhiOFBU4jhQVAJZ+UWGfJBq1kRXg1rEmiiwqTOThJ8OvyNf+8dnE4kFFwQVn1cjb62rkqnkN8pY3vylyKIINeGx1c05Als6oIhOSI4YjRQWOI0UFiKW9CZUtJB566CHp7+9npALEWVdV7HjssOx47OTBNXGXEQ4t75ghi5rP8pIn2aSJ3wrjCKf7nJ1hOm7+XOSI4UhRgeNIUQFi6T+Xw5w4qjtjlntvCvuRihCpePzZl+XmB5+MjEQY8fAb7z5H5pz\/ttTCIao6sAEHvSycaQ8DyToJQ8nVHziUnFMBZJk7U9UqKnRfh\/\/29Z+E7uugIuLq\/3yeLHzHjLJNimQDjqveZIlhSY4YjoxU4DgyUgFg6XqgWF1dHeBuyUxUk6jQORLX7NgXGJFQEXHlpWfLFz9yYcUmSLIBT1b3GPXB8QqzxDqJY0yWOJbV1O\/EPXUuVn\/4hx90KKS3tzfwFNG4B8r6ed6da07LvHbnvlOiEiok7mm\/1JtMmYeVFmx0stbGN\/KTJYYlOWI4MlKB48hIRQaWuspj3bp1sRZWrlwpeoJpJa48iwqddOkXEyoePjy7Tj45r6Ek8yKy+IANeBZ6k\/OSJYYlOWI4UlTgOFJUAFhGDX8AzGcykVdR0XrP45OGOVRM3LrkEm+SZR6iEkHQ2YBnqoqTMpMlhiU5YjhSVOA4UlRgWebOWt5EhS4F1XkT5lIB8T8XXSC\/+e5zcismTFnZgOOqN1liWJIjhiNFBY4jRQWWZe6s5UVU6NyJv9hzRG7\/m6cnCYoHv3BZ7sUERQW+WrMzxDAlRwxHigocR4qKlCzNkMfg4GCshal+oJgKiiX3Pj4xEVOjE19tuzR3cybiHMkGPI6Q++dk6c4qKiU5YjhSVOA4UlRgWebOWqUjFUGCopqiE7ZD2YDjqjdZYliSI4YjRQWOI0UFlmXurFVSVKigmHvbo5OGO\/Z++X25Y+RaIDbgrqTi05FlPCOXFOToQsktDVm6cXJJVcl+x6V8SdJUZJ8KU8CgJabr16+XpUuXRj6D2eJ79+7dXjrXJahDQ0PS2dkpPT090tzcHHiPSjnXLyhu+q13yJeueHsSX+YuLRsdnEvIEsOSHDEcGanAcWSkAsRSBcXOnTsnbXJl5l20tbVFCgv7MDLXPEaI7NmzJ\/JskUqICv+Qx12ffKf894VNINKVM8MGHMeeLDEsyRHDkaICx5GiAsASvU23LTLCimdOQtXP8xSp8AuKIkQojA\/YgANeltdNkCWGJTliOFJU4DhSVABYIkWFy0ZamuaWW26R9vZ2UQHiIipWrVolCxYs8J62sbHR+ynFdcMDz0wcS9505jT5YdflpbhNRWxqAz46OioNDQ1SW1tbkTIU5aZkifEkOWI4GlHB9zs9T2WnP3oNDw\/LmjVrZPv27dLS0pLeaA5yVmxORZbhD8PNHJfe2toq3d3doR2X3kuvefPmOc+psH3T0dEhy5Ytg7tr975X5Pe\/\/bxnVwXF7o7z4feopMETJ07I2NiY1NfXy\/Tp0ytZlKq\/N1liXEiOGI5qhSyzsdy2bZv09fVNMkJRkY2ppJ2o6b+tiouRkZFAYaGTM9VxN954oxw6dMhZVGzcuFFmzpzp3aoUkQod9mjZ9LhnX\/eh+INPNkvLrLdmJJqv7DqP5ciRIx6\/mpqafBWuykpDlhiHkSOGo1ohy2ws7UjFwMCAbN68mZGKbEhxuaNWdajgWLRokRdSysvqD\/+R5XqyaPvlpRlewVFObonj18mZheUgSwxLcsRwVCtkiWNZiQUCuNJPtlSR4Q\/XFRuuD20mYfqPS4\/axTMszFQO59rnebz\/4hmy+5rLXB+1qtKx0cG5iywxLMkRw5GiAsdRLZWj38GWONxaRUSFFsc\/9JFkLMle7WGWijY1NcUel56HSIW9H4UOe1TrbpkuFZQNuAsltzRk6cYpLhU5xhFy\/5ws3VnFpaSoiCOU8HMz4VKzqTjYunVr6OZUmsa\/+ZU9UdN8pis9\/LNo8yAqrt35M9n+T4c9QkUd9jDuZ6OT8EWISE6WGJbkiOHISAWOIyMVWJanWFOBoarNP5RR4ttOmC+lYvRHKap5C24Xf7ABd6HkloYs3TjFpSLHOELun5OlO6u4lKXsd+Lujf68YsMf9oPYkYpKnlBaasXYes\/j8shTx7xHV0Ghwx9Fvtjo4LxLlhiW5IjhyEgFjmOp+x1sSeOtVUxUJB3yiH8UTIpSKUY7SlHkyZm2F9iAY+okG3ByxBHAWeL7jWNZqn4HV0J3SxURFS67YLo\/AjZlqZxrRyl0cuYHLpmBLXgOrbHRwTmFLDEsyRHDkUIXx5GRCizL3Fkrhaj4\/pPHZMm9Jze6mipRCjY62KrNzhDDkxwxHPl+4zhSVGBZ5s5aKUTFVV\/bK\/\/wxIves06FuRTGqWzAcdWbLDEsyRHDkaICx5GiAssyd9bQomKqrfiwHcoGHFe9yRLDkhwxHCkqcBwpKrAsc2cNLSo2fOsZ2fCt\/d5zTpW5FIxU4Ks1O0MMU3LEcKSowHGkqACwRB59DijOJBNIUTGVoxRsdLA1k50hhic5Yjjy\/cZxpKgAsJwqosKeoKkHhukOmlPpYgOO8zZZYliSI4YjRQWOI0VFBpZBR50HmVu5cmXsOR4ZihGZFRmpmIrLSG24bMBxtZQsMSzJEcORogLHkaICwHIq7FMxFTe78lcNNuCAl+V1E2SJYUmOGI4UFTiOFBVYlrmzhopU2MebT7UJmsapbMBx1ZssMSzJEcORogLHkaICyzJ31lCiwgx96PkeRT84LMyJbMBx1ZssMSzJEcORogLHkaIiJUt7yGP27NmyYsUKGRwcDLRWyUPFUKKi7vrves\/2wUtmyDe+cFlKatWdjQ04zn9kiWFJjhiOFBU4jhQVWJa5s4YQFfbeFFNpB02\/M9mA46o3WWJYkiOGI0UFjiNFBZZl7qwhRAWHPk66lQ04rnqTJYYlOWI48v3GcaSoALA0QyFFHP7gqo83KggbcMDL8roJssSwJEcMR4oKHEeKCizLSdZUbNxwww3ye7\/3e9Lc3FzCO4WbzhqpsDe8mqqrPgxdNuC4KkyWGJbkiOFIUYHjSFGBZXmKNe3Ud+zYId3d3VJbWxt4t+PHj0tXV5fs3r3b+zxqsyx\/VKS1tTXSdlZRwaEPRipK8YqwM8RQJUcMR4oKHEeKCizLQFGxYcMG6e3tlbq6usC76ed6rV27VoxoaGtrk6VLl05Kb8THwoULvc\/M701NTaE7dmYRFRz6mOwuNuC4l4UsMSzJEcORogLHkaICy\/IUayoYRkZGIqMJ\/ky2yIgrnm4V3t\/fH2ofJSrWLp4laxdfFFecQn\/OBhznXrLEsCRHDEeKChxHigoAy6iJmhpF2Lp1q\/OciqRbfruKilWrVsmCBQu8p21sbPR+4q5P9e6TR5465iX7werLRDe+msqXNuCjo6PS0NAQOpQ1lfkkeXayTEIrPC05YjgaUcH3Oz1PZac\/eg0PD8uaNWtk+\/bt0tLSkt5oDnKeNj4+Pl6JcoQNTZihCpcyaYRiy5YtEjdPwtiKGioxaUykwr5\/R0eHLFu2LLZIrX2HZOSl16TpzGmyu+P82PRFT3DixAkZGxuT+vp6mT59etEft6TPR5YYvOSI4ahWyDIby23btklfX98kIxQVGZgGDXO4dPpBt3QZMjEiRvNHTQI1omLjxo0yc+ZM73YukQqdT9Gy6XEvvR5zfufHp\/bQh3JQ5keOHPH41dRM7ahNhlfFy0qWWQmezE+OGI5kmZ2jHakYGBiQzZs3M1KRFmvUkIXL6g\/\/fYeGhqSzs1N6enoCh01cBYXaTTunwt5Fc6ovJTX+4fh12jfk1HxkiWFJjhiOaoUscSzT9ju4EuAsVWT4I05UxK3+8D++OiQsj8uKD9teWueapaRq6+hdH8F5qIotsdHBOY8sMSzJEcORogLHMcuXWWwpMNYqIir88ynsR4mbSKlp7dUecaLBZWgEISrMAWJT+VRSf5VkA455SdmAkyOOAM4S328cy7RfZnElwFmqiKgwymz16tWTVnroMMby5ctl06ZNkTNg\/Ztf2RM1zWft7e0Sdhpq1CmoaZxr70\/xx59+t1x12bk4D1WxJTY6OOeRJYYlOWI4UujiODJSAWQZtNKi0rNf04gKbs0dXCnYgONeFrLEsCRHDEeKChxHigosy9xZSyMqOJ+CoqLUFZmdIYYwOWI4UlTgOFJUAFjaQxR52+gjjaiYe9ujokMgnE8xuXKwAQe8LK+bIEsMS3LEcKSowHGkqACwTLoLJuCWziaSigqe9xGOlg24c7WLTUiWsYicEpCjEyanRGTphMkpUdJ+x8lohRJVbKKmyyqPSjBJ6lzOp6CoKEc9ZQOOoUyOGI6MVOA4MlIBYBl19oeaj1qdAbh9pImkooKbXlFUlLpOsgHHEaaoIEscAZylpP0O7s54SxWLVOAfBWMxqXPNJE3OpziVPxtwTJ2kqCBHHAGcJb7fOJZJ+x3cnfGWKCp8TJM4l\/MpoiskGx3cC0uWGJbkiOFIoYvjyOGPlCztyZlhm1IZ09Uy\/GHPp1i7eJasXcxDxOzqwQY85csSkI0sMSzJEcORogLHkaICyzJ31pJEKjifgpGKclVgdoYY0uSI4UhRgeNIUQFi6T\/\/I+o8ENAtncwkERXX7NgnOx4b9ezyELFT8bIBd6pyTonI0glTbCJyjEXknIAsnVHFJkzS78Qaq3CCis2pCDroywyRtLW1ydKlSyuCJolzOUmTkYpyVVI24BjS5IjhyEgFjiMjFQCWcUef79ixQ7q7u6W2thZwt2QmXEUFJ2nGc2UDHs\/INQVZupKi0MWQirfCOhnPyDWFa7\/jaq+S6SoSqYgTFRrF6O3tlbq6urKzcXWuLSo4STPYTWx0cNWXLDEsyRHDkZEKHEdGKgAso+ZPVHqnTVdRwZ004ysCG\/B4Rq4pyNKVFCMVGFLxVlgn4xm5pnDtd1ztVTJdRSIVRpmtXr1atm7dKs3NzR6DoaEhWb58uWzatEkqddCYq3O58iO+2rLRiWfkmoIsXUlRVGBIxVthnYxn5JrCtd9xtVfJdBUTFXbIxwawffv2igmKJGEoTtKMr7ZsdOIZuaYgS1dSFBUYUvFWWCfjGbmmoKhwJVWF6VydW3f9d72ne\/\/FM2T3NZdV4ZOWvshsdHCMyRLDkhwxHNUKWeJYuvY7uDuWzlJFIxWle6w3LJv5G7t37\/b+uHLlSlm7dm3orV2da0RF++WNck\/7peV4lKq7BxsdnMvIEsOSHDEcKSpwHNWSa7+DvWtprBVeVOhKEr1USLjsg+HiXHuSpgoKFRa8TiXABhxXK8gSw5IcMRwpKnAcKSqwLMtuzRYZQTd3ERWcpOnmNjbgbpxcUpGlC6X4NOQYz8g1BVm6kopP59LvxFvJR4rCRypszFH7Y5h0Ls5dtetn8icDh70se7\/8PtFjz3kxUlHKOsAGHEOXHDEcGanAcWSkAsuybNY0QrFlyxZpbW2N3K3TiIpVq1bJggULvPI1NjZ6P+b6VO8+eeSpY9J05jT5YdflZXuGaruRNuCjo6PS0NBQkd1Rq41XVHnJEuNNcsRwNKKC73d6nspOf\/QaHh6WNWvWSKVXP6Z\/mjdyVixSYfakGBkZOeU5Snn0edCZI3YBjKiw\/9bR0SHLli2b+NP8u\/d7\/58\/s0b+6CrOpwiriCdOnJCxsTGpr6+X6dOnI+rrlLVBlhjXkyOGo1ohy2wst23bJn19fZOMUFSkZGpWZDQ1NUWuxEhpPjKbipnOzk7p6emZ2HQrSFRs3LhRZs6c6X1kRyp0e+6WTY97f7\/hivPlhisuKEUxC2FT\/XzkyBGPX00Nh4iyOJUss9B7Iy85YjiqFbLMxtKOVAwMDMjmzZsZqUiL1GVuQ1rbcfk0EhF1tkjcnAp75QfP\/IimzfHruNro\/jlZurOKSkmOGI5qhSxxLOP6HdydSm+pIsMfJlLR3t5e8t0z7dUeLhGSOOfueGxUrtmxz\/PMg1+4TD5wyYzSe6lK78BGB+c4ssSwJEcMR4oKHEe1FNfvYO9WWmsVERUGYjlOI\/VvfuU6UTNsbEsFhQoLvbjyg5GK0r6eb1hnZ4ghTY4YjhQVOI4UFQCWZvhjcHAw0FopJ2rGFT9OMfLMjziC7AjdCbmnZGfozioqJTliOFJU4DhSVGBZ5s5anKiYe9ujopM1eeZHvOvYgMczck1Blq6kGD3DkIq3wjoZz8g1RVy\/42onD+kqNvyRh4cPKkOUc1VMqKjQi6Ii3oNsdOIZuaYgS1dSFBUYUvFWWCfjGbmmoKhwJRWTbteuXbJu3Tovlc5hOHDggPT390duTgW6dagZV1HBlR\/xnmCjE8\/INQVZupKiqMCQirfCOhnPyDUFRYUrqYh0ZhMq3TPi2muv9far0LkUXV1dUon9K0xRo5xrLyflyo\/4SsBGJ56RawqydCVFUYEhFW+FdTKekWsKigpXUiHp7H0qZs+eLStWrPBERUtLi7e0phyrQsIeIcq5PEgsmePZ6CTjFZWaLDEsyRHDUa2QJY4lRUVGltUqKszKD338o3d9JCOF4mdno4PzMVliWJIjhiNFBY6jWqKoAPDU+RQ6f8Ie\/jBRi7a2Nlm6dCngLslNRDmXy0mT8WQDnowXIxU4XmGWWCdxjMkSx5KiAsQy6PCu9evXV0xQxCnGuuu\/6z05V364VQA2Om6cXFKRpQul+DTkGM\/INQVZupKKT0dREc+oalOEOddeTtp+eaPc035p1T5juQrORgdHmiwxLMkRw1GtkCWOJUUFjmXuLIU5lweJJXcVG53kzBi2xzELssQ6ieNLljiWFBUglvY+FcZkpc+TdxEVXE7qVgHY6LhxcklFli6U4tOQYzwj1xRk6UoqPh1FRTyj2BQqKHbu3Cm9vb1SV1fnpTerQvI4UZN7VMS69JQEbHSSM2OkAseMkQqyLC0BnHWKiows7SWlujeFfeV1nwqeTprc6RQVyZlRVOCYUVSQZWkJ4KxTVGRkWY2igstJkzudoiI5M4oKHDOKCrIsLQGcdYoKAEuFuHr1atm6das0NzfnfvjDnE56YV2N7P3y+wAEim+CogLnY7LEsCRHDEe1QpY4lhQVGVmaSMXg4GCsJT0P5IEHHohNh0oQ5lyzR8Wi2WfJA5+fi7pdoe2w0cG5lywxLMkRw5GiAsdRLVFUYHnmylqQc+09Kng6qbu72IC7s4pLSZZxhNw+J0c3Ti6pyNKFklsaigo3TlWZKsi53KMinSvZ6KTjFpSLLDEsyRHDkZEKHEdGKoAsg\/apcNmm2z980traKt3d3VJbWxtYOvs+cWmDRMWOx0ZFV3\/opfMpdF4Fr3gCbMDjGbmmIEtXUtHpyBHDkaICx5GiAsQy7T4Vx48fl66uLlm4cKF3Roj5vampyTs+3X\/ZS1RVdGjesLRhzuWR5+mczgY8HTdGKnDc\/JZYJ3FsyRLHksMfGVmil5SaE0+DohX+z6LShokKe48KHnnu7nw2Ou6s4lKSZRwht8\/J0Y2TSyqydKHkloaiwo1TaKpyioqgSIWJcgQVMMi53KMincPZ6KTjxkgFjhsjFWRZOgI4yxQVAJZphz\/8t3bZ2ntoaEiWL18uIyMjEne2iHHuqlWrZMGCBd7tPv\/tcdEVIE1nTpMfdl0OePqpYUJFxejoqDQ0NITOd5kaJLI\/JVlmZ6gWyBHDkSyzc9S2UX\/0Gh4eljVr1sT2T9nvWnoLp42Pj4+X\/jbBd0g7UdNYM\/Mp9PewiZp+8bJhwwYve9D8C\/27ERV2iY\/9Tq\/36\/yZNfJHVzVWClfV3ffEiRMyNjYm9fX1Mn369Korf54KTJYYb5AjhqNaIctsLLdt2yZ9fX2TjMR96c12x\/LkrqioyPKILoLCP6lT76dRi87OTunp6ZnYydMuhxEVGzdulJkzZ3of\/dc\/P+79+6nLzpHNnzq5+yeveALK\/8iRI9LY2Cg1NVwxE08sPAVZZqH3Rl5yxHBUK2SZjaUdqRgYGJDNmzczUpENafrccSs+jOUsosIoRnuPinvaL5X2yxmpcPUc51S4kopPR5bxjFxSkKMLJbc0ZOnGySUV51S4UCphGh3C0PkRUXtTmNsHDX9E5fU7l6IivSPZ6KRn589JlhiW5IjhqFbIEseSogLHMrGlsHND9IyQ3t5eb0Kg7kXR3t4u5lh1FSFbtmzx7pV08yvuUZHYRRMZ2OikZ0dRgWNnW2KdxHElSxxLigocy9xZ8jv3gb3PyYpt\/+yVk7tpJnMXG51kvKJSkyWGJTliODJSgeOoligqsDxzZc3vXLNHhRaSG18lcxUb8GS8KCpwvMIssU7iGJMljiVFBY5l7iyFiQo970MjFbzcCbDRcWcVl5Is4wi5fU6ObpxcUpGlCyW3NBQVbpyqMpXfuXNve9Tb+Or9F8+Q3ddcVpXPVKlCs9HBkSdLDEtyxHBUK2SJY0lRgWOZO0t+59Zd\/12vjBQVyV3FRic5M4btccyCLLFO4viSJY4lRQWOZe4s2c5tmj1XNFKh19rFs2Tt4otyV948F4iNDs47ZIlhSY4YjoxU4DiqJYoKLM9cWbOd+9o575Il9z5OUZHSQ2zAU4ILyEaWGJbkiOFIUYHjSFGBZZk7a2GigrtpJncVG\/DkzDj8gWPG4Q+yLC0BnHVGKnAsc2fJdu4zb54l1+zY55WRe1QkdxVFRXJmFBU4ZhQVZFlaAjjrFBU4lrmzZDv34X9tkA3f2u+V8cEvXCYfuGRG7sqb5wJRVOC8Q5YYluSI4ahWyBLHkqICxzJ3lmzn\/skzvyY7Hjt53j03vkruKjY6yZkxUoFjxkgFWZaWAM46RQWOZe4s2c69fc90eeSpY8KNr9K5iaIiHTd2hjhufkuskzi2ZIljSVGBY5k7SxQVOJew0SFLHAGMJdZJDEcOf+A4qiWKCizPXFmznfv5b49zN80M3mEDngGeLytZYliSI4YjRQWOI0UFlmXurBlRseGer8vKh8a98nE3zXRuYgOejhuHP3DcOPxBlqUjgLPMSAWOZe4sBYkK7qaZzk0UFem4UVTguFFUkGXpCOAsU1TgWObOknHuNbfeKzpRUy9ufJXOTRQV6bhRVOC4UVSQZekI4CxTVOBY5s4SRQXOJRQVZIkjgLHEOonhqFbIEseSogLHMneWjHNbv3SX6D4VenHjq3RuYqOTjhsjFThujFSQZekI4CxTVOBYprJ09OhRWbFihQwODnr5W1tbpbu7W2prawPtGYfph3PmzJHe3l6pq6uLTDun4w7RHTUpKlK5yMtEUZGeHTtDHDvbEuskjitZ4lhSVOBYJrZ0\/Phx6erqkoULF8rSpUvF\/N7U1CRr1649xd7Q0JAsX75cNm3aJC0tLbJr1y7p7+8PFSHGuU2f+H35l3+\/wLPHcz8Su4miIh2y0FxswDFAyRHDkV8acBzVEkUFlmdma1FCQT\/bv39\/oOAIurFx7isf6JTXznknd9PM4B024Bng+bKSJYYlOWI4UlTgOFJUYFlCrIWJCn9Uw+VmflHxpl8+Lz\/sulwaGxtdsjONRUAb8NHRUWloaAgdmiIwNwJk6cYpLhU5xhFy\/5ws3VkFpdS2UX\/0Gh4eljVr1sj27du9iHo1X6eNj4+f3OGpSi8zv6Ktrc0bDrEvIyoWL14s9913nzcHw3VOxUsf3SC\/Ov0cmfb8z+Wai4\/IsmXLqpRQ5Yp94sQJGRsbk\/r6epk+\/eTyXF7pCJBlOm7+XOSI4ahWyDIby23btklfX98kIxQV2Zhmzm1EgxoKmqhpPj948ODE5MwNGzbIyMhI7JyKY7\/T65VPd9O8+7fOYqQihbeU\/5EjRzx2NTU1KSwwiyFAlpi6QI4YjmqFLLOxtCMVAwMDsnnzZkYqsiHNljtOUJhKb0\/q1L\/pxM3Ozk7p6emR5ubmUwqhwx9tn71ONFKh12cWzpRNn5ydrbBTNDfHr3GOJ0sMS3LEcFQrZIljyYmaOJapLMWt+LCNamRi1qxZE0MjKiruuOMOufPOOwOXlfpFBbfoTuUiLxMbnfTs\/DnJEsOSHDEc+X7jOKoligosz8TW4oYwbIPqLE1v9qbQ\/+sVtPzUOPd3r7tFdPWHl27xLFm7+KLEZWQGigpkHWBniKFJjhiOFBU4jhQVWJaJrfk3vjIGzARM3QBLhzza29snZtHam1+5bJRliwruppnYRRMZ2ICnZ8dIBY6dbYl1EseVLHEsGanAscydJeNcM1GToiK9i9jopGdHUYFjR1FBlqUhgLNKUYFjmTtL6lw7UsHdNNO7iKIiPTuKChw7igqyLA0BnFWKChzL3FnyRyooKtK7iKIiPTuKChw7igqyLA0BnFWKChzL3Fnyi4qjd30kd2WslgJRVOA8RZYYluSI4ahWyBLHkqICxzJ3lmxRcWFdjXeYGK90BNjopOMWlIssMSzJEcORogLHUS1RVGB55soaRQXOHWzAyRJHAGOJdRLDkaICx5GiAssyd9ZsUaFbdO++5rLclbFaCsQGHOcpssSwJEcMR4oKHEeKCizL3FmzRUX75Y1yT\/uluStjtRSIDTjOU2SJYUmOGI4UFSX7JmEAAAuOSURBVDiOFBVYlrmzZosK7qaZzT1swLPxs3OTJYYlOWI4UlTgOFJUYFnmzhpFBc4lbMDJEkcAY4l1EsORogLHkaICyzJ31mxRwd00s7mHDXg2foxU4PgZS6yTOKZkiWPJ1R84lrmzRFGBcwkbHbLEEcBYYp3EcGSkAseRkQosy9xZU1Hxya575JfzPiOMVGRzDxvwbPwYqcDxY6SCLPEEcBYZqcCxzJ0lO1LBLbqzuYeiIhs\/igocP4oKssQTwFmkqMCxzJ0ligqcSygqyBJHAGOJdRLDkcMfOI4c\/sCyzJ01IyrO\/MyfcYvujN5hA54RoJWdLDEsyRHDkaICx5GiAssyd9aKFIaqNFw24DgPkCWGJTliOFJU4DhSVGBZ5s4aRQXOJWzAyRJHAGOJdRLDkaICx5GiAssyd9YoKnAuYQNOljgCGEuskxiOFBU4jhQVWJaprB09elRWrFghg4ODXv7W1lbp7u6W2traSHtDQ0PS2dkpPT090tzcHJiWoiKVSwIz7d+\/X\/r6+qSjo0NmzZqFMzwFLZElxunkiOGoVsgSx7JI\/c5p4+Pj4zg0pbd0\/Phx6erqkoULF8rSpUvF\/N7U1CRr164NLYBJt2fPHtm6dStFReldJUV6UcqAK\/IWZInxADliOBbt2zWOSjpLRaqXVScqgly2a9cu6e\/vj4xWqNM2bNjgZWekIl3FT5qrSC9K0mdHpydLDFFyxHCkqMBxLBrLKSEqdLjklltukfb2dk9YuIiKVatWyYIFC7A1Z4pZGx4eljVr1ghZZnc8WWZnqBbIEcORLHEcbZbbt2+XlpYWrPEyW6t6UWHmV7S1tXnDIWGRDP37vHnzYudUHDp0yOsIBwYGyuwK3o4ESIAESGCqEtAvsRs3bpTzzz+\/qhFUtagw8yTUA2ETNXVypk4WvPHGG0UFQ9xETbWl6fSHFwmQAAmQAAmUg4CKiWoXFMqpakWFi6DQB9ThjkWLFnkhJZfVH+WoPLwHCZAACZAACRSRQFWKCtcVH\/6lp7YDizB2VcQKyWciARIgARKoXgJVKSo0+jAyMuK0N4XtGkYqqreisuQkQAIkQAL5J1B1oiIs+jBnzhzp7e31NsDSfSx0pYd\/Fi1FRf4rJEtIAiRAAiRQvQSqTlRUL2qWnARIgARIgASKTYCiotj+5dORAAmQAAmQQNkIUFSUDTVvRAIkQAIkQALFJkBR4fOvTgLdsmWL91euEImv\/DpPZfny5d7E2biD3Wy2elZL1Bks8XcuXookLM3T+8\/CKR6VdE+UhKV\/nhbf+8nMk7C00\/Idd6+75j0OmgvobiUfKSkqLD+Y80F0wucTTzzh7XGh\/6+rq8uHt3JWCrtDW7JkyaSD3vxF9Z\/Por\/v3LmTfF8HlYSlzVY5rlu3TtavXx+6o2zOqk3Ji5OEpX95OidzT3ZPEpZGnOnBjjpJnu+4W1U3jHfv3l2IL7IUFZbfzYFj+lIUSTm6Ve3kqfwNsIqyHTt2OC31ZeN96rdBe7dXF5baiN9www1y7NgxidqmPrlnqztHknqpae+44w658847+eUhwO1JWdp1mO94\/HtkIjvz58+XgwcPeidt8+yPeG5VkSLsSHVzxHpVPESZC2lHdjSa4\/89qjhscCbTScNSRfDll18u3\/jGN4T19A2eSVi6nHBc5tcqV7dLwjIoUhF3enSuHrYChdED7vTSrRBWrFhBUVEBH5TslkGRCW20Z82axbByCHX\/t+kk3\/rSbmBWsgpQYcNJWZozba6\/\/nrvBF6Kismiwo6YRdVLFRX79+\/3MnMu1akvQdJ6aYfyV65c6XWSvOIJ+AVZfI78puDwx+u+oahIXkmTNjjmDtqQ33333ZyoaSFPwlLr6u233y4dHR3eAUS62RtFRXpRoXNSzORM9cPq1atZN1\/HmaRemlD+pk2bvBB+kshl8tanWDkoKorlT+9pOPyR3KlJQqMUFNF8k7DUtA8\/\/LD3LZCrP4K\/XduTrKM6N\/\/wB3lO5pmkXpJl8jbU5KCoSM8u1znt4Q5O1Ix3lT+sHDe5kLPBw5kmYWkvzbUtMtx8kkYSlv46y\/d+ch1NwpKiIr7NDEtBUZGeXa5zcklpMvckWW7GsHI02yQsbUv8Zn0q1yQs\/Y05Q\/aTeSZhGTT8waEktzaVosKNU1Wm4uZXydwWtTGOmQSnYfqwb9fcaOgN3q4sKSri62gSlvbmV9yw6VS2SViqKLv66qs9I2QZX085\/OHOiClJgARIgARIgASmGAGu\/phiDufjkgAJkAAJkECpCFBUlIos7ZIACZAACZDAFCNAUTHFHM7HJQESIAESIIFSEaCoKBVZ2iUBEiABEiCBKUaAomKKOZyPWzkCTz31lJx11lnOB1fpqoQXX3xRLr744pIV2qzKmTNnTqITY6vl7BbzfOVaiVDu+5WsYtAwCaQkQFGREhyzkUASAkn3PyjHuvUswiBL3iTcsqa1Tx7Oass1f7WwcX0epiOBJAQoKpLQYloSSEkgj6IiaZnsR6+WjpOiImWFZTYSSEmAoiIlOGYjAT8BexMl\/cwMKTzxxBMTGwLp382GX\/4NwUyI\/uyzz\/aOQR4cHPRuYbbftk+AtO3rsfNhl30PewhANybTg7TMtX79+sDTeMPSGVGxZMkSufXWWz0z\/iEGeyMk\/32U1Q033CAf+tCHvPyG1QsvvCDLly+XkZERL8tNN90kDz74oPT09Ehzc7P3t7BnCmLgFxX6+8svv+z97N69exLfoPxBR6PHHZdeLYKLbzAJlIIARUUpqNLmlCMQtF223aH5owJh5yQouO7ubu+gMBUWuhupnvhoBEtbW9tE5x91loopj7FXW1sr\/tNh4yIV\/vT2NswqfLTznz9\/vldeY3\/nzp3e3AwVB52dnZPEgG3PCKcLL7xwIr\/\/Gc3vY2Nj3qmh5kRWFS\/mSO247d+DRIUecW5EVBBXu\/JSVEy5V5kPnJEARUVGgMxOAkogrnOK68DVht2B+UVF0GFtUed+BH1b9qePKlPcmSL+cx60\/HHf0O3Pjajwi6T+\/v4JkaE2bdGgv99xxx1y5513TprsGjXEESQqNApihJC5h32qKUUF32kSSE+AoiI9O+YkgUkE7KEC\/2qKsA7cP0TQ2toaGKnwD0PYNw4augi7n30eS5SoiJsoGiQggv7mHxLyD\/GYSIw+T5A4sG1q9MOcK+GvemEntAaJCs1rIh1xYoiRCr7kJJCMAEVFMl5MTQKxBOyO1J5XYX8bNiLBP8\/BfFP3Ryo0r\/8bdlRBwgRD1JCMbS+rqFBbZm6EET1BkYokouJHP\/qRmOGVqHkk9nNQVMRWVyYgASgBigooThojgTcI2B2z+Sau8w10\/kFXV5csXLhw0uRIWzj4RUXU\/Ikg5uUY\/vDPmbDvqQIgaijDDH\/YoiIoKmAPf2ikIulR2qUY\/ogTeHHDQHxHSKDIBCgqiuxdPlvZCATNqbCjBfbExaAJhyZyYYY\/tOC28DD2ddKmCd0HzWswD4yaqGlHBux5FvPmzTtlIqZfVNh5TVm1fDrpMkhUuE7UVBtmTkTcXJasEzXN8JRZsWOew56g6q9kFBVle+14oxwSoKjIoVNYpOokYDocsxzSHtqwl4PqcMCVV145admoiomPfexjcvPNN3udri6f9AsNE70wS02VkunswohFLb90nTwatPTUZU6F\/96bNm3y5k3o5Ezz\/HakQp\/Bz9C\/pNS\/rFbzhC2HNdEh\/dcIsaAlpXb+oOey57Oon+bOnSt79+71hI1f\/Jln8EdxqrNGs9QkkJwARUVyZsxBAiRQJgLayQet+HC9vcucCldbrukYqXAlxXRFJEBRUUSv8plIoAoJ+OeNmKiEvS9F0seiqEhKjOlJIBsBiops\/JibBEgASMC\/y2jYUlHXW\/oP+Lr\/\/vu9rPaSUldbLul4oJgLJaYpMgGKiiJ7l89GAiRAAiRAAmUk8P8Bmuqbo9ZN760AAAAASUVORK5CYII=","height":321,"width":533}}
%---
%[output:85a19b40]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"13.2000"}}
%---
%[output:4142069f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"0.0075"}}
%---
%[output:851aa2d0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"0.0075"}}
%---
%[output:3b26f23c]
%   data: {"dataType":"textualVariable","outputData":{"name":"idab_zvs_min","value":"1.6400"}}
%---
