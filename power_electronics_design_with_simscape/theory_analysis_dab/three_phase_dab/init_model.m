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

% model = 'single_phase_dab';
model = 'three_phase_dab';
use_thermal_model = 1;
%[text] ### Voltage application
application400 = 0;
application690 = 1;
application480 = 0;

% number of modules (electrical drives)
n_modules = 2;
%[text] ### PWM and sampling time and data length storage
fPWM = 12e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*2; % PWM frequency 
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
tc = ts_dab/100;

z_dab=tf('z',ts_dab);
z_inv=tf('z',ts_inv);
z_afe=tf('z',ts_afe);

t_misura = simlength - 0.2;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

Pnom = 275e3;
ubattery = 1250;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 1500;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:0994022b]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:60f8fa48]
%[text] ### dead\_time and delays
dead_time_DAB = 0;
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
Pload = 250e3;
Rbrake = 4;
CFi_dc1 = 900e-6*5;
RCFi_dc1_internal = 1e-3;
CFi_dc2 = 900e-6*5;
RCFi_dc2_internal = 1e-3;
%[text] #### Tank LC and HF-Transformer parameters
% three phase DAB
Ls = (Vdab1_dc_nom^2/fPWM_DAB/Pnom/3) %[output:31850292]

f0 = fPWM_DAB/5;
Cs = 1/Ls/(2*pi*f0)^2 %[output:3eae90ae]

m1 = 12;
m2 = 12;
m12 = m1/m2;

Ls1 = Ls/2;
Cs1 = Cs*2;
Cs2 = Cs1/m12^2;
Ls2 = Ls1*m12^2;

lm_trafo = 5e-3;
rfe_trafo = 1e3;
rd1_trafo = 5e-3;
ld1_trafo = Ls1;
rd2_trafo = rd1_trafo/m12^2;
ld2_trafo = ld1_trafo/m12^2;
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
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 1;
ki_v_dab = 45;
%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:059ce2f2]
Iac_FS = I_phase_normalization_factor %[output:6dd6a7c7]

kp_afe = 0.15;
ki_afe = 18;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);
res_min = s/(s^2 + 2*delta*omega_grid_min*s + (omega_grid_min)^2);
res_max = s/(s^2 + 2*delta*omega_grid_max*s + (omega_grid_max)^2);

Ares_nom = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Aresd_nom = eye(2) + Ares_nom*ts_afe %[output:8559a343]
a11d = 1 %[output:1c320fa5]
a12d = ts_inv %[output:21ca4643]
a21d = -omega_grid_nom^2*ts_inv %[output:9d95cdce]
a22d = 1 -2*delta*omega_grid_nom*ts_inv %[output:54c7dc04]

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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:36c7af40]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:0e9ff8e7]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:62ec59a3]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:30b6bab7]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:8dea7355]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:420fcd2d]
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
%[text] ## Lithium Ion Battery
typical_cell_voltage = 3.6;
number_of_cells = floor(ubattery/typical_cell_voltage)-1; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 50; %Hr*A

Vbattery_nom = ubattery;
Pbattery_nom = Pnom;
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
figure;  %[output:0fad934a]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:0fad934a]
xlabel('state of charge [p.u.]'); %[output:0fad934a]
ylabel('open circuit voltage [V]'); %[output:0fad934a]
title('open circuit voltage(state of charge)'); %[output:0fad934a]
grid on %[output:0fad934a]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:74597843] %[output:88e088c4] %[output:18ca9ceb]
%[text] #### DEVICES settings
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

% danfoss_SKM1400MLI12BM7; % 3L-NPC Si-IGBT
% inv.Vth = Vth;                                  % [V]
% inv.Vce_sat = Vce_sat;                          % [V]
% inv.Rce_on = Rce_on;                            % [Ohm]
% inv.Vdon_diode = Vdon_diode;                    % [V]
% inv.Rdon_diode = Rdon_diode;                    % [Ohm]
% inv.Eon = Eon;                                  % [J] @ Tj = 125°C
% inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
% inv.Erec = Erec;                                % [J] @ Tj = 125°C
% inv.Voff_sw_losses = Voff_sw_losses;            % [V]
% inv.Ion_sw_losses = Ion_sw_losses;              % [A]
% inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
% inv.Rtim = Rtim;                                % [K/W]
% inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
% inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
% inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
% inv.Lstray_module = Lstray_module;              % [H]
% inv.Irr = Irr;                                  % [A]
% inv.Csnubber = Csnubber;                        % [F]
% inv.Rsnubber = Rsnubber;                        % [Ohm]
% inv.Csnubber_zvs = 4.5e-9;                      % [F]
% inv.Rsnubber_zvs = 5e-3;                        % [Ohm]
% 
% wolfspeed_CAB760M12HM3; % SiC Mosfet fpr 3L - NPC
% inv_mosfet.Vth = Vth;                                  % [V]
% inv_mosfet.Rds_on = Rds_on;                            % [Ohm]
% inv_mosfet.Vdon_diode = Vdon_diode;                    % [V]
% inv_mosfet.Vgamma = Vgamma;                            % [V]
% inv_mosfet.Rdon_diode = Rdon_diode;                    % [Ohm]
% inv_mosfet.Eon = Eon;                                  % [J] @ Tj = 125°C
% inv_mosfet.Eoff = Eoff;                                % [J] @ Tj = 125°C
% inv_mosfet.Eerr = Eerr;                                % [J] @ Tj = 125°C
% inv_mosfet.Voff_sw_losses = Voff_sw_losses;            % [V]
% inv_mosfet.Ion_sw_losses = Ion_sw_losses;              % [A]
% inv_mosfet.JunctionTermalMass = JunctionTermalMass;    % [J/K]
% inv_mosfet.Rtim = Rtim;                                % [K/W]
% inv_mosfet.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
% inv_mosfet.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
% inv_mosfet.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
% inv_mosfet.Lstray_module = Lstray_module;              % [H]
% inv_mosfet.Irr = Irr;                                  % [A]
% inv_mosfet.Csnubber = Csnubber;                        % [F]
% inv_mosfet.Rsnubber = Rsnubber;                        % [Ohm]
% inv_mosfet.Csnubber_zvs = 4.5e-9;                      % [F]
% inv_mosfet.Rsnubber_zvs = 5e-3;                        % [Ohm]
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
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);
% 

%[text] ## Enable/Disable Subsystems

if use_thermal_model
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
else
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_1/three_phase_inverter_1/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    set_param('three_phase_dab/dab_modA/dcdc_with_galvanic_isolation/three_phase_inverter_2/three_phase_inverter_2/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":40.5}
%---
%[output:0994022b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:60f8fa48]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:31850292]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     7.891414141414142e-05"}}
%---
%[output:3eae90ae]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     1.393166275082144e-05"}}
%---
%[output:059ce2f2]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:6dd6a7c7]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:8559a343]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000083333333333"],["-8.224670334241132","0.998691003061004"]]}}
%---
%[output:1c320fa5]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:21ca4643]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     8.333333333333333e-05"}}
%---
%[output:9d95cdce]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"  -8.224670334241132"}}
%---
%[output:54c7dc04]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.998691003061004"}}
%---
%[output:36c7af40]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.031062519151360"],["1.619345474049183"]]}}
%---
%[output:0e9ff8e7]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.125263346003807"],["30.829381225835562"]]}}
%---
%[output:62ec59a3]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:30b6bab7]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:8dea7355]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000083333333333"],["-8.224670334241132","0.998691003061004"]]}}
%---
%[output:420fcd2d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.129590696960579"],["22.638405094998713"]]}}
%---
%[output:0fad934a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ9431V97z9ubJKJ0AZhJcSurKSK06WU4UJAq+tYt92l7Lr5JOm9m4vR203Qey\/0SVJR7hCkSday6wBnxxOzbs+adruCtlOHWtHJjZ2s0Nw5EQKhLSFU\/pQKatGhvc\/nW044+eb755zzPZ\/f7\/s9eX+fh6ckv8\/5fM95fc7vnHfO31ecOHHiBOEBARAAARAAARAAgQoReAUETIWihayCAAiAAAiAAAhEBCBgUBFAAARAAARAAAQqRwACpnIhQ4ZBAARAAARAAAQgYFAHQAAEQAAEQAAEKkcAAqZyIUOGQQAEQAAEQAAEIGBQB0AABEAABEAABCpHAAKmciFDhkEABEAABEAABCBgUAdAwBOBo0ePUm9vb+RtZGSEGhsbPXme72ZycpJ6enrooosuosHBQWpoaBB7l+64lmU0KZDi8P73v586OztNkpTaZmhoiLZt2xblccOGDdTf32+V3127dtGmTZto8+bNpeTB5du3b5\/498MKGowrSwACprKhQ8bLRqCWnXuagOEOYvXq1dTW1iaCJ62M0u9NK4xLh8gd6Fe\/+lUrceCSxjYA\/I7169fPJgtRwIQmOG1jDHu\/BCBg\/PKENxCoC4Hjx4\/TwMAA7dmzh3bs2FEzAcMjP7V4bxJU1Rl2dHQYixE1QmEjDlzSuFQCJWBs8hZ\/T9lHYFQ9PXz4MEZhXCoJ0swhAAGDClFKAvpQOmdQHxLXRx9WrlxJN9xwQ1QG7sj06RQ1WjAxMTHvc93HFVdcQe95z3sim6amJhodHaWWlpZELrpQiNvHRyf4czWltGbNGrr55puptbU1arj1jj\/PD09Fqffu378\/yh8\/agrpuuuuo4985COReFFPnAX\/XjHVBY7qNHV71QkqX3qHqpfx1ltvpeHh4cT3Tk9PR\/mbmZmZzZMeQ50jM7\/tttvok5\/8JKnyMf84a8VOTc0lddYqrvp7VXnj5VKxbm5unhVhcX67d++OpmTUo9ePuL884Rivj1m+suph1kiNzoTzrPIeF0Xx75fOVvm4+uqrae\/evcTfHxU7vcz8O\/UOPbZ5XJLqYSkbIWSq9AQgYEofooWVwXinpZdeNcJJnVS842E\/LB6UeIl\/ntTBZnX+\/Fla3lRnc+aZZ85ZA6MEjJ4HFgpJgkMXMXE\/vgRM0l\/48c4k3rGlceXfpwmYvr4+uuqqq+axzxIM\/NlZZ51FTz31VCTQkkQFv1PvaON5T6sX6r333Xdfohi54447Zted6PVN76DjAibuS32eJmKy6iynOXToUKpQ0vMUFy9xkanEw1ve8hb62te+NqfxSBIhSd+vuABhm6Q88u\/Ve\/J861zKPkq0sFrcapcWAqba8Qsu96qB1v8CVY0\/F1YffeC\/slXDqHcQemOr\/+Wpd3gsEtQIgfKh3h3\/S19BTvpcb4wvv\/zyVAGj\/4Vq6ydPwPCoEz95UzlZI0Q8KvTMM8\/MY6KPGjCnFStWzCmjyRRSfHpLsVfx5NGWeNw5L7weJGlkiFmuW7cuKq8+YmOysNlkOihuE\/9ZMVFii\/Of925V95LKo37HQpfLnDaFpHNU9Ske0y9+8YuREEoaUUnzGx+FU6NOuo+sd6sRGlX\/87j4mCoLruFDgZwIQMA4YUMiKQJpf52pDoAb7lWrViXuwNFtDh48mPhXNedb98F\/9asdQ3mLcPP+ckwTCHqDzu+39eNLwOjvZjHCj95hpnUsegf+3ve+11jAJI1YJb2X8xGfIksb4WBb7ohVPnS2Se+LT6VlCZi0qbN4mqzRlCTxGy+bmp6MCyEl2tKERl791OOr+0iLa3w0R7FSAiZt6lDfYafXZfW91KfvVDuhc0matpRqT+A3bAIQMGHHt3Klq7WA0bch53UQtsKD4Sdtqzb1o3fO8c6OfevbqE1GYNhGX\/jKP\/OW3fgIVLwDtRUwcY7xUZq4cPIlYFRlTxNOvDMrScDEp6LyRmCqIGCSRvxUXOP1L20ERveR9t2AgKlcExtUhiFgggpn9QvjOoUUn+pQawrS\/ppNGvLPEzBZUz\/6qABHgf9KTRMwpn54aD4uLtTUWlzAsEgwWRwZ79z1EYr4NBx3+HlTSDw6lCcA4j5cp5D02p02qhH\/BsTFSHw0QpVZH4lT5VF1J54maQop75snPYWkxK4auUoTMEkjV4pRfAQmbdF1fPoqawopiQumkPJqCz43JQABY0oKdjUhIL2IN0sA5AmYrLwlrQ9JEzB5fni4Xa1niUM3ETCcJmkXkvIV30miHwBns4hXTSXoafi973jHO6LRoaSHOSWVz3QRL\/tUoi4unNIWuOppdBt+58c+9jG68cYb5y045jRxAcO\/S1sQrMqaJ5iTplfyRsB0jmllzBIfumD4wAc+kFq3snxwHpIW95ou4tW55I1A1qShwUuCIAABE0QYwyuE6TZqfQt03jbqpIXBNlNITDlreiJvkax+Mm+WH35PfMvthz\/8YTpw4MDsotWkERi9c0tbiKz7jq\/NSRI4ekeup+X\/VwIm6b233377nBNl+XA9fb2NyzZqXYjoHWrSFvu07dtxrvqaHPbJ3LZs2UIbN26McOgjaWo3Wdq27LzzW7K2UfO7TEcm0tau8ChckjhIG3ViRklb2JNGcdLEL\/8+fvJv2loi5cNkpDC8Fg0lkiAAASNBFT5FCeTt+BB9OZwXJpA0VRXfaZZ2Do\/+cpeD7ApnfoE60AWnEmpJO5Py8OAguzxC+NyGQJAChhs2PouCD9lKagizDjizgQfb+hCAgKkPd19vzZpCy5r6Snq\/y1UCvsqx0PwkTSExg7zDH5NEZyh3Vy20OlC28gYnYPIW96nP29vbo8vO1M\/8JbS9OK1swVwo+YGAqX6k439EcIlsxQunwd06ta0L8aldG\/HCOYXgrG28Qn9bcAKG53v5S8JP2ghMPKj8l8X4+HhNb\/UNvWKhfCAAAiAAAiAgSSAoAcN\/1V1\/\/fXU3d0diRgIGMmqA98gAAIgAAIgUD8CQQkYHknhh0+EzFoDo+NWQ9ldXV3RlBIeEAABEAABEACB8hMIRsDwXPj27dvp2muvJb6oz0TAqPUvHCb9FuN42H7xF3+x\/JFEDkEABEAABEAggQDfKn7eeecFxyYYAcNTRnzWBJ8emrcLiaNoKl7YlgXM1NRUcMGvd4HAVS4CYAu2cgTkPKPeyrANlWsQAiZpR4OqBknX29vuPAo1+DJfFXOv4GrOytYSbG2JmduDrTkrW0uwtSVmZh8q1yAETDyEeSMwPFrDp1BmTRvpPkMNvlnVl7MCV7CVIyDnGfUWbOUIyHgOtc4uCAGjRlx4d9KKFSuiG4LVseCqumQdvR5q8GW+KuZewdWcla0l2NoSM7cHW3NWtpZga0ss3\/7w0Reo7b9uopnP\/Xm+ccUsghQwvmOAL5Vvoif9Pfroo0EuLJOhZecVbO142ViDrQ0tO1uwteNlYj127xG6cuwBOnrz203MK2UDAWMQLggYA0gOJmisHKAZJgFbQ1AOZmDrAM0wCdgagrIwg4CxgBWiKQSMTFTRWMlwxeiWHFewBVtZAv69Q8D4Z1opjxAwMuGCgJHhik5WjivYgq0sAf\/eIWD8M62URwgYmXBBwMhwRScrxxVswVaWgH\/vQ3cdpKG7HsUaGP9oq+ERAkYmThAwMlzRycpxBVuwlSXg3zsEjH+mlfIIASMTLggYGa7oZOW4gi3YyhLw7x0Cxj\/TSnmEgJEJFwSMDFd0snJcwRZsZQn49w4B459ppTxCwMiECwJGhis6WTmuYAu2sgT8e4eA8c+0Uh4hYGTCBQEjwxWdrBxXsAVbWQL+vUPA+GdaKY8QMDLhgoCR4YpOVo4r2IKtLAH\/3vkUXt5KjZN4\/bOthEcIGJkwQcDIcEUnK8cVbMFWloB\/7xAw\/plWyiMEjEy4IGBkuKKTleMKtmArS8C\/dwgY\/0wr5RECRiZcEDAyXNHJynEFW7CVJeDfOwSMf6aV8ggBIxMuCBgZruhk5biCLdjKEvDvHQLGP9NKeYSAkQkXBIwMV3SyclzBFmxlCfj3vu62++meR45hEa9\/tNXwCAEjEycIGBmu6GTluIIt2MoS8O8dAsY\/00p5hICRCRcEjAxXdLJyXMEWbGUJ+PcOAeOfaaU8QsDIhAsCRoYrOlk5rmALtrIE\/HuHgPHPtFIeIWBkwgUBI8MVnawcV7AFW1kC\/r1DwPhnWhqPk5OT1NfXR8PDw9TS0pKYLwgYmXBBwMhwRScrxxVswVaWgH\/vK2\/8Oh0++gIW8fpHW1+Px48fp4GBAdq\/fz+Njo5CwNQ4HBAwcsDBFmzlCMh5Rr31z7bx6rsjp7hKwD\/bunrct28fDQ0NRXnACEztQ4HGSo452IKtHAE5z6i3ftnyyAuPwEDA+OVad29Hjx6l66+\/nrq7uyMRAwFT+5CgsZJjDrZgK0dAzjPqrV+2EDB+eZbG265du6K8rFq1Cmtg6hQVNFZy4MEWbOUIyHlGvfXL9p6Hj9G6j9+PERi\/WOvrjRfubt++na699lqanp42EjB6jvfu3VvfAgTydmbf3NwcSGnKVQywlYsH2IKtHAE\/ntesWRM5euH1V9ALr18HAeMHazm88JTR6tWrqa2tjbALqX4xwV9bcuzBFmzlCMh5Rr31y3boroM0dNejEDB+sdbPG6996e3tpYmJiXmZ2LFjRyRq4g+2UcvEC42VDFf2CrZgK0dAzjPqrV+26gyYn\/rB0\/T0J97p13kJvL3ixIkTJ0qQj7plASMwdUOPTlYQPToCObhgC7ZyBPx6VluoT3n6QXryb\/7Yr\/MSeIOAwUF2dauG6Ajk0IMt2MoRkPOMeuuPrb4D6ecm\/pam937Sn\/OSeFrwAsYkDphCMqFkb4PGyp6ZaQqwNSVlbwe29sxMU4CtKal8O13AnHbPMB3+xufzE1XMAgLGIGAQMAaQHEzQWDlAM0wCtoagHMzA1gGaYRKwNQRlYKbWvyxtPJWe++R\/oampKYNU1TKBgDGIFwSMASQHEzRWDtAMk4CtISgHM7B1gGaYBGwNQeWY6aMvly1fRN\/c+nsQMH7QVs8LBIxMzNBYyXBlr2ALtnIE5Dyj3vphO3bvEbpy7IHI2W3dF9C1ne0QMH7QVs8LBIxMzNBYyXCFgJHjCrZgK0uguHcefblq7AG655FjxNNHBz50CYXah2EKyaC+hBp8g6KLmkDAyOEFW7CVIyDnGfW2OFt99KXvN5bRwG+eBwFTHGt1PUDAyMQOjZUMV4wSyHEFW7CVJVDMO4++8N1H\/C+Pvux+34XRv6H2YRiBMagvoQbfoOiiJhAwcnjBFmzlCMh5Rr0txvavvjZNA3dORk76155H\/WuXRf8fah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn11p2tPnWk1r4ob6H2YRAwBvUl1OAbFF3UBI2VHF6wBVs5AnKeUW\/d2Orbplm83Np1AV12\/qJZZ6H2YRAwBvUl1OAbFF3UBI2VHF6wBVs5AnKeUW\/t2erihVPzuhddvGAKyZ5pUCkgYGTCicZKhit7BVuwlSMg5xn11o5tXLz8919bSv\/rd5bPcxJqH4YRGIP6EmrwDYouaoLGSg4v2IKtHAE5z6i35mzvefhYtONIPe+6pIn+\/J2vS3QQah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn1Np8tj7r8w33foY9+7uW7je7441Z624rG1MSh9mEQMPn1JdgtaAZFFzVBYyWHF2zBVo6AnGfU22y2POpy1c4HonNe+OEFu7xduvviJZkJIWDk6mzpPYca\/HqDR2MlFwGwBVs5AnKeUW\/T2b5\/57fp777xxKyBflBdXkRC7cMwApMX+YAPATIouqgJGis5vGALtnIE5Dyj3s5lyyMt\/zz5LH1g17fnfLD9j95IHb98lnEgIGCMUYVnGGrw6x0pNFZyEQBbsJUjIOcZ9fYkWxYun\/\/3p2nTS6fqKuLxA+pMIxFqH4YRGIMaEGrwDYouaoLGSg4v2IKtHAE5z6i3RFft\/Dbt0KaKmHbS4XQ2UQi1D4OAMagFoQbfoOiiJmis5PCCLdjKEZDzvFDrLS\/OHb7rUbrnkWNz4BYVLspZqH0YBIzBdzHU4BsUXdRkoTZWolBfcg62cpTBFmx9EGDRcueBJ2l0\/PF57nwJFwgYH5GqgY\/jx4\/TwMAA7dmzJ3rbhg0bqL+\/P\/XNu3btok2bNkWfd3R00ODgIDU0NCTaQ8DIBBAdgQxX9gq2YCtHQM5z6PWW17b8\/f4jdNPnH00ULZcuXxRti2YB4\/MJtQ8LZgRmaGgoijeLlqNHj1Jvby91dXVRZ2fnvHqwb98+YvuRkZFItLDwaWpqShU8oQbf5xfExVfojZULE19pwNYXyfl+wBZsbQjwSMtd33qabvvKY4nJfI+2JL0k1D4sGAETD5ouaOKf8ejL+Pj47KhL\/Oe4fajBt\/kSStiiI5CgetIn2IKtHAE5z6HU27Q1LYqcEi38r+\/RFggYufpZE89qBIZHY9ra2oxGYNrb2xNHazgxBIxM2EJprGToFPMKtsX4ZaUGW7CNE+Cpob\/9lxn6l6nvzluIq4uWv+h8PS07s6EmokXPY6h9WHAjMDzysm3bttx1LZOTk9TT00MzMzO0Y8eORKGjKgAHX3\/27t0r9w1eQJ6np6epubl5AZW4dkUFWznWYLuw2c4892IE4PHnXqTbv3GM9j9+8lj\/+NN0+il0zqtPoQ2\/uij6l3+u1bNmzZp5r5qaevnupFrlQ\/o9wQkYBYyFDIuTpMW5PGW0c+fOaA1MY2NjtB6Gn7RFv6GqV+nKlecff8nmEXL\/HGzd2eWlBNs8Qu6fl5WtWscy8djzqSMsXGqeDur6lSV02fmL6bLzF7mD8Jwy1D4sWAHDIyx9fX00PDxMLS0ts9VB7VbSp4zSbFWiUIPv+Tti7a6sjZV1QUqYAGzlggK24bNlwfKp+79Ddz94dPbixKRSs2C5bPli6rqYRUt5BEs8r6H2YcEKGH2nEY+yqAcCRq7xsfWMjsCWmLk92JqzsrUEW1ti5vb1YMti5cnnf0R\/Pf545uiKGmHhrc7dF58TjbbUYgGuOb10SwgYHxQ1H2qh7cTEhJXn1tZWuvPOO+el0aeBlEhJ2xqdNIWUNt3ELwo1+FbgBYzr0VgJFKOULsFWLixgW122LFae+t6PaPT\/5osVJVj+88qzqaf93KjQVREsGIGRq6OR57ydQkmvV6MqSQImfpCdfjid+qy7u3t2sa5a7MvvwUF2wsFOcY+OQI472IKtHAE5zz7rLYuVZ77\/Ixq5x1ysLF18KvW9dJBcVcVKUnRC\/SO8blNIvgWM3FcKIzBSbH02VlJ5rKpfsJWLHNiWhy1vX2ahwf9u\/dJBevSp47nTQGokhcXKe97STI0\/9zOVmg5yoQ8B40ItkDShBr\/e4UFHIBcBsAVbOQJynvPqLY+qPPid79NnDjxpJFR0sfLrF5xJq5aeXurFtlJkQ+3D6j4Cw2tg8u4tkgqqqd9Qg29afim7vMZK6r0LwS\/YykUZbGXZ\/vQZ50QjKvce\/O7JXUDPvpC5E0jPDY\/GqEW2s+LF871CcqWX8xxqH1Y3AaNCpa9F4UW3o6Ojc7Y9y4XU3HOowTcnIGOJjkCGK3sFW7CVI+DHM4sUfr784FG6477vWAsVngJ62+sa6c3Lzgh+Cqgo8VD7sLoLGBWY+K6kMo3KhBr8ol+KounRyRYlmJ4ebMFWjoCdZ5724edz33yKvvn494ynftQICv+rRlXKfNaKHZXaWofah5VGwOjh1I\/5L8OoTKjBr+1XaP7b0MnKRQBswVaOwHzPPJrC\/6nzVNjinkdOCheTR+34af35U+iG33tT5KtK56yYlLGeNqH2YaUUMHqg0w6kq2VlCDX4tWSY9C50snIRAFuw9U1AiZSpp39A\/\/Lod+mxoy9YiRQ1osJTP+3nL4pOsFXCRf2Leus7aif9hdqHlVLA6CMwDD\/vskWZkL\/sNdTgS3PL84\/GKo+Q++dg684uL2XobHnK5+zTf5b+4suH6fAzZtuSdWZKjPC0zzWXL6OZYz80Hk0JnW1e3ZL6PNQ+rDQCJusgOqmgmvoNNfim5ZeyQ2MlRRaLeOXIVputmpphkfLsD\/6D\/unfn3YaSdFHU\/6grYnOOeOV80ZTXGKANsGFWn6aUPuwugsYfRdSGUZbkqpCqMHPr\/ayFmis5PiC7cJmq3b47Lj3CRp\/+JjVDp\/4aApP+Vx6\/iK6dPni6CPJtSmotzL1NtQ+rG4CRt91lHeUv0xIzb2GGnxzAjKWaKxkuLJXsA2brRIoDxz5Pu2eeNJ5FEUfSblw6enUe+m5dV1Ai3orU29D7cPqJmAefvhh+sAHPkDXXXfd7P1EeaHLugspL22Rz0MNfhEmPtKisfJBMdkH2FaXrT7Nw6UYu\/eJSKDYHOgWH0Xhny87fzFdsORV1Nr8atFRlCLkUW+L0EtPG2ofVjcBg7uQZCpqlbyisZKLFtiWm63a0cPTMbfefZi+feT7zgJFH0X5pXNPo\/\/0xrOiwlfxzBTUW5l6CwHjmWv84DpT962trZR0G7Vpehe7UIPvwsJnGjRWPmnO9QW29WfLC2Vf+5JAedCDQIlEyfLF9D9+fSkd+e6PvCyalaPk5hn11o1bXqpQ+7C6jcDkAS\/T56EGv96M0VjJRQBsZdnyfT38vPDiT+iWLx+mQ88cLzyCcnLUZDGdf1YD\/covnBH5l1wwK0fI3TPqrTu7rJSh9mEQMAb1JdTgGxRd1ASNlRxesC3GVk3xfOFbT9OBx56PnNmcLBt\/u342yq+9rpEuXnZGtFi2itM8xchmp0a9laEbah8GAWNQX0INvkHRRU3QWMnhBdt0tkqcNC16Jd38pUPRYW2uC2TVW6KRksWnRlNGV7SeTa9f8qrZDCjxIhftcDyj3srEMtQ+DALGoL6EGnyDoouaoLGSw7tQ2artxfzv1x5+lr7+0n08PkZPWKC8qfnVdOGZ\/0FLlpyz4KZ35Grry54Xar2VZhtqHwYBY1BzQg2+QdFFTdBYyeENjW18a\/G3nvge\/eP\/e6rw1A47UCMkvP6kedEro3Uo+u\/jIyihsZWrhfaewdaemUmKUPswCBiD6IcafIOii5qgsZLDWxW2SpioaR0m8twLL9Jn\/+2pQmef6FM7kRhZfCq98dxX02+\/8TWz4sR1aqcqbOVql5xnsJVhG2ofVioBs2vXLtq0aVMUQb7A8dChQzQ+Pk6Dg4PU0NCQGdn4XUobNmyg\/v7+1DR8KN769eujz3lr9sjICDU2Nibahxp8ma+KuVc0VuasbC3LxFY\/8+QL33qGDkw\/72XdiRolYXGy7DUN1PHLZ1HDz\/y0+PbiMrG1rRdltwdbmQiF2oeVRsDwnUgzMzPU19dHV111VSQ+WFgMDAxQU1NTphjhkHN6fjidOmOmq6uLOjs759UIddv1li1bolOAWThlCaVQgy\/zVTH3isbKnJWtZa3Y6mtO7j34Xbr7waNRVousOdFHT1icrPj5V9HysxuiA9rUiI0SL7ZcfNjXiq2PvFbNB9jKRCzUPqwUAkY\/lXfFihXU29sbCREWF+r6gKwRkqSQ64Im\/jkLloMHD+aKIpUu1ODLfFXMvaKxMmdla1mUrb7m5ASdoM9\/82n65uPf8y5OeNfOey9rpu\/98MfiIye2DNPsi7L1lY8Q\/YCtTFRD7cOCFDBZ1xSoqab29vbE0Zmk6hNq8GW+KuZe0ViZs7K1TGMbXwx75LkfRqMmRe7a0fOm1pXwyMnSM0+ljl8+O7p\/J6QzT1BvbWujuT3YmrOysQy1DyuFgOFAqGkcfQpJjcakTQWljbxs27aN0m64VgJm7dq1dPvtt9PExATWwNh8EzzaorHyCPMlV2q9yRNHnqBHf9BA90w+63XUhJ2xOHlD02n0m7\/0Gjrlp14RHcamppJcF8b6JyHnEfUWbOUIyHiGgJHhOservrBWfbB582bjkRLdmVpTE18ArATM4cOHZxfuptkqfxx8\/dm7d28NaIT\/iunpaWpubg6\/oJ5KuP\/xFyJPP\/7JCfrcg9+nmedepCeefzH6t+jTdPopkYtzXn0KvWnJK6n9Fxqi\/1eP+rzoe0JIj3orF0Ww9cN2zZo18xxNTU35cV4iL6UZgfHNhBfq8mjO8PAwtbS0zLpPmkJKs9UFTIjB983c1h\/+kj1JjC\/942fJGT9Lt979GE099YOTv3\/pEDZbrrq9PqXzS02n0eoVi+kN55xWisWwRcpVz7Sot3L0wVaGLUZgZLiKec1a\/MsjLsuWLZsd2WEBc9NNN9HWrVsTt1KHGnwx+IaOQ2+s1LQK4xgdf5z2H3ouIlP02HqFd1acNDbQm5pOow1vbZ4VJo899hhd2vqycDcMCcwMCIRebw0QiJmArQzaUPuwUozAqEW3vB4l68k620XfdaRGWdK2X8fFTdaOJc5PqMGX+aqYe61aY6UfuqZK+ekDT9KXHnhGRpi8tBB24+XLaPrZH1rt0qkaW\/NaU39LsJWLAdjKsA21DyuFgOGQ8SLenTt3zjlQTj\/PZd26dZlnwsQPstMX8arPuru7o63Z\/OjrbdIW\/KqqFGrwZb4q5l7L0lglCZNvvHSmia\/dOUxFn87h7cN\/0NZEL\/74hJUwMaVbFram+a2SHdjKRQtsZdiG2oeVQsBkbXvWR0seeuih6MC6O++8UybKKV5DDX5NISa8TLqxih9TzwLi3x7\/Hn3um36OqdeLpN9GfOHS0+n1P\/8qEWFiGjNptqb5CNEObOWiCrYybEPtwyBgDOpLqME3KLqoSdHGSr8\/h3fm\/P3+I97OM1EF10dN3v66Rrp42RnRR5FgaTxVlE8R50XZFnl36GnBVi7CYCvDNtQ+rBQChkOWN4XEVwKos2I+9rGPyUQZIzA15Zp12JpaAHvmaT9DH\/\/KY3TomePeFr\/Gp3MuWb6I3qLdQFxmYWIaIHQEpqTs7cB2YDtJAAAgAElEQVTWnplpCrA1JWVnBwFjx8vJOukcGL7UUd1XdMstt9Do6OicbdFOL7JMFGrwLTF4Mdcv9vvrrzxI\/\/rETyK\/3nfmLD6VLvqF06mn\/dw5h6yFIE5MAoGOwISSmw3YunEzSQW2JpTsbULtw0ozAmMfktqlCDX4EgSVQPmPH\/+EPnX\/k15vHY5GTl46BfZ9q1+7IIWJaczQEZiSsrcDW3tmpinA1pSUnV2ofRgEjEE9CDX4BkWfZ6LfPvx\/7vtOdOha0QPX9AWw77qkiX70oszOHJfyVjUNOgK5yIEt2MoRkPEcah9WGgHDh8n19PTQzMzMvAi2trbO2V4tE+J0r6EGP63E+oV\/Dxz5Pu2ZeNJZpOji5IrWs+n12sV+6AjkajLYgq0cATnPqLcybEPtw0ohYPTj\/dV5L3xmi7rMsb+\/f\/b8FpnwZnsNNfh6qfk4+0\/d\/x165Em7ERUlUFa+9nS64JxX0aXLF826zVtvgsZKrjaDLdjKEZDzjHorwzbUPqwUAiZ+Dox+1D8v7B0bG6P4pYwyYU72GmLwx+49QmPfeMJoZEUJkbVveA1d+baTa0\/4BuKiDxqrogTT04Mt2MoRkPOMeivDNsQ+jEmVUsDwdumDBw8Sj7xk3WkkE+r5XqsefLWwduzeJ4iFS9bDYmV1y2J650VLxM86QWMlV4PBFmzlCMh5Rr2VYVv1PiyNSikEDGdOv49IFy1f\/OIXaXx8HCMwDvWahcvWLx2kv933RGJqNf1za\/cF0ed5Uz4OWchMgsbKN9GX\/YEt2MoRkPOMeivDFgJGhuusV30dDB9ax4Jm27ZtxBcy1uPsF724VQs+r2e5aucDs9uM9bKwSOFbi\/\/kra8Vjmi+ezRW+YxcLcDWlVx+OrDNZ+RqAbau5LLTVa0PM6VQmhEY0wzXw64KwefRli8\/eJSu\/ocH5yFi0XJr1wXiU0K2sUFjZUvM3B5szVnZWoKtLTFze7A1Z2VjWYU+zKY8yrYUAsb0MsfGxkaXMhZOU+bgq\/Ut6z5+\/5xysmj53ZVn07vbz6351JApcDRWpqTs7cDWnplpCrA1JWVvB7b2zExSlLkPM8l\/mg0EjAG9sgY\/aapIjbb42CVkgKaQCRqrQvgyE4Mt2MoRkPOMeivDtqx9WNHS1lXA8G6jTZs25ZZhw4YN0Y6kej1lCz6PuvBuoqG7Hp1FUiXhojKNxkquRoMt2MoRkPOMeivDtmx9mK9S1lXAqEJkTSH5KmgRP2UL\/sobvz7nHqDhd6yg33jDmUWKWJe0aKzksIMt2MoRkPOMeivDtmx9mK9SlkLA+CqMlJ+yBJ+njPS1LlUcddFjhMZKqsYSgS3YyhGQ84x6K8O2LH2Y79JBwBgQrXfwecrorn9\/mvrvnJzN7d\/0vJF+501nGeS+vCZorORiA7ZgK0dAzjPqrQzbevdhMqWq40m8atpoYmIit2wL+TJHFi+7\/vUIbf6nk+tdqj7qghGY3OruxQAdgReMiU7AFmzlCMh4hoCR4VoJr\/UKfnyxLouX3e+7sLTbom2DiY7Alpi5Pdias7K1BFtbYub2YGvOysayXn2YTR5dbIOZQlIn+e7ZsyfiYLpzaXJykvr6+mh4eJhaWloSGdYr+Lff8zj13\/HQ7MhLSOKFC4XGyuUra5YGbM04uViBrQs1szRga8bJ1qpefZhtPm3tSyVgkrZVb968mfhqgbxHv0tJTU91dXVlplWiZ\/\/+\/ZnXFdQj+LxN+sqxB4IVLxAweTW62OfoCIrxy0oNtmArR0DGcz36MJmSzPVaGgHD4mXnzp00MjJC6sRdUyGSBEoXNGkg1aWR\/HmZRmAWgniBgJH9eqOTleMLtmArR0DGMwSMDNfIq++rBEzOlWGb66+\/nrq7u6OLI8siYHjdC5\/zop4DH7okmDUv8SqEjkDuSwW2YCtHQM4z6q0MWwgYGa7eBYy6xbqjo4MGBwepoaEhMec84sPPqlWrjNbA6E727t0rQmPmuRfpT7\/0NO1\/\/IXI\/1+9YwlddO6pIu8qg9Pp6Wlqbm4uQ1aCywPYyoUUbMFWjoAfz2vWrJnnaGpqyo\/zEnkJegppZmYmUcTwwt3t27fTtddeS9wYlWUR79BdB2evB7hs+SLafeWFJaoq\/rOCv7b8M1UewRZs5QjIeUa9lWGLERgZrnO8FlnEG89e1u4iHqVZvXo1tbW1UVl2IelTR6Ftl06rOmis5L5UYAu2cgTkPKPeyrCFgJHhKuZVLdDVFwXzy7IO0NuxY0ckauKPdPBZvFw19gDd88ix6NW8XboKt0kXDR4aq6IE09ODLdjKEZDzjHorw1a6D5PJdb7XUkwhFdltpIqo7zpS26Obmppyb7EuwwiMvutoIUwdYZoj\/4tZ1AIdQVGCEIdyBMG21mwhYISJx6eP0kZD0rIRP8hOX8SrPuMdR\/ERlnoLmIU4dQQBI\/xlwiGBooAhDuXwgq0MWwgYGa6JXtVOIv6QR1FGR0dTT8mtRbYkg\/+Rz07R\/957KCpG\/9rzqH\/tsloUqRTvQGMlFwawBVs5AnKeUW9l2Er2YTI5NvNaiimkrKyymOH1LPG1LGbF82MlFfz46Auf+bKQHjRWctEGW7CVIyDnGfVWhq1UHyaTW3OvpRQw+ghMvW+iZpRSwf\/gpx+mT\/zzY1G0buu+gLovXmIeuQAs0VjJBRFswVaOgJxn1FsZtlJ9mExuzb2WRsCUbdpIRygV\/Mar745ew9umF9roC5cbjZX5F9XWEmxtiZnbg605K1tLsLUlZmYv1YeZvV3OqhQCxuTofzkE+Z4lgj\/8hYM0+E+PLtjRFwiY\/HpXxAIdQRF62WnBFmzlCMh4lujDZHJq57UUAsYuy7W39h38hb72RUUQHYFcXQZbsJUjIOcZ9VaGre8+TCaX9l4hYAyY+Q6+LmAWyqF1SZjRWBlUPkcTsHUEZ5AMbA0gOZqArSO4nGS++zCZXNp7hYAxYOYz+Cxe1n38fuJ\/F+raF4zAGFS6giboCAoCzEgOtmArR0DGs88+TCaHbl4hYAy4+Qy+PvryZ7+3gnovPdcgB2GaoCOQiyvYgq0cATnPqLcybH32YTI5dPNaCgGTtYg37U4jt+K6pfIZ\/HW33b\/g7jxKo47Gyq0+mqQCWxNKbjZg68bNJBXYmlCyt\/HZh9m\/XS4FBIwBW1\/B10dfFtKdRxAwBpXMswk6As9ANXdgC7ZyBGQ8++rDZHLn7rWuAiZ+\/1FaMTZs2JB7KaM7gvyUvoI\/dNdBGrrr5Nbphbx4VxFHR5Bf91wtwNaVXH46sM1n5GoBtq7kstP56sNkcufuta4CRmV7oZwDo6aPFvriXQgY9y+saUp0BKak7O3A1p6ZaQqwNSVlZwcBY8crKGsfwb\/n4WPR7iN++MoAvjpgoT9orORqANiCrRwBOc+otzJsffRhMjkr5rUUIzDFiiCf2kfw\/\/QfH6G\/+PLhKLOYPjoZMzRWcnUXbMFWjoCcZ9RbGbY++jCZnBXzWjcBo08brVixgnp7e2liYiKxNPW+0NFH8Bf6vUdJgUVjVezLm5UabMFWjoCcZ9RbGbY++jCZnBXzWjcBUyzbtU1dNPj69NHW319BPe0L9+wXPXJorOTqMdiCrRwBOc+otzJsi\/ZhMrkq7hUCxoBh0eDf8Nkp+vO9h6I3YfroZeBorAwqn6MJ2DqCM0gGtgaQHE3A1hFcTrKifZhMrop7LYWAUdNJoU4hYfdRckVFY1X8C5zmAWzBVo6AnGfUWxm2EDAyXDO9srC55ppr6IMf\/CC1tLTUIQcnX1kk+Pr00fvfvpSu71het3KU7cVorOQiArZgK0dAzjPqrQzbIn2YTI78eC3FCExWUfgqgbGxMRocHKSGhoZU0+PHj9PAwADt2bMnssk6\/C4+4tPR0ZHpv0jwcXhdenTRWPn5Eid5AVuwlSMg5xn1VoZtkT5MJkd+vFZCwAwNDdHIyAg1Njamlppt+Onv7yclULq6uqizs3NOGiV02tvbo8\/Uz01NTamn\/RYJPqaPIGD8fFXtvKAjsONlYw22NrTsbMHWjpepdZE+zPQd9bArvYBhYTIzM5M7AhOHpwuaPLB8pcH4+HjqO1yDr9999NaWxfTpP1mZl5UF9TkaK7lwgy3YyhGQ84x6K8PWtQ+TyY0\/r6UQMFmLeHlkZHR01GoNjO3VBFICRl\/\/wifv8gm8eF4mgMZKrjaALdjKEZDzjHorwxYCRobrrNe0qR011WP6eh552bZtG+Wta1H+sqablI1r8NX0EfvB9un5EURjZVqr7e3A1p6ZaQqwNSVlbwe29sxMUrj2YSa+62lTihEYBpA0VWQiLtLgmUw9KdHEPrIWCXPw9Wfv3r1GMevYPk0zz71ITaefQnve1WyUZiEZTU9PU3MzuEjEHGwlqJ70CbZgK0fAj+c1a9bMczQ1NeXHeYm8lELAZE35mO5CijOdnJykvr4+Gh4eTpx+MhUv7NdFverrX3B5Y3KNx19bci0B2IKtHAE5z6i3Mmxd+jCZnPj1WgkBY7ILKY6FhU9aOpOdR7o\/l+Bj+3R+RUVjlc\/I1QJsXcnlpwPbfEauFmDrSi47nUsfJpMTv15LIWDi61\/0IuYtsFW2+q6jPIFiMr1UVMDo61+O3vx2v1ELxBsaK7lAgi3YyhGQ84x6K8MWAkaG66xXHjHZuHHjnB1HPA3U09NDW7Zsoba2tswcxA+y0xfxqs+6u7sp7ebrrBuvXYKP26fzKwwaq3xGrhZg60ouPx3Y5jNytQBbV3IYgZEhZ+GVRcz69evnpNixY0eueLF4hZOprYDR17\/0rz2P+tcuc3pv6InQWMlFGGzBVo6AnGfUWxm2tn2YTC78ey3FFJL\/Yvn1aBt8\/fwXbJ9OjwUaK7\/1VPcGtmArR0DOM+qtDFvbPkwmF\/69lkLA6FM8eVNF\/hHke7QNvr7+5cCHLqGljafmv2QBWqCxkgs62IKtHAE5z6i3Mmxt+zCZXPj3WgoBY3tyrn8M2R5tg7\/yxq8TTyOxcGEBgyeZABoruZoBtmArR0DOM+qtDFvbPkwmF\/69lkLAcLFMdxv5R5Dv0Sb4+vqXy5Yvot1XXpj\/ggVqgcZKLvBgC7ZyBOQ8o97KsLXpw2RyIOO1FAIm6y4kLnbWDiEZLHO92gRfX\/+CBbzZ0UFjJVd7wRZs5QjIeUa9lWFr04fJ5EDGaykEjEzR\/Hm1CT4OsDPnjsbKnJWtJdjaEjO3B1tzVraWYGtLzMzepg8z81gOKwgYgzjYBF8t4MX6l3ywaKzyGblagK0rufx0YJvPyNUCbF3JZaez6cNkciDjtW4CRl+4m3a4nCpyVaaQsP7FrpKisbLjZWMNtja07GzB1o6XjTXY2tAyt4WAMWcVnKVp8HGAnV3o0VjZ8bKxBlsbWna2YGvHy8YabG1omdua9mHmHsthWbcRmHjx4\/chZd2PVGt0psHHAXZ2kUFjZcfLxhpsbWjZ2YKtHS8ba7C1oWVua9qHmXssh2VpBEzSBYtqmqmrq4s6OzvrRsw0+H\/3jSfo\/Tu\/HeUTJ\/DmhwuNVT4jVwuwdSWXnw5s8xm5WoCtK7nsdKZ9mMzb5byWQsBkHWTH9yONjY3R4OAgNTQ0yJHI8GwafCzgtQsPGis7XjbWYGtDy84WbO142ViDrQ0tc1vTPszcYzksKyFgeHRmZGSEGhsb60LNNPjqBmocYGcWJjRWZpxcrMDWhZpZGrA14+RiBbYu1PLTmPZh+Z7KZVEKAZO13qUMJ\/SaBB8LeO0rNhore2amKcDWlJS9HdjaMzNNAbampOzsTPowO4\/lsC6FgGEUPFW0ceNGGh0dpZaWlojO5OQk9fT00JYtW6ielzyaBB8LeO0rNBore2amKcDWlJS9HdjaMzNNAbampOzsTPowO4\/lsC6NgFEiZv369XPI7Nixo67ihTNjEnz9BF7cQG1WudFYmXFysQJbF2pmacDWjJOLFdi6UMtPY9KH5Xspn0WpBEz58JzMkUnwsYDXPnporOyZmaYAW1NS9nZga8\/MNAXYmpKyszPpw+w8lsMaAsYgDibBX3nj14nXwWABrwHQl0zQWJmzsrUEW1ti5vZga87K1hJsbYmZ2Zv0YWaeymUFAWMQj7zg4woBA4gJJmis3LiZpAJbE0puNmDrxs0kFdiaULK3yevD7D2WIwUEjEEc8oKPHUgGECFg3CA5pkJH4AjOIBnYGkByNAFbR3A5yfL6MJm3yntdkAJGbdves2dPRHjDhg3U39+fSjsv+FjA61ZR0Vi5cTNJBbYmlNxswNaNm0kqsDWhZG+T14fZeyxHigUpYPhgPH5YtJhcV5AX\/CvHHqCxe49EPrEDybxio7EyZ2VrCba2xMztwdacla0l2NoSM7PP68PMvJTPqjQCRp35MjMzM49Sa2ur6Em8uqBJClFe8LEDya1io7Fy42aSCmxNKLnZgK0bN5NUYGtCyd4mrw+z91iOFKUQMGpKp6mpKXMqRwJZ1j1M6n15wccVAm6RQWPlxs0kFdiaUHKzAVs3biapwNaEkr1NXh9m77EcKUohYExEhAQuHnnZtm0bdXR0ZF4WycHXn717987+OPPci9SxfTr6ecObF9F\/+9VFElkN0uf09DQ1NzcHWbZ6Fwps5SIAtmArR8CP5zVr1sxzNDU15cd5ibyUQsCoEZju7u66nLrLQoanrtJuvM5Sr7hCwL02468td3Z5KcE2j5D752Drzi4vJdjmEXL7HCMwbtyMU\/FdSPW6dZrX3\/T19dHw8PDsPUx6xrOCr+9A2v2+C+my8zECYxp0NFampOztwNaemWkKsDUlZW8HtvbMTFJAwJhQcrRRU0gTExOJHqQX8eaJp6zgX\/vph+kv\/\/mxKN\/YgWRXAdBY2fGysQZbG1p2tmBrx8vGGmxtaJnbQsCYsyq9pb7ryGQBcVbwsQPJPdxorNzZ5aUE2zxC7p+DrTu7vJRgm0fI7XMIGDdupUwVP8jOZBFv2gIo3IHkHmI0Vu7s8lKCbR4h98\/B1p1dXkqwzSPk9jkEjBs3q1S7du2iTZs2RWl27NhBhw4dovHx8cwdQlYvcDTOCj62UDtCJSI0Vu7s8lKCbR4h98\/B1p1dXkqwzSPk9jkEjBs341RqJxAvpr3qqqui82B47cvAwADV43wYPeNpwdd3IPWvPY\/61y4zLi8MIWAk6wA6Ajm6YAu2cgRkPEPAyHCNvOrnwKxYsYJ6e3sjAdPW1kZ5C2wFszXr2kTAYAeSfSTQEdgzM00Btqak7O3A1p6ZaQqwNSVlZwcBY8fLyhoCxgpXMMZorORCCbZgK0dAzjPqrQxbCBgZrrNeef0Lr3fRp5DUaExXVxd1dnYK5yDdfVrw1Q6kaBTp5rfXLX9VfTEaK7nIgS3YyhGQ84x6K8MWAkaG6xyvPF20fv36Ob\/bvHlzXcULZyZPwCxtPDU6AwaPHQE0Vna8bKzB1oaWnS3Y2vGysQZbG1rmthAw5qyCs0wLPnYgFQs1Gqti\/LJSgy3YyhGQ84x6K8MWAkaGayW8JgX\/8NEXiM+A4af74iV0W\/cFlShLmTKJxkouGmALtnIE5Dyj3sqwhYCR4TrHq34OjPqAz4Ph3Uj1fJKCjy3UxSOCxqo4wzQPYAu2cgTkPKPeyrCFgJHhOuuVxcvOnTtpZGSEGhsbo9+r3UllXMSrj8BgC7Vb5UBj5cbNJBXYmlByswFbN24mqcDWhJK9DQSMPTPjFPo26vhoS1nPgcEt1MbhTTVEY1WcIUZg5BiCLdjWnoDMGyFgZLjOGWlRh9fpryqrgLly7AEau\/fIyfxjC7VT7YCAccJmlAhsjTA5GYGtEzajRGBrhMnaCALGGpldAhYqGzdupNHRUWppaZkjbMo4hYRbqO3im2SNxqo4Q4wSyDEEW7CtPQGZN0LAyHCdI1QmJiZy38L3I9155525dj4NkoKPW6iLE4aAKc4QnawcQ7AF29oTkHkjBIwM10p4jQdfX8B72fJFtPvKCytRjrJlEgJGLiJgC7ZyBOQ8o97KsIWAkeFaCa9ZAga3ULuHEI2VO7u8lGCbR8j9c7B1Z5eXEmzzCLl9DgHjxs0qVdI5MGW8SkA\/A4avEOCrBPDYE0BjZc\/MNAXYmpKytwNbe2amKcDWlJSdHQSMHS9r6yqdA8O7j3gXEj84A8Y61LMJ0Fi5s8tLCbZ5hNw\/B1t3dnkpwTaPkNvnEDBu3IxSVe0cGJwBYxTWXCM0VrmInA3A1hldbkKwzUXkbAC2zugyE0LAyHCNvFZNwKz7+P3E00hR3nEGjHPNQGPljC43IdjmInI2AFtndLkJwTYXkZMBBIwTNvNERaeQlAhSW7E7OjpocHCQGhoaEjOhr7fJs40HH2fAmMc1yxKNlR+OSV7AFmzlCMh5Rr2VYQsBI8N1jlfXRbzHjx+ngYEBam9vp87OTlI\/NzU1EZ\/uG3\/0031Z4HDaNFtOGw8+zoDxUxnQWPnhCAEjxxFswba2BGTeBgEjw1XMK4uh8fHxxFGY+GdZtnEBgzNg\/IUMAsYfy7gnsAVbOQJynlFvZdhCwMhwFfOaJUqSRmDU6E1ShvTg6wLm5ne+jv7okiaxMoTuGI2VXITBFmzlCMh5Rr2VYQsBI8NVxKtaD5N1h9Lk5CT19PTQzMwM7dixg+K3YOsZ04OvnwGDQ+yKhQ+NVTF+WanBFmzlCMh5Rr2VYQsBI8PVu1e1\/oUdpy3ijS8YHhoaivKRtF6Gf8\/BV8+Pll5KP1j17ujHv3rHErroXBxi5xrE6elpam5udk2OdBkEwFaueoAt2MoR8ON5zZo18xxNTU35cV4iL684ceLEiRLlp1BWTMRLfMEvv5BHY\/r6+mh4eHj2Juy0ERicAVMoRHMS468tfyzjnsAWbOUIyHlGvZVhixEYGa7evObtPFIvKipg+ARePomXH1wjUCx8aKyK8ctKDbZgK0dAzjPqrQxbCBgZrt688jQQr2fJOvtFvSxpCikrrR58nAHjLWSExsofS4zAyLEEW7CtHQGZN0HAyHD14jV+iJ1y2traSiMjI9FhdnzWS3d39+xiXRY827Zti0xtDrKDgPESssgJBIw\/luhk5ViCLdjWjoDMmyBgZLhWwqse\/Mar747yfNnyRbT7ygsrkf+yZhICRi4yYAu2cgTkPKPeyrCFgJHhWgmvKvj6GTDdFy+h27ovqET+y5pJNFZykQFbsJUjIOcZ9VaGLQSMDNdKeE0SMDgDpnjo0FgVZ5jmAWzBVo6AnGfUWxm2EDAyXCvhVQVfP8SOR194FAaPOwE0Vu7s8lKCbR4h98\/B1p1dXkqwzSPk9jkEjBu3IFIlCZjd77uQLjt\/URDlq1ch0FjJkQdbsJUjIOcZ9VaGLQSMDNdKeFXBxyF2fsOFxsovT90b2IKtHAE5z6i3MmwhYGS4VsKrCj4OsfMbLjRWfnlCwMjxBFuwrQ0BmbdAwMhwrYRXFXycAeM3XBAwfnmik5XjCbZgWxsCMm+BgJHhWgmvEDAyYYKAkeHKXsEWbOUIyHlGvZVhCwEjw7USXlXwV974deKzYHCInZ+wobHywzHJC9iCrRwBOc+otzJsIWBkuFbCqwo+TuH1Gy40Vn55YppDjifYgm1tCMi8BQJGhmslvHLwv\/Kv3yIegeEHh9j5CRsEjB+OGIGR4wi2YFtbAjJvg4CR4VoJrxz8v\/nCfbTu4\/dDwHiMGASMR5gxV2ALtnIE5Dyj3sqwhYCR4VoJr3EBc+BDl9DSxlMrkfcyZxKNlVx0wBZs5QjIeUa9lWELASPDtRJeOfgf3TVOfA4MPziF10\/Y0Fj54YhpDjmOYAu2tSUg8zYIGBmulfDKwd\/wl1+mobsehYDxGDEIGI8wMYUkBxNswbZmBGReBAEjw7USXjn4v\/XRz9LYvUei\/GIKyU\/YIGD8cMQogRxHsAXb2hKQeRsEjAzXSnjl4L\/xmk\/RPY8ci9a+sIDBU5wABExxhmkewBZs5QjIeUa9lWELASPDtRJeIWBkwoTGSoYrewVbsJUjIOcZ9VaGLQSMDNdKeOXgn\/7uv8MpvJ6jhcbKM1DNHdiCrRwBOc+otzJsIWBkuFbCKwf\/2O+ORHnFNQL+QobGyh\/LuCewBVs5AnKeUW9l2ELAyHD15vXo0aPU29tLExMTkc+Ojg4aHBykhoaGxHfs27eP1q9fH33W2tpKIyMj1NjYmGi77I1vpud+Yyj6rPviJXRb9wXe8r2QHaGxkos+2IKtHAE5z6i3MmwhYGS4evF6\/PhxGhgYoPb2durs7CT1c1NTE\/X39897x+TkJPX09NCWLVuora2Ndu3aRePj46mCRxcwOAPGS8giJ2is\/LHECIwcS7AF29oRkHkTBIwMVzGvWaKEPzt48GCiuEnK0NI3\/xZ977K+6CMefeFRGDzFCUDAFGeY5gFswVaOgJxn1FsZthAwMlzFvKYJmPhojUkGmt\/2B\/SDVe+OTDECY0LMzAaNlRknFyuwdaFmlgZszTi5WIGtC7X8NBAw+YxKY6HWw3R1dUVTSvqjBMzatWvp9ttvj9bM5K2Bafrt\/0kvvH5d5Oa0e4bpq7s+XpqyVjkj09PT1NzcXOUilDbvYCsXGrAFWzkCfjyvWbNmnqOpqSk\/zkvk5RUnTpw4UaL8FM6KEijsKGkRr\/r88OHDswt3h4aGaGZmJnUNjC5gcApv4RDNOsBfW\/5Yxj2BLdjKEZDzjHorwxYjMDJcvXrNEy\/8sqQpJF7U29fXR8PDw9TS0jIvT0t+\/0b60dJLo98fvfntXvO8kJ2hsZKLPtiCrRwBOc+otzJsIWBkuHrzmrfzSH8Rj7gsW7ZsdnqJBcxNN91EW7duTdxKffYffoJefM3rcI2At2iddITGyjNQzR3Ygq0cATnPqLcybCFgZLh685o3DaS\/iM+AYXt19gv\/Pz9JW6759xAw3sI0xxEaKxmuEIdyXMEWbGUJyHiHgJHh6sVr\/BA75VQtzuXD7PicmO7u7rfKKB8AAAzESURBVOjcF370g+zyDr17zR\/\/A\/3k516DU3i9ROtlJxAwnoFiBEYOKNiCbU0IyLwEAkaGayW8Nl59d5RPXCPgN1wQMH556t7AFmzlCMh5Rr2VYQsBI8O1El6VgME1An7DhcbKL08IGDmeYAu2tSEg8xYIGBmulfCqBEz\/2vOof+2ySuS5CpmEgJGLEtiCrRwBOc+otzJsIWBkuFbCKwSMTJjQWMlwZa9gC7ZyBOQ8o97KsIWAkeFaCa9KwOAeJL\/hQmPllyemOeR4gi3Y1oaAzFsgYGS4VsKrEjC4B8lvuCBg\/PJEJyvHE2zBtjYEZN4CASPDtRJelYDBNQJ+wwUB45cnOlk5nmALtrUhIPMWCBgZrpXwCgEjEyYIGBmu7BVswVaOgJxn1FsZthAwMlwr4VUJGNyD5DdcaKz88sQogRxPsAXb2hCQeQsEjAzXSnhlAbO08VTiKSQ8\/ghAwPhjGfcEtmArR0DOM+qtDFsIGBmulfAKASMTJjRWMlwxhSTHFWzBVpaAjHcIGBmulfDKAgbXCPgPFQSMf6bKI9iCrRwBOc+otzJsIWBkuFbCKwSMTJjQWMlwxSiBHFewBVtZAjLeIWBkuFbCKwsY3IPkP1QQMP6ZYgRGjinYgq08AZk3QMDIcK2EVxYwuAfJf6ggYPwzRScrxxRswVaegMwbIGBkuFbCKwSMTJggYGS4YppDjivYgq0sARnvEDAyXCvh9ew\/\/AR97P2\/G00j4fFHAALGH8u4J7AFWzkCcp5Rb2XYQsDIcK2E11CDX2\/4aKzkIgC2YCtHQM4z6q0M21D7sFecOHHihAyycLyGGvx6RwiNlVwEwBZs5QjIeUa9lWEbah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn1VoZtqH1YMALm6NGj1NvbSxMTE1EN6OjooMHBQWpoaMisEZOTk9TX10fDw8PU0tKSaBtq8GW+KuZe0ViZs7K1BFtbYub2YGvOytYSbG2JmdmH2ocFIWCOHz9OAwMD1N7eTp2dnaR+bmpqov7+\/tQIK7v9+\/fT6OgoBIzZd8GbVahfKm+ACjgC2wLwcpKCLdjKEZDxHGqdDULAJIV8165dND4+njkKs2\/fPhoaGoqSYwRG5ouT5TXUL1XtSc5\/I9jKRQFswVaOgIznUOvsghUwPOV0\/fXXU3d3dyRiIGBkvjgQMLXnym8MtcGqD825bwVbuSiArQzbULkGKWDUepiurq5oSilthIZ\/v2rVKqM1MDLVCl5BAARAAARAQJ7A1NSU\/Etq\/IbgBIxa18Ic0xbx8sLd7du307XXXkvT09O5AqbGMcHrQAAEQAAEQAAEcggEJWBMxAvz4Cmj1atXU1tbG5nsQkItAgEQAAEQAAEQKBeBYASM6c6j+HZrPRw7duyIRA0eEAABEAABEACBchMIRsDwqMrMzIzR2S96SDACU+4KityBAAiAAAiAQBKBIARM2qhKa2srjYyMRIfZ8TkxvOMoPsICAYMvBgiAAAiAAAhUj0AQAqZ62JFjEAABEAABEACBIgQgYIrQQ1oQAAEQAAEQAIG6EICAycDO62q2bdsWWWCBr1v95Cm6np6eaH1S3v1UOm++BiLrege33ISVyoatKnn82o2wiPgpjQ3X+PQ12onsGNiw1W3RHhSr2+p7n7SMopjn+qaGgEnhr64Z4DU0Dz30ULT1mv+\/sbGxvhGr0Nv1znLdunVz7quKFyN+9QP\/vHPnTjBPibcNW90Fc920aRNt3rw59ZDHClUx71m14Rrf+Yj1dNnhsGGrhCHfZcfrFtEeuFd1xX3Pnj3B\/SEOAZNSL9QdSfwFClW9un8lzFLGG3QWhWNjY0Y7xdAZ5P8lq9+ibsKWO4VrrrmGjh07RlmnVJtFN0wrmzrLtjfddBNt3boVf9gYVAdbtnr9RntgADjBRI1iXXTRRXT48OHocuOQjgqBgEkIetrt1uq2a7eqtPBS6aNYPHIV\/zmLCBqs7PriwpZF+cUXX0yf+cxnZm9uX3i10h9XkwtjwfdlAjZ1NmkEJu9yXrCeT+Dxxx+Pfsk7cXt7eyFgFkIlSRpx4cZ\/2bJlGHa3qADxUQGbv1hdz\/WxyF6lTW3Zquszrr766ugSU4jx5PDbcGUBc\/DgwcgR1srlf51s2LI3fepjw4YNUeeLx41AXBC6eSlfKozAZIzA6AueIGDsK69tg6XewB3DLbfcgkW8Gcht2HJH8NGPfpTe9a53UXNzc+ZaJPsoh5XChqtaT6QW7nLajRs3ot6mVAkbtmrqY8uWLdGUh83obVg10k9pIGD8cKyEF0wh+QmTzZAxxIsdcxu2bPvVr341+gsWu5CyOdtwjU8hgS3Y2n2La2cNAVM71qV4kz7igkW8biGJTxnlLTTFTgNzzjZs9e3p+hswLD+ftw3XeH1GO5Fdf23YQhyatwUmlhAwJpQCssE26uLBtNk2ieF3O942bHXPGCXI5mzDNd4pYJrDH9ukKSRMz9m1Ebo1BIw7u8qmxEF2xUOXdXCVWgTJUxtpowQ4GCw9BqZsIWDs6rENV\/0gOxy2ls\/Zhi0LwvXr10dOwTafbZYFBEwxfkgNAiAAAiAAAiAAAt4IYBeSN5RwBAIgAAIgAAIgUCsCEDC1Io33gAAIgAAIgAAIeCMAAeMNJRyBAAiAAAiAAAjUigAETK1I4z0gAAIgAAIgAALeCEDAeEMJRyAAAiAAAiAAArUiAAFTK9J4DwiAAAiAAAiAgDcCEDDeUMIRCPgn8Mgjj9DixYuJb\/M2efi8h2effZaWL19uYu5ko87saW1tpZGREeO8VeWGcVW+Wp09Uuv3OQUdiUCghAQgYEoYFGQJBJiA7cmutTisqogIKZK2ljWCBQU\/tbz9uCpsahkHvAsE8ghAwOQRwucgUCcCZRQwtnnS0VWlk4aAqVOFx2tBwJIABIwlMJiDgE8C+lH07FdNyzz00EOzx6jz79WVCvErF9Q0x5lnnkm9vb00MTERZU9d1Kju9tmzZ0\/0e5NpH\/0d+jQKX\/2wadOm2eJv3ryZOjs75+FIs1MCZt26dXTDDTdE6eLTNPrx8cqxeg+zuuaaa+itb31rlF6V5ZlnnqGenh6amZmJknz4wx+m3bt30\/DwMLW0tES\/SytTUizjAoZ\/fv7556P\/FMesizDjFxHyO5J+V0Vx57PuwxcIFCUAAVOUINKDgCOBpIsV9c4zPtqRdkMvv35wcJDYH4sYnvpoa2sjJY66urpmhUbWjd8qP8pfQ0ND1PHecsstNDo6GomBvBGYuL1+KR+LLBYaF110UZRf5X\/nzp3RWhoWIn19fXOEh+5PibSlS5fOpo+XUf381FNPRXlubm6mgYGBSCipKaG8i0OTBMy2bdtIF1LMWeeqVwEIGMcvBJKBgCUBCBhLYDAHAV8EkgSG7jtPLMT\/so8LGE4\/NjY229mzfdZt1ElTPHH7rDzl3XQdv2GY85M3raR\/rgRMXJCNj4\/PKaMuUPgdN910E23dunXOYuOsaaIkAcOjO0p0sc8sDhAwvr4h8AMC2QQgYFBDQKCOBPTplvj0TlonGZ9m6ejoSByBiU\/l6MVMmv5Je59+a3hWx523iDhJrCT9Lj6tFp8mUyNMXJ4kIaL75FEddaNxPMxp00BJAobT6ot6s4QXBEwdv1B49YIiAAGzoMKNwpaVgN5p6+tguDNVW5WVIImvS1EjEPERGE4bHznIKn+aOMma1tL9FRUw7EutZVECK2kExkbA3HfffaSmqEy3okPAlPVbgnyBwFwCEDCoESBQIgK6CFAjDCxgeL0Ir+Vob2+fs3BWFylxAZO13iWpyLWYQoqvcdHfyWIjazpITSHpAiZptEOfQuIRmI0bN86u4TEJtcQUUp6YzJtKM8k3bEBgoRGAgFloEUd5S0MgaQ2MPgqiL2pNWoyqRmTUFBIXTBc5yj8v6FXTH0nrUBQQX4t49REPfV3MqlWr5i3SjQsYPa3KK+ePF+QmCRjTRbzsQ61hyVt7VHQRr5riUzvHVDn0xcvxSggBU5qvJTJSIQIQMBUKFrIaHgHVuaktwPr0kL4FmqdULr\/88jlbpVm4XHHFFXTdddfNjjDERY0alVHbq5mg6ljTaGZtOTZdWJy03dpkDUz83Vu2bInWufDCXVV+fQSGyxBnGN9GHd9KzmnStoCrUS\/+V4m+pG3UevqkcunrjzhOK1eupAMHDkQiKi40VRnio1Ph1XaUCAT8EoCA8csT3kAABOpMgAVF0s4j02yZrIEx9WVqhxEYU1KwA4GXCUDAoDaAAAhUlkB8nY8abdHPfbEtHASMLTHYg0B9CEDA1Ic73goCIOCJQPx04qxTck1eGb9c8Y477oiSSd2NhMscTaICGxCYT+D\/A1VDQIPxAPRSAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:74597843]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:88e088c4]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:18ca9ceb]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
