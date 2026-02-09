%[text] ## Settings for simulink model initialization and data analysis
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 2;
transmission_delay = 125e-6*2;
% model = 'psm_sv_bemf_ctrl';
% model = 'psm_sv_ekf_bemf_ctrl';
model = 'psm_mpc_bemf_ctrl';

load_step_time = 0;
%[text] #### local time allignment to master time
kp_align = 0.6;
ki_align = 0.1;
lim_up_align = 0.2;
lim_down_align = -0.2;
%[text] ### Enable one/two modules
number_of_modules = 1;
enable_two_modules = number_of_modules;
%[text] ### MOTOR Selection from Library
n_sys = 6;
run('n_sys_generic_1M5W_pmsm'); %[output:98950895] %[output:9dd00217]
run('n_sys_generic_1M5W_torque_curve');

% n_sys = 1;
% run('testroom_eq_psm_690V');
% run('testroom_torque_curve_690V');

b = tau_bez/omega_m_bez;
external_motor_inertia = 5*Jm;
% external_motor_inertia = 1;

%% inverter filter
% LFi = 40e-6;
% RLFi = 5e-3
%% inverter filter
LFi = 230e-6;
LFi_0 = 20e-6;
RLFi = 5e-3;
%[text] ### Settings for speed control or wind application
use_torque_curve = 0; % for wind application
use_speed_control = 1-use_torque_curve; %
use_mtpa = 1; %
use_psm_encoder = 0; % 
use_load_estimator = 0; %
use_estimator_from_mb = 0; %mb model based
use_motor_speed_control_mode = 0; 
use_advanced_pll = 0; % advanced pll should compensate second harmonic
use_dq_pll_ccaller_mod1 = 0; % only module 1
use_dq_pll_ccaller_mod2 = 0; % only module 1
%[text] ### Settings for CCcaller versus Simulink
use_observer_from_simulink_module_1 = 0;
use_observer_from_ccaller_module_1 = 1;
use_observer_from_simulink_module_2 = 1;
use_observer_from_ccaller_module_2 = 0;

use_current_controller_from_simulink_module_1 = 0;
use_current_controller_from_ccaller_module_1 = 1;
use_current_controller_from_simulink_module_2 = 1;
use_current_controller_from_ccaller_module_2 = 0;

use_moving_average_from_ccaller_mod1 = 0;
use_moving_average_from_ccaller_mod2 = 0;
mavarage_filter_frequency_base_order = 2; % 2 means 100Hz, 1 means 50Hz
dmavg_filter_enable_time = 0.025;
%%
%[text] ## Grid Emulator Settings
grid_emulator;
%%
%[text] ## AFE Settings and Initialization
%[text] ### Switching frequencies, sampling time and deadtime
fPWM_AFE = 6*2.5e3; % in case of mpc controller run 6 times the maximum pwm
% fPWM_AFE = 2*2.5e3; % in case of sv controller run 2 times the maximum pwm
tPWM_AFE = 1/fPWM_AFE;

dead_time_AFE = 3e-6;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;

ts_afe = 1/fPWM_AFE; % Sampling time of the control AFE as well as INVERTER
tc = ts_afe/100;

s=tf('s');
z_afe=tf('z',ts_afe);

% t_misura = simlength - 0.025;
t_misura = 1/omega_bez*2*pi*5;
Nc = ceil(t_misura/tc);
Ns_afe = ceil(t_misura/ts_afe);

time_gain_afe_module_1 = 1.0;
time_gain_inv_module_1 = 1.0;
time_gain_afe_module_2 = 1.0;
time_gain_inv_module_2 = 1.0;
wnp = 0;
white_noise_power_afe_mod1 = wnp;
white_noise_power_inv_mod1 = wnp;
white_noise_power_afe_mod2 = wnp;
white_noise_power_inv_mod2 = wnp;

trgo_th_generator = 0.025;

afe_pwm_phase_shift_mod1 = 0;
white_noise_power_afe_pwm_phase_shift_mod1 = 0.0;
inv_pwm_phase_shift_mod1 = 0;
white_noise_power_inv_pwm_phase_shift_mod1 = 0.0;

afe_pwm_phase_shift_mod2 = 0;
white_noise_power_afe_pwm_phase_shift_mod2 = 0.0;
inv_pwm_phase_shift_mod2 = 0;
white_noise_power_inv_pwm_phase_shift_mod2 = 0.0;
%[text] ### FRT Settings
enable_frt_1 = 0;
enable_frt_2 = 1-enable_frt_1;

% deep data for frt type 2
deepPOSxi = 0.5 %[output:29079460]
deepNEGxi = 0 %[output:05701a1c]
deepNEGeta = 0.5 %[output:7e5df9b4]
%[text] #### 
%[text] #### FRT, and other fault timing settings
test_index    = 25;
test_subindex = 4;

asymmetric_error_type = 0;  
% 0 -> Variant C, two phase, 
% 1 -> Variant D, single phase

start_time_grid_switch_open = 1e3;
start_time_LVRT = 2.0;
time_start_motor_control = 0.035;

time_aux_power_supply_fault = 1e3;
time_phase_fault = 1e3;
start_load = 0.25;

%[text] #### FRT gain factor for grid support
settle_time = 0.175;
k_frt_ref = 2;
%[text] #### Reactive current limits for grid support

i_grid_pos_eta_lim = 1;
i_grid_neg_xi_lim = 0.5;
i_grid_neg_eta_lim = 0.5;

%[text] #### Reactive current Limits - Red. Dyn. grid support
i_grid_pos_eta_red_lim = 0.1;
i_grid_neg_eta_red_lim = 0.1;
i_grid_neg_xi_red_lim = 0.1;

%[text] #### Grid voltage derivate implemented with double integrator observer
Aso = [0 1; 0 0];
Asod = eye(2)+Aso*ts_afe;
Cso = [1 0];
omega_rso = 2*pi*50;
p2place = [-1 -4]*omega_rso;
p2placed = exp(p2place*ts_afe);
Kd = (acker(Asod',Cso',p2placed))';
l1 = Kd(2) %[output:8bf67124]
l2 = Kd(1) %[output:45e82fc7]

%[text] #### Grid fault generator 
grid_fault_generator;
%[text] ### Current sensor endscale, and quantization
adc_quantization = 1/2^11;
Imax_adc = 1049.835;
CurrentQuantization = Imax_adc/2^11;
%%
%[text] ### Voltage sensor endscale, and quantization
Umax_adc = 1500;
VoltageQuantization = Umax_adc/2^11;
%%
%[text] ### DClink, and dclink-brake parameters
Vdc_ref = 1070; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi = 900e-6*8;

%[text] #### 
%[text] #### DClink Lstray model
Lstray_dclink = 100e-9;
RLstray_dclink = 10e-3;
C_HF_Lstray_dclink = 15e-6;
R_HF_Lstray_dclink = 22000;
Z_HF_Lstray_dclink = 1/s/C_HF_Lstray_dclink + R_HF_Lstray_dclink;
Z_LF_Lstray_dclink = s*Lstray_dclink + RLstray_dclink;
Z_Lstray_dclink = Z_HF_Lstray_dclink*Z_LF_Lstray_dclink/(Z_HF_Lstray_dclink+Z_LF_Lstray_dclink);
ZCFi = 7/s/CFi;
sys_dclink = minreal(ZCFi/(ZCFi+Z_Lstray_dclink));
% figure; bode(sys_dclink,Z_Lstray_dclink,options); grid on
%[text] ### LCL switching filter
LFu1_AFE = 0.5e-3;
RLFu1_AFE = 157*0.05*LFu1_AFE;
LFu1_AFE_0 = LFu1_AFE;
RLFu1_AFE_0 = RLFu1_AFE/3;
CFu = (100e-6*2);
RCFu = (50e-3);
%%
%[text] ### DClink voltage control parameters
Vdc_nom = Vdc_bez;
Vdc_norm_ref = Vdc_ref/Vdc_nom;
kp_vs = 0.85;
ki_vs = 35;
%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
kp_afe = 0.6;
ki_afe = 45;
delta = 0.015;
res = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);

Ares = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Bres = [0; 1];
Cres = [0 1];
Aresd = eye(2) + Ares*ts_afe;
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
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso = [-1 -4]*omega_rso;
Lrso = acker(Arso',Crso',polesrso)';
Adrso = eye(2) + Arso*ts_afe;
polesdrso = exp(ts_afe*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:3873a036]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:2f8df18e]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:45ef9943]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:20f05bf9]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:0445503e]
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;
kp_rc_pos_grid = kp_rc_grid;
ki_rc_pos_grid = ki_rc_grid;
kp_rc_neg_grid = kp_rc_grid;
ki_rc_neg_grid = ki_rc_grid;
%%
%[text] ### Settings for First Order Low Pass Filters
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
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application

%[text] ### Switching frequencies, sampling time and deadtime
fPWM_INV = fPWM_AFE;
% fPWM_INV = 2500;
dead_time_INV = 3e-6;
delayINV_modA = 0;
pwm_out_lim = 1;

ts_inv = 1/fPWM_INV;
t_measure = simlength;
Ns_inv = floor(t_measure/ts_inv);
s=tf('s');
z=tf('z',ts_inv);

%[text] ### Simulation parameters: speed reference, load torque in motor mode
% rpm_sim = 3000;
rpm_sim = 17.8;
% rpm_sim = 15.2;
omega_m_sim = omega_m_bez;
omega_sim = omega_m_sim*number_poles/2;
tau_load_sim = tau_bez/5; %N*m
b_square = 0;

%[text] ### Luenberger Observer
Aso = [1 ts_inv; 0 1];
Cso = [1 0];
% p2place = exp([-10 -50]*ts_inv);
p2place = exp([-40 -160]*ts_inv);
Kobs = (acker(Aso',Cso',p2place))';
kg = Kobs(1) %[output:04a70309]
kw = Kobs(2) %[output:69e958f5]

%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:8e3d2c7e]
luenberger_l2 = Klo(2) %[output:95bd65ef]
luenberger_l3 = Klo(3) %[output:7d37a44a]
omega_flt_fcut = 10;
% phase_compensation_omega = -pi/2-pi/12; % for motor mode
phase_compensation_omega = 0; % for generator mode
%[text] ### Control settings
id_lim = 0.35;
%[text] #### rotor speed control
kp_w = 2;
ki_w = 8;
iq_lim = 1.4;
%[text] #### current control
kp_i = 0.25;
ki_i = 18;
kp_id = kp_i;
ki_id = ki_i;
kp_iq = kp_i;
ki_iq = ki_i;
CTRPIFF_CLIP_RELEASE = 0.001;
%[text] #### Model Predictive Control 
kp_i = 0.25;
ki_i = 18;
%[text] #### 
%[text] #### Field Weakening Control 
k_kalman = 0;
%[text] #### BEMF observer
emf_fb_p = 0.2;
emf_p = emf_fb_p*4/10;

emf_fb_p_ccaller_1 = emf_fb_p;
emf_p_ccaller_1 = emf_fb_p_ccaller_1*4/10;

emf_fb_p_ccaller_2 = emf_fb_p;
emf_p_ccaller_2 = emf_fb_p_ccaller_2*4/10;

% omega_th = 0.25;
omega_th = 0;
%[text] #### Speed obserfer filter LPF 10Hz
fcut_10Hz_flt = 10;
omega_flt_g0 = fcut_10Hz_flt * ts_inv * 2*pi;
omega_flt_g1 = 1 - omega_flt_g0;
%[text] #### Motor Voltage to Udc Scaling
motorc_m_scale = 2/3*Vdc_bez/ubez;
inv_m_scale = motorc_m_scale;
%%
%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] ### HeatSink settings
heatsink_liquid_2kW;
%[text] ### DEVICES settings (IGBT)
igbt.data = 'infineon_FF1200R17IP5';
run(igbt.data);

igbt.inv.Vth = Vth;                                  % [V]
igbt.inv.Vce_sat = Vce_sat;                          % [V]
igbt.inv.Rce_on = Rce_on;                            % [Ohm]
igbt.inv.Vdon_diode = Vdon_diode;                    % [V]
igbt.inv.Rdon_diode = Rdon_diode;                    % [Ohm]
igbt.inv.Eon = Eon;                                  % [J] @ Tj = 125°C
igbt.inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
igbt.inv.Erec = Erec;                                % [J] @ Tj = 125°C
igbt.inv.Voff_sw_losses = Voff_sw_losses;            % [V]
igbt.inv.Ion_sw_losses = Ion_sw_losses;              % [A]
igbt.inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
igbt.inv.Rtim = Rtim;                                % [K/W]
igbt.inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
igbt.inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
igbt.inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
igbt.inv.Rth_diode_JC = Rth_diode_JC;              % [K/W]
igbt.inv.Rth_diode_CH = Rth_diode_CH;              % [K/W]
igbt.inv.Rth_diode_JH = Rth_diode_JH;              % [K/W]
igbt.inv.Lstray_module = Lstray_module;            % [H]
igbt.inv.Irr = Irr;                                % [A]
igbt.inv.Cies = Cies;                              % [F]
igbt.inv.Csnubber = Csnubber;                      % [F]
igbt.inv.Rsnubber = Rsnubber;                      % [Ohm]
% inv.Csnubber = (inv.Irr)^2*Lstray_module/Vdc_bez^2
% inv.Rsnubber = 1/(inv.Csnubber*fPWM_INV)/5

igbt.afe.Vth = Vth;                                  % [V]
igbt.afe.Vce_sat = Vce_sat;                          % [V]
igbt.afe.Rce_on = Rce_on;                            % [Ohm]
igbt.afe.Vdon_diode = Vdon_diode;                    % [V]
igbt.afe.Rdon_diode = Rdon_diode;                    % [Ohm]
igbt.afe.Eon = Eon;                                  % [J] @ Tj = 125°C
igbt.afe.Eoff = Eoff;                                % [J] @ Tj = 125°C
igbt.afe.Erec = Erec;                                % [J] @ Tj = 125°C
igbt.afe.Voff_sw_losses = Voff_sw_losses;            % [V]
igbt.afe.Ion_sw_losses = Ion_sw_losses;              % [A]
igbt.afe.JunctionTermalMass = JunctionTermalMass;    % [J/K]
igbt.afe.Rtim = Rtim;                                % [K/W]
igbt.afe.Rth_switch_JC = Rth_switch_JC;              % [K/W]
igbt.afe.Rth_switch_CH = Rth_switch_CH;              % [K/W]
igbt.afe.Rth_switch_JH = Rth_switch_JH;              % [K/W]
igbt.afe.Lstray_module = Lstray_module;              % [H]
igbt.afe.Irr = Irr;                                  % [A]
igbt.afe.Cies = Cies;                                % [F]
igbt.afe.Csnubber = Csnubber;                        % [F]
igbt.afe.Rsnubber = Rsnubber;                        % [Ohm]
% afe.Csnubber = (afe.Irr)^2*Lstray_module/Vdc_bez^2
% afe.Rsnubber = 1/(afe.Csnubber*fPWM_AFE)/5

%[text] ### DEVICES settings (MOSFET)
mosfet.data = 'danfoss_SKM1700MB20R4S2I4' %[output:9ef89060]
run(mosfet.data);

mosfet.inv.Vth = Vth;                                  % [V]
mosfet.inv.Rds_on = Rds_on;                            % [V]
mosfet.inv.Vdon_diode = Vdon_diode;                    % [V]
mosfet.inv.Rdon_diode = Rdon_diode;                    % [Ohm]
mosfet.inv.Eon = Eon/3*2/3;                                % [J] @ Tj = 125°C
mosfet.inv.Eoff = Eoff/3*2/3;                              % [J] @ Tj = 125°C
mosfet.inv.Erec = Eerr;                                % [J] @ Tj = 125°C
mosfet.inv.Voff_sw_losses = Voff_sw_losses*2/3;            % [V]
mosfet.inv.Ion_sw_losses = Ion_sw_losses/3;              % [A]
mosfet.inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
mosfet.inv.Rtim = Rtim;                                % [K/W]
mosfet.inv.Rth_switch_JC = Rth_mosfet_JC;              % [K/W]
mosfet.inv.Rth_switch_CH = Rth_mosfet_CH;              % [K/W]
mosfet.inv.Rth_switch_JH = Rth_mosfet_JH;              % [K/W]
mosfet.inv.Lstray_module = Lstray_module;              % [H]
mosfet.inv.Irr = Irr;                                  % [A]
mosfet.inv.Csnubber = Csnubber;                        % [F]
mosfet.inv.Rsnubber = Rsnubber;                        % [Ohm]
% inv.Csnubber = (mosfet.inv.Irr)^2*Lstray_module/Vdc_bez^2
% inv.Rsnubber = 1/(mosfet.inv.Csnubber*fPWM_INV)/5

mosfet.afe.Vth = Vth;                                  % [V]
mosfet.afe.Rds_on = Rds_on;                            % [V]
mosfet.afe.Vdon_diode = Vdon_diode;                    % [V]
mosfet.afe.Rdon_diode = Rdon_diode;                    % [Ohm]
mosfet.afe.Eon = Eon/3*2/3;                                % [J] @ Tj = 125°C
mosfet.afe.Eoff = Eoff/3*2/3;                              % [J] @ Tj = 125°C
mosfet.afe.Erec = Eerr;                                % [J] @ Tj = 125°C
mosfet.afe.Voff_sw_losses = Voff_sw_losses*2/3;            % [V]
mosfet.afe.Ion_sw_losses = Ion_sw_losses/3;              % [A]
mosfet.afe.JunctionTermalMass = JunctionTermalMass;    % [J/K]
mosfet.afe.Rtim = Rtim;                                % [K/W]
mosfet.afe.Rth_switch_JC = Rth_mosfet_JC;              % [K/W]
mosfet.afe.Rth_switch_CH = Rth_mosfet_CH;              % [K/W]
mosfet.afe.Rth_switch_JH = Rth_mosfet_JH;              % [K/W]
mosfet.afe.Lstray_module = Lstray_module;              % [H]
mosfet.afe.Irr = Irr;                                  % [A]
mosfet.afe.Csnubber = Csnubber;                        % [F]
mosfet.afe.Rsnubber = Rsnubber;                        % [Ohm]
% afe.Csnubber = (mosfet.afe.Irr)^2*Lstray_module/Vdc_bez^2
% afe.Rsnubber = 1/(mosfet.afe.Csnubber*fPWM_AFE)/5
%[text] ### DEVICES settings (Ideal switch)
silicon_high_power_ideal_switch;
ideal_switch.Vth = Vth;                                  % [V]
ideal_switch.Rds_on = Rds_on;                            % [Ohm]
ideal_switch.Vdon_diode = Vdon_diode;                    % [V]
ideal_switch.Vgamma = Vgamma;                            % [V]
ideal_switch.Rdon_diode = Rdon_diode;                    % [Ohm]
ideal_switch.Csnubber = Csnubber;                        % [F]
ideal_switch.Rsnubber = Rsnubber;                        % [Ohm]
ideal_switch.Irr = Irr;                                  % [A]
% ideal_switch.Csnubber = (ideal_switch.Irr)^2*Lstray_module/Vdab2_dc_nom^2
% ideal_switch.Rsnubber = 1/(ideal_switch.Csnubber*fPWM_DAB)/5
%[text] ### GATE drivers settings
positive_voltage_rail = 12;
negative_voltage_rail = 0;
dead_time = 3e-6;
use_deadtime = 1;  
use_deadtime_cmos_based = use_deadtime;
%[text] ### Lithium Ion Battery
ubattery = Vdc_nom;
Pnom = 250e3;

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
figure;  %[output:339abfd4]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:339abfd4]
xlabel('state of charge [p.u.]'); %[output:339abfd4]
ylabel('open circuit voltage [V]'); %[output:339abfd4]
title('open circuit voltage(state of charge)'); %[output:339abfd4]
grid on %[output:339abfd4]
%[text] ## C-Caller Settings
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

%[text] ## 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":35.4}
%---
%[output:98950895]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:9dd00217]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:29079460]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepPOSxi","value":"   0.500000000000000"}}
%---
%[output:05701a1c]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGxi","value":"     0"}}
%---
%[output:7e5df9b4]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGeta","value":"   0.500000000000000"}}
%---
%[output:8bf67124]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  24.984105426951174"}}
%---
%[output:45e82fc7]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.101088737746315"}}
%---
%[output:3873a036]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.101088737746315"],["24.984105426951174"]]}}
%---
%[output:2f8df18e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:45ef9943]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:20f05bf9]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000066666666667"],["-6.579736267392907","0.998952802448803"]]}}
%---
%[output:0445503e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.103672557568463"],["18.110724075998974"]]}}
%---
%[output:04a70309]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.013273093780640"}}
%---
%[output:69e958f5]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.423833817530528"}}
%---
%[output:8e3d2c7e]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.237172904101351"}}
%---
%[output:95bd65ef]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     1.436150968933706e+02"}}
%---
%[output:7d37a44a]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -1.779725838133451e+02"}}
%---
%[output:9ef89060]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"mosfet","value":"    data: 'danfoss_SKM1700MB20R4S2I4'"}}
%---
%[output:339abfd4]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAW8AAADdCAYAAAB0SfPeAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ1wV9WVP7i0S7SlGMRqNoUgBnX7gdbWhhQHHeoy7hrsdNeBZGeXDY6lU6S7bVhIYrddWiGBSpcvtZk2zdiZJWBHHMN0OtSN1Y4i1hVl68cKEmKMUYtBBLe4M3bZPc+9\/97cvI\/73rtf7\/7Pm3HU\/O8995zfOff3zjvvvnsnnDlz5gzQRQgQAoQAIVAoBCYQeRfKX6QsIUAIEAIBAkTeFAiEACFACBQQASLvAjpNVuXDhw9Dc3MzNDQ0wJo1a2S7Sbc7fvw43HzzzTB9+nTo7OyEiooK6b5ZGm7YsAH27NkDPT09UFtbm0VE5j6nT5+G1tZWqKqq0oJlZsUkOu7fvx+ampqClsuXL0+lv03MRdNYvC1ZsgQWL14sYbnfTYi8PfavDfLGMffu3Qu33nqrcmRFItE5lqg8I8AdO3ZAXV1dom1pddu1axfMmDFDSnbi4FwDdtMZGhqC7u5uqKysTNMdXCJvVBz1QV9ksSWV4QVoTORdACcVRUXdNwueSBATnU8VIuZpSCMtDkjcbW1tIHtjSBMPvpF32ptoGqyK1pbIO6PHcDJ3dXUFvfFRmn+UZyRzzTXXBBMSr46OjjGPenx\/LGuwsgOb+Nj31KlTQZlAlC+qzCY\/+zsjAZFEWDt8dEbd2SM0azcyMjLm0Vosi+CPWDpgWRz+PyubrF69Osi2Dx48GMiYM2fOmOyIkQj+JtrKl3VkcN2yZQvcfvvt48Zi+oTpwMZHPPFiGPB+4csLPOYMB8y4WfmJ\/U0cK0qHqL8fOnSoVNJgeuEYUbqEhauoC4sn5i9mM\/5\/2A0iqj+WwVgsT5s2rYQ3j5mIK48bi6srr7wyiBm8MGPmbZ4\/f37w9xMnTpTiRYxHXue0N8aM07sQ3Yi8M7hJfJQUMydGQCzIw35ntdupU6cGBMiIgQUnThYM9NHR0dgMM042msZnpzx5MxISJwMjDdT9uuuuG1PTjiNvJOTh4eFUuqI+27ZtK934ZHBluIm2iTcHURceJ7yx4E0IZTEf8XZjPZXPtJkPVq5cWboBh\/3ObkIipml0wziI00UseyTdYJGA+RtuUn8RNzGWRR\/xOPA3cz4eWCzj2KK+ePPDejy72YvxLsaI6fcsGejBWBci75RQx2VhjIDDarOMRG+55ZZxL\/niiCAuWJMeiaMybz6TiXtkTyKGqMka9YKU1+drX\/taQCosE0db+JsY\/l3EWqZsImaRmGGzsfi6bxhB8i9D+cdz1AUJhs84+ScEMZuNyg7DdMObaNwNGF\/MxpUKwn7j\/8ZuVFE1bxEHcTok3VCxvZh9s8w\/7GYujifG8IMPPjimhMSwZDfOpJhPOZ0L3ZzIO6X7wiamSHJbt24dsyqC\/10sL7Dh2eOmmFHGkXdSFiJD3nEvpFSTN9rGblRIWqtWrQI2KdPiGpV5s2z605\/+dOkpIOyGGUberAzGhwQSNr5IFMlbfLTHPozcozLvMN2iyDtKF3GVRdjNl7dt0aJFsZl3Ur09ibzZTQxvkiLOYeQtjhdF3uK0ZCU+Iu8\/IEPknZK8dWTevArixPcp80Y7GbFcffXVcOTIkVLJJC2uInmLuIVl+Wkyb94nSdkpI6SoG3CcbjKZd1yI2sy8Z8+ePeYpkj09saWjKjJv0XYibyLvlJQ9tnmabCRpcrKad1TAJ2XXYibD1whFggvLsqKIBTNiMWtj9UhWwxTLJmGlDxFovnQgrjmWwTXpXQG+HMN6Kz798C9ls9S8xfp63KN7WO1XfI8RpZtIwEklHR7TpKejtDVv0YdxPmHkjfrg+xlW8ogrm2SpefMrcZLmQ66JXbDOlHlndJjMqgis4X73u98NRohbbcKvzEiTeTPV0642iarRiqtN+EwZ\/xtLB7gCJmy1CVtBwnCJWyHD2oStfJDBla3sEcc6cOBAUC\/Fi61imDx5ckDmeLGXlKgb803UahNsz\/QLeyqIW2WBfdPoxggTX94x4mMv8piP45YRxq0WkclUZVabMMzFBIBfFYNxjEkIi4+ol+18HzGm8KWmWJLifUSrTRzJvDHA8Qr7+k90oLj0LCPnGukWV0c2ogANkohA2vXCfGad9kOXRGXKuEHYEtI4ONL6zWdorWXezAlRn+vi7729vUY+u1btYCJv1Yjmlxe2fJNfppg0AsYjvmC18Wl+km5F+l1cCou6i6uM4uyhm6jlzBsf09auXRtoEbVXBD6eDQ4OptqHwZUgJvJ2xRN\/0EMsDfBlERlti7y3iYx9JtuIZT7+I7U4PZgPaW+T91GyknkjudXU1ATkjFdY2YSvfaadaCYDkcYiBAgBQsAGAsbJGx+b7rnnHrjtttuCFQFh5M2ynPr6+uCLNnpktREaNCYhQAi4jIBR8kZSXrduHSxdujTY0jPuhSUPmkjm\/G8XXXSRy\/iSboQAIeAJAv39\/TBz5kxnrDFK3mFfpSESSXsMM\/JubGwct2UmkvfAwIAzgKpWxHf7EC+yUXXU2JHnux9ds88oeYshFZV544uJlpYWaG9vDzJ0LJtg27A9fF0DVPW08d0+Im\/VEWNPnu+x6pp9zpC3SNh8lh73wYdrgKqeOkePHnXqUU21fSiPbNSBqnmZvvvRNa6xSt4qwss1QFXYxMvwfUIQeauOGHvyfI9V17iGyNterEuN7PuEIPKWCoNCNPI9Vom8FYeha4AqNo9KCqoBtSTPd2Irh5uwa1xDmbelySw7LE16WaTcbkd+dNs\/MtoRecuglKKNa4CmUF2qKU16KZicb0R+dN5FiQq6xjWUeSe6zG4DmvR28Vc1OvlRFZL25BB5K8beNUAVm0c1b9WAWpJH5G0JeIXDusY1lHkrdK4OUTTpdaBqXib50Tzmqkck8laMqGuAKjaPMm\/VgFqSR+RtCXiFw7rGNZR5K3SuDlE06XWgal4m+dE85qpHJPJWjKhrgCo2jzJv1YBakkfkbQl4hcO6xjWUeSt0rg5RNOl1oGpeJvnRPOaqRyTyVoyoa4AqNo8yb9WAWpJH5G0JeIXDusY1lHkrdK4OUTTpdaBqXib50TzmqkcsLHmHHeCaBM6cOXPg\/vvvT2qW63fXAM1lTEhnmvSqEbUjj\/xoB3eVo7rGNdKZt7jfdhIouB\/3+vXroaenJ6lprt9dAzSXMUTequFzRh6RtzOuyKyIa1wjTd6ZLdbc0TVAVZtLk141onbkkR\/t4K5yVNe4Rpq8Wdlk+vTp0NnZCRUVFSpxySzLNUAzGxLRkSa9akTtyCM\/2sFd5aiucY00eSMIu3btgra2thIeHR0dsHjxYpX4pJblGqCpDUjoQJNeNaJ25JEf7eCuclTXuCYVefNA4IHAXV1dwZ\/wxWTY4cAqgYuS5Rqgqm2mSa8aUTvyyI92cFc5qmtck5m8GSjiKpTly5fDmjVrVGIWK8s1QFUbTpNeNaJ25JEf7eCuclTXuCY3efPgsBUmmzZtgsrKSpW4RcpyDVDVRtOkV42oHXnkRzu4qxzVNa7JTd5i5t3Q0GD0haZrgKoMFpRFk141onbkkR\/t4K5q1EdfOgFf+s5O+O1PvqJKZG45mcgbM+zm5mYYGRkJFKiqqgrWc9fW1uZWKK0AIu+0iLnXnojNPZ9k0chnP\/Y++Tqs6H0Bjn\/\/2izQaOkjTd5hX1ju2LED6urqtCgmK5TIWxYpd9v5POkZ6mSju\/Eno5kX5I1kbfKFZBKwRN5JCLn\/OxGb+z6S0dBnPxaevFtaWqC9vV2qPEKfx8uEe3IbnycEZaXJ\/i9SC59jdcPeQdiw96g\/ZZOkwKKNqZIQSv7d5wlB5J3s\/yK18DlWC03ergYRlU1c9Yy8Xj5PerpByceByy2JvDV4h8hbA6iGRRJ5GwZc03A++5HIW0PQEHlrANWwSJ8nPWXehoNJ03BE3hqAJfLWAKphkUTehgHXNJzPfiTyzhg0uAkWXmFLFIm8M4LqUDefJz1l3g4FWg5V8AMdXC5YyI90ctidq+v+\/fuhqakJoja8IvLOBa8TnYm8nXBDbiV89uOq+w7Bjx97lchbNkrwq861a9cGzfETfMq8ZZErVjufJz1l3sWKxShtF935NDx65IQf5H369GlobW2FPXv2AG5GtWzZMti8eTOo3FEQyyU1NTUwODhIZRM\/5kCoFUTefjjXZz96Q96MuOvr62HGjBnQ29sb7CTY19cH+\/btU7KrIH6hec8998Btt90GW7duJfL2Y34TeZMfC4nA5bc\/DkPH3y1+5s2fJD86Oloi7+Hh4eDE+LzZN94c1q1bB0uXLg0+xU96YYnR0N\/fX8igSFIaMa2urk5qVujfycZCu6+kvK9+XLBgAZz4YndgpxcvLJFQcUvYG2+8ER544IGgbLJixYqghJJ34ypxy1kWHWEvLemFZfEnvs+P21TzLn584l7ei+562h\/yRkvYShDmHl0HEidl3gMDA8WPkAgLiNj8cC35sbh+ZGu8vcm8TbqCyHumSbiNj0XEZhxyLQP66Eesc2PWjf8+63dvwps\/uEkLdlmESh\/GkEW4iT5UNjGBst4xfJz0ImJko94Y0iUdSRtfVuJV8ey98Oov7tY1VGq5mcg77FQdcWRTR6MReaf2uXMdiNicc0kmhXzzI591T6+cBCd2\/j0MPvvrTNjo6JSJvFERtgZ78eLFJb127doVrMnGF5b437hscMuWLTr0Lskk8tYKrxHhvk36MNDIRiOhpGwQJO5be18IPszB6wd\/\/afQetNccOn9Wiby5pcK8ocOs9NzcKkgLiHEZYN4MLHOi8hbJ7pmZBOxmcFZ9yi++BGJ+6dPvQHrfv7+QgjMup\/55lxwjWsykTfLvLu6uoAdQizuQUKZt5qp4suEiEODbFQTK7al+OBHvlTCiLvvq1cEBO4NeaNh\/HpsvsbNZ+CVlZVaY8o1QFUb68OESMKEbExCqBi\/F92P\/MtJkbjx\/13jmsyZtyvh5BqgqnEp+oSQwYNslEHJ\/TZF9CMSdlDf3vlC8G92YabNMm72N9e4hsjb8TlRxAmRFlKyMS1ibrYvmh9fPv4u3Pj\/a7h50t6+5DKYd\/GUcSB7Q97i15XMUjwxvru7G3SXS1y9G6qeVkWbEFnsJxuzoOZenyL4ET9137j3aGkVCU\/a25ZcCjMqK4L6dtjlBXmzdd4rV66Ehx56KNhACjdPwi1icadBfvmg7hBzDVDV9hZhQuS1mWzMi6Ab\/V31o7jsj0cLifq+5ZfDrGkViSC6xjWZyib8UsHdu3cHe24jYZt8UUmZd2KsFaaBq5NeJYBko0o042UhWW99aAh+vO\/V0IZI2NPPnQTbGy+LzLK9zbzZft64wmT+\/PnBMWU\/+tGPgt0Fh4aGqGyiME5p0isE06Io8qM+8JGsN+w9GpwxGXUxwl69cGZoPVtGOy8yb2bo9u3bYeHChcEHOUjguB0sHspQUZH8CCIDlkwb1wCV0TlNG5r0adByty35UY1vkKh79r0KT718clzdWhwBCft7fzkbLvnoOaky7ChNXeOaTGUTNW5QI8U1QNVY9QcpNOlVI2pHHvkxHe74YhHJFzPqV46\/m0jUKB3bf+svZsFnZkxWQtaixq5xDZF3upgy3pomvXHItQxIfgyHla2tvvPhV+CF196RImlG1J+fNQXWLJyphajDtPWCvGX2NqGlgmo4gCa9GhxtSylnPzKCfvTIW7Dz16\/D0Fvvfxgje2FGfc3sSvjGF2YEXaKW8snKy9qu0OQddTwZD4bpurdrgGYNjKh+5TzpVWNpU145+PGxg4fh5MRz4e6HXwmgZjvyyeLOXip+64ZZcP6HP2iNpKP0dY1rMpVNojJvWSepbOcaoCptQ1nlMOnJRtVRo0ceZstnAODuR16B50feSZ1B81nzvIvPhX+8rgYmTLCXSadFyTWuyUTeaY3W2d41QFXbSsSmGlE78lz3IytjDI6ehgcOHoPDb\/xXJnLmCRrXUrdePxOqp0xyLovOEgWucY00ecucnoOA0OfxWcIiuo\/rk16FtWSjChTjZTBy\/sXzo9B38LeZyhr8CKzEUXNeRVCLPmvCBPj926\/BzJn+nrdaWPLWH17ZRnAN0GxWEHn7POl1l78YMR8YOgn\/9p\/HYWj0dOasmUUieymIKzq+fHU1nHv2BxKzZ99vwq5xjXTmrZqUVMlzDVBVdjE5vk8I3cSm2h9Z5WX1IyPmnx54Ax558XjujFksa8yY+n7m\/EdnTQhk51nJkdXGrJia7uca1+Qibzwtp62trYRhR0eH0U2pcGDXAFUdUL5PiHIlb0bKz468Az9\/9k14WUG2zGfNWG\/+WOUk+Lu5VfDRyX+cm5hl4tr3WHWNazKTNx5AjNvCsu1fWU28rq4uOIDY1OUaoKrt9n1C+EbejJSfOPo2PHzoePB1YNp1zVExxLJiJObZHz0Hls6tgo9UTDRCzDJx7XususY1mcibPtKRCWU1bXyfEEUgb0bIv\/+fM8HmR\/sH3j9RXBUp8+UKrDFfduE5sOhT55cCKE8pQ00UyknxPVa9IG90JWXecgGdt5XvE8IGeYtf9z125AQ8+tJbSrNkvoSB\/43bRf\/VVR+D6z9+njOZct7YFPv7HqvekDc6jmreqsN\/vDzfJ4RK8uZJ+elXTsIvX3wLBo79TmmGzGfJrK688OPnweXVHy6RMuohZsvkR\/1zRfcIXpG3brBk5LsGqIzOadqU+6Rnu8shIT732jvwH8OntGTIIil\/7qKPwE1XXgCTJp4FmJk3fvaCNG4b17bc\/ZgLPEc6u8Y1mWrejmAZqOEaoKqx8XHSswz55Lvvwc4nX4cnXjoGFZMmKc+QRULGD0pu\/vyfBGuW+d9U+yxMno9+pLKJiciJHoPI2y7+iaMXYdIzMsZ\/Hxw+Bc+\/9o627Fgk5EsvOAcWf\/YCmPahD5awxJJFWOkiEWyNDYrgx7zm+26ja4liJvJmywKnT59u\/OQcMcBcAzTvBLCdzYgv8\/YffRseP3ICjmioHTNbqyZPhIkTJwbnCuLa5Msu\/BDc8Mnzgk+uTWfIqv3H5PlObGin7za6xjWZyBsdFbY9LH2ko37qq5gQPCE\/Mfg2\/OrQW8FHIXipXO7GW8+vSZ417Wy44VPToPb8s0OzYxU2qkderUSyUS2eNqR5Q95h4OHqk507d9IBxAojK2zS82SMZYq9z48G+1mYImPMjr9w2VS4cvrkMWSc1WwitqzIudXPdz96Q95hmTeeJt\/T0wO1tbWRUcX3i9uBUJQf1dY1QLNOJyRkVqe978Ab8NKx3wV144Fj78DIyfeyio3sx2fGSMafmfERWHBp5Zj2pmrHvk\/6cigplIONrnFNprIJq3mjw9jn8TLscvr0aVi3bh0sXbo0IHjM1Pft2xdaN8dP73t7exNr6q4BGoYDnylveehlOPyG3rXHuKoCX+Td8MlpSjJjGd\/maUPknQc9d\/r67kfXuCYTeasKF8yu169fD5s2bQLxzEsk9sHBwcR9UlwDFIn63qdeD+rKaY+BEnHFzPe9996Di6Z9CC654By4ZV41TPrAWUEzU1mxKl\/HyfF90pdDVloONrrGNVbJOy7zxs\/vu7q6SnN+x44dgJteiZdtQHEVxtfvfTEVUfN7JfO147A9LIjYTNw+9I9BftSPse4RbHONaJ8V8uZP5QkjZSyvtLa2Qn19fbDFLJZQVq1aFVpPR0Dx6u\/v1+27kvy79r8F3U++HTseLn+78MMTYf7Ms+HaWe+vssC\/pb2Gh4ehuro6bbdCtScbC+WuSGV99eOCBQtKNg8MDDjjLCvkzaxnJI5byIZl1aydSOY8eqbuhviZ9q07XwheKooXOxKq8aoLc39GLcqmjM2ZuZJLEfJjLvic6GyKa2SNzUTeqraEjSNl3gDWrrGxcRzJ6wSUEfWiu54eR9pI2Pd\/5fLgBBKdW3bSpJcNZbfbkR\/d9o+Mdjq5RmZ8sU0q8g5bHigKbGhoiFwhIpI+ylu9ejVs3LhxzPJCsR2WTbAGHrayRRegSNwiaSNJ\/83nLgw2LNJJ2DymNOmzhLV7fciP7vkkrUa6uCatHqx9KvLmyx0tLS3Q3t4eu6Y7TCnxBsBq3mHE3tzcDCMjIxC3flw1oGy9NRI3u5Co+756hTHCJvLOGs7u9iPydtc3spqp5hrZcaPaZSLvvIOq7K8a0A17B2HD3qMlFVdc87FgiZ6pTFvEhia9ymixJ4v8aA97VSOr5pq8ekmTN3u5eOzYMbjjjjuCMsbBgwfHjR\/31WReZcP6qwIUM+5be18oLflDst6+5DKYd\/EUHWpLy6RJLw2V0w3Jj067R0o5VVwjNZhEI2nylpBlpYkKQMOI21aZhDJvK2GkfVAib+0Qax9ABdeoVLLsydtl4kZH06RXGe72ZJEf7WGvamQvyJv\/yEYEpmhlk18dfgu+ePczgRk2X0xGBRhNelVTz64c8qNd\/FWM7gV5RwGBdfD58+fHfnCjAkReRh5AMeu+\/PbHnSVuyrxVR4s9eUTe9rBXNXIerlGlAy9HadkkbqMpHcqjzKyAiuu4n\/nmXGsrSuKwoUmvK3LMyiU\/msVbx2hZuUaHLihTKXkX6TCGf+p7Ce58+JUA13+56RJYOrdKF8a55NKkzwWfM53Jj864IrMiXpB3XM07ave\/zIgldMwCqFguwazb1YsmvaueSacX+TEdXi62zsI1Ou1QmnnrVDRKdhZAF935dGk9Ny4JtL2Wm8omR2HmzJk2wsfYmETexqDWNlAWrtGmTJ6yifg5e9ze3DoNSAson3XPmzUF+lZcoVO93LJp0ueG0AkB5Ecn3JBLibRck2swic6ZMu+o3QBlT7+R0Eu6SVpAi5R1Iwg06aVDwemG5Een3SOlXFqukRKao1Em8la1JWwOvUtd0wCKe3KzDafmXXwu9H31chUqaJVBk14rvMaEkx+NQa1toDRco00JTnAm8o7aXztu61ZdxqQBtGhZN2XeuqLGvFwib\/OYqx4xDdeoHjtMXibyRkFYImlrawO2ugSJu6mpCTo6OoKjy0xdsoAWaYUJjx1NelORpHcc8qNefE1Il+UaE7rgGJnJGztH7c1tSnkcRxZQfqtX11eYEHmbjCAzYxF5m8FZ5yiyXKNTB152LvI2pWTcOLKAspIJ7l\/i8rpu0Vaa9C5EWX4dyI\/5MbQtQZZrTOmZibyjXliaUpofRwZQ\/kXl2oZZsPLa6TZUzTQmTfpMsDnXifzonEtSKyTDNamF5uiQibxxPNyEqqamxmh9O8xOGUC\/87MB2Nz\/ctC9SCUT1JcmfY7odqgr+dEhZ2RURYZrMorO1C0TeRdtS9jKb\/wyAKdoJRMi70wx7WQnIm8n3ZJKKS\/IO5XFmhsnAcqvMtn9lTlwzexKzRqpFU+TXi2etqSRH20hr27cJK5RN5KcpEyZt5xoM62SAP3J\/hH4h3tfDJRxddvXOKRo0puJI92jkB91I6xffhLX6Ndg7AjS5F3UA4iLusqEuYkmvekpoWc88qMeXE1KLSx5mwQpzVhxgPIlk1vmVcOGL9WmEe1EW5r0TrghtxLkx9wQWhfgDXkXYVdBfolgEUsmGK006a3PWSUKkB+VwGhViBfkXZRdBYteMiHytjpXlQ5O5K0UTivCvCDvouwqyJYIFmHf7qhopElvZZ4qH5T8qBxS4wK9IO8i7CrI17uL9mEOH5U06Y3PUS0Dkh+1wGpUqBfkjYi5vqsgvxFVUevdVDYxOje1DkbkrRVeI8K9IW9Ey+VdBX2odxN5G5mTRgYh8jYCs9ZBvCJvrUhJCg8DtGjnVMaZSpNeMhAcb0Z+dNxBEuoReUuAlKZJEnmvWTgT1iysSSPSqbY06Z1yR2ZlyI+ZoXOmI5G3pCv4ksycOXOgu7sbKivH70sSBmjvk6\/Dit4XgpGK\/LKSyiaSwVKAZkTeBXBSgopE3hI+xNUs69atg6VLl0JtbW3wcnTfvn3Q2dkJFRUVYySEAcqfVVnkl5VE3hLBUpAmRN4FcVSMmkTeGXyIWfj69eth06ZN47LvOPIu4hawIjw06TMEjINdyI8OOiWlSkTeKQHD5mkyb59eVlLmnSFYHO1C5O2oY1Ko5Q15s9PiRdvj6tMpcAqa8oc+sFPqRRkIKF79\/f3Bv5969V348u7Xg\/9eftUU+PLnpqQd1qn2w8PDUF1d7ZROqpUhG1Ujakeer35csGBBCdCBgQE74IaMKr0lLN+XkeqSJUuMHIPGxluzZg3U1dWNMUO8G\/rycQ4zkjI2Z+ZKLkXIj7ngc6KzF5m36QOIozbCQo+KgPIvK49\/\/1onnJ5HCZr0edBzpy\/50R1fZNXEC\/JG47EOPTg4CJgNq77EmwO+sFy9ejVs3LgxWH3CX1Hk7cPLSrSTJr3q6LIjj\/xoB3eVo3pB3iYOIJb99J4H1LeXlUTeKqeeXVlE3nbxVzG6F+StAghVMkRA2TawRf+ykmreqiLEDTlE3m74IY8WRN550AvpywPKn5xT9C8ribwVB4plcUTelh2gYHhvyJu9RNyzZw80NDTAsmXLYPPmzaEf0ijALVIED6hPn8UTeeuMGvOyibzNY656RC\/Im1\/9MWPGDOjt7Q0+Xe\/r64v8jF01kEweD6gv28DyWNGk1xU5ZuWSH83irWM0L8ibXw0yOjpaIm9cpB\/1GbsOMFEmD+jltz8O+NLSl5UmaB9Nel2RY1Yu+dEs3jpG84K8EZgNGzbAyMgI3HjjjfDAAw8EZZMVK1YEJRQdywejnMED6sOZlaKdNOl1TEPzMsmP5jFXPaI35I3AiJ\/Id3R0GPnikncKA5RfJujLShPKvFVPP3vyiLztYa9qZK\/IWxUoeeSEkbcvK02IvPNEhlt9ibzd8kcWbYi8s6AW04cByu9pQuStGGTN4ojYNANsSLzvfiTyVhxIDFA8OQeXCuLlw54mDCbfJwQ9XSieEBbF+R6r3pA3v86bxcvy5cuNvqzEcRmgPi4TJGKzyESKh\/ad2MohVr0gb0bcVVVVJbJmf0MCIoulAAAK8UlEQVQnhh1XpngulMQxQH1caVIOE4Js1DUzzMv1\/QblBXlHbQkbd1yZrlBCQB\/+9+cB13jj1fjZC+DOxst0DWdcru8TgsjbeEhpG9D3WPWCvNH7uEyQfVnJDgXGtd81NTVGlwsioD\/5xQFYdNfTQVAicSOB+3L5PiGIvH2JVP8\/KPOCvOO2hOVDEY9Eu\/\/++7VGp0jePq00IWLTGjpGhdNN2CjcWgbzgry1IJNRKJF3RuAc6kbE5pAzcqjiux+JvHMER1hXBPT6dT\/zcpkgZd6Kg8WiON+JrRxi1SvyxqPQ2traSlPC1ufxn2i5Dx49csKrDakYqDTpLTKuwqHJjwrBtCTKG\/LGl5P40rK7uxsqKyuB1cHxdHfTG1NNXvav3u0mSORtaYZqGpbIWxOwBsV6Qd6uLRU88cXuwIXzZk2BvhVXGHSn\/qFo0uvH2MQI5EcTKOsdwwvyRohcybxrPnEVnPyzDYHXfNpNkDJvvRPRtHQib9OIqx\/PG\/JGaFyoeU+\/6np4Z95qIm\/1sWpMIhGbMai1DuS7H70ib62RICmcJ+9nvjk3eGnp0+X7hEBfkY1+RKzvfiTyVhynVX\/+dXj30kWBVN8+0CFiUxwsFsX5TmzlEKtE3oon0IVf+mf475r5gVTKvBWDa0gcEZshoDUP47sfibwVB9D5f\/sDeO+8S7xc410O2QzZqHhCWBRH5G0W\/Alnzpw5Y3ZItaMReavF04Y03yc93aBsRJX6MSnzVoypr\/t4M5iI2BQHjCVx5EdLwCsclshbIZgoipG3b\/t4E3krDhTL4oi8LTtAwfBE3gpA5EUw8vbxAx163FYcLBbFEXlbBF\/R0ETeioBkYhh5+3YIA2XeigPFsjgib8sOUDA8kbcCEMMybx\/XeFPmrThYLIoj8rYIvqKhy568cSfCpqamEpw7duwA3IlQvPA8zObmZhgZGQl+wlN52A6GRN6KotERMURsjjgipxq++7GsyRt3I1y7di18+9vfDraRRSLHDa7CSDnsjMyw2GJlEx8\/0KHMOyebONTdd2Irh1gta\/IW5xLbAxz3\/xazb9z0anBwMHFvcCRv3M8EydvHiya9H14lPxbfj0TenA+xNLJ69WrYuHEj1NbWjvEuZuRdXV2J5RUi7+JPCiK24vuQMm\/zPrT2heXp06ehtbUV6uvrYfHixWMsF3\/DEsqqVaugp6dnHMkjeft4CAMDhIjN\/KTQMSL5UQeqZmVS5v1\/hyYwcq6qqkosi6B74ogeyXvimy\/CE2sXmvWkodGGh4ehurra0Gh2hiEb7eCuelRf\/bhgwYISVAMDA6phyyzPeOYdR8RRVrA+jY2N42rjSN6+fqBTDo+iZGPmuetcR9+fLso685YlbvGMzLhVKb6Tt2sBo4MxyEYdqJqX6bsfXbPPaOYtrt1m4YVrvWfPng0tLS3Q3t4e1LX5tlheCat3Y38k77MP\/Bg+OPSY+WilEQkBQqCsECjrsklZeZqMJQQIAUJAEwJGM29NNpBYQoAQIATKDgEi77JzORlMCBACPiBA5O2DF8kGQoAQKDsEiLzLzuVkMCFACPiAAJG3D14kGwgBQqDsECgsefNLCZcvXy71pabL3pWxh62T37NnT2BKQ0MDdHZ2QkVFhcumlXSTsZE3JG7vGxcNlrWPbxe11bGL9qFOsjbyWz\/7MD+ZP3DPpZqamnFbetjwVyHJm9+NEIM\/ao8UG4BmGVPWHtxpES\/cC0b2g6cs+ujoI2sjG5vZ99RTT0Wu8dehZ1aZsvaJO2nK7p6ZVS+V\/WRt5G+6uLVD0ecnT9y4WV5HRweRd9bAwuBYv349bNq0KdgXHCfAvn37CpWFihlmFnuKZHdan6Ftv\/nNb+C5554L3XUya+zo6idrH2akjzzySCGfFNPYyO\/Tj\/+NF279XMSL34sJ9afMO4cXxYMa4j6fzzGMsa5Z7SnSpEhjIyMJfNxGG8O2DDbmHMmBZO3Dm9KxY8egv78fDh48GHlClOSwRpvJ2iiboRtVXtFgVDbJCaSYcRadvLPYUzSb09iIE2T+\/PkwderUyP3ec4aQ8u6y9mG7bdu2lUpBRboBy9qI4DICxxtU1FGHyp1gQCCRd06QZTOAnMMY657Wnrj9zY0pnXIgWRv5skKRXljK2ifWuIt0E5a1UbSpSDeopLAm8k5CKOF32dpbzmGMdU9jT5EmOw+grI3iCUooI25jMmNOUhSTsgToil1ZfYj9WI27qDEb5gMi75yR6VtNTdaeIk8CWRtFsog6Ji9nCCnvLmsfv90xW4kheyiJcqVTCpS1MSzzHhkZKeyCAh4mIu+UQRPWXHa9qYKhjIiIsocPlrCstEhrvWVsLCp5o96y9vHtiuS\/NDZieaitrS1wZ9HWssdNeCJvI3RIgxAChAAh4C8ChfxIx193kGWEACFACMghQOQthxO1IgQIAULAKQSIvJ1yBylDCBAChIAcAkTecjhRK0KAECAEnEKAyNspd5AyhAAhQAjIIUDkLYcTtbKMAL8dbpotRl38uo9tl6rz4yN+OaJPn6dbDkOnhifydsodbiqTZvfCNG3TWJv1AyVXybu3t1f7RyvshtfY2Ah1dXVp4Ka2BUCAyLsATrKtYhpCTtM2jV3iZ+WyfYm8W4HIWzZaitWOyLtY\/tKmrXhKDytN8CeisK8Bh4eHobm5GfCTZ7zi2qLcm2++Odj+FK+4R3h+Jzq+La9DVKmBLxPwbZC8T506FfyDJxCJ\/XnZOCbbaJ\/txTJ58uSgH9Ob\/8oV7cb+3d3dwb7yUTqIThNvRKKOcV8kijejuJslZd7aposTgom8nXCDfSX43e7ESc8TBGqKJ6OwbE7c+S+sbX19fXDySNwugeLJQOLOiXFlE\/F0Gr7tD3\/4w4B8e3p6oLa2NtgfnO2zgWO2tLRAe3t78Bvfb3R0NLhBrVy5snRqCv87Hj2HOAwNDQXkjRfepHAzJixRxOkbRt54Qgt\/g4jaC4TI2\/5ccUUDIm9XPGFZD3GfaV6duOxOJFm+LWbo\/AlBKDNqbwiR2MPInD+dhdcvjijTkB3qvnPnzoCMkbzFTbFEWfy4hw4dAr6OHZf1hpE3T9ZxN7k09lDmbXlSaR6eyFszwEUSz28mxJcXRPLmSwf4iI8XO+2Gb4ulkqampnEQhK0WEQk4DXnH3VziyI49RbADna+++mo4efJkKHmHnRnK6\/zggw+WNmLiDQ477zCMvLEP20KV33kQnwj4i8i7SDNKr65E3nrxLax0vrzQ19dXOiMUs2k+I40rm4Rl3lGA2Mi88ebCZ\/Ni2SRP5h3neMq8CzstnFKcyNspd9hTJiyjGxwcDLJBsRSCteA77rgjqO1iP76mHFfzZrXpJUuWjDt9W2XNm78R7N69OwCVZbXik8GqVauCejjbW5vVsMPKJmlq3uzlJcNJLPPwJRYRQ\/7GibV1sYTFSjus7o6\/d3Z2gtiWyib25pOJkYm8TaBcgDHE1Sb8igdGRNOmTQtKCvgSEF+w4bV9+\/bg\/9mLOrEttuFXm8R9YBO12gRlJK3z5ld6YHv+5V8UefNlEywT4YtLtAVLQHiFHQTBSkbYHu3Cp5Kw1SbYP6xkwmwRyRtr3njjEA8lFksoPEZMh2eeeSYgb\/EFLJF3ASZeDhWJvHOAR13LG4Gsa8+Tat6qUCXyVoWkm3KIvN30C2nlIAJhyymzHGFG5O2gcwuoEpF3AZ1GKttBQCzrZD3CTNzbRKzLq7CO9jZRgaLbMv4XuTCxwshr4jsAAAAASUVORK5CYII=","height":221,"width":367}}
%---
