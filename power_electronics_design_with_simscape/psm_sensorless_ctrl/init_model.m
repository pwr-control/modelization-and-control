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
% model = 'psm_mpc_bemf_ctrl';
model = 'psm_mpc_ekf_bemf_ctrl';

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
% fPWM_AFE = 2*4e3; % in case of sv controller run 2 times the maximum pwm
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
%[text] #### EKF BEMF observer
kalman_psm;
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATEAAAC4CAYAAACLrdvMAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQtwXsV1PkCTWJQ4RuYphC0ecngkMY+EKIodQz3AhI5N+khkOdO4tkM8iXHaxKoeJm3GA8KP2gFjw8QThOsmtWA6MbXcZOoakTBDFDokxi4hEBPLQigCRsg4dhLTNq0758L5Wa3u3bt77967u\/a5Mwygf\/fs2W\/P+e7Zs497yvHjx48DP4wAI8AIBIrAKUxigY4cq80IMAIRAkxibAiMACMQNAJMYgUO34svvggLFy6EOXPmQFtbm\/WWDh06BIsXL4YpU6bA6tWroaqqynobosA1a9bAzp07YcuWLVBfX19oW7LwY8eOQXt7O9TU1BSCZZGdeeqpp2D+\/PlRE0uWLDHS3yXmMiZkb\/PmzYOmpqYiITOSzSRmBJdZYRckhm3u2rULbr\/9djNlNUrLDlVkW7I6RATbtm2DhoaGVG1NdXvkkUdg6tSpWrJTGxcKEPkODg5CV1cXVFdXm1QHn0gMFUd9cCyy9MWo4waFmcQMwPK9aNGkKToUYlFklCljbeI8pjgggXV0dIAuQZrYwYlGYqYvExOsspYNlsTQqDdv3hz1G6cY4hSHnO3666+PDBOfVatWjQmBxfo43aPpGDkA1j169Gg0fZLly2CTE9DfyRlkZ6JyOKVA3WlqQeWGh4fHTDnk6SL+iFMqeqvj\/9N0srW1NYq+9u3bF8mYPn36mLclORP+JvdVnO7q4Lphwwa46667xrVF+sTpQO0jnvgQBuK4iNMuEXPCASMwmpbT3+S2knRI+vv+\/fsrUz3SC9tI0iXO0WRdyJ5ovKjP+P9xRJlUH9MDZMtnn312BW8RMxlXETeyq2uvvTayGXwwghL7PGvWrOjvhw8frtiLbI+izqYviKzEZFIvSBKTQ2z5TUqOSIMd9zvldiZPnhwRATkIDRIaDQ746OioMuJQyZajFZHEyBlloyDnQd1vvPHGMTkvFYkhMQ0NDRnpivps3Lix8gLQwZVwk\/smk6Ssi4gTEiySMcqiMRL7jfkWMfKiMVi2bFnlRRT3O5GxjKmJbmgHKl3k6WDaiwaJSHzxpNWXcZNtWR4jEQfxpSbaA9kyti3riy8BzNfRS0+2d9lGys7D6pBZcCSmeisTEcXlbmjqc9ttt41LhqscQjVoaVOFpEhMfLOppjJpDpJktEkLCaI+X\/7ylyPnosgM+yKSOf5dxlpnOilHFRhxUVtiXiiOKMRFA3Hagrqgo4kRiBgxytFNUrQQpxu+TFQvIlzAUE2h4n4T\/0aEnZQTk3GQnTbtxYLl5WiMIsG4l5rcnmzDu3fvHjO1JizpBZJm8zqkY7tMcCQWZ6Cys993331jVtHE3+VpFwFKYbgcYahILO2tpENiqsStbRLDvhFho\/O2tLQAGacprkmRGEVX11xzTSUqjHtxxJEYpQdEI0fiwoS7TGLylAfrEMklRWJxuiWRWJIu8qpc3EtI7NvcuXOVkVhaPi6NxIjM8WUh4xxHYnJ7SSQmEw2lPpjELFBwEZGYqJbsACdSJIb9JAebOXMmHDhwoDKVNMVVJjEZt7iozyQSE8ckLVohx0x6Eal004nEVGbrMhKbNm3amFkFRdO05cZGJCb3nUnMAolRNCFOPZJyYvT20MmJJQ18WrQlyxZzCEk5MVWiVAzf5bc45SsoxyFPJ+OmhDLk4pRK3rOk89ZPyyViEhnzMRgNi4sXWXJicv5NNaWJyw3Jec4k3WQiSpvqipimRcumOTF5DFVjQiSG+mD+lqaCqulklpyYuHKb5g+WXNxITHDTSeqdzioa5njuvPPOqIpqdVJcyTOJxEgX09XJpByOvDopRk743zilwhXTuNVJWnEkXFQrqlQmbqVMB1daCZbb2rNnT5RPwYdWvSZOnBiRGj6UzEfdaGySViexPOkXFyWqVuXoRYckSjiodCPiwCQ3EQAlvGmMVdsvVKuLOpGLzuokYS6\/NMVVVLRjfBmTfSQtSol1ZJvC5L88VRfH6KRbnUQjxydut7oMlLwlwIiKpcKqPFMeuVzXHgKm+43ESMt0w6g9rU88SXFbb1S9NB23MhArLBKjziYds8Dfu7u7CzkuwyRWhumYtRG37UXc3pEmDe0FFyJcHHlK0y2k3+UtRKi7vCqt6o+PL5NCSAzD45UrV0ZYJJ11w7B4YGDA6ByZrrEwiekiVV45ecokThd1tAj57KRO\/8osI6c\/xM3eKj1oDE+Ks5NIInV1dRFJJU0nxdyLqUGXOeDcFiPACPiNgPVIDMPVrVu3wh133BGtUMWRGL1VGxsbox3YPFXw20hYO0bAZwQqJBYX7qcpjsn4Rx99tFIMyamzsxMWLFgQXdWiSuyLsmVSE3+7+OKL09Tg3xkBRsADBHp7e+Giiy4qXZMxJLZ8+XJYsWKF1l1RGHHdfffdUaKVnrhd1Phb2h1KRGLNzc3jrkJBEuvv7y8dGBsNsu42UDSXwbibY2ajhivcrU8nRTCSIjGM+kTCxOkklo27o8gVMCEPKuvOLz4bNmAqw5WvjptO2rwlVCQxmbjEqE21MdMVMKYDGFf+4MGDTsJr1p1xt2EDpjJc+eqYSExeepV3uZt2ykZ5V8DY0J1JzAaK5jIYd3PMbNRw5auJ00lxC4TN3fSmYLkCxlRPjsRsIGZHBpOYHRxNpbjy1dScWNIdTKYdzFreFTBZ9RXrsTPZQNFcBuNujpmNGq58NZXExM7RiuT69euNP3iQFSRXwGTVl0nMBnL5ZDCJ5cMva21XvppKYqoT9lk7a1LPFTAmOiaVZWeygaK5DMbdHDMbNVz5aiyJyfu90j6UYQOAJBmugLHRJ3YmGyiay2DczTGzUcOVryp37BfxCStTsFwBY6pnXHl2Jhsomstg3M0xs1HDla+OIzHxs1g2OpZXhitg8uqN9dmZbKBoLoNxN8fMRg1Xvmr12JENIGQZroCx0Rd2Jhsomstg3M0xs1HDla9aPQBuAwgmsSJQNJfJRGCOmY0aIePunMRsDEARMlwBY6MvIRsk627DAsxlhIy7K19N3WJhPgx2a7gCxkYvQjZI1t2GBZjLCBl3V77KJGZuZ9o1QjZI1l17mK0WDBl3JrEEU3AFjA3LDNkgWXcbFmAuI2TcXfkqR2LmdqZdI2SDZN21h9lqwZBxZxLjSMyqM+QVFrIzse55Rz9bfSYxJrFsllNQLSaCgoBNERsy7jW3fAWGv39P6cAlTifFz8Tjd+kWLVoE9957L5R5gwWi4YrdbYxEyAbJutuwAHMZoeLe\/fSrsLT7eTj0jRvMO52zRiyJiV8fmjp1auVL3T09PdDX11fIV7uT+sEklnOEM1YP1Zmwu6x7xkHPUc07EhPvwx8dHa2Q2NDQUPSFozKjMSaxHJaVoyoTQQ7wclQNFXfvSAzHAK+nHh4ehltvvRV27NgRTSeXLl0KOLVsa2vLMUxmVZnEzPCyVTpUZ+JIzJYFmMlZs2sA1uw66M90ktTHT6nNnz+\/0hsXHw5hEjMzJlulmcRsIWkmJ1TcvSUxM\/iLKc0kVgyuaVJDdSaOxNJGtpjfT3oSS\/rILsLNJFaM0aVJZRJLQ6iY30PF3TsSk+\/VjxsuW1dW05R1yZIlsbk2JrFinCVNaqjOxJFY2sgW87t3JEaJ\/bq6Omhqaqr0Gj+uOzAwEJEN\/jdut9iwYUNmVJAsV65cGdVHUoxbMGASywxvropMYrngy1w5VNzv\/+HL8Lc9v\/QnsS9usaivr68MiPjJNtx6gdsttmzZknnAcBqJRInEiA+TWGYorVcM1Zk4ErNuCloC597\/DDx54LA\/JEaR2ObNm4E+FiJP+\/JGYkiIW7duhTvuuAPuu+8+JjEtUymvEJNYeViLLYWKu5ckhsCKn24Tc2B5P6KLJwI6OzthwYIFgJFeWmIfdent7XVjVTlaxc3BtbW1OSS4q8q6u8E+RNxvmNMER25aEwHmzbGjoodP\/q4ltReX3OecWNGjES8\/1IiAp5Pl28vgoTfhqrt+fHKRmAxzWiTW399f\/shYaJGJwAKIGUQw7hlAy1Gl5z9H4C\/\/4Wf+kZi8W5\/6OH36dOjq6oLq6uoc3R5blUnMGpTWBDERWIPSSFCIuFd\/9QdRH0\/93evw+jc\/bdRfG4Vjb7GgfWLLli2Dxx9\/PMpdYW6nvb0dGhsbx2y7sKGESgZPJ4tGmKeTbhAOH3ecRt7e\/Xy0KonP6XsegqEffrt0OBNJbPny5bBixQrYvn17tA0C94vlTehn6R2TWBbU8tcJMSKgXrPu+cc\/TQIS2CM\/eRVW\/dvBqOiU6glw5KHPgovUj\/I+MVyRnDVrVnQI\/MEHH4xusxgcHLQ+neRILM1kyv+diaB8zLHFEHBHApv7wDOA\/yYC6\/nS1XD9h6\/wh8Ro+DZt2gQ333wz4MZWJDK8hmf16tVQVVVV2ghzJFYa1GMaCsGZkpBh3YuxGSStvv7D8KVtz1cawAgMCQz\/7cpX+WtHxYx3JJWdqUBwFaIZd\/u4t27fDw8++asxgmdcMgk2NV8eERg+TGIJuLsCxoYZsDPZQNFcBuNujllcjadfOgK3ffu5yrSRyiBpbZp3Ocy4dNKYaq58NTWxn3R20uYWC86J2TE6m1KYCGyiqS\/LJe44XVz\/2AB8+6lXYhVOIi8q7AWJJe2kF3tUdl7MFTD6Zpdc0qVB5tWfdc+LYLb6ZeKOpIVXSuP9+EkPEtfXbrkY\/vyac1M75MpXjSKx1F4UUMAVMDa6UqZB2tBXlMG620ZUT15RuCNhrds9AAOvH6vs61IR1z9\/YTrUn3O6ntJvl3Llq5zYNxoms8JFGaSZFtlKs+7ZcMtbKy\/uSFa\/eO23sPHxwVSyIl0x2mq9qQ5mXHpmJUmfpR\/OSUznNlfsWBHHjlSAuQImyyDKdfIapA0dsspg3bMil6+eDu5IVEg8f\/XIC3BQI7ISNcJ6n6g\/E1purMtFWHG9dOWrHInlszllbR2DLLD5XKJZ91zwZa5MuCNR\/c\/\/Hod7el+CwdH0KaDcIG17+PS158JffLTGOmExiRkMsSt2N1AxsSgTgQ0UzWWEgDuS1P8dPw4begfhwMjvYPCNN8dtZdDpOZLVlDMnwB9dNhn+9OpzSiGrJL1c+aoyEsPbWzs6Oio683cndczqnTIhOFNSj1h3s7GWSyNJ7fr567Bz30j0Ex2SziKViGpG\/Zkw78PnRSIo0soir6g63pEYXo+D1\/HQtTuUM2toaOAvgGtaAROBJlCWixWJO+Wj7vx+Pzx98Ne5CUokpI9fMgkWT38XnHVerZcklTZMXpGYzodCeLNr2pDysaN0hIopkYXE6DDz0y\/9Gp7Y\/0a0FSHrFE\/sFUVMOOWbdu4fwu03TIFTT0mOpLLoXgyK5lK9IjFUnyMx80GUa4RskCeC7hQ1DYweg4f6fgV7B49GQ2SDnMQICgmq7qwq+Ps\/mwavHfnvXNO9kHH3jsRwJDgnlo\/IQjZIX3WniAlH5kcHDsNT\/YejbQa2iIlGnPJQ+O8br5gMV184sZQpnq+463iClySmo3jRZVwBY6NfIRtkmbqLxPSd\/3glIiabEZNITFEEdeYEuLB6AsysPxMaL37nELMPyfIycbdh46IMV77K+8Rsj6QgL2SDzKO7SEp7h47CE\/sPwYuv\/a4QYpKndVfWnAELP3gaTKiuyTWtK9AslKLz4O5KZ2qXSSxhBFwBY8MgQjZIUXfKLeG\/X3j1t\/DYC6Pwwiu\/LYyUxhHTBWfAgoYaOP3dp2kR04mCuw0bLFOGK19VfihkypQppd\/kKoPuChgbg++rM4mR0jMvH4Ef\/OIN6B8pLlKSSWnK5Cq4rm4iXD+tOsozEUnawBxl+Iq7Tv9C1t2VryZOJ+Ou5eHNrjpm+E6ZMgxSJKSj\/\/V72LF3pLCckth7cesA5pc+dMF74aYrJsNpuH\/A8WbMMnA3swT90iHr7h2JxcGOq5UPP\/wwfyhE0yazGiQdScEIac\/gEXhp9FihUzc5UsLtApNOfRM+P\/uySk+LiJg0YTQulhV344YKqBCy7t6RWFwkhl8\/2rJlC4i3vcrjKNZT3Xghy08q6woYG\/aJBnna+86PROFepX99dqTwXJIcBeFKHE7fFjScD+dOfM8YUlL1MWRnYt1tWK+5DFe+qsyJYTdMvvZ97Ngx6OzsjD62i0SHkVtfX19sXg2PNHV3d6fm3FwBoxpCmsL9bPg38L1nR+DlQ29a36ekmrpNv\/C9cONlxU7dmAjMndhGjZBxd+WrhW6xUH1sFwluYGAg9RymK2CQqPB2gXsee8kqQYm5JDgF4Mrzz4AFH3tn5c11PokcMWRnYt1t0Km5DFe+WiiJqSIxPNa0efPmClLbtm0DPFwuP2UBg1sHWr+7P9NtAyIxfeySSfCZa8+Fd512Krz88svw8en15tbgQQ0mAjeDEDLuZfmqPDKFkJh4S2wcOeG0s729HRobG6GpqSm6LaOlpSU234bA4NPb22vdqh748RvQ9ZO3biJIemom\/kH007UXTICmD70X3jfhrb1K9HdV3aGhIaitrbWudxkCWfcyUB7fRoi4z549u9KR\/v7+0oErhMSoF0RmbW1tsVEWlZNJTUTBNrvjNPH27udjIy46Lyd+EDTPiIT8VmXd84x89roh427bV3VRNPrakSrHFdegipzE8lSuubl5HNnZBGbu\/c+MIy8krn\/54lVQN7lKFzPtciEbJOuuPcxWC4aMu01fNQF1DInl\/e6kfA8ZymttbYW1a9eO2ZYhl8PpJObI4lZCbQDz5C8Pw9wHnqnggsR1w7RquOcz7zfByrhsyAbJuhsPt5UKIeNuw1ezgGgUiek0IBMh5cTiCG7hwoUwPDwMqv1neYDBqSN+GBQ\/EErPZ687H\/7mJvtfeonDJmSDZN11rN1+mZBxz+OreZAsNCeWRzGqmxUYOfeF0VfPl64u5U4o0j1kg2TdbVivuYyQcc\/qq+Yoja1RITFKwo+MjMC6deui6d2+ffvGyQ\/lu5Ni\/ssFgSFwIRsk657XtbLVDxl35ySWDfLia2UBZs2ugcoU0hWBMYkVbxtJLYRMBCHrnsVXbVjJCTedFJP4LgmMScyGeWaTETIRhKy7VyQmblaVzcjn6STmwa6668eRyq4JjEksGwHZqBUyEYSsu1cklmRImCebNWuWcuOqDSMUZZgAI+bBMIk\/49J37k+3rZeOvJANknXXGWH7ZULG3cRXbSJnNJ003exqQ1FdYMRp5IxLJkHP0qttNJ9LRsgGybrnGvrMlUPGXddXM4OTUNGIxHy+FJGiMB+mkYR1yAbJutt2NT15IePuFYmpcmJJt03oDZF5KR1gxCis81OXwhc\/caF5QwXUCNkgWfcCDEJDZMi46\/iqBgTGRYwiMWPpFiroAONjFMaJfQuDn1FEyEQQsu46vppxSJXVEklMPiakuhusCMVIZhow4opk80fOg\/ubLy9SHSPZIRsk62401NYKh4x7mq9aA0kSFEtiSbdP6N7GalPZNGDEFcm9X\/tYqceK0voZskGy7mmjW8zvIeOe5qvFIAZgdADct9VJMQrzZUVSHKiQDZJ1L8rl1HJDxt0rEku630t1ZU5RQ64CRkzo+7AvTMYgZINk3YuyaCYx28gm5sRw6tjR0QG0GokENn\/+fCj7A7oqEhMT+jiV9O1hInAzIoy7G9y9isQIgqS7wcqESAVM9Vd\/EKnyJ1edA12fu7JMtbTaYmfSgsl6IcbdOqRaAr0kMS3NCy6UBIw4lfQtoU+QsDMVbBwJ4hl3N7h7RWLy9go3kLzVahIwvk8lUXd2JjeWw7i7wd0rEkMI8LB3XV1d9Ek1l08SMDSV9HFVkiMxlxbDLw9X6HtFYr5fxSNurfBxVZJJzJUbvdUuR2Ju8PeKxNxAEN9qHDA+b3AVe8HO5MaSGHc3uDOJJeAeBwxefIjRGN5Y4ePWCo7E3DgR4+4Wd+ckFsqHQsSp5F\/Pngp\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\/MQvHvwR+a94xqMACNQKgJeRWJZei7v\/SIZmLSfNm0aiB\/kFcvi8aa4fFgWHbgOI8AInFwIKCOxkwsK7i0jwAiEiACTWIijxjozAoxABQEmMTYGRoARCBoBJrGgh4+VZwQYASYxtgFGgBEIGgFvSUxcvXRxo6zuqOroKd+Si5+9K\/NiyaS+6Ogu1lUdI9PFy2Y5Xf3Fckm3pdjUS0eWru7irTA++wH12cV1XV6SmHhwHI2uvb0dGhsbS73HTMcQdfXE\/Xb44PEr3b10Ou3nKaOrO7VBev\/0pz\/1YjuMrv7yJQS6Fw\/kwTatrq7u4ksDT3746gcigeF5aNMz1ml4pf3uJYnJN8ii4fX19XkRvciRiXjTra6euuXSBi\/P76YYo87PPvssPPfcc7EH+vPokqWurv4YyTzxxBNa532z6JGljonu4lVW+N\/46JxdzqJX1jriMUOUUfbFqV6SmHzXmGpHf1bgbdTLqqcPxmiiOzkdTmdQ97hbSWzgaSJDV38k35GREejt7YV9+\/aBD9NJXd11IzYT3Iouy9PJtxGWIxVfSSyLnr70xUR3NMxZs2bB5MmTE69WKto5ZPm6+mO5jRs3VqbAPrxAdHXHPou3LCddV1U29qr2mMTeRkf3TeV68Ez1VF05VHZfdHUXp2M+JfZ19ZdzYD68RHR1l3X1gYDT7JRJ7G2EdHMGaYAW\/buJnj44T5Z8nnx5Jcrw4ayrLva6hFG0rWTFXsyB+WZDcZgxib2NSii5AF09fTQ+Xd1l50u6qbdMEhCnWZjkVq1gizem0Aqf+D3VsvU20T0uEhseHvZugUvEkElMQEN3H40LI5Qde+HChYDGJe7jEQczLprxYa9YEsZJhujTdBLHQFd\/sZwPuJvoLl6H5cOiRJq\/MYmlIcS\/MwKMACMgIeDlFgseJUaAEWAEdBFgEtNFissxAoyAlwgwiXk5LKwUI8AI6CLAJKaLFJdjBBgBLxFgEvNyWFgpRoAR0EWASUwXqZO8nHjSIaI3AAADaklEQVSdkMmVMD7uMqfrbYrctCtu6wjhuFDI5s0kFsDomdx6YVLWpOtZN+z6SmLd3d2FbxpVfU\/VBHsuq0aASSwACzEhJpOyJl2Xj+\/o1mUSa4e4j0Lr4sfl0hFgEkvHqJQS8u2vNGUTb\/ak3eb4qXs6JYDKqcqi3MWLF0fX0OCjmtqINyaIZUUdkqZgSd8RRRI7evRo9M\/OnTvHnbsUZWObdKEenY2cOHFiVI\/0Fk8\/YL+xfldXF1RXV4\/Zwa+aKsqELOuo2hkvk7LqpcGRWCmuA0xi5eCc2op424Js\/KKjoCC84ZPe7vJRoLiydCuu6tiQfOOsfOOGajop354qlv3Wt74VkRB9HBlJgM7\/YZviB5XFeqOjoxFRL1u2rHKjr\/h7VVVVhMPg4GBEYvggWeNZyoaGhojcxAsFxQGIIzG8kVQkyqQzikxiqaZcegEmsdIhj29QvvdKLKV628tkI5bFiE28eRZl6p6LjCM1FSkk\/Wbi9Kj7ww8\/HJESkph82FyWJRLV\/v37QcxzqaKgOBITSUtF9ib94UisHOdiEisHZ61WxMO+4nRIJjFxSoVTH3zotlWxLE4h58+fP67tuNVFOXIxITEVyaqcnqJKjNTwmTlzJhw5ciSWxOK+TSDqvHv3bujo6BjX17j73uNIDCvStc\/izRf19fVjZDKJaZlyqYWYxEqFW78xcdrV09NT+cYARldihKKaTsZFYkkayHJMSEw1dVM5PZKsGMHJ08k8kZgKaY7E9O0whJJMYp6MUpyzDwwMRNGBPEXEXNG6deui3A\/WE3NOqpwY5a7mzZs37stRNnNiIiFu3749QpiiHDlSbGlpifJldNcX5bjippMmOTFaaCCc0nJiSXk7zL3JU3ua8lJeDn+P+wQfTyfLcS4msXJwTm1FXp0UV8jIIc8+++xoqoXJckxE47Np06bo\/ymhLZfFMuLqpGqjatLqJMpI2ycmrk5ieTFJnkRi4nQSp88rVqyI+oJTY3ziLmCkqTSWx35hlBq3Oon1kz4dlhSJIYHKHxORp5YiRqTD3r17IxKTFyqYxFLN3koBJjErMLIQFwhk3buWlhOz1RcmMVtIquUwiZWDM7diAYG4bShZrppmErMwGB6JYBLzaDBYFTUC8nQ361XT8tlJOW9nYxz47KQNFPVk\/D+ceYL1Wmx+4gAAAABJRU5ErkJggg==","height":0,"width":0}}
%---
