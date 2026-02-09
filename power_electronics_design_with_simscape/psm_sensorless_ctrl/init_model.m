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
model = 'psm_sv_ekf_bemf_ctrl';
% model = 'psm_mpc_bemf_ctrl';
% model = 'psm_mpc_ekf_bemf_ctrl';

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
% fPWM_AFE = 6*2.5e3; % in case of mpc controller run 6 times the maximum pwm
fPWM_AFE = 2*2.5e3; % in case of sv controller run 2 times the maximum pwm
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
%   data: {"dataType":"textualVariable","outputData":{"name":"deepPOSxi","value":"     5.000000000000000e-01"}}
%---
%[output:05701a1c]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGxi","value":"     0"}}
%---
%[output:7e5df9b4]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGeta","value":"     5.000000000000000e-01"}}
%---
%[output:8bf67124]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"     6.766822226281998e+01"}}
%---
%[output:45e82fc7]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"     2.831309534039184e-01"}}
%---
%[output:3873a036]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["2.831309534039184e-01"],["6.766822226281998e+01"]]}}
%---
%[output:2f8df18e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:45ef9943]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:20f05bf9]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","2.000000000000000e-04"],["-1.973920880217872e+01","9.968584073464102e-01"]]}}
%---
%[output:0445503e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["3.110176727053895e-01"],["5.433217222799691e+01"]]}}
%---
%[output:04a70309]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"     3.946150308374179e-02"}}
%---
%[output:69e958f5]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"     1.254711180325163e+00"}}
%---
%[output:8e3d2c7e]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"     6.069317568680815e-01"}}
%---
%[output:95bd65ef]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     3.449190983162578e+02"}}
%---
%[output:7d37a44a]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -4.186041938481557e+02"}}
%---
%[output:9ef89060]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"mosfet","value":"    data: 'danfoss_SKM1700MB20R4S2I4'"}}
%---
%[output:339abfd4]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAmcAAAFyCAYAAACwQX2kAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQe4FsXVxw9KEb1IERQpEUTRa6EYa+xR0Vj40E8RCaIhSNXEIE1CYjSoQZCIhSJEEVFKLCiWgBoMigUxgH4GNCCoiCBIF4KIfM8ZM9e9y77vO7s7s2ffu\/99Hh7gvjPnzP7mvDP\/O7XS7t27dxMeEAABEAABEAABEACBVBCoBHGWinpAIUAABEAABEAABEBAEYA4QyCAAAiAAAiAAAiAQIoIQJylqDJQFBAAARAAARAAARCAOEMMgAAIgAAIgAAIgECKCECcpagyUBQQAAEQAAEQAAEQgDhDDIAACIAACIAACIBAighAnKWoMlAUEAABEAABEAABEIA4QwyAQMIEZs2aRT169KAaNWrQxIkTqWXLlgmXgGjr1q3UtWtXmjdvHrVr145GjBhRVgYu35AhQ2jatGlUv379RMu2aNEi6ty5M23ZsoXGjBlDbdq0Uf51ec844wzq1atXomUq5KxPnz40ffr0cuUtlEd\/buO9Ro0aRXPmzKHx48dTSUmJqWtr6bx1xkYbNWoUK3byxaa1QidgSHMpLS0Vq5sEXhMuHBGAOHMEFmZBIBeBNIsz7uiHDx8eu4ONWvtB4mz16tXUvn17WrlyJfXt2zdV4kyXt2bNmqEFiY330sLwxBNPFBEAXiGl6zxuWSqKOPO+h\/cXjajfDeTLFgGIs2zVN94WBPISgDgLFyCal3\/00cRKRRNnURgEcaoo4ozfLU58mMQQ0lRcAhBnFbdui\/LN9KiSLnzQ1J93tODKK6+km266qexdc42s6Dw6oT+dv0O44IIL1NRjrvS54Ho73Fx5g0bOvO\/UunVrGjt2rMruLadu6LVd\/\/RRLmHlZap\/g\/e\/72233VY2zel9t1yjILlGbLzv7x8tMKlb\/8gZl8VbD7psXtv+uuU0QSMV\/uk3TrN06dLAkUKTqbow78pl8ooXP4uw7xUUZ34fJu+Qr5EoVF9++6bflaB8QVPYesrd5Lvo\/274vzv8f5PvmDeWOPbvuOMOuvbaawNHbQu1KexTvyv\/W2oJQ1F2BCg0bghADKSHQFAnG9Tg50vn76CCpl20TW9nmS9dnE4nyFc+ceatDa8wzfXO3jRJirNcU7P6537haFq3YcVZPrveDj+XGAoSurnS+n9RKMQg6JulY66QOCv0Xi1atCib6vX6KWTfdJ2jSX1FEWf56kH\/ImLyXfTWbZAwM203NI9mzZoF\/nLiZWtSPv\/ooY3R0fS00ChJkgQwcpYkbfjKScDbaHtHi3QnkUuoeBvDoLS64fbmD+pU\/R2Cbvy9HVC+tTTe\/F5hElSmQuLMP6oXxMZbLs0gjjjTGwJMpzWDOp1c01Fh6jbMmrOgUQlvuTSXXHXjLZeuMw5Qvb4tKL833nKxChpVDIrDXB236Xv5R4P0hoBCDApNP4aprzBTkN5y6e8Sv4PemKLrgDc16J\/x5\/q7GPReQaOX3jJ5v7NewWnyHfO3CTqPaZvCZQ\/DB90DCHgJQJwhHlJBwGSaTDeOOq1\/dMbf2fGuv6Adid4GM+i3Yb8IM1l0nWuXYdDOx3ziLGinW5D\/oF1+SYqzIGGwbNmywJ2WYeo2jDjzB65\/BEWLEK9Nf6fsj6X33nsvcCdt0IhgrvfyioB8Qsh0VCXXe+USZ4VG9ArtpgxTX2HER65y+Xeb5hJXud7XGwf+kbkgcZbvO+b\/zB87YdoUXS6T9iMVjTAKkSoCEGepqo5sFiZfAx\/0Wa7Gzp+2f\/\/+gVM\/QdNA+cpg0rjmEmdBNVpozZn\/SAQT\/+wnaXHmH+GZO3fuHuu3wtZtWHGWb0orSJz516L5mT311FPqHXI9QdNgfgEWNN0XNJ2YT5yZvFcusZIvL+fJN7UZtr5siDM\/67DfxXxTpUHiLGgE3FSQXnbZZcZtin4v09HobLb+eOtcBCDOEBviBMJ2CBBnwWdZJS3OvPXWvXt3WrBgwR7npoWt2zDizNspa8Fx0EEH7TEtmU84uxBn+gsVJBq8IzO5xJnpe0GcjSfvaC3zYPH9k5\/8pGzEHOJMvHlHASISgDiLCA7Z7BIw\/c2VDyCNO60ZVPKwv637beSaOtM\/HzZsWNmBqlFHzoIW2a9atarsfKukxZl3tK5hw4a0efNmhcW\/Ky1M3YYRZ\/p9vR1w0LokG9OaYUZ3guIrqAy5xJnpe+USZ7mmD02\/sWHqK8rImRZR+oBhLm+\/fv3K4ibMd\/GNN95Q09De70ahNWf5Rs6iTmvmYxtUn6Z1gXTZJQBxlt26T9Wbh1mEnGtNj+mGgCABEKZDCDqFPdeic+8Uk55SCyvOgtgELa7WHRVXrF5b5T9yIddRGmE3BOjg8U\/hBXV8Yeo2ijgL2rHK5bO1ISCXCMq3FpCPgSgkGguJs0LvlatcQQI1V9qgRiBMfYURZ0Exy98l\/\/fWu3PSP2XsZ+6Nef\/3i9\/NdOQs6J3DbAjINzpruiwhVQ0yCiNOAOJMvApQAE2g0PEB+jftfOm8nTL\/O9d5UP6GO644Y3u5jhbw+worzrwda1C0BO0szRVVhcRZvgXVQTZzdWD+tKZ1W0g4a7v8HjyFqa96CiqbybliRx55JC1ZsqTcyEu+NVtBRzj4R1vyrYHyCi4\/O36HsO+Va7OA6TvkihPT+gojzthXPjaFdscGCUz2r3fXBr2LqTgLqgu2p0eE+TqxXL\/weP36fzkJywe9AQhoAhBniIVUEfA33oUOof3Vr35FPXv2VHcx8mN6CK3\/N3Ib4iyXGAy6u9J\/t6bJb9f+DjeITdBIlveg3kLizN9JFdrZ5+1wC52hZVK3+Xa9Bh0K7LdperCsLmvQJoYgoZ2PNaf3T+UG\/VIQxNJffh2\/pu\/l9+MVB\/5YKFQ\/\/obApL6iiI+gX2K839uw30W\/PbZ12GGH7bHr1uQ75h+V924qyrXTV3ML2pmb76DiVDW8KEzqCECcpa5KUKBCBEwa2UI28DkIMIE4O+nQ8Vb8GDI97iQXiaAz6yo+NbyhDQIVWpxxwztlypSCFxLnmo6Ke4GvjQqCjT0JQJwhKsIQ8HawuRaOFzqYNZc\/Le6i5g\/zHkjrjoB3VNo7QhZ3cwXiw12dVXTLFVacacFVs2bNguLMv1uoold6sb8fxFmx12Dy5S+0TjHoLk6TUoZpZ0zsIY0MgXzrRblEUX5R907PRo0vGRrwmgYCFVKc5do5l++33zlz5pQdSZCGikEZchOAOEN0RCEQtEg+7DqsIL86HtEBR6mV9OQxuUw+TGm14CstLUXfEgYc0ioCFVKc6etAWrduTc8\/\/3zBkTNuXPnRxwkgNkAABEAABEAABEBAikCFE2feKUreiVVozZn+bYnP2+Et9frBGhKpkIRfEAABEAABEMg2gQolzrTQ6tChA\/FJ8iYbAvTQ8znnnFM2cqbtNGjQoOBw9MqVK7MdQXh7EAABEAABEEiAAG\/oycpTocQZT0\/6r7MpNHKWq6L1Lp1860hYmPG1I2+\/\/XZW4gXvCQIgAAIgAAIiBE466STiq\/CyINIqjDgL2nFpMnKWK8L0iBpf6MyjcEHPW2+9RR07dlTBwidJ40mOAAvikSNHgn1yyMs8gb0AdI9L8JfjD\/by7HnzHsSZXD2E9lxoq3yuk+NtiLOsBEvoSnGYQQtjsHcIOYdpsE+eudcj+MvxB3uwT4pAhRk5CwJmMnKWa\/rS5OwzfFGTCtM9\/YA92MsRkPWM2JfjD\/ZgnxSBzIszfSYar1WbNm0a1a9fv+wCa+8mgXzTmhi9SSpcf\/CzYsUKmjBhAg0ePJgqV66cfAEy7BHsZSsf\/OX4g70c+6wJ48yJMx5NGzt27B4XFfunRU2mQbMWLHJfyz09\/+c\/\/6EvvviCGjduDHGWcMWAfcLAfe7AX44\/2Muxz1p\/W6HFmeswylqwuOYZxj4ayTC07KYFe7s8w1oD\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\/J1iNjIK9ESYniab+4\/\/opluG0luT7qRGjRo58ZEmoxBnMWoD4iwGvJhZ0UjGBBgjO9jHgGchK\/hbgBjRBNhHBGch2yUPLKC5yzbSe30OhzizwLPMxKZNm2j06NHEf0d9atasSQMHDoya3Xo+iDPrSI0NopE0RmU9IdhbRxrKIPiHwmU1MdhbxRnKGMRZKFzmiVevXk3t27enlStXmmfypeShzDlz5kTObzsjxJltoub20Eias7KdEuxtEw1nD\/zD8bKZGuxt0gxnC+IsHC\/j1FqcDR48mNq0aWOcTyecNWsWDRkyBOIsNLmKmQGNpFy9gr0ce\/YM\/nL8wV6OPcSZI\/Zr166l66+\/Xv05\/fTTQ3t57bXX6P7776epU6eGzusqA0bOXJEtbBeNZGFGrlKAvSuyZnbB34yTi1Rg74Kqmc1WQ96kT9f\/B2vOzHCZp9q1axfxn6pVq5pnSnlKiDO5CkIjCfZyBGQ9I\/bl+IO9HHuIM0fseVqzY8eO1LhxY+rVqxcdf\/zxtPfeezvyloxZiLNkOAd5QSMJ9nIEZD0j9uX4g70ce4gzR+x5l+Yf\/\/hHeu655+ibb76hevXq0bXXXksdOnSg2rVrO\/Lq1izEmVu++ayjkQR7OQKynhH7cvzBXo59nT6zlXMcpeGoDrZu3UqzZ8+mhx56iN577z3lpUWLFtSzZ08666yzimraE+LMUZAYmEUjaQDJURKwdwTW0Cz4G4JykAzsHUA1NAlxZgjKRjLeJPDEE0\/QhAkTiP9dUlJCV1xxBXXp0oUaNmxow4VTGxBnTvHmNY5GEuzlCMh6RuzL8Qd7OfYQZwLsd+\/eTR999BGNGzeO\/va3v9G2bduoadOmajStbdu2qR1NgzgTCJb\/ukQjCfZyBGQ9I\/bl+IO9DHvepclrzvjBtKZMHaj1aG+++Sb97ne\/UyWYNm0a1a9fX6g0+d1CnMlVCxpJsJcjIOsZsS\/HH+xl2EOcyXBXXrUo4ylOFmd87AYfVnvHHXcQX9uU5DNq1CiaMmVKQWEIcZZkrZT3hUYS7OUIyHpG7MvxB3sZ9hBnCXNnAbZw4UKaPHly2XSm9C7ORYsWUefOnZUgLDRqB3GWcMB43KGRBHs5ArKeEfty\/MFehv3rSzdS21ELlHNMazqqA15fxvdr8gjZs88+S1999ZVaU3bhhRdS9+7dqXnz5lSpUiVH3vOb5Z2kXbt2pXnz5qlb7yHORKrByCkaSSNMThKBvROsxkbB3xiV9YRgbx2pkcHJ76ym3pMXQ5wZ0QqZaPv27fToo4\/SxIkTadWqVUqAlZaWqrPOLrjgArVTU\/rh6Uy+WL1169b0\/PPPQ5xJV0ge\/2gk5SoH7OXYs2fwl+MP9jLsh85cTkNnroA4c4FfX3y+cePGVB6XwRer9+vXT4nHuXPnhlpz9vjjj6uRNn7SunnBRZ1K2kQjKUcf7OXYQ5yBvSyBZL2zbuDn8XfX0Z\/+sQHizAV+HjlbsWIFNWvWLHVHY2jhyLcV8NVSYTcEeHnxerVrrrnGBULY9BDYsWOHOhuPxXDlypXBJkECYJ8g7ABX4C\/HH+yTZf\/II4+oAZOtp\/Wnb+seAXHmAr8WQIMHD1a7MMM+PLI1ZMgQNe1o++nTp4+aah0\/fryaXg0rzoYNG1Z2aC6LBYye2a6hPe2x2F+zZo0asYQ4c8\/b6wHsk+Xt9wb+cvzBPln2rBv4T4+XdxPv2OQHGwIs10FaxZl3OrNly5bqrcOKMxaMelrTMjaYy0EAU2tyoQH2cuzZM\/jL8Qf75Nl7j9GAOHPAX4sz3qkZ9WEBZHvkjEfNpk+fnrNIffv2VVOdQQ+O0ohak\/HzoZGMzzCqBbCPSs5OPvC3wzGKFbCPQi1eHu8xGhBn8VgG5t60aRONHj2a+O+oD589NnDgwKjZjfNh5MwYlVhCNJJi6DFyI4deeUbsy1UA2CfP\/pYZy+i+2Z8qx3ttW0cLB5+SiZmqSrv54DE85QhAnKU\/INBIytUR2MuxhzgDe1kCyXu\/5IEFNHfZRuV4nw+fo3ljfwNxlnw1pMMjxFk66iFfKSAQ5OoI7OXYQ5yBvSyBZL37pzRLXr+L3nhqHMRZstVQfN6w5kyuziAQwF6OgKxnxL4cf7BPlr131IynNPefNUCtO8\/CBjxMa8aINYizGPBiZkUjGRNgjOxgHwOehazgbwFiRBNgHxFchGz+XZpn1lxDix4ZBHEWgWXmskCcyVU5GkmwlyMg6xmxL8cf7JNjf+O0D2niW6uUwx\/V2YfGnFuJOnbsCHGWXBUUryeIM7m6QyMJ9nIEZD0j9uX4g30y7P1rzXqd2ZguPmgdxFky+L\/3wptFN2zYQLt27aI6derQXnvtRTt37kzdFU9BTCDOkoyU8r7QSIK9HAFZz4h9Of5g7549T2e2HbWg7EYAHjXj4zOy1t+KrTljUfbSSy\/R7373O3VHIi\/wmzZtGlWrVo1++ctf0vHHH098AGzVqlXdR0NED1kLloiYnGRDI+kEq5FRsDfC5CwR+DtDW9Aw2BdEFCsBC7PekxeXHZ3Bxh64qpSuOqE+xFkssiEy\/+Mf\/6Du3bvTcccdR4cffjjNnj1biTO+2\/Lmm2+mmTNnEt\/DyReJp\/WBOJOrGTSSYC9HQNYzYl+OP9i7Y8\/C7Dd\/\/ZBmf7i+zEm30xvSny5trv6ftf5WZORsx44d1KNHD\/ruu+9ozJgx9Nprr6lLzVmc8aXh33zzDfG1SuvWrSu7jNxdSES3nLVgiU7Kfk40kvaZmloEe1NSbtKBvxuuJlbB3oRS+DRBI2Z6OlNby1p\/KyLO9D2b3bp1o06dOhFfPu4VZ1wZkyZNogcffLBMsIWvbvc5shYs7omae0Ajac7Kdkqwt000nD3wD8fLZmqwt0nze1v+NWb8MxZmz\/Zqrf6GOLPPPKdFLc6uvvpquu666wLFGZ\/S\/8QTT9DUqVOpXr16CZbO3BXEmTkr2ynRSNomam4P7M1ZuUgJ\/i6omtkEezNOpqn8uzJzCTP+edb6W5GRMz2tuW3bNho3bpyC7h05W7lyJV177bXUuHFjNe3JmwTS+GQtWNJUB2gk5WoD7OXYs2fwl+MP9nbYB01jamHGOzODnqz1tyLijMEvWLCAeFqTNwPwnxdeeIFuvPFGWrp0KT355JNq3dnYsWPpzDPPtBMNDqxkLVgcIIxsEo1kZHSxM4J9bISxDIB\/LHyxMoN9LHxqCvPF\/1tHN0\/\/9x6Gnu7Ris5sXjung6z1t2LijGvgjTfeUEdpLF++vFyF8DTm7bffTueee268SHCcO2vB4hhnKPNoJEPhspoY7K3iDG0M\/EMjs5YB7KOhZFG2ZPXX1GH8e3sYCFpfhpEzIlFxxhXA553xrsx\/\/etfxNOdRx99tNqxuffee0eLggRzQZwlCNvnCo0k2MsRkPWM2JfjD\/bh2T\/29hd0w9QlgaJsZPsj846WeTNlrb8VF2fhqzo9ObIWLOkhj3U3knWBDkqSPmJfkj5ivzB9HiXjP0NnLi93mKw3Z+eTG1Cfcw8ptxuzkOWs9bci4mzTpk00evRo4r8LPXXr1lXTm8ccc0zqRtOyFiyF6irJz9FIJkm7vC+wl2PPnsFfjj\/Y52bPguzZRV\/S72csC0zE05enH1ab+rVpEkqUaWNZ629FxBlf13T99dfTBx98QLxjkx8WYfzwFGfQc\/rpp9N9991H+++\/v9w30+c5a8GSGvDooESrAh2UKH6IM0H8iP094b\/4wTq6+el\/l92F6U\/BouzqkxvQFccdFEmUQZwlHPDz5s2j3r17q+uZfvGLX6hrm\/jhXZp\/\/etfic85u\/\/+++moo46iZ599Vh21wWlvuummhEua2x3EmVxVoJEEezkCsp4R+3L8s85eT1lOfucLmvzO6rwVwaLs\/g6ldNphtaxUWNb6W5GRs61bt1LXrl2pefPmdOutt1KlSpXKVR5vErjlllvok08+UeecVa9enYYOHarOQ3v66aetVLQNI1kLFhvMbNnIeiNpi2MUO2AfhZq9POBvj2VYS1llz4fF5ltDpjmyIPtj28OoZaMasUbJguola\/2tiDjTNwTwyNmVV14Z+P3gmwEeeOCBsuubnnrqKbrnnntozpw5Yb9PztJnLVicgYxgOKuNZARU1rOAvXWkoQyCfyhcVhNngf0Po2OriUfICj0syK4\/+0fUpvQA64LM6ztr\/a2IONuwYQN16dKFmjVrRnfccQdVrVq1XP3z1OagQYNo2bJl9NBDD1Ht2rXVejM+qPbFF18sFCuJfZ61YEkMrIGjLDSSBhhEkoC9CPYyp+Avx7+isueRsb++u5oefbuwGGP6LMh6ndmYLji6rlNBBnEmEOt8qfnw4cPpiiuuUJsD+GwzfnhUjdeaTZkyRa0v69mzJ\/H6tJtvvlmdgcYiLS0PxJlcTVTURlKOqLlnsDdn5SIl+Lugamaz2NnrUbFP1m+nKe+sznnUhZ8Gi7HGtfehAec3VWLMeyG5Gbn4qbLW34qMnHE18egYi7OHH36Ydu3aVa7m+ABa3iTQt29ftTOJL0dftWoVsaArLS2NX8uWLGQtWCxhs2Km2BvVQRiRAAAgAElEQVRJKxCEjIC9EPj\/ugV\/Of7Fxl6LsSf+uYZe\/Wh9zh2VQWKMf8YL+qXEmL9MWetvxcSZBs\/Harzyyiu0aNEi9SMeHTv77LOpYcOG6v\/bt29Xo2kNGjRI3QXoWQsWuSZxT8\/F1kimiV3csoB9XILx8oN\/PH5xcqeVPYswfvhvHhHjkbG5yzYav6oeGet44sF0arNaIiNjhQqbtf5WXJwVqpA0f561YElTXaS1kUwTI1dlAXtXZM3sgr8ZJxep0sKe14jt3PUdjXj5k1AiTDNhMcYi7KoTDk7NyFih+spafysmzvi4jI8\/\/pimT58eePAs37P5+eef08iRI8vWoxWqvKQ\/z1qwJM03n7+0NJJpYpJUWcA+KdLBfsBfjn+S7PWUJL\/ttHdX0\/J14UbDvEJMrxfjn9k6dyzpWshafysmznjnZZ8+fdTas6CHd3Aef\/zxagMA79ZM45O1YElTHSTZSKbpvdNQFrCXrQXwl+Pvir2elhw\/dyUt+HRLpNEwpqKnJy8\/7iBqVm\/fohkVM6nRrPW3IuLs66+\/pu7du6tF\/vfeey8dcsghatF\/69at1a0BkydPpgkTJqgDaI899liTehNJk7VgEYGcw6mrRjJN75jWsoC9bM2Avxz\/uOx5OvK73bvpmUVr6aM1X0cWYVqInXl4bbrix\/UrlAjLVbtZ629FxJk+hJaP0bjhhhtUXfzhD3+gL7\/8Uk1j8sOjatWqVaNhw4btcYOA3FezvOesBUtauHM54jaSaXqXYisL2MvWGPjL8c\/H3rson0u4aOUW+tsH6+izDf8x3iUZ9GZ6NOxnR9elFv89eV\/iKAs56t97zlp\/KyrObrzxRrrssssU+EmTJtGMGTNo3Lhx6nJz\/v9jjz1Gjz76aNml6NLB4feftWBJE390UHK1AfZy7PGLSXrYv7Viq9oVybsj4wowPRLGa8M6n9yADq5ZLROjYWFqM2v9rYg427x5s5rGPO6442jAgAGqfl5++WU1esairEmTJkqksTCbNm0aNgSEieCMpIVAkKtosJdjD3GWHHueguTnvc+30Iv\/F38ETJdcj4S1a3UgnVd6gPpxFkfCwtYkxFlYYhHT33333eoA2v79+1OHDh3U+rNOnTqpGwN+9rOf0cCBA4l3dOrrmyK6cZota8HiFGZI4xAIIYFZTA72FmFGMAX+EaAFZNG7IT9et43mLd8c+mywfKXQAuys5nXopKY1VdJi3SVph3Z8K1nrb0VGzriaePSM15vx3yzAatWqRUOGDFEbAViUVapUiW655Rbq3Llz\/Fp1ZCFrweIIYySz6KAiYbOSCeytYIxsBPwLo9Prv3TK595bSy9aWP\/l9awFWMvGNeiCo+qWjYBhFKxw\/URJkbX+VkycceWwCNuyZQvVqFFDiTG+xumNN96g1157jc4\/\/3w17ck\/T+uTtWBJUz2gg5KrDbCXY8+ewf8H\/nr0663lG+kfH22wsvYrUIA1qkFHNyihetV3U\/VdW+mko5tS5cqVZQMhY96z1t+KirN8scVCbdOmTVSzZk3iuzbT+GQtWNJUB+ig5GoD7OXYZ0mceQ9hnfmvdbTwsy1OxBcz5YX4vzy1IdUtqZp3+hGxLxf7WetvRcSZPkpj8ODB1KZNm8Dafv7552no0KHYECD3XUi1ZzSSctUD9nLsK4o48wqvj778mt79ZLM6biLMfZCmtaCnH396RB06ocn367+iXuaN2Delbj8dxJl9psoiX8fEU5bbtm2jjRs30j333EOXXnoptWzZcg+PnJbXoXHaqVOnUr169RyVKp7ZrAVLPFp2c6ORtMszjDWwD0PLftpi4K\/XfL20+Cta8NkW+uSr7dZHvbTI0iNfZzavTSc3LX9pt+31X8XA3n7EpcNi1vrbREfOJk6cSLfeeqtaa1bo4anMfv36qSM30rruLGvBUqjOkvwcjWSStMv7Ans59mkYOdNHTCxbu42WrP6a\/m\/VVifCyy++TmxSkw47cF81BalFl23xVahmEfuFCLn7PGv9baLijO\/R5HVka9asoR49etCvf\/1rOv300\/eoTb5Xk+\/TTKso0wXOWrC4+9qFt4xGMjwzWznA3hbJaHZc8Peebs+CZ\/Hqr+nlxV8p8WXjgNVcb6qnHA85oDq1a1WP9qn8\/fritB474YJ9tCjIXq6s9beJijMdTrzYf\/369eomAL6iqVifrAVLmuoJjaRcbYC9HPuwI2dadLEI0v\/mka7R\/\/hMvYRr4cU+eKSrwwn16fTDaqsyRF3vJUv9e++IfblayFp\/m5g404KM\/zZ9eGqzTp062K1pCixD6dBIylU22Mux1wLh3Q8\/o4MPPpgOPbCkTHTxYaozP\/jK6TSjfnM9ncjC65IW9eiog0vUR8UsvExqFbFvQslNGogzN1xJ79BcuXKlsYdGjRpht6YxrWwlRCMpV99gnwx771SjPseLPbvY0eh9Iy28TmtWi046tBY1PaB6mfDSAiwZAunzgtiXqxOIM0fst2\/fTnPnzlW7Nk0fnvI89dRTqXr17xuHtD1ZC5Y08UcjKVcbYB+fvT5KYuFnm2nmv75SBl1OMXpFFY92ldbfj84+og7V2KdyuXsdk15gH59kshYQ+8ny9nrLWn+b2LSmXJW685y1YHFHMrxlNJLhmdnKAfY\/kNRrqPgn3rO75q3YRLM\/XJ+I6PIKr0PqVKdWjWvscaE2RJed6Efs2+EYxUrW+ltxcbZhwwZ6\/fXX6d133yXepdm6dWs6+eST1W7NtD9ZC5Y01QcaSbnayBp7Lbq+\/e47+vuS9ercLn5cTy\/6R7uOrL8fXXB0Xdr97Q51VuRxzRurK4QgvJL7LmQt9pMjW9hT1vpbMXHGGwMefPBBGjFihLpT0\/vwRoDevXtTr169lGBL65O1YElTPaCRlKuNisJei67PN35\/Mv3ydW4OSg2qKe+C+mb19qVjG5bQEQftp5IWOsOrovCXi+DonsE+Oru4ObPW34qJsyeeeIIGDBhAZ5xxBt10003UpEkTVXcrVqygu+++W90mMGzYMGrbtm3cOnWWP2vB4gxkBMNoJCNAs5QlbeyDphZ37d5N\/\/hoPb2zYrN6a9fruTRar+g6rN6+1KJRDTr8wH2NRJdp9aSNv2m5K0I6sJerxaz1tyLibOvWrdS1a1cqKSmh++67b48F\/7x54IYbblAjamPGjEntWWhZCxa5r+WentFIytVGEuz9govfln\/GZ3S98P5a4jtGkhJc3tEsXkx\/TIMSNdL1ozoyuxiT4C8XXen2DPZy9ZO1\/lZEnOljNbp160adOnUKrO1Jkyapac9p06ZR\/fr15SIij+esBUuaKgGNpFxtRGUfdCAqv8XrSzfQGx9vcnb\/Yi5S3lEu\/nfTutWpYa19RK8HMqnVqPxNbCNNfgJgLxchWetvRcTZ2rVr6corr1QXn\/MIWdDDI2pPP\/00Lj6X+y6k2jMaSbnq8bPXostboqVrt9HTC76kFY4uvM739l7RxYvoj21Ygw6tKzPK5aKWEPsuqJrZBHszTi5SQZy5oOqzuXPnTurTpw8tXryYxo0bR02bNi2XYvny5erC89LSUrVhoEqVKgmUKryLrAVLeELucqCRdMdWW9YL5rXY+eALnlJcpwTX8rVbadXmb90X4r8evIKryQHViUVXnf2qpH6UywUgxL4LqmY2wd6Mk4tUWetvRUbOuOLef\/996tKlizqU9rzzzlOHzfLDB9W+9NJLap0ZT2vy0RppfbIWLGmqBzSS0WrDexZX4zr70Iz31tLMD9YpY0mu4WJ\/\/mnF1o1r0H7VKpcTXN500d644uVC7MvVKdjLsc9afysmzriKP\/zwQxo0aBAtXLiQdu\/mJb5ElSpVolatWtEdd9xBRxxxhFwkGHjOWrAYIEksCRrJH1B7BddHa75WImv+J8nuUtSl8QuuoxuUUO19vx\/h8gotnM0V\/auC2I\/OLm5OsI9LMHr+rPW3ouJMVxOPnvFhtPzw4bM8alYMT9aCJU11UpEbSe90Iv+b71VctXEH\/fvLbYmPbvlHuJoftB+xzjqy1i6qW7deuUNQIbiS+YZU5NhPhmB0L2AfnV3cnFnrb0XEGW8IeOihh9QZZs2bNyc+dLYYn6wFS5rqqNgaSe\/o1rZvdtHLS76ixV98rZAmPZ3oF1xHHVxCP2lWkw7Yr+oep80HCa5iY5+muLVRFvC3QTGaDbCPxs1Grqz1t2LijHdr8oGzBxxwgBJp1157LTVq1EhNaxbLk7VgSVO9SDeSXrHFXD5Zv53eWLZRncUlIbb8guukpjXplENrUrXKexsJrjB1K80+TFkrYlrwl6tVsJdjn7X+VkSccfV+88039Oabb9KECRPU3\/x\/3rWpj9ioV6+eXBQYes5asBhiSSSZi0bSK7j+s3OXOntr\/iebiJdDSggu7\/qtQw6oTrxgXl\/x4xVjSU8numCfSNBUECfgL1eRYC\/HPmv9rZg481axX6jxURt8jAbv5rz44otTe79m1oJF7mu5p2fTRtIruHh0a97yTfRxgnco+kvuFVw8unVik5q0b9Xvp\/W9IitpwRWmbk3Zh7GJtOYEwN+cle2UYG+bqLm9rPW3qRBn3urZvHkzPfzww2pErUaNGrghwDx2M5OSBddHqzbQxo0baTOV0IKVX4scduoVVLwbkUe3TmiyP53dvI6a3tSfp1loRQkadFBRqNnLA\/72WIa1BPZhidlLD3Fmj6WxJb5rc\/bs2UqIcQXw06JFCzVyxmegVa1a1dhWkgmzFiyu2b6+dGOZi7nLNpD+\/9xlP\/zcdRm8QooFF49snXZYLaqy915FM7LlmhE6KNeE89sHfzn+YC\/HPmv9rdjImV+Q8SXnvObsF7\/4BV100UXqSI0oD988MH369LKsfHF6mzZt8ppatGgRde7cmbZs2VIu3Yknnkjjx49XF7QHPVkLlij1ofN4pxdnf7ReTS8mtY7Lf\/YWj3D95NBaZa\/D4guPOQF0UOasXKQEfxdUzWyCvRknF6my1t+KiDN98fnKlSuJF\/5fffXV6p7Nhg0bxqpTFmbz588vmwqdNWsW9ejRg\/r27Uu9evXKaZvT9evXjyZOnEgtW7Y0LkPWgiUfGD2Nx3+\/88km+vuS9U7FV4P9K6viNK1Xoka0Dj9wXzpo\/2qZvM7HOGAtJUQHZQlkRDPgHxGchWxgbwFiRBNZ629FxBkfODt58mQ6\/\/zz6dBDD7VyfIYe\/Ro2bFi5kTIWbKtWrco7AjZq1CiaM2dO3jQYOStPgKcc+XLrJ\/+5hmxOO3pHuXhnYstGNahpwKXVaCQjtnAWsoG9BYgxTIB\/DHgxs4J9TIAxskOcxYCXxqwm4ozT8MOXrId5shIsPBr2+cYddMeLH8cSYl7hddyP9leXV\/O6Lv3zMAvn0UiGiVS7acHeLs+w1sA\/LDF76cHeHsuwlrLS32ouIiNnYSslanoeERs+fDjlW3emp1h5XdmSJUvKXLVr166gWNPB8vjjj6sDdPmpX79+1OKmIt+qzd+qcjy1YA29smR9aDGmpxsvOKouXdzi+7PqXKzpQiMpFy5gL8eePYO\/HH+wT5Y998\/64WVQHTt2VLNcur9NtjTJequQ4kyvNWOUhUSWng4955xzysSYFmwNGjQw2hDgrTLeWHDNNdckW4sxvbEg+2LLtzRj8Vb1x+TRIqzdUSXUqsE+dHCNyqR\/ZpI\/bhq+j5WvAWMxXLny9+vP8CRDAOyT4ZzLC\/jL8Qf7ZNk\/8sgjai2494E4S7YOnHjjHaFdu3ZVa874mI4wo1pa4OUbddMjZ7zOTW9mYB9h\/Dh5cUOjPF157+xP6fF3vyqYg6cc27aoRz9tXktNQyYpxIIKt337dlqzZo36DQrirGD1WU0A9lZxhjYG\/qGRWcsA9tZQGhnigRI9evb222\/TyJEjMXJmRK4IEumRse7du+fdsel\/FZN8xTgHro+0GDpzed4pSxZg3U5vRC0a1nAyLRk3dDC9EJdg9PxgH52djZzgb4NiNBtgH42bjVzF2N\/GeW+Rac1NmzbR6NGj1dVMxxxzTGD53333XbrnnnvUVGOcezZNRFZQAUzyFVOwaFF2\/ZTFZafX+9+bBdlVJ9Snq044eI\/LsuMEmYu8aCRdUDWzCfZmnFylAn9XZAvbBfvCjFylKKb+1gYDEXGm13QNHjw48IBYPpCWF\/I\/\/\/zzxtORuc4qKzQ9metzk7PPiiVY+NiLXKKMBdk97Y+kQ+tWT70g8wY8GkkbX\/9oNsA+GjdbucDfFsnwdsA+PDNbOYqlv7X1vomJM77cfNCgQfTUU08Zl\/2nP\/0p3XfffVS9evWCefT6Mk6oT\/XXo198iXquk\/6D1qUFbRIIKkAxBMslDywInL5kUXZ\/h9JUTlkWrGzsWDNB5CwNOihnaI0Mg78RJieJwN4JViOjxdDfGr2IYaLExBmXZ\/ny5cS7L\/gQWr5Ls1WrVoG3AvCF561bt6bTTjtNXX4e5vFf3+S\/HYCP1xg7duwetwEUyldM4oynMPlg2N6TF+9RbBZlz\/ZqXVSjZEHs0UiG+VbYTQv2dnmGtQb+YYnZSw\/29liGtQRxFpZYhPQma84imE08SxqDhYUZizL\/qf3FPlLmr1w0komHe5lDsJdjz57BX44\/2MuxT2N\/65JGoiNnLl9EwnbagoWFWdtRC8ot+K9ookzXMxpJiYj\/3ifYy7EHf7CXJSDnPW39rWsSiYkzPVrGL\/Tzn\/+cHnvsMeKf5Xtq1qxJPXv2JP47jU+agmXyO6v3mMasKFOYQXUPgSD3jQB7OfYQZ2AvS0DOe5r62yQoJCbO9A5Nfile5H\/DDTcQX8eQ7+EDRsMeHpsENO0jLcESJMzuaHc49Tjj+yulKuIDgSBXq2Avxx7iDOxlCch5T0t\/mxSBxMRZUi+UpJ80BAsfJjt05oqy166o05j+eoVASDLSy\/sCezn2EGdgL0tAznsa+tsk3x7iLAZt6WDJqjBDBxUjaC1khTizADGGCfCPAS9mVrCPCTBGdun+NkbRI2UVEWd6\/RnWnEWqM5WJD5blxf\/6ycqImX5fNJLRYyduTrCPSzBefvCPxy9ObrCPQy9eXoizePyMcuv1Z1hzZoRrj0S8K7PVkDfLCbOKcHZZGBpoJMPQspsW7O3yDGsN\/MMSs5ce7O2xDGsJ4iwsMYvpd+\/eTevWraMpU6bQM888o+7WzHX3pkW3kU1JBUudPrMzO2KGkbPI4WotIzooaygjGQL\/SNisZAJ7KxgjGZHqbyMV1kImkWnNQuVmkXbLLbeomwT44vMqVaoUyiLyedLBEnTA7ANXlarLyrP2oJGUq3Gwl2PPnsFfjj\/Yy7FPur+Ve9PvPadSnHHBpk6dSg888ACO0vBEyKS3v6BfTV1S9pMpXVtQm6MOkI4hEf9oJEWwK6dgL8ce\/MFeloCcd4gzOfZlnvUl6R988AE9+uijVLdu3RSUas8iJBksQevMFg4+JZVckigUBEISlIN9gL0ce4gzsJclIOc9yf5W7i1\/8CwyclZot+bHH39M8+fPp86dO9Pvf\/97qlSpUhpY7VGGpILFfy1TRT7537SiIRBMSdlPB\/b2mYaxCP5haNlNC\/Z2eYaxllR\/G6ZMLtOKiLNCuzWrVq1Kl112GQ0cOJD2339\/l+8fy3ZSwTLhzVXU568flpU1q+vMvJWFRjJW6MbKDPax8MXODP6xEUY2APaR0cXOmFR\/G7uglgyIiDNLZRc3k0Sw+KczT21Wi2b0bi3+7tIFQCMpVwNgL8eePYO\/HH+wl2OfRH8r93Z7ek6VOONdmlu2bKEaNWqkdirTizCJYLnkgQU0d9lG5RbTmT\/QRyMp14yAvRx7iDOwlyUg5z2J\/lbu7VIkznjR\/5\/\/\/GfiRf+jRo2ikpISdXRGly5daNWqVfTb3\/6WLrnkklSLNNfB4r\/QHNOZEGdpaDwgzmRrAfzl+IO9HHvX\/a3cmwV7Fhs5e\/DBB2n48OFqbdngwYOVONu+fTvNnDmTxo8fT0uXLlVnnF144YVpY1ZWHpfB4j\/TjEfNsrw70x8EaCTlvhZgL8ceI2dgL0tAzrvL\/lburXJ7FhFneoTs8MMPp9tvv32PQ2Z5VK1Pnz7qtgAWaizc0vi4DBb\/qBlfz3TaYbXSiEGkTBAIItiVU7CXYw\/+YC9LQM67y\/5W7q1SJs70bs3evXvTlVdeGVi6p556Sl3fNG3aNKpfP50n4LsMFr47k0fP+MGo2Z4hAoEg15yAvRx7iDOwlyUg591lfyv3VikTZzwidvXVV9NZZ51FAwYMCCzd0KFD6dVXX83kIbT+UTOezmSBhucHAhAIctEA9nLsIc7AXpaAnHeIswTY867M2267jZ577jm66667lEjTB83yZyzK+vfvTxdffHHmDqHFWjOzAIRAMOPkIhXYu6BqbhP8zVnZTgn2toma24M4M2cVK+XKlSupW7dutGTJEnXQLB+fwc+OHTvUWrMjjzySeNNAo0aNYvlxmdlFsLy+dCO1HbWgrNgYNQuuQTSSLiM7v22wl2OPkTOwlyUg591Ffyv3NoU9i2wI0MXi3Zm8poz\/bN68Wf2YhVr79u3Vn+rVqxd+A8EULoLFf64ZdmhCnAmGeKBriDPZGgF\/Of5gL8feRX8r9zaFPYuIM566fOaZZ+joo48m3rFZrI\/tYMGomXkkoJE0Z2U7JdjbJhrOHviH42UzNdjbpBnOlu3+Npz35FOLiLO1a9eqXZqXX3459erVK\/m3tuTRdrD85q8f0iNvrlKlww7N\/JWERtJSEEcwA\/YRoFnMAv4WYYY0BfYhgVlMbru\/tVg0J6ZExJnerck3AECcfV+v\/js0\/3Tp4dTt9PSut3MSjSGMopEMActyUrC3DDSkOfAPCcxicrC3CDOkKYizkMCiJn\/55ZfpD3\/4gxpBu+iii2jffffdw9Tee+9NderUIf47jY\/NYPFPaeLQWYycpTHmuUzooGRrBvzl+IO9HHub\/a3cW5h7Fhk504fQ8o7NfA\/v1MzKIbTejQCnNqtFM3q3Nq\/FDKZEIylX6WAvxx7iGOxlCch5hzhLgD3v0pw7d646NiPfU61aNTr11FNTu2vTVrD4pzQxalY4CCEQCjNylQLsXZE1swv+ZpxcpAJ7F1TNbNrqb828yacSGTmTf207JbAVLLwJgDcD8IONAGZ1g0bSjJOLVGDvgqq5TfA3Z2U7JdjbJmpuz1Z\/a+5RNiXEWQz+NoKFR8340Fl9j2bf8w6hQT87NEapspEVjaRcPYO9HHv2DP5y\/MFejr2N\/lau9OE9JybO9DozLuJ9991HN9xwA2HNGRE2AoQPWnRQ0ZjZyoUOyhbJaHbAPxo3G7nA3gbFaDYgzqJxK5hr06ZNNHr0aJXu5z\/\/OT322GPEP8v31KxZk3r27En8dxofG8Fy+4sf090vfaJeD1Oa5rWMRtKcle2UYG+baDh74B+Ol83UYG+TZjhbNvrbcB5lUyc2cib7mm68xw0W\/0aA\/uc3oYHnN3VT2ApmFY2kXIWCvRx79gz+cvzBXo593P5WruTRPIuJM77C6cUXX6Q5c+bQ7373O9pvv\/3U\/Zp8KG2VKlVo4MCBdMQRR0R7q4RyxQ0WXNcUvaLQSEZnFzcn2MclGC8\/+MfjFyc32MehFy9v3P42nvfkc4uJsxdeeIH69OlDLVu2pDFjxlDt2rXVNOddd91FM2bMID5G48EHH6TWrdN73lfcYBk6czkNnblC1TqmNMMFPxrJcLxspgZ7mzTD2wL\/8Mxs5QB7WyTD24nb34b3KJtDRJxt3bqVunbtqs4v480BJSUl5Shs2LCBevTooW4NYOHGQi2NT9xg8R48e9UJ9emBq0rT+JqpLBMaSblqAXs59uwZ\/OX4g70c+7j9rVzJo3kWEWd652a3bt2oU6dOgSWfNGmSGjmrqDcE+NebLRx8iho9w2NGAI2kGScXqcDeBVVzm+Bvzsp2SrC3TdTcHsSZOavIKdeuXavu1Lz00kvVkRpBz6hRo+iJJ56gqVOnUr169SL7cpkxTrBMfmc19Z68WBUPU5rhawmNZHhmtnKAvS2S0eyAfzRuNnKBvQ2K0WzE6W+jeZTNJTJytnPnTrXebPHixTRu3Dhq2rT8DsXly5fTddddR6WlpTRixAi1QSCNT5xgwV2a8WoUjWQ8fnFyg30cevHzgn98hlEtgH1UcvHzxelv43tP3oKIOOPXfP\/996lLly60ZcsWOu644+iQQw5Rb79u3Tp67bXXqEaNGvTQQw\/RsccemzwVQ49Rg8U\/pclrzXjNGR5zAmgkzVnZTgn2tomGswf+4XjZTA32NmmGsxW1vw3nJT2pxcQZI\/j888\/pzjvvpJdffpm++eYbRaVq1ap07rnn0s0330wNGzZMD6mAkkQNFlx0Hr9a0UjGZxjVAthHJWcnH\/jb4RjFCthHoWYnT9T+1o735K2IirPkX9eux6jBwmvNeM0ZP1hvFq1O0EhG42YjF9jboBjdBvhHZxc3J9jHJRg9f9T+NrpH2ZwQZzH4Rw0WHKERA\/p\/s6KRjM8wqgWwj0rOTj7wt8MxihWwj0LNTp6o\/a0d78lbgTiLwTxKsGBKMwZwT1Y0knY4RrEC9lGo2csD\/vZYhrUE9mGJ2Usfpb+15z15SxBnMZhHCRYcoREDOMSZHXgxraCDigkwZnbwjwkwRnawjwEvZtYo\/W1Ml6LZIc5i4I8SLDhCIwZwiDM78GJaQQcVE2DM7OAfE2CM7GAfA17MrFH625guRbNDnMXAHyVYWg15k3hqkx8coREdPhrJ6Ozi5gT7uATj5Qf\/ePzi5Ab7OPTi5Y3S38bzKJsb4iwG\/7DBgvVmMWD7sqKRtMcyrCWwD0vMbnrwt8szjDWwD0PLbtqw\/a1d78lbExVnfBPA3\/72N\/r0008D37xmzZrUs2dP4r\/T+IQNlteXbqS2oxaoV8ERGvFqFI1kPH5xcoN9HHrx84J\/fIZRLYB9VHLx89e75MwAACAASURBVIXtb+N7lLUgJs5eeOEFdYWTPnw2CEOjRo0q1MXn3vVmEGfxAh+NZDx+cXKDfRx68fOCf3yGUS2AfVRy8fNBnMVnWNDC1q1bqWvXrvTll1\/SvffeS0cffTRVqlSpYL60JQgbLF5xdk\/7I6jzyQ3S9kpFUx40knJVBfZy7Nkz+MvxB3s59mH7W7mS2vEsMnK2evVqat++PV199dXqgvNifcIEC9ab2a1lNJJ2eYaxBvZhaNlPC\/72mZpaBHtTUvbThelv7XtP3qKIONuwYYO69PzCCy\/MjDjzrjfjal44+BS17gxPNAJoJKNxs5EL7G1QjG4D\/KOzi5sT7OMSjJ4f4iw6u1A5H3zwQXrxxReJ\/65Xr16ovGlJHCZYhs5cTkNnrlBFx3qz+DWIRjI+w6gWwD4qOTv5wN8OxyhWwD4KNTt5wvS3djzKWhEZOdu+fTu9+uqr9PDDD9OSJUvojDPOoBo1auxBoiLt1sThs3YDHY2kXZ5hrIF9GFr204K\/faamFsHelJT9dBBn9pnuYVGvOVu5cmVebxVpt2adPrPL3nXA+U1owPlNEyBdcV2gkZSrW7CXY8+ewV+OP9jLsYc4k2NfdJ5NgwWbAexXLRpJ+0xNLYK9KSk36cDfDVcTq2BvQslNGtP+1o335K2KTGsm\/5puPJoGCw6ftc8fjaR9pqYWwd6UlJt04O+Gq4lVsDeh5CaNaX\/rxnvyVkXF2bJly2jIkCH05ptv0oEHHqgOnN13331p4MCBdMEFF9All1yS2PlnfCDu9OnTy2pgzJgx1KZNm7w1Yhos2AxgP7DRSNpnamoR7E1JuUkH\/m64mlgFexNKbtKY9rduvCdvVUycvf\/+++o4jb322ouaNGlCX3zxhRJn1apVUz9fvHixOqC2kECygYyF2fz588tuI5g1axb16NGD+vbtS7169crpwjRYvJsB\/ve4g2hcp6NsFDvTNtBIylU\/2MuxZ8\/gL8cf7OXYm\/a3ciW061lEnO3cuVNd3cQbAvgojQULFqgRNBZn9evXp82bN6vzz3gUjUewWLC5ehYtWkSdO3emYcOGlROCXL5Vq1bR+PHjqaSkJNC9SbD415thM4CdmkQjaYdjFCtgH4WavTzgb49lWEtgH5aYvfQm\/a09b\/KWRMTZ2rVr6corr6TLL79cjUzxSJVXnDGWcePG0aOPPip2t6YrcfZsr9Z02mG15Gu+yEuARlKuAsFejj1GzsBeloCcd4izBNjrozS6detGnTp1ChRnkyZNUqNqejQtgWKVuRg1ahQNHz5cjdrlm1bVwfL4448TH\/vBD4\/8eZ+3VmyltqMWqB812L+yuhkAT3wCEAjxGUa1APZRydnJB\/52OEaxAvZRqEXPw1pBPzzT1rFjR5ozZ05Zfxvdcvpzioyc6YvP69atSyNGjFAH0gZNa1apUoXGjh1L++23XyIk9VozdtauXTtVtnyPFmfeNDxFes0115T96OkPttCQv39VJs5mXPO9iMMTj8COHTuIR2BZDFeuXDmeMeQORQDsQ+Gynhj8rSM1Ngj2xqisJHzkkUdo4sSJ5WxBnFlBm9vICy+8oBbc88jZIYccQqNHj1YjVayO77\/\/frUhYOjQoWrqM+lHi0dec5Zv5E6LM16v1rBhQ1VMFgve0bNLx75Pb3+yVX12\/lF16OFORyT9OhXSH98ysWbNGvUbFMRZslUM9sny9nsDfzn+YJ8sex4506Nnb7\/9No0cORIjZ66rYPfu3UoRs7DZtm1bOXd777232jDA0578b4lHbxTo3r17zh2bJnPguBnATe1hesENVxOrYG9CyV0a8HfHtpBlsC9EyN3nJv2tO+\/JWxaZ1vS+Ju\/MnDdvnvrzzTff0I9\/\/GM67bTTqHbt2snT8Hi0Ic5wM4C7KkQj6Y5tIctgX4iQ28\/B3y3ffNbBXo49xJkcexHPvM6sX79+ahSvZcuWZWXQ68\/ybQooFCzemwHYMG8G+FGdfUTes6I5RSMpV6NgL8eePYO\/HH+wl2NfqL+VK5kbz6IjZzy1+dFHH9GTTz5JW7ZsUW\/ImwQuu+wyato0mYvB9foy9q3PNNOjZqWlpbHOOcPNAG6CFh2UO64mltFBmVBylwb83bEtZBnsCxFy9znEmTu25SzzdOZvf\/tb4o0BLNK8T6VKlahDhw50yy23UNWqVRMpkf\/6pkK3A3ChCgVLx7+8T3\/7YJ0q\/6nNatGM3q0TeZcsOEEjKVfLYC\/HHr+YgL0sATnvhfpbuZK58SwycsZijDcCPPTQQ9S7d2919MT++++v3pBFGx9Ay2ec3XTTTWpTQFqfQsHivbYJNwPYrUUIBLs8w1gD+zC07KcFf\/tMTS2CvSkp++kK9bf2PcpaFBFn69ato6uvvppOOOEEuvXWW\/e43JzFG4+a8f2bLOCkNwfkqqJCweLdqfnAVaV01QnlD6iVrfri9o5GUq7+wF6OPUbOwF6WgJz3Qv2tXMnceBYRZ\/qGAB4142ucgp4ZM2ao0TWJGwJMUecLFv9OTWwGMKVqlg4CwYyTi1Rg74KquU3wN2dlOyXY2yZqbg\/izJxV5JR8yvINN9yg1pPxKfz+dWV8pMagQYPUFOd9993n9OLzyC9RYM2Zf6fm+hFnx3GFvD4CaCTlQgLs5dhj5AzsZQnIeYc4S4g93wTA68kaN25M\/fv3pyZNmtBee+2lTgPmGwL4KIs777yTjjnmmLIS8YG09erVS6iEhd3kCxavOOPjM3CnZmGeYVJAIIShZTct2NvlGdYa+IclZi892NtjGdYSxFlYYhHS62lNFmhhHr6qh+\/VSsuTL1i8mwFOO6wWPdsLOzVt1hsaSZs0w9kC+3C8bKcGf9tEze2BvTkr2ykhzmwTDbDH95PNnTuXeHozzFOtWjU699xzw2RxmtZUnGGnpv1qQCNpn6mpRbA3JeUmHfi74WpiFexNKLlJA3HmhmuFtJovWLBT022Vo5F0yzefdbCXY8+ewV+OP9jLsYc4S5j9hg0b6PXXX6d3331XbQxo3bo1nXzyyak9PsOLJ1ew4E5N90GERtI941wewF6OPcQZ2MsSkPMOcZYQ+127dqmDZnm3Jv\/b+\/DCfz5mo1evXondEBDltXMFC3ZqRqEZLg8EQjheNlODvU2a4W2Bf3hmtnKAvS2S4e1AnIVnFinHE088QQMGDKAzzjhD3QTAuzX5WbFiBd199930xhtvqHPO2rZtG8l+EplMxBl2arqpCTSSbriaWAV7E0ru0oC\/O7aFLIN9IULuPoc4c8e2zLK+bLykpESdY1a9evVyXnnDAJ+DxiNqY8aMKbpzznpPXkyT31mt3gl3aroJKDSSbriaWAV7E0ru0oC\/O7aFLIN9IULuPoc4c8e2zLI+SoPPOevUqVOgx0mTJqlpz2K8IcB7jAZf2cRXN+GxSwCNpF2eYayBfRha9tOCv32mphbB3pSU\/XQQZ\/aZ7mFx7dq16tqmSy+9VI2QBT08ovb000\/T1KlTU3XwrLesuYKl1ZA3iTcF8INjNNwEFBpJN1xNrIK9CSV3acDfHdtClsG+ECF3n0OcuWNbZnnnzp3Up08fWrx4MY0bN46aNm1azuvy5cvpuuuuo9LSUrVhoEqVKgmUKryLoGDBnZrhOUbJgUYyCjU7ecDeDseoVsA\/Krn4+cA+PsOoFiDOopILme\/999+nLl26qINozzvvPDr11FOVBT6c9qWXXlLrzHhak4\/WSOsTFCz+nZp8MwDfEIDHLgE0knZ5hrEG9mFo2U8L\/vaZmloEe1NS9tNBnNlnmtPihx9+qC44X7hwIe3evVulq1SpErVq1YruuOMOOuKIIxIsTXhXJuIMF56H52qSA42kCSU3acDeDVdTq+BvSsp+OrC3z9TUIsSZKSmL6Xj0jA+j5ad27dqp3Z3pf+WgYOFdmrxbkx8co2ExSHym0Ei6Y1vIMtgXIuT2c\/B3yzefdbCXYw9xJse+6DwHBYt3pyaO0XBXpWgk3bEtZBnsCxFy+zn4u+ULcSbHN59niLN01ksqS1VInGGnprtqQwfljm0hy2BfiJDbz8HfLV+IMzm+EGc\/EKi0Wy\/2Smd9pLpUQeIMx2gkU2XooJLhHOQF7OXYs2fwl+MP9nLsMXImx77oPPuDBReeJ1eFaCSTY+33BPZy7CHOwF6WgJx3iDM59kXn2R8sOEYjuSqEQEiONcSZHGuMXIJ9ugjIlQbiTI590XkuJM5wjIa7KoU4c8e2kGWwL0TI7efg75ZvPutgL8ce4kyOfdF59gfL0JnLaejMFeo9cIyG2+pEI+mWLzooOb6FPCP2CxFy9znYu2NbyDLEWSFC+LyMgD9Y+HwzPueMHxyj4TZQ0Ei65QtxJse3kGfEfiFC7j4He3dsC1mGOCtECJ\/nFGc44yy54EAjmRxrvyewl2PPnsFfjj\/Yy7GHOJNjX3Se\/cGCYzSSq0I0ksmxhjiTYx3kGbEvVx9gL8ce4kyOfdF59gdLnT6zy97hgatK6aoT6hfdOxVLgdFIytUU2Muxx8gZ2MsSkPMOcSbHvug8e4Plu33rEo+c6efZXq3ptMNqFd07FUuBIRDkagrs5dhDnIG9LAE57xBncuyLzrM3WFb8p4TajlpQ9g4LB5+idmzicUMAAsENVxOrYG9CyV0a8HfHtpBlsC9EyN3nEGfu2FY4y7lGznCMhvuqRiPpnnEuD2Avxx4jZ2AvS0DOO8SZHPui8+wNlsc+2IkzzhKsQQiEBGH7XIG9HHuIM7CXJSDnHeJMjn3RefYGy52vbcEZZwnWIARCgrAhzuRgB3hG7MtVB9jLsYc4k2NfdJ69wdLzmbU0d9lG9Q68S5N3a+JxRwCNpDu2hSyDfSFCbj8Hf7d881kHezn2EGdy7IvOszdYLp7wGX26\/j\/qHXCMhvuqRCPpnnEuD2Avx549g78cf7CXYw9xJse+6Dx7g6XFiH+XlR\/izH1VopF0zxjiTI4xRm\/APp0E5EoFcSbHvug862CZNH0W8ciZfnDGmfuqhDhzzxjiTI4xxBnYp5OAXKkgzuTYF51nHSx3Pvws8Zoz\/eCMM\/dVCXHmnjHEmRxjiDOwTycBuVJBnMmxLzrPucTZ+hFnF927FFuBIc7kagzs5dizZ\/CX4w\/2cuwhzuTYF51nHSwX\/XYi8Tln\/OAA2mSqEY1kMpyDvIC9HHuIM7CXJSDnHeJMjn3ReQ4SZ6c2q0UzercuuncptgJDIMjVGNjLsYc4A3tZAnLeIc7k2BedZx0stTqMJL5bkx+Is2SqEQIhGc4YOZPjnMszYl+uTsBejj3EmRz7ovMcJM4GnN+EBpzftOjepdgKjEZSrsbAXo49Rs7AXpaAnHeIMzn2RedZB8vmNkPpu33rqvJDnCVTjRAIyXDGyJkcZ4ycgX36CMiVCOJMjn3RedbBsrHdX8rKjjPOkqlGiLNkOEOcyXGGOAP79BGQKxHEmRz7ovPMwdKh66+IR870A3GWTDVCnCXDGeJMjjPEGdinj4BciSDO5NgXnecgcYYDaJOpRoizZDhDnMlxhjgD+\/QRkCsRxJkc+6LzzMHS\/le30tbT+peVHQfQJlONEGfJcIY4k+MMcQb26SMgVyKIMzn2ReeZg+Wy26bSf45sq8qOA2iTq0KIs+RY+z2BvRx79gz+cvzBXo49xJkc+6Lz7BdnOOMsuSpEI5kca4gzOdYYuQT7dBGQKw3EmRz7ovPMwdJ21EL6tu4RquwQZ8lVIcRZcqwhzuRYQ5yBfboIyJUG4kyOfdF59ouz3154KN107iFF9x7FWGCIM7laA3s59pjWBHtZAnLeIc7k2BedZw6Wiyd8hgNoBWoOAkEA+n9dgr0ce4gzsJclIOcd4kyOfdF55mC5cNr2snI\/cFUpXXVC\/aJ7j2IsMASCXK2BvRx7iDOwlyUg5x3iTI590Xn2izMcQJtcFUIgJMfa7wns5dhDnIG9LAE57xBncuyLzjPEmVyVQSCAvRwBWc+IfTn+YC\/HHuJMjn3RefaLMxxAm1wVopFMjjVGzuRYB3lG7MvVB9jLsYc4k2NfdJ694gwH0CZbfWgkk+Xt9Qb2cuwxrQn2sgTkvEOcybGP7Xnr1q3UtWtXmjdvXpmtvn37Uq9evfLaXrRoEXXu3Jm2bNlSLt2JJ55I48ePp5KSksD8EGexqyyyAQiEyOhiZwT72AhjGQD\/WPhiZQb7WPhiZYY4i4VPLvPq1aupffv21KBBgzJBpUXXOeecQyNGjMhZuFmzZlG\/fv1o4sSJ1LJlS+OX8IozHEBrjM1KQjSSVjBGMgL2kbBZywT+1lCGNgT2oZFZywBxZg1lsoZyCaxRo0bRlClTaNq0aVS\/fvAxF5xmzpw5eUfJgt4G4izZOvZ6QyMJ9nIEZD0j9uX4g70ce4gzOfZOPLPwGjt2bN5RsT59+ijf+UbXComzAec3oQHnN3XyDjC6JwE0knJRAfZy7Nkz+MvxB3s59hBncuydeGbhNX\/+\/JwjZ3o6lNeVLVmypKwM7dq1KyjWvCNnEGdOqi+nUTSSyfLGqKUcb79nxL5cXYC9HHuIMzn21j3zVGePHj0o36aAoHVpQevXCo2c3Xx6DXU7QK6pU+svl3GDaCTlAgDs5dhj5AzsZQkk6537Yv2sXLmSOnbsqJYgNWrUKNmCCHirtHv37t0Cfp271KKrtLQ09FoyLpwWdmPGjKE2bdoEltc7clby+l1Ued2HatfnNddc4\/z9su5gx44dtHbtWiWGK1eunHUcib4\/2CeKew9n4C\/HH+yTZf\/II4+oJUneB+Is2Tqw6i2uMOPCaBvdu3fPeRSHV5yN\/p96dEi1rUosYPTManUGGtu+fTutWbNG\/QYFceaet9cD2CfL2+8N\/OX4g32y7HnkTI+evf322zRy5EiMnCVbBfa86RGvQmeUFfIYVpwtHHwK8UG0eJIhsGLFCpowYYI61y4LQ9zJUDXzAvZmnFylAn9XZAvbBfvCjFylwJozV2QTsKuFmclifl2cXNOXJmefeUfOcHVTAhXscZG1L2qydPN7A3vZ2gB\/Of5gD\/ZJEagwa85MD5z1g9W3CqxatapsR6epLS3OcHVTUuH6gx80kskz1x7BXo49ewZ\/Of5gD\/ZJEagw4oyPzJg+fXpObnphf65zz\/z5Ta590uJsr23riKc18SRHQO\/cefzxxzGtmRx25QnsEwbucwf+cvzBXp49NgTI1UHReOYvaosR\/1a7NHm3Jh4QAAEQAAEQAAE3BE466SSaPHmyG+Mps1phRs6kuLJA4z94QAAEQAAEQAAE3BHgzV9Z2QAGceYujmAZBEAABEAABEAABEITgDgLjQwZQAAEQAAEQAAEQMAdAYgzd2xhGQRAAARAAARAAARCE4A4C40MGUAABEAABEAABEDAHQGIM3dsYRkEQAAEQAAEQAAEQhOAOAuNDBlAAARAAARAAARAwB0BiDN3bGEZBEAABEAABEAABEITgDgLjQwZQAAEQAAEQAAEQMAdAYgzd2xhGQRAAARAAARAAARCE4A4C42MSF+WPm\/ePJWbTyyeNm0a1a9fP4I1ZAki4L3rtEaNGjRx4kRq2bJlXlizZs2iHj16lKVBvUSPrSj8vd5Wr15N7du3pw4dOlCvXr2iFySDOaOw97dJjE3fJ5xBhLFeOQp\/He\/6thi0PbGqIG9mrh9+RowY4c5JCixDnIWsBN0INmjQoCw4OFjmz58PgRaSZa7kzHPVqlU0fvx4KikpoVyX1Xvzc5rhw4eX65DYziuvvGIk7CwVvUKYicLf\/+K6g+vbty\/EWYioiMJeCwNuk7zfGf\/3IUQxMps0Dv\/jjz8efYLjyNHtfLt27SDOHLMuOvNBQgGjBPaqUY9+eX\/rDxLEXo+5Pke9hK+XKPz9XrwjmBBn5nUQlT23SVOmTCn3y2Gh74x5qbKTMg7\/sWPHlvslcNGiRdS5c2fq3r07fjmxEEL+kWGIMwtQK5oJ\/29W+v1y\/byivb\/r9wnqaNhnrp\/nK48WZ97faF2Xv9jtx+WvmXft2lWN4mBa0zwiorDXndYZZ5wBEWCOOjBlFP66bYI4iwk\/T3Yd4zybMmHCBBo0aBB5Z67ceZa1jGnNEPzz\/TaKqc0QIPMkzSVyTaY2\/Wbx22v4OonD3\/v96N+\/P9achcQfhb0Ww4MHD6alS5eqqX1+TNdphixihU4ehT8DCfolEEsq3IRKlkaEIc5CxFC+wIgyshPCdWaS5mogecqhX79+xuvHvL9tYbOGefjE4e\/9DrBHbAgw584po7DXv4Bs2bKFvFM9QWsww5Ume6mj8PdS8m4kOPHEE8vW\/2WPpLs3hjhzx7aoLUOcua++uA2kLqFuKLFjLVydReWvRcKwYcOoTZs2ZaMJmNY05x+FveZeWlpaTgzotoq9600C5iXJZsoo\/JmUXqvmXV8JcewmhiDO3HAtequY1nRfhVGnFoJ+g4UwC19fUfgHfS+wGSMZ9lqcnXPOOXvsXsNofrg6iBP7QSIY65DD8TdJDXFmQimjabAhwG3FR12Uy6Xy7uiBMItWT1H4e6fWgrzizCezuojCPt+mF4gzM+46VRT+mE0JxzhuaoizuAQrcP6gLzBGCexVeNDaMpMvpE6zePFi43Vp9kpdcSxF5e8ngO9E+JiIyj5oM5LJdyZ8CSt2jij8800fY+TMfrxkKa6xISBk\/AQd+IidmiEh5kketJDfZKcmdkfZqYOo\/CHO4vOPyj5oatPkOxO\/xBXLQlT+WHOWXBxAnCXHuig9+Q\/Ew7SN\/Wr07nwKOhbAK4jXrFmjDnzkHWtBD3ZOha+fMPyDri3DyFl45jpHFPb+64NwlEay\/P1T++AfnX++nBBnbrjCKgiAAAiAAAiAAAiAQAECmNZEiIAACIAACIAACIBAighAnKWoMlAUEAABEAABEAABEIA4QwyAAAiAAAiAAAiAQIoIQJylqDJQFBAAARAAARAAARCAOEMMgAAIgAAIgAAIgECKCECcpagyUBQQAAEQAAEQAAEQgDhDDIAACIAACIAACIBAighAnKWoMlAUEAABEAABEAABEIA4QwyAQAUjsGvXLnrmmWdo+\/bt9POf\/zz0233++ec0fvx46tWrF9WrVy90\/jgZXn75ZfrDH\/5Aq1atooYNG9KUKVPU32GffBeCh7WVhvT+GwD69u2r6kfi8Z+GP2bMGGrTpo1EUeATBCosAYizClu1eLGsEoh7dRLfy8iiaNq0aRR0NZMrrps3b6brrrtOCbMePXrQQQcdRKeeeipVr149tMuKKs4OPvhgdVXZ4YcfTs2bNw\/NxUaGDRs20Ntvv03\/\/Oc\/lYiHOLNBFTZAoDwBiDNEBAhUMALFKs5sCiqbttIQHml8H33hN8RZGiIEZahoBCDOKlqN4n0qNIHdu3fTjBkz6O6776aVK1dSlSpVqFWrVnTrrbfSEUccQfkuYOZLg++\/\/36aPn06ffnll1SpUiVq1KgR3XTTTXTJJZeo\/3sv3WaQ7dq1oxEjRiimH374If3+979XIyb8tGjRgvr3708nnXRSQeY8VXrffffRc889R9u2baMGDRrQL3\/5SzXtWrVqVdIdvddQvqk75vDiiy\/Sn\/\/8Z\/r4448Vh1NOOYUGDx5MzZo1Iy1mWrduTXzx\/b333ktr165VfrnM+n3Z3zfffEOPPfYYTZgwQTFl2wceeCB16dKFrr32WlU+fvSI4i9+8Qvl97vvvqPbbruNLrvsMlq2bJn69xtvvEF77703XXXVVXTssceqepk4cSK1bNlS2eCpZvbz0EMP0VdffUUHHHAA\/e\/\/\/i9df\/31VFJSkpNjkDjzXgJ91lln0e23307r1q1TcTBo0CA16sh1muvJNUJqOnIKcVYw7JEABCITgDiLjA4ZQSB5As8\/\/7wSU2eeeSadf\/75tHHjRvrLX\/6iOuHHH3+catSooYTO8OHD6Sc\/+Qn97Gc\/U+Jpv\/32U8Lr1Vdfpcsvv5xOOOEEWr58OU2ePFmJhLFjxyqb7777rhITLDJuvPFGOvLII+nHP\/6xssn5mzRpQp06dVIvPmnSJFq6dKkSbxdeeGFOGCx4WOSwOOK\/DznkECUQ586dS+3bt1cChkXF7Nmz6Z577qFDDz0079QdiycereF3LC0tVeVh8cLlqVOnDj388MNKILFt\/jlPBV5zzTVUrVo1JYq4HMyMGbCtYcOG0bhx45RgO+OMM2jTpk3KFou+W265RZVFizMWxbVr16Zu3bopHz\/96U+VMOT34rrgsuy\/\/\/5KgLEQY+GnxRmLKRZh8+fPL6uDd955h5544gklsFkUcd6gJ584W7x4sSrDFVdcodiyPy47C9J8a8EgzpL\/\/sIjCJgSgDgzJYV0IJACAiyQeBSKxYTuyF9\/\/XUlnIYMGaI646BpTR714gXkPErjXUjO4oBHsHiNl\/65v9PmtWAsPnj9GYsTvQaMhQf75TViLEaChAWLHxY4vEGBhRELPX540wKLKx6x0kLJdOru008\/pY4dOypBw8JQj2y99dZbdMMNN9DNN9+shCmLMx7devTRR6lp06bKr35fXtvGaVkUdu\/eXa3f4pEvFjn8aB886qZHDpkLl\/nOO++kK6+8UqXT78ejeA8++CDxSB0\/XkGqxRkLUh55ZDunnXZaWTRxuVnssX8eqQwrzhYuXFhOIPOaMB71Y8Geq1602AxaW4iRsxR80VGEzBOAOMt8CABAMRFgccTCjMUFj+gE7aYMs+ZMp73oootowIABZSNE3k57wYIFauTp4osvVlNl3odH1Hg0zjt15\/2cxc\/VV1+txNHIkSPLxA+n4REffgceyWPfpuKMd3SykOQp2lwjQ95pTR6N09N7pj70lCGPxOlys2jhEUbvu+Z7P57G5QXznJ6ngHkkkkcmefSMRzj1wyNuXMbTTz+9TAj6YzLfyBmnZT\/eaVEuK8cJC9NjjjkmMMQxclZM33yUNWsEIM6yVuN436ImwKNmupPnF+G1UTx1scLRcwAABaZJREFUySJHjw7lE2c7duxQU14sjObNm0c86sYjX961Zf5OO2g9mBciCx8WLeeee+4ebIPEn07kFxymwolFBwuzXIKQ7eeylevnPArIbP\/1r38pAfXaa68pTjz1qYVPkDjjPDyKxn94JM4vXPv166fKyevgunbtqpjneni0jznyFHQYccbr6PTons5nsh4M4qyomwIUvoITgDir4BWM16t4BHgqjYXD1KlTaebMmWUbA\/TaryBxxuKDp+R4PRZPKfIUJJ8fxgvVX3rppXKjNrnEWZRdeS7EWZBIMhEzQaKNWbJ44nVnvFFh3333VWvUjj\/+eCWkeGQynzhbsWKF2tTQoUMHI3EWNMplEqGFNgQEibOePXvmFM3sE+LMhDzSgIAMAYgzGe7wCgLWCHz00UdqzRgvBmcBxWuOeL0VCwa9juzvf\/87cWfNa5F69+5dNgWmxQVvGvCurfJOa7JI4TVnPLWppz5NC28yrcmjfjzqZDpylmtakxf68xTqeeedpwQTM2CR5RUufh96apXf\/49\/\/KNa7M+PXrfFmwjyibN87+cVkUcddRT9+te\/VrtpeZ0db6wI8+QTZ7wGkOudy6of9s1CnH3lOg8tVxrezcrr4Aqdc2cyOhfmHZEWBEDgBwIQZ4gGECgSAlu2bKHf\/OY3auSLO1a9MH\/9+vVKONWtWzenOOP03IE\/8sgjZYvWedSIO2A+duF\/\/ud\/coozvSGA10Zx\/saNGytiPBrHefloDV7Ur6dVvTgLbQhgAcEbBXhKz1Sc5doQ8Oyzz6pjMniXIq\/xMhFnWmDw+jBed6cfnu5lYcuiKp84y7UhQIs7HuH0bgjgnbY8Lc3rzvQ6OJ5G5U0JfPwGfx70FNqt6d1swcekcDzwlDfXea5DfHlHKgvS0aNHq12n\/Hz22Wcq77fffgtxViTtAopZMQlAnFXMesVbVVACvCOQpyd51yMLKn6efvppNSKjj07Qozn8GY+W8ULz9957TwkAnsrk0TPusHkHJY+K8Y5GFiZ6hEl32txJ83Ed7OuFF15QOzN50Tnb4QXtnJ+FBQtGHrnLdaaWyVEavOPSVJx5j9LgES\/eUPDvf\/9bLX4\/7rjjlHDlBf0m4oxHzvg9mQG\/F09jzpkzRx0dsnPnTiVk84kzZsyjjzxix0dn8FEabIPFEr\/3PvvsUybOWOSy4HvzzTeVGGLmvMbtqaeeolq1aqmdlXzuXFhxxnXIGxe8vtmXd\/eoFqHes+P0u\/MOVd6xyw+Lby4Li0s9cqbrhde2eTceYOSsgjYyeK1UEIA4S0U1oBAgYEaAR814ypE7Xu78+fEfOsrihYUKi7ivv\/5arTs655xz1OG1d911l9oAwGur+FwzFm+8A5TP9tLHLvAidxZifB4XH+yqF6nzlT2cn4UeixkWEjxVxyKRz\/zK9wQdQstTmZdeemnZURim4oz9+A+hZbHJYkwLyDAbAvhMNz6GhI8b0Yf68kJ+Hol75ZVX1PlxP\/rRj5To8+\/W1O\/Mh9DqUUR9CC2XiacVvRsX9EHATz75pDpfjuvh7LPPpoEDB+a9QzTfyBmz4LrkUUge3WSBysdycFzoJ0ic8Wfed2eBx7uAuV7\/9Kc\/QZyZfSWRCgScEIA4c4IVRkEABLJOgKdKedMG\/4lyebuXX9ijNJJgj5GzJCjDR1YJQJxltebx3iAAArEJ8NEkvH5sr732UiOQ+qwxfYk7T\/X6zyCL4hTiLAo15AGB4iUAcVa8dYeSgwAIpIAAT1vytOjJJ5+szovjjRKmV1uZFl+LMz7mg3e3Hn744eqeUD47jR8bAtC0LLwejae4eSMI+41yxIqpL6QDgawSgDjLas3jvUEABKwQ4HWAvDmCbxLgdYA8isa7RXndGm9YyHf5uGkBtDjT6wx5YT+LNAlxxptP2DfvHuYH4sy0FpEOBMwJ\/D8twq3xYfNeIwAAAABJRU5ErkJggg==","height":296,"width":492}}
%---
