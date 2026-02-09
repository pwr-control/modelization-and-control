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

omega_th = 0.25;
% omega_th = 0;
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARUAAACnCAYAAAA\/m8goAAAAAXNSR0IArs4c6QAAH7FJREFUeF7tXQ+QVtV1P1rTuhqMLqAJbnDR7EZjKwbTDKJTSChiOgVTOi1\/nJYCpZSKm6lQYBccpBNgoawzBc1IHWRIO7vrkDIjOGkpQ4SCaGNBmKQmglkWuiJWQYpSsFFpz4PzefZ+971333v3vvcunDfjqPvdd9+5v3vO755z7r\/Lzp07dw7kEQQEAUHAEgKXCalYQlKqEQQEgQABIRVRBEFAELCKgJBKCjhffvllmDRpErS3t8PQoUNT1BD+ilr3mTNnYNWqVTB9+nSora218i2sc\/78+UFdra2tUFNTA\/jdw4cPw\/jx4618I6ySZ599Fnbv3l35btTHnnjiCRg9ejQ0NDTEypSkbGxlFwoQTps3b4YBAwbAunXrjGTR4Wv6TZvlTpw4AdOmTYN58+ZZ19MoOYVUbPaig7qWL18eGPzatWudkUpPTw9MmTIFHn74YaekQko+YcKE2O8g+axevdrIkJOUTdJFBw8eDHAZM2ZMYJimT1lIBeVF\/Tl69KgRiZu2L65caUmFOhQBwYe8AuqwU6dOBX\/fsWNH1ShCoz3+Pnjw4IpB0t8fffTR4G9Y97Jly0IVPEwG7k1g\/Tjqkzz4Do5odXV1wd9xlMNnxowZgWJSnWTAqmfC\/x89h+bm5uB9daTUKa5KQHEYYr1z586FWbNmwf79+3vJiYYa9m38zpo1awKZRowYAdu3b68YPx\/dsUKOr2r8RDL0bSrL+4\/6vrGxMRh1VTl1ZdF75PIjKZBHphpEmAzq33V1qG2lPtbpKNdDMnbEMExHsS78neqMwlyVlXvQLr3qMHIpJamohseVkYx1z549gSL37ds3ULaBAwcGisNH3bFjx\/Zy81EhMWzhHRXmBaijKjfYAwcOVMIfIhWSh1x1PkLQd7GzUV7uFUSRChpHlKeCuHR2dgYEiQ\/igO\/oyEuHIb6jYobhD+K\/dOlSaGtrq9RL+FJbkAAIX972MJyoLXzU5GW3bt3ayzNRCQjL1tfXBwMAEQYZj1qWY0pkRLhwQ6A+pt\/UvojzVML6WNUJ\/Kba5x0dHb2wJ2+IZCAdxXfpbzrMyR6oLzdt2tQLxyTeYZwHYvp7KUklbETDzm9qaqrKB\/Dye\/furVJOMjwiAxoRo9xm7Mg5c+Zo3W+dp0KdivmJqI5M4qnEkQrVtXLlyqC\/eZ4nCYZh4Y9utMf8DnpflF\/g3yGCJyPlOKgEjzjR6KuO4tgWXd+Ejcg6AuKDRVgIoMtf8VwT4aILf6L6mDyVI0eOaAmfjJPazz1ZnWeB5cIwVwmL6wT2g0qcpsSQpVwpSUUdnTkwcaSycePGwG3kD4UOx48fjzQ8\/k4c4ZAB06jESUUlDl6vTVIh5cX20YhGuZckGKqkQoqNxrRo0SJYvHhxUD96NUgq3GA5TqTgFLJSu3HUxUQz9yiRVNTwjJOLzrNCw0LvJIpA1bCTZDAhLjWkjCKVqD5W68H\/514kkTXHJcxbQvnVvuTYkE6rJEADJ9kOeZqIu+unlKSSZJRFkKI8FQ6gOgqYEoc6w2Pqqehcbpukwkf0\/v37V0If3UgfRcwqqXAlRnz56J3EU+HYRyUveW6C3H4dWYXloeI8lTAjsuGp6Po4ilTUQVElnKyeitpW8VQuIGKSU6FRi2LmJDmVsFicd4jaGbrRAevReSrq6IKjCcXUo0aN6jVqkQtMMqlKFTf7w0d7nqAzwZC8D5VUdG2lRCXPqVBb3nnnnUo4FJdTIS9HJasoGXhYRUZJ\/U9JWT5TlGdOhdrD+1gN9VTiUHNJmBAnMtWRCs+pqJhLTiWB38WNhc988FGgT58+gTusurZxsz8mpIKiJpn94eEP\/nfYzAB5ETSzQln+MFLhbdGti1Hjd76WxQRDDGnwoZkqJA8+I4TYoxeEDw+tXMz+8BkWLju68vgQZtjfSGTkuahleTIX30sy+6Mj5rAp5bjZH9IJlVTUfkF81US42teX9OwPAoZKuWLFiqqFQmon6OLcON4p0xqAOFkvxt+TelI67y\/vxVi+90MWzNHrNF1saAsnqzkVMnh1epWExZBi9uzZ0NLSYrQyUddIIRVbXZ+uHnVgwFqSrCwuQsnTtbQ8b6XF\/KJYUYsKg24pkoqOOPj6B1tLzsvT9SKJICAIIALWPBUkjPXr18PMmTNh4cKFWlLh6x7w41GrWaV7BAFBwE8EKqSiLiyKao6aC0H3bMmSJTB58uRgxahJiFOUa+ZnN4nUgoA\/CPQiFVzkhIudokITJAO1nDpLgs2P29VJceKwYcN67b25+eab\/UFPJBUEPEFg27ZtMGjQoFyktRb+mCRjMfzBh\/aX6GaJkFS6urpyaXyWj\/giJ7ZRZM3S0+HvCq56bKrCHyyWZZu9OsPDiSRqByuJ50tH+SKnkIobQhFcw3Ht5amoRh8XwrjoLl+M9dChQ7m5k1lxFlmzIqh\/3ydc87SryPCniNmaPBufRdV8UiiRNUtPh7\/rE6552pVxTkWXoHXRVXk2Pov8PimUyJqlp4VUkqIXSirqFHNeoZCQStIujC8vpBKPUZoSPuGap12FrlPJi0TUzsyz8WkUid7xSaFE1iw9LZ5KUvQqpIJJWvyn6OXzQipJuzC+vJBKPEZpSviEa552ZWXxW5oOCXsnz8ZnkdsnhRJZs\/S0eCpJ0bOyTD\/pR6PKC6nYRPN8XUIq9jH1Ddc87cp49sdNt1TXmmfjs7RJDDULehfH6O+TDuRpV0IqKW3DJ4USWVN2csxrPuEqpOLB3h+fFEpkFVIRUhFSsWoFQipW4axU5hOuQipCKlatwCflF1mtdn2lMiEVIRWrmiWGahVO8VRi4NQmavluZbqyIeyISNvdlSejZpFdDDULejL74wa98FrztKsqUuFXaY4bNy44d3bBggWAFz\/ncdR\/no3P0rFCKlnQE1Jxg154rXUj\/gh6tv99Lp+tIhW+GxlvXSNSQbIxOW4yq9RCKlkRrH5fCNA+plijL7h2vHIMHur4GZx4\/BtugFBq1YY\/eBsaXrI9depU2LBhQ3BC\/qxZs3rd1etKOiEV+8j6ovw+GapPspaCVBAwft0m\/n9e12kIqQip5HVAc1akfSHr0pBKVsDTvi+kkhY5yVPYRy66Rl9IZfmWbli+5VCx4U\/encO\/J6RiH31flN+nkMInWQsnlbhLxWwd3oR5G3zwsm4hFftEwmsUUnGDry+4Fk4qCD8eeN3d3d3L4Olvw4cPh46ODmhtbYWamppUvUX5mhkzZgippEIw2Uu+KL9Po79PshZOKmEHXNPfm5qaYNWqVbE3GYapPdXT0NAAp0+fFlJJxg+pSguppIIt9iVfcG1\/5RjMKnJKmRa\/7dmzB9atWwdo\/HSt6V133QUPPPAAPPfcc6k9FQx70Ns5fPhwlTeEvSg5lVhdTlzAF+X3afT3SVZco4IzQIWuU0HA1Cnl9vZ2aGxsNLp8PUzrsc4dO3YE3okuxBJSScwXRi8IqRjBlLiQL7iOffJV2PWLk8WTSmKEDV5AL2XNmjW9Sqp5FfRU8MELpcv89PT0QF1dXZlFrMgmsrrpJh9wHTlyJJy6bzl8clW\/i5NUeNeKp+JG0XW1+jKi+hRS+CLrkRNn4c7vvhSoRaHhj3rdKSnq4MGDM13eLqSSH5HwLwmpuMHdB1xp5qdQUsHZmdmzZ0NLSwts3LgxSKoOHToUMHSpr6+H8ePHu+mhC7VKotY+vD4oP7VaZLXb\/7WPvBBUePn\/vAvvPvUHdisPqS1yl\/LWrVsrMzRyl3JvBEX53ein4GoHVwx7cBoZE7T4fHbXCjjy43+yU3lMLaHnqQwbNgyGDBkCc+fOhRUrVsDevXuhs7PTWvgTJpd4Kvb7XQzVPqZlz6lgHgWJBZ+BtVfCqWcehK6cTlTUHn3AvRL0VpqbmwPhcFoZQyGXj5CKfXSFVOxjWlZS4YlZIpR9C+\/Odf2X3PuTUt\/EUFMCF\/Oa4JoO10PvnoG7lr7c62X0UJBQ8MlzsI7MqfDL2iWnIjmVdOqe7C0hFXO81LwJvYlk8sSE2+DeL11bqawQUonbnYzS4SHYWTYSmsCVZ+NN5AkrI8qfBb3wdwXXaFxff\/s0\/NUPDlQSsLw0ksmmv\/hqkENRnzztythTcaNC1bXm2fgsbRLlz4KekIoJeuiJ4OxN54\/f0pII5UwGXnclbHroq5FV5mlXklMx6V1NGSGVlMBJTkWLABJI17tn4PGt3aEEwsObe790Hcy9r17rleg+UAipmIQ\/NlfUhulWno3PYhZCKlnQu3Q9FSSP0x9+DPM2HoAj752tTPtGoYnhzD23XAvzRg8yJpFShT9uVMW8ViEVc6xMSwoBmiKVrFwYrrQ+5G9\/dAQOvn061vNQ8yIYzjwy6ia4ud9VqUlESIUhIKSSTLFNSgupmKBkXgZJA72Hp7b8FH74xi+DF2nlqmktlEwdeWstfOebNwWv6RKspvXFlcvTrmKvPSVh85j5wW\/l2fi4joj6XQw1C3rlD3+QONbs7IGf9LxvHKboWoVEgZ7HuCE3wDe\/XOuUOKJ6JE+7irz2lB9KTReMyZTy+a4TUvGTVCg02bDnGOw48F4qL0MNVwIv4wJx\/MndA4I8iUuvIw3yhZJK3Bm1ixYtAr4oLk0Dy8KoWWQXUsmCnn1PhcgCZ1B+sPdtOHL8TCYPgyQkckDSuO8rfWHs4OsroYpPOlAoqSBi6JVs3ry56oxaDIHUKzVsq1aejc8iu08K5ausNOLjv5Eotr9+Iugy01mTuP7lhHFHXR\/46zG3QM\/JD429DJ9wzdOuQtepqAc1ybWnvVXUJ4Uqm6zkVbx96kP4\/stvwWFLXkVYWPKHX\/s8\/FbDdRUPI45sTH8vG65liQBk8ZupBinlfFKoPGQlonip6yT868H34D9PnLXmUahkgaEIPr97R3+4\/\/Z+1snCVCXywNVUlrhypfBU4oR09Xuejc\/SBp8UKqmsRBCIz5lffgxP73oTDhw7bTX0CPMq+v3aR\/Cd+2+F6676TGFkYaoXSXE1rddFuTztSrv3Z9q0aTBw4EDnmwd14OXZ+Cyd55NCvbj\/INwzuCGYlfiPox\/Aj14\/Aa87JAnElecr7vziNTBuyPVQa0AUPuHqk6x52pXxOhXdFaVZjDLs3Twbn0X+IhWKexIvvH4C9h45BXieBj62kpgqNpwkvvj\/u2GHDLwGfvu2vnBT7ZVWp1CLxDWpPvgka552ZZxTwYvAcFZo7dq1MqVseZ0KkcTZjz6Bv9vZ4zTUIMPhJIF\/G3rztTD+azfAZ37l8kLDDp8M1SdZCycVuvoUp5XpGTBgQGWKWcfo\/EbDsNW3ar26DYp5Nj7pyMTLx+37OHX2I\/jhT9+FXQfPL7By5UHoSGJg3xq448bPwujb+8FlAPDxf78FgwYNytLc3N71yVB9kjVPuwrNqaAWmXol\/FoPvLVv\/vz5gAdnq9d58HJ4R7MPORXyIj7+5Bysf+loEGrg0\/XOB3D01EdOjE31In7jxj4w+va+wbQoP8zY9OM+Kb\/IatqrycoVSirJRK0uTd7IxIkTqw7Jxovely5dCm1tbaEhVJ6NDzyICyeOr33xTXj1yCknHgUnCfQivj24PzTecHUFPNdLusVQs2q1\/n2fcM3TroxzKibdQiFQWPhjsqDOdeO7j5+Bps6fJ95VqoYZ\/WsAamrOJyznjx4E\/\/X+\/xaai4jqH5+UX2Q1sbTkZVzbFZfIKqlQxUgeu3fvjpySpkOhcNk\/v\/YDG4+PzQva23aegPZ958OWqGfANVfAF\/pcAfjv8XdcA5+78vLgv3WPD5dzk9wia1zPp\/vdB1zxgnZ6Cr33Jx3En75lMlNEYZKae7HFqLveOAkrthwKPSAYV2Uu+b0G+NyVVxjv9eC4yIiaVUv8Dyl80gFbdmXS68YHX0dd0aH+hlPP+KibD9GDwQcTuJhfodsPedLWRuPHPvlqFZlg3uK5mXfCTX1rTHCJLeOTQomssd2ZqoBPuNqwK1OQKqRickZt1EFNYVPKnEjUKWXdJsUsjdfdzjb6K\/1g+Tj9TJMpSLpyPimUyJqlp8Pf9QnXLHaVFD1jTyVpxWnLp2k8ksnzP3kHFj73RuWz995yLTwx8bZUoY2J7D4plMhq0qPJy\/iEaxq7So7I+TecJGrTCoPvJW08EkrnK8egdcuh4LO629myyBP2rk8KJbK60AC\/Tv9LaldZEOsV\/ixevBiamppgzpw5sH\/\/\/qp6y3hFR8crx+Chjp9VCCXshrYsIEn4Yxu9iyOk8ImsCyGV\/NQm+ktJGo8zPGO\/92ruhIIf9EmhRFY32u0TrknsKitaXoc\/tY+8UAihCKlkVTvxVNwhqK+5UFKJmgUqU\/jDp40x5OE33OfRYT6NUiKrG43wCddCSSUMflx7Mnz48Kr9PLa7y6TxfOoYZ3niLqe2LaN4Ki4QPV+nT4bqk6wmdmWrV43Dn6jFb7aEwXpMGk9eCs705JWYVdvok0KJrDY19NO6fMLVxK5soWRMKiZL720IFdd4Ptuz4vcb4U\/vudHGZxPX4ZNCiayJu9foBZ9wjbMrowYbFgo9T0U3pdze3l54+FMGL0XcdEPtSlHMJ0P1SdZCSSWFHlh9JarxPJcyZ1Q9tHyruNPMfFIokdWqilYq8wnXwklF3eyH+3c6OzuNT4LL0oVRjeczPvsW3u1sCb6J\/D4plMhq0qPJy\/iEa6GkEnZym8kZKcm7pfqNsMaXYcaHS+uTQomsNjSzug6fcC2UVMp6QTtfPVu0lyI5FTdGKri6w7VQUsFmoVeyevXqyun5tCAOT2gr6oJ2nqBFUin68WmUElndaItPuBZOKtgFmFeZMmUKHD16NOiRoi9opyX5Ixqvg41\/fqcbLUlQq08KJbIm6NgERX3CtRSkkgBbq0V1jedrU8oQ+oibbrXLe1Xmk6H6JGuhpJLXytkwtdQ1vmyhj5CKkIpvOlAoqSBYuM+nvr6+6jIwd6r0ac26xlPoU9Q+H127fRqlRFY3musTroWSStl2KfOp5CJ2I4epo08KJbIKqRRKKm7gN69Vbfw\/\/Ntb0PTsz4MKhFTMceQlhVTS4Rb3lk+4XrSkop6mr9tLpDa+jPkU3+Jpn5RfZI2jsnS\/F0IqlKB1eUYtrn\/p7u4O1rqE7XrmjS\/bKloZ\/dMpdJK3hFSSoGVethBSMRfPTkkklY6OjqqrUXnj+SraMoU+4qnY0QFJgLvDUa25cFJxuaGQh0Bx4c\/yLd2w\/MLVG2VZn0KdJSOqG4MQXN3gWiip5LWh0OSC9j\/beAz2vHk2uCR98+Q6N2inrNWHy7mpaSJryk6Oec0HXEtxQXteGwpNLmi\/87svAeZV8NjIMuz3kZyKG+MUXN3jWqings1ztaEwyQXtPEk76ra+8Oz0O9wjn+AL4qYnACtBUcE1AVgJihZOKiiriw2FSaaUy5yklURtAm1OWFRIJSFghsVLQSqGslovRo3nmwjLNvMjpGK92ysVCqm4wVZIpasL+NGRJx7\/hhukM9Qqyp8BvIhXBVc3uAqpMFIpY5JWPBU3ii+4usNVSKWrC8q4M1lmKdwpPdUsnoobjC95Uvn+v+yFsd97NUD3yYm3wcTf\/LwbpDPUKsqfATwJf9yAF1Fr4aSCS+gnTZpUJWJeF7RzUinbSloZUd3ag5C1G3wLJZWwla5umlpdKzb+mX\/+dxj31P7gxzLO\/Ejs704bhFTcYFs4qSxevBgWLVoEtbW1bloY46b9+ux\/hF2\/OBmUKuPMj5CKO7UQUnGDbaGkgk3iRxS4aWJ4rdh4IpWyzvwIqbjTCiEVN9gWSiplOE7y5LfXBsiW6UxatatF+d0ov+DqBtdCScVNk8xrxcYTqcwbPQjmja43fznHkqL8bsAWXN3gekmTysCvfws+uHdugKyQih0FE0O1g6PP3mrhpMI3\/o0ZMwbmzp0LCxcuhJaWFmhoaHDTQxdq5aRS1pkfyam4UwEhQDfYFkoqRCgDBgyAcePGwfr162HBggWwadMm2L17d9Xxj7YhGPA7fwlnbx0bVCukYgddMVQ7OIqnYobjZefOnTvHi\/JDmo4fP14hFSSbPKaar\/\/jp+Cjfl8ORCrrdLJ4KmbKlaaUEGAa1OLfKdRTQfHwhkK8mH3q1KmwYcMGmDlzJsyaNQuGDh0anITv8iFSKfN0spCKOw0QUnGDbeGkgs1Sl+ovW7Ysl2tQy76RkLpclN+N8guubnAtBam4aVp8rUQqD379C7B6wq3xLxRUQpTfDfCCqxtchVRKPp0s4Y8bxRdc3eFaOKmoZ8liU3FqubW1FWpqaty1HKByjkqZZ35E+d2pgHgqbrAtlFT4lDIlZelv2FzXxELhj5CKPeUSQ7WHJa\/JJ1wLJZW09\/7gJsTm5uYA87BzV1QPSFeOSKWs56hIotaNgQqubnEtlFRo5ke95xinmevr67UzQHidx9KlS6GtrS04LoGmpFWvBglr9uzZkStziVTKvEZFwh93BuDT6O+TrIWSStQuZVKluBPgwi5fV8lHp5pIKmVfoyKkIqTimw4USio21CXMq+EhEn5Ht\/ZFSMVGD\/Suw6cRVWS13\/9Yo9ekYnrAU9ixlUgqZT5HRWJ\/N0ovuLrFtXBSSTulHJV3USELu6AdSeVXj7wIL\/3Ng25Rzlh7T08P1NXVZawln9dFVjc4+4DryJEjK43v6upyA4RSa9WGQt2UMr4Tlnyl+vD34cOHB\/uDwh6TC9qRVMp8jgq1LU\/mz6oJImtWBPXvC656XCJ3KfODr8OmmrFa3ZUetFgOj0zAZ\/z48aB6QGE5lav2PhN4K\/IIAoKAPQQK81TIK9m8eTOsW7cuOJQJZ22mTJkSrKp1vUvZHoRSkyAgCBSBQJWnQkKYzNQUIbB8UxAQBMqNQCiplFtskU4QEATKioCQSll7RuQSBDxFoDSkwsOt9vb2yFmkPLHmSeiwg6p4GTzbl3JRecqpJszjDtUq+npbE1wpv7dmzZoAyqL0wkRWvhK9SB0I0zmTLTK29LUUpMKX7x84cADUfUe2Gpu0Ht4R+C7f30R1qZ2F5NjZ2Qlr167N9dpYE1l5+3EJABprEYZqKitfSIk6Qoewuz5+g+NkKiviiQ9OZKDceRwSb6rPNNGC5fMY8EpBKrwTcNo5btOhKZhZy+EIhcqCBIGKPH\/+fJg4cWKkF2WyvymrXLr3k8iKZZ9\/\/nl4\/\/33Y9tTlKyoB0uWLIHJkyc7vxYmqo2muHICNF1V7gJbtU4kxaeffhruv\/9+eOyxx2DFihXO8SwNqXR3dwcsX7RbzjuFb4zEvyOpDBs2LPKs3qIUylRWWm+EdzmhgsWRpAvFN5GVSIXCnqLCHxNZCSMK4eNCTxeYxtWJgx31ueu7u4RUInojiUJhNWG7s+M63MbvprLSymfcaW7iedmQTa3DRFYaXCZMmBCQOPcY+KJMF\/IlHVjULSdlC3+wPZckqVAM6mv4U5SHQgZg4qaHHWuRd17FRFYyVPKk8jQKlVTiQmA1r1aUrFEEm6dMpfBUfE7UYkfyPU2uR86w+k0TivS+arR5ym0qKyfqojwVE1lVT6UoWYVUFAQoHi3bdByfTuQjOhHJkCFDgi0MePkaPXGHWLky4DhZMYwoA6lQqDhp0qRAHB2u6l6xIvXCBNeyTylfcp6KKyOTegUBQSB\/BEoR\/uTfbPmiICAIuEJASMUVslKvIHCJIiCkcol2vDRbEHCFgJCKK2SlXkHgEkVASOUS7XhptiDgCgEhFVfIxtRruljOxV4i2mCG0+CmC9+KXC2sQklTvK6m7qn+vO4PL0gFnX1WSMUZtNEVF0kqaDQ7duxIdDRo2UjF9U72onZFF6SOVj8rpGIVzurK+MIpGlnxeAda+DVjxozAuNXDw9GDaGxshGnTpsH+\/fsr91PTbmk8QxifKE+D10mjLtZF3w4bifnZNrQ5jkilT58+wTdVL4G\/wxeq4RL31157DXbu3Fm5PI4vdBwxYgRgnXT2sU5m9agDleDwG1dffTVs27YtwIowVXtD9fqiiFJIJb1hCKmkxy72Td1ZK\/gSrhZVzwrhO0j5hjS8W0a9pxrrICKaM2eO9owMCnFWrlwZEABuHkRjp\/fCRnpuaHwf1vHjxwMyIkJR66P9MXSXNsmoXu3CV3b27ds3IE281gXl4r\/hnUr8GxxsHanQQe1UJ9anXhcjpBKrslYKCKlYgVFfSdRpW1HhDzcaTir4FTRCMpio\/Tuq4fH9KFEHYYVdCKfuZ4mSn\/\/GDy9C+dX31HNI+OFGYZ6EjlSIxHTfoN4RUnGo7KxqIRXHOPOkKA83VOOik9hIHCqrIxV08fmjO79D3X5vsmkz7NZI\/JZqyFx+3Y2WFIKoJBVFMuoNDvhdXTJWRyr19fWVc27CCE9IxbGyX6heSCUfnIOvhJ0OpnoBUZ6K6al4LjwVHjJFeRiqpxJl8GlOTIvzVFTiCvNUos49kZxKesMQUkmPXeyb6sjIj0gI29ZPiVisvLW1FaJyKjxvossf4O7ppDkVbmjogVC4hfKYkAq9Q3kS1VMxzang6WRhV+3qSAX\/hsd+qiEi7yRdnolwVpPBQiqx6h1aQEglPXZGb3KXnoc\/NMuBYUJTU1OQlMRkIyZTW1paYMOGDdDW1lYxEvwPflYuzf5EHV0YNpMSNz3MQzF19geJDg2QkyLf9o\/hyvTp02HLli0BKa5atQq4p0IeW3NzcxDa4AXip0+f1s7+hK1D0ZEKnre7ffv24AgKjokuh4PfRpyRMPft26clbyEVI\/XWFhJSSY+dvJkRgagcTlTVcTmVjGIFrwuppEdRSCU9dvJmCgTU9Thha0riSAWnt8mTwdPiVW8ohWiVV2RFbRb0AIRUsuEnbwsCgoCCwP8BZXsn+wrltvEAAAAASUVORK5CYII=","height":167,"width":277}}
%---
