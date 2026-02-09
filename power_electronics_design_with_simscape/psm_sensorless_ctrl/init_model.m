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
use_psm_encoder = 1; % 
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
% fPWM_AFE = 2*2.5e3; % in case of sv controller run 2 times the maximum pwm
fPWM_AFE = 2*4e3; % in case of sv controller run 2 times the maximum pwm
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
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:45e82fc7]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:3873a036]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:2f8df18e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:45ef9943]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:20f05bf9]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:0445503e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.194386045440868"],["33.957607642498068"]]}}
%---
%[output:04a70309]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.024788847500562"}}
%---
%[output:69e958f5]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.790076231160342"}}
%---
%[output:8e3d2c7e]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:95bd65ef]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714302e+02"}}
%---
%[output:7d37a44a]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.994503273143434e+02"}}
%---
%[output:9ef89060]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"mosfet","value":"    data: 'danfoss_SKM1700MB20R4S2I4'"}}
%---
%[output:339abfd4]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAW8AAADdCAYAAAB0SfPeAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ2QV9V1P6aasGooLpLgFmFRF2NrisZqkGLQoqE2Wcxk4sAy0zDAKJkgaZNlgMVkUhphgZHUD7Rhki1jZspC2kBdmraUWY0ZRdSq0MQwoiwb3KwaXESxwc7Y0p7n3L+Xu+\/jvvfO\/Xj3f96Mo+7\/3nPv+Z1zfu+88+6794xTp06dAr4YAUaAEWAEKoXAGUzelbIXT5YRYAQYgQgBJm92BEaAEWAEKogAk3cFjaY75Zdeegnmz58Pra2tsHz5ct1u2u2OHTsGCxcuhPHjx8PatWuhoaFBu2+RhuvWrYOdO3fC5s2boaWlpYiIwn1OnjwJK1asgKamJiNYFp6YRse9e\/fC3Llzo5aLFi3KNX+XmKuqCX+bM2cOzJ49W0PzsJsweQdsXxfkjWPu2rUL7rjjDnJkVSIxOZY6eUGAW7ZsgSlTpmTqlndu27ZtgwkTJmjJzhxcaiBuOkeOHIGuri5obGzM0x18Im+cOM4HbVFEl1yKV6Axk3cFjFSVKZq+WchEgpiYfKpQMc9DGnlxQOLu6OgA3RtDHn8Ijbzz3kTzYFW1tkzeBS2Gwbxp06aoNz5Ky4\/ygmSuv\/76KCDx6uzsPO1RT+6PZQ1RdhCBj31PnDgRlQlU+eqURfCLvwsSUElEtMNHZ5y7eIQW7QYHB097tFbLIvgjlg5EFof\/L8omy5Yti7Lt\/fv3RzImT558WnYkSAR\/U3WVyzo6uN57771w1113DRtLzCduDmJ8xBMvgYFsF7m8IGMucMCMW5SfxN\/UsZLmkPT3gwcP1koaYl44RtJc4txVnYvwJ2EvoTP+f9wNIqk\/lsGEL48ZM6aGt4yZiquMm\/Crq666KvIZvDBjlnWePn169Pfjx4\/X\/EX1R3nOeW+MBcO7Et2YvAuYSX2UVDMnQUDCyeN+F7Xb0aNHRwQoiEE4JwYLOvrQ0FBqhpkmG1WTs1OZvAUJqcEgSAPnftNNN51W004jbyTkgYGBXHPF+dx\/\/\/21G58OrgI3VTf15qDORcYJbyx4E0JZwkay3lhPlTNtYYMlS5bUbsBxv4ubkIppnrmhH6TNRS17ZN1gkYDlG25WfxU31ZdVG8k4yDdz2R+EL+PY6nzx5of1eHGzV\/1d9RHb71kK0IO1LkzeOaFOy8IEAcfVZgWJ3nbbbcNe8qURQZqzZj0SJ2XeciaT9sieRQxJwZr0glSez9e+9rWIVEQmjrrINzH8u4q1TtlEzSIxwxZjyXXfOIKUX4bKj+c4FyQYOeOUnxDUbDYpO4ybG95E027A+GI2rVQQ95v8N3GjSqp5qzio4ZB1Q8X2avYtMv+4m7k6nurDu3fvPq2EJLAUN84sn88ZzpVuzuSd03xxgamS3H333Xfaqgj5d7W8IIYXj5tqRplG3llZiA55p72QoiZv1E3cqJC0li5dCiIo8+KalHmLbPpTn\/pU7Skg7oYZR96iDCa7BBI2vkhUyVt9tMc+gtyTMu+4uSWRd9Jc1FUWcTdfWbdZs2alZt5Z9fYs8hY3MbxJqjjHkbc6XhJ5q2EpSnxM3h8gw+Sdk7xNZN7yFNTADynzRj0FsVx33XVw6NChWskkL64qeau4xWX5eTJv2SZZ2akgpKQbcNrcdDLvNBd1mXlPmjTptKdI8fQklo5SZN6q7kzeTN45Kfv05nmykazgFDXvJIfPyq7VTEauEaoEF5dlJRELZsRq1ibqkaKGqZZN4kofKtBy6UBdc6yDa9a7Anw5hvVWfPqRX8oWqXmr9fW0R\/e42q\/6HiNpbioBZ5V0ZEyzno7y1rxVG6bZRJA3zgffz4iSR1rZpEjNW16JkxUPpQK7Yp058y5oMJ1VEVjD\/c53vhONkLbaRF6ZkSfzFlPPu9okqUarrjaRM2X8bywd4AqYuNUmYgWJwCVthYxoE7fyQQdXsbJHHeu5556L6qV4iVUMI0eOjMgcL\/GSEucmbJO02gTbi\/nFPRWkrbLAvnnmJggTX94J4hMv8oSN05YRpq0W0clUdVabCMzVBEBeFYN+jEmI8I+kl+1yH9Wn8KWmWpKSbcSrTTzJvNHB8Yr7+k81oLr0rCDnWumWVke2MgEeJBOBvOuF5cw674cumZOp4wZxS0jT4Mhrt5ChdZZ5CyMkfa6Lv3d3d1v57JrawEze1IiWlxe3fFNeppg1AvojvmB18Wl+1tyq9Lu6FBbnrq4yStOHb6KOM298TFu1alU0i6S9IvDxrL+\/P9c+DL44MZO3L5b4YB5qaUAui+jMtsp7m+joZ7ONWuaTP1JLm4ewIe9t8j5KTjJvJLfm5uaInPGKK5vItc+8gWbTEXksRoARYARcIGCdvPGx6aGHHoI777wzWhEQR94iy5k6dWr0RRs\/srpwDR6TEWAEfEbAKnkjKa9evRrmzZsXbemZ9sJSBk0lc\/m3iy66yGd8eW6MACMQCAK9vb0wceJEb7SxSt5xX6UhEll7DAvybmtrG7ZlJpJ3X1+fN4BSTyR0\/RAv1pHaa9zIC92OvulnlbxVl0rKvPHFRHt7O6xcuTLK0LFsgm3j9vD1DVDqsAldPyZvao9xJy90X\/VNP2\/IWyVsOUtP++DDN0CpQ+fw4cNePapR64fyWEcTqNqXGbodfeMap+RN4V6+AUqhkywj9IBg8qb2GHfyQvdV37iGydudr2uNHHpAMHlruUElGoXuq0zexG7oG6DE6nFJgRpQR\/JCJ7Z6uAn7xjWceTsKZt1hOeh1kfK7HdvRb\/vozI7JWwelHG18AzTH1LWactBrweR9I7aj9ybKnKBvXMOZd6bJ3DbgoHeLP9XobEcqJN3JYfImxt43QInV45o3NaCO5DF5OwKecFjfuIYzb0LjmhDFQW8CVfsy2Y72MacekcmbGFHfACVWjzNvakAdyWPydgQ84bC+cQ1n3oTGNSGKg94EqvZlsh3tY049IpM3MaK+AUqsHmfe1IA6ksfk7Qh4wmF94xrOvAmNa0IUB70JVO3LZDvax5x6RCZvYkR9A5RYPc68qQF1JI\/J2xHwhMP6xjWceRMa14QoDnoTqNqXyXa0jzn1iJUl77gDXLPAmTx5MuzYsSOrWanffQO0lDIxnTnoqRF1I4\/t6AZ3ylF94xrtzFvdbzsLFNyPe82aNbB58+aspqV+9w3QUsoweVPD5408Jm9vTFF4Ir5xjTZ5F9bYcEffAKVWl4OeGlE38tiObnCnHNU3rtEmb1E2GT9+PKxduxYaGhoocSksyzdACyuS0JGDnhpRN\/LYjm5wpxzVN67RJm8EYdu2bdDR0VHDo7OzE2bPnk2JT25ZvgGaW4GMDhz01Ii6kcd2dIM75ai+cU0u8paBwAOBN23aFP0JX0zGHQ5MCVySLN8ApdaZg54aUTfy2I5ucKcc1TeuKUzeAhR1FcqiRYtg+fLllJilyvINUGrFOeipEXUjj+3oBnfKUX3jmtLkLYMjVphs2LABGhsbKXFLlOUboNRKc9BTI+pGHtvRDe6Uo\/rGNaXJW828W1tbrb7Q9A1QSmdBWRz01Ii6kcd2dIM75ai+cU0h8sYMe\/78+TA4OBhh09TUFK3nbmlpocRKS5ZvgGpNOkcjDvocYHnclO3osXE0p+Yb12iTd9wXllu2bIEpU6Zoqm6mmW+AUmvJQU+NqBt5bEc3uFOO2nz5NdD\/i6cpRZaSlZu8kaxtvpDM0o7JOwsh\/39nYvPfRjozDNmO3c+8Bou7D8Cx796gA4WVNrnIu729HVauXKlVHuHP42nsF3JACIRYRxpfcS0lZDtWnrwXLlwI+\/fv1\/YR3phKG6rEhiEHBJN3ef\/wSULIvlpp8vbJSeS5cNnEV8vozyvkoOcblL4f+Nxy3a5+WLfrcDXLJr4Cy+Ttq2X058XkrY+Vzy1DtiOTtwHPY\/I2AKplkSEHPWfelp3J0HBM3gaAZfI2AKplkUzelgE3NFzIdmTyLug0uAkWXnFLFJm8C4LqUbeQg54zb48crcRUcJkgvrSs5FLBEnqX6rp3716YO3cuJG14xeRdCl4vOjN5e2GG0pMI2Y5M3jndA7\/qXLVqVdQLP8HnzDsngBVpHnLQc+ZdESfMmOasB56Hxw8dDyPzPnnyJKxYsQJ27twJuBnVggUL4J577gHKHQWxXNLc3Az9\/f1cNgkjBmK1YPIOw7gh2zEY8hbEPXXqVJgwYQJ0d3dHOwn29PTAnj17SHYVxC80H3roIbjzzjvhvvvuY\/IOI76ZvNmOlUTgiruehCPH3q1+5i2fJD80NFQj74GBgejE+LLZN94cVq9eDfPmzYs+xc96YYne0NvbW0mnyJo0Yjpu3LisZpX+nXWstPlqkw\/Vjje0zoa3P\/v+ookgXlgioeKWsLfccgs8\/PDDUdlk8eLFUQml7MZV6pazwjviXlryC8vqB37Ij9tc866+fz7+8nGY9eDz4ZA3aiJWggjzmDqQOCvz7uvrq76HJGjAxBaGadmO1bWj2NckmMzbpimYvCfahNv6WExs1iE3MmCIdsQ69x3dB6KVJh\/67RvwxvduNYJdEaHaW8IWEW6jD5dNbKBsdowQg15FjHU060OmpCN548tKvEYc+CcY\/Nd7TQ2VW24h8o47VUcd2dbRaEzeuW3uXQcmNu9MUmhCodkRiRtr3fjv8Y0j4PjWv6jmSTqqNcUa7NmzZ9d+2rZtW7QmG19Y4n\/jssF77zV7p2LyLhRnXnUKLejjwGUdvXK5zMkgYa\/f1Q9bnnk1arv+iy2w5sufAZ\/erxXOvONO1RGn5+BSQVxCiMsG8WBikxeTt0l07chmYrODs+lRQrEjEveOfb+BVf98KIIMs+5937wWfOOaQuSNCmHmvWnTJhCHEKt7kHDmTRMqoQREGhqsI42vuJYSgh3lUokg7p6vXhkReDDkjYrJ67HlGrecgTc2Nhr1Kd8ApVY2hIDIwoR1zEKoGr9X3Y7yy0mVuPH\/feOawpm3L+7kG6DUuFQ9IHTwYB11UPK\/TRXtiIQdLQfceiD6t7gw0xYZt\/ibb1zD5O15TFQxIPJCyjrmRczP9lWzo1oiEdn2xjmXwbRLRg0DORjyVr+uFJriifFdXV1gulzi692QOqyqFhBF9Gcdi6DmX58q2BE\/dV+\/63D00Y18Yaa94UuXQsvHzo7q23FXEOQt1nkvWbIEHnnkkWgDKdw8CbeIxZ0G5eWDpl3MN0Cp9a1CQJTVmXUsi6Af\/X21o\/yVpIoUEvU\/3D45Iu2syzeuKVQ2kXcV3L59e7TnNhK2zReVnHlnuVp1fvc16CkRZB0p0UyXhWR9\/6NHoOuJX8c2RMIef94I2Nh2WWKWHWzmLfbzxhUm06dPj44p+8EPfhDtLnjkyBEumxD6KQc9IZgORbEdzYGPZL1u1+HojMmkSxD2spkTY+vZOrMLIvMWim7cuBFmzpwZfZCDBI7bweKhDA0NDTpYkLTxDVASpSQhHPTUiLqRx3akwR2J+qEnB+GZ\/reG1a3jSiKdX2iBP2g6N1eGnTRT37imUNmExgw0UnwDlEarD6Rw0FMj6kYe2zEf7vhiEbNlzKhfOfZuJlGjdGx\/580Xwacn\/i4JWasz9o1rmLzz+ZT11hz01iE3MiDbMR5WsbY6D0kLov7ji0fB8pkTjRB13GyDIG\/5hSUeUyYufmFJH\/cc9PSYupBYz3YUBL3n0HHY8vSrcOTN9z+M0b0wo54+6Txov7E56pK0lE9XXtF2lSbvpOPJZDBs1719A7SoYyT1q+egp8bSpbx6sOMT+1+Cd846Dx549JUIanUtdRb+4qXitz53EXx85EeckXTSPH3jmkJlk6TMO8s4Jn73DVBqHesh6FlHaq8xIw+z5VMA8L3HXoEXBt\/JnUHLWfO0S86DpTdNgA+dcYZ3JB00eZtxjWJSmbyL4eZTLyZv99YQZYz+oZPQs\/8oHHz9vwqRs0zQuJZ6+Z9OhAvPG1EZgk6zhG9co51565yeg4rz5\/G0gcjERounK2mu7SjIefeBIXh4328KlTVk7ESJo\/n8BvjGje9n0P\/z1qswcWK4561WlrxdOX3WuL4BmjXfvL+7Dvq88y3SnnUsgtoHfQQxP\/\/K27D7wDE4MnSycNYspIqXgrii47Zp46DxnLMys+fQ7egb12hn3uXcy1xv3wCl1jT0gEC8WMdkrxHE\/I\/PvQ4\/ffFY6YxZLWtMGP1+5vw7Hzojkl1mJUfodvSNa0qRN56W09HRUfO8zs5Oq5tS4cC+AcrknR+B0IM+7gYlSBlf\/P3LL96AXxFky3LWjPXmCxtHwLxrm2DsyI+UJmYdq4ZuR9+4pjB54zFouC2s2P5V1MSnTJkSHUBs6\/INUGq9Qw+I0DJvQcpP9b8VZcr4dWDedc1JPiSyYiTmSR8\/B758bROMajjTCjHr+HXovuob1xQib\/5IR8eVadqEHhBVIG9ByP976hRsefo12Nv3\/l7QVKQslyuwxvyJsefALZM\/VnOgMqUMGi\/UkxK6rwZB3mhKzrz1HLpsq9ADwgV5q1\/3PXHoODz+8pukWbJcwsD\/HtMA8KWrL4SbLz\/fm0y5rG+q\/UP31WDIGw3HNW9q9x8uL\/SAoCRvmZSff+UEPPriMeg7+lvSDFnOkkVdeebvnw9XXPjRGinjPNRsme1oPlZMjxAUeZsGS0e+b4DqzDlPm3oPerG7HBLiL199B\/YPnDCSIaukjDvT3fpHY2HEmR8iyZTr3Y55fN7Xtr5xTaGat0\/g+gYoNTYhBr3IkE+8+160gf5TLx+FhhEjyDNklZCbRzfAgmm\/B41nn0VCyHlsHaIduWySxwPo2zJ502NKKrEKQS\/IGP+NmTFmyJSrLFRA5VUXl449B+ZcPRbGnPvhWjP8Pa50QWqYnMKqYMecKg1rHrqOviWKhchbLAscP3689ZNzVI\/xDdCyAeA6m1Ff5j11+C3ArTwPGagdC12bRp4JZ555ZnSuIK5NvuyCc+Hznzw\/+uRazp6psbUpL3RiQyxD19E3rilE3miouO1h+SMdejqgCAiZkJ\/ufwseO\/hm9FEIXpTL3WTt5ez44jFnw+f+cAxMkk7olrNjCh3pkaeVyDrS4ulCWjDkHQcerj7ZunUrH0BM6FlxQS+T8X8OnIB\/++VQtJ+FLTLG7PjGT4yGqyaMPK1UUVRtJraiyPnVL3Q7BkPecZk3nia\/efNmkE\/XUd1L7pe2A6EqP6mtb4AWDSckZFGn\/fFzr8PLR38b1Y37jr4Dg2+\/V1RsYj85M0YyRiJGQlazZxu149CDvh5KCvWgo29cU6hsImreaDDxebwOu5w8eRJWr14N8+bNiwgeM\/U9e\/bE1s3x0\/vu7u7MmrpvgMbhIGfK9z7yK3jpdbNrj3GzocsuOAc+\/8kxJJmxjm3LtGHyLoOeP31Dt6NvXFOIvKncJe3MSyT2\/v7+zH1SfAMUifpHz74GPzv4Zu5joFRcMTt+77334KIx58KksefA7dPGwYizPlh3bCMrprJ1mpzQg74estJ60NE3rnFK3mmZN35+v2nTplrMb9myBXDTK\/VyDSiuwvj6j17MRdTyXsk3XjYarhr\/fu04bg8LJjYbtw\/zY7AdzWNsegTXXKPq54S85VN54kgZyysrVqyAqVOnRlvMYgll6dKlsfV0BBSv3t5e07aryX9w75vQ9cxbqePh8rcLPnomfGbi2fAnF58dtcW\/5b0GBgZg3LhxebtVqj3rWClzJU42VDvOmDGjpnNfX583xnJC3kJ7QeK4hWxcVi3aqWQuo2frboifad+x9UD0UlG9xJFQbddcAG1XjyU1LmdspHA6E8Z2dAY92cC2uEZ3woXIm2pL2DRSlhUQ7dra2oaRvElABVHPevD5YaSNhL3jK1dEJ5CY3LKTg17Xlf1ux3b02z46szPJNTrjq21ykXfc8kBVYGtra+IKEZX0Ud6yZctg\/fr1py0vVNth2QRr4HErW0wBisStkjaS9J9\/+gK49aqxRglbxpSDvohb+9eH7eifTfLOyBTX5J2HaJ+LvOVyR3t7O6xcuTJ1TXfcpNQbgKh5xxH7\/PnzYXBwENLWj1MDKtZbI3GLC0m756tXWiNsJu+i7uxvPyZvf22jOzNqrtEdN6ldIfIuOyhlf2pA1+3qh3W7DtemuPj6C6PTs02WRtLw4KCn9BZ3stiO7rCnGpmaa8rOS5u8xcvFo0ePwt133x2VMfbv3z9s\/LSvJstONq4\/FaCYcd\/RfaC25A\/JeuOcy2DaJaNMTFtbJge9NlReN2Q7em0erclRcY3WYBqNtMlbQ5aTJhSAxhG3qzKJCiIHvRO3Ih+U7UgOqXWBFFxDOem6J2+fiRsNzUFP6e7uZLEd3WFPNXIQ5C1\/ZKMCU7Wyyc9eehO+8Lf7IjVcvphMcjAOeqrQcyuH7egWf4rRgyDvJCCwDj59+vTUD24oQJRllAEUs+4r7nrSW+LmzJvaW9zJY\/J2hz3VyGW4hmoOshzSsknaRlMmJo8yiwKqruPe981rna0oScOGg96U59iVy3a0i7eJ0YpyjYm5oExS8q7SYQzf6nkZHvjpKxGuf3PrpTDv2iZTGJeSy0FfCj5vOrMdvTFF4YkEQd5pNe+k3f8KI5bRsQigarkEs25fLw56Xy2Tb15sx3x4+di6CNeY1IM08zY50STZRQCd9cDztfXcuCTQ9VpuLpschokTJ7pwH2tjMnlbg9rYQEW4xthkypRN1M\/Z0\/bmNqlAXkDlrHvaxaOgZ\/GVJqdXWjYHfWkIvRDAdvTCDKUmkZdrSg2m0blQ5p20G6Du6Tca89JukhfQKmXdCAIHvbYreN2Q7ei1ebQml5drtISWaFSIvKm2hC0x71rXPIDintxiw6lpl5wHPV+9gmIKRmVw0BuF15pwtqM1qI0NlIdrjE1CElyIvJP2107butWUMnkArVrWzZm3Ka+xL5fJ2z7m1CPm4RrqsePkFSJvFIQlko6ODhCrS5C4586dC52dndHRZbYuXUCrtMJExo6D3pYnmR2H7WgWXxvSdbnGxlxwjMLkjZ2T9ua2NXkcRxdQeatX31eYMHnb9CA7YzF528HZ5Ci6XGNyDrLsUuRta5Jp4+gCKkomuH+Jz+u6VV056H3wsvJzYDuWx9C1BF2usTXPQuSd9MLS1qTlcXQAlV9Urmq9GJbcMN7FVAuNyUFfCDbvOrEdvTNJ7gnpcE1uoSU6FCJvHA83oWpubrZa347TUwfQv\/5JH9zT+6uoe5VKJjhfDvoS3u1RV7ajR8YoOBUdrikoulC3QuRdtS1hG7\/xaARO1UomTN6FfNrLTkzeXpol16SCIO9cGhtunAWovMpk+1cmw\/WTGg3PiFY8Bz0tnq6ksR1dIU83bhbX0I2kJ6lQ5q0n2k6rLEB\/uHcQ\/vJHL0aT8XXb1zSkOOjt+JHpUdiOphE2Lz+La8zP4PQRtMm7qgcQV3WViTATB73tkDAzHtvRDK42pVaWvG2ClGesNEDlkslt08bBui+25BHtRVsOei\/MUHoSbMfSEDoXEAx5V2FXQXmJYBVLJuitHPTOY5ZkAmxHEhidCgmCvKuyq2DVSyZM3k5jlXRwJm9SOJ0IC4K8q7KroFgiWIV9u5O8kYPeSZySD8p2JIfUusAgyLsKuwrK9e6qfZgjeyUHvfUYNTIg29EIrFaFBkHeiJjvuwrKG1FVtd7NZROrsWl0MCZvo\/BaER4MeSNaPu8qGEK9m8nbSkxaGYTJ2wrMRgcJiryNIqUpPA7Qqp1TmaYqB72mI3jejO3ouYE0psfkrQFSniZZ5L185kRYPrM5j0iv2nLQe2WOwpNhOxaGzpuOTN6appBLMpMnT4auri5obBy+L0kcoN3PvAaLuw9EI1X5ZSWXTTSdpQLNmLwrYKSMKTJ5a9gQV7OsXr0a5s2bBy0tLdHL0T179sDatWuhoaHhNAlxgMpnVVb5ZSWTt4azVKQJk3dFDJUyTSbvAjbELHzNmjWwYcOGYdl3GnlXcQtYFR4O+gIO42EXtqOHRsk5JSbvnIBh8zyZd0gvKznzLuAsnnZh8vbUMDmmFQx5i9PiVd3T6tM5cIqayoc+iFPqVRkIKF69vb3Rv5\/99btw+\/bXov9edM0ouP3To\/IO61X7gYEBGDdunFdzop4M60iNqBt5odpxxowZNUD7+vrcgBszqvaWsHJfQapz5syxcgyaGG\/58uUwZcqU09RQ74ahfJwjlOSMzZtYKTURtmMp+LzoHETmbfsA4qSNsNCiKqDyy8pj373BC6OXmQQHfRn0\/OnLdvTHFkVnEgR5o\/JYh+7v7wfMhqkv9eaALyyXLVsG69evj1afyFcSeYfwshL15KCn9i438tiObnCnHDUI8rZxALHup\/cyoKG9rGTypgw9t7KYvN3iTzF6EORNAQSVDBVQsQ1s1b+s5Jo3lYf4IYfJ2w87lJkFk3cZ9GL6yoDKJ+dU\/ctKJm9iR3EsjsnbsQEIhg+GvMVLxJ07d0JrayssWLAA7rnnntgPaQhwSxQhAxrSZ\/FM3ia9xr5sJm\/7mFOPGAR5y6s\/JkyYAN3d3dGn6z09PYmfsVMDKeTJgIayDayMFQe9Kc+xK5ftaBdvE6MFQd7yapChoaEaeeMi\/aTP2E2AiTJlQK+460nAl5ahrDRB\/TjoTXmOXblsR7t4mxgtCPJGYNatWweDg4Nwyy23wMMPPxyVTRYvXhyVUEwsH0wyhgxoCGdWqnpy0JsIQ\/sy2Y72MaceMRjyRmDUT+Q7OzutfHEpG0UAKi8TDGWlCWfe1OHnTh6TtzvsqUYOirypQCkjJ468Q1lpwuRdxjP86svk7Zc9isyGybsIail9BKDyniZM3sQgGxbHxGYYYEviQ7cjkzexIwlA8eQcXCqIVwh7mgiYQg8IfrogDgiH4kL31WDIW17nLfxl0aJFVl9W4rgC0BCXCTKxOWQi4qFDJ7Z68NUgyFsQd1NTU42sxd\/QiHHHlRHHQk2cADTElSb1EBCso6nIsC839BtUEOSdtCVs2nFlplwJAf3mlvIhAAAKyUlEQVTpf\/wScI03Xm1Xj4UH2i4zNZx1uaEHBJO3dZcyNmDovhoEeaP1cZmg+LJSHAqMa7+bm5utLhdEQH\/478\/BrAefj5wSiRsJPJQr9IBg8g7FU8P\/oCwI8k7bElZ2RTwSbceOHUa9UyXvkFaaMLEZdR2rwvkmbBVuI4MFQd5GkCkolMm7IHAedWNi88gYJaYSuh2ZvEs4R1xXBPTm1T8JcpkgZ97EzuJQXOjEVg++GhR541FoHR0dtZBw9Xn85e0\/hscPHQ9qQyoBKge9Q8YlHJrtSAimI1HBkDe+nMSXll1dXdDY2AiiDo6nu9vemGrkgr8PbjdBJm9HEWpoWCZvQ8BaFBsEefu2VPD4F7oiE067eBT0LL7SojnND8VBbx5jGyOwHW2gbHaMIMgbIfIl826+\/Bp4+7PrIquFtJsgZ95mA9G2dCZv24jTjxcMeSM0PtS8x19zM7wzbRmTN72vWpPIxGYNaqMDhW7HoMjbqCdoCpfJe983r41eWoZ0hR4QaCvWMQyPDd2OTN7Eftr0Z1+Hdz8xK5Ia2gc6TGzEzuJQXOjEVg++yuRNHEAXfPGv4L+bp0dSOfMmBteSOCY2S0AbHiZ0OzJ5EzvQx778PXjv\/EuDXONdD9kM60gcEA7FMXnbBf+MU6dOnbI7JO1oTN60eLqQFnrQ8w3KhVfRj8mZNzGmoe7jLWBiYiN2GEfi2I6OgCcclsmbEEwUJcg7tH28mbyJHcWxOCZvxwYgGJ7JmwBEWYQg7xA\/0OHHbWJncSiOydsh+ERDM3kTASnECPIO7RAGzryJHcWxOCZvxwYgGJ7JmwDEuMw7xDXenHkTO4tDcUzeDsEnGrruyRt3Ipw7d24Nzi1btgDuRKheeB7m\/PnzYXBwMPoJT+UROxgyeRN5oydimNg8MUTJaYRux7omb9yNcNWqVfDtb3872kYWiRw3uIoj5bgzMuN8S5RNQvxAhzPvkmziUffQia0efLWuyVuNJbEHOO7\/rWbfuOlVf39\/5t7gSN64nwmSd4gXB30YVmU7Vt+OTN6SDbE0smzZMli\/fj20tLScZl3MyDdt2pRZXmHyrn5QMLFV34acedu3obMvLE+ePAkrVqyAqVOnwuzZs0\/TXP0NSyhLly6FzZs3DyN5JO8QD2EQgDCx2Q8KEyOyHU2galcmZ97\/f2iCIOempqbMsgiaJ43okbzPfONFeGrVTLuWtDTawMAAjBs3ztJoboZhHd3gTj1qqHacMWNGDaq+vj5q2ArLs555pxFxkhaiT1tb27DaOJJ3qB\/o1MOjKOtYOHa96xj600VdZ966xK2ekZm2KiV08vbNYUwwButoAlX7MkO3o2\/6Wc281bXbwr1wrfekSZOgvb0dVq5cGdW15bZYXomrd2N\/JO+zn\/s7+PCRJ+x7K4\/ICDACdYVAXZdN6srSrCwjwAgwAoYQsJp5G9KBxTICjAAjUHcIMHnXnclZYUaAEQgBASbvEKzIOjACjEDdIcDkXXcmZ4UZAUYgBASYvEOwIuvACDACdYdAZclbXkq4aNEirS81fbaujj5infzOnTsjVVpbW2Ht2rXQ0NDgs2q1uenoKCuStveNjwrr6ie3S9rq2Ef9cE66OspbP4cQn8IeuOdSc3PzsC09XNirkuQt70aIzp+0R4oLQIuMqasP7rSIF+4Fo\/vBU5H5mOijq6MYW+j37LPPJq7xNzHPojJ19VN30tTdPbPovCj76eoo33Rxa4eqx6dM3LhZXmdnJ5N3UcdC51izZg1s2LAh2hccA2DPnj2VykLVDLOIPlXSO6\/NULef\/\/zn8MILL8TuOlnUd0z109UPM9LHHnuskk+KeXSU9+nH\/8YLt36u4iXvxYTz58y7hBXVgxrSPp8vMYy1rkX1qVJQ5NFRkAQ+bqOOcVsGWzOO5kC6+uFN6ejRo9Db2wv79+9PPCFKc1irzXR11M3QrU6eaDAum5QEUs04q07eRfSpms55dMQAmT59OowePTpxv\/eSLkTeXVc\/bHf\/\/ffXSkFVugHr6ojgCgLHG1TSUYfkRrAgkMm7JMi6GUDJYax1z6tP2v7m1iadcyBdHeWyQpVeWOrqp9a4q3QT1tVR1alKN6gst2byzkIo43fd2lvJYax1z6NPlYJdBlBXR\/UEJZSRtjGZNSMR+aQuAfqiV1EbYj9R466qz8bZgMm7pGeGVlPT1afKQaCro0oWScfklXQh8u66+snbHYuVGLqHkpBPOqdAXR3jMu\/BwcHKLiiQYWLyzuk0cc1115sSDGVFRJI+srPEZaVVWuuto2NVyRvnrauf3K5K9sujI5aHOjo6InNWbS17WsAzeVuhQx6EEWAEGIFwEajkRzrhmoM1YwQYAUZADwEmbz2cuBUjwAgwAl4hwOTtlTl4MowAI8AI6CHA5K2HE7diBBgBRsArBJi8vTIHT4YRYAQYAT0EmLz1cOJWjhGQt8PNs8Woj1\/3ie1STX58JC9HDOnzdMdu6NXwTN5emcPPyeTZvTBP2zzaFv1AyVfy7u7uNv7RirjhtbW1wZQpU\/LAzW0rgACTdwWM5HqKeQg5T9s8eqmflev2ZfJeAUzeut5SrXZM3tWyl7HZqqf0iNKEfCKK+BpwYGAA5s+fD\/jJM15pbVHuwoULo+1P8Up7hJd3opPbynNIKjXIZQK5DZL3iRMnon\/wBCK1vywbxxQb7Yu9WEaOHBn1E\/OWv3JFvbF\/V1dXtK980hxUo6k3InWOaV8kqjejtJslZ97GwsULwUzeXpjB\/STk3e7UoJcJAmeKJ6OIbE7d+S+u7dSpU6OTR9J2CVRPBlJ3Tkwrm6in08htv\/\/970fku3nzZmhpaYn2Bxf7bOCY7e3tsHLlyug3ud\/Q0FB0g1qyZEnt1BT5dzx6DnE4cuRIRN544U0KN2PCEkXafOPIG09okW8QSXuBMHm7jxVfZsDk7YslHM9D3Wdank5adqeSrNwWM3T5hCCUmbQ3hErscWQun84izy+NKPOQHc5969atERkjeaubYqmy5HEPHjwIch07LeuNI2+ZrNNucnn04czbcVAZHp7J2zDAVRIvbyYklxdU8pZLB\/iIj5c47UZui6WSuXPnDoMgbrWISsB5yDvt5pJGduIpQhzofN1118Hbb78dS95xZ4bKc969e3dtIyZZ4bjzDuPIG\/uILVTlnQfxiUC+mLyrFFFm58rkbRbfykqXyws9PT21M0Ixm5Yz0rSySVzmnQSIi8wbby5yNq+WTcpk3mmG58y7smHh1cSZvL0yh7vJxGV0\/f39UTaolkKwFnz33XdHtV3sJ9eU02reojY9Z86cYadvU9a85RvB9u3bI1BFVqs+GSxdujSqh4u9tUUNO65skqfmLV5eCpzUMo9cYlExlG+cWFtXS1iitCPq7vj72rVrQW3LZRN38WRjZCZvGyhXYAx1tYm84kEQ0ZgxY6KSAr4ExBdseG3cuDH6f\/GiTm2LbeTVJmkf2CStNkEZWeu85ZUe2F5++ZdE3nLZBMtE+OISdcESEF5xB0GIkhG2R73wqSRutQn2jyuZCF1U8saaN9441EOJ1RKKjJGYw759+yLyVl\/AMnlXIPBKTJHJuwR43LW+ESi69jyr5k2FKpM3FZJ+ymHy9tMuPCsPEYhbTlnkCDMmbw+NW8EpMXlX0Gg8ZTcIqGWdokeYqXubqHV5Cu14bxMKFP2W8X99v6XCQLl+wAAAAABJRU5ErkJggg==","height":0,"width":0}}
%---
