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
time_start_motor_control = 0.025;
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
%[text] ### 
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAhUAAAFBCAYAAADAE10CAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQm8T9X6\/x8ylukclDjkXEMRUYbU1UiJX0q56VK3gUqZSsZUyJQhqQgNku6\/VNwSud2KNGg0SxEZMp6QeQrxfz1L67TPPvv73dPz\/e793eezXy8vnLP2s9d+P+u71uf7rGetle\/kyZMnCRcIgAAIgAAIgAAI+CSQD6LCJ0HcDgIgAAIgAAIgoAhAVKAhgAAIgAAIgAAIiBCAqBDBCCMgAAIgAAIgAAIQFWgDIAACIAACIAACIgQgKkQwwggIgAAIgAAIgABEBdoACIAACIAACICACAGIChGMMBIlArt27aIOHTrQjh07aPLkyVStWrWEvd6aNWvo7rvvprJly9KkSZMoPT09Yc8yGk7mOzp5ocOHD1Pfvn1p0aJFCWfupD5+y4wYMYJeeOEFKl++vKf3eeutt+iRRx6hjh07Up8+ffxWR\/R+3XYaNWoUurqJviiMeSIAUeEJG26KMoFkDrhmUVG0aFE1uM6aNYveeOMN4o47EZf5HTMyMpLy3FjvogdRp+\/sZdD1co8X9t988w21a9dO3RpFUcHvpd\/Rqb+8cMQ9qUkAoiI1\/YZaR5SA\/sael0SFFjiVKlWi4cOHEwsru8uLQPByj109rH6vB9yWLVs6fh+znWTV1cv78T26nW7cuDGpETav9cV9ySMAUZE81nnuScYBUr+8MZyrv6XXq1ePbrzxRrrnnntUMXNnrAedZcuW5fr2F8tGnTp14nZ2VnXT37rM3+JLly6tpkP44lB0z549VR35307s8H16asNsm3+npz\/GjRtHI0eOVFEKK176Z1ZTJkZG+j10CF7fZ+RqrMeECRPo5ZdftnyuftbWrVtzsecfmH3Ddt57770c0xjxGLENPQg\/+eSTdOutt6rnxHuuHnD1e2lfG6M8+nfaZqx7eLrJ\/Du7KQdz3fhZmrnZVrxIhTGiYW73RlHBv+OpFL7MdTP72Ph7bYMZbNiwQflXT+cZ7+N7KleurKZbjD6w4+I2upTnOsA8+sIQFXnU8Yl+bauBxDxQWnXO5gFw8+bNatDVg5r+ve6s9aBs\/r2VONH3xqsbDw7Vq1fPkVOhRYUWNbpz79atW\/aUgZmn0Y60qLCKZhjFFX\/bf+6557IHImPdtLBgGzpvJJaouPfee1UZ43uzLc3eOGVifn9dxoqdLmsWP2ZRF+u5ixcvVgOgWVS89NJLlu\/MAyVfbu6JJSzitVmu\/y+\/\/JLjObFEhVlQmNv9zJkzc9gx8o0llMyfr1gCx8zPyrZZrJht8\/\/1O9iJsET3NbAfLgIQFeHyR2Rqozu0WN+O+RuTURBYDSj8s88++0wNFFbfwPhnN998c7bo0Dbskh+twtPGb4Z6MNWJmsaB0VgPp3acigqOZjjNqTB+C+Vv98b\/n3POOWpO3xitMQsRs3CyyqnQ72e0owcbHtisnqProQdTPYAZ24GRW+\/evalLly45kmKtuBqfa3xf7Q+rSI3Zjh6o9T26nbB\/9Df4ePk0RobGb\/Tm57AYYv6xpj+sRKH5uZqbkb2xjRoFra6Lud1\/\/PHHSpgYhY0VJ6NQYlsXXXSR+kzZcYGoiEx3LfoiEBWiOGFME9CDgDmRyzj46c6Lpz+Mc+n63ieeeIIWLlyYIyxvJGwUFcbVE7rjNA7mxvti1U2XiTX9YV4N4tROIkSFMTIxYMAAYlZ65YQekIwDn\/mbpVk42SVqmr9ZG7\/9G59jZvfOO+9YRg+4PjzoWokK85QKl43Vjqy+JZvv14O7WVTEihbw86wiDLHalXnlym+\/\/RZXVJijSlY5JFY5FbEGcXPkTQsRLSpiTTlafeaMYtGqNzNygahAf29FAKIC7SIhBCREBX975EEpXtJivPyCKIsK40DWr18\/GjZsGGlxpgdPv6LC\/A2WIwTGiAHzNc\/DxxIV5rrEEnDG5btW4sIc+tcDpnFg1SLCHDGIJSqcJlSGTVRoX2gRwWLGuDzZr6iw44KcioR0nSlvFKIi5V0YzhfwO\/2howL6m66xgzMObDra4SZSYfUt0GjzmmuuscypMEcq3NrhAVN\/uzPnhOj6O53+YK8bn89TRHrAtZq28DL9YX4\/4yDvdvrDLoxv3J\/Cqu04ncowtgPt01iRCqNocjJ1JjX9YTXFEmvqwmq6jX9mjjRx29Lc4kUq3Ex\/cJ5SLC66nWL1Rzj73yBrBVERJP0IPzvRiZqxvp1xNr\/d9IfVN2B2hbbJ\/zZufqVzKsyiws4O1yVWwpsTUcH1iJcEZxwUzeH6WM+1StTkiJA56dK4IsCqmbKo0OLLnFBpnD6IlahprK85qhUvGdIqSZH99thjj9GDDz6YK6FXT7NwqN+Y\/Kh9HS+5U69EMb6\/XaIm7yviZEmpOYlSPyNWJCbW9JUV+3iigu3Eahv8O\/38WGXMwhX7VES4E\/f4ahAVHsHhNnsCTpeU8rdL\/ubFCXvGQUDPNZsHb+O3Xi\/TH\/yMeDad5lTY2bH6vXnJJZcx76hpHHDsMut1529VzumSUp2kaH6ueXWLMY\/CLE704MbLYnmgNgowczswCyCruXmrwdsqN4C\/TZsHUd2G2rdvT507d1ZtypwYbBU5MQ\/ssVp4vCWlxsHf6fSB1XOd5FQYc0L4fZ566im13Jl9YVyJYtc2+Pd8GaNd\/H+z8NGCAjtq2vd9ebkEREVe9n7A7263SiPg6uHxNgSsVk9YTb3YgbSLLNndj987J2A1hWM1JeLcIkqCQE4CEBVoEYERgKgIDL3Ig+NNcdl9SzdXAEl\/Ii5xZCTW1IvXLcUdPRSF8gwBiIo84+rwvShERfh84rZGdlNcTu1F7UAxp+8dVLl4O4wGVSc8NxoEICqi4Ue8BQiAAAiAAAgETgCiInAXoAIgAAIgAAIgEA0CEBXR8CPeAgRAAARAAAQCJwBRYeGCv\/3tb4E7BhUAARAAARCIPoG5c+dSZmZmZF4UoiKGqFi3bl1knBzUi7A4A0cZ+mAJjjIE5KygTcqwjBpHiAqICplPBjgmjCMbjlrHk1BYcYyDoxx5sJRhmXHlv2jzp\/+WMRYCKxAVGAwT1gzR6cihBUsZluAowxFCV4bj1AVZ1HnqStr19FUyBkNgBaICoiJhzRAduBxasJRhCY4yHCEqZDhCVMhwDL0VdDwyLlq\/fn2kEpBkqHizApbeuJnvAkcZjmwFLP2zhKjwzzAlLEBUyLgJnY4MR3Tg4ChHQM4SPt\/+WUJU+GeYEhYgKmTchE5HhiNEBTjKEZCzhM+3f5YjPtxAIz5cj5wK\/yjDbQGiQsY\/6HRkOEJUgKMcATlL+Hz7ZwlR4Z9hSliAqJBxEzodGY4QFeAoR0DOEj7f\/llCVPhnmBIWICpk3IROR4YjRAU4yhGQs4TPt3+WEBX+GaaEBYgKGTeh05HhCFEBjnIE5Czh8+2fJUSFf4YpYQGiQsZN6HRkOEJUgKMcATlL+Hz7ZwlR4Z9hSliAqJBxEzodGY4QFeAoR0DOEj7f\/llCVPhnmBIWICpk3IROR4YjRAU4yhGQs4TPt3+WvEU371WBbbr9swy1BYgKGfeg05HhCFEBjnIE5Czh8+2fJUSFf4YpYQGiQsZN6HRkOEJUgKMcATlL+Hz7ZwlR4Z9hSliAqJBxEzodGY4QFeAoR0DOEj7f\/llCVPhnmBIWICpk3IROR4YjRAU4yhGQs4TPt3+WEBX+GaaEBYgKGTeh05HhCFEBjnIE5Czh8+2f5Q3PL6H5a\/cgUdM\/ynBbgKiQ8Q86HRmOEBXgKEdAzhI+3\/5ZQlT4Z5gSFiAqZNyETkeGI0QFOMoRkLOEz7d\/lhAV\/hmmhAWIChk3odOR4QhRAY5yBOQs4fPtnyVEhX+GKWEBokLGTeh0ZDhCVICjHAE5S\/h8+2dZd8jXtHHXEeRU+EcZbgsQFTL+QacjwxGiAhzlCMhZwufbP0uICv8MU8ICRIWMm9DpyHCEqABHOQJylvD59s8y\/eF5ygi26fbPMjAL8+fPp3HjxtH48eMpPT3dsh4QFTLuQacjwxGiAhzlCMhZwufbH0ue9uBIBUSFP46B3r1lyxa67777qFChQjRp0iSIigR7A52OHGCwlGEJjjIcIXT9c5z\/8x66YfwSiAr\/KIOxcPjwYRo8eDBVrlyZvv76axo9enRcUWGs5dy5c4OpdIo\/dfPmzZSRkZHibxGO6oOljB\/AUYYjWwFLbyybNGmibjxe5lw60Lg3RIU3jMHedfLkSZo+fTpxpKJZs2Y0cuRIW1Gxbt26YCsdgafjW6GcE8FShiU4ynBEpMI\/xxEfbqARH66HqPCPMvkWVq9eTRMmTKD+\/fvTzp07adiwYRAVSXADOnA5yGApwxIcZThCVPjnqPeoYEtI1PTPM2kW9u\/fTwMGDKD27dtTrVq1aM2aNRAVSaKPDlwONFjKsARHGY4QFf456uWk+Q\/tpJ0Tb\/FvMCQW8p3kuYEIXywi7r77btq6dWuut3zjjTeoUaNGuX6O1R8yDQIduAxHdODgKEdAzhI+395ZGld+FNixirb\/+wHvxkJ2Z+RFhZk3IhXJa4HodORYg6UMS3CU4Qih64\/j1AVZxMee81Vs\/kja+N0H\/gyG6G6ICgtnIFIh00LRgctwRAcOjnIE5Czh8+2dpTGfAqLCO8eUuROiQsZV6HRkOEJUgKMcATlL+Hx7Y2mc+mhcpRStGN2aorTaMM9FKpw0A4gKJ5Tsy6DTsWfktARYOiUVvxw4ynCE0PXO0SgqZna6kO649iKICu84U+NOiAoZP6EDl+GIDhwc5QjIWcLn2z1LFhS8iyb\/XSm9CC197BKK2niDSIVFu4iak903fZk70OnIcISoAEc5AnKW8Pl2z9KYoPnsrefRvy4+G6LCPcbUuwOiQsZn6HRkOEJUgKMcATlL+Hy7Y2mc9tBRCrYQtfEGkQpEKtx9MlyURqfjApZNUbCUYQmOMhwhdN1xZEHRZepKmr92j7rx+bY1qG2DcurfEBXuWKZk6ag5OSgnoAOXIw+WMizBUYYjRIVzjiwoeNpDn\/PBKz5mdr4w20DUxhtEKhCpcP7pcFkSHbhLYHGKg6UMS3CU4QhR4ZzjrOU76M5XV6gbeNqDV3zw3\/qCqHDOMmVLRs3JQTkCHbgcebCUYQmOMhwhKuw5coTi7UVZNOyDUyeRWgkK\/nnUxhtEKhCpsP90eCyBDtwjOIvbwFKGJTjKcISoiM\/RuHQ0nqCAqJBrj6G2FDXlGBRsdOBy5MFShiU4ynCEqIjNUedPsLCwExQQFXLtMdSWICpk3IMOXIYjOnBwlCMgZwmf75wszQmZTgQFRIVcewy1JYgKGfeg05HhCFEBjnIE5Czh832KJYsJPd2h6XL+BG9udUW1NFvgURtvkFNh4fKoOdm2VSeoADodObBgKcMSHGU4QuieEhNfrt2jlorqqQ6n0QmjF6I23kBUQFTI9TImS+jA5dCCpQxLcJThmJdFRTwxMe6fNahx1VKuIENUuMKVmoWj5uSgvIAOXI48WMqwBEcZjnlVVDz41ir697fbckDkqY7hN1Wn684v7Qlu1MYbRCoQqfD0QXByEzpwJ5SclQFLZ5zsSoGjHSHnv88rLOf\/vIdGfrg+e4ttTYjFxLv316XMMkWdQ8sD4w1ERR5wsq8W7+PmvNLp+EDk+FawdIwqbkFwlOEY5UiFzo8wntVhpMZiwss0RyzyiFTItcnQWoqak4MCjQ5cjjxYyrAERxmOURQVsSIS\/K4sJK6snk4PNz0nxxbbEjSjNt4kLVJx4sQJ2rt3L508edKVH\/Lly0dpafbLclwZtSkcNSdLsnFjCx24G1rxy4KlDEtwlOGY6qKCoxEsFOIJCX5HPvyrd7NM18mXbihHbbxJmqjYtWsXdejQgZYtW+aGN9WpU4feffddV\/f4LRw1J\/vl4fV+dOBeyeW+DyxlWIKjDMdUFRUsIqYu2KZODbW6WGhUSitC49rWUL82HvwlRy6npaiNN0kVFT169KB+\/fpRtWrVHPlnzZo1NGzYMJo8ebKj8lKFouZkKS5u7aADd0ssdnmwlGEJjjIcU0FU6NyIcZ9upFXbDuZKtNQkWDhcUT2NbrmoXEIjErHIR228SZqoOHDgAM2YMYOaNm1K5cqVc9Sys7KyaM6cOXT77bc7Ki9VKGpOluLi1g46cLfEICrkiFlbQpuUIxxGlhyJmLF0O81Z9VuODanMb81CYlTr6nTuWWckJRoRj3rUxpukiYrdu3fTmDFj6Morr6SGDRtSsWLF5Fq3sKWoOVkYj2NzYex0HFc+ZAXBUsYh4CjDMehIhc6J+OLnPTTKYrmnlYgIMhoBUSHX7rIt7d+\/nx555BH673\/\/SwULFqSWLVvSP\/7xD6pfvz4VKFAgAU\/0bhKiwjs7453owGU4Bt2By71F8JbQJuV8kCyWxqTKWct30MptB2JOZei340jErfXL0WVV01QkIhm5EV7JRm28SVqkQgPniMW8efNo2rRptGjRIipatCi1a9eOWrVqpXIt8ufP79U3YvdFzcliYFwaSlan47JaKVkcLGXcBo4yHBMtdFlIjJ23kX7Kip0LYRQQ\/O+hrapR7fLFQi0grOhHbbxJuqgwQmWB8dVXX9H\/+3\/\/TwmMsmXLqvyJG264gcqXL0+8nDSIK2pODoJhojudoN4pqOdiMJQhD44yHKU+35wDwVEEPpSLD+cyHswVq6Zc\/vJqadTzmsqqSJijEE5oR228CVRUGIFzIicLjGeeeYYKFy5MkyZNovT0dCc+ES8TNSeLA3JoEB24Q1AOioGlA0gOioCjA0gOi7hlyQKCRcObC7bZTl8YoxB\/r1KK2jY4O\/TTGA6x5SoWtfEmcFFx\/Phx+v7772nWrFn0n\/\/8RwFv3bo1Pfzww4Elc0bNyV4bu9\/73HY6fp8X5fvBUsa74CjDMV6kgoXD8i0H6MXPN9HG3UccRR90xKFx1TT6Z\/1ykRUQVvSjNt4EIiqOHj2qhATnVXzwwQd05MgRatKkCbVt21YlbnKeRZBX1JwcFEt04HLkwVKGJTj656gTJ6fPX0mnFStNk7\/c4lo88AZTd11ans4sXjhPCQiICv\/tL9sCb9O9YMECmj59On300UfEq0EaNGhAd9xxB11++eVUvHhxwaf5MwVR4Y+fvhsduAzHeN8K5Z6QNyyhTbrzs85xeOO7bfTV2j2Opy105IH\/vu78MnR97bJ5XjzEIh+18SZpkQq9TTdHJdq0aUPXX3+9SswM4xU1JwfFGB24HHmwlGEJjrk5auGwfudhenfpdlq345An8aBzH\/gJjauWknFYHrAStfEmaaLi999\/p3379lGZMmUCW9XhtH1GzclO31u6HDpwOaJgKcMyL3Nk8cB\/+OyLTbuOuBIOxsjD9ReUpetqlqGsrG3UsGZmyq++kGlZ3q1EbbxJmqjgSAXO\/vDe8FLxzrzcgUv7CyxliEaZo4448N8zl22nVQ72eLCiqg\/V4tM5tZiwWrYZZZYyrc2ZFYgKZ5xylWJR0a1bN7WTZoUKFRxZ2bJli1oVwvtYJPOKmpOTyc74LHQ6cuTBUoZlqnPUwmHtzkP07pLttGHnYdcRh2yhkFaE6lcuSVefe2rpvtspi1RnKdOi\/FuJ2niTtEgFb3TVv39\/2rx5sysvZGRk0NixY13d47dw1Jzsl4fX+9HpeCWX+z6wlGGZChz1hlDvL99B\/\/thp3rx+Wv3uAagIw7nljuDbqxzZtyog2vjRJQKLL28V7Lvidp4kzRRkWxH+Xle1Jzsh4Wfe9Hp+KGX816wlGEZBo46t+GsEoXo2U820sbfvEUbjBGHhpkl6crqpyIOyTrrIgwsZVpFsFaiNt5AVFi0p6g5OaiPDDodOfJgKcMyWRw52nCSiKYtylJTFG42gTK+qc5l4H0d2tQvp7an5p\/pvSJkqHizkiyW3mqXOndFbbyBqICoSNinD52OHFqwlGEpwdGYEMnTE8s37\/csGozRhipnnk6tLzwraZEGv0QlWPqtQxTuh6iIghdt3iFqTg7KZeh05MiDpQxLpxw50lC6WEEaN2+jWn7pNdKgRQP\/fdW56WoTqMIF8qeMcIhH3SlLGc9F10rUxhtEKhCpSNinFZ2OHFqwlGH55bI1VLFiRWVs9ood9MH33hMhdY30FAVv\/sS7R6adXjB774ZUP0ETokKm3cWzAlEhyJhXgvDZH8eOHVPHnW\/dupVq1KgR+JbdUXOyoMtcmcJA6ApX3MJg6ZwlRxmOHj9B7yzdrpIg\/UQZjNMT55QuSt2urqQiDWZB4bx20SmJNinjy6iNN4FFKpYsWUIPPPCA2qqbd9t8+umnaerUqep8kHHjxlH16tVlPObBStSc7AGByC3odEQwKiNgSTlOu3z1q6208Je9io2X5ZZmUcCJkLUqFKMWtU6dUQHRYN920SbtGTkpEbXxJhBRwaeU8p4VF154oTqdtFevXtSvXz\/KzMykl19+mTZs2ECDBg2iQoUKOfGJeJmoOVkckEOD6HQcgnJQLOosdfIjo5i7ahe9u+RX34JBRxn4b56a4HyGg3t2ULlyZ+eJ6QkHzcpXkai3SV9wXNwctfEmEFFh3LK7dOnSObbvXr16NQ0dOpTGjBlD6emn1l0n+4qak5PNTz8PnY4c+VRmaVwt8cO2A2q1hN\/kR6NgOBVlKE61KxRT4iFelCGVOcq1JhlLYCnDMWrjTSCigg8W69mzp9q2u3z58jlExYoVK2jUqFH0zDPPUFpamozXXFqJmpNdvr5YcXQ6YihDPf2hd4D8eOVv9N7S7aIRBhYMvCNki1plqOBp\/ldNoE3mjTYp95aJtxS18SYQUXHy5EmaOHEirVmzhrp27aqmOh555BE644wzaPjw4VS5cmV68MEHqUCBAon3qMUToubkQCAiD0AUe1CDIQuGiulFaNL8zbR0034RwaCjDCwY2HanKyrS7kPHle3szZ4MeQ2SIIPiKPkOYbEFljKeiNp4E4ioYFccOHBATXH8+9\/\/puPHT3UofLVo0YIGDBigEjiDuqLm5KA4otORIy\/NUkcX9hw6Ri\/N30K\/CKySyCEK0opQpdJFqeUFZemMQqeFJvlRmqOch1PPEljK+Cxq401gooLdwRELXkbKjZOvs846i6pUqUL58\/+1ZEvGbe6sRM3J7t5erjQ6nWBYasEwc\/kO+ujPA6n8Lqs0C4ZqZ51OLS84kwrkz5cdYUiFPRnQJoNpk3JPjZ6lqI03gYiKEydO0N69e5WosLp42qN48eKUL9+pDivZV9ScnGx++nnowOXIM8vTSp6tDG7ff5Re\/XqryB4MuobGMyYurVqKzkkvapv0KPd2ybOENinHGixlWEZtvAlEVPDqjw4dOtCyZctieoVFxf3330933XUXFS1aVMZ7Dq1EzckOX1u8GDode6TGlRHlSxWmFz7fRCu3HVQ3+tl\/wUoscP5Ck\/NK05nFTy3VTtZplvYUklcCbVKONVjKsIzaeBOIqGBXLFq0iJ577jm1AuTcc89V3uENsfhnPXr0oGLFiqmci\/r166tNspJ5Rc3JyWRnfBY6nVMbNvGfxRv30ZyVvyk8ElMRWhSov9OKEB99XaXs6ZGMLki2X7RJOZpgKcMyauNNIKLi8OHD9Pjjj1Pbtm2pXr16OTzDYoN31hw8eDCtXbtWiYwXX3xRxnsOrUTNyQ5fW7xYVDsdY3Rh8aZ99FPWQZF9F6yiC5VLF6UWtcvQgd1\/bdqUCrkL4o1JyGBU26QQHldmwNIVrpiFozbeBCIqjJtfVatWLQdsXmY6bNgwGj16NP3222\/q35MnT5bxnkMrUXOyw9cWL5ZqnY5xV8f3lm+nVdsSJxZ4KoI3aqqUXtTRyohUYynemIQMgqMQSCwZFwMZtfEmEFHBZ31wpKJOnToqWqFXe3Di5ttvv03z5s2jp556ir777jt68803EakQa77JNRSGDpyFAn+z1ysipi\/+ldbvPCy2hNIYXdD7LlxWLY0ySp06P0Jq34UwsExu60nM08BRjitYyrCEqJDhSLwdN+dKnH766dS4cWO10dXChQvp559\/VrkUnJzJSZpPPPEE3XzzzUJPdWYmak529tbypZLR6bBY4IunIeb8KJuzoEUBiwXec6Fq2aJU\/5yS6nmNq5bKPuAqGdMRyWAp3wLCZxEc5XwCljIsozbeBBKp0K7gaRA++vyLL75QS0wvvvhiat68uTqhNCsri7Zv3061a9dO+r4VUXOyTNN3b8Vrp2PMWfh09S76bv2p0yilEhyNYoGnIUqdXpA6XpahREJYT6j0ytK916J9BzjK+RcsZVhGbbwJVFRYuYQb6pQpU9QKEF5WGsQVNScHwZCfqTsdPVgbxcK36\/fSZ6t3iYoF414LLBbqZBSnmmcXU89I9eWT6MBlWjE4ynA0fr7lLOZNS1EbbwITFTz9wcLhhx9+yNWSLrvsMho3bhxERQp9xnTOwprth2jW8h20bschWrfjAG3d99cW7F5fxywW6lUqQUULnZanlk9iMPTaenLeB44yHCEq5DhCVAiwPHr0KPXv358KFy5Mf\/\/73+n111+nm266iXinTU7S7Nu3L1WoUEHgSd5MRM3J3iicusuc6Pjesu308Z+5C1KbM+kEx6Y1SlPZYn9tzKQjDH7qH5V7MRjKeBIcZThCVMhxjNp4E0ikYvfu3dSrVy\/q3bs3MdARI0aog8QuvPBCmjVrFm3evFntpoltuuUarhNLOtow4sP1at8FP6KBowt8UNzfyhaji84pQU3PK509BaGnQZKR4OjkvVOhDAZDGS+BowxHiAo5jhAVAizN+1TwnhQsKK6++mp1HPqzzz6r9qcoUaKEwNPcm4iak80E9C6PUxds8yQejNMRl1dPo0aZpdQj9IoI\/Xt04O7bXqw7wFKGJTjKcISokOMYtfEmkEiFnv7glR233nprdnSiS5cutHjxYrXx1fPPP09paWkxPcfRjrFjx6qpE75uu+026tq1q+U93JHw8tRNmzZl2+vEWDzhAAAgAElEQVTYsSP16dPH0n7UnMwRCBYQX67dk70M0u4joRIb04rQ36umUdsG5bKnQdxEF9CB21F2\/nuwdM4qXklwlOEIUSHHMWrjTSCigt2xYsUKNcXBQoCXknbq1EmFy3\/99Vdq3749de7cWe1dYXXxJlmcyMnihIUI38dChEUI\/988bcJninz88cdqysXJlEoqO1lPLfAUxtQFWbYtn0XCVeemU\/cm53gSDujAbRGLFMBgKIIxe0WSjLW8bQVtUsb\/qTzeWBEITFRwZVgM\/PHHHyphc+PGjTRz5kyqUqWKmgbhn8W69u3bRz179lSHkdWqVUsVmz9\/Pk2bNo2GDx+e61TT2bNnZ4sVJ82AnWy85s6d6+S2wMrwCott+4\/TC9\/uoUVbjsSsR\/kSBahJlTPossyiVK\/CqR0fE3lxbkxGRkYiH5FnbIOljKvBUYYjWwFLbyybNGmS68Z169Z5MxbCuwIRFQcOHKAZM2ZQ06ZNqVy5cjmw8KZXc+bMoVatWqmTSp1cBw8eVJGKypUr0x133JHrlldeeYW+\/PJLWrVqFe3cuVMlhfLUh\/nZ+sZUUY56WiNWRIKjEENvrEa1KxTLsamTE6YSZfBNRoLiKRtgKcMSHGU4ok3KcUyV8cbpGydVVPCZH8uXL1cD+8SJE6ldu3a5lo5u2LBB5VhMmDCB0tPTbd+DV4688MIL1KBBAxWlyMzMzHEPL1MdP368stW6dWu1bJX\/z5ERq6gG3xx2J7OY6PLmylz5ESwi+JCqPs0yAxERZmehA7dtvo4LgKVjVHELgqMMR4gKOY5hH2\/cvmlSRQWv+ujQoQMtW7YsZj0LFixIDz30EN17770xcyrMN\/M0yvvvv6924mQxEisCoe\/jhE2ePhkyZAiZT0kNs6iIJybG\/bOGWn0RpgsduJw3wFKGJTjKcISokOMIUSHAMt7R507Mc8fA0Qne50JHM3bs2KF26OTTT41CwWqqhaMh\/fr1U\/tjVKxYMdcjw+ZkTr7sMnVljn0jOCrxj4vOojsalQ9FVMLKb+jAnbRmZ2XA0hknu1LgaEfI+e\/B0jmreCXDNt74faukRir8Vlbfv3\/\/fhXNuPbaa+mWW26hY8eO0VtvvaWOSucpDWMuxuHDh9UOnXXr1lXTLXr6g\/MwWFhYrTAJi5NZTMxbvYu6v\/1TNjoWEwOvr0IXVSoRWjGhK4tOR6rFI6dCiiTapBRJtEkpkmEZb6TeJ2migiMGb7\/9tjqN1O4qWbIktWnTJm6iJmfLDhw4kL755hu1TJQFht7eWwuJSy+9VO2DwcmfvKfF9OnTqUiRIiq3ItaeFmGZ\/mBBccP4JTnyJm5reDb1urZy6MUERIVdC3f\/ewyG7pkheibDLJYVtEkZvhAVHjnyZlV83gcvQ7K7eBnioEGD4m5+ZWfDz++DdjILirpDvs4RneDkS96EKpUudDpy3gJLGZbgKMORrYClDMugxxuZt\/jLStIiFdIVT6S9IJ3My0M7T12Z\/XqXVU2jsf88L2WiE0a\/oNORa6VgKcMSHGU4QlTIcQxyvJF7ixCICt4Vk5eX8rTEggUL1DkffOQ577JZqVKlRLyrY5tBOfmLn\/fQjeOXZNdzZqcLQ7eiwzFEfJNxg8q2LAZDW0SOCoCjI0yOCoGlI0y2hYIab2wr5rFAYJEK3gGze\/fudP311ysxwYmTvCGWFhqNGjXy+Er+bwvCycYIBSdjpuJ0h5k8Oh3\/bVFbAEsZluAowxGRCjmOQYw3crXPbSkQUcGJlLzy4pprrlG7W+qLoxeczPnVV1\/F3JgqkTC07WQ7OYqCAp2ObEvFYCjDExxlOOLzLccx2eONXM2tLQUiKuLtU8FHn\/Ox57zttpMdNRMBKJlOjqqgQKcj2zIxGMrwBEcZjvh8y3FM5ngjV+vYlgIRFXwgGE998Mmk9erVy1E7Pr101KhR9Mwzz0R+9Yd5lcfzbWuk3AqPeI0UHbjcRxgsZViCowxHiAo5jhAVAix5muO1115TW2sPGDCAatasqfaa4J0ueevs+vXrq4RNJ8eUC1Qnl4lkONm8D0WqJ2Va+QEduFzrBEsZluAowxGiQo5jMsYbudraWwokUsHV4rwKPtjrxRdfVDti8sW7W95+++1qt0xeDRLUlQwnd5\/2E035eqt6Rd5\/gqMUUbvQgct5FCxlWIKjDEeICjmOyRhv5GprbykQUcEHgB06dEgJB54KWbt2LfEJplWrVqXSpUsHFqHQuBLtZD4YjHfL5ItXeix97BJ7T6VgCXTgck4DSxmW4CjDEaJCjmOixxu5mjqzFIio4ETNe+65R+VM3HbbbXTJJZdQ0aJFndU4CaUS6WTjtAcLijCeLiqFGB24FEnsXihFEm1SiiTapBTJRI43UnV0YycQUcGHevEqjw8++ICmTZtGfC5I8+bN6c4776Rzzz2X8ufP7+YdxMsm0snD\/7eeRn60QdW597WVqe91meL1D4tBdOByngBLGZbgKMMRkQo5jokcb+Rq6dxSIKLCWD0tMN577z0lMAoVKkQdOnRQEYzChQs7fxPBkolysnG1R5SnPbQr0IHLNUqwlGEJjjIcISrkOCZqvJGroTtLgYsKXd2jR4+qo8s5cZOPNp80aVLk9qm44fklNH\/tHvXKUVs+atXs0IG7+zDGKw2WMizBUYYjRIUcR4gKOZZq2mPJkiU0depUmjt3LpUpU0at\/rjhhhuofPnygSVsJsLJxuTMxlVK0czOFwqSDKcpdOByfgFLGZbgKMMRokKOYyLGG7naubcUSKSCxcTAgQNp5syZKkGzdevWdMstt4Qin4IRJsLJxigFr\/bg6Y+oX+jA5TwMljIswVGGI0SFHMdEjDdytXNvKRBRsXv3bnruueeoWbNmdNFFF6k8ijBd0k425lJcVi2N3nugbpheN2F1QQcuhxYsZViCowxHiAo5jtLjjVzNvFkKRFR4q2ry7pJ2sjFKEcWdM2N5Bh24XJsFSxmW4CjDEaJCjqP0eCNXM2+WICosuEk6OS\/mUmik6MC9fSit7gJLGZbgKMMRokKOo+R4I1cr75YgKhIsKvJqlAKdjvcPJUSFLDujNYgKObZgKcMSokKGY6itSDnZmEuRV1Z8oANPTNNGBy7DFRxlOOJLgxxHqfFGrkb+LAUSqeDVHzNmzKCmTZtSuXLlcrxBVlYWzZkzh1q1akXFihXz93Ye75Zy8tQFWdR56kpVi7yUS4HpD48NL85tGAxlmIKjDEeICjmOUuONXI38WUqqqOBDw5YvX047d+6kiRMnUrt27ahChQo53oCPP581axZNmDAhpTe\/ymu7ZyJk7++DaHc3BkM7Qs5+D47OODkpBZZOKNmXgaiwZxSzBB8kxltwL1u2LGaZggULqqPP7733XnUUehCXhJONCZp9mmVSn2aVg3iVQJ+JTkcOP1jKsARHGY6IVMhxlBhv5Grj31JSIxW6uiwuevToQf369aNq1ar5fwthCxJOzoubXZndgA5crmGCpQxLcJThCFEhx1FivJGrjX9LgYgK\/9VOrAW\/Ts7rCZraO+jA5dopWMqwBEcZjhAVchz9jjdyNZGxlDRRwbtojhw5ku677z4qVaoU9e\/fnzZv3mz5FhkZGTRo0CBKS0uTeUuXVvw62Tj1kVe25LZCjA7cZcOLUxwsZViCowxHiAo5jn7HG7mayFhKmqgwrvjgVR1vv\/027d271\/ItSpYsSW3atEnZ1R966iMvHG8erxmiA5f5kKIDB0c5AnKW8PmWYQlRIcMx1Fb8OBlTH3+5Fp2OXDMHSxmW4CjDEUJXjqOf8UauFnKWkhapMFZZLy3lv62uwoUL0wUXXED8dxCXHyfn9b0pjP5CBy7XesFShiU4ynCEqJDj6Ge8kauFnKVARAXnV5hzKvbt20f8geclpU2aNKGhQ4emZE4Fpj4QqZD7eIKlNEuICjmiYCnDEqJChqOllcOHD9OUKVPorLPOoptuuimBT4pv2o+T0x+ep4znxW25zVTR6cg1YbCUYQmOMhwRqZDj6Ge8kauFnKVAIhXxqr9jxw4aPXq02sOiRIkScm\/qwpJXJxtXfeTFbbkhKlw0MpdFMRi6BBajODjKcISokOPodbyRq4GspVCKCl5O+sQTT6TcNt15+URSq2aJDlzuwwqWMizBUYYjRIUcR4gKAZaxEjU512LatGlUvnx5tU9FoUKFBJ7m3oQXJ+Osj9yc0YG7b3ux7gBLGZbgKMMRokKOo5fxRu7p8pYCiVRYJWrqV6tduzZ16tQp1+ml8q8e26IXJxunPmY8UJcurxbMxl3J5GT3LHTgdoSc\/x4snbOKVxIcZThCVMhx9DLeyD1d3lIgosL4GkePHlURiRMnTqg\/QR0iZqyTFydjKSkiFfIfz78sYjCUoQuOMhwhKuQ4ehlv5J4ubykwUbFp0yZ67LHHqGbNmtSnTx\/iQ8Y6duxIdevWpa5duwaWpMmIvTgZS0khKuQ\/nhAV0kwhKuSIgqUMSy\/jjcyTE2MlEFFx\/PhxGjFiBB06dEgJiHLlyqkoxYYNG9T+FPXr16f777+f8uXLl5i3trHq1snYRdMaKDodueYLljIswVGGIyIVchzdjjdyT06MpUBEBedU9OrVi3r37k3Vq1fP8WYrVqyg5557jp566qnAohVunWzMp+jTLJP6NKucGG+lmFV04HIOA0sZluAowxGiQo6j2\/FG7smJsRQ6UbF69WoVrRgzZkzKLCkd8eEGGvHheuUh7E+BkH0iPqoYDGWogqMMR4gKOY4QFQIs9fRHVlaW2q67bNmyyurBgwfp2WefpSNHjqh8i1RZUop8Ckx\/CHws4prAYChDGBxlOEJUyHGEqBBiyTtnPvroozRnzhzKzMykIkWK0M8\/\/0z16tVT+RYVK1YUepJ7M26cjHyK2HzRgbtve7HuAEsZluAowxGiQo6jm\/FG7qmJsxTI9Id+HY5YrFq1ihYvXkx8oFjDhg2pTp06gZ1OquvlxsnIp4CoSNzHE1NJ0mwhKuSIgqUMSzfjjcwTE2slEFHBUYqRI0eqTa44ShG2y42TX\/9uG3V9c5V6BeRT5PQkOh25lg2WMizBUYYjIhVyHN2MN3JPTZylQEQFb9P95JNPUsuWLdV0R9guN05GPgUiFclovxgMZSiDowxHiAo5jm7GG7mnJs5SIKKC96Tg\/IkhQ4ZQq1at6Mwzz8zxhoULF6YLLrggsGkQp05GPkX8hokOXO6DC5YyLMFRhiNEhRxHp+ON3BMTaykQUcG7Z3bo0IGWLVtm+XacVzFp0qTQLylFPgVERWI\/nn9Zx2AoQxocZThCVMhxhKiQYxlaS06djP0pICqS1YgxGMqQBkcZjhAVchydjjdyT0yspaRFKnjK48CBA3T66adT\/vz5ae\/evXTy5EnLt+PtuUuWLKnKBXE5dbLOp+A6Ln3sEqqUXiSI6ob2mejA5VwDljIswVGGI0SFHEen443cExNrKWmigqc8evToQf369aPSpUtHYvoDSZqIVCT244npD2m+EBVyRMFShiVEhUeOUYtUIEnTviGg07Fn5LQEWDolBaErQ8reCtqkPSMnJSAqnFByUIanPrZu3Up8BHqjRo3U1MjSpUvV5lfFixd3YCFxRZw42Zikif0prH2BTkeujYKlDEtwlOGI6Q85jk7GG7mnJd5S0qY\/zK\/ywQcfqFNK77vvPnX8OZ9c2rNnT\/rpp59o4sSJVKtWrcS\/fYwnOHEykjTt3YMO3J6R0xJg6ZQUIhUypOytoE3aM3JSwsl448ROWMoEIioOHz6sciuaN29O11xzDXFiJl88RTJ16lT6\/vvvadCgQaE+UKzNi8tpzqrfVL13PX1VWPwZqnqg05FzB1jKsARHGY6IVMhxhKgQYGlM2qxWrVoOi2vWrKFhw4bR6NGjQ71PBZI07RsCOnB7Rk5LgKVTUohUyJCyt4I2ac\/ISQmICieUbMpw\/kTfvn3plltuoSuuuCJH6UWLFtH48eNpzJgxVKJECYGnuTdh52QkaTpjik7HGScnpcDSCSX7MuBoz8hpCbB0Sip+ObvxRuYpybMSyPQHv94777yjDhXjnIqrr75a7V\/x7bff0tNPP0133XUX3XHHHdnTIsnDcepJdk7GTprOPIJOxxknJ6XA0gkl+zLgaM\/IaQmwdEoKokKGlI0Vzp+YPXs2DR8+nLZt26ZK8\/4Vffr0UeeBFChQICn1sHqInagwRiqw8iO2m9DpyDVhsJRhCY4yHNkKWMqwtBtvZJ6SPCuBRSqMr8jTIceOHQt0F01jfeycbFz5gZ00ISqS8XFFBy5DGRxlOEJUyHG0G2\/knpQcS6EQFcl5VedPsXMykjSdsUQH7oyTk1Jg6YSSfRlwtGfktARYOiUVv5zdeCPzlORZgaiwYG3n5PSH56m7GlcpRTM7X5g8b6XYk9DpyDkMLGVYgqMMR0Qq5DjajTdyT0qOpTwhKnhjrbFjx9Lrr7+uqN52221qw620tDRLyvGcbMynaNugHD3ftkZyPJWCT0EHLuc0sJRhCY4yHCEq5DhCVMixTIol3g583LhxdPToUerSpQsdP35c7YHBgoL\/rzfeMlYmnpONKz9YULCwwGVNAB24XMsASxmW4CjDEaJCjiNEhRBLXv2xbNky+vLLL1WSpvHiY8\/btGlDxYoV8\/20ffv2qe2\/u3Xrlr319\/z582natGlq5UnRokVzPSOek7E9t3OXoAN3zsquJFjaEXL2e3B0xslJKbB0Qsm+DESFPSPbEhw94PM9eIOrKlWqUJEiRXLck5GRobbpjjU9YfuAOAUOHjyoIhWVK1dWe2FYXexk4zV37tzs\/w7+5Dea8cN+9f9Zd2ZQ+RLBLX31wyEZ927evJnYl7j8EwBL\/wzZAjjKcARL7xybNGmS6+Z169Z5NxiyOwPJqeAch86dO1PHjh1z7aiZSD4jRoygF154gRo0aKCiFJmZmTFFRSwnY+WHcw\/hm4xzVnYlwdKOkLPfg6MzTk5KgaUTSvZlEKmwZ2RbgkUFn1DKf8xnf9je7LMA51S8\/\/77NGXKFJowYQKVK5c7JyKek7Hyw7kD0Ok4Z2VXEiztCDn7PTg64+SkFFg6oWRfBqLCnpFtCT39kZ6ernInrJIlbY04LMANn6MTLGD4eXzt2LGDevToQY8\/\/rilqInlZOPKj25XV6KB11dxWIu8WQydjpzfwVKGJTjKcGQrYCnDEqJCgOPvv\/+uEjR5mWeNGjXokksuyZE\/UbhwYbrggguI\/\/Z77d+\/nx566CG69tpr1QFmnBT61ltv0XfffaemQKySQWM5GWd+uPMGOh13vOKVBksZluAowxGiQo4jRIUAS57+6N+\/v0qasrqkEzU5P2LgwIH0zTffqKgICww+JbVChQqWz4\/l5KkLsqjz1JXqHpz5Yd8Q0IHbM3JaAiydkopfDhxlOEJUyHGEqJBjGVpLsZyMMz\/cuQwduDteiFTI8YplCW1SjjFYyrCEqJDhSJxXwVMgkyZNIt6zgiMHixYtooYNG1L16tWFnuLNTCwnY+WHO57odNzxgqiQ4wVRAZaJJyDzBIgKGY70ySefUL9+\/ei6665Tm2ANHTqUPv\/8cyUyeP+Kxo0bCz3JvZlYTq475GviZM1K6UWITyfFhVBzstoABJoMaXCU4YjpDzmOEBUCLA8fPqwERcuWLalu3bpqJQb\/v2rVqvTee+\/RvHnzYu52KfB4WxNWTjau\/MBBYrYIVQF04M44OSkFlk4o2ZcBR3tGTkuApVNS8ctBVAhw3LVrV7aQKF26dPa\/ec+KNWvW0LBhw9Sul3oJqMAjXZmwExV9mmVSn2aVXdnMi4XR6ch5HSxlWIKjDEd8aZDjCFEhwPLAgQMqh+Luu+9Wu1rqSAWLCs6rGD9+vJoCKVGihMDT3JuwExU4SMwZU3Tgzjg5KQWWTijZlwFHe0ZOS4ClU1KIVMiQsrHyzjvvqP0iOnXqRC+99BI9+OCDdOjQIXrmmWeoVatW6lyORG6KFa96VqICB4m5bxbodNwzi3UHWMqwBEcZjohUyHFEpEKIJW+XPWPGDOLzOH777TdltWDBgtS1a1fq0KGD5emhQo+2NWMnKjhJk5M1ccUngA5croWApQxLcJThCFEhxxGiQo6lsnT06FHi6RC++Bhyq6PIhR9pa87KyVhOaostVwF04O6ZIVIhx8zKEtqkHF+wlGEJUSHDUVnZuHEjTZ06lRYuXKi2y77qqquoefPmVLZsWcGnuDdl5WQsJ3XPEZ2Oe2YQFXLMICrAMrEEZKxDVMhwpPnz51P37t3VMtL69esrq3weB2+pzWeCNGrUSOhJ7s1YORmnk7rnCFHhnhlEhRwziAqwTCwBGesQFQIcebqjZ8+eKjJhPKWUd9Z89dVX1WZYfNhXUFMhZicb96h47tbz6PaLzxagEH0TEBVyPgZLGZbgKMORrYClDEuICgGOxn0qeBmp8QrjPhU4ndSb09HpeOOGb9hy3MyW0Cbl2IKlDEuICgGOfPT5oEGD6Oabb6Z69erlsMj7VHC04sknn7Q8llzg8bYmzE7G6aS2yCwLoNPxxg2iQo4bRAVYJo6AjGWICgGOPM3B4uHFF1+k22+\/nWrWrEmnnXaaCqc9++yzaklp7dq11ZN4r4q0tDSBpzo3YXby3FW76JYXlykDOPLcOUeICues7EqCpR0hZ78HR2ecnJQCSyeU7MtAVNgzsi3B0x8sHDh3wu6qU6cOvfvuu3bFRH9vdrJeTsoP2fX0VaLPirIxdDpy3gVLGZbgKMORrYClDEuICgGOHKnYu3evOv7c7gpDpAJ7VNh5yfr36HS8cbO6CyxlWIKjDEeICjmOEBVyLIlzK5YvX05Lly6lUqVKqRNLq1SpQvnz5xd8intTZifrPSpwOqk7lujA3fGKVxosZViCowxHiAo5jhAVQiy3b9+ulpV+++23aq+KI0eOqHBaixYtaMCAAYFugGV0snE5aZPz0mnafXWECETfDDpwOR+DpQxLcJThCFEhxxGiQoAln\/vBZ3788ccf6oTSM844Q1nNysqioUOHUuXKldUBYwUKFBB4mnsTsUQFjjx3xxIduDteiFTI8YplCW1SjjFYyrCEqBDguHv3burVqxf17t2bqlevnsPiihUraNSoUeq00mSv+tAVMTrZuEcFjjx353x0Ou54QVTI8YKoAMvEE5B5AkSFAEde\/cFbdD\/66KO5REXYNr8yigqcTurO+RAV7nhBVMjxgqgAy8QTkHkCRIUARz6ZdMiQISoSwUed62kOXhXCB4zxUtPBgwdT4cKFBZ7m3oTRySM+3EAjPlyvjGCPCncsISrc8YKokOMFUQGWiScg8wSIChmOtHr1aurSpYva9OrKK69UwoIPFNuyZQtNnDiRatWqJfQk92aMTu48dSXxjpp8YY8KdywhKtzxgqiQ4wVRAZaJJyDzBIgKGY7Kyv79+2nmzJk0Z84ctfqDDxi76aabAl35wfUyOhl7VHh3OESFd3bmO8FShiU4ynBkK2ApwxKiQoZjqK0Ynaz3qKiUXoQ4pwKXcwLodJyzsisJlnaEnP0eHJ1xclIKLJ1Qsi8DUWHPKOVLGJ2c\/vA89T7Y+Mq9W9HpuGcW6w6wlGEJjjIcEamQ4whRIccytJa0k40bX2GPCvfuQgfunhlEhRwzK0tok3J8wVKGJUSFDMdQW9FONi4nxcoP9y5Dp+OeGUSFHDOICrBMLAEZ6xAVMhxDbcVKVGDjK\/cug6hwzwyiQo4ZRAVYJpaAjHWIChmOobainYw9Kvy5CaLCHz\/j3WApwxIcZTiyFbCUYQlRIcMx1FasRAV203TvMnQ67pkhUiHHDJEKsEwsARnrEBUyHENtRTtZ71HBlcXGV+5dBlHhnhlEhRwziAqwTCwBGesQFTIcQ23FLCqwR4U3d0FUeOOGwVCOm9kS2qQcW7CUYQlRIcMx1Fa0k7HxlT83odPxx894N1jKsARHGY5sBSxlWEJUyHAMtRXtZGx85c9N6HT88YOokOOnLaFNyjEFSxmWEBUyHENthZ386cIfiSMVfLVtUI54SSkudwTQ6bjjFa80WMqwBEcZjohUyHGEqJBjGVpLZlGB3TS9uQoduDduVneBpQxLcJThCFEhxxGiQo5laC2xk1\/7aDHdMH6JqiM2vvLmKnTg3rhBVMhxM1tCm5RjC5YyLCEqZDiG2go7eehbX1HnqStVPbFFtzd3odPxxg2iQo4bRAVYJo6AjGWIChmOobbCTu444RMa8eF6iAofnoKo8AHPdCtYyrAERxmObAUsZVhCVMhwDLUVdnLzobNp6oIsVU\/spunNXeh0vHFDpEKOGyIVYJk4AjKWISpkOIbaCju5Vo\/\/0Py1e1Q9sZumN3dBVHjjBlEhxw2iAiwTR0DGMkSFDMdQWzGKCuym6d1VEBXe2WEwlGNntIQ2KccVLGVYQlTIcAy1FXZyifav08ZdR6hxlVI0s\/OFoa5vWCuHTkfOM2ApwxIcZTiyFbCUYQlRIcMx1FbYyXtaTVJ1hKjw7ip0Ot7ZIVIhxw6RCrBMDAEZqxAVMhxDbaVyrYa079oRqo7YTdO7qyAqvLODqJBjB1EBlokhIGMVokKGY6itGEUFdtP07iqICu\/sICrk2EFUgGViCMhYhaiQ4RhqK5UaNqcDjXurOmLjK++ugqjwzg6iQo4dRAVYJoaAjFWIChmOobaSceW\/6NBF7SEqfHoJosInQMPtYCnDEhxlOLIVsJRhCVEhwzHUVsq36E5HzrsBosKnl9Dp+AQIUSEH8E9LaJNySMFShiVEhQzHUFsxigrspundVeh0vLPD9IccO0x\/gGViCMhYhaiQ4RhqK+VaD6aj5zRWdcRumt5dBVHhnR1EhRw7iAqwTAwBGasQFTIcQ23lzDsm0vEy5xJ20\/TnJogKf\/wwGMrx05bQJuWYgqUMS4gKGY6htgJRIeMedDoyHNkKWMqwBEcZjmiTchwhKmuHC1YAABRMSURBVORYhtZSmfun0YnTy2A3TZ8eQgfuE6DhdrCUYQmOMhwhKuQ4QlTIsQytpfSH56m6YYtufy5CB+6PH6Y\/5Phh+gMs5QnIWISokOEYaitaVAy5sSp1uqJiqOsa5spBVMh5ByxlWIKjDEdEKuQ4QlTIsQytJS0qsEW3PxehA\/fHD5EKOX6IVIClPAEZixAVMhxDbUWLiufb1lAHiuHyRgCiwhs3q7vAUoYlOMpwRKRCjiNEhRzL0FrSogLnfvhzETpwf\/wQqZDjh0gFWMoTkLEIUSHDMdRWICpk3ANRIcMR3wrBUY6AnCV8vmVYQlTIcAy1FS0qsEW3Pzeh0\/HHD5EKOX6IVIClPAEZixAVMhx9Wzl8+DC9+uqrNHHiROJ\/N2rUiAYOHEjsIPPFg9tdd91FmzZtyv5Vx44dqU+fPpb10KICW3T7cxNEhT9+EBVy\/CAqwFKegIxFiAoZjr6tvPbaa7R06VJ6\/PHHqXjx4jR9+nSaP38+DR8+nIoVK5bD\/pIlS+jjjz+mXr16Ub58+WyfDVFhi8hRAYgKR5gcFQJLR5hsC4GjLSLHBcDSMaq4BSEqZDj6svL7779Tv3796KabbqLGjU8d\/LVhwwYlGoYNG0bVqlXLYX\/27Nn066+\/Uvv27R09l0VF\/kM7qcRHpyIZc+fOdXQfCuUksHnzZsrIyAAWAQJgKQCRiMBRhiNbAUtvLJs0aZLrxnXr1nkzFsK78p08efJkCOsVt0pc5f3791ORIkWoUKFCquxnn31G48aNo7Fjx1K5cjmXgb7yyiv05Zdf0qpVq2jnzp3UokULNfVhLqcfyqICh4n5bxX4JuOfobYAljIswVGGI1sBSxmWiFTIcBSzcuLECZozZw498cQT1KNHDxW9ME5x8O\/Hjx9P6enp1Lp1a9L\/37hxo5oqKVq0aK66sKjAFt3+XYROxz9DiAo5hhgIwVKWgIw1iAoZjiJWvv\/+exoyZIjKoejevTudf\/75jnImOGGzZ8+e6l7zVAlXjEUFb3rFm1\/h8k4AosI7O\/OdYCnDEhxlOEKgyXGEqJBj6cvSN998Q4888oj607RpU8qfP7+lvQMHDtCMGTNUGT3dwfkXnJMxYsQIqlgx99keLCqwRbcv96ib0YH7Z4hIhRxDtEmwlCUgYw2iQoajLyu8hJRFQcuWLenqq6+Oa4vL9u3bl+rWrUvt2rXLnv44ePCgslGgQIFc90NU+HJP9s0QFTIcMRiCoxwBOUv4fMuwhKiQ4ejLyo4dO+iee+4hnv4wXuXLl6fJkyerFQcsJC699FK69dZbKSsrSyVw8rJTTu7k3IquXbtSWlqaZT1YVODcD18uQqTCP74cFtCBywAFRxmOELpyHCEq5FiG1lLGlf+izZ\/+O7T1S5WKoQOX8xRYyrAERxmOEBVyHCEq5FiG1lLUnBwUaHTgcuTBUoYlOMpwhKiQ4xi18SYl96mQc6e1pag5OdG8YtlHBy5HHixlWIKjDEeICjmOURtvICos2kbUnCzX\/N1ZQgfujle80mApwxIcZThCVMhxjNp4A1EBUSH36TBZitqHJWGgHBgGSweQHBQBRweQHBYBS4egbIpFjSNEBUSFzCcDHBPGkQ1HreNJKKw4xsFRjjxYyrCMGkeICgyGMp8McEwYR4gKObRR68DlyLi3BJbumVndETWOEBUxBkOZ5gIrIAACIAACIBCfAE4pRQsBARAAARAAARAAARMBRCrQJEAABEAABEAABEQIQFSIYIQREAABEAABEAABiAq0ARAAARAAARAAARECEBUiGGEEBEAABEAABEAAogJtAARAAARAAARAQIQARIUIRhgBARAAARAAARCAqEAbAAEQAAEQAAEQECEAUSGCEUZAAARAAARAAAQgKtAGQAAEQAAEQAAERAhAVPyJ8cCBA\/Tcc8\/RlClTqESJEvTAAw\/QHXfcQQUKFBABHVUjCxYsoP79+9PatWupUaNGNHDgQHX4lfk6efIkff755zR8+HD66aefKDMzkx5++GFq0aIF5cuXL6p4HL+XU45Gg9u3b6du3brRQw89pNjjOkXADUveHpnb7DfffENly5alfv36oU3+2ZCccjx+\/Di9+eabNGHCBNq2bRudf\/751LNnT7r88svx2XbwodywYQMNHTqURowYQenp6Q7uCHcRiAoi4gFv4sSJtGPHDurRowf98ccf9Oijj1Lz5s1VB4PLmgB\/GPr27auY1atXjz777DN6\/fXXadSoUZSWlpbjppUrV6oOe\/DgwVSzZk368ccf6fHHH6dhw4ZRjRo18jRiNxw1KO7Imd2rr75Kb7zxBkTFn2DcsNy9ezf16dOHbr75Zrr22mtp\/fr1qj0PGjQIbdLFZ5sF2ZgxY+jpp5+m8uXL0w8\/\/KA+50899RRVrFgxT3+24738rl27aPHixfTaa6\/R\/v37adKkSRAVUWktO3fuVN+ae\/fuTbVq1VKvNXv2bPrkk09Ux124cOGovKroe7z11lvZ4oAjOvv27aMuXbpQ165dqUGDBjme9e677xILi0ceeUR9e\/n999+VqGAxcuutt4rWK9WMueGo3+2dd95RnfeWLVvorrvugqj4E4wbliyC\/\/e\/\/6lIBX\/G+csFd+5FihShQoUKpVozEq2vG46LFi2i0aNHq2\/aGRkZ9O2336qoL4uMcuXKidYrSsY4EjR\/\/nz1GeZIL0RFhLy7Zs0aJR74g6HDT0uWLFGheg7pRSEklQh3cSdSuXLlbFFw9OhRNRVy4YUX5hIKhw8fVhGgYsWKqapkZWXRgw8+SJ06daIrrrgiEdVLGZtuOPJLrV69WnXaHGLmb4ht27aFqPjT225Yjhs3jo4dO0Y8BfLhhx9SlSpVVKQCYXtSAsHpZ5s\/1zxtPGTIEOUFFmhjx46lpk2bpsxnMMiKcqSHeUNUBOkF4WdbOdVKaAg\/NqXNsUjgDvjSSy\/NISDMnZHVS3KYmaeXKlWqRI899li20EhpIB4r75Yj5\/5w583RnfPOO0\/5AKLiFHy3LLmtfvTRRzRy5Ei66KKLaNWqVWoqj+e3WRjn1cstR474jB8\/Xn0JYyHCuVPMloUFCzVc8QlAVESwhcSKVHBuwPPPP58rPyCCCDy9UqxvMxdffDHddNNNuWzyHPYzzzyjQn4cpWjWrBmmlij2t0IzRw7P8\/wrX5xEfOTIEYgKUytz0ya5LH\/L1lNyzPfJJ59UA2Fen5Jzy5GTXNu3b6+8ES9i6amjifhNEBURdDBnLHNOxRNPPEHVq1dXb8g5FV988YVK2srr86uxXP7KK6+oaQzdKXNORffu3VVehfmbnk6C5cxw\/v0ZZ5wRwZbk7ZWcctTfIGfNmpXrQR07dlRJh3n9csqSOfGKBc7z4dwezgnSooITh61EcV5i64YjCxCjqOAkYk7UbNiwIf3f\/\/1fXsLm6V0hKjxhC\/dN\/CHgiAR\/8+PleZy42atXL+KOOq\/P98fzHCcXcfidxRiH4qdNm6aEGIdBde6Evp8Tv3gpKa8AwTLdnFTdcDTeqUUGpj\/+ouKGpS7LCdr169dX0x889cHTSxzGz8uXG448\/fHss8+qnDTmtnDhQvVvPR2Slzk6eXeICieUUrAMf8vmOUAOL5cpU0bNrd5www0YAOP4kr\/ZcQbzgAEDVPYyJ2ZxrkSFChXUXSwkvvrqKxVS5mmPl19+OZc1\/l1eDzU75ciddNGiRbMZQlTkbpxuWX7\/\/feqfXLHziuWWCTn5XwKTdQNR07M5ERXTh7mLw68ZwpHL2vXrp2CI0HyqwxRkXzmeCIIgAAIgAAIgEAKEMDmVyngJFQRBEAABEAABFKBAERFKngJdQQBEAABEACBFCAAUZECTkIVQQAEQAAEQCAVCEBUpIKXUEcQAAEQAAEQSAECEBUp4CRUEQRAAARAAARSgQBERSp4CXUEARAAARAAgRQgAFGRAk5CFUFAmgAfu8ybln3wwQdqbxbeW8Dpxdva84ZRfGZGtWrVnN6W9HK80+MLL7ygjuOePHlyQutq3O20Tp06kTkcKulOwwNTngBERcq7EC8QdgJuN7fhAb9Dhw5q2203g70bDlwnFhW8EyLvguhmK\/pUEhV89DZvYle8ePGEbmSnj03\/7rvv1O68UTlx0k2bQlkQYAIQFWgHIJBgAmEVFV6PW5YWFSdOnKB8+fKpP5KXkxNzJZ\/Httz6Wvr5sAcCQROAqAjaA3h+JAjwwWq8zfv06dPV+1x77bVqy+dNmzZRu3btst\/xjTfeUOdM8LQDHxfN2xqXLl2abrvtNnXKI59Dw1GKZcuWqXv0QWF8wivb\/89\/\/qPOqOEDr7p27Zq9JboZork814fPs+Hj5nn7dN5GWV9cJ3NEhOsxc+ZM9a2bj6rng+B69uxJl19+Of38889q+oPrO3XqVDWQnnvuuerwPd7qmq8ff\/xRbc3+6aefKrHQpEkTtfU9nwCqIzH\/+te\/6JNPPqHTTjtNnRPBDHnLbL6HGd17771qaobPi+FpFp5iePXVV4kPu9q7dy9deeWV6qyemjVrWrYho6jQ0xN8xPmhQ4do4sSJKjrzwAMPKPa81bT50tvMG7dHt\/qZ8T6Iikh8nPESPghAVPiAh1tBgAn8\/vvvakDlk1fvueceBYXPOfn111\/Vz\/l8FJ5m4LMR+FwUPm+CT8YcOHAgVa1alfgE18cee4xatmxJt99+O\/3yyy\/UrVs3dTy8Huy5bHp6Ot15551UsGBBJQzYLh\/cxCdEGq8DBw6o81iKFClC999\/vxowZ8yYoe558cUXVY7Bl19+maNOxukPDuWzOOKBlwf5zMxM+vjjj1VIn3\/G1913301nn322qjfb48GfbbJ9FglcdxYSLVq0UKd\/Mo+NGzfSU089pY7GZuHE5+2wfT6Mbv\/+\/dSpUyd1qmWrVq2IRRG\/A58gzPdyHVhUscDhk3D5wDoWJFOmTFE\/Z7FivqxExbfffqvE3t\/\/\/ndli8UVHyPPf8yREogKfL5BwD0BiAr3zHAHCOQgwAPgfffdR\/\/4xz+oTZs2anDiAZOnCWrVqkVLliwh41QDi4EtW7bQ9ddfn+PIbR6MOY\/CnFPBp0C+\/vrrakAuUaKEejY\/s3PnzkrEXH311Tnqw9+WWcy89NJL2ZEM\/U2dkwg5whDvGzWf0svvw8KBhQ5ffD8nZl588cVqAGexwv+vV6+e+v3q1atVJITryPkLfMAU38tCiK\/58+crAcTChC8WFVwPbZ8H8Llz59LTTz+dfcLtokWLVESEhQxHTlhY8e\/1gXUs5licsShhW05ERalSpbKPOufyfIw8n647bty4bLbaDkQFPugg4J4ARIV7ZrgDBHIQ4G\/i77\/\/vhrIOWpwxRVXqOkPPqWRj3k3D+BcfsOGDeqYeP6bRQdPF\/DAaCUqeMDjwdTqsjrlNVaInoUNX\/yMeKKCxRBPdYwaNYqqV6+e67FWORXmn7Ew+vrrr9Vx4itWrFB\/MjIycogKYyKqsW76gRylYKHCEQsWLTzdY3XpKSInouLSSy\/NcSous+doCwuXihUr5jABUYEPOgi4JwBR4Z4Z7gABSwIc1v\/hhx\/UYMrfgNPS0tQUAx8LryMV\/DOeWuCpEM4puOCCC9Q3f44qxIpU8Dd8tsHHyhuPPudK8P\/NP7MaDFnIsADRz7ATFfGWjNqJCn4W5yo0btxYCay\/\/e1vKreEhZExUmEnKvgeFjdDhgxRooRzKZhbyZIlc\/DnqRueDvEiKjhqxBEQnmLh6RzjZcWR68D5LuZj6PV9yKlA55DXCUBU5PUWgPf3TYDn5nnAfPjhh9XcP1+cJ8FTCJxk+Mcff2SLChYAPKd\/ySWX0D\/\/+U9VVofxOWHTKlIxe\/ZslavAey7wEkm+ePpj6NChSpjwlIbx4lwDFjPm8jxdcuONN6pv6vEGP44Q8PQED+h6asU43cLf6M2iwyg0Fi9erKY7eODlPBO+WGTxgBxLVHA0hm0YB2ue9unfv78a8DlPhJMyOaJQo0aNbG4suDj58rrrrnMkKnjqhKMfOn\/CatpFGzKLCp6CGTx4MO3ZsweiwvenBgaiSgCiIqqexXsljQAPeDwA88oKznHgAYsTG999912VRMiDNEcZOAeBRQcPTAcPHlQDc\/78+dWcPudMcI4F\/+zYsWPUpUsXtb8CD5acz8CJj5yfwfZZhPDgysKFB1XOYTBeLAC4PA\/+OlGTB0iOoPCKjDPPPDOuqOBIA9tnccJRAo6u8P2ciMnvw4NqPFHBUxUssnSS58KFC2nMmDEqosL3c1TBvA\/HypUr1buxEOPkTk7q5GfzO\/LGVTx1wvkT\/O78bE4+5TrNmTNHraLReRZGDlaJmryyZNiwYUrUccIsT1mx2Lr55ptV3gYnjLIQ4ugHixp+D14Bw\/Y\/\/\/xzlUDLq05Y\/HAiLJdnH+pICSIVSfvY4UEhJQBREVLHoFqpRYATL3mg+eijj9RqB161wd+IOa+CB3leZcC\/4+WbPNhzlIEHRJ764FUPPHDzqgaObPDqB\/5WzyJE51nwckseJP\/73\/+qwax169Yqx4Dvs7piLXHVg6\/d4Ge1fJOjKFxfu+kPFgC89FOvFOGVFZyQye9cpkwZJcD4nY3TH8yMxQcnevKUBPPjZbPMS+\/cycmvHNFgMcHLavWyXStBwUysRAUvTeV8DxZxXBde5srijXNf+L04OZXrwM\/XDPg9+Hm89JQTT3VEhZ\/BUSde\/cLvwpcd19Rq1agtCLgnAFHhnhnuAAEQECbAAzhHaDjqoqcmOI+ChQjnUZiXzTp5vJWoMCdqOrHjpgxEhRtaKBtFAhAVUfQq3gkEUowALx\/laQie4uCpCd7ciqd2OLLAP+dIgtuLRYXeppv39uApqESJCmzT7dY7KB9VAhAVUfUs3gsEUogAb9XNe1twfgSvoOGkVZ76ueuuu3KtbnH6WsYDxSZMmKASPhMlKnCgmFOvoFzUCfx\/K2C8hrQRCIgAAAAASUVORK5CYII=","height":321,"width":533}}
%---
