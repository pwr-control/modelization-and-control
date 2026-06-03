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
mosfet.data = 'danfoss_SKM1700MB20R4S2I4';
run(mosfet.data);

mosfet.inv.Vth = Vth;                                  % [V]
mosfet.inv.Rds_on = Rds_on;                            % [V]
mosfet.inv.Vdon_diode = Vdon_diode;                    % [V]
mosfet.inv.Rdon_diode = Rdon_diode;                    % [Ohm]
mosfet.inv.Eon = Eon/3*2/3;                            % [J] @ Tj = 125°C
mosfet.inv.Eoff = Eoff/3*2/3;                          % [J] @ Tj = 125°C
mosfet.inv.Erec = Err;                                 % [J] @ Tj = 125°C
mosfet.inv.Voff_sw_losses = Voff_sw_losses*2/3;        % [V]
mosfet.inv.Ion_sw_losses = Ion_sw_losses/3;            % [A]
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
mosfet.afe.Eon = Eon/3*2/3;                            % [J] @ Tj = 125°C
mosfet.afe.Eoff = Eoff/3*2/3;                          % [J] @ Tj = 125°C
mosfet.afe.Erec = Err;                                 % [J] @ Tj = 125°C
mosfet.afe.Voff_sw_losses = Voff_sw_losses*2/3;        % [V]
mosfet.afe.Ion_sw_losses = Ion_sw_losses/3;            % [A]
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
figure;  %[output:49e44520]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:49e44520]
xlabel('state of charge [p.u.]'); %[output:49e44520]
ylabel('open circuit voltage [V]'); %[output:49e44520]
title('open circuit voltage(state of charge)'); %[output:49e44520]
grid on %[output:49e44520]
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
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  67.668222262819981"}}
%---
%[output:45e82fc7]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.283130953403918"}}
%---
%[output:3873a036]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.283130953403918"],["67.668222262819981"]]}}
%---
%[output:2f8df18e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:45ef9943]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:20f05bf9]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000200000000000"],["-19.739208802178720","0.996858407346410"]]}}
%---
%[output:0445503e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.311017672705390"],["54.332172227996914"]]}}
%---
%[output:04a70309]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.039461503083742"}}
%---
%[output:69e958f5]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.254711180325163"}}
%---
%[output:8e3d2c7e]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.606931756868082"}}
%---
%[output:95bd65ef]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     3.449190983162578e+02"}}
%---
%[output:7d37a44a]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -4.186041938481557e+02"}}
%---
%[output:49e44520]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATMAAAC5CAYAAABEBNhUAAAQAElEQVR4AeydCdxNdf7Hv5f+k6GEVApFoWgbTZZKypRIi2VqwkhNi0SpUBoyaUFIZctTqUkL2kRNtBI9ypqokCWypBpbxpZk\/t6\/+t35Pefec+99nucu5z7P18v3Oef81u\/vc8753O9v+54S\/9V\/ioAioAgUAQRKiP5TBBQBRaAIIKBkVgRuojZBEVAERJTMAvoUqFqKgCKQPwSUzPKHl6ZWBBSBgCKgZBbQG6NqKQKKQP4QUDLLH175Tv3+++\/L8ccfLzk5OfnOm0iGzz\/\/XE4\/\/XTp2bNnIskLlWbHjh3Svn17Oe+88+SHH34oVFkFzQyOtJd2F7SMQuUrYGb0RW+E80SLCQLmrq7oTht4rt3wIJwrmQXhLiRRB0gGsoF0eBGSWHREUemsi8p5kUaPHi1Dhw6VU089lSBfya9uvJyp+tHhPgwcOFDKlSsnkyZNkqOOOspX76BHgHvTpk2lR48ewv0Ikr5ZTWZYIzyAVnggAZeHh5cZ4eGx8fyqE2\/FzQ8B8AIQRznk4aWhDM45Ui7x0YSySYfwy+V3o6mTul588UVjsVEX5ZGefORHKM8Nt\/WjA+eUYfUlHcL1lVdeKevWrZPZs2dLp06dhPTEWaE+t3ziveWhI2mskMfmt8d\/\/\/vfEq0udEA3m5eybB6OXBNHW8GXc9tW4qmLMCtcE46MHTvWEMJpp53GpWkbutu01Ev9SH50ow6wotDBgweHrWjCbdkcuSZNNKFO6icdwjlh4EvZ3A\/uS6tWreT777+PKIK05CEvAk7eRO+99555Zoh3MaMOFwfOCSM\/OpOe543ybbnu83bTTTdFWNxuPPnd+q655hqKFu6HOQnIn6wlM8CdOHFiHhh5aLh5NpAHqHv37vZSeFBtPDfVzc+Ddvvtt5sXxGYYNWqUIQWuKeuFF17gNEIok7JtxH\/+8x+55ZZbfLti1NW3b1+b3KQjPflsIOVRrr1O1hEiqFq1qqxcudIUuWvXLtmwYYNUrlxZSpcubV5kFxcSeXElLJrs3LlTwJD22XjK4l5xzZFrzmkr+HJuhfZSl73maC0AXvZ58+aF9SSuX79+4fvDNfVSP23i2pV4urlpOY+lC\/GuQBzUS\/02nHMIddOmTTbI9xgtPziBl81Eee4zg4UK4RDvxYFnlTDirJCXMrgGS\/d5gyTJQxxCuR06dBDuEdeI+zxiWWJlcj8oi\/jMS5YuzQDA8ePHy6GHHiqTJ0+Wr7\/+Wu666y6D59tvv22O\/PGLJz83omHDhrJ48WIjnHNDEfIihBH\/5JNPcikzZ87MQ3YE8iA+88wznArp0KVNmzbGOiKviYjyB31Je+GFFwoPLg8a+QijHLK4beE6nhx55JHyyiuvCGSF7pRzyCGH5MlGmnr16gntBwd0pO7GjRsLJJAIrhR4xBFHRNQVCoUMMVI35YIluoAb1ghH8qIX8aTj2grt9d4zXigsAPJv27ZN0JM2gfu3335r2ko9trwvv\/zSvIReHEIhf93QA53Qg\/vSuXNniaUL6VyhfoRy0APhHFz50aBsrsEC4qCrFis\/zzQ4gBckbNNSji0bXMDEDwd7f21e6ifvww8\/bJ53dLNhlGvTcZw1a5bBkHD3eeQ5pz5+9Pjx436gA3mCIFlpmQEgQJ588slSvXp1gyNEwMPCTbS\/hm78OeecY8iPF2DNmjVCfh5ALBWEc1OQ88e+OPwS8XA5UeFTCADLhrophwgeGB4CiIprr1AW+thwHnjOmzdvzkHIR37KMQFJ\/lOjRg3TfnCkbqsP1+Di4hYNVz91IMoZM2bIuHHjBCuWl4WXhvS7d+82RGdxgpCuu+46oozwknBveElbtmxpulNYAybywB90I+7AqflP\/mOOOcb8aFAP2FPeokWLoo6nxdLNFOj8iaeLk9ScgiEn9nlBN3QhDFLkGEu8+SE72gGOZcqUMVld3Gi3CTzwh7pIB+6QFTjEepYPZAlb5VZf8EOIQ6w+WMl0MTkSHnTJSjJLFqi8qJCGKxBJssoPajmWSN99911jbbrkVRidIQHGa3gBIExeKl7C\/JRJevK59wRSty+YWxbhvMD2ReSlY1wIi9NNx3lBdPPThfKCJAyZgDkYuXgURkd+4LAQ3fsAaUKe9gecriY\/9IWpJ5l5s5LMABAg6VKsXr3a4GG7anShKlasaMJ4KRAu6Krwy86vWrVq1cxAMlZctAef9ImK1+R2XxrGXRIphxefdDyMHHNycoxlwkPKNYL1x0NEe2k3YQUVrFkIbMGCBcZasr\/QieLqVy9YI3TV+FHgxbKWmc3DNeHgRLfFhvOScG9svA23R4uRvea+QVwQmO0OUa9ffvRCSOOnmy07ni42nT1a3egW0i7Ets1a2zZttKPNb+8\/zw3kxA+D282MlpfxLbqu\/DB3PtA9zs\/zYfUFF8SWjz68K3Q3bZh7xErGguddw+J14zJ5npVkBoDt2rUz\/Xpvl8T78PCw82BAdgBNPPm5ETz4\/KoTj\/By8JKQLlHhwYcMuPnoYs18ykUSKQdLiV9CulXowZFrZo0swVhdqYO64pXLw0nbebG8aa3OpOGhpH7SgEuiuJLeCuVQV9myZU1XHv1pB2E2DT8wlM014RYnrq1wbzgnnvxWeLnBAUyIR9CVe0jdlEVa6iUNaUmDEE95sXQjnRXKYOA9li42rT1ynxHqQheEcyw7zm06vyNpSMszSjvQl7R0VW03k+toQlv5Ybd5E3k+ID7qQ0fqtvXZ8nkewBEs0McKuJAGw4A6+WHgOiiSlWQGePwKcVM4t8IvNL+69pob9sADD9hLM0lg4+miuPm5eSNHjhReknCGBE+8ulDvY489JpBGIkUwRsIyBTct14SjDzNRNq5r167Ci2OvvUdrKXrDvdf2gcVCw1Kz8d62EO7FlTDEW1edOnWENUjEgSfdEnTFksRiYIaMa+LByL03hHFvqItzK7wwhNuX1loTxDNjZ8vjGrG45Vc3ykdnykCo008X4l3hPnO\/aZMN55xJCO6fDfM7koZnz63fttsvjw0nr\/2RIAydea758cMCJswr5In1TPHcMebp6kOZPBvWEiQ\/5XjLzuR1ysgM66Fjx47Cr6pE+Tdp0iTTlbKsXxCrCEJy+\/Q8gN6qeLlsGm6GG+\/mZ8CVm0g85ZDHpieceF5OHlzSeMUti8FYe6O9ZZGOsijTLcOmo16EaxvPOWEISxXQw9ZBOZRHuaRHP+JJy5Frwr1i80VLQ1nkt0L95Kcs0tu67TXpCOfa5kUniIZwziFMLACsAV44yli4cCHFCt0ac3LgD3VRnhV7D8CTl9Z2tw8kNT8WlG\/TciS\/N440sXQDCwQ9KcPWSVlcW+ncuTNFRxX0o002LeeEkZi60cENI9wVt37KsDpEy2sxRj\/KIC15EMLceK4JJw1pEd5J7gX3B8K76KKLhB8cZij5ESCNVx\/KdMMpl+sgSUrI7L\/\/\/a8wxe\/X5waA9evXC+uMABqJdaNJr5LdCPBS0h2nFbxI\/IjRNeLXH6uI8HiCNUm3mHzx0mq8PwJ0LbEc+WHh3HZNGbPkPvnnDHZMSsgMlp8zZ46ce+65vq3fuHGjMI7im6AQEdyQeL+EhShesxYQAawDfrhcwRrCCkikSNKRnnISSa9poiOAxYjx4N4Hzq31FT1X8EMjyIxtKm3btjWLE\/kljSfeJjLgTP+fX18\/smKGhjVF9Ltr1qxpxoDefPNNwaLzlqfXioAioAjEQ4D4CDKDUH766Sfp0qWL9OrVK6aQhkJcoXuJCVu\/fn03OM\/53r17hVmaPn36yLJly2TQoEEyZMgQ+fTTT\/Ok44LuiMrxecYXFQ\/FI8jPAJM9vLvplggyC4VCZovIBRdcIJdccklMIY2r8PLly+Wjjz4yG5xDoZAblee8fPnygvVGN7RkyZLGCjzzzDPNFps8CX+7wATORkH9bNTb6pzN+mez7uCfzfp7eYG2pEMiyOzggw+WJk2ayEEHHRS3fvbnuYnWrl0rubm50qBBA2NJMFBLd9Nd\/En6pUuXmn2MWIBcW2Ew2J7rURFQBBSB\/CAQQWYQzFNPPSV0E+3Sip9\/\/jmhMpmu5VfFCmtTmIb3DiwyQM9WCWZTfvnlF7OlhvUrLIJMqKIsSfTBBx8USNOgZMpm\/bNZd+5\/tutPG9ItEWSGtcVg\/BtvvGG6m7jQ+eMf\/yj333+\/0BdmTK0gSrKynrVkrHFhTI3BfxzWMQHA2Fnv3r2lVq1aBSla8ygCioAiEP3rTIxj1a5dW\/r37y\/s33v66aeFWc6LL77YeHR4\/fXXJRFrDYsMaw2c7XSwvWbBHh4FsOLomtK1JZ2KIqAIKAIFQSDCMvMW8n\/\/939C92\/48OEydepUYU8WWze2bt3qTarXikDRRkBbF2gE4pIZFhnjXsw8YpmxLQUyO\/zwwwPdMFVOEVAEihcCUckMqwuf4ezZYgsJFhljWvPnzxe6jnXr1hW6osULKm2tIqAIBBmBCDJjoL5Vq1YyYsQI4Th9+nRhjKxFixZmY2+QG6O6KQKKQHFE4Nc2R5AZa71YmsEmcVb4s5P+16T6VxFQBBSB4CIQQWa47sFP1ObNm+NqjRUXN5EmUAQUAUUgDQhEkBl14mbloYceEsbJYglpSK+iCCgCikCmEYggM7qZd955p7C\/isH\/WEKaTDeg6NWvLVIEFIGCIBBBZr\/\/\/e\/N3sx4m8xtfEEq1TyKgCKgCCQbgQgyS3YFWp4ioAgoAulAQMksHShrHUUBAW1DwBFQMgv4DVL1FAFFIDEElMwSw0lTKQKKQMARiElmrDnDBdBJJ50kuO+ZNm2a3H333caDRsDbpeopAopAMUHANtOXzPBb9uijjxpXP88\/\/7wcffTRUqNGDeG7eo888ojs27fPlqFHRUARUAQyjoAvmW3atEmWLFkirDk77rjjpESJElKqVCm59tprjZNGNqNnXHtVQBFQBBSB3xDwJbPf4iMOfBsAH2cRERqgCCgCikAGEfAlM755WadOHXnwwQcFb7D46v\/+++8lJyfHuNPmC0sZ1LvIVq0NUwSyGYG1W\/bIvoonSu7KbcJ5OtviS2ahUEgY\/MdrxtVXX20+A4dLILqXdD2x0NKpqNalCCgCwUdg1qptsqPRXXL54wuDQ2bAdsghh8i9995rxs7mzJljjvg5U6sMdFQUAUUgSAj4Wmbbt283XxrHawaExszmfffdF\/akgSfaDRs2BKktqosikDoEtOSEEEh319JVypfMSMSM5ltvvSUQm73mM3SEz549W1q3bk2wiiKgCCgCGUfAl8yYsdy7d6\/wMZORI0fKgAEDzPkTTzwheNYYPHiwca1d2BawMLdjx47C9zRF\/ykCikCRQeDYCqXS2hZfMoNk+DJT9erV8yjEwlnCiffG5UmYwAULc8ePHy+46E4guSZRBBSBgCOwdsvuNGv4v+p8yQwnjUwA0J2EdMjCkYkAzrHOVq5cyWmB5csvvxTK4zN2BS5EMyoCikBgsNJ9KQAAEABJREFUEFi3ZY\/RBasMMRdp+uNLZpBVt27dZNiwYdKgQQNp3LixnHXWWXLPPfcISzXWrFljdgcUVM8dO3YI3ddOnTpJxYoVYxZz\/PHHm10Hq1evzqrj+vXrs0pfL77ZrH826859yEb9eU9nz5kT811OZaQvmVHpKaecIm+\/\/bbwtaZevXrJkCFDJDc3V\/gY8EknnSTvvvsuyQokdC+rVq0q9evXj5ufRbt0abNNqlSpItmms6tvNuufzbpzD7JR\/w\/nLxEWzPJCH1s+veNl1BmTzOhWMjbGJvN69erJiSeeKOvWrZP27dsLHz0pU6YMZeRbli9fLh999JFglYVCoXznL7oZtGWKQPYiwIJZq\/05Ncrb07QdY5LZmDFjBBJr2LChWGnTpo1Uq1ZNDjnkkAIruXbtWmPh0X3FNJ04caIhtp49exa4TM2oCCgCmUOA9WXj5240CpTYtUna1atkztP5x5fMfvzxR5k+fbqMGzdOXnrpJTNOtmLFCrnlllvk5JNPNsszCqrohRdeaPZ70n1EIEiWgDz88MMFLVLzKQKKQAYRwCrLXbXNaFBi12ZJ9+A\/FfuS2U8\/\/US8GfMpV66c8FHg3bt3y+WXXy4zZswQyM4k0D+KQNFHQFsYA4Hx876TruOXmhSQWKllk815uv\/4khmzmWwmZ\/kFZMbYGYKCEJklO64LK1hkWGuFLUfzKwKKQPoQoGs56J01YSKj5nb1jpaDNn3FadrFl8xYZ9auXTuz\/OLbb7+VE044QVq2bGmkUqVKUqFChbQrqxUqAopAMBCAyG45YI0Nemd1WKFezapLr2bVwtfpPvElMxRhCQZdSsbIcPszdOhQGT58uDz00EOC1UYaFUVAESgeCEBgdCkvH7VQ\/vDgJ2LHyOha9soQkbnI+5IZm8sZlN+1a5chLrqdrNQ\/\/fTT5eWXXxYWvboF6bkioAgUTQQgMbqT+ChjbMySGK1tdEI5eaNL3YxaZOiBRJAZg\/zMYk6ZMkVeffVV4YjnDCuvv\/66CYfkKEBFEVAEihYCkBcWGMRVoft0Y4XRnSTcttSS2Btd62Zk5tLq4R4jyAyLC88YbDX65ptvzJajQYMGGd9mHPFj1qFDBzniiCPccvRcEVAEshQBSMqSl+1CQmSEuU2y3UksMUisUY1ybnTGzyPIDJKaMGGCsch69OhhjjNnzhQrbGG66qqrJBTSlfvJvntaniKQSgQgLQSSgqwgLmt5cU2424VEFwisXb1Kpiv52T1nme5k0EgMPZES\/HGFD5fg4mfPnj3SqlUr4fjDDz+IK8STzs2n54qAIhAMBCAsBHJCICqIi0F7hGvCvcSF9pa8RrWrHSYwzoNKYOhsJYLMWBx7xRVXhLcv2W1M7pF40tlC9KgIKALpRwDC4itIEBMEBWEhEBZCGEK8H3Ex9oXlBWFheSGcE5YNBOaiHkFmRx55pFnhzzYjP2G5BuncgvRcESiyCGSoYZAVAhkhEBNkhdjuoZ1hJB7CQqKpi8XlEhfjXhAXY1+WvEgTLW+2hEWQmas4XjMWLlxovtDEnkzWl0Fwbho9VwQUgYIhAFEhEBECWSGQ1R9HrDGziNEsrFiEBSFhVSGQFKS15ZEm4iWubLO6EkE4Jpnhy6xr166yb98+qVu3rtkczjja1KlTEylb0ygCxRYBSAqBpBDWaVmigqCsZcU54QjpED+yAkzICvFaWZAWhIVAYgiEVhRJCxyiiS+ZsQ8TB4p8Yq5\/\/\/5y\/fXXmw+a4KiRcOKjFahhikBRRgCCQuxYFeQDESFYVJBTNKJinRZpISryx8IIsvpj5VLGjQ4r6yEml6wgLLd7CGEhscosmnF5W+VLZiyexSKr7vmgCdeEE5+3KL1SBLIXAQgGSZSk7FgVJAZJIYkSFWRlLSsvWdku4ZNtKgkk1qtZNUNqkBX5shfh1GvuS2alS5cWPje3dOnSPFrgn5x9mWxvyhOhF4pAwBCAnBCXoGx3DxKKZkkVlKRoOmSDRCMqLCtLVK5lpWQFcskRXzLDk+x1110nLJzFTXbv3r2NN9gbb7xR8KaBV43kqKClKAKJIwA5IfEIyh1AdwnKdvcStaTQDIJCLEkxFoXVhEBSkFM8osKyoiyV1CHgS2ZUed5555kdAE2aNOFSateuLZMnTzYfNDEB+ieJCBTfoiAnJBpBYT0h3rGowhIUaENQiCWpXs2qm65dPJIiHkJDICnKoDyVzCLgS2Zbt26VnJwc2blzp9xwww0yYMAAueOOO4zn2cyqrLUHHQGICYlGTt7unTtYHo2gGIdCKC+RdkMsiB1Ah3AgHwQrColnSZFHSSoRtIOVxpfMSpYsaZZi4NMML7DMYkJwwVJftUk1ApAIAjEhdM8QO\/aE1YRYyykeOZG3IOSE9YRANAjkhEBOiJeg7AA6aUiPQFAIZJdq3LT89CPgS2Zly5aVwYMHy+eff26cMXKk23nllVfKtGnTRPdmpv9mJaNGiAmJRkzWarLk5B13SoblRBsgE8Qlp0S6eHY5QpoJCpVVsgABXzKzujOjyefmhg0bJq+88oqZ4bzvvvvMB05sGj1mBgFICUmEmLCYECwoJBoxWaspP5YTLYeYkETIKdpAOeTU67clCO3qVRK1nkBVJb8IxCQzrC+WZvTp00fq168v+DE75ZRTBJ9muArKb2Wa3h8BSAkJEjG54069fhscp0uH2G6dkpP\/PdWY9CLgS2a4+WnevLm0bt1a+BITLrRnz54tLNGoUqWK+jOLcZ8gJWTBhj2CtWOFbhxCNw7BQsJaQjhHUmExxbOaXHKCpBC6dO64U6\/fLCesJoQyY0CgUYpAyhHwVuBLZiyKZQaTsTI+BcfeTCYFvAV4r7\/44guBBI8\/\/nhp1KiR4ILbm4brSZMmCWmsMB6HzzTigioQFGKtJ79BcEgJ6TTx1+8JQmCIJTW6cQhlJdpWyMNKrO5cNGKy5ER3DrHEZLt0Sk6J3gVNF2QEfMmMRbOMlTFmlmgD2K8JAXbp0kWWL18uvXr1Mu62N27cGFHE+vXrZdSoUWbGFE8cQXIrBMlY4oGEsKKiWU92ASbEhJAvoqFRAiwpcVRiigKQBikCBUDAl8wKUJasW7dO9u\/fbywytjw1aNDATBhs2rQpojgIrmLFihHhmQiAhCAviMuSFucI4RBVPL0gJsQlp34XVjSLMLGWECwkd4yJa7pzWEuIWkzxUNZ4RcAfgaSSWZ06dYTvB\/CBYHyhzZkzx3ymjo8GuyqwEJcPC\/ft21dq1qxpvNq++eabQh43nT2nK8qe0GTKrEUr5O8TFkrToZ8Yv1GWuGyd7vGYsgcJg+GX1T5EEEiK8SRkwa3VBHn9r5UEGdainPRsWMpI3UO3ScOKu6Vyya1Gfvlxo\/i1IYjhWM9B1CsRnbJZd9qXjfrzniLuu5POc18ywysGA\/4cXYW4XrBggfz8889ucJ5zxtn+8Ic\/mH2djIWVK1cuT\/zevXulTJkywizpsmXLTFd0yJAh8umnn+ZJZy\/ohuKtIxlS8rCj5bYp2+SysevlibnbhEF6Ww9HrCvGkrCUsKa2PNJEvuh3rrzX4ywZe2M9I91anCpXNKptJJZOTJTEig96XDbrn82681xko\/68pwjvUSYkgsxYjsFMJr8OQ4cONZYEA\/NWvvrqK7O1KdZugFNPPVUWLVokOHe04jaufPny5hN2fFSYSYXGjRvLmWeeKfPmzXOTJfWcriTWFwPzbrfRJS+Ii64fRAahNaqRl4STqpAWpggoAklFIILM+FAJHyy59NJLBQuMY8OGDU1XkCM7ACArCMmrSW5urvTs2dMs5SAOk5N1aRs2bOAyLKxdY6kHSz7CgQdOUuWJg1lHSIzxrwPVmP+QGKTlkpeJ0D+KgIuAnmcNAhFkxodKmFlkvAsi44jpaGXFihXSr18\/M7DvbSULabHIFi9ebLY7sUxjyZIlwqyom5aZUrxv0I3FEpw5c6bZNuVN5+YpyDnWGETGrKPND4nRfYTEsL5suB4VAUUguxGIIDPIZcuWLcI4FwP0DMrbLqY90g0lnbfptWrVkm7dusmtt95qBvbxtnHzzTfLGWecYb67yfjZ+++\/L1WrVhXKHjhwoEnH2BmLccnvLbOg1xDZLeOXiiUySKxXs+rmww7afSwoqppPEQguAhFkRjeTLzExNkZ3k66lVwgnnbdZoVBILrvsMsHiwpLjyHUoFBJr8eGBg3yUyXga6eieWp9pxBVWLJHZsTGIDGusV7NqhS1a8ysCikAgEIhUIoLMIJ1x48YJY110NyEbrxBOusjighHC2JiXyCC0YGinWigCikAqEIggs1RUks4yITK3a4lFpkSWzjugdSkCmUHAl8wYH2OMixlJrxBOfGZU9q+V7iXLL0gBgY1sW1s4cq2iCCgCRRsBXzKjG0l30nYxV61aJR9\/\/LHcdNNNxhUQM5dBg4YBf6tTu3pHS\/YM9Fut9agIKAIFRcCXzLwFhkIhYVtS586dhcmBWItmvXnTcZ27cpu442S9dLA\/HbBrHYpAYBBImMysxmxF2r59u1lHZsMyfaR7Ofid1UYNupV0L82F\/lEEFIFig4AvmbGWrG3btsJWI1cuuugis8wi2g6ATKE2a9X\/rLJzTiin3ctM3YiiV6+2KIsQ8CUzVukzPoZPMleee+454RsAuPgJSjvHz91oVMEqY6zMXOgfRUARKFYI+JIZnmbPP\/98Of30043\/\/0suuUTYk1mlShVhc3hQUHLHytQqC8pdUT0UgfQj4EtmqDJmzBhp2rSp4KaH6w8\/\/FBYljF16lQuAyF2rAxl1CoDBRVFoOgjEK2FvmTG\/kz2UbIbAFc9ZO7YsaPk5OTI+PHjBRfZhGVSGPi3M5h4eNWlGJm8G1q3IpBZBHzJbN++faY7Wbly5Twa1qhRQ4jDSWOeiAxcsNrfVntXs+r2VI+KgCJQDBHwJbPDDjtMmAR47bXXwssw8KDB5nE+clK6dOmMwzVr5VajAwP\/apUZKPSPIlBsEfAls4MPPti483n11VcFF9gsz8CVT\/\/+\/aV79+6G6DKJmtvFZOA\/k7oUtm7NrwgoAoVHwJfMKBrPGbjpeeGFF8xn40aPHi04UmSGk\/hMitvF1IH\/TN4JrVsRCAYCvmTG15NYNMviWciLpRn4IMNiC4Lq2sUMwl1QHRSB4CDgS2ZsNK9du7bMnz\/f9xNwmWqG28U8tnypTKmh9RZ1BLR9WYWAL5mxkRwf\/j169JAGDRrk2dZkLbZMtZTtS7ZuncW0SOhRESjeCPiSGTOZXbp0kccee0zuvfdeM2ZmtzWxzYn4TEFnty9Rv85igoKKIqAI+JIZ25nwy89YmVcIJz5T8NmFsizJyJQOWq8ioAhkCoHo9UaQGQP+fGFp6dKlQneSJRleIZx00YtMbSh7MW0NOotpkdCjIqAIRJAZ3cf27dsbR4x0J23X0j0STrrCwMd4XPPmzQWX3I0aNZLp06cnVJw7XqbryxKCTPvp4WkAABAASURBVBMpAsUCgQgyo\/vIEgz8lUXzmnH22WdLYbuZ7OscMGCAMCa3fPlyMx43aNAg2bjxV1c+sZC3SzJIo+NloKCiCCgCIBBBZgRaSZXXjHXr1sn+\/fsFiwy\/aMyWskVq06ZNtmrf49qte0xc0RkvM83RP4qAIlBIBHzJLJVeM+rUqSMTJkyQChUqCPs958yZI5Aa3xiI1R7WlyGk0S4mKKgoAoqARcCXzPCMgRPGVHrN+Pzzz82+T9ay4SetXLlyVq88R8bVVq9eLXOXrA6HN6kqQliQZf369YHXMRZ+2ax\/NuvOPclG\/XlPkfBLmuYTXzJLh9cMPNcuWrRI2P9pJVr7+dxd9erVZdXO34ejK1U6WggLsuCVN8j6xdMtm\/VPgu4Zfb6yUX\/eUyT8kqb5xJfM2IPJEo1UeM3Izc2Vnj17yk8\/\/WSaC5uzqX3Dhg3m2u+PHfxnvEwH\/\/1Q0nBFoHgi4EtmwAHBYDEl22vGEUccIVhkixcvNr7SWKaxZMkSqVevHtX6SnixrO7H9MVIIxSB4opATDIDFGYZk+01o1atWsZX2q233io1a9aUG264QW6++WbBXxp1RhM78E\/cOTXKc1BRBBSBYoiAX5PjkplfxsKEh0IhueyyywSvtfSxOXIdCoV8i3XJjG6mb0KNUAQUgWKJQEbIrCBIuyv\/q2o3syAQah5FoEgjkD1kpv7+i\/SDqI1TBAqLQEwy2759u7z00kvSu3fvPMLWI+IKW3l+8tuV\/\/nJkw1pVUdFQBFIDgK+ZMayibvuukuefPJJs\/UoOdUVrBTGyxByq2dZUFBRBBQBLwK+ZPbjjz8K3mb\/+c9\/ykMPPSRsDLeCB42yZct6y0rZtSUyKtCZTFBQUQQUAS8CvmSG94zDDz\/cmz7j17onM+O3oHgooK3MOgR8yax06dLG1c+QIUNkxYoV8sMPP4QFx4y\/\/PJL2hrrzmSmrVKtSBFQBLIKAV8y27x5s4wcOVLeeustadasmeDjzMoVV1whxKerpXYbE\/XpNiZQUFEEFAEvAr5kxqfmZsyYISxq9QrhxHsLS\/W1LpZNNcJaviIQdAT89fMlM7Lgaww3PUwAsByD7ibfBiCc+HSJ7slMF9JajyKQvQjEJDM2mV9\/\/fWyatUqwRrDYSOeNMaNG5e2Fu8vXTFcl85khqHQE0VAEfAg4Etm+OkfP368jBgxQvr37y84TsT\/FVYa42gs3fCUlZLL\/aWDN6OakoZqoYqAIlAoBHzJbPfu3YK3WQjMraFq1armkkW15iTFf\/JYZidE90SbYhVSWLwWrQgoAslCwJfMDj30UOFzcni0cJdhrFy5Uli2gSRLiVjluGQWK53GKQKKQPFGwJfMWDTbtWtXs\/K\/ZcuWwkTAVVddZfyQXX311Ybo0gHdvoonhqvRZRlhKPREEVAEPAj4khnpcMr47rvvyt133y2tW7c2ThSnTJkifHyE+HSKLstIJ9rFvi4FIAsRiElmtIc9mJdccongFbZt27aS7vVldgJAN5hzN1QUAUXAD4GYZPbtt98KBMZ3Lhs0aCAnnXSSITW2M\/kVmMxwNpjrmFkyEdWyFIGii4AvmTFbef\/998sxxxxj3Fuz1uyjjz4Svgnw4IMPmpnOdMKia8zSibbWpQgEE4FYWvmSGevItm3bZj4Jx9eUQqGQVKpUSW6\/\/Xb57rvvhAW0sQpORhyWmS1Hx8wsEnpUBBSBaAj4khmzmSyUxULzZsQ1EPHe8GRfr9u6J1yk+v0PQ6EnioAiEAUBXzJjHVm7du2kb9++wgwm42dsacL7bOPGjYVFtbgFilJm0oJcyyxphWpBioAiUCQR8CUzupGjR4+WtWvXCluY2h6YyYTYILXHH39ccAOEeFFZv369mTQ44YQT5LTTThM+IBxtY\/qkSZOEL5lbYbmHlxzXbtkdLr4orjELN05PFAFFoNAI+JIZ42QTJkyQmTNnxhRXA6w1Jg0gpuXLlwv5x44dK3PmzHGTmXNIb9SoUWEXQzNmzIhY9rFuy6\/dTB0vM5DpH0VAEYiBgC+Zkefnn3+W5557Ti688EJhaQYf6mWTubu9iXRW2JzOxAHr0kqWLCks6WDh7WeffWaThI8bN26UihX\/5xEjHOGc6BeZHDD0VBFQBGIiEJPMnn32WZk4caJ0795dxowZY7qWDz\/8sDzzzDNRCy1fvrzQNa1cubKJ54MoLOlgFtQE\/PZn586dQneVbmvNmjWNF9s333xTvN3RtdYy04\/+\/oacHtKCgFaSlQj4ktmWLVtk2rRpglPGFi1ayKmnnirXXHON5OTkmHDivS1mDVqFChUEq2zDhg1y2223CRvWL7jggjxJ9+7dK2XKlJE+ffrIsmXLTB18a+DTTz8Np7NERsDc6W\/J6tWrs07oSmej3lbnbNY\/m3UH\/2zU345\/885mQkr4VYr7n1AoJFhbbhquQ6GQ76JZuqZPPfWUXH755XLyySfLE088YQjNWwbfFzj33HMN8TE7euaZZ8q8efPcZOHzO27sILgiyjaB\/LNNZ1ffbNY\/m3XnHmSj\/ta9fvjFTfOJL5kddthhhsimTp0qdoyMbiAugXANhMXl1ZV4vrPJUo7JkycL39eMth4N19t8XNi7hi1amdQxYmDfPDOf9hcg6Ed0D7qOsfTLZv2zWXfuSTbrj+6pkdil+pLZwQcfbFb7420WqwnriUkAvM7iGigaSWEav\/fee\/LII49IlSpVfGuGDCE7iBGiZMYUF0P16tUL52EGc8sjTeSze86S9R8+H571tOyvx68Vk68Vg6C+B+EXOY0nvmSGDgzOMzDPRABW1vDhw80yDWYoifcK25y+\/PJLYYyMXxcrjLOxhowlG++\/\/77grZbB\/4EDBwp1MHbWu3dvqVWrlrdIgdQiAjVAEVAEFAEPAjHJjLQM6kNeLLfgu5lYbIRHEyyrJUuWRFgMnTt3NmvIWEvGMg\/yUhYfTOGXJTc313xwmHAVRUARUAQKgkBcMitIoZonEQQ0jSKgCCQTASWzZKKpZSkCikDGEFAyyxj0WrEioAgkEwEls2SiqWUVBQS0DVmKQCDJjPVqeNvA6wauutm8vmPHjkBCnKiuLEFhnyttwqMIXkjY0pXJRiWqu6sj6w5Z0BmE+5Ef\/T\/++GNp1KiRWa\/YvHlz+eKLL9xmpf08Ud1x3sAuHN4DZv47dOhgtgKmXeF8VMj+7I4dO5ovuuUjW6GTBpLM2Nb00ksvCWvR5s6da9wQsUe00K1NQQGJ6srL9PLLL5s28SIx8wtJexcOp0BF3yIT1d0WwBY11hCyy8OGZfKYqP7r1q0zW+YeffRRwZvLpZdeKuxSYZdLpvRPVHeWMi1cuFA++eQTWbx4sfH2PGLEiEypHbNefrBZO\/qPf\/xDWKIVM3EKIgNJZtOnTzefs2NbR9myZY1\/tA8++EDYoJ4CDApVZKK6cnPPP\/98sy2LBccXXXSRQA78ihVKgUJkTlR3quDFZ9fG2WefHbE9jfhMSKL6syi7bt26wuLvgw46SG666Sbp16+f2UqXCb2pM1HdSYuAP0fkqKOO4hA44f1k+RV7s3nGk61gvPICR2bctDVr1kiNGjXCunPz2NgOWOHAAJzkR9f27dsL6+2s2itWrBBerFjr9mzaVBzzozv1f\/jhh0LXkj23XGda8qM\/lvCePXvk4osvFrr4V199tflhDIVCGWlGfnRnXSa+Bdl9w15n1nFeddVVGdE7XqUYHiyuv\/vuu81WyHjpkx0fODLjoYO42PKU7MYmu7z86MqNpk2Y4nShBwwYIJ06dRL2wCZbr0TKy4\/ujO2xCwSL5ne\/+10ixac8TX70p1uMEwO6mXhpOeecc4QuPuNRKVc0SgX50Z3hFXbP8GU0upmMueL5GUKMUnSxDgocmbHjgO8P8AC6dwYrJhTKzC+pq4d7nl9dce2Cq3E24z\/99NPGUnDLS+d5orozUI2+WDXRtpulU2e3rkT1t3lat24ttWvXNp9KZLvd999\/LzgTtfHpPCaqO+OpdEexhvERyI8h3+X45ptvBF+B6dQ5G+oKHJnR7WLvJmNMFkAePExtrBsbFoRjfnRlfIyvwvNg4q2XLWKZbEOiutO1516wl5a9ti1bthScCWDd4BwgU21IVH\/0wzko1hDnVkqUKCGhUGZ+HBPVnbGnUqVKWZXDR8iQuHCAnhgESpi\/AfvDbBMDiVgy\/AKxpAGX3TwEAVNVEtX1lVdeMftPr7322owOPLv4JaI71sC4cePC+22ZYW7atKnMmjXLOOx0y0v3eSL6o1OTJk2MQ1HGm+havvrqq8KEQDy37eRNlSSiO72Rs846S9544w0zWYTufFcDR6n4FUyVbtlabiDJ7IwzzhAGObECuJmnnHKKsDYoiCDH0rVnz57mI8qMb6xcuVL4gAsD0Fg4CF5EGA\/JVLsS0T1TuiVSb6L6k+7mm2+W6667znwxbPPmzYKVHAplxjKjbejk94zb54Z0f\/nLX8w3OOjmM15G1\/OOO+7ImFWJTkGVQJJZKBQSFgcy4MmALd\/qxLQOIoihkL+ufC8B4RcWz7p4CHEF6\/PII4\/MWLNCofi6e5XDKsB7MBabNy7d16FQYvqHQiHBsmcNFLPIw4YN859tS1MjQqHEdOe5v\/HGG80aM3RnnZ+fE9M0qR63Gp5pXIfxrMRNnMQEgSSzJLZPi1IEFIFigoCSWTG50dpMRaCoI6BkVtTvsLZPESgSCMRvhJJZfIw0hSKgCGQBAkpmWXCTVEVFQBGIj4CSWXyMNIUioAhkAQJKZhm5SVqpIqAIJBsBJbNkI5qi8vhcH4spEyk+P2kTKc+mYQU9Dg7xAIIHDRse74je6BQvXTrj8RPGwmW2laVqWxblUj7CeTrbVxzrUjIrjne9gG3Ge8axxx4r+DVL96JZNrzjcaSAqkfN1qZNG1m0aFHKtmWxaJRtXw0bNoxavwYmFwEls+TiWajS8OLQvXt3sS6SOScMK2Lw4MGCOxhrFbHCGisJ6wKvtXhOxdNItLRLly4VvEawlYoXi7yQQzRlWSXP1jHKddNSLi6LiOfotcwoj3LJQz0tWrTI45oaIsRVOHGUj48x6kdndKcN1EmbKIfysGb4XittJxzLEDzABRfS1PXYY48ZZ4tWH1zl4BWDsqgHfaknlpAXXHNycsI4XXnllcLe4Gj5vJYm2JCfcqKl17D0IKBklh6cE6qFDcW8ELhJZisXnh1wFYSDPrZ0YUmw6ZsXesyYMYJ\/LrZHcY6PNLa7eNPiybZHjx7yt7\/9zbiMfuaZZ2T48OGSm5sboRMupfGO8fe\/\/11WrVolblrKxSKDQDh6LTPKo1zyUA7bh\/C7RXuoCAvl3nvvNTr86U9\/EkiIfYZsV5syZYrgUpw6+\/TpI3RJN23aRDZZv3694LUU55Ds0aXN+FQDI75HsGDBgrArH8ivX79+Qr1gQTtoD\/qYwuL8ee2114TtQujUuHFjs68WrONk0+iAIKBkFpAq+ThfAAAEhElEQVQbYdXghabrw35O9nXefvvtNip8xNsDPsbYrAwhYMVAfNG6YZAAe+XwdIHbmDp16pgPe8yZMydcnj3BqjnxxBMF9z6hUEhI26xZM4FsbBq\/I2lISx7qwQrDmSPn5GGjNP7EuEbvXbt2CVYZPtIg7GrVqhnvr+xF3L9\/v9Am8h1zzDGCwwHCITh85+PTCzLFcwQfV6HtpMVyPffcc417bOqBeI877jjBoiM+nvz5z382bs2pizrRcc2aNfGyaXxAEFAyC8iNQA0sr65duxovqFghdHVsd4x4KxAYG9dxY4PFhDXBi2fj3SOWBT7wcblM1wsZO3asfPfdd24yc053lHohUhNw4A+kAekcOPX9j\/X1zTffCGltIogGYsGqIsyN49rK9u3bBYuNQXII5NlnnzUkZ+PRxfr0Qmd0wbedjcelepkyZcwlbcVdFF1Z2gl5Tps2LWpbTQbPH9dVO3VStyeJXmYIgUSqVTJLBKU0paF7g8sjrBxIjDEfxotwkOiqgE8xulkzZswQrCksOMjDTWPPsTIYv6KrRZcU+eyzz8wHPWwae8Q6wjK0VhFHritVqmSTRD3y4leoUEFwdWQT0D3EOwVkZcOiHR9\/\/HETTHeRj9bgQ94SoIlw\/hx++OGCFbZt27ZwKPVAYgTQ1i5duoR9r6H7\/PnzTReb+HiCE1CbBmeOWLpYeDbM7+jm80uj4alHQMks9RgnXAPkBDFh6ZAJawsX4lgIvKhu94t4XjbSvvjii8b6IJ5wNy3W0VdffWWcKUJOkBmD1ZAhaV2BSOl+QgCkpUvHdZMmTdxkEefoRxfzX\/\/6lxk0x4kgY2fUhf4RGTwB1AUR4ogTqxEC5NqTzLjtOfroo+X5558X6sB7L+ltOkj7nXfeMd1KyqQdf\/3rXwWr0aaJdaSbSpmUPXr0aKlSpYrpdnrz4IIHK5E6IFImLLxp9Dr9CCiZpR9z3xqvv\/56M1ZE95EZTQbN77nnHsHDLkTDNS8ns3XVq1cXwuhmcs6n67p16yY4eyTcpmXM6YEHHpCBAweaLxNBZIxlMTvoVYSxLL55iONCumq33Xab3HnnnUK4N633GiuSMSe6ijgRxGLCyoLovGnda3x1MRZG95axMMrBQmTGEFJx02Kx9TswwM8sI+lxbsgYn01Dm8AAJ4zoz+A\/XVjG5WyaWEfycw8omzqoizrBFEeazFqS\/4YbbhCsW76Y5NXBm5b0KulBQMksPTjnqcXvgl98xr+YiaOLhItkrAPSM7COlcRsJt0+m46lB61atZL+\/fsLlh2D\/W5axqoYCH\/77bdN94v0zDSGQpFeVkOhkPFqShqsKmYo3bQQJ\/VTJjq5gjUIMTELi\/7MeEKkpMHadD+z55ZDGtpJe9GROKwt6qlfv75g9dAmysESoktLPOnRj7Zh\/VF\/KBQKO2FEf8ojnryJCOvCyEPZ6IRu5KN+sEU3rvm4yKRJk2Tu3LlmcuS+++4T9AUXb1rSq6QHASWz9OCstSQBgb179wpLVFiXRhebbikkwjIKrNckVKFFZDEC\/w8AAP\/\/A6ukvQAAAAZJREFUAwBhGaVScBobgwAAAABJRU5ErkJggg==","height":185,"width":307}}
%---
