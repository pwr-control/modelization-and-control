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
%[output:49e44520]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAmgAAAF0CAYAAACXE8U0AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQm0FcW19zcyiV5GwRAGAXG6cQAMTnGewDhFjQISRENQ4KIxD0HQYIwG9SFIxIH5KSLKEAecA+rDgEhADKBPQQVBRQZBZiGIyrd28dVJ36bP6a7u6t59bv97LdbVe6t2Vf\/2PlX\/U2OlPXv27CE8IAACIAACIAACIAACqSFQCQItNb5ARUAABEAABEAABEBAEYBAQyCAAAiAAAiAAAiAQMoIQKClzCGoDgiAAAiAAAiAAAhAoCEGQAAEQAAEQAAEQCBlBCDQUuYQVAcEQAAEQAAEQAAEINAQAyAAAiAAAiAAAiCQMgIQaClzCKoDAiAAAiAAAiAAAhBoiAEQSJjAjBkzqGfPnlSzZk2aMGECtWrVKuEaEG3fvp26d+9O8+fPp8suu4yGDRuWqwPXb9CgQTR16lRq2LBhonVbvHgxde3albZt20ajRo2idu3aqfJ1fc844wwqKytLtE5+hfXp04emTZtWrr5+efTfbbzXiBEjaNasWTRu3DgqKSkJWrS1dE6fsdEmTZpEip1CsWmt0gkY0lxKS0vFfJPAa6KIGAlAoMUIF6ZBwItAmgUad\/ZDhw6N3MmG9byXQFu7di116NCBVq1aRX379k2VQNP1rV27trEosfFeWhyeeOKJIiLAKaa0z6PWpaIINOd7OL9shP1sIF\/2CECgZc\/neGMQyEsAAs0sODQv9yhkECsVTaCFYeDFqaIINH63KPERJIaQpmITgECr2P4turfTo0u64l7TgM5Rg44dO9Itt9ySe898Iyw6j07oTufuFC644AI1DZkvfT6wzk43X16vETTnO7Vp04ZGjx6tsjvrqRt7bdc9lZRPXDmZ6m\/y7ve9++67c1OeznfLNxqSb+TG+f7uUYMgvnWPoHFdnH7QdXPadvuW03iNWLin4jjNsmXLPEcMg0zbmbwr18kpYNwsTN\/LK87cZQR5h0INhJ+\/3PaDfla88nlNZ+vp9yCfRfdnw\/3Z4f8P8hlzxhLH\/r333kvXXXed5+itX5vCZep35f+WWs5QdJ0AKpwjAIGGYEgNAa+O1qvRL5TO3Ul5TcFom84Os1C6KB2PV1mFBJrTGU5xmu+dnWmSFGj5pmn1793iMahvTQVaIbvOTj+fIPISu\/nSur8s+DHw+mDpmPMTaH7vddxxx+WmfZ3l+NkPuu4xiL\/CCLRCftBfRoJ8Fp2+9RJnQdsNzaNly5aeX1CcbIPUzz2KaGOUNDUNNCqSOAEItMSRo0AvAs6G2zlqpDuKfGLF2SB6pdWNtzO\/V8fq7hR0B+DshAqtrXHmd4oTrzr5CTT36J4XG2e9NIMoAk1vEgg6xenV8eSbmjLxrckaNK\/RCWe9NJd8vnHWS\/uMY1Ovd\/PK74y3fKy8Rhe94jBf5x30vdyjQnqTgB8Dv6lIE3+ZTEc666U\/S\/wOerOK9gFvdNC\/47\/rz6LXe3mNYjrr5PzMOkVnkM+Yu03QeYK2KVx3Ez7oGUDATQACDTGRCgJBpsx0A6nTukdp3B0e7wb02qnobDS9vhW7hViQhdj5dh967YgsJNC8dsB5le+1+y9JgeYlDpYvX+65A9PEtyYCzR247pEULUScNt0dszuW3n\/\/fc8dtl4jg\/neyykEComhoKMr+d4rn0DzG9nz22Vp4i8TAZKvXu5dqPkEVr73dcaBe4TOS6AV+oy5\/+aOHZM2RdcrSPuRikYYlUgdAQi01LkkexUq1Mh7\/S1fg+dOe+utt3pOA3lNCRWqQ5AGNp9AKzRa6DUq6DVKF6R8LidpgeYe6ZkzZ84+67lMfWsq0ApNb3kJNPfaNDez5557Tr1DvsdrSswtwrym\/rymFgsJtCDvlU+wFMrLeQpNc5r6y4ZAc7M2\/SwWmjb1EmgmnzG3j6644orAbYp+r6Cj0tlr+fHGfgQg0PwI4e+xEzDtFCDQvM+6SlqgOf3Wo0cPWrhw4T7nqpn61kSgOTtmLTp+8pOf7DNFWUg8xyHQ9AfGSzg4R2jyCbSg7wWBNo6co7bMgwX4L37xi9zIOQRa7M03CoiRAARajHBhOjgBk2mVqFOcXrUy\/dbutpFvGk3\/fsiQIblDV\/12cboPHM33vvz71atX586\/SlqgOUftGjduTFu3blVY3LvVTHxrItD0+zo7Ya91SjamOE1Gebziy6sO+QRa0PfKJ9DyTSUG\/TSa+CvMCJoWUvoQYq5vv379cnFj8ll855131JS0U\/j6rUErNIIWdoqzEFsvfwb1BdJlmwAEWrb9n5q3N1mYnG+NT9BNAl4iwKRT8DqtPd9CdOd0k55eMxVoXmy8FlzrzoqdqtdauY9jyHfMhukmAR047uk8r87PxLdhBJrXTlaun61NAvmEUKG1gXxEhJ9w9BNofu+Vr15eIjVfWq8GwMRfJgLNK2b5s+T+3Dp3VLqnj93MnTHv\/nzxuwUdQfN6Z5NNAoVGaYMuUUhNY4yKpIYABFpqXIGKFNra73fulZOes1HPd16Uu\/GOKtDYXr5jB9xlmQo0Z+fqFSVeO07zRZOfQCu0yNrLZr5OzJ02qG\/9xLO2y+\/B05n6WiivugU5d+yoo46ipUuXlhuBKbSGy+t4B\/eoS6E1UU7R5WanR5ZM3ivfBoKg75AvToL6y0SgcVmF2PjtmvUSmVy+3nXr9S5BBZqXL9ieHhnmq8fyfelxluv+gmLKB70ACDgJQKAhHlJFwN2A+x1U+\/vf\/5569eql7m7kJ+hBte5v5jYEGpfvJQi97rp038UZ5Fu2u9P1YuM1ouU8zNdPoLk7Kr8df85O1++MrSC+LbQb1uvgYLfNoIfP6rp6bWzwEtuFWHN697SuVxx4sXTXX8dv0Pdyl+MUCO5Y8POPuyEI4q8wAsTri4zzc2v6WXTbY1uHHXbYPrtxg3zG3KPzzo1G+XYAa25eO3YLHWacqoYXlUklAQi0VLoFlSpEIEhDC4IgEIRAlB126HyDEC7uNEGPQsn3ll5n2hU3EdQ+SQIVWqBx4zt58mTfS4zzTU1FvfQ3SUdmqSwItCx5O\/q7OjvZfIvJ\/Q5vzVcLLfDC5o\/+drBgg4BzdNo5UhZ1wwXiw4Z3smujwgo0Lbpq167tK9Dcu4iyGw7F8eYQaMXhpzTVstCaKq6n192dQepv0s4EsYc0MgQKrR\/lGoX5su6cqg0bXzI0UGpaCFRIgZZvR12hb8GzZs3KHVeQFuegHt4EINAQGWEIeC2cN12X5VWujkd0wmG8kp48QS6gN6mtFn2lpaXoW0zAIW2OQIUUaPrqkDZt2tArr7ziO4LGDSw\/+qgBxAcIgAAIgAAIgAAISBKocALNOV3JO7T81qDpb018Hg9vt9cP1pRIhiXKBgEQAAEQAIFsE6hQAk2LrU6dOlFZWZm6m9BPoOlh6HPPPTc3gqbtNGrUyHdoetWqVdmOILw9CIAACIAACCREgDf6ZOWpUALN6+obP4GWz9F6906hdSUszviKknnz5mUlXvCeIAACIAACICBG4KSTTiK+Oi8LQq3CCDSvnZhBRtDyRZkeWeNLoHk0zuv55z\/\/SZ07d1bBwidO40mOAIvi4cOHg31yyMuVBP5C4InUF0LEPvjLEZArWcc+b+qDQJPzg3HJftvo850wb0OgZSVYjJ0SYwYtjsE+RsgFTIO\/DHcuFezl2IM\/2CdJoMKMoHlBCzKClm8qM8jZaGgokwzV8mWBvRx7dFJgL0tAtnS0PXL8s8Y+8wJNn5m2evXq3HEcXhsHCk1xYhQn+Q\/sypUrafz48TRw4ECqUqVK8hXIeIngLxcAYC\/HnksGfzn+EGhy7K2X7DWCxr8bPXr0Ppcbu6dIg0yJZi1YrDsogsF\/\/\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\/+IACNkB\/sI8CxkBX8LEEOaYIE2Z\/lmer\/P4RBoIRl6ZtuyZQuNHDmS+GfYp3bt2jRgwICw2a3ng0CzjjSwQTSSgVHFkhD8Y8EayCjYB8IUWyLwjw2tr2EINF9E4RKsXbuWOnToQKtWrQpngEgp5lmzZoXObzsjBJptosHtoZEMziqOlOAfB9VgNsE+GKe4UoF\/XGT97UKg+TMKlUILtIEDB1K7du2MbcyYMYMGDRoEgWZMrmJmQCMp61fwl+MP9nLsuWTwl+PfetBc+mLjvzHFadsF69evpxtvvFH9O\/30043Nz549mx555BGaMmWKcd64MmAELS6y\/nbRSPozijMF+MdJt7BtsJdjD4Emyx4CLSb+P\/zwA\/G\/atWqxVRC8mYh0JJnrktEJyXHHp0U2MsSkC0dbY8cfwi0mNjzFGfnzp2padOmVFZWRm3btqXKlSvHVFoyZiHQkuHsVQoaSTn2EGhgL0tAtnS0PXL86\/WZqQrHLk7LPuDdm3\/5y1\/o5Zdfpu+++44aNGhA1113HXXq1Inq1q1rubRkzEGgJcMZAk2Oc76S0UnJ+QTs5djjy4ksewi0mPlv376dZs6cSY899hi9\/\/77qrTjjjuOevXqRWeddVZRTYFCoMUcLAXMo5OSY49OCuxlCciWjrZHjj8EWoLseePAM888Q+PHjyf+75KSErrqqquoW7du1Lhx4wRrEq4oCLRw3GzkQiNpg2J4G+Afnl3UnGAflWC0\/OAfjV\/Y3Lx7k9eg8YMpzrAUQ+Tbs2cPffLJJzR27Fj6+9\/\/Tjt27KAWLVqoUbVLL700taNqEGghnG0pCxpJSyBDmgH\/kOAsZAN7CxAjmAD\/CPAiZIVAiwDPVlZenzZ37ly64447lMmpU6dSw4YNbZkPZGfEiBE0efJk37Ih0ALhjCURGslYsAY2Cv6BUVlPCPbWkRoZBH8jXNYSQ6BZQ2luSAsznu5kgcZHcvCBtvfeey\/xFU9JPYsXL6auXbuqMv3EIQRaUl7Ztxw0knLsuWTwl+MP9nLsEfty7N9etpkuHbFQVQBTnAn4gUXYokWLaNKkSbmpTcndnbyBoXv37jR\/\/nx1rRQEWgJBELIIdFIhwVnKBv6WQIYwA\/YhoFnMAv4WYRqYgkAzgBU2Ka834\/s4eaTsxRdfpG+++UatMbvwwgupR48edMQRR1ClSpXCmo+Uj6c2+a7PNm3a0CuvvAKBFolmvJnRSMbL1886+PsRiu\/vYB8f2yCWwT8IJftpJr27lnpPWoIRNPtoiXbu3ElPPvkkTZgwgVavXq1EWGlpqToL7YILLlA7OCUfvuuzX79+qn5z5szBGjRJZwQoG41kAEgxJgH\/GOH6mAZ7OfZcMvjL8B88fQUNnr4SAi0O\/Pqy9M2bN6fuKA1dNz40l285MN0k8PTTT6spUX6S3tAQh6+KwSYaSVkvgb8cf7CXYw+Bljx77p\/5ufnlb2jmJxsh0OJwAY+grVy5klq2bJm6YzP69OmjRvXGjRunRvJMBZqTF28wuPbaa+NACJsOArt27VJn57EgrlKlCtgkTAD8EwaO2JcD7ioZsZ+sK5544gk1s7X9tFvp+\/pHQqDFgV+PUg0cOFDtzjR9eApy0KBBao2Yzcc5tdmqVStl2lSgDRkyJHewLgsGjKLZ9JC3LRb869atUyOXEGjx83aXAP7JM9clgr0cey4Z\/JPlz9qB\/104dWeuYOzitOyDtAo0Hj2bNm1a3rft27evmvb0enDMhuUgMTCHaR4DWDEkBf8YoAY0CfYBQcWUDPxjAlvArHMHJyeDQLPsAy3QeAdn2IdHS2yPoHnVxXQEjeuk16CFfTfkMyOARtKMl+3U4G+baHB7YB+cVRwpwT8OqoVtOndw7rdjAy0aeEom+txKe\/jciwSeLVu20MiRI4l\/hn348NgBAwaEzR44HwRaYFRiCdFIiqFXBYO\/HH+wl2OP2Jdhf8mjC2nO8s2q8CobPqZ\/3XsxBJqMK+RLhUCT94FfDdBJ+RGK9+\/gHy\/fQtbBXo49BFry7J1XPHHp+y99keaPuQUCLXlXFFeJWIMm5y90UnLs0UmBvSwB2dLR9iTL373+rNaM\/vT235+DQEvWDcVXGgSanM\/QSMqxh0ADe1kCsqWj7UmOP4+e8e0BzunNkrfvV2vRs7DuO7E1aMm5NLmSINCSY+0uCY2kHHsINLCXJSBbOtqe5Pi7R8\/++PNd9OgdZRBoybmgeEuCQJPzHRpJOfYQaGAvS0C2dLQ9yfDn0bNLRywk\/snPIfX2p1HnVaLOnTtDoCXjguIuBQJNzn9oJOXYQ6CBvSwB2dLR9iTD\/7E5X1HfZz\/JFfbo1aXU4oeVEGjJ4N9bCp\/ysWnTJvrhhx+oXr16tN9++9Hu3btTdx2UFxMItCQjpXxZaCTl2EOggb0sAdnS0fbEz9957hmXdmrLOvRS7zaUtT5XbA0aC7PXX3+d7rjjDnWnIi\/4mzp1KlWvXp1+97vfUdu2bYlP8a9WrVr80RCyhKwFS0hMsWRDIxkL1sBGwT8wKusJwd46UiOD4G+Eyzix+1gNntp8sayNmuLMWp8rJtD+8Y9\/UI8ePej444+nww8\/nGbOnKkEGl9Wftttt9H06dOJ7+3ky8fT+mQtWNLkBzSSst4Afzn+YC\/HHqPH8bJ3izMujac2rz6hoSo4a32uiEDbtWsX9ezZk3788UcaNWoUzZ49W12EzgKNLxr\/7rvviO\/I3LBhA40bN06JtjQ+WQuWNPkAnZSsN8Bfjj\/Yy7GHQIuPvXtak0vikbPTDquTKzRrfa6IQNP3ct5www3UpUsXmjFjRjmBxt6YOHEijRkzJifa4guL8JazFizhSdnPiU7KPlMTi+BvQstuWrC3y9PUGvibEiucnkfNJr27hgZPX1ku4Yp7TqfaNaqU+13W+lxRgXbNNdfQ9ddf7ynQ+LqlZ555hqZMmUINGjSwGxGWrGUtWCxhs2IGjaQVjKGNgH9odJEzgn1khJEMgH8kfPtkbj1obu4oDf3HjcPO9iwka32uiEDTU5w7duygsWPHqnll5xTnqlWr6LrrrqOmTZuqKVDeOJDGJ2vBkiYfoJGU9Qb4y\/EHezn2XDL42+E\/cd4a+v2UpeWM8UaA\/u1b5NacuUvKWp8rItAY+sKFC4mnOHmDAP979dVX6Q9\/+AMtW7aMnn32WbUObfTo0XTmmWfaiYYYrGQtWGJAGNokGsnQ6KxkBH8rGEMZAftQ2KxlAv\/wKHk6k69tGjx9xT6jZs7dmvlKyFqfKybQ2AHvvPOOOmZjxYoV5fzBU5r33HMPnXfeeeEjIYGcWQuWBJAGLgKNZGBUsSQE\/1iwBjIK9oEwxZYI\/M3RsjDjfzdOXuIpzB7udBSdflhdX8NZ63NFBRp7g89D492aH330EfHU59FHH612clauXNnXWdIJshYs0ryd5aORlPUG+MvxB3s59lwy+Jvxn\/nxRvr16MWemfgAWj5Gg0fPgjxZ63PFBVoQp6Q1TdaCJU1+QCMp6w3wl+MP9nLsIdD82evRMp7G5OlMryfIdKZXvqz1uSICbcuWLTRy5Ejin35P\/fr11VTnMccck7pRtawFi5+vkvw7Oqkkae9bFvjL8Qd7OfYQaPnZszB7ftHXdNfLy\/MmYmH2SKfScmebmXgza32uiEDjq51uvPFG+vDDD4l3cvLDQowfnu70ek4\/\/XR6+OGHqVatWib+jDVt1oIlVpiGxtFJGQKznBz8LQM1MAf2BrBiSAr+5aGOmb2KBjz\/aUFRdu5R9ejmc5oFnsrMZyxrfa6IQGP48+fPp969e6urnH7729\/mbgvg3Zt\/+9vfiM9Be+SRR+hnP\/sZvfjii+oYDk57yy23xPCRC2cya8ESjlI8udBIxsM1qFXwD0rKfjqwt8\/UxGKW+evpSz5Ylk\/+L\/REHS3zsp21PldEoG3fvp26d+9ORxxxBN11111UqVKlcr7gjQN33nknff755+octBo1atDgwYPVeWnPP\/+8yWcp1rRZC5ZYYRoaz3IjaYgqluTgHwvWQEbBPhCm2BJljT+Lss827KAHXv8875oyDZtF2T2\/OoyObVwz8mgZBBqRiEDTVz3xCFrHjh09P0h8g8Cjjz6au+rpueeeowcffJBmzZoV2wfP1DAEmikxe+mz1kjaI2fHEvjb4RjGCtiHoWYvT0Xmz2KMH17c\/\/T8Nb6CjNOyKLvrkpbUpmmtWESZ03NZ63NFBNqmTZuoW7du1LJlS7r33nupWrVq5T49PM15++230\/Lly+mxxx6junXrqvVnfJjta6+9Zu+TFtFS1oIlIi6r2StyI2kVVEzGwD8msAHMgn0ASDEmqWj83162mb774Uf66xv+I2TOkTJe7M\/iLOgRGTZckrU+V0SgsaP4IvShQ4fSVVddpTYM8Nln\/PDoGq89mzx5slpv1qtXL7Ve7bbbblNnpLFQS8uTtWBJC3cgNKkxAAAgAElEQVSuR0VrJNPENkhdwD8IpXjSgH08XINaLVb+enSMf05b9DV9vO7bQCNkepTsrCPq0ZXH\/yT0DsygfAuly1qfKybQeJSMBdrjjz9OP\/zwQzmf8CG1vHGgb9++qiPmC9VXr16tRF1paakNP1uxkbVgsQLNkpFibSQtvb64GfCXcwHYy7Evti+HPDr2+cadNPndtYHFmBZkfLJ\/x7YNEx8lg0D7DwExgaarwEduvPnmm7R48d6ThnmU7Oyzz6bGjRur\/9+5c6caVWvUqFHqLk2HQJNrKNFJybEvtk5KlpT90hH79pmaWEwbfz0yxu\/w94820EuL1xuJMS3I2v\/sILrkuINTJcjcfslanysu0Ew+GGlLm7VgSRP\/tDWSaWKTRF3APwnK3mWAvRz7NHw54VExfp5btI4+XbcjlBjj\/D3PaErHNCoRnbI09WTW+lwxgcZHaXz22Wc0bdo0z8Np+V7Or776ioYPH55bn2bqzLjTZy1Y4uZpYh+dlAkt+2nB3z7ToBbBPiipeNIlwd+5Xmz6Rxto0ZfbjIWYHhnjn9ee0ohOaFY71aNjQbyVtT5XTKDxjsw+ffoQr0XzenhnZ9u2bdWmAN7FmcYna8GSJh8k0Uim6X3TVhfwl\/MI2Muxj2METR\/+uuDzLfTm0o2hhJhTjF3W+mA676iDil6MeXk5a32uiED79ttvqUePHmrh\/0MPPUTNmjVTGwHatGmjbheYNGkSjR8\/Xh1Se+yxx8p+GguUnrVgSZMj0EnJegP85fiDvRz7KAJNC7HJ766hzzf+O7QQ02Ksad396b\/Oa0bVKu9XIcUYBJrwQbV8xMZNN92k\/PDnP\/+Zvv76azWlyQ+PrlWvXp2GDBmyz00Dsh\/P\/5QOgSbnCXRScuyjdFKyta4YpSP2Zf3oxZ\/FF58Hpqcm563cQhPmrqYvN\/0797swtWabLMRObFGbzj6iXmaEWD5WWetzRUbQ9E0Cf\/jDH+iKK65Qvpg4cSK99NJLNHbsWHUhOv\/\/U089RU8++WTuIvUwAR5nnqwFS5wsTW2jkzIlZjc9+NvlaWIN7E1o2U\/7yerN9P5na2h7pRJ6dtGGyCLMOSJ24TH1c9ckJXkArH1K8VjMWp8rItC2bt2qpjSPP\/546t+\/v\/LkG2+8oUbRWJg1b95cCTUWZ1OnTsUmgXhivaitopOSdR\/4y\/EH+\/jZ6+nI1z7cQO+v2mZFhEGIRfcbBFp0hoEsPPDAA+qQ2ltvvZU6deqk1qN16dJF3Szwy1\/+kgYMGEC801Nf9RTIaMKJshYsCeMtWBw6KVlvgL8cf7APz15PQerpyO++\/5EemvkFrdiwM9KaMGeN9LTkqS3r0GmH7d3glvSVSOEJpTtn1vpckRE0DgEeReP1Z\/yTRVidOnVo0KBBanMAC7NKlSrRnXfeSV27dk1txGQtWNLkCHRSst4Afzn+YO\/PXo+AccoZH22ghV\/aGwVjm41qVaEWDUqoZYMD1PVHejoS05L+vomSImt9rphAYyexENu2bRvVrFlTCTK+8umdd96h2bNnU\/v27dUUKP8+rU\/WgiVNfkAnJesN8JfjD\/akFt7r0bAXFn9NS9d+a20aUntWj4S1alKTLji6fm4k7OADiNasWUNNmzalKlWqyAVCBkvOWp8rKtAKxReLtS1btlDt2rWJ7+ZM45O1YEmTD9BJyXoD\/OX4Z4G9FmCfrPuW\/vXFNnWfZNQdkW6P6dEunoq86NgGVGv\/KoFGwrLAXy66C5ectT5XRKDpXZwDBw6kdu3aeXrklVdeocGDB2OTQFo\/KcL1QiMp6wDwl+NfrOydd0ZqevNWbKEJ\/1yt\/nfO8r1XGNl6ctOOdfents1r03WnNMqZjjIVWaz8bXGVtAOBFhN9vrqJpy937NhBmzdvpgcffJAuv\/xyatWq1T4lclpel8Zpp0yZQg0aNIipVtHMZi1YotGymxuNpF2eptbA35SYvfRpZ6+F2KJV22jGR9\/Q59\/YH\/1imlpk8TlhJzSvTYcffIA6Myzu9WBp528v0tJnKWt9bqIjaBMmTKC77rpLrT3ze3has1+\/fuo4jrSuQ8tasPj5LMm\/o5FMkva+ZYG\/HH8p9s51X1t27qbXl2yk5et3WJ961GSdAozvkjyxeW217kx6R6QUf7mIS0\/JWetzExVofO8mrytbt24d9ezZk26++WY6\/fTT9\/E+38PJ92+mVZjpCmctWNLzMSVCIynrDfCX4x8He6f4Wr1lF839bDMt+zoZ8XVck5rU\/mcH0X6VKsU++mXDa3Hwt1GvLNjIWp+bqEDTAcQbADZu3KhuDODrnIr1yVqwpMlPaCRlvQH+cvxN2TvF16v\/t4E++GqbqrztNV9OInoHJE878lEUlxxXfplKlDVgcuT3lmzKX7q+Fan8rPW5iQk0Lcr4Z9CHpznr1auHXZxBgWUoHRpJWWeDvxx\/zX7PgQ3UMQ8swBZ+uVUdNcH\/bXu3o\/tNnbsfWYC1bVZbJYl77Zcc8fIlI\/blPAGBFhN7vXNz1apVgUto0qQJdnEGppWthGgkZf0N\/vHxd454LVu\/Q63zWvRl\/KNeWmTphfbHNq5JxzQqyYmvYh71suktxL5Nmma2INDMeAVOvXPnTpozZw7xDs2gD09\/nnrqqVSjRo2gWRJNl7VgSRSuT2FoJGW9Af7m\/J3Cq3Gd6vTU\/DXEx0zwE+d0o3N0i8XX8YfUovNK61GzejVSsejenKRsDsS+HP+s9bmJTXHKuTS+krMWLPGRNLeMRtKcmc0c4L+Xpj5SQu8u\/GzDTpq3YjO9vWzvmV5xC6\/yo1416PTD6lCTuvvnRr2c4sym\/7NsC7Ev5\/2s9bniAm3Tpk309ttv03vvvUe8e7NNmzZ08sknq12caX+yFixp8gcaSVlvVGT+XgeqbtqxmybOWxPLlUL5POk8ZuLco+rl1nrxVUPr16+nn\/70p3TowXunIPEkR6Aix35yFMOVlLU+V0yg8WaBMWPG0LBhw9QdnM6HNwf07t2bysrKlGhL65O1YEmTH9BIynqj2Ph7iS4m+N4XW+mNJd8ksrhee8wpvH7Rsg7xVUN8xIRztKvQeq9iYy8bqfZLB3\/7TINazFqfKybQnnnmGerfvz+dccYZdMstt1Dz5s2Vj1auXEkPPPCAunVgyJAhdOmllwb1XeLpshYsiQMuUCAaSVlvpIW\/Fl4saPR\/f7vrB5owbzX931fbFaQkphmd4orXeZ15RF11sKrzbC9nmijeSwv7KO9QzHnBX857WetzRQTa9u3bqXv37lRSUkIPP\/zwPpsAeEPBTTfdpEbWRo0aldqz0rIWLHIfy31LRiMp6404+XuJLn7b5xauozeXblQvHvdREk66+kyvyvtVouMPqUnnHHlQ7kgJ96hYEl6Jk30S9S\/2MsBfzoNZ63NFBJo+cuOGG26gLl26eHp74sSJagp06tSp1LBhQ7mIKFBy1oIlTU5AIynrDVP++UTXG0u\/ofc+35roFKN7tKtZvf3ptMPr0i8OrZPb1WhrtCsOL5myj6MOWbYJ\/nLez1qfKyLQeIFrx44d1WXpPFLm9fDI2vPPP4\/L0uU+C6kuGY2krHuY\/3sff5lbqO5c4zXr0030zxVbYrsku9CbO9d3ndSiNp15+H+ujKsoB6ki9uVjf82aNdS0aVN1UDCe5AhAoCXAevfu3dSnTx9asmQJjR07llq0aFGu1BUrVqhL0ktLS9UmgqpVqyZQK\/MishYs5oTiy4FOKh62+RbTz\/x4I81fuSXxkS73aNdhDWrQ2UcdRPUO2NsmVBTRZeJNxL4JLftpwd8+06AWs9bnioygsTM++OAD6tatmzq49vzzz1cH0vLDh9m+\/vrrat0ZT3HysRtpfbIWLGnyAxpJM294Ca+3PtlEf3tvrTKU5Jout+hikXXyoXWoyn6VSJ9ir98Op9fv62fEvlns204N\/raJBreXtT5XTKCxSz7++GO6\/fbbadGiRbRnzx7lpUqVKlHr1q3p3nvvpSOPPDK45wRSZi1YBBDnLRKN5H8OSdWQdn3\/I01b9DXxFGMaRNfxTWvRAdUrQ3RZ\/uAg9i0DNTQH\/obALCbPWp8rKtByHcuuXcQH1vLDB9Ty6FkxPFkLljT5pKI2kvo6ID1yxFcBzV2xmT5dt0PhT+rICPcIlh7ZatusFu1ftTI1qLGHavywXa1B43U4GOlK7tNRUWM\/OYLRSgL\/aPyi5M5anysi0HiTwGOPPabOODviiCOID6YtxidrwZImHxVTI+m+g\/HZhevoH5\/IjHKxD50L6ZsfVINOO6wO\/biHjEa6iol\/muLWRl3A3gbF8DbAPzy7qDmz1ueKCTTexcmH0h500EFKqF133XXUpEkTNcVZLE\/WgiVNfpFsJJ33LzKTtVt30TvLN9OnX+8d5Up6PZdTdLH4OuLgA9Vl2CXV9+4wc45u2RrpkuSfpjiUqAvYS1D\/T5ngL8c\/a32uiEBj93733Xc0d+5cGj9+vPrJ\/8+7OfXxGw0aNJCLgoAlZy1YAmJJJJntRtIpur7\/cQ+99\/kW+senm4iXRkoKLp5abNmgBtU7sBqdc2RdOqRejVSc1WWbfyJBU0EKAXtZR4K\/HP+s9bliAs3pYrdY42M4+IgN3uV58cUXp\/Y+zqwFi9zHct+SgzSSTtG1eNU2JbQ+XL33+h9p0dWqSU06+dDaVKdGVdFT6cP6NAj\/sLaRrzABsJeNEPCX45+1PjcVAs3p7q1bt9Ljjz+uRtZq1qyJmwTkPgupLFmLrs++3k5vL\/mKtu05gD5ak+ydixqMey3XMY1K6GeNDqRKVCmWacU0OQSdlJw3wF6OPZcM\/nL8IdAE2PPdnDNnzlRijB3Az3HHHadG0PiMtGrVqgnUyr\/IrAWLP5HwKZwL6XlNFx8VsWLDzsRHupyi69jGNdUC+lr7x7eWKzwx2ZzopOT4g70cewg0WfZZ63PFRtDcoowvRuc1aL\/97W\/poosuUsdthHn4hoJp06blsvJl6+3atStoavHixdS1a1fatm1buXQnnngijRs3Tl3q7vVkLVjC+EPn0SNf33y7m\/5nzlfqGqAkjoxwCi6uy\/mlB1GbQ2qqUS5+4lhAH4VTseSFSJDzFNjLsYdAk2WftT5XRKDpy9JXrVpFvBngmmuuUfdyNm7cOJL3WZwtWLAgNy06Y8YM6tmzJ\/Xt25fKysry2uZ0\/fr1owkTJlCrVq0C1yFrweIH5u1lm1WSd1duof\/9eGNso18sqr7\/\/nv6ac0qtP\/++9MFx9SnVo1r5qqXxet\/\/Hxj++8QCbaJBrcH9sFZxZES\/OOgGsxm1vpcEYHGh9JOmjSJ2rdvT4ceeqiVozX0KNiQIUPKjZixaFu9enXBkbARI0bQrFmzCqbBCFp5Ajwi9uL762n6hxusjoSxuOKdi3w+11lH1KWf1Np7aLFbdKGRDNagxZUK\/OMi628X7P0ZxZkC\/OOkW9g2BJoc+1hKDiLQOA0\/fDG7yZOFYNFrw976ZCPxqfZRpiW1yDrz8LrU\/uj6+6ztMjmjC42kSaTaTwv+9pkGtQj2QUnFkw784+EaxGoW+lwnB5ERtCCOsJGGR8aGDh1Khdah6elWXme2dOnSXLGXXXaZr2DTwfL000+rQ3b5adiwoY2qi9lYvfV7alSrCg15\/XN657OtxoKM8\/II2Kkt69Bph+1dR8jCy0R8BXl5NJJBKMWXBvzjY+tnGez9CMX7d\/CPl6\/bOvfR+uFlUZ07d1YzXrrPTbY2yZZWIQWaXnvGKP2Elp4aPffcc3OCTIu2Ro0aBdok4HQZbza49tprk\/VixNJYlK3Z9j29snQ7vfDR3iMr\/B4WYrwG7OKjSqhx7b3\/zb9L6tm1axfxlWEsiPkuSDzJEgD\/ZHk7SwN7OfZcMvgny\/+JJ55Q68OdDwRasj6IpTTeKdq9e3e1Bo2P8DAZ3dIir9Domx5B43VveoMDl2FSTiwvbmB08PQVNPwf\/\/mGki8rj4D1PrMJHd6ghhoNS1KMedVp586dtG7dOvUtCgLNwOGWkoK\/JZAhzIB9CGgWs4C\/RZgBTPGAiR5FmzdvHg0fPhwjaAG4FUUSPULWo0ePgjs53S8TJF8xzofrNWUszAqtJ2MR1rFtQzr9sLrqLLC0PZhmkPUI+MvxB3s59lwy+MvxL8Y+NwotkSnOLVu20MiRI9U1Tsccc4xn\/d977z168MEH1bRjlHs5gwgtrwoEyVdMwaKF2Y2Tl6i7HL0eFmWPdCqNZc1YlCD1yotG0jZRM3vgb8bLZmqwt0nT3Bb4mzOzlaOY+lwb7ywi0PQar4EDB3oeIsuH1vLi\/ldeeSXw1GS+s8z8pirz\/T3I2WjFEiyT3l1LPGLmJcxYlA2+4ggqbXig9YX8NgI0nw00knHS9bcN\/v6M4koB9nGRDWYX\/INxiiNVsfS5tt49MYHGF6Lffvvt9NxzzwWu+znnnEMPP\/ww1ahRwzePXm\/GCfXp\/3oUjC9ez3cjgNc6Na+NA14VKIZgueTRhZ5TmSzM+rdvQVefUJy7TtFI+n4kYk0A\/rHiLWgc7OXYc8ngL8e\/GPpcm3QSE2hc6RUrVhDvyOCDavnuzdatW3veHsCXpLdp04ZOO+00dWG6yeO+6sl9iwAfvTF69Oh9bg3wy1dsAo1HzXpPWrJPtfU0ZhrXlZn4GY2kCS37acHfPtOgFsE+KKl40oF\/PFyDWIVAC0IpYpoga9AiFpFI9rQGCwszFmjOp6IIM\/1OaCQTCfG8hYC\/HH+wl2OPETRZ9mntc+OikugIWlwvIWU3bcHCa8wuHbGw3FozFmY8jcnTmRXpQScl603wl+MP9nLsIdBk2aetz42bRmICTY+a8Qv95je\/oaeeeor4d4We2rVrU69evYh\/pvFJU7DwReUsztyjZi+WtSmqxf9B\/YxOKiipeNKBfzxcg1gF+yCU4ksD\/vGx9bOcpj7Xr642\/p6YQNM7N7nSvPD\/pptuIr62odDDh5CaHjBrA0pQG2kJFt6hOXj6ynLVvvS4BjT+Ou8jTIK+X5rToZGU9Q74y\/EHezn2GEGTZZ+WPjcpCokJtKReKMly0hAsXuLs0atLi3Z3ZlD\/oZMKSiqedOAfD9cgVsE+CKX40oB\/fGz9LKehz\/Wro82\/Q6BFoCkdLO6dmhVtI0Ah16CRjBC4FrKCvwWIIU2AfUhwlrKBvyWQIcxI97khqhwpi4hA0+vRsAYtvO+8xFkxn2tmSgKNpCkxu+nB3y5PE2tgb0LLflrwt880qEUItKCkIqTT69GwBi0cRN6t2XrQ3FzmLI2c6ZdGIxkudmzlAn9bJM3tgL05M5s5wN8mTTNbEGhmvKym3rNnD23YsIEmT55ML7zwgrqLM99dnVYLDmlMKlhYnDmvbeKdmsV+8KypC9BImhKzmx787fI0sQb2JrTspwV\/+0yDWpTqc4PWz3Y6kSlOv5dgoXbnnXeqGwf4svSqVav6ZRH5e9LBwqKMD6Gds3xz7n2zsCHAy7loJEVCPlco+MvxB3s59lwy+MvxT7rPlXvTvSWnUqBxxaZMmUKPPvoojtlwRIh73dl\/nduM7rjoUOkYEikfjaQIdgg0WeyqdMS+rBPAX44\/BJoc+1zJ+mL1Dz\/8kJ588kmqX79+Cmq1bxWSDBavdWeLBp6SSi5JVAqNZBKU85cB\/nL8wV6OPQSyLPsk+1zZNxUcQfPbxfnZZ5\/RggULqGvXrvSnP\/2JKlWqlAZW+9QhqWBxT21mcVOAGz46KdmPBPjL8Qd7OfYQaLLsk+pzZd\/yP6WLTHH67eKsVq0aXXHFFTRgwACqVatWWliJCTT31GZW1505HYBOSvZjAf5y\/MFejj0Emix7CDRZ\/kVVehLBgqlN75BAJyX7UQF\/Of5gL8ceAk2WfRJ9ruwbli9dZAQtHwDevblt2zaqWbNmaqc1nXVPIlguH7mI\/vHpJlUsT21W1MvPTT8U6KRMidlND\/52eZpYA3sTWvbTgr99pkEtJtHnBq1LEunEBBpvBPjrX\/9KvBFgxIgRVFJSoo7V6NatG61evZr++Mc\/0iWXXJJqoRZ3sLy9bDNdOmJhLg5uPucQuvPilknERerLQCMp6yLwl+MP9nLsMYImyz7uPlf27fYtXUygjRkzhoYOHarWmg0cOFAJtJ07d9L06dNp3LhxtGzZMnUG2oUXXpg2Zrn6xB0slzy6MHfmGUbPyocBOinZjwX4y\/EHezn2EGiy7OPuc2XfLiUCTY+UHX744XTPPffscxAtj6716dNH3SrAYo3FWxqfOIPFPXrWv31z4rs28ewlgE5KNhLAX44\/2MuxR9sjyz7OPlf2zbxLFxlB07s4e\/fuTR07dvSs2XPPPaeuepo6dSo1bNgwjewozmBxj55l+cwzL+ejk5L9SIC\/HH+wl2MPgSbLPs4+V\/bNUiTQeGTsmmuuobPOOov69+\/vWbPBgwfTW2+9lcmDat2jZ1m8a9Pvw4JOyo9QvH8H\/3j5FrIO9nLsIdBk2UOgJcCfd2vefffd9PLLL9P999+vhJo+jJb\/xsLs1ltvpYsvvjiTB9Vi9Mw\/CNFJ+TOKMwX4x0m3sG2wl2MPgSbLHgItIf6rVq2iG264gZYuXaoOo+WjNfjZtWuXWnt21FFHEW8kaNKkSUI1Mi8mjmBxj57x1CZvEMBTngA6KdmIAH85\/mAvxx4CTZZ9HH2u7BsVLl1kDZquEu\/a5DVm\/G\/r1q3q1yzWOnTooP7VqFEjzexiWYP2q5GLaLbj3DOsPfMOAXRSsh8N8JfjD\/Zy7CHQZNlDoCXAn6cxX3jhBTr66KOJd3IW62M7WNy3Blx9QkPia53w7EsAnZRsVIC\/HH+wl2MPgSbL3nafK\/s2\/qWLjKCtX79e7d688sorqayszL+WKU1hO1hmfryRfj16sXpbnHtW2OnopGQ\/FOAvxx\/s5dhDoMmyt93nyr6Nf+kiAk3v4uSbAiDQ9jqJR8\/41gD+yc9ZR9Sl53q29vdgRlOgk5J1PPjL8Qd7OfYQaLLsIdAS4v\/GG2\/Qn\/\/8ZzWSdtFFF9EBBxywT8mVK1emevXqEf9M42MzWHC0hpmH0UmZ8bKdGvxtEw1uD+yDs4ojJfjHQTWYTZt9brASZVOJjKDpg2p5J2ehh3dwZuWgWhytYfZBQCNpxst2avC3TTS4PbAPziqOlOAfB9VgNiHQgnGKlIp3b86ZM0cdqVHoqV69Op166qmp3c1pK1jcmwNwrZN\/eKGR9GcUZwrwj5NuYdtgL8eeSwZ\/Of62+ly5NzArWWQEzayK6U1tK1ic05vYHBDM32gkg3GKKxX4x0XW3y7Y+zOKMwX4x0m3sG1bfa7cG5iVDIFmxqtcalvB4pzePLVlHXqpd5sItcpGVjSSsn4Gfzn+YC\/HHiNosuxt9bmybxG89MQEml53xlV7+OGH6aabbiKsQdu7e7P1oLk5j+HezWDBi04qGKe4UoF\/XGT97YK9P6M4U4B\/nHQxguYkkJhA27JlC40cOVKV\/Zvf\/Iaeeuop4t8VemrXrk29evUi\/pnGx4aan\/TuWuo9aYl6PZ7exM0BwTyNRjIYp7hSgX9cZP3tgr0\/ozhTgH+cdCHQRASanEvjK9mGQHNOb55+WF16oQxnnwXxGBrJIJTiSwP+8bH1swz2foTi\/Tv4x8u3kHUbfa5c7c1LTmwEzV01vu7ptddeo1mzZtEdd9xBBx54oLqPkw+urVq1Kg0YMICOPPJI8zdKMEfUYMH0ZnhnoZEMz85GTvC3QTGcDbAPx81WLvC3RdLcTtQ+17xE2RxiAu3VV1+lPn36UKtWrWjUqFFUt25dNeV5\/\/3300svvUR8xMaYMWOoTZv0LpiPGiyY3gwf\/Ggkw7OzkRP8bVAMZwPsw3GzlQv8bZE0txO1zzUvUTaHiEDbvn07de\/eXZ1vxhsGSkpKylHYtGkT9ezZU90uwOKNxVoan6jB4pzePO2wOsQbBPAEI4BGMhinuFKBf1xk\/e2CvT+jOFOAf5x0C9uO2ufK1TxcySICTe\/ovOGGG6hLly6eNZ84caIaQauoNwm4pzcfvbqUrj6hYTgvZjAXGklZp4O\/HH+wl2PPJYO\/HH8ItATYr1+\/Xt3Befnll6vjNryeESNG0DPPPENTpkyhBg0aJFAr8yKiBAvu3jTn7cyBRjIav6i5wT8qwfD5wT48Oxs5wd8GxXA2ovS54UqUzSUygrZ79261\/mzJkiU0duxYatGiRTkKK1asoOuvv55KS0tp2LBhatNAGp8owYK7N6N5FI1kNH5Rc4N\/VILh84N9eHY2coK\/DYrhbETpc8OVKJtLRKDxK3\/wwQfUrVs32rZtGx1\/\/PHUrFkzRWLDhg00e\/ZsqlmzJj322GN07LHHyhIqUHrYYMHdm9FdikYyOsMoFsA\/Cr1oecE+Gr+oucE\/KsHw+cP2ueFLlM0pJtD4tb\/66iu677776I033qDvvvtOkahWrRqdd955dNttt1Hjxo1l6fiUHjZYcLxGdLeikYzOMIoF8I9CL1pesI\/GL2pu8I9KMHz+sH1u+BJlc4oKNNlXj1562GAZPH0FDZ6+UlUAtweE8wMayXDcbOUCf1skze2AvTkzmznA3yZNM1th+1yzUtKTGgItgi\/CBgsuR48A\/f9nRSMZnWEUC+AfhV60vGAfjV\/U3OAflWD4\/GH73PAlyuaEQIvAP0ywYHozAnBHVjSSdjiGtQL+YclFzwf20RlGsQD+UehFyxumz41WomxuCLQI\/MMEC47XiAAcAs0OPAtW0ElZgBjSBNiHBGcpG\/hbAhnCTJg+N0QxqckCgRbBFWGCBevPIgCHQLMDz4IVdFIWIIY0AfYhwVnKBv6WQIYwE6bPDVFMarJAoEVwRZhgwfqzCMAh0OzAs2AFnZQFiCFNgH1IcJaygb8lkCHMhOlzQxSTmiwQaBFcYRosWH8WAbYrKxpJeyzDWAL\/MNTs5AF7OxzDWgH\/sOSi5zPtc6OXKGtBVKDxjSqCMEcAACAASURBVAF\/\/\/vf6YsvvvCkULt2berVqxfxzzQ+psGC9Wf2vIhG0h7LMJbAPww1O3nA3g7HsFbAPyy56PlM+9zoJcpaEBNor776qrruSR9Q64WhSZMmFeqydKw\/sxfsaCTtsQxjCfzDULOTB+ztcAxrBfzDkoueDwItOkNfC9u3b6fu3bvT119\/TQ899BAdffTRVKlSJd98aUtgGixYf2bPg2gk7bEMYwn8w1Czkwfs7XAMawX8w5KLns+0z41eoqwFkRG0tWvXUocOHeiaa65Rl6IX62MSLLh\/066X0Uja5WlqDfxNidlLD\/b2WIaxBP5hqNnJY9Ln2ilR1oqIQNu0aZO6KP3CCy\/MrEB7sawNnXZYHVnvF3HpaCRlnQf+cvzBXo49lwz+cvwh0BJiP2bMGHrttdeIfzZo0CChUu0WYxIsWH9mlz0aSbs8Ta2Bvykxe+nB3h7LMJbAPww1O3lM+lw7JcpaERlB27lzJ7311lv0+OOP09KlS+mMM86gmjVr7kOiIu3i\/PWoRTTzk03qHU9tWYde6t1G1vNFXjoaSVkHgr8cf7CXY48RNFn2EGgJ8Ndr0FatWlWwtIq0i9O5QWB4x6PompN+mgDpilsEOilZ34K\/HH+wl2MPgSbLHgJNln9RlW4SLPX6zMy9G9afRXczOqnoDKNYAP8o9KLlBfto\/KLmBv+oBMPnN+lzw5eSnpwiU5zpef1oNQkaLDigNhpnr9xoJO0zNbEI\/ia07KYFe7s8Ta2Bvykxe+mD9rn2SpS1JCrQli9fToMGDaK5c+fSwQcfrA6lPeCAA2jAgAF0wQUX0CWXXJLY+Wh8aO60adNy3hg1ahS1a9euoHeCBgs2CNgPcjSS9pmaWAR\/E1p204K9XZ6m1sDflJi99EH7XHslyloSE2gffPCBOmpjv\/32o+bNm9OaNWuUQKtevbr6\/ZIlS9Qhtn4iyQY+FmcLFizI3VowY8YM6tmzJ\/Xt25fKysryFhE0WHBArQ0vlbeBRtI+UxOL4G9Cy25asLfL09Qa+JsSs5c+aJ9rr0RZSyICbffu3eqaJ94kwMdsLFy4UI2ksUBr2LAhbd26VZ2PxqNpPJLFoi2uZ\/HixdS1a1caMmRIOTHI9Vu9ejWNGzeOSkpKPIsPGiytB80lPqiWn\/7tm1P\/9i3iep3M2EUjKetq8JfjD\/Zy7Llk8JfjH7TPlauh3ZJFBNr69eupY8eOdOWVV6oRKh6xcgo0fsWxY8fSk08+KXYXp02Bhg0CdoMWjaR9nqYW0UmZErOXHuztsQxjCfzDULOTBwLNDseCVvQxGzfccAN16dLFU6BNnDhRja7pUbUEqpUrYsSIETR06FA1eldoilUHy9NPP018JAg\/PALofP65cjtdOmJh7lcLBpxAh9TbP8nXqZBloZGUdSv4y\/EHezn2+HKYPHvWC\/rhWbfOnTvTrFmzcn1u8jVKrkSRETR9WXr9+vVp2LBh6tBarynOqlWr0ujRo+nAAw9MhIhee8aFXXbZZapuhR4t0JxpeLr02muvzf1q9LzNNGb+ZvX\/jWpVoZeu3Svk8EQjsGvXLuKRWBbEVapUiWYMuY0JgL8xMmsZwN4aylCGwD8UttCZnnjiCZowYUK5\/BBooXEGy\/jqq6+qRfg8gtasWTMaOXKkGrFihfzII4+oTQKDBw9W06BJP1pA8hq0QiN4WqDx+rXGjRurarJgcI6iXTX2\/2jOim3qb3yDwN9+V5r061TI8vg2inXr1qlvURBoybsY\/JNnrksEezn2XDL4J8ufR9D0KNq8efNo+PDhGEGL2wV79uxRqpjFzY4dO8oVV7lyZbWJgKdA+b8lHr15oEePHnl3cgaZD3fu4MQGAXuexDSPPZZhLIF\/GGp28oC9HY5hrYB\/WHLR8wXpc6OXkh4LIlOcztfnHZvz589X\/7777jv6+c9\/TqeddhrVrVtXlJItgYYNAvG4EY1kPFyDWgX\/oKTspwN7+0xNLIK\/CS27aSHQ7PJMvTVed9avXz81mteqVatcffV6tEIbBfyCxX2DwKKBp2CDgKWIQCNpCWRIM+AfEpyFbGBvAWIEE+AfAV7ErH59bkTzqcsuOoLG05yffPIJPfvss7Rt2951Wrxx4IorrqAWLZI5K0yvN+Oy9ZlnevSstLQ00jloToHGOzdZoOGxQwCNpB2OYa2Af1hy0fOBfXSGUSyAfxR60fJCoEXjFzg3T23+8Y9\/JN4swELN+VSqVIk6depEd955J1WrVi2wzSgJ3Vc9+d0iwGX5Bcu0RV9TtwkfqmpBoEXxzr550Uja5WlqDfxNidlLD\/b2WIaxBP5hqNnJ49fn2iklPVZERtBYkPHmgMcee4x69+6tjqWoVauWosLCjQ+p5TPQbrnlFrVRIK2PX7Dgiqf4PIdGMj62QSyDfxBK8aQB+3i4BrUK\/kFJ2U\/n1+faL1HWoohA27BhA11zzTV0wgkn0F133bXPhegs4Hj0jO\/rZBEnvWEgn4v8ggVXPMUX3Ggk42MbxDL4B6EUTxqwj4drUKvgH5SU\/XR+fa79EmUtigg0fZMAj57xlU9ez0svvaRG2SRuEgjqkkLBwndvskDTz4tlbei0w+oENY10PgTQSMqGCPjL8Qd7OfZcMvjL8YdAS4A9n8R80003qfVlfFq\/e50ZH7dx++23q+nOhx9+ONbL0qO8bqFgwQ7OKGT986KR9GcUZwrwj5NuYdtgL8ceAk2WPQRaQvz5xgBeX9a0aVO69dZbqXnz5rTffvupE4P5JgE+5uK+++6jY445JlcjPrS2QYMGCdXQv5hCwTJ4+goaPH2lMoINAv4sTVOgkzIlZjc9+NvlaWIN7E1o2U8L\/vaZBrUIgRaUVIR0eoqTRZrJw9f68B1caXkKBcuT89bQzVOWqqryFU8v9W6TlmpXiHqgkZR1I\/jL8Qd7OfYYQZNlD4GWAH++y2zOnDnEU50mT\/Xq1em8884zyRJr2kLBgh2csaLHOpB48fpah0jwRRRbArCPDW0gw+AfCFMsiSDQYsFaMY0WChbs4IzX52gk4+XrZx38\/QjF93ewj49tEMvgH4RSPGkg0OLhmtfqpk2b6O2336b33ntPbRZo06YNnXzyyak9WsP5IvmCBTs44w8iNJLxMy5UAvjL8Qd7OfZcMvjL8YdAS4j9Dz\/8oA6j5V2c\/N\/OhzcD8BEcZWVlid0kEOa18wWLewcnjtgIQ7dwHjSS9pmaWAR\/E1p204K9XZ6m1sDflJi99BBo9lgWtPTMM89Q\/\/796YwzzlA3BvAuTn5WrlxJDzzwAL3zzjvqHLRLL700oRqZFxNEoGEHpznXIDnQSAahFF8a8I+PrZ9lsPcjFO\/fwT9evoWsQ6AlwF5fUF5SUqLOOatRo0a5UnkTAZ+TxiNro0aNKrpz0Ca9u5Z6T1qi3gkCLZ6AQiMZD9egVsE\/KCn76cDePlMTi+BvQstuWgg0uzw9reljNvgctC5dunimmThxopoCLcabBLCDM\/4gQiMZP+NCJYC\/HH+wl2PPJYO\/HH8ItATYr1+\/Xl3xdPnll6uRMq+HR9aef\/55mjJlSqoOp3XWNV+wOAVa\/\/bNqX\/7FglQzVYRaCRl\/Q3+cvzBXo49BJosewi0BPjv3r2b+vTpQ0uWLKGxY8dSixblBcyKFSvo+uuvp9LSUrWJoGrVqgnUyryIfMFSr8\/MnLFHry6lq09oaG4cOQoSQCclGyDgL8cf7OXYQ6DJsodAS4j\/Bx98QN26dVOH1Z5\/\/vl06qmnqpL5ANvXX39drTvjKU4+diOtj1ew4IiNZLyFTioZzvlKAX85\/mAvxx4CTZY9BFqC\/D\/++GN1KfqiRYtoz549quRKlSpR69at6d5776UjjzwywdqYF+UVLLgk3ZxjmBzopMJQs5cH\/O2xNLUE9qbE7KYHf7s8TaxBoJnQspSWR9H4wFp+6tatm9pdm+7X9RNo2MFpKUA8zKCRjI9tEMvgH4RSPGnAPh6uQa2Cf1BS9tNBoNlnWmEtegXL4OkraPD0leqdIdDicz0ayfjYBrEM\/kEoxZMG7OPhGtQq+AclZT8dBJp9phXWolew4IiNZNyNRjIZzvlKAX85\/mAvx55LBn85\/hBocuyLrmQ\/gYYjNuJzKRrJ+NgGsQz+QSjFkwbs4+Ea1Cr4ByVlPx0Emn2mFdaiV7A4j9hYNPAUNc2Jxz4BNJL2mZpYBH8TWnbTgr1dnqbWwN+UmL30EGj2WFZ4S+5gwREbybkcjWRyrL1KAn85\/mAvx55LBn85\/hBocuyLrmR3sOCIjeRciEYyOdYQaLKs3aUj9mX9Af5y\/CHQ5NgXXcmFBBp2cMbrTjSS8fL1sw7+foTi+zvYx8c2iGXwD0IpnjQQaPFwrZBW3cGCIzaSczMayeRYYwRNljVG0MA\/XQTkagOBJse+6Ep2B0vvSUto0rtr1Xuc2rIOvdQ7vddUFR1sV4Uh0GQ9CP5y\/MFejj2XDP5y\/CHQ5NgXXcnuYHGegcYXpPNF6XjiIYBGMh6uQa2Cf1BS9tOBvX2mJhbB34SW3bQQaHZ5Vmhr7mBpPWgu8U5OflicsUjDEw8BNJLxcA1qFfyDkrKfDuztMzWxCP4mtOymhUCzy7NCW3MGy48H1CcWaPp5sawNnXZYnQr9\/pIvh0ZSkj6meSTpI\/Yl6SP2JelDoEnSL7KyIdDkHIZOSo49lwz+cvzBXo49Yl+WPQSaLP+iKt0ZLCv\/XUKXjliYqz9uEYjXleik4uXrZx38\/QjF93ewj49tEMvgH4RSPGkg0OLhWiGtOoNl9poqxLs4+cEZaPG7G41k\/IwLlQD+cvzBXo49RtBk2UOgyfIvqtKdwfLUh7tp8PSVEGgJeRCdVEKg8xQD\/nL8wV6OPQSaLHsINFn+RVW6M1h6vbCe5izfrOqPM9DidyM6qfgZYwRNlnG+0hH7sn4Bfzn+EGhy7Iuu5HwCrX\/75tS\/fYuie59iqjAaSVlvgb8cf7CXY48RNFn2EGiy\/IuqdGewXDz+y9wZaDhiI343opOKnzFG0GQZYwQN\/NNJQK5WEGhy7IuuZB0sE6fNIBZo+oFAi9+VEGjxM4ZAk2UMgQb+6SQgVysINDn2RVcyBJqcyyDQ5NhjmgfsZQnIlo62R44\/BJoc+6IrWQfLfY+\/SLxJQD84Ay1+V6KRjJ8xRtBkGWMEDfzTSUCuVhBocuyLrmQdLL0emEr3zd6Wq\/\/GYWcX3bsUW4Uh0GQ9Bv5y\/MFejj1Gj2XZQ6DJ8i+q0nWwXPTHCcTnoPGDQ2qTcSE6qWQ4YxRHlrNX6Yh9WZ+Avxx\/CDQ59kVXsg6WY3uNodlrKqv64wy0ZNyIRjIZzhBospwh0MA\/fQTkagSBJse+6ErWwVKn03Diuzgh0JJzIQRacqwhEmRZu0tH7Mv6A\/zl+EOgybEvupK9BNqzPVrR2UfWK7p3KbYKo5GU9Rj4y\/EHezn2XDL4y\/GHQJNjX3Ql62DZ2m4w\/XhAfVV\/3CKQjBvRSCbDGVOcspwxegn+6SMgVyMINDn2RVcyB0un7r8nFmj6wSG1ybgRAi0ZzhBospwh0MA\/fQTkagSBJse+6EqGQJNzGQSaHHtM84C9LAHZ0tH2yPGHQJNjX3Qlewk0HFKbjBvRSCbDGSNospwxggb+6SMgVyMINDn2RVcyB8uVAx6lHcd3y9Udh9Qm40YItGQ4Q6DJcoZAA\/\/0EZCrEQSaHPuiK5mD5Yq7p9C\/j7pU1R2H1CbnQgi05FhDJMiydpeO2Jf1B\/jL8YdAk2NfdCW7BRoOqU3OhWgkk2MNgSbLGgIN\/NNFQK42EGhy7IuuZA6WS0csou\/rH6nqDoGWnAsh0JJjDYEmyxoCDfzTRUCuNhBocuyLrmS3QJtw3TF08XENiu49irHCEGiyXgN\/Of5gL8eeSwZ\/Of4QaHLsi65kDpaLx3+JQ2oFPIdGUgC6o0jwl+MP9nLsIdBk2UOgyfIvqtI5WC6cujNX50evLqWrT2hYVO9QrJVFJyXrOfCX4w\/2cuwh0GTZQ6DJ8i+q0t0CDbcIJOc+dFLJsfYqCfzl+IO9HHsINFn2EGiy\/IuqdLdAwyG1ybkPnVRyrCHQZFm7S0fsy\/oD\/OX4Q6DJsS+6kt0CDYfUJudCNJLJsYZAk2UNgQb+6SIgVxsINDn2RVeyU6DhkNpk3QeBlixviARZ3s7SEfuyvgB\/Of4QaHLsI5e8fft26t69O82fPz9nq2\/fvlRWVlbQ9uLFi6lr1660bdu2culOPPFEGjduHJWUlHjmh0CL7LLQBtBIhkZnJSP4W8EYygjYh8JmLRP4W0NpbAgCzRhZOjKsXbuWOnToQI0aNcqJKi28zj33XBo2bFjeis6YMYP69etHEyZMoFatWgV+IadAwyG1gbFZSYhG0grG0EbAPzS6yBnBPjLCSAbAPxK+SJkh0CLhk8ucT2SNGDGCJk+eTFOnTqWGDb2PwOA0s2bNKjha5vVmEGhy\/kYjKceeSwZ\/Of5gL8cesS\/LHgJNlr\/10ll8jR49uuDoWJ8+fVS5hUbZ\/ARa\/\/bNqX\/7FtbrD4PeBNBJyUYG+MvxB3s59hBosuwh0GT5Wy+dxdeCBQvyjqDpqVFeZ7Z06dJc+ZdddpmvYHOOoEGgWXddQYPopJLl7S4N\/OX4g70cewg0WfYQaLL8rZbO0549e\/akQhsFvNapea1n8xtBu+30muoWgXzTqFZfDMYwxSYcAxAJcg4Aezn2EGjJs+f+WD+rVq2izp07qyVJTZo0Sb4yCZdYac+ePXsSLjOR4rTwKi0tNV5bxhXU4m7UqFHUrl07zzo7R9BK3r6fqmz4WO0GvfbaaxN5xywXsmvXLlq\/fr0SxFWqVMkyCpF3B38R7KpQsJdjD\/7Js3\/iiSfUEiXnA4GWvB+slRhVnHFFtI0ePXrkPabDKdBG\/qoBNau+XQkGjKJZc2VeQzt37qR169apb1EQaPHzdpcA\/skz1yWCvRx7Lhn8k+XPI2h6FG3evHk0fPhwjKAl6wJ7pemRL78zzPxKNBVouObJj6jdv69cuZLGjx+vzr3LwlC3XXrRrYF\/dIZhLYB9WHJ28oG\/HY5hrGANWhhqKcmjxVmQBf66yvmmMoOcjeYcQcM1T8kGQdY+qMnS9S8N\/P0ZxZUC7OMiG8wu+AfjFEeqrLGvMGvQgh5K6w4affvA6tWrczs9g9rSAg3XPMXxUSxsM2sf1OQJg3\/amOv6IPZlPQP+cvyzxr7CCDQ+TmPatGl5I0cv9s93Lpo7f5ArorRAa77\/dnqxrI1c1GawZL2b5+mnn8YUp4D\/wV8A+v8vEuzl2HPJ4C\/HH7s45dgXXckcLMcN+1Tt3uRdnHhAAARAAARAAATiI3DSSSfRpEmT4isgRZYrzAiaFFMWafwPDwiAAAiAAAiAQLwEeFNYVjaGQaDFG0uwDgIgAAIgAAIgAALGBCDQjJEhAwiAAAiAAAiAAAjESwACLV6+sA4CIAACIAACIAACxgQg0IyRIQMIgAAIgAAIgAAIxEsAAi1evrAOAiAAAiAAAiAAAsYEINCMkSEDCIAACIAACIAACMRLAAItXr6wDgIgAAIgAAIgAALGBCDQjJEhAwiAAAiAAAiAAAjESwACLV6+sA4CIAACIAACIAACxgQg0IyREekL1ufPn69y86nGU6dOpYYNG4awhiz5CDjvR61ZsyZNmDCBWrVqVRDYjBkzqGfPnrk08E24+ArD3lnS2rVrqUOHDtSpUycqKysLV4kM5wrD390uMT59B3GGURq\/ehj2Ot71rTJod4yxG2VgH\/EzbNgwo3zFlhgCzdBjuhFs1KhRLjg4WBYsWACRZsiyUHJmunr1aho3bhyVlJRQvkvunTY4zdChQ8t1SmznzTffDCTuLFa\/qE2FYe9+Yd3J9e3bFwLNMBrC8NcCgdsl52fG\/XkwrErmkkdh37ZtW\/QJCUSMbucvu+wyCLQEeBdVEV5CAaMFdl2oR8Gc3\/69hLGz1Hx\/h2\/MfBOGvbsE5ygmBFoy\/Lldmjx5crkviX6fGbOaVfzUYWPfq09YvHgxde3alXr06IEvKJZCxz1CDIFmCWxFMuP+hqXfLd\/vK9K7J\/UuXp0Nl53v94XqpQWa89ttUu9RjOVEZa95d+\/eXY3kYIrTLArC8Ncd1xlnnAExYIa7XOow7HW7NHr06HKj9BBoERzhkVXHOM+qjB8\/nm6\/\/XZyzmLZLS091jDFaeCLQt9IMc1pANInaT6xG2Sa020aDaWZX6Kwd34+br31VqxBM0OvUofhr0XxwIEDadmyZWqan5+g6zZDVLNCZgnDnkF4fQnE0or4QiRLI8MQaAZxVCgwwozuGBSdqaT5GkqegujXr1\/g9WTOb13YxBEshKKwd34GuDRsEgjG3JkqDH\/9JWTbtm3knPbxWpNpXqPs5AjD3u27adOmqV+deOKJubWA2SGYzJtCoCXDuehKgUBLxmVRG0pdS71QHTvZgvstLHstEoYMGULt2rXLjSpgijM4+0IjaIW+nGj2paWl5USBbq\/Yrt44YFabbKUOG\/t67ZpzvSXEcXyxA4EWH9uitowpzmTcF3aqwevbLMSZmc\/CsPf6XGBzhhl355cK5+5l\/ftC0\/taoJ177rn77GrDyH5wP0SJfS8RjHXJwdmbpIRAM6GVsbTYJBC\/w8Mu1uWaOXf6QJyZ+yoMe+cUm1eJOBMquB\/C8C+0EQYCLV72mFUJztdWSgg0WyQroB2vBg+jBXYd7TWdE+RDqdMsWbIk8Do1uzUvfmth2bvfHJ+JcLEQlr\/XJqUgn5lwtayYucKwLzSNjBG0eOIkS3GNTQKGMeR1ICR2cBpC9Enutbg\/yA5O7JyK7oew7CHQorN3jgDzNKfe2BIk9r2mOYPks1PrimElbOxjDVqy\/odAS5Z30ZXmPjAPUzjxuFAv8mfrXkcGOIXxunXr1MGQvJPN68GuKjMfmbD3uuIMI2hmvN2pw\/B3XzeEYzbC+SAMe\/c0P9iHYx8kFwRaEEpIAwIgAAIgAAIgAAIgEAsBTHHGghVGQQAEQAAEQAAEQCA8AQi08OyQEwRAAARAAARAAARiIQCBFgtWGAUBEAABEAABEACB8AQg0MKzQ04QAAEQAAEQAAEQiIUABFosWGEUBEAABEAABEAABMITgEALzw45QQAEQAAEQAAEQCAWAhBosWCFURAAARAAARAAARAITwACLTw75AQBEAABEAABEACBWAhAoMWCFUZBQI7ADz\/8QC+88ALt3LmTfvOb3xhX5KuvvqJx48ZRWVkZNWjQwDh\/lAxvvPEG\/fnPfya+6qhx48Y0efJk9dP0KXSBuKmtNKR33xLQt29f5R+Jx31q\/qhRo6hdu3YSVUGZIFChCUCgVWj34uWySCDqNUt8hyMLI30XZFIMt27dStdff70SZz179qSf\/OQndOqpp1KNGjWMq1BRBdpPf\/pTdaXZ4YcfTkcccYQxFxsZNm3aRPPmzaN\/\/etfSshDoNmgChsgsC8BCDREBQhUMALFKtBsiiqbttIQHml8H31JOARaGiIEdaiIBCDQKqJX8U4VlsCePXvopZdeogceeIBWrVpFVatWpdatW9Ndd91FRx55JBW6tJkvGX7kkUdo2rRp9PXXX1OlSpWoSZMmdMstt9All1yi\/t95UTRDvOyyy2jYsGGK58cff0x\/+tOf1MgJP8cddxzdeuutdNJJJ\/ny5mnThx9+mF5++WXasWMHNWrUiH73u9+pKdhq1aqR7uydhgpN4zGH1157jf7617\/SZ599pjiccsopNHDgQGrZsiVpQdOmTRs68cQT6aGHHqL169ercrnO+n25vO+++46eeuopGj9+vGLKtg8++GDq1q0bXXfddap+\/OiRxd\/+9req3B9\/\/JHuvvtuuuKKK2j58uXqv9955x2qXLkyXX311XTssccqv0yYMIFatWqlbPC0M5fz2GOP0TfffEMHHXQQ\/frXv6Ybb7yRSkpK8nL0EmjOS6PPOussuueee2jDhg0qDm6\/\/XY1+sg+zffkGykNOoIKgeYb9kgAApEIQKBFwofMIJAsgVdeeUUJqjPPPJPat29Pmzdvpv\/5n\/9RHfHTTz9NNWvWVGJn6NCh9Itf\/IJ++ctfKgF14IEHKvH11ltv0ZVXXkknnHACrVixgiZNmqSEwujRo5XN9957TwkKFhp\/+MMf6KijjqKf\/\/znyibnb968OXXp0kW99MSJE2nZsmVKwF144YV5QbDoYaHDAol\/NmvWTInEOXPmUIcOHZSIYWExc+ZMevDBB+nQQw8tOI3HAopHbfgdS0tLVX1YwHB96tWrR48\/\/rgSSWybf8\/Tgtdeey1Vr15dCSOuBzNjBmxryJAhNHbsWCXazjjjDNqyZYuyxcLvzjvvVHXRAo2Fcd26demGG25QZZxzzjlKHPJ7sS+4LrVq1VIijMUYiz8t0FhQsRBbsGBBzgfvvvsuPfPMM0pkszDivF5PIYG2ZMkSVYerrrpKseXyuO4sSgutDYNAS\/azi9JAwJQABJopMaQHAUECLJJ4NIoFhe7M3377bSWeBg0apDpkrylOHv3iReU8WuNcXM4CgUeyeM2X\/r274+a1YSxAGjZsqEbu9JowFh9cLq8ZY0HiJS5YALHI4U0LLI5Y7PHDGxlYYPHIlRZLQafxvvjiC+rcubMSNSwO9QjXP\/\/5T7rpppvotttuU+KUBRqPcj355JPUokULVa5+X17rxmlZGPbo0UOt5+IRMBY6\/OgyePRNjyAyF67zfffdRx07dlTp9PvxaN6YMWOIR+z4cYpSLdBYlPIIJNs57bTTclHE9WbBx+XziKWpQFu0aFE5kcxrxHj0j0V7Pr9owem11hAjaIIfcBQNAg4CEGgIBxAoIgIskFicscDgkR2vXZYma9B02osuuoj69++vSLg76IULF6oRqIsvvlhNmzkfHlnjUTnnNJ7z7yyArrnmGiWQhg8fnhNAnIZHfvgdeESPyw4qmB6V6gAABh9JREFU0HinJ4tJnq7NN0LknOLkUTk91Re0DD19yCNyut7MhUcane9a6P14SpcX0XN6ng7mEUkeoeRRNB7p1A+PvHEdTz\/99JwYdIdkoRE0TsvlOKdIua4cJyxOjznmGM8IxwhaEX3wUdVMEoBAy6Tb8dLFSoBHz3RHz+\/Aa6V4GpOFjh4lKiTQdu3apaa\/WBzNnz+fePSNR8Cca83cHbfX+jAnPxY\/LFzOO++8fbB6CUCdyC06goonFh4szvKJQrafz1a+3\/NoILP96KOPlIiaPXu24sTToFr8eAk0zsOjafyPR+Tc4rVfv36qnrwurnv37op5vodH\/ZgjT0ebCDReV6dH+XS+IOvDINCKtRVAvbNCAAItK57Ge1YYAjytxuJhypQpNH369NxmAb0WzEugsQDh6Tlen8XTizwdyeeL8eL1119\/vdzoTT6BFma3XhwCzUsoBRE0XsKNWbKA4nVovHnhgAMOUGvW2rZtq8QUj1AWEmgrV65UGx06deoUSKB5jXYFCUy\/TQJeAq1Xr155hTOXCYEWhDzSgIAcAQg0OfYoGQSsEPjkk0\/UGjJeIM4iitcg8forFg16Xdn\/\/u\/\/EnfYvDapd+\/euekwLTB4I4FzrZVzbRILFV6DxtOceho0aMWDTHHy6B+PPgUdQcs3xcmL\/3k69fzzz1eiiRmw0HKKF3cZepqV3\/8vf\/mL2gDAj17HxRsLCgm0Qu\/nFJI\/+9nP6Oabb1a7bHndHW+2MHkKCTReE8h+57rqh8tmMc5l5TsvLV8a3uXK6+L8zsELMkpn8o5ICwIgUJ4ABBoiAgSKhMC2bdvov\/7rv9QIGHeuerH+xo0blXiqX79+XoHG6bkTf+KJJ3IL2Xn0iDthPpLhV7\/6VV6BpjcJ8Fopzt+0aVNFjEflOC8fu8EL\/fUUqxOn3yYBFhG8eYCn94IKtHybBF588UV1hAbvXuQ1X0EEmhYZvF6M1+Hph6d+WdyysCok0PJtEtACj0c6nZsEeAcuT1HzOjS9Lo6nVHmjAh\/NwX\/3evx2cTo3YPARKhwPPP3NPs930C\/vVGVROnLkSLUblZ8vv\/xS5f3+++8h0IqkXUA1Ky4BCLSK61u8WQUkwDsFeaqSd0OyqOLn+eefVyMz+lgFParDf+NRM158\/v777ysRwNOaPIrGnTbvrOTRMd7pyOJEjzTpjps7aj7Kg8t69dVX1Y5NXojOdniRO+dnccGikUfw8p25FeSYDd6JGVSgOY\/Z4JEv3mTw6aefqgXxxx9\/vBKvvMg\/iEDjETR+T2bA78VTmrNmzVLHiuzevVuJ2UICjRnzKCSP3PGxGnzMBttgwcTvvf\/+++cEGgtdFn1z585VgoiZ85q35557jurUqaN2XPK5dKYCjX3ImxmcZXNZzl2lWog6z5bT7847V3knLz8swLkuLDD1CJr2C691c25GwAhaBWxg8EqpIgCBlip3oDIgUJgAj57x9CN3viwA+HEfTMoChsUKC7lvv\/1WrUM699xz1QG3999\/v9oUwGut+NwzFnC8M5TP\/tJHMvDCdxZjfF4XH\/6qF67z9T6cn8UeCxoWEzxtx0KRzwQr9HgdVMvTmpdffnnumIygAo3LcR9Uy4KTBZkWkSabBPjMNz6ihI8i0Qf\/8uJ+HpF788031flyhxxyiBJ+7l2c+p35oFo9mqgPquU68RSjczODPiz42WefVefPsR\/OPvtsGjBgQME7RwuNoDEL9iWPRvIoJ4tUPrKD40I\/XgKN\/+Z8dxZ5vDuY\/frf\/\/3fEGhojEBAmAAEmrADUDwIgEDFJMDTpryRg\/+FufDdScX0mI0kiGIELQnKKCPLBCDQsux9vDsIgEAkAnxsCa8n22+\/\/dRIpD6LTF\/8ztO+7jPKwhQIgRaGGvKAQHETgEArbv+h9iAAAsIEeAqTp0hPPvlkdZ4cb54Ieg1W0KprgcZHgPCu18MPP1zdK8pnq\/FjQwQGrQuvT+Ppbt4cwuWGOX4laFlIBwJZJgCBlmXv491BAAQiE+B1gbxhgm8c4HWBPJrGu0h5HRtvYih0YXnQwrVA0+sOebE\/CzUJgcYbUrhs3lXMDwRaUC8iHQiYEfh\/VqTZ64byyRMAAAAASUVORK5CYII=","height":0,"width":0}}
%---
