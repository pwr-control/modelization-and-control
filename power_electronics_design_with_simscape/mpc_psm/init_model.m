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
model = 'mpc_psm';

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

% use_current_controller_from_simulink_module_1 = 0;
% use_current_controller_from_ccaller_module_1 = 1;
% use_current_controller_from_simulink_module_2 = 1;
% use_current_controller_from_ccaller_module_2 = 0;

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
fPWM_AFE = 6*2.5e3;
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
p2place = exp([-50 -250]*ts_inv);
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
ki_w = 14;
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
emf_fb_p = 2;
emf_p = emf_fb_p*4/10;

emf_fb_p_ccaller_1 = emf_fb_p;
emf_p_ccaller_1 = emf_fb_p_ccaller_1*4/10;

emf_fb_p_ccaller_2 = emf_fb_p;
emf_p_ccaller_2 = emf_fb_p_ccaller_2*4/10;
%[text] #### Speed obserfer filter LPF 10Hz
fcut_10Hz_flt = 10;
omega_flt_g0 = fcut_10Hz_flt * ts_inv * 2*pi;
omega_flt_g1 = 1 - omega_flt_g0;
%[text] #### Motor Voltage to Udc Scaling
motorc_m_scale = 2/3*Vdc_bez/ubez;
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
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.019856330123859"}}
%---
%[output:69e958f5]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.825051459217296"}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAW4AAADcCAYAAABQ10tFAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ1wV9WVP7pMNS1lNUjVFEOwBrXrlvrVSVNdtGxl60xwZ2dbSHZnaWAcdor0KwwhsTMdpkKAkY4f2G2qKYPtEOju6gq73VKburaWoq0f+IViDSnGqIUgBWrsrNvsnsfeeHPzPu67\/\/Puu+++82Y6xfzvPfec37nn98477757TxkdHR0FvhgBRoARYAQKg8ApTNyF8RUryggwAoxAgAATN08ERoARYAQKhgATd8EcxuoyAowAI8DE7fEceOmll6C1tRWampqgvb2d3NIjR47AkiVLoLa2FtatWwdVVVXkY8gC169fDzt37oTNmzdDfX19pmOpwkdGRmDVqlVQU1OTCZZZGrNnzx5oaWkJhli6dGkq\/fPEXMVEzLeFCxfCggULsoTMedlM3M67yFzBPIgbx9y1axfcdNNN5opH9FRJJMuxVBUE+W3duhUaGhoSbUur2\/bt22HGjBlashMHlxqIG87Bgwehp6cHqqur03QHl4gbFUd90BcmtqQy3PHGTNyOO6hI6mV9o5BJBHHJ8mlCxT0NYaTFAUm7o6MDdG8KaeaEb8Sd9gaaBqsitWXiNvQWBnJ3d3fQGx+f5cd3QTDXXHNNEIx4dXV1jXu8k\/tjKUOUGkTQY9\/jx48HpQFVvqqyCHzxd0EAKoGIdvi4jLqLx2bRbmhoaNzjtFoKwR+xXCCyN\/xvUSpZuXJlkGXv3bs3kDF79uxxWZEgEPxNtVUu5ejgevvtt8Mtt9wyYSyhT5gOYnzEEy+BgewXuaQgYy5wwExblJzE39SxonSI+vv+\/fvHyhhCLxwjSpew6arqIuaT8JewGf877OYQ1R9LX2IuT5s2bQxvGTMVVxk3Ma8uv\/zyYM7ghZmybPOcOXOCvx89enRsvqjzUdY57U3RMLyd78bEbeAi9fFRzZgE+YgJHva7qNVOnTo1ID9BCmJiYqDgJB8eHo7NLONko2lyVioTtyAgNRAEYaDun\/rUp8bVsOOIG8l4cHAwla6oz5133jl209PBVeCm2qbeGFRdZJzwpoI3IJQlfCTbjfVTOcMWPli+fPnYzTfsd3EDUjFNoxvOgzhd1FJH0s0VyVe+2Sb1V3FT57LqIxkH+UYuzwcxl3FsVV+88WH9Xdzo1fmuzhHb71UM6MFKFybulDDHZV+CfMNqsYJAb7zxxgkv9OJIIG6iJj0GR2XccgYT95ieRApRgRr1MlTW5wtf+EJAKCIDR1vkGxj+XcVap1SiZo+YWYux5DpvGDnKLz7lR3LUBclFzjTlJwM1i43KCsN0wxto3M0XX8LGlQfCfpP\/Jm5SUTVuFQc1HJJupthezbpFxh92I1fHU+fwgw8+OK5sJLAUN82kOZ8ynAvbnIk7pevCglIluDvuuGPc6gf5d7WkIIYXj5hqJhlH3EnZhw5xx718oiZutE3cpJCwVqxYASIg0+IalXGLLPqyyy4by\/7DbpZhxC1KX\/KUQLLGl4YqcauP89hHEHtUxh2mWxRxR+mirqYIu\/HKts2fPz82406qrycRt7iB4Q1SxTmMuNXxoohbDUtR1mPiPokME3dK4s4i45ZVUIPep4wb7RSkcvXVV8PLL788ViZJi6tK3CpuYdl9moxb9klSVirIKOrmG6ebTsYdN0XzzLhnzZo17ulRPDWJ5aEUGbdqOxM3E3dKyn63eZosJCkwRY07arInZdVqBiPXBFVyC8uuokgFM2E1WxP1R1GzVEslYeUOFWS5XKCuKdbBNendAL4Iw\/oqPvXIL2BNatxqPT3ucT2s1qu+t4jSTSXfpDKOjGnSU1HaGrfqwzifCOJGffB9jChzxJVKTGrc8oqbpHgwDuqCdeSM29BhOqsfsGb79a9\/PRghblWJvAIjTcYtVE+7qiSqJquuKpEzZPw3lgtwpUvYqhKxUkTgErcSRrQJW+Ggg6tYwaOO9cQTTwT1UbzEaoUpU6YERI6XeCGJugnfRK0qwfZCv7CngbjVFNg3jW6CLPFFnSA98dJO+DhuqWDcqhCdDFVnVYnAXL35y6tfcB5jAiLmR9SLdbmPOqfwBaZahpJ9xKtKHMi4cXLjFfZVn+o8dXmZId9a6RZXN7aiAA+SiEDa9cByRp32I5ZEZUrcIGyZaBwcaf3mK7S5ZdzCAVGf4OLvvb29Vj6lpnYuEzc1opXLC1uiKS9FTBoB5yO+TM3jc\/sk3Yr0u7rcFXVXVxPF2cM30Bwzbnw0W716daBB1N4P+Eg2MDCQal8FVyYwE7crnnhXD7UcIJdCdLQt8l4lOvbZbKOW9uQP0OL0ED7kvUpyWlWCxFZXVxcQc1SpRK51pg0ym5OQx2IEGAFGwDYC1ksl+Ki0ZcsWuPnmm4M3\/2HELbKbxsbG4Es1fky1PS14PEaAEXAZAavEjYS8Zs0aWLRoUbAtZ9zLSRk0lcjl384\/\/3yX8WXdGAFGwAME+vr6YObMmc5YYpW4w742QySS9ggWxN3c3Dxh20sk7v7+fmcApVbEd\/sQL99t9N0+9iF11CfLs0rcqjpRGTe+hGhra4POzs4gM8dSCbYN24PX96Dw3T4O+uQgLUIL3+epa\/Y5Q9wqWcvZedzHHK4BSh1kBw4ccOoRjdo+lOe7jb7bVwYfusYzuRI3BQm4BiiFTbIMDnpqRO3LYx\/ax5x6RNd4homb2sPE8jjoiQHNQRz7MAfQiYdk4vYcUGLzvC8jlOExm4mbOirsy2PiJsbcNUCJzWPipgY0B3lM3DmATjykazzDpRJiB1OL46CnRtS+PPahfcypR2TiJkbUNUCJzeOMmxrQHOQxcecAOvGQrvEMZ9zEDqYWx0FPjah9eexD+5hTj8jETYyoa4ASm8cZNzWgOchj4s4BdOIhXeMZzriJHUwtjoOeGlH78tiH9jGnHpGJmxhR1wAlNo8zbmpAc5DHxJ0D6MRDusYznHETO5haHAc9NaL25bEP7WNOPSITNzGirgFKbB5n3NSA5iCPiTsH0ImHdI1nOOMmdjC1OA56akTty2Mf2secesTCEnfYmX1J4ODJ7Pfff39Ss4p+dw3QiowJ6cxBT42ofXnsQ\/uYU4\/oGs9oZ9zqtqtJwOC2rGvXrg1Oxc7ycg1Qals56KkRtS+PfWgfc+oRXeMZbeKmBoJKnmuAUtkl5HDQUyNqXx770D7m1CO6xjPaxC1KJbW1tbBu3TqoqqqixsZInmuAGhkR04mDnhpR+\/LYh\/Yxpx7RNZ7RJm4EYvv27dDR0TGGSVdXV3AKe56Xa4BSY8FBT42ofXnsQ\/uYU4\/oGs+kIm4ZDDwDsru7O\/gTvoQMOw+SGrwwea4BSm0zBz01ovblsQ\/tY049oms8Y0zcAhh1tUnSie2+A0ptHwc9NaL25bEP7WNOPaJ3xC0DJFaSbNy4Eaqrq6mxC5XnGqDURnPQUyNqXx770D7m1CO6xjPkGXdTU5PVl5euAUo9YTjoqRG1L499aB9z6hFd4xkj4sbMurW1FYaGhgJ8ampqgvXa9fX11HglynMN0ESFUzbgoE8JmIPN2YcOOiWFSr2\/fB2W9e6DI9+4NkWvbJtqE3fYl5Nbt26FhoaGbDVMkM7EnSv8JIP7Tmy+24eTwGcbvSBuJOr29naSgKUQwsRNgWK+MnwOet9JTcwcn31YeOJua2uDzs5OrZIIf\/JOQ4Y+B0QZgp6JmyYO8pRSeOJesmQJ7N27VxtD3mRKG6rIhkzclWOYtwT2Yd4eqGz89bsGYP2uA8WscVdmena9uVSSHba2JPtObL7b5\/tTBRN3BkzAxJ0BqJZF+k5svtvHxG05YABAe1WJfdX0RmTi1sPJ5Va+E5vv9jFx248uJm77mKcakYM+FVxONmYfOukWbaW4VKINlX5Dzrj1sXK1pe\/E5rt9vmfc+PENriwp5Ac4eQY97kSIV9j6cSbuPD1DM7bvxOa7fUzcNHGQRorzpZI9e\/ZAS0sLRO06yMSdxt1utvWd2Hy3z3fi\/uL2F+C7j77mR8Y9MjICq1atgp07dwJuLLV48WK47bbbgHJnQPzMfvXq1QHb4H4onHG7SbyVauU7sflun+\/EPf+uJ+GRl48Wn7gFaTc2NsKMGTOgt7c32BFwx44dsHv3brLdAbFEUldXBwMDA1wqqZQdHe7vO7H5bp\/vxP3RW34BB4+8XXzilk98Hx4eHiPuwcHB4GR3iqwbP5nfsmUL3HzzzXDHHXfEEjf+2NfX5zA1mauGmE6fPt1cQAF6+m6j7\/bhFPPVxmubFsCx606+Y\/Pi5SRmw7it6w033AAPPPBAUCpZtmxZUDapdBMqzOjXrFkDixYtCvZF4ZeTMwtAv+Yq+p6R+m6fzxk3ZtqYcXtD3GiIeHEoQpbq8GB1v28hP+wFJb+cNCdMV3r6Tmy+2+czcYs13F4Rt63A54ybM25bcy2LcZi4s0DVjkzxYvLUtw7D4W99xs6gGqM4vxwQbWDiZuLWmMvONmHidtY1sYrJZZJJh1+E3977j84YYkTcYafhqBbZOs6MSyXOzCVjRXwnNt\/t87VU0nzP07Dr+eFgXk9+ZAMcfOw\/jec4dUcj4hZZMC7VW7BgwZhO27dvD5bu4ctJ\/DcuDbz99tupdR4nj4k7U3itCPed2Hy3z0filrPt2urT4dh3\/g76+\/utxIPOIEbELS8HlA8IFqfe4HJAXCaISwPxEOEsLybuLNG1I9t3YvPdPt+IWyXtHZ+\/FK654sPFJ26RcXd3d4M4MFj9NJ0zbhrS46CnwTFPKezDPNFPN\/Yjvz4K87\/55FgnJO2rLjgDXEsQjTJuYZW8bE+uacuZd3V1dTrkUrZ2DdCU6ic256BPhMj5BuxD510UfBm5\/VevQ9cPD0wgbfyDazxTEXG74A7XAKXGhIOeGlH78tiH9jFPM6JcGsF+WNN+6qsfHyfCNZ5h4k7j4RzactDnADrxkOxDYkAJxCFZv\/DG72Hh3U+Pk4akjeUR\/H\/58oa41a8mhZF4sntPTw9kXSIR47kGKMGcGieCg54aUfvy2If2MY8aEQn7pt59wW5\/8hVF2K7yjFHGLdZxL1++HH7yk58Ee4rgRki4zSvuGCgvEczaZUzcWSOcvXzfic13+3CGuGxjFFmLskhYhq3Oetd4xpi429raoLOzE+67775g61Uka5svJV29E1LTnMsBQWWr7zb6bp9rxB2UQV7\/PWx66OCEzFqQde2Zp8Om5osnlESi5rQXxC3248aVJHPmzAlOqLnnnnuCXQIPHjzIpRIqRnM8k6Ey03di892+vIkbifqZV09A909fCSVqMU+TyiFx89kL4hYGbtq0CebNmxd8bIPkjVu64oEKVVVVVDGdKMc1QBMVTtmAgz4lYA42Zx\/SOQVJGq+wOrU6ChL1Jz50BjRfeW6wFruSyzWeMSqVVAIAdV\/XAKW2j4OeGlH78tiHZpgjSQ8Mj8CtPxqIzaTljJqKqL2scZu5IZteTNzZ4GpTqu\/E5rt9lZZKkKBHAWDDrgPwypG3tUkax\/3iJ2th7kVTtWvVpvPaNZ4xyrh19irh5YCmU2R8Pw56GhzzlMI+PIk+EvSzr56Ab\/30FTj45tvBf+tcYk1100emwY1XTc+cpMN0KjRxR51MIxtqu87tGqA6EzFNGw76NGi52bZMPkQyfmrwONzzs8HAGep66SQPibr0ynkz4ZT\/b6x+DJMkI4vfXeMZ0ow7C8CSZLoGaJK+aX8vU9CnxaYo7X3yIW7ChER69yODsPeV46kyZ+EvQcRYj26f9+4hIS4QdNScco1njIjbpYBxDVBqbHwK+ihsfLexSPaJ8sW9e4bgsQO\/MyJm9LMgYVwv3f5XM+G8M0\/PpcRBFY+u8Yw2ceuceoMg8SfvVFPlpJwiBb2p5b7b6Ip9gpSHjv4BvvfoUFBjTlNrVv0ryPkv6s+E+RecCrNm1gbkjHJdzp5N5mlhidvEWBt9XAOU2mZXgp7aLlme7zbasE+Q8vOvnYAfPjcM\/YfeqoiU1ax5+Sdr4cKz3zfmNpWYbdiY5RxMku0az2hn3EmG5fW7a4BS4+B7QJThqaJSHwpS\/tVvjsFDLx6B3wyPVEzKgpixlHFe9emw7JrzYPJpk4Lpa5ItV2ojddxQy3ONZyoibjzlpqOjYwyjrq4uqxtM4cCuAUo9YXwPiDITtyDk136HpYvXyAhZJl98AXjdh6fCpedNicyWKeas7\/PUNZ4xJu7169cDbu0qtnAVNfCGhobgsGBbl2uAUtvte0D4RtyCjPH\/\/zg6Cv\/8+BvwwqtvwqGRk+uYKS7MiDFTrp1aBddfchZcUjPZOFOm0Mc3H4Zh4hrPGBE3f4BDNd2T5TBxJ2OUdQuVcH\/w7GH4wbOHAD\/3q+Tlnqq3vBLj6vozYcEV5+ROyLrY+j5PvSBudCZn3LpTurJ2vgdE3tmaIOXnhk7Avz9zKPjkmpKM0b6aKZNg0qRJQZb8kenvh8WNH4RJfyI+LzGrKVc2q+h7+z5PvSFudD3XuOkDQJXoe0BQE7ecHT87hCssDsPAYZqXebJv5OwYX+6dP+298JnLzg6tI7MPs4+TrEfwirizBktHvmuA6uicpg0HPYD4Wg9J+fnXTwRf7GWRGaNfZEK+6Jz3wd831MAZVearLahvTGnmjs22vs9T13jGqMZtc0IkjeUaoEn6pv3dt4AQH2eIzPjNt\/4buvv2w+CJUwNoqMsUKhnPPKsKmmZPg1kfGL8mOcuPRnzzYdgc9t1G13iGiTstk1puX4SAkMsT7\/xxFP7liTfg578+CqOjo5kQ8YTM+Nz3wd9edjacM+W0wDsm65CzdGsRfFip\/b7b6AVxi6V\/tbW11k+8USeYa4BWGgCu1LgFGeOSth17D8GLb\/w+s\/KESsRYM\/6zmsnBUrdTTzn5Es81Mk7jZ99JrQzlINd4xjjjDtvilT\/ASRPOem2pgl4Q8ShgRvxbePjFI5mVJoRlcr344nMnw2evOBumTX7POCJGvf7nd6\/BzJnv7hKnh0xxWlH50GWLfbfRG+IOm0S4ymTbtm18WDBhhIUFhPyRR80Zp8HWx14LdnLLqkYcRsSYFd8w+wOAL\/DkyyQz9j3ofbePM27CgNcURZpx46nvmzdvhvr6+sjh5Uw9bidBNaOPauvanVAT9wnN5Dox\/vv7j78eLGMbefttePxVmi\/u1EHljBhf2l17YTWcNfk9766sqD7d1JxU\/XwnNt\/tY+JONd1JGhsRt6hxowbik3cdbUZGRmDNmjWwaNGigNwxQ9+9e3donRw\/p+\/t7U2soReFuMWStu8+OgSP9pvvcxyHs0zE+Dn0wivPCT76UDNmHV\/ZbOM7sfluHxO3zWg5OZYRcVOpiVn12rVrYePGjaCeUYmkPjAwkLjviWvEjQT99KvH4YfPHk59bFNYRvzOO+\/A+dMmBx94NH7oDGiY+afOE3Ha+eE7sfluHxN32hlfeftciTsu48ZP6ru7u8cs3Lp1K+AGVuqVN3G\/fOgt+PL3X0xF0lF7UqBtZdvnuAxBz8RdOVHlLSFvnlHtz4W45dN0wggZSyqrVq2CxsbGYJtYLJusWLEitH6OgOLV19dnxbdDx96Bbz92FHbuOxE7Hu5Pce77J8En6qrgkrNPC\/6NF\/49zTU4OAjTp09P06VwbX230Xf7cML5auPcuXPH4qm\/v9+Z2MqFuIX1gsBxG9iwbFq0U4lcRs\/WnRBfGM7\/5pOhW3OKbTY3NV8cqGaysiJqRnC25kysGCvCPjSGzpmOtnhG12Aj4qba1jWOkGUDRLvm5uYJBJ8loGKlx0dv+cUEPJGcNy28GK664AxdrI3acdAbweZUJ\/ahU+4wUiZLnjFRKBVxh310ow7a1NQUuRJEJXyUt3LlStiwYcO4JYRqOyyVYM07bAVLVoDiS8abtu0bl2EjWd+x4CLAw1FtXRz0tpDObhz2YXbY2pKcFc+Y6p+KuOUSR1tbG3R2dsau2Q5TSiV\/UeMOI\/XW1lYYGhqCuPXh1IBilv2zl96E5dtfGFPfVnYdhhcHvenUdqcf+9AdX5hqQs0zpnqIfkbEXemglP2pAcWyiPwxzI+\/dAVcVvt+SpVTyeKgTwWXk43Zh066JZVS1DyTavCQxtrELV4kHjp0CG699dagdLF3794JIuO+hqxU2bD+VICqLx8xy97x+UtJXzSa2M9Bb4KaW33Yh275w0QbKp4xGTusjzZxUw1ILYcCUFdJG7HioKeeMfblsQ\/tY049IgXPUOpUeuJG0r6pd9\/YBzSYaT\/11Y9TYlyRLA76iuBzojP70Ak3VKSEF8Qtf0CjolG0UsmXvv8i3LtnKDDDNdLmjLuiWHOmMxO3M64wVsQL4o6yHuvec+bMif2Yxhi5iI6VAIpL\/vCjGkHaLtS0VTM56KlnjH157EP7mFOPWAnPUOuC8khLJXGbRmWhPMqsBNDqrzw0Rto2PqYxwYCD3gQ1t\/qwD93yh4k2lfCMyXhJfUiJu0gHKcy\/68mxujZm2ll\/AZnkiKjfOehNkXOnH\/vQHV+YauIFccfVuKN28TMFLKmfCaBqicSll5FcKknyePF+Z+Iuns9UjU14JkurSTPuLBWNkm0CqMi2XVmrHYcbB30es4p2TPYhLZ55SDPhmSz1NCZu9RP1uL21szQgLaC9v3wdlvXuC1T6yl\/OgK9ef3JbWFcvDnpXPaOvF\/tQHytXW6blmaztMCLuqF39dE+toTQqDaDymu0iZNuIEwc95WzJRxb7MB\/cKUdNwzOU40bJMiJuqm1dKQxMA6hc28ZMGzNu1y8Oetc9lKwf+zAZI9dbpOEZG7YYEXfU\/thx269mZUwaQItU2xZ4cdBnNXPsyWUf2sM6q5HS8ExWOshyjYgbBWBZpKOjA8QqEiTtlpYW6OrqCo4bs3XpAoplEnEgwlUfOgN2LLvUlooVjcNBXxF8TnRmHzrhhoqU0OWZigZJ0dmYuHGMqL21U4xfcVNdQG\/a9gJsfey1YDyX122rgHDQVzxFchfAPszdBRUroMszFQ+kKaAi4tYcI9NmOoDK2baL+5HEAcRBn+n0sSKcfWgF5kwH0eGZTBVQhBsRd9TLSZuKi7F0AJWXALbPmwnt8+ryUNVoTA56I9ic6sQ+dModRsro8IyRYMNORsSNY+GGUnV1dVbr2WE26gAqv5R0dU+SKP9x0BvObIe6sQ8dcoahKjo8YyjaqJsRcRdpW9eivpQU3uSgN5rXTnViHzrlDiNlvCBuI8sz6pQEqLx2G\/ckwRp3kS4O+iJ5K1xX9mHxfZjEM7YtNMq4bSsZN14SoPIugEzcLnnuXV18Jzbf7UNP+m5jEs\/Yjixt4i7iYcFFL5OUISDKYKPvpFYGHxaWuG3fUXTHiwNULpMUbTUJ17h1Z4D77Zi43fdRkobeEHcRdge855FXYeV9+wOfFOmjG3kScdAnhZT7v7MP3fdRkoZeEHdRdgeUlwG6fFhC3KThoE8KKfd\/Zx+676MkDb0g7iLsDuhDfbsMtcMy2MjEnUSL7v\/uBXEXYXdAmbiLWiYpA6mVwUYmbveJOUlDL4gbjXR9d8AFdz8ND+4bLnR9uwykVgYbmbiTaNH9370hboTa5d0Bfahvl4HUymAjE7f7xJykoVfEnWSsjd\/DAPWlvl0GUiuDjUzcNpgg2zGYuDXxlbP52bNnQ09PD1RXV0\/oHQaovH77ruaLofnKczRHda8ZB717PkmrEfswLWLutWfi1vAJvvxcs2YNLFq0COrr64N6+u7du2HdunVQVVU1TkIYoD\/edwQ+e\/fewte3y5CNlsFGJm6NoHe8CRO3gYMw+167di1s3LhxQtYdBqgv9e0ykFoZbGTiNgh6x7owcRs4JG3GXf2Vh4JRinS2ZBQsHPQGE8axLuxDxxxioA4TdwrQ5H2\/xaHEancVUPnF5JbPXQJNH5mWYkT3mnLQu+eTtBqxD9Mi5l57b4hbnOquQhz3ItHUHYLA29vboaGhYUKNG\/\/Q19cX\/H3o2DvQtGUw+Pe3\/+YcuPyDxdp\/W8VocHAQpk+fbgpdIfr5bqPv9uEk89XGuXPnjsVQf3+\/M\/Gkva2rrLEg0oULF1o5uixqbxTUSb0Trt81AOt3HQjULfIXkwJvztaciRVjRdiHxtA509GLjDvrw4JV+fhycuXKlbBhw4ZglYl8qYD69GIS7eSgdyZ2jRVhHxpD50xHL4gb0cQXhgMDA4Dliywu3a8yVUB9ejHJxJ3FzLIvk4nbPubUI3pB3K4eFiy\/mCzqwQnqhOOgpw5B+\/LYh\/Yxpx7RC+KmBqUSeTKg8heTPtS3OeOuZGa405eJ2x1fmGrCxG2KXEQ\/Jm5iQHMQ5zux+W5fGRIMb4hbrPTYuXMnNDU1weLFi+G2224L\/boxSy6QAZVPdD\/yjWuzHNaabA56a1BnNhD7MDNorQn2grjl5XkzZsyA3t7eYB+RHTt2RO4pkhXCYcRdW306FPWoMq5xZzVT8pPLxJ0f9lQje0Hc8nK94eHhMeLGRfhRe4pQAajKkQH1bUVJGR5By2AjE3dW0W9PrhfEjXCtX78ehoaG4IYbboAHHnggKJUsW7YsKJtktUQwzE0CUHlFyT+1XAwLrijuVq6ynRz09oIzq5HYh1kha0+uN8SNkKmfvXd1dVn5klJ2lwDUpz24mbjtBaSNkZi4baCc7RheEXe2UOlJF4DKn7pjfRvr3D5cHPTF9yL7sPg+ZOIm9qEAtOPfXoLun57cXIqJmxjkjMX5Tmy+21eG9xRM3MQkIAD1bY8SARMHPfGEyUEc+zAH0ImH9Ia45XXcAqOlS5dafTGJ4wpAP3rLLwBfUPpweALXuImjLmdxTNw5O4BgeC+IW5B2TU3NGFGLvyFGYWdDEmAXKgIB\/a9fPQ9I3Hhdf8lZ8L3Ff57VcNblctBbh5x8QPYhOaTWBXpB3FHbusadDZkV0gJQsYbbl82luFSS1YyxL5eJ2z7m1CN6QdwICi4FFF9MipPXcW13XV2d1SWBCOi9P3oC5n\/zycBXdzXEqXBbAAAKj0lEQVRfDM1X+rGGG+3hoKcOQfvy2If2Mace0QvijtvWVQYMjzG7\/\/77qTEcJw8BXbN9Nyzr3Rf83ZddATnjznTaWBXOxG0V7kwG84K4M0HGUCgC+uk1\/wG9v3w9kODTUkDOuA0nhWPdmLgdc4iBOkzcBqDFdUFAL2n7V3jk5aPBRze+bC7FGTfxRMlRHBN3juATDe0VcePxZR0dHWPQ5PXJOxM30ezMSYzvxOa7fWV4MvSGuPFFJL6g7OnpgerqahB174aGBqtruRHQo3\/dE1COb2u4yxAQZbCRiTunjIBwWC+I26XlgHWXfAyOXbc+cNE3PnMhfO7jNYTuyl8UB33+PqhUA\/ZhpQjm398L4kYYXcm4az\/2aThx1crAs76t4S5DNloGG5m48yfeSjXwhrgRCBdq3DJx+7YUsAykVgYbmbgrpc38+3tF3PnDCVBz\/Zfh7YvmB6owcbvgkfQ6+E5svttXhpsvE3f6uI7tce78DvjDBdcFbXxbw12GgCiDjUzcxEGfgzgmbmLQP\/AP34J3zrrQyzXcZSC1MtjIxE0c9DmIY+ImBp2JmxjQHMT5Tmy+21eGmy8TNzEx+HiyuwwRBz3xhMlBHPswB9CJh2TiJgZUEDfuCIg7A\/p2cdAX36Psw+L7kImb2Ie+7sMtYOKgJ54wOYhjH+YAOvGQTNzEgAri9m0fbiZu4omSozgm7hzBJxqaiZsISCFGELePa7jL8NKnDDYycRMHfQ7iSk\/cuDFVS0vLGPRbt24F3JhKvfAYtNbWVhgaGgp+wkMZxIZWclsm7hxmMfGQvhOb7\/aV4eZbauLGzalWr14NX\/va14IdBZHEcc+TMEIOOxotjC8Ecfv48U0ZAqIMNjJxE9\/pcxBXauJW8RZbwba3t0\/IunEflIGBgcQtYgVxH\/nGtTm4M\/shOeizxzjrEdiHWSOcvXwmbgljLIesXLkSNmzYAPX19ePQx0y8u7s7saSCxO3jyTfCcA767IMy6xHYh1kjnL18Ju7\/x3hkZARWrVoFjY2NE06FV3\/DssmKFStg8+bNEwgeidvHAxSYuLMPRlsjMHHbQjq7cZi4\/2\/fbEHMNTU1iaUQdEUcySNxTzr8Ijy6el52XstR8uDgIEyfPj1HDbIf2ncbfbcPZ4ivNs6dO3csAPr7+7MPBs0RThkdHR3VbEvSLI6EowYQfZqbmyfUwpG4ff1qEvHgbI1k2uUqhH2YK\/wkg5c649YlbfVotLjVJ0jcPp58I2abaxOGJAoUIb7b6Lt96E7fbXTNPqsZt7o2W8QvruWeNWsWtLW1QWdnZ1DHlttiSSWsvo39kbjf+8R34D0Hf54Fp7BMRoARYAQCBEpdKuE5wAgwAowAI1AZAlYz7spU5d6MACPACDACiAATN88DRoARYAQKhgATd8EcxuoyAowAI8DEzXOAEWAEGIGCIcDEXTCHsbqMACPACBSWuOXlgkuXLtX6AtNld+vYI9bB79y5MzClqakJ1q1bB1VVVS6bFuimY59sRNw+Nq4aq2uj3C5qu2IXbdS1T9662YfYFL7A\/ZPq6uombNGRh68KSdzyroI48aP2PMkDUJMxde3BHRPxWrBgQew2ACY6ZNlH1z6hg7hBPf7445Hr97PU10S2ro3qjpi6u2Ca6ETZR9c++YaLWzUUPTZl0sZN77q6upi4TScWTo61a9fCxo0bg329cfLv3r27MNmnarepPUWxO619aNczzzwDzz33XOjOkabzJst+ujZiNvrwww8X7gkxjX3yHvv4b7xw6+YiXvK+Sqg\/Z9wVeFE9ZCHuk\/gKhrHW1dSeogRFGvsEQeAjNtoXtuWvNcekGEjXRrwpHTp0CPr6+mDv3r2RJzulGNpKU137dDNzK0oTD8KlkgoBVTPNohO3iT1FsjmNfRgcc+bMgalTp0bu1V7h9Mmku66N2O7OO+8cKwEV5earax+CK8gbb0xRRxNm4oSMhTJxVwiw7t2\/wmGsdU9rT9z+5NaUTjGQrn1yGaFoLyd1bVRr2kW5Aevap9pTlBuTznRm4tZBKaaNbr2twmGsdU9jT1ECXQZP1z711COUEbfBmDUHaQyka6MuAWoMabWJrn0qURdxvkYBy8Rd4ZTzrY6ma09Rg0DXPpXso461q3D6ZNJd10Z5y2Kx6kL3QJFMFNcUqmtfWMY9NDRU2IUDMjxM3JqTJa6Z7ppSgqGsiIiyR54sYRlpUdZy69hXZOJG3XVtlNsVxX9p7MNyUEdHR+DOIq1TTwp0Ju4khPh3RoARYAQYgUgECvkBDvuTEWAEGIEyI8DEXWbvs+2MACNQSASYuAvpNlaaEWAEyowAE3eZvc+2MwKMQCERYOIupNtYaUaAESgzAkzcZfZ+gWyXt7RNs1Woi1\/uiW1Ps\/y4SF5y6NNn5wWaspmqysSdKbx+CE+zC2GatmnQMf34yFXi7u3tzfyjFHGza25uhoaGhjRwc1vHEWDidtxBLqiXhozTtE1jm\/qpuG5fJu5VwMStO1uK046Juzi+ylRT9XQdUY6QTzMRX\/kNDg5Ca2sr4KfMeMW1RblLliwJtjDFK+6xXd5VTm4r6xBVXpBLA3IbJO7jx48H\/8OTg9T+smwcU2yUL\/bmmDJlStBP6C1\/vYp2Y\/+enp5gX\/goHVTHqTchVce4rw3VG1HcjZIz7kxDJlfhTNy5wu\/O4PKudWrAy+SAGuOpJiKLU3fxC2vb2NgYnBoSt+OfGFO0VXdAjCuVqKfKyG3vvvvugHg3b94M9fX1wR7fYu8MHLOtrQ06OzuD3+R+w8PDwc1p+fLlYyeeyL\/jcXGIw8GDBwPixgtvUHhgAJYl4vQNI248XUW+OUTt78HE7U7M5KkJE3ee6Ds0trpPtKxaXFanEqzcFjNz+aQilBm134NK6mFELp+sIusXR5JpiA5137ZtW0DESNzqJldxO9\/t378f5Lp1XLYbRtwyUcfd4NLYwxm3QwFGrAoTNzGgRRYnbw4klxRU4pbLBfhYj5c4qUZui+WRlpaWCZCErQpRyTcNccfdWOKITjw9iMOXr776ajh27Fgocav6YF9Z5wcffHBsYyXZ4LAzCsOIG\/uI473kHQTxSUC+mLiLHGF0ujNx02HplSS5pLBjx46xMz0xi5Yz0bhSSVjGHQVSHhk33ljkLF4tlVSSccdNBs64vQqVXIxh4s4FdvcGDcvkBgYGgixQLX9g7ffWW28NarnYT64hx9W4RS164cKFE07KpqxxyzeB++67LwBbZLPqE8GKFSuC+rfYG1vUrMNKJWlq3OJFpcBJLe3IZRUVQ\/mmibV0tWwlyjmizo6\/r1u3DtS2XCpxL86oNGLipkKy4HLUVSXyygZBQtOmTQvKCPjCD1+m4bVp06bgv8VLObUttpFXlcR9PBO1qkQtS+AKDvWSV3Tgb\/KLvijixr\/jC0ax2gRfUqItWPbBK+wgB1EmwlIS2oVPI2GrSrB\/WJlE2KISN9a48aahHiCslk1kjIQOTz31VEDc6stWJu6CB2WM+kzc\/vqWLcsYAdO15Uk1biq1mbipkHRPDhO3ez5hjRxFIGzJpMmxY0zcjjq4QGoxcRfIWaxqvgiopRzTY8fUvUrUOjyFlbxXCQWK7sr4X4EbdsWZTk9eAAAAAElFTkSuQmCC","height":220,"width":366}}
%---
