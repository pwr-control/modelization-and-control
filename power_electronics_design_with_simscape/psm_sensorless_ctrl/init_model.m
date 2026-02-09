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
model = 'psm_bemf_ctrl';

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
fPWM_AFE = 5e3;
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
%[output:9ef89060]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"mosfet","value":"    data: 'danfoss_SKM1700MB20R4S2I4'"}}
%---
%[output:339abfd4]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAW8AAADdCAYAAAB0SfPeAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ+wV1Wd\/2JYPC2iRxS+RXioj3THEc20J8GiS8bU9LCpaeGxUwyQ0Yg0W48F3rOdlhIekLT+QYuxF2M78aCdMGFqllw0W0PMVSEzE\/TxwtdLw4coFu6Mu+x+r52fh8P9c+693\/Pnnt\/3zjTS+53zPd\/v5\/s9n\/u933vuOcNOnDhxAvhiBBgBRoARqBQCw5i8K+UvVpYRYAQYgQgBJm8OBEaAEWAEKogAk3cFnaar8oEDB2D+\/PnQ1tYGy5cv1+2m3e7IkSOwcOFCGD9+PKxZswYaGhq0+xZpuHbtWtixYwds2rQJWlpaiogo3Of48eOwYsUKaGpqMoJlYcU0Ou7Zswfmzp0btVy0aFEu\/V1irpom4m3OnDkwe\/ZsDcvDbsLkHbB\/XZA3jrlz5064\/vrryZFVicTkWKryggA3b94Mra2tmbbl1W3r1q0wYcIELdmZg0sNxE3n0KFD0NPTA42NjXm6g0\/kjYqjPuiLIrbkMrwCjZm8K+Ckqqho+mYhEwliYvKpQsU8D2nkxQGJu7OzE3RvDHniITTyznsTzYNV1doyeRf0GE7mjRs3Rr3xUVp+lBckc+WVV0YTEq\/u7u6THvXk\/ljWEGUHMfGx77Fjx6IygSpfVVlMfvF3QQIqiYh2+OiMuotHaNFucHDwpEdrtSyCP2LpQGRx+P9F2WTZsmVRtr1v375IxuTJk0\/KjgSJ4G+qrXJZRwfXW265BW688cZTxhL6xOkgxkc88RIYyH6Rywsy5gIHzLhF+Un8TR0rSYekv+\/fv79W0hB64RhJusSFq6qLiCfhL2Ez\/v+4G0RSfyyDiVgeM2ZMDW8ZMxVXGTcRV5deemkUM3hhxizbPH369OjvR48ercWLGo+yznlvjAWndyW6MXkXcJP6KKlmToKARJDH\/S5qt6NHj44IUBCDCE6cLBjoQ0NDqRlmmmw0Tc5OZfIWJKROBkEaqPvVV199Uk07jbyRkAcGBnLpivrcdttttRufDq4CN9U29eag6iLjhDcWvAmhLOEj2W6sp8qZtvDBkiVLajfguN\/FTUjFNI9uGAdpuqhlj6wbLBKwfMPN6q\/ipsay6iMZB\/lmLseDiGUcW9UXb35Yjxc3ezXe1Rix\/Z6lAD1Y68LknRPqtCxMEHBcbVaQ6LXXXnvKS740IkgL1qxH4qTMW85k0h7Zs4ghabImvSCV9fniF78YkYrIxNEW+SaGf1ex1imbqFkkZthiLLnuG0eQ8stQ+fEcdUGCkTNO+QlBzWaTssM43fAmmnYDxhezaaWCuN\/kv4kbVVLNW8VBnQ5ZN1Rsr2bfIvOPu5mr46kxfO+9955UQhJYihtnVsznnM6Vbs7kndN9cRNTJblbb731pFUR8u9qeUEMLx431YwyjbyzshAd8k57IUVN3mibuFEhaS1duhTEpMyLa1LmLbLp97\/\/\/bWngLgbZhx5izKYHBJI2PgiUSVv9dEe+whyT8q843RLIu8kXdRVFnE3X9m2WbNmpWbeWfX2LPIWNzG8Sao4x5G3Ol4SeavTUpT4mLzfRIbJOyd5m8i8ZRXUiR9S5o12CmKZNm0aPPvss7WSSV5cVfJWcYvL8vNk3rJPsrJTQUhJN+A03XQy77QQdZl5T5o06aSnSPH0JJaOUmTequ1M3kzeOSn75OZ5spGsySlq3kkBn5Vdq5mMXCNUCS4uy0oiFsyI1axN1CNFDVMtm8SVPlSg5dKBuuZYB9esdwX4cgzrrfj0I7+ULVLzVuvraY\/ucbVf9T1Gkm4qAWeVdGRMs56O8ta8VR+m+USQN+qD72dEySOtbFKk5i2vxMmaD6UmdsU6c+Zd0GE6qyKwhvv1r389GiFttYm8MiNP5i1Uz7vaJKlGq642kTNl\/DeWDnAFTNxqE7GCROCStkJGtIlb+aCDq1jZo4712GOPRfVSvMQqhpEjR0Zkjpd4SYm6Cd8krTbB9kK\/uKeCtFUW2DePboIw8eWdID7xIk\/4OG0ZYdpqEZ1MVWe1icBcTQDkVTEYx5iEiPhIetku91FjCl9qqiUp2Ue82sSTzBsDHK+4r\/9UB6pLzwpyrpVuaXVkKwrwIJkI5F0vLGfWeT90yVSmjhvELSFNgyOv30KG1lnmLZyQ9Lku\/t7b22vls2tqBzN5UyNaXl7c8k15mWLWCBiP+ILVxaf5WbpV6Xd1KSzqrq4ySrOHb6KOM298TFu5cmWkRdJeEfh41t\/fn2sfBl+CmMnbF0+8qYdaGpDLIjraVnlvEx37bLZRy3zyR2ppeggf8t4mb6DkJPNGcmtubo7IGa+4solc+8w70WwGIo\/FCDACjIALBKyTNz423XXXXXDDDTdEKwLiyFtkOVOmTIm+aONHVhehwWMyAoyAzwhYJW8k5VWrVsG8efOiLT3TXljKoKlkLv92zjnn+Iwv68YIMAKBILBr1y6YOHGiN9ZYJe+4r9IQiaw9hgV5t7e3n7JlJpJ3X1+fN4BSKxK6fYgX20gdNW7khe5H3+yzSt5qSCVl3vhioqOjA7q6uqIMHcsm2DZuD1\/fAKWeNqHbx+RNHTHu5IUeq77Z5w15q4QtZ+lpH3z4Bij11Dl48KBXj2rU9qE8ttEEqvZlhu5H37jGKXlThJdvgFLYJMsIfUIweVNHjDt5oceqb1zD5O0u1rVGDn1CMHlrhUElGoUeq0zexGHoG6DE5nFJgRpQR\/JCJ7Z6uAn7xjWceTuazLrD8qTXRcrvduxHv\/2jox2Ttw5KOdr4BmgO1bWa8qTXgsn7RuxH712UqaBvXMOZd6bL3DbgSe8Wf6rR2Y9USLqTw+RNjL1vgBKbxzVvakAdyWPydgQ84bC+cQ1n3oTONSGKJ70JVO3LZD\/ax5x6RCZvYkR9A5TYPM68qQF1JI\/J2xHwhMP6xjWceRM614QonvQmULUvk\/1oH3PqEZm8iRH1DVBi8zjzpgbUkTwmb0fAEw7rG9dw5k3oXBOieNKbQNW+TPajfcypR2TyJkbUN0CJzePMmxpQR\/KYvB0BTzisb1zDmTehc02I4klvAlX7MtmP9jGnHrGy5B13gGsWOJMnT4a77747q1mp330DtJQxMZ150lMj6kYe+9EN7pSj+sY12pm3ut92Fii4H\/fq1ath06ZNWU1L\/e4boKWMYfKmhs8beUze3riisCK+cY02eRe22HBH3wClNpcnPTWibuSxH93gTjmqb1yjTd6ibDJ+\/HhYs2YNNDQ0UOJSWJZvgBY2JKEjT3pqRN3IYz+6wZ1yVN+4Rpu8EYStW7dCZ2dnDY\/u7m6YPXs2JT65ZfkGaG4DMjrwpKdG1I089qMb3ClH9Y1rcpG3DAQeCLxx48boT\/hiMu5wYErgkmT5Bii1zTzpqRF1I4\/96AZ3ylF945rC5C1AUVehLFq0CJYvX06JWaos3wClNpwnPTWibuSxH93gTjmqb1xTmrxlcMQKk\/Xr10NjYyMlbomyfAOU2mie9NSIupHHfnSDO+WovnFNafJWM++2tjarLzR9A5QyWFAWT3pqRN3IYz+6wZ1yVN+4phB5Y4Y9f\/58GBwcjLBpamqK1nO3tLRQYqUlyzdAtZTO0YgnfQ6wPG7KfvTYOZqq+cY12uQd94Xl5s2bobW1VdN0M818A5TaSp701Ii6kcd+dIM75ai+cU1u8kaytvlCMgt83wDN0jfv7zzp8yLmZ3v2o59+yaOVb1yTi7w7Ojqgq6tLqzzCn8fnCYvktjzpaXB0LYX96NoD5cdvvvBy6P\/1L8sLIpKQi7wXLlwI+\/bt0x6aN6bShiqxIU\/68hj6IIH96IMXiuvQ+8jzsLj3KTjyzauKCyHuqU3exOOSifPtUYbMsL8I4klPjagbeexHN7hTjcrkTYWkJIfJ2wColkUysVkG3NBwIftx7c5+WLvzIGfelLHD5E2JphtZIU96gSjb6Ca2qEZl8qZCkjNvA0i6E8nE5g57ypFD9iOTd8FIwU2w8IpbosiZd0FQPeoW8qTnzNujQCuhCpN3AfD27NkDc+fOhaQNr5i8C4DqWRcmb88cUlCdkP2IK03wpSWvNtEMDvyqc+XKlVFr\/ASfM29N4CrWLORJz5l3xYIxQd0v\/dvTcNdDg2GQ9\/Hjx2HFihWwY8cOwM2oFixYADfffDNQ7iiI5ZLm5mbo7+\/nskkYcyDWCibvMJwbsh9n3f44PPjs0eqTtyDuKVOmwIQJE6C3tzfaSXD79u2we\/dukl0F8QvNu+66C2644Qa49dZbmbzDmN9M3uzHSiJw8Y0PwaEjr1WfvOWT5IeGhmrkPTAwEJ0YXzb7xpvDqlWrYN68edGn+FkvLDEadu3aVcmgyFIaMR03blxWs0r\/zjZW2n015UP141Vts+GVj7yxaCKImjcSKm4Je80118A999wTlU0WL14clVDKblylbjkroiPupSW\/sKz+xA\/5cZtr3tWPzwefOQqz7ng8HPJGS8RKEOEeUwcSZ2XefX191Y+QBAuY2MJwLfuxun4Un8YHk3nbdAWT90SbcFsfi4nNOuRGBgzRj1jnvr73qehl5Wl\/fhFe\/PanjWBXRChvTFUENYt9QpwQKnxso8WAMjhUiH6USyYjDvwEBnd8wyCC+UQXIu+4U3XUYW0djcY173wO97F1iJOeb1A+Rlo+nTDrxlo3\/nd84wh45bt\/Dz6VaAuRN0Ig1mDPnj27hsjWrVujNdn4whL\/jcsGb7nllnyI5WzN5J0TMA+bM3l76JQCKoXkRyTsb\/y0H77\/yz9ESNz0qUlw42emVZ+85aWC8qHD4vQcXCqISwhx2SAeTGzyYvI2ia4d2SFN+iTE2EY7sUQxChL3T379InT96EAkDrPuvV+5AnzjmlKZ98aNG0EcQqzuQcKZN0UYAfCkp8HRtRT2o2sP6I0vv6AUxL39uksiAg+GvNEweT22XOOWM\/DGxkY91Aq28g3QgmYkduNJT42oG3nsRze45xkViRu\/pBQXErYgbvybb1xTOPPOA4rJtr4BSm0rT3pqRN3IYz+6wT1rVCTsKNve8lT03yTiZvLOQrLA70zeBUDzrAsTm2cOKahO1fworyaRSXvDnAtg6nmjTkHBN64pnHmrX1cKS\/HE+J6eHjBdLhHj+QZowbjnsslE\/hCJOnZsy6sCeeO67XU7D0Yf3cgXlki+8Ddnw8cufHdU3467fOOaQuQt1nkvWbIE7rvvvmgDKdw8CbeIxZ0G5eWDpgPIN0Cp7a3ChChrM9tYFkE\/+vvqR\/UlpEraWz53EZw\/9sxMEH3jmsLk3dHRAV1dXbBt27Zoz20kbJsvKjnzzoy1yjTwddJTAsg2UqKZLguz6x2\/Ogx3PjgQ2xAz6\/HvGgEb2i9IzLKDzbzFft64wmT69OnRMWXf+c53ot0FDx06xGUTwjjlSU8IpkNR7Edz4GNmvXbnweiYsqRLEPaymRNj69k62gWReQtDN2zYADNnzow+yEECx+1g8VCGhoYGHSxI2vgGKIlRkhCe9NSIupHHfqTBHYn6Xx8ehIf7Xj6lbq2OgIT9tVnnwcXj3pErw07S1DeuKVQ2oXEDjRTfAKWx6k0pPOmpEXUjj\/2YD3csfSD5Ykb93JHXMokapWP75TMnwofOHUVC1qrGvnENk3e+mLLemie9dciNDMh+jIdVrK3OQ9KCqJGkkayTVodQOzII8tbZ24SXCtKEDk96GhxdS6lnPwqCfnLwVfjWA8\/BoZfe+DBG90Jynt7yLui4ujnqYousg8q8k44nk420Xff27W6oG5C67ep50utiVIV29eDHX+w7AH86vRE23H8ocom6ljrLT+KlYtfHzoGmd77NGUkn6ekb1xQqmyRl3lnOMfG7b4BS21gPk55tpI4aM\/IwWz4BAN9+4DnALDpvBi1nzVPPw0x6Arxl2DDvSDpo8jYTGsWkMnkXw82nXkze7r0hyhj43x\/t\/SPsf+FPhchZJmhcS\/2PM5thQmNDZQg6zRO+cY125q1zeg4azp\/H005EJjZaPF1Jc+1HQc7\/8dsh+NHjfyxU1pCxEyWOCaMbogz6tGHD4H9e\/gNMDHibg8qSt6ugzxrXN0Cz9M37u+tJn1ffIu3ZxiKovdlHEPPegWPw098MwaGh44WzZiFVvBTEFR2fmzoORp95emb2HLoffeMa7cy7XHiZ6+0boNSWhj4hEC+2MTlqBDH\/8LEX4P6nj5TOmNWyxvjRDfDlD0+A4acNi2SXWckRuh9945pS5I2n5XR2dtYir7u72+qmVDiwb4AyeedHIPRJH3eDEqT8mz+8Cj954kXoJ8iW5awZ681nN46Az7Y2wVnvfFtpYtbxauh+9I1rCpM3HkCM28KK7V9FTby1tTU6gNjW5Rug1HaHPiFCy7wFKf+y\/+UoU8avA4usyoiLI5EVIzG3vPdMmNd6Fow643QrxKwT16HHqm9cU4i8+SMdnVCmaRP6hKgCeQtC\/t8TJ6LNjx76y17QVKQslyuwxvy+sWfCJya\/pxZAZUoZNFGoJyX0WA2CvNGVnHnrBXTZVqFPCBfkrX7dt\/vZo\/Cfz7xEmiXLJQz895gGgE9d9sZm\/2Vry2VjylT\/0GM1GPLGAOCat6lp8Kbc0CcEJXnLpIwrL+7\/7RF49vCfycoWKiGLuvJH\/no0XHL2yBopox5qtsx+ND9XTI8QFHmbBktHvm+A6uicp029T3qxuxwSIr7c2zdwzEiGLGfDSMqXT3wn\/N0HxsKI4aeRZMr17sc8Me9rW9+4plDN2ydwfQOUGpsQJ73IkI+99jpseeR52PPMYWgYMYI8Q1YJuXl0Ayz40F9B45n2X\/KF6Ec11kO30TeuYfKmZltieVWYEPKn1ZgZY4ZMucpChVRedYEv9+ZcNhbGvP2ttWb4e1zpgtg1ucRVwY+5DIppHLqNQZC3WBY4fvx46yfnqDHjG6BlJ4DrbEZ9mffwwZcBX+iZqB0LW5tGDofhw4dH5wri2mQ8DLbtojHRJ9dy9kyNrU15oRMbYhm6jb5xTeHMO257WP5Ih54OKCaETMi4\/viB\/S\/B74aOR8pSLneTrZez43PGnAEfv+jdMOk9b57QLWfHFDbSI08rkW2kxdOFtGDIOw48XH2yZcsWPoCYMLLiJr1Mxr\/6\/TH49yff2M\/CFhljdjzj\/NHwgQlvrLAomx0zsREGjENRofsxGPKOy7zxNPlNmzZBS0tLYgjJ\/dJ2IFTlJ7X1DdCicwcJWdRpf\/j4C\/DMH\/8c1Y37Dr8Kg6+8XlRsYj85M0YyvnTCSPjw+aNPam+rdhz6pK+HkkI92Ogb1xQqm4iaNzpMfB6vwy7Hjx+HVatWwbx58yKCx0x99+7dsXVz\/PS+t7c3s6buG6BxOMiZ8i33\/Q4OvGB27TFu03nB2DPh4xeNIcmMdXxbpg2Tdxn0\/Okbuh9945pC5E0VLphdr169GtavXw\/qmZdI7P39\/Zn7pPgGKBL1Dx59Hn6+\/6Xcx0CpuGLm+\/rrr8M5Y94Ok957Jnx+2jgYcfqb6459W1FRNC5Cn\/T1kJXWg42+cY1T8k7LvPHz+40bN9b4YPPmzYCbXqmXa0BxFcaXfvB0LqKW90r+8AWj4dLxb36dp9rHxFb0luBXP\/ajX\/4ooo1rrlF1dkLe8qk8caSM5ZUVK1bAlClToi1msYSydOnS2Ho6AorXrl27ivijUJ879rwEPY+8nNoXl7+d9Y7h8DcTz4C\/PfeMqC3+Le81MDAA48aNy9utUu3Zxkq5K1HZUP04Y8aMms19fX3eOMsJeQvrBYnjFrJxWbVop5K5jJ6tuyF+pn39lqeil4px5Q1co9x++VnQftlYUudyxkYKpzNh7Edn0JMNbItrdBUuRN5UW8KmkbJsgGjX3t5+CsmbBFQQ9aw7Hj+FtLH0cfcXLoa3nGb29Gue9Lqh7Hc79qPf\/tHRziTX6IyvtslF3nHLA1WBbW1tiStEVNJHecuWLYN169adtLxQbYdlE6yBx61sMQUoErdK2kjYn\/ngWfDpS8eWOi4qj6N40udBy9+27Ed\/faOrmSmu0R2\/FHnL5Y6Ojg7o6upKXdMdp5R6AxA17zhinz9\/PgwODkLa+nFqQMV6ayRucSFpb7\/uEmuELePGk75oaPvVj\/3olz+KaEPNNUV0kPvkyrzLDmaiPzWga3f2w9qdB2uqXjf97GiJnqvTTHjSm4ga+zLZj\/Yxpx6RmmvK6qdN3uLl4uHDh+Gmm26Kyhj79u07Zfy0rybLKhvXnwpQzLiv732qtuQPyXrDnAtg6nmjTKitLZMnvTZUXjdkP3rtHi3lqLhGazCNRtrkrSHLSRMKQOOI21WZRAWRJ72TsCIflP1IDql1gRRcQ6l03ZO3z8SNjuZJTxnu7mSxH91hTzVyEOQtf2SjAlO1ssnPD7wEn\/jW3sgMly8mkwKMJz3V1HMrh\/3oFn+K0YMg7yQgsA4+ffr01A9uKECUZZQBFLPui298yFvi5sybOlrcyWPydoc91chluIZKB1kOadkkbaMpE8qjzKKAquu4937lCmcrStKw4UlvKnLsymU\/2sXbxGhFucaELiiTlLyrdBjDP21\/Bm7\/2XMRrv\/y6ffBvCuaTGFcSi5P+lLwedOZ\/eiNKworEgR5p9W8k3b\/K4xYRscigKrlEsy6fb140vvqmXx6sR\/z4eVj6yJcY9IO0szbpKJJsosAOuv2x2vruXFJoOu13Fw2OQgTJ050ET7WxmTytga1sYGKcI0xZcqUTdTP2dP25jZpQF5A5ax76rmjYPviS0yqV1o2T\/rSEHohgP3ohRtKKZGXa0oNptG5UOadtBug7uk3GnppN8kLaJWybgSBJ712KHjdkP3otXu0lMvLNVpCSzQqRN5UW8KW0LvWNQ+guCe32HBq6nnvgu3XXUyhglEZPOmNwmtNOPvRGtTGBsrDNcaUkAQXIu+k\/bXTtm41ZUweQKuWdXPmbSpq7Mtl8raPOfWIebiGeuw4eYXIGwVhiaSzsxPE6hIk7rlz50J3d3d0dJmtSxfQKq0wkbHjSW8rksyOw340i68N6bpcY0MXHKMweWPnpL25bSmP4+gCKm\/16vsKEyZvmxFkZywmbzs4mxxFl2tM6iDLLkXetpRMG0cXUFEywf1LfF7XrdrKk96HKCuvA\/uxPIauJehyjS09C5F30gtLW0rL4+gAKr+oXNl2Liy5arwLVQuNyZO+EGzedWI\/eueS3ArpcE1uoSU6FCJvHA83oWpubrZa346zUwfQr\/24D27e9buoe5VKJqgvT\/oS0e1RV\/ajR84oqIoO1xQUXahbIfKu2pawjV++PwKnaiUTJu9CMe1lJyZvL92SS6kgyDuXxYYbZwEqrzLZ9oXJcOWkRsMa0YrnSU+Lpytp7EdXyNONm8U1dCPpSSqUeeuJttMqC9Dv7RmEf\/jB05Eyvm77moYUT3o7cWR6FPajaYTNy8\/iGvManDyCNnlX9QDiqq4yEW7iSW97SpgZj\/1oBlebUitL3jZByjNWGqByyeTaqeNg7Sdb8oj2oi1Pei\/cUFoJ9mNpCJ0LCIa8q7CroLxEsIolE4xWnvTO5yyJAuxHEhidCgmCvKuyq2DVSyZM3k7nKungTN6kcDoRFgR5V2VXQbFEsAr7didFI096J\/OUfFD2Izmk1gUGQd5V2FVQrndX7cMcOSp50lufo0YGZD8agdWq0CDIGxHzfVdBeSOqqta7uWxidW4aHYzJ2yi8VoQHQ96Ils+7CoZQ72bytjInrQzC5G0FZqODBEXeRpHSFB4HaNXOqUwzlSe9ZiB43oz96LmDNNRj8tYAKU+TLPJePnMiLJ\/ZnEekV2150nvljsLKsB8LQ+dNRyZvTVfIJZnJkydDT08PNDaeui9JHKC9jzwPi3ufikaq8stKLptoBksFmjF5V8BJGSoyeWv4EFezrFq1CubNmwctLS3Ry9Hdu3fDmjVroKGh4SQJcYDKZ1VW+WUlk7dGsFSkCZN3RRyVoiaTdwEfYha+evVqWL9+\/SnZdxp5V3ELWBUenvQFAsbDLuxHD52SUyUm75yAYfM8mXdILys58y4QLJ52YfL21DE51AqGvMVp8artafXpHDhFTeVDH8Qp9aoMBBSvXbt2Rf999Pevwee3PR\/9e9Hlo+DzHxyVd1iv2g8MDMC4ceO80olaGbaRGlE38kL144wZM2qA9vX1uQE3ZlTtLWHlvoJU58yZY+UYNDHe8uXLobW19SQz1LthKB\/nCCM5Y\/NmrpRShP1YCj4vOgeReds+gDhpIyz0qAqo\/LLyyDev8sLpZZTgSV8GPX\/6sh\/98UVRTYIgbzQe69D9\/f2A2TD1pd4c8IXlsmXLYN26ddHqE\/lKIu8QXlainTzpqaPLjTz2oxvcKUcNgrxtHECs++m9DGhoLyuZvCmnnltZTN5u8acYPQjypgCCSoYKqNgGtupfVnLNmypC\/JDD5O2HH8poweRdBr2YvjKg8sk5Vf+yksmbOFAci2PyduwAguGDIW\/xEnHHjh3Q1tYGCxYsgJtvvjn2QxoC3BJFyICG9Fk8k7fJqLEvm8nbPubUIwZB3vLqjwkTJkBvb2\/06fr27dsTP2OnBlLIkwENZRtYGSue9KYix65c9qNdvE2MFgR5y6tBhoaGauSNi\/STPmM3ASbKlAG9+MaHAF9ahrLSBO3jSW8qcuzKZT\/axdvEaEGQNwKzdu1aGBwchGuuuQbuueeeqGyyePHiqIRiYvlgkjNkQEM4s1K1kye9iWloXyb70T7m1CMGQ94IjPqJfHd3t5UvLmWnCEDlZYKhrDThzJt6+rmTx+TtDnuqkYMibypQysiJI+9QVpoweZeJDL\/6Mnn75Y8i2jB5F0EtpY8AVN7ThMmbGGTD4pjYDANsSXzofmTyJg4kASienINLBfEKYU8TAVPoE4KfLognhENxocdqMOQtr\/MW8bJo0SKrLytxXAFoiMsEmdgcMhHx0KETWz3EahDkLYi7qampRtbib+jEuOPKiOdCTZwANMSVJvUwIdhJdmgzAAAK4ElEQVRGUzPDvtzQb1BBkHfSlrBpx5WZCiUE9Gf\/9RvANd54tV82Fm5vv8DUcNblhj4hmLyth5SxAUOP1SDIG72PywTFl5XiUGBc+93c3Gx1uSAC+r2fPgaz7ng8CkokbiTwUK7QJwSTdyiRGv4HZUGQd9qWsHIo4pFod999t9HoVMk7pJUmTGxGQ8eqcL4JW4XbyGBBkLcRZAoKZfIuCJxH3ZjYPHJGCVVC9yOTd4ngiOuKgH501Y+DXCbImTdxsDgUFzqx1UOsBkXeeBRaZ2dnbUq4+jz+wo4fwoPPHg1qQyoBKk96h4xLODT7kRBMR6KCIW98OYkvLXt6eqCxsRFEHRxPd7e9MdXIBd8PbjdBJm9HM9TQsEzehoC1KDYI8vZtqeDRT\/RELpx67ijYvvgSi+40PxRPevMY2xiB\/WgDZbNjBEHeCJEvmXfzhZfDKx9ZG3ktpN0EOfM2OxFtS2fyto04\/XjBkDdC40PNe\/zlH4VXpy5j8qaPVWsSmdisQW10oND9GBR5G40ETeEyee\/9yhXRS8uQrtAnBPqKbQwjYkP3I5M3cZw2fexL8Nr5syKpoX2gw8RGHCwOxYVObPUQq0zexBPorE\/+M\/x38\/RIKmfexOBaEsfEZglow8OE7kcmb+IAes9nvw2vv\/t9Qa7xrodshm0knhAOxTF52wV\/2IkTJ07YHZJ2NCZvWjxdSAt90vMNykVU0Y\/JmTcxpqHu4y1gYmIjDhhH4tiPjoAnHJbJmxBMFCXIO7R9vJm8iQPFsTgmb8cOIBieyZsARFmEIO8QP9Dhx23iYHEojsnbIfhEQzN5EwEpxAjyDu0QBs68iQPFsTgmb8cOIBieyZsAxLjMO8Q13px5EweLQ3FM3g7BJxq67skbdyKcO3duDc7NmzcD7kSoXnge5vz582FwcDD6CU\/lETsYMnkTRaMnYpjYPHFESTVC92NdkzfuRrhy5Ur46le\/Gm0ji0SOG1zFkXLcGZlxsSXKJiF+oMOZd0k28ah76MRWD7Fa1+StziWxBzju\/61m37jpVX9\/f+be4EjeuJ8JkneIF0\/6MLzKfqy+H5m8JR9iaWTZsmWwbt06aGlpOcm7mJFv3Lgxs7zC5F39ScHEVn0fcuZt34fOvrA8fvw4rFixAqZMmQKzZ88+yXL1NyyhLF26FDZt2nQKySN5h3gIgwCEic3+pDAxIvvRBKp2ZXLm\/f+HJghybmpqyiyLoHvSiB7Je\/iLT8PDK2fa9aSl0QYGBmDcuHGWRnMzDNvoBnfqUUP144wZM2pQ9fX1UcNWWJ71zDuNiJOsEH3a29tPqY0jeYf6gU49PIqyjYXnrncdQ3+6qOvMW5e41TMy01alhE7evgWMCcZgG02gal9m6H70zT6rmbe6dluEF671njRpEnR0dEBXV1dU15bbYnklrt6N\/ZG8z3jsu\/DWQ7+wH608IiPACNQVAnVdNqkrT7OxjAAjwAgYQsBq5m3IBhbLCDACjEDdIcDkXXcuZ4MZAUYgBASYvEPwItvACDACdYcAk3fduZwNZgQYgRAQYPIOwYtsAyPACNQdApUlb3kp4aJFi7S+1PTZuzr2iHXyO3bsiExpa2uDNWvWQENDg8+m1XTTsVE2JG3vGx8N1rVPbpe01bGP9qFOujbKWz+HMD+FP3DPpebm5lO29HDhr0qSt7wbIQZ\/0h4pLgAtMqauPbjTIl64F4zuB09F9DHRR9dGMbaw79FHH01c429Cz6Iyde1Td9LU3T2zqF6U\/XRtlG+6uLVD1eenTNy4WV53dzeTd9HAwuBYvXo1rF+\/PtoXHCfA7t27K5WFqhlmEXuqZHden6FtTzzxBDz55JOxu04WjR1T\/XTtw4z0gQceqOSTYh4b5X368d944dbPVbzkvZhQf868S3hRPagh7fP5EsNY61rUnipNijw2CpLAx220MW7LYGvO0RxI1z68KR0+fBh27doF+\/btSzwhSnNYq810bdTN0K0qTzQYl01KAqlmnFUn7yL2VM3mPDbiBJk+fTqMHj06cb\/3kiFE3l3XPmx322231UpBVboB69qI4AoCxxtU0lGH5E6wIJDJuyTIuhlAyWGsdc9rT9r+5taUzjmQro1yWaFKLyx17VNr3FW6CevaqNpUpRtUVlgzeWchlPG7bu2t5DDWuuexp0qTXQZQ10b1BCWUkbYxmTUnEcWkLgH6YldRH2I\/UeOuaszG+YDJu2RkhlZT07WnypNA10aVLJKOySsZQuTdde2TtzsWKzF0DyUhVzqnQF0b4zLvwcHByi4okGFi8s4ZNHHNddebEgxlRUSSPXKwxGWlVVrrrWNjVckb9da1T25XJf\/lsRHLQ52dnZE7q7aWPW3CM3lboUMehBFgBBiBcBGo5Ec64bqDLWMEGAFGQA8BJm89nLgVI8AIMAJeIcDk7ZU7WBlGgBFgBPQQYPLWw4lbMQKMACPgFQJM3l65g5VhBBgBRkAPASZvPZy4lWME5O1w82wx6uPXfWK7VJMfH8nLEUP6PN1xGHo1PJO3V+7wU5k8uxfmaZvH2qIfKPlK3r29vcY\/WhE3vPb2dmhtbc0DN7etAAJM3hVwkmsV8xBynrZ57FI\/K9fty+S9Api8daOlWu2YvKvlL2Paqqf0iNKEfCKK+BpwYGAA5s+fD\/jJM15pbVHuwoULo+1P8Up7hJd3opPbyjoklRrkMoHcBsn72LFj0f\/wBCK1vywbxxQb7Yu9WEaOHBn1E3rLX7mi3di\/p6cn2lc+SQfVaeqNSNUx7YtE9WaUdrPkzNvYdPFCMJO3F25wr4S825066WWCQE3xZBSRzak7\/8W1nTJlSnTySNougerJQOrOiWllE\/V0GrntnXfeGZHvpk2boKWlJdofXOyzgWN2dHRAV1dX9Jvcb2hoKLpBLVmypHZqivw7Hj2HOBw6dCgib7zwJoWbMWGJIk3fOPLGE1rkG0TSXiBM3u7nii8aMHn74gnHeqj7TMvqpGV3KsnKbTFDl08IQplJe0OoxB5H5vLpLLJ+aUSZh+xQ9y1btkRkjOStboqlypLH3b9\/P8h17LSsN468ZbJOu8nlsYczb8eTyvDwTN6GAa6SeHkzIbm8oJK3XDrAR3y8xGk3clsslcydO\/cUCOJWi6gEnIe8024uaWQnniLEgc7Tpk2DV155JZa8484MlXW+9957axsxyQbHnXcYR97YR2yhKu88iE8E8sXkXaUZZVZXJm+z+FZWulxe2L59e+2MUMym5Yw0rWwSl3knAeIi88abi5zNq2WTMpl3muM5867stPBKcSZvr9zhTpm4jK6\/vz\/KBtVSCNaCb7rppqi2i\/3kmnJazVvUpufMmXPK6duUNW\/5RrBt27YIVJHVqk8GS5cujerhYm9tUcOOK5vkqXmLl5cCJ7XMI5dYVAzlGyfW1tUSlijtiLo7\/r5mzRpQ23LZxN18sjEyk7cNlCswhrraRF7xIIhozJgxUUkBXwLiCza8NmzYEP1\/8aJObYtt5NUmaR\/YJK02QRlZ67zllR7YXn75l0TectkEy0T44hJtwRIQXnEHQYiSEbZHu\/CpJG61CfaPK5kIW1Tyxpo33jjUQ4nVEoqMkdBh7969EXmrL2CZvCsw8UqoyORdAjzuWt8IFF17nlXzpkKVyZsKST\/lMHn76RfWykME4pZTFjnCjMnbQ+dWUCUm7wo6jVV2g4Ba1il6hJm6t4lal6ewjvc2oUDRbxn\/B4+rn8K7CkI+AAAAAElFTkSuQmCC","height":221,"width":367}}
%---
