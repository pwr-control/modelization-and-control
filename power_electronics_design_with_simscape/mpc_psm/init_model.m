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
use_observer_from_simulink_module_1 = 1;
use_observer_from_ccaller_module_1 = 0;
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
fPWM_AFE = 24e3;
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
l1 = Kd(2) %[output:48227209]
l2 = Kd(1) %[output:3070f48b]

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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:3c155842]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:89f0f782]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:9a38523f]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:45969b88]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:36e3e7d0]
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
kg = Kobs(1) %[output:983bdd5b]
kw = Kobs(2) %[output:2c5ea1cd]

%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:9cd1b426]
luenberger_l2 = Klo(2) %[output:0f86fa0a]
luenberger_l3 = Klo(3) %[output:52a550a8]
omega_flt_fcut = 10;
% phase_compensation_omega = -pi/2-pi/12; % for motor mode
phase_compensation_omega = 0; % for generator mode
%[text] ### Control settings
id_lim = 0.35;
%[text] #### rotor speed control
kp_w = 2;
ki_w = 18;
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
%[text] #### Speed obserfer filter LPF 10Hz
fcut_10Hz_flt = 10;
omega_flt_g0 = fcut_10Hz_flt * ts_inv * 2*pi;
omega_flt_g1 = 1 - omega_flt_g0;
%[text] #### Motor Voltage to Udc Scaling
motorc_m_scale = 2/3*Vdc_bez/ubez;
%%
%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] ### HeatSink settings
heatsink_liquid_2kW; %[output:7700a89a] %[output:36462aac] %[output:57299408]
%[text] ### DEVICES settings (IGBT)
% infineon_FF650R17IE4D_B2;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;

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
igbt.inv.Lstray_module = Lstray_module;              % [H]
igbt.inv.Irr = Irr;                                  % [A]
igbt.inv.Csnubber = Csnubber;                        % [F]
igbt.inv.Rsnubber = Rsnubber;                        % [Ohm]
% inv.Csnubber = (inv.Irr)^2*Lstray_module/Vdc_bez^2
% inv.Rsnubber = 1/(inv.Csnubber*fPWM_INV)/5

% infineon_FF650R17IE4;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;

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
igbt.afe.Csnubber = Csnubber;                        % [F]
igbt.afe.Rsnubber = Rsnubber;                        % [Ohm]
% afe.Csnubber = (afe.Irr)^2*Lstray_module/Vdc_bez^2
% afe.Rsnubber = 1/(afe.Csnubber*fPWM_AFE)/5

%[text] ### DEVICES settings (MOSFET)
infineon_FF1000UXTR23T2M1;

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
dead_time = 2e-6;
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
figure;  %[output:51db43ea]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:51db43ea]
xlabel('state of charge [p.u.]'); %[output:51db43ea]
ylabel('open circuit voltage [V]'); %[output:51db43ea]
title('open circuit voltage(state of charge)'); %[output:51db43ea]
grid on %[output:51db43ea]
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
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);

%[text] ## 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":26.9}
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
%[output:48227209]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  15.921682195383369"}}
%---
%[output:3070f48b]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.064017382188212"}}
%---
%[output:3c155842]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.064017382188212"],["15.921682195383369"]]}}
%---
%[output:89f0f782]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:9a38523f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:45969b88]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000041666666667"],["-4.112335167120566","0.999345501530502"]]}}
%---
%[output:36e3e7d0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.064795348480289"],["11.319202547499357"]]}}
%---
%[output:983bdd5b]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.012443765785704"}}
%---
%[output:2c5ea1cd]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.517590710054527"}}
%---
%[output:9cd1b426]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.152987786896029"}}
%---
%[output:0f86fa0a]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"  93.745795204429072"}}
%---
%[output:52a550a8]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -1.166194503484206e+02"}}
%---
%[output:7700a89a]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:36462aac]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:57299408]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
%[output:51db43ea]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAN4AAACGCAYAAACyujQtAAAAAXNSR0IArs4c6QAAGKNJREFUeF7tXX9s1dd1P2lphcmIgoEkIsQxSXBStSkjbGvmsQWWJvAPZLU6OUYCBpRRBlisuGB+RARtGMNwpwGtwiLXgmkxKUmU4GkTS8mMknhUSwBLE0lNA8YNjIQYUCIgU7t4Ol\/nvJx3fe\/33e\/3e+\/3ved3vlJF83x\/nc89n3vOPffXTQMDAwOXL1+GJUuWQHd3N+T6pkyZAq2trVBeXp4rqfxdEBAEDAjcRMTbsmULbN68OZRQSFCbdIK2ICAIhCMQEE9AEgQEgXQRyFg8dDXxK2Y38tixYzBv3jx47rnn4OGHH3aKpFr2jRs3YNeuXbB06VJnbjeW2djYGLS7ubkZysrKAOs9d+4c1NbWOpVHLez555+Hrq6uTL1hle3ZswdmzZoFkydPztmmKGlzFvZ5AsKpo6MDJkyYAG1tbVZt0eFrW6fLdOg5ZiweFwYriSKQy0YVS1nbt28PSOFyoFIV4\/3334dFixbBqlWrvBKP5vhPPvlkznqQoLt377ZS9ihpo\/T76dOnA1zmzJkD69ats85aKMTDBhtdTQRt\/fr1GaG2bdsWdAoJfeHCheBvZF1IqI8\/\/jj4\/ejRo0PIS1YD\/86DNPT7U089FSgylk316VA1tYFbJSwfrQe1B\/PgyDhx4sTgdxwt8Vu2bFnQeVQmKblq4fh\/owUibNQBSte5KklzYYjtWrt2LaxcuTIT8KJ28n5R68Z69u7dG+A+Y8YM6OzszBBEHVg5vipB1GAbpeX9R31fVVWVFZijdurSohfC24\/EIcuu9rOpDervujJUWalNOh1Vg4WEoUlHsSzEmMoMw1xtK\/fErOZ4FFSZP38+rF69OjMC8w4jhX777beDzh47dmzQIRUVFQG4fPSeO3dulkuF0VR0EbkwJmuijs5cqXt6ejKuJhGP2kNuEaZHYmObqF4EBNvLrUsY8VCBwiwe4nLgwIFgEMEPccA8OoLrMCRXU7V4SNimpiZoaWnJlEv4kixIEsKXy27CiWQhTNC95WlfffXVLAunkhTTVlZWBoMykYoUTE3LMSXCEi6ceNTH9De1L3JZPFMfqzqBdap93t7enqWvZFWpDaSjmJd+02FOfKC+PHToUBaOWuKpTKWR9fjx41mZOUD19fVD5icceDUvV04iDI2sYS4KCtvQ0KB1dXQWjwRHhQpzqaJYvFzEo7J27twZ6BOfd5qsC5apYmhyNXVWA+ebaMVpvsProUGQFJnjoA6CRDwc1XVLR7q+MY3sOpLyAZUTBOulTzef5nNfwkXnaob1MVm8vr4+7aBI9XOrp1pwbrUwnQlzldRcJ7AfsoIrtI5nmt+po3wU4r300kuBieYf1dPf3x+qnDxPLlKSktPoxomnkouX65J41MEoH42MNBeMgqFKPFJIVDhc+sGlHZIPiceVmuNESkDTA5IblQqDQ9wzQQKorjAnoM5Co\/KhQoYNMqqLT22wIbdp7qsjXlgfq+Xgf3NvhAY0jovJ6mL71b7k2JBOqy40GZeAeNgA\/F+uRfEoozV2YJjF4w1SRzhbcqmRS1uLp3NvXBIPZSMZxo8fn3Ez+e9kmcIGL5V4vKMRX24Folg8jn1YwIHPlfgck4IrOlctzLrbBqRcWDxdH4cRT\/U2VFLqIuZRLJ5KwEgL6G+99RYsWLAgcPUWL16cRSw+ouHoRz58lDmeaW7AG636\/7pRBsvRWTx1lMJRiXz8xx57LGv0I3eD2qQCn0uJuNXg7olK8ChzPJ2sFFzgczyS5dKlSxnXM9ccj6ylSmiV\/CYXlhSXrB8FUngENM05HsnD+1h1q1VyqXNbDKCFuZp8jqdibjXHi7JlDAmFkcKLFy9mRS35aDJ69OjA9VDdiFxRTRviIXmiRDW5q4n\/3xTxImtE0UqKXpmIx2XRrRuq8wnuTfD2c7deHZGxTRSBRYLxSCfmQ2uKH3djfUQ1eeSQtx3dJvwIM+xvJDtFhtW0PACD+aJENXWDl2k5IVdUk3RCJZ7aL4ivGrxS+9prVFM1k7r\/RiFWrFgBJ06cgP3798ODDz6YlUwFQ\/Z82qBqnyaqRdZ5ERh1db3xwF6C4kuZBHOr5QQbSNBqoinHf9EdUYmHv69ZswY2bNhgtcvApk5J8wUC6sCGf4mygwf7zHbniuA+iEBczLN2riQFEzsO3QxcO9KRi69B5QriJG2L5BcECh0BJxYPSbVv3z5Yvnw5bNq0SUs8vvaEoITtTCl00KR9gkBSBBITD83t1q1bYeHChcHuDxt3koI5MqdI2n2Sv1gRSEw8NcKIQOTaYE2+cXV1ddam3HvuuadYcZR2D1MEjhw5ApMmTXIu3RDi8QkjhbBN7qPamrAACrqa+NFGawyN79ixIyvQgsQ7c+aMcyFLrUDB0V2P+8Iyi3h8q1NNTU0wb9u4cSPgBk+biJdKPE62sN3xBJMvId11Q3GUdPbsWS+jdHFI77aVvnQyi3j8agdc8SfiIWnSuPLBl5Buu6LwSxPiuesjXzo5xNWkHeO4JezgwYNBpBLPhen2vrkTb7AkX0K6bmehlyfEc9dDvnRSG1xRDzGmFfr3JaS7biiOkoR47vrJl04mjmq6E1EsnisshXiukPSnk0I8d31UMCUJ8dx1RSoWL9cphVzrc0nF9SVk0nYVW34hnrse86WTQyweLgH09vZm3d5Evz3yyCNAd1Lwo\/quxPQlpKv2FUs5Qjx3PeVLJ43LCXwjMy0z4GFHPOmc68bpuGL7EjJue4o1nxDPXc\/50kntAjq\/nYq2hE2bNg2eeOIJeOWVV6wuPY0jui8h47SlmPMI8dz1ni+dtFpOoKscbDZAJxHZl5BJ2lSMeYV47npt4oz58H7nP7kr8POSJKrpHNL8FyjEc9MH7f91EVa0vwOXfzTTTYGsFCGec0jzX6AQz00fpEo89cAqiZDGHSniarpRGCGeGxxTIx4\/XYAX0OLyAd2aTNd0uxFJX4oQzw26Qjw3OKZKPDqFgHfm03qe6wcpcSM2fupLL0I8NwojxHOD4\/bDvbD98Fn\/czx+Mvyhhx4K7nHEw6r47gE9wpH0oiLd4w8EkxDPjcII8dzgmBrxsLncuqHVo8tKo1wVZxKbysaXe65duyYWz41+DClFiOcG2FSJ56bJ+lLQxcR5I74vp25Lwxxi8dygL8RzgyMuJeA8z\/tygmku52KOhy4mPlaJ8zrdflAiHl4uI18yBPC9A7y7X75kCHxr82H47bj7\/REv16kEbH7YPfc24vE3xyg9f+RPLJ4NinZpxOLZ4RSWqu\/yp\/C7f\/ufQZK8WbzkYmSXEGbx5Jax5GgL8ZJj2NlzBWqeOZkO8ZI3164EIZ4dTnFTCfHiIjeY741fXYW5PzkR\/P8vXf8IPnrmz5MVqMmtfRFWV4vsXHGOvbcChXjxof2H1\/pgy7+8lyng1peXeLnrVfZqxu+jgs0pxIveNat\/9kvYf+xCJmNF+UjY8+TXYMHjDwnxosNZmjmEeLn7HYMnP+06D7te6xuSGEl36K+mAv7ra4kr9Ap3alHSiGZuGAZT+BLStv7hkk6IN7QnkWgvn\/wQfv5OP7zx3lVtV3PCUQJfOmm8wp3vo6RLbpubm8HHXSu+hRwuhLKVQ4gHsOrAu3Cu\/4aRZIQlku3A974JD9xxsxbeVIjncwHdRml8CWlT93BKUwrEQwt2\/uqnsO3fzgZdZ7Jiar8i0WZUlcMPvn134Erm+nzppPYK946ODmhrawte8qE7V0wPvedqeJS\/+xIyShuGQ9rhQDwk1gAA7HrtHJz+4Dr0XfkU8LcoHxJr+r1jYE\/dA0FeG6Kp5fvSSW1UM1+vt\/oSMkpnDYe0xUC8gQGAv3u1F944fSWSxdJZMPxt+n1jYPWfVsB9t41y2oW+dFKWE5x2U2EUlg\/icWv05ntXAf\/X138jlqXSkatizMiAVKtmVsCkcWWpAS3ESw3q4q\/IJfGIUFeu\/wb2HbsAv\/rgeiILZbJYSKw\/njwGfvh4ZUF1QCrEo83SFRUV3u7ODEPVl5AF1ZMpNMaGeJ8NAPzzL\/4Hjp29Cr++\/KkTy2SyVHeVj4SaqbfDow+UpyC92yp86aTVOp56isCtaF+U5ktIX+0thHLJIn02MABH3r0MJ3\/9Cbx7\/gpcugGRgxE28lCAAi1U5bgyqP29O+CuMYPRwTjBC5s685nGl05azfHwLB2u5bW2tkLSqx\/E4pkR4POkC1f\/F1448QH0XLwWZIgT1bNVWE4mnD8t+qM7oXzUV4YtmWxxwXSpEU99qxwrz\/VKEH\/I0rTLRS1Xt+nal5BRgPaR9ujpK\/DyiQ\/hvUuD8yOfJAr665YRMGLECECrNPn2UTD76+Pg\/tu\/WCAejpbJR7+lRjya42GFttaNXwmIp54bGxuhuroaamtrs7Dg6XB9UPcVIvG4FfrN\/30Gh0\/1w7sXr0HvRzdSIRG3RljhtLtvge\/\/yV1w+y1fNeqazRzPl6IOt3J96aSVq2kLJlm1urq64D5O\/uFCfFNTE7S0tBjdVV9Cqu0nMuG\/Lxz\/AM6kZIlUEmF4\/DtTb4O7y78Ij7uwRkI8W43Nnc6XTjojHrmbJlfTZlHepZBIqt8Z+WX4i7b\/9uLaqUGG+8aPgllfHwtlX\/lypjddkCi3agxNIcSLg5o+j0ud5DU4Ix4VigTr6uoKXY4glxY3YnPLmFRInEP99c9+ab1vj9rMSXT32DJ4pGpMEP7+0k3uOjDNkoR47tBOqpOmljgnnk0ElF+cy+eCKGScW8b+8qWL8PZ5\/T4+DDTg93BFGcyfeguM+JxN9Lu7LiqckuSWMTd98eijjwYF+bgHKNKLsLqXYNUTDabr2dES4odEw\/ke3VLNAy1RR5e\/\/\/k5+Jt\/PZOFMlqvud8cD9+bPnFYrivZqJRYPBuU7NJE1Um7UgGs71wJOwxrWk7gZFOXE7Zt2zYk8mkrJM7f8DIaHnHUHWK0BWG4pRPiuetRW52MWqOVxYtaaNz0NkLy+w6xHiTcyU1\/GLfKYZlPiOeuW210Mk5tzud4cRpBeWyExEtGydLhTvUtc+5NUuWwzCvEc9etNjoZp7aMq4nPc9XX10NDQwN0d3cPKSvf1\/sh2Va2v5OJWK59vBIaZ0+KI\/OwzyPEc9fFXonnrpnJSgoTkruY4l6G4yzES6aHPHfJE6\/8B\/8R4CGky61UQrzcGNmmSIV4YY+X5NPVbH3zPPzwxZ4Aqx\/XfQ3qfv8OW9xKMp0Qz123p0I8U3PpXTt1\/6U78QZLMgkp1i4a0kK8aHiFpc4r8Vy8j2cDhU7I0x9eh281\/yLIjrf7Tr\/vVpuiSjqNEM9d9+eVeDbbwFyIqhNSrF10ZIV40TEz5UiFeGFzPBdvoOeCQxWSP5f04rIpMPP+4ruzI5fMPv4uxHOHairEc9fceCWpQs798YnMuh3uTsnXMZt40uQvlxDPHfapEU\/dwIz7LQ8cOGB9Ij2JyKqQ4mbGQ1OIFw83Xa5UiGc6QW5zxs6FqFxIvmAuSwjR0BXiRcMr71HNQnq0BN8ua3hhcO1O3MxoiiTEi4ZX3omHDUDrtnv37syjJRRwwTU8\/nRXVNHUY0G6YA23eDS\/k50qUZEGEOJFxyyvUU2qnF4IunBh8Gla3dm5qKIhoXt7ewPympYniHjczZxZVQ4vfn9K1OpKOr0Qz133pzLHc9fc8JKQeO3t7UPuZSEh+TKCLJpH7xUhXnTM8mrxfO9Q4e5mmKvJH4KX+V10JRLiRccsr8TDynFfZmVl5ZBrGdyJAhB2yxhedkSXF+GFRB0LJ7qsuiTKksuO3HRzqpcdLVmyxPtB2LBbxjrfOgV4yhy\/6ffeCodWTHWDYgmVIhbPXWcX\/RzP9pYxTjyZ38VTICFePNx0uYqeeLbLCfv\/\/Xhwgxh+Mr+Lp0BCvHi4pU48CqoUwp0r31jzYmZ\/5uUfzXSHYAmVJMRz19lFb\/FsoEAhiXiycG6DmD6NEC8+dmrO1IiXz03Sld\/4A\/j48e0SWEmoN0K8hACy7KkQL9+bpDnx1s2aBOtmFdZD9O66029JQjx3+KZCvHxvkubEkxMJ8ZVHiBcfu7y5mr42SdtAgaPL1T9rDZLKUoINYjLHi4+SXc5ULB41xccmaRsxb1vwDPx23P1BUolo2iAmxIuPkl3OVIln1yT3qYh4EtFMhq24msnw47lLgnh01YNsFUumOEK8ZPiVLPEWVd8JLd+tcodeiZUkxHPX4SVl8SSwkkxxhHjJ8CtZiyfES6Y4Qrxk+OWFePxZZd6ANB4toTmeEC+Z4gjxkuGXOvFMB1RziYFrf+vXrw+SmQiqnk7QpSPiyamEXIiH\/12Ilwy\/vBAPX4bdvHkzlJfbXZeOa35NTU3Q0tIS5MET7HhJUnNzM5SVlWVkQFKvWbMGNmzYAJMnT9Yig8STpYTkSiPES44hlZBacIXfBhan+aaLjFSC6soW4sVBfGgeIZ4bHLGUVIjn4mFK050t3B1FgXRXBiLxZA0vudII8ZJjmLrFS9JkW2tpmksi8fC1V9wgLV98BIR48bFTc6Zi8ZI0N8rtZKbLjpB4\/1hzB0y7c2SSppR8XrllzI0KpHbLGDaXRx\/nzJkDa9euhU2bNoUGRWyeara57AiJN+r4T+GrfW+6QU5KEQQcIHDmzBkHpWQXcdPAwMAA\/USkmzBhAtTU1MC+fftg48aNcOjQIejq6hoSqcR8unU\/JCxGNTEffrW1tVmENs3xnEsnBQoCBYpAFvH4Qdj+\/v4M8ZCQUZcZClReaZYgUBAIZBEPW0TrcIsXL4aDBw\/C8uXLYeXKlZD0taCCkFYaIQgUCAJDiKdzH128FlQg8kozBIGCQEBLvLRbxtf4dI+ZpN2eYqmPz69Ng6N6m8CyZcsSvXNYLNi4aqfNjqs4deWdeHxHS09Pj\/b5rjiCDfc8XCFQVr5tj8tu2kk03PFxIR8NWlhWW1ubcatjnLqGEE\/dzIyFUpSS772MU5kuD39fHevOtZ\/TVb3FXg5\/3BP7pbGxEerq6oK5OP9sNzUUOx6u248D27PPPguzZ8+Gp59+Gnbs2OGPeHw5gZ5dpt9QMHXjswthuWLEPR3hoh3FVga3ZNh2JF51dXXW82rqIIrLRK5H7mLDLWp71Queo+Y3pTcuJ\/DTCT4frBTixetKG+KpJZuewI7XgtLIlQrxEErdnCDKdrCo3SGuZlTEBtPbupq8dF9KFE+C4sjlC7MhFs\/0MCXB5PokugRX4imgTXAFXc2tW7fCwoULg\/kJH+R8zNfjSVLYuVIhXr4goOUEmYNE6wG+nMCXYdR9sYsWLQoOJwu+0fDF1MOaeNHhkByCQHEjkPflhOKGT1ovCMRDwHg6gZYTsFjTPSrxqpRcgoAgkPflBOkCQaAUEdCeTujo6MgstNK2Gdy9wq1gKYIlMgsCrhDQ7tW0uZjIVQMKrRzbLVY2t6ZFlY1vaLbdLF5IezEpyup6yYlwpPJ9bmGM2mdx0+d9k3TchvvKl0\/ioWIdPXo0kmdRaMRrb2\/3srWQ+hsHJ7oZoZjXIkuWeHwNjEZoPB0xb968oI\/p+Ix6tQVaoqqqKqCNBpSXNiqjm45fmMXiZdLo3d3dnanbNKJzT4SOARHxRo8eHdSpWhueh6\/jYcDs1KlT8Prrr2euWuTrqTNmzAAsk6YXujariq8OAljHzTffDEeOHAGUz3QkSfUewgYTIZ4vk5NCueoZK77gzC2eunjKd37gTV7qDdrYdFRUVJyGhgbthmRyJ3fu3BmQBDc3IyEon8licGXkpzjwig4cLIh0anmo\/K2trZlbvqmNaqSayzp27NhgYKFbB\/jfJk6cmNVm3l064lG8gMpEOdUTFEK8FJS+EKoIO9wY5mpyxeLEQ5lQUUmp6FSA7piOqpx8z2XYeUTTfll143NY+\/nfsDwiIf6r5uP\/rW41M1kkHfHC6uDuIx\/ExOIVAks8tYEHMrhrpyogKujevXszraC0OuKhO8U\/3alwVYlt9qqa7iHFulQl5e3Xna0kd08lchgR1WAb1qsLoOiIV1lZmTmqZBoUxOJ5UvJCL1Yd3Xt7ezOuH3fVwiye7QFeHxaPu6dhlkq1eGGkMGES1pe5LJ5KbpPFC9vMLXO8QmdTSPvUEdY0x9MdvcFi8UBw2ByPz+N08xnctBx1jqcenyLXFttjQzy0fnzeplo82zkennIw7WTSEQ9\/wzmm6o7z7tHNewlnNYAjxCti4tGcht70464mRe\/QJauvrw8CCRggwAAIPjGGVx7ik2SkSPgvKpYa1Qy7mc0UIcy1NMDdXjWqSbcD6A4WowuMruHSpUvh8OHDwcCxa9cu4BaPY4Jp8frya9euaaOapnU6HfE++eQT6OzsDE5HcEx0c0rsD8QZB4iTJ09qBzghXpETT5pvRiBsThnV1VTJnRR3IV5SBCV\/QSGgrlfGuQaQyiCLiJcFuSQelS87VwpKdaQxgkDxIPD\/\/025a1OGz0AAAAAASUVORK5CYII=","height":134,"width":222}}
%---
