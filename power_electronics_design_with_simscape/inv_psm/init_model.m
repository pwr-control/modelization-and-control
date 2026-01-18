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
model = 'inv_psm';

use_mosfet_thermal_model = 0;
use_thermal_model = 1;
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
run('n_sys_generic_1M5W_pmsm'); %[output:0db165d8] %[output:255f588d]
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
use_dq_pll_ccaller_mod1 = 1; % only module 1
use_dq_pll_ccaller_mod2 = 0; % only module 1
%[text] ### Settings for CCcaller versus Simulink
use_observer_from_simulink_module_1 = 1;
use_observer_from_ccaller_module_1 = 0;
use_observer_from_simulink_module_2 = 1;
use_observer_from_ccaller_module_2 = 0;

use_current_controller_from_simulink_module_1 = 1;
use_current_controller_from_ccaller_module_1 = 0;
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
fPWM_AFE = 4e3;
tPWM_AFE = 1/fPWM_AFE;

dead_time_AFE = 3e-6;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;

ts_afe = 1/fPWM_AFE/2; % Sampling time of the control AFE as well as INVERTER
tc = ts_afe/300;

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
deepPOSxi = 0.5 %[output:57086c1f]
deepNEGxi = 0 %[output:08bc4522]
deepNEGeta = 0.5 %[output:858ca950]
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
l1 = Kd(2) %[output:55bd5b41]
l2 = Kd(1) %[output:079a03a1]

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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:72559c6d]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:8c11ae60]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:95cb5289]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:42d1bda9]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:85f7f9dd]
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

ts_inv = 1/fPWM_INV/2;
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
kg = Kobs(1) %[output:7c2251f1]
kw = Kobs(2) %[output:76a3d4ad]

%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:5713da6e]
luenberger_l2 = Klo(2) %[output:7c389dc3]
luenberger_l3 = Klo(3) %[output:8886a72c]
omega_flt_fcut = 10;
% phase_compensation_omega = -pi/2-pi/12; % for motor mode
phase_compensation_omega = 0; % for generator mode
%[text] ### Control settings
id_lim = 0.35;
%[text] #### rotor speed control
kp_w = 2.5;
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
%[text] #### Field Weakening Control 
kp_fw = 0.05;
ki_fw = 1.8;
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
heatsink_liquid_2kW;
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
igbt.afe.Cies = Cies;                                % [F]
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
figure;  %[output:29a5e80b]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:29a5e80b]
xlabel('state of charge [p.u.]'); %[output:29a5e80b]
ylabel('open circuit voltage [V]'); %[output:29a5e80b]
title('open circuit voltage(state of charge)'); %[output:29a5e80b]
grid on %[output:29a5e80b]
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
for i = 1:length(open_scopes) %[output:group:0a38cd65] %[output:37440596]
    set_param(open_scopes{i}, 'Open', 'off');
end %[output:group:0a38cd65]

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);

%[text] ## Enable/Disable Subsystems
% if use_mosfet_thermal_model
%     set_param('inv_psm/inv_psm_mod1/inverter/inverter/three phase inverter mosfet based with thermal model', 'Commented', 'off');
%     set_param('inv_psm/inv_psm_mod1/inverter/inverter/three phase inverter igbt based with thermal model', 'Commented', 'on');
%     set_param('inv_psm/inv_psm_mod1/inverter/inverter/three phase inverter ideal switch based model', 'Commented', 'on');
% else
%     if use_thermal_model
%         set_param('inv_psm/inv_psm_mod1/inverter/inverter/three phase inverter mosfet based with thermal model', 'Commented', 'on');
%         set_param('inv_psm/inv_psm_mod1/inverter/inverter/three phase inverter igbt based with thermal model', 'Commented', 'off');
%         set_param('inv_psm/inv_psm_mod1/inverter/inverter/three phase inverter ideal switch based model', 'Commented', 'on');
%     else
%         set_param('inv_psm/inv_psm_mod1/inverter/inverter/three phase inverter mosfet based with thermal model', 'Commented', 'on');
%         set_param('inv_psm/inv_psm_mod1/inverter/inverter/three phase inverter igbt based with thermal model', 'Commented', 'on');
%         set_param('inv_psm/inv_psm_mod1/inverter/inverter/three phase inverter ideal switch based model', 'Commented', 'off');
%     end
% end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":26.9}
%---
%[output:0db165d8]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:255f588d]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:57086c1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepPOSxi","value":"   0.500000000000000"}}
%---
%[output:08bc4522]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGxi","value":"     0"}}
%---
%[output:858ca950]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGeta","value":"   0.500000000000000"}}
%---
%[output:55bd5b41]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:079a03a1]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:72559c6d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:8c11ae60]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:95cb5289]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:42d1bda9]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:85f7f9dd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.194386045440868"],["33.957607642498068"]]}}
%---
%[output:7c2251f1]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:76a3d4ad]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:5713da6e]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:7c389dc3]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714302e+02"}}
%---
%[output:8886a72c]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.994503273143434e+02"}}
%---
%[output:29a5e80b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAN4AAACGCAYAAACyujQtAAAAAXNSR0IArs4c6QAAGJlJREFUeF7tXX2QltV1PybUYTE4soBSxHUx7hozsQRsG7tlIsQo\/gM2tJ11mREKDCUU2DqyheXDotOyLAybzADphDqbLczUxaK27k47gw4tlGS1rYLbaTEBxYXgFoMLVGfFTNJs5zzreT3vfe993vs8z73P+3WemQzx3ft1fvf87jn33K\/rRkZGRi5fvgzLly+H\/v5+yPfNmDEDOjs7obq6Ol9S+bsgIAgYELiOiPf000\/D1q1bQwmFBLVJJ2gLAoJAOAIB8QQkQUAQSBeBjMVDVxO\/UnYjX3vtNVi0aBE8++yzcN999zlFUi372rVrsHv3blixYoUztxvLbG1tDdrd3t4OVVVVgPWeO3cOGhsbncqjFvbcc89BX19fpt6wyvbu3Qvz5s2Durq6vG2KkjZvYZ8mIJx6e3th6tSp0NXVZdUWHb62dbpMh55jxuJxYbCSKAK5bFSplLVjx46AFC4HKlUxLly4AEuXLoW1a9d6JR7N8R999NG89SBB9+zZY6XsUdJG6fczZ84EuMyfPx82bNhgnbVYiIcNNrqaCNrGjRszQm3fvj3oFBJ6cHAw+BtZFxLqww8\/DH4\/duxYDnnJauDfeZCGfn\/yyScDRcayqT4dqqY2cKuE5aP1oPZgHhwZp02bFvyOoyV+K1euDDqPyiQlVy0c\/2+0QISNOkDpOlclaT4MsV3r16+HNWvWZAJe1E7eL2rdWM++ffsC3OfMmQNHjx7NEEQdWDm+KkHUYBul5f1HfV9fX58VmKN26tKiF8Lbj8Qhy672s6kN6u+6MlRZqU06HVWDhYShSUexLMSYygzDXG0r98Ss5ngUVHnsscfg8ccfz4zAvMNIod94442gsydOnBh0SE1NTQAuH70XLFiQ5VJhNBVdRC6MyZqoozNX6tOnT2dcTSIetYfcIkyPxMY2Ub0ICLaXW5cw4qEChVk8xOXgwYPBIIIf4oB5dATXYUiupmrxkLBtbW3Q0dGRKZfwJVmQJIQvl92EE8lCmKB7y9O+8sorWRZOJSmmra2tDQZlIhUpmJqWY0qEJVw48aiP6W9qX+SzeKY+VnUC61T7vLu7O0tfyapSG0hHMS\/9psOc+EB92dPTk4WjlngqU2lkPXHiRFZmDlBzc3PO\/IQDr+blykmEoZE1zEVBYVtaWrSujs7ikeCoUGEuVRSLl494VNauXbsCfeLzTpN1wTJVDE2ups5q4HwTrTjNd3g9NAiSInMc1EGQiIejum7pSNc3ppFdR1I+oHKCYL306ebTfO5LuOhczbA+Jot3\/vx57aBI9XOrp1pwbrUwnQlzldRcJ7AfsoIrtI5nmt+po3wU4r344ouBieYf1TM0NBSqnDxPPlKSktPoxomnkouX65J41MEoH42MNBeMgqFKPFJIVDhc+sGlHZIPiceVmuNESkDTA5IblQqDQ9wzQQKorjAnoM5Co\/KhQoYNMqqLT22wIbdp7qsjXlgfq+Xgf3NvhAY0jovJ6mL71b7k2JBOqy40GZeAeNgA\/F++RfEoozV2YJjF4w1SRzhbcqmRS1uLp3NvXBIPZSMZJk+enHEz+e9kmcIGL5V4vKMRX24Folg8jn1YwIHPlfgck4IrOlctzLrbBqRcWDxdH4cRT\/U2VFLqIuZRLJ5KwEgL6K+\/\/josXrw4cPWWLVuWRSw+ouHoRz58lDmeaW7AG636\/7pRBsvRWTx1lMJRiXz8Bx98MGv0I3eD2qQCn0+JuNXg7olK8ChzPJ2sFFzgczyS5dKlSxnXM98cj6ylSmiV\/CYXlhSXrB8FUngENM05HsnD+1h1q1VyqXNbDKCFuZp8jqdibjXHi7JlDAmFkcKLFy9mRS35aDJ+\/PjA9VDdiHxRTRviIXmiRDW5q4n\/3xTxImtE0UqKXpmIx2XRrRuq8wnuTfD2c7deHZGxTRSBRYLxSCfmQ2uKH3djfUQ1eeSQtx3dJvwIM+xvJDtFhtW0PACD+aJENXWDl2k5IV9Uk3RCJZ7aL4ivGrxS+9prVFM1k7r\/RiFWr14NJ0+ehAMHDsA999yTlUwFQ\/Z82qBqnyaqRdZ5ERh1db3xwF6C0kuZBHOr5QQbSNBqoinHf9EdUYmHv69btw42bdpktcvApk5J8xkC6sCGf4mygwf7zHbniuA+ikBczLN2riQFEzsO3QxcO9KRi69B5QviJG2L5BcEih0BJxYPSbV\/\/35YtWoVbNmyRUs8vvaEoITtTCl20KR9gkBSBBITD83ttm3bYMmSJcHuDxt3koI5MqdI2n2Sv1QRSEw8NcKIQOTbYE2+cUNDQ9am3DvuuKNUcZR2lykCR44cgenTpzuXLod4fMJIIWyT+6i2JiyAgq4mfrTRGkPjO3fuzAq0IPHOnj3rXMhKK1BwdNfjvrDMIh7f6rRw4cJg3rZ582bADZ42ES+VeJxsYbvjCSZfQrrrhtIo6d133\/UySpeG9G5b6Usns4jHr3bAFX8iHpImjSsffAnptiuKvzQhnrs+8qWTOa4m7RjHLWGHDh0KIpV4Lky3982deKMl+RLSdTuLvTwhnrse8qWT2uCKeogxrdC\/LyHddUNplCTEc9dPvnQycVTTnYhi8VxhKcRzhaQ\/nRTiueujoilJiOeuK1KxePlOKeRbn0sqri8hk7ar1PIL8dz1mC+dzLF4uAQwMDCQdXsT\/Xb\/\/fcD3UnBj+q7EtOXkK7aVyrlCPHc9ZQvnTQuJ\/CNzLTMgIcd8aRzvhun44rtS8i47SnVfEI8dz3nSye1C+j8diraEnbvvffCI488Ai+99JLVpadxRPclZJy2lHIeIZ673vOlk1bLCXSVg80G6CQi+xIySZtKMa8Qz12v+dJJiWq666OiKUmI564rhHjusCz7koR47ro4NeKpB1ZJhDTuSPElpLtuKI2ShHhu+qn7Py7C6u634PJ35ropkJWSE9WkeRxeQIvLB3RrMl3T7bwFrEAhnht0hXhucEyVeHQKAe\/Mp\/U81w9S4kZs\/NSXXoR4bhRGiOcGx9SIx0+Gz5o1K7jHEQ+r4rsH9AhH0ouKdI8\/EExCPDcKI8Rzg+OOwwOw4\/C7\/l1NbC63bmj16LLSKFfFmcSmsvHlnuHhYbF4bvQjpxQhnhtgU7N4bpprLgVdTJw34vty6rY0zCUWz00PCPHc4IiBFSRfKsEV3UlzF3M8dDHxsUqc1+n2gxLx8HIZ+ZIhgO8d4N398iVD4GtbD8MvJ93lj3j5TiVg88PuubcRj785Run5I39i8WxQtEsjFs8Op7BU5y9\/Al\/9y1eDJAWzeMnFyC4hzOLJLWPJ0RbiJcfw+NtX4ZG\/OpkO8ZI3164EIZ4dTnFTCfHiIjea74dvX4UFn5Lucx9\/AB98\/w+TFajJrX0RVleL7Fxxjr23AoV48aHde\/Sn8Oc9b2cKuOkflnu561U2Scfvo6LNKcSL3jVPHPoJ\/M2rg5mMNdVjYe+jd8Pih2YJ8aLDWZk5hHj5+x2DJ\/tfHYTvHjmXkxhJ1\/MnMwH\/9bXEFXqFO7UoaUQzPwyjKXwJaVt\/uaQT4uX2JBKt5z8vwcv\/\/QH88J2r2q7mhKMEvnTSeIU730dJl9y2t7eDj7tWfAtZLoSylUOIB7D24I\/h3NA1I8kISyTbs8vvgS\/\/+he08KZCPNNCuYsFdBul8SWkTd3llKYSiIcWbPB\/fw5t\/zT6yI3Jiqn9ikSbU18NT3zz9sCVzPf50kntFe69vb3Q1dUVvORDd66YHnrP1\/Aof\/clZJQ2lEPaciAeEmsEAHb\/83k48\/4wnL\/yCeBvUT4k1uwv3gR7m+4O8toQTS3fl05qo5qFer3Vl5BROqsc0pYC8UZGAHa9MgDHz1yJZLF0Fgx\/m33nBPjTb9RA3c3jnHahL52U5QSn3VQchRWCeNwa9b1zNXD9zg9di2WpTOT6xl3VsGZuDdwxqSo1oIV4qUFd+hW5JB4R6srHv4ADrw3Cmfc\/TmShTKSqmTAWZtdNgPUP1RZVB6RCPNosXVNT4+3uzDBUfQlZVD2ZQmNsiIeu3t\/++\/\/Aq2evwk8vf+LEMulIhYS6rXosfGvmLfDNL1WnIL3bKnzppNU6nnqKwK1on5XmS0hf7S2Gcski\/WpkBI78+DL0X\/gI3rpwBS5dg8jBCBt5KECBhKqdVAWNvzkFbpswGh2ME7ywqbOQaXzppNUcD8\/S4VpeZ2cnJL36QSyeGQE+Txq8+nN4\/uT7cPricJAhTlTPVmFVMi373VuhetyvlS2ZbHHBdKkRT32rHCvP90oQf8jStMtFLVe36dqXkFGA9pH2X89cgb8\/+TN459Lo\/MgniYL+unEMjBkzBtAq1d0yDuZ9eRJ8acoNGdHK0TL56LfUiEdzPKzQ1rphHroSEE89t7a2QkNDAzQ2NmZhwdPh+qDuK0bicSv0i\/\/7Fbx8agjeujgMAx9cS4VE3BphhbNuvxG+\/fXbYMqN1xt1zWaO50tRy61cXzpp5WragklWrampKbiPk3+4EN\/W1gYdHR1Gd9WXkGr7iUz47\/Mn3oezKVkilUR33jwOvvXVm+H2iZ+Fx11YIyGercbmT+dLJ50Rj9xNk6tpsyjvUkgk1RfGfh7+qOu\/vLh26rzozsno0k2Equs\/X3CXToiXn1C2KVzqJK\/TGfGoUCRYX19f6HIEubS4EZtbxqRCouV6\/O9+Yr1vj9rMSYTW5+v1E+D3Z94Cn7vOtnuKK50Qz11\/JNVJU0ucE88mAsovzuVzQRQyzi1jf\/ziRXjjPf0+Pgw04HdfTRU8NvNGGPMpm+h3d11UPCXJLWNu+uKBBx4ICvJxD1CkF2F1L8GqJxdM17OjJcQPiYbzPbqlmgdaoo4ueIjxL\/5xdHc6t17zf2MyrJg9rSzXlWxUSiyeDUp2aaLqpF2pANZ3roQdhjUtJ3CyqcsJ27dvz4l82gqJ8ze8jIZHHHWHGG1BKLd0Qjx3PWqrk1FrtLJ4UQuNm95GSH7fIdYjhMtFW4gXVwNz89noZJzanM\/x4jSC8tgIiZeMkqVbO7cGnp7\/xSRVlmVeIZ67brXRyTi1ZVxNvLq9ubkZWlpaoL+\/P6esQl\/vh2Rb0\/1WJmKJu9hbH54eR+ayzyPEc9fFXonnrpnJSgoTkruY6F6+ueV3klVWxrmFeO46t+KJV\/3EvwRoCunyK5UQLz9GtilSIV7Y4yWFdDU7f\/Qe\/NkLpwOsvtd0NzT91hRb3CoynRDPXbenQjxTc+ldO3X\/pTvxRksyCSnWLhrSQrxoeIWlLijxCnm935mffQxfa\/+3ABu83Xf2nTe5Q7VMSxLiuevYghLPZhuYC1F1Qoq1i46sEC86ZqYcqRAvbI7n4g30fHCoQvLnkl5YOQPm3lV6d3bkk9nH34V47lBNhXjumhuvJFXIBd87mVm3w+UDF2fV4rWstHIJ8dz1V2rEUzcw437LgwcPWp9ITyKyKqS4mfHQFOLFw02XKxXimU6Q25yxcyEqF5IvmMsSQjR0hXjR8Cp4VLOYHi35Qd970PL86NqduJnRFEmIFw2vghMPG4DWbc+ePZlHSyjggmt4\/OmuqKKpx4J0wRpu8Wh+JztVoiINIMSLjllBo5pUOb0QNDg4+jSt7uxcVNGQ0AMDAwF5TcsTRDzuZs6tr4YXvj0janUVnV6I5677U5njuWtueElIvO7u7px7WUhIvowgi+bRe0WIFx2zglo83ztUuLsZ5mrihUX4QIbM7+IpkBAvHm4Fi2pixbgvs7a2NudaBneiAITdMoaXHdHlRXghUe+SaS6rroiy5LIjN92c6mVHy5cv934QNuyWsaOvnwI8ZY4fvubZs3qmGxQrqBSxeO46u+TneLa3jHHiyfwungIJ8eLhVlBX012Ts0uyXU448PKJ4AYxmd\/F7wkhXnzs1JxeLR4FVYrhzpWvrHshsz\/z8nfmukOwgkoS4rnrbK\/Ec9fMZCWhkEQ8WTiPj6UQLz52qVo8XlkhN0nXfuW34cOHdkhgJaHeCPESAsiyp2LxCr1JmhNvw7zpsGFecT1E7647\/ZYkxHOHbyrEK\/QmaU48OZEQX3mEePGxK5ir6WuTtA0UOLpc\/b3OIKksJdggpk8jxIuPXcGIhxX72CRtA8XNi78Pv5x0V5BUIpo2iAnx4qNklzMVV9OuKf5SEfEkopkMY7F4yfDjuSuCeHTVg2wVS6Y4Qrxk+FUs8ZY23Aodf1DvDr0KK0mI567DK8riSWAlmeII8ZLhV7EWT4iXTHGEeMnwKwjx+LPKvAFpPFpCczwhXjLFEeIlwy914pkOqOYTA9f+Nm7cGCQzEVQ9naBLR8STW8XyIR7+dyFeMvwKQjx8GXbr1q1QXW13XTqu+bW1tUFHR0eQB0+w4yVJ7e3tUFVVlZEBSb1u3TrYtGkT1NXVaZFB4slSQnKlEeIlx5BKSC24wm8Di9N800VGKkF1ZQvx4iCem0eI5wZHLCUV4rl4mNJ0Zwt3R1Eg3ZWBSDxZw0uuNEK85BimbvGSNNnWWprmkkg8fO0VN0jLFx8BIV587NScqVi8JM2NcjuZ6bIjJN5fL5wC9946NklTKj6v3DLmRgVSu2UMm8ujj\/Pnz4f169fDli1bQoMiNk8121x2hMQbd+IHcP35H7lBTkoRBBwgcPbsWQelZBdx3cjIyAj9RKSbOnUqLFy4EPbv3w+bN2+Gnp4e6Ovry4lUYj7duh8SFqOamA+\/xsbGLEKb5njOpZMCBYEiRSCLePwg7NDQUIZ4SMioywxFKq80SxAoCgSyiIctonW4ZcuWwaFDh2DVqlWwZs0aSPpaUFFIK40QBIoEgRzi6dxHF68FFYm80gxBoCgQ0BIv7ZbxNT7dYyZpt6dU6uPza9PgqN4msHLlykTvHJYKNq7aabPjKk5dBSce39Fy+vRp7fNdcQQr9zxcIVBWvm2Py27aSVTu+LiQjwYtLKurq8u41TFOXTnEUzczY6EUpeR7L+NUpsvD31fHuvPt53RVb6mXwx\/3xH5pbW2FpqamYC7OP9tNDaWOh+v248D2zDPPwMMPPwxPPfUU7Ny50x\/x+HICPbtMv6Fg6sZnF8JyxYh7OsJFO0qtDG7JsO1IvIaGhqzn1dRBFJeJXI\/cpYZb1PaqFzxHzW9Kb1xO4KcTfD5YKcSL15U2xFNLNj2BHa8FlZErFeIhlLo5QZTtYFG7Q1zNqIiNprd1NXnpvpQongSlkcsXZjkWz\/QwJcHk+iS6BFfiKaBNcAVdzW3btsGSJUuC+Qkf5HzM1+NJUty5UiFeoSCg5QSZg0TrAb6cwJdh1H2xS5cuDQ4nC77R8MXUZU286HBIDkGgtBEo+HJCacMnrRcE4iFgPJ1AywlYrOkelXhVSi5BQBAo+HKCdIEgUIkIaE8n9Pb2ZhZaadsM7l7hVrASwRKZBQFXCGj3atpcTOSqAcVWju0WK5tb06LKxjc0224WL6a9mBRldb3kRDhS+T63MEbts7jpC75JOm7DfeUrJPFQsY4dOxbJsyg24nV3d3vZWkj9jYMT3YxQymuRFUs8vgZGIzSejli0aFHQx3R8Rr3aAi1RfX090EYDyksbldFNxy\/MYvEyafTu7+\/P1G0a0bknQseAiHjjx48P6lStDc\/D1\/EwYHbq1Ck4fvx45qpFvp46Z84cwDJpeqFrs6r46iCAddxwww1w5MgRQPlMR5JU7yFsMBHi+TI5KZSrnrHiC87c4qmLp3znB97kpd6gjU1HRUXFaWlp0W5IJndy165dAUlwczMSgvKZLAZXRn6KA6\/owMGCSKeWh8rf2dmZueWb2qhGqrmsEydODAYWunWA\/23atGlZbebdpSMexQuoTJRTPUEhxEtB6YuhirDDjWGuJlcsTjyUCRWVlIpOBeiO6ajKyfdchp1HNO2XVTc+h7Wf\/w3LIxLiv2o+\/t\/qVjOTRdIRL6wO7j7yQUwsXjGwxFMbeCCDu3aqAqKC7tu3L9MKSqsjHrpT\/NOdCleV2GavqukeUqxLVVLeft3ZSnL3VCKHEVENtmG9ugCKjni1tbWZo0qmQUEsniclL\/Zi1dF9YGAg4\/pxVy3M4tke4PVh8bh7GmapVIsXRgoTJmF9mc\/iqeQ2Wbywzdwyxyt2NoW0Tx1hTXM83dEbLBYPBIfN8fg8TjefwU3LUed46vEpcm2xPTbEQ+vH522qxbOd4+EpB9NOJh3x8DecY6ruOO8e3byXcFYDOEK8EiYezWnoTT\/ualL0Dl2y5ubmIJCAAQIMgOATY3jlIT5JRoqE\/6JiqVHNsJvZTBHCfEsD3O1Vo5p0O4DuYDG6wOgarlixAg4fPhwMHLt37wZu8TgmmBavLx8eHtZGNU3rdDriffTRR3D06NHgdATHRDenxP5AnHGAePPNN7UDnBCvxIknzTcjEDanjOpqquROirsQLymCkr+oEFDXK+NcA0hlkEXEy4JcEo\/Kl50rRaU60hhBoHQQ+H+uY7NrDZFCKQAAAABJRU5ErkJggg==","height":134,"width":222}}
%---
%[output:37440596]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Block diagram '<a href=\"matlab:open_system ('inv_psm')\">inv_psm<\/a>' contains one or more parameterized library links. To find the parameterized links use the Model Advisor.  The diagram has been saved but may not behave as you intended. Support for parameterized links will be removed in a future release. "}}
%---
