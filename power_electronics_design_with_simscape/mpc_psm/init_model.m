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

use_mosfet_thermal_model = 0;
use_thermal_model = 0;
load_step_time = 1.25;
%[text] #### local time allignment to master time
kp_align = 0.6;
ki_align = 0.1;
lim_up_align = 0.2;
lim_down_align = -0.2;
%[text] ### Enable one/two modules
number_of_modules = 1;
enable_two_modules = number_of_modules;
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
t_misura = 0.25;
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
deepPOSxi = 0.5 %[output:0db165d8]
deepNEGxi = 0 %[output:255f588d]
deepNEGeta = 0.5 %[output:57086c1f]
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
l1 = Kd(2) %[output:08bc4522]
l2 = Kd(1) %[output:858ca950]

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

ts_inv = 1/fPWM_INV;
t_measure = simlength;
Ns_inv = floor(t_measure/ts_inv);
s=tf('s');
z=tf('z',ts_inv);

%[text] ### MOTOR Selection from Library
n_sys = 6;
run('n_sys_generic_1M5W_pmsm'); %[output:6685555d] %[output:0345a522]
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
kp_w = 5;
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
heatsink_liquid_2kW; %[output:843555a4] %[output:90a5b190] %[output:9195034a]
%[text] ### DEVICES settings (IGBT)
infineon_FF650R17IE4D_B2;
% infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;

inv.Vth = Vth;                                  % [V]
inv.Vce_sat = Vce_sat;                          % [V]
inv.Rce_on = Rce_on;                            % [Ohm]
inv.Vdon_diode = Vdon_diode;                    % [V]
inv.Rdon_diode = Rdon_diode;                    % [Ohm]
inv.Eon = Eon;                                  % [J] @ Tj = 125°C
inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
inv.Erec = Erec;                                % [J] @ Tj = 125°C
inv.Voff_sw_losses = Voff_sw_losses;            % [V]
inv.Ion_sw_losses = Ion_sw_losses;              % [A]
inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
inv.Rtim = Rtim;                                % [K/W]
inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
inv.Lstray_module = Lstray_module;              % [H]
inv.Irr = Irr;                                  % [A]
inv.Csnubber = Csnubber;                        % [F]
inv.Rsnubber = Rsnubber;                        % [Ohm]
% inv.Csnubber = (inv.Irr)^2*Lstray_module/Vdc_bez^2
% inv.Rsnubber = 1/(inv.Csnubber*fPWM_INV)/5

infineon_FF650R17IE4;
% infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;

afe.Vth = Vth;                                  % [V]
afe.Vce_sat = Vce_sat;                          % [V]
afe.Rce_on = Rce_on;                            % [Ohm]
afe.Vdon_diode = Vdon_diode;                    % [V]
afe.Rdon_diode = Rdon_diode;                    % [Ohm]
afe.Eon = Eon;                                  % [J] @ Tj = 125°C
afe.Eoff = Eoff;                                % [J] @ Tj = 125°C
afe.Erec = Erec;                                % [J] @ Tj = 125°C
afe.Voff_sw_losses = Voff_sw_losses;            % [V]
afe.Ion_sw_losses = Ion_sw_losses;              % [A]
afe.JunctionTermalMass = JunctionTermalMass;    % [J/K]
afe.Rtim = Rtim;                                % [K/W]
afe.Rth_switch_JC = Rth_switch_JC;              % [K/W]
afe.Rth_switch_CH = Rth_switch_CH;              % [K/W]
afe.Rth_switch_JH = Rth_switch_JH;              % [K/W]
afe.Lstray_module = Lstray_module;              % [H]
afe.Irr = Irr;                                  % [A]
afe.Csnubber = Csnubber;                        % [F]
afe.Rsnubber = Rsnubber;                        % [Ohm]
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
figure;  %[output:61f68ec2]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:61f68ec2]
xlabel('state of charge [p.u.]'); %[output:61f68ec2]
ylabel('open circuit voltage [V]'); %[output:61f68ec2]
title('open circuit voltage(state of charge)'); %[output:61f68ec2]
grid on %[output:61f68ec2]
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
%[output:0db165d8]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepPOSxi","value":"   0.500000000000000"}}
%---
%[output:255f588d]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGxi","value":"     0"}}
%---
%[output:57086c1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGeta","value":"   0.500000000000000"}}
%---
%[output:08bc4522]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  15.921682195383369"}}
%---
%[output:858ca950]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.064017382188212"}}
%---
%[output:72559c6d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.064017382188212"],["15.921682195383369"]]}}
%---
%[output:8c11ae60]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:95cb5289]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:42d1bda9]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000041666666667"],["-4.112335167120566","0.999345501530502"]]}}
%---
%[output:85f7f9dd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.064795348480289"],["11.319202547499357"]]}}
%---
%[output:6685555d]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:0345a522]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:7c2251f1]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.012443765785704"}}
%---
%[output:76a3d4ad]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.517590710054527"}}
%---
%[output:5713da6e]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.152987786896029"}}
%---
%[output:7c389dc3]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"  93.745795204429072"}}
%---
%[output:8886a72c]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -1.166194503484206e+02"}}
%---
%[output:843555a4]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:90a5b190]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:9195034a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
%[output:61f68ec2]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAQgAAACfCAYAAAAF4kewAAAAAXNSR0IArs4c6QAAHFdJREFUeF7tXQ+QVtV1P0TTAAajC1hFXNc\/rDWNVXTMWGIEYhSbBExpOgidlgJDkIh0KgyygINMBYGyTgc0CXUIpU5YHCqJkE7LMESoFGxSUbRFi2ZZCGIMLjES1CbR7Zxnz+f57t733n3fu\/92v\/NmMpH97j333N+553fP\/d+nq6urC+QTBAQBQUCDQB8hCGkXgoAgkIaAEIS0DUFAEEhFQAgiBZpnnnkGJk2aBBs3boQbbrjBahNSZb\/77ruwevVqmD59OjQ0NFgpC2XOnz8\/kbV8+XLo168fYLlHjhyBCRMmWCkjTcjjjz8Oe\/furZSbVdjDDz8MY8aMgWHDhuXqVCRtrrD\/T0A4bdu2DYYMGQLr16830kWHr2mZNtOdPHkSpk2bBvfee6\/1dop6CkHYtFaNslasWJE477p165wRxLFjx2DKlClw9913OyUIarB33HFHbjlIJGvWrDFyyiJpi5jhlVdeSXAZO3Zs4mSmXywEgfpi+zl+\/LgRIZvWj9IFJQgyDlYOP+qtCfy33347+fvu3bu7sTv1wvj71VdfXXEu+vt9992X\/A1lP\/jgg6mNNU0H3sujfOyNSR\/Mgz3N0KFDk79j74PfjBkzkkZGMskZ1YiB\/xt79JaWliS\/2oPpGqFKJnkYotx58+bBrFmz4MCBA1V6otOllY3lrF27NtFp1KhRsGvXrooj814XBXJ8VUcmwqCyKS23H9m+ubk56Q1VPXVpMarj+qODU6SkOkGaDurfdTLUupKNdW2Ut0NyXMQwrY2iLPydZGZhrurKI1uX0W4wglCdiDcscrxnn302aZQDBw5MGk5jY2PSCHhvOG7cuKpQGhsXDg046Gm9s9rbcec7dOhQZYhBBEH6UDjMmZvKRcOhvry3ziIIbOhZEQTismnTpoTs8EMcMI+OiHQYYh4VMxxiIP7Lli2D1tbWilzCl+qCzkz48rqn4UR14b0ZT7tjx46qiEElE0zb1NSUkDk5PzmCmpZjSsRCuHCCIBvTb6ot8iKINBurbQLLVG3e1tZWhT1FKaQDtVHMS3\/TYU7+QLbcunVrFY5ForYeE0Gk9TRoyNmzZ3cbP\/P0+\/fv79bQyInIsamnygpN0Shz587Vhri6CIIMhOP5LKMUiSDyCIJkrVq1KrEtnxcpgmHaEEPXC+N8CEZFNB7n5RBZk8NxHFSyRpyoV1R7V6yLzjZpPaWOTDjxp4XZuvkePjdDuOiGGFk2pgji6NGjWvImR6T68whT1+NjujTMVfLhbQLtoJJgURLISh8sglB7TV7JPILYsmVLEprxj8Lzzs7OTCfiefLIg5yRegtOECoJcLk2CYIaItaPehqaqyiCoUoQ1EjRMRYvXgxLlixJ5GO0gQTBnY\/jRI2VhoVUb+wNcZKVR3pIEOoQiBOFLuJBJ8GoIYsM1aEd6WBCQuqwLYsgsmysysF\/8+iOiJfjkhbFoP6qLTk21KZVR6ZOkHyHIkDE3dYXjCCK9H5Y4awIgoOhsrMpCagrFaYRhC6stUkQvKcdPHhwZXih64GzSFYlCN4gEV\/eqxaJIDj2WRN3fCxPobWOeNLmbfIiiDSHsBFB6GycRRBqB6eSR9kIQq1rr4wgTOYgqDehMWaROYi0sSsHVwVWx9ooRxdBqKyPLE9j0FtuuaWqN6Ewk3RSG0jeKgbvhfnklAmGFBWoBKGrK03S8TkIqsuJEycqQ468OQiKPlTiydKBD13Iwcj+NCHJVzx8zkFQfbiN1eGUSgLq3AtOBhMx6giCz0GomNflHAQ2XN7w+Qw+Z+cBAwYkIacaPuatYpgQhKoD\/lvnxGkEkTbDTb07rRDQbHUaQfC66PZdqONdvlfCBEMcNuBHKy5IBHxlA7HH6AQ\/PnxxsYrBVwq47hgu40eYIeZIShRRqGn5RCbmK7KKoSPZtGXOvFUMImCVIFS7IL7qJLBq62hXMdSJoazxi26cR+kRFGx4K1eu7LbZRAU6S05Ma8y2xnI9SU7RCEcXlbnauNOTcCyiaxnMMRo03ZhWRCdMm8xBIEHgJBVOVmXt5MtKR06tLgWSQph3zpw5sGDBgtydakIQRc1oN71K5jyyMinJZYM1Kb8npqkV8x6zkxIbBYaESBA6EuDr7ra2E\/fEhiA6CwI9CYFKBIFLNHwMWqQS6PwbNmyAmTNnwqJFi7QEwdfbUXbW7sYiZUtaQUAQcIdAZZlTDXFMD65gvqVLl8LkyZOTHYQmwwjXYZE7uESyIFBfCKTugzDt8dWNMAhfHrkQGY0YMaLqjMSll15aX+hLbQWBGhHYuXMnXHLJJTXmNs9mtFGqyCRmWgSBhIMfnQPQrXYgQbS3t5trLymdIyA2cQ5xTQX4souWINRlz7yIgGqorlRwUsg6AUj5fVW6JovUaSaxSZyG92WXqklKOmZrSgi2ofNVadt692Z5hw8f9hLK9mYMXdTNl68kBIG9O\/4v9PKjr0q7MFhvlSkEEadlffmKtY1SNmD0VWkbutaLDCGIOC3ty1e0Q4wsSLK2SJeF0lely+pZT\/mFIOK0ti9fMVrF8AWRr0r7qk9vKEcIIk4r+vIVIYg47R+NVkIQ0ZiiShEhiDjtUndaCUHEaXIhiDjtUndaCUHEaXIhiDjtUndaCUHEaXIhiDjtUndaCUHEafJgBMG3RNPVZGlHuG1D56vStvXuzfKEIOK0ri9fqVrF4Fesjx8\/PrnjYeHChYAPdbi60orD76vScZo8Tq2EIOK0iy9fqSIIfmoTb\/AlgkDiMLmSriyUvipdVs96yi8EEae1fflKt30Q9ELR1KlTYfPmzcktUfiuo+5tANvQ+aq0bb17szwhiPisu+fVt+Crq3bAm9\/+U+fKaTdKqY+l+roeTgjCub0LFyAEURgy5xnafvwzuKvtJTj50GjnZclOSucQ9+wChCDis58QRHw2qVuNhCDiM30wgsh7QMf1RTIyxIivMQpBxGeTFds7YMX2w2GGGHhNXEdHB+DLSPTR30aOHAltbW3JC9A2XxCmcoQg4muMQhDx2SQYQaRdTkt\/x0dV8eXnvBe48iDFlRL8OAnhv4Ug8pDz\/7sQhH\/M80oMRhC65\/PoWvvrrrsObr\/9dnjyySdLRRC0QkIvHXMwhCDymob\/34Ug\/GOeV2IwgiDF1GVOfIW4ubnZ6FGcrMpRJDJs2DA4ffq0RBB5LSGC34UgIjCCokKwSUrXUODQAucxjhw50m2eQ4YYrtGvTb4QRG24ucyFeyCQJHrVPgiMSnbv3p1EDbqJUCEIl02qdtlCELVj5yrnuEeegz0\/eSsMQahP7lEly15Wi9HD2rVrqzBT5yFwDgI\/fFZMvjgQOHbsGAwdOjQOZUQLuPnmm+HtW1fAB\/0H+ScI\/jLWli1bkuEAnsFA525qaqp6R7OMrSSCKIOe37wSQfjFO6+0oyffg2se2Jck8z7E4MucO3bsqMwTmL7NmVc5+l0IwhSp8OmEIMLbgGuw8Uevw6xNL4chCP7q9rXXXgv0wO7+\/fth06ZNsG7dOqevb8kyZ1yNEbURgojLJg33PJUo9LF33gxzmlONIlpaWhKFcKkThxsuPyEIl+jWJlsIojbcbOfCocW4bz4H+P\/49d\/\/HTi26zHbxXSTJ6c5nUPcswsQgghvPz7vgNpMvP58+JeFX4b29nbnyqXeKMUf8rU9B5FWK4kgnNu7cAFCEIUhs5YBiWFW20vJkiZ9N152Dmy9a7i3YwnGb3PiBbauDmlR5YUgrLUta4KEIKxBaSxIHU5gxsaGvvDkN4bDxQ19Ezm+fMUogjCuWcmEvipdUs26yi4E4cfcL7x2ChZ9\/9WqaIFKxiHFIxOvrFLEl6\/IHIQf+\/fYUoQg7JsOI4T3P+iCv3r8ZS0hUMQwa1Qj3PrpgUn0oH5eCSLvohhUruxOShOYfVXaRBdJ8yECQhDlWgKtOuBcwtFfvFdZhdBJRSL40mcGwZ03XaQlBZ7Hl69IBFHO\/r0+txCEmYmJCNb+2zF48bVTqZEBl4aE0HhuX3h44pW5hBA0gjCDwH0qX6zovia9pwQhiGpbIhH8qOOX8I\/7judGBCoZ4L+\/ctVg+PrnhxYmhGgIgj+9R0r5WMHAsoQg4iOWeiMIJIDTv34fvrX7p9Dx5rtGkYBqNYoMFn7pUrjgU58oTQa6VuHLV1Kf3uPXwdFjOrLMGZ8Du9aoNxEEDQP2tb8F3\/2P1xPo+B6DIljSxOGNl58L99x8MZx5Rh8nRJCmUxCCyLuTsuxdlHkG8FXpPD3k948Q6CkEQc7\/ved\/Djtf6izl\/FR7igQuGdwf5nzxYq8EkNcGffmK9um9bdu2wfr16wGvhqM7KXGYoV4ym1eJor\/7qnRRveo5fUiCIKdHR92w7zg8sf+NxBR5qwEm9qIIACcJr7zgLFgxvjlZYdAtKZrI853Gl69oVzHUS2Pk6T3f5o+nPBcEQY7\/QVcXrP7hUXj15+9Y6fF5z4\/\/jc7fNKgfrBg\/DE6c+k3yc08hgLwWEJQg8pRz9buvSrvSvzfKNSUIcvpT7\/0WHt3zGrSfsOv03LnR8S9q6At3XH9BZetxb3F80zbky1dkH4SpReosHTl8x9Gj8GznJ+Cpl09aC+9VKHm43ziwXzLeP+NjfXpVj2+7+QQhCNpR2djY6Pxglg4wX5W2bayeII8c\/jfvd8HOlzvhBy+eAOiyM57X1Z87Pfb2t\/3+IBj7B4N71Dg\/Zrv68pVuEYRuH4TukRsX4PmqtAvdfcskh29\/8x3Y9sIJeOWND0N6GxN4aXXhTn\/5ef1hyucuhKuGfNJ31aU8j3uGjIYYeGU97oWQK+fst00+c\/4\/b7yTzNQ\/0\/7h+X+Xzq6O6dHh\/3j4eXBxQ7+q0N50DsI+MiIxCwFfnalRBJH3qjd\/iStt16UamegOf\/mqtKumR706yj+77xnw93tegz2v\/MK7s2OB1zSeDV8bfh6c0\/\/jpcbyQhCuWks5ub58pdt9ENOmTUs0N40W+FX5+H7C\/PnzYcSIEd2uyOfpcH9FT5iD4A5\/8PVfwb\/+d2dldt5n737F+WfBl68aBJcO6l+BzdesvRBEOUd2lTsIQZStDEUJEydO7HbBLW64WrZsGbS2tqbejO2r0lRPJIDjv\/xfeOyZ4\/DTk+\/VvO02Dzd1wu7axrNh6ogLoU8fiH7STggiz7phfvflK0ZzECYQ0DAjbYhhsvnKZaV19\/uZ1EtNozr7VRcOgDGfHpgsy\/nq1WvRu9Y8QhC1Iuc2n0tf4ZpbIwgSikSwd+\/ezGVSWk7Frdv8Kn2sNH62nt77m52d8P2Dp4wsNeTsM+GCAWfCRZ\/6OPzJVZ+Ec\/qekeTDv9fzJ0\/vxWV9fHqPPu+3WtuAwmTFgz\/QM2HChEqxNlhRd+EnFUA9\/F9\/8WIY3dzQK3t8GzbkMiSCsI2oHXk2fMVEE6NLa7OuvVd\/w+VQ\/NSDXRhZ4IeEgPMR9GoXn7AsU+k0YlBvAzYBRdJ8hIAQRJytoYyvFKmRlWvv05Y5OSmoy5y6A2C1Vlp9WARJYeZNF8GMm+RV6iKNQZdWCKIsgm7y1+orRbUxiiCKCq01fdFKIzHg+f8lP\/hJpch5tzbBpM9eIMOHWo2g5BOCsASkZTFFfaXW4q1PUtaqCOYrUmkkh7Yf\/wxWbD+cFIlRw8N3XAk3Xn5OGRUkrxBEj2gDRXylTIUqQ4wlS5bA7NmzYe7cuXDgwIFuMmO79v6fX3wT\/nz9ixVy2PqN4RI1lGkJKXklgnAAqgWRXgnCgr5WRJhWms85YOQg5GAFfq0QIQh32JaRbOorZcrAvD1uiCHkUNbkxfILQRTDy1fqIASR9cJWLEOMcY88V9kSffKh0b7sUbflCEHEafogBJEGBe5tGDlyZLfzFbahy6u0Gj08v+gPbasg8mSSske0gTxfsVUJoyFG1kYpW4qgnLxK8+gByaE3nn2wiacNWRJB2EDRvow8X7FVohFBmGyftqFQVqV59HDjZefA1ruG2yhSZOQgIAQRZxMJQhBZcxAbN24MOsSQ6CFMQxWCCIN7XqlBCCJPKde\/p1VaogfXyKfLF4IIh31WycEIQj1IhecpNm3aZHzDVBk40yq959W3YNw3n0tEy9xDGYSL5xWCKI6ZjxxBCCLtRiiTOx5sgJJWaRpe4KSkrFzYQNpchhCEOVY+UwYhiBgf7+XDi9lfaIT7v3KZTzvUfVlCEHE2gSAEgVBgtLBmzZrK4700cYk3P4V4vHfrCyfgL\/\/hvxIr4ZZqOYzlt8EKQfjF27S0YASBCtKL3sePH0\/0Dfl4rwwvTJuMm3RCEG5wLSs1KEGUVb7W\/Gql+fBiVPO5sOXOa2oVLflqREAIokbgHGcLQhC+dkymYadWGu97uKvtJRleOG5sWeKFIAKCn1F0EIJAffDcRVNTU7eHb3zApFZahhc+UM8uQwgivA10GgQhCJenOdU7KXU7M3mlZXNUHA1TCCIOO6haBCEIl1Dg6khHR0eyEpJ2toNXmm+OunfMJXDvmCaX6onsFASEIOJsGr2OIDjMSBBtbW3dHtfhlZb5hzgaphBEHHYIGkHQ5KTrOyn5MCNviCEXw8TRMIUg4rBDUILwDUHW03v0nNg1D+yrPGwr26t9W+ij8oQgwmGfVXKwIYaPw1pZT+8hKI99bzuM3XAswefzTf3g78b+bpxWqgOt5G3OuIwc9G1Ol4e1ijy9xycoZXt12AYqEURY\/NNKDxJBuDysVWSZk09QyvHusA1UCCIs\/lERBCoTw2Et2SAVT6MUgojHFlyTIBEEKRD6sJZMUMbTKIUg4rFFNAQRChJkxV3\/eRCQIPAbfUUDPDHj6lDqSLkAIAQRZzMIGkGEgkQlCNlBGcoSsswZHvlsDeqWIJY+vldOcEbUOiWCiMgYTJW6JYgZ3\/ohrNh+OIFCVjDCN04hiPA20GlQtwTxmTlPJG9vygW1cTRMIYg47KBqEYwg8CDVpEmTuqHi6\/Hes6d+N9liLa9nxdEwhSDisEMUBJF2RsIXRMiKb311XVKcEIQv1LPLEYKIww7REMSSJUtg8eLF0NDQ4B2Zxs\/+EfzqxnlJubKC4R1+bYFCEHHYIQqCQCX4xS6+oeEEIROUvtHXlycEEYcdoiAIl1fOmcDMCUIOaZkg5j6NEIR7jGspIdgkZS3K2spz\/tcegF83fi4Rd\/Kh0bbEipwSCAhBlADPYda6JIjz\/uLb8NtBV8gSp8OGVVS0EERRxPykD0YQ\/Fj22LFjYd68ebBo0SJYsGABDBs2zGntB925GT7oP0hWMJyiXEy4EEQxvHylDkIQRA5DhgyB8ePHw4YNG2DhwoWwdetW2Lt3b7dLZm2D0XDPU4lIWeK0jWzt8oQgasfOZc4gBMEvjOns7KwQBBKHj+VPIoh\/+vrV8IXf87\/M6tKgPVW2EESclgtCEAgFvqyFj\/ZOnToVNm\/eDDNnzoRZs2aBj9e9iSAemXglTLz+\/DgtU2daCUHEafBgBIFwqNutfb3uTQQhS5zxNEohiHhswTUJShBFIcHNVS0tLUm2tDMb6p2UunRCEEWRd59eCMI9xrWU0GMIAq+nW7ZsGbS2tibbs2mIsnz5cujXr1+l7ji\/MWfOnMzVECII2QNRS5Nxk0cIwg2uZaUGIwi1p8eK4HKn6vBpFUx7Vk8lEl1+JAg55l226djNLwRhF09b0oIQBF\/mxEd28aO\/4X+bkARGEE1NTTBhwoQqLPgwBH\/QzWsIQdhqPvbkCEHYw9KmpCAEUfZdDNODXmnHypEgZA+EzWZUXpYQRHkMXUgIQhBYEd0QIS0q4BU3SUPp057eQ4L4naP\/Dvv+9s9cYCoya0BAnt6rATSHWYI+vZd1mpPqrFt9QHIYOXJkslci7TN5eg8JQu6BcNi6ahDtq6eqQbW6zuLLLn26urq6yiCtu6KOJjVxizZ+OB+hTn6mzUH03\/+dJIqQTxAQBLIRaG9vdw5RaYJwrqEUIAgIAsEQ6EYQZZc5g9VEChYEBAHrCFQRhG6ZE0tM2\/xkXRsRKAgIAlEhUEUQZZc5o6qZKCMICAKlEeg2xMBoYdu2bbB+\/frkghh66RsnHmnzVOlSFQF8E9XGjRszV0Nsly3yPkSATzanHc5TX32fMWOGszYhdslGwOTogg0MtZOUJrsebRSOMvgW7EOHDkFbW5vRjk1b5YscAN7YEA9+tobjk7aNXjD0iwARNZZKHbkrDYKvYiAZ0W1VOAeSd6DLFRD1LBcdHyPHdevWJQfs5s+fDxMnTuwWyZnulK1nLF3XHcn80Ucfhdtuuw3uv\/9+WLlypdOrIKMgiI6OjiRUDf2yl2vjxiqfRwaoIxLEiBEjqs7TqKtbeC2h694rVrxi0AujCLwvVggiBmv0ch1MCEKFgEcdIV5h6+Umya1eXRGEDDFy24PTBKZDDK6ErwbqtOI9WLgv\/IMPMWSSMnwrNZmkxCHG0qVLYfLkycmYl88d8YuBwtemPjSoG4JAc9KqiYxrwzVuvszJl5rVQ3ZTpkxJLjUWW4WzFZZcVwQRFmopXRAQBNIQCD7EENMIAoJAvAgIQcRrG9FMEAiOgBBEcBOIAoJAvAgIQcRrG9FMEAiOgBBEcBOIAoJAvAgIQTi0jenZBZM3Q4qqyU9emp6QjekwFi27pr3UVhQPNT3JL\/LmS9kye2J+IQiHVgtJEOgAu3fvLnQcOzaCcH2yF0l0w4YNsHDhwqpX4Bw2iR4nWgjCgsn4JiPq8fDo+qRJkxLpdG+CesEv9uzNzc0wbdo0OHDgQOVdUzpRifdy4JcVAXCZ1BuiLCo7rYfkR\/rp\/gciiAEDBiRlqr03z8M3SuFJ0IMHD8LTTz9deRCJb34bNWoUoEy6T0Sns7obUyUrLOOss86CnTt3Jlil3UWhRmNZpCcEkd\/4hSDyMcpMoV7cwXce8ghC3fnGtyrj2xPq+6ZYKDoUNvC5c+dqT07SMGLVqlWJM+MpTHRcypfWA3On4UfsOzs7E2IhclDl0ZFweoOVdFSvJOR1HThwYEKA+CQC6sV\/Gzp0aJXOHGgdQdBFRiQT5alPLQhBlGzQSnYhiJJ4Zt3skzXE4A7ACQLVQYeixk\/HrHX3M6hOxA9dZV2+k\/bIkXpCM0t\/\/hvKI7LA\/1fz8X+rZzjSengdQWSVQWYUgijZoIUg7AKI0viEIA\/pVUdBR1q7dm1FAUqrIwgMo\/mnuwZOdTaTg29pr5phWapTcv11t51TmK8SThZhqLeVYbm6iUgdQfA3X9PISwjCbvuWCMIunlW9J2\/Eau+cFUGY3qrlIoLgw5Ksnl+NILKcV5VDFwRlQZ8XQagklBZBZJ06lTmI\/MYvBJGPUWYKtcdKm4PQ3bmAgvHF9Kw5CD7PoBtv4+nKonMQ6jV\/NKRBfUwIAqMJPq+gRhCmcxB4bDztSQUdQeDf8Fo8dRjGDaSblyGc1YlQIYj8xi8EkY9RbgoeNvMhBs3WYyg+e\/bsZEIOJ9pwInHBggWwefNmaG1trTR4\/A9+LyStYqTdMk3DAt2KRd6SJR\/uqKsYSFroTLzn5++24pBg+vTpsH379oTgVq9eDTyCoHmIlpaWZPiAD86ePn1au4qRts9BRxCnTp2CXbt2JcfNOSa6OQ8sG3FGInv++ee1RCwEkdu0QQgiHyNJUQKBrDmPokMMlYRKqJVkFYLIR1AIIh8jSVEQAXW\/Ry3vZ5AMijDwJmebBEHyZSdltnGFIAo2fkkuCNQTAv8Hn\/TrqsderK4AAAAASUVORK5CYII=","height":159,"width":264}}
%---
