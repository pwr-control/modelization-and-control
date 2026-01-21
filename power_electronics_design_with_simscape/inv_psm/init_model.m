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
use_torque_curve = 1; % for wind application
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
fPWM_AFE = 2.5e3;
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
rpm_sim = 18;
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
% igbt.data = 'infineon_FF1200R17IP5';
igbt.data = 'infineon_FF650R17IE4';
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
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

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
%   data: {"layout":"onright","rightPanelPercent":33.3}
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
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  67.668222262819981"}}
%---
%[output:079a03a1]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.283130953403918"}}
%---
%[output:72559c6d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.283130953403918"],["67.668222262819981"]]}}
%---
%[output:8c11ae60]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:95cb5289]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:42d1bda9]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000200000000000"],["-19.739208802178720","0.996858407346410"]]}}
%---
%[output:85f7f9dd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.311017672705390"],["54.332172227996914"]]}}
%---
%[output:7c2251f1]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.058720741750118"}}
%---
%[output:76a3d4ad]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   2.426376671832986"}}
%---
%[output:5713da6e]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.606931756868082"}}
%---
%[output:7c389dc3]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     3.449190983162578e+02"}}
%---
%[output:8886a72c]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -4.186041938481557e+02"}}
%---
%[output:29a5e80b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASAAAACtCAYAAAAQ2qIyAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXX+QV9V1P9jauvijuIg\/cMXFuFRbWxBrSjY2aIjR2gFT0rqwdkqAYRgibsdCkQUcwkzAZcvmD9Q0xCGE\/thdQ2NGMJMSuhMZFckPfuWHWlBYcF1tcQkBqbSThs55cL6537v3vXffe\/fedy+eN5Mh7vf+OPdzz\/ncc8\/9NeTMmTNngD9GgBFgBEpAYAgTUAmoc5WMACMQIcAExIrACDACpSHABGQI+p07d0JzczN0dnbChAkTDJV6thi57A8++ADWrl0Lc+bMgdraWiN1YZmLFy+Oympra4Oampqo3sOHD0NTU5OROuIKeeaZZ2DHjh2VepMqe\/LJJ+Gee+6BhoaGVJmypE0t7FwCwmnLli0wcuRI2LBhg5YsKnx16zSZ7tixYzB79mx49NFHjetpHjmZgPKgVnKe1atXR+Swfv16awTU19cHM2fOhIcfftgqAZFBTJs2LbUeJKonnnhCy+izpM3SnQcOHIhwmTx5cmTEup8vBITyov709\/drEb5u+\/KmC4qAqPMRPPzI26DOPXHiRPT37du3DxqdyIvA38eOHVsxXvr7Y489Fv0Ny3788cdjjSFOBtFLwfLRmyB5MA+OlHV1ddHfcfTEb+7cuZESU5lk7LLHI\/43eiStra1RfnkEVim5TFZpGGK5ixYtgvnz58O+ffuq5ESjjqsb61m3bl0k05133gkvvPBChShErwELFPGViYIIieqmtGL\/Ud+PGTMmGs1lOVVp0SsV5UcCIU9PNp44GeS\/q8qQ20p9rNJRUQ+JGBDDOB3FsvB3KjMJc1lW0TO36a1nJaJgCEg2UlFxybB37doVKf3w4cMjxRw1alSkZOJoPmXKlKqpBiovTp3ETo3zLuTRWjTu\/fv3V6ZgREAkD00XxJGH6kXFQHlFbyOJgNCQkjwgxKW7uzsiU\/wQB8yjIjoVhphHxgynYIj\/qlWroKOjo1Iu4UttQbIgfMW2x+FEbRFHYzHttm3bqjwemawwbX19fTRYELmQoclpRUyJuAgX0Wioj+k3uS\/SPKC4PpZ1AuuU+7yrq6sKe\/KySAbSUcxLf1NhTvZAfbl58+YqHLN4nVkJJWv6YAgobqRERWlpaRkUvxDT7969e5Aik5EScdBIm+S6Y6cvXLhQOQVQeUCkABhPSer0LB5QGgFRWWvWrIl0QYxLZcEwbgqm8iIwHoVeHcVDxHpoMCCDFnGQBwPEiUZ12TvAtqj6Jm6kV5GVOLDETUNU8TYxNka4qKZgSX1MHtCRI0eUgwMZLrVf9JBVHgumi8NcJjdRJ7AfZJLNShom0wdDQPKoL4KYRkDPPvts5LqKH01fBgYGEo1UzJNGTmTsNNqJBCSTjFiuSQIiRcf20UhJsaIsGMoEREaAhrd8+XJYsWJFVD56S0hAonGLOJEx0LSZ2o2jOQbRRU8VCUieIopEpPLY0AjR60kiW3nqSzLokJw8rU0ioKQ+lsvB\/xa9UyJ2EZc4Lwzll\/tSxIZ0WiYKGmTJdsiDRdzL+oIhoCyjNwKa5AGJYMujiy7JyCtduh6Qyu03SUCipzBixIjK9EvlQSSRuExAosIjvqJXkMUDErFPCsyKsRSaeqiILS5uluYBxRmcCQ9I1cdJBCQPoDI5FfWA5LayB5SDbnViQDQa0hw\/SwwoLnYgiip3nGrUwXJUHpA8auEoRTGAu+++u2o0JDecZJIVMG0VTPQixOCjDobk1cgEpGorBWHFGBC15ejRo5UpWVoMiLwnmdiSZBCndmTA1P8UcBZXzFzGgKg9Yh\/L002ZZOTYFwb7iXhVBCTGgGTMOQaUg2B0soiGJa4AiaPLpZdeGrnksnudtgqmQ0AoY5ZVMHEKhv8\/boWEvBNaYaLVjjgCEtui2nckxxvEvUI6GOK0Cj9asUOiEVfGEHv0rvATp3c2VsHElSZRdpxO4EeYYX8j6ZFHJKcVA9WYL8sqmIrE45bh01bBSCdkApL7BfGVg\/xyX\/MqWAxrIJiosO3t7YM2ackdpJqH65CRriuftSxOnx2BrB6ayqv0ZWNc9taXk6MI5ujN6m78tN064zEgIhh5CZoagtOYBQsWwJIlS7R2kOoA4NMmLx15z7c08qCC7cuyI9wngwilb\/Jift7vhEZlQlcYCUhFMuJ+ElPHCEJRGpaTEWAEqhEw6gEhuWzcuBHmzZsHy5YtUxKQuI8ERUnadcydxQgwAuc3AsYICF3ClStXwowZM6KdvTrTLN\/cwfO7q7l1jIB\/CBgjIHl1CJuadlqY5rGNjY1VZ69uuOEG\/5BiiRiBDwECPT09MHr0aGctNUZA8spGnAeEUzD86HyRarUMCejgwYPOQChaUWjyYntZ5qK9rpc\/NJxdy1tFQPK5miSIk5bP5ZUukXSSTkZTfa5B0FOl+FShycsEVLTH9fOHphuu5R1EQHjGB8\/6JK1QIcHopNPvpuqUrkHIKyflO3TokFO3tai8mJ9lNoFiehmh4eza9qxMwdK7JTmFaxCKyhuakjEBFe1x\/fyh6YZr21NOwRBek7ft6XfX2ZSuQcgqn5w+NCVjAira4\/r5Q9MN17Y3yAOSYzRpK1n6XaGf0jUI+pKpU4amZExARXtcP39ouuHa9lKnYGVsHHQNgr46MQEVxapI\/tCMOUSid217qQQkKozt4DPV5RqEIkYRopKxzEV7XD9\/aKTp2vYSCUhelnc1HXMNgr46sQdUFKsi+UMz5hCJ3rXtJe4DckU4slK6BqGIUYSoZCxz0R7Xzx8aabq2vSoCwgA0\/q\/sU+quQdBXJ\/aAimJVJH9oxhwi0bu2Pd6IWMQizuVlwzAAokYRjLMGSAWTlE5A4kNvSW0xcZNhXPmuQSjYZ7yruCiAmvmZgDSBKpDMte1lWgUr0K5MWV2DkEk4RWI2jKII6uVnnPVwKpLKte0xARXpLZ6CGUBPvwgmIH2s8qZkAuKjGHl1J1M+NuZMcOVOHBrOTEBMQLmVPUvG0AwD28YyZ+nhfGmZgJiA8mlOxlxszBkBy5k8NJyZgJiAcqp6tmyhGQZ7QNn6N29qbwhIPBVPL2PGvXSRt7Fx+VyDUFR+NuaiCOrlZ5z1cCqSyrXtKVfBiHzwKMbUqVOjp3aWLl0KmzdvdvKiomsQinQYj8xF0dPPzwSkj1XelK5tT0lA4qn3gYGBCgEhMdm8ipVAcw1C3s6ifGwYRRHUy8846+FUJJVr24vdB4QP3\/f398OsWbNg06ZN0WOD8+fPhwkTJgC+423zcw1C0bawYRRFUC8\/46yHU95UL71xHD6zZhu895W\/zFtE5nyJGxF37twJzc3NlUJdvWLKBJS5HzNnYGPODFmuDCHh3PXDd+Ghrtfg2JfuytXWPJl4J3Qe1KQ8ISkZTxsNdHiGIkLSjQ8dAeE0Dz95SsceUAYNz5k0JMNg0szZyRmzrd7aC6u3HirfA0p7oNDERWU0vZs7dy4TUEZFMZGcCcgEiullhISzNwSEsOJl9L29vVXkQH+bOHEidHV1QVtbG9TU1KT3gpSCVtkaGhrg1KlTTECZESyeISTDYA+oeH\/rlPDvrx+DB766zw8PSLXcTsTR0tICa9euTX1BNa7ROPVCEjt8+PAgksM8PAXTUZdiaZiAiuGnmzsknKc8tQdeevN4+QREGxF37doFGzZsAPRUDhw4ADNnzoTbbrsN7r\/\/fnjuuedyeUA49dq+fXvk9ai8LCYgXdUuli4kw2APqFhf6+b2hoBIYHkZvrOzE8aMGQMLFiyAJUuWRMSU9UPvZ926dVXZ5DgQekD49fT0ZC2+lPR9fX1QV1dXSt15K2WZ8yKXLV8oON81uQlOfPrsotCHZhmePaBsymwyNXtAJtGMLysUnHET4pQv72EC4ikYG0YcAqEYsyh\/KDIv\/Nf98LUdb\/tDQPKTzASqzcvoqQ4OQtsnoVAMI0RjDk3mI8dOw7gvvhKJfcF\/v1f+UQxc7aI4z7PPPhutWOEZMIzf1NfXQ1NTk1ULYQKyCm9UOBOQfYxDwBnJB6de+C9+l7zUDkd+8B034ABA6mn4bdu2VZbK+W14db+wMbvRV8bZPM7o+RD5\/NUfXwPPt94HBw8eNF9RTImJ9wE1NjbC+PHjYdGiRdDe3g67d++G7u5uWL9+vdXXU9kDst\/\/bMz2MfbZAxKDzijnqNqLYO+yjznfgxd7GFX0dtALam1tjXoMl+JxOmbzYwKyie7ZspmA7GPsI87ylAtl\/JMbL4fnPj8uAsS17fFpeAN6yMZsAESNIhhnDZAUSZB01mzrhX\/+\/jtVv6LX8+S0m+GOG4dV\/u4FAcXFejgGxDGgfCZgJhcTkD6OSDp4sh2v2JA\/JJ5vzBkLF114QTT1Er9SCSjtFDwKihfU5z2EqgufaxB05YpLx4ZRFEG9\/IxzPE5IOJ0\/eAd2vHk8Os+l+lQej5zOte2lroLV1tbqaYfBVK5BKCo6G0ZRBPXyM85ncaJVq\/ldr8WSDSGqQzreeEB6amA\/FROQfYzZmO1jjDUUxRnJ5ts\/eQ++89OjqWSD9SHhTLqpFv7mk9cPml7ptNi17VV5QDpTMN4JPbgbiyqZjmKYTsMym0ZUXZ4OzkgySBxLn3sDftJ3UotoiGzw37Y\/b4Dfu+aSXITj5RTMTdfE1+KahYu2V0fJitZhOj\/LbBrRZAKiaVPP68fgW3v+E478\/HRlKqUjCRLUqMsvgs81Xgt\/dP1lRshGVa9r2+NleJ3eT0nDxmwARI0ifMaZCGZf30n4t5+9B28dO63tyYhNJ6L5xJjL4YHbrrZGNHFwe0NA4tPMJKyLFTCsyzUIGrqfmMRnw4gTnGXO1us0TWr\/bi+8dODnUea41aa0kmnpe9JNw+Hhu66DC4YMcU40XhOQ+DSz+GIFPVbIy\/DV3cfGnGZyZn63hTN5L\/+w\/S34Wf\/7hchFjM3glOm2q4fA8s+Oi6Zb8p4bM6iYLcX14J9pGZ43IibP882qgt3SbBmzTamzykzEsrfvJPS8PgCHjn6QOfaiag8RCRLMfbdcAff9wYgomYpgsspsEz+dsr0gIBQUvZ0tW7YMuhMap2H8NDN7QDrKbDoNGvNv\/M41lWJ3HT4BG155G+AMGCEW2XuZcMMwePCj18CQIWpy0WkfE1AySolBaPlSMn6amT0gHaPLmwY9lvfe\/19Y\/\/LbURA360pRWr0U4L2u9iL45E3D4bO3Xml9asQEVICA0jrU1u+u3cCi7QhNybC9tmWm6Q\/W1f+L\/4F\/2tlvhVTIa8HpEBILei0TGy6PnRIV7eus+W3jnFWetPSubY+X4dN6ROP30JQsLwERqfzyV2fgub3\/Bd\/bf8zo9EeEWoyzjBpeA9Nvvxrg\/aNw3XXXBRHMpbaEphteEBDtiB41apT1g6cq+3YNggbHJCYJTcmIgMR4yp63TsC2147BkQEzgVoVYCKp3HTNxXDXmFr4\/ZGXaHsroeI8evTooirmLL9r24v1gFT7gFTvuNtAxjUIRdvgi2GQh3LtsN+Gr77YBz9755RVQqHpT\/QvLjlffxl8ZtyVcPnQC614Kb7gnEVfQpPZte1lmoLhQ4W4OsZXslaroA0lIzJBr+FHh0\/Alh8fhT1HTkQVmw7OJk197h87Au6+ebj1YK2OUdvAWafeImlCk9kbAlJ5QCNHjqwsy6s6RXxJNW7XtFyu6nCraxCKKJhuPIU2ovUOfBDd2fLym8crQdmIVM69SlBUFjm\/OO35yJVDAQnlhiuGwv\/94p1oSTuEzXGhxlN0dcN0nxcpz7XtxW5EnD17dtQOXW9HfMoHnylevHgx4KX28hM+Yrq4p51dg5C3w5A0fnXmDKz59qtw5OQFzrwTmvI0XDUUpt56FXz8I8MqBKZLKKGNzCEac4gyu7a9TFMwXUMlL2f69OmDLrA\/cOAArFq1Cjo6OmJf1nANgqpdSC54neXLb\/zc2pRH9E5wCRkDsn92y4ho45sYX9HFPUs6JqAsaOVPGxrOrm3POAHRNCxuCqazudEVCEgyV1xyIUx7+se5DxaSaoqeBwZkG666GP5i\/JVw7bBf37mr653kV3f9nKEZRojeRIgyu7I90lTjBEQFI9Hs2LEjcRmflvvxaIf41I9NEDAG09L9ujbhiF7KraMug1kfvxbOOSiV+Akbsz7xFUnJOBdBTy+vTdtTSWCNgHRWzGiqJseKEAT8enp69FBLSbXr7dOw7vvHAf9VfSMv+83oz58YPRQeHHcZ0H\/rVt7X1wcY9wrpY5nd9FYoOE+aNKkCSOkvo+Z5lkfOg8v1+MkHV9Ezwg+D0xgPoldXxYC0KRZWPcKGdaNXc8eNl8OiT9cbWQXikdmNMTPO9nE2ZXu6kma+EzrpUrK4ZXiRdORleNUB16IgIPHILwYg6eDO20c+le+y7iRA2TB01a1YOsa5GH46uYvank4dYppM9wFlLTxv+iIgqN68\/nLzzVA37CIj3o6qTWwYeXs6Wz7GORteeVIXsb089VmLAeURhvLkBUFFPnuXfayIKFp52TC0YCqciHEuDGFqAXltL7XgmASDpmArVqyAlpYWWLhwIezbt29QNl+f5ZHJ51vzxlWuZcgLjm4+NgxdpIqlY5yL4aeTu1QC0hHQRZqsIGDMZ9wXX6mItvnzt8IdNw5zIWpUBxuGG6gZZ\/s4Z7W9ohIFPwWTV7pckw8TUFEV1M\/PBKSPVd6UXhBQ0gupvk3BHtn0H7Dxlf4Ib1xWX3yv+7tX2DDyqnu2fIxzNrzypPaCgOIEx709EydOHHS+K09Dk\/LogiBOvXCZ3UXAWSU3G4ZpDVCXxzjbx1nX9kxJkmkK5tOzPPJeHySfss5asWGYUsfkchhn+zh7TUA6xytMQKQLQu3ffi+q7rH7bog2GJb1sWG4QZ5xto+zru2ZkiTxPiDVMnxnZ2fpUzAx8IxeDwaey\/J+OAhtShXTy2ECSseoaAovCKhoI4rmTwNB3PPz9Rm3wJSxZ1+mLOtjw3CDPONsH+c02zMtQWwMSD4oiue5uru7tW9ILCJoGghTntoTXafhg\/fDHlCRns6WlwkoG155UqfZXp4yk\/IoCSjuRkOdO35MCJgEgrjydcdHhsHmh241UWWhMtgwCsGnnZlx1oYqd0IvCCjPdRy5W6zImATC331zf\/R0L35lrnyJYrNhmOz9+LIYZ\/s4e0FA2Ez0dp544onKKxi0ORFvLpTv+DENSxwIvuz7kdvLhmFaA9TlMc72cfaGgLCpGAeaOXMm9Pef3WmsurvHBiQ6BFTGkYu4trJh2NCCwWUyzvZx9oqA7DdXXUMcCBR8xlxMQMV6h425GH66uUPD2QsCcrXjOa4TVSD4Ov3CNoSmZCyzLn0UTxeabnhBQAg7nvuqr68f9LBg8S5JLyGNgJ6afjNMv\/3q9IIcpQhNyZiAHClGgIOTFwTk42n4zT8+Cp\/7+k8jzfFl9YvUmAnIjUEzzvZx9oKA7DczuQYVCOLmw7JOvXMQulzNYAKyjz8TEADIIPi4+VBUBTYM+4bB00Y3GJdKQBR89u1OaPHsl2\/xHzYMN4bBOLvBuVQCctFE+V0w1el6GYTVW3th9dZDkXg+Lb9zDMiFxvy6DvY07ePtDQHZOoyKO6x7e3uj3dRx9wvJIPgc\/+GR2b5RMNG7w9gLAnJ1GBUJqKurC9ra2qCmpqaCsgiC7\/EfJiB3xsEekH2svSAg24dRxWlY2hRMjP\/4OP1iArJvFOwBucPYCwLC5ro4jEr7jXA6hodc6RNBEOM\/vu3\/YcNwZxhM9G6w9oaAsLm2D6OSJ9TY2Fi14xpBwK+npwfmfutd+FHf6ei\/dz1c76YXMtbS19cHdXV1GXOVm5xldoN\/KDhPmjSpAsjBgwfdgAMAmV7FMCEVelb4NTU1RQS3aNEiaG9vh4aGBqUHhC+eYhyozGd30trNsYk0hMz8zjibwTGpFK88IBvNzbIMLwagP3XzcPjGnD+0IVLhMtkwCkOoVQDjrAVToUTnPQHpoEMgiAT06D2j4dF7\/JyCsWHo9GrxNIxzcQzTSmACEo5idP3wXXio67UIM19XwFA2Now0tTbzO+NsBscP9RRMB0JiYSQfJCH8fF0BYwLS6VEzaZiAzOAYBAHhJsHm5uZBso4dO9b60zxEQL7vgCZw2DDsGwYTvRuMvZiCxe3PcQPBr0\/D09PLvjy\/E9d+JiA3msE428fZGwJasWIFLF++HGpra+23WqqBQCAC8jkAzSOzO\/VgArKPtRcEhM0UD43ab3Z1DQjCP353N0z58p7oBx+v4BAlZsNwoyGMs32cvSAgH65kXfnMjiBWwNgDsm8UHGtzh7EXBOSuueqaEIQHOrbCV1\/sixL4vALGBOROW9gDso81E9C5fUC3LPgmvPTmca+PYPDIbN8geKrrFmNvCEg8MjF58uTozNayZctgyZIlVee2bMCDIFw261+iM2C+r4CxB2RDA9RlsgdkH2svCIjIZ+TIkTB16lTYuHEjLF26FDZv3gw7duwYdIGYaVgQhOOfWR8VywRkGt2z5bEx28FVLjU0nL0gIPFCsoGBgQoBITG5WJ6vv+WjcOLTq6O+9H0Jno3ZjSEzzm5w9oKAsKn4Mmp\/fz\/MmjULNm3aBPPmzYP58+dHF4fhBWI2v1Ef\/VN4\/45FURU+nwHjGJBNLRhcdmjeRIik6Q0BIXjycYzHH3\/cyVPNI+97BE7fNIUJyKJ9szFbBFcoOjScvSIgN100uBaRgHxfgg9xlGOZ3Wk2E1Ay1s5vRNTp+iv\/+ivwyyt+N4gleDZmnR41kyY0Yw5RN7zxgOSbCxFMXI6Xn9Axo1rVpTAB2UC1ukw2ZvsYMwGlY6z0gMRleAo409+wSNskFMopeA5CpyuYyRRMmibRVJflhQdk+12wNBiJgKbffnV0ENX3jw3DTQ8xzvZx9oKAsJmqV0txab6+vt76ShgRkO+n4NkDsm8QYg1MQPbx9oKAkk7DEwQ2b0ZkArKvaGzM9jHmGFA6xl6ughEBhbAJMUQlY5nTDcNUitCI3gsPKC\/4eIlZa2trlD3OQ5JX11TpiIBC2APExpxXW7LnC82YQ9QNbwgo6zI8vnK6atUq6OjoiK5xpaMc8ooZTu8WLFiQeKqeCOjYl+7KrqUl5GDDcAM642wfZy8ISLUMj02PIxUVLKogNqaTiUqVFwnI56eYZZnZMOwbRojeRIgye0FAJpbh41bMxGkadpDqfBkSUAjXcJDZMQExAcUhEJpueEFA5O1s2bIFNmzYEF1Ahp7LzJkzo93QaafhdS+0j3v+hwnIvkGHZhghehMhyuwNASF4Ot6KbCpZ9grRVK+xsbFqbxES0G8deRle+fsH7VuigRr6+vqgrq7OQEnuimCZ3WAdCs6TJk2qAHLw4EE34ACA0WV4JJ+JEydGdwbFfUhq+DU1NUVeFV712t7eXnXNKxJQCBeRURtdjxomtINlNoFiehmh4exaXmMEpHrKmQ6v4lWuRDry6lpcDGjo7q9FXhB\/jAAj4BaBYD0gtzBxbYwAIxA6AsY8oNCBYPkZAUbAPQJMQO4x5xoZAUbgHAJMQKwKjAAjUBoCXhGQuOzf2dmZuJpWBmJioD3ugn4xDb6rRvuoypAX69SRmWSL25flUnZdeXHFdd26dZFoZeuKjsziDRM+6EVcn+oclTKpD94QkHhEY\/\/+\/dDV1WX95sUsQIodg\/nEc2+iAYvn3JBQu7u7Yf369dH5ONefjsyiTGTUZRm0rrziRlfUG3o4s6amxjXEoCszYosfbuJF+V088JkVDNpsjPlcDZzeEJDYKbhUn3ZgNSu4RdPjKIdKhGSCir548WKYPn16opemc+6tqFxJ+bPIjGmff\/55OHnyZGq7bMmsIy\/qxsqVK2HGjBnWnwjXaaeOzFiOSJq6JwV06jeVBon06aefhnvvvRe+8IUvDNqbZ6oeuRyvCKi3tzcaIXyYCshAiYdr8TckIHkHt5ynbEXTlZnO\/tGm0DRitaWMOvISAdHUq+wpmI7MhBeFGFy9r5enn+I2B+cpSycPE5AOStIVtToEFHcbgGZ1RpLpGgftYMe7mXQ8OyPCKQrRkZcGp2nTpkW76UUPpIxpro7M8pEjX6dg2CUfagKieXHoU7CyPR+ybZ3pQdz1u2XEgXTkJWMmL821wag847SpuRzYLVvmpAHEtWzeeEDnQxCa5vr4L47OZX+6AVKSUzZu1\/LryisSfNkekI7MsgdUtsxMQDEI0BzZ12VKcblV9BDogO348eOjK0v6+\/srLbR5eb8OQaTJLBJl2QSE7dGRVzxP6IOu6MgcyjL8h9YD0jEmTsMIMALnFwLeTMHOL1i5NYwAI6CDABOQDkqchhFgBKwgwARkBVYulBFgBHQQYALSQYnTMAKMgBUEmICswJqvUN39QzaOeNA5IFzB090D5MNmS0KaVqJsrTpS+XTLZxnnzvJpld+5mIA86p8yCQgNbPv27akvnohw+UZAtg8wl33w1SNVNSYKE5AxKPULEveN0IiNNwA0NzdHhcydOzciAjEd\/h09kzFjxsDs2bNh3759leev6XAsPqNE6eIeBhDLpNEcy6K640Z48aoUOstEBHTppZdGssneh5hH3K+DO4dfffVVePHFFyvvwol7wO68807AMun5J5XMsgcikyHWcfHFF0NPT0+EFWEq95LsTSaRKhOQvo7rpmQC0kXKUDp5W774Soh8zYT4Yoh4fgifepGfwUbxiLQWLlyovE6Bpllr1qyJyALPfSExUL44D0I0SvGYzMDAQERcRD5yeXREgZ7qJhnlF3bFzW\/Dhw+PCBYJFOUSf8Onj8Q6krwxrIPetaMysTyZmJmADCl2zmKYgHIClzdb0oVPSVMwkQREAkI50GDJuJJ2M8uju3gkIOkOpri33uQjBUnyi7+Jd+Og\/HI++eoK8e6cOA9F5QER4anqoP5jAsqryWbyMQGZwTFTKWLAV5zyyIYo3vqHFVBaFQHhNEP8VFc+yKewdc7fxT0eiXUI6f0GAAABmklEQVTJRi\/KLz+\/JE4tZUJLIiT5cUwsRxVoVhFQfX195UxeHDkyAWVSXeOJmYCMQ5qtwLiLqmTvIskD0r28zYYHJE7bkjwX2QNKIoc8l3eleUAyycV5QElXZXAMKJtu66RmAtJByWAaecSNiwGprqZAMdra2iApBiTGeVTxDjwsmzUGJBolejY05UN5dAiI8lBcR\/aAdGNADQ0N0a2UuFUAcRAD0SoCwr\/hDZbyNFXsTlVcjHCWA91MQAYN4VxRTEDmMU0tUZxWiFMwWu3BFZuWlpYo4IqBVAwUL1myBDZt2gQdHR0Vg8L\/I14RS6tgSTfuxa0opS2pi9NBeRWMyED0XMTT3zhlmjNnDmzdujUijrVr14LoAWE7CBNMi++Unzp1SrkKFrfPR0VAeL3sCy+8EBGWiIkq5tTa2hrhjOS6d+9eJdEzAaWqduYETECZIeMMNhFIijkl1ZsWAzIhMxOQCRSry2ACMo8pl5gRAXm\/U9yenTQCwi0B5CHhBeuyl5VRrKrkvBO6CHrxeZmA7ODKpTICjIAGAv8PV5Hl6ZDFux0AAAAASUVORK5CYII=","height":260,"width":432}}
%---
