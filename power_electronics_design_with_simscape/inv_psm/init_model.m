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
t_misura = 10/omega_bez*2*pi;
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
igbt.inv.data = 'infineon_FF1200R17IP5';
% igbt.inv.data = 'infineon_FF650R17IE4';
run(igbt.inv.data);

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

igbt.afe.data = 'infineon_FF1200R17IP5';
% igbt.afe.data = 'infineon_FF650R17IE4';
run(igbt.afe.data);

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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASMAAACvCAYAAAC2Jbg6AAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ1wV1eVP3W7blNKtWBtN8YYtMGq1bZ026XoCp2stuss1HbWAeKsCEyHRWlmVlg+UhzKWGjINs5KW5XpIIOjIZWR0dDRRYzCFhHd8uW6rTY1BIyplQawLQVXpuycB+ffm\/t\/H\/e9+\/HuDefNdKbkfz\/O+91zfu+cc78uOnv27FnghxFgBBiBkhG4iMmo5BHg7hkBRiBCgMmIFYERYAS8QIDJyPAw7NmzB5qbm6GzsxMmTpxotHW57VOnTsHatWvhnnvugTFjxhjpC9tcunRp1FZbWxvU1NQA9nv48GGYPn26kT6SGnn88cdh9+7dlX7TOnvkkUfg9ttvh8bGxkyZ8pTNbOx8AcJp69atUFtbCxs2bFCSJQ5f1T5Nljt27BjMnTsXlixZYlxPi8rJZFQUOQ\/qrVmzJiKK9evXWyOjgYEBmD17Ntx7771WyYiMY8aMGZn9IGk9\/PDDSgSQp2yeIe3t7Y1wmTp1amTQqo8vZITyov4MDg4qkb\/q++mUC5KMSBEQSHzIC6GBfumll6K\/79y5s+qrRd4F\/n799ddXDJn+\/vnPfz76G7b94IMPJhpGkgyi94Lto5dB8mAd\/ILW1dVFf8evKj7z5s2LFJraJMOXPSHx3+ipLFu2LKovf5njFF4mriwMsd3FixfDggUL4ODBg8PkRANP6hv7WbduXSTTlClTYMeOHRXSEL0JbFDEVyYNIifqm8qK40djP378+OgrL8sZVxa9VVF+JBPyAGVDSpJB\/ntcG\/K70hjH6aioh0QSiGGSjmJb+Du1mYa5LKvosdv04ouQUnBkJBusqMRk5Hv37o0MYOzYsZGS1tfXRwonfuWnTZs2LBxBRcbwShzgJK9D\/oqLhv7ss89WwjQiI5KHQgrxi0T9opKgvKIXkkZGaFRpnhHi0tXVFRErPogD1okjvTgMsY6MGYZpiP\/q1auho6Oj0i7hS++CxEH4iu+ehBO9i\/iVFstu3759mCckExeWbWhoiD4cRDRkdHJZEVMiMcJFNCAaY\/pNHosszyhpjGWdwD7lMd+0adMw7Mn7IhlIR7Eu\/S0Oc7IHGsvu7u5hOObxRouQS946wZFR0hcUlaalpaUq3yGW37dvX5VSk8ESidAXOM29RwVYtGhRbJgQ5xmRMmD+JU0B8nhGWWREbT300EORToh5rDwYJoVpcd4F5q\/Q26P8idgPfRjIuEUc5A8D4kRfe9lrwHeJG5skDyCOuMSPTFKoEpefE3NphEtcmJY2xuQZHTlyJPZDQQZM7y96znGeDJZLwlwmOlEncBxkws1LHqbLB0dGsjcgAppFRlu2bIncW\/GhEGdoaCjVYMU6WURFhk9fQZGMZMIR2zVJRqT0+H70BaXcUh4MZTIig0AjXLFiBaxcuTJqH70oJCPR0EWcyDAotKb3xq88JuBFDxbJSA4jRVKK8+TQINEbSiNeOTwmGVQITw5908gobYzldvDfotdKJC\/ikuSdofzyWIrYkE7LpEEfXLId8mwR9zKf4Mgoz1cdwU3zjETg5a+OKuHIM2aqnlFcaGCSjEQP4sorr6yEaHGeRRqhy2QkKj\/iK3oLeTwjEfu0pK6Ye6HwJI7kkvJsWZ5RkvGZ8IzixjiNjOSPqUxUup6R\/K7sGWlSr0rOiL6SlBPIkzNKyjWIYsuDGPc1wnbiPCP5a4ZfL8oZfOQjHxn2lSRXnWSSlTFrNk30LsTEpQqG5O3IZBT3rpTAFXNG9C5Hjx6thG1ZOSPyqmSSS5NBDP\/ImGn8KVktzry5zBnR+4hjLIekMuHIuTKcKCASjiMjMWckY845I02yUakuGpk4kyR+dUaPHh257bILnjWbpkJGKGOe2TQxTMP\/T5ppIa+FZqpo1iSJjMR3iVvXJOcnxLVIKhhi6IUPzfwh6YgzbIg9el34iCGgjdk0ccZKlB1DDnwIMxxvJEDylOSyYpIb6+WZTYsj9KSp\/azZNNIJmYzkcUF85QkCeax5Ni2DNRBUVNz29vaqxWDyQMXF7CqkJJfxaQ1HEflDr5PXc4vzNn1ahBfCeOhgjl6u6iJTF1hYyRkRKchT2vRCGOYsXLgQWltblVatqgLBZKSKlJ1y8kcGe8mzEt0347CDktlWi2J+wazARqVCVxnJKI5wxLUqprYxmB1ibo0RYARcI2DcM0Ki2bhxI8yfPx+WL18eS0biGhV84bSVzq4B4f4YAUagHASMkhG6jKtWrYJZs2ZFq4lVQjEf3cVyhoJ7ZQQubASMkpE8w4TQZu1opph30qRJw\/aBvfOd77ywR4bfnhEoEYGenh4YN26cUwmMkpE8O5LkGWGYhg\/tdYqbdUMy6uvrcwqGTmehyYvvyjLrjLh6XcZZDStnZCQSUNrubRI7tAEMTV4mIzUDMVGKdUMNxSoykjccpjVjan1QXB+hDeChQ4ecu7VqQ5xcimXWRVCtfog4l2F\/sWSEmx9xE2TatDuSlko5teGqLlUGGEVlxXohKhzLrDPi6nVDxLkM+7MWpqkPVXzJMsDQkTlEhWOZdUZcvW6IOJdhf4lhGkJt8jhT9aE7V7IMMPLKKJYPUeFYZp0RV68bIs5l2F+sZyQnmLOm59WHRb1kGWCoS1ddMkSFY5l1Rly9bog4l2F\/SmFaGSumywBDXb2YjHSw0qkbomGHKHMZ9qdERqLy2E5cU19lgMFGooOAm7ohGnaIMpdhf5lkJE\/1uwrZygBDx5xCVDiWWWfE1euGiHMZ9pe5zsgV+chDWwYY6urFYZoOVjp1QzTsEGUuw\/6qyAiT1\/hf2Ud7lAEGG4kOAm7qhmjYIcpchv3xokdDNhSiwrHMhgY\/o5kQcfaGjMTbOdNw5u0gr6MTosKxzExGSQh4QUZuhie7lzLAyJYquQQbtg566nUZZ3WsdEqWYX+Zs2k6L6RTtwwwdORlI9FBT70u46yOlU7JMuyPyUhnxIS6bCSGgByB+ZcQdYPJSFDEMsDQMacQFY5l1hlx9boh4lyG\/bFnpK5TqSVDVDiW2dDgj0BvjsmIPSM31nG+FyYjN3CHiLN3ZCTu3qerjZOuHzI9rGWAofMOISocy6wz4up1Q8S54bpboP+XP1d\/SQMlE8M0IiLcDnL33XdHd6Hdd9990N3d7eRKXCYjA6M7AsOHEA07NJk3\/ffv4bObnoFjX7zNvhIKPSSSkbg7f2hoqEJGSFI2j5sl2ZiM7OtBaEaCiLDM9vXCOzLCV16zZg0MDg7CnDlzYPPmzdEtsQsWLICJEyfCkiVLrKLCZGQV3qhxNmz7GIeIs5dkhEDu2bMHmpubK6Pm6ipqJiP7hsJkZB\/jEMlozbZ+WLPtkD9hmpthOud94SN7WkxG9keAycg+xkxG6hiXus6IvK558+YxGamPmbGSTEbGoExtKDScvfOMsi5z1D10jRLkjY2NcPLkSSYjN3YxrJfQjCRELyNEmb\/xs+eh5fFf+RWm4UH8\/f39w4iC\/jZ58mTYtGkTtLW1QU1NTW5TwvAM2zh8+HBVH9gYh2m5Ic1dgckoN2SFKoSGM07rYxLby6l98dRH8mhaWlpg7dq1mTfPxo0ehmc7d+6MSC6O8JiMCul87kqhGUmIXkaIMk97dD\/s+s0Jf8iIFj3u3bsXNmzYABhO9fb2wuzZs+Gmm26CO++8E7773e8W8ozQK1q3bt0w45HzRugZ4dPT05PbyMqoMDAwAHV1dWV0XbhPlrkwdLkqhoRzU1MTnPj4+uj9vPGMCG15ar+zsxPGjx8PCxcuhNbW1oikdB72jHTQ06vLnpEefqq1Q8L5yLHTcMMDP\/WTjFQBL1qOyagocvr1QjISeluWWX\/c01rY9dwJmPbl\/RcmGSUBwwlsu0oXYi6DZbarE6JX9IZXX4QXv\/oJux1KraeuM5Kvtaa6Ng\/ipz6YjOzrAXsZ9jEOhUCRiBZseiZKXONz2a52OPLz77sB6HwvqRtlKS+0ZcuWaBoe96Rh8rmhoQGmT59uVVAmI6vwRo0zGdnHOASckYi+tff3sPr7hyJA6sdcAi997ZPQ19fnBiAVMqLd+du3b6+sBRJ389u86JHJyL4eMBnZxzgEMsKENRISEVH3Z26EKX\/zXn\/IiKb2J02aBBMmTIDFixdDe3s77Nu3D7q6umD9+vVWb51lMrJvKExG9jH2mYw6fngYVn3vde8HPaIDy2+NQCnD\/lJzRqIXhN7RsmXLIkFxeh9DNptPGWDovA8btg566nUZZ3Ws4krKuSEq03zLX8PijzZEIZqXZKT32nq1mYz08FOpzYatgpJ+mbJxRgLCI0Fwi4f8IPlgWEYkRL+XYX9KJz3GbQdZsWIFh2nCyJatcEVMhmUuglr+OmXg\/NzRV+Fz3\/p1ZXZMlvpD73ozPDLzPVUk5BUZZe3WR2HxcP6iG2RVh7IMZlaVLa5cGQqnIy\/WZZl1EVSrbxtn9Hy6nvo97Oo9nkg+KGmSFxT3FmXYX27PSA1+\/VJlgKEjtW2F05EtqS7LbAPV6jZN4ozEs2X\/C\/CjXx1LJR4inw++682w5PZxiR5QEgJl2F+ph6ulqUIZYOiopkmF05EjT12WOQ9axcsWwRlJZ\/9vX4b1uwYySYckQ8\/n7665Av5NSEQXlboM+6siI5UwjVdg2\/36FVWgvPWKGEnePkyXH0kyI+H86cxr8B89h+G3x04rkw55PfVXXBLlfejfJrH2goxMvpBOW2WAoSPvSDISHRxs1w0JZ1pI2Plfv4bdv3sNjhw\/XVlcqIoTejtIOn877k3wzxNrc4dbqv3I5cqwPw7Tio6WVC8kIyHRWWa9wSey+d4vX4Tv\/c\/RQmQjhlhIOkvvGAd1V1zijHSCyRmJ11uT0C5m0rCvMphZRzXZsHXQU6\/rCmcimt4\/vApf6jkcCUibSNWlfb0krePBhPKnb62FmxveVKQZZ3XKsD+l663Fa4ToYkee2h+uF66MxKQ2XogyE8m8dvYsPP7UC\/CT545rE42Ys0Hv5q4br4Kma8dUvJsQcfaKjJI2xPJG2Xg6CFHhRpLMRDIY4nRs74cne8+RTJE8jTzC5NUg0Yx7Sw3ce1s9XPPWS5W\/CyHi7BUZIdLoBW3durXqDGwM1fh6a\/aMlK1RoyCRzOCJP8G3978AB\/qH4OgpyJ0IThJBJJrGq0bBZ6e8HS5+w0VRcXmLRNHXYDJSQy4zgS0fsMbXW7NnpKZayaWIYI6\/+mf49r4X4MBvXzbmxVCvIsm8fcwlcNcNb4XxV40ySjKqODAZqSGVSUZqzZgvVYabqPMWISqcCZmJWAi7geOn4Zs\/fz5aN4OPTtI3bjxqL78YLr744mi6+721l8HCv38HnPrza6WQjKq+mMBZtS9T5cqwPyYjQ6MXosKlyYwkg97F08+\/At0Hj8Lu88eRmsjBZOVkpn7gykQvZqThbEj9jDfjFRnRSuz6+nrrm2LjkCwDDJ0R9d1IyIMZM+ov4Us\/Ogw\/6\/sjnDp92mj+RcRPDJMwPPrQNW+GCfWXa+dhfMc5TodClLkM+0v1jOLWGcmXLeoYcFrdMsDQeReXCieGRs\/\/8U9RWNT\/4injeZckcsG\/X\/e2y+BTE2vh0jf+hdMQySXOOvog1g1R5jLsL3eYhpc64iwbHzs7XFV1FE6cll774yPw3B9ehSND9snlzJkzlfzLB+pGw7wP18HZs+fey9RMkimDpnZ0cDYti2p7IcrsHRnFeUa1tbWVqf64wRBvoE1arS23G7fxtgwwVJUryxUncjl28s\/Q\/Yuj8FT\/H616LSJ5YGIXLgK49qpRMOvWWnhf7WXRNHgcuYRoJCyzjpaq1y3D\/lLPM5o7d24kvaoXhHkmut4I751funQp4IH+8rVGYrmk67HLACNrqNCof3fiNHzjZ+dmi2wkc0kGMedSP7YGrr3qUvjHD1wJb7jI3BoYNuysETfze4g4l2F\/ucM01eEh72fmzJlVh\/f39vbC6tWroaOjI\/Ho2jLAQLLBEAmPdDBJNCKxIH7jrx4FM2++Gm6qv3zYFTGq2JoqF6KRsMymRj+9nTLszwoZUaiWFKapLKR0AcYvf\/cKtH6nt9BaGJFgbnrH5XDzlWfgusZ3eJtryQot3ai4fi9MRvoYqrTgwv5kOayQEXWCpLN79+7UpQG0hAC3l4jXHyEY+PT09Khgp1zmCz0vwneefiW1PC6sw+emt10Cn3j\/aHjfVX8Fgy+dAfp7XOWBgQHA0DSkh2V2M1oh4dzU1FQBxZsbZU0Mk8rMm3hZpJhbMsnMSXdF4TvS4VW6J+bxF9uExmS3wThnY2SihEn7U5Un94H8abv25d9wCQA+8qZa9JjwQfLB\/BHdVisms02BMe3R\/VVhWJ5bElSBZCNRRUqvHOOsh59qbVP2p9oflit0BnbaAWtJU\/siAclT+3Gbb3XBQG8I7xCnBwkItxl8Ydo1efBRLstGogyVVkHGWQs+5cq69qfckVAwt2dUpJMidYqCQXdItf3noUq3WRfWFZFPrsNGYgLF7DYY52yMTJQoan86fVtNYOsIVgQM+Rpf9IYemfGeaF+U7YeNxDbC59pnnN3gXMT+dCWLDdNWrlwJLS0tsGjRIjh48GBVH75eVbRmW390pzg+SEQHlt+qi49yfTYSZai0CjLOWvApV\/aCjJSltVwwLxi7njsB0768v0JE3Z+50emaHzYSywpxvnnG2Q3Oee3PhFQjIkwTk9U2ZspUgGYjUUFJvwzjrI+hSgtekVHazbI+hWnyGiL0iFzkiOQBZSNRUXH9MoyzPoYqLXhFRkkC49qhyZMnV+03U3nBPGVUwRDDs89Mfjs8cKedqfss2dlIshAy8zvjbAbHrFZU7S+rnTy\/5w7TfLqqyIfwjMBmI8mjdsXLMs7FsctTMwgyUtnikeelk8qqgNF98A\/w6Y3\/GzVRVnjGZGRitNXbYDJSx0qnpIr96bQfVzfzPKO4qf3Ozs7SwzTZK3I5jR8HJBuJadWMb49xdoOzV2Tk5pWTe8kC48nnjsOdXz7ghVeEQrCRuNEYxtkNzln2Z0OK1JyRvIkV95d1dXUpn\/yoI3AaGOgV4ZoiOk61bK+IyUhnpPPVZTLKh1fR0l6RUdJJjSpnFBUFQKyXBoY4g\/ad+TfAhxuvMNGlVhtsJFrwKVdmnJWh0iroFRklzZr5MJsmHguCXpEPN1mwkWjpvnJlxlkZKq2CXpERvgl6QQ8\/\/HDlNhBaCIknMspnFGm9eUzlNDDGfO7HUQ3cjd\/92RtNd12oPTaSQrDlrsQ454asUAXvyAjfAvNGs2fPhsHBweil4s4eKvS2GZWSwBBDtLKn88VXYCOxoQXVbTLObnD2kozcvHp1L0lgUIjmeld+Fg5sJFkImfmdcTaDY1YrXpGRq9xQEihJYPgYouE7sJFkqbeZ3xlnMzhmteIVGaGwuA+toaGh6hLGrBcx8XscGOJCR59CNCYjEyOu1gaTkRpOuqW8IiMfd+3PeOwX8INnhiKcmYx01Y29OX0E1VoIkUC9IiM1mO2VigPD13wRe0b29EBuOUTDDlFmJiNB8+LAoHzRp2+thS9+4t3uLEChpxAVjmVWGFgDRULE2QsyosS1b2dgi1P6vix0FPU0RIVjmQ0wjUITIeLsBRkpYKtdRL43Le4UABmMu75yAHb2Ho\/6ZjLSHoKogRCNhGU2M\/ZZrXhHRrY2yuLK7v7+\/mgVd9L5SDIYPueL2LCzVNvc70xG5rBMa8krMnK1URbJaNOmTdDW1gY1NTUVfGQwKF80ZfwVsOVfbnAzIjl6YSPJAZZGUcZZA7wcVb0iI9sbZcVQLStME9cXLbl9HCy5vSEHrG6KspEwzkkIhKgbXpERAutioyytZ8KQDTfg0oNg4NPT0wNbn3kF7v\/hi9G\/t86qg9rLL3aj+Tl6GRgYgLq6uhw1yi\/KMrsZg5BwbmpqqoDS19fnBqDzvWQeyG97oyx5SJMmTRq20ltkZt\/zRZwzcqezIXoZIcrsnWdkS8XQ48Jn+vTp0akAixcvhvb2dmhsbIzNGd3wwE+9OtUxDpcQFY5ltqXhw9sNEecLhozyTO2L+aKPXfcW+Mac97vRoJy9hKhwLHPOQS5YPEScLxgyUhlTAkNc7PjozPfAzJuvVqnuvEyICscyu1GTEHFmMhJ0I46MfNscK6pyiArHMjMZJSHAZBRDRuJ518e+eJsb7SnQCxt2AdAKVGGcC4BWoIp3ZIQLEpubm6te5frrr7d+XRGBEcJMGs+mFdD2glWYjAoCl7OaV2SUtP4n5zsVLo5g7HjqacCZNHx8OnyfZ9MKD6t2RSYjbQiVGvCOjFauXAkrVqyAMWPGKL2AyUIEBm0D8XXlNb0zG4nJ0U9ui3F2g7NXZISvLG5odQPB670gGF\/\/wb7o5lh8fJ5J4zDNnXYwGbnB2isy8uHY2Xlf+RGs2XYoQt\/HY0NEtWAjcWMkjLMbnL0iIzevnNwLgjGt7fvw9T3n7mtjMjI\/ImzY5jEdKflEJiNhJBGM6xZ+G3b95kR0fTWSkc8PG7ab0WGc3eDsHRmJ2zamTp0a7SFbvnw5tLa2DttHZgMeBOPyOd+M9qT5PpPGOSMbGhDfJpORG6y9IiMiotraWrj77rth48aNcN9990F3dzfs3r276jA00xAhGCc+vj5qlsnINLrn2mPDtoOr3GqIOHtFRuLhakNDQxUyQpJyMeXfcN0t8NJH10Tj6vu0Phu2G6NmnN3h7BUZ4WvjjbKDg4MwZ84c2Lx5M8yfPx8WLFgQHYKGh6HZfOpv+Qd45UOLoy583pNGGIT49WOZbWrw622HiLN3ZIRwyltCHnzwQSfXXdd+7F\/h9LXTmIws2kuIRsIyW1QIoWkvycjNq1f3cvU\/PQD\/V\/\/B6Affp\/U5fHCnJUxGbrBmMhJwfuunvgpn3vLuIKb1mYzcGAjj7A5n78hIPpERocApfvlaIRsQMRnZQHV4m+xl2Mc4VAL1iozEqX1KVtPfEGDbhEQbZEOY1g9V4ZiMmIySEPCKjGzfm5alBkRGeMwsbpL1\/WHDdjNCjLMbnL0iI5pJk297xen+hoYG6zNqREa+79Yn1WAjcWMkjLMbnL0io7Rd+wSHzRMfmYzsKx0btn2MQw3hvSIjN8OU3AuRUQgLHkNVOCYjN1oeIs4jgozwQLZly5ZFo5zkOcmzdHHliIxCWGPEZOTGqBlndzh7R0Z5p\/bxdtjVq1dDR0dHdFQtbSeRZ94wBFy4cGHq7n8iI59vBBFVI8SvH8vsxrhDxNkrMoqb2sehSyKYuGHFrSRyAhzLyaQVVxfJKIRzjDiB7cagGWe3OHtFRiam9pNm3sRQDiGO2++GZBTKGiMOH9wZSoheRogye0VG5AVt3boVNmzYEB2mhh7N7Nmzo1XYWbv2VQ\/zT7oSicnIvoGHaCQss329wB68IyMUSsWLkeHJsxaJwsFJkyYNW7uEZPTGIz+Bn\/77J92gr9nLwMAA1NXVabbitjrL7AbvkHBuamqqgNLX1+cGoPO9XHT27NmzJntEIpo8eXJ05lHSgwSHz\/Tp0yNvC4+zbW9vH3aULZJRCIeq0TuW8SXRHTeWWRdBtfqMsxpORsko7jps2liLx9USAcmzdEk5o0v3fS3yjvhhBBgB9wgE7xm5h4x7ZAQYgZGAgFHPaCQAwu\/ACDAC5SDAZFQO7twrI8AISAgwGbFKMAKMgBcIeEdG4lKCzs7O1Fm5MhAUk\/RJlxOIZfDeOVqnVYa82KeKzCRb0rov17Kryoyzt+vWrYvEK1tfVGQWT8PwQTeSxlVly5ZpnfCKjMRtIs8++2zsVhLTAORpTxwgrCfuwxONWdx3h+Ta1dUF69evj\/bruX5UZBZlIuMu07BVZRYX1qLu0EWjNTU1rmEGVZkRX3xw0TDK7+JC1Lxg0OJmrOfyQ+oVGYmDg9P\/WZtp84KsWx6\/fKhMSCyo8EuXLoWZM2emem8q+\/B05Uqrn0dmLPvEE0\/Ayy+\/nPleZcuM+rFq1SqYNWuW9avWVd5VFWeRQFV3Kaj0b6oMkupjjz0Gd9xxB9x\/\/\/1V6\/9M9RPXjndk1N\/fH301fAkXRNDEjb\/4dyQjeeW4DHLZCqcqM+1FpAWoWSRrUylVZCYyovCs7DBNRWbCjFIRru4gLDJWSYuRi7SlWofJSBWp87kXOoVAhYySTi3I0aV2UVUjoZXzeLaUisenLVhKAyoy08dqxowZ0Up+0TMpIxxWkVne+uRrmIZDw2QkxNChh2lle0Rk6yrhQ9IRw2XljVRkJsMmD64M45G95qwQXk4Kly1z2gelDNm88oxGQgIbB1jce2fTg1BpWzWxSm3JRq7Sh+kyqjKLhF+2Z6Qis+wZlS0zk1GG5lI87eu0pzh9K3oOREATJkyIjlkZHBysvKnNiwtUiCBLZgxzfCIjlEVFZnGPow\/6oiJzKFP7F7xnpGJYXIYRYARGJgJehWkjE2J+K0aAEVBBgMlIBSUuwwgwAtYRYDKyDjF3wAgwAioIMBmpoMRlGAFGwDoCTEbWIeYOGAFGQAUBJiMVlByXUV0waWPfG22SxKUJqosefVhpTkNE0+u2llNQ+3Scchmbch2ro7PumIycQa3eUZlkhMa2c+fOzKuoxLfxjYziLg5VRz+7ZNknBGRLGGYJJqMSx01cJEdfcjw6pbm5OZJq3rx5ESmI5fDv6LGMHz8e5s6dCwcPHgSqSycJ4F13VC7plhaxTfrKY1vUd9KXXzxvijZ6EhmNHj06kk32SsQ64uJE3D7x9NNPw5NPPlm5yFNc9DplyhTANumOvjiZZc9EJkbsY9SoUdDT0xNhRZjKwy57mWkEy2Rkx2iYjOzgmtmqvE9J3EIin9MjXuUkbq7E+7jEM5XEs3LQmBYtWhR7Hg2FYg899FBEHLgxFkmCiC\/JsxANVNw7ODQ0FJEYEZHcHu3Zwg2soozyVeniqt+xY8dGZItkinKJv+H9dGIfaV4a9kEXkVKb2J5M0kxGmSprvQCTkXWI4ztIO0mju0I\/AAACBUlEQVQvLUwTCUEkI+wFjZcMLW2PmfzVF\/dIpR1ql3Q5p7zHKk1+8TeRmFB+uZ589o94EFmS5xLnGWHb5F0lycZkVJIhCN0yGZU4BmKyWAyLZIMRj1ZFcalsHBlhKCI+cWfmyEdXqGxQTrr5F\/uSCUCUX74jTww\/ZXJLIycx1KP3i0tSx5FRQ0ND5bZiJqMSFT6jayYjT8Ym6QRA2etI84xUT8a04RmJoV2aRyN7RmlEUeRUxCzPSCY8Gn7ZM0o7a4hzRnaMhsnIDq6ZrcYpP1bCHfRJR2NQghrLtbW1QVrOSMwLxeVH8GSBvDkj0UDR46GwEOVRISOqQ3kg2TNSzRk1NjZGuSdcfoA4iEnsODLCv+FRwXIoKw5SXB6NcJaT5ExGmepdqACTUSHYzFQSQw8xTKNZI5z5aWlpiZK1mITFJHNrayts3rwZOjo6KsaF\/yOey02zaWnHmibNTGVN04shozybRsQgkql4ZAaGVffccw9s27YtIpG1a9eC6BlR3mjZsmVRYr2pqQlOnjwZO5uWtI4ojozwTO8dO3ZE5CViEpejwr4RZyTaAwcOxJI+k5EZ\/ZdbYTKygyu3qolAWo4qremsnJGmWFF1JiMTKFa3wWRkB1dutQAC8nqqpDVBWWSEywzIc8KbLmTvq4BolSq8AlsHvfS6TEb2sOWWGQFGIAcC\/w\/B0APyz5qgegAAAABJRU5ErkJggg==","height":175,"width":291}}
%---
