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
igbt.data = 'infineon_FF1200R17IP5';
% igbt.data = 'infineon_FF650R17IE4';
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
mosfet.data = 'danfoss_SKM1700MB20R4S2I4'; %[output:03d0d520]
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
%[output:03d0d520]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"mosfet","value":"    data: 'danfoss_SKM1700MB20R4S2I4'"}}
%---
%[output:29a5e80b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASAAAACtCAYAAAAQ2qIyAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXXuQVkV2P5hs4qgYHAR1xMmADmqKBMVo2NEqcPFdBW6Rx8BsVchAURQRp8qFIIwYpGpBmDD7B+jusoYgSWVmDOULTKVYMlkoFdl1QbBW3QWFAcfRWhwWUSJJbZbUuXA+++vpe2\/fe7v764Zzqyx1vn6c\/vU5vz59+jXo9OnTp4E\/RoARYAQqgMAgJqAKoM5VMgKMQIQAExArAiPACFQMASYgQ9Dv2rULmpqaoKOjA8aPH2+o1DPFyGV\/+eWXsGbNGpg9ezZUV1cbqQvLXLRoUVTWypUroaqqKqr38OHD0NjYaKSOuEKee+452LlzZ6nepMqeeuopuPfee6G+vj5VpixpUws7m4Bw2rJlC9TU1MCGDRu0ZFHhq1unyXTHjh2DWbNmwaOPPmpcT\/PIyQSUB7UK51m1alVEDuvXr7dGQL29vdDc3AwPP\/ywVQIig5g2bVpqPUhUa9eu1TL6LGmzdOeBAwciXCZPnhwZse7nCwGhvKg\/fX19WoSv27686YIiIOp8BA8\/8jaoc0+cOBH9fceOHQNGJ\/Ii8PexY8eWjJf+\/vjjj0d\/w7KffPLJWGOIk0H0UrB89CZIHsyDI+WIESOiv+Poid+cOXMiJaYyydhlj0f8f\/RIFi9eHOWXR2CVkstklYYhlrtw4UKYN28e7Nu3r0xONOq4urGedevWRTJNnDgRtm\/fXiIK0WvAAkV8ZaIgQqK6Ka3Yf9T3o0ePjkZzWU5VWvRKRfmRQMjTk40nTgb576oy5LZSH6t0VNRDIgbEME5HsSz8ncpMwlyWVfTMbXrrWYkoGAKSjVRUXDLs3bt3R0o\/dOjQSDFra2sjJRNH8ylTppRNNVB5ceokdmqcdyGP1qJx79+\/vzQFIwIieWi6II48VC8qBsorehtJBISGlOQBIS5dXV0RmeKHOGAeFdGpMMQ8MmY4BUP8V6xYAe3t7aVyCV9qC5IF4Su2PQ4naos4Gotpt23bVubxyGSFaevq6qLBgsiFDE1OK2JKxEW4iEZDfUy\/yX2R5gHF9bGsE1in3OednZ1l2JOXRTKQjmJe+psKc7IH6svNmzeX4ZjF68xKKFnTB0NAcSMlKkpLS8uA+IWYfs+ePQMUmYyUiING2iTXHTt9wYIFyimAygMiBcB4SlKnZ\/GA0giIylq9enWkC2JcKguGcVMwlReB8Sj06igeItZDgwEZtIiDPBggTjSqy94BtkXVN3EjvYqsxIElbhqiireJsTHCRTUFS+pj8oCOHDmiHBzIcKn9ooes8lgwXRzmMrmJOoH9IJNsVtIwmT4YApJHfRHENAJ64YUXItdV\/Gj60t\/fn2ikYp40ciJjp9FOJCCZZMRyTRIQKTq2j0ZKihVlwVAmIDICNLylS5fCsmXLovLRW0ICEo1bxImMgabN1G4czTGILnqqSEDyFFEkIpXHhkaIXk8S2cpTX5JBh+TkaW0SASX1sVwO\/r\/onRKxi7jEeWEov9yXIjak0zJR0CBLtkMeLOJeqS8YAsoyeiOgSR6QCLY8uuiSjLzSpesBqdx+kwQkegrDhg0rTb9UHkQSicsEJCo84it6BVk8IBH7pMCsGEuhqYeK2OLiZmkeUJzBmfCAVH2cREDyACqTU1EPSG4re0A56FYnBkSjIc3xs8SA4mIHoqhyx6lGHSxH5QHJoxaOUhQDuPvuu8tGQ3LDSSZZAdNWwUQvQgw+6mBIXo1MQKq2UhBWjAFRW44ePVqakqXFgMh7koktSQZxakcGTP1PAWdxxcxlDIjaI\/axPN2USUaOfWGwn4hXRUBiDEjGnGNAOQhGJ4toWOIKkDi6DB48OHLJZfc6bRVMh4BQxiyrYOIUDP87boWEvBNaYaLVjjgCEtui2nckxxvEvUI6GOK0Cj9asUOiEVfGEHv0rvATp3c2VsHElSZRdpxO4EeYYX8j6ZFHJKcVA9WYL8sqmIrE45bh01bBSCdkApL7BfGVg\/xyX\/MqWAxrIJiosG1tbQM2ackdpJqH65CRriuftSxOnx2BrB6ayqv0ZWNc9tZXJkcRzNGb1d34abt1xmNARDDyEjQ1BKcx8+fPh9bWVq0dpDoA+LTJS0fecy2NPKhg+7LsCPfJIELpm7yYn\/M7oVGZ0BVGAlKRjLifxNQxglCUhuVkBBiBcgSMekBILhs3boS5c+fCkiVLlAQk7iNBUZJ2HXNnMQKMwLmNgDECQpdw+fLlMGPGjGhnr840yzd38Nzuam4dI+AfAsYISF4dwqamnRameWxDQ0PZ2atRo0b5hxRLxAicBwh0d3fDyJEjnbXUGAHJKxtxHhBOwfCj80Wq1TIkoIMHDzoDoWhFocmL7WWZi\/a6Xv7QcHYtbxkByedqkiBOWj6XV7pE0kk6GU31uQZBT5XiU4UmLxNQ0R7Xzx+abriWdwAB4RkfPOuTtEKFBKOTTr+bylO6BiGvnJTv0KFDTt3WovJifpbZBIrpZYSGs2vbszIFS++W5BSuQSgqb2hKxgRUtMf184emG65tTzkFQ3hN3ran311nUroGIat8cvrQlIwJqGiP6+cPTTdc294AD0iO0aStZOl3hX5K1yDoS6ZOGZqSMQEV7XH9\/KHphmvbS52CVWLjoGsQ9NWJCagoVkXyh2bMIRK9a9tLJSBRYWwHn6ku1yAUMYoQlYxlLtrj+vlDI03XtpdIQPKyvKvpmGsQ9NWJPaCiWBXJH5oxh0j0rm0vcR+QK8KRldI1CEWMIkQlY5mL9rh+\/tBI07XtlREQBqDxn0qfUncNgr46sQdUFKsi+UMz5hCJ3rXt8UbEIhZxNi8bhgEQNYpgnDVAKpik4gQkPvSW1BYTNxnGle8ahIJ9xruKiwKomZ8JSBOoAslc216mVbAC7cqU1TUImYRTJGbDKIqgXn7GWQ+nIqlc2x4TUJHe4imYAfT0i2AC0scqb0omID6KkVd3MuVjY84EV+7EoeHMBMQElFvZs2QMzTCwbSxzlh7Ol5YJiAkon+ZkzMXGnBGwnMlDw5kJiAkop6pnyxaaYbAHlK1\/86b2hoDEU\/H0MmbcSxd5GxuXzzUIReVnYy6KoF5+xlkPpyKpXNuechWMyAePYkydOjV6auexxx6DzZs3O3lR0TUIRTqMR+ai6OnnZwLSxypvytrb7ocjP\/2PvNkz51MSkHjqvb+\/v0RASEw2r2Il6ZmAMvdj5gxszJkhy5UhJJw73\/wEHup8D459985cbc2TKXYfED5839fXBzNnzoRNmzZFjw3OmzcPxo8fD\/iOt82PCcgmumfKDskwCA2W2a5eeEVA2NRdu3ZBU1NTqdWuXjFlArKraExA9vENkTS9IyB33VReExOQfeTZm7CPcWhEv2prD6zaesiPKZiL7sFpHn7ylI4JyD76TED2MWYCSsc4NgiddCrexEVlNL2bM2cOE1B6PxlPwQRkHFJlgSHh7JUHhJfR9\/T0lJED\/W3ChAnQ2dkJK1euhKqqqsw9Sats9fX1cPLkSSagzAgWzxCSYYQYTwlRZm8IKO7yefp7S0sLrFmzJvUF1TgzwakXktjhw4cHkBzm4SlYcYJJK4EJKA0hM7+HhPOUp9+C1z44XvkYEG1E3L17N2zYsAHQUzlw4AA0NzfDLbfcAg8++CC8\/PLLuTwgnHrt2LEj8npUXhYTkBnFTyslJMMI0ZsIUWZvCIjAk5fhOzo6YPTo0TB\/\/nxobW2NiCnrh97PunXryrLJcSD0gPDr7u7OWnxF0vf29sKIESMqUnfeSlnmvMhlyxcKzndOboQT95xZFPJiI2I2mPOlZg8oH24mcrEHZALF9DJCwfm194\/DlO+9xQTEU7B0pTaRIhTDENvKMpvoeXUZT7zyAaz5ryP+EJD8JDOJbfMyeqqDg9D2FC3E2ATLbF8fqr\/946iSC\/77U\/j0B39pv8KzNcTuA6I4zwsvvBCtWOEZMIzf1NXVQWNjo1UBmYCswhsVzt6EfYxDwPnIsVPR1Av\/jd+Ql2bBwYMH3YADAKmn4bdt21ZaKue34dX9wsbsRl8ZZ\/M43\/SdN0rk09xQAy8uvL\/yBETL8A0NDTBu3DhYuHAhtLW1wZ49e6CrqwvWr19v9fVU9oDMK5pcIhuzfYx99oDEoDPKWVt9Iexd8nXne\/Bir+MQvR30ghYvXhz1GC7F43TM5scEZBPdM2UzAdnH2Eec5SkXyjih\/jJ4ce5NESCubY\/fBTOgh2zMBkDUKIJx1gBJkQRJp+1HPdDx04\/LfkWv56lpN8Id1w0p\/d0LAko7irF06VKeggldyYaRzzCy5mKc9RFD0sGrNfCOH\/lD4nl+zli4dthFA36rKAEh8aS9DY8X1Oc9hKoLn2sQdOWKS8eGURRBvfyMczxOSDj\/sqsPfnLos+g8l+pTeTxyOte2l7oKVl1dracdBlO5BqGo6GwYRRHUy884n8EJyeY0ADzc+V4s2RCiOqQjou\/a9jgGpKf7ianYMAyAqFHE+Ygzks2Wt4\/C1nc+TSUbhBAJ564bhkLLN2qj\/876VZSAdKZgvBN6YJeej4aRVbFNpD9XcUaSQbJofekA\/PyjL7SIhsgG\/902dTTccOXFuQjHyymYCWUpUoZrFi4iK+Y9Vw2jKC6m84eMM+003vZeP7y891dw5NenShsAdXBCgqq97EKYefvVMK72UiNko6rXte3xFEyn91PShGwYBprvrAifcSaC2fvh5\/CPr\/VGmMQFg5MAI6KZeH01\/MW4K6wRTZwM3hCQ+DQzCetiBQzrcg1CUQvy2TDi2sYyZ+t1mibhtaWvv\/\/rzB6MWBvFZibdMBQevvMauGDQIOdE4zUBiU8ziy9W0GOFvAxf3n1szNmMOW9qWziT9\/L9HR\/CO31f5PZeqF1EMDhl+tOrBsHfT70pmm7lCQrnxSpvPteDf6ZleD6Mqu5WW4aRV4l08p0PMovTou5f9sOho18W8lxUBPPAHw+DB8ZcHv2kIpjQcPaCgBBM9Ha2bNky4E5onIbx08zsAemQnOk0aMy\/8wdXlYr92eET8OwbHwFuiska1I2TTfRexo8aAt\/6s6tgUAy56LSPCSgZpcQgtHwpGT\/NzB6QjtHlTYMey6df\/C+sf\/0j+PDYKWOkInouOC26pvpCwPjL1JuHW58aMQEVIKC8ilQ0n2s3sKi8oSkZtte2zDT9wbr6Pvuf6JiADVKhqQ8RC3oteLo7bkpUtK+z5reNc1Z50tK7tj1ehk\/rEY3fQ1OyvAREpPKb356Gl\/b+CrbvP2Z0+iNCLU6FaodWQdOtV8LpL47CNddcE0Qwl9oSmm54QUC0I7q2ttb6wVOVfbsGQYNjEpOEpmREQGI8Zc+RE\/CfvzgGR\/rNBGpVgImkcsNVF8M3rq+GP7rqEm1vJVScR44cWVTFnOV3bXuxHpBqH5DqHXcbyLgGoWgbfDEM8lCuHvL78MNXe+Gdj09aJRRxmhMtOdddCt8cOxyGXPQ1K16KLzhn0ZfQZHZte5mmYPhQIa6O8ZWs5SpoQ8mITNBrePPwCXjl7aPw1pETUcWmVnzSvBSc+jw4dhjcfeNQ68FaHaO2gbNOvUXShCazNwSk8oBqampKy\/KqThFfUo3bNS2Xqzrc6hqEIgqmG0+hjWg9\/V\/Czg+Ow+sfHC8FZSNSOfsqQVFZ5PzitOfa4RfBg2OHw6jLq+D\/Pvs4WtIOYXNcqPEUXd0w3edFynNte7EbEfFiMvx0vR2MG9FTPvhM8aJFiwAvtZef8BHTxT3t7BqEvB2GpPHb06dh9b+\/C0c+v8Cpd4KV1V9xEUy9+Qq4\/dohJQLTJZTQRuYQjTlEmV3bXqYpmK6hkpczffr0ARfYHzhwAFasWAHt7e2x17q6BkHVLiSXzjc\/htffP25tyiN6J7g3ZUzNJfDAmGEwCHe+Fdj8ptNPTEA6KBVPExrOrm3POAHRNCxuCqazudEVCEgyl1\/yNZj2zNu5Ti6L6il6HhiQHX3FxfDn44bD1UO+uhRK1zsprvbpJYRmGCF6EyHK7Mr2SEONExAVjESzc+fOxGV8Wu7Hox3iUz82QcAYTEvXL7QJR\/RS8B6W5tuvjrbmix4KG3M64ZlIwTibQDG5DJu2p6rZGgHprJiJDyCKsSIEAb\/u7m4jiO\/+6BSs+8lxwH+rvppLfzf684SRF0HTTZcC\/b9u5b29vYBxr5A+ltlNb4WC86RJk0qAePU0s3gpfdJpePk3XK7HTz64ip4Rfkg4GA+iV1fFgLQpFlY9wkaeyx3XXQYL76kzsgrEI7MbY2ac7eNsyvZ0JS3zgHTuhE66lCxuGV4kHXkZXnXAtSgISDzzpBcDcCp15+hqeOSuPzRCOiLAbBi66lYsHeNcDD+d3EVtT6cOMU2m+4CyFp43fREQVG9ef7\/pxigYbCsIzIaRt6ez5WOcs+GVJ3UR28tTn7UYUB5hKE9eEFTks3fJ14uIopWXDUMLpsKJGOfCEKYWkNf2UguOSTBgCrZs2TJoaWmBBQsWwL59+wZk8\/VZHpl8Xpx7U+lahrzg6OZjw9BFqlg6xrkYfjq5K0pAOgK6SJMVBIz53PSdN0qibf7bm+GO64a4EDWqgw3DDdSMs32cs9peUYmCn4LJK12uyYcJqKgK6udnAtLHKm9KLwgoaTXMtynYI5t+CRvf6IvwxmX1Rfe5v3uFDSOvumfLxzhnwytPai8IKE5w3NszYcKEAee78jQ0KY8uCOLUC1e4XAScVXKzYZjWAHV5jLN9nHVtz5QkmaZgPj3LI+\/1QfKxtcyeBjYbRhpCZn5nnM3gaGLwNyVJJgLSOV5hQjBdFq7+9o+j6h5\/YFS0wbBSHxuGG+QZZ\/s469qeKUkS7wNSLcN3dHRUfAomBp7R68HAc6W8H+wINgxT6phcDuNsH2cvCMh+M5NrSANB3PPz7IwxMGXssIqKzIbhBn7G2T7OabZnWoLYKZh8UBTPc3V1dWnfkFhE0DQQpjz9VnSdhg\/eD3tARXo6W14moGx45UmdZnt5ykzKoySguBsNde74MSFgEgjiytcd1w6BzQ\/dbKLKQmWwYRSCTzsz46wNVe6EXhBQ3GqXD6tgf\/f8\/ujpXvwqufIl9jAbRm59z5SRcc4EV67EXhAQSo7eztq1a0uvYNDmRLy5UL7jJ1dLEzLFgeDLvh9ZdDYM0xqgLo9xto+zNwSETcU4UHNzM\/T1ndlprLq7xwYkOgRUiSMXcW1lw7ChBQPLZJzt4+wVAdlvrrqGOBAo+Iy5mICK9Q4bczH8dHOHhrMXBOQq1hPXiSoQfJ1+YRtCUzKWWZc+iqcLTTe8ICCEHc991dXVDXhYsHiXpJeQRkBPT78Rpt96ZXpBjlKEpmRMQI4UI8DByQsC8vE0\/Oa3j8LfPPvzSHN8Wf0iNWYCcmPQjLN9nL0gIPvNTK5BBYK4+bBSp945CF1ZzWACso8\/ExAAyCD4uPlQVAU2DPuGwdNGNxhXlIAo+OzbndDi2S\/f4j9sGG4Mg3F2g3NFCchFE+V3wVSn62UQVm3tgVVbD0Xi+bT8zjEgFxrzVR3sadrH2xsCsnUYFXdY9\/T0RLup4+4XkkHwOf7DI7N9o2Cid4exFwTk6jAqElBnZyesXLkSqqqqSiiLIPge\/2ECcmcc7AHZx9oLArJ9GFWchqVNwcT4j4\/TLyYg+0bBHpA7jL0gIGyui8OotN8Ip2N4yJU+EQQx\/uPb\/h82DHeGwUTvBmtvCAiba\/swKnlCDQ0NZTuuEQT8uru7Yc6Ln8DPek9F\/7\/74To3vZCxlt7eXhgxYkTGXJVNzjK7wT8UnCdNmlQC5ODBg27AAYBMl9KbkAo9K\/waGxsjglu4cCG0tbVBfX290gPCF08xDlTJZ3fS2s2xiTSEzPzOOJvBMakUrzwgG83NsgwvBqDvunEo\/NvsP7EhUuEy2TAKQ6hVAOOsBVOhROc8AemgQyCIBPTovSPh0Xv9nIKxYej0avE0jHNxDNNKYAISjmJ0vvkJPNT5XoSZrytgKBsbRppam\/mdcTaD43k9BdOBkFgYyQdJCD9fV8CYgHR61EwaJiAzOAZBQLhJsKmpaYCsY8eOtf40DxGQ7zugCRw2DPuGwUTvBmMvpmBx+3PcQPDVaXh6etmX53fi2s8E5EYzGGf7OHtDQMuWLYOlS5dCdXW1\/VZLNRAIREA+B6B5ZHanHkxA9rH2goCwmeKhUfvNLq8BQfjnH+2BKd97K\/rBxys4RInZMNxoCONsH2cvCMiHK1mXP7cziBUw9oDsGwXH2txh7AUBuWuuuiYE4a\/at8IPX+2NEvi8AsYE5E5b2AOyjzUT0Nl9QGPmPw+vfXDc6yMYPDLbNwie6rrF2BsCEo9MTJ48OTqztWTJEmhtbS07t2UDHgTh0pn\/Gp0B830FjD0gGxqgLpM9IPtYe0FARD41NTUwdepU2LhxIzz22GOwefNm2Llz54ALxEzDgiAc\/+b6qFgmINPonimPjdkOrnKpoeHsBQGJF5L19\/eXCAiJycXyfN2Y2+DEPauivvR9CZ6N2Y0hM85ucPaCgLCp+DJqX18fzJw5EzZt2gRz586FefPmRReH4QViNr\/a2+6HL+5YGFXh8xkwjgHZ1IKBZYfmTYRImt4QEIInH8d48sknnTzVXPPAI3DqhilMQBbtm43ZIrhC0aHh7BUBuemigbWIBOT7EnyIoxzL7E6zmYCSsXZ+I6JO1w\/\/6x\/Aby6\/PogleDZmnR41kyY0Yw5RN7zxgOSbCxFMXI6Xn9Axo1rlpTAB2UC1vEw2ZvsYMwGlY6z0gMRleAo409+wSNskFMopeA5CpyuYyRRMmibRVJflhQdk+12wNBiJgKbfemV0ENX3jw3DTQ8xzvZx9oKAsJmqV0txab6urs76ShgRkO+n4NkDsm8QYg1MQPbx9oKAkk7DEwQ2b0ZkArKvaGzM9jHmGFA6xl6ughEBhbAJMUQlY5nTDcNUitCI3gsPKC\/4eInZ4sWLo+xxHpK8uqZKRwQUwh4gNua82pI9X2jGHKJueENAWZfh8ZXTFStWQHt7e3SNKx3lkFfMcHo3f\/78xFP1REDHvntndi2tQA42DDegM872cfaCgFTL8Nj0OFJRwaIKYmM6mahUeZGAfH6KWZaZDcO+YYToTYQosxcEZGIZPm7FTJymYQepzpchAYVwDQeZHRMQE1AcAqHphhcERN7Oli1bYMOGDdEFZOi5NDc3R7uh007D615oH\/f8DxOQfYMOzTBC9CZClNkbAkLwdLwV2VSy7BWiqV5DQ0PZ3iIkoN878jq88Q\/fsm+JBmro7e2FESNGGCjJXREssxusQ8F50qRJJUAOHjzoBhwAMLoMj+QzYcKE6M6guA9JDb\/GxsbIq8KrXtva2squeUUCCuEiMmqj61HDhHawzCZQTC8jNJxdy2uMgFRPOdPhVbzKlUhHXl2LiwFdtOefIi+IP0aAEXCLQLAekFuYuDZGgBEIHQFjHlDoQLD8jAAj4B4BJiD3mHONjAAjcBYBJiBWBUaAEagYAl4RkLjs39HRkbiaVgnExEB73AX9Yhp8V432UVVCXqxTR2aSLW5flkvZdeXFFdd169ZFolVaV3RkFm+Y8EEv4vpU56iUSX3whoDEIxr79++Hzs5O6zcvZgFS7BjMJ557Ew1YPOeGhNrV1QXr16+Pzse5\/nRkFmUio66UQevKK250Rb2hhzOrqqpcQwy6MiO2+OEmXpTfxQOfWcGgzcaYz9XA6Q0BiZ2CS\/VpB1azgls0PY5yqERIJqjoixYtgunTpyd6aTrn3orKlZQ\/i8yY9pVXXoHPP\/88tV22ZNaRF3Vj+fLlMGPGDOtPhOu0U0dmLEckTd2TAjr1m0qDRPrMM8\/AfffdB0888cSAvXmm6pHL8YqAenp6ohHCh6mADJR4uBZ\/QwKSd3DLeSqtaLoy09k\/2hSaRqy2lFFHXiIgmnpVegqmIzPhRSEGV+\/r5emnuM3BecrSycMEpIOSdEWtDgHF3QagWZ2RZLrGQTvY8W4mHc\/OiHCKQnTkpcFp2rRp0W560QOpxDRXR2b5yJGvUzDskvOagGheHPoUrNKeD9m2zvQg7vrdSsSBdOQlYyYvzbXBqDzjtKm5HNittMxJA4hr2bzxgM6FIDTN9fHfODpX+tMNkJKcsnG7ll9XXpHgK+0B6cgse0CVlpkJKAYBmiP7ukwpLreKHgIdsB03blx0ZUlfX1+phTYv79chiDSZRaKsNAFhe3TkFc8T+qArOjKHsgx\/3npAOsbEaRgBRuDcQsCbKdi5BSu3hhFgBHQQYALSQYnTMAKMgBUEmICswMqFMgKMgA4CTEA6KHEaRoARsIIAE5AVWPMVqrt\/yMYRDzoHhCt4unuAfNhsSUjTSpStVUcqn275rMS5s3xa5XcuJiCP+qeSBIQGtmPHjtQXT0S4fCMg2weYK33w1SNVNSYKE5AxKPULEveN0IiNNwA0NTVFhcyZMyciAjEd\/h09k9GjR8OsWbNg3759peev6XAsPqNE6eIeBhDLpNEcy6K640Z48aoUOstEBDR48OBINtn7EPOI+3Vw5\/C7774Lr776auldOHEP2MSJEwHLpOefVDLLHohMhljHxRdfDN3d3RFWhKncS7I3mUSqTED6Oq6bkglIFylD6eRt+eIrIfI1E+KLIeL5IXzqRX4GG8Uj0lqwYIHyOgWaZq1evToiCzz3hcRA+eI8CNEoxWMy\/f39EXER+cjl0REFeqqbZJRf2BU3vw0dOjQiWCRQlEv8DZ8+EutI8sawDnrXjsrE8mRiZgIypNg5i2ECyglc3mxJFz4lTcFEEhAJCOVAgyXjStrNLI\/u4pGApDuY4t56k48UJMkv\/ibejYPyy\/nkqyvEu3PiPBSVB0SEp6qD+o8JKK8mm8nHBGQGx0yliAFfccojG6J46x9WQGlVBITTDPFTXfkgn8LWOX8X93gk1iUbvSi\/\/PySOLWUCS2JkOTHMbEcVaBZRUB1dXWlM3lx5MgElEl1jSdmAjIRCIQ1AAABb0lEQVQOabYC4y6qkr2LJA9I9\/I2Gx6QOG1L8lxkDyiJHPJc3pXmAckkF+cBJV2VwTGgbLqtk5oJSAclg2nkETcuBqS6mgLFWLlyJSTFgMQ4jyregYdls8aARKNEz4amfCiPDgFRHorryB6Qbgyovr4+upUStwogDmIgWkVA+De8wVKepordqYqLEc5yoJsJyKAhnC2KCcg8pqklitMKcQpGqz24YtPS0hIFXDGQioHi1tZW2LRpE7S3t5cMCv9DvCKWVsGSbtyLW1FKW1IXp4PyKhiRgei5iKe\/cco0e\/Zs2Lp1a0Qca9asAdEDwnYQJpgW3yk\/efKkchUsbp+PioDwetnt27dHhCViooo5LV68OMIZyXXv3r1KomcCSlXtzAmYgDJDxhlsIpAUc0qqNy0GZEJmJiATKJaXwQRkHlMuMSMC8n6nuD07aQSEWwLIQ8IL1mUvK6NYZcl5J3QR9OLzMgHZwZVLZQQYAQ0E\/h90xefphMrZdQAAAABJRU5ErkJggg==","height":173,"width":288}}
%---
