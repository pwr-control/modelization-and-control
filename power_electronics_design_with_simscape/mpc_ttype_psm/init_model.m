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
model = 'mpc_ttype_psm';

load_step_time = 0;
%[text] #### local time allignment to master time
kp_align = 0.6;
ki_align = 0.1;
lim_up_align = 0.2;
lim_down_align = -0.2;
%[text] ### Enable one/two modules
number_of_modules = 1;
enable_two_modules = number_of_modules;
%[text] ### Enable Thermal Models
use_mosfet_thermal_model = 0;
use_thermal_model = 0;
if (use_mosfet_thermal_model || use_thermal_model)
    nonlinear_iterations = 5;
else
    nonlinear_iterations = 3;
end
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
%[text] ### GATE drivers settings
positive_voltage_rail = 12;
negative_voltage_rail = 0;
dead_time = 3e-6;
use_deadtime = 1;  
use_deadtime_cmos_based = use_deadtime;
%[text] ### Settings for speed control or wind application
use_torque_curve = 1; % for wind application
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
fPWM_AFE = 6e3;
tPWM_AFE = 1/fPWM_AFE;

dead_time_AFE = dead_time; %3e-6;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;

ts_afe = 1/fPWM_AFE; % Sampling time of the control AFE as well as INVERTER
tc = ts_afe/200;

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
deepPOSxi = 0.5 %[output:0581fd83]
deepNEGxi = 0 %[output:14dd847c]
deepNEGeta = 0.5 %[output:4d98dff4]
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
l1 = Kd(2) %[output:0cb9abdb]
l2 = Kd(1) %[output:3b84529d]

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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:4052b205]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:341db486]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:4b2a82f6]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:76aaa531]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:0f98c5ff]
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;
kp_rc_pos_grid = kp_rc_grid;
ki_rc_pos_grid = ki_rc_grid;
kp_rc_neg_grid = kp_rc_grid;
ki_rc_neg_grid = ki_rc_grid;
%%
%[text] ### Settings for First Order Low Pass Filters
%[text] #### Second order LPF 50Hz in state space (for initialization)
fcut = 50;
fof = 1/(s/(2*pi*fcut)+1)^2;
[nfof, dfof] = tfdata(fof,'v');
[nfofd, dfofd]=tfdata(c2d(fof,ts_afe),'v');
[A,B,C,D] = tf2ss(nfofd,dfofd);
SOF50Hz_ss = ss(A,B,C,D,ts_afe);

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
dead_time_INV = dead_time;
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
p2place = exp([-10 -40]*2*pi*ts_inv);
% p2place = exp([-40 -160]*2*pi*ts_inv);
Kobs = (acker(Aso',Cso',p2place))';
kg = Kobs(1) %[output:7248d0a2]
kw = Kobs(2) %[output:2639405f]

%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:2d8fc428]
luenberger_l2 = Klo(2) %[output:6c675df8]
luenberger_l3 = Klo(3) %[output:0e8d59e9]
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

k_kalman = 1;
k_dc = 0;
k_sw = 0;
%[text] #### BEMF observer
emf_fb_p = 1;
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
m_scale = 2/3*Vdc_bez/ubez;
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
mosfet.data = 'danfoss_SKM1700MB20R4S2I4' %[output:7d7dae6a]
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
figure;  %[output:6fdddf8a]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:6fdddf8a]
xlabel('state of charge [p.u.]'); %[output:6fdddf8a]
ylabel('open circuit voltage [V]'); %[output:6fdddf8a]
title('open circuit voltage(state of charge)'); %[output:6fdddf8a]
grid on %[output:6fdddf8a]
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
%   data: {"layout":"onright","rightPanelPercent":15.7}
%---
%[output:98950895]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:9dd00217]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:0581fd83]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepPOSxi","value":"   0.500000000000000"}}
%---
%[output:14dd847c]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGxi","value":"     0"}}
%---
%[output:4d98dff4]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGeta","value":"   0.500000000000000"}}
%---
%[output:0cb9abdb]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  57.836573118544621"}}
%---
%[output:3b84529d]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.239974016359851"}}
%---
%[output:4052b205]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.239974016359851"],["57.836573118544621"]]}}
%---
%[output:341db486]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:4b2a82f6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:76aaa531]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000166666666667"],["-16.449340668482265","0.997382006122009"]]}}
%---
%[output:0f98c5ff]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.259181393921158"],["45.276810189997427"]]}}
%---
%[output:7248d0a2]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.051440061337114"}}
%---
%[output:2639405f]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   2.564084947360490"}}
%---
%[output:2d8fc428]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.525517392393834"}}
%---
%[output:6c675df8]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     3.034255806063732e+02"}}
%---
%[output:0e8d59e9]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -3.701967948541575e+02"}}
%---
%[output:7d7dae6a]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"mosfet","value":"    data: 'danfoss_SKM1700MB20R4S2I4'"}}
%---
%[output:6fdddf8a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANUAAACACAYAAACcAyzHAAAAAXNSR0IArs4c6QAAF\/9JREFUeF7tXQ+MlcdxH\/IXYoMI\/2qfyXGADXZiBWyo7WAs23UxUlKOiBJxh6o6x5lciTEq5Wz+GgQCbP5ZMuAYYs4nIoUzpXUEqLWIY4UrmCAnxqC0IfYlcGACTeAuCFJAFg3tLJ7HvH2737\/99r13780nRTH3dufbmZ3fzuzs7HzdOjo6rtXX18ORI0cg7BkxYgQ0NTVBnz59wprK7yKBspVANwTV0qVLYcmSJYFg6ezshCjtylaSwrhI4BMJdLt27dq1ri6NgwcPwtSpU2Hbtm3wwAMPpMLO5cuXYd68eYrWCy+8AD169ICNGzfC+PHj4Y477kjlHatWrQIcO1l\/\/O8TJ07AlClTUqGvE2lra4Nnn30WVq9eHcgD8r5+\/XqYPn16qFeS9phxjHV1dXD69GmYMGFCRvZBAjHNlRcBWoiiDHAuaR6VpUL3Dx9x7exTsX37dtiwYQM0Nzd7AVVHR4dSpqefftobqHDiUVlpkbBxq4Pd1o4AkOaYUc7z58+PtUAWGlToxSGGampq1NwpS0WD2r17t5JfRUWFVXmIAO3Bnn\/+eUWIaFy4cEHRaG1tzaKj9yOrQlbmueeeU6DGSSea+mTq42xoaIC5c+eq1Z4sVd++fZVyPvLII7B3714YNWqUUiJceTdv3qxI0gp46tSpLEXmdHD\/SJZq0qRJqh093CKaJlRXSvy3\/m60fNRuzZo18Mwzz2T2tcgXWgm+1yVecQykePjf+HekTWOyydkEANO4du3apZSa68GhQ4dy\/oZy1sc3a9YsJTPSI9s8Im3Tu1GncB7pMfWnOeLjGzhwoHov1z2+\/+fWD\/sRXaJFMtTl\/vDDD6uh9OrVS+kQPjb++Jwb3T8+aXwQNGHoYunKTEr43nvvKUCS0CsrK2Hx4sWwbNkyNSgcHE4crfq4QqMgSWmCVkm+0tIEoDLho4OKwITKi\/y8\/vrrCrTcItx7772RQKWPWXf\/OH0cCyobyYjzE\/SbbqmwX1VVlVqw+OpNiwYuDDgHpJwoB74QmOTc2NiYWSxNCxFZHD5mHNfKlSth3bp1Ss7IG84p0tcXJd7vww8\/tLrkQTIJslQEjrVr12bxSvpFuofjJKtfXV2tgFBbW6vmBN+NoEcd1XWPwE6\/Ex2+MJO7rvOHfyf5hu6peIBCJ8RXaR04pMwIHgQU\/k6TxoVDgKDVw+Zm6SaWW7EgBaExovVFJeSPvnrbLFUYqEz8oJIPGzYsC2BkZWhBeeONNzJ7KpP7Z7I6uOfi7lEQ73xc2I+7r3zF1\/eipoWNL7QmSz9u3LgsXm3zpS\/MukzIKpr2x3zx4hFomuOTJ0+qhVNfvPDf3FqRJ0ag0q08LRpkmfD\/cS86c+bMzGKp88fnIQdU+kRyV1BX+Kigmj17tnJv9AeBNGjQoKwVzQaqIP89SLFMk0jjSAtUHLhIm1YzE1A4fzZQ0eqKK6ZuiW3g4O1Mcm5vb8+s0GhpdVca+5BymSwOAgkjxBgBJo+DWyqy+ui+84e7rVy5+T6MyyQIVDYvRnfB8d\/kLZAbffbsWWWdkL7uJemgIi+D033yySdhxowZanti4o90EHU6E6igPVLQfkqPskUFlW6pbFYGmQmzVMRwXEtFioAWNG1Q8dW2f\/\/+OasZH3MUUAW5pXEsFZdRUKCFrBDtQ1599dXMwsD\/G2XHI6IcVLqlyllBP\/mDq6UyBYuCQIX7Ih4Z5nJwsVQ6f1mW6tKlSypQEeVAN8qeilZXcn2i7qlotQiafN3XJ7+ZWzvac9hWQtrroaLTKsb3P7TpD9qfmELq3L3grkuSPRVXUL75R7p8T6X\/FransikXbfRp4cGgDlnbt956K2dltwV68rWn0vd+eNSBxwQ0fm6pKMiEfUiuZLV0UGF\/056L+OVyCdxTxT38xYnEDRlZNlIgvlr07NlTuRI8AhMW\/YsCqjjRPz3Ma4vA2SJpuoLSqmyLTup+PV+k+Lu5O8SVkKwALko4iaNHj1buFj44FlQE4slH9I97KHy\/tWnTJnjllVfUfGMbtMT44N5FHzPtq+NG\/0yRTduZY1D0zwQqHszB3x966CHYt29fjltN55tcTxNH\/9JKUyr0WYHN3Si1v+v7wDjnZwhifPSATanJyIWfIM\/CRpd0f8yYMTfOqVwGQX0FVGlIMRoNbvmwR9B5EKcYNaMi2ihKs5V+phUl3zUno6IU0pRKc3qFq64qgdBzqq7KmIxbJFAoCQioCiV5eW\/JSqCgoBoyZEjJClYYK34JfFz5IFy6dxp0vvhoqoPNAhUPWWNYF1MzFi1aBAsWLEgtM5uPHkF17NixVBkqJ2IiP7fZbvn5f8NTLUf9gYqn2uCB2datW2HhwoUq+fXAgQOh1wWSsCdKkURqN\/ocP34cBg8e7EakjHt7BxVPnMWTZgIVgi2NG794nqKDU0DlptECKjf5rdrTDqv2HPdnqXB4dLVi2rRpsGPHDpVAyDNzk7JAsX9+HQNpCaiSSvR6PwGVm\/y8WyoaHk8Dwb9FPVi0sYeWbsWKFTB06FA4fPhwlhspoHJTCgGVm\/zyYqnchmjujW4fPpj02tLSkgOqt99+28dry4Im5iNiMqw8ySRw\/5I9cLXfcH\/un57wqg8z6EqIjSV0+2hvhgmZJlBJ9C+ZQoj7l1xu1LP65fdh\/2\/P+wMVvgitCl5m4wmX9DfM2NVBEcaWfi0f22Oo\/qWXXlJdxf0Lk2Dw7+L+ucmvzz\/9VBHwdk5lq+tHf8crH3ifJKw+oI1N3KuJpXJTAr23gCq5PPd+0AmTNl8vIOsNVHRORcUz8CIej9pNnDgRdu7cmfi8SkCVXAFsPQVUyWR6svMKjFz+M9X5U5fOwblN30pGyNIrJ01Jj\/5RAZM5c+aknlkh7p\/bXAqo4suPAwp737x\/NZx89834hAJ6FDz3TwIVyedTQBVPdpiShGdT9Ly\/6Gvw6Ogvp54qJ6CKNy9F1VpAFT4daJnW\/LgdfvjumUzjyj7dYWPNXTD29t5egmVZoDJF63AkUW4\/hrOX20LcvyRSu9FHQGWWHwLppx90wuwdH+Q0QEAdXvS1zN996GAGVBjlo30T1qPDEDpV9KRKqW4qIKBKW34CqhsSRSDNbDmqzp1MD4Jp13fvAfx\/\/ngHFSXOYlkqOq\/y+QkdHwylrbjFTK9cQYUAevvXHfCj9\/9gBRHOGwLoR\/8wEgb3u1HrUZ9PHzqYsVS8IgwWc6RPrmBFT6pDHqU2YBwl9MFQnPd39balDioEDz4v\/uQEHDt7KRBABKLKL3aHjbV35Vgk21z70MGsPRW3Smit6OsPaX73ybfp7epAiTP+UgLVx1f\/rPZAH3VeCQUPyQgt0YNDe8Pc8YMjgyivlirOZKbV1scqkdbYugKdrgQqtDpX\/3wNXt77EbT9\/n8iA8cEILJKacyRDx3MClSYLiPKniqNqfNDo5hARa7atnfPwIHfnoeTf7wC9Lc43KP1QRfutSfuhn43fzZO10RtvYAqSoXaqJ+JjMuVD4bijqErt88XqAgcF65chc3\/cQpOdFxODBq+96keMQCeHHtbQafAhw6GWiqfHPtgyOd4i412mqA6fOoivPbO76D9nBtguGuG+526MbfB6EG9ik10mfH40EHJqCja6Q4fWBioyML82y\/Pwpv\/eU4RtJ3jhL\/tRgty0e74i5ug4aGB0P2zn1I\/6mdAcWgWqq0XUEVx\/ySjolBTnvtevk\/ZebAN3jp+VTVKuofR30DAQCszZfQtUNX3+hlPVwRMlFnzAqpC1lL3wVAUQRZjGwLLTZ\/\/NKze0w5Hz\/wpNcvCQYFBgOG33KTcsps\/\/+mSBkyUefahg9ZimjQgX0EKpO+DoSiCzFcbBAqu8P\/7SSj5J0c7UrUqOli+9P95bd8cOQDG3dU3Xyx2+ff40MGcjAr9g9P8i\/D8055pSNMHQ2mMK4gGAQXPXLbs\/x38+y\/Ppg4UHSxD+n8B\/u7+W2FAz89lWZawPZVvWZQCfR86GBr9Czqn4hcabRaNf0tJLx7jgyGXiW7vuAzbf\/F7eOc3f\/QOFHzBN+8ZAH995w2rEnffIqByme3rfX3oYJb7x793yq\/TI2D0r+\/xrHb6Zix9SY5YpXzC2tpalfGuPz4Y0t+BluXaNYAt75yCIx9dTG1DT+8hIOBeBZ\/Hv9IPqr96\/ROe3OK4T38uBQGVu1R96GBOSF2\/UxWlmKYNPBx4po9Pp8kQgufU+SvwwpvHncPGHCiVfXuo\/DL8n2+QxFURAVVcieW2T1MHibrzORW5gCb3T693wT+YnIbpDbtDo4uQg2X8V\/rB33y1P3TrwuFiAVWJgorYMn2AgLOsf2yYQJWkQm3buY+hpuW0UaIVvT4Dt\/b8DEz88s1wT0V3wH+X6iMVat1m9rHHHlME0q6TkhWoqK+vh8rKykRlyPSPCZvY1b+OHtf07v\/Neaj+3vtZpNH6jL39i\/Ds41Ule0BpUx2xVG6gSsNbMo0g9JxKd9mIiB4V1AGD7RBora2tKshBZaXxvyloEQdUvzrzJxi75ucZHmzXo93F3HUoCKjc5yqODkZ9W+ieKsgCmULq+veseEg96Z7qBwfPwD\/+868zgQKqhBOVyVJtJ6Byn1nvoOKfJ6XhJvkwQVRWozDEix+KdcqWrIAqqqbZ20XRwbhvydlTIYGmpiZIux6FaWBRGKIi8gKoXAkKqOKqe277KDoY9y2h7l9cgnHahzFEnzpBmmkXkY8zzmJtK6Byn5kwHUzyhqIFFY\/0PfeNITD7sUFJ+CvpPgIq9+ktG1DJPiqasgiooskpqFXZgIpbKawqijWv5ZE9lQ8d8AqqsI++Jf3YW9xVAq0UHvDSFQte99qHULsyTbFU7rPnBVRRrtP7uqhoYohbqa3fvhsmsIxvdxGWFgUBlft8egEVXaf3Wd\/PxrqJIR7xQysV946Ru5i7DgUBlftceQEVWiosoonf9G1sbAT8irz+5LPwC51LjR3aG3Y9dY+71EqYgoDKfXK9gKqYCr\/wqJ8EKMIVRkAVLqOwFiUPKnH9wlQg+3cBVTx5mVp7BRVlkRfS\/eMpSRL1C1cYAVW4jMJaeAWV7eWYZU5fVQwbYNzfOUPc9dtYcydMve\/WuOTKrr2Ayn3KCwIqn1FBzhAPpUvUL5qyCKiiySmoVUFAFeVGb1LWOEOSPBtfigKq+DLTe3gFVdCeKh9fUpT9VHwFEVDFl1leQeU+PDMFfvNXL3dGqwTfT62bPBzqxlT4Gk5J0RVQuU+nV0uFw2tra8t8QBvr9GGFJJePaKPr2NLSogrJYOUf+jg31QA0gUr2U9EVRUAVXVa2ll5BZSuIGVZ6LCpbCNiVK1fCunXrMreKiaGdR\/4AdVv\/S5ESUEWVKICAKrqsCgIqn1nq5ALa3D8KUmCen5xPRVcUAVV0WRUEVPhStEobNmyA5uZmQBeNghdYUkyvpR6XnaBimqM2tCtyo27rDt+fdEtc0mXbXoppuk2992KaNDx00+rq6uD06esVYKPUUo\/KmqmY5t5f\/ApGLv+ZIrF84u3w3Ye\/FJVc2bcTS+WuAl73VO7Dy6UQpZjmD358KFN19uXau6D2L8VSRZ0LAVVUSdnbeQWVr8yJsJD6iu0H4KmWo4pryUyPpyQCqnjyMrX2Cip8IQKgqqoKpkyZ4j7aCBSQoW+t3QOv7j+lWkvkL4LQWBMBVTx55R1UhcpSv3vOv6rvSUnkL76CCKjiy0zv4d1SuQ8xHgVkqNe0H6oiL3LTN57ssLWAKr7M8gKqQl6nr7r7Prjw+CrF518N7wP\/0jDCXUplREFA5T7ZJWepOKjmjh8Mc8dXuUupjCgIqNwn2zuo0s79C2OZg0oif2HSyv1dQBVfZnlx\/6jwi+\/cPxP7FV+fDVfurFY\/CajiK4iAKr7M8goqn7l\/NtZvmbwcPq58UP0s4fT4CiKgii+zvIIKX+Yz98\/E\/oC\/3wRX+w2XcHpC3RBQJRQc6+Z9T4Xv8pn7p4tAQOWmFAIqN\/lh77yAyn2Y0SlINdrosjK1FFC5ya+kQVX\/4G2w5m+HuUuozCgIqNwnvGQtlWSnJ1MOAVUyufFeAip3GZYUBQGV+3SWLKjkjCqZcgioksktr5YKLxVOnTo1Z6S+PqVDgQoBVTLlEFAlk1veQEVXP7AWBdakyMdDoOp88dF8vK7k3iGgcp9Sr+6fr5u\/QWwjqOQeVXLFEFAllx319AoqfAlmVLS3t0eunITt58+fr8ZncxH5dfqKiopMpSbsI6ByUwoBlZv8vJ9Txb35qxfHRPBgBSasRtujRw\/FrS1Jl0SBoJLLickVQ0CVXHZ5s1QuQ+QlnglUCNQ5c+bAggULVB1B\/RFQuUhcbv66Se96b+\/uH1mW3bt3w4QJE1Tt80WLFllBwZkyFY3Ro4kNDQ1ZriWCquG+3vCd+3unIZ+yoyHFNN2m3HsxTQIU7nsmTZoEW7duhYULF8KuXbvgwIEDWW6dzkqUvZipQi2CSrIpkiuGuH\/JZZcX949H\/zo6OjKgQjAsXboUlixZkvmwQJiFsrGqV6hFUH3h0GvwuZPvuEtHKIgEEkrg2LFjCXuau3Wjm7\/4MwUbpk2bBjt27IAZM2bAzJkz1bmVqZZ62PeAwyrUpsqJEBMJFIkEskCFY9L3QbZa6qbsC9qHrV69OmPZeEhd31MViQxkGCKBVCWQA6pUqQsxkUAZSkBAVYaTLiz7lUAWqHhInV6LLh0\/0E1jONx1TPNTPWmMrRhp8HmxZa7oZRDE1Y42k2EJCtGoZLfKgIqH1CkoQX\/DLmkBix8II139k6VJmCj1PvzIwvYRCdPhe6nLxZU\/nkW0bdu21BLJM6DKV4kynHxUjKamJpXONG\/ePKitrU2NIVdBF1t\/\/XzPBp4oZ4XFxlshx4NyXb9+PUyePBkaGxtVdDut2xlZ7p9pwtL+vA5\/BwoVQTVmzJi8fb6nkBOZ5N26e8IXpT59+iiSutuuJy4neW+59PFx5SnLUtXX18ORI0es8kzjsqKAKp66RgGVTtEEvHhvLZ\/WXkGVLzGK+xdP0lHdP05Vr4kf743l1do7qPIR\/ZNARXylDQtU4LytWLECnnjiCXUbANuH5WvGH0Vp9vAKKlP0D8VouiflKl4eUk8z6uI6rmLtr98eoEgsggcf\/JwsD6nLnir6THoFVb6if9HZlZYiga4pgZyEWrxL1dzcrNwIWv3wANiUUNs1WZZRiwT8SiAnTYnXncBXS8aD3wkQ6qUnAcn9i1HwJs0zO75PipJS5COdJq468zGnvdjSPttHWlxcPl3bC6gKBKq4JeGKCVS+MmBwu0E3zqnOiauCF6J\/WYHKFCHDW85UlZcsBneBKZJ26NChTDk2WqX5XTHbyq0fU2C0c9iwYUAH7bZIHaeNffDgHbNPRo4cqVK8sHIVf6dpzAMHDlR9zp8\/D\/v27QOkgw\/nl1fACuPHdBDd0tICPXv2VLSDytRVVVWpKGXQ4iCgKsQS4PBOk0K0traqAAw\/B9KzEXgJAO7+8bMgLMBSV1cHa9euzckf40cSmK2CeWYYCOrbt6+10hQfD5WCW758OeDlT3wwpM5p4cJA+ZSYukRjnjVrlgIVAhf55OFjAinR47VIbPyYZIgA5YsM0tODWlxuAioHJS62rkGTGZSMajp4ra6uzslZNO239Hfy7Ihx48YZQWUbp55ZEVT+jcZMoKLcSj23k\/69ePFiWLZsWVYOZhR+9AVIrwVJOiCgKjY0pDgenurPXRUOHFNWCbmFpBwEKjx+4I8ecDAdLBING6hsh5E62DioyM3j48GxEKhoD6RnWuigCuPH5v7RYbSA6ro2lNWeigOAr9ro+lC5a13xgixV2Ibdh6Wid3JQ4X6PpyXplor6hFmquPzolsqWyMstVVAGg+ypUrQg+SClu0u80hMHDgcVggIDClRNyranonY1NTU5V1iS7Kn4Po6SY9E927JlS+bumQ1UfMy6pYq6p7LxY9tTUaqZXoKOu38UEMG9IO7DTOlpAqp8ICHld\/CcQ+7+0d\/RZZo+fXomModt8N979uxRwQG0aPhBBlP0z3bWZIr+IUiD9kSmPhRYMFkqDHpQNJGPmfZK3AIRrxi8wD6HDx\/O3OoOq3xlAtXmzZvVLGHQh58xmawTAerixYtqccAoKC8LLqBKWeGFXP4lYLMstpGE7alcORBQuUpQ+uddAnpxmLjZC3pGxaBBgwDPqdKoXyIZFXlXB3mhSKDrSOD\/AOdZuztKGpt0AAAAAElFTkSuQmCC","height":128,"width":213}}
%---
