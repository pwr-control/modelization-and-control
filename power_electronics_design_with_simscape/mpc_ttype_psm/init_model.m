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
use_psm_encoder = 1; % 
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
%[output:8bf67124]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  15.921682195383369"}}
%---
%[output:45e82fc7]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.064017382188212"}}
%---
%[output:3873a036]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.064017382188212"],["15.921682195383369"]]}}
%---
%[output:2f8df18e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:45ef9943]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:20f05bf9]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000041666666667"],["-4.112335167120566","0.999345501530502"]]}}
%---
%[output:0445503e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.064795348480289"],["11.319202547499357"]]}}
%---
%[output:04a70309]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.012443765785704"}}
%---
%[output:69e958f5]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.517590710054527"}}
%---
%[output:8e3d2c7e]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.152987786896029"}}
%---
%[output:95bd65ef]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"  93.745795204429072"}}
%---
%[output:7d37a44a]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -1.166194503484206e+02"}}
%---
%[output:9ef89060]
%   data: {"dataType":"textualVariable","outputData":{"header":"struct with fields:","name":"mosfet","value":"    data: 'danfoss_SKM1700MB20R4S2I4'"}}
%---
%[output:339abfd4]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAYsAAADuCAYAAADIrivWAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQ1wX9V15yGhGxQ+CgJSIoQth8hAPio+AmscWJK4NOl2bDLddCy5s3GF47pTk04GO7ZlwjDewbbs2GkJH6nrCK\/TxpKTdJnYs5s4rpISwLghNlFnEwMGWQFFuBgbyhLctE28c564ytXV+\/7f+9557\/7eDIP\/eu\/de87vnHt+795zP047derUKcIFBIAAEAACQCAGgdNAFvAPIAAEgAAQSEIAZJGEEO4DASAABIAAgSzgBHTixAlatGhRgERfXx81Nzc7QWXnzp3U09ND69evp\/nz5wd18N\/4Ur9tVhym18mTJ2nt2rW0cOFCam9vt1ldZFmHDx+m7u5u+vSnP51Kzzwy7t+\/nx5++GFauXKlE50UlkNDQ0H5O3bsoFmzZqWuK8z2qV929CDjvGrVKmppaXGGmyPRSykWZFEK7KhUEYVJHjaRMcmiqakpCA4HDhygbdu2FUYWGzZsIA7maYhYBbAsMnLZCxYsoCVLljgLeqoOneiz2EoiWSgfvPfeewv1hyy4SXoWZOHIGhwgtmzZEpTOXy56cFIN5\/bbb6fBwUHirzXzGfNLTg8E6kv1mmuuobPPPjv4yuMrqSGrek2ZzKB6\/Pjx4EtYfXnzF6sqO20Z3DsxA4weMFgGJgp1zZ07l3p7e4kDOl8qaD7\/\/PMTQTYskCosxsbGgvd0nHS97rvvPtq4cSPt3r17ok7Wad68eQGBmH\/XezrKlmyjTZs2Ef+eNm3ahLymDLod1D3WT331m7ZVtm9tbY2URcedFVB4se8wUairo6MjwIsv7i2qnkASkShsFQ6qHLajWbd+T28+cT6r235kZCRoG6bPK38xdWEZFI6mT958880TejImt9xyC33qU5+a0ntVvmbWGWYfRyGh8sWCLByYUCcKvXjVdTcbX1JDV\/dVEDKDk7pvNgS97rA61fPnn3\/+pGEoRRYqAKvnDh48OCnA66RjltEoWXDZSmaFm06STCyjo6MBqSk5TeLhAKiG16LIQgUuHSsdx7BAeezYMWKijpPBtLWynRmUddtHyTh9+vRJhKD7g3mPA\/nnP\/95+uxnPztBFKb\/mC4fJVOU3cPIwiQKVYciqSifV6QXZUv1vunzLNuXvvQl+vKXvzyJ6G+88UZ65JFHQj9uwkioqCFYB2Gm8CJBFpYhV0594YUXTnwRqy8m1TB27doVBF31m0VQX7eql8Bfi2aAUV\/ZKpjze9xj0b9Iw8aSVYPgIKd6OPqXnvo64\/L4q9Qsn7\/mspaRRBb85Z40NGF+9ZnPK1IOC8SMw8yZMyeRYJphKFWm\/j5\/nZvB37Sluq9wUj2PL37xi8FXtLof1mPSXTDNMJQ57BT1O8p\/zJyU6Z+Mk8LaDPZRvVfzeTMI7927d5LP60QeNjwX9WGgfJ59UsmtyEvZl3tHeq9R752GDaexzfmdIocmLYedQooDWViGOSwAmgFCNRw9sOtOzCLpQzRmz4F\/8xe1+rrVG3cYWZgNTw31qHKjhqH08rOWYYMsdNzUV7dq+Cx7WFJex9EkwTiyML98GUfuLZk4R5GB6UYcwJTMZv4hakiJ5Ysji6ghN5Msor7io3qeOkGqpLWpp\/rAiSKLsDLCerZJBGb2UMyeR5jP6zKF2V8Nxeny6MNySbJbDhGVLQ5kYdl0YV8uUWQR5eRRZMF\/jwpi5pCNrlbWQK96Fo2ShUmcSb\/DTKHeufPOO4Nejxr7j\/pCz0oWZqDQfzdCFvowSVSyOiyvpXqJ+jtpexJJQz7Kf8xZTGG+UzRZmD6nhqXM4T5bZKHnyEAW6YIgyCIdTqmfcjEMZVYeFvzjyEL\/WlM9Dz0ALV68ODRnoTfMtGWooS59aMxMjkf9DgPZ\/JrWe06NDkOZeRZ9GCPvMFTWISV+XidRlXDXycIMZuaQT9IwVJLz2hyGModWlR4q3xXVs1C9bXXflMkkD7ZVnmGoMCxAFkkeMn4fZJEOp0xPuUpwp+mSR81\/z5Pg1slCD2o6GHEzedRzSWTBz0XNsOF7Ck\/zmahEv8LJHBfXyYDL\/eQnPxkkgcOGKaImI7AMaRLc6ivfDERRieAoHLkcvtTMurChFH0WEZdzzz330N133z1FL3PGmSorKcHN+YGk\/FLaBHcSWZgNLc7nw+ROk+A2e1jIWaQLbyCLdDhlfsr21Fk9UGbtWSjhw8bleUgiTc4iqQy+rwdvlpd7LLfddtuUmSkqYOgBJo4s4tYRpJ06q5KoemDlQHzTTTdNzDTiwHTrrbfS0qVLJ4a7TLLiqbPLly+PnTqrB+WwYcmwwBqWv+K6WUbV81NTrO+\/\/3568MEHSeVvdBI0Z7gpIozDl+uJmzpr9n6iFlBG5Rv0nFoUWZhEznjwlG2VeGYZzPwR\/02vU7enufBTzwHq98zhNjOfl7nh1\/iF0smCG\/uKFSuCOfBhK2rNuddx00OrYqekr7Sq6FF3OfUAak5bNntdUVioYMSk7Gp1dd3tEKWf+lDg+2Gz\/NLsCoB1Fum9p1SySDNNkAMrz4WvU0MDWaR30LKfjBpSTFoAqcudZQV32fpWqf6kIb0027lwW8QK7nRWL5UsuNfADYmvqJ4F329ra0u1p046lct\/CmRRvg3SShA2Lp60GtosW3298hBWlv2U0sro83Nheau0+1Zhb6hsnlMaWfBXwZo1a6irqysgjDCyUMacPXt2rcgim4nwNBAAAkCgfARKIwv+uubr6quvjsxZhHUzs3T\/y4cXEgABIAAE6oFAKWTBXcft27fTHXfcEezvE5XgNrvv6M7Xw+mgBRAAAtVDoBSy4GEnnrLI47dJs6FMSFWOIyzh\/a53vat6FoDEQAAIAIE3EeBdqGfMmCESj8LJImoGA6OTJjEVl\/BmshgeHhYJtCuhoLMrZGWVCzvLsocraSTbuXCyMEGO61mo2VL6gipeEBW1O6RkoH10LuhsDwH4tj0sJZck2c7iyIIJor+\/f+JgGXNRXlzvQzLQrhwUOrtCVla5sLMse7iSRrKdSycLm6BLBtqmnnpZR44cETvGCZ3tIQA728NSckmSYxjIQrLnpJANQSQFSDV4BHaugRFTqACySAGSjUckA21Dv7AyEERcISurXNhZlj1cSSM5hqFn4crqBZWLIFIQ0CVXAzuXbICCqgdZAGhnCCCIOINWVMGwsyhzOBMGZOEM2skFSwbaFQQIIq6QlVUu7CzLHq6kkRzDMAzlyuoFlYsgUhDQJVcDO5dsgIKqB1kAaGcIIIg4g1ZUwbCzKHM4EwZk4QxaDEMhiBTkXCVXAzuXbICCqgdZAGhnCCCIOINWVMGwsyhzOBMGZOEMWvQsEEQKcq6Sq4GdSzZAQdWDLAC0MwQQRJxBK6pg2FmUOZwJA7JwBi16FggiBTlXydXAziUboKDqQRYA2hkCCCLOoBVVMOwsyhzOhAFZOIMWPQsEkYKcq+RqYOeSDVBQ9SALAO0MAQQRZ9CKKhh2FmUOZ8KALJxBi54FgkhBzlVyNbBzyQYoqHqQBYB2hgCCiDNoRRUMO4syhzNhakMWJ06coEWLFtHQ0FBqsDo6Ouihhx5K\/XwjD0oGuhG94t5FEHGFrKxyYWdZ9nAljeQYlmkjQSaLZcuW0erVq6m9vT0Rr8OHD9O6deto27Ztic\/aeEAy0Db0CysDQcQVsrLKhZ1l2cOVNJJjWCaycAWQrXIlA21LR7McBBFXyMoqF3aWZQ9X0kiOYZnIQg1DMVB9fX3U3NzsCrNc5UoGOpdCKV5CEEkBUg0egZ1rYMQUKkiOYZnIgnU9efIkrVq1inbv3h2ovmPHDpo1a1YKGNw\/IhloV9ojiLhCVla5sLMse7iQ5tFnX6VPrLqPjn7jcy6Kb7jMzGSh17h\/\/35asGBB8Ke5c+dSb28vNTU1NSxU3gJAFnmRq9Z7CJzVsldeaX2zc\/8TR2lp\/yE68YUP54XM6XsNkYWSTO9ttLS0BAntNAlw25qBLGwjKrM834IIWwE6y\/RFm1J5QRY6YGoG1ObNmwvPaYAsbLqu3LIQOOXaxqZkvtnZC7JAz8JmE8lWlm8NCl\/Z2fyjyk\/75tu1JgvkLMpvir41KJBF+T5XlAS++faGPSO0Yc+R+uQszNlQ69evp\/nz5xflP7H1YBhKhBmcC+FbEAFBOncpERXUiizUOotjx46VlsSOsyrIQoTPOxcCZOEcYhEV+GbnWpGFCA+KEQJkId1CduTzLYigZ2HHb6SXUiuywN5Q8twNgVOeTVxIBDu7QFVWmbUjC+w6K8vBEERk2cOVNLCzK2TllFsrspADa7gkGIaSbiE78iFw2sFReim+2ZlXb\/P02Vqv4JbidCALKZZwK4dvQQQ5C7f+JKV0kIVFS2zYsCEobeXKlaGlgiwsgi24KJCFYONYFM03O4MsLDmPWgC4ZMkSkIWGqW8NCl\/ZlhpUBYrxzbc\/87Wn6Sv7xzAM1Yhv8kLAtWvX0o9\/\/ONgO3T0LH6Npm8NCmTRSEuq1ru++fa8+5+kR597tZ5ksXPnTurp6Qk8kM+14Ku\/v9\/6VuVcD18jIyMYhjLau28NCmRRrYDfiLS++XZtyYLzB2NjY3TXXXfRmjVrqKurK\/jqT8orZHUeXtvB5XM9W7duBVmALLBdd9ZGVNHnfSOLK+9+nJ4\/8a\/16lnoi\/NaW1uDk\/MUWdjeopzJ56abbkpFRJzgVtfg4GBFm0g2sUdHR4lt4NMFnf2wti92njNnDv3q7RfQa787PoGnVlNn48iCE9Ec4G2c0c3Es337drrjjjuCE\/iSei2YDeVHEPHtixNDb\/X3az5Sdd4DT9aPLFgjziPs27dv0jDUzJkziVd4d3Z2WtmJVs+J6O7CR7jec889UzwIZFH\/RoXA6YeNfbNzrcmCjamfZ6Fc2OWW5ehZTA0U+Mr2I3jCzvW2s0puv+WNl+nlv\/pDkcpaOYO7KM1AFiAL3744lcVBFkVFmeLr4aQ2J7f5Ov3lp+mlr\/xp8UKkqLFSZJGkD4ahkhCqx30EznrYMUkLX+ysVm4zHmc9upGe\/8G3kqAp5X4uslCHIA0NDcUKHbfa2oW2IAsXqMor05cgoiMPneX5oQ2J9FzFtOYz6LUH\/4iGh4dtFG29jFxkwVJw8nlgYGDSrCdFIirBnTRsZFsbkIVtRGWWh8Ap0y62paq7nXn4iWdA8f\/5+tHnrqcPfeA99SILRQq87QYvxNMvfers8ePHad26dcERrEVcIIsiUC6\/jroHkTCEoXP5fmdTApMovrTgCpr\/gYtIcgzL1bMAWdh0m8bKQhBpDL+qvA07V8VSyXKaRMHDT9yr4Kt2ZJF2GEqtxQhbE5EMafYnJAOdXZt0byCIpMOp6k\/BzlW34Lj83336BH1iy69zvTpR1JYsWLGwdRa8oSAPTfG95cuXB0NQ7e3thVgaZFEIzKVXgsBZugkKEaAudlY5CT0\/wQDe23k5\/dF175yEpeQYlmsYqhBPyVGJZKBzqJPqlbo0qFTKvvkQdM6CVnWfrbqdmST4v9sGDk0ksdka3JvY9WdXBf83L8kxDGRR3bYUSF71BpUHfuicB7XqvVNFO6texF9+96f0P\/eNTQKdyeFvut9P77\/4rEhj1JIswoagFAIdHR1WNhLM6t6Sgc6qS9rnq9ig0uoW9Rx0bhTBarxfFTsrghh86gQt+8bTU8Blklj\/8XZ6b8tZob0J\/QXJMSxXz4JPruNtyWfPnk3z5s2b2KLc9kaCWV1aMtBZdUn7fFUaVFp90jwHndOgVP1npNuZSWLjnhHa8cSLoWDHDTdFWUdyDMtFFvoW5Zy85sV3bW1twU6z3ONwcVpeGteXDHQa+fM8I71B5dEp6R3onIRQPe5LszOTwwMPv0A\/GXs9OP407GKCWDP3UrrqknMSexFh70uOYVbIgqfI8pGnvEjP9uFHWdxeMtBZ9MjyrLQGlUX2vM9C57zIVeu9su3M5LBhzxHqf+JoLHB5ehDe9CxYUX0rD703sWvXruCci97e3uDAoiIvkEWRaJdXV9lBpAzNobM71FXO4a8fGaV\/Gv1\/kb0GJQGTw+9cfj79+Uem5eo9xGkiOYbl6lmwsuYqbiaPLVu2UEtLS6FrK3TgJQPtytURRFwhK6tc2NmOPZgYfnXqFH3+OyP02HOvTprSGlUDkwOvh+DtOMKmu9qRbLwUyTEsN1nYBMhWWZKBtqWjWQ6CiCtkZZULO2ezB+\/mytfXDhylkZdPJvYW9F7Df2k\/j5bf3OacGMI0khzDcpGFmeDWlUbOIptTN\/o0gkijCFbjfdh5qp3U8NHwsTfoC3\/\/U3r+lfFFcGku1UP4yGXN9Jk500shBu\/JQt91trm5OY3drD0jmZWtKWkUhCDiCllZ5fpq57f+5vh2GEdf+wXd\/b\/Hz3mImokUZTEmhmnnnUEb\/ttMOv76vwfE4Ho4Ka\/3SI5hmXoWcQvxdHCKPvRI1S0Z6LzOk\/Ser0FkxowZSdDU6n5d7az3BPoe+xk9+fxrmXoI+vAR\/\/uDl55LXde+UzQhxDmm5BiWiSyUknHDUGW2UMlAu8KlrkEkDi\/o7Mqb3JTL+QP+kv\/HI\/9Cf7N\/fAuMrL0Dfkf1BriX8NmPttH05iaxPYS8SEqOYbnIIi8Qrt+TDLQr3RE4XSErq1zJdla9g6\/+4EV6nGcYZcgdmCgrQrjx3efRzdOJrrxsPNHMdUgdOrLpKZJjWCaySHv2NvaGsuk+8WVJDiKuUIDOrpCdWq4igr2HjtM3f\/RS7l6BOVzEvYNPXHMRfWjmeROVmmTgo51rQxbFuWi+miQDnU+j5Ld8bFDQOdkv0jyhhoceefYV2vnmKuU8w0N6XfpQ0R\/Pvpg+MP2cSaJk6R34aGfJMSxTzyKNA5b5jGSgXeHiY4OCztHepCeMf\/5vv6Rt+35GT73484aGhsJ6BTMubKLbPjSN3nb6W5wND\/loZ8kxrCGy4D2henp6Jnnu+vXrgw0Fy7gkA+0KDx8blK86q2mk7Et\/d\/Cf6XtPnwjcqpEcQRgRXNJ8Bi2+oZVe\/8Uvg9s3vPtcV+4bW66PdpYcw3KTBRPFwMDApHMrVE6js7OzFMKQDLSr1uZjg6qbzmo4iLehuP8fXqCnj\/7cGglwQfrQ0NXTzqHuD15Mp735d9UTyTI85MqXzXLrZuc0uEmOYbnIwtwXSgcBi\/LSuIS9Z3xsUFXQWR8Oeuaff07\/5\/++TM++9IaVXkBYb+Dyi86k33vfBXTphW8PbksM\/lm9vgp2zqpT0vMgiySELN2XDLQlFacU42ODKktnnQD+41en6G\/\/cYx+OPKa1V6A2RPgIaHfueJ8+uXrx+m698zwahppWXZ21VbTlCs5huXqWbDSGIZKY3r3z\/jYoGzqrBPAKfbrJ47So8++Yp0ATBK47KIzqXt2C531ttMnnCSuN2BTZ\/deaacGH3WuJVkowkCC207DyFuKjw0qSWedAP79l6eIF4v9cORfnBMA9wJ4MdnsS3+dELY1HJSkc17\/kfyejzrXjizKTmRHObhkoF01Sh8alLmT6CP\/NEw\/PPZWeu6lN5wQgNkL+O3Ws+lTN1xMo6\/8IqivjNlBPtjZbCM+6iw5huUehjI3FdyxYwfNmjXLVUxMVa5koFMpkOOhqjYonQB4FtD\/evIleu7YG\/TCiX+1mgRWkOozgvhvV15yNnXPvpje+pbTgq0kyiCALOauqp2z6Aiy8ODwI329BU7Ka6R5ZH9XUhBRU0BZi+8+dYKefOE1OvLyyUCpRlcGhyGjE0DbBU102W+dSXN\/+8JUOYDsSJf7hiQ7F4WEjzpL\/uDN3bOIchg+XpV7HX19fYTzLNw3K5cNir+41QZuwy+fpK8fOBp8+fNlYyFYEgHw8M9\/fd8F1HreGcGjihxc6uzeYvlqgM75cKvaW7UnC3X+NhsmzSaCJ0+epFWrVtHu3bsDW8adf2EOd8X1XCQD7cppswQRNfTD\/3\/bb7yFvnHgKB160e4CMFNP\/et\/+vlN9JHLm+maaeP7BeUd\/smisyvciy4XOheNeDn1SY5huXsWjQw9MbnwtXLlSkpKlnM9IyMjwbNJl2Sgk2TPe\/+xocN0ySWXEM\/75y\/\/x948e7iIL3+e\/TPzt86kj1\/5jmBFsOqF2JoBFIUJAmdeb6nWez7aWXIMy0UWcSu487ijTh7m+3yvra0t1fYhkoHOiosa\/+dx\/+\/85Hhhid8b28+j\/zzjN6nt\/KZJIrsmgCz4+BhEoHMWD6nus5JjWC6ysGmKOOJRw1WzZ8+uHVkoMnjg4RfoJ2OvW8sBmLN+rnjnmfTH119MZ77trYHZ1EEy6t82bVlUWQicRSFdbj0+2hlkEeFzKtcxd+5c6u3tpaamyV+zYYctxe1qKxVoHp7ZsOdI0DvIOytIJ4Eb2s+jzg9cFKD6wgsv0Ac72r05SYx19jGIQOdyiauo2qXGMNa\/9J4FC8GkMTY2NoUwDh8+TN3d3bRp06ZgDYf52zRg2UCrMXveOvov\/v6nqYmBiYBPDpt+AZ8RcAk1\/cZ4LyBNAhhBpKhmXG49sHO5+BdVe9kxLE5PEWTBJLBixQrauHEjtbe3x9olLr\/BQKtrcHCwKPsG9Qw++wat+Nb4sZNRV8s5p9M1F59Bf3Ld+HYQ\/LvRa3R0lFpbWxstplLvQ+dKmSu3sL7Yec6cOZMwGh4ezo2ZyxdzkQUPDy1btoxWr149Jbhz4F+3bh1t3rw59TqLLNuaxyW8i2Zlzjts3HMktAcR9Baam+i+zsudbheNL06XzUNO2bCzHFu4lKToGJZFF+tkkSbw670DlcTm9RPm9FizLP69fPly2rZtW2gPpCigmSRuGzgUDBPpFxPEV299P7235awsNmjoWQSRhuCrzMuwc2VM1ZCgRcWwPEJmIgtzgVxUhXGL7Pgdc1GenuDmOvr7+yfyF1n2oCoC6Hn3PzmpJ8EE8fvvv5DW3vLuPPg3\/A6CSMMQVqIA2LkSZmpYyCJiWF4hM5GFqiRuGCqvIDbecwk09ybmPfDkhJhMEhv+YCZ99D3n2xA9dxkIIrmhq9SLsHOlzJVbWJcxLLdQb76YiywardTV+y6A5qGm\/ieOBlNf1XXDpefSrqVXuVIjU7kIIpngquzDsHNlTZdJcBcxLJMAMQ9nIgu17mHx4sW0detWGhoaCi06zf5QthTQy3EBtD7sxL2J+zqvELWdNYKIC0+SVybsLM8mLiRyEcNsyZmJLGxV6qocm0Bzj4KHnVQSm4li159d5XRmUx5cEETyoFa9d2Dn6tksj8Q2Y1ie+uPeAVlEoGP2KCQSBYuOIGK7ScgsD3aWaRfbUtWOLMK24dBBq\/ow1Ka9I7TuW+M5Cu5R\/Ohz19v2CWvlIYhYg1J0QbCzaPNYE652ZBGFDE+JXbt2LS1cuDBxJbY1dLWCbACtz3qSOvSkY4cg4sKT5JUJO8uziQuJbMQwF3JxmdaHocx1Eq4EDyu3UaA5P3Hl3Y9P9CikJbPDdEYQKdLDyqsLdi4P+yJrbjSGuZTVOlnk2e7DloKNAM1EcVv\/oYkFdzz0JOkMhyiMEERseY\/scmBn2faxJV0jMcyWDFHlWCeLqB1kXSvC5TcCNK+lWNp\/KBBzzuXN9PU\/6ShC5IbrQBBpGMJKFAA7V8JMDQvZSAxruPKEAnKRRVyCO+6MbNfKNAJ08+3fmxh+kjrzCcNQ4wggcLpuSTLK99HOjcQw11bLRRauhcpbfl6gl3\/jGXpw38+Capkobnj3+BbiVbh8bFDQuQqe2biMPto5bwxrHO3kEnKThXnkadzuscli2HkiD9BmUlvyNFn0LNCzsNNSqlEKyEKWnXKTRdghRGUTRh6y+MzXnqav7B8LrHJ\/1xXUde34caVVuXxsUNC5Kt7ZmJw+2jlPDGsM5fRv5yIL24cfpRc3\/smsQFe9V8Fo+NigoLOtFiO7HB\/tnDWGFWnB3GSxaNGi4LAiPhtbv9IcfuRKwaxAr\/3WEdq8d6SyvQqQhStPkleuj4HTR52zxrAiPTUXWbCAO3fupIGBAerr65s4PlXNkurs7KT58+cXqUdQV1ag9RlQVctVKHB9bFDQufCmVUqFPto5awwr0jC5yYKFDDs5b\/369aUQRVay0Lf1qGKuAmQxo8h2UnpdPgZOH3WuLVmU3oIMAbIArXaVrcL+T3E4+9igoLO0ludGHh\/tnCWGuUE9utRcPQs166mrq2tKzqJoBfT60gKtJ7YlnXqXBzsfGxR0zuMp1XvHRzunjWFlWDMXWVT9DO4Ne0Ymjkmt2iI800l8bFDQuYxQUXydPtq5dmShEtwjIyPBjCgpV1qg9SGoqia2kbNAzkJKu3MlB8jCFbL5ys3ds+Cps1U9g1vNglp4fQv9xR9elg85IW\/52KCgsxDncyyGj3ZO+8HrGPrQ4nORRRmCpqkzDdD6LKiqbEOOBPdkBHwMItA5TQSo\/jNpYlhZWnpHFnUagmKnQRApq+kUWy\/sXCzeZdVWG7JQi+4WL15MW7dureQwlBqCqvosKOQskLMoK6AVVa+PBFkbsijKSfLWkwR03Yag0LPI6ynVe8\/HwOmjzkkxrEzPzT0MVcUtytUQFAN+4gsfLhN3a3X72KCgszX3EV2Qj3auJVlUbYvyOi3E01u4jw0KOouO8daE89HOtSOLKm5RXpe9oMyW6GODgs7W4rHogny0cy3JompblPc\/cZSW9h8KGkfVV22jZ3GEZsxAglt0pLcgHMjCAogWi8ids6jaFuV1mzKL2VAgC4txQGRRIAtZZslNFqxGlbYor9uUWZAFyEJWKLEvDcjCPqaNlNgQWTRSsYt3o8b76pqvYAx9bFDQ2UXrkVemj3auXc5CnluNSxQFtL7LbB22+EDOAjkLqW3QplwgC5toNl6W+J6FPtTV0tJC27Zto\/b29lDNo8iijusrMAyFYajGm7\/sEkAWsuwjmiwOHz5MK1asoI0bNwYEEZZU1+GMIosr736ceJ0Fn4pX9S3JTffxsUFBZ1lBxJU0PtqUUQ1DAAAPY0lEQVQZw1CWvMkkD7PYMKD1xXi3dLyDti18ryVpZBTjY4OCzjJ8z7UUPtq5lmTBgbu7u5vGxsam+ExHRwf19fVRc3OzVX\/insW+ffuot7eXmpqappQdBrSe3K7T+goMQ2EYymrjElgYyEKWUXINQ6l9oTiHUMRJeTox7dixI\/Lc7zCy+OoPXqRPDzwVoA6ykOV8eaXxMYhA57zeUq33atezKOsMbkUamzZtCiWMMKDruhgPPQv0LKoVBrNL6yNB1o4sVM+iq6sr8is\/u2ukeyNsA0P1JgOtrsHBweCf19w7Mv7\/i8+gv\/6Di9JVUqGnRkdHqbW1tUISNy4qdG4cwyqU4Iud58yZM8kcw8PDIs2TaxiKNUnKH7jQ1twW3awjjJXVyu2VH51BKz\/a5kKsUsv08esLOpfqcoVV7qOda9ezUCfmDQ0NhTqOrQS3OfuJ11wsX748cq2FCXTdk9sMvo8NCjoXFq9LrchHO9eOLIr0IHP\/qSwJbpBFkZYqri4fgwh0Ls6\/yqwJZFEQ+ibQdV65rSBFECnIuUquBnYu2QAFVV9bsuC8RU9PTwAjf\/Hz1d\/fH7kOwjXeUWRRx5XbIAvMhnLdnsou30eCrCVZ8KwkXpB311130Zo1a0jNjIqbreTa+XSg63qMqomhjw0KOrtuSTLK99HOtSMLfZ0FT9tctWrVBFlwUnrdunW0efNm6yu4k1w4iizquBgPPQv0LJLaQ9XvgyxkWTDX1Nk4suCENPcuXGz3kQQdyCIJoXrc9zGIQOd6+G6SFrXrWbDCap2FPgw1c+ZM4rO5Ozs7af78+Um4WL+vA13nMyx04BBErLuRyAJhZ5FmsS5ULcmCUZJ8rKoPM6HYBggi1turyAJhZ5FmsS5UbcnCOlINFqgDXfc9oZCzQM6iweYi\/nUfCRJkUZBb6kCrbT5uuPRc2rX0qoIkKL4aHxsUdC7ez8qo0Uc715Ys9HUWypniVli7djgFtD5ttq57QqFngZ6F6\/ZUdvkgi7ItMLn+XLOhVIJ7YGBg0qwntWdU2QlunSzu77qCuq6t326zIAuQhaxQYl8akIV9TBspMRdZKFLgg49mzZo1qX4JU2f1mVB1XmOBBHcjrl+td30MnD7qXLthqDiykLAoz5dpsyCLagX8RqT1MXD6qHPtyIKdPmy7cCnDUL7MhAJZNBJ+q\/Wuj4HTR51rRxZJ51nozZDPtnjooYcKaZkK6Cvvfpw4b1HnDQSRs0DOopBGVWIlIIsSwQ+pOlfOQpYKv5aGyeIffvgTYrLgq+7TZtGzkOqJ9uXyMXD6qHPtehb2m4KdEk2yqPu0WZCFHb+pQik+Bk4fda4tWYSts1i\/fn0p+0Jxg2egv\/KdgzTvgSeD9l\/3mVAgiyqEeTsy+hg4fdS5lmTBRCFxncXanftoaf8hkIWdGCWyFB+DCHQW6YrWhaodWUheZ\/E\/+h+jP9\/5FMjCuhvLKRCBU44tXErio51BFi49SiubgX7fsr+jR597NfjriS98uKCay6vGxwYFncvztyJr9tHOtSMLdhipw1CKLHyYNoucRZGhq9y6fAycPupcS7JQhNHT0zOpFZWd4D7n1q8Gayx8mDYLsig3gBdZu4+B00eda0sWRTaWNHW1ve86eu13NwSPgizSIFbNZ3wMItC5mr6aVWqQRVbEcj7PQL\/68b7gbR\/WWKBnkdNRKvgayKKCRsshMsgiB2h5Xpl23e\/R6zesCF6t+9bkCh8EkTyeUr13YOfq2SyPxCCLPKjleKf1Q\/+d3rj61uBNHxbkoWeRw0kq+grIoqKGyyg2yCIjYHkf18niR5+7PthIsO4XgkjdLTyuH+zsh51BFgXZ+aJP3E3\/Nu2DQW0+rLFAECnIsQRUA7IQYIQCRABZFAAyV\/GOT\/4V\/ccFl3mxNbmCFEGkIOcquRrYuWQDFFQ9yKIgoC\/406\/Tr95+AciiILzLqgaBsyzki63XRzuDLArysebbvxfU5MsaCwxDFeRYAqrxMXD6qDPIoqDGpsii69qLgqmzPlw+Nijo7INn+5nUB1kU5NuKLHxZkIeeRUGOJaAaEKQAIxQgAsiiAJC5CkUWvizIA1kU5FgCqgFZCDBCASKALAyQ1XkYQ0NDwZ25c+dSb28vNTU1TTHH\/v37acGCBRN\/b2lpoW3btlF7e\/uUZxVZ+LIgD2RRQOsVUgXIQoghHIsBstAAPnnyJK1atYpmz54dHL+qfjMJrFy5coopeCv0kZGR0HvmwyALx54spHgETiGGcCyGj3YGWSQ4FRPCvn37QnsXGzZsoLa2tlTneiuy8GX1NnoWjqOVoOJ9DJw+6gyyyEkWZi8kqe0yWfhy6JHCwscGBZ2TWkI97vtoZ5BFjO+q\/EVnZ+eU3oOZ2+Bi4g5XAlnUI0gkaeFjEIHOSV5Rj\/sgiwg7qp4D3w5LcB8+fJi6u7tp06ZNNGvWLDJ\/h+UsTn\/5aTrr0Y00ODhYD+9J0GJ0dJRaW1u90FUpCZ39MLcvdp4zZ84kgw4PD4s08GmnTp06VYZkSUQRJRPnMPgKS4Zzz8Kn1dvIWZThueXUiZ5FObgXXSt6FgbiSTOg4gwUl\/BmsvBpQR7jJNm5XDU06OwKWVnlws6y7FFKz4ID\/tjYWOTaCgURr7HgZ\/v6+qi5uZn49\/Lly2PXWYAsZDmYC2kQRFygKq9M2FmWTQoni7CkNUPS0dERkMIzzzxD\/f39E0RiLsrbsWNHkL8Iu7hn8faDD9J\/ev4xWShDGiAABIBASgSQs0gJFB4DAkAACAABeQgU3rOQBwEkAgJAAAgAgSQEQBZJCOE+EAACQAAIEMgCTgAEgAAQAAKJCIAsEiHCA0AACAABIACygA8AASAABIBAIgK1IAvetbanpydQNm7vqEQ0hD7Aa022bNkSSBc3dVh\/Lu7cD6FqThIrrc7qpaybTkrFIK3e5hT0OL+QqquSK63OarsfXqNVdf+Os0mWnbaLtG3lyYIdaMWKFbRx48YAN\/XvsMORigTWVl36wkReg6IvUtTrMLd5598DAwMTCxptyVNEOWl1NvXnD4Yqfyyk1dvcAUFvA1Xz+7Q6K3LkbX54nVWV\/TuJKPjDUKIfV54szCAplZXzBll9LywVJLq6uiIXJqp6qhxAsurMgWTZsmX06quvUtjuxXmxL\/q9tHqzbdetW0ebN28Odjao8pVFZ\/1DsMr+HWYvRYbnnntucPtjH\/tYqjN8irR95cnC3FgwbqPBIoG1UVfUqYLqlMG4OqramPLozDa\/9tpr6Zvf\/ObECYw28C+yjCx6xx0WVqTMjdaVReewnkXUgWmNylXG+6zfK6+8Egyv6SeJliFLVJ21IAv9JL0sx7BKMkSYLGE9ibQ9p7T7b0nDIKvOTIrbt2+n22+\/ndasWVN5stB7jVG2Vj7OtkuTy5JmYyVPVlur53fv3k1LlixJddSyVN2j5JKcewNZCPamrI1JqcLB5N57743ccFGwyhNnsqcJmozP2rVraeHChcGZHlK\/yNLgncXWakKHSmonbbCZpv4ynsmis3mWjbnJaBnyu6gTZOEC1TfLxDDUZHCrTBSsSZahCQ4YDz\/8cPCFKbmRpXH\/LHqbw1BV1d1HnZN8QbItK9+zMIed0g7TJBlNyn1dn6QEd11miKTVWZ9yqdurqkMUafVmktR3Zk7yCym+HCZHWp3rQpBJtgBZJCHUwH1MnR0Hr6pDEWGmTzudUn9XciNL695p9TaTvVUekkmrc9gwVNzZNmkxl\/acZD+ufM+Cje3rojy9VxX1lV3VxVpRC7WiJjBIbmRZAlJavfVFeVVfoJZWZ\/1sm6rrHOUTkv24FmSRpTHiWSAABIAAEMiOAMgiO2Z4AwgAASDgHQIgC+9MDoWBABAAAtkRAFlkxwxvAAEgAAS8QwBk4Z3JoTAQAAJAIDsCIIvsmOENIAAEgIB3CIAsvDM5FAYCQAAIZEcAZJEdM7wRg4C5ujgOLNcrj\/W1CHPnzqXe3l5qampKtJ+56C3xhYIfUOsSOjo6nJ5Xom\/clwW\/guFAdQUhALIoCGhfqpFEFllk0e1TBbJgeXlPrCKuumyJXgRWda4DZFFn6zrSzTzSU+3FpK+wVV+9\/CXPu8HyttLq4lPA5s2bN+nv6mQw\/WuWn0\/6oo1a1auv6udywlayR+mh\/s4nsqktwM2veL1eLl+\/z3UfO3aMBgcHaWhoKKib7+s43HnnnbRr167ghEc+3S6L3ubmmVm2LA8jwiQySLrvyM1QrDAEQBbCDCJdnKSdQvWvedaFAyRvzaC+gvVdcdW24mo78rCtDuIOszL3wwr7rW+4p2Mbp8fNN99MixYtomnTpgVDV6YeZj06ubCeYTv\/6ueLqPIOHDgQbCMftr16nN5hZKEfKWvuo5TUa0oig6T70n0W8tlBAGRhB0dvSknauyZp6Eff+NEki7B31ZGpq1evDr7A1RUlhx5I42SJ23wvz9e3Xq8ZXMN00Ann+PHjk3aRZR2j9OZ7YWRhnhoXRTZ5dANZeNO8YxUFWcAPMiOgD8GYw0RRAVrfLE5tAmeShTl0pAQL2zQuKq+gbzQYRxZxATBtQFVf8GNjY4GoajjOLDvszGydNA8ePEjcMzCvqM3yooah9BxGlH5pddNlAVlkbiK1fAFkUUuzFqOUHiz1vIU+9KNIQpHK6OgorVixIhirDyOLtOcql0kWrEN3dzcxSahcSFzPIg1ZpNU7qmcxMjIyKeENsiimDfhUC8jCJ2s70tU8k0CRBQ8VLVu2jPQhpKRhKA66fX191NzcHCttmcNQnJg2g3Ojw1Bp9cYwlCMnRrGJCIAsEiHCAzoCSUlofeiHn+VEMQ+P8Mwi1RvgmUJ6YtdMcOsJ8bjcgs0Etx6EFy9ePEluvqd\/qTNZ6D0BNXwWNQylyuaeiJ4wNxPcafWOSnCrXo5OyHqeh+VQ9lN1KZuoZH7YOhQMQyEGMAIgC\/hBZgTMsXo9b2ESAidvFyxYENTBAWrTpk1Bgrazs5Pmz58\/cXCVCrTmdNakhWdxB+IkJdvNupQeJsmZZMG\/9WmwLHtbWxsNDAwEvaK9e\/dOIhM9SKspxDx19vvf\/z5t3rw56EVl0TuMLL797W8HGPOZ5HzpU4XDcihqGI3xZXLcs2dPQGRJuqdZ1JjZofBCJRAAWVTCTBCybgiE5THS6phmNlTastI8h55FGpTq\/wzIov42hoYlIxCWjI9bR5EkLsgiCSHcd4EAyMIFqigTCBgImCu+1bBbHqDMvaHChr3ylGu+g72hbKBYnzL+P0+FFsse3+BhAAAAAElFTkSuQmCC","height":238,"width":395}}
%---
