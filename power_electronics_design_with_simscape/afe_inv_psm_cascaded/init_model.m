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
model = 'afe_abc_inv_psm_n';
load_step_time = 1.25;
%[text] #### local time allignment to master time
kp_align = 0.6;
ki_align = 0.1;
lim_up_align = 0.2;
lim_down_align = -0.2;
%[text] ### Enable one/two modules
number_of_modules = 2;
enable_two_modules = number_of_modules;
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

use_moving_average_from_ccaller_mod1 = 1;
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

dead_time_AFE = 0;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;

ts_afe = 1/fPWM_AFE/2; % Sampling time of the control AFE as well as INVERTER
tc = ts_afe/200;

s=tf('s');
z_afe=tf('z',ts_afe);

t_misura = simlength - 0.025;
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
deepPOSxi = 0.5 %[output:65aea0a9]
deepNEGxi = 0 %[output:0853a9fc]
deepNEGeta = 0.5 %[output:9fc20da5]
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
settle_time = 0.15;
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
l1 = Kd(2) %[output:54bcb499]
l2 = Kd(1) %[output:2d7b31d9]

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
kp_afe = 0.25;
ki_afe = 45;
delta = 0.05;
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:9546200a]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega_fht0 = 2*pi*f_grid;
delta_fht0 = 0.05;
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:83b297af]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:8fe1a3ae]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:6ebfb55e]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:36166f87]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = 2*pi*f_grid;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:5884f183]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:38f3794c]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:0abfa4b8]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:617e9d41]
%[text] ### IGBT and snubber data
Ron = 3.5e-3;
Irr = 750;
Csnubber = Irr^2*Lstray_dclink/Vdc_nom^2 %[output:3a686f16]
Rsnubber = 1/(Csnubber*fPWM_AFE)/100 %[output:152b21ae]
Vgamma = 1.6;
i_swlosses_base_ff650r17ie4 = 650;
u_swlosses_base_ff650r17ie4 = 900;
eon_ff650r17ie4 = 0.3;
eoff_ff650r17ie4 = 0.205;
vcesat_ff650r17ie4 = 1.75;
%%
%[text] ### Reactive current control gains
% kp_rc_grid = 0.1;
% ki_rc_grid = 2;
% 
kp_rc_grid = 0.35;
ki_rc_grid = 35;

% kp_rc_grid = 0.15;
% ki_rc_grid = 18;

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
dead_time_INV = 0;
delayINV_modA = 0;
pwm_out_lim = 1;

ts_inv = 1/fPWM_INV/2;
t_measure = simlength;
Ns_inv = floor(t_measure/ts_inv);
s=tf('s');
z=tf('z',ts_inv);

%[text] ### MOTOR Selection from Library
n_sys = 6;
run('n_sys_generic_1M5W_pmsm'); %[output:683290bb] %[output:88d5bc00]
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
LFi_0 = 230e-6;
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
kg = Kobs(1) %[output:79e2ff1c]
kw = Kobs(2) %[output:487a1b6b]

%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:95c36cd7]
luenberger_l2 = Klo(2) %[output:849978af]
luenberger_l3 = Klo(3) %[output:6d657eda]
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
infineon_FF650R17IE4D_B2;
% infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;

igbt.inv.data = 'infineon_FF650R17IE4D_B2';
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
% igbt.inv.Csnubber = (inv.Irr)^2*Lstray_module/Vdc_bez^2
% igbt.inv.Rsnubber = 1/(inv.Csnubber*fPWM_INV)/5

infineon_FF650R17IE4;
% infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;

igbt.afe.data = 'infineon_FF650R17IE4';
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
% igbt.afe.Csnubber = (afe.Irr)^2*Lstray_module/Vdc_bez^2
% igbt.afe.Rsnubber = 1/(afe.Csnubber*fPWM_AFE)/5

%[text] ### DEVICES settings (MOSFET)
infineon_FF1000UXTR23T2M1;

mosfet.inv.data = 'infineon_FF1000UXTR23T2M1';
mosfet.inv.Vth = Vth;                                  % [V]
mosfet.inv.Rds_on = Rds_on;                            % [V]
mosfet.inv.Vdon_diode = Vdon_diode;                    % [V]
mosfet.inv.Rdon_diode = Rdon_diode;                    % [Ohm]
mosfet.inv.Eon = Eon;                                  % [J] @ Tj = 125°C
mosfet.inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
mosfet.inv.Erec = Erec;                                % [J] @ Tj = 125°C
mosfet.inv.Voff_sw_losses = Voff_sw_losses;            % [V]
mosfet.inv.Ion_sw_losses = Ion_sw_losses;              % [A]
mosfet.inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
mosfet.inv.Rtim = Rtim;                                % [K/W]
mosfet.inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
mosfet.inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
mosfet.inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
mosfet.inv.Lstray_module = Lstray_module;              % [H]
mosfet.inv.Irr = Irr;                                  % [A]
mosfet.inv.Csnubber = Csnubber;                        % [F]
mosfet.inv.Rsnubber = Rsnubber;                        % [Ohm]
% inv.Csnubber = (mosfet.inv.Irr)^2*Lstray_module/Vdc_bez^2
% inv.Rsnubber = 1/(mosfet.inv.Csnubber*fPWM_INV)/5

mosfet.afe.data = 'infineon_FF1000UXTR23T2M1';
mosfet.afe.Vth = Vth;                                  % [V]
mosfet.afe.Rds_on = Rds_on;                            % [V]
mosfet.afe.Vdon_diode = Vdon_diode;                    % [V]
mosfet.afe.Rdon_diode = Rdon_diode;                    % [Ohm]
mosfet.afe.Eon = Eon;                                  % [J] @ Tj = 125°C
mosfet.afe.Eoff = Eoff;                                % [J] @ Tj = 125°C
mosfet.afe.Erec = Erec;                                % [J] @ Tj = 125°C
mosfet.afe.Voff_sw_losses = Voff_sw_losses;            % [V]
mosfet.afe.Ion_sw_losses = Ion_sw_losses;              % [A]
mosfet.afe.JunctionTermalMass = JunctionTermalMass;    % [J/K]
mosfet.afe.Rtim = Rtim;                                % [K/W]
mosfet.afe.Rth_switch_JC = Rth_switch_JC;              % [K/W]
mosfet.afe.Rth_switch_CH = Rth_switch_CH;              % [K/W]
mosfet.afe.Rth_switch_JH = Rth_switch_JH;              % [K/W]
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
%[text] ## C-Caller Settings
open_system(model); %[output:43316579]
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
Simulink.importExternalCTypes(model,'Names',{'rpi_output_t'}); %[output:8992a639] %[output:8bbe89a1] %[output:5051c7e1] %[output:3bc79f61] %[output:49b0d812] %[output:373387ed] %[output:55adee64] %[output:80b19e09] %[output:7c486149] %[output:45f9ec14] %[output:2d73f6a9] %[output:1a96255a] %[output:5b2598eb] %[output:61de6204] %[output:78e7cd14] %[output:9b0cb18b] %[output:7cbaace5] %[output:11a62f63] %[output:67e2c55d] %[output:7a6c6b51] %[output:1dc0695b] %[output:01376725] %[output:05ac5a66] %[output:481acfac] %[output:23911edf] %[output:2e4616bc] %[output:917a02f1] %[output:6aca6314] %[output:3be5b0ef] %[output:8671ea59] %[output:523dbcfc] %[output:6193539e] %[output:64f90118] %[output:0baf945e] %[output:54951eb3] %[output:6d6a8838] %[output:09b8e745] %[output:03e637ee] %[output:7581076e] %[output:3ceed573] %[output:49e943e1] %[output:3f89f79b] %[output:1e57ebff] %[output:8c4574fb] %[output:28d0e30d] %[output:8b591dc3] %[output:37dd8cdf] %[output:1ae9a236] %[output:63ecff25] %[output:5345fe48] %[output:0c7eb187] %[output:03945430] %[output:9b36d06b] %[output:5c929ed4] %[output:49d57574] %[output:2ef025f6] %[output:0fff81f6] %[output:7597b93f] %[output:2fa316cb] %[output:623d7997] %[output:95252d2d] %[output:63819df4] %[output:7e0f2567] %[output:5cb15b07] %[output:309885ca] %[output:28668d21] %[output:62c4052b] %[output:56fe9b91] %[output:15b9d690] %[output:43ca510d] %[output:49d4fcef] %[output:188a0e6b] %[output:7d359ec5] %[output:7028027b] %[output:85cb729a] %[output:138e1198] %[output:9ad0e33a] %[output:7ead8f06] %[output:143f1907] %[output:4f88f548] %[output:75e8107b] %[output:2a8360d2] %[output:3ab592dc] %[output:38abe0b3]

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


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":19}
%---
%[output:65aea0a9]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepPOSxi","value":"   0.500000000000000"}}
%---
%[output:0853a9fc]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGxi","value":"     0"}}
%---
%[output:9fc20da5]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGeta","value":"   0.500000000000000"}}
%---
%[output:54bcb499]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:2d7b31d9]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:9546200a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:83b297af]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:8fe1a3ae]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:6ebfb55e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:36166f87]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:5884f183]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:38f3794c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:0abfa4b8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:617e9d41]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:3a686f16]
%   data: {"dataType":"textualVariable","outputData":{"name":"Csnubber","value":"     4.913092846536815e-08"}}
%---
%[output:152b21ae]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rsnubber","value":"  50.884444444444455"}}
%---
%[output:683290bb]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:88d5bc00]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:79e2ff1c]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:487a1b6b]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:95c36cd7]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:849978af]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714302e+02"}}
%---
%[output:6d657eda]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.994503273143434e+02"}}
%---
%[output:43316579]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAABRAAAAMnCAIAAADJUen8AAAAB3RJTUUH6gEfCjUdgrHXhAAAIABJREFUeJzs3W10nOVhJ\/zLjt+C8JjA2CALM0iQlxKMQNMmFhu2CWGdWXudeHGac5oU7VEUPYZdP6XdJu3zQN2jUxt3N2m7SVf7EFZRdNa05SSssgbWZsLSJF2dzXjbjEA4r40QDEYW2AKCjLJgg3k+3K2imhuDpdHLZX6\/kw8zl+a+rr\/kL\/lzXfc9C8IHfycAAAAA\/9jCuQ4AAAAA85HCDAAAACkWzXUAAACA2XPJ6vM+3PTOuU7BfPGf\/\/v+U\/zUDjMAAPBWsehtC9\/\/SxfNdQqioTADAABvFe\/\/pYvOfvvSuU5xBvrg6uOne8mrYcFMJKkuR7IBAIC3hAtXrnjvxRdM4cLx5468+PyR8y6+7JnHf\/jCz0ZDCGefkz3v4stOd56G1ec1X5Z790UrsyvODiGMPj\/+kycOl35YGTr0zJuc4cm39T\/74jMhhHOXnXfhK03JyPLjdSsWnn+6Yaqe7U06FLLPLDz3hXDWSycWLl144uzw8\/NOPLs6jE5hql\/\/4HufH3n8Y\/\/in5\/iM98p\/+iu7\/xgqmEVZgAA4K3hfVM6jD12+OALzzx1\/KX\/c14IL\/xs9Npf+0wI4Vt3f+W805znU9c1XV5\/wXcefvTb9w4+\/ewLIYTzzz17bX1t+8b3f\/+xp\/7iwf43M8mzLz7zG80fDSH8eeneCxf\/\/cgLb3vh+Ilj2YVrTjNRlbO9oaPhrMcW1V987rLfvvr8fN2yFcsWPv\/iifLwi13fffqRZ8+vf\/mx5eHnpzXhB\/O\/9OlPf+Geb9x9is987Ppfm05mR7IBAIAzX\/5dF2YzNad71XOHho7\/\/OivfPij01z9N7dcs3Dhgt\/vLn7zb39yaHTslRMnXjlx4tDo2Df\/9ie\/311cuHDBb265ZsqTf2Tt1S8sOvTUiUfnYbYJR8NZP1z07t\/+0IV33VB\/7SVnrVi2MISwYtnCay85664b6n\/7Qxf+cNG7j4azpr\/QSU69\/\/yGFGYAAOAMl11Rk3\/Xhad1yauvvvrMEz9esmjhL3\/4o8tqlk9n9U9d1\/Tc0Z\/f+UD5lRMnXvvTV06cuPOB8nNHf\/6p65qmNv+Kty\/\/yBVXv\/r2sUOv\/mi+ZZvw2KL6Wz608hNXLA8hPPnkk7\/7u7\/b0NDwyU9+8sknnwwhfOKK5bd8aOVji+qnuUrVKcwAAMAZ7nQPY7\/80otHhh4559zsFf9k\/cKF0ypNDavPu7z+gr988KFTf+wvH3zo8voLGlaf7kHvv7d00ZKPrL36rOWvHgwDr4aU6juH2UIIh0L24nOXfeKKTPL2z\/7sz\/bv3\/\/5z38+eZ0MfuKKzMXnLjsUslNeZSbMwT3MP+j53GUXnx9CGB59vmXXXd96aHDy4LHjr\/z7u771Bz3ffPBPtk58Pdpf9f\/0ut+5I4QwebB739985gtfT\/3YvPLgn2x9z0WrJn7TEMIN6\/P\/6ebrv\/6dgc984esTv3j4h98ohPCVz33iEx9s\/Ddf+sadD5QnJkl+zaM\/f+nffOkb76zLbmy+LP9\/\/YeJn4YQrvudO177Z0zWWn7W0olr\/9VHfvkUeU76e\/6Xb35v4kfJYPk\/\/\/be0g+vvvziyV9el\/xT\/sff\/Jev\/ZcFAIC59d6LL7gwu+LNf\/6l8eefeeInuXdfcdG7rzjpR0uXLv3W3V9ZdeHFb3625sty33n40Yn92\/\/vt68PIfzr\/\/CNk16\/cuLEdx5+tPmy3HQesvVP35P\/26EfPP5M+cKwdvGCZaebbXvLdReuPGfip48eeubzd327KtmeWXjub1\/9i8eSve9977v++uvXrVs3PDzc29s7Md5+9fm\/f+\/R1Sem8gCwGTLbhfkrn\/tECGHBhz47MXLtVZfuvuXXf\/zE4fe2fmFiMCmZH\/63X\/7WQ4PJBx78k61JGU5a5Q3r83\/6rz96aPQjYVLPnJ9+\/uLxZUsWf\/DKSyYK5K82XpI02BDCi8df3rH7f\/xBzzcnX9J8We7Yy6\/8q4\/8clKY\/7D1I\/UXnJv8NZIP\/GHrR1LXmpjtK5\/7xKc3vO87Dz8aQjh4+Gf\/95\/9t4lrf+1XG0+RJ\/zjv+cN6\/MHD\/+s8L5337A+P9HeQwjJv8Uftn5kore\/9l8WAADm3NlvX3q6X7z8wpHh89c0TG7Lhw8OLVmy5NknB48dO7Zhw4ZisRhCODL48Fnn1dW8Y+WpZ3v3Rau+fe93J96+bdJ+9dv+8d71gcdGbvxo82lFfa1faXjv95f+9MfD5TVh7dsXZN58tsWL3ja5LYcQLlr1i7fTzPZCOCtf94sC\/\/GPfzyE8OSTT\/b29m7ZsmViPF+37IUp3cb8sY997GMf+1gIYXR09Hd\/93ennPO1ZvtI9sUXvOOkkU9e1\/T8Cy9O3hm+YX2+8ZLVX933N0mj+9ZDg\/9v1773XLTqhvX5ic\/c+UD5icM\/m53M0\/fd7z++sfnvHzp\/7VWXNl+W6\/\/p8Ot9OPk1\/\/x\/lOsvOPfaqy4NIazOrli65PT+08ZfDzwaQqhbmf4f0k4rTwih8vRz\/8+vX3vqFV\/7LwsAAHPu\/b900aK3nV7rOXtl3dMHh574ySMTI0\/85JHLL7+8ZvGrv\/Irv\/L2t789\/8u\/cv7557\/nXZf+\/JlT\/b\/oRHZFTfLc6Tf09LMvJF\/pNE2X173zyovf9ejL3xt79Q22amcn26thwUsnFiZP+ZosabY333zzxMiKZQtfOjGVinrPPfd88YtfDCFks1U+0T3bhXnXn\/\/VirOXje29baL9Nl+WK\/2wMvkz76zLjo2\/mOyOJoaPPP\/SsZffWfeLX\/4PWz9y0apzfjo8GkJo2\/C+V7\/9x69++4+TTc755qxli8dfPHZOzbJkW\/iT1zWNPDv23NG\/f2D6ssWLtrf8s1e\/\/ccvPfDvkw\/8auMlI8+O3fu\/fhBC+OCVl4QQPvOFrz\/\/wov3\/\/v219tYfq1fbbzk+RdeTPaE16w656\/+9MZXv\/3HT969\/dqrLj11npD29\/zyvaUVZy879Z\/3tf+yAAAQo6U1K1Y1XDH82E\/+rv\/vd19XnLdyeHj4n1x99Zo1a0IIF6258AMf+MBTTz31tmVV6LczZcFcB\/gHC8KrSxeeeP7Ff3Rn9ZNPPjlxG\/OE5188sXThm70BO4Rwzz33JC+uuOKK3\/qt30o6c3XNdmH+1kODF\/7aji\/+1\/\/5lc9+Irnz9nQlde73fv3a2+\/5blIIu\/f9zYIPfXbBhz47bw9mj7947NsPP3r15ReHEK56Z913v\/\/4xI+SQ9QLPvTZpet\/LzmYnXzgWw8NPvbUs8klIYT3tn7hM3\/89d\/6+D\/9Qc\/nTrHQRP1uviw3ccT94OGfffjffnnBhz574a\/tSDbtT5EnpP09h488\/9V9f\/OhKy9JdrxTTf9fFgAAqu5\/\/+iJl185jQ6WWLR02cqGK3723Ogj\/+uB75ceHB76SS6XCyH87LmfDQ4O\/uy5n4UQLqhdPf7sU89WfnzqqUaff+H8c99Urz7\/3LNHnx8\/3aiv9YPhnz78+N9d8rZfzix4g+3WWct2dvh5efjFkwY\/\/\/nPr1u3bvJIefjFs0\/nq5gnCnN9ff0Xv\/jFRx55ZGKkWubmKdl\/0PPNf\/57XfUXnPuHrR8ZeXbspNO8Px0eXbpk0eTjxHUrVyxdsijZT07q3ES9jMVfDzz6notWJZu0p0j+h60fufziC5LS++Gmd77nolUTHfXOB8qZjbeGf7hb+JyaZa+tr0n9bvmju1a94+xTb0e\/yTwT\/qDnm4899ewtv\/HhN\/zYxL\/sG84JAAAz7YX\/89L\/\/tETU7hwwYIF5130nmMvnzj85OOFQiHZWx59ZvTVV18dfWY0hFB\/ca5QKBx97vCp5\/nJE0fW1tdOvE2+5fi1r0MIa+trf\/LEG8z2hv72sR8MHhqpX5B\/wxuYT8p2\/OVXnjzyj256nXwP7DSznXfi2a7vPn3S4PDwcPKdUhO6vvv0eSeefcPZPvnhq\/5F4\/l3\/M7Hv\/rVr371q1\/9jd\/4jXvuueeRRx4Jkyp0tcz910r9l29+733vuWjiuO+9t336nXXZHz9x+I\/aNySF8NqrLv2j9g0\/fuLw5IdORefOB8o\/fuLw5g9c\/tAp7xa++vKL+w4MJRu8Cz702edfePGTad949tPh0ZXnnJ38KLnle\/Iu8Z0PlG+\/57ufuq7pFBvCbzLPZLv+\/K\/qLzg3d757lQEAiMkPHn\/qydHnp3btO1Y3LKtZfv\/99x88eDCEkD0vu2DBgux52RDCwYMHi8Xi2W\/00K\/SDysfvPKSied7\/ev\/8I3ksdgnvX7bwoUfvPKSk25WPV3\/88flw88cvXhB\/s08Ivu12XbsfnDrn\/zXif8lj8iuSrbVYfTxZ1\/8+iNjEyP79+\/v7e298MJffDn21x8Ze\/zZF1eHN35E9q9eecl\/+8ueT\/+Do0ePnuLD9\/z3+6ccO8z+U7JP+taiib3N\/3Tz9W0b3jd58ME\/2fpXf3pj8tNTPwe7bcP7kmt\/+PjTkx+1Pd\/8l29+r\/bczF8+2D95MDlEvb3ln4UQ+n86fE7Nsr+Y9IHSDyvNl+W+8rlPJL9gCOGHjz+d\/CneWZf9vV+\/Nhnv3vc3J+0S\/0HPN6++\/OL\/+Jv\/8t\/d9a3kHubwD981deo84R\/\/Pf\/dpM9\/66HBv3iw\/7c+\/k9Tf7vX+5cFAIA59zc\/euLCa9ZO7drz33nV4UcPPFF5Ys2aNee845xz3vH3z45+4uDBmnMvOPfC192jSgwdeub7jz31yeuuOvX+3yevu+r7jz015e9teunlY9\/50fdefXHZmgWNb\/6q2cmWqH\/5sV3fWhLCgk9csTyE8PGPfzx5Vnbi648c3fWtI5e98tgUZv7Yxz52zz33TDwo+yTfKf8o\/OQHU469IHzwd6Z8MQAAQBTy77ow\/64L3\/hzaY4MPvyed116aGTkojVr1qxZ89jjlUOHhlfX1v747wZXXnrlm5nhN7dc89zRn\/\/lgw9NPoOdeNvChZ+87qp3LD\/rz3r73nCeR47\/j99o\/mgI4c9L916x+J8lI5uu\/NBf\/6i87OXsBQsvOf1frjrZPrj6+HcOLT71Z46Gsx5bVH\/xucvarz4\/X7dsxbKFz794ojz8Ytd3n3782RfrX35s+Zu7gfmO3\/n4pz\/96Ym3X\/3qV7f+yX99Mxem+s\/\/ff8pfjrbO8wAAACzr\/x3T+YueEc2UzOFa886r+7Hfzf4tmVnf+9738tmsw8\/1J\/J1v747wbPOq\/uTc7wZ719n7quaWdb4TsPP3rgsZHky5zOP\/fstfW1H7zyku8\/9tSbacuv55sHvpsN9dmFa6Z2+Yxmm2x5+PkVL\/\/g0OHs79979IVw1ksnFi5deOLs8PPzTjx7xZs4if167nuwOvFS2WEGAADeEi5cuWLD+39pOjM8U\/nRC88dWXHeqnPWvGcKlzesPq\/5sty7L1qVXVETQhh9\/oWfPHGk9MPKmz\/t\/OTb+p998ZkQwrnLzrvwlaZkZPnxuhULz59CnipmezM7zNXyyQ9fNTZ6aNN11yRv\/\/rhR\/\/yrx6a8myn3mFWmAEAgLeKf3L5xe+9+IK5TnEGms3CXF2nLsxz\/5RsAACA2fG\/f\/TEC\/\/npblOcQaKtC2\/IYUZAAB4q3j5lRNT+1pm3po89AsAAHgLefTQM49O7xuSeOuwwwwAAAApFGYAAABIsaC+vn6uMwAAAMC8Y4cZAAAAUijMAAAAkEJhBgAAgBQKMwAAAKRQmAEAACCFwgwAAAApFGYAAABIoTADwLS0t7f39vbm8\/l5Ms+U5fP53t7e9vb2iZHNmzffddddcxgJAOaWwgwApNi8eXNbW9uSJUvmOggAzJlFcx0AAOLW1dXV1dU11ymqrKOjY926dYODg7W1tXOdBQDmjMIMANPS3t5eKBR27do1Ojq6Y8eOVatWhRAGBwe3bdt2iqs6OzsvvfTSEMLx48e7u7v37NmTjH\/qU5+67bbbQgi9vb1JD8\/n87fccktNTc3kwdeLUalULrvssuSTjY2NyRITVyU1OIRw+PDh7du3VyqV5MItW7YkgxOz\/fSnP+3p6Vm\/fr3CDMBbmSPZAFAdra2tY2NjhUJh69atmUxm8s3AJ2lvb89kMlu3bk0q7nXXXZeM19TULFmypFAo9Pb2fvSjH928eXMul7v55psPHDhQKBS+\/OUvFwqFzZs3v960NTU1SYD9+\/dv2bJlYGCgUCgMDg5ec801uVyuvb197dq1t95669atW0MIn\/vc50II+Xw+Wa5QKAwNDSW1PITwF3\/xF0mdBoC3MjvMAFA1uVxu8+bNe\/bsaWlpOcXHJk5x53K5TCYzNjaWjB8\/fvzBBx8MITzwwAPXXHPNlVdeGUKoqal5+OGHQwh79uyZ2IhONT4+ft9994UQhoeHx8fH+\/v7Qwijo6OZTCaE0NjYODIyUi6XQwh9fX2FQiGfzzc1NYUQkk\/ed999a9eunf4fAQDOGHaYAaA6Ojo6yuXyjTfeWCwW77vvvlNsBW\/evPm+++4rFoudnZ0Tm7ohhGPHjh08eDCEUKlUkha9cuXKEEIyOB2rV6\/OZDKjo6MnjdfV1Y2Pjyfjo6Oj4+Pj01wIAM4kCjMAVE1HR0ehUEgOWl9\/\/fW5XC71Y9ddd12lUikUCps2bRoZGXntB5Kd5xDCkSNHQghr1qyZZrBDhw6NjY1ls9mTxoeHh2tqapLxbDY7ub0DAAozAFRHZ2dnZ2dn8np0dHRsbOwUtwFnMpnk\/PbkUl1TU7Np06YQwvr169\/xjnc8\/PDDDz300Pj4eHI2+7Xfk3xaBgYGamtr8\/l8Lpe75pprkuPZyWHsZNGmpiaFGQAmcw8zAFTHF77whR07dhSLxRDC+Pj4rl27Xu+TDz74YFtb2x133HH48OHvf\/\/7dXV1SW0eHx\/PZDLJDL29vckdy1\/60pduueWWZHD\/\/v1T\/gqrrq6uurq65BHchw8f\/sIXvhBCKJfLd955Z1tbW7FYfO6555577rmpTQ4AZ6QF9fX1c50BAAAA5h1HsgEAACCFI9kAMCNyudyOHTtWrVp10vjg4OC2bdumPG1nZ+ell1560uDhw4e3b9\/um5MBoLocyQYAAIAUjmQDAABAivlyJPvuu+9evnz5XKcAAADgjHL06NH169evWLFiCtfOl8K8fPny5ubmGZq8VCrFOHmkseOdPNLYJp\/lmU0+yzObfPYnjzR2vJNHGjveySONbfJZntnkszzzLEw+5WsdyQYAAIAU1dxh7ujoyGazyZM\/29vbt2zZkozv37+\/o6Nj4mGh4+Pju3btKpfLVVz61L7yla\/M2lpVFGnsEG3ySGMHyWddpLFDtMkjjR2iTR5p7BBt8khjh2iTRxo7SD7rIo0dok0+b2NXbYe5s7Nz3bp1E2\/r6uoGBwcLhUKhUOjo6AghtLa2jo2NFQqFkZGRTZs2VWvdN6O7u3s2l6uWSGOHaJNHGjtIPusijR2iTR5p7BBt8khjh2iTRxo7RJs80thB8lkXaewQbfJ5G7s6hbmjo2NgYGBwcHBiJJvNjo6OTrzN5XINDQ3JyMDAQENDQy6Xq8rSAAAAMBOqVpi7urom3ubz+dra2nXr1hWLxd7e3nw+n4wPDw8nL2pqarLZbFWWnnN79+6d6whTEWnsEG3ySGOHaJNHGjtEmzzS2EHyWRdp7BBt8khjh2iTRxo7RJs80tgh2uSRxp6mBfX19dWaq7OzM4SQ3MM8oaOjo6Gh4fbbb7\/pppv6+vq6urra29sLhcJJtzEXi8WTZtu7d+++ffuqlQ0AAIAz3oYNGzZu3HjSYHNz8zz9Wqnh4eG1a9eec845IYS6urpkcHx8fPKB7cTMPUZ8RjU1NfX39891itMWaewQbfJIY4dok0caO0SbPNLYQfJZF2nsEG3ySGOHaJNHGjtEmzzS2CHa5LHE7u\/v37lz5+SRefe1Uvl8fvfu3clJ7MbGxpGRkfvvv39oaCg5ht3Y2Dg0NFSpVGZiaQAAAKiKGSnM5XK5r6\/vtttuKxaLtbW1PT09IYSenp5MJlMsFjOZTDICAAAA81Y1j2RPvnu5q6tr8mPAQgiVSqWlpaWKywEAAMDMmZEdZgAAAIidwgwAAAApFGYAAABIoTADAABACoUZAAAAUijMAAAAkEJhBgAAgBQKMwAAAKRQmAEAACCFwgwAAAApFGYAAABIMY8Kc6lUKpVKbW1tcx0EAACAuLW1tSUdczqTLKpWmulrbm6e6wgAAACcCbq7u7u7u0MI0+nM82iHGQAAAOYPhRkAAABSKMwAAACQQmEGAACAFAozAAAApFCYAQAAIIXCDAAAACkUZgAAAEihMAMAAECKahbmjo6Ozs7O5HU+n+\/t7S0Wi7t3787lciGEXC63e\/fuYrHY29ubz+eruC4AAABUXdUKc2dn57p16ybetra2HjhwYOvWrSGE9evXJyNjY2OFQmFkZGTTpk3VWhcAAABmQnUKc0dHx8DAwODgYPI2n8\/X1tYODw9XKpWhoaHGxsZcLtfQ0DA6OhpCGBgYaGhoSLadAQAAYH6qWmHu6uqaPHLs2LEjR44krzOZzOrVq0MIw8PDyUhNTU02m63K0gAAADATFs11gF8olUqT3+7du3ffvn1zFea0NDU1zXWEqYg0dog2eaSxQ7TJI40dok0eaewg+ayLNHaINnmksUO0ySONHaJNHmnsEG3yKGJv2LBh48aN1ZptpgrzkiVLVq5cmbweGxs7dOhQCKGuri4ZGR8fT45nT9bc3DxDYWZUU1NTf3\/\/XKc4bZHGDtEmjzR2iDZ5pLFDtMkjjR0kn3WRxg7RJo80dog2eaSxQ7TJI40dok0eS+z+\/v6dO3dOHjlpa\/a0zMjXSpXL5ZGRkbq6uuTW5YGBgeRm5uQYdmNj49DQUKVSmYmlAQAAoCpm6nuYe3p61q5de8cdd4yNjSW3N\/f09GQymWKxmMlkenp6ZmhdAAAAqIpqHsnetm3bxOtyubxly5bJP61UKi0tLVVcDgAAAGbOTO0wAwAAQNQUZgAAAEihMAMAAEAKhRkAAABSKMwAAACQQmEGAACAFAozAAAApFCYAQAAIIXCDAAAACkUZgAAAEihMAMAAECKeVSYS6VSqVRqa2ub6yAAAADEra2tLemY05lkUbXSTF9zc\/NcRwAAAOBM0N3d3d3dHUKYTmeeRzvMAAAAMH8ozAAAAJBCYQYAAIAUCjMAAACkUJgBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFIsmqF529vbt2zZkrzev39\/R0dHLpfbsWPHqlWrxsfHd+3aVS6XZ2hpAAAAmL6ZKsx1dXWDg4Pbtm2bGGltbR0bG2tpaens7Ny0aZPCDAAAwHw2U0eys9ns6OjoxNtcLtfQ0JCMDAwMNDQ05HK5GVoaAAAApm9GCnM+n6+trV23bl2xWOzt7c3n88n48PBw8qKmpiabzc7E0gAAAFAVC+rr62d0gY6OjoaGhttvv\/2mm27q6+vr6upqb28vFAon3cZcLBZPunDv3r379u2b0WwAAACcSTZs2LBx48aTBpubm1esWDGF2WbqHuYJw8PDa9euPeecc0IIdXV1yeD4+PjkA9uJ5ubmmQ4zE5qamvr7++c6xWmLNHaINnmksUO0ySONHaJNHmnsIPmsizR2iDZ5pLFDtMkjjR2iTR5p7BBt8lhi9\/f379y5c\/JIqVSa8mwzdSR79+7dyUnsxsbGkZGR+++\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\/f+fOnZNHTtqaPS1TLMx79uzZs2fP6\/20XC6PjIzU1dUlty739fUlNzMnx7AbGxuHhoZe27EBAABg\/pip72Hu6elZu3btHXfcMTY2ltze3NPTk8lkisViJpPp6emZoXUBAACgKqp5D\/O2bdsmXpfL5S1btkz+aaVSaWlpqeJyAAAAMHNmaocZAAAAoqYwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkUJgBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFLMo8JcKpVKpVJbW9tcBwEAACBubW1tScecziSLqpVm+pqbm+c6AgAAAGeC7u7u7u7uEMJ0OvM82mEGAACA+UNhBgAAgBQKMwAAAKRQmAEAACCFwgwAAAApFGYAAABIoTADAABACoUZAAAAUijMAAAAkGJRFefq6OjIZrPbtm0LIbS3t2\/ZsiUZ379\/f0dHRy6X27Fjx6pVq8bHx3ft2lUul6u4NAAAAFRX1XaYOzs7161bN\/G2rq5ucHCwUCgUCoWOjo4QQmtr69jYWKFQGBkZ2bRpU7XWBQAAgJlQncLc0dExMDAwODg4MZLNZkdHRyfe5nK5hoaGZGRgYKChoSGXy1VlaQAAAJgJVSvMXV1dE2\/z+Xxtbe26deuKxWJvb28+n0\/Gh4eHkxc1NTXZbLYqSwMAAMBMWFBfX1+tuTo7O0MIyT3MEzo6OhoaGm6\/\/fabbrqpr6+vq6urvb29UCicdBtzsVg8aba9e\/fu27evWtkAAAA4423YsGHjxo0nDTY3N69YsWIKs03xoV+bN29ua2tbvHjx4cOHt2\/fXqlUXu+Tw8PDa9euPeecc0IIdXV1yeD4+PjkA9uJ5ubmqYWZW01NTf39\/XOd4rRFGjtEmzzS2CHa5JHGDtEmjzR2kHzWRRo7RJs80tgh2uSRxg7RJo80dog2eSyx+\/v7d+7cOXmkVCpNebYpFuY9e\/bs2bPn9X6az+dvvvnmL33pS+VyubGxcWRk5P7773\/\/+9+fHMNubGwcGho6RccGAACAOTcj38NcLpf7+vpuu+22YrFYW1vb09MTQujp6clkMsViMZPJJCMAAAAwb1Xze5gn373c1dU1+TFgIYRKpdLS0lLF5QAAAGDmzMgOMwAAAMROYQYAAIAUCjMAAACkUJgBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkmEeFuVQqlUqltra2uQ4CAABA3Nra2pKOOZ1JFlUrzfQ1NzfPdQQAAADOBN3d3d3d3SGE6XTmebTDHKkNGzbMdYSpiDR2iDZ5pLFDtMkjjR2iTR5p7CD5rIs0dog2eaSxQ7TJI40dok0eaewQbfJIY0+TwjxdGzdunOsIUxFp7BBt8khjh2iTRxo7RJs80thB8lkXaewQbfJIY4dok0caO0SbPNLYIdrkkcaeJoUZAAAAUrwlCnOkDxKLNHaINnmksYPksy7S2CHa5JHGDtEmjzR2iDZ5pLFDtMkjjR0kn3WRxg7RJp+3sd8Shfkzn\/nMXEeYikhjh2iTRxo7SD7rIo0dok0eaewQbfJIY4dok0caO0SbPNLYQfJZF2nsEG3yeRv7LVGYAQAA4HQtqK+vn+sMIYRw9913L1++fK5TAAAAcEY5evTo+vXrV6xYMYVr50thfv755+c6AgAAAGemqRXmRVXPMTVTSw8AAAAzxD3MAAAAkEJhBgAAgBQKMwAAAKRQmAEAACCFwgwAAAApFGYAAABIoTADAABAigXf\/va35zoDAAAAzDsL6uvr5zoDAAAAzDuOZAMAAEAKhRkAAABSKMwAAACQQmEGAACAFAozAAAApFCYAQAAIIXCDAAAACkUZgCYRzo7Ozs7O+cwQHt7e29vbz6fnzyye\/fuXC43h6kAYE4ozADA62pvb9+yZctcpwCAubForgMAAPNUZ2dnLpc7ePDg0qVL5zoLAMwBhRkApig5O71t27YQwubNm2+44YY777wzhNDW1rZ48eIQQm9vb1dX1+tdnsvlduzYsWrVqhDC+Pj4rl27yuVyCGHp0qW7d+9etWrV5MHNmzcn0x4\/fry7u3vPnj2vF2np0qVLly5NLr\/zzjtvuOGGmpqayVN1dnZeeumlIYTBwcEk\/EmDE7MNDAxs27ato6OjoaGhGn8wAIiMI9kAMEUDAwO1tbXJ7b5XXnnl+Pj4008\/ff311997772FQuHLX\/7ytddeO\/lm4JO0traOjY0VCoWtW7eOj49v2rQpGV+zZk1fX1+hUBgZGbn55ptzuVw+n7\/hhhuSae+9994bbrjhFNNecMEF3\/jGN5I5b7jhhl27dt16660hhGT+jo6OTCazdevWW2+9tba2tqOjI4TQ3t5eW1t76623bt26NZPJTEx1irYPAG8FdpgBYIr6+\/sLhUJTU9Po6GhDQ0NfX9+hQ4dCCNdcc80DDzywZ8+e19sHTiRlNYSQzWZramomxg8fPvzAAw+EEB588MEbbrjhqquuWrlyZbJcCKGrq+vUPbZSqSTrjo2NjY2NlcvlXC43Pj4eQsjlcg0NDUNDQ5VKpVKpHDhwoKGhIZfLNTY2joyMJPvPSVef1t8FAM4UCjMATFG5XD5w4EBjY2NSZfv7+yuVyvbt23fs2HHHHXeEEA4fPrx9+\/ZKpZJ6+cTztI4fP37s2LGJ8bGxseSSgwcPJuN1dXXj4+Ojo6PTDJw08+Hh4cmDq1evzmQyQ0NDydsjR45MDgMAb2WOZAPA1D388MPnnXfeddddl+zlhhAqlUpLS0uhULj11ltrampaW1tTL8zlctdcc83+\/fsLhcK2bduSHeCTrFmzZsmSJSGE4eHhmpqabDY7zbSjo6Pj4+N1dXWTBw8dOjQ2NjYx+cqVK5NFAQCFGQCm7qGHHjp+\/PgHPvCBBx98MISQz+d7e3vb29vDP7TTk7ZzT5LU1PXr1yeP\/krkcrnNmzeHEK677rrx8fGHHnoo2cFuamoKIWzevLm3tzf5wOmqVCpDQ0PJMex8Pr927drkePbAwMDEoo2NjVOYGQDOSI5kA8DUTVTQhx56KIRQLpfvvPPOtra25Kz14ODg691vXKlU+vr6tmzZUiwWBwcHBwcHJ\/Z4n3rqqeuvv\/7GG29MHm2d3G88edre3t5T3x19Ch0dHZ2dncmJ8cHBweQ+6q6urrq6uhtvvPHGG2+cOAcOACyor6+f6wwAAAAw7ziSDQAAACkcyQaAGbR58+a2trbFixefNN7b2zvlbznO5XI7duyYfNtzYnBwcNu2bVObEwB4LUeyAQAAIIUj2QAAAJBivhzJvvvuu5cvXz7XKQAAADijHD16dP369StWrJjCtfOlMC9fvry5uXmGJi+VSjFOHmnseCePNLbJZ3lmk8\/yzCaf\/ckjjR3v5JHGjnfySGObfJZnNvkszzwLk0\/5WkeyAQAAIMUUC3Mul9u9e3d7e3vyNp\/P9\/b2FovF3bt353K51JHkkmKx2Nvbm8\/nq\/ULvBlf+cpXZnO5aok0dog2eaSxg+SzLtLYIdrkkcYO0SaPNHaINnmksUO0ySONHSSfdZHGDtEmn7exp1KY8\/n8n\/7pn07+NovW1tYDBw5s3bo1hLB+\/frXGxkbGysUCiMjI5s2bapO\/Denu7t7Nperlkhjh2iTRxo7SD7rIo0dok0eaewQbfJIY4dok0caO0SbPNLYQfJZF2nsEG3yeRv7tAtzLpdrbW298847x8fHk5F8Pl9bWzs8PFypVIaGhhobG187ksvlGhoaRkdHQwgDAwMNDQ3JtjMAAADMT6ddmCuVyrZt2w4ePDh58NixY0eOHEleZzKZVatWnTSyevXqEMLw8HAyUlNTk81mpxV83ti7d+9cR5iKSGOHaJNHGjtEmzzS2CHa5JHGDpLPukhjh2iTRxo7RJs80tgh2uSRxg7RJo809jQtqK+vn8Jl+Xz+lltuKRaLXV1d+Xz+s5\/97Ne+9rU9e\/Z0dHQ0NDTcddddLS0tk0duv\/32m266qa+vr6urq729vVAo7Nq1q1wuT0xYLBZPWmLv3r379u2b1i8HAADAW8mGDRs2btx40mBzc\/Ncfq3UkiVLVq5cmbweGxs7fPjwSSOHDh0KIdTV1SUj4+PjyfHsyWbuMeIzqqmpqb+\/f65TnLZIY4dok0caO0SbPNLYIdrkkcYOks+6SGOHaJNHGjtEmzzS2CHa5JHGDtEmjyV2f3\/\/zp07J4\/M8ddKlcvlkZGRurq65EblgYGB144kNzMnx7AbGxuHhoYqlcr0lwYAAIAZUp3vYe7p6Vm7du0dd9wxNjbW1dX1eiOZTKZYLGYymZ6enqqsCwAAADNkikeyy+Xyli1bXu9t6kilUmlpaZnacgAAADDLqrPDDAAAAGcYhRkAAABSKMwAAACQQmEGAACAFAozAAAApFCYAQAAIIXCDAAAACkUZgAAAEihMAMAAEAKhRkAAABSKMwAAACQQmEGAACAFAozAAAApFCYAQAAIIXCDAAAACkUZgAAAEihMAMAAEAKhRkAAABSKMwAAACQQmEGAACAFPOoMJdKpVKp1NbWNtdBAAAAiFtbW1vSMaczyaJqpZm+5ubmuY4AAADAmaC7u7u7uzuEMJ3OPI92mAEAAGD+UJgBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkUJgBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkUJgBAAAghcIMAAAAKRRmAAAASLGoKrO0t7dv2bIleb1\/\/\/6Ojo58Pn\/LLbfU1NQcPnx4+\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\/v6OjI5\/O33HJLTU3N4cOHt2\/fXqlUcrncjh07Vq1aNT4+vmvXrnK5XJWlAQAAYCZUZ4e5rq5ucHCwUCgUCoWOjo4QQmtr64EDB7Zu3RpCWL9+fTIyNjZWKBRGRkY2bdpUlXUBAABghlSnMGez2dHR0Ym3+Xy+trZ2eHi4UqkMDQ01NjbmcrmGhobkMwMDAw0NDblcripLAwAAwEyoQmFO6vG6deuKxWJvb28+nw8hHDt27MiRI8kHMpnM6tWrQwjDw8PJSE1NTTabnf7SAAAAMEOqcA9zuVyeuIG5o6Pj5ptvvuuuu6YwT6lUmvx27969+\/btm368WdDU1DTXEaYi0tgh2uSRxg7RJo80dog2eaSxg+SzLtLYIdrkkcYO0SaPNHaINnmksUO0yaOIvWHDho0bN1ZrtikW5s2bN7e1tS1evHjimV7J+PDw8Nq1azOZzJIlS1auXJkMjo2NHTp0KIRQV1eXjIyPj08+wp1obm6eWpi51dTU1N\/fP9cpTluksUO0ySONHaJNHmnsEG3ySGMHyWddpLFDtMkjjR2iTR5p7BBt8khjh2iTxxK7v79\/586dk0dO2po9LVM8kr1nz55NmzYVCoWWlpZsNrt79+7kJHZjY+PIyMjXvva1kZGRurq65NblgYGB5Gbm5Bh2Y2Pj0NDQRMcGAACAeagK9zCXy+W+vr7bbrutWCzW1tb29PSEEHp6etauXXvHHXeMjY11dXUlI5lMplgsZjKZ5DMAAAAwb1Xne5i7urqSVjxh8o3NiUql0tLSUpXlAAAAYKZV52ulAAAA4AyjMAMAAEAKhRkAAABSKMwAAACQQmEGAACAFAozAAAApFCYAQAAIIXCDAAAACkUZgAAAEihMAMAAEAKhRkAAABSKMwAAACQQmEGAACAFAozAAAApFCYAQAAIIXCDAAAACkUZgAAAEihMAMAAEAKhRkAAABSKMwAAACQQmEGAACAFPOoMJdKpVKp1NbWNtdBAAAAiFtbW1vSMaczyaJqpZm+5ubmuY4AAADAmaC7u7u7uzuEMJ3OPI92mAEAAP7\/9u4vNqorsR\/4Qb8QkFxskRKr1EUTu5VW2S0NeF5iUR66QdTlX93lod3yR7IcL6GylGiVPJQkWkeAoyqrVVayCqzroiVNt1XkyiWBzqabbaXIcl7GhpLVaqXIy4g6jhxSZFOrBB76e7jq1DWXCdhm5h7y+Tys7Dt3zv367jAnX597x5AdCjMAAACkUJgBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkUJgBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkUJgBAAAgxSILcy6XO3PmTHd3d\/JtPp8fGhoqFApnzpzJ5XKpW5KnFAqFoaGhfD6\/XD8AAAAA3A+LKcz5fP573\/teY2NjeUtnZ+elS5cOHToUQti+ffudtszOzra3t09NTe3evXt54gMAAMD9cc+FOZfLdXZ2vvHGG3Nzc8mWfD6\/fv36ycnJUqk0MTHxxBNP3L4ll8u1tLRcvXo1hHDx4sWWlpZk2RkAAACy6Z4Lc6lU6unpuXLlyvyNN2\/e\/PTTT5Ov6+vrGxsbF2z59V\/\/9RDC5ORksqWurm7dunVLCg4AAAD300O1DvC\/RkdH53977ty58+fP1yrMPWltba11hMWINHaINnmksUO0ySONHaJNHmnsIHnVRRo7RJs80tgh2uSRxg7RJo80dog2eRSxd+zYsXPnzuUa7a4Kc3d39969e0MIH330UU9Pz+07PPzww48++mjy9ezs7PT09IItH3\/8cQihqakp2TI3N5dcnj1fW1vbon6EGnvppZeOHTtW6xT3LNLYIdrkkcYO0SaPNHaINnmksYPkVRdp7BBt8khjh2iTRxo7RJs80tgh2uSxxB4bG1uQc8HS7D25q0uyBwYG2tvb29vbU9tysVicmppqampKblS+ePHi7VuSm5mTy7CfeOKJiYmJUqm06NCZsoy\/vaimSGOHaJNHGjtEmzzS2CHa5JHGDpJXXaSxQ7TJI40dok0eaewQbfJIY4dok0cae4mW5+8wnz59euPGjadOnZqdnR0YGLjTlvr6+kKhUF9ff\/r06WU5LgAAANwniyzMxWJx7969SRMufzt\/Cfr2LaVS6eDBg+3t7QcPHqzy8nJXV1c1D7dcIo0dok0eaewgedVFGjtEmzzS2CHa5JHGDtEmjzR2iDZ5pLGD5FUXaewQbfLMxl6eFeaMe\/rpp2sdYTEijR2iTR5p7CB51UUaO0SbPNLYIdrkkcYO0SaPNHaINnmksYPkVRdp7BBt8szG\/lIUZgAAALhXK5qbm2udIYQQ3nrrrTVr1tQ6BQAAAA+U69evb9++vaGhYRHPzUphnpmZqXUEAAAAHkyLK8x39XeYq2Bx6QEAAOA+cQ8zAAAApFCYAQAAIIXCDAAAACkUZgAAAEihMAMAAEAKhRkAAABSKMwAAACQYsW\/\/Mu\/1DoDAAAAZM6K5ubmWmcAAACAzHFJNgAAAKRQmAEAACCFwgwAAAApFGYAAABIoTADAABACoUZAAAAUijMAAAAkEJhBoClyuVyZ86c6e3tXfpQ+Xx+aGioUCgsy2h3r7e398yZM7lcrrylv7+\/yhkAIGseqnUAAOB\/7d69e25u7tvf\/napVKphjP7+\/t\/6rd+6evVqDTMAQM0pzACwVKVS6eDBg8s12uzsbA3bcj6fP3LkyM2bN69du1arDACQEQozACxVLpc7evToxMTEhQsXurq6Vq5cmWy\/devW4Ldq4DAAACAASURBVODg8PBw6rM6OjrKO3\/00Uc9PT29vb1PPvlkCGFoaKivr2\/Dhg3JDpXHSY5+9erVXC5XV1c3PT394x\/\/+E\/+5E9Wrlw5PT398ssvl0qlpAbX1dUlgw8MDJSf2NjYeOvWrU8++aQ84BtvvDE+Pn706NFlPUkAEB\/3MAPAshkeHt69e3d7e3t7e\/tHH31UKpUqtNxvfOMbZ8+ebW9vP3ny5Pr16zs6Onp7ez\/44IPp6elvf\/vbIYQDBw4kO5w9e\/bAgQP5fL7CoXO5XF9f34svvlhXV\/f7v\/\/7PT09J0+eXLt27fbt23O53LPPPnvp0qX29vahoaE9e\/Z0dHSEEDo7O0MIhw4d6u3tfeSRR5JxisXinTIDwJeNwgwAy6+3t7e+vv6111670w7JVdzJSu+jjz768MMPL9ihtbV1bm7u3XffDSG8++67c3Nzra2tFY546dKlYrF49erVubm5iYmJUql05cqVmzdvhhA2b95cV1d34cKFZKhr165t2rQpl8u1tLQkexaLxUuXLi39pwaAB4xLsgFgmXV3d2\/cuLGvr6\/yrcjJB2uFEObm5pJmO19TU1NjY+OpU6fmb1lcnkcffTSEcOXKlfkb161bV1dXNzk5mXw7OTnZ0tKyuPEB4EGlMAPAcuro6Ghvb3\/jjTeKxWLl3davX3\/y5Mnh4eGOjo4DBw4s2GFycrJ8B\/ISI3366achhA0bNsyPlKxFl0v4ots4ADzAXJINAMsmn88fOHCgUCjczW3ADz\/8cLL2u23bttsvyR4bG6urq9u+fXsIoaOj4+233+7u7l5cqvHx8bm5uU2bNoUQtm\/fvnbt2gsXLpRKpYmJiY0bN+bz+eTy7MUNDgAPMIUZAJbN7t276+rq9u7dW\/gfd2q5w8PDpVIp2TO52Tgpz2XFYvGNN97Ys2dPoVB45plnisVicsPzIpRKpe9\/\/\/sbN24sFAp79+49e\/Zs0ud7e3unpqaOHz9+6tSp2dnZxQ0OAA+wFc3NzbXOAAAAAJljhRkAAABS+NAvALiPuru79+7du2DjrVu3BgcH7\/XPHXd0dHR1da1cuXLB9qGhoUVfrQ0AVOCSbAAAAEjhkmwAAABIkZVLst966601a9bUOgUAAAAPlOvXr2\/fvr2hoWERz81KYV6zZk1bW9t9Gnx0dDTGwSONHe\/gkcY2eJVHNniVRzZ49QePNHa8g0caO97BI41t8CqPbPAqj1yFwRf9XJdkAwAAQIovRWH+q7\/6q1pHWIxIY4dok0caO0hedZHGDtEmjzR2iDZ5pLFDtMkjjR2iTR5p7CB51UUaO0SbPLOxvxSFeXBwsNYRFiPS2CHa5JHGDpJXXaSxQ7TJI40dok0eaewQbfJIY4dok0caO0hedZHGDtEmz2zsL0VhBgAAgHuVocI8Ojo6Ojra1dVV6yD35ty5c7WOsBiRxg7RJo80dog2eaSxQ7TJI40dJK+6SGOHaJNHGjtEmzzS2CHa5JHGDtEmjy52V1dX0jGXMsj\/W7t27XIFWor9+\/e3tbUNDg6Oj4\/XOsu9+c\/\/\/M+pqalap7hnkcYO0SaPNHaINnmksUO0ySONHSSvukhjh2iTRxo7RJs80tgh2uSRxg7RJo8u9vj4+ODg4ODg4NNPPz04OLh69epFDJKhFWYAAADIDoUZAAAAUijMAAAAkEJhBgAAgBQKMwAAAKRQmAEAACCFwgwAAAApFGYAAABIoTADAABACoUZAAAAUijMAAAAkEJhBgAAgBQKMwAAAKRQmAEAACCFwgwAAAApqleYc7ncmTNnCoXC0NBQPp+v2nEBAABgEapXmDs7O2dnZ9vb26empnbv3l214wIAAMAiVKkw53K5lpaWq1evhhAuXrzY0tKSy+Wqc2gAAABYhKrewzw5OZl8UVdXt27dumoeGgAAAO7Jiubm5iocJpfLHT169P333x8YGOju7m5vb+\/r6ysWi+UdCoXCgqecO3fu\/PnzVcgGAADAg2HHjh07d+5csLGtra2hoWERoz20HJHuVlNTU\/LF3Nxccnn2fG1tbdUMs1xaW1vHxsZqneKeRRo7RJs80tgh2uSRxg7RJo80dpC86iKNHaJNHmnsEG3ySGOHaJNHGjtEmzyW2GNjY8eOHZu\/ZXR0dNGjVemS7FKpNDExkVyG\/cQTT0xMTJRKpeocGgAAABahevcwnz59ur6+vlAo1NfXnz59umrHBQAAgEWo3iXZpVLp4MGDVTscAAAALEVVPyUbAAAAYqEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkUJgBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkUJgBAAAgRYYK8+jo6OjoaFdXV62DAAAAELeurq6kYy5lkIeWK83StbW11ToCAAAAD4LBwcHBwcEQwlI6c4ZWmAEAACA7FGYAAABIoTADAABACoUZAAAAUijMAAAAkEJhBgAAgBQKMwAAAKRQmAEAACCFwgwAAAApFGYAAABIoTADAABACoUZAAAAUijMAAAAkEJhBgAAgBTVLszd3d1nzpzJ5XJVPi4AAADck4eqebDe3t4nn3xyenq6mgcFAACARajeCnN3d3cI4YMPPqjaEQEAAGDRqleYBwYGent7q3Y4AAAAWIoVzc3N1Txeb29vS0vLyy+\/XCqV5m8vFAoL9jx37tz58+erGA0AAIC47dixY+fOnQs2trW1NTQ0LGK0+3gPcy6XO3r0aGNj461btwYHB4eHhyvv39bWdv\/C3D+tra1jY2O1TnHPIo0dok0eaewQbfJIY4dok0caO0hedZHGDtEmjzR2iDZ5pLFDtMkjjR2iTR5L7LGxsWPHjs3fMjo6uujR7mNhLpVKBw8evH\/jAwAAwP3j7zADAABAiqr+WakQgs\/9AgAAIApWmAEAACCFwgwAAAApFGYAAABIoTADAABACoUZAAAAUijMAAAAkEJhBgAAgBQKMwAAAKRQmAEAACCFwgwAAAApFGYAAABIoTADAABACoUZAAAAUijMAAAAkEJhBgAAgBQZKsyjo6Ojo6NdXV21DgIAAEDcurq6ko65lEEeWq40S9fW1lbrCAAAADwIBgcHBwcHQwhL6cwZWmEGAACA7FCYAQAAIIXCDAAAACkUZgAAAEihMAMAAEAKhRkAAABSKMwAAACQQmEGAACAFAozAAAApFCYAQAAIIXCDAAAACkUZgAAAEihMAMAAEAKhRkAAABSPFS1I3V3d+\/duzeEcOvWrcHBweHh4aodGgAAAO5VlVaY8\/n817\/+9ZMnT7a3txeLxQMHDuTz+eocGgAAABahSivMxWLxm9\/8ZvL1hQsXvvKVr1TnuAAAALA4NbiHedOmTZ999lmxWKz+oQEAAOAurWhubq7m8bq7u7du3fryyy+XSqX52wuFwoI9z507d\/78+SpGAwAAIG47duzYuXPngo1tbW0NDQ2LGO0+XpKdy+WOHj3a2NhY\/pSv3t7edevWHTx4MHX\/tra2+xfm\/mltbR0bG6t1insWaewQbfJIY4dok0caO0SbPNLYQfKqizR2iDZ5pLFDtMkjjR2iTR5p7BBt8lhij42NHTt2bP6W0dHRRY92HwtzqVSa3417e3tDCD09PffviAAAALBcqvShXx0dHfl8fuXKlcml13Nzc319fW5jBgAAILOqVJiHh4f94WUAAAAiUoNPyQYAAIDsU5gBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkUJgBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFIozAAAAJAiQ4V5dHR0dHS0q6ur1kEAAACIW1dXV9IxlzLIQ8uVZuna2tpqHQEAAIAHweDg4ODgYAhhKZ05QyvMAAAAkB0KMwAAAKRQmAEAACCFwgwAAAApFGYAAABIoTADAABACoUZAAAAUijMAAAAkEJhBgAAgBQKMwAAAKRQmAEAACCFwgwAAAApFGYAAABIoTADAABACoUZAAAAUlSvMHd0dLz99tuFQmFoaCifz1ftuAAAALAIVSrMuVzuG9\/4xtmzZ9vb26empjo7O6tzXAAAAFich6pzmFKpdPDgwfK3V69erc5xAQAAYHGqeg9zclX2+vXr33777WoeFwAAAO7Viubm5iofsru7u729va+vr1gsljcWCoUFu507d+78+fPVjQYAAEDEduzYsXPnzgUb29raGhoaFjHafbwkO5fLHT16tLGx8datW4ODg8PDw8n2Tz\/9NISwYcOG+YU5hNDW1nb\/wtw\/ra2tY2NjtU5xzyKNHaJNHmnsEG3ySGOHaJNHGjtIXnWRxg7RJo80dog2eaSxQ7TJI40dok0eS+yxsbFjx47N3zI6Orro0e5jYZ5\/33IulxsYGDh37tzw8PCmTZvm5ubGx8fv36EBAABgiap0D3OpVDp37lxXV1ehUNi4ceP3v\/\/9UqlUnUMDAADAIlTpU7JDCMPDw+WrsgEAACDjqvop2QAAABALhRkAAABSKMwAAACQQmEGAACAFAozAAAApFCYAQAAIIXCDAAAACkUZgAAAEihMAMAAEAKhRkAAABSKMwAAACQQmEGAACAFAozAAAApFCYAQAAIIXCDAAAACkyVJhHR0dHR0e7urpqHQQAAIC4dXV1JR1zKYM8tFxplq6tra3WEQAAAHgQDA4ODg4OhhCW0pkztMIMAAAA2aEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkUJgBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkqHZhzufzQ0ND3d3dVT4uAAAA3JNqF+bOzs66uroqHxQAAADuVVULc3d396pVq6anp6t5UAAAAFiE6hXmfD6\/devWH\/3oR1U7IgAAACzaiubm5uocqb+\/\/+LFi+++++7Ro0fff\/\/9gYGB+Y8WCoUF+587d+78+fPVyQYAAMADYMeOHTt37lywsa2traGhYRGj3cfCnMvljh492tjYeOvWrR\/\/+Me\/93u\/N\/\/u5aGhofmduVAotLW13ack91Vra+vY2FitU9yzSGOHaJNHGjtEmzzS2CHa5JHGDpJXXaSxQ7TJI40dok0eaewQbfJIY4dok0caO4QwOjq66ML80LKnKSuVSgcPHix\/29\/fH\/6nRd++wgwAAACZ4u8wAwAAQIr7uMKcasGyMwAAAGSTFWYAAABIoTADAABACoV5qXbs2FHrCIsRaewQbfJIY4dok0caO0SbPNLYQfKqizR2iDZ5pLFDtMkjjR2iTR5p7BBt8khjL5HCvFS3\/42vKEQaO0SbPNLYIdrkkcYO0SaPNHaQvOoijR2iTR5p7BBt8khjh2iTRxo7RJs80thLpDADAABAii9FYe7q6qp1hMWINHaINnmksYPkVRdp7BBt8khjh2iTRxo7RJs80tgh2uSRxg6SV12ksUO0yTMb+0tRmJ9++ulaR1iMSGOHaJNHGjtIXnWRxg7RJo80dog2eaSxQ7TJI40dok0eaewgedVFGjtEmzyzsb8UhRkAAADu1Yrm5uZaZwghhLfeemvNmjW1TgEAAMAD5fr169u3b29oaFjEc7NSmGdmZmodAQAAgAfT4grzQ8ueY3EWlx4AAADuE\/cwAwAAQAqFGQAAAFIozAAAAJCi9oU5l8udOXOmUCgMDQ3l8\/lax\/kCvb29hUKhUCh0d3ff\/mjys6Q+VFt3il0++YVCob+\/v1bxKqhwwvv7+5OHent7axGtksqvk2SHM2fO5HK5Kgf7Qndzwt9+++2Ojo6axLuTCrE7OjrefvvtQqEQ1wkvb8\/mP88KJ7z8UNYyJyJ9S0nV3d2dzVf1Avl8\/kc\/+lHqm0ZmJ81EavLsz5t3OuHZf4VXeKmEDM+bX3jCMzhpJu6UPOPzZmrsjE+aiTud8IzPm\/G+pZR1d3cv8R9j7QtzZ2fn7Oxse3v71NTU7t27ax2nko6Ojq985Ssvvvji0NDQ1q1bF7yJ5PP5733ve42NjbWKdycVYpdP\/osvvrh+\/fqsveIrJO\/u7q6vrz906NDJkyc3btyYqamo8usk2SGbvxuqkDyXy9XX1w8NDbW3t+\/evXt4eLiGOReoEDufz\/\/xH\/\/x4OBge3v77Ozs9u3ba5jzdhWS9\/b2tre3J\/82p6enT58+XcOcC1SI3dHRsXHjxpMnTyZvKVkrQpXfUtavX\/\/iiy8eOnSopaUla8lv19vbu3fv3lqn+GIdHR29vb2\/8iu\/cvtDmZ00E3dKnv15MzV2lifNRIWXSsj2vJkaO8uTZuJOybM\/b6bGzvKkmbhT8uzPm3d6S4ll0szn81\/\/+tdPnjzZ3t5eLBYPHDiwiDeTGhfmXC7X0tJy9erVEMLFixdbWloy+Kussk2bNt26devq1atjY2MrV67cvHlz+aFcLtfZ2fnGG2\/Mzc3VMGGqCrF7e3t7enpCCMVicWpqqnYZ01VIPjAwcPDgwVKpFEK4efPmlStXahdzoQqxQwi5XG7nzp0ffvhhreJVUCH5unXrVq5c+emnn9Yw3p1UiN3a2vrZZ58l\/6XS09MzMDBQu5gpKr9UEp2dnRMTE8Visfrx7qRC7CtXrty8eTP5+ubNm1l7wVRI3tTUNDU1VSwWS6XS7OxsU1NTDXN+oeQ\/TT744INaB\/kC+Xx+27Ztf\/d3f1d+VZRledIMFZNned6sEDvLk2aomDxkeN6sEDvLk2aomDzL82bl10kig5NmqJg8y\/NmhdgRTZrFYvGb3\/xm8pK+cOFChRdPBbVfYQ4hTE5OJl\/U1dWtW7eutmEqm52dTeabhx9++NFHHy1vL5VKPT09WZuByu4Uuyyfz\/\/qr\/7qhQsXqh7tC1RO3t\/f\/8wzz\/ziF7\/I2jtjhdidnZ2Tk5MTExM1ivYF7pS8tbV17dq1zzzzTDYvGapwwpPf8Wf20rLKr\/COjo5Vq1Zl8Dfld4pdLBa\/+93vHjhw4MiRI9\/97nczuKhyp+STk5Pr16\/P5\/PJulDGZ6KBgYGsLWymKhaLPT09qZU445NmheRlGZw3vzB2ZifNyskzO29WiJ3xSbPyCc\/svPmFr\/DMTpoVkmd53qwQO65Js2zTpk2fffbZIt4AM1GYqa1cLvfss8\/+9Kc\/zdS\/0rvR09OTXAoSxX87hhA6Ojqampoy+Fb+hQYGBpKLnQ4dOlRfXx\/LCW9qaqqrq+vr6zt06FAIobOzs9aJ7s2mTZsmJyeTgheF7u7uI0eO9PX19fX1HTlyJMvXaC0wMDAwNTV1\/Pjx\/v7+zz\/\/vNZxyLpI583oJs0Q7bwZ6aQZIp83o5s0Q7TzZoyTZnd3d0tLy2uvvbaI5z607GkWobyOPzc3l1yenVn19fXJL9uydtVEZRVi5\/P5559\/\/u\/\/\/u+zOet\/4QlPLgXJ2m+27hR706ZNGzZsOHXqVPLt0aNHX3755Uy9sz9gJ3xycjK56SObsUPFE57L5Zqams6dO1ejaJXcKXb5Gq0QwtTU1BNPPFGziHdQ4YQn19mGEPr7+zM+E1FbGZ83K8vsm+GdZH\/erCy6E579efNOsjxpVpD9efNO4po0e3t7161bd\/DgwcU9vcYrzKVSaWJiIvnX+MQTT0xMTGT5TfDChQsrV65ct25da2vrrVu3xsfHa53orlSInc\/nn3322axdAVJWIXl3d3dyjVM+n1+\/fv3FixdrF3OhCrHLH0oxNDQ0PT2dtVn\/wTvh5VtVk0uGsvaGXvktJbnDNoPvMxViL7hGK6ITXv7E6Y6OjvXr12fqOlsyJePz5p1k+T28sozPm3cS7wnP+LxZQWYnzcoyPm\/eSVyTZnKJR7nhL0LtL8k+ffp0fX19oVCor6\/P+CU3w8PDv\/jFL44fP75nz55\/+Id\/KJVK+Xz+zJkz2fzkxrIKsTs7OxsbG48fP57Nz4WvkDz5FIpCoXD8+PFLly5l6kMpIn2dhLs+4VNTU7Gc8GKx+NOf\/vSZZ545derU7OxsRK\/wEMKjjz76+eefZ\/A\/DSu\/Ti5dunT8+PHoTvjAwMDs7OypU6eSew7j6kIRieXN8HaxzJsLRDFppor0pRLFpJkqlnlzgSgmzVSxzJsLxDhpJp+x\/+STTyZv2ov7M8Yrmpub70c4AAAAiFrtV5gBAAAggxRmAAAASKEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkUJgBAAAghcIMAAAAKRRmAAAASKEwAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkeGj\/\/v21zgAAAACZs+Ljjz+udQYAAADIHJdkAwAAQAqFGQAAAFIozAAAAJBCYQYAAIAUCjMAAACkSC\/MMzMz3\/rWt7bM88orr9y4cWOJBxsfH1+WcQAAAKJ24sSJctsaHx9f3pHfeeedyvvcuHHjlVdeWd7jJpIueZcjzz8JiW9961szMzOV99+7d+\/ly5fv5tFysZ3fQy9fvrx3794tW7acOHHiC0eutMLc398\/MjIyMjLy3nvvhRB+8pOf3M3PXMHmzZu\/853vrF69eonjAAAAxOvEiRPT09PvvffeyMjIm2++eezYsfvRXbPv8OHDIyMj58+f\/9rXvpbUzx\/84AcNDQ2pO7\/zzjvJSXvppZf6+voW9OrbH71x48brr7++Z8+ekZGRxsbG06dPhxBmZmb6+vpeeuml9957b3p6OvnNQoWR7+qS7NWrV+fz+StXroT\/u0r8zjvvnDhxIvnlRPl3A0lNv3z58vPPP\/\/KK6\/M\/5VJ8tyZmZnb908eLW+xEA0AADyQbty4MT09nc\/nk6XExx57bNu2bR988EH4v5f6zi9WZ86cSdY\/x8fHkx1SH12wOpo0tQqL2B999FGy1lpekS6XsvJTyuux88cvt7nyE8vHev311+\/LWQvhypUryUlraWkJIUxMTFR+9PPPP79+\/fpv\/\/ZvhxCefPLJ8fHxmZmZa9eu1dfXt7S0JCW3WCzeuHGjwsh3VZhv3LhRLBY3bNhQYZ+kkff39\/\/kJz9JzuMvf\/nLfD4\/MjKyf\/\/+s2fPLijAC\/afmZk5ceJEf39\/UvTv7owBAABEJqlqr776annt8PDhw4cPH56\/Ivrmm2+OjY2Vi9UjjzwyMjKybdu2Y8eOHTlyZMGjc3NzIyMjnZ2dP\/zhD+fXrtOnTzc2No6MjPT39x87duz2y5j\/+Z\/\/+a\/\/+q\/ffPPNf\/zHf7x8+fLly5f\/8i\/\/8s033xwZGfnzP\/\/zs2fPzszM\/PCHP3zppZdGRkZeeumlf\/qnfwrz1mPffPPN06dPJ706Wb9977339uzZ87Of\/Wwp5+f2G4STZdrp6emmpqYQwqpVq5qamiYnJ8tPSX302rVrIYS1a9cm\/\/vf\/\/3f165du3bt2po1a1atWhVCSHabnZ2tMPJDFYL29PSUv96\/f\/+uXbsq7Fxu5M3NzcmWRx55pNzmz549W3n\/iYmJpqamxx9\/fPXq1Xv27Ll9fwAAgAfDrl27tm7d+sILL2zZsiWEsH\/\/\/sOHD69evfo73\/lOssPatWvLVyaXi9WGDRu2bdv22GOPzczMzH\/0D\/7gD0IIW7du\/dd\/\/ddPPvkk2T4zMzM+Pn748OEQwuOPP\/47v\/M7165de+yxx+bH2LNnT0NDQ0NDQ2tr64cffrhr166BgYHkoaampmKxOH\/nzZs3b968OVlMTdpcsjY+OTn5+OOPT09P79mzZ\/Xq1Y8\/\/vj27duXcnIaGhp+8IMfLNg4MzMzv8cu8Pnnn9\/+6LVr12ZnZxdsvH231OeWVSrM\/f39mzdvvnz58vHjx5P\/DypIGvl8DQ0NSZu\/m\/0rRAQAAHjAlGvhjRs3\/uIv\/uKdd97ZtWvX+Ph4edny137t18p7VihWlR+dvwiaz+c3b948\/9HbS9yJEyf+5m\/+Jvl6+\/btq1ateu6551544YWf\/exnX\/va11577bVkbfbVV1999dVXk932799fuXMui2Tt954eXbt2bX19\/YKNt\/8ioPLIX3xJ9mOPPfaHf\/iHt99UHUJI7mpeFhUiAgAAPDCSD3Uq16vyJ0aVb1NNPgerfOnuF46WXHt87dq1BZWtvr4+ub46cfslw0nLTa5nDiGMj4+Pj4+fP38+uYo72Scp9iMjI4cPH3799dc\/\/\/zzMO\/zoZPtlTvnvUq9JHv16tWNjY1J4KSfzz9i6qPJ7xHKJ2fFihVr165du3bt9evXk58i2a2+vr7CyHd1D\/O2bduampref\/\/95Nt\/+7d\/++STT5Il\/uU6KS0tLZOTkz\/\/+c9v3LjhemwAAOBB1dDQ0NjY+Ld\/+7fJtzMzM2fPnl3wiVHvv\/\/+L3\/5y7sZ7T\/+4z8+\/PDDEMKHH374G7\/xG\/PXpX\/zN38zues4+eCu2+tb8pFXn3zyyb\/\/+78nV30nyqVsZmbm+eefL9\/83NjY2NDQkM\/nk8+oSprtO++8k3T+ZOPPf\/7zd999997Pyv8qV\/T5nTyEsGHDhiRw8qFcyQd0ld3+6KpVq9asWZOcnA8++GDz5s3Javzs7OzExMT8a8srjHy3n5K9Z8+e06dPX758efPmzdu2bdu3b98LL7zwu7\/7u0s5EQtOyuHDh3t6evbt2\/fVr351uYYFAADImqQBJsunO3bs2LNnz65duxoaGvbs2dPT07Nly5b\/+q\/\/Su46\/sKhmpubi8Xili1bzp49+9xzz83\/I76dnZ3T09NbtmzZt29fZ2fnguuxQwhf\/epX9+3bt2\/fvj\/7sz977LHHHn\/88aamph07duzbt2\/r1q3Xr19fvXp1skOyzPunf\/qnIYRdu3Y1NjY+9dRTO3bs2Lx5c7JwvW3bthDCU089dfbs2T\/6oz9axnNVVj5u8slnDQ0N8\/v87Y+uXr36ueeeO3v27JYtW6anpzs7O0MIDQ0NR44cOXbs2FNPPdXY2JiEv\/255YOu+Pjjj+\/HhN3KAgAAAFdJREFUD7MUyYfFJa8hAAAAUl2+fLm\/v\/\/ll1++098uZonuaoW5Csp\/3Wt++wcAAIBayeIKMwAAANRcVlaYAQAAIFMUZgAAAEihMAMAAEAKhRkAAABS\/H+Ol3OyHVw\/ZgAAAABJRU5ErkJggg==","height":636.38425925925924,"width":1022}}
%---
%[output:8992a639]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Error resolving Custom Code."}}
%---
%[output:8bbe89a1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: C:\\Git\\GitHub\\modelization-and-control\\power_electronics_design_with_simscape\\afe_inv_psm_cascaded\\include specified in custom include directory paths string does not exist in any of the following search directories:\n\""}}
%---
%[output:5051c7e1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: C:\\Git\\GitHub\\modelization-and-control\\power_electronics_design_with_simscape\\afe_inv_psm_cascaded\\src specified in custom include directory paths string does not exist in any of the following search directories:\n\""}}
%---
%[output:3bc79f61]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Error resolving Custom Code."}}
%---
%[output:49b0d812]
%   data: {"dataType":"warning","outputData":{"text":"Warning: C:\\Git\\GitHub\\library\\library\\masked_models\\filters\\include specified in custom include directory paths string does not exist in any of the following search directories:\n\""}}
%---
%[output:373387ed]
%   data: {"dataType":"warning","outputData":{"text":"Warning: C:\\Git\\GitHub\\library\\library\\masked_models\\filters\\src specified in custom include directory paths string does not exist in any of the following search directories:\n\""}}
%---
%[output:55adee64]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Error resolving Custom Code."}}
%---
%[output:80b19e09]
%   data: {"dataType":"warning","outputData":{"text":"Warning: C:\\Git\\GitHub\\library\\library\\masked_models\\filters\\include specified in custom include directory paths string does not exist in any of the following search directories:\n\""}}
%---
%[output:7c486149]
%   data: {"dataType":"warning","outputData":{"text":"Warning: C:\\Git\\GitHub\\library\\library\\masked_models\\filters\\src specified in custom include directory paths string does not exist in any of the following search directories:\n\""}}
%---
%[output:45f9ec14]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"Goto\" for \"From\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From1','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From1<\/a>' not found"}}
%---
%[output:2d73f6a9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"Goto\" for \"From\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From10','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From10<\/a>' not found"}}
%---
%[output:1a96255a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"Goto\" for \"From\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From6','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From6<\/a>' not found"}}
%---
%[output:5b2598eb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"Goto\" for \"From\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From1','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From1<\/a>' not found"}}
%---
%[output:61de6204]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"Goto\" for \"From\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From10','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From10<\/a>' not found"}}
%---
%[output:78e7cd14]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"Goto\" for \"From\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From6','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-PosNegReactiveCurrentReferences\/From6<\/a>' not found"}}
%---
%[output:9b0cb18b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"Goto\" for \"From\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_measurement_sim\/From6','error')\">afe_abc_inv_psm_n\/afe_measurement_sim\/From6<\/a>' not found"}}
%---
%[output:7cbaace5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"Goto\" for \"From\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/inverter_measurement_sim\/module_1\/From6','error')\">afe_abc_inv_psm_n\/inverter_measurement_sim\/module_1\/From6<\/a>' not found"}}
%---
%[output:11a62f63]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/current_ctrl\/current_transformation\/Goto4','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/current_ctrl\/current_transformation\/Goto4<\/a>' not found"}}
%---
%[output:67e2c55d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/current_ctrl\/current_transformation\/Goto5','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/current_ctrl\/current_transformation\/Goto5<\/a>' not found"}}
%---
%[output:7a6c6b51]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/instantaneus_power\/Goto5','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/instantaneus_power\/Goto5<\/a>' not found"}}
%---
%[output:1dc0695b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/speed_reference_generator\/Goto','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/speed_reference_generator\/Goto<\/a>' not found"}}
%---
%[output:01376725]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/afe\/dclink\/Goto19','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/afe\/dclink\/Goto19<\/a>' not found"}}
%---
%[output:05ac5a66]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/inverter\/inverter\/measurements\/Goto','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/inverter\/inverter\/measurements\/Goto<\/a>' not found"}}
%---
%[output:481acfac]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/inverter\/inverter\/measurements\/Goto1','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/inverter\/inverter\/measurements\/Goto1<\/a>' not found"}}
%---
%[output:23911edf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/inverter\/inverter\/measurements\/Goto2','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/inverter\/inverter\/measurements\/Goto2<\/a>' not found"}}
%---
%[output:2e4616bc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/current_ctrl\/current_transformation\/Goto4','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/current_ctrl\/current_transformation\/Goto4<\/a>' not found"}}
%---
%[output:917a02f1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/current_ctrl\/current_transformation\/Goto5','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/current_ctrl\/current_transformation\/Goto5<\/a>' not found"}}
%---
%[output:6aca6314]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/afe\/dclink\/Goto19','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/afe\/dclink\/Goto19<\/a>' not found"}}
%---
%[output:3be5b0ef]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/afe\/dclink\/Goto8','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/afe\/dclink\/Goto8<\/a>' not found"}}
%---
%[output:8671ea59]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/pmsm\/synchronous_motor\/Goto1','error')\">afe_abc_inv_psm_n\/pmsm\/synchronous_motor\/Goto1<\/a>' not found"}}
%---
%[output:523dbcfc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection<\/a>' is not connected."}}
%---
%[output:6193539e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection<\/a>' is not connected."}}
%---
%[output:64f90118]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection'], 'Outport', 3);\">Output Port 3<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection<\/a>' is not connected."}}
%---
%[output:0baf945e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. eta-Control'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. eta-Control','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. eta-Control<\/a>' is not connected."}}
%---
%[output:54951eb3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. xi-Control'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. xi-Control','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. xi-Control<\/a>' is not connected."}}
%---
%[output:6d6a8838]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Positive-Seq. eta-Control'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Positive-Seq. eta-Control','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Positive-Seq. eta-Control<\/a>' is not connected."}}
%---
%[output:09b8e745]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta'], 'Outport', 3);\">Output Port 3<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta<\/a>' is not connected."}}
%---
%[output:03e637ee]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta'], 'Outport', 4);\">Output Port 4<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta<\/a>' is not connected."}}
%---
%[output:7581076e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageB\/u_grid_pos_xi'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageB\/u_grid_pos_xi','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageB\/u_grid_pos_xi<\/a>' is not connected."}}
%---
%[output:3ceed573]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/inverter\/inverter\/three phase inverter ideal switch based model'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/inverter\/inverter\/three phase inverter ideal switch based model','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/inverter\/inverter\/three phase inverter ideal switch based model<\/a>' is not connected."}}
%---
%[output:49e943e1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone<\/a>' is not connected."}}
%---
%[output:3f89f79b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone<\/a>' is not connected."}}
%---
%[output:1e57ebff]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_alpha_hat'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_alpha_hat','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_alpha_hat<\/a>' is not connected."}}
%---
%[output:8c4574fb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_beta_hat'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_beta_hat','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_beta_hat<\/a>' is not connected."}}
%---
%[output:28d0e30d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone<\/a>' is not connected."}}
%---
%[output:8b591dc3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone<\/a>' is not connected."}}
%---
%[output:37dd8cdf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/afe\/three phase inverter ideal switch based model'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/afe\/three phase inverter ideal switch based model','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/afe\/three phase inverter ideal switch based model<\/a>' is not connected."}}
%---
%[output:1ae9a236]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageB\/u_grid_pos_xi'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageB\/u_grid_pos_xi','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageB\/u_grid_pos_xi<\/a>' is not connected."}}
%---
%[output:63ecff25]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta'], 'Outport', 3);\">Output Port 3<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta<\/a>' is not connected."}}
%---
%[output:5345fe48]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta'], 'Outport', 4);\">Output Port 4<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_voltage_phase_lock_loops\/dq_pll\/pll_stageA\/sequences\/alphabeta2xieta<\/a>' is not connected."}}
%---
%[output:0c7eb187]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/afe\/three phase inverter ideal switch based model'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/afe\/three phase inverter ideal switch based model','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod1\/afe\/three phase inverter ideal switch based model<\/a>' is not connected."}}
%---
%[output:03945430]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection<\/a>' is not connected."}}
%---
%[output:9b36d06b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection<\/a>' is not connected."}}
%---
%[output:5c929ed4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection'], 'Outport', 3);\">Output Port 3<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/FRT-VDE4110\/FRT-Detection<\/a>' is not connected."}}
%---
%[output:49d57574]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. eta-Control'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. eta-Control','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. eta-Control<\/a>' is not connected."}}
%---
%[output:2ef025f6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. xi-Control'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. xi-Control','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Negative-Seq. xi-Control<\/a>' is not connected."}}
%---
%[output:0fff81f6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Positive-Seq. eta-Control'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Positive-Seq. eta-Control','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/AFE_control\/afe_control\/grid_current_sequences_and_reactive_pwr_ctrl\/sequences_current_control\/Positive-Seq. eta-Control<\/a>' is not connected."}}
%---
%[output:7597b93f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/grid\/grid_emulator\/grid_emulator\/voltage_source\/Constant'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/grid_emulator\/grid_emulator\/voltage_source\/Constant','error')\">afe_abc_inv_psm_n\/grid\/grid_emulator\/grid_emulator\/voltage_source\/Constant<\/a>' is not connected."}}
%---
%[output:2fa316cb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/grid\/grid_emulator\/grid_emulator\/voltage_source\/Constant1'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/grid_emulator\/grid_emulator\/voltage_source\/Constant1','error')\">afe_abc_inv_psm_n\/grid\/grid_emulator\/grid_emulator\/voltage_source\/Constant1<\/a>' is not connected."}}
%---
%[output:623d7997]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/grid\/grid_emulator\/grid_emulator\/voltage_source\/Constant2'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/grid_emulator\/grid_emulator\/voltage_source\/Constant2','error')\">afe_abc_inv_psm_n\/grid\/grid_emulator\/grid_emulator\/voltage_source\/Constant2<\/a>' is not connected."}}
%---
%[output:95252d2d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_alpha_hat'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_alpha_hat','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_alpha_hat<\/a>' is not connected."}}
%---
%[output:63819df4]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_beta_hat'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_beta_hat','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/psi_beta_hat<\/a>' is not connected."}}
%---
%[output:7e0f2567]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone<\/a>' is not connected."}}
%---
%[output:5cb15b07]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone','error')\">afe_abc_inv_psm_n\/afe_abc_inv_psm_mod2\/INV_control\/inverter_control\/current_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone<\/a>' is not connected."}}
%---
%[output:309885ca]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/global timing\/From2'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/global timing\/From2','error')\">afe_abc_inv_psm_n\/global timing\/From2<\/a>' is not connected."}}
%---
%[output:28668d21]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/global timing\/From3'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/global timing\/From3','error')\">afe_abc_inv_psm_n\/global timing\/From3<\/a>' is not connected."}}
%---
%[output:62c4052b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/global timing\/From4'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/global timing\/From4','error')\">afe_abc_inv_psm_n\/global timing\/From4<\/a>' is not connected."}}
%---
%[output:56fe9b91]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/global timing\/From9'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/global timing\/From9','error')\">afe_abc_inv_psm_n\/global timing\/From9<\/a>' is not connected."}}
%---
%[output:15b9d690]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/global timing\/Unit Delay'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/global timing\/Unit Delay','error')\">afe_abc_inv_psm_n\/global timing\/Unit Delay<\/a>' is not connected."}}
%---
%[output:43ca510d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/global timing\/Unit Delay1'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/global timing\/Unit Delay1','error')\">afe_abc_inv_psm_n\/global timing\/Unit Delay1<\/a>' is not connected."}}
%---
%[output:49d4fcef]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/global timing\/Unit Delay2'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/global timing\/Unit Delay2','error')\">afe_abc_inv_psm_n\/global timing\/Unit Delay2<\/a>' is not connected."}}
%---
%[output:188a0e6b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['afe_abc_inv_psm_n\/global timing\/Unit Delay3'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/global timing\/Unit Delay3','error')\">afe_abc_inv_psm_n\/global timing\/Unit Delay3<\/a>' is not connected."}}
%---
%[output:7d359ec5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_5\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_5\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
%[output:7028027b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_6\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_6\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
%[output:85cb729a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_1\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_1\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
%[output:138e1198]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_2\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_2\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
%[output:9ad0e33a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_3\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_3\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
%[output:7ead8f06]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_1\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_1\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
%[output:143f1907]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_2\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_2\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
%[output:4f88f548]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_3\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_3\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
%[output:75e8107b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_6\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_6\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
%[output:2a8360d2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_5\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_5\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
%[output:3ab592dc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_4\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/uabc_sin_cos\/simple_moving_average_filter_4\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
%[output:38abe0b3]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Source '<a href=\"matlab:open_and_hilite_hyperlink ('afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_4\/dmavg_flt\/Step','error')\">afe_abc_inv_psm_n\/grid\/power meter iec\/iabc_sin_cos\/simple_moving_average_filter_4\/dmavg_flt\/Step<\/a>' specifies that its sample time (-1) is back-inherited. You should explicitly specify the sample time of sources. You can disable this diagnostic by setting the 'Source block specifies -1 sample time' diagnostic to 'none' in the Sample Time group on the Diagnostics pane of the Configuration Parameters dialog box."}}
%---
