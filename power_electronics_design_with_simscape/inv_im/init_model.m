clear;
[model, options] = init_environment('inv_im');;
%[text] ### Global timing
% simulation length
simlength = 1.25;

% switching frequency and tasks timing
fPWM = 4e3;
fPWM_AFE = fPWM;
tPWM_AFE = 1/fPWM_AFE;
TRGO_AFE_double_update = 1;
if TRGO_AFE_double_update
    ts_afe = tPWM_AFE/2;
else
    ts_afe = tPWM_AFE;
end

fPWM_INV = fPWM;
tPWM_INV = 1/fPWM_INV;
TRGO_INV_double_update = 1;
if TRGO_INV_double_update
    ts_inv = tPWM_INV/2;
else
    ts_inv = tPWM_INV;
end

fPWM_DAB = 3*fPWM;
tPWM_DAB = 1/fPWM_DAB;
TRGO_DAB_double_update = 0;
if TRGO_DAB_double_update
    ts_dab = tPWM_DAB/2;
else
    ts_dab = tPWM_DAB;
end

fPWM_CLLC = 5*fPWM;
tPWM_CLLC = 1/fPWM_CLLC;
TRGO_CLLC_double_update = 0;
if TRGO_CLLC_double_update
    ts_cllc = tPWM_CLLC/2;
else
    ts_cllc = tPWM_CLLC;
end

s=tf('s');
z_afe = tf('z',ts_afe);
z_inv = tf('z',ts_inv);
z_dab = tf('z',ts_dab);
z_cllc = tf('z',ts_cllc);


% deadtimes
dead_time_AFE = 0e-6;
dead_time_INV = 0e-6;
dead_time_DAB = 1e-6;
dead_time_CLLC = 1e-6;

% minimum pulse
minimum_pulse_time_AFE = 1e-6;
minimum_pulse_time_INV = 1e-6;
minimum_pulse_time_DAB = 1e-6;
minimum_pulse_time_CLLC = 1e-6;

% delays
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayINV_modB=0;
delayDAB_modB=0;

% maximum simulation and simscape step
tc = ts_afe/100;

% t_misura = simlength - 0.025;
t_misura = 0.6;
Nc = ceil(t_misura/tc);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);
Ns_dab = ceil(t_misura/ts_dab);
Ns_cllc = ceil(t_misura/ts_cllc);

%[text] ### Settings for simulink model initialization and data analysis
use_mosfet_thermal_model = 0;
use_thermal_model = 0;

if (use_mosfet_thermal_model || use_thermal_model)
    nonlinear_iterations = 5;
else
    nonlinear_iterations = 3;
end
load_step_time = 1.25;
transmission_delay = 125e-6*2;

%[text] ### Local time allignment to master time
kp_align = 0.6;
ki_align = 0.1;
lim_up_align = 0.2;
lim_down_align = -0.2;
%[text] ### Enable one/two modules
number_of_modules = 1;
enable_two_modules = number_of_modules;
%[text] ### Settings for speed control or wind application
use_torque_curve = 1; % for wind application
use_speed_control = 1-use_torque_curve; %
use_mtpa = 1; %
use_psm_encoder = 0; % 
use_load_estimator = 0; %
use_estimator_from_mb = 0; %mb model based
use_motor_speed_control_mode = 0; 

use_dq_pll_fht_pll = 1; % 
use_dq_pll_ddsfr_pll = 0; % 
use_dq_pll_mod1 = 0; % 
use_dq_pll_ccaller_mod1 = 0; % 
use_dq_pll_ccaller_mod2 = 0; % 

use_dq_pll_mode1 = use_dq_pll_mod1;
use_dq_pll_mode2 = use_dq_pll_ccaller_mod1;
use_dq_pll_mode3 = use_dq_pll_ddsfr_pll;
use_dq_pll_mode4 = use_dq_pll_fht_pll;
%[text] ### Settings for CCcaller versus Simulink
use_ekf_bemf_module_1 = 1;
use_observer_from_simulink_module_1 = 0;
use_observer_from_ccaller_module_1 = 0;
use_observer_from_simulink_module_2 = 0;
use_observer_from_ccaller_module_2 = 0;

use_current_controller_from_simulink_module_1 = 0;
use_current_controller_from_ccaller_module_1 = 1;
use_current_controller_from_simulink_module_2 = 0;
use_current_controller_from_ccaller_module_2 = 0;

use_moving_average_from_ccaller_mod1 = 1;
use_moving_average_from_ccaller_mod2 = 0;
%[text] ### Settings average filters
mavarage_filter_frequency_base_order = 2; % 2 means 100Hz, 1 means 50Hz
dmavg_filter_enable_time = 0.025;
%%
%[text] ### Grid Emulator Settings
grid_nominal_power = 1600e3;
application_voltage = 480;
us1 = 480; us2 = 480; fgrid = 60;
eta = 95; ucc = 5.6;
i1m = 5; p_iron = 1200;
up_xi_pu_ref = 1; up_eta_pu_ref = 0; un_xi_pu_ref = 0; un_eta_pu_ref = 0;
grid = grid_three_phase_emulator('wind_grid', grid_nominal_power, application_voltage, us1, us2, fgrid, ...
                eta, ucc, i1m, p_iron, up_xi_pu_ref, up_eta_pu_ref, un_xi_pu_ref, un_eta_pu_ref);
%%
%[text] ## Global Hardware Settings
afe_pwr_nom = 250e3;
inv_pwr_nom = 250e3;
dab_pwr_nom = 250e3;
cllc_pwr_nom = 250e3;
fres_dab = fPWM_DAB/5;
fres_cllc = fPWM_CLLC*1.25;

hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, fPWM_AFE); %[output:65befaf6]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, fPWM_INV); %[output:2a289f6a]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, fPWM_DAB, fres_dab); %[output:5704ba80]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, fPWM_CLLC, fres_cllc); %[output:88516f92]
%[text] ## AFE Settings and Initialization
%[text] ### Behavioural Settings
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

use_vector_control = 1;
use_uf_control = 0;
%[text] ### FRT Settings
test_index = 25; % type of fault: index
test_subindex = 4; % type of fault: subindex
enable_frt_1 = 1; % faults generated from abc
enable_frt_2 = 0; % faults generated from xi_eta_pos and xi_eta_neg
start_time_LVRT = 0.75;
asymmetric_error_type = 1;
frt_data = frt_settings(test_index, test_subindex, asymmetric_error_type, enable_frt_1, enable_frt_2, start_time_LVRT);
grid_fault_generator;
%[text] ### Reactive current limits for grid support
i_grid_pos_eta_lim = 1;
i_grid_neg_xi_lim = 0.5;
i_grid_neg_eta_lim = 0.5;
%[text] ### Reactive current Limits - Red. Dyn. grid support
i_grid_pos_eta_red_lim = 0.1;
i_grid_neg_eta_red_lim = 0.1;
i_grid_neg_xi_red_lim = 0.1;
%[text] ### Grid voltage derivate implemented with double integrator observer
Aso = [0 1; 0 0];
Asod = eye(2)+Aso*ts_afe;
Cso = [1 0];
omega_rso = 2*pi*50;
p2place = [-1 -4]*omega_rso;
p2placed = exp(p2place*ts_afe);
Kd = (acker(Asod',Cso',p2placed))';
l1 = Kd(2) %[output:97f6ba80]
l2 = Kd(1) %[output:6d9e7361]
%[text] ### Linear double integrator observer
Aso = [0 1; 0 0];
Asod = eye(2)+Aso*ts_afe;
Cso = [1 0];
omega_rso = 2*pi*50;
p2place = [-5 -20]*omega_rso;
p2placed = exp(p2place*ts_afe);
Kd = (acker(Asod',Cso',p2placed))';
kv = Kd(2)/ts_afe;
kx = Kd(1)/ts_afe;
%[text] ### Current sensor endscale, and quantization
adc_quantization = 1/2^11;
adc12_quantization = adc_quantization;
adc16_quantization = 1/2^15;
Imax_adc = 1049.835;
CurrentQuantization = Imax_adc/2^11;
%%
%[text] ### Voltage sensor endscale, and quantization
Umax_adc = 1500;
VoltageQuantization = Umax_adc/2^11;
%[text] ### DClink Lstray model
parasitic_dclink_data; %[output:9f375b25]
%[text] ### DClink voltage control parameters
Vdc_norm_ref = 1;
kp_vs = 0.85;
ki_vs = 35;
%%
%[text] ### AFE current control parameters
%[text] ### Resonant PI
kp_afe = 0.2;
ki_afe = 45;
delta = 0.015;
res = s/(s^2 + 2*delta*grid.omega_grid_nom*s + (grid.omega_grid_nom)^2);

Ares = [0 1; -grid.omega_grid_nom^2 -2*delta*grid.omega_grid_nom];
Bres = [0; 1];
Cres = [0 1];
Aresd = eye(2) + Ares*ts_afe;
Bresd = Bres*ts_afe;
Cresd = Cres;
%%
%[text] ### Grid Normalization Factors
pll_i1 = 80;
pll_p = 1;
pll_p_frt = 0.2;
Vmax_ff = 1.1;
ixi_pos_ref_lim = 1.65;
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:23d5f679]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1;
pll_p_ddsrt = pll_p;
omega_f = grid.omega_grid_nom;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
tau_ddsrf = 1/omega_f;
%%
%[text] ### PLL FHT
pll_i1_fht = pll_i1;
pll_p_fht = pll_p;
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega_fht0 = grid.omega_grid_nom;
delta_fht0 = 0.05;
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:18bcb055]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:19671167]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:3bfc9561]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:501fc6e6]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid.omega_grid_nom;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:7c3a646c]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:4b7a2ae7]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:62b55245]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:67ae9a02]
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;
kp_rc_pos_grid = kp_rc_grid;
ki_rc_pos_grid = ki_rc_grid;
kp_rc_neg_grid = kp_rc_grid;
ki_rc_neg_grid = ki_rc_grid;
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = 2*pi*rms_perios/grid.omega_grid_nom/ts_afe;
rms_perios = 10;
n10 = 2*pi*rms_perios/grid.omega_grid_nom/ts_afe;
%%
%[text] ### Online time domain sequence calculator
w_grid = grid.omega_grid_nom;
apf = (s/w_grid-1)/(s/w_grid+1);
[napfd, dapfd]=tfdata(c2d(apf,ts_afe),'v');
apf_z = tf(napfd,dapfd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(napfd,dapfd);
ap_flt_ss = ss(A,B,C,D,ts_afe);
% figure;
% bode(ap_flt_ss,options);
% grid on
%%
%[text] ### INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.125;
%[text] ### IM Machine settings
im = im_calculus(); %[output:828549da]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:00d598ff]
n_sys = psm.number_of_systems;
run('n_sys_generic_1M5W_torque_curve');
torque_overload_factor = 1;

% load
b = psm.load_friction_m;
% external_load_inertia = 6*psm.Jm_m;
external_load_inertia = 1;
%[text] ### Simulation parameters: speed reference, load torque in motor mode
% rpm_sim = 3000;
rpm_sim = 17.8;
% rpm_sim = 15.2;
omega_m_sim = psm.omega_m_bez;
omega_sim = omega_m_sim*psm.number_poles/2;
tau_load_sim = psm.tau_bez/5; %N*m
b_square = 0;

%[text] ### Double Integrator Observer Inverter Side
Aso = [1 ts_inv; 0 1];
Cso = [1 0];
% p2place = exp([-10 -50]*ts_inv);
p2place = exp([-50 -250]*ts_inv);
Kobs = (acker(Aso',Cso',p2place))';
kg = Kobs(1) %[output:6ca5bf04]
kw = Kobs(2) %[output:6d3a4ac1]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/psm.Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/psm.Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:7f01d332]
luenberger_l2 = Klo(2) %[output:78e13a6d]
luenberger_l3 = Klo(3) %[output:2d494d81]
omega_flt_fcut = 10;
phase_compensation_omega = 0; % for generator mode
%[text] ### Control settings
id_lim = 0.35;
%[text] ### Rotor speed control
kp_w = 2.5;
ki_w = 18;
iq_lim = 1.4;
%[text] ### Current control
kp_i = 0.25;
ki_i = 18;

kp_id = kp_i;
ki_id = ki_i;
kp_iq = kp_i;
ki_iq = ki_i;

kp_inv_d = kp_i;
ki_inv_d = ki_i;
kp_inv_q = kp_i;
ki_inv_q = ki_i;

CTRPIFF_CLIP_RELEASE = 0.001;
%[text] ### DAB current and voltage control
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 0.25;
ki_v_dab = 45;
%[text] ### Field Weakening Control 
kp_fw = 0.05;
ki_fw = 1.8;
%[text] ### BEMF observer
emf_fb_p = 0.2;
emf_p = emf_fb_p*4/10;

emf_fb_p_ccaller_1 = emf_fb_p;
emf_p_ccaller_1 = emf_fb_p_ccaller_1*4/10;

emf_fb_p_ccaller_2 = emf_fb_p;
emf_p_ccaller_2 = emf_fb_p_ccaller_2*4/10;
% omega_th = 0.25;
omega_th = 0;
%[text] ### EKF BEMF observer
kalman_psm;
k_kalman = 0;
%[text] ### Motor Voltage to Udc Scaling
motorc_m_scale = 2/3*hwdata.inv.udc_nom/psm.ubez;
inv_m_scale = motorc_m_scale;
%%
%[text] ### Settings Global Filters
filters = setup_global_filters(ts_afe, ts_inv, ts_dab, tc);
%[text] ### Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] ### HeatSink settings
heatsink_liquid_2kW;
%[text] ### DEVICES settings (IGBT)
% infineon_FF650R17IE4D_B2;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.inv = device_igbt_setting_inv(fPWM_INV, hwdata.inv.udc_nom);

% infineon_FF650R17IE4;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.afe = device_igbt_setting_afe(fPWM_AFE, hwdata.afe.udc_nom);
%[text] ### DEVICES settings (MOSFET)
infineon_FF1000UXTR23T2M1;
mosfet.inv = device_mosfet_setting_inv(fPWM_INV, hwdata.inv.udc_nom);

infineon_FF1000UXTR23T2M1;
mosfet.afe = device_mosfet_setting_afe(fPWM_AFE, hwdata.afe.udc_nom);

wolfspeed_CAB760M12HM3
mosfet.dab = device_mosfet_setting_dab(fPWM_DAB, hwdata.dab.udc1_nom);
%[text] ### DEVICES settings (Ideal switch)
silicon_high_power_ideal_switch;
ideal_switch = device_ideal_switch_setting(fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.afe = device_ideal_switch_setting(fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.inv = device_ideal_switch_setting(fPWM_INV, hwdata.inv.udc_nom);
%[text] ### Setting Global Faults
time_aux_power_supply_fault = 1e3;
%[text] ### Lithium Ion Battery
nominal_battery_voltage = grid.udc_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab); %[output:04da29c6]
lithium_ion_battery_2 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab); %[output:9dac27de]
%[text] ### C-Caller Settings
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
Simulink.importExternalCTypes(model,'Names',{'linear_double_integrator_observer_output_t'});

%[text] ### Remove Scopes Opening Automatically
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end
%[text] ### Enable/Disable Subsystems
if use_mosfet_thermal_model
    set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
    set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
else
    if use_thermal_model
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'off');
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    else
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    end
end

if use_torque_curve
    set_param('inv_im/fixed_speed_setting', 'Commented', 'off');
    set_param('inv_im/motor_load_setting', 'Commented', 'on');
else
    set_param('inv_im/fixed_speed_setting', 'Commented', 'on');
    set_param('inv_im/motor_load_setting', 'Commented', 'off');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:65befaf6]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe480V_250kW\nNominal Voltage: 480 V | Nominal Current: 360 A\nCurrent Normalization Data: 509.12 A\nVoltage Normalization Data: 391.92 V\n---------------------------\n","truncated":false}}
%---
%[output:2a289f6a]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv480V_250kW\nNominal Voltage: 400 V | Nominal Current: 470 A\nCurrent Normalization Data: 664.68 A\nVoltage Normalization Data: 326.60 V\n---------------------------\n","truncated":false}}
%---
%[output:5704ba80]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_800V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 800 [V] | Normalization Current DC1: 350 [A]\nNormalization Voltage DC2: 800 [V] | Normalization Current DC2: 350 [A]\nInternal Tank Ls: 1.697653e-05 [H] | Internal Tank Cs: 350 [F]\n---------------------------\n","truncated":false}}
%---
%[output:88516f92]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_800V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 800 [V] | Normalization Current DC1: 350 [A]\nNormalization Voltage DC2: 800 [V] | Normalization Current DC2: 350 [A]\nInternal Tank Ls: 1.280000e-05 [H] | Internal Tank Cs: 350 [F]\n---------------------------\n","truncated":false}}
%---
%[output:97f6ba80]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:6d9e7361]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:9f375b25]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAfcAAAEvCAYAAABYGIhlAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQl0VdXV3kCAMA+GyiSDKBhRwJYo\/UkFwcJf0GgQY4ASEESUQaz6GxmUgAwGpYgiICgIS2TQQhWLRQaDJksqomILiAhGC4KEeZAZ\/rVPuM+XkOTd+d5933fWYoWXnPH7zrnf2+ees3epixcvXiQkIAAEgAAQAAJAIDAIlIK4B4ZLDAQIAAEgAASAgEIA4o6JAASAABAAAkAgYAhA3ANGKIYDBIAAEAACQADijjkABIAAEAACQCBgCEDcA0YohgMEgAAQAAJAAOKOOQAEgAAQAAJAIGAIQNwDRiiGAwSAABAAAkAA4o45AASAABAAAkAgYAhA3ANGKIYDBIAAEAACQADijjkABIAAEAACQCBgCEDcA0YohgMEgAAQAAJAAOKOOQAEgAAQAAJAIGAIQNwDRiiGIxOB7du30+DBg6lChQpUunRpOnjwILVo0YKeeeYZqlWrlq5BLV68mBo2bEht2rSJmH\/9+vU0cOBAatSoUai91NRU6tu3r\/o8ffp0uvvuu6lx48YR60IGIAAE\/IcAxN1\/nKBHUYgAi\/u8efNo5MiRSuAvXLhAL730khLr5ORkXYgYFfeFCxfSc889p9o7ffo0zZkzh86cOaO+ZMTExOhqs7hMHGyS\/\/EXBSQgAATcRwDi7j7maBEIXIZAYXFnsc3MzKR27dpR27Ztaf78+bRo0SJVji3stLQ09X\/+QvDWW28p675JkyZ055130i233EIrVqyg1157jc6dO0ddunRRFjmLuJbYcg8Xd\/79nj17aPTo0TRixAhVtk+fPlSnTh31JeOTTz5Rwt+rVy\/V9qlTp2jKlCmUlZWldhi47vvvv5+++OIL2rJlC+Xm5tKjjz6q2p8wYYL68sB95PqrV6+u2rjhhhvogw8+oKuvvpo6depEb7zxBh05coTGjx+vxoAEBICAeQQg7uaxQ0kgYBsCLO4sjj\/99FOozq5du9LEiRNp48aNtG7dOiWInKZOnUrNmzdXgsq\/T09Pp7Nnz9Lw4cOV+JYvX179fsiQIVSmTBn1xSAuLo64vpLE\/cSJE0pYe\/ToQbwLwOJ+4MABJdT33XcfHTt2TFn6Q4cOpVWrVql2UlJS6Oeff6ann36annzySSXu2dnZqt+lSpWimTNnUs+ePdWXhA8\/\/JB27dqltvsffvhhGjRoEN16660qb8WKFemRRx6hL7\/8kv75z3+qMVndPbCNHFQEBAQiAHEXSBq6HDwEitqWZ8uahTU2NlZZsomJiWrgbHWzeHNiq177PVv2\/A6dLedx48YVAInfr7NgGhV33g3YsWOH+rdp0yb6+OOPadKkSbRkyRL1RaJp06Zq+33atGn0v\/\/7v0rcOfGXAU78hWDz5s307bffKuv\/mmuuoQEDBtCYMWOUFV+zZk21Q6GNozAOwWMaIwIC7iAAcXcHZ7QCBEpEoChR037HFnrHjh1DB+XYMv7Xv\/6ltryLEvfPP\/9cbYFrAsvb6efPn9e1Lc9b6PwlgC1uttz\/\/e9\/q39szdeoUUPtGnC9mmV\/7bXXqn7w75OSkgqIe15eHo0dO1a9FmjdujVt27aNcnJyihR3fv3ABwEh7lgoQMAeBCDu9uCIWoCAJQQKixpbw2wdf\/fdd\/Tb3\/5WWePDhg1TbbAAsxiePHmSPvvsM7Vdz\/9\/\/PHHqV+\/fkpsly1bpqzjsmXLKqs63PLXrP\/CB+o4H2+P8\/t53p5ncV+6dGnoCwT3gQ\/8seW+cuVK9QWCt+V3796t+sBb8+GWO49p7ty5qgxvsb\/44ouq\/0VZ7hB3S9MHhYHAZQhA3DEpgIAPECh8FY4PrF133XVKNNliLupAHVvjs2fPVkLO2+f8XvtPf\/qTspL5iwEfUOOkHcALf4dd0lU4LqOJOx+E43fp\/P6c35XzmYBq1aqpLXne+t+6davamq9cuTL179+\/gLhz2ZdffpmWL19OzZo1U9v23C\/eGeC+hW\/LQ9x9MAnRhUAhAHEPFJ0YDBBwB4HVq1dTgwYNlLDv37+fJk+erHYO+OAeEhAAAt4jAHH3ngP0AAiIQ2Dnzp1q25\/fq\/Oped5hSEhIEDcOdBgIBBUBiHtQmcW4gAAQAAJAIGoRgLhHLfUYOBAAAkAACAQVAYh7UJnFuIAAEAACQCBqEYC4Ry31GDgQAAJAAAgEFQGIe1CZxbiAABAAAkAgahGAuEct9Rg4EAACQAAIBBUBiHtQmcW4gAAQAAJAIGoRgLhHLfUYOBAAAkAACAQVAYh7UJnFuIAAEAACQCBqEYC4Ry31GDgQAAJAAAgEFQGIe1CZxbiAABAAAkAgahGAuEct9Rg4EAACQAAIBBUBiHtQmcW4gAAQAAJAIGoRgLhHLfUYOBAAAkAACAQVAYh7UJnFuIAAEAACQCBqEYC4Ry31GDgQAAJAAAgEFQGIe1CZxbiAABAAAkAgahGAuEct9Rg4EAACQAAIBBUBiHtQmcW4gAAQAAJAIGoRgLhHLfUYOBAAAkAguAicy8ulmFqNgjvACCODuBPR1VdfHbUTAAMHAkAACAQFgdrlzlHLKqeo0xXH6Mry52jTsVh6PreWqeHt3LnTVDm\/FIK4XxJ3yUTylxPJ\/ffLYrDaD\/BgFUHr5cGBdQztqMFtHthKP\/bRPDqW9YbqfpX2fanKbX1MW+5u998OzAvXAXGHuDsxr6KyziA8EKQTBw78waBbPGiifujtDCXkLOo1UkZbBsGt\/lvuaAkVQNwDIO7ff\/89NW7c2Ml5grp1IAAedIDkcBZw4DDAOqt3modTm7Po4NtjiH+yqLOgs7DblSDudiHpcT3SiXR6IXlMj5jmwYP3VIED7zngHjjBA1vpJzdn0bGseXRuXy5VaN6eqrTvQ7HN29s+aOmawIDAcoflbvvCiNYKnXigRSuWZscNDswiZ285O3koauvdyvt0PSOFuOtBSUAe6UTauZAE0OXbLoIH76kBB95zYJfl7tT7dD0ISdeEqLHcL168SO+\/\/z5NnTqVypcvTwMGDKCkpCQqXbq04lk6kXig6VmuzucBD85jHKkFcBAJIXf+boUHPvHOW+9OvU\/Xg4B0TYgacd+6dSvNnj2bxowZQ6VKlaKMjAzq3bs3tWzZEuKuZ6Yjjy4ErDzQdDWATBERAAcRIXIlg1EetPfph5aMUf1z8n26HgAg7npQ8kGexYsXE1vvqampqjeLFi1SP7XP0ok0upB8QEkguwAenKE1c2WuqvjHgyfVz\/8ePJX\/+dCln5c+l9R6g5qx1COhDqV3jl6PZc6wU3SteteCF+\/T9eAgXROixnLPzMykhIQE6tChg+I1OzubcnJyKD09XX2Oe+jtEN+1a9fWw72v8pw7d55iYsr4qk\/R2Jmg8vDT0XOe0Vm3akyo7TpV8v8f\/ru6VWKozqU8d8ZXpl27dlH9+vUL9Hf51uO05+g5+nz3Kdpz7BzxeAbeXJ3uiK9coC7PBhnAhoviocAwD+2ii6umEn3+N6Ia9Yla30Ol\/jjMcyQ6duwY6oN0x2BRcVqexb1t27aUmJhYpLhL\/5am91uy5ysn4B0AD94TrIcD3glYuGEP\/XjwFCU2qU7vDb7J+44HrAdF8RB+lY3fp\/MVtroZH\/ly5NI1wbeW+8mTJ+nTTz+lJUuW0M8\/\/6zIv\/LKKyklJYV+\/\/vfU4UKFQxNiMLb8PyZD9YlJyereqQTqeeBZggwZDaFAHgwBZuthYxwwOKeNP1L1f601HhKvKa6rX2J5srCeSjsGtbr9+l6eJGuCb4Td34v\/tVXX9HMmTPptttuozZt2lDVqlUVFyz4X3zxBb377rvUq1cv9Xe9iQ\/UzZ8\/n0aOHKmKjB8\/ntLS0ig+Ph7irhdE5IuIgBFhiVgZMphCwCgHLPALN+ylzJXfw4o3hXjRhZiHqyqXUv7ew13DOn0\/3a4hQNztQvJSPcePH6cdO3bQjTfeGLqmVriJCxcu0LZt20LCrKcL2lU4\/tLA\/x88eDB16dJFnZznJJ1Iow80PZghj3EEwINxzOwuYZaD7O8OKyse2\/TWGWFL\/cc5T6j36Xb6e7feM\/01SNcE31nu3CEW7x9++IGqV69OFStWpLVr19I333xD1113nToQx9vpdifpRJp9oNmNY7TXBx68nwFWOAjfpv9q1O+9H4ygHhR+n84H5OreMcQR17BuwCJdE3wn7ufOnaNXXnmF3nvvPSpTpowSdBb4Tp06qXfwV1xxBQ0cODBkcdtFsnQirTzQ7MIQ9TjjTxu4GkPA6lpggR+ycKu6Zof38JGxL+59+p6KDUUHs5KuCb4T90OHDtELL7xATzzxBJ06dUo5nXn22WepVq1axFv2f\/3rX2no0KFUo0aNyLPOQA7pRFp9oBmACllLQAA8eD897OIg6ZUvKXvHYXpv0E04aFcErZFcw9rFg1czSrom+E7cDx48qDzJPfLII+oAHYv76NGjqWbNmurzSy+9pFzH8mc7k3QipS8kO7n0si7w4CX6+W3bycHghVvVYTsI\/K+8sqizFzl2EVtSqFU7efBiVknXBIj7pVkjnUjpC8mLxetEm+DBCVSN1Wk3B3wnnk\/Sv9IjnnokyHNwZQy9onObCbVqNw92jMNIHdI1wZfiPnz4cGrSpAnx+\/esrCxq3769utd+9uxZdZJ+4sSJsNwLzVLpC8nIovNzXvDgPTtOcMDWO1vx0Sbwhd+nV2nfl\/ReZXOCBzdnF8TdZrRPnz5NX3\/9NfHPohKflG\/RooXtJ+alEyl9Idk8jTyrDjx4Bn2oYac40K7KpXduHHj\/9JHep+th2Ske9LRtRx7pmuA7y52vwR05ckTdRS8q8b30atWqFXsH3iyp0omUvpDM8ua3cuDBe0ac5EATeN6eZys+aIldwh58e0wo1GqD6d+bHqKTPJjulIGC0jXBd+LOp+WfeeYZFfzh6NGjtH\/\/fmrUKD+K04EDB6h79+70wAMPUOXKlQ3QFDmrdCKlL6TIDMnIAR6858lpDoIm8Gbep+th2Wke9PTBSh7pmuA7cdfI4G35GTNm0L333kv16tVTv2bBf+edd+jhhx\/GtnyhWSt9IVlZhH4qCx68Z8MNDoLgzc7pUKtu8ODkbIO4O4QuX4ljV7HDhg2jSpUqqVZOnDhBU6dOpYceeggH6iDuDs08a9VKf6BZG70\/SrvFgWSB56ts4f7ea6SMtp08t3iwveOXKoS4O4TsmTNnaMKECcrDEfuA56S5oX3qqadguUPcHZp51qqV\/kCzNnp\/lHaTA\/Zm12rcpyL80fO99GNZ80Lv01nQ+fS7U8lNHpwYA8TdCVQv1cmWOodmff\/999XpeXZB269fv1CUODublk6k9IVkJ5de1gUevEQ\/v223OdD80TeoEeu7uPBehlp1mwe7Z550TWA8Sl0s7mi63WjpqI9F\/NixYxQXF1dibj5oV1yeTZs2Kauf8\/D7+oyMDLUDwF8SeFufr9Oxl7ukpKTQqXvpREpfSDqmhogs4MF7mrzgINwfvR8CzhR+n27l1LtZRr3gwWxfiyonXRN8J+7suIaDxnBM95SUFOXMhh3YcGL3s1u2bFHx3Fu1akXdunW7jBP+YsAx2zmka7NmzWjdunX04YcfUmpqKs2dO1e5s+XrdCz4vXv3ppYtW6o6pBMpfSHZuSi9rAs8eIm+N5a7NmI\/BJwp7BqWt92deJ+uh2Xpa0G6JvhO3LVJk5eXR4sXL6Zly5apbTZOHCHu7rvvVtfhivMt\/+OPP9Lbb79NgwYNUl8Kdu\/erQLP3HzzzSq6HIs8J97u56R9lk6k9IWk52EhIQ948J4lrznggDNuRpQrHGpVeZFr38fzUKte82B1JkrXBN+Ku1ViuDzvAixYsEC5rI2NjaU2bdqoePCcsrOzKScnh9LT02G52wE26lAISH+gBYFGP3DgRkS5ot6n1xo81zcU+oEHK2BA3K2gZ0NZvjLXv39\/4vfsHOddE2u24MeNG6dCxXL42FmzZlHbtm0pMTGxWHHXurNmzRobeuZuFewDoH79+u42itYuQwA8eD8p\/MJBxur9tHzrcZrVrTb9rl6sfcAc2kUXP\/8b0aqpRDXqE7W+h0r9cZh99dtUk194MDqcjh07hors3LnTaHFf5ffVgTqryPDZQD44xxY7Cz2\/Uy9duvRl2\/C8Lc8H65KTk2G5WwUd5UMISLdWgkClnziwM2Ss21fZrM4FP\/FgZiyw3M2g5mCZvXv30tixY9WhOs2zHTe3detWmj9\/vvo9p\/Hjx1NaWhrFx+f7h5ZOpPSF5OCUcLVq8OAq3EU25jcOrESUK8o1rJ+23kti2288GJ2Z0jWBx+tby523REaNGqVOybPjGj5F\/+ijj6qt9uLShg0b6MEHH6QGDRqErrnxdjWfkud37Oz1jq17Pk3PznH45DzE3ei0R\/7iEJD+QAsCs37kQBN4IwFnCnuR0xtq1S8c+pEHI9hA3I2gZSAv33fPzMyk22+\/XZ2af\/rpp2nz5s0qvjs81F0OpPSFZGBq+DorePCeHr9yoMddrZ+uslll0q886B0XxF0vUgbz8UG52bNn05\/\/\/GeaNGkSjR49Wl1te+mll5QDmuKuwhlsJpRdOpHSF5JZ3vxWDjx4z4ifOShK4J2KyuY1E37mQQ820jXBt9vybLlPmzZNWe5z5sxR4s7v09944w0VEhYhXwtOT+kLSc9ik5AHPHjPkt850PzR39voND1X+1+OB3DxihG\/8xAJF4h7JIQs\/J3fufMBuP\/85z\/KUufT7Sz4TZs2tVBr0UWlEyl9IdlOqEcVggePgA9r1u8c8Kn3LUtfpVp71tNPMbWVw5mWA5\/zHjibe+B3HiINV7om+NZy14Dnw2\/sUvb8+fNUrVq10CG5SMQY\/bt0IqUvJKN8+TU\/ePCeGT9yUFwAl5Ssaq56s3OTHT\/yYGT80jXBd+J+6NAhte3ODhCKSnzyna+61ahRwwhPEfNKJ1L6QopIkJAM4MF7ovzEgZ4Dcm54s\/OCFT\/xYGb80jXBd+J+4cIFOnLkiLLU+eobb8ezZzlOfFKe3cjecccdoStsZkgrqox0IqUvJLt49Loe8OA1A967ANas9JNbslTs9Njm7aluxkclAmOnsxvvGcjvgfS1IF0TfCfu2sRkC55dxg4ZMoQqVaqkfs3x3fmdO99jh+VecAlLX0h+eSBZ7Qd4sIqg9fJeccBCfixrHvE79ZhajUgFcLmtj\/q\/npS5MpcyV35Pr\/SIJ74PLz15xYNduEHc7UKyUD3Hjx9X2+8PPPBA6AAdb9XzVTh2bFO1alVbW5ZOpPSFZCuZHlYGHjwE\/1LTbnIQ\/i6d\/89WOh+QY2E3k8w4uzHTjhtl3OTBifFI1wTfWu7cMfY2x1fgypYtG7Lc2W3sLbfcYjuX0omUvpBsJ9SjCsGDR8CHNes0B4VDrGpWul1x0\/U4u\/Ee5cg9cJqHyD2wlkO6Jvha3LlzHLZ1\/\/79xO\/i4+LiqFy5ctYYK6a0dCKlLyRHSPWgUvDgAeiFmnSCg6IEvULz9uSUn\/cgCLwTPLg5u6Rrgm\/FvbhT8zgtX\/T0lr6Q3Fy0TrYFHpxEV1\/ddnHgtqAXHp3m7KZBzVj6atTv9Q3eR7ns4sGrIUHcHUJeOzXP99w5HT16lNauXUsxMTHUq1cvKlOmjK0tSydS+kKylUwPKwMPHoJ\/qWmzHLCYn9uXSyc3ryPtpDtvubOFHtu8nen36FYQYYEfsnCrugsvTeDN8mAFLzvLStcE31ruRZHEW\/Qcp51jsONAXUGEpC8kOxell3WBBy\/Rz2\/bCAd8wj1czLm8Juh8MI4PyPkh8V14FvhpqfGUeE11P3QpYh+M8BCxMg8yQNxdBH337t00b948dT1Oj7hv27aNnnvuOZo8ebK6Ovf+++\/T1KlTlRtbDj6TlJQU8ngnnUjpC8nFaeRoU+DBUXh1VV4SB3xN7dTmdXQ2L1fdQQ8Xc6fen+vqtI5M0pzdSF8L0jXBt5Y7R4Xr378\/bdq0KTTtr7vuusvisBe3JrR39nl5eTR9+nT6+eefVZQ5juvOMdwzMjKod+\/e1LJlS1WFdCKlLyQdzzYRWcCD9zQxB3E\/rAuJOG+185a7JuQxv2lEZWs18myr3QpCmrObg3+9zUo1rpSVvhaka4JvxZ3fuf\/yyy\/KgQ2LMSd+\/86ObCpWrFiij3nevp85cyY1bNiQVq9era7TrVq1SpVPTU1VdS1atEj91D5LJ1L6QnLlaeNCI+DBfpDZ2tbSuX0\/hISarW9O4eKt5eOtdU3E\/W6RG0VMuwuf3rkxpXfW5yDHaBt25Je+FqRrgi\/F\/eTJkyq86\/z585UTG47jzond0rL1\/cQTT5QYzz07O5t4S\/6uu+6iZ599Vok7l0tISKAOHTqoujhPTk4Opaenq8\/L\/je\/DU4tW7WyY267WgeHyOXXDUjeIiCWh4NFx3KwDc1DNtRfo35+d2pe+lmjXv7nGvWpFP+t9T3qIzu74ls1QU4bd5+iB5fupTvjK1PG7XG+HKpUHjp27BjCkyOTSk6lLmpH0n0yCnZew5b2xo0blcMazYkNd6958+ZKoPnUPKfw7fuBAwdSnz596PXXX6dHHnmEzp49q7bhNXFnH\/WJiYmqXGFxn9GhCnW\/p7tPEDDejWPHj1GVylWMF0QJWxGQyoNeF6l6wLLLmYuetorKI91i1Dtu7S68X7fopfMAy13vTDSYjy2g3NxcatKkSUjI9VSxfv166tmzZ4GsdevWpfvuu49q1apVYFueLV0+ec9JOpHSF5IebiXkAQ\/esxRNHPj5Lrx0HqRrAq9EX1nufBBu8eLFakt9woQJl4V+NeLEhq16zXLnA3W8zT9y5Ej19GE3tmlpaRQfHw9x9\/55HJgeSH+gBYGIaOMg\/C68n67KSecB4u7jp0G4uGtX4figHb+FGDx4MHXp0iV0WE86kdIXko+nkaGugQdDcDmSORo58KOzG+k8SNcE31nu4audxfmDDz6gffv2hX5drVo1SklJocqVK9v6YJBOpPSFZCuZHlYGHjwE\/1LT0cyBn+7CS+dBuib4Vtw55CtvofM99KZNm4aeGPyevEWLFrafDJdOpPSF5L0k2NMD8GAPjlZqiXYOtLvwXseFl86DdE3wrbiz1c7X1\/jUu3YVzsqCj1RWOpHSF1IkfqT8HTx4zxQ4IPLDXXjpPEjXBN+KOzui4QNwbdq0oeuvv97xJ4Z0IqUvJMcJdqkB8OAS0CU0Aw7ywfE6bKx0HqRrgm\/FnU\/Ns4OZrKwsuuqqq0K+5I2cljfymJFOpPSFZIQrP+cFD96zAw5+5cBLgZfOg3RN8K24u\/2IkE6k9IXkNt9OtQcenEJWf73goCBWXt2Fl86DdE3wrbjzgbolS5Yol7Phib3Vsae5G2+80ZBzm0iPBulESl9IkfiR8nfw4D1T4OByDry4Cy+dB+ma4FtxZw917EaWw7x26tSJzp8\/r0K21qtXT91Tj42NVaFf7UrSiZS+kOzi0et6wIPXDBiL5+59b93tgZtx4aWvBema4FtxP3r0KM2YMUM5m9HutPPvpk2bFvIf\/8wzz9i2MqQTKX0h2UakxxWBB48JIIh7JAbcugsvfS1I1wRfi\/uUKVPUVTj2LseJD9m98MIL9NBDD9HcuXMJ4v7rMpa+kCI9kKT8HTx4zxQ4iMxB5spcylz5PTl5F146DxD3yPPIdI6lS5fSwoUL1XW40qVL01dffUV33HEHlSlThn766Sdsy4chK30hmZ4kPisIHrwnBBzo40C7C98jobYSebuTdB4g7nbPiEL15eXl0fbt2+nChQvqfXtcXJz6f6VKlXCgDuLu8OwzXr30B5rxEfuvBDjQz4mTV+Wk8wBx1z+PDOfkycGH6NihDSc+ZPfdd9\/R888\/H9qqN1xpMQWkEyl9IdnFo9f1gAevGcA7d6MMOCXw0teCdE3geeCrkK\/axDx58qQKy3rzzTfT559\/Tr\/97W\/pv\/\/9L7Vq1Yr+8Ic\/lDh\/2dofN24cbd26lerUqUOjR4+mxo0bqy8KU6dOVX7pBwwYQElJSWq7n5N0IqUvJKMPJL\/mBw\/eMwMOjHPAV+WSpn+pCn416vfGKyiihHQepGuCb8U93Lf8qlWrlPjytjz7mx80aFCxUeHYyp80aRK1bt2a\/vjHP9LHH39M69evp65du9KcOXNUfPdSpUpRRkYG9e7dWwWmgbjbspZRCU5q+2IOSBcVr0C0+y68dB4g7g7NRHZiw9vvbGHn5ubSL7\/8og7WTZ48mYYNG0Y1a9YssuU9e\/aoPHySvmrVqmpLn8ty6Fi+H5+amqrKLVq0SP3UPksnUvpCcmgauV4teHAd8ssaBAfWOLDrqpx0HqRrgm8td+7Yhg0baO3atdSrVy8aMWIE7dixQ4nxww8\/XOxhOj58x3fhOe77mjVr1Hb+U089RW+++SYlJCRQhw4d1MzPzs6mnJwc5b8elru1hwFK\/4qA9AdaELgEB9ZZ1MLGHvzrbaYrk84DxN009cYKahZ4lSpV1La6lnj7vn\/\/\/rRp0yYaOHCgEu8nnnhCOcDhaHLr1q2jd955h6688kpq3749JSYmFivuWp38pUBa2rVrF3FQHSRvEQAP3uLPrYMDeziY9a\/D9Opnh2ngzdXpwVuqG65UKg8dO3YMjXXnzp2Gx+2nAr46UMeOanhLnSdGUSlSVDi23OfNm0cjR45UceBZ\/Pk9e4sWLdT1ufBteT5Yl5ycDMvdT7NReF+kWyvC4VfdBwf2sWjlLrx0HmC52zePVE0s7vxO\/ezZs5SSkqJOy7NIa4mtdt5y1065F26e39WPHTtWWfPNmjVTW\/vvvvsu3XffffTWW28p0efEJ\/HT0tIoPj7feYN0IqUvJJunkWfVgQfPoA81DA7s5cDsVTnpPEjXBJ4FvrLcuUPspIYDxvDVtWXLlimru3v37ur9ebly5SLOXN5KYWudr8Q1adJEvXOvW7euqm\/mzJnqYB37rO\/SpUtoi186kdIXUkRbAMIDAAAgAElEQVRShWQAD94TBQ7s58CMwEvnQbom+FLcw6cmCzG7mn3vvfdo+fLl6sT8Y489VuxVOLPTWjqR0heSWd78Vg48eM8IOHCGA6NX5aTzIF0TfC\/u4VY8xL34RSt9ITnzOHK\/VvDgPuaFWwQHznFgROCl8wBxd2AeWd2WN9Ml6URKX0hmOPNjGfDgPSvgwHkO9NyFl86DdE3wneWuHag7cOAAPfDAA4YP1Jmd1tKJlL6QzPLmt3LgwXtGwIE7HGh34YsLGyudB+ma4Etxt3IVzuy0lk6k9IVklje\/lQMP3jMCDtzjQIsLn965MaV3blSgYek8SNcE34m7e9OyYEvSiZS+kLzi3e52wYPdiBqvDxwYx8xKCe0kfeG48NJ5kK4JEPdLs1o6kdIXkpWHi5\/Kggfv2QAH7nNQ1FU56TxI1wSIO8Td\/SdBgFuU\/kALAjXgwBsW+SR9q3GfUoOasSpsrHQeIO7ezCPbW5VOpPSFZDuhHlUIHjwCPqxZcOAdB+FX5Ua1r07dE\/M9gEpM0jUBljssd4nrzrd9hrB4Tw048J4Dviq3M+84zfzzjZR4jfGgM96PQL5Lcog7xN0P6ygwfYCweE8lOPCeA+7BHyd\/Sht3n6L3Bt0kUuBhuftjHlnuhXQi8UCzPAVsqQA82AKjpUrAgSX4bCvMPCz5thRlrvyeirsLb1tjDlQkXRNgucNyd2BZRG+VEBbvuQcH3nPAPdB4sBI21suRQNy9RN\/GtqUTiQeajZPBQlXgwQJ4NhUFBzYBabGacB7MRJWz2Lzl4tI1IZCWO8dwnzBhAp0+fZquvfZaGjVqFMXFxamQr1OnTqXy5cvTgAEDKCkpKRQXXjqReKBZXsu2VAAebIHRUiXgwBJ8thUuzIM0gZeuCYET96NHj6pY7hwWlmO4v\/vuu7Rjxw4Vu3327Nnqb6VKlaKMjAzq3bs3tWzZUk1m6URK779tTxSPKwIPHhMQgLXsPYL29KCotcBX5ZKmf6ka4Lvwfk5BWMulLnLQ9IAkFvcRI0bQ0KFDqWnTprRkyRI6cuQIVatWjXiYqampaqSLFi1SP7XP0omU3v+ATD\/xXxKDwAPWgj9YLI4HI2FjvRxJEOZRoMSdJ8Pq1auVuPO2PFvm06ZNozfffJMSEhKoQ4cOar5kZ2dTTk4Opaenhyx3LycS2gYCQAAIRBMCxxOfpHNxzaj63\/v7dtg7d+70bd\/0dEy0uB88eJD69+9PmzZtooEDB1KvXr1o7NixxJHl6tWrR1lZWbRu3ToqW7YstWvXjhITE4sUdz1AIQ8QAAJAAAjYhwCHjeVrckjOICBa3AtDsn79elqxYoUS95iYGMrLy1Ni36pVK6pUqVKBbXk+WJecnOwMqqgVCAABIAAEgICHCARK3HNzc+m5555TB+Zq166trHb+161bN1qwYAGNHDlSQT1+\/HhKS0uj+Hh8a\/Rw7qFpIAAEgAAQcAiBQIk7H5r7+OOP6cUXX1Tv3PlQxPDhw9XJeb4KN3PmTHWwbvDgweoEPZ+cRwICQAAIAAEgEDQEAiXuQSMH4wECQAAIAAEgYAYBiLsZ1FAGCAABIAAEgICPEYC4+5gcdA0IAAEgAASAgBkEIO5mUEMZIAAEgAAQAAI+RgDi7mNy0DUgAASAABAAAmYQgLibQQ1lgAAQAAJAAAj4GAGIu4\/JQdeAABAAAkAACJhBAOJuBjWUAQJAAAgAASDgYwQg7j4mB10DAkAACAABIGAGAYi7GdRQBggAASAABICAjxGAuPuYHHQNCAABIAAEgIAZBCDuZlBDGSAABIAAEAACPkYA4u5jctA1IAAEgAAQAAJmEIC4m0ENZYAAEAACQAAI+BgBiLuPyUHXgAAQAAJAAAiYQQDibgY1lAECQAAIAAEg4GMEIO4+JgddAwJAAAgAASBgBgGIuxnUUAYIAAEgAAQsI3BoyRhTdZzLyzVVzkihWoPnGsnuu7wQdyK6+uqrfUcMOgQEgAAQsAOBzlccpyvLn1NVXVnubKjK2uXyf6d+f+nv\/P\/w39vRfuE69p6JsVztz6et1xGpE8n\/PBkpi6\/\/DnG\/JO47d+70NVEldY6\/nEjuv1jgC3UcPHjPZNA5OJb1hgL51OZ16ufZSxbsuX25VJw1G1OrUYiYmN\/8+v+y4b8vkKehyl+lfV\/ThErnQXr\/mTiIO8Td9AJGwYIIBOGBIJ1TyRywOLNIs2CzePPPokRbE2sWak2gY5u3syTGdvMumQfGQnr\/Ie6XZrR0Ir\/\/\/ntq3Lix3esT9RlEADwYBMyB7JI4YCtcE\/FTm7N+ta4vWdEVmrdXv5P47lcSD0VNQ+maAHGHuDvweI3eKqU\/0ILAnB850Czyk5vX0cktWaQJuWaBs4j7zfK2Ohf8yIORMUHcjaDl47zSiZS+kHw8NQx1DTwYgsuRzH7goCQx14Sct9NjL1nmjgDhcaV+4MEKBNI1AZY7LHcr8x9lCyEg\/YEWBEK94oAF\/eRmtsrXkXbojS1zFnOJ2+pW54JXPFjtt1Ye4m4Xkh7XI51I6QvJY\/ptax482Aal6Yrc5KCwoLOY8yG3uhkfme5\/UAq6yYMTmEnXBFjusNydWBdRW6f0B1oQiHODAxb1Yx\/No0NvZ1A0W+clzRc3eHByvkLcnUTXxbqlEyl9IblItaNNgQdH4dVVuVMcaIKuHYhjUW8w\/XtdfYrGTE7x4BaW0jUhkJb74sWLafjw4aE5MHHiREpJSaH333+fpk6dSuXLl6cBAwZQUlISlS5dWuWTTqT0heTWgnW6HfDgNMKR67ebg8JWOjt2qdC8XaAPw0VGOXIOu3mI3KK9OaRrQiDFfebMmdSxY0e69tprQ2xv3bqVZs+eTWPGjKFSpUpRRkYG9e7dm1q2bAlxt3dNRHVt0h9oQSDPLg5Y1Pe9cr+6tsZWeo2U0b5yEuN3ruziwatxQty9Qr6Yds+cOUMvv\/wy9ezZk+rUqRPKxdb8xYsXKTU1Vf1u0aJF6qf2WTqR0heSz6aR6e6AB9PQ2VbQCgfaAbljWfOUZ7hoPeluBxlWeLCjfat1SNeEwFnuJ06cUNb5rl27aPv27dSpUycaOnQozZs3jxISEqhDhw6K8+zsbMrJyaH09HRY7lZXAcqHEJD+QAsClWY54Ohk2hU23nqvclsfZbEjmUPALA\/mWrO\/FMTdfkwt1Xj8+HFatWqV2pavUqUKrVy5kj755BOqWLEitWvXjhITE4sVd63hNWvWWOqDF4X5y0z9+vW9aBpthiEAHryfDoY4OLSLLn7+N6JVU4lq1CdqfQ+Van1P\/v+RLCFgiAdLLdlbmLVDS9KDcQU6cExeXh6NHTuWWrVqRZUqVSqwLc8H65KTk2G527s2oro26dZKEMjTw0FRh+T4nTqSfQjo4cG+1uyvCZa7\/ZhaqpEPzvGBOhb0qlWrUlZWlrLcu3XrRgsWLKCRI0eq+sePH09paWkUHx8PcbeEOAqHIyD9gRYENkviAKLuHsPS1wLE3b25oqslPjS3YsUKmjJlCpUrV06dmB81ahTFxcWpq3As\/Jxn8ODB1KVLF3VynpN0IqUvJF3kCsgEHrwnqSgOIOru8yJ9LUjXBGY80Nvyeqe0dCKlLyS9PPk9H3jwnqFwDljUtYNycDrjLjfS14J0TfCNuPNBOL7Gxokt7sqVK7s6E6UTKX0huUq2g42BBwfB1Vk1c3BV5VIFRJ1Pv+Oduk4AbcomfS1I1wRPxf3ChQvqOtq0adNo\/\/796h05p1OnTlG1atXU1nnbtm1DXuRsmnNFViOdSOkLyUlu3awbPLiJdsG2tDvqef98lWjHeuVwpkr7PvAk5xEl0teCdE3wTNxPnjxJr7\/+urq+dfvtt19mqfPf+Rrbl19+qe6tO52kEyl9ITnNr1v1gwe3kP61ncLv0881\/B1dnf6O+x1BiwUQkL4WpGuCZ+LOVjv\/i4mJKXFJcB7N\/7uTa0c6kdIXkpPculk3eHAPbXY4w57kNPew2tY7OHCPg5Jaks6DdE3wTNy1SXHo0CF65plnlEe58BQbG6u8y91zzz2h7Xonp6x0IqUvJCe5dbNu8OAs2izopzavU57ktFCrhbfewYGzHOitXToP0jXBc3E\/f\/68un\/ODmXYNSxfU\/v73\/+u5g9v2W\/cuJGefvppvfPJdD7pREpfSKaJ81lB8GA\/IYUFPeY3jdS7dLbUi0rgwH4OzNQonQfpmuC5uLPlPmvWLBoyZIjyIMeJ\/cPzIbs\/\/\/nPyif8iBEjzMwtQ2WkEyl9IRkiy8eZwYN1csKDt2hb7pEEPbxVcGCdAztqkM6DdE3wXNz5Ctzzzz+v4qtrvtF3796t4q4PGjRIifvo0c67hZROpPSFZMfDxA91gAdzLIS\/P+catC332ObtDIdZBQfmOLC7lHQepGuC5+LOHdiwYYMS8LJly6rDc8eOHVPx1vfu3au26e+99167591l9UknUvpCcpxglxoADyUDzZb42bxc9d48\/2eWKmBFzAu3CA5cmuwRmpHOg3RN8IW4s4D\/8MMPxJOhYcOG6v17vXr16Ny5cxFP0+udxtwGu5\/lHQGun3cKkpKSQifxpRMpfSHp5dHv+cADEW+rcyz0cBHnz\/x7Tch5m71srUbElnn+z\/a2UQsObIPSUkXSeZCuCb4Q96VLl9Ly5cuJD9c9+eST9OKLL1Lfvn1D4VktzbBLhTmgzOzZs9WdefYnzzsDvXv3ppYtW6oc0omUvpDs4NgPdUQTD9pBN8adhTxcwDUR558VmrdXIs6puENwdnIXTRzYiZvddUnnQbomeC7uR48eVUFeWGjZqubteXZDywfqWOg1r3VWJ97ixYvVFn9qaqqqatGiReqn9lk6kdIXklV+\/VJeKg\/sf12zrDWx5p+FBTscZ95K18Sbf7op4CXxLZUDv8xhu\/ohnQfpmuC5uGun5Xv16qUO1rG4s295\/v+wYcOoZs2atsy1zMxMSkhIUNftOGVnZyvXt+np6eqzdCKlLyRbSLapErZIw9O5fT\/orvnQ4UNUo3qNiPnDhbRwZraCi0sstsX+rYRyJXVIE2nOw9vlnHirnFOtwXMjjsVvGbAW\/MGIdB6ka4Ln4s7W9Jw5c4jdzX799df08MMP05o1a4id2Dz00EO2vXNncWc\/9YmJiUWK+85780O\/IgGBcAR+iqntCCB7yhRf708xV5bY5p5i+qTV+VPTu6hu1YKeH+tW+fXzg7dUd2RMfqmUHWJpN2\/80qdo7IdUHjp27Biia+fOnaKp8zzk6+nTp+nDDz8kfvfOQWO6du2qPNNVqFDBNmALb8PzZz5Yl5ycrNqQ\/i1N+rdk24j2uCKnechcWbTl\/uPBkwVG\/t+Dpwp8\/vHQr59\/LPS3oiBrUDNW\/bpBjfyfV9WMpQY189djeud8q96vyWkO\/Dpuv\/VLOg\/SNYHngyfizj7jjxw5ot6DF5X40BtHhrPLrzwfqJs\/fz6NHDlSNTd+\/HhKS0uj+Ph4iLvfngqC+yP9gaZBr32J0L408JcF\/oJQ1BeD8C8Cba\/JfyXh5ReAoHAgeBmorkvnAeJucgaG+5TnQ3Uc8rVRo3yL4MCBA9S9e3d64IEHbIvrrl2FmzlzpvpCweFku3Tpok7Oc5JOpPSFZHIa+a5YtPGwcMPekODnfHdI8ZG943CIF034eyTUUb9r26Q6JV7j7GuBaOPAd4vgUoek8yBdEzyz3LUJyVvyM2bMUI5q+G47J35X884776j377x17kaSTqT0heQGx260AR7yUWYLn\/\/lXBJ6Fv7Cos+C74SFDw7cmOmR25DOg3RN8FzcDx48SGxN88n4cN\/yfC2OD9TZdVo+0lSUTqT0hRSJHyl\/Bw+RmdK2\/Bdu2BOy+tnC\/2rU7yMX1pEDHOgAyYUs0nmQrgmeizvfaZ8wYQI1btxYbZNzWrt2LX3zzTf01FNPwXLXuQilLySdw\/R9NvBgjCK27nlrn1Pmyu\/VTxZ6K1Y9ODDGgVO5pfMAcbdhZnAUOD69zu5heZue47j369fPNgc2eroonUjpC0kPRxLygAdrLGlirwl9eufG1COhthJ8vQkc6EXK2XzSeZCuCZ5Z7iziHCAmLi6uxBnGB+0i5bFjikonUvpCsoNDP9QBHuxjIfu7w8Rb92zZs7hrQh+pBXAQCSF3\/i6dB+ma4Jm4c1CY9957j7766itKSUmhJk2ahO61s0ObLVu20LvvvkutWrWibt26OT4bpRMpfSE5TrBLDYAHZ4AevHBrSOQjvZsHB85wYLRW6TxI1wTPxF2bKHl5ecR+35ctW6buRXK67rrr6O6771bX4XCgTt+Skr6Q9I3S\/7nAg3Mc8Zb9kIVb1al7tuKLO2kPDpzjwEjN0nmAuBth28d5pRMpfSH5eGoY6hp4MASXqcy8XT9k0VZVdlpq\/GX35sGBKVhtLySdB+ma4LnlbvuMMlmhdCKlLySTtPmuGHhwjxK+UscH7\/jA3Ss98j1NcgIH7nFQUkvSeZCuCRD3S7NTOpHSF5I\/HkfWewEerGNotIaaj31EiU2q03uDb4K4GwXPwfzS14J0TQikuPM7\/OHDh4em7cSJE9WhPb5qx85x2OvdgAEDKCkpKeS7XjqR0heSg88YV6sGD67CrRrjbfqk6V+GBB4cuM9BUS1K50G6JvhC3NnPPId9zc3NVf7kOcjLXXfdZToqHHu847B91157bWjOcZ2zZ8+mMWPGKH\/yGRkZ1Lt3b2rZsqXKI51I6QvJH48j670AD9YxNFMDH7ZjgeeT9ODADIL2l5HOg3RN8Fzc+Urc5MmT1VW4devW0YgRI2jVqlXEwWTMxHNnj3cvv\/wy9ezZk+rUyQ9WwYmteQ4Yk5qaqj4XDgErnUjpC8n+R4s3NYIHb3DnVlngW437lH5XL5ZWPW6PK1vvRiO\/ZelrQbomeC7umm95Dr\/6\/PPP0+jRo9W2uVnf8uztjq1zDj6zfft25e1u6NChNG\/ePEpISKAOHTrkb+VlZ1NOTg6lp6fDcpf\/HPHNCKQ\/0HwDpMmOaAIf\/g7eZFUoZhEB6WsB4m5xAhw\/fpymT5+u7rSzoLO4sxMb\/v+oUaMiuqDlLwf9+\/enTZs20cCBA1UoV7b8eVu+SpUqtHLlSvrkk0+oYsWK1K5dO0pMTCxW3LWhrFmzxuKo3C\/OX2bq16\/vfsNosQAC4MH7CfHPr\/5LIz85TwNvrk4P3uJseFnvR+vfHkhdC6wdWtq5c6d\/AdbRs1IXeb\/aw8RWNG\/NHz58mH73u9\/Rhg0baPz48SEhttI1dpIzduxY5emOo86Fb8vzDkFycjIsdysAo2wBBKRbK0GgU+OAT9G\/N+gmx+PHBwEzJ8YgfS3AcrdpVrAFz9vo7HP+mmuuMe1Png\/O8YE6FvSqVatSVlaWstzZhe2CBQto5MiRqsf85YFfBcTH59+PlU6k9IVk0zTyvBrw4DkFoQN1Sa98ST8eOmVbKFnvRyarB9LXgnRN4NniueXOljpvpfPpdRZf3gp57LHHTPmU502IFStW0JQpU6hcuXLqxDxv73PwGb4Kx8LPeXj7nkPM8sl5iLush4afeyv9geZnbPX2LZwDtt4LO7nRWw\/yWUNA+lqAuFvjn9hinzRpEj344IPqENy2bdvU\/fOXXnqJ\/vKXv0R8526x+VBx6URKX0h28eh1PeDBawYKeqjT7sBje959XqSvBema4Lnlzgfi+P75I488Qm+++Sa1bt2amjZtavq0vNkpLJ1I6QvJLG9+KwcevGekMAe8Pc\/BZg7+9TbvOxdFPZC+FqRrgufizvfSMzMz6ZdffqG9e\/cSe5PjbfovvviCnnrqKXUtzo0knUjpC8kNjt1oAzy4gXLJbRTFAd9\/b9ukegEf9N73NNg9kL4WpGuC5+LOHeC76f\/+97\/pyiuvpFq1atFHH32krq3xgTi3knQipS8kt3h2uh3w4DTCkesvioOFG\/YSx4RnD3YNasZGrgQ5LCMgfS1I1wRfiDt7qTt27Jg66MaJ77mzB7l+\/fpRjRo1LE8yPRVIJ1L6QtLDkYQ84MF7lorjgLfnr6oZC+vdJYqkrwXpmuC5uLOw8wn2f\/zjH8rlbPXq1dUW\/f33369cyMbExLgyFaUTKX0huUKyC42ABxdAjtBEcRzgcJ273EhfC9I1wXNx56Ax7Auer76tXbtW3TsvW7asuhrHAg9x17cgpS8kfaP0fy7w4D1HJXHA1jsnLTys970Nbg+krwWIu8W5GS7uHDjmiiuuUJHa+Coch2WtWbOmxRb0FZdOpPSFpI8l\/+cCD95zVBIHmu\/5V3rEq\/vvSM4hIH0tSNcEzy13fs\/+6quvqkN1t99+O7322mt0\/fXX0zfffKO8yFWuXNm52RdWs3QipS8kV0h2oRHw4ALIJrfltWJ8sC5nx2F4rnOYKulrQbomeC7u3IELFy6o9+wc3GXjxo0qnvuf\/vQndXLerSSdSOkLyS2enW4HPDiNcOT69XDAnutgvUfG0koOPTxYqd\/pstI1wRfizofq2FLnwDFa4vvtLVq0iHjPnctOmzaNunbtqlzNcmL3tezG9sCBA\/SHP\/yBhg0bpiLEsftZjjbHdfOWP3vCK126tCojnUjpC8nphepW\/eDBLaSLb0cPB2y98\/U4OLZxji89PDjXuvWapWuC5+LOgWI4yAuflGcwtVStWjVKSUkpcVt+\/fr1tHDhQvr6669p1qxZStzZnW1GRgb17duXmjdvTkuWLKEjR44okWdPeBzrnf3Jcx72Zc\/v9yHu1hcCashHQPoDLQg86uWArff0zo0pvXOjIAzbd2PQy4PvOn6pQxB3i8yw+1m+CsfWNYdkNZLYD\/3u3buVgD\/++ONK3Dmy3Ny5c5XlzvV9++23NGfOHCX0fAo\/POQrt6V9lk6k9IVkhHc\/5wUP3rOjl4PMlbmUufJ7WO8OUaaXB4eat1ytdE3wheU+Y8YMuvfee6levXqGCWGHN3zwrk+fPkrc2Zr\/4IMP6Omnn1bX6LR47nwK\/9Zbb6UOHTqoNjiGfE5ODqWnp8NyN4w6ChSHgPQHWhCYNcIBrHfnGDfCg3O9MF8zxN0kdrx9rm2Z79+\/X8Vdv+2229RVOE5FbcuzD3o+Wc9b6a+\/\/rq6JleUuPN9+eHDh6vt93Bx59P4iYmJxYq7NpQ1a9aYHJV3xTiiXv369b3rAFpWCIAH7yeCEQ6Wbz1OGav30\/I+9aluVXccZnmPkDs9MMKDOz3S10rHjh1DGfn8luTkSTx3PiHP78I1l7OFAWRhZoHXDrwVB3Bhcde24dly17blFyxYoKx6tuTDt+X5YF1ycjIsd8mz12d9l26t+AxOU90xygHc0pqCOWIhozxErNDlDLDcLQDOUeCmT5+ugsawRW3Gl3xhcecdAT6g179\/fxU6lncH+NBeQkICzZ8\/X72L58Rb+WlpacojHifpREpfSBamka+Kggfv6TDKAdzSOsOZUR6c6YX5WqVrAo\/cE8udBffZZ59V8dtvvvlmdU2Nt+effPJJQy5nC4s7D4i3Uvg0PH95aN++vYoVz1Y8t8GH93i3YPDgwdSlSxe1dQ9xN78AULIgAtIfaEHg0wwHcEtrP\/NmeLC\/F+ZrhLibxK7wKXn+PGnSJBoxYoSroV617ksnUvpCMjmNfFcMPHhPiRkONLe07w26iRKvqe79IALQAzM8+GnY0jXBM8udxZzvnbNVXaFCBeLPfAd99OjRrvmTD59I0omUvpD8tKit9AU8WEHPnrJWOGg17lO4pbWHBvE+H6RrAsT90kSWTqSVB5pNaxnVwImNL+aAlbXA4t6gRiyixtnApBUebGjechXSNcFTcedDb5s2bSqShPDrbpZZ0lGBdCKlLyQdFInIAh68p8kKBzhcZx9\/Vniwrxfma5KuCZ6Ju3nInSkpnUjpC8kZVt2vFTy4j3nhFq1yoPmd\/2rU76lBzVjvByS0B1Z58HrY0jUB4o5tea\/XUKDal\/5ACwIZdnCA0\/PWZ4IdPFjvhfkaIO7msfNVSelESl9IvpoMFjoDHiyAZ1NRuziAa1prhNjFg7VemC8tXRNgucNyNz\/7UfIyBKQ\/0IJAqV0c4P27tdlgFw\/WemG+NMTdPHa+KimdSOkLyVeTwUJnwIMF8GwqaicHWuQ43H83To6dPBhv3XoJ6ZoAyx2Wu\/VVgBpCCEh\/oAWBSrs50A7YQeCNzQ67eTDWuvXcEHfrGFqq4dy5czRt2jTq2rWrCg7DafHixSoqnJYmTpxIKSkpyv3s1KlTiQPGDBgwgJKSkkKBaaQTKX0hWZoEPioMHrwnwykO+B38Kz3iqUdCbe8HKaAHTvHg1tCla4Joy51jty9cuJC+\/vprmjVrVkjc2X88h+3TxJ4HuXXrVuURj73gsT959j3fu3dvFT6Wk3QipS8ktxas0+2AB6cRjly\/UxxoW\/TpnRtTeudGkTsS5Tmc4sEtWKVrgmhx37ZtG+3evVtFfnv88ceVmJ85c4Zefvll6tmzJ9WpUyc0D9ia54Ax4SFf+Y\/aZ+lESl9Ibi1Yp9sBD04jHLl+JzlYuGEv8TY9W+9sxSMVj4CTPLiBu3RNEC3u3PnCUeFOnDihrPNdu3bR9u3bqVOnTjR06FCaN2+eCvvaoUMHNS+ys7MpJyeH0tPTYbm7sVKipA3pD7Qg0OQ0B3yKfsiirQqqaanxCDRTzKRxmgen5yrE3WmEw+rPzMykV199VW2lv\/766yrATFHx3FetWqW25atUqUIrV66kTz75hCpWrEjt2rVTceOLE3etqTVr1rg4Knua4i8z9evXt6cy1GIaAfBgGjrbCrrFwax\/HaZXPztMv6sXSxm3x1HdqjG2jSEIFbnFg91YsXZoicOHS06exHO3C7Ci4rmH152Xl0djx46lVq1aqZju4dvyfLAuOTkZlrtdZKAe8ZGwgkChmxYjh4pNmv4l8U+8iy84e9zkwYl5C8vdCVQN1FlY3PngHB+oY0GvWrUqZWVlKcu9W6LpMq8AAAxhSURBVLdutGDBAho5cqSqffz48ZSWlkbx8fnvzaQTKX0hGaDc11nBg\/f0eMGBtlWviTy\/k492v\/Re8GDn7JOuCYxFoCx3PjS3YsUKmjJlCpUrV04dshs1ahTFxcWpq3As\/Jxn8ODB1KVLF3VyHuJu55KI7rqkP9CCwJ6XHPCBu+zvDhH\/ZHHvkVAnak\/We8mDHfMY4m4Hij6oQzqR0heSD6aALV0AD7bAaKkSP3DAFjwLfObK79VYNKFv26R61BzA8wMPViaSdE0Qb7lbIS+8rHQipS8ku3j0uh7w4DUD5LtzD0UJPYt8g5oVKMhiL30tSNcEiPulZ5F0IqUvJO8lwZ4egAd7cLRSi5850IQ+57tDlL3jcMiqb1AjltpeUyNQW\/h+5kHP\/JKuCRB3iLueeY48OhGQ\/kDTOUxfZ5PGAXu+4xQu+NpWvib6\/FmaVzxpPBSe1BB3Xy9z\/Z2TTqT0\/utnyt85wYP3\/EjngK37nB2H1fW6Hw+epP8ePBWy8jV0tZP4LP5X1YxVW\/zaFwK\/+L6XzoP0\/sNyD4jlHoSJ6L0sWO8BeLCOodUaooEDzdrXxJ8x+\/EQfxk4VSx84Vfz+EsBJ\/5ikP+lIP\/LQeEvD1a+KEjnQXr\/Ie5h4m71oYLyQAAIAAG\/InDqurtCXbtQ8Qr1\/wsV40r8nVNjKf3LfluqLv3LAVvqKa6SffMfcrR+pysXfc\/daXBQPxAAAkAACDiHgLYLYaYF3rlwMkkPDgRxd3J2oG4gAASAABAAAh4gAHH3AHQ3mmS\/+s8++yzt2bOH\/vKXv9D\/\/M\/\/uNEs2ghDYN++fYqD3Nxc6tevH919990hr4gAyn0EOPRzw4YNqU2bNu43HuUtHj9+nF544QXauHEj9e\/fX60FJGcRgLg7i69ntS9btowaNGhAzZo1o7lz59L9999PlStX9qw\/0djwP\/7xDxWtr3nz5vTKK69QSkoK1alTJxqh8HzM\/GX38ccfp0GDBkHcPWBj3bp1dPToUbr99ttp1qxZ6nnE8T+QnEMA4u4ctp7WzGFx2X8+i0n4\/z3tVJQ1vmPHDqpVq5YKOczizgGMrrrqqihDwR\/DXb58udrFatGiBcTdA0rmzZundg853seJEyeoQoUKVLp0aQ96Ej1NQtwDyPX58+dpzpw5dM8996i499OmTaPOnTurhYXkLgLnzp0j3g7ev3+\/ClgUE4O43+4yQMRW+5o1a9QuCuOPbXm3GSBlrR8+fJg4cmf37t0LBO5yvzfR0SLEXRjPvDhWr15NQ4cOVT1n8eBvxW+99ZayEJ9++mlKSEhQW\/F33nmnshxZ6LX\/CxuuL7urlwO2UCZNmkQ33ngjJScnQ9htZlMvD\/yKijk4cCD\/6hTE3T4i9HIwdepUuu2229QrKv4\/v6LCLpZ9PBRVE8TdWXxtq\/3IkSO0ZMkSWrp0KbVv357S09NV3fwui\/+NGDFCWYccq57\/tmHDBvXOna11jmXft29ftRWGZB4Boxxs2rRJnXNo166d+UZR8jIEjPAwbNgw+tvf\/ka7d++mgwcPUqVKldQXYLbikcwjYIQDfh598cUXxI5hWNxnzJihjA2Iu3n89ZSEuOtByQd5+DDKli1baPPmzUrENXHPzMyktm3bUmJioopVP3HiRLr11lupadOm6qT23r17cVreJv6McpCTk0Mff\/wxlStXTh0eysjIoMaNG9vUm+itxigPvDY4rV+\/Hpa7TdPGKAd8sJefVT\/88AP16tWL7rrrLtwcsYmL4qqBuDsMsN3V8wOKLXUW95MnT9LYsWPVYrnhhhtUU4sWLVKL5r777rO7adR3CQFw4I+pAB685wEceM8BxN2\/HBjqWeHFNG7cOOrTp4+y1CHuhqA0nRkcmIbO1oLgwVY4TVUGDkzB5kohWO6uwGxfI+GLKXwbXtuW562vjh07qkN1SM4gAA6cwdVoreDBKGL25wcH9mNqV40Qd7uQdKme8MXETfIW\/eeff058cIjfxbO485Z97dq1XepR9DUDDvzBOXjwngdw4D0H2Jb3LweGelZ4MWlX4fguNZ8E5lPzsNoNQWo4MzgwDJkjBcCDI7AaqhQcGILL1cyw3F2FG40BASAABIAAEHAeAYi78xijBSAABIAAEAACriIAcXcVbjQGBIAAEAACQMB5BCDuzmOMFoAAEAACQAAIuIoAxN1VuNEYEAACQAAIAAHnEYC4O48xWgACQAAIAAEg4CoCEHdX4UZjQAAIAAEgAAScRwDi7jzGaAEIAAEgAASAgKsIQNxdhRuNAQEgAASAABBwHgGIu\/MYowUgAASAABAAAq4iAHF3FW40BgQKIsCxAD788EMV711LnTt3poceekg0VP\/5z3\/ou+++Ix7LU089RT169KA2bdqExsTjbtSoUZGhiXfv3k1r1qyhnj17UkxMjGgc0Hkg4BUCEHevkEe7QIBIBfpp165dAeELB4Yj\/\/G\/0qVLi8Hr5MmTNH36dEpLS6PKlSsbFnce6KJFi6hZs2Z00003iRk3OgoE\/IQAxN1PbKAvUYdAceLOgYC2bNlCubm59Oijj1K1atVo\/PjxtG\/fPmXxsjVcr1492rBhA40ePZrKly+vAgZVqVKFevXqRWPGjFG\/r1mzJmnBPZ588klasWIFvfbaa8QBh7p06UJ9+\/alTZs20YIFC1QdX3\/9NbVu3VoFIIqNjaX58+croT19+jQNGjSIatWqperjutiq5r9zYiHX0saNG+mLL76gAQMGEAt9SZb7oUOHaOXKlaooRzW8+eabacKECfTjjz\/S22+\/TU888QSVK1cu6uYFBgwErCIAcbeKIMoDAQsIsLi\/+uqroRrq1q1Lc+fOVeKYnZ1NEydOpLNnz9KUKVOUuHIoX97yXrp0KfXr148mT56sQvxeeeWVSmj37t2rRLUoce\/UqZMKETxkyBAqU6aMyh8XF0dXXHGFqufll19Wn8eNG0ddu3alX375ReXn+s+cOUPPPvus2l7n\/rH4s1U+duxY1d61114bGsO0adPo+uuvpw4dOoTEffny5ZehxGO777771O\/5S8vIkSNV6OIbbriBjh49qup+\/PHHqU6dOhYQRlEgEJ0IQNyjk3eM2icIlGS5cxdZ\/FjM77\/\/fjpw4ECo1y1btqTBgwfT559\/rsS3VKlS9O233yoruDjLna1uFu7wNHDgQPVaQBNx\/hvvGjRs2JBycnLUtjiLNCftFQFvud944430m9\/8Rln1LPRs9WspfEyRLHceH+8KsJC3atWKunfvrsbC5Xinok+fPgW+OPiENnQDCPgeAYi77ylCB4OMgB5x\/\/LLL9W2OVvOFSpUoAsXLiirmkV\/7dq1NHz4cCWI\/Hn16tVqi7woy71SpUpqW12zltkaP3\/+vNqWL0rc+Xdt27alxMRERQFb02XLllWvC7gdrot3DNjKD09GxD0lJYXeeecd4kN0vKOgHaCDuAd51mNsbiAAcXcDZbQBBIpBQI+45+XlqS1r3qJu2rQpZWVlKSG\/44476Pnnn6dnnnmmwLb8ww8\/HNrS5m38OXPmqPfZLNTLli1Tws8izdvnt9xyixLUosT98OHDtHnzZrVVzuW5r7xLwNvxGRkZdPz4cfWT2whP3B6fCwjfli\/utDyfG+Bt\/ueee059WdAS183b9iz42JbH8gECxhGAuBvHDCWAgG0I6BF3boyta95SP3HihNqmHjVqlHo\/zlfGWBj5IJ12oI4F8e9\/\/zvNmDFDHai7+uqr1Xt1FuklS5bQG2+8ofqfmpqqrHze2i9K3Hnrn7fg2UrnnYHHHnuMOnbsqP6vfWH4v\/\/7P\/U5PPGBus8++0xd5zt16lSxB+r4fAG3zecLeCyc6tevr76YHDlyhN566y31hSZ8y9824FEREAg4AhD3gBOM4UUPAtu3b1fv3FncnUx80p4P4PEBvaKuqvGWOh8A5AN\/ha16vf3id\/ls\/YffjddbFvmAABAggrhjFgCBgCDghrgfPHhQWdN8op2\/RBRnVW\/btk0d8ONXB4Ut+0hw8\/t3\/pLCuwpwYhMJLfwdCBSNAMQdMwMIAAEgAASAQMAQgLgHjFAMBwgAASAABIAAxB1zAAgAASAABIBAwBCAuAeMUAwHCAABIAAEgADEHXMACAABIAAEgEDAEIC4B4xQDAcIAAEgAASAAMQdcwAIAAEgAASAQMAQgLgHjFAMBwgAASAABIAAxB1zAAgAASAABIBAwBCAuAeMUAwHCAABIAAEgMD\/AwpSEwhynu3KAAAAAElFTkSuQmCC","height":303,"width":503}}
%---
%[output:23d5f679]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:18bcb055]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"5","name":"Afht0","rows":2,"type":"double","value":[["0","0.000010000000000"],["-1.421223033756867","-0.000188495559215"]]}}
%---
%[output:19671167]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.018661060362323"],["3.911916400415777"]]}}
%---
%[output:3bfc9561]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-17.765287921960841","0.997643805509808"]]}}
%---
%[output:501fc6e6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.215470420991426","41.436377683116810"]]}}
%---
%[output:7c3a646c]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"5","name":"Afht1","rows":2,"type":"double","value":[["0","0.000010000000000"],["-1.421223033756867","-0.000188495559215"]]}}
%---
%[output:4b7a2ae7]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.018661060362323"],["3.911916400415777"]]}}
%---
%[output:62b55245]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-17.765287921960841","0.997643805509808"]]}}
%---
%[output:67ae9a02]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.215470420991426","41.436377683116810"]]}}
%---
%[output:828549da]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: GM355L6-250kW\nIM Normalization Voltage Factor: 326.6 V | IM Normalization Current Factor: 610.9 A\nRotor Resistance: 0.00434 Ohm\nMagnetization Inductance: 0.00533 H\n---------------------------\n","truncated":false}}
%---
%[output:00d598ff]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00756 H\nPer-System Quadrature Axis Inductance: 0.00624 H\n---------------------------\n","truncated":false}}
%---
%[output:6ca5bf04]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:6d3a4ac1]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:7f01d332]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:78e13a6d]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714299e+02"}}
%---
%[output:2d494d81]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -3.927176478925923e+03"}}
%---
%[output:04da29c6]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAfcAAAEvCAYAAABYGIhlAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQncTmX6xy\/GnvVFyTYkxChkSWWmJsowKTRpqDGiUrYWW6lUtqLQIrRINSMtKpFpKtqGSlkSTYaShMiSfY\/\/53f7n7fznvc8z3PO9T7Pec9znt\/5fPqE977uc873ut77d657LXD8+PHjwosESIAESIAESCAyBApQ3CPjS74ICZAACZAACRgCFHcGAgmQAAmQAAlEjADFPWIO5euQAAmQAAmQAMWdMUACJEACJEACESNAcY+YQ\/k6JEACJEACJEBxZwxEjsCOHTukZ8+esnXrVpk2bZrUrl07Ze+4Zs0aufbaa6VixYoydepUycrKStm97BUH+Y5eXujAgQNy++23y5IlS1LO3Mvz5LXMmDFj5IknnpDKlSur3uell16SO+64Q3r16iVDhgzJ6+Mk1d6KnRYtWoTu2ZL6ohleGcU9wwMgiq8fpPA5xb148eJG5ObMmSMvvPCCoAFNxeV8x6pVqwZy31jvYomZ13fWiJ\/GRsP+008\/la5duxrTKIo73st6R6\/+0nCkTf4SoLjnL3\/ePWIErAw2k8Td+tCoXr26PPDAA4IPnESXRqg1Nomew+3nlvC1b9\/e8\/s46wnqWTXvBxsrTtevXx9oj5P2eWnnnwDF3T+zjLGwC5X10vZuRitrbdKkiVx++eVy3XXXmWLORtFq\/JcvX54rG4pVR8OGDeM2Om7PZmUhzqy2fPnyppseF7pIBw4caJ4Rf\/ZSD+ysLndn3fiZ1S0\/ceJEGTt2rMna3XhZ\/+bWlW9nZL2H1TVs2dm52p9j8uTJ8vTTT7ve17rXpk2bcrHHPzh9g3reeOONHN3r8RihDksM77\/\/frnqqqvMfeLd1xI+670sX9t7PayfWXXGssEwiPNnibrCnc+Ge1nMnXXFy9ztGb4z7u3ijp+hix+X89mcPrb\/3KoDDNatW2f8aw0z2e1gU6NGDTMMYPdBIi5+e1sypuGLyItS3CPiyGS\/hluD7hQst0bSKUQbNmww4meJi\/Vzq9G0xNH5c7ePBMs23rOhka5Tp06OMXdL3K2PC6uR7d+\/f3ZXtpOfvZ5ki7tbdm\/\/yEH2++ijj2YLgv3ZLIFHHda8gljifv3115sy9vdGXRZ7e1e+8\/2tMm7srLLOjxDnx1Ws+y5dutQIkVPcn3rqKdd3hmDh8mMTS+DjxSye\/\/vvv89xn1ji7hR2Z9zPnj07Rz12vrE+WJy\/X7E+NJz83Op2fjQ468bfrXdI9DGU7LaF9QVDgOIeDOe0u4vVsMTKFpFB2IXZrWHHv3344YemwXbLSPBvnTp1yhZ\/q45Ek9Tcuk3tmZIlataEOrtA2Z\/Daz1exR3Zvdcxd3tWhmzX\/vff\/va3ZszX3nvh\/CBwfsC4jblb72evx2r0ITBu97GewxI1S0jscWDnNnjwYOnbt2+OyYtuXO33tb+v5Q+3ngtnPZZgWjZWnMA\/VkYbb76FnaE9w3XeBx8l4B+rW97t48x5X4ubnb09Ru0fltazOOP+3XffNR8I9g8MN072DxbUdfbZZ5vfqURcKO5p1yz7emCKuy9cmVPYaoydE27sImQ1IuiWt4+1Wrb33XefLF68OEd3sZ2gXdzts82tBswuqna7WM9mlYnVLe+cPe+1nlSIuz1Tv+eeewSsrJnmljDYBciZaTk\/YBJNqHNmmvZs2H4fJ7vXXnvNNZvG80D83MTd2dWPsrHiyC1rdNpbIusU91jZM+7nlnHHiivnTP\/t27fHFXdnL4vbHAO3MfdYYursibI+CCxxjzUU5vY7Z\/9oc2ut7Fwo7tFuzynu0fav+u2SIe7IpiAO8SaXxRt\/jrK42wVl6NChMnr0aLE+kiwRy6u4OzM6ZMz2DBp8neO0scTd+SyxPqTsyw7dRN7ZJW0Jl13gLDF3ZtCxxN3rxLewibvlC0vM8VFhX1aZV3FPxIVj7urmMS0MKe5p4abgHzKv3fJWlmxlfvaGxi4wVvbvJ3N3y4rsdV588cWuY+7OzN1vPRAuK9txzhmwnt9rtzw8ar8\/hi4s4XPrTtd0yzvfzy62frvlE3Uv29e3u8WO1y52exxYPo2Vuds\/XrwM6SSrW96t6z9Wl7rbMBD+zdnzgtiyuMXL3P10y2MeSywuVpxytnzwbWtQd6S4B0U6ze6T6gl1sbIVzH5O1C3vlhECr1Un\/mzfxMYac3eKe6J68CyxJiZ5EXc8R7zJSnZxcnYjx7qv24Q69JA4J8fZZ1C7hR7E3foIck58s3drx5pQZ39eZy9PvElrbpPJ4Le77rpLbr755lwTL63uf3RB2yepWb6ONwnPmrlvf\/9EE+qwL4GXpXDOyW7WPWL1TMQaVnFjH0\/cUU+s2MDPrPvHKuP8gOQ69zRrmH08LsXdB6xMK+p1KRyyLWQimFhlb4ytsUiniNqzQE23PO4Rr06vY+6J6nH7uXOpGMo4d6izN\/yJZiJbjbBbOa9L4azJZM77OlcD2MfZnR8JlshgOR8E0\/4h5IwD54eI29itm4i6jR0ju3SKmRVDPXr0kD59+piYck7gdOtJcApsrN\/XeEvh7CLstVvb7b5extztcwbwPg899JBZpglf2GfuJ4oN\/ByXvfcHf3d+gFjCzh3qMqMlp7hnhp9T8paJZrWn5KasNGkE3Gabuw0JJLphop6WRPb8uXcCbkMLbl313mtkyagSoLhH1bMBvBfFPQDIKbxFvKGXRFmr87E4OSuFjnJUHWtIQLtVbnBPzjsFSYDiHiTtiN2L4p7+Dk009OL1DaN2cIzX986vcvF27MuvZ+J9w0UgLcV9wYIFgrHBSZMmuZ7C9d1330n37t3lhx9+yKadaOwzXG7h05AACZAACZCAnkDaifvGjRvlhhtukCJFisTce3zZsmWCNaKDBg2SAgUK6OnQkgRIgARIgATSkEBaiTu6\/kaMGGEOSfjkk09k3Lhxrpn73LlzZcuWLYLZtrxIgARIgARIINMIpI24Hz9+XGbOnCnI3Nu0aWNO34ol7s8884wsXLhQVq1aJdu2bZN27dqZE8AqVark6t\/TTjst0\/zO9yUBEiABEohBYP78+VKzZs205pM24r569WrB6VfDhg0zgo3tOt3E\/dixY9lj8VdccYVYf8dOTLHOmoa4r127Nq0dGfTDk5mOOLmRm46Azorxlrnc0kLc9+zZIzhcA93sDRo0MGdFxxJ3N1diYh02hxg5cqTY9762yvIXwP8vAJn5ZwYLciM3HQGdFeMtc7mlhbh72TLScuHevXtl1qxZ0rp16+xu+HXr1gkO58COX9WqVcvlbf4C+P8FIDP\/zCjuOmbkRm56AjrLKLRvaSHuTvfEy9yt9baNGjUyxzZa3fL79u0zAl+oUCGKuy7ec1hhuWG6j0klAYPvKsjNNzJjQG7kpiOgs6K467jl2cop7pagn3feeYLDIjZv3iyPPfaYmYBXrFgxwdh7v379pFy5cq73joIj8wzVZwVsbH0C+\/\/i5EZuOgI6K8abjlsUNCEtM3edu2JbRcGRyWaSqD42GokIuf+c3MhNR0BnxXjTcYuCJlDcOclJFf1sNFTY2L2sw0Zu5KYkoDOjuOu4hc4qCo4MGirFXUec3MhNR0BnxXjTcYuCJjBzZ+auin42GipszEB12MiN3JQEdGYUdx230FlFwZFBQ6W464iTG7npCOisGG86blHQBGbuzNxV0c9GQ4WNGagOG7mRm5KAzoziruMWOqsoODJoqBR3HXFyIzcdAZ0V403HLQqawMydmbsq+tloqLAxA9VhIzdyUxLQmVHcddxCZxUFRwYNleKuI05u5KYjoLNivOm4RUETmLkzc1dFPxsNFTZmoDps5EZuSgI6M4q7jlvorKLgyKChUtx1xMmN3HQEdFaMNx23KGgCM3dm7qroZ6OhwsYMVIeN3MhNSUBnRnHXcQudVRQcGTRUiruOOLmRm46AzorxpuMWBU1g5s7MXRX9bDRU2JiB6rCRG7kpCejMKO46bqGzioIjg4ZKcdcRJzdy0xHQWTHedNyioAnM3Jm5q6KfjYYKGzNQHTZyIzclAZ0ZxV3HLXRWUXBk0FAp7jri5EZuOgI6K8abjlsUNIGZOzN3VfSz0VBhYwaqw0Zu5KYkoDOjuOu4hc4qCo4MGirFXUec3MhNR0BnxXjTcYuCJjBzZ+auin42GipszEB12MiN3JQEdGYUdx230FlFwZFBQ6W464iTG7npCOisGG86blHQBGbuzNxV0c9GQ4WNGagOG7mRm5KAzoziruMWOqsoODJoqBR3HXFyIzcdAZ0V403HLQqawMydmbsq+tloqLAxA9VhIzdyUxLQmVHcddxCZxUFRwYNleKuI05u5KYjoLNivOm4RUETmLkzc1dFPxsNFTZmoDps5EZuSgI6M4q7jlvorKLgyKChUtx1xMmN3HQEdFaMNx23KGgCM3dm7qroZ6OhwsYMVIeN3MhNSUBnRnHXcQudVRQcGTRUiruOOLmRm46AzorxpuMWBU1g5s7MXRX9bDRU2JiB6rCRG7kpCejMKO46bqGzioIjg4ZKcdcRJzdy0xHQWTHedNyioAnM3Jm5q6KfjYYKGzNQHTZyIzclAZ0ZxV3HLXRWUXBk0FAp7jri5EZuOgI6K8abjlvVC\/8mGz74h844JFbM3Jm5q0KRjYYKGzNQHTZyIzclAf9mMz7fLH1mfC07xv\/Rv3GILCjuFHdVOFLcVdgoUjps5EZuSgL+zSju\/pmF1oLd8v5dQ3H3zwwW5EZuOgI6K8abf24Ud\/\/MQmtBcffvGjYa\/plR3HXMyI3c9AT8W1Lc\/TMLrQXF3b9rKO7+mVGkdMzIjdz0BPxbjnl7nYx5+zuOuftHFz4Lirt\/n1Dc\/TOjSOmYkRu56Qn4t6S4+2cWWguKu3\/XUNz9M6NI6ZiRG7npCfi3pLgnYHbs2DHZtWuXHD9+3BfdAgUKSLly5XzZ5LUwxd0\/QYq7f2YUKR0zciM3PQH\/lhT3BMx27NghPXv2lOXLl\/ui27BhQ3n99dd92eS1MMXdP0GKu39mFCkdM3IjNz0B\/5YUdw\/iPmDAABk6dKjUrl3bE+E1a9bI6NGjZdq0aZ7KJ6sQxd0\/SYq7f2YUKR0zciM3PQH\/ltjABjPmuYlNDHZ79+6VWbNmSevWraVSpUqeCG\/evFnmzZsn11xzjafyySpEcfdPkuLunxlFSseM3MhNT8C\/JcU9AbOff\/5ZJkyYIBdeeKE0b95cSpYs6Z9yQBYUd\/+gKe7+mVGkdMzIjdz0BPxbUtwTMNuzZ4\/ccccd8q9\/\/UsKFy4s7du3l7\/85S\/StGlTKVSokH\/iKbSguPuHS3H3z4wipWNGbuSmJ+DfkuLukRky+Pfff19eeeUVWbJkiRQvXly6du0qHTp0MGPxBQsW9FhT6opR3P2zpbj7Z0aR0jEjN3LTE\/Bvednjy2TBtzs55u4HHYT+448\/ln\/+859G6CtWrGjG1y+77DKpXLmyYBlcflwUd\/\/UKe7+mVGkdMzIjdz0BPxbUtz9M8thgQl3EPqHH35YihYtKlOnTpWsrKw81qozp7j750Zx98+MIqVjRm7kpifg35Li7p+ZsTh69KisWLFC5syZI6+++qr5tyuuuEJuu+22fJt0R3H370yKu39mFCkdM3IjNz0B\/5aNRn4i63ccZLe8F3SHDx82go5x97feeksOHjworVq1ki5dupgJdhiHT9W1YMECmThxokyaNClmzwDF3T99irt\/ZhQpHTNyIzc9Af+WWbe9b4y4zj0GO2w\/+\/nnn8vMmTPlnXfeEcyeb9asmXTr1k3+8Ic\/SKlSpfxT92mxceNGueGGG6RIkSJxu\/0p7j7B8lxy\/8D+34IfRTp05EZuOgL+rJCxI3OnuMfhZm0\/iyy9c+fOcumll5oJdEFdBw4ckBEjRkiNGjXkk08+kXHjxsXN3K3nmj9\/flCPmNb32bBhg1StWjWt3yE\/Hp7cdNTJjdx0BLxboTf5aIW6srflYIp7PGyHDh2S3bt3S4UKFQKfBY\/DatBjgMy9TZs2Mnbs2ITivnbtWu9RwJLCTEoXBORGbjoCOivGmz9uC77ZKZdNWkZxj4cNmXt+7S2\/evVqmTx5sgwbNky2bdtm9qtPlLlT3P39ErDR8MfLKk1u5KYjoLNivPnjZs2UZ7d8gm75\/v37m53pqlSp4okwMm3Mosc6eO2Fsf177rlHevToIQ0aNBDrMBqKu5aoux0bDR1PciM3HQGdFePNHzdrpnzB\/dtk25Qr\/RmHrHSB434PXPf4AtiwBpkzxsr8XBjHfeyxx\/yY5CgLMb\/22mtl06ZNuep44YUXpEWLFrn+nRPq\/ONmo+GfGSzIjdx0BHRWjDfv3OyT6YqsXyibZ97l3TiEJVMm7mF5V2buqfEEGw0dV3IjNx0BnRXjzTu3x95fL\/fM+dYYlH5niKxb+Zl34xCWpLiLCDN3\/5HJRsM\/M2buOmbkRm56At4s7Vl79axisvuZqyXd52FFXty9uJbi7oVSzjIUd\/\/MKFI6ZuRGbnoC3ixnfL5ZcBocLmxeEwVNoLgzc\/cW\/Y5SFHcVNo6567CRG7kpCSQ2c2btX9x1LsU9Mbb0KBGFr7SgSVPcdcTJjdx0BHRWjLf43CDsfWd8bY54xfV4l3rSpVklirvfcMPMeewtf+TIEXPMK2a016tXL5CtaOM9K8Xdryc569s\/sRMWbGx15MiN3HQEYltB2F9dtkVGzD2xgVnLWmVldp\/G5s9R0ITAuuWXLVsmN910k9mCFrvXjR8\/XmbMmGH2n8fBLnXq1Em27zzXFwVHen7ZJBVkY6sDSW7kpiOgs2K8uXODsGOcfczb35kCmEQ3u3dj83+Ku49Yw6lwWPPeuHFjcxrcoEGDZOjQoVKzZk15+umnZd26dTJ8+HBzwEt+XBR3\/9TZaPhnxsxdx4zcyE1PILclhB1bzOL\/bsJOcfdB274Vbfny5XNsS4utYkeNGiUTJkyIebCLj1upilLc\/WOjuPtnRpHSMSM3ctMTyGmJveP7vvh1DmHHBDrnFQVNCKRbHgfIDBw4ULAdbeXKlXOI+8qVK+XBBx+Uhx9+WMqVK5csH\/qqJwqO9PXCSShMcddBJDdy0xHQWTHeTnBzdsPHytgtylHQhEDEHTvcTpkyxezz3q9fP9MFf8cdd8hJJ50kDzzwgDmW9eabb5ZChQrpIjiPVlFwZB4R+DZno+EbmTEgN3LTEdBZMd5EXv\/iJ+n5\/FfZADGu\/nDnM+TCOrGTyShoQiDiDqp79+41Xe\/\/+Mc\/5OjRo9mg27VrZw56CfKs9yh2weh+9fVWbDR07MiN3HQEdFaZGm\/I1L\/atFeufmZFDnDOiXOxqFLcfcYbMngsf0PA4TrllFOkVq1aUrBgQZ81Jbd4FByZXCKJa8vURiMxmfglyE1HkNzIzQsBjKmPffu77HXrlg1EfeJf60nL08t6qYZL4TxREpFjx47Jrl27JNYBdOiOL1WqlBQoUMBrlUktR3H3j5ONrX9msCA3ctMR0FllQrwhS1+9ZZ90furLXJD8irpVQRQ0IZBuecyW79mzpyxfvjxmhELcb7zxRunevbsUL15cF8lKqyg4UvnqarNMaDTUcOIYkpuOKrmRm5PAS4s3y\/3\/\/i575rv951pRp7gr4mzJkiXy6KOPmhnzdevWNTVgYxv824ABA6RkyZJmTL5p06Zms5sgL4q7f9psbP0zY+auY0Zu5GatSR8373v5x6ebXIFA0Gfd1EgKFiiQvRmNllwUNCGQzP3AgQNy9913S5cuXaRJkyY5eEP0sVPdiBEj5NtvvzVi\/+STT2p9orKLgiNVL54HI4q7Dh65kZuOgM4qneMN4+cHjx6TR+d\/n2sM3aIBQe97YXW5pH75PAu6nXAUNCEQcbdvYlO7du0cUYrlcaNHj5Zx48bJ9u3bzZ+nTZumi2SlVRQcqXx1tVk6Nxrql06CIbnpIJJb9LlBzLfvPSxTF26MKeagAEGfcGVdqVWxRFIFneKuiDHsJY\/MvWHDhiZ7t2bHY4Ldyy+\/LO+\/\/7489NBD8tlnn8mLL77IzF3BOGgTNrY64uRGbjoCOquwxpvVzf7gO+vkP9\/87Dpubs\/Oz69VVoa0qWn+ydr\/XUfEm1UUEr5AMnfgxDazGEsvUaKEtGzZ0mxYs3jxYvnmm2\/MWDsm0WEy3X333SedOnXy5oEklYqCI5OEwnM1YW00PL9APhUkNx14cktfbpaQIyNftn533KzcEu\/fn15OBl1SIzAxd9KNgiYEJu6Ah+55HPn6n\/\/8xyyNO+ecc6Rt27bmRLjNmzfLTz\/9JGeeeWbg696j4Ejdr77eio2tjh25kZuOgM4qP+IN3euYyf799gMJhdwS8z+fWVHa\/q6C53XoOhreraKgCYGKuxtaBN9zzz1nZsxjOVx+XFFwZNDc8qPRCPodU3E\/ctNRJbdwcbOy8dlf\/iTvfLXdk4iHJSv3QjIKmhCYuKNbHgL+1Ve\/7vFrQf79739vznSnuHsJu3CUYWOr8wO5kZuOgM4qGfGGTHzDzoPywqIfPYu4JeQYK+\/S7FQzTh7EWLmOUm4rirtHktZ57kWLFpXzzz9fpk+fLh07djQ712Ey3e233y5VqlTxWFvyi0XBkcmnEr\/GZDQaQT9zGO5HbjovkFtquVmZ+MQP1suqH\/f5FvHq5YrJpWdVlPqnlgxN17qO2AmrKGhCIJn7zz\/\/LIMGDZLBgwcbaGPGjBEcGNO4cWOZM2eObNiwwexOx+1n8xKOwdqysdXxJjdy0xHQWdnjzRLwN1dslX+v3Cbrfz4Yd5a6845W5n15w5Ol5\/knkrF0ysb9EKS4e6TlXOeONe0Q9osuusgcA\/vII4+Y9e2lS5f2WGNyi0XBkcklkrg2ilRiRm4lyI3cdAS8WVkC\/o9Fm2TR2l2ydute2bT711M4vdRiutDLFZOWtcvJeaeVzRbwqAq5G5MoaEIgmbvVLY+Z8FdddVV2tt63b19ZunSp2cDm8ccfl3LlYp+v6yUotWWi4Ejtu2vtKFI6cuRGbjoCv1pBwI8cOy6Pv79evvlpv68udKsWS6hbnl5O\/tq0UnYWnkkCHs8PUdCEQMQdEFeuXGm63vv162eWwPXu3duc675lyxbp0aOH9OnTx6x9z48rCo4MmhtFSkec3MgtHgEr80aZR977XtZs2e+7+9wu4GhjLzyjogxo\/VvZuPNQJMbDdRHkzyoKmhCYuAMtAu2XX34RTKxbv369zJ4925znju55\/Ft+XVFwZNDsKFI64uRGbpaAP\/7BD\/L1j3vV4g2SVhd6\/colpfcF1cwYuv3McsabLt6ioAmBiPvevXtl1qxZ0rp1a6lU6UQXkHVh85p58+ZJhw4dzMlw+XFFwZFBc2OjoSNObtHmhmVjENyVm\/bK3BVb5YcdB1Xd5vbsG3\/GkrI\/\/a6CNKx6Yi8Qr93njDddvEVBE1Iq7thT\/ssvv5Rt27bJlClTpGvXrrmWvK1bt86MwU+ePFmysrJ0nsijVRQcmUcEvs3ZaPhGZgzILX25WRk3xrlnLf9J1m07kKes2y7SGPu+unkl+eXYr8LtVcDjEWW86eItCpqQUnHHLPmePXvK8uXLYxIuXLiw3HLLLXL99ddzzF0Xh\/lixUZDh53cwsnNyrjXbtsvc77cmqexbvsbWt3m1bOKS6fGJ8vpJ5fwlXnraP1qxXjTEaS4e+QW78hXj1WktFgUHJlSQC6Vs9HQESe34LlZwv3Fhj0y7+vtScm47Vk3lo21O7OitGtQIXvMG1l+MjJvHS2Ke165RUETUpq55xVwUPZRcGRQrKz7UKR0xMktedzsM8tnLt0iH6zeIXJc8txV7hTuFrXKyh9OP7FM1z5ZTfcmwVox3nS8o6AJKRN3TKLDWe04\/S3RVaZMGencuTMn1CUCFaKfs9HQOYPcvHGzsu1lP+yWd\/67XVZv2ilbD4ivHdVi3cnKqJFxX1g3S5rXKGOKZv97VjFvD5kGpRhvOidR3ONww5azw4YNM1vLJrqqVq0qw4cP5yY2iUCF6OdsNHTOyGRuVqaN\/x87flxeXrJF1m\/P+6Q0yxN20a6WVUy6n1tZDh09ni3cYegm10WN3iqT401PjXvL54VdqGyj8JUWNFA2GjriUeRm7x5Hlo1sG0vA\/O5dHo9o5dKFzIRbZNut6pWXjo1ODt34ti4iUmsVxXhLLbETtUdBE1LWLe90wPHjx82yuMcee0w+\/\/xzs488jnrFrnXVq1cPwl8x7xEFRwYNkI2Gjni6cLNn2ch4X\/j8R\/n4m53mpZMp2vZsu1H10tL6jCypUb54drZtTUxLF266qEidFbnp2EZBEwIT9wULFsitt94ql156qRH1ffv2mY1tLMFv0aKFzgtJsIqCI5OAwVcVbDR84couHAZu1nj2J2t3yn++2ZnUrnHrRXOIdrVSckn9CtmCjT\/47SIPAzedx\/PXitx0\/KOgCYGI+4EDB2To0KFy8cUXm6NerQvZPCbdffzxx\/LAAw9I8eInvtiDvqLgyKCZsdHQEU8FN3u3+Ldb98uS73fLR2t+TmmWjTHtciUKm13T7GLtV7S9UkwFN6\/3Tudy5KbzXhQ0IRBxj7fOHUe+4rhXnAzHHep0gZgfVmw0dNS9crN3i1ctV1Se\/\/RHWbzuxMqTZHaL5xDmcsWkbqWT5Ormp0rWSYWzXzBVgu2HoFdufurMhLLkpvMyxd0jt927d5sueZwE16RJkxxWOC3uwQcflIcffpiz5T3yDEMxNho6LyxcvkaOn1RRKpctKlMXbJAVG\/emXLCRZZ9R6SRpXK10juVeYdloxQtJxpsXSrnLkJuOG8XdIzd0vz\/\/\/PPy5ptvyj333CP169eXAgUKCPaVHzlypDRt2tRMrMO\/5ccVBUcGzY2Nxq\/ErTHsHfuOyD8X\/Sirt+xLuWDjBqefUkJuuei32Wu\/zVanEVqjbY9pxpvuN5zcdNyioAmBdMsDL8bdJ02aJE8++aQcOXKp0qz7AAAgAElEQVTEEMfSlmuuucbsLY\/Z8\/l1RcGRQbOLeqNhCfaCb36WT9buku+3HwhEsJvWKGPWZ9uvqAq2n5iNerz5YeGnLLn5ofVr2ShoQiDijnPc9+\/fbwQcXfTffvut4MS4008\/XcqXL59vGbvlyig4UhfCeqt0azQsscbmKTOXbJF1WIedxA1U3MQYa7JxNa9ZRi6sc+LEw82bf5Tm9WtGNsPWR1R8y3SLt1Rx8FsvufkldqJ8FDQhEHHHhLrrrrvOjKlfffXVcu655+bbzHg3V0fBkboQ1lvlZ6NhjRVbgv3x2p3y8bc7zYEguJI94Qx12pd21axQXFqcVlaqlTvRDY7nwZ7jXsaw85Ob3tv5b0luOh+Qm45bFDQhEHE\/duyYYFb8W2+9Ja+88opg3\/m2bdvK3\/\/+d6lbt64ULFhQ54EkWUXBkUlC4bmaVDQa1gzxw78ck+mLfpQl63cn7SAQtxezC3atiiWk1RlZUrbEiVni1vi1F8H2DI3nuftBlaNsKuJN\/TBpZEhuOmdFQRMCEXc7Xkvo33jjDSP0RYoUMWe+I6MvWrSozhN5tIqCI\/OIwLd5okbDmV3jxK7P1u2S9dsPBpJd\/7Z8cal\/6klyZpVS2dl1GA4GScTNtyMyxIDcdI4mNx23KGhC4OJuoT58+LB89tlnZoLdnj17ZOrUqVznrovDQK2s7PqHH36QFbuKy5qf9smaLftT0hXu7A6vfUoJaX1GeSlVrBCXdAXq9fy\/GUVK5wNy03GjuPvkhu74ZcuWyYwZM2T+\/PlSoUIFM1v+sssuk8qVK+fbxLooONKnK1yLW8L95oqt8u+V21KSYdu7wk+rWEKa1yht\/H5+rZxj1lGdIc7GVhep5EZuOgI6qyhoQiCZO0T93nvvldmzZ5uJdFdccYVceeWVoRhvh+uj4MhEIWwJN7rHF323K6mnduUYuz65hFxUN8tsTWrvBk\/22HWi9w3rzylSOs+QG7npCOisoqAJgYg7znZ\/9NFHpU2bNnL22WebcXa\/F+rAiXLTp083phij79evn+uudmgIunfvLug6tq5evXrJkCFDXG8bBUdaL2aJ+IT538u3P+2XBd+eOMnL7+U8G\/vUMkWlW4vK2TPCwfb8hrX9Vpvx5SlSuhAgN3LTEdBZRUETAhF3Hd5frbDD3cSJEwXj9H379hWsm8de9Fhah787d7ZD1\/+7774rgwYN8tTVn86OhJiPn\/e9ICO3HyCSiLmZDV6umGBr0upZxaVLs0rGxFralahbnI1tIsLuPyc3ctMR0Fkx3nTc0lkTrDdOC3HHxjcDBw6U\/v37S4MGDcyz4whZzLZ3O01u7ty5smXLFunRo4cnz6aLIy3xHvv2OnO+dqLLEvArmpwitSqU8LwWO1G9+DkbDS+UcpchN3LTEdBZMd503NJFE+K9XVqIu\/MFcBY8MvcaNWpIt27dcr3fM888IwsXLpRVq1bJtm3bzDGz6JKvVOlEduq84EjrwkS\/sF1LNh6UJxbtFPw\/1lW5dCE5\/7fFpdvZZUwR\/D2V14YNG6Rq1aqpvEUk6yY3nVvJjdx0BLxbtWrVKkfhtWvXejcOYcm0E\/cxY8bIE088Ic2aNTNZe82aNXNgxTp67GGP42Mxcc\/6+\/r162OeGR\/Wr7RbXv6fPP\/pJtewQVb+0nVnSfEiv8mXrUyZEeh+m8mN3HQEdFaMNx23sGqCn7cJRNwxW37WrFnSunXrXNnz5s2bZd68edKhQwcpWbKkp2fHmDtOmHvuuedk8uTJMTNyqzJM\/kK3Pk6gq1079ySwMDkSW6qOffu7XBPhIOZ\/qF1OBl5cI1\/E3OkYNhqeQjVXIXIjNx0BnRXjTcctTJqgewORlIo7Dof58ssvTdf4lClTpGvXrlKlSpUcz4pjX+fMmWNEGtm224UARbY+ePDg7DJbt26VAQMGyN13351DsN0+JHCPoUOHCrL+atWq5bpFWBx52ePLXEV94l\/rmfHyMF1sNHTeIDdy0xHQWTHedNzCogm6pz9hlVJxx4Ex2Fp2+fLlMZ+xcOHC5sjX66+\/3hwB63ZhBzuUueSSS8z6eBwZ+9JLL5kd7tA1b8\/4cbTs7bffLo0aNTIfE1a3PMbpIfBu98hPR2KS3L+\/2ia3v74mx6tfc86pocnSY31wOYdE8hKImWLLxlbnaXIjNx0BnVV+aoLuiXNbpVTcrdtB5JFlQ1zdusW9vAwmN2AjnE8\/\/dQsb4PQQ8TRE2AJ+nnnnSdXXXWVoKsfa+JnzpwpxYoVM2PvsdbE49755UgI+2WTluVYwtayVlmZ2KVeKLre4\/mFja2XqM1dhtzITUdAZ8V403HLL03QPa27VSDinswHTkVd+eHIGZ9vlj4zvs5+HYyph7H7PRZvNhq6SCQ3ctMR0Fkx3nTc8kMTdE8a2ypl4o4d5caOHSs33HCDlC1bVoYNGyZYzuJ2YUnV8OHDXXebS\/YLu9UXpCORrc9cukVG\/uvXZRaPd6mXvYlMEO+bjHuw0dBRJDdy0xHQWTHedNyC1ATdEya2Spm42ye2YUz85Zdfll27drk+UZkyZaRz586eZ8snfi1\/JYJyJIR9zNvfCbJ2XOmWrdupstHwF2NWaXIjNx0BnRXjTcctKE3QPZ03q5SJu7fbh6NUEI6EsPed8XX2bHgI++zejUM\/th7LQ2w0dLFLbuSmI6CzYrzpuAWhCbon824ViLhbS+Lwf7eraNGictZZZwn+nx9Xqh3plrGns7DDR2w0dJFKbuSmI6CzYrzpuKVaE3RP5c8qEHHH+LtzzB37xSPwsBQO2\/6NGjUqsmPuTy\/YKINfW208k+4ZO7uX\/f2COUuzsdXxIzdy0xHQWVHcddyyrbCEDbvMnXLKKdKxY8c81qY3T6UjkbU3GvlJpISdmbs+1ihSOnbkRm46AjqrVGqC7on8WwWSucd7LOw0h0NgsAa+dOnS\/t8gCRapdGTWbe9nC3s6LXVLhJWNbSJC7j8nN3LTEdBZMd503FKpCbon8m8VCnHHMrj77rsv5vaz\/l\/Ln0WqHGnfTjYdl7vFo8hGw1+MWaXJjdx0BHRWjDcdt1Rpgu5pdFaBiHusCXUYi8eZ7JUrVzbr3IsUKaJ7izxapcKRzyzcKANfPTHO3vuCajLy8tPz+JThMmejofMHuZGbjoDOivGm45YKTdA9id4qEHF3m1BnPfKZZ54pvXv3Tniym\/4VE1umwpH27vh0nxnvRpCNRuK4IjcdI3Ijt+QR0NWUCk3QPYneKhBxtz\/e4cOHTYaOA13wX6zDYvSv5N8ymY50rmeHsIftRDf\/hHJbUNx1FMmN3HQEdFaMNx23ZGqC7gnybhWYuONM9bvuukvq168vQ4YMERwm06tXL3N6Gw51ya\/JdECYTEfiPHYcBoMLh8DM7tM4714KYQ1sNHROITdy0xHQWTHedNySqQm6J8i7VSDifvToUXOW+v79+42QV6pUyWTtOGcd69ubNm0qN954ozntLT+uZDnSfspbVNazx\/IHGw1dpJIbuekI6KwYbzpuydIE3d2TYxWIuGPMfdCgQTJ48GCpU6dOjidfuXKlPProo\/LQQw\/lW\/aeLEfaT3qbcGVd+fu5lZPjpRDWwkZD5xRyIzcdAZ0V403HLVmaoLt7cqzyXdxXr15tsvcJEyak\/VI4+yS6L+46NzkeCmktbDR0jiE3ctMR0Fkx3nTcKO4euVnd8ps3bzbb0FasWNFY7tu3Tx555BE5ePCgGY9P56Vw9p3ohrSpKUPa1PBIJz2LsdHQ+Y3cyE1HQGfFeNNxo7j74Iad6O68806ZN2+e1KxZU4oVKybffPONNGnSxIzHV6tWzUdtyS2aV0c6x9qjnrWDPhsNXQySG7npCOisGG86bnnVBN1dk2sVSLe89cjI4FetWiVLly4VHBzTvHlzadiwYb6dBmc9V14daR9rn97jTGnboEJyvRTC2tho6JxCbuSmI6CzYrzpuOVVE3R3Ta5VIOKOrH3s2LFmsxpk7WG78upIa5vZqM+Qt\/uNjYYuismN3HQEdFaMNx23vGqC7q7JtQpE3LH97P333y\/t27c33fBhu\/LiSPu69ovrlZeXrj8rbK+Xkudho6HDSm7kpiOgs2K86bjlRRN0d0y+VSDijjXtGF8fOXKkdOjQQU4++eQcb1K0aFE566yz8q17Pi+O7DZtpby5Yqt5n6juRucWdmw0dL+M5EZuOgI6K8abjlteNEF3x+RbBSLu2I2uZ8+esnz5ctc3wLj71KlT024pnPOs9kyYSGc5kI2G7peR3MhNR0BnxXjTcaO467iFzkrrSPtEukxY\/mZ3HBsNXRiTG7npCOisGG86blpN0N0tNVYpy9zRFb93714pUaKEFCxYUHbt2iXHjx93fQtsO1umTBlTLj8urSPtE+kyKWuHj9ho6CKV3MhNR0BnxXjTcdNqgu5uqbFKmbijK37AgAEydOhQKV++fOS65e1d8lE+ICZW2LHR0P1Ckhu56QjorBhvOm4U9zjcop6522fJZ9JEOsvlbDR0jQa5kZuOgM6K8abjRnH3wQ1d8ps2bRIc\/dqiRQvTZf\/FF1+YTWxKlSrlo6bkF9U4MpO75Nktr49BNrY6duRGbjoCOiuNJujulDqrlHXLOx\/5rbfeMqfC3XDDDebYV5wUN3DgQPnf\/\/4nU6ZMkQYNGqTuLRPUrHGkdUhMJnbJU9z1oUqR0rEjN3LTEdBZaTRBd6fUWQUi7gcOHDBj723btpWLL744+9x2dN3PmDFDVqxYIcOHD0+bg2Pss+QzsUue4q7\/haRI6diRG7npCOisKO4eudkn19WuXTuH1Zo1a2T06NEybty4tFnnnuld8hR3j4HvUowipWNHbuSmI6Czorh75Ibx9dtvv12uvPJKueCCC3JYLVmyRCZNmmTOcy9durTHGpNbzI8jM32WvEWeja0uBsmN3HQEdFaMNx03P5qgu0PqrQLplsdrvPbaa+bwGIy5X3TRRWb9+6JFi2T8+PHSvXt36datW3Z3fepfO+cd\/Dgy085tj+ULNhq6KCU3ctMR0Fkx3nTc\/GiC7g6ptwpM3DG+PnfuXHnggQfkxx9\/NG+G9e9Dhgwx+80XKlQo9W8b4w5+HGl1yaMqbFyDk+Ay8WKjofM6uZGbjoDOivGm4+ZHE3R3SL1VYOJufxV00x85ciRfd6WzP48fR3K8\/QQ5Nhq6X05yIzcdAZ0V403HzY8m6O6Qeqt8EffUv5a\/O\/hxpLUErkuzSvJ4l3r+bhSh0mw0dM4kN3LTEdBZMd503Pxogu4OqbeiuIuIV0eOeXudjHn7O+OVTO6SZ+au\/8VkY6tjR27kpiOgs\/KqCbrag7GiuPsQd\/t4+47xfwzGQyG9CxtbnWPIjdx0BHRWjDcdN4q7jlvorLw60uqSxyS6TDsFzuk0Nhq6MCY3ctMR0Fkx3nTcvGqCrvZgrALL3DFbfvny5bJw4UIzmc5+4bjXzp07S8mSJYN5a8ddvDjSvgRuQue68vcWlfPlWcNyUzYaOk+QG7npCOisGG86bl40QVdzcFaBiDsOjcH+8dioplatWlKsWM7lY1WrVjXbz5YrVy64N7fdyYsjueVsTtew0dCFKrmRm46AzorxpuPmRRN0NQdnFYi445CYPn36SK9evXLtUBfcq8a+kxdHcryd4p6MWGVjq6NIbuSmI6Cz8qIJupqDswpM3HEiHP5z7i0f3KvmTdwbjfxE0DXP8fYTHNnY6iKX3MhNR0BnxXjTcaO4e+RmdctnZWWZsfUCBQp4tAymmBdHZvoRr05PsNHQxSa5kZuOgM6K8abj5kUTdDUHZxVI5n7o0CEzke6xxx6TevXqybnnnptjfL1o0aJy1llnCf6fH1ciRy74ZqdcNmmZebRMPeKV4p6cyGRjq+NIbuSmI6CzSqQJulqDtQpE3DHmPmzYMNmwYYPr24V9Qt0Ln\/0ofV9cRXG3eY+Nre4XldzITUdAZ8V403GjuOu4hc4qkSO5n3xul7HR0IUxuZGbjoDOivGm45ZIE3S1BmsVSOaOV8K4O7rmp06dKljzjvPdcZZ78+bNpU6dOsG+teNu8RzJ89vdXcNGQxey5EZuOgI6K8abjhvF3Qe39957T4YOHSp\/+tOfzGY2o0aNko8++siIPda\/t2zZ0kdtyS0az5H28fYhbWrKkDY1knvzNK2NjYbOceRGbjoCOivGm44bxd0jtwMHDhhhb9++vTRq1EgGDBhg\/n766afLG2+8Ie+\/\/74557148eIea0xusXiO5OY1zNyTGW1sbHU0yY3cdAR0VhR3j9x27NiRLejly5fP\/jPWvK9Zs0ZGjx4t48aNEyyVS8aFCXyYmT99+nRT3dVXXy39+vWLuQNePEf2mfG1QOBxZfpJcHbfsLHVRSq5kZuOgM6K8abjRnH3yG3v3r1mjP3aa6+VmjVr5hB3jLtPmjTJdM2XLl3aY42xi2Fsf+LEiXL48GHp27evHD161Hw4YGtb\/N1tjX08R3IyHTP3PAelrQI2tjqa5EZuOgI6K4q7D26vvfaavPTSS9K7d2956qmn5Oabb5b9+\/fLww8\/LB06dJBu3bolZXOb3bt3y8CBA6V\/\/\/7SoEED84QLFiyQV155JWbXPxxpXfPnz8\/+86bdR6X9cyeW7zWpUkye7FTJxxtHuyiWNWIJIy9\/BMjNHy+rNLmRm46Ad6tWrVrlKLx27VrvxiEsGdhseWTQs2bNkjFjxsj27dsNisKFC5vu8p49e6ZsvH3fvn0mc69Ro4b5gHC7Yn2l2WfKv35jI7mgTv4cbBPCuOH2s0qnMAPVgSM3ctMR0Fkxc1dwQ3c5uulxYQJdKifR4UPiiSeekGbNmpmsHUMCfsSdO9PFdjAbW0Xwc09+HTRyIzc1AZ0hxd0nt\/Xr18uMGTNk8eLF5uz2P\/7xj9K2bVupWLGiz5q8F0ePwZtvvinPPfecTJ48WSpVyt21HsuRz3+6SW55+X\/mZtx2Nidzirv3GLSXJDdy0xHQWTHedNwo7j64Ydz71ltvNcvfmjZtaiw\/++wzwbgGZra3aNHCR23xM0pk6ziBzpp9v3XrVjOJ7+6773Y9lS6WIzmZjpl7UoLSVgkbWx1RciM3HQGdFcXdIzd0w2OSGzJ1+6lw2Knu2WefNZvaJGud+549e+SWW26RSy65RK688ko5cuSImciHDwncAz0GziuWI61jXlvWKiuz+zT2+LaZUYyNrc7P5EZuOgI6K8abjhvF3SM3+zp353nuqVjnjt6Ae++9Vz799FMzAx9Cj6V4VapUcX1iN0dy29n4zmWj4TH4HcXIjdx0BHRWjDcdN4q7R2448nX48OHSqVMnadKkSQ4rrHNH9n7\/\/fe7ZtUeb5GnYonEndvO5sbLRkMXcuRGbjoCOivGm44bxd0jN3S\/Q8SffPJJueaaa6R+\/frym9\/8xiyneuSRR8xSuDPPPNPUhkwbG84Eebk5cszb62TM29+Zx+BkOop7suKRja2OJLmRm46Azori7pEbuuUh4BhbT3Q1bNhQXn\/99UTFkvpzN0dy29n4iNnY6kKQ3MhNR0BnxXjTcaO4e+SGzH3Xrl3m2NdEV1gyd86Up7gnilXNz9nYaqgJN03SYSM3JTeKu09wGHv\/8ssv5YsvvpCyZcuaE+Jq1aolBQsW9FlTcou7OZIz5SnuyY2yE7VR3HVUyY3cdAR0VhR3H9x++uknsxxu0aJFZq37wYMHTUPXrl07ueeee1K6kU2ix3Q60j5TvkuzSvJ4l3qJqsi4n7Ox1bmc3MhNR0BnxXjTcaO4e+SGXeKwFewvv\/xiNpM56aSTjOXmzZtl1KhRZt93HCRTqFAhjzUmt5jTkfZtZzlT3p01Gw1dDJIbuekI6KwYbzpuFHeP3HC++qBBg8yucXXq1MlhtXLlSnnwwQfN6XBBz5K3HsTpSJzfjgl1uDhTnuLuMcw9FWNj6wlTrkLkRm46AjorirtHbpgtj61n77zzzlzinopNbDw+VnYxpyPty+C+uOtcqZ5VzG+VkS\/PxlbnYnIjNx0BnRXjTceN4u6RG06CGzlypMnMccSr1f2OWfQ4SAZL5EaMGCFFixb1WGNyizkdyZnyifmy0UjMyK0EuZGbjoDOivGm40Zx98Ft9erV0rdvX7N5zYUXXmgEHvu9b9y4UaZMmSINGjTwUVtyi1Lc\/fNko+GfGSzIjdx0BHRWjDcdN4q7T2441GX27Nkyb948M1seB8l07NgxX2fK4xXsjuSe8t6cykbDGydnKXIjNx0BnRXjTceN4q7jFjqrWOLOmfKxXcVGQxfG5EZuOgI6K8abjhvFXcctdFZ2R9qXwXEyHcU92cHKxlZHlNzITUdAZ0Vx13ELnZXdkTwwxpt72Nh648RueR0nciO35BDQ1UJx13ELnVUscWfmzsw92cHKjyIdUXIjNx0BnRXFXcctdFZ2R3IZnDf3sLH1xokZqI4TuZFbcgjoaqG467iFzori7t8lFHf\/zGBBbuSmI6CzYrzpuFHcddxCZ2V3ZNZt75vna1mrrMzu0zh0zxqWB2KjofMEuZGbjoDOivGm40Zx13ELnZXlSPsady6Di+8mNhq6MCY3ctMR0Fkx3nTcKO46bqGzshzJ0+C8u4aNhndW9pLkRm46AjorxpuOG8Vdxy10Vm7iztPgmLmnIlDZ2Oqokhu56QjorCjuOm6hs7IcyTXu3l3DxtY7K2buOlbkRm55J6CrgeKu4xY6K8uROMMdZ7nj2jH+j6F7zjA9EMVd5w1yIzcdAZ0V403HjeKu4xY6K8uRXOPu3TVsNLyzYgaqY0Vu5JZ3AroaKO46bqGzshzZaOQnghnzXAaX2EUU98SM3EqQG7npCOisGG86bhR3HbfQWcGRHyz+r0DccVHcE7uIjUZiRhR3HSNyI7fkEdDVRHHXcQudlVPcucY9sYso7okZUaR0jMiN3JJHQFcTxV3HLXRWcOTz7yyVyyYtM8\/GZXCJXURxT8yIIqVjRG7kljwCupoo7jpuobOCI0e99LFgtjzF3Zt7KO7eODlLkRu56QjorBhvOm4Udx230FnBkb0mvydj3v6O4u7RO2w0PIJyFCM3ctMR0Fkx3nTcKO46bqGzgiM7jn1Lpn28yTwbz3FP7CI2GokZsXtZx4jcyC15BHQ1Udx13EJnBUc2GPCqLPh2p1TPKmbEnVd8AhR3XYSQG7npCOisGG86bhR3HbfQWVHc\/buEjYZ\/ZrAgN3LTEdBZMd503CjuOm6hs4Ijd3aYap6La9y9uYeNhjdOzlLkRm46AjorxpuOG8Vdxy10VjUaNJfdl4wxz9WlWSV5vEu90D1j2B6IjYbOI+RGbjoCOivGm44bxV3HLXRWdnHnBjbe3MNGwxsnZu46TuRGbskhoKuF4q7jFjqr6s3byt6Wg81zIWtH9s4rPgGKuy5CyI3cdAR0Vow3HTeKu45b6Kzs4s5lcN7cw0bDGydmoDpO5EZuySGgq4XiruMWOqvK7W6Vg2dcZp6LW896cw\/F3RsnipSOE7mRW3II6GqhuOu4hc6q0l9GyuHq55vnYubuzT0Ud2+cKFI6TuRGbskhoKuF4q7jFjqrk7tNkaMV6nIDGx+eobj7gGUrSm7kpiOgs2K86bhR3HXcQmdFcffvEjYa\/pnBgtzITUdAZ8V403GjuOu4hc6qwo2vyLESFbiBjQ\/PsNHwAYuZuw4WuZFbngnoKqC467iFzirrtvfNM3F3Ou+uobh7Z2UvSW7kpiOgs2K86bhR3HXcQmdliTs3sPHuGjYa3llR3HWsyI3c8k5AVwPFXcctdFYUd\/8uobj7ZwYLciM3HQGdFeNNx43iruMWOitL3Lk7nXfXsNHwzooZqI4VuZFb3gnoaqC467iFzsoSd25g4901FHfvrChSOlbkRm55J6CrgeKu4xY6K0vcuYGNd9dQ3L2zokjpWJEbueWdgK4GiruOm8rqwIED8uyzz8qUKVMEf27RooXce++9Aic4LwhP9+7d5Ycffsj+Ua9evWTIkCGu97bEfcf4P6qeLRONKO46r5MbuekI6KwYbzpuFHcdN5XV888\/L1988YXcfffdUqpUKZk5c6YsWLBAHnjgASlZsmSOOpctWybvvvuuDBo0SAoUKJDwfhD36lnFzNazvLwRYKPhjZPbh2fNmjV1xhlsxXjTOZ\/cdNwo7jpuvq0OHTokQ4cOlY4dO0rLli2N\/bp164x4jx49WmrXrp2jzrlz58qWLVukR48enu4FcS+4f5uUfmeIzJ8\/35NNphfasGGDVK1aNdMx+H5\/cvONzBiQG7npCHi3atWqVY7Ca9eu9W4cwpIFjh8\/fjyEz5XjkfCIe\/bskWLFikmRIkXMzz788EOZOHGiPPbYY1KpUs7z15955hlZuHChrFq1SrZt2ybt2rUzXfLOctZNIO7cwMZfFDAj8MfLKk1u5KYjoLNivOm4MXPXccuT1bFjx2TevHly3333yYABA0w2b+96x88nTZokWVlZcsUVV4j19\/Xr15su\/OLFi+e6P8Xdv0vYaPhnBgtyIzcdAZ0V403HjeKu46a2WrFihYwcOdKMsd96663yu9\/9ztOYOibWDRw40Ng6u\/DxMBB37k7nzy1RCH5\/b5yc0uSm40hu5KYjoLOKQrylRbc83PPpp5\/KHXfcYf5r3bq1FCxY0NVre\/fulVmzZpkyVjc8xucxZj9mzBipVq2aa+ZOcff3SxCF4Pf3xskpTW46juRGbjoCOqsoxFtaiDuWvkGc27dvLxdddFFcb6Hs7bffLo0aNZKuXbtmd8vv27fP1FGoUCFXcS+x9Bkpsn6hLhJoRQIkQAIkECkCnFAXgDu3bt0q1113naBb3n5VrlxZpk2bZmZtQ9DPO+88ueqqq2Tz5s1moh2Wy2ESHsbe+\/XrJ+XKlQvgaXkLEiABEiABEshfAmmRuecvIt6dBEiABEiABNKLAMU9vfzFpyUBEiABEiCBhAQo7gkRsfspmVsAAA6gSURBVAAJkAAJkAAJpBcBint6+YtPSwIkQAIkQAIJCVDcEyJiARIgARIgARJILwIU9\/TyF5+WBEiABEiABBISoLgnRMQCJEACJEACJJBeBCju6eUvPi0JkAAJkAAJJCRAcU+IiAVIgARIgARIIL0IZKy4Yw\/6Rx99VJ577jkpXbq03HTTTdKtWzfX7WnTy6XJe9rPP\/9chg0bJt9++620aNFC7r33XsGey84LR\/J+9NFH5tS9\/\/3vf1KzZk257bbbzFG79hP7kvdk4a7JKzf7W\/z000\/Sv39\/ueWWWwzrTLz8cMPWoIhHnDlRsWJFs7U04y3+7+nRo0flxRdflMmTJ8uPP\/5oDt7CgVp\/+MMfMvL3NN7vGM4jGTVqlDmPBCeMpuOVkeIOMZoyZYpgW1scG\/vLL7\/InXfeKW3btjUNBC8RBDe29AWfJk2ayIcffijTp0+XBx98MNc2vl9\/\/bVpXEeMGCH169eX\/\/73v3L33XfL6NGjpV69ehmF0w83CwwaXbB69tln5YUXXshIcffD7eeff5YhQ4ZIp06d5JJLLjHH6CJWhw8fzniL83uKD6EJEybI+PHjBVt3f\/XVV+Z39qGHHnI9UCujfnH\/\/2V37NghS5culeeff1727NkjU6dOpbinUyBs27bNZJaDBw+WBg0amEefO3euvPfee6aRLVq0aDq9Tkqe9aWXXsoWaRy2s3v3bunbt6\/Zo79Zs2Y57vn6668LBB4n9iFTP3TokBF3fBRgr\/9Muvxws7i89tprpqHduHGjdO\/ePSPF3Q83fGj++9\/\/Npk7flfxsY6GGOdIFClSJJPCTfxwW7JkiYwbN85koziPY9GiRab3EmJvnaCZUfBcXha9RwsWLDC\/i+ixpLinWUSsWbPGiDgC3epyWbZsmelWRpdVunbDJNMNaABq1KiRLc6HDx82XfSNGzfOJdg4iQ+9HyVLljSPgIN7br75Zundu7dccMEFyXys0NflhxteZvXq1aaBRfcosqouXbpkpLj74TZx4kQ5cuSIoGv+7bffllq1apnMPRO7l\/1ww+8ohiFHjhxpfo\/wYYQDtnA8Nq+cBNDLAbYU9zSLDDfHuQl+mr1W0h7XOjbXOmXPqtjZkLjdEF2kGOKoXr263HXXXdmCn7SHC3FFfrlh3gcaWvRunHHGGUagMlHc\/XJDHL7zzjsyduxYOfvss2XVqlVm+AhjpPj4zJTLLzf0eEyaNMkkMfhwxzwZsITA4wOJ168EKO5pGg2xMneMJz\/++OM8GlbE\/NK7Ze7nnHOOdOzYMZfnMQ768MMPmy4tZO1t2rTJyOENr9zQlYxxPVyYyHnw4MGMFXcw8MrNKoss1BoGAsv777\/fCFSmDQP55YbJhz169DBxF683Lk2b9qQ9NsU9aSiDrQgzRTHmft9990mdOnXMzTHm\/p\/\/\/MdMysm0cTs3+s8884zpXrcaUIy533rrrWbc3ZkdWRMTMfsWPz\/ppJOCdWiI7uaVm5V1zZkzJ9fT9+rVy0wYy6TLKzcwwYxvzPHAvA7MB7HEHZM33T48o8zRDzd8CNjFHRM5MaGuefPm8uc\/\/znKmHy\/G8XdN7JwGCCokaEjW8LSI0ywGzRokKBRzbQx4lgewWQSdBPjAwhdxq+88or5+EGXnjW2btliUg+WwGHGPBrbTL78cLNzssQ+E7vlwcEPN6ssJsQ2bdrUdMujSx5DHOhtyqTLDzd0yz\/yyCNmrhE4LV682PzZ6qbPJG6J3pXinohQiH+OTBRjTegarVChghmzu+yyyzJenCyXIRvCzNF77rnHNLyYdIOx9CpVqpgiEPSPP\/7YdIeiO\/7pp5\/O5W38LNO6Sb1yQ4NavHjxbGaZLu5+ua1YscLEHhphrN7Ah2gmjbf7\/T1FvGECHSYgYgInPsaxnwJ65s4888wQt9T582gU9\/zhzruSAAmQAAmQAAnEIZCRm9gwIkiABEiABEggygQo7lH2Lt+NBEiABEggIwlQ3DPS7XxpEiABEiCBKBOguEfZu3w3EiABEiCBjCRAcc9It\/OlSYAESIAEokyA4h5l7\/LdSIAESIAEMpIAxT0j3c6XTncCOJoSGwy99dZbZq8GP2fAY\/tlbACDvdlr164dWhTYUe2JJ54wx5NOmzYtpc9q3zGwYcOGaX1gSGgdygcLlADFPVDcvFk6EvC7oQWEt2fPnmYLWT+i64cNngnijh3HsNuYny2T00nccRQpNpcqVapUSjeYso6N\/eyzz8zulel8GpifOGLZ6BKguEfXt3yzJBEIq7hrj6RMtrgfO3ZMChQoYP5L5uXlFMJk3g91+fV1su\/P+kggWQQo7skiyXrSmgAOycF2xDNnzjTvcckll5gtTX\/44Qfp2rVr9ru98MILZj9zdIfj+Exs41m+fHm5+uqrzWlbOLcAWfvy5cuNjXUIDE7NQ\/2vvvqqOdMAB5z069cveztfJzxneTwPzj\/AUbrY+hfbhloXnsnZQ4DnmD17tslCcQwvDvXBmfE48\/ybb74x3fJ43hkzZhhBq1u3rjk0CVu54vrvf\/9rthX+4IMPjGi3atXKbNGMk9esnom\/\/e1v8t5778lvfvMbsz85GGJLWNiA0fXXX2+GDHDmALr\/0fX97LPPCg472bVrl1x44YXmbIf69eu7xo5d3K1ucxzxun\/\/fpkyZYrprbjpppsMe2yt6rysLZLtW\/26\/ZvdjuKe1r\/GfHgbAYo7wyHjCRw6dMgIG06zu+666wwP7JW\/ZcsW8+\/YYx\/d39iTG3vrY19znEh27733yumnny44FQ9n17dv316uueYa+f7776V\/\/\/7m6FtLdFE2KytL\/v73v0vhwoWNQKNeHNyBk7rsF855x57+xYoVkxtvvNEI16xZs4zNk08+acagFy5cmOOZ7N3y6GLGRwoEEGJbs2ZNeffdd01XM\/4N17XXXiunnnqqeW7UBxFGnagfYo1nh6C3a9fOnLoGHuvXr5eHHnrIHBWKDxicz4D6cbDQnj17pHfv3uZ0sQ4dOgg+TvAOOIERtngGfNzgQwOnC+LwIXwYPPfcczHPE3cT90WLFpmPrvPPP9\/UhY8cHJmL\/5w9BxT3jP\/VzmgAFPeMdj9fHgQgRDfccIP85S9\/kc6dOxuRgHCh+7pBgwaybNkyc964NQ4LUd64caNceumlOY4chShinN055o7TuKZPn26EsXTp0gY67tmnTx\/zMXHRRRflcASyR3xUPPXUU9mZvZW5YrIXMu54GSZOOcT7QMDxwYEL9phAd84555jsGx8N+HuTJk3Mz1evXm16BvCMGN\/GASOwxQcJrgULFpgPETDABXHHc1j1Q0jnz58v48ePzz41cMmSJaaHAB8U6EnABw5+bh0+hI8qfCTh48A6Y9wOwk3cy5Ytm33UK8riyFycWDhx4sRstlYdFHf+fmcyAYp7Jnuf724IIDN98803jaAii8axv+gGx2lZOMLWKaQov27dOnMELv4P8Uc3NgTKTdwhPBA1t8vt5LxYXccQO1y4Rzxxx0cJuuAffPBBqVOnTq7buo25O\/8NHyiffPKJOU515cqV5r+qVavmEHf7hEH7s1k3RNaODwZk8Ph4wDCE2xXr\/Ho3cT\/vvPNynDQI9uh9wAdEtWrVclRPcecveCYToLhnsvf57jkIoLv5q6++MqKGjLBcuXKm6xtH3lqZO\/4NXd7ooseY81lnnWUyYWTZsTJ3ZLyoA0fm2o95xc3xd+e\/uYkSPijwIWDdI5G4x1vqlkjccS+MZbds2dJ86Jx22mlm7gE+UOyZeyJxhw0+MnDOOj4OMNYObmXKlMnBHUMK6KZ3Xl7EHb0o6BFA1z+GGeyXG0c8A+ZDOI\/ctew45s5GISoEKO5R8STfQ00AY7cQrttuu82MDePCODq6tjEZ7JdffskWdwgxxnzPPfdc+etf\/2rKWt3LmFjnlrnPnTvXjGVjzTaWduFCt\/yoUaPMBwK62u0XxqLxUeEsj278yy+\/3GSu8UQIGTO6zSGsVpe\/fRgAGa5T\/O2Cv3TpUtMNDwHEPARc+NiBMMYSd\/ROoA67aGI4YtiwYUZ4MY8Ak+eQYderVy+bGz58MEnuT3\/6kydxR5c+egOs8XW34QCrIqe4Y2hgxIgRsnPnToq7+reFhulCgOKeLp7ic6aMAIQHQoiZ6BgDh3BgAtrrr79uJntBLJF1Y4wa4g+B2LdvnxHIggULmjFfjKljDB7\/duTIEenbt69Znw3Rwng3Jqhh\/B7142MAIocPCIgbxrjtF4QY5SHC1oQ6CBV6FDCD\/eSTT44r7si8UT8+EpA1o7cB9pgwh\/eBuMUTd3Sh42PHmoy3ePFimTBhgulhgD2ybOc6\/q+\/\/tq8Gz6IMAkPk+9wb7wjNqBBlz7G1\/HuuDcmCeKZ5s2bZ1YdWOPwdg5umTtm4o8ePdp8XGFiI4ZS8NHTqVMnM66PiX34IEFvAD4u8B5YMYD6P\/roIzPREbP08RGCCYsoDx9aPQfM3FP2a8aKAyZAcQ8YOG8XTgKYIIcG\/5133jFj8JjljgwR4+4QW8zKxs+w7Ayii6wbwoQuecwSh4BiFjgyfcwWR5aLjwFrHB7LxCBW\/\/rXv4yoXHHFFWYMGnZuV6yleZYIJhIht2Vn6FXA8ybqlocQY8maNbMeM9ExcQ7vXKFCBfMhhHe2d8uDGT4CMCEPXeXgh+V+4GXthIdJisjwIepYDmgtN3QTdjBxE3csqcN8AHxM4VmwPA8fUZgbgffCJEI8A+5vMcB74H5YMocJglYPA+6BXhisFsC74ErENZzRy6cigdwEKO6MChIggTwTgJCixwK9EFaXOcbZ8UGAcXbncj8vN\/Qy5u6lHj9lKO5+aLFsmAlQ3MPsHT4bCaQJASx7Q\/c4ut7RZY5NajDkgEwb\/47M2u8Fcbe2n8XeABgacc6W91tnrPLcfjZZJFlPWAhQ3MPiCT4HCaQxAWxBi7XxGD\/HigNMLsSQRPfu3XOtBvD6mvaDYyZPnmwm5qVK3HlwjFevsFy6EPg\/Go4SrdeKz2EAAAAASUVORK5CYII=","height":303,"width":503}}
%---
%[output:9dac27de]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAfcAAAEvCAYAAABYGIhlAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQncTmX6xy\/GnvVFyTYkxChkSWWmJsowKTRpqDGiUrYWW6lUtqLQIrRINSMtKpFpKtqGSlkSTYaShMiSfY\/\/53f7n7fznvc8z3PO9T7Pec9znt\/5fPqE977uc873ut77d657LXD8+PHjwosESIAESIAESCAyBApQ3CPjS74ICZAACZAACRgCFHcGAgmQAAmQAAlEjADFPWIO5euQAAmQAAmQAMWdMUACJEACJEACESNAcY+YQ\/k6JEACJEACJEBxZwxEjsCOHTukZ8+esnXrVpk2bZrUrl07Ze+4Zs0aufbaa6VixYoydepUycrKStm97BUH+Y5eXujAgQNy++23y5IlS1LO3Mvz5LXMmDFj5IknnpDKlSur3uell16SO+64Q3r16iVDhgzJ6+Mk1d6KnRYtWoTu2ZL6ohleGcU9wwMgiq8fpPA5xb148eJG5ObMmSMvvPCCoAFNxeV8x6pVqwZy31jvYomZ13fWiJ\/GRsP+008\/la5duxrTKIo73st6R6\/+0nCkTf4SoLjnL3\/ePWIErAw2k8Td+tCoXr26PPDAA4IPnESXRqg1Nomew+3nlvC1b9\/e8\/s46wnqWTXvBxsrTtevXx9oj5P2eWnnnwDF3T+zjLGwC5X10vZuRitrbdKkiVx++eVy3XXXmWLORtFq\/JcvX54rG4pVR8OGDeM2Om7PZmUhzqy2fPnyppseF7pIBw4caJ4Rf\/ZSD+ysLndn3fiZ1S0\/ceJEGTt2rMna3XhZ\/+bWlW9nZL2H1TVs2dm52p9j8uTJ8vTTT7ve17rXpk2bcrHHPzh9g3reeOONHN3r8RihDksM77\/\/frnqqqvMfeLd1xI+670sX9t7PayfWXXGssEwiPNnibrCnc+Ge1nMnXXFy9ztGb4z7u3ijp+hix+X89mcPrb\/3KoDDNatW2f8aw0z2e1gU6NGDTMMYPdBIi5+e1sypuGLyItS3CPiyGS\/hluD7hQst0bSKUQbNmww4meJi\/Vzq9G0xNH5c7ePBMs23rOhka5Tp06OMXdL3K2PC6uR7d+\/f3ZXtpOfvZ5ki7tbdm\/\/yEH2++ijj2YLgv3ZLIFHHda8gljifv3115sy9vdGXRZ7e1e+8\/2tMm7srLLOjxDnx1Ws+y5dutQIkVPcn3rqKdd3hmDh8mMTS+DjxSye\/\/vvv89xn1ji7hR2Z9zPnj07Rz12vrE+WJy\/X7E+NJz83Op2fjQ468bfrXdI9DGU7LaF9QVDgOIeDOe0u4vVsMTKFpFB2IXZrWHHv3344YemwXbLSPBvnTp1yhZ\/q45Ek9Tcuk3tmZIlataEOrtA2Z\/Daz1exR3Zvdcxd3tWhmzX\/vff\/va3ZszX3nvh\/CBwfsC4jblb72evx2r0ITBu97GewxI1S0jscWDnNnjwYOnbt2+OyYtuXO33tb+v5Q+3ngtnPZZgWjZWnMA\/VkYbb76FnaE9w3XeBx8l4B+rW97t48x5X4ubnb09Ru0fltazOOP+3XffNR8I9g8MN072DxbUdfbZZ5vfqURcKO5p1yz7emCKuy9cmVPYaoydE27sImQ1IuiWt4+1Wrb33XefLF68OEd3sZ2gXdzts82tBswuqna7WM9mlYnVLe+cPe+1nlSIuz1Tv+eeewSsrJnmljDYBciZaTk\/YBJNqHNmmvZs2H4fJ7vXXnvNNZvG80D83MTd2dWPsrHiyC1rdNpbIusU91jZM+7nlnHHiivnTP\/t27fHFXdnL4vbHAO3MfdYYursibI+CCxxjzUU5vY7Z\/9oc2ut7Fwo7tFuzynu0fav+u2SIe7IpiAO8SaXxRt\/jrK42wVl6NChMnr0aLE+kiwRy6u4OzM6ZMz2DBp8neO0scTd+SyxPqTsyw7dRN7ZJW0Jl13gLDF3ZtCxxN3rxLewibvlC0vM8VFhX1aZV3FPxIVj7urmMS0MKe5p4abgHzKv3fJWlmxlfvaGxi4wVvbvJ3N3y4rsdV588cWuY+7OzN1vPRAuK9txzhmwnt9rtzw8ar8\/hi4s4XPrTtd0yzvfzy62frvlE3Uv29e3u8WO1y52exxYPo2Vuds\/XrwM6SSrW96t6z9Wl7rbMBD+zdnzgtiyuMXL3P10y2MeSywuVpxytnzwbWtQd6S4B0U6ze6T6gl1sbIVzH5O1C3vlhECr1Un\/mzfxMYac3eKe6J68CyxJiZ5EXc8R7zJSnZxcnYjx7qv24Q69JA4J8fZZ1C7hR7E3foIck58s3drx5pQZ39eZy9PvElrbpPJ4Le77rpLbr755lwTL63uf3RB2yepWb6ONwnPmrlvf\/9EE+qwL4GXpXDOyW7WPWL1TMQaVnFjH0\/cUU+s2MDPrPvHKuP8gOQ69zRrmH08LsXdB6xMK+p1KRyyLWQimFhlb4ytsUiniNqzQE23PO4Rr06vY+6J6nH7uXOpGMo4d6izN\/yJZiJbjbBbOa9L4azJZM77OlcD2MfZnR8JlshgOR8E0\/4h5IwD54eI29itm4i6jR0ju3SKmRVDPXr0kD59+piYck7gdOtJcApsrN\/XeEvh7CLstVvb7b5extztcwbwPg899JBZpglf2GfuJ4oN\/ByXvfcHf3d+gFjCzh3qMqMlp7hnhp9T8paJZrWn5KasNGkE3Gabuw0JJLphop6WRPb8uXcCbkMLbl313mtkyagSoLhH1bMBvBfFPQDIKbxFvKGXRFmr87E4OSuFjnJUHWtIQLtVbnBPzjsFSYDiHiTtiN2L4p7+Dk009OL1DaN2cIzX986vcvF27MuvZ+J9w0UgLcV9wYIFgrHBSZMmuZ7C9d1330n37t3lhx9+yKadaOwzXG7h05AACZAACZCAnkDaifvGjRvlhhtukCJFisTce3zZsmWCNaKDBg2SAgUK6OnQkgRIgARIgATSkEBaiTu6\/kaMGGEOSfjkk09k3Lhxrpn73LlzZcuWLYLZtrxIgARIgARIINMIpI24Hz9+XGbOnCnI3Nu0aWNO34ol7s8884wsXLhQVq1aJdu2bZN27dqZE8AqVark6t\/TTjst0\/zO9yUBEiABEohBYP78+VKzZs205pM24r569WrB6VfDhg0zgo3tOt3E\/dixY9lj8VdccYVYf8dOTLHOmoa4r127Nq0dGfTDk5mOOLmRm46Azorxlrnc0kLc9+zZIzhcA93sDRo0MGdFxxJ3N1diYh02hxg5cqTY9762yvIXwP8vAJn5ZwYLciM3HQGdFeMtc7mlhbh72TLScuHevXtl1qxZ0rp16+xu+HXr1gkO58COX9WqVcvlbf4C+P8FIDP\/zCjuOmbkRm56AjrLKLRvaSHuTvfEy9yt9baNGjUyxzZa3fL79u0zAl+oUCGKuy7ec1hhuWG6j0klAYPvKsjNNzJjQG7kpiOgs6K467jl2cop7pagn3feeYLDIjZv3iyPPfaYmYBXrFgxwdh7v379pFy5cq73joIj8wzVZwVsbH0C+\/\/i5EZuOgI6K8abjlsUNCEtM3edu2JbRcGRyWaSqD42GokIuf+c3MhNR0BnxXjTcYuCJlDcOclJFf1sNFTY2L2sw0Zu5KYkoDOjuOu4hc4qCo4MGirFXUec3MhNR0BnxXjTcYuCJjBzZ+auin42GipszEB12MiN3JQEdGYUdx230FlFwZFBQ6W464iTG7npCOisGG86blHQBGbuzNxV0c9GQ4WNGagOG7mRm5KAzoziruMWOqsoODJoqBR3HXFyIzcdAZ0V403HLQqawMydmbsq+tloqLAxA9VhIzdyUxLQmVHcddxCZxUFRwYNleKuI05u5KYjoLNivOm4RUETmLkzc1dFPxsNFTZmoDps5EZuSgI6M4q7jlvorKLgyKChUtx1xMmN3HQEdFaMNx23KGgCM3dm7qroZ6OhwsYMVIeN3MhNSUBnRnHXcQudVRQcGTRUiruOOLmRm46AzorxpuMWBU1g5s7MXRX9bDRU2JiB6rCRG7kpCejMKO46bqGzioIjg4ZKcdcRJzdy0xHQWTHedNyioAnM3Jm5q6KfjYYKGzNQHTZyIzclAZ0ZxV3HLXRWUXBk0FAp7jri5EZuOgI6K8abjlsUNIGZOzN3VfSz0VBhYwaqw0Zu5KYkoDOjuOu4hc4qCo4MGirFXUec3MhNR0BnxXjTcYuCJjBzZ+auin42GipszEB12MiN3JQEdGYUdx230FlFwZFBQ6W464iTG7npCOisGG86blHQBGbuzNxV0c9GQ4WNGagOG7mRm5KAzoziruMWOqsoODJoqBR3HXFyIzcdAZ0V403HLQqawMydmbsq+tloqLAxA9VhIzdyUxLQmVHcddxCZxUFRwYNleKuI05u5KYjoLNivOm4RUETmLkzc1dFPxsNFTZmoDps5EZuSgI6M4q7jlvorKLgyKChUtx1xMmN3HQEdFaMNx23KGgCM3dm7qroZ6OhwsYMVIeN3MhNSUBnRnHXcQudVRQcGTRUiruOOLmRm46AzorxpuMWBU1g5s7MXRX9bDRU2JiB6rCRG7kpCejMKO46bqGzioIjg4ZKcdcRJzdy0xHQWTHedNyioAnM3Jm5q6KfjYYKGzNQHTZyIzclAZ0ZxV3HLXRWUXBk0FAp7jri5EZuOgI6K8abjlvVC\/8mGz74h844JFbM3Jm5q0KRjYYKGzNQHTZyIzclAf9mMz7fLH1mfC07xv\/Rv3GILCjuFHdVOFLcVdgoUjps5EZuSgL+zSju\/pmF1oLd8v5dQ3H3zwwW5EZuOgI6K8abf24Ud\/\/MQmtBcffvGjYa\/plR3HXMyI3c9AT8W1Lc\/TMLrQXF3b9rKO7+mVGkdMzIjdz0BPxbjnl7nYx5+zuOuftHFz4Lirt\/n1Dc\/TOjSOmYkRu56Qn4t6S4+2cWWguKu3\/XUNz9M6NI6ZiRG7npCfi3pLgnYHbs2DHZtWuXHD9+3BfdAgUKSLly5XzZ5LUwxd0\/QYq7f2YUKR0zciM3PQH\/lhT3BMx27NghPXv2lOXLl\/ui27BhQ3n99dd92eS1MMXdP0GKu39mFCkdM3IjNz0B\/5YUdw\/iPmDAABk6dKjUrl3bE+E1a9bI6NGjZdq0aZ7KJ6sQxd0\/SYq7f2YUKR0zciM3PQH\/ltjABjPmuYlNDHZ79+6VWbNmSevWraVSpUqeCG\/evFnmzZsn11xzjafyySpEcfdPkuLunxlFSseM3MhNT8C\/JcU9AbOff\/5ZJkyYIBdeeKE0b95cSpYs6Z9yQBYUd\/+gKe7+mVGkdMzIjdz0BPxbUtwTMNuzZ4\/ccccd8q9\/\/UsKFy4s7du3l7\/85S\/StGlTKVSokH\/iKbSguPuHS3H3z4wipWNGbuSmJ+DfkuLukRky+Pfff19eeeUVWbJkiRQvXly6du0qHTp0MGPxBQsW9FhT6opR3P2zpbj7Z0aR0jEjN3LTE\/Bvednjy2TBtzs55u4HHYT+448\/ln\/+859G6CtWrGjG1y+77DKpXLmyYBlcflwUd\/\/UKe7+mVGkdMzIjdz0BPxbUtz9M8thgQl3EPqHH35YihYtKlOnTpWsrKw81qozp7j750Zx98+MIqVjRm7kpifg35Li7p+ZsTh69KisWLFC5syZI6+++qr5tyuuuEJuu+22fJt0R3H370yKu39mFCkdM3IjNz0B\/5aNRn4i63ccZLe8F3SHDx82go5x97feeksOHjworVq1ki5dupgJdhiHT9W1YMECmThxokyaNClmzwDF3T99irt\/ZhQpHTNyIzc9Af+WWbe9b4y4zj0GO2w\/+\/nnn8vMmTPlnXfeEcyeb9asmXTr1k3+8Ic\/SKlSpfxT92mxceNGueGGG6RIkSJxu\/0p7j7B8lxy\/8D+34IfRTp05EZuOgL+rJCxI3OnuMfhZm0\/iyy9c+fOcumll5oJdEFdBw4ckBEjRkiNGjXkk08+kXHjxsXN3K3nmj9\/flCPmNb32bBhg1StWjWt3yE\/Hp7cdNTJjdx0BLxboTf5aIW6srflYIp7PGyHDh2S3bt3S4UKFQKfBY\/DatBjgMy9TZs2Mnbs2ITivnbtWu9RwJLCTEoXBORGbjoCOivGmz9uC77ZKZdNWkZxj4cNmXt+7S2\/evVqmTx5sgwbNky2bdtm9qtPlLlT3P39ErDR8MfLKk1u5KYjoLNivPnjZs2UZ7d8gm75\/v37m53pqlSp4okwMm3Mosc6eO2Fsf177rlHevToIQ0aNBDrMBqKu5aoux0bDR1PciM3HQGdFePNHzdrpnzB\/dtk25Qr\/RmHrHSB434PXPf4AtiwBpkzxsr8XBjHfeyxx\/yY5CgLMb\/22mtl06ZNuep44YUXpEWLFrn+nRPq\/ONmo+GfGSzIjdx0BHRWjDfv3OyT6YqsXyibZ97l3TiEJVMm7mF5V2buqfEEGw0dV3IjNx0BnRXjzTu3x95fL\/fM+dYYlH5niKxb+Zl34xCWpLiLCDN3\/5HJRsM\/M2buOmbkRm56At4s7Vl79axisvuZqyXd52FFXty9uJbi7oVSzjIUd\/\/MKFI6ZuRGbnoC3ixnfL5ZcBocLmxeEwVNoLgzc\/cW\/Y5SFHcVNo6567CRG7kpCSQ2c2btX9x1LsU9Mbb0KBGFr7SgSVPcdcTJjdx0BHRWjLf43CDsfWd8bY54xfV4l3rSpVklirvfcMPMeewtf+TIEXPMK2a016tXL5CtaOM9K8Xdryc569s\/sRMWbGx15MiN3HQEYltB2F9dtkVGzD2xgVnLWmVldp\/G5s9R0ITAuuWXLVsmN910k9mCFrvXjR8\/XmbMmGH2n8fBLnXq1Em27zzXFwVHen7ZJBVkY6sDSW7kpiOgs2K8uXODsGOcfczb35kCmEQ3u3dj83+Ku49Yw6lwWPPeuHFjcxrcoEGDZOjQoVKzZk15+umnZd26dTJ8+HBzwEt+XBR3\/9TZaPhnxsxdx4zcyE1PILclhB1bzOL\/bsJOcfdB274Vbfny5XNsS4utYkeNGiUTJkyIebCLj1upilLc\/WOjuPtnRpHSMSM3ctMTyGmJveP7vvh1DmHHBDrnFQVNCKRbHgfIDBw4ULAdbeXKlXOI+8qVK+XBBx+Uhx9+WMqVK5csH\/qqJwqO9PXCSShMcddBJDdy0xHQWTHeTnBzdsPHytgtylHQhEDEHTvcTpkyxezz3q9fP9MFf8cdd8hJJ50kDzzwgDmW9eabb5ZChQrpIjiPVlFwZB4R+DZno+EbmTEgN3LTEdBZMd5EXv\/iJ+n5\/FfZADGu\/nDnM+TCOrGTyShoQiDiDqp79+41Xe\/\/+Mc\/5OjRo9mg27VrZw56CfKs9yh2weh+9fVWbDR07MiN3HQEdFaZGm\/I1L\/atFeufmZFDnDOiXOxqFLcfcYbMngsf0PA4TrllFOkVq1aUrBgQZ81Jbd4FByZXCKJa8vURiMxmfglyE1HkNzIzQsBjKmPffu77HXrlg1EfeJf60nL08t6qYZL4TxREpFjx47Jrl27JNYBdOiOL1WqlBQoUMBrlUktR3H3j5ONrX9msCA3ctMR0FllQrwhS1+9ZZ90furLXJD8irpVQRQ0IZBuecyW79mzpyxfvjxmhELcb7zxRunevbsUL15cF8lKqyg4UvnqarNMaDTUcOIYkpuOKrmRm5PAS4s3y\/3\/\/i575rv951pRp7gr4mzJkiXy6KOPmhnzdevWNTVgYxv824ABA6RkyZJmTL5p06Zms5sgL4q7f9psbP0zY+auY0Zu5GatSR8373v5x6ebXIFA0Gfd1EgKFiiQvRmNllwUNCGQzP3AgQNy9913S5cuXaRJkyY5eEP0sVPdiBEj5NtvvzVi\/+STT2p9orKLgiNVL54HI4q7Dh65kZuOgM4qneMN4+cHjx6TR+d\/n2sM3aIBQe97YXW5pH75PAu6nXAUNCEQcbdvYlO7du0cUYrlcaNHj5Zx48bJ9u3bzZ+nTZumi2SlVRQcqXx1tVk6Nxrql06CIbnpIJJb9LlBzLfvPSxTF26MKeagAEGfcGVdqVWxRFIFneKuiDHsJY\/MvWHDhiZ7t2bHY4Ldyy+\/LO+\/\/7489NBD8tlnn8mLL77IzF3BOGgTNrY64uRGbjoCOquwxpvVzf7gO+vkP9\/87Dpubs\/Oz69VVoa0qWn+ydr\/XUfEm1UUEr5AMnfgxDazGEsvUaKEtGzZ0mxYs3jxYvnmm2\/MWDsm0WEy3X333SedOnXy5oEklYqCI5OEwnM1YW00PL9APhUkNx14cktfbpaQIyNftn533KzcEu\/fn15OBl1SIzAxd9KNgiYEJu6Ah+55HPn6n\/\/8xyyNO+ecc6Rt27bmRLjNmzfLTz\/9JGeeeWbg696j4Ejdr77eio2tjh25kZuOgM4qP+IN3euYyf799gMJhdwS8z+fWVHa\/q6C53XoOhreraKgCYGKuxtaBN9zzz1nZsxjOVx+XFFwZNDc8qPRCPodU3E\/ctNRJbdwcbOy8dlf\/iTvfLXdk4iHJSv3QjIKmhCYuKNbHgL+1Ve\/7vFrQf79739vznSnuHsJu3CUYWOr8wO5kZuOgM4qGfGGTHzDzoPywqIfPYu4JeQYK+\/S7FQzTh7EWLmOUm4rirtHktZ57kWLFpXzzz9fpk+fLh07djQ712Ey3e233y5VqlTxWFvyi0XBkcmnEr\/GZDQaQT9zGO5HbjovkFtquVmZ+MQP1suqH\/f5FvHq5YrJpWdVlPqnlgxN17qO2AmrKGhCIJn7zz\/\/LIMGDZLBgwcbaGPGjBEcGNO4cWOZM2eObNiwwexOx+1n8xKOwdqysdXxJjdy0xHQWdnjzRLwN1dslX+v3Cbrfz4Yd5a6845W5n15w5Ol5\/knkrF0ysb9EKS4e6TlXOeONe0Q9osuusgcA\/vII4+Y9e2lS5f2WGNyi0XBkcklkrg2ilRiRm4lyI3cdAS8WVkC\/o9Fm2TR2l2ydute2bT711M4vdRiutDLFZOWtcvJeaeVzRbwqAq5G5MoaEIgmbvVLY+Z8FdddVV2tt63b19ZunSp2cDm8ccfl3LlYp+v6yUotWWi4Ejtu2vtKFI6cuRGbjoCv1pBwI8cOy6Pv79evvlpv68udKsWS6hbnl5O\/tq0UnYWnkkCHs8PUdCEQMQdEFeuXGm63vv162eWwPXu3duc675lyxbp0aOH9OnTx6x9z48rCo4MmhtFSkec3MgtHgEr80aZR977XtZs2e+7+9wu4GhjLzyjogxo\/VvZuPNQJMbDdRHkzyoKmhCYuAMtAu2XX34RTKxbv369zJ4925znju55\/Ft+XVFwZNDsKFI64uRGbpaAP\/7BD\/L1j3vV4g2SVhd6\/colpfcF1cwYuv3McsabLt6ioAmBiPvevXtl1qxZ0rp1a6lU6UQXkHVh85p58+ZJhw4dzMlw+XFFwZFBc2OjoSNObtHmhmVjENyVm\/bK3BVb5YcdB1Xd5vbsG3\/GkrI\/\/a6CNKx6Yi8Qr93njDddvEVBE1Iq7thT\/ssvv5Rt27bJlClTpGvXrrmWvK1bt86MwU+ePFmysrJ0nsijVRQcmUcEvs3ZaPhGZgzILX25WRk3xrlnLf9J1m07kKes2y7SGPu+unkl+eXYr8LtVcDjEWW86eItCpqQUnHHLPmePXvK8uXLYxIuXLiw3HLLLXL99ddzzF0Xh\/lixUZDh53cwsnNyrjXbtsvc77cmqexbvsbWt3m1bOKS6fGJ8vpJ5fwlXnraP1qxXjTEaS4e+QW78hXj1WktFgUHJlSQC6Vs9HQESe34LlZwv3Fhj0y7+vtScm47Vk3lo21O7OitGtQIXvMG1l+MjJvHS2Ke165RUETUpq55xVwUPZRcGRQrKz7UKR0xMktedzsM8tnLt0iH6zeIXJc8txV7hTuFrXKyh9OP7FM1z5ZTfcmwVox3nS8o6AJKRN3TKLDWe04\/S3RVaZMGencuTMn1CUCFaKfs9HQOYPcvHGzsu1lP+yWd\/67XVZv2ilbD4ivHdVi3cnKqJFxX1g3S5rXKGOKZv97VjFvD5kGpRhvOidR3ONww5azw4YNM1vLJrqqVq0qw4cP5yY2iUCF6OdsNHTOyGRuVqaN\/x87flxeXrJF1m\/P+6Q0yxN20a6WVUy6n1tZDh09ni3cYegm10WN3iqT401PjXvL54VdqGyj8JUWNFA2GjriUeRm7x5Hlo1sG0vA\/O5dHo9o5dKFzIRbZNut6pWXjo1ODt34ti4iUmsVxXhLLbETtUdBE1LWLe90wPHjx82yuMcee0w+\/\/xzs488jnrFrnXVq1cPwl8x7xEFRwYNkI2Gjni6cLNn2ch4X\/j8R\/n4m53mpZMp2vZsu1H10tL6jCypUb54drZtTUxLF266qEidFbnp2EZBEwIT9wULFsitt94ql156qRH1ffv2mY1tLMFv0aKFzgtJsIqCI5OAwVcVbDR84couHAZu1nj2J2t3yn++2ZnUrnHrRXOIdrVSckn9CtmCjT\/47SIPAzedx\/PXitx0\/KOgCYGI+4EDB2To0KFy8cUXm6NerQvZPCbdffzxx\/LAAw9I8eInvtiDvqLgyKCZsdHQEU8FN3u3+Ldb98uS73fLR2t+TmmWjTHtciUKm13T7GLtV7S9UkwFN6\/3Tudy5KbzXhQ0IRBxj7fOHUe+4rhXnAzHHep0gZgfVmw0dNS9crN3i1ctV1Se\/\/RHWbzuxMqTZHaL5xDmcsWkbqWT5Ormp0rWSYWzXzBVgu2HoFdufurMhLLkpvMyxd0jt927d5sueZwE16RJkxxWOC3uwQcflIcffpiz5T3yDEMxNho6LyxcvkaOn1RRKpctKlMXbJAVG\/emXLCRZZ9R6SRpXK10juVeYdloxQtJxpsXSrnLkJuOG8XdIzd0vz\/\/\/PPy5ptvyj333CP169eXAgUKCPaVHzlypDRt2tRMrMO\/5ccVBUcGzY2Nxq\/ErTHsHfuOyD8X\/Sirt+xLuWDjBqefUkJuuei32Wu\/zVanEVqjbY9pxpvuN5zcdNyioAmBdMsDL8bdJ02aJE8++aQcOXKp0qz7AAAgAElEQVTEEMfSlmuuucbsLY\/Z8\/l1RcGRQbOLeqNhCfaCb36WT9buku+3HwhEsJvWKGPWZ9uvqAq2n5iNerz5YeGnLLn5ofVr2ShoQiDijnPc9+\/fbwQcXfTffvut4MS4008\/XcqXL59vGbvlyig4UhfCeqt0azQsscbmKTOXbJF1WIedxA1U3MQYa7JxNa9ZRi6sc+LEw82bf5Tm9WtGNsPWR1R8y3SLt1Rx8FsvufkldqJ8FDQhEHHHhLrrrrvOjKlfffXVcu655+bbzHg3V0fBkboQ1lvlZ6NhjRVbgv3x2p3y8bc7zYEguJI94Qx12pd21axQXFqcVlaqlTvRDY7nwZ7jXsaw85Ob3tv5b0luOh+Qm45bFDQhEHE\/duyYYFb8W2+9Ja+88opg3\/m2bdvK3\/\/+d6lbt64ULFhQ54EkWUXBkUlC4bmaVDQa1gzxw78ck+mLfpQl63cn7SAQtxezC3atiiWk1RlZUrbEiVni1vi1F8H2DI3nuftBlaNsKuJN\/TBpZEhuOmdFQRMCEXc7Xkvo33jjDSP0RYoUMWe+I6MvWrSozhN5tIqCI\/OIwLd5okbDmV3jxK7P1u2S9dsPBpJd\/7Z8cal\/6klyZpVS2dl1GA4GScTNtyMyxIDcdI4mNx23KGhC4OJuoT58+LB89tlnZoLdnj17ZOrUqVznrovDQK2s7PqHH36QFbuKy5qf9smaLftT0hXu7A6vfUoJaX1GeSlVrBCXdAXq9fy\/GUVK5wNy03GjuPvkhu74ZcuWyYwZM2T+\/PlSoUIFM1v+sssuk8qVK+fbxLooONKnK1yLW8L95oqt8u+V21KSYdu7wk+rWEKa1yht\/H5+rZxj1lGdIc7GVhep5EZuOgI6qyhoQiCZO0T93nvvldmzZ5uJdFdccYVceeWVoRhvh+uj4MhEIWwJN7rHF323K6mnduUYuz65hFxUN8tsTWrvBk\/22HWi9w3rzylSOs+QG7npCOisoqAJgYg7znZ\/9NFHpU2bNnL22WebcXa\/F+rAiXLTp083phij79evn+uudmgIunfvLug6tq5evXrJkCFDXG8bBUdaL2aJ+IT538u3P+2XBd+eOMnL7+U8G\/vUMkWlW4vK2TPCwfb8hrX9Vpvx5SlSuhAgN3LTEdBZRUETAhF3Hd5frbDD3cSJEwXj9H379hWsm8de9Fhah787d7ZD1\/+7774rgwYN8tTVn86OhJiPn\/e9ICO3HyCSiLmZDV6umGBr0upZxaVLs0rGxFralahbnI1tIsLuPyc3ctMR0Fkx3nTc0lkTrDdOC3HHxjcDBw6U\/v37S4MGDcyz4whZzLZ3O01u7ty5smXLFunRo4cnz6aLIy3xHvv2OnO+dqLLEvArmpwitSqU8LwWO1G9+DkbDS+UcpchN3LTEdBZMd503NJFE+K9XVqIu\/MFcBY8MvcaNWpIt27dcr3fM888IwsXLpRVq1bJtm3bzDGz6JKvVOlEduq84EjrwkS\/sF1LNh6UJxbtFPw\/1lW5dCE5\/7fFpdvZZUwR\/D2V14YNG6Rq1aqpvEUk6yY3nVvJjdx0BLxbtWrVKkfhtWvXejcOYcm0E\/cxY8bIE088Ic2aNTNZe82aNXNgxTp67GGP42Mxcc\/6+\/r162OeGR\/Wr7RbXv6fPP\/pJtewQVb+0nVnSfEiv8mXrUyZEeh+m8mN3HQEdFaMNx23sGqCn7cJRNwxW37WrFnSunXrXNnz5s2bZd68edKhQwcpWbKkp2fHmDtOmHvuuedk8uTJMTNyqzJM\/kK3Pk6gq1079ySwMDkSW6qOffu7XBPhIOZ\/qF1OBl5cI1\/E3OkYNhqeQjVXIXIjNx0BnRXjTcctTJqgewORlIo7Dof58ssvTdf4lClTpGvXrlKlSpUcz4pjX+fMmWNEGtm224UARbY+ePDg7DJbt26VAQMGyN13351DsN0+JHCPoUOHCrL+atWq5bpFWBx52ePLXEV94l\/rmfHyMF1sNHTeIDdy0xHQWTHedNzCogm6pz9hlVJxx4Ex2Fp2+fLlMZ+xcOHC5sjX66+\/3hwB63ZhBzuUueSSS8z6eBwZ+9JLL5kd7tA1b8\/4cbTs7bffLo0aNTIfE1a3PMbpIfBu98hPR2KS3L+\/2ia3v74mx6tfc86pocnSY31wOYdE8hKImWLLxlbnaXIjNx0BnVV+aoLuiXNbpVTcrdtB5JFlQ1zdusW9vAwmN2AjnE8\/\/dQsb4PQQ8TRE2AJ+nnnnSdXXXWVoKsfa+JnzpwpxYoVM2PvsdbE49755UgI+2WTluVYwtayVlmZ2KVeKLre4\/mFja2XqM1dhtzITUdAZ8V403HLL03QPa27VSDinswHTkVd+eHIGZ9vlj4zvs5+HYyph7H7PRZvNhq6SCQ3ctMR0Fkx3nTc8kMTdE8a2ypl4o4d5caOHSs33HCDlC1bVoYNGyZYzuJ2YUnV8OHDXXebS\/YLu9UXpCORrc9cukVG\/uvXZRaPd6mXvYlMEO+bjHuw0dBRJDdy0xHQWTHedNyC1ATdEya2Spm42ye2YUz85Zdfll27drk+UZkyZaRz586eZ8snfi1\/JYJyJIR9zNvfCbJ2XOmWrdupstHwF2NWaXIjNx0BnRXjTcctKE3QPZ03q5SJu7fbh6NUEI6EsPed8XX2bHgI++zejUM\/th7LQ2w0dLFLbuSmI6CzYrzpuAWhCbon824ViLhbS+Lwf7eraNGictZZZwn+nx9Xqh3plrGns7DDR2w0dJFKbuSmI6CzYrzpuKVaE3RP5c8qEHHH+LtzzB37xSPwsBQO2\/6NGjUqsmPuTy\/YKINfW208k+4ZO7uX\/f2COUuzsdXxIzdy0xHQWVHcddyyrbCEDbvMnXLKKdKxY8c81qY3T6UjkbU3GvlJpISdmbs+1ihSOnbkRm46AjqrVGqC7on8WwWSucd7LOw0h0NgsAa+dOnS\/t8gCRapdGTWbe9nC3s6LXVLhJWNbSJC7j8nN3LTEdBZMd503FKpCbon8m8VCnHHMrj77rsv5vaz\/l\/Ln0WqHGnfTjYdl7vFo8hGw1+MWaXJjdx0BHRWjDcdt1Rpgu5pdFaBiHusCXUYi8eZ7JUrVzbr3IsUKaJ7izxapcKRzyzcKANfPTHO3vuCajLy8tPz+JThMmejofMHuZGbjoDOivGm45YKTdA9id4qEHF3m1BnPfKZZ54pvXv3Tniym\/4VE1umwpH27vh0nxnvRpCNRuK4IjcdI3Ijt+QR0NWUCk3QPYneKhBxtz\/e4cOHTYaOA13wX6zDYvSv5N8ymY50rmeHsIftRDf\/hHJbUNx1FMmN3HQEdFaMNx23ZGqC7gnybhWYuONM9bvuukvq168vQ4YMERwm06tXL3N6Gw51ya\/JdECYTEfiPHYcBoMLh8DM7tM4714KYQ1sNHROITdy0xHQWTHedNySqQm6J8i7VSDifvToUXOW+v79+42QV6pUyWTtOGcd69ubNm0qN954ozntLT+uZDnSfspbVNazx\/IHGw1dpJIbuekI6KwYbzpuydIE3d2TYxWIuGPMfdCgQTJ48GCpU6dOjidfuXKlPProo\/LQQw\/lW\/aeLEfaT3qbcGVd+fu5lZPjpRDWwkZD5xRyIzcdAZ0V403HLVmaoLt7cqzyXdxXr15tsvcJEyak\/VI4+yS6L+46NzkeCmktbDR0jiE3ctMR0Fkx3nTcKO4euVnd8ps3bzbb0FasWNFY7tu3Tx555BE5ePCgGY9P56Vw9p3ohrSpKUPa1PBIJz2LsdHQ+Y3cyE1HQGfFeNNxo7j74Iad6O68806ZN2+e1KxZU4oVKybffPONNGnSxIzHV6tWzUdtyS2aV0c6x9qjnrWDPhsNXQySG7npCOisGG86bnnVBN1dk2sVSLe89cjI4FetWiVLly4VHBzTvHlzadiwYb6dBmc9V14daR9rn97jTGnboEJyvRTC2tho6JxCbuSmI6CzYrzpuOVVE3R3Ta5VIOKOrH3s2LFmsxpk7WG78upIa5vZqM+Qt\/uNjYYuismN3HQEdFaMNx23vGqC7q7JtQpE3LH97P333y\/t27c33fBhu\/LiSPu69ovrlZeXrj8rbK+Xkudho6HDSm7kpiOgs2K86bjlRRN0d0y+VSDijjXtGF8fOXKkdOjQQU4++eQcb1K0aFE566yz8q17Pi+O7DZtpby5Yqt5n6juRucWdmw0dL+M5EZuOgI6K8abjlteNEF3x+RbBSLu2I2uZ8+esnz5ctc3wLj71KlT024pnPOs9kyYSGc5kI2G7peR3MhNR0BnxXjTcaO467iFzkrrSPtEukxY\/mZ3HBsNXRiTG7npCOisGG86blpN0N0tNVYpy9zRFb93714pUaKEFCxYUHbt2iXHjx93fQtsO1umTBlTLj8urSPtE+kyKWuHj9ho6CKV3MhNR0BnxXjTcdNqgu5uqbFKmbijK37AgAEydOhQKV++fOS65e1d8lE+ICZW2LHR0P1Ckhu56QjorBhvOm4U9zjcop6522fJZ9JEOsvlbDR0jQa5kZuOgM6K8abjRnH3wQ1d8ps2bRIc\/dqiRQvTZf\/FF1+YTWxKlSrlo6bkF9U4MpO75Nktr49BNrY6duRGbjoCOiuNJujulDqrlHXLOx\/5rbfeMqfC3XDDDebYV5wUN3DgQPnf\/\/4nU6ZMkQYNGqTuLRPUrHGkdUhMJnbJU9z1oUqR0rEjN3LTEdBZaTRBd6fUWQUi7gcOHDBj723btpWLL744+9x2dN3PmDFDVqxYIcOHD0+bg2Pss+QzsUue4q7\/haRI6diRG7npCOisKO4eudkn19WuXTuH1Zo1a2T06NEybty4tFnnnuld8hR3j4HvUowipWNHbuSmI6Czorh75Ibx9dtvv12uvPJKueCCC3JYLVmyRCZNmmTOcy9durTHGpNbzI8jM32WvEWeja0uBsmN3HQEdFaMNx03P5qgu0PqrQLplsdrvPbaa+bwGIy5X3TRRWb9+6JFi2T8+PHSvXt36datW3Z3fepfO+cd\/Dgy085tj+ULNhq6KCU3ctMR0Fkx3nTc\/GiC7g6ptwpM3DG+PnfuXHnggQfkxx9\/NG+G9e9Dhgwx+80XKlQo9W8b4w5+HGl1yaMqbFyDk+Ay8WKjofM6uZGbjoDOivGm4+ZHE3R3SL1VYOJufxV00x85ciRfd6WzP48fR3K8\/QQ5Nhq6X05yIzcdAZ0V403HzY8m6O6Qeqt8EffUv5a\/O\/hxpLUErkuzSvJ4l3r+bhSh0mw0dM4kN3LTEdBZMd503Pxogu4OqbeiuIuIV0eOeXudjHn7O+OVTO6SZ+au\/8VkY6tjR27kpiOgs\/KqCbrag7GiuPsQd\/t4+47xfwzGQyG9CxtbnWPIjdx0BHRWjDcdN4q7jlvorLw60uqSxyS6TDsFzuk0Nhq6MCY3ctMR0Fkx3nTcvGqCrvZgrALL3DFbfvny5bJw4UIzmc5+4bjXzp07S8mSJYN5a8ddvDjSvgRuQue68vcWlfPlWcNyUzYaOk+QG7npCOisGG86bl40QVdzcFaBiDsOjcH+8dioplatWlKsWM7lY1WrVjXbz5YrVy64N7fdyYsjueVsTtew0dCFKrmRm46AzorxpuPmRRN0NQdnFYi445CYPn36SK9evXLtUBfcq8a+kxdHcryd4p6MWGVjq6NIbuSmI6Cz8qIJupqDswpM3HEiHP5z7i0f3KvmTdwbjfxE0DXP8fYTHNnY6iKX3MhNR0BnxXjTcaO4e+RmdctnZWWZsfUCBQp4tAymmBdHZvoRr05PsNHQxSa5kZuOgM6K8abj5kUTdDUHZxVI5n7o0CEzke6xxx6TevXqybnnnptjfL1o0aJy1llnCf6fH1ciRy74ZqdcNmmZebRMPeKV4p6cyGRjq+NIbuSmI6CzSqQJulqDtQpE3DHmPmzYMNmwYYPr24V9Qt0Ln\/0ofV9cRXG3eY+Nre4XldzITUdAZ8V403GjuOu4hc4qkSO5n3xul7HR0IUxuZGbjoDOivGm45ZIE3S1BmsVSOaOV8K4O7rmp06dKljzjvPdcZZ78+bNpU6dOsG+teNu8RzJ89vdXcNGQxey5EZuOgI6K8abjhvF3Qe39957T4YOHSp\/+tOfzGY2o0aNko8++siIPda\/t2zZ0kdtyS0az5H28fYhbWrKkDY1knvzNK2NjYbOceRGbjoCOivGm44bxd0jtwMHDhhhb9++vTRq1EgGDBhg\/n766afLG2+8Ie+\/\/74557148eIea0xusXiO5OY1zNyTGW1sbHU0yY3cdAR0VhR3j9x27NiRLejly5fP\/jPWvK9Zs0ZGjx4t48aNEyyVS8aFCXyYmT99+nRT3dVXXy39+vWLuQNePEf2mfG1QOBxZfpJcHbfsLHVRSq5kZuOgM6K8abjRnH3yG3v3r1mjP3aa6+VmjVr5hB3jLtPmjTJdM2XLl3aY42xi2Fsf+LEiXL48GHp27evHD161Hw4YGtb\/N1tjX08R3IyHTP3PAelrQI2tjqa5EZuOgI6K4q7D26vvfaavPTSS9K7d2956qmn5Oabb5b9+\/fLww8\/LB06dJBu3bolZXOb3bt3y8CBA6V\/\/\/7SoEED84QLFiyQV155JWbXPxxpXfPnz8\/+86bdR6X9cyeW7zWpUkye7FTJxxtHuyiWNWIJIy9\/BMjNHy+rNLmRm46Ad6tWrVrlKLx27VrvxiEsGdhseWTQs2bNkjFjxsj27dsNisKFC5vu8p49e6ZsvH3fvn0mc69Ro4b5gHC7Yn2l2WfKv35jI7mgTv4cbBPCuOH2s0qnMAPVgSM3ctMR0Fkxc1dwQ3c5uulxYQJdKifR4UPiiSeekGbNmpmsHUMCfsSdO9PFdjAbW0Xwc09+HTRyIzc1AZ0hxd0nt\/Xr18uMGTNk8eLF5uz2P\/7xj9K2bVupWLGiz5q8F0ePwZtvvinPPfecTJ48WSpVyt21HsuRz3+6SW55+X\/mZtx2Nidzirv3GLSXJDdy0xHQWTHedNwo7j64Ydz71ltvNcvfmjZtaiw\/++wzwbgGZra3aNHCR23xM0pk6ziBzpp9v3XrVjOJ7+6773Y9lS6WIzmZjpl7UoLSVgkbWx1RciM3HQGdFcXdIzd0w2OSGzJ1+6lw2Knu2WefNZvaJGud+549e+SWW26RSy65RK688ko5cuSImciHDwncAz0GziuWI61jXlvWKiuz+zT2+LaZUYyNrc7P5EZuOgI6K8abjhvF3SM3+zp353nuqVjnjt6Ae++9Vz799FMzAx9Cj6V4VapUcX1iN0dy29n4zmWj4TH4HcXIjdx0BHRWjDcdN4q7R2448nX48OHSqVMnadKkSQ4rrHNH9n7\/\/fe7ZtUeb5GnYonEndvO5sbLRkMXcuRGbjoCOivGm44bxd0jN3S\/Q8SffPJJueaaa6R+\/frym9\/8xiyneuSRR8xSuDPPPNPUhkwbG84Eebk5cszb62TM29+Zx+BkOop7suKRja2OJLmRm46Azori7pEbuuUh4BhbT3Q1bNhQXn\/99UTFkvpzN0dy29n4iNnY6kKQ3MhNR0BnxXjTcaO4e+SGzH3Xrl3m2NdEV1gyd86Up7gnilXNz9nYaqgJN03SYSM3JTeKu09wGHv\/8ssv5YsvvpCyZcuaE+Jq1aolBQsW9FlTcou7OZIz5SnuyY2yE7VR3HVUyY3cdAR0VhR3H9x++uknsxxu0aJFZq37wYMHTUPXrl07ueeee1K6kU2ix3Q60j5TvkuzSvJ4l3qJqsi4n7Ox1bmc3MhNR0BnxXjTcaO4e+SGXeKwFewvv\/xiNpM56aSTjOXmzZtl1KhRZt93HCRTqFAhjzUmt5jTkfZtZzlT3p01Gw1dDJIbuekI6KwYbzpuFHeP3HC++qBBg8yucXXq1MlhtXLlSnnwwQfN6XBBz5K3HsTpSJzfjgl1uDhTnuLuMcw9FWNj6wlTrkLkRm46AjorirtHbpgtj61n77zzzlzinopNbDw+VnYxpyPty+C+uOtcqZ5VzG+VkS\/PxlbnYnIjNx0BnRXjTceN4u6RG06CGzlypMnMccSr1f2OWfQ4SAZL5EaMGCFFixb1WGNyizkdyZnyifmy0UjMyK0EuZGbjoDOivGm40Zx98Ft9erV0rdvX7N5zYUXXmgEHvu9b9y4UaZMmSINGjTwUVtyi1Lc\/fNko+GfGSzIjdx0BHRWjDcdN4q7T2441GX27Nkyb948M1seB8l07NgxX2fK4xXsjuSe8t6cykbDGydnKXIjNx0BnRXjTceN4q7jFjqrWOLOmfKxXcVGQxfG5EZuOgI6K8abjhvFXcctdFZ2R9qXwXEyHcU92cHKxlZHlNzITUdAZ0Vx13ELnZXdkTwwxpt72Nh648RueR0nciO35BDQ1UJx13ELnVUscWfmzsw92cHKjyIdUXIjNx0BnRXFXcctdFZ2R3IZnDf3sLH1xokZqI4TuZFbcgjoaqG467iFzori7t8lFHf\/zGBBbuSmI6CzYrzpuFHcddxCZ2V3ZNZt75vna1mrrMzu0zh0zxqWB2KjofMEuZGbjoDOivGm40Zx13ELnZXlSPsady6Di+8mNhq6MCY3ctMR0Fkx3nTcKO46bqGzshzJ0+C8u4aNhndW9pLkRm46AjorxpuOG8Vdxy10Vm7iztPgmLmnIlDZ2Oqokhu56QjorCjuOm6hs7IcyTXu3l3DxtY7K2buOlbkRm55J6CrgeKu4xY6K8uROMMdZ7nj2jH+j6F7zjA9EMVd5w1yIzcdAZ0V403HjeKu4xY6K8uRXOPu3TVsNLyzYgaqY0Vu5JZ3AroaKO46bqGzshzZaOQnghnzXAaX2EUU98SM3EqQG7npCOisGG86bhR3HbfQWcGRHyz+r0DccVHcE7uIjUZiRhR3HSNyI7fkEdDVRHHXcQudlVPcucY9sYso7okZUaR0jMiN3JJHQFcTxV3HLXRWcOTz7yyVyyYtM8\/GZXCJXURxT8yIIqVjRG7kljwCupoo7jpuobOCI0e99LFgtjzF3Zt7KO7eODlLkRu56QjorBhvOm4Udx230FnBkb0mvydj3v6O4u7RO2w0PIJyFCM3ctMR0Fkx3nTcKO46bqGzgiM7jn1Lpn28yTwbz3FP7CI2GokZsXtZx4jcyC15BHQ1Udx13EJnBUc2GPCqLPh2p1TPKmbEnVd8AhR3XYSQG7npCOisGG86bhR3HbfQWVHc\/buEjYZ\/ZrAgN3LTEdBZMd503CjuOm6hs4Ijd3aYap6La9y9uYeNhjdOzlLkRm46AjorxpuOG8Vdxy10VjUaNJfdl4wxz9WlWSV5vEu90D1j2B6IjYbOI+RGbjoCOivGm44bxV3HLXRWdnHnBjbe3MNGwxsnZu46TuRGbskhoKuF4q7jFjqr6s3byt6Wg81zIWtH9s4rPgGKuy5CyI3cdAR0Vow3HTeKu45b6Kzs4s5lcN7cw0bDGydmoDpO5EZuySGgq4XiruMWOqvK7W6Vg2dcZp6LW896cw\/F3RsnipSOE7mRW3II6GqhuOu4hc6q0l9GyuHq55vnYubuzT0Ud2+cKFI6TuRGbskhoKuF4q7jFjqrk7tNkaMV6nIDGx+eobj7gGUrSm7kpiOgs2K86bhR3HXcQmdFcffvEjYa\/pnBgtzITUdAZ8V403GjuOu4hc6qwo2vyLESFbiBjQ\/PsNHwAYuZuw4WuZFbngnoKqC467iFzirrtvfNM3F3Ou+uobh7Z2UvSW7kpiOgs2K86bhR3HXcQmdliTs3sPHuGjYa3llR3HWsyI3c8k5AVwPFXcctdFYUd\/8uobj7ZwYLciM3HQGdFeNNx43iruMWOitL3Lk7nXfXsNHwzooZqI4VuZFb3gnoaqC467iFzsoSd25g4901FHfvrChSOlbkRm55J6CrgeKu4xY6K0vcuYGNd9dQ3L2zokjpWJEbueWdgK4GiruOm8rqwIED8uyzz8qUKVMEf27RooXce++9Aic4LwhP9+7d5Ycffsj+Ua9evWTIkCGu97bEfcf4P6qeLRONKO46r5MbuekI6KwYbzpuFHcdN5XV888\/L1988YXcfffdUqpUKZk5c6YsWLBAHnjgASlZsmSOOpctWybvvvuuDBo0SAoUKJDwfhD36lnFzNazvLwRYKPhjZPbh2fNmjV1xhlsxXjTOZ\/cdNwo7jpuvq0OHTokQ4cOlY4dO0rLli2N\/bp164x4jx49WmrXrp2jzrlz58qWLVukR48enu4FcS+4f5uUfmeIzJ8\/35NNphfasGGDVK1aNdMx+H5\/cvONzBiQG7npCHi3atWqVY7Ca9eu9W4cwpIFjh8\/fjyEz5XjkfCIe\/bskWLFikmRIkXMzz788EOZOHGiPPbYY1KpUs7z15955hlZuHChrFq1SrZt2ybt2rUzXfLOctZNIO7cwMZfFDAj8MfLKk1u5KYjoLNivOm4MXPXccuT1bFjx2TevHly3333yYABA0w2b+96x88nTZokWVlZcsUVV4j19\/Xr15su\/OLFi+e6P8Xdv0vYaPhnBgtyIzcdAZ0V403HjeKu46a2WrFihYwcOdKMsd96663yu9\/9ztOYOibWDRw40Ng6u\/DxMBB37k7nzy1RCH5\/b5yc0uSm40hu5KYjoLOKQrylRbc83PPpp5\/KHXfcYf5r3bq1FCxY0NVre\/fulVmzZpkyVjc8xucxZj9mzBipVq2aa+ZOcff3SxCF4Pf3xskpTW46juRGbjoCOqsoxFtaiDuWvkGc27dvLxdddFFcb6Hs7bffLo0aNZKuXbtmd8vv27fP1FGoUCFXcS+x9Bkpsn6hLhJoRQIkQAIkECkCnFAXgDu3bt0q1113naBb3n5VrlxZpk2bZmZtQ9DPO+88ueqqq2Tz5s1moh2Wy2ESHsbe+\/XrJ+XKlQvgaXkLEiABEiABEshfAmmRuecvIt6dBEiABEiABNKLAMU9vfzFpyUBEiABEiCBhAQo7gkRsfspmVsAAA6gSURBVAAJkAAJkAAJpBcBint6+YtPSwIkQAIkQAIJCVDcEyJiARIgARIgARJILwIU9\/TyF5+WBEiABEiABBISoLgnRMQCJEACJEACJJBeBCju6eUvPi0JkAAJkAAJJCRAcU+IiAVIgARIgARIIL0IZKy4Yw\/6Rx99VJ577jkpXbq03HTTTdKtWzfX7WnTy6XJe9rPP\/9chg0bJt9++620aNFC7r33XsGey84LR\/J+9NFH5tS9\/\/3vf1KzZk257bbbzFG79hP7kvdk4a7JKzf7W\/z000\/Sv39\/ueWWWwzrTLz8cMPWoIhHnDlRsWJFs7U04y3+7+nRo0flxRdflMmTJ8uPP\/5oDt7CgVp\/+MMfMvL3NN7vGM4jGTVqlDmPBCeMpuOVkeIOMZoyZYpgW1scG\/vLL7\/InXfeKW3btjUNBC8RBDe29AWfJk2ayIcffijTp0+XBx98MNc2vl9\/\/bVpXEeMGCH169eX\/\/73v3L33XfL6NGjpV69ehmF0w83CwwaXbB69tln5YUXXshIcffD7eeff5YhQ4ZIp06d5JJLLjHH6CJWhw8fzniL83uKD6EJEybI+PHjBVt3f\/XVV+Z39qGHHnI9UCujfnH\/\/2V37NghS5culeeff1727NkjU6dOpbinUyBs27bNZJaDBw+WBg0amEefO3euvPfee6aRLVq0aDq9Tkqe9aWXXsoWaRy2s3v3bunbt6\/Zo79Zs2Y57vn6668LBB4n9iFTP3TokBF3fBRgr\/9Muvxws7i89tprpqHduHGjdO\/ePSPF3Q83fGj++9\/\/Npk7flfxsY6GGOdIFClSJJPCTfxwW7JkiYwbN85koziPY9GiRab3EmJvnaCZUfBcXha9RwsWLDC\/i+ixpLinWUSsWbPGiDgC3epyWbZsmelWRpdVunbDJNMNaABq1KiRLc6HDx82XfSNGzfOJdg4iQ+9HyVLljSPgIN7br75Zundu7dccMEFyXys0NflhxteZvXq1aaBRfcosqouXbpkpLj74TZx4kQ5cuSIoGv+7bffllq1apnMPRO7l\/1ww+8ohiFHjhxpfo\/wYYQDtnA8Nq+cBNDLAbYU9zSLDDfHuQl+mr1W0h7XOjbXOmXPqtjZkLjdEF2kGOKoXr263HXXXdmCn7SHC3FFfrlh3gcaWvRunHHGGUagMlHc\/XJDHL7zzjsyduxYOfvss2XVqlVm+AhjpPj4zJTLLzf0eEyaNMkkMfhwxzwZsITA4wOJ168EKO5pGg2xMneMJz\/++OM8GlbE\/NK7Ze7nnHOOdOzYMZfnMQ768MMPmy4tZO1t2rTJyOENr9zQlYxxPVyYyHnw4MGMFXcw8MrNKoss1BoGAsv777\/fCFSmDQP55YbJhz169DBxF683Lk2b9qQ9NsU9aSiDrQgzRTHmft9990mdOnXMzTHm\/p\/\/\/MdMysm0cTs3+s8884zpXrcaUIy533rrrWbc3ZkdWRMTMfsWPz\/ppJOCdWiI7uaVm5V1zZkzJ9fT9+rVy0wYy6TLKzcwwYxvzPHAvA7MB7HEHZM33T48o8zRDzd8CNjFHRM5MaGuefPm8uc\/\/znKmHy\/G8XdN7JwGCCokaEjW8LSI0ywGzRokKBRzbQx4lgewWQSdBPjAwhdxq+88or5+EGXnjW2btliUg+WwGHGPBrbTL78cLNzssQ+E7vlwcEPN6ssJsQ2bdrUdMujSx5DHOhtyqTLDzd0yz\/yyCNmrhE4LV682PzZ6qbPJG6J3pXinohQiH+OTBRjTegarVChghmzu+yyyzJenCyXIRvCzNF77rnHNLyYdIOx9CpVqpgiEPSPP\/7YdIeiO\/7pp5\/O5W38LNO6Sb1yQ4NavHjxbGaZLu5+ua1YscLEHhphrN7Ah2gmjbf7\/T1FvGECHSYgYgInPsaxnwJ65s4888wQt9T582gU9\/zhzruSAAmQAAmQAAnEIZCRm9gwIkiABEiABEggygQo7lH2Lt+NBEiABEggIwlQ3DPS7XxpEiABEiCBKBOguEfZu3w3EiABEiCBjCRAcc9It\/OlSYAESIAEokyA4h5l7\/LdSIAESIAEMpIAxT0j3c6XTncCOJoSGwy99dZbZq8GP2fAY\/tlbACDvdlr164dWhTYUe2JJ54wx5NOmzYtpc9q3zGwYcOGaX1gSGgdygcLlADFPVDcvFk6EvC7oQWEt2fPnmYLWT+i64cNngnijh3HsNuYny2T00nccRQpNpcqVapUSjeYso6N\/eyzz8zulel8GpifOGLZ6BKguEfXt3yzJBEIq7hrj6RMtrgfO3ZMChQoYP5L5uXlFMJk3g91+fV1su\/P+kggWQQo7skiyXrSmgAOycF2xDNnzjTvcckll5gtTX\/44Qfp2rVr9ru98MILZj9zdIfj+Exs41m+fHm5+uqrzWlbOLcAWfvy5cuNjXUIDE7NQ\/2vvvqqOdMAB5z069cveztfJzxneTwPzj\/AUbrY+hfbhloXnsnZQ4DnmD17tslCcQwvDvXBmfE48\/ybb74x3fJ43hkzZhhBq1u3rjk0CVu54vrvf\/9rthX+4IMPjGi3atXKbNGMk9esnom\/\/e1v8t5778lvfvMbsz85GGJLWNiA0fXXX2+GDHDmALr\/0fX97LPPCg472bVrl1x44YXmbIf69eu7xo5d3K1ucxzxun\/\/fpkyZYrprbjpppsMe2yt6rysLZLtW\/26\/ZvdjuKe1r\/GfHgbAYo7wyHjCRw6dMgIG06zu+666wwP7JW\/ZcsW8+\/YYx\/d39iTG3vrY19znEh27733yumnny44FQ9n17dv316uueYa+f7776V\/\/\/7m6FtLdFE2KytL\/v73v0vhwoWNQKNeHNyBk7rsF855x57+xYoVkxtvvNEI16xZs4zNk08+acagFy5cmOOZ7N3y6GLGRwoEEGJbs2ZNeffdd01XM\/4N17XXXiunnnqqeW7UBxFGnagfYo1nh6C3a9fOnLoGHuvXr5eHHnrIHBWKDxicz4D6cbDQnj17pHfv3uZ0sQ4dOgg+TvAOOIERtngGfNzgQwOnC+LwIXwYPPfcczHPE3cT90WLFpmPrvPPP9\/UhY8cHJmL\/5w9BxT3jP\/VzmgAFPeMdj9fHgQgRDfccIP85S9\/kc6dOxuRgHCh+7pBgwaybNkyc964NQ4LUd64caNceumlOY4chShinN055o7TuKZPn26EsXTp0gY67tmnTx\/zMXHRRRflcASyR3xUPPXUU9mZvZW5YrIXMu54GSZOOcT7QMDxwYEL9phAd84555jsGx8N+HuTJk3Mz1evXm16BvCMGN\/GASOwxQcJrgULFpgPETDABXHHc1j1Q0jnz58v48ePzz41cMmSJaaHAB8U6EnABw5+bh0+hI8qfCTh48A6Y9wOwk3cy5Ytm33UK8riyFycWDhx4sRstlYdFHf+fmcyAYp7Jnuf724IIDN98803jaAii8axv+gGx2lZOMLWKaQov27dOnMELv4P8Uc3NgTKTdwhPBA1t8vt5LxYXccQO1y4Rzxxx0cJuuAffPBBqVOnTq7buo25O\/8NHyiffPKJOU515cqV5r+qVavmEHf7hEH7s1k3RNaODwZk8Ph4wDCE2xXr\/Ho3cT\/vvPNynDQI9uh9wAdEtWrVclRPcecveCYToLhnsvf57jkIoLv5q6++MqKGjLBcuXKm6xtH3lqZO\/4NXd7ooseY81lnnWUyYWTZsTJ3ZLyoA0fm2o95xc3xd+e\/uYkSPijwIWDdI5G4x1vqlkjccS+MZbds2dJ86Jx22mlm7gE+UOyZeyJxhw0+MnDOOj4OMNYObmXKlMnBHUMK6KZ3Xl7EHb0o6BFA1z+GGeyXG0c8A+ZDOI\/ctew45s5GISoEKO5R8STfQ00AY7cQrttuu82MDePCODq6tjEZ7JdffskWdwgxxnzPPfdc+etf\/2rKWt3LmFjnlrnPnTvXjGVjzTaWduFCt\/yoUaPMBwK62u0XxqLxUeEsj278yy+\/3GSu8UQIGTO6zSGsVpe\/fRgAGa5T\/O2Cv3TpUtMNDwHEPARc+NiBMMYSd\/ROoA67aGI4YtiwYUZ4MY8Ak+eQYderVy+bGz58MEnuT3\/6kydxR5c+egOs8XW34QCrIqe4Y2hgxIgRsnPnToq7+reFhulCgOKeLp7ic6aMAIQHQoiZ6BgDh3BgAtrrr79uJntBLJF1Y4wa4g+B2LdvnxHIggULmjFfjKljDB7\/duTIEenbt69Znw3Rwng3Jqhh\/B7142MAIocPCIgbxrjtF4QY5SHC1oQ6CBV6FDCD\/eSTT44r7si8UT8+EpA1o7cB9pgwh\/eBuMUTd3Sh42PHmoy3ePFimTBhgulhgD2ybOc6\/q+\/\/tq8Gz6IMAkPk+9wb7wjNqBBlz7G1\/HuuDcmCeKZ5s2bZ1YdWOPwdg5umTtm4o8ePdp8XGFiI4ZS8NHTqVMnM66PiX34IEFvAD4u8B5YMYD6P\/roIzPREbP08RGCCYsoDx9aPQfM3FP2a8aKAyZAcQ8YOG8XTgKYIIcG\/5133jFj8JjljgwR4+4QW8zKxs+w7Ayii6wbwoQuecwSh4BiFjgyfcwWR5aLjwFrHB7LxCBW\/\/rXv4yoXHHFFWYMGnZuV6yleZYIJhIht2Vn6FXA8ybqlocQY8maNbMeM9ExcQ7vXKFCBfMhhHe2d8uDGT4CMCEPXeXgh+V+4GXthIdJisjwIepYDmgtN3QTdjBxE3csqcN8AHxM4VmwPA8fUZgbgffCJEI8A+5vMcB74H5YMocJglYPA+6BXhisFsC74ErENZzRy6cigdwEKO6MChIggTwTgJCixwK9EFaXOcbZ8UGAcXbncj8vN\/Qy5u6lHj9lKO5+aLFsmAlQ3MPsHT4bCaQJASx7Q\/c4ut7RZY5NajDkgEwb\/47M2u8Fcbe2n8XeABgacc6W91tnrPLcfjZZJFlPWAhQ3MPiCT4HCaQxAWxBi7XxGD\/HigNMLsSQRPfu3XOtBvD6mvaDYyZPnmwm5qVK3HlwjFevsFy6EPg\/Go4SrdeKz2EAAAAASUVORK5CYII=","height":303,"width":503}}
%---
