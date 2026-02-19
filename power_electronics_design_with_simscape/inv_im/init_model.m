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
if use_mosfet_thermal_model %[output:group:19e1bf01]
    set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
    set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
else
    if use_thermal_model
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'off');
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    else
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on'); %[output:82a912c2]
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    end
end %[output:group:19e1bf01]

if use_torque_curve
    set_param('inv_psm/fixed_speed_setting', 'Commented', 'off');
    set_param('inv_psm/motor_load_setting', 'Commented', 'on');
else
    set_param('inv_psm/fixed_speed_setting', 'Commented', 'on');
    set_param('inv_psm/motor_load_setting', 'Commented', 'off');
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAfcAAAEvCAYAAABYGIhlAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnX9wVsd57x9aWkse44BsakeWkXAMCfFksJM4JbJbyFVzSSaB5uLWIJIZosGEEjPONGKQkJnBTG394EaZmmJzFVuh3EkkO065t9ZMp9TBQXMdhTaObf5waSERAis41y7CEzwVuaHlzrPSvj56eX+cH7tn9znv98wwQq\/2PLvn+9k933f37NmddeXKlSuEAwpAASgABaAAFMiMArNg7plhiQuBAlAACkABKKAUgLmjIkABKAAFoAAUyJgCMPeMAcXlQAEoAAWgABSAuaMOQAEoAAWgABTImAIw94wBxeVAASgABaAAFIC5ow5AASgABaAAFMiYAjD3jAHF5UABKAAFoAAUgLmjDkABKAAFoAAUyJgCMPeMAcXlQAEoAAWgABSAuaMOQAEoAAWgABTImAIw94wBxeVAASgABaAAFIC5ow5AASgABaAAFMiYAjD3jAHF5chU4NSpU9TS0kLnzp3LXcCqVauou7ubqqury17U5OQktbe3U3NzMy1btqxs+mPHjtH69etnpNu8eTO1tbVR1FhlM0MCKAAFUlcA5p665MgQClytAJv79u3bac+ePbRo0aLIBhvVkNnce3p6qL+\/n2pqanL51dbWKoPHAQWggGwFYO6y+aH0GVGgnLkHe9q6h82Xzgbd19dHy5cvV0rw37jn\/uyzz9KOHTtyn+Ubdr65c0IuQ2dnJz366KPqSwaPAixdulSNCAwNDalYejSB\/68\/589+9atfUUdHB73yyis0MjJCZ8+epXXr1lF9ff2MEYKBgQFavHgxtba20h\/+4R\/SX\/zFXxB\/ofjGN76hruX48ePU1dVFa9euzQhZXAYUcKMAzN2N7sgVCsxQoNCwvDa5oPHX1dUpU21sbFTGqXvf58+fV8P6bJJ8DA4OqiF9bcL5w\/WFzH1iYkKZ7te\/\/nV6+umnlbnfeuutKsYtt9xC+u9BE+c82JC3bdtGBw4cUOb+zDPP5EYEOB\/9mIC\/cIyNjdGmTZto48aN6nP+0sHXwOl4FOGFF15QXw7CPo5ANYICUKCwAjB31Awo4IECxXru2sS1WfPzd22Sutj5z8nPnDmT67XrNMHePn8W1tzZgPWQP\/feuZe9f\/9+Zf5cNu5hFzN9PVcgf9RBmzuXW48ysOnz75w2eK0eoEERoIBIBWDuIrGh0FlTIN\/c+fq0ifOQe1Rz12ZZTKeww\/J8Pk+8Cw6n6559OXPXowb8k3vizz\/\/\/IyeO8w9a7UY1+OTAjB3n2igLBWrQKme+0c\/+tHcZLuww\/J6mDyYPvgcu9SEuoceeig3856B6C8W+cPvevi82OfBRwL62T33\/NFzr9hqjgtPUQGYe4piIysoUEyBcq\/C2ZhQF+ZVOJ78xs\/H2cA5\/cWLF6+aaFdoQp1+Zq4n9rGpc5zXXntNfVHZunWrGobHsDzaBBSwowDM3Y6uiAoFKkaBQkP8FXPxuFAo4KkCMHdPwaBYUMBnBYKv2nE5+Zl8mMVzfL4mlA0KZEkBmHuWaOJaoAAUgAJQAAoQEcwd1QAKQAEoAAWgQMYUgLlnDCguBwpAASgABaAAzB11AApAASgABaBAxhSAuWcMKC4HCkABKAAFoADMHXUACkABKAAFoEDGFIC5ZwwoLgcKQAEoAAWgAMwddQAKQAEoAAWgQMYUgLlnDCguBwpAASgABaAAzB11AApAASgABaBAxhSAuWcMKC4HCkABKAAFoADMHXUACkABKAAFoEDGFIC5ZwwoLgcKQAEoAAWgAMwddQAKQAEoAAWgQMYUgLlnDCguBwpAASgABaAAzB11AApAASgABaBAxhSAuWcMKC4HCkABKAAFoADMHXUACkABKAAFoEDGFIC5ZwwoLgcKQAEoAAWgAMwddQAKQAEoAAUyp8Dlt8do9vyGzF1X2AuCuRPRbbfdFlYvpIMCUAAKQAFPFbj5dy\/T0jmX6L\/ecJFuuuYyHb9YRf99bH6s0o6OjsY6z5eTYO7T5i4ZJH85kVx+XxpD0nKAQ1IFk58PBsk1NBEhbQ7cS7\/4w4N08ehfq+LPWfFlmvOpDbF77mmX34Tm+TFg7jB3G\/WqImNm4YYgHRwY+EEwLQ7a1C8894gycjb1effvSixCWuVPXNASAWDuGTD306dP08KFC23WE8QOoQA4hBDJchIwsCxwyPC2OVx6\/ShNPLeb+CebOhs6G7upA+ZuSknHcaSDtN2QHOMRkz04uEcFBu4ZcAlscOBe+uTrR+ni0YN0+a0xqr5jBc1ZsYGq7lhh\/KKlewILgp47eu7GG0alBrRxQ6tULeNeNxjEVc7seSY5FBp6T\/I8PcyVwtzDqCQgjXSQJhuSAFzeFhEc3KMBA\/cMTPXcbT1PD6OQdE+oqJ77sWPHaP369YprV1cXrV27NsdYOkjc0MI0V\/tpwMG+xuVyAINyCqXz9yQceMY7D73bep4eRgHpnlAx5j4xMUGtra3U0dGhuHZ2dlJvby\/V1NSo36WDTNKQwlR0pAmnADiE08lmKjCwqW742FE56OfpF763W2Vi83l6mKuQ7gkVY+7ca+\/p6aH+\/n6qrq6m9vZ2am5upmXLlsHcw9R0pAmlQNQbWqigSEQ9h8eUCmcnJtXPNyYuTf1+Yfrn9O+lpFpQU0XNd7+f2lZW7oplaValsG3BxfP0MDrA3MOo5EEaNvfBwUHq7u5WpWFzb2xszA3N3\/hnz+VKefPNN3tQ4mhFuHz5P2j27N+OdhJSG1cgqxzO\/eqyca3CBqy9fnYu6fvnTP0\/+FntnNn0\/uk0q5ZcR+Pj41RXVzcj\/NCJd+nNX12ml39xid68eJn4ejZ\/Yi59fsl1M2KFLRPSlVegEIcZZ10YpysvPE708t8Qzasj+vh9NOvTXysf2HKKpqamXA7SFwariNny5cxd+re0sN+SLbeLig8PDu6rQBgGPBIw+JM36ezEJbr3A3Pp+Qfvcl\/wjJWgEIfgq2z8PJ1fYat95IdeXrl0T2BRvTX3Z599lnbs2DEDfP5EuLC1AsPyYZVCuiQKhDGWJPFxbnkFojBgc1\/95Ksq6L51S+je2+eWzwApQikQ5JC\/NKzr5+lhLgDmHkaliGn0rPZCRq4Nf2BgIPe8PEx4TKgLoxLSJFUgirEkzQvnF1YgKgM2+MGf\/JJ6Dp9GL95gpWIOt143S633Hlwa1vb76aYuAeZuSsnpOGzCQ0NDtGHDhpKRDx48WDZNfoDgq3D5Xw6kg4x6QzOMDeGmFQAH91UhLoOXfvaO6sVjmD45Q+6pn\/32NvU83eR678lLFj6CdE\/gK\/V2WD48huQppYOMe0NLrhwiBBUAB\/f1IQmD4DD9azs\/6f5iBJUg\/3k6T5Cr\/fxWK0vDpiGLdE\/w0txPnTpFLS0tdO7cObXYDB\/87L22tpYOHDhAixYtMs5WOsgkNzTjYlZwQHBwDz8pAzb4rYMn1Gt2eA5fnmex5+lvXlsvejMr6Z7gnblPTk7m3kFfvHgxbdy4kdatW6deWePn7SMjI+p1Nn5X3eQhHWTSG5pJLSs5Fji4p2+KweonXqWXfv4OPf\/VuzDRrgDWckvDmuLgqkZJ9wTvzJ2fue\/evZt27dqlVo\/jhWeWL1+uJs\/l\/80kdOkgpTckkyxdxgIHl+pP5W2SwYODJ9RkOxj8e1zZ1HkVOV4ittRWqyY5uKhV0j0B5j5da6SDlN6QXDReG3mCgw1Vo8U0zYDfieeZ9E80L6Hmu+UtcBVNvcKp42y1apqDieuIEkO6J8DcYe5R6jvSllFA+g0tC4BtMODeO\/fiK83g85+nz1nxZQr7KpsNDmnWT5i7YbV56J2fsx8\/frxg5KVLl6r14fWGL6aylw5SekMyxdF1HHBwTcDssHzwavSrcm0rF2Z+ffpyz9PDUJbeFqR7gnc99zCVxkYa6SClNyQbTF3EBAcXqs\/M0yYDbfA8PM+9+KwdvCTsxHO7c1utLnjydOxLtMkhdqEinCjdE7wzd\/TcI9S+QFLpDSneVft3Fji4Z2KbQdYMPs7z9DCUbXMIU4YkaWDuSdQrcy6\/+jY2NkZtbW0qJf\/OB78WZ\/qQDlJ6QzLN01U8cHCl\/Hv5psEgC6vZ2d5qNQ0ONmubdE\/wrueuYRV67Q2vwhWvytIbks1GmmZscEhT7cJ5pcVAssHzq2zB9d7n3b\/LOLi0OBgv+HRAmLslZfViNsE91\/mdd161DovYXC269IZkqRqlHhYcUpf8qgzTZMCr2d356I9FrEfP76VfPHow9zydDZ1nv9s60uRg4xpg7jZUnY6Z\/\/zd1kx5zk46SOkNyWI1SjU0OKQqd8HM0mag16NfMK\/Ku33hXW61mjYH0zVPuid4OyyfBFRwH\/jgF4LgrnD528lKBym9ISXh7dO54OCehgsGwfXofdhwJv95epJZ73GJuuAQt6yFzpPuCd6Ze9ItX3nTmc7OTurt7c0tX8tD+du3b6edO3dSR0eH4hhMg567ySZR2bGk39CyQM8VAx82nMlfGpaH3W08Tw9TT1xxCFO2MGlg7mFUiphG97Dze9ccRvfK8\/djL5YFxxocHKQ1a9bQX\/7lX6oFcHjTmfb2dmpublZr1sPcIwJC8qIKSL+hZQGtawa84UyaO8rlb7WqVpFbscH5VquuOSStyzD3pAqWOD84vK6TFTL8UkXgSXgNDQ1UX1+vTJ4n4\/HB5h6crCcdpPSGZLEapRoaHFKVu2BmPjBIY0e5Qs\/T5z94wD2A6RL4wCGJGNI9ga991pUrV64kEcHXc4PvyesefClz19dx5MgRXy+paLnGx8eprq5OXLmzVmBwcE\/UFwaP\/ODfaOjEu\/StNTfTx26pMifMhXG68vLfEL3wONG8OqKP30ezPv01c\/ENRfKFQ9TLaWpqyp0yOjoa9XSv0os2d37G3tLSol6RW7VqVe41Od1j1wvesLnzZxiW96ruZa4w0nsrWQDiEwOTW8am\/Spb0rrgE4c414KeexzVLJ8T3ANeZ8UT9VpbWzGhzrL2lR5e+g0tC\/x8Y5BkR7lCS8P6NPReqr74xiFq3Ya5R1XMcvrg6246K92j553m1q9frz7On5AnHaT0hmS5WqQWHhxSk7poRj4y0AYfZcOZ\/FXkwm616p7AVAl85BBFG+mewNfq7bB8cBGbp59+ml588UXasGEDLVq0KAqjUGmlg5TekEJBEpAIHNxD8pVBmOVqfXqVLSlJXzmEvS7pnuCtuQeXn+XNY5YvX66Y6Bnv\/DqbyUM6SOkNySRLl7HAwaX6\/vcYCxm8rV3ZXJOQ3hake4K35h7cJOapp55S5r548WLavXs37dq1Sy1QY\/KQDlJ6QzLJ0mUscHCpvv\/mziXU69H\/acOvqfvmf7S+gYsrItLbgnRP8NbcC\/Xch4eHsXFMkZYqvSG5ugGZzhccTCsaPZ7vDHjW+z8f6qP5bx6jc7NvVgvOLN08tf5Glg7fOZTTGuZeTqEEf8fGMeHFk96Qwl+p3ynBwT0fHxkU28Dl\/qPvS3U1uzTp+MghyvXD3KOo5XFa6SClNySPq0akooFDJLmsJPaJQZgJcmmsZmdF6DJBfeIQ5\/qle4J3w\/L5vfV8KLa2fZUOUnpDitP4fDwHHNxTcc1A99In\/\/mo2ju96o4VVPvID0sKY3KxG\/cEpkrgmkNSHaR7gnfmHgQSXD6WP+ff+dCrziWFFzxfOkjpDckkS5exwMGl+m5NhY384tGDxM\/UZ89vILWBy6c2qP+HOXoOj1HP4dP0RPMS4vfhpR\/S24J0T\/DW3IOz5fXM+EKfmWoA0kFKb0imOLqOAw6uCaTbYww+S+f\/cy+dJ8ixscc54ix2EyefNM6R3hake4K35h6cLa976rysLK8hz5u\/4D33mc1TekNK42aTRh7gkIbKpfOwzSB\/i1XdSze1b3qYxW7cq1y+BLY5lC9BshQw92T6lTwbs+XDiyu9IYW\/Ur9TgoN7PjYYFDL06jtWkK113rNg8DY4pFm7YO5pqm0xL+kgpTcki2hTDQ0OqcpdMDNTDNI29PyL0YvdLKipotd2ftK9sBFLYIpDxGyNJZfuCd4OyxebNY\/Z8oXrrvSGZKxFOg4EDo4BJJilzWZ++a0xmnx9mPRMdx5y5x561R3LYz9HT6IIG\/zWwRPqXXhpBi+9LcDck9TciOfybPn6+npatmxZxDPLJ5cOUnpDKk9IRgpwcM8pCgOe4R40cy69NnSeGMcT5Hw4+F14Nvh965bQvbfP9aFIZcsQhUPZYA4SSPcEb3vuhVhGnS1\/6tQp2r59O+3Zs0ftJBfcDrarq2vGK3XSQUpvSA7arpUswcGKrJGClmLAr6lden2YfvP2mHoHPWjmtp6fRyp8icTSFruR3hake4Ioc2dz5hnz\/f39ZTeO0bPtf\/rTn9KBAwfohhtuoNbWVuro6FDNp7Ozk3p7e3NxpIOU3pBM3QBdxwEH1wSmXoW78cxwzsR5qJ2H3LWRz\/69Bvqd+Q3OhtqTKKQXu5n45qeShEnlXOltQboneGvuxZ655\/e4i9VSHsJ\/++23ic2dDf38+fO5Lwb8Gl17ezs1Nzfnhvilg5TekFK526SQCTiYF5l72\/q4\/NaZnFFz75uPoHnrdDy0rk3c9x55VMX0u\/BtKxdS28pwC+REzcNEeultQboneGvuSSoXD8cfPHiQtmzZQjt37syZu94LnmOzuTc2NuaG5v\/XZ97bH37pnXcmyd7Jub\/+9a\/pmmuucZI3Mn1PAbEcJsbtYrxgIP68uqky1kz\/nHfL1O\/z6mgW\/+3j96lfx8fHqa5uOo3dq3IW\/ae\/uERfOfRLWrXkOnrkj250Vo5SGUvl0NTUlLus0dFRL7UNW6hZV65cuRI2cVrp4q5Qx8Pxjz32GG3YsGHGUDz33EuZ+\/7\/Mof+5L4\/SevyjOdz8d2LNOe6OcbjImA0BaRyCLtEahg1TC3mEiavQmmk9xjDXrd+F97XIXrpHNBzD1sTQ6bTz8qHhoYKnrFq1aoZK9RxL72lpUWtXMd\/e+CBB1SPnX\/XR21tLf35n\/85fec731HP6zEsHxIGkkVWQPoNLfIFe3hCJTHw+V146Rxg7pYad9SZ8YWKwTH0JDpMqLMECmFnKCD9hpYFnJXGIPguvE+vyknnAHM3fDfQpv7QQw\/Rtm3b6Pjx4zNyiLKITdDc81+FGxgYmPG+vHSQ0huS4WrkLBw4OJM+l3ElMvBxsRvpHKR7AjcIL5+5p32LkA5SekNKm7et\/MDBlrLh41YyA5\/ehZfOQboneG3u\/Drbjh07Yvfcw98OiKSDlN6QorDyOS04uKdT6Qz0u\/Cu94WXzkG6J3hr7vo997a2NivLzebfgqSDlN6Q3FuCmRKAgxkdk0QBAyIf3oWXzkG6J3ht7rt376Zdu3aVXY0uyY1AnysdpPSGZIKhDzHAwT0FMJhi4HrbWOkcpHuCt+bOBeNheT7Wrl1r\/Y4hHaT0hmQdcEoZgENKQpfIBgzeE8elwUvnIN0TvDV3bPka7SYpvSFFu1p\/U4ODezZgMJOBq3fhpXOAubtvy0ZKIB2k9IZkBKIHQcDBPQQwuJqBi3fhpXOQ7gnieu5cYF5xjnd643fXTR3SQUpvSKY4uo4DDq4JTO0Kt3DhQvcF8bAEae4LL52DdE\/w1tz1M\/exsTHiGfP6d\/5ZX1+v1ol\/\/PHHjTUf6SClNyRjIB0HAgfHAAjmXo5AWu\/CS28L0j3BW3MvtXEMr163d+9emHugFUtvSOVuSFL+Dg7uSYFBeQY9h8eo5\/BpsvkuvHQOMPfy9ShWCr2BDA\/BB3vuIyMjtH37drUJjP48VgZ5J0kHKb0hmWDoQwxwcE8BDMIx0O\/CN999szJ504d0DtI9wdueOxcsf8Y8ryu\/b98+2rNnz4y92E1USukgpTckEwx9iAEO7imAQXgGNl+Vk85Buid4be7hq2jylNJBSm9IyQn6EQEc3HMAg2gMbBm8dA7SPcFrc+\/p6aG+vr4ZNTXMrnDBPeGDM+uPHTtG69evV\/G6urpmLI4jHaT0hhTtduRvanBwzwYMojPgV+VWP\/mqOvG1nZ+MHqDAGdI5SPcEb809uF3rK6+8ombInzlzRlWhcivW8ZeChoYGlY4NfXh4mDZt2pTb251jdHZ2Um9vb25pW+kgpTckI3cTD4KAg3sIYBCPgel34aVzkO4JXpu7Xlv+5MmTOYMut958oVn2fJFs8mz6\/f39VF1dTe3t7dTc3JzblEY6SOkNKd7tyL+zwME9EzBIxsDUq3LSOUj3BG\/NnYfW+XU37nGfP3+eWlpa6Ny5c1RuWF6bO1\/Y0NBQbsEbjsHvxnd3d6uaz+be2NiYGwWQDlJ6Q0p2O\/LnbHBwzwIMkjPQ28ZOfPNTsYNJ5yDdE7w1dy7Y8ePH6dprr1Ur0XHPe9u2bWVXpjt16pT6IvCNb3xD9cp58xl+fW7NmjV06NChkuaua\/GRI0diV2hXJ46Pj1NdXZ2r7JHvtALg4L4qgIEZBt\/6x3eo75\/eoc2fmEtf+f25kYNK5dDU1JS71tHR0cjX7dMJs65cuXLFpwJFKYs2c+7Vr1q1Sr0Dv3PnTuro6Mh9KeDh+C1bttD+\/fsxLB9FXKSNrID03krkC\/bwBDAwByXJu\/DSOaDnbq4eGYsUnFCne+5B0+eMMKHOmNwIFFBA+g0tCzDBwCzFuK\/KSecAczdbj65auCY\/fLln7pw+uPhNMH3wVbiBgYHcZDo+RzpI6Q3JcDVyFg4cnEmfyxgMzDOIY\/DSOUj3BK4FXg3L569Kl2\/C5qvtVETpIKU3JFtc044LDmkrfnV+YGCHQdRX5aRzkO4J3pl7sFoWWn6WX2WrqakxXnulg5TekIwDdRQQHBwJj0cjqQgfxeCltwXpnuC1uefX1uC76qYNXjpI6Q0plTtTCpmAQwoil8kCDOwzCPMuvHQO0j3Ba3MPzoTngvJseH5PnRehMX1IBym9IZnm6SoeOLhS\/r18wSAdBvpd+GLbxkrnIN0TvDP3YpPhbFdX6SClNyTbfNOKDw5pKV08HzBIj4HeF75t5UJqW9kwI2PpHKR7gtfmXqiKhpktH6dqSwcpvSHFYebjOeDgngoYpMtAz6TP3xdeOgfpnuCduadbLd\/LTTpI6Q3JFXfT+YKDaUWjxwOD6JolPaPQq3LSOUj3BJj7dK2WDlJ6Q0p6c\/HlfHBwTwIM3DDgmfR3PvpjWlBTpbaNlc5BuifA3GHubu4EGc1V+g0tC1jAwB3F4KtyO1fMpT+5d4m7wiTMGeaeUEBfTpcOEjc0P2oSOLjnAAbuGfCrcqNvv0v\/40sfoXtvj77pjPsrkL+wGXru6Ln70I4yUwYYi3uUYOCeAZfg070\/pp\/+4hI9\/9W7RBq89A4fzB3m7sedICOlgLG4BwkG7hlwCZjD907Oop7Dp6nYu\/B+lLRwKWDuPtOJUDbpIHFDiwDbYlJwsChuyNBgEFIoy8k0hyTbxlouYsnw0j0BPXf03F22n8zlDWNxjxQM3DPQPfeFCxeqwsTZVc71VcDcXRMokH9w2dpiW752dXXR2rVrc2dLB4kbmh8VERzccwAD9wzyzV2iwUv3hMz13CcnJ6m9vZ0aGxuVeff09KiavmnTJmptbaWOjg71e2dnJ\/X29uZ2mJMOUnr5\/bgdJS8FOCTXMGkEMEiqoJnzC3HgV+VWP\/mqyoDfhff5yEI98mo\/dxOw2dAbGhpy5s7\/r6+vV0bPW8byxjP8BaC5uZmWLVumspQOUnr5TXD3IQY4uKcABu4ZlLqnRtk21uWVZKEeZc7cuUKwkff19dHAwIAycN4udnBwUO0qx0ewd68rosuKhLyhABSAApWkwLv3bqfLN36Q5v7vjd5e9ujoqLdlC1OwTJm73lWura1Nmboell++fHlJcw8jFNJAASgABaCAOQV421h+TQ6HHQVEm3tw8hzv9\/7AAw\/QN7\/5zdzzdO6xs8Fv2bKF9u\/fX3RY3o60iAoFoAAUgAJQwI0Cos09X7L8nvuzzz5LIyMjtH37dtq5c2fRCXVupEeuUAAKQAEoAAXsKJApc2eJwrwKp5\/F25EUUaEAFIACUAAKuFUgc+buVk7kDgWgABSAAlDAvQIwd\/cMUAIoAAWgABSAAkYVgLkblRPBoAAUgAJQAAq4VwDm7p4BSgAFoAAUgAJQwKgCMHejciIYFIACUAAKQAH3CsDc3TNACaAAFIACUAAKGFUA5m5UTgSDAlAACkABKOBeAZi7ewYoARSAAlAACkABowrA3I3KiWBQAApAASgABdwrAHN3zwAlgAJQAApAAShgVAGYu1E5EQwKQAEoAAWggHsFYO7uGaAEUAAKQAEoAAWMKgBzNyongkEBKAAFoAAUcK8AzN09A5QACkABKAAFoIBRBWDuRuVEMCgABaAAFIAC7hWAubtngBJAASgABaAAFDCqAMzdqJwIBgWgABSAAlDAvQIwd\/cMUAIoAAWgQEUqcOF7u2Nd9+W3x2KdF+Wk+Q8eiJLcu7QwdyK67bbbvAODAkEBKAAFTCiw8oZ36aZrLqtQN\/3ub3Ihb\/7dqc\/U59N\/5\/8HPzeRf36MX\/6\/2YnD\/t9fJ49RrhD\/7e8nyyXx+u8w92lzHx0d9RpUqcLxlxPJ5RcrfF7BwcE9yawzuHj0r5XIl14fVj9\/M92DvfzWGBXrzc6e35ADM\/v33vv\/7wQ\/n5GmXqWfs+LLsYFK5yC9\/AwO5g5zj92AceJMBbJwQ5DOVDIDNmc2aTZsNm\/+Wci0tVmzUWuDrrpjeSIzNs1dMgfWQnr5Ye7TNVo6yNOnT9PChQtNt0\/Ei6gAOEQUzEJySQy4F65N\/NLrR9\/rXU\/3oqvvWKE+k\/jsVxKHQtVQuifA3GHuFm6vlRtS+g0tC+R8ZKB75JOvD9PkPx8lbeS6B84m7lvPO2ld8JFDlGuCuUdRy+O00kFKb0geV41IRQOHSHJZSewDg1Jmro2ch9OrpnvmVoRwHNQHDkkkkO4J6Lmj556k\/uPcPAWk39AIJ6qYAAAePklEQVSyANQVAzb0yde5Vz5MetIb98zZzCUOqyetC644JC23Ph\/mbkpJx3Gkg5TekBzjN5Y9OBiTMnagNBnkGzqbOU9yq33kh7HLn5UT0+RgQzPpnoCeO3ruNtpFxcaUfkPLArg0GLCpX\/zhQbrw3CNUyb3zUvUlDQ426yvM3aa6KcaWDlJ6Q0oRtdWswMGqvKGC22KgDV1PiGNTX\/Dk6VBlqsREtjikpaV0T8hkz\/3YsWO0fv36XB3o6uqitWvXUvBz\/VlWnq9Ib0hpNVjb+YCDbYXLxzfNIL+Xzgu7VN+xPNOT4cqrXD6FaQ7lczSbAuZuVk8j0Z599lkaGxujtra2XLyJiQlqbW2ljo4O9VlnZyf19vZSTU2N+l06SOkNyQh4D4KAg3sIphiwqb\/1RIt6bY176fPu3+XVIjHulS5dAlMcXF2ndE\/IZM+9p6eHGhoaVG9dH9xr58\/7+\/upurqa2tvbqbm5mZYtWwZzd9V6Mpiv9BtaFpAkYaAnyF08elCtDFepM91N1IMkHEzknzQGzD2pgobP5x76xo0b6fjx4yry0qVLlaGfPHmSBgcHqbu7W33O5t7Y2Jj7AiAdpPSGZLgaOAsHDs6kz2UclwHvTqZfYeOh9zmf2qB67DjiKRCXQ7zczJ8l3RMy2XMPYuYh+pGREVqzZg0dOnSopLnr844cOWK+pliOOD4+TnV1dZZzQfhyCoBDOYXs\/z0SgwvjdOXlvyF64XGieXVEH7+PZn38vqn\/40ikQCQOiXIye3JTU1MuoPTNuDK9cYwejt+yZQvt378fw\/Jm2wGi5SkgvbeSBaBhGBSaJMfP1HGYUyAMB3O5mY+Enrt5TRNF5GH53bt3065du9RkOX7OzsemTZswoS6Rsjg5jALSb2hhrtH3NKUYwNTToye9LcDc06sroXMKvvKmn7mz0Qc\/HxgYyE2m48DSQUpvSKHhep4QHNwDKsQApp4+F+ltQbonMPFMD8uHrdLSQUpvSGE5+Z4OHNwTCjJgU9cT5bDoTLpspLcF6Z4Ac5+u79JBSm9I6d527OUGDva0DRuZGdx63awZps6z3\/FMPayCZtJJbwvSPcELc+cZ7Tt27JhRo\/JXkDNT3YpHkQ5SekOyzTet+OCQltJX56PfUX\/77\/uIfn5MLTgzZ8UGrCTnCIn0tiDdE5yau34GXsjIteHnPxu3VU+lg5TekGxxTTsuOKStOFH+8\/TL9R+j29q+n35BkOMMBaS3Beme4MzceVb70NAQbdiwoWSTOHjwYNk0JtqUdJDSG5IJhj7EAIf0KPCCM7ySnF4eVg+9g0F6DErlJJ2DdE9wZu5+VL\/3SiEdpPSG5Ft9iFsecIirXLjz2NAvvT6sVpLTW63mD72DQTgtbaeSzkG6Jzg39\/zlYoMVrra2lg4cOECLFi2yXQ\/xKpx1hSsjA+k3NB8p5Rv67N9rUM\/Suade6AADPyhK5wBzN1CP8ndx49\/5qK+vV+vBP\/744wZyKR1COkjpDck64JQyAIfkQgc3b9FD7uUMPZgrGCRnYCKCdA7SPcGLnntwRTkukF5l7qGHHqK9e\/fC3EO0NOkNKcQlikgCDvEwBZ+fcwQ95F51x\/LI26yCQTwGps+SzgHmnrBGTE5Oqh3aeAhe77+uN3vZvn07fec735mxL3vC7IqeLh2k9IZki2vaccGhtOLcE\/\/N22PqufnUz6PqhCRmnp8jGKRd6wvnJ52DdE9w3nPXPfX8bVr37dtHe\/bsmbEta9IqG1x+Nv\/1O+kgpTekpGx9OR8cpl5N473QgybOv\/Pn2sh5mP135jcQ98ynfq4whhAMjEmZKJB0DtI9wTtzf\/rpp+nFF19Ur7+ZnEjHQ\/2tra3U0dGhKmxnZyf19vaqzWX4kA5SekNKdBfx6ORK4qAnurH8bORBA9cmzj+r71ihTJyPYpPgTCKsJAYmdTMdSzoH6Z7g3Nz1sHxjYyONjY3R8uVTNwGeSNfd3U3V1dVG6pze+rW\/v1\/F5EcBzc3Nuc1jpIOU3pCMQPYgiFQOvP667llrs+af+YYdlJiH0rV58880DbwUaqkMPKi+RosgnYN0T3Bu7sEtWp966ill7osXL56xbauJGsfmrr8wcDw2d\/5CsXbtWvTcTQicoRjcIw0el986E\/rqLrxzgebNnVc2fdBI8xNzL7jYwWZb9G8lzitVIG3SnIaHy\/ngoXI+5j94oOy1+JZAuqn4pmfc8kjnAHOPS376vEI99+HhYTp37pzxnnspcx\/901kJrwSnZ1GBc7NvtnJZb\/528bjnZt9UMs83i5RJxzy3+I+p9vrZM2LUznnv96\/8\/lwr1+RL0PHxcaqrq\/OlOBVbDqkcmpqacsxGR0dF83O+5Wv+QjbBPdhNKYtheVNKIk4pBWz3VnoOF+65n52YnFGsNyYuzfj97IX3fj+b97dC17Ogpkp9vGDe1M9ba6poQc3UI7K2lVO9el8P2wx8vW7fyiWdA3ruvtWoIuXBhDohoIQXU\/oNTcuvv0ToLw38ZYG\/IBT6YhD8InDP7VOPJFx+AcgKA+FNgaRzgLnHrIGllp3lkLZ67+vXr1clzt9tTjpI6Q0pZjXy7rRK4zD4k1\/mDP9HP7ugeLz083dyXLTxN9\/9fvXZPR+YS\/febvexQKUx8K4RTBdIOgfpnsAYnA\/LF1t+Vk92S6PySgcpvSGlwTiNPMBhSmXu4fO\/H00bPRt\/vumz4dvo4YNBGjW9fB7SOUj3BOfmHpwtr985L\/RZ+aqULIV0kNIbUjJ6\/pwNDuVZ6CH\/wZ+8mev1cw\/\/tZ2fLH9yiBRgEEKkFJJI5yDdE5ybe3C2vO6p9\/T0GJ8tX64uSwcpvSGV4yPl7+AQjRT37nlon4+ew6fVTzb6JL16MIjGwFZq6Ryke4Jzc+cCpDFbvlwFlg5SekMqx0fK38EhGSlt9tro21YupOa7b1aGH\/YAg7BK2U0nnYN0T\/DC3O1WsXDRpYOU3pDCUfI\/FTiYY\/TSz94hHrrnnj2buzb6cjmAQTmF0vm7dA7SPcGZuXNvfWhoSK0hX+o4ePBg2TQmqqp0kNIbkgmGPsQABzsUHhw8kTP5cs\/mwcAOg6hRpXOQ7gnOzJ0z1ru05e\/Qxn\/jGfQ7duy46pW1qBUsbHrpIKU3pLCcfE8HDvYI8ZD91sETatY99+KLzbQHA3sMokSWzkG6Jzg1d11RtJEHK04hw49SsaKmlQ5SekOKysvX9OBgnwwP12995oTKaN+6JVe9Nw8G9hmEyUE6B+me4IW5h6kottNIBym9Idnmm1Z8cEhLaZ5dP6Zm2POEuyeal+QyBoP0GJTKSToH6Z4Ac5+undJBSm9IftyOkpcCHJJrGDVCzdd\/SPd+YC49\/+Bd6lQwiKqgnfTSOUj3hEyau36Wr6usHuIPfp4\/7C8dpPSGZOf2kn5UcEhfcx6mX\/3kqzmDB4P0GRTKUToH6Z6QSXPPX86WLxIbx\/jR4LNeCuk3NKl8eLIdGzzPpAcDPyhK5wBzN1CPgovYPP300\/Tiiy+q198WLVoUKzqvcNfQ0EDBtemx5WssKXFSRAWk39AiXq5Xydng73z0x\/SxW6rohVYzS9l6dYHCCiO9LcDcE1a44PKzY2NjtHz5chVxcHCQuru7qbp6ag\/psEex1e5OnjyZi8mx2tvbqbGxMfcFQDpI6Q0pLF\/f04GDW0La4IPP4N2WqHJzl94WpHuC82H54CYxTz31lDL3xYsX0+7du2nXrl2kN5OJ20R4iH5kZITWrFlDhw4dUl8Yipm7zuPIkSNxs3N23vj4ONXV1TnLHxlPKQAO7mvC37\/2Bj38f\/6DNn9iLn3l9+1uL+v+av0tgdS20NTUlBN1dHTUX4FDlMzplq+Feu7Dw8OhN445deoUtbS0qPSrVq26qrevh+O3bNlC+\/fvp\/7+fjUawD335uZmWrZsmZJI+rc06d+SQ9RTEUnAwT0mzYBn0T\/\/1bus7x\/v\/or9LIH0tiDdE5z33LkAJjeOyd8ulp+\/87Fp0yZqbW2ljo4O9XtnZyf19vbmRgakg5TekPy8PUUvFThE18z0GZrB6idepbMXLhnbStZ0ObMeT3pbkO4JXpi7nt3OBrxx40Y6fvx4omVng6+8LV26VPXWeXg\/+PnAwECu146ee9ZvM+ldn\/QbWnpK2cspyIB77\/mL3NjLGZGDCkhvCzD3hPWZh+Ufe+wxNTv+lVdeIZ5Ux8\/HecOYhx9+OPKEurjFkQ5SekOKy82388DBPZEgA\/0OPIbn0+civS1I9wTnPff8CXX8CtunP\/1pYxPqwlZp6SClN6SwnHxPBw7uCeUz4OF53mxm4pufcl+4CiqB9LYg3ROcm7vuuX\/+85+nvr6+3DNx9Nyj3QWkN6RoV+tvanBwz6YQA37\/\/Z4PzJ2xBr37kma7BNLbAszdQP3Uz8I3b948Y+Jb3EVs4hRJOkjpDSkOMx\/PAQf3VAoxGPzJL4n3hOcV7BbUVLkvZAWUQHpbkO4JznvuvtRx6SClNyRf6kHScoBDUgWTn1+MAQ\/P31pThd57colDRZDeFqR7ghfmzq+r8ZB88AjOcg9VkxImkg5SekNKiM+b08HBPYpiDDC5Ll020tuCdE9wbu7BDV14tnx9fT2dOXNG1cLg2vC2q6V0kNIbkm2+acUHh7SULp5PKQbce+dDbw\/rvrTZLYH0tiDdE7wwd73ULK\/\/zqvT8fvuppafDdt0pIOU3pDCcvI9HTi4J1SKgV57\/onmJer9dxz2FJDeFqR7gnNz59nye\/fuVYZ+\/vz53FKyGJaP1uikN6RoV+tvanBwz6YcA55Y96Ofv4OV6yyjKsfBcvaJw8PcE0tIakW6a6+9Vm3xyjPnt23bRgcOHIi95WucIkkHKb0hxWHm4zng4J5KGAa8ch1673ZZheFgtwTJokv3BOc992TyT52tl69ta2tTvwfXqg9uJhNcfrarq2vGM33pIKU3JBP1wIcY4OCeQhgG3Hvn1+OwsI09XmE42Ms9eWTpnuCFubM579ixYwaNsMPyeqY9vyOvzZ0\/45XuVq9endv9jbeRxcYxySs8IpRWQPoNLQt8wzLg3nvbyoXUtrIhC5ft3TWE5eBdwacLBHNPSEb3stmY9farYUPylwKeXc+T8PjgGPnxdK+e94ln08eWr2HVRbo4Cki\/ocW5Zt\/OCcug5\/AY9Rw+jd67JYBhOVjKPnFYmHtCCfO3aI0TTm\/rqs1d99D5GT6b+8jIiNqM5tChQ2q\/dz54P\/fGxsbc0Lx0kNIbUhzuPp4DDu6pRGGA3rs9XlE42CtF\/MjSPcGbYXkuSNz32k2Zu64GR44ciV8jHJ05Pj5OdXV1jnJHtloBcHBfF6IwGDrxLj3yg3+joQ11VHv9bPeFz1AJonDw6bKbmppyxRkdHfWpaJHLMuvKlStXIp+V8ITgpLdCofKfufMrc9zbHhoaotra2hmz6fPNnfeE18P8GJZPCAqnR1JAem8l0sV6mjgqAyxLawdkVA52ShE\/Knru8bUzdmbQ3DkoJtQZkxaBIiog\/YYW8XK9TB6VAZaltYMxKgc7pYgfFeYeXzs6depUbtGa4CtrUUPmm3twVCA4iz74KtzAwMCMCXzSQUpvSFGZ+5oeHNyTicMAy9Ka5xaHg\/lSxI8o3RP4yp0My+th9ubmZmWyurcd97l7fIRTZ0oHKb0hJeXny\/ng4J5EHAZ6Wdrnv3oX3Xv7XPcXkYESxOHg02VL9wRn5p4\/S5578YcPH6atW7c64SsdpPSG5AS6hUzBwYKoEUMmYXDnoz\/GsrQR9S6WPAkHQ0VIFEa6J3hl7gcPHqSHH36YqqurE0GJc7J0kNIbUhxmPp4DDu6pJGHA5r5gXhV2jTOAMQkHA9knDiHdE2Du01VAOkjpDSlxS\/QkADi4B5GEASbXmeOXhIO5UsSPJN0TnJo7v7LGm8YUOsIuPxsf3cwzpYOU3pBMcXQdBxxcEyBKykCvO\/\/azk\/Sgpoq9xcktARJObi+bOme4MzcXYPLz186SOkNybf6ELc84BBXOXPnmWCA2fPJeZjgkLwU8SNI9wSYO4bl49d+nHmVAtJvaFlAaooBlqZNVhtMcUhWivhnw9zja+fVmdJBSm9IXlWGBIUBhwTiGTrVFAM8f08GxBSHZKWIf7Z0T0DPHT33+LUfZ6Ln7mEdMGkqeuc4vP8eHbRJDtFzT34GzD25hl5EkA5SekPyohIYKAQ4GBAxYQjTDPQEOxh8NDCmOUTLPXlq6Z6QiZ673hyGN4vhI7jMLP\/e1dWldpwLfq4\/01VAOkjpDSl5U\/QjAji452CLAT+Df6J5CTXffbP7ixRQAlsc0rp06Z4g3tx52dq+vj4KriGfb\/Z8kbwint7nnX\/v7Oyk3t5eqqmpUXVFOkjpDSmtBms7H3CwrXD5+LYY6CH6tpULqW1lQ\/mCVHgKWxzSklW6J4g2dzbx+vp6Gh4eVrx1z73QOvXca+fP+\/v71Qp4vH2sXtce5p5Wc8l+PtJvaFkgZJPB4E9+STxMz7137sXjKK6ATQ5p6A5zT0PlMnkU2s9dL46jF8M5efIkDQ4OUnd3t4rG5t7Y2KiG62HuHkDMSBGk39CygME2A55Fv\/WZE0qqfeuWYKOZIpXGNgfbdRXmblvhEPHzt3wNnsK9+5GREVqzZg0dOnSopLnr844cORIiV7+SjI+PU11dnV+FqsDSgIN76Gkx+NY\/vkN9\/\/QOfeyWKnrkj26k2utnu794j0qQFgfTl9zU1JQLOTo6ajp8qvGcbPka9Qr1FrFDQ0NUW1tLBw4coEWLFqkwpcxdD8dv2bKF9u\/fj2H5qMIjfSQFpPdWIl2sp4nTZMBbxa5+8lXin3gWP7NCpMnBRlVEz92GqhFj5g\/L7969m3bt2qUmy+m\/bdq0CRPqIuqK5NEVkH5Di37F\/p3hgoEeqtcmz8\/kK31dehccTNZGmLtJNWPGyu+5B195C25AE\/x8YGCAli1blstROkjpDSkmeu9OAwf3SFwy4Al3L\/3sAvFPNvfmu99fsTPrXXIwUQulewJrIGJY3gSsUjGkg5TekGzzTSs+OKSldPF8fGDAPXg2+J7Dp1VBtdHf84G5FTMBzwcOSWqjdE+AuU\/Tlw5SekNK0gh9Ohcc3NPwjUEho2eTX1BTTVk2e984RK2Z0j0B5g5zj1rnkb6EAtJvaFmA6zMDbfQ\/+tkFeunn7+R69QvmVdE9t8\/L1BC+zxzC1HOYexiVBKSRDlJ6QxJQRUIVERxCyWQ1kTQGvPIdH0HD10P52vT5d2mr4knjkF8ppXsCeu4Z6blnoSJaveOnFBwcUhK6RDbSGXDv\/kc\/f0e9Xnd2YpLemLiU6+Xry9Yz8dn8b62pUkP8+guBL2vfS+cgvfwwd5i7+7txhkqQhRuCdByVwED39rX5M7OzF\/jLwKWi+IKv5vGXAj74i8HUl4KpLwf5Xx6SfFGQzkF6+WHuAXOXflND+aEAFIACxRS49KE\/zv3pP6+9Qf3\/P6+9seRnttT8rX\/\/NyOhf+vfzxuJUyzIW\/\/zz6zGtx0cr8LZVhjxoQAUgAJQoKACehQijjw8cmHzkL45EMzdZu1AbCgABaAAFIACDhSAuTsQHVlCASgABaAAFLCpAMzdproOY\/NmO8899xz967\/+Kz3wwAO0cOFCh6WpzKwvXLhA3\/3ud4l3yFq3bh3deeedlSmEJ1f9\/PPP06233kp33XWXJyWqnGJcvnyZvv\/979PLL79MX\/ziF8EgBfQw9xREdpHFP\/zDP9ANN9ygds9jg\/nyl79M1dUzZ8W6KFcl5clbDn\/oQx9S\/771rW\/Rl770JZo3b14lSeDNtZ48eZKeeuopuu+++2bsK+FNATNekB\/84Ad0zTXX0Cc+8Qn6u7\/7O\/rMZz6D+5Fl5jB3ywKbDj8xMZHb4U5ve8smsmPHDpWV3hSHzeSzn\/2s6qkcPHiQVq1apXbKw5FcgbAMdE7cg\/\/2t79Nmzdvpuuuuy55ARBBKRCWw7vvvkt\/+7d\/SzfddJPSP7hpFKRMpkBYBlz\/2dxff\/119SX3wx\/+cLKMcXZZBWDuZSXyJ8GpU6eopaVFFUjvac+fdXZ2Um9vL3HvZHBwkLq7u9VPNvT58+fD3A0ijMKAR0rY2Pfu3UvNzc20ePFigyWp7FBhOXR1ddHRo0fpgx\/8IL399ttKNJi7mboTlgHfj5588kn6gz\/4A\/rIRz5CfX19tGHDBoximcFQNArM3bLApsLzN2QeVuThrEceeYT27Nmjhty51z4yMqIMnZ+zt7a2UkdHh\/qGzN+OGxoalLmvXbuWrr\/+elPFqcg4URm8733vU8z4kcgtt9xSkZrZuOgoHLZt20YnTpygN954g37xi18oQ\/na176GEZSEYKIw4PvRSy+9RPfcc4+6Z3HH5Atf+AJGEhMyKHc6zL2cQp79nb8tb9++fYa5j42NUVtbmxqm3Lhxo\/o\/D8fz0DzfzLhBfe5zn\/PsSuQWJwwDZvTqq68qQ7nxxhuJjf7++++HqRjEHoYDtwXdUz927Bh67gb151BhGfAjETZ1HkmsqqpSI5CzZ882XBqECyoAcxdWH8I2Jgw92gMLBva0jRIZHKKoZSctGNjR1URUmLsJFVOMUagxFRqW15PtUixaxWQFBn6gBgf3HMDAPYNiJYC5+8umYMnyG1OxCXV47c0eWDCwp22UyOAQRS07acHAjq4mosLcTaiYYoz8xsRZ61fhamtrc7PoUyxSxWUFBn4gBwf3HMDAPQP03P1lgJJBASgABaAAFDCqAHruRuVEMCgABaAAFIAC7hWAubtngBJAASgABaAAFDCqAMzdqJwIBgWgABSAAlDAvQIwd\/cMUAIoAAWgABSAAkYVgLkblRPBoAAUgAJQAAq4VwDm7p4BSgAFoAAUgAJQwKgCMHejciIYFIACUAAKQAH3CsDc3TNACaAAFIACUAAKGFUA5m5UTgSDAlAACkABKOBeAZi7ewYoARSAAlAACkABowrA3I3KiWBQILwCk5OT1N7eTkNDQzNO2rx5M\/E+5JIPXnP88OHDtHHjRnWNjY2NtHbtWnVJ+rqbm5tze60Hr5X\/vnfvXtq0aRPV1NRIlgFlhwLOFIC5O5MeGVe6AtrkgsaXBU2C5sy7E0Y1d9ZAfznYunVrFiTBNUCB1BWAuacuOTKEAlMKlDJ33ulvZGSEzp49S+vWraOPfvSj1NLSQufOnaOlS5dSf3+\/6tUeO3aM1q9fT7wj4IoVK2jOnDmqx8s9Zu79L1u2TO0aODY2pn7XOwhy\/nqEgGP09fWpMg0PD9OqVauou7ub2Jh7enpyfxsYGFBpBgcH1d\/5YOPO74FzvDNnzqieeqFrDPbcOT+dN8cL5r1v3z5auXIlLVq0CFUGCkCBiArA3CMKhuRQwJQChYbltXG\/8MIL9MwzzygT56O1tZU6OjqU0WmzDpo4n8dGyyZfzNyXL19e0Jg5\/rZt29R2wTfccEPuiwF\/zubOZTh\/\/jx1dnbSww8\/TH\/1V39Fu3btyn3W29s7Y\/icz+G8+ItFsUcPHJu\/LHAafQTP48\/4OvnQw\/mmdEccKFAJCsDcK4EyrtFLBcL03LmHPD4+nuu16wthM9+yZQvt378\/14svZPrBnntDQwPt2LFjhhbce2cj1iauh9G5N869b93jD56kTZg\/4553cH4AX9Njjz1GGzZsUF9EyvXctbnnGzvH5hGA\/PhegkShoICHCsDcPYSCIlWGAlHMnXvN+T1kNj9tyjxEH8bcC5l1ME4Yc9emy5R0D10Ti2PuxXroMPfKaAe4SjsKwNzt6IqoUKCsAmHNndPxM3R+9s5D1Pp5\/Pbt24knnHHPOTgs\/9BDD+Umsa1evTo3XM9GrIff6+rqcmnq6+sL9tz5AoLD8pzfnj17iM\/l2ez\/8i\/\/ctUXDn1O\/rB8sdnyPDpQbOgdw\/JlqxASQIGiCsDcUTmggCMFwpo796Z59njYCXXBiXPBiXalJtQVGpbXQ\/p6KL+rqyv3\/Ds4SS9fvmCPu9Sw\/Oc+9zn1WOH48eO5EHrOAV9zcHjfESJkCwXEKgBzF4sOBYcCMxUoZbgmtUrjPXW8CmeSGGJVogIw90qkjmvOpAJpmPvExIR6RLBgwYLc63KFxEzyvDz\/uX0mYeGioIBlBWDulgVGeCgABaAAFIACaSsAc09bceQHBaAAFIACUMCyAjB3ywIjPBSAAlAACkCBtBWAuaetOPKDAlAACkABKGBZAZi7ZYERHgpAASgABaBA2grA3NNWHPlBASgABaAAFLCsAMzdssAIDwWgABSAAlAgbQVg7mkrjvygABSAAlAAClhWAOZuWWCEhwJQAApAASiQtgIw97QVR35QAApAASgABSwr8P8BfVjb26InwPkAAAAASUVORK5CYII=","height":303,"width":503}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAfcAAAEvCAYAAABYGIhlAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10XdV15zcNSREhBAtDiBCuDcjFKYmNCamiQJyMQt2klZnJpJXkaZuqSsaZYJhV8JLkEFh1ILasselQIBOVUbycWbXsdhUmVj+WS12ahBoXarCmMzFBIH8ghBPAOJBEaReJZ+3rHnF0de+75\/zffVf33fd\/a2UR65197jm\/vd\/53\/N9xqlTp04JPyRAAiRAAiRAAoUhcAbFvTC+ZEVIgARIgARIICBAcWcgkAAJkAAJkEDBCFDcC+ZQVocESIAESIAEKO6MARIgARIgARIoGAGKe8EcyuqQAAmQAAmQAMWdMVA4AidOnJDu7u6gXkNDQ1JfX1+xOo6NjUlXV5dcffXV0t\/fL3V1dRV7lp1xlnV0qZDhcNNNN0l7e7uLSa7TbN68WQYHB4MyrlmzRnp7e73Ku2vXLlm\/fr1s2rQplzy0fvv376\/478MLGhOnSoDinipOZpYHAlkKX5y4a+O5YsUKaW5urgiSuDpW+rlxlUHEQsXlm9\/8ppdwIja+DtBnrF69etqsiOJetJcxXx\/XQnqKey14mXXMjMDU1JT09fXJyMiI7NixIzNx1xGDLJ4bBdIIRVtbm7NQm56tj3AiNojjjbj7lC38nLz33E2cHjt2jL13JEiqwIbiXgVOmssi2sOTWg57mNHutS5btkzuvPPOoKjayNtD1KaXOTo6Out7O48bbrhBPvOZzwRpGhoaZNu2bdLU1BRZfVtEw+nDvVr93gzTt7a2yt133y1Lly4NGjVbFJPy0eF989wDBw4E5dOPGZa\/44475Etf+lIg7OYTZqF\/N0xt8TeCYqc3AmHyssXGruN9990nAwMDkc+dmJgIyjc5OTldJtuHNkdlfv\/998vXvvY1MfVT\/mHWhp2Z7ogSMuNX+7mmvuF6GV83NjZOv6CE+e3evTsY5jYfOz7C+SW9VIXjsVRepeKwVA\/fZqJlNmUPvzCEf182W5PHLbfcInv37hX9\/Rjf2XXWv5ln2L5N4hIVh3PZ1vDZ6RKguKfLszC5hRt0u2KmgYpqwMONsuajwmqEPfx9lPiUEkb9Lq5spiE+\/\/zzZ8y5G3G3y6AiGiXGtsCH80lL3KN6huGGNtzox3HVv8eJe09Pj6xdu3YW+1Jiqt9dcMEF8tJLLwUvL1GCq8+0RShc9ri4MM998sknI4X6wQcfnJ7ntuPNFq+wuIfzMt\/HCXypmFWbo0ePxr5E2GUKC3v4BcwI63XXXSff\/va3Z7QLUQId9fsKi7OmiSqj\/t08Jylvm0veRxcK05jOUUUo7nMEPu+PNY2X3XMxDaOW3e61au\/MNBp242k3RHaPxRYDFVDTszR5mGeHe4iGWdT3dkN1\/fXXx4q73bPxzSdJ3HW0Qj9Jw+OlRhZ0NOGVV16ZxcTubSqnxYsXz6ijy7B8eMrAsDf+1F562O9aFp1\/jhpRUJarVq0K6mv39F0WGboMsYfThP9tmJgXES1\/0rNN7EXVx\/xNXwK1znHD8jZHE09hnz788MPBS0JUTzwu3\/DojRmtsPMo9WzTszfxn8QljemHvLdjtVw+inste79E3ePe6k3jqI3a8uXLI1eK22mOHDkS2RvTR9t5aG\/RrGxPWhCX1OOIE0+7sdPn++aTlrjbz1ah1o8tJnGNri1un\/3sZ53FPWqkI+q5Wo7wtENcz1jTqkiZcthso54Xnp4oJe5x0xFhm1K98KgXw3DdzJRP+CXBvNDEiXBSfNr+tfOI82t4FMCwMuIeNx1j7wSxY9n8Lu0pEfNTt7lETQWxOSwOAYp7cXyZak2yFnd7K1lS4+krygomamucaz62cIWFQPO2t8K59Nw1jb0ITf+t267CIxdhcfEV9zDHcO8+\/FKRlribQIx7qdAdBFHiHh7eT+q5V4O4R40UGb+G4y+u527nEffboLin2vwVIjOKeyHcmH4l0GH58PCxmcOM6wVFDaMmiXup4XS7N6lUtHcTJ+6u+ehwZ1h4zXRFWNxVQF0WKoWFz+7Zhqc2VAyThuV1VCFJHMN5oMPydrTF9YbDERkW6nAv1tTZHsEx9TGxE7aJGpZP+iVUeljevAiaEY84cY8a8TCMwj33uAWQ4SmBUsPyUVw4LJ8ULdX9PcW9uv1XsdJXekFdKXFMEvdSZYuaj44T96R8dAjTzJ+HQbuIu9pErZY3eYVXPNuHv\/gsqDPDs7aNPveTn\/xkMKoQ9VFOUfVzXVCneZoXnvBLRdxiM9vGTqPPvOeee+Suu+6atfhPbcLirn+LW5xn6pr0Mhk1ZJ00cmJzjKtjKWG2xfTmm2+Oja1SeWgZohbauS6os7kkjVxVrHFhxpkQoLhngrl6H+K6Fc7expa0FS5qkZ7PsLzSLDXkm7RgzT6xrlQ++pzwtqnbb79dDh48OL2ALKrnbjf8cYsC7bzDawGixN8WOdtW\/78R96jnPvDAAzNOWtODdez5fWQrnC3StthEbZOM24IX5mqvAdA8lduWLVtk3bp1AQ57BMbseojbWpe0P73UVjh9lmuPNm6uXEdvooQzbrRCGUVtQ4zq\/ce9GOrfwyfixa1dMHm4jDBVb8vFklPcGQMwgaSVyXDGNMyEQNTwf3hHRNw5A3YBkUNsMqlgAR9iv4yZl5ioFfRJVechNkmEqv\/7qhR3bUx0D68e3BHV+JQ6NKX6XZafGlDc8+MLpCSlpiVKTSdEPQs5fhYpM22ih+WVS9LBT1EvZEW5C4BxMZtA1Yl70gIe831LS0twYYP5twa+7+UPDJjSBCju1R8h4RdhrZGvsKsNzyrPNhbC02U+wq4l5ctYtv6ai6dVnbjrvJUGpn7ieu5hkDr3tG\/fvkxv7ZoLZ\/KZJEACJEACJKAEqkrctZexYcMG6ezsDASe4s4gJgESIAESIIEqH5bXHrh+9ASmUnPudjXNsGNHR0cu71VmUJIACZAACZBA2gSqpueuc3rbt2+X2267TfSyERdxN\/PtCs2+pSwM8dJLL02bK\/MjARIgARKoUgJ6C9+iRYuqtPSni1014q7D8LpHV0\/rSlotrxVzFXZNq+I+Pj5e1Y7MuvBkhhEnN3LDCGBWjLfa5VYV4h61ote4LOpqR98V8vwB+P8AyMyfGV8kMWbkRm44AcyyCO1bVYh72D1JPXft5eupT6WG4u08i+BILIRxKzLD2JEbuWEEMCvGW+1yK4S4m566rqI391ybIyqNa0sdA8ofgP8P4PDhw1U\/J+Vf6\/ItyA1jSG7khhHwtxp+4rjcOHxITtz9UX\/jHFlUpbinzY\/i7k+Uja0\/M7UgN3LDCGBWjDd\/bhR3f2a5taC4+7uGjYY\/M4o7xozcyA0n4G9JcfdnllsLiru\/ayju\/swoUhgzciM3nIC\/JcXdn1luLSju\/q6huPszo0hhzMiN3HAC\/pab9xyRzXsOc87dH13+LCju\/j6huPszo0hhzMiN3HAC\/pYUd39mubWguPu7huLuz4wihTEjN3LDCfhbUtz9meXWguLu7xqKuz8zihTGjNzIDSfgb0lx92eWWwuKu79rKO7+zChSGDNyIzecgL8lxd2fWW4tKO7+rqG4+zOjSGHMyI3ccAL+lhR3f2a5taC4+7uG4u7PjCKFMSM3csMJ+Fvq6XS6HY4n1Pmzy50Fxd3fJRR3f2YUKYwZuZEbTsDfkuLuzyy3FhR3f9dQ3P2ZUaQwZuRGbjgBf0uKuz+z3FpQ3P1dQ3H3Z0aRwpiRG7nhBPwtKe7+zHJrQXH3dw3F3Z8ZRQpjRm7khhPwt6S4+zPLrQXF3d81FHd\/ZhQpjBm5kRtOwN9y1f1PyaPPneSCOn90+bOguPv7hOLuz4wihTEjN3LDCfhbUtz9meXWguLu7xqKuz8zihTGjNzIDSfgb0lx92c2JxZjY2PS09MjAwMD0tTUFFkGiru\/ayju\/swoUhgzciM3nIC\/JcXdn1nmFlNTU9LX1ycHDhyQbdu2UdxT9ADFHYNJbuSGEcCsGG\/+3Jbd9ZgcO\/ETzrn7o8vOYv\/+\/bJ58+bggey5p8udjQbGk9zIDSOAWTHe\/LnV3\/JIYMQT6vzZZWJx4sQJ2bBhg3R2dgYCT3FPFzsbDYwnuZEbRgCzYrz5cdMeu\/bcKe5+3DJNvWvXruB5y5cvd5pzN4Xbu3dvpuWs1odNTExIY2NjtRZ\/zspNbhh6ciM3jIC7VWtrq7wx\/xflh9f2UNzdsWWbUhfRbd++XW677TbRRoEL6tLnzx4BxpTcyA0jgFkx3vy46YUxeogNe+5+3DJLrcPwK1askObmZuFq+cpgZ6OBcSU3csMIYFaMNz9uFHc\/Xpmm1rn27u5uGR0dnfXcHTt2BIIf\/nArnL+L2Gj4M1MLciM3jABmxXjz42a2wf3cj1+Wl7\/6G37GOUt9xqlTp07lrEypFoc991RxTmfGRgPjSm7khhHArBhv7tzsxXRnvvxd+f7XP+dunMOUFHcRYc\/dPzLZaPgzY88dY0Zu5IYTcLd89NmTsuorTwUGZz29Wyb\/6g\/djXOYsvDi7sKc4u5CaWYairs\/M4oUxozcyA0n4G6565+Oy3\/ZcXox3TmPDsixx\/\/a3TiHKSnu7LlDYUlxh7Bxzh3DRm7kBhJwM7OH5BfUnyWvfe0\/yfj4uJtxTlNR3CnuUGhS3CFsFCkMG7mRG0jAzcwekt\/6qV+UO3\/rWoq7G7p8p+KwvL9\/KO7+zDi8jDEjN3LDCbhZmlXy2mvf\/fmr5CPvfw\/F3Q1dvlNR3P39Q3H3Z0aRwpiRG7nhBJIt7V77bze\/W+75zSsKsciaw\/Iclk+O\/ogUFHcIG4eXMWzkRm4ggdJm4bl27bVr770IHT6KO8Ud+tFQ3CFsFCkMG7mRG0gg3kyFfe3wIXn0uZNBovs7l0jnNRcF\/5\/injruucmwCI7MmhzFHSNObuSGEcCsGG\/R3FTYN+85LHrcrH6uvew82X3jVdOJi6AJ7LkX5C0N++njVmw0MHbkRm4YAcyK8Tabmwq7irqKu350GP7gFz84IyHFHYu33FkVwZFZQ2WjgREnN3LDCGBWjLeZ3FTY9RQ6\/a8RdjPPbqcsgiaw586eO9RqsNGAsHHuGMNGbuQGEnjTTFfFr915KFHY1YLiXjbufGRQBEdmTZLijhEnN3LDCGBWjLfT3P7XP74o\/3XX09MQzX52\/W\/UpwiawJ57Qd7SsJ8+bsVGA2NHbuSGEcCsajnedOg9WBFv9daV4p9+9n3ysSXnlwRKccfiLXdWRXBk1lBrudEohzW5YfTIjdxcCcSJelJv3c6\/CJrAnjt77q6\/mRnp2NhC2Dh3jGEjN3JzIhCeV1cjH1E3D6G4O+HOf6IiODJryhR3jDi5kRtGALOqlXh7\/MgP5Ff\/6MkZkFTUf\/eDF8snr7owEHifTxE0oWI99xMnTkh3d7eMjo76MJWlS5fKQw895GVTbuIiOLJcBr72tdJo+HJJSk9uSYSivyc3cgsT0L3qw4+\/OH3CnPlehbzvVxdJy6XneYs6e+4OcWbEvbe3V5qbmx0sRPbv3y+bN2+OFPepqSnp6+uTkZGRIK81a9aI5h332bVrl6xfvz74uq2tTfr7+6Wuri4yOcXdyT0zErGx9WemFuRGbhgBzKpo8abD7sNPvDh9spxNBRl+j6NaBE2oeM89LXFX0deP5mdeHDo6OqS9vX2Wf8xLwtDQUCDo+lLQ0NAQ+zJQBEdiP33cqmiNBk7Cz5Lc\/HiZ1ORWm9zMYTP2GfBhQf\/QZedJ5zXvlmsvPw+DFGFVBE2omLinRjkmI1vsw0m0175v377p3nr43+H0RXBkpXmH82djixEnN3LDCGBW1RhvKug\/O3VKbt759Kwhd0NBe+n3dSxJVdBtwkXQhIqJuz3nnjSE7hu2SUP+UT33lpaWyF6+PrsIjvRlWG76amw0yq1zGvbkhlEkt2JzC1\/kElXbSgs6xd0zxrSHPTg4GFjp0Pi2bdukqanJM5c3k5v8kubRx8bGpKurSyYnJ2XHjh0l5\/1V3M1n7969cNlqyXBiYkIaGxtrqcqp1JXcMIzkVgxuk6+9IQ3nnin63z9+\/KSMHPphbMU03QcuqZPu978zsKn0p7W1dcYjxsfHK\/3IiuZfsZ57uNTh1fPl9uZV5FW4oxbK6TD8zp07Refc6+vrg0V6+olbgMeeu3+MsSflz0wtyI3cMAKYVV7iTRfCDew5HDvMbmqnvfM\/ar9CFp5fB690x0jNtCqCJmQm7jY6u1eN9uY1j56eHhkYGJgxEmBW1dvD8HFpTZmK4Mg0Atonj7w0Gj5lzkNacsO8QG7Vw02FfM93XpbR5193EvPPffgS+cSV8+dUzMN0i6AJcyLuNkh7flx72a6fODuKuyvB8tKxscX4kRu5YQQwq0rFm86Ray9b\/\/vwoVfkGwe\/7yTkWovNn1wsb3\/bWyq2GA4jxZ57GtzE7rlrhklz4kFAWEPrRsDjtrdFDcvHDeFr3kV4S0vFMR6ZVKrR8ChCVSYlN8xt5Da33Mx57X\/79Cvy5NHXEoVcS6vir9vUelcuylWv3IVkETQhs557+BCapAVxYQeUsjffdXZ2Ti+csxfyJT2rCI50Cdg007CxxWiSG7lhBDArJN50WF2FefOew\/IPz52cvv+8VAk0\/YJ5Z8m6X1koP3fGGbnulbuQLIImVFzcbZF17aW7wE8zTREcmSYPl7yQRsMl36KnITfMw+RWGW4q5A8d\/J6Mfe\/HTr1x0yNXIb\/l+oVy6fzTp376nt2O1SY7qyJoQsXE3V4dn9Rzzs5l0U8qgiOzZsjGFiNObuSGEcCsNN5e+Om8QHzv+\/tj8vSLP3IWcSPa110+T9rff1GQR9FEPI5qETShYuL+7LPPys033yx33HFHKmfLY6HtZlUER7rVNL1UFCmMJbmRG0Yg2UrnxX\/6s1Oy5eEj8vyJn3iLuPbGP3NdoyxrfEche+PJBN9MUQRNqJi4J50iFwW61MUxPo7xTVsER\/rWudz0FCmMILmRG0ZAgrlv7TnrUProxOuy5\/+9LMde\/YnTnLh5ppkb\/+x1jbK08R010xP3ZV4ETai4uPPKV9+wqo70FCnMT+RGbqUI2FvMHhs\/KX\/y+Isip8SrF26G07Un3rb4rbJkUWNNDaljETbTiuKeBsUc5FEER2aNkSKFESc3clMC5razbz\/7qux64ngA5dHnTnrBMfPfZk58WtTrz5rOh\/HmhXQ6cRE0oWI9dwzp3FgVwZFZk2OjgREnt9rhZraUfX3\/pDx++AfeQ+hGrPW\/ul\/841deIO+7+JwAoOvCNsYbFm9F0ASKOw+xgaKfjQaEjWfLY9hyy00F\/Cdv\/Eweeup73ovYDAoj1DqM\/tvNDfLLi97pJeClkPJ3igUcxR3jljurIjgya6hsNDDi5FY93MzQuf73L\/75JfnO5A+h3ne4B\/7r771AfqnBrweOUeNFRSi3ImgCe+7suUPxT5GCsOW2B4rVJjurSsWbGTr\/n49OyMHnX4fF2wi49r6vuOjtsmrphdND565D6JWgWSlulShrnvKkuOfJG2WUpQiOLKP6kCkbDQgbxR3D5s3NXnWu\/\/+fjv5A\/u7pE2WLtxb\/I4vr5Xc\/2CA\/\/JefBsesmmeBVauoGX+nGN4iaEKmPXe90GX9+vUBbb0s5ujRo7Jv377IO9kxl2BWRXAkVnPcio0Gxo7c0uNmhs3HX\/6x\/NmB03Pevvu+7dLYc9+\/+kvz5dffd0Eg3Crg1fphvGGeK4ImZCbuesa83symd7CvXbtWent7ZenSpdLX1ydxt7thbvG3KoIj\/WtdngUbDYwfublxM8Ktqfc+\/YrseOyY1J11lvd2sUjxrq+Tz1x7sdSf\/dZcDJ27EcFSMd4wbkXQhEzE3T6tbvHixdLd3R2Ie3Nzs6D3uWMui7YqgiPT5OGSFxsNF0qz05DbaSZGvE9OvSHb\/uEFee6lH5fV69Y8Tc9bt439ynvmy1WXvKPqe95YlL1pxXjDCBZBEyjuXFAHRT8bDQib99wx9pS5t9KFao3zfl7u\/\/vn5bvHf1S2cDece6aceeaZwbWiyxacK5\/50MXTwp3nOe+59gR\/p5gHKO4e3HS+XefX7WF504vv6OiQ9vZ2j9zSTVoER6ZLJDk3NhrJjKJSFIGbCvcl9T8v9z2SjnDbvW49be1Ty98li\/7tKlHzXRG4YRFTnhW5YfyKoAmZ9NwNXh2CX7169QzamzZtmlNh18IUwZFYCONWbDQwdnnmpj1g\/d9F73xbINzjKQyV28Ktve6PXzlf3nvxmxeWuG4TyzM3LBKysSI3jHMRNCFTcccwn7aampoKFt+NjIwE\/16zZk0wbx\/3sV8kdOHe0NCQ1NfXRyYvgiPLYYvYstFAqM3doSJGuH\/8rz+Vb4x+v+yV5ab29grzZZe8I5jrnv6bdcY5RutNK8YbRpDcMG5F0ISqEXddba8fFXSzQC9uOH9sbEy6urpky5YtwaI9MyXQ398vdXV1s7xdBEdiIYxbsdHA2FWCW9QlJOVsCYsS7uW\/cK587Irzg6+Ca0NTFG4XkpXg5vLcak9DbpgHi6AJmYi7EeOk61+TeuO2m2yxD7tPxfzIkSMle\/a2TREciYUwbsVGA2Pnys0++vSCc94qDzz6gjzzvfIXpkUJ979bcr68f8G508JtBByrYWWsXLlV5unVmyu5Yb4rgiZkIu6KVwV3586dM4bH7R74qlWrnPe821vrtGduf8zwfUtLi\/NcfhEciYUwbsVGA2On3N7yzncHxirgTx57Tf720Cun\/\/3q6Tnvcj72MPkl9WfJ4ne9Xf7DsgtnnKKWda+7nPoYW8YbRpHcMG5F0IRMxL2UGNv73J955hnRHvlDDz0U6xH9fnBwUNra2iJPtjPivnLlSnnggQdERwtc5tzNA\/fu3YtFQ41ZTUxMSGNjY43VOrm6B174ibz7HWfKY8em5J+P\/4tMvvaGvPj6G8F\/y\/3odjD9aP5LLnybXLvwbLn43\/6m3+kzTJpyn5U3e8Yb5hFyc+fW2to6I\/H4+Li7cQ5TVp24G4bmxLvwPLoR92PHjk2PEsSlNXkV4S0t69iqtR6B6VE\/ffxHqS5IU7\/Zve2F8+vkN65+l5whZ1RkYVrWcZLW82ot3sgtLQJYPkXQhEzEXfEmDcvrPnez8O2ee+5J9IgumtM98wMDA9LU1DSdPmpYPi4txT0Rc2yCIjS29rz2OWe9Rb7+2KQ8+\/3yT0qzodnC\/eHF82Rh3ZRcdNG7c3\/hCB4ZlbEsQrxVhkzpXMkNo05x9+QWtc9dL5AxK9rvvfde2bZt2wyxjntEqWNrtae+cOHC6Tl3FfeNGzfK1q1bI7fDFcGRnq4oO3neGw1zlefoxOvBnPbhl6dSmdM24GzRvvLic4L929rbNj3xuHntvHMrOzAqlAG5YWDJDeNWBE3IrOeOIX7Tyl4db3rncRfOhIW\/1Mp6fUIRHFkuX1\/7uWo0TG\/7Oy\/+SEb+T3r7taNEu+ldb5dfu3K+XH7h2akddTpX3Hz9m7f05IZ5hNwwbkXQhKoR9\/AhNvaCOvNdZ2dnMAqgH3uUIG7xHYflscBXqzQbDXuF+KHjP5K\/+r8vyeGXKtfTvuLdb5f\/fF2jvO0tPzcNIKsV5Glyw71XfZbkhvmM3DBuFHcPbuZgGb32NfxJWs3u8RgoaREcCVW8DCPXRsMMjz\/y3RPy+JEfpHYyWlxP+9ffO1\/e+m+incd7uF25leGaQpqSG+ZWcsO4FUETMum524vczH527WWHr3\/F3FC+VREcWT4F9xy0p\/3888\/LqbdfIPPOPlO++q0JOfpK5XrayxecK4vfdbYsqD99umAljjd1r315KdnYYvzIjdwwAphVETQhE3EP73O3F7zp8Pnw8HDknnXMLf5WRXCkf63jLcw55D+YeiMYIn9eLxRJ4YCVGcI87yxpvvQ8+a1fPn2gS7gnnmZ98pQXRQrzBrmRG0YAsyqCJsyJuNvHw5Za9Y65xd+qCI70qbWZ437u5R\/Lnz\/5fTn2ypQ8+txJnyxmpbVXj7+n4Rz5tffOl1+or0ttIVpZhcuRMUUKcwa5kRtGALMqgiZkIu6K116xbgv6ww8\/HNzzHnepC+YaP6siODJcYxVwFVz9758eOC7feuZVWMCNcH\/osvPklxe9Uy6df7YcP\/6ifOA9i4LHZrUYzc+r+UxNkcL8Qm7khhHArIqgCZmJe\/hwGXOMrG5nc93bjrkp2aoIjtRaqpD\/t785Esx\/+\/TEg1u+5p0levPX77Vc7HQOORvb5LiKSkFu5IYRwKwYbxi3ImhCZuKOIc7GqlodqSvRh594UYafOJ4Iygj4dU3zpP39F00PlycaxiRgo4GRIzdywwhgVow3jFu1aoJd20zE3fXimPr6eswTZVpVkyO1d752+FDJnrkKuQ6hd15TuWNO2WhgQUdu5IYRwKwYbxi3atKEuBpS3HN+Qp1ZuT6w53CsoKuY3\/iRS2TJRecEZ5Zn8WGjgVEmN3LDCGBWjDeMG8U9gZuuil+\/fn0i3TVr1khvb29iukolyKMjVdS\/Nfaq3Lzr6VnVVjH\/yOJ6+dTyd2Um5uFCsNHAopHcyA0jgFkx3jBuedQE35rMec\/dt8CVSJ8nR8aJugr6iqZ5cuv1C3OxOp2NBhaJ5EZuGAHMivGGccuTJmA1EMlE3NHCZWWXF0fqArm1Ow8Fi93MR0X9vo4lgaDnacsZGw0sOsmN3DACmBXjDeOWF03ASn\/aiuKekzn3zXuOyOY9h2eJelZz6L5BxEbDl9jp9ORGbhgBzIrxhnGjuJfgZlbIj46OJtKt5YtjtJe+6itPTffWtXd+z29eISsWz0vkNpcJ2Ghg9MmN3DACmBXjDeNGcce45c5qrhyp+9NvHD40o7e++\/NX5Wr4Pc5ZbDSwMCY3csMIYFaMN4zbXGkCVtpoKw7Lz8GwvPbW\/+KfX5IvfuPZaa\/s+uz75Pol56fp24rmxUYDw0tu5IYRwKwYbxg3irsnt6itcZs2bZL29nbPnNK5kOhIAAAgAElEQVRNnqUjzVnvG\/\/69Py6WTCX17l19tzTjTU2thhPciM3jABmlaUmYCVMtsqs567CvnPnThkaGhJzEp2Zl+\/o6EgUeHM2\/cjISFAr173xY2Nj0tPTIwMDA9LU1BRJJEtH\/uHeo3LnX45PC3u1DMOHwbGxTf5xRaUgN3LDCGBWjDeMW5aagJUw2SoTcU\/j+Fn7VjnXlwLzQnDgwIGSl9Nk5Uh7jl177NUq7BpWbDSSf1wUd4wRuZFbegSwnLLSBKx0blZVI+7h6thiH1dVc7Wsfj\/XPfewsOve9Wobirc5U9zdfmAc8cA4kRu5pUMAy4Xi7sGt3GF5+1GlRgJMOk2zYcMG6ezsDO6Sn2txr7\/lkaBo1d5jN3wp7h7BbyUlN3LDCGBWjDeMG8Xdk1saC+rMPfBtbW3S398vdXV1kaXQZ+ln+fLlTnPuJpO9e\/d61io5edv2CZl87Y0g4R98bL60LTkn2SjnKSYmJqSxsTHnpcxf8cgN8wm5kRtGwN2qtbV1RuLx8dNro6r1k8mwfCXgqMhPTk5GCrwuotu+fbvcdtttoo3CXC6o+6NHjskfjDwXILjpowtkQ9tllcCReZ7sEWDIyY3cMAKYFeMN48aeuyM31wVwjtkFyUqtglfhX7FihTQ3N5dMZ55XSUfaw\/EHv\/hBnyrmOi0bDcw95EZuGAHMivGGcaukJmAl8rfKrOceHpLfsWNHIL7oxyyWs7fWaV6ljr2Ne2YlHKn72dcOH5q+g12FPU8Xv6DcjR0bDYwguZEbRgCzYrxh3CqhCVhJcKvMxN0uopk31781NDSU3KZm7OzV8WaLm9om3QM\/V\/vcVdyX3fVYUPzfa7lYtnxqMe6lHFqy0cCcQm7khhHArBhvGDeKO8ZthpWKtvbCwz3wcNbhQ2zsBXXmO10ZHx4NmAtxt4Vde+tFGo5nz728oGdji\/EjN3LDCGBWFHeMW7A1bXBwMLCe6xvhtAxpO\/JLfzku\/33v0aB+93cukc5rLgJJ5deMjS3mG3IjN4wAZsV4w7ilrQlYKcqzymxYHhmKL69q7tZpOrIWeu1Klo2Ge3zZKcmN3DACmBXjDeOWpiZgJSjfKhNxdzl0pvyq4Dmk6cj1D43J4LcnCt1rp7jjscbGFmNHbuSGEcCs0tQErATlW2Ui7uUXs7I5pOXIWum1U9zxeKRIYezIjdwwAphVWpqAPT0dK4p7inPuNw4fEj1DXj9FnWs3YcfGFvsBkhu5YQQwK8Ybxo3ijnHLnVVajizqgTVRDmOjgYUxuZEbRgCzYrxh3NLSBOzp6Vix555Sz\/3RZ0\/Kqq88FXild+Ui6V25MB0P5TQXNhqYY8iN3DACmBXjDeNGcXfklsZ97o6PgpKl4chV9z8VnEZXlFvfkkCy0UgiFP09uZEbRgCzYrxh3NLQBOzJ6Vll0nMvurjbC+muvXye7P78svQ8lNOc2GhgjiE3csMIYFaMN4wbxT2BW9QVr1Ema9asSTxGFnORm1W5jvzC\/x6Tr37r9Pa33Z+\/Sq69\/Dy3B1dxKjYamPPIjdwwApgV4w3jVq4mYE9N12rOe+7pVgfLrVxH1tJCOkOYjQYWa+RGbhgBzIrxhnErVxOwp6ZrlYm4p1vk9HMrx5H2Qrqib3+zybPRwOKQ3MgNI4BZMd4wbuVoAvbE9K0o7mWuljcL6WppSF7rykYD+zGSG7lhBDArxhvGjeJegpu9iG7x4sXS3d0to6OjkRZzfXkM6sgZC+kuO09233gVFklVaMVGA3MauZEbRgCzYrxh3FBNwJ5WGSv23Mvoudfa3nYOy5f\/I2RjizEkN3LDCGBWFHeMW+6sUEfW6pA8h+XxEKZIYezIjdwwApgVqgnY0ypjlUnP3QzRlzMsPzU1JX19fTIyMhKQKLV9Lvy8trY26e\/vl7q6ukiKqCNrcZW8AcjGFvtBkhu5YQQwK8Ybxg3VBOxplbHKRNzjiq4ifOutt8oXvvAFaWpqKllDvQ9eP729vWLEu6OjQ9rb22fYmZeAlpaW4Dvz74aGhti99Igj7SH5WtnbzmH58n+EbGwxhuRGbhgBzArRBOxJlbOaU3HXau3fv1+Gh4dL9qyjqm+LfRIePUxn3759sc9AHLnj8eOydueh4NEU9yQP8HuOeJQXAxR3jB+5YdwQTcCeVDmrXIi7CvXQ0JDU19c71bTUcbZRGVRC3O2z5A9+8YNO5S5SIjYamDfJjdwwApgV4w3jRnHHuM2wUmGfnJx07rlr+sHBQUmaRzcPKTWEb9IgjjTz7b\/\/sV+Q2z9xaQokqisLNhqYv8iN3DACmBXjDeOGaAL2pMpZZdJzL7WgTufCt23bljjnHkbg8lJg5tvVNmlBncl\/7969ibT\/+B9PyuDjJ4N0I59ulIZzz0y0KVqCiYkJaWxsLFq1Kl4fcsMQkxu5YQTcrVpbW2ckHh8fdzfOYcpMxF3rHbfQzSx882UzNjYmPT09MjAwEPli4Crs+lzft7RaH5JXZuwR+Ebs6fTkRm4YAcyK8YZx89UE7CmVtcpM3KN62i5D5nHV14V4cXP1Livk7Xx9HWmG5K+tsVPpbGZsNLAfJrmRG0YAs2K8Ydx8NQF7SmWtMhH3pPvcXVbL26vjk8TbZcgeFXd7C9yDn1sqH1nstgiwsm7MPnc2GhhzciM3jABmxXjDuFHcHbklibvLavnwITb2gjrzXWdnp8SdY1\/q\/HofRw4\/cVxuHK7dLXDG5Ww0HIM\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\/n5RAbc\/4e9KpVU725G86jURfZCAjYYbp3AqciM3jABmxXjDuFHcMW6pWJUS7Kieu+n1Rz3cduRNO5+WP3n8xdMjC3d\/NJWyFjETNhqYV8mN3DACmBXjDeNGcce4lW3lssp+bGxMurq6ZHJyMrhettR2O9uR3OPu5h42Gm6c2HPHOJEbuaVDAMuF4u7JLWqfu+\/xs2a+XR8dNSSvfw9vu9PrZvWjN9HF9dzN38\/+ne0y+dob0nDumTLy6UbPGtZO8omJCWlsJB9fj5ObL7HT6cmN3DAC7latra0zEo+Pj7sb5zDlGadOnTqVRbnS2OfuIuzhxXdaN+3F64U1AwMD04v57Drbb2nc4+4WDey5u3FiDxTjRG7klg4BLBf23B25pbEVLmmFvClKOeLOPe6ODuWCOndQoZR8KcLQkRu5YQQwK4q7I7c0xF2H1nX+PG4o3i5K1LB8KVvjSN7j7uhQirs7KIo7zMo2pLhjGMkN40Zx9+BWzrB83FY63Tanx9maveydnZ3TC+f0ZWBwcDAooeshNra48wCb0s5lo+ER\/FZSciM3jABmxXjDuFHcPbmlsaDO85FOyY0jeYCNE64gERsNd1bsgWKsyI3cyieA5UBxx7jlzso40j7Ahve4s+deiUDlSxFGldzIDSOAWVHcMW65szKONHvctYA8wIbiXolApUhhVMmN3DACmBXFHeOWO6uwuPMe92QXsbFNZhSVgtzIDSOAWTHeMG4Ud4xb7qyMI5fd9ZjodrhrLztPdt94Ve7KmacCsdHAvEFu5IYRwKwYbxg3ijvGLXdWxpE8wMbdNWw03FnZKcmN3DACmBXjDeNGcce45c5KHfn3\/\/Qd0Z67fnpXLpLelQtzV848FYiNBuYNciM3jABmxXjDuFHcMW65swqLO\/e4J7uIjUYyo6gU5EZuGAHMivGGcaO4Y9xyZ6WO\/PrfPCmrvvJUUDaKe7KL2GgkM6K4Y4zIjdzSI4DlRHHHuOXOSh355V375MbhQ0HZdn\/+Krn28vNyV848FYjijnmD3MgNI4BZMd4wbhR3jFvurNSRa\/7H38nmPYcp7o7eYaPhCCqUjNzIDSOAWTHeMG4Ud4xb7qzUkTf0\/7Vs3z8ZlI0H2CS7iI1GMiMOL2OMyI3c0iOA5URxx7jlzkodeeWtfy6PPndSeICNm3so7m6cwqnIjdwwApgV4w3jRnHHuOXOiuLu7xI2Gv7M1ILcyA0jgFkx3jBuFHeMW+6s1JHn\/t6f8HQ6D8+w0fCAZSUlN3LDCGBWjDeMG8Ud45Y7q4VXfkBe+5XNQbl49Kybe9houHHisDzGidzILR0CWC4Ud4xb7qxscefpdG7uobi7caJIYZzIjdzSIYDlQnHHuEFWJ06ckO7ubhkdHQ3s29rapL+\/X+rq6iLz279\/v6xevTr4bunSpTI0NCT19fWRaRd84OPyw2t7gu8o7m7uobi7caJIYZzIjdzSIYDlQnHHuHlbTU1NSV9fn7S0tEh7e7uYfzc0NEhvb++s\/MbGxqSrq0u2bNkizc3NsmvXLtm3b1\/sy4At7jydzs09FHc3ThQpjBO5kVs6BLBcKO4Yt1SsSgm2fnfkyJFI4Y96uC3uB7\/4wWA7HD+lCVDcsQghN3LDCGBWjDeMG8Ud45aKVZy4h3v5Lg9r+MTvy0+uWBUk5dGzLsS4pcuN0uxUbGwxcuRGbhgBzIrijnEr28rMv3d0dATD9PbHiPvKlSvlgQceCObok+bcL\/rUXfKvCz4UZMOeu5t72Ni6cQqnIjdywwhgVow3jBvFHeNWlpURb80kakGd+f7YsWPTi+g2b94sk5OTsXPuF\/7OV+WN+b8YlOvATbzH3cVBExMT0tjY6JKUaSwC5IaFA7mRG0bA3aq1tXVG4vHxcXfjHKY849SpU6dyWK7IIiUJuxpFDcvrAruenh4ZGBiQpqamWXkbcefRs+6RwB6BOys7JbmRG0YAs2K8YdzYc8e4QVZJK+TtTLWnvnDhwukhexX3jRs3ytatWyO3w83\/3J\/Jz86ez3PlPTzDRsMDlpWU3MgNI4BZMd4wbhR3jBtklTS0bmeqe9w1vdnbrv9fP1Hb5vTv9bc8EnzP0+ncXcNGw50Ve+4YK3Ijt\/IJYDlQ3DFu3lbhA2xMBmahnB5ko\/vgOzs7g33t+rEPsUk68MaIe+c1F4nuc+cnmQDFPZlRVApyIzeMAGbFeMO4UdwxbrmzMuLO0+ncXcNGw50Ve6AYK3Ijt\/IJYDlQ3DFuubOiuPu7hOLuz0wtyI3cMAKYFeMN40Zxx7jlzsqIO4+edXcNGw13VuyBYqzIjdzKJ4DlQHHHuOXOyog7T6dzdw3F3Z0VRQpjRW7kVj4BLAeKO8Ytd1YUd3+XUNz9mXFYHmNGbuSGE8AsKe4Yt9xZGXHn0bPurqG4u7NiDxRjRW7kVj4BLAeKO8Ytd1ZG3E\/c\/dHclS2vBaK4Y54hN3LDCGBWjDeMG8Ud45Y7KxV3Hj3r5xY2Gn68TGpyIzeMAGbFeMO4Udwxbrmzorj7u4SNhj8ztSA3csMIYFaMN4wbxR3jljsrFXcePevnFjYafrzYc8d4kRu5lUcAs6a4Y9xyZ0Vx93cJxd2fGXvuGDNyIzecAGZJcce45c5KxZ3nyvu5heLux4s9UIwXuZFbeQQwa4o7xi13ViruPFfezy0Udz9eFCmMF7mRW3kEMGuKO8Ytd1YUd3+XUNz9mXF4GWNGbuSGE8AsKe4Yt9xZXfg7X5V7bvr3wdA8P24EKO5unMKpyI3cMAKYFeMN40Zxx7jlzqoIjswaKhsNjDi5kRtGALNivGHciqAJZ5w6deoUVv3iWBXBkVl7g40GRpzcyA0jgFkx3jBuRdCEqhH3EydOSHd3t4yOjgbeamtrk\/7+fqmrqyvpvbGxMenp6ZGBgQFpamqKTFsER2IhjFux0cDYkRu5YQQwK8Ybxq0ImlAV4j41NSV9fX3S0tIi7e3tYv7d0NAgvb29sd4z6Q4cOCDbtm2juGNxHmnFRgODSW7khhHArBhvGDeKO8YtFatdu3bJvn37Svbe9+\/fL5s3bw6ex557KtinMylC8KdLxC03cnPjFE5FbuSGEcCsihBvVdFzj3JPkrjrMP6GDRuks7MzEHiKOxbkcVZFCP50ibjlRm5unCjuGCdyIzdDoCrF3cy\/d3R0BMP0ceKvf1++fLnTnHs6IcFcSIAESIAEikBgfHy8qqtRdeJu5tGVetyCOl1Et337drnttttkYmIiUdyr2oMsPAmQAAmQAAmECFSVuLsIu9ZPh+FXrFghzc3N4rJanlFBAiRAAiRAAkUiUDXi7rpCPrxlznbWjh07AsHnhwRIgARIgASKTKBqxF1745OTk057222Hsede5PBl3UiABEiABKIIVIW4x\/XGly5dKkNDQ8FBNroPXlfGh3vmFHcGPgmQAAmQQK0RqApxrzWnsL4kQAIkQAIkUA4Bins59GhLAiRAAiRAAjkkUNPirvP4g4ODgVu42G52dOqURldXV7DWIeksf5ulHgtc6rjfHP4OUi2SDzfz4PARy6kWqEoy8+EWnqqr5d+vDzc7ba3\/Tkv9LMzvMWqqt0p+TlKz4m6OptU5+2eeeSbYPqf\/v76+vlp8V9Fy2mKzatWqGWf7hx8cPi1Q\/71z586a5OnDzeaozNavXy+bNm2KPZipog6f48x9uIV3ztTyuhofbuaFSO\/j0LVJtfw7dRH2kZGRqu701ay4mzPnNdCL8JaWdtscbjD1ZWh4eNhpt0ItN7YIN210b731Vjl58qSUOnUxbR\/nKT8fbpp248aNsnXr1pp\/GfflZt+QWcu\/07jYNyMbV199tRw7diy4mKxat0\/XpLjH3TJnbp3LU6M3V2WxRzZ0NCP871LlquVGA+GmL5rXXHONfOMb35i++XCu\/D5Xz\/XhlnSvxFzVYS6e68MtqueedPnWXNRpLp\/5wgsvBI\/XHVh6xTjFfS69ATw7qqeuDezChQtrckg0CmG4p+7TW0LPJABcmTsTX27mqORbbrkluOioVl8wfbipuB85ciTwfa2vmfHhprxM26dDzmvWrCl5ZXbuflwZFij8IpTho1N7VE333O3FEhT3mTHl22gYa21477333ppdUOfDTRvaL3\/5y\/LpT39aGhsbS65rSO0Xn9OMfLiZ9QlmEZ3arlu3riZjzoebGXLesmVLMNTsMxqX07CpWLEo7hVDW9mMOSyfzNdnuI\/C\/iZPH26a9pvf\/GbQe6r11fI+3MLD8rXMjtyS2zIkBcUdoZYTG7unzgV1s50SHoZPWlDHlbenGfpws7cP2h6oxeFSH27hWKzl368PN74UuYsPxd2dVe5ScitcaZf4bLGp5WHRMEUfbrZtLfc+7blgXXOQtPUy3PDW8vCyT7xFDcvX6nRGkiBR3JMI5fx7HmJT2kGlDscwi5p0SDmuB1qrB4u4cqO4z4w\/H272ITa1fhiLDzd9EVq9enUAvta5lWr9KO45F28WjwRIgARIgARqkUBNrpavRUezziRAAiRAArVDgOJeO75mTUmABEiABGqEAMW9RhzNapIACZAACdQOAYp77fiaNSUBEiABEqgRAhT3GnE0q0kCJEACJFA7BCjuteNr1pQESIAESKBGCFDca8TRrGZ5BJ577jmZN2+e8xWjuk\/21Vdflcsuu6y8B5ewNucLLF26VIaGhpzLVi239pn6ZbUfO+vnVSwwmDEJiAjFnWFAAgkEfE9Ay+IAjHIEuhzbLINFxVY\/elBSVp9qYZMVDz6neglQ3KvXdyx5RgTyKO6+ZbJRVYuAUdwzCnA+ppAEKO6FdCsr5UvAPs5Ubc1Q9zPPPDN9XKf+3RypGz5y1wwdn3\/++dLd3S2jo6NBEcwlMPY92nb+9fX1sUW1n2EPTZsrT43hpk2bpL29fVY+cemMuOsZ7nfeeWdgFx76to8pDT9HWd16663y4Q9\/OLA3rF555RXp6uqSycnJwOT222+X3bt3y8DAgDQ1NQV\/i6tTFISwuOu\/X3\/99eB\/eh+5zTfKPnxRiqaJ+ls1vvj4xjfT1x4Binvt+Zw1DhGIurTFFpZwLznudi3Ntr+\/P7i+VQVeh5P13mzz4tDR0TEtwqVu0TPlMfnV1dUFonTvvfdO31me1HMPp7cvDdEXEBXhq6++OiivyX\/nzp3B3L2KdE9PzwxRtvMzLzALFiyYtg\/X0fz7pZdeCsps7qvXlwgzzJ504VCUuA8ODop5mYniaruW4s6fei0ToLjXsvdZ94BAkkgkCWm4RxgW96jrckvdAhc1bB5OX6pMSTfMhW8H0\/InDdXb3xtxD7+s7Nu3b1rsNU9bvPXfGzdulK1bt85Y+Fdq6D1K3HVUwLyQmGdouqgFhRR3\/sBrmQDFvZa9z7pPE7CHsMOrz+OENDx03dbWFtlzDw+P29ijhtTjnmffxFdK3JMW9EUJedTfwlMV4akHMzKh9YkSaTtPHQ0wt5GFwy7u\/voocVdbe4pf6dkAAAIfSURBVIFdqZcSijt\/4LVMgOJey95n3WcRsAXNnne3e4dGrMPz4KbnGu65q224x1kKfZxwl5oqsPMrV9w1LzN3bl4+onruPuL+5JNPihn2L7XOwK4HxZ0\/UBLACVDccXa0LDABWyBNz1SHfnV+uq+vT1paWmYsYrMFPCzupebXoxBmMSwfnlO3n6lCXGqI3QzL2+Ie1Uu2h+W1575u3brpNQMuoVOJYfmkF62k6QmXcjMNCeSBAMU9D15gGeaUQNScu917theYRS0MMz15MyyvlbFfAEz+urjODClHzXsbCGktqLN7yvY8\/PLly2ctmAuLu21ryqrl08VxUeLuuqBO8zBz5klrHcpdUGemTcwOB1MPeyFhOPAo7nP6U+TDUyRAcU8RJrOqXgKm4TfbuOwhd3sbmw5TX3\/99TO2u6mo33DDDXLHHXdM90zDgm9682aLnJIyohNHrdS2MddFfuvXr5\/OPmqI3WxRC4ta+NlbtmwJ5tV1EZ2pv91z14eEGYa3woW3A6pN3DY+M1qi\/zUvRFFb4Wz7KGG21zuon5YtWyYHDx4MXjDCL2GmDuFRjeqNapa8lglQ3GvZ+6w7CVSQgIpt1Ap510e6zLm75uWajj13V1JMl3cCFPe8e4jlI4EqIBBeV2B66fa+dt9qUNx9iTE9CbxJgOLOaCABEkiFQPjUvrgtbq4PC1\/k8uCDDwamlTprnhfHuHqG6aqBwP8HCmN3rbs7Ob0AAAAASUVORK5CYII=","height":303,"width":503}}
%---
%[output:9dac27de]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAfcAAAEvCAYAAABYGIhlAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10XdV15zcNSREhBAtDiBCuDcjFKYmNCamiQJyMQt2klZnJpJXkaZuqSsaZYJhV8JLkEFh1ILasselQIBOVUbycWbXsdhUmVj+WS12ahBoXarCmMzFBIH8ghBPAOJBEaReJZ+3rHnF0de+75\/zffVf33fd\/a2UR65197jm\/vd\/53\/N9xqlTp04JPyRAAiRAAiRAAoUhcAbFvTC+ZEVIgARIgARIICBAcWcgkAAJkAAJkEDBCFDcC+ZQVocESIAESIAEKO6MARIgARIgARIoGAGKe8EcyuqQAAmQAAmQAMWdMVA4AidOnJDu7u6gXkNDQ1JfX1+xOo6NjUlXV5dcffXV0t\/fL3V1dRV7lp1xlnV0qZDhcNNNN0l7e7uLSa7TbN68WQYHB4MyrlmzRnp7e73Ku2vXLlm\/fr1s2rQplzy0fvv376\/478MLGhOnSoDinipOZpYHAlkKX5y4a+O5YsUKaW5urgiSuDpW+rlxlUHEQsXlm9\/8ppdwIja+DtBnrF69etqsiOJetJcxXx\/XQnqKey14mXXMjMDU1JT09fXJyMiI7NixIzNx1xGDLJ4bBdIIRVtbm7NQm56tj3AiNojjjbj7lC38nLz33E2cHjt2jL13JEiqwIbiXgVOmssi2sOTWg57mNHutS5btkzuvPPOoKjayNtD1KaXOTo6Out7O48bbrhBPvOZzwRpGhoaZNu2bdLU1BRZfVtEw+nDvVr93gzTt7a2yt133y1Lly4NGjVbFJPy0eF989wDBw4E5dOPGZa\/44475Etf+lIg7OYTZqF\/N0xt8TeCYqc3AmHyssXGruN9990nAwMDkc+dmJgIyjc5OTldJtuHNkdlfv\/998vXvvY1MfVT\/mHWhp2Z7ogSMuNX+7mmvuF6GV83NjZOv6CE+e3evTsY5jYfOz7C+SW9VIXjsVRepeKwVA\/fZqJlNmUPvzCEf182W5PHLbfcInv37hX9\/Rjf2XXWv5ln2L5N4hIVh3PZ1vDZ6RKguKfLszC5hRt0u2KmgYpqwMONsuajwmqEPfx9lPiUEkb9Lq5spiE+\/\/zzZ8y5G3G3y6AiGiXGtsCH80lL3KN6huGGNtzox3HVv8eJe09Pj6xdu3YW+1Jiqt9dcMEF8tJLLwUvL1GCq8+0RShc9ri4MM998sknI4X6wQcfnJ7ntuPNFq+wuIfzMt\/HCXypmFWbo0ePxr5E2GUKC3v4BcwI63XXXSff\/va3Z7QLUQId9fsKi7OmiSqj\/t08Jylvm0veRxcK05jOUUUo7nMEPu+PNY2X3XMxDaOW3e61au\/MNBp242k3RHaPxRYDFVDTszR5mGeHe4iGWdT3dkN1\/fXXx4q73bPxzSdJ3HW0Qj9Jw+OlRhZ0NOGVV16ZxcTubSqnxYsXz6ijy7B8eMrAsDf+1F562O9aFp1\/jhpRUJarVq0K6mv39F0WGboMsYfThP9tmJgXES1\/0rNN7EXVx\/xNXwK1znHD8jZHE09hnz788MPBS0JUTzwu3\/DojRmtsPMo9WzTszfxn8QljemHvLdjtVw+inste79E3ePe6k3jqI3a8uXLI1eK22mOHDkS2RvTR9t5aG\/RrGxPWhCX1OOIE0+7sdPn++aTlrjbz1ah1o8tJnGNri1un\/3sZ53FPWqkI+q5Wo7wtENcz1jTqkiZcthso54Xnp4oJe5x0xFhm1K98KgXw3DdzJRP+CXBvNDEiXBSfNr+tfOI82t4FMCwMuIeNx1j7wSxY9n8Lu0pEfNTt7lETQWxOSwOAYp7cXyZak2yFnd7K1lS4+krygomamucaz62cIWFQPO2t8K59Nw1jb0ITf+t267CIxdhcfEV9zDHcO8+\/FKRlribQIx7qdAdBFHiHh7eT+q5V4O4R40UGb+G4y+u527nEffboLin2vwVIjOKeyHcmH4l0GH58PCxmcOM6wVFDaMmiXup4XS7N6lUtHcTJ+6u+ehwZ1h4zXRFWNxVQF0WKoWFz+7Zhqc2VAyThuV1VCFJHMN5oMPydrTF9YbDERkW6nAv1tTZHsEx9TGxE7aJGpZP+iVUeljevAiaEY84cY8a8TCMwj33uAWQ4SmBUsPyUVw4LJ8ULdX9PcW9uv1XsdJXekFdKXFMEvdSZYuaj44T96R8dAjTzJ+HQbuIu9pErZY3eYVXPNuHv\/gsqDPDs7aNPveTn\/xkMKoQ9VFOUfVzXVCneZoXnvBLRdxiM9vGTqPPvOeee+Suu+6atfhPbcLirn+LW5xn6pr0Mhk1ZJ00cmJzjKtjKWG2xfTmm2+Oja1SeWgZohbauS6os7kkjVxVrHFhxpkQoLhngrl6H+K6Fc7expa0FS5qkZ7PsLzSLDXkm7RgzT6xrlQ++pzwtqnbb79dDh48OL2ALKrnbjf8cYsC7bzDawGixN8WOdtW\/78R96jnPvDAAzNOWtODdez5fWQrnC3StthEbZOM24IX5mqvAdA8lduWLVtk3bp1AQ57BMbseojbWpe0P73UVjh9lmuPNm6uXEdvooQzbrRCGUVtQ4zq\/ce9GOrfwyfixa1dMHm4jDBVb8vFklPcGQMwgaSVyXDGNMyEQNTwf3hHRNw5A3YBkUNsMqlgAR9iv4yZl5ioFfRJVechNkmEqv\/7qhR3bUx0D68e3BHV+JQ6NKX6XZafGlDc8+MLpCSlpiVKTSdEPQs5fhYpM22ih+WVS9LBT1EvZEW5C4BxMZtA1Yl70gIe831LS0twYYP5twa+7+UPDJjSBCju1R8h4RdhrZGvsKsNzyrPNhbC02U+wq4l5ctYtv6ai6dVnbjrvJUGpn7ieu5hkDr3tG\/fvkxv7ZoLZ\/KZJEACJEACJKAEqkrctZexYcMG6ezsDASe4s4gJgESIAESIIEqH5bXHrh+9ASmUnPudjXNsGNHR0cu71VmUJIACZAACZBA2gSqpueuc3rbt2+X2267TfSyERdxN\/PtCs2+pSwM8dJLL02bK\/MjARIgARKoUgJ6C9+iRYuqtPSni1014q7D8LpHV0\/rSlotrxVzFXZNq+I+Pj5e1Y7MuvBkhhEnN3LDCGBWjLfa5VYV4h61ote4LOpqR98V8vwB+P8AyMyfGV8kMWbkRm44AcyyCO1bVYh72D1JPXft5eupT6WG4u08i+BILIRxKzLD2JEbuWEEMCvGW+1yK4S4m566rqI391ybIyqNa0sdA8ofgP8P4PDhw1U\/J+Vf6\/ItyA1jSG7khhHwtxp+4rjcOHxITtz9UX\/jHFlUpbinzY\/i7k+Uja0\/M7UgN3LDCGBWjDd\/bhR3f2a5taC4+7uGjYY\/M4o7xozcyA0n4G9JcfdnllsLiru\/ayju\/swoUhgzciM3nIC\/JcXdn1luLSju\/q6huPszo0hhzMiN3HAC\/pab9xyRzXsOc87dH13+LCju\/j6huPszo0hhzMiN3HAC\/pYUd39mubWguPu7huLuz4wihTEjN3LDCfhbUtz9meXWguLu7xqKuz8zihTGjNzIDSfgb0lx92eWWwuKu79rKO7+zChSGDNyIzecgL8lxd2fWW4tKO7+rqG4+zOjSGHMyI3ccAL+lhR3f2a5taC4+7uG4u7PjCKFMSM3csMJ+Fvq6XS6HY4n1Pmzy50Fxd3fJRR3f2YUKYwZuZEbTsDfkuLuzyy3FhR3f9dQ3P2ZUaQwZuRGbjgBf0uKuz+z3FpQ3P1dQ3H3Z0aRwpiRG7nhBPwtKe7+zHJrQXH3dw3F3Z8ZRQpjRm7khhPwt6S4+zPLrQXF3d81FHd\/ZhQpjBm5kRtOwN9y1f1PyaPPneSCOn90+bOguPv7hOLuz4wihTEjN3LDCfhbUtz9meXWguLu7xqKuz8zihTGjNzIDSfgb0lx92c2JxZjY2PS09MjAwMD0tTUFFkGiru\/ayju\/swoUhgzciM3nIC\/JcXdn1nmFlNTU9LX1ycHDhyQbdu2UdxT9ADFHYNJbuSGEcCsGG\/+3Jbd9ZgcO\/ETzrn7o8vOYv\/+\/bJ58+bggey5p8udjQbGk9zIDSOAWTHe\/LnV3\/JIYMQT6vzZZWJx4sQJ2bBhg3R2dgYCT3FPFzsbDYwnuZEbRgCzYrz5cdMeu\/bcKe5+3DJNvWvXruB5y5cvd5pzN4Xbu3dvpuWs1odNTExIY2NjtRZ\/zspNbhh6ciM3jIC7VWtrq7wx\/xflh9f2UNzdsWWbUhfRbd++XW677TbRRoEL6tLnzx4BxpTcyA0jgFkx3vy46YUxeogNe+5+3DJLrcPwK1askObmZuFq+cpgZ6OBcSU3csMIYFaMNz9uFHc\/Xpmm1rn27u5uGR0dnfXcHTt2BIIf\/nArnL+L2Gj4M1MLciM3jABmxXjz42a2wf3cj1+Wl7\/6G37GOUt9xqlTp07lrEypFoc991RxTmfGRgPjSm7khhHArBhv7tzsxXRnvvxd+f7XP+dunMOUFHcRYc\/dPzLZaPgzY88dY0Zu5IYTcLd89NmTsuorTwUGZz29Wyb\/6g\/djXOYsvDi7sKc4u5CaWYairs\/M4oUxozcyA0n4G6565+Oy3\/ZcXox3TmPDsixx\/\/a3TiHKSnu7LlDYUlxh7Bxzh3DRm7kBhJwM7OH5BfUnyWvfe0\/yfj4uJtxTlNR3CnuUGhS3CFsFCkMG7mRG0jAzcwekt\/6qV+UO3\/rWoq7G7p8p+KwvL9\/KO7+zDi8jDEjN3LDCbhZmlXy2mvf\/fmr5CPvfw\/F3Q1dvlNR3P39Q3H3Z0aRwpiRG7nhBJIt7V77bze\/W+75zSsKsciaw\/Iclk+O\/ogUFHcIG4eXMWzkRm4ggdJm4bl27bVr770IHT6KO8Ud+tFQ3CFsFCkMG7mRG0gg3kyFfe3wIXn0uZNBovs7l0jnNRcF\/5\/injruucmwCI7MmhzFHSNObuSGEcCsGG\/R3FTYN+85LHrcrH6uvew82X3jVdOJi6AJ7LkX5C0N++njVmw0MHbkRm4YAcyK8Tabmwq7irqKu350GP7gFz84IyHFHYu33FkVwZFZQ2WjgREnN3LDCGBWjLeZ3FTY9RQ6\/a8RdjPPbqcsgiaw586eO9RqsNGAsHHuGMNGbuQGEnjTTFfFr915KFHY1YLiXjbufGRQBEdmTZLijhEnN3LDCGBWjLfT3P7XP74o\/3XX09MQzX52\/W\/UpwiawJ57Qd7SsJ8+bsVGA2NHbuSGEcCsajnedOg9WBFv9daV4p9+9n3ysSXnlwRKccfiLXdWRXBk1lBrudEohzW5YfTIjdxcCcSJelJv3c6\/CJrAnjt77q6\/mRnp2NhC2Dh3jGEjN3JzIhCeV1cjH1E3D6G4O+HOf6IiODJryhR3jDi5kRtGALOqlXh7\/MgP5Ff\/6MkZkFTUf\/eDF8snr7owEHifTxE0oWI99xMnTkh3d7eMjo76MJWlS5fKQw895GVTbuIiOLJcBr72tdJo+HJJSk9uSYSivyc3cgsT0L3qw4+\/OH3CnPlehbzvVxdJy6XneYs6e+4OcWbEvbe3V5qbmx0sRPbv3y+bN2+OFPepqSnp6+uTkZGRIK81a9aI5h332bVrl6xfvz74uq2tTfr7+6Wuri4yOcXdyT0zErGx9WemFuRGbhgBzKpo8abD7sNPvDh9spxNBRl+j6NaBE2oeM89LXFX0deP5mdeHDo6OqS9vX2Wf8xLwtDQUCDo+lLQ0NAQ+zJQBEdiP33cqmiNBk7Cz5Lc\/HiZ1ORWm9zMYTP2GfBhQf\/QZedJ5zXvlmsvPw+DFGFVBE2omLinRjkmI1vsw0m0175v377p3nr43+H0RXBkpXmH82djixEnN3LDCGBW1RhvKug\/O3VKbt759Kwhd0NBe+n3dSxJVdBtwkXQhIqJuz3nnjSE7hu2SUP+UT33lpaWyF6+PrsIjvRlWG76amw0yq1zGvbkhlEkt2JzC1\/kElXbSgs6xd0zxrSHPTg4GFjp0Pi2bdukqanJM5c3k5v8kubRx8bGpKurSyYnJ2XHjh0l5\/1V3M1n7969cNlqyXBiYkIaGxtrqcqp1JXcMIzkVgxuk6+9IQ3nnin63z9+\/KSMHPphbMU03QcuqZPu978zsKn0p7W1dcYjxsfHK\/3IiuZfsZ57uNTh1fPl9uZV5FW4oxbK6TD8zp07Refc6+vrg0V6+olbgMeeu3+MsSflz0wtyI3cMAKYVV7iTRfCDew5HDvMbmqnvfM\/ar9CFp5fB690x0jNtCqCJmQm7jY6u1eN9uY1j56eHhkYGJgxEmBW1dvD8HFpTZmK4Mg0Atonj7w0Gj5lzkNacsO8QG7Vw02FfM93XpbR5193EvPPffgS+cSV8+dUzMN0i6AJcyLuNkh7flx72a6fODuKuyvB8tKxscX4kRu5YQQwq0rFm86Ray9b\/\/vwoVfkGwe\/7yTkWovNn1wsb3\/bWyq2GA4jxZ57GtzE7rlrhklz4kFAWEPrRsDjtrdFDcvHDeFr3kV4S0vFMR6ZVKrR8ChCVSYlN8xt5Da33Mx57X\/79Cvy5NHXEoVcS6vir9vUelcuylWv3IVkETQhs557+BCapAVxYQeUsjffdXZ2Ti+csxfyJT2rCI50Cdg007CxxWiSG7lhBDArJN50WF2FefOew\/IPz52cvv+8VAk0\/YJ5Z8m6X1koP3fGGbnulbuQLIImVFzcbZF17aW7wE8zTREcmSYPl7yQRsMl36KnITfMw+RWGW4q5A8d\/J6Mfe\/HTr1x0yNXIb\/l+oVy6fzTp376nt2O1SY7qyJoQsXE3V4dn9Rzzs5l0U8qgiOzZsjGFiNObuSGEcCsNN5e+Om8QHzv+\/tj8vSLP3IWcSPa110+T9rff1GQR9FEPI5qETShYuL+7LPPys033yx33HFHKmfLY6HtZlUER7rVNL1UFCmMJbmRG0Yg2UrnxX\/6s1Oy5eEj8vyJn3iLuPbGP3NdoyxrfEche+PJBN9MUQRNqJi4J50iFwW61MUxPo7xTVsER\/rWudz0FCmMILmRG0ZAgrlv7TnrUProxOuy5\/+9LMde\/YnTnLh5ppkb\/+x1jbK08R010xP3ZV4ETai4uPPKV9+wqo70FCnMT+RGbqUI2FvMHhs\/KX\/y+Isip8SrF26G07Un3rb4rbJkUWNNDaljETbTiuKeBsUc5FEER2aNkSKFESc3clMC5razbz\/7qux64ngA5dHnTnrBMfPfZk58WtTrz5rOh\/HmhXQ6cRE0oWI9dwzp3FgVwZFZk2OjgREnt9rhZraUfX3\/pDx++AfeQ+hGrPW\/ul\/841deIO+7+JwAoOvCNsYbFm9F0ASKOw+xgaKfjQaEjWfLY9hyy00F\/Cdv\/Eweeup73ovYDAoj1DqM\/tvNDfLLi97pJeClkPJ3igUcxR3jljurIjgya6hsNDDi5FY93MzQuf73L\/75JfnO5A+h3ne4B\/7r771AfqnBrweOUeNFRSi3ImgCe+7suUPxT5GCsOW2B4rVJjurSsWbGTr\/n49OyMHnX4fF2wi49r6vuOjtsmrphdND565D6JWgWSlulShrnvKkuOfJG2WUpQiOLKP6kCkbDQgbxR3D5s3NXnWu\/\/+fjv5A\/u7pE2WLtxb\/I4vr5Xc\/2CA\/\/JefBsesmmeBVauoGX+nGN4iaEKmPXe90GX9+vUBbb0s5ujRo7Jv377IO9kxl2BWRXAkVnPcio0Gxo7c0uNmhs3HX\/6x\/NmB03Pevvu+7dLYc9+\/+kvz5dffd0Eg3Crg1fphvGGeK4ImZCbuesa83symd7CvXbtWent7ZenSpdLX1ydxt7thbvG3KoIj\/WtdngUbDYwfublxM8Ktqfc+\/YrseOyY1J11lvd2sUjxrq+Tz1x7sdSf\/dZcDJ27EcFSMd4wbkXQhEzE3T6tbvHixdLd3R2Ie3Nzs6D3uWMui7YqgiPT5OGSFxsNF0qz05DbaSZGvE9OvSHb\/uEFee6lH5fV69Y8Tc9bt439ynvmy1WXvKPqe95YlL1pxXjDCBZBEyjuXFAHRT8bDQib99wx9pS5t9KFao3zfl7u\/\/vn5bvHf1S2cDece6aceeaZwbWiyxacK5\/50MXTwp3nOe+59gR\/p5gHKO4e3HS+XefX7WF504vv6OiQ9vZ2j9zSTVoER6ZLJDk3NhrJjKJSFIGbCvcl9T8v9z2SjnDbvW49be1Ty98li\/7tKlHzXRG4YRFTnhW5YfyKoAmZ9NwNXh2CX7169QzamzZtmlNh18IUwZFYCONWbDQwdnnmpj1g\/d9F73xbINzjKQyV28Ktve6PXzlf3nvxmxeWuG4TyzM3LBKysSI3jHMRNCFTcccwn7aampoKFt+NjIwE\/16zZk0wbx\/3sV8kdOHe0NCQ1NfXRyYvgiPLYYvYstFAqM3doSJGuH\/8rz+Vb4x+v+yV5ab29grzZZe8I5jrnv6bdcY5RutNK8YbRpDcMG5F0ISqEXddba8fFXSzQC9uOH9sbEy6urpky5YtwaI9MyXQ398vdXV1s7xdBEdiIYxbsdHA2FWCW9QlJOVsCYsS7uW\/cK587Irzg6+Ca0NTFG4XkpXg5vLcak9DbpgHi6AJmYi7EeOk61+TeuO2m2yxD7tPxfzIkSMle\/a2TREciYUwbsVGA2Pnys0++vSCc94qDzz6gjzzvfIXpkUJ979bcr68f8G508JtBByrYWWsXLlV5unVmyu5Yb4rgiZkIu6KVwV3586dM4bH7R74qlWrnPe821vrtGduf8zwfUtLi\/NcfhEciYUwbsVGA2On3N7yzncHxirgTx57Tf720Cun\/\/3q6Tnvcj72MPkl9WfJ4ne9Xf7DsgtnnKKWda+7nPoYW8YbRpHcMG5F0IRMxL2UGNv73J955hnRHvlDDz0U6xH9fnBwUNra2iJPtjPivnLlSnnggQdERwtc5tzNA\/fu3YtFQ41ZTUxMSGNjY43VOrm6B174ibz7HWfKY8em5J+P\/4tMvvaGvPj6G8F\/y\/3odjD9aP5LLnybXLvwbLn43\/6m3+kzTJpyn5U3e8Yb5hFyc+fW2to6I\/H4+Li7cQ5TVp24G4bmxLvwPLoR92PHjk2PEsSlNXkV4S0t69iqtR6B6VE\/ffxHqS5IU7\/Zve2F8+vkN65+l5whZ1RkYVrWcZLW82ot3sgtLQJYPkXQhEzEXfEmDcvrPnez8O2ee+5J9IgumtM98wMDA9LU1DSdPmpYPi4txT0Rc2yCIjS29rz2OWe9Rb7+2KQ8+\/3yT0qzodnC\/eHF82Rh3ZRcdNG7c3\/hCB4ZlbEsQrxVhkzpXMkNo05x9+QWtc9dL5AxK9rvvfde2bZt2wyxjntEqWNrtae+cOHC6Tl3FfeNGzfK1q1bI7fDFcGRnq4oO3neGw1zlefoxOvBnPbhl6dSmdM24GzRvvLic4L929rbNj3xuHntvHMrOzAqlAG5YWDJDeNWBE3IrOeOIX7Tyl4db3rncRfOhIW\/1Mp6fUIRHFkuX1\/7uWo0TG\/7Oy\/+SEb+T3r7taNEu+ldb5dfu3K+XH7h2akddTpX3Hz9m7f05IZ5hNwwbkXQhKoR9\/AhNvaCOvNdZ2dnMAqgH3uUIG7xHYflscBXqzQbDXuF+KHjP5K\/+r8vyeGXKtfTvuLdb5f\/fF2jvO0tPzcNIKsV5Glyw71XfZbkhvmM3DBuFHcPbuZgGb32NfxJWs3u8RgoaREcCVW8DCPXRsMMjz\/y3RPy+JEfpHYyWlxP+9ffO1\/e+m+incd7uF25leGaQpqSG+ZWcsO4FUETMum524vczH527WWHr3\/F3FC+VREcWT4F9xy0p\/3888\/LqbdfIPPOPlO++q0JOfpK5XrayxecK4vfdbYsqD99umAljjd1r315KdnYYvzIjdwwAphVETQhE3EP73O3F7zp8Pnw8HDknnXMLf5WRXCkf63jLcw55D+YeiMYIn9eLxRJ4YCVGcI87yxpvvQ8+a1fPn2gS7gnnmZ98pQXRQrzBrmRG0YAsyqCJsyJuNvHw5Za9Y65xd+qCI70qbWZ437u5R\/Lnz\/5fTn2ypQ8+txJnyxmpbVXj7+n4Rz5tffOl1+or0ttIVpZhcuRMUUKcwa5kRtGALMqgiZkIu6K116xbgv6ww8\/HNzzHnepC+YaP6siODJcYxVwFVz9758eOC7feuZVWMCNcH\/osvPklxe9Uy6df7YcP\/6ifOA9i4LHZrUYzc+r+UxNkcL8Qm7khhHArIqgCZmJe\/hwGXOMrG5nc93bjrkp2aoIjtRaqpD\/t785Esx\/+\/TEg1u+5p0levPX77Vc7HQOORvb5LiKSkFu5IYRwKwYbxi3ImhCZuKOIc7GqlodqSvRh594UYafOJ4Iygj4dU3zpP39F00PlycaxiRgo4GRIzdywwhgVow3jFu1aoJd20zE3fXimPr6eswTZVpVkyO1d752+FDJnrkKuQ6hd15TuWNO2WhgQUdu5IYRwKwYbxi3atKEuBpS3HN+Qp1ZuT6w53CsoKuY3\/iRS2TJRecEZ5Zn8WGjgVEmN3LDCGBWjDeMG8U9gZuuil+\/fn0i3TVr1khvb29iukolyKMjVdS\/Nfaq3Lzr6VnVVjH\/yOJ6+dTyd2Um5uFCsNHAopHcyA0jgFkx3jBuedQE35rMec\/dt8CVSJ8nR8aJugr6iqZ5cuv1C3OxOp2NBhaJ5EZuGAHMivGGccuTJmA1EMlE3NHCZWWXF0fqArm1Ow8Fi93MR0X9vo4lgaDnacsZGw0sOsmN3DACmBXjDeOWF03ASn\/aiuKekzn3zXuOyOY9h2eJelZz6L5BxEbDl9jp9ORGbhgBzIrxhnGjuJfgZlbIj46OJtKt5YtjtJe+6itPTffWtXd+z29eISsWz0vkNpcJ2Ghg9MmN3DACmBXjDeNGcce45c5qrhyp+9NvHD40o7e++\/NX5Wr4Pc5ZbDSwMCY3csMIYFaMN4zbXGkCVtpoKw7Lz8GwvPbW\/+KfX5IvfuPZaa\/s+uz75Pol56fp24rmxUYDw0tu5IYRwKwYbxg3irsnt6itcZs2bZL29nbPnNK5kOhIAAAgAElEQVRNnqUjzVnvG\/\/69Py6WTCX17l19tzTjTU2thhPciM3jABmlaUmYCVMtsqs567CvnPnThkaGhJzEp2Zl+\/o6EgUeHM2\/cjISFAr173xY2Nj0tPTIwMDA9LU1BRJJEtH\/uHeo3LnX45PC3u1DMOHwbGxTf5xRaUgN3LDCGBWjDeMW5aagJUw2SoTcU\/j+Fn7VjnXlwLzQnDgwIGSl9Nk5Uh7jl177NUq7BpWbDSSf1wUd4wRuZFbegSwnLLSBKx0blZVI+7h6thiH1dVc7Wsfj\/XPfewsOve9Wobirc5U9zdfmAc8cA4kRu5pUMAy4Xi7sGt3GF5+1GlRgJMOk2zYcMG6ezsDO6Sn2txr7\/lkaBo1d5jN3wp7h7BbyUlN3LDCGBWjDeMG8Xdk1saC+rMPfBtbW3S398vdXV1kaXQZ+ln+fLlTnPuJpO9e\/d61io5edv2CZl87Y0g4R98bL60LTkn2SjnKSYmJqSxsTHnpcxf8cgN8wm5kRtGwN2qtbV1RuLx8dNro6r1k8mwfCXgqMhPTk5GCrwuotu+fbvcdtttoo3CXC6o+6NHjskfjDwXILjpowtkQ9tllcCReZ7sEWDIyY3cMAKYFeMN48aeuyM31wVwjtkFyUqtglfhX7FihTQ3N5dMZ55XSUfaw\/EHv\/hBnyrmOi0bDcw95EZuGAHMivGGcaukJmAl8rfKrOceHpLfsWNHIL7oxyyWs7fWaV6ljr2Ne2YlHKn72dcOH5q+g12FPU8Xv6DcjR0bDYwguZEbRgCzYrxh3CqhCVhJcKvMxN0uopk31781NDSU3KZm7OzV8WaLm9om3QM\/V\/vcVdyX3fVYUPzfa7lYtnxqMe6lHFqy0cCcQm7khhHArBhvGDeKO8ZthpWKtvbCwz3wcNbhQ2zsBXXmO10ZHx4NmAtxt4Vde+tFGo5nz728oGdji\/EjN3LDCGBWFHeMW7A1bXBwMLCe6xvhtAxpO\/JLfzku\/33v0aB+93cukc5rLgJJ5deMjS3mG3IjN4wAZsV4w7ilrQlYKcqzymxYHhmKL69q7tZpOrIWeu1Klo2Ge3zZKcmN3DACmBXjDeOWpiZgJSjfKhNxdzl0pvyq4Dmk6cj1D43J4LcnCt1rp7jjscbGFmNHbuSGEcCs0tQErATlW2Ui7uUXs7I5pOXIWum1U9zxeKRIYezIjdwwAphVWpqAPT0dK4p7inPuNw4fEj1DXj9FnWs3YcfGFvsBkhu5YQQwK8Ybxo3ijnHLnVVajizqgTVRDmOjgYUxuZEbRgCzYrxh3NLSBOzp6Vix555Sz\/3RZ0\/Kqq88FXild+Ui6V25MB0P5TQXNhqYY8iN3DACmBXjDeNGcXfklsZ97o6PgpKl4chV9z8VnEZXlFvfkkCy0UgiFP09uZEbRgCzYrxh3NLQBOzJ6Vll0nMvurjbC+muvXye7P78svQ8lNOc2GhgjiE3csMIYFaMN4wbxT2BW9QVr1Ema9asSTxGFnORm1W5jvzC\/x6Tr37r9Pa33Z+\/Sq69\/Dy3B1dxKjYamPPIjdwwApgV4w3jVq4mYE9N12rOe+7pVgfLrVxH1tJCOkOYjQYWa+RGbhgBzIrxhnErVxOwp6ZrlYm4p1vk9HMrx5H2Qrqib3+zybPRwOKQ3MgNI4BZMd4wbuVoAvbE9K0o7mWuljcL6WppSF7rykYD+zGSG7lhBDArxhvGjeJegpu9iG7x4sXS3d0to6OjkRZzfXkM6sgZC+kuO09233gVFklVaMVGA3MauZEbRgCzYrxh3FBNwJ5WGSv23Mvoudfa3nYOy5f\/I2RjizEkN3LDCGBWFHeMW+6sUEfW6pA8h+XxEKZIYezIjdwwApgVqgnY0ypjlUnP3QzRlzMsPzU1JX19fTIyMhKQKLV9Lvy8trY26e\/vl7q6ukiKqCNrcZW8AcjGFvtBkhu5YQQwK8Ybxg3VBOxplbHKRNzjiq4ifOutt8oXvvAFaWpqKllDvQ9eP729vWLEu6OjQ9rb22fYmZeAlpaW4Dvz74aGhti99Igj7SH5WtnbzmH58n+EbGwxhuRGbhgBzArRBOxJlbOaU3HXau3fv1+Gh4dL9qyjqm+LfRIePUxn3759sc9AHLnj8eOydueh4NEU9yQP8HuOeJQXAxR3jB+5YdwQTcCeVDmrXIi7CvXQ0JDU19c71bTUcbZRGVRC3O2z5A9+8YNO5S5SIjYamDfJjdwwApgV4w3jRnHHuM2wUmGfnJx07rlr+sHBQUmaRzcPKTWEb9IgjjTz7b\/\/sV+Q2z9xaQokqisLNhqYv8iN3DACmBXjDeOGaAL2pMpZZdJzL7WgTufCt23bljjnHkbg8lJg5tvVNmlBncl\/7969ibT\/+B9PyuDjJ4N0I59ulIZzz0y0KVqCiYkJaWxsLFq1Kl4fcsMQkxu5YQTcrVpbW2ckHh8fdzfOYcpMxF3rHbfQzSx882UzNjYmPT09MjAwEPli4Crs+lzft7RaH5JXZuwR+Ebs6fTkRm4YAcyK8YZx89UE7CmVtcpM3KN62i5D5nHV14V4cXP1Livk7Xx9HWmG5K+tsVPpbGZsNLAfJrmRG0YAs2K8Ydx8NQF7SmWtMhH3pPvcXVbL26vjk8TbZcgeFXd7C9yDn1sqH1nstgiwsm7MPnc2GhhzciM3jABmxXjDuFHcHbklibvLavnwITb2gjrzXWdnp8SdY1\/q\/HofRw4\/cVxuHK7dLXDG5Ww0HIM\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\/n5RAbc\/4e9KpVU725G86jURfZCAjYYbp3AqciM3jABmxXjDuFHcMW6pWJUS7Kieu+n1Rz3cduRNO5+WP3n8xdMjC3d\/NJWyFjETNhqYV8mN3DACmBXjDeNGcce4lW3lssp+bGxMurq6ZHJyMrhettR2O9uR3OPu5h42Gm6c2HPHOJEbuaVDAMuF4u7JLWqfu+\/xs2a+XR8dNSSvfw9vu9PrZvWjN9HF9dzN38\/+ne0y+dob0nDumTLy6UbPGtZO8omJCWlsJB9fj5ObL7HT6cmN3DAC7latra0zEo+Pj7sb5zDlGadOnTqVRbnS2OfuIuzhxXdaN+3F64U1AwMD04v57Drbb2nc4+4WDey5u3FiDxTjRG7klg4BLBf23B25pbEVLmmFvClKOeLOPe6ODuWCOndQoZR8KcLQkRu5YQQwK4q7I7c0xF2H1nX+PG4o3i5K1LB8KVvjSN7j7uhQirs7KIo7zMo2pLhjGMkN40Zx9+BWzrB83FY63Tanx9maveydnZ3TC+f0ZWBwcDAooeshNra48wCb0s5lo+ER\/FZSciM3jABmxXjDuFHcPbmlsaDO85FOyY0jeYCNE64gERsNd1bsgWKsyI3cyieA5UBxx7jlzso40j7Ahve4s+deiUDlSxFGldzIDSOAWVHcMW65szKONHvctYA8wIbiXolApUhhVMmN3DACmBXFHeOWO6uwuPMe92QXsbFNZhSVgtzIDSOAWTHeMG4Ud4xb7qyMI5fd9ZjodrhrLztPdt94Ve7KmacCsdHAvEFu5IYRwKwYbxg3ijvGLXdWxpE8wMbdNWw03FnZKcmN3DACmBXjDeNGcce45c5KHfn3\/\/Qd0Z67fnpXLpLelQtzV848FYiNBuYNciM3jABmxXjDuFHcMW65swqLO\/e4J7uIjUYyo6gU5EZuGAHMivGGcaO4Y9xyZ6WO\/PrfPCmrvvJUUDaKe7KL2GgkM6K4Y4zIjdzSI4DlRHHHuOXOSh355V375MbhQ0HZdn\/+Krn28vNyV848FYjijnmD3MgNI4BZMd4wbhR3jFvurNSRa\/7H38nmPYcp7o7eYaPhCCqUjNzIDSOAWTHeMG4Ud4xb7qzUkTf0\/7Vs3z8ZlI0H2CS7iI1GMiMOL2OMyI3c0iOA5URxx7jlzkodeeWtfy6PPndSeICNm3so7m6cwqnIjdwwApgV4w3jRnHHuOXOiuLu7xI2Gv7M1ILcyA0jgFkx3jBuFHeMW+6s1JHn\/t6f8HQ6D8+w0fCAZSUlN3LDCGBWjDeMG8Ud45Y7q4VXfkBe+5XNQbl49Kybe9houHHisDzGidzILR0CWC4Ud4xb7qxscefpdG7uobi7caJIYZzIjdzSIYDlQnHHuEFWJ06ckO7ubhkdHQ3s29rapL+\/X+rq6iLz279\/v6xevTr4bunSpTI0NCT19fWRaRd84OPyw2t7gu8o7m7uobi7caJIYZzIjdzSIYDlQnHHuHlbTU1NSV9fn7S0tEh7e7uYfzc0NEhvb++s\/MbGxqSrq0u2bNkizc3NsmvXLtm3b1\/sy4At7jydzs09FHc3ThQpjBO5kVs6BLBcKO4Yt1SsSgm2fnfkyJFI4Y96uC3uB7\/4wWA7HD+lCVDcsQghN3LDCGBWjDeMG8Ud45aKVZy4h3v5Lg9r+MTvy0+uWBUk5dGzLsS4pcuN0uxUbGwxcuRGbhgBzIrijnEr28rMv3d0dATD9PbHiPvKlSvlgQceCObok+bcL\/rUXfKvCz4UZMOeu5t72Ni6cQqnIjdywwhgVow3jBvFHeNWlpURb80kakGd+f7YsWPTi+g2b94sk5OTsXPuF\/7OV+WN+b8YlOvATbzH3cVBExMT0tjY6JKUaSwC5IaFA7mRG0bA3aq1tXVG4vHxcXfjHKY849SpU6dyWK7IIiUJuxpFDcvrAruenh4ZGBiQpqamWXkbcefRs+6RwB6BOys7JbmRG0YAs2K8YdzYc8e4QVZJK+TtTLWnvnDhwukhexX3jRs3ytatWyO3w83\/3J\/Jz86ez3PlPTzDRsMDlpWU3MgNI4BZMd4wbhR3jBtklTS0bmeqe9w1vdnbrv9fP1Hb5vTv9bc8EnzP0+ncXcNGw50Ve+4YK3Ijt\/IJYDlQ3DFu3lbhA2xMBmahnB5ko\/vgOzs7g33t+rEPsUk68MaIe+c1F4nuc+cnmQDFPZlRVApyIzeMAGbFeMO4UdwxbrmzMuLO0+ncXcNGw50Ve6AYK3Ijt\/IJYDlQ3DFuubOiuPu7hOLuz0wtyI3cMAKYFeMN40Zxx7jlzsqIO4+edXcNGw13VuyBYqzIjdzKJ4DlQHHHuOXOyog7T6dzdw3F3Z0VRQpjRW7kVj4BLAeKO8Ytd1YUd3+XUNz9mXFYHmNGbuSGE8AsKe4Yt9xZGXHn0bPurqG4u7NiDxRjRW7kVj4BLAeKO8Ytd1ZG3E\/c\/dHclS2vBaK4Y54hN3LDCGBWjDeMG8Ud45Y7KxV3Hj3r5xY2Gn68TGpyIzeMAGbFeMO4Udwxbrmzorj7u4SNhj8ztSA3csMIYFaMN4wbxR3jljsrFXcePevnFjYafrzYc8d4kRu5lUcAs6a4Y9xyZ0Vx93cJxd2fGXvuGDNyIzecAGZJcce45c5KxZ3nyvu5heLux4s9UIwXuZFbeQQwa4o7xi13ViruPFfezy0Udz9eFCmMF7mRW3kEMGuKO8Ytd1YUd3+XUNz9mXF4GWNGbuSGE8AsKe4Yt9xZXfg7X5V7bvr3wdA8P24EKO5unMKpyI3cMAKYFeMN40Zxx7jlzqoIjswaKhsNjDi5kRtGALNivGHciqAJZ5w6deoUVv3iWBXBkVl7g40GRpzcyA0jgFkx3jBuRdCEqhH3EydOSHd3t4yOjgbeamtrk\/7+fqmrqyvpvbGxMenp6ZGBgQFpamqKTFsER2IhjFux0cDYkRu5YQQwK8Ybxq0ImlAV4j41NSV9fX3S0tIi7e3tYv7d0NAgvb29sd4z6Q4cOCDbtm2juGNxHmnFRgODSW7khhHArBhvGDeKO8YtFatdu3bJvn37Svbe9+\/fL5s3bw6ex557KtinMylC8KdLxC03cnPjFE5FbuSGEcCsihBvVdFzj3JPkrjrMP6GDRuks7MzEHiKOxbkcVZFCP50ibjlRm5unCjuGCdyIzdDoCrF3cy\/d3R0BMP0ceKvf1++fLnTnHs6IcFcSIAESIAEikBgfHy8qqtRdeJu5tGVetyCOl1Et337drnttttkYmIiUdyr2oMsPAmQAAmQAAmECFSVuLsIu9ZPh+FXrFghzc3N4rJanlFBAiRAAiRAAkUiUDXi7rpCPrxlznbWjh07AsHnhwRIgARIgASKTKBqxF1745OTk057222Hsede5PBl3UiABEiABKIIVIW4x\/XGly5dKkNDQ8FBNroPXlfGh3vmFHcGPgmQAAmQQK0RqApxrzWnsL4kQAIkQAIkUA4Bins59GhLAiRAAiRAAjkkUNPirvP4g4ODgVu42G52dOqURldXV7DWIeksf5ulHgtc6rjfHP4OUi2SDzfz4PARy6kWqEoy8+EWnqqr5d+vDzc7ba3\/Tkv9LMzvMWqqt0p+TlKz4m6OptU5+2eeeSbYPqf\/v76+vlp8V9Fy2mKzatWqGWf7hx8cPi1Q\/71z586a5OnDzeaozNavXy+bNm2KPZipog6f48x9uIV3ztTyuhofbuaFSO\/j0LVJtfw7dRH2kZGRqu701ay4mzPnNdCL8JaWdtscbjD1ZWh4eNhpt0ItN7YIN210b731Vjl58qSUOnUxbR\/nKT8fbpp248aNsnXr1pp\/GfflZt+QWcu\/07jYNyMbV199tRw7diy4mKxat0\/XpLjH3TJnbp3LU6M3V2WxRzZ0NCP871LlquVGA+GmL5rXXHONfOMb35i++XCu\/D5Xz\/XhlnSvxFzVYS6e68MtqueedPnWXNRpLp\/5wgsvBI\/XHVh6xTjFfS69ATw7qqeuDezChQtrckg0CmG4p+7TW0LPJABcmTsTX27mqORbbrkluOioVl8wfbipuB85ciTwfa2vmfHhprxM26dDzmvWrCl5ZXbuflwZFij8IpTho1N7VE333O3FEhT3mTHl22gYa21477333ppdUOfDTRvaL3\/5y\/LpT39aGhsbS65rSO0Xn9OMfLiZ9QlmEZ3arlu3riZjzoebGXLesmVLMNTsMxqX07CpWLEo7hVDW9mMOSyfzNdnuI\/C\/iZPH26a9pvf\/GbQe6r11fI+3MLD8rXMjtyS2zIkBcUdoZYTG7unzgV1s50SHoZPWlDHlbenGfpws7cP2h6oxeFSH27hWKzl368PN74UuYsPxd2dVe5ScitcaZf4bLGp5WHRMEUfbrZtLfc+7blgXXOQtPUy3PDW8vCyT7xFDcvX6nRGkiBR3JMI5fx7HmJT2kGlDscwi5p0SDmuB1qrB4u4cqO4z4w\/H272ITa1fhiLDzd9EVq9enUAvta5lWr9KO45F28WjwRIgARIgARqkUBNrpavRUezziRAAiRAArVDgOJeO75mTUmABEiABGqEAMW9RhzNapIACZAACdQOAYp77fiaNSUBEiABEqgRAhT3GnE0q0kCJEACJFA7BCjuteNr1pQESIAESKBGCFDca8TRrGZ5BJ577jmZN2+e8xWjuk\/21Vdflcsuu6y8B5ewNucLLF26VIaGhpzLVi239pn6ZbUfO+vnVSwwmDEJiAjFnWFAAgkEfE9Ay+IAjHIEuhzbLINFxVY\/elBSVp9qYZMVDz6neglQ3KvXdyx5RgTyKO6+ZbJRVYuAUdwzCnA+ppAEKO6FdCsr5UvAPs5Ubc1Q9zPPPDN9XKf+3RypGz5y1wwdn3\/++dLd3S2jo6NBEcwlMPY92nb+9fX1sUW1n2EPTZsrT43hpk2bpL29fVY+cemMuOsZ7nfeeWdgFx76to8pDT9HWd16663y4Q9\/OLA3rF555RXp6uqSycnJwOT222+X3bt3y8DAgDQ1NQV\/i6tTFISwuOu\/X3\/99eB\/eh+5zTfKPnxRiqaJ+ls1vvj4xjfT1x4Binvt+Zw1DhGIurTFFpZwLznudi3Ntr+\/P7i+VQVeh5P13mzz4tDR0TEtwqVu0TPlMfnV1dUFonTvvfdO31me1HMPp7cvDdEXEBXhq6++OiivyX\/nzp3B3L2KdE9PzwxRtvMzLzALFiyYtg\/X0fz7pZdeCsps7qvXlwgzzJ504VCUuA8ODop5mYniaruW4s6fei0ToLjXsvdZ94BAkkgkCWm4RxgW96jrckvdAhc1bB5OX6pMSTfMhW8H0\/InDdXb3xtxD7+s7Nu3b1rsNU9bvPXfGzdulK1bt85Y+Fdq6D1K3HVUwLyQmGdouqgFhRR3\/sBrmQDFvZa9z7pPE7CHsMOrz+OENDx03dbWFtlzDw+P29ijhtTjnmffxFdK3JMW9EUJedTfwlMV4akHMzKh9YkSaTtPHQ0wt5GFwy7u\/voocVdbe4pf6dkAAAIfSURBVIFdqZcSijt\/4LVMgOJey95n3WcRsAXNnne3e4dGrMPz4KbnGu65q224x1kKfZxwl5oqsPMrV9w1LzN3bl4+onruPuL+5JNPihn2L7XOwK4HxZ0\/UBLACVDccXa0LDABWyBNz1SHfnV+uq+vT1paWmYsYrMFPCzupebXoxBmMSwfnlO3n6lCXGqI3QzL2+Ie1Uu2h+W1575u3brpNQMuoVOJYfmkF62k6QmXcjMNCeSBAMU9D15gGeaUQNScu917theYRS0MMz15MyyvlbFfAEz+urjODClHzXsbCGktqLN7yvY8\/PLly2ctmAuLu21ryqrl08VxUeLuuqBO8zBz5klrHcpdUGemTcwOB1MPeyFhOPAo7nP6U+TDUyRAcU8RJrOqXgKm4TfbuOwhd3sbmw5TX3\/99TO2u6mo33DDDXLHHXdM90zDgm9682aLnJIyohNHrdS2MddFfuvXr5\/OPmqI3WxRC4ta+NlbtmwJ5tV1EZ2pv91z14eEGYa3woW3A6pN3DY+M1qi\/zUvRFFb4Wz7KGG21zuon5YtWyYHDx4MXjDCL2GmDuFRjeqNapa8lglQ3GvZ+6w7CVSQgIpt1Ap510e6zLm75uWajj13V1JMl3cCFPe8e4jlI4EqIBBeV2B66fa+dt9qUNx9iTE9CbxJgOLOaCABEkiFQPjUvrgtbq4PC1\/k8uCDDwamlTprnhfHuHqG6aqBwP8HCmN3rbs7Ob0AAAAASUVORK5CYII=","height":303,"width":503}}
%---
%[output:82a912c2]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Invalid Simulink object name: 'inv_psm\/inv_psm_mod1\/inverter\/inverter\/three_phase_inverter_mosfet_based_with_thermal_model'.\nCaused by:\n    Error using <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('init_model', 'C:\\Git\\GitHub\\modelization-and-control\\power_electronics_design_with_simscape\\inv_im\\init_model.m', 492)\" style=\"font-weight:bold\">init_model<\/a> (<a href=\"matlab: opentoline('C:\\Git\\GitHub\\modelization-and-control\\power_electronics_design_with_simscape\\inv_im\\init_model.m',492,0)\">line 492<\/a>)\n    The block diagram 'inv_psm' is not loaded."}}
%---
