clear;
[model, options] = init_environment('im_inv_model');
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
dead_time_AFE = 1e-6;
dead_time_INV = 1e-6;
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
tc = min(ts_afe)/400;

% t_misura = simlength - 0.025;
t_misura = 0.648228176318064;
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
application_voltage = 400;
us1 = 690; us2 = 690; fgrid = 50;
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

hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, fPWM_AFE); %[output:4afd7a36]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, fPWM_INV); %[output:9269bdd1]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, fPWM_DAB, fres_dab); %[output:0b84da60]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, fPWM_CLLC, fres_cllc); %[output:0c8e3447]
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
l1 = Kd(2) %[output:22dbc57b]
l2 = Kd(1) %[output:41924656]
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
parasitic_dclink_data; %[output:3dfc571b]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:6726d4a9]

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
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:71f55707]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:5cc7402a]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:502fdf2e]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:5674bd90]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid.omega_grid_nom;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:022e1431]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:84906125]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:1de224ac]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:2b5c76de]
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
im = im_calculus(); %[output:46039459]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:420bad97]
n_sys = psm.number_of_systems;
run('n_sys_generic_1M5W_torque_curve');
torque_overload_factor = 1;

% load
b = psm.load_friction_m;
% external_load_inertia = 6*psm.Jm_m;
external_load_inertia = 1;
%[text] ### Simulation parameters: speed reference, load torque in motor mode

%[text] ### Double Integrator Observer Inverter Side
Aso = [1 ts_inv; 0 1];
Cso = [1 0];
% p2place = exp([-10 -50]*ts_inv);
p2place = exp([-50 -250]*ts_inv);
Kobs = (acker(Aso',Cso',p2place))';
kg = Kobs(1) %[output:861df238]
kw = Kobs(2) %[output:35b72222]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/psm.Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/psm.Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:60b65de7]
luenberger_l2 = Klo(2) %[output:0c143538]
luenberger_l3 = Klo(3) %[output:1860066b]
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
lithium_ion_battery_1 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab); %[output:5c6a42ae]
lithium_ion_battery_2 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab); %[output:62be428f]
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
% if use_mosfet_thermal_model
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
% else
%     if use_thermal_model
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'off');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'off');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
%     else
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
%     end
% end
% 
% if use_torque_curve
%     set_param('afe_inv_psm/fixed_speed_setting', 'Commented', 'off');
%     set_param('afe_inv_psm/motor_load_setting', 'Commented', 'on');
% else
%     set_param('afe_inv_psm/fixed_speed_setting', 'Commented', 'on');
%     set_param('afe_inv_psm/motor_load_setting', 'Commented', 'off');
% end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":29}
%---
%[output:4afd7a36]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe400V_250kW\nNominal Voltage: 400 V | Nominal Current: 360 A\nCurrent Normalization Data: 509.12 A\nVoltage Normalization Data: 326.60 V\n---------------------------\n","truncated":false}}
%---
%[output:9269bdd1]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv400V_250kW\nNominal Voltage: 400 V | Nominal Current: 470 A\nCurrent Normalization Data: 664.68 A\nVoltage Normalization Data: 326.60 V\n---------------------------\n","truncated":false}}
%---
%[output:0b84da60]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_800V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 800 [V] | Normalization Current DC1: 350 [A]\nNormalization Voltage DC2: 800 [V] | Normalization Current DC2: 350 [A]\nInternal Tank Ls: 1.697653e-05 [H] | Internal Tank Cs: 350 [F]\n---------------------------\n","truncated":false}}
%---
%[output:0c8e3447]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_800V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 800 [V] | Normalization Current DC1: 350 [A]\nNormalization Voltage DC2: 800 [V] | Normalization Current DC2: 350 [A]\nInternal Tank Ls: 1.280000e-05 [H] | Internal Tank Cs: 350 [F]\n---------------------------\n","truncated":false}}
%---
%[output:22dbc57b]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:41924656]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:3dfc571b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAZQAAADzCAYAAAChQ+D2AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ9wldWd93+u2ZJQoxBkxfAvUYmlTgewW5cG2uAyLnZt2C46QqjzRl6gLIXBtw1DEmQHmUpCMuJUimYR0iw7lmjt0imZt7NUsWSGsuxaUeYdlhYUAmL0LUtghDaxTcvO78C5PPfJc+9znuc+f3733u8z4xjuPc\/58\/mdc773\/PudG65cuXKF8IAACIAACIBAhgRugKBkSBCvgwAIgAAIKAIQFFQEEAABEACBQAhAUALBiEhAAARAAAQgKKgDIAACIAACgRCAoASCEZGAAAiAAAhAUFAHQAAEQAAEAiEAQQkEIyIBARAAARCAoKAOgAAIgAAIBEIAghIIRkQCAiAAAiAAQUEdAAEQAAEQCIQABCUQjIgEBEAABEAAgoI6AAIgAAIgEAgBCEogGBGJJAInTpygRYsWUW9vbyJb1dXVtGnTJioqKnLNan9\/PzU0NFBNTQ1Nnz7dNfyhQ4do4cKFSeGWLVtG9fX15DUu18QQAAQEE4CgCDYOsuaPAAvKmjVrqLW1lSZNmuS5U\/cqAiwoLS0t1N7eTiUlJYn0SktLlajgAYF8IQBByRdL51E53QTFOqLQIwnGw6Kwbds2qqqqUrT4Ox6hvPLKK9TY2Jj4zC4SdkHhgJyHpqYmevrpp5Ww8WhnypQpauTT1dWl4tKjJv5bf86fffzxx7R27Vo6fPgwHTx4kM6cOUMLFiygiRMnJo2Edu3aRRUVFVRXV0df\/vKX6Tvf+Q6xiD3zzDOqLEeOHKHm5maaP39+HlkfRY2TAAQlTvpIOxQCTlNeumO1is24ceNUR15ZWak6az3KOH\/+vJoy446Zn87OTjVdpjt++1SYk6D09fWpjv7b3\/427dixQwnK+PHjVRxjx44l\/b1VODgNFoHVq1dTR0eHEpSXX345MfLhdPQUHItcT08PLV26lBYvXqw+Z6HjMnA4Hi299tprSpBMp\/pCMQYizSsCEJS8Mnd+FDbVCEULhxYIXk\/RHbMmY1\/3OH36dGJ0osNYRzX8mamgcKevp9N4lMKjiba2NiU4nDceSaQSGr32Yx9daUHhfOvRFAsN\/5vDWsuaH9ZHKeMkAEGJkz7SDoWAXVA4ES0cPJ3lVVB0B50qs6ZTXvw+L95bp6r0CMZNUPToiP\/PI449e\/YkjVAgKKFUJUTqkQAExSMwBJdPIN0I5d57700s2JtOeekpKGt467pEukX5VatWJXaMWafP7FNbemoq1efW6Ta9FsMjHIxQ5NfHfMohBCWfrJ0nZXXbNhzGorzJtmFeQOf1DhYNDn\/p0qUhi\/VOi\/J6DURvDmAh4XjeeecdJY4rV65UU1yY8sqTCi64mBAUwcZB1vKPgNP0Wf5RQImzlQAEJVsth3znDAHrtmQuFK+xmByozBkAKEjOEICg5IwpURAQAAEQiJcABCVe\/kgdBEAABHKGAAQlZ0yJgoAACIBAvAQgKPHyR+ogAAIgkDMEICg5Y0oUBARAAATiJQBBiZc\/UgcBEACBnCEAQckZU6IgIAACIBAvAQhKvPyROgiAAAjkDAEISs6YEgUBARAAgXgJQFDi5Y\/UQQAEQCBnCEBQcsaUKAgIgAAIxEsAghIvf6QOAiAAAjlDAIKSM6ZEQUAABEAgXgIQlHj5I3UQAAEQyBkCEJScMSUKAgIgAALxEoCgxMsfqYMACIBAzhCAoOSMKVEQEAABEIiXAAQlXv5IHQRAAARyhgAEJWdMiYKAAAiAQLwERAqK\/Y5tRtTc3Ezz58+PlxZSBwEQAIGACQye66FLP99JIx9dH3DM0UcnSlAOHTpECxcudBQPLTK7du2i6dOnB0bqjjvuCCwuRAQCIAACJgTGfGqQphQP0N+MuqT+\/9HvC+ix\/zfe5FUV5uTJk8ZhowwoRlD6+vqoq6uLamtr05Z\/586drmG8AGRBkWYciXnywjTqsOBlThyszFlxyKB56dHIpf3\/rDJSdM8sKp5VS4X3zDLOWNB5Mk7YIKAYQTHIayhBJBpHYp5CgR9QpOBlDhKszFkFKSgsIANHu4n\/XzC6jIpnPe57ikuyDUUJyokTJ2jRokXU29urpr34aWxspNLSUuro6KBJkyZ5qw0GoSUa59SpU1ReXm6QewRhAuBlXg\/AypxVpnUriNGIU24l9lk6n2IEpb+\/nxoaGqimpoYqKipo8eLFtGDBArUQz+snBw8epE2bNlFRUZG3GkFEem2GX7Qv7ks0Dhq9NxODlzkvsDJn5VdQWEgu\/HBD0mik+P5aNTIJ4pHYZ4kTFF5D2bBhA61fv55KSkqopaWFqqqq1AK8\/TsvRuF36+rqaO3ateq1pqYm2rx5s0ojyCGtlzy5hUWjdyOU\/D14mfMCK3NWXgSFRaT\/6H66tH8nDRzdr9ZEeG2Ep7aCfiAoBkTDEhQenbA4tbe3q9GNHgXpnWISjYNGb1BhLEHAayivzjc\/ojN9A3Smrz\/x5ft9A9Q\/MEBFhYVJL8y4ayTNuHMEzbxrhDfweRDarW7paa0Lrz6VWBspuqfK0yK7V4wS+6y8GaGwoHR2dqrpMn5YUCorKxNnWm79h1cT9hwzZoxX24YSfnDwj1RQcGMocedipNnGq\/fjwdDNUHpzQSKN24uv\/s2f\/fa3v6VPf\/rTSel3Hbuc+L76MzfRN\/4KwqIBnT17lsaNG5dsrwtnid77D7ry1r8SvXeIaOQ4uuGBJ4j+8uFQ7Tp79uxE\/NJ2pooUFF43OXLkiKNRpkyZokYZeqrK1HJugiJR7d1+FZmWPV\/CgZe5pdOxatnbQ51vfqhGNvVzyql+TjBz\/ua5kxfSystpkX30io7IMy2xzxInKGFZBVNeYZGVEy8ExdwWJqwOvHuRVr58TEW6dcHkvJ4KY163nu4ObMuvuaVSh4SgGFDkNZQwRihYlDeAn+VBTDrJLC9iYNn3wopHLC17T+XlaCWxNvL6DiooKPB1ADEwo9kigqB4JMvbhHt6eqi+vl69yf\/mx68vL+u2YbvrFonG8dLoPaLNyeDgZW5Wr6x4tDL3hbdp5p0jaM+KaeYJZWlI+5bfwal\/RxP+\/v8EtuU3CCwS+yyxU15OW4Qz2TbsZkCJxvHa6N3KmOvfg5e5hf2w4jWVlZ3H6MyFAXpn3RfNE8uSkOm2\/PrhFXaxJfZZYgVFH3C07sTibb98et7vwcZ0BpZoHImVOOxGkkn84GVOzy+rXBQVky2\/fnmZW8R7SIl9llhB4YzZ11P87vAyMZVE40isxCYs4woDXubkM2U19\/m36cB7F2nPN6dl7WI9+9PSBxD59Dq7jU91ADFTXuaWMQ8psc8SLSjmaDMPKdE4Eitx5qTDiwG8zNkGwUov1vc9e795wjGH9OtXKwheQRddYp8lTlDgvv56tZNYiYNuFEHGB17mNINixSfxV3QeE78DLFMvv0HxMreQe0gIijsjFSKuC7aknTqVWIkNTRhLMPAyxx4kK70DrOYLY+j5msnmmQg5pN\/RiFO2guQVVLEhKB5JRnkFsETjSKzEHk0YaXDwMscdNCtJ24rD8PIbNC9zS6UOKbHPEjflFQRoqxBZF\/Lhvj4IunLjkNjopdIKg5XeAcZljvqsSthefsPglWndgKBkStDgfb6cy+qaXm81XrNmDa1btw7u6w0YZmsQiY1eKsuwWFm3FUfhrsVky28QNgiLVyZ5g6BkQs\/nu9op5Lx58+i73\/0u3Nf75JgNr0ls9FK5hc2KtxXzAcgwRMU+GnHb8huEDcLm5SePEBQ\/1DJ8h0coZWVlNHHixLTu6yUaR2IlztAcob4OXuZ4o2CltxXzQj0v2Gf62EcjRdcur+JLrMJ+ouDltQwS+yzRayjWg407duygN954g2pra43vlLf6AjNxX69h7Nu3z6ttQwnveAdDKCnlRqTgZW7HqFjxHStPvf7ftOy+Ef7uV3G4c4TvG1H3jkT4RMXLpEi4D8WEki2M1fUKO4jka4D50Zdk6Tvlec1k0aJFyiVLdXV1wi2LHploR5JwX+\/DCFn2isRfkVIRRsnKzw6wILf8BmGDKHmZ5hcjFFNS19yu6Lvlt2\/frgSloqIi6b75VNFZ76HXYeC+3gP8LA0qsdFLRRk1K16sZ2\/FE0YWptwBptdGBo52Ex9E5LURdoXCLlHifqLmZVJeCIoJpWthnEYo3d3drs4hrVuDdXJ65MK3QC5cuFB9DPf1HoyRJUElNnqp6OJglc6x5IUfblAiwk+UayOm9omDl1veIChuhGzfwznkKSovL\/dILX+DS2z0Uq0RJyurY8m7934raTRSfH+tqDtHtP3i5JWqDkFQpLYuIpJoHImVWLAJCbzMrRMnKx6NHP3xP6nM3j3zQSqeVUtR7NQypzM0ZJy8ICgZWC6sK4DdsgRBcSMk\/3uJjV4qtShZOZ0b4bWRI+P\/jua9ekFtKZbkA8zJZlHyMq0zEvssnfcbrly5csW0IFGFC\/oK4HT5lmgciZU4Ktv7SQe8zKlFwcruU8tpbcTPDjDzUgYXMgpeXnMrsc8SKyi4ApgwheOxhUls9B6LEFnwsFhZt\/vy3yan2KN21+IHcli8\/ORFvwNB8UAPVwBDUDxUFxVUYqP3WoaowgfJSotI\/3\/tp4Gj+xPbfb0usIfpriVTrkHyyjQvEBSfBLHLC7u8vFQdiY3eS\/6jDJspq1QiUnRPVUYL7NpdS\/2ccqqfUxYlkrRpZcorjIJghBIG1YDilGgciZU4INyhRANe5lj9sLLeesgp8XQWr4uMXtFhnrBBSH0LpKTFej+8DIqaURCJfZYukLhF+VS7vaz3m7hZg92ysNv61tZW5f8L96G4Ecvu7yU2eqlE3VhZT63\/4VxPYiqr4C\/KqOizs0I\/vS5tsd6NVxx2hqBkSJ13fbHX4OnTp7vGpNdg3nrrLero6KBRo0ZRXV0d7kNxJZe9ASQ2eqk0rax43eOqaHRf+\/9+lW0egbCA\/LlygRL9WRFJi\/US6xYEJcPW5bTzK1WULD7nzp0jFpS1a9fS+fPniX18tbe3EzuWbGhooJqamoQ4STSOxEqcoQlDfR28UuPlw4Q86uBHiUfvu0TsyffaYxWPwnuqlA8tKY\/1ZP3Mu0bEki2JdUtinyV2ysup1lg9BpeUlKSsWDzVtXPnTlq+fHnilkYWFO2pmF9kQamsrCTtjfjHDxYl4psydWosldae6CeffELDhg0TkZdsyIRoXn3XO++MWFpEwDiekeOuBi0ZRzRyrPrz4xtvplse\/kfjKOIO+OJ\/XKRt\/3nRvxv8DAsA9\/XeAGbNGkpzc3NCBJyKyFNdGzduVPemWKe53ASl7a+L6ZGHH\/FGLeTQly5fouKbikNOJXeil8yLRwB+nrA87Ur8xe3GR6+rxLFYL5EXRihuNcbH9\/b7UJYsWaJGJnw\/in5KS0vpW9\/6Fr300kuY8vLBOFtekdjopbLLVla8rjL16X+nmXeOSOkGPwzmEnlBUDxYOoiT8tY7ULAo7wF+lgaV2OilosxmVnEs1kvkBUExaF16d1ZXV5djaOutjG7RWQXFvm0Y96G40cu+7yU2eqkUc4FVlIv1EnlBUDy0Li87ujxEmzKoRONIrMRBsA4rDvAyJ5srrKI6WS+Rl8Q+S9dAMYvyWkhWrVpFq1evJr5l0fp4Odho3rxwH4oXVlLDSmz0YBU+gSgOQUqsWxCU8OuW7xQkGkdiJfYNOIIXwcsccq6x0nfWM4F31n3RHIRhSIm8JPZZ4kYoVvvy4cTGxkaMUAwrfb4Hk9jopdokF1mFuVgvkRcExUPr0r686uvrjVyteIjaMahE40isxJlyDvN98DKnm8usVnQeI3Yw2ffs\/eZAXEJK5CWxzxI7QsGiPO738NobSGz0XssQVfhcZ6U9FgflBl8iLwiKx9bCU178aPcoHl\/3FFyicSRWYk9QIw4MXubA84GVXqwPYqQikZfEPkv0CGXx4sW+dnlZz7LwKXn2Ngz39eadTbaGlNjopbLMF1bWdZU935xGE0oKfZlEIi8Iii9Ten+JvQqXlZWpkQ07lOzu7qalS5fCfb13lFn1hsRGLxVgPrEKYrFeIi8IiofWleqCLY7COuqwR5lq7cXqqRju6z0YIouCSmz0UvHlIyt9CPL5msnEDia9PBJ5QVC8WJCIeA2lp6eHeKcXP3pNhS\/ZYlf0zz333JAYtaDwF+y+RYuPm7dhicaRWIk9mjDS4OBljjtfWenFeq\/rKhJ5SeyzRK+hbNiwgdavX0\/67hPrKfotW7Y4Cor2PvzMM8+o7cYsQgcPHqR58+bR7t27adOmTarM9vtQ2Dj62bdvn3nLDDGkpDsYQixmYFGDlznKfGb11gcD9I3dH9HnxxbSi\/PMRiqSeM2ePTth6JMnT5obPcKQYlyv6DLrhXUeYVhHKCwOfE88u6Lnz+3u6\/m7devWqVsa9UI8r6mwS\/u2tja4r4+wUkWdlMRfkVEzME0v31l5XVeRyAsjFNPafi2cfR2F\/Xht3bqVWltbk25btEdrXZTXIxSr0HD4pqYm2rx5c2L0I9E4EiuxRxNGGhy8zHGD1VVWph6LJfKS2GeJnfIybxpDQ1qFyOpMkhfmFy5cqF6A+\/pMCMt8V2Kjl0kKh2atdjHxWCyxbkFQPLYuHmls27Yt6S14G\/YIMY+CS2z0UvGDVbJl3DwWS+QFQfHQuqyXYx0+fJh4Z9fp06dVDGGcnJdoHImV2IMJIw8KXubIwWooq3TXC0vkJbHPEjvlZT1Pcvz48cThRPvOL\/MmlD6kRONIrMRB8Q4jHvAypwpWzqxSLdZL5CWxzxIrKLzLi7cG8wl3PkOyaNEi6u3tJUx5mXca+RZSYqOXagOwSm8Z7bGY3bXMvGsESeQFQfHYuvi2xuHDhye2\/\/INjtovl8eoXINLNI7ESuwKMsYA4GUOH6zcWVkX6x+tuELl5eXuL0UYQmKfJXaEEqFdVFISjYNG760WgJc5L7AyY6UX6\/kQ5Gt1wd8EaZYL51AS+ywIyjUCEo2DRu+tuYGXOS+wMmfF6yp\/u+VNKigooEw8FpunaBZSYp8lTlDSOYXkzGINxayy5WModJLmVgcrc1Yc8hdHTlDLgct05sIAbV0wWa2rxP1AUAwsYBcU+wFEgyiS3LGkOtjY3NyctP1YonHQ6E2sfT0MeJnzAitzVhxS89KL9X48FntL0T20xD5L3AjFitHJ9Up7e3vCXYoTcu0DrLKyUgkGH47kJxvvQ5FcYdyre\/QhwMucOViZs+KQVl7aYzG7wGdhieuRbENxziGdjGS900R7IHYKZ\/Xlpf\/mg5H8NwtSttyHIrnCxNWI0qULXuZWAStzVnZB4X+7naz3Fru\/0JJtKFJQrJ6EGXl1dbVyP8+C4PZoty16yozFiO9QMXFf7xY3vgcBEACBPw2\/lX537\/+mPw0fRTf\/7OqdTVE\/cF\/vQjyVY0dTQ+n32bU934eip7yqqqrSCopp\/AgHAiAAAlYC7LF4z4ppgGIhIGaE4nWXl\/0+lCVLltCzzz6bcE2vp8nc7kNBbQABEAABEAiGgBhBybQ49hGK6X0omaaL90EABEAABK4SyBlB4cJYRy2m96GgIoAACIAACARDIKcEJRgkiAUEQAAEQMAPAQiKH2p4BwRAAARAYAgBCAoqBQiAAAiAQCAEICiBYEQkIAACIAACEBTUARAAARAAgUAIQFACwYhIQAAEQAAEICioAyAAAiAAAoEQgKAEghGRgAAIgAAIQFBQB0AABEAABAIhAEEJBCMiAQEQAAEQgKCgDoAACIAACARCAIISCEZEAgIgAAIgAEFBHQABEMgrAgNH9w8p7x\/O9aRlMPib00aMBl3iMYrEINDoFR0GoaIPkveCwtdp4gEBEMgeAv+r9CLd9qk\/qAyP+dRgIuO3Dbv6t\/WzIEr10e8LjKP5\/5+YhzWO1CHg3\/9bfyavh\/YuBOWOO0jadZqS74wOrSZmEDF4mcOTzOrCDzcQ\/8Ln0cLgb3rU3\/anYHRZ4qOCvyijP7f+e3QZjXx0vTkMg5ASeUnMk0YJQYGgGDQr2UEkNzBp5OJkxQLRf3Q\/DRztVqJhn3pisdAicfXviVQ86\/FYEcbJK1XBJeYJgnKNgETjnDp1isrLy2NtSNmUOHiZWysqViwePMroP9pN\/f\/FInJ13UKPMIrumUWF91TFLhhu5KLi5ZYP6\/cS+ywICgTFSx0WHVZio5cKLExWegRyaf\/OJAHJFvFwslmYvPzWEQiKX3IRvCfROBIrcQSm8J0EeJmjC4MVr33oUQiPQFhApO5CMid1NWQYvLzmwR5eYp8lYoTyyiuvUGNjYxKv5uZmmj9\/vi\/mhw4dooULFybe1XFZP7fHL9E4EiuxL4NE9BJ4mYMOghWPRC79fGfOioiVZhC8zK1jFlJinxWroOgO3kk8tMjs2rWLpk+fbkb4Wih+t6enh+rr6xPv9fX1UV1dHa1du1Z91tTURJs3b6aSkhL1b4nGkViJPRki4sDgZQ48E1a8DsLTWZf2\/7NaC+GRSPGsWiq8Z5Z5BrIsZCa8wiqqxD4rNkHhDr6rq4tqa2vT8t65c6drGHsELS0tVFZWljTCYfHiz9vb26moqIgaGhqopqYmIVYSjSOxEofVOIKIF7zMKXplZR2N8CJ7Nq+HmFO6HtIrLz9peH1HYp8Vm6B4hWcanoVq8eLFdOTIEfXKlClTlIgcP36cOjs7adOmTepzFpTKysqE6Eg0jsRKbGqHOMKBlzl1L6x4beTCq0+p0Qhv3y2+vzaxS8s8xewO6YVXVCWV2GfFLih2AbAao7S0lDo6OmjSpEm+bcTTXwcPHqR58+bR7t270wqKTmTfvn2+0wvyxbNnz9K4ceOCjDKn4wIvc\/O6srpwlq689hzRL\/+VaOQ4uuGBJ4j+8mHzBHIspCuvCMs7e\/bsRGrSDmPHLiicAfuaB\/+bn4kTJ6pRxXPPPefbXHqqa\/ny5dTW1oYpL98k5b8o8VekVGqpWPG6CK+P6GmtXF8bMbWPxLqFEYqD9XiEsmHDBlq\/fn1igVx\/tmrVKtqyZYsnQbHHx+sm\/CxduhSL8qatJ0vDSWz0UlFaWen1ERYTfnhaK2jXJVI5mOZLYt2CoDhYr7+\/X61n8PSW3pWlp6nWrFlDL730UtJuLZMKYN0erNdQeDeX9XP77jGJxpFYiU34xxUGvMzJM6vxN91AvD6id2tBSFLzk1i3JPZZIqa8nBbSt27dSq2trUkL5+bNxXtIicaRWIm9k43uDfByZ61PsZ\/7t21E7x1Si+sTXjjl\/mKeh5BYtyT2WeIEZceOHfTGG2+orcKZLMZ7rf8SjSOxEnvlGmV48EpNm0ch7IxRj0YGJ36e7qj\/UZTmyeq0JNYtiX1W7IKip7x4Cy8fRqyqqlJ50lt8+cxIFI9E40isxFHYwm8a4JVMzi4iVlcoYOWtlknkJbHPil1QrIvo27dvV4JSUVExZKHem\/m9h5ZoHImV2DvZ6N7Id152p4zaDTzv1LK7f893Vl5rpUReEvus2AXFaYTS3d1Nvb296swIRihwX2\/a+CU2etO8+w1nHYVwHNoViptL+Hxk5ZcxvyeRFwQlhUVTnW7XfrYyqQjWd+EcMiiSMuOR2OiDJKXFw3oplV+vvrnOKkjuEBTvNHP+xkY4h\/ReKbLtjWzvJLVgMHen62+tNxnyCISvvfXrkDHbWUVdNyXywgjFUgvSuVzhYNbzI0FUHjiHDIKi7DgkNXo+38GPvg+dBUI\/bvek6+tv3aatMrGGJFaZlCOqdyXygqCksH4q1yt+70NxSoYFJZ1zyLa\/LqZHHn4kqvpplM6ly5eo+KZio7DWDsvohRwMNDAwQIWFhUkdt0kxdadvEtYkjL7elsOyOPDDown98Pdxn0SX2EGasI0rjEReEBSH2pDO9YrVHUumFclNUH784PXtyVOmTs00uUDe\/+STT2jYsGFGcX14421G4XIlUO+lwSFF+ePgH+nGghuTPv\/wxjEpi6zjSBeGX+4tSGb71rD09aP05oJEmrcXX\/2bPystLqBv\/NUIESaQ5OxQBBCXTEjiBeeQaYxl3eWlRyTsfyvoXV6Y8sqGZptZHqX8imzZe31660xfvyrU+30D6v9nLgzQmWt\/20s7oaSQJowspPH8\/5Iiqp9zfVSTGZmhb0thFXS5wopPIi+MUFJYO4pdXliUD6upyYlXYqM3oXPg3Yv0\/jWhYQFi8bELjxabGXeNpBl3jqCZd2U20slWViY8wwgjkRcEJQxLe4gTziE9wMrCoBIbfRAYO9\/8SI1qfvHuBTrw3sVElCwyNV+43ddIJldZBcHbKQ6JvCAoFkuFeQWwn0ol0TgSK7EftlG9k0+8tMh0vvmhEhstLjVfGKP+dnvyiZUbC5PvJfKS2GdplrGcQ9Ejhubm5qT73zlTvPOrsbGR7G7mTYzvJ4xE40isxH7YRvVOvvJiQWGBsYtLujWYfGXlty5K5CWxz4pVUHTiWjysxnYSGb+VweQ9icaRWIlNWMYVBrxIjVZYXFr2nlIjlXfWfdHRHGDlrZZK5CWxzxIhKN5MG05oicaRWInDoR9MrOB1naNdWLYumJy0kA9W3uqcRF4S+ywIyjUCEo0jsRJ7a4bRhgavobxZWFZ2HlOL+fVzyhML+GDlrW5K5CWxz8pJQbHu5uIC6ukzOIf01oiyLbTERi+FIW9NnvvC2zTzzhG0Z8U0kd5zpbByyofEugVBiajG2F25cLI4hxIR\/BiTkdjoY8QxJGk9WuEzLutmjaBHZk6WlD3ReZFYtyAoKaqM9WBjEFcA80n7srKypJ1jOCkvur0GkjmJjT6QggUcydzn36aT5y7TPz32uYwPSAacNbHRSaxbEBSH6hL0FcCpTt0fP348rXNINo5+9u3bJ6JiS\/IfJAKISybAy8xKvR8PUuP\/\/YD6fn8jddWOM3spz0NJqlvw5ZWmMoZ9BTBPfx08eJDmzZtHu3fvVrdA8tPQ0EB8j732HyZR7SX+KpJKYE9SAAAO10lEQVTcr4CXuXWY1RM\/vXrqntdU8KQnILFuSeyzNMVYDjZy4pleAXzixAlatGiRciZZXV095NpgPdW1fPlyamtro\/b2dnWtMAtKTU0NTZ8+XTGQaByJlVhyxwNe5tZhVjfecjtNffrfk3Z\/mceQXyEl1i2JfVbsgsIZCNI5pN0dPq+n8LN06VKqq6ujtWvXqn83NTXR5s2bSV8zLNE4Eiux5G4EvMyto1np3V97vjkN6ylp8EmsWxL7LBGCondlcae\/ePFiOnLkSEYuV6zbg603P8I5pHmHk40hJTZ6qRztrEq+\/XPqe\/Z+qdmNPV8S6xYExaFa8JTXxo0bqba2lg4fPkw9PT1qvWPnzp305JNPqumpKB6JxpFYiaOwhd80wMucnJ0VT32xW\/zna7CV2ImixLolsc+KfYRiX5Tn7b4PPPAAbdiwgYK8sdGtqUk0jsRK7MYxzu\/By5y+nRWmvtKzk1i3JPZZsQuKHqF89atfpW3btiXWODBCIZxmNu8fVUiJjd5jESIL7sSKz6fwg11fQ80gsW5BUFI0F722sWzZsqTF80mTJkXWwCQaR2IljswgPhICL3NoTqz4JD1PffG0F9+rguc6AYl1S2KfFfsIRUqllWgciZVYir2yZZ5bKq9Udatlb49yfc9u700u6pJavqDzJbEtSuyzRAgKb+3l6S7rY92d5VY57L67rNuQrWdT4BzSjWR2fy+x0Uslmo4VFugx5ZVpvY3tYKPVaSPv8po4cSKdPn1alUefYk9XOC1GPF1WX1+vgmpfXnPnzk0cYKyoqMA5lExrifD3ISjmBkrHii\/oWtF5jHA2BVNe5jUqOWSsgqJ3dLG\/re7ubrWOYrLLi0cmLED8Dj8sKHp0wn\/zKXg9eqmqqlJCg5PyfquI\/PcgKOY2cmOFBfpklm68zMkHFxJTXg4seZfXli1blIicP38+4UbFy5SXPg2vBUWfiOdFfS++vHT24BwyuEofZUySHPhFWW4\/aZmw+vz3eujFeWPo82ML\/SSRU++Y8IqqwHAO6UKaT8YPHz6cWAB4nWP16tXU0dGh\/m3yBCUoJ0+eNEkusjASfxVFVngfCYGXOTQTVjzt9Yv3Lqa8l948tewPacIr6lJihJIBce1Esquri0pLS5MExy4o7L4FU14ZwM7SVyU2eqkoTVlhgf6qBU15RWlvCEoK2jwt1djYmPSt3ykvjgSL8lFWazlpSWz0cugk58SUFU7QQ1D81OFYF+WtIwo\/mbeOUPh967Zh6+4vOIf0Qzd73jHtJLOnROHl1AsrXqDna4P5bEq+Pl54RcUIIxQH0nZ381EZw56ORONIrMRx2cckXfAyoeT9F7c+QT\/zzhF565ZFYt2S2GfpGhjbCIUzwFNe\/JicOzFvMt5CSjSOxErsjWq0ocHLnLdXVvk+9eWVl7kl\/IeU2GfFJij2S7XsWL2sofg3yfU3JRpHYiUOgnVYcYCXOVm\/rPjelHw88OiXl7lFvIeU2GfFJije8YX7hkTjSKzE4Vohs9jBy5yfX1a8nnLgvYt5Jyp+eZlbxHtIiX1WrILidh+8KWK7Ly\/r4jvH0dzcrKbT4MvLlGh2hpPY6KWSzIRVPopKJrzCqgMQFAtZfa6kpqZGuUjRW329rqM4+fKyCwwna\/UZxv\/GnfJhVfP44pXY6OOjkT7lIFjl0\/RXELyCrgsQFAtR++4uHq3s3buXVq5caczdyZcXv+wkTjw6gS8vY7RZGVBio5cKMghW2tV9PtxFHwSvoOsCBMVFUPze0uh0Up7dufCjF\/fZ8WRnZydt2rRJfd7Q0ECVlZWJnWUSjSOxEgfdKIKMD7zMaQbFSu\/+4rtTti6YTDPvGmGeiSwKGRSvIIsssc\/S5Yt827DTCCUIQbEbDM4hg6zCsuOS5MBPNimiIFn1fjxIL\/7nReo6dpmW3TeCvjr5Jiq9uUA6Ak\/5C5KXp4QdAsM5pAMUr9uGTX152ZPSU13Lly+ntrY2uK\/PtDYLfl\/ir0ipuMJgxaOV1r2n1C4wPgRZc9\/tOXOVcBi8Mq0bGKFkSjDF+\/YpL+tdKvo7do+v3dpzNFiUD8kYMUYrsdHHiCNt0mGy4pP1fI0wX9TFU2E1X7idZtw5Iqunw8Lk5beOQFD8knN5z+7Ly7o92HpAEr68QjKAkGglNnohaIZkIwpWLCwsKp1vfkj8N4sLC8uEkiKqn1MmFY1jvqLg5RUIBMUrsQjDSzSOxEocoUk8JwVe5siiZqXF5RfvXlBTYvywwEwYWUgz7hqp\/i1ZZKLmZWJJiX2Wznfki\/ImwKIMI9E4EvMUpU28pgVe5sTiZqUFhnNsFRktNPx\/PZqRIDZx83KyrMQ8QVCuEZBoHIl5Mu+yog8JXubMJbJikeH\/+JZILTTsNp8\/c3p4hKMEaGQhjdd\/lxQlggY54pHIS2KeICgWQTFvjggJAiAQF4HfT5ihkv7T8Fuv\/X9U4u+rn49K+j6ofP7Z7\/7bOKo\/+91547CZBPzNv\/xDJq+H9m7eT3mFRhYRgwAIiCPAW5ydnvcvOI+GOGyqkVKqwp3p6w+93M\/XTA49DT8JQFD8UMM7IAACIAACQwhAUFApQAAEQAAEAiEAQQkEY\/iRsMeAV199lX7961\/TkiVLqLy8PPxEszSFCxcu0A9+8APlZmTBggU0derULC1JtNnes2cPjR8\/nqZNmxZtwlmU2uDgIP3oRz+iX\/7yl\/T1r38drGy2g6BkSWX+2c9+RqNGjaJJkyapzvLxxx+noqLrO1uypBiRZJP9uH3mM59R\/7344ov02GOP0ciRV8884HEmwE5Ut2\/fTg8\/\/LC6VgKPM4HXX3+dhg0bRvfddx\/99Kc\/pQcffBDt0IIKghJzy7He18JiwQ93iI2NjervXbt2qQbOHeNXvvIV9QuSnWlWV1dTSUlJzLmPNnlTVjpXPFL5\/ve\/T8uWLaObbrop2swKSM2U1+XLl+knP\/kJ3XbbbYpTPgqKKSuuTywoR48eVT9UPvvZzwqwtJwsQFBitIW+uZKz0NHRoUYf\/Jn2N2Z1vc8u+FlERo8enZeC4oUVj9xYTLZs2UJ8kVtFRUWMVo4naVNefKvp\/v376e6776Zz586pzOaboJiy4iswXnjhBfrSl75En\/vc52jbtm1UW1uL0S9GKPE0cmuq\/IuIpxh4yPzUU09Ra2urEhTtdp8rL6+baMeW\/IuIfw2VlZUpQeEbLm+++eb4CxJBDryyuuWWWxRbnhYcO3ZsBDmUlYQXXqtXr6Zjx47R+++\/Tx988IHqHJ944om8GdF5YbV27Vo6cOAAzZgxQ7VV\/hH4ta99Le9mCtLVdoxQYu4L+NfRmjVrkgSlp6eH6uvr1fXFixcvVn\/zVBdPe3GD58r80EMPxZzz6JM3YcUs3377bdU53nrrrcTi8uijj+ZNB2m1igkvrlt6RMJOVPNxhMJlNmXF04IsJDxTUFhYSIsWLaKCgty6AyaTlg1ByYReAO+aVuR8m4ZwQgtW3ioceJnzAitzVhihBMMqlFicKvLBgwfVlcXWKS+9YB9KJrIkUrDyZijwMucFVuasICjBsAolFntFTrUojy3CQ6clwCp9lUTdMm+yYGXOCoISDKtQYrFXZE5EbxsuLS1N7P4KJfEsixSsvBkMvMx5gZU5KwhKMKwQCwiAAAiAQBoCWJRH9QABEAABEAiEAAQlEIyIBARAAARAAIKCOgACIAACIBAIAQhKIBgRCQiAAAiAAAQFdQAEQAAEQCAQAhCUQDAiEhAAARAAAQgK6gAIgAAIgEAgBCAogWBEJCAAAiAAAhAU1AEQAAEQAIFACEBQAsGISEAABEAABCAoqAM5QYA9Mzc0NFBXV1dSefj6X77zI5sf9jO1d+9edTcOl7GyslJdsMaPLjffTOl0xQF\/zzdXLl26FBdBZXMlyJK8Q1CyxFDIZnoCumO1dra5wMwqCOxx2qugMAMtSCtXrswFJCiDYAIQFMHGQdbMCaQTFH2t8pkzZ2jBggV07733qpv2ent7acqUKdTe3q5+vfONhQsXLiT28jxr1iwqLi5Wv+z1rZk8AuC49I2a2is051KPhDgOvmucn+7ubqqurlZ327AYtLS0JL7btWuXCtPZ2am+54fFwj7S4PhOnz6tRiROZbSOUDg9nTbHZ01769atNGfOHHXbJx4QCIsABCUssog3UgJOU15aLF577TV6+eWXlXDwU1dXR3w\/OHeuWiCswsHvcefOwpJKUKqqqhzFgOPne9r5mthRo0YlxIg\/Z0HhPJw\/f56amproySefpO9973u0fv36xGebN29OmpridzgtFrNU03ocNwuUdcrL+h5\/z+XkR0+VRWocJJY3BCAoeWPq3C6oyQiFRwJnz55NjE40ERaQ5cuXU1tbW2K04iQ01hFKWVkZNTY2JkHlUQp3\/lo49BQVjzp4lKFHNtaXdMevRzTW9R4u08aNG6m2tlaJn9sIRQuKXUw4bh7p8Agm29eTcrsWZ3\/pICjZb0OUwLI47bSGoqe8tKDw6MA+EuAOVwsBT3+ZCIqTQFjjMREU3dGzEfVIRBvUj6CkGolAUNBMoiAAQYmCMtIInYDpCIXD8ZoIr6Xw9I8WmzVr1hAvWvMveOuU16pVqxIL4XPnzk1MhXHnr6e2xo0blwgzceJExxGKfcqL02ttbSV+l3dh\/epXvxoicvod+5RXql1ePApKNa2FKa\/QqyASICIICqpBThAwFRQeNfCuJ9NFeRYY65XMerHe+jkDtC7KO0156ekyPU3W3NycWM+wLvTbjWEdWaSb8nrooYfUlN2RI0cSUeg1JC6zdeosJwyOQogkAEERaRZkKm4C6Tr5IPMWxTkSbBsO0mKIKx0BCArqBwg4EIhCUPr6+tT024QJExJbi52Mkcn6h30dBsYGgTAJQFDCpIu4QQAEQCCPCEBQ8sjYKCoIgAAIhEkAghImXcQNAiAAAnlEAIKSR8ZGUUEABEAgTAIQlDDpIm4QAAEQyCMCEJQ8MjaKCgIgAAJhEoCghEkXcYMACIBAHhGAoOSRsVFUEAABEAiTAAQlTLqIGwRAAATyiAAEJY+MjaKCAAiAQJgE\/gchshs1d4ZWEQAAAABJRU5ErkJggg==","height":243,"width":404}}
%---
%[output:6726d4a9]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:71f55707]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:5cc7402a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:502fdf2e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:5674bd90]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:022e1431]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:84906125]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:1de224ac]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:2b5c76de]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:46039459]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: GM355L6-250kW\nIM Normalization Voltage Factor: 326.6 V | IM Normalization Current Factor: 610.9 A\nRotor Resistance: 0.00434 Ohm\nMagnetization Inductance: 0.00533 H\n---------------------------\n","truncated":false}}
%---
%[output:420bad97]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00756 H\nPer-System Quadrature Axis Inductance: 0.00624 H\n---------------------------\n","truncated":false}}
%---
%[output:861df238]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:35b72222]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:60b65de7]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:0c143538]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714299e+02"}}
%---
%[output:1860066b]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -3.927176478925923e+03"}}
%---
%[output:5c6a42ae]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAZQAAADzCAYAAAChQ+D2AAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ1wV9WVP1q7GlsVI1qJAYIa1K4z8WNxEbXUUqUfQ+x02k1CZ2VjSmmrznQBScC6DlshCRK7FtSyGCndKYHZtqywOxVttCpFthU03arVaIgQU1sMUG3Faa3snEdPenPzvt+7793\/O+fNOGP4v3fvPefc+\/vdc879OObIkSNHQB7RgGhANCAaEA0k1MAxQigJNSifiwZEA6IB0YCjASEU6QiiAdGAaEA0kIoGhFBSUaMUIhoQDYgGRANCKNIHPDVw4MABaGpqcn7v7OyE8vJyI9ratGkTLF68GFpbW6Gurs6pA\/8NH\/o7zYrd5Dp8+DAsW7YM5syZA9XV1WlW51lWb28vNDY2ws033xxKzjht3LlzJzz++OPQ3NxsRCbSZU9Pj1P+hg0bYOrUqaHrcrN96I8NvYh6bmlpgYqKCmN6M9T03IsVQsndBNIAXQOmQUYnlLKyMgdAdu3aBevWrcuMUNrb2wEBPwxZE8hFaSOWPXv2bJg3b54xYKQ61MlAlB5t2tZR2qK+i+1atWpVpv0hbltt+k4IJWNrIIisWbPGqRVnQCqA0eCaP38+dHd3A8769Hf0GaEKFjTjvfTSS+Gkk05yZov4BA12qldvkw68Q0NDzoyaZvA486Wyw5aBXo4OQiqoYBvQW6Fn1qxZ0NbWBgj6+BCw7t27dxiI3cCWdDE4OOh8p+pJlWv16tWwYsUK2Lp163CdKFNtba1DMvq\/qx4T2RJttHLlSsC\/J0yYMNxevQ2qHeg3lI+8B922ZPvKykrPtqh6RwFIX9h3kEzoqampcfSFD3qd5FEEkQ3plvRA5aAd9brV39Rh5ddnVdv39\/c7Y0Pv89RfdFmwDaRHvU9ec801w3KiTq677jr44he\/OMoLpr6m1+lmn4yhoiSrE0LJ0GwqmajVUphAH6BBYEC\/E1DpAEa\/64NFn4mpAK6SymmnnTYi5EWEQiBN5e7evXsECfiVkZRQsGzSE+lNJVIkn4GBAYf4qJ06OSFIUijPi1AI3FRdqXp0A9P9+\/cDkrlfG3Rbk+104FZt79XGiRMnjiANtT\/ovyHY33nnnXDLLbcMk4nef\/Sh4NUmL7u7EYpOJlQHEZlXnydi9LIlfa\/3eWzbfffdB\/fff\/+IycBVV10FTz75pOsEyI2osgr3Zgg\/mVQlhJKJmmF4JnX66acPz6xp5kWDZ8uWLQ4w09\/YNJolk7eBs04dhGi2ToCP36Hno85s3WLbNGgQCMlTUmeMNMvD8nB2q5ePs8KoZQQRCnoAQWEQffaov0\/E7QbWqIfJkyePIMowIS8qU\/0eZ\/k6Qei2pN9JT+TBfOtb33Jm4\/S7m+elds0wIS89xOX1t1f\/0XNkev9EPZGudULw8oL193WgfuSRR0b0eZXs3UKBXpMH6vPYJ6ndRHBkX\/SyVO9T9XLdQndoc\/wmyzBoRnBkrBohFGOqHVmwG0jqIEKDSwV\/taNjibo3oXoD+P84M6dZsgoAboSiD04KK1HLvUJeavlRy0iDUFS90eydwAHb7raQQNWjTpR+hKLPoFGP6LnpevYiDL17IchRm\/V8iFf4CtvnRyhe4T2dULy8AS8PViVRSrTrctIkyItQ3Mpw85CDSE73dHQPxq3Pq21ysz+F\/dT2qCHAoLZnBB0lVY0QSkbmcpsBeRGK10DwIhT8dy+g08NDqrhRyYA8lKSEopNr0N9uJqJvbrvtNsd7olyE10w\/KqHoYKL+nYRQ1JCMV4LdLc9G3qb6TViPJCi8RP1HX53l1neyJhS9z1EITA8tpkUoas5OCCU6OAqhRNdZrC8IvNMMeekNcSMIP0JRZ33kwaggNXfuXNccijp4w5ZBYTU1DKcn9L3+dlO4PitXPbCkIS89d6SGTOKGvKKGr\/B9lWhpkYBKKDrg6eGloJBXUEdOM+Slh3FJDsq\/eXko5LXT73qbdIJBW8UJebnpQgglqIeM\/l0IJbrOYn9hKikfxv332h\/gFgahEIhXUl4lFBX4VMX4rVCi94IIBd\/zWjmEv5E+9Xe8FieQnvQ4vUoYWO7111\/vJK7dQiJeCyiwDWGS8uQt6GDllbz20iOWgw+tGHQL26iro7Ccu+++G+64445Rcukr6aisoKQ85iuC8l1hk\/JBhKIPOr8+79buMEl53VOTHEp0qBNCia6zRF+kvWxYBdOoHgoJ4pYnwPBHmBxKUBn4uwrw2F70fG666aZRK24IVFQQ8iMUv30WYZcNU+JXBV8E6+nTpw+voELwuuGGG+DGG28cDq3phIbLhhcuXOi7bFgFbrcQqBv4uuXTsG5sI3mQtLz8nnvugQceeAAon6QSpT5JILL00y\/W47dsWPeivDaheuU\/1ByfF6HoZI\/6wOXqlCzHNuj5LPw3tU7VnvrmWTUnqf6mh\/b0\/GIiECjwx9YQCgLAokWLnD0BbjuV9bXofkthS9VeQbO9UpWraO1WQVZfsq17b16yE2AhcZvaxV40vYeVhyYT+L7b6sUwpy\/IPpSw2h75nhWEEmZJJIIt7g0o8uATQonXifP4yit8GbSJVG1rlJ3yechYqnUGhQ\/DHK2DY1F2ykfvAVYQCnofOLjw8fJQ8PeqqqpQZx5FV4MdXwih2GGHMK1wi9MH7TrXy6VZMIbLopx\/FaZ93N9xy6OFPWdMzvKK33tyJxScTSxduhQaGhocUnEjFDLwtGnTCk0o8c0oX4oGRAOigfw1kDuh4Kwcn0suucQzh+LmwkYJLeSvZmmBaEA0IBoovgZyJRR0S9evXw+33nqrc\/6SV1JeDw1IqKD4HVMkFA2IBkpPA7kSCoa4cHkmxo+DVnnpqqWci1uS\/uyzzy49S0iLRQOiAdFACA3gSeSTJk0K8Wb2r+RGKF4rMVAFYZJnfkl6JJS+vr7stZlzjVzlRrVzlZ2r3GJzO\/EtN0LRsdfPQ6FVYOomNNxE5nUKKNdBxlVuARc7wcX0\/Iprf7dZbmsJBUmkq6tr+LIifWOjnxdjs8JNDjKucguhCKGYHFe2lW3zOLeGUNI0ms0KT1NOvaw9e\/ZYG1s1KTeWzVV2rnJztrnN+CaEYhrpMixfwMXORKXJLiA252dzIRSTI8qlbJsVblIVAi78wEVszs\/mNuObeCgmET7jsgVc+IGL2JyfzYVQMgZWmxVuUhUCLvzARWzOz+Y245t4KCYRPuOyBVz4gYvYnJ\/NhVAyBlabFW5SFQIu\/MBFbM7P5jbjm3goJhE+47IFXPiBi9icn82FUDIGVpsVblIVAi78wEVszs\/mNuObeCgmET7jsgVc+IGL2JyfzYVQMgZWmxVuUhUCLvzARWzOz+Y245t4KCYRPuOyBVz4gYvYnJ\/NhVAyBlabFW5SFQIu\/MBFbM7P5jbjm3goJhE+47IFXPiBi9icn82FUDIGVpsVblIVAi78wEVszs\/mNuObeCgmET7jsgVc+IGL2JyfzYVQMgZWmxVuUhUCLvzARWzOz+Y245t4KCYRPuOyBVz4gYvYnJ\/NhVAyBlabFW5SFQIu\/MBFbM7P5jbjm3goJhE+47IFXPiBi9icn82FUDIGVpsVblIVAi78wEVszs\/mNuNbLA\/lwIED0NTUBD09PaHxsaamBjZv3hz6\/SQv2qzwJHIFfSvgwg9cxOb8bG4zvsUmlAULFsCSJUuguro6COegt7cXli9fDuvWrQt8N40XbFZ4GvJ5lSHgwg9cxOb8bG4zvsUiFJOgmEbZNis8DfmEUEZrgCuwcpUbewBX2W3Gt1iEQiEvNGpnZyeUl5ebxMnIZdus8MjCRPiA6wDjDC5ic\/FQIkCE8VdjEQq26vDhw9DS0gJbt251GrlhwwaYOnWq8QaHqUAIJYyWivUOV2DlKjfnSYTN+BabUFQ42rlzJ8yePdv5p1mzZkFbWxuUlZXlhlg2K9ykUgRc+M1Wxea8bL795UPw2X\/dCL\/97pdNQknsslMhFKpd9VoqKiqcJHyYpH3s1nt8KISStkbtL48rsHKVm6uH0vXz1+HGrhfgwF1XWzkoUyUUVUJa2dXR0ZF5jkUIxcq+ZrRRXIGVq9xCKAwIRTwUo5gZWLiAC6\/wB1dQpYHAsb+z8FAkhxKI9Zm8wHGAcQYXIZQ9MGkSr0lE+7Z+aN+2p3ghL32VV2trK9TV1WUCnEGVSMgrSEPF+50rmXKVmyuZFpJQaB\/K\/v37c0u8+0GiEErxCCNIIq7AylVuIRQGOZSgQZ\/V70IoWWnannq4AitXuYVQCkQo6KHIWV72gCn3PAJXcOEsN1fZCx3yktOG7SIVma3yStByBVXOEyjcg4IrvdjtQzEJte3t7U7xzc3NrtVIyMuk9u0smyuZcpWbK5kKoaSMP7REed68eUIomm4FXMRDSXm4WV0cx\/4uhJJil8SlysuWLYPnnnvOOYhSPJSRyuU4wDiHP7jO0jnbvPaeZ2D7K4ck5JUGr2zatMkppr+\/X0JeLgoVQhEPJY1xViplcOzvQigp9U5cWbZ06VK4\/fbbYe3atUIoQigjNMARXMRD4bdT\/qI7noK9B94ptoeCnsPixYudAY73ouDT1dWV6jH2mIifPn26E+oKk5QntOnu7k6J0uwvZmBgACorK+1vqIEWcpWdq9zYhTjJPmPGDHjvxLHw5rVHFyQVdpUXgvvg4KDjOaAH0dDQEAr0o2AKnly8fv16uPXWW517VsIQSl9fX5QqCvEu11k655m62JxPmBM9E\/RQCkso6gZHnBnjDY5EKGkeX696QCry42Ved9999ygykGXDheDHSEJwBVaucnOcRNCmRpaEgst70ZMwcee8eCjuWCvgwme2Sj1AbM7H5pSQP\/btN+CNb38+0oQrq5cTX7CF3sOOHTtGhLwmT54MTU1NUF9fb+QEYiEUIRRdA1yBlavc3DwUNdx13BsvFvsKYPU+FBroeR5nLyGvrOYj9tTDFVi5ys2NUOhiLZT7g9tXwN6f\/ciewae0JLGHYqNUQig2WsVsm7gCK1e5uRFK+fzHnAE0ofwEePOBL4Cti46EUMziXKalC7jwiadLDgWAQ3\/HUNdNXS84u+Px2fLVi+H6ay8pJqHQRVtBpw77nbtlAnHFQzGhVbvL5AAubhbgKjcXD2XNEwOw+L96h72TZ79+OdiMb4k9FEzKb9y4ccRqLiIaSsoHJdHThiqbFZ62rGp5Ai7ioZjsX7aVXeT+jp7Jwy8MwaIfvDRMJuidYMjLZnxLRChEHHhII+5gVx912fDQ0BAsX77cuS44i8dmhZuUv8gDLEhvXGXnKnfRPRQ6YoXyJkQm+LfN+CaEEoRUJfS7gIt4KCXUXRM3tWj9Hb2S944cgUuW7RzWDXokKpkUmlBQuDAhL9qr4rarPXGvcinAZgY3IS+VWbQBFkVXXGXnKnfRPJTtLx+Cmza+4Bz8SA+SCeZM9MdmfEvkoZCgbvtQ8JBIDIPhbwsXLnTCXdXV1VEwIva7Nis8tlAhPhRwEQ8lRDcpzCtF6O9P9f0OPr169wibIJGsrr8Arjx3jKutbMa3VAjFth5qs8JN6qoIAyyufrjKzlXuUvZQ9KXAqkeyqv4CuMqDSOg9m\/FNCCUugln4nYCLeCgWdktjTSqV\/o4Egv+t2LZneD+JqpQgj4RVyMst3EUKqKmpMXI4ZFAPtZnBg9qe5PdSGWBJZPT6lqvsXOW23UNBAnn61TfhOzte8ySRK84ZA80zJzlLgaM8NuNbIg8F73jHI+unTZsGtbW1w8fXmz4cMkj5Nis8qO1JfhdwEQ8lSf8ptW9t6u9IIC\/+5g+w6tG9rgSCukXimHDqCbBo5iTP\/EgYG9iMb4kIRb0PBRPuuIGxqqrKOWEYPZe0b20Mo2x8x2aFh5Uhzns2DbA47U\/yDVfZucqdt4eCBPJv3a\/Coy8eGLEyS+\/DSCKfvHAsfOUj4yN7Il7jwWZ8S5VQcHlwf38\/4EbHNC\/Yigo0Nis8qixR3hdwEQ8lSn8p9Xez6u9IHqse2wsvvv4HT++DdIkEgqGshinjjnokEcNZYWxiM74lIhQUXj1WRfVKtmzZ4tyT0tbW5lzbm+Vjs8JN6iGrAWZShrhlc5Wdq9wmPBQkjjffeReWbO4NJA6VQD5\/6YdgenV5ojBWlH5vM74lJhT9+BUkmDVr1kBFRUWme09Ug9is8CgdJ+q7Ai7ioUTtM6X8ftz+jpsI3\/7jn2H1Y3th78GjK7DCPOhtfHRyOcz\/+EQjnkeYNtge0k9MKGGVkOV7QihZatuOuuKCix2tj98KrnIHeShEEqt\/shd+9evgUJVqAQpTzb5sHNT\/3Zm5kodbz7AZ3xIRip6UV4WXHEp8kIj7pYCLeChx+04pfvfTnl4YP348PNjzW3jk+aFI3gbKS8RBXof6bzbrgyWhqKcNl5eXZ2ofmxVuUhFCKEIoJvtXHmWjp4HhqXsf3wf9bxwOndvQPQ5crjtl0ilw9eRyY8nyrPRjM77F8lD8NjOqSs36Yi2q22aFm+x0QihCKCb7V9plU1jqyBGAbz+xD54b\/H1kL4PaRN4GrrCqn3ImTCw\/uhDIxCqrtPUQtTyb8S0WoZAC\/EJeUZWU5vs2KzxNOfWyhFCEUEz2r6hlE2G8euAwbPz567APjyCJkATX66ONgeeecSJ8bcZE2LdvH1xRk82Bs1FlN\/m+zfiWiFBMKi1J2TYrPIlcQd8KoQihBPWRNH9HwkCy+NEv34Bfvhbfu3DzMr4yfTwcevtd36W4XPu7zfgWi1DC3iUvZ3mlOXyDy+I6wFAzXGU3ITcdZnjWmOOhc8dr8It9byXyLFTCwFzG+eM+ADdMOwtO\/Jv3OSEprC9OaMqE7MGjLP83Ckco+avUvwU2K9yk7rgOMCGUcJ6Zut+if+gw\/PCZ30Lf\/rdTIwsnZ3HqCXDt346FiypPGiaJOGQRZpxw7e8241ssDyWMsfN8x2aFm9QL1wHGnVDed8q44W6FK6L+439\/Df83kI5X4ZDEX44PQbKYfl45\/H3VKVaslOLa323Gt1QIBc\/wWrx48QisbG1tdQ6JzOOxWeEm9cF1gBWVUFSP4q133nWWziZNbOv9T10dde2HT4OLx5\/svJIkFGWyj6tlc+3vNuNbYkIJc6d8Vh2M6rFZ4SZ1wXWAlRqhqETxq9f\/4GzMM0UU6FVcNukUmD1lHBz3vmNGdD9ToSiTfVwIxe7T1BMRin6Ol2ps2diY1bD6az1CKOFyCaYsg2dEIUj\/6c\/vwfd+9jo83f87p6okS2W9PAokignlZTDr3GPggnMmDie2S50kotiGa3+3ecIshBKlB1v+LtcBZtJDoRVP4089Hrb8Yj888sIQwJF0SULPU5x35gcAl80ed+xfPQovohCb5zuJyAMSCksoqEwJeeXRpdzrFHAJBy5EEpV\/IYkfPz+Uuiehk8T48hPg8nPGwEfOPXXEMtmkHoXYPJzN7RmlyVtSaEIhUpGkfPKOkrQEzuBCBwX2DLwFO1455BzjkXa4SScJ\/PvSiSdD47Szjtb1l\/0USUkiSj\/gbHOusheWUCiHUl9fn9uKLrfBZ7PCo4BF1HeLMsD0+ykefn4Intn3ZuqJa9KvuiwW\/+1j558Gn7nodDj2GPuT2EWxedS+bjLMGactWX5jM74lyqGgEvWDIjds2ABTp07NUr+j6rJZ4SYVYzO4qCRx8O0\/wfd3\/wZ69r1lxItwCzedf+YH4LqaM4bVXwrLYsP0FZttHqb9Sd7hKrvN+JaYUNQOoe5HkRsbkwyVeN9mPcAoF1Ex5nhn1\/UTLx0wRhA6SUw4rQyuqzkdyt5\/9PgOrgcFZm3zeD3TzFdcZWdDKGq3wauA0Xvp7OwEuQ\/FzIDSS006wMiLeO\/IEbh\/+2uAnkTa+yPUNuuhJsxH\/OPUCmd1E7blynPHhFZcUtlDV2TZi1zllpBXn2U98WhzUvVQ6D55LDjMwZCHDx+GlpYW2Lp1q9MYv\/tT9NCanwdkM4Ob7AUquOh5iKdffRO6fzWUCUE43sSpJ8DFE06GT104FsadcvwIsU0krbkCK1e5hVAKSihJwlxIQPg0NzdDUIIf6+nv73feDXqKTCgqUezsOwSP9x7MhCSQIM4540T4aPWpDlHQY0sugiuwcpVbCKWAhOK3Uz4I9N1+VwlG\/x1\/q6qqCrWarJQJhQjjmz9+FV5J6SRYrzAT7o342HnlMKXqlFyWvMbpI17fcAVWrnILoRSQUNIEBD9yotDYtGnTCkMoSBx3\/fhV5\/jw7a8cSqRKCiGdXgZwUdVpMPfKSjjh\/ceO8CISVVACH3MFVq5yC6EIoXjCEuVeZs2aBW1tbVBWdvQ+aHrcLvTyO83YNg8lLnmoSeuPTD4V\/uHSM31zEQIu\/HZNi8352dw2fFNBKdWkfNKJLBLL4ODgKFLp7e2FxsZGWLlypbPHRf9brxcVTk93d3fSZsX6vv\/gn2D5Y0Ow67V3fL+vOPk4GHfScXBVVRl8+EPHO\/+P\/xbnGRgYgMrKyjiflvw3XGXnKjd2WE6yz5gxY8QY7esTDyUQtJAoFi1aBCtWrIDq6mrf9\/3yLXkxOJ42u2LbHs8QFnocV5wzBppnTop15WmQAmW2ym+2KjbnZ\/O88C0If\/D3RB4KhqIWLFgAS5YsGUUASA7Lly+Hjo6O0PtQohx575ekz1LhGM56dt+b8E\/rnxulbySQz1x0hnN\/tomlsnqFAi78wEVszs\/mWeJbGBJJLeTlRyhhyEH1MijxjvtL9KXBeln498KFC2HdunWunkwWCqfVWBfd8dSovMbMD4+F9s\/6e1hRDRXmfQEXfuAiNudn8yzwLQzeuL0Ty0PRNxl6Ve63URG\/0Tc2qkl5rKOrq2s4nxLlzDDTCkcyqb33GWepLT3ogayuvyDS7u64RvP6TsCFH7iIzfnZ3DS+JcGlWIRCFfp5KEkalfRbkwrHPAmSiUokW756cSYhrSC9CLjwAxexOT+bm8S3IIwJ+j0RoQQVntfvJhSO3shDz70BLZt7h8X6wmXj4JZrq6wgE2yUgAs\/cBGb87O5CXxLC6tjEQrtC5k7dy6sXbsWenp6XNsT5jyvtARRy0lb4UgmXT9\/Hdq37XGqsSG85aY3ARd+4CI252fztPEtTQyORShpNsBEWWkqHMnkzof74Xs\/+\/UwmdgS4tJ1J+DCD1zE5vxsnia+pY2\/QigBGv1m96vwjf85uokIPRNbyURCXntg0iR+4CKEws\/mhSUUtyNRVHwu9ZAXhrlu7HqhJMhECEUIJe3Zpu3lcSXTwhKKV4fD5cDLli2DOXPmBO54N9Fp01A4hrpoj4mtORMJef1VA1zBhavcnCdQaeCbCdzFMo2FvPR9JKYEcCs3qcL1fSYY5opye2CWsqp1CbjwC3+IzfnZPCm+mcQnY4QS5+iVtARNqvCtv9gPc77zS6c5jdPOgo7PTU6raUbLEXDhBy5ic342T4pvJkHIGKF4nRxsUhgqO4nC9VDXs1+\/PIsmp1KHgAs\/cBGb87N5EnxLBWh8CklEKH5Jeb87300LlUThtfc8M3xaMJJJFoc6pqUPARd+4CI252fzJPiWFtZ4lZOIUEw3Lm75cRWuHqty5TljYMuNF8dtQi7fCbjwAxexOT+bx8W3LEApMaHo1\/P6nRqchUBYRxyFY6jrpq4XHO\/E9v0mXnoUcOEHLmJzfjaPg29ZYW9iQnG76CpvUomjcNU7mXN5BXzz8+dlZYPU6hFw4QcuYnN+No+Db6mBTEBBiQgl7Qu20hI6jsIpd1Kq3gnqTsCFH7iIzfnZPA6+pYWtQeUkJpSmpibnQiy86119wlywFdS4uL9HVXip505ITwIu\/MBFbM7P5lHxLS6OxvkuEaFghZs2bYKNGzdCZ2fn8FW\/tPqrvr4e6urq4rQr0TdRFV5777Ow\/eWDTp2ltrJLVZSACz9wEZvzs3lUfEsEphE\/TkwoWJ\/bDY6tra25kEnUpHwp7zvRbS3gwg9cxOb8bF54QolIYsZfj6Jw9QDIUjliRVZ5jdYAV2DlKjfnnGEUfDMOtloFiTwUWs3V0NAwKoeStSBqfVEUribjS2lXvJt+BVz4zVbF5vxsHgXfssbhRIRS6nfKq+Gu+R+fCF\/\/1NlZ6z\/V+gRc+IGL2JyfzQtLKJSU7+\/vd1Z62fKEVXj7tv7ha31LPdzFOQTAWXYhFCEUW3AX25HYQ8Flw6V6p3yRwl2cQZWz7EIoQiiFIRSbBImaQ1HDXV+bMRH+5dOlHe7iDKqcZRdCEUKxCYcTeSg2CRKVUIoW7uIMqpxlF0IRQrEJh2MRCm1cnDt3Lqxdu7YkQ15FC3dxBlXOsguhCKGUPKHYJIBbW4KS8mq4qxSPqffSv4ALP3ARm\/OzeRC+5YnPsTwUtcGleHx9kTYzqrYQcOEHLmJzfjYvNKGU4vH1pXwro9\/sQ8CFH7iIzfnZvLCEUqrH15fPf8zBZTyqvtR3x4uHclQDXIGVq9ycbV5oQim14+vV\/EnzzEnQPLMqz5BjqnULuPCbrYrN+dm8sISCaFhqx9cXNX\/CecbGWXYhFCGUVGelCQtLnJTH+kvp+Ho1f3LgrqsTqs+uzwVc+IGL2JyfzQvtodgFqUdb46fwouZPOM\/SOcsuhCKEYhMGp+Kh2CSQH6EUOX\/CGVQ5yy6EIoRiE\/6yIpQi5084gypn2YVQhFCEUGJoQM3TVFRUwLp166C6utq1JK+Q141dLwCSCj5Fy59wBlXOsguhCKHEgFNjn5SEh9Lb2wuLFi2CFStWOCTitrJM1ZAXoRTx\/C5VbgEXfuAiNudn80In5RHsGxsbYXBwcBTr1dTUQGdnJ5SXl6fKiDrB6IV7KZwS8kU6v0sI5agGuAIrV7k527ywhELneGEIKssbG9FD2bFjB7S1tUFZWdkosnJT+PaXD0Htvc847xbhdkY3hhZw4TdbFZvzs3lhCSXrO+VVb2jDhg0wderU0DmUoifkOc\/YOMsuhCKEkmr4J2FhiXIo5KE0NDR4gnvC9rl+TsSycuVK13qQdJ\/vAAAOh0lEQVSRwenp7u52\/vdLP3wddr32jvP\/u24uznErqoIGBgagsrLShMqtL5Or7Fzlxg7JSfYZM2aMGIN9fX1WjslEhIISBYWfTEntdsox1eXmEhY9Ic95ls5ZdvFQxEMxhbFxyk1EKHRzY09Pj2vdppLy+h0seuU6oRT1Qi1dbgEXfuAiNudn88LmUOIwWJxv9FVduCdl4cKFnntRdIWrCfminTCs6lPAhR+4iM352VwIJQ6LaN\/oB1BGScpzWOHFOezDWXYhFCGUFOA1tSIShbyoFZhHWbx4sfMnAj0+XV1dnst6U2u9R0E6g7dv64f2bXuct\/FCLbxYq4iPgAs\/cBGb87N5oT0UTI7jpsbbb78dli5dCrTiyy9pbhrMdYVzSMhznqVzll0IRQjFNJ5GKT+Rh6LuQ8Hlqi0tLcOEgnmP5cuXQ0dHR+o75YME1Aml6DvkSR8CLvzARWzOz+aF9VD8CAVzHuilmDh6JQqhqCu8GqedBR2fmxz0ecn+LuDCD1zE5vxsXlhCQeSlfShqyGvy5MmAd83X19dDXV1d5gCtKrzod6DIKq+jGuAKrFzl5mzzQhMKGtbmK4DVhHxRz\/CSkJcQSuazNgsq5EqmhScUC\/rWiCaoCv\/3JwegZXNv4Vd4cZ6xcZadK6hytrkQSsaMoyqcywovzgOMs+xCKJJDyRhefatLtMqLSlb3odC\/+W08NK0AlVAuuuMpwDxKUe9AkRyK5FAmTeIHqpwnEYX2UNxuT6QzvvJOyqsJ+Y+dXw7f\/1KNaS7LtXyZrfIDVrE5P5sXllCIOPByLf1uEluWDdMelCKf4SVJeUnK5zqTyalyrmTKklBs2NionuFV5CNXhFCEUHLC9FyrFULJVf2ulSfOobid\/GtLyIvTkmHOMWXOsnMFVc42L7yH4nUfikpheDfK5s2bM6FUUvjS\/34F7n50r1PngbuuzqTuPCsRcOEXTxeb87N5YQklT\/D0q5sUzmnJMOcZG2fZhVCEUGzC4cQhL5uEobYQoXBaMswZVDnLLoQihGITBqdCKG77UFpbW3M5xwuVi4Tyk6efByQUfDjsQeEMqpxlF0IRQikUodi8D4XTkmHOoMpZdiEUIZTCEIrN+1C++\/BuqL33GUfX9zRcAA1TzrRJ70baIuDCD1zE5vxsXtikvM2EMu++R4ev\/S36KcPETgIu\/MBFbM7P5oUlFAQyW0NeKqFw2NTIOezDWXYhFCEUI+GOmIUWNil\/4YIfwPZXDjlq4bAHhTOocpZdCEUIJSb2G\/ksFUIx0rIEhaJLSIQyofwEQA+FwyPgwg9cxOb8bF7okJeNQI0KP\/SZTlZLhjnP0jnLLoQihGITBhfSQ6m68DJ489p2R8+4ugtXeXF4BFz4gYvYnJ\/NxUPJGM1VQuGywovzLJ2z7EIoQigZw6tvdYX0UCZc9kn4\/ZWLHMG57EHhDKqcZRdCEUIRQjGsAZVQxEMxrGxLiucKrFzl5jyJkJBXxqBT8al\/hnfOr3VqFULJWPk5VccVWLnKLYTSl9NI86+2kCGvMz93B\/xxwhWO5Fz2oHAeYJxlF0KRkJdNzFJIQjnj+m\/Du2PPA057UDiDKmfZhVCEUIRQDGtg7Jf\/E947cawQimE921Q8V2DlKjfnSYTkUDJGHjq2nss9KKReARd+s1WxOT+bC6HkRCg3fnQ8fKP23Ixrz686ARd+4CI252dzIZSMMZbbxVrioQBwBVauckvIS1Z5ZUYrRCicNjVyHmCcZRdCEQ8lM2ANUVGuq7zogq6enh6nqbNmzYK2tjYoKysb1fSdO3fC7Nmzh\/+9oqIC1q1bB9XV1aPeJULhtAeFM6hyll0IRQglBM5n9kpuhHL48GFoaWmBadOmQV1dHdDfSBTNzc2jFIAXefX397v+pr8shJJZ\/7GmIq7AylVuzpMIyaGEhB0kjR07drh6Ke3t7VBVVeWQT9BDhMLlpkbJoUgOJWhMFPF3rmQqhBKyN3sRiu7NBBWHhMJtUyPnGRtn2bmCKmebC6EEMQAekXLgADQ1NUF9ff0oL0TPtWBxra2tnt6KEEoIhRfsFa7AylVuIRRZ5eUJYeSB4AtuSfne3l5obGyElStXwtSpU0H\/2y2HctwbL8IHt6+A7u7ugkGntzgDAwNQWVnJRl5VUK6yc5Ubbc9J9hkzZowY1319QiiuQBdEJl7oiDkVfNwS+OihcNslz3nGxll28VBklZdNM8jcVnmhEoJWdvkpyi9Jj4TSPHMSNM+ssknXxttic2zVtPBcZecqN\/YnrrLbLHeuhIKkMDg46Ln3hEAI96Dgu52dnVBeXg7498KFC333oQihmIZwu8q3eZCZ1BRXuYVQJOQ1Yly5JdrxhZqaGoc4XnrpJejq6homG31j44YNG5x8ituDHsqJux+Av9n7U5NjWcoWDYgGRAO5aEByKLmoXSoVDYgGRAOigaw0kGvIKyshpR7RgGhANCAaMK8BIRTzOpYaRAOiAdEACw0IobAwswgpGhANiAbMa0AIxbyOpQbRgGhANMBCA0IoLMwsQooGRAOiAfMaKBSh4OGSixcvdrTmd9aXebWarQH35KxZs8apxG\/5tPqe3\/0xZlubXulh5aYaox4qml5L0y8prOz6cny\/\/pF+K9MvMazcdBwT7msrQl8P0mSU09eDykrz98IQCnaoRYsWwYoVKxz90P+7XcCVpgKzLkvd5Il7ddQNn2pb9JOb8e+NGzcObw7Nut1J6wsrt64DnGCU+uQirOz6yRPqmCjFcRBWbiJRPIYJ96aVel8PGitEsjb268IQig6gtjJ4UGcJ+l09w4wApKGhwXOTJ5VX6uASVW4EmQULFsChQ4dcT7AO0rNNv4eVHW28fPly6OjocE6UKPUnitzqBLLU+7qX3Yg4x4wZ47zyiU98ItT9UFn2g8IQin5YpN\/hkVkqOM26vG65pFsv\/eoq5UEWR260\/5QpU+DBBx8cvhU0TVtkVVYU2f0uqMuqvWnVE0VuNw\/F66K+tNqXRzko58GDB52QnnrbbR5t8aqzUISi3ugY5cpgmwzi1xY3jySsJxb23DQbdRFVbiTP9evXw\/z582Hp0qWFIBTVC\/WyOfV5tGGYHJuNtqY2RbU5vb9161aYN29eqKvCbZY\/DA6EmUhmLaMQStYaT1Bf1EFGVSHQrFq1yvMwzQRNyuTTKHLju8uWLYM5c+Y4d8PYOpMLq7gostOiFErEBx2iGrYNebwXRW79fiT9MNk82m+yTpsXmxSKUNCIdD+KhLyOdulSJxOUIUr4A8Hk8ccfd\/qBzQMvLOBEkV0PeZWy\/FzlDtMvbLZrYQhFD3GFDQWFMaBN76hyBSXli7TaJazc6jJT1W6lHAYJKzuSqXpCd1D\/sKlfu7UlrNxFItIwNhFCCaOlhO\/IsuGRq3pKOdzh1hXCLiFVv7V54EXp7mFl15PTpR76CSu3W8jL776kKLq38V2b+3VhPBQK73De2Kh6aV4z9VLe6Oa1yc1rAYbNAy8qUIWVXd3YWIQNfmHlVu9LKoLcfv3D5n5dKEKJOkjlfdGAaEA0IBpITwNCKOnpUkoSDYgGRAOsNSCEwtr8IrxoQDQgGkhPA0Io6elSShINiAZEA6w1IITC2vwivGhANCAaSE8DQijp6VJKEg2IBkQDrDUghMLa\/NkJr2+686vZ9IY8dWntrFmzoK2tDcrKygKVoe\/zCPwg4xdoiW1NTY3RawrUc7Oi6C9jdUh1OWhACCUHpXOs0iZCidIW1ValQCjYXjp+yHQ\/K9LpxqZ1xaV8IRQuls5ATv22QDruRN10RrNn9Ajw4EY8HZYevDCotrZ2xL\/TJULqrBjfD5oZe210U2\/1xHLcNnp6yUH\/jpc40Wm+ujeg1ovlq79j3fv374fu7m7o6elx6sbfVT3cdtttsGXLFueiOLwUK4rc+vl1UU4fdiPLIMII+j2DLidVWKYBIRTLDFKqzQk6zE\/1ClBGBFHc0UyzafUQSzolmI5sd9sZ7Hf4p37sjNvf6plXqs795LjmmmugqakJJkyY4ITJdDn0elQCQjndDupUrxWg8nbt2uWcDO12WrKf3G6Eot5YqR9REuR9BRFG0O+l2pel3fE1IIQSX3fypaKBoOMggsJM6llsOqG4fUs3Mi5ZssSZydPj1Q4VbP3a4nf+VZxZvFqvDsBuMqikNDQ0NOKwR5TRS278zY1Q9IumvAgpjmxCKAIBugaEUKRPpKYBNdyjh6S8QFw9q4nOYNIJRQ9TUYPdzmzyynOo5335EYofSIYFXfIEBgcHnaZS6E8v2+3KXpVYd+\/eDXQ2nWokr7OqvEJeak7FS76wsqntEEJJbegUpiAhlMKY0h5BVEBV8yhqmImIhIhnYGAA6F5wN0IJe6VrnoSCMjQ2NgISCeVm\/DyUMIQSVm4vD6W\/v39Ekl4IxZ5xUsSWCKEU0aqWyKQfP06EgmGpBQsWgBquCgp5ITB3dnZCefnIY\/p1UfMMeWEyXQfwpCGvsHJLyMuSTs+8GUIozDtAWuIHJc7VMBO+i8ltDMXgiinyKnAFlJqM1pPyahLfL9eRZlJeBeq5c+eOaDf+ps74kVBUj4JCdV4hLyobPRo1ya8n5cPK7ZWUJ29JJW0174TtIPtRXWQTWoDgtk9HQl5pjZ7ilCOEUhxb5i6JnjtQ8yg6aWDCefbs2U6bEcRWrlzpJJXr6+uhrq7OAWrMHxAY60t5gzbv+d2PEbRAQK+L5NCJUCcU\/FtdAoxtr6qqgo0bNzre1SOPPDKCcFQgp+XTuGz4iSeegI6ODscbiyK3G6E89NBDjo7xWmR81GXSbjkdCtmhfpFAt23b5pBdkOxhNobm3kGlAcY1IIRiXMVSgWggvAbc8iphvw6zyitsWWHeEw8ljJZ4vSOEwsveIq1FGnBbQOC3zySo6UIoQRqS301rQAjFtIalfNGAjwb0nfUU4oujNP0sL7cQW5xy9W\/kLK80tFjMMv4fq+27vBjlU5QAAAAASUVORK5CYII=","height":243,"width":404}}
%---
%[output:62be428f]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAZQAAADzCAYAAAChQ+D2AAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ1wV9WVP1q7GlsVI1qJAYIa1K4z8WNxEbXUUqUfQ+x02k1CZ2VjSmmrznQBScC6DlshCRK7FtSyGCndKYHZtqywOxVttCpFthU03arVaIgQU1sMUG3Faa3snEdPenPzvt+7793\/O+fNOGP4v3fvPefc+\/vdc879OObIkSNHQB7RgGhANCAaEA0k1MAxQigJNSifiwZEA6IB0YCjASEU6QiiAdGAaEA0kIoGhFBSUaMUIhoQDYgGRANCKNIHPDVw4MABaGpqcn7v7OyE8vJyI9ratGkTLF68GFpbW6Gurs6pA\/8NH\/o7zYrd5Dp8+DAsW7YM5syZA9XV1WlW51lWb28vNDY2ws033xxKzjht3LlzJzz++OPQ3NxsRCbSZU9Pj1P+hg0bYOrUqaHrcrN96I8NvYh6bmlpgYqKCmN6M9T03IsVQsndBNIAXQOmQUYnlLKyMgdAdu3aBevWrcuMUNrb2wEBPwxZE8hFaSOWPXv2bJg3b54xYKQ61MlAlB5t2tZR2qK+i+1atWpVpv0hbltt+k4IJWNrIIisWbPGqRVnQCqA0eCaP38+dHd3A8769Hf0GaEKFjTjvfTSS+Gkk05yZov4BA12qldvkw68Q0NDzoyaZvA486Wyw5aBXo4OQiqoYBvQW6Fn1qxZ0NbWBgj6+BCw7t27dxiI3cCWdDE4OOh8p+pJlWv16tWwYsUK2Lp163CdKFNtba1DMvq\/qx4T2RJttHLlSsC\/J0yYMNxevQ2qHeg3lI+8B922ZPvKykrPtqh6RwFIX9h3kEzoqampcfSFD3qd5FEEkQ3plvRA5aAd9brV39Rh5ddnVdv39\/c7Y0Pv89RfdFmwDaRHvU9ec801w3KiTq677jr44he\/OMoLpr6m1+lmn4yhoiSrE0LJ0GwqmajVUphAH6BBYEC\/E1DpAEa\/64NFn4mpAK6SymmnnTYi5EWEQiBN5e7evXsECfiVkZRQsGzSE+lNJVIkn4GBAYf4qJ06OSFIUijPi1AI3FRdqXp0A9P9+\/cDkrlfG3Rbk+104FZt79XGiRMnjiANtT\/ovyHY33nnnXDLLbcMk4nef\/Sh4NUmL7u7EYpOJlQHEZlXnydi9LIlfa\/3eWzbfffdB\/fff\/+IycBVV10FTz75pOsEyI2osgr3Zgg\/mVQlhJKJmmF4JnX66acPz6xp5kWDZ8uWLQ4w09\/YNJolk7eBs04dhGi2ToCP36Hno85s3WLbNGgQCMlTUmeMNMvD8nB2q5ePs8KoZQQRCnoAQWEQffaov0\/E7QbWqIfJkyePIMowIS8qU\/0eZ\/k6Qei2pN9JT+TBfOtb33Jm4\/S7m+elds0wIS89xOX1t1f\/0XNkev9EPZGudULw8oL193WgfuSRR0b0eZXs3UKBXpMH6vPYJ6ndRHBkX\/SyVO9T9XLdQndoc\/wmyzBoRnBkrBohFGOqHVmwG0jqIEKDSwV\/taNjibo3oXoD+P84M6dZsgoAboSiD04KK1HLvUJeavlRy0iDUFS90eydwAHb7raQQNWjTpR+hKLPoFGP6LnpevYiDL17IchRm\/V8iFf4CtvnRyhe4T2dULy8AS8PViVRSrTrctIkyItQ3Mpw85CDSE73dHQPxq3Pq21ysz+F\/dT2qCHAoLZnBB0lVY0QSkbmcpsBeRGK10DwIhT8dy+g08NDqrhRyYA8lKSEopNr0N9uJqJvbrvtNsd7olyE10w\/KqHoYKL+nYRQ1JCMV4LdLc9G3qb6TViPJCi8RP1HX53l1neyJhS9z1EITA8tpkUoas5OCCU6OAqhRNdZrC8IvNMMeekNcSMIP0JRZ33kwaggNXfuXNccijp4w5ZBYTU1DKcn9L3+dlO4PitXPbCkIS89d6SGTOKGvKKGr\/B9lWhpkYBKKDrg6eGloJBXUEdOM+Slh3FJDsq\/eXko5LXT73qbdIJBW8UJebnpQgglqIeM\/l0IJbrOYn9hKikfxv332h\/gFgahEIhXUl4lFBX4VMX4rVCi94IIBd\/zWjmEv5E+9Xe8FieQnvQ4vUoYWO7111\/vJK7dQiJeCyiwDWGS8uQt6GDllbz20iOWgw+tGHQL26iro7Ccu+++G+64445Rcukr6aisoKQ85iuC8l1hk\/JBhKIPOr8+79buMEl53VOTHEp0qBNCia6zRF+kvWxYBdOoHgoJ4pYnwPBHmBxKUBn4uwrw2F70fG666aZRK24IVFQQ8iMUv30WYZcNU+JXBV8E6+nTpw+voELwuuGGG+DGG28cDq3phIbLhhcuXOi7bFgFbrcQqBv4uuXTsG5sI3mQtLz8nnvugQceeAAon6QSpT5JILL00y\/W47dsWPeivDaheuU\/1ByfF6HoZI\/6wOXqlCzHNuj5LPw3tU7VnvrmWTUnqf6mh\/b0\/GIiECjwx9YQCgLAokWLnD0BbjuV9bXofkthS9VeQbO9UpWraO1WQVZfsq17b16yE2AhcZvaxV40vYeVhyYT+L7b6sUwpy\/IPpSw2h75nhWEEmZJJIIt7g0o8uATQonXifP4yit8GbSJVG1rlJ3yechYqnUGhQ\/DHK2DY1F2ykfvAVYQCnofOLjw8fJQ8PeqqqpQZx5FV4MdXwih2GGHMK1wi9MH7TrXy6VZMIbLopx\/FaZ93N9xy6OFPWdMzvKK33tyJxScTSxduhQaGhocUnEjFDLwtGnTCk0o8c0oX4oGRAOigfw1kDuh4Kwcn0suucQzh+LmwkYJLeSvZmmBaEA0IBoovgZyJRR0S9evXw+33nqrc\/6SV1JeDw1IqKD4HVMkFA2IBkpPA7kSCoa4cHkmxo+DVnnpqqWci1uS\/uyzzy49S0iLRQOiAdFACA3gSeSTJk0K8Wb2r+RGKF4rMVAFYZJnfkl6JJS+vr7stZlzjVzlRrVzlZ2r3GJzO\/EtN0LRsdfPQ6FVYOomNNxE5nUKKNdBxlVuARc7wcX0\/Iprf7dZbmsJBUmkq6tr+LIifWOjnxdjs8JNDjKucguhCKGYHFe2lW3zOLeGUNI0ms0KT1NOvaw9e\/ZYG1s1KTeWzVV2rnJztrnN+CaEYhrpMixfwMXORKXJLiA252dzIRSTI8qlbJsVblIVAi78wEVszs\/mNuObeCgmET7jsgVc+IGL2JyfzYVQMgZWmxVuUhUCLvzARWzOz+Y245t4KCYRPuOyBVz4gYvYnJ\/NhVAyBlabFW5SFQIu\/MBFbM7P5jbjm3goJhE+47IFXPiBi9icn82FUDIGVpsVblIVAi78wEVszs\/mNuObeCgmET7jsgVc+IGL2JyfzYVQMgZWmxVuUhUCLvzARWzOz+Y245t4KCYRPuOyBVz4gYvYnJ\/NhVAyBlabFW5SFQIu\/MBFbM7P5jbjm3goJhE+47IFXPiBi9icn82FUDIGVpsVblIVAi78wEVszs\/mNuObeCgmET7jsgVc+IGL2JyfzYVQMgZWmxVuUhUCLvzARWzOz+Y245t4KCYRPuOyBVz4gYvYnJ\/NhVAyBlabFW5SFQIu\/MBFbM7P5jbjm3goJhE+47IFXPiBi9icn82FUDIGVpsVblIVAi78wEVszs\/mNuNbLA\/lwIED0NTUBD09PaHxsaamBjZv3hz6\/SQv2qzwJHIFfSvgwg9cxOb8bG4zvsUmlAULFsCSJUuguro6COegt7cXli9fDuvWrQt8N40XbFZ4GvJ5lSHgwg9cxOb8bG4zvsUiFJOgmEbZNis8DfmEUEZrgCuwcpUbewBX2W3Gt1iEQiEvNGpnZyeUl5ebxMnIZdus8MjCRPiA6wDjDC5ic\/FQIkCE8VdjEQq26vDhw9DS0gJbt251GrlhwwaYOnWq8QaHqUAIJYyWivUOV2DlKjfnSYTN+BabUFQ42rlzJ8yePdv5p1mzZkFbWxuUlZXlhlg2K9ykUgRc+M1Wxea8bL795UPw2X\/dCL\/97pdNQknsslMhFKpd9VoqKiqcJHyYpH3s1nt8KISStkbtL48rsHKVm6uH0vXz1+HGrhfgwF1XWzkoUyUUVUJa2dXR0ZF5jkUIxcq+ZrRRXIGVq9xCKAwIRTwUo5gZWLiAC6\/wB1dQpYHAsb+z8FAkhxKI9Zm8wHGAcQYXIZQ9MGkSr0lE+7Z+aN+2p3ghL32VV2trK9TV1WUCnEGVSMgrSEPF+50rmXKVmyuZFpJQaB\/K\/v37c0u8+0GiEErxCCNIIq7AylVuIRQGOZSgQZ\/V70IoWWnannq4AitXuYVQCkQo6KHIWV72gCn3PAJXcOEsN1fZCx3yktOG7SIVma3yStByBVXOEyjcg4IrvdjtQzEJte3t7U7xzc3NrtVIyMuk9u0smyuZcpWbK5kKoaSMP7REed68eUIomm4FXMRDSXm4WV0cx\/4uhJJil8SlysuWLYPnnnvOOYhSPJSRyuU4wDiHP7jO0jnbvPaeZ2D7K4ck5JUGr2zatMkppr+\/X0JeLgoVQhEPJY1xViplcOzvQigp9U5cWbZ06VK4\/fbbYe3atUIoQigjNMARXMRD4bdT\/qI7noK9B94ptoeCnsPixYudAY73ouDT1dWV6jH2mIifPn26E+oKk5QntOnu7k6J0uwvZmBgACorK+1vqIEWcpWdq9zYhTjJPmPGDHjvxLHw5rVHFyQVdpUXgvvg4KDjOaAH0dDQEAr0o2AKnly8fv16uPXWW517VsIQSl9fX5QqCvEu11k655m62JxPmBM9E\/RQCkso6gZHnBnjDY5EKGkeX696QCry42Ved9999ygykGXDheDHSEJwBVaucnOcRNCmRpaEgst70ZMwcee8eCjuWCvgwme2Sj1AbM7H5pSQP\/btN+CNb38+0oQrq5cTX7CF3sOOHTtGhLwmT54MTU1NUF9fb+QEYiEUIRRdA1yBlavc3DwUNdx13BsvFvsKYPU+FBroeR5nLyGvrOYj9tTDFVi5ys2NUOhiLZT7g9tXwN6f\/ciewae0JLGHYqNUQig2WsVsm7gCK1e5uRFK+fzHnAE0ofwEePOBL4Cti46EUMziXKalC7jwiadLDgWAQ3\/HUNdNXS84u+Px2fLVi+H6ay8pJqHQRVtBpw77nbtlAnHFQzGhVbvL5AAubhbgKjcXD2XNEwOw+L96h72TZ79+OdiMb4k9FEzKb9y4ccRqLiIaSsoHJdHThiqbFZ62rGp5Ai7ioZjsX7aVXeT+jp7Jwy8MwaIfvDRMJuidYMjLZnxLRChEHHhII+5gVx912fDQ0BAsX77cuS44i8dmhZuUv8gDLEhvXGXnKnfRPRQ6YoXyJkQm+LfN+CaEEoRUJfS7gIt4KCXUXRM3tWj9Hb2S944cgUuW7RzWDXokKpkUmlBQuDAhL9qr4rarPXGvcinAZgY3IS+VWbQBFkVXXGXnKnfRPJTtLx+Cmza+4Bz8SA+SCeZM9MdmfEvkoZCgbvtQ8JBIDIPhbwsXLnTCXdXV1VEwIva7Nis8tlAhPhRwEQ8lRDcpzCtF6O9P9f0OPr169wibIJGsrr8Arjx3jKutbMa3VAjFth5qs8JN6qoIAyyufrjKzlXuUvZQ9KXAqkeyqv4CuMqDSOg9m\/FNCCUugln4nYCLeCgWdktjTSqV\/o4Egv+t2LZneD+JqpQgj4RVyMst3EUKqKmpMXI4ZFAPtZnBg9qe5PdSGWBJZPT6lqvsXOW23UNBAnn61TfhOzte8ySRK84ZA80zJzlLgaM8NuNbIg8F73jHI+unTZsGtbW1w8fXmz4cMkj5Nis8qO1JfhdwEQ8lSf8ptW9t6u9IIC\/+5g+w6tG9rgSCukXimHDqCbBo5iTP\/EgYG9iMb4kIRb0PBRPuuIGxqqrKOWEYPZe0b20Mo2x8x2aFh5Uhzns2DbA47U\/yDVfZucqdt4eCBPJv3a\/Coy8eGLEyS+\/DSCKfvHAsfOUj4yN7Il7jwWZ8S5VQcHlwf38\/4EbHNC\/Yigo0Nis8qixR3hdwEQ8lSn8p9Xez6u9IHqse2wsvvv4HT++DdIkEgqGshinjjnokEcNZYWxiM74lIhQUXj1WRfVKtmzZ4tyT0tbW5lzbm+Vjs8JN6iGrAWZShrhlc5Wdq9wmPBQkjjffeReWbO4NJA6VQD5\/6YdgenV5ojBWlH5vM74lJhT9+BUkmDVr1kBFRUWme09Ug9is8CgdJ+q7Ai7ioUTtM6X8ftz+jpsI3\/7jn2H1Y3th78GjK7DCPOhtfHRyOcz\/+EQjnkeYNtge0k9MKGGVkOV7QihZatuOuuKCix2tj98KrnIHeShEEqt\/shd+9evgUJVqAQpTzb5sHNT\/3Zm5kodbz7AZ3xIRip6UV4WXHEp8kIj7pYCLeChx+04pfvfTnl4YP348PNjzW3jk+aFI3gbKS8RBXof6bzbrgyWhqKcNl5eXZ2ofmxVuUhFCKEIoJvtXHmWjp4HhqXsf3wf9bxwOndvQPQ5crjtl0ilw9eRyY8nyrPRjM77F8lD8NjOqSs36Yi2q22aFm+x0QihCKCb7V9plU1jqyBGAbz+xD54b\/H1kL4PaRN4GrrCqn3ImTCw\/uhDIxCqrtPUQtTyb8S0WoZAC\/EJeUZWU5vs2KzxNOfWyhFCEUEz2r6hlE2G8euAwbPz567APjyCJkATX66ONgeeecSJ8bcZE2LdvH1xRk82Bs1FlN\/m+zfiWiFBMKi1J2TYrPIlcQd8KoQihBPWRNH9HwkCy+NEv34Bfvhbfu3DzMr4yfTwcevtd36W4XPu7zfgWi1DC3iUvZ3mlOXyDy+I6wFAzXGU3ITcdZnjWmOOhc8dr8It9byXyLFTCwFzG+eM+ADdMOwtO\/Jv3OSEprC9OaMqE7MGjLP83Ckco+avUvwU2K9yk7rgOMCGUcJ6Zut+if+gw\/PCZ30Lf\/rdTIwsnZ3HqCXDt346FiypPGiaJOGQRZpxw7e8241ssDyWMsfN8x2aFm9QL1wHGnVDed8q44W6FK6L+439\/Df83kI5X4ZDEX44PQbKYfl45\/H3VKVaslOLa323Gt1QIBc\/wWrx48QisbG1tdQ6JzOOxWeEm9cF1gBWVUFSP4q133nWWziZNbOv9T10dde2HT4OLx5\/svJIkFGWyj6tlc+3vNuNbYkIJc6d8Vh2M6rFZ4SZ1wXWAlRqhqETxq9f\/4GzMM0UU6FVcNukUmD1lHBz3vmNGdD9ToSiTfVwIxe7T1BMRin6Ol2ps2diY1bD6az1CKOFyCaYsg2dEIUj\/6c\/vwfd+9jo83f87p6okS2W9PAokignlZTDr3GPggnMmDie2S50kotiGa3+3ecIshBKlB1v+LtcBZtJDoRVP4089Hrb8Yj888sIQwJF0SULPU5x35gcAl80ed+xfPQovohCb5zuJyAMSCksoqEwJeeXRpdzrFHAJBy5EEpV\/IYkfPz+Uuiehk8T48hPg8nPGwEfOPXXEMtmkHoXYPJzN7RmlyVtSaEIhUpGkfPKOkrQEzuBCBwX2DLwFO1455BzjkXa4SScJ\/PvSiSdD47Szjtb1l\/0USUkiSj\/gbHOusheWUCiHUl9fn9uKLrfBZ7PCo4BF1HeLMsD0+ykefn4Intn3ZuqJa9KvuiwW\/+1j558Gn7nodDj2GPuT2EWxedS+bjLMGactWX5jM74lyqGgEvWDIjds2ABTp07NUr+j6rJZ4SYVYzO4qCRx8O0\/wfd3\/wZ69r1lxItwCzedf+YH4LqaM4bVXwrLYsP0FZttHqb9Sd7hKrvN+JaYUNQOoe5HkRsbkwyVeN9mPcAoF1Ex5nhn1\/UTLx0wRhA6SUw4rQyuqzkdyt5\/9PgOrgcFZm3zeD3TzFdcZWdDKGq3wauA0Xvp7OwEuQ\/FzIDSS006wMiLeO\/IEbh\/+2uAnkTa+yPUNuuhJsxH\/OPUCmd1E7blynPHhFZcUtlDV2TZi1zllpBXn2U98WhzUvVQ6D55LDjMwZCHDx+GlpYW2Lp1q9MYv\/tT9NCanwdkM4Ob7AUquOh5iKdffRO6fzWUCUE43sSpJ8DFE06GT104FsadcvwIsU0krbkCK1e5hVAKSihJwlxIQPg0NzdDUIIf6+nv73feDXqKTCgqUezsOwSP9x7MhCSQIM4540T4aPWpDlHQY0sugiuwcpVbCKWAhOK3Uz4I9N1+VwlG\/x1\/q6qqCrWarJQJhQjjmz9+FV5J6SRYrzAT7o342HnlMKXqlFyWvMbpI17fcAVWrnILoRSQUNIEBD9yotDYtGnTCkMoSBx3\/fhV5\/jw7a8cSqRKCiGdXgZwUdVpMPfKSjjh\/ceO8CISVVACH3MFVq5yC6EIoXjCEuVeZs2aBW1tbVBWdvQ+aHrcLvTyO83YNg8lLnmoSeuPTD4V\/uHSM31zEQIu\/HZNi8352dw2fFNBKdWkfNKJLBLL4ODgKFLp7e2FxsZGWLlypbPHRf9brxcVTk93d3fSZsX6vv\/gn2D5Y0Ow67V3fL+vOPk4GHfScXBVVRl8+EPHO\/+P\/xbnGRgYgMrKyjiflvw3XGXnKjd2WE6yz5gxY8QY7esTDyUQtJAoFi1aBCtWrIDq6mrf9\/3yLXkxOJ42u2LbHs8QFnocV5wzBppnTop15WmQAmW2ym+2KjbnZ\/O88C0If\/D3RB4KhqIWLFgAS5YsGUUASA7Lly+Hjo6O0PtQohx575ekz1LhGM56dt+b8E\/rnxulbySQz1x0hnN\/tomlsnqFAi78wEVszs\/mWeJbGBJJLeTlRyhhyEH1MijxjvtL9KXBeln498KFC2HdunWunkwWCqfVWBfd8dSovMbMD4+F9s\/6e1hRDRXmfQEXfuAiNudn8yzwLQzeuL0Ty0PRNxl6Ve63URG\/0Tc2qkl5rKOrq2s4nxLlzDDTCkcyqb33GWepLT3ogayuvyDS7u64RvP6TsCFH7iIzfnZ3DS+JcGlWIRCFfp5KEkalfRbkwrHPAmSiUokW756cSYhrSC9CLjwAxexOT+bm8S3IIwJ+j0RoQQVntfvJhSO3shDz70BLZt7h8X6wmXj4JZrq6wgE2yUgAs\/cBGb87O5CXxLC6tjEQrtC5k7dy6sXbsWenp6XNsT5jyvtARRy0lb4UgmXT9\/Hdq37XGqsSG85aY3ARd+4CI252fztPEtTQyORShpNsBEWWkqHMnkzof74Xs\/+\/UwmdgS4tJ1J+DCD1zE5vxsnia+pY2\/QigBGv1m96vwjf85uokIPRNbyURCXntg0iR+4CKEws\/mhSUUtyNRVHwu9ZAXhrlu7HqhJMhECEUIJe3Zpu3lcSXTwhKKV4fD5cDLli2DOXPmBO54N9Fp01A4hrpoj4mtORMJef1VA1zBhavcnCdQaeCbCdzFMo2FvPR9JKYEcCs3qcL1fSYY5opye2CWsqp1CbjwC3+IzfnZPCm+mcQnY4QS5+iVtARNqvCtv9gPc77zS6c5jdPOgo7PTU6raUbLEXDhBy5ic342T4pvJkHIGKF4nRxsUhgqO4nC9VDXs1+\/PIsmp1KHgAs\/cBGb87N5EnxLBWh8CklEKH5Jeb87300LlUThtfc8M3xaMJJJFoc6pqUPARd+4CI252fzJPiWFtZ4lZOIUEw3Lm75cRWuHqty5TljYMuNF8dtQi7fCbjwAxexOT+bx8W3LEApMaHo1\/P6nRqchUBYRxyFY6jrpq4XHO\/E9v0mXnoUcOEHLmJzfjaPg29ZYW9iQnG76CpvUomjcNU7mXN5BXzz8+dlZYPU6hFw4QcuYnN+No+Db6mBTEBBiQgl7Qu20hI6jsIpd1Kq3gnqTsCFH7iIzfnZPA6+pYWtQeUkJpSmpibnQiy86119wlywFdS4uL9HVXip505ITwIu\/MBFbM7P5lHxLS6OxvkuEaFghZs2bYKNGzdCZ2fn8FW\/tPqrvr4e6urq4rQr0TdRFV5777Ow\/eWDTp2ltrJLVZSACz9wEZvzs3lUfEsEphE\/TkwoWJ\/bDY6tra25kEnUpHwp7zvRbS3gwg9cxOb8bF54QolIYsZfj6Jw9QDIUjliRVZ5jdYAV2DlKjfnnGEUfDMOtloFiTwUWs3V0NAwKoeStSBqfVEUribjS2lXvJt+BVz4zVbF5vxsHgXfssbhRIRS6nfKq+Gu+R+fCF\/\/1NlZ6z\/V+gRc+IGL2JyfzQtLKJSU7+\/vd1Z62fKEVXj7tv7ha31LPdzFOQTAWXYhFCEUW3AX25HYQ8Flw6V6p3yRwl2cQZWz7EIoQiiFIRSbBImaQ1HDXV+bMRH+5dOlHe7iDKqcZRdCEUKxCYcTeSg2CRKVUIoW7uIMqpxlF0IRQrEJh2MRCm1cnDt3Lqxdu7YkQ15FC3dxBlXOsguhCKGUPKHYJIBbW4KS8mq4qxSPqffSv4ALP3ARm\/OzeRC+5YnPsTwUtcGleHx9kTYzqrYQcOEHLmJzfjYvNKGU4vH1pXwro9\/sQ8CFH7iIzfnZvLCEUqrH15fPf8zBZTyqvtR3x4uHclQDXIGVq9ycbV5oQim14+vV\/EnzzEnQPLMqz5BjqnULuPCbrYrN+dm8sISCaFhqx9cXNX\/CecbGWXYhFCGUVGelCQtLnJTH+kvp+Ho1f3LgrqsTqs+uzwVc+IGL2JyfzQvtodgFqUdb46fwouZPOM\/SOcsuhCKEYhMGp+Kh2CSQH6EUOX\/CGVQ5yy6EIoRiE\/6yIpQi5084gypn2YVQhFCEUGJoQM3TVFRUwLp166C6utq1JK+Q141dLwCSCj5Fy59wBlXOsguhCKHEgFNjn5SEh9Lb2wuLFi2CFStWOCTitrJM1ZAXoRTx\/C5VbgEXfuAiNudn80In5RHsGxsbYXBwcBTr1dTUQGdnJ5SXl6fKiDrB6IV7KZwS8kU6v0sI5agGuAIrV7k527ywhELneGEIKssbG9FD2bFjB7S1tUFZWdkosnJT+PaXD0Htvc847xbhdkY3hhZw4TdbFZvzs3lhCSXrO+VVb2jDhg0wderU0DmUoifkOc\/YOMsuhCKEkmr4J2FhiXIo5KE0NDR4gnvC9rl+TsSycuVK13qQdJ\/vAAAOh0lEQVSRwenp7u52\/vdLP3wddr32jvP\/u24uznErqoIGBgagsrLShMqtL5Or7Fzlxg7JSfYZM2aMGIN9fX1WjslEhIISBYWfTEntdsox1eXmEhY9Ic95ls5ZdvFQxEMxhbFxyk1EKHRzY09Pj2vdppLy+h0seuU6oRT1Qi1dbgEXfuAiNudn88LmUOIwWJxv9FVduCdl4cKFnntRdIWrCfminTCs6lPAhR+4iM352VwIJQ6LaN\/oB1BGScpzWOHFOezDWXYhFCGUFOA1tSIShbyoFZhHWbx4sfMnAj0+XV1dnst6U2u9R0E6g7dv64f2bXuct\/FCLbxYq4iPgAs\/cBGb87N5oT0UTI7jpsbbb78dli5dCrTiyy9pbhrMdYVzSMhznqVzll0IRQjFNJ5GKT+Rh6LuQ8Hlqi0tLcOEgnmP5cuXQ0dHR+o75YME1Aml6DvkSR8CLvzARWzOz+aF9VD8CAVzHuilmDh6JQqhqCu8GqedBR2fmxz0ecn+LuDCD1zE5vxsXlhCQeSlfShqyGvy5MmAd83X19dDXV1d5gCtKrzod6DIKq+jGuAKrFzl5mzzQhMKGtbmK4DVhHxRz\/CSkJcQSuazNgsq5EqmhScUC\/rWiCaoCv\/3JwegZXNv4Vd4cZ6xcZadK6hytrkQSsaMoyqcywovzgOMs+xCKJJDyRhefatLtMqLSlb3odC\/+W08NK0AlVAuuuMpwDxKUe9AkRyK5FAmTeIHqpwnEYX2UNxuT6QzvvJOyqsJ+Y+dXw7f\/1KNaS7LtXyZrfIDVrE5P5sXllCIOPByLf1uEluWDdMelCKf4SVJeUnK5zqTyalyrmTKklBs2NionuFV5CNXhFCEUHLC9FyrFULJVf2ulSfOobid\/GtLyIvTkmHOMWXOsnMFVc42L7yH4nUfikpheDfK5s2bM6FUUvjS\/34F7n50r1PngbuuzqTuPCsRcOEXTxeb87N5YQklT\/D0q5sUzmnJMOcZG2fZhVCEUGzC4cQhL5uEobYQoXBaMswZVDnLLoQihGITBqdCKG77UFpbW3M5xwuVi4Tyk6efByQUfDjsQeEMqpxlF0IRQikUodi8D4XTkmHOoMpZdiEUIZTCEIrN+1C++\/BuqL33GUfX9zRcAA1TzrRJ70baIuDCD1zE5vxsXtikvM2EMu++R4ev\/S36KcPETgIu\/MBFbM7P5oUlFAQyW0NeKqFw2NTIOezDWXYhFCEUI+GOmIUWNil\/4YIfwPZXDjlq4bAHhTOocpZdCEUIJSb2G\/ksFUIx0rIEhaJLSIQyofwEQA+FwyPgwg9cxOb8bF7okJeNQI0KP\/SZTlZLhjnP0jnLLoQihGITBhfSQ6m68DJ489p2R8+4ugtXeXF4BFz4gYvYnJ\/NxUPJGM1VQuGywovzLJ2z7EIoQigZw6tvdYX0UCZc9kn4\/ZWLHMG57EHhDKqcZRdCEUIRQjGsAZVQxEMxrGxLiucKrFzl5jyJkJBXxqBT8al\/hnfOr3VqFULJWPk5VccVWLnKLYTSl9NI86+2kCGvMz93B\/xxwhWO5Fz2oHAeYJxlF0KRkJdNzFJIQjnj+m\/Du2PPA057UDiDKmfZhVCEUIRQDGtg7Jf\/E947cawQimE921Q8V2DlKjfnSYTkUDJGHjq2nss9KKReARd+s1WxOT+bC6HkRCg3fnQ8fKP23Ixrz686ARd+4CI252dzIZSMMZbbxVrioQBwBVauckvIS1Z5ZUYrRCicNjVyHmCcZRdCEQ8lM2ANUVGuq7zogq6enh6nqbNmzYK2tjYoKysb1fSdO3fC7Nmzh\/+9oqIC1q1bB9XV1aPeJULhtAeFM6hyll0IRQglBM5n9kpuhHL48GFoaWmBadOmQV1dHdDfSBTNzc2jFIAXefX397v+pr8shJJZ\/7GmIq7AylVuzpMIyaGEhB0kjR07drh6Ke3t7VBVVeWQT9BDhMLlpkbJoUgOJWhMFPF3rmQqhBKyN3sRiu7NBBWHhMJtUyPnGRtn2bmCKmebC6EEMQAekXLgADQ1NUF9ff0oL0TPtWBxra2tnt6KEEoIhRfsFa7AylVuIRRZ5eUJYeSB4AtuSfne3l5obGyElStXwtSpU0H\/2y2HctwbL8IHt6+A7u7ugkGntzgDAwNQWVnJRl5VUK6yc5Ubbc9J9hkzZowY1319QiiuQBdEJl7oiDkVfNwS+OihcNslz3nGxll28VBklZdNM8jcVnmhEoJWdvkpyi9Jj4TSPHMSNM+ssknXxttic2zVtPBcZecqN\/YnrrLbLHeuhIKkMDg46Ln3hEAI96Dgu52dnVBeXg7498KFC333oQihmIZwu8q3eZCZ1BRXuYVQJOQ1Yly5JdrxhZqaGoc4XnrpJejq6homG31j44YNG5x8ituDHsqJux+Av9n7U5NjWcoWDYgGRAO5aEByKLmoXSoVDYgGRAOigaw0kGvIKyshpR7RgGhANCAaMK8BIRTzOpYaRAOiAdEACw0IobAwswgpGhANiAbMa0AIxbyOpQbRgGhANMBCA0IoLMwsQooGRAOiAfMaKBSh4OGSixcvdrTmd9aXebWarQH35KxZs8apxG\/5tPqe3\/0xZlubXulh5aYaox4qml5L0y8prOz6cny\/\/pF+K9MvMazcdBwT7msrQl8P0mSU09eDykrz98IQCnaoRYsWwYoVKxz90P+7XcCVpgKzLkvd5Il7ddQNn2pb9JOb8e+NGzcObw7Nut1J6wsrt64DnGCU+uQirOz6yRPqmCjFcRBWbiJRPIYJ96aVel8PGitEsjb268IQig6gtjJ4UGcJ+l09w4wApKGhwXOTJ5VX6uASVW4EmQULFsChQ4dcT7AO0rNNv4eVHW28fPly6OjocE6UKPUnitzqBLLU+7qX3Yg4x4wZ47zyiU98ItT9UFn2g8IQin5YpN\/hkVkqOM26vG65pFsv\/eoq5UEWR260\/5QpU+DBBx8cvhU0TVtkVVYU2f0uqMuqvWnVE0VuNw\/F66K+tNqXRzko58GDB52QnnrbbR5t8aqzUISi3ugY5cpgmwzi1xY3jySsJxb23DQbdRFVbiTP9evXw\/z582Hp0qWFIBTVC\/WyOfV5tGGYHJuNtqY2RbU5vb9161aYN29eqKvCbZY\/DA6EmUhmLaMQStYaT1Bf1EFGVSHQrFq1yvMwzQRNyuTTKHLju8uWLYM5c+Y4d8PYOpMLq7gostOiFErEBx2iGrYNebwXRW79fiT9MNk82m+yTpsXmxSKUNCIdD+KhLyOdulSJxOUIUr4A8Hk8ccfd\/qBzQMvLOBEkV0PeZWy\/FzlDtMvbLZrYQhFD3GFDQWFMaBN76hyBSXli7TaJazc6jJT1W6lHAYJKzuSqXpCd1D\/sKlfu7UlrNxFItIwNhFCCaOlhO\/IsuGRq3pKOdzh1hXCLiFVv7V54EXp7mFl15PTpR76CSu3W8jL776kKLq38V2b+3VhPBQK73De2Kh6aV4z9VLe6Oa1yc1rAYbNAy8qUIWVXd3YWIQNfmHlVu9LKoLcfv3D5n5dKEKJOkjlfdGAaEA0IBpITwNCKOnpUkoSDYgGRAOsNSCEwtr8IrxoQDQgGkhPA0Io6elSShINiAZEA6w1IITC2vwivGhANCAaSE8DQijp6VJKEg2IBkQDrDUghMLa\/NkJr2+686vZ9IY8dWntrFmzoK2tDcrKygKVoe\/zCPwg4xdoiW1NTY3RawrUc7Oi6C9jdUh1OWhACCUHpXOs0iZCidIW1ValQCjYXjp+yHQ\/K9LpxqZ1xaV8IRQuls5ATv22QDruRN10RrNn9Ajw4EY8HZYevDCotrZ2xL\/TJULqrBjfD5oZe210U2\/1xHLcNnp6yUH\/jpc40Wm+ujeg1ovlq79j3fv374fu7m7o6elx6sbfVT3cdtttsGXLFueiOLwUK4rc+vl1UU4fdiPLIMII+j2DLidVWKYBIRTLDFKqzQk6zE\/1ClBGBFHc0UyzafUQSzolmI5sd9sZ7Hf4p37sjNvf6plXqs795LjmmmugqakJJkyY4ITJdDn0elQCQjndDupUrxWg8nbt2uWcDO12WrKf3G6Eot5YqR9REuR9BRFG0O+l2pel3fE1IIQSX3fypaKBoOMggsJM6llsOqG4fUs3Mi5ZssSZydPj1Q4VbP3a4nf+VZxZvFqvDsBuMqikNDQ0NOKwR5TRS278zY1Q9IumvAgpjmxCKAIBugaEUKRPpKYBNdyjh6S8QFw9q4nOYNIJRQ9TUYPdzmzyynOo5335EYofSIYFXfIEBgcHnaZS6E8v2+3KXpVYd+\/eDXQ2nWokr7OqvEJeak7FS76wsqntEEJJbegUpiAhlMKY0h5BVEBV8yhqmImIhIhnYGAA6F5wN0IJe6VrnoSCMjQ2NgISCeVm\/DyUMIQSVm4vD6W\/v39Ekl4IxZ5xUsSWCKEU0aqWyKQfP06EgmGpBQsWgBquCgp5ITB3dnZCefnIY\/p1UfMMeWEyXQfwpCGvsHJLyMuSTs+8GUIozDtAWuIHJc7VMBO+i8ltDMXgiinyKnAFlJqM1pPyahLfL9eRZlJeBeq5c+eOaDf+ps74kVBUj4JCdV4hLyobPRo1ya8n5cPK7ZWUJ29JJW0174TtIPtRXWQTWoDgtk9HQl5pjZ7ilCOEUhxb5i6JnjtQ8yg6aWDCefbs2U6bEcRWrlzpJJXr6+uhrq7OAWrMHxAY60t5gzbv+d2PEbRAQK+L5NCJUCcU\/FtdAoxtr6qqgo0bNzre1SOPPDKCcFQgp+XTuGz4iSeegI6ODscbiyK3G6E89NBDjo7xWmR81GXSbjkdCtmhfpFAt23b5pBdkOxhNobm3kGlAcY1IIRiXMVSgWggvAbc8iphvw6zyitsWWHeEw8ljJZ4vSOEwsveIq1FGnBbQOC3zySo6UIoQRqS301rQAjFtIalfNGAjwb0nfUU4oujNP0sL7cQW5xy9W\/kLK80tFjMMv4fq+27vBjlU5QAAAAASUVORK5CYII=","height":243,"width":404}}
%---
