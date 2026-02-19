clear;
[model, options] = init_environment('inv_im_ekf');
%[text] ### Global timing
% simulation length
simlength = 3.25;

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
kp_i = 0.5;
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
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe480V_250kW\nNominal Voltage: 480 V | Nominal Current: 360 A\nCurrent Normalization Data: 509.12 A\nVoltage Normalization Data: 391.92 V\n---------------------------\n","truncated":false}}
%---
%[output:9269bdd1]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv480V_250kW\nNominal Voltage: 400 V | Nominal Current: 470 A\nCurrent Normalization Data: 664.68 A\nVoltage Normalization Data: 326.60 V\n---------------------------\n","truncated":false}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAWEAAADVCAYAAACG00EAAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXX1wVtWZf7BUCWNcEqBiDBBU6Fr+oGq72ugYuqzFjobp0loJf2zMBKRWlk4NS0hkBpxK+Kg4giiDkKHZscaPKd2SndmlFoQZZem2oPmD2qJIQJp1pXwMcQq22brzO\/S83lzej3vf9957zr33d2cckve95+v3POeXx+c853mGffLJJ58IHyJABIgAETCCwDCSsBHcOSgRIAJEQCFAEqYiEAEiQAQMIkASNgg+hyYCRIAIkISpA0SACBABgwiQhA2Cz6GJABEgAiRh6gARIAJEwCACJGGD4HNoIkAEiABJmDpABIgAETCIAEnYIPgcmggQASJAEqYOEAEiQAQMIkASNgg+hyYCRIAIkISpA0SACBABgwiQhA2Cn+ah33nnHWlqapL+\/v4MDPX19bJ69WopKysrCM358+dl6dKl0tDQILfddlvB9\/fv3y9z584d8t6CBQuktbVV\/PZVcDC+QAR8IEAS9gEWXw0OAZDwkiVLZO3atTJ58mTfROiXOEHCa9askc7OTqmsrMyMV1VVpYiYDxEwhQBJ2BTyKR+3EAk7LVdtsQIyEOnmzZulrq5OIYjvYAm\/9NJL0tbWlvnMTaxuEsaLmENHR4c8\/vjj6o8BrOpp06YpC7unp0f1pa1z\/Kw\/x2fnzp2T9vZ2OXjwoOzbt0+OHz8uc+bMkYkTJw6xuF944QWZMmWKtLS0yJ133ik\/+MEPBMT\/xBNPqLX09vbKqlWr5P7770+5RqR3+STh9Mre6MqzuSM0GTkJurq6WpFfbW2tIjhtzZ46dUq5M0BmeLq7u5UrQ5Ol202RjYRPnz6tyPGRRx6RrVu3KhIeP3686uPaa68V\/b2TbDEGiHPx4sWybds2RcIvvvhixsLGONo9gj8MfX19Mn\/+fGlublaf448D1oD3YJW\/+uqrisS9umGMCo2Dh4IASTgUWNlpIQRyWcKabDWpwj+syUz36fbjHjt2LGMF63ec1jM+80rCIErt6oA1DKt106ZNiqQxN1isuchZ+7LdVrwmYcxbW+0gZ\/yOd51rLYQbv08eAiTh5Mk0FitykzAmrckWrga\/JKxJLdfivboj0B4HeE43graUC5GwtsLxLyzbHTt2DLGEScKxUM3IJ0kSjhxyDggE8lnCN998c+bQzqs7QrsHnO87\/az5DuYWLVqUibRwujbcbgftNsj1udMVon3LsKRpCVPn8yFAEqZ+GEGgUIhaGAdzXkLUcIgG\/y2IFu8PDAxccmCX7WBO+3T1ASHIF\/289dZb6g\/KwoULlfuB7ggj6mb1oCRhq8XDydmIQDbXho3z5JzigQBJOB5y4iwNI+AMgcNU4DP2cknE8LQ5fAwQIAnHQEicIhEgAslFgCScXNlyZUSACMQAAZJwDITEKRIBIpBcBEjCyZUtV0YEiEAMECAJx0BInCIRIALJRYAknFzZcmVEgAjEAAGScAyExCkSASKQXARIwsmVLVdGBIhADBAgCcdASJwiESACyUWAJJxc2XJlRIAIxAABknAMhMQpEgEikFwESMLJlS1XRgSIQAwQsJKE3clSgCPrcMVAmzhFIkAEfCNgFQnrHLLZCFcTM7NX+ZYxGxABImAxAtaQMOp2oaZXY2NjXri6uroKvmMx3pwaESACRGAIAtaQMOVCBIgAEUgjAlaRsLPkDVwSeNra2lTFW5QXnzx5chplxDUTASKQYASsIWFdZryhoUF0na85c+aoEuPwB+sii7qseIJlwqURASKQIgSsIWH4hB977DFZvny5VFZWypo1awSlz1FCxv1diuTDpRIBIpBwBEjCCRcwl0cE0oLA4Mk+OX9oj5RPfyBWS049CV933XWxEhgnSwSIwFAExl0+KF8b85F8bfSA+qJ3YIT8sG9sQZjee++9gu9E8YJVJNzc3Cy9vb1Z1z1t2jTp7OxUroogH5CwKWGYHDtIDNlXPBGIs\/5pq3dgT5dcOLRHho+tURZw+Vcb1c+FHpvWbg0JFwKtlO\/1JRD04b4IYlIYR48elUmTJpWyNLYlAkUjEEf9A\/kOvNYlZ15Zoci2bOp0KZ\/eKCOmTveFg8l9756oNSSMw7cwLGH029LSIu3t7WrtHR0dsm7duoxFbVIYcdwEvjSdL1uNQFz0L5fVW\/Ht5UXja3LfW0vCzokhJK2vr09aW1vVx\/gdD8LV\/D6wghFpAVcGwtuWLl0qCIND1AUek8KIyybwiznfjwcCtusfyPfMy4\/JwJ4flWT1ZpOGyX1vPQlnC0crJUQNJNzd3S2rV69WawcJ19bWZgjdpDBs3wTxoJLwZrlmZ1\/Ozo+fPp\/zu\/dPXxjy3fEzF38\/7vrc3cGEyhEyoWKE3H5DhbTOLOzXLHXltuofiPf8b\/ZkfL2weIOOeDC5760nYX1pw0mUsGT7+\/sVkfq9rFGIhMd85xWFybhx40rVad\/tBwf\/T4YP\/4zvdmxgBwLXlA\/POpGqq4Z+XlU+XB68dVTeSR\/4\/QU5cOIiWW\/+77OCPur\/9sqC7UpB4sSJE1JdXV1KF8G1PbJfPjnwE5Ff\/0SkolrkS9+UYdfdKnL9xf9jDfKZMWOG6s7Ugbz1JIwJuv3DpURG0B0RpPqyrygQgMXc\/asPZM3OowLreOOcG+WOG\/KTeDHzMm0J60M2uBvwDP8cIhwaA7d6s2FDS7gYjSmyDQ\/migSOzYwj4CTj1pmTAndRmCBhfch24dDejK\/XT2hZUEIhCWdBMsxUls4QNXc+YpPCMLEJglJi9hMdArCKH+5+W+64fpTsePimwAaOUv\/cVm+xoWVBLd7kvrfaHWEiqbtJYUS5CYJSXvZjBgFYxbOefVMN\/tayrwQyibD1L4zQskAWbjgqymoS1pOLsrwRSTgotWY\/YSMAIl7Y\/ba8fuSs7PjuTSX7icMi4aAuVISJp8l9HwsSDhN8d98mhRHWJogSP44VPQJwTcBFUaqfOEj9s9nqzSYhk\/ueJOxCwKQwgtwE0VMBRzSJAGKYET3R8OVx8kzDjUVNJQj9y3ahYuzD24qaT5SNTO57kjBJOEpd51ghIvD6u2eVn7jYA7tiSVi7G\/SFCuRtiCq0LCg4ScJBIenqx+lLdsYWM4FPSICzW+MIlHJg54eEc4WWlZK\/wSR4JOEC6Dsva2zdulV2796tKiznqzGH+nTO5Dz6lt2SJUtk2bJlTOBjUuM5dqgI6AM7XI\/2c7HDCwlnCy0bMbUukgsVYYJGEs6DrvPaMpL4oMQRHp3\/weu1ZX1defbs2fLUU08xgU+YGs2+rUBA+4lPP\/lVT\/PJR8K4xVZsrl5Pgxt+iSScRwDOZD1btmxRJIzCn876c17kB0u4pqZGJk6cWDCBD\/rbtWuXl24Dfcequ\/uBroydmUKg5+2PZMUv\/iD1N14pK\/5hTN5pZNO\/T15dL4L\/kL\/h+ltl2Ld\/aGopoY3L3BEFoM1mCe\/du9dXAh9nKsxCCXxM\/kX08r+DoWkiO04sAl4P7KB\/468cpuqyua3euPp6vQrV5L53z9GapO7OiXlJ4AMfcFNTkyLn+vr6TIY1bQHr3MNM4ONVLflekhDI5yfWh2wnf\/nvKmuZLg1UNrXOd4WKuGJGEg5JciBguC90wnYMwwQ+IYHNbmOBgL7YgVji+2ouqNJAOmvZ4MRbZOyt98b+kK0YQZCEs6BWankjZxia7l5byCgeOnfuXPUxE\/gUo7JsE2cEnvvhaqnofUlu+fgtZfVOePaoWk6a3WEk4QIaHWR5o0Kbx6Qw0rwJCsmF3xePQLaYXmQt+\/yRf1L5iXUCoDTrn8l9b71POOjyRoVU2aQw0rwJCsmF3\/tHAKXfccDmrMnmjumd9cybgnhiEHGa9c\/kvreehIMub1RIlU0KI82boJBc+L03BJzEixZwNxSqyab9xAv+bpSsmhNcfmJvM7bjLZP73noSxgS9REcEJUqTwiAJByXF9PTjdjVo4kV1Cj9hZWElio+LJEzu+1iQcKmCRPgariuvXbtWXXVm7ohSEWV70wi4KxAHUY\/tjd53pL7rxBA\/sel1RjU+STgP0rmiJLwW+9TujAMHDsi2bdtk9OjR0tLSwtwRUWk3xwkEgZPPNMmfT\/apsu\/a2sXhWpBpIvF\/Yp\/5m2tUoni\/eScCWaTBTkjCPsFHtASuHzvjf3N1gXdPnjwpIOH29nY5deqUIH64s7NTkHdi6dKl0tDQkOnLpDDojvCpCAl5HcQKgsWDgpf4efDDPoGrQRMu\/g2adN3wOfXPb96JuIvC5L6PpTsiW8RENiWAG6Krq0seeuihTOY0kLBO\/oM2IOHa2lrRN+o2\/X25fOub3zKiUwMfDUj5leVGxuagwSCgydTdG0gVjybWbKPhEA1uhc\/iX\/XzxEgvTriNAK\/XnYNBzmwvJGGf+DuvHldWVmZtDTfEypUrVcpLpwuiEAn\/9O4y1d+0L37R56xKf\/3jjz+WK664ovSO2IM5BCquzT52RbUMu+t75ublYeRsCXz6zw3Kgp9+oFojAdAt147w0FO8XmECnwLyyuUTXrVqVcZ6RRfu3BHz5s1TFjBySeinqqpKvv\/978vzzz9Pd0S89glnGwEC+dxhiCdGQVFcd0YJpaQ9tIRDlqgzXwQP5kIGm93HFoFCZxJB1LGzFRyScB7JBHFjzknC7hA15o6wdVtwXlEjUIiEMZ+k+olJwlm0TYeW9fT0ZNVFZ7rKIJXVpDC8bIIg18q+iIATAa\/6V2z5JJvRNrnv3bhYl0\/YayREUAI2KQyvmyCotbIfIlAMCes2+rpz68xJ0jqzJtZgmtz31pKwJt9FixbJ4sWLBeknnY\/Xyxp+NcOkMEjCfqXF94NEoBj9S8p1Z5P73loSDlK5\/PRlUhjFbAI\/a+O7RCAfAsXqH9wTs559U3Xtp7qzTdIwue9jQcK49dbW1kZL2Cat5VwSh0CxJAwgtJ8YYWxeqzvbBCBJOI80dJxwa2urp2vKzq6ch3uIEUbuCCbwsUn1ORebECiFhPU6dBhb3PzEJOECJOy3vH1GIf5a5h5XknHLDlWa58+fzwQ+Nu18zsUaBIIgYSwmjmFsJOECagh3BB6d38GL1uaKqmC1ZS\/o8Z00IhAUCTvdE7pqh+14koQ9uCP8RkdoEkbXiDXW7ohCuSMgDDy7du2KXG+y3d2PfBIcMLUIhKF\/D27\/QA78\/oKgaseDt46yElvmjghJLDqXxBNPPKF8ybCm9+3bJ7Nnz5bt27fL6tWr1cjuLGom\/yIGaYmEBCu7TTACYemfDmNDzgnknrDxMbnv3XhYeVmjubn5kjhhTNx52OZO4INKGsuWLVM5hPVhHPIII6nPpk2bmMDHxp3AORlFICwSdvqJUd15x3dvUtU7bHpIwgWkkavkPRK7Izfw+vXrs\/YA0q2pqVG+ZG0JO8kZjTo6OmTdunWiU2KaFEaYm8Amhedc7EQgbP2z+bqzyX0fC0vYHR3hvE23YcOGnCTsTIPpvGHnrDHHBD52EgJnFT0CYZOwXpGNYWwk4Tz6pmN94XpArDAep1WL3MD68yDU1qQwotoEQeDEPpKHQJT6Z1sYm8l9b70ljAlmK3m\/ceNGVT3ZWZooiG1hUhhRboIgsGIfyUIgav2zyT1hct\/HgoSjVHWTwoh6E0SJK8eyHwFT+mdD1Q6T+z4WJIwDts2bNw+ZK7Oo2b+pOcN4IWCKhIGS6aodJOE8uuqsinHw4EFV6v7YsWOqRaEbdM6wtVwHc+5adSaFYXITxIsuONswEDCtf9pPbCKMzeS+t94Sdl4\/Pnz4cCb\/Q6F8EvpAT\/uMYU3jYe6IMLYv+0wCAqZJGBia8hOThPNoMMgUYWggT1w5bmpqUhWUvbgjnHHC+mdY0vi5s7NTysrK1I25hoaGTIY2k8IwOXYSSIRrKA0Bm\/Qv6jA2m9Zu3Y05qBXyRowcOTJz8w2VNnRaykJqp\/3JOh4YMcK44JHv2nKhPvk9ESAC4SMwOObz8tEdS2TUvzWHP5iIvPfee5GMU2gQK0m40KSzfe\/OQ6zdEXV1dXlJuJix2IYIEIFwEIB7wrYrzuGs9NNeY0vC7twR8+bNkyeffDJzJVmnsCyUOyJsgNk\/ESACRCAfAtaQsPuChnvShXzCbkvYa+4IqgcRIAJEwCQC1pKwO8eDF5C8hKgV06+XsfkOESACRKAYBKwhYefks11bRnSDznxWzELZhggQASJgIwJWkrAbKGeJIhKxjWrEOREBIlAsAlaSsNOtgIXV19erEDPE+fIhAkSACCQJAWtIOFcu4CSBzbUQASJABNwIWEnC2cRUKDqCoiUCRIAIxBEBa0g4juBxzkSACBCBUhEgCZeKINsTASJABEpAgCRcAnhsSgSIABEoFQGScKkIsj0RIAJEoAQESMIlgMemRIAIXIrAhUN75M8n+\/JCM\/jhxUIN2Z7BAm2Dwnzsw9uC6qqkfoySMPI7tLW1DVmAu\/JFSavz0Bh5RfkQASJwEYGZoz+Sq68YlKsv\/7P6fdzlg+pffOb8vRS8PvjT8LzN\/\/fj\/N+XMraz7T\/+5\/mguiqpHyMkjBtwc+fOlWyEq4k5qhwPJpM7mxy7JK1h41gioC3UC4f2Kkv1l\/v3y7TyC5esZfjYGvXZ8M\/VyGf1z3\/9t+Lby2O5dvekbdp7kZMwLmX09PRIY2NjXmF2dXUVfCcIbTApDBvKywSBIfuwE4GBPT8SEO75Q3tE\/y++JtiyqdNl4KMBGXvrvVI+\/QE7FxDirEzue\/eyIifhEHEtqmuTwiAJFyUyNsqCAEgWZDuwp0tg8SpLdmxNxpodMbXuErJNs\/6Z3PfWkHC+\/MFVVVWeyxk5F6TdHPoz7e5wfs5qy+SwpCAA4h14rUvO\/2aPIl6QLixcEC7cCCOmTs+7VJIwyxsJ\/L99fX3S2tqqlAW\/40FxTtSFW79+va\/94u4PjUH2LS0t0t7ervrq6OjIVN\/A7yb\/IqZ5E\/gSLF\/OIJCNeOFOKMZXm2b9M7nvrbKE3WXsdbn7RYsWqYrLfknYWW1ZL9SZBtO2astp3gTkVX8IwL+rXQ2weCc8e9RfB1neTrP+kYRFBKXtUX4ergenJbxv3z5ZsmSJPP\/885nPvWhbrkTwhw8fLlht2VTV1TRvAi8y5TsiZ15+TEDAeOBqCDK2Nc36RxL+6+7KRpwbN26UtWvXSm1trdx\/\/\/1F70NdY2727Nmyffv2giXvd+3aVfRYxTY8ceKEVFdXF9uc7RKMwCcv\/4vIr38iUlEt8qVvyrC7vhf4atOqfzNmzFBYmjK+rHFHaH9tc3Oz9Pb2ytatW2X37t0qLG3y5MklK5zXassm\/yKm2RIpWcAJ68Ad3YBDtfLpjaGGj6VZ\/0zue2tIWLsjYPHicK6urk7NDQdyxVTR0P7k5cuXq1p08A\/jmT9\/Pg\/mEkZYSVqOPmg788qKTHQDyLdQZEMQGJCEUx4d4STNLVu2KBKeMmWKuA\/r\/CibMxTNmQTe+bn7Jp7Jv4hp3gR+5Jq0d91WLw7aio1wKAWbNOufyX1vtSW8d+9e6e\/vL8oSLlYZTQojzZugWHnFuZ2+wYZ\/dUxvVFZvNtzSrH8m9701JOz2CeN3EyWMTAojzZsgzmTqde7a4sXVYU28yMdQteI1r12E+l6a9c\/kvreKhEPVMI+dmxRGmjeBR\/HE8rWTzzSpBDn6FhuIN+xDtmKASrP+mdz3xkk433XlsKxhXlsuZouyTT4EnFauJlz9vnY1BBnTG4Y0SMIpP5iDUuW6tlxKfLBbWXltOYztm54+tR8XK85Gtvhc52uIWzYyknDKSdgdUqZ9xKVER2SjBl5bTg9h5lopbp3hcVZscFZ+GPywb8h3zn7cuXVtt279SJsknHISdsYJa8sXsb1BR0eAhHXsMRQUV6Wdt\/Heu2+YH73luzFCoH\/4uMxsq8qHi1Q6bidWXPvpSiqqQ7mRZjtUvDGXchLWlq++MRemPzgfCZt00MfdEun+1QcFeeb46UsrNxRslOOF46cLl6N53zXe8TOfju9lLhMqR6jRJ1SMkPGVI2RCZZm0zrxYaSJpT9z1rxR5mNz37nknPqk73RGlqGo6267Z2ScgfBA6SNxN3iBqkPTtN1TI7dePkjtuGBVLoEjCKbWEoy5vxIO5WPKD1ZMGSb\/x7hl5\/cjZzDxBzA1fvkYu\/vupG8TmhZCEU0rCUMqoC33y2rLNVBD\/ub3+7ll548jZIcTcOnOS9W4MknCKSVhvO1tK3ptKaZfmTRB\/6s2\/goe73xb4zLWFbKNfOc36R5+wRTvQpDDSvAksUoHQpgJfMoh4zc6jiow3zrnRKv9xmvXP5L5P9MEcC32GxifsuEQE4EcGGd9x\/SjZ8fBNJfYWTHOSMN0RwWiSoxcW+gwcUnYYIAKwjGc9+6bq8a1lXwmw5+K6IgmThIvTnDytWOgzcEjZYQgIzHrmTRVZseO7Nxl1T5CEScKqHH1Q5Y1Y6DMEtmCXoSFgAxGThFNOwkGXN3LvFhb6DI0\/2HGACNzydJ88N3uc3HLtxZt6UT68tpxyEi61vNE777wjTU1NKtdEfX39JdU4WOgzyu3MsYpFABYxbuWZ8BHTEk45CWezhEspb8RCn8XSANuZRuCLj\/+XugYdddQESTjlJAzFz+XHRbXkYh4W+iwGNbaxAYHKR16L\/KCOJEwSziR1R1l6fUDnroYc9gYxGbSd5k0Qtlzj1r+OIz795Fcjm3qa9c\/kvncL2FgWNbgjVq5cKY2NjXLw4EHp6+uT2bNnS1dXlzz66KNSVlYWiTKaFEaaN0Ekwo3ZIPAPI33mMw03RjLzNOufyX1vDQm7D+ZqamrkrrvuEj+VNdyXM5zuDedhHWvMRbKnOUgACETpliAJp9wdoS3he++9VzZv3izt7e1Khb1awriYgXYLFiyQ1tZW1VZf1pg1a5aqoNHQ0CBTpkyRlpaWTP8dHR2ybt060X5nk38R07wJAuCrRHYRZbREmvXP5L63xhLGRLSFCiKFX1iT5eTJk\/NuMFjAEydOFERT4AEJaysYP992220Zf3NdXZ0i587OTuXi0OSMd\/CYFEaaN0EiGTSgRcEajiIVZpr1z+S+t4qES9VZkKuThJ0k7ueyBlNZlioJtg8SAWReQypMxA7rcktB9q\/7Igmn3B2h3QdwKTifadOmKavVS5haUCSM8Xft2hWGnuftM603liIHOoYDPrj9Yv0+3KYL60mr\/s2YMUNBasr4ssYSdpYdQnQE3AvHjh1T89PVl\/GzvtTR09MjVVVVsm3bNtHuCjcJI8yN7oiwtiz7jRIBVOtAxrUwk\/zQEk65JeyMjjh8+LDy78Iv7Cc6wknC2rJGlAUP5qKkC44VFgKIHe7+1f+EdqWZJJxyEoaFu2HDBkW8p06dyuSBKNYdgY3gDFFzRk2wxlxYNMF+w0YAV5pR0TmM2GGScMpJGMrb29srI0eOVO4FEOXixYuHuBvCVnD0b\/KUNM2bIArZJmEM7ZYACQddxTnN+mdy31vjE7Zlg5gURpo3gS3yj8M8wvIPp1n\/TO57q0g4W7VlP+6IIDaQSWGkeRMEIbu09RH0bbo065\/JfW8NCbsvVxSzodzXllnosxgU2SYuCOhqHEEl+SEJp9wn7M7\/63cjZLu2zEKfflHk+3FDQBNxEDfqSMIpJ2EoP0gTjzMu2MumyHZtGe1Y6NMLenwn7gjotJc4qCslaoIknFISdidyd28IPz7hbJc1EHGBR\/eDGOTu7m5V\/ggPckfU1tZmiB++ITy8MRd3akrX\/A\/8\/oKs+MUfpP\/coBz455qiFs8bcykl4aK0JUcj92UN52vMHREk0uzLVgS0VYwcE37r1NESTjEJFyrS6VR4r9eW3ZuEhT5tpQ3OKwwE9O2646cvqKQ\/DV++Rlpn5reQScIpJWFNqsj1i3SS2fy4XpXU7Y5wXnnW3zlTZKJf5hP2ii7fiyMCIGFkYVuz86iaPggZN+4mVJapf++4YVRmWSThlJKwOyoCVvHOnTtl4cKFvnXe7Y5goU\/fELJBghHQhPzGu2fk9SNnMysFMaO689TRIqvm3JRgBHIvLdVxwtlI2Gs1jTC0xaQwTI4dBpbs034EcPvujSNn5fjp8\/LKf+yVwTGfv2TSOocxiBo17y5a1GUF3Rv2r\/7TGdq09yIv9GkjCcdJeThXIhAmAn+acLv8ZeQYNcRfRo4e8vPFzy5+V8pz2R\/\/kLf5ZX88VUr3ntt++K\/f8fxumC8aIWFd3j7bwvyEqIUJDPsmAkSgOARgbb9\/5kLOxnCT5HtgpUfxlBJjHeT8IifhICfPvogAESACcUeAJBx3CXL+RIAIxBoBkrBl4kMI3yuvvCK\/+93vZN68eTJp0iTLZsjpJBWBgYEBefnll+XcuXMyfvx4VaHm8ssvT+pyrVkXSdgaUVycyM9\/\/nMZPXq0SnT\/4x\/\/WB544AEpKyuzbJacThIRwBX\/I0eOyNe\/\/vUkLs\/aNZGEIxKNs7CpLlTqzKf8wgsvqMsrzz33nNoEsEQQuldfX++p8nREy+AwMUTAq+7t3r1bfvazn8lll10m3\/jGN+TOO++UYcOGxXDF8ZoySTgCeelr2hhKV4vGZ\/r2njPJEJINgXjHjh1LEo5ANkkfwo\/ugazLy8tVybFNmzbJfffdJ+PGjUs6RMbXRxIOWQRQ7C1btsjdd98tK1askLVr1ypXg04whOxu8AO3tLRIe3u7HDp0SL7whS8IqkbDEkaaz6uuuirkWbL7JCLgV\/fef\/99pXtXX321ImEYA\/g\/Mj7hIkASDhffTO+wSJYsWTKEhPv6+qS1tTVTJRo\/Q+nhkqioqFBkfc8990Q0Qw6TVAS86h7+2MM4gO6NGTNG5syZI8OHD08qLNasiyQckSi8bgT4hfkQgSARoO4FiWbwfZGEg8c0a4\/ZNsK+fftUsnmnO0If2kU0LQ6TAgSoe3YLmSQckXzcGyHXwRzD0SISSIqGoe7ZLWyScETycW8EDKtD1KqqqjJRExFNh8OkCAE\/mG8hAAAEDUlEQVTqnt3CJgnbLR\/OjggQgYQjQBJOuIC5PCJABOxGgCRst3w4OyJABBKOAEk44QLm8ogAEbAbAZKw3fLh7IgAEUg4AiThhAuYyyMCRMBuBEjCdsuHsyMCRCDhCJCEEy5gLo8IEAG7ESAJ2y0fzo4IEIGEI0ASTriAuTwiQATsRoAkbLd8ODsiQAQSjgBJOOECtml5yBa3dOlS6enpGTKtBQsWqLzKcX6Qn2Hnzp3S3Nys1lhbW6sS8uPR625oaFAlrNwPvt+wYYPMnz+fpazirARFzp0kXCRwbOYfAU1GToLy34t9LZwkiix4fkkYK9IkvnDhQvsWyBmFigBJOFR42bkTgXwkrMs9HT9+XFV0uPnmm6WpqUn6+\/tl2rRp0tnZqazE\/fv3y9y5cwWZ56ZPn65qosGChAUKaxqWJvrSVUucxVS1xY0+Nm\/erKa2d+9eVcYHeZ1BoGvWrMl8h+KreFD3D9\/jAcG6LVr0d+zYMWX5Zluj0xLGeHps9Occe+PGjTJz5kxVUYVPehAgCadH1sZXms0doQn21VdflRdffFGRLR5dc0\/X4wOpOskW7UCIIONcJFxXV5eVQNH\/4sWLVfrQ0aNHZwgcn4OEMYdTp06pQqyPPvqoPP3007J8+fLMZ+vWrRviNkAbjIU\/ALlcLuhbV9TWgnC2w2f4g4FHuzGMC4wTiAQBknAkMHMQIODFEobFeeLEiYwVrJED6T700EOqAKW2irXFm4uEUSy1ra1tCPiwhkGYmmy1+wDWLaxZbUE7G2my1Jaz03+NNa1cuVIaGxuVBVvIEtY+YTcBo29Y1LCU4+4fp7b7Q4Ak7A8vvl0CAn5IGFao2+IESWnyhGvCCwlnI1VnP15IWJMjlq4tXg1DMSScy+IlCZegXDFuShKOsfDiNnWvJIz34OOFbxj\/a679xahWjYMrWIpOd8SiRYsyh2GzZs3KuClAmNrtUF1dnXln4sSJWS1htztCV8dGW0Qv\/Pa3v73kD4Nu43ZH5IqOgLWdy+VAd0TcNDqY+ZKEg8GRvXhAwCsJwzpFtIDXgzmQsrNUlD6wc36O6TkP5rK5I\/ShnnZhrFq1KuOfdR72uZfqtGDzuSPuuece5U7p7e3NdKF94liz063hAU6+khAESMIJEWQal5GPGIPEI4o4X4aoBSmxePVFEo6XvDhbBwJRkPDp06eVa2TChAmZMLZsQijFn+v2K1PI6UKAJJwueXO1RIAIWIYASdgygXA6RIAIpAsBknC65M3VEgEiYBkCJGHLBMLpEAEikC4ESMLpkjdXSwSIgGUIkIQtEwinQwSIQLoQIAmnS95cLREgApYhQBK2TCCcDhEgAulCgCScLnlztUSACFiGAEnYMoFwOkSACKQLgf8HS2PDFwkNG24AAAAASUVORK5CYII=","height":213,"width":353}}
%---
%[output:6726d4a9]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:71f55707]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"5","name":"Afht0","rows":2,"type":"double","value":[["0","0.000010000000000"],["-1.421223033756867","-0.000188495559215"]]}}
%---
%[output:5cc7402a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.018661060362323"],["3.911916400415777"]]}}
%---
%[output:502fdf2e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-17.765287921960841","0.997643805509808"]]}}
%---
%[output:5674bd90]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.215470420991426","41.436377683116810"]]}}
%---
%[output:022e1431]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"5","name":"Afht1","rows":2,"type":"double","value":[["0","0.000010000000000"],["-1.421223033756867","-0.000188495559215"]]}}
%---
%[output:84906125]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.018661060362323"],["3.911916400415777"]]}}
%---
%[output:1de224ac]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-17.765287921960841","0.997643805509808"]]}}
%---
%[output:2b5c76de]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.215470420991426","41.436377683116810"]]}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAWEAAADVCAYAAACG00EAAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ1wH8V1X0iGWjQ1IEMCRsiygwxJ0yFAIMLgGtchTDojSNMkttQhru1StzHOZCLVskRmMm6wZas2Y8BJowFF40ximU4CsTxtahiRjwHhEL5UIAQTy0II5cOWYmzATErqzjt31dXqPnbv3u7trt7NZGL033373u\/t+927d3u7p508efIko4sQIAQIAUKgFAROIxIuBXcalBAgBAiBCAEiYZoIhAAhQAiUiACRcIngYwz98ssvsxUrVrD6+nrW0tKCIXKSjPHxcbZq1SpWXV3NNm\/ezCoqKtDHEAVu2bKF7d27l3V3d7Pa2lqjY8nCT5w4wdavX89mz55tBEuTxuzfv581NjZGQ6xevVpL\/zIxlzHh823ZsmVs6dKlJiFzRjaRsDOuyKdIGSQMY+7bt4\/ddttt+ZRO6SUTgsmxZDU4ke3atYvV1dVl2qar2\/3338\/mzJmjJDtzcKEBv3kMDw+zrq4uVllZqdOduUTCoDjoA77IY4uW4Y40JhJ2xBG+qGGa9EVCAExMZvky5jrBr4sDEHBraytTJXid+RAaCeveDHWwcrEtkXBGVtbZ2Rm1gEdU8RGZk8X1118fBRZc7e3tkx6hoA3vD+UC\/jjPAxj6Hj9+PHr8luXLavEg5n\/nwSyTAW8Hj6QwNn805e1GR0cnPbLK5Qb4ER7JeVYF\/83LEevWrYuy34GBgUjGZZddNilb4WQAv8m2iuUSEZckXO+66y52xx13TBmL6xOnAx8f8ISLYyD6RXxsF8fmOEAGzMs6\/G\/yWEk6JP39wIEDE6UCrheMkaRL3JSUdeHzifuL2wz\/HUf0Sf2hvMTn8nnnnTeBt4iZjKuIG59XV155ZTRn4IIMVrR50aJF0d+PHj06MV\/k+SjqrHuDc5FYdXQiEk5AS35EkzMZTiR8ssb9zmubs2bNioiMBzifZDDpYcKOjY2lZnxpskF9MVsUSZiTiTypefCD7jfccMOkmm8aCQOxjoyMaOkK+txzzz0TNzAVXDlusm0yycu6iDjBDQJuJiCL+0i0G+qNYubLfbB27dqJG2nc7\/xmImOqoxvMgzRd5HJC1o0SiFS8cWb1l3GT57LsIxEH8aYszgc+l2FsWV+4iUG9mt+05fkuzxHb7yF0CNNEWyLhGFTTsiJOpHG1S06Gt95665SXWWkBnTbpsh41kzJhMbNIexTOCvCkoEt6ESjq84UvfCEiB54Zgy3izQj+LmOtUo6QszrIePlYYl00jujEl37iYy\/oAkQhZoBixi5nl0nZWpxucDNMu5HCC8i0R\/C438S\/8RtOUk1YxkGe8lk3RmgvZ8M8E4+7KcvjyXP44YcfnlSa4VjyG2DWnDdBhGXKJBKOQT8uwGSyuvvuuye9xRd\/lx\/b+RD8MU7O8NJIOCsrUCHhtBcv2CQMtvEbDpBPc3Mz48Gli2tSJsyz2yuuuGIiK4+78cWRMC8PiW4H4oUXZjIJy4\/M0IeTdFImHKdbEgkn6SKvCoi7iYq23XTTTamZcFY9OouE+c0IbnYyznEkLI+XRMJy6PHSGZFwmbcER8Y2kQmLpskBHFImDHZygli4cCE7ePDgRClCF1eZhGXc4rJunUxY9ElWtsiJJelGmqabSiacNvXLzITnz58\/6amOP83wJYsYmbBsO5GwI0RYtho62UFWkPGacNLEzcp25cxCrKHJRBWX9SQRBGSochbF63W8xieXI+JKCrKvxEdyec2qCq5ZtXR4CQT1SHgaEV8+5qkJy\/XntEfiuNqoXOdP0k0m0qxSiYhp1tOKbk1Y9mGaTzgJgz7w\/oKXEtLKEXlqwuLKkax4KJsbsMenckQKoipv8aHG+dWvfjWSkrY6QlxJoJMJc\/V0V0ck1TDl1RFi5gr\/hkdyWLERtzqCr3jguKSt6OBt4t7Uq+DKV6LIYz399NNRPREu\/tZ95syZESnDxV\/GgW7cN0mrI6A91y8uS09bFQB9dXTjxAcvqTiB8RdW3Mdpy9fSVjeoZI4qqyM45vKNXFzFAfMYkgk+P5JeKot95DkFL+\/kUo\/oI1odgU3zMfJg8sIV94WX7Bx5GZQF9ZSGSKuzKgmgRsYR0F1vKma6uh88GDfG4wHili6mmaPrN4+hiVS3nglzgJM+rYTfe3p6rHwiW8R5RMJF0DPTN27ZoLg8LmtUmHvwIrGMT6azdPPpd3kJJugur4pJs2e63QytkjA8Em3YsCHCP+n7fHgUGhoa0vr2vYwJSiRcBurpY8qP3GK5QUVbn\/eOULHPZhu5fCZ+rJSmB\/ch7R1hyFtAXDU1NRHJJpUjxHqhbhAZUpvEEgKEACFgDAFrmTA8ouzcuZPdfvvt0VvtOBLmmciCBQuir5bo8dCY30kwIUAIOIKAFRIGct24cSNbvnx5tD1h2os5EReZlMXf5s2b5wiEpAYhQAj4hkBfXx+bO3euE2pbIeG4L4\/A+qx9TzkJNzQ0TNn+D0h4cHDQCRAxlQjVLsAoVNtCtYt8hhnZybKskLA8fFImDEX5pqYm1tbWFmXMUI6AtnH7ioY68UO1iwLaTkBjjxLqfHTJrtJJWCZeMWtO+xjAJRAxJ\/6hQ4eceUzCtAtkhWpbqHaF7DOX+KMUEsYIbpdAxLCHy6CAxkTTjizymR2cMUdxiT+IhDE9iyCLAhoBRMsiyGeWAUcYjkg4MBARzJkQQQGNiaYdWeQzOzhjjkIkjICmSyAimEMkjAmiZVlEwpYBRxjOJf6gcgSCQzFFUEBjomlHFvnMDs6YoxAJI6DpEogI5lAmjAmiZVlEwpYBRxjOJf6gTBjBoZgiKKAx0bQji3xmB2fMUYiEEdB0CUQEcygTxgTRsiwiYcuAIwznEn9QJozgUEwRFNCYaNqRRT6zgzPmKETCCGi6BCKCOZQJY4JoWRaRsGXAEYZziT8yM+G4jbKzMIAjiR588MGsZoV+dwnEQoZInSmgMdG0I4t8ZgdnzFFc4g8lEhY31ckCAvZ+2LRpU3REjMnLJRAx7aSAxkTTjizymR2cMUdxiT8ySRjTcExZLoGIaRcFNCaadmSRz+zgjDmKS\/yRScK8HAHHXPMjzzHByCvLJRDz2hDXjwIaE007sshndnDGHMUl\/sgkYTBcPrSvvb09On6ozMslEDFxoIDGRNOOLPKZHZwxR3GJP5RIWDRePIgTXsDFbbiOCVaSLJdAxLSXAhoTTTuyyGd2cMYcxSX+0CZhDoS8aiLrqCJMAEGWSyBi2kYBjYmmHVnkMzs4Y47iEn\/kJmEREL4iYtu2bayyshITq0RZLoGIaTAFNCaadmSRz+zgjDmKS\/yRm4TlTLi+vt7qizuXQMScHBTQmGjakUU+s4Mz5igu8YcWCcunJqedAYcJWJwsl0DEtJUCGhNNO7LIZ3ZwxhzFJf7IJOG4L+Z27do15Qh6TIBUZLkEooq+qm0ooFWRcqcd+cwdX6hq4hJ\/KJNwXV0da2lpUbXReDuXQMQ0lgIaE007sshndnDGHMUl\/lAiYfpsGdP96bIooO1hjTUS+QwLSTtyen72a7am50U2fudiOwNmjKJEwqtWrWIDAwPKCtMGPspQTWlIAZ0fu7J6ks\/KQj7fuN6RcD4zzfdy6XEC01oKaEw07cgin9nBGWsUImEkJImEkYC0KCZUsgrVLpgaIdpGJIwU9ETCSEBaFBNiQIdKVHxahOizLfuG2JZ9h\/ypCVuMUa2hiIS14HKicYgBTSTsxNTSUoJIWAuu5MZEwkhAWhRDJGwRbKShQvQZkTDS5CASRgLSopgQA5oyYYsTCGkoImENIGHbTLjiPhIhEtYA0pGmRMKOOEJDjRB95j0Jnzhxgq1fv57t3buXwaY9K1euZNu3b2fYO6jt37+fNTY2sqQtMomENSLJkaYhBjRlwo5MLg014EMNWCHhzccaom2cgBcsWMDmzJnDenp6op3Tent7WX9\/P9ouarBfxYYNG6KhYZMgyoQ1ZpjDTYmEHXZOgmoh+uymrz3DHj141E8SBnLknzCPjY1NkPDIyEh0wjJWNgxliJqaGjY0NETlCP\/iNlHjEAOaMmH\/JqjXJAxwA0GOjo6ym2++me3ZsycqR6xZsyYqTWBs8APbZe7cuZPdfvvt7O677yYS9m+OEwmTz5xG4MN3PM6Gx9\/2MxPmyPJ6Lf9vrIM\/odyxceNGtnz5clZbWxsRftqLOfitr6\/PaYfrKgdPFVVVVbrdvGgfqm2h2gWTKjTbFtcvZcc+fopXvKwJm450edN4Pl7cyzl6MWfaG\/jyqRyBj6lpiaH57NFfHmU3ff0ZImHViZOVCQ8ODqqK8qZdaJNeBD5U20K1K8R6N1+e5m0mHHfKhsxumEceEQl7c+9QUjRUsgrVrtBIGOrAt\/W8GK2MOP2tI+zINz6jNG9NN8rcT1hWgK9cWLp06cRP999\/f7SSAV7Mwb9hudpdd91lVHcqRxiF14jwUMkqVLtCI2GxFFEx8G32Wl+XkXmuK1SLhMUlavDijF\/ikfewdA2Wq3V3d+vqotWeSFgLLicah0pWodoVEglDFgy1YPj\/6soZbLz3n9nwEz9wIi60SBg0hky4s7OT8cM+5S\/bKBMu5lcK6GL4ldGbfFYG6upjimUI6NX7+cvZ5z5+BXPlnZI2CYMR4ioGsQYsZsSVlZXqKOVoSZlwDtBK7hIqWYVqVwiZsJgBgz2QBT\/75WuYS\/yRi4RLjuVoeJdAxMSDAhoTTTuyyGd2cNYdBWrAt+1+MSpBcAKGLBiI2CX+IBLW9azh9hTQhgE2IJ58ZgDUgiL5p8lcDBAvJ2DXkjhtEpa\/luNGwgnLXV1dzHQZgo\/n0p2s4HyZ1J0CGhNNO7LIZ3ZwzhoFMl44tgh2SBPJ9zsr\/4z96ez3TOruEn9okTBfJ7x27Vr2yCOPRJ8Xwye2sLUl7KwmLlvLAqzo7y6BWNQWsT8FNCaadmSRz+zgnDSKXPcVCRjqv3GXS\/yhTcJ8F7UHHngg2ukMiNfmCznKhMud8EVGD5WsQrULfO2qbVDv7dh3KPrwQr7k0kNQJMz3E4YVEYsWLYo2Xb\/vvvui3dSGh4epHFGEof6vr6uTHsE0ZwO6qG3ks6IIZveHbPdfHhpi33niV7GNgXibPlbDbqm7IFuYYy\/2tTJhbt2OHTvYjTfeyODDDCBi2MYSNnevqKhQAgCjkUuPExj2cBkU0Jho2pFFPsPHGUj37keG2YHfvBmb7cKIQLzXvv9s1nDVBey6i8\/WUsIl\/shFwlrWGmrsEoiYJlJAY6JpRxb5rBjOQLi\/+PWbbMcPhxMJl48AxPvlv5zHrq45KyLhvJdL\/EEknNeLhvpRQBsC1qBY8pkauHy97rf2j7InDr2eSbg82132kfNZ49UXFCJdWUNvSVhl7whaoqY2IZNaUUAXw6+M3uSzyahzsv3hS+Pse0\/\/RolsOeFWnzODbf3MJWz+e8806krvSDhps3URJdt1YZdAxJwtFNCYaNqRNV19xsm246FDbHjsbWWyFQm349Pz2ZHj\/61d0y3qWZf4Q6sckZQJFwUkT3+XQMyjP2XCmKiVKytkEn5s4GV2wYVVbHvfK2zoyAktohXJ9i8urWRfXDKnXEcJo7vEH1ok7AyCji0xwcQl5IAO1Taf7eLZ7J6B37KHfz7Ghn\/39sReCzrzGl6SQSnh01e+j10\/vxK1fqujh2pbr0hY5TQNMJw+W1Z1f3o7nwM6C4FQbXPZLvllWF6SFbPaj847i93y0dmRu4usUMiaLyZ\/94qETQJRRLZLIBaxQ+7rckAXtTNU28qyixPs86NvsH9\/7jB7dfzt3JmsSKiQ0X7phjls3rlnsj+8\/is2d+7coq53rr9L\/EHlCMemR1kBbQOGUG0zYRcn2CdfOcb6fjFWmGAnkWzlDLZiwYXsyuqZmdmsCdtszLWsMbwnYTg9o7W1dcLO9vZ2q5v3wMAugZjlcJ3fQ530gEGotunYxcn1D\/9zkt376Ah7\/rU3oukRtweCzrwRywXzzjuTrbn+IvZH7z49k2SzxtCxLUuWS7+7xB\/amTAcbwTbWfJtK3nNuK6uLjro09blEoiYNoc66UMn4XeddWrPgpOMsW\/vH2U\/PfQ6GrmKBFtzbgX722tms3Pfc0ZhglWZt6HOR5f4Q4uE6WMNlWlbrE2ok95HEuZZK+h+7O132M7HR9lLv34TnVwjkj1nBvvQhX\/Cbr3uQvau00+zQrAqMzXU+egtCYPTKBNWmbr524Q66V0iYSBXfuruf75whA2MHEepuYpe56sGgFwvqpwRfXZ70Tmn9jrwaUVBqPPRaxKGSUQ14fwkm9Uz1ElvmoTFrPWxg0dZ\/8Gj7JWxE4VWC8T5SiRXKA189srz2WlvHmYXXXSRV+SaNQ\/576HOR+9JWNWBJtu5BCKmnaFOel0SFkkV\/g3LsJ57DT9j5b4TyXXxJZXsry5\/Lzv9tFNlgazslXyGGQF2ZLnEH1o1YTvwqI3iEohqGqu1CjWggUhhzSl\/gXXkjd9HZ4HxGmuRjwiSkBWJ9QMX\/HG0LOvMM96VSapqnvr\/VqH6TPfGqYtbme1d4g8i4TJnQszYvgU0z1hPnmSs979+G+0LW\/SjgTSXiMQKhzdCrfWsindPECuv9dp0q28+08EmVNu8JWG+HK26utr6SRryxHEJRJ1JndW27EkvlgF+dGA8Wmpli1ThBdb7Zp7Bll9zIeOFAB9eYpXts6w5VeT3UG1ziT+0M+G4bS3pY40i03xyX+xJL5Jq3y\/G2ZOvmCVVsX4KKwOqZ81gjVddwKrOmcFeffVVdu1ltXhgOSIJ22eOmBWpEaptXpNw3ASB1RK7d++mgz4Roidr0ouk+h\/PHzH6soqbIy+3WnLpLPaROac+eRVJN8v8LNuy+rv6e6h2EQnbmXEomTCcvtzd3c1qa5OzHDGDTttxTc60k9q6dCcr6ipOrAcPv8V2\/uQgG\/\/9qRqniZdVUzPVCrbw4rPZNfMmH5RoogwQKlmFaheRcNHIVuuvRcK8Jgyi+WfLKsOcOHGCbdy4kS1fvjwiasic+\/v7Y+vK8El0T09PZs3ZFxIGgj124h3W1f8aO\/jbt4wQq5ipVs+qYB+7tJJd8X+bs+hmqyr+zNsmVLIK1S4i4bwzXa+fFgnriU5uDdnupk2b2LZt25h8Jh0Q9NDQUOY+FK6R8E9e\/h3b+tAQCsnKj\/+L51eyq+eeNQlQE5kqln+T5IRKVqHaRSRsOiJOyS+FhNMyYfgsurOzc8L6Xbt2MdgcSL7KJOFXxt9ma3te1N75ihPnwovPYXXzzmLw\/3KmSgFtZ+JjjkI+w0TTjqwy+UO20CoJi6d0xJErlC3Wr1\/PFixYEG2NCaWJ5ubm2HozgAhXX1+fca+NHnuHdT35Ovv+C8dTx5o981Qt98oLZ7C\/u+psBvuw8L+pKjkyMsKqqqpUm3vVLlTbQrULJldoti1ZsmQiZgYHB52IH6skzC3mZAxbX8ZlubydTMoiYjbuZFDPvenrz8SeucXP1Lrzs5ewnw0dYw1XnY\/iUMqqUGC0KoR8ZhVulMFs8IeqolokjLWVZRq5iorzdg0NDVPI2iSIj\/7yaES+8gXE+\/1\/\/HC0p4CpmiwFtOrUdacd+cwdX6hqYpI\/VHXg7ZRIOO4DDXmg+vr6xBUNMnmDvHXr1rGOjo5Jy9rkdlCOgBpx3EoMUyB++I7HJ2W+QLYdn5rPPv7BWbrY5mpPAZ0LtlI7kc9KhT\/X4Kb4I48ySiQslhGamppYW1tb6prgOEVkIuc14TiCXrFiBRsdHWVp64+xQfzB80fY33zzuQnVgXx7P3+5sYw3yVkU0Hmmcbl9yGfl4p9ndGz+yKODViZcZABTfbFAhLrvln2Hoh29+LX1r+ezlddeaEr1VLkU0KXAXmhQ8lkh+ErpjMUfGMpnZsL8Jdrhw4fZ1q1bo\/LAwMDAlLHTvoLDUFSWgQGi\/OKtrOxXtI0C2sRsMSuTfGYWXxPSMfgDS69MEsYaCFtOURCBgG8T1vq6QMCAEQU09kwxL498Zh5j7BGK8gemPtOShIGAOx4aYrue+FWEpSsETCSMObXtySIStoc11kjekrD4sYUMhk\/lCHEJmksETCSMFWJ25RAJ28UbYzRvSTjJeKgTL1q0KPXDCwzgRBl5QYQsGJahuZYBc9sooLFninl55DPzGGOPkJc\/sPUAeSjliLQNeUwoDTLzgiiuA4YlaNddPHkLR1P6qsqlgFZFyp125DN3fKGqSV7+UJWv0w6FhH3Z1B2Woa3peTHCZ+3iarah\/v06WFlpSwFtBWbUQchnqHBaEeYtCafVhJN2OzOFaB4QK7\/0w4kyxLNfvsaUaoXkUkAXgq+UzuSzUmAvNGge\/ig0YEpnlEzYlHJpcnVB\/OK\/vcS+tX80EuliGYLbSgFdxmwqNib5rBh+ZfTW5Q+TOmqTsPyZcdrewCYV1wFRfhnnahYMeFFAm5w1ZmSTz8zgalKqDn+Y1ANka5Fw0u5nqqdhYBqjA+I\/fe8A63rstWh4IGBTO6Bh2EcBjYGiXRnkM7t4Y4ymwx8Y46XJ0CJhrK0sMYzSAdGHWjCVIzBmRTkyiITLwb3IqDr8UWQclb5aJJy0v2\/alpMqSuRpowqi+GHG1xo+gLb5eh6dVfpQQKug5FYb8plb\/lDRRpU\/VGQVbaNFwjAYlB5aW1sZXw0BBNzY2Mja29ujI4lsXaog3vS1Z6Kz4KAE4XItmDJhWzMHfxwiYXxMTUtU5Q\/TeoB8bRKGTkl7A9tQmI+hCiIvRexYdilrvPoCmyrmGosCOhdspXYin5UKf67BVfkjl3DNTrlIWHMMI81VQORZMCjg+gs5yoSNTBMrQomErcCMOogKf6AOmCJMi4STXszZUlYcRwVEn17IEQmXMYtwxiQSxsHRphQV\/rCljxYJg1KwWU9NTY3V+m8cGFkgimuDfXghRyRsa8rjj0MkjI+paYlZ\/GF6fFG+Fgn7tJWlj6UIcAwFtM3pjzMW+QwHR5tSvCVhmyBljZUGopgFX\/f+s1nvmsuzxDnzOwW0M65QVoR8pgyVMw2JhBFckQaiuDa45ca5rOXGGoQR7YiggLaDM+Yo5DNMNO3I8oqEfTzos\/m7B9g3+099puzyZj1x040C2k4QYo5CPsNE044sr0jYDiT6o6SB6NsHGqL1FND6c6HsHuSzsj2gP77XJOz6LmpiPbjhqvMZrIzw6aKA9slbp3Qln\/nnM29J2Idd1MTTM3wrRVBA+xfM5DM\/feYtCfuwi5qvS9P4VKasyr+gJp\/55zNvSdiHXdT4QZ6+bNgjT18KaP8Cmnzmn8+8JWGA2vVd1Pinys031LC2T8z1bnZQQHvnMqoJ++ey3Ke1mzBV64s5roCru6iJ64N92bCHMmET09quTLpx2sUbYzSvM2EMANJkiAR\/2WWXsa6uLlZZWTmlSxyIcJw9vJiDi0jYtKf05YdKVqHaFfJLRyLhhPiFmvPGjRvZ8uXLWW1tbVT66O\/vZ5s3b2YVFRWTesWB6PP6YHoxp0\/qrvQgEnbFE+p6EAkrYgVZ8aZNm9i2bdumZMMyiD7vFyHCQQGtODkcakY+c8gZiqoQCSsCpZMJiyTs234RRMKKE8LRZkTCjjomRS0i4QyfiVtm8rPs5C4yiFv2DbEt+w55XQ8OuQYXsm1EwkTCRRDItTqiyIA6fTkZt7S0sLq6uik1YfhDX19f9Pe1vb9h\/a+ciP791Fp\/dk2T8RgZGWFVVVU6MHnTNlTbQrULJlZoti1ZsmQiXgYHB52IHW0S5qcry9qnrWTIa2nSZ9IgT86EQ3gpF3K2GLJtlAnnjfDy+nlbjuCZ6bJly4wcbyR\/Fg0v5tatW8c6Ojqi1RLiJYIYyku5kIkqZNuIhMsj07wje03CTU1NrK2tbQop5gVD7qf6IYgIoviRho+b9ogYUEBjzSR7cshn9rDGGslbEgYAYMXC0NAQgzptmVdSJkwkXKZX0scOlaxCtSvkpxdvSdjVgz5DWRkR8qQP2TYiYXdv\/EmaeUvCLkEtghjKS7mQiSpk24iEXWIGNV2IhNVwSm0lgsi3r\/TtZOU4AymgESaHZRHkM8uAIwznNQnzZWN79+5l9fX1bOXKlWz79u2xnxYjYJUogoMoroz41OXvY\/fd8kGTwxqXTQFtHGL0Achn6JAaF+gtCYvrdufMmcN6enqizXV6e3sTN9oxhSYHUVwZAefJwblyPl8U0P55j3zmn8+8JWFxHe\/Y2NgECcNXNUkb7ZhyDwfR9zPlZHwooE3NGHNyyWfmsDUl2VsSBkC2bNnCRkdH2c0338z27NkTlSPWrFkTlSZsLluLI+HxOxeb8pk1uRTQ1qBGG4h8hgalNUFekzCgJH+63N7ebuQLujSPcBBDWhkB9lJAW4tDtIHIZ2hQWhPkPQlbQyplIA6i7wd7UjnChdlUTAci4WL4ldGbSBgBdQ4iP9gzhOVplAkjTIwSRBAJlwB6wSGJhAsCCN0BxB89+XMGmTBcPm\/kLsJBAY0wOSyLIJ9ZBhxhOK9JWFwnzLFYvXq11ZdycSQcwvI0yoQRoqsEEUTCJYBecEhvSZgT8OzZsydIl\/8NMIk7kLMgVondAcTV\/\/rIxGkavm\/cww2lgDY1Y8zJJZ+Zw9aUZG9JWN7vlwOUdiCnSRA7vvs4+\/tv\/zwawtcj7mV8KKBNzRhzcsln5rA1JdlbEgZAYHka\/1KOH0MPa4dramqsLlMDED\/U9D326MGjrLpyRkTCIVwU0P55kXzmn8+8JeG0rSxFN8BRRw8++KBRzxAJG4XXiPBQySpUu0J+R+EtCRuJzJxCAcSjn+yKeoeyPC3kSR+ybUTCOYO4xG5EwgjgiyQMm\/bA6ogQLgpo\/7xIPvPPZ96TMBxx1NraOoF8GZ8tV1\/9CfbGdesiHUJZIxxythiybUTCRMJFENA+8h5ewsHLua6uLlZZWcl4nbiurs7qWmGRhENZnhYyUYVsG5FwEQoqp6+3mbDZuttkAAAJM0lEQVRLS9Sqrr+FvXXFysiDoSxPC5moQraNSLgcIi0yqrckDEa7kgmf\/+k72O+rryUSLjITLfcNlaxCtSvkG6fXJAyOcaEm\/N7PfYO9c+4lQa0RDnnSh2wbkbDluznCcN6TMAIGhUUQCReG0LqAUMkqVLtCvnESCSOEf2hbWHJIKKARJodlEeQzy4AjDEckjAAiJ+GQ1giHnHmEbBuRMEJAWxZBJIwAOCfhkNYIh0xUIdtGJIwQ0JZFEAkjAM5JOKQ1wiETVci2EQkjBLRlEUTCCIATCSOAaFlEqGQVql0h3zinJQnLJzTv2rWLwVd28gV7E69YsYKNjo5GP8GObPzrPLEtkbBlBkUYLlSyCtUuImGESa8gQvuzZQWZU5rAl3YbNmxgX\/nKV6JPnYGQ4aOPOHKN2684bkxOwiF9LRfypA\/ZNiLhPKxQbp9pmQmLkPP9JlpaWqZkw\/AhyNDQUOY+FJyEx+9cXK43kUengEYG1II48pkFkJGHmPYkDCWHdevWsY6ODlZbWzsJXsiQOzs7J\/6WVLYAEg7pRA1uMAU0crRZEEc+swAy8hDTmoT5waALFiyYchyS\/BuUJpqbm1l3d\/cUsgYSDmkzdyJh5CizKI5I2CLYSENNWxKOO605DdM0wgYSfveRl9hPN9yI5BY3xIyMjLCqqio3lEHWIlTbQrUL3B+abUuWLJmY1YODg8gzPJ84Ky\/mQLU0Qk1SnfdpaGiYUjsGEv6HP69imz45uZyRDwZ3elFW5Y4vVDUhn6ki5U67aZcJqxKwvF9x2ioKIOHQvpaDKerS5MAOmVBtC9WukOejSz6zkgnLa395cMNLt\/nz57OmpibW1tYW1X3FtrNnz46tB0N\/IOEzn\/4mO2P4MWyuIHmEACEwDRCYduWIaeBTMpEQIAQIAW0ErGTC2lpRB0KAECAEpgkCRMLTxNFkJiFACLiJAJGwm34hrQgBQmCaIEAkPE0cTWYSAoSAmwgQCbvpF9KKECAEpgkC3pGwuIRt9erVmRv9uOpHFTv4+uq9e\/dGZtTX17PNmzeziooKV82atMRQxT9p+4i4ZqSKz0BnsV3SVqwu2aZql7gdrYpvXbIxThfYp6ampmbK9gm29faKhMXd12Byr1+\/nsXtQWEbRN3xVO2AHeXgWrp0aa4vDnX1Ktpe1S4+Dr\/JPPXUU4nrwYvqhNVf1TZ5h0DVXQGx9NSVo2qXeLOEz+p9jT2OD98orL29nUhYZ9LARNi0aRPbtm1btC8xTPD+\/n7ns0PZxrx2uG6vrl1gz3PPPcdeeOGF2B31dOaG6baqtkG2+OMf\/9ibJzQdu8Q9wOHfcMF2tD5d4v41oDdlwprekzd8T\/usWVO01eZ57XB94uvYxYMfHmvBrrhtTa06JWMwVdvgxnL48GHW19fHBgYGEk+GccU2VbtUM2ZX7FLRg8oRKihJbeRM0FcSzmOHD7bq2AUBsGjRIjZr1qzEvaVzTBFjXVRtg3b33HPPRHnF9Runql0ALCdiuLkk7fNtzAEGBBMJ5wBV9a6dQ7TVLrp2pO2rbFVxpGxRfGT35cWcqs\/kGrDrN09Vu2Q7XL+5qMQFkbAKSlIb1fpVDtFWu+jY4XoQi8Cp2iWfngIy0jZrsuqchMFUbVMlNRdsAh1U7ZJJ16d5mYQ1kXCOWRhKXUrVDt8muqpdMnEnHXWVY4oY66Jqm7gdK19FADcYV19gqdoVlwnDieiuL5lMmxBEwjnDRXVNY07x1rol2SFOjLiM0fW1wip2+UjCPGtcsWIFA\/IR18nKwSxi4Lq\/dOyCUktra2vkPh\/WP2cFM5FwFkL0OyFACBAC0wABrz7WmAb+IBMJAUJgmiFAJDzNHE7mEgKEgFsIEAm75Q\/ShhAgBKYZAkTC08zhZC4hQAi4hQCRsFv+IG0IAUJgmiFAJDzNHO6yueLWnTpbJbr49Rbf9tHkRyjiUrgQPiN2eW6a1I1I2CS6HsnW2aFNp60OBHk\/TnGVhHt6eox\/zMBvXA0NDayurk4HbmrrCAJEwo44omw1dIhVp62OXfInv6p9iYTXMyJh1dniXjsiYfd8Ykwj+aQO\/sgvnpjAv\/AaGRlh\/OswUCitLchdtWpVtHUjXGmPxuJOXGJbUYekR3jx8VtsAyR8\/Pjx6H9wConcX5QNY\/KNvPm+CTNnzoz6cb3FLxXBbujf1dUV7WGdpIPsNPmGIuuY9sWZfFNJu+lRJmwsXKwJJhK2BnX5A4k7fMnBKwY6aAonJ\/DsSt7pLK4tP+EkbVc0PiZvK+8Ol1aOkE+sENvee++9EYl2d3ez2traaH9ivq8BjNnU1MTa2tqi38R+Y2Nj0Y1m7dq1E6criL\/DMVKAw\/DwcETCcMHNBvaBgEf\/NH3jSLizs3MS0SftvUAkXH6s2NSASNgm2iWPJe91K6qTlm3JZCm2hYxZPO0EZCZ9ky8TdBwpi6c3iPqlEZ4OaYHuu3fvjkgVSFjePChtt7ADBw4wsc6bloXGkbBIumk3Kx17KBMuOagQhicSRgDRJxHiJiziY7tMwuIjOTw6w8VPvxDbQgmisbFxCgRxqxtkItUh4bSbRBpp8ayeH5a6cOFCduzYsVgSlvWBvqLODz\/88MQGNqLBceeUxZEw9OG7qYm7rUGGLl5Ewj5FVHFdiYSLY+itBPGxvbe3d+K8PshuxQwxrRwRlwknAVJGJgw3CTG7lssRRTLhNMdTJuxtWFhXnEjYOuTlDRiXYQ0NDUXZmVxigFrp1q1bo9on9BNrrmk1YV67XbZs2ZRTbDFrwiKhP\/DAAxGoPMuUM\/Xm5uaoXsz39+U13rhyhE5NmL+k4zjJ5ROxdCFjKN4AofYsl4Z4yYTXpeH3uL17qRxRXjxhjUwkjIWkB3Lk1RHiG3pOKOedd170qA4vu+BFElw7duyI\/pu\/kJLbQhtxdUTahxZJqyPkR39YiSBf4soE+E1czZBEwvB3eLnGV03ACzqwBUorcMVtKM9LMVCuAbvgKSFudQT0TzoyPSkThhuAfACoXJoQMeI6PPvssxEJyy8aiYQ9CLwMFYmE\/fchWWAQgbxrl7NqwlgqEwljIVmeHCLh8rCnkR1EIG4ZX57jiYiEHXSuoyoRCTvqGFKrHATkckne44nkvSPkujWGdbR3BAaK5cv4X2shtcvOaLbVAAAAAElFTkSuQmCC","height":213,"width":353}}
%---
%[output:62be428f]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAWEAAADVCAYAAACG00EAAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ1wH8V1X0iGWjQ1IEMCRsiygwxJ0yFAIMLgGtchTDojSNMkttQhru1StzHOZCLVskRmMm6wZas2Y8BJowFF40ximU4CsTxtahiRjwHhEL5UIAQTy0II5cOWYmzATErqzjt31dXqPnbv3u7trt7NZGL033373u\/t+927d3u7p508efIko4sQIAQIAUKgFAROIxIuBXcalBAgBAiBCAEiYZoIhAAhQAiUiACRcIngYwz98ssvsxUrVrD6+nrW0tKCIXKSjPHxcbZq1SpWXV3NNm\/ezCoqKtDHEAVu2bKF7d27l3V3d7Pa2lqjY8nCT5w4wdavX89mz55tBEuTxuzfv581NjZGQ6xevVpL\/zIxlzHh823ZsmVs6dKlJiFzRjaRsDOuyKdIGSQMY+7bt4\/ddttt+ZRO6SUTgsmxZDU4ke3atYvV1dVl2qar2\/3338\/mzJmjJDtzcKEBv3kMDw+zrq4uVllZqdOduUTCoDjoA77IY4uW4Y40JhJ2xBG+qGGa9EVCAExMZvky5jrBr4sDEHBraytTJXid+RAaCeveDHWwcrEtkXBGVtbZ2Rm1gEdU8RGZk8X1118fBRZc7e3tkx6hoA3vD+UC\/jjPAxj6Hj9+PHr8luXLavEg5n\/nwSyTAW8Hj6QwNn805e1GR0cnPbLK5Qb4ER7JeVYF\/83LEevWrYuy34GBgUjGZZddNilb4WQAv8m2iuUSEZckXO+66y52xx13TBmL6xOnAx8f8ISLYyD6RXxsF8fmOEAGzMs6\/G\/yWEk6JP39wIEDE6UCrheMkaRL3JSUdeHzifuL2wz\/HUf0Sf2hvMTn8nnnnTeBt4iZjKuIG59XV155ZTRn4IIMVrR50aJF0d+PHj06MV\/k+SjqrHuDc5FYdXQiEk5AS35EkzMZTiR8ssb9zmubs2bNioiMBzifZDDpYcKOjY2lZnxpskF9MVsUSZiTiTypefCD7jfccMOkmm8aCQOxjoyMaOkK+txzzz0TNzAVXDlusm0yycu6iDjBDQJuJiCL+0i0G+qNYubLfbB27dqJG2nc7\/xmImOqoxvMgzRd5HJC1o0SiFS8cWb1l3GT57LsIxEH8aYszgc+l2FsWV+4iUG9mt+05fkuzxHb7yF0CNNEWyLhGFTTsiJOpHG1S06Gt95665SXWWkBnTbpsh41kzJhMbNIexTOCvCkoEt6ESjq84UvfCEiB54Zgy3izQj+LmOtUo6QszrIePlYYl00jujEl37iYy\/oAkQhZoBixi5nl0nZWpxucDNMu5HCC8i0R\/C438S\/8RtOUk1YxkGe8lk3RmgvZ8M8E4+7KcvjyXP44YcfnlSa4VjyG2DWnDdBhGXKJBKOQT8uwGSyuvvuuye9xRd\/lx\/b+RD8MU7O8NJIOCsrUCHhtBcv2CQMtvEbDpBPc3Mz48Gli2tSJsyz2yuuuGIiK4+78cWRMC8PiW4H4oUXZjIJy4\/M0IeTdFImHKdbEgkn6SKvCoi7iYq23XTTTamZcFY9OouE+c0IbnYyznEkLI+XRMJy6PHSGZFwmbcER8Y2kQmLpskBHFImDHZygli4cCE7ePDgRClCF1eZhGXc4rJunUxY9ElWtsiJJelGmqabSiacNvXLzITnz58\/6amOP83wJYsYmbBsO5GwI0RYtho62UFWkPGacNLEzcp25cxCrKHJRBWX9SQRBGSochbF63W8xieXI+JKCrKvxEdyec2qCq5ZtXR4CQT1SHgaEV8+5qkJy\/XntEfiuNqoXOdP0k0m0qxSiYhp1tOKbk1Y9mGaTzgJgz7w\/oKXEtLKEXlqwuLKkax4KJsbsMenckQKoipv8aHG+dWvfjWSkrY6QlxJoJMJc\/V0V0ck1TDl1RFi5gr\/hkdyWLERtzqCr3jguKSt6OBt4t7Uq+DKV6LIYz399NNRPREu\/tZ95syZESnDxV\/GgW7cN0mrI6A91y8uS09bFQB9dXTjxAcvqTiB8RdW3Mdpy9fSVjeoZI4qqyM45vKNXFzFAfMYkgk+P5JeKot95DkFL+\/kUo\/oI1odgU3zMfJg8sIV94WX7Bx5GZQF9ZSGSKuzKgmgRsYR0F1vKma6uh88GDfG4wHili6mmaPrN4+hiVS3nglzgJM+rYTfe3p6rHwiW8R5RMJF0DPTN27ZoLg8LmtUmHvwIrGMT6azdPPpd3kJJugur4pJs2e63QytkjA8Em3YsCHCP+n7fHgUGhoa0vr2vYwJSiRcBurpY8qP3GK5QUVbn\/eOULHPZhu5fCZ+rJSmB\/ch7R1hyFtAXDU1NRHJJpUjxHqhbhAZUpvEEgKEACFgDAFrmTA8ouzcuZPdfvvt0VvtOBLmmciCBQuir5bo8dCY30kwIUAIOIKAFRIGct24cSNbvnx5tD1h2os5EReZlMXf5s2b5wiEpAYhQAj4hkBfXx+bO3euE2pbIeG4L4\/A+qx9TzkJNzQ0TNn+D0h4cHDQCRAxlQjVLsAoVNtCtYt8hhnZybKskLA8fFImDEX5pqYm1tbWFmXMUI6AtnH7ioY68UO1iwLaTkBjjxLqfHTJrtJJWCZeMWtO+xjAJRAxJ\/6hQ4eceUzCtAtkhWpbqHaF7DOX+KMUEsYIbpdAxLCHy6CAxkTTjizymR2cMUdxiT+IhDE9iyCLAhoBRMsiyGeWAUcYjkg4MBARzJkQQQGNiaYdWeQzOzhjjkIkjICmSyAimEMkjAmiZVlEwpYBRxjOJf6gcgSCQzFFUEBjomlHFvnMDs6YoxAJI6DpEogI5lAmjAmiZVlEwpYBRxjOJf6gTBjBoZgiKKAx0bQji3xmB2fMUYiEEdB0CUQEcygTxgTRsiwiYcuAIwznEn9QJozgUEwRFNCYaNqRRT6zgzPmKETCCGi6BCKCOZQJY4JoWRaRsGXAEYZziT8yM+G4jbKzMIAjiR588MGsZoV+dwnEQoZInSmgMdG0I4t8ZgdnzFFc4g8lEhY31ckCAvZ+2LRpU3REjMnLJRAx7aSAxkTTjizymR2cMUdxiT8ySRjTcExZLoGIaRcFNCaadmSRz+zgjDmKS\/yRScK8HAHHXPMjzzHByCvLJRDz2hDXjwIaE007sshndnDGHMUl\/sgkYTBcPrSvvb09On6ozMslEDFxoIDGRNOOLPKZHZwxR3GJP5RIWDRePIgTXsDFbbiOCVaSLJdAxLSXAhoTTTuyyGd2cMYcxSX+0CZhDoS8aiLrqCJMAEGWSyBi2kYBjYmmHVnkMzs4Y47iEn\/kJmEREL4iYtu2bayyshITq0RZLoGIaTAFNCaadmSRz+zgjDmKS\/yRm4TlTLi+vt7qizuXQMScHBTQmGjakUU+s4Mz5igu8YcWCcunJqedAYcJWJwsl0DEtJUCGhNNO7LIZ3ZwxhzFJf7IJOG4L+Z27do15Qh6TIBUZLkEooq+qm0ooFWRcqcd+cwdX6hq4hJ\/KJNwXV0da2lpUbXReDuXQMQ0lgIaE007sshndnDGHMUl\/lAiYfpsGdP96bIooO1hjTUS+QwLSTtyen72a7am50U2fudiOwNmjKJEwqtWrWIDAwPKCtMGPspQTWlIAZ0fu7J6ks\/KQj7fuN6RcD4zzfdy6XEC01oKaEw07cgin9nBGWsUImEkJImEkYC0KCZUsgrVLpgaIdpGJIwU9ETCSEBaFBNiQIdKVHxahOizLfuG2JZ9h\/ypCVuMUa2hiIS14HKicYgBTSTsxNTSUoJIWAuu5MZEwkhAWhRDJGwRbKShQvQZkTDS5CASRgLSopgQA5oyYYsTCGkoImENIGHbTLjiPhIhEtYA0pGmRMKOOEJDjRB95j0Jnzhxgq1fv57t3buXwaY9K1euZNu3b2fYO6jt37+fNTY2sqQtMomENSLJkaYhBjRlwo5MLg014EMNWCHhzccaom2cgBcsWMDmzJnDenp6op3Tent7WX9\/P9ouarBfxYYNG6KhYZMgyoQ1ZpjDTYmEHXZOgmoh+uymrz3DHj141E8SBnLknzCPjY1NkPDIyEh0wjJWNgxliJqaGjY0NETlCP\/iNlHjEAOaMmH\/JqjXJAxwA0GOjo6ym2++me3ZsycqR6xZsyYqTWBs8APbZe7cuZPdfvvt7O677yYS9m+OEwmTz5xG4MN3PM6Gx9\/2MxPmyPJ6Lf9vrIM\/odyxceNGtnz5clZbWxsRftqLOfitr6\/PaYfrKgdPFVVVVbrdvGgfqm2h2gWTKjTbFtcvZcc+fopXvKwJm450edN4Pl7cyzl6MWfaG\/jyqRyBj6lpiaH57NFfHmU3ff0ZImHViZOVCQ8ODqqK8qZdaJNeBD5U20K1K8R6N1+e5m0mHHfKhsxumEceEQl7c+9QUjRUsgrVrtBIGOrAt\/W8GK2MOP2tI+zINz6jNG9NN8rcT1hWgK9cWLp06cRP999\/f7SSAV7Mwb9hudpdd91lVHcqRxiF14jwUMkqVLtCI2GxFFEx8G32Wl+XkXmuK1SLhMUlavDijF\/ikfewdA2Wq3V3d+vqotWeSFgLLicah0pWodoVEglDFgy1YPj\/6soZbLz3n9nwEz9wIi60SBg0hky4s7OT8cM+5S\/bKBMu5lcK6GL4ldGbfFYG6upjimUI6NX7+cvZ5z5+BXPlnZI2CYMR4ioGsQYsZsSVlZXqKOVoSZlwDtBK7hIqWYVqVwiZsJgBgz2QBT\/75WuYS\/yRi4RLjuVoeJdAxMSDAhoTTTuyyGd2cNYdBWrAt+1+MSpBcAKGLBiI2CX+IBLW9azh9hTQhgE2IJ58ZgDUgiL5p8lcDBAvJ2DXkjhtEpa\/luNGwgnLXV1dzHQZgo\/n0p2s4HyZ1J0CGhNNO7LIZ3ZwzhoFMl44tgh2SBPJ9zsr\/4z96ez3TOruEn9okTBfJ7x27Vr2yCOPRJ8Xwye2sLUl7KwmLlvLAqzo7y6BWNQWsT8FNCaadmSRz+zgnDSKXPcVCRjqv3GXS\/yhTcJ8F7UHHngg2ukMiNfmCznKhMud8EVGD5WsQrULfO2qbVDv7dh3KPrwQr7k0kNQJMz3E4YVEYsWLYo2Xb\/vvvui3dSGh4epHFGEof6vr6uTHsE0ZwO6qG3ks6IIZveHbPdfHhpi33niV7GNgXibPlbDbqm7IFuYYy\/2tTJhbt2OHTvYjTfeyODDDCBi2MYSNnevqKhQAgCjkUuPExj2cBkU0Jho2pFFPsPHGUj37keG2YHfvBmb7cKIQLzXvv9s1nDVBey6i8\/WUsIl\/shFwlrWGmrsEoiYJlJAY6JpRxb5rBjOQLi\/+PWbbMcPhxMJl48AxPvlv5zHrq45KyLhvJdL\/EEknNeLhvpRQBsC1qBY8pkauHy97rf2j7InDr2eSbg82132kfNZ49UXFCJdWUNvSVhl7whaoqY2IZNaUUAXw6+M3uSzyahzsv3hS+Pse0\/\/RolsOeFWnzODbf3MJWz+e8806krvSDhps3URJdt1YZdAxJwtFNCYaNqRNV19xsm246FDbHjsbWWyFQm349Pz2ZHj\/61d0y3qWZf4Q6sckZQJFwUkT3+XQMyjP2XCmKiVKytkEn5s4GV2wYVVbHvfK2zoyAktohXJ9i8urWRfXDKnXEcJo7vEH1ok7AyCji0xwcQl5IAO1Taf7eLZ7J6B37KHfz7Ghn\/39sReCzrzGl6SQSnh01e+j10\/vxK1fqujh2pbr0hY5TQNMJw+W1Z1f3o7nwM6C4FQbXPZLvllWF6SFbPaj847i93y0dmRu4usUMiaLyZ\/94qETQJRRLZLIBaxQ+7rckAXtTNU28qyixPs86NvsH9\/7jB7dfzt3JmsSKiQ0X7phjls3rlnsj+8\/is2d+7coq53rr9L\/EHlCMemR1kBbQOGUG0zYRcn2CdfOcb6fjFWmGAnkWzlDLZiwYXsyuqZmdmsCdtszLWsMbwnYTg9o7W1dcLO9vZ2q5v3wMAugZjlcJ3fQ530gEGotunYxcn1D\/9zkt376Ah7\/rU3oukRtweCzrwRywXzzjuTrbn+IvZH7z49k2SzxtCxLUuWS7+7xB\/amTAcbwTbWfJtK3nNuK6uLjro09blEoiYNoc66UMn4XeddWrPgpOMsW\/vH2U\/PfQ6GrmKBFtzbgX722tms3Pfc0ZhglWZt6HOR5f4Q4uE6WMNlWlbrE2ok95HEuZZK+h+7O132M7HR9lLv34TnVwjkj1nBvvQhX\/Cbr3uQvau00+zQrAqMzXU+egtCYPTKBNWmbr524Q66V0iYSBXfuruf75whA2MHEepuYpe56sGgFwvqpwRfXZ70Tmn9jrwaUVBqPPRaxKGSUQ14fwkm9Uz1ElvmoTFrPWxg0dZ\/8Gj7JWxE4VWC8T5SiRXKA189srz2WlvHmYXXXSRV+SaNQ\/576HOR+9JWNWBJtu5BCKmnaFOel0SFkkV\/g3LsJ57DT9j5b4TyXXxJZXsry5\/Lzv9tFNlgazslXyGGQF2ZLnEH1o1YTvwqI3iEohqGqu1CjWggUhhzSl\/gXXkjd9HZ4HxGmuRjwiSkBWJ9QMX\/HG0LOvMM96VSapqnvr\/VqH6TPfGqYtbme1d4g8i4TJnQszYvgU0z1hPnmSs979+G+0LW\/SjgTSXiMQKhzdCrfWsindPECuv9dp0q28+08EmVNu8JWG+HK26utr6SRryxHEJRJ1JndW27EkvlgF+dGA8Wmpli1ThBdb7Zp7Bll9zIeOFAB9eYpXts6w5VeT3UG1ziT+0M+G4bS3pY40i03xyX+xJL5Jq3y\/G2ZOvmCVVsX4KKwOqZ81gjVddwKrOmcFeffVVdu1ltXhgOSIJ22eOmBWpEaptXpNw3ASB1RK7d++mgz4Roidr0ouk+h\/PHzH6soqbIy+3WnLpLPaROac+eRVJN8v8LNuy+rv6e6h2EQnbmXEomTCcvtzd3c1qa5OzHDGDTttxTc60k9q6dCcr6ipOrAcPv8V2\/uQgG\/\/9qRqniZdVUzPVCrbw4rPZNfMmH5RoogwQKlmFaheRcNHIVuuvRcK8Jgyi+WfLKsOcOHGCbdy4kS1fvjwiasic+\/v7Y+vK8El0T09PZs3ZFxIGgj124h3W1f8aO\/jbt4wQq5ipVs+qYB+7tJJd8X+bs+hmqyr+zNsmVLIK1S4i4bwzXa+fFgnriU5uDdnupk2b2LZt25h8Jh0Q9NDQUOY+FK6R8E9e\/h3b+tAQCsnKj\/+L51eyq+eeNQlQE5kqln+T5IRKVqHaRSRsOiJOyS+FhNMyYfgsurOzc8L6Xbt2MdgcSL7KJOFXxt9ma3te1N75ihPnwovPYXXzzmLw\/3KmSgFtZ+JjjkI+w0TTjqwy+UO20CoJi6d0xJErlC3Wr1\/PFixYEG2NCaWJ5ubm2HozgAhXX1+fca+NHnuHdT35Ovv+C8dTx5o981Qt98oLZ7C\/u+psBvuw8L+pKjkyMsKqqqpUm3vVLlTbQrULJldoti1ZsmQiZgYHB52IH6skzC3mZAxbX8ZlubydTMoiYjbuZFDPvenrz8SeucXP1Lrzs5ewnw0dYw1XnY\/iUMqqUGC0KoR8ZhVulMFs8IeqolokjLWVZRq5iorzdg0NDVPI2iSIj\/7yaES+8gXE+\/1\/\/HC0p4CpmiwFtOrUdacd+cwdX6hqYpI\/VHXg7ZRIOO4DDXmg+vr6xBUNMnmDvHXr1rGOjo5Jy9rkdlCOgBpx3EoMUyB++I7HJ2W+QLYdn5rPPv7BWbrY5mpPAZ0LtlI7kc9KhT\/X4Kb4I48ySiQslhGamppYW1tb6prgOEVkIuc14TiCXrFiBRsdHWVp64+xQfzB80fY33zzuQnVgXx7P3+5sYw3yVkU0Hmmcbl9yGfl4p9ndGz+yKODViZcZABTfbFAhLrvln2Hoh29+LX1r+ezlddeaEr1VLkU0KXAXmhQ8lkh+ErpjMUfGMpnZsL8Jdrhw4fZ1q1bo\/LAwMDAlLHTvoLDUFSWgQGi\/OKtrOxXtI0C2sRsMSuTfGYWXxPSMfgDS69MEsYaCFtOURCBgG8T1vq6QMCAEQU09kwxL498Zh5j7BGK8gemPtOShIGAOx4aYrue+FWEpSsETCSMObXtySIStoc11kjekrD4sYUMhk\/lCHEJmksETCSMFWJ25RAJ28UbYzRvSTjJeKgTL1q0KPXDCwzgRBl5QYQsGJahuZYBc9sooLFninl55DPzGGOPkJc\/sPUAeSjliLQNeUwoDTLzgiiuA4YlaNddPHkLR1P6qsqlgFZFyp125DN3fKGqSV7+UJWv0w6FhH3Z1B2Woa3peTHCZ+3iarah\/v06WFlpSwFtBWbUQchnqHBaEeYtCafVhJN2OzOFaB4QK7\/0w4kyxLNfvsaUaoXkUkAXgq+UzuSzUmAvNGge\/ig0YEpnlEzYlHJpcnVB\/OK\/vcS+tX80EuliGYLbSgFdxmwqNib5rBh+ZfTW5Q+TOmqTsPyZcdrewCYV1wFRfhnnahYMeFFAm5w1ZmSTz8zgalKqDn+Y1ANka5Fw0u5nqqdhYBqjA+I\/fe8A63rstWh4IGBTO6Bh2EcBjYGiXRnkM7t4Y4ymwx8Y46XJ0CJhrK0sMYzSAdGHWjCVIzBmRTkyiITLwb3IqDr8UWQclb5aJJy0v2\/alpMqSuRpowqi+GHG1xo+gLb5eh6dVfpQQKug5FYb8plb\/lDRRpU\/VGQVbaNFwjAYlB5aW1sZXw0BBNzY2Mja29ujI4lsXaog3vS1Z6Kz4KAE4XItmDJhWzMHfxwiYXxMTUtU5Q\/TeoB8bRKGTkl7A9tQmI+hCiIvRexYdilrvPoCmyrmGosCOhdspXYin5UKf67BVfkjl3DNTrlIWHMMI81VQORZMCjg+gs5yoSNTBMrQomErcCMOogKf6AOmCJMi4STXszZUlYcRwVEn17IEQmXMYtwxiQSxsHRphQV\/rCljxYJg1KwWU9NTY3V+m8cGFkgimuDfXghRyRsa8rjj0MkjI+paYlZ\/GF6fFG+Fgn7tJWlj6UIcAwFtM3pjzMW+QwHR5tSvCVhmyBljZUGopgFX\/f+s1nvmsuzxDnzOwW0M65QVoR8pgyVMw2JhBFckQaiuDa45ca5rOXGGoQR7YiggLaDM+Yo5DNMNO3I8oqEfTzos\/m7B9g3+099puzyZj1x040C2k4QYo5CPsNE044sr0jYDiT6o6SB6NsHGqL1FND6c6HsHuSzsj2gP77XJOz6LmpiPbjhqvMZrIzw6aKA9slbp3Qln\/nnM29J2Idd1MTTM3wrRVBA+xfM5DM\/feYtCfuwi5qvS9P4VKasyr+gJp\/55zNvSdiHXdT4QZ6+bNgjT18KaP8Cmnzmn8+8JWGA2vVd1Pinys031LC2T8z1bnZQQHvnMqoJ++ey3Ke1mzBV64s5roCru6iJ64N92bCHMmET09quTLpx2sUbYzSvM2EMANJkiAR\/2WWXsa6uLlZZWTmlSxyIcJw9vJiDi0jYtKf05YdKVqHaFfJLRyLhhPiFmvPGjRvZ8uXLWW1tbVT66O\/vZ5s3b2YVFRWTesWB6PP6YHoxp0\/qrvQgEnbFE+p6EAkrYgVZ8aZNm9i2bdumZMMyiD7vFyHCQQGtODkcakY+c8gZiqoQCSsCpZMJiyTs234RRMKKE8LRZkTCjjomRS0i4QyfiVtm8rPs5C4yiFv2DbEt+w55XQ8OuQYXsm1EwkTCRRDItTqiyIA6fTkZt7S0sLq6uik1YfhDX19f9Pe1vb9h\/a+ciP791Fp\/dk2T8RgZGWFVVVU6MHnTNlTbQrULJlZoti1ZsmQiXgYHB52IHW0S5qcry9qnrWTIa2nSZ9IgT86EQ3gpF3K2GLJtlAnnjfDy+nlbjuCZ6bJly4wcbyR\/Fg0v5tatW8c6Ojqi1RLiJYIYyku5kIkqZNuIhMsj07wje03CTU1NrK2tbQop5gVD7qf6IYgIoviRho+b9ogYUEBjzSR7cshn9rDGGslbEgYAYMXC0NAQgzptmVdSJkwkXKZX0scOlaxCtSvkpxdvSdjVgz5DWRkR8qQP2TYiYXdv\/EmaeUvCLkEtghjKS7mQiSpk24iEXWIGNV2IhNVwSm0lgsi3r\/TtZOU4AymgESaHZRHkM8uAIwznNQnzZWN79+5l9fX1bOXKlWz79u2xnxYjYJUogoMoroz41OXvY\/fd8kGTwxqXTQFtHGL0Achn6JAaF+gtCYvrdufMmcN6enqizXV6e3sTN9oxhSYHUVwZAefJwblyPl8U0P55j3zmn8+8JWFxHe\/Y2NgECcNXNUkb7ZhyDwfR9zPlZHwooE3NGHNyyWfmsDUl2VsSBkC2bNnCRkdH2c0338z27NkTlSPWrFkTlSZsLluLI+HxOxeb8pk1uRTQ1qBGG4h8hgalNUFekzCgJH+63N7ebuQLujSPcBBDWhkB9lJAW4tDtIHIZ2hQWhPkPQlbQyplIA6i7wd7UjnChdlUTAci4WL4ldGbSBgBdQ4iP9gzhOVplAkjTIwSRBAJlwB6wSGJhAsCCN0BxB89+XMGmTBcPm\/kLsJBAY0wOSyLIJ9ZBhxhOK9JWFwnzLFYvXq11ZdycSQcwvI0yoQRoqsEEUTCJYBecEhvSZgT8OzZsydIl\/8NMIk7kLMgVondAcTV\/\/rIxGkavm\/cww2lgDY1Y8zJJZ+Zw9aUZG9JWN7vlwOUdiCnSRA7vvs4+\/tv\/zwawtcj7mV8KKBNzRhzcsln5rA1JdlbEgZAYHka\/1KOH0MPa4dramqsLlMDED\/U9D326MGjrLpyRkTCIVwU0P55kXzmn8+8JeG0rSxFN8BRRw8++KBRzxAJG4XXiPBQySpUu0J+R+EtCRuJzJxCAcSjn+yKeoeyPC3kSR+ybUTCOYO4xG5EwgjgiyQMm\/bA6ogQLgpo\/7xIPvPPZ96TMBxx1NraOoF8GZ8tV1\/9CfbGdesiHUJZIxxythiybUTCRMJFENA+8h5ewsHLua6uLlZZWcl4nbiurs7qWmGRhENZnhYyUYVsG5FwEQoqp6+3mbDZuttkAAAJM0lEQVRLS9Sqrr+FvXXFysiDoSxPC5moQraNSLgcIi0yqrckDEa7kgmf\/+k72O+rryUSLjITLfcNlaxCtSvkG6fXJAyOcaEm\/N7PfYO9c+4lQa0RDnnSh2wbkbDluznCcN6TMAIGhUUQCReG0LqAUMkqVLtCvnESCSOEf2hbWHJIKKARJodlEeQzy4AjDEckjAAiJ+GQ1giHnHmEbBuRMEJAWxZBJIwAOCfhkNYIh0xUIdtGJIwQ0JZFEAkjAM5JOKQ1wiETVci2EQkjBLRlEUTCCIATCSOAaFlEqGQVql0h3zinJQnLJzTv2rWLwVd28gV7E69YsYKNjo5GP8GObPzrPLEtkbBlBkUYLlSyCtUuImGESa8gQvuzZQWZU5rAl3YbNmxgX\/nKV6JPnYGQ4aOPOHKN2684bkxOwiF9LRfypA\/ZNiLhPKxQbp9pmQmLkPP9JlpaWqZkw\/AhyNDQUOY+FJyEx+9cXK43kUengEYG1II48pkFkJGHmPYkDCWHdevWsY6ODlZbWzsJXsiQOzs7J\/6WVLYAEg7pRA1uMAU0crRZEEc+swAy8hDTmoT5waALFiyYchyS\/BuUJpqbm1l3d\/cUsgYSDmkzdyJh5CizKI5I2CLYSENNWxKOO605DdM0wgYSfveRl9hPN9yI5BY3xIyMjLCqqio3lEHWIlTbQrUL3B+abUuWLJmY1YODg8gzPJ84Ky\/mQLU0Qk1SnfdpaGiYUjsGEv6HP69imz45uZyRDwZ3elFW5Y4vVDUhn6ki5U67aZcJqxKwvF9x2ioKIOHQvpaDKerS5MAOmVBtC9WukOejSz6zkgnLa395cMNLt\/nz57OmpibW1tYW1X3FtrNnz46tB0N\/IOEzn\/4mO2P4MWyuIHmEACEwDRCYduWIaeBTMpEQIAQIAW0ErGTC2lpRB0KAECAEpgkCRMLTxNFkJiFACLiJAJGwm34hrQgBQmCaIEAkPE0cTWYSAoSAmwgQCbvpF9KKECAEpgkC3pGwuIRt9erVmRv9uOpHFTv4+uq9e\/dGZtTX17PNmzeziooKV82atMRQxT9p+4i4ZqSKz0BnsV3SVqwu2aZql7gdrYpvXbIxThfYp6ampmbK9gm29faKhMXd12Byr1+\/nsXtQWEbRN3xVO2AHeXgWrp0aa4vDnX1Ktpe1S4+Dr\/JPPXUU4nrwYvqhNVf1TZ5h0DVXQGx9NSVo2qXeLOEz+p9jT2OD98orL29nUhYZ9LARNi0aRPbtm1btC8xTPD+\/n7ns0PZxrx2uG6vrl1gz3PPPcdeeOGF2B31dOaG6baqtkG2+OMf\/9ibJzQdu8Q9wOHfcMF2tD5d4v41oDdlwprekzd8T\/usWVO01eZ57XB94uvYxYMfHmvBrrhtTa06JWMwVdvgxnL48GHW19fHBgYGEk+GccU2VbtUM2ZX7FLRg8oRKihJbeRM0FcSzmOHD7bq2AUBsGjRIjZr1qzEvaVzTBFjXVRtg3b33HPPRHnF9Runql0ALCdiuLkk7fNtzAEGBBMJ5wBV9a6dQ7TVLrp2pO2rbFVxpGxRfGT35cWcqs\/kGrDrN09Vu2Q7XL+5qMQFkbAKSlIb1fpVDtFWu+jY4XoQi8Cp2iWfngIy0jZrsuqchMFUbVMlNRdsAh1U7ZJJ16d5mYQ1kXCOWRhKXUrVDt8muqpdMnEnHXWVY4oY66Jqm7gdK19FADcYV19gqdoVlwnDieiuL5lMmxBEwjnDRXVNY07x1rol2SFOjLiM0fW1wip2+UjCPGtcsWIFA\/IR18nKwSxi4Lq\/dOyCUktra2vkPh\/WP2cFM5FwFkL0OyFACBAC0wABrz7WmAb+IBMJAUJgmiFAJDzNHE7mEgKEgFsIEAm75Q\/ShhAgBKYZAkTC08zhZC4hQAi4hQCRsFv+IG0IAUJgmiFAJDzNHO6yueLWnTpbJbr49Rbf9tHkRyjiUrgQPiN2eW6a1I1I2CS6HsnW2aFNp60OBHk\/TnGVhHt6eox\/zMBvXA0NDayurk4HbmrrCAJEwo44omw1dIhVp62OXfInv6p9iYTXMyJh1dniXjsiYfd8Ykwj+aQO\/sgvnpjAv\/AaGRlh\/OswUCitLchdtWpVtHUjXGmPxuJOXGJbUYekR3jx8VtsAyR8\/Pjx6H9wConcX5QNY\/KNvPm+CTNnzoz6cb3FLxXBbujf1dUV7WGdpIPsNPmGIuuY9sWZfFNJu+lRJmwsXKwJJhK2BnX5A4k7fMnBKwY6aAonJ\/DsSt7pLK4tP+EkbVc0PiZvK+8Ol1aOkE+sENvee++9EYl2d3ez2traaH9ivq8BjNnU1MTa2tqi38R+Y2Nj0Y1m7dq1E6criL\/DMVKAw\/DwcETCcMHNBvaBgEf\/NH3jSLizs3MS0SftvUAkXH6s2NSASNgm2iWPJe91K6qTlm3JZCm2hYxZPO0EZCZ9ky8TdBwpi6c3iPqlEZ4OaYHuu3fvjkgVSFjePChtt7ADBw4wsc6bloXGkbBIumk3Kx17KBMuOagQhicSRgDRJxHiJiziY7tMwuIjOTw6w8VPvxDbQgmisbFxCgRxqxtkItUh4bSbRBpp8ayeH5a6cOFCduzYsVgSlvWBvqLODz\/88MQGNqLBceeUxZEw9OG7qYm7rUGGLl5Ewj5FVHFdiYSLY+itBPGxvbe3d+K8PshuxQwxrRwRlwknAVJGJgw3CTG7lssRRTLhNMdTJuxtWFhXnEjYOuTlDRiXYQ0NDUXZmVxigFrp1q1bo9on9BNrrmk1YV67XbZs2ZRTbDFrwiKhP\/DAAxGoPMuUM\/Xm5uaoXsz39+U13rhyhE5NmL+k4zjJ5ROxdCFjKN4AofYsl4Z4yYTXpeH3uL17qRxRXjxhjUwkjIWkB3Lk1RHiG3pOKOedd170qA4vu+BFElw7duyI\/pu\/kJLbQhtxdUTahxZJqyPkR39YiSBf4soE+E1czZBEwvB3eLnGV03ACzqwBUorcMVtKM9LMVCuAbvgKSFudQT0TzoyPSkThhuAfACoXJoQMeI6PPvssxEJyy8aiYQ9CLwMFYmE\/fchWWAQgbxrl7NqwlgqEwljIVmeHCLh8rCnkR1EIG4ZX57jiYiEHXSuoyoRCTvqGFKrHATkckne44nkvSPkujWGdbR3BAaK5cv4X2shtcvOaLbVAAAAAElFTkSuQmCC","height":213,"width":353}}
%---
