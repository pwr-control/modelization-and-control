clear;
[model, options] = init_environment('afe_inv_psm');
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
application_voltage = 690;
vp_xi_pu = 1;
vn_xi_pu = 0;
vn_eta_pu = 0;
grid_emu_data = grid_emulator(grid_nominal_power, application_voltage, vp_xi_pu, vn_xi_pu, vn_eta_pu);
%%
%[text] ## Global Hardware Settings
afe_pwr_nom = 250e3;
inv_pwr_nom = 250e3;
dab_pwr_nom = 250e3;
cllc_pwr_nom = 250e3;
fres_dab = fPWM_DAB/5;
fres_cllc = fPWM_CLLC*1.25;

hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, fPWM_AFE); %[output:1671d5cf]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, fPWM_INV); %[output:7d61eaae]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, fPWM_DAB, fres_dab); %[output:67ba1b32]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, fPWM_CLLC, fres_cllc); %[output:0f56a32d]
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
l1 = Kd(2) %[output:4c6b811c]
l2 = Kd(1) %[output:7fbcc266]
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
Imax_adc = 1049.835;
CurrentQuantization = Imax_adc/2^11;
%%
%[text] ### Voltage sensor endscale, and quantization
Umax_adc = 1500;
VoltageQuantization = Umax_adc/2^11;
%%
%[text] ### DClink, and dclink-brake parameters
Vdc_ref = grid_emu_data.Vdclink_nom; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
%[text] ### DClink Lstray model
parasitic_dclink_data; %[output:98f96b6a]
%[text] ### DClink voltage control parameters
Vdc_nom = grid_emu_data.Vdclink_nom;
Vdc_norm_ref = Vdc_ref/Vdc_nom;
kp_vs = 0.85;
ki_vs = 35;
%%
%[text] ### AFE current control parameters
%[text] ### Resonant PI
kp_afe = 0.2;
ki_afe = 45;
delta = 0.015;
res = s/(s^2 + 2*delta*grid_emu_data.omega_grid_nom*s + (grid_emu_data.omega_grid_nom)^2);

Ares = [0 1; -grid_emu_data.omega_grid_nom^2 -2*delta*grid_emu_data.omega_grid_nom];
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:45aea3a5]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1;
pll_p_ddsrt = pll_p;
omega_f = grid_emu_data.w_grid;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
tau_ddsrf = 1/omega_f;
%%
%[text] ### PLL FHT
pll_i1_fht = pll_i1;
pll_p_fht = pll_p;
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega_fht0 = grid_emu_data.w_grid;
delta_fht0 = 0.05;
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:35515fc7]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:5bf91939]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:62e4964c]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:3b1f3b29]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid_emu_data.w_grid;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:692a65d7]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:1c141170]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:3c15e2f9]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:4677dc5c]
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;
kp_rc_pos_grid = kp_rc_grid;
ki_rc_pos_grid = ki_rc_grid;
kp_rc_neg_grid = kp_rc_grid;
ki_rc_neg_grid = ki_rc_grid;
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = 2*pi*rms_perios/grid_emu_data.w_grid/ts_afe;
rms_perios = 10;
n10 = 2*pi*rms_perios/grid_emu_data.w_grid/ts_afe;
%%
%[text] ### Online time domain sequence calculator
w_grid = grid_emu_data.w_grid;
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
time_start_motor_control = 0.25;
%[text] ### MOTOR Selection from Library
n_sys = 6;
run('n_sys_generic_1M5W_pmsm'); %[output:3033f022] %[output:4c9e8daa]
run('n_sys_generic_1M5W_torque_curve');

% n_sys = 1;
% run('testroom_eq_psm_690V');
% run('testroom_torque_curve_690V');

torque_overload_factor = 1;

b = tau_bez/omega_m_bez;
external_motor_inertia = 5*Jm;
% external_motor_inertia = 1;
%[text] ### Simulation parameters: speed reference, load torque in motor mode
% rpm_sim = 3000;
rpm_sim = 17.8;
% rpm_sim = 15.2;
omega_m_sim = omega_m_bez;
omega_sim = omega_m_sim*number_poles/2;
tau_load_sim = tau_bez/5; %N*m
b_square = 0;

%[text] ### Double Integrator Observer Inverter Side
Aso = [1 ts_inv; 0 1];
Cso = [1 0];
% p2place = exp([-10 -50]*ts_inv);
p2place = exp([-50 -250]*ts_inv);
Kobs = (acker(Aso',Cso',p2place))';
kg = Kobs(1) %[output:21a96945]
kw = Kobs(2) %[output:38469327]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:938a66db]
luenberger_l2 = Klo(2) %[output:2ab93113]
luenberger_l3 = Klo(3) %[output:69e6c58a]
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
CTRPIFF_CLIP_RELEASE = 0.001;
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
Vdc_bez = grid_emu_data.Vdclink_nom;
motorc_m_scale = 2/3*Vdc_bez/ubez;
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
igbt.inv.data = 'infineon_FF1200R17IP5';
igbt.inv = device_igbt_setting_inv(fPWM_INV);

% infineon_FF650R17IE4;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.afe.data = 'infineon_FF1200R17IP5';
igbt.afe = device_igbt_setting_afe(fPWM_AFE);
%[text] ### DEVICES settings (MOSFET)
infineon_FF1000UXTR23T2M1;
mosfet.inv.data = 'infineon_FF1000UXTR23T2M1';
mosfet.inv = device_mosfet_setting_inv(fPWM_INV);

infineon_FF1000UXTR23T2M1;
mosfet.afe.data = 'infineon_FF1000UXTR23T2M1';
mosfet.afe = device_mosfet_setting_afe(fPWM_AFE);

%[text] ### DEVICES settings (Ideal switch)
silicon_high_power_ideal_switch;
ideal_switch = device_ideal_switch_setting(fPWM_AFE);
%[text] ### Setting Global Faults
time_aux_power_supply_fault = 1e3;
%[text] ### Lithium Ion Battery
nominal_battery_voltage = grid_emu_data.Vdclink_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_inv); %[output:6428a939]
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
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
else
    if use_thermal_model
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'off');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'off');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    else
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    end
end

if use_torque_curve
    set_param('afe_inv_psm/fixed_speed_setting', 'Commented', 'off');
    set_param('afe_inv_psm/motor_load_setting', 'Commented', 'on');
else
    set_param('afe_inv_psm/fixed_speed_setting', 'Commented', 'on');
    set_param('afe_inv_psm/motor_load_setting', 'Commented', 'off');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":27.4}
%---
%[output:1671d5cf]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:7d61eaae]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:67ba1b32]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:0f56a32d]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.880000e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:4c6b811c]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:7fbcc266]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:98f96b6a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVgAAADPCAYAAACweMkPAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXX1wVtWZf3CoEkYcCFJpDF8qbJU\/qLZu3dQ1dKyL3Rra4loJs9OYAZa1Ujo1LAnRWWAq4WPFqYilETI0O2qsTpkt+aeUBWFGKd1WlN2ltioYkEldKR9DnIItU3d+B8\/ryeX9uPe9H+fc+\/7OP0ne93z+nuf88tznnvM8Qz788MMPhYUIEAEiQAQiR2AICTZyTNkhESACREAhQIKlIhABIkAEYkKABBsTsOyWCBABIkCCpQ4QASJABGJCgAQbE7DslggQASJAgqUOEAEiQARiQoAEGxOw7JYIEAEiQIKlDhABIkAEYkKABBsTsOyWCBABIkCCpQ4QASJABGJCgAQbE7DslggQASJAgqUOEAEiQARiQoAEGxOw7HYwAm+++aY0NzdLf39\/7ouGhgZZvXq1VFVVlYTr7Nmz0tbWJo2NjXLLLbeUrL9v3z6ZM2fOoHoLFiyQ1tZWCdpXycFYgQgUQIAES9VIBAEQ7JIlS2Tt2rUyefLkwCQXlBRBsGvWrJGuri6prq7OjVdTU6NIloUIJIEACTYJlDmGlCJY0+LUliZgA0l2dnZKfX29QhHfwYL98Y9\/LEuXLs195iVNL8GiIubQ0dEhjzzyiCJ6WMPTpk1TlnFvb6\/qS1vV+F1\/js\/OnDkj7e3tsn\/\/ftm7d68cPXpUZs+eLRMmTBhkKT\/77LMyZcoUaWlpkdtuu02+973vCUj90UcfVWs5cOCArFq1Su69915qRQUgQIKtACG7sMR8LgJNNCb51tbWKmKrq6tT5KWt0BMnTigXA4gKpaenR7kXNBF6XQf5CPbkyZOK+B588EHZvHmzIthx48apPq6++mrR35tEijFAiosXL5YtW7Yogn3uuedyljHG0S4LkH5fX5\/Mnz9f5s6dqz4H8WMNqAdreseOHYqg\/bpGXJAd51A+AiTY8rFjywAIFLJgNZFqwoQ\/VhOV7t7rNz1y5EjOetV1TKsXn\/klWJCgdj\/AioW1uXHjRkXAmBsszULEq33HXutbEyzmra1tEC\/+Rl1zrQEgZNUUIkCCTaHQ0jhlL8FiDZpI8fgflGA1YRXCwq+LAO3xMsx8tNcWbimC1dYzfsIi3bZt2yALlgSbRk2Nds4k2GjxZG8FEChmwd500025F2B+XQT6kd2sb\/o1i73kWrRoUe5Egulu8LoC9KN8oc9N94T25cICpgXLbaARIMFSFxJBoNQxrThecvk5poUXUvCXgkRRf2Bg4KKXX\/lecmkfqn7ZBmJFP6+99pr6Z7Fw4ULlEqCLIBH1cnYQEqyzouHEXEEgn7vBlblxHm4jQIJ1Wz6cnSUEzGNgmAJ8tH4uOFiaLod1FAESrKOC4bSIABFIPwIk2PTLkCsgAkTAUQRIsI4KhtMiAkQg\/QiQYNMvQ66ACBABRxEgwToqGE6LCBCB9CNAgk2\/DLkCIkAEHEWABOuoYDgtIkAE0o8ACTb9MuQKiAARcBQBEqyjguG0iAARSD8CJNj0y5ArIAJEwFEEnCRY7zVFYMco8I5qEKdFBIhAQQScIlgdUSkfmWrS5Z1wajMRIAJpQcAZgkXUeESUb2pqKopdd3d3yTppAZ\/zJAJEINsIOEOw2YaZqyMCRKASEXCKYM2gzHAToCBzKIIZI+Ec0j2zEAEiQATSgoAzBGvmvddR5pEWGWlA4H9lJs60qBTnSQSIgEbAGYKFD3bFihWybNkyqa6uVqmOkQwPQY6931F8RIAIEIE0IECCTYOUOEciQARSiQAJNpVi46SJABFIAwJOEazO7pkPOKRF7urqUu4DFiJABCoTgfPH+2TgxW4Z9Y1lqQDAGYK1hdY111xja2iOSwSIgA8Exl56XqaNOCd\/N3pA\/Xz3T0PlH\/9nXMmWhw8fLlkn7grOECxeZNmwYEGwNgRha9y4FYr9u4dAGnUNlur59\/pkYHe3DOz+kQwdM1Gqpk6XEdObZNjU6SVBdmXNzhCsiRiOZfX19Ulra6v6GH+j4MhWOUVfwUVb7zVcW4J4++23ZdKkSeUsh22IQCAE0qRr2gUAUkUBqQ6bWi8jpt8XaM229rV3ks4RbL4jWWGOaaFtS0uLtLe3q7V3dHTIunXrcr5cW4JIk9IH0mxWdg4B13UNpHr24G5lrZ47uFtZqyDUEV9sUr+XU2zta+cJVl84qKury1msOBPb398vq1evlqqqqkB4w3pFe7wgQ9u2tjZpbGxU52tRbAnCdaUPBDIrO42Aq7oGYj31\/IqyXAClALe1r50nWEzQ648Nc4IABNvT06PIGQUEa5L3J7\/5Q\/U5xkiyfPDBB3LZZZclOSTH8iDw+4HzZWHSfyZYu5orhubG8bZd8Ncj5Z8+P7KsefhtdOzYMamtrfVbPd56p47Jh7\/+icjhX4oc2icyqlaG3PEdkc\/dHdm4t99+u+rLxruVVBBsZEiLSCmCHfsPj8jdd0cnXL9zf\/\/99+Xyyy\/3W531YkBgfHWwp6HWGeU9rhaa+prtffLyW6fkpUOn5dZrR8q2B26MYZUiti3YQi6Aqqn1vl5YlQMKLVgPanGFK6SLoBz1ZJskETh68pws7HldEW3rjEkSNZHbIlj9wurUC8sDnwIIiz8JNg+CcQTc5kuusKrK9kkhAIt2zfa3I7dmkyTYQtZq0hcDSLBFtDbqlDHmMS1vRgRbgkhS6ZMiCI4THgFtzR49dU42zL5ebr0uvH82bl0Le2Y1PGoX92BrX1ecD7aU8GwJIm6lL7Vufu82AjOffFW5DLZ968bQJBuXrnnPrA79JI5XNQU+sxqHJGztaxKsBwFbgohL6eNQVvZpBwHtMjj52BdDTSBKXYvjzGqoxRVobGtfk2BJsHHoM\/uMCYGX3jotM3\/waii\/bBQEm+\/M6pgHtsS06vDdkmDDYxhJD7YEEYXSRwIAO3EeAfhlQbIocBmMrx4WaM5hdA0XAc7+ZnfuhhVeVgW9thposhFVtrWvU2HBmhcNNm\/eLLt27VKZZEvl5DJfjpmXExiLICKtZTfWEAjz8isIwWoXwLmDe3I3rECoSZ8CCAs0CbYAguZVWQR8QdoYFH0bq9BVWSRMNOMM6Ou1S5YskYcffpixCMJqLNs7gYD2ywY5L+uHYG2eWY0DWBJsAVTNwC6bNm1SBIskiGa+Lj8C0Te4Zs2aJd\/\/\/vcZi8APaKyTCgSC+mULEawrZ1bjAJ0EG8CC3bNnT+BgL7BgJ06cKBMmTCgai0AH3N65c2ccci7Yp1P3wxNdOQeLAgHENGjoPiaIc9D59bHqZ6GST9c+fP5fRBATYFStyLWflyHf+LcopuVEH4xFUEIMYYO9mPFkS8UisPWfzs9jmxPaykk4jQDOy5a6lABdG3f5EJVqxYyz6vIpgLCg29rX3nk7Fw\/WL7DwuTY3NyvLtqGhIRfKUFuuOjg3YxH4RZT10opAIb+sdgEc\/1mnilwVRZzVtGBEgvVIKoqUMSBX+Gx1rFcMwVgEadkSnGcYBEy\/7DNTD4h5CuD8hM9KzV0LY4tcFWbecbUlwRZBtpyUMeZRLN21tmwPHDggc+bMUR8zFkFcKs1+bSIAa\/XAD5fKqP9+TvqHjpW\/uvVO0S6ASnRHkWALaGPUKWNKKb0tQVSi0peSBb8PhoAZCwC\/axdA0\/GZKo7Bk43XS+PNY63Hgw22qmhq29rXzvtgo04ZU0pctgRBgi0lGX5fCAHv7SpcBPAGr9Z+WRDs4luGVVyCTVv72nmC1X5TM4V3mJQxpbapLUGQYEtJht9rBLSlal5Z9XO7SvtlcYTrf5f\/bUUBamtfp4Jgw2oCThjgBtfatWvV9VpelQ2LKNsniYD3uirG1o\/\/QdOs4Irt3C2vyvGzEll82SSxKHcsEmwRH6xpvepqfq1Y7WJ45ZVXZMuWLTJ69Gim7S5XS9kuUQSOP9ksfz7elwusElV8VTwtPf\/GEJUtIcgV20QXH\/FgJNgAgOJUAW5kmcevCjVH3ePHjwsItr29XU6cOMG03QGwZtV4EdAH\/XGMSpOptlBBqFU3TI88sIp2RwW9YhsvEvH2ToINgG++kwX5msM10N3dLffff38uwAsItljabluCoA82gAI4XBUvnExfKX4HceY+e69P8MhvFjzug0w\/MWZi7ihVnEs0dS1MVK445xh137b2tXcdqbjJZd7Gqq6uzisLuAZWrlypwhqaboFSBHv4niFRy5b9VQoCuMdvluqP\/h51de5TF+7454tFsPw\/\/yC9r78vr3w72lTkLoiesQiKSKHQja5Vq1aJvv6K5t6rsvPmzVOWK67O6lJTUyPf\/e535emnn2Y0LRc0n3OwgkChp6WeX70rD\/S8HipbgpUF+RiUFqwPkMqtYl6P5UuuclFku6wgUMwdZWZLiCqLrQu4kWALSCGKm1wmwXqPafGqrAvqzzkkiUApf7\/2y+L2V9gEi0muq9hYJFgPOvp4VW9vb17czIhZUQrRliBKKX2Ua2RflY2AX13Tt79uvXakbHvgxlSDZmtfe0Fz7iWX3xMDUUnfliD8Kn1U62Q\/lYtAEF3TR7mQWDHNLgNb+9pZgtXEumjRIlm8eLEgApZZ\/F40CLqNbAkiiNIHXRPrEwETgaC6lgWXga197SzB2toStgQRVOlt4cNx049AubpmBoxBZK40FVv7OhUEa6bf1hP2Y8Gaflwc0cJVWcYiSNO24FzjQKBcgsVcTJfBtm\/dKHAdpKGQYAtISZ+DbW1t9XU11uzGTBeDywlIljh\/\/nzGIkjDjuAcY0MgDMFiUmm8\/UWCLUKwQVN0o6tCL8eYkyu2fcuOU4JAWILVyyyU+8tFGEiwRaQCFwGKeXOrlBA1waIejnppF0Gpq7K2BBGV0pfChd8TgSh1LS0BY2zta+d9sIWuypbyweqrs48++qhyLYCk9+7dK7NmzZKtW7eqrLMobW1tUldXlyNvCAJl586die7EfPfDE50AB6sYBKLWtf4z5wWxDH4\/cF6Wf+lK+ezVbvllGYsgAtX2xiJAgO2HH35YhSjUL7bgk0V8go0bNzIWQQSYs4t0IhClBWsigDgGiGfgYoxZWrBFfLD5Am6junkyIF9z8yWXtmBN4kWbjo4OWbduneioXLYEEZfSp5MCOOs4EYhT11wNGGNrXzvvIsAEC6XtRtBtxHZ9\/PHH8+qj6V4wXQpmyhjGIohzK7NvFxGIk2CxXhcDxpBgi1iw3lME5i2v9evXFyTYcpTbliDiVvpysGCbbCKQlK7NfPJVlS7cBZeBrX3tvAWrLwvAHYCzsNqixQsrPO4jtqv+PIrtYEsQSSl9FBixj3QjkKSuuRIwxta+dp5gMUHvSQI87m\/YsEFliTVPAESh9rYEkaTSR4ET+0gvAknrGlwGn3nkF+rWl62AMbb2dSoINklVtiWIpJU+SUw5llsI2NA1M2AM4hg03jw2UVBs7etUECxOA3R2dg6aa6lzsKhsHt0q9JLLm3rGliBsKH2iGs7BnEHApq7ZChhja187T7BmNoL9+\/erdN1HjhxR8y52s0v7brULASSNwlgEzuxzTsQSAjYJFku2ETCGBFtA2cyYAm+88UYuYIuf+ATmOVj9Owgav3d1dUlVVZW6ydXY2JgLJGNLELbGtbTHOaxFBFzQNdNlgKhct143MlZEXFgzFuhcRgNYojiKBcsTcQSam5tVplg\/LgIsSLsX9HlXnIHF2dlSV2VjlTY7JwJEQCFw7tNflXOfnikj\/2Nu7IgcPnw49jFKDeAcwWLCyGYwfPjw3JVXZDjQsV0LLcgb5lC7COrr64sSbCmA+D0RIALRIgCXQdwWbLQzLr83JwnWz3K8sQjmzZsnjz32WO4arA5TWCoWgZ+xWIcIEAEiUA4CqSVY72K9FqzfWATlgMY2RIAIEAE\/CDhDsIXCFOpF+PHB+jmm5Y1F4Ack1iECRIAIlIOAswRLIixHnGxDBIiASwg4Q7AmKPmuyuKYlQ4x6BKAnAsRIAJEoBACThKsd7JmXi2SLJWZCBCBtCDgJMGavlQA2dDQoM6x4qIACxEgAkQgLQg4Q7CFgmWnBUjOkwgQASLgRcBJgs0nJj+nCCheIkAEiIBLCDhDsC6BwrkQASJABKJAgAQbBYrsgwgQASKQBwGrBIvbVkuXLh00LW+8VkqNCBABIpBWBKwQrM7ymo9MNenyokFaVYrzrhQEBnb\/qOBSz793IYZzvnL+eF8iEI15YEsi4xQbJHGCxWmB3t5eaWpqKrr47u7uknWiQA9xI1mIQKUh8M2a03LVpX9Wyx576Xn186rLLvzUf5eLybt\/Glqy6f99ULpOyU5KVPj6z86G7SJ0+8QJNvSMI+7AVmBeW+NGDB+7cxABWJawIGEp\/vl4n5w7uPuiWQ4dM1F9NvSTE+UT+vcxE2XUN5Y5uKLgU3Jlf1kj2GLBXZCyu1T81+CQ529hSxC203hEhR\/7sYcACPTswd1y7uCei4gUBKrJ8\/2\/\/1eZNGmSvYlaGNnWvvYu1RrBYiLwt\/b19Ulra6uaF\/5GQZoXZCF4\/PHHYxeNLUGQYGMXbeYGAKGef69PBnZ35whVW6JVU6dLIZ9jJeqarX3tDMGaubd0fAH92aJFi1TamKAEq1+e6UXql2jm58wqmzneyfyC8MgPK1W\/VAKpFiNULyAkWHsqYs2C1Vlg4Q4wLdi9e\/fKkiVL5Omnn8597hcer0WMdmaWWvzd0dGRy3qAv239p6tEpfcrR9a7gMDxJ5sVqerH\/ZrlL5YFTSXqmq197YwFq8lv7ty5KgcXCq7DbtiwQdauXSs6\/XYQjTKzyup2ZiQul7LKVqLSB5FlJdaFC2DgxW45+xv4VXcrYsVLpxHT7wsFRyXqGgn2I+tSE+zmzZtl165d6mjW5MmTAytUoRiySP3tJ6vszp07A48ZpsGxY8ektrY2TBdsmxUETh2TD3c8LvLrn4iMqhX53N0y5HN3X\/g9glJpunb77bcr1Co6q6x2EcBSxYsuZH9F0WQYNjShzsk1a9Ys2bp1a9G03TYEUYlWRQRckZku9AkAvLCCtTps6nQZMb0ptLWaD6BK1LWKt2DNl1ybNm1SBDtlyhRZsWKFLFu2LHT2Ar9ZZW0JohKVPjPsGGIh2g1w6oXlygWAx\/+4z55Woq7Z2tfO+GDzWbB79uyR\/v7+soJre08lwB+LMn\/+fGlpaZH29nb1N19yhWAHNi0bAbys0tYqiHX8D94uu6+gDUmwQRGLrr61UwRYQtS5t8zjWGb8WPNzb4wDW\/\/pKlHpo1Nb93syLwHokwBBjlZFucJK1DVb+9oZCzZKBQrTly1BVKLSh5FTWtqaZ1b18aq4fKt+MalEXbO1r60TbLErsphc0pkLbAmiEpXeLyGkqZ4mVH3nP+yZ1TjWXom6ZmtfWydYcwKFrsree++9keoZb3JFCmdFd+YlVIChb1YNm1ofyymAsICTYMMiWH57az7YYldlozhFoCHhTa7ylaMSW3ojUeHuvxm\/VJPphZ\/16niV64UEa09C1gjWPEWgLVa8+S\/3FEEhCHmTy55y2Rr51PMrckOb5IjHeBSQpvqZJ\/BzvjB+rlqmfvElwfpFKvp61ggWS4n6FEE+eECwxW5yHb5nSPSoskdrCPQPHTto7JoJF+KeqjLq6o9+XrghNeSO71ibZ5ID8yZXkmgPHssqwSax7FIEa8sZnmaroudX75YU3dGT50rW8Vvh6Mnikenf8Yx19NTHY\/uZx\/jqYTJ+1DA1nS9cN0r9bJ1hELPfiTpaL826Vi6ktva1d76JE2zSKWPoIihXRSuj3ZrtH+eHevmtU2rRLx06PWjxmoBBvvi98ebBVrLrSJFg7UkocYLFUpNMesiXXPaUKwsjv\/TWaXn50GmBFQ1LWZPvBaL9VCoIlwRrTxOtEKxeblJpu3mTy56CZXFkuB3gJoHFC8LVZOuqW4EEa08LrRKsvWV\/PLItX00lKr0L8o5jDnAzmGTbOmOSU26EStQ1W\/vaug82DgXXfTJlTJzosu9SCGjLds32t5VV+9rDf1OqSSLfk2ATgTnvIJmyYJkyxp4iceSPEQDRLux5XbkPYM3adh2QYO1pZ6YIlilj7CkSR74YAbgOYM3eeu1I2fbAjdYgIsFag16sEqx50cBmyhhmNLCngFkfGdbsZx75hVWSJcHa0zJrBOtSyhjAz5xc9pQw6yP3nzkvDd3H5JVv27m8wJtc9jTMGsGGTRnz5ptvSnNzs4pd0NDQcFEWBKaMsadUHPliBLQla8MnSwvWnkZaI1imjHlbJk2aZE\/yHDlxBHBpYeYPXpVt37pRbr1uZGLjk2ATg\/qigawRLGYSdbAXpoyxp0gc2T8C1Q++mCjJkmD9yybqmlYJVh+rQmLCuXPnyoEDB8SbMyvqBXv7s3UguRKVPm5ZpqX\/mU++qqaa1MmCStQ1W\/vaq4PWCBYugpUrV0pTU5Ps379f+vr6ZNasWdLd3S0PPfSQVFVVJbJfbAmiEpU+EYGmYBDtKkjKH1uJumZrXztDsN6XXBMnTpQ77rhDVqxYIX4zGngvFpguB\/PFF1PGpIB1KnCKcBXgthdufcVZSLBxolu8b+sW7F133SWdnZ3S3t6uZurXgsWlArRbsGCBtLa2qrb6osHMmTOlra1NGhsbZcqUKdLS0pLrv6OjQ9atWyfV1dWqja3\/dJWo9PbU3M2R4SoYVz1Mnmy8PtYJVqKu2drXzliwmIi2LEGS8MNqIpw8eXJRhYPlOmHCBNmzZ4+qB4LV1it+v+WWW0Rbt\/X19Yp4u7q6lNtBEy\/qkGBj3dfsvAQCSZ0qIMHaU0VrFmwUSwZxmgRrEjQIdu\/evcqvu3XrVnVOFgUEW1dXJzoPmK3\/dJWo9FHIPGt9PNDzuoo3G2dgmErUNVv72ikLVj\/mm5OaNm2asjb1I3yxDRUVwWIM3uTKGnWlZz2ffaJPGq6\/XJZ\/6cpYJs2bXLHA6qtTaxasmWkApwjwyH\/kyBE1aW1d4nd9IaG3t1dqampky5Ytol0IXoLFUS+6CHzJnZUcQgDBu2HJxnUBgRasPWFbJVh9YuCNN95Q\/lT4YYOcIjAJFhDyJZc9ReLI4RDACy8ka4zDVUCCDSebMK2tESws0\/Xr1ytSPXHiRC6uQLkuAoBgHtMyTxcwZUwYFWHbpBDAsa04zsaSYJOS4MXjWCNYTAU3t4YPH64e+UGCixcvHuQCSAIWW87wSlT6JOSZ5jHiOlVQibpma1879ZLLhc1gSxCVqPQuyNv1Oegg3VH6YytR12zta6cINl9W2SAuAu9NLubkcp0+OD+\/CEQZEIYE6xf16OtZcxF4LwYEXVq+m1zMyRUURdZ3FQGcKsDpgih8siRYe1K2SrBBTgyYEOW7yWWeIjCPeenA27zJZU\/JOHJ5CGh3wcnHvlheBx+1IsGGgi9UY2sEi1mDKFFMQgyymnznYPHiDEW7GnAErKenhze5ggDLus4goF98hUkDToK1J87ECdYbZNu79CA+WO85WK+V6\/eqLNrxJpc9JeTIpRF46penpfO\/TkvNFUOlt6m2dAOjBm9yBYIr0sqJE2zQ2fu9yeXtlzm5giLN+q4jgLxe8MsiFThK481jfUXiogVrT7JWCLZUwkK\/cHhdBKZPV39nRulCvwxX6Bdd1nMZAfhnj548qwgXBS6Exps\/Ja0zLs5cS4K1J8nECVZbpIjVipCB+nprOX5Yr4uAObnsKRJHtosACPflt07JS4dO5wgXv3zh2pFyxZBzsmr2jXYnmPDoFXsO1sxkgIhZsGa3b98uCxcuTFgEF4azJQhb41oBmYMmjgAIF8UkXXMSOovC+FHDVNDvC1ZwVV4LOPHJRzCgK\/srcQs2H8H6zWIQAe4XdQFBsBCBSkPg3Ke\/qpb8l+Gj5S\/DL4RJxO8XfoYLm3jJH\/9QEs5L\/niiZJ2wFd77938O20Xo9hVPsKERZAdEoAIR0L7ffEvHy7hSBf7juEvcqXj8zN8KweoU3fkmGOSYlp8Fsg4RIAJEwBYCiROsrYVyXCJABIhA0giQYJNGvMh4OGHxwgsvyO9+9zuZN2+eTJo0yaHZcSpZQWBgYECef\/55OXPmjIwbN06QhfnSSy\/NyvKcWgcJ1iFx\/PznP5fRo0er+LjPPPOM3HfffSoTLgsRiBIBXB8\/dOiQfPnLX46yW\/aVBwESbAJqYeYf0\/nEzFCNzz77rDoT\/NRTTymlh1WBkxUNDQ2+kj8msAQOkQIE\/OrZrl275Kc\/\/alccskl8rWvfU1uu+02GTJkSApWmL4pkmBjlpm+tYZhdMJGfKZvlJnBaBCUBqQ6ZswYEmzMcsla90H0DEQ8YsQIlU1k48aNcs8998jYsWOzBokT6yHBxigGKPKmTZvkzjvvlOXLl8vatWvV4z+sVwSiWb16tcqa29LSIu3t7XLw4EG54YYbZOLEiYpgcbvtiiuuiHGG7DoLCATVs3feeUfp2VVXXaUIFv\/U8dTEEj0CJNjoMb2oR1gXS5YsGUSwfX19KsW4GXgcSg43wahRoxQRf+UrX0lgdhwiKwj41TP808Y\/eejZlVdeKbNnz5ahQ4dmBQan1kGCTUAcfhUfflgWIlAuAtSzcpGLrx0JNj5scz3nU\/x8LgL9AiyBKXGIDCJAPXNPqCTYBGTiVfxCL7l4JCsBYWR4COqZe8IlwSYgE6\/iY0h9TKumpiZ3uiCBqXCIDCNAPXNPuCRY92TCGREBIpARBEiwGREkl0EEiIB7CJBg3ZMJZ0QEiEBGECDBZkSQXAYRIALuIUCCdU8mnBERIAIZQYAEmxFBchlEgAi4hwAJ1j2ZcEZEgAhkBAESbEYEyWUQASLgHgIkWPdkwhkRASKQEQRIsBkRJJdBBIiAewiQYN2TCWdEBIhARhAgwWZEkC4uA8HE29rapLe3d9D0FixYoGLhprng3v\/27dsFKeixxrq6OhUgHUWvu7GxUaUC8hZ8v379epk\/fz5TAqVZCXzMnQTrAyRWKQ8BTTQm+ZTXk1utTIJp0BSzAAADIElEQVREBLSgBIvVaIJeuHChW4vjbCJFgAQbKZzszESgGMHqtDlHjx5VEfVvuukmaW5ulv7+fpk2bZp0dXUp627fvn0yZ84cQdSx6dOnq1xSsPxgOcIKhoWIvnSGCDOZpLaU0UdnZ6ea2p49e1SKFKTrATmuWbMm9x2ST6IgNxq+RwF5ei1R9HfkyBFlseZbo2nBYjw9Nvozx96wYYPMmDFDZa9gySYCJNhsytWJVeVzEWjy3LFjhzz33HOKSFF0XjKdswyEaRIp2oHsQLSFCLa+vj4vOaL\/xYsXq7CQSIuuyRmfg2AxhxMnTqhElA899JA88cQTsmzZstxn69atG\/QojzYYC+ReyA2CvnW2YC0Msx0+wz8DFO1acEJonESkCJBgI4WTnQW1YGEpHjt2LGe96vYg1Pvvv18l5dPWrLZUCxEskkUuXbp0kBBgxYIMNZHqR3pYpbBCteVrNtJEqC1e018MQl25cqU0NTUpy7OUBat9sF5yRd+whGHhpt0fTa0vjAAJltoRGwJ+XASaYHUac7gFdAEBaWLE534INh9hmv34IVhNfJiHtlT1nMoh2EKWKgk2NtVzpmMSrDOiyN5E\/BIs6uGxHb5YPC5r\/ywy8eIlECw800WwaNGi3IulmTNn5lwHIEPtCqitrc3VmTBhQl4L1usi0Jl\/0RZv+X\/729+K1z2g23hdBIVOEcBKLuQGoIsgezrvXREJNvsytrZCvwQLqxJv1f2+5ALhmil39Msv83Ms2nzJlc9FoF+QabfCqlWrcv5Q88WZF0DT8izmIkDadbg4Dhw4kOtC+6CxZtPVYE1IHDhWBEiwscLLzqNEoBjpRTlOEudYeUwrSom52xcJ1l3ZcGYeBJIg2JMnTyp3xfjx43NHufIJIoz\/1OvHpaCziwAJNruy5cqIABGwjAAJ1rIAODwRIALZRYAEm13ZcmVEgAhYRoAEa1kAHJ4IEIHsIkCCza5suTIiQAQsI0CCtSwADk8EiEB2ESDBZle2XBkRIAKWESDBWhYAhycCRCC7CJBgsytbrowIEAHLCJBgLQuAwxMBIpBdBP4fF9MqKZ5vUCsAAAAASUVORK5CYII=","height":207,"width":344}}
%---
%[output:45aea3a5]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:35515fc7]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:5bf91939]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:62e4964c]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:3b1f3b29]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:692a65d7]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:1c141170]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:3c15e2f9]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:4677dc5c]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:3033f022]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:4c9e8daa]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:21a96945]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:38469327]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:938a66db]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:2ab93113]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714302e+02"}}
%---
%[output:69e6c58a]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.994503273143434e+02"}}
%---
%[output:6428a939]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVgAAADPCAYAAACweMkPAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ2QF0eVf1HULJeQZIF8kAUW48ZE4yFE42YNRZSLnKeAd2eODy9yQCmWBK07KNglKVOcgV2oYEHAKCbrFlrHklPhWOrqisttJHWRYD4gaMUYIssGNgs5AtlADKhJ9upN0mtv73x0z3T3dDdvqizD\/rtf9\/u9179586b7zXm9vb29QBchQAgQAoSAdgTOI4LVjikJJAQIAUIgQoAIlhyBECAECAFDCBDBGgLWhNjnn38e5syZA1OmTIGlS5dqH+LkyZMwb948GDVqFDQ1NUFFRYX2MXiBq1atgh07dkBLSwvU1NQYHUsUfubMGaivr4cRI0YYwdKkMnv27IFZs2ZFQ8yfP19p\/mViLmLC\/G3GjBkwffp0k5CVJpsItjTo1Qcug2BxzJ07d8Ltt9+uPuGMHuJiNzmWOBVGUps3b4ba2tpM3VTn9uCDD8Lo0aOlZGcOzjVgN4bDhw9Dc3MzVFZWqnQHlwgWJ47zQVvk0UVJ8ZIaE8GWBLwPw5omdH6xIx4mo3MRb5WFrYoDkmtDQwPIkreKL4RGsKo3OhWsXGhLBPuOFXDBbdy4MfoXPjbyj62MCG6++eZo0eDV2NjY77GG74+P8OwRmy1O7Hv69OnokViULzoCW6Ds72yhigudtcPHRJw7e1xk7bq7u\/s9RoopAPwRH5NZNIT\/ZimCJUuWRFHr\/v37Ixljx47tF2WwhY6\/ibryKQwZXNetWwd33333gLHYfOLmwMZHPPFiGPB24R+lecwZDhi5slQL+5s4VtIckv5+4MCBvsd3Ni8cI2kucSQgzoX5E7MX0xn\/HUfiSf0x5cN8efjw4X1485iJuPK4Mb+6\/vrrI5\/BCyNPXueJEydGf+\/p6enzF9Ef+Tmr3rxcIE2VORDBvvOYwucCxQiEkQRzxLjfWf+hQ4dGJMUWL3MgdGh0xhMnTqRGammy0bB8lMcTLCMK0WHZwsa533LLLf1yrGkEi6TZ1dWlNFecz\/r16\/tuTuLjqIpuIoGLc+FlIfnjjQJtwGzE6435PT5iZTZYuHBh300y7nd2oxAxVZkb+kHaXMRH\/KybIJIkf1PM6i\/iJvqyaCMeB\/6Gy\/sD82UcW5wv3qAwP8xuyKK\/iz5iO++vQo462p7zBJsWzTCSjMsVMqL7yle+MuDFUNpiTXOorMe\/pAiWjwjSHk+zFm\/Sgkp6qcbP5xvf+Ea08FlEi7rwNxr8u4i1TIpAjMYwUmVj8XnIOBLjb5r8oyjOBUmAj9z4SFuMCpOirLi54Y0u7UaCL\/PSHovjfuP\/xm4mSTlYEQeRJLJuethejGJZBB13wxXHE334oYce6pcuYViym1uWz+sguTJlnPMEG7d4RCK69957+73t5n8XH6WZMdmjlRh1phFs1t1chmDTXmLoJljUjd1MkFgWL14MbOGo4irixC9yJMLx48f3RdNxN7U4gmUpH36BoSx8+SQSrPgYi30YASdFsCxi5ueWRLBJcxHfnsfdIHndpk6dmhrBZuV\/swiW3WjwRibiHEew4nhJBCuSHEtnEcGWSf8WxjYRwfLTFhdnSBEs6skW\/4QJE+DgwYN96QFVXEWCFXGLi5ZVIljeJllRHiONpJtk2txkItg0ty4zgr366qv7PY2xpxC2bU9HBCvqTgRrgeTKHkLlrp61gFgONskps6JUMSLgc1ZJOdi0lwb8I5kY\/bD8GMupiSmCuMd80Vb8Y7K4J1MG16zcNb5QwfwfPkXwL\/Ly5GDZWAzHtMfUuFykmFdPmptIklnpCx7TrKcM1RysaMM0mzCCxfng+wL2eJ+WIsiTg+V3WGSth7K5oej453yKgAEo87Ybc4rf\/va3oy5puwj4N+4qESybi+ougqScobiLgI848b\/xMRl3NsTtImA7AxguaTsfWJu4N9oyuLIdG+JYe\/fujfJ3eLG300OGDIkIFy\/2YgvnxmyTtIsA27P5xUXXaW\/Psa\/K3Bip4QsfRk7s5Q+zcdoWrrRdADIRn8wuAoa5eEPndzugH2OgwPwj6QUt30f0KXwRJqZfeBvRLoKiFC70R0fFK+4kkmgIcWuQ5qlIi0vLa0oLoYZGEVDdT8lHqKqb9Y0q4rnwuO17aSqp2s03eKxGsAzMpON9+Htra6uVY5oqhiKCVUHLTtu4rXP8FrGsWaCv4Uu5Mo7pZs3Np9\/FbYg4d3H3SJo+od\/orBEsPrYsX748wjrp\/Dc+rnR2diqdrbbhjESwNlBWG0N8DOZTADKSfK5FIKOfzTZiSos\/aJM2D2ZDqkWgwVpIUtXV1RGBJqUI+Hyd6oLRMEUSQQgQAoSAVgSsRLD4GLFp0ya44447orfBcQTLIoq6urrodA09wmm1MwkjBAiBEhAwTrBInCtWrIDZs2dHJenSXnLx+ouEy\/\/2\/ve\/vwSoaEhCgBDwBYH29nYYM2ZM6dM1TrBxJ2RQ66w6loxgZ86cOaDkGxJsR0dH6eDpnADppBNNc7LITuaw1SnZFTsZJ1gRtKQIFhPeixYtgmXLlkWRLqYIsG1cnUhXwAvRIUindATI93R6iDlZrtipVIIVSZWPdtM2trsCnk73OHTokBOPNKRTOgJkJ50eYk6WKxxhnWB1QOoKeDp0YTJo4epE05wsspM5bHVKdoUjiGB1WrWALFq4BcCz2JXsZBHsAkMRwQYAXgEVBnSlhasTTXOyyE7msNUpmQi2AJqugFdABSJYneBZlEUEaxHsAkO5whGUIihgRJ1daeHqRNOcLLKTOWx1SiaCLYCmK+AVUIEiWJ3gWZRFBGsR7AJDucIRFMEWMKLOrrRwdaJpThbZyRy2OiUTwRZA0xXwCqhAEaxO8CzKIoK1CHaBoVzhiNQINq4kXJbOWCR727ZtWc0K\/e4KeIWUEDrTwtWJpjlZZCdz2OqU7ApHZBIsf3w1CwA8ibVy5cqoiLHJyxXwdOpIC1cnmuZkkZ3MYatTsiscQTlYnVYtIIsWbgHwLHYlO1kEu8BQXhCsq198dAW8AvanHKxO8CzKIoK1CHaBoVzhiMwIVvwchPg11QIY5O7qCni5FYjpSAtXJ5rmZJGdzGGrU7IrHJFJsLzS\/Cddyvziqyvg6XQIWrg60TQni+xkDludkl3hCCWCZQCIuwuyimfrBA5luQKeTr1o4epE05wsspM5bHVKdoUjchEsDwTbObBmzRqw9X15V8DT6RC0cHWiaU4W2ckctjolu8IRuQhWjGBlP9OrC0BXwNOlD8qhhasTTXOyyE7msNUp2RWOkCZY8dtaaV8c0AlUnCxXwNOpJy1cnWiak0V2MoetTsmucETmQYN58+bB\/v37+3TfvHnzgI8Q6gRGRpYr4MnMVbYNLVxZpMptR3YqF3\/Z0V3hCCmCra2thaVLl8rqZrydK+DpVJQWrk40zckiO5nDVqdkVzgik2DpqKxOsyfLooVrB+eio5CdiiJop783BCumCLLgoWIvWQjF\/04LNx9utnuRnWwjnm88Lwg2n2rme7kCnk5NaeHqRNOcLLKTOWx1Sq66+Tbo2vVjnSJzyZLeRZBLuqFORLCGgNUslshIM6CGxIVmp9YnjsGC1mfh5Hc+ZQgxebFEsPJYGW0ZmpMjWKSTUZfRJjw0OxHBFnQNimALAmipe2gLl24alhyn4DBEsAUBJIItCKCl7kSwloAuOExodlq1sxNW7TxEKYK8fkEEmxc5u\/1CW7gUwdr1n7yjEcHmRe6dfkSwBQG01J0I1hLQBYcJzU7eEuyZM2egvr4eduzYAVjgZe7cubB27VowUUkLa8\/iFXeCjAi24Iqy1D20hUsRrCXHKTiMlwTLyLWurg5Gjx4Nra2t0NTUBG1tbbB79+7ovysqKgpC83b3PXv2wKxZsyCpziwRrBaYjQshgjUOsZYBQrMTbtHCF11ebdPCEoXs2OyJEyf6CLarqyv6kqyuKBbHWb58eeQ4WLGLIlgta6gUIaEtXIpgS3Ej5UG9JFjUEh\/bu7u7Ydq0abB9+\/YoRbBgwYIoXaCrGAyOUV1dDZ2dnZQiUHYttzoQwbplj6TZhGanqd\/dB48e7PErgmXGYY\/v7N86P4KINWc3bdoEd9xxB9x7771EsH6sz8RZhrZwKYL1wyE\/evdjcPjkWT8J1hTEmONdsWIFzJ49G2pqaqJoGa+kFAH+1t7ebmo61uViqqWqqsr6uCYHJJ1MoqtPdkh2mjRpEvR8oTkCx6scrD5zxksSv5jAWsW96KKXXKatoUc+RbB6cDQtJSQ7YeSKEax3BCt+hyvO6Do\/I5MVwXZ0dJj2O6vyQ3JyBhzpZNWFcg8Wkp3YCy7vCBYnzF5ATZ8+vc+YDz74YPRCCh\/l8b9xy9a6detyG5t1JIItDGHpAkJauHTTKN2dMifAR6\/vev1lePn7t2b2Md1AupoWv00Lc6Ts4j\/bjdu3cMtWS0uL0XlTisAovNqEE8Fqg9KooFDsxA4YIFiD9\/7Qv3qwGFVu3LgR2IcPxQMBOiPYNI8igjW63rQJD2Xh8oCQTtrcQ6sgPnodVXk+nPrhl8CFNKJ0BMtHrHPmzIn2w\/I5Vz6Srays1AqeKIwI1ii82oQTGWmD0qgg3+3EkysC1fb1cfDlz4z3k2CNWlpSOBGsJFAlN\/N94cbBRzqV7FTC8CK5\/rL+E1Bz6WBwhSOUI1gX4HUFPJ1Y0MLViaY5WWQnc9iqSEZixZqvWHOAXfff9iH4+3GXRf90hSOUCFY8xcUUwy\/JNjc3g+nUABvPFfBUHCKrLS3cLITc+J3sVK4dkFi37vs\/+Nf\/PNg3Ecy5YloA\/981jpAmWLYPduHChfDwww9HJ67w5BGWL8QKW\/zWLdMmIII1jbAe+URGenA0LcUHOyGx3t76bFRjgL+QVJ++88YBELnCEUoEy6ppbd26NSrIgqRq8+WWa3cnnY7vg5Or6ks6qSJWTntX7YSket8jR+AH\/9s1AJi4qJVv5B3BsnqwuHNg4sSJUb3WBx54IKqqdfjwYUoRFFwbrjp5EbVIpyLo2evrkp2SIlWGxk1XXQxtC8ZlguMdwTKNNmzYAJMnTwY8VIAki6UKdRbbzkTOoQS2zFxl27jk5LJzzmpHOmUh5MbvZdoJCXX1zk7Y\/MTRRDCyotW4jt4SrAsu4Qp4OrEo08l16sHLIp1MIatXri07IZl29ZyFpv86NCCXKuZV8d\/iiysVrV3hCOkcrIpyptu6Ap5OPW05uc45Z8kinbIQcuN3E3ZCMv3FwR5offxoKpkyBDBKXTGtBj73kWFaQHGFI6QJVqYWAW3Tyu8bJpw8\/2z09CSd9OBoWkoROyGR9px5A+78j+fh8Ctno0LXWReS6ahLzoe106+BQe86r9\/2qqy+sr97Q7BJdVp5RW3nYV0BT9bYMu2KOLmM\/DLakE5loK4+ZpadGGn+5Klj8MiBV6QiUj4yRTKd+8krYfyoIUbINE5jVziicASrbs7iPVwBr7gmf5aQ5eQ6x7Ili3SyhXSxcdBO777oikjI1n0vwcO\/PSkdjfJEiv\/9mQ8NhdtvHhX9md\/4X2yG6r1d4QhpglVX0VwPV8DTqSGRkU40zcny2U7Rd6p+\/ydo\/sWL8MKJM0qRqEikt1w7FBZ+6m0iLZtMvY1gZb5igMrRUdniC9rnhZukPelU3C9kJSB54v\/++OZb8LO9L8GRk2dzEShPlp+86mJY9FfVMOjdZvKksrrlaedKEEYRbB7rGehDZGQAVAMiy7ITkmcvANy36wg8e\/Q15Ud4EQr2ounj1RfBp6vehJEjR5b6SK\/bVESwBRB1BbwCKgzoWtbC1amDKIt0ykaXvUD6wxtvwebHj8JTL5wqTJ4sCsWXSx8ecQF8beJIOC8lJxqinVzhCOUIFr9a0NDQ0Oc5jY2NVgu94MCugJe9fORbhOjk57pOjDzbf3sStu17KXIG2a1MaZ7DXh4hgd76scvhtk9cEaUH8r5UCtFOrnCEEsHiJ2OwZCErTchytLW1tdFHD21droCnU98QnTxUndgbd9yytOdQT5Tv1EGcfP4TyfOzHxkGn7tueORmeclTxkdDtJMrHCFNsHTQQMZV87cJ0cl90YnfHP9Wby9seqw7elTXFXEyr2B5z5GV58Nff3gYjK260Dh5ynikL3aS0YW18Y5gceIUwaqYWK1tiE5etk48cT76u1eio5s6o02eOPG\/b\/rAJXD96CEw6YNvf5MOCbXIo7uaB+VvXbad8s88uaeXBIvqUA7WhDsAhOjkJnR69Hc9fY\/Lj3e+CrueewUOnzyj7RFdJE18VMeIEzfQjxs5BI4cOQKfHPvnz9ab8Qa7Uk3Yya4GA0fzlmDLBg7HdwU8nViE6OSyOrFIs7e3N\/ocyM+fO6n98TyOOP9y5IUwt+7K6Dw8izizbCqrU5Ycl34PUSdXOEI6B+uSQ7gCnk5MQnJyRpiP\/+YQjKoaAf\/2+DE4dPx1K6T5gUsHw63XXwZXXvz295l0vxwKyU7Mf0PUyRWOIILVyZIFZLnu5Iw0e17\/E7Q+eQyeefE1Y4TJEyM+on+s+iL44vhL4YL3DepHmmXkN123Ux4XDFEn7wiWbckaNWqU9S8YiE7jCnh5nDmpj20nZ4R56uwbsOWJY\/CrrtPWCLN6WEX0Bh3Ps4uP7joxNSHLtp1M6CDKDFEnVzhCKYKNK11IBw30LIEiTs6\/LT\/+2h\/h3588Bs8e\/b01wsSBxo68EG77xAg4\/z3v6gPkzVePwpgxY\/QA5IiUInZyRIUB0whRJy8JNs5BcFfBli1bUj96yBNzWmEYkcCT2roCns4Fwzs5T5hHX\/0D\/HTvS\/DcMTuEGT2eX3I+1Fw2OCLMyr94T+4oM8SFSzrp9HpzslzhiMIRLH5ltqWlBWpq4reu4NdoV6xYAbNnz47aICHv3r07Ns2Ap8RaW1szUxCugKfqHow4n3zhFOw6cBI6X9a\/vUh85EayxOu6Ky+AL3z0Urh8yPv65TFVdVBpT2SkglZ5bUO0kyscIU2wLAeLbsCOyuZxCYxSV65cCWvWrAHxEzNIvp2dnZnHbl0BT9QfCfSZo6\/B93YdiX569GBPHohi+\/Dnz7HBx8dcBLeOvwwGv\/fd1ghTVZkQFy7ppOoF5bR3hSOkCVYXTGkRLJ4U27hxY99QmzdvBqxzIF5lg4dE+stDr8KP93QXIlGeNCvf9wZ8c\/I1MPSdR3L2WxlvynXZmshIF5Jm5YRop7I5glnMGsHyxbvjiBNTCfX19VBXVxdV58J0weLFi2PTD7bBQ5Jb1\/4CtDzWLeXpPHHiFqN\/unFEZpQZopOTTlLuUnqjEO1kmyOSjGiNYNkEGNFi9a246JS1EwmXVwDBw6u9vd2Yc3afegMeeKIHtv\/m7f2ecdeIIYPgigsHwezrL4Ixl7wH8N95r66uLqiqqsrb3cl+pJOTZhkwqZDsNGnSpD79Ojo6SjeAdYJNI04eDdZu5syZA4jY5N0Jo9XbW5+NffTHyPSuz18Fwy94L9z0gYu1Gi\/EKIJ00uoixoSFaCeTHKFiCGmCzVuuUOyHL7mWLFkCq1ev7rfzQGyHKQLMyca9UDMF3tTv7htArEiqbV8fp\/3IpWikEJ2cdFJZiuW1DdFOpjhC1UqZBBt3uEAcZMqUKalbq0QZLAcbR75z5syB7u5uSNv+pRM8jFhf\/+ObULf68T61kFT\/btxl8K3PvZ2KsHGF6OSkkw3PKT5GiHbSyRFFEM4kWCY8KYItMnjevrrAQ3Kdet++qGYnu750wxWwfsY1eaeWu1+ITk465XYHqx1DtJMujihqCGmCLTqQzv46wBPJFaPWDTOu1Z5bldU7RCcnnWStX267EO2kgyN0WCWVYNkb\/+PHj8M999wT5UT3798\/YNy04686JinKKApeHLnayLOmYRGik5NOJrxfv8wQ7VSUI3ShfM5FsHHk+vSdN+rCM7ecEJ2cdMrtDlY7hmgnItgCLpQXPHELFqYFXCBXhCJEJyedCji5xa4h2ikvR+iGXTqC5U9iiZPwJUWA33PCl1p4uUSuRLC63dqcvBDJKESdvCPYJJfFvOzEiRNTT2Xpdvc84GH0+tG7H+sj17JzriImITo56aTb883IC9FOeTjCBLrSEWzS4GnVsUxMGGXmAY8\/RIDkqvskVlFdQ3Ry0qmoV9jpH6Kd8nCECbQLE6xMwW3dE1cFj08N3HTVxdC2YJzuKRWWF6KTk06F3cKKgBDtpMoRpoCWJti0HGxSWUFTk1YBj981YOvYax69Q3Ry0imPJ9jvE6KdVDjCJOLSBGtyEqqyVcDjo9cfzbkOPv+R4arDWWkfopOTTlZcp\/AgIdpJhSMKA5giQIlgxeOyacWzTU5aFjxfolfEKkQnJ51MrgJ9skO0kyxH6EMxXpI0wSaVGZT9zItORWTBa33iGCxofTYa+qfzx8KnP1ipcxpaZYXo5KSTVhcxJixEO8lyhDFQ3xEsTbB5yxWaUEAWPLZzwOXcK8MnRCcnnUx4v36ZIdpJliP0o9lfojTBJhXATqvbamryMuDx+15d3TnA4xOik5NOplaAXrkh2kmGI\/SiWDBFgN0xHdDQ0ABs1wCS66xZs6CxsTH6jpatSwa8pVsPwP2PvhhNycV9ryJWITo56WRrRRQbJ0Q7yXBEMdTkektHsExcUvFsueH0tMoCTzy15Uq9gTTtQ3Ry0kmPv5uWEqKdsjjCNKZMvjLB2ppY2jhZ4PEEu3TyGFg6udqFaafOIUQnJ52cd7togiHaKYsjbFlGmmB9+qKB68di44wbopOTTraWcbFxQrSTdwSLJsTCLtXV1VbzrXGukwaej+mBUKOIEBcu6VSMzG319o5gfSlXyJ\/c8iU9QARra9kVH4cItjiGNiR4R7A2QJEdIw28b7UdhA27DkeifNg9wHSmhStr\/XLbkZ3KxV92dCJYWaRi2qWBhzVfMU3gWkHtLHVp4WYh5MbvZCc37JA1Cy8I1rePHvbfPVANmCLw5aKF64elyE5+2MkLgnUVyiTwVu3shFU7D3mXHqAcrKueNnBeRLB+2MpLgnW9mhZfe8CHwwW8q9LC9WPhkp38sJN3BOt6NS3fag+IbkoL14+FS3byw07eEazr1bT47VkbZlwLs2643A9PeGeWtHD9MBfZyQ87eUewrlfT4mu\/+rQ9i7Zp+bFgyU5+2ck7gkV4TVfT4gvJjB07Fpqbm6GycmCR7Djw+OOxJ7\/zKb+8IdDz4BTt+eGGIdrJS4JFdzFVTQsj5BUrVsDs2bOhpqYmIvPdu3dDU1MTVFRU9PPUOPB83f9KkZEfJER28stO3hKsLZiRyFeuXAlr1qwZEMXGgVf5Lz+PpvbF8ZfBD\/7xQ7amqW2cEKMI0kmbexgVFKKdiGAzXEYlguVfcPmYf0UoQnRy0skoL2oTHqKdiGAT3IMvKsO+nCA2FcHz\/QUXEaw2rjAuKEQyClEnItiMpcCIdunSpVBbWzsgB4t\/aG9vj\/7+1a3H4KkXz8KIIYNgx+wq44vMxABdXV1QVeXn3JPwIJ1MeIp+mSHZadKkSX0AdXR06AdLUaJ0wW1FuYWbJx1sQMHi3YnlX334uGESMCFGEaRT4WVgRUCIdqIIVnAd8SADvuRasmQJrF69OtpVwF88ePwJrqa\/rYGvTvAzCgzRyUknK\/xYeJAQ7eQlwbKvyIoWTduzqmJ92S1gPHghvOCiHKyKl5TbNkQyClEn7wiW5URnzJjh1CdjiGDLJZy00UNcuKSTu\/6W9JRb5oylc7CufvTQ9xNczPi0cMtcBvJjk53ksSqzpXcRLIKFe1M7OzsB3+yXefHg+VyikMeQFm6ZHiU\/NtlJHqsyW3pHsK5+9DCEHQSUgy1zKaqNTQSrhldZrb0j2LKAihuXgdf\/EzFjYOnkapemqTQXWrhKcJXWmOxUGvRKAxPBKsHVvzEDL5QXXBTBFnAGy12JYC0DnnM4LwmWbf7fsWMHTJkyBebOnQtr166NLciSExepbgw8n7\/BJSpKC1fK9KU3IjuVbgKpCXhHsPzJqtGjR0Nra2tUSrCtrS2xrKAUEjkaMfD+4Qe\/gv\/57YlIgo81YHnVaeHmcIQSupCdSgA9x5DeESy\/TevEiRN9BIvnmJPKCubARaoLAy+UHQSUIpAyuxONiGCdMEPmJLwjWNRo1apV0N3dDdOmTYPt27dHKYIFCxZE6QKbW7cYeKHsICCCzVwvzjQggnXGFKkT8ZJgUSPxuGxjY6P1k10iwS6d7PcOAiJYPxYt2ckfO3lLsC5AjOD96L\/3wtT79kXTIYJ1wSoD50DRnpt2EWcVop2IYAv4HoK368nfAH6HCy9fv2LAQxCik5NOBZzcYtcQ7UQEW8CBELz533sYVu08RARbAEfTXUNcuKSTaa\/RI99LguX3wTIY5s+fb\/UFF46L4F236Gfw6MGeaBq+b9Gi3J6eRWVDChGsDZSLj+EdwTJyHTFiRB+hsr8hHHGf1y4OU7wEnmBHVZ4PT995o6mhrMmlhWsN6kIDkZ0KwWets3cEm1SuMO3z2qbQRPB6vtAciff5MzGUgzXlIebkEsGaw1anZO8IFpXHLVrsBFdFRUWEB+6Nra6utrpVq\/q6G+DUZ1ZF48\/8+OXw3ZnX6rRNKbJo4ZYCu\/KgZCdlyErp4B3BppUr5BHEz8ds27bNKKg8wSK5Isn6ftHC9cOCZCc\/7OQdwboE66gbPguv3bQkmhIRrEuW6T8XIiN3bRN6eooItoDv8QQbwh5YhILIqIBDWOxKdrIIdoGhvCVY\/GxMQ0NDn+plHJUd8Tf\/DGevmRrNAXcQ4E4C3y9auH5YkOzkh528JFh8oYUvupqbm6GyshJYXra2ttbqXthLv\/x9eGPYByNLh7AHliJYPxYt2ckfO3lHsC5t02IEG8oeWFq4\/ixcimD9sJV3BIuwuhLBDvvaT+CtwcOC2QNLBOvHoiU7+WMnLwkW4XUhBxtSHVjmshQZ+bF4yU5+2MlbgnUBXkawIZQpJIJ1waPk50AEK49VmS26mJdcAAAIVElEQVSJYAugzwg2lD2w9OhZwBksdyWCtQx4zuGIYHMCh90YwYayB5YItoAzWO5KBGsZ8JzDnVMEK35mZvPmzYBbu8QLC8fMmTMn+u4XXnjslm0J49sSweb0OsvdiIwsA55zuBDtdM4QLG7vWr58Odx1113R3lkkW9yNEEecccVk4nyGEWwohwwogs3JDCV0C5GMQtTpnCFYcQ2wwwn4FVoxisUdCp2dnZmHFhjBhnLIgAi2BKbMOWSIZBSiTucswWIaYMmSJbB69Wqoqanp5+YY2W7cuLHvb0mpBCTYkA4ZEMHmZLsSuoVIRiHqdE4SLPsCQl1d3YD6seJvmC5YvHgxtLS0DCBiJNhQCm0zjgjRyUmnEu4AOYYM0U7nHMHGfXImzRfSyBgJdtDLz8Evl0\/O4U5udunq6oKqqio3J5dzVqRTTuAsdwvJTpMmTepDr6OjwzKSA4c7r7e3t9f0LNLIMmls1mfmzJkDcrVIsN\/89Ci46\/NXmZ66NfkhRhGkkzX3KTRQiHY6ZyJYWXIVi8mk7TZAgg3pFBeuDlccotBKFTqTTjrRNCeL7GQOW+MRrLi3lamCL7CuvvpqWLRoESxbtizKs\/Jt8eu1cflX7I8EO3jvD+G9h39hDhmSTAgQAl4jcM6kCLy2Ek2eECAECIGcCBiPYHPOi7oRAoQAIeA9AkSw3puQFCAECAFXESCCddUyNC9CgBDwHgEiWO9NSAoQAoSAqwgQwbpqGZoXIUAIeI+AVwTLb+OaP39+ZlEY16wjM3+2b3jHjh3R9KdMmQJNTU1QUVHhmjrRfGR04ieeVovCJQVl9eLbJZXXdEUvWZ348qI+rjOGN9Y2qa6uHnAs36Y9vCFYvgoXOnJ9fT3E1TSwCZ7KWLLzx4pieE2fPh1kD2mozENnW1md2JhMn6eeeipxj7PO+eWVJauXWBlOthpc3nkV6SerE38DxKPbvq0znlyxcFRjYyMRrIzjoOFXrlwJa9asierKojPv3r3b6ehOjNzyzN9lPVVtgrr8+te\/hmeeeSa2mpqMH9hoI6sXRnqPPPKIF09SKjrx9Zrxv\/HC8qI+XHzNE5wvRbCSVhOLcacdpZUUabVZ3vm77OAqOrEFjo+cqFNcuUqrBkkZTFYvvGEcP34c2tvbYf\/+\/Ylf4HBBL1mdZCNdF3TKmgOlCLIQ4n4XIznfCDbP\/F3XUUUndPaJEyfC0KFDE+sBK7iD0aayemG79evX96U7XL4ZyuqEwDKSxZtGUk1mowbQJJwIVgFI2TuwgkirTVXnn1YP1+rENUR6\/KO0Dy+5ZG0l5lxdviHK6iTq4PJNI2sdEMFmIcT9LptDUhBptanK\/F1eqDxosjqJX6pAGWnFfKwaJmYwWb1kSatsfXB8WZ1EQvXFF+MwJoJV8Dzfc0Oy8\/fJoWV1Ekk56ZNBCu5gtKmsXnyJTfbGHW8cLr4QktUpLoLFrzy7vFUwyRmIYBWXiew+PkWx1ponzZ93hLhoz+W9sDI6+UawLOJjn5Dn94KKi5bX32U7qeiEqY+GhobIbK7v7U1bvESw1qiNBiIECAFCwD4C3hw0sA8NjUgIEAKEQDEEiGCL4Ue9CQFCgBBIRIAIlpyDECAECAFDCBDBGgKWxBIChAAhQARLPlAKAnzVMJWKTS5ufGfVp0zu7eV3K\/h8uqoUZytxUCLYEsEva2iVAjIqbVX0ybvf11WCbW1tNb5XlN2UZs6cCbW1tSpwU9uSECCCLQn4ModVIU2Vtio6iaegZPsSwdYDEayst5Tfjgi2fBsYmYFYuJs9hvPFlNnG+K6uLmCb6nEyaW1R7rx586LqUXilPa7yRUP4tvwckh6r+Udivg0S7OnTp6P\/YVFysT8vG8dk9UDZUdEhQ4ZE\/di8+YMdqDf2b25ujkpiJs1BNJh4sxDnmLZZX7xhpN3QKII1slSMCiWCNQpvecL5QiTiwuQXMc4QiyqzqEgsxhLXlhU6TyvcIhYLF4vXpKUIxELWfNv7778\/IsiWlhaoqamJSh+yo5w45qJFi2DZsmXRb3y\/EydORDeRhQsX9hVg5n\/HL0YgDocPH44IFi+8keCxV3wcT5tvHMFisWeexJOOmxLBlrdGbIxMBGsD5RLGEEvp8VNIi5JEIuTbYqTLFw1HmUnHEUXyjSNcvrAzP780MlMhJJz7li1bIsJEghVrIKQVNjlw4ADwedW06DGOYHlCTbsRqehDEWwJC6ngkESwBQF0uTt\/ppx\/lBYJln9MxsdZvFhBbL4tpgVmzZo1QOW4XQAiSaoQbNoNII2QWDTOvmc2YcIEOHXqVCzBxn2Oh5\/zQw891Hcen1c47hMkcQSLfVjRF74oDEbW\/EUE6\/IKKj43ItjiGHohgX+Ubmtr6\/vcDkalfGSXliKIi2CTlC8jgsUbAB8ViymCIhFsmpEpgvViCZQySSLYUmA3P2hcZNTZ2RlFVeJjP+Ym77nnnijXiP34HGdaDpblSmfMmDHgw3I6c7A8WW\/dujUCj0WHYoS9ePHiKD\/LygeynGpcikAlB8teeDGcxJQGn04QMeRvbuLXgfk0BssDo+y48oCUIjC\/bnSPQASrG1FH5Im7CPg32Ywshg8fHj0+44sjfCmD14YNG6J\/s5c7Yltsw+8iSDskkLSLAGVk7YPl3+Bje\/6FURLB8ikCTIngyy7UBdMdeMXVoWXpEWyPemF0H7eLAPsnfaE0KYJFche\/1SWmC3iM2ByefvrpiGDFl3ZEsI4sLoVpEMEqgEVNw0Yg797crBysLtSIYHUhaU8OEaw9rGkkxxCI28qW54sERLCOGdah6RDBOmQMmopdBMQURt4vEoi1CMQ8sQ6tqBaBDhTty\/h\/jigr3ZRsVWEAAAAASUVORK5CYII=","height":207,"width":344}}
%---
