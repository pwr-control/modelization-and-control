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
time_start_motor_control = 0.25;
%[text] ### MOTOR Selection from Library
n_sys = 6;
run('n_sys_generic_1M5W_pmsm'); %[output:921bd9be] %[output:354530e4]
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
kg = Kobs(1) %[output:72bc0c6f]
kw = Kobs(2) %[output:22f5aea8]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:1ea0ba39]
luenberger_l2 = Klo(2) %[output:8af08427]
luenberger_l3 = Klo(3) %[output:2908255d]
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
motorc_m_scale = 2/3*hwdata.inv.udc_nom/ubez;
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
lithium_ion_battery_1 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab); %[output:76440439]
lithium_ion_battery_2 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab); %[output:476701f6]
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
%   data: {"layout":"onright","rightPanelPercent":29}
%---
%[output:4afd7a36]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:9269bdd1]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:0b84da60]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:0c8e3447]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.880000e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:22dbc57b]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:41924656]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:3dfc571b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAX8AAADnCAYAAAD2Blb9AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnX1wV1eZxx9W1oYOVAilreEttAWtjEOpVjGtBqfrolNhXdgtBJ1NM0BZLIujMOSlnQHGkhC2dCzSZinEyE4l1SqzkhlHbKlkFmN3W2j5g0XB0vBS2hUJTGEEV3bZeU44P04u9\/7uub\/79vx+93tnOg3Juefl8zzne88959znDLpy5coVwgUCIAACIJApAoMg\/pmyNxoLAiAAAooAxB+OAAIgAAIZJADxz6DR0WQQAAEQgPjDB0AABEAggwQg\/hk0OpoMAiAAAhB\/+AAIgAAIZJAAxD+DRkeTQQAEQADiDx8AARAAgQwSgPhn0OhoMgiAAAhA\/OEDIAACIJBBAhD\/DBodTQYBEAABiD98AARAAAQySADin0GjF3OTjxw5QnV1dXTq1KlcM2bOnEnr1q2jIUOG+Dbt4sWL1NDQQDU1NTRt2jTf9K+++irNnz9\/QLrFixdTfX09Bc3LtzAkAIEECUD8E4SNosITYPFfuXIlrV+\/niZOnBhYgIMKNot\/a2srtbe3U3l5ea68iooK9QDABQLFSgDiX6yWy2i9\/cTfHKnrETqjYgHfvHkzVVdXK3L8Nx75\/\/CHP6TGxsbc75yC7hR\/Tsh1aG5upieeeEI9hPgtYsqUKeqNoqurS+Wl30b4Z\/17\/t37779PTU1NtH\/\/furp6aHjx4\/TvHnzaPz48QPeMLZv306TJk2i5cuX0+c+9zn69re\/TfzAefLJJ1VbDhw4QC0tLTR37tyMegKaHZYAxD8sQdyfKAG3aR8tguaDYcyYMUp0q6qqlLDq0fuZM2fUtBGLKF+dnZ1qykiLtHM6yE38+\/r6lCh\/61vfoq1btyrxHzt2rMpj9OjRpP9uijyXwYK9YsUK6ujoUOL\/wgsv5N4ouBw9DcUPpN7eXlq0aBEtWLBA\/Z4fStwGTsdvIS+99JJ6eNhOdyVqJBRWFAQg\/kVhJlRSE\/Aa+WuR12LO8\/9aRPW9znn6Y8eO5Ub9Oo35tsC\/sxV\/Fmg9pcSjfx6lt7W1qYcD141H6F4PBb1W4Xxr0eLP9dZvKfxQ4H9zWrOt8BAQCEoA4h+UGNKnSsAp\/lwZLfI8pRNU\/LWYejXKdtqH7+eFYXO6Rr8Z+Im\/fuvg\/\/NIfufOnQNG\/hD\/VF2uZAuH+JesaUuzYflG\/vfcc09uMdh22kdPw5jpzXn0fAu+y5Yty+0cMqeQnNM7enrG6\/fmlJNeO+A3B4z8S9OHpbQK4i\/FEqiHFQG\/rZ5xLPjabPXkxVmen2eB5\/Tnz5+\/biHYbcFXz9nrhWcWfc7nzTffVA+ypUuXqmkeTPtYuQcSBSAA8Q8AC0lBIAwBtymkMPnhXhAIQwDiH4Ye7gUBHwLmVlJOymsCNh+XASwIxE0A4h83YeQPAiAAAgIJQPwFGgVVAgEQAIG4CUD84yaM\/EEABEBAIAGIv0CjoEogAAIgEDcBiH\/chJE\/CIAACAgkAPEXaBRUCQRAAATiJgDxj5sw8gcBEAABgQQg\/gKNgiqBAAiAQNwEIP5xE0b+IAACICCQAMRfoFFQJRAAARCImwDEP27CyB8EQAAEBBKA+As0CqoEAiAAAnETgPjHTRj5gwAIgIBAAhB\/gUZBlUAABEAgbgIQ\/7gJI38QAAEQEEgA4i\/QKKgSCIAACMRNQKT4Ow\/AYAgtLS1knq0aNxjkDwIgAAKlTECU+OvzV92EXj8QcBJSKbsj2gYCIJAUATHi39fXR11dXVRbW5u37du2bfNNkxQ8lAMCIAACxUpAjPgXK0DUGwRAIHsELp\/upYsH99Cw6Q8XbeNFif+RI0eorq6OTp06peb4+WpsbKSKigrq6OigiRMnRg769ttvjzxPZAgCIFCaBG774GX665sv0F+PPK8aeOB8Gf1z7yirxh49etQqXVKJxIj\/xYsXqaGhgWpqamjSpEm0YMECmjdvnlrk5fn+np4eWrduHQ0ZMiRSNiz+UowiqS6RQo4gM7Bxhwgu3s4VFRs9yr90sJvO7\/k+DR5VqUb8wz5fq362uaKqi01ZtmnEiD\/P+a9Zs4ZWrVpF5eXl1NraStXV1TRt2jRy\/s22cTbpJBnl7bffpgkTJthUO3NpwMbd5ODi3RXCsmHRP\/\/LbUrw+RoyeToNm15LZZOnB+5\/knRGVz4T4q93EXGjnTuJJBklrLMG9sgiugFsIP5B3bVQn7l0cA+d39Mv+oWM8t3qKUlnMiP+\/NawfPlyampqUm1ubm6mDRs2qLcLviQZpVBnDdopijE92ED8g\/ptEJ+JcpQP8Q9oKRZpnuc\/cOCA651Tpkyh9vb2nGjbZs+jfp5C4nt5vUCvK\/B0EsTflmL66YJ05PRrm1wNwCXctE8co3yIf3L+n7ckFv\/Ozk61WMwXi39VVVXua+Hb\/u4JmjNnjojaXrhwgYYOHSqiLtIqUYpsTvRdcsV8\/Oy13x\/3SONnn3HlZVRz74ep5t7biH\/O4uX1YHQb5ZdNro5126akGQaR0z5xjfzzif8t\/\/AvigW\/WaR9\/elPf6Ibbrgh7WqILL\/U2FTcNNiTc8Wwa3975NPD89rj5MmTNGbMmFyaU+9fpnfPX6Z9Jy9R128uEP\/7E6PL6LnZt4m0a5yVcrKht16lK\/t+QvT6T4hGjCH65Bwa9Mk5\/T\/HeD3wwAMqdym7CsWJv8met3b29vZSfX29+jX\/m69CYvtg2idGr04wa0xvuMO24TLrmTdo71vnqH7GBKqfYbc1MUHTxlYUsxk7dNB1O3biHuW7NQgjfwszu23rDLPVEwu+FtCLIImNyBVBMyKvoi2X1l291Lrrbbr\/juG089GpkddDWoY8tXP8eyvUKD+qHTth2gjxt6CnP\/Yy5+V5wZa\/+i30Iy9zq6czMJwko9h2ZAuMJZcEbAof+es7ef1g1rNvqH9umncX3X9n\/imlYnMiPZd\/8b\/2EC\/k0h3TaNQXF8c6l2\/LSJLOiJ72ce78KXSnj41hJBkFAudtMbAJL\/46Bz0NtPPrU0viAcCif\/ZHa67bl3\/iwhUxH01K0hnR4m8j2lGlkWQUCBzEP6hfF+ozehqoWNcBnKN8\/uqWv741A60VyiaoDWzSS9IZceKfVkhnSUaR5Kw2Dp1kGrCJbuSvc9r7u3NqGoi3gz5Tc1eS5gxV1uln6qy+vpXkM5J0Rpz4c4XSOMxFklEkOWuo3hnDzWATvfhzjvoBIHkhWAdW45ALPJfvNsp3oyPJZyTpjEjx15VK8hhHSUaR5Kwx6HeoLMEmHvHnXHkh+O4nfq0+BuN1ACkfhbnN5Q+ZXG0dWE2Sz0jSGdHiH0olAt4sySiSnDUgxtiTg0184q8fAEs7DxF\/XZz2TiAOqKZH+bxNc8RDqwrasSPJZyTpTEmKv\/nGYO4QQlTP2LU59gIkdeTYGxuggKi5pLkTyDmXz6If5oqaTZi6QPzD0PO5l08BMyN26m8DVq5cSY8\/\/jiiesbIPomsJXXkJNprW0YcXJLcCRTVKB9z\/rYecy2dmHj+waue\/w4d0G327Nn0ne98B1E9owaccH5xiFzCTYiluLi4dL72Hj3aeSiWnUBh5\/JtQcbFxrZ8Mx1G\/pbUzI+8tm7dSq+88grV1tYGOsOXR\/6VlZU0fvz4vFE9JRlFkrNamiqxZGAT75y\/W+5R7gRy7tgJM5dv63SSfEaSzoid8zfDO3BwNz7KkS8dmdPmDF8zMJxfSGc2Cl+7d++29anY0l0XhTC2koovY7Bxt1ncXDgq6OqX\/6Aiha7+q5tVhNBA19mTdOWlpwdG0rz90yr0QtxX3Gxs64+onpakzCBuW7ZsUeLPB7qb5\/tyVjzHX1dXp2L+zJw5Mxf3R4\/4dQRQRPW0BC88maRRnCRUSXHhhWDbnUDOr2+TGOW72SQpNjb+gJG\/BSW3kX93d7dVYDfz0HddFKJ6WkAvgiSSOrIkXEly8VsIdjsVK8i+\/Ki5JsnGr+4Qfz9CV\/9eSGA3czunLka\/EfDRkPPnz1e\/RlRPSyMISyapI0tCkzQX50Kw81SswbdUXhdjJy1eSbPJ106If1pekKdcSUaR5KzSTAU27hZJg8ueX79JHd9\/gb5+QzeNevdVEfHyMe0TvMeK2eoZ1wHufkgg\/n6EZPw9DZGT0fL8tUiSix7ln31xtRL8fWVTaE15fepfBHsRSpKNn69I0hldVzHib8KL8hjHYjKKJGf145b038EmnZG\/2+LtuGffzlUmzS+C\/XxQks9A\/P2sRURRH+PoV6Qko0hyVj9uSf8dbJIVf+eXtxwn3yvcgt9CcNK+osuT5DOSdEbsyD+OYxyLZSFGkrOm1WGL4RVeEpuofMbtI6whk6fTqEc7rJob5QdhVgVaJIqKjUVRvkkg\/r6I+hMUstvHMuvrkkkyiiRnLZRnXPeBTfQjfy\/BL5tcXVAUTfOMYAmhoSX5jCSdETvyj0s8vPKVZBRJzpq0HfzKA5voxJ+jZ\/75dK86GIUXbnmEX6jgO2vFDwApoaEl+YwknREr\/l67foIc4s5f\/3I0z\/Xr16t4QAjp7Cet8v8uqSNLouXHhefuLx3szok9150FP98cfhTtk7AO4Mcminba5gHxtyXlSMe7fzhA27Rp\/vFA9JrBvn37qKOjg0aOHEnLly9HSOcC2Uu5TVJHlsKE68FcPvzHY0rc+XITev59lKN72\/brD8LSOiJSks9A\/G29xpHObQeQV1b8oDh9+jSx+Dc1NdGZM2eIwz60t7cTB4VraGigmpqa3INEklEkOWuBporttqyyOfujNcRz8\/pikb\/8+\/5\/m7\/XI3r+wvYvR1VaL9TGZrCrGZtHRL75+GfiLm5A\/pJ8RpLOiJ32cfMOMzhbeXm5pwPxdM+2bdtoyZIluQNcWPx1RFC+kcW\/qqqKdOC3o38\/KFGHRGEgYEVgxJiBycrHEI0YnfvdoNunEX1yDkmJXJmvTWZk0K5aR7usYBSWSAobRPW0tJ\/XnH9LS0tOsN2y4umetWvXqrj\/5lSPn\/hLeiJLGqlYmiuxZGDjjrqYuPDhMDwV9EzNXeqQmLgvSWwk6UxRjfzdnMQZ0nnhwoVqxM8hnvVVUVFB3\/zmN+n555\/HtE\/cPS3m\/CV15JibGij7YuOi1wH6nvp8oHYWklgSG4i\/hQWj+MLXDOOMBV8L6EWQRFJHloSrGLkk9UGYJDYQ\/zy9Ru\/S6erqck1lHtji1\/lM8Xdu9URIZz96Mv8uqSNLIlSsXMzvAeJaCJbEBuJv0WuC7OyxyM43iSSjSHJWX3AJJwCb4p\/zd2tBnOsAknxGks6Im\/PXor9s2TJasWIF8QEs5hXkI68guiTJKJKcNQjDJNKCTWmKP7dKfxAW9TqAJJ+RpDPixD8JAXErQ5JRJDlrWvbwKhdsSlf8uWVxrANI8hlJOiNa\/PlDrcbGRoz8pSlwivWR1JFTxHBd0aXEJeq4QJLYQPwteo3e519fX28VzsEiy7xJJBlFkrOG5Rr1\/WBT2iN\/s3VRHRAjyWck6YzYkT8WfCdErZslkZ+kjiwJaKlyiSIwnCQ2EH\/LXsPTPnzpEAyWt5G5XZQ\/8OLAbojqaUtPdjpJHVkSqVLmEnYdQBIbiL9FrwkT0pkDuFVWVqqHBscD6u7upkWLFiGqpwV36UkkdWRJrEqdS5h1AElsIP4x9hqv6SIzKByiesZogJizltSRY25qoOyzwqWQdQBJbCD+Fm7tNfLnW82pHGdWWvz59\/yVsE5rE9iN79m9e7dF7eJNIiUKYbytLCx3sHHnliUuz\/3HOdr8n+do5l1DafVf3ezrSFLYIKqnr6muJeA5\/97eXuIdP3zpNQA+0IXDMz\/99NPX5aYDvT355JNqlxDf09PTQ7Nnz6YdO3bQunXr1D3OkM6SnsiSRioBzJVIUrBxx5w1LkHWASSxkaQz2pMGXbly5UoivdeykHyB3fjr340bNyrxd0b15GMbH3\/8cXWAi17k5TUAjvTZ1taGqJ6W\/KUmk9SRJTHKIhfzgJhN8+6i++8c7moSSWwg\/ha9Ru\/Y4Wkbc+TPo3gWeA7PrH\/vzM5c8NUjf\/OhwOmbm5tpw4YNpA+FkWQUSc5qYapEk4ANRv4mAb0QvPetc57nA0jyGUk6I3bkzxVzzvtzXJ9NmzapA9nNU7ic3cG8z4wFZB7gjqieiWp2ZIVJ6siRNSqCjLLORX8PwIfD8CEx5iWJDcQ\/AmePOgtJRpHkrFFzDpsf2GDk7+VDXgfFS\/IZSTojeuTP0zebN28eYGtE9Qwrn8V9v6SOLIkkuPRbw20dQBIbiL9FrzEPYtm\/fz\/xDp9jx46pO4N+8WtRHEkyiiRntWGXZBqwwcjfz9\/MdYCdX59Koz9wliZMkBEuRZLOiB35m7t9Dh8+nPtKd82aNbRq1arcQq2fI9j+XZJRIHDeVgMbiL9tn9YHxCz+1HBqmTfV9rZY00nSGbHiz7t9eDsnh2XgD7Tq6urUoeyY9onVN8VnDvGH+AdxUq91gCB5RJkW4m9Jk0\/xuvHGG3P79flkLx2kzTIL62SSjAKBw8jf2nGvJoTPeBP78d5D9MiO92hceRnl+x4gKPNC0kvSGbEj\/0LA6nvMD7+8tnq2tLQMWDuQZBR0ZIh\/UP+Hz+T3mQ986MO0tPMQ8fcAvA7g9UFYUO5B00vSmZITf\/1xmP4OgHcM8YWonkHdVGZ6iBymfYJ6pukzeh2gfsYEqp9RGTSr0Okh\/nkQ5gvoxrfZzPmbX\/jqn3m3EP\/c3t5O0qN6SnSQ0F4fUQZg4w4SXLwdzMkmzXUAiXYSE9vHKf7OL3FtNUR\/I6Dv5697ORhcvsButnkjHQiAQHETuHzzR+jC\/Stp+L8tSLwhR48eTbzMfAWKEX+zkm7hHXjkruPxuDXIefavnvaprq7OK\/6irIHKgAAIxE6AvwfgReCsXyLF32kU80AW\/QBwRvVcuHAhPfXUU7mgbfoev6ieWXcAtB8EQCCbBESKvynsbJaZM2eqaRues\/e6nCN\/26ie2TQ7Wg0CIJB1AmLE3ysiZxAD2Wz1LHQtIUg9kBYEQAAEpBMQKf5u0Gx2+0iHjfqBAAiAgBQCYsRfChDUAwRAAASyQADinwUro40gAAIg4CAA8YdLgAAIgEAGCUD8M2h0NBkEQAAEIP7wARAAARDIIAGIfwaNjiaDAAiAAMQfPgACIAACGSQA8c+g0dFkEAABEID4wwdAAARAIIMEIP4ZNDqaDAIgAAIQf\/gACIBAURG4dHBPrr5\/Pt2bt+6Xf3\/Mt22XffLwzcAywahHOyxTJpMs8+LPJ+zgAgEQSIbAjJEX6NYbLtOtH\/yzKvC2D15W\/+ffmf+Oojbv\/c9gq2z++0926awyy5Pob39+MWwWkd4P8b\/9dpJywo7Eo94i9bYQmYGNOzwpXHg0zqPwSwe71f\/5MkfouvaDR\/Wfnzv4lv7\/\/+XVf6vfjaqkwbeMp2HTHw7hKddulcKGaySpLpoQxF+Q+OOQcu8+DzbubNLgcn7P93Mibwq8FvYhk6eryqY9zZEGGy8PhvhH8jyPNhNJRpHkrNFSDp8b2KQn\/iz25\/dsy43kTZEvm1ytRu9lVwU\/vKWjy0GSz0jSGREjfz5tq7GxcYC1W1paaO7cudF5gE9OkowiyVkTM4BlQWCTjPjz4ufFg3vUyJ5FX0\/H8Gg+7ZG8pavkkknyGUk6k6r48\/m68+fPJzeh1w+EQk7c0vnqxun8zd87y5RkFEnOGrSjxZ0ebOITf6fg98+9V9KQj02nIZOrRY7qbfxNks9I0pnUxJ+Pa+zq6qLa2tq89tu2bZtvGmcG\/ODo7e2l+vr63J+4vOXLl1NTU5P6XXNzc+6Qd\/63JKNIclabzpVkGrCJXvxZ9M\/\/chudfXG1WmwtxtF9Ph+U5DOSdCY18Y9TMFpbW6mysnLAtBGP+vn37e3t6gD4hoYGqqmpoWnTpqmqSDKKJGeN006F5A020Ym\/OYfPoj\/u2bcLMYn4eyT5jCSdSV38zQPbnV5UUVFBHR0dNHHiRGsHc+anz\/w9fPgwdXZ20rp161ReLP5VVVW5B4Qko0hyVmvwCSUEm3DiX+qjfDc6knxGks6kLv5cAec0Df+br\/HjxyvBfvrppwuWFs6rp6eHZs+eTTt27Mgr\/lzI7t27Cy4rqhtPnjxJY8aMiSq7ksoHbNzN6cvl9Z\/QlX0\/IXrrVaIRY4g+OYcGfeEbJeUbXo3xZZMQhQceeECVJOV7otTFn0fqa9asoVWrVlF5ebmqj\/7dsmXLaOPGjaHEX0\/3LFmyhNra2jDtk5Cjx1WMpFFcXG0sJF8vLjy1c\/ZHa4hH\/KU8tZOPmSSfwcjfsNTFixfVFAxP8egFWj1aX7lyJT3\/\/PMDFm79OobzYcLz\/HwtWrQIC75+8Irg75I6siRcJpcsTu1A\/Av3xlS\/8HWbp9+0aROtX79+wLy8bfPMLZ16zp\/fKszfO7eQSnoiQ+C8LQ027myYy9ihg9Qon0f7PMrn8AgjHlpl221KNp0kn5GkM6lP++hpngULFtCBAwdo69at9Morr6jtnUEWesN6riSjSHLWsFyjvh9sBhLNjfL3\/1zN52d1agcj\/8J7Wmojfz3twztveG9+dXW1aoXemcPbMpO4IP5JUA5fBsS\/n6GOq6NH+Zfv\/hu6\/ZHvhAdcgjlI8hlJOpP6yN+co9+yZYsS\/0mTJl23CBy3T0oyiiRnjZt70Pyzysbt61vzY6yscrHxH0lsJOlM6uLvNvLv7u6mU6dOqW2ZGPnbuHd20kjqyHFTd0bN1F\/fchA1Z7jjLHEJyl0SG4i\/w3peH2bprZ9Bje2VHrF9oiKZXj6SOnKUFJxx8HWIZNtwC6XKJQrGkthA\/KOwaMA8ENsnIDChySV15KCI9NQN36cPO7n8+161B58vHUiNQyMPm14bKJBaMXMJyjFoeklsIP5XP+TSO3zcjGlu0QxqbLf0iO0TBcX085DUkU0aOuwxnxXLYq5PsTLFXac3RT6qOPhSuaTvMUSS2ED8HR7hFd4hynj+LP6I7SOhK4arg1dH5v3tzsvvQO58h36zaOvLLx9T1NUI\/pbK3LGELPRJ7LWXJHDhLBz93ZLYQPwN++YL72CGfAjrEn7if\/TvB4UtAvfHRODU4NsKzvndD3jfe2rwrZ75vnu1zH13Lc6lqbjp2gHfFcP6f37k08MLrluUN0qJXxNlm6LKSwobxPZxWNTc7aNH+hySIerdPpj2iaorpZtPEqO41l3XRv3H+y7mGnyi71Lu5+Nn+38+bvzOSWZceRmNG1FGY\/n\/5f3fq9TP6D+wPOorCS5R1zmp\/CSxwcjfYfUkdvtgwTeprhZvOZI6sldLO197L\/dQ4IcHPzT4YeF8UPDD4b47hqsHA\/\/\/\/jsLf4soBi7xeoZ37pLYQPxT8gLE9kkJfITFSurIYZvFbxi\/+t1Z2vvWuVxW\/ECouffDgd8QSolLWK7O+yWxgfhf3e0T1zGOhTiPJKNIctZCWMZ5Tymz0dNNrbv6T9TSD4Kae29TP+e7SplLWH+SxEaSzmiuqcT2iesA90KcRZJRJDlrISzjvCcrbHiKiKePOl97V00X+b0RZIVLIb4liY0knUlV\/HXhvNWzsbFxgF1bWloGnMFbiNGD3CPJKJKcNQjDJNJmkY1+EPAbAT8E6mdMIH4bMK8scrH1N0lsJOmMCPG3NWKc6SQZRZKzxsm8kLyzzMZ8CNx\/x3Da+ejUHMIsc\/HzI0lsJOlMSYq\/ubDLDdRvEYjt49dN5P9dUkdOixY\/BJZ2HlILxTu\/PlXtEgIXb2tIYgPxj7nXOL8Y5uKw1TNm6AllL6kjJ9Rkz2J4gZingvqe+jzEP48xJPkMxD\/mXsMfiVVWVg5YM8BHXjFDTyh7SR05oSbnLWbv787RrGffoE+MLqOXln9GQpXE1UGSz0D8He5hfuQV9hhHrw\/GDh8+jNg+4rpl8ApJ6sjBax\/PHfoB4FwHiKe04stVks9A\/A3\/ifsYR54C6unpodmzZ9OOHTvUATF8NTQ0DDgcno3C1+7du1P3bimxSFIH4VIBsHG3ys\/fPEGP\/fv\/qjeA52YXHgtJos3D1kmKzyC2j8uof82aNcRB3Ao5xvHIkSNUV1enYgHNnDnzutO\/9HTPkiVLqK2tjdrb29XpYCz+NTU1NG3aNFUjSU9kSSOVsB0v6vvBxp0oc3nnf0eoKSC9CBw1+2LNT5LPSNIZbc9UPvLiwqM+xtEZJZTn\/\/latGgRLV++nJqamtS\/m5ubacOGDaRPC5NkFEnOKq3Dg423+E+YMIH0IvCbj3\/G96tgabaNqz6SfEaSzqQu\/lyBqAO7mVs6zUNhENsnru6VXL6SOnJyrfYvyeQy65k31A3mdwD+OZRuCkk+A\/F3+Jnemsmjc3261\/bt23NTMkm4pSSjSHLWJNgHKQNs8o\/89V\/Lv\/VLeqbmruu+BA7CulTSSvIZSTqT+sifp33Wrl1LtbW1tH\/\/furt7VWLs9u2baPHHntMzc8ncUkyiiRnTYJ9kDLAxk78Mf1zjZMkn5GkM6mLvzlHzwu+vD\/\/C1\/4AulFYD0nH0QgCkkrySiSnLUQlnHeAzZ24s+pePqHD5LhN4AsX5J8RpLOpC7+euT\/5S9\/mTZv3pxbkMXIf0KW+6tn2yV1ZEkGcuOi9\/\/zF8BZviT5DMTf4Yl6IXbx4sUDduVMnDjRymed4RzMBWRz+ydi+1jhFJ1IUkeWBMqLy6Odh+hXb50j3v2T1UuSz0D8I\/RC3srJbwz84Kivr1c56\/AOs2bNyu3nnzRpErZ6Rsg9rawkdeS0GLiVm49L1hd\/JfkMxN\/hvVrAzV+bWzS9OhmP+MePH0\/d3d0qCYu\/HvXzz\/wBl34rqK6uVg8FfOQlSbKC10VSRw5e+\/juyMcl64u\/knwG4m\/0ATPaJu\/2YTE\/duyYSjF37lyr3qI\/5NLirz\/m4mmjIOEdjh49alVe3IkkOWvcbQ2aP9i4E\/PjcvcTv1aHxGdx8dePTVAfDJMe4u8Qf72zh4OrrLyyAAALhElEQVSv8Sie9\/sH2e0TlfhztRDbJ4xrx3+vlDgt8bc0WAl+XPa9c4ke2fGeivvD8X+ydPmxSYoFYvs4SPNun40bNyrBP3PmTC5Oj3PaR4eB4EPfKyoqqKOjg\/SCsFP8+UMxTPsk5dLJliNpFJdsy\/OXZsMlq1\/+2rBJypYY+TtIHzhwgG688UYl5rwjZ8WKFQPE3c8wpvhzWiz4+hEr3r9L6siSKNpy4cVfPgO4fkalpOrHWhdbNrFW4mrmEP+IKTvF39zqae4CQmyfiMGnkJ2kjpxC8z2LDMKFHwBZivwZhE3cNoX4OwjzomxjY+OA39rs9onSUJKMIslZo2QcRV5g404xCBee\/jl+9lJm9v4HYROFj+bLQ5LO6HqmFtLZuTUzbvhe+UsyiiRnTcseXuWCTXjx5xx498+4EWWZiPwpyWck6YwI8Q+ysycuMZJkFEnOGhfvQvMFm2jEX4d+yML8vySfkaQzqYs\/V4Cnffiy3ddfqHAUy+uYJGeNg3WYPMEmGvHnXDpfe484\/EOph36W5DMQf5cDXJwuHWTO3xnbx1zY5XxbWlrUgwWxfcLIrox7JXVkGUT6a1Eolyw8AAplE4d9If4RUnWL7eN8GHBx5pfE\/G8c4xihERLMSlJHTrDZvkWF5cI7gGruva0kvwAOy8YXfoAEEP+rsPwOX\/dj6hbbh+\/R+\/zNaSR9kDti+\/hRlf13SR1ZEqmwXPQawLjyMto07y66\/87hkpoXqi5h2YQq3HEzxN84uL2mpkYFYHMTbFvobl\/48odjfOnpIw4d0dnZSevWrVO\/b2hooKqqqtw6AxuFL4R3sKWeTjopn+qn03rvUqPgcur9y7T65T8Qh4LY90+l8xFYFGyisDfCO1ylaJ7gxad18VvArl27aOnSpYE5Oz\/yMjNAYLfAOEXfIGkUJwlUlFz0WwC3rxR2A0XJJqzNMfK\/OgdvbvFk8c93epdtbB+ncfR0z5IlS6itrQ0hncN6b8r3S+rIKaMYUHwcXHQoaC6Ip4OK9UCYONgUanuIfwHinw+2c9rHfKjov3HgOB3qmfPCgm+h7pvufZI6crokBpYeJ5fjfZfUttDWXW+rQu+\/Yzjdd+cIFSK6GNYG4mQT1Acg\/hFv9XRO+5hbOs0to4jtE9RV5aWX1JEl0UmCi34IcLv1g4DfCPhLYX4YSA0WlwQbW1+A+NuSSjCdJKNIctYETWBVFNi4Y0qLC08N8dX52rvEDwd9mQ+F\/rWD9BaQ02LjZilJOqPrl1psH6sen0AiSUaRVJcE0AcqAmzccUniwlNE\/CA43neRTvRdor1vnXOtND8g+OI3h7H65\/IhkT8oJLGRVBeI\/1UCeqtnICVCYhAAgYIIXPro3+Tu+78bR9L\/3Xiz+jf\/3P\/\/\/n9Hdf3FH\/\/gm9Vf\/PGMb5ooEvz+X\/8ximwiyyPzI\/\/ISCIjEACB2AnwdlTndeLstWkn59\/MKSmvyvGbShKXtHOUIf5JWB1lgAAIgIAwAhB\/YQZBdUAABEAgCQIQ\/yQoF1gGf+D24osv0m9\/+1tauHAhTZgwocCcSu+2s2fP0g9+8APiT\/jnzZtHd999d+k1MmSLdu7cSWPHjqWpU6eGzKl0br98+TL9+Mc\/ptdff52++tWvZpoNxF+wX\/\/iF7+gkSNHqgPuWegefvhhGjJkiOAaJ1c1Dt\/x0Y9+VP333HPP0de+9jUaMWJEchUQXhLHtNqyZQvNmTNHxdDC1U\/g5ZdfphtuuIE+9alP0c9+9jP64he\/mNk+BfFPoVeYYaZZ2PkyzzPevn276rAsal\/60pfU6I1DYMycOZM4HlIpX7ZsNAN+A\/je975HixcvpqFDh5YymgHhyfP5zYULF+inP\/0p3XrrrYpJFsTf1m\/YV1j8Dx48qAYMH\/vYx0raZ\/I1DuKfsOl1OGsutqOjQ43q+Xc67IQZhZSjkbLgjxo1KhPiH4QNvwGx8G\/cuJE4QuykSZMStmSyxdmy4QOM9uzZQx\/5yEfo9OnTqpKlLv62bDiy77PPPkuf\/exn6eMf\/zht3ryZamtrM\/vGCPFPsA\/z6IRfxflVc\/Xq1bR+\/Xol\/joCKTsnz\/PrWEQ8OuGRSWVlpRJ\/PqfgpptuSrDGyRUVlM2HPvQhxZKnwkaPHp1cRVMoKQibFStW0KFDh+jEiRP0zjvvKGH7xje+UbJvRUHYNDU10d69e+m+++5T\/Y4HX1\/5yldK\/m3ay2Uh\/il0Zh6prFy5coD49\/b2Un19vXq1X7BggfqZp3t46oc7MDvrgw8+mEJtky3Shg2ze+ONN5S43XzzzcQPgoceeqhkBU5bwIYN+40e6XNMqyyM\/LmNtmx4KoxFn9+my8rKqK6ujgYPHpyskwspDeKfgiFsHbXUX9fd0IONt0OCDdhEKVcQ\/yhpWubl1ol7enrUaWPmtI9e1LPMtiSSgU0wgYPf9POC3wTv\/hD\/4MxC3+F0VK8F3yxu6wQbe\/GH31xjBb8JLksQ\/+DMQt\/hdFTOUG\/1rKioyO0CCl1QEWYANvbiD7\/xFn+w8e\/8EH9\/RkgBAiAAAiVHAOJfciZFg0AABEDAnwDE358RUoAACIBAyRGA+JecSdEgEAABEPAnAPH3Z4QUIAACIFByBCD+JWdSNAgEQAAE\/AlA\/P0ZIQUIgAAIlBwBiH\/JmRQNAgEQAAF\/AhB\/f0ZIAQIgAAIlRwDiX3ImRYNAAARAwJ8AxN+fEVKAAAiAQMkRgPiXnElLr0Ec6bShoYG6uroGNI6PbuT49cV8cSyjXbt2qTMcuI1VVVXq0B6+dLv5pDK38N78dz7JbNGiRZk9kKSYbZ923SH+aVsA5fsS0CJoCqPvTUWQwBRvjuAaVPy5ifrhsXTp0iJoMaooiQDEX5I1UBdXAvnEXx+Befz4cZo3bx7dc8896nSmU6dO0ZQpU6i9vV2NivlUq\/nz5xNHTZ0+fToNGzZMjZj1qWk8sua89IlqOsoqV0i\/YXAefO4rX93d3ep8ZT6DgYW7tbU197ft27erNHwGM\/+dLxZ25wie8zt27Jga6bu10Rz5c3m6bM7PLHvTpk00Y8YMddobLhCwJQDxtyWFdKkRcJv20cL+0ksv0QsvvKBEni99\/rE+G5nF3BR5vo+FmB8CXuJfXV3tKtycP5+Ry8cAjhw5Mvfg4N+z+HMdzpw5Q83NzfTYY4\/Rd7\/7XVq1alXudxs2bBgwPcP3cFn84PGa2uK8+WFiTvuY9\/Hf+UHFl54uSs1QKLioCED8i8pc2ayszcifR9gnT57Mjfo1KRb7JUuWUFtbW+4tQI\/wvcS\/srKSGhsbB8Dm0T8LtRZ5PU3Do3keves3BvMmLdL6TcFcn+A2rV27lmpra9WI3W\/kr8XfKfycN79B8JtBsa9\/ZNO702s1xD899ijZkkAQ8edRt3OEzeKoRZungGzE303MzXxsxF+LMjdTj\/B1kwsRf68RPsTf0pGQbAABiD8cQjwBW\/HndDyHz3P\/PAWi1wNWrlxJvCDKI2Nz2mfZsmW5RdZZs2blpoNYqPX0zpgxY3Jpxo8f7zryd077cHnr168nvpd34\/zmN7+57oGk73FO+3jt9uG3C6+pHUz7iHdhkRWE+Is0CyplErAVfx6N8+4X2wVffhiYx2fqhWDz91wPc8HXbdpHLxbrqaKWlpbc\/Lu5iOy0qjlizzft8+CDD6ppqwMHDuSy0Gse3GZz+gieAwK2BCD+tqSQrmQI5BPkKBuZxD59bPWM0mLZygviny17o7VXd8e4zelHCaevr09NQY0bNy63HdQt\/zDz9c51gyjrj7xKnwDEv\/RtjBaCAAiAwHUEIP5wChAAARDIIAGIfwaNjiaDAAiAAMQfPgACIAACGSQA8c+g0dFkEAABEID4wwdAAARAIIMEIP4ZNDqaDAIgAAIQf\/gACIAACGSQAMQ\/g0ZHk0EABEAA4g8fAAEQAIEMEvh\/Azl\/O21\/mnIAAAAASUVORK5CYII=","height":231,"width":383}}
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
%[output:921bd9be]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:354530e4]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:72bc0c6f]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:22f5aea8]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:1ea0ba39]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:8af08427]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714302e+02"}}
%---
%[output:2908255d]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.994503273143434e+02"}}
%---
%[output:76440439]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAX8AAADnCAYAAAD2Blb9AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnXuQV8WVx4+JMUx8BMcXjhMc1AHJuouPxUyIxhhWjdkaTGWNM8NuSSGy7AZNVQB5rmWoEpiZDKkYRUNwZEltGKhKihX+WAlFDD7QyIKZ3TJGiTDBcTRBHhoTXBNl61zsHz0999F9f7fv7dvn3KpUHH73dvf5dvenzz3dt\/uEo0ePHgW+WAFWgBVgBUgpcALDn1R9s7GsACvACgQKMPy5IbACrAArQFABhj\/BSmeTWQFWgBVg+BNrAwcPHoRp06YFVnd3d0Ntba0VBdavXw8LFiyAZcuWQUtLS5AH\/hte4u8sMw6z68iRI7BkyRKYMmUKNDY2ZpldZFq7d++GqVOnwp133qllZ5oyPvvss7Bt2zaYN2+eFZuElr29vUH6a9euhaamJu28wupe+2FLN6LO8+fPh7q6Omu6WSq6tWQZ\/tak5YRlBWwDQYV\/TU1N0Nl37twJq1evzg3+HR0dgHDWGVgFkEzKiGlPnjwZZsyYYQ1iIg954DZpzbbr2qQsahu8\/\/77c20Pacuax3MM\/wxUxg6\/cuXKICX0LGTYiI4wa9Ys2Lp1K6A3pd6jelpyxxae5BVXXAGnnnpq4IXhldQxRb5qmVRIHjhwIPBUhWeMHqVIWzcNfHtQgSEDAMuAbwHiam5uhvb2dkBA4yUguG\/fvgo0w8AotBgYGAiek3WS7XrggQegs7MTNm3aVMkTbZo0aVIwIKj\/Lr+JiLrEOurq6gL8e+TIkZXyqmWQ60H8hvYJr1ytW1H39fX1kWWRdUcDhF7YdhD84ho3blygF174Nic89aSBQWgrdBDpYD2qecu\/yV0lrs3Kdd\/X1xf0DbXNi\/ai2oJlEDqqbfK6666r2Ima3HTTTXD77bcPebsUbU3NM6x+Muj+pU2C4V9l1cngl5MSr8pqZ0rquOJ3ARUVNuJ3tWGrHo4MW3kAOOOMMwaFfQT8BVBFurt27RoE7Lg0qoU\/pi10ErrJgx4OFP39\/cEgJcqpDiQINBHOioK\/AJGslaxjGPj2798POPDGlUGta1F3KmTluo8q4\/nnnz8I8HJ7UH9DMH\/729+Gu+66qwJ+tf2ozTuqTFH1HgZ\/FfwiDzHoRLV5MYhF1aV4Xm3zWLaHHnoIHn744UED99VXXw1PPvlkqLMSNqjkFfKsEim5Pc7wr0Jq0UjPOuusiscqPBrR0Ddu3BhAVPyN2QnvU3jx6M2pwBBesIAzPodvFLLHGBaLFQ0coSXeQGRPTHhPmB56jWr66G2ZppEEf\/Ssk0IBqlem3i8G2TCwog6jR48eNKjphH1EmvLz6D2rMFfrUvwudBJvBt\/73vcCL1f8HvZGIzc3nbCPGuaJ+juq\/ahzOmr7RJ2E1iq8o94u1ftVqG7ZsmVQm5cH5rBwWNRAL9o8tklRbjEYifrFtxf5rU5+ewwLX2Gd4zN5hgKrQIzVRxn+VcgbBjS1w4uOIINabpSYveqly142\/jd6vML7lDtrGPzVjiRCK8LMqLCPnL5pGlnAX9ZNeMWiI2PZwyapZR3VQS0O\/qpnijriG5GqcxTc1SaDQBJlVuP3USEcLF8c\/KNCXCr8o7zsqDdDecATk7iqncJhiYJ\/WBphb55JA5L6BqG+GYS1eblMYfUvQl9yeeQwWFLZq8BB6R5l+FdRZWGeRRT8oxptFPzx36OgpIZIZBNMwS08\/2rhrw6ESX+HyS6eufvuu4O3EhE7j\/KgTeGvdnz572rgL4cloiZvw+aFxFuc\/Iyup58UYhHtR12lE9Z28oa\/2uZEGEgNr2UFf3mOieF\/vOcx\/KuAv42wj1qcMJjHwV\/2psSbgQyU6dOnh8b85Y6mm4YILcmhKHWyOOrvMNlVb1d+s6k27KPOdchhg7RhH9MQDt4vD4piAlqGvwonNcSSFPZJas5Zhn3UUKawQ8wXRXn+4m1Y\/K6WSR0MsK7ShH3CtGD4M\/yT+oj277YmfHVegaPWX4eFAkQYIGrCV4a\/DClZiLiVKuK+JPjjfVErSPA3oad6T9TEt9BJjSvLcMd0b7311mBSNCwsEDU5j2XQmfAVXrgKlqiJ0SgdMR28xMqxsNCFvEoG07nvvvvg3nvvHWKXuqJKpJU04Yvx9aT5Gd0J3yT4q50srs2HlVtnwld9A+KYP8NfG+46N2a91FMGn6nnL8obFtfGEIBOzD8pDfxdhjGWF98o7rjjjiErLwQAZGDEwT9uHbvuUk8xqSiDEsF6zTXXVFbSIGhuu+02mDlzZiW8pA4+uNRzzpw5sUs9ZciGhQHDQBk2\/4N5YxnFm5lYErxixQp45JFHQMx\/yIOaOqCLgS1OX8wnbqmn+nYS9UFeVLxenpOKgr86MKMeuMRYTMRiGdT5F\/w3OU+5PtUPCeU5NPk3Nbylzofp9HWf7ik07IOdee7cucGa7LAvMNW1wHHLG12tlCQvytVyUyuXDER1ma36VhSljYALDrK2vr6lVi\/CXjHw499hq9h0vhrndf6DW09h8NdZ5obgxLXZZe5IDP\/y4CoqhJf0QZ1sockXvuVRpviSJoXQdLbvwL7IX\/g6EPZBrx47Cl5Rnj\/+3tDQoLVHSvHNM7wEDH9Xa2ZoucLiyklfy6qpCO8SQ0Ym++GUR6XiSho276O77xDv7TO03grx\/HEUX7x4MbS1tQUDQBj8RWVNmDCh1PAvrqtwzqwAK8AKRCtQCPzRG8br8ssvj4z5h73mmbx+c6WzAqwAK8AKOAR\/fHVbs2YNLFq0KNivJWrCV3195tdpbsasACvACmSnQO6eP4Z5cMkdxkOTVvuoZoo5grAJ4AsuuCA7VTglVoAVYAUyVAB39B01alSGKVafVK7wj5qxRzN0Jm7iJoAR\/nv27KlekRKkwLaWoJJSFpHrNqVwjj\/mYr3mCn+1fuI8f7EaSP5gBz+4idqNz0VxbbVHttWWssWny3VbfB3YKIGL9eoU\/BH4PT09lYMz1I+84t4OXBTXRiPCNPfu3evcKyTbmo0CXLfZ6OhaKi7yqVD4Z1lBLoqbpX1yWgwIW8oWny7XbfF1YKMELvKJ4W+jpi2nyYCwLHCByXPdFii+xawZ\/sTEtWUuA8KWssWny3VbfB3YKAHD34aqH6bpori2zGVA2FK2+HS5bouvAxslcJFPHPaxUdOW02RAWBa4wOS5bgsU32LWDH9i4toylwFhS9ni0+W6Lb4ObJSA4W9DVQ77WFS1+KQpwRDVpmQvJVsZ\/hZZ4qK4tsyl1Gko2crwt9Vjik\/XRT5xzL\/4dmFcAkpApGQrw9+4K5TmAYa\/xapyUVxb5lICIiVbGf62ekzx6brIJ\/b8i28XxiWgBERKtjL8jbtCaR5g+FusKhfFtWUuJSBSspXhb6vHFJ+ui3xiz7\/4dmFcAkpApGQrw9+4K5TmAYa\/xapyUVxb5lICIiVbGf62ekzx6brIJ23PP+4glihpx40bBxs2bMhFeRfFtWU4JSBSspXhb6vHFJ+ui3wygv\/s2bNh4cKF0NjYmKgmHtSydOnS4PCVPC4XxbVlNyUgUrKV4W+rxxSfrot80oZ\/8fLFl8BFcW1pRgmIlGxl+NvqMcWn6yKftOEvwj4oozhasXhJj5fARXFt6UMJiJRsZfjb6jHFp+sin7Thj\/IdOXIE5s+fD5s2bQrU1Dl0PS\/ZXRTXlu2UgEjJVoa\/rR5TfLou8skI\/rKE8vm6zc3NlXN3i5LZRXFtaUEJiJRsZfjb6jHFp+sin1LDX8gpvw3U1dUFE7w6E8JZV4eL4mZto0iPEhAp2crwt9Vjik\/XRT5VDX9ZVrHCZ\/ny5VBbW5ur4i6Ka0sASkCkZCvD31aPKT5dF\/lUNfzZ88+\/YVECIiVbGf7596W8cvQK\/hzzz6vZDM2HEhAp2crwL65P2c659PBXV\/ssW7YMWlpabOumlb6L4moVPMVNlIBIyVaGf4rOUIJHnvrNYfhK1xZ48\/tfc6q02mEfsc5\/\/\/79hU3qxinH8HeqXWVWGIZ\/ZlI6lxCVuu3Z8QbM7HkRDn7nWqfqQBv+TpU6pDAMf9drKF35qABCqEPJXiq2lh7+6Pnz3j7pAJb1U1Q6DbUwCDV7qbRjL+A\/bdo06O3t1WYZ7+qpLZXRjVQ6DTUYUrOXSjvu2NwHHZv3ctjHiHIGN3PYx0CsEt1KBRAc9ilRozQsKsPfUDDT2xn+poqV436GfznqKU0pqdQtwz9N6zB4huFvIFaJbqUCCPb8S9QoDYvK8DcUzPR2hr+pYuW4n+FfjnpKU0oqdcvwT9M6pGc6OjqCv+bNmxeaEsO\/SoEdfZwKINjzd7QBZlAsXOOPK368Wee\/fv16WLBgQSAN7uuPV09Pj5WtncVWEjNmzGD4AwAlIFKyFfsQJXup2OoV\/NELHxgYgHvuuQcWL14MbW1t0NTUBEneeZpBFLeUWLJkCbzwwgtBHuz5MyDStKOyPEMFiJQGupt\/0As\/+\/XB8nv+8sde9fX1wcleAv42tnTGNwy8+vr6OOzzIcEYEGVBuXk5uW7NNXP9iUkrnoenXjnsN\/wxPIPef1Zn\/OJAg28W+IaxatUqhj\/D3\/V+XnX5GP5VS+hcApfe+wzsO\/hu+eGPyqI3vn379kFhn9GjRwN+Adza2prZTp84kFxzzTVaISWc8MVr69atzlV+1gXq7+8HfOuicFGyFeuTkr0UbL22uQXevv7YYhVvJnzl\/fwFhLLc4hlDSGvWrIFFixZBTU1N4nwCr\/bxcyig5AlTioNTsRW3c5704PN+wd82auTVRHJeeFj8fffdNyR7hr\/tGikmfYZ\/MbrnkSuFumX4Z9CSklYSMfwzENnBJCgAQpadkr0UbBWTvR\/505vlPcxFNFBxqEvS7p5xa\/LTMIbhf1w1Cp1GWEvJViqhEEp1Wzvr8cDcE998CX7\/w39Jgz5rz6Q6zAVDMuvWrRu0qkcMCmLCNwnWWVvEnn\/WirqRHsPfjXqwUQrf61Z4\/ajdKU91wr7n\/suGjKnTNIa\/gDx+bIUfXcmXvNTzwIEDsHTp0uDIxzwuhn8eKuefh++AUBWlZK\/Ptsqx\/pG1w+DtR\/4R9uzZk38HismR4e9UdegVxudOQxmGHPbRa\/+u34Vr+nFtP14I\/gdax8Kt119efvijQTphH\/EtQNjKHBuVx56\/DVWLT5PSQMfwL769VVsCGfyYFoJ\/8pUjwEU+GXv+Qpywdf64wRuGgvC3OXPmBCGfxsbGavXUet5FcbUKnuImSkCkZCvDP0VncOgROdSDxZp3wyiYd0NDUEIX+ZQa\/g5p7qy4tjSiBERKtjL8bfUY++mKLRxETo9943K4suGTlYwZ\/hbrwEVxbZlLCYiUbGX42+ox9tKVV\/RgLiLGf9VFwwdl6iKfUnn+YSEfYem4ceMy29jNpMpcFNek\/Cb3UgIiJVsZ\/ia9oLh7Ma5\/R8+LwU6d8tU2fgSsaBsbWjAX+WQMf9xfH7dxnjBhAkyaNKmypbONjd1MqtdFcU3Kb3IvJSBSspXhb9IL8r0X4\/k9O14PTuRSL\/T2N379ssDrj7pc5JMx\/OX9\/HEyFz\/mamhoCHbyxDcCW6d5JVW1i+ImlTnt75SASMlWhn\/aHmHnuSgPX4R3rmk8He5ruVgrcxf5VDX8cUknHrSCH33ZOMxFS1lHZ9N1y256HyUgUrKV4W\/aE7K9H2Hf\/XQ\/PNq7P9h\/P+yKiuknlcQL+KOR8tYNsre\/cePGYJ\/\/9vb2YBvmPC8XxbVlPyUgUrKV4W+rx4Sni4Dv2Lw3NJQjnkDYjzx9GDzQNjY2rJNUchf5ZOz5o5HqFg84GKxcuRLq6upyXdsvC+6iuEkNIu3vlIBIyVaGf9oekfwcgv7Bba\/CrwbeGTJRqz6NwP\/aFSNg0Y2jkhPWvMNFPqWCv6a9ud7mori2BKAEREq2Mvyr7zEiXIMe\/asH300EPeaIsP+nz5wLt1wxIihA3MRt2hK6yCdj+KsTvrIYHPNP2zTMnqMEREq2Mvz1+0EayAuwf+7C4cHXtzYgH2WB9\/DP+gB3\/abg5ufTJuU3uZcSECnZyvAf2gsQ8m++8x58a9MrsO\/Qu5ETsWH9B+F+1UWnw9zrG3IFfVhZSg3\/uA+7ZGOzPsRFF4ouiqtbdtP7KAGRkq1U4Y+Af\/2t\/4MfPjugHaqR+4yYlP1W84Vw5iknFQ567+AvDIoL+5hCLMv7Gf5ZqulOWgx\/d+oiTUnkJZP3P74PXnrjj8YevByHx5ANxuZHnVnjJOS9DvukaQB5PMPwz0Pl\/PNg+OevuUmOMtzXPDsAO\/a+lQruMuBxaeWUz9bBOad9HNQ9ckzK5tK9LvJJe8JX9+xe3tvHfpOjBERKtroY9sFtDTCsgnH3f39mAPrePJIa7qoHf9NFH4GLLzy\/VB582t5davinNTqv51wU15btlIBIyda84Y9eO\/7vnNNOgke2vwYvvPZOVWBXvfc7vzgSxpxzcqUbqKtrKNWti3zS9vxtgSyrdF0UNyvb1HQodRpKtmYFfzkU81zfW\/Dzlw7BvoPVeeyiDYrJ1ZG1NfDPV58Hwz\/xseCnNMsmKdWti3xKDX\/c02fBggWDuLRs2bJgg7ciLhfFtaUDpU5DyVYd+Atv\/dxPngRrnhmAX776h6CZqdsLp2l7AuAYc7\/5inPgC6NrI732NOlTdmJc5FMq+Ouc4ZtF4zBJw0VxTcpvci8lIFKxVcTWd+\/dB1v2nRBsQ5AV1GXP\/OqLTofPjPokfL7x9IrHjgNKGs\/dpM2G3UulbtF2F\/lkDH91Xx+5Uvkjr2q7g97zlDpNmW0dEn55+RDsO5BN+EUOwwRwP30YjD33FJh+9Xlw0kc\/kjoUo9cCs7mrzHVrqgDD31Qxg\/tdFNeg+Ea3Uuo0LtoqoP7eXz6AHz33Ouz87dtB\/Zl+gRpX6XII5tN1p8BXLzsbRpz28cK9daOGmnCzi3WbpX1yWi7yydjzR4M47GOrieilS6nT5GWrAPr7HxyFnzz\/O3ji5UOZA10Ov6CnftHZn4AZn6+Hmo99tFLx77\/1Oowald1uknotqpi78qrbYqwbnKs38BcDAE\/4FtOsKHWaamwVQD\/0pz\/Dhl\/+HnZZ8NBVoH+qdhhgXH3ChccP8DaJp1djbzGtMX2ulGz1Av4i5t\/a2lrYyp6w5uaiuOm7RfyTlDqNbKscQ8fVLdtfORzE0PHKYrWLrLocdsF\/\/8KYWrj58nMqt5gA3aQdUK1bE43KeK+LfEoV9lE3eVu7di00NTUVWicuimtLEJ8AIZYunnnKx+ChJ16FvfuPwTzL+LmoBxno6KGPGXEyfGXc2daBbtIOfKrbJLsp2eoin1LBX65Ueb0\/n+SV1Nyz+d3VTiN75vjfO3577AOjo0ePWoF5WMjlurFnwOUjT3MK6Ca17mrdmtigey8lW72Ev1zReJwjvhV0d3dDbe3xD0R0G0M197kobjX2xD2bZ6eRJ0If7f09\/OzXB6155irM8e+6kz+A1gkXwAVnHj8T2lbIxVZ9maSbZ92alMvGvZRsdZFPVXv+4vxebBxFbeqGebsoro0Og2mm7TSyZ\/7n9z+AH+\/6HTz9m8NWYa4CHUMtN\/7VmcFKF\/Fb3EdGaW21pb3tdCnZS8lWF\/mUCv7VhHqOHDkC8+fPh02bNgX9KO7wF3VuIS6s5KK4tkARNgn63vsfwLodb8Bze9\/KFebnn1ETTIaOP\/9YqAVBnuU2vJQAUc3Abqut2UyXUt26yCdj+Md94avTUPBNAa958+ZB0sohHGT6+vqCe5MuF8VNKnPU78JD\/+N77wf7t4hP\/W1Mgqqe+SXnnQLXjqkdtBujfE9am9I+RwkQDP+0rcT951zkkzH8s5ZZHgzUtPG3hoYGrSWlLoobppUA+9GjAJ0\/3RscW5cl1OV4OH5INPqck+GWvz1n0NehwkMvQ+yc4Z91j3MnPUp16yKfCoV\/3FuECA9NmDChtPBH0Pf2\/wFWPdlf1Tp0dYliw8l\/hparRlsJs7iDhmMloQQIavZSqluGv0QWMVHc3NwM7e3tUFNzfDUH3hZ2cljcltGuiDtpxfNGoBdgx7NJr\/\/0GXDZp47FzuO8ckqdhpKtDH\/XXI\/syuMKn2SLCvX8sSA4CAwMDAwZAHbv3g1Tp06Frq6u4AMy9W+1WlBcvLZu3ZpdjWmk9ItXj0D3jrdg52vvRt5dd9qJcO6pJ8JnPjUMbhxzSuU+\/Pc0V39\/P9TX16d5tHTPULIVK4eSvRRsnThxYqXP7dmzx6n+Zwx\/9Mhnz54NCxcuhMbGxkHGIKCXLl0Ky5cv117nj8\/MnTsXOjs7h6SnKhU3P5DXyCpi9nf0vBjq4QuPvfOro+HiESdb2SedkjdMyVb2\/J1iY6aFyYtPJoXOFP5p9vM3eSZuAjgPcfHAjTvWvRgsZ5QvBP70q+qh+W\/OsgJ7tUIpAZGSrQx\/E3SV6948+GSqiDb81TX3URnFrdsXYR78f1y+KSZ1cf2+upxTHRTw7zlz5sDq1atD3xBsiouwD\/P028aPgLbx52a6rl2nAikBkZKtDH+d1l\/Oe2zyKa0i2vAXGcSFfXQKoX7kJU\/4IuB7enoq8X+TDeRsiYve\/qQHn6+Yhl7+3118BnzjiyNz8fLDNKUEREq2Mvx1CFLOe2zxqRo1jOFfTWY2n81aXPT2t\/76IMz+8UuDwL\/x65cVBn1REEpApGQrw98mIYpNO2s+ZWGNNvzF0svp06fDqlWroLe3NzT\/ovb3yVrcjs190LF5b2AjevsPtI7NPbwTVcGUgEjJVoZ\/FkhzM42s+ZSFldrwzyIzm2lkJa4a30fwu+Dty9pRAiIlWxn+NglRbNpZ8SlLKxj+ipqqx+8a+BkQWTZ\/99KiNNhRstUL+Id9eSt3oTKHfXp2vAEze16shHpcBD\/D3z1gZ1kiSkCkZKsX8I9q6LiKZ8mSJTBlypTEj7Wy7CwirWrFxXDPpfc+4zz4Gf42Wo87aVICIiVbq+WTjRaaadhHXappo8BRaVYrLoJffLy1om0s4Bp+Vy9KnYaSrTywu9rjqi9XtXyqvgRDU8gU\/mm2d8jKqGrEvesnL0P3068FRfnmxPPh7r8\/tk+QqxclIFKyleHvao+rvlzV8Kn63MNTyBT+UZu02Sq8nG5acdVwzy\/\/7bN5FLeqPCgBkZKtDP+quoXTD6flk02jjOEfN+Ebd8yiTSMw7bTiylswI\/j5gBPbNWWWPsPfTK8y3U2pbtPyyWZ9GsPfZmGqSTuNuPLWDZOvHBF8yFWGi1KnoWQre\/5l6H3pypiGT+ly0n8qFfzVU7biNmjTL0p1d5qKi+Ee3LMH\/9\/FD7ni1KAEREq2MvyrY4DLT5vyKQ9bUsE\/bF\/9ogcAU3HlNf3\/cdtfw5cvOTMPvTPJgxIQKdnK8M+keziZiCmf8jDCGP5ZH+aSlZGm4opYf9m8fgZEVi3GzXQoDXaUbDXlUx6tMxX8p02bFuy\/j8crypfJwSxZG2cirhzrv7WpDr57y5isi2M1PUqdhpKtPLBb7TaFJm7Cp7wKagx\/LNj69eth3bp10N3dXTmuUawCam1thZaWlrzKX8nHRNwyrvCRBaUEREq2Mvxzx0ZuGZrwKa9CpYI\/Fi7sZK9ly5YVAn4sj6648rr+qy4cDhtnXpaX1pnlQwmIlGxl+GfWRZxLSJdPeRY8NfzzLKROXrriyhO9ZVnXr9pPCYiUbGX46\/T0ct6jy6c8rTOGv1jV09bWNiTmn2fB1bx0xZUnesvwNW+YppSASMlWhn+RBLGbty6f7JZicOrG8K\/2DF9bxumKWzvr8aAIt3\/uPOj8h9G2imM1XUpApGQrw99qtyk0cV0+5VlIY\/hj4XDCt6+vL1jx48qlI+7mXx2Atof\/JyhyWUM+DAhXWpydclAa7CjZqsMnOy0qOlVj+Jf5MBcfQj4M\/7y7SL75UQIiJVu9gH++XUE\/tyRx5VU+t004D7puLmfIh+Gv3ybKeCclIFKyNYlPRbRVY8+\/iELq5JkkrvxhV5lDPgx\/ndZQ3nsoAZGSrUl8KqLFasNfhHumT58Oq1atgt7e3tDyunqGry8hH4Z\/Ed0kvzwpAZGSraWGf37NP11OceL68GGXrAqlTkPJVh7Y0\/X9MjzlDfzLtqWzDP95N4yCeTc0lKG9RJaREhAp2crwL3W3jC28N\/Av25bOK5\/shwUbdgeVU\/Z4PwPCX0Bw3fpbt17Av4xbOvsU72dA+AsIrlt\/69Yb+JdpS+dB8f6LhsPGr5dvIze1S1AKhVCyleHP8M9TAe3VPnKhyrSls7zEE8F\/1UXD89TXSl6UgEjJVoa\/le7iRKJeeP5CybJs6fzQtldh0aO\/CYrN8HeiHxgVguFvJFepbqZUt17BP49WJg8wdXV1sHr1amhsbAzNOkpc3+L97B3m0fKKy4MSECnZyvA36FO7d++GuXPnQmdnZwD8sFCTnFyYuL6t7xf2Uuo0lGzlgd0AECW7leFfRYWpg4GaVJi4Psb7GRBVNKISPEppsKNkK8O\/is6Hnv\/27duhvb0dampqhqTE8K9CXIcfpQQIHtgdbohVFo3hn0JA9PinTp0KAwMDsHbt2sjTw8LElQ9qP\/ida1Pk7uYjlIBIyVaGv5v9LYtSeQN\/GciqMLY2dhN5dnV1hQ4AYeJeeu8zgHH\/kbXDgi97fbkoAZGSrQx\/X3roUDu8gL\/Y1wdX3+R9klfYthJCZhQXr61btwb\/P\/D2X6B5TX\/w31ecNwx+8NUR3rSs\/v5+qK+v98aeOEMo2Yo6ULKXgq0TJ06sNO89e\/Y41WeNP\/Iq6gxfdTM5VUV1ZJUne33YzE22l5I3TMlW9vydYmOmhfHK829ra4uMv2ehmrq6B9f8z5kzJ3Ktvypux+Y+6Ni8NyiKD5u5MfyzaFXup0FpsKNkqxfwx+6TtPImqy6mfkVsMuHr62Qve4dZtS4306EEREq2egH\/shzg7uOXvQJXlDoNJVt5YHdzQM6iVF7APwshbKQhi+vrl70Mfxstx600KQ12lGwpQc3TAAANQklEQVRl+FvsZ1Hw922yl71Di43IgaQpAZGSrV7BH+P+CxYsCLoLxuLx6unpifwC13a\/ksWVV\/r4NtnL8LfdkopNnxIQKdnqDfxxvT1+cXvPPffA4sWLQaz8iVuHb7tLyeI+9sKbMLn7f4MsfdnGmVf72G5BbqRPCYiUbPUC\/vI6f\/zQaP78+RX44\/LMpUuXwvLly6G2tjbX3iSL6\/NkL3v+uTar3DOjBERKtnoPf1yaid5\/d3d3ofD3dVsHnvDNncW5Z0gJiJRs9QL+2BvEOn857DN69GjAs31bW1uhpaUl904jxPV9pQ97\/rk3rVwzpARESrZ6A3\/sDa4e4yjDf0XbWGgb78+ePuz558rhQjKjBERKtnoF\/0J6RkymQlxfD3DhCV\/XWpyd8lACIiVbGf52+kuQqhC3Z8cbMLPnRW9X+nDYx2IjciBpSkCkZKtX8JfX+Ys+E7f3ju1+JcRF8OMAgJdPB7iw52+7BbmRPiUgUrLVG\/iHHaYu9vwpesLX92We7Pm7AWlbpaAEREq2egF\/AXk8yKWpqWlQH3BhqadY5nnVhcNh48zLbPXRQtOl1Gko2coDe6Hdymrm3sO\/6I+8fv7fvwKEP163NtXBd28ZY7VCi0qcEhAp2crwL6pH2c\/XC\/ijTGEHq7gQ9vnhT3fBpAefD2rSxw3dRBOlBERKtjL87UO4qBy8gH\/Sfv6yuHiY+4YNG3LRG8WVPX8f9\/Rh+OfSlArNhNJgR8lWL+BfaM+IyRzFnfHQzypHNzL8Xa0ps3JRAgR7\/mZto0x3M\/wt1haK27J8M6x8sj\/IxcetnNnzt9iAHEma0mBHyVav4B+2zn\/ZsmWF7OuD\/RbFvWT2T+CpVw7DyNphAfx9vSh1Gkq2sufva489\/hGqSxaecPTo0aOmBXJ1nf9pt\/0IcG8fhr9pjbp7P8Pf3bqptmSU6tYLz9\/ldf6Hv9IdtEef1\/izd1gtctx+nhIQKdnK8LfY7xouuRLevr4jyMHnZZ4Mf4uNyIGkKQGRkq1ewB\/7h4thH4a\/A+SyUARKgOCB3UIDciRJb+AvBgBxgLvQt8gJ35FX3gjvXDU3KIrPyzwZEI70ZkvFoDTYUbLVK\/hbavupk6378jfh3YsnMfxTK+jmg5QAwQO7m20wi1Ix\/LNQMSKNETffC++N\/Fzwq89r\/BkQFhuRA0lTGuwo2crwt9i5zr71+\/CXM8d4v8yT4W+xETmQNCUgUrKV4W+xczH8LYpbYNKUAMEDe4ENzXLWDH+LAtfOejxI3fc1\/gwIi43IgaQpDXaUbGX4W+xcAv5t40fAiraxFnMqPmlKnYaSrTywF9+3bJWA4W9LWQAQ8P\/Pf70UPt94usWcik+aEhAp2crwL75v2SoBw9+WshL80etH79\/nixIQKdnK8Pe31zL8AUA9DKa5uRna29uhpqZmSM3jiWGTJ0+u\/HtdXR2sXr0aGhsbh9wrPH\/fP\/BiQPgLCK5bf+uWPPyPHDkC8+fPhwkTJgRbP4u\/Eep4ILx64TYSfX19ob+p9wr4+77GnwHhLyC4bv2tW\/LwD6taBPz27dtDvf+Ojg5oaGjQOiOA4e9nx+Gwj5\/1Sm2gY\/iHtOMo+KtvCUldQMD\/4HeuTbq19L9TAiIlW6kBkVLdMvwV7Ir4f2tr6xDvPuyg+LiN4xD+vh\/iIuSj1Gko2crwL71fFmkAw1+SRnj2+E9hE767d++GqVOnQldXFzQ1NYH6d1jMn8IHXgwIfwHBdetv3TL8P6zbJPBHNQGcA8ArbHIYPf8T33wJfrH4Bn9b0IeW9ff3Q319vfd2ooGUbKVmL4W6nThxYqWf7tmzx6k+m+oM32osSFrhE5d23AQwwp\/C173sHVbT+tx\/llKYi5Kt7PkDAAJ8YGAgcm2\/6J64xh\/v7e7uhtraWsC\/58yZE7vO3\/fjG4U2LjYkW1ilZCtqSMlettVWr9FLN1fPP2wSF4s5bty4APIvv\/wy9PT0VAYG9SOvtWvXBvH\/sAs9\/0\/segRO2ve0nuV8FyvACrACOSpAPuyTo9acFSvACrACrECEArl6\/lwLrAArwAqwAm4owPB3ox64FKwAK8AK5KoAwz9XuTkzVoAVYAXcUIDh70Y9cClYAVaAFchVAYZ\/rnJzZqwAK8AKuKFA6eGPG8MtWLAgUDNu7x835NYvBX7jsHLlyuCBuCWu8n1x5x3o51zMnbr2itKZbvxXjFXhueraqi6NjmsHLtmnlkXXXrGFC34HVOa2HFUXJrsU51GfpYY\/Npa5c+dCZ2dnoJX477DDXvIQM6s85A\/c8NsH+WM3OQ91R1T8e926dZUP47Iqj+10dO1VbcdBv2wDvq6t6pfwclsvU\/vWtVcMdLh1C37LU9a2HAd+dOZcaq+lhr8KP9dG1rTQlPcwEhBoa2uL\/MBN5FNWQJjai6CYPXs2HD58GMJ2hE2rex7P6dqKdbl06VJYvnx58IV7WS8Te2XnraxtWa0nMagNHz48+OlLX\/qS1vkkedR3qeGvbvQWt\/FbHmJmkUfUaWfi9LO4PMrYYdLYi\/U8fvx4ePTRRyunwmWhve00TGyNO+TIdjmzSt\/E3jDPP+qQp6zKl0c6aNehQ4eCMJZ8imEeeSflUXr4yyd9mRz7mCRMUb+Hefq6bzS6+yYVZVtYvqb24gC3Zs0amDVrFixevLiU8Jff4qLqVrRl1Exn7selOhVlMa1bcf+mTZtgxowZWse3umh3XDvXceLysonhn5fSmvmYdhiRLMLi\/vvvj9z4TjP73G8zsRfvXbJkCUyZMiXY0to1TypJPBNbxUIGMcmbtLFhUt5F\/G5ir3peh7qxYxHlzzJPFxcolB7+WEFif3+qYZ+ygh\/rziQ0gEDYtm1bUN8udqYkWJjYqoZ92N4kdd3+3cX6KzX81TCPbnjE7WZybNtrEc5KmvD1YVWErr3ykkG5DssUItC1FQc6eYfbpHbgapvWtdeHwS6uDhj+GbdQ6ks9yxgKCGsCussB5Wdd7Ew6zVvXVnUCtKxhEF17w8I+ced36Gjt0j0uttdSe\/5YudQ+8pLfdqI84TJ+DBT1IVDUJL6LnUkXNrq2yh95lfmjJ1175fM7ymwvT\/jq9gS+jxVgBVgBViB3BUrv+eeuGGfICrACrIAHCjD8PahENoEVYAVYAVMFGP6mivH9rAArwAp4oADD34NKZBNYAVaAFTBVgOFvqhjfzwqwAqyABwow\/D2oRDaBFWAFWAFTBRj+porx\/RUF1K9Q46Sx\/YWqvCa+ubkZ2tvboaamJrG21I+pEh\/I+QaxRn7cuHFWz2mQN1Uz0S9nOTi7DBVg+GcoJrWkXIK\/SVnkeioD\/LG8Yv8q223Mh62kbWvkS\/oMf19q0pId6lGCYh8d+WtM4ZWip407beKWvOLCk4smTZo06N\/FaUayt4n3J3mcUV+Ayl95YzphXzhH2SH+HU+PElsnq162nC+mL\/+Oee\/fvx+2bt0Kvb29Qd74u6zD3XffDRs3bgxOnMNTuEzsVjcrNNnqOWxgS4J70u+WmhknW4ACDP8CRC9Llkm7UMreNtqEwMPP8oWXKu82KrZgFnvZh23PELcrq7qPUdjf8kZossZxdlx33XUwbdo0GDlyZBAqUu1Q85EHC7QzbEdV+VwFkd7OnTuD7bbDtqKOszsM\/vLxleqeOElvNUlwT\/q9LG2Xy5msAMM\/WSOydyTtn5MUapE33lPhH\/asOJ5x4cKFgYcsrqhyyGCMK0vcpmhpvGM5XxWWYTbIA8iBAwcG7daJNkbZjb+FwV894Spq8EhjG8OfTndn+NOp61SWyiEPNSwTBVx5Iy+xQZcKfzVUIwoXtqFXVFxe3vQtDv5xQNMFpPCwBwYGgqKK8JeadtjZu\/IguGvXLkDPXb2iNjKLCvvIcwBR9unaJpeF4Z+qm5TyIYZ\/Kast\/0LL8JPj\/nKoRUBfDBL9\/f0gDuUOg7\/uGa1Fwh9tmDp1KiD0xVxCnOevA39du6M8\/76+vkETwAz\/\/PuDDzky\/H2oxRxtUPdnF\/DH0Mzs2bNBDtkkhX0Qot3d3VBbWxtrQZFhH5yoVWFbbdhH124O++TYsAlmxfAnWOm6JidNysqhFrwXJ04xHIErZ4S3jith5IlOdcJXniCOi81nOeErQ3X69OmDyo2\/yZ40wl\/21EW4KirsI9LGNwV5Almd8NW1O2rCV7yFyAOsPE+C5RD1J\/ISdSImt8O+g+Cwj27vKP99DP\/y16FVC9RYtxz3VwGPk5mTJ08OyoPA6erqCiYsW1tboaWlpXLwjgCnuvwy6UOmuMM+kiaf1byEHeqgpcIf\/5aXbWLZ8YjNdevWBW8tW7ZsGTQ4yNAVS15xqecTTzwBy5cvD95yTOwOg\/9jjz0WaIznGeMlL20Nm4MQYSvUFwe7zZs3BwNTku06H8lZbXycuFUFGP5W5eXEWQGAsHkAXV10VvvopqVzH3v+Oir5cQ\/D3496ZCscUSBscjpuHX9SsRn+SQrx72kVYPinVY6fYwUiFFC\/CBZhrjSCqXv7hIWZ0qSrPsN7+2ShYrnS+H\/haTHR2KTUYwAAAABJRU5ErkJggg==","height":231,"width":383}}
%---
%[output:476701f6]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAX8AAADnCAYAAAD2Blb9AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnXuQV8WVx4+JMUx8BMcXjhMc1AHJuouPxUyIxhhWjdkaTGWNM8NuSSGy7AZNVQB5rmWoEpiZDKkYRUNwZEltGKhKihX+WAlFDD7QyIKZ3TJGiTDBcTRBHhoTXBNl61zsHz0999F9f7fv7dvn3KpUHH73dvf5dvenzz3dt\/uEo0ePHgW+WAFWgBVgBUgpcALDn1R9s7GsACvACgQKMPy5IbACrAArQFABhj\/BSmeTWQFWgBVg+BNrAwcPHoRp06YFVnd3d0Ntba0VBdavXw8LFiyAZcuWQUtLS5AH\/hte4u8sMw6z68iRI7BkyRKYMmUKNDY2ZpldZFq7d++GqVOnwp133qllZ5oyPvvss7Bt2zaYN2+eFZuElr29vUH6a9euhaamJu28wupe+2FLN6LO8+fPh7q6Omu6WSq6tWQZ\/tak5YRlBWwDQYV\/TU1N0Nl37twJq1evzg3+HR0dgHDWGVgFkEzKiGlPnjwZZsyYYQ1iIg954DZpzbbr2qQsahu8\/\/77c20Pacuax3MM\/wxUxg6\/cuXKICX0LGTYiI4wa9Ys2Lp1K6A3pd6jelpyxxae5BVXXAGnnnpq4IXhldQxRb5qmVRIHjhwIPBUhWeMHqVIWzcNfHtQgSEDAMuAbwHiam5uhvb2dkBA4yUguG\/fvgo0w8AotBgYGAiek3WS7XrggQegs7MTNm3aVMkTbZo0aVIwIKj\/Lr+JiLrEOurq6gL8e+TIkZXyqmWQ60H8hvYJr1ytW1H39fX1kWWRdUcDhF7YdhD84ho3blygF174Nic89aSBQWgrdBDpYD2qecu\/yV0lrs3Kdd\/X1xf0DbXNi\/ai2oJlEDqqbfK6666r2Ima3HTTTXD77bcPebsUbU3NM6x+Muj+pU2C4V9l1cngl5MSr8pqZ0rquOJ3ARUVNuJ3tWGrHo4MW3kAOOOMMwaFfQT8BVBFurt27RoE7Lg0qoU\/pi10ErrJgx4OFP39\/cEgJcqpDiQINBHOioK\/AJGslaxjGPj2798POPDGlUGta1F3KmTluo8q4\/nnnz8I8HJ7UH9DMH\/729+Gu+66qwJ+tf2ozTuqTFH1HgZ\/FfwiDzHoRLV5MYhF1aV4Xm3zWLaHHnoIHn744UED99VXXw1PPvlkqLMSNqjkFfKsEim5Pc7wr0Jq0UjPOuusiscqPBrR0Ddu3BhAVPyN2QnvU3jx6M2pwBBesIAzPodvFLLHGBaLFQ0coSXeQGRPTHhPmB56jWr66G2ZppEEf\/Ssk0IBqlem3i8G2TCwog6jR48eNKjphH1EmvLz6D2rMFfrUvwudBJvBt\/73vcCL1f8HvZGIzc3nbCPGuaJ+juq\/ahzOmr7RJ2E1iq8o94u1ftVqG7ZsmVQm5cH5rBwWNRAL9o8tklRbjEYifrFtxf5rU5+ewwLX2Gd4zN5hgKrQIzVRxn+VcgbBjS1w4uOIINabpSYveqly142\/jd6vML7lDtrGPzVjiRCK8LMqLCPnL5pGlnAX9ZNeMWiI2PZwyapZR3VQS0O\/qpnijriG5GqcxTc1SaDQBJlVuP3USEcLF8c\/KNCXCr8o7zsqDdDecATk7iqncJhiYJ\/WBphb55JA5L6BqG+GYS1eblMYfUvQl9yeeQwWFLZq8BB6R5l+FdRZWGeRRT8oxptFPzx36OgpIZIZBNMwS08\/2rhrw6ESX+HyS6eufvuu4O3EhE7j\/KgTeGvdnz572rgL4cloiZvw+aFxFuc\/Iyup58UYhHtR12lE9Z28oa\/2uZEGEgNr2UFf3mOieF\/vOcx\/KuAv42wj1qcMJjHwV\/2psSbgQyU6dOnh8b85Y6mm4YILcmhKHWyOOrvMNlVb1d+s6k27KPOdchhg7RhH9MQDt4vD4piAlqGvwonNcSSFPZJas5Zhn3UUKawQ8wXRXn+4m1Y\/K6WSR0MsK7ShH3CtGD4M\/yT+oj277YmfHVegaPWX4eFAkQYIGrCV4a\/DClZiLiVKuK+JPjjfVErSPA3oad6T9TEt9BJjSvLcMd0b7311mBSNCwsEDU5j2XQmfAVXrgKlqiJ0SgdMR28xMqxsNCFvEoG07nvvvvg3nvvHWKXuqJKpJU04Yvx9aT5Gd0J3yT4q50srs2HlVtnwld9A+KYP8NfG+46N2a91FMGn6nnL8obFtfGEIBOzD8pDfxdhjGWF98o7rjjjiErLwQAZGDEwT9uHbvuUk8xqSiDEsF6zTXXVFbSIGhuu+02mDlzZiW8pA4+uNRzzpw5sUs9ZciGhQHDQBk2\/4N5YxnFm5lYErxixQp45JFHQMx\/yIOaOqCLgS1OX8wnbqmn+nYS9UFeVLxenpOKgr86MKMeuMRYTMRiGdT5F\/w3OU+5PtUPCeU5NPk3Nbylzofp9HWf7ik07IOdee7cucGa7LAvMNW1wHHLG12tlCQvytVyUyuXDER1ma36VhSljYALDrK2vr6lVi\/CXjHw499hq9h0vhrndf6DW09h8NdZ5obgxLXZZe5IDP\/y4CoqhJf0QZ1sockXvuVRpviSJoXQdLbvwL7IX\/g6EPZBrx47Cl5Rnj\/+3tDQoLVHSvHNM7wEDH9Xa2ZoucLiyklfy6qpCO8SQ0Ym++GUR6XiSho276O77xDv7TO03grx\/HEUX7x4MbS1tQUDQBj8RWVNmDCh1PAvrqtwzqwAK8AKRCtQCPzRG8br8ssvj4z5h73mmbx+c6WzAqwAK8AKOAR\/fHVbs2YNLFq0KNivJWrCV3195tdpbsasACvACmSnQO6eP4Z5cMkdxkOTVvuoZoo5grAJ4AsuuCA7VTglVoAVYAUyVAB39B01alSGKVafVK7wj5qxRzN0Jm7iJoAR\/nv27KlekRKkwLaWoJJSFpHrNqVwjj\/mYr3mCn+1fuI8f7EaSP5gBz+4idqNz0VxbbVHttWWssWny3VbfB3YKIGL9eoU\/BH4PT09lYMz1I+84t4OXBTXRiPCNPfu3evcKyTbmo0CXLfZ6OhaKi7yqVD4Z1lBLoqbpX1yWgwIW8oWny7XbfF1YKMELvKJ4W+jpi2nyYCwLHCByXPdFii+xawZ\/sTEtWUuA8KWssWny3VbfB3YKAHD34aqH6bpori2zGVA2FK2+HS5bouvAxslcJFPHPaxUdOW02RAWBa4wOS5bgsU32LWDH9i4toylwFhS9ni0+W6Lb4ObJSA4W9DVQ77WFS1+KQpwRDVpmQvJVsZ\/hZZ4qK4tsyl1Gko2crwt9Vjik\/XRT5xzL\/4dmFcAkpApGQrw9+4K5TmAYa\/xapyUVxb5lICIiVbGf62ekzx6brIJ\/b8i28XxiWgBERKtjL8jbtCaR5g+FusKhfFtWUuJSBSspXhb6vHFJ+ui3xiz7\/4dmFcAkpApGQrw9+4K5TmAYa\/xapyUVxb5lICIiVbGf62ekzx6brIJ23PP+4glihpx40bBxs2bMhFeRfFtWU4JSBSspXhb6vHFJ+ui3wygv\/s2bNh4cKF0NjYmKgmHtSydOnS4PCVPC4XxbVlNyUgUrKV4W+rxxSfrot80oZ\/8fLFl8BFcW1pRgmIlGxl+NvqMcWn6yKftOEvwj4oozhasXhJj5fARXFt6UMJiJRsZfjb6jHFp+sin7Thj\/IdOXIE5s+fD5s2bQrU1Dl0PS\/ZXRTXlu2UgEjJVoa\/rR5TfLou8skI\/rKE8vm6zc3NlXN3i5LZRXFtaUEJiJRsZfjb6jHFp+sin1LDX8gpvw3U1dUFE7w6E8JZV4eL4mZto0iPEhAp2crwt9Vjik\/XRT5VDX9ZVrHCZ\/ny5VBbW5ur4i6Ka0sASkCkZCvD31aPKT5dF\/lUNfzZ88+\/YVECIiVbGf7596W8cvQK\/hzzz6vZDM2HEhAp2crwL65P2c659PBXV\/ssW7YMWlpabOumlb6L4moVPMVNlIBIyVaGf4rOUIJHnvrNYfhK1xZ48\/tfc6q02mEfsc5\/\/\/79hU3qxinH8HeqXWVWGIZ\/ZlI6lxCVuu3Z8QbM7HkRDn7nWqfqQBv+TpU6pDAMf9drKF35qABCqEPJXiq2lh7+6Pnz3j7pAJb1U1Q6DbUwCDV7qbRjL+A\/bdo06O3t1WYZ7+qpLZXRjVQ6DTUYUrOXSjvu2NwHHZv3ctjHiHIGN3PYx0CsEt1KBRAc9ilRozQsKsPfUDDT2xn+poqV436GfznqKU0pqdQtwz9N6zB4huFvIFaJbqUCCPb8S9QoDYvK8DcUzPR2hr+pYuW4n+FfjnpKU0oqdcvwT9M6pGc6OjqCv+bNmxeaEsO\/SoEdfZwKINjzd7QBZlAsXOOPK368Wee\/fv16WLBgQSAN7uuPV09Pj5WtncVWEjNmzGD4AwAlIFKyFfsQJXup2OoV\/NELHxgYgHvuuQcWL14MbW1t0NTUBEneeZpBFLeUWLJkCbzwwgtBHuz5MyDStKOyPEMFiJQGupt\/0As\/+\/XB8nv+8sde9fX1wcleAv42tnTGNwy8+vr6OOzzIcEYEGVBuXk5uW7NNXP9iUkrnoenXjnsN\/wxPIPef1Zn\/OJAg28W+IaxatUqhj\/D3\/V+XnX5GP5VS+hcApfe+wzsO\/hu+eGPyqI3vn379kFhn9GjRwN+Adza2prZTp84kFxzzTVaISWc8MVr69atzlV+1gXq7+8HfOuicFGyFeuTkr0UbL22uQXevv7YYhVvJnzl\/fwFhLLc4hlDSGvWrIFFixZBTU1N4nwCr\/bxcyig5AlTioNTsRW3c5704PN+wd82auTVRHJeeFj8fffdNyR7hr\/tGikmfYZ\/MbrnkSuFumX4Z9CSklYSMfwzENnBJCgAQpadkr0UbBWTvR\/505vlPcxFNFBxqEvS7p5xa\/LTMIbhf1w1Cp1GWEvJViqhEEp1Wzvr8cDcE998CX7\/w39Jgz5rz6Q6zAVDMuvWrRu0qkcMCmLCNwnWWVvEnn\/WirqRHsPfjXqwUQrf61Z4\/ajdKU91wr7n\/suGjKnTNIa\/gDx+bIUfXcmXvNTzwIEDsHTp0uDIxzwuhn8eKuefh++AUBWlZK\/Ptsqx\/pG1w+DtR\/4R9uzZk38HismR4e9UdegVxudOQxmGHPbRa\/+u34Vr+nFtP14I\/gdax8Kt119efvijQTphH\/EtQNjKHBuVx56\/DVWLT5PSQMfwL769VVsCGfyYFoJ\/8pUjwEU+GXv+Qpywdf64wRuGgvC3OXPmBCGfxsbGavXUet5FcbUKnuImSkCkZCvDP0VncOgROdSDxZp3wyiYd0NDUEIX+ZQa\/g5p7qy4tjSiBERKtjL8bfUY++mKLRxETo9943K4suGTlYwZ\/hbrwEVxbZlLCYiUbGX42+ox9tKVV\/RgLiLGf9VFwwdl6iKfUnn+YSEfYem4ceMy29jNpMpcFNek\/Cb3UgIiJVsZ\/ia9oLh7Ma5\/R8+LwU6d8tU2fgSsaBsbWjAX+WQMf9xfH7dxnjBhAkyaNKmypbONjd1MqtdFcU3Kb3IvJSBSspXhb9IL8r0X4\/k9O14PTuRSL\/T2N379ssDrj7pc5JMx\/OX9\/HEyFz\/mamhoCHbyxDcCW6d5JVW1i+ImlTnt75SASMlWhn\/aHmHnuSgPX4R3rmk8He5ruVgrcxf5VDX8cUknHrSCH33ZOMxFS1lHZ9N1y256HyUgUrKV4W\/aE7K9H2Hf\/XQ\/PNq7P9h\/P+yKiuknlcQL+KOR8tYNsre\/cePGYJ\/\/9vb2YBvmPC8XxbVlPyUgUrKV4W+rx4Sni4Dv2Lw3NJQjnkDYjzx9GDzQNjY2rJNUchf5ZOz5o5HqFg84GKxcuRLq6upyXdsvC+6iuEkNIu3vlIBIyVaGf9oekfwcgv7Bba\/CrwbeGTJRqz6NwP\/aFSNg0Y2jkhPWvMNFPqWCv6a9ud7mori2BKAEREq2Mvyr7zEiXIMe\/asH300EPeaIsP+nz5wLt1wxIihA3MRt2hK6yCdj+KsTvrIYHPNP2zTMnqMEREq2Mvz1+0EayAuwf+7C4cHXtzYgH2WB9\/DP+gB3\/abg5ufTJuU3uZcSECnZyvAf2gsQ8m++8x58a9MrsO\/Qu5ETsWH9B+F+1UWnw9zrG3IFfVhZSg3\/uA+7ZGOzPsRFF4ouiqtbdtP7KAGRkq1U4Y+Af\/2t\/4MfPjugHaqR+4yYlP1W84Vw5iknFQ567+AvDIoL+5hCLMv7Gf5ZqulOWgx\/d+oiTUnkJZP3P74PXnrjj8YevByHx5ANxuZHnVnjJOS9DvukaQB5PMPwz0Pl\/PNg+OevuUmOMtzXPDsAO\/a+lQruMuBxaeWUz9bBOad9HNQ9ckzK5tK9LvJJe8JX9+xe3tvHfpOjBERKtroY9sFtDTCsgnH3f39mAPrePJIa7qoHf9NFH4GLLzy\/VB582t5davinNTqv51wU15btlIBIyda84Y9eO\/7vnNNOgke2vwYvvPZOVWBXvfc7vzgSxpxzcqUbqKtrKNWti3zS9vxtgSyrdF0UNyvb1HQodRpKtmYFfzkU81zfW\/Dzlw7BvoPVeeyiDYrJ1ZG1NfDPV58Hwz\/xseCnNMsmKdWti3xKDX\/c02fBggWDuLRs2bJgg7ciLhfFtaUDpU5DyVYd+Atv\/dxPngRrnhmAX776h6CZqdsLp2l7AuAYc7\/5inPgC6NrI732NOlTdmJc5FMq+Ouc4ZtF4zBJw0VxTcpvci8lIFKxVcTWd+\/dB1v2nRBsQ5AV1GXP\/OqLTofPjPokfL7x9IrHjgNKGs\/dpM2G3UulbtF2F\/lkDH91Xx+5Uvkjr2q7g97zlDpNmW0dEn55+RDsO5BN+EUOwwRwP30YjD33FJh+9Xlw0kc\/kjoUo9cCs7mrzHVrqgDD31Qxg\/tdFNeg+Ea3Uuo0LtoqoP7eXz6AHz33Ouz87dtB\/Zl+gRpX6XII5tN1p8BXLzsbRpz28cK9daOGmnCzi3WbpX1yWi7yydjzR4M47GOrieilS6nT5GWrAPr7HxyFnzz\/O3ji5UOZA10Ov6CnftHZn4AZn6+Hmo99tFLx77\/1Oowald1uknotqpi78qrbYqwbnKs38BcDAE\/4FtOsKHWaamwVQD\/0pz\/Dhl\/+HnZZ8NBVoH+qdhhgXH3ChccP8DaJp1djbzGtMX2ulGz1Av4i5t\/a2lrYyp6w5uaiuOm7RfyTlDqNbKscQ8fVLdtfORzE0PHKYrWLrLocdsF\/\/8KYWrj58nMqt5gA3aQdUK1bE43KeK+LfEoV9lE3eVu7di00NTUVWicuimtLEJ8AIZYunnnKx+ChJ16FvfuPwTzL+LmoBxno6KGPGXEyfGXc2daBbtIOfKrbJLsp2eoin1LBX65Ueb0\/n+SV1Nyz+d3VTiN75vjfO3577AOjo0ePWoF5WMjlurFnwOUjT3MK6Ca17mrdmtigey8lW72Ev1zReJwjvhV0d3dDbe3xD0R0G0M197kobjX2xD2bZ6eRJ0If7f09\/OzXB6155irM8e+6kz+A1gkXwAVnHj8T2lbIxVZ9maSbZ92alMvGvZRsdZFPVXv+4vxebBxFbeqGebsoro0Og2mm7TSyZ\/7n9z+AH+\/6HTz9m8NWYa4CHUMtN\/7VmcFKF\/Fb3EdGaW21pb3tdCnZS8lWF\/mUCv7VhHqOHDkC8+fPh02bNgX9KO7wF3VuIS6s5KK4tkARNgn63vsfwLodb8Bze9\/KFebnn1ETTIaOP\/9YqAVBnuU2vJQAUc3Abqut2UyXUt26yCdj+Md94avTUPBNAa958+ZB0sohHGT6+vqCe5MuF8VNKnPU78JD\/+N77wf7t4hP\/W1Mgqqe+SXnnQLXjqkdtBujfE9am9I+RwkQDP+0rcT951zkkzH8s5ZZHgzUtPG3hoYGrSWlLoobppUA+9GjAJ0\/3RscW5cl1OV4OH5INPqck+GWvz1n0NehwkMvQ+yc4Z91j3MnPUp16yKfCoV\/3FuECA9NmDChtPBH0Pf2\/wFWPdlf1Tp0dYliw8l\/hparRlsJs7iDhmMloQQIavZSqluGv0QWMVHc3NwM7e3tUFNzfDUH3hZ2cljcltGuiDtpxfNGoBdgx7NJr\/\/0GXDZp47FzuO8ckqdhpKtDH\/XXI\/syuMKn2SLCvX8sSA4CAwMDAwZAHbv3g1Tp06Frq6u4AMy9W+1WlBcvLZu3ZpdjWmk9ItXj0D3jrdg52vvRt5dd9qJcO6pJ8JnPjUMbhxzSuU+\/Pc0V39\/P9TX16d5tHTPULIVK4eSvRRsnThxYqXP7dmzx6n+Zwx\/9Mhnz54NCxcuhMbGxkHGIKCXLl0Ky5cv117nj8\/MnTsXOjs7h6SnKhU3P5DXyCpi9nf0vBjq4QuPvfOro+HiESdb2SedkjdMyVb2\/J1iY6aFyYtPJoXOFP5p9vM3eSZuAjgPcfHAjTvWvRgsZ5QvBP70q+qh+W\/OsgJ7tUIpAZGSrQx\/E3SV6948+GSqiDb81TX3URnFrdsXYR78f1y+KSZ1cf2+upxTHRTw7zlz5sDq1atD3xBsiouwD\/P028aPgLbx52a6rl2nAikBkZKtDH+d1l\/Oe2zyKa0i2vAXGcSFfXQKoX7kJU\/4IuB7enoq8X+TDeRsiYve\/qQHn6+Yhl7+3118BnzjiyNz8fLDNKUEREq2Mvx1CFLOe2zxqRo1jOFfTWY2n81aXPT2t\/76IMz+8UuDwL\/x65cVBn1REEpApGQrw98mIYpNO2s+ZWGNNvzF0svp06fDqlWroLe3NzT\/ovb3yVrcjs190LF5b2AjevsPtI7NPbwTVcGUgEjJVoZ\/FkhzM42s+ZSFldrwzyIzm2lkJa4a30fwu+Dty9pRAiIlWxn+NglRbNpZ8SlLKxj+ipqqx+8a+BkQWTZ\/99KiNNhRstUL+Id9eSt3oTKHfXp2vAEze16shHpcBD\/D3z1gZ1kiSkCkZKsX8I9q6LiKZ8mSJTBlypTEj7Wy7CwirWrFxXDPpfc+4zz4Gf42Wo87aVICIiVbq+WTjRaaadhHXappo8BRaVYrLoJffLy1om0s4Bp+Vy9KnYaSrTywu9rjqi9XtXyqvgRDU8gU\/mm2d8jKqGrEvesnL0P3068FRfnmxPPh7r8\/tk+QqxclIFKyleHvao+rvlzV8Kn63MNTyBT+UZu02Sq8nG5acdVwzy\/\/7bN5FLeqPCgBkZKtDP+quoXTD6flk02jjOEfN+Ebd8yiTSMw7bTiylswI\/j5gBPbNWWWPsPfTK8y3U2pbtPyyWZ9GsPfZmGqSTuNuPLWDZOvHBF8yFWGi1KnoWQre\/5l6H3pypiGT+ly0n8qFfzVU7biNmjTL0p1d5qKi+Ee3LMH\/9\/FD7ni1KAEREq2MvyrY4DLT5vyKQ9bUsE\/bF\/9ogcAU3HlNf3\/cdtfw5cvOTMPvTPJgxIQKdnK8M+keziZiCmf8jDCGP5ZH+aSlZGm4opYf9m8fgZEVi3GzXQoDXaUbDXlUx6tMxX8p02bFuy\/j8crypfJwSxZG2cirhzrv7WpDr57y5isi2M1PUqdhpKtPLBb7TaFJm7Cp7wKagx\/LNj69eth3bp10N3dXTmuUawCam1thZaWlrzKX8nHRNwyrvCRBaUEREq2Mvxzx0ZuGZrwKa9CpYI\/Fi7sZK9ly5YVAn4sj6648rr+qy4cDhtnXpaX1pnlQwmIlGxl+GfWRZxLSJdPeRY8NfzzLKROXrriyhO9ZVnXr9pPCYiUbGX46\/T0ct6jy6c8rTOGv1jV09bWNiTmn2fB1bx0xZUnesvwNW+YppSASMlWhn+RBLGbty6f7JZicOrG8K\/2DF9bxumKWzvr8aAIt3\/uPOj8h9G2imM1XUpApGQrw99qtyk0cV0+5VlIY\/hj4XDCt6+vL1jx48qlI+7mXx2Atof\/JyhyWUM+DAhXWpydclAa7CjZqsMnOy0qOlVj+Jf5MBcfQj4M\/7y7SL75UQIiJVu9gH++XUE\/tyRx5VU+t004D7puLmfIh+Gv3ybKeCclIFKyNYlPRbRVY8+\/iELq5JkkrvxhV5lDPgx\/ndZQ3nsoAZGSrUl8KqLFasNfhHumT58Oq1atgt7e3tDyunqGry8hH4Z\/Ed0kvzwpAZGSraWGf37NP11OceL68GGXrAqlTkPJVh7Y0\/X9MjzlDfzLtqWzDP95N4yCeTc0lKG9RJaREhAp2crwL3W3jC28N\/Av25bOK5\/shwUbdgeVU\/Z4PwPCX0Bw3fpbt17Av4xbOvsU72dA+AsIrlt\/69Yb+JdpS+dB8f6LhsPGr5dvIze1S1AKhVCyleHP8M9TAe3VPnKhyrSls7zEE8F\/1UXD89TXSl6UgEjJVoa\/le7iRKJeeP5CybJs6fzQtldh0aO\/CYrN8HeiHxgVguFvJFepbqZUt17BP49WJg8wdXV1sHr1amhsbAzNOkpc3+L97B3m0fKKy4MSECnZyvA36FO7d++GuXPnQmdnZwD8sFCTnFyYuL6t7xf2Uuo0lGzlgd0AECW7leFfRYWpg4GaVJi4Psb7GRBVNKISPEppsKNkK8O\/is6Hnv\/27duhvb0dampqhqTE8K9CXIcfpQQIHtgdbohVFo3hn0JA9PinTp0KAwMDsHbt2sjTw8LElQ9qP\/ida1Pk7uYjlIBIyVaGv5v9LYtSeQN\/GciqMLY2dhN5dnV1hQ4AYeJeeu8zgHH\/kbXDgi97fbkoAZGSrQx\/X3roUDu8gL\/Y1wdX3+R9klfYthJCZhQXr61btwb\/P\/D2X6B5TX\/w31ecNwx+8NUR3rSs\/v5+qK+v98aeOEMo2Yo6ULKXgq0TJ06sNO89e\/Y41WeNP\/Iq6gxfdTM5VUV1ZJUne33YzE22l5I3TMlW9vydYmOmhfHK829ra4uMv2ehmrq6B9f8z5kzJ3Ktvypux+Y+6Ni8NyiKD5u5MfyzaFXup0FpsKNkqxfwx+6TtPImqy6mfkVsMuHr62Qve4dZtS4306EEREq2egH\/shzg7uOXvQJXlDoNJVt5YHdzQM6iVF7APwshbKQhi+vrl70Mfxstx600KQ12lGwpQc3TAAANQklEQVRl+FvsZ1Hw922yl71Di43IgaQpAZGSrV7BH+P+CxYsCLoLxuLx6unpifwC13a\/ksWVV\/r4NtnL8LfdkopNnxIQKdnqDfxxvT1+cXvPPffA4sWLQaz8iVuHb7tLyeI+9sKbMLn7f4MsfdnGmVf72G5BbqRPCYiUbPUC\/vI6f\/zQaP78+RX44\/LMpUuXwvLly6G2tjbX3iSL6\/NkL3v+uTar3DOjBERKtnoPf1yaid5\/d3d3ofD3dVsHnvDNncW5Z0gJiJRs9QL+2BvEOn857DN69GjAs31bW1uhpaUl904jxPV9pQ97\/rk3rVwzpARESrZ6A3\/sDa4e4yjDf0XbWGgb78+ePuz558rhQjKjBERKtnoF\/0J6RkymQlxfD3DhCV\/XWpyd8lACIiVbGf52+kuQqhC3Z8cbMLPnRW9X+nDYx2IjciBpSkCkZKtX8JfX+Ys+E7f3ju1+JcRF8OMAgJdPB7iw52+7BbmRPiUgUrLVG\/iHHaYu9vwpesLX92We7Pm7AWlbpaAEREq2egF\/AXk8yKWpqWlQH3BhqadY5nnVhcNh48zLbPXRQtOl1Gko2coDe6Hdymrm3sO\/6I+8fv7fvwKEP163NtXBd28ZY7VCi0qcEhAp2crwL6pH2c\/XC\/ijTGEHq7gQ9vnhT3fBpAefD2rSxw3dRBOlBERKtjL87UO4qBy8gH\/Sfv6yuHiY+4YNG3LRG8WVPX8f9\/Rh+OfSlArNhNJgR8lWL+BfaM+IyRzFnfHQzypHNzL8Xa0ps3JRAgR7\/mZto0x3M\/wt1haK27J8M6x8sj\/IxcetnNnzt9iAHEma0mBHyVav4B+2zn\/ZsmWF7OuD\/RbFvWT2T+CpVw7DyNphAfx9vSh1Gkq2sufva489\/hGqSxaecPTo0aOmBXJ1nf9pt\/0IcG8fhr9pjbp7P8Pf3bqptmSU6tYLz9\/ldf6Hv9IdtEef1\/izd1gtctx+nhIQKdnK8LfY7xouuRLevr4jyMHnZZ4Mf4uNyIGkKQGRkq1ewB\/7h4thH4a\/A+SyUARKgOCB3UIDciRJb+AvBgBxgLvQt8gJ35FX3gjvXDU3KIrPyzwZEI70ZkvFoDTYUbLVK\/hbavupk6378jfh3YsnMfxTK+jmg5QAwQO7m20wi1Ix\/LNQMSKNETffC++N\/Fzwq89r\/BkQFhuRA0lTGuwo2crwt9i5zr71+\/CXM8d4v8yT4W+xETmQNCUgUrKV4W+xczH8LYpbYNKUAMEDe4ENzXLWDH+LAtfOejxI3fc1\/gwIi43IgaQpDXaUbGX4W+xcAv5t40fAiraxFnMqPmlKnYaSrTywF9+3bJWA4W9LWQAQ8P\/Pf70UPt94usWcik+aEhAp2crwL75v2SoBw9+WshL80etH79\/nixIQKdnK8Pe31zL8AUA9DKa5uRna29uhpqZmSM3jiWGTJ0+u\/HtdXR2sXr0aGhsbh9wrPH\/fP\/BiQPgLCK5bf+uWPPyPHDkC8+fPhwkTJgRbP4u\/Eep4ILx64TYSfX19ob+p9wr4+77GnwHhLyC4bv2tW\/LwD6taBPz27dtDvf+Ojg5oaGjQOiOA4e9nx+Gwj5\/1Sm2gY\/iHtOMo+KtvCUldQMD\/4HeuTbq19L9TAiIlW6kBkVLdMvwV7Ir4f2tr6xDvPuyg+LiN4xD+vh\/iIuSj1Gko2crwL71fFmkAw1+SRnj2+E9hE767d++GqVOnQldXFzQ1NYH6d1jMn8IHXgwIfwHBdetv3TL8P6zbJPBHNQGcA8ArbHIYPf8T33wJfrH4Bn9b0IeW9ff3Q319vfd2ooGUbKVmL4W6nThxYqWf7tmzx6k+m+oM32osSFrhE5d23AQwwp\/C173sHVbT+tx\/llKYi5Kt7PkDAAJ8YGAgcm2\/6J64xh\/v7e7uhtraWsC\/58yZE7vO3\/fjG4U2LjYkW1ilZCtqSMlettVWr9FLN1fPP2wSF4s5bty4APIvv\/wy9PT0VAYG9SOvtWvXBvH\/sAs9\/0\/segRO2ve0nuV8FyvACrACOSpAPuyTo9acFSvACrACrECEArl6\/lwLrAArwAqwAm4owPB3ox64FKwAK8AK5KoAwz9XuTkzVoAVYAXcUIDh70Y9cClYAVaAFchVAYZ\/rnJzZqwAK8AKuKFA6eGPG8MtWLAgUDNu7x835NYvBX7jsHLlyuCBuCWu8n1x5x3o51zMnbr2itKZbvxXjFXhueraqi6NjmsHLtmnlkXXXrGFC34HVOa2HFUXJrsU51GfpYY\/Npa5c+dCZ2dnoJX477DDXvIQM6s85A\/c8NsH+WM3OQ91R1T8e926dZUP47Iqj+10dO1VbcdBv2wDvq6t6pfwclsvU\/vWtVcMdLh1C37LU9a2HAd+dOZcaq+lhr8KP9dG1rTQlPcwEhBoa2uL\/MBN5FNWQJjai6CYPXs2HD58GMJ2hE2rex7P6dqKdbl06VJYvnx58IV7WS8Te2XnraxtWa0nMagNHz48+OlLX\/qS1vkkedR3qeGvbvQWt\/FbHmJmkUfUaWfi9LO4PMrYYdLYi\/U8fvx4ePTRRyunwmWhve00TGyNO+TIdjmzSt\/E3jDPP+qQp6zKl0c6aNehQ4eCMJZ8imEeeSflUXr4yyd9mRz7mCRMUb+Hefq6bzS6+yYVZVtYvqb24gC3Zs0amDVrFixevLiU8Jff4qLqVrRl1Exn7selOhVlMa1bcf+mTZtgxowZWse3umh3XDvXceLysonhn5fSmvmYdhiRLMLi\/vvvj9z4TjP73G8zsRfvXbJkCUyZMiXY0to1TypJPBNbxUIGMcmbtLFhUt5F\/G5ir3peh7qxYxHlzzJPFxcolB7+WEFif3+qYZ+ygh\/rziQ0gEDYtm1bUN8udqYkWJjYqoZ92N4kdd3+3cX6KzX81TCPbnjE7WZybNtrEc5KmvD1YVWErr3ykkG5DssUItC1FQc6eYfbpHbgapvWtdeHwS6uDhj+GbdQ6ks9yxgKCGsCussB5Wdd7Ew6zVvXVnUCtKxhEF17w8I+ced36Gjt0j0uttdSe\/5YudQ+8pLfdqI84TJ+DBT1IVDUJL6LnUkXNrq2yh95lfmjJ1175fM7ymwvT\/jq9gS+jxVgBVgBViB3BUrv+eeuGGfICrACrIAHCjD8PahENoEVYAVYAVMFGP6mivH9rAArwAp4oADD34NKZBNYAVaAFTBVgOFvqhjfzwqwAqyABwow\/D2oRDaBFWAFWAFTBRj+porx\/RUF1K9Q46Sx\/YWqvCa+ubkZ2tvboaamJrG21I+pEh\/I+QaxRn7cuHFWz2mQN1Uz0S9nOTi7DBVg+GcoJrWkXIK\/SVnkeioD\/LG8Yv8q223Mh62kbWvkS\/oMf19q0pId6lGCYh8d+WtM4ZWip407beKWvOLCk4smTZo06N\/FaUayt4n3J3mcUV+Ayl95YzphXzhH2SH+HU+PElsnq162nC+mL\/+Oee\/fvx+2bt0Kvb29Qd74u6zD3XffDRs3bgxOnMNTuEzsVjcrNNnqOWxgS4J70u+WmhknW4ACDP8CRC9Llkm7UMreNtqEwMPP8oWXKu82KrZgFnvZh23PELcrq7qPUdjf8kZossZxdlx33XUwbdo0GDlyZBAqUu1Q85EHC7QzbEdV+VwFkd7OnTuD7bbDtqKOszsM\/vLxleqeOElvNUlwT\/q9LG2Xy5msAMM\/WSOydyTtn5MUapE33lPhH\/asOJ5x4cKFgYcsrqhyyGCMK0vcpmhpvGM5XxWWYTbIA8iBAwcG7daJNkbZjb+FwV894Spq8EhjG8OfTndn+NOp61SWyiEPNSwTBVx5Iy+xQZcKfzVUIwoXtqFXVFxe3vQtDv5xQNMFpPCwBwYGgqKK8JeadtjZu\/IguGvXLkDPXb2iNjKLCvvIcwBR9unaJpeF4Z+qm5TyIYZ\/Kast\/0LL8JPj\/nKoRUBfDBL9\/f0gDuUOg7\/uGa1Fwh9tmDp1KiD0xVxCnOevA39du6M8\/76+vkETwAz\/\/PuDDzky\/H2oxRxtUPdnF\/DH0Mzs2bNBDtkkhX0Qot3d3VBbWxtrQZFhH5yoVWFbbdhH124O++TYsAlmxfAnWOm6JidNysqhFrwXJ04xHIErZ4S3jith5IlOdcJXniCOi81nOeErQ3X69OmDyo2\/yZ40wl\/21EW4KirsI9LGNwV5Almd8NW1O2rCV7yFyAOsPE+C5RD1J\/ISdSImt8O+g+Cwj27vKP99DP\/y16FVC9RYtxz3VwGPk5mTJ08OyoPA6erqCiYsW1tboaWlpXLwjgCnuvwy6UOmuMM+kiaf1byEHeqgpcIf\/5aXbWLZ8YjNdevWBW8tW7ZsGTQ4yNAVS15xqecTTzwBy5cvD95yTOwOg\/9jjz0WaIznGeMlL20Nm4MQYSvUFwe7zZs3BwNTku06H8lZbXycuFUFGP5W5eXEWQGAsHkAXV10VvvopqVzH3v+Oir5cQ\/D3496ZCscUSBscjpuHX9SsRn+SQrx72kVYPinVY6fYwUiFFC\/CBZhrjSCqXv7hIWZ0qSrPsN7+2ShYrnS+H\/haTHR2KTUYwAAAABJRU5ErkJggg==","height":231,"width":383}}
%---
