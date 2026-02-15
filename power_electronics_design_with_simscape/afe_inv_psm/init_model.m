preamble;
model = 'afe_inv_psm';
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

fPWM_DAB = fPWM;
tPWM_DAB = 1/fPWM_DAB;
TRGO_DAB_double_update = 0;
if TRGO_DAB_double_update
    ts_dab = tPWM_DAB/2;
else
    ts_dab = tPWM_DAB;
end

s=tf('s');
z_afe = tf('z',ts_afe);
z_inv = tf('z',ts_inv);
z_dab = tf('z',ts_dab);


% deadtimes
dead_time_AFE = 3e-6;
dead_time_INV = 3e-6;
dead_time_DAB = 3e-6;

% minimum pulse
minimum_pulse_time_AFE = 2e-6;
minimum_pulse_time_INV = 2e-6;
minimum_pulse_time_DAB = 2e-6;

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
nominal_power = 1600e3;
application_voltage = 690;
vp_xi_pu = 1;
vn_xi_pu = 0;
vn_eta_pu = 0;
grid_emu_data = grid_emulator(nominal_power, application_voltage, vp_xi_pu, vn_xi_pu, vn_eta_pu);
%%
%[text] ## Global Hardware Settings
hwdata = global_hardware_setup(application_voltage, fPWM_AFE, fPWM_INV, fPWM_DAB); %[output:870743c4]
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
parasitic_dclink_data; %[output:0c17d7f4]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:6e6ea8f6]

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
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:0a7f75d6]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:4db39541]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:08f83058]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:89ccd37d]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid_emu_data.w_grid;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:2e22cd95]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:1cf68f47]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:6bacded8]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:63025039]
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
run('n_sys_generic_1M5W_pmsm'); %[output:7ea2de59] %[output:9ff4664a]
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
kg = Kobs(1) %[output:0b291691]
kw = Kobs(2) %[output:375e0e67]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:89445da1]
luenberger_l2 = Klo(2) %[output:726a88d3]
luenberger_l3 = Klo(3) %[output:88710ee3]
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
lithium_ion_battery = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_inv); %[output:2ea7c278]
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

if use_torque_curve %[output:group:154f60b8] %[output:4ff6cffb] %[output:312632c6]
    set_param('afe_inv_psm/fixed_speed_setting', 'Commented', 'off');
    set_param('afe_inv_psm/motor_load_setting', 'Commented', 'on');
else
    set_param('afe_inv_psm/fixed_speed_setting', 'Commented', 'on');
    set_param('afe_inv_psm/motor_load_setting', 'Commented', 'off');
end %[output:group:154f60b8]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":27.4}
%---
%[output:870743c4]
%   data: {"dataType":"text","outputData":{"text":"Device AFE: afe400V_250kW\nNominal Voltage: 400 V | Nominal Current: 360 A\nCurrent Normalization Data: 509.12 A\nVoltage Normalization Data: 326.60 V\n---------------------------\nDevice AFE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\nDevice AFE: afe480V_250kW\nNominal Voltage: 480 V | Nominal Current: 360 A\nCurrent Normalization Data: 509.12 A\nVoltage Normalization Data: 391.92 V\n---------------------------\nDevice INVERTER: inv400V_250kW\nNominal Voltage: 400 V | Nominal Current: 470 A\nCurrent Normalization Data: 664.68 A\nVoltage Normalization Data: 326.60 V\n---------------------------\nDevice INVERTER: inv480V_250kW\nNominal Voltage: 400 V | Nominal Current: 470 A\nCurrent Normalization Data: 664.68 A\nVoltage Normalization Data: 326.60 V\n---------------------------\nDevice INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\nDevice DAB: DAB_800V\nNormalization Voltage DC1: 800 V | Normalization Current DC1: 350 A\nNormalization Voltage DC2: 800 V | Normalization Current DC2: 350 A\n---------------------------\nDevice DAB: DAB_1200V\nNormalization Voltage DC1: 1200 V | Normalization Current DC1: 250 A\nNormalization Voltage DC2: 1200 V | Normalization Current DC2: 250 A\n---------------------------\n","truncated":false}}
%---
%[output:4c6b811c]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:7fbcc266]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:0c17d7f4]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAgAAAAE0CAYAAABae4vcAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ1wVsd57x9aWksek4AwiSvLSMSGxPF0iJ24JTKpcJWUZBLRXtwaRDKXcGXiS811phEjCZkZoamtDybK1BSbi42i6k4qxXHC3KKZTqmDA3MdVfcm\/tDcS51CLASWia8dBGN7KnJDqzvP4n05Orwf52P3nH2f9\/\/OeIzed8+zu\/\/f7nmes7tnd97s7Ows4QMFoAAUgAJQAAqUlALzEACUFG9UFgpAASgABaCAUgABABoCFIACUAAKQIESVAABQAlCR5WhABSAAlAACiAAQBuAAlAACkABKFCCCiAAKEHoqDIUgAJQAApAAQQAaANQAApAASgABUpQAQQAJQgdVYYCUAAKQAEogAAAbQAKQAEoAAWgQAkqgACgBKGjylAACkABKAAFEACgDUABKAAFoAAUKEEFEACUIHRUGQpAASgABaAAAgC0ASgABaAAFIACJagAAoAShI4qF4cCp06doi1bttC5c+cyBW5oaKCenh4qLy8vWImZmRlqa2ujxsZGWrVqVcH0Y2NjtGnTpjnpHnzwQWptbaWwtgpmhgRQAAqkrgACgNQRoABQILsCHAC0tLTQnj17aPny5aGdcFinzQFAb28v9ff3U0VFRSa\/yspKFQTgAwWggCwFEADI4onaCFKgUADgfWLXT+pcfXbiBw4coLq6OqUG\/8YjAM888wzt3Lkz853fqfsDAE7IZejq6qJHH31UBSI8mrBy5Uo1sjAyMqJs6VEJ\/rf+nr975513qL29nV566SUaHR2ls2fP0saNG6m6unrOSMPQ0BCtWLGCmpub6Q\/+4A\/oL\/\/yL4mDjm9+85uqLuPj49Td3U0bNmwQRBdVgQLpK4AAIH0GKAEUyKpAtikA7Qi9wUFVVZVyvLW1tcq56qf48+fPqykEdqT8GR4eVtMH2lH7pwayBQDT09PKMX\/jG9+ggwcPqgDglltuUTZuvvlm0r97HT3nwU57x44dNDAwoAKA7373u5mRBc5HT0lwUDI5OUlbt26lpqYm9T0HJlwHTsejEc8995wKIIJOfaA5QQEoEEwBBADBdEIqKJC4ArlGALSj1w6d1wNoR6oL6Z+3P3PmTObpX6fxjhrwd0EDAHbSenqBRwH4aX3\/\/v0qQOCy8ZN6rsBAr13wj17oAIDLrUcrODDgvzmtt66Jg0CGUECoAggAhIJFtYpfAX8AwDXSjp6H98MGANqh5lIm6BQAX8+LBb1D93qEoFAAoEcf+P\/8RH\/48OE5IwAIAIq\/3aIGxaMAAoDiYYWSlpgC+UYA7rrrrswCwaBTAHpI3pveO6+ebxHgww8\/nHmjgDHo4MM\/1K+H6nN9rwMA71oCHkHACECJNW5U1wkFEAA4gQGFgALXKlDoNUAbiwCDvAbIC\/Z4vp6dPKd\/9913r1kcmG0RoJ7D14sR2fGznVdeeUUFM9u3b1dD\/pgCQG+AAskogAAgGZ2RCxQoKQWyTSeUlACoLBQoAgUQABQBJBQRChSDAt7XDLm8vEYgyAZExVA3lBEKSFQAAYBEqqgTFIACUAAKQIECCiAAQBOBAlAACkABKFCCCiAAKEHoqDIUgAJQAApAAQQAaANQAApAASgABUpQAQQAJQgdVYYCUAAKQAEogAAAbQAKQAEoAAWgQAkqgACgBKGjylAACkABKAAFEACgDUABKAAFoAAUKEEFEACUIHRUGQpAASgABaAAAgC0ASgABaAAFIACJagAAoAShI4qQwEoAAWgABRAAIA2AAWgABSAAlCgBBVAAFCC0FFlKAAFoAAUgAIIANAGoAAUgAJQAAqUoAIIAEoQOqoMBaAAFIACUAABANoAFIACUAAKQIESVAABQAlCR5WhABSAAlAACiAAQBuAAlAACkABKFCCCiAAKEHoqDIUgAJQAApAAQQAaANQAApAAShQUgpcfnuSZk4cowVrvlpS9fZXFgGAR5GPfOQjJd0YUHkoAAWggGQFbvrty\/RHN75Hf7T4XVXNr\/zvWyJXd2JiIvK1rlyIAMAXABQ7VA5iir0OrnSOuOUAi7gKmrseLMxpGddSGiz4if\/dHw3ShWd30\/wlNerJf9H9HZGrkkYdIhc2z4UIABAA2GhXsElEUm4SEmCChTsUk2TBjv\/C9zrp3WN\/oxw\/O30Tw\/5J1sEmOQQAwgKA06dP07Jly2y2GdgOqABYBBQqgWRgkYDIAbOwzULP7797bJAuvT\/Pv2DNZiq7Y03AEhZOhgCgsEZFl0ICVNudq+igplhgsEhRfF\/WYCGfhXb8\/MTPn\/I71pBpx69VlOAruC4YAcAIgDt3BmElgdNxByhYyGWRbX5\/wb2b1ZC\/rQ8CAFvKpmhXAlTc6FJsQHjqdEd8sBDPwvTCvjCCSfAVJTkCMDY2Rps2bVKsu7u7acOGDRnuEqAiAAjTje2mBQu7+oaxDhZh1LKbNi4LnteffrZTze+bXNgXptYSfEXJBQDT09PU3NxM7e3tinVXVxf19fVRRUWF+lsC1LidK0wnQNr8CoCFOy0ELIqbhXdh3+W3Jq3O7wdRSoKvKLkAgJ\/+e3t7qb+\/n8rLy6mtrY0aGxtp1apVCACCtHqkCaUAnE4ouWIlHv7Jm+r6s9OX6Oz0TMbW69OX1L9nLl2it69+TUsXlanv77ltEd1z60JafdvCWPnj4uAKhOkXepifX+PjD7\/CZ3t+P0hNEAAEUcmxNBwADA8PU09PjyoZBwC1tbWZaYCb\/vRRuu+++xwrdbjivPfee3TDDTeEuwiprSgAFtfKqh1yPsHPXrjitPWHnXqQz9KKK06dP9rB3\/L+d8zi40uXeGxeiQZ04KCuqSijfRtvRzAQROwYaYIEAGnO7wepGgKAICo5lqZQAPCh\/\/hfMyVeuXKlY6UPVpxf\/epXdN111wVLjFRWFQCLa+Wt\/MD8gppXLria5mu\/b+bJfGpqiqqqqnLm\/eIbl2j3D39J5965TFzG3Z+9kT5589WAomChkSCwAnlZXJii2eceJ\/rpD4gWVdG8z32d6FPuPJTV19dn6ilhx9WSeg0QUwCB+ygSGlAgyJOOgWxgIoACQVnwaEPvkdNqZKB17TJqXWvvVbIAxRaZxM8iiY17TAuJEQDTiiZgD4sAExAZWWQUCOp0IJl9BcKy4ADgoeFX1bTAK7s+bb+AJZSDZuGf3+eNe3irXpvv75uSGQGAKSUL2HnmmWdo586dc1L5X98LUxTva4BDQ0OZBYBsQwLUsDe6MNohbTgFwCKcXjZTR2HBowHbh1+lF167SIf\/\/E6sDTAE6PRL\/4MW\/vx5YwfzGCpWKDMSfAVX2NkpAO2oszl7HRT4HXgoglkSS4Aa5UYXVzdcn10BsHCnZcRhwSMBmBKIz9J\/ME\/cE\/nilyi6BQm+wtkAgIfqR0ZGaPPmzXkJDQ4OFkwTBrEEqHFudGG0QtrCCoBFYY2SShGXhZ4SmP7WvUkVWUw+\/AqfPpiHh\/f\/bX0XLftsY1HXT4KvcDYASKtlSIAa90aXlvYS8wULd6iaYPHCzy\/SuidfptW3LqTDD93pTuUcLEm+jXtMsEi7yhJ8hdMBwKlTp2jLli107tw5tWUvf3gtQGVlJQ0MDNDy5cuNtwEJUCV0LuNgUzIIFikJnyVbUywQBORnGmTjHlMs0mxdEnyFswHAzMxMZpe+FStWUFNTE23cuFFt2MPz\/6Ojo2ozH97Nz+RHAlQJncsk0zRtgUWa6s\/N2yQLXhz4iUf\/iTAdcFXjMBv3mGSRVguT4CucDQB4DUBnZyd1dHSoffp5+966ujq1Yt\/\/m8kGIAGqhM5lkmmatsAiTfXtBQBsWb8hwP8u5ekA\/8I+fo2PF\/fl+0joFxJ8BQIAXyuVAFVC53LHbcQrCVjE08\/k1TZY6CCAty4uxb0Czu2+l6IczGODhcm2EsSWBF+BAAABQJC2jjQRFZBwo4tYdecus8WilIKAIPP7QcDbYhEkb1NpEACYUjKLHR7m53n\/8fHxrLnwPv18op8+xtdUUSRAldC5TPFM2w5YpE3gav62Wax74mWSOhIQZn4\/CHHbLIKUIW4aCb7C2RGAuHCiXi8BqoTOFZWfa9eBhTtEkmChgwApJwpGmd8PQjwJFkHKESeNBF\/hbACAEYDoTVNC54pee7euBAt3eCTFothHApI4mCcpFjZbHwIAm+p6bPNrf5OTk9Ta2qq+5b\/5w68Emv5IgCqhc5nmmpY9sEhL+WvzTZJFMY4EZDuYZ8lDA1YAJsnCSgWEnBvj7AiAhpbtlT+8Bpi\/SUvoXLY6bdJ2wSJpxXPnlzQLDgKK4RAh0\/P7QYgnzSJImcKmkfCw6HwAoDcEqq2tzTzx854AvDsgNgLK3mQldK6wndHV9GDhDpk0WLgcBKR5ME8aLEy3RAQAphXNYc+\/HiDoGwDeY4S913iPA\/afNCgBqoTOlVDTsp4NWFiXOHAGabHQJwm6cpyw\/2CepU+eDqyhqYRpsTBVfrYjwVc4PwIQFRifI9DV1UV9fX2ZnQR51KClpYV27dpF7e3tyrQ3jRSoEjpXVO6uXQcW7hBJk0XaQYBe2Hfhe50KSPkda2jBms1UdseaVAClycJUhREAmFIyix3TxwHzU\/\/w8DCtX7+e\/uqv\/krtIcDnCLS1tVFjY6PaYhgBgEWgJWpawo1OCrq0WejjhJ9ovJ0a774pEVmzze8vuHcz8ZG8aX7SZmGi7ggATKiYx4YeqvcP0\/Mlenh\/aGgo47zzFYfXDdTU1FB1dbUKBHj9AH84APCuL5AAVULnsty0EjMPFolJXTAjF1gkFQT4HX8aw\/z5gLjAomCDKZBAgq\/gKs6bnZ2djSuGzeu9c\/k6n2xBQa4yeF8j1CMB+QIAbefo0aM2q2XN9tTUFFVVVVmzD8PBFQCL4FrZTukKixffuERfO\/QmNdx+A+3+7I1mq\/3TH9Dsiz8gem2MaFEVzfvc14k+dZ\/ZPAxYc4VFlKrU19dnLpuYmIhiwqlrnA8AgqjFc\/5btmxRbwc0NDRk3hDQT\/56zwAOAPg7TAEEURVp4iog4UknrgauXO8Sixd+fpHWPfmymgrgKYE4H+\/GPVEO5omTd9RrXWIRtQ4YAYiqXELXeY8Q1lny2oLm5mYsAkyIQalnI+FGJ4Whayx0ELD61oWRjhNO4\/19U23BNRZR6oUAIIpqCV3jfdVPZ6lHBviAoU2bNqmv\/WsIJECV0LkSaibWswEL6xIHzsBFFlGCgDTf3w8sdoGELrIIWzcJvoLrLGIKICy8XOklQJXQuUzxTNsOWKRN4Gr+rrLg44Q\/8eg\/Ub6RAP\/+\/Pz6Hr\/Gt2DNV90ROERJXGURogrYByCMWHHSejcCOnjwID3\/\/PO0efNmWr58eRyzWa9FAGBc0pI2KOFGJwWgyyw4CNg+\/Oo1xwknuT9\/kpxdZhFUBwm+wvkRAO9WwHwgUF1dneKjX+Xjd\/lNfiRAldC5TDJN0xZYpKn+3LxdZ+ENAvg44TteepwuPLtbvbPPT\/qL7u9wR8yYJXGdRZDqSfAVzgcA3oN\/nn76aRUArFixgjo7O6mjo0Pt8mfyIwGqhM5lkmmatsAiTfWLKwDQw\/z\/cPBx+uSvXlGOn51+sQ7z5yMvoV9I8BXOBwDZRgCOHz+Ow4Dy9C4JncsdtxGvJGARTz+TV7vKItsw\/5O\/qqPHp6rJlfMDTHJgW66yCFNPBABh1IqRNuphQFGylABVQueKws7Fa8DCHSqusfCv5vfv1pf2+QE2ybnGIkpdJfgK50cAooCJc40EqBI6VxyGLl0LFu7QcIWF\/zS+fMP8vUcmqffIabVZUFLnByRBzBUWceoqwVc4GwD4n\/r9oIIeCRwWsASoEjpXWG6upgcLd8ikySLbpj3ld9QFOo1Pnx\/QunYZta5N9xAfUzTTZGGqDhJ8hbMBgBeSdy9\/\/p7\/5o\/e3tcUULYjAaqEzmWSaZq2wCJN9efmnTQL\/7v7cRb1mdw62AUiSbOwUWcJvsL5AMD7FoBe8Z\/tO1OAJUCV0LlM8UzbDlikTeBq\/kmw0E7\/0onjxEP97PTL71hDSx4aiC1ElF0DY2dqyUASLCwVPWNWgq9wPgDwvgWgn\/h5j38+9IdP9MM+ANc2cwmdy3bnTco+WCSldOF8bLO48L3OzHv77PTL7qgz\/gqflCDANovCrSF+CgQA8TUMZAFvAQSSKZNIQucKV2N3U4OFO2xMs\/C+vsf\/TmrDHr118NKKMnpl16fdEThESUyzCJG1saQIAIxJ6Y4hCVAldC53WkS8koBFPP1MXm2CBQ\/r6+F9Lpt2+kEX9Jmqj3\/XwNW3LTRlOhE7JlgkUtA8mUjwFc5PAeR6GwBvAeRumRI6V9qd21T+YGFKyfh2orLwvrbndfoubM277omX1fkBvHVwMQUBUVnEbwXmLCAAMKdlKEv8FkB1dTWtWrUq1HVBEkuAKqFzBWFVDGnAwh1KhVjwMP7ltybp129P0rvHBunSiWOq8Hohn405fRPqcBDwwmsXi2rXwEIsTOhi24YEX+H8CEA2iGHfAjh16hS1tLTQnj171AmCY2NjtGnTJmW6u7t7zuuEEqBK6Fy2O29S9sEiKaUL5+NnwU\/2\/OEhfXb62Rz+by2pCfSufuHc7aYotl0DJfQLCb6iKAMAduD8JkB\/f3\/Bw4D0WwQvvvgiDQwM0OLFi6m5uZna29tVj+zq6qK+vr6MHQlQJXQuu7fL5KyDRXJaZ8uJnfzlt86QWrD3+s9o\/jtvqn\/rDz\/dz\/9QDV1x9OZX7SdZ+2LaNVBCv5DgK5wPAHKtAfA\/uefqaDxd8PbbbxMHAOz0z58\/nwke+BXCtrY2amxszEwnSIAqoXMleeO0mRdYxFNXO3Btxeu8+amdPzxsr\/7vcexeB8\/\/Zid\/qXwxLbhhgXL0xfJkH1Y9vWsgbxvM2we7+pHQLyT4CucDgDgNmIf+BwcHadu2bbRr165MADA8PKz2EOAPBwC1tbWZaYCJP5sXJ0tcCwWggEkFFlXNtVbh+XvRzVd\/W1RF83TaT92XtQRTU1NUVeWzZ7Ksjth68Y1L9LVDb9Inby6jp9bf5Eip5hajmFnU19dnKjMxMeGkvmEKNW92dnY2zAVJpo26EyAP\/T\/22GO0efPmOcP+PAKQLwCQENVJiK6TbGM28wILm+qGs11KLFzfMEgCCwm+wtkRAD13PzIykrWXNzQ0zNkJkJ\/2t2zZonYI5N8eeOAB9eTPf+tPZWUl\/cVf\/AV95zvfUesHMAUQ7gaK1OEVkHCjC19rN68oNRZ6w6DVty6kww\/d6RQUCSwQACTQpMKu+M9WJLahF\/5hEWAC0JBFRgEJNzopOEuRhXfDoMN\/fifx7oEufCSwQABgsSVpx\/\/www\/Tjh07aHx8fE5uYTYC8gYA\/tcAh4aG5uwnIAGqhM5lsWklahosEpU7b2alysLFXQMlsJDgK5ydAkjrtiEBqoTOlRZ\/0\/mChWlFo9srdRYubRgkgYUEX1EUAQC\/yrdz587IIwBhbhkSoEroXGGYuZwWLNyhAxZErmwYJIGFBF\/hfACg9wFobW21svWv\/\/YkAaqEzuWO24hXErCIp5\/Jq8HiipoubBgkgYUEX1EUAUBnZyd1dHQU3PXPxM1CAlQJncsESxdsgIULFK6UASyustAbBrWuXUata2sShySBhQRf4XwAwAXkKQD+bNiwwXpDlQBVQueyDjqhDMAiIaEDZAMWc0XSewWksWugBBYSfIXzAQCOAw5wZ\/MlkdC5wtfazSvAwh0uYHEti7Q2DJLAAgGAO33bWEkkQJXQuYwBTdkQWKQMwJM9WGRnkcaGQRJYSPAVRTsCwAXnnf34hD9+t9\/URwJUCZ3LFM+07YBF2gSu5g8WuVkkvWGQBBYSfIXzAQAXkNcATE5OEr8JoP\/m\/1dXV6t9\/R9\/\/HFjdxkJUCV0LmNAUzYEFikDwAhAYABJbhgkoV9I8BXOBwD5DgPiXQL37t2LAMDXxSV0rsB3LccTgoU7gMAiGIskNgySwAIBQLD2FCuVPhSIh\/u9IwCjo6PU0tKiDvbR38fK6P2LJUCV0LlMsHTBBli4QOFKGcAiOAvbGwZJYCHBVzg\/AsAF9L8JwOcA7Nu3j\/bs2UO1tbVGXw+UAFVC5wp+q3I7JVi4wwcswrGwuWGQBBYSfEVRBADhmm281BKgSuhc8Si6czVYgIU7CoQvid4wyPReARL6hQRfURQBQG9vLx04cGBO6w1zGmCYZi8BqoTOFYaZy2nBwh06YBGNhY29AiSwkOArnA8AvEf5vvTSS2rl\/5kzZ1RLLrQzoF4\/MDIyMueVwbGxMdq0aZOy0d3dPceOBKgSOle0W5V7V4GFO0zAIjoL00GABBYSfEVRBAD6LICTJ0\/S8ePHaevWrRTkfAAeOaipqVEOnp2+vra5uZna29tVb+jq6qK+vr7MOQMSoEroXNFvVW5dCRbu8ACLeCxMbhgkgYUEX+F8AMBP8fyqHzv98+fP05YtW+jcuXNUaAog2+uDXFkOBDgw6O\/vp\/Lycmpra6PGxsbMSYMSoEroXPFuVe5cDRZg4Y4C8Uvi3SvglV2fjmxQQr+Q4CucDwC4gOPj43T99derHf\/Yge\/YsaPgDoA6AODrvVMAHETw5kE9PT2q8XIA4H2TgKHqz9GjRyM38DQvnJqaoqqqqjSLgLzfVwAs3GkKYGGGxbl3LtPuH\/6SfvHuZdr92RvpkzeXhTZczCzq6+sz9Z2YmAhdd9cumDc7OzvrWqHilufUqVNqtOCb3\/ymerrn3QR574D169fToUOH8gYAxQ5VQnQdl78r14OFKySwD4BpEnE2DJLQLzACYLpFxbCnHT5PDzQ0NKhNgnbt2qXm+vXIAQ\/9b9u2jfbv348pgBha49LgCki40QWvrdspwcI8n6gbBklggQDAfHvKWMx1DLBOUGgNAKfzLgLUIwDewIDTYBGgRYgwjd3nHGoDEpyOQ3JmihJlwyAJLBAAWGyN\/gBgaGgos1AvaLZeG96AwfsaoN+uBKgSOldQxq6nAwt3CIGFPRZhNwySwEKCr+AW4fwagGxbAfMq\/oqKCuMtWgJUCZ3LONiUDIJFSsJnyRYs7LIIs1eABBYSfEVRBAD+Zut9lc90ECABqoTOZfdWlZx1sEhO60I5gUUhheL\/HjQIkMBCgq8oigDAu8CPC8yL\/Pg1Pn6P3\/RHAlQJncs017TsgUVayl+bL1gkw0JvGLS0ooxy7RUggYUEX+FsAJBr\/t52E5YAVULnss05KftgkZTShfMBi8IamUrh3TBo38bbafVtC+eYlsBCgq8oigAgW6MM8hZAlMYsAaqEzhWFnYvXgIU7VMAieRa8V8DZC5fIHwRIYCHBVzgbACTfVK\/kKAGqhM6VFn\/T+YKFaUWj2wOL6NrFuTLbhkESWEjwFQgAfC1bAlQJnSvODcela8HCHRpgkR4LvWHQE423U+PdN4nYH0OCr0AAgAAgvbtCCeQMp+MOZLBIl4XeMKh17TK6f8UsLVu2LN0CxcwdAUBMAV28XAJU3OjcaVlgARbuKJB+SfSGQQ2330CDW+9Ov0AxSiDBV2AEACMAMboALi2kAAKAQgol9ztYJKd1vpyC7hXgRmlzlwIBgOuEIpRPAlTc6CKAt3QJWFgSNoJZsIggmqVLvv\/Cq\/S1Q2\/S6lsX0uGH7rSUi12zEnwFRgAwAmC3l5S4dTgddxoAWLjF4jc\/+Du07smXVaFybRjkTomvLQkCAJfpRCybBKi40UWEb+EysLAgakSTYBFROAuXaRaFNgyykLUxkxJ8BUYAMAJgrEPA0LUKwOm40yrAwl0WuTYMcqfEGAFwmUXWsnnPEMh1HHB3dzdt2LAhc72EqA43OneaKliAhTsKuFOSbP0i24ZB7pQYAYDLLK4p28zMDLW1tVFtba1y8L29vSrN1q1bqbm5mdrb29XfXV1d1NfXlzlaWEIAIKEORdXY8hQWLNwhCRbus\/BvGOROiREAuMwia9nY6dfU1GQCAP53dXW1Cgb6+\/vVaYIcJDQ2NtKqVauUDQk3CQl1KLrGlqPAYOEOSbAoDhZ6rwDeMKh1bY07hfaVREp7mjc7OzvrrMoxC8bO\/sCBAzQ0NKSc\/NjYGA0PD6vjhPnjHSXQAUDMLHE5FIACUAAKxFDg8o0fpfdWt9DC\/94Uw4r9SycmJuxnYjkHkQGAPk64tbVVOX49BVBXV5c3ALCsNcxDASgABaBAAAV4wyD\/McIBLkOSkAqICAC8C\/4aGhrogQceoG9961uZ+X1+8ucgYNu2bbR\/\/\/6cUwAhtUNyKAAFoAAUgAJFq4CIAMCvvn8E4JlnnqHR0VFqaWmhXbt25VwEWLQUUXAoAAWgABSAAiEVEBkAsAZBXgPUawNCaobkUAAKQAEoAAWKXgGxAUDRk0EFoAAUgAJQAApYVAABgEVxYRoKQAEoAAWggKsKIABwlQzKBQWgABSAAlDAogIIACyKC9NQAApAASgABVxVAAGAq2RQLigABaAAFIACFhVAAGBRXJiGAlAACkABKOCqAggAXCWDckEBKAAFoAAUsKgAAgCL4sI0FIACUAAKQAFXFUAA4CoZlAsKQAEoAAWggEUFEABYFBemoQAUgAJQAAq4qgACAFfJoFxQAApAASgABSwqgADAorgwDQWgABSAAlDAVQUQALhKBuWCAlAACkABKGBRAQQAFsWFaSgABaAAFIACriqAAMBVMigXFIACUAAKQAGLCiAAsCguTEMBKAAFoAAUcFUBBACukkG5oAAUgAIlrMCF73XGqv3ltydjXV\/o4iUPDRRK4vzvCAA8iD7ykY84DwwFhAJQAAqYUmDt4veUqQ9fd5k+\/Nu\/zpi96bcvz8mCf+eP\/3tT5fDaefP\/zTdi9v\/+yoydXIX5D\/8wY6ScaRpBAOALACYmJtLkETtvDmKKvQ6xRXDEAFg4AoKISoHFu8f+Rgl++a0zxE+\/v37\/CfjyW5Pq71yf+UtqMj\/N\/9CVf\/+W5zv+W6eZ\/6EJd12nAAAgAElEQVRq9fuCNV+NDFcCCwl1YIAIABAARO7IuDC\/AlJuEhI4S2HBTv7SieMKCTv4SyeOzcFz1VHXZJw4f7fo\/g5nMEpgIaEOCAB8XUIC1NOnT9OyZcuc6eylXBCwcId+sbHQjt7v5LWDL79jjRK3GOehi41FtlYswVcgAEAA4M4dWmBJJNzopGBxlQUPzfMQ\/cyJ4zTzz8cyT\/ReR1+MTj5fu3GVRZi2jgAgjFpFklYCVAmdq0iaS8FigkVBiRJL4AqLfA6fn+rL7qhTQ\/dl7z\/hJyZQghm5wiJOlSX4CowAYAQgTh\/AtQUUkHCjkwI5TRbs9GdO8NP9cdIL9fgJXzv8OAvqipFPmixM6YUAwJSSDtmRAFVC53KoScQqCljEks\/oxUmz0E7\/3WODaljf6\/ClP+EXApc0i0LlifK7BF+BEQCMAERp+7gmoAISbnQBq+p8sqRYsLNnp89P+qX8lJ+vQSTFwmajRABgU92UbEuAKqFzpYTfeLZgYVzSyAZts+Bd6zLD+x+qoQVrNsd6Vz5yRYvgQtsskpBAgq\/ACABGAJLoKyWbh4QbnRR4NljwML92\/Py0z3P5C+7dnNk0R4p2puthg4XpMhayhwCgkEIp\/z42NkabNm3KlKK7u5s2bNhA3u\/1dzqRBKgSOlfKTcdY9mBhTMrYhkyx8M\/t82p9PO2Hw2OKRbhczaaW4CtEjwA888wzNDk5Sa2trRny09PT1NzcTO3t7eq7rq4u6uvro4qKCvW3BKgSOpfZrpqeNbBIT3t\/ziZYeIf5eQW\/tPfzk6JlgkVSZc2VjwRfIToA6O3tpZqaGvXUrz\/89M\/f9\/f3U3l5ObW1tVFjYyOtWrUKAUDaPUpg\/hJudFKwRGXBT\/zv\/miQLjy7Ww3tY5g\/fouIyiJ+zuYsIAAwp6VxS\/yk39TUROPj48r2ypUrldM\/efIkDQ8PU09Pj\/qeA4Da2tpMkOA9DfDo0aPGy5WEwampKaqqqkoiK+RRQAGwcKeJhGZxYYpmf\/oDouceJ1pURfSp+2je577uToWKuCShWThU1\/r6+kxpJBy6VhKHAfF0wOjoKK1fv54OHTqUNwAodqgSomuH+nusooBFLPmMXhyURbYnfpcO0jEqSkrGgrJIqXiBssUIQCCZ3Eikh\/63bdtG+\/fvxxSAG1jEl0LCjU4KpEIs4PiTI12IRXIliZ4TAoDo2lm\/kqcAOjs7qaOjQy3w43l\/\/mzduhWLAK2rjwy0AhJudFJo5mLhd\/xLnzwtpcrO1kNCv0AA4GzzulIw7+t+eg0ABwPe74eGhjILAPkaCVAldC7Hm1bg4oFFYKmsJ\/SzwBO\/dclzZiChX0jwFQyoJNYABG3qEqBK6FxBebmeDizcIaRZZNu8B3P8yXKS0C8k+AoEAL52LwGqhM6V7O3IXm5gYU\/bsJaZxXWD\/ylzMA+G+sMqaC69hH4hwVcgAEAAYK5Xw9I1Cki40RUz1jm79p37OZVV3kYVf9ZBvHsfPukpIKFfIAAw3H74Vb2dO3fOserfqtdwlteYkwBVQueyzTkp+2CRlNJz88k2v3\/xtj+kZXd9Jp0CIdc5CkjoFxJ8hRMjAHpRXjZnr4MC\/2I9W\/1JAlQJncsW36TtgkVyivv36Odd+7zD\/GCRHItCOUlgIcFXpB4A8Ot6IyMjtHnz5rxtZnBwsGCaQo0uyO8SoEroXEFYFUMasLBLSTv9SyeOq6N42enzHv18OI9\/mB8s7LIIY10CCwm+IvUAIEyjSSKtBKgSOlcSrJPIAyzsqMzO3u\/0y+6oU\/v05\/qAhR0WUaxKYCHBVzgTAPj37vc2qsrKShoYGKDly5dHaWuhrpEAVULnCgXN4cRgYQ4On8Q388\/HMqv4+Um\/kNP35g4W5ljEtSSBhQRf4UwAwAXxH9\/Lf\/OnurpaHeDz+OOPx213Ba+XAFVC5yoIqkgSgEV4UDysf\/mtSZo5cTzj8NlKvuH9ILmARRCVkkkjgYUEX+FMAODfupcLpr97+OGHae\/evQgAAvZNCZ0rYFWdTwYWhRFdOnGMfv32pBrSv\/L\/Y+oi7fD5Kf+3ltTEfnUPLAqzSCqFBBYIAAy2lpmZGXU0Lw\/3t7a2Ksv6BL+Wlhb6zne+k\/k+brberYD9bx5IgCqhc8Vl7Mr1YHGFhF6sx\/\/2O3rt7Od\/qEY5+iUPDVjBBxZWZI1kVAILCb7CmREA\/cTf1NRE4+PjqlHx\/v379u2jPXv2UG1tLW3YsCFSY\/NexKMKzc3N1N7err7u6uqivr4+dWAQfyRAldC5YoN2xECpseB5enb2\/CR\/xdlfeZrXH36q147e1JN9UNSlxiKoLmmkk8BCgq9wNgA4ePAgPf\/88+rVP5OL\/\/SxwP39\/VReXq5GHRobGzMHAkmAKqFzpXFTspFnMbHglfX6c\/mtM+qf7Mz5ox06z817v\/c7d\/5bO3j+d9JOPh\/DYmJhoy26ZFMCCwm+wpkAQE8B8JP+5OQk1dXVqfbKi\/96enqUszbx4QBA22R7HAB4RxckQJXQuUywNmXD6xiV83vfOQaxf+HiBVq0cFHepNrJ5kqknW+237VDzvrb+847SDmzOXLtzPn\/PDSv\/l5SQ8V6cA76RdiWYC+9BBYSfIUzAYB3EeDTTz+tAoAVK1ZQZ2cndXR0ZIbo4zbJQgHAxJ\/Ni5sFrheswLn5N1mp3S9+M7fdc\/M\/nDPPX+Qpj9fmuRV\/TJUfmJ+xU7ng6r+\/9vsLrdTJNaNTU1NUVVXlWrFKsjzFzKK+vj7DbGJiouj5OXEccLYRgOPHj9O5c+eMjwD09vYSpgCKvt0WRQWSeNLpPXJlWN77OTs9M+fv16cvzf39wtW\/z\/p+yybs0oqyzNdLF5XRLRVltLTiyqgc\/9Z4t53AyCTkJFiYLK9kWxJYYATAcAv1bwbEiwDZUesFeiaywyJAEyrCRlAFJNzodF11oMHBhQ4ozl64RNkCCA4KvIEC\/33LojJafVt6ow2SWARtf66mk8ACAYCrratAubyvAfoPGZIAVULnKtKmdU2xS5HF8E\/eVEGBDhReeO3iHF10cHDPbYvU6EFSgUEpsnC1H0lgIcFXcPtIdQog3xbAXDgbowD5OoUEqBI6l6s3rrDlAou5ir3w84v049cuXhMceIOCe25daGWkACzCtl576SWwkOArUg8AvE0s11bAJt7\/D9qUJUCV0LmC8nI9HVgEI6SnF3qPnFYXeAOC1rVX3kCI+wGLuAqau14CCwm+wpkAIN9WwCbfAijUhCVAldC5CnEqlt\/BIhopDgh4CoGnE3RA0Hj371CcYAAsorGwcZUEFhJ8hTMBgPctAP3Ez6v1Tb8FUKgxS4AqoXMV4lQsv4NFfFK8noADAT060Lp2mXrrwPtmQpBcwCKISsmkkcBCgq9wJgDggiTxFkCh5i0BqoTOVYhTsfwOFmZJ8RqC4Z\/8QgUEV14\/DD4qABZmWcSxJoGFBF\/hVAAQp0GZulYCVAmdyxTPtO2AhR0C3lEBDgT2bby94MJBsLDDIopVCSwk+IrUAwB+6h8ZGVF7\/uf7DA4OFkwTpSH6r5EAVULnMsHSBRtgYZcCBwI8NcAjAqtvXUj7Gm\/POTUAFnZZhLEugYUEX5F6AMAF0O\/l+4\/m5d\/4zYCdO3eS\/339MI0tTFoJUCV0rjDMXE4LFsnQ4UBg3ZMvq\/0HeI1AtsWCYJEMiyC5SGAhwVc4EQDoBqOdvbcBZQsKgjSwqGkkQJXQuaLyc+06sEiWCL89wCMCPBpw+KE752QOFsmyyJebBBYSfIVTAYALzVMCVAmdy4W2YKIMYGFCxXA2eKEgjwbw2oBXdn06czFYhNPRZmoJLCT4CgQAvlYuAaqEzmXz5pOkbbBIUu2refFUwPbhV4nPKtALBMEiHRbZcpXAQoKvEB0AePf854rq6QTv9\/4pBglQJXQud25V8UoCFvH0i3v1uideVkEAjwSARVw1zV0vgYUEXyE6APBvLcyVxWmA5joxLBVWQMKNrnAt3U6hg4BdaxbSn66+3e3ClkjpJPQLBACGG6t3I6CDBw\/S888\/r179W758eaSceCfBmpoa8p4lwE\/\/\/D0fM1xeXk5tbW3U2NhIq1atUnlIgCqhc0UC7uBFYOEGFA4CJt5+j\/7P7s+4UaASL4WEfiHBVzgzAuDdCnhycpLq6upUFxkeHqaenh7lrMN8cu0qePLkyYxNtscBQG1tbSZIYKj6c\/To0TBZOpN2amqKqqqqnClPKRcELNygf+6dy9T07Bt0y6Lr6Kn1N7lRqBIuRTH3i\/r6+gy5iYmJoqeY6nHAWj3vYUBPP\/20CgBWrFhBnZ2dZOIwIJ4OGB0dpfXr19OhQ4dUUJErACh2qBKi66LvVe9XACzcIfnj8VPUMDiV9RVBd0pZGiWR0C8wAmCwrWYbATh+\/Hjgw4BOnTpFW7ZsUekbGhquGTXQQ\/\/btm2j\/fv3YwrAIDuYyq2AhBudFL7M4o1\/W6ReETz853cW3DpYSr1drIeEfoEAwHDLMnkYkP94YZ7358\/WrVupubmZ2tvb1d9dXV3U19dHFRUV6m8JUCV0LsNNKzVzYJGa9NdkrFnweoAXXrtI09+6153ClVhJJPQLCb6Cm50TUwBcEL1qn510U1MTjY+Px9oC2Pu638qVK9VTPzt67\/f+LYYlQJXQuaTcD8HCHZJeFp949J9o6aKya3YLdKe0sksioV9I8BXOBAA8BfDYY4+pVf8vvfQS8UJAnq\/nQ4AeeeSR0IsAo3YfCVAldK6o\/Fy7DizcIeJloXcLxFRAOnwk9AsJvsKZAMC\/CJBf3\/vc5z5nbBFg0GYuAaqEzhWUl+vpwMIdQtlYVHzjR5gKSAGRhH4hwVc4EwDoEYAvfelLdODAgcwcPUYAwvdOCZ0rfK3dvAIs3OGSKwDIdXqgOyWXVxIJ\/QIBgOF2qefmH3zwwTmL9aJuBBSleBKgSuhcUdi5eA1YuEMlG4vhn7xJDw2\/qrYK5sOD8ElGAQn9QoKvcGYEIJlmVzgXCVAldK7CpIojBVi4wykXC14QeM+tC+mJRmwTnBQtCf1Cgq9wKgDgV\/V4+N\/78a7eT6JxSoAqoXMlwTqJPMAiCZWD5ZGLhR4FwILAYDqaSCWhX0jwFc4EAN5DevgtgOrqajpz5oxqa969\/E00vnw2JECV0Llsc07KPlgkpXThfPKx4L0B+HP4oTsLG0KK2ApI6BcSfIVTAYDe9pf36+ddAHk\/AFNbAQdtsRKgSuhcQXm5ng4s3CFUiAW\/EYBRgGR4FWKRTCni5SLBVzgTAPBbAHv37lVO\/\/z585ltfTEFEL6RSuhc4Wvt5hVg4Q6XQiwwCpAcq0IskitJ9JwQAETXLuuVvPPf9ddfr47\/5TcCduzYQQMDA5GPA45SPAlQJXSuKOxcvAYs3KFSiAU2B0qOVSEWyZUkek4SfIUzIwDRMVy9Um8l3Nraqr70ni3gPSDIuxVwd3f3nDUGEqBK6Fwm2oMLNsDCBQpXyhCEBb8S+OPXLqrXAvGxp0AQFvZyN2NZgq9wKgBgB75z5845dIJOAeg3CHgPAR0A8He8o+C6deuora2NGhsb1RHDOAzITAeAlcIKSLjRFa5lcaQIwuLs9CXi1wL5lcDGu28qjooVYSmDsHC9WggADBLST+vsvFetWhXKMgcO\/NYALxzkD9vw29OjA3V1dcSBAR8MVF5engkMdJ4SoEroXKEagMOJwcIdOEFZYBTAPrOgLOyXJHoOEnyFMyMA\/uN7o2DRR\/7qAEA\/6fOaAg4ARkdH1QFDhw4dop6eHpUFjwzU1tZmpgEkQJXQuaLwd\/EasHCHSlAWGAWwzywoC\/sliZ6DBF\/hTADABWEnzZ+o7\/2bCgB0kzh69Gj01pHilVNTU1RVVZViCZC1VgAs3GkLYVjs\/uEvaeTV9+jF\/1LjTgUElSQMC9eqXV9fnynSxMSEa8ULXZ55s7Ozs6GvMnSBd6FeNpP+NQD8uiA\/tY+MjFBlZeWctwT8AUBTU5OaDuDhfUwBGAIGM6EUkPCkE6rCDicOy4L3BcBBQXaAhmVhpxTxrGIEIJ5+xq\/2BgBsHIsAjUsMgyEVkHCjC1llZ5OHZYGDguyhDMvCXkmiW0YAEF27OVeeOnUqs\/GP93W9sOb9AYB3dMH7doD3NcChoaE5iw4lQJXQucKydzU9WLhDJgoLHBRkh18UFnZKEt2qBF\/BtU91CkAP6fMrejxUr5\/ao64DiI7zypUSoEroXHE5unI9WLhCItg+AP7S4qAgO\/wk9AsJviL1AMC\/+p9HA44cOULbt2+30\/IKWJUAVULnSgW+hUzBwoKoEU1GZYEtgiMKnueyqCzMlyS6RQm+wskAYHBwkB555BH1nn7SHwlQJXSupLnbyg8sbCkb3m5UFnqLYGwOFF7zXFdEZWGuBPEtSfAVCAB87UACVAmdK373dMMCWLjBgUsRh0XvkUnqPXIapwUawhmHhaEixDYjwVc4EQDw63p8EFC2T9CtgGPTfN+ABKgSOpcpnmnbAYu0CVzNPy4LTAWYYxmXhbmSRLckwVekHgBEl9\/OlRKgSuhcdugmbxUsktfc1rCzngrA3gDxmUroFxJ8BQIATAHE782wkFMBCTc6KXhNsNBTAdPfuleKLKnUwwSLVAruyRQBQNoELOQvAaqEzmUBbSomwSIV2bNmaooFTwWcvXBJrQdYWlHmTgWLqCSmWKRZZQm+AiMAGAFIsw+Jz1vCjU4KJJMsdBDwyq5PS5En0XqYZJFowTECkJbcyeQrIaqT0LmSoW0\/F7Cwr3HQHEyy4BMDtw+\/qrI+\/NCdQYuAdO8rYJJFWqJK8BUYAcAIQFr9pyTylXCjkwLKNAt9bPDqWxciCAjZSEyzCJm9keQIAIzIaM6IPvGPTwDkj3fPf\/67u7tbHTXs\/V5\/p0shAaqEzmWuVaRrCSzS1d+buy0WfGogrwXAdEBw1rZYBC9B\/JQSfIWYEQA+Q+DAgQPkPfTHHxBwZXnr4ebmZmpvb1ctoKuri\/r6+qiiokL9LQGqhM4Vv3u6YQEs3ODApbDFQk8H8MJABAHBeNtiESx3M6kk+AoRAQA7+urqajp+\/Lgiq0cAsh0sxE\/\/\/H1\/f7\/aaritrY30QUQIAMx0DFi5qoCEG50UnrZZ8MLAF167SNgnoHCLsc2icAnip0AAEF9Doxa8xwF7jwLmTPSOgidPnqTh4WHq6elReXMAUFtbq6YGdACgC3X06FGj5UvK2NTUFFVVVSWVHfLJowBYuNM8kmDx1P+8SAf+10X65M1l9NT6m9ypvGMlSYKFrSrX19dnTE9MTNjKJjG7qR4HbLKW3gDAb5dHCUZHR2n9+vV06NChvAFAsUOVEF2bbBdp2gKLNNWfm3dSLPSUAEYDcrNPioXN1ocRAJvq5rA9MzOjntpHRkaosrKSBgYGaPny5Sp1vgBAD\/1v27aN9u\/fjymAFNiVYpYSbnRSuCXNQu8aiAWC17agpFnYaMMIAGyoGsOmfwqgs7OTOjo61AI\/\/dvWrVuxCDCGxrg0nAISbnThauxu6rRYeAOBxrt\/hxrvvqnkdxBMi4XJ1okAwKSaBmz5RwC8r\/t5TxX0fj80NESrVq3K5C4BqoTOZaA5OGECLJzAoAqRNgsOBIZ\/8gviKQIeFeBgoHVtjTsCJViStFmYqKoEX8E6iFkDAKhXFJDQuUywdMEGWLhAwa1+wQHA8E\/epN4jp1XBdDBQSiMDEvoFAgB3+raxkkiAKqFzGQOasiGwSBmAJ3sXWfARwz9+7eKcYOCeWxfS0opy4v+vvm2hOwIaLImLLMJWT4KvwAiAj7oEqBI6V9jO6Gp6sHCHjOss9MjAj39+Qe0noEcH+P\/SpgtcZxGk1UrwFQgAEAAEaetIE1EBCTe6iFV37rJiZMHrBvjjDQp0YLB0URndc9si9XuxrSUoRhb+Bo0AwLkuHr9AEqBKqEN8km5YAAs3OHApJLDgUQKeMuD\/n52eodenL2VGC7xK87oCDhBuqSh7fySh3KlAQQILCXXACIDAEQApDdMd1xG9JGARXTvTV5YKC15gyAGCHjng\/\/M5Bfq7bLpywKA\/HDjwRwcPV0YcrgQQmTTqLYboOx1KYCGhDggAsgQApm88sAcFoAAUcFmBSx\/740zx\/v36xerf\/379jdd85\/\/eVp1+419\/acT0b\/zreSN2chl567\/9Z6v2kzCO1wCTUBl5QAEoAAWgQCgF9BqIUBd5EvM0ic3PE4232zSfiG0EAInIjEygABSAAlAACrilAAIAt3igNFAACkABKAAFElEAAUAiMqebCR+i9Oyzz9K\/\/Mu\/0AMPPEDLli1Lt0AlnPuFCxfob\/\/2b4mPRN24cSN94hOfKGE13Kj64cOH6ZZbbqE777zTjQKVYCkuX75M3\/\/+9+mnP\/0pffnLXwaLhNoAAoCEhE4zm3\/8x3+kxYsXq5MT2fl89atfpfLyuSt70yxfKeXNR1N\/7GMfU\/899dRT9JWvfIUWLbryPjc+yStw8uRJevrpp+m+++6bcy5I8iUp7Rx\/+MMf0nXXXUe\/93u\/R3\/\/939Pn\/\/853GPSqBJIABIQGRbWUxPT2dON9THIrOD2blzp8pSH3bEjuYLX\/iCesoZHBykhoYGdUoiPuYUCMpC58gjAd\/+9rfpwQcfpBtuuMFcQWCJgrJ477336O\/+7u\/owx\/+sGLgPRgMMppRICgL7gscAJw4cUIFxR\/\/+MfNFABW8iqAAKBIG8ipU6doy5YtqvQDAwPq6Z6\/6+rqor6+PuInm+HhYerp6VH\/Z6e\/ZMkSBAAWeIdhwSMv7Pz37t1LjY2NtGLFCgslKl2TQVl0d3fTsWPH6KMf\/Si9\/fbbSjAEAGbbTVAWfI968skn6TOf+Qz97u\/+Lh04cIA2b96MkTGzOLJaQwCQgMims+ComocteZhs9+7dtGfPHhUA8NP\/6Oiocvo879\/c3Ezt7e0qquaIuqamRgUAGzZsoA984AOmi1WS9sKy+OAHP6jY8TTMzTffXJKa2ap0GBY7duygV199lV5\/\/XV64403lLP5+te\/jtEYQ3DCsOB71AsvvED33HOPuo\/xA82f\/MmfYJTSEIt8ZhAAJCCyrSw4wm5paZkTAExOTlJra6saBm1qalL\/5qF\/ngbgmxx3sC9+8Yu2ilSydoOwYFYvv\/yycjg33ngjcTBw\/\/33w+kYbjVBWHC\/0E\/8Y2NjGAEwzECbC8qCp2HY8fMoZVlZmRrdnD9\/vqVSwaxWAAFAEbeFoJ0LQ5v2IYOFfY2D5gAWQZWynw4s7GscJwcEAHHUS\/nabJ0r2xSAXiCYcnFFZw8W7uAFC7BwRwG3S4IAwG0+eUvnv9HlWgSIV\/7sQwYL+xoHzQEsgiplPx1Y2Nc4Tg4IAOKol\/K1\/s7FxdGvAVZWVmbeDki5mCWRPVi4gxkswMIdBdwuCQIAt\/mgdFAACkABKAAFrCiAAMCKrDAKBaAAFIACUMBtBRAAuM0HpYMCUAAKQAEoYEUBBABWZIVRKAAFoAAUgAJuK4AAwG0+KB0UgAJQAApAASsKIACwIiuMQgEoAAWgABRwWwEEAG7zQemgABSAAlAAClhRAAGAFVlhFApAASgABaCA2wogAHCbD0oHBaAAFIACUMCKAggArMgKo1AACkABKAAF3FYAAYDbfFA6KAAFoAAUgAJWFEAAYEVWGIUC0RSYmZmhtrY2GhkZmWPgwQcfJD7Dvpg\/vEf\/kSNHqKmpSdWxtraWNmzYoKqk693Y2EjZjq\/m3\/fu3Utbt26lioqKYpYBZYcCziiAAMAZFCgIFLjqCL3OUYIuXgfOp1OGDQBYAx1AbN++XYIkqAMUSF0BBACpI0ABoMBVBfSTcLYAgE96HB0dpbNnz9LGjRvprrvuoi1bttC5c+do5cqV1N\/fr56Ox8bGaNOmTcQnQq5Zs4YWLFignpz5yZtHEfgJm21NTk6qv\/UJklwKPdLANg4cOKAKdvz4cWpoaKCenh5i593b25v5bWhoSKUZHh5Wv\/OHnbv\/SZ7tnTlzRj3xZ6ujdwSA89N5sz1v3vv27aO1a9fS8uXL0WygABSIqQACgJgC4nIoYFKBbFMA2rk\/99xz9N3vflc5ev40NzdTe3u7cobaoXsdPV\/HzpgDgVwBQF1dXVbnzfZ37NihjpRevHhxJnjg7zkA4DKcP3+eurq66JFHHqG\/\/uu\/po6Ojsx3fX19c4bq+RrOi4OPXNMcbJsDCu8UgPc6\/p3ryR89dWBSe9iCAqWmAAKAUiOO+jqtQJARAH7Snpqayjz96wqxw9+2bRvt378\/MxqQLTDwjgDU1NTQzp0752jCowDsrLWj10P2\/FTPT\/F65MB7kXbUesTAu16B6\/TYY4\/R5s2bVbBSaARABwB+58+2eSSBRwiKfT2E040QhSsZBRAAlAxqVLQYFAgTAPDTt\/9Jmx2kdtw8HRAkAMjm0L12ggQA2jGzxvpJX+sdJQDI9aSPAKAYWjHKWCwKIAAoFlIoZ0koEDQA4HQ8p89rAXg4XK8PaGlpIV4kx0\/I3imAhx9+OLPwbt26dZmpAXbWeqi\/qqoqk6a6ujrrCIB\/CoDz27NnD\/G1vEr\/Zz\/72TVBib7GPwWQ6y0AHmXINcyPKYCS6AaoZEIKIABISGhkAwWCKBA0AOCncl4VH3QRoHexn3dxYL5FgNmmAPT0gZ426O7uzszHexcW+uvqfXLPNwXwxS9+UU1hjI+PZ0zoNRBcZ+9UQhA9kQYKQIHcCiAAQOuAAoIVyOeUTVY7iff48RqgSWKwBQWIEACgFUABwQokEQBMT0+r6YilS5dmXhXMJmmc+Xv\/OgLByFA1KJCYAggAEpMaGUEBKAAFoAAUcEcBBADusEBJoAAUgAJQAAokpgACgMSkRkZQAApAASgABdxRAAGAOyxQEigABaAAFIACiSmAACAxqZERFIACUAAKQAF3FEAA4A4LlAQKQAEoAAWgQGIKIABITGpkBD9S21kAAABDSURBVAWgABSAAlDAHQUQALjDAiWBAlAACkABKJCYAggAEpMaGUEBKAAFoAAUcEcBBADusEBJoAAUgAJQAAokpsD\/B4K8jNvdSGWQAAAAAElFTkSuQmCC","height":0,"width":0}}
%---
%[output:6e6ea8f6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:0a7f75d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:4db39541]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:08f83058]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:89ccd37d]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:2e22cd95]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:1cf68f47]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:6bacded8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:63025039]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:7ea2de59]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:9ff4664a]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:0b291691]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:375e0e67]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:89445da1]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:726a88d3]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714302e+02"}}
%---
%[output:88710ee3]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.994503273143434e+02"}}
%---
%[output:2ea7c278]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAgAAAAE0CAYAAABae4vcAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10HtV55x9SNlj5ILaAxCiKY2JkoE0wmJqqqoNDtIn7cWTSNLuSvHuadZXELTicXeNKct2wh4TYko5NNwXSKhzh42zqjzSBxjpNS6lL0xDhkBhQmpbEAvkDWZCAhQNpxLa03vOMufLVaD7\/78y8M\/P+33NyiPXOc+fe3\/O89\/nP\/ZpzTp8+fVr4IQESIAESIAESqCkC51AA1JS\/2VgSIAESIAEScAhQADAQSIAESIAESKAGCVAA1KDT2WQSIAESIAESoABgDJAACZAACZBADRKgAKhBp7PJJEACJEACJEABwBioCQJTU1PS1dXltHVoaEjq6+tTa\/fY2JisW7dOrrnmGunr65O6urrU7mUXnGUbozTIcPjkJz8p7e3tUUxyfU1\/f78MDg46dVy\/fr309PTEqu++fftk8+bNsm3btlzy0PYdPHgw9d9HLGi8OFUCFACp4mXheSGQZXL0EwDawa5atUqam5tTweLXxrTv69cYJKFoAvrGN74RK7kiNnEdoPdYu3btjFkZBUDZBFtcH9fi9RQAteh1tjlTAtPT09Lb2yvDw8Oye\/fuzASAjjxkcV8vmCaZtLW1RU7m5gk5TnJFbBDnGwEQp27u++R9BMDE6fHjxzkKgARJAW0oAArotGpX2R4K1brYQ5r20+9VV10ln\/nMZ5zqaiKwh8PN0+ro6Oic7+0ybrjhBvnYxz7mXNPQ0CA7d+6UpqYmTwR2onVf73461u\/NlEBra6vccccdsmzZMqfjsxNnWDk6lWDue+jQIad++jFTALfeeqt8+tOfdpK\/+bhZ6N8NU1sgmKRjX2+SiCnLTkh2G++66y4ZGBjwvO\/ExIRTv8nJyZk62T60OSrzu+++W+69914x7VP+btaGnZla8Up2xq\/2fU173e0yvm5sbJwRMW5++\/fvd4bUzceOD3d5YcLLHY9BZQXFYdBIgc1E62zq7hYV7t+XzdaUsXHjRjlw4IDo78f4zm6z\/s3cw\/ZtGBevOKx2f8P7p0eAAiA9tqUr2d3p2w00nZhXJ+\/uuLUcTb4m+bu\/90pQQclTv\/Orm+msL7jggllrAIwAsOugidYrYdsiwF1OUgLA6wnT3Rm7E4MfV\/27nwDo7u6WDRs2zGEflHD1u4suukief\/55R+B4JWW9p52o3HX3iwtz38cee8wzmd93330z8+52vNkJzi0A3GWZ7\/1EQFDMqs2xY8d8hYZdJ3fyd4s0k3zf+973yje\/+c1Z\/YNXEvf6fbkTuF7jVUf9u7lPWNk2l7yPUpSuU61ygygAquyAIt3edHD2E5DpPLUd9tOvPuWZjsXuYO3Oyn7ysROGJlnzhGrKMPd2P2kafl7f253ZBz7wAV8BYD8hxS0nTADoqId+wobig0YodFTi5MmTc5jYT63KaenSpbPaGGUKwD09Ydgbf+rTvtvvWhedD\/camVCWa9ascdprjxhEWRgZZTjffY3734aJESta\/7B7m9jzao\/5mwpFbbPfFIDN0cST26cPPvigIyS8nuj9ynWPAplRD7uMoHubEQIT\/2FckpjqKFKfVut1pQCo9QiI0X6\/pwPTgWrHt3z5cs8V8PY1R48e9Xyq06rYZehTp1mxH7aIL+zJxS\/B2h2i3j9uOUkJAPvemsz1Yyccv47ZToAf\/\/jHIwsArxETr\/tqPdxTHH5P2HqtJjJTD5ut1\/3cUyFBAsBv6sNtE\/Q07yUe3W0z00tuIWFEj1+iDotP2792GX5+dY8mGFZGAPhN\/dg7XOxYNr9Le\/rF\/PRtLl7TTjG6CF5aMAIUAAVzWDWrm7UAsLfRhXWwcRO3cvTaFhi1HDu5uZOFlm1vA4wyAqDX2Avn9N+65cw9AuJOQHEFgJuje5TALTySEgAmbv2Eh+6M8BIA7qmEsBGAIggArxEn41d3\/PmNANhl+P02KACq2VsW494UAMXwUy5qiU4BuIeqzZyq39OU15BtmAAIGrq3n0oVpD4l+QmAqOXo0Ko7OZupEbcA0CQbZXGVOznaT8juaRRNmGFTADo6EZZA3WWgUwB2gPo9VbuD2J3M3U\/Dps32SJBpj4kdt43XFEDYjyftKQAjFs3IiZ8A8Bo5MYzcIwB+izbd0w9BUwBeXDgFEBYt5fqeAqBc\/ky1NWkvAgxKoGECIKhuXvPjfgIgrBwdLjXz+W7YUQSA2njtAjBluVdy2wfoxFkEaIaCbRu974c\/\/GFndMLro5y82hd1EaCWaUSRW3j4LZCzbexr9J6f+9zn5Pbbb5+zYFFt3AJA\/+a3oNC0NUxweg2Ph43A2Bz92hiUvO2Ee\/PNN\/vGVlAZWgevxYFRFwHaXMJGwFLtYFh45gQoADJHXvwbRt0GaG\/hC9sG6LWwMM4UgFINGl4OW2RnnwwYVI7ex71l7FOf+pQ88cQTM4vevEYA7OTgt5DRLtu9NsFLINiJ0LbV\/28EgNd977nnnlkn2unhRPZ6A2QboJ3I7YTktUXUb\/uhm6u9JkHLVG7bt2+XTZs2OTjskRyzm8NvW2HY\/v2gbYB6r6hPxn5z9zoK5JVc\/UY9lJHXFkyvUQQ\/8ah\/d5886LeWwpQRZaSq+D0YW2AIUAAwFhIlELbiOtGbsbDECXhNNbh3evidw2BXBjkIKPHG1EiBtmAzQsdrZ0AYDh4EFEaofN8XWgBoJ6P7mvXAE69OKeiwmfK5Mh8togDIhx\/QWgRNgQRNXXjdDzkKGK13rdt5TQEok7DDs7xEW1ne3VDrMRGl\/YUVAGELjcz3LS0tzos3zL\/1BxH3JR5RQPKaMwQoAIofCW7hrC2Km\/ztWGBCySYm3FNzcZK\/1pCCLRs\/5ekuhRUAOnemAasfvxEAN2hVySMjI5m+oS1PzmZdSIAESIAESMAQKKQA0CeU2267TTo7Ox0RQAHAgCYBEiABEiCBeAQKKQD0SV4\/erpV0BoAG4UZ1uzo6Mjlu7jjuY1XkwAJkAAJkEBlBAonAHSOedeuXbJlyxbRl8ZEEQBm\/l9R2W+kc6N717veVRlNWpMACZAACZSegL6J8ZJLLil8OwsnAHTIX\/ct60loYbsA1DtRk79eqwJgfHy88E6tRgPIDqdOdmSHE8AtGXdkVygB4LU62bjQ61WfcVf+8wfBHwROALdk3JEdTgC3ZNyRXaEEgNtdYSMAOlqgJ2oFDfvbZfIHwR8ETgC3ZNyRHU4At2TckV2pBIB54tfdAebd6OaIUOPqoGNY+YPAfxBHjhwpxZwYTgC3JDuywwnglow7nF1ZckWhBQDuPm\/Lsjg1aS5RymNnEoWS9zVkR3Y4AdyScYezK0uuoACwYqAsTsXDGrdkZ0J2OAHcknFHdjgB3HLxu6+Vo99\/FC8gJ5YUABQAiYQiO2IcI9mRHU4At2TcYez2fOc5uWnPkzJ1x\/VYATmyogCgAEgkHNmZ4BjJjuxwArgl4w5jRwGAccu9FacAcBexMyE7nABuybgjO5wAZkkBgHHLvRUFAO4idsRkhxPALRl3ZIcTwCwpADBuubeiAMBdxI6Y7HACuCXjjuxwAphl\/wNHpf+BI1wDgOHLrxUFAO4bdsRkhxPALRl3ZIcTwCwpADBuubeiAMBdxI6Y7HACuCXjjuxwApglBQDGLfdWFAC4i9gRkx1OALdk3JEdTgCzpADAuOXeigIAdxE7YrLDCeCWjDuywwlglhQAGLfcW1EA4C5iR0x2OAHcknFHdjgBzFIPAdKdADwICOOXWysKANw17IjJDieAWzLuyA4ngFlSAGDccm9FAYC7iB0x2eEEcEvGHdnhBDBLCgCMW+6tKABwF7EjJjucAG7JuCM7nABmSQGAccu9FQUA7iJ2xGSHE8AtGXdkhxPALNfc\/bg8\/PQprgHA8OXXigIA9w07YrLDCeCWjDuywwlglhQAGLeqWo2NjUl3d7cMDAxIU1OTZ10oAHAXsSMmO5wAbsm4IzucAGZJAYBxq5rV9PS09Pb2yqFDh2Tnzp0UACl4gh0xDpXsyA4ngFsy7jB2V93+iByfeoVTABi+7K0OHjwo\/f39zo05ApAOf3YmOFeyIzucAG7JuMPY1W98yDHkOQAYv0ytpqam5LbbbpPOzk5HBFAApIOfnQnOlezIDieAWzLu4rPTJ38dAaAAiM+uKhb79u1z7rt8+XKuAUjRA+xMcLhkR3Y4AdyScRefHQVAfGZVs9CFf7t27ZItW7bIxMREJAFgKnvgwIGq1buIN1a+jY2NRax61etMdrgLyI7scALxLFtbW+XVCy+Tn67s5ghAPHTVuVqH\/FetWiXNzc3CXQDp+oBPEzhfsiM7nABuybiLz07fAaAHAXEKID67TC107r+rq0tGR0fn3Hf37t2OKHB\/uA0QdxE7E7LDCeCWjDuywwnEtzRbACkA4rOrqgVHANLFz44Y50t2ZIcTwC0Zd\/HZGQHwup+9IC\/86X+JX0DOLM45ffr06ZzVKZXqUACkgnWmUHYmOF+yIzucAG7JuIvHzl4AeO4LP5Qff\/F34xWQw6trRgBEYc8pgCiUvK9hZ0J2OAHcknFHdjiBeJb2\/P+8H+yXya\/\/UbwCcng1BYDlFAoAPELZEZMdTgC3ZNyRHU4gnqU9\/\/+mhwfk+KN\/Fa+AHF5NAUABkEhYsiPGMZId2eEEcEvGXXR29vD\/yiXz5fs7fkvGx8ejF5DTKykAKAASCU12JjhGsiM7nABuybiLzs4e\/t9\/49Xy2x9cTgEQHV8xruQUAO4ndiZkhxPALRl3ZIcTiGZpP\/0vqp8nT\/zhL0tZcgVHADgCEO1XEHIVO2IcI9mRHU4At2TcRWO35S+ekj\/5h2eci+\/uvEI6VyykAIiGrlhXlUXVVYM6OxOcOtmRHU4At2TchbOzh\/7N079alSVXcASAIwDhv4IIV7AziQDJ5xKyIzucAG7JuAtmZw\/965U69K8igAIAj7lcW5ZF1VUDMjsTnDrZkR1OALdk3Pmz0+S\/5vOPi\/5XP0e3vlfOn3fujEFZcgVHADgCgPcgliU7Exwj2ZEdTgC3ZNx5s3Mnf53z17l\/+0MBgMddbi3L4tRqAGZnglMnO7LDCeCWjDtvdlfd\/sjMk789708BgMdaISwpAHA3sTMhO5wAbsm4IzucwFlLfeo3T\/7mr37JX78vS67gFACnAJL4\/Qg7Yhwj2ZEdTgC3ZNydYaeJf8OeJ+Xhp0\/NwPzS77xHfv3dF\/rCpQDA4y63lmVxajUAszPBqZMd2eEEcEvGnciXvv2s3LzvBzMQ9am\/Z\/Ulzl7\/oE9ZcgVHADgCgPcgliU7Exwj2ZEdTgC3rOW4e\/ipU84qf\/ujZ\/zvv+nqSEApACJhKtZFZXFqNajXcmdSKW+ywwmSHdnFIfD9yZ\/Kddu\/M8tEn\/rv6rhCVl46P3JRZckVHAHgCEDkoA+6kB0xjpHsyA4ngFvWStzpHH\/\/A0dET\/WzP0jiN\/YUAHjc5dayLE6tBuBa6UzSYEt2OFWyIzs\/AjrMv2HvkzNb+sx1lSR+CoCY8TY1NSVdXV0yOjoay3LZsmVy\/\/33z7GZnp6W3t5eGR4edr5bv3699PT0+Ja9b98+2bx5s\/N9W1ub9PX1SV1dnef1FACxXDTrYnbEZIcTwC0Zd2RnE9CkP\/DAkVkr+pNM\/BQAMePNCABN0s3NzZGsDx48KP39\/Z4CQP+uHy3PlN3R0SHt7e1zyjblDA0NOUlfhUNDQ4OvYKAAiOQez4vYEZMdTgC3ZNzVNjtzVK97G5+d9D++slHarrxo5hx\/nNhZy7LkitTXACQtANzOswWB+zt9+h8ZGZl56nf\/2319WZyaRIDHLYMdcVxiZ68nO7LDCeCWRYw7Tfg6hP\/Fg5PylUM\/8n3S\/5Ul853tfOblPTglb8uy5IrUBUDS4O3ywsSF1whAS0uL52iBllsWp6bJ3K\/sInYm1eDkdU+ywz1BdrXBLmhoXwlootek37ni4lir+VF6ZckVqQsAew1A2Hx9HGfok\/\/g4GDovP7Y2JisW7dOJicnZffu3YHTEOpU8zlw4ECc6tT8tRMTE9LY2FjzHBAAZIdQO2NDduVjN\/nSq06jvvDoKRl+8qe+DWw4\/1y5trFOula8RfT\/p\/1pbW2ddYvx8fG0b5l6+akLANMCk7D13zoPv3PnTmlqaqq4gVquJnevxX065L93717RNQD19fXOugL9+C0aLIuqqxgqUACfxABor5mQHdnhBHDLvMRd2Dy+aaE+5S9aME\/ueu3NfGkN70chWpZckZkAMFDduwIqHRXQJ\/zu7m4ZGBiYJSjMbgF7yN\/vWlO3sjg1SgAnfU1eOpOk25VFeWSHUya74rHT4fy\/\/cFJeezYS55z+HaLNMnv7rpS3nTez6U2n48QLEuuyFwA2LDt4Xl0VMCe59enfPOhAEDCGrdhR0x2OAHcknGXb3aa7P\/j9GnZ\/jdHIyV7fcL\/P+2Xy7mvOydXCd9NmQIAjztPS79E7r7YHsY3Sd5va5\/XFIDfdIHepyxOTdg1kYpjRxwJk+dFZEd2OAHcMum406H8hw5PyVd9Vue7a6pP9++9dIH8\/gcXO19Vc0g\/LsWy5IrcjACoA8IW6ek17oOA7MN9zHednZ0zi\/3stQc8CChumEe\/PunOJPqdi38l2eE+JLvqsNMn+\/uf+JGM\/ehnoU\/2Jrnr0\/1N1y+SN77+5zJZqY+TCbekAAhn5HlFUAIHi0zMrCxOTQxIjILYEceA5bqU7MgOJ4BbRok7far\/+vdfkK\/\/4\/OREr2d7P9Hy9vlrW9+vfNkX6Sn+yhEy5IrMhsBsJ\/Eoz7tR3FEkteUxalJMolaVpTOJGpZtXYd2eEeJ7vK2ZlV+N+beFm+8M0JOf7iK3POz\/e7i1mZf2fnFXJOwYbxcXLlmS5OXQDYq\/7DhuArcUgSthQAOEV2xGSHE8AtGXfR2ZlEf+AHU3L\/4z+S8ed\/KmbPfVgp5gn+w1e\/Vd5\/2QWFH8IPa2\/Y92XJFakLgKeeekpuvvlmufXWWxN5F0CYYyr5vixOrYQBasuOGCUnQnZkhxOYa6mJXv\/354eekyMvTEceujfD9\/rflUvmS8eKi53CyziEXynvsuSK1AVA2HG9Xo4IehlQpY4Lsi+LU9Nk5Fc2kxhOnezILi4B8zQ\/\/L3n5YF\/eiHWsL1J6q+++qq8\/4qL5L9es5CJPqYDypIrMhMASb0OOKafYl1eFqfGanRCFzOJ4SDJjuz8COhq+\/EXfua8+CbO3Lwpz8zRX3Hxm+Sm971jJtHr\/2Hc4XFXllyRugDAEWdvWRanZk+OnUklzNkR4\/SKzs48yT\/45En52hM\/hpO8EtSX4bT\/4kJZfEHdrETPUTs8vvwsy5IrKAAsD5fFqcmHe3iJRe+Iw1uY3hVkh7MtAjt9itcn8Tv+9piMP\/+zipP8b7znIjl\/3rkVz80XgR0eGelaliVXUABQACTyS2FngmMku2KzMwn+4JGfyJcOTjqNefjpU7EbZVba65P87616h5z62avOansdJUhjHz3jLraLZgwoAHB2ubUsi1OrAZidCU6d7PLPTpP8G17\/Ohn61gl5ZuoVKMFrK82cvP63fcXF8s76eZGG63FC\/paMO5xqWXIFRwA4AoD\/CixLdiY4RrKrPjtN8C\/+7N\/kr\/\/phYoTvJPoF8yTX333hXLl2988k+DTeIrHyXHdTiXsKAAqoZdT27I4tRp4mcRw6mSXLjv7pLu\/qjDB20\/xv9DwJrmy8c3O4jvzd7wl2Vsy7nDmZckVVRkB0Lf0bd682aGvLwA6duyYjIyMSF9fn9TVnVnBWo1PWZxaDXbsTHDqZFcZuxP\/vsAZWr\/3WyfkseMvOYUhc\/CmFmaYXlfTf+Sat82sqi9ikg8iy7jD464suSJzAaDvBNBX8nZ3d8uGDRukp6dHli1bJr29veL3Wl\/cTfEsy+LUeK1O5mp2JjhHspvLzix8M0\/vJ069Il\/69rPO8DyyH96+g0nwl771DfLhq9826+k9b8P0eFSFWzLuwhn5XVGWXJGpALBPBVy6dKl0dXU5AqC5uVnM6X9DQ0NSX1+Pe6YCy7I4tQIEsCk7ExhdTR\/IYlbQ\/\/FDx+Xwc\/+SSHJ3ntQXzJPrL6uX31p+NsGntZoe93x1LfmbxfmXJVdQAFgxUBan4mGNW7IzITs3AZPc\/+zRZ+WR17bFVTI0b4bgNbm\/o36erP6FC+VfX3pBFi68uOI98bj3imvJ3yzuu7LkikwFgOLW+X+d77enAMxoQEdHh7S3t+NeqdCyLE6tEANkzs4EwuYYFY2dGZbX\/37vxMvy199\/wWlHEsndPL1ffvEbZcP7Fjl74IP2wheNHR4lyVuSHc60LLkicwGgyHW4f+3atbPob9u2LVLyn56edtYLDA8PO\/br1693phH8Pva9dK1B0BRDWZyKhzVuyc6kPOxMgv+\/356Ub4\/\/JPHkftnCN8qaZRfJO+vPLvhF594Zd+WJO7wl2VuWJVdURQBU4i5dRKgfTfpmTYHfyMHY2JisW7dOtm\/f7qwzMKMPfrsNyuLUSviituyIUXLZjQCY18Q2zD9P7nrouDz1Y+xYWndLTfLWoflLLqyT5e88X5Zc+AbnMn16T\/PDuMPpkh3Oriy5onACwO0yWxC4v9OEf\/To0cARAtumLE7Fwxq3ZGdSXXZmvv1bT5+S7x77iYz9KPnkrvve11\/XKBMv\/r+Zo2nRJ3ec1mxLxh1OkuxwdmXJFZkKAPPEHvZq4LBhfeM2e1eBPuHbHzNV0NLSEmlqQW3L4lQ8rHFLdibpsTPJ\/QvfnJDvTbzs3KjS+XYtw35yf8\/b3yy\/9u4LnbJn\/v7aMbV4y9K3ZNzhjMkOZ1eWXJGpAFDc+lS+d+\/eWXPx9lD+mjVrIp0JoE\/+g4OD0tbW5nmAkBEAq1evlnvuuUdUdHANAB7wYZbsTMIIzf3eJPZ9Dx+WyVfOk6fBN8V53dkk8ZWXLpCWd71FFtXXFSqxR6XJuItKau51ZIezowAA2AU9sdvnABw+fFg0wd9\/\/\/2hdzEHC7nn9Y0AOH78+IzY8LvW3ESdaj4HDhwIvTcvOEtgYmJCGhsbax7J5EuvOgyefflVOf+818m+770sx0\/9m\/Nv810lkBrOP9cxv\/jN50rjW86V9y95o9T9p3Nm\/ma+r+QeRbJl3OHeIrt47FpbW2cZjI+Pxysgh1dnOgKQhgDQhX66pXBgYECamppmEHtNAfhdawuAMji1GnFW9qcJe+tb44Lz5M6Hjjvz7PpJYjheyzFP7e+9dIH84uIzC+mKNBzPuKsGAfyeZf\/N4mTCLTkCEM7I84qwKQA9B8Cs1v\/c5z4XepegEwT1iX\/x4sUzawBUAGzdulV27NjhedpgWZwaCi2FC4rcmZiheE3yf\/fDKfnu0WS2vhnM9ly7boFru\/Iied05Z57a9bt\/\/8mzcskll6TglfIXWeS4q7Z3yA73QFlyRaYjAAa31zkA+lIgs1XvzjvvlJ07d856oje29qp\/85Tv9w4BtzgI2jGg5ZfFqXhY45Z57EzsxG4fWFPpWfJeif3nG94k\/+3ai+UtdWeG6O0n+jCqeWQXVue8fE92uCfIDmdXllxRFQGAYxdxHwRkLwI033V2djpiQj+22PBbMGjqUxanVsIXtc2qM7GH4vU4WF0Z\/4+vrYxPI7G\/84I6ua5pgVz8lvNmJfUkt79lxQ71bZ7tyA73Dtnh7MqSKwonAHCXhVuWxanhLU3+iko7E\/cc+91\/\/4z88Ll\/cSqa9By7OUte59pbrHe5V+tlMZWyS96bxSmR7HBfkR3Oriy5InMBYE7n01cCuz9h2\/Rwd0WzLItTo7U22auCOhN7KP7AD07KoWOVv7Pdrr09x66jArr17R0L5qX2xJ4suexOAky63nkoj0kM9wLZ4ezKkisyFQD2ynyz31+H692vBsbdUpllWZxaGYXo1vZT+z+NT8j3Tv5cIu9rNzVwL567YdlbZ1bF6zVJDsNHb3XyV7IjxpmSHdnhBHDLsuSKTAWAexugvUpf5+r37NnjeagP7qZ4lmVxarxW+19tEvzRk9Py5UM\/kuMnpyt+X7uduHUoXufY33dZvbztza8vzBN7UnxNOUxiOFGyIzucAG5ZllxRVQFgn9UftJ0Pd1M8y7I4NU6rdXheh823\/81ROXZyuuL5dn0qN3PsKxa\/Rd5\/Wf1MYq\/WHHscHtW4lkkMp052ZIcTwC3LkisyFQCK296KZyf9Bx98UEZGRjgCgMekp6VJuproX3\/uOXL7X47DT\/EmuetT+2+850J503lntrzp35955hn5lWVnD2JKuBmlLo5JDHcv2ZEdTgC3pAAA2blP6DNn+utefr+9\/+CtYpuVwalm2P7gkVPypYPPxnqiNwn+8ovfKGuuPDvfHmWunR1x7HCbMSA7ssMJ4JaMO5xdGXKFtj7zEQAcefqWRXSqecLf\/eizsvc7z4UmfJPM37e0Xj6y\/G0zT\/BRknyQB9iZ4PFJdmSHE8AtGXc4uyLmCq\/WZioAor4LoL7+zLxx1p8iOVWH9Pd+9znRxO\/3MU\/0N7e+U5a+9Q0zyT4NruxMcKpkR3Y4AdyScYezK1KuCGolBYBFpwhO3f3oc7Jh75OePtWEr4fbtP\/iQmdevtKn+jg\/D3YmcWjNvpbsyA4ngFsy7nB2RcgVUVqXiQDQ1f6bN28Orc\/69eulp6cn9Lq0LsizU9fc\/bjn8L4m+f03Xp3q030U3uxMolDyvobsyA4ngFsy7nB2ec4VcVqViQAwFQqaAohT6bSuzZtTdX5\/w54n5yR+Tfp3dVyR+VN+EHd2JnhUkh3Z4QRwS8Ydzi5vuQJtSaYCAK1kVnZ5caom\/vse\/7F8+i+fntV087Sf5dB+VPbsTKKSmnsd2ZEdTgC3ZNzh7PKSK\/AWnLGkALAI5sGpurhP5\/jNdj6tXp4Tv8HHzgT\/KZId2eEEcEvGHc4uD7kCr\/1Zy9QFgBn2Hx0dDa1vLb8MSBP+kRem5Tf\/9IkZTmaof+Wl80PZVfsCdiY9kByEAAAgAElEQVS4B8iO7HACuCXjDmdHAYCzy61ltZyqyX\/N5x+f9dT\/lfXLZo7RzS0wq2LsTHAvkR3Z4QRwS8Ydzq5auQKvsbdl6iMASVc4zfKq4VR38i\/SU7\/tC3YmeGSSHdnhBHBLxh3Orhq5Aq+tv2VVBIDXtsBt27ZJe3t7Gm2MXGbWTvVK\/rqlL4+L\/MIgsjMJI+T\/PdmRHU4At2Tc4eyyzhV4TYMtMxcAmvz37t0rQ0NDYk78M+sEOjo6QkWAeZfA8PCw07KoZweMjY1Jd3e3DAwMSFOT90trsnSqO\/mvXDJf9t90Zj9\/ET\/sTHCvkR3Z4QRwS8Ydzi7LXIHXMtwyUwGQxFHA9tsEowoHIxoOHToU+MKhrJzqTv6dKxbK3Z1XhHsrx1ewM8GdQ3ZkhxPALRl3OLuscgVew2iWhRMA7mbZgsCvyea1w\/p9HkYA7FP9\/vsvXSx\/3H55NG\/l+Cp2JrhzyI7scAK4JeMOZ0cBALKrdArAvm2UkwX1mttuu006OztFxUK1BcBdf\/+M3Lr\/KacZOtf\/xB\/+MkgyX2bsTHB\/kB3Z4QRwS8Ydzo4CAGcnSSwC1GQ+ODgobW1t0tfXJ3V1dZ410nvpZ\/ny5ZHWAJhCDhw4UEELvU0PnXhFPnHfc86XDeefK4O\/udD5bxk+ExMT0tjYWIamZN4GssORkx3Z4QTiWba2ts4yGB8fj1dADq\/OdAogjfarEJicnPQUAbrwb9euXbJlyxbRjqKaiwB13v+q2x+ZefIv6mp\/Px\/yaQKPbrIjO5wAbsm4w9lxBABgF3XRXpyig1b3qzhYtWqVNDc3S7V3Adjz\/rrgTxf+lenDzgT3JtmRHU4At2Tc4ewoAEB27uH\/3bt3Owka\/ZgFfva2Qi0r6Ahiv3um5VQ9319P+tNPmeb9bZ+xM0EjWITsyA4ngFsy7nB2aeUKvEaYZVWnAMw8vla9oaEhcIueaZ696t9s71Pbnp6eQALVGgGwt\/wV4aU+WBgxiaHc1I4dMU6P7MgOJ4BbUgDg7DwtNbHr07z7Sd59sfsgIHsRoPlOV\/y7RxWqJQD+YexF+dCfnHnBz+03XCo3rnpHwuTyURw7YtwPZEd2OAHcknGHs6MAwNnNWNojANV+E6BWKmmnuhf+lWXLn5fr2ZngPwiyIzucAG7JuMPZJZ0r8JpUZpn5FAAy7F9ZE6NbJ+3Uz359XHb87TGnArrqvwiv9Y1Oa\/aV7ExQcpwCwMmRHdlVQgC3TTpX4DWpzDJTARDl4J7KmlOZddJOrd\/4kFOhsi78s2lTAOCxR3ZkhxPALRl3OLukcwVek8osMxUAlVU1fesknXrnQ8flfw8\/7VS6jNv+3N5gZ4LHJ9mRHU4At2Tc4eySzBV4LSq3pACwGCbl1Fqa+zf42JngP0ayIzucAG7JuMPZJZUr8BokY0kBkIIA2POd5+SmPU86JZd97p8CoPIfIjtinCHZkR1OALekAMDZ5dYyCae69\/2XeeW\/7Uh2xHhYkx3Z4QRwS8Ydzi6JXIHfPTnLTEcAghYB+p3ol1xTw0tKwqn2qX+\/\/8HFsvlXLwm\/cQmuYGeCO5HsyA4ngFsy7nB2SeQK\/O7JWVIAJDwFYJ\/5r0\/\/ugOgFj7sTHAvkx3Z4QRwS8Ydzo4CIAY7r9f\/epmvX78+9EjfGLeNfWmlTrUX\/61cMl\/233R17DoU1YCdCe45siM7nABuybjD2VWaK\/A7J2uZmxGAZJuFlVapU2tx8Z8hzc4Eizm1Ijuywwnglow7nF2luQK\/c7KWmQqAZKuefGmVOtUM\/9fCwT9u+uxM8HgkO7LDCeCWjDucXaW5Ar9zspYUABbPSpxqD\/\/\/7nWNsvVDTcl6KuelsTPBHUR2ZIcTwC0Zdzi7SnIFftfkLVMXAPbK\/6VLl0pXV5eMjo56tqTaLwSqxKn3fuuEbPrqYaddtbT4j1MAlf8o2RHjDMmO7HACuGUluQK\/a\/KWqQuA5KucXomVOLWWh\/\/VI+yI8bgkO7LDCeCWjDucXSW5Ar9r8pYUAAlMAdh7\/zdcv0g+3bYkeU\/lvER2JriDyI7scAK4JeMOZ0cBALAz0wFlmwKo5dX\/nAIAfgguE3bEOEOyIzucAG5JAYCzm2OpwuCWW26RP\/iDP5CmpuDFc9PT09Lb2yvDw8NOOUFnB7gFR1tbm\/T19UldXZ1n7VGn1vrwP6cAKvsxMInh\/MiO7HACuCWaK\/A7pmOZmykAPQp4z549gQlaEfT39zskenp6xCT4jo4OaW9vn0XICIWWlhbnO\/PvhoYG38OGUKfWb3zIuffqn79A9nzsynQ8lfNS2RHjDiI7ssMJ4JaMO5wdmivwO6ZjmSsBoMl9aGhI6uvrI7fWFgRhRnoi4cjIiK\/IQJza\/8BR6X\/giHPrWlz9b5izMwmLPv\/vyY7scAK4JeMOZ4fkCvxu6VnmRgBoIp+cnAwdAbBRBL1cyAtZGgKAw\/9nSLMzwX+kZEd2OAHcknGHs6MAANgFLQLUofmdO3eGrgEwt1XBMDg4KGHz+ub6oOkCc01cp9by2f9u97MzAX4Qr5mQHdnhBHBLxh3OLm6uwO+UrmXmIwB+c\/Nmrj5uc6OMHJh7atlhiwDN\/Q8cOBBalUMnXpFP3Pecc936a+fLJ35pfqhNWS+YmJiQxsbGsjYv1XaRHY6X7MgOJxDPsrW1dZbB+Ph4vAJyeHXmAsArYUd5OvdjNzY2Jt3d3TIwMOA5ehA1+Wv5cVUdt\/+d9QqfJvBfN9mRHU4At2Tc4ezi5gr8TulaZioAgubso+4CcONQO7\/Fg1FW\/tvlxXUq5\/8pAJL4ebIjximSHdnhBHDLuLkCv1O6lrkSAFF2Adir\/sMSfJTpAVQAcP5\/dmCyI8Z\/qGRHdjgB3JJxh7OjAADYuef\/7SLCVuiba90HAdmLAM13nZ2d4vfioaAXDsVxqn38792dV0jnioUAkfKYsDPBfUl2ZIcTwC0Zdzi7OLkCv0v6lpmOAGhzdMh+06ZNs1b86zz+unXrZPv27dLc3Jx+q33uEMepux99Vjbs\/YFT0v4br5aVl9buAkBlwM4ED1uyIzucAG7JuMPZxckV+F3St8xcABgRsHbt2lmt2717d1WTv1YmjlM5\/z87ONmZ4D9WsiM7nABuybjD2cXJFfhd0resigBIv1nYHaI6lfP\/c\/myM8FijqMnODeyI7vKCODWUXMFfodsLDMVAPYcfTWH+v3QRnWqPf\/fs\/oS6Vm9OBtv5fguFAC4c8iO7HACuCXjDmcXNVfgd8jGMlMBEPfo3mwQnL1LVKdy\/z9HAJKMTXbEOE2yIzucAG4ZNVfgd8jGMlMBoE2Kuto\/m+bPvktUp3L+nwIgyfhkEsNpkh3Z4QRwy6i5Ar9DNpaZCoCgdwFoc4O26GWBI6pTr7r9EdF1AIvq5zlvAOSHuwAqiQEmMZwe2ZEdTgC3jJor8DtkY5mpAMimSfhdojjVXgCoe\/\/1DAB+KAAqiQEmMZwe2ZEdTgC3jJIr8NKzs6QAsFhHcaq9AJD7\/8\/CY0eM\/2jJjuxwArgl4w5nFyVX4KVnZ5m6ALAX\/vmdzmeaW4QpgP4Hjkr\/A0ecKlMAUAAk8VNlR4xTJDuywwnglhQAOLvcWkZxKhcAeruPHTEe1mRHdjgB3JJxh7OLkivw0rOzTH0EwN0U9\/sAgt4PkB2GM3eK4tT6jQ85165cMl\/233R11lXM7f3YmeCuITuywwnglow7nF2UXIGXnp1l5gLA6w19Zpqgo6ND2tvbs2u9605hTrUXAN7VcbmsvfbiqtU1bzdmZ4J7hOzIDieAWzLucHZhuQIvOVvLTAVA0EFA+pKgPXv2SF9fn9TV1WVL4bW7hTmVbwD0dws7EzxkyY7scAK4JeMOZxeWK\/CSs7XMlQDQ0YGhoSGpr6\/PlkJEAWAvANT9\/3oOAD9nCLAzwSOB7MgOJ4BbMu5wdhQAALug+f48nBAY5lQuAOQIABD2oSbsiEMR+V5AdmSHE8Atw3IFXnK2lpmOAGjTdKh\/06ZNsnPnTmlqanJaOzY2JuvWrZPt27dX9ZXAYU7lCYAUAGn8PJnEcKpkR3Y4AdwyLFfgJWdrmbkAMCJg7dq1s1q6e\/fuqiZ\/rUyQU\/kK4ODAZEeM\/3DJjuxwArgl4w5nRwGAs8vM0kw5DA8PO\/dcv3699PT0+N4\/yKlcAEgBkFbgsiPGyZId2eEEcEsKAJxdZpa6qFA\/mvSjbDUMcipPAKQASCtwmcRwsmRHdjgB3JICAGdXNUtbEHhVIsip937rhGz66mHHjDsA5tJjR4yHNdmRHU4At2Tc4ewoAHB2VbEMOoPAVCjIqdwBwBGAtAKXHTFOluzIDieAW1IA4Owyt9Qn\/8HBQWlraws8aEidaj4HDhyYVc+2XRMy+dKrcs3b58kXPrww8zbk\/YYTExPS2NiY92rmsn5kh7uF7MgOJxDPsrW1dZbB+Ph4vAJyeHVVdgFUi4PXMcR2XfxUnb0D4D9ffoF8+RNXVqsJub0vn8Rw15Ad2eEEcEvGHc6OIwAgO7Pnf3Jyck4Jab8OWO\/d3d0tAwMDM2cQRBEA9g6AntWXSM\/qxWDry2vGzgT3LdmRHU4At2Tc4ewoAAB2ZlteQ0ND4HY8oOhIJnoIUdBxw35OtQXA\/huvlpWXzo90v1q6iJ0J7m2yIzucAG7JuMPZUQAA7KIsxAOK9TWxV\/1HER9+TuU7AMK9ws4knJHfFWRHdjgB3JJxh7OjAADYmSTc2dmZyal\/7oOAoiwC9FrYwR0A4c5mZxLOiAIAZ0R2ZJc8AbxECgCQXdgwPFhsImZ+TuU7AMLxUgCEM2ISwxmRHdklTwAvkQIAYGemAEZHRz2t014EGFZlL6fyHQBh1M58TwEQjZPXVWRHdjgB3JJxh7OjAMDZ5dbSz6n1Gx9y6swdAP6uY2eChzXZkR1OALdk3OHsKABwdrm19HIqXwIUzV3sTKJx4ggAzonsyC5ZAnhpFAA4O9m3b59s3rzZKUFfA3zs2DEZGRkJPKWvgttFNvVyKl8CFA0fBUA0TkxiOCeyI7tkCeClUQCA7MxpfHogz4YNG5zzAHTuv7e3V6p1PoBpipdTN33lsNw7csK5hC8B8nc6BQD4g+D6CRwc2ZFdRQRwYwoAgJ19DsDSpUulq6vLEQDNzc2Sh90BXk7lFsBojqYAiMaJT7E4J7Iju2QJ4KVRAADsiigAzBbAlUvmy\/6brgZaXRsmFAC4n8mO7HACuCXjDmdHAQCy0\/l\/ne+3pwDMaEBHR4e0t7eDJVdu5nYqtwBGZ8rOJDor95VkR3Y4AdyScYezowDA2TnD\/WvXrp1VwrZt26qa\/LUyQQKAWwCDHc7OBP9BkB3Z4QRwS8Ydzo4CAGeXW0u3U7kFMLqr2JlEZ8URAJwV2ZFdcgTwkigAcHa5tXQ7dc93npOb9jzp1Jc7ADgCkFbgUjzhZMmO7HACuCUFAM5u1jkAphg9D0B3A1Tz43Yq3wIY3RvsiKOz4lMszorsyC45AnhJFAAgO10EuHfvXhkaGpL6+nqnFLM7IG+LALkFMLqTKQCis2ISw1mRHdklRwAviQIAYGdvA3Q\/7efxHAAKgOhOpgCIzopJDGdFdmSXHAG8JAoAgF3RBIB5CRDPAAh3NgVAOCO\/K8iO7HACuCXjDmdHAQCy0yf9TZs2yc6dO6WpqSm3UwD2GQCdKxbK3Z1XgC2uDTN2JrifyY7scAK4JeMOZ0cBALAzIwCjo6Oh1vp+gPvvv3\/Ode4y2traAl8iZL94KOxa26n2FsD9N14tKy+dH1rnWr6AnQnufbIjO5wAbsm4w9lRAODsYMvp6WnnpUEtLS3OoUHm334vEbLXFdTV1YW+cMhPAOjTv44C8ONPgJ0JHh1kR3Y4AdyScYezowDA2SVqaY4W7uvrE03y9sf9XdC1amc7la8BjucmdibxeNlXkx3Z4QRwS8Ydzo4CAGfneQ4AehRwUFL3GgEwowde1bed+sWDk\/I\/v\/xD5zIeAhTubHYm4Yz8riA7ssMJ4JaMO5wdBQDILslzAKKcHzA2Nibr1q2TyclJCTtsyHaq2QKozZy643qwtbVjxs4E9zXZkR1OALdk3OHsKAAAdkluAzTz\/1oNr+F\/\/btbbPT39zu17unp8ay9OtV8ltz0Z3LoxCvScP65MvzRRqC1tWUyMTEhjY3khHid7BBqZ2zIjuxwAvEsW1tbZxmMj4\/HKyCHV59z+vTp01nVKykBECX5uxcMaht1NEBfQzwwMDCzBdFuu63qrrr9EdGtgDwDIFp08GkiGievq8iO7HACuCXjDmfHEQCQXaVTAGEr\/021KhEA9hkAq3\/+AtnzsSvB1taOGTsT3NdkR3Y4AdyScYezowDA2VW0CFCH8XU+32\/Y366W1xRAkK1xqi0AelZfIj2rF1fQ2towZWeC+5nsyA4ngFsy7nB2FAA4O9jS7yAhPTRIXy5k9vp3dnbOvFlQBcPg4KBzz6gHAdmHAPEMgGjuYmcSjROnAHBOZEd2yRLAS6MAwNnl1tI4lacAxncRBUB8ZsaC7MgOJ4BbMu5wdhQAOLvcWhqn8hCg+C5iZxKfGQUAzozsyK5yAngJFAA4u9xaGqfetOdJ2fOd55x68hCgaO6iAIjGicPYOCeyI7tkCeClUQDg7HJraZxqDgFaVD\/PEQD8hBOgAAhn5HcF2ZEdTgC3ZNzh7CgAcHa5taQAwF3DzoTscAK4JeOO7HACuCUFAM4ut5bGqfUbH3LqyEOAoruKHXF0Vu4ryY7scAK4JeMOZ0cBgLPLraU69e+\/+8+ipwBSAMRzEzuTeLzsq8mO7HACuCXjDmdHAYCzy62lewSAhwBFdxU7k+isOAKAsyI7skuOAF4SBQDOLreW6tQv\/s1jsubzjzt1pACI7ioKgOismMRwVmRHdskRwEuiAMDZ5dbSLQD233i1rLx0fm7rm6eKUQDg3iA7ssMJ4JaMO5wdBQDOLreW6tT1f\/J30v\/AEaeOFADRXcXOJDorPsXirMiO7JIjgJdEAYCzy62lOnXgK4\/IJ770z04deQhQdFdRAERnxSSGsyI7skuOAF4SBQDOLreW6tR33\/JVefjpU04dp+64Prd1zVvFKABwj5Ad2eEEcEvGHc6OAgBnl1tLWwDwFMB4bmJnEo+XfTXZkR1OALdk3OHsKABwdrm1VKee\/zt\/JsenXhEKgHhuYmcSjxcFAM6L7MguGQJ4KRQAOLvcWqpTT31oyKkfTwGM5yYKgHi8mMRwXmRHdskQwEuhAMDZ5dZy8buvlZc+2O\/Ur3PFQrm784rc1jVvFaMAwD1CdmSHE8AtGXc4OwoAnF1uLW0BwEOA4rmJnUk8XnyKxXmRHdklQwAvhQIAZ1eR5dTUlHR1dcno6KhTTltbm\/T19UldXZ1nuQcPHpS1a9c63y1btkyGhoakvr7e89pF1\/6a\/HRlt\/MdBUA8N1EAxOPFJIbzIjuyS4YAXgoFAM4Otpyenpbe3l5paWmR9vZ2Mf9uaGiQnp6eOeWOjY3JunXrZPv27dLc3Cz79u2TkZERX8FgjwDwEKB4bqIAiMeLSQznRXZklwwBvBQKAJxdopZBSV2\/O3r0qKc48KpEw6\/\/L3nl8jXOVxQA8dxEARCPF5MYzovsyC4ZAngpFAA4u0Qt\/QSAe7Qgyk1tAcBTAKMQO3sNBUA8XkxiOC+yI7tkCOClUADg7BKzNOsBOjo6nCkB+2MEwOrVq+Wee+5x1gyErQFY+JHb5V8X\/YpTzKFPLk6snrVQ0MTEhDQ2NtZCUxNvI9nhSMmO7HAC8SxbW1tnGYyPj8crIIdXn3P69OnTOaxXaJVMgtcLvRYBmu+PHz8+s\/Cvv79fJicnfdcAvPW3\/1RevfAyHgIUSn\/uBRwBAKC9ZkJ2ZIcTwC0Zdzg7jgDg7Cq2DEv+egOvKQBdFNjd3S0DAwPS1NQ0px4UALhr2JmQHU4At2TckR1OALekAMDZVWQZtvLfLlyf+BcvXjwzPaACYOvWrbJjxw7PrYAX\/u6fy3+84UKeAgh4iB0xAI0jADg0siO7igngBVAA4OwqsgwbxrcL1zMA9Hqz91\/\/v368tgzq3+s3PuR8z2OA47uIAiA+M2NBdmSHE8AtGXc4OwoAnB1s6T4EyBRkFvfpYUB6TkBnZ6ez718\/9kFAYYcGGQHAY4Dju4idSXxmFAA4M7Iju8oJ4CVQAODscmtpBMBf\/N5Vcl3TgtzWM48VowDAvUJ2ZIcTwC0Zdzg7CgCcXW4tjQDgMcDxXcTOJD4zPsXizMiO7CongJdAAYCzy62lEQD6FkCdBuAnOgEKgOis3FeSHdnhBHBLxh3OjgIAZ5dbSyMAeAxwfBexM4nPjE+xODOyI7vKCeAlUADg7HJrSQGAu4YCgOxwArgl447scAK4JQUAzi63lkYA8D0A8V3Ejjg+Mz7F4szIjuwqJ4CXQAGAs8utpREAU3dcn9s65rViFAC4Z8iO7HACuCXjDmdHAYCzy62lCoBF9fNERwD4iUeAnUk8XvbVZEd2OAHcknGHs6MAwNnl1pICAHcNOxOywwnglow7ssMJ4JYUADi73FqqAOAxwJh72BFj3NSK7MgOJ4BbMu5wdhQAOLvcWlIA4K5hZ0J2OAHcknFHdjgB3JICAGeXW0sVAHwPAOYedsQYN44A4NzIjuwqI4BbUwDg7HJrqQKAxwBj7qEAwLgxieHcyI7sKiOAW1MA4Oxya0kBgLuGAoDscAK4JeOO7HACuCUFAM4ut5YLP3K7fKVvg6y8dH5u65jXirEjxj1DdmSHE8AtGXc4OwoAnF1uLcvi1GoAZmeCUyc7ssMJ4JaMO5xdWXLFOadPnz6NYyiXZVmcWg2vsDPBqZMd2eEEcEvGHc6uLLmicAJgampKurq6ZHR01PFeW1ub9PX1SV1dXaA3x8bGpLu7WwYGBqSpqcnz2rI4FQ9r3JKdCdnhBHBLxh3Z4QRwy7LkikIJgOnpaent7ZWWlhZpb28X8++Ghgbp6enx9aa57tChQ7Jz504KADzufS3ZEeNQyY7scAK4JeMOZ0cBgLNL1HLfvn0yMjISOApw8OBB6e\/vd+7LEYBE8c8UVpYfRDp0gkslO5w62ZEdTgC3LEvcFWoEwMtdYQJApwxuu+026ezsdEQABQAe9EGWZflBpEOHAiAtrow7nCzZkV2hBYBZD9DR0eFMCfgJBP378uXLI60BwEOCliRAAiRAArVCYHx8vPBNLawAMPP66gG\/RYC68G\/Xrl2yZcsWmZiYCBUAhfcmG0ACJEACJEACEQkUUgBESf7afh3yX7VqlTQ3N0uUXQARmfEyEiABEiABEig8gcIJgKgr\/93bBW1P7d692xEF\/JAACZAACZBArRIonADQp\/rJyclIe\/9tp3IEoFZDnO0mARIgARLwIlAoAeD3VL9s2TIZGhpyDgPScwJ0xb\/7CZ8CgD8AEiABEiABEjhLoFACgI4jARIgARIgARJIhgAFQDIcWQoJkAAJkAAJFIoABcBruwUGBwcdx3GBoHf86hTKunXrnPUXYe9f0HUahqce0xx0\/HKhfi1gZeOwM7dwH3sN3roUZnH4uacJa\/33HIedfS1\/t8E\/HfP79JpuLtKPruYFgDkmWNcQHD582Nk6qP+\/vr6+SH5Mta52MlqzZs2s9zG4b+w+mVH\/vXfv3pplGoedzVK5bd68WbZt2+Z7yFWqTs9J4XH4uXcI1fq6nzjsjHDSd6ro+qla\/90Ghb\/hOjw8XPgHxpoXAOYdARr4ZVF1Sffd7o5URdOePXsi7cSo9U4YYaed8S233CKnTp2SoFMuk\/ZzHsuLw0+v3bp1q+zYsYMCXmTO2SdBv1s351r\/3fr9FswoyTXXXCPHjx93XkJX5C3lNS0A\/N4uaN42mMcOsRp1skdJdGTE\/e+gOtV6R4KwU1G6YsUK+drXvjbz5stq+D0P94zDL+y9IHloT5Z1iMPOawQg7CVrWbYlL\/c6ceKEUxXdcaavpacAyItngHp4PfFr57t48eKaHnZ1o3Q\/OcR50kLPbQDcmUuTuOzM8dUbN250XmJV62I0Dj8VAEePHnXigGt6xBHq9khd2O\/WHtpev3594CvWc\/ljy7BSbsGU4a0TvRVHAFznBlAAzI2vuB2JKUE75DvvvLOmFwHGYacd8Gc\/+1n56Ec\/Ko2NjYFrLRLtBXJcWBx+Zt2EWfintps2barZ+IvDzgxtb9++3RnSjjPKl+PwSa1qFACpoc2uYE4BRGMdZyiRyX820zjs9NpvfOMbzpMXdwGc4RiHn3sKoNYZkl20\/g25igIAoZZDG\/uJn4sAvR3kHjoMWwTIFcRnOcZhZ2+ftD1Ry8Oxcfi547LWf89x2FE8xUtOFADxeOX2am4DDHdNnO1EtT7s6qYZh51tW+tPr4ZFHH7uTrnWh7HjsPOaAqjl6ZOwXpECIIxQgb63n7xq\/eAQP7cFHShiFl\/p0LXfU2wtc43KjgLAfwTK7xAqO\/bU2j4IiIfZnNkKGJWdCqa1a9c6TiC74ARGAVCgBM+qkgAJkAAJkAAJzCZQ07sAGAwkQAIkQAIkUKsEKABq1fNsNwmQAAmQQE0ToACoafez8SRAAiRAArVKgAKgVj3PdpMACZAACdQ0AQqAmnY\/G08CJEACJFCrBCgAatXzbDcJkAAJkEBNE6AAqGn3s\/GVEHj66adlwYIFkV89q3uHX3zxRVmyZEkltw20NecwLFu2TIaGhiLXrShvbTTty2qfetb3Sy0wWDAJeBCgAGBYkABAIO4pc1kcHFJJEq\/EFsAHm2hC1o8eOpXVpyhssuLB+5SHAAVAeXzJlmRIII8CIG6dbFxFSXIUABkGOW9VegIUAKV3MRuIErCPldUyzLD64cOHZ45M1b+bY47dxyCbYeoLLrhAurq6ZHR01KmKebmP\/f51u\/z6+nrfKtv3sIfBzatwjeG2bdukvb19Tjl+1xkBsGbNGvnMZz7j2LmH2e2jYngrqfcAAAPDSURBVN33UVa33HKLXHfddY69YXXy5MmZo2jV5lOf+pTs379fBgYGpKmpySnGr01eENwCQP\/98ssvO\/8bHh6exdfL3v3SG73G629FFEdonNOudglQANSu79nyAAJeL+Oxk4\/7advvbWp6i76+Puf1vioCdOha37duxEVHR8dMog56i6Kpjymvrq7OSVx33nnnzPvuw0YA3NfbL4BRkaJnxl9zzTVOfU35e\/fuddYSaCLv7u6elbjt8ozIWbRo0Yy9u43m388\/\/7xT58bGRunt7XWEhhnSD3uZlJcAGBwcFCN4vLjabqYA4M+eBM4SoABgNJCAB4GwRBKWbN1Plm4B4PVK5aA3AHoN0buvD6pT2NsF3W+D0\/qHTQvY3xsB4BY0IyMjM4JAy7QTvP5769atsmPHjlmLFYOG+b0EwOTk5Jx76HVeiyApAPhzJwEKAMYACYQSsIfL3avq\/ZKte5i8ra3NcwTAPRRvV8Zr+N7vfvbb8IIEQNgiRK9k7\/U397SIe5rDjHCYoX39r71gzy5TRxXM2+fczjDTJO6\/ewmAoHuYaQZTDgVAaNjzghoiwBGAGnI2m4oRsJOevQ7Afso0Cd09L2+egN0jAGrrfnINqp1fcg+alrDLq1QAaFnmtbJGoHiNAMQRAI899piYKYagdQ92OygAsBimFQl4EaAAYFyQQEQCdhI1T7g6zKzz5TqX3dLSMmvhnZ3k3QIgaL7fqzpZTAG45\/jte2qyDhrON1MAtgDwetq2pwB0BGDTpk0zaxiiuCGNKYAwMRY2FRKl3ryGBPJIgAIgj15hnapOwGsNgP0Ubi+K81rMZkYEzBSANsgWCaZ8XRBohsi95uENiKQWAdpP3Pa6gOXLl89Z5OcWALatqavWTxf0eQmAqIsAtQyz8DBs7UWliwDNFI3ZuWHaYS9+dAcfBUDVf46sQEoEKABSAstii0\/AJAcdqtePPbxvb+HTIfEPfOADs7b6aeK\/4YYb5NZbb515wnWLAjMqYLYH6j1MYvKjF7RlLurCxM2bN88U7zWcb+bN3YnPfe\/t27c7W\/h04Z9pvz0CoDdxM3RvA3RvhVQbvy2MZtRF\/2tEk9c2QNveK3nb6y\/UT1dddZU88cQTjghxCzXTBvfoSPGjmy0gAREKAEYBCZBAZgQ0IXut\/I9agShrAKKWFfU6jgBEJcXrikaAAqBoHmN9SaAgBNzrHMzTvr3vP25TKADiEuP1JOBPgAKA0UECJJAaAffpiH7b+6JWwP1ynvvuu88xTevdAHwZUFTP8LoiEvj\/ZnLJnmXhf5YAAAAASUVORK5CYII=","height":0,"width":0}}
%---
%[output:4ff6cffb]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Block diagram '<a href=\"matlab:open_system ('afe_inv_psm')\">afe_inv_psm<\/a>' contains one or more parameterized library links. To find the parameterized links use the Model Advisor.  The diagram has been saved but may not behave as you intended. Support for parameterized links will be removed in a future release. "}}
%---
%[output:312632c6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Block diagram '<a href=\"matlab:open_system ('afe_inv_psm')\">afe_inv_psm<\/a>' contains one or more parameterized library links. To find the parameterized links use the Model Advisor.  The diagram has been saved but may not behave as you intended. Support for parameterized links will be removed in a future release. "}}
%---
