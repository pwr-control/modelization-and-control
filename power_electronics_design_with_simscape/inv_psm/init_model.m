clear;
[model, options] = init_environment('inv_psm');;
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

hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, fPWM_AFE); %[output:443cbfc2]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, fPWM_INV); %[output:46b446ec]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, fPWM_DAB, fres_dab); %[output:89d86b7f]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, fPWM_CLLC, fres_cllc); %[output:9cb21187]
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
l1 = Kd(2) %[output:6e66ec03]
l2 = Kd(1) %[output:1e102cde]
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
parasitic_dclink_data; %[output:078fd29d]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:73c23903]

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
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:0a16ef95]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:6bf0c62d]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:2a3b72d6]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:1d18604a]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid_emu_data.w_grid;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:9b4f537e]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:3376e5af]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:887be726]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:8267ed95]
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
run('n_sys_generic_1M5W_pmsm'); %[output:6c6a53dd] %[output:707a07e0]
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
kg = Kobs(1) %[output:82109cd6]
kw = Kobs(2) %[output:7eeccdb8]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:54aaefde]
luenberger_l2 = Klo(2) %[output:62f28817]
luenberger_l3 = Klo(3) %[output:052cd999]
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
lithium_ion_battery = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_inv); %[output:3e4a61fb]
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
    set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
    set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
else
    if use_thermal_model
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'off');
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    else
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    end
end

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
%   data: {"layout":"onright","rightPanelPercent":36.3}
%---
%[output:443cbfc2]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:46b446ec]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:89d86b7f]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:9cb21187]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.880000e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:6e66ec03]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:1e102cde]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:078fd29d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAdYAAAEbCAYAAABnbKUwAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ2QVke55x+uaJhUiDCAwWGAgQQUUy75MC53Eh2ymEU3DqvEm2HQKpwlyMVQZHWo+QpVA6WZYbiZlEGSqQkgd7biTDDKrrBliZEIVZHLrvmQqs2NghkGQjAbLgMVUoLKytbTQ7+c9\/B+nI8+fc7znv+poph53+6nT\/+e7v5P9+nz9KjLly9fJlwgAAIgAAIgAAJGCIyCsBrhCCMgAAIgAAIgoAhAWNEQQAAEQAAEQMAgAQirQZgwBQIgAAIgAAIQVrQBEAABEAABEDBIAMJqECZMgQAIgAAIgACEFW0ABEAABEAABAwSgLAahAlTIAACIAACIABhRRsAARAAARAAAYMEIKwGYcIUCIAACIAACEBY0QZAAARAAARAwCABCKtBmDAFAiAAAiAAAhBWtAEQAAEQAAEQMEgAwmoQJkylm8DRo0epoaGBTp06lQFRW1tLGzdupLKysqJwLly4QC0tLVRfX0\/z5s0rmv7QoUO0dOnSrHQrV66k5uZm8muraGFIAAIg4JkAhNUzKiQEgcIEWFibmppo06ZNNGvWLN\/i5lcMWVi7urpo+\/btVF5enimvoqJCiSsuEACBeAhAWOPhjlJLkEAxYXXOMPXMkjGwOPb29lJNTY2iwt\/xjHXnzp3U2tqa+cwtlm5h5YR8Dx0dHfTd735XCTzPfufOnatmwnv27FG29Cyaf9af82fvvfcetbW10auvvkoHDx6kEydO0JIlS2j69OlZM+P+\/n6aPXs2NTY20mc\/+1n6zne+Qyzmjz\/+uKrL4cOHqbOzk+rq6krQy6gSCBQnAGEtzggpQMATgVxLwVpgnKJbWVmpBK26ulqJlp51njlzRi0ls0DxNTAwoJaRtQC6l4hzCevw8LASvG9\/+9u0bds2JaxTp05VNqZMmUL6e6eAchkshmvXrqUdO3YoYX3uuecyM2EuRy9Ns9gPDQ3RihUraPny5epzFnyuA6fj2fMLL7yghNnrErgnuEgEAoIIQFgFOQu3mmwC+WasWkC1UPLzVi1Qukbu56LHjx\/PzFZ1Gucslz\/zKqwsfnqZmWetPLvs6elRwsv3xjPLfIKrnw27Z9taWPm+9eyaBZd\/57TOuibba7g7EDBPAMJqniksppSAW1gZgxZQXub1K6xaqPLh9LoUzPl5k5NzCVfPaIsJq54t8\/88A929e3fWjBXCmtLGjmoXJABhRQMBAUMECs1Y77jjjszGJq9LwXpp1pne+dyy0OalNWvWZHYYc\/W0qLuXfPWSbb7PncvQ+lktz3gxYzXUaGCmJAlAWEvSrahUHASKvW4TxeYlL6\/b8EYjfh7K4snpz58\/f82mplybl\/QzUr2JigWV7fz2t79VfySsXr1aLf1iKTiO1oYyk0wAwppk7+DeQMASgVzLypaKRjEgUHIEIKwl51JUCAS8EXC+zsM5+Bmsl8AU3qwjFQiklwCENb2+R81BAARAAAQiIABhjQAqTIIACIAACKSXAIQ1vb5HzUEABEAABCIgAGGNACpMggAIgAAIpJcAhDW9vkfNQQAEQAAEIiAAYY0AKkyCAAiAAAiklwCENb2+R81BAARAAAQiIABhjQAqTIIACIAACKSXAIQ1vb5HzUEABEAABCIgAGGNACpMggAIgAAIpJcAhDW9vkfNQQAEQAAEIiAAYY0AKkyCAAiAAAiklwCENb2+R81BAARAAAQiIABhjQAqTIIACIAACKSXAIQ1vb5HzUEABEAABCIgAGGNACpMggAIgAAIpJcAhDW9vkfNQQAEQAAEIiAAYY0AKkyCAAiAAAiklwCENb2+R81BAARAAAQiIABhjQAqTIIACIAACFwlcOn0EI2eVJUaJKkX1pkzZ6bG2agoCIAACNgiMPlDl2ju2Iv0HyecV\/\/vPXMD\/dPQJN\/FDw4O+s4TdwYI68yZJM1x\/MeAtHuOu6EHKR+cg1Dznwec\/TMLksMWZ56dnv9VH53f\/8\/qNsfO\/zqNvXdZoBmrrXsOwrNQHgirQJGS2thMN96o7YFz1IRH7INzaXBmQT37ow1KUHnZlwV1\/IPtoSontW1AWAUK67Fjx2jGjBmhGiwyFycAzsUZmUgBziYoFrcRBWcW0wuv76fz+\/vo4uv7laCymLKomrggrCYoxmBDouOi6CAxoE98keBsx0XgLI+ze7m37Nb5NHb+Mhpz63yjlZE4PjMAzFgxYzXaEUrJGAZ8O94EZzmctaCefX69seXeQrWHsNppG4FKOXToEC1dulTl7ezspLq6uowdiY7DQBSoGfjOBM6+kQXKAM6BsPnOFIYzPzeNarkXwurblfFnGB4epsbGRmpra1M309HRQd3d3VReXq5+h7DG76Ok3kGYgSipdUrifYGzHa\/45WxruRfCasf\/tHPnTmptbc0qzT3T9HorPFvt6uqi7du3U1lZGbW0tFB9fT3NmzcPwuoVYkrT+R2IUoopb7UHfvMOnRi+SCeGL2TSvDV8Uf184uyV\/6\/87jYyrXwM3X3zOJpWXkbNC9MTWCDKNuS1Peda7g36ukzY+kic+HCdE\/WMVS\/Z5hJRLbb9\/f0ZUfTiNLY5MDBAGzduVMlZWKurqzPLwRP\/8fmMmcmTJ3sxGXuaS5f+H40e\/YHY76PUb6BUOJ9671Isrqq4cXSm3I+OHflZf1Zx5fdv\/PtxdPLkSaqsrMy6x2f+1zl6+e2L9MrbIwJ855QxtP5zEzP5Y6mQ8EJzcc6q0puH6PIrPyF6+SdE4yuJPvUAjbrvkVhqvWDBgky5Et\/ZT4yw8pLtnj17aNmyZQUd2dfXVzSN00AxYZX4F5HXvzxj6RElVCg423FmMc486+3ae4x4Bswz2d3fvF39j8sfgVycna\/LXHp3iHh376SHd\/gzHGFqieNz4masUfgHS8FRUE2HzWIDfjooRF9LP5wXPfUavfTmOWpeOANLxD5d4+ScpOXeQtWAsPp0cq7kR48epYaGBjp16pTavcsXP2utqKigHTt20KxZs3yXgs1LvpEhwxUCfgZ8QAtOwC\/nrr1DagZ7z83jaPfDtwcvOGU5mfPUG0YZj44UJUYIa0i6Fy5cyGwsmj17Ni1fvpyWLFminoXy89WDBw+q56S8Acnv5Xzdxv2MVqLj\/A5Efnkh\/QgBcLbTEoJw5uXh2777L2pJeMuSOXTPLePs3KzAUvRy7+mf9xK9eWgkdm8EwRyiQCNxfGYOiXrGumHDBmpvb1evwvBO3pqaGrVRiWedzu9MOlCi44IMRCaZpcUWONvxdFDOLK6rB95QS8P83BXimu0v9+syl277zzTty\/81UDB8Oy3h2lIkjs8QVrzHGld\/EVFu0AFfROUSdJNhOevnrhDXEafmC4YflnMcTQbCGpK6e1aKGWt+oBI7SMjmEUt2cLaD3QRn\/dz1qfo5VH+XjNfmTNL1EgzfBGeT9+zFFoTVC6UCaVhY+bnq4cOHc6aaO3euCvKgIyaFLC6TXaLjJHYQU\/6yaQec7dA2xZlfx3l44I1U7Rj2Ex3JFGc7rWKkFInjc6KWgm06y1mWRMdJ7CBx+TdMueAchp73vCY5v\/SHc7To6dfUrJVnr6V6BQmGb5KzLa4Sx+dECStmrN6bqsQO4r12yUkJznZ8YZqzFtdSfB0nTDB805xttA4Iq0HK\/HrN0NAQNTc3K6v8O1\/OU2lMFSfRcRI7iCl\/2bQDznZoR8G5lMQ113JvkOhIUXCOuoVIHJ8TNWPVDsr1ag1et8luvhI7SNQdMAr74BwF1WttRsVZv+sqdeZqOjpSVJyjbCUQVkN0daAIZ6B83iHM0ZiCBogodGsSHSexgxhqHlbNgLMd3FFy1u+68mk6v13393YqFLKUi6\/vV2ef8rLv6ElVNP7BdhXUIewVJeew95Yvv8TxOZEzVr4p9\/PWqHYEc1kSHSexg0TV8aK0C85R0r1qO2rOTnFNapQmG8Hwo+YcRWuROD4nVlijcFAp\/UUksYPY9KmpssDZFMnCdmxx5kASPHNNkriaXu4tRNoWZ5OtBsIakqaJY+OcB6Q7Z7nOWMHus14lOk5iBwnZPGLJDs52sNvkrKM0DT9xr53K5SklX3SkKG\/KJmdT9ZA4PiduxhrmoHM+Gaejo4O6u7szsYb5uWxTUxOtW7eO2tralK+dabAUbKr5l6YdiQORRE\/Y5sxBJDiYhO0QiLmWe20Gw7fN2URbhLCaoHjFhnPmqc26Z5rFitMHnC9evJi+973vqahNfDJOS0sL1dfXq+D+ENZiFNP9vcSBSKLH4uBsMwRikGAOUfgxDs5h6wFhDUvQcH7eSVxVVUXTp0+ngYEBtaOYLxZW545jiY6T2EEMu9eKOXC2gjm24\/miDoHoXu6d9vQxO0DzlCKxPUscnxO3FGyq1TkDTOiZayFh1eXu27fP1C1EaufkyZNUWVkZaRkwTgTOdlpBnJxfefsifWPXO1Q75wZa\/7mJZir88k\/o8is\/UWef0qceoFF3PkB088gKWZxXnJz91nvBggWZLIODg36zx54+Meex+iXBz1QbGhrU+621tbWZd1z1TFVHaWJh5c+wFOyXMNJL\/Atfotfi5mwiSpPN3b1BfRw35yD3jRlrEGqG8ziPmtOmebdxY2MjNi8ZZp0GcxIHIol+SQLnoFGa4tjdG9THSeDs994hrH6JFUjvDBCxbds2evHFF2nZsmU0a9asvLmcr9ToRHomy0fRLV26VH3c39+f2bjEv0t0nMQOYrB5WDMFznZQJ4Wz1yhNNoI5REE+KZz91E3i+Mz1S9xSsDOkIQfir6mpUX7QG5B4Z6\/JS6LjJHYQkz6zZQuc7ZBOEudi4nr2Rxvo7PPrVahBDjM49t5l6mcJV5I4e+UlcXxOpLA6A+5v3bpVCevs2bNpw4YN1N7ejoPOiWLbRem1M5RKOokDkUT2SeSsA0nwu65zT\/4PFbuXY\/hqQeX4vdKuJHIuxhDCWoyQx+9zzVgPHDiAIPwOfhI7iEf3JyoZONtxRxI583LvM5s20r87+VOaNn4Mld06n2wGc4iCfBI5F6snhLUYIR\/fIwh\/YVgSO4gP9ycmKTjbcUWSOLtPljk7t47u\/f1Cal44g5oXyljyzee1JHH22rIgrF5JJSydRMdJ7CAJc7un2wFnT5hCJ4qbs35V5sK\/7qdL7w6p2anzIHEdSKL+rsn0VP2c0PWNy0DcnIPUW+L4nKhnrO5ZqtsJUR0dJ9FxEjtIkE4Vdx5wtuOBODizmLKIOs89LbQZycS7rnZo5i8lDs5h6yxxfE6UsDod4IycxJ\/z73zpoA9hneXML9FxEjuISZ\/ZsgXOdkjb5sw7e\/kQcb54djrm1hpPB4nrd12nlY8Rc2i604O2OZtoPRLH50QKq3NXcHl5ufJNrs9MOI1tSHScxA5iyl827YCzHdo2OLOQunf2BnlVRsKh6fm8ZoOz6RYjcXxOpLA6dwXrGSpHVOLQhRzvF++x4nUb0523lAYiW2xMlhPVgM9ievH1A2p2yq\/JmNrZW+xdV5NsTNqKirPJe3TbgrAapItdwYVhSuwgBpuHNVPgbAe1Sc65xNTrUq\/f2jrfdb3nlnF+s1tPb5KzrZuHsNoibbgciY6T2EEMu82KOXC2gjl0wBP3Mq+f56Zha6jPdZXwOo7E9ixxfE7kUnC+3cHYFXx1CJDYQcIOYHHkB2c71P1w1nF69RIv36Fe5o1qZlqMgpTXcfxwLlZnW99DWCMkzbuC+cDyefO8nWnIR8o1NTXRpk2bVOB+Z4D+zs7OrN3FEh0nsYNE2DwiMw3OkaHNMlyIMwds+Ovpkddi+GctpKM\/UkVln5hPZbfW0Jhb59u50QKlSHgdR2J7ljg+J3LGmqvt+tkVrDc\/vfLKK7Rjxw6aMGECjo2LfdiReQMSByKJpJnzxOMH1K3zTJSFVIuoe0b6wUlViRDSXJy1uCb1dRyJ7RnCGmGPdh5Wrl\/ByVccz25Pnz5NLKxtbW105swZHHQeoW9K2bTEgSiJ\/uD3RnkJly8WTb44OIP+TN8zL+nyTHREPGuu\/B\/\/bNQP0yS\/jiOxPUNY\/bS+AmnzPWN1L+HmMsFLwH19fbRq1Spat25dRlj1kXOcp6WlhaqrqzPLwf\/981ePoZt7222GahGtmT\/\/+c903XXXRVsIrJMozsMno\/PY2RC2x1deva\/ySqLxU0Z+H19Jo+57RP148uRJqqx0pIuuJlYsn3rvEq3\/5b\/RH89fovWfm0h3ThljpdxihUjivGDBgkx1BgcHi1Utcd8n7jzWoIR4Cfixxx5TB6I7l395xlpIWHv+w1j6ygNfCVpsLPnOv3+ext4wNpay01SoJM6mzgSN4zg0iTMpL\/0gaa\/jSOSMGauXluYhjdfISzw7bWhoUIEjamtr6aGHHlIzVf5dXxUVFfStb32Lnn32Wdq+fbsKLsEz1vr6+sxGKImOk9hBPLg+cUnA2Y5LSplzkl7HkchZ4vjMvSYxM1a96WjPnj05ezOLp9fISyzOjY2NaikYm5fsDI6lWIrEgUiiH0qdc1Jex5HIGcJqqEf72QGcr0insLpft+nv7896bUei4yR2EEPNw6oZcLaDOw2ck\/A6jkTOEsfnRM1YtaCuWbOG1q5dS4cPH87q1QgQcRWHxA5iZ4g2Wwo4m+WZz1paOMd9Oo5EzhBWO33QeCkSHSexgxh3nAWD4GwBMqXrUIk4A\/hLbM8Sx+dEzVidXZjfRW1tbcWMNc+4JrGD2BmizZYCzmZ5pn3G6qx\/HDuGJbZnCKuhPqjfY21ubvYcwjBM0RIdJ7GDhPFRXHnB2Q75tHJ+eOAN4o1NT9XPofq7JkcOWyJnieNzImesJjYv+WmhEh0nsYP48UlS0oKzHU+kmbPN13EkcpY4PidSWPmmeCmYL33QeZTdW6LjJHaQKH0YlW1wjopstt20c7a1Y1giZ4njcyKFFcfGFR\/MJHaQ4rVKXgpwtuMTcCayIa4SOUNY7fRB46VIdJzEDmLccRYMgrMFyCnbFVyIKO8YXvT0ayrJliVz6J5bxhl1gMT2LHF8FjVj5ZvlEIV8FBwHfTB1SXScxA5iyl827YCzHdrgfJVzlKfjSOQscXxOpLDqZ6xDQ0PEO4P17\/w\/H3bOAfWffPJJYz1eouMkdhBjDrNoCJztwAbnaznr13GGn7jXmBMkcpY4PidSWAsF4eeoTJs3b4awHjtGM2bMMNbhYCg3AYkDkURfgnNur5neMSyRM4TVUI\/Wwfh52dc5Yz148CA1NTWpk2r05+4inYH8ncvGfFD60qVLVXL3ua4SHSexgxhqHlbNgLMd3OCcn7PJAP4SOUscnxM5Y+Wbcu8M5jjBW7ZsoU2bNmUdUu5ujl1dXVRVVaVe02ExPXDgAK1YsSJz0g2n7+jooO7ubiovL1fZJTpOYgexM0SbLQWczfLMZw2cC3M2tWNYImeJ43NihTVId84XWIIFlgUX57EGoZruPBIHIokeA+fiXjMRwF8iZwhr8bbhOQULYW9vb1b6YqfbaGHlTHymq14KPnPmjNrwxGe58sUHnVdXV2eCT0h0nMQO4tn5CUoIznacAc7eOIcN4C+Rs8TxOZEzVudZqq+++qraCXz8+HHV8gpFYjp69Cg1NDTQ448\/rmIMc\/Qmfi67ePFi2rVrV0Fh1c1637593lp4zKlOnjxJlZWVMd9F6RcPznZ8DM7+OH9j1zv0ytsX6ZnFk+nOKWM8Z5bEecGCBZl6DQ4Oeq5jUhKOunz58uWk3Azfh3NJ98iRI5nnpBs2bKD29vbMs1EtpKdOnaLa2lq1sWndunXU1tam3nPVS8CrVq2inp4eLAUnyclC7kXiX\/hC0GbdJjj791qQAP4SOWPG6r9t5MzBO3v5lRredMTLuDwLZfEsthTMxpybl\/SM1Sm4nAablww5KgVmJA5EEt0CzsG85vd1HImcIazB2kbOXIcPH6brr78+M\/Ncu3atp4hLzt3ETiF2vm7T39+fdRydRMdJ7CAGm4c1U+BsBzU4B+fsZ8ewRM4Sx2f2ZuKWgoM3sWA5JTpOYgcJ5p14c4GzHf7gHI6zV3GVyFni+AxhxXus4Xp0ieeWOBBJdAk4h\/eaM4D\/7m\/eTtPKr93UJJEzhDVk28h3XJw26+UZa5BbkOg4iR0kiG\/izgPOdjwAzmY4FwvgL5GzxPE5UTNWt7C6n4WaaXrXWpHoOIkdJCr\/RWkXnKOke9U2OJvlrAP488zVefScRM4Sx+dECauzaeUKaciRk3QYQpPNUKLjJHYQkz6zZQuc7ZAGZ\/Oc9es4zQtnUPPCKlWARM4Sx+fECqu7mTnDEpoWV4mOk9hBzA8d0VsE5+gZSx3w7ZAJV4o7gL\/E9ixxfE6ssDqDP\/BNcgAIDklYVlYWrqXlyC3RcRI7iHHHWTAIzhYgC51J2SETvhTnjuEn\/9M4ccdNShyfEyWs+d5BDd+0CluQ6DgM+FG3ihH74AzOdghEW4oO4F9x42j6P+s\/E21hhq1LHJ8TK6y5fINdwVepYMA33HvzmANncLZDIPpSWFyX73iNTl8g2rJkTtampuhLD14ChDU4u1hzSnQcBnw7TQacwdkOATulcHt+5Gfn6KU3z5F7x7CdO\/BfisTxOVEzVv\/IzeSQ6DgM+GZ8X8wKOBcjZOZ7cDbDsZgVzTlIAP9itqP6XuL4DGFF5KWo+kNJ2MWAb8eN4Gyfs98A\/nbu8NpSIKxxkXeU69xNnC8If2dnZ9a5rhIdh4HITmMDZ3C2Q8BOKe727DXGsJ27y12KxPG5pGasfNxcS0sLVVdXK+HkI+T44uPnGhsb1TmtfJXCsXFSG1ucHTRI2eAchJr\/PODsn1mQHLk4J11cpbaNkjrdxnkeq\/55+vTpSmQ5chO\/B8viW19fnzk6TqLjJN5zkIEg7jzgbMcD4BwvZy8B\/O3cIZaC4+JctFwW0d7eXtKxhjlq08DAgAowwZdzVsu\/c6fGBQIgAAJpJvC36yfSn+74L\/S36yfQjb9oThSKwcHBRN2Pl5spmRmrDjDR3NysZqN6KbimpqagsHqBhDQgAAIgkAYCHMB\/98O3p6GqkdZRrLA6NypxyMOHHnqInnjiCeru7lbB+nV84VWrVlFPT0\/epeBI6cI4CIAACIBA6giIFVa3p9wz1p07d9LBgwepqamJ1q1bl3fzUuo8jgqDAAiAAAhESqBkhJUpeXndxtY5r5F6DcZBAARAAAQSS6CkhDWxlHFjIAACIAACqSEAYU2Nq1FREAABEAABGwQgrDYoowwQAAEQAIHUEICwpsbVqCgIgAAIgIANAhBWG5RRBgiAAAiAQGoIQFhT42pUFARAAARAwAYBCKsNyigDBEAABEAgNQQgrKlxNSoKAiAAAiBggwCE1QZllAECIAACIJAaAhDW1LgaFQUBEAABELBBAMJqgzLKAAEQAAEQSA0BCGtqXI2KggAIgAAI2CAAYbVBGWWAAAiAAAikhgCENTWuRkVBAARAAARsEICw2qCMMkAABEAABFJDAMKaGlejoiAAAiBQnMDZH20onuhKikunhzynDZpw0sM7gmaNLV\/qhXXmzJmxwUfBIAAC6SSwcML7quI3XXeJbvrQXzMQJn\/oUuZn\/k5fzs9NE3vnL6MDm\/y\/fw6e12uhX\/75Ba9JE5MOwjpzJg0ODibGIV5uhP8YkHbPXuqVtDTgbMcjpcD54uv76a9XZm8XXz+gwPHvl94donyzutGTqjKAR39k5OcPOj\/L+n56ljPGzv+6b+dI5CzxntkxEFaBIiW1sfkeCWLOAM52HCCJ8\/n9\/0xO4WRBdV4sliySWiCTtIwpibNmKvGeIaxEJNFxx44doxkzZtgZ9VJcCjjbcX4SOesZKIsozzy1gOpZZtmt8xWcMbfWKBEdc+V3O8SClZJEzsVqInF8hrBCWIu161R\/L3EgkuiwpHA+\/VTDNSKqZ5+SBDRfG0gKZz9tFMLqh1aC0kp0nMQOkiCXe74VcPaMKlTCODjzc88Lr++n8\/v7smajPBNlEQ3yDDMUBAuZ4+ActloSx2fMWDFjDdvuSzq\/xIFIokNscWYxPf+rPrrwr\/uVmPKybikLqbst2OJssg1CWE3StGhLouMkdhCLLjVWFDgbQ1nQUJSc84np2PnLRDwXNemBKDmbvE+nLYnjM2asmLFG1R9Kwq7EgUgieNOc9TIvbzziXbx6ZppGMXW2B9OcbbQ1CKsNykXKOHToEC1dujSTqrOzk+rq6sj5uf5MJ5LoOIkdJAHNw\/ctgLNvZIEymOTMUYNYTPlK0zKvF\/AmOXspz0QaieNzyc1Yd+7cSUNDQ9Tc3Jzx6fDwMDU2NlJbW5v6rKOjg7q7u6m8vFz9LtFxEjuIiU5m2wY42yEelrNe7j37\/Ho1O+WNR2PvXaZ+xnWVQFjOcbCUOD6XnLB2dXVRVVWVmqXqi2er\/Pn27duprKyMWlpaqL6+nubNmwdhjaOnCCpT4kAkCG\/mVoNyZkF996mGzEak8Q+2l+RuXlM+DcrZVPlB7EBYg1AzmIdnpsuXL6fDhw8rq3PnzlVieuTIERoYGKCNGzeqz1lYq6urM+Ir0XESO4hBV1szBc52UPvh7HxNhsMF8nJvkqIb2SEWrBQ\/nIOVYD6XxPG55GasTrfysvDBgwdp8eLFtGvXroLCqvPt27fPfMuIwOLJkyepsrIyAssw6SQAznbag2fOL\/+ELr\/w5MhNfeoBGvWpB4jGox949ZJnzl4NRphuwYIFGesS46KXbKxgvQS8atUq6unpwVJwhJ2gVE1L\/Atfoi8KcdbPT\/WGJDw\/De5hie0ZM9bg\/jaSk5eCN2zYQO3t7WpjEj9X5WvFihXYvGSEcPqMSByIJHopF+dcgsrPUHEFJyCxPUNYA\/ibl2tbW1uzcrpfh\/Fj1vlajX7GyiLr\/Ly\/vz+zcYltS3ScxA7ix49JSQvOdjzh5syvzDh3+EJQzfhBYnuWOD6zt2JZCtZCl0tEtdi6BdBM07rWikTHSewgUfkB5IT\/AAAYCUlEQVQvSrvgHCXdq7aZ89QbRpF+B5Vfk8EOX\/PsJbZnieNzLMLKS7Z79uyhZcuWFWw5fX19RdOYaHoSHSexg5jwlW0b4Bwtcb3D9\/TPe4nePKRCDJb\/Q3vqQg1GSzn7Dxhpx01KHJ9jEVZbjchrORIdhwHfq3fDpQPncPzy5XYHdLg0\/U6a2fzjaAqD1QwBie1Z4vgcq7C63zt1tv+KigrasWMHzZo1K\/JuIdFxEjtI5I6MoABwNgfVfUybjpDES77gbI5zIUsSOUscn2MVVi7cHYKQf+dr+vTpKqjDk09eeWctwnYn0XESO0iELozMNDiHQ5tLTDmggzsYPjiH4+w1t0TOEsfnWIXV\/XoM34z+bM2aNbR582YIa54eI7GDeO38SUoHzv69kU9MCx0eDs7+OQfJIZEzhNWnpy9cuKDCC\/Kyrw6ar6MlNTU10bPPPpsVTN+nec\/JJTpOYgfx7JAEJQTn4s5wH9HGOfweIA7OxTmbSCGRs8TxOdYZq56huuP7btmyhTZt2pQVz9dEo8pnQ6LjJHaQKH0YlW1wzk2WoyCd39+ngt9rIR39kSoq+8R8Kru1xveuXnCOqgVn25XIWeL4nChh3bZtG7344ovqFRvTm5ZwHqudjltqpUgciEz5gEXzr6eHiA8LH\/l\/RESDzEiL3VOaORdjY\/J7iZwhrD5bgF4K5pNm+AzVmpoaZUGfRMNHvJm4cB6rCYrptCFxIPLiKb18y2kvvXuc+HcWTz4thn92Cij\/rA8M\/+CkKt+zUS\/3U6qcvdTdZhqJnCGsPluIc\/PS1q1blbDOnj07K96vT5M5k+M8VhMU02lDykCkA9SzSCqxvCKOLJYj4pktmG7h5GVcFk2+4jiCTQpn6b1AImcIq89Wl2vGeuDAATp16pQ64s3UjJWFFeex+nROiSXXwuOulhaifNU9e+4sjR83vigN5ywvV2ItcLm+Y9HLdxWz68zHG4YygvmRkZ+1WOoQgUUrElMCiQN+TKhCFSuRM4Q1gMvzHU7OgfNNXcWEdfAfRpkqCnYEETg1erLRu\/3jB\/LbOzX6prxl\/THPfbwyZ2UmT8WNo6\/+PPbqzx+9cTTVzrnBaD3iMCbpnNA4+JgqUxJnnMdqyusR2cFScERgU2DW1l\/4XXuzZ60nhi9k0X1r+GLm9xNnHT87Ps\/ljmnlY9TH08aP\/D+1fAxNKx\/Zu8Df1d9l9o+LoE3CFueg91cq+SRyxozVY+srFMqQTTiPe\/NosmAybF4yQTGdNiQORNpTA795h044BXn4ArFAszA7P9fpWWhZgO++ZWTp++6bx9E9t4yz4njJnK0AMlSIRM4Q1gDOzxfSsK6uLoC1\/FlwHqtRnKkxJnEgCuocnjXzTJnF96U3z2XMOAW3eeHV57hBy8mVL02cTXLza0siZwirTy8XCmnY3t5OJp+zFro1iY6T2EF8No9EJAdnIr1M\/es\/nM0I7sgy8keVj0yILTjbae4SOUscn9mbsRx0zgU7dwXrGWpXV5fxXcHFmqxEx0nsIMX8kMTvwTnbK7yEzP9+\/eY5yiW0QUUWnO20fomcJY7PsQorF25jV3CxJivRcRI7SDE\/JPF7cPbmFZ7Vdu09phLzbLZ54QxfG6PA2RvnsKkkcpY4PscurGEbion8Eh0nsYOY8JVtG+Dsn7gWWb1c7GUWC87+OQfJIZGzxPE5FmHlWeqePXtUTOBCV19fX9E0QRqXO49Ex0nsICZ8ZdsGOAcjzsvFvCuZZ7FeBBacg3H2m0siZ4njcyzCyoXqXbqdnZ3k3gHMO4VbW1upv7+f5s2b57ft+E4v0XESO4hvxyQgAziHc4JbYLcsmZPzFR5wDsfZa26JnCWOz7EJq24IWkSdDSOX2HptOEHSSXScxA4SxDdx5wFncx7QS8T8\/NW9PAzO5jgXsiSRs8TxOXZhtdOcCpci0XESO0gSfO33HsDZL7HC6V\/6wzla9PRrannYOXsFZ7Oc81mTyFni+AxhJSKJjpPYQewMHWZLAWezPLW1RU+9piJAaXEF52g4u61K5CxxfC45YXVGWOLK6WVlHHRup+OWWikSByIpPmBx5QhPw0\/cS+Bsx2sSOUNY7bSNgqW4QyRyYsQKToBjhN6CxIFIEuqHB95Qu4efWTyZvnLPHEm3LvJeJbZnCGuApuYMELFt2zZ68cUX1Ss2s2bNCmCNiCM3VVVVZe00xuk2gVAiExFmUhZagZ657v7m7daC\/luoViKLgLDac0siQhoODQ1RTU2NqrU+lNzvQef5ojgdOXKk6EHnGve+ffvskQ9RkqRzFUNUM\/as4GzHBcsGjtPwXz5AvV+eTM6zZ+2Unp5SJLVnnMcasF06g\/Bv3bpVCevs2bNpw4YNZCIIPy8LHzx4kBYvXky7du2ijRs3qjttaWmh6urqzKxW4lKDxL88AzaTWLOBsx38zPmRn51TG5p+u+7v7RSawlIktmeJ4zM3rUTNWA8cOOA5CP\/Ro0epoaFBpa+trVXC6Zzl6iXgVatWUU9PD23fvl19z8JaX1+fCT4h0XESO4jEcQyc7XiNOX\/gwx+l2777LyrOsJcwiHburLRKkdieJY7PsQorF24yCL\/7GDp+3srXihUrqLGxkdra2tTvHR0d1N3dnTmWTqLjJHYQiUMUONvxmubMG5l4QxOet0bDXWJ7ljg+xy6sehcvi9\/y5cvp8OHDoUIZOl+rmTt3rpql8rmuOOg8mo5a6lYlDkQSfeLkrN9xxZKweU9KbM8QVp\/tgM9jfeyxx9Qu4FdffZV4AxM\/D+Xg+48++mjWsq5P076SS3ScxA7iyykJSQzOdhzh5lz+7V\/RPTePo90P327nBlJSisT2LHF8jnXG6t68xK\/J3HfffcY2L3ntKxIdJ7GDePVHktKBsx1vuDnr0IdYEjbLX2J7ljg+xyqsesb6xS9+kXp7ezPPQDFjLd6ZJHaQ4rVKXgpwtuOTXJx5SZgvzFrN+UBie4awBvC\/fva5cuXKrE1GQQNEBLgFxAoOAi0leSQORBJdk4szZq3mPSmxPUNYzbcDKxYlOk5iB7HiTMOFgLNhoHnM5ePMO4R\/\/eY5vNtqyA0S27PE8TnWpWAunF+J4WVg5+XczWuoPRU0I9FxEjuIDV+aLgOcTRPNba8QZ97IhHdbzfhBYnuWOD7HKqzO4Pi8K3j69Ol0\/Phx1YLq6urMtCQPViQ6TmIH8eCKxCUBZzsuKcRZv9vKr9\/wOa64ghOQ2J4ljs+xC6sOX8jxfDnqEr\/PaiqkodfmJ9FxEjuIV38kKR042\/FGMc4ckenum8fRU\/U4ASeMR4pxDmM7qrwSx+dYhZV3BW\/evFmJ6ZkzZzLhCf0sBbuPiXNGcnKGOcR5rFE1+9K2K3EgkuiRYpwxazXj1WKczZRi1gqENQBPjrR0\/fXXq2PiWPzWrl1LO3bs8HRsnH4+yzuKm5ubVen62LhFixZlYgJzYH+ENAzgHGTBsXGW2oCXAZ9fv5laPgaz1hA+8cI5hPlIskJYI8Ga2yjPVPmZLC8f88XCqmer\/PO8efNIz2b51BwWXATht+igEilK4kAkEb0XzogjHN6zXjiHL8WsBQhrAJ4sfq2trVk5\/SwF60D7Wlj1zJRnwH6OjdM3gPNYAzixhLNIOr9Sshu8cv7GrndUNZ9ZPFlydWO7d6+cY7tBR8E4jzWgF9wzzCBmTAnr4OBgkOJjyyPxL8\/YYIUoGJxDwPOR1StnBI3wATVHUq+cw5ViNjdmrD55uo95y5edNznxGap79uyhioqKrGewbmHlE3KwFOzTEUiel4DEgUiiO\/1wRtCI4B72wzl4KWZzQlgD8OTlWr6CvrfqFFa2g81LAZyALBDWmNuA3wEfQSOCOcwv52ClmM0FYfXI0324uTtb0GesbMdp27lbGOexenQOkmURkDgQSXShX85de4eoa+8xFeoQQSO8e9wvZ++Wo0sJYY2ObaSWJTpOYgeJ1IkRGQfniMC6zAbhjKAR\/n0ThLP\/UszmkDg+M4FRly9fvmwWRXFrR48ezQSEcAZyKJ7TfAqJjpPYQcx7LnqL4Bw9Yy4hCGdsZPLvmyCc\/ZdiNofE8TkWYdWbkerr69X7pvq5aNDnrGHdKNFxEjtIWD\/FkR+c7VAPypk3MvH7rVgS9uanoJy9WY8mlcTxORZhde8G5tnr3r17afXq1dF4pohViY6T2EFicW7IQsE5JECP2cNwxoHoHiEHXBnwbj2alBLH58QIa19fHz366KNUVlYWjXcKWJXouDADkXXAggsEZzvOC8NZLwnjaLnivgrDubj1aFJIHJ8hrEQk0XESO0g03S5aq+AcLV9tPSxnhDv05qewnL2VYjaVxPE5NmHlQA4cgD\/X5ed1GxMulOg4iR3EhK9s2wBnO8RNcNbPW3d\/83a655Zxdm5cWCkmONuussTxORZhte2YYuVJdJzEDlLMD0n8HpzteMUUZ37e+tKb5wjimttvpjjbaRUjpUgcn8ULq\/s8VmcgCK5cZ2eniuqE81htdoXSKUviQCSRvmnOHJkJ4nptSzDN2UZbg7DaoOwoI9d5rG6h5eS8CxnnsVp2TokUJ3EgkojeNGc9c8WGpuzWYJqzjbYGYbVB+UoZuc5j5a9yvRPLs1Wcx2rROSVUlMSBSCL+KDjrsIcQ16stIgrOUbc3CGvUhHPYz3W6jd4UpTdBHTlyhAYGBmjjxo3KAp+UU11dnQn8L9FxEjtIDM0jdJHgHBqhJwNRcdav4nA84S1L5qR+U1NUnD05OWAiieMzVzWWkIYBGV+TzX26jTMBDjo3RTm9diQdDC3ZS1FyPvXeJfqfb7xPvf\/7HN05ZQyt\/9xEqrhxtGRcge89Ss6BbypPRhx0bpqoy57X81jdt6GXgFetWkU9PT20fft2FYCCZ6w6nCLnkfgXkcS\/PCNuJpGYB+dIsF5j1AZnnr2ufu4NOjF8kdK6PGyDs+kWI3F8LqkZqztUop7NrlixApuXTLf2lNiTOBBJdI1NzvzsdeA3f1QCy0vELLL1d02WiM33Pdvk7Pvm8mSAsJoi6cOOeynY+VqNM9AEzmP1ARVJMwQkDkQS3RcHZxZWPtOVozaxwNbf9VG6++ZxJf0cNg7OYdsjhDUswZjyS3ScxA4Sk3tDFQvOofB5zhw3Z57F\/voPZ1VwCRbZaePH0N23jKfmhVWe6yAhYdycgzCSOD6LXwoO4ih3HomOk9hBTPjKtg1wtkM8SZydIsu1dwqt9Bltkjh7bVkSx2cIKzYveW3fqUwncSCS6KikcublYl4q1rNZzVbqrDapnAu1WQirxB4tVFilNjZpTQSc7XhMGmee1fLlFlw9w1X\/jx9DU3lZubwsMUvK0jgzR4n3jBmrUMdJbWx2hmlzpYCzOZalOCtx14lntzzL5evE8AV6a\/ginTh7MfNZLgY8+9VCzP+zGI8I9MjZ1Caf80pszxLvGcJ6RVjtDB8oBQRAAASuEvjLtLvpb9dPVB\/87foJV\/4f+T3fZ1Hw+7s\/\/Vsos3\/3pzOh8hfL\/O5\/+8diSRL3vejIS4mjiRsCARAAAeEE9FK312rw7DzK66n6OVGaj8Q2hDUSrDAKAiAAAiCQVgIQ1rR6HvUGARAAARCIhACENRKsdo1yPOXnn3+efv\/739NDDz1EM2bMsHsDKSnt7Nmz9MMf\/pA4mPmSJUvotttuS0nN46vm7t27aerUqXT77bfHdxMlXPKlS5foxz\/+Mb388sv01a9+FZwN+RrCaghknGZ+8Ytf0IQJE2jWrFlq4P\/617+uDhzAZZYAn5j08Y9\/XP175pln6Gtf+xqNHz\/ebCGwliHARz5u3bqVHnjgAZo3bx7IREDgl7\/8JV133XX06U9\/mn72s5\/R5z\/\/eYwdBjhDWA1AjMoEHyzQ2NhIbW1tSjT54sG9tbVV\/dzf368GHB7kv\/CFL6i\/7Pv6+qi2tpbKy8ujuq2Ss+uVs644z1x\/8IMf0MqVK+mGG24oOR5RVsgr6\/fff59++tOf0k033aQYQ1j9ecUrZ27HLKyvv\/66+kPxE5\/4hL+CkDonAQhrQhvG0aNHqaGhQd3djh07lLDyZx0dHdTd3U3OA9z5IHcW00mTJkFYffrTD2deBWBR3bx5szp6cPbs2T5LS3dyr6w7Oztp\/\/799LGPfYxOnz6toEFYvbcdr5w3btxITz\/9NH3mM5+hT37yk9Tb20vLli3DKox31HlTQlgNQDRtgv\/a5CUwXpZZv349bdq0SQmrPrydOwQ\/V9WzWf5rk\/\/SrKqqUsJaV1dHN954o+nbKjl7fjl\/+MMfVn7hpfYpU6aUHI8oK+SH9dq1a+mNN96gt956i95++2010D\/yyCNYHfDgID+ceSXspZdeorvvvluNL\/wH\/Je+9CWsdnngXCwJhLUYoRi\/5788m5qasoR1aGiImpubiTvQ8uXL1c+8BMzLwTwAcQe5\/\/77Y7xreUV74cx+eO2119RAP3HiRGKRffDBBzHY+3S3F9bcpvUMlY98xIzVJ2QitbrlZezgpXYWVF7tGjNmjFolGz16tP8CkSOLAIQ1wQ3Ca+fAMlk4J4JzOH5+coO1H1rB04JzcHYmckJYTVCMyEauznHw4EFyLwXrjU0R3UbJmwVney4GazuswdkO53ylQFjj5V+wdHfnyLd5Ca\/WhHMiOIfj5yc3WPuhFTwtOAdnZyInhNUExYhsuDsHF6Nft6moqMjsFo6o+NSYBWd7rgZrO6zB2Q5nzFjj5YzSQQAEQAAEUkIAM9aUOBrVBAEQAAEQsEMAwmqHM0oBARAAARBICQEIa0ocjWqCAAiAAAjYIQBhtcMZpYAACIAACKSEAIQ1JY5GNUEABEAABOwQgLDa4YxSQAAEQAAEUkIAwpoSR6OaIAACIAACdghAWO1wRikgAAIgAAIpIQBhTYmjUU0QAAEQAAE7BCCsdjijFBAAARAAgZQQgLCmxNGoZjQE+MD5lpYW2rNnT1YBK1euVGflSr443uzevXvVub9cx+rqaqqrq1NV0vWur6\/PnJ3qrCt\/v3nzZlqxYgUOzpbcCHDvgQhAWANhQyYQGCGgBcYpOqXAximMfHqSX2FlBlqYV69eXQpIUAcQ8EwAwuoZFRKCwLUECgkrn0TE5+eeOHGClixZQnfccQc1NDTQqVOnaO7cubR9+3Y1mzt06BAtXbqU+MSi+fPn09ixY9VMj2eKPOvlg+zZ1tDQkPpdn3DEd6Nnxmyjt7dX3eCBAweotrZWndvLotjV1ZX5rr+\/X6UZGBhQ3\/PFoumeebK948ePqxlqrjo6Z6xcni6b7TnL3rJlCy1cuJBwZjB6T5oIQFjT5G3U1TiBXEvBWjRfeOEFeu6555SA8tXY2EhtbW1KZLRQOgWU87HIscDmE9aampqcosj2165dq44SnDBhQkaU+XMWVr6HM2fOUEdHBz366KP0\/e9\/n9rb2zOfdXd3Zy3Zch4ui0U933I322ah5jT6cubjz7iefOklZOMOgEEQSCABCGsCnYJbkkPAy4yVZ4YnT57MzFZ17VhIV61aRT09PZnZay7Bdc5Yq6qqqLW1NQsQz1pZBLWA6qVbnoXyrFPPdJ2ZtADqGa7zeTDX6bHHHqNly5apPwKKzVi1sLpFlW3zzJdntNKfN8tpkbjTJBCAsCbBC7gHsQT8CCvPFt0zQxYeLYi8LOxFWHMJpdOOF2HVgsfg9cxUOyGIsOabmUJYxTZt3HgIAhDWEPCQFQS8Ciun42em\/KyVl0X189empibizT08o3MuBa9ZsyazYWjRokWZJWIWQb3kW1lZmUkzffr0nDNW9pBzKZjL27RpE3Fe3rX7u9\/97hqx13ncS8H5dgXzrDjfci+WgtFH0kgAwppGr6POxgh4FVaeRfIuWa+bl5yblJybmgptXsq1FKyXkfXycWdnZ+Z5p3NDlBuIc6ZZaCn4\/vvvV0vZhw8fzpjQz5i5zs4lZWPQYQgEEk4AwppwB+H20kWgkNiZJGHjPVS8bmPSY7AliQCEVZK3cK8lT8CGsA4PD6tl6WnTpmVeyckFNszzUfdz2pJ3HCoIAg4CEFY0BxAAARAAARAwSADCahAmTIEACIAACIAAhBVtAARAAARAAAQMEoCwGoQJUyAAAiAAAiAAYUUbAAEQAAEQAAGDBCCsBmHCFAiAAAiAAAhAWNEGQAAEQAAEQMAgAQirQZgwBQIgAAIgAAIQVrQBEAABEAABEDBIAMJqECZMgQAIgAAIgMD\/BzJMkflGtP8lAAAAAElFTkSuQmCC","height":283,"width":470}}
%---
%[output:73c23903]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:0a16ef95]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:6bf0c62d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:2a3b72d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:1d18604a]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:9b4f537e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:3376e5af]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:887be726]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:8267ed95]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:6c6a53dd]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:707a07e0]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:82109cd6]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:7eeccdb8]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:54aaefde]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:62f28817]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714302e+02"}}
%---
%[output:052cd999]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.994503273143434e+02"}}
%---
%[output:3e4a61fb]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAdYAAAEbCAYAAABnbKUwAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQtwHdWZ5z+ybEDAECwbAkIYG5AD4WFjD6zi8BjiAjI1ZTPJZMuSdzcpRckoGQxbZbskGwJVBGLLKpsMy2PGmxJaklnL3slAsHc3QxiHYUOEE2KDsskQEH5ixMNgvMAgJsuira+tIx8ddfe\/+\/bte7tv\/28VZSR95\/Q5v\/Od87\/f6fM4ZnR0dFT4IQESIAESIAESKAuBYyisZeHITEiABEiABEjAI0BhpSOQAAmQAAmQQBkJUFjLCJNZkQAJkAAJkACFlT6QKQKHDh2S9vZ2r0y9vb1SX1+fWvmGhoakra1N5s2bJ93d3VJXV5fas+yMK1nHKBUyHG666SZZvHhxlCSZtlm7dq1s2LDBK2NHR4d0dXXFKu\/mzZtl1apVsmbNmkzy0Ppt37499f4RCxqNJxCgsNIhMkWgkqITJKw6cF199dXS3NycCpugOqb93KDKlDJQ68D+5JNPxhKtUtLEbQB9xpIlS8aT1aKw1toXobhtnAd7CmseWollrAiBkZERWblypWzdulU2btxYMWHVSLkSz\/WDaAbphQsXRhZJE9HFEa1S0pTS6EZY45TNfU7WI1bjp\/v372fUWoqTVCANhbUCkKv1CHtKTMtgT23Z0dqcOXPkzjvv9IqpA6w9LWqiq8HBwUl\/t\/O44YYb5Gtf+5pn09DQIH19fdLU1ORbdVvAXHs3mtO\/m6nhBQsWyN133y2zZ8\/2BhRbkFA+OqVsnrtjxw6vfPoxU8G33367fPvb3\/ZE1XxcFvp7w9QWXjOY2\/ZmcDZ52QO9Xcf77rtPenp6fJ974MABr3zDw8PjZbLb0OaozO+\/\/3558MEHxdRP+busDTszxe4nIqZd7eea+rr1Mm3d2Ng4\/uXA5bdlyxZvatV8bP9w80NfaFx\/DMsrzA\/DIlubiZbZlN0Va7d\/2WxNHsuWLZNt27aJ9h\/Tdnad9XfmGXbbIi5+flitcYbPnUyAwlqDXuEOpnYVzeDgN3i6A6Lmo6JmRNX9u9\/AHyZK+regsplBcOrUqRPesRphtcugAuYnhLa4uvmUS1j9IiJ3kHMH3CCu+vsgYe3s7JSlS5dOYh8mZPq3U089VQ4ePOh9cfATO32mLQBu2YP8wjx3586dviL58MMPj7\/XtP3NFg5XWN28zN+DxDXMZzXNvn37AgXcLpMrqu6XHyNqV155pfzsZz+bMEL4iaNf\/3KFUW38yqi\/N89Bedtcsh5V1+CwGqtKFNZYuPJhbAYO+xu7GZS0Bna0plGJ6bD2wGUPAvY3dXsgVvEyEZXJwzzbjYwMOb+\/24PEtddeGyis9jf6uPkgYdUoXT9oSjYsotYo+q233prExI6ylNOsWbMm1DHKVLA7TW3Ym\/bU6NRtdy2Lvm\/0i6SV5aJFi7z62hFulAVdUaZ1XRv3Z8PEfAnQ8qNnG9\/zq4\/5nX4B0zoHTQXbHI0\/uW36+OOPewLtF4EG5evOWpgo3c4j7NkmojX+j7iUY8o7H6NZPktJYc1nu4WWOujbrBmYdECZO3eu74pY22bv3r2+UYg+3M5DoySzghctPkLftIOEyx5o9Plx8ymXsNrPVpHUjz2QBw14trB8\/etfjyysfhG+33O1HO5Ud1BEqLYqEKYcNlu\/57lT4mHCGjQF7qYJiz79vpS5dTOvGVyBNl8mggQQ+afdvnYeQe3qRr+GlRHWoFcA9op325dNv7Sn4U1nt7n4vX6owaEst1WisOa26YILXmlhtberoIErriBqLf2230TNxxYNdxDWvO3tNlEiVrWxF\/zoz7q1w43Y3YE9rrC6HN2o1hX0cgmr8aogQdeV0n7C6k4po4g1D8LqN0Ni2tX1v6CI1c4jqG9QWGtvEKaw1l6bTpoStKfZwqaC3SlL884q6Nu\/39QdEtawKVw7itJy6rf6IGGNmo9OsbmiZ6bIXWFV8YqyKMQVHTuic6fTVYjQVLBG00iY3DxKnQq23T0oCnS7hCuSbvRm6mzPXJj6GN9x0\/hNBaOumPZUsPkSZiL9IGH1i\/QNIzdiDVps5k5Dh00F+3HhVDDylur+ncJaXf6pPD3txUthwoSENaxsfu8fg4QV5aPTZuZ9qQs5irBqGr9VwSYvd2WnfbBCnMVLZkrQTqPP\/eIXv+hF034f5eRXv6iLlzRP82XDFfSghT12GttGn3nPPffIXXfdNWmhlaZxhVV\/F7QQytQVfZHzmyZFMwY2x6A6homiLWQ333xzoG+F5aFl8FvUFHXxks0FzdikMrAw08gEKKyRUeXPMOp2G3urDNpu47cgKs5UsFIMm2ZEi4Psk5jC8tHnuFszbrvtNnnuuefGF+v4Raz2oBu0AMvO23336ye8tsDYafX\/jbD6Pfd73\/vehBOE9NAK+31uKdttbIG0B3q\/rVhB23xcrvY7X81Tua1bt05WrFjh4bBnHszq7qDtO2j\/adh2G31W1Egu6N2ozlr4iVZQlK6M\/LY6+UW9QV\/K9PfuSU9B76pNHlFmVvI3YtVOiSmstdOWsWqCVmDGyozGFSfgN+XsrvwO2kdsF7aUAyIqXtkaeaD9Rch8gfBbKYyqywMiEKHq\/z0XwqqdX\/f16UZ6v8HCL3JB33yrj766JaCwVpd\/0qeHTYWHTWH7PbeUIw2Tlr+o6f2mgpUFOlTF78tQrZztXIu+kHlhjbLAQqeHli9fLrfcckvgaT+12HhJ6kRhTUIvG2ndaVEtVVxR1TQ8e7ay7em+ookjqlpSfhGqbHuV8rTMC6u+21BH0k9QxKoDw+rVq2X9+vWp3oZSCmCmIQESIAESKBaBTAurfiO\/4447pLW11RPXIGE14pv2NWPFcg3WlgRIgARIoBQCmRZWfR+hHz2NJOwdq\/veImw1ZymQmIYESIAESIAEohLIrLDq9O5DDz0kt956q+hh72HCqtGsLns3t7K4P7swzjnnnKh8aEcCJEACJFAlAnoz0MyZM6v09NIfm1lhtS99RquC3eojexXW3bt3l06txlOSD25gMiIjTABb0I\/CGeWVTyaF1W+1o8GP7mtUO7SYKa+NhbtpeSzIB3MkIzLCBLAF\/YjCir0kJYuwCNRsx5k\/f77osXLmZ13C3tXV5VsiOnNtOnNK7kcfKhEs+xkGR0a1ORZlMmJFU7tGPHW1sB5yHnYoul+z0ZnDnXnPnj25fK+Bh7HyWZARZklGZIQJUFiTMspMegorhTWpM1I0MEEyIiNMgMKalFFm0lNYKaxJnZGigQmSERlhAhTWpIwyk57CSmFN6owUDUyQjMgIE6CwJmWUmfQUVgprUmekaGCCZERGmACFNSmjzKSnsFJYkzojRQMTJCMywgQorEkZZSY9hZXCmtQZKRqYIBmRESZAYU3KKDPpKawU1qTOSNHABMmIjDABCmtSRplJT2GlsCZ1RooGJkhGZIQJUFiTMspMegorhTWpM1I0MEEyIiNMgMKalFFm0lNYKaxJnZGigQmSERlhAhTWpIwyk57CSmFN6owUDUyQjMgIE6CwJmWUmfQUVgprUmekaGCCZERGmACFNSmjzKSnsFJYkzojRQMTJCMywgQorEkZZSY9hZXCmtQZKRqYIBmRESZAYU3KKDPpKawU1qTOSNHABMmIjDABCmtSRplJT2GlsCZ1RooGJkhGZIQJUFiTMspMegorhTWpM1I0MEEyIiNMgMKalFFm0lNYKaxJnZGigQmSERlhAhTWpIwyk57CSmFN6owUDUyQjMgIE6CwJmWUmfQUVgprUmekaGCCZERGmACFNSmjzKSnsFJYkzojRQMTJCMywgQorEkZZSY9hZXCmtQZKRqYIBmRESZAYU3KKDPpKawU1qTOSNHABMmIjDABCmtSRplJT2GlsCZ1RooGJkhGZIQJUFiTMspMegorhTWpM1I0MEEyIiNMgMKalFFm0lNYKaxJnZGigQmSERlhAhTWpIwyk57CSmFN6owUDUyQjMgIE6CwJmWUmfQUVgprUmekaGCCZERGmACFNSmjzKSnsFJYkzojRQMTJCMywgQorEkZZSY9hZXCmtQZKRqYIBmRESZAYU3KKDPpKawU1qTOSNHABMmIjDABCmtSRplJT2GlsCZ1RooGJkhGZIQJUFiTMspMegorhTWpM1I0MEEyIiNMgMKalFFm0lNYKaxJnZGigQmSERlhAsEWT710WL747U3yxve\/kSSbqqQ9ZnR0dLQcTz506JC0t7fL4OBgrOxmz54tjzzySKw0SY0prBTWpD5E0cAEyYiMMIFgi\/5nXpMb+5+XQ3dfkySbqqQtu7B2dXVJc3NzpMps375d1q5dS2GNRKtyRhwQMWsyIiNMAFvQjyisoV5iIlYKK+5MWbdgZ8ctREZkhAlgC\/oRhRV7SU4sOBXMqeCkrsoBERMkIzLCBIIt1j62V9Y+todTweYda0dHh2jkmtUPhZXCmtQ3KRqYIBmRESZAYY3ESN+ZbtiwwbNtaGiQvr4+aWpqipS2UkYUVgprUl+jaGCCZERGmACFNRYjd5VwlqJYCiuFNZYz+xhTNDBBMiIjTIDCWjKjoaEhaWtrk+Hh4UxEsRRWCmvJzjyWkKKBCZIRGWECwRa61Ua33BR6u01UgGaLTW9vr9TX10dNVlY7CiuFNalDUTQwQTIiI0yAwloyIzti1Uw2btwYea9ryQ8NSUhhpbAm9SuKBiZIRmSECVBYYzEaGRmRlStXytatW710CxculO7ubqmrq4uVTxrGFFYKa1K\/omhggmRERpgAhTUSI3tVcDWiU42OOzs7paenJ3A1MoWVwhrJmUOMKBqYIBmRESYQbLHo\/mflqV2Hi\/2O1V4FXK3o1ETJO3bsCN3mQ2GlsCbp8JqWooEJkhEZYQIU1lBGL730ktx8881y++23R35\/Wu6zgk1+WlBGrKW7NAdEzI6MyAgTwBb0o2BGc+56WvYf+oARq568VK2zgjVivuOOO6S1tdU72J\/Cijt1kAU7O2ZHRmSECWAL+lEwo\/plT3h\/LPR2m2pfG7d582avEebOnRvpHatpzm3btmHvL5jFgQMHpLGxsWC1jlddMsK8yIiMMIHJFgsWLJCPTpgm71y3lsJaCsBypdEFSw899JDceuutop2Zi5eSkeW3aMyPjMgIE8AW9CN\/RjoFrFPBhY9YsQulZ6FTv1dffbX3bpergpNzZmfHDMmIjDABbEE\/8mdkbrahsGIfSsUibAo66CAKrgoObwp2duyqZERGmAC2oB\/5MzJbbT72\/pvy5l\/\/WwwyYxbHjI6OjmasTImKw4g1ET4vMTs7ZkhGZIQJYAv60WRG9jTwsW++IG98\/xsYZMYsKKwZa5AsFIedHbcCGZERJoAt6EeTGenB+3oAv35OeqpH9v\/yxxhkxixqTlij8OVUMKeCo\/hJmA0HREyQjMgIE5hoYUer0+uPl3ce\/Heye\/fuuNlU3T5VYdUtMKtWrfIqqe879+3bJwMDA1U\/M5jCSmFN2vMoGpggGZERJjDRwo5Wu66fKRu++TkKq41IV+rq\/au69WXp0qXewRGzZ8\/2DuZvaGjwfq7Wh8JKYU3qexQNTJCMyAgTOGrhRqvPfeszktexOpWI1azUVfGcNWuW2Ccy8T7WOK5WHVsOiJg7GZERJoAt6EdHGKmoLnrgWe9f\/dzfeoG0XnY6hdV2IQor7lBZtmBnx61DRmSECWAL+tFkUV1y+RlyX8v5HjxGrI4P6ftVfZ9qTwWb6LWlpUUWL16MvS4li7w2Vko4JmXLzo5JkxEZYQLYouh+5EaqumBJp4DNJ69jdSpTwQaKTvsuWbJkgnetWbOmqqKa529BuJuWx6LonT0KRTLClMiIjMIIPPXSYW\/613xcUc3zWJ2qsGK3qo5FXr8FVYoWB0RMmozICBPAFkX1I3OykiGk71P1var7yetYTWHFvl84i6J29jgNTUaYFhmRkUvgb37xqty8+XcTotT7Wi6QK847xRcWhdXCEvUKuY6Ojqpsu8lrY+FuWh4LDoiYIxmRESaALYrgR\/oe9b\/\/74PyrUdfmgBEp363\/MWlov8GffI6VqcWseripU2bNklvb6\/U19d73Izg6uKlRYsWVW1Pa14bC3fT8lgUobMnJUVGmCAZFZuRCurS\/uflqV2HYwuqSZDXsToVYbW32+hVbvbH3sf64osvih4k8cgjj2APLKNFXhurjAhCs+KAiEmTERlhAtii1vxIxfSen+6TvoHhSZWPEqG6ifI6VlNYse8XzqLWOnsaDUhGmCoZ1T4jc6DDd7ftk4ee9hfTz557irRedkbge9QwShRWhw6aCtZ9rGav6z333IM9sIwWeW2sMiJgxJoQJkUDAySj2mWkW2V6HtszaZrX1Fij07BFSZjMEYu8jtWpRKwGmt8+VnP5uIrqvffeK319fdLU1BSVc1ns8tpYZal8hEw4IGJIZERGmAC2yIsfqZD2P\/Oq6CH5QZ9yiamdf17H6lSFFbtVdSzy2liVopWXzl4pHn7PISNMn4zyy0gFtP+XrwZGpFozFdLpU46X+8b2n4at7sUk\/C3yOlZTWEtt8RpOxwERNy4ZkREmgC2y4EdHFhztl6HX\/zlUSI2Y9n75Qjn1pI+HbpPBNY9mQWF1OA0NDUlbW5t3dZz70evj7G040RCXzyqvjVU+AuE5ZaGzV6qupT6HjDA5MsoWI7PQ6MGfvyI7978TSUS1Bt1faJKTjju2pMVHmEC4RV7H6lQi1pGREW+P6vz588f3q7a2tk66Qi4p9FLT57WxSq1v3HQcEDExMiIjTABbpOVHKqLvffChrHxkSPa\/\/cH4dWxhJdKp3D+aVS9fmvvJqoioX9nyOlanIqzuPlbdqzpjxgzv8H1d0NTf3y\/d3d1SV1eHPS8Fi7w2VgoofLNMq7NXqvyVeA4ZYcpklD4jFdC9b43I5l+9Ji8f+gBGoaZEKqK6Dabr+pkVmdLFJPwt8jpWV0RYdQXw3r17veMLedF5qS5WuXQcEDFrMiIjTABbRPUjXZV7eOT\/yn\/+XwciR6D6dLOgaMH5U+U\/fm66V6A0FhnhmpZmQWF1uGmUqh9XTB9\/\/HHvnlZGrKU5WiVSRe3slShLVp9BRrhlyCg6I4089b8zTzlO\/vKn+2TPwZHI0acbhbZcdoacPXb+bp5E1I8WhdWhYr9n1SlgFdoNGzZIQ0NDVfau2sXLa2PhbloeCw6ImCMZkREmMNHCLB7adfB9+eHO12NN3driqf9\/xXlTpPO6GZ4Ye9teQg6yj1vOLNnndaxOZSo4Sw1TS9+CKsWVooFJkxEZ+REw4rnxl6\/KwK7DsaZt7fzMHtHPXzRNLjnzDzKzmAi3enktKKwWz6iH8Jtbb8rbFDi3vDYWrll5LCgamCMZFZOREc6f\/u6QPPzs6x4E9\/YWTOaIhYrnqXUi1158hsw\/55TxqLNWo8+oXGy7vI7VqUSsFNZSXCg7aSgauC3IqPYYGdHUmqloqngmFU5Nf+V5U+TK806R5nOOXuZtxJN+FO5HFFYR71D9VatWwR5XrQvOTcHy2lgQbJkM2NkxSDLKHyMjnB+Njsp3\/2Gf7HlzpOSpWhNxev9OOV7mnX2ytM0\/M\/Y7T\/oRhRX3pDGLsIg1ciYpGlJYw+Gys2PnI6NsMTKrajUSLEe0GSScptblmq6lH1FYcU\/KiQWFlcKa1FU5IGKC5WRkhPPvf\/um\/PrAu4mmaF1x1IMSLpvxCfncp+rHK1Uu4USUyskIPSuPf8\/rWJ3KO9asN2BeG6tSXNnZMWkySs7IfqepYvnj377pbUGJegRfWAmMMOo07QVnnCgLLzltfFuK2aKCa5C+Bf2IEWsoATP9Ozg4CL2Rh\/BDRFU1YGfH+MkonJGK1y\/\/aY9c\/umZsuXXb8hPfvtWWaJMd4r2U6efKItmn+YdiFCpKBN7R3QL+hGFNbq3ZNySESungpO6aJEHRBNp\/rcdr3kLgMoVZbqiefnMT8iXmxtiLwhK2raVTF9kP4rCOa9jNaeCo7RuwWzY2XGD1xoje\/HPjn3vSN\/AKx6EckzLuoJ5Vv3xXpR54sf\/1YQoM48RJ\/YUfolNwojC6kPPb\/vNmjVrvFtuqvnJa2NVilmtiUYa3PLAyH6H+f8+GpX7\/vFl7zLrcgqmEU19l6mCOf\/cU7x9m\/p5+eWX5bOzm9LAXzN55sGPqgk7r2N1ahGriuqmTZsmXGhu3sO2tLRUVVzz2liVcnB2dky62oxMhPnme7+Xn730tgy9\/n7ZoktTe3Osngpm02knyB+e\/YlYC4CqzQi3YvUtyCi8DfI6VqcirDx5qfodNkkJ2NkxvTQY6dVgKmbvfPCh\/HDH67Jz\/zupRJdelDnleJkxrU4+f+E0uajhpAnvMcu1ajYNRrhl8mVBRhTWyB5LYY2MKpOG7Oy4WaIwsqdiP\/xoVP7qyZflhdfKPxVrpmONYJ572gnyJxdPk+OPPfIOU\/8rl1hiMkctojCKk18t2pIRhTWWX3MqOBauTBmzswc3hxFLfX\/4Yd00+R+\/OSi\/ezV9sdTpWD3E4LxTT8jNgh\/6Ee7WZERhxV7iWHDxUmxkmUhQxM5u3lnWffxj8oPtr8rug++XfRrWjSzPnlonjVOOk9bLzqjJLSVF9KO4HZiMKKxxfSaz9nl9IV4poLXQ2U1kqf++9s6\/yM93HZZdb5R\/gY8rlhpZzpxWJ80zj1wDps+\/4rxTqjIVWyl\/CXpOLfhR2gzJiMIa2ceysvo3qMAU1vCmzGpnN1HlGZ84Tn703Bvy5ItHrvUq115Lm4p9JJ6K5YUNJ3kXThsh1angs846K5en\/UTuyAkNs+pHCatV1uRkRGGN5VDuNPDGjRulubk5Vh5pGVNYsyGs9qEE+w6NyOZfvS773xrxClfq5dFhNXPF0n5nGXeBDwdE3DvJiIwwAQpryYzWrl0rGzZs8NI3NDRIX1+fNDVVb+M4hTU9YTVbRv7lw49Ej7z7xe7\/k3pU6UWRU46Xpk+eKNd\/eqqcf\/qJEyqYxok+FA08HJARGWECFNakjLz0KrLbt2+fcHBEWTKOkQmFNbqw2ltGXnj9n70p2HKeDeuWxI0qLz3rZLnu01OPiPOhD8a3j8Ro7lRMKRoYKxmRESZAYS2ZkR2xRr3ZZmRkRFauXClbt271ntvR0SFdXV2+ZXBtkX3RhdUWy98Mvyf\/8zcHZf9bH3hsyz0Fawul5j\/v7JO9I+\/OO+2ETAllXOemaGBiZERGmACFNRajpNO\/ml4\/KqZoMZT+ffny5XLLLbdEmmKudWE107G\/fuVd+fFvynfHpXEAWyybPnmCXHzmH3iXRGcpoozlrCUYUzQwNDIiI0yAwhqZUdjJS5EzcQxtoXXzGBoaktWrV8v69eulvr4ePqIWhNVEnT\/4xavyi92HE6+MtcVy6nEfStvV58nHjjlmfNVr3MU9sBFybkDRwA1IRmSECVBYkzIqOT0San1nq8Lb29tbs8KqUWjPY3tKmqq1D1O\/\/sJpUn\/Cv54gmG7DcEDErkpGZIQJYAv6EYUVe0kKFmZKeeHChdLd3S11dXWTnuJu7UHvcbMcsZrp1CdeOCTf\/Yd9kYTURJv67nLxH56e+HxYdnbsyGRERpgAtqAfUVixl6RooQI7PDzsK67u38JstYgqrOazbdu2FEsdL+sHth+W3mcOByZqOPlY72+fO\/dEWXzJkcMKzO\/iPSnc+sCBA9LY2FjOLGsuLzLCTUpGZIQJTLZYsGDBhF\/u3r27lGyqmiaVa+PSqJG+R+3s7JSenh64QAnZZili1SnepZue9xb+uB8zhXtf6wXen9LYj+nXVvwWjT2YjMgIE8AW9KNwRlkaq3FrHrVIRVjTuDYuzntUtJip2o2lInrwvd\/LtX+5w1dM\/+arF8vJxx9bMSF1C8HOjrsQGZERJoAt6EcUVuwlYxblEFZ7FbDZp6qnNrl7Wc3f5s+fL4sXL5YwW1OBagmrOcJv0QPPTmCpkWj7Z8+Um66ZHplxmobs7JguGZERJoAt6EcUVuglftfE+SUKO+zB2LuHPtiLl8zfWltbvfOHw2z9nl8NYVVRVUG1p3xVUO9rucC7\/SRLH3Z23BpkREaYALagH1FYsZdEiFgjZ5KiYSWFVYV018H35c82DI7XSAV1y19cWrWpXoSWnR0REiEjMsIEsAX9iMKKvSQnFpUSVr8odf2XPiULzq\/PrKhqE7KzY0cmIzLCBLAF\/YjCir0kJxaVEFZXVLM67evXZOzs2JHJiIwwAWxBP6KwhhKwFyzNmjVL2tvbZXDw6PSnnRgd4IDdMZlF2sLqJ6pZnvp1abKzY\/8iIzLCBLAF\/YjCir0kJxZpCqsrqksuP0Puazk\/J2SOFJOdHTcXGZERJoAt6EcUVuwlObFIS1hdUf3zKxul+wvVu9C91OZgZ8fkyIiMMAFsQT+isGIvGbMw08JFmgpWUV3a\/\/z42b7\/\/t+cIf9pcb4iVdPA7OzY1cmIjDABbEE\/orBiLwEWce9NTfzAgAzSiFi\/99QB6Xp4yHuiLlR67lufSav4qefLzo4RkxEZYQLYgn5EYcVeEsFCjybs7+8PvKkmQhaJTcotrBqtzrnr6XFRzdNCJT+Y7OzYxciIjDABbEE\/orBiL4lgEefM3wjZlWRSTmF136tqpFqpw\/JLqnyEROzsGBIZkREmgC3oRxRW7CURLNCVbhGySGxSTmHt+cle6f77PV6Zer98oXxhzmmJy1ftDNjZcQuQERlhAtiCfkRhxV4yZhG2eEkP0u\/r64NXv0V+WAmG5RJWvfLNHKif9\/eqNkZ2duxUZERGmAC2oB9RWLGXWBZBt86YW2hiZVZm43IIq7sKWN+rZu0w\/VKxsbNjcmRERpgAtqAfUVixl1gWflO+JpJtaWnxrnir1qccwtr\/zGtyY\/\/zXhWu+\/RU2fS1S6pVnbI\/l50dIyUjMsIEsAX9iMKKvWTMAt3HmvdVwXa0mvWbaiI3mmXIzo6pkREZYQLYgn5EYcVeElFYNZrt7e2V+vr6yHnbHK2tAAAWSUlEQVSW0zBpxGq\/W+28boas\/PzMchav6nmxs+MmICMywgSwBf2Iwoq9ZMzCfb9qJ9TL0AcGBnK7j7XWo1VtK3Z27OpkREaYALagH1FYsZdYFrpfdcWKFRNWAA8NDUlbW5usW7dOmpubY+VXTuMkEasdrX7jqrNk9Z+eV86iZSIvdnbcDGRERpgAtqAfUVixlzgWKq5LliyZ8NuNGzdWVVS1MEmEddH9z46fB1xLK4HtRmJnx65ORmSECWAL+hGFFXtJTixKFVb76MIrzj1Fttx4aU5qHK+Y7OyYFxmRESaALehHFFbsJWMW5h1ra2tr1aNTv0KXKqzLf\/iC9A0Me1nWwtGFQQ3Kzo5dnYzICBPAFvQjCiv2kjGLsO02kTNJ0bAUYXUP2s\/z7TUILTs7IsQFXpgQGZFRFAIU1liUsrD6N6jApQirfSDE\/a0XSOtlp8fikSdjCituLTIiI0wAW9CPKKzYS5yItZYuOjeLlmrxQAi3YdnZsauTERlhAtiCfkRhxV6SE4u4EWtRFi2Z5mNnx45MRmSECWAL+hGFFXtJTiziCqu9d7VWt9jYTcfOjh2ZjMgIE8AW9CMKaygBe8HSrFmzpL29XWplKtieBq7lRUuMWPFASEZkFJ0AtqSwUlixl+TEIm7EWr\/sCa9mtbx3lRFrPOflgIh5kREZYQIU1liMauU+Vns1cC3vXaWwxnJvnqccAReFFUMiIwor9hLLolbuYy3aNLA2ITs7dnUyIiNMAFvQjyis2EvGLGrlPtairQbm+8PILs4vHxFQUTQwJDKisGIviSisebmP1V4N3HX9TOm6fkZkBnk2ZGfHrUdGZIQJYAv6EYUVe8mYRa3cx\/rAky\/Ltx59yatVEbbZMGKN7OKMWCOgomhgSGREYcVeYlnUwn2sRXy\/qk3Izo5dnYzICBPAFvQjCiv2Esciz\/ex2u9Xr2qaIj\/65pzY9c9rAnZ23HJkREaYALagH1FYsZfkxCLKPtainbZkNx07O3ZkMiIjTABb0I8orNhLcmIRRVjNNLBWqSj7V03zsbNjRyYjMsIEsAX9iMKKvSQnFlGEdc5dT4tOB+ttNkU4xpARazzn5YCIeZERGWECFNakjDKTHgmr\/X5V713V+1eL9OGAiFubjMgIE8AW9CMKK\/aSnFggYS3y+1VtQnZ27MhkREaYALagH1FYsZfkxAIJ69rH9srax\/Z4tSna+1UKazQn5oCIOZERGWECFNZYjIaGhqStrU2Gh4cnpZs9e7b09vZKfX19rDzLZYyEtaj7Vw1fDojY08iIjDABbEE\/orBiLxmzMCcvNTQ0SFdXV+R0lTIME1b7\/eq1F0yVzV+\/pFLFysxz2NlxU5ARGWEC2IJ+RGHFXjJmEXYIf+RMUjQME1b7\/aouWtLFS0X7sLPjFicjMsIEsAX9iMKKvcSJWFtbW6W5uTlyukoZhgmr\/X61SOcD2+zZ2bEnkhEZYQLYgn5EYcVeYlnocYaVusXGTD1v3brVK0FHR0foFHSYsBb9\/aryY2fHrk5GZIQJYAv6EYUVe4kzFTw4OOibptyLl1TA9aPvc800dEtLiyxevNj3+WHCWuSDIQwsdnbs6mRERpgAtqAfUVixl2TEwhZavyIFCau9cOlPLj5VftB2UUZqVNlisLNj3mRERpgAtqAfUVixl2TAIsrCqSjCWqSLzd1mY2fHjkxGZIQJYAv6EYUVe4ljsXnzZlm1apX3240bN8q+fftkYGBAuru7pa6uLnZ+KIFGqhs2bJCFCxeGPiNIWLlw6QhhdnbkaWSECZERGUUhQGGNRUlFTg+H6OzslKVLl3rvP\/Xd6sqVKyXt\/a3m2UECrsJqPtu2bRv\/\/z9\/+DXZ8coH3s87bpoRq761ZHzgwAFpbGyspSqVvS5khJGSERlhApMtFixYMOGXu3fvLiWbqqY5ZnR0dLTcJbCnY2fNmiXt7e2esOrWm0qsFtZTn1TQe3p6pKmpaVL1giJWLlxixBq1LzCqx6TIiIwwAUaskRlVW1iRePsJq71w6aqmKfKjb86JXN9aM+SAiFuUjMgIE8AW9CMKK\/YSy0Lfr+r7VHsq2ESvYVthYj1kzNheBRzlOEU\/YbVPXCrywiVFys6OvZCMyAgTwBb0Iwor9hLHQiPHJUuWTPjtmjVrAveXxn7AWAL3gIhSFi\/1P\/Oa3Nj\/vJdjUU9cMvzZ2bEnkhEZYQLYgn5EYcVekhMLv4i16FfF2U3Hzo4dmYzICBPAFvQjCiv2kpxY+AkrjzI82njs7NiRyYiMMAFsQT+isGIvcSzsfazmT7qftdoH8\/sJa\/2yJ7wiXnHuKbLlxktj17WWErCz49YkIzLCBLAF\/YjCir3EslBR3bRp04QLzaOc4xvrISUahwmrXhOn18UV+cPOjlufjMgIE8AW9CMKK\/aSMYuwYwXRVpjID0lg6AorVwRPhMnOjp2LjMgIE8AW9CMKK\/aSnAorjzKksEZ27jFDDoiYGBmRESZAYY3FSCPTFStWSF9f3\/jpR1mdCuaKYAprLOfmXt9IuCisGBMZUVixlzgRa9B9rHZGen7wI488Ejnvchi6U8FcEUxhjetXHBAxMTIiI0yAwpqUUWbSu8LKM4IprHGdk6KBiZERGWECFNakjDKT3hZW+4xgbrU50kQcELGrkhEZYQLYgn5EYcVe4lj47WNN40jDuAVzI1azh7XoZwQbjuzs2KPIiIwwAWxBP6KwYi+xLPKyj9XealP0M4IprNFdnAMiZkVGZIQJUFgjM8rTPlZutZncrBwQsauTERlhAtiCfkRhxV4yZpEnYV32ty\/If3l62Cv5obuviVzHWjZkZ8etS0ZkhAlgC\/oRhRV7SQ6ngrnVhhFrLMceM+aAiKmRERlhAhTW2IzysHjJbLXhiuCjzcsBEbs6GZERJoAt6EcUVuwlObEwq4LtrTZ\/fNE0+a9fvTgnNUi3mOzsmC8ZkREmgC3oRxRW7CU5sfATVm61YcQax305IGJaZERGmACFNSmjzKT3E1ZutaGwxnFQigamRUZkhAlQWJMyykx6I6zcauPfJBwQsauSERlhAtiCfkRhxV6SEwsjrDf2Py\/9z7zmlfq5b31Gptcfn5MapFtMdnbMl4zICBPAFvQjCiv2kpxYGGHlVhtGrKW6LAdETI6MyAgToLAmZZSZ9BTW8KbggIhdlYzICBPAFvQjCiv2kpxYGGE1h+9zD+vEhmNnx45MRmSECWAL+hGFFXtJTixUWP\/xV\/8kejiEfm66ZrrcsfDcnJQ+\/WKys2PGZERGmAC2oB9RWLGX5MRChfX7P9kpix541isx97AyYo3ruhwQMTEyIiNMgMKalFFm0rvCyj2sFNa4zknRwMTIiIwwAQprUkaZSa\/C2vFXP5W1j+3xysStNhTWuM5J0cDEyIiMMAEKa1JGmUlPYQ1vCg6I2FXJiIwwAWxBP6KwYi\/JiYUK60XL\/06e2nXYOxRCI1Z+jhJgZ8feQEZkhAlgC\/oRhRV7SU4sKKyMWJO6KgdETJCMyAgToLAmZZSZ9Cqsh\/+01ysP97BObhYOiNhVyYiMMAFsQT+isGIvyYnFjIsul3euW+uVtvWy0+X+1gtyUvLKFJOdHXMmIzLCBLAF\/YjCir0kJxa2sP7om3PkqqYpOSl5ZYrJzo45kxEZYQLYgn5EYcVekhOL6Zf\/sbx3RadXWo1WNWrl5ygBdnbsDWRERpgAtqAfUVixl+TEovGP\/oO8P\/erXml5OMTkRmNnx45MRmSECWAL+hGFFXtJTizsiJXCSmEtxW05IGJqZERGmACFNSmjzKQ\/\/Ut3ye+nf9YrD09dorCW4pgUDUyNjMgIE6CwJmWUmfSnffmv5cNpn+LhEAEtwgERuyoZkREmgC3oRxRW7CU5saCwhjcUOzt2ZDIiI0wAW9CPKKzYS3JiMe0bfysfnTCNh0MwYi3ZYzkgYnRkREaYAIU1KaPMpK9f9oRXFp665N8kHBCxq5IRGWEC2IJ+RGHFXpITCyOsvOCcwlqqy3JAxOTIiIwwAQprUkaZSW+ElYdDUFhLdUqKBiZHRmSECVBYkzLKTHoKa3hTcEDErkpGZIQJYAv6EYUVe0kZLQ4dOiTt7e0yODjo5bpw4ULp7u6Wurq6SU8ZGRmRlStXytatW8f\/1tHRIV1dXb4lMsLKwyEYsZbqshwQMTkyIiNMgMKalFHk9EYo58+fL4sXLxbzc0NDg69YqggvX75cbrnlFmlqaoLPMcLKwyEorNBZAgwoGpgcGZERJkBhTcooUfrNmzfLwMCAb9Q6NDQkq1evlvXr10t9fT18jhHWQ3dfA22LaMABEbc6GZERJoAt6EcUVuwlKVqECev27dtl7dq10tvbG1lYp9cf7x1nyM9kAuzs2CvIiIwwAWxBP6KwYi9JycK8b21pafGmht2Piu6qVavGfz179uxQkdWI9WPvvykn\/6RLtm3bllKp85vtgQMHpLGxMb8VqEDJyQhDJiMywgQmWyxYsGDCL3fv3l1KNlVNc8zo6OhoVUsAHm7er6pZ0OIljVaHh4fH\/+7+7D5ChZWHQwSD57do3CPIiIwwAWxBP2LEir2kzBZRRNXvkfrOtbOzU3p6enwXM3nCet4p3l2s\/EwmwM6OvYKMyAgTwBb0Iwor9pIyWqCVwGGPQouZVFh56lIwwXPOOUfyOP1SRveDWZERRCRkREaYAIU1KaNY6dF0rsks7tYcTUdhrU1njuVgCY0pGhggGZERJlCbY1Em37G6h0MY9GZRkh4SoQdCtLa2SnNz8\/g+V3NARNhhEkZYT9j5oHx8\/8+TtjvTkwAJkAAJpEggj7NnmRTWFNuIWZMACZAACZBAqgQorKniZeYkQAIkQAJFI0BhLVqLs74kQAIkQAKpEqCwpoqXmZMACZAACRSNAIW1aC3O+pIACZAACaRKgMKaKl5mTgIkQAIkUDQCFNaitTjrSwIkQAIkkCqBQgmrHjqxYcMGD+jGjRu9PbBF\/ejpVG1tbd4Zy2jfr81N78Tt6+uLdO9t3tnGYWTq6h5YkncGYeWPw8fdm16U\/heHkW1bpH4W5mOmP5kzC\/LSnwojrPbVci+++GKsa+by0phRy2kP\/osWLfIO2zCXyrt5uNf16c+bNm2KfEVf1DJlzS4OI7vs5qalNWvW+N7ElLV6llqeOHzc40nRWd6llilr6eIwMl88urq6vC\/8RelnUURVD\/7J2xexwgirRl36UcfN67egcg0c7sCmXzr6+\/sDbw+yn1uUQbEURjo4Ll++XA4fPixBVxyWqw2rnU8cPujs7mrXJa3nx2VkXxxSlH4WxN5E7\/PmzZP9+\/d743aeZhgLIaxB5wkHRWlpdbSs5OteDB\/noviidPhSGOmXt8suu0weffTRwBmArPhA0nLE4ePOeiR9dl7Sx2HkF7EODAxE+rKbFx5xyvnKK6945np8bXt7O4U1DrxK2fpFqDoIzpgxo6an64L4uhFqnIgi6uUIlWrbtJ4Tl5EyfOihh2TZsmVyxx13FEJY7VmOMB9SYd27d6\/XVEVa4xDXh8w4pVOfHR0dnpgU\/eN+4cgLj0JFrPYLcArr0anfqMKqA+S9995biMVLcQZFHRC\/853vyFe+8hVpbGwMfWedl4EBlTMOH\/Pe2bwn07QrVqyoeT+Kw8hMfa5bt86b8owzi4TaKs9\/p7BmuPU4FTyxceJMUZmURRJVrXMcRmr75JNPTnh\/X+uvGeLwcaeCi7JymoySiwKFNTnDVHOwI1QuXhqS1atXy\/r166W+vt4TkbDFS0VcoehG8WGM7O1IthPX8nReHD4uu6L0vziMivrlAw36FFZEqMp\/53abow0QZxtAUabtXPeMw8hOW5RoLA4fd3AsyjRnHEZ+U8FFmC5HskBhRYQy8HceEHG0EcI2rpvFJrp4Iigay9u+slLcLyqjIgqr1jkOH\/uAiCIdfhCHkX7hWLJkiedORWIU1jcprKWMXExDAiRAAiRAAjVGoBCrgmuszVgdEiABEiCBDBOgsGa4cVg0EiABEiCB\/BGgsOavzVhiEiABEiCBDBOgsGa4cVg0EiABEiCB\/BGgsOavzVhiEiABEiCBDBOgsGa4cVg0EiABEiCB\/BGgsOavzVjimAR27dolU6ZM8U6ZivLRvXNvv\/22nHvuuVHMS7Ix+4Nnz54d627bvNwuZOpXqf2YlX5eSY3ORIUhQGEtTFMXs6JxT\/mpxIb0JOKYJG0lPcC+\/7hSz80Lm0rx4HOqR4DCWj32fHIFCGRRWOOWycaUF\/GgsFbAufmIzBKgsGa2aViwqATs4\/I0jZleffHFF8ePiNPfm2MY3WMazXTl1KlTvUuVBwcHvUebQ\/TtezLt\/MOmlu1n2NOh5go1U7c1a9b43gkcZGeEddGiRXLnnXd62bjTrfbReO5zlNXy5cvlqquu8tIbVm+99Za0tbXJ8PCwl+S2226TLVu2SE9PjzQ1NXm\/C6qTXzu5wqo\/v\/vuu95\/et+ozdcvvd\/l6OjC9Lx86Yjq17TLLwEKa37bjiUXEb9D7+1B3Y0Og24RUZjd3d1efiquek6y3otpRLulpWVcAMNu+zHlMfnV1dWJe+Ueilhde\/uAdhV\/FcB58+Z55TX5b9q0yXtXqwLZ2dk5QRDt\/MyXh+nTp4+nd+tofj548KB3Z6q5Y1YF3Fy+jS5n8BNWveTcfJHw42o7NIWV3TvPBCiseW49lt1X+GwsSMTU1h7EXWH1uy4u7AYbv6jJtQ8rE7odx70FRcuPIjX770ZY3S8KAwMD40KredrCqT\/b1wwavmHTvX7CqtGw+TJgnqF2+oXAjf4prOzceSZAYc1z67HsHgF72tRdZRskYu506cKFC30jVndK1kbuN40b9Dz7xqAwYUWLp\/xE1O937vS4O91tInIzxav\/mmjUFWuNgs2tK67LBd056yesYc8w080mfworO3eeCVBY89x6LPsEAraY2O9Z7ajICKX73tNEbG7EqmndSCsMe5Bohk1P2\/klFVbNy7wrNcLvF7HGEdadO3eKmWqOumWJwsrOWWQCFNYit36N1t0WJxOR6XSjvo9cuXKlzJ8\/f8KCIVs8XWENe5\/qh68SU8HuO1T7mSqCYdO6ZirYFla\/6NCeCtaINe6l22lMBaMvOWhKvEbdndXKIAEKawYbhUWKTsBvEYwdNdqLefwW4ZgI1kwF65Nt8TX560ImM1Xq957TlLhci5fsCNF+7zp37txJi5NcYbXTmrJq+XQhkp+wRl28pHmYd6Ro8VHSxUtmqt6s5Db1sBdtuV5CYY3eb2iZLgEKa7p8mXsFCJhB12wVsad57a0yOjV67bXXTthSo4J6ww03yO233+4Jj77rc8XWRLFmG45WyQz4QdUL25oSdUHVqlWrxrP3m9Y17yVdQXGfvW7dOm+rjC5YMvW3I1Z9iMvQ3W7jbjnSNEFbhcwsgf5rvoz4bbex0\/uJov1+W9tpzpw58txzz3ni7n4BMnVwo\/kKuB8fQQKTCFBY6RQkQAKTCKjQ+a0EjooqyjvWqHlFtWPEGpUU7dImQGFNmzDzJ4GME3DfI5vo1N63GrcKFNa4xGhfSwQorLXUmqwLCZRIwD2NKmgbTdTs3UPxH374YS+pvaUnal5R7HgIfxRKtKkUgf8PS+PGj8Shd7UAAAAASUVORK5CYII=","height":283,"width":470}}
%---
