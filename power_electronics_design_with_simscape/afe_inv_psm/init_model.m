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
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"     4.478239263389039e+01"}}
%---
%[output:7fbcc266]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"     1.838728410453592e-01"}}
%---
%[output:0c17d7f4]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAdAAAAEXCAYAAAAdsBUMAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ9wVdW973\/W2Iu+pJdY\/8QEFCfpTBlvm\/Kq0Kc24uBErn315tI2ROZOcCBTQ0yhKNS+3AwoE3lPglSmkZe2SV7IzOVPfHp5tfqQW+ci4z+iHS521Ntp0KgxQKFCTaaEayp3fius4zo7e5+z\/6y9z177fPcME87J+vv5rbW\/WX9\/F5w7d+4c4QEBEAABEAABEPBE4AIIqCdeCAwCIAACIAACggAEFA0BBEAABEAABHwQgID6gIYoIAACIAACIAABRRsAARAAARAAAR8EIKA+oCEKCIAACIAACEBA0QZAAARAAARAwAcBCKgPaIgCAiAAAiAAAhBQtAEQAAEQAAEQ8EEAAuoDGqKAAAiAAAiAAAQUbQAEQAAEQAAEfBCAgPqAhiggAAIgAAIgAAFFGwABEAABEAABHwQgoD6gIQoIgAAIgAAIQEDRBkAABEAABEDABwEIqA9oiAICksC2bdto8+bNaUCKioqor6+PKisrPYM6duwY1dbWUmtrK1VXV3uKf99999GePXsylmXfvn3U1tZG\/f39VFJS4il9BAYBEEgnAAFFiwCBAARYQA8cOEBdXV1UWFgoUuLvdu3a5Uukggoo579ly5ZUjbgsP\/vZz1KCDgENYGxEBQELAQgomgQIBCBgJ6CHDx+m5uZm6ujo8DwK1S2gXDUemVqFNUCVERUEQOA8AQgomgIIBCDgNAK1jkrV6dWampq0USKPChsbG0Up5s6dSyMjI6kpXCmow8PD4vednZ2OU7tOQqmW8eWXX06bwlXz5vTXrFlDTU1NIq+xsTFqaGiggYEB4mlpLtvo6KgYbXM6W7duFeG4bDxlzU99fb0Iw49aTy4bf8\/\/OL0ZM2bQT3\/6U\/rBD34g4vNnTCsHaIiImhMCENCcYEemSSFgtwZqFQNVwFiUeI2zrq5OCJV1xCnTY6G88cYbhYBVVVWJsNlGtk4Cqk7bvvHGGykBZRssX76cNm7cKEbK1uldu3KXlpamBJRFXwq6FNtly5YJgbeWlcv2\/PPPC6EtLy8X9eI\/FFg0+VGZJKVtoB7JJwABTb6NUcMQCdiNQFmI1q5dmyYWUgS5KJlGhKqgXnnlldTS0kLd3d2pDT8sRBUVFalRolo1rwJq3USkrt3yeq4q3tnKbUWsMmBxtpZN\/SzFV2UUosmQNAhoIwAB1YYSCeUjATsBVQVh0aJFU3bVqnF4RKZO96ojOeYpp3ZVttYpYPk7r1O4UiR5SpWfL3\/5y2LaVh0VqruBMwk\/x1enqcvKykSach0YApqPvSP5dYaAJt\/GqGGIBLIJKK8JehnJZRuBZqqKnYBKQeapV96d6zSdy6NR9XdeR6DWKVu7KVwuu9whjBFoiI0SSUdGAAIaGWpklEQC2aZwefrSzRqoXBOVm3rs1kCluMqwVp52AprpGAuvh1qnmuW6JAtqtjVQ9TwpCyb\/sdDe3i7WQNU1T0zhJrHlo05MAAKKdgACAQjYbSLi5Ky7ZTPtwpXiwztUZ8+eLUqzatUqIUTWXbhO07fWKVRZJeuGJrtRptxle\/\/999MvfvGL1LSruguX07n11lvpd7\/7XWoTkfVCBrWODz74oAgnp4AxhRugkSFqbAlAQGNrGhQMBOJFgP9YGBwcTDuCE68SojQgEC0BCGi0vJEbCBhDQN3xm2362JhKoaAgoJEABFQjTCQFAkkioE4tc70yTR8nqd6oCwi4JQABdUsK4UAABEAABEBAIQABRXMAARAAARAAAR8EIKA+oCEKCIAACIAACEBA0QZAAARAAARAwAcBCKgPaIgCAiAAAiAAAhBQtAEQAAEQAAEQ8EEAAuoDGqKAAAiAAAiAAAQUbQAEQAAEQAAEfBCAgPqAhiggAAIgAAIgAAFFGwABEAABEAABHwQgoD6gIQoIgAAIgAAIQEDRBkAABEAABEDABwEIqA9oiAICIAACIAACEFC0ARAAARAAARDwQQAC6gMaooAACIAACIAABBRtAARAAARAAAR8EICA+oCGKCAAAiAAAiAAAUUbAAEQAAEQAAEfBCCgPqAhCgiAAAiAAAhAQNEGQAAEQAAEQMAHASMFdGxsjBoaGmhgYGBKlefOnUtdXV1UWFjoAweigAAIgAAIgIA7AkYJ6OHDh6m+vl7UrK+vjyorK6fUct++fdTY2EhFRUWOYdyhQSgQAAEQAAEQcCZgjIAeO3aMNm3aRBs2bHA1uuRR6rp162jLli2wPwiAAAiAAAhoJ2CMgGqvORIEARAAARAAgQAEjBJQ69pnZ2enqDpP2fKD9c8ALQFRQQAEQAAEPBEwSkDvu+8+UTmelpVrnTU1NeKzFNfS0tJIp22Hh4c9AUdgEAABEEgygZK\/mqAzb+6nP\/51BRVcPst1VWfMmOE6bFwCGiOgvAa6fPly2rhxo9g8JAWzqqqKmpqaBE\/eZNTS0kLd3d1UUlISOmMWz7Vr19LBgwdDzwsZgAAIgECcCVz5+QmqLz1Ft39xjI79RwH1jRTTvj+6Pw0xb948am9vJ5OE1HgBXbZsGVVXV+dEQF999VVasmSJMHpZWVls2zYL\/NatW2Nfzg8\/\/JD4jxLuSHF\/TCmrKeVke5tSVlP6UxRMJ068R+Nv7qfR\/b1UWTQuRpxF85fSn750Gx07W+C6G0umBw4cgIC6puYhoNMINA4CGnejS6GPezk9NAcEBYGcEUB\/Ipo4MUSn+h8SwsmiefF186lw\/lLx089jKlOMQDNYe9u2bbR582YRYs2aNampYhnFFKMPDQ1Rb28vtba2UkGB+78K\/XQExAGBpBPI1\/7Eoslrm6P7t4tRpxxtFtc+GNjkprxLrRU1SkBra2vFFF+mh+fP+\/v7A6+B8npqc3MzdXR0iOzk\/9XLG0wx+vj4OB09epRmzpwJAQ3c1ZFAvhPIt\/6ke7Rp135MeZcaK6BRd1oeffKUp7wWkHcAV1RUpI1CTTF6vnX4qNsK8ssvAvnQn1g0R\/+1l8689YL20SYENA\/6i3pkhqtr\/czfQUDzoCGgiqETeP+j8bQ85OerL50mvpc\/Qy+IywySLKBRjDYhoC4bms5gvIkoyilc64iTR6SDg4NpZ0xZQGtXPnR+d2t8zzBNTEzQyZMn6LLLLscUrs5GmQdpfXAqXdwyVfn9j87YCiF\/qYqkTNMqnJnSZhG964YS+t6cy3MuqIkT0FPDaaNNKp4hdtLqWNvM1kX4vc4PL83xiQbTNjoaswZqNYTdlKp12jWb8TL93q2A3tGf\/tIIkifigoCpBEq\/MHVz2lVFn31n\/T3\/Tv1O\/b+Md3R0QuB4fXhSxH\/172M08vGEiPffv1xI98ybnhNcZ8+epRMnToh9FkZvyjvyKp37zZNErz9JLJpUPo8u+Pp3iMq\/ERnX7du3C6cf8oGARoDeeqRFZqnzIgW3U7h1DSupvX0TlZVNjkCjuMDBK+Lx8TN08uRJKim5yuwO77XiCB+YgJ0wBk40QAIsoE8cOkGPPDckhPSx71bQzRXFAVL0HvXMmTN0\/PhxcV7RNAH95A9DNLa\/l868+E9Ep4ZTO2kvvvPH3kFoiMHvcv6Hc6AaYHpJggVuz549xPfh8kUK1qv9vKRlF9Y6ZYtNREGJIj4I6CPA07\/37nybXjpymh64fRY9cPu1+hLPkpKJU7h8\/GRs\/3Zt5zZ1wzZlP4m13sZO4XJFpH\/Q0dFR7f4\/cYxFdxdBeiCgn8Ajz70rRqO8RvrLpjmRrI+aIqByJy2f2+T\/6zy3qduSEFDdRGOQXlIuUjClw8fA5CiCgQTkaJQ3J3XUzaabK8JdG41zf5KXHYy\/+UJsR5t2TQwCamDHC1pkU4we5w4f1AaIDwKSwLcfPxTJlG4c+5PdaLO4dj0Vzb\/biAZiyrvU2ClcXmjetGkTbdiwgQoLs9\/wz95a1q1bF6prM1OMHscOb0SvRiGNIyCndG8qn05P3zsnlPLHpT+ZOtrECDSUZpk9UbnmySF567N6rZ6MLTcTFRUVOYbJnpO7EBBQd5wQCgSiJPDi4Gm6c9shCktEcy2gpo82IaBR9gabvKQv0IGBgSm\/nTt3bur6vbCLCQENmzDSBwF\/BHhd9Gttr4hNRbrXRXMhoHYXuQf1gOKPbDixTHmXGjuFG47ZgqVqitFz0eGDkUVsEAhOQD3qwjt0dW0uirI\/JXG0iRFo8LadiBQgoIkwIyqRcAJyc9Hjd80W1wEGfcIWUCe3YdOum+\/b32bQOocd35R3KUagGluCKUYPu8NrRIqkQCAUAnJzEQsoC2mQJ6z+JEebp554SBSPz22atJM2CFNT3qUQUIWAehEDf11TU5O2axfnQIN0CcQFgXgR2PnaMXF7UdDNRToF1Gm0WXTr3UJA8+WBgBpmaendpbW1VVwFKD\/X1dUJn5+4icgwg6K4IOCCgNyhG+TmIh0Cah1t8vQse0Ax5dymC9SegkBAPeGKZ2D1Ank41I6njVAqEAhKIOjNRX4FFKNNZ8tBQIO2ao\/x1aMsfHRl5cqV9PDDD1N3d7dvjyiqgLr1xmKCDzu\/Hd6jSRAcBIwhoO7Q9bq5yGt\/UkebPC3Lx0+mXXdL3o427RoJBDTCriPFs6qqiioqKqinp0ec\/eTLFdifHP\/fzW1FapHlemh7e7uY0nXrD5QFdMeOHcK1URxdmXEdvXb4CE2JrEAgpwRWPfF74rVR9uhy\/4KZrsriqj+dGib2gMIXuY+\/uV\/425RTtAVX5M\/aZjagcKidjVAIv1f9gbJfPimgR44coZaWFs+jULn+ef3116c2EXkRUFnF+vp6Wrp0aQg1DpZkYhwAB8OA2CBgS+Dpt8fowV+fpK+XTaOfL8p+zCVjfzo1TOf+ZWtOnVSbZGY41M6RtVjgRkZGaPHixbR7924xhbtixQpasGCB7f230n8oF1e9rchOPDmMlylcHrWWlZWJEWgcR6EmOwDOUfNCtnlG4NWhMVr0898KJ91Pff8rGd2iWftTykk1jzTfOfiZk+qb\/4Ho0hl5RtJbdeFQ2xsvraHlvbcy0TVr1ogdtG4f685bNR4caruliHAgkAwCbjcXiSnct16jyy48S+Mv\/pNRbsPiaimsgcbVMg7lkuuopaWltiNWHGMxzKAoLghoIpDJLVpqQ9Cvu4lODcfaSbUmHJEkAwGNBLO+TKyXKMiU1eldXKSgjzdSAgGTCKg3F2297eIpG4KofB5dvrCRiipvM6lasS0rBDRk08jp1uHh4Yw58W7Y\/v7+SNYiTTG6q12DIdsPyYOAaQT2v\/JvtLfrMfr+x9tF0eVlBxd9o46OHj1KM2fOpIKCAtOqFcvymvIutcK74Ny5c+diSTRLoay7ZDm49fKDsOtlitEhoGG3BKSfFAJyivbMWy+I4yd8bnPb2Vvo8My\/o3\/8h1uFRxf0J\/3WNuVdmggBVY+xqE61eVrWzzEWv83BFKOjw\/u1MOLlA4Fsvjatly78\/VemYwSquWGY8i5NhIByJeSxlM7OTnHxgdyRa70QXrOd05IzxegQ0DBbAdI2kYAUzfE3X0jtouULDjLdR8sX0fOlC9+bcxn9+OZCTOFqNLwp79LECChXRN0IVFRUJG4iUkekGu1rm5QpRoeAht0SkL4JBFg0J89sbheiyQ9P0QrRdOn9RHp04UsX\/v+qG7AGqsnwprxLEyWgmmznOxlTjA4B9W1iRDScQCbR9Ougev+\/nxSXLrBHl4662WJdFE8wAqa8SyGgweyMKVyN\/JAUCIRBwDo9q440\/YqmWk7+g\/Q3v\/uA2vafpoPvjdEvm+ZARAMaEgIaEGCuo\/MaaltbW9oRGJwDzbVVkD8IZCcgR5m8a1bdPcsxeXpWh2haBVQeY1Evo3\/g9muzFxYhbAlAQCNsGJnOhPo5ByrT4yrIM6S4iShCgyIrEPBAQB1hfnJiaNLbyfn1zChchVmXROS66E3l0+npe+d4qAmCSgIQ0Bi0Bd6Zu3DhQrEr18vDI81f\/epXxNf7SQGFQ20vBBEWBPQTkCNL\/sm7ZVWxtAqm9LOpvxRTU7TbU\/Di4Gm6c9shrIv6NAAE1Cc4ndH8nAPlOB0dHXTHHXfQT37yk5SAevHGwj5IeeQb1webiOJqmfwsFwsiP7wjVv5\/Qvz\/vcnvlVGlJCQFsuDya7RPyXq1glN\/Us+LYl3UG1UIqDdeoYTmUeOuXbs8XeUnR61cIHUN1K0\/0L33\/y2tWrUqlProSnRiYoJGx8aoeDp2C+pimo\/pSIFzqjsLn\/VhYeRHCqUjt+LJP0Avvu4W8ZPXLfmJcmTp1qbZ\/iD146Tbbd5JCweH2jmwaKY1UHmxgpti8cahvXv3Cm8s1k1EbgX00\/\/5zVRWJVdmd8brply6w\/zlLxMiyQsvxL2dutnmXXrFZc5VtvN9eV4YLzj\/U0SWaXB49XtDYLpxUO\/VSbchVddeTDjU1o40nAStDrV5unbdunXU3NwsLl+wE1AuCYsrP9YpXf5OTjvAoXY4NkOqIBBHAm4d1KtOuh\/7bgXdXFEcx+rktExwqJ0D\/DruwnVyZyZvNHrppZdocHAwTUArKirSHHabMm+fbcopByZEliBgLAEv\/Qnrou7MbMq71Fobo7yxOImeWinVn6c7002Gso5AcYzFCz2EBYH8IeBFQCUVeY\/uA7fPIpwXndpWIKAR9h+nEWiQIuAihSD0EBcE8oeAHwFlOjgv6txGIKD5039SNTXF6H47fB6aFFUGgawEgvQnnBe1x2vKu9TYKVy58\/ZPf\/oT8aYdPnIyPDw8xRp+biLK2mMcAphi9CAd3i8bxAOBpBII2p+s\/kXvuiGeu\/ejtJ8p71JjBTRKY7rNyxSjB+3wbnkgHAjkAwFd\/Umui7KAPn7X7HxA51hHU96lEFCNzdQUo+vq8BrRISkQMJaAzv6EddHJZmDKuxQCqrHbmmJ0nR1eIz4kBQJGEtDdn3hK92ttr+T1PbqmvEsTI6C8a7axsXFKB8Qa6NR3ku4Ob+RbD4UGAU0EwuhP+b4uCgHV1DjdJCM3FNXV1aVdbOAmrs4wphg9jA6vkyPSAgGTCITZnx557l165Lkhyrd1UVPepYkYgeo8B6o6zbZewgCH2ia91lBWEIiGQJgCyjXIx3VRCGg0bTeVC4ubetWen+x5Gnjt2rXU19cn7sNV77vFTUR+iCIOCCSfQNgCygR5Spf9i\/LTUTebbq5IticlCGiE\/SaTNxYva6BWjytqFeBQO0KDIisQMIhAFAIqRZSPurx05DQl\/QpACKhBHYCLKkW4tbWVqqurp5QeDrUNMyiKCwIREYhKQGV18mFdFAIaUePVlY1cR128eDE9+uijNDo6SuoaqFt\/oEuWLKEdO3YQj3xLSuJ5o0jUHV6XjZAOCMSRQC76E18BuOjnvxVHXV7\/8Q1xxOKrTHCo7QtbsEiZpnBlytItGa9t2j0yjdLSUurq6hJBGhoaiD+zD1AvAirTr6+vp6VLlwarXAix3TgADiFbJAkCiSSQq\/408vEE3fPPxwTTB2+7jL5eNs14vnConSMT2q1fqhuL5Brmrl27RAmtDrU3btxId999N6lTuKpHlk2bNol4cKidIwMjWxCIKQG3DrXDKD6L6Konfi\/WRVfdUmK8azQ41A6jlWRJ041D7ePHj1NLSws988wztqmNjY2JEeeyZctSa6AsoD09PWJEyjtz4VA7B8ZFliAQcwK5mMK1IpHrojeVT6en750Tc2LZi4c10OyMtIaQI8rOzk4hgPJmopqaGjFqtI5A7TJXd9ry71lQq6qqxOUMOMai1VxIDAQSQyAOAsowVddov2yaI9ZHTX0goDmwHIscrzvyBiB1zZO\/59Fnd3d31o096mUJUnxlVXCRQg6MiixBIOYE4iKgjEleAfjBqXGjz4tCQGPe6MMonilGj1OHD8MOSBMEoiQQx\/707ccPGX1e1JR3qbWdXXDu3LlzUTa+JOVlitHj2OGT1A5Ql\/wiENf+ZPK6qCnvUgioxr5uitHj2uE1mgJJgUBkBOLcn0xdFzXlXZoYAYU7M\/fvizh3ePe1QEgQiAeBuPcnE9dFIaARtm15CcLq1avp2WefpebmZiovL0\/bRRtFcUwxetw7fBS2Qh4goIuAKf3JpHVRU96liRiBqudA+SaLioqK1NETt7tvdXQmU4xuSofXYROkAQJhEzCpP5myLmrKuzQRAiovQeBr9xYuXEiNjY3iPtvdu3fTyMgI9ff3Zz2+oqOTmWJ0kzq8DrsgDRAIk4Bp\/UldF42razRT3qWJEFBZifXr19OiRYuIbx1iEbU6xHbTidQr\/nAO1A0xhAGB\/CZgmoCyteS6KF8ByJcuxM2\/KATUwD6l3kTEo9ra2lqqq6tL3E1EQ0ND1NvbK9aI2WsMHhAAAf8ETO5PcV0XhYD6b485i5nJ52eSHGqb2jhz1jCQMQhkIGB6f4rjuqipTI25SMGNCzNu8zzCcrsGajcCld5ZvDjUlv5A4\/rWGR4eJtVvaVzLyeV68sknad68eUaMlE0pqynlNMX+JvUnJ6Y8lbvi\/50Q9+f+j28WEV9Kn8tHMj1w4IARfV+yMlJAvYhktkYhz5Na\/Ye68QfKRl+7di0dPHgwWzb4PQiAAAjEisCnl1xGf\/6vy+jTS75IhS+20+f+fDKn5eM\/nHfu3JnTMnjN3BgBVSumXqIQRExZJHnXrupQW3pjcSOgXCYWUf6HBwRAAARMJPDuhbPo2r8M5bzo\/C43bY+GkQKqWtrqKJvFsLCwcEpjCMOhds5bHAoAAiAAAiCQMwLGC6gkJ9dI+bObNVAZXq55cjwe2ba1tYn4Tz31VFaH2jmzGjIGARAAARDIOQGjBVReqDAwMCBAWs9xZqNrN4XLlzOwQ243DrWzpY\/fgwAIgAAIJJeAkQKa6fIDL6bKJsDZHGp7yQthQQAEQAAEkkXAGAFVj7F4HWkmy2SoDQiAAAiAQBwIGCmgmcAF2ZUbB4OgDCAAAiAAAmYQMEZAzcCJUoIACIAACOQLAQhovlga9QQBEAABENBKAAKqFScSAwEQAAEQyBcCENB8sTTqCQIgAAIgoJUABFQrTiQGAiAAAiCQLwQgoPliadQTBEAABEBAKwEIqFacSAwEQAAEQCBfCEBA88XSqCcIgAAIgIBWAhBQrTiRGAiAAAiAQL4QgIDmi6VRTxAAARAAAa0EIKBacSIxEAABEACBfCEAAQ1o6eHh4YApIDoIgAAIOBMo+auJKb\/85A9DjhEmTjj\/jiNNZIhrl+jEifciM8\/l9\/6fyPLSkREENABFFs+1a9fSwYMHA6SCqCAAAkkkcOXnJ4WPBVD+X3z+\/Cd05XlRLDkfhr+X302GmSqaOhgd+48Cz8kcP+s9judMzke4oev3xA5BTHkgoAEs9eqrr9KSJUuovb2dysrKAqQUblQW+K1bt8a+nB9++CHxHyXz5s0LF4iG1E0pqynlZJOYUlbuT7v+96N075K\/Swndmbf2i1bFozsWQicBLLh8lghXcMXkz4vOfxbfXX5N2u9kM5Vx1GZ7PIMQHlMEzySm\/I46cOAABFTD+8mIJKSAxt3oppTTCKOjkHlFgKdDebqUf46\/+QJ9In5OiqVV4FgUWRBZCKVAsvhddAV\/NymYeOwJmPqOiu0IdGxsjBoaGmhgYGAK8blz51JXVxcVFhbmtD2aYvShoSHq7e2l1tZWKiiIbjomp8ZB5iDgkYAUSxbIM2+9kCaULIBSII\/9l2uI309\/882FVFR5m8dcENyOgCnvUmvZYyeghw8fpvr6elHOvr4+qqysnMJ737591NjYSEVFRY5homimphh9fHycjh49SjNnzoSARtEwkIcRBKRgju3fnjayZLG8+Lr5YiQ57br5U0aQ6E\/6zWvKuzTWAnrs2DHatGkTbdiwwdXokv8KXLduHW3ZskW\/RV2kaIrR0eFdGBNB8oIAi+aZN\/fT6P7tqRGmFMxp190iplpZPDM96E\/6m4op79JYC6h+s2ROkQW7trZWbFyRz5o1a6ipqUl83LZtG23evFn8X\/1ehjXF6OjwUbcs5BcnAiyao\/\/am5qW9SqY1rqgP+m3rinvUgioQoCni1taWqi7u5tKSkrS2PDvmpubqaOjQ3wv\/69OKZtidHR4\/R0eKcabgHWkqYpm0fy7AxUe\/SkQPtvIprxLjRFQu9GhtfBB10B5LbWnp8d2QxKPPnl3rdysdN9991FFRUVqdMplMcXo6PD6OzxSjCcBFs5T\/Q\/R6P7e1HQsT80GFU21tuhP+m1vyrvUGAHlgtqJFgvb4OCgWPeUIrdr1y5fFrWKpJoI582PXF+1foaA+kKOSCCgnYDdaLNo\/lIquvXuUI6PQEC1m9CYwYgxAsoj0OXLl9PGjRvTduKq067Hjx8XU7DPPPOML4uyKO7ZsycVVz0eYxVvVbhlBPlX044dO8ThX+s0sK9ChRAJHT4EqEgy9wRODYu1zVNPPERUPIMuvu4WKpx\/d9ZNQEELjv4UlOBn8fk9zw\/vQ+FLaeJ+pt4YAZUjUBa4zs5Oqq6uJnl8paamJvAIVJ4zLS0tFWlZP3sRUAmVj98sXbpUX+vSlNLZs2fpxIkTQuBxDlQTVCSTOwKnhuncv2wlev1JIZx0\/Xfoguu\/M\/n\/CB70J32Qt2\/fLo4iygcCqo+tSEmeCx0dHU0795lpA5DfIrBAt7W1UX9\/vzhOw4+bKVx5lR8LVBxHoWfOnCEerfMoGQLqt3UgXi4J8G1A42\/tFyNOeuegmJotrl1PF32jLvJioT\/pQ84jUP4nrxuFgOpjG3lK6qYi\/qtIrrXK0TA2EUVuEmSY5wTkERSeppU7aQvnLw19mjYTdkzh6m+U2ESkn2moKcpdvny9HU8Py891dXVipy2OsYSKH4mDQEYCfNkB3xAkd9OGuSnIqykgoF6JZQ8PAc3OyHMI9T5c3uCzcuVKevjhh23PbXpOnCglmvIiBbm2KtPCRQp+qCIOCPgjYHfhAQtnce2D\/hIMKRYEVD9YCKi+s6b1AAAU+ElEQVRmplI8q6qqxPlLeV6Tp1bV85mas\/WUnClGR4f3ZFYEjpCAPILCnk7U0SbfQZvtSr0Ii5mWFfqTfvKmvEutNY\/dZfKygOoxFt4AIwX0yJEjjrcH6Tdr5hRNMTo6fNQtA\/llIqBe4s6iyQ+vb8ZxtGlXD\/Qn\/e3blHepMQLKBeWjJCMjI7R48WLavXu3mMJdsWIFLViwIGcXyKsATTE6Orz+Do8UvRGwjjRV0YzzaBMC6s3OfkOb8i41SkC5sPLspyy43aXufo0WNJ4pRoeABrU04nsl4ORbU440TRNNtf7oT15bQ\/bwprxLjRPQ7OjDC4FNROGxRcrJIqCOMD85MWTrKmzSx+Ys4ysOAdVvQgiofqY5TRHHWHKKH5nHlIAcWfJP3vijiqWclmWhdOtbM6bVzFgsCKh+q0FANTB144GFs+Ebdfi2oDBv\/YE3Fg0GRRJGEWBR5Idv\/ZH\/lyLJ34+\/uT9VHzmSTLpY2hkQAqq\/WUNA9TN19MYSxTEWeGMJwaBIMlQCUvRkJiyE\/KjfTwhxfG9SKM8L5uR3k2HVRxVJ\/l6OKi+6YlYipmL9GgMC6pecczwIqGambryxhDkCdXuZfO8P7qRVq1bRjBllmgnoS25iYoJOnzpN04un4y5cn1hZZII8UrQypSEFzSmMtQx2opexjOcvWy+4YnId8qLLrxE\/WSjld\/z\/i5TPQeqc1LgQUH2WhTcWfSynpCTdjTl5Ywkx6ymjXyd3Zlc8+t\/CLAbSjgGBkYISLaU4emHmdEYKrsyYjzW+tVwXFE\/+EXfVFwom0ymeQaX8f\/mTf1dUMPkdHt8E4I3FN7opEeGNRR9L25ScvLGEnK0QUH7gjSVs0tGlP\/LxhLbMdKX1wanxrGV6\/6MzU8K8\/1F6PPWzTNMaxpqIFFIWVX6uvnSa+DezePLn1ZdeLH7iSScAbyz6WgS8sehjGauUrCNO65QuF9aUeXtMOcWqaeWkMFJM5U8WWVWYXxw8Lco1+X26OKeL6jS6qbyYbq6YnpN6xCFT9Cf9VjDlXWqteWyv8tNvIm8p4hiLN14InSwCUkRfOnI6JbQssvxZPlJYWUzzSVQhoPrbOgRUA1MezrMj6w0bNlBhYWHWFPnC+XXr1oV2rR8uUshqAgTIQwIsrvzvpSOnyE5U77qhhO664arETv9CQPU3egioJqZyzZOTY88rlZWVU1KW1\/sVFRU5htFUnIzJmGJ0dPgoWkN+58GCuvO1owLCI89N7ljmEWoSxRT9SX9bN+VdaswUruoL1Fpo9g3a1dXlapSq39SfpWiK0dHhw2wFSNuOgBRUVUwfuP1aIaimP+hP+i1oyrvUGAHVbyL9KZpidHR4\/bZHiu4J8DQvj053vnYsNSplMTX1QX\/SbzlT3qUQUI22N8Xo6PAajY6kfBPgUekjz72bEtKOutlG7uZFf\/LdBBwjmvIuhYAqBOzu3lXdpWETkf6OghRBgIX03p1vix29D9w+i0wbjUJA9bdhCKh+pqGnyBuWWlpaqLu7e8rF9DjGEjp+ZJDnBHg0ymukvNno31rNudELAqq\/4UJA9TMNPUXezdvT02O7IQneWELHjwxAQByHuXPbIUHClCldCKj+hgsB1c809BStIqlm6OUqP\/YOwy7W4vqgw8fVMigXE1CndH\/ZNCf266LoT\/rbLQRUP1NSj7Lw0ZWVK1fSww8\/bDvl6id7eVm9jKsej3HrjWXJkiW0Y8cOIaBheofxUz8ZBx0+CD3EjYrA3\/\/st2Jd9KnvfyXWIor+pK9FwBuLPpZpKUnxrKqqooqKitRUK1+uoMMfqEy\/tLRU3GRk\/exFQGXB6+vraenSpSER8Z8svEf4Z4eY0RL4\/lPH6DcfjtPTS897kok2e1e5oT+5wuQqELyxuMLkPZDqD\/T48eMpAT1y5Ijjxp9MuaijTaeLGHhNtK2tjfr7+8WVgvzAG4t32yEGCAQh8L3ut+ndE2P0+o9vCJJMaHHhjUUfWnhj0cdySkoseiMjI7R48WLavXu3mMJdsWIFLViwIJT7b9VNRTzSHRwcTBNQHgk3NTWlymnKvD2mnEJspEhaOwFeE\/1a2yt0U\/l0evreOdrTD5og+lNQglPjm\/IutZY89t5Y5L23suDqOc0gZpRnQFtbW6m6uprk57q6OiGSOMYShC7igkAwAnx7Ee\/OjeM5UQhoMNvaxYaA6mcaeorWixRqamrSRra4SCF0EyADEHAkIM+Jxm1nLgRUf6OFgOpnGvsUTTE6OnzsmxIK6EDg248fEk6+43TRAvqT\/uZqyrvUqClcdVPPU089RZs3b6ZcuzBTAZpidHR4\/R0eKUZDQK6HxmkqF\/1Jv+1NeZcaI6DyWMmyZcvoq1\/9KtXW1hKvV\/LjdHuQfrNmTtEUo6PDR90ykJ9OAnIql0ehfO1frh\/0J\/0WMOVdaoyAWo+xyOMlfKTF6f5a\/WaFgEbNFPmBgB0BnsrlJw67ciGg+tsoBFQzU3UEysdJ5OUJfi9ScLr3Vj0f2tnZKXbkygebiDQbFcmBgE8CclduHDYUQUB9GjFDNAiofqYkj7DIdU\/Oorm5mTo6OqiystJ1jjId6wUK6hrrG2+8kbpEga\/kwzEW13gREAQiIRCXDUUQUP3mhoDqZ6olRR5hPv\/888TiOTo6muZ5Rb0wXh3x8igU3li04EciIKCNQFw2FEFAtZk0lRAEVD9TLSnyKNNOENW7dvniBOtneGPRgh+JgIBWAnHYUAQB1WpSkRgEVD\/T1BSuNWn2fML31XrxfmIdUVpHnJyHeoG8l8vk4Y0lBOMjSRBwIHD9\/3qNZhZPo3++5ys5YQQB1Ycd3lj0sUxLSd4StHr1anr22WfF2md5eTk1NDQQe2hR76R1U4QwBVTmD28sbiyBMCAQjAB7a2GvLT9fVEJfL4v+WAu8sQSznxob3lj0sZwioMuXL6eNGzcSQ5YXufPmHrtjLPw9Cxivc\/Jjt6NWdYOmcwq3vb2dysrKxIjYy6g4JHRTkoX3iKhII5+oCOTSYwv6kz4rwxuLPpZpKan+ORcuXEiNjY306KOPCq8s7KEl6BSudcrWbhMRvLGEZFwkCwIBCcgNRXfdUEKP3zU7YGreomMK1xsvN6GxBuqGko8w69evp0WLFhFfoMAi6uTLM1vS1ilcDo9jLNmo4fcgEF8CO187RvfufJuiPhsKAdXfJiCg+plqTdFOQOUodM+ePY7Tvnz\/Lj92btRMMTo6vNamhMRiRCAXNxShP+lvAKa8S601j70\/UP2m0peiKUZHh9dnc6QULwK5cL6N\/qS\/DZjyLoWAarS9KUZHh9dodCQVOwJRX\/OH\/qS\/CZjyLjVKQOUVfNZC+zkHqt\/k5hz+RYcPw\/pIM04EeC2U10SjWA9Ff9JveQioZqbyHGhdXZ3nM5+ai+KYnClGR4ePqkUgn1wSkHfldtTNppsrpodWFPQn\/WhNeZcaMwJV3Zl5uThev2mdUzTF6OjwUbYK5JUrArweyiPRD06NU5giiv6k38KmvEuNEVAuKE\/h7t27l7Zs2RLYYnbuzOQod3h4OJW+utsW7swCY0cCIBApgShEFAKq36QQUA1M7QTNLlmva6BO7sycbjXiPOHOTINBkQQI5IAAiyhfOh\/WmigEVL9RIaD6mWpJMZM7Mycn25wx3JlpwY9EQCBnBHhN9KUjp+mB22fRA7dfq60cEFBtKFMJQUA1MmXRc7rcwGs2Tu7M7ERSTduLOzN4Y\/FqFYQHgWgI7HztKK16YpCuvnQaPfbdL2nZXAQB1Wc7eGPRx1KkxML1+uuvi7tu+fo+viD+nnvuCbwT1+4mIlWoOW\/1mkAv7swkAnhj0dwYkBwIaCAw8vEEPfjrk8ReXNh7y4O3XUalXyjwnTK8sfhGNyUivLHoY0l2O28zTbN6ydrJnVlpaanYpKReXs+fvQgovLF4sQTCgkBuCLw6NEY\/\/L+\/J14jnXdNIW393pfEyNTrA28sXok5h4c3Fn0sAwmoV3dmdsVWL5fftGmTCCJ3AFundPl3pszbY8pJYyNFUsYTkBuMWEhZQNmji5c1UvQn\/U3AlHepteaxugs3yhGok4D29PRQV1cX9fX1EdyZ6e8oSBEE4kKABZTXSB95bkgUSYrpTeXFGddKIaD6LQgB1cA0SgGVR2ZaW1upurpajH5ra2tJ3nyEYywaDIokQMAAAiykvFv3\/Y\/OiKMvcmQ6s3iaEFKroEJA9RsVAqqBaVjnQLlodpuIrPnV1NSkXdqQlIsUhoaGqLe3lxoaGojP0OIBARBwJiBHpnxJPQurfHiEyqL6jVmFNDo6RnOuuogW3\/I3QKmBAARUA0TTkjDF6KaU0zT7o7z5QUAdobKovvLbQfr0ksvSKi\/FlX\/KjUlXX3qxEFz1u\/wg5r2Wpr6jYrUG6h17bmNIo8tzoLktjXPufFXhkiVLKO7l5Bo8+eSTNG\/ePCNGyqaU1ZRymmJ\/2Z8eebxHCCmLKj8stB+cOiN+WgVWHcXK\/7O48vOZ4H62I5jF1xpO7eGZdg9\/7s8n014GJthfMj1w4IARfV8ChoAGUD02+tq1a+ngwYMBUkFUEACBJBKQIvrpJV8U1ePPqrDy99bPMpwTDxZHJ3F2w9Aqrm7icJjP\/fmPboMGCveHvsZA8aOODAENSJxFVL2MPmByiA4CIAACaQTsBDOTiLI3GqeHN0pZHx7t2n2vhuNRdRTP43fNjiIbbXlAQLWhREIgAAIgAAL5RAACmk\/WRl1BAARAAAS0EYCAakOJhEAABEAABPKJAAQ0n6xtU1f1LKz1HGyeo0H1QcAzAbU\/efVb7DkzRMg5AQhozk2Q2wLwZRH8sCeZ1atXU3NzM1VWVua2UMgdBAwlwDeYyXu5Da0Ciu2BAATUAywTgkqvMsuWLRNXFMrH6Val9evX06JFi4RocpiKioq0eCbUGWUEgbAIeO1P7JCisXHyKAZmdMKySnzShYDGxxaBSyI7+8DAAHV2dqaEMNO9vhDQwNiRQEIJ+OlPckanqalJuERcuHAh\/iBNaPvgakFAE2JcOW00e\/ZsGhkZIXlJPlfPeg+w6usUApqQBoBqaCXgtz+phcCMjlaTxDIxCGgszeK9UO+8846IdMkllwivMqqAWn2Zqp+xBuqdNWIkn4Df\/qSOOtU\/TpNPLD9rCAFNmN2tbtq4euqIU45Ipa9TddfgmjVriKee8IAACEwSQH9CS8hEAAKasPbhtcMnrPqoDghoJYD+pBVn4hKDgCbMpE4dnqu5ZcsWUVvrlG7CEKA6IKCNAPqTNpSJTAgCmjCz2nV4XueUU7Z2U7oJQ4DqgIA2AuhP2lAmMiEIaMLMatfhMx1jSVj1UR0Q0EoA\/UkrzsQlBgFNmEntOjxX0ekihYRVH9UBAa0E0J+04kxcYhDQxJkUFQIBEAABEIiCAAQ0CsrIAwRAAARAIHEEIKCJMykqBAIgAAIgEAUBCGgUlJEHCIAACIBA4ghAQBNnUlQIBEAABEAgCgIQ0CgoIw8QAAEQAIHEEYCAJs6kqBAIgAAIgEAUBCCgUVBGHiAAAiAAAokjAAFNnElRIRAAARAAgSgIQECjoIw8QAAEQAAEEkcAApo4k6JCIAACIAACURCAgEZBGXmAAAiAAAgkjgAENHEmRYVAAARAAASiIAABjYIy8kgMAemdY3h4eEqdOjs7qbq6OjF1dVuRffv20d69e4XDdidn7W6duHO4hQsX5iVHt7wRLj4EIKDxsQVKYgABJ\/dWBhQ9lCIyjx\/+8If02GOPUUlJSWABtaYXSqGRKAhoIgAB1QQSyeQHAQhoup3Zzyw\/TU1N4mfQESinYU0zP1oWamkiAQioiVZDmXNGIJuAjo2NUUNDA5WWltKePXuopqZGTG0ePnyY6uvraXR0lIqKiqivr48qKytFPdTfLViwQISpqqoSomQVJBaXAwcOUFdXFxUWFpJ1SllOI8tycF4DAwMizRkzZlB\/f78YKVrzlWXi71taWqi7u1uEk+ksW7ZsyrQq\/2716tXU3NycqosbAeUwzEZ9JCf+jqeEe3p6UnXMmbGRMQhkIQABRRMBAQ8EnNZA16xZIwRPCg4naRW51tZWIUIsEG1tbULM+KmtraW6ujoRn3\/X2NhIMr1MAspxWayl2LIQs5h1dHRQeXm5+N3IyIjIh8VWCjsLuvUPAVmm3t5eIaBSMDlNVVBVVHa\/sxNHGUcVSfmdWmb5BwWXbfny5bRx48aUMHswEYKCQGQEIKCRoUZGSSDgdgQqRc1uRKWO6vj3UkzVEZ+bEeiRI0emiBsLWEVFhRjtquLK+ahirIq4HJFK+\/Aod3BwUIyc1f9b7Wc3UnQzApXpOLHMxjgJ7Qh1SAYBCGgy7IhaREQg28tdiqMqoCxCmzdvnlJCHmWy2PkV0JdfflmMVq0Pj\/Q2bNiQUUCtU8FOI8tNmzY57ooNIqB2nGQZMk0bR2RmZAMCrghAQF1hQiAQmCTgR0AzrelZR4JWYck0hWs3ArWKkCrkbkegsgx33HEHPfvss6kdtjpHoJmOtWRjjLYIAnEhAAGNiyVQDiMIZHu5242srHHkpqH29na68cYb00aK1jVQHinu2rUrbR2TQfH6Kj\/qNK3Mh9dTs03h2pVJrp\/yWqQcNc+dO9dxM4\/TGiiXi6d\/1UcVzEyjX\/lHCtZAjegOeV9ICGjeNwEA8ELAj4By+upOW\/4sNwmpo1q+nGH27NmiON\/61rfEpiJ10xLvlF2yZAkdOnTIcReu3KhjJ+TWUZ+1TOpFEPJ399xzT+qIipWTn124P\/rRj8SmKetFFKpQYxeulxaJsLkkAAHNJX3kDQIWApnWBqOE5XYnbBhnNuVGKHm2NMp6Iy8Q8EIAAuqFFsKCQMgE4iKgbkeBum8O0p1eyOZC8nlOAAKa5w0A1Y8XgTgIKI8An3\/++bTLHjJRUu\/CDUoTd+EGJYj4URKAgEZJG3mBAAiAAAgkhgAENDGmREVAAARAAASiJAABjZI28gIBEAABEEgMAQhoYkyJioAACIAACERJAAIaJW3kBQIgAAIgkBgCENDEmBIVAQEQAAEQiJIABDRK2sgLBEAABEAgMQQgoIkxJSoCAiAAAiAQJQEIaJS0kRcIgAAIgEBiCEBAE2NKVAQEQAAEQCBKAhDQKGkjLxAAARAAgcQQgIAmxpSoCAiAAAiAQJQE\/hPubndQw6BuogAAAABJRU5ErkJggg==","height":223,"width":371}}
%---
%[output:6e6ea8f6]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["1.838728410453592e-01"],["4.478239263389039e+01"]]}}
%---
%[output:0a7f75d6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht0","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:4db39541]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht0","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:08f83058]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000e+00","1.250000000000000e-04"],["-1.233700550136170e+01","9.980365045915064e-01"]]}}
%---
%[output:89ccd37d]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["1.819093456368656e-01","2.958796181316803e+01"]]}}
%---
%[output:2e22cd95]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht1","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:1cf68f47]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht1","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:6bacded8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000e+00","1.250000000000000e-04"],["-1.233700550136170e+01","9.980365045915064e-01"]]}}
%---
%[output:63025039]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["1.819093456368656e-01","2.958796181316803e+01"]]}}
%---
%[output:7ea2de59]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:9ff4664a]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:0b291691]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"     3.699727490026117e-02"}}
%---
%[output:375e0e67]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"     1.533540968663871e+00"}}
%---
%[output:89445da1]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"     4.140209036166578e-01"}}
%---
%[output:726a88d3]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714302e+02"}}
%---
%[output:88710ee3]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.994503273143434e+02"}}
%---
%[output:2ea7c278]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAdAAAAEWCAYAAADW7MapAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQtwXsWVZpMoO4SRzBs0fkAYzACThyBxRAGJZognzpsxTNaluHZlyqbWRgiWEClsuby1bK3LqYyNgoMhclZmY2XG1mhmKA1sMsQTT7YUDFg4A87LhLHBASGjiJexg3Fe3jrX6d+tVt\/bffvc\/\/63\/\/vdKhfYf597+37n6\/Pd08+Tjh07dkzgAgJAAAgAASAABFIhcBIENBVeKAwEgAAQAAJAIEIAAgoiAAEgAASAABDwQAAC6gEaTIAAEAACQAAIQEDBASAABIAAEAACHghAQD1Ag4kZgdtvv10MDw+LhQsXit7e3lxheumll8SiRYvE2NiY6OvrEwsWLKg8f9u2beL+++8X\/f39orGxMZd6JdXnvvvuE3v37s0do7gXJ79t375dDAwMiJaWllT4cLA9fPiwuPHGG8XSpUun+CtVBTwLU71XrFhRseZwdvfu3aKjoyO6lw+Gnq+QmRnH\/5lVItAbQUADdVwRq11EASWxWrdunWhtbS2EgNYSIxNnZPC\/9NJLU+PDwTbpA6Pa3FafLZ\/V3d0tOjs7vR4duoDKj4m824gX2AUzgoAWzCGoTrYIcII8pyZxAlE0AZX18REQDrZFEVC9t8LH56ELqPTFwYMHg8ygfXyWlQ0ENCskHe\/j8vWrNsi1a9eK1atXR12TdJm6mvR7zp49WwwNDYnm5ubIRg9WDz\/8cNTVSldTU5NTo5HdbaOjo7HdXro4qM+98847owyH3kP90pVBWN5Ur3tckNb\/neqnd+HK+qiuMQlFXABMCvL6vfWvd932qquuirorVfyoXjKAm3hh8rWs66FDh6LXInvpT728jq3+7kmBU38\/nSc2bPXfVb\/q70Dvof5u43NSUzPdWxVJvetWf7bp3rqNjoXOn82bN1fal85nvT3K56m+c2k3Kr7EvVtvvVXcdNNN0e3UbmS93ca196J92DmG05oXg4Dm6AJT4zU1IFMQUKupBuu4smpDMQVn9X6mRq7+nmSvNvwkATWJmCkI6wEtDwFVg4wqMqZnJ2ERJwIUwJME9H3ve19F\/HU6qvjaeGHyRdL9JB91\/7v4JUlAbfYTExPRmKH8CFB9bvrN9UNP\/2AwcS6tgMbdU21fqoDOmDFDvPjii1NgV9urS1uKKyO5GYevjlPSs\/TMm9ObkGMILdyjIKA5uUQN0mqDUhu0JLUaKOOyNT0omwK\/DKhqQ1KDpenZJjhk41KDhrRV\/y1JQPUgrb6jrLtaT\/lvHAGld3ENDHo5spUZownbuA8UE+ZqsDJ96ctnmwRQ9b+0jfOhtDcJoynbNGFj+pgw+SUOW+nXU089tdILYuKKKbu3fcgkTfRR76eWM3HXtfvYdE+1jvI5tvaqcsXF12pviku7UeuU1B7JZ3HjnXEfUzmFx2AfAwHNyXVx3YRJDTKpO4aC+vXXXx+buahf9fT\/sntTFQPXQOLavZMkoHr3oWuDzUtAdYEhzPSZlXEBXhUTGfBUzG0CqlJQz3ikgMYJOv27jntSJkbl9UxGFyY9w4kbH7V9nOgZc9xHR1I3tmtPSRyfTOLvynvXsU31PVVf2+zjfK0KqGu70Z914YUXGocMJJ62bui0s7FzCqOFewwENCeXJAmGHgBdxJYa1tVXXz2tK8wUcOKCuUsgSRINHbo0AmoLvvLeeQmoKkSELV367N0kvHT\/phHQpK5ZKaCmMV6JkY57Uhefi4Caxrvls1SBsGWwOj9sAmrrok4aaojjiS2jTppE5PqRF9deTf+e1te6gMa9p\/6sc889N\/HjGgKaTeCHgGaDo\/UuLqJo6hJSG7geDNQMNCkQxAV+FwE1ZThxL5skoKa1mbQOzzb+KgNG3DidTWBchZreSQZMWtJB1549e6asKa1GBqreU31HiSUnA7UtS3DtWVAzJdOQQtIwA63Hde3CdeWjiX9FyECpXuoEnqSsMMnXSR9Lce+Z9CyXGda2bNka4EpaAAKak+PV4FCNMVDTWJltPM41YJnGkUzdVmkE1DQGaurOtj07SwF1mf0p3zGrMVBbFyNnDJSorY+rq0HeNknKNParBn6TvS78tHGFDTNZx7h5Ai5Cr\/KpmmOg6gel9M2+ffuMGykkZYWmcX8bl+nZtnZjGgO1+Uz9eLR90OYULoN5DAQ0R1dlNQtXDRBx97R1l9Fruwpo0mw+H+HWux6Tuvps3Xq2oKOPM9l2nFHLuywZUuueNAtX7oxkGl+kXYnksiIdC9uMa\/KznPkp65vUBWviTtzSIVPTMImqLEf3\/vjHPz5lhx\/1Hkkzw+VvNAtX3SFI2rsst3KZhZuG91Q26Z5JHyeq2KkfLUnd6zYuJ7WbWbNmiTfeeCMqIrPgpLaDWbjZBH4IaDY4Ot\/FJEY6mdUv16997Wviq1\/9amX9oMvaQD3YcLtw4xpu3OQTW+argqV\/AJi+gE3brs2dO3fKGGVct5cuJjYBjZsQojtYD4T6fV0wp3uSINFkJXWNKN1ryZIliVkNLQGRfpbrDm2TgeLqaFpAb\/ow07sC47BVRYf8+Y1vfEOsXLky4rB6D\/UZpmUhcpmLi3hK\/yRNXJJlXD8cXe+ZZgzUhJnua3X8Mm5oRuVfEl9c4o2aVbt09zoHuxIULKSAEjnGx8etW4uZvuZs4z4h+BTjESF4Kf862rpCfYKfvKePbf4I4Im2rmqfLlgp6jTmH+JevrVkReEE1HVfRun0trY27z0sawl80rMhoEX1TG3rlTQEkCZLU99Ccs1nL9zaolHOpyd1zxMith4WE2rggD+XCiWgcRNtTK8ny65atSr3kxz84XazhIC64VTGUiYR9RVPtWve9zSWMvqg1u8cJ6I+4ql232axL3Ctscn7+YUSUOpOkpetC5dEhsZVNm3aVNnzNW\/w8DwgAASAABAoLwKFEVD6su7p6Yn64GlShE1ATbPj6mH8s7xUxJsDASAABMJCoBACqo9nukwiojJqt5O8B8GfdHCyPNUkLDehtkAACACB+kaAJkCFdhVCQCmbHBkZqQifi4CagJZjh3QEmFx3p5Yj8aQsd+fOnaH5CfUFAkAACNQ1AldccYWg2B2SkNZcQEn0urq6xIYNG4TcwNhXQOXEovb2duPM3Mcff1wsXrw4chItPMZ1AgH6qFi\/fj2wiSEF8EluLcAH+HDiqeQPJVIQ0BRIJu30kXZ2oauAhuakFHB6F5UfF8DGDCHwSaYW8AE+3sFHCBEqf2qegZpAt2Wgcrxz5syZore3t3ILUzar3j9UJ3GI6WoLbBAAXbliKgf+gD9l5E+QAkqOkuvh9LME582bN0VUIaButN6\/f3+05Rqtq21oaHAzKlEp4JPsbOADfDjhINQPsGAEVK4RVTNO0x6p6u+6Q0N1EoeYrrZvvfWWOHDggJgzZw4E1AAa8ElmEvABPq6xpp56MAopoBxHJNlCQOPRQQBEAOS0O\/AH\/OHwJ9TYDAHleL2ObBEAEQA5dAZ\/wB8OfyCgHPRysg3VSXnAgwCIAMjhGfgD\/nD4E2psRgbK8Xod2SIAIgBy6Az+gD8c\/kBAOejlZBuqk\/KABwEQAZDDM\/AH\/OHwJ9TYjAyU4\/U6skUARADk0Bn8AX84\/IGActDLyTZUJ+UBDwIgAiCHZ+AP+MPhT6ixGRkox+t1ZIsAiADIoTP4A\/5w+AMB5aCXk22oTsoDHgRABEAOz8Af8IfDn1BjMzJQjtfryBYBEAGQQ2fwB\/zh8AcCykEvJ9tQnZQHPAiACIAcnoE\/4A+HP6HGZmSgHK\/XkS0CIAIgh87gD\/jD4Q8ElINeTrahOikPeBAAEQA5PAN\/wB8Of0KNzc4ZqDysemxsLDVOdML40NCQaG5uTm2bpUGoTsoSg7h7IQAiAHJ4Bv6APxz+hBqbUwsonRe5YMECZ6zoyLHVq1dDQJ0Rq01BBEAEQA7zwB\/wh8MfCGgMehBQDq3ys0UARADksA38AX84\/Kl7AeWAUxTbUJ2UB34IgAiAHJ6BP+APhz+hxmbnLlwOOEWxDdVJeeCHAIgAyOEZ+AP+cPgTamx2FtDDhw8L+lPriUBldBLnnV1tEQARAF25YioH\/oA\/HP7UvYCqs3CLMqs2rcNCdVLa9\/QpjwCIAOjDG2kD\/oA\/HP6EGpudM1AJzn333SfWrVtXwaq1tVX09\/eLxsZGDn652IbqpDzAQQBEAOTwDPwBfzj8CTU2pxZQFaTbb79dDA8PV\/5p4cKFore3l4NjVW1DdVJVQfn9zREAEQA5PAN\/wB8Of0KNzSwBTRLT7u5u0dnZycE0c9tQnZQ5EIYbIgAiAHJ4Bv6APxz+hBqbMxNQFTzq5h0cHCzE5glqvUJ1EoeYrrYIgAiArlwxlQN\/wB8Of0KNzZkJqN6diwyUQ6f8bREAEQA5rAN\/wB8Of0opoKFNKArVSRxiutoiACIAunIFGWh6pNC+kjELNTanzkB10QxpSUuoTkrfXNNboIFDQNOz5oQF+AP+cPgTamx2FlB1HWhTU5MYGBgQLS0tHMxytw3VSXkAhQCIAMjhGfgD\/nD4E2psTiWgjz32mLjuuus4ONXUNlQn5QEaAiACIIdn4A\/4w+FPqLE5lYAuWrRI4DgzDk2Ka4sAiADIYSf4A\/5w+AMBjUEPx5lxaJWfLQIgAiCHbeAP+MPhz2NPPS0+9586xCMPPyBoXk0oV+oMdGxsLPW7FWWiUahfOakB9zBAAEQA9KBNxQT8AX84\/PnMvU+KHfteFz+8\/aL6FFAOOEWxhYDGewIBEAGQ007BH\/CHwx8IKAe9nGwhoBBQX6pBICAQvtwhO\/AnGT0IKIddOdlCQCGgvlRDAISA+nIHAmpHDgJqx6jmJSCgEFBfEkJAIaC+3IGA2pGDgNoxqnkJCCgE1JeEEFAIqC93IKB25C5b\/Zh4\/tW3MInIDlXtSkBAIaC+7IOAQkB9uQMBtSMHAbVjVPMSEFAIqC8JIaAQUF\/uQEDtyEFA7RhVtQRt2NDT05O4Ry8EFALqS0IIKATUlzsQUDtyZ9z+vahQqdaBqhvMt7a2irvvvlvcdtttoq2tTXR2dtpRy6iErMfBgwchoJ6YQiAgEJ7UiczAH\/CHw5\/SCaia8e3YsUOMjIyI\/v5+sW\/fPtHR0SGWL1+em4jKw7xtp8QgA0UG6tvIIRAQCF\/u4APDjlypBPTw4cPixhtvrGSadEaoFNDGxkah\/90On38Judcu1eeuu+5CBuoJJQQCAuFJHWSgDsChfcWDRLNvaQy0NF24ssu0vb09yjJNAjo4OCiGhoZEc3OzA738iqj1mDt3LsZA\/WBEAHTADQEQHxgONIktAv5AQCsIyAx05syZore3d5qAUpfq+Ph41KVLGWm1LlW4H330UQgoA2g0cAgEgz4YA7WAh\/YVD9Aje18X1973ZHkyUHpT6jpdsWKF6OvrE3v37q104Q4MDIh169ZF\/75gwQJOm0y03b17t+jq6hIbNmwQLS0tUX1cZ+Fu2bIl2vG\/mtlx1V68SjdGA4eAcqgF\/oA\/PvyhXsTvH2gQN2\/dUy4BpbdVZ+FK8GwTeXxA1m30MVgp6K4CKu9Hk52WLFmSRZWCv8fRo0fF5ORk9FHR0NAQ\/Ptk\/QLAJxlR4AN8fNrc5s2bxddHD4q3Lrm2fALqA1gWNpR9kvgdOnTIeLvu7m7jDGA5C3ft2rVi1qxZkVggCz0O4ZEjR8TExESUmUNAp9MK+CS3XOADfHxiOyVh13\/9R2L\/W8eH+kq1DtQHsGrZpOnCpRnDIZ16Xi3M1PuiCy4ZZeADfDjtEPwxo6fOwIWAchjGtIWA8gBEA4dAcBgE\/oA\/PvzZ+sRLlfHP0gioaewzDrxqTyaSz4WA+tD3hA0CIAIgh0HgD\/jjwx95jBnZvu3Nl8VTq64MqnfwpGPHjh3zeXE5C1cfb6SlJXIW7rnnnpv7rkRJ74KdiOLRQQBEAPSJA9IG\/AF\/0vJH775tfOSvxaMP\/O\/6F1B9HagOnLoOlJa1qLsUpQU5y\/IQUAioL58gEBAIX+6QHfgzFT0ST1q6smPf69EPlH3O2HZHpBUhzU\/xykD1nYh0YlEWKncieuCBByr\/X+sZrxBQCKhvEEQAhID6cgcCOh25LaMHRNfg05Uf\/vMFB8VDX7m9HAKaNgPNY1s\/F3JDQCGgLjwxlYGAQkB9uQMBnYqcPnHovDNOFn1\/cZJYvHhxOQSU4HAZA73qqquiTeflln8cAmZhCwGFgPryCAIKAfXlDgT0BHIm8dzQfqloePnpcgkoQWKajUv917SJPO2BS+JJV7X3xHUlNgQUAurKFb0cBBQC6ssdCOhx5Hq\/+3Ox+tvPVmCkzJPE80NzTxOhxmavMVAOkWppG6qT8sAMAgGB4PAM\/AF\/4hCgzeK7BvcImjgkLxJPWrIir1BjMwSUEzXqyBYBEAGQQ2fwB\/wxIaCu81TF88HOywWJaGkF1LaZguzKrfXMW9WpoX7lcAKbqy0CIAKgK1dM5cAf8IcQkFmmukRFReaai88QX\/mPF08RT\/o91NjsnYHqaz3pSDM6G1Q\/ZozTKLO2DdVJWeOAAJgeUQgEBCI9a05Y1Dt\/SDifffmIuG3o6SldtWrWKcc7TTiGGpu9BFRfB0ozcu+\/\/\/7KZCFaByoFlUO6rG1DdVLWOEBA0yNa7wEwPSJTLYBPOT8w9A0RdBTUiUJJCIUamzMRUMo6V65cKTZt2hQdD6b\/nds4s7IP1UlZvX\/SfRAAyxkAs+IW+FMO\/pBgPvfyEbHuX\/ZXdhEyvTkJpz7OCQH9PQL6gdaUkS5btkysWbNGtLS0QECziko53gcBsBwBsFqUAn\/qlz8kmn+9bb+g3YOSLhLNT7z7LHHTn82ZNsZp412oyY1XBkpgqNv1UdZJY6Jz586NDrKm34qy\/63quFCdZCNfFr8jANZvAMyCH7Z7gD\/h84eEkkSQ\/ksZ5t\/sTBZMemMqP+f0k8UdH7sgWs\/pe4Uam70FlIBSJxJRVrpo0SIxNjYWbQZMmykUaQYu1TdUJ\/mSMo0dAmD4ATCNv7MuC\/6EyR8Sy58eOCzu\/X8vJHbJqm8nM81PvfdslmjWQ3LDEtCsG2G17wcBjUcYATDMAFjtNuN6f\/CnuPxRM8tnJn4p1v\/r885imWWWmYRQqLEZAuoaIeq8HAJgcQNgCNQDf4rBH3W3n2\/9aFJ8+8cvpxLLvARTR6tUAqpPGtLB0MdHixIAQnVSHvghABYjAObh62o8A\/zJnz8klvTnx+OHBYmlPFszrX+pS\/avLj9X0CYHnHHMtM8tbRcuBJRDlWLaIgDmHwCLyQS\/WoE\/1eOPzCrv+pf90WYFvkIps8uPXnqm+MuWc2omliakQk1unLtw5dKV0dFRpxbW3d0dzcgt0hWqk\/LAEAGwegEwD\/\/V+hngD48\/tOE6ZYL\/\/OOXxf9lZJOyFnJ27K0fOU9cfO4fRv+s7j1ba76gC1dZ91k0Z8TVBwIa7ykEQF4ADKUNVKue4I+dPz\/42QvinHObxYaRF8Wzk7xMUhfK+ZecIa6\/\/NzCC2W9xWbnDLRaDS\/P+0JAIaC+fINA2AXiwIEDYs6cOaKhocEX5qDt5JjkU2OHxHd+8rJ44bXjY5RZXDKjvOWa88Qp\/+Hthep+zeL9Qo3NENAsvF8H94BAQCA4NC4Df2Q36z\/+24T415+9mqlAyi5W2pTgE+85S3z6vWdH4kvCWeSuVw5nVNu6F1Db8WU6kEXcTCFUJ2VF0qT7lCEAcnAEPvX9gSHFcfLwr6It656ZeDNzgZw5oyHKzkkkv\/DRd4k\/PuudwXa5ctqSyTbU2IwMNGsmBHo\/CER9C0S1aVlU\/shuVcrifv3bY2Lz4y+KJ58\/lLk46mOSC\/70THH5nBmVDPKZ8dfFO46+Vuou7iQOQkCr3UIzuH+oTsrg1a23KGoAtFY8pwLAp3gfGKo4HjzyGzHw+Lh4+qVfVlUcCQXKID8893TR\/sHmCii2blbwJ5k\/ocZmZKA5BeCiPwYNvHgCUXTOqPXLmj+qOH7\/318Tg7teih7HWQNpw1OKYNtFpwv60\/quUzMbh8waH9u7hPZ7KQWUzv3s6OgQhw4dqvirqalJDAwMRMeaFe0K1Ul54IgGDgHl8CwNf2i8ka7nXz0ihn4wIX77u2NVyxrVrlX6fxJJ2gSdjt3Kc5JOGnw4fgjVNtTY7J2Bbtu2TaxYsULoGybQNn7r1q0TfX19YsGCBYXyZ6hOygNENHAIKIdnNMY3OTkpfv0Hp4tnJt8SD\/1wMrpdlks54uonl3hccNY7xRUXnBp1r+Ypji64oX2hC7eCgNyVaObMmaK3t3caMuoxZ42NjS78yqUMBDQeZjRwCKiOgLqGce\/km+Lhn7ws9hz4ZVSsml2petZIY44fOH+G+PM\/OSOauVo0cXQJXmhfENAKAnJJS3t7u3G7Pmwm79KkilUGDbw8AqqOL2776SviwR\/+Qhw7lk+2KLtR6b8kjBeefYqgGaszTm6Yst7RNimnWK3HXhu0LwgoMlB7Owm2BBp4uAKq73ZDe6n+ZPxwlKnl0YUqkVPXOf7pHzWKay4+XdB\/qR50yofMHINtJIyKo31BQKcggDFQRmsqoCkaePEEVE62mXPGyWL7nlfED55\/I3dRVDNGygo\/+Z6zxSffM30CDvhTPP4UMMzEVinU4TXvSUSEBGbhhkRRNHCOt7ISCJkt0nrFx587KJ7YfzCqVp6Zot6NOu\/8GYL+vHdW0xSI0nSjZoUPx0dFtgU+yECLzE+nuoX6leP0csxCaOB+HxjqeCJNtKGDjauxDZyLe6XgybHFhZedIxredlJkKg9KrlY3Kvjjxx8Xv5ahTKixmZWBhubYUJ2UB84IgLQucerJGZQVPvDkRCSIdD03eViMv\/GbPNxReYYqipfNaRLvnzMjmpGqXmkyxWpVHvyBgHK4FWps9hJQOQuXlrH09\/eLIi1VSXJiqE7iENPVtp4DoMwSm05+uxh+6hdi18\/fqEm3qfSFKopX\/vFpovWCU8WfnHPKlMk2ajerqw9rWa6e+ZMFrsAHXbgVBOJOZini5gmq2yCg8SQOpYGrXaZv\/uq34sEfTgra6o2uvMcRTYJ4\/pnvFB88f4a45uIzop9DXLPoIxih8Mfn3bKwAT4Q0FgE5IxctUBra2vhslMIaLEEVO8ypcyQFug\/M3F8sX4RBJFGEGmd4kcuahQ\/3T8hzjrrbPHnl5w1RRyzCLCh3wMCgS5cDodDjc1eXbhJQMldisbHx8XQ0JBobj5xYgEH4CxsQ3VSFu9uu0dWAVCKIv339SO\/Ft\/d86rYN3l8DLEIgkj1uHxOk\/jwRaeLd77j7ZXjpmyTa7LCx+aHUH8HPhBQDndDjc2ZCKjc\/1YFUN8j1wYubf83PDwcFXPdkF61kfdPynxDdZINuyx+jwuAapcpCeG3fvSy+FmBMkR6dzo1g\/ZAvaT5D6cs1s9ycg0EAgLBaWfgTzJ6ocZmLwE1jYG6ip4JRn3rP\/r7xo0bE091kZluW1ubcTtB03NCdRKn4eq2arfp744dE5sfG69MqqnFLFOqnzqp5qSThPiLS86MFuu\/9MavoupXewmGC74IgBBQF57ElQF\/IKAVBKSAzps3z7iZfBqimfbVdRFHabdq1SrnU1\/qXUBVcdy25xXxT0\/9InJFHht\/qz5XBZH+nzLDz7zvbDH22lHnLtM0HMqjLAIgBJTDM\/AHAsrhTypb22b1dDPaBWnlypVi06ZNzuOs9SKgJJR7Xvql2PC953MZV1QFkWaZ0hjiRy89M\/KpOnaYZZdpKsLkUBgBEALKoRn4AwHl8CeVrctpLqZxV9vM39AEVGaUt\/zd0+LnrxyZttA\/FahaYVUUaQzx0+8+Vewb+wVmmcaAigAIAeW0N\/AHAsrhj5OtuhzGNgmJJhBt3769Mk4qu33pQXGbO0gB3bJli5g9e7Zz5upU+QwK0S43rx5+S\/z3h57z7nalEzHoou3c3n\/eDLH06pnihVePd53SFZclooFDIDgUBn\/AHx\/+UG8jXWNjY2Lx4sViZGQkis2hXF6TiKr9crYDu+OeLze3X7t2rXFcVAqotO\/o6BBLliyp9usk3v8HL74ltu97U\/zd7uO747hcUiTbLjhFfOTCUyKTD8w6LpC+19GjR8Xk5GT0UdHQcFyEcZ1AAPgkswH4AB+feLF58+YoCZIXBNQHRYMNZaM9PT2JM3F1M9vYqRRQEthZs2ZFYlGLdarUNfvr3\/5OfPgru61oUcZ49YWnif\/6Z38UCZsUT6thygJHjhwRExMT0dcfBHQ6eMAnmVDAB\/ikDDlRcYrZ9Gfnzp1i\/fr1yEB9QDTZVFNAa\/WVQ8L5D\/82IVZ\/+9lYmEgwb7hypph3\/qmV5RtZYZp0H3TBJaMMfIAPpx2CP8nohTY\/Rb6NVxcufTEsW7ZMrFmzRrS0tExDxmUSkDSK63ZNukdcFy\/dq6urS2zYsMFYr1o6aegHE2LF3\/7UyCISzeGbLhNvO+mk2DFKTuN1sUUDh0C48CSuDPgD\/nD4U8vYzKl3zQWUKk8Tgnbt2lXZ+k+K6vLly2M3SZATjuQG9i5rU2vhpEf2vi66BvdMm0FLovmJd58lvnTdRRz\/ZWaLAIgAyCET+AP+cPhTi9jMqW\/qDFRmfaOjo07Ptc2i1W+ib8unn+xCv9PV29tbMdU3sV+4cGHixg55Oom6a\/\/xyQnxv741tbuWhHPzDe8Rp5\/yjpplmyYHIgAiADo17JhC4A\/4w+FPnrGZU0\/dtioZaJYVzPJeeTmJxPPa+56clnUuuXKm+Pz88wslnBJfBEAEQE5bA3\/AHw5\/8orNnDqabL0ENOtK5HW\/PJxE4nnZ6semvBJlnQ92Xl5I4YSAurEPAgGBcGOKuRT4k4xeHrGZ4784WwhohqiGKp4EARo4BILTFMAf8IfDn7oXUDlJZ+bMmdHs2xtuuCHaPSLuovWEZToP1CSeW5a9V3z83ccPXy76hQCIAMjhKPgD\/nD4U\/cCygGnKLbVcpJ1EG0NAAAaAklEQVRJPKnLVh7DVZT3T6oHAiACIIen4A\/4w+FPtWIzp04utujCdUEpoYxpwlBo4okuXDsJIBAQCDtL4kuAP8noQUA57MrJthpO+uIDz4j+R16svEHvZy8WN1w1M6c3yu4xaOAQCA6bwB\/wh8OfasRmTn1cbb0yUDkeWvYxUL3rlvasfejmy12xL1Q5BEAEQA4hwR\/wh8OfUgloElC0ucHq1asLN4GI6py1k864\/XsVKEJYqpLkNwRABEBOAAR\/wB8Of7KOzZy6pLH1ykBtDzDtGmSzyeP3LJ30mXufrJzZGbp4EvYIgAiAnDYI\/oA\/HP5kGZs59UhrWxUBTbOZfNoKc8pn5STa35Z2GpLXpo53i+suO4dTtZrbIgAiAHJICP6APxz+ZBWbOXXwsYWAeqBGOw3R+CddlH0+tepKj7sUywQBEAGQw0jwB\/zh8AcC+nv0XE5F4QDNsc3CSXr2ee\/nLhWf+2Azp1qFsEUARADkEBH8AX84\/MkiNnOe72vrlYHaZuG2traK\/v5+0djY6FuvqthxnVRPs251gBEAEQA5jQ78AX84\/OHGZs6zObZeAiofSGOdIyMjFbGkGbg9PT1iYGDAeKA1p6JZ2HKdpGefIW6YEIcjAiACIKeNgT\/gD4c\/3NjMeTbH1ltASTw3btw4TSz1g645lcvaluOkes4+CWcEQARATnsDf8AfDn84sZnzXK6tl4DKw7VpY3n1gGtZGVrGMj4+XrhuXI6Ttj7xkrh5654K3vWUfUJA7c0IAgGBsLMkvgT4k4weJzZz\/MK19RJQOQba3t4uOjs7p9WhHpexqOs+Q95xCF24fk0GARAC6sec41bgDwS0gkDZBFQf+6RlK7R8pZ4uNHAIBIfP4A\/4w+FPqTJQAiqum9bWvcsBmWvr6yR916F6WPepY4kAiADIaV\/gD\/jD4Y9vbOY8Mwtbry5cevDu3btFR0eHmD9\/\/pRx0LjJRVlUlnsPXyepe97e8bF3iTs+dgG3KoWzRwBEAOSQEvwBfzj88Y3NnGdmYestoPRwmW2Ojo5W6jJ79uxCbiRPFfRxkjr7lrptN7RfGtRB2a4kQQBEAHTliqkc+AP+cPjjE5s5z8vKliWgWVUir\/v4OKneJw9J7BEAEQA57RD8AX84\/PGJzZznZWXrJaAy81y6dKlYsGBBVnWp+n18nKR239bb0hUVcARABEBOAwR\/wB8Of3xiM+d5Wdl6CahtFm5Wlcv6Pmmd9OXvPCe+\/J39UTXqZdP4OEwRABEAOe0N\/AF\/OPxJG5s5z8rS1ktAZQba1tZmXAeaZQWzvFdaJ6ndt\/MvOUP8\/X9pybI6hboXAiACIIeQ4A\/4w+FP2tjMeVaWtl4CShWgWbhdXV1iw4YNhdz31gRSGifpW\/fVc\/ctYYUAiADICSzgD\/jD4U+a2Mx5Tta2XgJqO42FKlnE2bhpnFSm7lsIqL1ZQSAgEHaWxJcAf5LRSxObOX7I2tZLQLOuRF73S+Okssy+ldijgUMgOO0Q\/AF\/OPxJE5s5z8naFgJqQLRs3bfIQO3NCgIBgbCzBBmoL0YQUF\/kcrRzdVIZ9r7VYYdAQCA4TRH8AX84\/HGNzZxnVMPWOQOV4550hNmaNWvEDTfcIMbGxmLrFPIYaBn2voWApmtOEAgIRDrGTC0N\/iSjV\/cCyiFPUWxdnVSGvW8hoOlYiQAIAU3HGAhoGrxcY3Oae+ZR1jkDNVWGNo4fGRmpHJy9bds20dPTIwYGBgq5tMXFSWUc\/yTfQiAgEJyAA\/6APxz+uMRmzv2rZestoHGnrpCIrlixQvT19RVumz8XJ2194iVx89Y9Ed71vvuQSioEQARATpABf8AfDn9cYjPn\/tWy9RJQ25mfcWeFVuslXO\/r4qSyLV+R2CEAIgC6tiNTOfAH\/OHwxyU2c+5fLVsvAbXthUvZ6eDgYOGONbM5Se++rdezPxEA0zcnCAQEIj1rTliAP8no2WIzB\/tq2kJANXTLcvqKTio0cAgEJ9CAP+APhz+lElACKq6b1ta9ywGZa2tzUtm271PxRABEAOS0L\/AH\/OHwxxabOfeupq1XBkoVos3kOzo6xPz580Vvb2+ljnGTi6rxEnLCkrx3d3d34ukwNidde9+TgjZRoOvqC08TD918eTWqXch7IgAiAHKICf6APxz+2GIz597VtPUWUKqUzDZHR0crdcxrAwUp4GvXro1m+8q\/L1++PFZEbU66bPVjgsZB6SrT+Ce9LwIgAiAn0IA\/4A+HP7bYzLl3NW1ZAlrNitnubepC1tel6vdIclJZ139KjBAAEQBtbS7pd\/AH\/OHwBwLKQS8jW9vymTQC+tSqK6N1oGW5EAARADlcB3\/AHw5\/IKAc9DKw1bt0TbdMclKZJxChC9dOQAgEBMLOkvgS4E8yehBQDrsYturh3q2trZVtBdMKaFk3UEAXrhv5EAAhoG5MMZcCfyCgHP7kYmvbwEF+5WzZskXQZKfm5uZKvc754vcr\/\/+F+bPFHR+7IJc6F+UhaOAQCA4XwR\/wx4c\/lADRRSd7LV68ONpbnWJzKFewk4hMANt2SJICKm1pGc6SJUvE+Bu\/EZ\/ZfOJotq9f3yw+MKs845+Ex9GjR8Xk5GT0UdHQ0BAKf3OrJ\/BJhhr4AB+fxrh58+bo8BF5QUB9UMzIxlVAaenLrFmzIrGgP4\/sfU0suv\/pSi3G11yZUY3Cuc2RI0fExMRE9PUHAZ3uN+CTzGXgA3x8oh3FbPqzc+dOsX79+vJkoKY1oCqA1VwPGrfbkW0iUdxAddknEJHf0AWHLjifAChtwB\/wh8Of0k0ioiUju3btqtmG8fqxaVJUyYn9\/f2isbFxmj\/jnFT2CUQQUHvTh0BAIOwsiS8B\/iSjVyoBtXWVcoiWxlbfym\/hwoVTthXU7xXnJHUHogc7LxcfmntammrURVk0cAgEh8jgD\/jD4Q8ElINeTrZxTirrCSwq7AiACICcZgj+gD8c\/pRKQIt84kqSE01Oos3jaRN5eb3aew2HB8HaIgAiAHLIC\/6APxz+lEpACSh9DJIDXl62JidhAtFx9BEAEQA57RD8AX84\/CmVgKq7\/8SBVs1ZuL6OMjnp5q17xNYnji\/mLdsRZujCdWcSBAIC4c6W6SXBn2T0SiWgHCLV0tbkJHUGbtmOMIOAurMRARAC6s4WCGharCCgaRGrQXmTkzCBCF24LlSEgEJAXXgSVwb8QQbK4U8hbHUBLfsZoMhA3WmJAAgBdWcLMtC0WJUyA9VPQrn77rvFbbfdJtra2kRnZ2daDKteXneSPgO3bGeAQkDdKQcBhYC6swUCmhar0gkozcLt6emJNgLesWNHtIch7QC0b98+QZu0L1++vHAiqjsJM3BP0BwCAYFIG\/TwAeaOGNoXunArCMh1oDLTpGPEpIDSFnr6391pVt2SuoCu\/vazove7P48eWuYZuPT+aOAQUE7rA3\/AHw5\/SpWB6lv5mQR0cHCwZvvkxjlSd5I6A\/dzH2wW937uUg4HgrZFAEQA5BAY\/AF\/OPwplYDqOxHpAkobzY+Pj8du6s4BmmOrO0ndA7fMS1iQgdpZBYGAQNhZEl8C\/EEX7hQE1J2I9u7dW+nCpTHRdevWib6+PrFgwQIO5zK3VQX0d6ecJUhA5VXmCUQQUDvVEAAhoHaWQEB9MSpVBipBMu1I1NTUFE0samlp8cWyanaqk\/a\/1ThlD9yynsIiwYZAQCA4DQ\/8AX84\/CmlgHIAq4VtkoCWdRN5CKgbEyEQEAg3pphLgT\/owuXwpxC2qoD+7U9+Lb78nf1Rvc4742RBXbhlvtDAIRAc\/oM\/4A+HP8hAOejlZKs6qfufXxXb9rwSPbnsS1gIAwRABEBOMwR\/wB8Of0opoKYx0NbW1sLNvpWOVZ100z9Nih37Xo9+KvsMXAiovelDICAQdpbElwB\/0IU7BQF1JyJ1whAtadm4cWMhJxKpAvq+3n+vvA8EFBmoLTgiAEJAbRxJ+h38gYBWEJDrQJcuXWpcqlL0daB\/M7xNfPobL1Tep+wzcJGB2kMjAiAE1M4SZKC+GJWqC5e6bpctWybWrFljXK5CWWiRdyL60v95UFAXrrwgoMhAbQ0fAgoBtXEEGag\/QqUSUH0vXB22omeguoCWfQkLMlB7w4eAQkDtLEEG6otRqQSUQFJ3IlJ3HKLss+g7Ed1015D40vcPRb7GEpbjlIdAQCB8gx\/4Y0cO7SsZo1IJqGn2bRI8s2fPLsTG8tJJ773p6+L7B94eVRlLWCCg9vCHDwwbRhAIfIDZOJL0e6kElANULW2lk05rXy9oKz8I6AlvIAAiAHLaJvgD\/nD4AwHloJeTrXTSGwu+LGgzebroCDM6yqzsFwIgAiCnDYA\/4A+HP6UU0N27d4uOjg5x6NDx8US6QthM\/vWFmyr1hYCiC9el4UMgIBAuPIkrA\/4ko1c6AZWTiLq7u0VnZ2cFnRAmEakCiiUsEFCXwIgACAF14QkE1A+lUgmofqC2DlmRl7EsuvV\/isMf+mKlymU\/B1QCAYGAQPiFPnyAueCG9oUMtIKAnIXb3t4+JfuUBYq8kYIuoFgDigCIAOiCAD4wOChBQCGgFQRCzkA\/+9\/uFW++f2n0LlgDeoLUaOAQCAgEBwHwh4NeqbpwCahQx0AXrvuu+NV5V0e+xhpQCKhro8cHBgTClSumcuAPMtBpCIQ4C\/fa+54SvznrYgio5k00cAgEBIKDAPjDQa90GSgHrFrZkpNUAcUxZshAXbmIDwwIhCtXkIGmRwoCmh6z3C3ISZ8cOlJ5LgQUAupKQggoBNSVKxDQ9EhBQNNjlruFLqBYAwoBdSUhBBQC6soVCGh6pCCg6THL3QICGg85BAICwWmQ4A\/4w+EPBJSDXk62uoBiEwVkoK7Ug0BAIFy5ggw0PVIQ0PSY5W6hCijWgE6FHwIBgeA0SPAH\/OHwBwLKQE9uzDA6Olq5S19fn1AP6jbdnrYMHB4envJTa2ur6O\/vF42Nx48rUy8IKLpwfWkKgYBA+HKH7MCfZPQgoJ7skuJJ5lL45CYNSSIq7dra2ozbCZqqowooNlFABpqGsgiAENA0fNHLgj8QUA5\/Ym1pM4auri6xYcMG0dLSUiln25Be7se7atUqa6YqbwoBRQbqS2IEQAioL3eQgdqRQwZqxyhVCdqQfmRkJLY7loR35cqVYtOmTaK52e1AbFVAsQYUGWgaQkJAIaBp+IIMNB1aENB0eCWWtm1WT8by3FH1Rknjn1QOAooM1JemEFAIqC93kIHakYOA2jFyLuEyBkpdvNu3bxcDAwNR169pLFV\/oCqg2EQBGagzITEJxAoVPjDwgWElSUIBCCgHPcVWblA\/f\/580dvbm+qu0nbt2rXGcVFVQL\/2l2eLv7r6klT3r+fCCIAIgBx+gz\/gjw9\/aC4LXWNjY2Lx4sXRsN3s2bN9blUTm5OOHTt2rCZPNjyUI550O9tB36qAzth2h7jhs58US5YsKcrr17QeR48eFZOTk9F4ckNDQ03rUsSHA59krwAf4OPTbjdv3hz1IsoLAuqDohCCK55pBfTbi94ZiYXrBCTP1wrGbP\/+\/YLIvGzZsqC+APMCGPgkIw18gI9PW6Skh\/7s3LlTrF+\/HhmoD4hpxTNuklHckhhZJ5mBYhei6V4KdQzCh28+NsAnGTXgA3x82pUam9GF64Gg7HadN29eqjFPfaKRy30goPEOQgBEAPRovhUT8Af8KSN\/aj4GalqOIh1Bg8lDQ0NRNyvNuqVLnVgkRVSWX7hwYaIISwF918mHBc3CxXUCATmIv2XLFnThGogBfJJbC\/ABPpx4iklEHPRysiUnfeF\/fFk88b1v5fREPAYIAAEgAARcELjiiivE1q1bXYoWpkzNM9C8kSARpT+4gAAQAAJAoDgIUI9jSEtYCLnSCWhx6IKaAAEgAASAQMgIQEBD9h7qDgSAABAAAjVDAAJaM+jxYCAABIAAEAgZAQhoyN5D3YEAEAACQKBmCEBAawY9HgwEgAAQAAIhI1AKAZU7F42Ojka+sh17FrJDk+qur5vt6+uzHkaur9O1rbUNGTsffNT3Jfuenp7KCUEhY2Gquw8+coMTOfO9ntueDz5yF7ZDhw5FkNdz+3JpD7Tef+7cuaKzs9OleM3L1L2A6tv+uZw1WnOvVKECenB3CfYknhs3bqwIgstuT1Woei639MFHrZjE5uDBg3UpoD746HxxOXIwF2dX4SEcfNrb2yPBqOf25QK5\/Fjv7u6GgLoAlkcZk1DY9szNo155PiPuo8G0u5Osl7Rpa2ubQmYX4c3z3bJ4lg8++nMJy+HhYdHU1FR3AuqLDwVEOl2jv79fNDY2RpDVY9vj4DM4OFjZbY3wofa1evXqKf+WBceLfA+9hxACWiBvmRpxnDgUqNqZVkV+2a5atWpKl61PY63HAMjFR+J44403irvuuqvuBNQHnzK1MR98qIFTbCq7gEqejI+Pi3vuuUfccsstQmbkmQbBKt2s7rtwTVlW2bpx40TPJ5s0NfoqcTO323LwUc+gpbGbehwD9cFHFZWHH344ys7pqscxUB98CAv9\/OKyd+HaznPOLSCkeBAEtLc3BVxhFvVt4Prb1msD5+Cj9nA8+uijENDfk0adHCMnq9XrGCiHPyF3X2YdDSGgWSOawf2QgcaPO6XJQNWuFnlCTgbuKcQtfAOgbpcGz0K8uGMlfPCRArp8+fIpY+jy39euXWudAe5YvZoX88FHjneuWLFC1PsHhquDIKCuSOVYDgLKF9B6Fk+iok8ANI3xQUBPNOw4oQwxSNrCFYc\/M2fOnHIEYz1+YNjwk7+HyI2678LFJKITYy0+k4jqXTzVsag0+Ojr9\/QgEdJMQluA85kkE2cTYpCsBj5xk6zqER8bfhBQV4RqUA7LWITwnWZfBvEkSvrio9O5XjNQX3xMvT\/1mGH54BNnE\/fhUYPQmfsjQ\/x4qPsMVJ+4QKyg5QZ610nubMn5gXKXFDne4hLsKQDu2rWrFGvSfPApi4Caxutc+KOLpQyQ1PbUtaE5N4WqPM6HP7pNvU6ycgUcAuqKVM7lsJXfccBtW42pGYOti9JlG8Cc3cx+XBp8TA9zERV2JWt4Ax989K386nmrOh989HZWj8t8XCkLAXVFCuWAABAAAkAACASOQN134QbuH1QfCAABIAAECooABLSgjkG1gAAQAAJAoNgIQECL7R\/UDggAASAABAqKAAS0oI5BtYAAEAACQKDYCEBAi+0f1A4IAAEgAAQKigAEtKCOQbWAABAAAkCg2AhAQIvtH9QOCAABIAAECooABLSgjkG1gAAQAAJAoNgIQECL7R\/U7vcI0G5Sn\/\/850VXV5doaWlJhQvtEEOHOvfmeParvvuV785N9XCAubrbTlNTkxgYGEjtw1QONxRWd0SqVR247wD74iEAAS2eT1AjAwK+2+TFbdpdbZB966vXq54EtAhngGbll2rzB\/cPAwEIaBh+Kn0tfQNfrQQ0K+HL6j61JFCRTmDx5VEt8cOzi4sABLS4vilVzfSNuNVuNhKRdevWVfBQNyQ3bXovz+LUNzLXu+6S7psEftJ99a5buk\/SBuF6eVMdBwcHoxOE7rzzzkq1TOeN6u+j30vW+1Of+pTYsmWLOHTokJBYmjZ9p4eNj49POTnFhqkJN5OAyg8D\/b1sXd1xH0T6ySZx\/oOAliqsVP1lIaBVhxgPsCFgCn565hV3rmtHR4dQuwbjjojSj6+jk2e2b99eGY+TwjBv3rzEsVIpBvPnz6+Uk8KlBn+XzNF0vBfZbdy4sVIveW9VhF3wIsz14+jk8w4ePDhlHNL0TmQ7PDw8Rfzj3l2tbxoBpY+i2bNnV47LcxFBCKitNeH3PBGAgOaJNp5lRMBFbEwCSkFez5D0AGsKuFIIli9fLjo7Oyt1cgngSc+kG8lzLl3eyVRG1retrS2qmy6o9Az92Ke4Y6DizuPUPxJM576axN1UzuUMy7gM1CS8JnxV0kBAEUSKhAAEtEjeKGld1G5YU9ckwWLretO7gGXXpCngmkTJJEy6O5LGU3UxdBFQm1jQ8033sWXLMnuU9ZeZsUlo495JF0b6+6JFi4QpQ7e9a1IX7tDQkGhubq5AHecbWQACWtIgUdDXhoAW1DFlq5Zp7FAV06QuXBrLo4vKU5cujavJLts4AVXHVHWs40Q8SbjSCqhL5pZGQNXxTzn2SfZqF7dJQJPeSRV4KaBjY2NGaqpdsXqBOAEdGRmZMr7q8qEEAS1bZCj2+0JAi+2f0tZOZlFSzHQBjRMgly5cW8YUB3qWGWiWAhrXJR3Xhdve3l7puk6bgaq2ruREBuqKFMqFhgAENDSPlaS+emakC2hc5qRPdEkaA9XXJbost3AdB3QR6bgyauZHmw7QLFy1q9OEzYoVK4Q+g1Wf3BQ3VppmDFQfcyY62rqi8xgDNU3kMjUV21BASZoXXjMjBCCgGQGJ2\/gjYBr30if06EFYCiMFdCkuajewutTFFOD1WbiuGWG1Z+Hq7+0yBmqqkzqunDQGSl5LmoVrWjKkzkB2mXgVJ6DUja7OLnYVQZ0vpneNYyME1L+dwnI6AhBQsKIQCOhrGKlSakaliqMMulSGxjtHR0cr70A2tG3frl27pi2P0O+Z1TpQ0\/ifSwZK9XFdB5qUgaoiKMeDqU733HOPuOWWWyoTf+IyULKPWwdKv6lbIGa9DvSaa64R3\/zmNyP\/mbbYi+ueVidK0buuWrVK9PT0TFnSZMqsIaCFaO51UwkIaN24Ei8CBLJDQF9Ow7lzmjFQznNcbCGgLiihjCsCEFBXpFAOCNQpAqZMzbacJA0UENA0aKFsSAhAQEPyFuoKBKqAgGkJUdKylLRVMJ3GsmPHjmmTo9LeN015nMaSBi2UdUUAAuqKFMoBASAABIAAEFAQ+P+GGorK3KLS5gAAAABJRU5ErkJggg==","height":223,"width":371}}
%---
