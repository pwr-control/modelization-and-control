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
l1 = Kd(2) %[output:59afecfa]
l2 = Kd(1) %[output:6273c9b1]
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
Rbrake = 4;
CFi = 900e-6*8;
%[text] ### DClink Lstray model
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:82a1ee3d]

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
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:8da9fcc7]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:9d901226]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:2b6324c3]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:1d0e7351]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid_emu_data.w_grid;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:5ad4d344]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:6fac1d01]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:1ec31383]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:8e6905b6]
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
run('n_sys_generic_1M5W_pmsm'); %[output:9565d738] %[output:2efd1d55]
run('n_sys_generic_1M5W_torque_curve');

% n_sys = 1;
% run('testroom_eq_psm_690V');
% run('testroom_torque_curve_690V');

torque_overload_factor = 1;

b = tau_bez/omega_m_bez;
external_motor_inertia = 5*Jm;
% external_motor_inertia = 1;

%% inverter filter
% LFi = 40e-6;
% RLFi = 5e-3
%% inverter filter
LFi = 230e-6;
LFi_0 = 20e-6;
RLFi = 5e-3;

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
kg = Kobs(1) %[output:2e86d8c0]
kw = Kobs(2) %[output:8374d010]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:5e758c7f]
luenberger_l2 = Klo(2) %[output:154790c7]
luenberger_l3 = Klo(3) %[output:0ed06d2a]
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
lithium_ion_battery = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_inv); %[output:7c8034af]
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
%[output:59afecfa]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:6273c9b1]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:82a1ee3d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:8da9fcc7]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:9d901226]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:2b6324c3]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:1d0e7351]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:5ad4d344]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:6fac1d01]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:1ec31383]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:8e6905b6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:9565d738]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:2efd1d55]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:2e86d8c0]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:8374d010]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:5e758c7f]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:154790c7]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714302e+02"}}
%---
%[output:0ed06d2a]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.994503273143434e+02"}}
%---
%[output:7c8034af]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAb0AAAEMCAYAAABQuGiUAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQ+QVdWd5k\/D7tBmcLDFqJ0Wm2gTTUxaNKbaHoxjWEJ2suCOO1PdzdSEwQ5DJpitDSBNYyyLKWm6CVhr1BgG2y6yMzRUJusKu7VhnJ6ESJDRiLYzidFGaEnbEoEGzWqb3UzY+t32PM877\/753ffuve+ee79XZQm83zv33O985\/vu79zz55yzZ8+eJXyAABAAAkAACOQAgXNgejloZdwiEAACQAAIOAjA9EAEIAAEgAAQyA0CML3cNDVuFAgAASAABGB64ECiCIyPj1NHR4dzzb6+PqqtrY3l+rt27aKuri7auHEjtba2Otfgf+OP+nuUF3a7r4mJCdqwYQMtWbKEGhsbo7ycZ1nDw8O0dOlS+upXvyq6z3LqePDgQdq3bx91dnbGck8Ky6GhIaf8HTt2UHNzs\/habm0v\/nFMgYzz2rVrqa6uLjbcYqp65oqF6WWuSXFDbgjELYSm6dXU1Dgi9+yzz1J\/f39iptfb20tsSpIHCiXEYerIZS9evJiWL18em3ira+gPLGFYHXdbh6mLHsv1euCBBxLlQ7l1zfLvYHoZaF0Wuq1btzp3wk+SusgqAVi5ciUNDg4SPz2bMeaTtS5oKnO4\/vrradq0ac5TN3+CBEld16yTaQ6nTp1yMhOVCXEGocqWlsHZoimUuvBxHTjrU5+FCxdST08PsTHxR4n\/sWPHCmbhZggKi7GxMed3Ok76fT344IO0adMm2rNnT+GafE+LFi1yjND8dz3zVG3JbbR582biv8+cObNQX7MOejuo7\/j+VBZmtq1q+\/r6es+66LjzDSi8mDtseOrT1NTk4MUfzt5VZhZkiApbhYMqh9vRvLb+nd5V\/Tirt\/3IyIjTN0zOK76Y98J1UDianJw\/f37hPhmTW2+9lb70pS+VjCYorpnXdGufDMiPdbcA07OuyYorrBue\/o0aEjJFJEiw1PdKTE2RVd+bHdp8otVNRje+Cy+8sGh4U5meMhJV7qFDh4qMyq+MSk2Py1Y4Kdx0s2eDHB0ddcxZ1dM0UBZyNWzrZXpKgHWsdBzdBP\/EiRPEDxx+dTDbWrWdaS5623vV8fLLLy8yNp0P5ndsSN\/4xjfozjvvLBieyR+ze3nVyavd3UzPNDx1DWW2XpxX5u3Vlur3Jue5bg8\/\/DA98sgjRQ8sN910Ez355JOuD2luZprU0L7lkhZ79WF6sUMc3wVU57zooosKGYp6glUdfPfu3Y55qL9zbVS2obI2fno3hVJlPcqU+HecQeoZgtu7FtWxWaxVxqk\/eaunZS6PswSzfH66DltGkOlxJhU05GU+hZvx6uHCzVAYh9mzZxeZuWR4U5Wp\/56zJdPEzLZU3yucVCb4zW9+08lq1PduGazORsnwpjmc6fV3L\/6Y72xNfjJOCmvTtLxGE8x400yeeOKJIs7rDyRuw75eDziK88xJVW9lwqp9OVvVs3h9tMBtmJbbnH+T5JB3fApkZ8kwPTvbzam1m5CbQqcEQDcovTNyOWZWpmdV\/GfOcFS2oYuUm+mZAqKGEBXMXsObevlhy4jC9HTcVBakBIzr7jb5RsfRNHM\/0zMzEcaRM2ATZy9TMynLQqzqbL6f8xqq5Pr5mZ7XUK5pel5ZlddIgG70anKKeZ\/qQc3L9NzKcBtpCDJiM2M0M0E3zut1cmt\/NcSr10cf7g2qu8VyZE3VYXrWNFVpRd2eJL1Mz6uzepke\/7uXGJtDgXrNwhqWyvQqNT3zASDo727Nrn5z9913O1moejfmlTGFNT1T8PS\/V2J6+vCb16QUt\/e+KmvXfyPN7IKGEhV\/zFmXbtxJ2vRMzqnhTnMYOSrT098hw\/SqL7gwveq3Qdk1iGN406yMm4n5mZ7+9KwyQV1Ily1b5vpOTxcYaRlqCFUfcjUnwXj93Q10M7vRM9lKhzfNd5n68Fi5w5thhyrN0QE1sUY3PVOUzaHEoOHNIDJHObxpDtmr+1Dvg70yPTX6ob4362SaILdVOcObbljA9IIYEv\/3ML34MY71CnFNZJEM9Xitn3Ib8lLDXV4TWXTT08VZB89v5qGKCzI9jvOaEcjfKTzNGK8JPQon872Rbmpc7he\/+EVnsofb8JfXpCOug2Qii8q6TEH1mvDhhSOXwx81E9htiE6f9cjl3H\/\/\/XTvvfeW3Jc5Q1aVFTSRhd+fBb1\/lU5kCTI9s2P6cd6t3pKJLGbGi3d6scqhqHCYngimdAdFvWRBF\/ywmZ5Cyu29FQ91Sd7pBZXB3+smxPXlDPKOO+4omUmnhE8XSj\/T81uHJl2yoCZL6AbBhnLzzTcXZkaywN5+++20YsWKwjCqabq8ZGH16tW+SxZ0c3Eb7nYzCLf3u3xtrqPKxNXSloceeogeffRRUu83dTM3H2SUofvhy9fxW7JgZqNeGwl4vY\/T3zl7mZ75QMJ48FIZNcGE62C+X+V\/06+pt6e5AYL+jlz\/zhzGNd93p1tlslO7VJsei8yaNWucNU9uO1qYa238ptFnp8nC3UnQU3O40hAdFwK6EZjLRcws2KsOSlT54SKu3VLiuv+0l6seeLiebrOSJbv8YJ1eOlo5taYnmU7Ngs5rn9DBvckE00tHR5PUwmuoOmgjAL3sMDuySOqEmEkEgoaKJdvMcV\/EjizVZ1RqTY+zOO7A\/PHK9Pj7hoYG0R6D1Ye6OjWA6VUH93Ku6vbeKGh3E\/M6KpvgodEw+1WWU9+8\/cbtva50X1DsvZketqTS9Pipav369dTe3u4Yn5vpKRK1tLTA9NLDJ9QECAABIJBqBFJpepyd8Oe6667zfKfnNtwQZhgo1a2CygEBIAAEgEAsCKTO9HgIYfv27XTXXXc5+x16TWQxh3EwrBMLP1AoEAACQCBTCKTO9Hg4k6d28\/uIoNmbZkuod4BuE1s+8pGPZKrhcDNAAAgAgbQgwCe4zJo1Ky3V8a1HqkzPa4YU34HkhbHfxBY2vSNHjljRKElUEniUogxMgImk74EndvMkVaZnQumX6anZnfpCYF7I67V7OYhajC7wsLvjSsQ5ihjwBDyR8Mgmnlhlemx0AwMDhQM1zcXpftmgTY0iIVmlMcADYibhEHgCnmSNJ6k2PQnY0hh03mKkjh49as0YvLSNK40DJqUIAhNgIulXNukrTE\/SohmMgZhBzCS0Bk\/AEwlPYHoSlBKOsalRkoAGYgYxk\/AMPAFPJDyxSV+R6UlaNIMxEDOImYTW4Al4IuEJTE+CUsIxNjVKEtBAzCBmEp6BJ+CJhCc26SsyPUmLZjAGYgYxk9AaPAFPJDyB6UlQSjjGpkZJAhqIGcRMwjPwBDyR8MQmfUWmJ2nRDMZAzCBmElqDJ+CJhCcwPQlKCcfY1ChJQAMxg5hJeAaegCcSntikr8j0JC2awRiIGcRMQmvwBDyR8ASmJ0Ep4RibGiUJaCBmEDMJz8AT8ETCE5v0FZmepEUzGAMxg5hJaA2egCcSnsD0JCglHGNToyQBDcQMYibhGXgCnkh4YpO+ItOTtGgGYyBmEDMJrcET8ETCE5ieBKWEY2xqlCSggZhBzCQ8A0\/AEwlPbNJXZHqSFs1gDMQMYiahNXgCnkh4AtOToJRwjE2NkgQ0EDOImYRn4Al4IuGJTfqKTE\/SohmMgZhBzCS0Bk\/AEwlPYHoSlBKOsalRkoAGYgYxk\/AMPAFPJDyxSV+R6UlaNIMxEDOImYTW4Al4IuEJTE+CUsIxNjVKEtBAzCBmEp6BJ+CJhCc26SsyPUmLZjAGYgYxk9AaPAFPJDyB6UlQSjjGpkZJAhqIGcRMwjPwBDyR8MQmfUWmJ2nRDMZAzCBmElqDJ+CJhCcwPQlKCcfY1ChJQAMxg5hJeAaegCcSntikr8j0JC2awRiIGcRMQmvwBDyR8ASmJ0Ep4RibGiUJaCBmEDMJz8AT8ETCE5v0FZmepEUzGAMxg5hJaA2egCcSnsD0JCglHGNToyQBDcQMYibhGXgCnkh4YpO+ItOTtGgGYyBmEDMJrcET8ETCE5ieBKWEY2xqlCSggZhBzCQ8A0\/AEwlPbNJXZHqSFs1gDMQMYiahNXgCnkh4AtOToJRwjE2NkgQ0EDOImYRn4Al4EsST\/YfP0G1\/tZPe+M6Xg0JT8X1kmd74+Dh1dHTQ0NCQ+MaamproscceE8dXEgjTK0YPYgYxk\/Qn8AQ8CeLJwDPHacXAizR+3y1Boan4PlLTW7VqFa1bt44aGxsDb254eJi6u7upv78\/MDaKAJgeTC+IRxB4CHwQR\/h78KQYpdyanoQs1YyB6cH0gvgHMYPpBXEEpleKUG5NTw1vMiR9fX1UW1sr4U9iMTA9mF4Q2WB6ML0gjsD0ShHq3TtCvXuP5m94k6GYmJigtWvX0p49exxkduzYQc3NzRIexR4D04PpBZEMpgfTC+IITA+m58mRgwcP0uLFi53vFy5cSD09PVRTUyPhVCwxMD2YXhCxYHowvSCOwPRgeoEc0bO\/uro6Z+KKZKJLYMEhA2B6ML0gysD0YHpBHIHpwfQkHCnEqBmbW7ZsSfydH0wPphdEVpgeTC+IIzC9UoR4uQJPZsndkgUvsiDTk3Sj5GMg8BB4CevAE\/AkiCcwvfcQSvqdXm9vr3Plzs5O1zZCpodML6jzQuAh8EEcQaaHTK8IAXP25saNG6m1tVXCo4pilMEuX74cpidEEgIPgZdQBTwBT4J4suih52j\/K2fyN7yp1umdOHEi0ckqbLQbNmygn\/70p87yCGR6QRSd\/B5iBjGTMAU8AU+CeJJb0wsCJq7vd+3a5RQ9MjKC4c0QIEPMIGYSuoAn4EkQT6699yk6Nv5uPjO9pPfe5Oxy\/fr1dM8999C2bdtgekHs1L6HmEHMJHQBT8CTIJ7UrvyBE5K72ZvVOGWBJ6\/cfPPNzrCmZCKLarzBwcGgdsz896Ojo1RfX5\/5+wxzg8CkFC1gAky8+tC8efPot+fNoLc+NzmJMHemF0ZcoojlNX\/bt2+nu+66y9npRWJ6R44cieLSmSgDT\/B4gpcQGTwBT\/x4wsOaPLwJ05P0pgpj+F1eV1dXSSm85dn9999f8u9YslAMCcQMYibpguAJeOLHE7XZNExP0psijkGmFw5QiBnETMIY8AQ88eOJmrl57jsn6eS3\/0RCqarHRHaIbLXvBKYXrgUgZhAzCWPAE\/DEiyf60OaUky\/RG9\/5soRSVY\/JjOkFIYnhTQxvBnEEAg+BD+IIfw+eTKKkDo\/lP5936FEa\/eF\/k8BX9ZjYTE9\/58bn6jkgDQxU7YghmB5ML6i3QcxgekEcgelNIsRZ3qJvPef8f2btVHrr0T8lWyYKxmJ6PNQ4NjbmrJ\/jdXTt7e2iZQUSwpUbA9OD6QVxB6YH0wviCExvEqGh0V\/RLff9xPnzQ+1X012tLfk1PV6vpxap8zowPkldmR6OFpJ0qWRiIPAQeAnTwBPwxERg\/+EzTpbHH87ynv\/6jWRTUhF5pudnerwxNGeBfX19OE9PojgxxkDMIGYSeoEn4ImOgD55hf9991fm0Nwrp+fb9BgIfp934MCBouHN2bNnU0dHB7W1tSVy8oJJVZueRCRiVGkMxAxiJuEQeAKeKAT093j8b8+ua6ZZM2qcr23S18gzPQWQfp6e+rekjhpy68w2NYpEjCqNgZhBzCQcAk\/AE0bAzPDu+IPL6K8WXVkAxyZ9jc30JB0qyRibGiUJXCBmEDMJz8CTfPOEzY6XJvTuPVoAYtuffYz+05yLi4CxSV9hepKen8EYiFm+xUxKafAknzxhsxs5NUH\/8eHnCwDwpJUH26523uGZn1ybnvS0Bb9TzqUdMkycTY0S5r7KjYWY5VPMwvIFPMkfT46cnKBPdR8sunE2PJ60wv93+9ikr7FkejyRZefOnUWzNJUZqoksQduGhe2cQfE2NUrQvUTxPcQsf2JWDm\/Ak3zwhDO7TX9\/lHY8fbzE7LyyOz3QJn2N3PSUuXV2djoL0vWPvmTh1KlT1N3dTf39\/eX0xdC\/salRQt9cGT+AmOVDzMqgRtFPwJNs84Szutu+\/bwzUUX\/+A1lItMzEIDpVSozyfweYpZtMYuKReBJtnjC5vbz42\/Tgz84RvtfOVNyc0HDmF68simpiDzTY1Akw5tqLZ\/b2XdRdVhb0+847t8sE2KWLTGLizPgif08YaPb\/MQI\/c0\/ve5KEza6v\/zMZbT8M\/Vl0yj3psfIua3T442neciTv1u9erUztNnY2Fg20GF+aFOjhLmvcmMhZvaLWbltH+Z34Il9PGGT2\/WT4\/Tk8GnXbI7viI3uP3ziIvr8x2e4zsYMwxGOtUlfY8n0wgKWRLxNjZIEHhAz+8QsCV5gRCAY5bT1Hd4Lc+CZ1+nHr5wpeTen3w0bXc8fNTpGF\/XHJn2F6UXd+paUl7aOmwbYgAkeBCQ8rCZPOIvb+cxx2n\/YO4tT98Am99mP1tJ\/mXe5809eyw0k9xwUk3vTcxvaVKA1NTVhw+kgBiXwfTU7bgK3V9YlgAlMT0KcJHiiZlP+9ZOj9MLorzyHKc1M7gvXXOS8m4vT4NwwyrXpTUxMOMcJtbS00KJFiwpHC2HDaUl3Si4miY6b3N1EcyVgAtOTMClqnrDBHX7jHfqvg6\/SsdPv+g5R6lnczAum0poFsxyDS9rkTJxybXr60UI8SYUXoTc0NDgnK3AGWK3T021qFEnHqzQm6o5baX3S8HtgAtOT8LAcnrCx8X8f\/J0P0D27D4vNTQ1LssGtnN9AH5lRU3WDQ6ZnIGCaHi9NGBkZIV6sjkNkJV0qmZhyOm4yNaveVYAJTE\/CPj+eqGHJ\/\/XPJ+h\/\/8tJ0bCknr3xn+deeQGt+VyD88\/VzuAkeHCMTUlFLBNZ9C3G9Oxu9+7dzjl7PT09VFMzeQ5TUh+bGiUJTCDwEHgJz8CTUpR+PDRMl112WWFZgHRI0jS3Wz5aWzitwG0TZ0n7pCXGJn2NxfTMXVnYBLdu3Up1dXWJrs3TCWFToyRBZIgZTE\/CszzyRA1FMj7\/\/blfOu\/bwhqbytJ4WHL2xR+k\/\/zZmQW4bcneJPxQMTbpayymFwaspGJtapQkMMmjmAXhCkzy8yCghiF5bdvA05M7lbhtyxXEGWVgv3\/FdFo9v4FeO\/PrVEwsCap31N\/bpK+Rm575Tk8HF+\/0oqZa+eVB4PMj8OWzhMhWnihTG37jHbp\/8NWyTU1lbPx\/Nra2Gy6hXx4\/Tp\/+2OSsSXwmEYDprVpF69atK9liTD9loba2NlG+2NQoSQBjq5jFiQ0wsedBQJnadw6O0dNH3yxr+FHdrTIvHor87FW19KnLf883WwNPSnlik75Glun5LUjXIUr68Fgbx5zjFHZVNjquPQKfBB+8rlENnihD+7tDv6QfvjReUZamZ2psap+sn0Z\/cdPkxspsdnytsBlbNTCpJgck186l6Slg\/IY3JeDFFWNTo8SFgV4uOi5MT8KzqHmiJon809E3ad\/LlRuaaWp\/NOdimndV8ShSWFMLwiVqTIKuZ8P3NulrZJle2hvGpkZJAkt0XJiehGdSnugzHv\/nP5+gn439n4ozNNPQLr+whlb+u8vpA+eeU6h61IYWJSaSsrISY5O+RmZ6apnC0NCQbzti78100FwqZumobTK1ACbuDwIf+L1LnWHA42\/9mn7w0jj9gncXEW6X5ddy+ru0y2qn0vLPXEZvTfymMPRYDUOTMA08KUUpl6YnIUs1Y2xqlCRwQsdFpqeys7E336V9L5+OzMzMDK1hRg3dNudimnLuOYX3Z2k1NEnfQ9+B6Ul4UvUYmF5xE6DjZtf0lJkdOfkO8buzqDIzhZi+Nu36mec7i691E7PZ0CRChb4D03PlCe+52dXVVfTdxo0bnY2nq\/GB6cH0gniXZjFTMxr5\/z8aPk2vnXk3NjPjWY6fqJ9G\/\/7jM+j48dfpj+deXdYsxyC8bf0+zTypFqY26Wtk7\/R0sNnwdu7cWXRunnrn19bWVhXjs6lRkiAuOm46Mj1lZs\/94i164sVxOnZqwqlYObuDePFGf3f2sbrfpT+8Zgade45sqBE8SQdPktCESq5hk75Gbnrmvps6kFicXgmtov0txCw+MdOzst0vvEE\/f\/1t52JRTP7Qa+2co3bB5K4gn5l9ATXPmu78uWByEewYAp7Ex5Noe3R1S4PpdXQ4Rwk1NzcXtQRMr7rE1K8OMQsnZsrIzhLR\/3j+DRp88VRsRuYY1wVTiWc0\/vF1F9O\/+cC5kZuZlIngSTieSHHNWlyuTY8bE8Ob6ac0xIwKJ1T\/9uxZ+t6hN+j7L4xRzdSpsWRkupE1fuiDdNucDxWun4aTr70YC57A9CRqlnvTU8aHiSwSulQnJqtipjKyd\/7vv9LfPv06Df3iV7FkZEXDiBdMpesvP5+W3FjnvCvTt7ayfSZjVnlSSa8DJqXo5dr0qj1hxYvMNjVKJR1S+lsbOq6aeq+M48nDp+mZkbfoyInyzjcLwqbu\/Ck0ZcqUwnuyaz48jZbN\/XDRDiC62QWVl4XvbeBJ0jgDE5heCQLm5tM7duwoeb+XNFFhesWIV6vj6ttVTT9vCvUfeI2Gf\/lOItkYvyPj\/xbfcGkBDH3T4WphknRfCHM9YILhTQlfbNLXyGdvmgDp6\/VwcrqEPsnERClm+mzFA6+coVfHJyJfQ6ajok\/B54XR1142jRourKn48M4oMUmmFeO\/CjCB6UlYBtPzQKm3t5c4C+zr6yOcpyehUnwxQWKmjOznx98m3kB45OTk+rGop93rw4Vq+v1Vl36QvnDNRTRrRo3zfmzulZNT8eP+BGES9\/XTWD4wgelJeAnT01Bio9u6davzL5LNpicmJmjt2rW0Z88e5zd+5++Zw6h+maRNjSIhWTkxupE99vQIvfb25FT4KBdCq3qZmwl\/4RMX0flTpzhfR7mOrBwcvH4DgYfAS\/gEnpSiZJO+xjK8WcmQJpskf3idX9CkGL7OyMiIExv0salRgu7F7Xs2NJ56\/8j+1+iF0V9FnpGZJjaztobab7jEqUpWZitCzGB6kr4HnsD0ihDw25FFQigzRjdBt+8aGhpE25rZbnoqS7v\/H191Jn5EMcyoGxmb2IKPX0hN9dMK2Zi6pu3T7qW8g5jB9CRcAU9gehKelBXjZ6BqGLSlpSVzprf\/8Bn6h5+fokOvvlXW0KNuZtfNPJ+W\/v6H6f1jNyeb4l\/ffJ1mzZpVVrtk9UcQM5iehNvgCUxPwpPQMepd4MKFC6mnp4dqampcM0r90Fq\/UxzSnOlxRnXHwItig1N7Ls68cCrd+bkGOkezNGlWho4LgZd0SvAEPJHwJM36atY\/lnd6EpCkMWx+Y2NjJcY3PDxMS5cupc2bNztrAM2\/m+Vzo6jP4OCg9PKxxb325m9o\/eBJeva1dz2vwYulL502hRZfez5N+51znT\/zv0XxGR0dpfr6+iiKykwZwKS0KYEJMPHq4PPmzSv66siRI1ZoQepNj81szZo1tGnTJmpsbPQF1e\/9X7WfRNT7Ma+MTmVoOzo+SeNv\/7\/Yp+njCR5P8BKFAk\/AEwlPqq2vkjqqmMhNj9\/DrVq1itatW1diUmxg3d3dtGXLFvE6vTAnM7DpeU1sqWaj8Du6Rd96rqRd2OjaPnUJLf70pUUnT4dpwHJjIWYQMwl3wBPwRMKTauqrpH56TKKmJzEwPVtTk1V4\/Z25LMEsi\/++evVq6u\/vd80Ik24Utd3WHTtfLOymz8Cz0c298gJ6sO2qsG0VaTzEDGImIRR4Ap5IeJK0vkrq5BUTmemZC8W9Lui32Jx\/Yy5O1yey8DUGBgYK7\/fC7PGZZKO4TUxhs3uw7eqKt8qqpLH130LMIGYSLoEn4ImEJ0nqq6Q+fjGRmZ66iN\/wZqWVreT3STXKwDPHacXAi4WqKrNLaistKUYQM4iZhCvgCXgi4UlS+iqpS1BM5KYXdMFqfR93o3B2x3tUfv3xw0WGt\/srcxJ\/XyfBGGIGMQNPJAiAJxKU4tZXSR2kMZGZnlpIvmzZMtq2bRvp6+f0ykj235RWPkxcnI3ChscZXu\/eo06V0prd6XjB9CBmkv4DnoAnEp7Eqa+S64eJicz0wly0GrFxNYr5\/o4NL63ZHUzPn3kQeAi8RJvAk1KU4tJXSXuEjYHphUVMi2fD4+yOszyV4dlgeFxXdFwIvIT64Al4IuFJrk1PDXPmYXhz+1Nj9LXvvmSd4cH03LsxBB4CLxF48ASZnoQnzlKEDRs20JIlSwJ3VhEVGDIo6icRfcG5LUOaGN7E8GbIboMRARfAYHowPXE\/MtfZiX8YQWCUpsfDmtfe+1Qhw+P1d2lbkhAEGTouspogjmBEACMCEo5wTJT6Kr1muXGJvtMrZxuycm\/M\/F2UjbLooecKJyJ0LphFnQsaoqpmYuXA9GB6ErKBJ+CJhCdR6qvkepXEJGp6XicmVHID0t9G1Sg7nj5OvLUYf\/7005fSA1XeTkx6\/2YcxAxiJuEOeAKeSHgSlb5KrlVpTOSm5zeRhffQ9Nobs9IbCfp9FI1iDmvaMlPTDRuIGcQsqM\/w9+AJeCLhSRT6KrlOFDGRm14UlYqjjCgaRR\/WfP7rN6ZypxUpdhAziJmEK+AJeCLhSRT6KrlOFDGxmJ7aNLqlpYVaW1sLm0i7nZYQxU1Iyqi0UfTZmnOvmE67V8yRXDa1MRAziJmEnOAJeCLhSaX6KrlGVDGxmJ7bYa5+xwRFdTN+5VTaKHqWx8Oats3WNLGBmEHMJP0OPAFPJDypVF8l14gqJnLTi\/oQ2ahutJJG0d\/l3XTlBfT4V66NqlpVKwdiBjGTkA88AU8kPKlEXyXlRxkTi+l1dHQ4h742NzcX1VVyiGyUN6eXVUmjZC3LY1wgZhAzSV8DT8ATCU8q0VdJ+VHGRG56XLldu3bRzp07qa+vj2pra536qlmdbW1tznu+pD\/lNoqe5WXhXZ7CHWIGMZP0QfAEPJHwpFx9lZQddUwspseVdDtJfePGjVUxPK5PuY2SxSwPmZ78UeZ0AAAUe0lEQVR7N4LAQ+AlAguelKJUrr5K8I46JjbTi7qilZZXTqNkNcuD6cH0pP0JAo8HAQlXytFXSblxxERuemqWZnt7e8k7vThuQFpmOY3CRwatGJjcfcX2dXkmThAziJmk74An4ImEJ+Xoq6TcOGIiNz2\/2Ztx3IC0zLCNoh8Oy6cosOll6QMxg5hJ+AyegCcSnoTVV0mZccVEbnpcUZ7IMjIy4szgTMsnbKPoi9Hbb7iEHmq\/Oi23Ekk9IGYQMwmRwBPwRMKTsPoqKTOumMhNLyuHyGZ1AosiEsQMYiYRFfAEPJHwJNemJwGoGjFhGsXcWDprQ5uMP8QMYibph+AJeCLhSRh9lZQXZ0zkmV6cla2k7DCNog9t8rAmD29m7QMxg5hJOA2egCcSnoTRV0l5ccZEZnpqWHPZsmW0bds2Ghoacq13U1NT0aL1OG9OLztMo2R9aBOZnjvrIPAQeIkegSelKIXRVwnGccZEZnpxVjKKsqWNkoehTZgeTE\/apyDweBCQcEWqr5Ky4o6JxfRsPlpIH9rsXDCLOhc0xN0GVSkfYgYxkxAPPAFPJDzJvenZfLRQ349fozu\/97LTzlk4QsiLsBAziJlEzMAT8ETCk1ybnu1HC6n3eVlckK6TF2IGMZOIGXgCnkh4knvTs\/loodqVP3DaeNnceuq9rVHS3lbGQMwgZhLigifgiYQnuTY9BsjWo4V6945Q796jThtnba9Nk7gQM4iZRMzAE\/BEwpPcmx6DZOPRQnkZ2uT2gZhBzCRiBp6AJxKewPQkKCUcE9QoWT5GyA1qiBnETNIFwRPwRMKTIH2VlJFUTCxLFpKqfJjrBDWKbnpZXqqgMIOYQcwk\/Qc8AU8kPAnSV0kZScXA9N5DOk\/v8zC86d69IPAQeInwgielKMH0JMxJOCaoUfL0Pg+mB9OTdj8IPB4EJFwJ0ldJGUnFINN7D2m1VGHuFdNp94o5SeFftetAzCBmEvKBJ+CJhCe5N73h4WFaunQpjY2NleCVxg2n8\/Y+D5keMj2JkIEn4ImUJ7k2PbXvZl1dnTUnpw88c5xWDLzotG+Wtx7TCYwneDzBSwQNPAFPJDzJten5bUMmAS+uGL9GaX\/kBdr7s1POpbO+KF3hCzGDmEn6GngCnkh4kmvTU5lee3s7NTc3S\/AqO0ZfAM+ZZX9\/PzU2um8d5tcoeZvEgmErDFtJOx1MD6Yn4UquTY8B4m3IDhw4QD09PVRTUyPBLHQMvzdcs2YNbdq0yTE6t63P9EK9GiVvi9KR6XlTDQIPgZcIEXhSilKuTU+doJ70yemmCZrN4tUoeTk\/z8QDHRcCD4GXIACeSFDKtelJAIojJii79GqUvC1KR6aHTC9M\/8PDEUxPwheYngSliGL05RE7duzwfI\/o1Si3fut5evLwaac24\/fdElGt0l8MxAxiJmEpeAKeSHgC03vvvV5XV5eDF5sRfwYGBmJ7z6fMb\/Pmza7G59UoeZzEwm0BMYOYScQMPAFPJDzJven19vY6C9PvueceWr9+PamZnPzv\/Ons7JTgGDrGr3xuFPUZHBx0\/jj21m9o4fZR58\/Xf3gq\/fVtl4S+pq0\/GB0dpfr6elurH0u9gUkprMAEmHh1tnnz5hV9deTIkVj6ZdSFRr4Nmb5Oj0V17dq1BdPjbKy7u5u2bNlCtbW1kd6LWirR0tJCra2tJWW7PYnkdRILMj136iGrQVYjESXwpBSlXGd6fqbH6+o4G+vr66vY9MzZmlz26tWrPdfqBZleXnZiUXRFx4XAQ+AlCIAnEpRybXoMkJpJqQ9vzp49mzo6Oqitrc01E5MAa8aYp7OHncjCW4\/xFmT8yctOLDA9b6bhQQACL9Eh8ASZnitPTEPioI0bN0ZmeBJy6jFuTyJ5ncSC4U0Mb0r7DwQeDwISruQ+05OAlHSMW6Pk7TghHXOIGcRM0gfBE\/BEwhOYngSlhGPMRtG3H7vvTz5Kf35jXcI1qu7lIGYQMwkDwRPwRMITmJ6xTk+B5vfOTQJsJTFmo+gzNx9qv5rab8jPcgUMb2J4U9qXYHowPQlXcm96bps\/qz05o5zIImkMFeNnenmbuQnTg+lJ+w5MD6Yn4UquTU+ZGy9AN48WinLJgqQh9BizUfI8cxOmB9OT9h+YHkxPwhWYXkeHs+uKaXpxLk4PahizUfI8cxOmB9ML6i\/qe5geTE\/ClVybHgPktlA8bcObeZ65CdOD6UmEDDwBT6Q8ybXpBZ2np4PY1NREjz32mBTXiuLMRlGmxxNYeCJL3j54gscTvITz4Al4IuFJrk1PAlA1YvRGyfOemxi28mYfBB4CL9Em8KQUJZiehDkJx+iNwluP8UQW\/uRx5iaGrTBsJe1+EHg8CEi4AtPzWKeXlm3I8npauk5eiBnETCJm4Al4IuFJ7k0v7ev08j5zE5keMj2JkIEn4ImUJ7k2PRvW6cH0cHK6W2dGVoOsRiLy4Ane6RUhYIPp5X25Ap7g8QQvEXfwBDyR8iTXmR6DlPbhTWV6nQtmUeeCBmm7ZioOT6vIaiSEBk\/AEwlPcm96yvi6urqK8ErDRBYsV5hsEogZxEwiZuAJeCLhCUxPglLCMapRsFwBpudFPQg8BF4iS+AJ3ulJeFL1GGV6258ao6999yWnPnldo4dMD+9qpB0SAo8HAQlXkOlJUEo4RjWKmrnJlx+\/75aEa5Gey0HMIGYSNoIn4ImEJzA9CUoJx5imN7N2Kj3\/9RsTrkV6Lgcxg5hJ2AiegCcSnsD0JCglHKMaBcsVJoGHmEHMJF0QPAFPJDyB6UlQSjiGG+WHP\/kZXXvvU86V83q6goIdYgYxk3RB8AQ8kfAEpidBKeEYM9PL8xo9ZHru5IPAQ+AlsgSelKIE05MwJ+EYbpTv\/P0hWvSt55wr8xl6nO3l9YOOC4GXcB88AU8kPIHpSVBKOIYbZfnD\/0i9e486V87zcgVkesj0pN0PpgfTk3AFpidBKeEYbpRNf\/cU\/cXf\/My5Ms\/c5Bmcef1AzCBmEu6DJ+CJhCcwPQlKCcdwo1yz6nu0\/5UzzpXzvEYPmR4yPWn3g+nB9CRcgelJUEo4Rje9vK\/Rg+nB9KTdD6YH05NwBaYnQSnhGG6U82\/\/Wzo2\/i7NvWI67V4xJ+EapOtyEDOImYSR4Al4IuEJTE+CUsIxDdd8mt76XK9zVZgeFqe70Q8CD4GXyBJ4UooSTE\/CnIRjdNPL+xo9DG9ieFPa\/SDweBCQcAWmJ0Ep4Rjd9PK+Rg+mB9OTdj+YHkxPwhWYngSlhGPq\/+DP6J3rbneumvc1ejA9mJ60+8H0YHoSrsD0JCglHFP3h1+jd69aBNN7D3eIGcRM0gXBE\/BEwhOYngSlhGMuvXUd\/fqK+c5V874wHZkeMj1p94PpwfQkXIHpSVBKOOZDX\/w2\/WbGR51dWPJ8jp6CHWIGMZN0QfAEPJHwBKYnQSnhGJheMeAQM4iZpAuCJ+CJhCcwPQlKCcfM+PJ36bfnzcAaPbzT82QeBB4CL5El8KQUJZiehDkJx6gT0xd87EIa+NInE756+i6HjguBl7ASPAFPJDyB6UlQSjhGmR4Wpk8CDzGDmEm6IHgCnkh4AtOToJRwDEwP7\/SCKAeBh8AHcQQPjO4IwfQkzPGIGR8fp46ODhoaGnIiFi5cSD09PVRTU1Pyi4MHD9LixYsL\/15XV0f9\/f3U2NhYEqtMDwvTkel50ROmB9OTSBd4UooSTE\/CHJeYiYkJWrt2LbW0tFBrayupv7OZdXZ2lvxi165dNDIy4vqdGQzTQ6YXREuIGUwviCPI9JDpSThSUQwb24EDB1yzvd7eXmpoaHAMMuijTA8L05HpIdML6i3vf48HATwISNiCTE+CkjDGy\/TMrDCoOGV6eT8xXeEEMYOYBfUZZDXuCKHvYHhT0nfKilHv99ra2kqyOfPdH19g48aNnlkfmx52Y8ETvB8RIWZ4EJAIFXgC05PwJHSMyuT4h24TWYaHh2np0qW0efNmam5uJvPv5gVhesWIoONC4CWdEjwBTyQ8wfCmBCWfmCDD8\/opv+Pjj9ukFza9KSdfot\/dv4kGBwcrrKH9Px8dHaX6+nr7byTCOwAmpWACE2Di1cXmzZtX9NWRI0ci7I3xFXXO2bNnz8ZXfPiSg2Zs+pXoN7GFTW\/uFdNp94o54SuVwV\/gCR5P8BJagyfgiYQnyPQkKHnEsHGNjY15rs1TP+M1ehzb19dHtbW1xH9fvXq17zo97MbyPug2kbQCOoX6KTAphQuYABNJJ7KJJ6nK9NwmpzDgTU1Njrm9\/PLLNDAwUDBEc3H6jh07nPd7bh\/O9GB6MD2\/DmxTx5UIURQxwASmJ+GRTTxJlelJwC03hk3vvEOP0r899uNyi8DvgAAQAAJAwAMBvNMDNYAAEAACQAAIpAyB3GR6KcMd1QECQAAIAIEqIADTqwLouCQQAAJAAAhUBwGYXnVwx1WBABAAAkCgCgjA9KoAOi4JBIAAEAAC1UEAplcd3HFVIAAEgAAQqAICmTc9PqWhq6vLgdZvQ+oqYJ\/IJXkB\/9atW51r+a1j1OP8DuNNpNIxX0SKiapG2BM9Yq5+LMVLMTHX0vpxKpaKJlioFBO17y9vqpH1vuMHf5ij3hJsxpJLZdr0mIxr1qyhTZs2OTeu\/ux2sno1GyGua+u71vDCfn0HG\/2a5vFN\/PedO3cWdruJq37VKFeKiYkPPzhl9aFJiom5RaDev7LWp6SYqIcA3u+XN8bIct8JMjx+uLahj2Ta9Ewxt+VJJCoz0DfgVoLV3t7uuWuNum6WxSwsJixqq1atojNnzpDbEVdRtVU1y5Fiwrzo7u6mLVu2OFv\/ZfkTBhP9YTrLfcetvZXpT58+3fn685\/\/vOhQ72pyJ9OmZ5664HcKQzUbIY5rm0NyYYbostpxy8GEOXPDDTfQ448\/Ti0tLanv0GG5FAYTrwOdw14z7fFhMHHL9A4cOBC4d3DaMZDWj+\/\/9OnTzrDu2rVrregjmTe9hoaGglBxpx0ZGXE9ekjayLbEuWV20kxXuum3LVioeobFhM1\/+\/bttHLlSlq\/fr0VHTpsm4TBRPUfvobkPXHYuqQlPgwmXGcVv2fPHlq+fHku9MVsqzAP1dVuZ5hetVsgpuuH7biqGixsDzzwgOdpFTFVN5Fiw2DCsRs2bKAlS5Y45w7a8hQbFsgwmKhJYWryStDJJmHrkpb4MJiYh1ebp7+k5Z7irgdML26EheVjePP94QYJKbNsePoTuRqm9MOExWvfvn3OU7sEOyElUxcWZijPHN7MKi7AJDxNbeJCpjM9czhTOrwXvsnT+Qv9foMmsuRl1pkUE326ut66WRy+kmLCDwL60V5BnEpnr5DVSopJXh4EglCD6QUhlND3WLLw\/kG7fksWsjpM5UYz6VR0\/bc2dehyupYUE3PSRpaH8qSYuA1v+h1mXU772PAbm\/pIpjM9JgsWp7svTtezYK+sJqsLj70WHXtNdLKpQ5crkFJM9MXpWV+ILcVEP8w665h48cumPpJ50ytXBPA7IAAEgAAQyB4CML3stSnuCAgAASAABDwQgOmBGkAACAABIJAbBGB6uWlq3CgQAAJAAAjA9MABIAAEgAAQyA0CML3cNDVuFAgAASAABGB64AAQAAJAAAjkBgGYXm6aOns3au4Q4neHce8eoq9fW7hwoXiXfXPBd9paSa1Va2pqivV8RX3T5jD4pQ0v1Cf9CMD00t9GqKEHAmkyvTB10W\/HBtPj+vIepEl88nJ8URJY4hruCMD0wIxUI6BnUFxRtfelvguGykJqamqc0xD4iBf14ZOcFy1aVPTv6nRnPbvg+KAMw2vnDX3XHy7HbScbr\/tQ\/86nbqvjesysSr8ul69\/z9c+ceIEDQ4O0tDQkHNt\/l7H4e6776bdu3fTpk2biE84D3Pf5qbtYY4XcjP0IFML+j7VZEXlrEAApmdFM+WzkkG73evZFSPEQs\/bQKmsRD81Qh0PpE6Od9s2ye+QYXN\/Ure\/65sx6y3mdx\/z58+njo4OmjlzpjMkat6HeR3dJPk+3U7G0M9DVOU9++yzznFRbsck+d23m+l1dXWRenAw954MymKDTC3o+3z2BNx1lAjA9KJEE2VFikDQfn5BQ4r6huOm6bn9lg1l1apVtG7dOicjUh+veuiG4FcXv42Zy8mG9OuaJuF2D7pxnjp1quikBL5Hr\/vm79xMzzwZ3Ms0y7k3mF6kXQiFuSAA0wMtUo2APrRnDj96GY2+UbDaANg0PXNIUoHgtmGw13s3fYNqP9PzE3KpMaiMamxszKmqGuY1y+a47u5u2rJlC9XW1jqxuvkfOnSIOFMzP14bJXsNb+rv+LzuT3pvel1geqnujpmoHEwvE82Y\/ZvQRV9\/r6cPKSqzU+Y4OjpKa9ascd5luZmembF4oVhN0+N7WLp0KbHZqXeFfpmexPSk9+2V6Y2MjBRNbIHpZb\/\/ZekOYXpZas0c3It5zpkyPR6CNIcmg4Y32Tz6+voKGZEXfNUc3uQJKKbJVDq8Kb1vDG\/moEPl8BZhejlsdFtuOWiyiT6kyLE8IYSH3XgmpMrOeGajPoHDnMiiT3zxe\/cW5UQW3UyWLVtWVG\/+Ts+c2PT0zEwNy3oNb6qyOTPUJ8aYE1mk9+01kUVlnfqDhf4elOuh2k9dS7WJmrTDs23ND4Y3bemd9tYTpmdv2+Wi5ua7LP29nmlsPElj8eLFDi4stJs3b3YmYrS1tVFra2vhQGFlGOYygqAF2H6HhQZNqjGvpe7DNGvT9Pjv+vIDrntDQwPt3LnTyVKfeOKJIlPUzUYt3eAlCz\/60Y8K7\/nC3Leb6X3\/+993MN63b5\/zf32Jhts7RjU8y\/iyye\/du9cx5KB7dzPFXJAeNxkrAjC9WOFF4UCg+gi4veeT1koye1NaliQOmZ4EJcRUggBMrxL08FsgkDIE3Cbd+K3DC6o+TC8IIXxvGwIwPdtaDPUFAgEImDu4qOHccoAz9950G04tp1zzN9h7MwoUUYYEgf8PGgdQnjeqVR0AAAAASUVORK5CYII=","height":268,"width":445}}
%---
