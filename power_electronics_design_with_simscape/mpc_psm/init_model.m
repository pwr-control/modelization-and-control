clear;
[model, options] = init_environment('mpc_psm');
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

fPWM_INV = 6*fPWM;
tPWM_INV = 1/fPWM_INV;
TRGO_INV_double_update = 0;
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

hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, fPWM_AFE); %[output:902685e6]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, fPWM_INV); %[output:9e9d4466]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, fPWM_DAB, fres_dab); %[output:833598fd]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, fPWM_CLLC, fres_cllc); %[output:8ba1bd62]
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
l1 = Kd(2) %[output:75480c5c]
l2 = Kd(1) %[output:3356c104]
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
parasitic_dclink_data; %[output:8c8cc1aa]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:4a13890b]

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
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:3ae14074]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:7858ff8a]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:8b993c89]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:4f0a8057]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid_emu_data.w_grid;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:2be5a5f7]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:0aedf096]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:93d9fff2]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:09c73bc1]
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
run('n_sys_generic_1M5W_pmsm'); %[output:2788be2b] %[output:1ed1daa1]
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
kg = Kobs(1) %[output:3b2d4c40]
kw = Kobs(2) %[output:6142afe8]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:513fbeb6]
luenberger_l2 = Klo(2) %[output:72f2672f]
luenberger_l3 = Klo(3) %[output:5b8b0b72]
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
lithium_ion_battery = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_inv); %[output:6705b52e]
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
    set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
    set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
else
    if use_thermal_model
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'off');
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    else
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    end
end

if use_torque_curve
    set_param('mpc_psm/fixed_speed_setting', 'Commented', 'off');
    set_param('mpc_psm/motor_load_setting', 'Commented', 'on');
else
    set_param('mpc_psm/fixed_speed_setting', 'Commented', 'on');
    set_param('mpc_psm/motor_load_setting', 'Commented', 'off');
end


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":38.5}
%---
%[output:902685e6]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:9e9d4466]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:833598fd]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:8ba1bd62]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.880000e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:75480c5c]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:3356c104]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:8c8cc1aa]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQ+QVUeZ7xdFZbLBhYnRSCYwJIJGyiJRo+MkClleZLcUdh9RYdBXSE0wjySV1DoUM0x4RViT+UM5KZNNgiQZ2dnSmWCUfcvUqmyWCPXihDWahH0b40IyDDhiDI8Bk5SDiubVd0jfnLnce6fvuX3O6fOb36mimLm3T5\/+fr\/v6\/7N192nz3nttddeE15EgAgQASJABIgAEcgQAudQwGSILTaVCBABIkAEiAARCBCggKEjEAEiQASIABEgAplDgAImc5SxwUSACBABIkAEiAAFDH2ACBABIkAEiAARyBwCFDCZo4wNJgJEgAgQASJABChg6ANEgAgQASJABIhA5hCggMkcZWwwESACRIAIEAEiQAFDHyACRIAIEAEiQAQyhwAFTOYoY4OJABEgAkSACBABChj6ABEgAkSACBABIpA5BChgMkcZG0wEiAARIAJEgAhQwNAHiAARIAJEgAgQgcwhQAGTOcrYYCLgDoGDBw\/KqlWr5OjRo7lKFy9eLB0dHVJVVTXug0ZHR6WlpUUaGhqkrq5u3PL79u2TFStWjCl3ww03SHNzs5Rb17gPYwEiQASgEaCAgaaXxhGB0giogFm3bp1s3rxZZs+eXbaIKFd0qIDp7OyU7u5uqa6uzj1v+vTpgYjhRQSIABGwRYACxhYpliMCgAiMJ2DCGROTKVEYVIRs3bpV5s+fH6Ci32kGZvv27bJ+\/frcZ\/miJF\/AaEFtQ1tbm9xxxx2BkNJszrx584LMTn9\/f1CXyQrpz+Zz\/ezll1+W1tZWeeqpp2RgYECOHDkiy5cvl5kzZ47J9PT29sqcOXOkqalJPvGJT8hXvvIVUdH01a9+NbBl\/\/790t7eLsuWLQNkmSYRAUwEKGAweaVVRMAKgUJTSGYgD4ubmpqaQDjU19cH4sBkUY4fPx5MQakQ0Kuvry+YfjJCI39qqZCAGRkZCYTFl7\/8ZXnooYcCAXPxxRcHdVx00UVivg8LFX2Gio61a9fKtm3bAgHz8MMP5zI7+hwzpaWiamhoSFavXi2NjY3B5yqs1AYtp9mgRx99NBBAtlNnVuCyEBEgArEiQAETK7ysnAj4jUCxDIwRKkaQ6HoYIwSMRfnrVg4fPpzLvpgy4ayNfmYrYFRkmOkpzcJotmTLli2BwNG2aaakmLAxa3fys0dGwGi7TbZIhY3+rmXDtvrNGltHBIiAIkABQz8gAhMYgXwBo1AYoaLTQ+UKGCMIikFqO4Wk9+ti3\/DUj8nQjCdgTPZH\/9eMys6dO8dkYChgJrDD03QoBChgoOikMUSgPARKZWA++MEP5hb42k4hmSmdcPnwupJSi3hvueWW3I4mtcKIp\/ypIjPVU+xzI2DCa2k0g8MMTHm+wdJEwHcEKGB8Z4jtIwIxIjDeNuo4FvHabKPWBbe6XkVFipZ\/5ZVXzlrcW2gRr1nDYhYTq3DRep555plAjN18883BlBGnkGJ0KlZNBBJCgAImIaD5GCJABNwiUGg6yu0TWBsRIAI+I0AB4zM7bBsRIAJjEAhv09YvdI2MzQv0CCMRIAJ4CFDA4HFKi4gAESACRIAIwCNAAQNPMQ0kAkSACBABIoCHAAUMHqe0iAgQASJABIgAPAIUMPAU00AiQASIABEgAngIUMDgcUqLiAARIAJEgAjAI0ABA08xDSQCRIAIEAEigIcABQwep7SICBABIkAEiAA8AhQw8BTTQCJABIgAESACeAhQwOBxSouIABEgAkSACMAjQAEDTzENJAJEgAgQASKAhwAFDB6ntIgIEAEiQASIADwCFDDwFNNAIkAEiAARIAJ4CFDA4HFKi4gAESACRIAIwCNAAQNPMQ0kAkSACBABIoCHAAUMHqe0iAgQASJABIgAPAIUMPAU00AiQASIABEgAngIUMDgcUqLiAARIAJEgAjAI0ABA08xDSQCRIAIEAEigIcABQwep7SICBABIkAEiAA8AhQw8BTTQCJABIgAEcgqAqePDckrP+yRaZ\/bmFUTYms3BYwFtJdccolFKRYhAkSACBABIuAGgQvfelr+x\/QTsuj8V+XF30+SL\/zfiyuqeHBwsKL7fbyZAsaCFRUwlZBvc3+pMoW+G++z8Pc2P1vAcFYRG7v0pmLlbD\/PLzeePbbtKmaz7f2Vcpa0XaW4CGNBu86gwRg7M+AVijfbGGGMFUagFH5rrnyn\/K9PzZVTz+6RSRfUypQFX5QPtfSMGYPS6DuijBFx30MBY4FwpcFq8YhUitCuVGCP\/FDyFRm61G4kZ6lBH+nBafGl00Sjz+6RV\/b05ISLThmpeHFxpWWXi7aXqoMCxgJhVPJplwX5HhUhXx6RYdkUcmYJlCfFkubLrG95Zc8\/BAhUzV0gUxaslMlzFzhFJGm7nDa+RGUUMBZIo5J\/6NAhmTVrlgUC2SpCu8iXLwjQF31hwq4dSfFlhMuJR24PpolUuGjGRX+O40IdwyhgLLwFlfykgtUCYqdFaJdTOGOvDJUvBQ7VNtoVLSzyhYtOEU25ZmVswsW0EnUMo4Cx8ENU8tkJWZDvURHy5REZlk0hZ5ZAeVIsLr5UuJz49ibRqSKzMDfJbdGoYxgFjEXgoJIfV7BaQBprEdoVK7zOK0flixkY564Se4WufVGFy0v3rRqzoyhJ4cIMTOwu4\/8DKGD85yjcQtedkC\/W0y5fmLBvBzmzx8qHki74intHURScUMcwZmBe94Z9+\/bJihUrgt\/a29tl2bJlOT9BJd9FsEYJprjvoV1xI+y2flS+mIFx6ydJ1FaJLxrholNF+nOwviWGHUVRcEAdwyhgRGRkZESampqktbU18I22tjbp6uqS6urq4HdU8isJ1ihBlNQ9tCsppN08B5Gvzl1DATgnT5yQl1+bLDOqq3JgNS+KZ6eJGzbsakHkLKrgTGortB0zhUuhjmEUMCKi2ZfOzk7p7u6WqqoqaWlpkYaGBqmrq6OAqSRqUrqXnWtKwEd8bJp89T35YtDqIyOnxrT+yMho7vdfhL47cuJMufzy+abPqJ4cfHT69Gm55ILzztxT4F4t13DluyVroiZNziK6mdVt5diV1o4iK0PyClHAREEtI\/eogOnr65OOjo6gxSpg6uvrc9NI1V\/+YUYsYTOJABEoF4Hpb5901i3vnvLGZ+Hvp7\/++bvfPkkWX3ZGmJS6hoeHpaam5qwiR18+LT\/95Sn51cunpf\/nr4r+fsNHpsqXPjp1vCq9+L6YXV40roJGWNl1Ylhee\/RukZ98V2RajciHr5Nzrr21gqfGc+vChQvHVFzJcTjxtLDyWpmBeT0DU0rAoKrXcv7aqNzVkquBdiWHtYsnofKl2NjaplNOfU\/+KsjsNC+a5X1GxtYuF\/6RZB2l7MrfCj3j\/kNJNq2iZ6GOYRQwnEKqKDB8vHkidq4+8mDbJlS+yhEwYaw046tTSztvvCL438cLlbN8u3zcURTFHyhgoqCWkXu4iDcjRFk2c6J0rpZweF8Mla+oAkazMDf3PSePv3BSRu66xkv+UDkzdvm8oyiKQ1DAREEtQ\/eEt1H39vbmFvCqCajko3dCGXI\/q6aSLyuYvCpUCWc6rdS565A0XHmh3NdwGYxdXhmS15hDT\/0fmfr8Y8Ebc\/WK63DFpDFAHcM4hWThSajkV9K5WsCWWhHalRr0kR6MylfUDEwYxMefPylL7n9arr50quy86YpI+MZxExpnWdpRFIVP1DGMAsbCG1DJR+uEDJW0y8KpPSqCypcLAaN1+ChiUDjLFy6nL\/9rueRLX\/MoOtw0BXUMo4Cx8A9U8lE6oXwKaZeFU3tUBJUvVwJG69F1MZff8USwqPeZDR9Lnb2sc1bscMWs21XMMVDHsMwKmO3bt8v69evH8JV\/BICrKEclHzVYaZcrz0+mHlS+XAoYI2J0ca++EC9tEZNVznRtyyt7enKHK+rBivrKf\/TsLeoYljkBYxbbFhIrRtTkL8KttBtGJT+rndB4fNKu8RDy63tUvlwLGJ9ETJY4y98KPXnuAqn+7EbR\/ydK9hZ1DMuUgNHtzv39\/bJy5cqSPXBPT8+4ZcrpwlHJz1InVA5ftKsctNIvi8pXHAImX8Tcu\/wyufo9yb+9NwucRTmjKAt2RYlY1DEsUwImCnEu7kElHzVYaZcLr0+uDlS+4hIwPogYnzmrZEeRz3ZVEpGoY1jmBMzBgwdl1apVcvToUdFpJL10Lcz06dNl27ZtMnv27Ep4LngvKvmowUq7nIdArBWi8hWngDGELLnv6WBNTNKZGB85KyRcdI1LOZePdpXT\/mJlUcewTAmY0dHR3EnRc+bMkcbGRlm+fHlw6KKufxkYGAgOZNQTpV1eqOSjBivtcun98deFylcSAkafYURMkgt7feLMhXAxXu6TXS4jD3UMy5SA0TUwmzZtko0bN0p1dbV0dnbK\/Pnzg7fm5n9H8sdHADVYadf43PtUApWvpARMWMQklYnxgbP8rdD5O4qi+LgPdkVp93j3UMCMh1AC31PAuAUZNVhpl1s\/ibs2VL6SFDBJi5g0OTNboU+\/NOT8Vf9p2hVnnFHAxImuZd0UMJZAWRZDDVbaZekAnhRD5StpAZOkiEmas\/BWaCNcNOMy6YJap16ctF1OG1+iMgqYpJAu8RwVMLruZf\/+\/QVLzZs3T7q7u4PpJZcXKvmowUq7XHp\/\/HWh8pWGgElKxCTFWf5WaH3p3JRrVjoXLsbLk7Ir\/qga+wTUMSxTa2CSJt08D5V81GClXWlFSrTnovKVloBJQsTEzZnLhbnleGXcdpXTFpdlUcewTAkYZmBcurQIarDSLrd+EndtqHylKWDiFjFxcnbi25vkxCO3B1mWGfcfitv9xtQfp12JGpL3MAqYNNEv8GzdNj00NCTNzc3Bt\/q7Xrql2vWFSj5qsNIu1xEQb32ofKUtYIyIefyFkzJy1zVOSXTNWRw7iqIY7NquKG2I4x7UMSxTGRhDbKEt09xGXb7bowYr7SrfF9K8A5UvHwRMWMTsvPEKZ8cOuOIszh1FUXzalV1Rnh3nPRQwcaJbZt3mhXb19fW5jIu+E0bfzjvei+zCp1iHF\/2aQyK1KfkHRaKSjxqstKvMgEq5OCpfvgiYOERMJZyZHUU6VaRX1dwFEseOoihuXYldUZ6X1D2oY1gmMzBKev56GJsdSHoMQVtbm3R1deVehKeiZ926dbJhwwZpbW0N\/ClcRn9HJR81WGlXUt2im+eg8uWTgHEtYqJwlvSOoijeGcWuKM9J+h7UMSyzAsaFA2jWpa+vT5YuXSpf+9rXgi3YegxBS0uLNDQ0BG\/4pYBxgXSydaB2QrQrWT9y8TTfONNjB1ysiSnXrjQX5pbDY7l2lVN3mmUpYNJE\/\/Vna9alv79fVq5cWbI1PT0945bRCnTaqba2VmbOnBkIGZ1+0ksFTHh6SskPX7t37\/YAjcqbMDw8LDU1NZVX5FkNtMszQsZpDipfaraPtn1px4vy01+ekgeWXigfumhyJGexsuvEsLz26N0iP\/muyLQaOefaW0U+fF2k5yV1k5VdSTWmwucsXLhwTA2Dg4MV1ujf7ZnLwJi1KvnrVBRas76lt7c3lz0pBnl4F5PJxJQSMIjko\/61Qbv862hKtQiVL7XZV9tu6ntO+p58UaIu7C1ll04VvXTfKjn17J5gK\/Q7b9omk+cuyIRT+spXpeAxA1Mpgo7vDy\/GNVXnixpd87Jq1apgce\/ixYtzC3xN5sVsuVYBo59xCskxSSlVh9oJ0a6UHKqCx\/rMWeeuIencdSiSiMm3q9Cr\/qcsWJkZ4WIo9pmvCtwQdh1n5jIwlZCo94ZPsDZ16dRUU1MTF\/FWCq4n96N2QrTLEwcroxm+c6ZZGM3G3NdwmTRceaG1ZcauLCzMtTbK44xZOTYUKssMTKUIenB\/eKu0aY7JzOj5SitWrAg+zp+CQiXf9841qsvQrqjIpXMfKl8+TyGFmY4iYpSzqU\/+Y2pvzI3LU1F9EXUMm3AZmCiOj0o+arDSrihent49qHxlRcBoOx9\/\/qQsuf9paV40S5oXFT\/hOf+NuXq4or7DBeVC9UXUMYwCxiLyUMlHDVbaZeHUHhVB5StLAiYsYnQqSaeUwpd5Y65ZmPvHa26SWZ9d65EXuWkKqi+ijmGZFTDhF9k99NBD8thjjwVbp2fPnu3Gk0O1oJKPGqy0y3kIxFohKl9ZEzD5Iubu\/1Ylo8\/ukVf29Mjpl4aCN+aahbmonKHahTqGZVLAhI8S0AMd58+fH3Sw5l0u+jI6lxcq+ajBSrtcen\/8daHylUUBo23e88Qz8oPur8mXftMTbIPWaaIp16wMfjYXKmeodqGOYZkUMOGDGx988MFAwMyZM0c2bdokGzduDI4JcHmhko8arLTLpffHXxcqX1kTMPnrWy6f\/A25+tKpsvOmK85yAlTOUO1CHcMyKWAKZWD27t1rdZhjlO4YlXzUYKVdUbw8vXtQ+cqCgAm\/v8Wsb9FFuZp1OTJySi6\/44mCIgaVM1S7UMewTAoY7RiiHOYYtYtGJR81WGlXVE9P5z5UvnwWMPnvbwmvbwl7QTERg8oZql2oY1hmBUySXS0q+ajBSruSjI7Kn4XKl68C5th9q0R3Femalhn3HxqXQBUxN\/c9J0dOnJJnNnwsKI\/KGapdqGNYpgRMftYlP\/LmzZsXHAfANTDj9knshOwg8qoUaueKapcvA32paSJbB88XMaicodpFAWPr6QmVCx\/GqI\/U3\/Uy5xu5bAYq+ajBSrtcen\/8daHylbaAMdNEJx65PbebqGru\/IrOJ1py39NBJmbDgqnymavHvismfk+J\/wmovog6hmUqA2PcN7wLyWRbCn3myt1RyUcNVtrlyvOTqQeVrzQETKFsi800UTlMq4gZPPaqfP0LH5Cr3zO1nFu9L4vqi6hjWCYFTHgXksm46CGNeup0R0eH8D0wdv0EarDSLjv+fSmFyleSAkbXtJx6dm9ubYvuIqo021LKP67tekJ++stTMnLXNb64kZN2oPoiBYwT93BXCXchVY4larDSrsp9I8kaUPmKW8Dotmd9S65ZkKs7iSbPnR9sgY77Us5u\/d5JefyFk7LzxitgMjGovkgBE3dEeFw\/KvmowUq7PA6mAk1D5cu1gNHpIX2lv4oWfcW\/\/m7elJv0gYqGs5v6nhM9zRpFxKD6IuoYlskppGK7kbgLqbyBCzVYaVd5fpB2aVS+XAgYs6bFTA9pnbbbn+PkNcxZ564h6dx1KDgAUg+CzPKF6osUMJ57pe5CmjlzptTV1TlvKSr5qMFKu5yHQKwVovIVRcCYLMvos3tl9Gd7xLwdd9I7a6Xq\/QvOOpMoVmJKVJ7PmWZhNBuTdRGD6ouoY1gmMzCF4oq7kMrvylCDlXaV7wtp3oHKl42ACWdY\/nBsKBAsJsuS5JqWcvkvxNnjz5+UJfc\/Lc2LZknzojcOfiy37jTLo\/oiBUyaXmXx7H379onuRLJ9kd3Bgwdl3bp1snnzZpk9e7bo\/StWrAie1N7ePuZ9MqjkowYr7bIIGI+KoPJVSMCY3UJhsZIvWN5yQW1F72pJgtpinBkRo1NJmo3J2oXqi6hjWCYzMMXWwOQLj2LBY7Zh\/\/SnP5Vt27bJ+eefL01NTdLa2hrc0tbWJl1dXbk3+qKSjxqstCtbwwYaXypSTr90OFhk+8ovfi6TXn4x+NlcuobF5+yKjfeU4syImGInWdvUn1YZNF80OKKOYZkUMJU6t66XOXbsmKiAUdFy\/PjxXPZG3yHT0tIiDQ0NufU0qOSjBivtqjRCkr3fR75UhJhLxYheRoRo9iT4\/aWhMcIkLFCCrMo7a+VU1flywUc\/HXyVhcyKLfPjcZZVETOeXbb4+FYOdQzLpICp5E28OnXU09Mja9askQ0bNuQETF9fX\/ASPL1UwNTX1+emkQb++i1j\/PHCd2V7pb0x5o9\/PC1vfvMk32Kt4vbQroohTLSCMXydGE702eM+bFrNG0WqX\/952kVnPptWI+fo9x++rmg1w8PDUlMTqmPcB2ajgI1dR18+LYt7huVDF02WB5Zmo8+0sSsbDIksXLhwTFMHBwez0nTrdmZKwJipn\/7+\/oIGLl68uOSbePX+O++8U1auXDlm2kgzMKUEDKp6Rf1rg3ZZx78XBVH5UnBRbbO1q9BJ1l44XZFG2Nrlsw2F2oY6hmVKwBhibHccabZl1apVwREDKm6uv\/76IPOiv5tr+vTp8rd\/+7fyzW9+M1gAzCmkrIXm2e1F7YRoV\/Z8k5yJZEnEoPJFAeNB32GEyy233CJr166V\/fv3j2lVOS+y07rMwl0u4vWAXIdNQO2EaJdDJ0moKnL2BtDmJOt7l1\/m7dEDqHxRwCQU8Ek9Jixg8rdR9\/b2jnkhHir5qMFKu5KKIjfPQeWLU0hn+4fvIgbVF1HHsExOIWlY6E6i9evXR87AlNP1opKPGqy0qxzvTr8sKl8UMIV9S0WMr4dAovoi6hiWSQFj3gPT3Nwcy9EB+WGHSj5qsNKu9EVJOS1A5YsCprgX+HoIJKovoo5hmRUwmzZtko0bN+ZeNldOh1luWVTyUYOVdpXr4emWR+WLAqa0X5lDIH06yRrVF1HHsEwKGDOFpP8vW7Ys9t4XlXzUYKVdsYeE0weg8kUBM76b+HYIJKovoo5hmRQwxY4SKGcX0vih9UYJVPJRg5V2lePd6ZdF5YsCxs63jIjx4RBIVF9EHcMyKWDswsJdKVTyUYOVdrnz\/SRqQuWLAsbee3w5BBLVF1HHsEwKmGIZGA0XfTGdHtCoW6NdXajkowYr7XLl+cnUg8oXBUx5\/uPD+Umovog6hmVSwJg1MENDQ6I7kcJrYmbOnBkcC3D33XeXFz0lSqOSjxqstMuZ6ydSESpfFDDlu0\/aIgbVF1HHsEwKmFKHOepbeu+55x4KGIu+AzVYaZcF+R4VQeWLAiaak+nRA5ff8YRcfelU2XnTFdEqiXgXqi9SwER0iDhuM4c66nRROAMzMDAg69atC841Mp+7eD4q+ajBSrtceH1ydaDyRQET3YfSOj8J1RdRx7BMZmA0LPLXwegOpHvvvVc2b94s9fX1TrdXo5KPGqy0K\/rAkcadqHxRwFTmTWERo++KmVE9ubIKLe5G9UXUMSyzAsbCF50VQSUfNVhplzPXT6QiVL4oYNy4T5LnJ6H6IuoYllkB09nZKVu3bh0TIXwPTHkdBmqw0q7y\/CDt0qh8UcC486ykRAyqL1LAuPPFimsKnyT91FNPie48Onz4cFBvHG\/mRSUfNVhpV8UhlmgFqHxRwLh1I3MI5Mhd17itOFQbqi+ijmGZzMCEdyEdOHBA9u7dK6tXr5a4zkdCJR81WGlXbP17LBWj8kUB495d4j4EEtUXUcewTAoY3YWkW6VVtBw\/flxWrVolR48eFU4hlddhoAYr7SrPD9IujcoXBUw8nmUOgbyv4TJpuPJCpw9B9UUKGKduUnll+\/fvl3PPPTd44+6+fftk7dq1zt\/Aa1qJSj5qsNKuyuMryRpQ+aKAic+L4joEEtUXUcewTGZgKgkL8w6Z\/v7+MccOqAhasWJFUHV7e\/uYtTSo5KMGK+2qJEKSvxeVLwqYeH0pjvOTUH0RdQybcAJGdy\/V1tYGAkVFi1k\/09TUJK2trUHEtbW1SVdXl1RXVwe\/o5KPGqy0K96Bw3XtqHxRwLj2lLPrcy1iUH0RdQzLlIApdYijuvZ4a2AKHUGg96mQUWHT3d0tVVVV0tLSIg0NDVJXV0cBE38f5PwJqJ0Q7XLuKrFXSM5ih1hcnp+EyhcFTPx+OO4T8gVMb29vTmSMe\/Prb+\/VnUp6haeQdCGwHgDZ0dERfKcCJvw2XyU\/fO3evdvmcd6XGR4elpqaGu\/bWW4DaVe5iKVbHpUvRRXVNt\/sOvryaVncMywfumiyPLA0+sJe3+yqJDIXLlw45vbBwcFKqvPy3kxlYMIIFjpKQDMoZtqnENoHDx4Mdix99atfDYTP9u3bRc9PWrp0qezYsaOkgEEkH\/WvDdrlZV9TtFGofKnBqLb5aJeLQyB9tMtFNDMD4wLFGOsITwMZEWMEi26xXrx4cXDQ44YNG4K1Lmb3kk4drVmzRrZs2cIppBj5SbJq1E6IdiXpRW6eRc7c4GhbS6WHQKLyRQFj60EJlQuLE32kChSdAtI1LKWu8CJek4EJCxu9l4t4EyIxpsegdkK0KyaHibFachYjuEWqDouYe5dfJle\/Z6p1I1D5ooCxdoH4CoanjcZbsFusFcXqCG+jzl9bg0o+arDSrvhiMI6aUfniFFIc3mJfZ5Tzk1B9EXUMy9QamEp3Idm7\/tiSqOSjBivtiurp6dyHyhcFTDr+FH6qOT9p541XWGViUH0RdQzLlIBJKxxQyUcNVtqVVqREey4qXxQw0fzB9V3lnJ+E6ouoYxgFjEW0oJKPGqy0y8KpPSqCyhcFjD9OZitiUH0RdQyjgLGIMVTyUYOVdlk4tUdFUPmigPHIyUTE5hBIVF9EHcMoYCxiDJV81GClXRZO7VERVL4oYDxystebYg6BbF40S5oX1Z7VQFRfRB3DKGAsYgyVfNRgpV0WTu1REVS+KGA8crJQU0qdn4Tqi6hjGAWMRYyhko8arLTLwqk9KoLKFwWMR06W15Ri5yeh+iLqGEYBYxFjqOSjBivtsnBqj4qg8kUB45GTFWhKIRGD6ouoYxgFjEWMoZKPGqy0y8KpPSqCyhcFjEdOVqQp+ecnofoi6hhGAWMRY6jkowYr7bJwao+KoPJFAeORk5VoSvjogX\/6\/IUya9asbDS8jFaijmEUMBZOgEo+6sBBuyyc2qMiqHxRwHjkZOM0xYiYwWOvyn\/e\/vHsNNyypahjGAWMhQOgko86cNAuC6f2qAgqXxQwHjmZZVOu7XpCjo2KlHsIpGX1qRVDHcMoYCxcCpV81IGDdlk4tUdFUPmigPHIySybor546\/dOypETp6BEDOoYRgFj4dio5KMOHLTAcRnvAAAgAElEQVTLwqk9KoLKFwWMR05m2RTji+UeAmlZfWrFUMcwChgLl0IlH3XgoF0WTu1REVS+KGA8cjLLpoR90fb8JMuqUy2GOoZRwFi4FSr5qAMH7bJwao+KoPJFAeORk1k2Jd8Xbc5Psqw61WKoYxgFjIVboZKPOnDQLgun9qgIKl8UMB45mWVTCvmiOT\/pvobLpOHKCy1r8qsY6hg24QTMwYMHZdWqVXL06FGZN2+edHd3S3V1tezbt09WrFgReF17e7ssW7Ys54Go5NMuvzqZ8VpDvsZDyL\/vyZl\/nJRqUTG+Sp2flAULUf1wQgmY0dFRaWlpkfr6+kCgdHZ2Br63evVqaWpqktbW1uD3trY26erqCoSNXqjk064sdD1vtJF8ZYsv9h1YfGVZxKD2HRNKwGg4qWipra3NCRj9eebMmcHnmo2pqqoKRE5DQ4PU1dU5ETA2zlOqTKHvxvss\/L3Nz1G6Ghu7SnXixe7P\/7zU74Vss21XMZtt76+Us6Ttsh1QadcZz2CMDZ6Fg8HENkbQYqzYIZBhO22wyWKMRRkj4r5nwgkYI2K2bt0qvb29gUjR6aO+vj7p6OgI8A5naUxHFjcRrJ8IEAEiQAT8R+BP575DXv5kp0z9343+NzbUwsHBM4IU6ZpQAmZkZEQaGxulubk5EC5mCmn+\/PklBQwS4bSFCBABIkAEKkNAjx6YUT25skp4d8UIQAuY8ILdxYsXy\/XXXy933XVXbn2LZl5UxKxZs0a2bNlSdAqpYpRZAREgAkSACBABIuAUAWgBk49UfgZm+\/btMjAwIOvWrZMNGzYUXcTrFHFWRgSIABEgAkSACFSMwIQSMIqWzTZqszamYnRZAREgAkSACBABIhALAhNOwMSCIislAkSACBABIkAEEkWAAiZRuPkwIkAEiAARIAJEwAUCFDAuUGQdRIAIEAEiQASIQKIIUMAkCjcfRgSIABEgAkSACLhAgALGBYqsgwgQASJABIgAEUgUAQqYROHmw4gAESACRIAIEAEXCFDAuECRdRABIkAEiAARIAKJIkABkyjcfBgRIAJEgAgQASLgAgEKGBcosg4iQASIABEgAkQgUQQoYBKFmw8jAkSACBABIkAEXCBAAeMCRdZBBIgAESACRIAIJIoABUyicPNhRIAIEAEiQASIgAsEKGBcoMg6iAARIAJEgAgQgUQRoIBJFG4+jAgQASJABIgAEXCBAAWMCxRZBxEgAkSACBABIpAoAhQwicLNhxEBIkAEiAARIAIuEKCAcYEi6yACRIAIEAEiQAQSRYACJlG4+TAiQASIABEgAkTABQIUMC5QZB1EgAgQASJABIhAoghQwCQKNx9GBIgAESACCAic+PYmp2acPjbktL78yi64aVus9adROQWMBeqXXHKJRSkWIQJEgAgQAZcILDr\/VXnX204HVb7rrX\/IVX3hW898Zi5TRn\/P\/85le8J1vfj7SU6r\/vXv3NaX37j\/\/oNRp+31oTIKGAsWVMAMDg5alCxcxOb+UmUKfTfeZ+HvbX6OYpyNXVpvsXK2n+eXG88e23YVs9n2\/ko5S9quUlyEsaBdZ9BgjJ3p8wrFm22MGL96Zc8\/yOmXDovJMjze\/7B8tK5OTr80lPusUDyqSKi5qCb31aR31gY\/v+WCWvnOd78jn7nuM8Hvky448\/mHWnrG9NWMsTNjV7l8RRkP0riHAsYCdVTyaZcF+R4VIV8ekWHZFGTODvz7YwEKo8\/uCf4\/9exe+cOxoYKixAiMsAAxwmPa5zZaohl\/MWS+KvkjPH7koz2BAsYCNzq1BUgeFSFfHpFh0RRUvvKzFxZQeF1EsyhGpAw9\/fiYqRoVKCpONDOiV1bXW6D6IqpdFDAWXQYq+YcOHZJZs2ZZIJCtIrSLfPmCQBZ9Uad5NKtixMqp1zMsJotSNXeBvPrOuTLrs2t9gdlZO7LIl43xqGMYBYwF+6jkowYr7bJwao+KoPKlEPtum1mTooLllT09EhYrJqMyee78ILMyee6CnNf4bldU90a1C3UMo4Cx8HRU8lGDlXZZOLVHRVD58lXAhDMsOi2kl2ZXNLNiO\/WDyhmqXahjGAWMRUeOSj5qsNIuC6f2qAgqXz4JGCNaTJbFrFmZfvsPI3kCKmeodqGOYRQwFuGLSj5qsNIuC6f2qAgqX2kLGBUtr\/ywR0Z\/putZ9pSdZSnlIqicodqFOoZRwFh05KjkowYr7bJwao+KoPKVpoA5evs1Y0TLlAUrx6xhqZR+VM5Q7UIdwyhgLCIZlXzUYKVdFk7tURFUvpIUMCbbcuKR23OZFteiJewyqJyh2oU6hlHAWHTkqOSjBivtsnBqj4qg8pWEgNGpIV3XootxdV3LlAVflCReDIfKGapdqGMYBYxFR45KPmqw0i4Lp\/aoCCpfcQoYFSwqXPRV\/OXsHnJFOypnqHahjmETTsDs27dPVqxYkYvj9vZ2WbZsmYQ\/N5+ZQqjkowYr7XI1TCVTDypfrgWMmSYyW5812zLlmpW5c4CSYevMU1A5Q7ULdQybcAJm+\/btMjQ0JM3Nzbl4HxkZkaamJmltbQ0+a2trk66uLqmurg5+RyUfNVhpV5JDWeXPQuXL1UCfv74lqWmiUsyicoZqF+oYNuEETGdnp9TW1gZZF3Np9kU\/7+7ulqqqKmlpaZGGhgapq6ujgKl8fEq8BtROiHYl7koVP7ASznwULgaQSuyqGNQYK0C1iwImRqdJqmrNtDQ2Nsr+\/fuDR86bNy8QLQcOHJC+vj7p6OgIPlcBU19fnxM5Sn742r17d1JNjvU5w8PDUlPzxlH1sT4swcppV4JgO3gUKl8KTSTbTgzLaz\/5rsijd4tMqxH58HVyzrW3OkDaXRWR7HL3+NhqQrJr4cKFY3DiadSxuU06Fet00sDAgCxdulR27NhRUsAgko\/61wbtSieeoj4VlS\/FoxzbfM645HNbjl1R\/SKN+1DtYgYmDW+K+Zlm6mjNmjWyZcsWTiHFjHdS1aN2QrQrKQ9y9xwbzrIkXAwyNna5QzG5mlDtooBJzodie5JOIW3atEk2btwYLNDVdS96rV69mot4Y0M9+YpROyHalbwvVfrE8Tg78e1NYl4+N+P+Q5U+LrH7x7MrsYY4fhCqXRQwjh0lrerC26XNGhgVM+HPe3t7cwt4tZ2o5KMGK+1KK7qiPReVr2JTSFnMuHAKKZpv+3IX6hg24XYhRXEoVPJRBw7aFcXL07sHla9CAiaccfFhO3RU1lE5Q7ULdQyjgLGIYFTyUYOVdlk4tUdFUPkyAubi884JToU2U0VZFi7GbVA5Q7ULdQyjgLHoyFHJRw1W2mXh1B4VQeVLp4qO\/NPXgu3QSZ5TlAS1qJyh2oU6hlHAWEQ7KvmowUq7LJzaoyJofKlw0ami4JX\/02rkghVfCQ5ZRLrQOEPPLKGOYRQwFr0KKvnshCzI96gI+fKIjLymqGgZff1kaHPA4pQFK+VX586UWbNm+dvwiC2jL0YELqXbUMcwChgLh0Iln52QBfkeFSFfHpHxelPG21FEzvzjrFSLUPlCHcMoYCziC5V81GClXRZO7VGRrPFlRMvoz\/bIqWf3BOtbpn1uY8FpoqzZZusWtMsWKT\/KoY5h3gsYfd3\/+vXrx3hBe3v7mMMY43YRVPLZCcXtOW7rJ19u8SyntvAUkREtuq6lau58mTx3QdGqyFk5KKdfFpUv1DHMWwFjXixXSKwYUZP\/wrm43B+VfNRgpV1xRUI89frKV6FMS9XcBXLBTdusgfDVNmsDihSkXZUimOz9qGOYlwJGX\/nf398vK1euLMlyT0\/PuGVcuAkq+eyEXHhHcnWQr3ixNlmWU8\/uPbODSCSYHipXtIRbSc7i5cx17ah8oY5hXgoY105ZaX2o5KMGK+2q1OOTvT8tvsKCRXcQ6e8qWCa9s1am3\/5DJyCkZZuTxpeohHbFjbDb+lHHMK8FjGZiGhsbZf\/+\/WexOX36dNm2bZvMnj3bLdMFakMln51Q7K7j9AHkqzI4Naui2ZU\/HBsKFt+aDIsKlqr3LwgW4rq+yJlrROOtD5Uv1DHMawGjrqrrXYaGhqS5uTnwXP1dr5kzZ0pfX5\/cfffd8Xo0D3OMHV\/XD0DthGjX+J6iwuSMQBkrVMJi5S0X1MrkufMTebkcORufM59KoPJFAZOCl2kGZtOmTbJx40bRE6P1Mp\/dcsstcs899zgTMOHTqPMXDqOSjxqstCuFYK3gkeXyZTIp+shwNsU0wUwDGaGi\/2uWRT9P+irXtqTbF\/V5tCsqcunchzqGeZ2BGR0dlZaWFtHponAGZmBgQNatWyff\/OY3c59X4hYqipqamqS1tTWopq2tTbq6unKiCZV8dkKVeE3y9yLxZRbJKorHDjwjU\/7wmwBQFSR66dtsdU1KocsIEV1cq5dmU\/Q6I1iKb2lOnjERJM7C+NGuNLwp+jNRxzCvBYzJuITXwcybN0\/uvfde2bx5s9TX1zt5H4xmXzo7O6W7u1uqqqoC0dTQ0CB1dXWBx6CSz04oeoeQxp22fJn1HaaNRhQUavPplw4XNKWYeChVp4oOc413vymnYuT06dMyadKkIEuiIsRcRphk+RwhW87S8KdKnkm7KkEv+XtRx7BMCZiHHnpIHnvssWDrtMvFuypgdD1NR0dH4FkqYMLiaPCz5yTvcXwiEUgAgaOTLiz6lF+9+Y3vPlQzeWy5aRedfd+0mtxn5+jPH77OyoLh4WGpqXnjXqubMlII1Tba5b8DLly4cEwjBwcH\/W90mS30WsCYKSQVE7qQd\/78M6liIzY0W+LiGk\/AoKpX\/hVVuff0PfnimEqOjJyquNIjI6MF63j11VflvPPOK1n\/Lyp4\/pETxdteiV0zqt8QPzOmnfn54tc\/m1FdJSdPnJCp06ZJ86Lk16hUTNY4FTDG4kbYbf2ofKGOYV4LmPAi3gcffDAQMHPmzDlrYW+lLswppEoR9Ot+1E4oq3Z17hq7lkUFWlhoDR57VY6+fLqoE6kAMsLnqvdMC8plRexklbPxIpp2jYeQX99TwKTAR6EMzN69e+Xo0aPBdI+rDAwX8aZAboyPZOcaI7gxVF2ML81uhTM\/P3r+RPD0x184eVYrjMhRgeOTuKEvxuAwMVaJyhcFTIxOU6rq\/JfZ6SJeXWxrtlW7alZ4G3X+GUuo5KMGK+1yFRXJ1OOCL83ymMxOWOCEhc1Vl06Vq98zNRmjXn+KC9sSbbDlw2iXJVCeFEMdw7yeQvKEe+5C8oUIy3awc7UEypNicfClmRv996PXszWavTHCRkWNihldfxN3tiYO23ygjXb5wIJ9Gyhg7LGquGSpIwS08riyMMUajko+O6GKXTXRCsiXG7g1W5MvaBqufHcgalxnaMiZG86SqgWVL9QxzPsMTLGjBJYtW5aUTzMDkxjSbh6E2gnRLjf+Ea5FszS61sYIGs3OuBQz5Mw9Z3HWiMoXBUycXlOk7lJHCYSPF4i7aajkowYr7Yo7ItzW7xNf4eyMETOVTDP5ZJtL1miXSzTjrwt1DPM6AxPehWQyLvrGXNe7kMZzH1Ty2QmNx7xf35Ov5PgwmZm+J38VrKVpXjQr0noZcpYcZy6ehMoX6hjmtYBRh0xqF1Ip50clHzVYaZeLrjy5OnznS6eYOncdCoTM1ZdOlXsbLpPwy\/lKIeW7bVFZpl1RkUvnPtQxzHsBkw7dY5+KSj47IR+8y74N5MseqzhKPv78Sbn54efKysiQsziYiK9OVL5QxzAvBYxmXfr7+4Mzj0pdPT0945Zx4eqo5KMGK+1y4fXJ1ZE1vnSdjGZkNAtz7\/LLSu5cyppttqzTLluk\/CiHOoZ5KWCUcvNiufb29rNOnNadSevXr5f8F87F5Sqo5LMTistj4qmXfMWDa9RajZBpuPJCua\/hsoLVkLOo6KZzHypfqGOYtwLGuK8RK2F3LiRq4nR3VPJRg5V2xRkN7uvOMl86rbTk\/qeLZmOybFsppmmX+ziIs0bUMcx7ARMnqbZ1o5LPTsjWA\/woR7784CG\/Fbq49+a+50RP886fUiJnfnJWrFWofKGOYRQwFvGFSj5qsNIuC6f2qAgKX0vuezo4rmDnjVfk1sWg2JbvLrTLowCyaArqGEYBM4HJZydkQb5HRciXR2QUacpNfc8Fb\/YdueuaoAQ585+zcAtR+aKAyZYfFm1t+NRpLWTW04Q\/z19jg0o+arDSrmwFKxpfZnGvZmIuevMJmTVrVrYIsWgtGmfGZFS7UMcw7zMw4RfZPfTQQ\/LYY48FW6dnz55tEWZnF8k\/W0lL6DOampqktbU1uKGtrU26urqkuro6+B2VfNRgpV2RQiO1mxD5MpmYB5ZeKJ+5uvAOpdQAd\/BgRM6QM2aoY5jXAiZ8lMDQ0JDMnz8\/CL2+vj7p6OiQqqqqskNRjyKora0dszVbsy\/6eXd3d1BnS0uLNDQ0SF1dHQVM2QinfwM71\/Q5KKcFqHzpmpjBY6\/Kf97+8XLgyERZVM5Q7aKASSGswoc5Pvjgg4GAmTNnjmzatEmiHOZY7FiCAwcO5ESRmqkCpr6+PidylPzwtXv37hTQcP\/I4eFhqampcV9xyjXSrpQJKPPxqHwpDCv7Dsvb3vY20UwM0oXKGZJdCxcuHONyg4ODSC4Y2JK5DMzevXudHeao00kDAwOydOlS2bFjR5DVKSZgEMlH\/WuDdmWrn0LlS1n40f6DsrhnODhDaedNV2SLmBKtReUM1S5mYFIKvUoOczx48KCsWrUqEDyLFy8+a9rJTB2tWbNGtmzZwimklDh2\/VjUToh2ufaU+OtTzn75x2nBy+7C26vjf3K8T6Avxouv69opYFwjalmfWXS7evVqaWxslP3790c+QiA8JaULdHXdi15aNxfxWhKSgWLsXDNAUqiJqHypicY2XQ+jL7p7ZsPHskVOkdaicoZqFwVMCmGni3jvvPPOYNfRU089JbqQV6d79BDH2267LdIi3vB26Xnz5gVZFxUz4c\/zz1hCJR81WGlXCsFawSNR+QoLGP358juekIYr3y3Ni2orQMuPW1E5Q7ULdQzzeg1M\/iJe3T107bXXRl7EGzX0UclHDVbaFdXT07kPla98AWPOTUKYSkLlDNUu1DHMawFjMjCf\/vSnZevWrbn3tFSSgYnSRaOSjxqstCuKl6d3Dypf+QJGf9epJL2yvqAXlTNUu1DHMK8FjAa6mdq54YYbxqxVifoiuyjdNCr5qMFKu6J4eXr3oPJVSMDoZ9Vf\/qHc13CZNFyZ3a3VqJyh2oU6hnkvYNLrVt94Mir5qMFKu3yIGvs2oPJVTMCYowbMeUn2SPlTEpUzVLtQxzDvBYzuFNLpo\/AVXnybREijko8arLQriahw9wxUvooJGP086wt6UTlDtQt1DPNawITPKNJdSDNnzpTDhw8HPeeyZcvc9aDj1IRKPmqw0q7EQsPJg1D5KiVg9MRqPS9Jt1XPqJ7sBMckK0HlDNUu1DHMewFjjg3Q1\/3rW3j1nS1RjxKIGuCo5KMGK+2K6unp3IfKVykBY7IwV106NVgPk7ULlTNUu1DHMK8FjO5CuueeewLRcvz48dxbdTmF5Ka7Qw1W2uXGP5KqBZWv8QRMlrMwqJyh2kUBk1RvlvccffPuueeeK7rrSHckrV27VrZt2xb8ntSFSj5qsNKupCLDzXNQ+RpPwOj3uq364urJmcvCoHKGahfqGOZ1BsZN91h5LajkowYr7arc55OsAZUvGwFjsjBZe7kdKmeodqGOYd4LGD0Laf369WP6U04huRleUIOVdrnxj6RqQeXLRsBkNQuDyhmqXRQwSfVmoeeYk6ibm5ulrq4uhRaceSQq+ajBSrtSC5VID0bly1bAZDELg8oZql2oY5jXGZj806Mj9Y4ObkIlHzVYaZcDp0+wClS+bAVMFrMwqJyh2oU6hnktYDSwdQpJr6jvfdH79RRrzeLoZbI6ujh48eLF0tHREZxqHT6Nur29fczzUMlHDVbalaD6cPAoVL7KETBZy8KgcoZqF+oY5qWACYuMQv2j7RoY8xZfPUfJCBj9TE+1XrJkibS0tEhDQ4PMmTNHmpqacodFtrW1SVdXl1RXV3MKycEAlXQVqJ0Q7Urakyp\/XjmcZWlHUjl2VY5icjWg2kUBk5wPOXmSZl70zb368ju9VMDkr6kx2Zn58+eLCpvu7u4gG2OEjVl3g0o+arDSLichlFglqHyVk4HRslnKwqByhmoX6hjmZQZGg\/ngwYO5F9eFp3rK7VVVmIQFjMm06HtkVMAMDAzI0qVLZceOHcF0kl4qYOrr63PTSEp++Nq9e3e5zfCy\/PDwsNTU1HjZtkoaRbsqQS\/5e1H5UiTLte1LO14MCHhgqd8nVZdrV\/JeFe2JSHYtXLhwDAiDg4PRQPH4Li8FjL6BN5wFMdM+UdbBuBIwiOSj\/rVBuzzucQo0DZWvcjMwWv7x50\/KkvufFt\/fC4PKGapdzMAk2Cfm7z7SbMyuXbvk5ptvLtgKI3j6+\/tl+vTpY97Umy9gGhsbg+kknR7iFNIhmTVrVoLMJvMo1E6IdiXjPy6fEoUzXQuj186brnDZFKd1RbHLaQNiqgzVLgqYmBymULWFBExPT4\/cdtttwRqVcq6wgNH7uIj3DfRQg5V2lRMh6ZdF5StKBkbvOTJySi6\/4wmvszConKHaRQGTYD8Xp4AJ73AK704Kb6Pu7e0d8+I8VPJRg5V2JRisDh6FyldUAaP33dT3nPzohZPyzIaPOUDYfRWonKHahTqGebkGxtU2aldhi0o+arDSLleen0w9qHxVImD03uov\/zA45LHhSv8W9KJyhmoX6hjmpYBJptu0fwoq+ajBSrvsfduHkqh8VSpgjIgZuesaH2ga0wZUzlDtQh3DKGAsugZU8lGDlXZZOLVHRVD5ciFgdC3MjGmTvVvQi8oZql2oYxgFjEVHjko+arDSLgun9qgIKl8uBIyv26pROUO1C3UMo4Cx6MhRyUcNVtpl4dQeFUHly4WA0Tp0W\/XjL5wUn6aSUDlDtQt1DKOAsejIUclHDVbaZeHUHhVB5cuVgNF6dCrpqkunBot6fbhQOUO1C3UMo4Cx6A1QyUcNVtpl4dQeFUHly6WA8W0qCZUzVLtQxzAKGIuOHJV81GClXRZO7VERVL5cChgzlXTkxCm5d\/llcvV7pqbKICpnqHahjmEUMBbdACr5qMFKuyyc2qMiqHy5FjBhEZP2C+5QOUO1C3UMo4Cx6MhRyUcNVtpl4dQeFUHlKw4Bo8cM6GGPaW+tRuUM1S7UMYwCxqIjRyUfNVhpl4VTe1QEla84BIzWac5KuvrSqam9HwaVM1S7UMcwChiLjhyVfNRgpV0WTu1REVS+4hIwhjo9akCPGUhjZxIqZ6h2oY5h8AJm+\/btMjQ0JM3NzUHchw9t1N\/b29tl2bJlYz43n5mOApV81GClXR6pE4umoPIVt4AxO5PSyMSgcoZqF+oYBi1gOjs7ZevWrRI+dTpf0Ggno4dHNjU1SWtra9DdtrW1SVdXl1RXVwe\/o5KPGqy0y0I1eFQEla+4BYzWn5aIQeUM1S7UMQxWwKhQmTlzpuzduzfoqk0GRkVNbW1tkHUxl2Zl9PPu7m6pqqqSlpYWaWhokLq6OgoYjwY626agdkK0y9YD\/CmXBGdmTcyM6smJbbFOwq40WES1iwImDW9y8EwVJkbAaKalsbFR9u\/fH3w2b968QLQcOHBA+vr6pKOjI\/hcBUx9fX1O5KCSjxqstMtB4CRYBSpfSWRgDE0qYm7uey44cqB50SxpXlQbK4OonKHahTqGwWZgTPSGBUx+RGuWZmBgQJYuXSo7duwoKWDC9+7evTvWziGpyoeHh6WmpiapxyX2HNqVGNROHoTKl4KTtG0P\/PtJ2frjkzL97ZOkf2V8sZ20XU4czaISJLsWLlw4xuLBwUELBLJVBELAjI6OBlmT\/v5+mT59umzbtk1mz54dMFFKwJipozVr1siWLVs4hZQt3y3aWtS\/omhX9hw0Dc6SyMakYVcS7KPaxQxMEt4TwzPyp5A2bdokGzduDBbomu9Wr17NRbwxYJ9WlaidEO1Ky6OiPzdNzjp3DUnnrkOia2Ncv7k3TbuiszH+nah2UcCMz72XJfIzMOFt1GYNjIqZ8Oe9vb25BbxqFCr5qMFKu7wMxQmXMVODffDFsJBpuPLdwbtjVNRUcvlgVyXtL3Yvql2oYxjEFFIcjhyuE5V81GClXXFHhNv6UfnyRcAYtlTI9D35q+BNvipgVMxEXeyLyhmqXahjGAWMRV+MSj5qsNIuC6f2qAgqX74JGEO5Cpi+J18Mppf00l1LZ\/6337mEyhmqXahjGAWMRUeOSj5qsNIuC6f2qAgqX74KmHwh86PnTwTbr\/WyXS+DyhmqXahjGAWMRUeOSj5qsNIuC6f2qAgqX74LmLALaFZG\/\/3ohZO5zIwRNDrVdGbK6cLcLaicodqFOoZRwFh05KjkowYr7bJwao+KoPKVJQGT7w5mmkk\/z8\/QzJg2WeaeLzJ12rSypp08crmiTUH1RdQxjALGIqpQyaddFuR7VIR8eUSGZVPQONOFwHp1fesHcvod782hYHY1qbi56j3Tyl5TYwln7MXQ+DKAodpFAWMREqjk0y4L8j0qQr48IsOyKeichaeeTLZG\/zdrasIwhUXOxdWTZUZ1VfB1\/vSUJbSxFEPnKxbQUqyUAsYC\/Eqd2ub+UmUKfTfeZ+HvbX62gOGsIjZ26U3Fytl+nl9uPHts21XMZtv7K+UsabtKcRHGgnadQYMxdubV84XizTZGwn5lsjdHRkblke\/vlbqPflSOnDiz9qbUFX5njWZ49FIB9N3vfleuu+66nBDSz7eu+QsJvzKfMXZJgEcUvqKMCUnfQwFjgbiSz4sIEAEiQASSR+DU+\/4699A\/nXt+8POfzn3HmIaYzwt9F1eL3\/Tb\/+e86jf99rjzOk2FL\/3j\/4yt7oE83OQAAAquSURBVLQqpoBJC3k+lwgQASJABDKLgMkouTRAs1NxXfc1XBZX1anVSwGTGvR8MBEgAkSACBABIhAVAQqYqMjxPiJABIgAESACRCA1BChgKoR+YGBA\/uzP\/kz0YMisX6Ojo\/LII4\/If\/3Xf8n1118vs2adecU4ynX69Olg4d8nP\/lJmTbtzFbPrF8nTpyQb33rWzI8PCzLly+Xyy+\/POsmBe1\/+eWXA7vUvpUrV8pFF10EYZcxYufOnXLxxRfLFVdcAWHXK6+8It\/+9rcD3tSuJUuWyFvf+lYI27Tf+M53viM\/+clP5POf\/zwMZz\/4wQ\/kueeeCzh629veJkuXLpULL3zjZYVZII8CJiJL6tTaCf3Lv\/yLrF69eszp1RGrTP22f\/3Xf5Xzzz9fZs+eHQweX\/ziF6Wq6sxWx6xfv\/nNb+TBBx+UoaEh+bu\/+zvRE8gRru3bt8v73ve+4N8DDzwgX\/jCFyDE2fe\/\/32pra2Vd7zjHYEv3njjjTAD4oEDBwJf1B00dXV1CG4oatMLL7wgf\/VXfwVhT9iIf\/u3fwsG+I985CPyve99T\/7yL\/8Spl9UO5W7H\/\/4x4E4O+ecczLFHwXM63Tt27dP+vr6pKOjI3BOzUa0tLRIf39\/kF3p7u4eM+ipgPntb38rP\/vZz4IafO6IRkZGpKmpSVpbWwNxopcOfOvXrw9+7u3tDdqvA6B2QPoXVE9PjyxevNj7gd7WNuXzT3\/6U\/CXFJJdprfRTMU3vvENueGGG+S8887zthOy5UsN0L\/mdfB4y1veIp\/+9Ke97lxt7Xr11Vfln\/\/5n+Vd73pXwJPP\/YZyYGvXY489Ftj1pje9Sf7mb\/5GPvGJT3jNVzm2aVypgHn22WeDPxDe\/\/73extf5dilZX\/3u98FYnrZsmVywQUXeG1XocZRwIQGcx3YjIDRAV7\/Wm9ubpbOzs7gr0ENzP\/4j\/8ISNe\/ePWvQxU+PguYgwcPyqpVq4I2btu2LRAw+llbW5t0dXUF6tsIN\/1fMVBHzoKAKcc2k0lCtEvFyz333CMNDQ0yZ84cbzuhcvn6\/e9\/L\/pqd83GqDDzNRtoa1d7e7vs2bNH3vve98qxY8e87je0cbZ2aZ+pQmfKlCly7rnnypYtW+Szn\/2s19MR5dh2\/\/33y8c\/\/nH5wAc+IFu3bg2mNH2dgi7HLo2np59+Wo4ePSqf+tSnvO03SjVswgsYFSCHDx8OMNL1LBqMemn2pb6+PlCmJjujg752qq+99lrw15PO8fosYLRTUXWtKc\/bb79dNm\/eHAgYFWfGVs1MmOyM\/oWhf12oWNOBXm1\/+9vf7qVjl2ubyTz5LmDKtevP\/\/zPA451us\/ndSLl2vXrX\/868EUdKL7+9a8Hayp8tK8cu9auXRusOfjFL34hv\/zlLwPbbr31Vi8zZuXYpZldtUn50sySChj9Q0gzuT5e5dr2+OOPy1VXXRX0nfpHoP4h6+MUdLl2qT3aH+o4Z\/pHH\/migLFgJTyoGwGjf9FqildFimZh8qeRtJzPAsaYrap83bp1YwSMyS6p0zc2NgaZJu1wdBpJO1Z16CyoclvbTKredwFTDmfKqf4FpYOhZgNVzHzuc5\/zckAsxy71RR0gNCb1DwX9WefnJ02aZBHJ6RQp1w+z0G+YLIxN36F\/6Chf2neoL+qCcp\/5Ksc2FWUqXDQzPXny5CCj7bNttr6oC8g1u6R\/\/PiaURovmid8BsYAFFXAjAewD9\/bOrTv8\/GFsES1jXbVBWuW9J\/Pg0W5wixrMYbqh+UIGHLmwyhWuA0UMK\/jUkjA5E8hmfUx\/tJZuGWFOqFCU0hZTCOi2ka7ziw2z8pFvrLFVzEBg9Avovpiob6AAqaAgNHFTYUW8eqakCxe+Q5dbBGvr4skS2GOahvtytb2ffKVLb4KCRiUfhHVFylgSoyE4QxM\/jbq8O4kBAGjNpht1NOnT8\/tTqJt\/iCQ3wmhcEa7\/PExm5ag8lVIwDDGbDzCrzLMwPjFB1tDBIgAESACRIAIWCBAAWMBEosQASJABIgAESACfiFAAeMXH2wNESACRIAIEAEiYIEABYwFSCxCBIgAESACRIAI+IUABYxffLA1RIAIEAEiQASIgAUCFDAWILEIESACRIAIEAEi4BcCFDB+8cHWEAEiQASIABEgAhYIUMBYgMQiRIAIEAEiQASIgF8IUMD4xQdbQwSIABEgAkSACFggQAFjARKLEAEiQASIABEgAn4hQAHjFx9sDREgAkSACBABImCBAAWMBUgsQgSQEBgdHZWWlhbp7+8fY9YNN9wgzc3NmTZVz+7ZtWuXNDY2BjaaE+XVKGN3Q0OD1NXVnWWnfn\/PPffI6tWrpbq6OtM4sPFEYCIgQAEzEVimjUQghIAZyMODOwJAYQGiB7KWK2AUAyOAbr75ZgRIaAMRgEaAAgaaXhpHBM5GoJSAMaeyHzlyRJYvXy4f\/OAHZdWqVXL06FGZN2+edHd3B9mJffv2yYoVK0RPM1+wYIFMmTIlyFxo5kOzOJrh0LqGhoaC383p59oak+nROrZu3Ro0cO\/evRI+9b2zszP3XW9vb1Cmr69POjo6gp9VnORnUrS+w4cPy7Jly3LZlmIZGH2eebbWF372vffeK4sWLZLZs2fTfYgAEfAYAQoYj8lh04hAHAgUmkIy4uTRRx+Vhx9+OBAqejU1NUlra2swmBtBEhYqep+KCRUyxQTM\/PnzC4oPrX\/t2rWybds2Of\/883PiRz9XAaNtOH78uLS1tcltt90mf\/\/3fy8bN27MfdbV1TVmqkfv0WepeCo2TaZ1qyAKTyGF79Pv1U69VAjxIgJEwF8EKGD85YYtIwKxIGCTgdFMx\/DwcC77YhqigmXNmjWyZcuWXDamkLAJZ2Bqa2tl\/fr1Y2zRLIyKDSNUzJSPZlU0i2IyN+GbjNDQzzSDEl6vozbdeeedsnLlykBsFbKx0BqYfPGidWsmJ7\/+WIhgpUSACFSEAAVMRfDxZiKQPQTKETCa\/cjPdOgAb4SHTifZCJhCgiRcj42AMcJCETeZFoN+FAFTLNNCAZM9n2aLJyYCFDATk3daPYERsBUwWk7XtOhaGJ1OMetj1q1bJ7rIVTMg4SmkW265JbdwdsmSJbmpJRUbZqqopqYmV2bmzJkFMzBKTXgKSZ+3efNm0Xt1l9DPf\/7zs0SVuSd\/CqnYGhjN8uhVaJqIU0gTODhoeqYQoIDJFF1sLBGoHAFbAaNZEd2VY7uIN7xYN7y4t9Qi3kJTSGb6yUw7tbe354RGeGFwPhLhzEmpKaRPfepTwRTY\/v37c1WYNUBqc3gqqnK0WQMRIAJxIUABExeyrJcITBAESokKlxAk8R4XbqN2yRjrIgLxIkABEy++rJ0IwCOQhIAZGRkJprNmzJgRbKXWTEmhq5L1K\/nraOCJo4FEIOMIUMBknEA2nwgQASJABIjARESAAmYisk6biQARIAJEgAhkHAEKmIwTyOYTASJABIgAEZiICFDATETWaTMRIAJEgAgQgYwjQAGTcQLZfCJABIgAESACExEBCpiJyDptJgJEgAgQASKQcQQoYDJOIJtPBIgAESACRGAiIkABMxFZp81EgAgQASJABDKOAAVMxglk84kAESACRIAITEQEKGAmIuu0mQgQASJABIhAxhH4\/7rfr4N\/zmLTAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:4a13890b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:3ae14074]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:7858ff8a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:8b993c89]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:4f0a8057]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:2be5a5f7]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:0aedf096]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:93d9fff2]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:09c73bc1]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:2788be2b]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:1ed1daa1]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:3b2d4c40]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.012443765785704"}}
%---
%[output:6142afe8]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.517590710054527"}}
%---
%[output:513fbeb6]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.152987786896029"}}
%---
%[output:72f2672f]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"  93.745795204429072"}}
%---
%[output:5b8b0b72]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -1.166194503484206e+02"}}
%---
%[output:6705b52e]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10XsV55x8SuosICUZ8xCiKY8eRCWlagQmporh1qEppcirT5CSV5d1N6qqpewpJt+AjyXHCrgvBso6hzRLSKqyidfdUstIWNvZmezgcl9BQxQ0xoH6RIBC2I4TDh3ECQQml8Z7nOiNGV\/djZu4873vv6H\/P4WDpfea5M79n3pm\/5vO0kydPniQ8IAACIAACIAACIFAhAqdBwFQoWsgqCIAACIAACIBARAACBhUBBEAABEAABECgcgQgYCoXMmQYBEAABEAABEAAAgZ1AARAAARAAARAoHIEIGAqFzJkGARAAARAAARAAAIGdQAEQAAEQAAEQKByBCBgKhcyZBgEQAAEQAAEQAACBnUABDwROH78OPX09ETehoeHqbGx0ZPnxW6mpqZo8+bNdNlll9HAwAA1NDSIvUt3XMsymhRIcfjEJz5BXV1dJklKbbNr1y4aGhqK8rhlyxbq6+uzyu\/4+Dht27aNdu7cWUoeXL6DBw+Kfz+soMG4sgQgYCobOmS8bARq2bmnCRjuINavX09tbW0ieNLKKP3etMK4dIjcgd53331W4sAljW0A+B2bNm2aTxaigAlNcNrGGPZ+CUDA+OUJbyBQFwJzc3PU399P+\/fvp9HR0ZoJGB75qcV7k6CqzrCzs9NYjKgRChtx4JLGpRIoAWOTt\/h7yj4Co+rp0aNHMQrjUkmQZgEBCBhUiFIS0IfSOYP6kLg++nDJJZfQjTfeGJWBOzJ9OkWNFkxOTi76XPdx9dVX0+\/8zu9ENk1NTTQyMkItLS2JXHShELePj07w52pKqaOjg2699VZqbW2NGm6948\/zw1NR6r2HDh2K8sePmkK64YYb6I\/+6I8i8aKeOAv+vWKqCxzVaer2qhNUvvQOVS\/j5z\/\/eRocHEx878zMTJS\/2dnZ+TzpMdQ5MvPbb7+dvvSlL5EqH\/OPs1bs1NRcUmet4qq\/V5U3Xi4V6+bm5nkRFue3b9++aEpGPXr9iPvLE47x+pjlK6seZo3U6Ew4zyrvcVEU\/37pbJWP6667jg4cOED8\/VGx08vMv1Pv0GObxyWpHpayEUKmSk8AAqb0IVpaGYx3WnrpVSOc1EnFOx72w+JBiZf450kdbFbnz5+l5U11Nueee+6CNTBKwOh5YKGQJDh0ERP340vAJP2FH+9M4h1bGlf+fZqA6e3tpWuvvXYR+yzBwJ+df\/759Mwzz0QCLUlU8Dv1jjae97R6od774IMPJoqRO++8c37diV7f9A46LmDivtTnaSImq85ymiNHjqQKJT1PcfESF5lKPPziL\/4iff3rX1\/QeCSJkKTvV1yAsE1SHvn36j15vnUuZR8lWlotbrVLCwFT7fgFl3vVQOt\/garGnwurjz7wX9mqYdQ7CL2x1f\/y1Ds8FglqhED5UO+O\/6WvICd9rjfGV155ZaqA0f9CtfWTJ2B41ImfvKmcrBEiHhV67rnnFjHRRw2Y05o1axaU0WQKKT69pdirePJoSzzunBdeD5I0MsQsN2zYEJVXH7ExWdhsMh0Ut4n\/rJgoscX5z3u3qntJ5VG\/Y6HLZU6bQtI5qvoUj+k999wTCaGkEZU0v\/FRODXqpPvIercaoVH1P4+Lj6my4Bo+FMiJAASMEzYkkiKQ9teZ6gC44V67dm3iDhzd5vDhw4l\/VXO+dR\/8V7\/aMZS3CDfvL8c0gaA36Px+Wz++BIz+bhYj\/OgdZlrHonfgH\/\/4x40FTNKIVdJ7OR\/xKbK0EQ625Y5Y5UNnm\/S++FRaloBJmzqLp8kaTUkSv\/GyqenJuBBSoi1NaOTVTz2+uo+0uMZHcxQrJWDSpg71HXZ6XVbfS336TrUTOpekaUup9gR+wyYAARN2fCtXuloLGH0bcl4HYSs8GH7StmpTP3rnHO\/s2Le+jdpkBIZt9IWv\/DNv2Y2PQMU7UFsBE+cYH6WJCydfAkZV9jThxDuzkgRMfCoqbwSmCgImacRPxTVe\/9JGYHQfad8NCJjKNbFBZRgCJqhwVr8wrlNI8akOtaYg7a\/ZpCH\/PAGTNfWjjwpwFPiv1DQBY+qHh+bj4kJNrcUFDIsEk8WR8c5dH6GIT8Nxh583hcSjQ3kCIO7DdQpJr91poxrxb0BcjMRHI1SZ9ZE4VR5Vd+JpkqaQ8r550lNISuyqkas0AZM0cqUYxUdg0hZdx6evsqaQkrhgCimvtuBzUwIQMKakYFcTAtKLeLMEQJ6Aycpb0vqQNAGT54eH29V6ljh0EwHDaZJ2ISlf8Z0k+gFwNot41VSCnobf+6EPfSgaHUp6mFNS+UwX8bJPJeriwiltgaueRrfhd37uc5+jm266adGCY04TFzD8u7QFwaqseYI5aXolbwRM55hWxizxoQuGT37yk6l1K8sH5yFpca\/pIl6dS94IZE0aGrwkCAIQMEGEMbxCmG6j1rdA522jTloYbDOFxJSzpifyFsnqJ\/Nm+eH3xLfcfuYzn6GHH354ftFq0giM3rmlLUTWfcfX5iQJHL0j19Pyv5WASXrvHXfcseBEWT5cT19v47KNWhcieoeatMU+bft2nKu+Jod9Mrfdu3fT1q1bIxz6SJraTZa2LTvv\/JasbdT8LtORibS1KzwKlyQO0kadmFHSFvakUZw08cu\/j5\/8m7aWSPkwGSkMr0VDiSQIQMBIUIVPUQJ5Oz5EXw7nhQkkTVXFd5qlncOjv9zlILvCmV+iDnTBqYRa0s6kPDw4yC6PED63IRCkgOGGjc+i4EO28hpCG1sbsLCVIwABI8e2Fp6zptCypr6S8uZylUAtyhjiO5KmkLiceYc\/JonOUO6uCjHOVSpTcALGdHEfB8nGtkpBDT2vEDDVj3B8OoVLZCteOA3u1qltXYhP7dqIF84pBGdt4xX624ITMDzfy18SfvJGYGxsQ68IKB8IgAAIgAAIVIlAUAKG\/6rbsWMHdXd3RyImS8DY2FYpoMgrCIAACIAACCwFAkEJGJ6j5YdPhMxbA2NjuxQqAsoIAiAAAiAAAlUiEIyA4bnwPXv20Pbt24kv6ssSMDa2HMy3vvWtVYop8goCIAACIAAC8wT4VvFVq1YFRyQYAcNTRnzWBJ8emrezyMZWCZjp6enggl\/vArEwBFeZKICtDFe0B3JcwVaObajtQRACJmlHg6oK8evtbWyVj1CDL\/d1MfMMrmacXKzA1oWaWRqwNePkYgW2LtTy04TKNQgBEw9f3giMbm9iG2rw86u9rAW4yvEFW7CVIyDnGfVWhm2oXJeEgFHnvfDuJJ5igoCR+ZLYeg31S2XLQcIebCWonvIJtmArR0DGc6h1NkgB47sKhBp835xs\/T3xxBNBLiyz5SBhD7YSVE\/5BFuwlSMg4znUPgwCxqC+hBp8g6KLmqAjkMMLtmArR0DOM+qtDNtQ+zAIGIP6EmrwDYouaoLGSg4v2IKtHAE5z6i3MmxD7cMgYAzqS6jBNyi6qAkaKzm8YAu2cgTkPKPeyrANtQ+DgDGoL6EG36DooiZorOTwgi3YyhGQ84x6K8M21D4MAsagvoQafIOii5qgsZLDC7ZgK0dAzjPqrQzbUPswCBiD+hJq8A2KLmqCxkoOL9iCrRwBOc+otzJsQ+3DIGAM6kuowTcouqgJGis5vGALtnIE5Dyj3sqwDbUPg4AxqC+hBt+g6KImaKzk8IIt2MoRkPOMeivDNtQ+DALGoL6EGnyDoouaoLGSwwu2YCtHQM4z6q0M21D7MAgYg\/oSavANii5qgsZKDi\/Ygq0cATnPqLcybEPtwyBgDOpLqME3KLqoCRorObxgC7ZyBOQ8o97KsA21D4OAMagvoQbfoOiiJmis5PCCLdjKEZDzjHorwzbUPgwCxqC+hBp8g6KLmqCxksMLtmArR0DOM+qtDNtQ+zAIGIP6EmrwDYouaoLGSg4v2IKtHAE5z6i3MmxD7cMgYAzqS6jBNyi6qAkaKzm8YAu2cgTkPKPeyrANtQ+DgDGoL6EG36DooiZorOTwgi3YyhGQ84x6K8M21D4sSAEzNTVFvb29NDg4SC0tLYtqxPHjx6mnp4cmJyejzzo7O2lgYIAaGhoSa0+owZf5qph7RWNlzsrWEmxtiZnbg605K1tLsLUlZmYfah8WnICZm5uj\/v5+OnToEI2MjCwSMOrz9vZ26urqIvVzU1MT9fX1QcCYfR+8WKGx8oIx0QnYgq0cATnPqLcybCFgZLh693rw4EHatWtX5DdtBCb+0vHxcZqYmEgdhQk1+N7hWzpEY2UJzMIcbC1gWZqCrSUwC3OwtYBlYRpqHxbUCAxPDe3YsYO6u7sjEQMBY1HD62CKxkoOOtiCrRwBOc+otzJsIWBkuHr1yiMp\/KxduzZzDYz+UrUeZuPGjdGUUtITavC9wndwhsbKAZphErA1BOVgBrYO0AyTgK0hKEuzFe9+Px395t9Ypiq\/eTAjMLxwd8+ePbR9+3aamZkxEjBq\/QuHKW8Rrx7KAwcOlD+yFcghx6m5ubkCOa1eFsFWLmZgC7ZyBPx47ujomHf08or30ktrf5uO33qFH+cl8hKMgOEpo\/Xr11NbWxvl7UJi\/qbihW0xAiNTY\/HXlgxX9gq2YCtHQM4z6q1\/tmMPHKNrxh6BgPGP1o\/H+LZo3evo6GgkavTHZOeRbg8B4ydOcS9orGS4QsDIcQVbsJUl4N87BIx\/pqIe80ZgeLRmdnY2c9oIAkY0RJFzCBg5xmALtnIE5Dyj3vpnCwHjn6mox7iAUSMuvDtpzZo1Cw6xUxlpbW2l4eFhamxsXJQ3jMDIhAuNlQxXiEM5rmALtrIE\/HuHgPHPtFIeIWBkwgUBI8MVnawcV7AFW1kC\/r3vuvsw7br7CayB8Y+2Gh4hYGTiBAEjwxWdrBxXsAVbWQL+vUPA+GdaKY8QMDLhgoCR4YpOVo4r2IKtLAH\/3iFg\/DOtlEcIGJlwQcDIcEUnK8cVbMFWloB\/7xAw\/plWyiMEjEy4IGBkuKKTleMKtmArS8C\/dwgY\/0wr5RECRiZcEDAyXNHJynEFW7CVJeDfOwSMf6aV8ggBIxMuCBgZruhk5biCLdjKEvDvHQLGP9NKeYSAkQkXBIwMV3SyclzBFmxlCfj3ztcI8FkwuAvJP9tKeISAkQkTBIwMV3SyclzBFmxlCfj3DgHjn2mlPELAyIQLAkaGKzpZOa5gC7ayBPx7h4Dxz7RSHiFgZMIFASPDFZ2sHFewBVtZAv69Q8D4Z1opjxAwMuGCgJHhik5WjivYgq0sAf\/eIWD8M62URwgYmXBBwMhwRScrxxVswVaWgH\/vG25\/iO5\/\/AQW8fpHWw2PEDAycYKAkeGKTlaOK9iCrSwB\/94hYPwzrZRHCBiZcEHAyHBFJyvHFWzBVpaAf+8QMP6ZVsojBIxMuCBgZLiik5XjCrZgK0vAv3cIGP9MK+URAkYmXBAwMlzRycpxBVuwlSXg3zsEjH+mlfIIASMTLggYGa7oZOW4gi3YyhLw7x0Cxj\/T0nicmpqi3t5eGhwcpJaWlsR8QcDIhAsCRoYrOlk5rmALtrIE\/Hu\/5KZv0NHjP8IuJP9o6+txbm6O+vv76dChQzQyMgIBU+NwQMDIAQdbsJUjIOcZ9dYvWxYuLGD4wV1IftnW3dvBgwdp165dUT4wAlP7cKCxkmMOtmArR0DOM+qtX7YQMH55lsbb8ePHaceOHdTd3R2JGAiY2ocGjZUcc7AFWzkCcp5Rb\/2yvf+xE7ThCw9hBMYv1vp7Gx8fjzKxdu1aozUweo4PHDhQ\/wIEkIOZmRlqbm4OoCTlKwLYysUEbMFWjoAfzx0dHZGjl1e8l15a+9sQMH6wlsMLL9zds2cPbd++nbgxwiLe+sQFf23JcQdbsJUjIOcZ9dYv2113H6Zddz8BAeMXa3298ZTR+vXrqa2tjbALqX6xQGMlxx5swVaOgJxn1Fu\/bNUW6te89Cw9+2cf8eu8BN5OO3ny5MkS5KNmWeC1Lz09PTQ5ObnonaOjo5GoiT\/YRi0THjRWMlzZK9iCrRwBOc+ot37ZNl53b+Tw9Ge\/Q0\/\/+e\/5dV4Cb0tOwMSZYwSmfrUQjZUce7AFWzkCcp5Rb\/2x1Rfwvv5rN9KRB\/\/Wn\/OSeIKAwUF2dauKaKzk0IMt2MoRkPOMeuuP7dgDx+iasUcih2fdP0hHv\/k3\/pyXxNOSFzAmccAUkgklexs0VvbMTFOArSkpezuwtWdmmgJsTUnl26n1Lysaz6AffOk\/0fT0dH6iillAwBgEDALGAJKDCRorB2iGScDWEJSDGdg6QDNMAraGoHLM9APs\/vMvXEj\/d9sHIGD8oK2eFwgYmZihsZLhyl7BFmzlCMh5Rr31w1aNvrC3hz\/9Hnrfu94BAeMHbfW8QMDIxAyNlQxXCBg5rmALtrIEinvXF++uW72M9l1zKYXah9VtCilrO3NWCFtbW+muu+4qHmULD6EG3wKBiCkEjAjWyCnYgq0cATnPqLfF2PLU0bVjj9D9j5+IHPHoC6+BCbUPq7uA6evrSzx7JSmM6vJFCJhilbwsqdFYyUUCbMFWjoCcZ9TbYmz\/\/OAs\/dcvfydy8tG2JvqT37wo+jcETDGui1KrERgIGM9gK+QOjZVcsMAWbOUIyHlGvXVnq08d8ajLvt+\/NBp9gYBxZxpEylDVa72Dg8ZKLgJgC7ZyBOQ8o966sY2Ll89vvJjWvW3ZvLNQ+7C6TyHxkf5btmwhHokp6xNq8OvNG42VXATAFmzlCMh5Rr21Z6sfWMepeeRFFy8YgbFnapyCL1YcGhqK7JuammhkZIRaWlqM09fCEAJGhjIaKxmu7BVswVaOgJxn1Ftztrxg9x+e+D5t+Yt\/nU90e\/fF1H358kVOQu3D6jYCEycc35VUplGZUINv\/lWRsURjJcMVAkaOK9iCrSwBM+8sXjZ84SHi\/\/PDa13i00a6p1D7sNIIGB02X7C4efNmmp2dLcWoTKjBN\/uqyFlBwICtHAE5z6i3YCtHINszC5ZHnvohdQ\/\/47xhfMFukodQ+7BSChg9AGrr9PDwMDU2Ntal3oQa\/LrA1F6KjkAuAmALtnIE5Dyj3qaz1RfqKque976JPnHFivndRmmpQ+3DSilg9BEYDsjo6KjxWTESX61Qgy\/BysYnGisbWna2YGvHy8YabG1o2dmC7UJePOISHU6395H56SK2yJsyilMPtQ8rjYCZm5uj\/v5+2r9\/f8S+s7OTBgYGqKGhwe4bIGAdavAFUFm5RGNlhcvKGGytcFkZg60VLitjsD2Fi0XL337nOF33l6cOpVOPrXBR6ULtw+ouYPRdSAy73qMtSd+2UINv1bIIGKOxEoD6U5dgC7ZyBOQ8L+V6q0ZbBu9+Yv4qAF247PyNFvrZprNyp4uWUh9WNwGj7zoq02jLUgq+XDNk5nkpN1ZmhNytwNadXV5KsM0j5P75UmPLooVHVf74wBG68avTi8C5jrjEHYX6R3jdBMxjjz1Gn\/zkJ+mGG24wXt+SdRdSfAoqbxv2+Pg4bdu2LYpznoAKNfjuzYyflEutsfJDzcwL2JpxcrECWxdqZmmWAlslWv70775L2\/\/PY4lgfAkX5TzUPqxuAsb3XUg8FcUPn+irfG\/cuJG6uroWVRB9ZxOvseG1N3yIXtppwKEG36xJkbNaCo2VHL1sz2ArRx5swdaWgDqvZdfdTxCfnJv0+BYt+jtC7cPqLmD4KgGbp7W1lUxuo9YFTdw\/j75MTEzMLxKO\/7xUht9suEvYoiOQoHrKJ9iCrRwBOc8h1VsWLfv\/8Rm6+1+eXbSmRRFk0XLN+95MH1\/XLAcVt1GLsvXuPG90J2kEpr29PXG0hjMXqnr1Dt7SYUiNlWXRxc3BVg4x2IJtEgEWLH839Tx9+VvHUgULp1MHz6l\/y9F81XOofVjdRmCkgqZ2NeWta9HPmsnb+RRq8KViYOoXHYEpKXs7sLVnZpoCbE1J2dtViS0Llq8\/9jyNP5AvWNatPoc2Xr48Ei\/8X62fUPuw4ASMqhgsZPgqgqSzZHjKaO\/evaRO982ablIjMHqFO3DgQK3rX5Dvm5mZoeZm2aHTIMEZFApsDSA5moCtIziDZGVlO\/uDV+ipF16h\/Y+8GP2X9TS94XS68PWn03\/\/lfMiM\/651k9HR8eiV05PL97lVOt8+X5fsAKGR1h6e3tpcHBwwe3WareSPmWUZqtgh6pefVcmW39V+mvLtmz1tgdbuQiAbbhs1Q4hPrb\/q\/\/0DP3L7IuZ00FMgkdU3rt6GXVffiGte9syOTgFPIfahwUrYNLuUIKAKfAt8JwUHYFnoJo7sAVbOQJynmtdb9XhcV8+dIwOPzuXK1aUYPnw2jfSR9uaIhD1mBKyjQAEjC2xGtvr00BKpKRtjU6aQkqbbuJihBr8Godo0etq3VjVu7y1fD\/YytEG2+qxVduYn3nxZdqx\/3EjoRIfXanX+hUftEPtw0o1AqMfLscLa48cObJgu3NWILPuUlKfdXd3zx+ap19hkLfgN9Tg+\/hiFPGBjqAIvey0YAu2cgTkPPuot2pUZfxbx+jIc2ajKkqsrF\/TSB9Z+8a6LbaVIhtqH1YaAaMW3fK6lWuvvTY6VI7PfMk7ZE4q4LrfUINfC3ZZ7\/DRWNW7DGV9P9jKRQZs68tWX6fy41d+Qnc+9D367vEfWY2qcAmubr2Arrz43ODESlJ0Qu3DSiFg9HNb1qxZQz09PZGAaWtro7S1LHJfocWeQw1+LRkmvQsdgVwEwBZs5QjIeU6qt7yg9tvf+yHte\/hpOvr8j6Kbmk0ffYGtGmWpwpoV0\/KZ2oXah0HAGNSAUINvUHRRE3SycnjBFmzlCPjzrI+mvPjjV2hs4gl6\/uXTjUdTVE5YlPBZKx9eewGd\/trXlHY3kD9ydp5C7cNKIWA4FOo4f30KSY3GpN1pZBdCd+tQg+9OxE9KdLJ+OGJ0S44j2Pphq0ZNpp5+ib4y+bTxjh\/97Wrk5Nd\/7nz6tZ89b373z1IcUbGNSqh9WGkEDAeEp4s2bdq0IDY7d+5MPeLfNoiu9qEG35WHr3QQML5ILvYDtmArRyDZsxIp3z72Q\/r8vUcjo\/sfP2GdjWi3zzln0BVvb6TL33L2klijYg3JMkGofVipBIxlTGpmHmrwawYw5UXoZOUiALZg65uA2t3Dfu96+Gma+t4PrdekqDwpkfILbz2b1rc0RlM+7P\/fv\/8UrVq1ynfWl7y\/UPswCBiDqh1q8A2KLmqCTlYOL9iCrS0BNYLC\/\/\/m4e\/T175zvJBA4ffzSMpH3rWcVp3bMC9SsqZ8UG9to2ZmH2ofVgoBo3YhTU5OZkZjy5Yt0e6kWj+hBr\/WHOPvQ2MlFwGwBds4AX33zl8\/+D26t4BAicTJTy8lXLd6GX3ksuW06ryG6JVF1qSg3srU21D7sFIIGA5Z\/HRc\/p0SNryId8OGDXU7EybU4Mt8Vcy9orEyZ2VrCba2xMzty8xWTfOMPfBUdDaK7bZjnYISIjyK8pvvWk4rz31VoBQRKVmky8zWvIaUzzLUPqwUAkY\/B4bPftEf\/RyYRx99lPjAu7vuuqumNSTU4NcUYsLL0FjJRQBsw2PL56Hw43J4WxINXaD8+s+fT++48Ky67+xBvZWpt6H2YRAwBvUl1OAbFF3UBI2VHF6wrRZbFicsKKafnaN9k0\/TY0+\/VGj0RJ\/K4ZuSP3jpBXTG6a+dn+KRGkEpSh31tijB5PSh9mGlEDCMPG8Kqaura\/6smM997nMyUU7xGmrwawoRIzA1xY2OQA63C1sWKD85eZL4fp6iUzuqZGonz5o3vo5+7k1n0erzz6z7CEpR6i5si75zKaQPtQ8rjYDhSpR0Dgxf6sjTSixwbrvtNhoZGaGWlpaa1rlQg19TiBAwNcWNjkAOt85WTevwUff\/\/OSLNP1M8ZGT+OjJu95yNrVccGbpR098EEe99UFxsY9Q+7BSCRiZ0BX3Gmrwi5Mp5gGNVTF+WanBtjhbJU7+\/Scn6cuH\/I6cRCLlnDPovW87h3iKhx8+C2WpP6i3MjUg1D4MAsagvoQafIOii5qgsZLDC7bpbNVOHZ6C+eLXZ+gfZ16IjF1OjY2\/RV8Ye8VFjbT87P84L1D0kRW5yFfbM+qtTPxC7cNKI2CmpqZo8+bNNDs7uyiCra2tNDw8TI2NjTLRzfEaavDrAlN7KRoruQgsRbb6OSd8xskDR35AR5+bK7wYVkVJiZNLlp9Ob2s6lz7a1hSdHjsvWn56LopcVMP3vBTrbS2iGmofVgoBMzc3F53x0t7ePn\/eS3d3N6nLHPnwuvj26loEXb0j1ODXkmHSu9BYyUUgBLZKHOjC5OkXXqY\/PzgbXQZY5IwTnbw+avLmxjPoD375LcTvSRMmIbCVq3nFPINtMX5pqUPtw0ohYOLnwPBZLytXrowuceSFvWNjYzQwMEANDacOUqr1E2rwa80x\/j40VnIRKDNbXZiofz\/\/0r\/R8N8\/6VWYMF0lQnidyTuazqLWN72+8IhJmdnK1ajaeAZbGc6h9mGlFDC84+jw4cPRtQH6QXZZU0hqFGf\/\/v1RDci7dkDf8ZQ3RRVq8GW+KuZe0ViZs7K1rCdbfY3Jkyd+RH\/xzWNep3J0YcILYX+u+fX0\/p89bx6R9GLYerK1rQdVswdbmYiF2oeVQsBwyHjUhZ+4aLnnnntoYmIidwRGT69fQcCjOPFHrbfZvXv3\/BbtrHeEGnyZr4q5VzRW5qxsLaXYqp05T33\/x\/S1R497O9NElU+fzml545m04ecvoNe+5rToY2lhYspYiq3p+0O2A1uZ6Ibah5VGwOjrYFh0sCAZGhqipqYmp7NfdEETrxL6CI9JdQk1+CZll7RBYyVH14atGjHhg9b+bup5+uYT348y5muNSXzE5KLlr6Nr3vdmmnn+x6USJqbRsGFr6hN2pwiArUxNCLUPK42A8Rm2rLuV4kLJ5L2hBt+k7JI2aKzk6DLb1559YbRL5rkfvkz3PHLc+zROXJjwgWu\/1X5GbNxMAAAgAElEQVRqZ476rKxH1hchj3pbhF52WrCVYRtqH1YKAWN6maPJNmo1ctPZ2Zk47aQEzFVXXUV33HEHTU5OkskaGL1aHThwQKaWLTGvMzMz1NzcvMRK7V7c2R+8Qk+98Apd+PrT6ciJf6OJI3P0nWdejhzy7\/lzH0\/TG06P3PB73rLsZ+icM19Dv\/GO18+\/mz9TNj7eVzUfqLdyEQNbP2w7OjoWOZqenvbjvEReghMwii0LGT5TJr57SQmYo0ePzp8tk2arfIWqXutdD\/HX1qkIqIv8+N\/\/++As\/YPAFI4aEYn+\/9OFr6vPa6BfufhcnGVi+UVAvbUEZmEOthawLExD7cPqKmB4Lcq2bdtyw5C3oyjJAS\/U7e3tpcHBwQV3JyVNIaXZQsDkhqaQQaiNlZpCUVuEvzL5NH372A+9L3iNixI+w+SyFW+ghv\/wWvqZHz9P737Hqvn4hDiVU6jyFUgcar0tgMRbUrD1hnKBIwgYGa6R16wpJNfXZm2\/1s+ZYf8sYG6++Wa65ZZbEk\/7DTX4rmx9patSY6VvDeZ\/\/8vsi\/TPsy\/SEY8nvepc9d0461rOofe89Ww6jU7txoluIc459bVKbH3Vp1r5AVs50mArwzbUPqyuIzA+Q6XvOlKjLLyDibdlx5+4uMnascRpQw2+T\/4uvsrQWOnTN\/\/vn5+NRInP4+fTRMmKcxvoDztW0FPfTz\/x1YWpSlMGtkXyX+a0YCsXHbCVYRtqHxaMgIkfZKcv4lWf8fUE6koC\/SC7tAW\/qiqFGnyZr4q5V4nGSp++4ZxMzrxA\/\/rUiyLTN2o0JPr\/OWfQxReeRavOa6B3Np01P1Ki25iTKW4pwbZ4rsLwALZycQRbGbah9mF1EzBq2oh3AeU9ebuE8tIX\/TzU4BflUjS9aWOl34XD\/+ZREp7CqcX0zVvPP7XQ9eff9PoFi13rJUxMmZuyNfUHu1cJgK1cbQBbGbah9mF1EzAyYZLxGmrwZWiZe\/37ySl685vfHCX4pydfpHseeY6mn3kp+vn+x0+YOzKwjF\/Yd\/UlF9CZP\/PawvfiGLy6LiboCOSwgy3YyhGQ8RxqHwYBY1BfQg2+QdGdTPQRkxMv\/Rv9T4FL+vQREJ6+efuFr6O3nndmKaZvnKB5ToRO1jNQzR3Ygq0cARnPofZhpRIwSduqd+7cGd1KXc8n1OC7MFVbg3nx60k6SXsfOOZ1fYk+UsL\/5umbtSveEE3f8F04+imvLvlfKmnQycpFGmzBVo6AjOdQ+7DSCBgWL3v37p0\/XI7DmHcpo0yoF3sNNfhp\/JRI4f8\/9N0f0PDfP0l0sti0ji5M1O3Bx449RcuXX1iaS\/pqVZ9q8R50snKUwRZs5QjIeA61DyuFgPF5lYBE+EMNvmLFoym8IHb8W8ecLvBT4uS9q5dR++pl9JbGhsh13u3B6Agkauspn2ALtnIE5Dyj3sqwDbUPg4AxqC+hBZ8Fy18\/+D2699Hj81MyWRj00ZMrLmqky1ee7WXxKxorg8rnaAK2juAMkoGtASRHE7B1BJeTLLQ+TBW3FAKGM4MpJJmKy15ZsPAOn9vuPZr5EiVUPvDO8+gD7zx\/fs1J3qmvrjlHY+VKLj8d2OYzcrUAW1dy+enANp+RiwUEjAs1yzRYxGsJLMVcHXs\/9sBTNPbAsVSnLEw+eMkFtLn9TZGNlFBJywAaKz\/xTvICtmArR0DOM+qtDFsIGBmulfBaleCzcHni2Tn64J89nMiVBcq171tBb1\/+utz1KbUIDBorOcpgC7ZyBOQ8o97KsK1KH2Zb+lJMIZVlt1EavCoE\/\/q\/epRGJp5cVAQWLZ\/feLHRBYC2laeoPRqrogTT04Mt2MoRkPOMeivDtgp9mEvJSyFgOOPx6aPR0dH5e4tcCuYzTVmDzyMuPEW06+4nFhSXRcv\/6Ho7rTy3oebTQjbc0VjZ0LKzBVs7XjbWYGtDy84WbO14mVqXtQ8zzX+aXWkEjJ5Bvh16aGgo+hXfKD0yMkItLS1Fy+qcvmzBT5sqUqMteduXnUF4TojGyjNQzR3Ygq0cATnPqLcybMvWh\/kqZSkFTFzM8M3Rw8PD1NjY6KvcVn7KFvxLbvrGgu3PVRMuCj4aK6tqaGUMtla4rIzB1gqXlTHYWuEyNi5bH2ac8RzDUgoYfQSm3jdRM7+yBJ+ni64Ze2Q+pFUVLhAwvr6+6X7QEcgxBluwlSMg47ksfZjv0pVGwJRt2kgHXe\/gJ6116f+1VbTxXctLvcYlr7KiI8gj5P452Lqzy0sJtnmE3D8HW3d2WSnr3YfJlIqoFAIm6yoBqYLb+K1n8Fm8bPjCQwsuMeRdRVVZ55LFGY2VTS20swVbO1421mBrQ8vOFmzteJla17MPM82ji10pBIxLxuNp5ubmqL+\/n\/bv3x99tGXLFurr68t1PTU1Rb29vTQ4OJi6ULhewU8SL\/t+\/9JKj7roAUFjlVs9nQ3A1hldbkKwzUXkbAC2zugyE9arD5MpzategxEwPAXFD4sW03NllOg5dOhQ5k6negQ\/Sbw8\/On3SNeHmvpHYyWHG2zBVo6AnGfUWxm29ejDZEqy0GswAiYOSxc0aSB5d5OyK9MITFy8dF++nG7vvrgW9aGm70BjJYcbbMFWjoCcZ9RbGbYQMDJcRbyarKlhmx07dlB3d3ckYsoiYFi8XDv2CN3\/+ImITajihcuGxkqk+kdOwRZs5QjIeUa9lWELASPDNfKaJTjUKInpOTBqN1NnZycNDAxQQ0NDYs755F9+1q5dW6o1MNf95Xfof31jNsobb5MObdpIDwYaK7kvFdiCrRwBOc+otzJsIWBkuHoXMCqbLGRmZ2cTRQwv3N2zZw9t376dZmZmjASMXvwDBw6I0PjiP5ygoW+eGnlpesPpNPTB5dH\/Q32YfXNzc6jFq2u5wFYOP9iCrRwBP547OjoWOZqenvbjvERe6roGJn7\/URoX0x1Fevqs3UUsbtavXx\/dtVSWXUg8dcQn7KqRl5B2G6XFFX9tybUEYAu2cgTkPKPeyrDFCIwM19wRGNfXpk09qemqycnJRa7TLpCUDn580S5PG\/H0UegPGiu5CIMt2MoRkPOMeivDVroPk8l1vte6jsDkZ8\/cQt91pLZH80WQeWfBlGEE5gv3fZc+\/ZXHosL+VnsT3frhi8wLXmFLNFZywQNbsJUjIOcZ9VaGLQSMDFdvXuMH2emLeNVnvOOIp430p94CJj51FPKi3Xiw0Vh5q\/6LHIEt2MoRkPOMeivDFgLGM1d959GaNWuop6eHkqZ1+LX1vtBRKvjxLdNLZepIVSU0Vp6\/VJo7sAVbOQJynlFvZdhK9WEyuTX3GswUknmR7S2lgq\/fLv2+NY105++12meuwinQWMkFD2zBVo6AnGfUWxm2Un2YTG7NvULAGLCSCL6+cJcX7C6FXUeYQjKobJ5M0BF4ApngBmzBVo6AjGeJPkwmp3ZeSyFgsnYGhTqFpI++9P7qSur\/tVV2kQvAGh2BXBDBFmzlCMh5Rr2VYQsBI8M10ysLm+uvv54+9alPpd4UXYts+Q7+Ul64q8cLjZVc7QVbsJUjIOcZ9VaGre8+TCaX9l5LMQKTlW0+z2VsbCzzWgD7YtulkAh+43X3Rpnou2oV9V210i5DgVijsZILJNiCrRwBOc+otzJsJfowmZzaea2EgOEzXkzvQrIrvpm1z+Bj9OVV5miszOqfixXYulAzSwO2ZpxcrMDWhVp+Gp99WP7bamdRegGTdadRrTD5DP79j52gDV94KMr67d0XR7dNL9UHjZVc5MEWbOUIyHlGvZVh67MPk8mhm9dSCJisRbx8mu7IyEgQa2D0c19Cv2napDqisTKh5GYDtm7cTFKBrQklNxuwdeOWlwoCJo9Qwc\/Vabnt7e3U1dVF8Z8Lui+U3Ffw9dGXT71\/FW29cmmufVHBQGNVqFpmJgZbsJUjIOcZ9VaGra8+TCZ37l5LMQLD2U+aKlIjMxs3boxETb0eX8HfcPtDdP\/jJ6JiLLVTd5Nih8ZKrkaDLdjKEZDzjHorw9ZXHyaTO3evpRAw+rUC8buKQtmFpC\/eXbd6Ge275lL3qAWSEo2VXCDBFmzlCMh5Rr2VYQsBI8M18ponYELYhaQfXIfRl1OVCY2V3JcKbMFWjoCcZ9RbGbYQMDJcI69Z613Gx8dpYmKi8ufAqOkjLN59tSKhsZL7UoEt2MoRkPOMeivDFgJGhuu8V54q2rp164IdR1NTU7R582bavXs3xaeWhLOzwH3R4OuLd\/\/gl1fQf\/v11bXMfmnfhcZKLjRgC7ZyBOQ8o97KsC3ah8nkqrjXUqyBUcVgEbNp06YFpRodHa2reOHMFA3+jV+dpj8+cCQqF1\/auO5ty4pHLgAPaKzkggi2YCtHQM4z6q0M26J9mEyuinstlYApXhwZD0WCj5N302OCxkqmvrJXsAVbOQJynlFvZdgW6cNkcuTHaykEjFoD093d7Tzaonzs378\/IrNlyxbq6+tLpBQ\/OK+zszNzjU2R4OsCZinfe5QUCDRWfr7EYCvHEWzBtrYEZN5WpA+TyZEfr6UQMFm7kEyLyTuV+GHRknV+TNqBeXzib5rgKRJ8nP2CERjTOuzTDuLQJ82FvsAWbOUIyHgu0ofJ5MiP11IIGC6K791GuqDJQ5X3btfgY\/oomzw6grya6f452Lqzy0sJtnmE3D8HW3d2WSld+zCZ3PjzWgoBk3UXEhe1tbXV6jZq2xEdKQGj7z7C9NHiSovGyt8XOe4JbMFWjoCcZ9RbGbYQMDJcvXvlkZehoSHKW9eiXmxyXYFr8O+4\/0nqu\/PR6FXYfQQB472yZzhERyBHG2zBVo6AjGfXPkwmN\/68lmIExl9xXvWUdLdS\/D1qPQz\/fmBggBoaGhKzwsHXnwMHDhhl+XfvPEaHnvwRNb3hdNr\/sWajNEvJaGZmhpqbwUUi5mArQfWUT7AFWzkCfjx3dHQscjQ9Pe3HeYm81E3A6NM8a9asoZ6eHpqcnExEYzuFxE74ELze3l4aHByklpaWRX5NxQsndFGv+vqX7suX0+3dF5co7OXICv6SlYsD2IKtHAE5z6i3Mmxd+jCZnPj1WjcB47cYi73xoXhpdygp8ZK180j36BJ8\/e4jTB8lRxuNldy3AGzBVo6AnGfUWxm2Ln2YTE78ei2NgEnb3tze3k5dXV25pdZ3HeUJFJPppaICBncf5YYMh63lI3K2QEfgjC43IdjmInI2AFtndJkJIWBkuM57TRIVJgtslYP4QXb6Il79oLy06aqsaSrb4OvTR+tWL6N911wqTK+a7tFYycUNbMFWjoCcZ9RbGba2fZhMLvx7LcUITNa2Z54KGhsbq9Rt1Ng+bVZR0ViZcXKxAlsXamZpwNaMk4sV2LpQy08DAZPPyNkiT8CkrWVxfqFlQtvg77r7MO26+4noLVj\/kg4bjZVlRbQwB1sLWJamYGsJzMIcbC1gWZja9mEWrutqWooRmPj6F51I3iFztaBnG3ysfzGLChorM04uVmDrQs0sDdiacXKxAlsXavlpbPuwfI\/lsCiFgGEUPFW0detWGhkZmd\/2zFuhN2\/eTLt373a+5NEHZpvgY\/2LOXE0VuasbC3B1paYuT3YmrOytQRbW2Jm9jZ9mJnHcliVRsAoEbNp06YFZEZHR+sqXjgzNsHH+hfzio3GypyVrSXY2hIztwdbc1a2lmBrS8zM3qYPM\/NYDqtSCZhyIFmcC5vgY\/2LeRTRWJmzsrUEW1ti5vZga87K1hJsbYmZ2dv0YWYey2EFAWMQB5vgY\/2LAdCfmqCxMmdlawm2tsTM7cHWnJWtJdjaEjOzt+nDzDyWwwoCxiAOpsHH+hcDmJoJGis7XjbWYGtDy84WbO142ViDrQ0tc1vTPszcYzksIWAM4mAafH39C7ZP54NFY5XPyNUCbF3J5acD23xGrhZg60ouO51pHybzdjmvEDAGbE2Dj\/UvBjAxAmMHydEaHYEjOINkYGsAydEEbB3B5SQz7cNk3i7nFQLGgK1p8LH+xQAmBIwdJEdrdASO4AySga0BJEcTsHUEBwEjA87UqzrzZXZ2dlGSrHuKTP0XsTMVMJfc9A3idTArGs+ghz\/9niKvXBJp0VjJhRlswVaOgJxn1FsZtqZ9mMzb5byWYgQm7\/ZoueKbeTYJvr6A93fXNdPAh1rMnC9hKzRWcsEHW7CVIyDnGfVWhq1JHybzZlmvpRAwWXchyRbfzLtJ8PUFvLd3X0zdly83c76ErdBYyQUfbMFWjoCcZ9RbGbYmfZjMm2W9lkLAqBGY7u7uup+6m4TbJPhjDxyja8YeiZJjB5JZpUVjZcbJxQpsXaiZpQFbM04uVmDrQi0\/jUkflu+lfBalEDCMhe9Cqvet02nhMQk+FvDaV240VvbMTFOArSkpezuwtWdmmgJsTUnZ2Zn0YXYey2FdCgGjppAmJycTqVRhEW\/jdfdGeV+3ehntu+bSckS35LlAYyUXILAFWzkCcp5Rb2XYQsDIcK2E17zg6wt4+65aRX1XraxEueqdSTRWchEAW7CVIyDnGfVWhm1eHybzVnmvpRiBkS\/mwjeoNTf79++PPtiyZQv19fWlZiMv+LqAwQJe82iisTJnZWsJtrbEzO3B1pyVrSXY2hIzs8\/rw8y8lM+qVAJmfHyctm3bFlEaHR2lI0eO0MTEBA0MDFBDQ4M3erzWhh8WLWr6auPGjdTV1ZX4jrzg4wRet9CgsXLjZpIKbE0oudmArRs3k1Rga0LJ3iavD7P3WI4UpREwLCr4ELve3l669tprI3HBa1\/6+\/upqakpc4SkKEpd0CT5ygs+FvC6RQCNlRs3k1Rga0LJzQZs3biZpAJbE0r2Nnl9mL3HcqQohYDRz4FZs2YN9fT0RIKlra1NfHeSyRk0ecHHCbxulRmNlRs3k1Rga0LJzQZs3biZpAJbE0r2Nnl9mL3HcqRY0gKGR16Ghoaos7Mzc5qKg68\/Bw4cmP9x9gevUOeemejny950Bn3xQzjAzrRqz8zMUHNzs6k57CwIgK0FLEtTsLUEZmEOthawMkw7OjoWfTo9Pe3HeYm8lELAMA9e\/8LrXfQpJDUak7U+xQdLNX2VttYmS73qJ\/DiADu7aOCvLTteNtZga0PLzhZs7XjZWIOtDS1zW4zAmLNytuTD7DZt2rQg\/c6dO1MX1zq\/KJaQL5Jk4TQ4OEgtLYvvMMoKPhbwukcBjZU7u7yUYJtHyP1zsHVnl5cSbPMIuX0OAePGrRKp8k4Bzgr+tXu\/TaPffCoqJ99AzTdR4zEjgMbKjJOLFdi6UDNLA7ZmnFyswNaFWn4aCJh8RpWx0HcdmdyEnRV87EByDzsaK3d2eSnBNo+Q++dg684uLyXY5hFy+xwCxo2bVSr9HBiVkM+D4d1IPp\/4QXYmi3jTFkDhCgH3yKCxcmeXlxJs8wi5fw627uzyUoJtHiG3zyFg3LgZp2LxsnfvXhoeHqbGxsYonckhc8YvKGCYFnz9BN4\/3XQxdb0LO5BsMKOxsqFlZwu2drxsrMHWhpadLdja8TK1hoAxJeVgl3UWS976FIfXWSdJC76+AwlXCFhjJTRW9sxMU4CtKSl7O7C1Z2aaAmxNSdnZQcDY8bKyrqqAGXvgGF0z9khUVmyhtgp5ZIzGyp6ZaQqwNSVlbwe29sxMU4CtKSk7OwgYO17W1jzSsnXrVhoZGZnfylz2KSQWLyxi+Dl+6xXWZV7qCdBYydUAsAVbOQJynlFvZdhCwMhwPdX5Hz8eXR8wOTmZ+xa+H+muu+7KtfNpkBZ87EAqRhmNVTF+WanBFmzlCMh5Rr2VYQsBI8O1El7Tgo8dSMXCh8aqGD8IGDl+YAu29SEg81YIGBmulfCaFHx9B1L35cuJF\/HisSMAAWPHy8YabG1o2dmCrR0vG2uwtaFlbgsBY87K2TLpHJhaXCWQl+Gk4Os7kPquWkV9V63Mc4PPYwTQWMlVCbAFWzkCcp5Rb2XYQsDIcJ33WrVzYLADqXiFQGNVnGGaB7AFWzkCcp5Rb2XYQsDIcI28VnEbtX6JI+5AcqscaKzcuJmkAlsTSm42YOvGzSQV2JpQsreBgLFnZpyiigIGO5CMw5tqiMaqOEOMwMgxBFuwrT0BmTdCwMhwrewUEgRM8QoBAVOcITpZOYZgC7a1JyDzRggYGa4LvFZpES+2UBevEBAwxRmik5VjCLZgW3sCMm+EgJHhWgmv8eDrW6ixA8k9hBAw7uzyUoJtHiH3z8HWnV1eSrDNI+T2OQSMG7cgUsWDr2+hxgJe9xCjsXJnl5cSbPMIuX8Otu7s8lKCbR4ht88hYNy4BZEqHnx9BxIucXQPMRord3Z5KcE2j5D752Drzi4vJdjmEXL7HALGjVsQqbIEDEZg3EOMxsqdXV5KsM0j5P452Lqzy0sJtnmE3D6HgHHjVrNU8QshOzs7aWBggBoaGhLzoC8YzrONBx87kPyEFY2VH45JXsAWbOUIyHlGvZVhCwEjw9WL17m5Oerv76f29nbq6uoi9XNTUxP19fUtesfBgwdp165dNDw8HAkcTptmy4khYLyEaZETNFYyXNkr2IKtHAE5z6i3MmwhYGS4innlEZaJiYnEUZj4Z1m2SQIGW6j9hA2NlR+OGIGR4wi2YFtbAjJvg4CR4SrmNUuUJI3AqNGbpAzFg68EDLZQFwsfBEwxflmpwRZs5QjIeUa9lWELASPDVcSrWg+zcePGaEop6ZmamqLNmzfT7OwsjY6OUltbW2pe9ODjFmp\/IUNj5Y9l3BPYgq0cATnPqLcybCFgZLh696rWv7DjtEW88ZuveT0MP0nrZfj3HHz1vHLeRfTiut7oxy9+aDld9qYzvJdhqTicmZmh5ubmpVLcmpYTbOVwgy3YyhHw47mjo2ORo+npaT\/OS+TltJMnT54sUX4KZcVEvMQX\/PILeTSmt7eXBgcHqaWlZVEedPWKM2AKhWhBYvy15Y9l3BPYgq0cATnPqLcybDECI8PVm9e8nUfqRUUFzDVjj9DYA8cid8dvvcJb\/peiIzRWclEHW7CVIyDnGfVWhi0EjAxXb155GojXs2Sd\/aJeljSFlJVWDz7OgPEWMmz19YdykSd0BHJwwRZs5QjIeIaAkeHqxWv8EDvltLW1dcFZL93d3fOLdVnwDA0NRaY2B9ldctM3iC9zXNF4BvEpvHjcCaAjcGeXlxJs8wi5fw627uzyUoJtHiG3zyFg3LgFkUoPPs6A8RdSNFb+WMY9gS3YyhGQ84x6K8MWAkaGayW8quDzyAuPwPCDM2CKhw6NVXGGaR7AFmzlCMh5Rr2VYQsBI8O1El5V8HEGjN9wobHyy1P3BrZgK0dAzjPqrQxbCBgZrpXwmiRg9v3+pbTubcsqkf+yZhKNlVxkwBZs5QjIeUa9lWELASPDtRJeVfD1M2B4AS8v5MXjTgCNlTu7vJRgm0fI\/XOwdWeXlxJs8wi5fQ4B48YtiFQq+PoZMBAwxUOLxqo4wzQPYAu2cgTkPKPeyrCFgJHhWgmvKvg4A8ZvuNBY+eWpewNbsJUjIOcZ9VaGLQSMDNdKeIWAkQkTGisZruwVbMFWjoCcZ9RbGbYQMDJcK+FVBR9nwPgNFxorvzwxAiPHE2zBtjYEZN4CASPDtRJeOfhf+9a\/zp8B0335crq9++JK5L3MmYSAkYsO2IKtHAE5z6i3MmwhYGS4VsJrXMDgEDs\/YUNj5YdjkhewBVs5AnKeUW9l2ELAyHCthNe4gMEZMH7ChsbKD0cIGDmOYAu2tSUg8zYIGBmulfDKwf\/s+ATxNmp+IGD8hA0Cxg9HdLJyHMEWbGtLQOZtEDAyXCvhlYO\/5U\/\/lnbd\/QQEjMeIQcB4hBlzBbZgK0dAzjPqrQxbCBgZrpXwysF\/\/2e\/SmMPHIvye\/zWKyqR77JnEo2VXITAFmzlCMh5Rr2VYQsBI8O1El45+O+8\/q\/p\/sdPRNcH8Cm8eIoTQGNVnGGaB7AFWzkCcp5Rb2XYQsDIcK2EVwgYmTChsZLhyl7BFmzlCMh5Rr2VYQsBI8O1El45+Cd+YzjK67rVy2jfNZdWIt9lzyQaK7kIgS3YyhGQ84x6K8MWAkaGqzevx48fp56eHpqcnIx8dnZ20sDAADU0NCS+4+DBg7Rp06bos9bWVhoeHqbGxsZE25XvfDf94Fd3QcB4i9YpR2isPAPV3IEt2MoRkPOMeivDFgJGhqsXr3Nzc9Tf30\/t7e3U1dVF6uempibq6+tb9I6pqSnavHkz7d69m9ra2mh8fJwmJiZSBY8+AoND7LyEDALGH8ZET+gI5ACDLdjKEZDxDAEjw1XMa5Yo4c8OHz6cKG6SMrTi3e+nF9f1Rh9BwPgLGToCfyzjnsAWbOUIyHlGvZVhCwEjw1XMa5qAiY\/WmGRAFzA4xM6EmJkNGiszTi5WYOtCzSwN2JpxcrECWxdq+WkgYPIZlcZCrYfZuHFjNKWkP0rAXHXVVXTHHXdEa2by1sA0feAP6Udv3xC5Oev+Qbpv\/AulKWuVMzIzM0PNzc1VLkJp8w62cqEBW7CVI+DHc0dHxyJH09PTfpyXyMtpJ0+ePFmi\/BTOihIo7ChpEa\/6\/OjRo\/MLd3ft2kWzs7Opa2Ca3\/df6KW1vx3ljc+A4bNg8BQngL+2ijNM8wC2YCtHQM4z6q0MW4zAyHD16jVPvPDLkqaQeFFvb28vDQ4OUktLy6I8Lf\/wTfTyivdGv8cpvP5ChsbKH8u4J7AFWzkCcp5Rb2XYQsDIcPXmNW\/nkf4iHnFZuXLl\/PQSC5ibb76ZbrnllsSt1Bd89M\/olfMuwim83qJ1yhEaK89ANXdgC7ZyBOQ8o97KsIWAkeHqzWveNJD+Ij4Dhu3V2S\/8b36Stlzz78\/7vb+kn5x5HgSMt2hBwHhGucgdOgI5wmALtnIEZDxDwMhw9eI1foidcqoW5\/JhdnxOTHd3d3TuCz\/6QXZ5h941XndvlAan8GPx1ggAAAxUSURBVHoJ17wTdAR+eerewBZs5QjIeUa9lWELASPDtRJelYDpvnw53d59cSXyXIVMorGSixLYgq0cATnPqLcybCFgZLhWwqsSMDjEzm+40Fj55YkRGDmeYAu2tSEg8xYIGBmulfAKASMTJggYGa7sFWzBVo6AnGfUWxm2EDAyXCvhVQkYnj7iaSQ8fgigsfLDMckL2IKtHAE5z6i3MmwhYGS4VsKrEjC4RsBvuNBY+eWJaQ45nmALtrUhIPMWCBgZrpXwCgEjEyYIGBmumEKS4wq2YCtLQMY7BIwM10p4VQIGp\/D6DRcEjF+eGCWQ4wm2YFsbAjJvgYCR4VoJryxg+P4jvgcJjz8CEDD+WMY9gS3YyhGQ84x6K8MWAkaGayW8QsDIhAmNlQxXTHPIcQVbsJUlIOMdAkaGayW8soDBKbz+QwUB45+p8gi2YCtHQM4z6q0MWwgYGa6V8AoBIxMmNFYyXDFKIMcVbMFWloCMdwgYGa6V8MoCBtcI+A8VBIx\/phiBkWMKtmArT0DmDRAwMlwr4ZUFDK4R8B8qCBj\/TNHJyjEFW7CVJyDzBggYGa6V8AoBIxMmCBgZrpjmkOMKtmArS0DGOwSMDNdKeF3+4ZvorwaupXVvW1aJ\/FYlkxAwcpECW7CVIyDnGfVWhi0EjAzXSngNNfj1ho\/GSi4CYAu2cgTkPKPeyrANtQ877eTJkydlkIXjNdTg1ztCaKzkIgC2YCtHQM4z6q0M21D7sGAEzPHjx6mnp4cmJyejGtDZ2UkDAwPU0NCQWSOmpqaot7eXBgcHqaWlJdE21ODLfFXMvaKxMmdlawm2tsTM7cHWnJWtJdjaEjOzD7UPC0LAzM3NUX9\/P7W3t1NXVxepn5uamqivry81wsru0KFDNDIyAgFj9l3wZhXql8oboAKOwLYAvJykYAu2cgRkPIdaZ4MQMEkhHx8fp4mJicxRmIMHD9KuXbui5BiBkfniZHkN9UtVe5KL3wi2clEAW7CVIyDjOdQ6u2QFDE857dixg7q7uyMRAwEj88WBgKk9V35jqA1WfWgufCvYykUBbGXYhso1SAGj1sNs3LgxmlJKG6Hh369du9ZoDYxMtYJXEAABEAABEJAnMD09Lf+SGr8hOAGj1rUwx7RFvLxwd8+ePbR9+3aamZnJFTA1jgleBwIgAAIgAAIgkEMgKAFjIl6YB08ZrV+\/ntra2shkFxJqEQiAAAiAAAiAQLkIBCNgTHcexbdb6+EYHR2NRA0eEAABEAABEACBchMIRsDwqMrs7KzR2S96SDACU+4KityBAAiAAAiAQBKBIARM2qhKa2srDQ8PR4fZ8TkxvOMoPsICAYMvBgiAAAiAAAhUj0AQAqZ62JFjEAABEAABEACBIgQgYIrQQ1oQAAEQAAEQAIG6EICAycDO62qGhoYiCyzwdaufPEW3efPmaH1S3v1UOm++BiLrege33ISVyoatKnn82o2wiPgpjQ3X+PQ12onsGNiw1W3RHhSr2+p7n7SMopjn+qaGgEnhr64Z4DU0jz76aLT1mv\/d2NhY34hV6O16Z7lhw4YF91XFixG\/+oF\/3rt3L5inxNuGre6CuW7bto127tyZeshjhaqY96zacI3vfMR6uuxw2LBVwpDvsuN1i2gP3Ku64r5\/\/\/7g\/hCHgEmpF+qOJP4Chape3b8SZinjDTqLwrGxMaOdYugM8v+S1W9RN2HLncL1119PJ06coKxTqs2iG6aVTZ1l25tvvpluueUW\/GFjUB1s2er1G+2BAeAEEzWKddlll9HRo0ejy41DOioEAiYh6Gm3W6vbrt2q0tJLpY9i8chV\/OcsImiwsuuLC1sW5Zdffjl95Stfmb+5fenVSn9cTS6MBd9XCdjU2aQRmLzLecF6MYEnn3wy+iXvxO3p6YGAWQqVJGnEhRv\/lStXYtjdogLERwVs\/mJ1PdfHInuVNrVlq67PuO6666JLTCHGk8Nvw5UFzOHDhyNHWCuX\/3WyYcve9KmPLVu2RJ0vHjcCcUHo5qV8qTACkzECoy94goCxr7y2DZZ6A3cMt912GxbxZiC3YcsdwWc\/+1n62Mc+Rs3NzZlrkeyjHFYKG65qPZFauMtpt27dinqbUiVs2Kqpj927d0dTHjajt2HVSD+lgYDxw7ESXjCF5CdMNkPGEC92zG3Ysu19990X\/QWLXUjZnG24xqeQwBZs7b7FtbOGgKkd61K8SR9xwSJet5DEp4zyFppip4E5Zxu2+vZ0\/Q0Yll\/M24ZrvD6jnciuvzZsIQ7N2wITSwgYE0oB2WAbdfFg2mybxPC7HW8btrpnjBJkc7bhGu8UMM3hj23SFBKm5+zaCN0aAsadXWVT4iC74qHLOrhKLYLkqY20UQIcDJYeA1O2EDB29diGq36QHQ5by+dsw5YF4aZNmyKnYJvPNssCAqYYP6QGARAAARAAARAAAW8EsAvJG0o4AgEQAAEQAAEQqBUBCJhakcZ7QAAEQAAEQAAEvBGAgPGGEo5AAARAAARAAARqRQACplak8R4QAAEQAAEQAAFvBCBgvKGEIxAAARAAARAAgVoRgICpFWm8BwRAAARAAARAwBsBCBhvKOEIBPwTePzxx+mcc84hvs3b5OHzHp5\/\/nlavXq1ibmTjTqzp7W1lYaHh43zVpUbxlX5anX2SK3f5xR0JAKBEhKAgClhUJAlEGACtie71uKwqiIipEjaWtYIFhT81PL246qwqWUc8C4QyCMAAZNHCJ+DQJ0IlFHA2OZJR1eVThoCpk4VHq8FAUsCEDCWwGAOAj4J6EfRs181LfPoo4\/OH6POv1dXKsSvXFDTHOeeey719PTQ5ORklD11UaO622f\/\/v3R702mffR36NMofPXDtm3b5ou\/c+dO6urqWoQjzU4JmA0bNtCNN94YpYtP0+jHxyvH6j3M6vrrr6df+qVfitKrsjz33HO0efNmmp2djZJ85jOfoX379tHg4CC1tLREv0srU1Is4wKGf37hhRei\/xTHrIsw4xcR8juSfldFceez7sMXCBQlAAFTlCDSg4AjgaSLFfXOMz7akXZDL79+YGCA2B+LGJ76aGtrIyWONm7cOC80sm78VvlR\/hoaGqKO97bbbqORkZFIDOSNwMTt9Uv5WGSx0Ljsssui\/Cr\/e\/fujdbSsBDp7e1dIDx0f0qkrVixYj59vIzq52eeeSbKc3NzM\/X390dCSU0J5V0cmiRghoaGSBdSzFnnqlcBCBjHLwSSgYAlAQgYS2AwBwFfBJIEhu47TyzE\/7KPCxhOPzY2Nt\/Zs33WbdRJUzxx+6w85d10Hb9hmPOTN62kf64ETFyQTUxMLCijLlD4HTfffDPdcsstCxYbZ00TJQkYHt1Root9ZnGAgPH1DYEfEMgmAAGDGgICdSSgT7fEp3fSOsn4NEtnZ2fiCEx8KkcvZtL0T9r79FvDszruvEXESWIl6XfxabX4NJkaYeLyJAkR3SeP6qgbjeNhTpsGShIwnFZf1JslvCBg6viFwquXFAEImCUVbhS2rAT0TltfB8OdqdqqrARJfF2KGoGIj8Bw2vjIQVb508RJ1rSW7q+ogGFfai2LElhJIzA2AubBBx8kNUVluhUdAqas3xLkCwQWEoCAQY0AgRIR0EWAGmFgAcPrRXgtR3t7+4KFs7pIiQuYrPUuSUWuxRRSfI2L\/k4WG1nTQWoKSRcwSaMd+hQSj8Bs3bp1fg2PSaglppDyxGTeVJpJvmEDAkuNAATMUos4ylsaAklrYPRREH1Ra9JiVDUio6aQuGC6yFH+eUGvmv5IWoeigPhaxKuPeOjrYtauXbtokW5cwOhpVV45f7wgN0nAmC7iZR9qDUve2qOii3jVFJ\/aOabKoS9ejldCCJjSfC2RkQoRgICpULCQ1fAIqM5NbQHWp4f0LdA8pXLllVcu2CrNwuXqq6+mG264YX6EIS5q1KiM2l7NBFXHmkYza8ux6cLipO3WJmtg4u\/evXt3tM6FF+6q8usjMFyGOMP4Nur4VnJOk7YFXI168f+V6EvaRq2nTyqXvv6I43TJJZfQww8\/HImouNBUZYiPToVX21EiEPBLAALGL094AwEQqDMBFhRJO49Ms2WyBsbUl6kdRmBMScEOBF4lAAGD2gACIFBZAvF1Pmq0RT\/3xbZwEDC2xGAPAvUhAAFTH+54KwiAgCcC8dOJs07JNXll\/HLFO++8M0omdTcSLnM0iQpsQGAxgf8PxFR8g7XjJMEAAAAASUVORK5CYII=","height":337,"width":560}}
%---
