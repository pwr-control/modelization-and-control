preamble;
model = 'single_phase_cllc';
%[text] ### Global timing
% simulation length
simlength = 1.25;

% switching frequency and tasks timing
fPWM = 20e3;
fPWM_AFE = fPWM;
tPWM_AFE = 1/fPWM_AFE;
TRGO_AFE_double_update = 0;
if TRGO_AFE_double_update
    ts_afe = tPWM_AFE/2;
else
    ts_afe = tPWM_AFE;
end

fPWM_INV = fPWM;
tPWM_INV = 1/fPWM_INV;
TRGO_INV_double_update = 0;
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

fPWM_CLLC = fPWM;
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
nominal_power = 1600e3;
application_voltage = 690;
vp_xi_pu = 1;
vn_xi_pu = 0;
vn_eta_pu = 0;
grid_emu_data = grid_emulator(nominal_power, application_voltage, vp_xi_pu, vn_xi_pu, vn_eta_pu);
%%
%[text] ## Global Hardware Settings
fres = fPWM_CLLC*1.25;
hwdata = global_hardware_setup(application_voltage, fPWM_AFE, fPWM_INV, fPWM_DAB, fPWM_CLLC, fres); %[output:2951b0eb]
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
l1 = Kd(2) %[output:1144ae32]
l2 = Kd(1) %[output:1d963221]
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
%%
%[text] ### DClink, and dclink-brake parameters
Vdc_ref = grid_emu_data.Vdclink_nom; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
%[text] ### DClink Lstray model
parasitic_dclink_data; %[output:7ef6d9d7]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:0fd0717d]

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
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:44396bd3]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:8de91a74]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:360d2452]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:0c11bd1d]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid_emu_data.w_grid;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:2fa74918]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:79eff6bc]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:6d3e5d2e]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:14c546cf]
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
run('n_sys_generic_1M5W_pmsm'); %[output:075d78cf] %[output:6e51d2e7]
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
kg = Kobs(1) %[output:57b55c06]
kw = Kobs(2) %[output:616fcdfb]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:3623cefa]
luenberger_l2 = Klo(2) %[output:14798efd]
luenberger_l3 = Klo(3) %[output:2059d561]
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
igbt.inv = device_igbt_setting_inv(fPWM_INV);

% infineon_FF650R17IE4;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.afe = device_igbt_setting_afe(fPWM_AFE);
%[text] ### DEVICES settings (MOSFET)
infineon_FF1000UXTR23T2M1;
mosfet.inv = device_mosfet_setting_inv(fPWM_INV);

infineon_FF1000UXTR23T2M1;
mosfet.afe = device_mosfet_setting_afe(fPWM_AFE);

wolfspeed_CAB760M12HM3
mosfet.dab = device_mosfet_setting_afe(fPWM_DAB);
%[text] ### DEVICES settings (Ideal switch)
silicon_high_power_ideal_switch;
ideal_switch.dab = device_ideal_switch_setting(fPWM_DAB);
ideal_switch.afe = device_ideal_switch_setting(fPWM_AFE);
ideal_switch.inv = device_ideal_switch_setting(fPWM_INV);
%[text] ### Setting Global Faults
time_aux_power_supply_fault = 1e3;
%[text] ### Lithium Ion Battery
nominal_battery_voltage = hwdata.dab.udc1_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab); %[output:5c304d9d]
lithium_ion_battery_2 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab); %[output:19da337d]
%[text] ### C-Caller Settings
open_system(model);
% Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'rpi_output_t'});
%[text] ### Remove Scopes Opening Automatically
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

%[text] ### Enable/Disable Subsystems


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":35.9}
%---
%[output:2951b0eb]
%   data: {"dataType":"text","outputData":{"text":"Device AFE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\nDevice INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\nDevice DAB: DAB_1200V\nNormalization Voltage DC1: 1200 V | Normalization Current DC1: 250 A\nNormalization Voltage DC2: 1200 V | Normalization Current DC2: 250 A\n---------------------------\nDevice CLLC: CLLC_1200V\nNormalization Voltage DC1: 1200 V | Normalization Current DC1: 250 A\nNormalization Voltage DC2: 1200 V | Normalization Current DC2: 250 A\n---------------------------\n","truncated":false}}
%---
%[output:1144ae32]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  18.982392004991411"}}
%---
%[output:1d963221]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.076483869223994"}}
%---
%[output:7ef6d9d7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAdAAAAEYCAYAAADs5qfZAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ2QVVe151ee7QukQoQm5KPToZsYUExZhLwXB0m0yaCDTmzGIW8Cjda0PQQZDIWlzfRXqCKUSTdNpVMGSboIIK+nYnd4KjPSVdbDCMIzIo4JCTWFUUiahpA2Ex6QkpSg4mNq7WbfnHu4H\/ucs\/c5Z937P1UU3bf31\/mtvff\/rn32WfuqS5cuXSJcIAACIAACIAACgQhcBQENxAuJQQAEQAAEQEARgICiI4AACIAACIBACAIQ0BDQkAUEQAAEQAAEIKDoAyAAAiAAAiAQggAENAQ0ZAEBEAABEAABCCj6AAiAAAiAAAiEIAABDQENWUAABEAABEAAAoo+AAIgAAIgAAIhCEBAQ0BDFhAAARAAARCAgKIPgAAIgAAIgEAIAhDQENCQBQRAAARAAAQgoOgDIAACIAACIBCCAAQ0BDRkAQFN4OjRo9TU1EQjIyMZKPX19bRu3ToaO3ZsUVDnz5+ntrY2amhooFmzZhVNf+DAAVq8eHFWumXLllFraysFLatoZUgAAiBQkAAEFB0EBCIQYAFtaWmh9evX09SpUwOLWFDRYwHt7u6mrVu3UmVlZaa+qqoqJaK4QAAE4iMAAY2PNWoqQQLFBNTrMWpPkTGwCG7atInq6uoUFf4be6Dbt2+n9vb2zGd+UfQLKCfkNnR2dtJjjz2mhJy92RkzZijPdnBwUJWlvWL+WX\/On\/3hD3+gjo4OOnjwIO3fv59OnDhBixYtopqamixPt7+\/n6ZNm0bNzc306U9\/mr71rW8Ri\/YTTzyh7uXQoUPU1dVFCxcuLEEr45ZAIDcBCCh6BghEIJBrCVcLiVdcq6urlXDNnj1biZP2Ik+fPq2WgFmI+BoYGFDLv1ro\/Eu7uQT0zJkzSti++c1v0pYtW5SA3nrrraqMW265hfTfvULJdbDorVq1irZt26YE9Pnnn894tlyPXlJmUR8eHqalS5fSkiVL1Ocs7HwPnI694RdeeEEJsOnSdQTkyAoCqSEAAU2NKdAQiQTyeaBaKLUg8vNQLUT6Pv3PLY8fP57xPnUar9fKn5kKKIucXh5mL5S9xd7eXiWw3Db2FPMJq3526\/eetYByu7W3zMLKv3Na771KtCXaDAJBCUBAgxJDehDwEPALKP9JCyUvzwYVUC1I+SCbLuFyft5s5F161R5qMQHV3i\/\/zx7lzp07szxQCCiGAAiMEoCAoieAQAQChTzQu+66K7PByHQJVy+petN7nysW2kS0cuXKzI5eviUt3v6lWr3Umu9zLaDeZ6nswcIDjdBRkLUkCUBAS9KsuKm4CBR7jcXFJiKT11h4ww8\/r2SR5PTnzp27YnNRrk1E+hmm3szEwsnlvPrqq+rLwIoVK9SSLZZw4+phqCfNBCCgabYO2gYCDgnkWg52WB2KBoGSIwABLTmT4oZAID8B72synIqfkZoEcABTEACBKwlAQNErQAAEQAAEQCAEAQhoCGjIAgIgAAIgAAIQUPQBEAABEAABEAhBAAIaAhqygAAIgAAIgAAEFH0ABEAABEAABEIQgICGgIYsIAACIAACIAABRR8AARAAARAAgRAEIKAhoCELCIAACIAACEBA0QdAAARAAARAIAQBCGgIaMgCAiAAAiAAAhBQ9AEQAAEQAAEQCEEAAhoCGrKAAAiAAAiAAAQUfQAEQAAEQAAEQhCAgIaAhiwgAAIgAAIgAAFFHwABEAABEACBEAQgoCGgIQsIgAAIgAAIQEDRB0AABEAABEAgBAEIaAhoyAICIAACIAACEFD0ARAAARAAARAIQQACGgIasoAACIAACOQmcPHUMJ37WR9NeHBNySMqewG97bbbSt7IuEEQAAEQcE1gxrgL9F9vPkv8\/9t\/rqAv\/99bA1c5NDQUOE+SGSCgt91GkozGgi+pvUl27rB1g3FYcub5wNicVdiUcTBmb\/P84b10bm8fXXxnmMbeMYfGzWmkMXfMCdzsONobuFFFMkBAhQmSxE5mu9O6Lg+MXRMmAmPZjPUy7dnvP0oVk2pp3Jyv0Lj7GtXPYS+JfQICKkxAjx07RlOmTAnbR5HPgAAYG0CKmASMIwI0yO6CMQvn2X9aS+f2\/mNGOG0964SAGhg1bUmkGc3FoEibTZJuDxi7twAYy2HsXaa9cHjvqLcZcpm20F1Lm4v5XuCBwgN1P5KF1YDJ3b3BwDj9jPUyLXubfNlYpoWAure79RoOHDhAixcvVuV2dXXRwoULM3VI+9aDicd697iiQDAGY\/cE3NcQth+7XKaFgLq3O23fvp3a29uzavILn2kzzpw5Q83NzdTR0aGydHZ2Uk9PD1VWVqrfIaCmJMsnXdiJp3wIRb9TMI7OsFgJQRj7l2l5MxA\/22SvM65L2lzMXFK1hKs9xVxiqUW1v7+fZs2aZWxTLrO7u5u2bt1KY8eOpba2NmpoaMiUIc1oQQaFMSQkzCIAxsE6xMCv36YTZy6oTCfOnM9kflN\/dlb\/bfT\/fNfkyjE0ecIYuuf2CdQ6L\/xuzmCtL93UJv3Yv0wb5TWUqCSlzcWpElD2FAcHB6mxsbGgHfr6+oqm8RbAAjowMEDr1q1TH7OAzp49O7OMW\/nNn0W1O\/KDQFkTqLquIuv+bx43+rv386pxFXTz5XT106+lkydPUnV1dVa+Z3\/1Lr301gV6+a1RoeX8g43ZacoadMCbz8U4U8TZk3TppR8SvfAU0YRqor9\/gK767NcD1mAn+dy5czMFSXvHPVUeqB1zZJdSTEClfesx+VbpgmM5lQnG7q1djPGLr79LK55\/TXm29354PG1smE7soeIyJ5CLMW8I4qAHvJuWl2knP3PMvEDHKaXNxanyQLkxR48epaamJhoZGVGbffjiZ6FVVVW0bds2mjp1amATYgk3MLKyz1Bsci97QBYAmDJmAZ3\/zCtKSM88eZ+FmsunCM1YP9\/k9zf5SnKZthB9CGiEvnn+\/PnM88lp06bRkiVLaNGiRWqplZ9\/7t+\/Xy3D8nPMIBc2EQWhhbRMwHRyB63wBIIy7t41TN27jilvdOfDM8NXXEY5jx38OY1\/fQ\/ZjBbkEh8ENAJdFrq1a9fSmjVr1A5Z3vhTV1enNvv4\/xa0Gu9rLP5NSNKMFnTiCcoK6SGgcfSBMP2Yl3XZG+Wl3FdXfzKOZoqsI6nXUKLCkjYX8\/2m5hmoSwEtpWWDMBNP1I5dbvnB2L3FwzLmpdwVA6\/RibMXaOOi6XTv7ePdN1ZADbmCur\/3kf9AUz7TIKD1o02EgEYwFQTUDF7YicesdKTCEm48fSBKP4aIvm+jQkHdozCOpxdk1wIBjUCdBZSfex46dChnKTNmzFDvcuoACBGqysoqzWjSBoUtO8VZDhi7p22D8fynX6EX33iXdn5tZtl5oibLtDYYu+8J79cgbS5O1RJunIby1iXNaNIGRVJ2jVIvGEehZ5bXFuOHB14jDuTwdMN0arj7JrPKhabyRwviMzc5qHu+aEG2GMeFS9pcnCoBhQdq1k2lDQqzu0pXKjB2bw+bjFlAWUhb500p2QhG+ggxtozpayg2GbvvEXgGao0xv7YyPDxMra2tqkz+nS9vEHhblUn71iNtUNiyU5zlgLF72rYZaxEtpXdFcz3fDHL2pm3GrnuFtLk4VR6oNk6uV1aivsZSyPDSjCZtULgedC7KB2MXVLPLdMFYi6j0d0X90YLCBnV3wdhlz5A2F6dSQHVABW+8Wn4nlKMThQmkUMzg0owmbVAU45\/Gv4Oxe6u4YqzfFZUoomGWaQtZyhVjV71D2lycSgHlRvmfh7ragct1STOatEHharC5LBeMXdIdLdslY0kimms37bj7GlWc2qiXS8ZR25Yrv7S5OLUC6sI4+cqUZjRpgyJOW9qqC4xtkcxfjmvGaRbRXGdv8k7aIM83TSzkmrFJG4KkkTYXp0pAXR1nVsyA0owmbVAU45\/Gv4Oxe6vEwdgbcCENof+8Z2\/yz65PQ4mDsc2eIm0uTpWAcmOiHqitD93msrzLvt5YuP7DuqUZTdqgsDnA4ioLjN2TjotxGqIW+XfTmr6GEtUKcTGO2k6dX9pcnDoB1SC9Qqg\/8wuf32h8FFpnZyf19PRkgtHzxqOWlhZavXo1dXR0qCzeNPy7NKNJGxS2Blec5YCxe9pxM+aoRXHHz\/XvpnWxTFvIUnEzjtprpM3FqRXQqIbQ3uzAwAAtWLCAvv3tb6swgHwUWltbGzU0NKhTXiCgNkiXXhnSJh6JFkiCsQ795\/Jd0VzLtGFfQ4lq1yQYR2kzBDQKPct5+dWX2tpaqqmpIRZSfgWGLxZQ7ysy0owmbVBYNmssxYGxe8xJMdah\/2zHz821TDvp4W3uQRaoISnGYW9a2lxcsh6oN5IRP\/8sJqDa4Lt37w5r+9jynTx5kqqrq2OrrxwrAmP3Vk+S8bO\/epc2\/Z936dHPXE\/106+NdrMv\/ZAuvfxDojcOEE2opqva\/yVaeRZzJ8k4yG3MnTs3k3xoaChI1sTTpuY80KAk+JlnU1OTCrBQX1+fCbKgPU8d9o8FlD\/DEm5QwuWbXto3d4mWSppx1NB\/OuiB3k2b1DJtIdsnzThov4QHGpRYnvTeQApbtmyhPXv2UGNjI02dOrVgDSyUdXV1meebnJjLam5uxiYiS7Yph2KkTTwSbZIGxvpdUT7FhU9zKXYltZu2WLvy\/T0NjIO0HQIahFaetN5QfhxQngWRL70MyxuBcl3eV1X037VnymeMLl68WH3c39+fJbDSjCZtUFjoErEXAcbukaeFcbGAC3EFPXBBPC2MTe9N2lzM95W6JVxv4PjNmzcrAZ02bRqtXbuW1qxZgwO1jx2jKVOmmPZJpAtBQNrEE+IWE8+SJsb8ruidj\/2SvPFztXDyUm0cQQ9cGCRNjE3uDwJqQqlImlwe6L59+xBM\/jI3aYPCQpeIvQgwdo88bYx1wIW\/nBqmdTf\/isb9tFtFCoor6IEL4mljXOweIaDFCBn+HcHk84OSNigMTZ6qZGDs3hxpY8xBD36zYxNN+v0BGqm4iWYs6yIOfCD5ShvjYiwhoMUIpfDv0owmbVCk0ORFmwTGRRFFTpAGxt6gB3xD2tt8cO+H6MU33iXb74pGhhawgDQwDtJkaXMx31tqnoH6vU4\/eFdHmkkzmrRBEWQApSUtGLu3RFKMTTcF6YALvDuXd+lKvJJiHJaVtLk4VQLqhe4NhMCf8+986Xc7wxooVz5pRpM2KGzaKq6ywNg96bgZh4kUpN8VbZ03hVrnRT+f0z3V7BriZhz1\/qTNxakUUO8u3MrKSmWTXJ9FNZbOL81o0gaFLTvFWQ4Yu6cdB+Nc3ubkZ44Furlir7kEKizmxHEwtnlL0ubiVAqodxeu9jg5QAJHHOJ4tvneAw1rSGlGkzYowtolyXxg7J6+K8ZaNC8c3ke8McjGTlqpIuqKsaveIW0uTqWAao9zyZIlxAEQ+HL1\/JPLlmY0aYPC1WBzWS4Yu6Q7WrZNxvlEc8wdddZ20up3RSdXjqE0HM5tYiGbjE3qi5pG2lycWgGNaogg+aUZTdqgCGKLtKQFY\/eWiMo4l2hW3FBL4+Y0WhNNP4U0HM4dxDJRGQepy0ZaaXNxKgU0325cV16oNKNJGxQ2BlbcZYCxe+JBGbNg8nXuZ310\/jd76cLhvWp51rVo5iIRx7miNiwQlLGNOqOUIW0uTqWA5jIA78Llcz31IdjFjMQntbS0tND69etVAHpvnNyurq6s3bzSjCZtUBSzVRr\/DsburWLKmEPpacHkVulnmjaXZ8PcrYTXXEwZh7l\/F3mkzcViBDTILly9Cenll1+mbdu20cSJE3Eai4veXsJlSpt4JJrCy1gvx\/J98OYfDqfHHqZXMJM+nDoX4+5dw9S96xil9TUXaf0YAupoJHvP9NSvtuSrir3VU6dOEQtoR0cHnT59GueBOrJLqRYrbeKRZIdTTzeNLsW++Vuq+MPbKlC7vvSS7Acn1RJ7mBwZiD9L85XmHbrS+jEE1EJPz\/cM1L\/0mqsqXrrt6+uj5cuX0+rVqzMCqo9C4zxtbW00e\/bszDKuNKNJGxQWukTsRYDxKHJePtWXV+j0Z+wpeq+L74z+niut9ibV\/zfU0oWxE2nClI9TxQ01zjb9xNVx0iqi0vqxtLmY+1dqQvlF7ey8dPv444+rg7e9y7bsgRYS0KH\/clXUqpEfBGQTmFBt1v5KX7oJt2Tnu1zOVfz\/3z9QsMyTJ09SdbVhvWatSzTVyB8uUn3fSaq6roIGG9NxX1IYz507N2O7oaGhRO0YtPLUCahpJCL2NpuamlSABT44+6GHHlKeJ\/+ur6qqKvrGN75Bzz33HG3dulUFYWAPtKGhIbMhSdq3HmnfKoN2yDSkB2P3VihFxml7zUUaY2lzcao8UL35Z3BwMOfoZZE0jUTEItzc3KyWcLGJyP1kWGo1SJt4JPIvZcb6NZekT3ORxhgCamEkB9lxm686r4D6X2Pp7+\/Peh1GmtGkDQoLXSL2IsDYPfJSZ5yG11ykMZY2F6fKA9XCuXLlSlq1alUmjJ8eygikMEpC2qBwPxXbrwGM7TP1l1gOjJN+zUUaYwio+3FnvQZpRpM2KKwbLIYCwdg95HJhnOQOXWmMpc3FqfJAvUOW3+Vsb2\/PGsXwQOGBup\/WwRiM7RNISkQhoPZt6S8xlbtw+SSW1tZW49B9UTBJ+9YjbVBEsU1SecHYPflyY8w7dOc\/84oCy5uL+FQX15c0xtLm4lR6oDY2EQXpmNKMJm1QBLFFWtKCsXtLlCNj72sucRyJJo2xtLk4lQLKjeIlXL70gdouh7M0o0kbFC5t56psMHZF9v1yy5lxXK+5SGMsbS5OpYDiOLPCk5e0QeF+KrZfAxjbZ+ovsdwZx\/GaizTGEFD34856DdKMJm1QWDdYDAWCsXvIYEw08Ou3iYW04e6b6OmG6dahS2MsbS4W5YFyYzk0Hx9RxsERbF3SjCZtUNiyU5zlgLF72mA8ytjlDl1pjKXNxakUUP0MdHh4WO3E1b\/z\/3yoNgeGf+qpp6yNcGlGkzYorBkqxoLA2D1sMH6fsSsRlcZY2lycSgEtFEyeoxRt2LABAjplivsZroxrkDbxSDQVGGdbzUUgemmMIaAWRrIOKs\/LtV4PdP\/+\/dTS0qJOVtGfW6iOpBlN2qCwYaO4ywBj98TB+ErGtkVUGmNpc3EqPVBulH8nLkch2rhxI61fvz7rMGx\/F\/Se6OJ9XnrgwAFavHixSu4\/mFua0aQNCvdTsf0awNg+U3+JYJyfsa3XXKQxljYXp1ZAww7f7u5uqq2tVe+Psmju27ePli5dmjnajMvt7Oyknp4eqqysVNVIM5q0QRHWlknmA2P39MG4MGMbgeilMZY2F6dWQFkIN23alNXDisXCzRfBiIWUy8OB2u4nxVKpQdrEI5E7GBe3WtTXXKQxhoAW7xNFU3jP8jx48KDaeXv8+HGVr1BkIi2gnI4P5dZLuKdPn1Y7d\/kwbr7a2tqyloHZaPravXt30fYlneDkyZNUXV2ddDNKun4wdm9eMDZj\/PJbF+irO96mv7tlDD274CazTJdTSWE8d+7czH0NDQ0FusekE6cymPzatWtpzZo1dOTIkcwyrP5ML736wR09epSamproiSeeUEHoORwgbzxasGAB7dixo6CASjKatG+VSXfwMPWDcRhqwfKAsTkv3lx052O\/VAHog8TQlcYYHqh5n8ibkjcC8asq\/OySvUcWxZGREfIv4WrB5L\/V19erHbqrV6+mjo4OFWhBL90uX76cent7sYRrwTblUoS0iUeiXcA4mNXC7NCVxhgCGqxP5E196NAhuuaaazJCuGrVKqMIRN5NRNoD9QorV4hNRJaMVMLFSJt4JJoCjMNZLcgOXWmMIaDh+oS1XN7XX7weq\/c1lv7+\/qxzRqUZTdqgsGbcGAsCY\/ewwTg8Y9NA9NIYS5uL2YKpewYavluFyynNaNIGRTirJJsLjN3zB+NojE1ec5HGWNpcnCoBzXeMme5mxV5jCdsdpRlN2qAIa5ck84Gxe\/pgHJ1xsRi60hhLm4tTLaD+pdbo3S13CdKMJm1QuLKby3LB2CXd0bLB2A7jQiIqjbG0uThVAurtTrlC+XEghHyvsETpitKMJm1QRLFNUnnB2D15MLbHmHfozn\/mFVXgzq\/NVK+7SPySIm0uTq2A+ruWN5qQbRGVZjRMPPYmnnwlgTEYuydgt4Zcr7lI68fS5uLUCqj3HU9uJL\/nyZGExo4da7fXIRaudZ6lUKC0iUciczB2YzXvay63fOAsTRF09CEENEKfyPcKSoQijbJKMxomHiOzRkoExpHwGWUGYyNMoRLp11we\/cz1tPI\/fjxUGUlkkjYXp8oDxS5csy6LiceMU5RUYByFnlleMDbjFDZV1ED0YeuNkg8CGoVeQnmlGQ0Tj\/uOAsZg7J6A+xp+8OJrKhD9vR8eTzsfnum+wog1SJuLU+WBRmQfOrs0o2FyD21q44xgbIwqdEIwDo3OOCMzfuuvE9QO3aCB6I0rsZhQ2lwMAcUmIovdv3SKwuTu3pZgHB\/jMIHo3bfuyhogoElQ99Tp3b2bLxZuV1dX1rmi0oyGicd9JwNjMHZPwH0N3n4sQUSlzcUl5YHyMWjew7L5ZBa++Fi05uZmdcwZX9JPY5HYydxPFXZrAGO7PHOVBsbJMA5ymov7FmbXILFPlFQwee9xZvrnmpoa4p85khG\/R8oi29DQkDmRRZrRpLU37kFooz4wtkGxcBlgnBxjk0D07luHJdwkGBetk8Vy06ZNpGPpchSjgYEBFYiBL6+Xyr\/zQMYFAiAAAuVE4M+T76E\/3vXfaPz\/XpKq2x4aGkpVe4o1pmQ8UP0eaWtrq\/Iu9RJuXV1dQQEtBgh\/BwEQAIFSJMCB6O+9fXwp3lps9yRWQL0bhjjU30MPPURPPvkk9fT0qKDzOn7u8uXLqbe3N+8SbmykUREIgAAIgEBJERAroH4r+D3Q7du30\/79+6mlpYVWr16ddxNRSVkTNwMCIAACIBAbgZIRUCZm8hpLXOeMxmZBVAQCIAACIJAIgZIS0EQIolIQAAEQAIGyJAABLUuz46ZBAARAAASiEoCARiWI\/CAAAiAAAmVJAAJalmbHTYMACIAACEQlAAGNShD5QQAEQAAEypIABLQszY6bBgEQAAEQiEoAAhqVIPKDAAiAAAiUJQEIaFmaHTcNAiAAAiAQlQAENCpB5AcBEAABEChLAhDQsjQ7bhoEQAAEQCAqAQhoVILIDwIgAAIgUJYEIKBlaXbcNAiAAAiAQFQCENCoBJEfBEAABECgLAlAQMvS7LhpEAABEACBqAQgoFEJIj8IgAAIOCZw8dTwFTVcfOfKz3Siv+RIr\/928Z3joVqbqw2hCiqQadLD22wX6bS8shfQ2267zSlgFA4CIAAC8ya+pyDcePVFuvFv\/5IBctPfXsz8zH\/jy\/uZbXJv\/7kidJH\/70\/h85pW+p\/\/+bxp0lSkg4DedhsNDQ2lwhgmjWDBl9Rek3tKWxowdm8R6YwvHN5L2su7cHhf5mf2CvN5ahWTajNgK24Y\/fmD3s8u\/1xxQ02WATjNmDvmBDaKNMbS2ssGgYAKEySJnSzwyE84Axi7N4AUxuf2\/iOxQPLFgsnC6b1YFFkMtRCmaQlSCmPNU1p7IaBEJM1ox44doylTprif4cq4BjB2b\/y0MWav8fzhvUosvUKpvcaxlz3AMXfUKbEM4xG6p5pdQ9oYF7t\/aXMxBBQCWqxPl+XfpU08Eo2UJGO9xHruZ310\/jcsmqNepdeblCSU+eyfJOMwfRICGoZawnmkGU3aoEjYvKGqB+NQ2AJlipux18PkZVktmOxZsliOm\/OVQO2XkDhuxlGZSJuL4YHCA43a50syv7SJR6IR4mCsRfPc3j7lZbKHqQVTyjJsFNvGwThK+\/x5IaA2acZUljSjSRsUMZnRajVgbBVnzsJcMS4kmqXoZRaylCvGrnqHtLkYHig8UFdjQXS50iYeibBtM2bh5GeaZ7\/\/aJanWW6i6e0Lthm77mcQUNeEi5R\/4MABWrx4cSZVV1cXLVy4kLyf6890ImlGkzYoEu4SoaoH41DYAmWywTiXt8mCOeHBNYHaUqqJbTCOk420ubjkPNDt27fT8PAwtba2Zux+5swZam5upo6ODvVZZ2cn9fT0UGVlpfpdmtGkDYo4B6CtusDYFsn85URhnMvbHDenUcSrJe7Jvl9DFMZxtlOqM1NyAtrd3U21tbXK69QXe5\/8+datW2ns2LHU1tZGDQ0NNGvWLAhoEqNEQJ3SJh4BSK9oYhjGfuFkb3PcfY1qyRbXlQTCME6SozRnpqQElD3NJUuW0KFDh1QfmDFjhhLNI0eO0MDAAK1bt059zgI6e\/bsjMh6Y+Hu3r07yf5jVPfJkyepurraKC0ShSMAxuG4BckViPHZk3TphaeIXvoh0YRquqr9X4JUVbZpAzFOkNLcuXMztUsLU1qyofx4OXf\/\/v20YMEC2rFjR0EBlWQ0ad8qExyXoasG49DojDOaMh559L7MKyj8bLOcNwUZw72c0JRx0HJdpYcH6opsiHL10u3y5cupt7cXS7ghGJZrFmkTj0Q7FWLMgQ68725COMNZWFo\/hoAGtDN7ie3t7Vm5\/LtkTYvkJdy1a9fSmjVr1AYhfu7J19KlS7GJyBQi0ikC0iYeiWbzM\/bvqFXPN7ExKJJppfVjCKihufVrJbnEUotqf39\/ZqOPYbFZr6voZ6Aspt7XWPzlSjOatEFhars0pQNj99bQjPXGIB1ejyMFpelEE\/ck3NUgrR9Lm4vZcrE\/A2VPcXBwkBobGwv2nL6+vqJpbHQ9aUaTNihs2CjuMsDYPfFjB39O41\/fkxX4gJdqsaPWHntp\/VjaXJyIgNrrHnZKkmY0aYPCjpXiLQWM3fHmmLT8fJM9ThZLvIrijrW0fixtLk5UQP2vnXi7UVVVFW3bto2mTp3qrneblx0bAAAXmklEQVRdLlma0aQNCucGdFABGNuFmiti0F\/\/xx6ca2sX8xWlSevH0ubiRAWUK\/dHDuLf+aqpqVHvbj711FOOuxgiETkHLLACaRNPGhH7jw\/TJ6Ho55tg7N5q0hhDQAP0Cf+uWc6qP1u5ciVt2LABApqDp7RBEaBLpCYpGIczRS7RrLiBl2kbr3h\/E4zDMQ6SSxpjCGgA654\/f15FBeLlWh27Vgc\/aGlpoeeeey4rpm2AogMllWY0aYMikDFSkhiMzQ2Ra3mWRXPsx+YUDOoOxuaMw6aUxljaXJz4Em6u8HsbN26k9evXZ4XbC9uBTPJJM5q0QWFig7SlAePCFuENQBcO76Pzh\/cSC6j3oGrTSEFg7L7XS2MsbS5OlYBu2bKF9uzZo15diWPzkO6+0owmbVC4nybs1wDGo0y9S7J\/OTWsQurxxYJp4mUWsgwY2++3\/hKlMZY2FycqoHoJlwO78xFkdXV1yv468DufnGLrwnmgtkiWRznSJp6oVmFhHBXIfaoov1jyZxzgYMwddfTBSbVWjg0rN8ZRbRQmvzTGENAAVvZuItq8ebMS0GnTpmWF4wtQXN6kOA\/UBsXyKkPaxFPIOjrCz8V3jiuPUgvkxXeGM79rr1ILJf\/PYmm6HBumd5QS4zD3H0ceaYwhoAF6RS4PdN++fTQyMqJOTrHlgeI80ABGQVJFIG0TD4sgCyBfWgS1EGqT+QXRb0od4Yc9SS2YFTfUWPMog3adtDEO2n4J6aUxhoAG7FX5zvDk+LW2LhbQQueB9v77cfQPD\/yDreqcl3PuvXM07tpxBevhJThc4QlcGHmdKioqAhXgFbZAGQ0S+8Pb8fNHvng5VV+jzyVr1K8uPUeD5holkTa5G91UyhJJYwwBTVkH4uYUE9D\/9bn3n7XOuPPOFN5BdpP+9Kc\/0dVXX12wnb\/\/wI2pv48kGjhy7qJRtW\/SJKr4wAeuSFso\/+8\/cFPeskcqCttD5x2pyF1G1XXvi\/nN497\/mT+v8vz+1X833uj+0pBIymHPaWAVtg1SGONA7QAWLhTCj4vxnqISoNi8SbGEa4NieZWRxm\/u3btGVxVOnDmv\/n\/zzIWMUU6cvUAnPL\/7rTW5coz6aPKEMXRr5RiaXDmW+LOGu\/OLvmuLp5Gx63uOu3xpjOGBBuwh+UL5LVy4MGBJ+ZNjE5E1lGVTkLSJp5hhvOLLwptLcFlQWWDvuX0Ctc57f2m4WNlh\/15qjMNycJlPGmMIaIDeUCiUnz4UO0BxBZPiPFBbJMujHGkTjw2rsMj+4vWzWeKqPdeGu2+mez48nu693d4ScTkytmGnIGVIYwwBDWBd7y5c7XF2d3db34VbrEnSjCZtUBTjn8a\/lztjvRw88Ou3lXkGfv37zBLx6NLvzZG91HJnHEe\/l8ZY2lzMNoz9QG1vx4ljF26xjirNaNIGRTH+afw7GOe2youvv0u\/eONd6t51TCWIIqZg7L7nS2MsbS5OXEDdd6HiNUgzmrRBUdwC6UsBxsVtwl4qe6jaO2UxbZ03xXhjEhgXZxw1hTTG0ubiRASUvc7BwUEV87bQ1dfXVzRN1A7G+aUZTdqgsGGjuMsA42DEtZiyZ2rqlYJxMMZhUktjLG0uTkRAuVK9qaerq4v8O255Z257ezv19\/fTrFmzwvSbQHmkGU3aoAhkjJQkBuPwhuDNSFpINy6annfjERiHZ2yaUxpjaXNxYgKqO4AWS2+HyCWqph0mTDppRpM2KMLYJOk8YBzNAl6PlN81fbph+hUFgnE0xia5pTGWNhcnLqAmncB1GmlGkzYoXNvPRflgbIcqC+n8Z15Rhfm9UTC2w7hQKdIYS5uLIaB4Bup+FAusQdrEk3bEelmXNxnpIA1g7N5q0hhDQN33iYI1eAMmcEK9HIzzQBM2jLDqpU08EvDyjt2HB15Tu3R5SReM3VtNGmMIqPs+UbAGf2hAToxQfgkbRWD10iYeKYj5PVJe0j3z5H0Q0BiMJq0fQ0ADdgpvIIUtW7bQnj171KsrU6dODVjSaHKOZFRbW5u1sxfB5EOhLOtM0iYeScbSIlo\/\/VrqW3q3pKaLa6u0fgwBDdDFch2ozdn12Z1BD9TOF9XoyJEjBc8DlWY0aYMiQJdITVIwdmsKLaLeZ6JuayzP0qX1Y2lzMfeqxEL5eYPJb968merq6mjatGm0du1ashFMnpdz9+\/fTwsWLKAdO3bQunXr1Chqa2uj2bNnZ7xUNpq+du\/enfqRJuWMv9SDLNBAMHZvvX9+9U165Od\/pWcX3ER\/d8vocWu47BKQ0o9xHmgIu+fyQPft22ccTP7o0aPU1NSk0tfX1yuB9Hqteul2+fLl1NvbS1u3blV\/ZwFtaGjIBGmQ9q1H2rfKEF0j8Sxg7N4EzPifjlylgi68uvqTKoIRLrsEpPVjaXNxoh4oV24zmLz\/eDR+HsrX0qVLqbm5mTo6OtTvnZ2d1NPTQ5WVlep3aUaTNijsTgnxlAbG7jlrxvOfHn1PdOfDM91XWmY1SOvH0ubixAVU75plkVuyZAkdOnQoUgg\/7+sqM2bMUF4nCyXOAy2zmSPi7UqbeCLebiLZNWMOtnDnY79Ur7bwKy647BGQ1o8hoAFsz0u4jz\/+uNp1e\/DgQRoeHlbPKzmI\/COPPJK1HBug2MBJpRlN2qAIbJAUZABj90bwMtbviO782kyrh3a7v4t01yCtH0ubixP1QP2biPj1k89+9rPWNhGZdm1pRpM2KEztkKZ0YOzeGn7GWMq1z1xaP5Y2FycqoNoD\/cIXvkCbNm3KPKOEB1p4IEkbFPanBfclgnH8jPWrLfBC7bGX1o8hoAFtr59NLlu2LGuzT9hACgGrV8mlGU3aoAhjk6TzgLF7C+RizKH+fvHGu2pXLq7oBKT1Y2lzcaIeaPTuYacEaUaTNijsWCneUsDYPe98jCu\/+TNCgAU7\/KX1Y2lzceICyq+a8PKt9\/LunrXTjQqXIs1o0gZFHDa0XQcY2yZ6ZXn5GOsNRXg3NLoNpPVjaXNxogLqDfLOu3Bramro+PHjqtcsXLgweu8xLEGa0aQNCkMzpCoZGLs3RyHG\/FrLPR8en\/MgbvctK50apPVjaXNx4gKqw\/ZxvFqOQsTvg9oK5Wc6DKQZTdqgMLVDmtKBsXtrFGKM11rs8JfWj6XNxYkKKO\/C3bBhgxLN06dPZ8LyBVnC9R9f5o1s5A3vh\/NA7QzIcilF2sQj0S7FGPNrLbdWjoEXGsG4xRhHKNpJVghoQKwceeiaa65Rx5exyK1atYq2bdtmdJyZfn7KO3hbW1tVzfo4s\/nz52di3nKAeoTyC2iYMk8ubeKRaK5ijPFaS3SrFmMcvQa7JUBA7fLMWxp7nvzMlJd9+WIB1d4n\/zxr1izS3imf8sLCimDyMRmnBKqRNvFIRG7CGMEVolnWhHG0GuzmhoAG5Mki197enpUryBKuDhivBVR7muzRBjnObGhoKGDLk0subVAkRyp8zWAcnp1pThPG8EJNaeZOZ8I4Wg12c0NAA\/D0e4wBsmaS2hJQXSDOAw1jhdLLI+UcRcnkTRl\/dcfb9PtzF2mwsVry7SbSdlPGiTTOUynOAw1hAf\/xY\/mK0OeGDg4OUlVVVdYzUr+A8okuWMINYQxkySIg7Zu7RPOZMsZpLeGta8o4fA12c8IDDciTl1n5Cvvep1dAuRxsIgpoACTPSUDaxCPRjEEYd+8aVgdvn3nyPom3mlibgzBOrJGeiiGgBlbwH6LtzxL2GSiX4y3buzsX54EaGAZJMgSkTTwSTReUMYIrBLdyUMbBa7CbAwJql2cspUkzmrRBEYsRLVcCxpaB5iguKGMEVwhuk6CMg9dgN4e0uZjv\/qpLly5dsouheGlHjx7NBE7wBjwontN+CmlGkzYo7FvMfYlgnE7GeK0lmF2k9WNpc3EiAqo3BTU0NKj3NfVzy7DPQYN1qStTSzOatEER1T5J5Adj99TDMsZpLea2CcvYvAa7KaXNxYkIqH\/3LXuju3btohUrVti1hmFp0owmbVAYmiFVycDYvTnCMsa7oea2CcvYvAa7KaXNxakR0L6+PnrkkUdo7Nixdi1iUJo0o0kbFAYmSF0SMHZvkiiMcfC2mX2iMDarwW4qaXMxBJSIpBlN2qCwO8TiKQ2M3XOOyph35U6eMIZ2PjzTfWOF1hCVcdy3LW0uTkxAOeABB5LPdQV5jcWGgaUZTdqgsGGjuMsAY\/fEozLWARbu\/fB4iGgec0Vl7L4XZNcgbS5OREDjNkqx+qQZTdqgKMY\/jX8HY\/dWscFYPw9tuPsmHHuWw2Q2GLvvCe\/XIG0uFi+g\/vNAvQET+Oa6urpUlCOcBxrnMJBfl7SJRyJxm4x5Zy5E9MpeYJNxHH0MAhoH5ct15DoP1C+onJR3\/eI80BgNUwJVSZt4JCK3yVh7oljOze4JNhnH0ccgoHFQJlJHlfnPA+Wqc71Tyt4nzgONyTAlUo20iUcidtuM+Zno\/GdeUSh2fm0mTa4cIxGL1TbbZmy1cTkKg4C6JuwrP9dpLHpzkt6MdOTIERoYGKB169ap3G1tbTR79uxMAHtpRpM2KGLuElaqA2MrGAsW4oIxi+iKgdfoxTfepdZ5U6h1Xq37G0lxDS4Yu7xdaXMxs0gklJ8tI\/hPY\/GWG+RAbZ0P54HasozscqScoyiZskvGg6+9R4\/+9F+p6roK+uonxlP99GslowrddpeMQzcqR0acB2qTpq8s0\/NA\/U3QS7fLly+n3t5e2rp1qwrUwB6oDiPIeaR965H2rdJh13BWNBg7Q5sp2DVjrzfKy7mvrv6k+5tKWQ2uGdu+XWlzcUl5oP4Qgdo7Xbp0KTYR2e7pJV6etIlHojniYsxCymeJ8mkuLKQNd99cNku7cTG21f8goLZIGpbjX8L1vq7iDciA80ANgSKZIiBt4pFotrgZs5CyiLKYaiEd\/f8mifiM2hw3Y6NGFUgEAY1KMIH80owmbVAkYNLIVYJxZIRFC0iScfeuYfrF62fVZiO++PWXe26fUHKeaZKMi3aAHAmkzcXil3DDGMmfR5rRpA0KGzaKuwwwdk88LYxZTPliz5Qv9kpLJcZuWhib9iZpczEEFJuITPt2WaWTNvFINE4aGetlXq+galHl\/6U9P00j40J9FQIqcCRLM5q09grsEuJ2ZoOxWwLaS\/Uu++oadcAG9lpvZe+1cmxqloKlzRXS2gsPVKAHKrGTuZ3e7JcOxvaZSn90kosIb0pir5WvE2fO05tnLtCJsxcyn\/nzeKMjseDyxaI76uWOnoVsM\/iDtH4srb0Q0MsC6n66QA0gAALlTuDCR\/9TBsG\/XTNR\/fxv11xf8DMXzP7mj\/8auti\/+ePp0HlNMr7zP\/+7SbLUpBEdiSg1FNEQEAABEHBIQHu63ipyfab\/\/ubZUc\/YfxXKY9J89rRdXk83THdZvPWyIaDWkaJAEAABEACBciAAAS0HK+MeQQAEQAAErBOAgFpHGn+BHC\/4+9\/\/Pv3ud7+jhx56iKZMmRJ\/I0q8xrNnz9L3vvc94gDdixYtojvvvLPE7zi529u5cyfdeuutNHPmzOQaUaI1X7x4kX7wgx\/QSy+9RF\/60pfAOKKdIaARAaYh+09+8hOaOHEiTZ06VU3yX\/nKV1TgfFz2CPDpPh\/96EfVv2effZa+\/OUv04QJE+xVgJIUAT5+cPPmzfTAAw\/QrFmzQMUygZ\/+9Kd09dVX0yc+8Qn68Y9\/TJ\/73OcwV0RgDAGNAM91Vg6Q39zcTB0dHUoc+eKJvL29Xf3c39+vJhme0D\/\/+c+rb+19fX1UX19PlZWVrptXEuWbMtY3y57od7\/7XVq2bBlde215HpMV1PCmjN977z360Y9+RDfeeKNiCwE1J23KmPsuC+jhw4fVl8CPfexj5pUg5RUEIKAp7RRHjx6lpqYm1bpt27YpAeXPOjs7qaenR31T1weF8\/8smpMmTYKABrBnEMbs0bN4btiwQR2HN23atAA1lW9SU8ZdXV20d+9e+shHPkKnTp1SwCCgZv3GlPG6devomWeeoU996lP08Y9\/nDZt2kSNjY1YSTHDnDMVBDQCPFdZ+dskL2Px8sqjjz5K69evVwKqDwnngcDPPbV3yt8m+ZtkbW2tEtCFCxfSdddd56p5JVFuUMYf+tCHlE14efyWW24pCQaubyII41WrVtFrr71Gb775Jr311ltqUv\/6178OL7+IkYIw5pWsF198ke655x41n\/AX8y9+8YtYrYowECCgEeC5zsrfLFtaWrIEdHh4mFpbW4kHzpIlS9TPvHTLy7g86fDAuP\/++103rWTKN2HMNnjllVfUxH799dcTi+mDDz6Iyd2wF5gw5n6sPU4+fhAeqCHcy8lMGfPyOAsnr1aNGTNGrXJVVFQEqwypMwQgoCnuDKaDAktd4Y0IxuHZmeYEY1NS4dOBcXh2UXJCQKPQc5w316DYv38\/+Zdw9QYjx80pyeLB2L1ZwRiM3RNIpgYIaDLcjWr1Tzz5NhHhlRUjnDkTgXF4dqY5wdiUVPh0YByeXZScENAo9Bzn9Q8Krk6\/xlJVVZXZneu4GSVdPBi7Ny8Yg7F7AsnUAAFNhjtqBQEQAAEQEE4AAircgGg+CIAACIBAMgQgoMlwR60gAAIgAALCCUBAhRsQzQcBEAABEEiGAAQ0Ge6oFQRAAARAQDgBCKhwA6L5IAACIAACyRCAgCbDHbWCAAiAAAgIJwABFW5ANB8EQAAEQCAZAhDQZLijVhAAARAAAeEEIKDCDYjmgwAIgAAIJEMAApoMd9QKAiAAAiAgnAAEVLgB0fz4CfBh5m1tbTQ4OJhV+bJly9T5rJIvjlu7a9cuddYs3+Ps2bPVAe186ftuaGjInN3pvVf++4YNG2jp0qU4pFlyJ0DbjQlAQI1RISEIjBLQQuIVl1Jg4xVAPuEnqIAyAy3AK1asKAUkuAcQKEgAAooOAgIBCRQSUD4th89sPXHiBC1atIjuuusuampqopGREZoxYwZt3bpVeWcHDhygxYsXE5+qM2fOHBo3bpzy3NjzYy+WD0nnsoaHh9Xv+hQebqr2dLmMTZs2qdbv27eP6uvr1VmxLH7d3d2Zv\/X396s0AwMD6u98sTj6PUku7\/jx48rjzHWPXg+U69N1c3neujdu3Ejz5s0jnFMbsGMhuTgCEFBxJkODkyaQawlXi+MLL7xAzz\/\/vBJKvpqbm6mjo0OJiRZEr1ByPhYzFtJ8AlpXV5dT\/Lj8VatWqWPtJk6cmBFf\/pwFlNtw+vRp6uzspEceeYS+853v0Jo1azKf9fT0ZC21ch6ui8U73zI1l82CzGn05c3Hn\/F98qWXfpO2F+oHAVcEIKCuyKLckiVg4oGyp3fy5MmM96lhsGAuX76cent7M95oLmH1eqC1tbXU3t6exZO9UBY7LZR6yZW9SvYitefqzaSFTnus3ue1fE+PP\/44NTY2KrEv5oFqAfWLJ5fNnix7qNKfB5dsB8aNWSMAAbWGEgWVC4EgAsren9\/TY4HRwsfLuSYCmksQveWYCKgWNraT9jS1zcIIaD5PEwJaLiMB9wkBRR8AgYAETAWU0\/EzTX4WysuZ+vloS0sL8SYb9tC8S7grV67MbNyZP39+ZmmXxU4v1VZXV2fS1NTU5PRA\/Uu4XN\/69euJ8\/Iu2d\/+9rdXiLrO41\/CzbcLl73cfMu0WMIN2KGQXCwBCKhY06HhSREwFVD2CnlXqukmIu9mIe\/mokKbiHIt4erlX73s29XVlXke6d2Y5Ofn9RwLLeHef\/\/9agn60KFDmSL0M2C+Z+9ScFI2Qr0gEAcBCGgclFEHCBQgUEjUbIKL4z1OvMZi02IoK+0EIKBptxDaV\/IE4hDQM2fOqOXkyZMnZ151yQU2yvNL\/3PUkjccbrDsCUBAy74LAAAIgAAIgEAYAhDQMNSQBwRAAARAoOwJQEDLvgsAAAiAAAiAQBgCENAw1JAHBEAABECg7AlAQMu+CwAACIAACIBAGAIQ0DDUkAcEQAAEQKDsCUBAy74LAAAIgAAIgEAYAhDQMNSQBwRAAARAoOwJQEDLvgsAAAiAAAiAQBgCENAw1JAHBEAABECg7An8f+ISagLAg6\/TAAAAAElFTkSuQmCC","height":280,"width":464}}
%---
%[output:0fd0717d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.076483869223994"],["18.982392004991411"]]}}
%---
%[output:44396bd3]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:8de91a74]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:360d2452]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000050000000000"],["-4.934802200544680","0.999214601836603"]]}}
%---
%[output:0c11bd1d]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.075698471060596","12.858521001586354"]]}}
%---
%[output:2fa74918]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:79eff6bc]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:6d3e5d2e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000050000000000"],["-4.934802200544680","0.999214601836603"]]}}
%---
%[output:14c546cf]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.075698471060596","12.858521001586354"]]}}
%---
%[output:075d78cf]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:6e51d2e7]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:57b55c06]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.014919077108659"}}
%---
%[output:616fcdfb]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.620334234422426"}}
%---
%[output:3623cefa]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.181651947592838"}}
%---
%[output:14798efd]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     1.108704262459437e+02"}}
%---
%[output:2059d561]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -1.377465560618278e+02"}}
%---
%[output:5c304d9d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAdAAAAEYCAYAAADs5qfZAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQuQXsV15sEhWQaIkUa8PBaKZDSycRyPkIJLlnmEaAmVbEk43s2ORluJazL2joMFWyupZiQwVDAgjaYktljAZZVrPFEeesQOrKXspgiWMQUetBAJJskao0FPxAhbSMhgW06WtbbO1fSop+c+uu+59\/637\/1ulcto\/j7dfb\/++nz39PO8M2fOnCE8QAAIAAEgAASAgBMC50FAnfBCYiAABIAAEAACAQIQUBABCAABIAAEgEAKBCCgKUCDCRAAAkAACAABCCg40HAETp48SV1dXUE9BgYGqLm5Obc6jYyMUGdnJ82fP5\/6+vqoqakpt7L0jIt8R5sXUjjccccd1N7ebmNS6jTr16+nTZs2BXXs7u6m3t5ep\/pu376d1qxZQ+vWrSslHvx+u3fvzr1\/OIGGxBjCBQcaj0CR4hIloOygbrrpJlqwYEEugES9Y97lRr1MGofMDvyZZ55xEqc0Nq4NwGUsW7Zs3KyKAlq1Dx7XNi5rekSgZW0Z1KsQBE6fPk2rV6+mnTt30pYtWwoTUI58iyg3DETljBcvXmwthipCcxGnNDZpGl0JqEvdzHLKHoEqnh45cgRRaBqS5GQDAc0J2EZlqw9lcR30ISk9+po7dy7df\/\/9QTXZkerDmSpaGh4envS7nsdtt91Gn\/vc54I0LS0tNDg4SK2traGvrguVmd6Mzvh3NaS7aNEieuihh6itrS1wHLrwJOXDQ8Gq3D179gT140cN4d5777305S9\/ORBP9ZhY8N8VprrAKqetp1dOWOWlO3T9HR999FHq7+8PLffo0aNB\/UZHR8frpLehjiNj\/thjj9HXv\/51Uu\/H+JtYK+zU0HiYWKh21ctV72u+l2rr6dOnj38EmPjt2LEjGBJVj84PM7+kDxeTj3F5xfEwLlLVMeE6q7qbomz2Lx1blceKFSto165dxP1HtZ3+zvw3VYbetkm4hPGwUX4G5Z5FAAJaESaYTlN\/LeUEwpyk6fg4HxYvJZ7m72EOPk58+LeouilnN23atAlzoEpA9TqwUIUJni6iZj5ZCWhYhGM6M9OxRuHKf48S0J6eHlq+fPkk7OMEi3+77LLL6Pjx48EHQpiocZm6ozfrHsULVe7evXtDxfDxxx8fn3fU+aYLhCmgZl7q9ygRjeMs2xw+fDhSqPU6meJpfuQo8brhhhvo2WefneAVwkQwrH+ZAshpwurIf1flJOWt41L2KLkirtTpNSCgTnCVN7FyEPoXuHI+XGs9+uIoQ3VM3UHpnV3\/8tYdLouUipBUHqpsM9JRaIX9rjuDW265JVJA9S9013ySBJSjbn6ShlLjImSOik+cODEJEz1qYpzmzJkz4R1thnDN4WWFvWpPjjbNdue68HxgWGTMWC5ZsiR4Xz1itVlYZTMca6Yx\/60wUWLP9U8qW3Ev7H3U3\/hDi985aghXx1HxyWzTp556KhDisIgyKl9zFEJF3XoecWWrCFXxPwmXLIaqy+vB\/KwZBNTPdptU66ivU+WA2HHMmzcvdAWqnubQoUOhUQUXqOfBUY9aMZu0CCjpyzlKoHSHwuW75pOVgOplsxjyozvsKMemC8jnP\/95awENi9jDyuV6mEPUUREep2UhUPXQsQ0rzxzKjhPQqKFr0yYumgz7+DLfTU0PmEKsPhqihC6Jn3r76nlEtasZzSqslIBGDd3rK8x1Lqt+qQ+fqw6u4xI2bVAR9+Xta0BAvW26iRUvWkD1bSBJDspV+PjNwra12Oaji4PpbDlvfRuLTQTKafSFN\/xv3jJhRuCmA3cVUBNHM0o1hTsrAVVMihJuXpkcJqDmUHBSBOqDgIaNeKh2NfkXFYHqeUT1DQhoNRwvBLQa7ThpKE8fHosbwjWHGtWcUtTXfNiQW5KAxg296lER15O\/0qME1DYfHhozxU0NbZsCyiJlszjDFBc9QjOHwVlwkoZwOTpOEiAzj7RDuDrFo6I6sxuYYmhGY+qd9ZEI9T6KO6ZN2BBuUvfLewhXfWypyD1KQMMid4WRGYFGLfoyh4\/jhnDDcMEQbhJbiv8dAlo85rmUmPciojgBShLQuLqFzQ9GCWhSPjzcpeYzTZBtBJRtwlbhqrzMlZT6AQQui4jUUJ5uw+V+5jOfCaLjsIdxCns\/20VEnKf6qDCFO2qBjW6jp+EyH374YXrggQcmLXhiG1NA+W9RC5LUuyZ9sIUNbyaNAOg4Rr1jnPjpgnXnnXdGcisuD65D2OIi20VEOi5JIzC5OBZkGosABLRiBLHdxqJvQUnaxhK2MMllCJchjhseTFqko59MFJcPl2Nuebjnnnvo5ZdfHl80ExaB6s41aiGUnrc5NxsmsLqQ6Lb830pAw8r92te+NuFEHT7cQZ9vTbONRRdC3aGHbXGK2j5j4qrPyXKejNuGDRto1apVARz6SIJaTR21LSZp\/2bcNhYuyzYyi5q75FGIMHGKiroZo7AtRGFRbNTHF\/\/dPPkoai5Z5WEzUlIxd1b614GAlr6Jsqtg0orH7EpCTnkgEDZUbK60jtqHq9cnzUEKebxPHfLUP3jUh0LYytwkLHCQQhJCjfm99ALKnZ33x\/HG8zDnEBaRJH3RNgbqxpcKAW18G0hqEDeEHTf0HFZmmqP8JHWvs23YEC7jkXT4SNhHT1XOLq4KH0otoDYLHnh4Z+XKlXTXXXdFnoJTlcaSvgcEVIpg4+3N4Uyukat4sg3OVi22Lc2pFRfx5Jrig6fY9rItrdQCynMQTBx+oiJQdgRr166ljRs35nqLhy2gSAcEgAAQAAL1QKC0Aspf2vfddx91dHQEIholoEpk874Gqx50wFsCASAABICALQKlFVCeN+CHT+mImwM15xfiVlHagoJ0QAAIAAEgAASSECilgPKw7ObNm+nuu+8mPrw8TkA5OuVl5eo2EfPfJgAf+tCHkjDB70AACAABIFAwAnyDzaxZswouVVZcKQVUv2Q4aRWu+fpJ6VlADxw4IEOtotbAJr5hgQ\/wkXR98Kd6\/CmdgIatMlSwJ90byOmSFhWBxNEkBjbV6+ASh+9qC\/6AP66c0dP7yJ\/SCahLRKm2uSxcuJD4WDX1b14i3tvbG9qWPjaShJQutsAGDtCFL2HTIxjdwQdqWg756H+8E1Alkrw6lw\/tjjvkO6whfWyktIR0tTt48KB3cxCu7yhJD3zi0QM+wEfSv3z0zaUXUEmDQEDd0IMDhAN0Y8zE1OAP+CPhDwRUgl5Btj42UkHQEBwgHKCEa+AP+CPhj4++GRGopMUrZgsHCAcooTT4A\/5I+AMBlaBXkK2PjVQQNIhAE4CGQEAgJH0R\/IlHz0ffjAhU0iMqZosODoGQUBr8AX8k\/IGAStAryNbHRioIGkSgiEBFVIOAQkAlBPLRNyMClbR4xWzhAOEAJZQGf8AfCX8goBL0CrL1sZEKggYRKCJQEdUgoBBQCYF89M2IQCUtXjFbOEA4QAmlwR\/wR8IfCKgEvYJsfWykgqBBBIoIVEQ1CCgEVEIgH30zIlBJi1fMFg4QDlBCafAH\/JHwBwIqQa8gWx8bqSBoEIEiAhVRDQIKAZUQyEffjAhU0uIVs4UDhAOUUBr8AX8k\/IGAStAryNbHRioIGkSgiEBFVIOAQkAlBPLRNyMClbR4xWzhAOEAJZQGf8AfCX8goBL0CrL1sZEKggYRKCJQEdUgoBBQCYF89M2IQCUtXjFbOEA4QAmlwR\/wR8IfCKgEvYJsfWykgqBBBIoIVEQ1CCgEVEIgH30zIlBJi1fMFg4QDlBCafAH\/JHwBwIqQa8gWx8bqSBoEIEiAhVRDQIKAZUQyEffjAhU0uIVs4UDhAOUUBr8AX8k\/IGAStAryNbHRioIGkSgiEBFVIOAQkAlBPLRNyMClbR4xWzhAOEAJZQGf8AfCX8goBL0CrL1sZEKggYRKCJQEdUgoBBQCYF89M2IQCUtXjFbOEA4QAmlwR\/wR8IfCKgEvYJsfWykgqBBBIoIVEQ1CCgEVEIgH30zIlBJi1fMFg4QDlBCafAH\/JHwBwIqQa8gWx8bqSBoEIEiAhVRDQIKAZUQyEffjAhU0uIVs4UDhAOUUBr8AX\/S8mfri2\/SF7e+QicfujltFg2xg4A2BPZyFgoHCAcoYSb4A\/6k5Q8ENC1yBdv5OExQFERwgHCAEq6BP+BPWv5AQNMiV7AdBDQacDhAOEBJdwR\/wJ+0\/IGApkWuYDsIKAQ0LeUgEBCItNxhO\/AnGr1aC+jJkyepq6uLhoeHnfjV1tZGTzzxhJONNDEEFAKalkNwgBDQtNyBgMYjt\/7JQ7T+yYP1XESkBLS3t5cWLFhgxbHdu3fT+vXrIaBWaBWTCAIBgZAwDfwBf9LyBwLa1UUQ0LT0KYcdHCAcoISJ4A\/4k5Y\/tRbQtKA1wg5DuBjCTcs7CAQEIi13MISLIdxIBPQ50O7u7iASLesDAYWApuUmBBQCmpY7EFAIaCJ3eE5z06ZNQbqWlhYaHByk1tbWRLsiE0BAIaBp+QYBhYCm5Q4EFAJqzR1zVW6ZolIIKATUmshGQggoBDQtdyCg8cjxMX68lQVH+Rk4jYyMUGdnJ42OjpYiKoWAQkDTOkEIKAQ0LXcgoBBQCXcCW7V1ZWBggJqbm8X5pckAAgoBTcMbOMBk1PCBgQ+MZJaEp0AEGoGcHoFyki1btljvFU3bGHF2EFAIaFpeQSAgEGm5gw8wRKDW3Dl9+jStXr2adu7cGdgsXryY+vr6qKmpyTqPvBJCQCGgabkFAYWApuUOBBQCmsgdfRVuI6JNjnZ7enqov78\/cvUvBBQCmkjkiAQQUAhoWu5AQOORW\/LYS\/Tc\/lP1XESkr7ptVLSpot49e\/bEbp+BgEJA0zpBCCgENC13IKAQ0EgEXnvtNbrzzjvp3nvvtZ7fzPosXJUfVxIRaLpuDoGAQKRjzlkr8Af8ScufuQ88T0dO\/rzeEWijzsLlCPi+++6jjo6O4IB6CGg6GsMBwgGmYw4E1AY39K9olJpXPB38WMt9oI2+zmz79u0B+PPmzcMcqE1PxhxfKpTgAPGBkYo4Y0bgTzh6HHlyBFpbAZWQSmrLC4c2b95Md999Nx09etRKQFWZu3btkhZfKXvGb\/r06ZV6pyxfBvjEowl8gI9rf1u0aBG9d+mH6SfX90BAXcHLIj0P2d50003B3CtW4coQxRcyIiwJg8Af8CcNf\/gIPz5IARFoGvQENnFDx1EHNmAVbjTgcIBwgILuiEVECeChf4UDpLawvO9nb9FbX\/0DCQULtz3vzJkzZwovNacCEYHKgEUHh4BKGAT+gD9p+KMWEJ3\/1qv0oz\/\/QposGmYDAW0Y9OUrGA4QDlDCSvAH\/HHlj76A6OLn+unIC3\/nmkVD01dKQG2QxBAuhnBteBKWBgIBgUjLHbYDfyajp4Zv+RcIqIYPby1Zs2ZN8Beejzx8+DANDQ01\/ExcCCgENK0ThAOEgKblDgR0MnLPvXaKlnzlpeCH66+eQv+88d\/TgQMHJBAXbptLBMorY\/n+Tz6Xdvny5cQHLLS1tQUHzLe0tAT\/btQDAYWApuUeBBQCmpY7ENCJyPHQ7fKtrwTn3\/LDByj46JszF1C1MpZFcs6cOdTV1RUIJm8zwX2gku6Xvy0EAgIhYRn4A\/7Y8mfz86P0X7\/xapC847or6bGOayCgwZfEyZPjogkBtaVTOdLBAcIBSpgI\/oA\/NvzRh25nNF9AO26\/lvj\/EYGOocfznzzfqQ\/hKjFdunQptbe32+CcSxofGykXIEIyhQOEA5RwDfwBf5L4Y4rny1\/65LiJj7458yFchQYP1y5btmwCnuvWrWuoeHJlfGykJFJm9TscIByghEvgD\/gThQDPef71njdp7d8dDJJwxPno0mvo+tlTIKCSTle0LQQ0GnE4QDhASX8Ef8CfMARYPHm1Lf9\/lHj6GtzkFoFKOmKethBQCGhafkEgIBBpucN2deMPC+b6Jw8Sn3WrHn3O08TSR9+cuYDaXm3W3d3dkO0sPjaSpNO62Natg7tgU0cHCHxcEcAHhkKAV9jySlv9GfjDX6ffv\/bySJB89M2ZCyijw4uItm3bRgMDA9Tc3BwApoSVFxEtWbKkYXtCfWykbLsxItC0eOIDAwKRljtV\/wBTw7P63k6bqFPH00ffnLmA6ttYeO+n\/uj7QPft20d84MITTzwh4aSzrY+N5PySKQ0gEBCIlNQJzMCfevLnr144Rnds+8Gkl48brg1DykffDAGVeIyK2cIB1tMBZkVj8Kc+\/Hnl2E+pY+AfxxcG6W\/OByP03jorWGnr8kBAx9BKGsLlfaBqr+jDDz\/sgrE4rY+NJH5pywzgAOvjAC0p4ZQM\/Kkuf3iI9pGnj9DA994IfUkWy0faP0I3tE514oye2EffnHkEqgAJ2weqLrlm8XzkkUdocHCQWltbUwOextDHRkrznmls4ACr6wDT8MHVBvypDn9YMF9586f02NNHxs+rNd+ORbPjug9Q760zXakSmt5H35ybgGaCaA6Z+NhIOcAQmiUcYHUcYFGc0csBf\/zlD58Q9N4vfkEPPXU4UjD57Vg0v\/qfPkotl\/wb5yHaJE766JshoEmtWqPf4QD9dYBloCn44wd\/1IpZ3qP5vf2nQucx1ZuwYH7q6ilBpMn\/7Tqv6cJLCOgYWiMjI9TZ2RlcaWY+fK2Zvr3FBeAs0vrYSFm8t00ecIB+OECbtmxEGvCnfPxhsVTH6B1663RsdKkizBlTL6CesUVAeQqmiZaPvjnzCPT06dPBHs+FCxeO7\/fs6OiYdLVZIzo4l+ljIxWFFRxg+RxgUW2fRTngT2P5w8OwZ4ho24vHEiNLPcL8nY9OoyUfvzz3CDOJYz765swF1NwHyns9Z86cGRwizwuLtm7dSn19fdTU1JSEZy6\/+9hIuQARkikcYGMdYFHtnFc54E8x\/FFR5fZ\/eJMOn0iOKs3hWN5ioqLNvLiQJl8ffXPuAsorbg8dOhQc24cLtdPQqjgbOMBiHGBxLVpsSeBPtvxRc5X\/7duHaf\/xnyUOv+pCyf99Y+tUWnXL2RWyRQ7FpmUdBHQMOY46+TFF86mnngruCUUEmpZi+drBAWbrAPNtrfLlDv6k4w8Pve59\/R369vdP0JG3z85Z2j7Bwp6pF1D3jVfRb3zwYm\/EMuz9IKBjqOjzoDx0y4K6adMmamlpacjeT72xfGwk284kTQcHmM4BSnGvij34E96SLJAsdH\/z\/Ag9feg9Z5FUESQLZeenPkiXXfwrE+7RrAp\/fPTNmQ\/hlr0xfWykojCFA4SASrhWZ\/6oeclv7v0hHTj+s9Qiyfjf1DqVVo4Nvfoy\/CrhjbL10TdnLqC2h8mrW1qyAN4lDx8byeX9JGnr7ABtcAM+9f3AUAL5z6M\/of\/1T8cDIJ7bf8qGNhPSqLlI3lv5B\/OvpPPfd14lo0lnYDzdIQEBTdPSFbWBQNRXILKgtK\/8UeLI4rbnyDs0OHbeaxqB1CPG3\/5IM3267XKaOe3sjoP\/9+NjNGvW2RWweCYj4GNwk5mA8mrbNWvWJPKiURdp+zxMkAhqRgl8dYAZvX5iNsDHzw8MtSjnF2fO0MO7jgQrWl0X6+hvrqJInpNsv+5KumH2uQPU41a7gj\/x\/Km1gCpo4oZwEz1UAQl8bKQCYAmKQAf3UyCK4kdSOY3gj75i9S92j9L\/PvjjoJppo0c9gmSBXHTNNJo\/4\/1BntfPnpIEQezvjcBHVOGCjX30zZlFoAVjnbo4Hxsp9cs6GqKDQ0AdKTMhedb80cWRL21+fmzOUSKOukDyPOS\/vWZasKpVnfPKZea1ZzJrfCRtVUZbH30zBLSMTGpQndDBIaAS6rnwR23t4GHVR7\/7Ou1786fiyNGMHluvuIh+f+65I+ryFEcb3FzwscmvamlqK6Bq2HZ4eDixTXGYfCJEDUuADg4BlZCP+fNLl3xgPIsXD\/+YvvODk\/Q6H2jueEBAWD30uceZlzbRbW2XU+vlF44nzStylGCi26J\/xSNZWwHNimBF5ONjIxWBC5eBDg4BjUNARY1vvvMv9I09P6RX3\/xpJsJoRo6zr7hwfPUqi2KjI8es+h\/6FwQ0Ky41LB8IaDT06OD1FFC1jYP3JP7lC8foyInTmQmjEkdekHNV8wX0kSsvomuven8hc44NczIRBaN\/QUCtORm2rWXdunXBrSyNfCCgENC0\/PPNAapFOE\/+n7fo5aPvZjaUqvDTh1R\/\/YMX07xp79EHPnDu4uWqRI5p+WLa+cafrN7bNh8ffXMui4hYPLdt2zbh4mw1T7p06dKGiqiPjWRLQGk6dPByR6D6hn9efPP4Sz+i7756Mqh0FnOMYcL44Ssvoi\/ceBUd+\/G\/BD\/HbeUAf8rNH6l\/yNveR9+cuYDiKL+8aZZf\/nCAjXOAPL\/Iz+tv\/5x2HzxFB49nO4yqhlKD\/596AX18+q\/S567\/IL3vvPPG5xili3DAn8bxJz+vUFzOEFAigoAWR7isS4IDzNYBqiHU7+0\/Rc+99nYwhMqPdB+jXkt9GJXnGD81e2ogkEXsa8QQpVsPRP+KxwsCOoYPhnDdOlZZUqODx7fE94ZH6MxFlwXidOjEafrbfzpOPziWzf5Fs2RdGFuvuJC6b5hOP3r3\/yYOozaSS+BPth9gjWzLRpQNAdVQxyKiRlBQVmbdHKBa5KIixadeOUF7j7yT+WIb1SqmKF438xJa+KEpE7ZpSIdRZQyQWdeNP65oAR9EoK6cKV16H79yigKxKh1czSeyGL146Mf0nVez28wfFynyECqfmzrniovGI0Ulzj4Loy3\/qsIf2\/d1TQd8IKCJnCnLatuoikJAo5uwzB1cCdE\/Hn2XOFI8+Nbp4EWyXH0aFinOuqyJ\/t3HLgv2L77w\/YP0iY\/Oyu2s1MTOVfIEZeZPGaADPhBQKx6aw7dbtmyhBQsWWNnmnQgCWg4B1bdkPPn9t2j46E8y38Cvv6l5DNzij19GTb\/8S06LbeAA43sn8AE+Ev\/to2\/OfBuLCeD69etp06ZNwZ9bWlpocHCQWltbJTiLbH1sJNELOxindYD6rRlc3PF3\/5X+7PlROnyimCiRziP6zV+7hD77yRb+z8y2ZZjQpcXHoQm8Tgp8IKASAvvom3MXUB1QFtPdu3dPOGBBAngaWx8bKc17prExHaCKEjmvfT\/8aXCazaG3st+fGDZ0GmzJuHoKzWhuCn7mDfyNnk+EQEAg0vQrZQP+xKPno2\/OXUD1CLTRN7Fw8\/nYSJJOa9rq0eLQ\/lP07Nj+xLznElkQp0+9gJZddyWdl+Hm\/SyxScoLDhACmsSRuN\/BHwioFX+kw7anT5+m1atX086dO4Pyuru7qbe3N7RsM21S+ioLqC6O\/+PlH9G3XzkRYJbXxv1fm9ZE\/3H+FYEgqiiR\/7+qZ6DCAUJArRxgRCLwBwKayJ+4k4gSjccSsADzw6KZtKqXf1+5ciXdddddVnOrVRBQNbS69cVjwZ5FqUCqBTaXNRHNn3Up3ThnKn2s5eLK7E+05V1SOjhACGgSRxCBpkfIR9+c+xBuejjPWeqCauY3MjJCa9eupY0bN1Jzc3Nicb41Eosl72Xc\/Pyos1DqK095OwZfQDz7snMXEDNY+v5ECAQEIrEDxSQAf8AfCX988838rqUX0KSIlhclscAODAx4L6BqCLb\/7w8FWzpsIsvgzNOpF9AnZl1Cf7SgZZy\/aTbuwwHCAUocIPgD\/kj4AwGVoBdiq+ZSFy9eTH19fdTUdHZFpv6Ye06TFipxI6ln165dGdfYPbvRd94LjP7022\/RnjfOHjYe9rS8\/3z6wK+eT7e0XkQfav5lmv\/BC9wLS7A4evQoTZ8+PfN8q5Ih8IlvSeADfFz7+qJFiyaYHDhwwDWLhqYvfQTK6LCQjo6Ohoqo+VtcWs6rTF85vY+P0NeeOxpKAI4g23\/zSrph9tTYOxizZA8iiHg0gQ\/wkfQ38CcevTL5Ztt2zlxA87jOjOc5e3p6qL+\/P3GhUFLaRjcSD9Mu3\/rKpOFZNRT7aMc1weKduIuLbRvXNR06OATClTN6evAH\/JHwp9G+OU3dvRBQl3nOpEVFjWokFsX1Tx6krS++OaGdWDgfXXpNQwTTJAwcIBxgGieibMAf8EfCn0b5ZkmdMxPQsOvLwioWt6dTpddX3ap9nnwMoLkXVP22cOFCam9vp7i0Ku+iG0ltOVnylZcmCeeO268t1cHkcIBwgBJnAv6APxL+FO2bJXVVtpkJqMowadWsTaXNwxH0RUTqt46OjuCA+ri0YWUV2Ugsniyc+gEHZYo4EYHasPFcGggEBMKNMRNTgz\/x6BXpmyXtqNtmLqBZVSyvfIpqJL6TUo86yyycGIKzYxscIATUjinhqcAfCKiEP6WwLUJAw8SzbMO1YY2BDg6BkHRS8Af8kfCnCN8sqV+YbSYRqD5sO2fOHOrq6qLh4eHQuibt08z6Bc388m4kXiT0xa2vjBf7ze42+u0PJ5+QlPd72+QPBwgHaMOTqDTgD\/gj4U\/evllStyjbTAQ0j4rllWeejaRHnj4M2ZoYwwHCAUr6HfgD\/kj4k6dvltQrzhYCmhGypnj23jqLOq67MqPci8kGDhAOUMI08Af8kfAHAko0fntKnYZweZXt3AeeH+cOz3c24iAECXnZFg4QDlDCIfAH\/JHwBwIag57rtWOShoizzaORljz20vjJQr6KJwQ0mXEQCAhEMkuiU4A\/8ejl4Zsl7WVjW+gQLp8otHXr1siD4W0qLE2TdSP99+8coT\/92\/1BtXjI9rGOa6RVbJg9OjgEQkI+8Af8kfAna98sqYutbeEC6nL1mO1LuKTLspHMec+Xv\/RJl6qULi0cIByghJTgD\/gj4U+WvllSDxfbQgU06aYUl4qnTZtVI5mnDLF4prmDM+175GEHBwgHKOEV+AP+SPiTlW+W1MHVNnMBVXtCwxYR8Xm2g4ODiTequL6ES\/qsGmn9k4eCw+H5+ZMbr6IHPz3bpRqlTAuXzZypAAAW4ElEQVQHCAcoISb4A\/5I+JOVb5bUwdU2cwHlCkQd8q4OfXetZJbps2gkfdUtR52+D90qfOEA4QAlfQ38AX8k\/MnCN0vKT2Obi4CGDdWqyHTp0qXBzSmNerJopJXf3EeDQ28Er8CLhnzb7xmFPRwgHKCkX4I\/4I+EP1n4Zkn5aWwzF9CkC7V9X4Vb1eiTyQMHCAeYxolgBMMONfSveJwgoNpBCnx3J183pj8uF2PbUdI9lbSRqrLnMww5dHAIqHuPOmcB\/oA\/Ev5IfbOk7LS2mUeg5vynXjG+dHtoaMjrfaDNK54OXqlKc5+IIOy6DwQCAmHHlPBU4A8iUCv+cKS5atWqCStuR0ZGqLOzkzZs2DApMrXKNKNEkq+ce3e8Ro9+9\/WgJj6fOBQFJTo4BELSzcAf8EfCH4lvlpQrsc08AlWVYRFdtmzZhLpt2bKloeLJlUnbSFWe+0QEateFIBAQCDumIAJNg1Na35ymrKxschPQrCqYdT5pG0m\/57OK0SfjDIGAQEj6G\/gD\/kj4k9Y3S8qU2mYuoGoOtKOjo+HRZhg4aRtJLR6q4twnIlC7bgSBgEDYMQURaBqc0vrmNGVlZZO5gMZtY8mq0pJ80jSSfubtHTfPoPsWXy2pQmltIRAQCAk5wR\/wR8KfNL5ZUl4WtpkLKFeqDKtto8BJ00hrnhihTc8eDbKs6vAthnCTuxMEAgKRzJLoFOBPPHppfLOkPbKwzVxA487C5Qq3tbXRwMAANTc3Z1F\/5zxcG6kOi4cUiOjgEAjnDqUZgD\/gj4Q\/rr5ZUlZWtpkLaFYVyysf10bSh297b51FvbfOzKtqDc8XDhAOUEJC8Af8kfDH1TdLysrKFgKagGSVTx4yXx0OEA5Q4ljAH\/BHwp\/aCqi+cGjOnDnU1dVFYdeZ+TaEqw\/fXn\/1FNrxxWsl\/Ci9LRwgHKCEpOAP+CPhT20FVAJa0bYujaQLaNWHb7kd4ADhACX9EfwBfyT8cfHNknKytM1lCLcq94Hqw7d85yfvAa3yAwcIByjhN\/gD\/kj4AwEdQ68q94FW+eD4MKLDAcIBShwg+AP+SPgDAbW4zsyX+0D11bfbP\/9xuuWaaRJueGELBwgHKCEq+AP+SPgDAbUQUI5OfdgHWoezb02ywwHCAUocIPgD\/kj4AwEloqrcB1qHs28hoG7dHQIBgXBjzMTU4E88ehDQMXyqcB+omv+sw\/YVRWt0cAgEBEKCAPgjQQ8CqqHn832g+vxnlc++RQTq1t3xgQGBcGMMIlAXvCCgLmg1KK1NI\/3F7mP0X\/76B0ENIaANaqgSFgsBhYBKaAn+YAhXwp9S2NoIaB3nP7lx0MEhEJJOCv6APxL+2PhmSf552OZykEIeFc0qz6RG0k8f4q0rvIWlLg8cIByghOvgD\/gj4U+Sb5bknZctBNRAtq7zn4hAk7sYBAICkcyS6BTgD4ZwJfwphW3SV85fvXCM7thWv\/lPCGgyPeEAIaDJLIGApsUoyTenzTdPO0SgBrp1nf+EgCZ3MwgoBDSZJRDQtBhBQMeQGxkZoc7OThodHZ2EZVtbW2lPIqrb9WVm40AgIBBpnR8+wJKRQ\/\/CEG4iS9RJRC0tLdTb25uYvugEcV85dbu+DALqxj44QHxguDFmYmrwBwKayB\/9cu0FCxYkpi86QZyArn\/yEK1\/8mBQpTpcXwYBdWMfHCAE1I0xEFAXvDCEq52F29HRQb4JqH7\/58mHbnZp+0qkhUBAICREBn\/AHwl\/IKBj6PExfkXduqKGjHfu3BmU3t3dHTt0HNdIcx94nngYly\/O5gi0bg8cIByghPPgD\/gj4Q8EVLvObHh4OBTLrBcRsVDzw\/Otavh46dKl1N7eHlp+VCPp8583tE6lb\/3JXAkXvLSFA4QDlBAX\/AF\/JPyBgErQy8hWF9SwLKMaST9AoffWWdR768yMauRPNnCAcIAStoI\/4I+EPxBQCXoZ2NosYIpqpLovIGL44QDhACXdEPwBfyT8gYBq6G3fvp3WrFkT\/GXLli10+PBhGhoaor6+PmpqapLgHGrLkeemTZto8eLFsWVwI6ln165d4\/\/9nx9\/k\/a88fPg33vuqF\/0ye999OhRmj59euZtU5UMgU98SwIf4OPa1xctWjTB5MCBA65ZNDR9LicRsZjxIQo9PT20fPnyYH6S5z5Xr15Nee8PVWVHCXXUV06dTyBSDEQEgQhC4o3AH\/BHwh9EoNoiIhbNOXPmUFdXVyCgvKWliNW5fAoSC3d\/fz+1trZOas+oRmpe8XSQ9vqrp9COL14r4YG3tnCAcIAS8oI\/4I+EPxDQEghokkiHNVKdb2DRCQ8HCAcocYDgD\/gj4Q8EdAw9nv\/k+U59CFdFo3FbTNKAr6+6tTlGMKyR9AVEO26\/lq6fPSVNVby3gQOEA5SQGPwBfyT8gYBq6HEkuGzZsgl4rlu3LnJ\/ZlrgzYMUbBYRmRPVWIF7Fn04QDjAtP0Q\/ElGDv0rHiMIaDKHGp4irJGwgAgCakNMOEB8YNjwJCoN+AMBlfCnFLZhAqqO8KvzAiJEEMn0hAOEgCazJDoF+AMBteaPvg9UGfF+0EYfMG8KqH6EX8d1V9JjHddYv2PVEqKDQyAknAZ\/wB8JfzCEO4Yei+e2bdsmXJxtc06tBHxbW7ORsAL3HHJwgHCAtv0oLB34A\/5I+AMBNbaxmNFm0hYTCfi2tmYjYQUuBNSWOxAICIQtV\/CB4Y4UBNRDAf3KM6\/Tl771WtDadbxEW6c5BAIC4e728AFmixn6VzxSENAxfDjSXLVqFQ0ODo6fBlTWIVyswIUDhAO0RQAfGBKkIKAQ0ET+KKGMug9Uz4DPx33iiScS88wygfmVU\/dLtBGB2rMLDhACas+WySnBHwiohD+lsNUFVF+B+3sfu5T+8o9\/oxR1bFQl0MEhEBLugT\/gj4Q\/GMKVoFeQbZSA1vUSbUSg9sSDQEAg7NmCCNQVKwiohljYPtA8jvKTNJK+haXuC4gYRwgEBMK1P+EDzB4x9C8M4VqxxZd9oNjCMrE50cEhoFYdPCIR+AP+SPiDCNSzbSxf3PoKbX3xzaDNTz50s6TtK2ELBwgHKCEy+AP+SPgDAfVMQLGFBRGoS4eHQEAgXPhipgV\/MIRrxR9fhnBxiDwE1IrQY4ngACGgLnyBgLqhhQhUw6vsi4j0LSx1v4VFNRsEAgLh5vLwAeaCF\/oXIlAXvpQyrfrK0QUUW1jONhU6OARU0mnBH\/BHwh9EoBL0CrINE1C+woyvMqv7AwcIByjpA+AP+CPhDwRUgl5BtqqRsIVlMuBwgHCAkm4I\/oA\/Ev5AQCXoFWSrGknfwoJDFDCEa0M\/CAQEwoYnUWnAH8yBSvhTClsloNjCggjUlZBwgBBQV87o6cEfCKiEP6WwhYBGNwM6OARC0knBH\/BHwh8M4UrQK8hWNVLziqeDErGF5RzwcIBwgJJuCP6APxL+QEAl6BVky4303X\/4PvEhCvw8cNtsuv2mqwoqvdzFwAHCAUoYCv6APxL+QEAl6BVky43053+\/l5Z85aWgROwBRQRqSz0IBATClith6cCfePQgoBJ2FWRrCuiO26+l62dPKaj0cheDDg6BkDAU\/AF\/JPyBgErQK8iWG+nB7UPE21j4gYAiArWlHgQCAmHLFUSg7khBQN0xK9yCG+l3H\/yf49eYYQ8oBNSWhBBQCKgtVyCg7khBQN0xK9yCG+ljK\/+Gntt\/imY0X0AsoHjOIgCBgEBI+gL4A\/5I+AMBlaBXkC0ENBpoOEA4QEk3BH\/AHwl\/IKAS9Aqy5UY69emBoDTsAZ0IOhwgHKCkG4I\/4I+EPxBQCXoF2c782Cfond9ZH5TGN7DwTSx4MIRrwwEIBATChidRacCfePQgoBJ2FWSrCyiuMUME6kI7OEAIqAtfzLTgDwRUwp9S2M74xO\/ST67vCeoCAYWAupASDhAC6sIXCKgbWohA3fBqSOrpv\/WH9LN5fxyUjT2gEFAXEkJAIaAufIGAuqEFAXXDqyGpIaDRsEMgIBCSTgn+gD8S\/kBAJegVZHvlf3iA\/nXGp4LScIgCIlAX2kEgIBAufEEE6oYWBNQNr4akvvyPvkrvXfphHKIQgj4EAgIh6ZTgD\/gj4Q8EVIJeQbYQUAzhpqUaBAICkZY7bAf+xKMHAZWwqyDbS7\/wDfrFhZfiEAVEoM6MgwOEgDqTRjMAfyCgEv6UwrZ5xdNBPXAK0eTmQAeHQEg6KfgD\/kj4gwhUgl5BtkpAcZE2BNSVchAICIQrZ\/T04A8iUAl\/SmELAY1uBnRwCISkk4I\/4I+EP4hAJegVZKsEFKcQIQJ1pRwEAgLhyhlEoPaIQUDtsYpNefLkSerq6qLh4eEg3eLFi6mvr4+ampom2Z0+fZpWr15NO3fuHP+tu7ubent7Q8tQAopTiCCgrnSFgEJAXTkDAbVHDAJqj1VkSiWICxcupPb2dlL\/bmlpCRVFFtuVK1fSXXfdRa2trYk1UAKKQxQgoIlkMRJAQCGgrpyBgNojBgG1x8op5fbt22loaCg0Ch0ZGaG1a9fSxo0bqbm5OTFfJaAnH7o5MW3dEkAgIBASzoM\/4I+EPxBQCXoxtnECunv3blq\/fj0NDAxYC+iM5guCY\/zwTEQADhAOUNInwB\/wR8IfCKgEvQhbNR+6dOnSYEjXfFhc16xZM\/7ntra2WDHlCBQCGg42HCAcoKQLgz\/gj4Q\/EFAJeiG2av6Tf4paRMTR5+jo6Pjv5r\/NbFlAz3\/rVbr4uX7atWtXxjX2O7ujR4\/S9OnT\/X6JHGsPfOLBBT7Ax7X7LVq0aILJgQMHXLNoaPrzzpw5c6ahNYgo3EY8w0x5TrSnp4f6+\/tDFxWxgOIUIkSgaTiPCAsRVhreKBvwJx49RKASdmm2SStv44pJWlTEAopTiMIR9JHAGVHOKhvgUz0HaNXwGSUCf6rHn1JGoEnDsKoZXLe8sB0ENJrE6ODV6+AZ+X6rbMAf8MeKKBGJfORP6QTUPERBYa0WB\/FhCnxwQkdHBy1YsGB8n6g6SCHu0AUloBfu\/Tr9ypHvSdoatkAACAABIJAxApgDzRhQZAcEgAAQAAJAoIwIlC4CLSNIqBMQAAJAAAgAARMBCCg4AQSAABAAAkAgBQIQ0BSgwQQIAAEgAASAAAQUHAACQAAIAAEgkAIBCGgK0GACBIAAEAACQAACCg4AASAABIAAEEiBQG0ElA9n2LRpUwDRli1bgj2kdXz4pKbOzs7g\/OCkPbM6Znwf6+DgoNWdqz7j6oKPek\/zQA+f3z+p7i74mHu669DvXPDR09alf8XxS\/Ujtcc\/iYtl+L0WAqpfebZv3z6n68\/K0EhZ1UF39EuWLAkOpFAXl5tlmFfI8b+3bdtmfW1cVnUuMh8XfPR6qRuB1q1bF3pjUJHvkGdZLviYx3EmnVGdZ72LytsFH\/Vx0dvbG3zM16F\/2YgnH4jj04dWLQSUIyl+mKw+fuVk5QBMJ8YfFlu3bo286UYvtw4OMA0+7AhXrlxJp06doqgr97Jqv0bn44JP0pnUjX6XPMp3xUe\/9KIO\/SsKcxWJz58\/n44cORL4aV9GCCsvoFHn5UZFXnl0rLLkaV4+7nIZeR06eBp8+OPsuuuuo29961uR0XxZ2l9aDxd8zBEMadk+2LvgExaBDg0NWX3M+oCFSx3feOONIDkf09rV1QUBdQEv77RhESc7vZkzZ1Z6uC0MVzPidIkSbA\/4z7s988zfFR\/Gb\/PmzbRixQq67777aiGg+ohFHH9YQA8dOhQ0V13WHrjyR\/kmHrbs7u4OhKPOj\/lR4QMWtYlA9YlpCGhf8LVnK6DsDB955JHKLyJycYDs\/B588EH67Gc\/G1xCHjef7IMjsKmjCz5qXljNZ7HtqlWrKs0hF3zUsOWGDRuC4UqX0SCbtvIxDQS0hK2GIdxzjeIyxKSs6iKe\/L4u+HDaZ555ZsK8etWnBVzwMYdw67BSGfjIBAACKsMvN2s94qz7IqK1a9fSxo0bqbm5ORCMuEVEdVsZaEbkcfjoW3x04lZ5KM4FHxO7OvQ7F3zq+IGR5OAhoEkINeh3bGM5C7zLMvs6DLmZdHTBR7etQ3Tlyh\/TGdZhiNKFP2FDuFUf4k5y\/xDQJIQa+DsOUjgLftxGb7XwgxczREVYPu3RSkM3W3zqKKAu\/OG0+kEKdTkowIU\/\/FGxbNmygEp1wSeuT0JA03gs2AABIAAEgAAQ8BCByq\/C9bBNUGUgAASAABDwAAEIqAeNhCoCASAABIBA+RCAgJavTVAjIAAEgAAQ8AABCKgHjYQqAgEgAASAQPkQgICWr01QIyAABIAAEPAAAQioB42EKrojsH\/\/fpo6dWpwYITNw0vo3377bbr66qttkqdKo7YGtbW1OV0L58tB\/ur9itqSUXR5qRodRpVGAAJa6eat58u5btovYv+ZRAQltkUyQL82sKhyfcGmKDxQTrEIQECLxRulFYBAGQXUtU46TL6IBAS0AHKjiFIhAAEtVXOgMrYI6KfcsI0aFt23b9\/46S78d3VyknmykhpmnDZtWnAH4fDwcFC0OstWv2pKzz9uSFgvQx\/GVDeTqHdbt25d6FV6UemUgC5ZsoTuv\/\/+IBtzmFQ\/1cYsR136feONNwb2CqsTJ05QZ2cnjY6OBib33HMP7dixg\/r7+6m1tTX4W9Q7hbWTKaD873fffTf4H1\/ZpeMbZh92h2jSvaK+fFzY8hrp\/EIAAupXe6G2xpm+7e3t446e\/4OPITSjvaiDuzl9X19fcEawfpGvEuelS5eOC13cwfpKbFV+fFWceYtNUgRqptfPSmWRZ6GbP3\/++IXLen1YCHt6eiYIn56f+kiYMWPGuL35jurfx48fD64cU1e0sVCreyqTzkcOE1C+C1R9MIThqhMaAoru7RsCEFDfWgz1HT9jVRc4HZYkseK0urM2BTTsFpa4A+PDoiAzfVydkg6jNw8e5\/onRV7670pAzQ+CoaGhcUHlPHWB5H\/rN\/cofOOGacMElKNb\/kjhjwpVBqcbGBiYtMALAorO7RsCEFDfWgz1DRDQhzvNVa1RYmUOcy5evDg0AjWHUnXIw4Zfo8rTD+ePE9CkRUxhYhn2N3NY2xym5kiSL2\/mJ0wI9Tw5qlUHnZuUi7qyLUxA1aiAyiNO+CGg6Ny+IQAB9a3FUN8JCOiioc+D6lGOEkRzXlJFYGYEyrZm5BQHe5Q46oKSp4By3dRcphL4sAjURUD37t1L27Ztc9puAwFF56wbAhDQurV4Rd9Xj+JUhMXDhDx0uHr1alq4cOGEhTu6SJoC6nqReBFDuOYcp14mi13ccKwawtUFNCza04dwOQJ1vZ8yjyHcpI+ZpKHsitIdr1USBCCgJWkIVMMegbDFKHqEpy+qCVsMoyJSNYTLJesiq\/Ln4U61gCZsHlLVOKtFRHrEp8+Lzps3b9IiIVNAdVtVV64fLwgKE1DbRUSch5rDTFoEJF1EpIbY1cpp9R764imTJRBQ+36DlNkjAAHNHlPkWAAC+sXFXJw+PKtvQeEhzVtuuWXCVhUWzttuu43uvffeQGB4y4YpqioqVdtbuIyky8TjtnzYLmxas2bNOHphw7Fqe4kpHGbZGzZsCOY5eeGQen89AuVCTAzNbSzmVh62idqCo6J+\/n\/10RG2jUW3DxM\/ff6Z22nu3Ln08ssvByJufuiodzCj8wLohyKAQIAABBREAAJAIECABS1s5a0tPDZzoLZ52aZDBGqLFNLlgQAENA9UkScQKDkC5jyvijb1fZ+urwABdUUM6X1HAALqewui\/kAgJQLm6UxR21NsszcPd3\/88ccDUzWka5uPbTocJm+LFNLlhcD\/B2YOgZgmzWRVAAAAAElFTkSuQmCC","height":280,"width":464}}
%---
%[output:19da337d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAdAAAAEYCAYAAADs5qfZAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQuQXsV15sEhWQaIkUa8PBaKZDSycRyPkIJLlnmEaAmVbEk43s2ORluJazL2joMFWyupZiQwVDAgjaYktljAZZVrPFEeesQOrKXspgiWMQUetBAJJskao0FPxAhbSMhgW06WtbbO1fSop+c+uu+59\/637\/1ulcto\/j7dfb\/++nz39PO8M2fOnCE8QAAIAAEgAASAgBMC50FAnfBCYiAABIAAEAACAQIQUBABCAABIAAEgEAKBCCgKUCDCRAAAkAACAABCCg40HAETp48SV1dXUE9BgYGqLm5Obc6jYyMUGdnJ82fP5\/6+vqoqakpt7L0jIt8R5sXUjjccccd1N7ebmNS6jTr16+nTZs2BXXs7u6m3t5ep\/pu376d1qxZQ+vWrSslHvx+u3fvzr1\/OIGGxBjCBQcaj0CR4hIloOygbrrpJlqwYEEugES9Y97lRr1MGofMDvyZZ55xEqc0Nq4NwGUsW7Zs3KyKAlq1Dx7XNi5rekSgZW0Z1KsQBE6fPk2rV6+mnTt30pYtWwoTUI58iyg3DETljBcvXmwthipCcxGnNDZpGl0JqEvdzHLKHoEqnh45cgRRaBqS5GQDAc0J2EZlqw9lcR30ISk9+po7dy7df\/\/9QTXZkerDmSpaGh4envS7nsdtt91Gn\/vc54I0LS0tNDg4SK2traGvrguVmd6Mzvh3NaS7aNEieuihh6itrS1wHLrwJOXDQ8Gq3D179gT140cN4d5777305S9\/ORBP9ZhY8N8VprrAKqetp1dOWOWlO3T9HR999FHq7+8PLffo0aNB\/UZHR8frpLehjiNj\/thjj9HXv\/51Uu\/H+JtYK+zU0HiYWKh21ctV72u+l2rr6dOnj38EmPjt2LEjGBJVj84PM7+kDxeTj3F5xfEwLlLVMeE6q7qbomz2Lx1blceKFSto165dxP1HtZ3+zvw3VYbetkm4hPGwUX4G5Z5FAAJaESaYTlN\/LeUEwpyk6fg4HxYvJZ7m72EOPk58+LeouilnN23atAlzoEpA9TqwUIUJni6iZj5ZCWhYhGM6M9OxRuHKf48S0J6eHlq+fPkk7OMEi3+77LLL6Pjx48EHQpiocZm6ozfrHsULVe7evXtDxfDxxx8fn3fU+aYLhCmgZl7q9ygRjeMs2xw+fDhSqPU6meJpfuQo8brhhhvo2WefneAVwkQwrH+ZAshpwurIf1flJOWt41L2KLkirtTpNSCgTnCVN7FyEPoXuHI+XGs9+uIoQ3VM3UHpnV3\/8tYdLouUipBUHqpsM9JRaIX9rjuDW265JVJA9S9013ySBJSjbn6ShlLjImSOik+cODEJEz1qYpzmzJkz4R1thnDN4WWFvWpPjjbNdue68HxgWGTMWC5ZsiR4Xz1itVlYZTMca6Yx\/60wUWLP9U8qW3Ev7H3U3\/hDi985aghXx1HxyWzTp556KhDisIgyKl9zFEJF3XoecWWrCFXxPwmXLIaqy+vB\/KwZBNTPdptU66ivU+WA2HHMmzcvdAWqnubQoUOhUQUXqOfBUY9aMZu0CCjpyzlKoHSHwuW75pOVgOplsxjyozvsKMemC8jnP\/95awENi9jDyuV6mEPUUREep2UhUPXQsQ0rzxzKjhPQqKFr0yYumgz7+DLfTU0PmEKsPhqihC6Jn3r76nlEtasZzSqslIBGDd3rK8x1Lqt+qQ+fqw6u4xI2bVAR9+Xta0BAvW26iRUvWkD1bSBJDspV+PjNwra12Oaji4PpbDlvfRuLTQTKafSFN\/xv3jJhRuCmA3cVUBNHM0o1hTsrAVVMihJuXpkcJqDmUHBSBOqDgIaNeKh2NfkXFYHqeUT1DQhoNRwvBLQa7ThpKE8fHosbwjWHGtWcUtTXfNiQW5KAxg296lER15O\/0qME1DYfHhozxU0NbZsCyiJlszjDFBc9QjOHwVlwkoZwOTpOEiAzj7RDuDrFo6I6sxuYYmhGY+qd9ZEI9T6KO6ZN2BBuUvfLewhXfWypyD1KQMMid4WRGYFGLfoyh4\/jhnDDcMEQbhJbiv8dAlo85rmUmPciojgBShLQuLqFzQ9GCWhSPjzcpeYzTZBtBJRtwlbhqrzMlZT6AQQui4jUUJ5uw+V+5jOfCaLjsIdxCns\/20VEnKf6qDCFO2qBjW6jp+EyH374YXrggQcmLXhiG1NA+W9RC5LUuyZ9sIUNbyaNAOg4Rr1jnPjpgnXnnXdGcisuD65D2OIi20VEOi5JIzC5OBZkGosABLRiBLHdxqJvQUnaxhK2MMllCJchjhseTFqko59MFJcPl2Nuebjnnnvo5ZdfHl80ExaB6s41aiGUnrc5NxsmsLqQ6Lb830pAw8r92te+NuFEHT7cQZ9vTbONRRdC3aGHbXGK2j5j4qrPyXKejNuGDRto1apVARz6SIJaTR21LSZp\/2bcNhYuyzYyi5q75FGIMHGKiroZo7AtRGFRbNTHF\/\/dPPkoai5Z5WEzUlIxd1b614GAlr6Jsqtg0orH7EpCTnkgEDZUbK60jtqHq9cnzUEKebxPHfLUP3jUh0LYytwkLHCQQhJCjfm99ALKnZ33x\/HG8zDnEBaRJH3RNgbqxpcKAW18G0hqEDeEHTf0HFZmmqP8JHWvs23YEC7jkXT4SNhHT1XOLq4KH0otoDYLHnh4Z+XKlXTXXXdFnoJTlcaSvgcEVIpg4+3N4Uyukat4sg3OVi22Lc2pFRfx5Jrig6fY9rItrdQCynMQTBx+oiJQdgRr166ljRs35nqLhy2gSAcEgAAQAAL1QKC0Aspf2vfddx91dHQEIholoEpk874Gqx50wFsCASAABICALQKlFVCeN+CHT+mImwM15xfiVlHagoJ0QAAIAAEgAASSECilgPKw7ObNm+nuu+8mPrw8TkA5OuVl5eo2EfPfJgAf+tCHkjDB70AACAABIFAwAnyDzaxZswouVVZcKQVUv2Q4aRWu+fpJ6VlADxw4IEOtotbAJr5hgQ\/wkXR98Kd6\/CmdgIatMlSwJ90byOmSFhWBxNEkBjbV6+ASh+9qC\/6AP66c0dP7yJ\/SCahLRKm2uSxcuJD4WDX1b14i3tvbG9qWPjaShJQutsAGDtCFL2HTIxjdwQdqWg756H+8E1Alkrw6lw\/tjjvkO6whfWyktIR0tTt48KB3cxCu7yhJD3zi0QM+wEfSv3z0zaUXUEmDQEDd0IMDhAN0Y8zE1OAP+CPhDwRUgl5Btj42UkHQEBwgHKCEa+AP+CPhj4++GRGopMUrZgsHCAcooTT4A\/5I+AMBlaBXkK2PjVQQNIhAE4CGQEAgJH0R\/IlHz0ffjAhU0iMqZosODoGQUBr8AX8k\/IGAStAryNbHRioIGkSgiEBFVIOAQkAlBPLRNyMClbR4xWzhAOEAJZQGf8AfCX8goBL0CrL1sZEKggYRKCJQEdUgoBBQCYF89M2IQCUtXjFbOEA4QAmlwR\/wR8IfCKgEvYJsfWykgqBBBIoIVEQ1CCgEVEIgH30zIlBJi1fMFg4QDlBCafAH\/JHwBwIqQa8gWx8bqSBoEIEiAhVRDQIKAZUQyEffjAhU0uIVs4UDhAOUUBr8AX8k\/IGAStAryNbHRioIGkSgiEBFVIOAQkAlBPLRNyMClbR4xWzhAOEAJZQGf8AfCX8goBL0CrL1sZEKggYRKCJQEdUgoBBQCYF89M2IQCUtXjFbOEA4QAmlwR\/wR8IfCKgEvYJsfWykgqBBBIoIVEQ1CCgEVEIgH30zIlBJi1fMFg4QDlBCafAH\/JHwBwIqQa8gWx8bqSBoEIEiAhVRDQIKAZUQyEffjAhU0uIVs4UDhAOUUBr8AX8k\/IGAStAryNbHRioIGkSgiEBFVIOAQkAlBPLRNyMClbR4xWzhAOEAJZQGf8AfCX8goBL0CrL1sZEKggYRKCJQEdUgoBBQCYF89M2IQCUtXjFbOEA4QAmlwR\/wR8IfCKgEvYJsfWykgqBBBIoIVEQ1CCgEVEIgH30zIlBJi1fMFg4QDlBCafAH\/JHwBwIqQa8gWx8bqSBoEIEiAhVRDQIKAZUQyEffjAhU0uIVs4UDhAOUUBr8AX\/S8mfri2\/SF7e+QicfujltFg2xg4A2BPZyFgoHCAcoYSb4A\/6k5Q8ENC1yBdv5OExQFERwgHCAEq6BP+BPWv5AQNMiV7AdBDQacDhAOEBJdwR\/wJ+0\/IGApkWuYDsIKAQ0LeUgEBCItNxhO\/AnGr1aC+jJkyepq6uLhoeHnfjV1tZGTzzxhJONNDEEFAKalkNwgBDQtNyBgMYjt\/7JQ7T+yYP1XESkBLS3t5cWLFhgxbHdu3fT+vXrIaBWaBWTCAIBgZAwDfwBf9LyBwLa1UUQ0LT0KYcdHCAcoISJ4A\/4k5Y\/tRbQtKA1wg5DuBjCTcs7CAQEIi13MISLIdxIBPQ50O7u7iASLesDAYWApuUmBBQCmpY7EFAIaCJ3eE5z06ZNQbqWlhYaHByk1tbWRLsiE0BAIaBp+QYBhYCm5Q4EFAJqzR1zVW6ZolIIKATUmshGQggoBDQtdyCg8cjxMX68lQVH+Rk4jYyMUGdnJ42OjpYiKoWAQkDTOkEIKAQ0LXcgoBBQCXcCW7V1ZWBggJqbm8X5pckAAgoBTcMbOMBk1PCBgQ+MZJaEp0AEGoGcHoFyki1btljvFU3bGHF2EFAIaFpeQSAgEGm5gw8wRKDW3Dl9+jStXr2adu7cGdgsXryY+vr6qKmpyTqPvBJCQCGgabkFAYWApuUOBBQCmsgdfRVuI6JNjnZ7enqov78\/cvUvBBQCmkjkiAQQUAhoWu5AQOORW\/LYS\/Tc\/lP1XESkr7ptVLSpot49e\/bEbp+BgEJA0zpBCCgENC13IKAQ0EgEXnvtNbrzzjvp3nvvtZ7fzPosXJUfVxIRaLpuDoGAQKRjzlkr8Af8ScufuQ88T0dO\/rzeEWijzsLlCPi+++6jjo6O4IB6CGg6GsMBwgGmYw4E1AY39K9olJpXPB38WMt9oI2+zmz79u0B+PPmzcMcqE1PxhxfKpTgAPGBkYo4Y0bgTzh6HHlyBFpbAZWQSmrLC4c2b95Md999Nx09etRKQFWZu3btkhZfKXvGb\/r06ZV6pyxfBvjEowl8gI9rf1u0aBG9d+mH6SfX90BAXcHLIj0P2d50003B3CtW4coQxRcyIiwJg8Af8CcNf\/gIPz5IARFoGvQENnFDx1EHNmAVbjTgcIBwgILuiEVECeChf4UDpLawvO9nb9FbX\/0DCQULtz3vzJkzZwovNacCEYHKgEUHh4BKGAT+gD9p+KMWEJ3\/1qv0oz\/\/QposGmYDAW0Y9OUrGA4QDlDCSvAH\/HHlj76A6OLn+unIC3\/nmkVD01dKQG2QxBAuhnBteBKWBgIBgUjLHbYDfyajp4Zv+RcIqIYPby1Zs2ZN8Beejzx8+DANDQ01\/ExcCCgENK0ThAOEgKblDgR0MnLPvXaKlnzlpeCH66+eQv+88d\/TgQMHJBAXbptLBMorY\/n+Tz6Xdvny5cQHLLS1tQUHzLe0tAT\/btQDAYWApuUeBBQCmpY7ENCJyPHQ7fKtrwTn3\/LDByj46JszF1C1MpZFcs6cOdTV1RUIJm8zwX2gku6Xvy0EAgIhYRn4A\/7Y8mfz86P0X7\/xapC847or6bGOayCgwZfEyZPjogkBtaVTOdLBAcIBSpgI\/oA\/NvzRh25nNF9AO26\/lvj\/EYGOocfznzzfqQ\/hKjFdunQptbe32+CcSxofGykXIEIyhQOEA5RwDfwBf5L4Y4rny1\/65LiJj7458yFchQYP1y5btmwCnuvWrWuoeHJlfGykJFJm9TscIByghEvgD\/gThQDPef71njdp7d8dDJJwxPno0mvo+tlTIKCSTle0LQQ0GnE4QDhASX8Ef8CfMARYPHm1Lf9\/lHj6GtzkFoFKOmKethBQCGhafkEgIBBpucN2deMPC+b6Jw8Sn3WrHn3O08TSR9+cuYDaXm3W3d3dkO0sPjaSpNO62Natg7tgU0cHCHxcEcAHhkKAV9jySlv9GfjDX6ffv\/bySJB89M2ZCyijw4uItm3bRgMDA9Tc3BwApoSVFxEtWbKkYXtCfWykbLsxItC0eOIDAwKRljtV\/wBTw7P63k6bqFPH00ffnLmA6ttYeO+n\/uj7QPft20d84MITTzwh4aSzrY+N5PySKQ0gEBCIlNQJzMCfevLnr144Rnds+8Gkl48brg1DykffDAGVeIyK2cIB1tMBZkVj8Kc+\/Hnl2E+pY+AfxxcG6W\/OByP03jorWGnr8kBAx9BKGsLlfaBqr+jDDz\/sgrE4rY+NJH5pywzgAOvjAC0p4ZQM\/Kkuf3iI9pGnj9DA994IfUkWy0faP0I3tE514oye2EffnHkEqgAJ2weqLrlm8XzkkUdocHCQWltbUwOextDHRkrznmls4ACr6wDT8MHVBvypDn9YMF9586f02NNHxs+rNd+ORbPjug9Q760zXakSmt5H35ybgGaCaA6Z+NhIOcAQmiUcYHUcYFGc0csBf\/zlD58Q9N4vfkEPPXU4UjD57Vg0v\/qfPkotl\/wb5yHaJE766JshoEmtWqPf4QD9dYBloCn44wd\/1IpZ3qP5vf2nQucx1ZuwYH7q6ilBpMn\/7Tqv6cJLCOgYWiMjI9TZ2RlcaWY+fK2Zvr3FBeAs0vrYSFm8t00ecIB+OECbtmxEGvCnfPxhsVTH6B1663RsdKkizBlTL6CesUVAeQqmiZaPvjnzCPT06dPBHs+FCxeO7\/fs6OiYdLVZIzo4l+ljIxWFFRxg+RxgUW2fRTngT2P5w8OwZ4ho24vHEiNLPcL8nY9OoyUfvzz3CDOJYz765swF1NwHyns9Z86cGRwizwuLtm7dSn19fdTU1JSEZy6\/+9hIuQARkikcYGMdYFHtnFc54E8x\/FFR5fZ\/eJMOn0iOKs3hWN5ioqLNvLiQJl8ffXPuAsorbg8dOhQc24cLtdPQqjgbOMBiHGBxLVpsSeBPtvxRc5X\/7duHaf\/xnyUOv+pCyf99Y+tUWnXL2RWyRQ7FpmUdBHQMOY46+TFF86mnngruCUUEmpZi+drBAWbrAPNtrfLlDv6k4w8Pve59\/R369vdP0JG3z85Z2j7Bwp6pF1D3jVfRb3zwYm\/EMuz9IKBjqOjzoDx0y4K6adMmamlpacjeT72xfGwk284kTQcHmM4BSnGvij34E96SLJAsdH\/z\/Ag9feg9Z5FUESQLZeenPkiXXfwrE+7RrAp\/fPTNmQ\/hlr0xfWykojCFA4SASrhWZ\/6oeclv7v0hHTj+s9Qiyfjf1DqVVo4Nvfoy\/CrhjbL10TdnLqC2h8mrW1qyAN4lDx8byeX9JGnr7ABtcAM+9f3AUAL5z6M\/of\/1T8cDIJ7bf8qGNhPSqLlI3lv5B\/OvpPPfd14lo0lnYDzdIQEBTdPSFbWBQNRXILKgtK\/8UeLI4rbnyDs0OHbeaxqB1CPG3\/5IM3267XKaOe3sjoP\/9+NjNGvW2RWweCYj4GNwk5mA8mrbNWvWJPKiURdp+zxMkAhqRgl8dYAZvX5iNsDHzw8MtSjnF2fO0MO7jgQrWl0X6+hvrqJInpNsv+5KumH2uQPU41a7gj\/x\/Km1gCpo4oZwEz1UAQl8bKQCYAmKQAf3UyCK4kdSOY3gj75i9S92j9L\/PvjjoJppo0c9gmSBXHTNNJo\/4\/1BntfPnpIEQezvjcBHVOGCjX30zZlFoAVjnbo4Hxsp9cs6GqKDQ0AdKTMhedb80cWRL21+fmzOUSKOukDyPOS\/vWZasKpVnfPKZea1ZzJrfCRtVUZbH30zBLSMTGpQndDBIaAS6rnwR23t4GHVR7\/7Ou1786fiyNGMHluvuIh+f+65I+ryFEcb3FzwscmvamlqK6Bq2HZ4eDixTXGYfCJEDUuADg4BlZCP+fNLl3xgPIsXD\/+YvvODk\/Q6H2jueEBAWD30uceZlzbRbW2XU+vlF44nzStylGCi26J\/xSNZWwHNimBF5ONjIxWBC5eBDg4BjUNARY1vvvMv9I09P6RX3\/xpJsJoRo6zr7hwfPUqi2KjI8es+h\/6FwQ0Ky41LB8IaDT06OD1FFC1jYP3JP7lC8foyInTmQmjEkdekHNV8wX0kSsvomuven8hc44NczIRBaN\/QUCtORm2rWXdunXBrSyNfCCgENC0\/PPNAapFOE\/+n7fo5aPvZjaUqvDTh1R\/\/YMX07xp79EHPnDu4uWqRI5p+WLa+cafrN7bNh8ffXMui4hYPLdt2zbh4mw1T7p06dKGiqiPjWRLQGk6dPByR6D6hn9efPP4Sz+i7756Mqh0FnOMYcL44Ssvoi\/ceBUd+\/G\/BD\/HbeUAf8rNH6l\/yNveR9+cuYDiKL+8aZZf\/nCAjXOAPL\/Iz+tv\/5x2HzxFB49nO4yqhlKD\/596AX18+q\/S567\/IL3vvPPG5xili3DAn8bxJz+vUFzOEFAigoAWR7isS4IDzNYBqiHU7+0\/Rc+99nYwhMqPdB+jXkt9GJXnGD81e2ogkEXsa8QQpVsPRP+KxwsCOoYPhnDdOlZZUqODx7fE94ZH6MxFlwXidOjEafrbfzpOPziWzf5Fs2RdGFuvuJC6b5hOP3r3\/yYOozaSS+BPth9gjWzLRpQNAdVQxyKiRlBQVmbdHKBa5KIixadeOUF7j7yT+WIb1SqmKF438xJa+KEpE7ZpSIdRZQyQWdeNP65oAR9EoK6cKV16H79yigKxKh1czSeyGL146Mf0nVez28wfFynyECqfmzrniovGI0Ulzj4Loy3\/qsIf2\/d1TQd8IKCJnCnLatuoikJAo5uwzB1cCdE\/Hn2XOFI8+Nbp4EWyXH0aFinOuqyJ\/t3HLgv2L77w\/YP0iY\/Oyu2s1MTOVfIEZeZPGaADPhBQKx6aw7dbtmyhBQsWWNnmnQgCWg4B1bdkPPn9t2j46E8y38Cvv6l5DNzij19GTb\/8S06LbeAA43sn8AE+Ev\/to2\/OfBuLCeD69etp06ZNwZ9bWlpocHCQWltbJTiLbH1sJNELOxindYD6rRlc3PF3\/5X+7PlROnyimCiRziP6zV+7hD77yRb+z8y2ZZjQpcXHoQm8Tgp8IKASAvvom3MXUB1QFtPdu3dPOGBBAngaWx8bKc17prExHaCKEjmvfT\/8aXCazaG3st+fGDZ0GmzJuHoKzWhuCn7mDfyNnk+EQEAg0vQrZQP+xKPno2\/OXUD1CLTRN7Fw8\/nYSJJOa9rq0eLQ\/lP07Nj+xLznElkQp0+9gJZddyWdl+Hm\/SyxScoLDhACmsSRuN\/BHwioFX+kw7anT5+m1atX086dO4Pyuru7qbe3N7RsM21S+ioLqC6O\/+PlH9G3XzkRYJbXxv1fm9ZE\/3H+FYEgqiiR\/7+qZ6DCAUJArRxgRCLwBwKayJ+4k4gSjccSsADzw6KZtKqXf1+5ciXdddddVnOrVRBQNbS69cVjwZ5FqUCqBTaXNRHNn3Up3ThnKn2s5eLK7E+05V1SOjhACGgSRxCBpkfIR9+c+xBuejjPWeqCauY3MjJCa9eupY0bN1Jzc3Nicb41Eosl72Xc\/Pyos1DqK095OwZfQDz7snMXEDNY+v5ECAQEIrEDxSQAf8AfCX988838rqUX0KSIlhclscAODAx4L6BqCLb\/7w8FWzpsIsvgzNOpF9AnZl1Cf7SgZZy\/aTbuwwHCAUocIPgD\/kj4AwGVoBdiq+ZSFy9eTH19fdTUdHZFpv6Ye06TFipxI6ln165dGdfYPbvRd94LjP7022\/RnjfOHjYe9rS8\/3z6wK+eT7e0XkQfav5lmv\/BC9wLS7A4evQoTZ8+PfN8q5Ih8IlvSeADfFz7+qJFiyaYHDhwwDWLhqYvfQTK6LCQjo6Ohoqo+VtcWs6rTF85vY+P0NeeOxpKAI4g23\/zSrph9tTYOxizZA8iiHg0gQ\/wkfQ38CcevTL5Ztt2zlxA87jOjOc5e3p6qL+\/P3GhUFLaRjcSD9Mu3\/rKpOFZNRT7aMc1weKduIuLbRvXNR06OATClTN6evAH\/JHwp9G+OU3dvRBQl3nOpEVFjWokFsX1Tx6krS++OaGdWDgfXXpNQwTTJAwcIBxgGieibMAf8EfCn0b5ZkmdMxPQsOvLwioWt6dTpddX3ap9nnwMoLkXVP22cOFCam9vp7i0Ku+iG0ltOVnylZcmCeeO268t1cHkcIBwgBJnAv6APxL+FO2bJXVVtpkJqMowadWsTaXNwxH0RUTqt46OjuCA+ri0YWUV2Ugsniyc+gEHZYo4EYHasPFcGggEBMKNMRNTgz\/x6BXpmyXtqNtmLqBZVSyvfIpqJL6TUo86yyycGIKzYxscIATUjinhqcAfCKiEP6WwLUJAw8SzbMO1YY2BDg6BkHRS8Af8kfCnCN8sqV+YbSYRqD5sO2fOHOrq6qLh4eHQuibt08z6Bc388m4kXiT0xa2vjBf7ze42+u0PJ5+QlPd72+QPBwgHaMOTqDTgD\/gj4U\/evllStyjbTAQ0j4rllWeejaRHnj4M2ZoYwwHCAUr6HfgD\/kj4k6dvltQrzhYCmhGypnj23jqLOq67MqPci8kGDhAOUMI08Af8kfAHAko0fntKnYZweZXt3AeeH+cOz3c24iAECXnZFg4QDlDCIfAH\/JHwBwIag57rtWOShoizzaORljz20vjJQr6KJwQ0mXEQCAhEMkuiU4A\/8ejl4Zsl7WVjW+gQLp8otHXr1siD4W0qLE2TdSP99+8coT\/92\/1BtXjI9rGOa6RVbJg9OjgEQkI+8Af8kfAna98sqYutbeEC6nL1mO1LuKTLspHMec+Xv\/RJl6qULi0cIByghJTgD\/gj4U+WvllSDxfbQgU06aYUl4qnTZtVI5mnDLF4prmDM+175GEHBwgHKOEV+AP+SPiTlW+W1MHVNnMBVXtCwxYR8Xm2g4ODiTequL6ES\/qsGmn9k4eCw+H5+ZMbr6IHPz3bpRqlTAuXzZypAAAW4ElEQVQHCAcoISb4A\/5I+JOVb5bUwdU2cwHlCkQd8q4OfXetZJbps2gkfdUtR52+D90qfOEA4QAlfQ38AX8k\/MnCN0vKT2Obi4CGDdWqyHTp0qXBzSmNerJopJXf3EeDQ28Er8CLhnzb7xmFPRwgHKCkX4I\/4I+EP1n4Zkn5aWwzF9CkC7V9X4Vb1eiTyQMHCAeYxolgBMMONfSveJwgoNpBCnx3J183pj8uF2PbUdI9lbSRqrLnMww5dHAIqHuPOmcB\/oA\/Ev5IfbOk7LS2mUeg5vynXjG+dHtoaMjrfaDNK54OXqlKc5+IIOy6DwQCAmHHlPBU4A8iUCv+cKS5atWqCStuR0ZGqLOzkzZs2DApMrXKNKNEkq+ce3e8Ro9+9\/WgJj6fOBQFJTo4BELSzcAf8EfCH4lvlpQrsc08AlWVYRFdtmzZhLpt2bKloeLJlUnbSFWe+0QEateFIBAQCDumIAJNg1Na35ymrKxschPQrCqYdT5pG0m\/57OK0SfjDIGAQEj6G\/gD\/kj4k9Y3S8qU2mYuoGoOtKOjo+HRZhg4aRtJLR6q4twnIlC7bgSBgEDYMQURaBqc0vrmNGVlZZO5gMZtY8mq0pJ80jSSfubtHTfPoPsWXy2pQmltIRAQCAk5wR\/wR8KfNL5ZUl4WtpkLKFeqDKtto8BJ00hrnhihTc8eDbKs6vAthnCTuxMEAgKRzJLoFOBPPHppfLOkPbKwzVxA487C5Qq3tbXRwMAANTc3Z1F\/5zxcG6kOi4cUiOjgEAjnDqUZgD\/gj4Q\/rr5ZUlZWtpkLaFYVyysf10bSh297b51FvbfOzKtqDc8XDhAOUEJC8Af8kfDH1TdLysrKFgKagGSVTx4yXx0OEA5Q4ljAH\/BHwp\/aCqi+cGjOnDnU1dVFYdeZ+TaEqw\/fXn\/1FNrxxWsl\/Ci9LRwgHKCEpOAP+CPhT20FVAJa0bYujaQLaNWHb7kd4ADhACX9EfwBfyT8cfHNknKytM1lCLcq94Hqw7d85yfvAa3yAwcIByjhN\/gD\/kj4AwEdQ68q94FW+eD4MKLDAcIBShwg+AP+SPgDAbW4zsyX+0D11bfbP\/9xuuWaaRJueGELBwgHKCEq+AP+SPgDAbUQUI5OfdgHWoezb02ywwHCAUocIPgD\/kj4AwEloqrcB1qHs28hoG7dHQIBgXBjzMTU4E88ehDQMXyqcB+omv+sw\/YVRWt0cAgEBEKCAPgjQQ8CqqHn832g+vxnlc++RQTq1t3xgQGBcGMMIlAXvCCgLmg1KK1NI\/3F7mP0X\/76B0ENIaANaqgSFgsBhYBKaAn+YAhXwp9S2NoIaB3nP7lx0MEhEJJOCv6APxL+2PhmSf552OZykEIeFc0qz6RG0k8f4q0rvIWlLg8cIByghOvgD\/gj4U+Sb5bknZctBNRAtq7zn4hAk7sYBAICkcyS6BTgD4ZwJfwphW3SV85fvXCM7thWv\/lPCGgyPeEAIaDJLIGApsUoyTenzTdPO0SgBrp1nf+EgCZ3MwgoBDSZJRDQtBhBQMeQGxkZoc7OThodHZ2EZVtbW2lPIqrb9WVm40AgIBBpnR8+wJKRQ\/\/CEG4iS9RJRC0tLdTb25uYvugEcV85dbu+DALqxj44QHxguDFmYmrwBwKayB\/9cu0FCxYkpi86QZyArn\/yEK1\/8mBQpTpcXwYBdWMfHCAE1I0xEFAXvDCEq52F29HRQb4JqH7\/58mHbnZp+0qkhUBAICREBn\/AHwl\/IKBj6PExfkXduqKGjHfu3BmU3t3dHTt0HNdIcx94nngYly\/O5gi0bg8cIByghPPgD\/gj4Q8EVLvObHh4OBTLrBcRsVDzw\/Otavh46dKl1N7eHlp+VCPp8583tE6lb\/3JXAkXvLSFA4QDlBAX\/AF\/JPyBgErQy8hWF9SwLKMaST9AoffWWdR768yMauRPNnCAcIAStoI\/4I+EPxBQCXoZ2NosYIpqpLovIGL44QDhACXdEPwBfyT8gYBq6G3fvp3WrFkT\/GXLli10+PBhGhoaor6+PmpqapLgHGrLkeemTZto8eLFsWVwI6ln165d4\/\/9nx9\/k\/a88fPg33vuqF\/0ye999OhRmj59euZtU5UMgU98SwIf4OPa1xctWjTB5MCBA65ZNDR9LicRsZjxIQo9PT20fPnyYH6S5z5Xr15Nee8PVWVHCXXUV06dTyBSDEQEgQhC4o3AH\/BHwh9EoNoiIhbNOXPmUFdXVyCgvKWliNW5fAoSC3d\/fz+1trZOas+oRmpe8XSQ9vqrp9COL14r4YG3tnCAcIAS8oI\/4I+EPxDQEghokkiHNVKdb2DRCQ8HCAcocYDgD\/gj4Q8EdAw9nv\/k+U59CFdFo3FbTNKAr6+6tTlGMKyR9AVEO26\/lq6fPSVNVby3gQOEA5SQGPwBfyT8gYBq6HEkuGzZsgl4rlu3LnJ\/ZlrgzYMUbBYRmRPVWIF7Fn04QDjAtP0Q\/ElGDv0rHiMIaDKHGp4irJGwgAgCakNMOEB8YNjwJCoN+AMBlfCnFLZhAqqO8KvzAiJEEMn0hAOEgCazJDoF+AMBteaPvg9UGfF+0EYfMG8KqH6EX8d1V9JjHddYv2PVEqKDQyAknAZ\/wB8JfzCEO4Yei+e2bdsmXJxtc06tBHxbW7ORsAL3HHJwgHCAtv0oLB34A\/5I+AMBNbaxmNFm0hYTCfi2tmYjYQUuBNSWOxAICIQtV\/CB4Y4UBNRDAf3KM6\/Tl771WtDadbxEW6c5BAIC4e728AFmixn6VzxSENAxfDjSXLVqFQ0ODo6fBlTWIVyswIUDhAO0RQAfGBKkIKAQ0ET+KKGMug9Uz4DPx33iiScS88wygfmVU\/dLtBGB2rMLDhACas+WySnBHwiohD+lsNUFVF+B+3sfu5T+8o9\/oxR1bFQl0MEhEBLugT\/gj4Q\/GMKVoFeQbZSA1vUSbUSg9sSDQEAg7NmCCNQVKwiohljYPtA8jvKTNJK+haXuC4gYRwgEBMK1P+EDzB4x9C8M4VqxxZd9oNjCMrE50cEhoFYdPCIR+AP+SPiDCNSzbSxf3PoKbX3xzaDNTz50s6TtK2ELBwgHKCEy+AP+SPgDAfVMQLGFBRGoS4eHQEAgXPhipgV\/MIRrxR9fhnBxiDwE1IrQY4ngACGgLnyBgLqhhQhUw6vsi4j0LSx1v4VFNRsEAgLh5vLwAeaCF\/oXIlAXvpQyrfrK0QUUW1jONhU6OARU0mnBH\/BHwh9EoBL0CrINE1C+woyvMqv7AwcIByjpA+AP+CPhDwRUgl5BtqqRsIVlMuBwgHCAkm4I\/oA\/Ev5AQCXoFWSrGknfwoJDFDCEa0M\/CAQEwoYnUWnAH8yBSvhTClsloNjCggjUlZBwgBBQV87o6cEfCKiEP6WwhYBGNwM6OARC0knBH\/BHwh8M4UrQK8hWNVLziqeDErGF5RzwcIBwgJJuCP6APxL+QEAl6BVky4303X\/4PvEhCvw8cNtsuv2mqwoqvdzFwAHCAUoYCv6APxL+QEAl6BVky43053+\/l5Z85aWgROwBRQRqSz0IBATClith6cCfePQgoBJ2FWRrCuiO26+l62dPKaj0cheDDg6BkDAU\/AF\/JPyBgErQK8iWG+nB7UPE21j4gYAiArWlHgQCAmHLFUSg7khBQN0xK9yCG+l3H\/yf49eYYQ8oBNSWhBBQCKgtVyCg7khBQN0xK9yCG+ljK\/+Gntt\/imY0X0AsoHjOIgCBgEBI+gL4A\/5I+AMBlaBXkC0ENBpoOEA4QEk3BH\/AHwl\/IKAS9Aqy5UY69emBoDTsAZ0IOhwgHKCkG4I\/4I+EPxBQCXoF2c782Cfond9ZH5TGN7DwTSx4MIRrwwEIBATChidRacCfePQgoBJ2FWSrCyiuMUME6kI7OEAIqAtfzLTgDwRUwp9S2M74xO\/ST67vCeoCAYWAupASDhAC6sIXCKgbWohA3fBqSOrpv\/WH9LN5fxyUjT2gEFAXEkJAIaAufIGAuqEFAXXDqyGpIaDRsEMgIBCSTgn+gD8S\/kBAJegVZHvlf3iA\/nXGp4LScIgCIlAX2kEgIBAufEEE6oYWBNQNr4akvvyPvkrvXfphHKIQgj4EAgIh6ZTgD\/gj4Q8EVIJeQbYQUAzhpqUaBAICkZY7bAf+xKMHAZWwqyDbS7\/wDfrFhZfiEAVEoM6MgwOEgDqTRjMAfyCgEv6UwrZ5xdNBPXAK0eTmQAeHQEg6KfgD\/kj4gwhUgl5BtkpAcZE2BNSVchAICIQrZ\/T04A8iUAl\/SmELAY1uBnRwCISkk4I\/4I+EP4hAJegVZKsEFKcQIQJ1pRwEAgLhyhlEoPaIQUDtsYpNefLkSerq6qLh4eEg3eLFi6mvr4+ampom2Z0+fZpWr15NO3fuHP+tu7ubent7Q8tQAopTiCCgrnSFgEJAXTkDAbVHDAJqj1VkSiWICxcupPb2dlL\/bmlpCRVFFtuVK1fSXXfdRa2trYk1UAKKQxQgoIlkMRJAQCGgrpyBgNojBgG1x8op5fbt22loaCg0Ch0ZGaG1a9fSxo0bqbm5OTFfJaAnH7o5MW3dEkAgIBASzoM\/4I+EPxBQCXoxtnECunv3blq\/fj0NDAxYC+iM5guCY\/zwTEQADhAOUNInwB\/wR8IfCKgEvQhbNR+6dOnSYEjXfFhc16xZM\/7ntra2WDHlCBQCGg42HCAcoKQLgz\/gj4Q\/EFAJeiG2av6Tf4paRMTR5+jo6Pjv5r\/NbFlAz3\/rVbr4uX7atWtXxjX2O7ujR4\/S9OnT\/X6JHGsPfOLBBT7Ax7X7LVq0aILJgQMHXLNoaPrzzpw5c6ahNYgo3EY8w0x5TrSnp4f6+\/tDFxWxgOIUIkSgaTiPCAsRVhreKBvwJx49RKASdmm2SStv44pJWlTEAopTiMIR9JHAGVHOKhvgUz0HaNXwGSUCf6rHn1JGoEnDsKoZXLe8sB0ENJrE6ODV6+AZ+X6rbMAf8MeKKBGJfORP6QTUPERBYa0WB\/FhCnxwQkdHBy1YsGB8n6g6SCHu0AUloBfu\/Tr9ypHvSdoatkAACAABIJAxApgDzRhQZAcEgAAQAAJAoIwIlC4CLSNIqBMQAAJAAAgAARMBCCg4AQSAABAAAkAgBQIQ0BSgwQQIAAEgAASAAAQUHAACQAAIAAEgkAIBCGgK0GACBIAAEAACQAACCg4AASAABIAAEEiBQG0ElA9n2LRpUwDRli1bgj2kdXz4pKbOzs7g\/OCkPbM6Znwf6+DgoNWdqz7j6oKPek\/zQA+f3z+p7i74mHu669DvXPDR09alf8XxS\/Ujtcc\/iYtl+L0WAqpfebZv3z6n68\/K0EhZ1UF39EuWLAkOpFAXl5tlmFfI8b+3bdtmfW1cVnUuMh8XfPR6qRuB1q1bF3pjUJHvkGdZLviYx3EmnVGdZ72LytsFH\/Vx0dvbG3zM16F\/2YgnH4jj04dWLQSUIyl+mKw+fuVk5QBMJ8YfFlu3bo286UYvtw4OMA0+7AhXrlxJp06doqgr97Jqv0bn44JP0pnUjX6XPMp3xUe\/9KIO\/SsKcxWJz58\/n44cORL4aV9GCCsvoFHn5UZFXnl0rLLkaV4+7nIZeR06eBp8+OPsuuuuo29961uR0XxZ2l9aDxd8zBEMadk+2LvgExaBDg0NWX3M+oCFSx3feOONIDkf09rV1QUBdQEv77RhESc7vZkzZ1Z6uC0MVzPidIkSbA\/4z7s988zfFR\/Gb\/PmzbRixQq67777aiGg+ohFHH9YQA8dOhQ0V13WHrjyR\/kmHrbs7u4OhKPOj\/lR4QMWtYlA9YlpCGhf8LVnK6DsDB955JHKLyJycYDs\/B588EH67Gc\/G1xCHjef7IMjsKmjCz5qXljNZ7HtqlWrKs0hF3zUsOWGDRuC4UqX0SCbtvIxDQS0hK2GIdxzjeIyxKSs6iKe\/L4u+HDaZ555ZsK8etWnBVzwMYdw67BSGfjIBAACKsMvN2s94qz7IqK1a9fSxo0bqbm5ORCMuEVEdVsZaEbkcfjoW3x04lZ5KM4FHxO7OvQ7F3zq+IGR5OAhoEkINeh3bGM5C7zLMvs6DLmZdHTBR7etQ3Tlyh\/TGdZhiNKFP2FDuFUf4k5y\/xDQJIQa+DsOUjgLftxGb7XwgxczREVYPu3RSkM3W3zqKKAu\/OG0+kEKdTkowIU\/\/FGxbNmygEp1wSeuT0JA03gs2AABIAAEgAAQ8BCByq\/C9bBNUGUgAASAABDwAAEIqAeNhCoCASAABIBA+RCAgJavTVAjIAAEgAAQ8AABCKgHjYQqAgEgAASAQPkQgICWr01QIyAABIAAEPAAAQioB42EKrojsH\/\/fpo6dWpwYITNw0vo3377bbr66qttkqdKo7YGtbW1OV0L58tB\/ur9itqSUXR5qRodRpVGAAJa6eat58u5btovYv+ZRAQltkUyQL82sKhyfcGmKDxQTrEIQECLxRulFYBAGQXUtU46TL6IBAS0AHKjiFIhAAEtVXOgMrYI6KfcsI0aFt23b9\/46S78d3VyknmykhpmnDZtWnAH4fDwcFC0OstWv2pKzz9uSFgvQx\/GVDeTqHdbt25d6FV6UemUgC5ZsoTuv\/\/+IBtzmFQ\/1cYsR136feONNwb2CqsTJ05QZ2cnjY6OBib33HMP7dixg\/r7+6m1tTX4W9Q7hbWTKaD873fffTf4H1\/ZpeMbZh92h2jSvaK+fFzY8hrp\/EIAAupXe6G2xpm+7e3t446e\/4OPITSjvaiDuzl9X19fcEawfpGvEuelS5eOC13cwfpKbFV+fFWceYtNUgRqptfPSmWRZ6GbP3\/++IXLen1YCHt6eiYIn56f+kiYMWPGuL35jurfx48fD64cU1e0sVCreyqTzkcOE1C+C1R9MIThqhMaAoru7RsCEFDfWgz1HT9jVRc4HZYkseK0urM2BTTsFpa4A+PDoiAzfVydkg6jNw8e5\/onRV7670pAzQ+CoaGhcUHlPHWB5H\/rN\/cofOOGacMElKNb\/kjhjwpVBqcbGBiYtMALAorO7RsCEFDfWgz1DRDQhzvNVa1RYmUOcy5evDg0AjWHUnXIw4Zfo8rTD+ePE9CkRUxhYhn2N3NY2xym5kiSL2\/mJ0wI9Tw5qlUHnZuUi7qyLUxA1aiAyiNO+CGg6Ny+IQAB9a3FUN8JCOiioc+D6lGOEkRzXlJFYGYEyrZm5BQHe5Q46oKSp4By3dRcphL4sAjURUD37t1L27Ztc9puAwFF56wbAhDQurV4Rd9Xj+JUhMXDhDx0uHr1alq4cOGEhTu6SJoC6nqReBFDuOYcp14mi13ccKwawtUFNCza04dwOQJ1vZ8yjyHcpI+ZpKHsitIdr1USBCCgJWkIVMMegbDFKHqEpy+qCVsMoyJSNYTLJesiq\/Ln4U61gCZsHlLVOKtFRHrEp8+Lzps3b9IiIVNAdVtVV64fLwgKE1DbRUSch5rDTFoEJF1EpIbY1cpp9R764imTJRBQ+36DlNkjAAHNHlPkWAAC+sXFXJw+PKtvQeEhzVtuuWXCVhUWzttuu43uvffeQGB4y4YpqioqVdtbuIyky8TjtnzYLmxas2bNOHphw7Fqe4kpHGbZGzZsCOY5eeGQen89AuVCTAzNbSzmVh62idqCo6J+\/n\/10RG2jUW3DxM\/ff6Z22nu3Ln08ssvByJufuiodzCj8wLohyKAQIAABBREAAJAIECABS1s5a0tPDZzoLZ52aZDBGqLFNLlgQAENA9UkScQKDkC5jyvijb1fZ+urwABdUUM6X1HAALqewui\/kAgJQLm6UxR21NsszcPd3\/88ccDUzWka5uPbTocJm+LFNLlhcD\/B2YOgZgmzWRVAAAAAElFTkSuQmCC","height":280,"width":464}}
%---
