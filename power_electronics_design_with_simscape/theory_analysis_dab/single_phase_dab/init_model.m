preamble;
model = 'single_phase_dab';
%[text] ### Global timing
% simulation length
simlength = 1.25;

% switching frequency and tasks timing
fPWM = 12.5e3;
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
fres = fPWM_DAB/5;
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
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  29.672660942652026"}}
%---
%[output:1d963221]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.120462434577236"}}
%---
%[output:7ef6d9d7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAc8AAAEXCAYAAADLFc9pAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQtsVte151du6C2OQgsmaVJwwJBAkkYVIX1c6tCaipvSKoXbIZqAaSXXIpRxYYhaI7+CRFATG6O4aiiJ5YDr6yq1Qx\/RFEvVpQkpaKjLnTzRKKWFxhji0EwoGJXMhWnpZbS22V\/Pd\/ge57H3OWd9538khP19+3V+a+\/999pnn7WvuXz58mXCBQIgAAIgAAIg4JnANRBPz6yQEARAAARAAAQUAYgnOgIIgAAIgAAI+CQA8fQJDMlBAARAAARAAOKJPgACIAACIAACPglAPH0CQ3IQAAEQAAEQgHiiD4AACIAACICATwIQT5\/AkBwEQAAEQAAEIJ7oAyAAAiAAAiDgkwDE0ycwJAcBEAABEAABiCf6AAiAAAiAAAj4JADx9AkMyUEABEAABEAA4ok+AAIgAAIgAAI+CUA8fQJDchBwEjh27BjV1dXRqVOnMh8vXbqUtm7dSmVlZUVhXbhwgZqbm6mmpoYWLFhQNP2hQ4do1apVWenWrl1LTU1N5LesopUhAQiAQF4CEE90DhAIQYDFs7GxkbZt20Zz5szxLWB+BY\/Fs6Ojg3p6eqi8vDxT37Rp05SA4gIBEIiGAMQzGs6opUQJFBNPp6eoPURGwQLY3d1N1dXVigx\/x57n7t27qaWlJfOZWxDd4skJuQ1tbW302GOPKRFnL3bevHnKox0cHFRlaW+Yf9af82d\/\/vOfqbW1lV577TUaGhqikydP0sqVK2nmzJlZHm5\/fz\/NnTuXGhoa6HOf+xx95zvfIRbsJ554Qt3L4cOHqb29nVasWFGilsZtgUA2AYgnegQIhCCQa9lWi4hTWCsqKpRoVVVVKWHS3uOZM2fUsi+LEF8DAwNqyVeLnHs5N5d4nj17Vonat7\/9bdq1a5cSz1tuuUWVMX36dNLfO0WS62DB27hxI\/X29irxfO655zIeLdejl5FZ0EdGRmjNmjW0evVq9TmLOt8Dp2Mv+IUXXlDi63W5OgRyZAWBRBCAeCbCDGiEVAL5PE8tkloM+fmnFiF9r+7nlCdOnMh4nTqN01vlz7yKJwucXhJm75O9xK6uLiWu3Db2EPOJqn5W6\/aatXhyu7WXzKLKv3Na571KtSfaDQJeCUA8vZJCOhDIQcAtnpxEiyQvyfoVTy1G+WB7Xbbl\/LyxyLncqj3TYuKpvV7+nz3JPXv2ZHmeEE8MBRDAYdjoAyAQikAhz\/Oee+7JbCbyumyrl1Gd6Z3PEQttGNqwYUNm5y7flBZu9\/KsXl7N97lzyVg\/O2XPFZ5nqK6CzCVGAJ5niRkUtxMtgWKvqtjYMOTlVRXe3MPPJ1kgOf358+ev2kiUa8OQfmapNy6xaHI5b7zxhvpDYP369WqZFsu20fYz1JY8AhDP5NkELQKBSAjkWgKOpGJUAgIlQADiWQJGxC2AgFcCzldhOA8\/E\/USnMFr+UgHAmkhAPFMi6VxnyAAAiAAAsYIQDyNoURBIAACIAACaSEA8UyLpXGfIAACIAACxghAPI2hREEgAAIgAAJpIQDxTIulcZ8gAAIgAALGCEA8jaFEQSAAAiAAAmkhAPFMi6VxnyAAAiAAAsYIQDyNoURBIAACIAACaSEA8UyLpXGfIAACIAACxghAPI2hREEgAAIgAAJpIQDxTIulcZ8gAAIgAALGCEA8jaFEQSAAAiAAAmkhAPFMi6VxnyAAAiAAAsYIQDyNoURBIAACIAACaSEA8UyLpXGfIAACIAACxghAPI2hREEgAAIgAAJpIQDxTIulcZ8gAAIgAALGCEA8jaFEQSAAAiAAAmkhAPFMi6VxnyAAAiAAAsYIQDyNoURBIAACIJBuApdOj9D5X\/XRlAc3lzyI1Ivn7NmzS97IuEEQAAEQsElg3qSL9IWp52nJ1Pfp3b9MoK\/971t8Vzc8POw7T5wZIJ6zZ5Mko7HYS2pvnJ07aN1gHJSct3zg641TmFRRMGYv88Kb++n8\/j669N4Ild21iCYtqqWJdy3y3fQo2uu7UUUyQDyFiZHETma609ouD4ztEgZfu3y5dJuM9dLs2E8epQk3VtKkRV+nSZ+vVT8HvWy2N2ibiuWDeAoTz+PHj9OsWbOK2RXfhyAAxiHgecgKvh4ghUxigzGL5tiPt9D5\/f+aEU1TzzYhniENHkd2aUazMSji4J7kOsHYrnXA1y5fLt0UY5NLs4XuWto8zPcCzxOep\/2RLKwGUxOPsNuOrLngax91WMY2lmYhnvbtbryGQ4cO0apVq1S57e3ttGLFikwd0v7iCTsojMMtwQLB2K5Rwdcu3zCep82lWYinfbvT7t27qaWlJasmt+h5bcbZs2epoaGBWltbVZa2tjbq7Oyk8vJy9TvE0yvJ9KTD5G7X1uBrl69f8YxqaRbiadHu2kPMJZRaUPv7+2nBggWeW8FldnR0UE9PD5WVlVFzczPV1NRkyoB4ekaZmoSY3AubeuDld1WCk2cvXvn\/QlaGt\/XnY\/r78f+LXTPKJ9K9t06mGeVl1LQk+M7NYvWk4XsvfTjqpVmIp6Wexx7i4OAg1dbWFqyhr6+vaBpnASyeAwMDtHXrVvUxi2dVVVVm6bb827+ydEcoFgRKl8C0D03IurmPThr\/3fn5tCuffeOfJmelHR0dpYqKiqzPXn3nIr06epFe4f\/fuajKWXrH9eTOW7pEzd5ZLsaZGsZG6fILTxK98jOiKRVEn3yArrnvYbMN8Fja4sWLMymlvb9e8huGioknPE+PvTxFybz81Z4iHMZvtRhf9mjZu+3Ye1zV3bRkFjxRn1ZwM3YuzV58c796N\/PGdb0+S7WXXNo8zCQSJZ7Hjh2juro6OnXqlNrYwxc\/+5w2bRr19vbSnDlzfFsPy7a+kaU+Q7HJPfWAQgLww7dj74gS0YW3TqY96+aHrDk92TVjvTTL72byZSKggQ2KEM8QVC9cuJB5Hjl37lxavXo1rVy5Ui2v8vPOoaEhtfTKzy39XNgw5IcW0jIBP5M7iPkn4Jcve6LLnn5dVbRj5Z208LbsZWD\/LSj9HMdf+590\/W9+YCWggQ16EM8QVFnktmzZQps3b1Y7YXmTT3V1tdrY4\/7ObzXOV1XcG46kGc3vxOOXFdJDPG33gaB9eNlTr9PBt85hGTePgdxLsxxjlmPNsreZ9EvaPMw8E7Nsa1M8C3UcaUYLOvEkffAkqX1gbNcaYfjqZdyaT91MT9XcabehQkp3L81ygPb3b\/8CzfrnGiF3IO+VQYgn3vMUM7iibGiYyT3KdkqtKyzfg384p5Zx0\/4ctFBAg7CMo+5b0pyYxIknP+c8fPhwTrvNmzdPvaupgxuYMq40o0kbFKbsFGU5YGyXtgm+aRVQ99Isn2TCwdndS7MmGNvtBdmlS5uHEyWeURrKWZc0o0kbFHHZNUy9YByGXvG8pvg6NxK9sekzxSsWnCLX0myhszNNMY4KmbR5OFHiyc884XkW76rSBkXxO0peCjC2axPTfHkj0cmxi7Tnm\/OJoxSV0hU01qxpxraZQjwNEeZXU0ZGRqipqUmVyL\/z5QzobqgqxLY1BbKEypE28UhDb4OvFtBSeJXF69JsIbvbYGyzn0E8DdDN9VpK2FdVCjVLmtGkDQoDXSLyIsDYLnJbfKULqN+lWYin3X5arPTEvKqiG6qDJTjjz\/I7nxx1KEiQhGIAIJ7FCKXve1uTe\/pI5r5jm3z1u6C8hCslmELQpVmIZ7wjKnHiyTjczz9t7bTluiCe8XbAJNZuc3JP4v1G3SbbfNcNHFGxcZMuoBwy7\/z+PuJYs\/l2zQa1jW3GQduVL5+0eZjvI5HiadowWLaNkqj8uqRNPNKIR8FXB1PgQAocUCEpl\/sYMA5oUGjXbNB2R8E4aNty5YN4hqBp60iyYk2SZjRpg6IY\/yR+D8Z2rRIVX\/Y+2QtNwqksNpZmsWxrt58WKz1RnmfYw7D1gdl8086lXmdsW\/dB2xDPYl0kfd9HNbmnj+z4HUfJVwdTiCOcX65dszOeHj9mzfYVJWMT9yJtHuZ7TpR4aiM4RVB\/5hY9t8H4OLO2tjbq7OzMBJbnTUaNjY20adMmam1tVVmcafh3aUaTNihMDKyoywBju8Sj5ht1NKKolmbhedrtp8VKT6R4Fmu0l+\/1IdjLly+n733veyq0Hx9n1tzcTDU1Neq0FoinF5LpSxP15J42wnHw5WhEdz\/2Gzr73c9bw51raXbS52vVZqCorzgYh7lHaU5MYj3PMEbQefn1lsrKSpo5cyYNDAyo11z4YvF0vgbDRtPXvn37TFRttYzR0VGqqKiwWkfaCwdjuz0gLr6n\/nyJHn3xT\/TH85dosNbQGBobJXrr3+nyqz8jeusQ0ZQKok8+QNfc97BdiEVKj4ux35tevHhxJsvw8LDf7LGmL0nP0xmhSHughcRTktGk\/UUZa+8OWDkYBwTnMVucfNkDXT9wRIXzCxONKNfS7I3rej0SsJ8sTsZB7g6eZxBqAfPwM866ujoVPGHp0qWZAAra49Sh\/Fg8+TMs2wYEncJs0iYeaSZKAt+gwRT4nUx+N5Pf0eTl2Kg2APm1cRIY+2kzxNMPrQJpnUESdu3aRS+99BLV1tbSnDlzCtbAIlldXZ15nsmJuayGhgZsGDJkmzQUI23ikWaTpPD1GkzBRKzZqG2UFMZe7xvi6ZVUgXTO8HwcHJ7FkC\/93JI3\/eS6nK+j6O+1R8pnhK5atUp93N\/fnyWu0owmbVAY6BKRFwHGdpEnia8OppDrXdAk7JoNaokkMfZyD9LmYb6nxD3zdAaB37lzpxLPuXPn0pYtW2jz5s04DPv4cZo1a5aX\/og0AQlIm3gC3mZs2ZLGVwdT0O+C8pLsxTcPZJZm+aBpPnBa0pU0xsXYQTyLEfLwfS7P88CBAwgMf4WdtEHhweSJSwLGdk2SRL77f\/MG\/VvP9+jL\/3cvzZgykThsXpI2APm1SBIZF7oHiKdfC+dJj8Dw+UFKGxSGukSkxYCxXdxJ4ut+N\/Ppv1TTb+c\/THvWzbcLwXLpSWLs5VYhnl4oJSyNNKNJGxQJM7en5oCxJ0yBE8XNVz\/LvPDb\/epEk4lXgrPz8qypV1kCwzGUMW7Gfm9D2jzM95eYZ55ub9MN39axZNKMJm1Q+B1ESUgPxnatECff00\/VZT3LzBUBqBQENE7GQXqPtHk4UeLpBO4McsCf8+986Xc3gxgnXx5pRpM2KEzaKqqywNgu6aj5jv14ixJMviZ8pFIdAcZeZrEr6LugxcqN4vuoGYe9J2nzcCLF07nbtry8XNkk12dhjaXzSzOatEFhyk5RlgPGdmlHwdd90DSLZZA4s4VeZbFLKVzpUTAO18Ls3NLm4USKp3O3rfY0OfgBRxLiEHv53vMMakhpRpM2KILaJc58YGyXvi2+7ldMTB00rV9lsRlU3jRxW4xNt1OqE5NI8dSe5urVq4mDG\/Bl63knlw3xtDUc5JYrbeKRRtok31yCOfGuak\/Lsn64RX2smZ+25UprknHYtnjJL20eTqx4eoFtKo00o0kbFKbsFGU5YGyXdli+\/AxT75Tl+LLsYdoQTDcFfazZjPKJ9Mamz9iFFLL0sIxDVu87u7R5OJHimW\/XrS3vU5rRpA0K36MoARnA2K4R\/PDVcWV1xB9uWZSCmUtATZzKYpcwkR\/GttvipXxp83AixTMXaN5ty+dy6gOsixmDT1xpbGykbdu2qWDyzri37e3tWbt2pRlN2qAoZqskfg\/Gdq1SiC8vw15670TGs9Riybtkyz62iMruqlbvZcZ56VdZDr51jvZ8cz4tvG1ynM3JWbe0PixtHhYjnn522+oNR6+++ir19vbS1KlTcapK4oZ2shskbeJJNs2rW8d8bzhxQIkke5Z\/PT2ighXoy+lZfuDGytjFMh9f\/SpLEjcSSevDEE9Lo9h5Jqd+fSVfVeylnj59mlg8W1tb6cyZMzjP05JdSrVYaRNPEuyg36PUgqjbxMJ46b0RJZLui0WSPcpxgay+8n+8XqVflkl9lUVaH4Z4+u15OdLne+bpXm7NVRUv1\/b19VF9fT1t2rQpI576ODPO09zcTFVVVZml26F\/+UCmqJtvutnAHdgt4m9\/u0TXXjvBbiUpL70kGI+NRm\/FKRV\/r7O8gmjK9Mzv18xeQPTJB9Tvo6OjVFHhSBt9S43WOHjkfXr0xT\/RJ6ZPpGeWJ2MOkcJ48eLFGVsMDw8btYvtwhITni\/sjfJy7eOPP64OzXYu1bLnWUg8pf3FI+0vyrB2jSM\/GNulXop8k\/YqizTG0uZhHiGJE0+vEYbYy6yrq1PBE\/jQ64ceekh5nPy7vqZNm0bf+ta36Nlnn6Wenh4VYIE9z5qamszmI2lGkzYo7E7DdkoHYztcdamlypc3Ei17+nV1m7yRiF9pieuSxljaPJwo8dQbfQYHB3P2NxZIrxGGWIAbGhrUsi02DMU1fOXWK23ikUa6lPk6g8rH+S6oNMYQTwOj2M\/O2nzVOcXT\/apKf39\/1isv0owmbVAY6BKRFwHGdpGngW\/cQeWlMZY2DyfK89SiuWHDBtq4cWMmNJ8exgiSME5C2qCwOw3bKR2M7XAt9WVbN7U4d+JK68MQT7tjzkrp0owmbVBYMZrlQsHYLuA08dVB5Ws+dTM9VXOnXbCO0qUxljYPJ8rzdPYqflezpaUlq6PB84TnGdXMI23iiYqLqXrSxjeOnbjSGEM8DYwu\/Z5nU1OT53B8YaqVZjRpgyKMbeLKC8Z2yaeRb9RB5aUxljYPJ9LzNLFhyM\/Ql2Y0aYPCjy2SkhaM7VoirXydO3F3rLzTakxcaYylzcOJFE9uFC\/b8qUPw7Y5lKUZTdqgsGk7W2WDsS2yePTABKLYiSutD0ubhxMpnjiSrPDEJW1Q2J2G7ZQOxna46lLBl2jdwBHizUS2gspLYwzxtDvmrJQuzWjSBoUVo1kuFIztAgbfcb76VRYbO3GlMZY2D4vyPLmxHG6PjxnjwAemLmlGkzYoTNkpynLA2C5t8P07X1s7caUxljYPJ1I89TPPkZER4h23+nf+nw\/E5iDvTz75pLHRLc1o0gaFMUNFWBAY24UNvtl8tYByLFxTIf2kMZY2DydSPAsFhufoQ9u3b4d4zppld3ZLeenSJh5p5gLfqy1meieuNMYQTwOjWAeI5yVap+c5NDREjY2N6oQU\/bm7OmdweecSLx+mvWrVKpXcfS6oNKNJGxQGukTkRYCxXeTgm5uvFtCDb51Tp7IsvG1yYENIYyxtHk6k58mNcu+45ehCO3bsoG3btmUdZO3uWR0dHVRZWalecWHBPHDgAK1ZsyZzwgqnb2tro87OTiovL1fZpRlN2qAIPPpjzAjGduGDb2G++lUWDufHm4mCXNIYS5uHEyueQTpLvuAKLKIsqjjPMwjVdOaRNvFIsxL4FrdY2KDy0hhDPIv3CU8pWOy6u7uz0haLbavFkzPxmaB62fbMmTNqkxGfBcoXH4ZdVVWVCcDARtPXvn37PLUvzkSjo6NUUVERZxNKvm4wtmti8PXG99V3LtI3nn+XPjF9Ij2z3J8HKoXx4sWLMzCGh4e9gUlIqmsuX758OSFtUc1wnsX52muvqR22J06cUN8Vijh07NgxqquroyeeeELFxOUoRfycdPny5fT8888XFE9JRpP2F2WS+pbXtoCxV1LB0oGvd25BX2WRxhiep\/c+kTelc\/n16NGjmeeWW7Zsoc2bN2eeVWqxPHXqFC1dulRtJtq0aRO1traq90D1cm19fT11dXVh2daAbdJShLSJR5pdwNefxXgj0bKnX1eZvMbElcYY4umvT+RMzTtm+XUU3ujDS67sTbJAFlu25cKcG4a05+kUVU6DDUMGjFTiRUibeKSZA3z9W8zvqyzSGEM8\/feJnDkOHz5M1113XcaD3Lhxo6fIQs5duk6xdb6q0t\/fn3XUmTSjSRsUhrpEpMWAsV3c4Bucr9eg8tIYS5uH2YKJe+YZvFsFyynNaNIGRTCrxJsLjO3yB99wfL3sxJXGWNo8DPHEe57hRnGJ5pY28UgzA\/iGtxifyMIns+QLKi+NMcQzRJ\/IdxSZLtLLM88g1UszmrRBEcQmcecBY7sWAF8zfAvtxJXGWNo8nCjP0y2e7meTZrrb1aVIM5q0QWHLbjbLBWObdInA1xxf3kh092O\/IQ4qzyH9+H++pDGWNg8nSjyd3SlXeD6OEKRD6pnregjPZ5JlqZQlbeKRxh18zVos105caYwhnmb7RKY0Z4g90wIqzWjSBoWlLmG1WDC2ilecV2SXhrnSnTtxp187RrMEnb4kbR5OrOfpDIDAjeQgCBxer6yszFxPu1KSNKNhYjfeBa4qEIztMgZfe3x5ExFvJlr76cnUvnK+vYoMlyxtHk6UeOZ7R9Owja4qTprRMPHY7hHynhfZJ2K2BvRhszzdpRXbiWu39mClS5uHEyueufBjt+04FUw8wQann1xg7IeW\/7Tg65+Z3xw\/PXhEBZVfeOtk2rMu+R4oxNOvhROQXprRMPHY7zRgbJcx+Nrlq\/\/IvvbDH83sxH1j02fsVxqiBmnzcKI8zxDcQ2WVZjRMPKHM7SkzGHvCFDgR+AZG5zmjZuw3Jq7nCgwnlDYPl5x4Ojca5Ytt297ennW0mTSjYeIxPGpzFAfGdhmDr12+uR7veI2Ja79luWuQNg+XlHjyaSzOg675hBW++HSWhoYGdVQZX9JPVZHYyeIakEHrBeOg5LzlA19vnMKkysVY78R9quZOFdYvSZfEPlFSgeGdR5Lpn\/kwbf6Zgyzwqy4ssDU1NZmTVaQZTVp7kzRAvbYFjL2SCpYOfINx85MrH+Ok7sSV2CdKSjy5c7FQdnd3kw7vxwEWBgYG1HuifDm9U\/6djYYLBEAABNJC4NINt9P7Cxtp8v9YnahbHh4eTlR7ijWmZMRTvyfa1NSkvEq9bFtdXV1QPIsBwvcgAAIgUGoEOKj8wtsml9ptRXo\/YsXTuTmIIxA99NBD9N3vfpc6OztVDFwd0q++vp66urryLttGShuVgQAIgAAIlAQBseLppu\/2PHfv3k1DQ0PU2NhImzZtyrthqCSsiJsAARAAARCIlEDJiCdT8\/KqSlRHnUVqRVQGAiAAAiAQKYGSEs9IyaEyEAABEACB1BKAeKbW9LhxEAABEACBoAQgnkHJIR8IgAAIgEBqCUA8U2t63DgIgAAIgEBQAhDPoOSQDwRAAARAILUEIJ6pNT1uHARAAARAICgBiGdQcsgHAiAAAiCQWgIQz9SaHjcOAiAAAiAQlADEMyg55AMBEAABEEgtAYhnak2PGwcBEAABEAhKAOIZlBzygQAIgAAIpJYAxDO1pseNgwAIgAAIBCUA8QxKDvlAAARAAARSSwDimVrT48ZBAARAAASCEoB4BiWHfCAAAiAAAqklAPFMrelx4yAAAhIIXDo9clUzL7139Wc60V9zpNffXXrvRKBbztWGQAUVyHTjul7TRVotL\/XiOXv2bKuAUTgIgEC6CSyZ+r4CcNMHL9FN\/\/jXDIyb\/\/FS5mf+ji\/nZ6apvfuXCYGL\/D\/\/L3her5X+l3+74DVpItJBPGfPpuHh4UQYw0sjWOwltdfLPSUtDRjbtYh0vhff3E\/au7v45oHMz+wN5vPQJtxYmYE64SPjP3\/A+dmVnyd8ZGYWfE4z8a5Fvg0ijbG09rJBIJ7CxEhiJ\/M98mPOAMZ2DSCJ7+mn6hQMFksWTefFgshCqEWQf5\/y4Ga78DyWLokx35K09kI8BRrt+PHjNGvWLI9DCMmCEADjINS850ki3\/P7\/5W0F6lFUnuLZVc8P0nP5JLIuFAPgXh6Hz+JSSnNaNIGRWIM7aMhYOwDVoCkcfLVy6rnf9VHF367P+NNOr1ISSKZD3+cjAN0CXieQaDFnQfiGbcFkle\/tIkneQQLtyhqviyYF95koTxA7GHyxWLJHuXEu6rVsmuQ54pJ5h4147AspM3DWLbFsm3YPl+S+aVNPNKMEAVfLZjn9\/cp71J7lpMW1ZakWLr7QBSMTfY7iKdJmhGVJc1o0gZFRGY0Wg0YG8V5VWG2+Lo9TO1dlsIyrF+L2GLstx1e00ubh+F5wvP02rdTlU7axCPNOKb5smjyM8yxnzyaWY5No2A6+4Fpxrb7GMTTNuEi5R86dIhWrVqVSdXe3k4rVqwg5+f6M51ImtGkDYqYu0Sg6sE4EDbPmUzwzbUsO2nR1xPzqohnGJYSmmBsqWk5i5U2D5ec57l7924aGRmhpqamjIHOnj1LDQ0N1Nraqj5ra2ujzs5OKi8vV79LM5q0QRHlADRVFxibIpm7nLB8x368JbPxhzf98HPMUtvwE9YCYRmHrd9vfmnzcMmJZ0dHB1VWVipvU1\/sdfLnPT09VFZWRs3NzVRTU0MLFiyAePrt4SlJL23ikWaWIHzdS7PsZU76fK1apsV1NYEgjOPkCPGMkT57mKtXr6bDhw+rVsybN08J5tGjR2lgYIC2bt2qPmfxrKqqygisM7btvn37YrwDb1WPjo5SRUWFt8RIFYgAGAfC5jmTL75jo3T5hSeJXvkZ0ZQKok8+QNfc97DnutKa0BfjGCEtXrw4U7u0sKMlG56Pl3CHhoZo+fLl9PzzzxcUT0lGk\/YXZYzjMnDVYBwYnaeMXviyp\/neU3WZ10w47B17m7i8EfDC2FtJ0aSC5xkNZ0+16OXa+vp66urqwrKtJ2pIxASkTTzSrFaILwcxcL6bCdEMZl1pfRji6dPO7B22tLRk5XLvhvVaJC\/bbtmyhTZv3qw2A\/FzTr7WrFmDDUNeISKdIiBt4pFmNjdfvXOWNwLxz+p5JjYBhTKrtD4M8fRobv3qSC6h1ILa39+f2dTjsdisV1L0M08WUuerKu5ypRlN2qDwarskpQNju9bQfPUmIB0yj3fOpv39TFPkpfVhafMw2ynyZ57sIQ4ODlJtbW3BftLX11c0jYmOJs1o0gaFCRtFXQYY2yXOfCe\/\/MNMUAPsnDXPW1pWWSWUAAAXW0lEQVQfljYPxyKe5rtJuBKlGU3aoAhnnXhyg7Ed7hxjlp9nsqfJr5ggqIEdzhIfPUibh2MVT\/erJc5uNG3aNOrt7aU5c+bY611XSpZmNEzs1rsEnnkaRJxrafb9279As\/65xmAtKMpNQNo8IW0ejlU8uXJ3RCD+na+ZM2eqdzOffPJJ66NCmtGkDQrrBrRQARiHg1osdB74huPrJbc0xtLm4VjF0707lhujP9uwYQNt374d4pljlEgbFF4GetLSgLF\/i+QSzHwbgMDXP1+\/OaQxhnj6sPCFCxdUtB9eotWxaHVgg8bGRnr22WezYtT6KNpXUmlGkzYofBkjIYnB2Jsh+NklHzD919MjmWAGXnbMgq83vmFSSWMsbR6O1fPUnqY7pN6OHTto27ZtWSH0wnSiYnmlGU3aoCjGP4nfg\/HVVnGelanFklMF2fgDvvZ7vTTG0ubhRInnrl276KWXXlKvp0SxUUh3X2lGkzYo7E8T5msAY1I7Yp1epRZK\/p+9y4l3VQcOlwe+5vusu0RpjKXNw7GKp1625SDtfIxYdXW1sr8O4s4noJi6cJ6nKZLpKEfaxOPXKuxFXnpvRC23XnrvhIrq4\/Qm3UJpOnBBqfP1aw8b6aUxhnj66AXODUM7d+5U4jl37tysEHs+isubFOd5mqCYrjKkTTxO6+hoPVoU+TsWRr74PUv3xcuuEz5SSR+4sTKy6D6S+UoZCdIYQzx99KxcnueBAwfo1KlT6gQUU54nzvP0YRQkVQSKTTy5REij00KVDyWLWq6LvT\/35S6LvUW+cqV15tVnXLIo8sXCyBcvtfIV9+kkxfiiG4YnII0xxNOnzfOdwcnxaE1dLJ6FzvMc\/q\/XmKoK5YBAQQKnJtxc8Ps\/XnszfaJi4t\/TTJmenZ7Ps+SYmlfOtZSKW8pZk1L5crulMMZ5ngnuZcXEU9pfPNL+orTVNQZefjdv0SfPXgxV7bmxMfrzZYeIBSjtbQ9tODmW3c4g7Z5RPt7OGVPG\/7+lfCLNKB\/fL8Df1XyqsGAHuLXQWdCHQyMsWoA0xtLmYfVH7OXLly8XtYTBBIXC8nE1ztNQTFSLZVsTFNNVhqSJp2Pv+FLuybMX1P9atFmYc4mxU2zvvW2KytO0ZHxZN6pLEt+omJiuRxpjiKfPHpAvPN+KFSt8lpQ\/OTYMGUOZmoKkTTxeDcPeuhbUX\/9hTGU7+Na5THYtrDWf+qhVUS1Vvl7tEEU6aYwhnj56RaHwfPpAax\/FFUyK8zxNkUxHOdImnrBWYUHlf7++IqQsrG5RZUG999bJtPC2yWGrK7ohK3QFKEAcY4inj07r3G2rPc2Ojg7ju22LNUma0dI2sRezn43vwXicKguqfrY88PIfM17r+LPUjwZe7gVfG702u0xpjKXNw0w78meeThNHsdu2WDeVZjRpg6IY\/yR+D8a5reIU0469x1UiFtKmJbN8bUwCX\/u9XhpjafNw7OJpvwsVr0Ga0aQNiuIWSF4KMPZmk4N\/OKeWellI\/Xij4OuNb5hU0hhLm4djEU\/2NgcHB1UM20JXX19f0TRhOpfOK81o0gaFCRtFXQYY+yOuPVLtjbInWmgHL\/j64xsktTTG0ubhWMSTK9UbeNrb28m9s5Z34La0tFB\/fz8tWLAgSL\/xlUea0aQNCl\/GSEhiMA5uCH51RnuiO1bemXODEfgG5+s1pzTG0ubh2MRTdwAtlM4OkUtQvXaYIOmkGU3aoAhik7jzgHF4C2gRXXjrZNqzbn5WgeAbnm+xEqQxljYPxy6exTpAFN9LM5q0QRGFDU3XAcZmiPJy7vqBI+q1lz3fnJ\/xQsHXDN9CpUhjLG0ehngSkTSjSRsU9qcJ8zWAsVmm6waOqFdenqq5U+3KBV+zfHOVJo2xtHm45MTTGQyBb04vAeM8T\/uDtZRqkDbxSGCvl3HPfvfzEM8IDCatD0M8I+gUhapwh\/vjtAjPF7NRBFYvbeKRgphfbVn29Ou09tOTqX1l9nNQKfcgpZ3S+jDE02fPcgZJ2LVrF7300kvq9ZQ5c+b4LGk8OUcoqqyszNrBi8DwgVCmOpO0iUeSsbSA6iVcSW2X1FZpfRji6aN35ToMm7Prszf9HoadL1rR0aNHC57nyUbT1759+3zcQTxJpZzTFw8dM7WCsRmO+Ur54dDb9OSrf6Nnlt9Mn5ge7ug3uy2VW7qUPozzPAP0MWdg+J07d1J1dTXNnTuXtmzZQiYCw\/MS7tDQEC1fvpyef\/552rp1q2plc3MzVVVVZbxTaX\/xSPuLMkDXiD0LGNs1geZb\/u1f0RubPqOiE+EyS0BaH5Y2D7O1Yottm8vzPHDggOfA8MeOHaO6ujqVfunSpUocnd6qXq6tr6+nrq4u6unpUd+zeNbU1GQCMEgzmrRBYXZKiKY0MLbLWfNd9tTrqiL3e6B2a09H6dL6sLR5OFbx5MpNBoZ3H3HGzz\/5WrNmDTU0NFBra6v6va2tjTo7O6m8vFz9Ls1o0gaFxKkKjO1aTfPl90Dvfuw3KrB81Ady273D+EuX1oelzcOxi6feHcsCt3r1ajp8+HCosHzOV1LmzZunvE0WSZznGf9gltQCaROPJLbcVidfvYHIGURB2v0ksb3S+jDE00cv4mXbxx9\/XO2ufe2112hkZEQ9n+SA8I888kjWEqyPYn0nlWY0aYPCt0ESkAGM7RohF1\/2QPn5Jy4zBKT1YWnzcKyep3vDEL9ict999xnbMOS1C0ozmrRB4dUOSUoHxnatkYsvbx7C8q057tL6sLR5OFbx1J7nl7\/8Zeru7s48k4TnWXgASRsU5qaD6EoCY7usc\/HVEYiw+9YMe2l9GOLp0+76WeTatWuzNvYEDZLgs3qVXJrRpA2KIDaJOw8Y27VAPr68dHvvrZNVDFxc4QhI68PS5uFYPc9wXcNcbmlGkzYozFkqupLA2C7rfHyxecgcd2l9WNo8HLt48uskvGTrvJy7ZM11pfwlSTOatEERhQ1N1wHGpolml1eIL979NMNeWh+WNg\/HKp7OgO2823bmzJl04sQJ1XNWrFhhpgd5KEWa0aQNCg8mSFwSMLZrkkJ84X2aYS+tD0ubh2MXTx2Kj+PPcnQhft\/TVHg+r11QmtGkDQqvdkhSOjC2a41ifPn8z1+\/dQ6vroQwQzHGIYq2klXaPByrePJu2+3btyvBPHPmTCbUnp9lW\/cRZM6IRc6QfTjP00p\/L9lCpU080gxRjK+OPISTV4Jbthjj4CXbyQnx9MmVIwpdd9116ggyFriNGzdSb2+vpyPJ9PNS3qnb1NSkatZHki1btiwTw5aDzSM8n0\/DpDy5tIlHmrm88GXvc+Dld4kPz8bln4AXxv5LtZcD4mmPbVbJ7HHyM1Je6uWLxVN7nfzzggULSHulfFoLiyoCw0dknBKoRtrEIw25V74InBDcsl4ZB6\/BbE6Ip0+eLHAtLS1Zufws2+rg71o8tYfJnqyfI8l0A3Cep08DlmhyKWchSsXvle8z\/36Ouv\/XOXr1v1dKvdXY2u2VcWwNvFIxzvMMYAG3pxigCOVROj3PoOI5PDwcpPpY8kj7izIWSCErBeOQAItk98MXgROC2cIP42A1mM0Fz9MHT\/cRYvmy6nM\/BwcHadq0aVnPRN3iySezYNnWhxGQNCcBaROPNDP64YtXV4JZ1w\/jYDWYzQXx9MmTl1b5Cvpep1M8uRxsGPJpACSHeMbQB\/xO7Aic4N9Ifhn7r8FsDoinB57uA7DdWYI+8+RynGU7d+HiPE8PhkGSDAFpE4800\/nlq19dwZmf3i3tl7H3ku2khHja4Wq1VGlGkzYorBrPUuFgbAnslWKD8OVTVwZe\/iMCJ3g0TRDGHou2kkzaPMwQrrl8+fJlKzQKFHrs2LFMUARnMIOo28H1STOatEERh03D1gnGYQkWzh+UL5ZvvdslKGPvNZhNKW0ejkU89Qagmpoa9T6mfk4Z9LlnWBNKM5q0QRHWPnHkB2O71IPyxfKtd7sEZey9BrMppc3DsYine5cte6F79+6l9evXm7WGx9KkGU3aoPBohkQlA2O75gjDl6MOcfQhPP+0493btXz+0qXNw4kRz76+PnrkkUeorKwscttJM1qYiSdyuEIrBGO7hgvLl5dvT45dxPPPAmYKy9huD7i6dGnzMMQTzzyjHiMi6pM28YiA6mikCb54\/gnPM+5+H\/mGIZOvqpiAJ+0vHhMTjwlupVwGGNu1rgm+\/Pxz2dOv04wpE2nPuvl2GyywdBOMo7xtafNwLJ6nSYO4jyRzvs\/J9bS3t6sADDiSzCT10i9L2sQjzSKm+OoNRAtvnQwBdXUCU4yj6lsQz6hIX4km1N3dTc5gCG4x5eawp4sjySI0TAlUJW3ikYbcNF8+fWVG+UTasfJOWnjbZGk4rLTXNGMrjXQUCvG0TfhK+bmOJOOvcr32wl4njiSLyDAlUo20iUcadtN82QNdP3BEbSKCgI73BtOMbfcxiKdtwq7ycwWG5wO2+dJh\/o4ePUoDAwO0detW9XlzczNVVVVl4ulKM5q0QRFxlzBSHRgbwZi3EFt8eRPRwbfOUdOSWdS0JN3HmNlibKtnSJuHmUPkG4ZMwncHhneWjfM8TZJOV1lSzkKUahWbfF995yI9+uKfFJqld1xP3\/indC7j2mRsst\/hPE+TNF1leT2SzN0EvVxbX19PXV1d1NPTo94jZc9TRzfiPNL+4pH2F6XFrmGtaDC2hjayJUWOhdux97h6FlrzqY+mzhOV1oelzcMl5Xm6Ixdpr3TNmjXYMGR3Liy50qVNPNIMEBVffhbKEYnSKKJRMTbV9yCepkh6LMe9bOt8JcV5tBmOJPMIFMki84zSjDqOiV17osydX22597YpJe2NxsE4TJ+GeIahF1NeaUaTNihiMmuoasE4FL6imePky57owT+MKY+Ur1Jd1o2TcdEOkCOBtHlY\/LJtECO580gzmrRBYcJGUZcBxnaJJ4mv0yNlIeWIRaXglSaJsZfeJG0ehnhiw5CXfp26NNImHmkGSiJf\/XyUWfIzUn2xoPIlbdNREhkX6qcQT2mjWKB4Suxk0roFGNu1mDS+7J3++g9j6h1S56WFlb3VW9hrLS9LzHNUaYyltReeJ8TT7iwptHSJA1kS6lLgy89M2Vvl6+TZC\/T22YsqwpH+zG0PLbTqOeuUcW+WBVf9Xj5+FKPJwA7SGEtrL8TzinhKmnjQVhAAAXkELt7xL5lG\/+d1U9XP\/3ndDQU\/s3GX\/\/Af4wEkglz\/8B9ngmTznOe9H\/43z2mTkFB0hKEkAEQbQAAEQMAmgVzebD4Pl9vx9ti4R5zrKpSv2D2wh23zeqrmTpvFGy8b4mkcKQoEARAAARAodQIQz1K3MO4PBEAABEDAOAGIp3Gk0RbIsX9\/8pOf0O9\/\/3t66KGHaNasWdE2IAW1jY2N0Y9+9CPiYNsrV66ku+++OwV3Hc8t7tmzh2655RaaP39+PA0o4VovXbpEP\/3pT+mVV16hr371q2Ac0tYQz5AA487+y1\/+kqZOnUpz5sxRE\/zXv\/51FQAflzkCfELPHXfcof4988wz9LWvfY2mTJlirgKUpAjw8YE7d+6kBx54gBYsWAAqhgm8+OKL9MEPfpA+\/elP0y9+8Qv64he\/iLkiBGOIZwh4NrNyoPuGhgZqbW1VwsgXT+ItLS3q5\/7+fjXB8GT+pS99Sf213tfXR0uXLqXy8nKbTSuZsr0y1jfMHugPfvADWrt2LV1\/\/fUlw8HmjXhl\/P7779PPf\/5zuummmxRbiKd3q3hlzH2XxfPNN99UfwB+7GMf814JUl5FAOKZwE5x7NgxqqurUy3r7e1V4smftbW1UWdnp\/oLXR\/wzf+zYN54440QTx+29MOYPXkWzu3bt6vj7ObOneujpvQm9cq4vb2d9u\/fT7fffjudPn1aAYN4eus3Xhlv3bqVnn76afrsZz9LH\/\/4x6m7u5tqa2uxguINc85UEM8Q8Gxk5b8ieemKl1QeffRR2rZtmxJPfbg3DwJ+zqm9Uv4rkv+CrKysVOK5YsUK+tCHPmSjaSVTpl\/GH\/7wh5VNeEl8+vTpJcPB5o34Ybxx40Y6cuQIvf322\/TOO++oCf3hhx+Gd1\/EQH4Y8wrWwYMH6d5771XzCf9R\/pWvfAWrVCEGAcQzBDybWfkvysbGxizxHBkZoaamJuJBs3r1avUzL9fy0i1PODwo7r\/\/fpvNKqmyvTBmG7z++utqUr\/hhhuIhfTBBx\/ExO6xJ3hhzP1Ye5p8fCA8T49wryTzypiXxFk0eZVq4sSJanVrwoQJ\/ipD6gwBiGdCO4PXAYHlreAGBOPg7LzmBGOvpIKnA+Pg7MLkhHiGoWcxb64BMTQ0RO5lW72ZyGJTSrZoMLZvWjAGY\/sE4qkB4hkP96K1uiedfBuG8FpKUZR5E4BxcHZec4KxV1LB04FxcHZhckI8w9CzmNc9ILgq\/arKtGnTMrtwLTah5IsGY\/smBmMwtk8gnhognvFwR60gAAIgAAKCCUA8BRsPTQcBEAABEIiHAMQzHu6oFQRAAARAQDABiKdg46HpIAACIAAC8RCAeMbDHbWCAAiAAAgIJgDxFGw8NB0EQAAEQCAeAhDPeLijVhAAARAAAcEEIJ6CjYemgwAIgAAIxEMA4hkPd9QKAiAAAiAgmADEU7Dx0HQQAAEQAIF4CEA84+GOWkEABEAABAQTgHgKNh6aHg8BPoy8ubmZBgcHsxqwdu1adcaq5Itj0e7du1edF8v3WFVVpQ5Y50vfd01NTeb8Tee98vfbt2+nNWvW4JBlyZ0AbfdEAOLpCRMSgcDfCWgRcQpLKfBxih+f1uNXPJmBFt\/169eXAhLcAwjkJQDxROcAAZ8ECoknn3zD566ePHmSVq5cSffccw\/V1dXRqVOnaN68edTT06O8skOHDtGqVauIT8hZtGgRTZo0SXls7PGx98qHnHNZIyMj6nd9og43VXu4XEZ3d7dq\/YEDB2jp0qXqvFcWvo6Ojsx3\/f39Ks3AwID6ni8WRrcHyeWdOHFCeZq57tHpeXJ9um4uz1n3jh07aMmSJYSzZn12LCQXRQDiKcpcaGwSCORattXC+MILL9Bzzz2nRJKvhoYGam1tVUKixdApkpyPhYxFNJ94VldX5xQ+Ln\/jxo3qeLqpU6dmhJc\/Z\/HkNpw5c4ba2trokUceoe9\/\/\/u0efPmzGednZ1Zy6uch+ti4c63NM1lsxhzGn058\/FnfJ986eXeJNgMbQAB0wQgnqaJorySJ+DF82QPb3R0NON1aigslvX19dTV1ZXxQnOJqtPzrKyspJaWliyu7H2y0GmR1Mus7E2y96g9VmcmLXLaU3U+n+V7evzxx6m2tlYJfTHPU4unWzi5bPZg2TOV\/vy35DsybjAUAYhnKHzInEYCfsSTvT63h8fiokWPl3C9iGcuMXSW40U8taixzbSHqe0XRDzzeZgQzzSOivTdM8QzfTbHHYck4FU8OR0\/w+Rnn7yEqZ+HNjY2Em+oYc\/MuWy7YcOGzCadZcuWZZZzWej08mxFRUUmzcyZM3N6nu5lW65v27ZtxHl5N+zvfve7qwRd53Ev2+bbbcvebb6lWSzbhuxgyC6CAMRThJnQyCQR8Cqe7A3y7lOvG4acG4OcG4kKbRjKtWyrl3z1Um97e3vm+aNzE5KbqdNjLLRse\/\/996tl58OHD2eK0M98+Z6dy79JshvaAgImCUA8TdJEWSAQgEAhQQtQXN4sUbyniVdVTFoMZSWZAMQzydZB21JBIArxPHv2rFpCnjFjRuZ1llxwwzyvdD83TYXxcJOpJQDxTK3pceMgAAIgAAJBCUA8g5JDPhAAARAAgdQSgHim1vS4cRAAARAAgaAEIJ5BySEfCIAACIBAaglAPFNretw4CIAACIBAUAIQz6DkkA8EQAAEQCC1BCCeqTU9bhwEQAAEQCAoAYhnUHLIBwIgAAIgkFoCEM\/Umh43DgIgAAIgEJQAxDMoOeQDARAAARBILYH\/D6jnGwWWNBxFAAAAAElFTkSuQmCC","height":279,"width":463}}
%---
%[output:0fd0717d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.120462434577236"],["29.672660942652026"]]}}
%---
%[output:44396bd3]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:8de91a74]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:360d2452]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000080000000000"],["-7.895683520871487","0.998743362938564"]]}}
%---
%[output:0c11bd1d]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.119205797515800","19.904497133076124"]]}}
%---
%[output:2fa74918]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:79eff6bc]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:6d3e5d2e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000080000000000"],["-7.895683520871487","0.998743362938564"]]}}
%---
%[output:14c546cf]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.119205797515800","19.904497133076124"]]}}
%---
%[output:075d78cf]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:6e51d2e7]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:57b55c06]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.023793337349253"}}
%---
%[output:616fcdfb]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.988088839533330"}}
%---
%[output:3623cefa]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.279919958678713"}}
%---
%[output:14798efd]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     1.684282178369838e+02"}}
%---
%[output:2059d561]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.082930235403649e+02"}}
%---
%[output:5c304d9d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAc8AAAEXCAYAAADLFc9pAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQ+UH1WV5mXNumlUhA54Mm0bErAjLI6JiXBiBmHGiDhnTTPHnTnpzpzRaVun3TEyc5JsdwLCHgST7kziLgKuWbfpxd3JH52FMTk7OxEjw8i2OSJ\/WodF0hA6McRoSIg6EnXV3nMrec3rl6p69e6rql+9X311jkfSv3frvfru9+5X99X7c87k5OQk4QICQAAIAAEgAAQyI3AOxDMzVigIBIAAEAACQCBCAOIJIgABIAAEgAAQcEQA4ukIGIoDASAABIAAEIB4ggMNR+DEiRPU29sbtWN4eJhaW1sLa9P4+Dj19PTQ4sWLaXBwkFpaWgqrS79xmc+Y5YEUDp\/4xCdoxYoVWUwqXWZoaIi2bt0atbGvr48GBgac2rtz505av349bdy4sZJ48PPt27ev8P7hBFrNC0M8a06AKjx+mcKSJJ4cnK699lpasmRJIZAkPWPR9SY9jCQYc\/B++OGHnYRJYuPqAK5j5cqVU2bNKJ7N9rLj6uMqlod4VtEraFNpCJw6dYrWrVtHu3fvpm3btpUmnpzxllFvHJAqEC9fvjyzEKrMzEWYJDYSxyvxdGmbWU\/VM0\/F00OHDiH7lJCkABuIZwGgNvKW+vAVt0MfhtKzroULF9Ltt98eNZWDqD6EqbKksbGxs37X73HDDTfQRz7ykahMW1sbjYyMUEdHR+zj6yJlljezMv5dDeMuW7aMPvOZz9CCBQuioKGLju0+PPyr6n3sscei9vGlhm1vvfVW+tSnPhUJp7pMLPjvClNdXFXA1surAKzupQdz\/Rnvvvtu2rRpU2y9hw8fjtp35MiRqTbpPtRxZMzvueceuvfee0k9H+NvYq2wU8PhcUKh\/KrXq57XfC7l6\/b29qkXABO\/Xbt2RcOg6tL5Yd7P9tJi8jHtXmk8TMtQdUy4zartpiCb\/UvHVt1j9erVtHfvXuL+o3ynPzP\/TdWh+9aGSxwPGxlr6l43xLNJGGAGTP2xVACIC5Bm0OP7sHAp4TR\/jwvuacLDvyW1TQW6WbNmTfvmqcRTbwOLVJzY6QJq3icv8YzLbMxAZgbVJFz570ni2d\/fT6tWrToL+zSx4t8uuugiOnbsWPRyECdoXKce5M22J\/FC1fv444\/HCuH9998\/9Z1R55suDqZ4mvdSvycJaBpn2ebgwYOJIq23yRRO8wVHCde73vUu+sY3vjEtKsQJYFz\/MsWPy8S1kf+u6rHdW8el6tlxk4TSzI8B8cwMVbULquCgv3mrwMMt17Muzi5Up9SDk97R9TduPdiyQKnMSN1D1W1mOAqxuN\/1QHDdddcliqf+Zu56H5t4crbNl234NC0z5mz4+PHjZ2GiZ0uM0\/z586c9Y5ZhW3NIWWGv\/MlZpul3bgt\/\/4vLiBnLzs7O6Hn1TDXLJKosQ7BmGfPfChMl9Nx+W92Ke3HPo\/7GL1n8zEnDtjqOik+mTx988MFIhOMyyaT7mqMPKtvW75FWt8pMFf9tuOQxPF3tKBZW6yCeYfkrsbVJb6Uq+HDQWLRoUexMU73MxMREbDbBFev34GxHzYy1TfixvTEniZMeTLh+1\/vkJZ563SyEfOnBOimo6eLx0Y9+NLN4xmXqcfVyO8xh6aTMjsuyCKh26NjG1WcOX6eJZ9JwtWmTlkXGvXiZz6Y+CZgirF4YkkTOxk\/dv\/o9kvxqZrEKKyWeScP1+kxyncuqX+pD5qqj67jEfSpokvAV5GNAPIN029mNLls89aUetuDkKnr8dHFLV7LeRxcGM9DyvfWlKlkyTy6jT7Lhf\/OyCDPzNoO3q3iaOJrZqSnaeYmnYlOSaPMM5DjxNId\/bZlnCOIZN9Kh\/GryLynz1O+R1DcgnuEHXohn+D6MnkA6bGsOL6pvSElv8XHDbDbxTBtu1bMhfg5+O08Sz6z34eEwU9jUcLYpnixQWSZimMKiZ2bm0DeLjW3YlrNim\/iY95AO2+oUT8rmzG5gCqGZhaln1kcg1PMo7pg2ccO2tu5X9LCtetFSGXuSeMZl7AojM\/NMmuBlDhmnDdvG4YJhWxtbyv0d4lku3oXVVvSEoTTxsYlnWtvivgcmiaftPjzEpb5fmkBnEU+2iZttq+5lzpjUNxdwmTCkhu90G673Ax\/4QJQVx12MU9zzZZ0wxPdULxSmaCdNptFt9DJc55133kl33HHHWZOb2MYUT\/5b0uQj9ay2l7W4IU1b5q\/jmPSMacKni9WNN96YyK20e3Ab4iYSZZ0wpONiG3kpLLjgxrEIQDybjBhZl6roy0xsS1XiJiG5DNsyxGlDgrYJOfqOQ2n34XrMZQ233HILPfnkk1MTZOIyTz2wJk160u9tfouNE1ddRHRb\/m8lnnH1fuELX5i2Uw5v3KB\/X5UsVdFFUA\/mccuYkpbImLjq32D5nozb5s2bae3atREc+giCmjWdtPTFtj4zbakK15U1I0v6VsmjD3HClJRtM0Zxy4TistekFy\/+u7mjUdK3Y3WPLCMkTRbOKv04EM9KuyffxtlmNuZbG+6WNwJxw8PmjOqkdbZ6WySbJOT9LHW5n\/6yo14S4mbg2vDAJgk2hMr\/vfLiyR2d17\/xovK4wBCXidjeZMuHuRo1Qjyr4QdpK9KGrdOGm+Pqk2zPJ2133e3ihm0ZE9vGInEvPM2yF3EzcKLS4pllcgMP6axZs4ZuuummxN1tmsFReTwDxDMPFBt7D3MIk1vjKpxsg71Sy\/Wj+TnFRTi5pXjZKddfWWqrtHjyNwcmDV9JmScHgQ0bNtCWLVsKPY0jC5goAwSAABAAAvVAoLLiyW\/Yt912G3V3d0cCmiSeSmCLPsqqHnTAUwIBIAAEgEAWBCornvydgC\/efSPtm6f5PSFttmQWQFAGCAABIAAEgIANgUqKJw\/F3nfffXTzzTcTb0SeJp6clfLUcXUqiPlvE4BLLrnEhgl+BwJAAAgAgRIR4FNo5s2bV2KN\/lVVUjz1A4Jts21NCGzlWTwPHDjgj1wT3gHYpDsV+AAfn24P\/iSjFyI2lRPPuNmECnLbuX9czjaBKEQn+XRYF1tgA3Fw4UvcqA5eTJtLIHz44GIbYuypnHi6ZJJqKcvSpUuJt0pT\/+Zp4AMDA7G+C9FJLiT0KQtsIJ7gjw8C4I8UvRBjT3DiqQSSZ+HyBtxpG3bHOTJEJ0kJ6Wr3\/PPPB\/fdwfUZfcoDn3T0gA\/wkfavEONy5cVT6owkuxCdlDcGSfdD8EPw8+Ea+AP+SPkTYlyGeEq93YR2CH4Ifj60Bn\/AHyl\/IJ5S5Eq0C9FJZcGD4Ifg58M18Af8kfInxLiMzFPq7Sa0Q\/BD8POhNfgD\/kj5A\/GUIleiXYhOKgseBD8EPx+ugT\/gj5Q\/IcZlZJ5SbzehHYIfgp8PrcEf8EfKH4inFLkS7UJ0UlnwIPgh+PlwDfwBf6T8CTEuI\/OUersJ7RD8EPx8aA3+gD9S\/kA8pciVaBeik8qCB8EPwc+Ha+AP+CPlT4hxGZmn1NtNaIfgh+DnQ2vwB\/yR8gfiKUWuRLsQnVQWPAh+CH4+XAN\/wB8pf0KMy8g8pd5uQjsEPwQ\/H1qDP+CPlD8QTylyJdqF6KSy4EHwQ\/Dz4Rr4A\/5I+RNiXEbmKfV2E9oh+CH4+dAa\/AF\/pPyBeEqRK9EuRCeVBQ+CH4KfD9fAH\/BHyp8Q4zIyT6m3m9AOwQ\/Bz4fW4A\/4I+UPxFOKXIl2ITqpLHgQ\/BD8fLgG\/oA\/Uv6EGJeReUq93YR2CH4Ifj60Bn\/AHyl\/IJ5S5Eq0C9FJZcGD4Ifg58M18Af8kfInxLiMzFPq7Sa0Q\/BD8POhNfgD\/kj5A\/GUIleiXYhOKgseBD8EPx+ugT\/gj5Q\/IcZlZJ5SbzehHYIfgp8PrcEf8EfKH4inFLkS7UJ0UlnwIPgh+PlwDfwBf6T8CTEuI\/OUersJ7RD8EPx8aA3+gD9S\/kA8pciVaBeik8qCB8EPwc+Ha+AP+CPlT4hxGZmn1NtNaIfgh+DnQ2vwB\/yR8gfiKUWuRLsQnVQWPAh+CH4+XAN\/wB8pf0KMy8g8pd5uQjsEPwQ\/H1qDP+CPlD8QTylyJdqF6KSy4EHwQ\/Dz4Rr4A\/5I+RNiXEbmKfV2E9oh+CH4+dAa\/AF\/pPyBeEqRK9EuRCeVBQ+CH4KfD9fAH\/BHwp9Hnj1Jf7jubjr6N5+UmDfMBplnw6CvXsUIfgh+PqwEf8AfCX+2P3qUPr79aTrxmd+TmDfMJhfxPHHiBPX29tLY2JjTgyxYsIAeeOABJxvfwsg8kxFE8EPw8+lf4A\/4I+EPxLO3lwYGBmjJkiWZ8Nu3bx8NDQ1BPDOhVU4hBD8EPx+mgT\/gj4Q\/EE+Ip4Q3lbJB8EPw8yEk+AP+SPgztGeChvY8X89hWwlgjbLBsC2GbaXcgzhAHKTcYTvwJx69Woun\/s2zr68vGr6t6gXxhHhKuYngB\/GUcgfimYxcrcVTwcLfMLdu3Rr9s62tjUZGRqijo8OHb7nbQjwhnlJSQTwhnlLuQDwhnpm4Y86+rVI2CvGEeGYicUwhiCfEU8odiCfE05k74+Pj1NPTQ0eOHKlENgrxhHg6k\/iMAcQT4inlDsQT4unDHVLLU4aHh6m1tdXrXlJjiCfEU8odiCfEU8odiGcycrxBAi9XqeUmCWmE0jNPLrdt27bMa0F9iJpkC\/GEeEp5BfGEeEq5A\/GEeGbizqlTp2jdunW0e\/fuqPzy5ctpcHCQWlpaMtkXWQjiCfGU8gviCfGUcgfiCfFM5Y4+27YKWWZcYyGeEE9pAIR4Qjyl3IF4QjxjEdBn1zYyy+Qh4v7+ftq0aVPiEhmIJ8RTGgAhnhBPKXcgnsnIdd7zBD3y3Ml6fvN89tln6cYbb6Rbb7018\/fMvPe2VUPFjz32WOr6UognxFMaACGeEE8pdyCeEM\/UzLORG8MrMeYGIvOUdXGIA8RBxpzTVuAP+CPhz8I7vkmHTvy8nplno48k4\/pvu+026u7ujk5qgXhKKIzgZ0MN4gBxsHEk7XfwJx6d1tUPRT9gqYoPu4S2O3fujCwXLVqEb55CDJE52IFD8IN42lmSXAL8ORsbzjg584R4+jBLaMuThO677z66+eab6fDhw5nEU1W1d+9eYa3Nacb4tbe3N+fD5fBUwCcdROADfFy62bJly+hXF76F\/vnqfoinC3B5leVh2muvvTaaqITZtn6o4s0YmZUPg8Af8MeVP+pEFWSersh5lk\/71pq0kxFm22JYSUo7iAPEQcodtgN\/zkZPLVP5Fy+\/SC9+\/o984C3d9pzJycnJ0mstqEJknn7AonNDHHwYBP6APy780b93znjxGfrRFz\/mYt7wshDPhrugOg1A8EPw82Ej+AP+uPDnkWdPUufnnohMzvvqAE3807dczBtetjDx5Bmw69evjx6Qh1APHjxIo6OjDd\/jFsO2GLaV9jqIA8RByh0M2yYP2c5pnUk\/ufeP6cCBAz7wlm5biHjyJB4+v5O3ylu1ahXx5gkLFiyINotva2uL\/t2oC+IJ8ZRyD+IJ8ZRyB+I5HTk96\/zw0jfS\/f3vg3iqSTwskPPnz6fe3t5ILHk2LM7z9Ol6xdtCHCAOPiwDf8CfLPzRv3Vy1nl31+X0wfcugnhCPLPQp5plEPwQ\/HyYCf6APzb+sHCu2v50tBE8X4\/f\/E6aO2smhTgiWMiwLX\/v5O+b+rCtykK7urpoxYoVNowL+z1EJxUGhnFjBD8EPx+ugT\/gTxoCLJzbHz1KQ3uej4px1vnkJ98Z\/XeIcbkQ8WQweIh25cqV07DcuHFjQ4UzVCf5BDQXWwQ\/BD8XvphlwR\/wJwkBFk4WTRZPJZy7\/vztkYCGGpcLE0+fTlikbYhvOEXiod8bwQ\/Bz4dr4A\/4E4cACycvSeH\/jxNOiKdPryvRFuKZDDaCH4KfT1cEf8AfHQEWy8cO\/YR6v\/jU1J8509QzTvVDiHE598wz6\/FkfX19DVmyEqKTfAKaiy2CH4KfC18wbOuGVp36l74URaH0H95\/Kf3Fu+fEghZiXM5dPBkZnjC0Y8cOGh4eptbW1ggsJao8Yaizs7Nhaz5DdJJbF5WXrlPnlqAEfPByIeGNsqkDf9SEIDVEq4ZpeTnK1W8+PxG+EONy7uKpL1XhtZ36pa\/z3L9\/f3Rw9QMPPODDR2fbEJ3k\/JBCgzp0biE0kRnwgXiCP\/EI\/OWXnqEv7jsy7cekIdq4O4QYlyGePr2hyWwhDhAHH0qDP\/Xiz8P7X6K\/+NL3piYCqadXGx+kZZomUhDPM4jYhm15nadaC3rnnXf69Fdn2xCd5PyQQgMEv3oFPyFNEs3An+bnz7M\/eplWf\/mZqU0O9CeWiKayDzEu5555KjDi1nmqMzZZOO+66y4aGRmhjo6OvPtw6v1CdFJZACH4NX\/wK5JL4E\/z8Ye\/Xf7VVyfor7\/1g9iHY8H84JI2Wv2ei72oFWJcLkw8vZAs0DhEJxUIx7RbI\/g1X\/AriztcD\/gTPn94luyjEz+mh545EZtd8hOyYP7+FRfSv7v2TVObHPjyLMS4DPH09XoT2SP4hR\/8GklH8Ccs\/nBWyf\/b8e0f0KHjP08USyWY\/+PDv03nzZyRm2DqaEE8z6AxPj5OPT090bFk5sVHk+lLWMru7CE6qSyMEPzCCn5l8SJrPeBPtfnDWeVPfv4r+vzD308VSiWW13ZcQH+0eHYklmobvaxccC0XYlzOPfM8depUtIZz6dKlU+s5u7u7zzqezBXcvMqH6KS8nt12HwS\/agc\/m\/8a\/Tv4Ux3+sFAePHGKdj561CqUSizfd8WF9P7fvqgUsTSRCjEu5y6e5jpPXss5d+7caEN4nkS0fft2GhwcpJaWlob09RCdVBZQCH7VCX5l+TzPesCf8vmjhl63P\/oD+v6J9KFX1book7xgJn226zI6\/NIvUjcvyJMfafcKMS4XLp48s3ZiYiLaig+HYZdFRVk9CH7lBz+Zp6ppBf4Uxx8lkju\/fZQOHj+VKZtUGSUL5Sf\/zSX0y19NNiSrzMJWiOcZlDjb5MsUzAcffDA65xOZZxY6lV8Gwa+44Fe+N8uvEfzx44\/a0u7lX\/6a7nroUOZMUs8of+fS82ng+nlTDSn6W2VeLIN4nkFS\/+7Jw7Usplu3bqW2traGrO3UHRyik\/IiqO0+CH5+wc+Gb7P\/Dv5k488rs1yP0iGHLNIcdv3TpW30jotfX\/hknjJ4G2Jczn3YtgygfeoI0Uk+z+tii+CXLfi5YFqnsuDPaW+rDHJykmjL1yZo4sXsw6w6Xzhr5EzyhgVvoHNf\/apKfJssis8hxuXcxTPrxvDqtJWinJF03xCdVBZGCH4QTx+u1Yk\/SiD\/9skf0deePh7B9shzJ53hU5N3Fr7pdfSRq9sj4S1jaYhzQws2CDEuQzwLJkVIt69T8JP4BfjU5+WCl3qwiH3v6M9o13eOiYZXzWHWOa\/7DfW\/\/4pIIF02TZdwNTSbWosnz6pdv3691WeNOgRbNSxEJ1lBzakAxKE+4pATZabdJhT+6GdN\/uP4S\/Slbx8VZ45sqCbl8KzWzgUX0Xv\/9YVTuOgTdkLBpwhu2O4ZYlwuNfO0AVjG7yE6qQxcuA50boinD9eqxB8WyJ\/\/v9\/QZx86FGWNh146vRWd9FLDq29rfx392bvaY8XRdu8q4WNra9m\/hxiXcxfPskF3rS9EJ7k+o7Q8OjfEU8qdMl++WAQnJyfpzq8fIj4iKw9hjDLIC2bSoovPo\/dcNiuCgYdW1TdIH1yULfpXMoohxmWIZx69oknugc4N8fShsi9\/VGY4cfwU\/c3jP4xmqfoKYySKrTOjx+KZq0suOZ\/mzWqZtryjrLWQvvj4+KbqtrUVTzXDdmxszOojbAxvhahhBdC5IZ4+5EvjjxJGPurqWxM\/jjYAyFMYOWu8at7r6XfntzZEGLPghv6FzDMLTypbJsQ3nLLAROeGeLpyTYni\/h+9TH\/9yHN0\/BczoltIlm2YdesTcRZffB71LH3jtCJlZYyumCSVR\/+CeObFpYbcB+KZDDs6N8STEdAn1vxmcpLu\/ofv0\/6jP8tNFPWh1He9+QK6fPZr6P1vuyhoYcwSzNC\/IJ5ZeEJxS1c2btwYna7SyAviCfGU8i\/04Ke2heOM7Ve\/nqR7Hj5E4z98uRBR5GHUK9peS9ddPove\/IZza7v4X+da6PyR9pssdiHG5UImDLFw7tixY9qh1+q7aFdXV0MFNEQnZSFfHmXQucPMPHVR5EX9\/\/upF+m5HGah6mjoQ6hzZrXQsstaafGc86YB9usf\/4DmzXtlU\/I8ONlM90D\/QuaZymdszxdud0fnrpZ4KlGcpEn6+6eO03cP\/zRqYB4TbdST6qL4ptaZ1Pm2i+g1\/2rG1BZxLks1wJ9q8SekSBRiUpN75gnxDImy09uK4Fds8DMX6X\/psaP0\/Iuncpt5mpQpXnrRudE3xY4zw6d5r19U9YI\/xfIn3MhibznE8wxGGLa1k6WKJRD8ZMFP7YPK1t88cJJ4y7e8lmIkCSJniTxs+p7LZ1XmeyL4I+NPFWNB2W2CeGqIY8JQ2fTzrw\/B7xUMVZbI\/\/+9H\/6Mnjj0E9p\/5CQdOzV9Nqo\/6tP3RuXt3979ltYgJ9mAPxBPaX+AeEqRK9EuRCeVBU8zBz99yJTPWbx39IVIEPP+hhj3LfHiWS3RhuEt\/\/JVom+JZfnft55m5o8vNmwPfJJRDDEuF\/bNs9GzapPcFKKT8ui4We4RYufmIVO+Xj3jHNrz1HF6dOLHhQli23kzaMaMGdEeqLz8YuVVvxVtPj416aZ1Zq57oWbxWZXKhMifMvEDPhBPK9\/MIdtt27bRkiVLrHZlFIB4JqPc6M5tTqjZ\/uhROnSimAk1cRki73v6jovPo8tmv2YaSEocG41PGf3Dpw7gg2FbKX9CjMu5Z54meENDQ7R169boz21tbTQyMkIdHR1SjL3tQnSS90NnvEFRwU9NqOFJNH\/3Ty\/Sd1\/If8lFnBjypBqeaXrV3Nfnkh0WhU9G91S+GPCBeEpJGmJcLlw8dTBZSPft2zdt8wQp2FK7EJ0kfVZXuyzBb9q3QyIaGX2Bjv30l4XMLo0TxPmzX0NXXnwevfOS6cdFlbHPaRZ8XDFvpvLAB+Ip5XOIcblw8dQzz0afqMKODdFJUkK62HF2eM7PjtHstjfSlx\/7IY0+d\/pbYp4L8pOyw3kXttCSeedHP0eHDp\/5dqj+7fIcRZaFOEAcfPgF\/iSjF2JcLkQ8fYdqT506RevWraPdu3dHaPf19dHAwEAs8mZZW\/kQneTTYdUuNRecO4P+yzcOR4vyixbEt8x+Da14x2x6w+tePW0CTRnZoQ9WNlsEP4injSNpv4M\/EM9U\/qTtMJSVeCy+fLFg2vbE5d\/XrFlDN910U6Zvqc0mnup7Ii\/M\/9K3j0a45X0cFH87ZCH803e20fdf+kVTL7dA8MvaS88uB3HAy4WUPSHG5UIyTymASXa6mJplxsfHacOGDbRlyxZqbW21Vh2ik\/SH+vj2p6Pvi1KB1Pcy5Vmlfde+iX5w8hdRFW981UvY2DuFQRAHiIM1wIA\/IohCjMuVF09bJssTkFhch4eHm0o8ebj1P37tID137OXMQqkL43uvuJAWtr9u2ixTG6shDhAHG0eQmcsRQv\/CsK2cPY6W6tvp8uXLaXBwkFpaWs66g7mm1DYpqYpvOOrkis9+\/RB97enjVrFUk2o+ds2b6K1tr52aYOP7TRGdG+Lp2EWnFQd\/wB8pf6oYl23PUvnMkx+ARfTIkSOxAmr+llaW78VOUtfevXtt+BT++99972d0y4PHEuvhXW2umXcuvfvSc2nxG2cW2p7Dhw9Te3t7oXWEfHPgk+494AN8XPr3smXLphU\/cOCAi3nDy+YunkUcScbfNfv7+2nTpk3WSUG2slV4w+FJPpv2PB+bYXL2uPxtF9FHr26fGnItiyXIHJA5+HAN\/AF\/pPypQlx2bXsQ4unyXdM2gajRTuq854mzRJMF8y\/ffTG9+7LW0gVTJwyCH4KfawABf7Ijhv6VjFWj43J2L75SMjfxjDuCLK5BaWs2VXl9dq1ax8lb+5lrPdVvS5cupRUrVlBaWXXvRjlpaM8EDe15fhokV196PvVfP4\/4cOIqXOjcEE8fHoI\/4I+UP42Ky9L2sl1u4qkaYZsdm6Wx5sYH+oQh9Vt3d3e02Xxa2bi6ynYSD9F2fu6JaU3546t+i+7quiwLFKWWQfBD8PMhHPgD\/kj5U3ZclrZTt8tdPPNoVJH3KMtJPIN213eO0a27np16HB6e3fXnb2\/o0Gwatgh+CH4+fQ\/8AX+k\/CkrLkvbF2cH8cwTzTP3YuHkbFNtos6ieXfX5ZUZnk16ZAQ\/BD+f7gD+gD9S\/tRWPPWh2vnz51Nvby+NjY3F4mhbhykFP6td0U6KE84qZ5s6bgh+CH5Z+1FcOfAH\/JHyp+i4LG1Xmh0yzxxRDVk4GQYEPwQ\/n+4A\/oA\/Uv5APKXIlWhXlJNM4eSZtLs+\/vYSn8y\/KgQ\/BD8fFoE\/4I+UP0XFZWl7stjlnnmqIdw6DduycK7a\/vTU+k3+xvnkJ9+ZBf9KlUHwQ\/DzIST4A\/5I+QPxTEHO9egwqRNsdkU46fP\/eJhu+tvxqOpQhRPDtjbmYFjbhhDEE+Jp40jS70XEZWlbstrlnnmmVcw7BW3fvj1xk\/esjfYpl7eTtj96lPiYMCWcoUwOisMQwQ\/Bz6dvgT\/gj5Q\/ecdlaTtc7EoXT5fjw1weJGvZPJ3Ew7UL7\/jmlHCGsBwlDScEPwS\/rP0IL1\/uSKF\/JWOWZ1x294zMolTxtJ14InsEN6s8naTvU3tP9+XUfeVst8ZUrDQ6N8TTh5LgD\/gj5U+ecVnaBle73MUzbcIQ7087MjJiPRnF9SFcyuflJDPrDHGCkIkbgh+Cn0tfAn\/c0EL\/QuZpZUzShu1qA3frDQoskJd48nCtOsQ65O+cOtTo3BBPn64H\/oA\/Uv7kFZel9Uvscs88uRFxw7MqI+2KpuhZAAAWc0lEQVTq6opOQGnUlYeT9Kzz3793Lq1\/37xGPU6u9SL4Ifj5EAr8AX+k\/MkjLkvrltrlLp62w7BDn23bjMO1ijwIfgh+0kDCduAP+CPlD8STiGziGfpsW31pCn\/n5HWdzXIh+CH4+XAZ\/AF\/pPyBeBJNna8Z932TD8weHR0Ndp1nM2edyBzs3R7iAHGwsyS5BPiTjA3E8ww2vBnC2rVrp82sHR8fp56eHtq8eXN0iHWjLh8n6VlnMyxNMX2Azg1x8OmX4A\/4I+WPT1yW1ulrl\/s3T9UgFtCVK1dOa9+2bdsaKpzcGB8nqXWdIW\/Bl0YYBD8EP5+AAv6AP1L++MRlaZ2+doWJp2\/DirKXOkkfsh24fh4NXD+3qCY27L4Ifgh+PuQDf8AfKX+kcVlaXx52uYunWuPZ3d3d8CwzDiCpk\/TdhHhd59VvPj8P\/Ct1DwQ\/BD8fQoI\/4I+UP9K4LK0vD7vcxTNttm0eDfa9h8RJetYZ4jmdWTFD8EPwy8qVuHLgD\/gj5Y8kLkvryssud\/HkhlVhVm0SQBInPfTMCfq3W8eiWzbb8hQdJwQ\/BD+fwAL+gD9S\/kjisrSuvOxyF89mPAy72ScKKTIh+CH4+QQW8Af8kfIH4ilFrkQ7VyfpQ7Yb\/6CD+q5pL7G15VaF4Ifg58M48Af8kfLHNS5L68nTLvfMM8\/GFXEvVyd96n8doP+092DTD9nyAyL4Ifj59DnwB\/yR8sc1LkvrydMuF\/HUJwnNnz+fent7aWzs9DdC81qwYAENDw9Ta2trns+R+V4uTmr2HYVM0BD8EPwyd6SYguAP+CPlj0tcltaRt10u4pl3o4q8n4uTdPH8r39yBX3g7W8osmkNvzeCH4KfDwnBH\/BHyh+XuCytI2+7QsSzWc7zHNozQUN7no8wb+ZZtopUCH4Ifj4BBvwBf6T8gXieQa5ZzvOsyyxbiGe2Lg9xgDhkY0p8KfAnGT2IZ4YjyUI5z1Mfsu2+cjbxRvDNfqFzQxx8OA7+gD9S\/kA8M4hnKOd5PvLsSer83BMRF5p1Oz6T6Ah+CH7S4Md24A\/4I+UPxLOJzvOs25Atgp+920McIA52liSXAH8wbGvlT+jnedZlL1tknlYqTyuA4AfxdGPM9NLgD8QzE39CPs9TH7JtxkOvkxyIzg1xyNS5EwqBP+CPlD8YtpUiV6JdFif9\/VMv0srh79bqeyeGbe0khDhAHOwswbCtBKMscVly3yJtClnnWWSDfe+dxUl1\/N4J8bQzC+IJ8bSzBOIpwShLXJbct0gbiKeBbl2\/d0I87d0M4gnxtLME4inBCOIpQa1kG5uT9O+dA9fPo4Hr55bcwsZVB3GAOPiwD\/wBf6T8scVl6X2LtEPmaaCrb8lXl\/WdCgIEPwQ\/n2AD\/oA\/Uv5APKXIlWhnc1Jdv3di2NZOQogDxMHOEgzbSjCyxWXJPYu2KSTzHB8fp56eHjpy5MhZ7a\/ykWR1\/t4J8bR3NYgnxNPOEoinBCOIp7bDUFtbGw0MDEhwLNQmzUl1\/t4J8bTTDuIJ8bSzBOIpwQjiadnbVgJq3jZpTqrz906Ip51pEE+Ip50lEE8JRhBPLfPs7u6mJUuWSHAs1CbNSSu+8B168OnjUf11OL\/TBBriAHHw6XzgD\/gj5Q\/E8wxyvDVfWaenqIO3d+\/eHdXe19eXOlyc5qQ6TxZC5mnv9hAHiIOdJcg8JRhBPLVh27GxsVgM854wxCLNF39fPXHiBPX29lJXVxetWLEitv4kJ9V9shDE097lIZ4QTztLIJ4SjCCeEtRyttHFNO7WSU6q+2QhiKediBBPiKedJRBPCUYQTwlqOdqozJOz0KTvrUlO2v7oUfr49qej1tRtcwTlAogDxMGnO4I\/4I+UPxBPDbmdO3fS+vXro79s27aNDh48SKOjozQ4OEgtLS1SjBPtOOPcunUrLV++PLWOJCep751cQR0nCyHztFMS4gBxsLMEmacEI4jnGdRYyHiDhP7+flq1alX0PZK\/da5bt46KXv+p6k4SaXaSuvbu3Tv13392\/1F67IWfU9t5M2j3h9ol\/g\/e5vDhw9TeXs9nz+I84JOOEvABPln6kSqzbNmyacUPHDjgYt7wsrnvMKQPnc6fPz+awKOGUcuYhcu7G7Fob9q0iTo6Os4COO4NB5OFTsOEzAqZlU9EAn\/AHyl\/kHkamyQ0QjxtAh3nJH2yUF2\/d0I87d0e4gBxsLMEw7YSjCCeZ1Dj7538fVMftlVCmraMRAK6PrtWrflMGxqGeKJzS3iGlws7ani5wMuFnSXxJSCeGi6cAa5cuXIaUhs3bkxcfykF3dwkQTJhCDNtMWybhX8QB4hDFp4klQF\/ktGDePowqyTbOCfVfWchBT06N8TBpxuCP+CPlD8QTylyJdrFOWnhHd8knjQ0p3VmtEylrheCH4KfD\/fBH\/BHyh+Ip4acvs5T\/ZnXezZ6s3jTSZhp+4rTEPwQ\/KTBj+3AH\/BHyh+I5xnkWDh37NhBw8PD1NraGv01y76zUuBd7Ewn6TNt7+m+nLqvnO1yu6Yqi+CH4OdDaPAH\/JHyB+JpOc\/TtoxECryLnekkTBZC5pmVPxAHiENWrsSVA3+S0YN4Biie9z\/xI\/rIf38q8mqd13hi2M0eFhH8IJ52liSXAH8gnlb+cIa5du1aGhkZmdrlp6rDtphpi8zTSugzBRD8IJ5ZuYLM0w0pZJ4ZzvPUIeX9bh944AE3lD1Lm07CTFuIZ1ZKQTwhnlm5AvF0Qwri6YZXQ0rrTsJM2+kugDhAHHw6JfgD\/kj5A\/GUIleiXZJ4Dlw\/jwaun1tiS6pXFYIfgp8PK8Ef8EfKH4inhlzcOs8itudzdZbuJGwIj8zThT8QB4iDC1\/MsuBPMnoQzzPYhLLOc2jPBA3teT5qdd1n2jIG6NwQB4iDDwLgjxQ9iGdgS1VWf\/kZ+m\/fPBL5m7fl4+356nxBPBH8fPgP\/oA\/Uv5APAMTTyxTwbCtS2eHOEAcXPiCYdvsaEE8Axu2VctUrr70fNr18bdn93STloQ4QBx8qA3+gD9S\/kA8NeSqPmEIy1TOpjmCH4KfNPjhm7kdOfSvZIwgnnb+NLyEcpIunlimctot6NwQT58OCv6AP1L+QDylyJVop5yE01SQebrSDuIAcXDljF4e\/EHm6cOfhtsq8dSXqWCmLTLPLMRE8IN4ZuFJUhnwB+Lpw5+G20I8k12Azg1x8Omg4A\/4I+UPhm2lyJVop5yEZSoYtnWlHcQB4uDKGQzbZkMM4pkNp4aWgngi85QSEOIJ8ZRyh+3AHwzb+vCn4bZKPFtXPxS1BWs8X3EJOjfEwaeDgj\/gj5Q\/yDylyJVox076h2\/\/X+INEvha9btz6FOdl5bYgupWheCH4OfDTvAH\/JHyB+IpRa5EO3bSF7\/6OHV+7omoVqzxROaZlX4QB4hDVq7ElQN\/MGzrw5+G25riidNUIJ5ZSYngB\/HMyhWIpxtSyDzd8GpIaYhnMuwQB4iDT6cEf8AfKX8gnlLkSrRjJ\/X9569PneOJDRKQeWalH8QB4pCVK8g83ZCCeLrh1ZDS7KS3rvmf9MhzJ6PzO1k8cZ1GAOIAcfDpC+AP+CPlD8RTilyJdhBPDNtK6QZxgDhIuYOX03TkIJ4+zCrJlp108g+Go9qwxnM66BAHiINPNwR\/wB8pfyCeUuRKtJv71qvoJ+8dimrsvnI23dN9eYm1V7sqBD8EPx+Ggj\/gj5Q\/EE8pciXa6Zkn1ngi83ShHsQB4uDCF7Ms+JOMHsTTh1kl2c656vfpn6\/uj2rjrJOzT1ynEUDnhjj49AXwB\/yR8gfiKUWuRLv23\/0TennRh6MasUECMk8X6kEcIA4ufEHmmR0tiGd2rBpWEuKZDD3EAeLg0zHBH\/BHyh+IpxS5Eu1m\/+Ed9Ms5vxPViA0SkHm6UA\/iAHFw4Qsyz+xoQTyzY9Wwkm\/44OfpVxe+BRskxHgA4gBx8OmY4A\/4I+UPxFOKXIl2EE8M20rpBnGAOEi5w3bgTzJ6EE8fZpVke+HHvky\/OfdCbJCAzNOZcQh+EE9n0mgG4A\/E04c\/DbdtXf1Q1AbsLnS2K9C5IQ4+HRT8AX+k\/EHmKUWuRDslntggAeLpSjuIA8TBlTN6efAHmacPfxpuC\/FMdgE6N8TBp4OCP+CPlD\/IPKXIlWinxBO7CyHzdKUdxAHi4MoZZJ7ZEIN4ZsPJWurEiRPU29tLY2NjUdnly5fT4OAgtbS0nGV76tQpWrduHe3evXvqt76+PhoYGIitR4kndheCeFqJaBSAeEI8XTkD8cyGGMQzG06ppZQYLl26lFasWEHq321tbbGCyEK7Zs0auummm6ijo8PaAiWe2CAB4mklC8TTCSK8XODlwokwWmGIpxQ5i93OnTtpdHQ0NvscHx+nDRs20JYtW6i1tdXaAiWeJz7ze9aydSuA4Ifg58N58Af8kfIH4ilFzkM89+3bR0NDQzQ8PJxZPOe0zoy25sM1HQEEPwQ\/nz4B\/oA\/Uv5APKXIpdip759dXV3RMK55cVa6fv36qT8vWLAgVUg584R4xgOO4Ifg59OFwR\/wR8ofiKcUuQQ79b2Tf06aMMRZ55EjR6Z+N\/9t3prFc8aLz9BrH9lEe\/fuzbnFYd\/u8OHD1N7eHvZDFNh64JMOLvABPi7db9myZdOKHzhwwMW84WXPmZycnGx4K2IakEU449rN30D7+\/tp06ZNsROIWDyxuxAyTwnnkVkhs5LwRtmAP8noIfP0YZZma5thm1aNbQIRiyd2F4pHMEQC50S5TLcBPukwAR\/gk6kjxRQKkTuVzDxtQ68Ke9dlLWwH8Wyutz9pZ5XYhdjBJc8ptQE+EM86cady4mlukKCcoSYC8UYJvClCd3c3LVmyZGodqNokIW1DBSWe5z5+L7360P+R+hl2QAAIAAEgkDMC+OaZM6C4HRAAAkAACACBqiFQucyzagChPUAACAABIAAETAQgnuAEEAACQAAIAAFHBCCejoChOBAAAkAACAABiCc4AASAABAAAkDAEQGIpyNgKA4EgAAQAAJAAOIJDgABIAAEgAAQcESgNuLJGy9s3bo1gmfbtm3RGtE6XrwDU09PT7QfsG1NrI4Zn6c6MjKS6czUkHF1wUc9p7lZR8jPn9Z2F2zM9dp16HMu+Ohl69K30ril+pBavx9CH6qFeOrHlu3fv9\/pCLMQnJi1jXqQ7+zsjDabUIeOm\/cwz1Dlf+\/YsSPz0W9Z21Slci746O1WJ\/ts3Lgx9uSfKj2jtC0u2Jjba9r2m5a2qUp2LvioF4uBgYHoJb4OfSuLcPJGNyG9ZNVCPDmD4ovJGuIbTl5Bwgxi\/FKxffv2xBNr9HrrEAAl+HAgXLNmDZ08eZKSjs3Ly3+NvI8LNrb9pRv5HEXV7YqPfnhFHfpWEu4qA1+8eDEdOnQoitGhjAo2vXgm7X+blHEV1bmqcF\/z4HCXg8Tr0MEl+PCL2ZVXXklf+cpXErP4Kvjetw0u2JijFr51h2Dvgk9c5jk6OprpJTYELFza+MILL0TFedvV3t5eiKcLeEWXjcs0OeDNnTu3aYfYkjA1M02XDCHrZv1F+7PI+7viw\/jdd999tHr1arrtttuaXjz1UYo07rB4TkxMRK6qyzwDV+6ouMRDlX19fZFo1PkyXyhCwKI2maf+IRriORi96WUVTw6Gd911V9NPGHIJgBz8Pv3pT9OHPvSh6ADxtO\/HIQQCWxtdsFHfgNX3K7Zdu3ZtU\/PHBR81VLl58+ZoiNJlBMjmp1B\/h3hW0HMYtn3FKS5DS8qqLsLJz+uCD5d9+OGHp31Hb+ZPAS7YmMO2dZiNDHz8gj\/E0w+\/wqz1TLPuE4Y2bNhAW7ZsodbW1kgs0iYM1W0WoJmJp+GjL+PRidusQ3Au2Ji41aHPueBTx5cLW3CHeNoQatDvWKpyGniX6fR1GGoz6eiCj25bh8zKBRszENZhWNIFn7hh22Yf1raFfoinDaEG\/o5NEk6Dn7aQW0304MkLSZlVSOuwJHTLik\/dxNOFO1xW3yShLpsAuHCHXyhWrlwZ0agu+KT1R4inJFrBBggAASAABIBAYAg0\/WzbwPyB5gIBIAAEgEAACEA8A3ASmggEgAAQAALVQgDiWS1\/oDVAAAgAASAQAAIQzwCchCYCASAABIBAtRCAeFbLH2gNEAACQAAIBIAAxDMAJ6GJ7gg899xzdMEFF0SbQWS5eKr8Sy+9RJdeemmW4qIyavnPggULnI52C2VTfvV8ZS29KLs+kdNh1LQIQDyb1rX1fTDXRfllrDHzEUAf2zJZoB\/9V1a9oWBTFh6opzwEIJ7lYY2aSkKgiuLp2iYdqlAEAuJZEsFRTSUQgHhWwg1ohCsC+g42bKuGQvfv3z+1cwv\/Xe2IZO6YpIYWZ82aFZ0jODY2FjVB7U2rHxml3z9tGFivQx+6VKeMqGfcuHFj7HF4SeWUeHZ2dtLtt98e3cYcGtV3rDHrUQd2X3PNNZG9wur48ePU09NDR44ciUxuueUW2rVrF23atIk6OjqivyU9U5y\/TPHkf\/\/0pz+N\/sdHb+n4xtnHnQNqOxs0lBcLV36jfPURgHhW30dooYFA3F6yeuA2s7ykjbj5toODg9Gev\/pBvEqYu7q6pkQubZN81R51Pz7uzTyNxpZ5muX1\/U9Z4FnkFi9ePHVgst4eFsH+\/v5poqffT70gzJkzZ8refEb172PHjkVHh6lj1lik1VmTtv2O48STz\/NULwtxuOquhXiiq4eEAMQzJG+hrRECtiBsEyq+hx6oTfGMO00lbfP3uOzHLJ\/WJtvG8uZG4tx+W8al\/67E03wZGB0dnRJTvqcujvxv\/QQeRb20odk48eSsll9Q+IVC1cHlhoeHz5rMBfFEBw8JAYhnSN5CW6cQ0Ic4zdmrSUJlDm0uX748NvM0h0912OOGXJPq0zfaTxNP24SlOKGM+5s5lG0OTXMGyYcv8xUngvo9OZtVG5ebtEs6di1OPNlWZa420Yd4ooOHhADEMyRvoa1nIaALhv7dU89ulBia3yFV5mVmnmxrZkxp0CcJY9pQsn4\/X\/Hke6lvl0rc4zJPF\/F8\/PHHaceOHU5LaiCe6KB1QgDiWSdvN\/Gz6gKkMiseGuThwnXr1tHSpUunTdLRBdIUT9dDwMsYtjW\/aep1stClDcGqYVtdPOOyPH3YljNP1zMmixi2tb3I2Iavm5jyeLQGIwDxbLADUL07AnHfPPXsT59AEzfxRWWiatiWW6ALrLo\/D3GqIce4746q5XlNGNIzPf076KJFi86aEGSKp26r2srt48k\/ceKZdcIQ30N9s7R9a\/adMKSG1dUMafUc+kQpky0QT\/f+A4t8EIB45oMj7lIyAvrBw1y1PiSrLzPhYczrrrtu2nIUFs0bbriBbr311khceFmGKagqG1VLWLgO20Hgacs6sk5iWr9+\/RSScUOwagmJKRpm3Zs3b46+a\/IkIfX8eubJlZgYmktVzOU6bJO0zEZl+\/z\/6oUjbqmKbh8nfPr3ZvbTwoUL6cknn4wE3HzJUc9gZuUlUxHV1RQBiGdNHY\/HBgJxWVzcDNusSGX55pn1XlnLIfPMihTK5Y0AxDNvRHE\/IBAAAuZ3XZVl6us6XR8D4umKGMqHjADEM2Tvoe1AwAMBc9elpCUoWaswN2q\/\/\/77I1N9qUrWe2Uph43hs6CEMkUh8P8BPpN+m02joakAAAAASUVORK5CYII=","height":279,"width":463}}
%---
%[output:19da337d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAc8AAAEXCAYAAADLFc9pAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQ+UH1WV5mXNumlUhA54Mm0bErAjLI6JiXBiBmHGiDhnTTPHnTnpzpzRaVun3TEyc5JsdwLCHgST7kziLgKuWbfpxd3JH52FMTk7OxEjw8i2OSJ\/WodF0hA6McRoSIg6EnXV3nMrec3rl6p69e6rql+9X311jkfSv3frvfru9+5X99X7c87k5OQk4QICQAAIAAEgAAQyI3AOxDMzVigIBIAAEAACQCBCAOIJIgABIAAEgAAQcEQA4ukIGIoDASAABIAAEIB4ggMNR+DEiRPU29sbtWN4eJhaW1sLa9P4+Dj19PTQ4sWLaXBwkFpaWgqrS79xmc+Y5YEUDp\/4xCdoxYoVWUwqXWZoaIi2bt0atbGvr48GBgac2rtz505av349bdy4sZJ48PPt27ev8P7hBFrNC0M8a06AKjx+mcKSJJ4cnK699lpasmRJIZAkPWPR9SY9jCQYc\/B++OGHnYRJYuPqAK5j5cqVU2bNKJ7N9rLj6uMqlod4VtEraFNpCJw6dYrWrVtHu3fvpm3btpUmnpzxllFvHJAqEC9fvjyzEKrMzEWYJDYSxyvxdGmbWU\/VM0\/F00OHDiH7lJCkABuIZwGgNvKW+vAVt0MfhtKzroULF9Ltt98eNZWDqD6EqbKksbGxs37X73HDDTfQRz7ykahMW1sbjYyMUEdHR+zj6yJlljezMv5dDeMuW7aMPvOZz9CCBQuioKGLju0+PPyr6n3sscei9vGlhm1vvfVW+tSnPhUJp7pMLPjvClNdXFXA1surAKzupQdz\/Rnvvvtu2rRpU2y9hw8fjtp35MiRqTbpPtRxZMzvueceuvfee0k9H+NvYq2wU8PhcUKh\/KrXq57XfC7l6\/b29qkXABO\/Xbt2RcOg6tL5Yd7P9tJi8jHtXmk8TMtQdUy4zartpiCb\/UvHVt1j9erVtHfvXuL+o3ynPzP\/TdWh+9aGSxwPGxlr6l43xLNJGGAGTP2xVACIC5Bm0OP7sHAp4TR\/jwvuacLDvyW1TQW6WbNmTfvmqcRTbwOLVJzY6QJq3icv8YzLbMxAZgbVJFz570ni2d\/fT6tWrToL+zSx4t8uuugiOnbsWPRyECdoXKce5M22J\/FC1fv444\/HCuH9998\/9Z1R55suDqZ4mvdSvycJaBpn2ebgwYOJIq23yRRO8wVHCde73vUu+sY3vjEtKsQJYFz\/MsWPy8S1kf+u6rHdW8el6tlxk4TSzI8B8cwMVbULquCgv3mrwMMt17Muzi5Up9SDk97R9TduPdiyQKnMSN1D1W1mOAqxuN\/1QHDdddcliqf+Zu56H5t4crbNl234NC0z5mz4+PHjZ2GiZ0uM0\/z586c9Y5ZhW3NIWWGv\/MlZpul3bgt\/\/4vLiBnLzs7O6Hn1TDXLJKosQ7BmGfPfChMl9Nx+W92Ke3HPo\/7GL1n8zEnDtjqOik+mTx988MFIhOMyyaT7mqMPKtvW75FWt8pMFf9tuOQxPF3tKBZW6yCeYfkrsbVJb6Uq+HDQWLRoUexMU73MxMREbDbBFev34GxHzYy1TfixvTEniZMeTLh+1\/vkJZ563SyEfOnBOimo6eLx0Y9+NLN4xmXqcfVyO8xh6aTMjsuyCKh26NjG1WcOX6eJZ9JwtWmTlkXGvXiZz6Y+CZgirF4YkkTOxk\/dv\/o9kvxqZrEKKyWeScP1+kxyncuqX+pD5qqj67jEfSpokvAV5GNAPIN029mNLls89aUetuDkKnr8dHFLV7LeRxcGM9DyvfWlKlkyTy6jT7Lhf\/OyCDPzNoO3q3iaOJrZqSnaeYmnYlOSaPMM5DjxNId\/bZlnCOIZN9Kh\/GryLynz1O+R1DcgnuEHXohn+D6MnkA6bGsOL6pvSElv8XHDbDbxTBtu1bMhfg5+O08Sz6z34eEwU9jUcLYpnixQWSZimMKiZ2bm0DeLjW3YlrNim\/iY95AO2+oUT8rmzG5gCqGZhaln1kcg1PMo7pg2ccO2tu5X9LCtetFSGXuSeMZl7AojM\/NMmuBlDhmnDdvG4YJhWxtbyv0d4lku3oXVVvSEoTTxsYlnWtvivgcmiaftPjzEpb5fmkBnEU+2iZttq+5lzpjUNxdwmTCkhu90G673Ax\/4QJQVx12MU9zzZZ0wxPdULxSmaCdNptFt9DJc55133kl33HHHWZOb2MYUT\/5b0uQj9ay2l7W4IU1b5q\/jmPSMacKni9WNN96YyK20e3Ab4iYSZZ0wpONiG3kpLLjgxrEIQDybjBhZl6roy0xsS1XiJiG5DNsyxGlDgrYJOfqOQ2n34XrMZQ233HILPfnkk1MTZOIyTz2wJk160u9tfouNE1ddRHRb\/m8lnnH1fuELX5i2Uw5v3KB\/X5UsVdFFUA\/mccuYkpbImLjq32D5nozb5s2bae3atREc+giCmjWdtPTFtj4zbakK15U1I0v6VsmjD3HClJRtM0Zxy4TistekFy\/+u7mjUdK3Y3WPLCMkTRbOKv04EM9KuyffxtlmNuZbG+6WNwJxw8PmjOqkdbZ6WySbJOT9LHW5n\/6yo14S4mbg2vDAJgk2hMr\/vfLiyR2d17\/xovK4wBCXidjeZMuHuRo1Qjyr4QdpK9KGrdOGm+Pqk2zPJ2133e3ihm0ZE9vGInEvPM2yF3EzcKLS4pllcgMP6axZs4ZuuummxN1tmsFReTwDxDMPFBt7D3MIk1vjKpxsg71Sy\/Wj+TnFRTi5pXjZKddfWWqrtHjyNwcmDV9JmScHgQ0bNtCWLVsKPY0jC5goAwSAABAAAvVAoLLiyW\/Yt912G3V3d0cCmiSeSmCLPsqqHnTAUwIBIAAEgEAWBCornvydgC\/efSPtm6f5PSFttmQWQFAGCAABIAAEgIANgUqKJw\/F3nfffXTzzTcTb0SeJp6clfLUcXUqiPlvE4BLLrnEhgl+BwJAAAgAgRIR4FNo5s2bV2KN\/lVVUjz1A4Jts21NCGzlWTwPHDjgj1wT3gHYpDsV+AAfn24P\/iSjFyI2lRPPuNmECnLbuX9czjaBKEQn+XRYF1tgA3Fw4UvcqA5eTJtLIHz44GIbYuypnHi6ZJJqKcvSpUuJt0pT\/+Zp4AMDA7G+C9FJLiT0KQtsIJ7gjw8C4I8UvRBjT3DiqQSSZ+HyBtxpG3bHOTJEJ0kJ6Wr3\/PPPB\/fdwfUZfcoDn3T0gA\/wkfavEONy5cVT6owkuxCdlDcGSfdD8EPw8+Ea+AP+SPkTYlyGeEq93YR2CH4Ifj60Bn\/AHyl\/IJ5S5Eq0C9FJZcGD4Ifg58M18Af8kfInxLiMzFPq7Sa0Q\/BD8POhNfgD\/kj5A\/GUIleiXYhOKgseBD8EPx+ugT\/gj5Q\/IcZlZJ5SbzehHYIfgp8PrcEf8EfKH4inFLkS7UJ0UlnwIPgh+PlwDfwBf6T8CTEuI\/OUersJ7RD8EPx8aA3+gD9S\/kA8pciVaBeik8qCB8EPwc+Ha+AP+CPlT4hxGZmn1NtNaIfgh+DnQ2vwB\/yR8gfiKUWuRLsQnVQWPAh+CH4+XAN\/wB8pf0KMy8g8pd5uQjsEPwQ\/H1qDP+CPlD8QTylyJdqF6KSy4EHwQ\/Dz4Rr4A\/5I+RNiXEbmKfV2E9oh+CH4+dAa\/AF\/pPyBeEqRK9EuRCeVBQ+CH4KfD9fAH\/BHyp8Q4zIyT6m3m9AOwQ\/Bz4fW4A\/4I+UPxFOKXIl2ITqpLHgQ\/BD8fLgG\/oA\/Uv6EGJeReUq93YR2CH4Ifj60Bn\/AHyl\/IJ5S5Eq0C9FJZcGD4Ifg58M18Af8kfInxLiMzFPq7Sa0Q\/BD8POhNfgD\/kj5A\/GUIleiXYhOKgseBD8EPx+ugT\/gj5Q\/IcZlZJ5SbzehHYIfgp8PrcEf8EfKH4inFLkS7UJ0UlnwIPgh+PlwDfwBf6T8CTEuI\/OUersJ7RD8EPx8aA3+gD9S\/kA8pciVaBeik8qCB8EPwc+Ha+AP+CPlT4hxGZmn1NtNaIfgh+DnQ2vwB\/yR8gfiKUWuRLsQnVQWPAh+CH4+XAN\/wB8pf0KMy8g8pd5uQjsEPwQ\/H1qDP+CPlD8QTylyJdqF6KSy4EHwQ\/Dz4Rr4A\/5I+RNiXEbmKfV2E9oh+CH4+dAa\/AF\/pPyBeEqRK9EuRCeVBQ+CH4KfD9fAH\/BHwp9Hnj1Jf7jubjr6N5+UmDfMBplnw6CvXsUIfgh+PqwEf8AfCX+2P3qUPr79aTrxmd+TmDfMJhfxPHHiBPX29tLY2JjTgyxYsIAeeOABJxvfwsg8kxFE8EPw8+lf4A\/4I+EPxLO3lwYGBmjJkiWZ8Nu3bx8NDQ1BPDOhVU4hBD8EPx+mgT\/gj4Q\/EE+Ip4Q3lbJB8EPw8yEk+AP+SPgztGeChvY8X89hWwlgjbLBsC2GbaXcgzhAHKTcYTvwJx69Woun\/s2zr68vGr6t6gXxhHhKuYngB\/GUcgfimYxcrcVTwcLfMLdu3Rr9s62tjUZGRqijo8OHb7nbQjwhnlJSQTwhnlLuQDwhnpm4Y86+rVI2CvGEeGYicUwhiCfEU8odiCfE05k74+Pj1NPTQ0eOHKlENgrxhHg6k\/iMAcQT4inlDsQT4unDHVLLU4aHh6m1tdXrXlJjiCfEU8odiCfEU8odiGcycrxBAi9XqeUmCWmE0jNPLrdt27bMa0F9iJpkC\/GEeEp5BfGEeEq5A\/GEeGbizqlTp2jdunW0e\/fuqPzy5ctpcHCQWlpaMtkXWQjiCfGU8gviCfGUcgfiCfFM5Y4+27YKWWZcYyGeEE9pAIR4Qjyl3IF4QjxjEdBn1zYyy+Qh4v7+ftq0aVPiEhmIJ8RTGgAhnhBPKXcgnsnIdd7zBD3y3Ml6fvN89tln6cYbb6Rbb7018\/fMvPe2VUPFjz32WOr6UognxFMaACGeEE8pdyCeEM\/UzLORG8MrMeYGIvOUdXGIA8RBxpzTVuAP+CPhz8I7vkmHTvy8nplno48k4\/pvu+026u7ujk5qgXhKKIzgZ0MN4gBxsHEk7XfwJx6d1tUPRT9gqYoPu4S2O3fujCwXLVqEb55CDJE52IFD8IN42lmSXAL8ORsbzjg584R4+jBLaMuThO677z66+eab6fDhw5nEU1W1d+9eYa3Nacb4tbe3N+fD5fBUwCcdROADfFy62bJly+hXF76F\/vnqfoinC3B5leVh2muvvTaaqITZtn6o4s0YmZUPg8Af8MeVP+pEFWSersh5lk\/71pq0kxFm22JYSUo7iAPEQcodtgN\/zkZPLVP5Fy+\/SC9+\/o984C3d9pzJycnJ0mstqEJknn7AonNDHHwYBP6APy780b93znjxGfrRFz\/mYt7wshDPhrugOg1A8EPw82Ej+AP+uPDnkWdPUufnnohMzvvqAE3807dczBtetjDx5Bmw69evjx6Qh1APHjxIo6OjDd\/jFsO2GLaV9jqIA8RByh0M2yYP2c5pnUk\/ufeP6cCBAz7wlm5biHjyJB4+v5O3ylu1ahXx5gkLFiyINotva2uL\/t2oC+IJ8ZRyD+IJ8ZRyB+I5HTk96\/zw0jfS\/f3vg3iqSTwskPPnz6fe3t5ILHk2LM7z9Ol6xdtCHCAOPiwDf8CfLPzRv3Vy1nl31+X0wfcugnhCPLPQp5plEPwQ\/HyYCf6APzb+sHCu2v50tBE8X4\/f\/E6aO2smhTgiWMiwLX\/v5O+b+rCtykK7urpoxYoVNowL+z1EJxUGhnFjBD8EPx+ugT\/gTxoCLJzbHz1KQ3uej4px1vnkJ98Z\/XeIcbkQ8WQweIh25cqV07DcuHFjQ4UzVCf5BDQXWwQ\/BD8XvphlwR\/wJwkBFk4WTRZPJZy7\/vztkYCGGpcLE0+fTlikbYhvOEXiod8bwQ\/Bz4dr4A\/4E4cACycvSeH\/jxNOiKdPryvRFuKZDDaCH4KfT1cEf8AfHQEWy8cO\/YR6v\/jU1J8509QzTvVDiHE598wz6\/FkfX19DVmyEqKTfAKaiy2CH4KfC18wbOuGVp36l74URaH0H95\/Kf3Fu+fEghZiXM5dPBkZnjC0Y8cOGh4eptbW1ggsJao8Yaizs7Nhaz5DdJJbF5WXrlPnlqAEfPByIeGNsqkDf9SEIDVEq4ZpeTnK1W8+PxG+EONy7uKpL1XhtZ36pa\/z3L9\/f3Rw9QMPPODDR2fbEJ3k\/JBCgzp0biE0kRnwgXiCP\/EI\/OWXnqEv7jsy7cekIdq4O4QYlyGePr2hyWwhDhAHH0qDP\/Xiz8P7X6K\/+NL3piYCqadXGx+kZZomUhDPM4jYhm15nadaC3rnnXf69Fdn2xCd5PyQQgMEv3oFPyFNEs3An+bnz7M\/eplWf\/mZqU0O9CeWiKayDzEu5555KjDi1nmqMzZZOO+66y4aGRmhjo6OvPtw6v1CdFJZACH4NX\/wK5JL4E\/z8Ye\/Xf7VVyfor7\/1g9iHY8H84JI2Wv2ei72oFWJcLkw8vZAs0DhEJxUIx7RbI\/g1X\/AriztcD\/gTPn94luyjEz+mh545EZtd8hOyYP7+FRfSv7v2TVObHPjyLMS4DPH09XoT2SP4hR\/8GklH8Ccs\/nBWyf\/b8e0f0KHjP08USyWY\/+PDv03nzZyRm2DqaEE8z6AxPj5OPT090bFk5sVHk+lLWMru7CE6qSyMEPzCCn5l8SJrPeBPtfnDWeVPfv4r+vzD308VSiWW13ZcQH+0eHYklmobvaxccC0XYlzOPfM8depUtIZz6dKlU+s5u7u7zzqezBXcvMqH6KS8nt12HwS\/agc\/m\/8a\/Tv4Ux3+sFAePHGKdj561CqUSizfd8WF9P7fvqgUsTSRCjEu5y6e5jpPXss5d+7caEN4nkS0fft2GhwcpJaWlob09RCdVBZQCH7VCX5l+TzPesCf8vmjhl63P\/oD+v6J9KFX1book7xgJn226zI6\/NIvUjcvyJMfafcKMS4XLp48s3ZiYiLaig+HYZdFRVk9CH7lBz+Zp6ppBf4Uxx8lkju\/fZQOHj+VKZtUGSUL5Sf\/zSX0y19NNiSrzMJWiOcZlDjb5MsUzAcffDA65xOZZxY6lV8Gwa+44Fe+N8uvEfzx44\/a0u7lX\/6a7nroUOZMUs8of+fS82ng+nlTDSn6W2VeLIN4nkFS\/+7Jw7Usplu3bqW2traGrO3UHRyik\/IiqO0+CH5+wc+Gb7P\/Dv5k488rs1yP0iGHLNIcdv3TpW30jotfX\/hknjJ4G2Jczn3YtgygfeoI0Uk+z+tii+CXLfi5YFqnsuDPaW+rDHJykmjL1yZo4sXsw6w6Xzhr5EzyhgVvoHNf\/apKfJssis8hxuXcxTPrxvDqtJWinJF03xCdVBZGCH4QTx+u1Yk\/SiD\/9skf0deePh7B9shzJ53hU5N3Fr7pdfSRq9sj4S1jaYhzQws2CDEuQzwLJkVIt69T8JP4BfjU5+WCl3qwiH3v6M9o13eOiYZXzWHWOa\/7DfW\/\/4pIIF02TZdwNTSbWosnz6pdv3691WeNOgRbNSxEJ1lBzakAxKE+4pATZabdJhT+6GdN\/uP4S\/Slbx8VZ45sqCbl8KzWzgUX0Xv\/9YVTuOgTdkLBpwhu2O4ZYlwuNfO0AVjG7yE6qQxcuA50boinD9eqxB8WyJ\/\/v9\/QZx86FGWNh146vRWd9FLDq29rfx392bvaY8XRdu8q4WNra9m\/hxiXcxfPskF3rS9EJ7k+o7Q8OjfEU8qdMl++WAQnJyfpzq8fIj4iKw9hjDLIC2bSoovPo\/dcNiuCgYdW1TdIH1yULfpXMoohxmWIZx69oknugc4N8fShsi9\/VGY4cfwU\/c3jP4xmqfoKYySKrTOjx+KZq0suOZ\/mzWqZtryjrLWQvvj4+KbqtrUVTzXDdmxszOojbAxvhahhBdC5IZ4+5EvjjxJGPurqWxM\/jjYAyFMYOWu8at7r6XfntzZEGLPghv6FzDMLTypbJsQ3nLLAROeGeLpyTYni\/h+9TH\/9yHN0\/BczoltIlm2YdesTcRZffB71LH3jtCJlZYyumCSVR\/+CeObFpYbcB+KZDDs6N8STEdAn1vxmcpLu\/ofv0\/6jP8tNFPWh1He9+QK6fPZr6P1vuyhoYcwSzNC\/IJ5ZeEJxS1c2btwYna7SyAviCfGU8i\/04Ke2heOM7Ve\/nqR7Hj5E4z98uRBR5GHUK9peS9ddPove\/IZza7v4X+da6PyR9pssdiHG5UImDLFw7tixY9qh1+q7aFdXV0MFNEQnZSFfHmXQucPMPHVR5EX9\/\/upF+m5HGah6mjoQ6hzZrXQsstaafGc86YB9usf\/4DmzXtlU\/I8ONlM90D\/QuaZymdszxdud0fnrpZ4KlGcpEn6+6eO03cP\/zRqYB4TbdST6qL4ptaZ1Pm2i+g1\/2rG1BZxLks1wJ9q8SekSBRiUpN75gnxDImy09uK4Fds8DMX6X\/psaP0\/Iuncpt5mpQpXnrRudE3xY4zw6d5r19U9YI\/xfIn3MhibznE8wxGGLa1k6WKJRD8ZMFP7YPK1t88cJJ4y7e8lmIkCSJniTxs+p7LZ1XmeyL4I+NPFWNB2W2CeGqIY8JQ2fTzrw\/B7xUMVZbI\/\/+9H\/6Mnjj0E9p\/5CQdOzV9Nqo\/6tP3RuXt3979ltYgJ9mAPxBPaX+AeEqRK9EuRCeVBU8zBz99yJTPWbx39IVIEPP+hhj3LfHiWS3RhuEt\/\/JVom+JZfnft55m5o8vNmwPfJJRDDEuF\/bNs9GzapPcFKKT8ui4We4RYufmIVO+Xj3jHNrz1HF6dOLHhQli23kzaMaMGdEeqLz8YuVVvxVtPj416aZ1Zq57oWbxWZXKhMifMvEDPhBPK9\/MIdtt27bRkiVLrHZlFIB4JqPc6M5tTqjZ\/uhROnSimAk1cRki73v6jovPo8tmv2YaSEocG41PGf3Dpw7gg2FbKX9CjMu5Z54meENDQ7R169boz21tbTQyMkIdHR1SjL3tQnSS90NnvEFRwU9NqOFJNH\/3Ty\/Sd1\/If8lFnBjypBqeaXrV3Nfnkh0WhU9G91S+GPCBeEpJGmJcLlw8dTBZSPft2zdt8wQp2FK7EJ0kfVZXuyzBb9q3QyIaGX2Bjv30l4XMLo0TxPmzX0NXXnwevfOS6cdFlbHPaRZ8XDFvpvLAB+Ip5XOIcblw8dQzz0afqMKODdFJUkK62HF2eM7PjtHstjfSlx\/7IY0+d\/pbYp4L8pOyw3kXttCSeedHP0eHDp\/5dqj+7fIcRZaFOEAcfPgF\/iSjF2JcLkQ8fYdqT506RevWraPdu3dHaPf19dHAwEAs8mZZW\/kQneTTYdUuNRecO4P+yzcOR4vyixbEt8x+Da14x2x6w+tePW0CTRnZoQ9WNlsEP4injSNpv4M\/EM9U\/qTtMJSVeCy+fLFg2vbE5d\/XrFlDN910U6Zvqc0mnup7Ii\/M\/9K3j0a45X0cFH87ZCH803e20fdf+kVTL7dA8MvaS88uB3HAy4WUPSHG5UIyTymASXa6mJplxsfHacOGDbRlyxZqbW21Vh2ik\/SH+vj2p6Pvi1KB1Pcy5Vmlfde+iX5w8hdRFW981UvY2DuFQRAHiIM1wIA\/IohCjMuVF09bJssTkFhch4eHm0o8ebj1P37tID137OXMQqkL43uvuJAWtr9u2ixTG6shDhAHG0eQmcsRQv\/CsK2cPY6W6tvp8uXLaXBwkFpaWs66g7mm1DYpqYpvOOrkis9+\/RB97enjVrFUk2o+ds2b6K1tr52aYOP7TRGdG+Lp2EWnFQd\/wB8pf6oYl23PUvnMkx+ARfTIkSOxAmr+llaW78VOUtfevXtt+BT++99972d0y4PHEuvhXW2umXcuvfvSc2nxG2cW2p7Dhw9Te3t7oXWEfHPgk+494AN8XPr3smXLphU\/cOCAi3nDy+YunkUcScbfNfv7+2nTpk3WSUG2slV4w+FJPpv2PB+bYXL2uPxtF9FHr26fGnItiyXIHJA5+HAN\/AF\/pPypQlx2bXsQ4unyXdM2gajRTuq854mzRJMF8y\/ffTG9+7LW0gVTJwyCH4KfawABf7Ijhv6VjFWj43J2L75SMjfxjDuCLK5BaWs2VXl9dq1ax8lb+5lrPdVvS5cupRUrVlBaWXXvRjlpaM8EDe15fhokV196PvVfP4\/4cOIqXOjcEE8fHoI\/4I+UP42Ky9L2sl1u4qkaYZsdm6Wx5sYH+oQh9Vt3d3e02Xxa2bi6ynYSD9F2fu6JaU3546t+i+7quiwLFKWWQfBD8PMhHPgD\/kj5U3ZclrZTt8tdPPNoVJH3KMtJPIN213eO0a27np16HB6e3fXnb2\/o0Gwatgh+CH4+fQ\/8AX+k\/CkrLkvbF2cH8cwTzTP3YuHkbFNtos6ieXfX5ZUZnk16ZAQ\/BD+f7gD+gD9S\/tRWPPWh2vnz51Nvby+NjY3F4mhbhykFP6td0U6KE84qZ5s6bgh+CH5Z+1FcOfAH\/JHyp+i4LG1Xmh0yzxxRDVk4GQYEPwQ\/n+4A\/oA\/Uv5APKXIlWhXlJNM4eSZtLs+\/vYSn8y\/KgQ\/BD8fFoE\/4I+UP0XFZWl7stjlnnmqIdw6DduycK7a\/vTU+k3+xvnkJ9+ZBf9KlUHwQ\/DzIST4A\/5I+QPxTEHO9egwqRNsdkU46fP\/eJhu+tvxqOpQhRPDtjbmYFjbhhDEE+Jp40jS70XEZWlbstrlnnmmVcw7BW3fvj1xk\/esjfYpl7eTtj96lPiYMCWcoUwOisMQwQ\/Bz6dvgT\/gj5Q\/ecdlaTtc7EoXT5fjw1weJGvZPJ3Ew7UL7\/jmlHCGsBwlDScEPwS\/rP0IL1\/uSKF\/JWOWZ1x294zMolTxtJ14InsEN6s8naTvU3tP9+XUfeVst8ZUrDQ6N8TTh5LgD\/gj5U+ecVnaBle73MUzbcIQ7087MjJiPRnF9SFcyuflJDPrDHGCkIkbgh+Cn0tfAn\/c0EL\/QuZpZUzShu1qA3frDQoskJd48nCtOsQ65O+cOtTo3BBPn64H\/oA\/Uv7kFZel9Uvscs88uRFxw7MqI+2KpuhZAAAWc0lEQVTq6opOQGnUlYeT9Kzz3793Lq1\/37xGPU6u9SL4Ifj5EAr8AX+k\/MkjLkvrltrlLp62w7BDn23bjMO1ijwIfgh+0kDCduAP+CPlD8STiGziGfpsW31pCn\/n5HWdzXIh+CH4+XAZ\/AF\/pPyBeBJNna8Z932TD8weHR0Ndp1nM2edyBzs3R7iAHGwsyS5BPiTjA3E8ww2vBnC2rVrp82sHR8fp56eHtq8eXN0iHWjLh8n6VlnMyxNMX2Azg1x8OmX4A\/4I+WPT1yW1ulrl\/s3T9UgFtCVK1dOa9+2bdsaKpzcGB8nqXWdIW\/Bl0YYBD8EP5+AAv6AP1L++MRlaZ2+doWJp2\/DirKXOkkfsh24fh4NXD+3qCY27L4Ifgh+PuQDf8AfKX+kcVlaXx52uYunWuPZ3d3d8CwzDiCpk\/TdhHhd59VvPj8P\/Ct1DwQ\/BD8fQoI\/4I+UP9K4LK0vD7vcxTNttm0eDfa9h8RJetYZ4jmdWTFD8EPwy8qVuHLgD\/gj5Y8kLkvryssud\/HkhlVhVm0SQBInPfTMCfq3W8eiWzbb8hQdJwQ\/BD+fwAL+gD9S\/kjisrSuvOxyF89mPAy72ScKKTIh+CH4+QQW8Af8kfIH4ilFrkQ7VyfpQ7Yb\/6CD+q5pL7G15VaF4Ifg58M48Af8kfLHNS5L68nTLvfMM8\/GFXEvVyd96n8doP+092DTD9nyAyL4Ifj59DnwB\/yR8sc1LkvrydMuF\/HUJwnNnz+fent7aWzs9DdC81qwYAENDw9Ta2trns+R+V4uTmr2HYVM0BD8EPwyd6SYguAP+CPlj0tcltaRt10u4pl3o4q8n4uTdPH8r39yBX3g7W8osmkNvzeCH4KfDwnBH\/BHyh+XuCytI2+7QsSzWc7zHNozQUN7no8wb+ZZtopUCH4Ifj4BBvwBf6T8gXieQa5ZzvOsyyxbiGe2Lg9xgDhkY0p8KfAnGT2IZ4YjyUI5z1Mfsu2+cjbxRvDNfqFzQxx8OA7+gD9S\/kA8M4hnKOd5PvLsSer83BMRF5p1Oz6T6Ah+CH7S4Md24A\/4I+UPxLOJzvOs25Atgp+920McIA52liSXAH8wbGvlT+jnedZlL1tknlYqTyuA4AfxdGPM9NLgD8QzE39CPs9TH7JtxkOvkxyIzg1xyNS5EwqBP+CPlD8YtpUiV6JdFif9\/VMv0srh79bqeyeGbe0khDhAHOwswbCtBKMscVly3yJtClnnWWSDfe+dxUl1\/N4J8bQzC+IJ8bSzBOIpwShLXJbct0gbiKeBbl2\/d0I87d0M4gnxtLME4inBCOIpQa1kG5uT9O+dA9fPo4Hr55bcwsZVB3GAOPiwD\/wBf6T8scVl6X2LtEPmaaCrb8lXl\/WdCgIEPwQ\/n2AD\/oA\/Uv5APKXIlWhnc1Jdv3di2NZOQogDxMHOEgzbSjCyxWXJPYu2KSTzHB8fp56eHjpy5MhZ7a\/ykWR1\/t4J8bR3NYgnxNPOEoinBCOIp7bDUFtbGw0MDEhwLNQmzUl1\/t4J8bTTDuIJ8bSzBOIpwQjiadnbVgJq3jZpTqrz906Ip51pEE+Ip50lEE8JRhBPLfPs7u6mJUuWSHAs1CbNSSu+8B168OnjUf11OL\/TBBriAHHw6XzgD\/gj5Q\/E8wxyvDVfWaenqIO3d+\/eHdXe19eXOlyc5qQ6TxZC5mnv9hAHiIOdJcg8JRhBPLVh27GxsVgM854wxCLNF39fPXHiBPX29lJXVxetWLEitv4kJ9V9shDE097lIZ4QTztLIJ4SjCCeEtRyttHFNO7WSU6q+2QhiKediBBPiKedJRBPCUYQTwlqOdqozJOz0KTvrUlO2v7oUfr49qej1tRtcwTlAogDxMGnO4I\/4I+UPxBPDbmdO3fS+vXro79s27aNDh48SKOjozQ4OEgtLS1SjBPtOOPcunUrLV++PLWOJCep751cQR0nCyHztFMS4gBxsLMEmacEI4jnGdRYyHiDhP7+flq1alX0PZK\/da5bt46KXv+p6k4SaXaSuvbu3Tv13392\/1F67IWfU9t5M2j3h9ol\/g\/e5vDhw9TeXs9nz+I84JOOEvABPln6kSqzbNmyacUPHDjgYt7wsrnvMKQPnc6fPz+awKOGUcuYhcu7G7Fob9q0iTo6Os4COO4NB5OFTsOEzAqZlU9EAn\/AHyl\/kHkamyQ0QjxtAh3nJH2yUF2\/d0I87d0e4gBxsLMEw7YSjCCeZ1Dj7538fVMftlVCmraMRAK6PrtWrflMGxqGeKJzS3iGlws7ani5wMuFnSXxJSCeGi6cAa5cuXIaUhs3bkxcfykF3dwkQTJhCDNtMWybhX8QB4hDFp4klQF\/ktGDePowqyTbOCfVfWchBT06N8TBpxuCP+CPlD8QTylyJdrFOWnhHd8knjQ0p3VmtEylrheCH4KfD\/fBH\/BHyh+Ip4acvs5T\/ZnXezZ6s3jTSZhp+4rTEPwQ\/KTBj+3AH\/BHyh+I5xnkWDh37NhBw8PD1NraGv01y76zUuBd7Ewn6TNt7+m+nLqvnO1yu6Yqi+CH4OdDaPAH\/JHyB+JpOc\/TtoxECryLnekkTBZC5pmVPxAHiENWrsSVA3+S0YN4Biie9z\/xI\/rIf38q8mqd13hi2M0eFhH8IJ52liSXAH8gnlb+cIa5du1aGhkZmdrlp6rDtphpi8zTSugzBRD8IJ5ZuYLM0w0pZJ4ZzvPUIeX9bh944AE3lD1Lm07CTFuIZ1ZKQTwhnlm5AvF0Qwri6YZXQ0rrTsJM2+kugDhAHHw6JfgD\/kj5A\/GUIleiXZJ4Dlw\/jwaun1tiS6pXFYIfgp8PK8Ef8EfKH4inhlzcOs8itudzdZbuJGwIj8zThT8QB4iDC1\/MsuBPMnoQzzPYhLLOc2jPBA3teT5qdd1n2jIG6NwQB4iDDwLgjxQ9iGdgS1VWf\/kZ+m\/fPBL5m7fl4+356nxBPBH8fPgP\/oA\/Uv5APAMTTyxTwbCtS2eHOEAcXPiCYdvsaEE8Axu2VctUrr70fNr18bdn93STloQ4QBx8qA3+gD9S\/kA8NeSqPmEIy1TOpjmCH4KfNPjhm7kdOfSvZIwgnnb+NLyEcpIunlimctot6NwQT58OCv6AP1L+QDylyJVop5yE01SQebrSDuIAcXDljF4e\/EHm6cOfhtsq8dSXqWCmLTLPLMRE8IN4ZuFJUhnwB+Lpw5+G20I8k12Azg1x8Omg4A\/4I+UPhm2lyJVop5yEZSoYtnWlHcQB4uDKGQzbZkMM4pkNp4aWgngi85QSEOIJ8ZRyh+3AHwzb+vCn4bZKPFtXPxS1BWs8X3EJOjfEwaeDgj\/gj5Q\/yDylyJVox076h2\/\/X+INEvha9btz6FOdl5bYgupWheCH4OfDTvAH\/JHyB+IpRa5EO3bSF7\/6OHV+7omoVqzxROaZlX4QB4hDVq7ElQN\/MGzrw5+G25riidNUIJ5ZSYngB\/HMyhWIpxtSyDzd8GpIaYhnMuwQB4iDT6cEf8AfKX8gnlLkSrRjJ\/X9569PneOJDRKQeWalH8QB4pCVK8g83ZCCeLrh1ZDS7KS3rvmf9MhzJ6PzO1k8cZ1GAOIAcfDpC+AP+CPlD8RTilyJdhBPDNtK6QZxgDhIuYOX03TkIJ4+zCrJlp108g+Go9qwxnM66BAHiINPNwR\/wB8pfyCeUuRKtJv71qvoJ+8dimrsvnI23dN9eYm1V7sqBD8EPx+Ggj\/gj5Q\/EE8pciXa6Zkn1ngi83ShHsQB4uDCF7Ms+JOMHsTTh1kl2c656vfpn6\/uj2rjrJOzT1ynEUDnhjj49AXwB\/yR8gfiKUWuRLv23\/0TennRh6MasUECMk8X6kEcIA4ufEHmmR0tiGd2rBpWEuKZDD3EAeLg0zHBH\/BHyh+IpxS5Eu1m\/+Ed9Ms5vxPViA0SkHm6UA\/iAHFw4Qsyz+xoQTyzY9Wwkm\/44OfpVxe+BRskxHgA4gBx8OmY4A\/4I+UPxFOKXIl2EE8M20rpBnGAOEi5w3bgTzJ6EE8fZpVke+HHvky\/OfdCbJCAzNOZcQh+EE9n0mgG4A\/E04c\/DbdtXf1Q1AbsLnS2K9C5IQ4+HRT8AX+k\/EHmKUWuRDslntggAeLpSjuIA8TBlTN6efAHmacPfxpuC\/FMdgE6N8TBp4OCP+CPlD\/IPKXIlWinxBO7CyHzdKUdxAHi4MoZZJ7ZEIN4ZsPJWurEiRPU29tLY2NjUdnly5fT4OAgtbS0nGV76tQpWrduHe3evXvqt76+PhoYGIitR4kndheCeFqJaBSAeEI8XTkD8cyGGMQzG06ppZQYLl26lFasWEHq321tbbGCyEK7Zs0auummm6ijo8PaAiWe2CAB4mklC8TTCSK8XODlwokwWmGIpxQ5i93OnTtpdHQ0NvscHx+nDRs20JYtW6i1tdXaAiWeJz7ze9aydSuA4Ifg58N58Af8kfIH4ilFzkM89+3bR0NDQzQ8PJxZPOe0zoy25sM1HQEEPwQ\/nz4B\/oA\/Uv5APKXIpdip759dXV3RMK55cVa6fv36qT8vWLAgVUg584R4xgOO4Ifg59OFwR\/wR8ofiKcUuQQ79b2Tf06aMMRZ55EjR6Z+N\/9t3prFc8aLz9BrH9lEe\/fuzbnFYd\/u8OHD1N7eHvZDFNh64JMOLvABPi7db9myZdOKHzhwwMW84WXPmZycnGx4K2IakEU449rN30D7+\/tp06ZNsROIWDyxuxAyTwnnkVkhs5LwRtmAP8noIfP0YZZma5thm1aNbQIRiyd2F4pHMEQC50S5TLcBPukwAR\/gk6kjxRQKkTuVzDxtQ68Ke9dlLWwH8Wyutz9pZ5XYhdjBJc8ptQE+EM86cady4mlukKCcoSYC8UYJvClCd3c3LVmyZGodqNokIW1DBSWe5z5+L7360P+R+hl2QAAIAAEgkDMC+OaZM6C4HRAAAkAACACBqiFQucyzagChPUAACAABIAAETAQgnuAEEAACQAAIAAFHBCCejoChOBAAAkAACAABiCc4AASAABAAAkDAEQGIpyNgKA4EgAAQAAJAAOIJDgABIAAEgAAQcESgNuLJGy9s3bo1gmfbtm3RGtE6XrwDU09PT7QfsG1NrI4Zn6c6MjKS6czUkHF1wUc9p7lZR8jPn9Z2F2zM9dp16HMu+Ohl69K30ril+pBavx9CH6qFeOrHlu3fv9\/pCLMQnJi1jXqQ7+zsjDabUIeOm\/cwz1Dlf+\/YsSPz0W9Z21Slci746O1WJ\/ts3Lgx9uSfKj2jtC0u2Jjba9r2m5a2qUp2LvioF4uBgYHoJb4OfSuLcPJGNyG9ZNVCPDmD4ovJGuIbTl5Bwgxi\/FKxffv2xBNr9HrrEAAl+HAgXLNmDZ08eZKSjs3Ly3+NvI8LNrb9pRv5HEXV7YqPfnhFHfpWEu4qA1+8eDEdOnQoitGhjAo2vXgm7X+blHEV1bmqcF\/z4HCXg8Tr0MEl+PCL2ZVXXklf+cpXErP4Kvjetw0u2JijFr51h2Dvgk9c5jk6OprpJTYELFza+MILL0TFedvV3t5eiKcLeEWXjcs0OeDNnTu3aYfYkjA1M02XDCHrZv1F+7PI+7viw\/jdd999tHr1arrtttuaXjz1UYo07rB4TkxMRK6qyzwDV+6ouMRDlX19fZFo1PkyXyhCwKI2maf+IRriORi96WUVTw6Gd911V9NPGHIJgBz8Pv3pT9OHPvSh6ADxtO\/HIQQCWxtdsFHfgNX3K7Zdu3ZtU\/PHBR81VLl58+ZoiNJlBMjmp1B\/h3hW0HMYtn3FKS5DS8qqLsLJz+uCD5d9+OGHp31Hb+ZPAS7YmMO2dZiNDHz8gj\/E0w+\/wqz1TLPuE4Y2bNhAW7ZsodbW1kgs0iYM1W0WoJmJp+GjL+PRidusQ3Au2Ji41aHPueBTx5cLW3CHeNoQatDvWKpyGniX6fR1GGoz6eiCj25bh8zKBRszENZhWNIFn7hh22Yf1raFfoinDaEG\/o5NEk6Dn7aQW0304MkLSZlVSOuwJHTLik\/dxNOFO1xW3yShLpsAuHCHXyhWrlwZ0agu+KT1R4inJFrBBggAASAABIBAYAg0\/WzbwPyB5gIBIAAEgEAACEA8A3ASmggEgAAQAALVQgDiWS1\/oDVAAAgAASAQAAIQzwCchCYCASAABIBAtRCAeFbLH2gNEAACQAAIBIAAxDMAJ6GJ7gg899xzdMEFF0SbQWS5eKr8Sy+9RJdeemmW4qIyavnPggULnI52C2VTfvV8ZS29KLs+kdNh1LQIQDyb1rX1fTDXRfllrDHzEUAf2zJZoB\/9V1a9oWBTFh6opzwEIJ7lYY2aSkKgiuLp2iYdqlAEAuJZEsFRTSUQgHhWwg1ohCsC+g42bKuGQvfv3z+1cwv\/Xe2IZO6YpIYWZ82aFZ0jODY2FjVB7U2rHxml3z9tGFivQx+6VKeMqGfcuHFj7HF4SeWUeHZ2dtLtt98e3cYcGtV3rDHrUQd2X3PNNZG9wur48ePU09NDR44ciUxuueUW2rVrF23atIk6OjqivyU9U5y\/TPHkf\/\/0pz+N\/sdHb+n4xtnHnQNqOxs0lBcLV36jfPURgHhW30dooYFA3F6yeuA2s7ykjbj5toODg9Gev\/pBvEqYu7q6pkQubZN81R51Pz7uzTyNxpZ5muX1\/U9Z4FnkFi9ePHVgst4eFsH+\/v5poqffT70gzJkzZ8refEb172PHjkVHh6lj1lik1VmTtv2O48STz\/NULwtxuOquhXiiq4eEAMQzJG+hrRECtiBsEyq+hx6oTfGMO00lbfP3uOzHLJ\/WJtvG8uZG4tx+W8al\/67E03wZGB0dnRJTvqcujvxv\/QQeRb20odk48eSsll9Q+IVC1cHlhoeHz5rMBfFEBw8JAYhnSN5CW6cQ0Ic4zdmrSUJlDm0uX748NvM0h0912OOGXJPq0zfaTxNP24SlOKGM+5s5lG0OTXMGyYcv8xUngvo9OZtVG5ebtEs6di1OPNlWZa420Yd4ooOHhADEMyRvoa1nIaALhv7dU89ulBia3yFV5mVmnmxrZkxp0CcJY9pQsn4\/X\/Hke6lvl0rc4zJPF\/F8\/PHHaceOHU5LaiCe6KB1QgDiWSdvN\/Gz6gKkMiseGuThwnXr1tHSpUunTdLRBdIUT9dDwMsYtjW\/aep1stClDcGqYVtdPOOyPH3YljNP1zMmixi2tb3I2Iavm5jyeLQGIwDxbLADUL07AnHfPPXsT59AEzfxRWWiatiWW6ALrLo\/D3GqIce4746q5XlNGNIzPf076KJFi86aEGSKp26r2srt48k\/ceKZdcIQ30N9s7R9a\/adMKSG1dUMafUc+kQpky0QT\/f+A4t8EIB45oMj7lIyAvrBw1y1PiSrLzPhYczrrrtu2nIUFs0bbriBbr311khceFmGKagqG1VLWLgO20Hgacs6sk5iWr9+\/RSScUOwagmJKRpm3Zs3b46+a\/IkIfX8eubJlZgYmktVzOU6bJO0zEZl+\/z\/6oUjbqmKbh8nfPr3ZvbTwoUL6cknn4wE3HzJUc9gZuUlUxHV1RQBiGdNHY\/HBgJxWVzcDNusSGX55pn1XlnLIfPMihTK5Y0AxDNvRHE\/IBAAAuZ3XZVl6us6XR8D4umKGMqHjADEM2Tvoe1AwAMBc9elpCUoWaswN2q\/\/\/77I1N9qUrWe2Uph43hs6CEMkUh8P8BPpN+m02joakAAAAASUVORK5CYII=","height":279,"width":463}}
%---
