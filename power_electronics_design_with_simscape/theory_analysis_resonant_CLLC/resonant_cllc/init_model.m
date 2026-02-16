clear;
[model, options] = init_environment('single_phase_cllc');
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
nominal_power = 1600e3;
application_voltage = 690;
vp_xi_pu = 1;
vn_xi_pu = 0;
vn_eta_pu = 0;
grid_emu_data = grid_emulator(nominal_power, application_voltage, vp_xi_pu, vn_xi_pu, vn_eta_pu);
%%
%[text] ## Global Hardware Settings
afe_pwr_nom = 250e3;
inv_pwr_nom = 250e3;
dab_pwr_nom = 250e3;
cllc_pwr_nom = 250e3;
fres_dab = fPWM_DAB/5;
fres_cllc = fPWM_CLLC*1.25;

hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, fPWM_AFE); %[output:2951b0eb]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, fPWM_INV); %[output:8b44e43c]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, fPWM_DAB, fres_dab); %[output:2a3cccd4]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, fPWM_CLLC, fres_cllc); %[output:18b0f7c7]
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
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:8b44e43c]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:2a3cccd4]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:18b0f7c7]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.880000e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:1144ae32]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:1d963221]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:7ef6d9d7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQ+QVUeZ7xdFZbLBhYnRSCYwJIJGyiJRo+MkClleZLcUdh9RYdBXSE0wjySV1DoUM0x4RViT+UM5KZNNgiQZ2dnSmWCUfcvUqmyWCPXihDWahH0b40IyDDhiDI8Bk5SDiubVd0jfnLnce6fvuX3O6fOb36mimLm3T5\/+fr\/v6\/7N192nz3nttddeE15EgAgQASJABIgAEcgQAudQwGSILTaVCBABIkAEiAARCBCggKEjEAEiQASIABEgAplDgAImc5SxwUSACBABIkAEiAAFDH2ACBABIkAEiAARyBwCFDCZo4wNJgJEgAgQASJABChg6ANEgAgQASJABIhA5hCggMkcZWwwESACRIAIEAEiQAFDHyACRIAIEAEiQAQyhwAFTOYoY4OJABEgAkSACBABChj6ABEgAkSACBABIpA5BChgMkcZG0wEiAARIAJEgAhQwNAHiAARIAJEgAgQgcwhQAGTOcrYYCLgDoGDBw\/KqlWr5OjRo7lKFy9eLB0dHVJVVTXug0ZHR6WlpUUaGhqkrq5u3PL79u2TFStWjCl3ww03SHNzs5Rb17gPYwEiQASgEaCAgaaXxhGB0giogFm3bp1s3rxZZs+eXbaIKFd0qIDp7OyU7u5uqa6uzj1v+vTpgYjhRQSIABGwRYACxhYpliMCgAiMJ2DCGROTKVEYVIRs3bpV5s+fH6Ci32kGZvv27bJ+\/frcZ\/miJF\/AaEFtQ1tbm9xxxx2BkNJszrx584LMTn9\/f1CXyQrpz+Zz\/ezll1+W1tZWeeqpp2RgYECOHDkiy5cvl5kzZ47J9PT29sqcOXOkqalJPvGJT8hXvvIVUdH01a9+NbBl\/\/790t7eLsuWLQNkmSYRAUwEKGAweaVVRMAKgUJTSGYgD4ubmpqaQDjU19cH4sBkUY4fPx5MQakQ0Kuvry+YfjJCI39qqZCAGRkZCYTFl7\/8ZXnooYcCAXPxxRcHdVx00UVivg8LFX2Gio61a9fKtm3bAgHz8MMP5zI7+hwzpaWiamhoSFavXi2NjY3B5yqs1AYtp9mgRx99NBBAtlNnVuCyEBEgArEiQAETK7ysnAj4jUCxDIwRKkaQ6HoYIwSMRfnrVg4fPpzLvpgy4ayNfmYrYFRkmOkpzcJotmTLli2BwNG2aaakmLAxa3fys0dGwGi7TbZIhY3+rmXDtvrNGltHBIiAIkABQz8gAhMYgXwBo1AYoaLTQ+UKGCMIikFqO4Wk9+ti3\/DUj8nQjCdgTPZH\/9eMys6dO8dkYChgJrDD03QoBChgoOikMUSgPARKZWA++MEP5hb42k4hmSmdcPnwupJSi3hvueWW3I4mtcKIp\/ypIjPVU+xzI2DCa2k0g8MMTHm+wdJEwHcEKGB8Z4jtIwIxIjDeNuo4FvHabKPWBbe6XkVFipZ\/5ZVXzlrcW2gRr1nDYhYTq3DRep555plAjN18883BlBGnkGJ0KlZNBBJCgAImIaD5GCJABNwiUGg6yu0TWBsRIAI+I0AB4zM7bBsRIAJjEAhv09YvdI2MzQv0CCMRIAJ4CFDA4HFKi4gAESACRIAIwCNAAQNPMQ0kAkSACBABIoCHAAUMHqe0iAgQASJABIgAPAIUMPAU00AiQASIABEgAngIUMDgcUqLiAARIAJEgAjAI0ABA08xDSQCRIAIEAEigIcABQwep7SICBABIkAEiAA8AhQw8BTTQCJABIgAESACeAhQwOBxSouIABEgAkSACMAjQAEDTzENJAJEgAgQASKAhwAFDB6ntIgIEAEiQASIADwCFDDwFNNAIkAEiAARIAJ4CFDA4HFKi4gAESACRIAIwCNAAQNPMQ0kAkSACBABIoCHAAUMHqe0iAgQASJABIgAPAIUMPAU00AiQASIABEgAngIUMDgcUqLiAARIAJEgAjAI0ABA08xDSQCRIAIEAEigIcABQwep7SICBABIkAEiAA8AhQw8BTTQCJABIgAEcgqAqePDckrP+yRaZ\/bmFUTYms3BYwFtJdccolFKRYhAkSACBABIuAGgQvfelr+x\/QTsuj8V+XF30+SL\/zfiyuqeHBwsKL7fbyZAsaCFRUwlZBvc3+pMoW+G++z8Pc2P1vAcFYRG7v0pmLlbD\/PLzeePbbtKmaz7f2Vcpa0XaW4CGNBu86gwRg7M+AVijfbGGGMFUagFH5rrnyn\/K9PzZVTz+6RSRfUypQFX5QPtfSMGYPS6DuijBFx30MBY4FwpcFq8YhUitCuVGCP\/FDyFRm61G4kZ6lBH+nBafGl00Sjz+6RV\/b05ISLThmpeHFxpWWXi7aXqoMCxgJhVPJplwX5HhUhXx6RYdkUcmYJlCfFkubLrG95Zc8\/BAhUzV0gUxaslMlzFzhFJGm7nDa+RGUUMBZIo5J\/6NAhmTVrlgUC2SpCu8iXLwjQF31hwq4dSfFlhMuJR24PpolUuGjGRX+O40IdwyhgLLwFlfykgtUCYqdFaJdTOGOvDJUvBQ7VNtoVLSzyhYtOEU25ZmVswsW0EnUMo4Cx8ENU8tkJWZDvURHy5REZlk0hZ5ZAeVIsLr5UuJz49ibRqSKzMDfJbdGoYxgFjEXgoJIfV7BaQBprEdoVK7zOK0flixkY564Se4WufVGFy0v3rRqzoyhJ4cIMTOwu4\/8DKGD85yjcQtedkC\/W0y5fmLBvBzmzx8qHki74intHURScUMcwZmBe94Z9+\/bJihUrgt\/a29tl2bJlOT9BJd9FsEYJprjvoV1xI+y2flS+mIFx6ydJ1FaJLxrholNF+nOwviWGHUVRcEAdwyhgRGRkZESampqktbU18I22tjbp6uqS6urq4HdU8isJ1ihBlNQ9tCsppN08B5Gvzl1DATgnT5yQl1+bLDOqq3JgNS+KZ6eJGzbsakHkLKrgTGortB0zhUuhjmEUMCKi2ZfOzk7p7u6WqqoqaWlpkYaGBqmrq6OAqSRqUrqXnWtKwEd8bJp89T35YtDqIyOnxrT+yMho7vdfhL47cuJMufzy+abPqJ4cfHT69Gm55ILzztxT4F4t13DluyVroiZNziK6mdVt5diV1o4iK0PyClHAREEtI\/eogOnr65OOjo6gxSpg6uvrc9NI1V\/+YUYsYTOJABEoF4Hpb5901i3vnvLGZ+Hvp7\/++bvfPkkWX3ZGmJS6hoeHpaam5qwiR18+LT\/95Sn51cunpf\/nr4r+fsNHpsqXPjp1vCq9+L6YXV40roJGWNl1Ylhee\/RukZ98V2RajciHr5Nzrr21gqfGc+vChQvHVFzJcTjxtLDyWpmBeT0DU0rAoKrXcv7aqNzVkquBdiWHtYsnofKl2NjaplNOfU\/+KsjsNC+a5X1GxtYuF\/6RZB2l7MrfCj3j\/kNJNq2iZ6GOYRQwnEKqKDB8vHkidq4+8mDbJlS+yhEwYaw046tTSztvvCL438cLlbN8u3zcURTFHyhgoqCWkXu4iDcjRFk2c6J0rpZweF8Mla+oAkazMDf3PSePv3BSRu66xkv+UDkzdvm8oyiKQ1DAREEtQ\/eEt1H39vbmFvCqCajko3dCGXI\/q6aSLyuYvCpUCWc6rdS565A0XHmh3NdwGYxdXhmS15hDT\/0fmfr8Y8Ebc\/WK63DFpDFAHcM4hWThSajkV9K5WsCWWhHalRr0kR6MylfUDEwYxMefPylL7n9arr50quy86YpI+MZxExpnWdpRFIVP1DGMAsbCG1DJR+uEDJW0y8KpPSqCypcLAaN1+ChiUDjLFy6nL\/9rueRLX\/MoOtw0BXUMo4Cx8A9U8lE6oXwKaZeFU3tUBJUvVwJG69F1MZff8USwqPeZDR9Lnb2sc1bscMWs21XMMVDHsMwKmO3bt8v69evH8JV\/BICrKEclHzVYaZcrz0+mHlS+XAoYI2J0ca++EC9tEZNVznRtyyt7enKHK+rBivrKf\/TsLeoYljkBYxbbFhIrRtTkL8KttBtGJT+rndB4fNKu8RDy63tUvlwLGJ9ETJY4y98KPXnuAqn+7EbR\/ydK9hZ1DMuUgNHtzv39\/bJy5cqSPXBPT8+4ZcrpwlHJz1InVA5ftKsctNIvi8pXHAImX8Tcu\/wyufo9yb+9NwucRTmjKAt2RYlY1DEsUwImCnEu7kElHzVYaZcLr0+uDlS+4hIwPogYnzmrZEeRz3ZVEpGoY1jmBMzBgwdl1apVcvToUdFpJL10Lcz06dNl27ZtMnv27Ep4LngvKvmowUq7nIdArBWi8hWngDGELLnv6WBNTNKZGB85KyRcdI1LOZePdpXT\/mJlUcewTAmY0dHR3EnRc+bMkcbGRlm+fHlw6KKufxkYGAgOZNQTpV1eqOSjBivtcun98deFylcSAkafYURMkgt7feLMhXAxXu6TXS4jD3UMy5SA0TUwmzZtko0bN0p1dbV0dnbK\/Pnzg7fm5n9H8sdHADVYadf43PtUApWvpARMWMQklYnxgbP8rdD5O4qi+LgPdkVp93j3UMCMh1AC31PAuAUZNVhpl1s\/ibs2VL6SFDBJi5g0OTNboU+\/NOT8Vf9p2hVnnFHAxImuZd0UMJZAWRZDDVbaZekAnhRD5StpAZOkiEmas\/BWaCNcNOMy6YJap16ctF1OG1+iMgqYpJAu8RwVMLruZf\/+\/QVLzZs3T7q7u4PpJZcXKvmowUq7XHp\/\/HWh8pWGgElKxCTFWf5WaH3p3JRrVjoXLsbLk7Ir\/qga+wTUMSxTa2CSJt08D5V81GClXWlFSrTnovKVloBJQsTEzZnLhbnleGXcdpXTFpdlUcewTAkYZmBcurQIarDSLrd+EndtqHylKWDiFjFxcnbi25vkxCO3B1mWGfcfitv9xtQfp12JGpL3MAqYNNEv8GzdNj00NCTNzc3Bt\/q7Xrql2vWFSj5qsNIu1xEQb32ofKUtYIyIefyFkzJy1zVOSXTNWRw7iqIY7NquKG2I4x7UMSxTGRhDbKEt09xGXb7bowYr7SrfF9K8A5UvHwRMWMTsvPEKZ8cOuOIszh1FUXzalV1Rnh3nPRQwcaJbZt3mhXb19fW5jIu+E0bfzjvei+zCp1iHF\/2aQyK1KfkHRaKSjxqstKvMgEq5OCpfvgiYOERMJZyZHUU6VaRX1dwFEseOoihuXYldUZ6X1D2oY1gmMzBKev56GJsdSHoMQVtbm3R1deVehKeiZ926dbJhwwZpbW0N\/ClcRn9HJR81WGlXUt2im+eg8uWTgHEtYqJwlvSOoijeGcWuKM9J+h7UMSyzAsaFA2jWpa+vT5YuXSpf+9rXgi3YegxBS0uLNDQ0BG\/4pYBxgXSydaB2QrQrWT9y8TTfONNjB1ysiSnXrjQX5pbDY7l2lVN3mmUpYNJE\/\/Vna9alv79fVq5cWbI1PT0945bRCnTaqba2VmbOnBkIGZ1+0ksFTHh6SskPX7t37\/YAjcqbMDw8LDU1NZVX5FkNtMszQsZpDipfaraPtn1px4vy01+ekgeWXigfumhyJGexsuvEsLz26N0iP\/muyLQaOefaW0U+fF2k5yV1k5VdSTWmwucsXLhwTA2Dg4MV1ujf7ZnLwJi1KvnrVBRas76lt7c3lz0pBnl4F5PJxJQSMIjko\/61Qbv862hKtQiVL7XZV9tu6ntO+p58UaIu7C1ll04VvXTfKjn17J5gK\/Q7b9omk+cuyIRT+spXpeAxA1Mpgo7vDy\/GNVXnixpd87Jq1apgce\/ixYtzC3xN5sVsuVYBo59xCskxSSlVh9oJ0a6UHKqCx\/rMWeeuIencdSiSiMm3q9Cr\/qcsWJkZ4WIo9pmvCtwQdh1n5jIwlZCo94ZPsDZ16dRUU1MTF\/FWCq4n96N2QrTLEwcroxm+c6ZZGM3G3NdwmTRceaG1ZcauLCzMtTbK44xZOTYUKssMTKUIenB\/eKu0aY7JzOj5SitWrAg+zp+CQiXf9841qsvQrqjIpXMfKl8+TyGFmY4iYpSzqU\/+Y2pvzI3LU1F9EXUMm3AZmCiOj0o+arDSrihent49qHxlRcBoOx9\/\/qQsuf9paV40S5oXFT\/hOf+NuXq4or7DBeVC9UXUMYwCxiLyUMlHDVbaZeHUHhVB5StLAiYsYnQqSaeUwpd5Y65ZmPvHa26SWZ9d65EXuWkKqi+ijmGZFTDhF9k99NBD8thjjwVbp2fPnu3Gk0O1oJKPGqy0y3kIxFohKl9ZEzD5Iubu\/1Ylo8\/ukVf29Mjpl4aCN+aahbmonKHahTqGZVLAhI8S0AMd58+fH3Sw5l0u+jI6lxcq+ajBSrtcen\/8daHylUUBo23e88Qz8oPur8mXftMTbIPWaaIp16wMfjYXKmeodqGOYZkUMOGDGx988MFAwMyZM0c2bdokGzduDI4JcHmhko8arLTLpffHXxcqX1kTMPnrWy6f\/A25+tKpsvOmK85yAlTOUO1CHcMyKWAKZWD27t1rdZhjlO4YlXzUYKVdUbw8vXtQ+cqCgAm\/v8Wsb9FFuZp1OTJySi6\/44mCIgaVM1S7UMewTAoY7RiiHOYYtYtGJR81WGlXVE9P5z5UvnwWMPnvbwmvbwl7QTERg8oZql2oY1hmBUySXS0q+ajBSruSjI7Kn4XKl68C5th9q0R3Femalhn3HxqXQBUxN\/c9J0dOnJJnNnwsKI\/KGapdqGNYpgRMftYlP\/LmzZsXHAfANTDj9knshOwg8qoUaueKapcvA32paSJbB88XMaicodpFAWPr6QmVCx\/GqI\/U3\/Uy5xu5bAYq+ajBSrtcen\/8daHylbaAMdNEJx65PbebqGru\/IrOJ1py39NBJmbDgqnymavHvismfk+J\/wmovog6hmUqA2PcN7wLyWRbCn3myt1RyUcNVtrlyvOTqQeVrzQETKFsi800UTlMq4gZPPaqfP0LH5Cr3zO1nFu9L4vqi6hjWCYFTHgXksm46CGNeup0R0eH8D0wdv0EarDSLjv+fSmFyleSAkbXtJx6dm9ubYvuIqo021LKP67tekJ++stTMnLXNb64kZN2oPoiBYwT93BXCXchVY4larDSrsp9I8kaUPmKW8Dotmd9S65ZkKs7iSbPnR9sgY77Us5u\/d5JefyFk7LzxitgMjGovkgBE3dEeFw\/KvmowUq7PA6mAk1D5cu1gNHpIX2lv4oWfcW\/\/m7elJv0gYqGs5v6nhM9zRpFxKD6IuoYlskppGK7kbgLqbyBCzVYaVd5fpB2aVS+XAgYs6bFTA9pnbbbn+PkNcxZ564h6dx1KDgAUg+CzPKF6osUMJ57pe5CmjlzptTV1TlvKSr5qMFKu5yHQKwVovIVRcCYLMvos3tl9Gd7xLwdd9I7a6Xq\/QvOOpMoVmJKVJ7PmWZhNBuTdRGD6ouoY1gmMzCF4oq7kMrvylCDlXaV7wtp3oHKl42ACWdY\/nBsKBAsJsuS5JqWcvkvxNnjz5+UJfc\/Lc2LZknzojcOfiy37jTLo\/oiBUyaXmXx7H379onuRLJ9kd3Bgwdl3bp1snnzZpk9e7bo\/StWrAie1N7ePuZ9MqjkowYr7bIIGI+KoPJVSMCY3UJhsZIvWN5yQW1F72pJgtpinBkRo1NJmo3J2oXqi6hjWCYzMMXWwOQLj2LBY7Zh\/\/SnP5Vt27bJ+eefL01NTdLa2hrc0tbWJl1dXbk3+qKSjxqstCtbwwYaXypSTr90OFhk+8ovfi6TXn4x+NlcuobF5+yKjfeU4syImGInWdvUn1YZNF80OKKOYZkUMJU6t66XOXbsmKiAUdFy\/PjxXPZG3yHT0tIiDQ0NufU0qOSjBivtqjRCkr3fR75UhJhLxYheRoRo9iT4\/aWhMcIkLFCCrMo7a+VU1flywUc\/HXyVhcyKLfPjcZZVETOeXbb4+FYOdQzLpICp5E28OnXU09Mja9askQ0bNuQETF9fX\/ASPL1UwNTX1+emkQb++i1j\/PHCd2V7pb0x5o9\/PC1vfvMk32Kt4vbQroohTLSCMXydGE702eM+bFrNG0WqX\/952kVnPptWI+fo9x++rmg1w8PDUlMTqmPcB2ajgI1dR18+LYt7huVDF02WB5Zmo8+0sSsbDIksXLhwTFMHBwez0nTrdmZKwJipn\/7+\/oIGLl68uOSbePX+O++8U1auXDlm2kgzMKUEDKp6Rf1rg3ZZx78XBVH5UnBRbbO1q9BJ1l44XZFG2Nrlsw2F2oY6hmVKwBhibHccabZl1apVwREDKm6uv\/76IPOiv5tr+vTp8rd\/+7fyzW9+M1gAzCmkrIXm2e1F7YRoV\/Z8k5yJZEnEoPJFAeNB32GEyy233CJr166V\/fv3j2lVOS+y07rMwl0u4vWAXIdNQO2EaJdDJ0moKnL2BtDmJOt7l1\/m7dEDqHxRwCQU8Ek9Jixg8rdR9\/b2jnkhHir5qMFKu5KKIjfPQeWLU0hn+4fvIgbVF1HHsExOIWlY6E6i9evXR87AlNP1opKPGqy0qxzvTr8sKl8UMIV9S0WMr4dAovoi6hiWSQFj3gPT3Nwcy9EB+WGHSj5qsNKu9EVJOS1A5YsCprgX+HoIJKovoo5hmRUwmzZtko0bN+ZeNldOh1luWVTyUYOVdpXr4emWR+WLAqa0X5lDIH06yRrVF1HHsEwKGDOFpP8vW7Ys9t4XlXzUYKVdsYeE0weg8kUBM76b+HYIJKovoo5hmRQwxY4SKGcX0vih9UYJVPJRg5V2lePd6ZdF5YsCxs63jIjx4RBIVF9EHcMyKWDswsJdKVTyUYOVdrnz\/SRqQuWLAsbee3w5BBLVF1HHsEwKmGIZGA0XfTGdHtCoW6NdXajkowYr7XLl+cnUg8oXBUx5\/uPD+Umovog6hmVSwJg1MENDQ6I7kcJrYmbOnBkcC3D33XeXFz0lSqOSjxqstMuZ6ydSESpfFDDlu0\/aIgbVF1HHsEwKmFKHOepbeu+55x4KGIu+AzVYaZcF+R4VQeWLAiaak+nRA5ff8YRcfelU2XnTFdEqiXgXqi9SwER0iDhuM4c66nRROAMzMDAg69atC841Mp+7eD4q+ajBSrtceH1ydaDyRQET3YfSOj8J1RdRx7BMZmA0LPLXwegOpHvvvVc2b94s9fX1TrdXo5KPGqy0K\/rAkcadqHxRwFTmTWERo++KmVE9ubIKLe5G9UXUMSyzAsbCF50VQSUfNVhplzPXT6QiVL4oYNy4T5LnJ6H6IuoYllkB09nZKVu3bh0TIXwPTHkdBmqw0q7y\/CDt0qh8UcC486ykRAyqL1LAuPPFimsKnyT91FNPie48Onz4cFBvHG\/mRSUfNVhpV8UhlmgFqHxRwLh1I3MI5Mhd17itOFQbqi+ijmGZzMCEdyEdOHBA9u7dK6tXr5a4zkdCJR81WGlXbP17LBWj8kUB495d4j4EEtUXUcewTAoY3YWkW6VVtBw\/flxWrVolR48eFU4hlddhoAYr7SrPD9IujcoXBUw8nmUOgbyv4TJpuPJCpw9B9UUKGKduUnll+\/fvl3PPPTd44+6+fftk7dq1zt\/Aa1qJSj5qsNKuyuMryRpQ+aKAic+L4joEEtUXUcewTGZgKgkL8w6Z\/v7+MccOqAhasWJFUHV7e\/uYtTSo5KMGK+2qJEKSvxeVLwqYeH0pjvOTUH0RdQybcAJGdy\/V1tYGAkVFi1k\/09TUJK2trUHEtbW1SVdXl1RXVwe\/o5KPGqy0K96Bw3XtqHxRwLj2lLPrcy1iUH0RdQzLlIApdYijuvZ4a2AKHUGg96mQUWHT3d0tVVVV0tLSIg0NDVJXV0cBE38f5PwJqJ0Q7XLuKrFXSM5ih1hcnp+EyhcFTPx+OO4T8gVMb29vTmSMe\/Prb+\/VnUp6haeQdCGwHgDZ0dERfKcCJvw2XyU\/fO3evdvmcd6XGR4elpqaGu\/bWW4DaVe5iKVbHpUvRRXVNt\/sOvryaVncMywfumiyPLA0+sJe3+yqJDIXLlw45vbBwcFKqvPy3kxlYMIIFjpKQDMoZtqnENoHDx4Mdix99atfDYTP9u3bRc9PWrp0qezYsaOkgEEkH\/WvDdrlZV9TtFGofKnBqLb5aJeLQyB9tMtFNDMD4wLFGOsITwMZEWMEi26xXrx4cXDQ44YNG4K1Lmb3kk4drVmzRrZs2cIppBj5SbJq1E6IdiXpRW6eRc7c4GhbS6WHQKLyRQFj60EJlQuLE32kChSdAtI1LKWu8CJek4EJCxu9l4t4EyIxpsegdkK0KyaHibFachYjuEWqDouYe5dfJle\/Z6p1I1D5ooCxdoH4CoanjcZbsFusFcXqCG+jzl9bg0o+arDSrvhiMI6aUfniFFIc3mJfZ5Tzk1B9EXUMy9QamEp3Idm7\/tiSqOSjBivtiurp6dyHyhcFTDr+FH6qOT9p541XWGViUH0RdQzLlIBJKxxQyUcNVtqVVqREey4qXxQw0fzB9V3lnJ+E6ouoYxgFjEW0oJKPGqy0y8KpPSqCyhcFjD9OZitiUH0RdQyjgLGIMVTyUYOVdlk4tUdFUPmigPHIyUTE5hBIVF9EHcMoYCxiDJV81GClXRZO7VERVL4oYDxystebYg6BbF40S5oX1Z7VQFRfRB3DKGAsYgyVfNRgpV0WTu1REVS+KGA8crJQU0qdn4Tqi6hjGAWMRYyhko8arLTLwqk9KoLKFwWMR06W15Ri5yeh+iLqGEYBYxFjqOSjBivtsnBqj4qg8kUB45GTFWhKIRGD6ouoYxgFjEWMoZKPGqy0y8KpPSqCyhcFjEdOVqQp+ecnofoi6hhGAWMRY6jkowYr7bJwao+KoPJFAeORk5VoSvjogX\/6\/IUya9asbDS8jFaijmEUMBZOgEo+6sBBuyyc2qMiqHxRwHjkZOM0xYiYwWOvyn\/e\/vHsNNyypahjGAWMhQOgko86cNAuC6f2qAgqXxQwHjmZZVOu7XpCjo2KlHsIpGX1qRVDHcMoYCxcCpV81IGDdlk4tUdFUPmigPHIySybor546\/dOypETp6BEDOoYRgFj4dio5KMOHLTAcRnvAAAgAElEQVTLwqk9KoLKFwWMR05m2RTji+UeAmlZfWrFUMcwChgLl0IlH3XgoF0WTu1REVS+KGA8cjLLpoR90fb8JMuqUy2GOoZRwFi4FSr5qAMH7bJwao+KoPJFAeORk1k2Jd8Xbc5Psqw61WKoYxgFjIVboZKPOnDQLgun9qgIKl8UMB45mWVTCvmiOT\/pvobLpOHKCy1r8qsY6hg24QTMwYMHZdWqVXL06FGZN2+edHd3S3V1tezbt09WrFgReF17e7ssW7Ys54Go5NMuvzqZ8VpDvsZDyL\/vyZl\/nJRqUTG+Sp2flAULUf1wQgmY0dFRaWlpkfr6+kCgdHZ2Br63evVqaWpqktbW1uD3trY26erqCoSNXqjk064sdD1vtJF8ZYsv9h1YfGVZxKD2HRNKwGg4qWipra3NCRj9eebMmcHnmo2pqqoKRE5DQ4PU1dU5ETA2zlOqTKHvxvss\/L3Nz1G6Ghu7SnXixe7P\/7zU74Vss21XMZtt76+Us6Ttsh1QadcZz2CMDZ6Fg8HENkbQYqzYIZBhO22wyWKMRRkj4r5nwgkYI2K2bt0qvb29gUjR6aO+vj7p6OgI8A5naUxHFjcRrJ8IEAEiQAT8R+BP575DXv5kp0z9343+NzbUwsHBM4IU6ZpQAmZkZEQaGxulubk5EC5mCmn+\/PklBQwS4bSFCBABIkAEKkNAjx6YUT25skp4d8UIQAuY8ILdxYsXy\/XXXy933XVXbn2LZl5UxKxZs0a2bNlSdAqpYpRZAREgAkSACBABIuAUAWgBk49UfgZm+\/btMjAwIOvWrZMNGzYUXcTrFHFWRgSIABEgAkSACFSMwIQSMIqWzTZqszamYnRZAREgAkSACBABIhALAhNOwMSCIislAkSACBABIkAEEkWAAiZRuPkwIkAEiAARIAJEwAUCFDAuUGQdRIAIEAEiQASIQKIIUMAkCjcfRgSIABEgAkSACLhAgALGBYqsgwgQASJABIgAEUgUAQqYROHmw4gAESACRIAIEAEXCFDAuECRdRABIkAEiAARIAKJIkABkyjcfBgRIAJEgAgQASLgAgEKGBcosg4iQASIABEgAkQgUQQoYBKFmw8jAkSACBABIkAEXCBAAeMCRdZBBIgAESACRIAIJIoABUyicPNhRIAIEAEiQASIgAsEKGBcoMg6iAARIAJEgAgQgUQRoIBJFG4+jAgQASJABIgAEXCBAAWMCxRZBxEgAkSACBABIpAoAhQwicLNhxEBIkAEiAARIAIuEKCAcYEi6yACRIAIEAEiQAQSRYACJlG4+TAiQASIABEgAkTABQIUMC5QZB1EgAgQASJABIhAoghQwCQKNx9GBIgAESACCAic+PYmp2acPjbktL78yi64aVus9adROQWMBeqXXHKJRSkWIQJEgAgQAZcILDr\/VXnX204HVb7rrX\/IVX3hW898Zi5TRn\/P\/85le8J1vfj7SU6r\/vXv3NaX37j\/\/oNRp+31oTIKGAsWVMAMDg5alCxcxOb+UmUKfTfeZ+HvbX6OYpyNXVpvsXK2n+eXG88e23YVs9n2\/ko5S9quUlyEsaBdZ9BgjJ3p8wrFm22MGL96Zc8\/yOmXDovJMjze\/7B8tK5OTr80lPusUDyqSKi5qCb31aR31gY\/v+WCWvnOd78jn7nuM8Hvky448\/mHWnrG9NWMsTNjV7l8RRkP0riHAsYCdVTyaZcF+R4VIV8ekWHZFGTODvz7YwEKo8\/uCf4\/9exe+cOxoYKixAiMsAAxwmPa5zZaohl\/MWS+KvkjPH7koz2BAsYCNzq1BUgeFSFfHpFh0RRUvvKzFxZQeF1EsyhGpAw9\/fiYqRoVKCpONDOiV1bXW6D6IqpdFDAWXQYq+YcOHZJZs2ZZIJCtIrSLfPmCQBZ9Uad5NKtixMqp1zMsJotSNXeBvPrOuTLrs2t9gdlZO7LIl43xqGMYBYwF+6jkowYr7bJwao+KoPKlEPtum1mTooLllT09EhYrJqMyee78ILMyee6CnNf4bldU90a1C3UMo4Cx8HRU8lGDlXZZOLVHRVD58lXAhDMsOi2kl2ZXNLNiO\/WDyhmqXahjGAWMRUeOSj5qsNIuC6f2qAgqXz4JGCNaTJbFrFmZfvsPI3kCKmeodqGOYRQwFuGLSj5qsNIuC6f2qAgqX2kLGBUtr\/ywR0Z\/putZ9pSdZSnlIqicodqFOoZRwFh05KjkowYr7bJwao+KoPKVpoA5evs1Y0TLlAUrx6xhqZR+VM5Q7UIdwyhgLCIZlXzUYKVdFk7tURFUvpIUMCbbcuKR23OZFteiJewyqJyh2oU6hlHAWHTkqOSjBivtsnBqj4qg8pWEgNGpIV3XootxdV3LlAVflCReDIfKGapdqGMYBYxFR45KPmqw0i4Lp\/aoCCpfcQoYFSwqXPRV\/OXsHnJFOypnqHahjmETTsDs27dPVqxYkYvj9vZ2WbZsmYQ\/N5+ZQqjkowYr7XI1TCVTDypfrgWMmSYyW5812zLlmpW5c4CSYevMU1A5Q7ULdQybcAJm+\/btMjQ0JM3Nzbl4HxkZkaamJmltbQ0+a2trk66uLqmurg5+RyUfNVhpV5JDWeXPQuXL1UCfv74lqWmiUsyicoZqF+oYNuEETGdnp9TW1gZZF3Np9kU\/7+7ulqqqKmlpaZGGhgapq6ujgKl8fEq8BtROiHYl7koVP7ASznwULgaQSuyqGNQYK0C1iwImRqdJqmrNtDQ2Nsr+\/fuDR86bNy8QLQcOHJC+vj7p6OgIPlcBU19fnxM5Sn742r17d1JNjvU5w8PDUlPzxlH1sT4swcppV4JgO3gUKl8KTSTbTgzLaz\/5rsijd4tMqxH58HVyzrW3OkDaXRWR7HL3+NhqQrJr4cKFY3DiadSxuU06Fet00sDAgCxdulR27NhRUsAgko\/61wbtSieeoj4VlS\/FoxzbfM645HNbjl1R\/SKN+1DtYgYmDW+K+Zlm6mjNmjWyZcsWTiHFjHdS1aN2QrQrKQ9y9xwbzrIkXAwyNna5QzG5mlDtooBJzodie5JOIW3atEk2btwYLNDVdS96rV69mot4Y0M9+YpROyHalbwvVfrE8Tg78e1NYl4+N+P+Q5U+LrH7x7MrsYY4fhCqXRQwjh0lrerC26XNGhgVM+HPe3t7cwt4tZ2o5KMGK+1KK7qiPReVr2JTSFnMuHAKKZpv+3IX6hg24XYhRXEoVPJRBw7aFcXL07sHla9CAiaccfFhO3RU1lE5Q7ULdQyjgLGIYFTyUYOVdlk4tUdFUPkyAubi884JToU2U0VZFi7GbVA5Q7ULdQyjgLHoyFHJRw1W2mXh1B4VQeVLp4qO\/NPXgu3QSZ5TlAS1qJyh2oU6hlHAWEQ7KvmowUq7LJzaoyJofKlw0ami4JX\/02rkghVfCQ5ZRLrQOEPPLKGOYRQwFr0KKvnshCzI96gI+fKIjLymqGgZff1kaHPA4pQFK+VX586UWbNm+dvwiC2jL0YELqXbUMcwChgLh0Iln52QBfkeFSFfHpHxelPG21FEzvzjrFSLUPlCHcMoYCziC5V81GClXRZO7VGRrPFlRMvoz\/bIqWf3BOtbpn1uY8FpoqzZZusWtMsWKT\/KoY5h3gsYfd3\/+vXrx3hBe3v7mMMY43YRVPLZCcXtOW7rJ19u8SyntvAUkREtuq6lau58mTx3QdGqyFk5KKdfFpUv1DHMWwFjXixXSKwYUZP\/wrm43B+VfNRgpV1xRUI89frKV6FMS9XcBXLBTdusgfDVNmsDihSkXZUimOz9qGOYlwJGX\/nf398vK1euLMlyT0\/PuGVcuAkq+eyEXHhHcnWQr3ixNlmWU8\/uPbODSCSYHipXtIRbSc7i5cx17ah8oY5hXgoY105ZaX2o5KMGK+2q1OOTvT8tvsKCRXcQ6e8qWCa9s1am3\/5DJyCkZZuTxpeohHbFjbDb+lHHMK8FjGZiGhsbZf\/+\/WexOX36dNm2bZvMnj3bLdMFakMln51Q7K7j9AHkqzI4Naui2ZU\/HBsKFt+aDIsKlqr3LwgW4rq+yJlrROOtD5Uv1DHMawGjrqrrXYaGhqS5uTnwXP1dr5kzZ0pfX5\/cfffd8Xo0D3OMHV\/XD0DthGjX+J6iwuSMQBkrVMJi5S0X1MrkufMTebkcORufM59KoPJFAZOCl2kGZtOmTbJx40bRE6P1Mp\/dcsstcs899zgTMOHTqPMXDqOSjxqstCuFYK3gkeXyZTIp+shwNsU0wUwDGaGi\/2uWRT9P+irXtqTbF\/V5tCsqcunchzqGeZ2BGR0dlZaWFtHponAGZmBgQNatWyff\/OY3c59X4hYqipqamqS1tTWopq2tTbq6unKiCZV8dkKVeE3y9yLxZRbJKorHDjwjU\/7wmwBQFSR66dtsdU1KocsIEV1cq5dmU\/Q6I1iKb2lOnjERJM7C+NGuNLwp+jNRxzCvBYzJuITXwcybN0\/uvfde2bx5s9TX1zt5H4xmXzo7O6W7u1uqqqoC0dTQ0CB1dXWBx6CSz04oeoeQxp22fJn1HaaNRhQUavPplw4XNKWYeChVp4oOc413vymnYuT06dMyadKkIEuiIsRcRphk+RwhW87S8KdKnkm7KkEv+XtRx7BMCZiHHnpIHnvssWDrtMvFuypgdD1NR0dH4FkqYMLiaPCz5yTvcXwiEUgAgaOTLiz6lF+9+Y3vPlQzeWy5aRedfd+0mtxn5+jPH77OyoLh4WGpqXnjXqubMlII1Tba5b8DLly4cEwjBwcH\/W90mS30WsCYKSQVE7qQd\/78M6liIzY0W+LiGk\/AoKpX\/hVVuff0PfnimEqOjJyquNIjI6MF63j11VflvPPOK1n\/Lyp4\/pETxdteiV0zqt8QPzOmnfn54tc\/m1FdJSdPnJCp06ZJ86Lk16hUTNY4FTDG4kbYbf2ofKGOYV4LmPAi3gcffDAQMHPmzDlrYW+lLswppEoR9Ot+1E4oq3Z17hq7lkUFWlhoDR57VY6+fLqoE6kAMsLnqvdMC8plRexklbPxIpp2jYeQX99TwKTAR6EMzN69e+Xo0aPBdI+rDAwX8aZAboyPZOcaI7gxVF2ML81uhTM\/P3r+RPD0x184eVYrjMhRgeOTuKEvxuAwMVaJyhcFTIxOU6rq\/JfZ6SJeXWxrtlW7alZ4G3X+GUuo5KMGK+1yFRXJ1OOCL83ymMxOWOCEhc1Vl06Vq98zNRmjXn+KC9sSbbDlw2iXJVCeFEMdw7yeQvKEe+5C8oUIy3awc7UEypNicfClmRv996PXszWavTHCRkWNihldfxN3tiYO23ygjXb5wIJ9Gyhg7LGquGSpIwS08riyMMUajko+O6GKXTXRCsiXG7g1W5MvaBqufHcgalxnaMiZG86SqgWVL9QxzPsMTLGjBJYtW5aUTzMDkxjSbh6E2gnRLjf+Ea5FszS61sYIGs3OuBQz5Mw9Z3HWiMoXBUycXlOk7lJHCYSPF4i7aajkowYr7Yo7ItzW7xNf4eyMETOVTDP5ZJtL1miXSzTjrwt1DPM6AxPehWQyLvrGXNe7kMZzH1Ty2QmNx7xf35Ov5PgwmZm+J38VrKVpXjQr0noZcpYcZy6ehMoX6hjmtYBRh0xqF1Ip50clHzVYaZeLrjy5OnznS6eYOncdCoTM1ZdOlXsbLpPwy\/lKIeW7bVFZpl1RkUvnPtQxzHsBkw7dY5+KSj47IR+8y74N5MseqzhKPv78Sbn54efKysiQsziYiK9OVL5QxzAvBYxmXfr7+4Mzj0pdPT0945Zx4eqo5KMGK+1y4fXJ1ZE1vnSdjGZkNAtz7\/LLSu5cyppttqzTLluk\/CiHOoZ5KWCUcvNiufb29rNOnNadSevXr5f8F87F5Sqo5LMTistj4qmXfMWDa9RajZBpuPJCua\/hsoLVkLOo6KZzHypfqGOYtwLGuK8RK2F3LiRq4nR3VPJRg5V2xRkN7uvOMl86rbTk\/qeLZmOybFsppmmX+ziIs0bUMcx7ARMnqbZ1o5LPTsjWA\/woR7784CG\/Fbq49+a+50RP886fUiJnfnJWrFWofKGOYRQwFvGFSj5qsNIuC6f2qAgKX0vuezo4rmDnjVfk1sWg2JbvLrTLowCyaArqGEYBM4HJZydkQb5HRciXR2QUacpNfc8Fb\/YdueuaoAQ585+zcAtR+aKAyZYfFm1t+NRpLWTW04Q\/z19jg0o+arDSrmwFKxpfZnGvZmIuevMJmTVrVrYIsWgtGmfGZFS7UMcw7zMw4RfZPfTQQ\/LYY48FW6dnz55tEWZnF8k\/W0lL6DOampqktbU1uKGtrU26urqkuro6+B2VfNRgpV2RQiO1mxD5MpmYB5ZeKJ+5uvAOpdQAd\/BgRM6QM2aoY5jXAiZ8lMDQ0JDMnz8\/CL2+vj7p6OiQqqqqskNRjyKora0dszVbsy\/6eXd3d1BnS0uLNDQ0SF1dHQVM2QinfwM71\/Q5KKcFqHzpmpjBY6\/Kf97+8XLgyERZVM5Q7aKASSGswoc5Pvjgg4GAmTNnjmzatEmiHOZY7FiCAwcO5ESRmqkCpr6+PidylPzwtXv37hTQcP\/I4eFhqampcV9xyjXSrpQJKPPxqHwpDCv7Dsvb3vY20UwM0oXKGZJdCxcuHONyg4ODSC4Y2JK5DMzevXudHeao00kDAwOydOlS2bFjR5DVKSZgEMlH\/WuDdmWrn0LlS1n40f6DsrhnODhDaedNV2SLmBKtReUM1S5mYFIKvUoOczx48KCsWrUqEDyLFy8+a9rJTB2tWbNGtmzZwimklDh2\/VjUToh2ufaU+OtTzn75x2nBy+7C26vjf3K8T6Avxouv69opYFwjalmfWXS7evVqaWxslP3790c+QiA8JaULdHXdi15aNxfxWhKSgWLsXDNAUqiJqHypicY2XQ+jL7p7ZsPHskVOkdaicoZqFwVMCmGni3jvvPPOYNfRU089JbqQV6d79BDH2267LdIi3vB26Xnz5gVZFxUz4c\/zz1hCJR81WGlXCsFawSNR+QoLGP358juekIYr3y3Ni2orQMuPW1E5Q7ULdQzzeg1M\/iJe3T107bXXRl7EGzX0UclHDVbaFdXT07kPla98AWPOTUKYSkLlDNUu1DHMawFjMjCf\/vSnZevWrbn3tFSSgYnSRaOSjxqstCuKl6d3Dypf+QJGf9epJL2yvqAXlTNUu1DHMK8FjAa6mdq54YYbxqxVifoiuyjdNCr5qMFKu6J4eXr3oPJVSMDoZ9Vf\/qHc13CZNFyZ3a3VqJyh2oU6hnkvYNLrVt94Mir5qMFKu3yIGvs2oPJVTMCYowbMeUn2SPlTEpUzVLtQxzDvBYzuFNLpo\/AVXnybREijko8arLQriahw9wxUvooJGP086wt6UTlDtQt1DPNawITPKNJdSDNnzpTDhw8HPeeyZcvc9aDj1IRKPmqw0q7EQsPJg1D5KiVg9MRqPS9Jt1XPqJ7sBMckK0HlDNUu1DHMewFjjg3Q1\/3rW3j1nS1RjxKIGuCo5KMGK+2K6unp3IfKVykBY7IwV106NVgPk7ULlTNUu1DHMK8FjO5CuueeewLRcvz48dxbdTmF5Ka7Qw1W2uXGP5KqBZWv8QRMlrMwqJyh2kUBk1RvlvccffPuueeeK7rrSHckrV27VrZt2xb8ntSFSj5qsNKupCLDzXNQ+RpPwOj3uq364urJmcvCoHKGahfqGOZ1BsZN91h5LajkowYr7arc55OsAZUvGwFjsjBZe7kdKmeodqGOYd4LGD0Laf369WP6U04huRleUIOVdrnxj6RqQeXLRsBkNQuDyhmqXRQwSfVmoeeYk6ibm5ulrq4uhRaceSQq+ajBSrtSC5VID0bly1bAZDELg8oZql2oY5jXGZj806Mj9Y4ObkIlHzVYaZcDp0+wClS+bAVMFrMwqJyh2oU6hnktYDSwdQpJr6jvfdH79RRrzeLoZbI6ujh48eLF0tHREZxqHT6Nur29fczzUMlHDVbalaD6cPAoVL7KETBZy8KgcoZqF+oY5qWACYuMQv2j7RoY8xZfPUfJCBj9TE+1XrJkibS0tEhDQ4PMmTNHmpqacodFtrW1SVdXl1RXV3MKycEAlXQVqJ0Q7Urakyp\/XjmcZWlHUjl2VY5icjWg2kUBk5wPOXmSZl70zb368ju9VMDkr6kx2Zn58+eLCpvu7u4gG2OEjVl3g0o+arDSLichlFglqHyVk4HRslnKwqByhmoX6hjmZQZGg\/ngwYO5F9eFp3rK7VVVmIQFjMm06HtkVMAMDAzI0qVLZceOHcF0kl4qYOrr63PTSEp++Nq9e3e5zfCy\/PDwsNTU1HjZtkoaRbsqQS\/5e1H5UiTLte1LO14MCHhgqd8nVZdrV\/JeFe2JSHYtXLhwDAiDg4PRQPH4Li8FjL6BN5wFMdM+UdbBuBIwiOSj\/rVBuzzucQo0DZWvcjMwWv7x50\/KkvufFt\/fC4PKGapdzMAk2Cfm7z7SbMyuXbvk5ptvLtgKI3j6+\/tl+vTpY97Umy9gGhsbg+kknR7iFNIhmTVrVoLMJvMo1E6IdiXjPy6fEoUzXQuj186brnDZFKd1RbHLaQNiqgzVLgqYmBymULWFBExPT4\/cdtttwRqVcq6wgNH7uIj3DfRQg5V2lRMh6ZdF5StKBkbvOTJySi6\/4wmvszConKHaRQGTYD8Xp4AJ73AK704Kb6Pu7e0d8+I8VPJRg5V2JRisDh6FyldUAaP33dT3nPzohZPyzIaPOUDYfRWonKHahTqGebkGxtU2aldhi0o+arDSLleen0w9qHxVImD03uov\/zA45LHhSv8W9KJyhmoX6hjmpYBJptu0fwoq+ajBSrvsfduHkqh8VSpgjIgZuesaH2ga0wZUzlDtQh3DKGAsugZU8lGDlXZZOLVHRVD5ciFgdC3MjGmTvVvQi8oZql2oYxgFjEVHjko+arDSLgun9qgIKl8uBIyv26pROUO1C3UMo4Cx6MhRyUcNVtpl4dQeFUHly4WA0Tp0W\/XjL5wUn6aSUDlDtQt1DKOAsejIUclHDVbaZeHUHhVB5cuVgNF6dCrpqkunBot6fbhQOUO1C3UMo4Cx6A1QyUcNVtpl4dQeFUHly6WA8W0qCZUzVLtQxzAKGIuOHJV81GClXRZO7VERVL5cChgzlXTkxCm5d\/llcvV7pqbKICpnqHahjmEUMBbdACr5qMFKuyyc2qMiqHy5FjBhEZP2C+5QOUO1C3UMo4Cx6MhRyUcNVtpl4dQeFUHlKw4Bo8cM6GGPaW+tRuUM1S7UMYwCxqIjRyUfNVhpl4VTe1QEla84BIzWac5KuvrSqam9HwaVM1S7UMcwChiLjhyVfNRgpV0WTu1REVS+4hIwhjo9akCPGUhjZxIqZ6h2oY5h8AJm+\/btMjQ0JM3NzUHchw9t1N\/b29tl2bJlYz43n5mOApV81GClXR6pE4umoPIVt4AxO5PSyMSgcoZqF+oYBi1gOjs7ZevWrRI+dTpf0Ggno4dHNjU1SWtra9DdtrW1SVdXl1RXVwe\/o5KPGqy0y0I1eFQEla+4BYzWn5aIQeUM1S7UMQxWwKhQmTlzpuzduzfoqk0GRkVNbW1tkHUxl2Zl9PPu7m6pqqqSlpYWaWhokLq6OgoYjwY626agdkK0y9YD\/CmXBGdmTcyM6smJbbFOwq40WES1iwImDW9y8EwVJkbAaKalsbFR9u\/fH3w2b968QLQcOHBA+vr6pKOjI\/hcBUx9fX1O5KCSjxqstMtB4CRYBSpfSWRgDE0qYm7uey44cqB50SxpXlQbK4OonKHahTqGwWZgTPSGBUx+RGuWZmBgQJYuXSo7duwoKWDC9+7evTvWziGpyoeHh6WmpiapxyX2HNqVGNROHoTKl4KTtG0P\/PtJ2frjkzL97ZOkf2V8sZ20XU4czaISJLsWLlw4xuLBwUELBLJVBELAjI6OBlmT\/v5+mT59umzbtk1mz54dMFFKwJipozVr1siWLVs4hZQt3y3aWtS\/omhX9hw0Dc6SyMakYVcS7KPaxQxMEt4TwzPyp5A2bdokGzduDBbomu9Wr17NRbwxYJ9WlaidEO1Ky6OiPzdNzjp3DUnnrkOia2Ncv7k3TbuiszH+nah2UcCMz72XJfIzMOFt1GYNjIqZ8Oe9vb25BbxqFCr5qMFKu7wMxQmXMVODffDFsJBpuPLdwbtjVNRUcvlgVyXtL3Yvql2oYxjEFFIcjhyuE5V81GClXXFHhNv6UfnyRcAYtlTI9D35q+BNvipgVMxEXeyLyhmqXahjGAWMRV+MSj5qsNIuC6f2qAgqX74JGEO5Cpi+J18Mppf00l1LZ\/6337mEyhmqXahjGAWMRUeOSj5qsNIuC6f2qAgqX74KmHwh86PnTwTbr\/WyXS+DyhmqXahjGAWMRUeOSj5qsNIuC6f2qAgqX74LmLALaFZG\/\/3ohZO5zIwRNDrVdGbK6cLcLaicodqFOoZRwFh05KjkowYr7bJwao+KoPKVJQGT7w5mmkk\/z8\/QzJg2WeaeLzJ12rSypp08crmiTUH1RdQxjALGIqpQyaddFuR7VIR8eUSGZVPQONOFwHp1fesHcvod782hYHY1qbi56j3Tyl5TYwln7MXQ+DKAodpFAWMREqjk0y4L8j0qQr48IsOyKeichaeeTLZG\/zdrasIwhUXOxdWTZUZ1VfB1\/vSUJbSxFEPnKxbQUqyUAsYC\/Eqd2ub+UmUKfTfeZ+HvbX62gOGsIjZ26U3Fytl+nl9uPHts21XMZtv7K+UsabtKcRHGgnadQYMxdubV84XizTZGwn5lsjdHRkblke\/vlbqPflSOnDiz9qbUFX5njWZ49FIB9N3vfleuu+66nBDSz7eu+QsJvzKfMXZJgEcUvqKMCUnfQwFjgbiSz4sIEAEiQASSR+DU+\/4699A\/nXt+8POfzn3HmIaYzwt9F1eL3\/Tb\/+e86jf99rjzOk2FL\/3j\/4yt7oE83OQAAAquSURBVLQqpoBJC3k+lwgQASJABDKLgMkouTRAs1NxXfc1XBZX1anVSwGTGvR8MBEgAkSACBABIhAVAQqYqMjxPiJABIgAESACRCA1BChgKoR+YGBA\/uzP\/kz0YMisX6Ojo\/LII4\/If\/3Xf8n1118vs2adecU4ynX69Olg4d8nP\/lJmTbtzFbPrF8nTpyQb33rWzI8PCzLly+Xyy+\/POsmBe1\/+eWXA7vUvpUrV8pFF10EYZcxYufOnXLxxRfLFVdcAWHXK6+8It\/+9rcD3tSuJUuWyFvf+lYI27Tf+M53viM\/+clP5POf\/zwMZz\/4wQ\/kueeeCzh629veJkuXLpULL3zjZYVZII8CJiJL6tTaCf3Lv\/yLrF69eszp1RGrTP22f\/3Xf5Xzzz9fZs+eHQweX\/ziF6Wq6sxWx6xfv\/nNb+TBBx+UoaEh+bu\/+zvRE8gRru3bt8v73ve+4N8DDzwgX\/jCFyDE2fe\/\/32pra2Vd7zjHYEv3njjjTAD4oEDBwJf1B00dXV1CG4oatMLL7wgf\/VXfwVhT9iIf\/u3fwsG+I985CPyve99T\/7yL\/8Spl9UO5W7H\/\/4x4E4O+ecczLFHwXM63Tt27dP+vr6pKOjI3BOzUa0tLRIf39\/kF3p7u4eM+ipgPntb38rP\/vZz4IafO6IRkZGpKmpSVpbWwNxopcOfOvXrw9+7u3tDdqvA6B2QPoXVE9PjyxevNj7gd7WNuXzT3\/6U\/CXFJJdprfRTMU3vvENueGGG+S8887zthOy5UsN0L\/mdfB4y1veIp\/+9Ke97lxt7Xr11Vfln\/\/5n+Vd73pXwJPP\/YZyYGvXY489Ftj1pje9Sf7mb\/5GPvGJT3jNVzm2aVypgHn22WeDPxDe\/\/73extf5dilZX\/3u98FYnrZsmVywQUXeG1XocZRwIQGcx3YjIDRAV7\/Wm9ubpbOzs7gr0ENzP\/4j\/8ISNe\/ePWvQxU+PguYgwcPyqpVq4I2btu2LRAw+llbW5t0dXUF6tsIN\/1fMVBHzoKAKcc2k0lCtEvFyz333CMNDQ0yZ84cbzuhcvn6\/e9\/L\/pqd83GqDDzNRtoa1d7e7vs2bNH3vve98qxY8e87je0cbZ2aZ+pQmfKlCly7rnnypYtW+Szn\/2s19MR5dh2\/\/33y8c\/\/nH5wAc+IFu3bg2mNH2dgi7HLo2np59+Wo4ePSqf+tSnvO03SjVswgsYFSCHDx8OMNL1LBqMemn2pb6+PlCmJjujg752qq+99lrw15PO8fosYLRTUXWtKc\/bb79dNm\/eHAgYFWfGVs1MmOyM\/oWhf12oWNOBXm1\/+9vf7qVjl2ubyTz5LmDKtevP\/\/zPA451us\/ndSLl2vXrX\/868EUdKL7+9a8Hayp8tK8cu9auXRusOfjFL34hv\/zlLwPbbr31Vi8zZuXYpZldtUn50sySChj9Q0gzuT5e5dr2+OOPy1VXXRX0nfpHoP4h6+MUdLl2qT3aH+o4Z\/pHH\/migLFgJTyoGwGjf9FqildFimZh8qeRtJzPAsaYrap83bp1YwSMyS6p0zc2NgaZJu1wdBpJO1Z16CyoclvbTKredwFTDmfKqf4FpYOhZgNVzHzuc5\/zckAsxy71RR0gNCb1DwX9WefnJ02aZBHJ6RQp1w+z0G+YLIxN36F\/6Chf2neoL+qCcp\/5Ksc2FWUqXDQzPXny5CCj7bNttr6oC8g1u6R\/\/PiaURovmid8BsYAFFXAjAewD9\/bOrTv8\/GFsES1jXbVBWuW9J\/Pg0W5wixrMYbqh+UIGHLmwyhWuA0UMK\/jUkjA5E8hmfUx\/tJZuGWFOqFCU0hZTCOi2ka7ziw2z8pFvrLFVzEBg9Avovpiob6AAqaAgNHFTYUW8eqakCxe+Q5dbBGvr4skS2GOahvtytb2ffKVLb4KCRiUfhHVFylgSoyE4QxM\/jbq8O4kBAGjNpht1NOnT8\/tTqJt\/iCQ3wmhcEa7\/PExm5ag8lVIwDDGbDzCrzLMwPjFB1tDBIgAESACRIAIWCBAAWMBEosQASJABIgAESACfiFAAeMXH2wNESACRIAIEAEiYIEABYwFSCxCBIgAESACRIAI+IUABYxffLA1RIAIEAEiQASIgAUCFDAWILEIESACRIAIEAEi4BcCFDB+8cHWEAEiQASIABEgAhYIUMBYgMQiRIAIEAEiQASIgF8IUMD4xQdbQwSIABEgAkSACFggQAFjARKLEAEiQASIABEgAn4hQAHjFx9sDREgAkSACBABImCBAAWMBUgsQgSQEBgdHZWWlhbp7+8fY9YNN9wgzc3NmTZVz+7ZtWuXNDY2BjaaE+XVKGN3Q0OD1NXVnWWnfn\/PPffI6tWrpbq6OtM4sPFEYCIgQAEzEVimjUQghIAZyMODOwJAYQGiB7KWK2AUAyOAbr75ZgRIaAMRgEaAAgaaXhpHBM5GoJSAMaeyHzlyRJYvXy4f\/OAHZdWqVXL06FGZN2+edHd3B9mJffv2yYoVK0RPM1+wYIFMmTIlyFxo5kOzOJrh0LqGhoaC383p59oak+nROrZu3Ro0cO\/evRI+9b2zszP3XW9vb1Cmr69POjo6gp9VnORnUrS+w4cPy7Jly3LZlmIZGH2eebbWF372vffeK4sWLZLZs2fTfYgAEfAYAQoYj8lh04hAHAgUmkIy4uTRRx+Vhx9+OBAqejU1NUlra2swmBtBEhYqep+KCRUyxQTM\/PnzC4oPrX\/t2rWybds2Of\/883PiRz9XAaNtOH78uLS1tcltt90mf\/\/3fy8bN27MfdbV1TVmqkfv0WepeCo2TaZ1qyAKTyGF79Pv1U69VAjxIgJEwF8EKGD85YYtIwKxIGCTgdFMx\/DwcC77YhqigmXNmjWyZcuWXDamkLAJZ2Bqa2tl\/fr1Y2zRLIyKDSNUzJSPZlU0i2IyN+GbjNDQzzSDEl6vozbdeeedsnLlykBsFbKx0BqYfPGidWsmJ7\/+WIhgpUSACFSEAAVMRfDxZiKQPQTKETCa\/cjPdOgAb4SHTifZCJhCgiRcj42AMcJCETeZFoN+FAFTLNNCAZM9n2aLJyYCFDATk3daPYERsBUwWk7XtOhaGJ1OMetj1q1bJ7rIVTMg4SmkW265JbdwdsmSJbmpJRUbZqqopqYmV2bmzJkFMzBKTXgKSZ+3efNm0Xt1l9DPf\/7zs0SVuSd\/CqnYGhjN8uhVaJqIU0gTODhoeqYQoIDJFF1sLBGoHAFbAaNZEd2VY7uIN7xYN7y4t9Qi3kJTSGb6yUw7tbe354RGeGFwPhLhzEmpKaRPfepTwRTY\/v37c1WYNUBqc3gqqnK0WQMRIAJxIUABExeyrJcITBAESokKlxAk8R4XbqN2yRjrIgLxIkABEy++rJ0IwCOQhIAZGRkJprNmzJgRbKXWTEmhq5L1K\/nraOCJo4FEIOMIUMBknEA2nwgQASJABIjARESAAmYisk6biQARIAJEgAhkHAEKmIwTyOYTASJABIgAEZiICFDATETWaTMRIAJEgAgQgYwjQAGTcQLZfCJABIgAESACExEBCpiJyDptJgJEgAgQASKQcQQoYDJOIJtPBIgAESACRGAiIkABMxFZp81EgAgQASJABDKOAAVMxglk84kAESACRIAITEQEKGAmIuu0mQgQASJABIhAxhH4\/7rfr4N\/zmLTAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:0fd0717d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:44396bd3]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:8de91a74]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:360d2452]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:0c11bd1d]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:2fa74918]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:79eff6bc]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:6d3e5d2e]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:14c546cf]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:075d78cf]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:6e51d2e7]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:57b55c06]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.012443765785704"}}
%---
%[output:616fcdfb]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.517590710054527"}}
%---
%[output:3623cefa]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.152987786896029"}}
%---
%[output:14798efd]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"  93.745795204429072"}}
%---
%[output:2059d561]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -1.166194503484206e+02"}}
%---
%[output:5c304d9d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ9431V97z9ubJKJ0AZhJcSurKSK06WU4UJAq+tYt92l7Lr5JOm9m4vR203Qey\/0SVJR7hCkSday6wBnxxOzbs+adruCtlOHWtHJjZ2s0Nw5EQKhLSFU\/pQKatGhvc\/nW044+eb755zzPZ\/f7\/s9eX+fh6ckv8\/5fM95fc7vnHfO31ecOHHiBOEBARAAARAAARAAgQoReAUETIWihayCAAiAAAiAAAhEBCBgUBFAAARAAARAAAQqRwACpnIhQ4ZBAARAAARAAAQgYFAHQAAEQAAEQAAEKkcAAqZyIUOGQQAEQAAEQAAEIGBQB0AABEAABEAABCpHAAKmciFDhkEABEAABEAABCBgUAdAwBOBo0ePUm9vb+RtZGSEGhsbPXme72ZycpJ6enrooosuosHBQWpoaBB7l+64lmU0KZDi8P73v586OztNkpTaZmhoiLZt2xblccOGDdTf32+V3127dtGmTZto8+bNpeTB5du3b5\/498MKGowrSwACprKhQ8bLRqCWnXuagOEOYvXq1dTW1iaCJ62M0u9NK4xLh8gd6Fe\/+lUrceCSxjYA\/I7169fPJgtRwIQmOG1jDHu\/BCBg\/PKENxCoC4Hjx4\/TwMAA7dmzh3bs2FEzAcMjP7V4bxJU1Rl2dHQYixE1QmEjDlzSuFQCJWBs8hZ\/T9lHYFQ9PXz4MEZhXCoJ0swhAAGDClFKAvpQOmdQHxLXRx9WrlxJN9xwQ1QG7sj06RQ1WjAxMTHvc93HFVdcQe95z3sim6amJhodHaWWlpZELrpQiNvHRyf4czWltGbNGrr55puptbU1arj1jj\/PD09Fqffu378\/yh8\/agrpuuuuo4985COReFFPnAX\/XjHVBY7qNHV71QkqX3qHqpfx1ltvpeHh4cT3Tk9PR\/mbmZmZzZMeQ50jM7\/tttvok5\/8JKnyMf84a8VOTc0lddYqrvp7VXnj5VKxbm5unhVhcX67d++OpmTUo9ePuL884Rivj1m+suph1kiNzoTzrPIeF0Xx75fOVvm4+uqrae\/evcTfHxU7vcz8O\/UOPbZ5XJLqYSkbIWSq9AQgYEofooWVwXinpZdeNcJJnVS842E\/LB6UeIl\/ntTBZnX+\/Fla3lRnc+aZZ85ZA6MEjJ4HFgpJgkMXMXE\/vgRM0l\/48c4k3rGlceXfpwmYvr4+uuqqq+axzxIM\/NlZZ51FTz31VCTQkkQFv1PvaON5T6sX6r333Xdfohi54447Zted6PVN76DjAibuS32eJmKy6iynOXToUKpQ0vMUFy9xkanEw1ve8hb62te+NqfxSBIhSd+vuABhm6Q88u\/Ve\/J861zKPkq0sFrcapcWAqba8Qsu96qB1v8CVY0\/F1YffeC\/slXDqHcQemOr\/+Wpd3gsEtQIgfKh3h3\/S19BTvpcb4wvv\/zyVAGj\/4Vq6ydPwPCoEz95UzlZI0Q8KvTMM8\/MY6KPGjCnFStWzCmjyRRSfHpLsVfx5NGWeNw5L7weJGlkiFmuW7cuKq8+YmOysNlkOihuE\/9ZMVFii\/Of925V95LKo37HQpfLnDaFpHNU9Ske0y9+8YuREEoaUUnzGx+FU6NOuo+sd6sRGlX\/87j4mCoLruFDgZwIQMA4YUMiKQJpf52pDoAb7lWrViXuwNFtDh48mPhXNedb98F\/9asdQ3mLcPP+ckwTCHqDzu+39eNLwOjvZjHCj95hpnUsegf+3ve+11jAJI1YJb2X8xGfIksb4WBb7ohVPnS2Se+LT6VlCZi0qbN4mqzRlCTxGy+bmp6MCyEl2tKERl791OOr+0iLa3w0R7FSAiZt6lDfYafXZfW91KfvVDuhc0matpRqT+A3bAIQMGHHt3Klq7WA0bch53UQtsKD4Sdtqzb1o3fO8c6OfevbqE1GYNhGX\/jKP\/OW3fgIVLwDtRUwcY7xUZq4cPIlYFRlTxNOvDMrScDEp6LyRmCqIGCSRvxUXOP1L20ERveR9t2AgKlcExtUhiFgggpn9QvjOoUUn+pQawrS\/ppNGvLPEzBZUz\/6qABHgf9KTRMwpn54aD4uLtTUWlzAsEgwWRwZ79z1EYr4NBx3+HlTSDw6lCcA4j5cp5D02p02qhH\/BsTFSHw0QpVZH4lT5VF1J54maQop75snPYWkxK4auUoTMEkjV4pRfAQmbdF1fPoqawopiQumkPJqCz43JQABY0oKdjUhIL2IN0sA5AmYrLwlrQ9JEzB5fni4Xa1niUM3ETCcJmkXkvIV30miHwBns4hXTSXoafi973jHO6LRoaSHOSWVz3QRL\/tUoi4unNIWuOppdBt+58c+9jG68cYb5y045jRxAcO\/S1sQrMqaJ5iTplfyRsB0jmllzBIfumD4wAc+kFq3snxwHpIW95ou4tW55I1A1qShwUuCIAABE0QYwyuE6TZqfQt03jbqpIXBNlNITDlreiJvkax+Mm+WH35PfMvthz\/8YTpw4MDsotWkERi9c0tbiKz7jq\/NSRI4ekeup+X\/VwIm6b233377nBNl+XA9fb2NyzZqXYjoHWrSFvu07dtxrvqaHPbJ3LZs2UIbN26McOgjaWo3Wdq27LzzW7K2UfO7TEcm0tau8ChckjhIG3ViRklb2JNGcdLEL\/8+fvJv2loi5cNkpDC8Fg0lkiAAASNBFT5FCeTt+BB9OZwXJpA0VRXfaZZ2Do\/+cpeD7ApnfoE60AWnEmpJO5Py8OAguzxC+NyGQJAChhs2PouCD9lKagizDjizgQfb+hCAgKkPd19vzZpCy5r6Snq\/y1UCvsqx0PwkTSExg7zDH5NEZyh3Vy20OlC28gYnYPIW96nP29vbo8vO1M\/8JbS9OK1swVwo+YGAqX6k439EcIlsxQunwd06ta0L8aldG\/HCOYXgrG28Qn9bcAKG53v5S8JP2ghMPKj8l8X4+HhNb\/UNvWKhfCAAAiAAAiAgSSAoAcN\/1V1\/\/fXU3d0diRgIGMmqA98gAAIgAAIgUD8CQQkYHknhh0+EzFoDo+NWQ9ldXV3RlBIeEAABEAABEACB8hMIRsDwXPj27dvp2muvJb6oz0TAqPUvHCb9FuN42H7xF3+x\/JFEDkEABEAABEAggQDfKn7eeecFxyYYAcNTRnzWBJ8emrcLiaNoKl7YlgXM1NRUcMGvd4HAVS4CYAu2cgTkPKPeyrANlWsQAiZpR4OqBknX29vuPAo1+DJfFXOv4GrOytYSbG2JmduDrTkrW0uwtSVmZh8q1yAETDyEeSMwPFrDp1BmTRvpPkMNvlnVl7MCV7CVIyDnGfUWbOUIyHgOtc4uCAGjRlx4d9KKFSuiG4LVseCqumQdvR5q8GW+KuZewdWcla0l2NoSM7cHW3NWtpZga0ss3\/7w0Reo7b9uopnP\/Xm+ccUsghQwvmOAL5Vvoif9Pfroo0EuLJOhZecVbO142ViDrQ0tO1uwteNlYj127xG6cuwBOnrz203MK2UDAWMQLggYA0gOJmisHKAZJgFbQ1AOZmDrAM0wCdgagrIwg4CxgBWiKQSMTFTRWMlwxeiWHFewBVtZAv69Q8D4Z1opjxAwMuGCgJHhik5WjivYgq0sAf\/eIWD8M62URwgYmXBBwMhwRScrxxVswVaWgH\/vQ3cdpKG7HsUaGP9oq+ERAkYmThAwMlzRycpxBVuwlSXg3zsEjH+mlfIIASMTLggYGa7oZOW4gi3YyhLw7x0Cxj\/TSnmEgJEJFwSMDFd0snJcwRZsZQn49w4B459ppTxCwMiECwJGhis6WTmuYAu2sgT8e4eA8c+0Uh4hYGTCBQEjwxWdrBxXsAVbWQL+vUPA+GdaKY8QMDLhgoCR4YpOVo4r2IKtLAH\/3vkUXt5KjZN4\/bOthEcIGJkwQcDIcEUnK8cVbMFWloB\/7xAw\/plWyiMEjEy4IGBkuKKTleMKtmArS8C\/dwgY\/0wr5RECRiZcEDAyXNHJynEFW7CVJeDfOwSMf6aV8ggBIxMuCBgZruhk5biCLdjKEvDvHQLGP9NKeYSAkQkXBIwMV3SyclzBFmxlCfj3vu62++meR45hEa9\/tNXwCAEjEycIGBmu6GTluIIt2MoS8O8dAsY\/00p5hICRCRcEjAxXdLJyXMEWbGUJ+PcOAeOfaaU8QsDIhAsCRoYrOlk5rmALtrIE\/HuHgPHPtFIeIWBkwgUBI8MVnawcV7AFW1kC\/r1DwPhnWhqPk5OT1NfXR8PDw9TS0pKYLwgYmXBBwMhwRScrxxVswVaWgH\/vK2\/8Oh0++gIW8fpHW1+Px48fp4GBAdq\/fz+Njo5CwNQ4HBAwcsDBFmzlCMh5Rr31z7bx6rsjp7hKwD\/bunrct28fDQ0NRXnACEztQ4HGSo452IKtHAE5z6i3ftnyyAuPwEDA+OVad29Hjx6l66+\/nrq7uyMRAwFT+5CgsZJjDrZgK0dAzjPqrV+2EDB+eZbG265du6K8rFq1Cmtg6hQVNFZy4MEWbOUIyHlGvfXL9p6Hj9G6j9+PERi\/WOvrjRfubt++na699lqanp42EjB6jvfu3VvfAgTydmbf3NwcSGnKVQywlYsH2IKtHAE\/ntesWRM5euH1V9ALr18HAeMHazm88JTR6tWrqa2tjbALqX4xwV9bcuzBFmzlCMh5Rr31y3boroM0dNejEDB+sdbPG6996e3tpYmJiXmZ2LFjRyRq4g+2UcvEC42VDFf2CrZgK0dAzjPqrV+26gyYn\/rB0\/T0J97p13kJvL3ixIkTJ0qQj7plASMwdUOPTlYQPToCObhgC7ZyBPx6VluoT3n6QXryb\/7Yr\/MSeIOAwUF2dauG6Ajk0IMt2MoRkPOMeuuPrb4D6ecm\/pam937Sn\/OSeFrwAsYkDphCMqFkb4PGyp6ZaQqwNSVlbwe29sxMU4CtKal8O13AnHbPMB3+xufzE1XMAgLGIGAQMAaQHEzQWDlAM0wCtoagHMzA1gGaYRKwNQRlYKbWvyxtPJWe++R\/oampKYNU1TKBgDGIFwSMASQHEzRWDtAMk4CtISgHM7B1gGaYBGwNQeWY6aMvly1fRN\/c+nsQMH7QVs8LBIxMzNBYyXBlr2ALtnIE5Dyj3vphO3bvEbpy7IHI2W3dF9C1ne0QMH7QVs8LBIxMzNBYyXCFgJHjCrZgK0uguHcefblq7AG655FjxNNHBz50CYXah2EKyaC+hBp8g6KLmkDAyOEFW7CVIyDnGfW2OFt99KXvN5bRwG+eBwFTHGt1PUDAyMQOjZUMV4wSyHEFW7CVJVDMO4++8N1H\/C+Pvux+34XRv6H2YRiBMagvoQbfoOiiJhAwcnjBFmzlCMh5Rr0txvavvjZNA3dORk76155H\/WuXRf8fah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn11p2tPnWk1r4ob6H2YRAwBvUl1OAbFF3UBI2VHF6wBVs5AnKeUW\/d2Orbplm83Np1AV12\/qJZZ6H2YRAwBvUl1OAbFF3UBI2VHF6wBVs5AnKeUW\/t2erihVPzuhddvGAKyZ5pUCkgYGTCicZKhit7BVuwlSMg5xn11o5tXLz8919bSv\/rd5bPcxJqH4YRGIP6EmrwDYouaoLGSg4v2IKtHAE5z6i35mzvefhYtONIPe+6pIn+\/J2vS3QQah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn1Np8tj7r8w33foY9+7uW7je7441Z624rG1MSh9mEQMPn1JdgtaAZFFzVBYyWHF2zBVo6AnGfU22y2POpy1c4HonNe+OEFu7xduvviJZkJIWDk6mzpPYca\/HqDR2MlFwGwBVs5AnKeUW\/T2b5\/57fp777xxKyBflBdXkRC7cMwApMX+YAPATIouqgJGis5vGALtnIE5Dyj3s5lyyMt\/zz5LH1g17fnfLD9j95IHb98lnEgIGCMUYVnGGrw6x0pNFZyEQBbsJUjIOcZ9fYkWxYun\/\/3p2nTS6fqKuLxA+pMIxFqH4YRGIMaEGrwDYouaoLGSg4v2IKtHAE5z6i3RFft\/Dbt0KaKmHbS4XQ2UQi1D4OAMagFoQbfoOiiJmis5PCCLdjKEZDzvFDrLS\/OHb7rUbrnkWNz4BYVLspZqH0YBIzBdzHU4BsUXdRkoTZWolBfcg62cpTBFmx9EGDRcueBJ2l0\/PF57nwJFwgYH5GqgY\/jx4\/TwMAA7dmzJ3rbhg0bqL+\/P\/XNu3btok2bNkWfd3R00ODgIDU0NCTaQ8DIBBAdgQxX9gq2YCtHQM5z6PWW17b8\/f4jdNPnH00ULZcuXxRti2YB4\/MJtQ8LZgRmaGgoijeLlqNHj1Jvby91dXVRZ2fnvHqwb98+YvuRkZFItLDwaWpqShU8oQbf5xfExVfojZULE19pwNYXyfl+wBZsbQjwSMtd33qabvvKY4nJfI+2JL0k1D4sGAETD5ouaOKf8ejL+Pj47KhL\/Oe4fajBt\/kSStiiI5CgetIn2IKtHAE5z6HU27Q1LYqcEi38r+\/RFggYufpZE89qBIZHY9ra2oxGYNrb2xNHazgxBIxM2EJprGToFPMKtsX4ZaUGW7CNE+Cpob\/9lxn6l6nvzluIq4uWv+h8PS07s6EmokXPY6h9WHAjMDzysm3bttx1LZOTk9TT00MzMzO0Y8eORKGjKgAHX3\/27t0r9w1eQJ6np6epubl5AZW4dkUFWznWYLuw2c4892IE4PHnXqTbv3GM9j9+8lj\/+NN0+il0zqtPoQ2\/uij6l3+u1bNmzZp5r5qaevnupFrlQ\/o9wQkYBYyFDIuTpMW5PGW0c+fOaA1MY2NjtB6Gn7RFv6GqV+nKlecff8nmEXL\/HGzd2eWlBNs8Qu6fl5WtWscy8djzqSMsXGqeDur6lSV02fmL6bLzF7mD8Jwy1D4sWAHDIyx9fX00PDxMLS0ts9VB7VbSp4zSbFWiUIPv+Tti7a6sjZV1QUqYAGzlggK24bNlwfKp+79Ddz94dPbixKRSs2C5bPli6rqYRUt5BEs8r6H2YcEKGH2nEY+yqAcCRq7xsfWMjsCWmLk92JqzsrUEW1ti5vb1YMti5cnnf0R\/Pf545uiKGmHhrc7dF58TjbbUYgGuOb10SwgYHxQ1H2qh7cTEhJXn1tZWuvPOO+el0aeBlEhJ2xqdNIWUNt3ELwo1+FbgBYzr0VgJFKOULsFWLixgW122LFae+t6PaPT\/5osVJVj+88qzqaf93KjQVREsGIGRq6OR57ydQkmvV6MqSQImfpCdfjid+qy7u3t2sa5a7MvvwUF2wsFOcY+OQI472IKtHAE5zz7rLYuVZ77\/Ixq5x1ysLF18KvW9dJBcVcVKUnRC\/SO8blNIvgWM3FcKIzBSbH02VlJ5rKpfsJWLHNiWhy1vX2ahwf9u\/dJBevSp47nTQGokhcXKe97STI0\/9zOVmg5yoQ8B40ItkDShBr\/e4UFHIBcBsAVbOQJynvPqLY+qPPid79NnDjxpJFR0sfLrF5xJq5aeXurFtlJkQ+3D6j4Cw2tg8u4tkgqqqd9Qg29afim7vMZK6r0LwS\/YykUZbGXZ\/vQZ50QjKvce\/O7JXUDPvpC5E0jPDY\/GqEW2s+LF871CcqWX8xxqH1Y3AaNCpa9F4UW3o6Ojc7Y9y4XU3HOowTcnIGOJjkCGK3sFW7CVI+DHM4sUfr784FG6477vWAsVngJ62+sa6c3Lzgh+Cqgo8VD7sLoLGBWY+K6kMo3KhBr8ol+KounRyRYlmJ4ebMFWjoCdZ5724edz33yKvvn494ynftQICv+rRlXKfNaKHZXaWofah5VGwOjh1I\/5L8OoTKjBr+1XaP7b0MnKRQBswVaOwHzPPJrC\/6nzVNjinkdOCheTR+34af35U+iG33tT5KtK56yYlLGeNqH2YaUUMHqg0w6kq2VlCDX4tWSY9C50snIRAFuw9U1AiZSpp39A\/\/Lod+mxoy9YiRQ1osJTP+3nL4pOsFXCRf2Leus7aif9hdqHlVLA6CMwDD\/vskWZkL\/sNdTgS3PL84\/GKo+Q++dg684uL2XobHnK5+zTf5b+4suH6fAzZtuSdWZKjPC0zzWXL6OZYz80Hk0JnW1e3ZL6PNQ+rDQCJusgOqmgmvoNNfim5ZeyQ2MlRRaLeOXIVputmpphkfLsD\/6D\/unfn3YaSdFHU\/6grYnOOeOV80ZTXGKANsGFWn6aUPuwugsYfRdSGUZbkqpCqMHPr\/ayFmis5PiC7cJmq3b47Lj3CRp\/+JjVDp\/4aApP+Vx6\/iK6dPni6CPJtSmotzL1NtQ+rG4CRt91lHeUv0xIzb2GGnxzAjKWaKxkuLJXsA2brRIoDxz5Pu2eeNJ5FEUfSblw6enUe+m5dV1Ai3orU29D7cPqJmAefvhh+sAHPkDXXXfd7P1EeaHLugspL22Rz0MNfhEmPtKisfJBMdkH2FaXrT7Nw6UYu\/eJSKDYHOgWH0Xhny87fzFdsORV1Nr8atFRlCLkUW+L0EtPG2ofVjcBg7uQZCpqlbyisZKLFtiWm63a0cPTMbfefZi+feT7zgJFH0X5pXNPo\/\/0xrOiwlfxzBTUW5l6CwHjmWv84DpT962trZR0G7Vpehe7UIPvwsJnGjRWPmnO9QW29WfLC2Vf+5JAedCDQIlEyfLF9D9+fSkd+e6PvCyalaPk5hn11o1bXqpQ+7C6jcDkAS\/T56EGv96M0VjJRQBsZdnyfT38vPDiT+iWLx+mQ88cLzyCcnLUZDGdf1YD\/covnBH5l1wwK0fI3TPqrTu7rJSh9mEQMAb1JdTgGxRd1ASNlRxesC3GVk3xfOFbT9OBx56PnNmcLBt\/u342yq+9rpEuXnZGtFi2itM8xchmp0a9laEbah8GAWNQX0INvkHRRU3QWMnhBdt0tkqcNC16Jd38pUPRYW2uC2TVW6KRksWnRlNGV7SeTa9f8qrZDCjxIhftcDyj3srEMtQ+DALGoL6EGnyDoouaoLGSw7tQ2artxfzv1x5+lr7+0n08PkZPWKC8qfnVdOGZ\/0FLlpyz4KZ35Grry54Xar2VZhtqHwYBY1BzQg2+QdFFTdBYyeENjW18a\/G3nvge\/eP\/e6rw1A47UCMkvP6kedEro3Uo+u\/jIyihsZWrhfaewdaemUmKUPswCBiD6IcafIOii5qgsZLDWxW2SpioaR0m8twLL9Jn\/+2pQmef6FM7kRhZfCq98dxX02+\/8TWz4sR1aqcqbOVql5xnsJVhG2ofVioBs2vXLtq0aVMUQb7A8dChQzQ+Pk6Dg4PU0NCQGdn4XUobNmyg\/v7+1DR8KN769eujz3lr9sjICDU2Nibahxp8ma+KuVc0VuasbC3LxFY\/8+QL33qGDkw\/72XdiRolYXGy7DUN1PHLZ1HDz\/y0+PbiMrG1rRdltwdbmQiF2oeVRsDwnUgzMzPU19dHV111VSQ+WFgMDAxQU1NTphjhkHN6fjidOmOmq6uLOjs759UIddv1li1bolOAWThlCaVQgy\/zVTH3isbKnJWtZa3Y6mtO7j34Xbr7waNRVousOdFHT1icrPj5V9HysxuiA9rUiI0SL7ZcfNjXiq2PvFbNB9jKRCzUPqwUAkY\/lXfFihXU29sbCREWF+r6gKwRkqSQ64Im\/jkLloMHD+aKIpUu1ODLfFXMvaKxMmdla1mUrb7m5ASdoM9\/82n65uPf8y5OeNfOey9rpu\/98MfiIye2DNPsi7L1lY8Q\/YCtTFRD7cOCFDBZ1xSoqab29vbE0Zmk6hNq8GW+KuZe0ViZs7K1TGMbXwx75LkfRqMmRe7a0fOm1pXwyMnSM0+ljl8+O7p\/J6QzT1BvbWujuT3YmrOysQy1DyuFgOFAqGkcfQpJjcakTQWljbxs27aN0m64VgJm7dq1dPvtt9PExATWwNh8EzzaorHyCPMlV2q9yRNHnqBHf9BA90w+63XUhJ2xOHlD02n0m7\/0Gjrlp14RHcamppJcF8b6JyHnEfUWbOUIyHiGgJHhOservrBWfbB582bjkRLdmVpTE18ArATM4cOHZxfuptkqfxx8\/dm7d28NaIT\/iunpaWpubg6\/oJ5KuP\/xFyJPP\/7JCfrcg9+nmedepCeefzH6t+jTdPopkYtzXn0KvWnJK6n9Fxqi\/1eP+rzoe0JIj3orF0Ww9cN2zZo18xxNTU35cV4iL6UZgfHNhBfq8mjO8PAwtbS0zLpPmkJKs9UFTIjB983c1h\/+kj1JjC\/942fJGT9Lt979GE099YOTv3\/pEDZbrrq9PqXzS02n0eoVi+kN55xWisWwRcpVz7Sot3L0wVaGLUZgZLiKec1a\/MsjLsuWLZsd2WEBc9NNN9HWrVsTt1KHGnwx+IaOQ2+s1LQK4xgdf5z2H3ouIlP02HqFd1acNDbQm5pOow1vbZ4VJo899hhd2vqycDcMCcwMCIRebw0QiJmArQzaUPuwUozAqEW3vB4l68k620XfdaRGWdK2X8fFTdaOJc5PqMGX+aqYe61aY6UfuqZK+ekDT9KXHnhGRpi8tBB24+XLaPrZH1rt0qkaW\/NaU39LsJWLAdjKsA21DyuFgOGQ8SLenTt3zjlQTj\/PZd26dZlnwsQPstMX8arPuru7o63Z\/OjrbdIW\/KqqFGrwZb4q5l7L0lglCZNvvHSmia\/dOUxFn87h7cN\/0NZEL\/74hJUwMaVbFram+a2SHdjKRQtsZdiG2oeVQsBkbXvWR0seeuih6MC6O++8UybKKV5DDX5NISa8TLqxih9TzwLi3x7\/Hn3um36OqdeLpN9GfOHS0+n1P\/8qEWFiGjNptqb5CNEObOWiCrYybEPtwyBgDOpLqME3KLqoSdHGSr8\/h3fm\/P3+I97OM1EF10dN3v66Rrp42RnRR5FgaTxVlE8R50XZFnl36GnBVi7CYCvDNtQ+rBQChkOWN4XEVwKos2I+9rGPyUQZIzA15Zp12JpaAHvmaT9DH\/\/KY3TomePeFr\/Gp3MuWb6I3qLdQFxmYWIaIHQEpqTs7cB2YDtJAAAgAElEQVTWnplpCrA1JWVnBwFjx8vJOukcGL7UUd1XdMstt9Do6OicbdFOL7JMFGrwLTF4Mdcv9vvrrzxI\/\/rETyK\/3nfmLD6VLvqF06mn\/dw5h6yFIE5MAoGOwISSmw3YunEzSQW2JpTsbULtw0ozAmMfktqlCDX4EgSVQPmPH\/+EPnX\/k15vHY5GTl46BfZ9q1+7IIWJaczQEZiSsrcDW3tmpinA1pSUnV2ofRgEjEE9CDX4BkWfZ6LfPvx\/7vtOdOha0QPX9AWw77qkiX70oszOHJfyVjUNOgK5yIEt2MoRkPEcah9WGgHDh8n19PTQzMzMvAi2trbO2V4tE+J0r6EGP63E+oV\/Dxz5Pu2ZeNJZpOji5IrWs+n12sV+6AjkajLYgq0cATnPqLcybEPtw0ohYPTj\/dV5L3xmi7rMsb+\/f\/b8FpnwZnsNNfh6qfk4+0\/d\/x165Em7ERUlUFa+9nS64JxX0aXLF826zVtvgsZKrjaDLdjKEZDzjHorwzbUPqwUAiZ+Dox+1D8v7B0bG6P4pYwyYU72GmLwx+49QmPfeMJoZEUJkbVveA1d+baTa0\/4BuKiDxqrogTT04Mt2MoRkPOMeivDNsQ+jEmVUsDwdumDBw8Sj7xk3WkkE+r5XqsefLWwduzeJ4iFS9bDYmV1y2J650VLxM86QWMlV4PBFmzlCMh5Rr2VYVv1PiyNSikEDGdOv49IFy1f\/OIXaXx8HCMwDvWahcvWLx2kv933RGJqNf1za\/cF0ed5Uz4OWchMgsbKN9GX\/YEt2MoRkPOMeivDFgJGhuusV30dDB9ax4Jm27ZtxBcy1uPsF724VQs+r2e5aucDs9uM9bKwSOFbi\/\/kra8Vjmi+ezRW+YxcLcDWlVx+OrDNZ+RqAbau5LLTVa0PM6VQmhEY0wzXw64KwefRli8\/eJSu\/ocH5yFi0XJr1wXiU0K2sUFjZUvM3B5szVnZWoKtLTFze7A1Z2VjWYU+zKY8yrYUAsb0MsfGxkaXMhZOU+bgq\/Ut6z5+\/5xysmj53ZVn07vbz6351JApcDRWpqTs7cDWnplpCrA1JWVvB7b2zExSlLkPM8l\/mg0EjAG9sgY\/aapIjbb42CVkgKaQCRqrQvgyE4Mt2MoRkPOMeivDtqx9WNHS1lXA8G6jTZs25ZZhw4YN0Y6kej1lCz6PuvBuoqG7Hp1FUiXhojKNxkquRoMt2MoRkPOMeivDtmx9mK9S1lXAqEJkTSH5KmgRP2UL\/sobvz7nHqDhd6yg33jDmUWKWJe0aKzksIMt2MoRkPOMeivDtmx9mK9SlkLA+CqMlJ+yBJ+njPS1LlUcddFjhMZKqsYSgS3YyhGQ84x6K8O2LH2Y79JBwBgQrXfwecrorn9\/mvrvnJzN7d\/0vJF+501nGeS+vCZorORiA7ZgK0dAzjPqrQzbevdhMqWq40m8atpoYmIit2wL+TJHFi+7\/vUIbf6nk+tdqj7qghGY3OruxQAdgReMiU7AFmzlCMh4hoCR4VoJr\/UKfnyxLouX3e+7sLTbom2DiY7Alpi5Pdias7K1BFtbYub2YGvOysayXn2YTR5dbIOZQlIn+e7ZsyfiYLpzaXJykvr6+mh4eJhaWloSGdYr+Lff8zj13\/HQ7MhLSOKFC4XGyuUra5YGbM04uViBrQs1szRga8bJ1qpefZhtPm3tSyVgkrZVb968mfhqgbxHv0tJTU91dXVlplWiZ\/\/+\/ZnXFdQj+LxN+sqxB4IVLxAweTW62OfoCIrxy0oNtmArR0DGcz36MJmSzPVaGgHD4mXnzp00MjJC6sRdUyGSBEoXNGkg1aWR\/HmZRmAWgniBgJH9eqOTleMLtmArR0DGMwSMDNfIq++rBEzOlWGb66+\/nrq7u6OLI8siYHjdC5\/zop4DH7okmDUv8SqEjkDuSwW2YCtHQM4z6q0MWwgYGa7eBYy6xbqjo4MGBwepoaEhMec84sPPqlWrjNbA6E727t0rQmPmuRfpT7\/0NO1\/\/IXI\/1+9YwlddO6pIu8qg9Pp6Wlqbm4uQ1aCywPYyoUUbMFWjoAfz2vWrJnnaGpqyo\/zEnkJegppZmYmUcTwwt3t27fTtddeS9wYlWUR79BdB2evB7hs+SLafeWFJaoq\/rOCv7b8M1UewRZs5QjIeUa9lWGLERgZrnO8FlnEG89e1u4iHqVZvXo1tbW1UVl2IelTR6Ftl06rOmis5L5UYAu2cgTkPKPeyrCFgJHhKuZVLdDVFwXzy7IO0NuxY0ckauKPdPBZvFw19gDd88ix6NW8XboKt0kXDR4aq6IE09ODLdjKEZDzjHorw1a6D5PJdb7XUkwhFdltpIqo7zpS26Obmppyb7EuwwiMvutoIUwdYZoj\/4tZ1AIdQVGCEIdyBMG21mwhYISJx6eP0kZD0rIRP8hOX8SrPuMdR\/ERlnoLmIU4dQQBI\/xlwiGBooAhDuXwgq0MWwgYGa6JXtVOIv6QR1FGR0dTT8mtRbYkg\/+Rz07R\/957KCpG\/9rzqH\/tsloUqRTvQGMlFwawBVs5AnKeUW9l2Er2YTI5NvNaiimkrKyymOH1LPG1LGbF82MlFfz46Auf+bKQHjRWctEGW7CVIyDnGfVWhq1UHyaTW3OvpRQw+ghMvW+iZpRSwf\/gpx+mT\/zzY1G0buu+gLovXmIeuQAs0VjJBRFswVaOgJxn1FsZtlJ9mExuzb2WRsCUbdpIRygV\/Mar745ew9umF9roC5cbjZX5F9XWEmxtiZnbg605K1tLsLUlZmYv1YeZvV3OqhQCxuTofzkE+Z4lgj\/8hYM0+E+PLtjRFwiY\/HpXxAIdQRF62WnBFmzlCMh4lujDZHJq57UUAsYuy7W39h38hb72RUUQHYFcXQZbsJUjIOcZ9VaGre8+TCaX9l4hYAyY+Q6+LmAWyqF1SZjRWBlUPkcTsHUEZ5AMbA0gOZqArSO4nGS++zCZXNp7hYAxYOYz+Cxe1n38fuJ\/F+raF4zAGFS6giboCAoCzEgOtmArR0DGs88+TCaHbl4hYAy4+Qy+PvryZ7+3gnovPdcgB2GaoCOQiyvYgq0cATnPqLcybH32YTI5dPNaCgGTtYg37U4jt+K6pfIZ\/HW33b\/g7jxKo47Gyq0+mqQCWxNKbjZg68bNJBXYmlCyt\/HZh9m\/XS4FBIwBW1\/B10dfFtKdRxAwBpXMswk6As9ANXdgC7ZyBGQ8++rDZHLn7rWuAiZ+\/1FaMTZs2JB7KaM7gvyUvoI\/dNdBGrrr5Nbphbx4VxFHR5Bf91wtwNaVXH46sM1n5GoBtq7kstP56sNkcufuta4CRmV7oZwDo6aPFvriXQgY9y+saUp0BKak7O3A1p6ZaQqwNSVlZwcBY8crKGsfwb\/n4WPR7iN++MoAvjpgoT9orORqANiCrRwBOc+otzJsffRhMjkr5rUUIzDFiiCf2kfw\/\/QfH6G\/+PLhKLOYPjoZMzRWcnUXbMFWjoCcZ9RbGbY++jCZnBXzWjcBo08brVixgnp7e2liYiKxNPW+0NFH8Bf6vUdJgUVjVezLm5UabMFWjoCcZ9RbGbY++jCZnBXzWjcBUyzbtU1dNPj69NHW319BPe0L9+wXPXJorOTqMdiCrRwBOc+otzJsi\/ZhMrkq7hUCxoBh0eDf8Nkp+vO9h6I3YfroZeBorAwqn6MJ2DqCM0gGtgaQHE3A1hFcTrKifZhMrop7LYWAUdNJoU4hYfdRckVFY1X8C5zmAWzBVo6AnGfUWxm2EDAyXDO9srC55ppr6IMf\/CC1tLTUIQcnX1kk+Pr00fvfvpSu71het3KU7cVorOQiArZgK0dAzjPqrQzbIn2YTI78eC3FCExWUfgqgbGxMRocHKSGhoZU0+PHj9PAwADt2bMnssk6\/C4+4tPR0ZHpv0jwcXhdenTRWPn5Eid5AVuwlSMg5xn1VoZtkT5MJkd+vFZCwAwNDdHIyAg1Njamlppt+Onv7yclULq6uqizs3NOGiV02tvbo8\/Uz01NTamn\/RYJPqaPIGD8fFXtvKAjsONlYw22NrTsbMHWjpepdZE+zPQd9bArvYBhYTIzM5M7AhOHpwuaPLB8pcH4+HjqO1yDr9999NaWxfTpP1mZl5UF9TkaK7lwgy3YyhGQ84x6K8PWtQ+TyY0\/r6UQMFmLeHlkZHR01GoNjO3VBFICRl\/\/wifv8gm8eF4mgMZKrjaALdjKEZDzjHorwxYCRobrrNe0qR011WP6eh552bZtG+Wta1H+sqablI1r8NX0EfvB9un5EURjZVqr7e3A1p6ZaQqwNSVlbwe29sxMUrj2YSa+62lTihEYBpA0VWQiLtLgmUw9KdHEPrIWCXPw9Wfv3r1GMevYPk0zz71ITaefQnve1WyUZiEZTU9PU3MzuEjEHGwlqJ70CbZgK0fAj+c1a9bMczQ1NeXHeYm8lELAZE35mO5CijOdnJykvr4+Gh4eTpx+MhUv7NdFverrX3B5Y3KNx19bci0B2IKtHAE5z6i3Mmxd+jCZnPj1WgkBY7ILKY6FhU9aOpOdR7o\/l+Bj+3R+RUVjlc\/I1QJsXcnlpwPbfEauFmDrSi47nUsfJpMTv15LIWDi61\/0IuYtsFW2+q6jPIFiMr1UVMDo61+O3vx2v1ELxBsaK7lAgi3YyhGQ84x6K8MWAkaG66xXHjHZuHHjnB1HPA3U09NDW7Zsoba2tswcxA+y0xfxqs+6u7sp7ebrrBuvXYKP26fzKwwaq3xGrhZg60ouPx3Y5jNytQBbV3IYgZEhZ+GVRcz69evnpNixY0eueLF4hZOprYDR17\/0rz2P+tcuc3pv6InQWMlFGGzBVo6AnGfUWxm2tn2YTC78ey3FFJL\/Yvn1aBt8\/fwXbJ9OjwUaK7\/1VPcGtmArR0DOM+qtDFvbPkwmF\/69lkLA6FM8eVNF\/hHke7QNvr7+5cCHLqGljafmv2QBWqCxkgs62IKtHAE5z6i3Mmxt+zCZXPj3WgoBY3tyrn8M2R5tg7\/yxq8TTyOxcGEBgyeZABoruZoBtmArR0DOM+qtDFvbPkwmF\/69lkLAcLFMdxv5R5Dv0Sb4+vqXy5Yvot1XXpj\/ggVqgcZKLvBgC7ZyBOQ8o97KsLXpw2RyIOO1FAIm6y4kLnbWDiEZLHO92gRfX\/+CBbzZ0UFjJVd7wRZs5QjIeUa9lWFr04fJ5EDGaykEjEzR\/Hm1CT4OsDPnjsbKnJWtJdjaEjO3B1tzVraWYGtLzMzepg8z81gOKwgYgzjYBF8t4MX6l3ywaKzyGblagK0rufx0YJvPyNUCbF3JZaez6cNkciDjtW4CRl+4m3a4nCpyVaaQsP7FrpKisbLjZWMNtja07GzB1o6XjTXY2tAyt4WAMWcVnKVp8HGAnV3o0VjZ8bKxBlsbWna2YGvHy8YabG1omdua9mHmHsthWbcRmHjx4\/chZd2PVGt0psHHAXZ2kUFjZcfLxhpsbWjZ2YKtHS8ba7C1oWVua9qHmXssh2VpBEzSBYtqmqmrq4s6OzvrRsw0+H\/3jSfo\/Tu\/HeUTJ\/DmhwuNVT4jVwuwdSWXnw5s8xm5WoCtK7nsdKZ9mMzb5byWQsBkHWTH9yONjY3R4OAgNTQ0yJHI8GwafCzgtQsPGis7XjbWYGtDy84WbO142ViDrQ0tc1vTPszcYzksKyFgeHRmZGSEGhsb60LNNPjqBmocYGcWJjRWZpxcrMDWhZpZGrA14+RiBbYu1PLTmPZh+Z7KZVEKAZO13qUMJ\/SaBB8LeO0rNhore2amKcDWlJS9HdjaMzNNAbampOzsTPowO4\/lsC6FgGEUPFW0ceNGGh0dpZaWlojO5OQk9fT00JYtW6ielzyaBB8LeO0rNBore2amKcDWlJS9HdjaMzNNAbampOzsTPowO4\/lsC6NgFEiZv369XPI7Nixo67ihTNjEnz9BF7cQG1WudFYmXFysQJbF2pmacDWjJOLFdi6UMtPY9KH5Xspn0WpBEz58JzMkUnwsYDXPnporOyZmaYAW1NS9nZga8\/MNAXYmpKyszPpw+w8lsMaAsYgDibBX3nj14nXwWABrwHQl0zQWJmzsrUEW1ti5vZga87K1hJsbYmZ2Zv0YWaeymUFAWMQj7zg4woBA4gJJmis3LiZpAJbE0puNmDrxs0kFdiaULK3yevD7D2WIwUEjEEc8oKPHUgGECFg3CA5pkJH4AjOIBnYGkByNAFbR3A5yfL6MJm3yntdkAJGbdves2dPRHjDhg3U39+fSjsv+FjA61ZR0Vi5cTNJBbYmlNxswNaNm0kqsDWhZG+T14fZeyxHigUpYPhgPH5YtJhcV5AX\/CvHHqCxe49EPrEDybxio7EyZ2VrCba2xMztwdacla0l2NoSM7PP68PMvJTPqjQCRp35MjMzM49Sa2ur6Em8uqBJClFe8LEDya1io7Fy42aSCmxNKLnZgK0bN5NUYGtCyd4mrw+z91iOFKUQMGpKp6mpKXMqRwJZ1j1M6n15wccVAm6RQWPlxs0kFdiaUHKzAVs3biapwNaEkr1NXh9m77EcKUohYExEhAQuHnnZtm0bdXR0ZF4WycHXn717987+OPPci9SxfTr6ecObF9F\/+9VFElkN0uf09DQ1NzcHWbZ6Fwps5SIAtmArR8CP5zVr1sxzNDU15cd5ibyUQsCoEZju7u66nLrLQoanrtJuvM5Sr7hCwL02468td3Z5KcE2j5D752Drzi4vJdjmEXL7HCMwbtyMU\/FdSPW6dZrX3\/T19dHw8PDsPUx6xrOCr+9A2v2+C+my8zECYxp0NFampOztwNaemWkKsDUlZW8HtvbMTFJAwJhQcrRRU0gTExOJHqQX8eaJp6zgX\/vph+kv\/\/mxKN\/YgWRXAdBY2fGysQZbG1p2tmBrx8vGGmxtaJnbQsCYsyq9pb7ryGQBcVbwsQPJPdxorNzZ5aUE2zxC7p+DrTu7vJRgm0fI7XMIGDdupUwVP8jOZBFv2gIo3IHkHmI0Vu7s8lKCbR4h98\/B1p1dXkqwzSPk9jkEjBs3q1S7du2iTZs2RWl27NhBhw4dovHx8cwdQlYvcDTOCj62UDtCJSI0Vu7s8lKCbR4h98\/B1p1dXkqwzSPk9jkEjBs341RqJxAvpr3qqqui82B47cvAwADV43wYPeNpwdd3IPWvPY\/61y4zLi8MIWAk6wA6Ajm6YAu2cgRkPEPAyHCNvOrnwKxYsYJ6e3sjAdPW1kZ5C2wFszXr2kTAYAeSfSTQEdgzM00Btqak7O3A1p6ZaQqwNSVlZwcBY8fLyhoCxgpXMMZorORCCbZgK0dAzjPqrQxbCBgZrrNeef0Lr3fRp5DUaExXVxd1dnYK5yDdfVrw1Q6kaBTp5rfXLX9VfTEaK7nIgS3YyhGQ84x6K8MWAkaG6xyvPF20fv36Ob\/bvHlzXcULZyZPwCxtPDU6AwaPHQE0Vna8bKzB1oaWnS3Y2vGysQZbG1rmthAw5qyCs0wLPnYgFQs1Gqti\/LJSgy3YyhGQ84x6K8MWAkaGayW8JgX\/8NEXiM+A4af74iV0W\/cFlShLmTKJxkouGmALtnIE5Dyj3sqwhYCR4TrHq34OjPqAz4Ph3Uj1fJKCjy3UxSOCxqo4wzQPYAu2cgTkPKPeyrCFgJHhOuuVxcvOnTtpZGSEGhsbo9+r3UllXMSrj8BgC7Vb5UBj5cbNJBXYmlByswFbN24mqcDWhJK9DQSMPTPjFPo26vhoS1nPgcEt1MbhTTVEY1WcIUZg5BiCLdjWnoDMGyFgZLjOGWlRh9fpryqrgLly7AEau\/fIyfxjC7VT7YCAccJmlAhsjTA5GYGtEzajRGBrhMnaCALGGpldAhYqGzdupNHRUWppaZkjbMo4hYRbqO3im2SNxqo4Q4wSyDEEW7CtPQGZN0LAyHCdI1QmJiZy38L3I9155525dj4NkoKPW6iLE4aAKc4QnawcQ7AF29oTkHkjBIwM10p4jQdfX8B72fJFtPvKCytRjrJlEgJGLiJgC7ZyBOQ8o97KsIWAkeFaCa9ZAga3ULuHEI2VO7u8lGCbR8j9c7B1Z5eXEmzzCLl9DgHjxs0qVdI5MGW8SkA\/A4avEOCrBPDYE0BjZc\/MNAXYmpKytwNbe2amKcDWlJSdHQSMHS9r6yqdA8O7j3gXEj84A8Y61LMJ0Fi5s8tLCbZ5hNw\/B1t3dnkpwTaPkNvnEDBu3IxSVe0cGJwBYxTWXCM0VrmInA3A1hldbkKwzUXkbAC2zugyE0LAyHCNvFZNwKz7+P3E00hR3nEGjHPNQGPljC43IdjmInI2AFtndLkJwTYXkZMBBIwTNvNERaeQlAhSW7E7OjpocHCQGhoaEjOhr7fJs40HH2fAmMc1yxKNlR+OSV7AFmzlCMh5Rr2VYQsBI8N1jlfXRbzHjx+ngYEBam9vp87OTlI\/NzU1EZ\/uG3\/0031Z4HDaNFtOGw8+zoDxUxnQWPnhCAEjxxFswba2BGTeBgEjw1XMK4uh8fHxxFGY+GdZtnEBgzNg\/IUMAsYfy7gnsAVbOQJynlFvZdhCwMhwFfOaJUqSRmDU6E1ShvTg6wLm5ne+jv7okiaxMoTuGI2VXITBFmzlCMh5Rr2VYQsBI8NVxKtaD5N1h9Lk5CT19PTQzMwM7dixg+K3YOsZ04OvnwGDQ+yKhQ+NVTF+WanBFmzlCMh5Rr2VYQsBI8PVu1e1\/oUdpy3ijS8YHhoaivKRtF6Gf8\/BV8+Pll5KP1j17ujHv3rHErroXBxi5xrE6elpam5udk2OdBkEwFaueoAt2MoR8ON5zZo18xxNTU35cV4iL684ceLEiRLlp1BWTMRLfMEvv5BHY\/r6+mh4eHj2Juy0ERicAVMoRHMS468tfyzjnsAWbOUIyHlGvZVhixEYGa7evObtPFIvKipg+ARePomXH1wjUCx8aKyK8ctKDbZgK0dAzjPqrQxbCBgZrt688jQQr2fJOvtFvSxpCikrrR58nAHjLWSExsofS4zAyLEEW7CtHQGZN0HAyHD14jV+iJ1y2traSiMjI9FhdnzWS3d39+xiXRY827Zti0xtDrKDgPESssgJBIw\/luhk5ViCLdjWjoDMmyBgZLhWwqse\/Mar747yfNnyRbT7ygsrkf+yZhICRi4yYAu2cgTkPKPeyrCFgJHhWgmvKvj6GTDdFy+h27ovqET+y5pJNFZykQFbsJUjIOcZ9VaGLQSMDNdKeE0SMDgDpnjo0FgVZ5jmAWzBVo6AnGfUWxm2EDAyXCvhVQVfP8SOR194FAaPOwE0Vu7s8lKCbR4h98\/B1p1dXkqwzSPk9jkEjBu3IFIlCZjd77uQLjt\/URDlq1ch0FjJkQdbsJUjIOcZ9VaGLQSMDNdKeFXBxyF2fsOFxsovT90b2IKtHAE5z6i3MmwhYGS4VsKrCj4OsfMbLjRWfnlCwMjxBFuwrQ0BmbdAwMhwrYRXFXycAeM3XBAwfnmik5XjCbZgWxsCMm+BgJHhWgmvEDAyYYKAkeHKXsEWbOUIyHlGvZVhCwEjw7USXlXwV974deKzYHCInZ+wobHywzHJC9iCrRwBOc+otzJsIWBkuFbCqwo+TuH1Gy40Vn55YppDjifYgm1tCMi8BQJGhmslvHLwv\/Kv3yIegeEHh9j5CRsEjB+OGIGR4wi2YFtbAjJvg4CR4VoJrxz8v\/nCfbTu4\/dDwHiMGASMR5gxV2ALtnIE5Dyj3sqwhYCR4VoJr3EBc+BDl9DSxlMrkfcyZxKNlVx0wBZs5QjIeUa9lWELASPDtRJeOfgf3TVOfA4MPziF10\/Y0Fj54YhpDjmOYAu2tSUg8zYIGBmulfDKwd\/wl1+mobsehYDxGDEIGI8wMYUkBxNswbZmBGReBAEjw7USXjn4v\/XRz9LYvUei\/GIKyU\/YIGD8cMQogRxHsAXb2hKQeRsEjAzXSnjl4L\/xmk\/RPY8ci9a+sIDBU5wABExxhmkewBZs5QjIeUa9lWELASPDtRJeIWBkwoTGSoYrewVbsJUjIOcZ9VaGLQSMDNdKeOXgn\/7uv8MpvJ6jhcbKM1DNHdiCrRwBOc+otzJsIWBkuFbCKwf\/2O+ORHnFNQL+QobGyh\/LuCewBVs5AnKeUW9l2ELAyHD15vXo0aPU29tLExMTkc+Ojg4aHBykhoaGxHfs27eP1q9fH33W2tpKIyMj1NjYmGi77I1vpud+Yyj6rPviJXRb9wXe8r2QHaGxkos+2IKtHAE5z6i3MmwhYGS4evF6\/PhxGhgYoPb2durs7CT1c1NTE\/X39897x+TkJPX09NCWLVuora2Ndu3aRePj46mCRxcwOAPGS8giJ2is\/LHECIwcS7AF29oRkHkTBIwMVzGvWaKEPzt48GCiuEnK0NI3\/xZ977K+6CMefeFRGDzFCUDAFGeY5gFswVaOgJxn1FsZthAwMlzFvKYJmPhojUkGmt\/2B\/SDVe+OTDECY0LMzAaNlRknFyuwdaFmlgZszTi5WIGtC7X8NBAw+YxKY6HWw3R1dUVTSvqjBMzatWvp9ttvj9bM5K2Bafrt\/0kvvH5d5Oa0e4bpq7s+XpqyVjkj09PT1NzcXOUilDbvYCsXGrAFWzkCfjyvWbNmnqOpqSk\/zkvk5RUnTpw4UaL8FM6KEijsKGkRr\/r88OHDswt3h4aGaGZmJnUNjC5gcApv4RDNOsBfW\/5Yxj2BLdjKEZDzjHorwxYjMDJcvXrNEy\/8sqQpJF7U29fXR8PDw9TS0jIvT0t+\/0b60dJLo98fvfntXvO8kJ2hsZKLPtiCrRwBOc+otzJsIWBkuHrzmrfzSH8Rj7gsW7ZsdnqJBcxNN91EW7duTdxKffYffoJefM3rcI2At2iddITGyjNQzR3Ygq0cATnPqLcybCFgZLh685o3DaS\/iM+AYXt19gv\/Pz9JW6759xAw3sI0xxEaKxmuEIdyXMEWbGUJyHiHgJHh6sVr\/BA75VQtzuXD7PicmO7u7rfKKB8AAAzESURBVOjcF370g+zyDr17zR\/\/A\/3k516DU3i9ROtlJxAwnoFiBEYOKNiCbU0IyLwEAkaGayW8Nl59d5RPXCPgN1wQMH556t7AFmzlCMh5Rr2VYQsBI8O1El6VgME1An7DhcbKL08IGDmeYAu2tSEg8xYIGBmulfCqBEz\/2vOof+2ySuS5CpmEgJGLEtiCrRwBOc+otzJsIWBkuFbCKwSMTJjQWMlwZa9gC7ZyBOQ8o97KsIWAkeFaCa9KwOAeJL\/hQmPllyemOeR4gi3Y1oaAzFsgYGS4VsKrEjC4B8lvuCBg\/PJEJyvHE2zBtjYEZN4CASPDtRJelYDBNQJ+wwUB45cnOlk5nmALtrUhIPMWCBgZrpXwCgEjEyYIGBmu7BVswVaOgJxn1FsZthAwMlwr4VUJGNyD5DdcaKz88sQogRxPsAXb2hCQeQsEjAzXSnhlAbO08VTiKSQ8\/ghAwPhjGfcEtmArR0DOM+qtDFsIGBmulfAKASMTJjRWMlwxhSTHFWzBVpaAjHcIGBmulfDKAgbXCPgPFQSMf6bKI9iCrRwBOc+otzJsIWBkuFbCKwSMTJjQWMlwxSiBHFewBVtZAjLeIWBkuFbCKwsY3IPkP1QQMP6ZYgRGjinYgq08AZk3QMDIcK2EVxYwuAfJf6ggYPwzRScrxxRswVaegMwbIGBkuFbCKwSMTJggYGS4YppDjivYgq0sARnvEDAyXCvh9ew\/\/AR97P2\/G00j4fFHAALGH8u4J7AFWzkCcp5Rb2XYQsDIcK2E11CDX2\/4aKzkIgC2YCtHQM4z6q0M21D7sFecOHHihAyycLyGGvx6RwiNlVwEwBZs5QjIeUa9lWEbah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn1VoZtqH1YMALm6NGj1NvbSxMTE1EN6OjooMHBQWpoaMisEZOTk9TX10fDw8PU0tKSaBtq8GW+KuZe0ViZs7K1BFtbYub2YGvOytYSbG2JmdmH2ocFIWCOHz9OAwMD1N7eTp2dnaR+bmpqov7+\/tQIK7v9+\/fT6OgoBIzZd8GbVahfKm+ACjgC2wLwcpKCLdjKEZDxHGqdDULAJIV8165dND4+njkKs2\/fPhoaGoqSYwRG5ouT5TXUL1XtSc5\/I9jKRQFswVaOgIznUOvsghUwPOV0\/fXXU3d3dyRiIGBkvjgQMLXnym8MtcGqD825bwVbuSiArQzbULkGKWDUepiurq5oSilthIZ\/v2rVKqM1MDLVCl5BAARAAARAQJ7A1NSU\/Etq\/IbgBIxa18Ic0xbx8sLd7du307XXXkvT09O5AqbGMcHrQAAEQAAEQAAEcggEJWBMxAvz4Cmj1atXU1tbG5nsQkItAgEQAAEQAAEQKBeBYASM6c6j+HZrPRw7duyIRA0eEAABEAABEACBchMIRsDwqMrMzIzR2S96SDACU+4KityBAAiAAAiAQBKBIARM2qhKa2srjYyMRIfZ8TkxvOMoPsICAYMvBgiAAAiAAAhUj0AQAqZ62JFjEAABEAABEACBIgQgYIrQQ1oQAAEQAAEQAIG6EICAycDO62q2bdsWWWCBr1v95Cm6np6eaH1S3v1UOm++BiLrege33ISVyoatKnn82o2wiPgpjQ3X+PQ12onsGNiw1W3RHhSr2+p7n7SMopjn+qaGgEnhr64Z4DU0Dz30ULT1mv+\/sbGxvhGr0Nv1znLdunVz7quKFyN+9QP\/vHPnTjBPibcNW90Fc920aRNt3rw59ZDHClUx71m14Rrf+Yj1dNnhsGGrhCHfZcfrFtEeuFd1xX3Pnj3B\/SEOAZNSL9QdSfwFClW9un8lzFLGG3QWhWNjY0Y7xdAZ5P8lq9+ibsKWO4VrrrmGjh07RlmnVJtFN0wrmzrLtjfddBNt3boVf9gYVAdbtnr9RntgADjBRI1iXXTRRXT48OHocuOQjgqBgEkIetrt1uq2a7eqtPBS6aNYPHIV\/zmLCBqs7PriwpZF+cUXX0yf+cxnZm9uX3i10h9XkwtjwfdlAjZ1NmkEJu9yXrCeT+Dxxx+Pfsk7cXt7eyFgFkIlSRpx4cZ\/2bJlGHa3qADxUQGbv1hdz\/WxyF6lTW3Zquszrr766ugSU4jx5PDbcGUBc\/DgwcgR1srlf51s2LI3fepjw4YNUeeLx41AXBC6eSlfKozAZIzA6AueIGDsK69tg6XewB3DLbfcgkW8Gcht2HJH8NGPfpTe9a53UXNzc+ZaJPsoh5XChqtaT6QW7nLajRs3ot6mVAkbtmrqY8uWLdGUh83obVg10k9pIGD8cKyEF0wh+QmTzZAxxIsdcxu2bPvVr341+gsWu5CyOdtwjU8hgS3Y2n2La2cNAVM71qV4kz7igkW8biGJTxnlLTTFTgNzzjZs9e3p+hswLD+ftw3XeH1GO5Fdf23YQhyatwUmlhAwJpQCssE26uLBtNk2ieF3O942bHXPGCXI5mzDNd4pYJrDH9ukKSRMz9m1Ebo1BIw7u8qmxEF2xUOXdXCVWgTJUxtpowQ4GCw9BqZsIWDs6rENV\/0gOxy2ls\/Zhi0LwvXr10dOwTafbZYFBEwxfkgNAiAAAiAAAiAAAt4IYBeSN5RwBAIgAAIgAAIgUCsCEDC1Io33gAAIgAAIgAAIeCMAAeMNJRyBAAiAAAiAAAjUigAETK1I4z0gAAIgAAIgAALeCEDAeEMJRyAAAiAAAiAAArUiAAFTK9J4DwiAAAiAAAiAgDcCEDDeUMIRCPgn8Mgjj9DixYuJb\/M2efi8h2effZaWL19uYu5ko87saW1tpZGREeO8VeWGcVW+Wp09Uuv3OQUdiUCghAQgYEoYFGQJBJiA7cmutTisqogIKZK2ljWCBQU\/tbz9uCpsahkHvAsE8ghAwOQRwucgUCcCZRQwtnnS0VWlk4aAqVOFx2tBwJIABIwlMJiDgE8C+lH07FdNyzz00EOzx6jz79WVCvErF9Q0x5lnnkm9vb00MTERZU9d1Kju9tmzZ0\/0e5NpH\/0d+jQKX\/2wadOm2eJv3ryZOjs75+FIs1MCZt26dXTDDTdE6eLTNPrx8cqxeg+zuuaaa+itb31rlF6V5ZlnnqGenh6amZmJknz4wx+m3bt30\/DwMLW0tES\/SytTUizjAoZ\/fv7556P\/FMesizDjFxHyO5J+V0Vx57PuwxcIFCUAAVOUINKDgCOBpIsV9c4zPtqRdkMvv35wcJDYH4sYnvpoa2sjJY66urpmhUbWjd8qP8pfQ0ND1PHecsstNDo6GomBvBGYuL1+KR+LLBYaF110UZRf5X\/nzp3RWhoWIn19fXOEh+5PibSlS5fOpo+XUf381FNPRXlubm6mgYGBSCipKaG8i0OTBMy2bdtIF1LMWeeqVwEIGMcvBJKBgCUBCBhLYDAHAV8EkgSG7jtPLMT\/so8LGE4\/NjY229mzfdZt1ElTPHH7rDzl3XQdv2GY85M3raR\/rgRMXJCNj4\/PKaMuUPgdN910E23dunXOYuOsaaIkAcOjO0p0sc8sDhAwvr4h8AMC2QQgYFBDQKCOBPTplvj0TlonGZ9m6ejoSByBiU\/l6MVMmv5Je59+a3hWx523iDhJrCT9Lj6tFp8mUyNMXJ4kIaL75FEddaNxPMxp00BJAobT6ot6s4QXBEwdv1B49YIiAAGzoMKNwpaVgN5p6+tguDNVW5WVIImvS1EjEPERGE4bHznIKn+aOMma1tL9FRUw7EutZVECK2kExkbA3HfffaSmqEy3okPAlPVbgnyBwFwCEDCoESBQIgK6CFAjDCxgeL0Ir+Vob2+fs3BWFylxAZO13iWpyLWYQoqvcdHfyWIjazpITSHpAiZptEOfQuIRmI0bN86u4TEJtcQUUp6YzJtKM8k3bEBgoRGAgFloEUd5S0MgaQ2MPgqiL2pNWoyqRmTUFBIXTBc5yj8v6FXTH0nrUBQQX4t49REPfV3MqlWr5i3SjQsYPa3KK+ePF+QmCRjTRbzsQ61hyVt7VHQRr5riUzvHVDn0xcvxSggBU5qvJTJSIQIQMBUKFrIaHgHVuaktwPr0kL4FmqdULr\/88jlbpVm4XHHFFXTdddfNjjDERY0alVHbq5mg6ljTaGZtOTZdWJy03dpkDUz83Vu2bInWufDCXVV+fQSGyxBnGN9GHd9KzmnStoCrUS\/+V4m+pG3UevqkcunrjzhOK1eupAMHDkQiKi40VRnio1Ph1XaUCAT8EoCA8csT3kAABOpMgAVF0s4j02yZrIEx9WVqhxEYU1KwA4GXCUDAoDaAAAhUlkB8nY8abdHPfbEtHASMLTHYg0B9CEDA1Ic73goCIOCJQPx04qxTck1eGb9c8Y477oiSSd2NhMscTaICGxCYT+D\/A1VDQIPxAPRSAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:19da337d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ9431V97z9ubJKJ0AZhJcSurKSK06WU4UJAq+tYt92l7Lr5JOm9m4vR203Qey\/0SVJR7hCkSday6wBnxxOzbs+adruCtlOHWtHJjZ2s0Nw5EQKhLSFU\/pQKatGhvc\/nW044+eb755zzPZ\/f7\/s9eX+fh6ckv8\/5fM95fc7vnHfO31ecOHHiBOEBARAAARAAARAAgQoReAUETIWihayCAAiAAAiAAAhEBCBgUBFAAARAAARAAAQqRwACpnIhQ4ZBAARAAARAAAQgYFAHQAAEQAAEQAAEKkcAAqZyIUOGQQAEQAAEQAAEIGBQB0AABEAABEAABCpHAAKmciFDhkEABEAABEAABCBgUAdAwBOBo0ePUm9vb+RtZGSEGhsbPXme72ZycpJ6enrooosuosHBQWpoaBB7l+64lmU0KZDi8P73v586OztNkpTaZmhoiLZt2xblccOGDdTf32+V3127dtGmTZto8+bNpeTB5du3b5\/498MKGowrSwACprKhQ8bLRqCWnXuagOEOYvXq1dTW1iaCJ62M0u9NK4xLh8gd6Fe\/+lUrceCSxjYA\/I7169fPJgtRwIQmOG1jDHu\/BCBg\/PKENxCoC4Hjx4\/TwMAA7dmzh3bs2FEzAcMjP7V4bxJU1Rl2dHQYixE1QmEjDlzSuFQCJWBs8hZ\/T9lHYFQ9PXz4MEZhXCoJ0swhAAGDClFKAvpQOmdQHxLXRx9WrlxJN9xwQ1QG7sj06RQ1WjAxMTHvc93HFVdcQe95z3sim6amJhodHaWWlpZELrpQiNvHRyf4czWltGbNGrr55puptbU1arj1jj\/PD09Fqffu378\/yh8\/agrpuuuuo4985COReFFPnAX\/XjHVBY7qNHV71QkqX3qHqpfx1ltvpeHh4cT3Tk9PR\/mbmZmZzZMeQ50jM7\/tttvok5\/8JKnyMf84a8VOTc0lddYqrvp7VXnj5VKxbm5unhVhcX67d++OpmTUo9ePuL884Rivj1m+suph1kiNzoTzrPIeF0Xx75fOVvm4+uqrae\/evcTfHxU7vcz8O\/UOPbZ5XJLqYSkbIWSq9AQgYEofooWVwXinpZdeNcJJnVS842E\/LB6UeIl\/ntTBZnX+\/Fla3lRnc+aZZ85ZA6MEjJ4HFgpJgkMXMXE\/vgRM0l\/48c4k3rGlceXfpwmYvr4+uuqqq+axzxIM\/NlZZ51FTz31VCTQkkQFv1PvaON5T6sX6r333Xdfohi54447Zted6PVN76DjAibuS32eJmKy6iynOXToUKpQ0vMUFy9xkanEw1ve8hb62te+NqfxSBIhSd+vuABhm6Q88u\/Ve\/J861zKPkq0sFrcapcWAqba8Qsu96qB1v8CVY0\/F1YffeC\/slXDqHcQemOr\/+Wpd3gsEtQIgfKh3h3\/S19BTvpcb4wvv\/zyVAGj\/4Vq6ydPwPCoEz95UzlZI0Q8KvTMM8\/MY6KPGjCnFStWzCmjyRRSfHpLsVfx5NGWeNw5L7weJGlkiFmuW7cuKq8+YmOysNlkOihuE\/9ZMVFii\/Of925V95LKo37HQpfLnDaFpHNU9Ske0y9+8YuREEoaUUnzGx+FU6NOuo+sd6sRGlX\/87j4mCoLruFDgZwIQMA4YUMiKQJpf52pDoAb7lWrViXuwNFtDh48mPhXNedb98F\/9asdQ3mLcPP+ckwTCHqDzu+39eNLwOjvZjHCj95hpnUsegf+3ve+11jAJI1YJb2X8xGfIksb4WBb7ohVPnS2Se+LT6VlCZi0qbN4mqzRlCTxGy+bmp6MCyEl2tKERl791OOr+0iLa3w0R7FSAiZt6lDfYafXZfW91KfvVDuhc0matpRqT+A3bAIQMGHHt3Klq7WA0bch53UQtsKD4Sdtqzb1o3fO8c6OfevbqE1GYNhGX\/jKP\/OW3fgIVLwDtRUwcY7xUZq4cPIlYFRlTxNOvDMrScDEp6LyRmCqIGCSRvxUXOP1L20ERveR9t2AgKlcExtUhiFgggpn9QvjOoUUn+pQawrS\/ppNGvLPEzBZUz\/6qABHgf9KTRMwpn54aD4uLtTUWlzAsEgwWRwZ79z1EYr4NBx3+HlTSDw6lCcA4j5cp5D02p02qhH\/BsTFSHw0QpVZH4lT5VF1J54maQop75snPYWkxK4auUoTMEkjV4pRfAQmbdF1fPoqawopiQumkPJqCz43JQABY0oKdjUhIL2IN0sA5AmYrLwlrQ9JEzB5fni4Xa1niUM3ETCcJmkXkvIV30miHwBns4hXTSXoafi973jHO6LRoaSHOSWVz3QRL\/tUoi4unNIWuOppdBt+58c+9jG68cYb5y045jRxAcO\/S1sQrMqaJ5iTplfyRsB0jmllzBIfumD4wAc+kFq3snxwHpIW95ou4tW55I1A1qShwUuCIAABE0QYwyuE6TZqfQt03jbqpIXBNlNITDlreiJvkax+Mm+WH35PfMvthz\/8YTpw4MDsotWkERi9c0tbiKz7jq\/NSRI4ekeup+X\/VwIm6b233377nBNl+XA9fb2NyzZqXYjoHWrSFvu07dtxrvqaHPbJ3LZs2UIbN26McOgjaWo3Wdq27LzzW7K2UfO7TEcm0tau8ChckjhIG3ViRklb2JNGcdLEL\/8+fvJv2loi5cNkpDC8Fg0lkiAAASNBFT5FCeTt+BB9OZwXJpA0VRXfaZZ2Do\/+cpeD7ApnfoE60AWnEmpJO5Py8OAguzxC+NyGQJAChhs2PouCD9lKagizDjizgQfb+hCAgKkPd19vzZpCy5r6Snq\/y1UCvsqx0PwkTSExg7zDH5NEZyh3Vy20OlC28gYnYPIW96nP29vbo8vO1M\/8JbS9OK1swVwo+YGAqX6k439EcIlsxQunwd06ta0L8aldG\/HCOYXgrG28Qn9bcAKG53v5S8JP2ghMPKj8l8X4+HhNb\/UNvWKhfCAAAiAAAiAgSSAoAcN\/1V1\/\/fXU3d0diRgIGMmqA98gAAIgAAIgUD8CQQkYHknhh0+EzFoDo+NWQ9ldXV3RlBIeEAABEAABEACB8hMIRsDwXPj27dvp2muvJb6oz0TAqPUvHCb9FuN42H7xF3+x\/JFEDkEABEAABEAggQDfKn7eeecFxyYYAcNTRnzWBJ8emrcLiaNoKl7YlgXM1NRUcMGvd4HAVS4CYAu2cgTkPKPeyrANlWsQAiZpR4OqBknX29vuPAo1+DJfFXOv4GrOytYSbG2JmduDrTkrW0uwtSVmZh8q1yAETDyEeSMwPFrDp1BmTRvpPkMNvlnVl7MCV7CVIyDnGfUWbOUIyHgOtc4uCAGjRlx4d9KKFSuiG4LVseCqumQdvR5q8GW+KuZewdWcla0l2NoSM7cHW3NWtpZga0ss3\/7w0Reo7b9uopnP\/Xm+ccUsghQwvmOAL5Vvoif9Pfroo0EuLJOhZecVbO142ViDrQ0tO1uwteNlYj127xG6cuwBOnrz203MK2UDAWMQLggYA0gOJmisHKAZJgFbQ1AOZmDrAM0wCdgagrIwg4CxgBWiKQSMTFTRWMlwxeiWHFewBVtZAv69Q8D4Z1opjxAwMuGCgJHhik5WjivYgq0sAf\/eIWD8M62URwgYmXBBwMhwRScrxxVswVaWgH\/vQ3cdpKG7HsUaGP9oq+ERAkYmThAwMlzRycpxBVuwlSXg3zsEjH+mlfIIASMTLggYGa7oZOW4gi3YyhLw7x0Cxj\/TSnmEgJEJFwSMDFd0snJcwRZsZQn49w4B459ppTxCwMiECwJGhis6WTmuYAu2sgT8e4eA8c+0Uh4hYGTCBQEjwxWdrBxXsAVbWQL+vUPA+GdaKY8QMDLhgoCR4YpOVo4r2IKtLAH\/3vkUXt5KjZN4\/bOthEcIGJkwQcDIcEUnK8cVbMFWloB\/7xAw\/plWyiMEjEy4IGBkuKKTleMKtmArS8C\/dwgY\/0wr5RECRiZcEDAyXNHJynEFW7CVJeDfOwSMf6aV8ggBIxMuCBgZruhk5biCLdjKEvDvHQLGP9NKeYSAkQkXBIwMV3SyclzBFmxlCfj3vu62++meR45hEa9\/tNXwCAEjEycIGBmu6GTluIIt2MoS8O8dAsY\/00p5hICRCRcEjAxXdLJyXMEWbGUJ+PcOAeOfaaU8QsDIhAsCRoYrOlk5rmALtrIE\/HuHgPHPtFIeIWBkwgUBI8MVnawcV7AFW1kC\/r1DwPhnWhqPk5OT1NfXR8PDw9TS0pKYLwgYmXBBwMhwRScrxxVswVaWgH\/vK2\/8Oh0++gIW8fpHW1+Px48fp4GBAdq\/fz+Njo5CwNQ4HBAwcsDBFmzlCMh5Rr31z7bx6rsjp7hKwD\/bunrct28fDQ0NRXnACEztQ4HGSo452IKtHAE5z6i3ftnyyAuPwEDA+OVad29Hjx6l66+\/nrq7uyMRAwFT+5CgsZJjDrZgK0dAzjPqrV+2EDB+eZbG265du6K8rFq1Cmtg6hQVNFZy4MEWbOUIyHlGvfXL9p6Hj9G6j9+PERi\/WOvrjRfubt++na699lqanp42EjB6jvfu3VvfAgTydmbf3NwcSGnKVQywlYsH2IKtHAE\/ntesWRM5euH1V9ALr18HAeMHazm88JTR6tWrqa2tjbALqX4xwV9bcuzBFmzlCMh5Rr31y3boroM0dNejEDB+sdbPG6996e3tpYmJiXmZ2LFjRyRq4g+2UcvEC42VDFf2CrZgK0dAzjPqrV+26gyYn\/rB0\/T0J97p13kJvL3ixIkTJ0qQj7plASMwdUOPTlYQPToCObhgC7ZyBPx6VluoT3n6QXryb\/7Yr\/MSeIOAwUF2dauG6Ajk0IMt2MoRkPOMeuuPrb4D6ecm\/pam937Sn\/OSeFrwAsYkDphCMqFkb4PGyp6ZaQqwNSVlbwe29sxMU4CtKal8O13AnHbPMB3+xufzE1XMAgLGIGAQMAaQHEzQWDlAM0wCtoagHMzA1gGaYRKwNQRlYKbWvyxtPJWe++R\/oampKYNU1TKBgDGIFwSMASQHEzRWDtAMk4CtISgHM7B1gGaYBGwNQeWY6aMvly1fRN\/c+nsQMH7QVs8LBIxMzNBYyXBlr2ALtnIE5Dyj3vphO3bvEbpy7IHI2W3dF9C1ne0QMH7QVs8LBIxMzNBYyXCFgJHjCrZgK0uguHcefblq7AG655FjxNNHBz50CYXah2EKyaC+hBp8g6KLmkDAyOEFW7CVIyDnGfW2OFt99KXvN5bRwG+eBwFTHGt1PUDAyMQOjZUMV4wSyHEFW7CVJVDMO4++8N1H\/C+Pvux+34XRv6H2YRiBMagvoQbfoOiiJhAwcnjBFmzlCMh5Rr0txvavvjZNA3dORk76155H\/WuXRf8fah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn11p2tPnWk1r4ob6H2YRAwBvUl1OAbFF3UBI2VHF6wBVs5AnKeUW\/d2Orbplm83Np1AV12\/qJZZ6H2YRAwBvUl1OAbFF3UBI2VHF6wBVs5AnKeUW\/t2erihVPzuhddvGAKyZ5pUCkgYGTCicZKhit7BVuwlSMg5xn11o5tXLz8919bSv\/rd5bPcxJqH4YRGIP6EmrwDYouaoLGSg4v2IKtHAE5z6i35mzvefhYtONIPe+6pIn+\/J2vS3QQah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn1Np8tj7r8w33foY9+7uW7je7441Z624rG1MSh9mEQMPn1JdgtaAZFFzVBYyWHF2zBVo6AnGfU22y2POpy1c4HonNe+OEFu7xduvviJZkJIWDk6mzpPYca\/HqDR2MlFwGwBVs5AnKeUW\/T2b5\/57fp777xxKyBflBdXkRC7cMwApMX+YAPATIouqgJGis5vGALtnIE5Dyj3s5lyyMt\/zz5LH1g17fnfLD9j95IHb98lnEgIGCMUYVnGGrw6x0pNFZyEQBbsJUjIOcZ9fYkWxYun\/\/3p2nTS6fqKuLxA+pMIxFqH4YRGIMaEGrwDYouaoLGSg4v2IKtHAE5z6i3RFft\/Dbt0KaKmHbS4XQ2UQi1D4OAMagFoQbfoOiiJmis5PCCLdjKEZDzvFDrLS\/OHb7rUbrnkWNz4BYVLspZqH0YBIzBdzHU4BsUXdRkoTZWolBfcg62cpTBFmx9EGDRcueBJ2l0\/PF57nwJFwgYH5GqgY\/jx4\/TwMAA7dmzJ3rbhg0bqL+\/P\/XNu3btok2bNkWfd3R00ODgIDU0NCTaQ8DIBBAdgQxX9gq2YCtHQM5z6PWW17b8\/f4jdNPnH00ULZcuXxRti2YB4\/MJtQ8LZgRmaGgoijeLlqNHj1Jvby91dXVRZ2fnvHqwb98+YvuRkZFItLDwaWpqShU8oQbf5xfExVfojZULE19pwNYXyfl+wBZsbQjwSMtd33qabvvKY4nJfI+2JL0k1D4sGAETD5ouaOKf8ejL+Pj47KhL\/Oe4fajBt\/kSStiiI5CgetIn2IKtHAE5z6HU27Q1LYqcEi38r+\/RFggYufpZE89qBIZHY9ra2oxGYNrb2xNHazgxBIxM2EJprGToFPMKtsX4ZaUGW7CNE+Cpob\/9lxn6l6nvzluIq4uWv+h8PS07s6EmokXPY6h9WHAjMDzysm3bttx1LZOTk9TT00MzMzO0Y8eORKGjKgAHX3\/27t0r9w1eQJ6np6epubl5AZW4dkUFWznWYLuw2c4892IE4PHnXqTbv3GM9j9+8lj\/+NN0+il0zqtPoQ2\/uij6l3+u1bNmzZp5r5qaevnupFrlQ\/o9wQkYBYyFDIuTpMW5PGW0c+fOaA1MY2NjtB6Gn7RFv6GqV+nKlecff8nmEXL\/HGzd2eWlBNs8Qu6fl5WtWscy8djzqSMsXGqeDur6lSV02fmL6bLzF7mD8Jwy1D4sWAHDIyx9fX00PDxMLS0ts9VB7VbSp4zSbFWiUIPv+Tti7a6sjZV1QUqYAGzlggK24bNlwfKp+79Ddz94dPbixKRSs2C5bPli6rqYRUt5BEs8r6H2YcEKGH2nEY+yqAcCRq7xsfWMjsCWmLk92JqzsrUEW1ti5vb1YMti5cnnf0R\/Pf545uiKGmHhrc7dF58TjbbUYgGuOb10SwgYHxQ1H2qh7cTEhJXn1tZWuvPOO+el0aeBlEhJ2xqdNIWUNt3ELwo1+FbgBYzr0VgJFKOULsFWLixgW122LFae+t6PaPT\/5osVJVj+88qzqaf93KjQVREsGIGRq6OR57ydQkmvV6MqSQImfpCdfjid+qy7u3t2sa5a7MvvwUF2wsFOcY+OQI472IKtHAE5zz7rLYuVZ77\/Ixq5x1ysLF18KvW9dJBcVcVKUnRC\/SO8blNIvgWM3FcKIzBSbH02VlJ5rKpfsJWLHNiWhy1vX2ahwf9u\/dJBevSp47nTQGokhcXKe97STI0\/9zOVmg5yoQ8B40ItkDShBr\/e4UFHIBcBsAVbOQJynvPqLY+qPPid79NnDjxpJFR0sfLrF5xJq5aeXurFtlJkQ+3D6j4Cw2tg8u4tkgqqqd9Qg29afim7vMZK6r0LwS\/YykUZbGXZ\/vQZ50QjKvce\/O7JXUDPvpC5E0jPDY\/GqEW2s+LF871CcqWX8xxqH1Y3AaNCpa9F4UW3o6Ojc7Y9y4XU3HOowTcnIGOJjkCGK3sFW7CVI+DHM4sUfr784FG6477vWAsVngJ62+sa6c3Lzgh+Cqgo8VD7sLoLGBWY+K6kMo3KhBr8ol+KounRyRYlmJ4ebMFWjoCdZ5724edz33yKvvn494ynftQICv+rRlXKfNaKHZXaWofah5VGwOjh1I\/5L8OoTKjBr+1XaP7b0MnKRQBswVaOwHzPPJrC\/6nzVNjinkdOCheTR+34af35U+iG33tT5KtK56yYlLGeNqH2YaUUMHqg0w6kq2VlCDX4tWSY9C50snIRAFuw9U1AiZSpp39A\/\/Lod+mxoy9YiRQ1osJTP+3nL4pOsFXCRf2Leus7aif9hdqHlVLA6CMwDD\/vskWZkL\/sNdTgS3PL84\/GKo+Q++dg684uL2XobHnK5+zTf5b+4suH6fAzZtuSdWZKjPC0zzWXL6OZYz80Hk0JnW1e3ZL6PNQ+rDQCJusgOqmgmvoNNfim5ZeyQ2MlRRaLeOXIVputmpphkfLsD\/6D\/unfn3YaSdFHU\/6grYnOOeOV80ZTXGKANsGFWn6aUPuwugsYfRdSGUZbkqpCqMHPr\/ayFmis5PiC7cJmq3b47Lj3CRp\/+JjVDp\/4aApP+Vx6\/iK6dPni6CPJtSmotzL1NtQ+rG4CRt91lHeUv0xIzb2GGnxzAjKWaKxkuLJXsA2brRIoDxz5Pu2eeNJ5FEUfSblw6enUe+m5dV1Ai3orU29D7cPqJmAefvhh+sAHPkDXXXfd7P1EeaHLugspL22Rz0MNfhEmPtKisfJBMdkH2FaXrT7Nw6UYu\/eJSKDYHOgWH0Xhny87fzFdsORV1Nr8atFRlCLkUW+L0EtPG2ofVjcBg7uQZCpqlbyisZKLFtiWm63a0cPTMbfefZi+feT7zgJFH0X5pXNPo\/\/0xrOiwlfxzBTUW5l6CwHjmWv84DpT962trZR0G7Vpehe7UIPvwsJnGjRWPmnO9QW29WfLC2Vf+5JAedCDQIlEyfLF9D9+fSkd+e6PvCyalaPk5hn11o1bXqpQ+7C6jcDkAS\/T56EGv96M0VjJRQBsZdnyfT38vPDiT+iWLx+mQ88cLzyCcnLUZDGdf1YD\/covnBH5l1wwK0fI3TPqrTu7rJSh9mEQMAb1JdTgGxRd1ASNlRxesC3GVk3xfOFbT9OBx56PnNmcLBt\/u342yq+9rpEuXnZGtFi2itM8xchmp0a9laEbah8GAWNQX0INvkHRRU3QWMnhBdt0tkqcNC16Jd38pUPRYW2uC2TVW6KRksWnRlNGV7SeTa9f8qrZDCjxIhftcDyj3srEMtQ+DALGoL6EGnyDoouaoLGSw7tQ2artxfzv1x5+lr7+0n08PkZPWKC8qfnVdOGZ\/0FLlpyz4KZ35Grry54Xar2VZhtqHwYBY1BzQg2+QdFFTdBYyeENjW18a\/G3nvge\/eP\/e6rw1A47UCMkvP6kedEro3Uo+u\/jIyihsZWrhfaewdaemUmKUPswCBiD6IcafIOii5qgsZLDWxW2SpioaR0m8twLL9Jn\/+2pQmef6FM7kRhZfCq98dxX02+\/8TWz4sR1aqcqbOVql5xnsJVhG2ofVioBs2vXLtq0aVMUQb7A8dChQzQ+Pk6Dg4PU0NCQGdn4XUobNmyg\/v7+1DR8KN769eujz3lr9sjICDU2Nibahxp8ma+KuVc0VuasbC3LxFY\/8+QL33qGDkw\/72XdiRolYXGy7DUN1PHLZ1HDz\/y0+PbiMrG1rRdltwdbmQiF2oeVRsDwnUgzMzPU19dHV111VSQ+WFgMDAxQU1NTphjhkHN6fjidOmOmq6uLOjs759UIddv1li1bolOAWThlCaVQgy\/zVTH3isbKnJWtZa3Y6mtO7j34Xbr7waNRVousOdFHT1icrPj5V9HysxuiA9rUiI0SL7ZcfNjXiq2PvFbNB9jKRCzUPqwUAkY\/lXfFihXU29sbCREWF+r6gKwRkqSQ64Im\/jkLloMHD+aKIpUu1ODLfFXMvaKxMmdla1mUrb7m5ASdoM9\/82n65uPf8y5OeNfOey9rpu\/98MfiIye2DNPsi7L1lY8Q\/YCtTFRD7cOCFDBZ1xSoqab29vbE0Zmk6hNq8GW+KuZe0ViZs7K1TGMbXwx75LkfRqMmRe7a0fOm1pXwyMnSM0+ljl8+O7p\/J6QzT1BvbWujuT3YmrOysQy1DyuFgOFAqGkcfQpJjcakTQWljbxs27aN0m64VgJm7dq1dPvtt9PExATWwNh8EzzaorHyCPMlV2q9yRNHnqBHf9BA90w+63XUhJ2xOHlD02n0m7\/0Gjrlp14RHcamppJcF8b6JyHnEfUWbOUIyHiGgJHhOservrBWfbB582bjkRLdmVpTE18ArATM4cOHZxfuptkqfxx8\/dm7d28NaIT\/iunpaWpubg6\/oJ5KuP\/xFyJPP\/7JCfrcg9+nmedepCeefzH6t+jTdPopkYtzXn0KvWnJK6n9Fxqi\/1eP+rzoe0JIj3orF0Ww9cN2zZo18xxNTU35cV4iL6UZgfHNhBfq8mjO8PAwtbS0zLpPmkJKs9UFTIjB983c1h\/+kj1JjC\/942fJGT9Lt979GE099YOTv3\/pEDZbrrq9PqXzS02n0eoVi+kN55xWisWwRcpVz7Sot3L0wVaGLUZgZLiKec1a\/MsjLsuWLZsd2WEBc9NNN9HWrVsTt1KHGnwx+IaOQ2+s1LQK4xgdf5z2H3ouIlP02HqFd1acNDbQm5pOow1vbZ4VJo899hhd2vqycDcMCcwMCIRebw0QiJmArQzaUPuwUozAqEW3vB4l68k620XfdaRGWdK2X8fFTdaOJc5PqMGX+aqYe61aY6UfuqZK+ekDT9KXHnhGRpi8tBB24+XLaPrZH1rt0qkaW\/NaU39LsJWLAdjKsA21DyuFgOGQ8SLenTt3zjlQTj\/PZd26dZlnwsQPstMX8arPuru7o63Z\/OjrbdIW\/KqqFGrwZb4q5l7L0lglCZNvvHSmia\/dOUxFn87h7cN\/0NZEL\/74hJUwMaVbFram+a2SHdjKRQtsZdiG2oeVQsBkbXvWR0seeuih6MC6O++8UybKKV5DDX5NISa8TLqxih9TzwLi3x7\/Hn3um36OqdeLpN9GfOHS0+n1P\/8qEWFiGjNptqb5CNEObOWiCrYybEPtwyBgDOpLqME3KLqoSdHGSr8\/h3fm\/P3+I97OM1EF10dN3v66Rrp42RnRR5FgaTxVlE8R50XZFnl36GnBVi7CYCvDNtQ+rBQChkOWN4XEVwKos2I+9rGPyUQZIzA15Zp12JpaAHvmaT9DH\/\/KY3TomePeFr\/Gp3MuWb6I3qLdQFxmYWIaIHQEpqTs7cB2YDtJAAAgAElEQVTWnplpCrA1JWVnBwFjx8vJOukcGL7UUd1XdMstt9Do6OicbdFOL7JMFGrwLTF4Mdcv9vvrrzxI\/\/rETyK\/3nfmLD6VLvqF06mn\/dw5h6yFIE5MAoGOwISSmw3YunEzSQW2JpTsbULtw0ozAmMfktqlCDX4EgSVQPmPH\/+EPnX\/k15vHY5GTl46BfZ9q1+7IIWJaczQEZiSsrcDW3tmpinA1pSUnV2ofRgEjEE9CDX4BkWfZ6LfPvx\/7vtOdOha0QPX9AWw77qkiX70oszOHJfyVjUNOgK5yIEt2MoRkPEcah9WGgHDh8n19PTQzMzMvAi2trbO2V4tE+J0r6EGP63E+oV\/Dxz5Pu2ZeNJZpOji5IrWs+n12sV+6AjkajLYgq0cATnPqLcybEPtw0ohYPTj\/dV5L3xmi7rMsb+\/f\/b8FpnwZnsNNfh6qfk4+0\/d\/x165Em7ERUlUFa+9nS64JxX0aXLF826zVtvgsZKrjaDLdjKEZDzjHorwzbUPqwUAiZ+Dox+1D8v7B0bG6P4pYwyYU72GmLwx+49QmPfeMJoZEUJkbVveA1d+baTa0\/4BuKiDxqrogTT04Mt2MoRkPOMeivDNsQ+jEmVUsDwdumDBw8Sj7xk3WkkE+r5XqsefLWwduzeJ4iFS9bDYmV1y2J650VLxM86QWMlV4PBFmzlCMh5Rr2VYVv1PiyNSikEDGdOv49IFy1f\/OIXaXx8HCMwDvWahcvWLx2kv933RGJqNf1za\/cF0ed5Uz4OWchMgsbKN9GX\/YEt2MoRkPOMeivDFgJGhuusV30dDB9ax4Jm27ZtxBcy1uPsF724VQs+r2e5aucDs9uM9bKwSOFbi\/\/kra8Vjmi+ezRW+YxcLcDWlVx+OrDNZ+RqAbau5LLTVa0PM6VQmhEY0wzXw64KwefRli8\/eJSu\/ocH5yFi0XJr1wXiU0K2sUFjZUvM3B5szVnZWoKtLTFze7A1Z2VjWYU+zKY8yrYUAsb0MsfGxkaXMhZOU+bgq\/Ut6z5+\/5xysmj53ZVn07vbz6351JApcDRWpqTs7cDWnplpCrA1JWVvB7b2zExSlLkPM8l\/mg0EjAG9sgY\/aapIjbb42CVkgKaQCRqrQvgyE4Mt2MoRkPOMeivDtqx9WNHS1lXA8G6jTZs25ZZhw4YN0Y6kej1lCz6PuvBuoqG7Hp1FUiXhojKNxkquRoMt2MoRkPOMeivDtmx9mK9S1lXAqEJkTSH5KmgRP2UL\/sobvz7nHqDhd6yg33jDmUWKWJe0aKzksIMt2MoRkPOMeivDtmx9mK9SlkLA+CqMlJ+yBJ+njPS1LlUcddFjhMZKqsYSgS3YyhGQ84x6K8O2LH2Y79JBwBgQrXfwecrorn9\/mvrvnJzN7d\/0vJF+501nGeS+vCZorORiA7ZgK0dAzjPqrQzbevdhMqWq40m8atpoYmIit2wL+TJHFi+7\/vUIbf6nk+tdqj7qghGY3OruxQAdgReMiU7AFmzlCMh4hoCR4VoJr\/UKfnyxLouX3e+7sLTbom2DiY7Alpi5Pdias7K1BFtbYub2YGvOysayXn2YTR5dbIOZQlIn+e7ZsyfiYLpzaXJykvr6+mh4eJhaWloSGdYr+Lff8zj13\/HQ7MhLSOKFC4XGyuUra5YGbM04uViBrQs1szRga8bJ1qpefZhtPm3tSyVgkrZVb968mfhqgbxHv0tJTU91dXVlplWiZ\/\/+\/ZnXFdQj+LxN+sqxB4IVLxAweTW62OfoCIrxy0oNtmArR0DGcz36MJmSzPVaGgHD4mXnzp00MjJC6sRdUyGSBEoXNGkg1aWR\/HmZRmAWgniBgJH9eqOTleMLtmArR0DGMwSMDNfIq++rBEzOlWGb66+\/nrq7u6OLI8siYHjdC5\/zop4DH7okmDUv8SqEjkDuSwW2YCtHQM4z6q0MWwgYGa7eBYy6xbqjo4MGBwepoaEhMec84sPPqlWrjNbA6E727t0rQmPmuRfpT7\/0NO1\/\/IXI\/1+9YwlddO6pIu8qg9Pp6Wlqbm4uQ1aCywPYyoUUbMFWjoAfz2vWrJnnaGpqyo\/zEnkJegppZmYmUcTwwt3t27fTtddeS9wYlWUR79BdB2evB7hs+SLafeWFJaoq\/rOCv7b8M1UewRZs5QjIeUa9lWGLERgZrnO8FlnEG89e1u4iHqVZvXo1tbW1UVl2IelTR6Ftl06rOmis5L5UYAu2cgTkPKPeyrCFgJHhKuZVLdDVFwXzy7IO0NuxY0ckauKPdPBZvFw19gDd88ix6NW8XboKt0kXDR4aq6IE09ODLdjKEZDzjHorw1a6D5PJdb7XUkwhFdltpIqo7zpS26Obmppyb7EuwwiMvutoIUwdYZoj\/4tZ1AIdQVGCEIdyBMG21mwhYISJx6eP0kZD0rIRP8hOX8SrPuMdR\/ERlnoLmIU4dQQBI\/xlwiGBooAhDuXwgq0MWwgYGa6JXtVOIv6QR1FGR0dTT8mtRbYkg\/+Rz07R\/957KCpG\/9rzqH\/tsloUqRTvQGMlFwawBVs5AnKeUW9l2Er2YTI5NvNaiimkrKyymOH1LPG1LGbF82MlFfz46Auf+bKQHjRWctEGW7CVIyDnGfVWhq1UHyaTW3OvpRQw+ghMvW+iZpRSwf\/gpx+mT\/zzY1G0buu+gLovXmIeuQAs0VjJBRFswVaOgJxn1FsZtlJ9mExuzb2WRsCUbdpIRygV\/Mar745ew9umF9roC5cbjZX5F9XWEmxtiZnbg605K1tLsLUlZmYv1YeZvV3OqhQCxuTofzkE+Z4lgj\/8hYM0+E+PLtjRFwiY\/HpXxAIdQRF62WnBFmzlCMh4lujDZHJq57UUAsYuy7W39h38hb72RUUQHYFcXQZbsJUjIOcZ9VaGre8+TCaX9l4hYAyY+Q6+LmAWyqF1SZjRWBlUPkcTsHUEZ5AMbA0gOZqArSO4nGS++zCZXNp7hYAxYOYz+Cxe1n38fuJ\/F+raF4zAGFS6giboCAoCzEgOtmArR0DGs88+TCaHbl4hYAy4+Qy+PvryZ7+3gnovPdcgB2GaoCOQiyvYgq0cATnPqLcybH32YTI5dPNaCgGTtYg37U4jt+K6pfIZ\/HW33b\/g7jxKo47Gyq0+mqQCWxNKbjZg68bNJBXYmlCyt\/HZh9m\/XS4FBIwBW1\/B10dfFtKdRxAwBpXMswk6As9ANXdgC7ZyBGQ8++rDZHLn7rWuAiZ+\/1FaMTZs2JB7KaM7gvyUvoI\/dNdBGrrr5Nbphbx4VxFHR5Bf91wtwNaVXH46sM1n5GoBtq7kstP56sNkcufuta4CRmV7oZwDo6aPFvriXQgY9y+saUp0BKak7O3A1p6ZaQqwNSVlZwcBY8crKGsfwb\/n4WPR7iN++MoAvjpgoT9orORqANiCrRwBOc+otzJsffRhMjkr5rUUIzDFiiCf2kfw\/\/QfH6G\/+PLhKLOYPjoZMzRWcnUXbMFWjoCcZ9RbGbY++jCZnBXzWjcBo08brVixgnp7e2liYiKxNPW+0NFH8Bf6vUdJgUVjVezLm5UabMFWjoCcZ9RbGbY++jCZnBXzWjcBUyzbtU1dNPj69NHW319BPe0L9+wXPXJorOTqMdiCrRwBOc+otzJsi\/ZhMrkq7hUCxoBh0eDf8Nkp+vO9h6I3YfroZeBorAwqn6MJ2DqCM0gGtgaQHE3A1hFcTrKifZhMrop7LYWAUdNJoU4hYfdRckVFY1X8C5zmAWzBVo6AnGfUWxm2EDAyXDO9srC55ppr6IMf\/CC1tLTUIQcnX1kk+Pr00fvfvpSu71het3KU7cVorOQiArZgK0dAzjPqrQzbIn2YTI78eC3FCExWUfgqgbGxMRocHKSGhoZU0+PHj9PAwADt2bMnssk6\/C4+4tPR0ZHpv0jwcXhdenTRWPn5Eid5AVuwlSMg5xn1VoZtkT5MJkd+vFZCwAwNDdHIyAg1Njamlppt+Onv7yclULq6uqizs3NOGiV02tvbo8\/Uz01NTamn\/RYJPqaPIGD8fFXtvKAjsONlYw22NrTsbMHWjpepdZE+zPQd9bArvYBhYTIzM5M7AhOHpwuaPLB8pcH4+HjqO1yDr9999NaWxfTpP1mZl5UF9TkaK7lwgy3YyhGQ84x6K8PWtQ+TyY0\/r6UQMFmLeHlkZHR01GoNjO3VBFICRl\/\/wifv8gm8eF4mgMZKrjaALdjKEZDzjHorwxYCRobrrNe0qR011WP6eh552bZtG+Wta1H+sqablI1r8NX0EfvB9un5EURjZVqr7e3A1p6ZaQqwNSVlbwe29sxMUrj2YSa+62lTihEYBpA0VWQiLtLgmUw9KdHEPrIWCXPw9Wfv3r1GMevYPk0zz71ITaefQnve1WyUZiEZTU9PU3MzuEjEHGwlqJ70CbZgK0fAj+c1a9bMczQ1NeXHeYm8lELAZE35mO5CijOdnJykvr4+Gh4eTpx+MhUv7NdFverrX3B5Y3KNx19bci0B2IKtHAE5z6i3Mmxd+jCZnPj1WgkBY7ILKY6FhU9aOpOdR7o\/l+Bj+3R+RUVjlc\/I1QJsXcnlpwPbfEauFmDrSi47nUsfJpMTv15LIWDi61\/0IuYtsFW2+q6jPIFiMr1UVMDo61+O3vx2v1ELxBsaK7lAgi3YyhGQ84x6K8MWAkaG66xXHjHZuHHjnB1HPA3U09NDW7Zsoba2tswcxA+y0xfxqs+6u7sp7ebrrBuvXYKP26fzKwwaq3xGrhZg60ouPx3Y5jNytQBbV3IYgZEhZ+GVRcz69evnpNixY0eueLF4hZOprYDR17\/0rz2P+tcuc3pv6InQWMlFGGzBVo6AnGfUWxm2tn2YTC78ey3FFJL\/Yvn1aBt8\/fwXbJ9OjwUaK7\/1VPcGtmArR0DOM+qtDFvbPkwmF\/69lkLA6FM8eVNF\/hHke7QNvr7+5cCHLqGljafmv2QBWqCxkgs62IKtHAE5z6i3Mmxt+zCZXPj3WgoBY3tyrn8M2R5tg7\/yxq8TTyOxcGEBgyeZABoruZoBtmArR0DOM+qtDFvbPkwmF\/69lkLAcLFMdxv5R5Dv0Sb4+vqXy5Yvot1XXpj\/ggVqgcZKLvBgC7ZyBOQ8o97KsLXpw2RyIOO1FAIm6y4kLnbWDiEZLHO92gRfX\/+CBbzZ0UFjJVd7wRZs5QjIeUa9lWFr04fJ5EDGaykEjEzR\/Hm1CT4OsDPnjsbKnJWtJdjaEjO3B1tzVraWYGtLzMzepg8z81gOKwgYgzjYBF8t4MX6l3ywaKzyGblagK0rufx0YJvPyNUCbF3JZaez6cNkciDjtW4CRl+4m3a4nCpyVaaQsP7FrpKisbLjZWMNtja07GzB1o6XjTXY2tAyt4WAMWcVnKVp8HGAnV3o0VjZ8bKxBlsbWna2YGvHy8YabG1omdua9mHmHsthWbcRmHjx4\/chZd2PVGt0psHHAXZ2kUFjZcfLxhpsbWjZ2YKtHS8ba7C1oWVua9qHmXssh2VpBEzSBYtqmqmrq4s6OzvrRsw0+H\/3jSfo\/Tu\/HeUTJ\/DmhwuNVT4jVwuwdSWXnw5s8xm5WoCtK7nsdKZ9mMzb5byWQsBkHWTH9yONjY3R4OAgNTQ0yJHI8GwafCzgtQsPGis7XjbWYGtDy84WbO142ViDrQ0tc1vTPszcYzksKyFgeHRmZGSEGhsb60LNNPjqBmocYGcWJjRWZpxcrMDWhZpZGrA14+RiBbYu1PLTmPZh+Z7KZVEKAZO13qUMJ\/SaBB8LeO0rNhore2amKcDWlJS9HdjaMzNNAbampOzsTPowO4\/lsC6FgGEUPFW0ceNGGh0dpZaWlojO5OQk9fT00JYtW6ielzyaBB8LeO0rNBore2amKcDWlJS9HdjaMzNNAbampOzsTPowO4\/lsC6NgFEiZv369XPI7Nixo67ihTNjEnz9BF7cQG1WudFYmXFysQJbF2pmacDWjJOLFdi6UMtPY9KH5Xspn0WpBEz58JzMkUnwsYDXPnporOyZmaYAW1NS9nZga8\/MNAXYmpKyszPpw+w8lsMaAsYgDibBX3nj14nXwWABrwHQl0zQWJmzsrUEW1ti5vZga87K1hJsbYmZ2Zv0YWaeymUFAWMQj7zg4woBA4gJJmis3LiZpAJbE0puNmDrxs0kFdiaULK3yevD7D2WIwUEjEEc8oKPHUgGECFg3CA5pkJH4AjOIBnYGkByNAFbR3A5yfL6MJm3yntdkAJGbdves2dPRHjDhg3U39+fSjsv+FjA61ZR0Vi5cTNJBbYmlNxswNaNm0kqsDWhZG+T14fZeyxHigUpYPhgPH5YtJhcV5AX\/CvHHqCxe49EPrEDybxio7EyZ2VrCba2xMztwdacla0l2NoSM7PP68PMvJTPqjQCRp35MjMzM49Sa2ur6Em8uqBJClFe8LEDya1io7Fy42aSCmxNKLnZgK0bN5NUYGtCyd4mrw+z91iOFKUQMGpKp6mpKXMqRwJZ1j1M6n15wccVAm6RQWPlxs0kFdiaUHKzAVs3biapwNaEkr1NXh9m77EcKUohYExEhAQuHnnZtm0bdXR0ZF4WycHXn717987+OPPci9SxfTr6ecObF9F\/+9VFElkN0uf09DQ1NzcHWbZ6Fwps5SIAtmArR8CP5zVr1sxzNDU15cd5ibyUQsCoEZju7u66nLrLQoanrtJuvM5Sr7hCwL02468td3Z5KcE2j5D752Drzi4vJdjmEXL7HCMwbtyMU\/FdSPW6dZrX3\/T19dHw8PDsPUx6xrOCr+9A2v2+C+my8zECYxp0NFampOztwNaemWkKsDUlZW8HtvbMTFJAwJhQcrRRU0gTExOJHqQX8eaJp6zgX\/vph+kv\/\/mxKN\/YgWRXAdBY2fGysQZbG1p2tmBrx8vGGmxtaJnbQsCYsyq9pb7ryGQBcVbwsQPJPdxorNzZ5aUE2zxC7p+DrTu7vJRgm0fI7XMIGDdupUwVP8jOZBFv2gIo3IHkHmI0Vu7s8lKCbR4h98\/B1p1dXkqwzSPk9jkEjBs3q1S7du2iTZs2RWl27NhBhw4dovHx8cwdQlYvcDTOCj62UDtCJSI0Vu7s8lKCbR4h98\/B1p1dXkqwzSPk9jkEjBs341RqJxAvpr3qqqui82B47cvAwADV43wYPeNpwdd3IPWvPY\/61y4zLi8MIWAk6wA6Ajm6YAu2cgRkPEPAyHCNvOrnwKxYsYJ6e3sjAdPW1kZ5C2wFszXr2kTAYAeSfSTQEdgzM00Btqak7O3A1p6ZaQqwNSVlZwcBY8fLyhoCxgpXMMZorORCCbZgK0dAzjPqrQxbCBgZrrNeef0Lr3fRp5DUaExXVxd1dnYK5yDdfVrw1Q6kaBTp5rfXLX9VfTEaK7nIgS3YyhGQ84x6K8MWAkaG6xyvPF20fv36Ob\/bvHlzXcULZyZPwCxtPDU6AwaPHQE0Vna8bKzB1oaWnS3Y2vGysQZbG1rmthAw5qyCs0wLPnYgFQs1Gqti\/LJSgy3YyhGQ84x6K8MWAkaGayW8JgX\/8NEXiM+A4af74iV0W\/cFlShLmTKJxkouGmALtnIE5Dyj3sqwhYCR4TrHq34OjPqAz4Ph3Uj1fJKCjy3UxSOCxqo4wzQPYAu2cgTkPKPeyrCFgJHhOuuVxcvOnTtpZGSEGhsbo9+r3UllXMSrj8BgC7Vb5UBj5cbNJBXYmlByswFbN24mqcDWhJK9DQSMPTPjFPo26vhoS1nPgcEt1MbhTTVEY1WcIUZg5BiCLdjWnoDMGyFgZLjOGWlRh9fpryqrgLly7AEau\/fIyfxjC7VT7YCAccJmlAhsjTA5GYGtEzajRGBrhMnaCALGGpldAhYqGzdupNHRUWppaZkjbMo4hYRbqO3im2SNxqo4Q4wSyDEEW7CtPQGZN0LAyHCdI1QmJiZy38L3I9155525dj4NkoKPW6iLE4aAKc4QnawcQ7AF29oTkHkjBIwM10p4jQdfX8B72fJFtPvKCytRjrJlEgJGLiJgC7ZyBOQ8o97KsIWAkeFaCa9ZAga3ULuHEI2VO7u8lGCbR8j9c7B1Z5eXEmzzCLl9DgHjxs0qVdI5MGW8SkA\/A4avEOCrBPDYE0BjZc\/MNAXYmpKytwNbe2amKcDWlJSdHQSMHS9r6yqdA8O7j3gXEj84A8Y61LMJ0Fi5s8tLCbZ5hNw\/B1t3dnkpwTaPkNvnEDBu3IxSVe0cGJwBYxTWXCM0VrmInA3A1hldbkKwzUXkbAC2zugyE0LAyHCNvFZNwKz7+P3E00hR3nEGjHPNQGPljC43IdjmInI2AFtndLkJwTYXkZMBBIwTNvNERaeQlAhSW7E7OjpocHCQGhoaEjOhr7fJs40HH2fAmMc1yxKNlR+OSV7AFmzlCMh5Rr2VYQsBI8N1jlfXRbzHjx+ngYEBam9vp87OTlI\/NzU1EZ\/uG3\/0031Z4HDaNFtOGw8+zoDxUxnQWPnhCAEjxxFswba2BGTeBgEjw1XMK4uh8fHxxFGY+GdZtnEBgzNg\/IUMAsYfy7gnsAVbOQJynlFvZdhCwMhwFfOaJUqSRmDU6E1ShvTg6wLm5ne+jv7okiaxMoTuGI2VXITBFmzlCMh5Rr2VYQsBI8NVxKtaD5N1h9Lk5CT19PTQzMwM7dixg+K3YOsZ04OvnwGDQ+yKhQ+NVTF+WanBFmzlCMh5Rr2VYQsBI8PVu1e1\/oUdpy3ijS8YHhoaivKRtF6Gf8\/BV8+Pll5KP1j17ujHv3rHErroXBxi5xrE6elpam5udk2OdBkEwFaueoAt2MoR8ON5zZo18xxNTU35cV4iL684ceLEiRLlp1BWTMRLfMEvv5BHY\/r6+mh4eHj2Juy0ERicAVMoRHMS468tfyzjnsAWbOUIyHlGvZVhixEYGa7evObtPFIvKipg+ARePomXH1wjUCx8aKyK8ctKDbZgK0dAzjPqrQxbCBgZrt688jQQr2fJOvtFvSxpCikrrR58nAHjLWSExsofS4zAyLEEW7CtHQGZN0HAyHD14jV+iJ1y2traSiMjI9FhdnzWS3d39+xiXRY827Zti0xtDrKDgPESssgJBIw\/luhk5ViCLdjWjoDMmyBgZLhWwqse\/Mar747yfNnyRbT7ygsrkf+yZhICRi4yYAu2cgTkPKPeyrCFgJHhWgmvKvj6GTDdFy+h27ovqET+y5pJNFZykQFbsJUjIOcZ9VaGLQSMDNdKeE0SMDgDpnjo0FgVZ5jmAWzBVo6AnGfUWxm2EDAyXCvhVQVfP8SOR194FAaPOwE0Vu7s8lKCbR4h98\/B1p1dXkqwzSPk9jkEjBu3IFIlCZjd77uQLjt\/URDlq1ch0FjJkQdbsJUjIOcZ9VaGLQSMDNdKeFXBxyF2fsOFxsovT90b2IKtHAE5z6i3MmwhYGS4VsKrCj4OsfMbLjRWfnlCwMjxBFuwrQ0BmbdAwMhwrYRXFXycAeM3XBAwfnmik5XjCbZgWxsCMm+BgJHhWgmvEDAyYYKAkeHKXsEWbOUIyHlGvZVhCwEjw7USXlXwV974deKzYHCInZ+wobHywzHJC9iCrRwBOc+otzJsIWBkuFbCqwo+TuH1Gy40Vn55YppDjifYgm1tCMi8BQJGhmslvHLwv\/Kv3yIegeEHh9j5CRsEjB+OGIGR4wi2YFtbAjJvg4CR4VoJrxz8v\/nCfbTu4\/dDwHiMGASMR5gxV2ALtnIE5Dyj3sqwhYCR4VoJr3EBc+BDl9DSxlMrkfcyZxKNlVx0wBZs5QjIeUa9lWELASPDtRJeOfgf3TVOfA4MPziF10\/Y0Fj54YhpDjmOYAu2tSUg8zYIGBmulfDKwd\/wl1+mobsehYDxGDEIGI8wMYUkBxNswbZmBGReBAEjw7USXjn4v\/XRz9LYvUei\/GIKyU\/YIGD8cMQogRxHsAXb2hKQeRsEjAzXSnjl4L\/xmk\/RPY8ci9a+sIDBU5wABExxhmkewBZs5QjIeUa9lWELASPDtRJeIWBkwoTGSoYrewVbsJUjIOcZ9VaGLQSMDNdKeOXgn\/7uv8MpvJ6jhcbKM1DNHdiCrRwBOc+otzJsIWBkuFbCKwf\/2O+ORHnFNQL+QobGyh\/LuCewBVs5AnKeUW9l2ELAyHD15vXo0aPU29tLExMTkc+Ojg4aHBykhoaGxHfs27eP1q9fH33W2tpKIyMj1NjYmGi77I1vpud+Yyj6rPviJXRb9wXe8r2QHaGxkos+2IKtHAE5z6i3MmwhYGS4evF6\/PhxGhgYoPb2durs7CT1c1NTE\/X39897x+TkJPX09NCWLVuora2Ndu3aRePj46mCRxcwOAPGS8giJ2is\/LHECIwcS7AF29oRkHkTBIwMVzGvWaKEPzt48GCiuEnK0NI3\/xZ977K+6CMefeFRGDzFCUDAFGeY5gFswVaOgJxn1FsZthAwMlzFvKYJmPhojUkGmt\/2B\/SDVe+OTDECY0LMzAaNlRknFyuwdaFmlgZszTi5WIGtC7X8NBAw+YxKY6HWw3R1dUVTSvqjBMzatWvp9ttvj9bM5K2Bafrt\/0kvvH5d5Oa0e4bpq7s+XpqyVjkj09PT1NzcXOUilDbvYCsXGrAFWzkCfjyvWbNmnqOpqSk\/zkvk5RUnTpw4UaL8FM6KEijsKGkRr\/r88OHDswt3h4aGaGZmJnUNjC5gcApv4RDNOsBfW\/5Yxj2BLdjKEZDzjHorwxYjMDJcvXrNEy\/8sqQpJF7U29fXR8PDw9TS0jIvT0t+\/0b60dJLo98fvfntXvO8kJ2hsZKLPtiCrRwBOc+otzJsIWBkuHrzmrfzSH8Rj7gsW7ZsdnqJBcxNN91EW7duTdxKffYffoJefM3rcI2At2iddITGyjNQzR3Ygq0cATnPqLcybCFgZLh685o3DaS\/iM+AYXt19gv\/Pz9JW6759xAw3sI0xxEaKxmuEIdyXMEWbGUJyHiHgJHh6sVr\/BA75VQtzuXD7PicmO7u7rfKKB8AAAzESURBVOjcF370g+zyDr17zR\/\/A\/3k516DU3i9ROtlJxAwnoFiBEYOKNiCbU0IyLwEAkaGayW8Nl59d5RPXCPgN1wQMH556t7AFmzlCMh5Rr2VYQsBI8O1El6VgME1An7DhcbKL08IGDmeYAu2tSEg8xYIGBmulfCqBEz\/2vOof+2ySuS5CpmEgJGLEtiCrRwBOc+otzJsIWBkuFbCKwSMTJjQWMlwZa9gC7ZyBOQ8o97KsIWAkeFaCa9KwOAeJL\/hQmPllyemOeR4gi3Y1oaAzFsgYGS4VsKrEjC4B8lvuCBg\/PJEJyvHE2zBtjYEZN4CASPDtRJelYDBNQJ+wwUB45cnOlk5nmALtrUhIPMWCBgZrpXwCgEjEyYIGBmu7BVswVaOgJxn1FsZthAwMlwr4VUJGNyD5DdcaKz88sQogRxPsAXb2hCQeQsEjAzXSnhlAbO08VTiKSQ8\/ghAwPhjGfcEtmArR0DOM+qtDFsIGBmulfAKASMTJjRWMlwxhSTHFWzBVpaAjHcIGBmulfDKAgbXCPgPFQSMf6bKI9iCrRwBOc+otzJsIWBkuFbCKwSMTJjQWMlwxSiBHFewBVtZAjLeIWBkuFbCKwsY3IPkP1QQMP6ZYgRGjinYgq08AZk3QMDIcK2EVxYwuAfJf6ggYPwzRScrxxRswVaegMwbIGBkuFbCKwSMTJggYGS4YppDjivYgq0sARnvEDAyXCvh9ew\/\/AR97P2\/G00j4fFHAALGH8u4J7AFWzkCcp5Rb2XYQsDIcK2E11CDX2\/4aKzkIgC2YCtHQM4z6q0M21D7sFecOHHihAyycLyGGvx6RwiNlVwEwBZs5QjIeUa9lWEbah8GAWNQX0INvkHRRU3QWMnhBVuwlSMg5xn1VoZtqH1YMALm6NGj1NvbSxMTE1EN6OjooMHBQWpoaMisEZOTk9TX10fDw8PU0tKSaBtq8GW+KuZe0ViZs7K1BFtbYub2YGvOytYSbG2JmdmH2ocFIWCOHz9OAwMD1N7eTp2dnaR+bmpqov7+\/tQIK7v9+\/fT6OgoBIzZd8GbVahfKm+ACjgC2wLwcpKCLdjKEZDxHGqdDULAJIV8165dND4+njkKs2\/fPhoaGoqSYwRG5ouT5TXUL1XtSc5\/I9jKRQFswVaOgIznUOvsghUwPOV0\/fXXU3d3dyRiIGBkvjgQMLXnym8MtcGqD825bwVbuSiArQzbULkGKWDUepiurq5oSilthIZ\/v2rVKqM1MDLVCl5BAARAAARAQJ7A1NSU\/Etq\/IbgBIxa18Ic0xbx8sLd7du307XXXkvT09O5AqbGMcHrQAAEQAAEQAAEcggEJWBMxAvz4Cmj1atXU1tbG5nsQkItAgEQAAEQAAEQKBeBYASM6c6j+HZrPRw7duyIRA0eEAABEAABEACBchMIRsDwqMrMzIzR2S96SDACU+4KityBAAiAAAiAQBKBIARM2qhKa2srjYyMRIfZ8TkxvOMoPsICAYMvBgiAAAiAAAhUj0AQAqZ62JFjEAABEAABEACBIgQgYIrQQ1oQAAEQAAEQAIG6EICAycDO62q2bdsWWWCBr1v95Cm6np6eaH1S3v1UOm++BiLrege33ISVyoatKnn82o2wiPgpjQ3X+PQ12onsGNiw1W3RHhSr2+p7n7SMopjn+qaGgEnhr64Z4DU0Dz30ULT1mv+\/sbGxvhGr0Nv1znLdunVz7quKFyN+9QP\/vHPnTjBPibcNW90Fc920aRNt3rw59ZDHClUx71m14Rrf+Yj1dNnhsGGrhCHfZcfrFtEeuFd1xX3Pnj3B\/SEOAZNSL9QdSfwFClW9un8lzFLGG3QWhWNjY0Y7xdAZ5P8lq9+ibsKWO4VrrrmGjh07RlmnVJtFN0wrmzrLtjfddBNt3boVf9gYVAdbtnr9RntgADjBRI1iXXTRRXT48OHocuOQjgqBgEkIetrt1uq2a7eqtPBS6aNYPHIV\/zmLCBqs7PriwpZF+cUXX0yf+cxnZm9uX3i10h9XkwtjwfdlAjZ1NmkEJu9yXrCeT+Dxxx+Pfsk7cXt7eyFgFkIlSRpx4cZ\/2bJlGHa3qADxUQGbv1hdz\/WxyF6lTW3Zquszrr766ugSU4jx5PDbcGUBc\/DgwcgR1srlf51s2LI3fepjw4YNUeeLx41AXBC6eSlfKozAZIzA6AueIGDsK69tg6XewB3DLbfcgkW8Gcht2HJH8NGPfpTe9a53UXNzc+ZaJPsoh5XChqtaT6QW7nLajRs3ot6mVAkbtmrqY8uWLdGUh83obVg10k9pIGD8cKyEF0wh+QmTzZAxxIsdcxu2bPvVr341+gsWu5CyOdtwjU8hgS3Y2n2La2cNAVM71qV4kz7igkW8biGJTxnlLTTFTgNzzjZs9e3p+hswLD+ftw3XeH1GO5Fdf23YQhyatwUmlhAwJpQCssE26uLBtNk2ieF3O942bHXPGCXI5mzDNd4pYJrDH9ukKSRMz9m1Ebo1BIw7u8qmxEF2xUOXdXCVWgTJUxtpowQ4GCw9BqZsIWDs6rENV\/0gOxy2ls\/Zhi0LwvXr10dOwTafbZYFBEwxfkgNAiAAAiAAAiAAAt4IYBeSN5RwBAIgAAIgAAIgUCsCEDC1Io33gAAIgAAIgAAIeCMAAeMNJRyBAAiAAAiAAAjUigAETK1I4z0gAAIgAAIgAALeCEDAeEMJRyAAAiAAAiAAArUiAAFTK9J4DwiAAAiAAAiAgDcCEDDeUMIRCPgn8Mgjj9DixYuJb\/M2efi8h2effZaWL19uYu5ko87saW1tpZGREeO8VeWGcVW+Wp09Uuv3OQUdiUCghAQgYEoYFGQJBJiA7cmutTisqogIKZK2ljWCBQU\/tbz9uCpsahkHvAsE8ghAwOQRwucgUCcCZRQwtnnS0VWlk4aAqVOFx2tBwJIABIwlMJiDgE8C+lH07FdNyzz00EOzx6jz79WVCvErF9Q0x5lnnkm9vb00MTERZU9d1Kju9tmzZ0\/0e5NpH\/0d+jQKX\/2wadOm2eJv3ryZOjs75+FIs1MCZt26dXTDDTdE6eLTNPrx8cqxeg+zuuaaa+itb31rlF6V5ZlnnqGenh6amZmJknz4wx+m3bt30\/DwMLW0tES\/SytTUizjAoZ\/fv7556P\/FMesizDjFxHyO5J+V0Vx57PuwxcIFCUAAVOUINKDgCOBpIsV9c4zPtqRdkMvv35wcJDYH4sYnvpoa2sjJY66urpmhUbWjd8qP8pfQ0ND1PHecsstNDo6GomBvBGYuL1+KR+LLBYaF110UZRf5X\/nzp3RWhoWIn19fXOEh+5PibSlS5fOpo+XUf381FNPRXlubm6mgYGBSCipKaG8i0OTBMy2bdtIF1LMWeeqVwEIGMcvBJKBgCUBCBhLYDAHAV8EkgSG7jtPLMT\/so8LGE4\/NjY229mzfdZt1ElTPHH7rDzl3XQdv2GY85M3raR\/rgRMXJCNj4\/PKaMuUPgdN910E23dunXOYuOsaaIkAcOjO0p0sc8sDhAwvr4h8AMC2QQgYFBDQKCOBPTplvj0TlonGZ9m6ejoSByBiU\/l6MVMmv5Je59+a3hWx523iDhJrCT9Lj6tFp8mUyNMXJ4kIaL75FEddaNxPMxp00BJAobT6ot6s4QXBEwdv1B49YIiAAGzoMKNwpaVgN5p6+tguDNVW5WVIImvS1EjEPERGE4bHznIKn+aOMma1tL9FRUw7EutZVECK2kExkbA3HfffaSmqEy3okPAlPVbgnyBwFwCEDCoESBQIgK6CFAjDCxgeL0Ir+Vob2+fs3BWFylxAZO13iWpyLWYQoqvcdHfyWIjazpITSHpAiZptEOfQuIRmI0bN86u4TEJtcQUUp6YzJtKM8k3bEBgoRGAgFloEUd5S0MgaQ2MPgqiL2pNWoyqRmTUFBIXTBc5yj8v6FXTH0nrUBQQX4t49REPfV3MqlWr5i3SjQsYPa3KK+ePF+QmCRjTRbzsQ61hyVt7VHQRr5riUzvHVDn0xcvxSggBU5qvJTJSIQIQMBUKFrIaHgHVuaktwPr0kL4FmqdULr\/88jlbpVm4XHHFFXTdddfNjjDERY0alVHbq5mg6ljTaGZtOTZdWJy03dpkDUz83Vu2bInWufDCXVV+fQSGyxBnGN9GHd9KzmnStoCrUS\/+V4m+pG3UevqkcunrjzhOK1eupAMHDkQiKi40VRnio1Ph1XaUCAT8EoCA8csT3kAABOpMgAVF0s4j02yZrIEx9WVqhxEYU1KwA4GXCUDAoDaAAAhUlkB8nY8abdHPfbEtHASMLTHYg0B9CEDA1Ic73goCIOCJQPx04qxTck1eGb9c8Y477oiSSd2NhMscTaICGxCYT+D\/A1VDQIPxAPRSAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
