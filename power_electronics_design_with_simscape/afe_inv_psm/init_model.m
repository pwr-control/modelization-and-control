clear;
[model, options] = init_environment('afe_inv_psm');

CTRPIFF_CLIP_RELEASE = 0.001;

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
dead_time_AFE = 0e-6;
dead_time_INV = 0e-6;
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

hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, fPWM_AFE); %[output:08726187]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, fPWM_INV); %[output:64b5cd42]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, fPWM_DAB, fres_dab); %[output:1121fc83]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, fPWM_CLLC, fres_cllc); %[output:460f4ea1]

%[text] ### Sensors endscale, and quantization
adc_quantization = 1/2^11;
adc12_quantization = adc_quantization;
adc16_quantization = 1/2^15;

Imax_adc = 1049.835;
CurrentQuantization = Imax_adc/2^11;

Umax_adc = 1500;
VoltageQuantization = Umax_adc/2^11;
%[text] ## 
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


%[text] #### DClink Lstray model (partial loop inductance)
parasitic_dclink_data; %[output:80ace58c]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:5907a544]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:312bbcff]
n_sys = psm.number_of_systems;
run('n_sys_generic_1M5W_torque_curve');
torque_overload_factor = 1;

% load
b = psm.load_friction_m;
% external_load_inertia = 6*psm.Jm_m;
external_load_inertia = 1;

%[text] ### EKF BEMF observer
kalman_psm;
k_kalman = 0;
%[text] ### Motor Voltage to Udc Scaling
u_psm_scale = 2/3*hwdata.inv.udc_nom/psm.ubez;
u_im_scale = 2/3*hwdata.inv.udc_nom/im.ubez;
%[text] ## CONTROL Settings and Initialization
%[text] ### 
psm_ctrl = ctrl_pmsm_setup(ts_inv, psm.omega_bez, psm.Jm_norm);
im_ctrl = ctrl_im_setup(ts_inv, im.omega_bez, im.Jm_norm);
afe_ctrl = ctrl_afe_setup(ts_afe, grid.omega_grid_nom); %[output:09b075b4]

%[text] ### Simulation parameters: speed reference, load torque in motor mode
% rpm_sim = 3000;
rpm_sim = 17.8;
% rpm_sim = 15.2;
omega_m_sim = psm.omega_m_bez;
omega_sim = omega_m_sim*psm.number_poles/2;
tau_load_sim = psm.tau_bez/5; %N*m
b_square = 0;

%[text] ### 
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
lithium_ion_battery_1 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab);
lithium_ion_battery_2 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab);
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
%   data: {"layout":"onright"}
%---
%[output:08726187]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:64b5cd42]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:1121fc83]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:460f4ea1]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.880000e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:80ace58c]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAArwAAAGmCAYAAACeH1cjAAAAAXNSR0IArs4c6QAAIABJREFUeF7tvQ2QFtd55\/soGccodyYF2sgaw6DIy6SuWVUy1o2FYkuLUcmLdJ1rFZHsEczeGhRgYzwQEeFRtJeiJFmF2RLgsacyTGFnIMC9BRKKlFkr9pVIVMazkqVBTmGcUrS5O9hEGvMhvIJkKAMRDreeHvqdfnvej+5+++v0+XUVxftx+pzz\/J5\/d\/\/nvKdPX3PlypUrwgYBCEAAAhCAAAQgAIGCErgGw1vQzBIWBCAAAQhAAAIQgIBDAMOLECAAAQhAAAIQgAAECk0Aw1vo9BIcBCAAAQhAAAIQgACGFw1AAAIQgAAEIAABCBSaAIa30OklOAhAAAIQgAAEIAABDC8agAAEIAABCEAAAhAoNAEMb6HTS3AQgAAEIAABCEAAAhheNAABCEAAAhCAAAQgUGgCGN5Cp5fgIAABCEAAAhCAAAQwvGgAAhCAAAQgAAEIQKDQBDC8hU4vwUEAAhCAAAQgAAEIYHjRAAQgAAEIQAACEIBAoQlgeAudXoKDAAQgAAEIQAACEMDwogEIQAACEIAABCAAgUITwPAWOr0EBwEI1CIwODgo27ZtKyvS0tIie\/fulY6OjtDwTp06JZ2dnbJx40ZZvHhxqP3Xr18vw8PDNfty8OBB2bRpkxw4cEBaW1tD1U9hCEAAAjYTwPDanH1ih4DlBNTwjoyMyNDQkDQ3Nzs09LOnn346kqls1PBq+319faWsaF++8Y1vlAw4htdywRI+BCAQmQCGNzI6doQABEwnUMnwHj16VNauXSsDAwOhR3njNrzKV0d+\/UbYdO70HwIQgEDaBDC8aROnPQhAIDcEqo3w+kd9vdMNlixZUjYKq6Ouq1evdmJasGCBnDhxojSlwTXA4+Pjzvc7duyoOtWhmrH19vH73\/9+2ZQGb9taf29vr\/T09DhtnT9\/XlatWiWHDx8WnaahfZuYmHBGs7We\/v5+p5z2Tadw6Nbd3e2U0c0bp\/ZNP9d\/Wl9bW5v86Z\/+qfzRH\/2Rs7++Z5pFbmRNRyAAgQoEMLzIAgIQsJZApTm8fvPmNZxqInWO7tKlSx1j6R\/RdetTY\/vJT37SMZwLFy50ytYbOa5meL3TGH70ox+VDK8mbeXKlbJ582ZnJNo\/3aFSv2fPnl0yvGrSXQPumuMVK1Y4htzfV+3byy+\/7BjjefPmOXGpsVeTq5uXibViInAIQCDXBDC8uU4PnYMABJIkUGmEV43jI488UmbuXNOqfak14uo1wDfccINs2LBBdu7cWbrBTI1je3t7aRTWG1tYw+u\/ac0791jnI3vNdr1++xl7GaiZ9vfN+941y15GSeaMuiEAAQhEIYDhjUKNfSAAgUIQqGR4vQbuvvvum7bqgncfHfH0Tn\/wjpQqIHeqgxeWf0qE+13YKQ2uqdUpBrp99KMfdaYxeEddvatF1DLqur932sacOXOcOt15zBjeQsidICBgNQEMr9XpJ3gI2E2gnuHVOa1hRkrrjfDWol3J8LoGWqci6OoN1aY36Giv97uwI7z+KQyVpjRo390VJBjhtfu4IXoImEgAw2ti1ugzBCAQC4F6Uxr05\/wgc3jdOb3uTWSV5vC6Ztgt6w+gkuGttSyZzuf1T71w59WqAa43h9e7nq8aXDX3W7dudebweufsMqUhFqlRCQQgkDEBDG\/GCaB5CEAgOwKVblrT3vhXU6i1SoNrFnUFg\/nz5zvBrFu3zjGO\/lUaqk1n0H0qPXjCfwNdpVFcdxWGL33pS\/Jnf\/ZnpWkI3lUatJ4777xT\/uEf\/qF005r\/ARbe9p944gmnnDslgikN2WmUliEAgXgIYHjj4UgtEIAABHJNQM392NhY2ZJque4wnYMABCAQIwEMb4wwqQoCEIBAXgh4V4SoN50iL32mHxCAAASSIoDhTYos9UIAAhDIkIB3qoV2o9Z0igy7SdMQgAAEUiGA4U0FM41AAAIQgAAEIAABCGRFAMObFXnahQAEIAABCEAAAhBIhQCGNxXMNAIBCEAAAhCAAAQgkBUBDG9W5GkXAhCAAAQgAAEIQCAVAhjeVDDTCAQgAAEIQAACEIBAVgQwvFmRp10IQAACEIAABCAAgVQIYHhTwUwjEIAABCAAAQhAAAJZEcDwZkWediEAAQhAAAIQgAAEUiGA4U0FM41AAAIQgAAEIAABCGRFAMObFXnahQAEIAABCEAAAhBIhQCGNxXMNAIBCEAAAhCAAAQgkBUBDG9W5GkXAhCAAAQgAAEIQCAVAhjeVDDTCAQgAAEIQAACEIBAVgQwvFmRp10IQAACEIAABCAAgVQIYHhTwUwjEIAABCAAAQhAAAJZEcDwZkWediEAAQhAAAIQgAAEUiGA4U0FM41AAAIQgAAEIAABCGRFAMObFXnahQAEIAABCEAAAhBIhQCGNxXMNAIBCEAAAhCAAAQgkBUBDG9W5GkXAhCAAAQgAAEIQCAVAhjeVDDTCAQgAAEIQAACEIBAVgQwvFmRp10IQAACEIAABCAAgVQIYHhTwUwjEIAABCAAAQhAAAJZEcDwZkWediEAAQhAAAIQgAAEUiGA4U0FM41AAAIQgAAEIAABCGRFAMObIvnx8fEUW6MpCEAAAhCAAARMIdD6wcuiPqHp+pti63JbW1tsdZleEYY3pQyqiB955BEZHR1NqUWagQAEIAABCEAg7wRu+JXLcve\/mZDu2efk1L80yf\/5d3Nj6\/Jtt90mW7duFYyvCIY3NlnVruj111+Xrq4uR3hz5sxpuFU1zv39\/YHrq1e+1veVvgvymbeMGn63v97XcbBQmPXi8wOvVx4e5cTgES8PP89a7+M+XuppP+yxUuv4q9ZWmPj1HFHtXBLH+QMewbVd6cIVhF9QHbj1F1kf3tjU6J599gm5+cJ\/d0Z137z2o7LmL\/+h7Loex7V2ZGQEwysY3oaNZ9AKXMMbl\/D0Ivjcc8\/JunXrAnWhXvla31f6Lshn3jLe13GzUAD14vNDqlc+TR71+hIowb5CYeusVz5NHjbow8+z1vt6uQmrj7D1BSlfrUzQz8PwiFsfQeLzMg5SPmjcbr1h4vef74L0J4xGwtYXpHyaPPKuD2XxzI6vSveHz8nEod2O0W1ZtFxmdT5R8TqWt2ttGC3lrSwjvCllJO6DMKVuJ9IMLBLBWphK0UdhUplIIOgjEayFqTTP+lCDO3Foj1x885BjdGd1Pi4tix5MjH2eWSQWdI2KMbwpUUd4U6BhkZLoDG0GfRiauJS6jT5SAm1oM3nTx+Uzx+XCm4fKjO71a\/5crr15UeKE88Yi8YDrNIDhTSkDCG8K9PHjx2X37t2yceNGaWpqSikDNGMKAfRhSqay6Sf6yIa7Ka3mRR9qdCe+Ozmiq5sa3OZFy1Mxum6u8B3lqsXwpnQUI7wp0BcvXpSTJ0\/K3LlzMbwp6c+kZtCHSdlKv6\/oI33mJrWYtT5co3v22S870xbU6OrUhTiXGguaD3wHhjeoVmIth\/AwvLEKqsCVZX3BKjDaQoSGPgqRxsSCyEoffqOrN6K13PlgJkaXEd7K8mKEN7HDrrxiDC+GNyWpGd9MVhcs48FZEgD6sCTREcNMWx9qdM8e+PK0FRcidj\/W3fAdjPDGKqiglSE8DG9QrdheLu0Llu28TYsffZiWsXT7m5Y+1Oi+u\/0PSisuuEuLpRtt7dbwHRjeTPSI8DC8mQjPwEbTumAZiIYuiwj6QAa1CCSpj0orLiS9tFgj2cZ3YHgb0U\/VfQcHB2Xbtm3O9729vdLT01NWFuFheBMRXgErTfKCVUBc1oWEPqxLeaiAk9CHa3R16oK+1hvQ0lpaLFTwvsL4DgxvI\/qpuO\/Ro0dl7dq1MjAw4Hzvvu7o6CiVR3gY3tiFV9AKk7hgFRSVlWGhDyvTHjjoOPWRh6XFAgdeoSC+A8PbiH4q7quju\/rI4KGhIWlubpb169dLe3t72SgvwsPwxi68glYY5wWroIgKH9bb710sxaiv3zk79f7H707IxMR5mTVr5jQON153rcydNUPuaJ\/+XeGhEaBDII7zR56WFmskrfgODG8j+qm4rxpc3fr6+pz\/\/e\/1M1d4+\/btk7a2tlI9ra2tsfcn7xXGcULKe4z0LzoB9BGdXZp7nvjny9Oa8xpV\/dJrVPX92+9dcPZxy3nLa1n\/\/v4GZv9ak1y+PNmuPrRGza3bhr8\/N143Q26fN1P0\/y\/dNTdNNLSVIYGGzh9nx52HRegaujKrTZylxRY9KE0fuinDiII3ferUqbLC4+Pj0tXV5QzIeX1H8BqLVZJlyWLIp39EV0d8x8bGSgbYNbz3Dv5wWmutrTfIDTfYZ3ovXbokH\/zgB2OgTxVFJIA+4svqyYnpxjRo7ZVMbdB9K5lV97MPt0w+YVENrPcz73v3tZb1fq7aOHPmjOhgQaUnNWqfNeYfjF+Uv\/3pRee1bv\/HR5vlC7cx8hs1f6bsV08fFeM4Oy5X\/rpf5AfPOUZXPn6\/XPMf1pkScqmfe\/bskb17907rN4Z3EgmGNwZJBzW8S7b9jdx\/\/\/0xtGh2FTpCo3+F6\/QPNgj4CaCPeDWhI5xRNp0eUGvT0dVKWyXTGqX9avtcuHBBTp8+7YxYBXk0uRrgZ4+ckadeOu5Uue5TrbLs1g87I79sxSMQRh\/vv3tczmx\/UOTHo86NaDqie+29\/9lYKDrC6x3lHR0dlf7+fkZ4r2YUwxuDtMNMaeAvrXjmWMWQNqrIKYGGfpLMaUx0Kz4CUfWh0yX2v3GyZHy3L5svy26179e1+DKRz5rq6cO0pcUaocwc3nJ6GN5G1HR1X\/8UBm5aqw213gkphpRQhcEE0IfByUuh63Ho46mXfuIYX53j+8KaW1LoNU2kRaCaPvxLi+nc3GYd0b15UVpdS70dDC+GN3bRsSxZOKRxXLDCtUhpkwigD5OylX5f49LHK2Pn5N7BI87UhoGl81nZIf1UJtKiXx+mLy3WCCQML4a3Ef1U3ZcHTwTHGtcFK3iLlDSJAPowKVvp9zVOfeg0hzX735JXj50Tpjikn8skWnT18eEP\/kIuvPL\/OCsuuPNzW+580Hlty4bhxfBmonWEN4U9zgtWJsmk0UQJoI9E8RpfeRL6UNO7\/41T8ujdN8mjd3\/EeEY2B3DxxJic+KsBkb\/ut9bouvnHd2B4MzkXIDwMbybCM7DRJAyNgRjochUCSelDDa8aX72RTUd72cwioFMX9NG\/E4d2O0uLzfr0SpnV+YRZQcTcW3wHhjdmSQWrDuFheIMphVJJGRrIFoNAkvpw5\/VyM5s5Wrnw5iFn2sLFNw85I7of+N0H5OK\/\/4LMnTs30LJ15kQavqf4DgxveNXEsAfCw\/DGICMrqkjS0FgBsOBBJq0PTG\/+BVRrabGk9ZF\/OlM9xHdgeDPRK8LD8GYiPAMb5YJlYNJS7HIa+sD0ppjQEE15V1zQ15WWFktDHyG6nGlRfAeGNxMBIjwMbybCM7BRLlgGJi3FLqelD13B4WObXmOt3hRzW62pMEuLpaWPHGCp2wV8h2WG9\/z587Jq1So5fPjwNHEsWLBAhoaGUnnELcLD8NY9O1HAIcAFCyHUIpCmPlzTq2v1\/nDjJ0hMygRcoxtmabE09ZEyjtDN4TssMbz6MIju7m4n2r1790pHR8c0sRw8eFBWr14tLS0tVcuEVliVHRAehjcuLRW9Hi5YRc9wY\/GlrQ9Mb2P5irJ3JaMbdMWFtPURJb609sF3WGB4T506JVu2bJEnn3wy0OitjgI\/9thj0tfXl5gOER6GNzFxFaxiLlgFS2jM4WShD0xvzEmsUp13aTH3YRFBja5bZRb6SIdO+FbwHRYY3vCySH4PhIfhTV5lxWiBC1Yx8phUFFnpA9ObVEbFWTt34tCe0tJiLYuWR15DNyt9JEcnes34DksMr3\/u7o4dO5zIdQqDbmnO39X2EB6GN\/ppy649uWDZle+w0WapD0xv2GxVL19rabFGWslSH430O4l98R2WGN7169c7keo0BXeu7pIlS5z3rhmePXt2otMYvKgRHoY3iRNaEevkglXErMYXU9b6wPQ2lssgS4s10kLW+mik73Hvi++wwPDqHN6VK1fK5s2bnZvVXIO7cOFC6enpcQjoTW0bNmyQnTt3Smtra9w6m1YfwsPwJi6ygjTABasgiUwojDzoA9MbPrn+G9GuvXmRNC9aLvp\/nFse9BFnPI3Uhe+w2PCuWLFCFi9ejOFt5AiKYV9OSDFALHAV6KPAyY0htLzoA9MbLJlRlhYLVnPlUnnRRyMxxLUvhhfDi+GN62iKWA8npIjgLNkNfViS6Ihh5kkfanrX7H9L3jl7kXV6fflsZGmxiNJwdsuTPhqJI459MbwYXgxvHEdSA3VwQmoAngW7og8LktxAiHnTB6a3PJlxLC3WgDwwvB54GF5LDG9nZ6eMj4\/XPG7a2trkwIEDzOFt5OwSYd+8XbAihMAuCRJAHwnCLUDVedQHpnf60mKzOh+XlkUPpq64POojdQhXG8TwWmB4sxJXrXYR3hQdTkh5VGh++oQ+8pOLPPYkr\/qw0fRWWlrs+jV\/HvuNaGF0mFd9hIkhrrL4DgxvXFoKVQ\/Cw\/CGEozFhblgWZz8AKHnWR+2mF7v0mKasqRWXAggh2lF8qyPKPE0sg++wwLDq8uSMaWhkcMk2X05ISXL1\/Ta0YfpGUy2\/3nXR5FNb6WlxXTqgj4GOC9b3vWRJicMrwWG1y8ofQhFe3t7aQ1e\/X5wcFBGRkZkaGhImpubE9cgwmOEN3GRFaQBLlgFSWRCYZigD6\/pHVg6X+5on5kQjXSqTXtpsUaiMkEfjcQXZl98h2WG1\/8QCjd8HjwR5rCJtywnpHh5Fq029FG0jMYbjyn6KILpzXrFhSjKMUUfUWILuw+G1zLDq+HqCO\/w8LDs2LHDefCE\/1HDQUSkBrm7u1smJiac4u5jit19dcR427Ztztve3t6y0WT9DOExwhtEZ5RhHU00UJuASYbGVNN74c1DcvbZL8vFNw850xVaFi2XWZ1PGCFNk\/SRNFB8h4WGV0P2GtaWlhbZu3ev89jhIJs7J3jjxo2OYXbfL1261DG2WvfatWtlYGDAqc597a0f4WF4g2iNMhheNFAcw6uRmGJ6K624kNXSYo0cAxjeKXr4DksNbyMHUKV9ddRYt76+vmnzgSvNGUZ4GN64NVjU+rhgFTWz8cRloj7ybHq9Ky7oa107t3nR8kyXFmtEKSbqo5F4a+2L78DwxqItr+H1vtbK\/e\/1M1d4+\/btE33ghbu1trbG0h+TKuGEZFK20u8r+kifuUktmqyP3\/\/G3zmPIf76534z+xvZzo7LxHd3O1MXZFabXHvzp6R50YPGGl1Xwybro9HjUH999m768K2uri7nBn2v72i0HVP3v+bKlStXTO18tX5r0rds2SJPPvlkoBUYzp8\/L4899pgzWhtkc6dHbN261Zni4B\/R1fm8Y2NjZfW5htdfv84LXr58eZBmC1Pm0qVLcubMGecJd01NTYWJi0DiIYA+4uFY1FpM18cfPn9K\/vanF+Wb97XK78yZkX6azo7LlR88J\/LX\/Y7RlY\/fL9d8\/P7J1wXYTNdHIynYs2ePM13Tv2F4J4kU0vBqYK4p1dfV5uu6N6+FmdPrzt\/9+Mc\/XjK0YQyvmuQ5c+aU9Kimz7ZR3gsXLsjp06edvzgxvI2c3oq5L\/ooZl7jiqoI+vj8zrfk1WPn5MCKj8od7bPiQlOznvffPS5nn31CLr\/+TOlGtGvv\/c+ptJ1mI0XQR1Re6k+8o7yjo6PS39\/PCO9VoIU1vK5gdPR21apVcvjw4WkaWrBgQcV1eN1VHXQHb5lKZlfLhJnSwF9a3JQU9WRmy342\/yRpS44bibMo+vjs9iOO6f1Wzy2JTm+YOLRbJg7tMXLFhSg6KYo+osTu34c5vOVECm944xCN1uFfmcFbr38KAzet1abOCSkuVRazHvRRzLzGFVWR9JGU6S3KigtRNFMkfUSJ37sPhhfDG1pD7ijx7NmzK87zZVmycEg5IYXjZVtp9GFbxsPFWzR9xGl6i7biQjhlTJYumj6iMHD3wfBieEPrx\/\/QCbcC73QHHjwRHCsnpOCsbCyJPmzMevCYi6iPRk2v\/9G\/1968SHQNXX1ohG1bEfURNYcYXgxvVO00tB\/Cm8LHCakhKRV+Z\/RR+BQ3FGBR9bFm\/1uy\/41Tsn3ZfFl2a7DlKv1GV5+I1nLng1YaXVdURdVHlIMG34HhjaKbhvdBeBjehkVkSQVcsCxJdMQwi6yPoKZXje672\/\/AmhvRwkilyPoIw0HL4jswvGE1E0t5hIfhjUVIFlTCBcuCJDcQYtH18dRLP5GnXjo+baTX5hvRwsil6PoIwwLfYaHh9S5NpvNuH3roIfnKV74iO3fuTG0NXISH4Q1zorK5LBcsm7NfP3Yb9KFTG3S0V6c29H\/6WueJaLq0WBEe\/Vs\/w42VsEEfQQnhOywzvK7ZXbhwobS3t8uuXbuctXf1YRS6Jq6+bm5uDqqfyOUQHoY3sngs25ELlmUJDxmuLfrY++3X5e+f3yF\/+M97nDm5Nt+IFkYitugjCBN8h2WGV9fPXblypWzevNl5updreI8dOyYbNmxIbZQX4WF4g5ygKMOyQmigNoGiGxodxT174MuiD4xQozt46VMy8elHnSkObPUJFF0f9QlMlcB3WGZ4NVx9EMSJEyfkgQcekGeeecaZ0vDFL35R7rrrrorr6oYRVNCyCA\/DG1QrtpfjgmW7AuwzvLXm574ydk7uHTwit8+bKS+suQVx1CHA+QPDW00i1jxp7eDBg7J69eoSh97eXunp6Unt5IHhxfCmJjbDG+KCZXgCE+5+kfThfVCEYtNpC82Lljv\/ezdMb3BRFUkfwaOuXBLfYeEIb6OiiWN\/hIfhjUNHNtTBBcuGLEePsQj68E9bCLJ+LqY3mGaKoI9gkdYvhe\/A8NZXSQIlEB6GNwFZFbJKLliFTGtsQZmqj0rTFtTozup8IjAbTG99VKbqo35k4UvgOywwvHqjWmdnp4yPj9dUSFtbmxw4cCCVpckQHoY3\/OnKzj24YNmZ96BRm6aPSo\/9rTRtIWj8b793UT626TW58boZ8sONnwi6mzXlTNNHkonBd1hgeP0C0pvWdEky75zdwcFBliVL8kirUTcnpIzAG9Is+jAkURl10wR9VBvNjeuxv5je6uIzQR9pHToYXssMr3dZso6OjlL0R48eZVmytI46XzuckDICb0iz6MOQRGXUzTzrw3sTmr7WZcVmdT4uLYsejJ2Wml59OMU7Zy\/KwNL5ckf7zNjbMLHCPOsjbZ4YXssMr4arI7zDw8OyY8cOWbx4sbgrNixZsoRlydI+AoV1VjNAblSTXLCMSlfqnc2bPiqN5lZbbSFuWJje6UTzpo+4cx6mPgyvhYZXQ9YR3e7ubpmYmJCWlhbnSWveEd8wIopSFuFNUeOEFEVB9uyDPuzJdZRI86IP79xcjSPJ0dxanDC95XTyoo8o2o57H3yHpYY3biGFrQ\/hYXjDasbW8lywbM18sLiz1Idrci\/8\/ffk4puHHJMbZEmxYJE1Vuqz24\/Iq8fOybd6brF6ekOW+mgsg\/Hvje\/A8MavqgA1IjwMbwCZUIQpL2igDoG0DU2WUxbCigHTy5Q5r2bwHRjesOeQWMojPAxvLEKyoJK0DY0FSAsVYhr6qGZyZ9z8qURuQIszQbab3jT0EWe+kqwL32GZ4a21Ji\/r8CZ5qFWvmxNSNtxNaRV9mJKpbPqZhD7U4Oo28d3d4p2uoDefmWBy\/ZnQ1Rv2v3FKti+bL8tubc0mURm1moQ+Mgql4WYxvJYZ3mqK0ZUb7rnnHmfVhjQ2hMcIbxo6K0IbXLCKkMXkYohLH+4o7sU3vycTh3Y7HdY5uaaaXEzvJIG49JGcgtOrGd+B4XUIsA5vegedvyVOSNmxN6Fl9GFClrLrY1R9eA3u+2eOl24600j0xrMZNy9yzG6Rtqde+ok89dJxZ5RXR3tt2KLqo4hsMLwYXoeAPmnt6aefjvRoYV3Hd9OmTWX7an3btm1z6u7t7S17qpt+hvCmhMcJqYin1vhiQh\/xsSxiTUH0oeb2\/XePi7OqwqE9jrl1N3cUt5HH+5rEVac26BQHW0xvEH2YlL9G+orvsMzw1prD6z6IIoyg3Pp0nwMHDkhra6szWrx27VoZGBhwqnJfe9f5RXgY3jA6s7ksFyybs18\/dq8+5Ox4ydjq1AR35NZrbps+dJNc++8+VcgR3Pq0Jku8MnZO7h08IrfPmykvrLkl6G5GluP8MZU2fIdlhjfuI1ZHcv\/qr\/5Kzp8\/XzK8+tnIyIgMDQ1Jc3Oz82S39vb2slFehIfhjVuLRa2PC1ZRMxs+Lu9Ire6tpla3ibffEvnxaKlCHbXVTackNF3\/G465\/cCHbnLm5LLZZXo5f2B4qx3z11y5cuVKkU8IOiK7cuVK2bx5c9mT1aLM4dV9dBT3M5\/5jHzta18rGV41uLr19fU5\/\/vf62cYXgxvkY+zOGPjghUnzWzrclc\/0F7oFAN3cz+\/fPWzy2f+0flKR2gnje3UFAR3n5J5va5NLrfcILN+5x5pamoq3WiWbaRmtG7DSC\/nDwyvdYbX+yjhasEvWLCgNCob5HTlruygZb1zeP0jujriOzY2VjLAruF98Uv\/u6xbty5IU4Uuc\/nyZZk4f15mzZxZ6DgJLhoB9BGNW629XEMZtmbXgFbbzzWsfiMbqJ1ZbaViOu1Atw9c\/xvO\/zpCq5t35NYtjKEJRLdqITW9933z75zpDX\/5hd9qrLIc7m2zPnSAz7uNj49LV1eX8wu0LsNq+2btCG\/YxOuNai+++KJjYv03rQU1vP\/6X\/79tGZbb7BrjUQF8ItfXHY4\/PIvN4VNA+UtIIA+EkryrDnhK76uzkXSY1q18mt878Xbprcuf7kQPbt06ZKcOXPGuX9CR3jZwhM48c+X5bN7xmX2rzXJC8uLZYRs1seePXtk794HFeR7AAAgAElEQVS90wSB4Z1EUnjDG\/5UMDklYXh42NlVR4F1+sJjjz3m3IymN6JVMrxaNsiUhq1bt8qcOVMXHj1p6z+btgsXLsjp06edvzi5YNmU+WCxoo9gnGwthT7iybyaXh3p1V9Uvv65drmjfVY8FWdci8360BFe7yjv6Oio9Pf3M8J7VZOFNLzuSgr\/9E\/\/JGowdfqBDu37t6BPWqs2PaKlpcX5a+rVV18tm8LATWu1z3g2\/+SU8bXAiObRhxFpyqyT6CM+9G+\/d9FZsuydsxdlYOl8uaPd\/Glm6GNKH9w7VH6sFNLwxnc6qFyTf4SXZcnCEeeEFI6XbaXRh20ZDxcv+gjHq17pople9IHhraZ5DG+9s0GF73nwRARonl04ITXGr+h7o4+iZ7ix+NBHY\/yq7f3Z7Ufk1WPn5Fs9txg90os+MLwY3mTOEYFr5aeFKVSckALLxsqC6MPKtAcOGn0ERhW6YBFML\/rA8FpteHVEdvXq1dMYBJ3DG\/qsUWEHDC+GNw4d2VAHFywbshw9RvQRnV2QPU03vegDw2ut4XVvYFu6dGnZk8+CHPhxlsHwYnjj1FOR6+KCVeTsNh4b+micYb0a9Ea2\/W+cku3L5suyW81aRQh9YHitNryVnrRW74CP+3sML4Y3bk0VtT4uWEXNbDxxoY94ONarxTW9j959kzx690fqFc\/N9+gDw2ut4dXAKz35LO2jE8OL4U1bc6a2xwXL1Myl02\/0kQ5nbUVHedX46iivjvaasKEPDK+1hted0tDIOrxxHOQYXgxvHDqyoQ4uWDZkOXqM6CM6uyh7mmZ60QeG11rDG+UAT2IfDC+GNwldFbFOLlhFzGp8MaGP+FgGremVsXNy7+ARuX3eTHlhzS1Bd8ukHPrA8GJ4Mzn0EF4l7JyQMhZjzptHHzlPUMbdQx\/ZJMAU04s+8B3WGt5aUxpcKO4jgjs6OhI7kzDCywhvYuIqWMVcsAqW0JjDQR8xAw1RnQmmF31geK01vBr4+vXrpb29vWxZMu+NbPp6ZGREnn766RCHfriiGF4MbzjF2FuaC5a9uQ8SOfoIQim5Mvoo4o9tek1uvG6G\/HDjJ5JrKGLN6APDa63h1RHeSsuSHT16VDZs2CA7d+6U06dPO6+\/\/e1vRzzE6u+G4cXw1lcJJZQAFyx0UIsA+sheH3k2vegDw2ut4XVHeIeHh2XHjh2yePFicZ+8tmTJEunr63OWLWOEN72TKCek9Fib2BL6MDFr6fUZfaTHulZLanp1ybJ3zl6UgaXz5Y72mbnoGPrA8FpteDV4HdHt7u6WiYkJ8c7Z9Y70trYm90QZRngZ4c3F1cCATnDBMiBJGXYRfWQI39d0Hk0v+sDwWm94sz5FYHgxvFlr0JT2uWCZkqls+ok+suFeq9XPbj8irx47J9\/quSXzkV70geHF8GZ8jsDwYngzlqAxzXPBMiZVmXQUfWSCvW6jeTG96APDi+Gte7gmWwDDi+FNVmHFqZ0LVnFymUQk6CMJqvHUmQfTiz4wvFYbXvcmNT+EtrY2OXDggCQ5d9dtE8OL4Y3nklL8WrhgFT\/HjUSIPhqhl\/y+eiObPo54+7L5suzW5O6LqRYJ+sDwWmt43QdPPPzww\/Kd73xH1q5dK\/PmzZNVq1bJwoULy9bmTfJUgOHF8CapryLVzQWrSNmMPxb0ET\/TuGt0Te+jd98kj979kbirr1kf+sDwWm143XV49+zZU3oARVqrMzDCO116nJBSPf8b1xj6MC5lqXYYfaSKO3JjOsqrxldHeXW0N60NfWB4rTW858+fd0ZzZ8+eLffcc4+sXr1avvrVr8ozzzwjJ06cYEpDWmchTzuckDKAblCT6MOgZGXQVfSRAfSITWZhetEHhtdaw+sG\/vjjj8t9993nPFVNTe+CBQtkaGhImpubIx7K4XZjSsMUL05I4bRjW2n0YVvGw8WLPsLxyrr0K2Pn5N7BI3L7vJnywppbEu8O+sDwWm94Ez\/K6jSA4cXwZq1BU9rngmVKprLpJ\/rIhnsjraZpetEHhhfD28jRenVffQTxtm3bnHf+EWLvd729vdNuhsPwYnhjkKAVVXDBsiLNkYNEH5HRZbpjWqYXfWB4rTK87soM4+PjNQ\/wMMuS6dJmjzzyiOzdu1c6Ojpk\/fr1Tt19fX3OY4t19YeBgQHnM\/e1lnM3DC+GN9OrjUGNc8EyKFkZdBV9ZAA9pib1UcQf2\/Sa3HjdDPnhxk\/EVGt5NegDw2ut4Q1jamsdfWpw29vbKy5jpqO7IyMjpTnBlcpieDG8iZzdC1gpF6wCJjXGkNBHjDAzqMprevVRxGp+49zQB4bXKsPrDdb70Imo5tcdMd64caMsXrx4GkvvaK9+6X+vn7mGd9++faL9cLc0HnoR58kkjro4IcVBsbh1oI\/i5jaOyNBHHBSzrUNN77pn\/4e8c\/aifP1zvyl3tM+MrUM260O9infTX7m7urqcATmv74gNtmEVXXPlypUrhvU5cnfViA4PDzv7h1mlQUWka\/k+8MADzpJmExMTZfv7R3R1xHdsbMyZ7uBuruH1d767u1uWL18eOSYTd7x06ZKcOXPGecJdU1OTiSHQ5wQJoI8E4RagavRRgCSKyIl\/vixP\/M3P5OTEZXni078uvzMnnpFem\/WhzxrQaZf+DcM7ScQqw+uKwB2x1fdBHi3slte1fHUpM93ctX3V1IYxvFu3bpU5c+aU9Kimz7ZR3gsXLjjLw+lfnBjeYly84owCfcRJs3h1oY9i5fTzO9+SV4+dkwMrPip3tM9qODib9aFexTvKOzo6Kv39\/YzwXlWVNYbXfQDF4cOHndCXLFlSNgLrPcr8I8GbN2+WBx98ULxTGnSqxKZNmxzDvGXLFmd3d0S31pQG\/tISsfknp4bP5hZUgD4sSHIDIaKPBuDldNfPbj\/imF6d09vo9Ab0MZVk7h0qF3zhDa\/XvNYyubXOA65ZXrFiRWkOrxreXbt2OSO++hOCdwoDN63VPqtyQsrpVScn3UIfOUlETruBPnKamAa7FZfpRR8Y3mpSLKTh9S5LFtXk+oF5V2LQ73RKw8KFC51VG1iWLNyZjhNSOF62lUYftmU8XLzoIxwvk0qv2f+W6OOIty+bL8tubY3UdfSB4bXW8NY6YsKu2uB9uITfSPPgieDnJk5IwVnZWBJ92Jj14DGjj+CsTCz51Es\/kadeOi6P3n2TPHr3R0KHgD4wvFYZ3tBHSAo7MJdmCjInpBQEZ3AT6MPg5KXQdfSRAuSMm9BRXh3t1VFeHe0Ns6EPDC+GN8wRk0BZDC+GNwFZFbJKLliFTGtsQaGP2FDmuqKophd9YHgxvBkf2hheDG\/GEjSmeS5YxqQqk46ij0ywZ9LoK2Pn5N7BI3L7vJnywppbAvUBfWB4MbyBDpXkCmF4MbzJqatYNXPBKlY+444GfcRNNN\/1hTW96APDi+HN+JjG8GJ4M5agMc1zwTImVZl0FH1kgj3TRsOYXvSB4cXwZnq4imB4MbwZS9CY5rlgGZOqTDqKPjLBnnmjb793UT626TW58boZ8sONn6jaH\/SB4cXwZny4YngxvBlL0JjmuWAZk6pMOoo+MsGei0bV9OqcXt0Gls6v+FQ29IHhxfBmfLhieDG8GUvQmOa5YBmTqkw6ij4ywZ6bRtX06pJl75y9WNH0og8ML4Y348MVw4vhzViCxjTPBcuYVGXSUfSRCfZcNVrL9KIPDC+GN+PDFcOL4c1YgsY0zwXLmFRl0lH0kQn2XDb62e1H5NVj5+RbPbeUpjegDwwvhjfjwxXDi+HNWILGNM8Fy5hUZdJR9JEJ9tw26je96APDi+HN+HDF8GJ4M5agMc1zwTImVZl0FH1kgj3XjbqmVx9D\/Pu\/NVNOnjwpc+fOlaamplz3O+nO4TvKCV9z5cqVK0lDp36WJfNqgAsWR0QtAugDfaAPNBCWgN7Ipo8j3rbkJrlzrmB4Bd\/h1xCGN+xRFbE8f2kxwhtROtbthuG1LuWhAkYfoXBZVfipl34iT710XP5wwUzZ9LnfYoT39delq6tLRkZGpK2tzSotVAoWw5uSBDC8GN6UpGZ8Mxga41OYaADoI1G8xlf+f7\/2jqx7dkyW3doqOsXB5g3fwZSGTPSP8DC8mQjPwEYxNAYmLcUuo48UYRvYlOrjr37wE\/nD50\/J7fNmygtrbjEwini6jO\/A8MajpJC1IDwMb0jJWFscQ2Nt6gMFjj4CYbK2kKuPf7zUIvd98++sNr34DgxvJicChIfhzUR4BjaKoTEwaSl2GX2kCNvAprz6eP34eedRxLaO9OI7MLyZHMIID8ObifAMbBRDY2DSUuwy+kgRtoFN+fXxytg5a00vvgPDm8khjPAwvJkIz8BGMTQGJi3FLqOPFGEb2FQlfeijiD+26TW58boZ8sONnzAwqmhdxndgeKMpp8G9EB6Gt0EJWbM7hsaaVEcKFH1EwmbNTtX0oaZX1+p95+xFGVg6v\/Qo4iKDwXdgeCPre\/369TI8POzsv2TJEunr6yvVNTg4KNu2bXPe9\/b2Sk9PT1k7CG8Kx\/Hjx2X37t2yatUq1gaMrMbi7og+ipvbOCJDH3FQLG4dtfRhm+nFd2B4Ix3pamh18eahoSE5f\/68dHZ2ytKlSx1je\/ToUVm7dq0MDAw4dbuvOzo6Sm0hvCnssIgkQWt2Qh\/WpDpSoOgjEjZrdqqnD9f0vnrsnHyr55ZCj\/TWY2GNKK4GyoMnAmZcR3d1c0d1ve+9Zri5uVn0u\/b29rJR3riFNz4+Ls8995zcf\/\/9gUZJ65Wv9X2l74J85i2j7Nz+6udxP\/2lXnz+NNcrnyYPL5u4noZTL74887BBH\/781Hoftz7i1ob2r1qdQT8PwyNufZjOA320lekvqD4+u\/2IBDG9cesjzWtLUBYBbZDxxTC8AVNYaYR348aNsnjxYsfgVjPDbvVxG96w9dUrX+v7St8F+cxbxnvgJXEQ1ovPn+Z65eFRTgwe8fLw86z1Pu7jpZ72wx4rWr5anUE\/h0f5I2DhkQ6PIKY37uPF9HNpQMuUy2IY3hBpOXjwoKxevVpaWlpk79694k5Z8I\/oqjkeGxsrm+Prinzfvn2BRmTrdcu9CAatr175Wt9X+i7IZ94yGo+O6mp\/va\/jHNF06w9SJzzKFQaPdHn4edd6H\/fxUi\/XlUb\/6x1b1eoM+jk8Jn\/1cs\/n8EiPx3\/5bxOy\/41T8n\/9+xbnccRR9O\/dp97xlcW1VqdjBrku1vMdpn+P4Q2YQTW1J06ccObw6qY3XC1cuNCZthDE8KrIH3nkERkdHQ3YIsUgAAEIQAACEEiawM\/\/txXySz\/\/nzLjv\/\/XpJtKvf7bbrtN9u\/fn3q7eWwQw1shK97VGBYsWCCbN2+WBx98UNwpDLqLjvZu2rRJDhw4IFu2bHFqqTS\/1\/+XnxpfNghAAAIQgAAEIJA0AR3ZZXR3kjKGN4DaTp065azKUM3wPv\/882VTGCrdtBagGYpAAAIQgAAEIAABCCRAAMMbEGqlKQ2zZ892RnWDLEsWsBmKQQACEIAABCAAAQjETADDGxCorr2r83YPHz7s7BH2wRMBm6EYBCAAAQhAAAIQgEDMBDC8MQOlOghAAAIQgAAEIACBfBHA8OYrH\/QGAhCAAAQgAAEIQCBmAhjemIFSHQQgAAEIQAACEIBAvghgePOVD3oDAQhAAAIQgAAEIBAzAQxvzECpDgIQgAAEIAABCEAgXwQwvPnKB72BAAQgAAEIQAACEIiZAIY3ZqBUBwEIQAACEIAABCCQLwIY3nzlg95AAAIQgAAEIAABCMRMAMMbM1CqgwAEIAABCEAAAhDIFwEMb77yQW8gAAEIQAACEIAABGImgOGNGSjVQQACEIAABCAAAQjkiwCGN1\/5oDcQgAAEIAABCEAAAjETwPDGDJTqIAABCEAAAhCAAATyRQDDm6980BsIQAACEIAABCAAgZgJYHhjBkp1EIAABCAAAQhAAAL5IoDhzVc+6A0EIAABCEAAAhCAQMwEMLwxA6U6CEAAAhCAAAQgAIF8EcDw5isf9AYCEIAABCAAAQhAIGYCGN6YgVIdBCAAAQhAAAIQgEC+CGB485UPegMBCEAAAhCAAAQgEDMBDG\/MQKkOAhCAAAQgAAEIQCBfBDC8+coHvYEABCAAAQhAAAIQiJkAhjdmoFQHAQhAAAIQgAAEIJAvAhjeFPMxPj6eYms0BQEIQAACEICA6QRaP3i5oRCarr+pof2LsjOGN6VMqtl95JFHZHR0NKUWaQYCEIAABCCQPwI3\/Eq5gatm6PzlvJG0\/sr70wK7oYYxbPW16d251n5B6NWqO8j+SZf5la+9I21tbUk3k\/v6Mbwppej111+Xrq4u2bp1q8yZM6fhVtU49\/f3B66vXvla31f6Lshn3jJq+N3+el\/HwUJh1ovPD7xeeXiUE4NHvDz8PGu9j\/t4qaf9sMdKreOvWlth4tdzRLVzSRznD3jU1\/blM\/9YKnT5zHG5\/O7x0vvjR16R988cLxkq73deIxmnKTz1L01O+21zJk3c+E8nfz3V96f\/pUn0mLntd3+3LLAP1BjlbLr+N6pek8d\/+lN57i\/+Qu7\/3Oekrcq1u+lDUyOoo6+Pyl8895ysW7dO2tqmX+trfV\/pO\/ez3k19MueqaS1pdssW5zPve+2vc63dskXc13\/67cMYXhHB8DZsPYNV4BrekZGRWIQXtr565Wt9X+m7IJ95y+gJSA2\/xu99HddfnfXi82epXnl4lBODR7w8\/DxrvY\/7eKmn\/bDHipavVmfQz+ExOSDiXh\/S4KHGVbf33z0u7uuLb35P1Pwd2vWUYxjVvLrfVbrS6U\/lrtm86ZY7nCJeY+kaSTVe2\/f915IJdH9id43a13YfKLsuvv76qPOL6LN\/833n8zR4VLuSx328mH4uDeZ48lkKw5tSXsIeNPW6pRfB567+FVmvrPMXcJ3ytb6v9F2Qz7xlvK\/jZhEkPj+jPPGo15cg+Q0bX9jy6KOcWKM8\/PvXeh+3PsLWF6R8tTJBPw\/DI+7zR5D4vNkPUj5o3G69YeL3n+8qteU3s2pkHXN7dXS2kol1TaiOVrqmVQ2rO3rpfv+BD90k3jmheeORd300eu6odL1L81ob5XqUl30wvCllIu6DMKVuJ9IMLBLBWphK0UdhUplIIOhjCqsaV3eEVkdjL\/z996aNyvqNrGtiqxnYRJKWYqXoYwo2LMqFh+FN6UBEeFOgjx8\/Lrt375aNGzdKU9PkXCw2CLgE0AdaqEXARn24I7IX3jwkOlqrI7UX3zxUwuSa12tvXuR8NuPmTzmjsP7RWBuUZaM+quUV34HhzeSYR3hT2C9evCgnT56UuXPnYngzUWO+G0Uf+c5P1r2zQR9qcCuZW6+xtdnU1tKgDfoIegziOzC8QbUSazmEh+GNVVAFrowLVoGTG0NoRdSH1+BOHNpdNnKro7auuXVHcGPAWNgqiqiPqMnCd2B4o2qnof0QHoa3IQFZtDMXLIuSHSHUIuijksHV0Vu9Qezaf\/cpmXHzIsHcRhCHiBRBH9Ein74XvgPDG5eWQtWD8DC8oQRjcWEuWBYnP0DopupDTe7Ed3c7N5a582\/V5KqxbV60HIMbIPdBipiqjyCxhS2D78DwhtVMLOURHoY3FiFZUAkXLAuS3ECIJunDb3Jdg6tTFFoWPdgABXatRsAkfSSdRXwHhjdpjVWsH+FheDMRnoGNcsEyMGkpdjnv+qhmchnFTUckeddHOhQmW8F3YHjT1FupLYSH4c1EeAY2ygXLwKSl2OU86sOdkztxaI8zXYGpCikKwtdUHvWRFQ18B4Y3E+0hPAxvJsIzsFEuWAYmLcUu50kf7mju2We\/7JhcvemsZdFypiukqAd\/U3nSR4YYGOGtAJ8HT6SkSAwvhjclqRnfDBcs41OYaABZ68N9upmaXHc01zG5dz5Y9sjdRCFQeVUCWesjT6nBdzDCm4keER6GNxPhGdgoFywDk5Zil7PShzuaq9MW9LXedMbNZykmPmBTWekjYPdSLYbvwPCmKji3MYSH4c1EeAY2ygXLwKSl2OW09eGftsBoborJjtBU2vqI0MXUdsF3YHgjie3UqVPS2dkp4+Pjpf17e3ulp6fHeT84OCjbtm1zXns\/x\/BOx80JKZIErdkJfViT6kiBpqUPNbpnD3xZ9MlnOj9Xje6szici9Zmd0iOQlj7Siyh6SxheDG8k9Rw9elQ2bNggO3fulNbW1rI69Lu1a9fKwMCA87n7uqOjo1QO4U0h44QUSYLW7IQ+rEl1pECT1oca3Xe3\/0HZ\/FyMbqRUZbJT0vrIJKiIjeI7MLyRpHPw4EHZtWuXDA0NSXNzc1kdOro7MjJS+m79+vXS3t5eGv3VwggPwxtJeBbuxAXLwqSHCDkpffiN7qzOx1ltIURe8lI0KX3kJb4w\/cB3YHjD6KVU1m9qvZWowdWtr6\/P+d\/\/3mt49+3bJ21tbaXd\/aPFkTpn2E6ckAxLWMrdRR8pAzesudj1cXa8NKIrs9rk+jV\/zmN+DdOEt7ux68MgFjr10rvpFMyuri5nQM7rOwwKKdausixZQJxqYoeHh0ulFyxYUHVEV83x2NhYyQB7Da+\/ue7ublm+fHnAXhSj2KVLl+TMmTPO1JCmpqZiBEUUsRFAH7GhLGRFsenj7LhceeYRkR+Pihrdazq3iMz73UIysymo2PRhILQ9e\/bI3r17p\/UcwzuJBMMbQNTnz5+XVatWyezZsx0T63\/vn8JQy\/Bu3bpV5syZU2pVTZ9to7wXLlyQ06dPO39xYngDCNCyIujDsoSHDLdRfbz\/7nE5s\/1Bx+jqzWg6onsNRjdkFvJbvFF95Dey+j3TEV7vKO\/o6Kj09\/czwnsVHYa3voYqltA5vZs2bZIDBw7Ili1bnDJBpjTwl5aIzT85RZSbVbuhD6vSHTrYqPpgjm5o1EbuEFUfRgZbp9PM4S0HhOGNqHLvTWz6E4J3CgM3rdWGygkpougs2Q19WJLoiGGG1Yd\/eTFuRosI3pDdwurDkLAidRPDi+ENLRx3Dd6NGzfK4sWLnZ8MdE3epUuXOisxsCxZOKSckMLxsq00+rAt4+HiDaoPjG44rkUpHVQfRYm3VhwYXgxvJJ37HzyxZMmSspvSePBEcKyckIKzsrEk+rAx68FjrqcPHhgRnGURS9bTRxFjrhYThhfDm4neEd4Udk5ImUjQmEbRhzGpyqSj1fSB0c0kHblrlPPHVErwHRjeTA5QhIfhzUR4BjbKBcvApKXYZb8+MLopwjegKc4fGN5qMuWmtZQOYAwvhjclqRnfDBcs41OYaACuPj78wV\/Ie9\/8T6VHAHMzWqLYjamc8weGF8Ob8eGK4cXwZixBY5rngmVMqlLvqI7mThz9Gzn710OldXQxuqmnIdcNcv7A8GJ4Mz5EMbwY3owlaEzzXLCMSVVqHXWM7nd3y8ShPaKv5d\/eJtd3bZKWjk+n1gcaMoMA5w8ML4Y342MVw4vhzViCxjTPBcuYVCXeUdfonn32y85T0a69eZE0\/e4Dcu66\/1Xmzp3LkxoTz4B5DXD+wPBieDM+bjG8GN6MJWhM81ywjElVIh1Vk3vhzUPOaO7FNw85Rrdl0XJpufNB5zX6SAR7YSpFHxheDG\/GhzOGF8ObsQSNaZ4LljGpirWj\/mkLam4rzc9FH7FiL1xl6APDi+HN+LDG8GJ4M5agMc1zwTImVQ13tNJork5baF603Jm+UGlDHw1jL3QF6APDi+HN+BDH8GJ4M5agMc1zwTImVZE6Ws3kzrj5U9Ky6MG6daKPuoisLoA+MLyFNrznz5+XVatWyeHDh6fFuWDBAhkaGpLm5uZMTwIYXgxvpgI0qHEuWAYlK2BXK5ncpg9dnZsbwOR6m0EfAaFbWgx9YHgLaXiPHj0q3d3dTmx79+6Vjo6OaXEePHhQVq9eLS0tLVXLpHFewPBieNPQWRHa4IJlfhbV4L7\/7nHnprMLf\/+90s1nUU0uhtd8TaQVAecPDG\/hDO+pU6dky5Yt8uSTTwYavdVR4Mcee0z6+vrSOu7K2sHwYngzEZ6BjXLBMi9pztq4ImWrK+h7dymxoNMVgkSOPoJQsrcM+sDwFs7wmnY4Y3gxvKZpNqv+csHKinzwdiuN4PoNrmt2g9carCT6CMbJ1lLoA8OL4c346MfwYngzlqAxzXPByleqXHOr\/19883vyvvP\/IaeT3hHcpAyunwb6yJc+8tYb9IHhLbTh1ekNnZ2dMj4+XvXYYw5vfk5LnJDyk4s89gR9ZJOVWsbWNbc6B\/cD198kOkUhLYOL4c1GD6a2yvkDw1tow6vBrV+\/Xtrb26Wnp6cU6+DgoIyNjTnzdvX1yMiIPP3004kcx1r\/tm3bnLp7e3vL+qGfMcLLCG8iwitgpVywkkmqd56ttuCO1k6+nhyxdY2t\/q\/r4LrG9gMfuskxuHnY0EcespDfPqAPDG+hDa+O8K5cuVI2b95ctlKDruKwYcMG2blzp5w+fdp5\/e1vfzv2I1XbWbt2rQwMDDh1u6+9q0ZgeDG8sQuvoBVywQqeWNfE6h66KoL7Xs2s89mZ43LZ83klU6uf5dHYVqOAPoLrw8aS6APDW2jDq8HpCO\/w8LDs2LFDFi9eLO5yZEuWLEl8hNcdPXbX+6002ozhxfDaePGJErNNFyy\/YVVe3s8mzeo\/ljCqgXXKVDCxfjPrTj\/wGlp9Xe0JZlFylcU+NukjC76mt4k+MLyFN7waoLsu78TERNm6u96R3tbW1tiPZzW4urlLnvnf63eu4d23b5+0tbWV+pBEf2IPMOYKOSHFDLRg1cWij7PV5\/MrLh0NrbR5zab\/ezWZ1TavKXXLuObUfe\/dv1Y7pTZmTZ0n1Lzq9oHrf8P5X6cXuJ+5Uw10bq37WcEkURZOLPooMiDLY7NZH\/prt3fT+5q6urqc6Zxe32GrRK65cuXKFVuDjytu\/4iud\/L\/b70AACAASURBVO6w24Ya3pNP3DmtSTW8N9xwQ1xdMaaeS\/9yST74Kx80pr90NCCBsz8NWLB2sV\/84rL88i83TRaqY15jaTBIJR4DOq34rDnTa7huyrA6X17d\/xp\/Pe6+bvla7QTpZ8HLXLp0Sc6cOSN67mxquqqRgsdMeMEJ2KyPPXv2OA\/Y8m8Y3kkiGN7gx1HVkkEN75EN\/0E+d\/\/nYmjR7CouX35fLly8KC3NLTUDefu9C7kK9ERTq8y+XP4XdK46mHJn3j57cVqLJ3852C8ob783fd9q3VfuUbaTTdX\/kDxRpZ8nQ7Q1+9emm60Pt0x9duN1M0rd9r7WD+fOmvyuvMy10z6LEnfR97lw4YJzT4aOWGF4i57t8PHZrA8d4fWO8o6Ojkp\/fz8jvFdlVBjDq09SW7VqlRw+fFgWLFggDz30kHzlK19xblhLetpAmCkN\/KUlYvNPTuFP3\/btUU8fYcxyJXpR9n+ngrmv9AeZt25\/O24dQdv3mmE1yO77qf+vLTPOflNdVOXU00dR4yauYATQxxQn7h0q10whDK9rdhcuXOgsTbZr1y7RG8h0aF8NpnszWbDDJXwp\/xQGblqrzZATUniN2bSHTfqoZJCnjPHULxxazi2r31czza7pdQ2y3xzf0T7TeCnZpA\/jk5VBAOgDw1tNdoUwvN5lyfSnLtfwHjt2rLQsWZKjvCxLFu6sxgkpHC\/bSqOP4Bl3ja\/+P2mEJ02ya5ArmWOvKXYN8O3zZokpZhh9BNeHjSXRB4a30IZXg9NR1RMnTsgDDzwgzzzzjDOl4Ytf\/KLcddddpdUTkjz4efBEcLqckIKzsrEk+og\/615j\/Oqxs04Dr4ydu2qSp+ZTqxnW0eE8G2H0Eb8+ilQj+sDwFt7waoDu2rtusJWeeJbVgc1cminynJCyUqEZ7aKP9PPkjgirGZ56fa7UETXCy26dvHnw0bs\/kn4HPS2ij0zx575x9IHhtcLw5vlIxPBiePOszzz1jQtWfrKh5nf\/GydLI8KvHps0wVkaYPSRH33ksSfoA8OL4c34yMTwYngzlqAxzXPBym+qvCPBOiXCb4DTmAuMPvKrjzz0DH1geAtnePVGtc7OTtEnidTadK3GAwcOJL40Wb0DHcOL4a2nEb6fJMAFyxwleA3wUy9NPonOHf1dduuHy9YZjisq9BEXyWLWgz4wvIUzvP6AKi0FpjeSpbEsWZDTBoYXwxtEJ5TB8JqsAXcKhN\/8xjnvF0NjskKS7zv6wPAW2vB6lyXr6OgoxarLhW3YsCGVh0\/UO4wxvBjeehrhe0Z4i6QBv\/l99O6bJI5RXwxNkVQSfyzoA8NbaMOrwekI7\/DwsOzYsUMWL15cWrFhyZIlqSxLVu+wxfBieOtphO8xvEXVwFMv\/UT2v3HKWQFCpzzoiK+76kPYmDE0YYnZVR59YHgLb3g1QB3R7e7ulomJCWlpaXGetOYd8c3ysMfwYniz1J9JbXPBMilb4fqqN7qp+dWb3dT4DiydH\/qBF+gjHHPbSqMPDK8VhjfPBzaGF8ObZ33mqW9csPKUjWT6oiO9a\/a\/5Rjf2+fNlO3L5ge+wQ19JJOTotSKPjC8GN6Mj2YML4Y3Ywka0zwXLGNS1XBHdcR37dNvOVMddI5vkJvb0EfD2AtdAfrA8BbO8OqNalu2bJEnn3xSmpub6x7A58+fl8ceeyyz+bwYXgxvXZFSwCHABcs+Ieg0B13ZQac5fKvnlpqjvejDPn2EiRh9YHgLZ3g1IHfOrr6uNl\/Xfdxw1nN6MbwY3jAnbZvLcsGyM\/veaQ61RnvRh536CBo1+sDwFtLwukHp6O2qVavk8OHD0+JcsGCBDA0NBRoFDnpARSmH4cXwRtGNjftwwbIx61Mxu6O9Orf3hTW3TIOBPuzWR73o0QeGt9CGt94BkIfvMbwY3jzo0IQ+cMEyIUvJ9lHn9t47eMSZ2vDDjZ8oawx9JMve9NrRB4YXw5vxUYzhxfBmLEFjmueCZUyqEu2oTnH42KbXpple9JEoduMrRx8YXgxvxocxhhfDm7EEjWmeC5YxqUq8o+683nfOXiyt2Ys+EsdudAPoA8OL4W3wENZVITo7O2V8fLxUU29vr\/T09DjvBwcHZdu2bc5r7+duYQwvhrdBCVqzOxcsa1IdKFC\/6f142ww5efKkzJ07V5qamgLVQSF7CHD+wPBieBs83nVFiA0bNsjOnTultbW1rDb9bu3atTIwMOB87r72PuUNw4vhbVCC1uzOBcuaVIcK9LPbj4iO9G5bcpPM+18uYnhD0bOnMOcPDC+Gt8HjXZc327VrV8UVH3R0d2RkpPTd+vXrpb29vTT6q01jeDG8DUrQmt25YFmT6tCBuqb3L\/9jK4Y3ND07duD8geEtvOH1Lk2mS5E99NBD8pWvfKXiiGyUw95var11qMHVra+vz\/nf\/x7DW06cE1IUBdqzD\/qwJ9dhI3VvZPudOTPk\/113K1MawgK0oDznDwxvoQ2va3YXLlzojKy6I7H6MArvyGsjx7qa2OHh4VIV3vV9\/SO6ao7HxsbKnurmjvDu27dP2traSvX4p0c00kdT9uWEZEqmsukn+siGuymtfu8ffiaf3\/mWfOmutkCPIjYlLvoZDwGbzx96r5F303uOurq6HB\/k9R3xkDavlmuuXLlyxbxul\/dYk7xy5UrZvHmznD59umR4jx07VnXebZiYXUM9e\/Zsx8T634cxvP52u7u7Zfny5WG6Y3zZS5cuyZkzZ5y50Nx0Ynw6Yw8AfcSOtFAVqj6e+dvT0v+3\/yrfvK9VdLSXDQIuAZvPH3v27HGeOuvfMLyTRApheDUQNZ0nTpyQBx54QJ555hlnSsMXv\/hFueuuu8pGWoOcFryjudWe1KZzejdt2iQHDhyQLVu2ONUGmdKwdetWmTNnTqkbavpsG+W9cOGC84eJ\/sWJ4Q2iSLvKoA+78h02WlcfX3vjfXn2yM\/kwIqPyh3ts8JWQ\/mCErD5\/KGDf95R3tHRUenv72eE96rWC2N4NR41oatXry4dxpWWB4vrGPfexKZ\/UXmnMHDTWm3KNv\/kFJf+ilwP+ihydhuPzauP3\/\/G3zkVVnoEceMtUYOJBDh\/TGWNm+XLFVwow5vUwemuwbtx40ZZvHix8xeUrsm7dOlSZyUGliULR54TUjhetpVGH7ZlPFy8Xn28fvy88wjiR+++ifm84TAWtjTnDwxvNXFjeAMe9v4HTyxZsqRsqgQPnggIUkQ4IQVnZWNJ9GFj1oPH7NfHUy\/9RJ566bh8q+cWuaN9ZvCKKFlIApw\/MLyFN7zeObXPP\/+889SzlpYWZwK39wEQWR3h\/LQwRZ4TUlYqNKNd9GFGnrLqZSV96Pq8ujG1Iaus5Kddzh8Y3kIbXnfVhBUrVshv\/\/ZvO9MNdPqBbtUeFpH24YnhxfCmrTlT2+OCZWrm0ul3JX28MnbOmdqwfdl8WXZr+ZMw0+kVreSFAOcPDG+hDa9\/WTJ39QRdCaDa44DTPjgxvBjetDVnantcsEzNXDr9rqaPNfvfkv1vnJIfbvyE3HgdS5Wlk438tcL5A8NbaMPrHeHV1RLch03E+eCJRg9rDC+Gt1EN2bI\/FyxbMh0tzlr6+Nim1+T2eTOdkV42Owlw\/sDwFtrwanDukmTuvF39bO3atTIwMMAc3pyd9zgh5SwhOesO+shZQnLWnVr60BFeHenlBracJS3F7nD+wPAW3vCmeDxFaooRXkZ4IwnHwp24YFmY9BAh19MHN7CFgFnAovX0UcCQq4aE7yhHw7JkKakf4WF4U5Ka8c1wwTI+hYkGUE8f7g1sjPImmobcVl5PH7nteAIdw3cU1PD6n7LmhqmPr9XH\/2b9+F6Eh+FN4HxWyCq5YBUyrbEFFUQfjPLGhtu4ioLow7igInYY31FAw+s+FOLhhx+W73znO87c3Xnz5smqVatk4cKFztPQst4QHoY3aw2a0j4XLFMylU0\/g+iDZcqyyU0eWg2ijzz0M40+4DsKanhXrlwpmzdvlj179kh7e3vpkb8sS5bGYRWuDU5I4XjZVhp92JbxcPEG1YeO8r5z9qKzTBmbPQSC6sMGIhjeAhped1my2bNnyz333COrV6+Wr371q\/LMM8\/IiRMnmNKQsyObE1LOEpKz7qCPnCUkZ90Jqo+337soukwZD6PIWQIT7k5QfSTcjVxUj+EtoOF1Q3r88cflvvvuE33ghJreBQsWyNDQkDQ3N2cuPoQ3lQJOSJnLMdcdQB+5Tk\/mnQujD\/dhFO\/13Zl5v+lAOgTC6COdHmXXCr6jwIY3O1nVbxnhYXjrq4QSSoALFjqoRSCsPq5b\/1159O6b5NG7PwJYCwiE1UeRkeA7MLyZ6BvhYXgzEZ6BjXLBMjBpKXY5rD4Y5U0xOTloKqw+ctDlxLqA78DwJiauWhUjPAxvJsIzsFEuWAYmLcUuR9GHjvIylzfFJGXYVBR9ZNjdRJvGdxTU8LIOb6LHTayVc0KKFWfhKkMfhUtprAFF0QejvLGmINeVRdFHrgNqoHMY3gIaXncd3qVLl+Zizd1K+kR4jPA2cN6yalcuWFalO3SwUfXBKG9o1EbuEFUfRgZbp9P4joIaXncd3o6OjoZ1q6PFu3btmrbCw\/r162V4eNipf8eOHbJ48eJSW4ODg7Jt2zbnfW9v7zTjjfAwvA0L05IKuGBZkuiIYUbVB6O8EYEbtltUfRgWZqDu4jsKaHg1JDWpL774ovT19QUSQrVC7tQI\/5Jm+vmmTZucNX1\/9KMflV7rI4uPHj3qPN1tYGDAqdZ97TXfCA\/D25AwLdqZC5ZFyY4QaiP6YJQ3AnDDdmlEH4aFWre7+I6CGF53GsP4+HjNpLe1tQV+8ISO4L788svO+r0TExNlI7z6nW5qqN0HXaxYscIZ5dXR3ZGRkVJ5Les+7c3tHMLD8NY9O1HAIcAFCyHUItCIPhjlLb62GtFH0ejgOwpieJMQpo7iVjKwrsFduHChM1XB\/95rhrVf\/vf6mSu8ffv2iZpwd9MRYts2Tki2ZTxcvOgjHC\/bSjeqjw\/9yX+T\/s+3y7JbP2wbOivibVQfJkPSgUDvpgOCXV1dzoCc13eYHGMjfb\/mypUrVxqpIOt9a82rjdo3\/4itf0TXNbXuKK5\/RFf3HxsbK5te4Rpef5+6u7tl+fLlUbtq5H6XLl2SM2fOiJr9pqYmI2Og08kRQB\/JsS1CzY3q44m\/+Zm88NZ5+ds\/uqkIOIjBR6BRfZgMdM+ePbJ3795pIWB4J5EYbXjVaP7gBz9wpizo44TVPH7hC19oeKWGJA3v1q1bZc6cOSVBqumzbZT3woULTr70L04Mr8mn12T6jj6S4VqUWuPQx+wNr8mX7mqTL901tyhYiOMqgTj0YSpMHeH1jvKOjo5Kf38\/I7xXE2qs4dWk+ldmqLa6gl+8epOZmmOdp6tbpRUXvHNy45zSwF9azNE09WSaVr9t\/kkyLcYmtxOHPp566Sfy1EvH5YcbPyE3XjfDZBz03UcgDn0UBSpzeMszaaXhrSdm\/wivlvdOW6h005p3CgM3rdUmzAmpngLt\/h592J3\/etHHpY+PbXpNbp8303kCG1txCMSljyIQwfBieOvquJLhZVmyutgCF+CEFBiVlQXRh5VpDxx0XPrY\/8Yp0VUbGOUNjN6IgnHpw4hg63QSw4vhravjSobXHeXlwRN18dUtwAmpLiKrC6APq9NfN\/g49cEob13cxhWIUx\/GBe\/rMIa3QIa3s7NT4lyHN0lxI7wpupyQklSa+XWjD\/NzmGQEcerDHeX9Vs8tckf7zCS7Td0pEYhTHyl1ObFm8B0FMbyJKSShihEehjchaRWuWi5YhUtprAHFrY\/Pbj\/i9O+FNbfE2k8qy4ZA3PrIJop4WsV3YHjjUVLIWhAehjekZKwtzgXL2tQHCjxufbwydk7uHTwijPIGwp\/7QnHrI\/cB1+ggvgPDm4l+ER6GNxPhGdgoFywDk5Zil5PQh47yvnP2onMDG5vZBJLQh6lE8B0Y3ky0i\/AwvJkIz8BGuWAZmLQUu5yUPq5b\/11Zdmsry5SlmMskmkpKH0n0Nek68R0Y3qQ1VrF+hIfhzUR4BjbKBcvApKXY5aT0wQ1sKSYxwaaS0keCXU6sanwHhjcxcdWqGOFheDMRnoGNcsEyMGkpdjlJfTC1IcVEJtRUkvpIqMuJVYvvwPAmJi4MbzC0nJCCcbK1FPqwNfPB4k5SH2+\/d1F0bV6mNgTLRR5LJamPPMaL7wieFWMfLRw8xHyU5C8tRnjzocT894ILVv5zlGUPk9YHUxuyzG7jbSetj8Z7mF4N+A5GeNNTm6clhIfhzUR4BjbKBcvApKXY5TT0wdSGFBMac1Np6CPmLidWHb4Dw5uYuPhpIRhaTkjBONlaCn3YmvlgcaelD53aMHfWDB5IESwtuSmVlj5yE3CNjmB4MbyZ6BThMcKbifAMbJQLloFJS7HLaenDfSDF7fNmYnpTzG+jTaWlj0b7mcb++A4Mbxo6m9YGwsPwZiI8AxvlgmVg0lLscpr6cE3vo3ffJI\/e\/ZEUo6SpqATS1EfUPqa1H74Dw5uW1sraQXgY3kyEZ2CjXLAMTFqKXU5bH9zElmJyY2gqbX3E0OXEqsB3YHgTE1etihEehjcT4RnYKBcsA5OWYpez0Mea\/W+JGt9v9dwid7TPTDFamgpLIAt9hO1jWuXxHRjetLTGCG8V0pyQMpGgMY2iD2NSlUlHs9IHpjeTdIduNCt9hO5oCjtgeDG8dWV28OBB2bVrlwwNDUlzc7NT\/tSpU9LZ2Snj4+Ol\/Xt7e6Wnp8d5Pzg4KNu2bXNeez93CyM8RnjrCo8CDgEuWAihFoEs9eGaXub05lejWeojb1TwHRjemppUs7t69WpZsGBBmeE9evSobNiwQXbu3Cmtra1ldeh3a9eulYGBAedz93VHR0epHMLD8ObtZJjX\/nDBymtm8tGvrPXx1Es\/kadeOs7T2PIhh2m9yFofecKC78DwVtXj+vXr5eWXX3bM7sTERJnhrTTq61ako7sjIyOl8lpPe3t7afRXyyE8DG+eToR57gsXrDxnJ\/u+5UEf7o1sLFmWvR78PciDPvJCBd+B4a2qRTW1ixcvdqYneA2s7lDpM7ciNbi69fX1Of\/732N4y5FzQsrL6TCf\/UAf+cxLXnqVF324S5bdeN0MGVg6n5vZciKQvOgjDzgwvBjeujqsZG7VxA4PD5f29U558I\/o6v5jY2MlA+w1vPv27ZO2trZSPf7pEXU7V4ACnJAKkMQEQ0AfCcItQNV50sfb712Udc\/+D3n12DnReb1fumtuAQibHUKe9JE2Sb3XyLvpPUddXV3OAJ7Xd6Tdr7y0d82VK1eu5KUzeemH3\/CeP39eVq1aJbNnz3ZMrP99GMPrj7G7u1uWL1+el9BT6celS5fkzJkzzlzopqamVNqkEXMIoA9zcpVFT\/Ooj2+MnpNvHj4ns3+tSZ749K\/L78yZkQUa2hSRPOojrcTs2bNH9u7dO605DO8kEisNr95kpkZT5+nqtmPHDmcqg7vVmr7gltHpD5s2bZIDBw7Ili1bnI+DTGnYunWrzJkzp9SWmj7bRnkvXLggp0+fdv7ixPCmdSo0px30YU6usuhpXvVx4p8vl0Z7b\/uNZun\/\/G+KTndgS5dAXvWRBgUd4fWO8o6Ojkp\/fz8jvFfhW2l46wkvqOF1ly7Tv6i8Uxi4aa02YZt\/cqqnPb5nWTI0YPb5Q1dx0JvadLoDy5elr2auL1PMmcNbrj8Mb4Xj0W943TV4N27c6IwEu++XLl3qrMTAsmThTmqckMLxsq00+rAt4+HiNUUf7vJlOsq77NZWWXbrhxnxDZfqSKVN0Uek4ELuhOHF8NaVTKURXv+DJ5YsWVJ2UxoPnqiLtVSAE1JwVjaWRB82Zj14zCbpQ0d5979x0lm3Vzc1v4\/e\/RHHALMlQ8AkfSRDgBHealwZ4U1acVfr5y+tKdCckFISnaHNoA9DE5dSt03Vh3eqA6O+yYnFVH0kQQTfwQhvErqqWyfCw\/DWFQkFHAJcsBBCLQJF0Ic73cEd9WXKQ3yaL4I+4qKB78DwxqWlUPUgPAxvKMFYXJgLlsXJDxB6kfShD6949djZsikPan5vnzeLB1kE0EKlIkXSR0QEpd3wHRjeRjUUaX+Eh+GNJBwLd+KCZWHSQ4RcVH245tdd4cE7+osBDi6QouojOIGpkvgODG8U3TS8D8LD8DYsIksq4IJlSaIjhmmDPtyb3RSRe8Ob1wDra0xwZQHZoI+ghw6+A8MbVCuxlkN4UziPHz8uu3fvdp5ex+MOY5VZISpDH4VIY2JB2KgPNcD6+OK337sgkyPB50p89Qa4ubNmOFMgbrzu2tLrxBKQ84pt1Ee1lOA7MLyZHK4Ij59ZMhGegY1yrBiYtBS7jD4mYdcywe5osBphNcST\/ybNsPs+xZSl2hT64FpbTXAsS5bSochByEGYktSMb4ZjxfgUJhoA+qiOV02wbu5osL53jbF\/L\/exx34TbLoxRh9cazG8iZ6C61ce90E4Pj4uzz33nNx\/\/\/2BpgXUK1\/r+0rfBfnMW0YJuf3Vz7u6umJ9vne9+PwZqlc+TR5eNnFN8agXX5552KAPf35qvY9bH3FrQ\/tXrc6gn4fhEbc+TOcRRh+uAX7nrBrhC6WRYv188rNJw1zNHOvnrkHW165p1te\/9PP\/6VyL9Ht3c7\/\/pZ\/\/rOL1ykZ9pHltiftYqe908l2CEd6U8uMa3n379gUyqPW65Qo5aH31ytf6vtJ3QT7zltF41ORqf72v4zR4bv1B6oRHucLgkS4PP+9a7+M+XurlutIfQ\/WOrWp1Bv0cHpODAO75PGseS1c9JFu3bpF\/\/dVfd0yw1whPmuMps6x60XJBN69J1n1O\/H9H5bbbbpu2+xvf\/bYzoONu7oCJ7j8+\/lPHQK9b95Dztfv+4f\/0H533\/f39sm7dutK11mvCvQ35+1IpBn8u1LzX2uodX1lca0dGRmLxHUFznNdyGN6UMqMif+SRR2R0dDSlFmkGAhCAAAQgkC4Br\/n911\/9N07j5Z+Vm2O3jLeXtQx0pfJhTXe6ROq3Vs9E16+hdomf7fh8o1UUYn8Mb4ppVNOr\/9ggAAEIQAACEEiHQG0DHXx02u2tTv+Ic3Onl8RZp7euR+\/+SFJVG1UvhteodNFZCEAAAhCAAAQgAIGwBDC8YYlRHgIQgAAEIAABCEDAKAIYXqPSRWchAAEIQAACEIAABMISwPCGJUb5RAmcOnVKOjs7nbnOS5Yskb6+vkTbo3KzCBw9elS6u7tlYmICfZiVutR6e\/DgQXnxxRc5d6RG3IyGuLaYkacke4nhTZIudYcmMDg46Oyjpubhhx+WtWvXSkdHR+h62KGYBB5\/\/HG57777HE14XxczWqIKS8A1NR\/\/+McxvGHhFbw8546CJzhAeBjeAJAoEp3A+fPnZdWqVbJixQpZvHhxqSI1ttu2bXPe9\/b2Sk9Pj\/Pae1LSMu3t7WX7Re8Je+aRQFh9eGPA8OYxo\/H1KYo21q9fLzfccIP8\/Oc\/ly9\/+cvxdYaackcgjD607NatW+UHP\/iBvPXWW\/w6lLtsptMhDG86nK1sxT0hHT58WHbs2FEyrvqztI7cDgwMOFzc1\/5ROwxvsWUTRR8uEX62Rhv+c4erieXLl8vzzz+P4S2wRMKeO+bNm1f2i6H+YXTPPfcwmFJgjVQKDcNrWcLTCtedazl\/\/nw5ceKEbNy4sXRyUSOrT34ZGhqS5uZm0ZOPjuTqKC8jvGllKNt2oupDe636GRsb4yfrbFOYWOtRtaHnkeHh4VK\/vH9kJ9ZZKk6dQBR96BS57du3y5o1a5xrDoMpqactFw1ieHORhuJ14sc\/\/rET1K\/+6q86N6F5Da9emHRzb0jzvmcOb\/G0UCmiqPrQUTw1u+4UGDto2RVlVG24lNQQMcJbXM1E1Yd3VJfpUMXVR63IMLx25j21qN2bSPyG1x3R9Y\/Yee+k9c7tTa3DNJQqgTD6ePLJJ5354DpFxt0YxUs1Xak2FkYb3tVcMLyppimzxsLqg1UaMktVbhrG8OYmFcXsSNiTUjEpEFU1AugDbaANNBCFAOeOKNTs3gfDa3f+E4++2klJG640pSHxDtFArgigj1ylI1edQRu5SkfuOoM+cpeS3HcIw5v7FJndwUonJf9NR96b1syOlt6HJYA+whKzpzzasCfXUSJFH1Go2b0Phtfu\/CcefaWTUq1lyRLvEA3kigD6yFU6ctUZtJGrdOSuM+gjdynJfYcwvLlPkdkdrHRS0oiqPXjC7GjpfVgC6CMsMXvKow17ch0lUvQRhZrd+2B47c4\/0UMAAhCAAAQgAIHCE8DwFj7FBAgBCEAAAhCAAATsJoDhtTv\/RA8BCEAAAhCAAAQKTwDDW\/gUEyAEIAABCEAAAhCwmwCG1+78Ez0EIAABCEAAAhAoPAEMb+FTTIAQgAAEIAABCEDAbgIYXrvzT\/QQgAAEIAABCECg8AQwvIVPMQFCAAIQgAAEIAABuwlgeO3OP9FDAAIQgAAEIACBwhPA8BY+xQQIAQhAAAIQgAAE7CaA4bU7\/0QPAQhAAAIQgAAECk8Aw1v4FBMgBCAAAQhAAAIQsJsAhtfu\/BM9BCAAAQhAAAIQKDwBDG\/hU0yAEICAS+DUqVPS2dkp4+Pj06Ds2LFDFi9ebB2sgwcPyosvvih9fX2yfv16J3597d2qfe6HpeXuueceKzlaJxwChoBhBDC8hiWM7kIAAtEJuIZ348aNmDIRUR5\/\/Md\/LF\/\/+teltbW1YcPrry96ptgTAhCAQLwEMLzx8qQ2CEAgxwQwvOXJGRwcdD7o6elx\/m90hFfr8NeZYznQNQhAwCICGF6Lkk2oELCdQD3De\/78eVm1apXMnj1bhoeHZcmSJc7P+0ePHpXu7m6ZmJiQlpYW2bt3r3R0dDg4vd\/dddddTpmFCxc6Wk+OiQAABDNJREFUJtJvINUMjoyMyNDQkDQ3NzsjrN4pFu60Crcf2tbhw4edOtva2uTAgQPOSKy\/XbdP+vmGDRtk586dTjm3nhUrVkwb0dbvHn74YVm7dm0pliCGV8soG+\/mctLPdIrErl27SjHarjnihwAE8kEAw5uPPNALCEAgBQLV5vD29vY6BtU1iNoVvyl1p0Goodu0aZNjPnVTw7p06VJnf\/1u9erV4tZXy\/DqvmquXXOsxlnN58DAgMybN8\/57sSJE047ao5dI64G3G\/c3T7t3r3bMbyuwdU6vQbYi7jSd5XMrLuP19S6n3n77P4BoH1buXKlbN68uWSkU0gtTUAAAhCoSQDDi0AgAAFrCAQd4XVNaKURS++oqX7vml\/viGqQEd5jx45NM6NqONvb253RZK8Z1na85tlrut0RXzeJOoo8NjbmjEx7X\/uTXGkkNsgIr1tPNZb1GFsjNgKFAARyRQDDm6t00BkIQCBJAvXMmGtmvYZXTeO2bdumdUtHcdWcRjW83\/\/+953RYP+mI6lPPvlkTcPrnxpRbeR2y5YtVVdNaMTwVuLk9qHWNIokc0vdEIAABGoRwPCiDwhAwBoCUQxvrTmp\/pFWvxGsNaWh0giv3zR6jXfQEV63D5\/5zGfkO9\/5TmkFhjhHeGstU1aPsTViI1AIQCBXBDC8uUoHnYEABJIkUM+MVRq59O\/j3qS2detW+eQnP1k2Euufw6sjsU8\/\/XTZPFyNT+cH6+adtuC2o\/OB601pqNQnd\/6vzqV1R6UXLFhQ9eaxanN4tV+11uGtNbqs+zKHN0kFUzcEIBCVAIY3Kjn2gwAEjCMQxfBqkN6VGPS9e1Oaa\/DclRbmz5\/vMPm93\/s95yY2701yupJCV1eXHDlypOoqDe6NYZWMt39U1d8n74Mz3O++8IUvlJYc8ycryioNf\/Inf1LxwR1eY80qDcYdFnQYAlYQwPBakWaChAAE0iBQa25rGu27bQQdZU1izVz3xjt3bd8046YtCEAAAtUIYHjRBgQgAIGYCOTF8AYdZY37yWhx1xdTWqgGAhCAgGB4EQEEIACBmAjkwfDqCOvLL79c9nCMWuGpOX7xxRenzduNgkTbvueee3hscxR47AMBCCRKAMObKF4qhwAEIAABCEAAAhDImgCGN+sM0D4EIAABCEAAAhCAQKIEMLyJ4qVyCEAAAhCAAAQgAIGsCWB4s84A7UMAAhCAAAQgAAEIJEoAw5soXiqHAAQgAAEIQAACEMiaAIY36wzQPgQgAAEIQAACEIBAogQwvInipXIIQAACEIAABCAAgawJYHizzgDtQwACEIAABCAAAQgkSgDDmyheKocABCAAAQhAAAIQyJoAhjfrDNA+BCAAAQhAAAIQgECiBDC8ieKlcghAAAIQgAAEIACBrAlgeLPOAO1DAAIQgAAEIAABCCRK4P8HBB+CgvvhjZMAAAAASUVORK5CYII=","height":0,"width":0}}
%---
%[output:5907a544]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: GM355L6-250kW\nIM Normalization Voltage Factor: 326.6 V | IM Normalization Current Factor: 610.9 A\nRotor Resistance: 0.00434 Ohm\nMagnetization Inductance: 0.00533 H\n---------------------------\n","truncated":false}}
%---
%[output:312bbcff]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00756 H\nPer-System Quadrature Axis Inductance: 0.00624 H\n---------------------------\n","truncated":false}}
%---
%[output:09b075b4]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Unrecognized function or variable 'rms_perios'.\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('ctrl_afe_setup\/rms_setup', 'D:\\git\\github\\library\\library\\setup_files\\objs\\ctrl_afe_setup.m', 160)\" style=\"font-weight:bold\">ctrl_afe_setup\/rms_setup<\/a> (<a href=\"matlab: opentoline('D:\\git\\github\\library\\library\\setup_files\\objs\\ctrl_afe_setup.m',160,0)\">line 160<\/a>)\n            out.n1 = 2*pi*rms_perios\/obj.omega_base\/obj.ts;\n            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('ctrl_afe_setup', 'D:\\git\\github\\library\\library\\setup_files\\objs\\ctrl_afe_setup.m', 71)\" style=\"font-weight:bold\">ctrl_afe_setup<\/a> (<a href=\"matlab: opentoline('D:\\git\\github\\library\\library\\setup_files\\objs\\ctrl_afe_setup.m',71,0)\">line 71<\/a>)\n                obj.rms = rms_setup(obj);\n                ^^^^^^^^^^^^^^^^^^^^^^^^^"}}
%---
