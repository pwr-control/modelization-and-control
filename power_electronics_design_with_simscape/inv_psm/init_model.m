clear;
[model, options] = init_environment('inv_psm');;

CTRPIFF_CLIP_RELEASE = 0.001;
s = tf('s');
%[text] ### Global timing
% simulation length
simlength = 1.25;

fpwm = 4e3;
fpwm_afe = fpwm; % for PWM
trgo_afe = 1; % double update
fpwm_inv = fpwm; % for MPC
trgo_inv = 1; % double update
fpwm_dab = 3 * fpwm;
trgo_dab = 0; % double update
fpwm_cllc = 5 * fpwm;
trgo_cllc = 0; % double update
t_measure = 0.648228176318064;
tc_factor = 400; % tc is ts_afe / tc_factor
tc_decimation = 1;
delay_pwm = 0;
dead_time_afe = 0;
dead_time_inv = 0;
dead_time_dab = 2e-6;
dead_time_cllc = 2e-6;

glb_time = timing_setup(fpwm_afe, trgo_afe, fpwm_inv, trgo_inv, fpwm_dab, trgo_dab, ...
                fpwm_cllc, trgo_cllc, t_measure, tc_factor, tc_decimation, delay_pwm, dead_time_afe, ...
                dead_time_inv, dead_time_dab, dead_time_cllc);

% fPWM_AFE = glb_time.fPWM_AFE;
% TRGO_AFE_double_update = glb_time.TRGO_AFE_double_update;
% fPWM_INV = glb_time.fPWM_INV;
% TRGO_INV_double_update = glb_time.TRGO_INV_double_update;
% fPWM_DAB = glb_time.fPWM_DAB;
% TRGO_DAB_double_update = glb_time.TRGO_DAB_double_update;
% fPWM_CLLC = glb_time.fPWM_CLLC;
% TRGO_CLLC_double_update = glb_time.TRGO_CLLC_double_update;
% 
% ts_afe = glb_time.ts_afe;
% ts_inv = glb_time.ts_inv;
% ts_dab = glb_time.ts_dab;
% ts_cllc = glb_time.ts_cllc;
% tc = glb_time.tc;
% 
% Nc = glb_time.Nc;
% Ns_afe = glb_time.Ns_afe;
% Ns_inv = glb_time.Ns_inv;
% Ns_dab = glb_time.Ns_dab;
% Ns_cllc = glb_time.Ns_cllc;


%[text] ### Settings for simulink model initialization and data analysis
use_mosfet_thermal_model = 1;
use_thermal_model = 0;

if (use_mosfet_thermal_model || use_thermal_model)
    nonlinear_iterations = 5;
else
    nonlinear_iterations = 3;
end
load_step_time = 1.25;
transmission_delay = 125e-6*2;

%[text] ### 
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
grid_emu = grid_three_phase_emulator('wind_grid', grid_nominal_power, application_voltage, us1, us2, fgrid, ...
                eta, ucc, i1m, p_iron, up_xi_pu_ref, up_eta_pu_ref, un_xi_pu_ref, un_eta_pu_ref);
%%
%[text] ## Global Hardware Settings
single_phase_inverter_pwr_nom = 225e3;
afe_pwr_nom = 250e3;
inv_pwr_nom = 250e3;
dab_pwr_nom = 250e3;
cllc_pwr_nom = 250e3;
fres_dab = glb_time.fPWM_DAB/5;
fres_cllc = glb_time.fPWM_CLLC*1.2;

hwdata.single_phase_inverter = single_phase_inverter_hwdata(application_voltage, single_phase_inverter_pwr_nom, glb_time.fPWM_INV);
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:699911a1]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:7639f7dd]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:5ef42f17]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:39d7aa60]

%[text] ### Sensors endscale, and quantization
adc_quantization = 1/2^11;
adc12_quantization = adc_quantization;
adc16_quantization = 1/2^15;

Imax_adc = 1049.835;
CurrentQuantization = Imax_adc/2^11;

Umax_adc = 1500;
VoltageQuantization = Umax_adc/2^11;
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
parasitic_dclink_data; %[output:2c133dd0]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:39caa129]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:1473b13d]
n_sys = psm.number_of_systems;

% load
b = psm.load_friction_m;
% external_load_inertia = 6*psm.Jm_m;
external_load_inertia = 1;

%[text] ### Motor Voltage to Udc Scaling
u_psm_scale = 2/3*hwdata.inv.udc_nom/psm.ubez;
u_im_scale = 2/3*hwdata.inv.udc_nom/im.ubez;
%[text] ## **CONTROL Settings and Initialization**
psm_ctrl = ctrl_pmsm_setup(glb_time.ts_inv, psm.omega_bez, u_psm_scale, psm.Jm_norm);
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, glb_time.ts_inv);

im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
afe_ctrl = ctrl_afe_setup(glb_time.ts_afe, grid_emu.omega_grid_nom);

kp_udc = 0.5;
ki_udc = 18.0;
kp_idc = 0.5;
ki_idc = 18.0;

dab_ctrl = ctrl_dab_setup(kp_udc, ki_udc, kp_idc, ki_idc);
cllc_ctrl = ctrl_cllc_setup(kp_udc, ki_udc, kp_idc, ki_idc);

%[text] ### Local time allignment to master time
kp_align = 0.6;
ki_align = 0.1;
lim_up_align = 0.2;
lim_down_align = -0.2;
%[text] ### Simulation parameters: speed reference, load torque for energy production application
run('n_sys_generic_1M5W_torque_curve');
torque_overload_factor = 1;
%[text] ### Simulation parameters: speed reference, load torque for driver application
% rpm_sim = 3000;
rpm_sim = 17.8;
% rpm_sim = 15.2;
omega_m_sim = psm.omega_m_bez;
omega_sim = omega_m_sim*psm.number_poles/2;
tau_load_sim = psm.tau_bez/5; %N*m
b_square = 0;
%[text] ### Settings Global Filters
filters = setup_global_filters(glb_time.ts_afe, glb_time.ts_inv, glb_time.ts_dab, glb_time.tc);
%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] ### Diode rectifier
Vf_diode_rectifier = 0.35;
Rdon_diode_rectifier = 3.5e-3;
%[text] ### HeatSink settings
heatsink_liquid_2kW;
%[text] ### DEVICES settings (IGBT)
% infineon_FF650R17IE4D_B2;
% infineon_FF650R17IE4;
% infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
% infineon_FF1800R23IE7;
% infineon_FF900R12IE4
used_device = 'infineon_FF1200R17IP5';

igbt.inv = device_igbt_setup(used_device, glb_time.fPWM_INV, hwdata.inv.udc_nom);
igbt.afe = device_igbt_setup(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
igbt.dab = device_igbt_setup(used_device, glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
igbt.cllc = device_igbt_setup(used_device, glb_time.fPWM_CLLC, hwdata.cllc.udc1_nom);
%[text] ### DEVICES settings (MOSFET)

% wolfspeed_CAB760M12HM3
% infineon_FF1000UXTR23T2M1;
% danfoss_SKM1700MB20R4S2I4
used_device = 'wolfspeed_CAB760M12HM3';

mosfet.inv = device_mosfet_setup(used_device, glb_time.fPWM_INV, hwdata.inv.udc_nom); %[output:0be9f8a6]
mosfet.afe = device_mosfet_setup(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
mosfet.dab = device_mosfet_setup(used_device, glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
mosfet.cllc = device_mosfet_setup(used_device, glb_time.fPWM_CLLC, hwdata.cllc.udc1_nom);
%[text] ### DEVICES settings (Ideal switch)
used_device = 'silicon_high_power_ideal_switch';
ideal_switch = device_ideal_switch_setting(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.afe = device_ideal_switch_setting(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.inv = device_ideal_switch_setting(used_device, glb_time.fPWM_INV, hwdata.inv.udc_nom);
ideal_switch.dab = device_ideal_switch_setting(used_device, glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
ideal_switch.cllc = device_ideal_switch_setting(used_device, glb_time.fPWM_CLLC, hwdata.cllc.udc1_nom);
%[text] ### Setting Global Faults
time_aux_power_supply_fault = 1e3;
%[text] ### Lithium Ion Battery
nominal_battery_voltage = grid_emu.udc_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, glb_time.ts_dab);
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, glb_time.ts_dab);
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
%   data: {"layout":"onright"}
%---
%[output:699911a1]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:7639f7dd]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:5ef42f17]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:39d7aa60]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.000000e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:2c133dd0]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAV8AAADTCAYAAAAmsBCWAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQmYjWX7vzE0fAZjicFnL0lJZai\/KWNJX5SSJUuWyL6V+izZhmRNElEq21VIPlP08VkbNa6IyVJIqEm27AyazND\/+t16p9dxtvec95znfc97P9flcpbnfZbf\/Ty\/uc\/93M995\/rzzz\/\/JCmCgCAgCAgCYUUgl5BvWPGWzgQBQUAQYASEfGUhCAKCgCCgAAEhXwWgS5eCgCAgCAj5yhoQBAQBQUABAkK+CkCXLgUBQUAQEPKVNSAICAKCgAIEhHwVgC5dCgKCgCAg5CtrQBAQBAQBBQgI+SoAXboUBAQBQUDIV9aAICAICAIKEBDyVQC6dCkICAKCgJCvrAFBQBAQBBQgIOSrAHSnd7l\/\/37q06cP5c+fn3Lnzk1nzpyhGjVq0MiRI6lEiRJ+wfPxxx9T+fLl6YEHHvBZf\/PmzdSjRw+qUKFCTn9t2rShzp078\/uZM2fSU089RRUrVvTZllQQBMxCQMjXLCSlHb8RAPnOnz+fhg0bxgR87do1euutt5hMmzdv7lc7Rsl30aJFNGHCBO7vjz\/+oDlz5tCVK1f4j0BUVJRffXqqhMCA+AcilyII+IuAkK+\/SEk90xBwJV+Q4cSJE6levXpUt25dWrBgAS1evJj7g4basWNHfg3CXrhwIWvHlStXpieeeILq1KlDK1eupPfff5+ys7OpSZMmrNGCZLUCzVdPvvj82LFjNGrUKHrllVf42U6dOlFcXBz\/Efjqq6+YmNu3b899Z2Zm0tSpUyklJYU1dLT93HPP0bfffkt79uyh9PR0euGFF7j\/cePGMbljjGi\/SJEi3Mddd91Fq1atokqVKlHjxo1p3rx5dP78eXrttdd4DlKch4CQr\/NkrnzGIF+Q19GjR3PG0rRpUxo\/fjylpaXRxo0bmbBQpk2bRtWrV2fCw+eDBw+mrKwsGjp0KJPjLbfcwp\/37duX8uTJw8RdvHhxQnveyPfSpUtMfG3btiVo0SDf06dPM5E+88wzlJGRwZpyv379aO3atdxP69at6bfffqMRI0bQoEGDmHxTU1N53Lly5aJ33nmH2rVrxyS+Zs0aOnz4MJszevXqRb1796aHH36Y6xYoUID69+9P27dvp\/\/97388p2C1b+VClQEYRkDI1zBk8kCwCLgzO0AzBfFFR0ezJpiQkMDdQGsFuaJAK9Y+h2YMGy40z7Fjx94wJNh3QWhGyRfa9MGDB\/nfzp076csvv6RJkybRkiVLmOhvv\/12Ni\/MmDGD\/vWvfzH5ooCsUUDYu3fvph9\/\/JG15ypVqlC3bt1o9OjRrAUXLVqUNXxtHq44BIurPG8vBIR87SWviBitO9LRPoOG27Bhw5yDNGiWW7Zs4Z\/07sh327Zt\/BNfI0CYC65eveqX2QEmApA0NFZovt999x3\/gzYcGxvLWjfa1TTj2267jceBz5s1a3YD+Z48eZLGjBnDZo9atWrRvn37aNOmTW7JF+YVHBQK+UbEcg54EkK+AUMnDwaKgCvpQJuEdnngwAG67777WJsdMGAANw+CBFn9\/vvv9M0337A5Aq9feukl6tKlC5NhcnIya5d58+ZlrVSvOWvas+uBG+rh5z\/swzA\/gHyXLVuWQ\/AYAw4EofmuXr2aCR5mhyNHjvAYYHrQa76Y09y5c\/kZmBDefPNNHr87zVfIN9CVE1nPCflGljxtMRtXVzMcaN1xxx1MatA43R24QZt97733mGhhHoBd9bHHHmMtE8SNAywU7YBOb0P15mqGZzTyxUEZbLmw38JWC5t04cKF2eQA08bevXvZ9FCwYEHq2rXrDeSLZ6dPn04rVqygqlWrslkC44JmjbHpzQ5CvrZYpiEfpJBvyCGWDuyOwLp166hcuXJMvKdOnaIpU6aw5o2DPSmCQKAICPkGipw85xgEfvrpJzZrwK4Lrwdo6PHx8Y6Zv0w0NAgI+YYGV2lVEBAEBAGvCAj5ygIRBAQBQUABAkK+CkCXLgUBQUAQEPKVNSAICAKCgAIEhHwVgC5dCgKCgCAg5CtrQBAQBAQBBQgI+SoAXboUBAQBQUDIV9aAICAICAIKEBDyVQC6dCkICAKCgJCvrAFBQBAQBBQgIOSrAHTpUhAQBAQBy5EvwgV+\/fXXHBEKWQNQSpYsyeH8HnzwwRvitIr4BAFBQBCwKwKWIV\/EdN2xYwcHtq5fvz4Hmy5UqBDjCkJG7NTPPvuMw\/vheymCgCAgCNgZAcuQ78WLFzl9y9133+0xCyyy3CJDQLVq1eyMuYxdEBAEBAGyDPlCFiDXX375hTO+IsvAhg0b6IcffuBA2w0aNOBwflIEAUFAEIgEBCxDvkgH8\/bbb9Py5cs5Cy0IFwSMNNuwARcrVoyQGBFZBqQIAoKAIGB3BCxDvmfPnqXXX3+dXn75ZUJaGQSvfvXVVzl3FkwSb7zxBqfxRpoZKYKAICAI2B0By5DvmTNnOEdX\/\/79+YBNn24b79966y1ORoj021IEAUFAELA7AkK+dpegjF8QEARsiYClyHfo0KGcmRb235SUFEpMTGS\/3qysLPaEGD9+vGi+tlxmMmhBILQIZJ9Mp993p1BMYufQdmRi65YhX6Te3rVrF+F\/dwWeDjVq1DDd46FSpUomwilNCQKCQDgRKJUvmxoXv0iNi2VwtzszomlyegmfQ0BSVNXFMuQLN7Pz588TLlu4K\/ByKFy4sEcf4ECBBPmqEoTKvgPFS56LDATsvPY0LTcjZT5l7k6hqBIVWOONqd+JX\/sqVpm7ZcgX3g4jR46kw4cP04ULF+jUqVNUocJ1IE+fPk0tW7ak559\/ngoWLOgL2xu+B5l\/\/vnnNG3aNNaacWjXrFmzHBJXKYiff\/6ZKlasaGg+UlkQMAMBO649kG7GF\/Pp7CdJTLL5qydSTGIniq6eaAgSlXteP1DLkK82KJgdZs2aRa1ataIyZcrwxyDkpUuXUq9evQybHfbu3cteFPCegPaclJREHTp0oHvuuYfbVikIO24AQ6tcKlsWAbusPXdabmzrUUHZdlXueUuTL1zOEN9hwIAB9I9\/\/IPHeunSJdZce\/bsafjA7eOPP2ZTRps2bbitxYsX8\/\/ae5WCsMsGsCyDyMACRsDKaw+Em30inWBWyEiZl2NWyF+9nmEt1x1AKve8pcn3ypUrNG7cOP453qRJEx6rds14yJAhhjXfiRMnUnx8PF9PRklNTaVNmzbR4MGD+X2plmOpRYsWAS\/iYB7E5RGjZpRg+pNnzUXg1zOZbhs8dPbvzw95qOPuwXJFo6ltfBy1jS9FeB3KYkXy1cwKIFwUmBWiq9cLSssV8jW4iqDpQkOFrRZmCFwx7tKlS06UMyPNgXzr1q1LCQkJbsn31o7v8OeaGcJI28HWxdwkXkWwKKp7vnShKLedl465\/nn3OkX8GtzRC9l0LCOb0g5n0rYjmZR2JJPQdlKj4nR\/mdCQMEx5ZcuW9Wt8Ia109jDRwS30Z9p\/iA5uJootS1SrBeWq1eL6a5NLw4YNuUVVh+z66VjG5gsiysjIoOLFi3uFGwdxvuroG3A1M+A9CK958+ZcTeVPECtqHyavdWkuQASavb2dUg+eYy347bbmR\/FTvfbMOjwLBF6Ve96S5IuLFQiqg5i+CJyOyxa4YIGC68V79uzheL41a9akp59+2m\/MceC2YMECGjZsGD\/z2muvUceOHXPCUqoUhOoN4DeIUlEJAqkHzlGzmdvZBLFj+IOmjkHF2vPkIoYDtHAWlXvekuSrDerkyZOEQ7Lk5GTCAkFBhLOnnnqK3c2MxnbQXM1wiIfXffr0YVuyFh1NpSBUbIBwLnLpK3gEYDPuu2gva8HLe99LCVX8M2X46jlcay\/Uh2e+5unue5V73tLkGwiYwTyjUhDh2gDB4CPPWgOBiavTaeLqn2nwoxVp8KO+LxL4GnWo1164Ds98zVPINxCEwvSMkG+YgJZugkZAM0MkVC5Cy\/vcG1R7oSTfo0n1A7p5FtSEDDyscs+L5qtDQKUgQrkBDKxFqWojBDQzBNzZYIYI1CXNzLWn8vAsENGp3PNCvkK+gaxZecYiCOgJONCDuGDJ1yqHZ4GIRMjXC2rwwRs+fDh7OeBiBbwgXnjhBc5q4a3s3LmTL2jAHQ1Xk3GVGJc1JLZDIEtUnrE6AsHYgQMhXysengUiIyFfD6jB3xcXIxo1asReDyNGjKDdu3dzfF9vN9zgIwx3MngzVK1alTZu3Ehr1qzha8Rz586V2A6BrFJ5xvIILNp6nPos2ktn3qhvaKxGyNfKh2eGJv1XZSFfD6hp6YSeffZZmjRpEo0aNYr9fX2lETp06BB98skn1Lt3b65\/5MgRzgFXu3ZtTsQpsR0CWabyjB0QCOQgzh\/yxTVffdjGcjOvu37avQj5etF8Z8yYwZrvnDlzmHyPHz9O8+bN45CT\/sRCwIWNjz76iLNfREdH0wMPPOAxtoNKQfizAey+0GX84UFAbwee0aaaT39gT2sPWu7ZJaNzAtogvkKJPnPDM4kw9aJyz+unaJnrxfpBweYLE8L333\/PlypwHRiEfPvtt+dUg4bctWtXgp0XKeW1QDnQgMeOHcv2YWRCnj17ttfYDlomi\/Xr14dJ9H93Y5n79WGfuXQYKgSS1p2iFXsvclyIJ6p5jn19w9o7e5j+3PYfop+25MRXyPXIAI6xEGlFYjv4IVHcRoMd9+rVq35lsNBuskHjBREjUE7u3LlvCiEpsR38AF+q2BoBzQ7sLS4ENN\/iv2ykzN0bQxK20coAiubrIh19Jgt3gkMEpjFjxlBsbKxbucI0ge+hMWtB2FFRYjtYeRvI2EKFgCc7sDuf3EgzK\/jCVMjXBSEthxs0XbiWwdyAUJAo8HSA7fbxxx\/PicngCvDWrVupe\/fuVK5cuZwUQSBsZLBA\/F6J7eBrScr3kYaAZgfOOplOE+K2UIljW3JunmXXfJIqdX8z0qbs13yEfD3ABA0Ydtq+ffvekMkCNl+QqyfN1y\/U3VRSKQg5cAtUavKcLwRcL0EcjSpFu8o+Sb0nX49f7eS1p3LP6+VmuQM3ZHeA+QDJMrUDNhwOwNUMFy8KFSrka90Z+l6lIJy8AQwJSSr7jQDcw\/R2XC3JZPLlO9gfWIsL4eS1p3LPW5p8MTiYEOBiljdvXh4rMlsgDm+dOnX8XoT+VlQpCCdvAH\/lI\/W8I6BpuHrCjbq1ApVO+uKmB2GGqDn2a44HMTyxCLVMMD9Iux3kpXLPW558MUD46uKaMGzByFyRL1++kMhVpSCEfEMi0ohv1BPhIo16TGJnr\/PXxwc2eisuUoBVuectTb6evB58eTsEujBUCkLIN1CpOe85d4FsoOH6Q7ju0Bq6eDu9+8050+ID20kiKve8pclX83qA3y7KhQsXOHtxVFQUtW\/fnvLkyeNTzvv27aMJEybQlClT+IBOAuv4hEwqWBABvf0Ww4sqUYEz+prhGoY\/\/EeuxnKaIjPiA1sQPo9DEvI1IC3tujCSXvo6cNM0Z6QjmjlzJv3222\/03nvvSWAdA3hL1fAioJEseoVbWObuFB6AmWTrOiPtV5fRa8nhRSY0vQn5GsAVQXLmz5\/P7mfeyBckDX\/e8uXL07p16\/jQbu3atZy7TQLrGABcqgaNAAg1+8Qv3A5MBhqx8vsT6TmfaR2BaGFGyFuigimara8JuJq84AmBm3FmpSny1b\/K74V8PaCvj9mgVUECTdfEl+4eT01NJZgcnnzySY5oBvKF1hsfH+8xsM5PrXKpXAfSd6QiEFv275kV\/et1bJnrn8WWpVz4XmHsBHdxRTgmxLpTlNYv+BxxVhWrxHbwIhnYfC9fvswXLLQMw9Bc4W6G0JCI14DiGlinU6dO9MEHH1D\/\/v0pKyuLzQwa+eKmXEJCAj8HgsaNNy0Qj8q\/gnLgZtUtGvnj8rT2YIaAHRjFn+hodkRK5Z7X42WpSxbIXIEYDQsWLOBLFojLi3L+\/HnWYBGlzFPq+M2bN1O7du1uWAulS5emZ555hiOc6c0OiJIG+zGKSkEI+dpx60bGmL2tvVClq7cKcir3vGXJF5crYKNNS0vjCxXaJQsMuHr16mw6gNeDrwKtWNN8ceAGMkfAHRRc1ujYsSNVq3bdwVylIIR8fUlSvg8VAv6sPS1NkbfoaKEaXyjbVbnnLUu+GBjSCKWnp1PlypX9Ilp3QtKTr+ZqJoF1QrmcpW27IeAP+WJOWnQ03IoLNFmn1bAR8nWRCFzEkLMNh2VIgokDAX2RSxZWW8IyHjsj4C\/5Yo5mZEu2ElZCvhaRhkpBGNkAFoFLhhEhCASy9oLJlmwl2FTueUubHTA4mA1WrVpFJ06cyBlr4cKFqXXr1n7lcDMiaJWCCGQDGJmb1BUEPCEQ6NoLJFmn1aSgcs9bmnwRUhKHY0gDpM\/ZBg+FGjVqcD43M4tKQQS6Acycv7TlTASCWXt2N0Oo3POWJl8tdTz8dTVXM3+3B64UI3kmUgfFxcWxn2\/FihUltoO\/AEo9xyAQDPlqIDV7ezulHrRfcB4hXw\/LHFeE4RqGdO933nmn35sBz02aNIlq1apFjzzyCH355ZcE39+mTZtyCnq4nuHSRlJSEnXo0IE1axSVgjBjA\/gNkFQUBHQImLX2NDuwnYLzqNzzltZ84fWA22fI2\/bPf\/4zJ5aDL2+HY8eOcRSzkSNH8jMgY9yUg+1YYjsI7wgCNyJgFvmiVbu5own5mrwb9u\/fT8jzhoO59evX03333UdDhgyhDz\/80GtsBwgCBc+Eu7i7Xx\/uMUh\/zkTA7LV39EL29bgQRzJp9tOl6P4y0ZYEVmI7eBELDtyWLFnCV4r1BbfdEKPh7rvv5ssXrrEdcPsN149nzZrF5oqNGzfS0qVLqWTJkpSYmCixHSy5FWRQqhAwU\/PVz8EO7mii+XpYdbjhhgA5CCPZuHFjQip5BEMvU6YMmw+QQh6hJV0LNF+EnYSnBA7qtFtu8JBAkB6J7aBqm0u\/VkQgVOSrN0NY1Q4s5OthRSJzBbRXhJAsWLAg18JnMClokctg13UtWtbjrl27UtWqVTkJ52effcaBdRYuXCixHazIADImZQiEknwxKSsHaRfy9UK+U6dO5dCQiMuAgkO4119\/nXr27Elz587lQzV35aeffmKvBricITYEbL6IbAbNWWI7KNvn0rEFEQg1+WpTtqIZQsjXy4JctmwZLVq0iN3NEL93x44d9Pjjj3P+tqNHj7o1OwS6vlUKIlwbIFBs5LnIRSCcaw8ZMpApwypmCJV7Xr+iLBXPVz8waK+w4yK4Ouy9SB+P17Df+hNW0t9to1IQ4dwA\/uIh9ZyBQLjXnpWCtKvc85YnXywMmArgq4uCQ7gDBw7Q5MmTc0wRZm0RlYII9wYwCzNpx\/4IqFp7VrgVp3LPW5p8kc0CAc9r165N27ZtY3\/dX3\/9lWrWrEkPPfSQ6atepSBUbQDTQZQGbYeAyrWnOki7yj1vafLVx3ZAVgsABbMD0gj17t3ba1QzeDggFjA05dtuu42GDx\/O5gpo0dOmTeOgPN26daNmzZrl5IJTKQiVG8B2bCEDNhUB1WtPfytuee97CcHaw1VU7nlLky9cxmBeAEkiowWuCOPgDVeHBwwY4DGHG9zR4OkwcOBA9nCAm9nBgwepSZMmTNwS2yFcS1v6sQMCqskXGKnKFSfk62WFQoPdsGEDtW\/fnl555RUmUVyS6NWrl8fDNpAv6vbr149DUWq35HDd2KqxHayyCOxAFjJGcxGw0toLtzuaVeZuWW8HbalpAXJiYmJyUsl7Wobr1q1j8oXZAVHLcDHD39gO5i5taU0QEASMIJBdvCpdTBhERT7tauSxgOviToDqYhnyxUUKXJ5wzd2mAeQa1cw1tgO05DFjxnAbsBEjKhriOyAmRL169TzGdlAtAOlfEBAEriMAM0Q4bb+qcbcU+cKmm5WVxemC4O2gD6aOWLwwIeDShbuC2L0rV65k8oUfMPyEQcbwkvAW20G1AKR\/QUAQcCYCliFfwI9LFAioA++E5ORkThvUsmVLdjfLly+fVwnhcG7ChAkcLL1UqVKs9eLf008\/TR999JHH2A7OFLvMWhAQBFQjYCny1YOBQzJcJV6+fDmtWLGCPR7gyaAF23EFDvWRveLNN99kmy+M6kOHDvUZ20G1AKR\/QUAQcCYCliVfvRbsD\/k6U3wya0FAELArApYi32DMDnYVgIxbEBAEnImAZcgX3g44cDt9+jQ9\/\/zzhg\/cnCk+mbUgIAjYFQFLka8RVzO7Ai7jFgQEAUEACFiGfEUcgoAgIAg4CQEhXydJW+YqCAgClkFAyNcyopCBCAKCgJMQEPJ1krRlroKAIGAZBJSTL0JIXrlyhQHBLTZPlygsg5gMRBAQBAQBExBQQr7w5920aRNHHTt16hQVKlSIp5KZmcnxG5A2vm7duh7jOJgwb2lCEBAEQoBARso8n61mn\/jFY53sk+k+nzejQok+c81oJqg2wk6+SBP0wQcfEKKUNWrU6CZNF9+vXr2atm\/fzgHQQ11wDVmKICAIED1a7CKVvCWbSubLYjhK5bueQxGf6d8Hg9XxK1FeH\/\/tD+\/fB9O3\/tnm\/\/vdrKYCbifs5AutF\/98ZSBGHU8RzAKerZsHVQZWVtm3mRhKW9ZHIHN3CmWdTKfM3Rv5\/y2bN9M9MZk3DTyqRAX+LOrWCpRXe\/3X\/7GtR1l\/on6M0Cr7Luzkq2HjKX5vdHQ0NW7cmFq0aJFjjvADz4CrqBSEFVK5BAycPGhpBPDzH0T7++4U0n7Ka8Sav3oiZVzMoEqDl1p6DqEanMo9r5+TMvK9evUqh3pEUssGDRpwqp9PP\/2UxwaTRFpaGo0YMSJU+Oe0q1IQQr4hF69jOgDZZqTMJ2i4rLmWqMDaa+mkL9xi4OS1p3LPW4J8ofnOnj2b+vbty8HOUS5dusSHcM8++yzNnz+fc7IZKR9\/\/DGHkdTK+PHjOTC7ZC82gqLUtQMC0GYzvphPv+9JYcIF2UKj9fcgScjXwWmE9FmKoemiIJA6UrwjRTzId9QoYzamd955hxo2bMhp47Wyd+9eyV5sBzaRMfpEwB3hxiR2pvzV61F09USfz+srCPk6mHyxEJClGASLPGs4XMvIyOBMFMePH2czRKtWrfxeUPAVnj59OrVr147i4uJynoM2bNXsxU7eAH4LViqS3qQADReEG+zhl5PXnuPNDthTIMVffvmFsBDKly\/P9l8kv0TGYl\/eEK57EiYLuKYhAef+\/fv50A6ZjKFBx8fHs10ZJTU1lX2MBw8ezO9VCsLJG0A41TsCmpar+c0aMSn4g62T157KPa+XjbIDNwxi2bJlnCIIh2+DBg3iFECdO3fOyTTszyLS6sCMsXbtWjY7IM08fIW\/+uorKlCggNfsxZqf7\/r16410Z0pd\/KHQTC6mNCiN2B+Bs4fpz7XTiLb9hyi2LFGtFpTrkQGmz8upaw\/8gOLo1PEXLlygqVOnUocOHdjOC\/MDTAc4cAMRa7feAl11\/mYvVvlX0MnaR6ByjcTnoOXCJUzzVoBZIbp6PTYvhKo4ee2p3POW0Hw1b4f27dvT5MmTmXwR2wGvkdGiaNGihtYdDtZw4IZ08SDulJQU1nx9ZS9WKQgnbwBDwo3Qyppp4ewnSeytALKNqd+JX4e6OHntqdzzliBf2HvnzJlDuE68a9cu6tWrF+GnPy5Z9OzZ07DNF+2tXLmStWmQODwehg8fTsWLF2dXMxAz6iBuRJMmTShXrlxi8w31Dpf2b0LAVcs16wDNKNRCvg73dkCK9zVr1rDtF0F1mjZtyjfb8ufPb3QtBVxf5V9BJ2+AgAVm4wePJtUPyCc3FFN28tpTueeVar6I2XD+\/HnWQt0VaKSIbBaOuA7oX6UgnLwBQkEoVmoz2EsQoZ6Lk9eeyj2vlHz1MR1w6IaQkhUqXLdxIXNxy5YtOXuxWXF9QfJywy3UW1nah0sYQiVqN86AiHbrLCaxk+FLEKFGVMjXwWYHmBxmzZrFFyng24sC95elS5ey\/Rc+v2YUueFmBorSBhCANpt9Iv2G6GBaLAU92YbaU8EMaQj5Oph8z5w5w4dg8GzQx3aA2xkO3Ix6O3hakHLDzYyt6ow2tAsN0GBBtAi9iKInWI1k8T8uPoBoEXrR6PVe1YgK+TqYfOHTO27cOKpYsSJ7H6Bs2LCBfvjhBxoyZIhpmu\/EiRPlhpvqnW6w\/7NL3AfR9yfLgUaY+i6hrWrFVxvu4tnaQZM1CDHfKsXec2JxrM1XL2xcCV68eDHbZGGGwJXgLl26BH3BQt8HyBcpiRISEvhj1+vFP7W67nImJfwIHI0qZbjTY3m8P3M0quRNbR77q5+0aj34u9KFrmdLKB3zd9aE7nWKGB6LnR+QG24O1HxBsgigA\/9bbwUHcb7q+LP4Qe4obdq04f\/xHvbk5s2b83uVfwWdrH34I7tg60xcfWM+sENnrqeO+fXM3xkcDp3NpEO69\/o+yxWN5rflYqOpbpVYfj340dBfgAh23v487+S1p3LP62UT9tgOCJqzfPly2rFjB8farVy5co5fLy5c7Nmzhz777DOqWbMm304LtuDAbcGCBTRs2DBu6rXXXqOOHTtStWrVhHyDBTeCn1+09XgOKYO0QdipB8\/lzFgj5rbxcbYkZCFfB2q+2upF7AUchiUnJ7P9CeWOO+6gp556it3NzDpw01zN5IZbBDOloqlBs9504GwOKYOQ7ULGQr4OJl9F++WmblX+BHHyBrCK\/M0cB8h40dZjrDFbnYidvPZU7nmlZgczF7sZbakUhJM3gBmys2obIF+YLSau\/tmyJOzktadyzwv56hBQKQgnbwCrEqfZ49K0YbS7Y\/iDZjcfcHtOXnsq93zEkq8k0Ax4L8qDIUYAJAxNOKFyEVre594Q9+a7eSFfh9t8EecBYSXT09M5ngM8E5588smAo5pJAk3fm05qqEMA5ohmM7fzAGa0qUYJVdT5Fgv5Oph84XI2Zcpa2dXiAAAO30lEQVQUdjXbuHEjp4lHGiAE2wkknq8k0FRHKtKzMQSavb2d4F+skoCFfB1MvlpsB\/jcapkscPkh0NgOwSTQxNaRHG7GCERqB4dA92XHKe1IJqX1U3NpQ264OZh8kfBy5syZ7NOr5XDDJQu8RgYKXzncQN5du3alnTt3Uo8ePThDRaAJNFUl03Oy9hEcdUXG05oGrOIgzslrTw7c\/oqzANPDuXPn6P7776etW7fyDTQtDkMwW0wSaAaDnjwbLgRqjv2ary+H+xBOyNfBmq+2uKEB79+\/nwPrVKlSJeB4DpJAM1x0If2YiUDqgXN8CDf40YphvaYs5Otw8oWmC1MB0scj9gJ+\/g8cODCgmA6SQNNMSpC2wokALmT0WbSX\/YC1mBGh7l\/I18HkC4130qRJ1L17d85gsW\/fPmrWrBm99dZb9OKLL\/q0+Zq1OFXaf5y8AcySX6S0A\/svSrjMD05eeyr3vH69hj2qmdY5Dszee+896t+\/P3344YdUq1Ytuv322wP2dgh0E6oUhJM3QKDyitTn4AMM++\/y3veGxf\/XyWtP5Z63BPnCLxeBzi9fvkzHjx+n8ePH84Hbt99+61cmC\/gJz5gxg9PN33bbbTwnmC1gvkAizoceeohTFMXExEgCzUhlrAibl3YL7swb9UM+MyFfB5sdsLrgm\/vdd99RyZIlqUSJEvTFF19QvXr1fJocNm\/eTIsWLaJdu3bR7NmzmXxhxkhKSqLOnTtT9erVacmSJZyiHiQMDXv06NGEtPSoAxvzPffcwwtc5V9BJ2+AkLOLTTuA9lu3chF6u+31eNOhKk5eeyr3vCU0XwwC2iuyWuCwDAV+vsg0gVRCsbHXMwe4K7APHzlyhAn2pZdeYvKFx8TcuXNZ80VCzh9\/\/JGvLoOI8+bNe0MmC7SpZbZQKQgnb4BQkYrd29W8H0JtfnDy2lO55y1BviBexGL473\/\/y1eKixQpwiaI5557jtq1a0dRUX\/n13K3oUDU8Anu1KkTky+04VWrVtGIESP4Wc3Pt1ixYvTwww9TgwYNuBnXHG4QBIrccLM7bUXO+HH77VhGNq3oVDZkk5Ibbg42OyCozvTp09m1DFmLkdYHGipcz0DAevKFbfjdd99lU8EHH3zAWS7ckS\/aGTp0KJsX9OTbqFEjjwk0Vf4VdLL2ETJWiYCGtcO3UPr+OnntqdzzltB89eSLwDrQUEGucDXr1q2bzzRCruSrmRmg+Wpmh48++oi1YhC5JNCMAFZy0BQ0399QmR+EfB2s+cLOC20Wh27QTN9\/\/32688476YcffmBzQsGCBb1uNVfyxYHbmDFjON4DXNZgD8atufj4eEmg6SDSiqSphtL3V8jXweSLTXLt2jW28xYoUIDS0tI4nu9jjz3Gng++iiv5oj5czeDNANe1xMRE9iGGFvz555+zfRmEjwA8TZo0YdMEisqfIE7eAL7kK98T54KD90MoArA7ee2p3POWMDtgEDh0g6aLwDpaQVjJGjVqEP4PR1EpCCdvgHDINhL6CJX3g5PXnso9bwnyhUkAZgJ4OmgeBxhY4cKFqXXr1j7NDmZtLJWCcPIGMEt+Tmmn6MAvTL395uS1p3LPW4J8tWDquIUG04DR4u6Gm+RwM4qi1LcLAgi8g0M4s26\/Cfk62OYLzXfWrFnUqlUrKlOmjKE94O6GGxqQHG6GYJTKNkNAI2AzXNCEfB1IvvBK0K7+njp1ilJSUqh+\/frsauav2cHdDTfJ4WYzJpHhBoSA5oKGQ7gZbasFHIJSyNeB5AsPB8Rc0K4Uu65AeCHA7ps7d26vi9PV20FyuAW0l+UhGyJw9EI2Ja07FVQOOLnh5kDyxVqHKxjytyGoDlIG+Yrl4M8NN2jUksPNhkwiQw4YAS0KGgKwt42PM5QJQzRfB5IvbL2vvvoqx++tXbs2++DC\/DBo0CCf8Rz0q9Sdn6\/+e8nhFvCelgdthgBIeNHWY+wXrBExIqMlVCnicSZCvg4kX1cvB7xHRotXXnnFZyhJb+QrOdxsxhgyXNMRAPnCJjxx9c\/cNogYyTnrVonlMJV6MhbydSj5ahks8ufPTyBfxNodNWqUz3gO3shXcriZvpelQZsjAI1404GzlHrw70tMGiFXL0Y0vs29Np9hYMN3rJ+vPn1QMOQbGOw3P6VSECr7Ngs\/acc+COC23KaD5+jQmd\/pk1UbKbt41ZsGryXwhMb8z6LRf2nQ+Q3Zk62OiFX2XdhzuIF8Efxm586dbmWkDxsZDiHqb9eFoz\/pQxCwMgKZdzzJw7tWoBhdK1A85\/X1z66\/D6bkvnzK6+O5L58Opnm\/nz2xoKffdUNVMezkG6qJSLuCgCCgHgFo17+ezfQ4ENilvRVo5eEooU7T5M8chHz9QUnqCAKCgCBgMgJCviYDGmxzcJGDK96xY8foxRdfpP\/7v\/8Ltkl5XhDwiQC8H9544w2+3FS1alU2DYYrsqDPwUVoBSFfiwk2OTmZypUrxxsACUGRUslXYHmLTUGGY0MEvv\/+e8rKyqJ773WmB4QKkQn5qkDdS5\/IUYdg73FxcZyvTnttsWHKcCIMgS1btrDWi8wvUsKDgJBveHD2q5erV69yuvsWLVqwz\/OMGTPo0Ucf5Tx0UgSBUCKwYsUKWr58Ocdcad68+Q3ZXkLZr5PbFvINk\/RxA2\/dunXUr18\/7hHxiOfPn08LFy7kNEpI\/AmtA6aGJ554glMpgYi112EapnQTYQj4u+6OHj1KMTExnEEcSWw7duzIv76khA4BId\/QYcstI4IbQmguW7aM88oNHjyYP0fGZvzDtWrEtkDSUHy3detWtvlC20X25c6dOxMuo0gRBIwgYHTdpaenc\/ZwrDUtznapUqWMdCl1DSIg5GsQMKPVkSZpz549tHv3biZZjXwRqa1u3boc1Q0\/9caPH08PP\/wwZ16GtwMiv4m3g1G0pb6GgNF1h2wyiLGSmZlJHTp0YNODlmRWUA0NAkK+ocH1plaRfQOaLsgXEdmQv659+\/Z01113cd3FixfzYn\/mmWfCNCLpxgkIyLqzrpSFfMMkG9dNMHbsWOrUqRNrukK+YRKCA7uRdWddoQv5hkk2+k2gNzNoZgeYIRo2bCiuPmGSh1O6kXVnXUkL+YZJNvpNgC5hgti2bRshezNswSBfmCTkkCNMAnFIN7LurCtoId8wycZ1E2iuZkh3j8MOeD2Ig3uYhOGgbmTdWVfYQr7WlY2MTBAQBCIYASHfCBauTE0QEASsi4CQr3VlIyMTBASBCEZAyDeChStTEwQEAesiIORrXdnIyAQBQSCCERDyjWDhytQEAUHAuggI+VpXNjIyQUAQiGAEhHwjWLgyNUFAELAuAkK+1pWNjEwQEAQiGAEh3wgWrkxNEBAErIuAkK91ZRNxI0P8ijVr1lChQoVy5oY0ST179rT1XJF88sCBA5zyaciQIdS2bVt64IEHcuaEeVeoUMFtuNAjR47Q+vXrqV27dhQVFWVrHGTwxhAQ8jWGl9QOAgGQUL169W4gJn1ziPaGf0jkaJeC2MwzZ87ktDvIMm2UfDFPxHJGtmrJHGwXqZszTiFfc3CUVvxAwBP5IrgQsn0glc0LL7xAhQsX5rRKJ06cYI0RhFamTBlOsTRq1Ci65ZZbOAgRco4hIP3o0aP5cyQd1QLJDBo0iFauXEnvv\/8+58tDFmikZNq5cyenZ0Ibu3btolq1anFQo+joaFqwYAET4R9\/\/EG9e\/fmPHpoD21BK8X3KCBaraSlpdG3335L3bp14yD53sj37NmztHr1an4Ukexq165N48aNo0OHDtEnn3xCL7\/8MuXLl88PJKVKJCAg5BsJUrTJHEC+7777bs5oS5cuzQlDQV6pqamcSikrK4umTp3K5IfwmvhJj\/x3Xbp0oSlTpnDYzZIlSzIRItUSSM8d+TZu3JjDdvbt25fy5MnD9YsXL07FihXjdqZPn87vEdS+adOmdPny5ZxMI1euXOFUTjAfYHwgZ2i1yD6C\/vTZpJFh+s4776QGDRrkkC8yAbsWzE3LUoI\/KsOGDeNwoshkgpQ\/aPull16SpJU2WctmDFPI1wwUpQ2\/EPCm+aIBkBPI9rnnnqPTp0\/ntInEjn369OH4xyBfpFv68ccfWYv0pPlCawWx6kuPHj3Y7KGlc8J30LrLly9PmzZt4p\/9IFEUzQQCk8Ldd99Nt956K2vFIGJozVrRz8mX5ov5QasG0dasWZNatmzJc8Fz0PSR2URP7H6BKpVsi4CQr21FZ7+B+0O+27dvZ7MANE9k0r127RprpSDlDRs20NChQ5mw8H7dunVsAnCn+SJGMghY0zahzV69epXNDu7IF59pCU2BLLRRpFGHOQT9oC1o3NCS9cUI+bZu3ZqWLl1KOGSDRq4dsAn52m8tmzFiIV8zUJQ2\/ELAH\/I9efIk\/yTHT3Dkt0tJSWGiffzxx2ny5Mk0cuTIG8wOvXr1yvnJDjPFnDlz2J4KIk1OTmZiBonCPFCnTh0mPHfke+7cOc4w7ZpZBOaGpKQkunjxIv\/vmmkE\/cEurTc7ePJ2gN0aZowJEyYwmWsFbcMsAUKOi4vzC0upZH8EhHztL0PbzMAf8sVkoJ3CZHDp0iX+GT58+HC2z8IlC8SFgzbtwA2E9emnn9KsWbP4wK1SpUps1wWJLlmyhObNm8f4tGnThrVkmC7ckS9MGzAxQMuFZj1w4EDOqYfXGqH\/+9\/\/vimdOg7cvvnmG3aXQ9p1TwdusG+jb9i3MReUsmXL8h+O8+fP08KFC\/kPjt6kYRvBykADQkDINyDY5CHVCOzfv59tviDfUBZ4SuCADgd47lzBYDLAASEOBAPNvwdbMrRnvW9wKOckbVsDASFfa8hBRmEQgXCQ75kzZ1gbhUcCSN6TVrpv3z4+AIRpBJqykQL7L\/6IQCuXSxZGkLN\/XSFf+8tQZiAICAI2REDI14ZCkyELAoKA\/REQ8rW\/DGUGgoAgYEMEhHxtKDQZsiAgCNgfASFf+8tQZiAICAI2REDI14ZCkyELAoKA\/REQ8rW\/DGUGgoAgYEMEhHxtKDQZsiAgCNgfASFf+8tQZiAICAI2REDI14ZCkyELAoKA\/RH4f4uSwzug53aBAAAAAElFTkSuQmCC","height":211,"width":351}}
%---
%[output:39caa129]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: GM355L6-250kW\nIM Normalization Voltage Factor: 326.6 V | IM Normalization Current Factor: 610.9 A\nRotor Resistance: 0.00434 Ohm\nMagnetization Inductance: 0.00533 H\n---------------------------\n","truncated":false}}
%---
%[output:1473b13d]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00756 H\nPer-System Quadrature Axis Inductance: 0.00624 H\n---------------------------\n","truncated":false}}
%---
%[output:0be9f8a6]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Unrecognized function or variable 'device_type'.\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('device_mosfet_setup', 'C:\\Git\\GitHub\\library\\library\\setup_files\\simulink_models_initialization\\device_mosfet_setup.m', 10)\" style=\"font-weight:bold\">device_mosfet_setup<\/a> (<a href=\"matlab: opentoline('C:\\Git\\GitHub\\library\\library\\setup_files\\simulink_models_initialization\\device_mosfet_setup.m',10,0)\">line 10<\/a>)\ndev = device_mosfet_setting(device_name, device_type, Vth, Rds_on, Vdon_diode, Rdon_diode, Eon, Eoff, Err, ...\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"}}
%---
