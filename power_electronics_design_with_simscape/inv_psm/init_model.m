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
delay_pwm = 0;
dead_time_afe = 0;
dead_time_inv = 0;
dead_time_dab = 2e-6;
dead_time_cllc = 2e-6;

glb_time = timing_setup(fpwm_afe, trgo_afe, fpwm_inv, trgo_inv, fpwm_dab, trgo_dab, ...
                fpwm_cllc, trgo_cllc, t_measure, tc_factor, delay_pwm, dead_time_afe, ...
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
use_mosfet_thermal_model = 0;
use_thermal_model = 1;

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
grid = grid_three_phase_emulator('wind_grid', grid_nominal_power, application_voltage, us1, us2, fgrid, ...
                eta, ucc, i1m, p_iron, up_xi_pu_ref, up_eta_pu_ref, un_xi_pu_ref, un_eta_pu_ref);
%%
%[text] ## Global Hardware Settings
afe_pwr_nom = 250e3;
inv_pwr_nom = 250e3;
dab_pwr_nom = 250e3;
cllc_pwr_nom = 250e3;
fres_dab = glb_time.fPWM_DAB/5;
fres_cllc = glb_time.fPWM_CLLC*1.25;

hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:2dd1a26c]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:42e81c8c]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:80659824]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:12e516b5]

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
parasitic_dclink_data; %[output:5e09d60e]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:04da844e]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:32738d8d]
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
afe_ctrl = ctrl_afe_setup(glb_time.ts_afe, grid.omega_grid_nom);

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
%[text] ### HeatSink settings
heatsink_liquid_2kW;
%[text] ### DEVICES settings (IGBT)
% infineon_FF650R17IE4D_B2;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.inv = device_igbt_setting_inv(glb_time.fPWM_INV, hwdata.inv.udc_nom);

% infineon_FF650R17IE4;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.afe = device_igbt_setting_afe(glb_time.fPWM_AFE, hwdata.afe.udc_nom);
%[text] ### DEVICES settings (MOSFET)
infineon_FF1000UXTR23T2M1;
mosfet.inv = device_mosfet_setting_inv(glb_time.fPWM_INV, hwdata.inv.udc_nom);

infineon_FF1000UXTR23T2M1;
mosfet.afe = device_mosfet_setting_afe(glb_time.fPWM_AFE, hwdata.afe.udc_nom);

wolfspeed_CAB760M12HM3
mosfet.dab = device_mosfet_setting_dab(glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
%[text] ### DEVICES settings (Ideal switch)
silicon_high_power_ideal_switch;
ideal_switch = device_ideal_switch_setting(glb_time.fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.afe = device_ideal_switch_setting(glb_time.fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.inv = device_ideal_switch_setting(glb_time.fPWM_INV, hwdata.inv.udc_nom);
%[text] ### Setting Global Faults
time_aux_power_supply_fault = 1e3;
%[text] ### Lithium Ion Battery
nominal_battery_voltage = grid.udc_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:843d2ac7]
lithium_ion_battery_2 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:1a80e500]
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
%   data: {"layout":"onright","rightPanelPercent":29.3}
%---
%[output:2dd1a26c]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:42e81c8c]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:80659824]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:12e516b5]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.880000e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:5e09d60e]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPUAAACUCAYAAABLGW+aAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQmUVNXRxwsYEVAMyr58LIJBBJFNQCGyxA8lCCqLMkpYE3ajCSpBUUkkiBuaqKDEhMVEFPyAgGJQElEhLIIQEZBNBxRQlCVCZJRBvvMr8iY9ba\/T3a9fv657zpyZ6X7vvnfr3f+runWr\/lXi1KlTp8SaScAk4BsJlDBQ++ZZ2kBMAioBA7VNBJOAzyRgoPbZA7XhmAQM1DYHTAI+k4CB2mcP1IZjEjBQ2xwwCfhMAgZqnz1QG45JwEBtc8Ak4DMJGKh99kBtOCYBA7XNAZOAzyRgoPbZA7XhmAQM1DYHTAI+k4CBOgMf6I4dO2TUqFFStmxZKVmypBw6dEiaNm0q9957r1SuXDmmEb344otSp04dadu2bdTjV69eLcOGDZO6desWXq9v374ycOBA\/X\/q1Kly3XXXSb169aL2ZQekXgIG6tTLOOlXANSzZs2Su+++W4H97bffyu9+9zsF6fXXXx\/T9eIF9Zw5c2Ty5Ml6va+\/\/lr++Mc\/yjfffKMvl5ycnJiuGe4gEgX54QVhLXEJGKgTl6HrPQSDGpA9+OCD0qFDB2nXrp3Mnj1bXnjhBb0vNGr\/\/v31b14Ezz\/\/vGrz+vXrS\/fu3aVNmzayZMkSefbZZ6WgoEB+9KMfqQYGvE5DUweCms\/3798v9913n9x111167oABA6R69er6cnn77bcV8DfffLNeOz8\/Xx577DFZvny5WhT0PWjQIHn33Xdly5YtkpeXJ7fddptef9KkSfrS4B7pv0KFCnqNJk2ayKuvvirnn3++dOnSRWbOnCn\/+te\/5De\/+Y2Owdp\/JWCgzsDZAKgBxb59+wrvvlu3bvLAAw\/I+vXr5c0331Qg0H77299K48aNFUh8PnbsWDlx4oSMGzdOQXfmmWfq56NHj5ZSpUrpC6FSpUpCf5FA\/e9\/\/1sBlZubK2h9QH3w4EEF6I033ihHjx5VzX7LLbfI66+\/rte54YYb5LPPPpN77rlH7rzzTgX1ihUr9L5LlCghTz\/9tNx00036cnjttdfkk08+UbN+xIgRMnLkSLniiiv02HLlysnPfvYz2bBhg\/z1r3\/VMSVqLWTgNAh7ywbqDHyaocxvNCmAKlOmjGqu9u3b68jQsoCWhhZ3PkeTs0ZGU06cOLGIFFg\/A5R4QY3237Vrl\/7885\/\/lLfeekseeughmTt3rr5Avv\/976uZ\/eSTT8rVV1+toKbxEqDxIti8ebNs375dtX2DBg3kpz\/9qfzqV79SrX3eeeepReKMI1gOGfgoU3LLBuqUiDW1nYaazM5naOQf\/vCHhQ4wNOGaNWvUtA0F6nXr1qmp6wALs\/nkyZMxmd+YyoAfDYum3rRpk\/6gvc8991y1EujX0eQXXHCB3gef9+jRowioP\/\/8c\/n1r3+t5n+rVq1k27ZtsnLlypCgZpmBg89AHXqeeQ7Ux48fl1WrVunbHVONVrVqVTXdLrvssiKTLbXQ8W7vwZMZ7Ye8du7cKS1atFDte+utt+oAAB4gQK5r165Vs5y\/x4wZI4MHD1aQLViwQLXhGWecoVo0UNM72j7YUcZxmMGsvzHDAfX8+fMLXxzcA448NPXSpUv1xcEz3Lt3r94DJnigpmZMM2bM0HMwpR9\/\/HG9\/1Ca2kAdeW56BtRMzI0bN+pbv1OnTvomPuecc\/TumYRMgL\/85S9qxvF9NrfgLS0cURdeeKGCBQ0ZylGG9v3973+vAMZMZt3atWtX1Yq8EHA80RzHWuAaNdKWFuc4oMbBxVqZ9TFrYdb83\/ve9\/SZYeJv3bpVTfCzzz5bhgwZUgTUnPvEE0\/I4sWLpWHDhmqec19YAtxboPltoM4QUB87dkzXYhdffHHYrQ22bjDLGjVqlM2YzrixL1u2TGrXrq2A\/uKLL+TRRx9VSwGHnLXkS8AzmpqhAdrdu3frNgam3d\/\/\/nf54IMPVAt17txZPajWMk8CH374oZr3rJt5hlgUl156aeYNJEPu2DOgZm331FNPyaJFi3RrBSADbPYkWWNXrFhRo5ow7ayZBEwC4SXgGVAfPnxYHnnkEbn99ts1WIE3+\/33368OFkzzKVOm6J4na0ZrJgGTQAaAmvhlHDkEFeAYC9yb5H8ilfCEsldpzSRgEjBQ2xwwCWSNBDxjfqOpCV1ku4X1NXHCHTt21H1pwhrxjBMiaJo6a+amDbSYEvAMqNmnfO+99zSYP1TDa0oygHnAi\/mk7bSskYBnQM12Flk34er14fUmkMHS87JmbtpAiykBz4Aa7zdJ\/mTmfPnllxqkQMIBjeyf3r17y09+8hONRrJmEjAJZICjzLlFzO9p06ZJnz59pGbNmvoxQH\/ppZc0Bc\/Mb5vOJoHIEvCMpnZuE4cZ8d8kJJx11ln6Mbm7ZPYMHz7cHGU2o00CUSTgOVCT+kdmEXxXpOHRnHDRX\/7yl6apbUqbBDIN1I5mJon\/5ZdfVm84oaKkCTpZW\/ZUTQImgQxYUwNemC+iZe7gQIt2jD1wk0A2S8Az5jcBJyRzkFNNMj1BKA75HWGiJN2TT92sWTPp2bNnNj8zG7tJIKIEPANq5y5Jz4P+hmT+jz76SD8mY4uke7a1LKLMZrRJIMO83\/bATAImgcQk4DlNndhw7GyTgEnAQG1zwCTgMwkYqH32QG04JgFPghpOq\/HjxytZAgEneMUpyxJr8bfAx0qCCPvdRKQRYgrRApzTlhhikz9VEij4PE+7PvrGLDn3hvtSdZmw\/XoO1E5dqCuvvFK94PBDU7WB\/OriRJRBSwujCkwqZHpNmDBBfvzjH8sll1yiQqE2kzWTQKISqFa6QC4pny9Nyx+Xqyoe0+4+\/SZH+m36n+90jdJKZfMcqB1ao379+ikRPHzP7FcXl86IFwPaGj5rWmDhOAfUyRIyW3BWzjWV09U7faONCw7kyfHNb8rxLcslf\/NyyalcV8o27ihlGneQMyrXlTKNO4a8YRRJsuZcqAt4DtRoaqo\/oKkplwqoP\/30UyV0JzUz3tRLai9BRwvFMI0yNJRzcWpFVRo+T6pVq5aU2VJQcFJyckolpa9M72TflwUJDaHGOTnS\/cKzZWibCgn1k9STD38ismuNnPpwtci6\/zvd9bm1ROq3kRIte4nUj17rm1Moi5RVoGbQDJjyK++\/\/74Gm7AWBuiQwcfbAguqhQJ1Mt+apqnjfTrhj39waZ7MeWe\/7DmUL+3rV5BFo5onr\/MYe0IbH9+MFn5TTnyeV0QbVx41I8ZevntYMudcRmhq5yYxmYkFp1xMrIwnVFokw4v4cHKxWT9THI6ibdSR4uVABZDWrVsXhpomU8AG6mLP87Anrth5REa\/sFXBPfaqejL2qtPEGalqDpCPLp+lIKZhVpfvOFDKNu4Q1qSO536SOec8DepA5pNQN1qrVi2tihiO95sXANp91KhRWouJ8q3UOKbYGwXPKfCGo4yKjNAQE3ZKS6aADdTxTO34jkVzP7j0I6l9Xhl5sm8jad8geWb50eUzVRujlQF14NoYMCe7JXPOeRrUDkcZmpktLMxuSq\/S8HxTd\/maa64JW6Fjz549Mm\/ePC1OjmON6ooUA2A9TTVISA3R\/s2bN5cmTZoouA3UyZ6uqe0PbT3nnU8V3ImY5ACX7aZABxd3DoDd2ILKGlA70wGNPX36dBk9enQR5hPW1EOHDo2pQgcZX3\/+85+VVpiXARU0wznKkilg09SpBXVg7z2e2iArdh2RQ1Niq4DqaGN+OyY1nupE1sbFHW0y55ynNbVzc5TYwcyGZNBxjMFRxpYWASkOUQJbX5RDZR1NjS3Hm43GpmwqgSqU8OEFEVhsPdj7jYD\/9re\/Fff5FDmP+2SZYM0dCUxfc0SeWXtEQnrKd62WUx+uEeFn1+rTXurzaomc30ZK\/O\/p2t3palnp\/X7nnXd0K4si6DQ4yqiBzPo4XHMix9DQAJzgEqLGgvel+R+H2fXXX2\/md7pmdRKv65jkz72ySq47a5uMPPPNIg6udGnjSEPMOk3tCAMTGi82a22YTkqXLh1xKrCXjYbHWYa2nzx5stZB5nNeCBRAB8ys1TmmcePGBuokgsvtrgK3mxyTel9ONVl\/5iWSn\/t0yr3kiYw360AdzgsezfuNdmfNzVYWAKZUz+WXXy6DBg1ScAN0tDm1rwk3hUGFlkwB25o6kake\/dxQ6+KcKnWlxoQ3Ck92ewss+l1\/94hkzrlQ1\/dcRFlwpQ6I\/WETzcnJkZtvvllrV4draHfohevUqSPLli1TE\/7111+3MNHizLw0nuOA1wn4cG7F2WqKxbkVuAW2cfxlaRyNgTqk8B1vNuvgSIyiOMG2bdsm1157rW5nAWqSOSKFiS64uqxc8h+tneiTJ8Q1q4sNHPokdhESchmuOU6tc2tKifPbirTqFXu\/AUcSqvry1mOFzrTFA7zhxMxKR1nwE2TPedasWbrNFc77PWDAAPnDH\/6ggSWY3k59a0Adyfs9rXN56d2rd7EmTfBJR48dlfJnl09KX5naCdo0UnNjHzj4+jjTRs\/Zqltg7G8\/mdtIg1jS1bLO\/A7cqnKEDvEgkWKQ+xMVFqqtXr1abrrppiJfValSRU32\/fv3W5houmawh67rlfV21oGaNfVXX32lgScOgHFwsa1Vrly5sOQGmOikarZq1UpatmypWp1wUf6eMmWKhYl6CFzpvpV0r7ezCtQwnbAFNXv2bA0+cXi\/KXGLGU0wSTiKYLQxXm7SMwE4yRxjxoyRVatWaYCKhYmmG0reu34q48kjjTarQM22FN7q9evXa6CJE3yCgNhXJtQTL3iotmPHDk3PJKOLCLEWLVro1tWf\/vSniI6yZArYtrS8B9xod8R6m1hyYsoTiSePdp3A75M550JdN+1bWuwfUxSPRoAJQM7Ly9MKHeEAzLHBYaIAHk1OGdyLLrpIs7Qof1u1alXp2LGjtG\/fXq9hYaLxTL\/sORZP+YRlX8j6vfkyrHWFlJIz+NL7zboZ9hE0K1FjeLQxmQ8cOKDBI4A7Pz+\/iFMsWvAJmhoPOdFimO2AHg9406ZNdX0eSGdkYaLZA9Z4R+o40zgv99LqKYlM852mZt3M1hMghbIomJ6I75cuXSobNmxQUMba0PhsZxFNRsw3a2+0NFtdsJ9YmGiskrTjkEDgehtyhtxLk0N5Rd++AzVamp9opvWSJUuEulpOY61M4bxwHGVEnt1xxx2CU42\/0cYkdXAOQSgWJmpgjVcCgfnbySRn8B2oHcGGi\/FmTY227tq1q5rOTgOk\/B8uYgsg33XXXXLLLbdoyubcuXMV4LwMjE003ulsxwdKINnBK74FNQwnpEkCUpxcAG\/hwoW6Rw1XNwyf8ZjfPATivQE14ZpoadbssXi\/bQqbBGKRQEGlhvJVi8HybblKUmHhkFhOCXuML9lEwzGcQIYAWwmEA+w7h2vB3m8ix0i9ZJ8aZxsUSHjA0fwdOnQI6\/1O6MnYyVkpAdbbqSZATESwadvSYo378MMPaxkchy2EGG8+Yy1N2GfdunULY72jeb85nnU4oGa9Th+AnBTLSN7vRIRn55oEvCiBtIEaYQQynOCxhhGUSDCiyjDH+\/TpE7PM2NuGGIHzMd3R0vz07NlTzXy2umgQJvTv318aNWoUc992oEkgkySQVlAD3N27dwuRWORAs77GsQXlEAAPbJjRZFvB2x3Kc05fb731ljz++OO6psYZMW7cOKlRo4YWyCPPmmOiJYZk0sOzezUJhJJAWkE9f\/58Wbx4sRL233nnnQpI1sY4yjDFu3Tpot8BStbJgJL1Nska1kwCJoHQEkgbqNmCgmSfCpSUmWUvmXBRMqrYi\/7FL35RuCfNsXiynZxp1s3WTAImAY+B2vF+o5lxjgFqwkNZ89LQ3E41Do595JFHZPjw4TJjxgx1hlkzCZgEPAZqTGmqWhJoQlrkiBEjNLsK85rwToJHIOHHgbZx40atzgE\/GeGeZn7bdDYJhJdA2sxvbgmHFvWuWFuTwNGtWzfp1auXJmSwJUWSBiGlrKehCeZvtqcihZjawzYJZLsEXAd1MFto8AOA7YTAEvacydxywE89LMz0cAXysv1B2vhNAo4EXAd1YMw3DjBSLwkyoR08eFCrUaKl2b5at26dkh18\/PHHGkTygx\/8wJ6cSSDpEjg8N3w2IEUDUtFioTku7nVdB7Vzo5jeEBoQYIJ5TSM09LnnnlMzG+83LCjsN\/M9dEZUtAyXpcX5L774ou5NO+2BBx7QLC22xPCwsw9OBFuPHj0Kuc7o35o\/JNC\/xhGpWvqEVCtdIFXPLNDfsbRPvwnNpsO5n30d\/rtY+g51TJu2bYsUIChuP+HOSxuoMbEJCLn11luLVLfEy01SB4kZRInxNw4z4sA5NhxHGQOkP1glLrjggsLxsufNC4HkEEx7Is7YRiPhg5bMjBmjM0r29AzdXySyfyp2nPEfmuIyjTtoeVqvtWTOuVBjSxuo2ZOeNGmS1KtXT6l\/aVTi+OCDD5Q8AdohtrtIp6QkLcwleMjDOcno74knnlCa4OrVqxeOFe1tqZdem9bx3U+4cjuAN5VmbHx3GfvRvgU1IoD2l5BQzGPMcSLIBg8eXKQKB84ytHX58uXDcn47faGNMeHxmtMX2h6Ko0gVOqyUbeyT0bUjqd6xa42cWv9\/nitDmwwZ+I6jDPAS180WVWALJk1A8wZWuoyWpUXWF2twBMYLAEqkt99+W7nC6Yf\/WVNfccUVqrlhGjXzOxlTNHl94LA6vmW5lqKNp25W8u7AnZ58p6nRvIsWLdKAEpxYsIayLw2oCSqBrYTKlJ06dSqsIY2oWQ+T7EEwSizNSb3EFMeLjsamD0JNATbrcwN1LJJM3TGBIOYqfgZyoBR9B2pncICO9e6CBQs0S4tGeR3ofIkcQ7NCX9S7d2\/d1opWnxqHGI4ycqhhJ4UkAU2Nt3zNmjVKdkgbMmSIOt5+\/vOfG6hTh9ciPSejiqVLt+rKZXwL6mjSw0QmJBStTiYXQAxM8gg+n+MJWCFJhBcAHvDx48cXghmAcwzmOQX0zPwWcYq1I8uCA7ujPRIJ3rOl1KzTCg6c\/jvUvi4a2PFKZ6JjK6pg4jwga0HNXjXplzjRYgF1OLlCDxyp6uWHfUIX3IvzOWX04ftyYqO\/3V+q6HH7cqoWjnt\/TjVZ32iY\/l\/jnBypUT4npYT4mSxw3znKIj2MQCBjlsdjftNvqOAT+ty0aZNVvUwhCuDsou05dFx\/f3woX\/YczhdYOAMbNLsQ5NO8zPGVQlFp11mjqXGU4bwiVJTieK1bty4skIcgYnGUhQo+eeWVVzRtc86cOdpHbm6ukv4TjppsAVvwSXg4OMBfufOw1ommAfKN4y9LNYY817+vQQ2QSb8kcuzGG2\/UhA3WvTjKglu0La1wwSfsg1vVS8\/Na62AgWanMF0yifK9N9Lv3pFvQc3WFqGfbGlBEEjkGPvMJHlAhhBveiWBLBZ8kglTuug9Uphu+tojsnjrMene6GyZcGXR+IXMG1H0O\/btmtqJ\/YbZ02E+ITiExAtAHSnGG7EF835DKBgu+CQS73cy35pmfkef0OGOQGOPmrNVDk3pVPxOMuTMZM65UENOW+w3EWBTp07VfWiHowwWFP5mK4q95kgNTQ9vGcQKTgIHVQ+gAmZd3qpVKwU+v\/l87dq1GlEGGynrdaiDbU3tLRRQcbLH1A2u1YlO1+h9C2oEStIGJviRI0ekZcuWygMOR5lTSzqc0CHux\/EFDdL06dMV1BSqJ6DkoYceUtDef\/\/9snnzZs3IgszQHGXpmsLxXdcBNpUm\/eoh9zWoedxobBIwiAlv0KDBd2LCQ02Jbdu26R42PGZjxoxRUG\/fvl2jyQhYgecMQn9iwHlZEG3GCwAnXPPmzaVJkybqBTdNHR\/g3DraAfaikc2lfYMKbl3Wtev4GtRoZtbBaFPMZsxkosYc0ziSlDHV0erEcgNqtPerr74q99xzT5GyOxUrVtRYb4rwOdYBBe\/Hjh1bCGoID5PRyBBzSgglo79s7mPCsi\/UeTa9ZzVpWbOMr0ThW0cZGhpTeejQoZouifaFkYQCeZjRgWtqosKeeeYZJTYghhsnGple5Fuz50z+NaBmjY5JTyPohOPRzJAVWpZW5uGix1MbNIjlyb6NfKWxfaupcWLBSAIoKTeLQ4u60rF4vwEw57AVRqw3oMb8pug8a2ki0fifGlqY4P\/4xz8sSyvzMF14x80mrvIVsH0LaoJF0MAQIFAQDz4xzPF3331Xky3CFZfnSaPV2T7C\/KYIAKDmJdGvXz\/d7ybWm\/U263Q+X7VqlWVpZTCo0dhEofnFeeZbUDPHCBghLhvy\/sqVK8sbb7yhtaSjbWdxLmCF4ohKHoCavm6\/\/XY1wzHN0frQG82bN0+ns2VpZTCqRTTyjH1sJ348kz3jvgY1e80AEK80DecXYZ1QGgXye4daUwc7yiIxn0QLPjFHWWYAnuizl7cek2fWno4dJxts8YBamXHzAXfpW0cZgCYBg4QLQkNhO8EUHzRokJIHRgsTDQZ18JONteh8Mt+aFlHmHr7Y9lq564g8uPQ0wQYNLU5rV7+C1D6vrGf3uZM550JJPG0RZSRzYB6zhQWLKEXgqUHNFhfAjhfU4ZhPohWdN95v94CY6ivlX3itXqKgUsMiv6Ndt+RXX0Q8pORXB6N1Eff3B2YPj\/ucWE\/wBKjxYrOfzBYUW1oQ7keL\/Q7W1OGYTyA4tKLzsU6H7DzOSQsNN3onTzyZ0nkqt1EyuyvSV9pADQjZe8bBhaPr2WeflYsuukh5v\/FqR6rEkTJpWMcmAR9IIG2gRnYEiLCOhsaX2G1M6K5du6on3C+NtT175\/v379egmssvv9wvQ7NxeFQCaQU1zjI0MwkdTmN\/muCRSPvUHpVlyNuClql27drSsGFDmTFjhvoLzArJpCeYefeaNlATGEICBp7vQGcV3N7wgftl4hPWSlkh+McD\/868qWJ3nCkSSBuoQxXIyxShxXqfJ0+eVLqmXr16qeOP\/O+rrrqqSAG\/WPuy40wCsUogbaAOVco21pv2wnGs\/5ctW6b1umgsJagC8vzzz6uPgGwxanhhcnfv3l39BADc+dsLY7B78KcEXAc1kV\/EZVNeh4LzVNKgxA5bWjSvm9\/cN\/c\/f\/58rSbipHCyLedwrTEuPPh8Rzw7a2rSQ0kwGThwYBGWVH9OKxtVOiXgOqjxeAMMJzQ0ePCxUAGnU2D4ALZs2aKsKoDXAXVg0QDGRoIKedzEoOP9JmnFvN\/pfHLZc23XQY1omeDkPpPMAXVRcKx3JoifxBE0M6AmEAanH\/ndsKrQiGHnBQX1sTWTgJsScB3UrKXRXORPwyVGtBcaj2yraKGhbgom2rWCQT1x4kRlYUEzG6ijSc++T6UEXAd1sNeb\/2FAIQ86lpTLVAojnr4DQR1obmN58D\/mONk4OMusmQTclEBaQO0wnlCXGlBDwg\/ZQbR4bzcFE+1agaDmWExx6mBTOgjLA1BjmkOAaM0k4KYEDNTFlHYwqJ0tLYr0wYmG5WFaupjCtdMSkkBaQE3hd+pbhWqB5IIJjcxONglkqQRcB3WWytmGbRJwTQIGatdEbRcyCbgjAQO1O3K2q5gEXJOAgdo1UduFTALuSMBA7Y6c7SomAdckYKB2TdR2IZOAOxIwULsjZ7uKScA1CRioXRO1Xcgk4I4EDNTuyNmuYhJwTQIGatdEnfiFiCd\/7bXXiiS+QI80fHjqiOETv+voPbz\/\/vuyc+dOpXqiOGJubq60bdu28ETGXbdu3ZBprHv37hXKJsVS1SX6nfjjCAN1Bj1HJjd1wQInfODtkx3GT8mSJTNmVOSik1vfv39\/JZuMF9QMlNx12FqpRW5NxECdQbMgHKhJIoGNJS8vT2677TalhIJO6cCBA6rhAErNmjWVWolsOOiXSTahdjfEDoFZck6iCvntS5Ys0SILJKvAiAoVEzH70DLRx3vvvad58SSvlClTRmbPnq0AI2d+5MiRystGf06uPN\/TALDT4HunfDFVWQB4JFBTqmnp0qV6Kplw5ONPmjRJ9uzZo9VNqXpaunTpDHqiqblVA3Vq5JqSXp3qn07nNWrUUGJDQLFixQqlUDpx4oQ89thjCirSPjFt4VODXebRRx\/VdFBKBwMwGGgAUyhQd+nSRdNJR48eLaVKldLjKWEElxz9UAeN\/yGH6NatmxZlcJhgqD0OEQZmNPcH6NHCsMNwPfjanAbDKpVZOnfuXAjqxYsXf0d+jM1hkeFldffdd2uaK0wzUEzR95gxY5SKOdubgTqDZkAkTc0wmPSAmIIBBw\/+t6gbmW+jRo3SfG9ADc3S9u3bVeuF09RoWQAb2IYNG6bmvwNevsNKqFOnjqxcuVLNX8BJc5YCmNYXX3yxVKlSRbU4AA8s1BA4pmiamvE5fPHNmjWT3r1761iiVUDNoEeclFs1UCdFjO50EguoN2zYoOYxmhISCqe0EWCnuui4ceMUCPwPxTGmcChNTU44wHa0I9oXHnPM71Cg5rN27dop5xwN7UkVU5YFXIe+sBDQ6oEtHlBT5OGll14SnGNYEA79lYG66PwzULuDx6RcJRZQU7sL0xRTFL40KJgB8DXXXCMPP\/yw3HvvvUXM7xEjRhSarpjrcJOzXgWglAwC8IATM7lNmzYKpFCgpnQSDKvBzC+Y3RMmTBCoofkdzATD9Vj3B5rf4bzf+AUw5ydPnlyk3hp9Y54DdDO\/zVGWFLC51UksoOZe0KaYzlQUZf06fvx4Xf+y9QMgcJA5jjKAsHDhQpk2bZrSSVECiXUz4ITffObMmTq8vn37qlbHhA8Fakx8TG20MpYAdcfhaONv50Vxxx136P+BDUfZ2rVrdVsuPz8\/rKMM\/wHXxn\/AWGi1atXSFxKU0xRR4EXmlxpsicwp09SJSC+Dz92xY4euqQF1KhuecxxrON5CbTlhOuPYw5FXXD431upo+3Bbfakcnxf7NlB78amv5NHBAAAAoklEQVS4cE9ugBpSSbQnHmpeHuG06LZt29RxxxIhWJNHEwXra15OWBGZRDEdbVyJfG+gTkR6dq5JwIMSMFB78KHYLZkEEpGAgToR6dm5JgEPSsBA7cGHYrdkEkhEAgbqRKRn55oEPCgBA7UHH4rdkkkgEQkYqBORnp1rEvCgBAzUHnwodksmgUQkYKBORHp2rknAgxIwUHvwodgtmQQSkcD\/A\/2bEIAJr3v7AAAAAElFTkSuQmCC","height":148,"width":245}}
%---
%[output:04da844e]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: GM355L6-250kW\nIM Normalization Voltage Factor: 326.6 V | IM Normalization Current Factor: 610.9 A\nRotor Resistance: 0.00434 Ohm\nMagnetization Inductance: 0.00533 H\n---------------------------\n","truncated":false}}
%---
%[output:32738d8d]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00756 H\nPer-System Quadrature Axis Inductance: 0.00624 H\n---------------------------\n","truncated":false}}
%---
%[output:843d2ac7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPUAAACUCAYAAABLGW+aAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQu8lVP6x5+IktIVqVSUlElSLl3cQkxGERXJpOQWhT9KLjMYldJoSDXiHyNDuSu3f2GqKWpETUgqQlHu16JB\/D\/f1axtnfe8797v3vu9nDrP+nzO55yz93rXu9Zvrd96nvWs9Tyrwi+\/\/PKLaFIEFIFtBoEKSuptpi+1IYqAQUBJrQNBEdjGEFBSb2Mdqs1RBEqQ+osvvpABAwbI0qVLQyGzevXqUPk0kyKgCCSHQClS33DDDXLddddJrVq1staCCSBXnuSakf+bFi5cKGeccYY88MAD0q5du\/wLyPKEt+zRo0cLn02ePDkyzL7\/\/nsZNmyYqcWoUaPkgw8+kP79+8vgwYPltNNOi7Q93sLCYvfggw\/K7bffLvfcc4\/ss88+WeuUT958GmdxevLJJ+X888+XK6+8MtTjXnx32mmnUM9FnWnVqlUydOhQufnmm3NiaN9dgtT\/+c9\/5K233pLmzZtLpUqVstaPvLnyRN3ArbW8bYnUVps7\/fTTc04e+RA1n7z5jANIwWTXtWvX0ISm\/LJCauoCNtOmTQstFEpJ6pEjR0qHDh2kU6dOUrNmzXzwK5XXzuj2i5tuuskMBDswGjZsKGvWrDHq\/gEHHFCi0u6z7nf288MPP1zmzZtnis42AwPIVVddlambzetKm2bNmpllh5XY69atKyH9+J9k67Fy5Uoj5W17vJLL\/f\/999\/PvL9evXqlpJb3WTsIrcQNwtAddMzkgwYNyiybbBuZTCZNmlSq7Xxgv6NOu+66q8ljNQlXuvG5bSd\/U58rrrgi0w7vks2LkX05GlHt2rUNwSyeFo\/PP\/\/c4OnmpS\/cvoOUaCR+EtNiZsu17ffWzW+ceJ+19ec9aEKMz08\/\/dTU2R2HXoxcnOwEdfXVVwt8+vOf\/2zGVjbMvXV1NUjvmMhFylKSeu7cufLII4\/IggULpGXLlqYTIHnVqlVzlVXiezsY7YCwHcT\/nTt3NiQCLFQzkjubus9269YtAy6DzhLKHbioVn4qXlAdAIxk1W9LamtLYABZojDxuOot3x155JGhSW07M0j99ko+V2LZwe6HocWFdvip39mWABZDcLBtt6R2B7OLt62Dq3XYvLYODHQ7OaLmum1p0KBB1uWCV1Lb8eLW0faFS2yvJLbt9hKbfvCq3q5w8fbxxRdfbOr76quvlhijdrLN1rYZM2aUECRB481izm9ws+2zz1ti23r6tcGPlIHW7w0bNsiSJUtk+vTp8txzz0nHjh2ld+\/ectBBB\/nOlt7CvSqnBb9t27alCEMnufnvuusucYnqdrBLRjubB63bsql0fpLalVbZZkfvZJFNUucitStxsWVg07AkGTduXIm1uIvhH\/\/4R\/nTn\/6UyRu0pnYlgJU04OtOMi72TCTuBOsOfO9EZ0nsp2lRsSD8XQnsTli2H+0EgJS02gN19Ju8ve+w9fUSxo8Q2frYq357Jyw73l1Jb7UJS0pXw\/Hywf3fq\/l5Jyrv5JNrfR9qS+uHH34wqh2DAemay+BjAXE7xQV7zJgxMmTIkMzM5EdqV2101bI4Se1KgqRI7Q5+1DQ6m\/WqV0PBKOliOH78eGM8CZLUXrWSfJDaPuf2jbtms6S2qqzF3k97od+8yxvyWwnjJZx3OeCnsqJxWVJDYjf5LV+8ZLdjz0pYVH5Xe3DLK4bU3mUR5XpJbXHw44OLuSW1VzCG0TbyktRk\/vnnn+XDDz+UWbNmGZWc1KdPH+nZs2dOI1k+kppy7fqFCcMrSdyKe6ViNmnsSnhmavdZd3KwKqgfqa2BxVWBTjnllBKW5mIkNfVwZ2ZXGmXDMJek9g52ryaUS1L7WdGzSQx3ErEaARqelb6009UAvITKps5mk0xRSGq\/Prbqt500XUltv7MTo3cZ4lWfKSOMpA7aibFY2bW5H5Hdz0pJaohMIc8++6xRd0g9evQwkoOZskKFCrnKNN9Hsaa2jfRTVYKkgd9MbDstqBw\/UntnV3cdag0+brloFrZO+W5puUYX1yCUDcNca2p3sFtpBTZMmpAN42G2NXWQBuBiSB5XCnpVVga3JbVXYro2Foynfmtqrzpu6xTVmjpbHzMxuVuGXvXbb8nC0pK1uR+p\/WwEtj+8OPqNH2u8zaV6U1YJUn\/55ZcycOBAeeedd4w0Pumkk8ze2HbbbReKyN5Muazf5PezLLqTAn+7alc+kppnXfUwqBw\/UrsS1Kqido3ktXxaS3wQqW2d\/dRHi5l3kNvPw1i\/GUhW27H7seeee27mIBEDFC2E79yJkokojPXbtRp7rd\/e+rkWYld68153JwAj5Zw5c8QSwdoEwNqdsO3OhXd3xG\/y9lq\/yZPLyORdpnj72E9SY3Bzn8NwascLy4fFixdnJk33DEQ+1m9bj0j2qdevX2\/WNBUrViyIyGEeynfhH6ZMzVM4Avnso9sJjR2RuA+5FN6isv9kPpjn25pSkhpDCnueufaokeq58gRVRkmdbzdFm9\/VCrxqfBj1zqstRVu7bbO0YjHPB5VSh08uvfRSc6IsV+cyY7O5rkkRUATKFgKlDp+89tprwhHQMOmwww4Lk03zKAKKQIIIhNqnTrA+svfeeyf5On2XIpAoAj807CjftTlbvhjbKbb3lklSq0tnbP1dqmAmUcU7ObynLvpILpq6XEmdHOTl703vvvuu7LXXXuWv4Sm1WEmdEvDl6bVK6mR7O3VSczDk5Zdflo8\/\/li6dOli9q6tm15cUKg6GBey\/uUqqZPFe\/TM92T0zHfTUb\/nz58v119\/vRxyyCEmqoaNvtC3b1859thjY0NCSR0btL4FK6mTxTs1UuN2CaE5aohktiGO8ODhaCHf5etfHRY6JXVYpKLJp6SOBsewpaRGak584duLJ9D222+fITUVDxvDLFsjf\/rpJxk7dqw0atSo1FFDJXXY4RFNPiV1NDiGLSW1NTWkw1e3fv36RtXm72uvvVZeeOEFE8OMk2Q77rhj2HaUyvf000\/LX\/7yF6MJeM8PK6kLhrWgB5XUBcFW8ENsZ0HsVPapv\/nmG+M2N2XKFPnxxx+Nkez444830rsYYxkujJyDxWmkSpUqvqRm8tCUDALYS+gLTckgcOh1M+WnOvumQ2rbRKKesMbmLHiu8+C5YKEcQvTgdvevf\/3LZFdJnQu1eL9XSR0vvt7Sa10223yUuKQmUMLXX38tQXfnESihevXqeflZU9ZDDz1kpPzRRx9tpLWSOtkB5fc2JXVyfbDmi03SeviCdEiNWyWO4ITgbdy4sVSuXFlQx9euXStNmjTJoEB0lLDJL6Qqz3rDtuqaOiyi0eRTUkeDY5hSuk1cIvPf\/iodUmMou+WWW0ysYhvVAUlLpIply5bJBRdcIC+++GLmuzAN8uZRSV0IatE\/o6SOHtOgEq3qXfGzFfLJlAtie7GvQwdbWn5bV+7n1KiYa3eU1LH1aV4FK6nzgqugzK7a3bBWZfnm7j6xOtH4khp\/amJK4y+NxZsYZVZSP\/bYY+bGhjfeeCPyO6hATNXvgsZNwQ8pqQuGLtSDjy35RM65b1km74wLD5S+x7VJntTU4KOPPpLhw4dn1tVIaazfhDvCUEakxUcffTRUw\/LJpKTOB63i8yqpi8fQrwRXOvM9Enr86S3ksKY1YhdcOf2p2YZiW6sQi3chcCmpC0Gt8GeU1IVj530SIk+cu1bunPdBia8gNBKa30loo1lJjcWaH5vY5uIwCsHMCw06mAtCJXUuhKL9XkldHJ4Qeezz78uUhVsuUXSTl8z2u7jHeCCpWTsTc5nTZDZVq1bNHByB1MUeRAmCMu4GF9eF297TSur8+hQSv\/ze1zJlwTqZ\/86W7amwZE6V1OxJc9abyKJsb7F9ddZZZ8nDDz9sznx37949PyTyyK2kzgOsCLIqqYNBhMCkQVOXBxLYPo1Unj7wQGlUe4uKnS3FPcZzbmlt3rzZXOV5ySWXmAMo7F\/j0LHLLrvkqntB38fd4IIqtQ0\/pKTe0rkQeNm6DfLXuWtzEpj8kPjwpjVlyHGNM2vlsMMk7jGedUuL62vbt29vPKrOO+882bhxo7lAG68tXVOH7cKyna88kRriQsZ7Xlonjy\/5OBR5LYH5\/T\/HNJJO+9bKm8TeEZAKqakEt13ee++95k5q\/r7oootk06ZNZo8a9TvsRXn5Dum4G5xvfbb1\/NsiqSEvxzGnLVova77cZKRw2ATpG9asLN0O2E2O26920QT2e2\/cYzznlpatFGtrDqDssMMOYfEpKF\/cDS6oUtvwQ1sjqSHpdz9ulvGz18qaz78PLXHdbrTkPahRdenXoV4s5A0aNnGP8ZxravcoKIEIOWnGEdJijoimaUTYhvlZUNPKGqmtVLXrWxoVZGXO1WC7L3xYk5pyU\/em8uV3PyVK3jJBantx3dKlSwPxwv+ZQAm6pZVrSG0d3ydJakvYnzb\/IrfPXiPvfPpd3uqxF1Urcds2qi79O9QzX1syl9UeSEVS43oZ9vbLqIGLu8FR13drLy9KUkPap177VP5v2WcGlkIlrMXUkpM1bsemNaX3wXW3CtLmGhNxj\/HQa+pcFY3q+7gbHFU9t5VycpHaSteXVn9ljE+sYfM1PgVhZaVso9o7GSlbp+qWuHdlXdIW2\/dxj\/FSV9kOGDBAsqnfBxxwgEyePFnX1MX2bMrPQ9ZfRGTqvJXy0gebTW2iJKshZ83KAmHPO7yBVN+pYrkgbJhuTZTUYSoUd564Gxx3\/dMs30rVdz\/7Xh7\/9yeyOoI1a9Aads9aleWoZrXk0L2qK1nz7PS4x3hW9Zv9aSJ7EsS\/WbNm5iBKXFZvi0vcDc4T\/zKRHbKu+GijPLvsM3n7k+KNS35EtZK1cZ2dpGOTGtJ+7xrbvBqcVufGPcYDSW2v3TniiCMER47PPvtM+GzEiBEmeEJcKe4Gx1XvsOXaU038nr3iC+McsPaLTZGpvm49XEPTfvWqyoCO9WWf3aqYwxj2u1xr6rDt0nzhEYh7jPuSGh\/qa665xgTbb9myZaa2RDv529\/+ptfuePrPqr1VK28v\/zv\/Q5m\/6stI16hBRG3VoJp0aVlH9qy5xYmgEAOTkjo8GaPKmQqpw8Qoi0sNj7vBYTvGEvXHn3+Wu+Z9KG+u22AeLXabxu\/9rkTdv341ObXNbsYSXAhJw7bP5lNS54tY8fnjHuO+kppIJyNHjpTmzZtLjx49zO0cxAJ\/6qmnZMGCBUZSV6pUKdM6jo8+88wzxvED10wqjS821\/a4yXu4xc+SHneDbX0g7S6VK8ofZrwt7xd41DCoe12SkueE\/etIl5a7SoUCpWnxwyi4BCV1nOj6lx33GA9cU3MklOOgNvb3unXrZJ999pHRo0fLnnvuWaK2xAPHc4tL9WrXri0zZ840oYQvv\/zyEvkYQMQKv\/DCCwORjKPBEPiu+R\/I0rXfFixpXaI2rL2T9Gizm7H+uuvT5IdH8W9UUhePYb4lxDHG3TrkPHxiY5QhrTGY+Xln4WdNoEJIT+ImDiKScpe1m5YsWSKvv\/56qc\/dPDQ4iru01n3zk1z\/\/Gfy6oe5PXTq7VJR9qhWURrX3EH6tqku2yFSRYTPt\/Wkd2kl28PHHHOMeeHq1atje3GgQwc3dLRp00a6desm9erVC+VquXDhQhPuCLV9woQJstdee5WoONbz2267TVDDuRwPiW1DENuMxcxiSM36NSrJrlfM8QUMaXtY05oy9LjG5vsk1qyx9VxEBaukjgjIPIopZoyHeY0vqVk\/s0c9a9YseeKJJ0xIo169ekmXLl1k9913z0pwGx+c62pvvPHGEo4fixYtMuvz1q1by1dffWUcQ\/r16ydt27bN1LXQBnOEcdC05SV8ZyEtKvJlxzZSAgeMBiV1GJpEm6fQMR62FjnVbwrCwYNbKgmagIQl+okb+QS1mu+5jocU1kXz73\/\/u\/HPdm++zLfBSOcaVSpK46vnZdocFMUxLCjlKZ+SOvnezneM51vDrKS2EpvAg9OnT5dPPvlEevbsaYIQuq6Xy5cvlzvuuMP4WhO7jDu3Xn75ZWMoQzKT7EX2Bx10kHTu3Fm+\/fZbY4hj3Y0VvBD1G0I\/uuRjufHpX9cnD53bSo5tUTtfHMptfiV18l2fCqkxjk2aNEmmTZtmSMq21oknnmi2qLiCx5vslhbraP5mTU1wQq6tZauLdMIJJ2Qk+IoVKwzZCZHE567xLZ8GP\/Dyehk07a3M+tjegJB8N229b1RSJ993+YzxQmrnK6mRopCxQ4cOgUQu5GVhngnbYNbQXA1KUnU7DLL+eZTUhWNX6JNhx3ih5YdaUxdaeCHPhWmw9xZB90qTQt5Znp9RUiff+2HGeDG12ipJ3W3CkswhEgjNpWOaCkNASV0YbsU8paT2oOeq3UfsU1OeGNi6GHzL\/bNK6uSHQCqkxuqNsaxq1aolDGNBn0cJS64GWymt6+hoUFdSR4NjPqXkGuP5lOWXt4T6DWm52ZKDIWPGjJEhQ4ZI9epbIluQ8KkeO3ascfZIw0vLldIjTm4qA48oeQa9WDDK4\/NK6uR7PVFSc8iEU16cS7VngvG6clPXrl3N3rLdf44akmwNVikdNdoiSuroMc1VYqKktpVB9cabimOhqOBJpqAGuxZvDpdwyERT8QgoqYvHMN8SEiW1XTNzFJS9ag6SeBMHRVDJ\/Q6h5Ns4v\/xBDR7y6EqZ\/OKH5pF\/X9tez3JHAbaopI4IxryKSZTUNoj\/mWeeadbOqODe1KBBA3McNMlbL7370pBaUzQIqKSOBsd8SkmU1PlULK68fg12Sa370tEir6SOFs8wpaVCamsFLyvqd\/e\/\/lvm\/jeYn6reYYZN+DxK6vBYRZUzFVJbK7irfvM3211HHXWUCWmUpPpd67LZBk\/2plX1jmpobSlHSR0tnmFKS4XUfhVDar\/yyiuZcERJbWm5e9O39tpX+rbbcrOhpmgQUFJHg2M+pZQZUlPp77\/\/XsaNG2figSd1+MQltare+QydcHmV1OFwijJXKqQOWlMvXrzY7F8T3CCu\/Wtvg90DJ6p6Rzm0VP2OHs1wJaZCar81NdVlO2vgwIGy3377hat9Abm8Dbbr6cOa1JAZFx1YQIn6SDYEVFInPz5SIXXyzfz1jW6DdSsr\/p5QUsePsfcNqZGaaKJYuYlHRrTPGTNmyKpVq+Tss8+OzfJN44Mkta6n4xl8Sup4cM1WaiqkJhA\/p8Z+85vfSPfu3U2QQY6Ncjkev4cOHZqIQ4cbDOGLsZ2SR78cvFFJnXwnp0LqoAvywob+DYLJPm8DD55zzjnmsgB3e8xtcOvhCzLX2qiRLJ7Bp6SOB9cyJ6nZusIF86STTipxF\/XSpUvlvvvuK\/gq2\/HjxxtjG+Xis02w\/8GDB5e4ycOS2l1Pa4ST+Aaekjo+bINKTkVSU5mVK1fKJZdcYlRvYnW\/9dZbwkV4o0aNkkMPPbQgJFiTc8MHYYe5e4s1O9f71K1bN1OebbC7Pz2hdwvpffCveQp6uT7ki4CSOvmBkRqpaSpX2hKon3X0zjvvbC7AK3Z\/Gi1g2LBh8uSTT5rfGN686jcX5D25fIO54I505yl1pW39LRera4oWAb0gL1o8c5WW2gV5hC3CUPaHP\/zBBOSPI3FT5jXXXCN9+vSRdu3alZLUrhOHGsni6IEtZaqkjg\/bMqV+c86bi\/E2btwoJ598ctHSmcZhUZ84caIpz96Gme0uLT1JlsxgU1Ing7P7llTUb6zfAwYMEAxj3sS9V5MnT8777LffXVrDhw+X\/v37S4sWLUpI6jmvvClYvkl6kizeQaekjhdfv9JTIXVczWRLCyJjMCMF3aXlklqNZHH1hqrf8SIbXHqipC4rMcqU1MkNN5XUyWFt35QoqctKjLIRD74kF01dbjDQ8EXxDjoldbz4bvPqdxj4mMW6jHhapi76yGRXy3cY1ArPo6QuHLtCn0xUUruVTNOho+Xlj5oL8DR8UaHDJvxzSurwWEWVMxVSp+3QscvZ95sz32r5jmoYBZejpI4fY+8bUiF1XA4dYeCjwV+dPNlk5Wgo1m9N8SGgpI4P26CSUyF1XA4dYeBreEgX2XDYUJP1yuP3kiuPbxzmMc1TIAJK6gKBK+KxVEhNfeNw6AiDg0tqtXyHQay4PErq4vAr5OnUSE1l43DoyAWCkjoXQtF+r6SOFs8wpaVK6jAVjDpP3R7D5YeGHU2xGsIoanRLl6ekjh9j7xvKHal363uH\/FRnX93OSmisKakTAtp5jZI6eczL1RuV1Ml3d7kjtcb5TnaQKamTxZu3pUZqPKqITsKleG7iwvlevXpF4mPtB6cldb\/29WRsz32TR7ycvVFJnXyHp0LqDRs2mFBDe+yxhwk8uP3222daXqlSJWnVqpXwO45kSa171HGgq4ayZFDN\/pZUSM2Jsuuuu85EFI0rnFFQsy2p1Y86meGnkjoZnN23pEJqopTcdtttcuyxxwqRTpJMltR68CQZ1JXUyeCcOqlRv7mN48EHH5QjjjhC6tSpk6lTUmtqJXUyg01JnQzOqZMaL63XXnvNBAv0pqTW1OpHncxgU1Ing3PqpE6+mb++EfVb\/aiT6wEldXJY2zelsqbm5XhqTZ06VR555BFj6b722mtl2bJl0rNnT3NrhzcReXTkyJFCzPD69eubq3movJu8UUr9IpMqqZMdZErqZPHmbamQGkPZhAkTzB71KaecIpMmTZKrr75aHn\/8cUN2ruNxb9XgBg8C8xMddN9995W5c+fKrFmzjPXc3fpiAD377LNy4YUXBiIJqTU4QnIDTUmdHNapSmoCEHJ53VVXXWX2qG+44QazxbV582a56aabzM0dNWvWzKCxZs0aefjhhw1ZkeKEQuJ5JHetWrUy+ZYsWSKvv\/669O3bNyup9UK85Aaakjo5rFMlNVfioG5fdtll5jI7S2qk9NixYw3B+dwvIeXvv\/9+eeedd0wZO+64Yybb\/PnzzVYZaniVKlXMJHD88cfLdtttl8mDpD7\/kBpy3qE1kke7HL5R79JKttNTu0uLZj722GNmTc21s08\/\/bSceuqp5tjoiSeeKD169JAKFSqUQgOJTbB+DqxcccUVJaQ5mRctWmTU9tatW5urbFHP+\/XrJ23bti1Baj1NltxAU0mdHNapSmpezn1aRD956KGHZPHixebGS4h98MEHl5CsNu9TTz1lJDRX02IAc6VvEGx+d2khqassvlt2XPNi8mjrGxWBhBBYvXp1bG+q8AvsDUh8hREMlRo1OugaW+6a5pZMjGVYvoPU8ptvvtncdd25c2dTLmo96+ukT63FhqYWrAiUAQQCSc1MgqGMy+YbNmwoqNZcNs962ktc1OrzzjvP5LMSukGDBoboCxZsuejuhBNOEDy\/+GzFihVGDfe7S6sMYKJVUAS2agR8SY1BDAs3i3pryEJas2fNfjR70HF5aW3VaGrlFYEygIAvqYPifgd9XgbaoVVQBBSB\/yLgS2qk8l133SX77befceiwlm62pF588UW5\/PLLSxw+UTQVAUWg7CDgS2oOn2DFnjdvnjRu3FgqV65swgWzrmatzP8kTphpUgQUgbKFgC+ps3lpudUnKoomRUARKFsIZN3Soqr4ViOl2c5yT4dF2QzU\/XvvvVceeOABc9IMIx374X4HXKJ8b3kpi61JzhFwmg8D57nnnivdunUrdZZg4cKFcsYZZ2RgOf\/8843GpilaBJYvXy7PP\/+8DB48ONqCs62p+Y7tJ\/aRn3nmGZOVLajevXvLpZdeWuqkWLE1wwGEH5xG8PIaMWKEGUyo+pqKR4BBhI2E\/mSiZPfi97\/\/fanzASyniEvXrl274l+qJZRCAAcpDnNxWvOoo46KbcL0ldRIZpwxmjdvbo6EQmi2uagQzhpDhw6N1FA2evRo6dixowlyiFTBaQQDnar30TCDCDbgevrpp5sCp02bZn7b\/+1bJk+ebM4itGzZMpoXayklEMCn4s033zQuzAivuLSgwC0tv8CD9vAIM77rfVVM3zFZcCClT58+mcHEoEOinHbaacUUrc\/+FwEmTZYzRx99tPnE7mK4gwrSjxkzxkS84XAQ+dnlaNKkieIYMQIsc9BMEyU162hUNNZe+EfbtGrVKrMuQ4oHeWnl235IjRPIWWedJc2aNctIEiV1vkgG53c1oSBSo53NnDnTkLlu3bqCmywqO0sh1802ulqV35JSIbXteIiNV9aBBx4ob7\/9tvHawl2SwAlRJa+6zf8MQk6zMcA0FY+AV93mfwxm3bt3DyyciZ3JdsCAAcaZR1N0CKRGaprA+vmFF16Qzz\/\/3Fi\/Wdw3bdo0cqs0qsgrr7xiIqqw1oDUqCZIDE3FI4ChbMqUKcbhhoT0xZGmRYsWmcKtUw7GSgyUrP3uuOMOk7datWrFV0JLyCCQKqmT6ge7pYVBZ+eddzZWcJXS0aFvt7QgKX+7jjR2dwOHGxxz0M5+\/vln2W233YzzjjfOXHS1Kr8llQtSl9\/u1ZYrAtEjkPPwSfSv1BIVAUUgTgSU1HGiq2UrAikgoKROAXR9pSIQJwJK6jjR1bIVgRQQUFKnALq+UhGIEwEldZzoatmKQAoIKKmLAJ19dVKuM+ocr8W1lMMffveQFVIFYsVxug8XyrBniOPeH82nHdbN0+8+tXzKCcpry+\/atauMGjUqMtyjqFvcZSipi0A4TVLz7vfeey80oWlmsaTm4Ao\/YWK654KVunDsOE7CxTGZ5mpXWfheSZ2jFxjE\/\/znP83g4yohoqtyswh\/24ACuIr26tXL+J7j7II3Gz7JnM6qUaOGOT+NZLVSY9OmTUIMdPxqObGFl5rfCTpO2s2YMUNuueUWc3wWp5eLL75Y3njjjcy7\/QIZEI7q9ttvN5crUD4aAm6sEIny0Bbuu+8+ad++vakvIZ95F0dJcdihfpww42Qft61wbJff+FtTFrHbccPlc8JCg8nGjRvNBANeHC0GE670sXX2xox3SU0XcBzRa4TAAAAESUlEQVQVhx6Ivm7dOnNkmKOs7kWMdmJyJ4Nsk4OSuixMMWWwDpAJcjJgGfwMKEgzaNAgcSX12rVrzUBm0BNogHwMap5zBxdSDhLvv\/\/+xled7xjQOE8QD85NuEjyDiYRvOLGjx9vIsNccMEFhlR+qj\/khGy4xp599tlm8uF\/fpDs1I8JhauP7r77biF0FW3BKwui2\/DPt956q6kPSwuepS3Uk3pwVp\/JwV6ayGWIu+++u2kr5fAseZkI3Dq7kWy8pB42bJi5jJH3f\/fdd+YeNgI5HHnkkSUw8ZJYSV2aNCqpc0wkODpYqYE\/srsmdkkNmRiMOD8g6f7xj38YieolNeUhHZkA6tSpkwkKgWS3\/s5UifIgC4Pafs41SBASv2euCvYjNVoCEpaJYM899zTl8Jx10uAqYcJFIQFddRzXSxu2Cg8tCM7EYElt\/bGtV51bX9c\/G62Cd1k7A+\/mzDkTmSutvaSmzjxjo65wf5vFz+0iJXVuyaekzo2RiaLKwEStxL98yJAhRtK6pIYI48aNM3lQSevVq2ekmpfUSLz+\/fsbFdNNkNw1uOFnjvQihJQd6OvXrzcSEII899xzvqRG8jNpQC5vIAvvmtr93709BZWdyYv3WlIzufC\/X73ccpDq3GfuJj9jmJ\/6japu3TyD1v9K6twDVkmdAyPUbwY8oZ2QUnPmzDFSkugvTzzxRIZYSJZXX33VSEmkoCu9XPUbSc11wFaVpQAmBII6uoEdkZpI2+OOOy4jqZFcd955pyEt7\/OT1CwDeI48LANQr5HOEBL124244RIHNRktg3UsarIbfQaiWlLbeuFnb8NNoZXg4cUExmQCVr\/73e9M\/VwNJpv6jfZA9Bt890lMmODO0sBNqPfYCtBimHhU\/Vb1O\/c058lh18rcK8aaGlLzw5oPw5GN\/cXf+IRDKNbckBZS8RwBJu655x6znmVN7V5p9P777xvJy2TgDUaA4Q1C8j2x1jFioQFAvCDLOyQmf6tWrcyanckFEkBMwhQFkRoyolmwDqdOEJRbTolj5pIaeLjSePbs2Zn1L22EyDzjtQMQIZabXVjCuEYvvzV17dq1TQglrjmmTJ5xrznm3Vy9S10hNZMQEyR+\/35WdDWU5T3cy8cDkNZatVF\/kXgYtVBRkRpcDMhd3L\/97W\/NOnn69OnSqVMnOfPMM00+QkIRXAKVHV9xBh9RJSE94YOIAYbU9wuJ7FqkrVTG+s3aNNt2mlWlKR9LNQRhuZBN\/YYYTCzkQWIihSHpxIkTjX3ASmrqgQrO52gNWKyJoMI7yY8vNloE7UTLYQIiFK43\/JUfqVm2EMoYzYWyTj75ZDMR0NaXXnrJlLnDDjtkwkmzswDuTEJMmCxtCIqJ3YEJUkldPjiqrYwAAUiHpdoaDSE9ExxSNmzyI7VrPwhbTrZ8SuooUNQyygUC9uALMeCx+KORcLLNu\/2UDQwldXxD5f8B6kue2qoN53AAAAAASUVORK5CYII=","height":148,"width":245}}
%---
%[output:1a80e500]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPUAAACUCAYAAABLGW+aAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQu8lVP6x5+IktIVqVSUlElSLl3cQkxGERXJpOQWhT9KLjMYldJoSDXiHyNDuSu3f2GqKWpETUgqQlHu16JB\/D\/f1axtnfe8797v3vu9nDrP+nzO55yz93rXu9Zvrd96nvWs9Tyrwi+\/\/PKLaFIEFIFtBoEKSuptpi+1IYqAQUBJrQNBEdjGEFBSb2Mdqs1RBEqQ+osvvpABAwbI0qVLQyGzevXqUPk0kyKgCCSHQClS33DDDXLddddJrVq1staCCSBXnuSakf+bFi5cKGeccYY88MAD0q5du\/wLyPKEt+zRo0cLn02ePDkyzL7\/\/nsZNmyYqcWoUaPkgw8+kP79+8vgwYPltNNOi7Q93sLCYvfggw\/K7bffLvfcc4\/ss88+WeuUT958GmdxevLJJ+X888+XK6+8MtTjXnx32mmnUM9FnWnVqlUydOhQufnmm3NiaN9dgtT\/+c9\/5K233pLmzZtLpUqVstaPvLnyRN3ArbW8bYnUVps7\/fTTc04e+RA1n7z5jANIwWTXtWvX0ISm\/LJCauoCNtOmTQstFEpJ6pEjR0qHDh2kU6dOUrNmzXzwK5XXzuj2i5tuuskMBDswGjZsKGvWrDHq\/gEHHFCi0u6z7nf288MPP1zmzZtnis42AwPIVVddlambzetKm2bNmpllh5XY69atKyH9+J9k67Fy5Uoj5W17vJLL\/f\/999\/PvL9evXqlpJb3WTsIrcQNwtAddMzkgwYNyiybbBuZTCZNmlSq7Xxgv6NOu+66q8ljNQlXuvG5bSd\/U58rrrgi0w7vks2LkX05GlHt2rUNwSyeFo\/PP\/\/c4OnmpS\/cvoOUaCR+EtNiZsu17ffWzW+ceJ+19ec9aEKMz08\/\/dTU2R2HXoxcnOwEdfXVVwt8+vOf\/2zGVjbMvXV1NUjvmMhFylKSeu7cufLII4\/IggULpGXLlqYTIHnVqlVzlVXiezsY7YCwHcT\/nTt3NiQCLFQzkjubus9269YtAy6DzhLKHbioVn4qXlAdAIxk1W9LamtLYABZojDxuOot3x155JGhSW07M0j99ko+V2LZwe6HocWFdvip39mWABZDcLBtt6R2B7OLt62Dq3XYvLYODHQ7OaLmum1p0KBB1uWCV1Lb8eLW0faFS2yvJLbt9hKbfvCq3q5w8fbxxRdfbOr76quvlhijdrLN1rYZM2aUECRB481izm9ws+2zz1ti23r6tcGPlIHW7w0bNsiSJUtk+vTp8txzz0nHjh2ld+\/ectBBB\/nOlt7CvSqnBb9t27alCEMnufnvuusucYnqdrBLRjubB63bsql0fpLalVbZZkfvZJFNUucitStxsWVg07AkGTduXIm1uIvhH\/\/4R\/nTn\/6UyRu0pnYlgJU04OtOMi72TCTuBOsOfO9EZ0nsp2lRsSD8XQnsTli2H+0EgJS02gN19Ju8ve+w9fUSxo8Q2frYq357Jyw73l1Jb7UJS0pXw\/Hywf3fq\/l5Jyrv5JNrfR9qS+uHH34wqh2DAemay+BjAXE7xQV7zJgxMmTIkMzM5EdqV2101bI4Se1KgqRI7Q5+1DQ6m\/WqV0PBKOliOH78eGM8CZLUXrWSfJDaPuf2jbtms6S2qqzF3k97od+8yxvyWwnjJZx3OeCnsqJxWVJDYjf5LV+8ZLdjz0pYVH5Xe3DLK4bU3mUR5XpJbXHw44OLuSW1VzCG0TbyktRk\/vnnn+XDDz+UWbNmGZWc1KdPH+nZs2dOI1k+kppy7fqFCcMrSdyKe6ViNmnsSnhmavdZd3KwKqgfqa2BxVWBTjnllBKW5mIkNfVwZ2ZXGmXDMJek9g52ryaUS1L7WdGzSQx3ErEaARqelb6009UAvITKps5mk0xRSGq\/Prbqt500XUltv7MTo3cZ4lWfKSOMpA7aibFY2bW5H5Hdz0pJaohMIc8++6xRd0g9evQwkoOZskKFCrnKNN9Hsaa2jfRTVYKkgd9MbDstqBw\/UntnV3cdag0+brloFrZO+W5puUYX1yCUDcNca2p3sFtpBTZMmpAN42G2NXWQBuBiSB5XCnpVVga3JbVXYro2Foynfmtqrzpu6xTVmjpbHzMxuVuGXvXbb8nC0pK1uR+p\/WwEtj+8OPqNH2u8zaV6U1YJUn\/55ZcycOBAeeedd4w0Pumkk8ze2HbbbReKyN5Muazf5PezLLqTAn+7alc+kppnXfUwqBw\/UrsS1Kqido3ktXxaS3wQqW2d\/dRHi5l3kNvPw1i\/GUhW27H7seeee27mIBEDFC2E79yJkokojPXbtRp7rd\/e+rkWYld68153JwAj5Zw5c8QSwdoEwNqdsO3OhXd3xG\/y9lq\/yZPLyORdpnj72E9SY3Bzn8NwascLy4fFixdnJk33DEQ+1m9bj0j2qdevX2\/WNBUrViyIyGEeynfhH6ZMzVM4Avnso9sJjR2RuA+5FN6isv9kPpjn25pSkhpDCnueufaokeq58gRVRkmdbzdFm9\/VCrxqfBj1zqstRVu7bbO0YjHPB5VSh08uvfRSc6IsV+cyY7O5rkkRUATKFgKlDp+89tprwhHQMOmwww4Lk03zKAKKQIIIhNqnTrA+svfeeyf5On2XIpAoAj807CjftTlbvhjbKbb3lklSq0tnbP1dqmAmUcU7ObynLvpILpq6XEmdHOTl703vvvuu7LXXXuWv4Sm1WEmdEvDl6bVK6mR7O3VSczDk5Zdflo8\/\/li6dOli9q6tm15cUKg6GBey\/uUqqZPFe\/TM92T0zHfTUb\/nz58v119\/vRxyyCEmqoaNvtC3b1859thjY0NCSR0btL4FK6mTxTs1UuN2CaE5aohktiGO8ODhaCHf5etfHRY6JXVYpKLJp6SOBsewpaRGak584duLJ9D222+fITUVDxvDLFsjf\/rpJxk7dqw0atSo1FFDJXXY4RFNPiV1NDiGLSW1NTWkw1e3fv36RtXm72uvvVZeeOEFE8OMk2Q77rhj2HaUyvf000\/LX\/7yF6MJeM8PK6kLhrWgB5XUBcFW8ENsZ0HsVPapv\/nmG+M2N2XKFPnxxx+Nkez444830rsYYxkujJyDxWmkSpUqvqRm8tCUDALYS+gLTckgcOh1M+WnOvumQ2rbRKKesMbmLHiu8+C5YKEcQvTgdvevf\/3LZFdJnQu1eL9XSR0vvt7Sa10223yUuKQmUMLXX38tQXfnESihevXqeflZU9ZDDz1kpPzRRx9tpLWSOtkB5fc2JXVyfbDmi03SeviCdEiNWyWO4ITgbdy4sVSuXFlQx9euXStNmjTJoEB0lLDJL6Qqz3rDtuqaOiyi0eRTUkeDY5hSuk1cIvPf\/iodUmMou+WWW0ysYhvVAUlLpIply5bJBRdcIC+++GLmuzAN8uZRSV0IatE\/o6SOHtOgEq3qXfGzFfLJlAtie7GvQwdbWn5bV+7n1KiYa3eU1LH1aV4FK6nzgqugzK7a3bBWZfnm7j6xOtH4khp\/amJK4y+NxZsYZVZSP\/bYY+bGhjfeeCPyO6hATNXvgsZNwQ8pqQuGLtSDjy35RM65b1km74wLD5S+x7VJntTU4KOPPpLhw4dn1tVIaazfhDvCUEakxUcffTRUw\/LJpKTOB63i8yqpi8fQrwRXOvM9Enr86S3ksKY1YhdcOf2p2YZiW6sQi3chcCmpC0Gt8GeU1IVj530SIk+cu1bunPdBia8gNBKa30loo1lJjcWaH5vY5uIwCsHMCw06mAtCJXUuhKL9XkldHJ4Qeezz78uUhVsuUXSTl8z2u7jHeCCpWTsTc5nTZDZVq1bNHByB1MUeRAmCMu4GF9eF297TSur8+hQSv\/ze1zJlwTqZ\/86W7amwZE6V1OxJc9abyKJsb7F9ddZZZ8nDDz9sznx37949PyTyyK2kzgOsCLIqqYNBhMCkQVOXBxLYPo1Unj7wQGlUe4uKnS3FPcZzbmlt3rzZXOV5ySWXmAMo7F\/j0LHLLrvkqntB38fd4IIqtQ0\/pKTe0rkQeNm6DfLXuWtzEpj8kPjwpjVlyHGNM2vlsMMk7jGedUuL62vbt29vPKrOO+882bhxo7lAG68tXVOH7cKyna88kRriQsZ7Xlonjy\/5OBR5LYH5\/T\/HNJJO+9bKm8TeEZAKqakEt13ee++95k5q\/r7oootk06ZNZo8a9TvsRXn5Dum4G5xvfbb1\/NsiqSEvxzGnLVova77cZKRw2ATpG9asLN0O2E2O26920QT2e2\/cYzznlpatFGtrDqDssMMOYfEpKF\/cDS6oUtvwQ1sjqSHpdz9ulvGz18qaz78PLXHdbrTkPahRdenXoV4s5A0aNnGP8ZxravcoKIEIOWnGEdJijoimaUTYhvlZUNPKGqmtVLXrWxoVZGXO1WC7L3xYk5pyU\/em8uV3PyVK3jJBantx3dKlSwPxwv+ZQAm6pZVrSG0d3ydJakvYnzb\/IrfPXiPvfPpd3uqxF1Urcds2qi79O9QzX1syl9UeSEVS43oZ9vbLqIGLu8FR13drLy9KUkPap177VP5v2WcGlkIlrMXUkpM1bsemNaX3wXW3CtLmGhNxj\/HQa+pcFY3q+7gbHFU9t5VycpHaSteXVn9ljE+sYfM1PgVhZaVso9o7GSlbp+qWuHdlXdIW2\/dxj\/FSV9kOGDBAsqnfBxxwgEyePFnX1MX2bMrPQ9ZfRGTqvJXy0gebTW2iJKshZ83KAmHPO7yBVN+pYrkgbJhuTZTUYSoUd564Gxx3\/dMs30rVdz\/7Xh7\/9yeyOoI1a9Aads9aleWoZrXk0L2qK1nz7PS4x3hW9Zv9aSJ7EsS\/WbNm5iBKXFZvi0vcDc4T\/zKRHbKu+GijPLvsM3n7k+KNS35EtZK1cZ2dpGOTGtJ+7xrbvBqcVufGPcYDSW2v3TniiCMER47PPvtM+GzEiBEmeEJcKe4Gx1XvsOXaU038nr3iC+McsPaLTZGpvm49XEPTfvWqyoCO9WWf3aqYwxj2u1xr6rDt0nzhEYh7jPuSGh\/qa665xgTbb9myZaa2RDv529\/+ptfuePrPqr1VK28v\/zv\/Q5m\/6stI16hBRG3VoJp0aVlH9qy5xYmgEAOTkjo8GaPKmQqpw8Qoi0sNj7vBYTvGEvXHn3+Wu+Z9KG+u22AeLXabxu\/9rkTdv341ObXNbsYSXAhJw7bP5lNS54tY8fnjHuO+kppIJyNHjpTmzZtLjx49zO0cxAJ\/6qmnZMGCBUZSV6pUKdM6jo8+88wzxvED10wqjS821\/a4yXu4xc+SHneDbX0g7S6VK8ofZrwt7xd41DCoe12SkueE\/etIl5a7SoUCpWnxwyi4BCV1nOj6lx33GA9cU3MklOOgNvb3unXrZJ999pHRo0fLnnvuWaK2xAPHc4tL9WrXri0zZ840oYQvv\/zyEvkYQMQKv\/DCCwORjKPBEPiu+R\/I0rXfFixpXaI2rL2T9Gizm7H+uuvT5IdH8W9UUhePYb4lxDHG3TrkPHxiY5QhrTGY+Xln4WdNoEJIT+ImDiKScpe1m5YsWSKvv\/56qc\/dPDQ4iru01n3zk1z\/\/Gfy6oe5PXTq7VJR9qhWURrX3EH6tqku2yFSRYTPt\/Wkd2kl28PHHHOMeeHq1atje3GgQwc3dLRp00a6desm9erVC+VquXDhQhPuCLV9woQJstdee5WoONbz2267TVDDuRwPiW1DENuMxcxiSM36NSrJrlfM8QUMaXtY05oy9LjG5vsk1qyx9VxEBaukjgjIPIopZoyHeY0vqVk\/s0c9a9YseeKJJ0xIo169ekmXLl1k9913z0pwGx+c62pvvPHGEo4fixYtMuvz1q1by1dffWUcQ\/r16ydt27bN1LXQBnOEcdC05SV8ZyEtKvJlxzZSAgeMBiV1GJpEm6fQMR62FjnVbwrCwYNbKgmagIQl+okb+QS1mu+5jocU1kXz73\/\/u\/HPdm++zLfBSOcaVSpK46vnZdocFMUxLCjlKZ+SOvnezneM51vDrKS2EpvAg9OnT5dPPvlEevbsaYIQuq6Xy5cvlzvuuMP4WhO7jDu3Xn75ZWMoQzKT7EX2Bx10kHTu3Fm+\/fZbY4hj3Y0VvBD1G0I\/uuRjufHpX9cnD53bSo5tUTtfHMptfiV18l2fCqkxjk2aNEmmTZtmSMq21oknnmi2qLiCx5vslhbraP5mTU1wQq6tZauLdMIJJ2Qk+IoVKwzZCZHE567xLZ8GP\/Dyehk07a3M+tjegJB8N229b1RSJ993+YzxQmrnK6mRopCxQ4cOgUQu5GVhngnbYNbQXA1KUnU7DLL+eZTUhWNX6JNhx3ih5YdaUxdaeCHPhWmw9xZB90qTQt5Znp9RUiff+2HGeDG12ipJ3W3CkswhEgjNpWOaCkNASV0YbsU8paT2oOeq3UfsU1OeGNi6GHzL\/bNK6uSHQCqkxuqNsaxq1aolDGNBn0cJS64GWymt6+hoUFdSR4NjPqXkGuP5lOWXt4T6DWm52ZKDIWPGjJEhQ4ZI9epbIluQ8KkeO3ascfZIw0vLldIjTm4qA48oeQa9WDDK4\/NK6uR7PVFSc8iEU16cS7VngvG6clPXrl3N3rLdf44akmwNVikdNdoiSuroMc1VYqKktpVB9cabimOhqOBJpqAGuxZvDpdwyERT8QgoqYvHMN8SEiW1XTNzFJS9ag6SeBMHRVDJ\/Q6h5Ns4v\/xBDR7y6EqZ\/OKH5pF\/X9tez3JHAbaopI4IxryKSZTUNoj\/mWeeadbOqODe1KBBA3McNMlbL7370pBaUzQIqKSOBsd8SkmU1PlULK68fg12Sa370tEir6SOFs8wpaVCamsFLyvqd\/e\/\/lvm\/jeYn6reYYZN+DxK6vBYRZUzFVJbK7irfvM3211HHXWUCWmUpPpd67LZBk\/2plX1jmpobSlHSR0tnmFKS4XUfhVDar\/yyiuZcERJbWm5e9O39tpX+rbbcrOhpmgQUFJHg2M+pZQZUlPp77\/\/XsaNG2figSd1+MQltare+QydcHmV1OFwijJXKqQOWlMvXrzY7F8T3CCu\/Wtvg90DJ6p6Rzm0VP2OHs1wJaZCar81NdVlO2vgwIGy3377hat9Abm8Dbbr6cOa1JAZFx1YQIn6SDYEVFInPz5SIXXyzfz1jW6DdSsr\/p5QUsePsfcNqZGaaKJYuYlHRrTPGTNmyKpVq+Tss8+OzfJN44Mkta6n4xl8Sup4cM1WaiqkJhA\/p8Z+85vfSPfu3U2QQY6Ncjkev4cOHZqIQ4cbDOGLsZ2SR78cvFFJnXwnp0LqoAvywob+DYLJPm8DD55zzjnmsgB3e8xtcOvhCzLX2qiRLJ7Bp6SOB9cyJ6nZusIF86STTipxF\/XSpUvlvvvuK\/gq2\/HjxxtjG+Xis02w\/8GDB5e4ycOS2l1Pa4ST+Aaekjo+bINKTkVSU5mVK1fKJZdcYlRvYnW\/9dZbwkV4o0aNkkMPPbQgJFiTc8MHYYe5e4s1O9f71K1bN1OebbC7Pz2hdwvpffCveQp6uT7ki4CSOvmBkRqpaSpX2hKon3X0zjvvbC7AK3Z\/Gi1g2LBh8uSTT5rfGN686jcX5D25fIO54I505yl1pW39LRera4oWAb0gL1o8c5WW2gV5hC3CUPaHP\/zBBOSPI3FT5jXXXCN9+vSRdu3alZLUrhOHGsni6IEtZaqkjg\/bMqV+c86bi\/E2btwoJ598ctHSmcZhUZ84caIpz96Gme0uLT1JlsxgU1Ing7P7llTUb6zfAwYMEAxj3sS9V5MnT8777LffXVrDhw+X\/v37S4sWLUpI6jmvvClYvkl6kizeQaekjhdfv9JTIXVczWRLCyJjMCMF3aXlklqNZHH1hqrf8SIbXHqipC4rMcqU1MkNN5XUyWFt35QoqctKjLIRD74kF01dbjDQ8EXxDjoldbz4bvPqdxj4mMW6jHhapi76yGRXy3cY1ArPo6QuHLtCn0xUUruVTNOho+Xlj5oL8DR8UaHDJvxzSurwWEWVMxVSp+3QscvZ95sz32r5jmoYBZejpI4fY+8bUiF1XA4dYeCjwV+dPNlk5Wgo1m9N8SGgpI4P26CSUyF1XA4dYeBreEgX2XDYUJP1yuP3kiuPbxzmMc1TIAJK6gKBK+KxVEhNfeNw6AiDg0tqtXyHQay4PErq4vAr5OnUSE1l43DoyAWCkjoXQtF+r6SOFs8wpaVK6jAVjDpP3R7D5YeGHU2xGsIoanRLl6ekjh9j7xvKHal363uH\/FRnX93OSmisKakTAtp5jZI6eczL1RuV1Ml3d7kjtcb5TnaQKamTxZu3pUZqPKqITsKleG7iwvlevXpF4mPtB6cldb\/29WRsz32TR7ycvVFJnXyHp0LqDRs2mFBDe+yxhwk8uP3222daXqlSJWnVqpXwO45kSa171HGgq4ayZFDN\/pZUSM2Jsuuuu85EFI0rnFFQsy2p1Y86meGnkjoZnN23pEJqopTcdtttcuyxxwqRTpJMltR68CQZ1JXUyeCcOqlRv7mN48EHH5QjjjhC6tSpk6lTUmtqJXUyg01JnQzOqZMaL63XXnvNBAv0pqTW1OpHncxgU1Ing3PqpE6+mb++EfVb\/aiT6wEldXJY2zelsqbm5XhqTZ06VR555BFj6b722mtl2bJl0rNnT3NrhzcReXTkyJFCzPD69eubq3movJu8UUr9IpMqqZMdZErqZPHmbamQGkPZhAkTzB71KaecIpMmTZKrr75aHn\/8cUN2ruNxb9XgBg8C8xMddN9995W5c+fKrFmzjPXc3fpiAD377LNy4YUXBiIJqTU4QnIDTUmdHNapSmoCEHJ53VVXXWX2qG+44QazxbV582a56aabzM0dNWvWzKCxZs0aefjhhw1ZkeKEQuJ5JHetWrUy+ZYsWSKvv\/669O3bNyup9UK85Aaakjo5rFMlNVfioG5fdtll5jI7S2qk9NixYw3B+dwvIeXvv\/9+eeedd0wZO+64Yybb\/PnzzVYZaniVKlXMJHD88cfLdtttl8mDpD7\/kBpy3qE1kke7HL5R79JKttNTu0uLZj722GNmTc21s08\/\/bSceuqp5tjoiSeeKD169JAKFSqUQgOJTbB+DqxcccUVJaQ5mRctWmTU9tatW5urbFHP+\/XrJ23bti1Baj1NltxAU0mdHNapSmpezn1aRD956KGHZPHixebGS4h98MEHl5CsNu9TTz1lJDRX02IAc6VvEGx+d2khqassvlt2XPNi8mjrGxWBhBBYvXp1bG+q8AvsDUh8hREMlRo1OugaW+6a5pZMjGVYvoPU8ptvvtncdd25c2dTLmo96+ukT63FhqYWrAiUAQQCSc1MgqGMy+YbNmwoqNZcNs962ktc1OrzzjvP5LMSukGDBoboCxZsuejuhBNOEDy\/+GzFihVGDfe7S6sMYKJVUAS2agR8SY1BDAs3i3pryEJas2fNfjR70HF5aW3VaGrlFYEygIAvqYPifgd9XgbaoVVQBBSB\/yLgS2qk8l133SX77befceiwlm62pF588UW5\/PLLSxw+UTQVAUWg7CDgS2oOn2DFnjdvnjRu3FgqV65swgWzrmatzP8kTphpUgQUgbKFgC+ps3lpudUnKoomRUARKFsIZN3Soqr4ViOl2c5yT4dF2QzU\/XvvvVceeOABc9IMIx374X4HXKJ8b3kpi61JzhFwmg8D57nnnivdunUrdZZg4cKFcsYZZ2RgOf\/8843GpilaBJYvXy7PP\/+8DB48ONqCs62p+Y7tJ\/aRn3nmGZOVLajevXvLpZdeWuqkWLE1wwGEH5xG8PIaMWKEGUyo+pqKR4BBhI2E\/mSiZPfi97\/\/fanzASyniEvXrl274l+qJZRCAAcpDnNxWvOoo46KbcL0ldRIZpwxmjdvbo6EQmi2uagQzhpDhw6N1FA2evRo6dixowlyiFTBaQQDnar30TCDCDbgevrpp5sCp02bZn7b\/+1bJk+ebM4itGzZMpoXayklEMCn4s033zQuzAivuLSgwC0tv8CD9vAIM77rfVVM3zFZcCClT58+mcHEoEOinHbaacUUrc\/+FwEmTZYzRx99tPnE7mK4gwrSjxkzxkS84XAQ+dnlaNKkieIYMQIsc9BMEyU162hUNNZe+EfbtGrVKrMuQ4oHeWnl235IjRPIWWedJc2aNctIEiV1vkgG53c1oSBSo53NnDnTkLlu3bqCmywqO0sh1802ulqV35JSIbXteIiNV9aBBx4ob7\/9tvHawl2SwAlRJa+6zf8MQk6zMcA0FY+AV93mfwxm3bt3DyyciZ3JdsCAAcaZR1N0CKRGaprA+vmFF16Qzz\/\/3Fi\/Wdw3bdo0cqs0qsgrr7xiIqqw1oDUqCZIDE3FI4ChbMqUKcbhhoT0xZGmRYsWmcKtUw7GSgyUrP3uuOMOk7datWrFV0JLyCCQKqmT6ge7pYVBZ+eddzZWcJXS0aFvt7QgKX+7jjR2dwOHGxxz0M5+\/vln2W233YzzjjfOXHS1Kr8llQtSl9\/u1ZYrAtEjkPPwSfSv1BIVAUUgTgSU1HGiq2UrAikgoKROAXR9pSIQJwJK6jjR1bIVgRQQUFKnALq+UhGIEwEldZzoatmKQAoIKKmLAJ19dVKuM+ocr8W1lMMffveQFVIFYsVxug8XyrBniOPeH82nHdbN0+8+tXzKCcpry+\/atauMGjUqMtyjqFvcZSipi0A4TVLz7vfeey80oWlmsaTm4Ao\/YWK654KVunDsOE7CxTGZ5mpXWfheSZ2jFxjE\/\/znP83g4yohoqtyswh\/24ACuIr26tXL+J7j7II3Gz7JnM6qUaOGOT+NZLVSY9OmTUIMdPxqObGFl5rfCTpO2s2YMUNuueUWc3wWp5eLL75Y3njjjcy7\/QIZEI7q9ttvN5crUD4aAm6sEIny0Bbuu+8+ad++vakvIZ95F0dJcdihfpww42Qft61wbJff+FtTFrHbccPlc8JCg8nGjRvNBANeHC0GE670sXX2xox3SU0XcBzRa4TAAAAESUlEQVQVhx6Ivm7dOnNkmKOs7kWMdmJyJ4Nsk4OSuixMMWWwDpAJcjJgGfwMKEgzaNAgcSX12rVrzUBm0BNogHwMap5zBxdSDhLvv\/\/+xled7xjQOE8QD85NuEjyDiYRvOLGjx9vIsNccMEFhlR+qj\/khGy4xp599tlm8uF\/fpDs1I8JhauP7r77biF0FW3BKwui2\/DPt956q6kPSwuepS3Uk3pwVp\/JwV6ayGWIu+++u2kr5fAseZkI3Dq7kWy8pB42bJi5jJH3f\/fdd+YeNgI5HHnkkSUw8ZJYSV2aNCqpc0wkODpYqYE\/srsmdkkNmRiMOD8g6f7xj38YieolNeUhHZkA6tSpkwkKgWS3\/s5UifIgC4Pafs41SBASv2euCvYjNVoCEpaJYM899zTl8Jx10uAqYcJFIQFddRzXSxu2Cg8tCM7EYElt\/bGtV51bX9c\/G62Cd1k7A+\/mzDkTmSutvaSmzjxjo65wf5vFz+0iJXVuyaekzo2RiaLKwEStxL98yJAhRtK6pIYI48aNM3lQSevVq2ekmpfUSLz+\/fsbFdNNkNw1uOFnjvQihJQd6OvXrzcSEII899xzvqRG8jNpQC5vIAvvmtr93709BZWdyYv3WlIzufC\/X73ccpDq3GfuJj9jmJ\/6japu3TyD1v9K6twDVkmdAyPUbwY8oZ2QUnPmzDFSkugvTzzxRIZYSJZXX33VSEmkoCu9XPUbSc11wFaVpQAmBII6uoEdkZpI2+OOOy4jqZFcd955pyEt7\/OT1CwDeI48LANQr5HOEBL124244RIHNRktg3UsarIbfQaiWlLbeuFnb8NNoZXg4cUExmQCVr\/73e9M\/VwNJpv6jfZA9Bt890lMmODO0sBNqPfYCtBimHhU\/Vb1O\/c058lh18rcK8aaGlLzw5oPw5GN\/cXf+IRDKNbckBZS8RwBJu655x6znmVN7V5p9P777xvJy2TgDUaA4Q1C8j2x1jFioQFAvCDLOyQmf6tWrcyanckFEkBMwhQFkRoyolmwDqdOEJRbTolj5pIaeLjSePbs2Zn1L22EyDzjtQMQIZabXVjCuEYvvzV17dq1TQglrjmmTJ5xrznm3Vy9S10hNZMQEyR+\/35WdDWU5T3cy8cDkNZatVF\/kXgYtVBRkRpcDMhd3L\/97W\/NOnn69OnSqVMnOfPMM00+QkIRXAKVHV9xBh9RJSE94YOIAYbU9wuJ7FqkrVTG+s3aNNt2mlWlKR9LNQRhuZBN\/YYYTCzkQWIihSHpxIkTjX3ASmrqgQrO52gNWKyJoMI7yY8vNloE7UTLYQIiFK43\/JUfqVm2EMoYzYWyTj75ZDMR0NaXXnrJlLnDDjtkwkmzswDuTEJMmCxtCIqJ3YEJUkldPjiqrYwAAUiHpdoaDSE9ExxSNmzyI7VrPwhbTrZ8SuooUNQyygUC9uALMeCx+KORcLLNu\/2UDQwldXxD5f8B6kue2qoN53AAAAAASUVORK5CYII=","height":148,"width":245}}
%---
