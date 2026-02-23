clear;
[model, options] = init_environment('inv_im');;

CTRPIFF_CLIP_RELEASE = 0.001;
s = tf('s');
%[text] ### Global timing
% simulation length
simlength = 2;

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
tc_factor = 100; % tc is ts_afe / tc_factor
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
use_mosfet_thermal_model = 0;
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
use_torque_curve = 0; % for wind application
use_speed_control = 1-use_torque_curve; %
use_mtpa = 0; %
use_psm_encoder = 0; % 
use_im_encoder = 1; % 
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
application_voltage = 400;
% trafo data
us1 = 400; us2 = 400; fgrid = 50;
eta = 95; ucc = 5.6;
i1m = 5; p_iron = 1200;
% rererence for the voltage sequence
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
fres_cllc = glb_time.fPWM_CLLC*1.25;

hwdata.single_phase_inverter = single_phase_inverter_hwdata(application_voltage, single_phase_inverter_pwr_nom, glb_time.fPWM_INV);
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:664e2c2a]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:56ee86e8]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:868af156]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:94c40d02]

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
parasitic_dclink_data; %[output:51602118]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.05;
%[text] ### IM Machine settings
im = im_calculus(); %[output:7fcd2cd7]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:79465b8b]
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
infineon_FF900R12IE4
% infineon_FF650R17IE4D_B2;
% infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;

igbt.inv = device_igbt_setting_inv(glb_time.fPWM_INV, hwdata.inv.udc_nom);
igbt.afe = device_igbt_setting_afe(glb_time.fPWM_AFE, hwdata.afe.udc_nom);
%[text] ### DEVICES settings (MOSFET)
wolfspeed_CAB760M12HM3
% danfoss_SKM1000GB17E4
% infineon_FF1000UXTR23T2M1;

mosfet.inv = device_mosfet_setting_inv(glb_time.fPWM_INV, hwdata.inv.udc_nom);
mosfet.afe = device_mosfet_setting_afe(glb_time.fPWM_AFE, hwdata.afe.udc_nom);
mosfet.dab = device_mosfet_setting_dab(glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
%[text] ### DEVICES settings (Ideal switch)
silicon_high_power_ideal_switch;
ideal_switch = device_ideal_switch_setting(glb_time.fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.afe = device_ideal_switch_setting(glb_time.fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.inv = device_ideal_switch_setting(glb_time.fPWM_INV, hwdata.inv.udc_nom);
%[text] ### Setting Global Faults
time_aux_power_supply_fault = 1e3;
%[text] ### Lithium Ion Battery
nominal_battery_voltage = grid_emu.udc_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage, nominal_battery_power, ... %[output:group:866005dd] %[output:8b3aa18c]
    initial_battery_soc, glb_time.ts_dab); %[output:group:866005dd] %[output:8b3aa18c]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage, nominal_battery_power, ... %[output:group:67c59edc] %[output:9c65305f]
    initial_battery_soc, glb_time.ts_dab); %[output:group:67c59edc] %[output:9c65305f]
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
    set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
    set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
else
    if use_thermal_model
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'off');
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    else
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
        set_param('inv_im/inv_im_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    end
end

if use_torque_curve
    set_param('inv_im/fixed_speed_setting', 'Commented', 'off');
    set_param('inv_im/motor_load_setting', 'Commented', 'on');
else
    set_param('inv_im/fixed_speed_setting', 'Commented', 'on');
    set_param('inv_im/motor_load_setting', 'Commented', 'off');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":37.2}
%---
%[output:664e2c2a]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe400V_250kW\nNominal Voltage: 400 V | Nominal Current: 360 A\nCurrent Normalization Data: 509.12 A\nVoltage Normalization Data: 326.60 V\n---------------------------\n","truncated":false}}
%---
%[output:56ee86e8]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv400V_250kW\nNominal Voltage: 400 V | Nominal Current: 470 A\nCurrent Normalization Data: 664.68 A\nVoltage Normalization Data: 326.60 V\n---------------------------\n","truncated":false}}
%---
%[output:868af156]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_800V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 800 [V] | Normalization Current DC1: 350 [A]\nNormalization Voltage DC2: 800 [V] | Normalization Current DC2: 350 [A]\nInternal Tank Ls: 1.697653e-05 [H] | Internal Tank Cs: 350 [F]\n---------------------------\n","truncated":false}}
%---
%[output:94c40d02]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_800V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 800 [V] | Normalization Current DC1: 350 [A]\nNormalization Voltage DC2: 800 [V] | Normalization Current DC2: 350 [A]\nInternal Tank Ls: 1.280000e-05 [H] | Internal Tank Cs: 350 [F]\n---------------------------\n","truncated":false}}
%---
%[output:51602118]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUgAAADGCAYAAACw2RkvAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQeUFVXSLmBAYMlBkpIFAUFUkjsocWElCUpGssQBZMElSEYyIiJJEUlHoiwjsAsLAg46\/IvKkBSQpAOSJAwgaST5n6+GfjbP9+al7r79Xtc9h8Obed237\/1u9TdVt+pWpfn9999\/J2mCgCAgCAgCf0IgjRCkSIUgIAgIAp4REIIUyRAEBAFBwAsCQpAiGoKAICAICEGKDAgCgoAgEBgCokEGhpdcLQgIAg5CQAjSQYstUxUEBIHAEBCCDAwvuVoQEAQchIAQpIMWW6YqCAgCgSEgBBkYXnK1ICAIOAgBIUgHLbZMVRAQBAJDQAgyMLzkakFAEHAQAkKQDlpsmaogIAgEhoAQZGB4ydWpIHD06FGKiYmhTJkyUdq0aSkpKYkqVKhAI0eOpLx58\/qF3cqVK6lIkSJUrVo1n9fv3LmTevToQUWLFnU9r3Xr1tSpUyf+ec6cOdS0aVMqVqyYz77kAkHAEwJCkCIXhiEAgly8eDENGzaMSfL+\/fv0\/vvvM+E1a9bMr+cESpDLly+nSZMm8fN+++03WrBgAd2+fZuJOioqyq9nersIia7wD2QrzZkICEE6c91NmbU7QYKwJk+eTDVq1KDo6GhasmQJrVixgp8NTa9Dhw78GaS6bNky1jJLlChBjRs3pqpVq9KGDRto\/vz5dPfuXWrQoAFrhiBCrUGD1BMkfn\/27FkaNWoUvfXWW3xvx44dqUCBAkzUX331FZNnu3bt+NnJyck0ffp0iouLY00XfXfu3Jl2795NBw8epMTEROrfvz8\/f8KECUzAGCP6z5EjBz\/jqaeeoo0bN1Lx4sWpXr16tGjRIrp69SqNHz+e5yAtvBEQggzv9bPV6EGQIJgzZ864xtWwYUOaOHEiJSQk0Pbt25lU0GbMmEHlypVjUsLvBw8eTHfu3KGhQ4cygT3yyCP8+z59+lC6dOmYXPPkyUPoLzWCvHHjBpNTmzZtCNooCPLSpUtMdq1ataJr166xxtm3b1\/6\/PPP+TktW7akX375hUaMGEGDBg1igoyPj+dxp0mThj744ANq27YtE+3mzZvp1KlTbLr36tWLevfuTS+++CJfmzlzZurXrx\/t2bOH\/vvf\/\/KcQtVibbXADhyMEKQDF92sKXsysaHhgZwyZszIGlX16tX58dD+QIBo0C6130PDxJ4iNLhx48Y9NFTsN4J0AiVIaKXHjx\/nf\/v27aMvv\/ySpkyZQqtWrWIyLlWqFJvSs2bNor\/\/\/e9MkGggVDSQ6oEDB+jIkSOshZYsWZK6detGY8aMYW0yV65crClr83DHwSy8pV\/zERCCNB9jxzzBEzFov4OmWKdOHZfzBRra119\/zearJ4LctWsXm7MaScE0vnfvnl8mNsxhECk0P2iQ3333Hf+DVpkzZ07WXtGvpmE+8cQTPA78vkmTJg8R5IULF2js2LFs4leqVIkOHz5MO3bs8EiQ2EqAc0kIMnJEXggyctZS+UzciQFaGbS0Y8eO0bPPPsta4RtvvMHjBImBUG7dukXffPMNm974PHDgQOrSpQsTVmxsLGtp6dOnZ+1Or4FqWqi7kwbXwdTFfiVMbRDkmjVrXCSMMcCJBA1y06ZNTMIwsU+fPs1jgJmt1yAxp4ULF\/I9MJffe+89Hr8nDVIIUrkIGj4AIUjDIXVuh+5hPnCCPPnkk0w80Nw8OWmgFX700UdMhjCFsc\/30ksvsbYGcoXTA01z6uj39FIL88E9GkHCuYK9RewnYu8Qe6TZs2dn8xpm\/KFDh9jMzpIlC3Xt2vUhgsS9M2fOpPXr11Pp0qXZBMe4oKFibHoTWwgy8mRfCDLy1lRm5CcCW7ZsocKFCzM5Xrx4kaZNm8YaLJxB0gQBICAEKXLgWAR+\/PFHNuGxzwhvNjTdypUrOxYPmfifERCCFKkQBAQBQcALAkKQIhqCgCAgCAhBigwIAoKAIBAYAqJBBoaXXC0ICAIOQkAI0kGLLVMVBASBwBAQggwML7laEBAEHISAEKSDFlumKggIAoEhIAQZGF5ytSAgCDgIAdsRJM7j\/u9\/\/+PjXEhBhZYvXz4+L\/v8888\/lKzAQeskUxUEBAEFCNiGIJHYYO\/evZyBpVatWpwVJVu2bAwJSBMJBNauXcvnZ\/G9NEFAEBAEzEbANgR5\/fp1ztdXvnx5rynukcIf6abKlCljNi7SvyAgCAgC9jqLDQI8ceIEp7NHyqpt27bRDz\/8wBlhateuzedlpQkCgoAgYBUCttEgkf9v9uzZtG7dOk6xD1IESaLOB\/Ykc+fOzRXskLJKmiAgCAgCViBgG4K8fPkyvfPOO\/Tmm29yMSVkWXn77bc5oSnM73fffZfriCCvoDRBQBAQBKxAwDYEiRrKSJyKokdwyujrfeBnVKVDFmfU\/5AmCAgCgoAVCAhBWoGyPEMQEATCEgFbESRKfiLtPvYjUau4Zs2aHPeIcqDwcKO0pmiQYSlnMmhBICwRsA1BovbH\/v37uTi7pwYPNoq7iyc7LOVMBi0IhCUCtiFIhPhcvXqV6xN7avBeo9BS2rRpwxJoGbQgIAiEHwK2IUh4sUeOHEmnTp2iX3\/9lYsooYA82qVLl6h58+b0+uuvc+U5aYKAIBDeCNy9kEi3DsRR1pqdbD0R2xCkhhJM7Llz51KLFi2oUKFC\/GuQ5urVq6lXr16Gm9jFixe39QLJ4ASBSELg6azJVC\/3Naqf+zqdux1Fr333uM\/pobiaqmY7gkS4D85jo8D8X\/7yF8blxo0bNGPGDOrZs2fAThqY7P\/+97\/5fuxfIlSoSZMmLlMdBGn1Aqh4pioBk+eag0A4yRC0xWtfLKZrcYsInzOWq0lZa3b0S3tUPU\/bEeTt27dpwoQJVKxYMWrQoAFLl3bkcMiQIQFrkCgKj\/hKxFViH3P06NHUvn17evrpp7lvFQvw008\/8fykCQLBImB3GdJM6Gtxiyn5QBxF5S3KhJi1Vkf+7G9T8X7qx2Y7gtQ0xhUrVrDmB5Mbxw27dOniyu7jL7i4buXKlez4ad26Nd+GftG0n1UsgN2FOxB85Vo1CNhVhkCGIEVoiyDCTA+0RWiNwTQV76ctCRJEeO3aNcqTJ0+qOMJ54+safQeTJ0\/mYvBIdoEWHx9PO3bsoMGDB4sGGYzEOvCeyZsSedYnk265Zv9zUrLr88nLf3zGL6NL5KDZbczNOGUnggzFhPYlTkKQDxBCcDgSVSAnJJLjImAcQeJoOGp48OBBzgdZsWJFeuWVV3zh6voeBBkdHU3Vq1f3SJC5Bnzhd19yofMQKJgt6qFJF8ia8rP+9wUf\/E67cNfpZEo4nUKazxXKSKPr5nnoeiNQhOPyscceM6Kr4Pq4fIro+Nf0e8K\/iI7vJMr5GFGlVylNpVdTPhvQ6tSpw71Y7SPQD912JvaFCxfYLI6NjSX8lURDZp+mTZtyqE+gJ2ncTWr8DGdNs2bNuG8Vf6Hs9NffADmWLrwgsPzbc7T8m7MUf\/wKDa5fjAbX93\/vzReoKmQImuLd84l\/MqHzxiz0Ndygv1fxftqaIING0suNcNIsWbKEhg0bxleMHz+eOnTo4Eq6q2IBVAi30bhKf\/4jABN98qafqHCujLR3+PP+35jKlVbKkN6ExpCiHoXDxT8vdKiTVfF+OoogtTAfhA7hc0xMDHvHtbySKhbASuEOVUDlfmMQOJmUTH2WH2Jtcl3vZ6h6yRwhdWy2DBnlhQ5pkoosPEcRpK8FEoL0hZB8byQCmjYZqsltBkFqpJh8YLthXuhQsVPxfgpB6hBQsQBmCHeogij3W4dA\/LEr1GTOHqpeIgeti3kmqAcbKUMqTWhfk1fxftqeIOG1Gj58OHuvERwO73b\/\/v05u7jRTcUCGCncRuMh\/VmDgGZyI0RoVusyAZvcocqQXUxoX2ireD9tTZCIh0RoTt26ddmbPWLECDpw4ADnh\/R1kmbfvn18CgexkjjHjVMzOLFit6OGoQq3L6GS78MHgSaz9wS1LxmKDF2Y3dk2JrSvlRKCdENIK73w2muv0ZQpU2jUqFEcD+mr5AKCzOGphhOmdOnStH37dtq8eTOfmFm4cKEcNfQlifK9MgS0fck2lfP7HWAeCEFqJvTlT0e7TreYGZpjJJBCkG5oQoOcNWsWa5ALFixggjx37hwtWrSI06F5S3d28uRJ+vTTT6l3795MqKdPn+aiX1WqVOHqiHLU0Eixlb6MRiDQfUlfBBkuJrQvHIUgPSCEPUhog99\/\/z0HhiOwG6RZqlQpX3jy9ziVs3TpUi7TkDFjRqpWrZocNfQLOblIJQKB7Et6IkhvXuhw0RY9YS8E6UUiEbMIs\/nevXseM4nDFO\/atSth3xH1srWz1dAkx40bxw4dlJCdN29eqkcNtXyQW7dutezdUH5MzLKZyoOCQWD0lou0\/tB16lElB3Wv6jle8iEZunyKft\/1L6LPZ6Qc8ytRldK0nBrMo211jxw11C2HPqO4p1XCudOxY8d6rYutBYRDcwRZIp0ZyjPIUUNbybwMxk8EcEwxZvkhr6FA0CBzfLuEbh2MeyidWM6Wo\/x8QnhcJhrkg3XSatJAY0RYD0xrJJlAgwcbpnKjRo1cJ2Dclxf7lCBQmOZaJnJcI0cNw+NFkFH+GQGY3IiXREMoULXsV7hMgXuOxUgjRT0SQpBucgFNEmZxnz59Hsoojj3I7t27e9Ugv\/32W\/6+cOHCrmzh0DqRKBfpzeSooVBQOCIAkpw3dRLlPbuTGt\/Y5PJCXy9dj4rVbROOUwpozEKQbnBdv36dNUEU6NKcMthvQZgPgsezZcsWEMC+LlaxAL48kL7GLN9HNgLesuaUPt6BtFAgp8iQivdTL122S3eGwUEbRHhP+vTpeayoSYMsPFWrVjX8zVCxAE4RbsMXK8I71GfjxlSRkRvms1b5TwsFQlag2Y3zUPTTT0Q4ImrSEdqeIDFAhOrgRAz2JpFBPEOGDKYIgxCkKbBKp34g4B6Wo5Fi4TkpeVA9NX1WoKR3a\/nxlPC+RMX7aWuC9ObN9uXF1k\/q8OHDNGnSJJo2bRrvWcpRw\/B+SSJl9CBENFT403ufg8mvOHTFHvrwmysukztSMHKfhxCkGyKaNxthO2i\/\/vorVzWMioqidu3aUbp06VKVBY1gkZl8zpw59Msvv0hVw0h9e2w+L01DvHv+hIsQNS0RxaxCCeDGNs3peznZyw2TGzkm8X+kNSFIP1ZUOxmDMgmpOWlwHbzVRYoUoS1btvA+5ueffy5VDf3AWC4JHgHsHd55oB0ilyI+43da06r7hUKI7qPT9rH1p2+MylYePBLG3ykE6QemOFe9ePFiDv1JjSBRsRDm9csvv8znsEGQqIktVQ39ANkhl1xeNcbrTDUT2P0Cjfzwe9Rk4f8fEKL+WhAhzOX0eYtSxnI1XM4VM6B1d\/QZlYjXjLGG0qcQpBt6+iOE2lco2uVeKsH9qGHHjh3p448\/pn79+tGdO3c4\/lEjyNSqGv7fyyme8vz58oeyjgHde+\/eXUqX7uFqeQF1IBebh0AuDxX5chb643kPKval+dsb5o3Bj549HVdFJcXua85xBcUPm+U3vJKiH8My9BI5augBTuxB3rx5k4PEtbox2I9EqA+y8uD4oKe2c+dOatu27UNfFSxYkFq1asXnsvXZfKSqoaFyLJ0pQMBbqFikmdyiQeqECxnEcWQQVQgRKK7Vxb569Sqbykg+4U\/ZV2iXmgYJJ41UNVTwBssjTUXAVyxtpJjcQpA6MUKAOJwqCQkJHBSuBYrjknLlynHKMnizfTU9QWphPnLU0Bdq8n04IeCLIDEXfWB5uHq5hSDdpBIJcxMTE6lEiRJ+kWGoQq1iAfwR7lDnJfdHNgL+ylC4m9wq3k+95NjmqCHiF1GDBh5o1JXBJrS+BRIoHsiroWIB\/BXuQOYh1zoLgUBlKFxNbhXvpy0JUpV4q1iAQIVbFTbyXPsiEIwMhaPJreL9tD1BYg9x48aNdP78eddYs2fPTi1btvRakwYX4vQMsokjB2SBAgU4zEeqGtr3JZeRBY9AMASJp4XbWW4hSDcZQbozJL1FRnB9DRqE5lSoUIHr03hqOEWDKoiVKlWiv\/3tb\/Tll18SQn8aNmzIxb\/g1UbYEErBtm\/fnvtHU7EAwQp38K+T3BlpCIQqQ8FUUlSBoYr309YapFb2FQHfWpiPPwtz9uxZTk6Byoc4bQPCRDwlNFHEUUpVQ39QlGvCBYFQCRLzDAeTWwjSTSJBbIhbRCXCsmXL+i2vR48e5cqHMMVRgOvZZ5+lIUOG0CeffCJHDf1GUS4MFwSMIEh3kxuhQNVLei4SpgoXIUg35OHNRtEt1KF5\/PHHXWev3b3Y7kcNESOJQPK5c+cysW7fvp1Wr15N+fLlo5o1a1L16tX5STivjRIMWhVEqWqoSvTluaEgYHRlzHlfX+H0aY3LZKHRdfOEMjTD7pWjhoZBSQQNEgktsH8J01wLFse+JY4tylFDA8GWrpQjYJQGqZ+IHU1u0SDdRA1OmlWrVhGOF+obTtUg6UT58uU9BpBrtWxQK7t06dJctmHt2rV8FnvZsmVMnGgo3dChQwcqU6YM\/6xiAcwQbuVvrAzAUgTMkiG9l9sOJreK91O\/kLYJFNcGhZM0yMqDFGf16tUjlIFFRnCUcoWzBeVfkfbMU\/vxxx\/ZW41wH5zEwR4kElbgfjlqaOn7Kw8zGQGzCFIbtl283EKQboKEDOLYR0R6syxZsvC3+B0cMFpKM3iqjWoqFsBs4TYKG+nHvghYIUN2MLlVvJ+21iBBhtOnT+e8jkg0gQbHzTvvvEM9e\/akhQsXciiPUU3FAlgh3EbhI\/3YEwGrZEi1ya3i\/bQ1QWJwa9asoeXLl3OoD\/I\/7t27lxo1asT1aM6cOePVxA5GlFUsgFXCHQweck94IGC1DKkyuVW8n7YnSAwQ+4jwTCOBLvYfUfoVn+GR9pbyDI4ZJLrAPuYTTzxBw4cP5\/ukqmF4vPQySv8RsJogMTK9yT2rdRlLYiaFID3IBBYfpIagcTQQ3rFjx2jq1Kkus9v9NpjmcNAMGDCAHTPwYB8\/fpwaNGggVQ39f+\/kyjBBQAVBAhq9yT27TRkuO2tmE4J0QxdZxRGKU6VKFdq1axefiPn555+pYsWK9MILL3hdCxDkW2+9RX379uUz3FqoEE7W2O2ooepFN1OgpW9rEFAtQ1aZ3KrnabswH\/1ZbGQXB0AwsVFyoXfv3qlm80GpVxAkNE4ko4Dn25+jhtaItDxFEIgsBO7mKU3Xqw+itDcvUrbNg02bHML3VDXbESQCvmFKd+vWjTOLI+EEnDVIRPHGG2+4atK4HzVs164djR07lj3cIFQcVcRxQwSY16hRw+tRQ1XAy3MFgUhAQDO518U8EwnT+dMcbEeQGCGcLdu2bSOQHsxm7CXiqGCvXr28OmiQ2mzDhg1MkHDiwMkDwoRpntpRw4hcVZmUICAIGIKALQlSPzMtbVnWrFldZWA9zRza5qRJkzjfY\/78+Vl7xL9XXnmFli5d6vWooSEoSieCgCAQkQjYhiARDA7tz70WjYa6r5o0cMQgSe57773He5DYuxw6dKjPo4YRuaoyKUFAEDAEAVsRJPYY79y5w6UV4MXWJ8xFNnB4pBE4Lk0QEAQEASsQsA1BYrIIBEeSCsRAxsbGcomF5s2bc6hPhgwZrMBDniEICAKCgAsBWxGkfl1gMuNY4bp162j9+vXsyUYQuJbAQtZQEBAEBAGzEbAtQeq1SSFIs8VA+hcEBAFPCCgnSMQ93r59m8eG8BwkyhUTW4RVEBAE7ICAEoKEdoi6MDjpcvHiRVcVQpwvhWmNsqyIgUT8otbESWMHcZExCALOQsBygsRZa2QMR9hO3bp1XXuKWpjPiRMnWIu8ceMGF+3Smq8wH2ctm8xWEBAErEDAcoKE9oh\/3lKWaZPGNRLSY4UIyDMEAUHAGwKWE6Q2EG+B4ag5g1o0r776qqvkqyyfICAImI\/AtbhFHh9y9\/wJrw+\/eyHR9IHljVlo+jNsR5AoxoUjgI888gihpjX2Hj\/77DMeJ8zphIQEGjFihOnAaHWxTX+QPEAQMAmB+rmvc88Vst7i\/\/NnSMmjmu+RlP+1n4N5\/LnbUT5v++U339f47CSVC5r9N2VeKppSDXLevHlcPkFzxmDfEY6b1157jWtcI1GF2U1FvjkVzzQbR+nfPASSD8TRnQuJlHxgO\/9\/93wiuWtuUXmL8gCiHi1K6bXPeYtS1KNFKGvNTuYNzuSeVb8ryghSn9YMGiMaTtHMmDGD8z6CIEeNGmUy\/FIX23SA5QEBIwBTVyNDkCMT3wPSy1SuJn++kuYvVKzFmwH3HW43OJYgsVBIawYSRM5GOGSuXbvG2XjOnTvHJneLFi0CWs+VK1dyggqtTZw4kc91S02agGCUiy1GACR468B2unUwjvBZT4YZy9XwqAGqKrlgMTScdMaxCXNBggjrwWIXKVKE9yOR7BYpznx5uT0t1AcffEB16tThgl1aO3TokNSksVqq5Xk+EQARXotbTJpjBKQI7TBrzY6UsVxNn\/cLQfqEyJALlJnYGD3Ku+IYIRw2gwYN4lRlnTp1cmX\/DmSGOI0zc+ZMatu2LRUoUMB1K7RKu9WkcYpwB7J+TrgW+4bXvkghRXwGKWJ\/MGfLwLeSnCJDjtUgUWRr+vTpfGoG+44wtUFycNKALLNlyxbQOwMHD6oaIp8kysUiVAj1abCXWblyZfaUo8XHx\/MpnsGDU2poqFgApwh3QAsYwReDEKEtauZz4Tk\/hTxbp8iQivdTvzjKNEjEQcKLjSOFqEEDgkRKM3zW157xV5Lg9EGRL5jYyD6+adMm+uqrryhz5syp1qTRwny2bt3q76NCvg4krjmmQu5MOrAnApdP0e+7\/kWEf2glqlKallMNG6sTZAjvMpoj9yBh9i5YsIBw9HD\/\/v1cbwYkhUDxnj17BrUHqZc+f2vSqPgL5ZS\/\/oaxQZh0BLP51gE4WrazGR2KCe1ryk6RIRXvpy00SAwCpRE2b97Me5HJycnUsGFDPkGjzyTuS1C07+GMgZMGhbpgnqOqITRIXzVpVCyAU4Tb37UL9+v0e4uYSyDOlmDn7hQZUvF+KiVInLFGMgpokJ5asFl70B+qGmJfE6Y6PNnDhw+nPHnycJgPyBPXxMTEUIMGDVwFwFQsgFOEO9iXPxzu07RFo\/cW\/Z27U2RIxfuplCD1Z7DhqEG6s6JFU04BXLp0iUssvP7665ZlDlexAE4Rbn9f9nC67vKqMQ\/FK0JbVHFW2CkypOL9VEqQ2sNhXs+dO5eDwRH7iIaN59WrV\/N+JGIijWjQGiVQ3AgkndmHdqrFPV5RBSnqV0AI0hp5VObFTkpKYrMXHmv9WWyE\/MBJkytXLkMQkEBxQ2CM+E40AvR0xE873pepXA2\/gritAEsI0gqUiZQRJGIeJ0yYQMWKFeM9QbRt27bRDz\/8QEOGDDFMg5RAcWsEye5P4eDs8ydcSR48JX2A11lL9qBaQ\/SFpxCkL4SM+V4ZQWL4CO5esWIFm8AwuRHc3aVLl4CDxFODYvLkyRIoboysGNIL9vBSa77yC4LY9A2ZbbSW2r3a+WZcC40Qzds5Z0MmanInQpAmA\/yge8sJEkSIpBTwLqfW4LzxdY0\/EIEgo6OjXccX3U\/S\/NgijT\/dyDUhIHAmKr\/fd59Nl\/q1Z6LyPdTX2Qd9J5Tpwb8vmC0lN2HBrH\/kKOxeNYffzw+XCyVQ3JqVspwgkYgCta737t3LmXZKlCjhintE0PjBgwdp7dq1VLFiRY5hDLVBQ0Vr3bo1\/4+f4QBq1qwZ\/6zCS+aUv\/6hrp2v+ydv+kN7PJmUklT156Rk120nLyfTSd3P+v4K58pIhXNmpOiSOfnX0SVyUPWS4UOkTpEhFe+nXk4sJ0jt4Tjpgv3B2NhYzuaD9uSTT1LTpk051MdIJ82SJUto2LBh\/Izx48dThw4dqEyZMkKQvhgoQr\/XiBWkCkKNP37FNVMQJ8iycK5MNLh+SviZHZsQpDWroowgrZkecXC4BIpbhXZ4Pyf+2BXacfwK7Th22UWaIMw2lQvYjiyFIK2RtYgnSF8wqlDhnSLcvrC3+\/cwz5d\/e46Wf3uWTXWQ5d7hz9ti2E6RIRXvpy1MbDOkTDKKm4Gq9AkENLKcvOknWxClEKQ1chlRGqRkFLdGaJz8FBAlSBKaZZvK+Wl2m5S9bKubEKQ1iCslSJzLRsqzxMREPn+NUy8vv\/xyUNl8JKO4NQIjT0lBAPuVTebsYW1yVusylnvAhSCtkURlBIlwn2nTpnGYz\/bt27nEKxLeIoFFMPkgJaO4NQIjT\/kDAWiTfZYfIoQTWb03KQRpjSQqI0jtLDZCbrSM4ohP9PcsNu7v2rUr7du3j3r06MFpzCSjuDVCI095GIHua85RwulkWt\/xMVegutkYSaC42Qin9K+MIFEiYc6cORzzqNWkQaA4PiOPY6A1adzhkozi1giQPCUFgSaz9\/D\/62KesQQS0SAtgVkdQfI+Tnw8m9lXrlyh5557jutkI5C7evXqAc9eMooHDJncYCACMLd5TzJnRktIUgjSwMVLpStlGqQ2JmiSqEKIM9olS5YM+vy1ZBS3RmDkKakjkGvAF7Su9zOmO22EIK2RRKUECY0R+4Yo\/YqjgKheNmDAAEPOYPsLn4pAVKcIt79rEEnX4RgjwoDgtIGH26zmFBlS8X7q10wZQUJznDJlCnXv3p0ziR8+fJiaNGlC77\/\/Pv3jH\/\/wuQcJLzhqaKPQF+rPoIFgQbQo3fDCCy9wMl6UgJWM4ma9ptKvJwSs2I8UgrRG9pQRJLzQH330EfXr148++eQTqlSpEpUqVcovL\/bOnTtp+fLlXC4WtbVBkCDc0aNHU6dOnahcuXK0atUqLg4GosRzxowZw4W6cA001qeffpoRVvEXyinCbY0I2\/MpZpvaTpEhFe+nLTRIBHYjV+PxHRNGAAAMXUlEQVTNmzfp3LlzNHHiRHbS7N6922dGcWibp0+fZhIcOHAgEyT2MRcuXMgaJEo4HDlyhIPQQZbp06d\/KN0ZANDSn6lYAKcItz2py5pRaaZ20ru1THmgU2RIxftpC4LEIBDc\/d1331G+fPkob9689MUXX1CNGjV8mte4FyFB8Hh37NiRCRJa5caNG2nEiBEUFRVFWphP7ty56cUXX6TatWvzvN0T5qpYAKcItynMEEadVhz3P06dZsZxRKfIkIr30zYEiX1EZBfXamSD9JDQFmUXcuZMSWSKBk3zww8\/ZLP4448\/5lyRnggSNW2GDh3KprSeIOvWres1ozgWAG3r1q2WvXpOCPK1DEybP+i5mYk075X89FwhYx02TpChOnXq8OrCt6CqKduDBDkiucR\/\/vMfPl6YI0cONrc7d+5Mbdu2ZS0wteZOkJpJDQ1SM7GXLl3K2iX6koziqkTM2c+Fw8aMo4iiQVojV8oIEokqZs6cyWE90PyQ4Rt7hQj7AUkGSpBw0owdO5aPH8LZg\/1JxFZWrlyZJKO4NcIkT\/GMgBmmthCkNdJmC4JEsgrsFcKERphPt27dfJZccNcgNVUcXmo4fWrWrMkecmiTklHcGmGSp3hHwGivthCkNdKmjCCx74h9RThqsEc4f\/58Klu2LNfFhvMlS5YsliCgYhPYKcJtyQKGyUM0r7ZRp2ycIkMq3k+9SCkjSAzi\/v37vO+YOXNmSkhI4HyQL730Enu0rWoqFsApwm3VGobTc2BuG5E\/0ikypOL9tA1BwlEDjRHJKrSGlGcVKlTg0qypNU8naaTkQjhRhTPHapTTRgjSGvlRpkHCgQKnCjzYWqgNppw9e3aul52aie3pJA3ulZIL1giNPCU0BECSKDU7uH6xoKslCkGGtgb+3q2MILWEuTgvDUdKIM3TSRopuRAIgnKtagS0PcnqJXLQrDZlAk5sIQRpzQoqI0hokHPnzqUWLVpQoUKFAp6tuxc7lJILeLgEige8BHJDiAic+fUujd5ykbOR96iSgxqVyeJ3RnIJFA8RfD9vt5wgEa+oJZK4ePEixcXFUa1atTjMx5uJ7c9JGvQbbMkFqyP1nfLX308ZdPxlqJCIFGla7e02lQv4NL2dIkOOc9LAc40sO9rxQve3A8cEsQ+ZNm3aVF8cT3GQ+huk5ILjeScsAdBMbwwe+SS9kaUQpDXLa7kGiWkhkBv1aJCoAuUV3M9e+zN1d4KUkgv+oCbXhAsCKCu74\/gVWv7tWdYsNcJESYfokjnpyuXLNLG1NfVvVGLmOA0Se49vv\/0253+sUqUKn3KBqT1o0CCfxwv1C+VOkOFUckH1oqsUeHl24AiAIPEPhHky6Rb9nJTMXnB907KXg0Aff5DJvHCuTHzJ4PpFA3+oTe5Q\/a5YrkG6e6\/xMzKLoy52qJUMg1lTfYhRMPfLPYKAagRuF46m+5nz8DDuZ87t8XOwY0x786LPW9PevOTzmlAuOL+kZyi3h3SvEoLUMolnypSJQJDI9j1q1Cif569DmqncLAgIAqkiAGeRe9PM+9RuhFZrZjMjn6a\/4xWC9BcpuU4QEAQch4ASgkRKsn379nkEW58U1ymrAY879mXPnj3LBcv++te\/OmXqMk8DEYBn+9133+UIkNKlS3PqP19Hdg18fER2ZTlBRiSKIU4qNjaWChcuzEKNujrIh2lVNqMQhy632wiB77\/\/nu7cuUPPPBP53m2rYBeCtArpVJ6DMhINGjSgAgUKcEkJ7bMNhiZDCCMEvv76a9YekSRamjEICEEag2PQvdy7d4+rL7766qvspEKt7\/r167tqfQfdsdzoOATWr19P69at40MYzZo14z+0OHghLXgEhCCDx87nnQhe37JlC\/Xt25evRYq2xYsX07JlyzgHJurn4K89zOrGjRtzHkyQpfbZ5wPkAkcg4K8cnTlzhrJmzcqlS5CZv0OHDmyVSAseASHI4LHzeieOUuK8+Zo1a7j0w+DBg\/lalJbAP8R8IjgemdPxHeqBYw8SBcZQaKxTp06EEChpzkYgUDlKTEzksiWQHS0RTP78+Z0NYoizF4IMEUBPtyPH5cGDB+nAgQNMhBpBIulGdHQ0H6+EGTRx4kSu2Y0iY\/Bi4wimeLFNWJAw7TJQOULaQBy6SE5Opvbt27OZLSZ2aIsvBBkafqnejcS+0BhBkDgaiQTB7dq1o6eeeorvQw1wCHCrVq1MHIV0He4IiBypW0EhSBOxdxfscePGUceOHVljFII0EfgI61rkSN2CCkGaiL1esPUmtWZiw+SuU6eOhGWYuAaR0LXIkbpVFII0EXu9YGtOml27dhHKTGBvEgQJ81s20k1chAjoWuRI3SIKQZqIvbtga2E+qL6IDXV4syWo18QFiJCuRY7ULaQQpDrs5cmCgCBgcwSEIG2+QDI8QUAQUIeAEKQ67OXJgoAgYHMEhCBtvkAyPEFAEFCHgBCkOuzlyYKAIGBzBIQgbb5AMjxBQBBQh4AQpDrs5cmCgCBgcwSEIG2+QDI8QUAQUIeAEKQ67OXJgoAgYHMEhCBtvkAyPEFAEFCHgBCkOuwj5sk4U75582bKli2ba04oG9Gzp7qC70aAiyJYx44d4xIYQ4YMoTZt2lC1atVcXWPeRYsW9Ziu7vTp07R161Zq27YtRUVFGTEc6UMBAkKQCkCPtEeCKGrUqPEQeejniExG+IeCUuHSkL9zzpw5XLYAFSYDJUjME\/k+UalSqgyGy6r\/eZxCkOG7drYZuTeCRFIOZFZHKYD+\/ftT9uzZuczE+fPnWfMC6RQqVIhLTowaNYprOCN5B+qqILHwmDFj+PcoZqYlbBg0aBBt2LCB5s+fzzV+UJgKJSpQZx3lKtDH\/v37qVKlSpwMJGPGjLRkyRImq99++4169+7NtX\/QH\/qCdofv0UCGWktISKDdu3dTt27dONlxagR5+fJl2rRpE9+KLE1VqlShCRMm0MmTJ+nTTz+lN998kzJkyGCb9ZKB+I+AEKT\/WMmVXhAAQX744YeubwsWLMiFyEAw8fHxXFoC9ZqnT5\/OBIX0bjBfUbOnS5cuNG3aNE77li9fPiYrlJ4AMXkiyHr16nGW9j59+lC6dOn4+jx58lDu3Lm5n5kzZ\/LPSE7csGFDunnzpiur++3bt7m0BUxljA8ECu0Qmd7xPNQE0hqqS5YtW5Zq167tIkhUDXRvmJuWER7EP2zYME5nh6zxKJmAvgcOHCjFs8L07RGCDNOFs9OwU9MgMU4QCAixc+fOdOnSJdfQUWAqJiaGkCMTBInyE0eOHGFtzJsGCe0P5KdvPXr0YBNfK2+B76C9FilShHbs2MEmLogOTTP3YT6XL1+eHn30UdYuQZbQPrWmn5MvDRLzg3YKMqxYsSI1b96c54L7oDEji7yefO20djKW1BEQghQJCRkBfwhyz549bAJDg0PVvfv377N2B+Lctm0bDR06lEkFP6NULsxdTxok8miCJDWtDVohaovDxPZEkPidVigNE4VWh7KoMP3xHPQFzRXapr4FQpAtW7ak1atXExwz0Gw1p4wQZMiipbwDIUjlSxD+A\/CHIC9cuMDmJ8xN1OSJi4tjMmzUqBFNnTqVRo4c+ZCJ3atXL5d5CpMc9cKxvweyi42NZfIE0cEUrlq1KpOSJ4K8cuUKV5d0z+IO03r06NF0\/fp1\/t89qzueh31SvYntzYuNfVSY7JMmTWLC1Rr6hgkO0pT61OEp50KQ4bluthq1PwSJAUPLg3l848YNNjmHDx\/O+4UIhwG5wDmjOWlAKp999hnXd4aTpnjx4rzPCKJDzfFFixYxBq1bt2ZtE2a6J4KEGQ9zGtoiNNQBAwZwHSB81kj3n\/\/855\/Ko8JJ880333CoEsqoenPSYL8Vz8Z+K+aC9thjjzG5o671smXL+I+C3ny31eLJYFJFQAhSBMRWCBw9epT3IEGQZjZ4wOHUgdPHUxgOzGM4leBECrZmEPY2oYXqYyfNnJP0bTwCQpDGYyo9hoCAFQSZlJTEWh08zSBib9rd4cOH2WmEbQBonIE07EeC6KHdSqB4IMjZ61ohSHuth4xGEBAEbISAEKSNFkOGIggIAvZCQAjSXushoxEEBAEbISAEaaPFkKEIAoKAvRAQgrTXeshoBAFBwEYICEHaaDFkKIKAIGAvBIQg7bUeMhpBQBCwEQJCkDZaDBmKICAI2AsBIUh7rYeMRhAQBGyEgBCkjRZDhiIICAL2QuD\/AVGrlVPwhQGXAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:7fcd2cd7]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: GM355L6-250kW\nIM Normalization Voltage Factor: 326.6 V | IM Normalization Current Factor: 610.9 A\nRotor Resistance: 0.00434 Ohm\nMagnetization Inductance: 0.00533 H\n---------------------------\n","truncated":false}}
%---
%[output:79465b8b]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00756 H\nPer-System Quadrature Axis Inductance: 0.00624 H\n---------------------------\n","truncated":false}}
%---
%[output:8b3aa18c]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUgAAADGCAYAAACw2RkvAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQnYjlX6v8keso59y3z2MhJpEhqkMkllaYws0SWFVltUQmStsQ0VSYk2olQUI6n4VyM02fVR2UWMZRT9r98x553zPd\/7vO953vc8q\/u5Lhe+7zxn+d33\/Xvu+yz3yfHbb7\/9RvwwAowAI8AIZEMgBxMkawUjwAgwAvERYIJkzWAEGAFGwAYBJkhWDUaAEWAEmCBZBxgBRoARcIYAe5DO8HK19Jo1a6hTp040evRo6tixo\/G2tm3bRt27d6ebb76ZBg4caLx+tcJTp07RoEGDaPfu3TRz5kwqVqyYq+3Fq\/y1116jyZMn04svvkgZGRmet59qg1JOeN9J34OAuTpmOY7x48dTo0aNUoXD1\/eYIH2FP2vjXhPkTz\/9RD169KCKFSvS008\/Tfnz5zeGhtVYUbFbbcXrtBPjdIqDm3JScZsyZYqQie7HJWgECbmMGTOG3nnnHUdEb0wJDVR0wRAkBDVjxowYZK+++qr4qkmlwi9uueUW6tmzpyhj9eLU9+vWrRvziqSxPPbYY\/T1118LZYCHlohw4NkMHjxYtFO2bNmY8lgND23iZ3fffTf16dOHZJ+l8e\/ZsydLX+XP69evL9rHo3pxhw8fjnmQqBOEtX79elEuXp+t\/YlngMlwhQc5btw46t+\/f7a2UJ\/ah169emXxbGXdwAjlRowYkUUusn9SqBIfaZj4vfReZd8hHzxShpK4rTjY9c3aptSTRH2x2qkkZNmm7AvIEPKSfVR1Q63D+r7ETZWP1BmrLltxUOUuxwBifv7554V9INJQ9Q0RzvHjx7NEBnb6iLa9jFoM8GG2Ki4IglQJSUUABgXlVJXS+nsoiZUEVEKBkkNprI9dmGw1JNVYt27dmiXEjkc+1apVy0IqKjkUL15cEKApgpSGKA3F6mktXrw4RvR2uNoR5OOPP07Dhw+PkYF8X+JmJzM7QlI\/NsDB6q3ayXDAgAHi46MSZKK+VapUKYu80R\/rz6wfPhUbK0HJ30EPQUxjx45NSJB276Mfbdq0iavLKtHGw0ESrFU38XPrh1TtLz4+8qMrP9bWj5XUGfzcr6mWdIgz8gQpBXTw4MGYpyaNDwrQr18\/oVRfffVV3N9LBZFhqPTK8JUHweIBQcovsSTMeB6ZqtzS25EKq9YlSUD+TiVbte\/4ukulxlhuu+02bYLEuzqhpfRiodwqgbds2VKQUDJc5Ryk9NQkjtJDi\/d\/laCsHzEVGzV0U3GROPTt21fM5Vo9X+mpSZlbCTVR3+CZSxnr9MU61yvlJfXDqk\/yg203d2t9\/4cffojJPBFuKo6ybomD\/L+Ubzyv0qrf0uv98MMPxUdS6rPd+FT7SoewvH438gRpDTuhFOpXTX61AbwMi9V3EHL37t2brF9IlI9HavHak0JN9jWNF2Jb529UQrVOfDsJsXUJUl3o+Oc\/\/xlb9MCYVG\/VDlc7glTnO1VvEYYovTrV61CxsfOUUB4fiqZNm2Zb7LKbCsA7iT4U1r5ZCTJZX6wEKetTP3rqR05+sO0IMt77Ur8STYGo0w8or3qLkuysEQzKWduztoFQXJ26kn2xfgCYIL2mds32TBGk3cqvldSiRpByPPAWMccqPySq5yI\/LPE+PHYEafWc5NQBPEpdgnRKIsnmIe2iBLVvdgSpu1rvN0GqUYkkd6sHGS9ikT+LR5CJFmGSOQWaZuxbsch7kLohtgyZ1TlHdQ7Gbg7FCUFK5ZJf0\/Lly8fmjOxCbKvyWQ0sXmhZsmRJMd8j54es\/5dkrxNiq4tYMKQ77rhDhK26uNoRpDVMVUMzdV4wUYgNz8XqGanekTTqeOOUuOH9RASIsVrDxnghtl1frJZtKsS2zhtCpjLEVslajTjkHLWUv\/z4Sf1I5EEmC7Ht5tx5kcY3btdv2I1FGruwJJEHqYYsau+tymc3t4V3rCuG+JmchJdzaXLBQbYh+6quYqshNsonWnmX+FlXVXVwtRKkbMtu6kL2w24BKNEijZz2kEQg5yDx83iLE\/FwQ\/vJ+qYuzNkt0si+WKdBEi3S4KNmnRe0bvGxrmBLGcebY8S7KkHaLfAlCrHt2rPqlHUKStVhdTeBvtUGo2TkPUgJs852FLutEVYDS7Q1JxlBWuuKt2UoEUFa54+shqjOLcmtR5KkrASpGmsigkw0Jh1cVcNXt0FNmjRJzF8Bz6lTp9KsWbNii2Wqdw2MpGzizd1JGSfyGOMRk3Wrl27f0B7mX0EKdoSdaLO\/3TYfEJrOXka7bTU6c5DqRw3j\/\/jjj4UM4kUwEtdEOoU+W\/sjvVsne1GDQYfZe+EJQf773\/+mJ554gtq3bx93R\/2KFSti+w\/RRZU03AZORyHd7gPXnxWBZKv9OqcyQATz588P5daSIOlDolVz04cLgjRu2RfXCRLpJt98800aMmQIzZkzJy5BQpHhMTRu3NhzjJggPYdcq8F44bvdxul4FUrvRQ2ztRrmQlkQsAuxrZv6owqb6wS5Y8cOeuGFF+js2bNin168r\/+zzz5LLVq0oDp16kQVZx4XI8AIhBABVwkS3hnmYjp06EBLliwR+9OsBPmf\/\/xHhN+YJ8Pq7qWXXipWdps0aUI5cuQIIaTcZUaAEYgKAq4S5IIFC+jAgQNi8y6OUMUjyGPHjtGECROobdu2Yu5x3bp1NHLkSBo1ahTVrFkzC84gT34YAUbgwkFg+fLlVKVKFd8G7BpBZmZmirOlmHssWrSo2G4QjyCtI8ecJbzOqlWrZkv5BYLcuXOnb2CZajgq4wAePBZTWmG2nqjIxe9xuEaQ8ZIyQAWsk7vfffedCK2xqz9PnjwkCRLeZOvWrbN5kEyQZg0p3dr8VuB0+6++z2MxiaaZuvyWiWsEaYXHzoPct2+fSKmEjCrIQgOyRMiNLQSVK1eOJEHio+Bn2GBGdc\/XwmMxiaa5uqIilwuSIOXWARAjFm02btwowmp4nbVr1xZHpho0aJBNW\/wGy5T6RkV5mSBNaYT5eqKiY37bvGcepAkV8BssE2NgUjGFovl6okIqUdIxv20+IUEeOXJEeHPI3KLzLFy4UKdYymX8BivljlteZEM0haTZelguZvE0UZvfNp+QIBEKY0\/irbfeSoUKFUo4XqRhv\/HGG01gYluH32CZGhwboikkzdbDcjGLp4na\/Lb5pB4ktupgAQVbdRI98DaTlUkXML\/BSrf\/8n02RFNImq2H5WIWTxO1+W3zCQkSSSawcHLVVVcl9SBNgJGsDr\/BStY\/3d+zIeoi5W05lou3eOu05rfNJ\/Ug77\/\/flq7dq3IF9iuXTu68sorKVeuXDpjM17Gb7BMDYgN0RSSZuthuZjF00Rtftt80lXsc+fO0a5du2jRokX0xhtviDF37txZbOxGdhUvz0v7DZYJgaMONkRTSJqth+ViFk8Ttflt80kJUh3kr7\/+Km62Q\/qyt956i2rUqCESmTZr1swTr9JvsEwInAnSFIrm62GCNI9pujX6bfOOCFIOFscBkU0ZF7nv37\/fs6SkfoOVrrDl+2yIppA0Ww\/LxSyeJmrz2+YdESRWqj\/66COR+BbpyXAfNLLwZGRkUM6cOU3gkbAOv8EyNUA2RFNImq2H5WIWTxO1+W3zSQkS+xtXrVpFr7zyirj289prr6U777xTLNaodxubACNZHX6Dlax\/ur9nQ9RFyttyLBdv8dZpzW+bT7pRHPchnz59mrp27UrNmzcnXBHp1+M3WKbGzYZoCkmz9bBczOJpoja\/bT4hQSLbN+YYcV+MFyF0MkD9BitZ\/3R\/z4aoi5S35Vgu3uKt05rfNp\/Ug3zyySfFlQjW+3mtg8OxxGRldABJVMZvsNLtv3yfDdEUkmbrYbmYxdNEbX7bfFKCfOCBB8R2nmTzjbh\/5tFHHzWBiW0dfoNlanBsiKaQNFsPy8UsniZq89vmk4bYGzZsIITaOo\/b17b6DZYOBjpl2BB1UPK+DMvFe8yTtei3zSddxU42AC9\/7zdYpsbKhmgKSbP1sFzM4mmiNr9tngnShBQd1sGG6BAwj4qzXDwC2kEzTJAhAstBVxMWZUM0haTZelguZvFMt7bV249Su0FTaN+bQ9OtKuX32YNMGbrUX2RDTB07N99kubiJrvO6532xj+6bt4l+mnid85cNvcEEaQhIJ9WwITpBy7uyLBfvsNZpKVQEieS5kyZNotdee01cFo8tPTiCeNddd7meSVyC6fd8hI5QdcqwIeqg5H0Zlov3mCdqMTQEiew906dPp4MHD9Jtt91GM2bMEASJS7qw\/xFJdb1IossEGSwFRm+YVIInk6jIZczSTBqz9Lvgh9jydkOQYt68eUmersH+yFGjRtHw4cM98SKZIINnjEyQwZMJE6Q5mWjNQdoR5NmzZ2n06NH02GOPMUE6kAmTigOwPCzKcvEQbI2mQuNByhAb92N36NCBnn\/+eRFWf\/DBB4Qs43379uUQW0PgsggbogOwPCzKcvEQbI2mQkOQGAvmGmfOnEmTJ0+mX375RRAi7qbBWe3ChQtrDDd5ESwEITFG+\/btqVGjRtle4BA7OYZel2BS8RpxvfaiIJdQEaQUCzxGJNG9+OKLKU+ePHrS0igFLxV33QwZMkRkLGeC1AAtAEWiYIjs2QdAkeJ0Yeii7TTt4++Dv0gDz+7111+nn3\/+OS6S1apVo4YNG6aVTHfHjh30wgsvEOY1sVLOBBlMpbX2igkymHKKglzaTF1Hq3ccDQdBTps2jRYvXkwtW7YUIfWhQ4do2bJldP3114uVbeyJxH01qTwI37HYg\/nNJUuWUNOmTZkgUwHSh3eiYIjsQfqgOBpNhoYgsZ1n\/PjxdMstt1CdOnViQ1u\/fj29++679Mgjj9C2bduy\/E5j\/LEiCxYsoAMHDlCvXr1o7NixCQkSLy1fvtxJ9YEri8UuZGmPwsNjCaYUwy4XXO9ytO1MAW7gjxoiW3i8zOLqzzGQVDKKZ2Zm0pQpU8TcY9GiRWnMmDHsQQbT5uL2ij3IYAor7HLZ\/dNp+sPIz8NBkPAgsRm8RYsW1KxZM8qRIwdhUQVhNby\/p556ijCHWLduXcfasmbNGnF9rPWBNzlw4MAsP+ZVbMfwuv5C2A1RBYjH4rq6aDeATD5tpq0LB0Gilzt37qTBgweLuUfMQeKmQ2wgR+hdqlQpsS\/yvffe0wbAriB7kGlD6GkFTCqewq3dWJjlAu8R5Ii\/c548RIemt9cet+mCWidpZKPY4gOixHwhtvlkZGRQwYIFxWZxrD5jsSbdhwkyXQS9fT\/MhmhFisfire7YtaZ6jwXWzaYf\/vGSbx3TJkiE1Nj\/CDKUD7b9PPvsszRs2DA+auhAhGyIDsDysCjLxUOwbZpS5x4rFstHR+ffT5nf\/J9vHdMmSMw1Dho0KAtBFipUiLp160b33nuvEe8xGQo8B5kMIe9\/z6TiPeY6LYZRLio5YoyL761HXa6\/QkStfj1aBImN4lhlvvvuu0U\/V65cKUgRm8cxH3nTTTd50n8mSE9gdtRIGA3RboA8FkeiN1YYxCjnHWWlr9x1Gd1Up4TIPRt4glS385w4cYJmzZpFAwYMoKNHj4otOli8wVyk24\/fYJkaHxuiKSTN1sNyMYunTm2Yb+wzf5MgSPl8MfgqqlqygPiv3zav7UFinvHOO++kihUrim09CLeRtAL\/xhagVPZA6gColvEbLKf9ZU\/FFGLe1MME6Q3OaEVdqZatYs4RYTX+lo\/fNq9FkOjs6tWrRXJcLMrgJAsSS5w5c4ZuuOEGsV+RM4rrKxcboj5WXpZkubiLNkhx\/IeZ9MravVkaAiGOaPN7qlu+UBZyDI0HKUeDrTx4sJL97bffin\/XqlXLkwWaIIBlSn3YEE0habYelotZPKWn2GfeJpF0wvqAGDtfVYY61C+djRhD5UFikeb999+nG2+8MctcI36OBBWtWrWi\/Pnzm0fXUqPf7rapAbIhmkLSbD0sl\/TxhJc4ZeVu2rz3RFxSRAsgxil31KTGvy+StEG\/bT5hiI0jhhs2bBCnZ5CjsUuXLoStPfLB4g0u7powYQLPQSYV9f8KsCE6AMvDoiwXZ2CDDM\/99hv1m7\/ZlgxljfHmF3VaCzRByrtosMwus4OoSXLz5ctH3bt3FynQcD7b7cdvsEyNjw3RFJJm62G5JMbzy13HaPi7O5KSofQSO9QvRZ2vKmsbPutIz2+b11qksQuxdQZosozfYJkaCxuiKSTN1sNyOb+6\/MvZ32jiR5n0\/U+ntckQknimfXXKfVFOrdBZV3J+23xCgjx37pzIIo5jhnYPPMdLLrmEcubMqTvmlMv5DVbKHbe8yIZoCkmz9VwocpF7Dues2UP\/993PWiSohsr4d6eGZeiOK+0XV0xJxm+bT0iQmGPs0aMHITGu3YMUZ7jMi\/dB6qvEhWKI+ogEo2SU5PLp+m1UoUIFmv35Hvoy82fafeT8aRXdR+5FvKZqEZr6l5riXXV\/om496ZYLNEGmOzjT7\/sNlqnxRMkQeSymtMJZPZLsXvp8D32RAgFaPcJrf1+U+l9fWfzYDyK0G73fNq81B4nOI8zeunUrffjhh3Tw4EFxZQC2\/Xh5dYDfYDlTYfvSTCqmkDRbT1DkIsnv2OlfafZne2jr\/hOOPUAVGRBexaL56NKSBejZDtV98wZTkZbfNq9NkPIkDS7pqlSpEn355ZciaQXyNzZu3DiVsTt+x2+wHHfY5oWgGKKJ8fBYnKEoye\/Qv8\/QrM\/20O7Dp9IiP9XjAwne26wC1SpTkL7\/\/nu6pm6Gs84FsLTfNq9FkFjFxllspDZTL+365ptvaPbs2eJ3nKxCX7uYVPSx8rJkunKR5PfBvw7RuxsOiq47nfuLN17pAVYsnp+6XV2WrqxUOKkXmO5YvMQ9UVuhIEidS7t4kUZfpaKivBhx1MeiLmy8\/fUB+mjTYWPEZ\/X+sDL8x6rnT5ekOw8YFbmEgiBxb\/Xjjz8u7q1u0KBBjAnYg9QnRbVkVJQ3zASpzvPN\/PRH2nHgJJ06fZq++lF\/pTeR9CXBIexF6q4OV5aickXOZ6lJl\/x0tC4qOhYKgoRAvvrqK3H\/9eWXX06VK1cWxw95DlJHVbOXiYryBo0g5VaUzMOnaNH6g7TcsLdn9fgqFMtHzWsUFyGvV8Snq3FR0bHQECQEg1AbpLhr1y4qXrw44XLvcuXK6cos7XJ+g5X2AP5bQVSU1wuClJ7ekZO\/iDRZW\/adMBriSpnCq0OWqktLFiTM9bW+rATVLnM+CbQXHp8p3ZL1REXH\/LZ5rUUaXNaFK12xWl22bFlPzl3HUxi\/wTKlxFFR3lQIUp3TW7XtCH2+86g40mZiMcMqHzXMBenVLnMxtb6sZIz0rJufL2S5mNJt0\/X4bfNaBIlV7OnTp9PLL79MNWrUoI4dO4oEFWpmH9PAMEF6gWj6bcgTG6hpx8GTtPDrA5R56JQrXl68EBdJVm+oXSI2kHS8PSbI9PXBdA2hIEg5aIQg2Cz+1ltv0QcffEA4Zti1a1excMNnsfVVI6iGaD2Khrm89zYeom\/3\/ts1wrOSXsVi+alN3ZJUo\/TFRkhPXyrRX5F3gkVQyoaKICVoOFWDechnnnmGdu\/ezWexHWqTlwQpSQ\/pRt7\/5hBt\/PF4LKRFt52cz9UdphraVimRn6qXvpha1ylJMiOe\/L1f53vtxuGlXHSxTLVcVMYSGoJEZh+Q4qJFi+idd94RckOo3aZNGypVqpQn85J+g5WqslrfS1V5VTI7ePwMzf9yn2uLFmqfVcLDym2VEgXo9nq\/o4ty5hAnNpAUIZ3Q1hSu6daTqlzSbdeN96MyFr9tXmsOEolze\/fuTZs2baJOnTpR27ZtKSMjw5OwWlUev8EypchSeVXCQzLSFVt+EkfPhGfnMPuKk75ZCe\/qS4tQk4yiKYW0UTFEDJ7H4kSLvCnrt81rESQ2iu\/YsUMs0Di5vRCZyIcOHUpr1qyh2rVri83m6kZzCfGKFSuoZ8+eMcTtUqj5DZauSkjiW775MC1cd8BVwlM9N5mQoN0VpbJ4dG56d0wqulrhbbmoyMVvm9ciyFREi5XvIUOGCG+zSZMm4trYGTNm0KRJk6hEif+tOqLu+fPni6xAyZJe+A2WxAEEePLMWZr8j93aWZd1MbR6d6UK5xU3v+XKef5KCzfJTreParmoGCJ7kKlI3\/13\/LZ51wgShjNx4kR66qmnqHDhwmKT+cMPP0yPPvqoCM\/VB3dtt2jRIksijHjQ+wEWyHDGqh\/E4ka8qyuTqYhKeJSD6LpqxahByTNUsUKFLK8GjfiSjUv+nglSFylvy0VFLn7YvCop1whSbQSr3qtWraK5c+eK9GhFi\/5vvgs3Jz7xxBNiNRzHGQHIoEGDhNdpvQgMv8OzfPlyV7Vt04Ez9Mzqn7TO5ZYtnIvKFMpFl5XOS7fXOX\/jI36W6JEXoLk6CI8q57F4BLTDZsIuF5zSkw+m6vx6tAgSJLZ582YxB5k3b95YX\/FzdB4eod3c5LZt28TNh4cPH6bBgweLRR617LFjx8S1sQjFMfe4bt06GjlyJI0aNYpq1qyZBRc3vybwFO0uOEcn4OEh\/fzAVlVEn9Lx+KLydQcOPBa\/TDdxu1GRi5s2ryM5rUu7jh49SuPGjaP+\/fuLC7rkg4QVCKNBZsnSneGLNmDAAOrVqxc1bdrUtm\/wNkePHk1Vq1YV24jUxw2wVm8\/Sn3mb8q2HxAEiMWOLo3Su7Yy3kCjorxMkDom5k+ZqOiYGzbvRCJpXdqVO3du6tOnj9gCZPUg4QkuW7aMHnzwQZJ3aSO8RiYglfggSITW2E+JcpIg4U22bt3aNYKEx9hm2rosxAhSRHaWCe2qOcHQcdmoKC8TpGPRe\/ZCVHQs0AQppYl9kFOmTBFkqM4fJpJ2ZmammEvENp9atWqJUOzJJ58UYbYaOu\/bt48GDhwo6q5fv74gS4TcTz\/9tCBTNzxIeI0gR\/mAGN\/u\/QfKmSNHWqGzrvZHRXmZIHUl7n25qOhYoAkSp2ewXadAgQKEjD7x7se2uxdbLsyA6LZs2ULVq1ePLb6AcHGdLIixUaNGtHHjRhFWJ9svmS5Y8BqRFXrYuztiGosU9hPbV\/dUg6OivEyQnqqNo8aiomPp2rwj0OIUThpiw+vr16+fSJYb737ssNyLDXKc98U+GrP0OwEDvMYpd9Skxr8\/n+LeyycqyssE6aXWOGsrKjoWaIJ0JhL3S6cDFvYyDn57W4wcF99bz5NwOh4qUVFeJkj3dT7VFqKiY+nYfKrYqe9pbfMx0ZCJOlIFS51zhOfoJzkyqZjQBHfqiAqpREnHUrV5UxqiRZA4BYM5wzCG2Ait\/zDy80B4jlJobIim1NdsPSwXs3iaqC0UBGkdKBZgsPEbqc+wL\/L2228PbLozkKNMHgHP0Y85Ryt+bIgmTMd8HSwX85imW2MoCVIOGivcU6dOFfsgcd7a7ccpWLM\/30MPvbFFdGvA9ZVp0A3nT8H4\/bAh+i2B+O2zXIInF6c2b3oEWiG2XaPYrjN+\/Hixwq27PzKdATgByxpafz306nSaNvouG6JROI1VxnIxBqWxipzYvLFGlYq0CBJEiFyOOC6oPjhq2K5dO7rvvvsc5YlMdSBOwGoz7Wtavf2IaCooobUcNxtiqhrg7nssF3fxTaV2JzafSv3J3tEiSCSl2LBhA+Fv9cFVCzgz7cWFXWhXFyzVe2xctQgtvq9eMhw8\/T0boqdwazfGctGGyrOCujbvVoe0CBKNY2Fm\/\/79IpsPwum9e\/dSwYIFPb36VResNlPXxXI3Bs17BJZsiG6pc3r1slzSw8+Nt3Vt3o22Uac2QS5YsIBGjBhBs2bNonr16omLu3CMEAkokmUCN9V5XbCKPfQP0WQQvUcmSFPaYL4eJkjzmKZbo67Np9uO3ftaBImcjcgE3rdvX3GmWj7YF\/nyyy\/TsGHDhDfp9qMD1mOLt9PUld+LrmBhJp28jW6Nhw3RLWTTq5flkh5+brytY\/NutCvr1CJIbBTHmWxk\/lbzPtr93K0O64AlvUcQY5BWrlVM2BDd0pD06mW5pIefG2\/r2Lwb7ToiyDNnzgiCvPzyy6l9+\/ZiUQZzkitXrqQlS5aI0Dt\/\/vxu9lPUnQws9UhhEOceJUBsiK6rSkoNsFxSgs3Vl5LZvKuNO5mDxNUKyOWIv3EDIbb4gBSRJ7JaNXcTzEoQkoElF2eCcN46keDYEN1W69TqZ7mkhpubbyWzeTfbRt1aIbbsxK+\/\/ioI8sCBA2L1GolvZbZwtzuazIMM+tYeDrG90JD02mCCTA8\/N94OBUEePHiQhg8fLjaLlyxZ0g0ctOpMBBZyPd43b5OoJ6iLMxxia4nZt0JMkL5Bb9twKAgSnuP06dOpdu3a1KxZM08SU8RDLBFYangd1MUZJsjgGSB79sGWSSgIEkcNhwwZIu6jxvyjmpgC\/4d36fdZ7KDvfWRDDLYhonfsQQZPRqEgSLujhoATJ2uwuq3el+0WzHZgqfOPQV69Zg\/SLc0wUy8TpBkcTdYSCoI0OeB06rIDK+hHC61jZkNMRwvce5fl4h62qdYcaIKU17127tyZJk6cmC2bDwYdhBBbJsUN8uZwDrFTNRHv3mOC9A5r3ZYCTZAIrTdv3kxVqlQRV7das\/kEIcRWw+uHWlSioTddqou9b+XYEH2DPmHDLJfgySXQBKnCFdRsPmE5PcMeZPCMj6c+gi+T0BBkULP5qPOPP028LvgS59XSwMqIPcjgiSYUBBnkbD5hm3+ECrIhBs8QWS7BlEkoCNLtbD44vjh06FBas2aN2IyOEzsNGjTIJrF4YMn9jwNbVaGBrSoHU8qWXjFBBlNMLJfgySUUBOlmNh\/cjIhN6G3btqUmTZrQ6tWracaMGTRp0iQqUaJEFolZwVLnH4N+vJDnIINnfDwHGXyZhIIgAaNb2Xzw1cYWoqeeekqc0IG3+vDDD4sEvRkZGQkJcszSTBqz9DtRhglX4tFuAAATRElEQVTSH2Vnr8sf3JO1GhW5hIYgIRC3s\/lgpXzVqlU0d+5ccZWD9fiiFawwnb9mDzKZSfv\/+6iQSpTmU0NFkG6q8LZt26h79+50+PBhkXeyU6dO2a6SBVh4cCZ8z7Ff6eaXzl9DW79cPnruttJuds9o3bg+Fxvso\/DwWIIpxbDLpXnz5jFgEb369TjKB+lFJyHYAQMGUK9evahp06a2Iba6QTxMCzRR+rrzWLywiNTaiIo3fMF7kOvWraNly5bRgw8+GEu+i\/C6cuXK1LFjR1uCVOcfw5CggkPs1Azdy7eiQipR+nBd8ASZmZlJgwYNEtt8atWqJfYI4v4bhNnIWK4+KlhhXaCJkvLyWLykb2dtRYXsQ0OQJ06coE8++YQ2bTqftVs+l1xyCXXo0CHla1\/lwgzu2MZ5b1wrC8LElp8cOXLYEmRYF2iYVJwZupelo0IqUdKxUBAk9kGOHDlSEFjDhg3FrYamCNKJAahghSlBrnWMbIhOpO5dWZaLd1jrthQKgsTeRJxuwYbuMmXK6I7NeDkJVpgXaKL0deexGFdxYxVGhexDQZA47TJu3Di65557AkGQYT1BI7U\/KsrLBGmMz4xXFBUdCwVBnjt3jj766CNxVhp7FS+++OKYQDFPiHlINew2Lu3\/VijBCmOKMxWTqCgvE6Rbmp5+vVHRsVAQJELsHj160Pr167NJrm7dujRz5kwqVqxY+lJNUoMEK4wpzpggXVePtBuICqlE6cMVCoJMW\/MMVWAlyLBcscCLNIYUwOVqmCBdBjiF6kNDkNiOs337dlq5ciVddNFF1LJlS3E2Gxu6rdtxUsBB6xUJVphXsKP0deexaKmtL4WiQvahIUikIRs2bJjY5iOPA2LrT5cuXeimm27yRAkA1sovvyUkycUTtiOGEqSoKC8TpCdqn1IjUdGxUBAkVrFBjt26daOyZcuKky5PPPEE7d+\/n6ZPny5SlRUsWDAlQTp5iQnSCVrelI2KITLZe6MvTlsJBUGqGcUxQEmQ6r+9WqTp9fcVsRyQYTuDzR6kU\/PwtjyTvbd467QWCoLEda\/Dhw+nFi1a0GWXXUYjRowQHuT3339P8+bNE\/\/Onz+\/znjTKgOwnpi7mh58Y4uoJ0xJctWBsyGmpQauvcxycQ3alCsOBUFidFu3bqX777+fcOwQl3hVqFCBjhw5QuPHj6f69eunDICTFwFWnYffotU7jlJYV7A5lHMicW\/LMkF6i7dOa6EhSAwGCSvWrl1L\/\/rXvwgdv\/rqqz3Z\/yiBZILUUSlvyzCpeIu3bmtRkUuoCBLCgQcJovTq9IyqEADraNuZ4keNqxahxffV09WXQJWLivKyNxwotcrSmajoWGgI8uDBg2IecunSpWL\/Y+7cuemvf\/0r9e3bN9vdMW6pTeU6DenY9WNE9eNur0Y9rinnVlOu1hsV5WWCdFVN0qo8KjoWCoKU6c7y5ctHvXv3piJFioi7Y5577jnCBvKBAwdmuz8mLenavKwSZFj3QDKpuKEZZuqMCqlEScdCQZDY5oOVaqQ8K1myZEwb9+7dK\/ZAwrP0YptP+WZ30skr7hLth3WLT5SUl8dihpjdqCUqZB8KgsRG8bFjx4qLtMqV+19YC4LERvH+\/ft7slGcCdINU0qvzqgYIpN9enrg1tuhIEikO\/vqq6\/o1Vdfpdtuu41q1KhBu3fvptmzZ4tjhldddZXAx20vsnS7kXSm4jWirZ8mXueWTFyvl0nFdYhTaoDlkhJsrr4UCoLEfkeE1ziDnehZuHChq2D9rst0+rVE9VDvgWRPxVUVSatyJsi04HPl5VAQpCsjT6HSEve8QecKlGCCTAE7t15hUnEL2fTqjYpcQkOQ2Nrz\/vvv0yuvvCJO0GAu8q677qJrrrnGk2ziIoR\/6B9Ca8K8B5I9yPQM3823o0IqUdKx0BDkggUL6L333qOuXbtS6dKlxWmaOXPmUOfOncW8pBePJMjpf61JHeqX9qJJV9pgQ3QF1rQrZbmkDaHxCkJBkDh7PXToUHrooYdEglz5ZGZm0sSJE8WVsIULFzYOjrVCSZBh3gMZpa87j8V1lU+5gaiQfSgIUk13pq5U2\/08ZakmeVESZFiz+MjhRUV5mSDd0vT0642KjoWCIHGSZtSoUXTFFVfQn\/\/8ZzHniBM0uH4Btx1ihTtv3rzpS1WTIMO8SZxJxXU1SbmBqJBKlHQsFAQJwJH7EUcK8XeJEiVEyjPcTTNlyhSqVq1aykrp5EX2IJ2g5U1ZJhVvcHbaSlTkEhqChICwkr1z5046cOAAFSpUiGrWrEl58uSxld2+ffvEUUQkuMARxUcffVRsLLde8rVixQrq2bNnrB67q2QlQYZ5k3iUvu48Fqe05V15JkgzWOf4DbGyC4\/MQn755ZdT+\/btadeuXTR48GDxBwSoPvPnz6fy5ctT48aNE\/YEBBnmRLk8B+mCohmsMiqkEqUPV6g8SCe6iAUczE0OGTKEypQpI14dM2aMWAXv2LFjlqqeffZZcZ1DnTp1khJk2PdARkl5eSxOLMLbslEh+8gSJMLxkydPiiQWWNQ5fvw4Pfzww3THHXfQn\/70p5i2wNNEpiCc7cZ5bwAyaNAgatKkSbZQHB5krkNbaO2TrbzVNsOt4cgmPOYoPDyWYEox7HJp3rx5DFhM6\/n1uBZiqwNCst2nn35a\/Ag3IqpXxGKxZ8KECdS2bVsReq9bt07sq8SqOeY41QcE+VCLSjT0pkv9wstIu1H5urMHaUQdXKkkKjoWWQ8SUocX+fbbb9PkyZOpS5cu4tRNsu1AmBIdPXo0Va1aNVsoDoIM+yZx4OK30E1aJI\/FJJrm6oqKXPweh2seJIjuzTffpA8\/\/FAk1MXxxHgPvnQIrdu0aSNWxCVBwpts3bp1Ng+ywD9nUZ7dn5rTJK6JEWAEAo1AJENsbPEBMWKRRk2ya5UEymF\/ZZ8+fcT1sSBLhNwIydVjjYGWIHeOEWAEIomAax7kN998Q927dxd316gPwueWLVtSjx49BDE2atSINm7cKMLqNWvWUO3atcXqd4MGDSIJOA+KEWAEwoOAawQZHgi4p4wAI8AIxEeACZI1gxFgBBgBGwSYIFk1GAFGgBFggmQdYAQYAUbAGQLsQTrDi0szAozABYRA4AkS+yJx1QNO1uB+7nvuuYe6detG+fPnD7SYdPuNK3WXLFkitjXhxFGrVq3EKj6yHwXl0R2L2t\/Vq1fTokWLxFavoMjKyTjWr18vsujjahHstMDpLmxaDsqjOxYc5cU9UpMmTaLTp0+LbFrIqhUk\/UqEKWx+2LBh1KtXL8rIyPAc\/sAT5KZNm2js2LFCQYsUKUIjRowQF4XdfPPNnoPlpEHdfuNoJZJ14AOAPJu4exyb52GciVLJOelLumV1xyLbQTq8fv36UalSpQTxB4UgdceBvbnYv\/vggw+K467INoXrRbAtLVeuXOnCaeR93bF8\/PHH4sAG0g7iFBt0DWNAXoQgP3AcsP3v3XffpWXLltELL7zABBlPYLNmzRI\/xg2KeLBXUhV4UIWs2+\/XXnuNTpw4ERvftm3bBFlis7x6vYWf49QdC\/qI46XTp08X3ZVEExSC1B0HPHqc3sDhBeQulYlXkAPVmsvUL7nojkUSJDz5AgUKCIJEdi0c\/Q3yA8\/x9ddfFzr06aefin6zB2mRGBQTHmPTpk1jGYCCSCBWRXPSbygCMrNLEkHyYHgsuAxNTerhlzI7GYv8gOEqjltuuYXmzp0rPLEgEKSTceDjhAenur788ksx7YFx2B2X9Vo2TsaCEBtpBmfPni26iZyr0C1EK2F4kDYR3i6mBZggLRI7deqUSH32l7\/8RcwD4QkDQabSb4QUuN8HJ4rwUUiWPNgr5XYyFsyhwvu9\/\/776ZdffqGXXnopMATpZBwgFBAjpnYqVKhA8PK\/\/fZbz+5eSiZbJ2PB\/D3+IKUgPrjTpk0Tc5FBmi5INF4myATo4LIwLFjgojBJGCBIKC7+FC1aNJku+fJ7p\/3+8ccfRRo4OSHt1R0\/OuDojgWLBjNmzKCKFSuKhQDIKUgEqTsOYAKCrFSpkshdigdzwpgDHzduXCCmPXTHIstde+21scQvW7duFbaDsQTVflS9ZIJMYqW4FAxzJ+oc5DvvvCO+iEFZxIg3BN1+Y0Gjf\/\/+dOutt4qMRkguHLRHZyzSq4Fs1Kds2bL04osv+hIeWXHUGQfesc7vgSDh2YM4g0IqOmORoXjDhg2zECTm87B45sVd9unqMhNkEgTV1TpMksPTApFgXjLIj26\/58yZI1K8YdI8KAsAVlx1x6K+FzQPEn3THQfKgRChayB4SZjYYhYUGemORQ2xQYiSWIM0Fg6x02Ay634vzG+BTIKy3cJuaIn6DU8EDyafMd\/48ssvZ6nG7lbHNGBM61WdsWBOK+gEqTsOlFu1apXwsrCaDX3r27dvoDwu3bHAi1y8eLHYFXHo0CHq2rWr2IIVhAVAHaVkD1IHJS7DCDACjIAPCAR+o7gPmHCTjAAjwAgIBJggWREYAUaAEbBBgAmSVYMRYAQYASZI1gFGgBFgBJwhwB6kM7y4NCPACFxACDBBXkDC5qEyAoyAMwSYIJ3hFYrSODss03Ml67CTssnqUn+P0yfYb4ccik5SnmGPKK777dixo5PmXC2LDFKdOnUSm8bdOhWEjfW4BRSPW224ClJEK2eCjKBgnZCek7JOoAKpzJs3zxE5on4TBIlN1Phj6tgmxoK0YdbN8E7w0CmL45rI24jN3H5krtHp44VWhgkypBJHYgtkiQbBITuLPCGB+8jh7eBBUmEcmfviiy9o\/PjxIjt2lSpV6L777hPHNZGxRi0LTw\/59\/AOUpZVrVpVZFNq0qRJ3CN26n3m1atXj5Vdu3ZttnrVlGfqSZUdO3aITE3IGg1vEwSJs\/dIqrB06VK68sorY9m85amQqVOnigQSuEP9kUceEf3bvn27KIcjqMgX+re\/\/U3kPQRGSJqBupEVCvXKFGxIVowxAx\/0YfDgwXTZZZdl0wiVIOWZ8z\/+8Y+0fPlygVOLFi1EneXKlcv2rpXwE304mCCDZ4xMkMGTiVaPkNT1s88+E9mO8KiJUFWvEMfL4PngWGOtWrUEQTz22GOCGECWalmkK0M5hHogml27dgnSwPv16tXL0i+E8CBPlAeJ4f9q2UREAGICcYCIkTXnjTfeEImQkSoNhCZTjeF3yGOIPiPhK\/qDTDQgU4S7SKT6\/PPPi7FjnOh37969RXiOHJtI3Hvy5Em69957Rfo1vHf27Fkx9h9++EHkGETSE+CC3I84joffIcRXn3gEuX\/\/flEeWdOB4SeffCI+QtYjfEyQWuoc2EJMkIEVTeKOgSBBHiBIXAugnk1XSU9mw4bh4t8gMpACCAdhnFoWdSKUxPlwpOfHY80aI3u1cOFC+vrrrwXZyraR1OHIkSOCNBMRJIgFWXFkhqZjx44RPEkQFcgO96Wo2ZtkqA7Sg7eMpCX4G\/fGoH8g1cOHD4vkEkjjBc8R\/UCWpAEDBpBMH4c+Ybwg4pkzZ4owXGYNl6nBrr\/++lhyZjlWOw9SzpPu3btXtAVcraExE2RIDey\/3WaCDKn8kCkaXguyAYEocE8PiAlhsUp6CNtApLi4CVmkYcDIBCNT2Ktl8W94gdYHFyZZ59\/izRWqddkRpBqixluISUQoIMXJkyeL0BseZPny5WnPnj3097\/\/XRCkelUFFj0QcsOrkxdUqRmGQKrIX2l94NVa+xWPINUkzokSKjBBhtTAmCDDLbgtW7ZQ8eLFBenBM0QeRtwkCJJ4++23Y6vY8ApxuyC8NnheVmNWSS2eV2i9EkKihmshdu\/eLTwnpACDNwbvDf2B92dHkDJHIbxemZAW857Ipt62bVvC\/KK6iq3WA68P\/UdoDK9VzS5vJUhMF8B7hIeNqQQ8wAfhPMJoZNZWvVj0\/\/jx45QvX75seUbjESQ8zdatW4t64UHiAwIPVrYlcbISJOSBS6jirezzHGTwbJI9yODJRKtH8BzhPSH9W+7cuYXXCNLEbYgwQniJ8AaRDxAECfJCgmEQBBYt4HUh9AQpyrKY41PnFZHpHIsPDzzwQLY5SLwDokI4jsWSzZs3i\/+DoEF+iUJshPHw3iShIjQH8YFkQOSJCBLEh3YQEoPk0M5zzz1HR48ezeJBwsPGNAIuPkMYjTAe\/8fKNsgJ\/ZXzoGgPC1j4ObxOnTlI1I+xyxyL6D\/IWE5NSCFiCkCG8ricDTKRfbDe1cMEqaX6nhZigvQUbnONyVVskB0MUL1YCiu8CItBVCBMLGTIlVx4fBs2bBBzjfDIfv7551hZEARWgxFmgnjUlWlroli0idVfzLuBmNUVZZRNRJDWHIXt2rUTJIxQOFFIijGD5OTqNt7BzXdYgOnZs6e4jEq9DRJkKstjLMAIXifGCU9R5nxE\/52uYtevX58++OCD2KVe8i5zEGWPHj0E2aNOZIwHoUMmWOVGdm\/ZBxCiWpYJ0px9mKqJCdIUklxPoBCIFzIjxMY9yyBNJ9d1JJuDNDVwJkhTSJqrhwnSHJZcU4AQkKvS2NeIBRV5Idp1110n9oc6eZggnaAVrbL\/D9LG9QetNLlVAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:9c65305f]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUgAAADGCAYAAACw2RkvAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQnYjlX6v8keso59y3z2MhJpEhqkMkllaYws0SWFVltUQmStsQ0VSYk2olQUI6n4VyM02fVR2UWMZRT9r98x553zPd\/7vO953vc8q\/u5Lhe+7zxn+d33\/Xvu+yz3yfHbb7\/9RvwwAowAI8AIZEMgBxMkawUjwAgwAvERYIJkzWAEGAFGwAYBJkhWDUaAEWAEmCBZBxgBRoARcIYAe5DO8HK19Jo1a6hTp040evRo6tixo\/G2tm3bRt27d6ebb76ZBg4caLx+tcJTp07RoEGDaPfu3TRz5kwqVqyYq+3Fq\/y1116jyZMn04svvkgZGRmet59qg1JOeN9J34OAuTpmOY7x48dTo0aNUoXD1\/eYIH2FP2vjXhPkTz\/9RD169KCKFSvS008\/Tfnz5zeGhtVYUbFbbcXrtBPjdIqDm3JScZsyZYqQie7HJWgECbmMGTOG3nnnHUdEb0wJDVR0wRAkBDVjxowYZK+++qr4qkmlwi9uueUW6tmzpyhj9eLU9+vWrRvziqSxPPbYY\/T1118LZYCHlohw4NkMHjxYtFO2bNmY8lgND23iZ3fffTf16dOHZJ+l8e\/ZsydLX+XP69evL9rHo3pxhw8fjnmQqBOEtX79elEuXp+t\/YlngMlwhQc5btw46t+\/f7a2UJ\/ah169emXxbGXdwAjlRowYkUUusn9SqBIfaZj4vfReZd8hHzxShpK4rTjY9c3aptSTRH2x2qkkZNmm7AvIEPKSfVR1Q63D+r7ETZWP1BmrLltxUOUuxwBifv7554V9INJQ9Q0RzvHjx7NEBnb6iLa9jFoM8GG2Ki4IglQJSUUABgXlVJXS+nsoiZUEVEKBkkNprI9dmGw1JNVYt27dmiXEjkc+1apVy0IqKjkUL15cEKApgpSGKA3F6mktXrw4RvR2uNoR5OOPP07Dhw+PkYF8X+JmJzM7QlI\/NsDB6q3ayXDAgAHi46MSZKK+VapUKYu80R\/rz6wfPhUbK0HJ30EPQUxjx45NSJB276Mfbdq0iavLKtHGw0ESrFU38XPrh1TtLz4+8qMrP9bWj5XUGfzcr6mWdIgz8gQpBXTw4MGYpyaNDwrQr18\/oVRfffVV3N9LBZFhqPTK8JUHweIBQcovsSTMeB6ZqtzS25EKq9YlSUD+TiVbte\/4ukulxlhuu+02bYLEuzqhpfRiodwqgbds2VKQUDJc5Ryk9NQkjtJDi\/d\/laCsHzEVGzV0U3GROPTt21fM5Vo9X+mpSZlbCTVR3+CZSxnr9MU61yvlJfXDqk\/yg203d2t9\/4cffojJPBFuKo6ybomD\/L+Ubzyv0qrf0uv98MMPxUdS6rPd+FT7SoewvH438gRpDTuhFOpXTX61AbwMi9V3EHL37t2brF9IlI9HavHak0JN9jWNF2Jb529UQrVOfDsJsXUJUl3o+Oc\/\/xlb9MCYVG\/VDlc7glTnO1VvEYYovTrV61CxsfOUUB4fiqZNm2Zb7LKbCsA7iT4U1r5ZCTJZX6wEKetTP3rqR05+sO0IMt77Ur8STYGo0w8or3qLkuysEQzKWduztoFQXJ26kn2xfgCYIL2mds32TBGk3cqvldSiRpByPPAWMccqPySq5yI\/LPE+PHYEafWc5NQBPEpdgnRKIsnmIe2iBLVvdgSpu1rvN0GqUYkkd6sHGS9ikT+LR5CJFmGSOQWaZuxbsch7kLohtgyZ1TlHdQ7Gbg7FCUFK5ZJf0\/Lly8fmjOxCbKvyWQ0sXmhZsmRJMd8j54es\/5dkrxNiq4tYMKQ77rhDhK26uNoRpDVMVUMzdV4wUYgNz8XqGanekTTqeOOUuOH9RASIsVrDxnghtl1frJZtKsS2zhtCpjLEVslajTjkHLWUv\/z4Sf1I5EEmC7Ht5tx5kcY3btdv2I1FGruwJJEHqYYsau+tymc3t4V3rCuG+JmchJdzaXLBQbYh+6quYqshNsonWnmX+FlXVXVwtRKkbMtu6kL2w24BKNEijZz2kEQg5yDx83iLE\/FwQ\/vJ+qYuzNkt0si+WKdBEi3S4KNmnRe0bvGxrmBLGcebY8S7KkHaLfAlCrHt2rPqlHUKStVhdTeBvtUGo2TkPUgJs852FLutEVYDS7Q1JxlBWuuKt2UoEUFa54+shqjOLcmtR5KkrASpGmsigkw0Jh1cVcNXt0FNmjRJzF8Bz6lTp9KsWbNii2Wqdw2MpGzizd1JGSfyGOMRk3Wrl27f0B7mX0EKdoSdaLO\/3TYfEJrOXka7bTU6c5DqRw3j\/\/jjj4UM4kUwEtdEOoU+W\/sjvVsne1GDQYfZe+EJQf773\/+mJ554gtq3bx93R\/2KFSti+w\/RRZU03AZORyHd7gPXnxWBZKv9OqcyQATz588P5daSIOlDolVz04cLgjRu2RfXCRLpJt98800aMmQIzZkzJy5BQpHhMTRu3NhzjJggPYdcq8F44bvdxul4FUrvRQ2ztRrmQlkQsAuxrZv6owqb6wS5Y8cOeuGFF+js2bNin168r\/+zzz5LLVq0oDp16kQVZx4XI8AIhBABVwkS3hnmYjp06EBLliwR+9OsBPmf\/\/xHhN+YJ8Pq7qWXXipWdps0aUI5cuQIIaTcZUaAEYgKAq4S5IIFC+jAgQNi8y6OUMUjyGPHjtGECROobdu2Yu5x3bp1NHLkSBo1ahTVrFkzC84gT34YAUbgwkFg+fLlVKVKFd8G7BpBZmZmirOlmHssWrSo2G4QjyCtI8ecJbzOqlWrZkv5BYLcuXOnb2CZajgq4wAePBZTWmG2nqjIxe9xuEaQ8ZIyQAWsk7vfffedCK2xqz9PnjwkCRLeZOvWrbN5kEyQZg0p3dr8VuB0+6++z2MxiaaZuvyWiWsEaYXHzoPct2+fSKmEjCrIQgOyRMiNLQSVK1eOJEHio+Bn2GBGdc\/XwmMxiaa5uqIilwuSIOXWARAjFm02btwowmp4nbVr1xZHpho0aJBNW\/wGy5T6RkV5mSBNaYT5eqKiY37bvGcepAkV8BssE2NgUjGFovl6okIqUdIxv20+IUEeOXJEeHPI3KLzLFy4UKdYymX8BivljlteZEM0haTZelguZvE0UZvfNp+QIBEKY0\/irbfeSoUKFUo4XqRhv\/HGG01gYluH32CZGhwboikkzdbDcjGLp4na\/Lb5pB4ktupgAQVbdRI98DaTlUkXML\/BSrf\/8n02RFNImq2H5WIWTxO1+W3zCQkSSSawcHLVVVcl9SBNgJGsDr\/BStY\/3d+zIeoi5W05lou3eOu05rfNJ\/Ug77\/\/flq7dq3IF9iuXTu68sorKVeuXDpjM17Gb7BMDYgN0RSSZuthuZjF00Rtftt80lXsc+fO0a5du2jRokX0xhtviDF37txZbOxGdhUvz0v7DZYJgaMONkRTSJqth+ViFk8Ttflt80kJUh3kr7\/+Km62Q\/qyt956i2rUqCESmTZr1swTr9JvsEwInAnSFIrm62GCNI9pujX6bfOOCFIOFscBkU0ZF7nv37\/fs6SkfoOVrrDl+2yIppA0Ww\/LxSyeJmrz2+YdESRWqj\/66COR+BbpyXAfNLLwZGRkUM6cOU3gkbAOv8EyNUA2RFNImq2H5WIWTxO1+W3zSQkS+xtXrVpFr7zyirj289prr6U777xTLNaodxubACNZHX6Dlax\/ur9nQ9RFyttyLBdv8dZpzW+bT7pRHPchnz59mrp27UrNmzcnXBHp1+M3WKbGzYZoCkmz9bBczOJpoja\/bT4hQSLbN+YYcV+MFyF0MkD9BitZ\/3R\/z4aoi5S35Vgu3uKt05rfNp\/Ug3zyySfFlQjW+3mtg8OxxGRldABJVMZvsNLtv3yfDdEUkmbrYbmYxdNEbX7bfFKCfOCBB8R2nmTzjbh\/5tFHHzWBiW0dfoNlanBsiKaQNFsPy8UsniZq89vmk4bYGzZsIITaOo\/b17b6DZYOBjpl2BB1UPK+DMvFe8yTtei3zSddxU42AC9\/7zdYpsbKhmgKSbP1sFzM4mmiNr9tngnShBQd1sGG6BAwj4qzXDwC2kEzTJAhAstBVxMWZUM0haTZelguZvFMt7bV249Su0FTaN+bQ9OtKuX32YNMGbrUX2RDTB07N99kubiJrvO6532xj+6bt4l+mnid85cNvcEEaQhIJ9WwITpBy7uyLBfvsNZpKVQEieS5kyZNotdee01cFo8tPTiCeNddd7meSVyC6fd8hI5QdcqwIeqg5H0Zlov3mCdqMTQEiew906dPp4MHD9Jtt91GM2bMEASJS7qw\/xFJdb1IossEGSwFRm+YVIInk6jIZczSTBqz9Lvgh9jydkOQYt68eUmersH+yFGjRtHw4cM98SKZIINnjEyQwZMJE6Q5mWjNQdoR5NmzZ2n06NH02GOPMUE6kAmTigOwPCzKcvEQbI2mQuNByhAb92N36NCBnn\/+eRFWf\/DBB4Qs43379uUQW0PgsggbogOwPCzKcvEQbI2mQkOQGAvmGmfOnEmTJ0+mX375RRAi7qbBWe3ChQtrDDd5ESwEITFG+\/btqVGjRtle4BA7OYZel2BS8RpxvfaiIJdQEaQUCzxGJNG9+OKLKU+ePHrS0igFLxV33QwZMkRkLGeC1AAtAEWiYIjs2QdAkeJ0Yeii7TTt4++Dv0gDz+7111+nn3\/+OS6S1apVo4YNG6aVTHfHjh30wgsvEOY1sVLOBBlMpbX2igkymHKKglzaTF1Hq3ccDQdBTps2jRYvXkwtW7YUIfWhQ4do2bJldP3114uVbeyJxH01qTwI37HYg\/nNJUuWUNOmTZkgUwHSh3eiYIjsQfqgOBpNhoYgsZ1n\/PjxdMstt1CdOnViQ1u\/fj29++679Mgjj9C2bduy\/E5j\/LEiCxYsoAMHDlCvXr1o7NixCQkSLy1fvtxJ9YEri8UuZGmPwsNjCaYUwy4XXO9ytO1MAW7gjxoiW3i8zOLqzzGQVDKKZ2Zm0pQpU8TcY9GiRWnMmDHsQQbT5uL2ij3IYAor7HLZ\/dNp+sPIz8NBkPAgsRm8RYsW1KxZM8qRIwdhUQVhNby\/p556ijCHWLduXcfasmbNGnF9rPWBNzlw4MAsP+ZVbMfwuv5C2A1RBYjH4rq6aDeATD5tpq0LB0Gilzt37qTBgweLuUfMQeKmQ2wgR+hdqlQpsS\/yvffe0wbAriB7kGlD6GkFTCqewq3dWJjlAu8R5Ii\/c548RIemt9cet+mCWidpZKPY4gOixHwhtvlkZGRQwYIFxWZxrD5jsSbdhwkyXQS9fT\/MhmhFisfire7YtaZ6jwXWzaYf\/vGSbx3TJkiE1Nj\/CDKUD7b9PPvsszRs2DA+auhAhGyIDsDysCjLxUOwbZpS5x4rFstHR+ffT5nf\/J9vHdMmSMw1Dho0KAtBFipUiLp160b33nuvEe8xGQo8B5kMIe9\/z6TiPeY6LYZRLio5YoyL761HXa6\/QkStfj1aBImN4lhlvvvuu0U\/V65cKUgRm8cxH3nTTTd50n8mSE9gdtRIGA3RboA8FkeiN1YYxCjnHWWlr9x1Gd1Up4TIPRt4glS385w4cYJmzZpFAwYMoKNHj4otOli8wVyk24\/fYJkaHxuiKSTN1sNyMYunTm2Yb+wzf5MgSPl8MfgqqlqygPiv3zav7UFinvHOO++kihUrim09CLeRtAL\/xhagVPZA6gColvEbLKf9ZU\/FFGLe1MME6Q3OaEVdqZatYs4RYTX+lo\/fNq9FkOjs6tWrRXJcLMrgJAsSS5w5c4ZuuOEGsV+RM4rrKxcboj5WXpZkubiLNkhx\/IeZ9MravVkaAiGOaPN7qlu+UBZyDI0HKUeDrTx4sJL97bffin\/XqlXLkwWaIIBlSn3YEE0habYelotZPKWn2GfeJpF0wvqAGDtfVYY61C+djRhD5UFikeb999+nG2+8MctcI36OBBWtWrWi\/Pnzm0fXUqPf7rapAbIhmkLSbD0sl\/TxhJc4ZeVu2rz3RFxSRAsgxil31KTGvy+StEG\/bT5hiI0jhhs2bBCnZ5CjsUuXLoStPfLB4g0u7powYQLPQSYV9f8KsCE6AMvDoiwXZ2CDDM\/99hv1m7\/ZlgxljfHmF3VaCzRByrtosMwus4OoSXLz5ctH3bt3FynQcD7b7cdvsEyNjw3RFJJm62G5JMbzy13HaPi7O5KSofQSO9QvRZ2vKmsbPutIz2+b11qksQuxdQZosozfYJkaCxuiKSTN1sNyOb+6\/MvZ32jiR5n0\/U+ntckQknimfXXKfVFOrdBZV3J+23xCgjx37pzIIo5jhnYPPMdLLrmEcubMqTvmlMv5DVbKHbe8yIZoCkmz9VwocpF7Dues2UP\/993PWiSohsr4d6eGZeiOK+0XV0xJxm+bT0iQmGPs0aMHITGu3YMUZ7jMi\/dB6qvEhWKI+ogEo2SU5PLp+m1UoUIFmv35Hvoy82fafeT8aRXdR+5FvKZqEZr6l5riXXV\/om496ZYLNEGmOzjT7\/sNlqnxRMkQeSymtMJZPZLsXvp8D32RAgFaPcJrf1+U+l9fWfzYDyK0G73fNq81B4nOI8zeunUrffjhh3Tw4EFxZQC2\/Xh5dYDfYDlTYfvSTCqmkDRbT1DkIsnv2OlfafZne2jr\/hOOPUAVGRBexaL56NKSBejZDtV98wZTkZbfNq9NkPIkDS7pqlSpEn355ZciaQXyNzZu3DiVsTt+x2+wHHfY5oWgGKKJ8fBYnKEoye\/Qv8\/QrM\/20O7Dp9IiP9XjAwne26wC1SpTkL7\/\/nu6pm6Gs84FsLTfNq9FkFjFxllspDZTL+365ptvaPbs2eJ3nKxCX7uYVPSx8rJkunKR5PfBvw7RuxsOiq47nfuLN17pAVYsnp+6XV2WrqxUOKkXmO5YvMQ9UVuhIEidS7t4kUZfpaKivBhx1MeiLmy8\/fUB+mjTYWPEZ\/X+sDL8x6rnT5ekOw8YFbmEgiBxb\/Xjjz8u7q1u0KBBjAnYg9QnRbVkVJQ3zASpzvPN\/PRH2nHgJJ06fZq++lF\/pTeR9CXBIexF6q4OV5aickXOZ6lJl\/x0tC4qOhYKgoRAvvrqK3H\/9eWXX06VK1cWxw95DlJHVbOXiYryBo0g5VaUzMOnaNH6g7TcsLdn9fgqFMtHzWsUFyGvV8Snq3FR0bHQECQEg1AbpLhr1y4qXrw44XLvcuXK6cos7XJ+g5X2AP5bQVSU1wuClJ7ekZO\/iDRZW\/adMBriSpnCq0OWqktLFiTM9bW+rATVLnM+CbQXHp8p3ZL1REXH\/LZ5rUUaXNaFK12xWl22bFlPzl3HUxi\/wTKlxFFR3lQIUp3TW7XtCH2+86g40mZiMcMqHzXMBenVLnMxtb6sZIz0rJufL2S5mNJt0\/X4bfNaBIlV7OnTp9PLL79MNWrUoI4dO4oEFWpmH9PAMEF6gWj6bcgTG6hpx8GTtPDrA5R56JQrXl68EBdJVm+oXSI2kHS8PSbI9PXBdA2hIEg5aIQg2Cz+1ltv0QcffEA4Zti1a1excMNnsfVVI6iGaD2Khrm89zYeom\/3\/ts1wrOSXsVi+alN3ZJUo\/TFRkhPXyrRX5F3gkVQyoaKICVoOFWDechnnnmGdu\/ezWexHWqTlwQpSQ\/pRt7\/5hBt\/PF4LKRFt52cz9UdphraVimRn6qXvpha1ylJMiOe\/L1f53vtxuGlXHSxTLVcVMYSGoJEZh+Q4qJFi+idd94RckOo3aZNGypVqpQn85J+g5WqslrfS1V5VTI7ePwMzf9yn2uLFmqfVcLDym2VEgXo9nq\/o4ty5hAnNpAUIZ3Q1hSu6daTqlzSbdeN96MyFr9tXmsOEolze\/fuTZs2baJOnTpR27ZtKSMjw5OwWlUev8EypchSeVXCQzLSFVt+EkfPhGfnMPuKk75ZCe\/qS4tQk4yiKYW0UTFEDJ7H4kSLvCnrt81rESQ2iu\/YsUMs0Di5vRCZyIcOHUpr1qyh2rVri83m6kZzCfGKFSuoZ8+eMcTtUqj5DZauSkjiW775MC1cd8BVwlM9N5mQoN0VpbJ4dG56d0wqulrhbbmoyMVvm9ciyFREi5XvIUOGCG+zSZMm4trYGTNm0KRJk6hEif+tOqLu+fPni6xAyZJe+A2WxAEEePLMWZr8j93aWZd1MbR6d6UK5xU3v+XKef5KCzfJTreParmoGCJ7kKlI3\/13\/LZ51wgShjNx4kR66qmnqHDhwmKT+cMPP0yPPvqoCM\/VB3dtt2jRIksijHjQ+wEWyHDGqh\/E4ka8qyuTqYhKeJSD6LpqxahByTNUsUKFLK8GjfiSjUv+nglSFylvy0VFLn7YvCop1whSbQSr3qtWraK5c+eK9GhFi\/5vvgs3Jz7xxBNiNRzHGQHIoEGDhNdpvQgMv8OzfPlyV7Vt04Ez9Mzqn7TO5ZYtnIvKFMpFl5XOS7fXOX\/jI36W6JEXoLk6CI8q57F4BLTDZsIuF5zSkw+m6vx6tAgSJLZ582YxB5k3b95YX\/FzdB4eod3c5LZt28TNh4cPH6bBgweLRR617LFjx8S1sQjFMfe4bt06GjlyJI0aNYpq1qyZBRc3vybwFO0uOEcn4OEh\/fzAVlVEn9Lx+KLydQcOPBa\/TDdxu1GRi5s2ryM5rUu7jh49SuPGjaP+\/fuLC7rkg4QVCKNBZsnSneGLNmDAAOrVqxc1bdrUtm\/wNkePHk1Vq1YV24jUxw2wVm8\/Sn3mb8q2HxAEiMWOLo3Su7Yy3kCjorxMkDom5k+ZqOiYGzbvRCJpXdqVO3du6tOnj9gCZPUg4QkuW7aMHnzwQZJ3aSO8RiYglfggSITW2E+JcpIg4U22bt3aNYKEx9hm2rosxAhSRHaWCe2qOcHQcdmoKC8TpGPRe\/ZCVHQs0AQppYl9kFOmTBFkqM4fJpJ2ZmammEvENp9atWqJUOzJJ58UYbYaOu\/bt48GDhwo6q5fv74gS4TcTz\/9tCBTNzxIeI0gR\/mAGN\/u\/QfKmSNHWqGzrvZHRXmZIHUl7n25qOhYoAkSp2ewXadAgQKEjD7x7se2uxdbLsyA6LZs2ULVq1ePLb6AcHGdLIixUaNGtHHjRhFWJ9svmS5Y8BqRFXrYuztiGosU9hPbV\/dUg6OivEyQnqqNo8aiomPp2rwj0OIUThpiw+vr16+fSJYb737ssNyLDXKc98U+GrP0OwEDvMYpd9Skxr8\/n+LeyycqyssE6aXWOGsrKjoWaIJ0JhL3S6cDFvYyDn57W4wcF99bz5NwOh4qUVFeJkj3dT7VFqKiY+nYfKrYqe9pbfMx0ZCJOlIFS51zhOfoJzkyqZjQBHfqiAqpREnHUrV5UxqiRZA4BYM5wzCG2Ait\/zDy80B4jlJobIim1NdsPSwXs3iaqC0UBGkdKBZgsPEbqc+wL\/L2228PbLozkKNMHgHP0Y85Ryt+bIgmTMd8HSwX85imW2MoCVIOGivcU6dOFfsgcd7a7ccpWLM\/30MPvbFFdGvA9ZVp0A3nT8H4\/bAh+i2B+O2zXIInF6c2b3oEWiG2XaPYrjN+\/Hixwq27PzKdATgByxpafz306nSaNvouG6JROI1VxnIxBqWxipzYvLFGlYq0CBJEiFyOOC6oPjhq2K5dO7rvvvsc5YlMdSBOwGoz7Wtavf2IaCooobUcNxtiqhrg7nssF3fxTaV2JzafSv3J3tEiSCSl2LBhA+Fv9cFVCzgz7cWFXWhXFyzVe2xctQgtvq9eMhw8\/T0boqdwazfGctGGyrOCujbvVoe0CBKNY2Fm\/\/79IpsPwum9e\/dSwYIFPb36VResNlPXxXI3Bs17BJZsiG6pc3r1slzSw8+Nt3Vt3o22Uac2QS5YsIBGjBhBs2bNonr16omLu3CMEAkokmUCN9V5XbCKPfQP0WQQvUcmSFPaYL4eJkjzmKZbo67Np9uO3ftaBImcjcgE3rdvX3GmWj7YF\/nyyy\/TsGHDhDfp9qMD1mOLt9PUld+LrmBhJp28jW6Nhw3RLWTTq5flkh5+brytY\/NutCvr1CJIbBTHmWxk\/lbzPtr93K0O64AlvUcQY5BWrlVM2BDd0pD06mW5pIefG2\/r2Lwb7ToiyDNnzgiCvPzyy6l9+\/ZiUQZzkitXrqQlS5aI0Dt\/\/vxu9lPUnQws9UhhEOceJUBsiK6rSkoNsFxSgs3Vl5LZvKuNO5mDxNUKyOWIv3EDIbb4gBSRJ7JaNXcTzEoQkoElF2eCcN46keDYEN1W69TqZ7mkhpubbyWzeTfbRt1aIbbsxK+\/\/ioI8sCBA2L1GolvZbZwtzuazIMM+tYeDrG90JD02mCCTA8\/N94OBUEePHiQhg8fLjaLlyxZ0g0ctOpMBBZyPd43b5OoJ6iLMxxia4nZt0JMkL5Bb9twKAgSnuP06dOpdu3a1KxZM08SU8RDLBFYangd1MUZJsjgGSB79sGWSSgIEkcNhwwZIu6jxvyjmpgC\/4d36fdZ7KDvfWRDDLYhonfsQQZPRqEgSLujhoATJ2uwuq3el+0WzHZgqfOPQV69Zg\/SLc0wUy8TpBkcTdYSCoI0OeB06rIDK+hHC61jZkNMRwvce5fl4h62qdYcaIKU17127tyZJk6cmC2bDwYdhBBbJsUN8uZwDrFTNRHv3mOC9A5r3ZYCTZAIrTdv3kxVqlQRV7das\/kEIcRWw+uHWlSioTddqou9b+XYEH2DPmHDLJfgySXQBKnCFdRsPmE5PcMeZPCMj6c+gi+T0BBkULP5qPOPP028LvgS59XSwMqIPcjgiSYUBBnkbD5hm3+ECrIhBs8QWS7BlEkoCNLtbD44vjh06FBas2aN2IyOEzsNGjTIJrF4YMn9jwNbVaGBrSoHU8qWXjFBBlNMLJfgySUUBOlmNh\/cjIhN6G3btqUmTZrQ6tWracaMGTRp0iQqUaJEFolZwVLnH4N+vJDnIINnfDwHGXyZhIIgAaNb2Xzw1cYWoqeeekqc0IG3+vDDD4sEvRkZGQkJcszSTBqz9DtRhglX4tFuAAATRElEQVTSH2Vnr8sf3JO1GhW5hIYgIRC3s\/lgpXzVqlU0d+5ccZWD9fiiFawwnb9mDzKZSfv\/+6iQSpTmU0NFkG6q8LZt26h79+50+PBhkXeyU6dO2a6SBVh4cCZ8z7Ff6eaXzl9DW79cPnruttJuds9o3bg+Fxvso\/DwWIIpxbDLpXnz5jFgEb369TjKB+lFJyHYAQMGUK9evahp06a2Iba6QTxMCzRR+rrzWLywiNTaiIo3fMF7kOvWraNly5bRgw8+GEu+i\/C6cuXK1LFjR1uCVOcfw5CggkPs1Azdy7eiQipR+nBd8ASZmZlJgwYNEtt8atWqJfYI4v4bhNnIWK4+KlhhXaCJkvLyWLykb2dtRYXsQ0OQJ06coE8++YQ2bTqftVs+l1xyCXXo0CHla1\/lwgzu2MZ5b1wrC8LElp8cOXLYEmRYF2iYVJwZupelo0IqUdKxUBAk9kGOHDlSEFjDhg3FrYamCNKJAahghSlBrnWMbIhOpO5dWZaLd1jrthQKgsTeRJxuwYbuMmXK6I7NeDkJVpgXaKL0deexGFdxYxVGhexDQZA47TJu3Di65557AkGQYT1BI7U\/KsrLBGmMz4xXFBUdCwVBnjt3jj766CNxVhp7FS+++OKYQDFPiHlINew2Lu3\/VijBCmOKMxWTqCgvE6Rbmp5+vVHRsVAQJELsHj160Pr167NJrm7dujRz5kwqVqxY+lJNUoMEK4wpzpggXVePtBuICqlE6cMVCoJMW\/MMVWAlyLBcscCLNIYUwOVqmCBdBjiF6kNDkNiOs337dlq5ciVddNFF1LJlS3E2Gxu6rdtxUsBB6xUJVphXsKP0deexaKmtL4WiQvahIUikIRs2bJjY5iOPA2LrT5cuXeimm27yRAkA1sovvyUkycUTtiOGEqSoKC8TpCdqn1IjUdGxUBAkVrFBjt26daOyZcuKky5PPPEE7d+\/n6ZPny5SlRUsWDAlQTp5iQnSCVrelI2KITLZe6MvTlsJBUGqGcUxQEmQ6r+9WqTp9fcVsRyQYTuDzR6kU\/PwtjyTvbd467QWCoLEda\/Dhw+nFi1a0GWXXUYjRowQHuT3339P8+bNE\/\/Onz+\/znjTKgOwnpi7mh58Y4uoJ0xJctWBsyGmpQauvcxycQ3alCsOBUFidFu3bqX777+fcOwQl3hVqFCBjhw5QuPHj6f69eunDICTFwFWnYffotU7jlJYV7A5lHMicW\/LMkF6i7dOa6EhSAwGCSvWrl1L\/\/rXvwgdv\/rqqz3Z\/yiBZILUUSlvyzCpeIu3bmtRkUuoCBLCgQcJovTq9IyqEADraNuZ4keNqxahxffV09WXQJWLivKyNxwotcrSmajoWGgI8uDBg2IecunSpWL\/Y+7cuemvf\/0r9e3bN9vdMW6pTeU6DenY9WNE9eNur0Y9rinnVlOu1hsV5WWCdFVN0qo8KjoWCoKU6c7y5ctHvXv3piJFioi7Y5577jnCBvKBAwdmuz8mLenavKwSZFj3QDKpuKEZZuqMCqlEScdCQZDY5oOVaqQ8K1myZEwb9+7dK\/ZAwrP0YptP+WZ30skr7hLth3WLT5SUl8dihpjdqCUqZB8KgsRG8bFjx4qLtMqV+19YC4LERvH+\/ft7slGcCdINU0qvzqgYIpN9enrg1tuhIEikO\/vqq6\/o1Vdfpdtuu41q1KhBu3fvptmzZ4tjhldddZXAx20vsnS7kXSm4jWirZ8mXueWTFyvl0nFdYhTaoDlkhJsrr4UCoLEfkeE1ziDnehZuHChq2D9rst0+rVE9VDvgWRPxVUVSatyJsi04HPl5VAQpCsjT6HSEve8QecKlGCCTAE7t15hUnEL2fTqjYpcQkOQ2Nrz\/vvv0yuvvCJO0GAu8q677qJrrrnGk2ziIoR\/6B9Ca8K8B5I9yPQM3823o0IqUdKx0BDkggUL6L333qOuXbtS6dKlxWmaOXPmUOfOncW8pBePJMjpf61JHeqX9qJJV9pgQ3QF1rQrZbmkDaHxCkJBkDh7PXToUHrooYdEglz5ZGZm0sSJE8WVsIULFzYOjrVCSZBh3gMZpa87j8V1lU+5gaiQfSgIUk13pq5U2\/08ZakmeVESZFiz+MjhRUV5mSDd0vT0642KjoWCIHGSZtSoUXTFFVfQn\/\/8ZzHniBM0uH4Btx1ihTtv3rzpS1WTIMO8SZxJxXU1SbmBqJBKlHQsFAQJwJH7EUcK8XeJEiVEyjPcTTNlyhSqVq1aykrp5EX2IJ2g5U1ZJhVvcHbaSlTkEhqChICwkr1z5046cOAAFSpUiGrWrEl58uSxld2+ffvEUUQkuMARxUcffVRsLLde8rVixQrq2bNnrB67q2QlQYZ5k3iUvu48Fqe05V15JkgzWOf4DbGyC4\/MQn755ZdT+\/btadeuXTR48GDxBwSoPvPnz6fy5ctT48aNE\/YEBBnmRLk8B+mCohmsMiqkEqUPV6g8SCe6iAUczE0OGTKEypQpI14dM2aMWAXv2LFjlqqeffZZcZ1DnTp1khJk2PdARkl5eSxOLMLbslEh+8gSJMLxkydPiiQWWNQ5fvw4Pfzww3THHXfQn\/70p5i2wNNEpiCc7cZ5bwAyaNAgatKkSbZQHB5krkNbaO2TrbzVNsOt4cgmPOYoPDyWYEox7HJp3rx5DFhM6\/n1uBZiqwNCst2nn35a\/Ag3IqpXxGKxZ8KECdS2bVsReq9bt07sq8SqOeY41QcE+VCLSjT0pkv9wstIu1H5urMHaUQdXKkkKjoWWQ8SUocX+fbbb9PkyZOpS5cu4tRNsu1AmBIdPXo0Va1aNVsoDoIM+yZx4OK30E1aJI\/FJJrm6oqKXPweh2seJIjuzTffpA8\/\/FAk1MXxxHgPvnQIrdu0aSNWxCVBwpts3bp1Ng+ywD9nUZ7dn5rTJK6JEWAEAo1AJENsbPEBMWKRRk2ya5UEymF\/ZZ8+fcT1sSBLhNwIydVjjYGWIHeOEWAEIomAax7kN998Q927dxd316gPwueWLVtSjx49BDE2atSINm7cKMLqNWvWUO3atcXqd4MGDSIJOA+KEWAEwoOAawQZHgi4p4wAI8AIxEeACZI1gxFgBBgBGwSYIFk1GAFGgBFggmQdYAQYAUbAGQLsQTrDi0szAozABYRA4AkS+yJx1QNO1uB+7nvuuYe6detG+fPnD7SYdPuNK3WXLFkitjXhxFGrVq3EKj6yHwXl0R2L2t\/Vq1fTokWLxFavoMjKyTjWr18vsujjahHstMDpLmxaDsqjOxYc5cU9UpMmTaLTp0+LbFrIqhUk\/UqEKWx+2LBh1KtXL8rIyPAc\/sAT5KZNm2js2LFCQYsUKUIjRowQF4XdfPPNnoPlpEHdfuNoJZJ14AOAPJu4exyb52GciVLJOelLumV1xyLbQTq8fv36UalSpQTxB4UgdceBvbnYv\/vggw+K467INoXrRbAtLVeuXOnCaeR93bF8\/PHH4sAG0g7iFBt0DWNAXoQgP3AcsP3v3XffpWXLltELL7zABBlPYLNmzRI\/xg2KeLBXUhV4UIWs2+\/XXnuNTpw4ERvftm3bBFlis7x6vYWf49QdC\/qI46XTp08X3ZVEExSC1B0HPHqc3sDhBeQulYlXkAPVmsvUL7nojkUSJDz5AgUKCIJEdi0c\/Q3yA8\/x9ddfFzr06aefin6zB2mRGBQTHmPTpk1jGYCCSCBWRXPSbygCMrNLEkHyYHgsuAxNTerhlzI7GYv8gOEqjltuuYXmzp0rPLEgEKSTceDjhAenur788ksx7YFx2B2X9Vo2TsaCEBtpBmfPni26iZyr0C1EK2F4kDYR3i6mBZggLRI7deqUSH32l7\/8RcwD4QkDQabSb4QUuN8HJ4rwUUiWPNgr5XYyFsyhwvu9\/\/776ZdffqGXXnopMATpZBwgFBAjpnYqVKhA8PK\/\/fZbz+5eSiZbJ2PB\/D3+IKUgPrjTpk0Tc5FBmi5INF4myATo4LIwLFjgojBJGCBIKC7+FC1aNJku+fJ7p\/3+8ccfRRo4OSHt1R0\/OuDojgWLBjNmzKCKFSuKhQDIKUgEqTsOYAKCrFSpkshdigdzwpgDHzduXCCmPXTHIstde+21scQvW7duFbaDsQTVflS9ZIJMYqW4FAxzJ+oc5DvvvCO+iEFZxIg3BN1+Y0Gjf\/\/+dOutt4qMRkguHLRHZyzSq4Fs1Kds2bL04osv+hIeWXHUGQfesc7vgSDh2YM4g0IqOmORoXjDhg2zECTm87B45sVd9unqMhNkEgTV1TpMksPTApFgXjLIj26\/58yZI1K8YdI8KAsAVlx1x6K+FzQPEn3THQfKgRChayB4SZjYYhYUGemORQ2xQYiSWIM0Fg6x02Ay634vzG+BTIKy3cJuaIn6DU8EDyafMd\/48ssvZ6nG7lbHNGBM61WdsWBOK+gEqTsOlFu1apXwsrCaDX3r27dvoDwu3bHAi1y8eLHYFXHo0CHq2rWr2IIVhAVAHaVkD1IHJS7DCDACjIAPCAR+o7gPmHCTjAAjwAgIBJggWREYAUaAEbBBgAmSVYMRYAQYASZI1gFGgBFgBJwhwB6kM7y4NCPACFxACDBBXkDC5qEyAoyAMwSYIJ3hFYrSODss03Ml67CTssnqUn+P0yfYb4ccik5SnmGPKK777dixo5PmXC2LDFKdOnUSm8bdOhWEjfW4BRSPW224ClJEK2eCjKBgnZCek7JOoAKpzJs3zxE5on4TBIlN1Phj6tgmxoK0YdbN8E7w0CmL45rI24jN3H5krtHp44VWhgkypBJHYgtkiQbBITuLPCGB+8jh7eBBUmEcmfviiy9o\/PjxIjt2lSpV6L777hPHNZGxRi0LTw\/59\/AOUpZVrVpVZFNq0qRJ3CN26n3m1atXj5Vdu3ZttnrVlGfqSZUdO3aITE3IGg1vEwSJs\/dIqrB06VK68sorY9m85amQqVOnigQSuEP9kUceEf3bvn27KIcjqMgX+re\/\/U3kPQRGSJqBupEVCvXKFGxIVowxAx\/0YfDgwXTZZZdl0wiVIOWZ8z\/+8Y+0fPlygVOLFi1EneXKlcv2rpXwE304mCCDZ4xMkMGTiVaPkNT1s88+E9mO8KiJUFWvEMfL4PngWGOtWrUEQTz22GOCGECWalmkK0M5hHogml27dgnSwPv16tXL0i+E8CBPlAeJ4f9q2UREAGICcYCIkTXnjTfeEImQkSoNhCZTjeF3yGOIPiPhK\/qDTDQgU4S7SKT6\/PPPi7FjnOh37969RXiOHJtI3Hvy5Em69957Rfo1vHf27Fkx9h9++EHkGETSE+CC3I84joffIcRXn3gEuX\/\/flEeWdOB4SeffCI+QtYjfEyQWuoc2EJMkIEVTeKOgSBBHiBIXAugnk1XSU9mw4bh4t8gMpACCAdhnFoWdSKUxPlwpOfHY80aI3u1cOFC+vrrrwXZyraR1OHIkSOCNBMRJIgFWXFkhqZjx44RPEkQFcgO96Wo2ZtkqA7Sg7eMpCX4G\/fGoH8g1cOHD4vkEkjjBc8R\/UCWpAEDBpBMH4c+Ybwg4pkzZ4owXGYNl6nBrr\/++lhyZjlWOw9SzpPu3btXtAVcraExE2RIDey\/3WaCDKn8kCkaXguyAYEocE8PiAlhsUp6CNtApLi4CVmkYcDIBCNT2Ktl8W94gdYHFyZZ59\/izRWqddkRpBqixluISUQoIMXJkyeL0BseZPny5WnPnj3097\/\/XRCkelUFFj0QcsOrkxdUqRmGQKrIX2l94NVa+xWPINUkzokSKjBBhtTAmCDDLbgtW7ZQ8eLFBenBM0QeRtwkCJJ4++23Y6vY8ApxuyC8NnheVmNWSS2eV2i9EkKihmshdu\/eLTwnpACDNwbvDf2B92dHkDJHIbxemZAW857Ipt62bVvC\/KK6iq3WA68P\/UdoDK9VzS5vJUhMF8B7hIeNqQQ8wAfhPMJoZNZWvVj0\/\/jx45QvX75seUbjESQ8zdatW4t64UHiAwIPVrYlcbISJOSBS6jirezzHGTwbJI9yODJRKtH8BzhPSH9W+7cuYXXCNLEbYgwQniJ8AaRDxAECfJCgmEQBBYt4HUh9AQpyrKY41PnFZHpHIsPDzzwQLY5SLwDokI4jsWSzZs3i\/+DoEF+iUJshPHw3iShIjQH8YFkQOSJCBLEh3YQEoPk0M5zzz1HR48ezeJBwsPGNAIuPkMYjTAe\/8fKNsgJ\/ZXzoGgPC1j4ObxOnTlI1I+xyxyL6D\/IWE5NSCFiCkCG8ricDTKRfbDe1cMEqaX6nhZigvQUbnONyVVskB0MUL1YCiu8CItBVCBMLGTIlVx4fBs2bBBzjfDIfv7551hZEARWgxFmgnjUlWlroli0idVfzLuBmNUVZZRNRJDWHIXt2rUTJIxQOFFIijGD5OTqNt7BzXdYgOnZs6e4jEq9DRJkKstjLMAIXifGCU9R5nxE\/52uYtevX58++OCD2KVe8i5zEGWPHj0E2aNOZIwHoUMmWOVGdm\/ZBxCiWpYJ0px9mKqJCdIUklxPoBCIFzIjxMY9yyBNJ9d1JJuDNDVwJkhTSJqrhwnSHJZcU4AQkKvS2NeIBRV5Idp1110n9oc6eZggnaAVrbL\/D9LG9QetNLlVAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
