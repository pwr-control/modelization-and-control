clear;
[model, options] = init_environment('single_phase_cllc');

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
t_measure = 2;
tc_factor = 200; % tc is ts_afe / tc_factor
tc_decimation = 20;
delay_pwm = 0;
dead_time_afe = 0e-6;
dead_time_inv = 0e-6;
dead_time_dab = 1e-6;
dead_time_cllc = 1e-6;

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
use_torque_curve = 1; % for wind application
use_speed_control = 1-use_torque_curve; %
use_mtpa = 1; %
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

% single phase inverter
rpi_enable = 0; % use RPI otherwise DQ PI
system_identification_enable = 0;
use_current_controller_from_ccaller_mod1 = 1;
use_phase_shift_filter_from_ccaller_mod1 = 1;
use_sogi_from_ccaller_mod1 = 1;

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
%[text] ### Single phase inverter control
iph_grid_pu_ref_1 = 1/3;
iph_grid_pu_ref_2 = 1/3.;
iph_grid_pu_ref_3 = 1/3;
time_step_ref_1 = 0.025;
time_step_ref_2 = 0.5;
time_step_ref_3 = 1;
%[text] ### Setting global behavioural (system identification versus normal functioning) and operative frequency
if system_identification_enable
    frequency_set = 300;
else
    frequency_set = 50;
end
%[text] ### Settings average filters
mavarage_filter_frequency_base_order = 2; % 2 means 100Hz, 1 means 50Hz
dmavg_filter_enable_time = 0.025;
%%
%[text] ### Grid Emulator Settings
grid_nominal_power = 1600e3;
application_voltage = 690;

if application_voltage == 690
    % trafo data
    us1 = 690; us2 = 690; fgrid = 50;
    eta = 95; ucc = 6;
    p_iron = 1800;
elseif application_voltage == 480
    % trafo data
    us1 = 480; us2 = 480; fgrid = 60;
    eta = 95; ucc = 4.5;
    p_iron = 1400;
else
    % trafo data
    us1 = 400; us2 = 400; fgrid = 50;
    eta = 95; ucc = 4.5;
    p_iron = 1000;
end

n2 = 14; n1 = floor(n2*sqrt(3));
core_area = 0.05; core_length = 2.5;
mu0 = 4*pi*1e-7; mur = 10e3;

% two simple calculation:
Lm1 = (n1^2 * mu0 * mur * core_area) / core_length;
% Lm1 = u1_nom/sqrt(3)/i1m/(2*pi*fgrid);
i1m = us1/sqrt(3)/Lm1/(2*pi*fgrid);

% rererence for the voltage sequence
up_xi_pu_ref = 1; up_eta_pu_ref = 0; un_xi_pu_ref = 0; un_eta_pu_ref = 0;

grid_emu = grid_three_phase_emulator('Dyn11-690V-690V-1600kW', grid_nominal_power, application_voltage, us1, us2, fgrid, ...
                eta, ucc, i1m, p_iron, n1, n2, core_area, core_length, mur, ...
                up_xi_pu_ref, up_eta_pu_ref, un_xi_pu_ref, un_eta_pu_ref);
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
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:3aa5d6d4]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:9f8d1b23]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:1ee82e17]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:1694fb26]

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
enable_frt_1 = 0; % faults generated from abc
enable_frt_2 = 0; % faults generated from xi_eta_pos and xi_eta_neg
start_time_LVRT = 0.75;
asymmetric_error_type = 1;
frt_data = frt_settings(test_index, test_subindex, asymmetric_error_type, enable_frt_1, enable_frt_2, start_time_LVRT);
grid_fault_generator;


%[text] #### DClink Lstray model (partial loop inductance)
parasitic_dclink_data; %[output:9e1e8aaa]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:3ff93b24]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:8a64fcac]
n_sys = psm.number_of_systems;

% load
b = psm.load_friction_m;
% external_load_inertia = 6*psm.Jm_m;
external_load_inertia = 1;

%[text] ### Motor Voltage to Udc Scaling
u_psm_scale = 2/3*hwdata.inv.udc_nom/psm.ubez;
u_im_scale = 2/3*hwdata.inv.udc_nom/im.ubez;

u_psm_scale_ekf = sqrt(3)/2 * 2/3 * hwdata.inv.udc_nom/psm.ubez;
u_im_scale_ekf = (2/3)^2 * hwdata.inv.udc_nom/im.ubez;
%[text] ## 
%[text] ## **CONTROL Settings and Initialization**
psm_ctrl = ctrl_pmsm_setup(glb_time.ts_inv, psm.omega_bez, u_psm_scale, psm.Jm_norm);
% psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, psm.Jm_norm, glb_time.ts_inv);
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:0c12a94d]

im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:7e8491f9] %[output:5d43af41]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:7e8491f9] %[output:5d43af41]

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

%[text] ### Sogi
sogi_delta = 0.05;
omega_set = 2*pi*frequency_set;
sogi = sogi_filter(omega_set, sogi_delta, glb_time.ts_afe); %[output:3b7b0d5c]

% gain for active sogi
kepsilon = 2;
%[text] #### Resonant PI
kp_rpi = 0.25;
ki_rpi = 45;
delta_rpi = 0.05;
res_nom = s/(s^2 + 2*delta_rpi*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta_rpi*omega_set] %[output:20857e0b]
Aresd_nom = eye(2) + Ares_nom*glb_time.ts_inv %[output:4afdc36a]
a11d = 1 %[output:29f5bd82]
a12d = glb_time.ts_inv %[output:7eec76c6]
a21d = -omega_set^2*glb_time.ts_inv %[output:38ad0aad]
a22d = 1 -2*delta_rpi*omega_set*glb_time.ts_inv %[output:0257cae7]

Bres = [0; 1];
Cres = [0 1];
Bresd = Bres*glb_time.ts_inv;
Cresd = Cres;
%[text] ### Current control parameters DQ PI
kp_inv = 0.5;
ki_inv = 45;
pi_ctrl = kp_inv + ki_inv/s;
pid_ctrl = c2d(pi_ctrl, glb_time.ts_inv);
plant = 1/(s*grid_emu.trafo.Ld1 + 1);
plantd = c2d(plant, glb_time.ts_inv);

G = sogi.fltd.alpha * pid_ctrl * plantd;
figure; margin(G, options);  %[output:6945820b]
grid on %[output:6945820b]
%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] ### Diode rectifier
Vf_diode_rectifier = 0.35;
Rdon_diode_rectifier = 3.5e-3;
%[text] ### HeatSink settings
heatsink_liquid_2kW;
%[text] ### DEVICES settings (IGBT)
% infineon_FF650R17IE4D_B2;
% infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
% infineon_FF1800R23IE7;
% infineon_FF900R12IE4
used_device = 'infineon_FF900R12IE4';

igbt.inv = device_igbt_setup(used_device, glb_time.fPWM_INV, hwdata.inv.udc_nom);
igbt.afe = device_igbt_setup(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
igbt.dab = device_igbt_setup(used_device, glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
igbt.cllc = device_igbt_setup(used_device, glb_time.fPWM_CLLC, hwdata.cllc.udc1_nom);
%[text] ### DEVICES settings (MOSFET)

% wolfspeed_CAB760M12HM3
% infineon_FF1000UXTR23T2M1;
used_device = 'danfoss_SKM1700MB20R4S2I4';

mosfet.inv = device_mosfet_setup(used_device, glb_time.fPWM_INV, hwdata.inv.udc_nom);
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
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, glb_time.ts_cllc); %[output:308b735d]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, glb_time.ts_cllc); %[output:3ae562e1]
%[text] ### Load

trafo_load_name = 'Load Single Phase Transformer';
trafo_load_pwr_nom = 225e3;
trafo_load_u1_nom = 400;
trafo_load_n1 = 50;
trafo_load_n2 = 1;
trafo_load_u2_nom = trafo_load_u1_nom/trafo_load_n1*trafo_load_n2;
trafo_load_f_nom = 50;
trafo_load_eta = 98;
trafo_load_ucc = 5;
trafo_load_i1m = 10;
trafo_load_p_iron = 2e3;
output_transformer = single_phase_transformer_setup(trafo_load_name, trafo_load_pwr_nom, trafo_load_u1_nom, ...
    trafo_load_u2_nom, trafo_load_n1, trafo_load_n2, trafo_load_f_nom, trafo_load_eta, trafo_load_ucc, ...
    trafo_load_i1m, trafo_load_p_iron);

uload = 2;
rload = uload / output_transformer.i2_nom;
lload = 250e-6 / output_transformer.n12^2;

% rload = 0.86/m12_load_trafo^2;
% lload = 3e-3/m12_load_trafo^2;
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
%   data: {"layout":"onright","rightPanelPercent":28.1}
%---
%[output:3aa5d6d4]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:9f8d1b23]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:1ee82e17]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:1694fb26]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.000000e-05 [H] | Internal Tank Cs: 1.465874e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:9e1e8aaa]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPkAAACWCAYAAAAc7S4fAAAAAXNSR0IArs4c6QAAFqRJREFUeF7tnQm0jdX7x7ep0EAkbkpUhjJUIkqpWEuDFA3GlUKSKYqUJJSxQqVokLD+KZqUEpIQq8gQhYQGCimikEz91+fp917n3s69nVNnX95zvnutu1znvme\/+\/3u\/X2fZz\/7GXL9+eeffzo1ISAEkhaBXCJ50s6tHkwIGAIiuRaCEEhyBETyJJ9gPZ4QEMm1BoRAkiMgkif5BOvxhIBIrjUgBJIcAZE8ySdYjycERHKtASGQ5AiI5Ek+wXo8ISCSaw0IgSRHQCRP8gnW4wkBkVxrQAgkOQIieZJPsB5PCIjkWgNCIMkREMmTfIL1eEJAJNcaEAJJjoBInuQTrMcTAiK51oAQSHIERPIkn2A9nhAQybUGhECSIyCSJ\/kE6\/GEgEiuNSAEkhwBkTzJJ1iPJwREcq0BIZDkCIjkST7BejwhIJJrDQiBJEdAJE\/yCdbjCQGRXGsgAwJr1qxxHTt2dAUKFHC5c+d227Ztc1WqVHEPPvigK1asWExoTZw40Z122mmuZs2a\/3j9J5984tq1a+dKly6dfr+mTZu6W2+91f4\/cuRI17BhQ1emTJl\/7EsXREdAJNfK+BvJx40b53r16mVEP3jwoHvyySeNtI0aNYoJrXhJ\/vLLL7vBgwfb\/f744w83ZswYt3fvXnvZ5M2bN6Z7ZnUR9Tz54YWRqk0kT9WZz+K5keSRJId0Q4YMcZdeeqmrVauWGz9+vHvllVfs20jcli1b2u98Z8KECSbtzzjjDNegQQNXo0YNN3XqVDd69Gi3f\/9+d\/XVV5uEhsxBQ5JHkpzPN23a5Pr06ePuv\/9+++4tt9zi0tLS7GXz0Ucf2QugRYsWdu89e\/a44cOHu9mzZ5vGQd+tWrVyS5YscStXrnTffvut69q1q91\/4MCB9hJhjPRfuHBhu0elSpXce++9504\/\/XRXr149N3bsWLdjxw43YMAAe4awN5E87DOY4PFDckiycePG9J7r16\/vBg0a5BYvXuzmzJljxKA98cQTrmLFikYsPr\/33nvdvn37XM+ePY2ERx99tH3eqVMnlydPHntBnHjiiY7+siP5rl27jGDNmjVzaAWQfOvWrUbYJk2auN9++80kf+fOnd37779v92ncuLH78ccfXe\/evV2PHj2M5PPmzbNx58qVyz3zzDOuefPm9rKYMWOG+\/77720b0L59e9ehQwdXu3Ztu7ZgwYLuzjvvdEuXLnXTpk2zZ\/qv2kSCpyju7kTyuCFL7i9kluSo60haCJY\/f36TbBdffLGBgBSGxDSkfPA5kp49NpK0f\/\/+GQBj\/w1x4iU52sG6devsZ9myZW7u3LnukUcecZMmTbIXSrly5Uwtf+qpp9yVV15pJKfxUqDxYlixYoX76quvTBs488wzXdu2bV2\/fv1MqhcpUsQ0luA5MuMQ5lkXycM8ex7GHm1xB58hsevWrZtuUENSLliwwFThaCRftGiRqcYB0VCzDxw4EJO6jmrNywAJjCT\/\/PPP7QfpfsIJJ5gWQb+BpC9btqyNg8+vvfbaDCT\/6aef3EMPPWTbhWrVqrnVq1e7+fPnRyU52xIMhiK5h8WlLo8MBDIvbqQj0nLt2rWuatWqJp27dOlig4WIkOL33393CxcuNDWe37t16+Zat25tpHvzzTdNWubLl8+kbKQmEGgDmQ1vXIfazP4dtR2Sv\/HGG+kvEsaAYRBJPn36dHuRoK7\/8MMPNgZU9khJzjO9+OKL9h1U78cff9zGH02Si+RHxjrUKDwikPkIDcNWhQoVjDxI0GiGN6Tz888\/b4RGrWbfe9VVV5nU5AWBIYsWGOoi97jZHaHxnYDkGMzYa7O\/Zi+NzaBQoUKmqrMlWLVqlansxx57rGvTpk0GkvPdESNGuClTprjy5cubOs+40BQYW6S6LpJ7XFzqWgj8GwRmzpzpSpUqZQT\/+eef3dChQ02TwMCn9hcC2pNrJYQaga+\/\/tq2A+y7sbKjcVSvXj3Uz5TowYvkiUY0oj\/2px9\/\/LGphhzv0IoXL277xwsvvDCDAcrjMNR1iiMgkntYABirPvvsM7MMX3755WatPf744+1OEB+j0FtvvWX7Sf6uJgR8IiCSe0B3586ddp5buXLlLN0pOX\/mKOess87yMAJ1KQQOISCSe1oNkPi7774z10mOg2bNmuW+\/PJLs1TXqVPH9o9qQiAnEBDJPaDM+fDTTz\/t3n77bXPnhNgQHb9o9uhFixa1yCuOg9SEgG8ERHIPCP\/yyy\/usccec927d7cACqy\/Dz\/8sDltoMoPGzbM\/K45d1YTAr4REMk9IEwMNs4hBDpgaIv0j+b\/RFPhbYW\/tJoQ8I2ASO4BYZHcA6jq8l8jIJL\/a+iy\/iIkJ9wSF0\/258Q6X3bZZXYuTigmlnfCGiXJPYCvLv+GgEjuYVHgK718+XJLUBCtYVknwYEs7B7AV5cieU6sAY7PyCyCU0y0hlWd4IpUTkmUE\/Oge\/yFgCS5h5WAdZ3Eh2Qf+fXXX82v+tRTT7U78TfcWm+77TaLmFITAr4REMk9IYw0Zy8O2TkrD\/bfZCjhWI0kBuzTJc09TYC6TUdAJPewGDgme+GFF8zbjWQL99xzjzvmmGPsTuQv4wwdoxyx2xyvqQkBnwiI5B7QRYoHP2RPIWc4qYdogXvrfffdZ9lSJMk9TIC6zICASO5xQbD\/Jr6Z3GTbt2834mNwO+WUU4z0N9xwQ3p0msdhqOsUR0Ak97AAODpj743b6ksvvWRHZQSlYG2fPHmy3RGikwgRSa8mBHwiIJJ7QBcHGIJTyF+G9EY1D9IRkaKIvGUcoyHpSTCoJgR8IiCSe0SXXOV33XWX1RPbsGGD3YkqHRjhMLiROIIkgmpCwCcCIrlPdJ1zn376qRE5MLKhxvft29dt3rzZ1PebbrrJ8wjUfaojIJJ7XgEQmeQR33zzjRUNZH9esmRJ82kPe\/kdz9Cp+wQhIJInCMisuqEoAPm+yU1O3nAS+1M0ICgp5Pn26l4IyK3V5xrApZWKmzfffLOV70Ftp1QQFUIgfJDc0ecY1LcQkCT3uAawnlPKhyJ75Afnd6pw0rC4K9TUI\/jqOh0BkdzjYsBHnTpe7MWppMmZOKWEqNlFuaHA1dXjENS1EJC67nMNcHQ2atQoc3wZOXKkRaJRmxuSUxNbktwn+uo7QECS3MNaCOLJ8XwbPXq0Vf7E841AlS1btjjqd5PYUXtyD+Cry78hIJJ7WBSR8eRbt261Cpw0jsywslPdk4qaiif3AL66FMlzcg0gyVHXb7zxRpPaEBxnGI7V2rdvr\/RPOTkZKXwvSXIPkx9IctxaKXRYokQJSxxBg+jElJOyGRdXNSHgGwGR3APCwZ6c5BFdu3Y1gxt51mkkkyAdFMUQ8+fP7+Hu6lIIZERAJPe4IpDoOL4QgTZjxgzL3krKJ47WVEHFI\/DqOgMCIrmHBRHEkyOpyeVG0sZy5crZnUjuSAWVBx54wLzfghBUD8NQl0LAEBDJPSyEIJ6cGuUVK1a0mPEgxzqGt1atWll+t3PPPdddf\/31HkagLoXAIQREco+rgb33xIkTzZqOEY5Wvnx516hRI7O4x+oMg\/TnRy05EcBZih9fTST3hez\/cqwH+dcjb8OEosbHUtUUcg994C63Mld5jyNNzq7T9m9Of7Dcu3+233Pv3prhs5MjrolEofjR+7MEpcRRWf8tq+9FfqfbV2lu2W+HjK41atRwjz76qDeii+Qe13fmSipEpZGtFaeYFi1apB+rZTcEUkiN7Xyt61H6J48jTe2uN+YtkSUAm\/IUz\/JvWX2PF0mh\/NGr5+wtWMytz5fmNv+R1\/rFOLt48WI3YcIEV7NmTS8TIZJ7gTXrTtmv4+KKyh6LWyuSHHfYb5fOz+GRJs\/tftz7F6FibQcLnpjh0oMFi6b\/P\/hb5Gf8MfN3Ci4ZE+vtnCR5zFCF40KCU8aNG+c6deoUE8l5KqT5ggULwvGAIR5l8WzUcJ+PVfq8Wt6kOOOWJPc4e0ShEWpK3nWys9LS0tJMMjds2DD9s+yGgCT\/v4c6u0JrZ3ocac52nd2e9r+OJNqe2Of9\/ut4g+8fNXyD9uSJAjMn++EcfMiQIa5ChQpu7ty5VmgBpxiCVpDkRx111D8OJyB5+fXT\/vHaZL0guz1zNNV6U57oe+zs+omG3aa8We\/HE431iz1vEskTDarv\/nBpXb16tTm9kJ0Vd9bu3bubtxv\/4glHPbRYmo7QYkEp4zWH9s5\/7a83\/LIn\/YL1236Pv0Pn3Ppth\/r4Vx1k86Wnm52V6C7T+5O67glaUjG\/++67btq0aVY9BQsqceX79u2zdFAjRoyI+Zzc0xDVbYogIJJ7nGjcW3GEwSFmx44d7vzzz7c87FRQUbZWj8Cr6wwIiOQeFgRnnxD7uuuus7xueLtBeHKwsw8nxDRWZxgPw7MoODLTbNq0ySq8XHTRRT5uoz6PEAREcs8TgeQmQytpmXv16mVZW+++++7D6rNOMslSpUqZiy1+9fjSK0uN54VwGLsXyT2Cv3PnTpOYxJNTQQWrOtFoy5YtsyILHKcdjoYRkNLJ3D\/y98MxFt3TPwIiuUeMKXLYunVrC0ahVBIEx2+diqccp5ExJqcbmWnGjBljtdEJkMHKf8UVV7iyZcvm9FB0vxxCQCT3CDRFDW+\/\/XZTiylZPGjQIDO8LVmyxIorBOGniRrCqlWr3MyZMy0hBQ0XWrzr8IsuWLCg6927t6tevbqp6A0aNHDFihUzwge\/J2oc6ufIQkAk9zgfkIxcbkSb4Z8MqT788EM7SovFbz3WoWG5nzRpklnyyTxDJljanDlz7AetIaiLzt940bAnR3rjR09ttgIFCsR6O10XMgREco8ThpUdF9bZs2ebmn7cccc5ItPwhHvuuedcmTJlEnJ3ottWrlzpVqxYYWQOSI63Xa1atey4Dss+mkTt2rVt24CtAE1D1vWETMER3YlI7nF6kOQkbMQpBiIWLlzY7d6926zZzZs3T3jpYgJZkNyQHI87jukIaa1UqZI9JUUd8KFv0qSJx6dW10caAiK5xxnBHbVbt26uatWqZl0nnxupmTk3Z99M+qdE1ijPTPL+\/ftbgEyQX04k9zjZR3DXIrnHyUEd7tixo\/mosx+nwCH+7JAfgrNXJ1AlUS2S5JHqeaCuo77XrVvXjG9qqYOASO5xrjGItWvXzp1zzjl2Lk1dNAj\/zjvvWAFEJCvpoRLVIkkeGN4WLVrkunTpYnt1SI4qfziO7hL1jOonfgRE8vgxi\/kb7MOHDRtmKZlPPvlkC1LhhwAVDF4cZfkkeXCEhostWgRWdknxmKcvaS4UyT1P5auvvurGjh1rx1UYvdatW2fWbmqW40OeSHXd86Oo+5AiIJJ7nDiCUrBwQ2Z8w9kncz6OukzwykknnZRQw5vHR1HXIUZAJPc4eaR\/Yh8MmXPnzm13gvhr1661FLyxpGT2ODx1nSIIiOQeJ3r79u2uZcuWFnFG9BlHafizY1m\/5JJLPN5ZXQuBQwiI5B5WA9FnuJlyhDZ\/\/nzLrU0MOdFoqOt4pg0dOtQVL55zOcQ8PKa6DAkCIrmHiQqKKkB2oryIJUeC4+1WrVo1Oz7D3VXFDj2Ary7\/hoBI7mlRIMUhM2fX+KxjgGN\/jnW9adOmrn379jK6ecJe3WZEQCT3sCIwrhEAgtS+4IILzPkFZxQywhCcAumDPOwebq8uhUAGBERyDwsCI1vbtm3NAQVfdZxSNm7c6EqWLGn\/j6fgoYfhqcsUQ0Ak9zDhkJzAELzcUM0JEGFvTr51js2Q4oUKFUo\/VvMwBHUpBNIREMk9LAbOx4kXJ8USmVpee+01y7dOoQWSOsRSOcXDsNRliiIgknuYeEjepk0bS9gYrWFVf\/311+1ITU0I+EZAJPeNMGVtDx50VDPFADdlyhSrYIkRTmmQcwB83UJVTX2tgUhik+e8SpUqlrUVrzep675QV7\/REJAk97Au8HAjhps864SZcowWmShRhjcPoKvLLBEQyT0sDkhOnDgZYKI1HaF5AF1diuRaA0IgVRGQJE\/VmddzpwwCInnKTLUeNFUREMlTdeb13CmDgEieMlOtB01VBETyVJ15PXfKICCSp8xUZ\/+gxLrPmDEjQyFGShrfcccdoUboiy++sJx6PAuVZJs1a2Yeh0HjuUuXLh21dBReih988IGXklY5CapInpNoH8H3YrFTbTWSAJHDJdMsP0FCyiP4UdKHRj04EneQZw8X4nhJTkcUwKD09HnnnReGR446RpE8tFOX2IFnRXIKM1AxlfptXbt2tRDZAQMGuC1btpgEhDjEyVMOuU+fPlZznQIOJMag2GK\/fv3s8yJFiliWHAoy9ujRw02dOtUqyhBrT3UZyicT0EMpZfpYvny5Jd2gIET+\/Pnd+PHjjXAk5OjQoYOVnaI\/+qKeHH+nQeigUciCWvDE9kP47EiOA9P06dPtqyT4wEtx4MCBbv369Y7c+YQJh9UdWSRPLFdC2xskf\/bZZ9PHTyw8FV4gybx586zsMeGyw4cPN5KROx5VmJrorVu3tsSUlGAiOSWEI\/0V5IpG8nr16hnZKSxBEg2uJzKvaNGi1s+IESPs\/xRsrF+\/vuXGC6q1klmHrDuo3YyPlwBSmvRa3I8iFkEjhv\/ss892derUSSc5AUKZG88WVHrl5dWrVy9zS6YaLFVw6JvClWlpaaGcX5E8lNOW+EFnJ8m5GySA1JRdxic\/aNR5o6gjNdcgOX75lIFCKmYlyZHCEDiyUTOO7UJAZv6GFkGlGTLeoi5DVlqwdUAVr1y5suW1R8pDeLSAoEU+0z9Jcp4vKIZBymyCiXgWvofmQhKQyBdI4mfAX48iuT9sQ9VzLCRfunSpqdNIUgJuiLRDykL+WbNmuZ49exox+D\/JMlCdo0ly0mJB9EB6Ip0PHDhg6no0kvNZrVq1rLwUDemaL18+20ZwH\/pCg0DqR7Z4SN64cWNL7oGxDQ0jKCktkodqGWuw2SEQC8kp94Qqi+pKSqvZs2cboa+55hqrCENQTqS6TkbaQNVFvR8zZoztdyEs4be8ACAranWNGjWMWNFITpEKctVnrs6Kmt63b19H6mv+zVytlfthN4hU17OyrmNXQP0fPHiwvTSCRt+o8xBf6ro4FGoEYiE5D4i0RdXetWuXqa+ktGL\/zFETBMHgFhjeIMbkyZPdqFGjzPBGgQn23ZCV4hMUgqSRBw+pj8ofjeRsCVDNkdpoCiTcoM46vwcvDvLYZ86Ai+Ft4cKFdgy4Z8+eLA1v2B+4N\/aHIBd+EClI+ekJEybYiy1yKxCmyZa6HqbZCslY16xZY3ty3xVbscxjqMOQF+2IC1UbQyGGwX9bk529PtpAVkeLYZgSkTwMsxSyMeYEycmjh3TFAs7LJCspu3r1ajMEsqWIN9c9+3NeVmgZwR49ZFNhwxXJwzhrGrMQiAMBkTwOsHSpEAgjAiJ5GGdNYxYCcSAgkscBli4VAmFEQCQP46xpzEIgDgRE8jjA0qVCIIwIiORhnDWNWQjEgYBIHgdYulQIhBEBkTyMs6YxC4E4EBDJ4wBLlwqBMCIgkodx1jRmIRAHAiJ5HGDpUiEQRgRE8jDOmsYsBOJAQCSPAyxdKgTCiIBIHsZZ05iFQBwIiORxgKVLhUAYERDJwzhrGrMQiAMBkTwOsHSpEAgjAiJ5GGdNYxYCcSAgkscBli4VAmFEQCQP46xpzEIgDgRE8jjA0qVCIIwIiORhnDWNWQjEgYBIHgdYulQIhBGB\/wcuRnrzz5E\/EwAAAABJRU5ErkJggg==","height":190,"width":315}}
%---
%[output:3ff93b24]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:8a64fcac]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00756 H\nPer-System Quadrature Axis Inductance: 0.00624 H\n---------------------------\n","truncated":false}}
%---
%[output:0c12a94d]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:5d43af41]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:3b7b0d5c]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPkAAACWCAYAAAAc7S4fAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQu01UX1x39XS01LS80SSySflGkiZi5dapqP5RPpgfggX5ivUosIUzTBktIehpqE+WQpWWmilAomZkqZ7wc+yYykqIjCRxYp\/\/WZf\/u47zC\/38yc8\/v9zjn3zqx114Vz58xjz\/7O7Nl7z949gwYNWp5lWbb11ltnP\/jBD7K1116b\/7ZcfvjDH2annXZaKe0+88wz2ZFHHpktXLjQjMse67\/+9a9s3Lhx2U033dQY9zXXXJN95CMfaXkerTTgooGeS9ljLJPmzPvrX\/96NmXKlFwSnHvuudmIESNaIVHhd5uh1d\/\/\/vfs6KOPzh5++OFG265xytz233\/\/bNKkSdlb3vKW4Hk0M67QxqXtd77zndkZZ5yRnXzyyQ2+z2vDtw49c+fOXX7IIYeY7\/sqhw6UemUynA1y2tcA0YT561\/\/aohSNoBi5l5Ut0oGqYrmmi80iAYMGJBdfvnl2aabbloWeVpu59e\/\/nUm\/CyNuYDcqSCXcUHzIUOGNA43m5\/1weY7oHuefvrp5XJK2iC3d\/K8RbVB+JnPfCbbaKONnCe5vQi+AbJQun3a5nTRY5U2Wdw5c+asAHLXwtPOl770pV5MZddzzWPx4sW9CH\/nnXf2Ou30uGzQzZo1y9BEF2HA7373u6YdmyGlDZv2VdO8aMPQdLKZL5RnmqF1iGQm\/cNXG264oZHuXHxrgxzQiATAGv7+97\/vta6uQ0UOE5sHbJq4DinXxiMbKAcVmydFsOk6tPRctRRu99czffr05cJ40pBL\/NWM6QJY3hasQZwn\/vlOBD3o8ePHm2vFtttu2xCzpF35mz7JhVld49OEdm0E+jsyDw1yn\/hUFchDx8rVq1maa7qFSEVV8Iy9ofpArqUMNuhddtmlcarnHWDCAxrkvnV1gdb+jtCsaK3sA07qypj++Mc\/5oJcz1XzsYvfe+RO7jqBtFisF1FAuc466zR2Pz1gPTH5\/Omnn16B4LrNoruRJuqFF16YTZ061dAUsHOX4j7+hz\/8wZzMY8aMaZzk9C13dRegm5mHZjw9Zz1G6WvGjBkrSDN54nqe+Gif5O2guWZglwSkr2dl8kwsyG0JY7PNNmvwp81fRSe5PnQ0mFwbvY8HRELLq+c6WOWzkM0kr129Tg2QS2UWScQWe0F1p2wKAwcObAA3T2yTdkVUzdu9ik5zGxiIR4hhiDTC9IhmRx11VHbiiSd67+Q2MzBnucf55qEZT2+Mrg2rCpDTf+hYW6E5NPFJDDJ\/+wTV16BWeCYW5C7xVT6z+asI5Dbf21INPOe64jZzaAkNtV5JRO8QkGtdmqyXPddeijcmN3z4cOcEaCxPHHIB1BZVOX2LNLW2Mk2fHjbIn3\/+eXNC6o3GHrsGa5HITj0Becg8ihjPZpwqQR4y1lZoboufeWK\/rSCyxeJWeCYG5HkbjV57PbYikNsbvX0oaJDnHQqhV0EZk4xTbzBFSlqXZP3AAw+soPNhHXteeeWV5SLScspqtb1vwUS5VjXD2ZMVUGrFGMR2EV8zp71ryt1dNo2QebQb5LKQIWMtE+R5my7MfMwxx2THH3+8kaDK5JkYkPukDsavgVcXyGWjp38BsN6QoNcBBxxgrpX3339\/L2uFzxKj50w7FFux6wT5eeedl33xi180dsYyxXVhuDLsknLXYgKI6UIc\/q+1kXk7blnium8Xb+Ykt68z9p08RlxvheZ5mltobIulY8eOzU466aTSeSYU5D6ln2xQemP0ade1\/d9eA5vPtEJQt6vpoje\/vCuMjY1mQW7z0AriOvcpl2a1TMWbBkcRM8ni2JPVCjXquBQi9skuRLYZgnpaQeNTIFZ1krto7rJJN6t4i6V5nphr39XlIKiCZ0JBbptYbdOoay6hijfddh6fhYBcH5i2dCnmOlsKihXX9eaj22oo3vQu59sZyzahaeWBfRfk\/67JakIJAX2bgattYf6ie3vRRpK3wHhR+U5yLUIiOdlOHHq8en18ommI2dJHcx8PaJpgqvPVz7PeuNYkBEz6ez5znwuotpQTYkJzab190pxo113z5DOuxyiRxTauHYtCFW96A8k1oeWJ0C5li8tmajOdvh\/kadNdIlQeIVwgt+8jiFeuerabIwQZPXq007TSqoNGiOKNOWq6FilotK3XvoNXTXNZi7wNJc87slmeCXE8ctnJQzy\/XNKbOLC47OS2M0yRI5IP5NBRu1zT31lnnZWdffbZBtwbbLBB9sILL6xwNbYPtzxsuPC4gjPM8uXLje96Km4KhFwnEu3KoUA7aW0rw6r0yS+HWuGt9LQK8v\/+979GLL3ooouy5557Lttuu+2MQ8rQoUOznp6e8JG0sabLkQUnG99dr41D7tquO5XWCeQFLPWzn\/3MiB+8ltlpp52yG264wTwO+f73v59ts802XcOMeXZgJuBzu+2aSXbIQDuR1gnkOcyxdOlSYzrh9OY3J\/e\/\/\/3vDB\/ytdZay7iZvulNb+oQ1krDSBTonxRoSVx\/7LHHsuOOOy6bPHlyr1N7+vTp2bXXXptdeumlGe9iU0kUSBRoHwVaAvkvfvELoyW84oorskGDBjVmgUaWe3noW2Ne2\/CTSqJAf6TAe97znoyfqkpLIMcmx6ltR5SJATngxsPuN7\/5TVVz7Ip2WeQdd9wxW2211cx4\/\/znP2e33nprV4w9DbI1Cmy\/\/fYZnqZVAb3tIBc7LJPEZlhHwVf9ySefzHbfffemdQaxbRTV\/+c\/\/5nNnj07e+tb39pr+rwN2HzzzRuf5bUR87ldl831ggsuMEzWX+kvBPataQydadNVP4\/+IW\/2m8VGSyCfOXNm9u1vf9uI5e9973t7ieso3Wwx3jVIAXmVk2yWOHV87x\/\/+Ed24403mkgklC233DJD10F59dVXzYOPKgv9XnnlldmnP\/1pE80nlXopUAf\/twTyBx98MPvsZz+bXXLJJYY5pSDCY1ojwMOaa65ZSLU6JlnvssX1ds8992S8+6a8\/e1vN6bIm2++2Ty6oSC+277YcT0U12Yj+dOf\/pStv\/76jatCme2ntoopUAf\/twTyJUuWmCANe+21VzZq1ChjQhMXwk022aRhViuaZh2T7FRGQ0zmJBeAc5oCdIr+GzZc\/l9FSSCvgqrhbdbB\/y2BnKlcf\/31RqTktMFPHdMZfsE4w2y88cbe2dYxSXsQZTB2bBtSH086FGoinjM2\/ibisj5VsVxIAeinnnpqhheyfermjcX1uf1Z7Dy8CxpQoYw+Y9sIqe+rE0NnWVdbSrLbqIP\/Wwa5dmtdsGCBiXWOtvyDH\/xgwHK\/EWKozju5bzFDBh7TBqc14vevfvUrZ9MHHnhg9qEPfciAXTMFykEsGFL4O9aIiRMn9ro\/xzBfAnn+tcS3pjF07lMgDwFEiLjOXRRmf\/e7391qk23\/PqC+9957jS+\/iOMuaeLZZ5\/Nrr76au94cQPVZd68ednqq69uPAtXWWWVbN111\/W2kVcBxsVc9653vSsqwUDTHaYvNigA3dn8ORSrPORaPslbXTP9lBGRlbt91QXXW97v4o236qqrBnf30ksvGTMX7rzz58\/PkFxw2\/3b3\/4W1AaAeuSRRwxAiRpig7NoXCjo2BR0QQsvY\/r4xz9u7KzaDOZqz\/6sWVoETTinUhl9xrYRUt9XJ+\/vMZ\/rukhpWDYo\/QLk2GmJpe47ye2TEUVV0Wevv\/66EYO5VrBzAlCAsWjRIgPQlVde2RD55ZdfNr\/5WxmFPilEN+HlFYAmqsspp5ySa6pCacm4oIE4xeixsJnAGPYY6euhhx4y86TQF4+G5N2Abs\/uw9dnGbSw2yijz9g2Qur76uT9PeZzXVfMpyhV+zzIEUe5k3ZbEVDxmwVjE6HI54CLH5SSvs0rdu5a+66\/yxWBjULGwN9kHARmYFPUY\/HdQ2PHlerHUaArFG9xU1qxNpNsF8jFXCWjkv\/zm9MS68Bdd91lopCiBNPAyZs3ADrhhBNMfrCygW33ycaCJYNT3FX4e9G4Bfyf+MQnjJiP9FD1mFvll772\/Y4DOeayadOmmddlkv201aARMsnvfe97xt5O0eK3AM\/1WbMLLqcXdn3E+R\/96EfmFM4DS14\/iMUEx0CX4ANHyInpq5P3d8aOjoEHQ0WKvkcffTR77bXXgjcr5sQV6mMf+5ghgW+Ooevhm2dIO7FthNT31enz2nUUTccee6zxYNMPUloNGlHVTgbjw5T4ZuOBR4kFsTCbnHiEe15vvfVMuz6GsBk1pL6vTiiT2a6yLtCIVl3b60PAZdPk\/e9\/f7bvvvtmK620UvAm4JtnyDhi2wip76sTSn8Zv6u+\/VlV\/K9pGKRdRyM4YcKEjHxmnNwC8jKCRjQ7SUCMmeqOO+5oCcA20xIAY7fddiv15Aph2irqyKmORl\/olNcPzEeSBE56pKpWNkWbplx79ttvv4YysSyJoAqa1d1ms\/wfM84gkHNao9U9+OCDjSebgLyMoBH2JAHv448\/nt13331GhJafmEnZdYWp+E0yhpEjR\/YJEMfSRECvH8TktcE1ifcI3NVFWcda4Kxz1VVXlbIJ6CsAazN48OBs7733NrZ\/\/bfYeXZT\/Y4AOeIc7pRnnnmmCe2EkkxAXkbQCCZJih1bCRa6UGJrxjwFQ5LLbY011igUHUVUbcUBJLaNkPq+Onl\/j\/lc12U9+T+gz7vL63VgjUjpg2uua73EuoDkwMFAeeqpp0KX0ltPrk2sM\/EEkSpZ\/5DXcz7a0rmvTgyd89rTbfDEuO3OMBDxa1\/7monXxkMUTlcN8jKCRgBy7vqA1C5iW+b3Bz7wAeMyW0bxOT2E9BHbRkh9X50Ypwvm0IwzDFYF0kCz1r6CYxAb6g477GDu5PZ7eNf3sfUzLsx8aP5xKMKXINShyDcm\/q43fiQ3NgTGx4MqrB55DlB1078jnGE4qXkT\/s1vftN4h4m5S07yskBO5pAvfOELRoGT7mshbFxeHZ8DCD0Bjttuu63Xo5q8EQjwSSKAo1GzEppIBVg\/MBMiHVCaVZ7mjVdvCGwABBKRMcOLosAtj+JvtETbSFG1OMO4MoccdthhxjtLQi0zNBvkKWhEFUtfb5s+jbJrNIj2nPi33357EPAFNPopbZmzlA1BQmaVpcuxx6h1O\/wb0yJXh1Y2g9ru5P\/5z396uUqi+ECpVpSbC++pPffcsyuDRjTD2PaCx7YRUt9XpwoTjq\/PUDCiu6EtTJahZjnAv88++xgROrbEjlvq4xuB0w\/XAw6pqjYE5gP4ufLye4899jBz5RUhPg280mQctYHcRWAb+NQhNzaiBUncEN+XLVvWlUEjYhnERZ\/YNkLq++p0MshtGokij3fweA3GAJ+4dqJ\/yRP3fbRqZlO22xRR\/YknnjAbAvHZ+KxM\/QHjpF\/CqJWlc7LnHmRCky\/Z4jqfd2PQiNhToy\/XjwVLK7QQ4HO6AZoY4AN2dDby6KbZu34r48\/7rmwG\/ObgQ38hcfpCTMAdD\/JuDBpRxUJ3a5t1gjzvfi+fF\/nhu74L0HFewuuO0knAd41Xbwbyd9KK4XuCA1JHnORVMLLcSeoMGuGzh4bMM7aNkPq+OmXbabF3+\/oMoUVsHV+fnPgCWPK2EzAUk1tI4Xty15eEm2j8fX2K2FwUQKNs+neEnTyEqK3W6aagEXquPpuqTZeQ+r46ddjJW13PkO\/75lnUBlp94txxz4+5GwP0973vfSZ0OHZyl03fN66y6d8RdvKQBWu1jk6uEBI0otX+0vd7UyDETt4NNOP0520F7+lD7\/oAnR+8+Dj16xb3a7WTt3MR6zAhtHN+nd53u+\/kVdIH4HP63nLLLcHAF6DjQ4\/bc9XAr4P\/o7TrVSxIHZO0x10GY8e2EVLfV6ebTGhFvOKbZwifxbah6\/PvuXPnZn\/5y1+iwA\/ghwwZYszHIdF1XGO0P6uD\/xPI\/5dgMISxdJ1WmMwVv00UP0XZTBLI31iBsunPqY\/7LjHxQ5V8jAbg47uPEhOnHr22CeT\/W686drJYAPen+rFg6U+0Ya6A\/+677zZKvtC7vmj4CTEuG0Ee3erg\/355kvc3Ri2ab0p4GM8NAP\/FF1804bZigE9PuLYi7stdv1+BHDs5eZrrKDjw8Lhiiy22yAYOHNhUl7FthNT31cn7e8zndt0XXnjBBPdP9M+Mn3kRX+T9nVMeRx7y\/3HdkrDaIYwF2OfMmWMi+vZZZxgc9mEyHjakkijQlyggtnje3BNyPO+9PUDnhR7JMaoobRfXmRRA5yeVRIH+SAHAXRXAoWdHgLw\/Lmyac6JAXRRIIK+L0qmfRIE2USCBvE2ET90mCtRFgQTyuiid+kkUaBMFEsjbRPjUbaJAXRRIIK+L0qmfRIE2USCBvE2ET90mCtRFgQTyuiid+kkUaBMFEsjbRPjUbaJAXRRIIK+L0qmfRIE2USCBvE2ET90mCtRFga4BOQHuZ8+ebTK2pFIfBYgBd\/HFF5tgCoRS4iHFqFGjGvHP6xtJ\/+yJbCsTJkww2WGJOU8OeWLSSfz5EKp0PMgJW3vdddeZJA677rqreZKXSn0UID4a+eJ5ikqAibPOOsukzyLoZirVU+DCCy80j1cIQME79okTJ5qDbtCgQcGd1wbyyy67zOSAskGqkzMQaZNg+WPGjMmGDh1qImguXbo0mzdvnmE03u0mkAevba+KzdKfDLZbbbWVWRcKiQBIZT1ixIjmBtJPv9Us\/Z955hkTUHLNNdc0KZpIHQ4GYrL\/1gJyAhOQIRXg2iAlWT2ng2RPJaPENddcY5hpm222abAEETR4mJ9AHo+SMuhPr0RBGTduXHbaaadlW2+9dfxA+uk3WqU\/VyboftNNN5nfRx11VOeI69yhyW0+f\/58E3WDTKgapJzSJ510kjkl+M3Jzb1v\/Pjx5rSgrtw9EsjjEVIW\/VmTGTNmZJdffrlZk5133tmsVSrFFCiL\/tILeDn99NOzQw89NCqKTKUn+YIFC0wmSDKkTp061UTH0CAnKdxxxx2XTZ48udepPX369Ozaa6\/NLr30UhMPi5JAHg+pMuhPrzAWqXaPOOKI7G1ve1v8QPrpN1qlPwpP9FHDhg1r3MGnTZuWvfnNb466LkWBnPvAlClTcpds\/\/33zyZNmmTC0+oi4saAAQN6gZxAeGeffXZ2xRVX9FIkAGju5ZwclCOPPDJbuHBhtt9++5m0yc0W0ugeffTRGfm1XMUev8w3b17NjqOO75GCB7Fal4033ticyLI+IfQnPjlpidCoh57e0jciPXf6tddeO2jKmr9c39Xrd+6550YxOgPgfiu8xJWwzJhqWqQ+77zzTCpieDav0P8rr7xSyP8cjFxfueaS35zgkeecc46Zw+DBg4NoSqVSQU6DrsXJAznMwKltM0K7QG6PvxtBrpmtiMFgcB\/92dBRGD300ENG8SNl9OjRJupoXmkG5BqA0q4NxE4GuYwfyfOMM84wOqYikDPHgw46KPvd735XyP\/EfwPYtE9B0841IObgaQrkrg504kJ7ly0L5K3uvppJbP2APvlk\/N0Icr0OQi\/of8IJJ2T3339\/RsJA2YhnzZrl3WRJGFBHcUke9hp1MsiFV+AdsqxoiQF6ozBDkv3c5z7XUKIBWHRPV155ZS9pRx9yNv2b4cnSQK53YhvkV199tdGg6wIDLl682Ig1iOVkm7RPIUImn3nmmUZ0Z1fUILd3fghIO0VMWQRy3Z4wlyYoGs0TTzyxsTu7NjrXdca1MbkY2iV+asDmSUk2AF1MIHTF5+CXv\/ylYTZoRYJA6M8PdNbXGET6H\/\/4xyuIhfbYGTdad059oQlXAtHAh4jret132WUXMyUsKbZU6AK5\/ZmMxSUN2OI6fejrp71WLunCte4yBkzE9hWTNjXI0UnJGkk48Kuuusrwv73e8D93cK47eVdNGXPRWEsDuV58TayiezwKhXvvvTe75JJLDOMV3ZdZNGnXBZI8EU+DoFmQ551ksuD8XUwcrroawEX00CdXXj3fZibf0\/XyJCnyfqP4pMCgdsEWyykjG2cR3flusyC3DwjaEn2C5iUfyPPWSejvAoL9HenPBpyuZ28+UlfmT+ThvJOcq47w+U477WQsT2w0M2fOdOq7ZB3XWWcdJz4YLwUHJVdhrE2BvEh80wytCYVDBTmj9E5GOygQPvWpTxlDv+yq1OPOyMBx45MThgkxWSGgCxRFyp5mxXXGmQdUe0yuenmLL4ofAaaMnRNWFk3a06dd0X0sjzk1\/WX9SARAW9BF+uYER7R\/4IEHjKLHNXbdvwZ+syCXNoShGZ9rjX0g1xubritzQ3KUdjWfaPDLHFDwwo959fRVSDb4olPVxgztYl7m\/r7ZZptlnOYUFNH33XefSfqBZQrbuKa3S1Kz+YeTX8+pdJBrZpHJi5IGkQSQ6wX46Ec\/ahQ7BJ7H5LDllltmL7\/8snGGQRts3zFl17JPtBDNqU+7Ttt6UV3Eo47r3uva+OzvaybTkon93TzFlfTrO819J67rtGLDHD58uDFdIsYiZSHGS1+AntPV7tu1+cSI667va8lIr4cP5EV6lqLNOHQDdV1JtcJNriYhEoMcHJjDuCqhad98882NKZlEI\/A\/0hWbvaZ56J28ZZD7FG8QW4sliCekiRUTmiYqZjFOmfPPP9\/Y0wH2t771LWOXpdjgZXezTUM2SPLMKz6Qh5rQXBtKkVZbGJVx5l1JQkR1Pc8QJWTRmPi+0BIN7uqrr27WCOmCjDZstCJN6LouSclmvBiQ5+ly9HVF5uoDuU0TezPWUqBd1wWeIpFdeEw2VL1+Nn\/kKd7AA74gmNzuuOOOIgG5cVXNA3nRWJs6yV0g1wzFpBg4zIKobe+w9s6JSH788ccbpZYN0KpAbo+p6CS252uPCXFLwKt3XdeJHLIZYB8t8kcokgKKOEUzAvPfaKONnKezS1oRJVXZIPdJHYxF1qpOkMtGVdQ\/r8GQVrFaaKWvT6q0LVG2stC1hrIpuUCuaeiiVWUg5\/XMN77xjRXuFEzAvhvrU98Gn70by+kT62jh6tfnB5+3a+ZtPPaY8sR9vYi2go3FFEDF2EJp0yd22nRHk61Pa+0cYt+TqxDXfZKV0MklCclhUGRWK7rrF53kY8eONW7WHFD60LGlDu7N0M93CNhON3kgD1lvmydZczlg8sZaGsjtUwIA5dnOXaKYTzyT00uLXHpS9oK6TGlFirdWTnLZePQprhnCpVDLMwfa2lKXpaJog3PRUeZm+wLISYRyx6eI0hrjshRvRb4VjNmei5aYXCD30V\/rREJBnneN0qZDn\/SpQa43Yhkv43Jttq5DogjkeWNtCuRFIqGtmAk1GYXs6iEmtCIxvCqQ643HRZuQOznfc2lRXe0VuXSG0FH3U6QkillLPf7QO7nL3KfnqzcBxuw6YUPm69J6++7kol3P43W04myOYhvXh0qo4i1UD1NkgtWSX95YSwV5nrgR6vxhLxjtaSeUvNNPJudTRlUFcnZqW\/Fhn8guU5heFNfmZLfp06rr9vLuui4auYCSJy2U5Qyj+8zjG12HuV9wwQXGxVOL0UXOMDEWGPuEhJba94Ex4tCFiQtwb7DBBhlPSF3rFgJy1zqErLfdNu2IYo9xyWarxxoF8kL1X\/pjv6SATw9QNVGK7uRV990t7SeQd8tKtXmcLp0LQ\/Ldq6sedgK5n8IJ5H4apRqW5r5I7xD6rLQsoiaQ+ymZQO6nUaqRKNDVFEgg7+rlS4NPFPBToCNAjg2Wn1QSBfojBXhXzk9Vpe0gB9yYJYj31hcKgScJWknhN\/\/PC4TP35csWWJ89lPpvxTYfvvtjRt4VUBvO8jRzh522GEmYDyvcIitvuqqq5oVJ7YY\/8YjaLXVVmt8BmhQuOh6K620UkZQBF716MJDi5VXXtm0RYwyfnjxQ6IAghLyxDW28B0BsgZwM21J37vttpt5cvjkk09mu+++u3NjwLPO9feYz+26vHjC\/gyTYfuto+SNN6bv2DZC6vvqxNCZubjq59Hf5+MRQxu7bkeAHJe+KifZCoFa\/S7A14Hw+f\/rr79uQMUTW10+\/OEPG1fOOguPIwgMQfojHqykUi8FxARZJf8nkNe7piv0BuhHjhzZ+Nz35LDs4SLREDxi\/fXXb0hLZfeR2sunQAJ5P+EODfS999671iwxCeTtZbIE8oroXwZjx7bhq8+rPeLdUQh86VLC5LUR87ld1zeuKpagjD5j2wip76sTQ2fo5qpvf9YVIPclLPQxSR2TtMfgW0zfmPMWsOh7vj65n5966qmmCQDPiR467hjmSyDPv5b41iiGzn0K5KEJC\/MA0A6Qh4C47jrtEtl9jF03Hfpbf3Xwf0uKt5iEhZ0E8jIYO7aNkPoEtaTk3curOElCxlU28MroM7aNkPq+OlXQv+NBHpOwsAjkRKskFFGsMwABCCmSowtbuBTs19jOly1bZn5TB1u7\/JuE7tjJ5UEFtnR++B5BIHBQ0bb1d7zjHaZpvieiGItO7miyfuoi49KfUZfTmvp2rjipR7hgsbX\/\/Oc\/d4rrrjby2nZ9bn8WMq4qQO6jha\/P2HGH1PfViaGz8Ig9T90Gfh3EhyMWYsea0EIS5vnS7LCTYRsm42l\/KISephCVlh+KfEZI37vvvtt8Rny8ddddt3KSsEER3YQcXuJcVHmnqQNDAWKt46NA6ViQ+xLm+dIWMbn+BnIXfwNyck4TVJ+A+xRop51oqsIFQR8WLVpk+hKvwqr6Su32pgCn\/I033mg8Dvs8yOv2ePPdvUKY0dWGiPKI6\/Jv2uIa8Nvf\/tbkHkNUx8vMVXDrlVDMJDawpZsq7oRl0CKEXrpOGX3GthFS31enCvp3\/J2c\/E06YaEsJAPHDGTnHXcxQx2TtPv1LWYI08a2kbcpkICQnGSadtQltt3hhx\/eayhVMFnsPEJo46tTRp+xbYTU99Wpgv518H9L2nWYk4clJCwkvZEUco5jWiP2us5r3Skg9zFlLTeyAAAKIUlEQVRh3X8X\/3H6RYTjEQqnOBto1cXH2FX339\/b73iQ80ySdL577bVXNmrUKKPBlsB+m2yyiQlQL5rvvMWsY5LdwEjcy0TEnzNnjhkyecmqvpcnkLeXO+rg\/5ZOcshz\/fXXmywTiOeEhpWEeZKw0EfCOibZqeK6HhcebyhhKPwbwNv38irExXaAvIw+Y9sIqe+rUwX96+B\/A3Ls3ZMmTcoIKG8H4nv00UfNs0gGg5mFnGWf\/OQnG+YWJj5hwoTsJz\/5ibFJI56TkJB0xL5THIauY5LdAHItsgvI7dO8CibzMbZvk27m72X0GdtGSH1fnSroXwf\/97z66qvLv\/Od7xiwSepVWTgY79hjj82GDBli8i09++yz2Ve\/+lVj7iF5PSBObq3NsLn7O0TIoXAfZ02kILIjIVVRfIxdRZ+pzTcoUAvIt9pqq+Ukm3dlzEBxds8992QXXXRRJh5fZGkg3SqmHtLdcu\/ebrvtGvdvnCvGjx+frbXWWkaEzwt9JNOsY5LdwlRyL8cEt8MOO3ifnOr7uvybd+GsC5595IMfPHiwWQMi5mDKo1BXPOsAOc4wpIrGTq6j21StD+iWdalynHXwf89dd921HDMOdlx9kr\/00kvZ5z\/\/eZM7HCBLwdaL+yVeajAjJ\/rkyZNN8nQpaNc5edgMEPGLSh2T7AZxnTFyJ5doMaS5AXB4RN1yyy1V8lnHtO3bVOy\/o+Rl82IzW2WVVUzEHQ4jvCzXWGMNc\/Xk72x48CrBMcS9WTY7PXmfVNO14jp3cpfnGgRBRCet8EEHHdSghQSzP\/jggw2AETFtezjAHTNmTK+czXmcJCA\/+eSTswMPPLBybTLjYLE60Xf65ptvzh5\/\/HFDKnJs2T7wMm7ciR955JEGSfFYo2gHnI5Bbh8bCK7GIp3qTYdNhXcJO+64Y4ZlCZywXsQs5DuE1uLfeDcK79Xqu+4CuSRWO\/\/88zOdelWDnPXj1Lbv8s2AnLaIM4Yprr8WFv+2224z04dZxLe9bHoI49Guy3edv0sR\/\/mYzzhRUcLi3YdECM\/ww8nL5zz+IZgm9XgERKBO\/bjIni9\/dxU9prJpVFd7xGNgM0AatvOYlzUGo13vBJCjwd92221rOcnLIl4V7UggR0C+0047VdFFrzaT7\/r\/OyBJkVeIZCzFDwTHJAmbzWdS9HdaidIrkiXm0tpBvmDBAnP3JlrJvvvu25icnOScuNyDkltrWABE331PFltAjoad64suVdwJQ8ZV9k5TRp+xbYTU99WJpT\/WKXmuLI9\/aOO5554zyUQWLlxoHiVRry0nuQSE2HXXXY0ftRQUb9jKEeMp3ejW6lvMEKaObSOkPnWwZCDiUlC+JZC7VyOEniG0i6kTC3JXffuzOhTPueI6OxAmNJxheP5IgAUKoj1aYJiRktxaQ7aE8Dra8w1lpCsARXhr\/pqxYPG3mGrEUKCtIGeg8+fPN84w3A3JcgLgCWYwduzYbPjw4WYu3ejWGrMIddfVnm9oZFFGVlkSyKukrr\/ttoOcIWq3VuyRnNwHHHBAw4ygo7Vyj0d5QDgbnCtCSh2TtMdRBmPHthFSX+pgIpMEkJzkAJ3fVYiLIeMKWceYOmX0GdtGSH1fnSroXwf\/9xx++OHL6QgngT333DMbN25cr5xYPt\/1FJK5XMWbZDPhWmQHlwDo0JvfbKK4G3N\/h\/kkhJO23YbcCX2MHQPe0Lpl9BnbRkh9X52uBfmXv\/zl5cccc4wx1KMpx4SAeyWeQ8l3PZRtq6mHS\/GsWbOqaTyw1VCdQEg9PNA4TOCx1157zdjRccElcCZeaTiK8Ju\/w3+2c48dcYcpyGch\/QdOudZqtZzkS5cuXS5KtSeeeCI74ogjzNNRsmwm3\/Va19vZmTA6G+7DDz9sGD95tpW3Lq7NQX\/Gv\/H5502AxMGTNwB602l2k6kF5DjDCMm0Nxu28Tp919Ekk6e5joLIe\/vtt2dbbLFFNnDgwKa6jG0jpL6vTt7f+RyLx9ChQ818cJek4F125513ms95MISHGaZR5r7hhhua58J4nj311FPm\/\/h5c8KGbiISZTaPgL6\/N0X4Lv2S3gSgOxuFvOsgJDnxFyp1hhG6yUnOvZwO6\/BdR8GEoo482akkCsRQgE1EfArsDYXP5TOctrgaAC7Axv8Bmf0d1\/+l\/ZhxxdYV5Wps3oHQfhqRYXDNY0dBLGRX4RTA461q33UGCtBFmxw68FQvUaCvUABwVwVwaNSzbNmy5T\/96U\/N01GUHfiQ87a8rgcqfWWh0jwSBTqVAj2jR49eTuDAkSNHmnfjck+oy3e9UwmTxpUo0Fco0DNs2LDlX\/nKV0xwCB2TrS7f9b5CyDSPRIFOpUDPokWLlq+33norjK8u3\/VOJUwaV6JAX6FAz+LFi99IBfq\/WaGJ5KcO3\/W+Qsg0j0SBTqVAz6BBg1YAOc4wI0aMMGOu2ne9UwmTxpUo0Fco0HJyhb5CiDSPRIG+SoEE8r66smleiQL\/o0ACeWKFRIE+ToEE8j6+wGl6iQIJ5IkHEgX6OAW6BuQ8npk9e7YJHJlKfRQgZPPFF1+c3XrrrSZGu8TG96W\/qm+EfbsngoGQUJSXgtCc2A86MlPI7Dse5DybvO6660wsOSLHkl8tlfooQIomsrrwFJjIKESQPeSQQ0yM\/FSqpwAxHXi8QnhungBPnDjRHHSDBg0K7rw2kF922WUmRJENUl\/4KNxr582bZxiN6DUJ5MFr26tis\/QnOw4uzyS1pPBCkbfp4kfR3Gj637eapT8PxUi\/REpwXorykAwM+PLGaQrXAnIyT5xyyikmeIEN0tDUx0TQIABCAnk8QMqgP73yDJlYA+Sf56ViKmEUaJX+XJmgOxmF+U0ehJjrUqUg5w5NzHbcYzmxSeKnQSqPYEJSHyeQhzGUrlUW\/bmLz5gxwySwZP123nnnXo+Z4kfWP75RFv2FWuDl9NNPzw499NCoKDKVgpznqs8\/\/7xJcDd16tSM1D8a5I899lhw6uME8nhglEF\/eoWxiA5L\/D+JBxg\/mv73jVbpj8ITfdSwYcMad\/Bp06aZMF0x16VKQS7LKuLGgAEDeoGc+OKhqY8TyJsHSSv0nzt3rsnnRe47\/RS5+dH0v282S38OxhtuuMFcc\/fYY4\/sxRdfzM455xwTsWnw4MHBhGwryF3ZVBm5K\/VxAnnwmq5QMY\/JfPSfMmVKhsKI1E0ofqSQs36fffZpfkD97JvN0p\/rEfHfADYKOArJTaB9zIbbNSDvZ3xR6nRbYbJNN9201LH0x8baTf+2gnzmzJktpz7uj0wTO+c8Jkv0j6Vkc\/XbTf+2gvzBBx9sOfVxc2TvX9\/KY7JE\/3r4oN30byvIlyxZ0nLq43qWqbt7yWOyRP961rXd9G8ryCFxq6mP61mm7u4lj8kS\/etZ13bTv+0gbzX1cT3L1N29FDFZon\/1a9tu+tcC8urJmHpIFEgUyKPA\/wF320TGC4WOUAAAAABJRU5ErkJggg==","height":190,"width":315}}
%---
%[output:20857e0b]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.0001"],["-9.8696","-0.0031"]]}}
%---
%[output:4afdc36a]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.0000","0.0001"],["-12.3370","0.9961"]]}}
%---
%[output:29f5bd82]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"1"}}
%---
%[output:7eec76c6]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"1.2500e-04"}}
%---
%[output:38ad0aad]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"-12.3370"}}
%---
%[output:0257cae7]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"0.9961"}}
%---
%[output:6945820b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPkAAACWCAYAAAAc7S4fAAAAAXNSR0IArs4c6QAAHm1JREFUeF7tnQm41cMbx4eUVBSS9tKinfZoQ5ZUslM3Qpv2BWlRtEhFJVnaCyGtShstVFq1akdFllIShSzZ\/s\/n9Z\/rd0+\/e+85tzNnub3zPPepe+7vzMzvO\/Od9513Zr5zxj\/\/\/POP0aQI\/B+B3bt3m1dffdX06tXLnHPOOeb33383Tz\/9tLnqqqtMjRo1zKRJk8yUKVPk6caNG5v77rtP\/s93Jk+ebC666CJTtGhR07BhQ1OtWjWzYMECM378ePPnn3+a+vXrmwceeEDytWnt2rXmzTffNIMHD078\/JtvvjF9+vQxjz32mHz3\/vvvN3ny5DHPP\/+8WbFihTlx4oS55557pOzffvvNDB8+3CxbtsxcdtllkkezZs3Mpk2bzM6dO82+fftMly5dpPyBAwfK+1BH8s+RI4eUUbZsWfPOO++YIkWKmBtuuMG88sor5tixY+app56Sd4j3dIaSPN6bMLz1h+SQ5MCBA4kZN2jQwAwaNMhs3LjRLF++XIhBGjFihClTpowQi8+7d+9u\/vjjD9OzZ08h4dlnny2fd+jQwWTIkEEGiJw5cxryS4nkx48fF4IlJCSYqVOnCsmPHDkihG3UqJH56aefZFDo2LGjWbx4sZRz9913m0OHDpnHH3\/cdOvWTUi+cuVKqfcZZ5xhRo8ebZo0aSKDxaJFi8zXX39tbr31VtO2bVvTrl07U7t2bXk2S5YsplOnTmbz5s3m3XfflXc666yzwgtyhHNTkkcY8FgvLtCS\/\/3332JpIVjmzJnFstWsWVNeAysMiUlYefs5lr5w4cJiSQcMGJDklVu3bi3ECZXkeAd79+6Vny1btpgPPvjAPPPMM2batGkyoFx66aUGp\/TFF180N954o5CcxKBAYmDYsWOH+fTTT8UbKFasmGnVqpXp16+fWPULLrhAPBb7HoE4xHq7pVQ\/JXk8t56Duvt1bvsZFvvaa681V1xxhZSMpfzwww\/FFfYj+YYNG8Q1tkTDzf7rr7+CctdxrRkMsMBY8m3btskP1v38888XL4J8raUvXry41IPPb7755iQkP3z4sOnfv79MFypXrmw++eQTs2rVKl+SMy3h\/ZTkDjqXZhkbCAR2bqwj1nLPnj2mYsWKYp07d+4slYWIkOLXX38169atEzee\/z\/yyCOmefPmQrpZs2aJtcyYMaNYWa8nYL0B75ycOTPP4TYzf8dth+RvvfVW4kBCHYgZYMkXLlwoAwnu+v79+6UOuOxeS847vfzyy\/IdXO\/nnntO6u9nyZXksdEPtRYOEYAQ7du3F2t75plnSmCrZMmSQh4sqF\/gDes8btw4ITRuNfPeevXqidVkgCCQRbKBOu8cF5cfFx73nvK+\/\/57eQ6CkyzJIT9zbebXzKWJGWTPnl1cdaYEu3btEpc9W7ZspkWLFklIzndfeOEFM3fuXFOiRAlx56kXngJ187rrSnKHnUuzVgTSgsCSJUtMwYIFheDfffedGTZsmHgSBPg0\/YuAzsm1J8Q1Ap999plMB5h3E2XH46hSpUpcv1O4K68kDzeimp8iEGMIKMljrEG0OopAuBFQkocbUc1PEYgxBJTkMdYgWh1FINwIKMnDjajmpwjEGAJK8hhrEK2OIhBuBJTk4UZU81MEYgwBJXmMNYhWRxEINwJK8nAjqvkpAjGGgJI8xhpEq6MIhBsBJXm4EdX8FIEYQ0BJHmMNotVRBMKNgJI83IhqfopAjCEQdyRH\/wt5IUQDP\/\/8c3PhhRea66+\/Xs5A58uXL+LwcvoJ8QQEC5FHQlABgUHOTCOawPlr1EpIKJmgH8aZ58CEOAOSRmiXoVxyySWXmIcffljUTDhDnVJCqMGeu0YhBRmjMWPGJPkKaiecu0asMLk0f\/58+ZNXg83vWdRYkIPyyjj5Pcf5bQQeUnoOoQfqzpFRzrC3adMmUeyR9+K8N+ow\/J936Nu3b4rvEGwH+Pnnn0UYEgHK8847T7TebLv98MMPcv78jTfekOw4s46eHOfpAxPn7znnjoAF2NvEuXjOtfPuVknHr27o5tGP77jjjlTb2e\/7nMdHu86b4IT3feKK5DQMB\/whE6Dnzp3b8NnMmTNFIQTFj1y5cgXbzqf8HGIEyAohRACB6Ry9e\/c2TZs2FcUUiM8PdaYjUT\/+pSMHEhfRA45JPvnkk6Z06dJCDhROGEBKlSoVMskRYbCySww2EBNts6FDh\/oOMpCNjk2n9OvM3gqkRHIGKwYA5JUgJ6qtyZEc\/CBt+fLlzV133SVijbwzgwzfY4D86KOP5LNzzz3XzJgxQySnGAj9BspgG5Q6UjcGaM6eI3qBagxCF\/ygTINUFQKUYMcZdTDh98B2O1WSgwH501YIboSa\/NRuv\/rqK9O1a1cRqKQfxhXJsQz8AAqNbhNAzZ4925QrV04IwotjAVASRdI3b9680pirV6+W0RmBAfLg31NJEBM5YCynJQaDDsqkpB49ephbbrnF1KlTR35HAXTIkCHmpZdeOolIqKqQH0qndCTeic5dqVKlRLLauiKuiNWls1Ney5YtDQ2LnJG15F6S8z3yRo2UgQbRwsAEoTiPbQcGSE\/+KJsyqGJpGFjxMqzlgIheKWXyhBxz5syR+kDIqlWrJktynnniiSdkIENNhoSX9sUXX4iMMoPebbfdligQyeDx6KOPyvO8JxaQd0KOGYKAN6qrXiJCQp6nve17Iy6Bl4QFRo6ZBJ7vv\/++1JWBGo\/L\/o33mD59+knvyveCITn9DKuOAKVN9EkkqXgPxDAZFKlPqMqwfiSnDPqk7QNxQ3I6Dx2CUR95oJSSlRSiQ9Por7\/+uhk5cqS4zQgK4KLRkSBRqKB6y6VjYB0ZcKzcMIMJQoI\/\/vijDCx0VOvG2REWt9nr2pEn7igWxVqogwcPiutvR2Nvubwf1garX6hQIemAvB8Dmh\/JyZsOBXHBEDJ7E3WFIBDLqp4yGDH9QTiR7+ONINYI6YJ11+lopNTcelsXLCsCEFh12g2FVQaYTJkyySOQgffG42AgBV\/ko7FWtCcDJGVVqFAh8fX8SO73GQMwA9aoUaOSDIJMDxkgIIzVmPdiFwzJve46ngH1Z6CxbYHOPO+NV4N3GkoKJDleCkIaDIb0n7iy5HQ0OmL16tUTrQ2djYa1ycr98uITJ040zz77rJCG37HgNGLWrFnl90BBf5uH33yWv11++eVmwoQJSToA5WMl+A6i\/DQW4N57771CtsC5GvO0QOL7NSgWCvcRWSOsitc1pREhIH+zgx0kZTChUZObkzMXp56IMQa6nJTHwEO+WDvKsARjEDx69Kh0dC4vsAqpwczJgyW5bVu8NDwf3gVxRpvwXJizQwTwY6BhGgTpGejsoGUFIBGRtMmP0LQ\/dfO2p99zti9gGOg7xEkCk59OvfcZYkeW5ODKlAOvzTu1xBujb2PtvQNUMGRPV3Ny3Fcr7eNnySE1loCRPJDEqf0eDJh+z0ByO0dk8CBZdxPX2c+SMwjgYvp1GOb0ND55Qti6deueZHUtIbCwtvP4Bd687jqdC81xBh2CXIEdafv27dKJCURZkhMEpJPzXbwFsEUR1QXJLbbW49i6dWti7AAZZgYgBrqHHnpIpmAMUoEDvHegB\/tA99j799tvv\/0kFz65qRSWd968eeL9YeUDLW0olpx2xSjQxt6pol+bBtsnUzJYNo+4cdepMC4x0cjAeaCdvxJVjCTJaTTqZD0GS3LcYrwK5n24ePbSAToS1oZ5PAE4b7JBIDoxAR47aAQ2Np0O60Uwzg52DA5YfkiQ3JzczxOyeQeSnGkFgwxBRTsv9c7xwuWugwfzdwYfe3WSN3bARQh4avxcd911ouZqE9bQBuXslMvGQ7zXMPlZaDwu2gbPwJLNTr0YHLDw1MnO4W3bML0LnGYFS3I7\/aJf2BiNd3DDknsH7tOW5N9++63MG6+88kpZZmEujFtJkI35KAGhUyV5KO46jU8UE7Jhdek8eBvUEWuJVvjSpUvFepJsJNlvbgdxCGrx\/dTiBLipLJGxbEMAJ7U5OdYYHXIsol+0PtBdtyRnHk4wk4GV+SP657xrYJAwuQ6ZmrvOVUUEuBhQmIP\/8ssvEjehvnwGqQjuBZKC8pguQAy8JaSfGVgZ6Cz2KbnrDJQEP5Gb5nnmx3hYEJDpDJ8x\/SI2wLVPtA268n5R\/WBIzkAJlvnz55fgZWD7Mu\/nXVmVSYu7ntzUMy4tOZWGWFb7myUXNMGJLhcoUEDWmWn01NzzYFycYEdS7zovEV6sKev2NkJO0A83mBS4BmxjDLjBuPAMVIGJz23E2\/4tMLqO5aezeaPrgevkeAgMSIHRZ\/LEAjK4PPjgg2K5IRlzXgYE\/ka9WSbk3cj34osvFkLgTQR6Vd76+5E8cD68fv168RiYTjBo2yg+RKRdcde9yRuV5m\/gQ57UL9joOvkRxyAAFhgwhYAErhiQyZd2hPDk7bcPIxiSU17gWjaf2fm6veARz4JVBgYVpn2BMSC\/PhlMX44rdz1Y4ulzoSNAZyf5eRmh56bfCAWBwCU0Bjguf2CV5lT2A8StJQ8FPH02eASYM2LZsPapbYYJPld9MjUE7NXQzMftfJ9YDx4UN72EI6klDweKmociEMMIKMljuHG0aopAOBBQkocDRc1DEYhhBJTkMdw4WjVFIBwIKMnDgaLmoQjEMAJK8hhuHK2aIhAOBJTk4UBR81AEYhgBJXkMN45WTREIBwJK8nCgqHkoAjGMgJI8hhtHq6YIhAMBJXk4UEwmD453rlmzRvYhHzp0SJ7icAcHUjhJ5z0S6bAamvVpjoCS3EEH4BQXZ50RC7zmmmtE3MGeH4f4HPt8++23RQWUv2tSBFwioCR3gC6HC\/bu3Stnsb1CB96iOC7K+fHUlFgdVE+zPM0QUJI7anBIjLhgjhw5TJYsWUQJ9OOPP5bz74ggBIopOqqGZqsIxJckc7y0l1UeQdoIVVGIDdERH2COjkwVogupXZoQL++r9YxtBNSSO2gfNNe4xICz2UgMofiBLhsKpLjyaMIldyOHg+polqc5AkpyBx0A6eVx48aJfhmBNkiOxhfCgPyOHBRSTX6XHDiojmZ5miOgJHfQAQJJjnY62nMornCBAvLRSnIHwGuWvggoyR10DEiO8B\/BNZROUQOF4ETaUf8kKIdyKvI+yUXfHVRLszxNEVCSO2h4bhxBqZTIOhK\/fpteUOhE7xxXXpMi4BIBJbkDdImuE3yzFx8GFkFUPXv27PKxWnIHDaBZJkFASe6gQ0BwLiPg0gLWyomw23Vx3HWi7NyXhhZ34E0qDqqjWZ7mCCjJHXYAbgRp3769qVatmlwawHZX7tTirq8777xT\/uXmDE2KgEsElOQO0eUmDu7cGjt2rMmVK5eUxFVP3FTC1UnsX+fmEk2KgEsElOQO0SXKjkter169xGtyZs6cafjhLi7usGL9XJMi4BIBJblLdI0xK1askMv0uJgRdz1z5sxyfxh3cfE7l+ppUgRcIqAkd4AuV99A6pw5cwqRCb4RhOOebwJwXJxHBJ6lNp7RpAi4REBJ7gBdCMzhFM6UQ+INGzZIKdx5PWzYMFOrVi25erd8+fLm9ttvd1ADzVIR+A8BJbnD3kDgjUMq7Hjjzm9SsWLF5Ire4cOHi2XXpAi4RkBJ7hBh1suJrKMAM2TIEAmyZcqUSf7fuXNnPaDiEHvNWi15RPoA83EOo3DyjDXxtm3bmvfee0+Cb23atDFceK9JEXCNgFpyxwgThFu0aJF56623ZOdbgwYNZGOMijg6Bl6zT0RASe6gM3DK7NixYxJZ90t277ruW3cAvmZ5EgJKcgedwu5d\/\/rrr82RI0fMgQMHZOksT5485uDBgyYhIUHm5NmyZXNQumapCCRFQEnusEew4eX+++83zZo1k7k4+9SXL19uXnvtNTNp0iQ9nOIQe81aA28R6QPIMqMIQzT9xRdflOg6rnzLli1lvbxo0aIRqYcWcnojoJbcYfuz640jpZxCY9cba+bz5s0zCxYsMFOmTBHVVk2KgGsElOSOEUb9pUOHDob5OYE2NsIg8ohijCZFIBIIKMkdoOzdu072RNmx6og4oghjo+rshNO96w4aQLNMgoCS3EGHOHz4sJwZx3oju8zmF0ts5uRE1fPnzy9uvO5dd9AAmqWS3HUfsOvkCEQg6Lhjxw4D8Ukso1WpUkUi7Tond90Smj8IqCV32A\/s3nXm5FmzZpWSjh8\/LpF2LD0yzZoUAdcIKMkdIsyVSP3795cls0svvVRKwoXnBhUuXFARR4fgRyHrN9cfNCv3\/GC+\/P43Kb1J1TwmoUruKNQkaZFKcsdNsH79elkfz5gxY6IlR9+N+bim9IPA0ws\/N08v3HfSC0HylxJKRfVFleQRgB8RCSLpzNWJpnPcNJSEZcADOJV05i\/fncrX9bspIPDVD7+Z9m9+nOwTLyWUNAXOz5zs3wnC8uMqKcldIWuMXLDAdUkoxJw4cUL01jmJVqZMGTNw4MCg5uSQG424hXkeDLmmEPvvLMnLSyVH\/DN\/OZJiWfE+YKSEiX3xv7NcaMAh05ergsIi55+HZBUFSS\/aOJSEV8euSFdEV5KH0hohPguxBw0aZAoXLmxWr15tWrRoYV5++WW5Dw21VnvhQkrZWpKv2ns0xNL\/e9yvU9OJ\/ZL\/s+lbhy6lQevPnCVSHCi9GBY+us5k\/vhtOYQUSlKSh4JWjD2LJPPo0aPNfffdl6gMw9VJ7dq1E0serPwTRJ+6fHtY367gBf+5jym5kmEtNJ1l9sXv\/54ifH37H4ZBmMHCzwuqUTSH6V63sLrr6az95XWIro8cOVJuSxkxYoQE4FBt5UIFFGNQbdUU\/wis3HPU3Dxyc7IvMqddBVOzWI6ovai66w6hx5Kjq46gI0IRbG8lyt66dWu5WYXPNKUPBNq\/ucuwhBaYsODd614S1ZdUkjuEn2g696FB7j179hj2tHO8FOmnLFmy6I2mDrEPNWt2JD755JOGK6UfeughU7169VCzMFh0ltJITIdqFjtf18lDRjGOvoB4IwEYoqbcU44aDPNw3HUCcbjx7GvXFBsIzJo1yxQsWNCUKFFCgqMIfaQX5R615I76GJtg5s+fL2fH69atKxYdKSh2viEmAfmV5I7AT0O2EyZMMPXr15ezBd7\/pyGrmPuKktxhk2DJuUSBLayIRFSuXFlIThAOSWYluUPwQ8iaI8AEQlHRpU04W8DAXLx48RByid1HleQO2oZNMFOnThXL8MADD8jlhqyZFyhQQA6ocOx08uTJJnfu6O9rdvD6MZXlrl27zJIlS0zHjh2lXuw+5I548CcuwmlATgXiojds2FA2LEF4+\/+Yepk0VkZJnkbggv0apN62bZu5+OKLpQMtXbrUXHXVVXo4JVgA0\/gcktjTpk0Tvfurr77adO\/eXXJCSJMfljHZasw5Av7G9Io5Odb7jTfekME5vWjjK8nT2ImC\/RpnymfPni0disRuNyLt7IRztY0x2Lql5+fwnnbu3Cln+cHekpzz\/TVq1DA1a9aUVQ\/aoXbt2jKNIrrOFCut0fVYxVNJ7rBl2MfcpEkTw3o5biJnynHbceNZJ7dnzB1W4bTPeu3atWK5ITkrHhz95W66smXLCjbEStiv0KhRo3SLlZLcYdOyCaZLly4SrV2zZo0pVaqUbIZZvHixLNHoXWgOwf9\/1oEkHzBggGjh2\/P9SnL3bZCuS2CDBaownCI7dOiQyD1dfvnlIhrRqlUrja5HoPW9JPe659Zdx32\/9tprJfiWXpNacocti5uOG8i6OOuvrJUTzME9JPquc3KH4PtYcht427Bhg1xTxVwdkuPKp+eVDiW5435mt7ayXLNx40bDkk69evUk0q7JPQJeS05pdgmNQZaYCFH29GzFeWclucN+xik05nxYcq+QAOfIb7vtNlOpUiWdlzvEX7P+FwElucOewDIOZ8mJspcvX17kn3bv3i1WvGTJkrJWzpxdkyLgEgEluUN0uf+Mud\/YsWNNrly5pCSIz7ZJIrxE3Z944gmHNdCsFQG15E77ANF1ouicPitdurSUxZbXoUOHyt51tlIqyZ02gWau7rqbPsBcnC2VbK1ku+SmTZtkuyTBN3ZUsYx2xRVXyCYZddfdtIHm+h8C6q476A32miTWZUkcMWVjDJ+zlMZ6OWeVueFUN8Q4aADNMgkCSnJHHQKLjTAEa7IE2C677DJZI0cdhr3rnCfXa5Icga\/ZKsld9wGIzGEHiI0Us5V\/at68uTlw4IBE2mvVquW6Gpq\/IiAIqCV30BG8UswcW2zatKl55JFHZB87e6bHjRsnsszpRV7IAYSaZRgRUJKHEUybFSSHyFhulstQbCXCjvLI9ddfb4YNGyZLa6oM4wB8zfIkBJTkDjqFJXmnTp3M9u3bRett\/\/79osP+5ZdfmsaNG5u2bdtq0M0B9prlyQgoyR30CkjOlUhbtmzxzZ0lNDbCqCV3AL5mqZY8En2ADS9scknuJlJOnyFeoNH1SLSGlqGW3EEfgOTMuZF6uvvuu03VqlWT6IVx1DR79ux6uYID7DVLddcj1gfY+MI8fN68eQbhfpbTuBOtYsWKId9PHrFKa0HpEgG15BFoVtbJWR+fM2eOmTt3rmxpReNNl9AiAL4WoevkkegDXquuJI8E4lqGFwG15I76g7rrjoDVbENGQEkeMmSpf8EG3jiY0rJlSw28pQ6ZPuEQASW5A3B1Cc0BqJplmhFQkqcZOv2iIhAfCCjJ46OdtJaKQJoRUJKnGbr09UX0xxctWpTkIkau70WmKp4TZwc4v8+79OjRwyQkJMgSpk28d+HChX2vSWKfw3vvvSdXXcWzuIeSPJ57cBjrTmfntlUvAbzZs9bPD9cux0vi7jOEO1DMZU9CqCTnPZHULlGihKlQoUK8vPZJ9VSSx23ThbfiyZGcSwi4HXTfvn1yHp7tuFz3y22tWECIky9fPtGy69Onj0FTnssKkLbiYsF+\/frJ5xzGsRcddOvWzSxYsMCMHz9eLjuw97hzoIfz9+SxdetWU7lyZbn8IHPmzGbSpElCOAQ5OIuPrDX5kRdWlr+TILRNXGaBvh5imhA+JZITLF24cKF8lZtV2IrM8WBODU6fPt107do1bncqKsnDy5W4zQ2SjxkzJrH+efPmFTVZSLJy5Uq54pe9+MOHDxeSca0QrjD3f3NunjPyXDfEPewQDvkryOVH8htuuEFuGkXEMkOGDPJ8zpw5RfuOfF544QX5ncsJGzRoINdL2ZtJuRUW1R3cburHIICV5sAP5SGYaRNn+VHJrVOnTiLJ2YwUmHg3e6spg1evXr3k7AE3nyKhTd6IfqDPF49JSR6PreagzilZcoqDBJCa21hZ\/7eJY7Pt27cXLTtIzuGbTz\/9VKxicpYcKwyBval169YyXbBk5m94EYUKFTKrVq0SdxmykuzUAVe8XLlyommPlYfweAE2ed8pNUvO++ElQGjkuThnwLvwPTwXdPK9A4iDJnCWpZLcGbTxlXEwJN+8ebO401hSRCntPW+Q\/\/333zc9e\/YUYvD7kiVLxHX2s+TcQQbRrfXEOv\/1119y\/t6P5HxWo0YNw02kJKwrV0AzjaAc8sKDwOp7Uygk57TgjBkz5FARHoYNtCnJ46sfa21TQCAYknNZBK4sritadcuWLRNC33TTTaI+yxl6r7uO+o11dXHvJ06cKPNdCMvJPAYAyIpbXa1aNSGWH8m5ZmrHjh0n3USKm963b19R3OHfwJtJKY+4gdddTy66TlwB93\/w4MFJLqMkb9x5iK\/uulIorhEIhuS8INYWV\/v48ePivvbu3Vvmzyw1QRACbjbwBjFmz55tRo0aJYG3IkWKyLyb+S6XT7zyyiuCGXJYWH1cfj+SMyXANcdq4ylwgo87xfm\/HTi4A57fvYnA27p162QZkAsnkwu8EX+gbOIPvAvJCntwQcbkyZNlYPNOBeKpsdVdj6fWipO6cqkjc3LXt8MQmSdQRyDPb4kLV5tAIYHBtN4\/zlwfbyC5pcV4aBIleTy0UpzVMRIkR0cP60oEnMEkOSv7ySefSCCQKUWgpU8NVubnDFZ4GboZJjW09O+KgCIQNQTUkkcNei1YEYgMAkryyOCspSgCUUNASR416LVgRSAyCCjJI4OzlqIIRA0BJXnUoNeCFYHIIKAkjwzOWooiEDUElORRg14LVgQig4CSPDI4aymKQNQQUJJHDXotWBGIDAJK8sjgrKUoAlFDQEkeNei1YEUgMggoySODs5aiCEQNASV51KDXghWByCCgJI8MzlqKIhA1BJTkUYNeC1YEIoOAkjwyOGspikDUEFCSRw16LVgRiAwCSvLI4KylKAJRQ0BJHjXotWBFIDIIKMkjg7OWoghEDQEledSg14IVgcggoCSPDM5aiiIQNQSU5FGDXgtWBCKDwP8A2OLYTQy6FJsAAAAASUVORK5CYII=","height":190,"width":315}}
%---
%[output:308b735d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPkAAACWCAYAAAAc7S4fAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQu4jcX6fxG5RAhb7hEi3VwOFamQvzoUIpIIuac4cjtdD45EKtqScknukWvkcooUdZCiC\/ah7VJyjUgiOc9vnFn\/2bO\/tdasb33f+vb6vneeZz9777Xmm2\/mN\/Ob95133nkn24ULFy4QJ0aAEfAtAtmY5L7tW24YIyAQYJLzQGAEfI4Ak9znHczNYwSY5DwGGAGfI8Ak93kHc\/MYAc9IfurUKZo7dy6dOHHCqBf69u1rlI8zMQKMQEYEPCP5sWPHaNCgQdS8eXPKnz9\/xH45efIkNWnShPuOEWAEbCDgGcl\/\/vlneu2116h3795UqFChiFVH3mh5bLSdH2EEAoGAZySHJP\/nP\/9Jt9xyC91xxx1M4kAMN26kFwh4RvLff\/+d1q5dS\/PmzaMNGzZQtWrV6JFHHhGkv+yyy7zAgt\/JCPgSAc9IrqIJI9yWLVto0aJFtGrVKrr11lupbdu2VLNmTcqTJ48vgedGMQKJQiBLkFxt7NmzZ+mrr76iN998kw4fPkyTJk2iwoULJwoPfg8j4DsEsgzJ\/\/zzT\/rhhx9o5cqVQoVHateuHbVq1YouvfTSLAP8Z599Rg8++CDNnDmT6tSp40i9fvvtN7HTgPTCCy\/Q4sWLady4cTRlyhSqWLGiI+8YOXIkoe6YNI8ePSqWRo899hg98MADjpSvFwKbS+fOnQVGAwcODPsOtV6RJvO0tDTH6yzL\/PHHH437U++rRGuac+bMofXr14txYvpuT0kOYgPo5cuX05IlS8RAuP\/++6lZs2ZUokQJypYtmysDMKsXio5MdpKDvCBPtMHoJcmB8+DBg40JjnHjNcnl+8GPSJOnOsY9Izm2xXr06EG7du0S0vree+8VUit79uyh+qkzLT7s1q2baJiUEmXKlKG9e\/cK9f6GG24Iqfbye3yOJKWulML16tWjdevWie9kmTrx9XfL8nfu3BmS5HgGUh1bgVhetGnThho1aiQkmHw3OgMSGdITeUeMGCGkp6oRoGwpyVu0aCEklky6xqBrEqqEw+SIcuSEqWIiyTRq1Ch68sknQ\/VD++U7QUokWWf0hyQCPgdu6C+pYYTDWX4OPKSmgPe\/8cYbGdq1Z88eQTL1ne+9916GfKjfo48+mgFTfNanT58MbZW46v0oSaFjIvsxHM74XK8z3iExxrjDchKYqTjL\/pHlyu9ga8LEPWTIELGrNHr0aKpUqVKoXciH8po2bSrGuF5vtX2xCgHPSA7r+oEDB6hUqVJ0ySWXZBKucvDKRkvw1E4HKBhwSCAG8soBgQlAqr5ytpaklMRGJ6LzdbVYnURQxv79+0Pl169fPxPJZeWff\/552rRpk5h4oBYjgfCoiyRSNJJHU9d1AskOf\/311+mtt97K9G7UAXXBJGSlrstBi3xqW6HKV69ePaQiy3ybN28WeF1xxRWhtoXDWU5Q+qBUpbdaL3UChZofbomhEg1tk8\/pRJdE0ftDYqK\/Tx2EupSXY0XiLHGQYw94yQleLlHUCVmd0KTgwe6S7BNZF3UCk\/XW2xfrktEzkmOwghTPPvuspWFNHRgYUJIw+C2lkSQy1iZyQEA76N69e0hiqpOFSlB0RLgZMdL6TwVYnzQirUPlc\/GSXEoYSBBgBwyRunTpIjQjdZ0tJRHIpg6ocGtyVWqjnkjqskHFS9dMVJzxnDp5qhJR10ys1HVVQ5CSUK2znHykANAnZblWtepHFRPZf1b2lXDLCF1dx\/9Wtge9zZLksv\/1saL+LwVCuPZBS1S1wkwSUvvAU5Jj1jp9+jTlypUrUz0xiI8fP05XXXUV5ciRQxjjoIpiBn311Vdp2LBhQkLKNZ9Ocr1AvMstkofrOLUOTpJckg0qH9oNtbhs2bKZOl6SFvVLT08Pa3jT1VLUWz6jkhVt6N+\/f4blhxXOOsn1pQ++V7UpXcOQywbki0RyNR\/yghSqDUDHHHlUTCRmTpFcX0JIbFC+JLl8l66pqmttffkky5HtkyQ3Nf56RnIY3XA4JVz0qYULFwr1c+zYsVS+fPmIkhxfYgKAeiMluRUAupoTTZLrMymkv5W6rpNcTj6yXlaSNtyaPJq6jrLUASJJqKqN+jo4kiTXpaIq\/UwkeTiCWC2D9HWmXi+o7roGYLXEUJcRkXYH4pXkal30ZRGwQV+pklyODSttTSe5iSQPt\/uBusyePdt4e9kzkkdTMeJdk1uti3T1LBzJ9bWclbENA1SWp64F1XW+SjydTKraqBreTEiukkXO7upEp9oD8HmkNbm+jrSSdKrBSV+Tm+Csq74q7jC0SSJL45S65pf1V9X1SHYElfTxrMlVLUB93zPPPEP\/+Mc\/RN\/rJJcSWJJT7WOd5OHGmLoml++QNiF13a8aNaNxKcuSXJVYUi3TrevIY2Xh1K2+VjNrpDW51butyrAiuf5uWWddikkLPyYLneSyUyPt36pklAM7nCUZ+88q0bBmlVZ41K9cuXIhKzfWemvWrKEaNWpkMFyirdGs67pGIweijgnK0nc8YNGXyzC5WwKNCNJUxQj\/W1nXw+2SRMIkmgFLXcbIZYPEzorksIqrz6BO2FFQMVY1H932EMm6LtuXdPvk0WagcN+HM7TYLY+fM0Mglq0bO4PRrBb+zGXqPGSn9VlGkv\/xxx9iqwqzWdWqVYUxLmfOnJZtYpLb6erYn1G3LaWUkqp1NFdjNwdt7C3Jek9IDcNqK85pL8QsQXKseR9\/\/HGC3zoGz4ABA8TWDX7jdJppwiSBH06MQDIhAF8R\/LiVPCc5ZjQYM+DxBgku987hWYXwUDBymPjogtzw5Pr888\/dworLZQRcQaB27drC98MtontOctUpBghKkqt\/R1MNkVeqlgCrZMmSrnRGUAuFZXj79u3UoEEDS+\/EoOLiRLtHrDtF6w7koGWt8zh24Emvl+ckh3srpHXDhg3puuuuo6FDhwpPrm+\/\/VacLzeV5NEspU50SFDLOHPmjHBBvvLKKyl37txBhcGVdvea9R3N2viTv0kO5Pbt2yec8nHUFF5uRYoUEYY3bKnAid8kMclNULKXh0luDzeTpwJDcoAhz5NDNSxWrJjwcrM6uBIOOCa5yZCyl4dJbg83k6dA8vnz59PC\/g39q65Hi7+ObTSEg7rpppsiYsYkNxlS9vIwye3hZvJUICQ51uRvv\/22sIrjXDmCOB45coRmzZpFjRs3Fmeb8TfyyAR\/d1jecehCHpxnkpsMKXt5mOT2cDN5qmnqFvp013F\/r8khyXGIvmvXrsK9UiYQeOLEieKQPUivfoegj88995yI7MokNxlK8eVhkseHX6SnA0FyRIjBPjnIDOutTLDmDh8+XFjXz58\/T0WLFhVfHTp0SERiqVWrlrDA6ySH\/zT23Dk5hwBI\/tNPP1FKSoqRz4Jzb\/Z\/Sa0mfed\/SQ7VG0cMv\/jiC+rYsSNVqFBBSG6o5yA9gjnC+w3753B9nTBhglijy7jtOsk7dOhADz\/8sP9HRwJbCKzhboyJNisF1UwgBK69qsa4dFG2r\/fJ0UBY1hGBcvr06XTw4EGxF3vPPfdQy5YtxZlzXL6AO9MQ3QTbbSA+1vD4Xyf5tGnTRLx2Ts4hAK9E9Evx4sV5n9w5WGnvsTNUZ\/SWYJDcCjes1RHFFRcdyhtVrCKY4Dw19tPZ8Obg6NOK4jW5O9hOXLefBi1ICwbJN27cKNblO3bsyIAmvOBeeukly1tPQWorSW4aEsedbvNnqUxy5\/sVUvzGYRtEwdlPH6GlHUv7e58clnIYy7Anvm3bNvH3jBkzhHGtbt26lggzyZ0feOFKZJI7i\/Un\/zlOzcZfVNORCqwcSLPfGutfkqsHVGDggSRGmBuE+4GRDSfLTC5AZHXd2YGolsYkdwZbSG84v2BfXCYY3Jy+kUevrecHVLD2hiRv3769iMz68ssvi8D5iOKKwyrYQ4\/lFBqr684MSCa5czhu++EUtZ+yTRjaZCpTODd9+dTNCbEleU5yNBpr8jFjxohtMsT3wj44LO64MqlXr15GPuwsyZ0blHpJLMljwxZkHrN6D0377OKNNHrqfGtJGtXy4sGrRIzbLEFyFQSQG44X2D\/HPrl6bVIkqBMBVmxd7Z\/cTPLwfQlC\/3nhAj02e3sGNVx\/ApJ78sPXUpHLchH+likR49Zzkoe7SQXOF\/B2g3Rndd3bCYNJTiFVG9L58+9PRCS0qpJPan8t1ShbIGwH+prkVmF6dSQQ0A5baybhnxIBlrdU8+7tQSE5pDKk7Ix\/H6DZG3+ifT+fybCOjtQDUjqPaVWZri6aN4O09loD9VySw3cda\/DevXtToUKFbI9kJrlt6KI+6BeSSxKv+OYopa7dSxcukJFEtlK9SxfKTYObXEWlCuY2JrQV0IkYt56TPOoIM8yQCLAMq+K7bMlAcmm5\/u3ceXrj4\/2Uduh0TJLYisj47NYKBanPnWWocko+V\/o1EePWM5KbqOvqvc\/REE4EWNHq4NfvvSY5CHzyzB80dcOPtP2nX+Mir9pHULEhkW8qnZ+61C0Vl0S22\/eJGLeekdwuKOGeSwRYTtc5Wcpzg+RS8i7Zepg++OaIgCKWNXA07OQaud7VhajZDUVDkli1bEcrIxHfJ2LcZgmS45TT1KlTafLkycLT7ZprrqFOnTqJq2hNjzYmAqxEdHpWfIcJySVpL9AFGrN6L+0+fNpx4qJASVJI4Eop+aj37aUpR\/Zs4l1ZjcAmfZmIces5yXFGPDU1VRwpxWXusKTjb1wUh\/uju3fvTtmyXezESCkRYEWrg5++l6Q99us5mrvpAG3cfVQ4JR04+YexxdkUD5W4eOb+6il0R+XCoceTkbymbU\/EuPWc5JEiw8ClFXvlJlb3RIBl2nFZMZ902liX9jP9O\/0XSj\/6myuSVrZdJe5VRfJQ9TIF6M7\/EdfPpI217xMxbj0n+S+\/\/CIcXvr165fh5hPEYMf9z0zyjMNGbgPtOXaGPtt9nHCiyW3CogYlClwiJDnUZKTbKl5c6+bJmSNpVeVYCelG\/kCQHO6r8+bNo6VLl1KXLl3Eenzv3r1CXUcYZj+r61IlPn32PL2\/7Qh9nPazcJGEAQpJPdDg9ABTJS3+vqVCQWpTszj9cPz3TKQ1WZM7Xb+glBcIkqMz9fBPBQsWFIdTEJLZ9IKFRIAVaeBJCbvz4Gmav+Ugrf\/fcUInLcbh3q+vaWuVu5za1ipOFYvlFRNFvOoxk9y9KScR49ZzdR1rcljVEZyxevXq4nokO8ltsECWr384Ra9\/vM\/RrR69rSphyxfNQ1WK56PGVYt4akFmktsZkWbPuD1uUQvPSS6jrkJlRzx13JwJn3VcfmgqxdEQJ8ECoTfsPk7TPz9gy+0R9VGlJ9ax5a7IQ\/fdWIwaXFM4pIbHK2HNhlH8uZjk8WMYrgQnx224d3hOcrViZ8+eFeGfli1bRqtXrxb75FiTJyoyzLKvj9BDk7dF7FFV0rasnkIdby6RdKSNdcgyyWNFzDx\/IEm+a9cuWrFiBS1ZsoSqVq0qIsPkz58\/KmrxgIVTRzgPbKU6N6lWhHrcVjrudW3UBmThDExy9zonnnFrWivPJbmqriO++l\/+8hdxJxrW6Cbklg21AxbUcgTU08PyLO55U6BJrQ8eJrkpnWLPZ2fcxvoWz0kOwxviuuFes1iJrTY2VrDUkLhyDc3kth4+TPJYaWWeP9Zxa17y\/+f0nOR2Km31TCxg6SFxF\/S4kepXtH+W3ak2ZNVymOTu9Uws49ZuLQJJ8sL9PgrhBeld9+qCdvELxHNMcve6mUkeA7YmYOlrcCa4GcBMcjOc7OQyGbd2ylWf8VySw\/C2fft24c6qHisN93m4BpuApd49hciZ2LfmFB0BJnl0jOzmMBm3dsuWz3lGcpB469at4ppi3ESK64ZVazoixyxYsEDcheZEtFbV0CYD28cLXlCeZ5K719O+JjluTpk7d66Isb5mzRq6\/fbbM0VlrV27Nt18882OnCdvmrol5L2GmyuSxdvMveFlXjKT3ByrWHP6muQSjFjVcrvqujS2ITDfkl43xdoXgc7PJHev+31Ncnn\/eL169YQbK6LB6Onyyy+n1q1bx+3W+vTi\/1Dqmn2ieJbisQ9YJnnsmJk+4WuSSwmOSw5xLzn+1xMMcddff71RnLdwYPFa3HS4hc\/HJI8fQ7saqBNv9szw5kTl1TLCkVx1fOEtM3uoM8nt4WbylK8luQQgUvx1J+KuS4MbW9RNhpx1Hia5feyiPRkIkluBgLhvy5cvp5w5c1Lz5s1tW9dVVZ0NbtGGG6vr9hGy\/2RgSQ7Ifv31V3FHWteuXW1Ha1VJzqq6\/YHIktw+dtGeDDTJca58\/Pjx9Oyzz1KBAuGvfpUgWoGl7o0fG3NHNLz5+zAIMMndGxqBIHm4NfmVV15JTz31FDVp0sQIYR0sVtWNYDPKxCQ3gslWpkCQXCKD0Mz4yZ49O+FWFav4bthbHzt2LP3rX\/+iHDlyiPV6x44dhaecDpZqVR\/YuBwNbHyVrU7gh4iY5O6NgsCQfOPGjTRgwAAaMWIE1alTh+bMmUOLFi2iYcOGUfny5UMIv\/\/++5SWlkZ9+vShc+fO0fDhw0UcuFq1akUkOTvAxDdImeTx4Rfp6UCQHNL5ueeeE3HW4auOe88Qhx1x3j755BN65plnQs4w+\/btE38XK1aM8BxIjoMtVapUyURy3jpzbmAyyZ3DUi8pECTHmhzXJMHApp42C\/c5QBo5cqS4YeXBBx+kwYMHU758+UIkhzZw7733UokhGwSetcteRgu6XedeLwWgZJAcB4lSUlIyHSIKQPNdbeKmTZuEoJo5c6bQYt1Innu8IQwzIrLiPDmkOdbi4SS5CgDcYF955RUqV66ciNMuZ8QOHToI0GqMSxfZh99VhP6v8mVuYBeYMoH14cOHqWjRokYuxoEBxoGG4pj122+\/7W+SAycMIEjzVatWUYUKFQiXHV577bVCYpcuXToE5YQJE4RKjzvSkNatW0fr16+ngQMHhkgO0IqVr0Z1Rm8ReT7rz5FX4x2LuD\/+4MGDVLx4ccqd++KFh5ycQQC2J2ijvpbksKSfPHlSBIyAWgjJDmmO\/\/V7yUFg5IdFHYa3V199VZC+fv36Gdbk2y+UoX7v7hC9wE4w8Q9GXpPHj2G4EgKxJocLK64n\/tvf\/kbYG4+U5BYaJDgmg3bt2gnVHJOCCtbwzZeKABHsr+7M4GSSO4OjVSmBIDnW37gSCWo31tYw7sgESY4z5dg7j5ZUsO6e+5vIzv7q0VAz+55JboaTnVyBIDkuV8A22f79+zNhVKpUKSHlCxWKHhNdgjUydTJ1W3VBlDWqZSXqfGtJO9jzMwoCTHL3hkMgSO4UfBKsXkPHE9R1Xo87hSx7vDmHZOaSfE1ySHCcMnvooYdozJgxjknypn3H0DvfX84kd3BksiR3EEytKF+THGtxGNLy5s0rrOvwW9eTnTX5qboD6I8ilUVRfPLMmcHJJHcGR6tSfE1ytcE4O75w4UKqWbMmVa5cWbizwnqOCxDVCxciQS3BkiRny7pzA5NJ7hyWekmBIDn2vV988UURJOKJJ54QXlXff\/+9OKwCp5fu3bvHFBnm+H2TBI5sWXduYDLJncMykCSHjzr81mFhB8FlOnDggHB3jdW6LknetlZxSm1bxb3eCVDJTHL3OjsQklyeQnv00UeFqi5Tenq6MMjhuKlpZJjWfZ4nqOtIfIbcuYHJJHcOy0BKcjQaa3AcN23UqBHVqFGDdu7cSe+99x717NmTWrRoYYQwZsT7B6XS6eqdRH4+Q24Em1EmJrkRTLYyBUKSS2RwKAURX44ePSpuTMHdaFdffbXRehxlAKw2XfrQL3eNFEWyz7qtMWf5EJPcOSwDK8mdgFDOiHJNzttnTqB6sQwmuXNYMsnjwFIlOW+fxQGkxaNMcmfxVEsLlLoeL4wASxreePssXjQzPs8kdxZPJrlNPFVJziS3CWKYx5jkzuIZOJLDnXXp0qViPxxGNzXZuQsNa3LePnN2UDLJncUzcCTHQZV+\/fqJMMs33nijsTU9nAGDSe78gGSSO4+pLDEQa3JEhoFnGyLDqB5vscKqquu8fRYrepHzM8mdxTNwkhwNxqUJCPmLEMu4DcVOUvfJmeR2EAz\/DJPcWTwDR3In7yeXzjBvNMpGpQtxVFGnhiYOEcFRCWGzy5Yt61SxXA6RiEz85JNP+jtaq1M9jfBRAGvV5S2owMqBThXL5TACriOAiMOjRo0ihDtzI3l+uQIahbjes2bNonnz5onz47jN9JtvvqFWrVrFpL6D6Fax4twAjstkBJxCAOR2i+Coo+ckhyqYmppKJ06cEIdRcP3RkCFDaMGCBYL8jz\/+uOUNp04BzOUwAn5HwHOSYwtt6NCh4hYJXEcs70U7f\/68CBzx9NNPG0Vr9XtHcfsYAbsIeE5ybKFBPcdeOc6NS5JDiuM8OQJKmJwntwsAP8cI+B0Bz0kOgHF2HGty3EaK7bSWLVvSkiVL6K9\/\/au4BFG\/LknvFOk1h2uTsKZHAIpmzZoZXcrg9w6OtX3yXrrvvvuOqlatSn\/\/+9\/FHWhqkj4J8rNu3bqJ++g42UMA28eTJ0+mvn37xmSDMn1bliA5SIpAEXPnzqUvvviCKlasKIheq1YtI6JiQL755ptCC8CEgAAU7du3J7jFcjJHQMbbwxW6d955J61du1b8DBo0iHLlyhUqCPYSXGnl1lW75jVO7py4LRZY4gcT6QsvvOBfkqOrEAYqLS1NBHTEZYdVqlTJMLAideecOXNESOc2bdqIbLNnzxa\/5f\/JPRQSV3s93h6kOs4U6PH3Jk2aJC6arFatWuIq58M3geRbt26l3bt305dffilwtusMFgmeLCHJEf4J6h6uLcaVxWjwoUOHxMwGaR4t4Ypj5IP0QUJ5n376KauQ0YDTvscki90NaEKIzoMJd\/jw4fTII48I7QoJkyn2dDE4d+zYIXCHSzL6jpM9BIA77ijH0siXJIcER+M6deqUQb0GUd99910xyDDgIiWQHDHa69atyyS3N87EUxhsU6ZMEf2RL18+S5IjHv6KFSsEuaFibtmyRSyV0E8md9bFUT3fPup7kmMLDRFZMbAKFy4c6kiojhg4sLxHGzy6eo7\/YYBr3ry5bweGGw3T1XP8H20bE5M0+q9z584hae9G3fxcpu9JDvVv\/vz5Io4YPNxATlyhBCkOBxlYyqNZ12F4mzZtmpgokDA54N5yrOs5mSMAw9tLL70kDGr169cXRjdY0qGO4w54JFiCsU6HwxK8tL799luaMGGCwBy2FE6xI+B7kkc6oKLCBeNEuCS30DDY8HevXr3o7rvvjjo5xN4d\/n9CSnOstzFJQpPCEeBly5aJxgPXjRs3inU7JuNixYoJX4by5cv7HxyXWuh7kmOgQGJbXXioYqqq8i5hzcUyAr5EIEtY1+HdNnXqVOEQgBBQONIIQ1zTpk2NLzz0Ze9woxgBBxDwnOTqARUYb7CFAMmOrRycXTa98NABLLgIRsCXCHhOcljX4QQAQw68qGSK9cJDX\/YON4oRcAABz0mOAypwR8UBlZIlS4aahIgZcIYxvdXUASy4CEbAlwh4TnIY3BAsAmGZu3TpItbje\/fuFep6LPeT+7J3uFGMgAMIeE5ytAEW9vXr19P06dPp4MGDVLBgQXH6rHHjxr4LGAE\/e6QHHnggYvfBiw971U4cAsGSCOf1gS38zqPtVMAQin3vDh06eObggjrgYAxOI8IhJxpesXBBnqKDYdetQyGx1MftvFmC5G43MiuV7wXJsQ+LsNdwdIlGcGAVL8mhneEne\/bstqGXJG\/btq0jE51eEbf3pm033IUHmeQugIoit23bJiQQpAakMVw\/cegGYaeR8B2O08JTD+fg4fEHRxMYID\/88EMheZFmzpwpfPrHjx8v8sKPH3msnH1gx4AEXr16tTgwAnsGfmPX4quvvhLl6JJcfaZmzZoiEk+5cuVEOZUqVRK+7HBYwjtRXySrOqNeeAZOMYsWLaLRo0fTnj17hKMMEk4EwlsOzjVI0dqjkxyTI\/DDkWS0r2HDhplOx+nPRJoomOQuDfygFIvBBYK1bt1a3AqzZs0a+uijj8QAx9lhqa7jcMc777wjvMfgzvvKK68IgkE1leo6jnTCk+\/06dPUs2fPEOGeeOIJYbOQCe8EQRs0aCCWOfBYA+kwuZw7d87ylBP8zqES33XXXXTPPffQxx9\/LAJ4YLcDz6WkpIgYeyA56o7Pjhw5YllnBOlAWSVKlCDUDR6KiOyDtkF7AKlha8HkhhNXJu1BeVKSg+QTJ06k1157TUwk8KlAvXB6UbrcMsmtGcaS3IWZR5IHJ7VwSEYNX6Wq6zjRhR9IQTwDwoMQKskrV65MAwYMED\/yuCcO4IAkcBiS6euvvxYkgFqO8qAug1BwTcW5b6ujjJCKmFiwLkUdcb55+\/btYqLBZ+3atRPPquo7fBes6gySY2KTz+h1xLtQh969e4tJJFp7rCQ5NAOE3cZZhn379onYgKinXIIwyZnkLtA5fJHwAYdau3DhQmFI7N+\/P912220i+o2U5KqfOKQTHIGg2qskh8os1W31bXrIJSwLEEJLNSRB2uHIaPXq1S1JbvUM3qGvydX\/0RaQGZqCWmeQXDXWQRORWgnKlOpxjx49BNGxfIjUHiuSS9zwG1JcxgNkkkce2izJXaA+9v6hmmI7EBFoEUMe6+4XX3yRVq5cGSI5VE+c3MKJOUgnSD\/81iU5TtdhjV66dGnxLAiApAYYgOqP9TZIDkkOT0JIOljo8ZyVJNefOXnypDiIguAbqK+0rqskx1lyqzrrJEfbihQpEorOA01jxowZ4mJLGZ03UnusSI5JEROEnDRQR2guUlOChgEt4b777hOTJa\/JLw5uJrlLJIehCi65CIYoj2NiQC5fvjwUqgrWbgxQqN1QRbG+hHELRioZCAODFYMZ0gpGO0wPO7uZAAACCUlEQVQgkKSYCGSQDDQB22RQZZEHxMbxW0RwQTkgrxXJ8QzeCRUbZcFfARFfoErjWSuSL1682LLOCMKpSvLNmzeLemPJAOkPYoN0WNdjWRGtPVYkxySIdT7W\/ZhEEHcOpFePIgNTaD\/YHoO2AS0IRkB9K5INby4M\/KAVieOYkCoYaFhXg5hYo0N6du3aVajvIBYmA6jNkmggHYxUMHDBgAVyYqIYN26cUP8hHWGQgqFM36KCsQsGMpSHMFp4P94ZaUDLZzZt2iQs1jJ4h0pYVZLnzZvXss4vv\/wyffDBB6GJAZoEliaYZKBZgGx4F8qHITBae6xIjmhBsDVAG4JBDvggoAjegQTs5Dtw\/gHExqQInwu57EEefM4kDxojub2OIwCS4oIMGboL6jpsBph4sJMQLVmRPD093bG4fUzyaD3A3zMCURCANoE1ODSCnDlzCu0EwSdgfzBJTHITlMzy8JrcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGwH8BRL3C1PnZ1KoAAAAASUVORK5CYII=","height":190,"width":315}}
%---
%[output:3ae562e1]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPkAAACWCAYAAAAc7S4fAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQu4jcX6fxG5RAhb7hEi3VwOFamQvzoUIpIIuac4cjtdD45EKtqScknukWvkcooUdZCiC\/ah7VJyjUgiOc9vnFn\/2bO\/tdasb33f+vb6vneeZz9777Xmm2\/mN\/Ob95133nkn24ULFy4QJ0aAEfAtAtmY5L7tW24YIyAQYJLzQGAEfI4Ak9znHczNYwSY5DwGGAGfI8Ak93kHc\/MYAc9IfurUKZo7dy6dOHHCqBf69u1rlI8zMQKMQEYEPCP5sWPHaNCgQdS8eXPKnz9\/xH45efIkNWnShPuOEWAEbCDgGcl\/\/vlneu2116h3795UqFChiFVH3mh5bLSdH2EEAoGAZySHJP\/nP\/9Jt9xyC91xxx1M4kAMN26kFwh4RvLff\/+d1q5dS\/PmzaMNGzZQtWrV6JFHHhGkv+yyy7zAgt\/JCPgSAc9IrqIJI9yWLVto0aJFtGrVKrr11lupbdu2VLNmTcqTJ48vgedGMQKJQiBLkFxt7NmzZ+mrr76iN998kw4fPkyTJk2iwoULJwoPfg8j4DsEsgzJ\/\/zzT\/rhhx9o5cqVQoVHateuHbVq1YouvfTSLAP8Z599Rg8++CDNnDmT6tSp40i9fvvtN7HTgPTCCy\/Q4sWLady4cTRlyhSqWLGiI+8YOXIkoe6YNI8ePSqWRo899hg98MADjpSvFwKbS+fOnQVGAwcODPsOtV6RJvO0tDTH6yzL\/PHHH437U++rRGuac+bMofXr14txYvpuT0kOYgPo5cuX05IlS8RAuP\/++6lZs2ZUokQJypYtmysDMKsXio5MdpKDvCBPtMHoJcmB8+DBg40JjnHjNcnl+8GPSJOnOsY9Izm2xXr06EG7du0S0vree+8VUit79uyh+qkzLT7s1q2baJiUEmXKlKG9e\/cK9f6GG24Iqfbye3yOJKWulML16tWjdevWie9kmTrx9XfL8nfu3BmS5HgGUh1bgVhetGnThho1aiQkmHw3OgMSGdITeUeMGCGkp6oRoGwpyVu0aCEklky6xqBrEqqEw+SIcuSEqWIiyTRq1Ch68sknQ\/VD++U7QUokWWf0hyQCPgdu6C+pYYTDWX4OPKSmgPe\/8cYbGdq1Z88eQTL1ne+9916GfKjfo48+mgFTfNanT58MbZW46v0oSaFjIvsxHM74XK8z3iExxrjDchKYqTjL\/pHlyu9ga8LEPWTIELGrNHr0aKpUqVKoXciH8po2bSrGuF5vtX2xCgHPSA7r+oEDB6hUqVJ0ySWXZBKucvDKRkvw1E4HKBhwSCAG8soBgQlAqr5ytpaklMRGJ6LzdbVYnURQxv79+0Pl169fPxPJZeWff\/552rRpk5h4oBYjgfCoiyRSNJJHU9d1AskOf\/311+mtt97K9G7UAXXBJGSlrstBi3xqW6HKV69ePaQiy3ybN28WeF1xxRWhtoXDWU5Q+qBUpbdaL3UChZofbomhEg1tk8\/pRJdE0ftDYqK\/Tx2EupSXY0XiLHGQYw94yQleLlHUCVmd0KTgwe6S7BNZF3UCk\/XW2xfrktEzkmOwghTPPvuspWFNHRgYUJIw+C2lkSQy1iZyQEA76N69e0hiqpOFSlB0RLgZMdL6TwVYnzQirUPlc\/GSXEoYSBBgBwyRunTpIjQjdZ0tJRHIpg6ocGtyVWqjnkjqskHFS9dMVJzxnDp5qhJR10ys1HVVQ5CSUK2znHykANAnZblWtepHFRPZf1b2lXDLCF1dx\/9Wtge9zZLksv\/1saL+LwVCuPZBS1S1wkwSUvvAU5Jj1jp9+jTlypUrUz0xiI8fP05XXXUV5ciRQxjjoIpiBn311Vdp2LBhQkLKNZ9Ocr1AvMstkofrOLUOTpJckg0qH9oNtbhs2bKZOl6SFvVLT08Pa3jT1VLUWz6jkhVt6N+\/f4blhxXOOsn1pQ++V7UpXcOQywbki0RyNR\/yghSqDUDHHHlUTCRmTpFcX0JIbFC+JLl8l66pqmttffkky5HtkyQ3Nf56RnIY3XA4JVz0qYULFwr1c+zYsVS+fPmIkhxfYgKAeiMluRUAupoTTZLrMymkv5W6rpNcTj6yXlaSNtyaPJq6jrLUASJJqKqN+jo4kiTXpaIq\/UwkeTiCWC2D9HWmXi+o7roGYLXEUJcRkXYH4pXkal30ZRGwQV+pklyODSttTSe5iSQPt\/uBusyePdt4e9kzkkdTMeJdk1uti3T1LBzJ9bWclbENA1SWp64F1XW+SjydTKraqBreTEiukkXO7upEp9oD8HmkNbm+jrSSdKrBSV+Tm+Csq74q7jC0SSJL45S65pf1V9X1SHYElfTxrMlVLUB93zPPPEP\/+Mc\/RN\/rJJcSWJJT7WOd5OHGmLoml++QNiF13a8aNaNxKcuSXJVYUi3TrevIY2Xh1K2+VjNrpDW51butyrAiuf5uWWddikkLPyYLneSyUyPt36pklAM7nCUZ+88q0bBmlVZ41K9cuXIhKzfWemvWrKEaNWpkMFyirdGs67pGIweijgnK0nc8YNGXyzC5WwKNCNJUxQj\/W1nXw+2SRMIkmgFLXcbIZYPEzorksIqrz6BO2FFQMVY1H932EMm6LtuXdPvk0WagcN+HM7TYLY+fM0Mglq0bO4PRrBb+zGXqPGSn9VlGkv\/xxx9iqwqzWdWqVYUxLmfOnJZtYpLb6erYn1G3LaWUkqp1NFdjNwdt7C3Jek9IDcNqK85pL8QsQXKseR9\/\/HGC3zoGz4ABA8TWDX7jdJppwiSBH06MQDIhAF8R\/LiVPCc5ZjQYM+DxBgku987hWYXwUDBymPjogtzw5Pr888\/dworLZQRcQaB27drC98MtontOctUpBghKkqt\/R1MNkVeqlgCrZMmSrnRGUAuFZXj79u3UoEEDS+\/EoOLiRLtHrDtF6w7koGWt8zh24Emvl+ckh3srpHXDhg3puuuuo6FDhwpPrm+\/\/VacLzeV5NEspU50SFDLOHPmjHBBvvLKKyl37txBhcGVdvea9R3N2viTv0kO5Pbt2yec8nHUFF5uRYoUEYY3bKnAid8kMclNULKXh0luDzeTpwJDcoAhz5NDNSxWrJjwcrM6uBIOOCa5yZCyl4dJbg83k6dA8vnz59PC\/g39q65Hi7+ObTSEg7rpppsiYsYkNxlS9vIwye3hZvJUICQ51uRvv\/22sIrjXDmCOB45coRmzZpFjRs3Fmeb8TfyyAR\/d1jecehCHpxnkpsMKXt5mOT2cDN5qmnqFvp013F\/r8khyXGIvmvXrsK9UiYQeOLEieKQPUivfoegj88995yI7MokNxlK8eVhkseHX6SnA0FyRIjBPjnIDOutTLDmDh8+XFjXz58\/T0WLFhVfHTp0SERiqVWrlrDA6ySH\/zT23Dk5hwBI\/tNPP1FKSoqRz4Jzb\/Z\/Sa0mfed\/SQ7VG0cMv\/jiC+rYsSNVqFBBSG6o5yA9gjnC+w3753B9nTBhglijy7jtOsk7dOhADz\/8sP9HRwJbCKzhboyJNisF1UwgBK69qsa4dFG2r\/fJ0UBY1hGBcvr06XTw4EGxF3vPPfdQy5YtxZlzXL6AO9MQ3QTbbSA+1vD4Xyf5tGnTRLx2Ts4hAK9E9Evx4sV5n9w5WGnvsTNUZ\/SWYJDcCjes1RHFFRcdyhtVrCKY4Dw19tPZ8Obg6NOK4jW5O9hOXLefBi1ICwbJN27cKNblO3bsyIAmvOBeeukly1tPQWorSW4aEsedbvNnqUxy5\/sVUvzGYRtEwdlPH6GlHUv7e58clnIYy7Anvm3bNvH3jBkzhHGtbt26lggzyZ0feOFKZJI7i\/Un\/zlOzcZfVNORCqwcSLPfGutfkqsHVGDggSRGmBuE+4GRDSfLTC5AZHXd2YGolsYkdwZbSG84v2BfXCYY3Jy+kUevrecHVLD2hiRv3769iMz68ssvi8D5iOKKwyrYQ4\/lFBqr684MSCa5czhu++EUtZ+yTRjaZCpTODd9+dTNCbEleU5yNBpr8jFjxohtMsT3wj44LO64MqlXr15GPuwsyZ0blHpJLMljwxZkHrN6D0377OKNNHrqfGtJGtXy4sGrRIzbLEFyFQSQG44X2D\/HPrl6bVIkqBMBVmxd7Z\/cTPLwfQlC\/3nhAj02e3sGNVx\/ApJ78sPXUpHLchH+likR49Zzkoe7SQXOF\/B2g3Rndd3bCYNJTiFVG9L58+9PRCS0qpJPan8t1ShbIGwH+prkVmF6dSQQ0A5baybhnxIBlrdU8+7tQSE5pDKk7Ix\/H6DZG3+ifT+fybCOjtQDUjqPaVWZri6aN4O09loD9VySw3cda\/DevXtToUKFbI9kJrlt6KI+6BeSSxKv+OYopa7dSxcukJFEtlK9SxfKTYObXEWlCuY2JrQV0IkYt56TPOoIM8yQCLAMq+K7bMlAcmm5\/u3ceXrj4\/2Uduh0TJLYisj47NYKBanPnWWocko+V\/o1EePWM5KbqOvqvc\/REE4EWNHq4NfvvSY5CHzyzB80dcOPtP2nX+Mir9pHULEhkW8qnZ+61C0Vl0S22\/eJGLeekdwuKOGeSwRYTtc5Wcpzg+RS8i7Zepg++OaIgCKWNXA07OQaud7VhajZDUVDkli1bEcrIxHfJ2LcZgmS45TT1KlTafLkycLT7ZprrqFOnTqJq2hNjzYmAqxEdHpWfIcJySVpL9AFGrN6L+0+fNpx4qJASVJI4Eop+aj37aUpR\/Zs4l1ZjcAmfZmIces5yXFGPDU1VRwpxWXusKTjb1wUh\/uju3fvTtmyXezESCkRYEWrg5++l6Q99us5mrvpAG3cfVQ4JR04+YexxdkUD5W4eOb+6il0R+XCoceTkbymbU\/EuPWc5JEiw8ClFXvlJlb3RIBl2nFZMZ902liX9jP9O\/0XSj\/6myuSVrZdJe5VRfJQ9TIF6M7\/EdfPpI217xMxbj0n+S+\/\/CIcXvr165fh5hPEYMf9z0zyjMNGbgPtOXaGPtt9nHCiyW3CogYlClwiJDnUZKTbKl5c6+bJmSNpVeVYCelG\/kCQHO6r8+bNo6VLl1KXLl3Eenzv3r1CXUcYZj+r61IlPn32PL2\/7Qh9nPazcJGEAQpJPdDg9ABTJS3+vqVCQWpTszj9cPz3TKQ1WZM7Xb+glBcIkqMz9fBPBQsWFIdTEJLZ9IKFRIAVaeBJCbvz4Gmav+Ugrf\/fcUInLcbh3q+vaWuVu5za1ipOFYvlFRNFvOoxk9y9KScR49ZzdR1rcljVEZyxevXq4nokO8ltsECWr384Ra9\/vM\/RrR69rSphyxfNQ1WK56PGVYt4akFmktsZkWbPuD1uUQvPSS6jrkJlRzx13JwJn3VcfmgqxdEQJ8ECoTfsPk7TPz9gy+0R9VGlJ9ax5a7IQ\/fdWIwaXFM4pIbHK2HNhlH8uZjk8WMYrgQnx224d3hOcrViZ8+eFeGfli1bRqtXrxb75FiTJyoyzLKvj9BDk7dF7FFV0rasnkIdby6RdKSNdcgyyWNFzDx\/IEm+a9cuWrFiBS1ZsoSqVq0qIsPkz58\/KmrxgIVTRzgPbKU6N6lWhHrcVjrudW3UBmThDExy9zonnnFrWivPJbmqriO++l\/+8hdxJxrW6Cbklg21AxbUcgTU08PyLO55U6BJrQ8eJrkpnWLPZ2fcxvoWz0kOwxviuuFes1iJrTY2VrDUkLhyDc3kth4+TPJYaWWeP9Zxa17y\/+f0nOR2Km31TCxg6SFxF\/S4kepXtH+W3ak2ZNVymOTu9Uws49ZuLQJJ8sL9PgrhBeld9+qCdvELxHNMcve6mUkeA7YmYOlrcCa4GcBMcjOc7OQyGbd2ylWf8VySw\/C2fft24c6qHisN93m4BpuApd49hciZ2LfmFB0BJnl0jOzmMBm3dsuWz3lGcpB469at4ppi3ESK64ZVazoixyxYsEDcheZEtFbV0CYD28cLXlCeZ5K719O+JjluTpk7d66Isb5mzRq6\/fbbM0VlrV27Nt18882OnCdvmrol5L2GmyuSxdvMveFlXjKT3ByrWHP6muQSjFjVcrvqujS2ITDfkl43xdoXgc7PJHev+31Ncnn\/eL169YQbK6LB6Onyyy+n1q1bx+3W+vTi\/1Dqmn2ieJbisQ9YJnnsmJk+4WuSSwmOSw5xLzn+1xMMcddff71RnLdwYPFa3HS4hc\/HJI8fQ7saqBNv9szw5kTl1TLCkVx1fOEtM3uoM8nt4WbylK8luQQgUvx1J+KuS4MbW9RNhpx1Hia5feyiPRkIkluBgLhvy5cvp5w5c1Lz5s1tW9dVVZ0NbtGGG6vr9hGy\/2RgSQ7Ifv31V3FHWteuXW1Ha1VJzqq6\/YHIktw+dtGeDDTJca58\/Pjx9Oyzz1KBAuGvfpUgWoGl7o0fG3NHNLz5+zAIMMndGxqBIHm4NfmVV15JTz31FDVp0sQIYR0sVtWNYDPKxCQ3gslWpkCQXCKD0Mz4yZ49O+FWFav4bthbHzt2LP3rX\/+iHDlyiPV6x44dhaecDpZqVR\/YuBwNbHyVrU7gh4iY5O6NgsCQfOPGjTRgwAAaMWIE1alTh+bMmUOLFi2iYcOGUfny5UMIv\/\/++5SWlkZ9+vShc+fO0fDhw0UcuFq1akUkOTvAxDdImeTx4Rfp6UCQHNL5ueeeE3HW4auOe88Qhx1x3j755BN65plnQs4w+\/btE38XK1aM8BxIjoMtVapUyURy3jpzbmAyyZ3DUi8pECTHmhzXJMHApp42C\/c5QBo5cqS4YeXBBx+kwYMHU758+UIkhzZw7733UokhGwSetcteRgu6XedeLwWgZJAcB4lSUlIyHSIKQPNdbeKmTZuEoJo5c6bQYt1Innu8IQwzIrLiPDmkOdbi4SS5CgDcYF955RUqV66ciNMuZ8QOHToI0GqMSxfZh99VhP6v8mVuYBeYMoH14cOHqWjRokYuxoEBxoGG4pj122+\/7W+SAycMIEjzVatWUYUKFQiXHV577bVCYpcuXToE5YQJE4RKjzvSkNatW0fr16+ngQMHhkgO0IqVr0Z1Rm8ReT7rz5FX4x2LuD\/+4MGDVLx4ccqd++KFh5ycQQC2J2ijvpbksKSfPHlSBIyAWgjJDmmO\/\/V7yUFg5IdFHYa3V199VZC+fv36Gdbk2y+UoX7v7hC9wE4w8Q9GXpPHj2G4EgKxJocLK64n\/tvf\/kbYG4+U5BYaJDgmg3bt2gnVHJOCCtbwzZeKABHsr+7M4GSSO4OjVSmBIDnW37gSCWo31tYw7sgESY4z5dg7j5ZUsO6e+5vIzv7q0VAz+55JboaTnVyBIDkuV8A22f79+zNhVKpUKSHlCxWKHhNdgjUydTJ1W3VBlDWqZSXqfGtJO9jzMwoCTHL3hkMgSO4UfBKsXkPHE9R1Xo87hSx7vDmHZOaSfE1ySHCcMnvooYdozJgxjknypn3H0DvfX84kd3BksiR3EEytKF+THGtxGNLy5s0rrOvwW9eTnTX5qboD6I8ilUVRfPLMmcHJJHcGR6tSfE1ytcE4O75w4UKqWbMmVa5cWbizwnqOCxDVCxciQS3BkiRny7pzA5NJ7hyWekmBIDn2vV988UURJOKJJ54QXlXff\/+9OKwCp5fu3bvHFBnm+H2TBI5sWXduYDLJncMykCSHjzr81mFhB8FlOnDggHB3jdW6LknetlZxSm1bxb3eCVDJTHL3OjsQklyeQnv00UeFqi5Tenq6MMjhuKlpZJjWfZ4nqOtIfIbcuYHJJHcOy0BKcjQaa3AcN23UqBHVqFGDdu7cSe+99x717NmTWrRoYYQwZsT7B6XS6eqdRH4+Q24Em1EmJrkRTLYyBUKSS2RwKAURX44ePSpuTMHdaFdffbXRehxlAKw2XfrQL3eNFEWyz7qtMWf5EJPcOSwDK8mdgFDOiHJNzttnTqB6sQwmuXNYMsnjwFIlOW+fxQGkxaNMcmfxVEsLlLoeL4wASxreePssXjQzPs8kdxZPJrlNPFVJziS3CWKYx5jkzuIZOJLDnXXp0qViPxxGNzXZuQsNa3LePnN2UDLJncUzcCTHQZV+\/fqJMMs33nijsTU9nAGDSe78gGSSO4+pLDEQa3JEhoFnGyLDqB5vscKqquu8fRYrepHzM8mdxTNwkhwNxqUJCPmLEMu4DcVOUvfJmeR2EAz\/DJPcWTwDR3In7yeXzjBvNMpGpQtxVFGnhiYOEcFRCWGzy5Yt61SxXA6RiEz85JNP+jtaq1M9jfBRAGvV5S2owMqBThXL5TACriOAiMOjRo0ihDtzI3l+uQIahbjes2bNonnz5onz47jN9JtvvqFWrVrFpL6D6Fax4twAjstkBJxCAOR2i+Coo+ckhyqYmppKJ06cEIdRcP3RkCFDaMGCBYL8jz\/+uOUNp04BzOUwAn5HwHOSYwtt6NCh4hYJXEcs70U7f\/68CBzx9NNPG0Vr9XtHcfsYAbsIeE5ybKFBPcdeOc6NS5JDiuM8OQJKmJwntwsAP8cI+B0Bz0kOgHF2HGty3EaK7bSWLVvSkiVL6K9\/\/au4BFG\/LknvFOk1h2uTsKZHAIpmzZoZXcrg9w6OtX3yXrrvvvuOqlatSn\/\/+9\/FHWhqkj4J8rNu3bqJ++g42UMA28eTJ0+mvn37xmSDMn1bliA5SIpAEXPnzqUvvviCKlasKIheq1YtI6JiQL755ptCC8CEgAAU7du3J7jFcjJHQMbbwxW6d955J61du1b8DBo0iHLlyhUqCPYSXGnl1lW75jVO7py4LRZY4gcT6QsvvOBfkqOrEAYqLS1NBHTEZYdVqlTJMLAideecOXNESOc2bdqIbLNnzxa\/5f\/JPRQSV3s93h6kOs4U6PH3Jk2aJC6arFatWuIq58M3geRbt26l3bt305dffilwtusMFgmeLCHJEf4J6h6uLcaVxWjwoUOHxMwGaR4t4Ypj5IP0QUJ5n376KauQ0YDTvscki90NaEKIzoMJd\/jw4fTII48I7QoJkyn2dDE4d+zYIXCHSzL6jpM9BIA77ijH0siXJIcER+M6deqUQb0GUd99910xyDDgIiWQHDHa69atyyS3N87EUxhsU6ZMEf2RL18+S5IjHv6KFSsEuaFibtmyRSyV0E8md9bFUT3fPup7kmMLDRFZMbAKFy4c6kiojhg4sLxHGzy6eo7\/YYBr3ry5bweGGw3T1XP8H20bE5M0+q9z584hae9G3fxcpu9JDvVv\/vz5Io4YPNxATlyhBCkOBxlYyqNZ12F4mzZtmpgokDA54N5yrOs5mSMAw9tLL70kDGr169cXRjdY0qGO4w54JFiCsU6HwxK8tL799luaMGGCwBy2FE6xI+B7kkc6oKLCBeNEuCS30DDY8HevXr3o7rvvjjo5xN4d\/n9CSnOstzFJQpPCEeBly5aJxgPXjRs3inU7JuNixYoJX4by5cv7HxyXWuh7kmOgQGJbXXioYqqq8i5hzcUyAr5EIEtY1+HdNnXqVOEQgBBQONIIQ1zTpk2NLzz0Ze9woxgBBxDwnOTqARUYb7CFAMmOrRycXTa98NABLLgIRsCXCHhOcljX4QQAQw68qGSK9cJDX\/YON4oRcAABz0mOAypwR8UBlZIlS4aahIgZcIYxvdXUASy4CEbAlwh4TnIY3BAsAmGZu3TpItbje\/fuFep6LPeT+7J3uFGMgAMIeE5ytAEW9vXr19P06dPp4MGDVLBgQXH6rHHjxr4LGAE\/e6QHHnggYvfBiw971U4cAsGSCOf1gS38zqPtVMAQin3vDh06eObggjrgYAxOI8IhJxpesXBBnqKDYdetQyGx1MftvFmC5G43MiuV7wXJsQ+LsNdwdIlGcGAVL8mhneEne\/bstqGXJG\/btq0jE51eEbf3pm033IUHmeQugIoit23bJiQQpAakMVw\/cegGYaeR8B2O08JTD+fg4fEHRxMYID\/88EMheZFmzpwpfPrHjx8v8sKPH3msnH1gx4AEXr16tTgwAnsGfmPX4quvvhLl6JJcfaZmzZoiEk+5cuVEOZUqVRK+7HBYwjtRXySrOqNeeAZOMYsWLaLRo0fTnj17hKMMEk4EwlsOzjVI0dqjkxyTI\/DDkWS0r2HDhplOx+nPRJoomOQuDfygFIvBBYK1bt1a3AqzZs0a+uijj8QAx9lhqa7jcMc777wjvMfgzvvKK68IgkE1leo6jnTCk+\/06dPUs2fPEOGeeOIJYbOQCe8EQRs0aCCWOfBYA+kwuZw7d87ylBP8zqES33XXXXTPPffQxx9\/LAJ4YLcDz6WkpIgYeyA56o7Pjhw5YllnBOlAWSVKlCDUDR6KiOyDtkF7AKlha8HkhhNXJu1BeVKSg+QTJ06k1157TUwk8KlAvXB6UbrcMsmtGcaS3IWZR5IHJ7VwSEYNX6Wq6zjRhR9IQTwDwoMQKskrV65MAwYMED\/yuCcO4IAkcBiS6euvvxYkgFqO8qAug1BwTcW5b6ujjJCKmFiwLkUdcb55+\/btYqLBZ+3atRPPquo7fBes6gySY2KTz+h1xLtQh969e4tJJFp7rCQ5NAOE3cZZhn379onYgKinXIIwyZnkLtA5fJHwAYdau3DhQmFI7N+\/P912220i+o2U5KqfOKQTHIGg2qskh8os1W31bXrIJSwLEEJLNSRB2uHIaPXq1S1JbvUM3qGvydX\/0RaQGZqCWmeQXDXWQRORWgnKlOpxjx49BNGxfIjUHiuSS9zwG1JcxgNkkkce2izJXaA+9v6hmmI7EBFoEUMe6+4XX3yRVq5cGSI5VE+c3MKJOUgnSD\/81iU5TtdhjV66dGnxLAiApAYYgOqP9TZIDkkOT0JIOljo8ZyVJNefOXnypDiIguAbqK+0rqskx1lyqzrrJEfbihQpEorOA01jxowZ4mJLGZ03UnusSI5JEROEnDRQR2guUlOChgEt4b777hOTJa\/JLw5uJrlLJIehCi65CIYoj2NiQC5fvjwUqgrWbgxQqN1QRbG+hHELRioZCAODFYMZ0gpGO0wPO7uZAAACCUlEQVQgkKSYCGSQDDQB22RQZZEHxMbxW0RwQTkgrxXJ8QzeCRUbZcFfARFfoErjWSuSL1682LLOCMKpSvLNmzeLemPJAOkPYoN0WNdjWRGtPVYkxySIdT7W\/ZhEEHcOpFePIgNTaD\/YHoO2AS0IRkB9K5INby4M\/KAVieOYkCoYaFhXg5hYo0N6du3aVajvIBYmA6jNkmggHYxUMHDBgAVyYqIYN26cUP8hHWGQgqFM36KCsQsGMpSHMFp4P94ZaUDLZzZt2iQs1jJ4h0pYVZLnzZvXss4vv\/wyffDBB6GJAZoEliaYZKBZgGx4F8qHITBae6xIjmhBsDVAG4JBDvggoAjegQTs5Dtw\/gHExqQInwu57EEefM4kDxojub2OIwCS4oIMGboL6jpsBph4sJMQLVmRPD093bG4fUzyaD3A3zMCURCANoE1ODSCnDlzCu0EwSdgfzBJTHITlMzy8JrcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGAJPcDCfOxQgkLQJM8qTtOq44I2CGwH8BRL3C1PnZ1KoAAAAASUVORK5CYII=","height":190,"width":315}}
%---
