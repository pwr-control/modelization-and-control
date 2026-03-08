clear;
[model, options] = init_environment('frt_dqpll');

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
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:98cafe20]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:14f1bf82]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:2afa06f8]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:854ddeb9]

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
enable_frt_2 = 1; % faults generated from xi_eta_pos and xi_eta_neg
start_time_LVRT = 0.75;
asymmetric_error_type = 1;
frt_data = frt_settings(test_index, test_subindex, asymmetric_error_type, enable_frt_1, enable_frt_2, start_time_LVRT);
grid_fault_generator;


%[text] #### DClink Lstray model (partial loop inductance)
parasitic_dclink_data; %[output:8549903d]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:87087136]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:6f969853]
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
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:4ee4822a]

im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:022fe42d] %[output:7e4e4179]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:022fe42d] %[output:7e4e4179]

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
sogi = sogi_filter(omega_set, sogi_delta, glb_time.ts_afe);

% gain for active sogi
kepsilon = 2;
%[text] #### Resonant PI
kp_rpi = 0.25;
ki_rpi = 45;
delta_rpi = 0.05;
res_nom = s/(s^2 + 2*delta_rpi*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta_rpi*omega_set] %[output:416a8410]
Aresd_nom = eye(2) + Ares_nom*glb_time.ts_inv %[output:9e0d4f6d]
a11d = 1 %[output:7721d2a0]
a12d = glb_time.ts_inv %[output:0942af15]
a21d = -omega_set^2*glb_time.ts_inv %[output:6c05179c]
a22d = 1 -2*delta_rpi*omega_set*glb_time.ts_inv %[output:7fe8c576]

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
figure; margin(G, options); 
grid on
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
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, glb_time.ts_dab);
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, glb_time.ts_dab);
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
Simulink.importExternalCTypes(model,'Names',{'phase_shift_flt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sogi_flt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'linear_double_integrator_observer_output_t'});

%[text] ### Remove Scopes Opening Automatically
% open_scopes = find_system(model, 'BlockType', 'Scope');
% for i = 1:length(open_scopes)
%     set_param(open_scopes{i}, 'Open', 'off');
% end

%[text] ### Enable/Disable Subsystems
% if use_mosfet_thermal_model
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
%     set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
% else
%     if use_thermal_model
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'off');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'off');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
%     else
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
%         set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
%     end
% end
% 
% if use_torque_curve
%     set_param('afe_inv_psm/fixed_speed_setting', 'Commented', 'off');
%     set_param('afe_inv_psm/motor_load_setting', 'Commented', 'on');
% else
%     set_param('afe_inv_psm/fixed_speed_setting', 'Commented', 'on');
%     set_param('afe_inv_psm/motor_load_setting', 'Commented', 'off');
% end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:98cafe20]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:14f1bf82]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:2afa06f8]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:854ddeb9]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.000000e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:8549903d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tvQt4VcXVPr7CNdwhQuUmgigaqYItUfuRlpuFr6BREDGABBURJSCt+hkgCAmCiIoYBUSoIDxFLlqpaLGIYNDkLy2ioiIiEpGCIgHCVe7w+6+J+3g4nCRz9tln79kr734eHpKT2TOz3nfNzHvW3OLOnj17lvAAASAABIAAEAACQMBHCMRBwPiILVQVCAABIAAEgAAQUAhAwMARgAAQAAJAAAgAAd8hAAHjO8pQYSAABIAAEAACQAACBj4ABIAAEAACQAAI+A4BCBjfUYYKAwEgAASAABAAAhAw8AEgAASAABAAAkDAdwhAwPiOMlQYCAABIAAEgAAQgICBDwABIAAEgAAQAAK+QwACxneUocJAAAgAASAABIAABAx8AAgAASAABIAAEPAdAhAwvqMMFQYCQAAIAAEgAAQgYOADQAAIAAEgAASAgO8QgIDxHWWoMBBwDoEtW7ZQeno6VatWjSpUqED79u2jq6++msaOHUsNGjTQKmjx4sV08cUX0\/XXX19m+rVr19KQIUOoefPmgfJSU1PpzjvvVL\/PmDGDbrnlFmrRokWZeSEBEAAC5RsBCJjyzT+sL+cIsICZN28eZWZmKhFz5swZeu6555Qg6dmzpxY6kQqYhQsX0hNPPKHKO378OM2ZM4dOnDihhFSlSpW0yiwp0dmzZ4n\/sRjCAwSAgGwEIGBk8wvrgECpCIQKGBYUkydPpg4dOlD79u1p\/vz5tGjRIpUHR0rS0tLUzyx6XnnlFRWladmyJd1000103XXX0fLly+mvf\/0rnTp1irp3764iKyxUrIcjMMEChj\/\/4YcfaNy4cTR69Gj17sCBA6lRo0ZKSH3wwQdK3PTv31+VfezYMZo6dSrl5uaqSBHnfdddd9HHH39MX375JW3bto3+\/Oc\/q\/Iff\/xxJZC4jpx\/3bp1VRm\/\/vWv6e2336ZLLrmEunbtSi+\/\/DIdOHCAJk6cqGzAAwSAgD8QgIDxB0+oJRCICQIsYFgAfP\/994H8e\/ToQZMmTaL169fTmjVr1KDPT05ODrVu3VqJBv48IyODTp48SaNGjVICo2rVqurzYcOGUcWKFZX4qV+\/PnF+pQmYI0eOKPHQt29f4mgOC5i9e\/cqMXL77bfToUOHVMRm+PDhtHLlSlVOnz596Mcff6RHH32UHnnkESVg8vLyVL3j4uJo5syZ1K9fPyWE3nnnHdqxY4eamrr\/\/vtp6NCh9Ic\/\/EGlrV69Oj3wwAP0ySef0L\/+9S9lU7RRoJgQhUyBABA4DwEIGDgFECjHCISbQuIICYuH+Ph4FZFITk5WCHH0hAUKPxydsT7nCA2vaeEIyIQJE85Bk9e7sCiIVMBwVGfr1q3q34YNG+j999+nJ598kpYsWaLEUqtWrdRU0bRp0+h\/\/\/d\/lYDhhwUPPyx6Nm7cSF9\/\/bWK4lx66aU0ePBgys7OVtGYhIQEFWmy7AjFoRy7BEwHAr5BAALGN1ShokDAeQTCDdzWZxxp6dKlS2BxLkc4\/v3vf6vpmXAC5qOPPlLTNZaI4Kmf06dPa00h8XQPCx2OnHAE5vPPP1f\/OCpTr149Ff3hfK0IzWWXXabqwZ+npKScI2AKCwtp\/PjxagqrXbt2tHnzZsrPzw8rYHiqjBcfQ8A471vIEQjEGgEImFgjjPyBgMEIhA7cHNXgKMc333xDv\/nNb1RUZcSIEcoCFhk84B89epT+85\/\/qKkl\/vmhhx6iu+++WwmKpUuXqihH5cqVVXQkOIJjRXFCF\/FyOp7K4fUyPJXEAub1118PiCSuAy8y5gjMihUrlEjiKaSdO3eqOvA0UnAEhm2aO3eueoeng5599llV\/3ARGAgYg50TVQMCZSAAAQMXAQLlGIHQbdS8SPaKK65QwoAjH+EW8XJUZfbs2Uqs8FQPrzP505\/+pKIdLH54USw\/1qLf4DUlpW2j5ncsAcOLb3ltC69n4bUrvEanTp06avqIp6k2bdqkppFq1qxJgwYNOkfA8LvPP\/88vfnmm3T55ZerKSauF0d4uG7BU0gQMOXY+WG67xGAgPE9hTAACJQfBN59911q1qyZEi979uyhKVOmqAgQLxbGAwSAQPlCAAKmfPENa4GArxEoKChQU1S8zoV3I3GkKCkpydc2ofJAAAjYQwACxh5ueAsIAAEgAASAABDwEAEIGA\/BR9FAAAgAASAABICAPQQgYOzhhreAABAAAkAACAABDxGAgPEQfBQNBIAAEAACQAAI2EMAAsYebngLCAABIAAEgAAQ8BABCBgPwUfRQAAIAAEgAASAgD0EIGDs4Ya3gAAQAAJAAAgAAQ8RgIDxEHwUDQSAABAAAkAACNhDAALGHm54CwgAASAABIAAEPAQAQgYD8FH0UAACAABIAAEgIA9BCBg7OGGt4AAEAACQAAIAAEPEYCA8RB8FA0EgAAQAAJAAAjYQwACxh5ueAsIAAEgAASAABDwEAEIGA\/BR9FAAAgAASAABICAPQQgYOzhhreAABAAAkAACAABDxGAgPEQfBQNBIAAEAACQAAI2EMAAsYebngLCAABIAAEgAAQ8BABCBgPwUfRQAAIAAEgAASAgD0EIGDs4Ya3gAAQAAJAAAgAAQ8RgIDxEHwUDQSAABAAAkCgNAROFW6jQ+\/No3p9xgGoEAQgYDRc4pJLLtFIhSRAAAgAASAABJxBoGGVUzSgcRF1u+Aw7TpRie74\/KKoMi4oKIjqfRNfhoDRYIUFTDTk67xfWppwfyvrs+C\/6\/ysAcN5SXTs4pdKSqf7eWi6suzRrVdJNuu+Hy1nbttVGhfBWMCuYjTQxooHvHDtTbeNoI2FR6A0\/O5P+hU92qM1HduYS5UaNKdaHe+k346cd84Y5EXfYWeMiPU7EDAaCEfbWDWK8CQJ7PIEdtuFgi\/b0Hn2IjjzDHpbBXvFF08THd2YS4dy5wWEC08ZsXhx4vHKLifqXloeEDAaCEslH3ZpkG9QEvBlEBmaVQFnmkAZksxtvqz1LYdyX1YIVGvdkWp1HEjxrTs6iojbdjla+VIyg4DRQFoq+d9++y21aNFCAwF\/JYFd4MsUBOCLpjChVw+3+LKES9GrWWqaiIULR1z451g8UscwCBgNb5FKvluNVQNiR5PALkfhjHlmUvli4KTaBrvsNYtQ4cJTRLU6DYyZcLFqKXUMg4DR8EOp5KMT0iDfoCTgyyAyNKsCzjSBMiRZrPhi4VK0JJt4qshamOvmtmipYxgEjEbDkUp+rBqrBqQxTQK7Ygqv45lL5QsRGMddJeYZOu2LLFx2T7\/rnB1FbgoXRGBi7jLmFwABYz5HwTV0uhMyxXrYZQoT+vUAZ\/pYmZDSCb5ivaPIDk5SxzBEYIjo7Nmz9NZbb1FOTg5VrVqVBg8eTCkpKVShQgXlK1LJd6Kx2mlMsX4HdsUaYWfzl8oXIjDO+okbuUXji5Zw4aki\/lmtb4nBjiI7OEgdwyBgiGjTpk00e\/Zsys7Opri4OMrKyqIBAwZQmzZtIGDstBaP34mmE\/K46qUWD7tMZufcuk1esU19sL+oiA6ejadmCdUCCTK6xWaniZvowBd\/QdutrdDR8AsBEw16hr+7ePFiFYVJTU1VNV20aJH63\/pdKvnohAx3zJDqgS\/n+Vq4bpfKdPu+Y+dkvn3f0cDv\/w362\/ai4nSh6UNr1iwhXn106tQpuqRBzeJ3wrzL6fomNSK\/iRr4IqkoC99RZG2FdmtHkZ1WIHUMQwTm\/z88aPLkyZSUlESdO3dWvpGXl0f5+fmUkZGhfk948D07PoN3gAAQ8AECjWtXOq+WjWr98lnw3xv\/\/Hmj2pXopsRiYVLas2PHDmratOl5Sb4\/eIrW7zxGPxw8RW9+dZj49yHX1qV7r6tbVpZG\/L0ku4yoXBSV0LKraAedXZlD9NHfieo1JWp3K8X9cUQUpcbm1S5dupyTcTTX4cSmhtHnCgHzs4Bp3749JScnhxUwUtUrvkVF34DczAF8uYm2M2XpcsZTTgvX\/aAiOxndWhgfkdG1yxkU3culNLtCt0I3m\/GtexWLsiSpYxgETJgpI55C4sW8PXv2VG4jlfzy2AlF2Q94+jr48hR+W4Xb4Ywjvjy1tGzoNep\/Ex87dploR2idQu0ycUeRHRyljmEQMD8v4p0\/fz5lZmYq35g4cSKlpaVRYmIiBIyd1uLxO+Wlc\/UYZseKl8oXA2THNo7CDFu4ifK27qd9z3RyDGcnM7Jjl5Plxyovyy6TdxTZsR0Cxg5qPnnH2kY9c+ZMtZg3PT2dunfvrnYkIQLjExKDqim9c\/UfI6XXWCpfdgWMhRZPK01e8S31TWpI0\/sWf5ky5ZHK2bcff0B1v1mtTszlJ1aXK7rNIwSM24gbVJ5U8sV2Qrik0qDWU3ZVpPphtAKG38\/7Zj+lzPiEklvWpWXp15QNpksppHHmpx1FdiiWOoZhCknDG6SSL60TsqiEXRpObVASqXw5IWBMFTFSOAsVLqfa3kyX3PusQa3DmapIHcMgYDT8Qyr5UjqhUAphl4ZTG5REKl9OCRjOh9fFtJ3woVrU++mY33nOnt85K+lyRb\/bVZJjSB3DfClgjh49Sh9++CEtWbKEfvzxR8XZhRdeSH369KHf\/e53VK3aL6deOtHSpZIvtbHCLie83r08pPLlpICxRAwv7uUD8bwWMX7ljNe2HMqdF7hckS9W5APopEdvpY5hvhIwvMD2008\/JV5s26lTJ7r++uupdu3ayvdY1Hz88cf0xhtvUP\/+\/dXfnXqkku\/XTqgsXmFXWQiZ9XepfDktYEwSMX7iLHQrdHzrjpRw2zji\/8tL9FbqGOYrAXP48GHaunUrXXXVVYGLFkMd8MyZM7R58+bAFmgnumqp5PupE4qER9gVCVrep5XKVywETKiImZaaSMmXun96rx84s3NHkR\/sstNipY5hvhIwTBwLlO+++47q1q1L1atXp9WrV9NXX31FV1xxhboKgA+gc\/qRSr7Uxgq7nG4Bsc1PKl+xEjAmiBiTOYtmR5HJdkXTCqWOYb4SMHwx2vTp02nZsmVUsWJFJVpYxHTt2lWtibngggtoyJAhgfNboiE8+F2p5EttrLDLKc93Jx+pfMVSwFjMpEz\/RK2JcTsSYyJn4YQLr3GJ5DHRrkjqX1JaqWOYrwRMUVERPf300\/Twww\/TsWPHKDs7mx577DFq0KAB8fTSM888Q8OHD6d69eo5wXkgD6nkS22ssMtR9495ZlL5ckPAcBmWiHFzYa9JnDkhXCwnN8kuJxue1DHMVwJm3759NHv2bHrggQfUol0WMOPGjaOEhAT1+3PPPUeDBw9Wvzv5SCVfamOFXU56f+zzksqXWwImWMS4FYkxgbPQrdChO4rseK4Jdtmpd1nvSB3DIGDKYh6XOWogZFYSqZ0Q7DLLz3Rq4yZnbk4nuWlXKM7WVuhTu7c5ftS\/l3bp+JPdNBAwdpFz8D2OwIwaNYpatmxJvB4mNzeXOnbsqM59OXnypNqhNGnSJERgNDGX2lhhl6YDGJJMKl9uRmAsKt0SMW5zFrwV2hIuHHGp1KC5o17stl2OVr6UzCBg3EK6lHKOHz9On332GfH\/4R7egXT11Vc7vhNJKvlSGyvsMqCxRlAFqXx5IWDcmk5yi7PQrdB86FytTgMdFy6Wu7plVwTNw5GkUscwX00h8RbqAwcOqBujwz18e3SdOnVKPCPGridIJV9qY4Vddj3dm\/ek8uWVgHFDxMSaMycX5kbi1bG2K5K6OJlW6hjmKwHDu5DGjh1LO3bsoIMHD9KePXuoefPiEOLevXupd+\/edM8991DNmjWd5J6kki+1scIuR90\/5plJ5ctLARNrERNLzoqWZFPRq1kqytJsxrcx97\/gAmJpl6uGhBQmdQzzlYCxOOEppBdeeIFuu+02atKkifqYRc1rr71G999\/P6aQNFuK1MYKuzQdwJBkUvnyWsBYIiZv637a94xzV6vEwq5Y7Ciy495SfRECxo43xOgdXszL9yGNGDGCatSooUo5cuQI5eTk0H333YdFvJq4S22ssEvTAQxJJpWvWAz0dijjhb0sYpYNvcaxawec4iyWO4rsYOWUXXbKjuU7EDCxRDfCvE+cOEGPP\/44tWjRgrp3767etq4UGDlyZKkRmA0bNqh3efqJozdZWVkqn7feeksJIF4IzGfJpKSkBNbSSCVfamOFXRE2KI+TS+XLFAETHIlxSsREw5m1o4inivip1rojxWJHkR23jsYuO+W59Y7UMcyXU0hWxGXRokVKePCUEl8ncPfddwdupw7nGIcOHaLMzExKT0+nyy+\/nNasWUPvvPMOpaam0ty5c9XBeLwQmEXNgAEDqE2bNiobqeRLbaywy61u0ZlypPJlkoBxWsTY4cztHUV2vNOOXXbKcfsdqWOYrwQMCxUWIfXr1y+Vf46uhEuzfft2evXVV2no0KHq7JidO3eqqwiuvfZadacSCxl+WBjxY\/0ulXypjRV2ud09RleeVL5MEzDBIibaNTGRcublwtxIvDNSuyLJ28u0UscwXwkYPryOL3L89NNPqU+fPupAOxYi\/PBVAl9++SW98cYb1LZtW+rVq1ep\/sJ5LViwQB1+Fx8fT9dff726zZqfvLw8ys\/Pp4yMjEAEJjizVatWeemLjpXNC5+bNm3qWH6mZAS7TGFCrx5S+WLrTbTt3td30fqdx2hWr4b02ybxeiSFpNKyq2gHnV2ZQ\/TR34nqNaW4P44ganerrfLceknLLrcqE2U5Xbp0OSeHgoKCKHM073VfCRgLvsLCQlq8eDEtXbqUWDHzwzdT33LLLWortXUXEi\/2HTRoEPG6F76l2hIkHImZMGGCugSSL4acNWsWtW\/fnpKTk0sUMBLJl\/ptA3aZ19GUViOpfLHNptqWvnATLVy3y\/bC3tLs4qmi3dPvomMbc9VW6F+lz6X41h194ZSm8hUteIjARIugAe\/zAXi8ZoYjLyxmeI1LhQoVzpsy4ikkXszbs2fPQAQGAsYAAjWrILUTgl2aDmBQMpM5m7xiG01e8a0tERNqV7ij\/mt1HOgb4WK5jMl8RePWEDDRoGfIu7t27aLx48erhbzW+TFctU2bNtH8+fPV5\/xMnDiR0tLSKDExEQLGEO4iqYbUTgh2ReIFZqQ1nTOOwnA0ZnrfROqb1FAbNMsuPyzM1TbK4IhZJDaESwsBEy2CBry\/bt06uvfee6lZs2aBLdK8BoR3H\/GaFz5bhqM0vEuJt2fzjiR+pJJveudq12Vgl13kvHlPKl+Mph9ssyNi2K666+Z7dmJurDzVD3zZsV3qGObLNTB2CIzmHankS22ssCsab3f\/Xal8+UXAcD3zvtlPKTM+oYxuLSijW8k3PIeemMuXK\/IZLlIeqb4odQzzrYDhNSljxoxRu4\/48DrenfTnP\/9ZLcx1+pFKvtTGCrucbgGxzU8qX34SMMEihqeSeEop+LFOzLUW5p7ulE4tbns4to7hQe5SfVHqGOZLAcPnwUyePJluuOEGtRvp0UcfpY0bN1Jubq4SM7wA18lHKvlSGyvsctL7Y5+XVL78JmBCRUzODdXo6MZcOpQ7j07t3qZOzLUW5krlTKpdUscwXwoY3h49e\/ZsuuOOO+jJJ5+kcePGqfNgnnvuOXUNgLWN2qmuVyr5Uhsr7HLK893JRypffhQwXOfcDz+lf730LN17YJ7aBs3TRLU6DVQ\/W49UzqTaJXUM86WA4QjMtGnTVARmzpw5SsDwDqOXX36Zxo4dSzVr1nS055VKvtTGCrscdf+YZyaVL78JmND1LW3j51Byy7q0LP2a83xAKmdS7ZI6hvlSwHBr4jUwvO35iy++UBEXnjZiUdOqVSvHO1yp5EttrLDL8SYQ0wyl8uUHARN8fou1voUX5XLUZfu+Y9R2wodhRYxUzqTaJXUM862A4c6Btzzz3UinT5+mOnXqBLZGO93bSiVfamOFXU63gNjmJ5UvkwVM6PktwetbgtkuScRI5UyqXVLHMF8JmKKiIjVFxPdVhHv4TBc+qK5evXqO9rhSyZfaWGGXo+4f88yk8mWqgCmcfhfxriJe09JsRvFVLKU9LGKGLdxE24uO0adjfqeSSuVMql1SxzBfCZgzZ87QgQMHVMSFt03z1BHfYcQP70DiSxlvvPHGwAF0ZTVM3b9LJV9qY4Vdup5tRjqpfJky0Jc2TaTrAaEiRipnUu2SOob5SsBYjY0jMXwB47Bhw6hGjRrq4yNHjqg1MHzSLiIwet2S1MYKu\/T4NyWVVL68FjDWNFHRq1mB3UTVWneI6n6ilOmfqEjMmI51qXfyuWfFmOJP0dRDqi9CwETjFQ6\/e\/jwYTVVdM899wQW7fK0Em+j5sPtateu7WiJUsmX2lhhl6PuH\/PMpPLlhYAJF23RmSaKhGQWMQWFh2nmHVdR8qV1I3nV+LRSfVHqGObLCAy3Ar7XiLdPV65cORCB4UsYr7vuOscbiVTypTZW2OV4E4hphlL5clPA8JqWYxvXBNa28C6iaKMtpZH+xykf0vqdx2jfM51i6htuZy7VF6WOYb4VMOzYp06doj179hCvjalfvz5VqVIlJv4ulXypjRV2xaQZxCxTqXzFWsDwtmc+JddakMs7ieJbd1BboGP9MGcjlu+nvK37adnQa8REYqT6otQxzJcCpqTdSNiFFFm3JbWxwq7I\/MDr1FL5clrA8PQQH+nPooWP+OffrZNy3b5Q0eIsfeEm4tuspYgYqb4IAeN1LxdUvrUbic+B4efgwYO0evVqqlSpEvXv358qVqzoaG2lki+1scIuR90\/5plJ5csJAWOtabGmhzhP3e3PsSQumLPJK7bR5BXfqgsg+SJIPz9SfVHqGObLCEy4BsLTSQsWLKCePXtiEa9mDyK1scIuTQcwJJlUvuwIGCvKcnTjGjr6ZS5Zp+NW+lVzqnZlx\/PuJPKKwlDOOArD0Ri\/ixipvggB41VL0Sx3586dNG\/ePLW1GruQ9ECT2lhhlx7\/pqSSypeOgAmOsJws3KYEixVlcXNNS6S+EI6zvG\/2U8qMTyijWwvK6PbLxY+R5u1leqm+CAHjpVeFlM23UQ8aNIg2bNgQ+MsVV1xB6enp1L17d62D7DZv3kxPPPEETZkyRZ0b89Zbb1FOTo66U4lvtE5JSQlcTSCVfKmNFXYZ1Fg1qiKVr3ACxtotFCxWQgVL5QbNozqrRQPyqJOUxJklYngqiaMxfnuk+qLUMcyXU0i8Buann35Sh9jFxcWpNsLrYfgwu+rVq5d5J5K1CLiwsJBmzJhBP\/74I82ePZuys7NVfllZWTRgwABq06aNylsq+VIbK+zy17AhjS8WKad2f6cW2R7671dU6eAu9bP18BoWk6MrOt5TGmeWiCnpJmud\/L1KI80XLRyljmG+EzBHjx6lXbt20fz589VBdtWqVVMc8RUDLEIefvhhdcVASQ+vlZk5cyZdfPHF9O6776qzZFauXKkEUGpqqnpt0aJF6n\/rd6nkS22ssMur7t9euSbyxSLEeliM8GOJEI6eqN93bztHmAQLFP6Z160cq3YBNbjuRvUnP0RWdBksizO\/ipiy7NLFx7R0Uscw3wkYPsCOBcf69evVoXXWQXbsMK1bt6bOnTur3UglPXl5ecTTRzfffDM99thjSsCw8ElKSlLv8sNp8vPzKSMjQ\/3+\/91cfFie9TS80N8r7S07Tp8+RRUrloyVaY1Qtz6wSxcpM9Kdw1dR+ItaPatpvaa\/FJ3w88\/1mhR\/Vq8pxfHf291aYvX4hHA+3kHao2PX9wdP0U3zdtBvm8TTrF7+6DN17PILl126dDmnqgUFBX6punY9fSdg2LLjx4\/Ttm3bqGXLlqWKleC1MkOGDKGBAwfSSy+9RA888ACdPHlSTRlZAoYvhUxOTg4rYKSqV6nfNmCXdvs3IqFUvhhcqbbp2hXuJmsjnK6ESujaZbIN4eomdQzzlYDhtSuLFy9W0ZPHH3+cWC0HP2UdZLd27Vrq16\/fOe80btyYbr\/9dmrQoME5U0i8mJe3ZPMjlXypjRV2+at7lcoXBEyxH\/pJxEj1RaljmK8EjJPdMkdnrAgML+LlNTWZmZmqCL5TKS0tjRITi1fRSyVfamOFXU62lNjnJZUvCJhzfce6yXpaaqKxVw9I9UWpY5hvBQwLkLfffpt2794daCV16tShPn36UM2aNcvsdYMFjLWNmhf38mLe0O3YUsmX2lhhV5nub1QCqXxBwJzvZqaLGKm+KHUM86WAOXz4sIqW8DbnVq1aBVoJT\/tcffXV6iwXJx+p5EttrLDLSe+PfV5S+YKACe87LGJMvQRSqi9KHcN8KWA4esI7h3gxrrWNOpbdrFTypTZW2BXL1uB83lL5goAp2VdMvQRSqi9KHcN8KWD4LBdes3L99dfTlVde6XyPGpKjVPKlNlbYFfMm4WgBUvmCgCndTaxLIE26yVqqL0odw3wpYHg3Ep\/RkpubSxdddFHg7qOydiHZ7XWlki+1scIuu57uzXtS+YKAKdufTLsEUqovSh3DfClgym4WzqaQSr7Uxgq7nPX\/WOcmlS8IGD3PsUSMCZdASvVFqWOYLwUML+JdsmSJuj4g+OFTeflAuquuuqrUA+70mtUvqaSSL7Wxwq5IPdzb9FL5goDR9ytTLoGU6otSxzBfChg+iZdP1N25cyd17dqVTp8+rW6TbtKkidoGHR8fT8OGDdNvPWWklEq+1MYKuxxzfVcyksoXBExk7mPC\/UlSfVHqGOZLAXPw4EF64YUX1Hkt1pkv\/Nm0adMC1wWMHTs2stZTSmqp5EttrLDLMdd3JSOpfEHARO4+XosYqb4odQzzrYCZOnWq2kbNh9Dxwwt7n376abrvvvto7ty5BAFTduchtbHCrrK5NymFVL4gYOx5GV890HbCh5Tcsi4tS7\/GXiY235LqixAwNh0iVq+9\/vrrtHDhQrWVukKFCvTpp5\/SjTfeSBUrVqTvv\/8eU0gawEttrLBnIJzSAAAgAElEQVRLg3yDkkjlCwLGvpN5dX+SVF+EgLHvizF7s7CwkLZs2UJnzpxR61\/q16+vfq5RowYW8WqgLrWxwi4N8g1KIpUvCJjonCxYxPBZMc0S4qPLUONtqb4IAaNBvptJ2NF44S4fascPL+z95ptv6KmnngpMKzlVH6nkS22ssMspz3cnH6l8QcA44z9u3p8k1ReljmG+XANz9OhRdWP0tddeSx999BH95je\/of\/+97\/Utm1b+v3vf+9MqwnKRSr5Uhsr7HK8CcQ0Q6l8QcA45zZuiRipvih1DPOlgAm+C2nlypXE5PAUEt+PNHToUK3bqCNpWlLJl9pYYVck3u19Wql8QcA461vWJZD7nunkbMZBuUn1RaljmC8FDB9kx1NFgwcPpm3bttFPP\/2kFvNOmTKFRowYQQkJCY46uFTypTZW2OWo+8c8M6l8QcA47zqxvgRSqi9KHcN8KWC4Waxbt45Wr15N\/fv3p9GjR9PWrVspNTWV7r\/\/fkcX8HJZUsmX2lhhl\/MDRyxzlMoXBExsvMa6BHJ630Tqm9TQ0UKk+qLUMcy3AibYa3khL0dhatWqRXFxcY46NASM43DGPEOpnRDsirnrOF4AOHMcUpVhrC6BlMoXBExs\/DCiXPmwOj6gbseOHWHf07mNmrdeT5gwgTZt2kSNGjWicePGUYsWLdSOppycHKpataqamkpJSVHny0DARESREYmldkKwywj3iqgS4CwiuCJKHIv7k6TyBQETkWvFJjELGF7jcvLkSerTp4\/ahVStWrVAYRx9qVOnTkB4hNaCIzVPPvkktWvXjv74xz\/S+++\/T2vXrqUePXrQnDlzKDs7W0VwsrKyaMCAAdSmTRsImNhQGdNcpXZCsCumbhOTzMFZTGANZOq0iJHKFwRMbP1QO3c+qI4vceSIydKlS+nqq6+m3r17q63UVapUKTWfH374QS305ShO7dq11RkyPPX09ttvq0sgeQ0NP4sWLVL\/W79LJV9qY4Vd2s3JiIRS+WJwpdpmkl1O3p9kkl1ONk6pY5iv18Cw6OBrA5YtW0Zvvvmm2on04IMPlriNmk\/t5QsfOUqzatUqJXpGjhxJf\/vb3ygpKYk6d+6sfCYvL4\/y8\/MpIyMjEIEJdiZ+V8LDU3E87SbtgV3+YlQqX8yCVNtMs+v7g6fopnk76LdN4mlWL\/sLe02zK5qW3KVLl3NeLygoiCY7I9\/1tYAJjsaEEzB8XsygQYNow4YNNGTIECVQHn74YXWT9ZVXXklr1qyh1157jS688ELq2LEjJScnlyhgJJIv9dsG7DKyrymxUlL5QgTGXT904hJIqb6ICIy7vlhiadFMIXEEZt68eZSZmanWzrDA4XUvPA3F9ycFTyHxYt6ePXsGIjAQMIY4gEY1pHZCsEuDfMOSgDN3CYn2EkipfEHAuOuHYUuzFvHu3buX7rnnnogX8fIBeOPHj1dRmcsvv1ydJfPGG2\/Q7bffTq+88ooSNvzwNQVpaWmUmJgIAWMA75FWQWonBLsi9QTv04Mz9zkIFjHTUhMp+dK62pWQyhcEjLYLxC6hE9uoOZLCURfeTt2yZUu1BqZx48ZqUfDMmTPVYt709HTq3r174EwZqeRLbaywK3ZtMBY5S+ULU0ix8Bb9PO3cnyTVF6WOYb5eA6PvytGllEq+1MYKu6Lzd7fflsoXBIzbnnR+edb9ScuGXqMViZHqi1LHMAgYjTYmlXypjRV2aTi1QUmk8gUBY4aTRXJ\/klRflDqGQcBotDGp5EttrLBLw6kNSiKVLwgYc5xMV8RI9UWpYxgEjEYbk0q+1MYKuzSc2qAkUvmCgDHIyYhI5xJIqb4odQyDgNFoY1LJl9pYYZeGUxuURCpfEDAGOdnPVbEugczo1oIyujU\/r4JSfVHqGAYBo9HGpJIvtbHCLg2nNiiJVL4gYAxysqCqlHZ\/klRflDqGQcBotDGp5EttrLBLw6kNSiKVLwgYg5wspCol3Z8k1ReljmEQMBptTCr5Uhsr7NJwaoOSSOULAsYgJwtTlXAiRqovSh3DIGA02phU8qU2Vtil4dQGJZHKFwSMQU5WQlVC70+S6otSxzAIGI02JpV8qY0Vdmk4tUFJpPIFAWOQk5VSleCrB5b2b0gtWrTwR8UjqKXUMQwCRsMJpJIvdeCAXRpObVASqXxBwBjkZGVUxRIxBYWH6Yus3\/un4po1lTqGQcBoOIBU8qUOHLBLw6kNSiKVLwgYg5xMsyp\/nPIhFR4livQSSM3sPUsmdQyDgNFwKankSx04YJeGUxuURCpfEDAGOZlmVdgXRyzfT9uLjokSMVLHMAgYDceWSr7UgQN2aTi1QUmk8gUBY5CTaVbF8sVIL4HUzN6zZFLHMAgYDZeSSr7UgQN2aTi1QUmk8gUBY5CTaVYl2Bd170\/SzNrTZFLHMAgYDbeSSr7UgQN2aTi1QUmk8gUBY5CTaVYl1Bd17k\/SzNrTZFLHMAgYDbeSSr7UgQN2aTi1QUmk8gUBY5CTaVYlnC9a9ydN75tIfZMaauZkVjKpY1i5EzDr1q2jxx9\/nI4fP06XXXYZjRkzhurXr09vvfUW5eTkUNWqVWnw4MGUkpJCFSpUUF4olXzYZVYnU1ZtwFdZCJn3d3BmHiel1agkvkq7P8kPFkr1w3IlYA4ePEjZ2dn04IMPUuPGjemNN96grVu3Uvfu3Wn27Nnqb3FxcZSVlUUDBgygNm3aQMD4oXWG1FFqY4Vd\/nNGcOYvzkrjy88iRqofljsBM3r0aBo+fDi1atWKlixZQgcOHKA6derQ2bNnKTU1VbW2RYsWqf+t36MlX+f90tKE+1tZnwX\/XednO92Mjl2lRbBKej\/089J+D2ebbr1Ksln3\/Wg5c9su3Wgi7Cr2DLSxgvNwsDDRbSPS2lhJl0AG26mDjR\/bmJ0xItbvlCsBw2C+++67SsDwFBJHWKZNm0Z\/+9vfKCkpiTp37qzwzsvLo\/z8fMrIyAg04FgTgfyBABAAAkDAfATOVK9PB7tOprr\/GGR+ZYNqWFBQLEglPaIFzL59+2jQoEG0YcMGGjJkCPXv35\/Gjx9PY8eOpSZNmlBubi6tWbOGKleuTB06dKDk5OSwAkYS4bAFCAABIAAEokOArx5olhAfXSZ4O2oERAuYUHTWrl1Ly5cvVwKmUqVKVFhYqARN27ZtqUaNGudMIfFi3p49e0YNMDIAAkAACAABIAAEnEegXAmYbdu20RNPPKEW6TZs2FBFX\/hfr169aMGCBZSZmakQnjhxIqWlpVFiYqLziCNHIAAEgAAQAAJAIGoEypWA4YW677\/\/Pj377LNqDQwvpBo1apTakcTbqGfOnKkW86anp6udSbwjCQ8QAAJAAAgAASBgHgLlSsCYBz9qBASAABAAAkAACNhBAALGDmp4BwgAASAABIAAEPAUAQgYT+FH4UAACAABIAAEgIAdBCBg7KCGd4AAEAACQAAIAAFPEYCA8RR+FA4EgAAQAAJAAAjYQQACxg5qeAcIAAEgAASAABDwFAEIGE\/hR+FAAAgAASAABICAHQQgYOyghneAABAAAkAACAABTxGAgPEUfhQOBIAAEAACQAAI2EEAAsYOangHCAABIAAEgAAQ8BQBCBhP4UfhQAAIAAEgAASAgB0EIGDsoIZ3gAAQAAJAAAgAAU8RgIDxFH4UDgSAABAAAkAACNhBAALGDmp4BwgAASAABIAAEPAUAQgYT+FH4UAACAABIAAEgIAdBCBg7KCGd4AAEAACQAAIAAFPEYCA8RR+FA4EgAAQAAJAAAjYQQACxg5qeAcIAAEgAASAABDwFAEIGE\/hR+FAAAgAASAABICAHQQgYOyghneAABAAAkCgXCNQtCTbUftPFW5zNL\/QzBqkz41p\/l5kDgGjgfoll1yikQpJgAAQAAJAwEkEul1wmC6sekpleWGVk4GsG1Yp\/sx6rDT8e+jfnKxPcF67TlRyNOsfjzubX2jlev7rqKP1NSEzCBgNFljAFBQUaKQMn0Tn\/dLShPtbWZ8F\/13nZzvG6djF+ZaUTvfz0HRl2aNbr5Js1n0\/Ws7ctqs0LoKxgF3FaKCNFfd54dqbbhux\/OpQ7st0avd3ZEUZ8t5cRNddfz2d2r0t8Fm49sgioWmTpoE\/VfpVc\/Vz5QbN6bW\/v0a9b+2tfq\/UoPjz346cd05fjTZWPHZFyped8cCLdyBgNFCXSj7s0iDfoCTgyyAyNKsimbOv\/71aoXB0Y676\/9jGNXSycFtYUWIJjGABYgmPen3GaaIZ+2SS+YrmS3jskbdXAgSMBm5wag2QDEoCvgwiQ6MqUvkKjV5oQGF0Eo6iWCJl2yd550zVsEBhccKREX78ut5Cqi9KtQsCRqPLkEr+t99+Sy1atNBAwF9JYBf4MgUBP\/oiT\/NwVMUSK8d+jrBYUZRqrTvS4V+1pha3PWwKzI7Vw4986RgvdQyDgNFgXyr5Uhsr7NJwaoOSSOWLITbdNmtNCguWQ7nzKFisWBGV+NYdVGQlvnXHgNeYbpdd95Zql9QxDAJGw9Olki+1scIuDac2KIlUvkwVMMERFp4W4oejKxxZ0Z36kcqZVLukjmEQMBoduVTypTZW2KXh1AYlkcqXSQLGEi1WlMVas9I46z1bniCVM6l2SR3DIGA0mq9U8qU2Vtil4dQGJZHKl9cChkXLoffm0dEveT1LbsRRltJcRCpnUu2SOoZBwGh05FLJl9pYYZeGUxuURCpfXgqY77M6nSNaanUceM4almjpl8qZVLukjmEQMBotWSr5Uhsr7NJwaoOSSOXLTQFjRVuKXs0KRFqcFi3BLiOVM6l2SR3DIGA0OnKp5EttrLBLw6kNSiKVLzcEDE8N8boWXozL61pqdbyT3DgYTipnUu2SOoZBwGh05FLJl9pYYZeGUxuURCpfsRQwLFhYuPBR\/JHsHnKKdqmcSbVL6hhW7gTM4sWLadSoUYF2PGnSJOrTpw+99dZblJOTQ1WrVqXBgwdTSkoKVahQQaWTSr7Uxgq7nBqm3MlHKl9OCxhrmsja+szRllqdBgbuAXKHreJSpHIm1S6pY1i5EzAzZ86kLl260GWXXRZo75s2baLZs2dTdnY2xcXFUVZWFg0YMIDatGkDAeNmr+hQWVI7IdjlkIO4mI0TnIWub3Frmqg0mJywy0UatIuSahcEjLYLmJvwxIkT9Pzzz1O\/fv2oUaNGgYpyVObs2bOUmpqqPlu0aJH63\/pdKvlSGyvsMrcNhquZVL6ijVSYKFws\/qRyJtUuqWNYuYrAHDlyREVZduzYQVu2bKGuXbvS8OHDad68eZSUlESdO3dW7TMvL4\/y8\/MpIyMjEIEJ7nhXrVrlrxGihNoyDk2b\/nJVvQijiBS\/sMs\/bErlixmwZVvRDjr70d+JVuYQ1WtK1O5WivvjCKMItWWXURaEr4wku3imIfjBbdQ+cMDSqnj48GFauXKlmkKqVasWrVixgj744AOqXr06dejQgZKTk0sUMBLJl\/ptA3b5q6FK5SvSCIzJEZdQj5LKmVS7EIHxV5+oVdvCwkIaP348tW3blmrUqHHOFBIv5u3Zs2cgAgMBowWpEYmkdkKwywj3iqgSOpz5SbhgCiki+o1JDAFjDBX2K8KLdXkRL4uW2rVrU25urorA9OrVixYsWECZmZkq84kTJ1JaWholJiZCwNiH27M3dQYNzyoXRcGwKwrwPHq1LM6KlmSTdfhcsxnfelTLyIsty67IczTjDal2QcCY4V9R1YIX6i5fvpymTp1KVapUUTuRxowZQ\/Xr11fbqFnccJr09HTq3r272pHEj1TypTZW2BVVM3H9Zal8MZDhbPNjxCXUKaRyJtUuqWNYuVrEa7dnlkq+1MYKu+x6ujfvSeUrnIAJjriYsB3aLuNSOZNql9QxDAJGowVLJV9qY4VdGk5tUBKpfFkC5qKacepWaGuqyM\/CxXIbqZxJtUvqGAYBo9GRSyVfamOFXRpObVASqXzxVNH2pc+q7dBu3lPkBrVSOZNql9QxDAJGo7VLJV9qY4VdGk5tUBJpfLFw4akideR\/vabUoN9j6pJFSY80zqRHlqSOYRAwGr2KVPLRCWmQb1AS8GUQGSFVYdFy9Oeboa0LFmt1HEg\/VL+YWrRoYW7FbdYMvmgTOI9ekzqGQcBoOJRU8tEJaZBvUBLwZRAZP1elrB1F4Mw8zkqrkVS+pI5hvhAwfIIu32PED29\/rlmzpqutQir5Uhsr7HK1eURdmN\/4skTL0S9z6djGXLW+pV6fcWGnifxmmy6ZsEsXKTPSSR3DjBUwZ86cUfcRTZs2jfbs2aMOnuPn2LFjVKdOHXVWS\/v27alChQox9xCp5KMTirnrOFoA+HIUzogyC54iskQLr2up1roDxbfuWGJe4CwimD1PLJUvqWOYkQLm6NGj9NJLL6kL+W644YbzIi78d77H6JNPPlGXM8b6kUq+1MYKu2LdIpzN31S+wkVaqrXuSA3S52oDYKpt2gaUkBB2RYugu+9LHcOMFDAcfeF\/lSpVKpVlToMIjP2GgE7IPnZevAm+You6FWU5tnFN8Q4iIjU9FKloCa4lOIstZ07nLpUvCBinPUUjv6KiIho7dqy6kj74iY+Pp65du9Ktt94amFrSyM52EqnkS22ssMu2q3vyold8BQsW3kHEv7NgqfSr5tQ46z1HsPDKNkcqX0omsCvWCDubv9QxzMgIjEXd6dOn1SWLfDN0586d1T1F\/\/jHP9SfeXpp\/fr19OijjzrLdJjcpJKPTijmruNoAeArOjg5qsLRlZOF29TiWyvCwoKl2pUd1UJcpx9w5jSisc1PKl9SxzCjBQxHYGbNmkXDhg2jGjVqKM89cuSIWth7xx130Lx582j06NGx9Whc5hhzfJ0uQGonBLvK9hQWJsUC5VyhEixWKjdoTvGtO7hyuBw4K5szk1JI5QsCxgMv4+3TTz31FA0ePFhFXPjZuXMn5eTk0NChQ5WAGTcu+m9NHNnh26g5X472cHkpKSmB9TVSyZfaWGGXB401iiIj5cuKpHCRwdEUqwrWNJAlVPh\/jrLw524\/kdrmdv3slge77CLnzXtSxzCjIzBM9bp165RIqVy5shIUhw4doqysLNq1a5eaUrrtttui9ohNmzbR7Nmz1Y6muLg4lf+AAQOoTZs2Km+p5KMTitp1XM1AEl\/WIlkGsPDrT6nWyQMKSxYk\/PBptrwmJdxjCRFeXMsPR1P4KRYsJW9pdpWsnwuTxFkwfrDLC2+yX6bUMcx4AcMi5bvvviNuMBdffLGKkDRp0oROnTpV5i4lXboXL16sxFBqaqp6ZdGiRep\/63ep5KMT0vUQM9Lp8mWt77BqbYmCcFac2v1dWONKEg+l5cmiw3rKet9Kx2LEasscJWERYj2WMPHzPUK6nJnhYfq1gF36WJmQUuoYZryAef311+nNN98kXtD7yCOP0LPPPkt33nknJScnO+YXkydPpqSkJLVQmJ+8vDx1iF5GRob6veC2OMfKQkZAwCQEvq\/UsMTq\/FDxl7\/9tmn8uenqNTn\/vXrF07z8xPHP7W7VMpV3GVpTxFov+CiRVNtgl\/lO2KVLl3MqWVBQYH6lI6yh0QLm4MGDNHXqVDWdw+tTeCqJrxTgRbwsZqzTeSO0+bzkLGD4VF9LFIUKGKnqFd+iovUcooXrdp2TyfZ9x6LOdPu+o2Hz4DVhZV2j8d8oyt9eVHLdo7GrWcIv4qdZveKfL\/r5s2YJ1Wh\/URHVrVePMrq5v0YlarLKyABtLNYIO5u\/VL6kjmFGCxhrF1L\/\/v3VYl4WMHwXEv88YsQISkhIcMR7Q6eM+HeequrZs6fKXyr5Uhsr7HKkWTiWyeQV565lYYEWLLQKCg\/T9wdPlVgeCyBL+LS\/tJ5K5xexA190zI1cyUgqX1LHMKMFDK9LmTNnDvHVAZ999hndf\/\/9tGrVKuKD7O677z7H1sDwIt758+dTZmamaiQTJ06ktLQ0SkxMhIBxpdtwthCpnVB5s4ujW8GRn\/xvipSj5G3df57DWCKHBY5J4qa8ceZsS3Y\/N6l8QcC470uqxOPHj9M777xDvBaGL3Ls0aOHOoG3WrVqjtXI2kY9c+ZMtZiXL4rs3r272pGECIxjMLuWkdROCHaV7EIc5bEiO8ECJ1jYtG9Zl5IvreuaH3JB4MxVuKMuTCpfEDBRu4Z+BnzH0YEDB5SYCPewsOAbqd24BwkCRp83U1JK7YRgl76HceSG\/+X\/HK3h6I0lbFjUsJjh9TexjtaAM33OTEgplS8IGBe9K\/gOJF7Iu2fPHmrevHiB3969e6l37950zz33lLmg0akqSyVfamOFXU55vjv5uM0XR2tCBU3fpEZK1DgdoXHbNncYQ2TJLZydKkfqGGb0GhiePnrhhRfUYXV89gs\/vH3vtddeU+theKGtG49U8tG5uuE9zpUBvpzD0sqJozS81sYSNBydcVLMgDPnOYtljlL5kjqGGS1g9u3bR7wuhXccBd+FxFuqeRGvU7uQymoQUsmX2lhhV1kebdbfTeIrODpjiZlopplMss1J1mGXk2jGPi+pY5jRAobPfHn88cepRYsWalEtP6tXr6avvvqKRo4ciQhMlH6PTihKAF1+HXy5B7gVmVm47ge1liajWwtb62XAmXucOVGSVL4gYJzwDht58O3TfC4LX7bIU0pdu3alu+++27FD7HSqJJV8qY0Vdul4tTlpTOeLp5gmr\/hWCZnklnVpWt9ECj6crzQkTbfNrhfALrvIefOe1DHMyAgMCxW+tLF+\/fqlss2Le8tK44S7SCUfnZAT3uFeHuDLPazDlZT3zX4atmhTRBEZcOYtZ5GWLpUvqWOYkQKGL3dbtmwZffrpp9SnTx9q2bJl4NwXPtTuyy+\/pDfeeIPatm1LvXr1itRHI04vlXypjRV2Rezinr7gN754nQxHZDgKMy01sdSdS36zTdcRYJcuUmakkzqGGSlgLMoLCwuJb4peunSpOhCKnyuuuIJuueUWtZUai3ijaxzohKLDz+23wZfbiJdeniVk+iY1pOl9i0\/tDn3AmVmclVUbqXxBwJTFvOC\/SyVfamOFXf5qjH7mi6eVUmZ8UmI0xs+2leZFsMtfbUzqGGZ0BMYUF5FKPjohUzxMrx7gSw8nt1Px4t5hCzcR3+YdOqUEztxmI7rypPIldQyDgNHwd6nkS22ssEvDqQ1KIoWvlOmfqOsKlg29JrAuRoptmBozqMHYqIrUMQwCRsMZpJKPzlWDfIOSgC+DyCihKukLN6mTffc900mlAGfmcxZcQ6l8SR3DjBcwfC\/SnDlzaNu2ber+o02bNtHNN99s+zZqXhQ8atSogM9OmjRJ7XTic2b4hF++nmDw4MGUkpISuCxSKvlSGyvswqDhJQLW4l6OxDSpWKQO4pT2oI35i1GpY5jRAoa3U0+ZMkVto16zZg2NHj2aVq5cSXzBI18lUKlSpYi9iK8m6NKlC1122WWBd1kUzZ49m7Kzs4lvus7KyqIBAwZQmzZtVBqp5KMTith9PH0BfHkKf0SFW5GYWb0aUu\/k8DuUIsrQsMTwRcMIKaM6UscwowWMdRdSWloaPfXUUzRu3DgVIbF7FxJfTfD8889Tv379qFGjRgHKOSpz9uxZSk1NVZ\/xyb\/8WL9LJR+dkL86IfDlL754TUxB4WH6Iuv3\/qq4Rm3hixogGZRE6hhmtIA5fPgwzZgxQ535wqKFBQwfZMc\/jxkzJuLrBPhaAo6y8I3WW7ZsUdcSDB8+nObNm0dJSUnUuXNn5XJ5eXmUn59PGRkZgQhMsC+uWrXKINe0XxXGoWnTpvYzMPRN2GUoMSVUSypfbO7Ahd+pL10ciZH0SOVMkl080xD8FBQUSHJBZYvRAsYSEzyNtH\/\/fvrtb39L69ato4kTJ1JycnKZZHAEZ9CgQbRhwwYaMmQIpaenqykoJrZWrVq0YsUK+uCDD6h69erUoUOHQJ7hBIxE8vEtqkwXMioB+DKKDq3K5G\/YQjfN26HuUFqWfo3WO35IBF\/0A0u\/1BERGA\/54kgMR0z4jqRLL73UsfuP+KTf8ePHqysJatSocc4UEn9r6tmzZyACAwHjoQNEWDQ61wgB8zi5VL4YVrZt5+l66rC74O3VHkMedfFSOZNqFwRM1C5vLwOOuHDUhBfVZmZmEguJBx980NYdSLxYlxfxsmipXbs25ebmqggM36e0YMEClT8\/HOHhdTeJicWL76SSL7Wxwi57bc2rt6TyZQkY3oXE62H4oLtPx\/zOK5gdLVcqZ1LtkjqGGT2FxJGXJ598ku699161bmXz5s1qe\/Nzzz1Hf\/nLXyJeA8MLdZcvX05Tp06lKlWqqJ1IvJaGb7TmbdQsbjgNTzV1795d7UiCgHG033MlM6mdEOxyxX0cLSSYs7YTPqS+SY0oo1tzR8vwIjP4oheo2y8TAsY+drbf5DUsvL35gQceoL\/97W\/Url07atWqle1dSHYrIpV8dEJ2PcKb98CXN7hHU2owZ9a9SRKmkuCL0XiF++9KHcOMjsDwtufJkyfTTz\/9RLt27SI+dI6nlD7++GMaOXKkWt3vxiOVfHRCbniPc2WAL+ewdCunUM54Kokfvy\/ohS+65UHOlCN1DDNawDB1vPX5888\/pwsvvJAaNGhA7733ntoxxGtY3Hqkko9OyC0PcqYc8OUMjm7mEo6zhAffo+l9E6lvkn+3VsMX3fSi6MuSOoYZL2D4NN5Dhw6ptSn88DkwfNDc3XffTfXq1YueWY0cpJKPTkiDfIOSgC+DyNCsSjjOrKsGrPuSNLMyKhl80Sg6yqyM1DHMaAHD4oUX1v7zn\/9U1wfUrVtXTSfddddd6jRdO1cJlMl0mARSyUcnZMcbvHsHfHmHvd2SS+LM7wt64Yt2PcKb96SOYUYLGL7IkY\/+523Tq1evVtuaK1eurLZVs4iBgImuMaATig4\/t98GX24jHn15JXHGN1bzfUm8rbpZQgyKWN0AABkQSURBVHz0BbmcA3zRZcCjLA4CJkoA7bweLGD4MscLLrhAXbDI26j5xuiEhAQ72Ub8jlTy0QlF7AqevgC+PIXfVuGlccZRmPYt66r1MH574Iv+YkzqGGZ0BIbXvbz44otqIe8NN9xAf\/3rX+nKK6+kr776Sh02V7NmTVe8SCr56IRccR\/HCgFfjkHpWkalcebnKAx80TUXcqQgqWOY0QKGmTtz5oxa98L3Fa1fv574NN0\/\/elPakeSW49U8tEJueVBzpQDvpzB0c1cyuKMt1VflBDvuyhMWXa5ibGTZUm1S+oYZryA4YW8HHHhyxyth89\/ufrqq3EOTJQtV2pjhV1ROobLr0vli2EsyzYrCuO3w+3KsstlF3KsOKl2QcA45iL6GfHljXxvEe9AYgKsp06dOtSnTx9MIelDGTal1MYKu6J0DJdfl8qXjoDhNH6MwkjlTKpdEDAud2pcHF8lwNuoR4wYoW6L9uqRSr7Uxgq7vGop9sqVypeugPFjFEYqZ1LtkjqGGT2FxBGYF154gW677TZq0qSJvd7Rgbekki+1scIuB5zexSyk8qUrYPwYhZHKmVS7pI5hRgoYvoV6yZIldODAAdqzZw\/l5uZSp06d1DZqfnSnkHj9zLRp06hHjx7q5ml+CgoKKDMzk\/bu3Uu\/\/\/3vVXSnVq1a6jbqnJwcta6Gt2jzrdcVKlRQ70glX2pjhV0uqg8HipLKVyQCxm9RGKmcSbVL6hhmpIDhnUcsXqzrA0L7yLi4OCViLIERrg9du3YtLVy4kD777DOaNWuWEjAsjLKysujOO++k1q1bB0QSCxm+9To7O5s4b04zYMAAdeYMBIwDI5TLWUjthGCXy47kQHGRcOantTCR2OUAjK5lIdUuCBjXXKi4IL59esaMGeoix+Tk5IjvPtq8eTPt3LlTiZSHHnpICZgtW7bQ3LlzVQSG19R8\/fXXNGfOHCVm+ITf1NRUVTbftcSP9btU8qU2VtjlcmONsjipfEUSgeG0forCSOVMql1SxzAjIzC89uWxxx6jdu3a0bXXXqumd3gq6ZFHHono+gC++JEPvBs4cKASMByVefvtt+nRRx9V+RQWFqpdTjw19Yc\/\/IE6d+6suuK8vDzKz8+njIyMQAQmuI9etWpVlF22Ga\/v2LGDmjZtakZlHKwF7HIQTBeyksoXQxepbfe+vkshPquX2TdVR2qXC27kSBGS7OrSpcs5mPDyCWmPkQImdPcR\/\/7kk0\/S6NGjqXbt2mE5mDx5sjq1l6d9XnrpJXXNQDgBw3cqjRo1Sk0VBQsYPumXIz0lCRiJ5Ev9tgG7\/NVNSeUr0giM6nu+2U8pMz4h08+FkcqZVLsQgXGxT2TBwmtSHnjgAapWrZraTs3rU8aNGxfR\/UehAsaaMuIIjDWFtGDBAhWd4YhM8BQSL+bt2bNnIAIDAeOiA0RZlNROCHZF6RgevG6HM14Lw8+y9Gs8qLFekXbs0svZ21RS7YKAcdGvYiVgeBEvTxkNGjSIWrVqpdbH8HRVUlISzZ8\/X62N4YenndLS0tTt1\/xIJV9qY4VdLjZWB4qSypedCAy\/s33fMeKLHk2OwkjlTKpdUscwY6eQWGRs2LAhbPcYPE1UWv8ZGoHhtBxJ4V1GvEi4Y8eOKsrD0RheZ8OH5vHOp\/T0dOrevbuaZoKAcWCEcjkLqZ0Q7HLZkRwozi5n6Qs3Uf7W\/fTpmN85UAvns7Brl\/M1cTZHqXZBwDjrJ77KTSr5Uhsr7PJV8yrzviB\/WXNubaPxxYQH31OXPPZNMm9BbzR2mcynVLukjmFGRmBMc3Cp5EttrLDLtBZUen2k8mV3CikYLRYx+57pZByhUjmTapfUMQwCRqNrkEq+1MYKuzSc2qAkUvlyQsDwWphm9eKNW9ArlTOpdkkdwyBgNDpyqeRLbaywS8OpDUoilS8nBIyp26qlcibVLqljGASMRkculXypjRV2aTi1QUmk8uWEgOE8eFt13tb9Rk0lSeVMql1SxzAIGI2OXCr5Uhsr7NJwaoOSSOXLKQHD+fBUUvuWddWiXhMeqZxJtUvqGAYBo9EbSCVfamOFXRpObVASqXw5KWBMm0qSyplUu6SOYRAwGh25VPKlNlbYpeHUBiWRypeTAsaaStpedIympSZS8qV1PWVQKmdS7ZI6hkHAaHQDUsmX2lhhl4ZTG5REKl9OC5hgEeP1AXdSOZNql9QxDAJGoyOXSr7Uxgq7NJzaoCRS+YqFgOFrBviyR6+3VkvlTKpdUscwCBiNjlwq+VIbK+zScGqDkkjlKxYChvO07kpKblnXs\/NhpHIm1S6pYxgEjEZHLpV8qY0Vdmk4tUFJpPIVKwFjUcen9PI1A17sTJLKmVS7pI5hogXMqVOnaNq0adSjRw+67LLLVLtfvHgxjRo1KtB9T5o0ifr06aMuc8zJyaGqVavS4MGDKSUlhSpUqKDSSSVfamOFXQapE42qSOUr1gLG2pnkRSRGKmdS7ZI6hokVMGvXrqWFCxfSZ599RrNmzQoIGL5xukuXLoHfuZPZtGkTzZ49m7Kzs9UN1Hxb9YABA4hvvYaA0RiBDEsitROCXYY5mkZ1Ys2ZVyIm1nZpQBuTJFLtgoCJibvELtPNmzfTzp07acmSJfTQQw8pwXLixAl6\/vnnqV+\/ftSoUaNA4RyVOXv2LKWmpqrPFi1apP63fpdKvtTGCrti165ikbNUvmIdgbG4sNbENEuId22LtVTOpNoldQwTG4Hhxn306FGaOHEiDRw4UAmYI0eOqCjLjh07aMuWLdS1a1caPnw4zZs3j5KSkqhz586qT8jLy6P8\/HzKyMhABCYWI1aM85TaCcGuGDtODLJ3izMWMcMWblJXDmR0a0EZ3ZrHwJpfsnTLrpgaESZzqXZBwLjtSRGWN3nyZHrxxRfVtM9LL71ECQkJ5wmYw4cP08qVK9UUUq1atWjFihX0wQcfUPXq1alDhw6UnJxcooAJrs6qVasirJ2ZyVnINW3a1MzKRVEr2BUFeB68KpUvhtJt22b9ez+9+J\/91Lh2JXpzYOzattt2ueWWkuzicS74KSgocAtG18opVxGYUFQLCwtp\/Pjx1LZtW6pRo8Y5U0i8mLdnz56IwLjmis4VJPVbFOxyzkfcyskLztyIxnhhlxucSbULERg3vMfhMkKnkHixLi\/iZdFSu3Ztys3NVRGYXr160YIFCygzM1PVgKed0tLSKDGx+OI0qeRLbaywy+GGFOPspPLFsHlp2+QV22jyim+J18Y4fXKvl3bF0h2l2iV1DCtXERheqLt8+XKaOnUqValSRa2LGTNmDNWvX19to2Zxw2nS09Ope\/fuakcSBEwsu4vY5C21E4JdsfGXWOZqAmfBQqZvUiN1dgyLmmgeE+yKpv4lvSvVLgiYWHiLT\/KUSr7Uxgq7fNKwfq6mVL68jsCEegELmYXrflAn+bKAYTFjd7GvVM6k2iV1DBMdgXGqG5dKvtTGCruc8nx38pHKl2kCxmKTBczCdbvU9BI\/vGup+H\/9nUtSOZNql9QxDAJGo4+WSr7Uxgq7NJzaoCRS+TJVwIQKmfxvitT2a35018tI5UyqXVLHMAgYjY5cKvlSGyvs0nBqg5JI5ct0ARPsAhyV4X\/5W\/cHIjOWoOGppuIpp4aBV6RyJtUuqWMYBIxGRy6VfKmNFXZpOLVBSaTy5ScBE+oO1jQTfx4aoWlWL55aX0BUt169iKadDHK5Eqsi1ReljmEQMBqtSir5sEuDfIOSgC+DyNCsijTOeCEwP1MW\/ItO1b88gIK1q4nFTftL60W8pkYTzpgnk8aXBZhUuyBgNJqEVPJhlwb5BiUBXwaRoVkV6ZwFTz1Z0Rr+31pTEwxTsMi5KCGemiVUU38OnZ7ShDYmyaTzFRPQPMwUAkYD\/GidWuf90tKE+1tZnwX\/XednDRjOS6JjF79UUjrdz0PTlWWPbr1Ksln3\/Wg5c9uu0rgIxgJ2FaOBNlZ89Hy49qbbRoL9yorebN93lF59ew1df911tL2oeO1NaU\/wmTUc4eGHBdDf\/\/53uvXWWwNCiD9\/8f7OFHxkPtrYJQoPO3zZGRPcfgcCRgNxJh8PEAACQAAIuI\/AsStuDhR6pvoF6ucz1eufUxHr83B\/i1WNK\/y0x\/GsK\/y01\/E8rQx3z78vZnl7lTEEjFfIo1wgAASAABDwLQJWRMlJAzg6Fatnet\/iq3EkPRAwktiELUAACAABIAAEygkCEDBREv3666\/TVVddpe5V8vvDt3M\/9thj9MMPP9Bf\/vIX+p\/\/+R+\/m3RO\/Q8fPkwLFy6kO+64g6pVK15A6Pdn9+7dirNt27bR3XffTbfcckvgDi8\/28bbWSdNmkTsk\/fccw\/16NHDz+acV\/fFixfTxRdfTNdff70Iu5ivZ555hipUqECXX345DRo0iKpWrSrCNu43nn76aVq\/fr2yi9sYHjMQgICxycOJEydoxowZ9MYbb9CsWbNECJilS5dSs2bNVAc0d+5cuuuuu6hmzZo2ETLrte+\/\/55GjRpFdevWpSeeeEKMgPnnP\/9JTZs2pdatW9P06dOpT58+1KhRI7PAt1GbefPmKQF94YUXqpvi77zzTjGcsSh76KGHaOjQoWIEzBdffEEnT56ka665xgbbZr+yZs0aOnjwIN1www2qr+d+sXbt2mZXWqN2kydPphdffFGlbNy4serz\/fZFHAJGg+jSkkybNo26devmO+LD2fTSSy+pW7h5AAz+OUqIjHl93759NHv2bHrggQfEDIZbt26lBg0aUPXq1ZWA6dWrF1100UXGYG63IvyNnjvVn376SQmYwYMHi\/lG\/+abb6oo59VXXy1GwPz73\/9W0ZekpCS7lBv7niWmeXA\/cuSI6jvYVinPrl27KDc3l3r37k2VKlXylVkQMFHSJUXAnD59mubMmaO2JSYkJJAUu4LplShg2L5Tp04RT0ns2bOH0tPTfdcJldQEv\/zyS8rMzKS0tDQxU2McfVm1apWKmvFgIWUKiUXZsmXL6OzZs9SzZ0\/1RSguLi7K3tWM1znqsn\/\/ftq0aZMa5CXZxn0Hf0Hg6FKTJk3MADyCWkDA\/AzW\/PnzqVWrVoEOhTua7Oxs5bRXXnml6kjr1KlDI0eOVHOhVrjNDwM92\/Duu+\/S8OHDlbXstPyt4pVXXlHf3B999FH1zYltuummm9Q3ehYz1s8R+JPrSXVt487UTwJG1y7+Rvjkk0+qdVg8cJj+DSoSu9g3z5w544upMV27eJqWudq7t3i7rOkCRtcunqKtVasWVa5cmZ577jklOk2fytS1LScnhzp16qSmaflnnqY1Ocqpaxf3iVu2bKHPP\/9cRW79+JR7AbN582a1sJM7Fp4P5A6FB3geFPjnzp07E8+B8j8WL1WqVDmHZ5MFzIEDB2jJkiXEC407duxIGRkZqu6WPaNHj1bf2idOnKj+tm7dOrUGhkOlpq87iNQ2\/sbrBwETqV0bNmxQ65Q6dOhgdP8TiV28PuStt95S0UAW0yav7YnErhEjRqjD13bu3Kl8sUaNGurLA\/umaU8kdnHfwYvI27Rpo6ZXXnjhBbrtttuoYcNfLn80yb5Ibfv444\/VQXAsYNg2\/mJnooCJ1C72O47csl2\/\/vWvTaJIuy7lXsD897\/\/pe+++04R2b9\/fyVauHMZN24cjR07VnWgHI0ZP3584PdgdE0WMLzwjMPwGzduVELFEjC8eKt9+\/aUnJysQr682+MPf\/iDikDxjhaeEzV9F1KktrGtfhAwkdqVn59P77\/\/vhLWvLAwKyuLWrRood0BuJUwUrt4GpMHd15rwAt4TQ3bR2oX+yE\/a9euNToCE6ldLMb4S9+xY8dowIABKhpo6hRSpLbxpgbuM3mc4DHi5ptvNtK2SO3iqDtvROH2Va9e8f1VfnvKvYCxCGMH5W+xLGA4rMbRGB4M+Nsth+k5SsGrz\/22StvqLDnqwgLm6NGjSoxxQ7RU96JFi1SDvP322\/3mv2ogkGgb7PKXL4Ivf\/EluV+U6ovhBicImJ9RCRUwvB6E173wNwtpAmbChAk0cOBAFXHhR5KAkWJbaCcEu8zW1uDL\/wIGbczsNgYBUwo\/wQImdMqIf+dpFg5p+zHUFty5Bk8ZWVNIbHuXLl18uQVSqm2wy1\/bccGXv\/gKjcBI6hel+iIEjKaA4UW8U6ZMUdNJPK3EUxTsFLy40PRdHuFMDHZo\/jvb89FHHxEvKuS1MSxgeHrJ1EV3pX0vkGob7DJzAWhJvgi+\/MVXqICR1C9K9UUIGE0Bw8msKAzvUkpMTKQxY8aoBb1+fEId2tpGzQuXeYqMdyP59QAqqbbBLn+1NPDlL77CCRgp\/aJUX4SA8V8bQ42BABAAAkAACACBMAhgES\/cAggAASAABIAAEPAdAhAwvqMMFQYCQAAIAAEgAAQgYOADQAAIAAEgAASAgO8QgIDxHWWoMBAAAkAACAABIAABAx8AAkAACAABIAAEfIcABIzvKEOFgQAQAAJAAAgAAQgY+AAQAAJAAAgAASDgOwQgYHxHGSoMBIAAEAACQAAIQMDAB4AAEAACQAAIAAHfIQAB4zvKUGEgED0CfP\/VO++8Q7Vr1w5k1q1bN7rvvvuiz9zDHL744gv65ptviG0ZOXIk9e3bV91pZj1sd\/Pmzen228+\/PXnnzp20atUq6tevny\/vPPMQdhQNBDxBAALGE9hRKBDwFoHg29fD1YRv5+V\/FSpU8LaiEZR+9OhRmjFjBqWlpVHNmjUjFjBc1KJFi+jyyy+na665JoKSkRQIAAEvEICA8QJ1lAkEPEagJAHDF3x++eWXtG3bNvrzn\/9MderUoYkTJ9Lu3btV5IKjGk2aNKF169bRuHHjqGrVquoi0Fq1alH\/\/v0pOztbfZ6QkKBucOebzx955BFavnw5\/fWvfyW+MK979+5055130oYNG2jBggUqj88++4zatWunLhaNj4+n+fPnKzFx\/PhxGjp0qLpIlfPjvPhGeP47PyxWrGf9+vX08ccf0+DBg4nFTGkRmKKiIlqxYoV6lW9kv\/baa+nxxx+n7du306uvvkoPP\/wwValSxWOWUDwQAAKlIQABA\/8AAuUQARYwL774YsDyxo0b09y5c5UAyMvLo0mTJtHJkydp6tSpSkA0bNiQeHrm9ddfp7vvvpumTJlCGRkZdOGFFyoxsWvXLiUcwgmYrl27KiEzbNgwqlixokpfv359uuCCC1Q+zz\/\/vPp9woQJ1KNHD\/rpp59Ues7\/xIkT9Nhjj6mpIK4fCxyOrowfP16Vd9lllwVsmDZtGl155ZXUuXPngIB58803z2OXbbOmkFiYZWZm0ogRI+jXv\/41HTx4UOX90EMPUaNGjcqhZ8BkIOAfBCBg\/MMVagoEHEOgtAgMF8IDPAuWu+66i\/bu3Rsot02bNpSenk4fffSREhhxcXH09ddfq2hGSREYjp6wOAl+hgwZQh06dAgIFf4bR38uvvhiys\/PV1M4LET4saazeHroqquuol\/96lcqOsNihqM31hNsU1kRGLaPozssVtq2bUu9e\/dWtvB7HHEaOHDgOeLIMeCRERAAAo4hAAHjGJTICAj4BwEdAfPJJ5+oKR6OgFSrVo3OnDmjoiMsbFavXk2jRo1Sgz7\/\/u6776rpnHARmBo1aqgpICvqwVGV06dPqykkK9ISLGD4s\/bt21NycrIClKMilStXVlNbXA7nxZEfjtYEP5EImD59+tBrr71GvHCXI0M8LcUPBIx\/fBg1BQIQMPABIFAOEdARMIWFhWp6hadTWrVqRbm5uUqs3HjjjfTUU0\/R2LFjz5lCuv\/++wPTLzzlNGfOHLW+hMXI0qVLlbhhIcJTPdddd50SDeEEzP79+2njxo1qWoff57pytIenjrKysujw4cPqfy4j+OHyeJ1O8BRSSbuQeB0PT0k98cQTShBZD+fNU0wsajCFVA4bBkz2FQIQML6iC5UFAs4goCNguCSOkvD0z5EjR9SUypgxY9R6Fd5uzIM\/L961FvHyoP+Pf\/yDXnjhBbWI95JLLlHrXFiILFmyhF5++WVV+dTUVBWt4WmocAKGp6l4uoijLRzhefDBB6lLly7qZ0sU\/d\/\/\/Z\/6PfjhRbz\/+c9\/1FbwY8eOlbiIl9f7cNm83odt4adp06ZKfB04cIBeeeUVJdqCp6ecQR25AAEg4CQCEDBOoom8gEA5RGDLli1qDQwLmFg+vIOJF\/3youBw25x5+ocXHfMi49DojG69eG0NR3GCz47RfRfpgAAQcBcBCBh38UZpQEAcAm4ImH379qmoCO8UYqFUUnRk8+bNalExT3OFRmjKAp7Xw7AQ4+iQtSamrHfwdyAABLxDAALGO+xRMhAAAkAACAABIGATAQgYm8DhNSAABIAAEAACQMA7BCBgvMMeJQMBIAAEgAAQAAI2EYCAsQkcXgMCQAAIAAEgAAS8QwACxjvsUTIQAAJAAAgAASBgEwEIGJvA4TUgAASAABAAAkDAOwQgYLzDHiUDASAABIAAEAACNhGAgLEJHF4DAkAACAABIAAEvEMAAsY77FEyEAACQAAIAAEgYBMBCBibwOE1IAAEgAAQAAJAwDsE\/h87s\/6hm6Q4gQAAAABJRU5ErkJggg==","height":337,"width":560}}
%---
%[output:87087136]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:6f969853]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00756 H\nPer-System Quadrature Axis Inductance: 0.00624 H\n---------------------------\n","truncated":false}}
%---
%[output:4ee4822a]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:7e4e4179]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:416a8410]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.003141592653590"]]}}
%---
%[output:9e0d4f6d]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.996073009183013"]]}}
%---
%[output:7721d2a0]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:0942af15]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.250000000000000e-04"}}
%---
%[output:6c05179c]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":" -12.337005501361698"}}
%---
%[output:7fe8c576]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.996073009183013"}}
%---
