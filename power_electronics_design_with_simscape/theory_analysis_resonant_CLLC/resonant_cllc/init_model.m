clear;
[model, options] = init_environment('single_phase_cllc');

CTRPIFF_CLIP_RELEASE = 0.001;
s = tf('s');
%[text] ### Global timing
% simulation length
simlength = 0.25;

fpwm = 4e3;
fpwm_afe = fpwm; % for PWM
trgo_afe = 1; % double update
fpwm_inv = fpwm; % for MPC
trgo_inv = 1; % double update
fpwm_dab = 3 * fpwm;
trgo_dab = 0; % double update
fpwm_cllc = 3 * fpwm;
trgo_cllc = 0; % double update
% t_measure = 0.648228176318064;
t_measure = 0.2;
tc_factor = 400; % tc is ts_afe / tc_factor
tc_decimation = 1;
delay_pwm = 0;
dead_time_afe = 0e-6;
dead_time_inv = 0e-6;
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
sst_num_of_modules = 2;

%[text] ### 
%[text] ### Enable one/two modules
number_of_modules = 1;
enable_two_modules = number_of_modules;
%[text] ### Control Mode Settings
use_torque_curve = 1; 
use_speed_control = 1-use_torque_curve; %
use_mtpa = 1; %
use_psm_encoder = 0; % 
use_im_encoder = 1; % 
use_load_estimator = 0; %
use_estimator_from_mb = 0; % mb model based
use_motor_speed_control_mode = 0; 

% advanced dqPLL
use_dq_pll_fht_pll = 1; % 
use_dq_pll_ddsfr_pll = 0; % 
use_dq_pll_mod1 = 0; % 
use_dq_pll_ccaller_mod1 = 0; % 
use_dq_pll_ccaller_mod2 = 0; % 

% dqPLL
use_dq_pll_mode1 = use_dq_pll_mod1;
use_dq_pll_mode2 = use_dq_pll_ccaller_mod1;
use_dq_pll_mode3 = use_dq_pll_ddsfr_pll;
use_dq_pll_mode4 = use_dq_pll_fht_pll;

use_dq_pll_mode1_mod2 = 1; % simulink dqPLL
use_dq_pll_mode2_mod2 = 0;
use_dq_pll_mode3_mod2 = 0;
use_dq_pll_mode4_mod2 = 0;

% single phase inverter
rpi_enable = 0; % use RPI otherwise DQ PI
system_identification_enable = 0;
use_current_controller_from_ccaller_mod1 = 1;
use_phase_shift_filter_from_ccaller_mod1 = 1;
use_sogi_from_ccaller_mod1 = 1;

% two afe in parallel connected to a dc microgrid
ixi_ref_mod1 = -0.85;
ixi_ref_mod2 = -0.85;
ixi_ref_mod3 = -0.85;
en_parallel_mode = 1;
if en_parallel_mode
   u_cm_comp_mod1 = 0;
   u_cm_comp_mod2 = -1;
   u_cm_comp_mod3 = -1;
   u_cm_comp_mod4 = -1;
else
    u_cm_comp_mod1 = 0;
    u_cm_comp_mod2 = 0;
    u_cm_comp_mod3 = 0;
    u_cm_comp_mod4 = 0;
end

% reactive current references 
enable_i_react_pos_steps = 0;
if enable_i_react_pos_steps
    time_i_react_pos_ref_1 = 0.45;
    time_i_react_pos_ref_2 = 0.9;
    i_react_pos_ref_1 = 0;
    i_react_pos_ref_2 = 0.2;
    i_react_pos_ref_3 = -0.2;
else
    time_i_react_pos_ref_1 = 0;
    time_i_react_pos_ref_2 = 0;
    i_react_pos_ref_1 = 0;
    i_react_pos_ref_2 = 0;
    i_react_pos_ref_3 = 0;
end
%[text] ### Settings for CCcaller versus Simulink
use_ekf_bemf_module_1 = 1;
use_ekf_bemf_module_2 = 1;
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
use_moving_average_from_ccaller_mod3 = 0;
use_moving_average_from_ccaller_mod4 = 0;

use_single_phase_inverter_based_FHT = 0;
use_single_phase_inverter_based_SOGI = 0;
use_single_phase_inverter_based_PHSH = 0;
use_single_phase_inverter_based_SOGI_ccaller = 1;
use_single_phase_inverter_based_PHSH_ccaller = 0;

use_system_identification_based_FHT = 0;
use_system_identification_based_SOGI = 0;
use_system_identification_based_PHSH = 0;
use_system_identification_based_SOGI_ccaller = 0;
use_system_identification_based_PHSH_ccaller = 1;


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
omega_set = frequency_set*2*pi;
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
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:575f45c8]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:7fce821c]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:3ee271eb]
hwdata.three_phase_dab = three_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:7f3f4e86]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:953c82fc]

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
time_gain_afe_module_3 = 0.9988;
time_gain_afe_module_2 = 1.0;

time_gain_inv_module_1 = 1.0005;
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

afe_pwm_phase_shift_mod3 = 0;
white_noise_power_afe_pwm_phase_shift_mod3 = 0.0;
inv_pwm_phase_shift_mod3 = 0;
white_noise_power_inv_pwm_phase_shift_mod3 = 0.0;

afe_pwm_phase_shift_mod4 = 0;
white_noise_power_afe_pwm_phase_shift_mod4 = 0.0;
inv_pwm_phase_shift_mod4 = 0;
white_noise_power_inv_pwm_phase_shift_mod4 = 0.0;
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
parasitic_dclink_data; %[output:31e84a7c]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:373fc281]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:3fa602c7]
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
%[text] #### Permanent magnet synchronous motor control with EKF based observer
psm_ctrl = ctrl_pmsm_setup(glb_time.ts_inv, psm.omega_bez, u_psm_scale, psm.Jm_norm);
% psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, psm.Jm_norm, glb_time.ts_inv);
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:22c35417]
psm_ctrl.kp_i = 0.25;
psm_ctrl.ki_i = 35;
%[text] #### Induction Motor Control
im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:26b0870d] %[output:9b0a60dd]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:26b0870d] %[output:9b0a60dd]

%[text] #### AFE control (sequences)
afe_ctrl = ctrl_afe_setup(glb_time.ts_afe, grid_emu.omega_grid_nom);

kp_udc = 0.5;
ki_udc = 18.0;
kp_idc = 0.5;
ki_idc = 18.0;
%[text] #### DCDC Control
dab_ctrl = ctrl_dab_setup(kp_udc, ki_udc, kp_idc, ki_idc);
cllc_ctrl = ctrl_cllc_setup(kp_udc, ki_udc, kp_idc, ki_idc);
%[text] #### Resonant PI settings
pres_ctrl.kp_rpi = 0.75;
pres_ctrl.ki_rpi = 45;
pres_ctrl.delta_rpi = 0.025;
pres_ctrl.omega_set = omega_set;
pres_ctrl.res_nom = s/(s^2 + 2*pres_ctrl.delta_rpi*pres_ctrl.omega_set*s + (pres_ctrl.omega_set)^2);

pres_ctrl.Ares_nom = [0 1; -omega_set^2 -2*pres_ctrl.delta_rpi*pres_ctrl.omega_set];
pres_ctrl.Aresd_nom = eye(2) + pres_ctrl.Ares_nom*glb_time.ts_inv;
pres_ctrl.a11d = 1;
pres_ctrl.a12d = glb_time.ts_inv;
pres_ctrl.a21d = -pres_ctrl.omega_set^2*glb_time.ts_inv;
apres_ctrl.a22d = 1 -2*pres_ctrl.delta_rpi*pres_ctrl.omega_set*glb_time.ts_inv;

pres_ctrl.Bres = [0; 1];
pres_ctrl.Cres = [0 1];
pres_ctrl.Bresd = pres_ctrl.Bres*glb_time.ts_inv;
pres_ctrl.Cresd = pres_ctrl.Cres;
%[text] #### Sogi
sogi_delta = 1;
kepsilon = 2;
sogi = sogi_filter(omega_set, sogi_delta, kepsilon, glb_time.ts_afe); %[output:28b97761]
%[text] #### Current control parameters DQ PI
dqvector_pi.kp_inv = 0.5;
dqvector_pi.ki_inv = 45;
dqvector_pi.pi_ctrl = dqvector_pi.kp_inv + dqvector_pi.ki_inv/s;
dqvector_pi.pid_ctrl = c2d(dqvector_pi.pi_ctrl, glb_time.ts_inv);
dqvector_pi.plant = 1/(s*grid_emu.trafo.Ld1 + 1);
dqvector_pi.plantd = c2d(dqvector_pi.plant, glb_time.ts_inv);

G = sogi.fltd.alpha * dqvector_pi.pid_ctrl * dqvector_pi.plantd;
figure; margin(G, options);  %[output:17a0e5ed]
grid on %[output:17a0e5ed]
%[text] #### Single phase inverter - with resonant PI and virtual DQ
single_phase_inverter_ctrl = ctrl_single_phase_inverter_setup(glb_time.ts_inv, pres_ctrl.omega_set, ...
    dqvector_pi.kp_inv, dqvector_pi.ki_inv, pres_ctrl.kp_rpi, pres_ctrl.ki_rpi, pres_ctrl.delta_rpi);
%[text] #### 
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
% Aluminum plate liquid cooled with a size fit for primepack2
% heat exchange made by an aluminum plate with a liquid flow > 28 l/min
% "A" as "ambient" here means water: so HA means delta temperature between water and
% heatsink surface
% moreover the delta temperature between water in and water out is maximum
% 5K assuming a overall power losses of 2kW 

weight = 0.150;                         % kg
no_weight = 0.150/10;                   % kg - when /10 is applied thermal inertia is not accounted 
cp_al = 900;                            % specific heat_capacity J/K/kg - aluminum
heat_capacity_hs = cp_al * weight;      % J/K
thermal_conductivity_al = 160;          % W/(m K) - aluminum
Rth_switch_HA = 15/1000;                % K/W 
Rth_mosfet_HA = Rth_switch_HA;          % K/W
Rth_diode_HA = Rth_switch_HA;           % K/W
Tambient = 40;                          % degC - water temperature
DThs_init = 0;                          % degC

heatsink = liquid_cooled_plate_2kw_setup(weight, no_weight, cp_al, heat_capacity_hs, thermal_conductivity_al, ...
    Rth_switch_HA, Rth_mosfet_HA, Rth_diode_HA, Tambient, DThs_init);
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
% nominal_battery_voltage_1 = hwdata.cllc.udc1_bez;
nominal_battery_voltage_1 = hwdata.dab.udc1_bez;
% nominal_battery_voltage_2 = hwdata.cllc.udc2_bez;
nominal_battery_voltage_2 = hwdata.dab.udc2_bez;
% nominal_battery_voltage = grid_emu.udc_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage_1, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:0fbb3f45]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage_2, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:337c0d48]
lithium_ion_battery_1.R0 = lithium_ion_battery_1.R0/2;
lithium_ion_battery_1.R1 = lithium_ion_battery_1.R1/2;
lithium_ion_battery_2.R0 = lithium_ion_battery_2.R0/2;
lithium_ion_battery_2.R1 = lithium_ion_battery_2.R1/2;
lithium_ion_battery_1.C1 = lithium_ion_battery_1.C1/50;
lithium_ion_battery_2.C1 = lithium_ion_battery_2.C1/50;

%[text] ### Load
trafo_load_name = 'Load Single Phase Transformer';
trafo_load_pwr_nom = 225e3;
trafo_load_u1_nom = 400;
trafo_load_n1 = 50;
trafo_load_n2 = 1;
trafo_load_u2_nom = trafo_load_u1_nom/trafo_load_n1*trafo_load_n2;
% trafo_load_f_nom = 50;
trafo_load_f_nom = frequency_set;
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
Simulink.importExternalCTypes(model,'Names',{'sv_pwm_cm_output_t'});
Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
Simulink.importExternalCTypes(model,'Names',{'rpi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'phase_shift_flt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sogi_flt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'linear_double_integrator_observer_output_t'});

%[text] ### **Remove Scopes Opening Automatically**
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
%[output:575f45c8]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:7fce821c]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:3ee271eb]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:7f3f4e86]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: Three_phase_DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 1.200000e-04 [H] | Internal Tank Cs: 750 [F]\n---------------------------\n","truncated":false}}
%---
%[output:953c82fc]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.580123e-05 [H] | Internal Tank Cs: 4.734509e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:31e84a7c]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAOoAAACNCAYAAAC5ZeasAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQmUVNXRx0tBRBQFAVkEBEEjAUQEAwgGNSRgBCMRRMAguCCbCwlHRUiEiIKGiNGgSDAQEh0lboghIiaiQJAdouyL4MIiq4qASuQ7v+K7Y9P0m349M6\/f6+6658yZme633Ff3\/l\/VrVv1r+MOHz58WKyZBEwCkZbAcQbUSI+Pdc4koBIwoNpEMAlkgAQMqBkwSNZFk4AB1eaASSADJGBAzYBBsi6aBAyoNgdMAhkgAQNqBgySddEkYEC1OWASyAAJGFAzYJCsiyYBA6rNAZNABkjAgJoBg2RdNAkYUG0OmAQyQAIG1IgP0rp166R\/\/\/5y0kknyfHHHy+7d++W888\/X37zm99IpUqVfPX++eefl7POOkuaN2+e9Ph3331Xbr31VqlVq1b+\/a677jrp2bOn\/v\/EE0\/I1VdfLbVr1056LTug+CRgQC0+WQZyJYD6l7\/8RYYMGaJg\/fbbb+Wxxx5T4HXs2NHXPVMFal5enowaNUrv99VXX8mf\/\/xn+frrr\/WFUbJkSV\/39DqIZC1+AL01\/xIwoPqXVShHxgMV4Dz00EPSunVradmypUyePFmee+457Ruar0ePHvo34H722WdV69apU0c6dOggzZo1k+nTp8uECRPk0KFD8tOf\/lQ1JYB0DY0aC1Q+37p1q9x3331y77336rk33HCDVK1aVV8Ys2fPVhB3795d733w4EEZM2aMzJo1SzU\/1+7Vq5csWbJEVq5cKZs2bZI777xT7\/\/ggw\/qi4A+cv1y5crpPRo0aCD\/\/Oc\/5eyzz5af\/OQnMmnSJPnss8\/kgQce0GfIxWZAjfioA1Qm+pYtW\/J7euWVV8rIkSNl8eLF8vbbb+vkpv3hD3+Q+vXrKzj4\/O6775ZvvvlGBg8erEA68cQT9fMBAwZIiRIlFOQVK1YUrlcQUL\/88ksFSdeuXQXtDFB37dqloOvSpYt88cUXqoFvu+02mTlzpt7n2muvle3bt8uvf\/1rueuuuxSoc+bM0X4fd9xxMm7cOOnWrZsC\/o033pCPP\/5YTeq+fftKv3795Ic\/\/KEeW6ZMGbn99ttl6dKl8vrrr+szFVWrR3zIE3YvckA9cOCAzJs3T6ZMmaIDTatcubIOfIsWLY56+2eiwFPtcyLTF40HSEqXLq0aplWrVnpZtCFApKFt3edoXNacaLQRI0Yc1QXWo0z+VIGKlt6wYYP+LF++XN555x15+OGHddx4KZx77rlq4v7xj3+Udu3aKVBpAJsGuFesWCFr165VrVy3bl255ZZbZPjw4apdTz\/9dLUc3HPEyyFVOWb68ZEBKoO6bNkyfdNedtll6vg49dRTVb6Al4GeOnWqTgK+z5WWaIK6z9CcP\/rRj\/KdRGis+fPnq1mZCKiLFi1SM9OBBZP1f\/\/7ny\/TFzMVQDM+aNT33ntPf9Cy5cuXV23OdZ3GPeecc7QffH7VVVcdBdQdO3bIb3\/7WzW9mzZtKmvWrJG5c+cmBComPnPBgBoRKpZ9+\/bp27lhw4aejgYcKQxqvXr1cgWnx0xQhguttX79ernwwgtVS95xxx0qD8DExObFtmDBAjWJ+ftXv\/qV3HjjjQqcl19+WbXWCSecoNouViM7rRzvTOI4TFDWs5jAAPWll17KfxnQB5xdaNQZM2boywAL6JNPPtE+YP7GalRAN3HiRD0HM\/bRRx\/V\/ifSqAbUI1M9MhqVzgDEzZs3q1OBifHvf\/9bVq9eLeedd55cfvnluvbJtRa\/PYOzBnkAADRZImcSWvJPf\/qTghITlXXgFVdcodoLkOOcoTnnU+yar6DtGc5xQMUJxNqT9SZrS9bQp512mlo8mNerVq1S8\/eUU06Rm2666Sigcu7jjz8u06ZNk+9973tqGtMvNDZ9izV9DagRAypv+7Fjx8qrr76qjg4mI2DF68eatUKFCrq\/x8SwFl0JvPnmm1KzZk0F6c6dO+X3v\/+9anScVtYKL4HIaNQ9e\/bI6NGjZdCgQerixzy7\/\/771YzCLH7kkUfUq4gWsRZdCWzcuFHHjnUoFhCa\/6KLLopuhzOkZ5EBKhE3mGu44llXxXr\/+J89O9YweAOtmQRyTQIG1FwbcXvejJRApIDKxjzOD9arRLZceumlunXApj0eYTbATaNm5DyzThdRApEBKp7A\/\/73vxpSlqix3iEkLRc9v0UcYzs9CyQQGaCyNUM8p9e2Lt5e3P8WzJ0Fs84eIWUJRAaoeH1J3SLm8\/PPP1fXPmFvNOJKO3XqJDfffLPuy1kzCeSaBCIDVCd4TN8nn3xSOnfuLGeeeaZ+DHhfeOEFDdg20zfXpqg9LxKIHFDZpiGelLC4k08+WUeJ7A1iRvv06WPOJJu3OSmByAGVQHFiVmEQIGib5kIJ77nnHtOoOTlN7aEjB1SnQUnNeu2119QLTBghQeUum8aGzSSQaxKIDFABJDmKyWJCcTIlOybXBtGeN\/slEBmgEuRAQD45qaRIEfjgKEIIISSVinzUCy64QH7+859n\/8jYE5oEYiQQGaC6PhHMTfIxKVoffPCBfkwmDalUbNFYZJLN31yUQOSAmouDYM9sEkgmAQNqMgnZ9yaBCEjAgBqBQbAumASSScCAmkxC9r1JIAISiCRQYQkYOnSoJpAT5IA3GNJmvyUcIiBX64JJoFglEDmgOib4Nm3aqPcXBjv4X8lPLUxkEtk4BE4QgkicMCwR0FdaFk6xziO7WMASiBxQHSXL9ddfr\/STMNKxn1pYKhbY8KB4gdqFVLlhw4bJL37xC2nUqFHAorXLmwSKTwKRAyoaFR5ZNCrFiQDqtm3blEaSNLhU09zQymhVqDFpsXVa+J\/6JtZMAsUhAZZsQbXIAZUH5YEhZ37\/\/fc1wAGTFfBCQZlqoywCLHjwAtNgk4eV3ZVxqNJphFxzzTWpXrbA42FNTPWFkqwD2XrNj3YfPObRP9zz3Wc1y5eW+hVERl7XOJmIUvqeYJriKh15aMcmOfTpJhnY81p5cuGnKfXD78GRBCqdRwsS+wuZtF9mB2qgkHlDPDC5rJi5f\/\/733U9CoM7gKf4ENdmvUs7o8c4M4P9zpYAjqt26rFlHKuV\/e6zLV8ckg92fCnv7zysd+9Q7xQZ1iZEjuA9H4vs\/lhkzydyeOO7Ihvmi\/CZiGz7uqRcPPWbAKQUoXzUWIaHRE9avXp1rVfixesLqNHC1PCEfZ1iSVQJQyOjQSlDyBqVcgyA1ZWBwPQN0mQJZNRy7KKx2q9\/3irJW7hNap5eWrpeVFXubnuEBSTItmfKcDmwcpZqTbQnrWSlWnJS\/Uv\/\/3drKV3\/Ul1GBTWXIqNRHWcSGpTtGABGoSMaHl8ql7Vv396TKf\/DDz9U7UnJPpxP1D2BwJuCvwT6P\/3003otyitQdGjgwIH6fxDCLU6zyk1Au+Z3Fc4\/3H1QwfrQjA8KDdhE8vxi1iQ59OlmBeXBFbPysZ8IlIleDEHMJXefyADVdQjNOn78eK3hGcvwwBq1d+\/evpjyycR55plnlGIUc5cSGZT2w+Sl+hn0o870DVK4Qb7lc+naBb2kHpqxSfIWbhXAi5ZdNrSFb9HsGNtLvtmxKSEoS9dvLSdUqqWa0m8Lci5FDqg4TTBxITJzziM4k9ieIQjCJY+zjYN2ZF0aW+MTzQoIqR\/6t7\/9LekaFeH+61\/\/8jsWdlwIEmD8WfoU1LZ8fkheW7VPnlqwV1j3djjvFOndrNyRUxa9eGQ9uecTkQ3vfneZ8tVF6jST485uLtK06A5FlEDWm76xg7Bw4ULdlqE0IM1VvC6oLLwLbGAtipmM1oQkjf\/RprZGDQFhxXRLv2Y\/5urGNatl5quvSLVD26XJV8uOMV8r9Z9YTL069jI5pVHd42O+4r1l7QqjQ6lSpQoUMHutBDXw5oWom6pigJ2yhFSDszXqseLzC4BUZna6rsl68uCKt9V0jXfy0F8cPZiv92xtltD5FEQ\/cwqoXt7fZF5ftLDjVcIRhTOJ\/dMqVaqocymda9QgJkGuXNM5dJx3VdeQBw9Kyc+35XtcASIOnlhAlr20p+f7xDmf3Fq260VVZFDz0sW2j+punFNAjWfMh4wbFkKK7VIkF8cQLX6NyrYLWpNqcJi9rhoc4YN4j1u1aqXnxQc8bOxs9VZT0ZiBH8u60bXTq4uUP1P2f7lfTm54ebGsI1nLjl+wV6at2nfsWraID5dza9R4eTkvbseOHT2ZCKmU3a1bt6NOPeOMMxTcW7dulQULFqgHuGHDhvKDH\/wgn3cpiLdgrmi\/RPM6U5597vJ1MmfbCYX2GCd69iDmkrtP5Ly+iQSAGYsziC0bL8pQwEwQf9OmTaVJkyZ6LIEP\/E0R5Ly8PN2D7dq1q2pdOJhoQQg3Uyar9fPI3iymMXuyRQ2kCGIuRRaosSat6yTkZkQcQcgN2BI1tCZl6AncB7SED1KSHkcSWzhUisMz3LhxY2nQoIECNiigFtGCstPjJBDECyWRkOPXsuzL3t22trCm9dOyGqjsm8KOT8OzW6ZMGdm\/f78GOzhQAjC2aPjOK4903bp1GrhPXDD7ohdeeKHup7KXWlBQvu2j+pmC4R7jZx+1uHuYaC3bvt4puq71alm3RsVhRPwtwGILxpmzABQPbeXKlTW4wUUmUY4Rp9CgQYPy6ULjnUlkx9x2221Srlw5BTPX5KdmzZq6NvUKyg\/iLRiEBrBrfhdCWBygTEWe8dFPXlo2iLkUmukLvQreWbZbyDmNTQdji2X69Oka28vnLjWNztavX1\/\/x\/ubqC1dulRNXQCNwAghZA+VquWLFi1Ka8BDKpPA76Sza4YHVDdGiWKMMYsxkYNeRqXdmYQ25ccLcCSOE\/5HrmCyIIfYSc4aFYZ9NDG1ashDxdPL1gy\/0xnw4Bd8dpw\/CQTxkvJ358RHea1lh3S5OPtCCOMDG3AAYc4SoMAGN\/ulbs8UcSULeOAY1qNEI5GBQzrchAkTZObMmSrtggIeLNa3KNM2+HPDWKP6farYGOPj9++UneM6+z01pePSrlFd7wAT5inrR0xaHEavvPKKfg0oFy9erMRmXi1+jcp+KcH8eH1JGsd8JieVeOHWrVt7BjwEsa4IQgPYNcM3fQtCFlq2+fWDZcv0MSkB0O\/BoQG1oHQ28k5HjRqlnlvX8OZi2npRnBDwwPoWoGJWU8MG4FJUCqdULGcSLweCJ9y6wq+w7DiTQDIJZF32DNsyv\/vd75S+06UwEdgwevRoIWyQ7\/Hiuga4CLbnd6LGuhZws3+K+Yw25YfKb2hu2B9oDzzwgPTo0UPq1auXTOb2vUkgMhIITaMigdh0NrZUoFPBcwtTA3tSmLN+G6bzO++8I48++qgWP8akHTx4sFSrVk15fceNG6fmdbLACb\/3s+NMAumUQKhABTibN2\/W8opQpqAt2UOFGvTiiy+W73\/\/++mUhd3LJBBZCYQK1JdeekmmTZumXtq77rpLtSHUndRGxRlUo0aN\/GAIP17fyErZOmYSKKIEQgMq69AxY8Yoaz3lJthWIZSQaCVA6xV8X8TntdNNAhkpgdCA6ry+rENxKgFUAhzg5WVdSU5pbGObheAF0tS8giUycgSs0yYBHxIIDaisTylZQUghmS19+\/bVYHpAyA+RRkQYYRbjDGJvlHPgQyKFzZpJIJckEBpQETLeWUiyWasSjXTllVfKj3\/8Y3Um4Z11e6aYyZjEjsWBvVJrJoFckkDagRpPtRIvbPZP0bQw2TtWfMxk9lf79OkjEydO1KCGRI3tHkxnUuIIxidBnGAHK7uYS1M6O5817UCNjfFFU5LmVqvWEaKqXbt2SadOnQQKFTy\/MNqzvwrTPdFKxP5u2bIloekLCyGB+BR\/YotnypQpqrHJRbWyi9k5eXPpqdIOVCdcQATvbufOnXX9SSP4+oUXXtD1KiAmGRwNzPdQhvI3GjKRMwltSgA+yeKAm3Ohb4GOBUeUV9nFXBpse9bMlUBoQCWonmghTNzY0hVs1UC5QrYLGTVuLbt+\/Xr1DnsViQLkaFRCBdHIcCQR5E8eK0WhvMouZu7QWc+DlACFoeKbozAt6L5BEXyHBlT2TFlPkncKMGnQglITFYC2aNFCE74JzP\/oo480uP6SSy7xlJELIWQtC1PEVVddJZjZfrJnghxwu3b6JdC2wj45v+wBqVLqyIu+8omH8v9O1htKJ3q17V95f+fO6fj6gWS3KNT3oQGV3uL0oQI4zh5MYbZjyGrhMxxBmLLE7GL6ss6kUptX9gzgBqBly5ZVriUYH8imYf3L2rag7JnizniwlLTgU9IoX3FgxdtKyn1gxaxjyLlLnlFLizzREmm5IMYoiJRJh+q0AxVAEnzPmjNRc1k1hBKyTgV8OJVgGMRMhgU\/UWPfFQ2N6Yt5jBlMHipmMJQsXtkzQQq3UK9OO+kYCXywZLZU\/HxDfgkLVxLRlUP0AmO6RRnkXEo7UNF81D\/Fk0t+aZ06dZTQjEbww8qVK1V7Orb7e++9V8snohFxMnlFJWH6okHHjh2rpjPHk86GY4m8VNas\/N2lSxdNSHfsEUEKN90TJVvu57QktWUocUEDlE5LBrUOLKr8gpxLaQeqEwaJ3c8\/\/7xuw2CG0ODvhRibLRqnOeNN2lSFuWrVqgK3Z4IQbhBmVTZfM7bgU6y2BJjVhr2l8wNfRnG2IK4ZxFwKzfT1ErZXcSh3fGGzZ3gZoG29tmeCFG5xTqxMvlZs4SeKPtHiK3oXpC2DAFUQ8gxyLoWmUeMFBVBZg2LyYhJTH8aZxByLgwg6Fi8Cbi\/Bs1YtiID7ycvLSqdrOgUxbild003glE5K08GUNUzU\/GxXOLNVfydx8Hg9jgFVJDJAZZAIaICOBS8wJjHUK5jBbNGkQh0aO+AAtaBqbi+3O0kaXXBBmqa89222lqgceh+8OrB4\/9GOv\/Gn3aCHwsDnpzl2+SZnlpZqZUt+Vwncz8n\/HwiTrOK4z0sFeljWMeX7kRbmKuGCOJ5ILsfz+8tf\/tJze8brmmz10Gx75lgJBaGpEl0Tpnna3PV79PecDXvzOwN5ddeLqsrdbY9spSRq6eqnn3lZ0DFZa\/pi7hKADzHZzTffLDh+fvazn6nJG6tdiwJUrpnu7ZlMmVhh93PO+r0yd8NeBTDgBbTLhrY4Bgth99MvgLMSqHhz2Rtlewa2QLZhSHkjFJD6MVOnTk3Z9E2UPUNhKdue8TvVwj0OzUv5w3gtGwRQg3jSrASqi\/Vlr5MYXhxJlKNYu3at\/k1sbirOpMJmzwQh3CAmVi5d0wG2VZ1y8mr\/xrY9gzP1MIvBEBoRSE888YQ6iwjEB5xw7hIIQchgbDkLupdse6aw2TMG1GjuT8I8f8GIeapdX+5exfZRwwKqOhXmzFHzd+\/evVoZHLAB1latWqX86ihs9kwQQE2583ZCQgkA1gF5q2Tjjn3y\/jDvhIyoiC\/IuRSaRnXCRbOSO0oMcN26dT1jgOMHI772DMyFEHAXJnvGikRFZaon7scNeZtl99clZFibisIWT1Rb1m7POHMVylCC5sliYQuGMhSpNsueSVVikjFrP9bnd0zfq57hV\/s1llZ1y6X+sHFnBLHmz0qNiiZ9+OGHpXfv3urpXbNmjeaQPvbYYzJw4MCUeX2jlD0TxCSwa9aW\/nmrJG\/hNqHid0H7rn5QHIQ8sxKomK5kyZB3Sl3Tpk2byrnnnquOJUjMvNLZ3CCgQWEmhLnwnHPO0Xjexx9\/XFkjoBjlWuPHj1f+JNue8TN1o3tMLKjYex3w3CrtbLJAiXQ\/UVYCFYYHwvvIN2VrZeTIkepMWrJkifIeeVVtQ\/iUWCRtDT5gwAhQyV0dPny4ms4QeLMPS3oc7BEFkZsFKdx0T5RsvZ9XtFPewq2Cw8lPdFM6ZBPkXArVmQTDw3vvvadar1KlSvLWW29psneychaYycQEwzRI9TcHVIImKNWINuW7zz77TAP50509E4RZZddMvI0EUDGHXXSTAyTgrVm+tNTg9+knHWMqByHPrAUq5itsD24rl8RxYnNvvPFGTxIzNxAcy1YOpNwAlfbmm28qUPEgN2rUSE1jzOqCsmcQrrXsksDXNVvKt2UqyrdlKhz1289THr9\/Z4GHHb9\/V4Hffzq5j5\/bpHxMaBoVkLKe\/Mc\/\/qFmK2GDmMG9evWSbt26HcXkgIn81FNPKfiefvppXb\/GAxWHFGtRyLkJmKAaHKGJycjNUpaYnZATEnCJBPEP++HugsnLxnYNpkB2aEAlIB\/nD2tK2AepAA6oIDQDrMkKQcUDlXUrVCwAlXNhkAC4sBdCR+qVPZMTs84eMuMlEAmgovkqVKigGpPtmVtuuSWp1zceqGTgjBo1SoYNGyZVqlRRbcoPe7LPPPOMJ7lZ\/AgC8Pvvv1+LVLFNREFla9GTAGvMRx55RIkEIFm\/6aabCnRARu8JUutRaEBlXYo5i0OpTZs2MmHCBK0wvnr1al17etGCeq1RHa8vxZBZo7L2HDx4sHqASUTHzOYYik\/hCYYxIlEjYb1mzZo6+NS5Qbsn60tqIreji0MC8D\/DBtK4cePiuFzkrxEaUJEMOaesS0lFW7x4seajXnHFFeoBDquxBgbIVatW1fWw+zus\/th9E0tg\/vz5qk1xFOZCCxWoOJTQoATlu8b+KRQsBe2jBjUwBEqQyA6nMA4rvMZt27bN9yoHdV+7buoSgEwA9g+sJEjbC7KSUr969M4IDaiYpzh78PjGbpGw7wm5WXGbm2hrt33DMPCSoIjUs88+qxodrl\/ezpi7HTp0UK0OaN3f0Ru67OyR33GCpoeqCDgg8WuQ14wVlK0tNKAmKhIVhJAJeiD4gWLJ1EylLCPNOZsIkqD0I+tiviM6ijUqe7M4oXr27HlUAnsQfbRriganpDJOOA9xPkIu4KoC4kTM1hYaUBOVXQxCyGhs2PdXrFihgHRAjWUnxHwihJGqb0Q14fUlrNG8vkGMSOJrpjpObLmR1EGlerKvMH+9HITpe4rg7pR2oJI148L7AA6BCZdddpluz9CCMn3ZZ0WLAlS2djC7u3fvLg0aNND7EhHFQFPywlp4ErBxSiz7tAMVTy9mjhcDTGGJtpNNrfgJMGLECA0\/RIMaUJNJL33f2zhFBKh0A7MSviQC8qFd8RPbW9SpEjsBYk1d7s\/\/mMJk6OeKu7+o8gzqfBuniACVtSlrQPJPKVtBMAImMFQqycIGizI5YieAcyZRKBlSNe4PUDGLs9khURT5petcG6eIADXe28v\/OAXwviZLbyvKZImfAG57hiJSOCa4v2nToki4eM61cYoQUB2zA651gErC93333Zc0vrd4poJdxSSQeRJIuzMploLFgJp5E8Z6HI4EQgEqmQ7Lly9P+MSxOafhiMTuahKIngTSDtToicB6ZBKIvgQMqNEfI+uhSSC82jMme5OAScC\/BEyj+peVHWkSCE0CBtTQRG83Ngn4l4AB1b+s7EiTQGgSMKCGJnq7sUnAvwQMqP5lFciRBIDceuutyh1VqlSp\/HtAe5rpxF2kFcJrdMklqdc2pRTnjBkzZMCAASoTQgvhbqbwdWyDwJ0Uxa5duxY7K0ggA17IixpQCym44jrNTwglqYGk\/2VSYjS0q5MnT5Z+\/frlM2SQpcQP4E3WYoHKOVDkAHhXFSH2fF4I5DlTMCxbmwE15JH1AirJ7eTMkjAAxzDx0PPmzVNaVRIKIPOCJgbOIMf9VL16dalVq5YyOW7evFmfjET4WA7k2rVrK4CgRSVbCXYEyl2++OKLyoTBvSiudfXVV0vfvn2VQQFOIhL8eWFQwGvBggXSsmVLrRMEQOBSJtoMEnXXIB8DkICHxAeuDX3KnXfeqf1\/8MEHldYVbirivOHNggaHv2nNmjXT2G9KlMD+QDEw+J6prDBp0iQ9BlJ1uJIgxxszZoxmPwWZ2BHmVDGghil9EU1KiA+phFAN0xcWCgjAIXtbtmyZMlRgCpYoUULBVrFiRTX3XLoegKRoFvm9XkBFU2Euck1YF8lcAthr166VOXPmKCUNgOTecCDPnj1bgcVLARBzPP0DrKQmcj1eFACM2G0alfq4DmyOMGgAVHdtrAI4lilbAhnZG2+8oSYtLx7uSRYTJUlgAQHYgG\/p0qWau4xG5UXFMbBUktzRrl07PT72fiEPaSC3N6AGIlb\/Fy1Io8YWwYIREQ0b21jbohXRPiTA01ivoVUTARXqGb7\/61\/\/etR1mOSuOSoaqFKhyIGlkfWfo6zBDEWDcQ7lSABa+fLlFbyuxVcxAKhOu\/ObFwUcVrwceBHUrVtXtTOlTSBNB8x8xxoVjQrJHDHgPBcycWYuz829aeQTc43mzZv7F34GHWlADXmw\/AIV4GAmOiChtdCImKWYoQ6oAJqKA4mACuAAKWatm9CACg2NKRwLJu7HxAfYjrIGkAIyp83OOussdfJQeLpGjRq+gOpqAqFBIQ+ghObcuXM9gUqlgrFjx6oZjllLH7Zv365mMibw0KFD1elmQA15Imf77f0CFdMRMLFWZV0KkNAo8E+hndBusabvrl27dEJjBlNLdtCgQWpaUigaEHM8nmbq9QAGPo8HKuTj1Kx1hHNoOaoHYJZjlqLdABssGbHsHJjKo0ePlvbt2+ebvu7amMo4hoYMGaLnUIKEhranf5jQlCHBtIeyh\/Uza3OegzIWkG5jcvNy4aXEmhv2yNj7ZeOcMY0a8qj6BSqTn3VbvCOF+ivwT1HJLtaZhHMGbQNY4SnmONavHMPxkJFjYrpaPFw7EVBZB6OtqO6OY4trYoZifgJ2AJTI3MSZRMMkjjV9cSBRxY\/vqe\/DGpN741yCVBvHFC8G2Da4H\/2l\/02aNNG1MgBGywNU1qy33367PhuAx1HlTOGQh7XYb29ALXaRhntBQIFJGvRaDW0HOHD2JAIHJq5j8ijuqgfxEsbJxgsPbt9sbQbULBvZdACV9SGmKyYolfi82n\/+8x\/9KsjSlayZqSrPNlPQL4Qwp4oBNUymtz3JAAAAbklEQVTp271NAj4lYED1KSg7zCQQpgQMqGFK3+5tEvApAQOqT0HZYSaBMCVgQA1T+nZvk4BPCRhQfQrKDjMJhCkBA2qY0rd7mwR8SsCA6lNQdphJIEwJGFDDlL7d2yTgUwIGVJ+CssNMAmFK4P8AhM0xpE41IEMAAAAASUVORK5CYII=","height":141,"width":234}}
%---
%[output:373fc281]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:3fa602c7]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:22c35417]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:9b0a60dd]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:28b97761]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAOoAAACNCAYAAAC5ZeasAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQmwXUXRnlQJIpCogIFKRIkY0BJQspRaKgjI4kIIKRUIe0gICApYIRAggIACCkbEsIVNRUDKMhhEUXYoUEohWAiIgAsIGkOMghaRqNf65v\/72m\/eLD1zztxz7n0zVa\/ee\/fO2tPfdE\/PdM+oCRMmdBRLZ599ttpnn330Jz\/96U\/VzJkz+ddD\/n7nO9+prrjiCrXRRhupc889V1166aXD8o4bN05dddVVauLEidZ6nnzySXXooYeq559\/Xi1cuFDXN3nyZHXOOeeo17zmNd166Tvku\/baa9V73vMe9e1vf1stWLDAWu+ee+7ZrUM6jlWrVnX74ho00YfaJhrceuutw\/pCffjqV7+qacP7hPqpDk4jaV+r0JzTjWjpnGSl1Msvv6xOPPFEddNNN1mzpfAMp7WkD3\/5y1\/UYYcdpn7xi1+ouXPnqh133LHLm7x9dJB4keiN\/lPZ0LxyfnTlpf765opjg2OJ+vSHP\/yhy2vm+PlYKf+oTqfT4RNBTLPxxht3B8cb5Z2jz3\/9618PIxqv02RQTgBOmK997WtqyZIl+msAFkAFgzzzzDPqhBNOUPPmzdOAxsDQNjGPDZQp4+DMw8fM+0htLVu2TAPTlY8T32QcjMsG1CZozucCAACdzWQDdlWeiQUq5zvQdqutturyp8lfPqDyRZEDguYxhgdoAQ7xAKcV8YVkQeD1aqBiYnhBrFBvfvObu+AzEU+EMKWJaxXxSVXeLtq5++679coNKUyM+6Y3vUnNmjVLHXXUUV2gQqLakjmhyENaQWgcfJL4Km1bdHIAFe1L+0oSPIXmEm2Jxm9KMg7kKjwTC1ST57hGYfKXD6jmYmQuQuA50vBCPEALrkvwUHmi0xve8IauBioBKuqlOrpAdakWNpCZah+koE3t5QNwqTcmUH\/\/+99rScUXCxB3xowZVlXBp\/6iTQKqZBw+5jEnPydQJX2tQnNzgXNtWzAHkyZNsjIu6qjCMzFAdS0WfFHmoPIB1eRDc2HnQHUt7NJtlblN4ouEyfdc8Ni0FSdQt9hiCw2Y3ExjdpiAhUFRH0AwGwE5g5mrF6nIBHzJOJoG6kMPPdQTmvv2o6aaP3v2bHXkkUdqTcbcC5oAiuGZGKCG9u0YDwdPr4BKizXaJxBymoBe06ZN01u0Bx98cIitxgdUU+NBPbWovrS6+\/aiLuYwO0x7D+SHyksDxP+kkriAaw6QS1R8F6P6hlbTFIlqqqmmMSlG9a1Cc5saSfNjqvnz589XRx99dNeI02vVN2TIon7zhdgHVHOxMefA5DMu6Xi9nC68Ttd2wMRGElCrGga4MclmRDEZ1KXTm0Yi5LNt8k2gEqHMSTWNDiGjWC6JajPG8JU3xfBVhea8PybjculFUqJJYxJnaJuxyzYWqTGJ1+3iMwlQeb9MLe93v\/ud3haadI5WfXMfz\/ANsU2q2jrMB0tEMPNxq69LWkuOcXyLgWuScHQUkqhcHcORgu+Yq+7jmRDNJVKKL2qh\/JwJfTYDKa35fIaOkmxgM7UNyfGMzRob0qrI6uviv1NOOUUbRleuXDnsiFJqTCL+H8WBajP4mPsDTAqSeTRhqp34P3SGijw2oNqMBLZ8XCrRPmHOnDlWs705Dr4Hjl1NXUBFH\/gi4zM68LNAk065aU6M5dr7mas\/5bcZnSQ8E0trmxru0spsWhRODvi5NQcqxkZSzsajPklnSmqU5+fLmO\/TTjtNfe5zn9MAHT9+vHruuee6+1eXJhkSNPi+u0d1ZR7kz317tUEedxNja5LWpoGHLvQ0QYfUNgceqLbLCjj\/Cu19Ugk6ksu1ldYFqEqpf\/3rX3q\/tnjxYvXb3\/5WTZ06Vd8gmjJliho1alQr+NZ1TihVz1sxiD7pRBtpXYCqlPrBD36g9fJjjjlGvf\/971dLly7VxyCXXXaZ2n777fuEvUo3CwXaTYFKqu+LL76oz9ggRfEbEvSf\/\/ynvlz\/2te+Vt8bfdWrXtVuCiil\/vrXv1r7GPP56173uiF10P\/m560nRulgKylQCai\/\/OUv1RFHHKEuvPDCIdLz+uuvV9ddd526\/PLLFe432tJdd92lP\/7b3\/6WRBgXiKiyGJAldSCiEAct7lAjrbfeet0FYrPNNtOf4Tf9HVF9yToCKFAJqHfccYc2RV999dVqwoQJXXLB7I99qs+9DeVKUmrNmjX6BwsL\/WDfj4Tf0FCQKF+hWbsp8Jvf\/CZLBysBFYfRkJ7kk0o97CVQQ6ql63vX51DZeUK+P\/3pT2rFihX698MPPxycCEhLSvib\/wQL\/z8o0RZ+AFCeSOK+613vUptuuqnCb\/y4Egx8fBGVtF\/ypFHgLW95ixo4oG633XbqnnvusVIkBL5YMsYwK4GRwAlg4m9p2mSTTdQb3\/jGrgqLg3pKABmpvARAOA3gAN6VkA\/tk7R15SO1Gao16EcAjhm7dIy+fFXbiykvyevL4\/pO+rmZr7VAvfnmm9WiRYu0irv55pt35w8SFYYkUyXmE5xzUCYj2QhP4AMQccVPKi1RN4ECvwFELuVSmIf20+jLr371Ky29bYlAS78lwMLCgaMyifSV1BfKIxl\/XUCXtFWAqpRavny5+vSnP60uueQStc0223TpD3UYxzaI2DBmzBjrvPQSqKRG4ncKKCGdCJC9MPiQ9ISkxXU4F2jRF+RB3zAuiVrOFxo+Jp\/6HAJn+f7\/KJCTpyvtUVevXq2jLuy+++7qoIMO0sczdPfyrW99a\/fIxjaROQZVFZAEQmLgtjAvgEuS32bNhqqLPsMfFKo1qesx4CUAS\/e+BZzDKZCDp6mVSkBFJd\/97ne1Cw9UXTALjmUgBXDhYcstt3TOZ5VBmWorGonZSwKQBEL0GX\/7jkUkKhYNVJK3ijoWAu2GG26og8MBtPhBW7gyyVX7GADzxQtjzG28ktCvV7QObaH6Zo+KgfArhM8++6yODnj88cerbbfd1rvoSoFqMljVvWRbpGQdEikEWkhaGJc++MEPagOTmbjhLAa8RX22z56Up1PmvrJETWnUps9XVVtptScra0hKpva7reUItK49LanHoIvPqk7zQPvdFACT+kzSeJAWR9\/8DxxQwQy77bab2n\/\/\/aOsrXwl5xbXkcII0kWiLtDy9jiAY86UqQ7absD6PKgAbhSojzzyiPrSl76kg3HDWASAwVkWTrGm6hvjPbPTTjt5+c52BJIKyJh9j61TMeUleavsUWP3TZg\/hJgJSVqoxynJVJ8RWPqFF14QV2Ua8FBwjz32EJVvG60bA+oTTzyhDj\/8cO0Vg2h0mACcm+Ki\/QUXXKBe\/\/rXJ3vPfOADH9D11AlI1+xKJtTHGTHlJXl7CVTeFo5yfEc+pB6nghY0pPZMAJNEFiGQnVXzrYy5ULeN1o0AFXG5IUmfeuopdf7556vRo0drGj\/++OPqkEMO0ZZeHKSnes8g2iACdJWUlwIuZsYeFLeiXGevdYDWNrJeADgvRd21NwJUOg\/FmzEAIyVywt13333VO97xjmTvmZyDamqi2thuSOrQfhZeTL0GrbkH5ue\/+E56gYPbLnwSOPf85ORpp9UXEvWll15S66yzjj6Ho0QSFftUXGBP9Z7JOajQPi52wkLMzuuT5G1K9Q2Nm0Dru38ckrSS8cdsM8y71zHHc2gHVyixxeIXOfA5\/pfe6aX+mvlbe44KIp166ql6n4MLDQ888ECy9wyAevvtt4d4p3xfkQIw7sBJIDb9\/e9\/V08\/\/XTXc8hWHpcrcKmFOx7EtpOaH\/YSGMlwNxoJ9hT6W1onQIyfrbfeWv+mv6XlzXy77LJLs94zuNRw44036lCYMCBh74rJqeLm1kuJmkr4QShXVcKBBnVI2l7Rsuo5cBU1OidPa9XXFvOWni5AXFKot4jIsN9+++n9KkVt6BfvmV4xSRvbqQOofFz9BFreb37tlDtnxLgwmr7A5nlwdqC+8sorCqoOpXXXXVdBrfnzn\/+sPvOZz+goA6effrqCDymPLNgv3jNVmTWmvCRvW\/eosQtNDtBK6OfaM9r6L6G1uQ\/G1i5WjSZPJgT3yxFOx2tMgpvafffdpxC6f+zYscPo0DbvGRejxUx+7GTHMjfyV+1PTJu9aisGtKFrjNLxVR1byJhkqtGwyYQAfOedd0q7H5XPCVSAEJcdcEtk7733HlYpLMH4acJ7JmqEIzxzVWZOIV\/MkU9doE3pZ2oZU42mK5U\/\/\/nP1b333ptarbecE6iYYFxsgEeMLdH7JLm9Z+oYdVVmjSkvyStRx8xxh1Z\/lzoo6U8dNHa1HwtaCAhpjCfJ2HpJ6+x71DonSlpXzkFJmVzaVwlD1L1vko4hdLYX03cpPXz5fO1JQGv606a2JZkP6eInXQRz8nRr3NzqYJJSx3AK9BqoMXMgvXtMTvAxdTeRtwC1CaoPSJttBion8SCAtjVAheHommuu0RHwN9poI03n1Eeicg5KqjZKsRjD7JK8vdw3SfojpYMkX9X2UB6XanDPN+Sah\/hQ\/LFpW\/96SeucPC1WfXGdDFZgRBXkAbdTH4nKOagC1P+9WlAVOBJw8jxV2zPLS53gKZZxzNwP3B4VFx7OOOMM7ZYGCUpArfJIVC+BGstsg5S\/KnDaRAtJjCjsZ+EsUsWnNnXMOXlaJFEhNXGvF65tuIxPQK3ySFTOQaUSehDLDRJQ+fyEQIu8AC0Fd+vF3Obk6SBQsck\/7rjjtNcMJCsu5hNQqzwSlXNQMeqPZAJjmF2St5f7Jkl\/JDSQ5qnaXkx5yisBrc09b2BUX6i5X\/jCF7QqgUDbuHnBgVrVe6ZXbm6prl7EnDHlJXl9eVzfST8380n6IwWhJF\/V9mLK2\/LGuOdtvPHGVhfAVFo35uYGiYn3YxCKBR4z8LKpE6i5Xr6SMNRIyRMjoQaNJjH3j+vY0+bUEp1ubgcccIA69thj1THHHKODmyGZQC1ubu1n7ZEMVNueVhK9IvX+cXag2tzcYCiaOXOmkxPnzp2rQ4f2wyNRVZk1prwkb9mjuhc4Cf2otCSvLQ9JWt\/LebaA5WZd5v\/ZgWojmwle5HnooYd0mFC4vUEVXrt2baseiXJNv2RCfbIxprwkbwFqs0DlQKfLFZLgbtjT8qdaWgFUGylN1Rd5iptbu9VfycLR7hH0rneS5y75y3k4\/uGpEYkqBWo\/uLn1bqrb11IBavqcxN4\/bg1Q04c8vGTOQZmtVWXWmPKSvEX1bY\/qa\/N9de1rETcMj2bZEiQt7iZfeeWV3ke4UjEUvPCQWnGoHIBaUqFAP1IAPrN4qNsWJvXggw\/WN6LqTo0Bte6BlPoKBZqgAHcagFTFcWaOVICag6qlzkKBmilQgFozQUt1hQI5KFCAmoOqpc5CgZopUIBaM0FLdYUCOShQgJqDqqXOQoGaKVCAWjNBS3WFAjkoUICag6qlzkKBminQBSr8TC+99FJn9Xvuuac655xzhjxqXLUvTz75pDr00EPV888\/r6699tpgRDlfe\/QSuuvmiNl\/Gm+OcVWlS6g8HPYXLFgwJFuvxkFt47CfB7kL9Znzl60snz96hSFUJ\/++Tl4y23355ZcVHu6+6aab9JOjixYt0jzrSlV52VavGKgoHDs5IUL7iIuXzW+77TbtRidJIaCa\/e9HoHKGqYNJUN9FF12kfvSjH+kwO7hVc9BBB+kXul0pBai2uTGZuc1AJT6Fx9gpp5yiLzX4gAramYvNypUrdYBAPLgM+s6ePVtNmzZtCK19PDkMqLaVmb+fmrLauSbdBlS4G91www3aKwde9\/ROawisfKLhK8vLcQlE\/e9HoPJ54IzO6RizmN5yyy3q0Ucf1Yy3Zs0addppp2kf5MmTJ4fIHfW9TQMw56jNQCVeAe9MmjTJqQXyhdScB7yMiJff99prL\/0w9JlnnqmFEL9rXBmonBFMoNomAUz02GOPKawiBBhTGuANVqzi+A2\/1m9961vqve99r0II0h\/\/+MfqrLPO0n8jjRs3Tl111VVq4sSJTgbxAZX3nxiEE2XWrFnar5ZWSdtiZdsa2FQcGz1si5v5eLQEYK6JxEXwe+65R\/9wWsGzCX1E3Cv4F1Oi8eE7vHk7depU\/dWnPvUpBfBSQr\/hQYItEZVZtmyZVrsl\/UU9fN533HFHXTUur5vlbUA1P6O+UP9cixU+Rxt8K2fOFecJky54pZAS9QG8DB6EAAGvIjYTr5O8yABACJsNNthAfeUrX1E777yzflP4O9\/5jpo\/f363Xlw3RPm3ve1tyqUNUv3oq0iicubjnfPtazfddFM1ffp0DVSJWopXzQ888EAdltTcf9kmxkRsKlBdyCfGxPe0P7Hl5SD00YNLEFe+0IJE5Xg+vAiPkDlTpkwZpn1gDwmQdjqdYV0321q8eLGOjRWiRyxQzUUe9dP8cl4KAdXVL6K\/DXhmGWrPXCR5PnMBobzgBzzqDe3jmWeeGQZUhNRduHChWn\/99RXmBEJl1apVasmSJVrw2Ow\/NAdwSD\/ssMOGeeagv0jQcqL2qJwpbeow9pRgahAd6WMf+5iOBsEZc5tttlG77rqrbhx6Ohl\/PvzhD2tVAJ9BsoEwGAiATuV9q3iq6ot+usAGQoGIZPCy5SNAI3KdzTBm9h1BzCnEDdXHpY7PKORisP\/85z\/qyCOPHAJUvBYP+r\/wwgtd6YVVHsx2\/\/33q3\/84x+axlgg8UwJ5glSgbfPF81UiUp1EFOC3kQnvniFgMoXFp6XeAKgoHo5n3AA0xgwVgDHlY8AzecFaiveRYX2N3r0aA1UV0K9F198sfryl7+s6fzDH\/5QZ6X5xtzgzeE\/\/vGPQ+ht05josyig0sCgGpCU4ROLt1QRa+m8887Tr2pTflotZsyYoQOkXXjhhWr77bfXfxPTQlc\/\/vjjtfqFSYGUwDMaAKrEoieR2nxiXOB37QPNSTHLc0ZBXpflz2WMoXZDUtWncfB2r7vuOnXyySfrbvO+XH\/99Qr7JSyG0HpAd7htYZE127YtIDES1Vaeayh8PkJA9dkdfAuqdBG0be\/oM4TLPfroo9WrX\/1qHYoIvO0DKoESmgynNcCLwGlIUJGhSnOaV96jcuYFwebMmdMV1SYBQRgADqrAlltuqcONYpCQnLAo4ul0fAYG4cTBo1P4Hnq9L7mMWSGgSo9nbIuCz9pKzIY+29QXfC5Re\/mYJeZ9X59QHnvNb3zjG2rs2LHqm9\/8Znd\/j7nEfJDWA5pjn3TSSSdZ950m88QA1WXbsG2lQkA1aWIuqFzzMfPaAOBTf4nHqJ80f6A5BMmDDz44RPWlQPSXXHKJwg+OcUjYwNIbAjX11wVU9FW0RzWtWThLAhgBvhBQIf6hlmEFh7hHLF86f8sFVLNPNuC7iGICdauttuoCkK9+NskoATT2LL7zap80to2D2sTq\/f3vf19nwfihXi1dulRtttlm6utf\/7oTqPj+3\/\/+tzb8QVOC5feTn\/xkt6kqQA1Jf76I9RKotNj42sfRCbRGgJIMmS6g8kD0fGsDgQbNpgpQiYZJQIU4\/+IXv6hXDlNSgeDYKzzyyCPDVN8ddthBW8QIqHxV4xJValHkjOvbo1YBKuLA2qyckn2zaTTi1sjYCwouFY4+RzQ9LJy0eGLfikXBBD5ofsQRR2iLOi08iC6JMdap+oY0HJoTm0ZCEs13ZOPb+\/okKiyvpOFx7cyU\/nizBtsyPk9VgCqZb3NRRHukpYmAaqq+2De6zlY5c9qMSVjBQWQYNriqiMNk3HyC5QzSlxPRnBTbMU1uoHImtp1b8pXUd2wABqV9uc2C7lukOG1NgwcWQBzP0P4Ix154LwjGDF4njgkASkhRmyGsLmNS6OzdHAvXXGxADdGf2wikQHVtSfixFOdDF1ApED32oBBiXPWF9gltxVwwbQt9FFB9+0NzxfUdR2B\/+qEPfUh8PANrMKyPsJC5jmd8Km0uoPK9j402kj0qynEA+Ojmu1AikVLUDoJLQ3IiEaPw\/psqsa9PvP\/SPartKIm3z4GMPtsknWS8\/KzRdR3VBABZfV28jttHABudnZJgMIHqwwp4FVIZthrEV4JdxpZ8x3tcAxNbfV2i27YPgckfD0rR8Qo6CKLjihpuwtDE46IBNucwU3\/84x\/XxzA4HLadiYUMLLmAihetTcMDP98iCbbPPvsMOdznk2JbYMw6Q9ZeXp9r78dptHr1an2JA8+R3HrrrUPO6ABSMCIWIV+9qRce+Fy4+IbnwdhhTcUlF6juLtWXX3gw6eU7GTCBijHzs3H0EXtz8C3oMn78eH0WarO\/cGOSC6iYByzgaAMghar9wAMPDHl5wjbfJt\/zerJ4z9C+iQOVBpUasNu3eo3k73LSWnq0kYv+Va8V1t2vnLQO9bXnQE0N2B0ayEj93sc8UlrbbBCgZ2ifmZvm\/QRUKa1TaZYFqKmdKeWaoUDIKyfFCl\/HSNoG1DrGlFpHAWoq5Uq5QoEeUqAxoD788MPdYcK4gUS\/ezj+0lShQF9QoDGg7rTTTl4CmeCl\/3E\/lUDN8xSQ9wW\/lU4mUqC1QE0cj\/YAQcJv+LvS3\/ifflLrLuUKBUIUwBXZHKkxoMJhGdcQ4TpEacWKFfpP+sz8XRcBIH3pBxKa\/ibPhrraaUM9ktfl2tDPQehDzhcKGwNqyqBsAKbPAHL+PV8AYpgAoAVgAWD8xk8Ms0vylmcX3TMioR+VluTtJa1TeFrKm30FVOmgKB+BFYYrE9DcmBWqd5NNNtERFDh4XWXaxjyS\/oTGH\/N91fZiykvyFqDGzJ4lb87VR9o1E8i4viYBMJe6hxxyiLS5RvJJmLmRjg1gozl5eqAlaiovEIAp0BfCafpUaQB39913V3vssUfrjpgKUFO5IL5cq4HKr06BKRDRbt68eVpVxAV7V8o5KLPNqsyK8nCqBlghcX3AhZr80Y9+1AvaXqpjVccey65V24spL8nbS1rn5OnKEhVuPPA8QHQ2eGogqgBu\/V922WU6LlIbgBrLbJL8AC0kLoxYPMQmL0uStkn1WMLMkvGWPGEKtBaoiBIAFx5IUfyGBMXZJZy\/ERAKDuauqOs5BxUmaf05IGnxg7AnZmpSNS5ArX+umxA+lSQqIg7COZmiCtIAEOkOsWIuv\/xyhcgNtjRoQOVjhJOwS9L2WsoWoBagKoq+RlEFiSRwj8I+1RfdHkC9\/fbbe0JFxNxFXNbUFFOe50VM3fvuu0\/\/4G+esJd93\/vep3\/wNyVXW9LPzXwxfU+lDy9Xtb2Y8pK8vjxSmrrmxiy\/yy676OB9OVIlicqjryE4WQxQEZkQVlIkqMmUEOofyfU7hQhVpUpMeVde7GmhFtv2s1zKuspLPzfzxfQ9hbZmmartxZSX5C3GJKV0kDKouebzexKJirAXKYkDmP4moON\/\/pNSf+4yZITq1V5Wwsy5xzxS6s+5naskUSn6GlTczTfffIhEhSHJVIn5hKUCNXbSOXAJ0FtssUUX0LH11Zkf0hWXLHxSturZbAFqnTPmr6u1QF2+fLl+LwbRwRFFkBKkLI5tEDpxzJgx1tHRoPAEHSX62\/yN7xEOk3\/Oy1WZCgIyIsYhAcT44SmG2SV5zTw+KYt+QDV+97vfrZ+hNB0HQqqupD9V6FdU3wldErQWqBTpDrdy8DQCjmcorAdFX3NdeqhrUCZ46X8ObESvi00AMAALKWwDb2x90vwS0JLTgOR8ttdAlY5zEPPVxdM22lRSfVFhalTBnIOyDZQDmoCLKPhIUiADvIgf1Cvg+s5maYw2b59UbWAQwdPLMeXk6cpATY2+lnNQKZMDIOMHoIU0llzOJ+BCwpFhK6VtSRmStKF7x6Qqk9QFkMm6Lmmn5EmnQE6ergzU1GHlHFRd+yYCL0AL8PokL4F2vfXWUwja7UtVjwwItGgDBj3zjNbWNoWq4QAmaZw6h7ZyVVXtmPKSvFVp7dNOzLpz8nQBqoBL+YQAvAAu1GYXcEOSVsJggm51swC45DAgddUz6zejXpBk5p\/H9KkXeavSUXo2TWMJGe6yAxVXAfFAE97k4BcX0EG8yoaHbnA2iuuAuKjwiU98Qj\/qitQP3jM5mQZgxc\/dd99tbYZAC4ttLxOB96677tJqeSqAeZ9tAedKsLn\/USgrUNesWdPB48EAonlxAQx4+OGHq0mTJumX15566in1+c9\/Xu2\/\/\/76ji8suv3gPZNr5TWBB2kLYAAUZuJSFtZyPOQcozpKV\/\/Qqs\/D1dA+nPor2ZfHLjY8OiRJZ9RBAOeSG3+vXbtWn8lLokpK5nVgVN\/tttuu89JLL1lfm8Y56P33368WL16s8P4mEh7RwWV7PMa7\/vrr94X3jGRCfQwYU57yArAuo9SGG26oJk+erM9FzSQFpFQdi+k71UnSmH7jcx6TKgegXfQ3gU7Axm9odVgAXaFl8fnAAPXee+\/t4F3Nn\/3sZ0MkKl5J\/uxnP6sQLRAubJTwDiiet8MLWSBS8Z7xyxja09pU49BeNlZ6xUjoOuoOBZtDG3UEnKva11ipziV\/TNtZVd9Op9Ox3dnFU4hQd\/G8+d57793tL70Hsu++++o9K64Ctt17JobYOfNi8YOa+fTTTw9rBlIWb8rinLbOJPEwqbM9SV3cSo0HiOl\/\/jn9je+RbHkkbVXNQ55N9BtPVfLP+Od4ejOr94wNqPRW43nnnTfkuIEDFURIvZSfc\/WRqpPSSYxRHyV5YaADA9qkLKnF5qUKqUoc2qNKx5yaTzJ+X93S8pDUzz77rN7PchXdjBONbR1eXSfpnhpGVkoP+GFL9tfS+iifPp4pQPWTTco8qEWSl+fBXtZ11MNvQnU6HasBKgRMSX9imaYOoLnqiOmvJK8rjwl0DmRsV+iVhVjV\/c4776yTnN26nEDFaoW96HHHHaeDdVEiiYq7veuuu65atGiRdhCP9Z7ppUTNQrmaK\/XtZdFU6vVFCTPXPJSBr84EL\/0PoytsPTmSE6gUDwmWyVmzZnXbhjEJZ6lQiZGqes9f1Wv1AAAF+klEQVTkGFS\/1xkCbQxwC1B7xw05hY8TqFC1cDyD\/dT555+vRo8erUcMNfl73\/uePrJBOuqoo3RM26a8ZyTTUJVZY8pL8sYcGUhAy4GL\/LgmSK56kv5IaCjNU7W9mPKSvDG0pjGm2gMaASo6DeskLjwgDOgBBxygQYuHnebPn69mzJihx9UP3jOSCfUxYkx5Sd5U5sFZduj6Ih8H1GVYKREvivxuTV9bKQCl+STj7wda2y6khOwBjQEVBOVXCLEPhQSdNm1aNwzooHjPSBmxTfkk945t\/TWjXvRDCJs20d3Vl+xAbYIIOQfVxHja0Ca56eG3z2kg1FczLlU\/xaQKjS3n9zl5unjPCGYuRp2T5E1VfVPUMWhE0IRwbxsApsgXUmf5EHlMUOPckkKzkqSmOsz\/bXVL6BfaS\/J6e0nrrEA98MADO7iQjwv2u+22mzrxxBPV+PHju2MdBO+ZmMkfNObxjZ2iXhCIMXYCMvnihoCa+j13tOcAhhHTFzKWl5PM68AA9aSTTurMnj1bX9HCmSieoLjgggv0JfxB8Z5JZaZBKCdhZt84JTGpcoPa1T8XoENxonNF48gqUV988cUOHb08\/vjjCgGzzj77bLXzzjvr45lB8J4ZBMCljqEqUGPbtUWS5BEjedA51N0UyNG2S6rzPTnlMxcFG12yAhVXCKlRfo8Xt5EGxXumKrPGlJfk7aU6JulPLBh9+au0B9DS\/V1J6FhEvMRzmL0GO4EWbnZoH\/8D3Ai8gOPKHGmIMYkkKvapiPuT23smx4BKnSObAnBqoER\/m7832GADnYV\/zstVoSCeIM2RukDFfcVTTz1V70vxtik87XHXN5f3TI7BlDoLBapSgKvpJKldqrspySFZ8U5wjjRq7dq1nRtvvFE7gsOAhPhI8InM7eaWYzClzkKBJijA1fRcN79GzZkzpwNXq\/32209HcqD3THN7zzRB0NJmoUC\/UmDU9OnTO6effroOucKfn8jtPdOvBCv9LhRoggKjVqxY0Rk7duywtnN7zzQx2NJmoUC\/UmDUqlWrusczNAiYnPGT03umXwlW+l0o0AQFRk2YMGEYUHHhAYGakHJ5zzQx2NJmoUC\/UqCxS\/n9SrDS70KBJihQgNoE1UubhQKRFGgtUHFL6rbbbtMxmUrKQwFcwbvooosUnnJE1L2DDz5Yh9SBY0ZJ9VJg5cqV6owzzlBPPPGEpi8cYXgAhlBrrQMqLm3fcMMN+s4kAqudcMIJoTGU7xMpcMstt6hHH31U36ZZs2aNwvW3mTNn6uc2SqqXAnBwgZ\/uXnvtpe8mn3nmmVoIud4gMlvPBtQrr7xSYRUxgRZ6\/Q3nt4899phmILjeFaCGGSaV1ngUDOfnU6dO1Y3g6igul5MhMdzyyMuRSmvc9MPDWGPGjNEBw3ETELwtDdadBajPPfecOvbYY9WUKVOGAU36+huc2RFJvgDVD4Y6aI0WcMcbzhgLFiyo\/VmNQYFzVVpjqwEa46E1\/EYYXuk2o1agYk+J0KI4f4XknDt37hCg0W0nrOC4roibUNgbLVy4UK\/kACV1vADVz9510Rr0X7ZsmQ6iDvrvsMMOQ26oDQrIqoyjLlpTH4CDk08+WT9fGnqdnsrUClTcD0ZQrVdeeUUtWbJEx5flEhEPJktffytA9bNWHbRGC2CYbbfdVgcMoAACVZh6EMtWpTUMdrC7TJ8+vbsnveaaa9Q666wj3mbUClSaJBLx48aNGwLUO+64Q\/z6WwGqjOWr0PonP\/mJwlVRCp4ua3Hk5kqlNYTW0qVL9VZw1113VQgAd9ZZZ2k30re\/\/e0igvYUqLbHqNBLgHLevHla\/Zo4caLueAGqaP6Ui3lCtMZD1DCMIKA3DByU8MzmRz7yEVnjIyxXKq3B1\/BVBThhVEJCfGzQmTvC+MjZWqCOMB5IHm4V5qFFMbnxEVawSVr3FKg333xz8utvI4wnxMN1MU+htZiE4oxN0rqnQF2+fHny629iao6wjC7mKbSunxGapHVPgbp69erk19\/qJ\/tg1OhinkLr+ue3SVr\/F1sqQpRpJuwmAAAAAElFTkSuQmCC","height":141,"width":234}}
%---
%[output:17a0e5ed]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAOoAAACNCAYAAAC5ZeasAAAAAXNSR0IArs4c6QAAH4tJREFUeF7tnQm8VdMXxzcVCUmFSr1KhVSkIpEyJCVFKclQEkqToZSh\/kKmVCIVGUMiEUU0kCgNpEHzQKEyNJmj8P98V\/Zz3nXPvee+O5xz713783mf9969Z9hn7f07a+211\/qtff7++++\/jTaVgEog0BLYR4Ea6PHRzqkERAIKVJ0IKoE0kIACNQ0GSbuoElCg6hxQCaSBBBSoaTBI2kWVgAJV54BKIA0koEBNg0HSLqoEFKg6B1QCaSABBWoaDJJ2USWgQNU5oBJIAwkoUNNgkLSLKgEFqs4BlUAaSECBGvBBWrt2renWrZs54IADzL777mu2b99ujj\/+ePO\/\/\/3PHHbYYZ56\/\/LLL5vy5cubU045Jerx8+bNM507dzYVKlTIvd8ll1xirrzySvl\/5MiR5sILLzQVK1aMei09IHESUKAmTpZJuRJAHTNmjLn99tsFrH\/99Zd55JFHBHgtW7b0dM9YgTpu3Dhz\/\/33y\/1+\/\/138\/TTT5s\/\/vhDXhgFCxb0dE+3g0jW4gfQa\/MuAQWqd1n5cmQoUAHOAw88YBo2bGhOO+0089xzz5mXXnpJ+obma9++vfwNuF988UXRupUqVTLNmzc3devWNVOmTDFPPvmk2bNnjznvvPNEUwJI29CoTqDy+ZYtW8wdd9xhbrvtNjm3Q4cOpnTp0vLC+PDDDwXEl112mdx7165d5qGHHjLvv\/++aH6u3bFjR\/Ppp5+aFStWmA0bNpgbbrhB7n\/vvffKi4A+cv1ixYrJPapXr27efvttc9RRR5nGjRubZ5991vzwww\/mnnvukWfIxqZADfioA1Qm+ubNm3N72qxZM3PfffeZhQsXmlmzZsnkpj388MOmWrVqAg4+79u3r9m9e7e59dZbBUj777+\/fN69e3dToEABAXnJkiUN14sE1F9++UVA0q5dO4N2Bqjbtm0T0LVt29b89NNPooF79Ohhpk+fLve5+OKLzbfffmv69+9v+vTpI0CdPXu29HufffYxjz32mLn00ksF8NOmTTNff\/21mNTXXXed6dq1q2nQoIEcW6RIEdOzZ0+zaNEi884778gzxavVAz7kYbunQA34qIUzfdF4gKRw4cKiYerXry9PgTYEiDS0rf0cjcuaE402cODAPE\/MepTJHytQ0dLr16+XnyVLlpgPPvjADBo0yIwfP15eCkcffbSYuI8++qhp0qSJAJUGsGmAe\/ny5WbNmjWilStXrmyuueYac+edd4p2LV68uFgO9jlC5RDwYUt49xSoCRdpYi8YboLaz9CcZ599dq6TCI01f\/58MSvDAfWTTz4RM9OCBZP1zz\/\/9GT6YqYCaDQhGvWzzz6TH7TsoYceKtqc61qNW6VKFekHn7do0SIPUL\/\/\/ntz1113ieldp04ds3r1ajNnzpywQMXExwmmQFUqlsQiK8FXC52gDBdaa926daZWrVqiJa+\/\/nq5K2BiYv\/2229mwYIFYhLzd69evcxVV10lwJk4caJorUKFCom2c2pkq5VDnUkchwnKehYTGKC+9tpruS8D+oCzC406depUeRlg+m7atEn6gPnr1Kg80zPPPCPnYMYOGzZM+h9OoypQ904o1agJBlaiLxe6PYOz5thjjxUAoMnCOZPQkk888YSAEhOVdWDTpk1FewFynDM063xyrvkibc9wjgUqTiDWnqw3WVuyhj7kkEPE7MW8XrlypZi\/Bx10kOnUqVMeoHLu8OHDzeTJk80xxxwjpjH9QmPTN6fpq0BVoCYaU3o9Y8yMGTNMTk6OgHTr1q1myJAhotFxWmnLvwQCp1Ex1ebOnStvWLyGtCOOOEJMqXr16uVZT+X\/sfXMZEng888\/F9OadSjeXzT\/SSedlKzbZc11AwNU1l6LFy8WZ8WZZ54pDoSiRYvKQABe1jhvvPGGmFZ8r00lkE0SCAxQf\/75Z3H116hRwzVqhagcPIRVq1bNpjHSZ1UJBMuZBBA3btwoESp4Gd977z2zatUqcZ6cddZZYkppUwlkowQCo1HZOhgxYoSZNGmSRM0ATsBKCBlr1hIlSkiwOF5GbSqBbJNAYIC6Y8cOM3jwYNO7d2+JF8Uhcffdd8ueHGbx0KFDJUSNLQltKoFsk0BggEr6Fnt\/xHXiPHKGkvE\/AeBsiBNapk0lkG0SUKBm24jr86alBAIFVLI8iKRhvUqa1BlnnCH7pmSA4BEmm0I1alrOM+10nBIIDFAJK1u6dKnkJ4ZreHzJb1TPb5wjrqenpQQCA1S2ZkgOdssRwNtLLKkyA6TlPNNOxymBwAAVry88QCQQ\/\/jjjxInSg4ljSTl1q1bm6uvvlqCvLWpBLJNAoEBqhU8pu+oUaNMmzZtzJFHHikfA94JEyZI9r+avtk2RfV5kUDggMo2DfG+5FgeeOCBMkpQgZCA3KVLF3Um6bzNSgkEDqiwDpAADR0lDAA0G0p4yy23qEbNymmqDx04oFoNCs\/Pm2++KV5gwghhKLDZNDpsKoFsk0BggAogIbyKlmCMkynaMdk2iPq8mS+BwACVIAcC8slJJUmcwAfLN0sIIbw85KPWrFnTtGrVKvNHRp9QJeCQQGCAavsEMwBMdvD9fPHFF\/IxmTTw8rBFo5FJOn+zUQKBA2o2DoI+s0ogmgQUqNEkpN+rBAIgAQVqAAZBu6ASiCYBBWo0Cen3KoEASCCQQIVysl+\/fpJATpAD3mAqgHmtB+qUK0H+7McS2UT4IcnnlFjQ4P4AzD7tgmcJBA6otqxgo0aNxPtLOQSKCZGfmp\/IJBjbYY6AMYIMnAEDBpgrrrjCnHDCCZ6FpAeqBPyWQOCAailZLr\/8cqllQnkD9lPzS8UC2NGqlG+gOWuJ+i18vb9KwKsEAgdUNCpFidCoVLoGqN98843UJCENLtY0N0r3wdQO3SiNimdUDnOWGvQqLD1OJeCXBAIHVATBGpVKX8uWLZMAB9aWgJd6JrE2Z43NcEClqnU05v2ZM2fKbZ3HhfvMrW+ZcH64Zw+VSbjnj\/TsQTqflEqqz9HwX8B2CZkBVevcxj7c58zdZLRAApUHxVwl9pfKZPEwO4SauvwP8Fu2bCnyBKjJEm4yBixTr0kUGhlTiWrjPv7GtDuplOfLYa1Rs5VGWuXrr78uaZZExHltyZxLgQGqk+EhnGDKli0rgoyV1xdnEqUJ0dA0yga2b98+tyxGvMJN9ATzOiliOS7b+tht3EozZ\/1O8+X2XSaneGGzuF+9qOLyAtRocox3LkXqZGCAajmT0KBsx2DyUjWbhse3cOHC5vzzz4+ZKd9uz5CMzt\/dunWTPFfLuJ9M4UadHXpArgSigcCLqABmi5GL5FDAyf\/dx600X+7YFRWsXoAarQ\/JnEuBAaoVApp19OjRpnv37nkYHlijXnvttRE16pIlSyTpnFQ41hxsxWBORdpHTaZwow2sfv+vBOIF6gNTN5gHpn4RVoO2GLEXvJO6negqcgVqjLOR8hWYuBCZWecRnElszxAE4ZY8znoW8xaNSRXrWbNmmWnTpsm2DGXo3fZRFagxDlCSDs8PUDFxv9q+y8xevzOqiVtz4FyTc2hhV7AqUPMxsB9\/\/LFsyxQqVCh3cc\/asm7duq5X+\/LLL80rr7xiunbtKvuuePCoXXPyySdLsSm3fdR4gRrLBMPBMXvdDsNvbfFJgLUn7bRKxcyIdt7KcBa\/aaapX6lYWLB6AWq0sY53LqXFGjW0kySSY8KydoXRYb\/99vM8spw7duxYYddnbUtRZLd9VIT77rvver526IFoexxdkdro+TvN4wt2mjJFC5raRxY2AxqVzPf98nOilz7m57qJPCcVfdz84x7TfMzXMgajW+X1CGN1dezYUR6J0FX22nnhWz8Jn0fr49lnn520HYRArlEtv69zIoR6fYlg6tSpk2FdSjlGG8CAZh04cKDEBVMZjvUuwq5fv75cLjTgIZlvwVicGYmc9Ol4rWjaKlHPxJhgBm8fmrdqvReNGq0PyZxLvgOVNSnMgzS0JmaqkzEfMm5YCAsWLGguu+wyqZ0arlnvLpoU0BLLy8Z1svdR3SZYqAcy2iBn+\/epAipytmDFM8wyZM66Hbnr3HYnlTbd6x+m+6gICnMW0wJPLuatdRBRF5XgBhxCaEGb4WJNWYIU3JxJhBnihMKhZIm7uZcf+6jWA9n33Iqm77l72f79bKkEQX6fM9V9nL1up+n+0koBbWhjHdu22Ir\/BDxE62NGaVTs\/6eeekrWdcTzhsbu8v3UqVPNokWLxFNLwzE0ZswY2bJxAyoOKLZvcnJyRENv3rxZ1qZs1wwfPtyMGzdOgN+2bVvJyLGaOdHCtSANNa3yO4Gz5bxoIEi0HNCkeI3d2uUVdpqzcvbJ3sgktCk\/mLLhmnPtab+H3Cw0UMFNwDbCCZK0kSNHmm+\/\/TZimlsigQpIx328JermeqInXSZcL9VAZW+VbR23Vny\/P83gU37OXqBawbiFDOKlbdiwoWnXrp2YwTTWn8Rfsn6NlPCNiUwEUvny5c2MGTNki2f69OkR09ziBaqdYDZszUu4WqqBlWoQ5Of5Ut1HtmoitXBAjdbHeOdSpP745kwiVBDHDwHybJ0ARvZBCVxYvXq1Ofzww3M9uTiXSP7GixuJLhSPLudecMEFsocKUDkvUppbvMKds2St6TZ5q8g4iCClX9EmWH6AlehzUt1HPL\/h1qf2uRSo\/0giXKjgBx98YIYOHSoVxzGN0ay2VatWTQBtTebQ7ZkOHTrI2rdnz55SoZz1rQVqtO2Z\/O6jLty0y1z72jeyPzq5Q+S91ERP7Ey7XrQ9ykQ\/7+SVP5sBM\/a+YMO1c0puN81zfsuzjxqtDxm5j8q2zIMPPigcRjZgAKcRQG3WrJmEAFpnUjQB8f28efPMpZdemufQMmXKiPOIPVVnZFIi0tyC5tn1IqMgH5NqjYosIq1TdY3qmC3OUEEcTHhq0YIwO6BxnWZuLGluaFurUXEmJTLNLTT4248JFivgtI\/uErMOQHuE3UftPPJd07rKX3mcSdHkGO8yKpBrVDrFunTjxo2yhsIBhKZjD9SavrFOSHu8E6jkr5I945bmdnj7x1xv81eREvLdX0X+Dfnb99etpui0vvntmp6XBhLAoVmvXj3Dvv7cuXNj6nGySAh8cybx9NBcTJ48WVgc+vTpY4YNG2auvPJKc9xxx5m3337bfPfdd7lCwgNM8ahYOZOiSTnSXlpO8QPk9CAELUR7Dv0+syXgG1AJDXzooYeEuhPOXUxeAhX4DM9vnTp18nAkoW2PP\/54LWSc2fNRn85FAr4B1Xp9id\/FqQRQifVlW4WsBeJ1bdlFHT2VQLZLwDegsj7FaUTI4NKlS811110n6WY2MP\/UU08VE1ibSkAlYIxvQEX4cPjCwsBalYU72zLsRaFd4UkqV65cbmxvLF5fHViVQKZJIOVAtSRmaNRwDdKxeOhBM22A9HlUAkgg5UB1xvjiUCLNrUKFvalg27ZtkwJOBx98sGhYZ4OWhQijGjVquAb065CqBDJVAikHqhUkZu+oUaNMmzZtcvNHCSOjVgygJFChcePGsnXDPij7q2hh9rhId9OmEsgmCfgGVIISCEK4\/vrr89CC4gGmEYBv90zRvCSZ23heaDO8Ni276FVSelyQJeAbUNkzJakb3l0IsWlQruABJpXtpptuyuXwxVwePHiw6dKli1B\/xgJULbsY5OmnffMqAd+ASgfJMYXTCNMWUxhT96qrrpJcUhgZYGgAtIsXLxaWfFgZiAeOxfSNVnaR+MxUtJ\/r9zF7Sh4jt9IwxMRJ\/I+c00SuhHnakM94QzzzG0KYrPBBX5xJAJLIIyhAIzXySnEu4SVmfcrx\/E3hHjd2iHDXi1Z2sVTrgeaiiy4K2xUbQhjuy1jDCklUhn+WwkU2sN95XctTC0k0rdw\/vLWJm9LBvhLZVG7hoZBs20Z5CmceKXJDZsjLjheM+fFQ4WiRKGMk4J7aMmhJYncrVaqUG4FE8MOKFSvM888\/LxrWMuXz97p16ySCKVKRKLJxMKcR9BlnnCG5qSNGjBCtDA8TYYgNGjQQpxTVy2kE5UeqPr7lpz0RZzhcseEaOarNjz3IXFu3mHxde\/gG4ZKFUzZcg\/uXtvmf+7ldNz9wQ348e5BbpD4iS9vKHFwwV6Zuz2P5e5tXPShfHMrK6+uQLJxGmKUTJ06U7Bka3EgEPfA\/gPrkk09MrVq1zFdffWVq1qxpTj\/9dNe5Bgsh2pPQwyOOOMKMHz9ewI5W\/vDDD4UcjT1aHFJcGycWLVmpSWhNJw0lWgAGCKs5gwwaP\/oWLYUs1j7BMkjBqEldTzT1K+99WXptqlE9SApvMPQpaEP4jgASpi+fUa7CzTxCm3I8mhINunbtWgEnwRMAHvYHGqTdrH1vvPHGpALVw6PqIQ4JJBqo9tIsOWI1gxWoHqamk\/lhw4YN5tdffxVgDRkyRLSgG2cSe7BoVHh94VvCGUUxWihccA6gVTF5CVGEqsWavsnSqB4eVQ9JAVBJY4Qa1GudVLqkQPU4NdGObNWQWXPbbbdJDRmoVAjcd+NMIp8VziW2cQA3EU5s6xA8AfdSpJIW+eVM4nFSzfXjUYR5DtM+GuOs\/+P0HYSTp65R8zHLcD4BPMIKbfFhLsPnBEGwpq1SpYr8Dwshua2sTUuUKCGsEax7cVAtWLBAHCqEIFLhrVWrVgkxfZNlsuVDVK6naB\/\/FY2lXsFn4FbZzU2jRpNjMq0zX\/dR0XikumHiQkKGVxfzNFx9GWf2DERmmLYER1AECqCuWbNG9mABL+CkcBTr03PPPVe0LMcDdPiCWf9eeOGFCQFqIgGVzdeKBoJkyMaSm4WuYdX0dUgbDci6k+0ZGAdhwieQgYglgARnjTNx3JlVwx4rjIV4dnv16iVAhfsXoO7cuVOATjFj1qfFihWTim+AmpfAiSeeaKpXry6ApcX7FvRjgiVj0vp9Tb\/kGK4EiQLVMRtsrG\/79u1zGR5YT7IPSnAD+55Qr7Ru3Vq2aELro7KOoLgx2y0AlUZEU48ePcT0ZW8U7frCCy9EJOD2Eo5o62YSvmhbuM\/CTXZ7HN\/5cT73Peecc4SSNT\/35\/x4nj8dzl+1p5SZsauqaXnAInNkgf+WuYjl+SlUlozmm+mLd5faMADRciYBPv7u16+frEkJFyQ4gq0VgI0mfPbZZ2VflRqoaFZAism8fPlyAakteYH25BpNmjQRXl+3gId4NWoyBiUbr+mXRrWytt5h9rpL7P+nll10TkKcP5i\/mKu1a9c2eHvRknhoCRfEvCUOGKZCtmgI1AeAbMFcffXVYvpC1wJIAST7qGhIPMMAmmCKunXryj5qqgMeshFs8Tyz30Cl76xZCUUc1CJHgRo6mGhWghMwV9Fu\/AacgCyc6UtFcWrUYI7Aqg9AMW\/JrCFtbsCAAZJ0DgE3gRLsu86fPz9pAQ9BmGDRAKJ9jCahf78nQGJchypm67JZ2V0f1SkyG00EZSj7oNREBVg333yzbKG4OZO4BmYyjIXE\/hLkj5bFGwzdKCYzoEdbEzdMixTwoPuo3idyso4Myl4v9WiIFb646DKZf7CK2Batjxlbe2bQoEFSfBgBAFJ+CNYvVapUni0agEdsMF5cTFoAybmsWwE2TBGYwBxDxNHChQvlWICM9o0UlK9r1GRBL7brBknro1V7Hr3N1CxVUOujOmN6MV0t4TbOJMxYt1BBTF8bp4sWhbKFgAbCCwEv2hWPb+nSpQXQrFfnzJmja9TYcJPyo4MEVNaqu3duNp2r7Vagsl9KbC5RR2S+3HfffeJM+vTTT0UruqVlsZ5FU7KNwxrUZsmwBcH1ACZlMQh64NoEVFA\/JFlB+UGaYG7o0j7G9t5hH7Vc\/wXm6QY\/aJEoRIdAPvvsM0lLYwtl5syZEpdbtGhRV8kCZsxlQgTJqCfgHo8wARM4kwAyziSKFwN4L2vU2IZRj850CTCnvmkywlTb8Z7ZNGtsTI+bLJYH3\/ZReXqikzBVLccvDiKoWYgwciaIoykff\/xxMWnRjNYs5jyIut966y1582EK2\/Uq0U4TJkyQlwBJ5G5B+TGNgh6sEvBJAr4BFZCiAQEZLIOE+mGqsu1CQWI3uhUcTmy3sI6l4UAiGgRzly0dwIq3zpZeZIsH+ha3QsY+yV1vqxKISQK+AZWA\/OHDh0sQAyltVatWlZQ0ghYAqxtQYRUE4IATExmNSmYMKXBEK5EYTpwvJvIbb7whwf4vvviiAJhGQAVhi9xPW7AlwEsYf8SWLVvEgUg9omxtgQAqZiprTkzbRx55xFxzzTWuXl\/M3SlTpggXEn\/jNCJnlTUu6wMCHRhggv1Zo5YpUyZiIeNsHfh0eG4spJycHHnxEqfMCzzR9XHTQQ700TegAjLWnTiUGjVqZJ588knZC121apVovWwdkHSZOKnoJ\/4IOJ\/ZanP+nYp7B+0evgEVQRDPy7q0SJEiEqSAWdu0aVPRjtqyWwKUMmFrDSpXnIdkQpFbbDOlsk06vgIVhxIalKB827SyeOZPQV7INiWRp2UekDSBL4GXdv\/+\/WV7DXO3efPm8uIGtPbvzJfQf5\/QN6ASfI9DCI+vk60eVgb4ftX0zbzpSHI\/ASrUw2XLDGpXGj4KfvA1UN2PpQ\/f4RBkjYoWHTt2rHj2s7UKvW9ADVckKvOmpj6RUwK8lAn3JC0RQFqgsk9O8Dt73fguiFKDexkCdry+RK6p19etonCS51i4sotJvqVePiASIAYbDQpQCXLBsoJxEmIAGkEvUO+wtaZtrwRSrlHJP8X8wQzirco+6JlnninbMzQ1fTN\/aoYClf1vKHVsCRMFagDWqHh6AambIneSmGX+lM3OJ3QC1WnqWtMXU5jcThxK2nzSqNyWNQd8SQTkMzihsb06OJktASdQrTMJuhxSFbGybA0h8pK1+QRU1qY4CMg\/hcUB2hUGB4aHWMop6gCmrwRCgWq3ZygaRlw23l\/VpnnHN+Vr1FBvL\/+T8M3gREpvS99pqT1XCcQvAV+Aaqu1ObNc7rjjDtf43vgfU6+gEkhvCShQ03v8tPdZIgFfgEoqGmUmwrXQ5PAsGQd9TJVARAmkHKg6HioBlUDsElCgxi4zPUMlkHIJKFBTLnK9oUogdgkoUGOXmZ6hEki5BBSoKRe53lAlELsEFKixy0zPUAmkXAIK1JSLPO8Niczq3LmzUNI4izVTPpLq6OncSGWj7s\/pp58e82NApE5NW6rQ0wg7pEYR9XSdDV5osm2oIJ\/JZAMK1JinUGJPsPzDkSKzyDgiq4ifdGkwQT733HOma9euuawMZMrwY4tNR3oWJ1A5B1oWAB+OM4kXAumTzZo1SxfxxNxPBWrMIkvsCW5AJaGaPE2C1OG1hQaVGjqwNRLEDjsf1CRwIVu+obJly5oKFSoIQdzGjRuloyRfcy3oTcj5rFixogAIKk6SICh52aJFC\/Pqq68K+wL3Wr9+vVQegCuZ8iBQuJI3zAsDClZ4lGFkoPwIAKEmLUEsTq5kik8DSMBDsD3X3rBhg7nhhhuk\/9QOIkEDPiReUtDxQL3C3zQKUENqRhV5mCFGjx4tNLIQtlPRjwapOhzNcG5RbpNE9EyNF1egJhZ3MV8NoIZGakHihekL8wGk03BIUY4SzYEpWKBAAQFbyZIlxdyzKWIAslevXpI26AZUNBXmIteE6Y+ECIC9Zs0aQwV4aFAAJPemng91ZQEWLwVAzPH0D7CS8cT1eFEAMMtnRAEwrgODIKwNANVeG6sAAnWqIUADOm3aNDFpefFwT5IzbPEvgA34qI5ASiQalRcVx0CCR8w4haw53nm\/mAchDU5QoPo8SJE0qtWCmHuw8KFhnY21LVoR7WNr67BeQ6uGAyp0J3xvC2fZazHJbbP0J9BzwrwBMyDrP0uTghmKBuMcqhwANOoEAV7bnBqcvgNUq935zYsC3iReDrwIKleuLNqZigm33nqrmPh8xxoVjQqxGaGlPBcysWYuz21rFJHDyjUoGJaJTYHq86h6BSrAwUy0QEJroRExSy0xGI8CoCEyDwdUAAdIMWvthAZUaGhMYSeYuB8TH2BbmhRACsisNitfvrw4eagDVK5cOU9AtbWC0KDkJK9evVrq17oBFXZ8qiJghmPW0gdq4mImYwL369dPnG4KVJ8ncqbf3itQMR0BE2tV1qUACY0CrQ3aCe3mNH23bdsmExozeNOmTaZ3795iWlJ\/FhBzPJ7m+++\/X0pF8HkoUCG8phSm5bFCy8FYj1mOWYp2A2wwMziT\/jGVBw8ebM4\/\/\/xc09deG1MZxxC1gDhn2LBhcl+0Pf3DhKYMCaY9TCCsn1mb8xzLli0zkyZNEpOblwsvJdbcMBY675eJc0Y1qs+j6hWoTH5I4UIdKbt37xZaG+rxOJ1JOGfQNoAVblyOY\/3KMRwPATYmJutQtBvXDgdU1sFoq6VLl4pji2tihmJ+AnYAFM7cxJlEwyR2mr44kCgOxvfUlGGNyb1xLm3evFkcU7wYYHjgfvSX\/teuXVvWygAYLQ9QWbP27NlTng3A46hyluv0eWgTensFakLF6f\/FAAUmabLXamg7wIGzJxw4MHEtQUCy9zdxsvHCa9mypf8DkKQeKFCTJFi\/LpsKoLI+xHTFBKXAl1v76KOP5KtklktkzfzCCy\/INlOyXwh+jSn3VaD6KX29t0rAowQUqB4FpYepBPyUgALVT+nrvVUCHiWgQPUoKD1MJeCnBBSofkpf760S8CgBBapHQelhKgE\/JaBA9VP6em+VgEcJKFA9CkoPUwn4KQEFqp\/S13urBDxKQIHqUVB6mErATwn8H\/91PpUH\/xH6AAAAAElFTkSuQmCC","height":141,"width":234}}
%---
%[output:0fbb3f45]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAOoAAACNCAYAAAC5ZeasAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQuUFcXRxxtUQBB5qghCMPgAQwwKKghRISpCBIGAghgUjLxRPxF8hIRoAEEiiQgE9KARE0FRIyDmgFExQkBAiEZFRXxAAiZRJAaUGMXv\/Frq2js7c+\/M7MzsLlt9zp7dvbenp\/vf\/e+qrq6urvTll19+aTQpAopAmUagkhK1TPePVk4RsAgUIerOnTvNlVdeaV566aVQ8Lz99tuh8mkmRUARKBkCxYh6yy23mPHjx5u6devmLRlSF8pTsqrp04qAIiAIFCHqf\/\/7X\/P666+b5s2bm6pVq+ZFibyF8ijMioAikAwCxSTqpEmTzJlnnmk6duxo6tSpk8xbtBRFQBEoEQLFJOpzzz1nHnnkEbN69WrTsmVLM3DgQEvcww47rEQv0ocVAUUgPgKBVt\/du3ebjRs3mkWLFpmnnnrKtG\/f3vTr18+0adPGHHroofHfWApPrlmzxlx66aXmwQcfNG3btk20Bt6yp0yZYvhs7ty5ia3hP\/30U3PjjTfaek+ePNn87W9\/sxPoqFGjzCWXXJJoe7yFhcXuoYceMnfddZe57777zPHHH5+3TlHyRmmc4LRkyRIzZMgQc8MNN4R63ItvaY3vzZs3m7Fjx5rbb7+9GIahtmc+++wzawm+5557zL\/+9a9EB2EoJMtRpgOJqLIL0Ldv34ITQhTyRckbpesZ6Exg3bp1C01Syi8rRKUuYLNgwYJiHMtL1H379pm\/\/\/3vZvny5VYdJvXv39\/06dOnmCFJZl4B9rbbbrOdK53dpEkTs3XrVkv473znO0Uq4j7rfieff\/e73zXPP\/+8LTrfTEkjb7rpplzfSl5XKpxwwgl2C0ok6\/bt24tIKf4nST3efPNNK42lPV4J4\/7\/3nvv5d7fsGHDYtLF+6wMLJGMQRi6A4kZd+TIkbktNGkjE8ScOXOKtZ0P5DvqdMQRR9g8IvFdKcTn0k7+pj7XX399rh3e7TsvRvJyNJd69epZ0giegseHH35o8XTz0hdu30E0NAc\/ySaYSbnSfm\/d\/MaJ91mpP+9BY2F8Iogo2x2HXoxcnGTSufnmmw32nV\/84hd2bOXD3FtXV9PzjgnBqRhRISeZ\/\/CHPxhUCFLv3r1N9+7dDWBXqlSp2CQnA0w6WUDn\/\/POO88SAwBQi0jurOc+yzsEMAaSkMQdjNTJT70KqgMgkET1FaLKXjGDQgY\/k4mrWvLd2WefHZqo0kFBqq9XQrmSRQawH4aCC+3wU33zqd+CIThI24Wo7gB18ZY6uNqB5JU6MHhlwkPFdNtyzDHH5FXVvRJVxotbR+kLl6xeiSnt9pKVfvCqva7A8Pbx1Vdfbev74osvFhmjMoHma9vixYuLCIeg8SaY8xvcpH3yvJBV6ultQxGifvTRR2bYsGFmy5YtVmpedNFFVleuXLlyMXK6H3jVPQG0devWxUgA8G5+1GmXfG6nuQSTWTdoHZRPnfKTqK5UCZrFRKqElaiFiOpKRvaq2bOWgT99+vQia1sXw5\/+9Kfm1ltvzeUNWqO6M7VIBPB1Jw4XeyYHd9J0B7N38hJi+mlEVCwIf1dSupOQ9KOQGmkmUp46+k3I3ndIfb0k8CNqvj72qr7eSUjGuiuRReoL0VxNxMsH93+vhuadfLwTikxUxay+O3bsMIB38MEH5yWnfCmNdIF2AZw6daoZM2ZMbgbxI6qrsrkqUZpEdWfsrIjqDmhUJDqQ9Z9Xk8CRxMVwxowZ1sAQJFG9Kh35IKo85\/aNuwYSoooaKdj7aRn0m3dpQX6RBF4SeVVx8uYjqmhvUge\/pYOXwDL2RBKibrtS3h3AJSGqd0lCuV6iCg5+fHAxF6J6yVVIKygmUelc1kCF9lCRvpInikSlgq56653x3QZ4Vbp8UtOVxMyo7rMu4UX98yOqGCFc9aNXr15FLKz51qiFJCr1cGdQV2rkw7CQRPUOYK\/GUkii+lmPg2Z2tw3uWo6dAZGS5HEltZck+VTJfBbXJCSqXx+L6isToStR5TuZ7LxLAK\/qShlhJGrQDoRgJWtd4UMxh4drr73WeiYVMlHTGBbQfuphnDWqVNxPTQiatf1mTOmIoHL8iOqdBd11nRhF3HLRAKROUbdnXMOEazTJt84vtEZ1B7BIFVEHIRAGtnxr1CBJ7WJIHldaedVFBqwQ1SvZ3PGAgdFvjepVhaVOSa1R8\/Ux2oe7\/eVVff2WCyzrWOv6EdVvzS394cXRb\/yIgdNtezHV9+WXXza4B4ZJHTp0yGUrZPUlo59FzSU6f7sqTxSJyrOuahZUjh9RvVLCVdO8Fj+xQAcRVersp7oJWN6BK5+HsfoyOEQrkf3Cq666KneYgkGHtsB37uTH5BLG6utaS71WX2\/9XMuoq37zXtcCzhp\/xYoVRga3rLEZkO4kLBZ7766A34TstfqSJ8gQI897lwiiihdao7rPYVyU8YJRc8OGDbmJ0N2jj2L1lXqUeB81DGmD8uRToUpSrj4bD4Eo+7wygPFMS9uxIl5rysdTUTAPalEoh4eSwKFELQl6JX\/Wld5eFbrQ8sbVdtLw6ip568pmCSXF3K9VqRO1bEKptVIEyhcCStTy1V9a2wqKQF6iYvxZu3at+cc\/\/mG6dOli91bFBS0tvL75zW+mVbSWqwikhsBnTdqbT04dZMtfPPwU0+G42om+K5CoK1euND\/72c\/M6aefbk9riFf\/gAEDzLnnnptoJdzCIKqGeEkHXsU2HVwpdf66982I+ZuyJSpH3CApZn8kqIRnwZMFMz\/fpXU+VQdTeoPpnXfeMccee2x6L6jAJZcKUbHU4ouKR8xBBx2UIyr9EDamUtw+U6LGRa7wc0rUwhjFzVEqRP3888+tb2mjRo2smsvf48aNM08\/\/bSNqYRHUpUqVeK2Ke9zStRUYLWFKlHTw3bKsnfNlGXvZKv68raPP\/7YuoTNmzfP\/O9\/\/7OGpM6dO1spm6ZBSYma3mBSoqaHbakRVZpEdAfWrGyOh9kgDwMFEnvatGnmG9\/4RjGPFyVqGATj5VGixsMtzFMuUf8yrp1pUrdamMdC5\/G1+nJ4\/N\/\/\/rcJuu2Cw+O1atUqeE41qBZLly41v\/zlL62xyuuapkQN3XeRMypRI0MW+gEsvqxTSZkRlSNsnJAn\/EnTpk1NtWrVrCq8bds206xZs1zliQIRNXEyBRcrzrxWr17dl6ishTUljwDbbOCuKXkEzhi\/zHxe\/0QrSSFq0slXoqKa3nHHHTb2i5wWQLpyAuLVV181Q4cONatWrcp9F7ZSqNBEMuA0xQsvvGAfU4kaFr2S51OJWnIMg0qoe92z9qtMicr2jN82jPs5lYpypQVEf\/jhh60hqlOnTlaqKlHTGzh+JStR08F75Vu7TPdZG23hN3Q+1tzQuWniL\/KVqJxHJUYP502x9BIzSSTqY489ZkNqvPLKK5Fi5PpFcqM13mhxukZNvI9zBSpR08G2+8yNZuWWXbbwNNwHKTfQhfD99983EyZMyK1TkaZYfQnVgjGJE\/GPPvpo7JarRI0NXewHlaixoQt80LX2Vv7kA\/PB7D7JvyQfUeVtrCvZoimppddbeyVqKv2Zt1AlanKYb92511p5xcmBtenOxbearWujG1jD1Crv6RnUVX4ksWWDAwQBnwoFPwvzcr88qvrGRa7wc0rUwhiFyQFJR87flFN3IWm\/0442c4Z1Su1ASSBRWYsSwwavJEk1a9a0FluImpTzgxcYJWqYoRIvjxI1Hm48BTlXbdll5q\/dkSMonwtJMSClOXZ9icqeKb69RCRkq4atmMsvv9wsXLjQ+vj27NkzfosLPJlmY1OrdDkpWIkaraOEnCvf+ijnzOCW0O+0BtbKK15IaY5dX6K62zBffPGFDfN\/zTXXWKcH9ldxyj\/88MOjtTpk7jQbG7IKB2w2JWr+rnWJuW3n3iKSU56ElO2b1baqrvdweJpjN+\/2DFcttmvXzrr7DR482OzZs8degsNpGl2jlj8+K1G\/7jNIyc+2j\/YaJCZqLf8HJVFxkaJBfryZE5XKcovb\/fffb+9E5e8RI0aYvXv32j1UVF+\/y6KSGLppNjaJ+pXnMioiUb2ERFJu\/egrkuZLIjk7HFfHStAwTvZpjt3Qwc1Yq+L0cMghh6Q6VtNsbKoVLweFH6hEFdIhFUlIyLCE9Kq0ELNxnWqxYh6lOXYLrlFdN0GCneGxhHthFPfBKGM4zcZGqceBmLc8E9WVjHHJyHMiGZGSJSGl3\/hIc+wWu3uG+0Xk7lC\/yuBEz+Fx3Z4pf1Quq0QViShrRpeI9u\/9kjIK4l5C8mxcSRn2vZkRVSrEMbewt7qFbUTYfGk2NmwdDtR8pUHUIBKCcVT11NsvLhn5TiQkn4dZUybdz2mO3dBr1KQbFVRemo3Nqg1l9T1JE9VLwq\/U008tAUlhjDb5sBKyNalTzTSuy7qxjs2etmSM239pjt3Iqm++m7biNtB9Ls3GJlG\/8lxGWKIGETAJKejiZyXffhKKRCzLRCzU92mOXZWohdA\/QL63679t28yXNY6we4dW4iUsAQUqryT0krC0VNO0u7LUiMr+KWFRCLzNvaI4P6Rl7RUQ02xs2h2Vdfl+6z8hX9LSL4iETeoeateDqKOkA5WEYfo2zbFb8EqLs846y+CM\/8EHHxiuuZg4caI9UJ5WSrOxadU5yXLzkS+ptZ+3vn4SUAkYvVfTHLu+ROUM6o9\/\/GMbJbBly5a5GhPV4Te\/+Y1eaRGhD8NIPauGhvCWifDanHSzUm7\/OpB+vaBVY\/udSMCkLzOKWscDKX\/mRA0TMyktFTjNxiYxKFziWfVy\/3oPbxhRN9MinqiWLvn427WG5lM\/wxqTksCpIpaR5tj1lahEdJg0aZJp3ry56d27t42ST6zfJ554wqxevdpK1KpVq6bSF2k2Nl+FvZ4vaa\/1pC751E6RfEmt+5SoqQzZXKFpjt3ANSrugrgKSmzf7du3m+OPP95MmTLFNG78lfokCR\/gJ5980p6y4bwqFebQOXfXuAlJ7Xo++W31pNVY1x\/UtXYmrXK6xLNE2x8xPazUS3MoKVHTRNdkf3DcbY7ETEKqYlTyOzWD2Z+jb9wAV69ePbNs2TIb\/3f06NFFkGGgELR7+PDhgYglRVSRkPPX7bAb8HHc0MJIvfJk7VSiHmBERfIRKf\/UU0813bt3Nw0bNsx7rI0D5UQtROKSiN9LyFEuPXbTxo0bzV\/\/+tdin7t5IGpJIuVv\/\/hzc\/faXWbJpt0Fe6Xh4QfbPK0bfbW1IL\/l86NrHmzk74KFlYMMGik\/vU763ve+ZwtP6xLuwLtn2ENdvny5efzxx204losvvth06dLFHHXUUYGkXbNmjY2pxNp25syZxS7NZXvnzjvvNEwEXGeBZJW4wQJhXInqDTjldonrAZP0iYn0uj75klWiJo+pV8hkSlRvc3DS5woKDpJDMKI8BEV4kEDdXAT185\/\/vMgpm3Xr1lnDVKtWrcyuXbvsKZwrrrjCtG7dOvfKqET1hm101VXCZXx1nKl2uj1UTkpXoqbbUVHHbpTa5HUhxNKLZCW42aJFi8w\/\/\/lP06dPHxvozD3mhkoLkbmThhT23Opvf\/tbexDdvX8mSmMhKXFV5RYt3o30nNG3hZLTZxQoUaNQI3reKGM3aumBDg9z5swxCxYssEHM2KK58MILrRWX6y28adOmTWb27Nn2UDn5uUxq7dq11piEBCXJLeZt2rQx5513nvnPf\/5jrcqsY7H+RlV9g2KrpnHvR1RQy2p+JWq6PZM5USER2y1nnnlmIDndJsv2DOtS\/maNSqRCLoSiHFLXrl1zkvaNN96wBCYOE5+7luQwjfUjKXd+lMYZxHS7PtnSlajJ4uktLczYjVuDcnl6xr3vA3IqScN1vxI1HE5xcylRHeSQpq0mrLafKEmjDSklajS8ouZWou5HzKvyzuzXwhBnVVM4BJSo4XCKmytzomLtxSPpsMMOK2I8Cvo8bsP8nsvXWKy7I+Zvso91aFbbLB5xSpKvPuDLUqKm28WZERUicmMbe5xTp041Y8aMMbVq1cq1jjOp06ZNsw77WZ+ecaWpbsHEG3BK1Hi4hX0qM6Li2IATAt4V4m6Gk72bunXrZrdUZNslbCPC5gtqrHv9Ououaq+maAgoUaPhFTV3ZkSViqH24jyPyyDqb5YpqLFZXL+eZTtL411K1HRRz4yosgbFTZC9VPZEvSnpm8e95fs11rX06to0\/mBTosbHLsyTmRFVAm9fdtlldi2K+utNxxxzjPVAyvI2N3fflD1T9d0NM2yK51GixsMt7FOZETVshdLM59dYUXsxIv1lXLs0X39Al61ETbd7MyeqWH\/Lguqram9yg0uJmhyWfiVlTlSx\/rqqL3+zdXPOOefYcCxZqb6q9iY3uJSoyWFZJojqVwmk6\/r163MRGrLanlG1N7nBpURNDssyS1Qq9umnn5rp06fbeL9ZODyo2pvswFKiJotnmB2LpN4YGIoFNde7Rt2wYYPdX+UcaVr7q66e7zo53ND5WKNnTUvW7UrUkuFX6OkysUalkmzNDBs2zJx00kmF6hz7e7exuj6NDaPvg0rUZPEsdYmabnPyl+4SVdenyfaEEjVZPMsEUYmVhHWX+EgEH1u8eLHZvHmzGTRoUGoWXxruErXudc9aLNQbKZkBpkRNBsegUjJXfYnJi\/fRt771LdOzZ08byAyXQi6I4vfYsWNjOeVL0DMJxfKjH\/3Ixg12LcjSWNeQpOvTZAaYEjUZHMsMUYMuiQobXTCoITNmzLDr3IsuusgepSOc6KhRo4rE\/xWiuoYkdRtMZoApUZPBscwQlW0YjrtBKPcu1Jdeesk88MADsa9dRHUmgDeRComsj2pNRP4GDb6O0gBRiZR\/9wu7zJy1uywmd\/dqkItiny7UB3bpGik\/vf4tlUj5NOfNN98011xzjVV7CfH5+uuv26vlJ0+ebM4444zYLWYSuPHGG82SJUvsb9a8fqqvGpJiQxz4oErU5DF1S8x8jSov5\/pFYvayLq1Ro4a9Wyap\/VPuq+Gy5P79+5u2bdvm2iuNVUNS8oNKiZo8pqVKVEKuYEz6yU9+YmPzJpEwUM2aNcv06NEjtyYNipS\/Yv1ruUiDakhKAv2vylCiJoelX0mZS1Q8krgcas+ePZZYSUhRv0j5EyZMMAMHDjQtWnwdVoXGzlu+wXSftdFioZEGkxtcStTksCwTRPVeOOxWyu\/y4bDNx2oMOTEqkYIi5Q\/59TP2ThmSWnzDols4nxK1MEYlyZG5RC1JZUv6LI3tMnFp7uKnndM6lrRIfX4\/AkrUdIdCZkQtKzGTWo5+1N4QrhEdkh1YStRk8fSWlhlRy0rMpF095loM1HUw2YGlRE0Wz1IjarrNCFd605anm4\/Pn2Iza\/zecJiFzaVEDYtUvHyZSVS3eqXllN\/k9C5md4extiq6NRNvwAQ9pURNFs9Sl6hpOeWHgcklqlp8wyAWPo8SNTxWcXJmLlHTcsoP0\/iGXf\/P7G3e3WZVooZBLHweJWp4rOLkzJyoaTnlh2l8g94TzGdN2tusujUTBrHweZSo4bGKkzNzolLJtJzyCwFw5IDZ5vP6J+rWTCGgYnyvRI0BWoRHSoWo1C9Np\/yg9tcfutDsq15ft2YiDJCwWZWoYZGKl6\/UiBqvuvGf0vCg8bEL86QSNQxK8fNUSKLq1kz8ARP0pBI1eUzdEisMUTWOb7oDSYmaLr4Vhqjz171vRszfZNHUrZnkB5USNXlMS12iciSNcClEzHdTrVq1zMUXX5zIGVUvbBpwO92BpERNF9\/MJeru3bttPKOjjz7aBjc76KCDci2sWrWqOfnkkw2\/k05IU6QqiXtQOT2jKTkElKjJYelXUuZExTNp\/PjxNhJhUqFYwkAkAc3Iq84OYRCLlkeJGg2vqLkzJyphU+68805z7rnnGiI6ZJU08mC6SCtR08U3c6Ki+hIV\/6GHHjJnnXWWqV+\/fq6FQWtUYv5OmjTJEBitUaNGNvYvFXeTN8SLN6xLqwmrDXupeg41nQGlRE0HVyk1c6Jyeubll182\/PYmvzUq4UQJ\/UkMpBNPPNE899xzZvny5VZ1dteyDBSubRw+fLgvYhoiNN2BpERNF9\/MiRq1OVu3bjULFy60BCRgN2dZua4CCeteeLxx48bcjeXed7heSUNOr20Gn1E7ajU0fwEENFJ+ekOk1CLlc4Jm\/vz55pFHHrFScdy4cebVV181ffr0sWQMSqxvf\/e735ktW7bYZ6pUqZLLunLlSrv2RQWuXr26JXbnzp1N5cqVjTo7pDeIpGSVqOlinLlEhWwzZ860e6i9evUyc+bMMTfffLP5\/e9\/byAwV12411BI85GshAPFUnz99dcXu55x3bp19rlWrVrZS6JQja+44gp7raMrUatvuNdU2boqXVS1dEUgBQTefvvtFEo1ptKXRNv2JIKcobredNNNdg\/1lltusds1X3zxhbnttttsBP06derknqKIJ554wkpSLn3CSISULJT8IuVDWN0\/LYScfl\/REPAlKvfCoLZed9119uY1ISrSdNq0aZa0fC6Jm9m4AgODEhZfv+QXKZ9yBwwYkOkWUEXrYG3vgYGAL1Fp2mOPPWbXqFy9uHTpUvODH\/zAuhReeOGFpnfv3qZSpUo5BFBpBw8ebJo0aZKTpNyDCnlXr15t83Xt2tV4LzL2i5R\/YMCqrVAEkkUgkKios0R5ePjhh82GDRvsTW6Q9bTTTgul1iZbTS1NEajYCAQSFVggK3ukqK1Yb5O4LKpiw62tVwTiIRBIVKxXGJO4wBiVFosuFxizPg1ah8argj6lCCgChRDwJSpGIyy7bOLKPidSlT1VXAVxD0zj9Eyhyur3ikBFRcCXqEFxfYM+Lyl4TAL333+\/efDBB60jBJMEa2HXYFXSd1SU52WrDMcSJtOrrrrKdO\/evZhdYc2aNebSSy\/NwTJkyBC7taYpPgKbNm0yf\/zjH82oUaPiFxLwpC9RIc4999xjTjrpJOuUL4TBs2jVqlVm9OjRvg4PcWuHbzA\/OFXg1D9x4kQ7aLAca4qGAIOFvmPri35D+\/nhD39YbAsM5xXOG7dt2zbaCzR3MQRwDMLoyk7JOeeck8qEF+jwAFGef\/5507RpU1OtWjUbOpR1KuThfxKdnUSaMmWKad++vT2kjkTAqYIJgv81RUOAE09g2LdvX\/vgggUL7G\/5X0qbO3eutTm0bNky2gs0dzEE8Dt47bXXrIstgiYNzcSXqPlOz7i1TIJIrIfZb+3fv39u0DC4kAaXXHKJDouICDDpsWzo1KmTfVK0IHfwQOSpU6faE1JvvPGGzY+W1KxZs4hv0+wuAiwn0AwzI6r7cs6mIk3ZmnEd7JPqIoiKf\/Dll19uTjjhhJwUUKLGQ9jVToKISn8uW7bMErRBgwaGU02oyyw5XNfQeDWouE+VClHxImKd8+STT1rkcabv16+fufbaaxPtTK+qy\/8MNizODCRN0RDwqrr8j1GpZ8+egQUxGTNZXnnlldaxRVM8BDInKjMuZ0mbN29u3QUhKZKPBTNnTceOHZu4MWn9+vX2VA46PkRFfWC21xQNAYxJ8+bNs37XJKQk\/tQtWrTIFSS+2RjvsDmwvpo9e7bNW7NmzWgv1Nw5BDInalBwM\/HVRdK6B8JL2leyPYMhpEaNGtb6q9I0HqqyPQPx+Nv1pxbtCL9r\/LOxCO\/bt88ceeSR1pHFGzonXg0q7lOZExVViE5kD47QKpI2b95sD34jbd3TMxW3a7TlikA2CAS6EGIthKycljnllFPMW2+9ZU\/TEJWBw+SaFAFFIDsE8jrlsx59+umnzYcffmitvmzmHnfcceoxlF3\/6JsUAYtAXqIqRoqAIlA2EFCilo1+0FooAnkRUKLqAFEEygECStSQncTWEamQWyOWcU4CsY+ZL6xqyNfabBwtxIjHKZiw7mlpbhVEqTt55aSO92aEqOUE5Zfyu3XrZiZPnpwY7knVL4lylKghUSxNovLud999NzRJhRwl8TtlD5afMNEkC0EIkdgxSJNEaUyQhdqV5fdKVAdtBuaf\/vQnO6AIIM6heWIP87ec3eRkD\/fD4jzAfjJOIBwVYyurdu3a1g0PCSiz+969e83tt99uj0DhUMABBD9nDpw+Fi9ebO644w7rnYXv89VXX21eeeWV3Lv9zowS2vWuu+6yoVopH0nOYQnIQXlI9QceeMC0a9fO1pfoHLwL7yX2xKkfDhA4mRCPGa8wfnMyirLatGljPdL4nEgfYLJnzx47aYAXuwJgQhR+qbM3ZI9LVODGAwq\/bsi7fft265GG95Q3VrSX4PkIr0TNctoo5XdBEAjHIGRAM5AgwsiRI+2FWaL6btu2zQ5OBjJnOsnHQOU5d8AgjSDmt7\/9beuKyXcMUvxqOT7oJvateQcTA84kM2bMsIfohw4daonip3ZDOAiEl9igQYPshML\/\/CCBqR+TBAHP7733XnuXEG3BCR\/ySqSOX\/3qV7Y+qPU8S1uoJ\/VAKkN4ietMvOejjjrKtpVyeJa8kNuts3vo30tU7t4lXjTv\/+STT2xoWs7Mnn322UUwUaJ+DYdKVGdo4AMrszvHxNw1pktUCMIAwy8WifTMM89YyeclKuUhxSA1N+LJAQQksBxD4\/WUBwEYqPI5ESAhGcfRuHDLj6hIcyQh5G7cuLEth+fEf5cLuYiWgaRy16z4csuJKLzQIC1kF6LKMTm\/+rrH5pD+vEvW7bwb10UmJ1eqeolKnXlGDq0Tjlbwc5mqRFWiBspuDscz2FDpcJ8cM2aMlYguURnc06dPt3lQBxs2bGilj5eoSKaBAwda9c5NENc1SnHgASnD6SQZvDt27LCSikH\/1FNP+RIVCc1EAGG8vtdeY5L7vxtfGXWZCYn3ClGZMPjfr15uOUhfrjtxk5\/ByE\/1RU2WkzpBhi8lqhLVl6iovgxiTg0hTVasWGGlGYcQHn\/88RxZkAAvvvj4NlyDAAACMElEQVSilWZIK1fKuKovEpWbBUSNpABIzrle92wv0g2peP755+ckKhLm7rvvtkTkfX4SFRWc58iDCo5qixSFZKi+rjHJJQMqKtoA60JUVPegPuQTokq9cCOVIAFoDzj0MykxQYDV97\/\/fVs\/V9PIp\/oi5QkUgGsqiUkQ3FHL3YRqzdobbYPJRNeogfKlYn0ha0\/CpLJGhaj8sIbCuCIhTvibY3mQhDUsRIQoPIdP9H333WfXh6xR3WiO7733npWQENx77hPjFCTje0LdYOhBUkOmIIszxCT\/ySefbNfATBgMbMhG5IYgokIwNADWtdQJ0hFcnXAtLlHpfW5HePbZZ3PrSdoIOXnGu64mOB0nr7yXiPmtUevVq2ejSnBZGGXyDJeFuYnb56grRGViYdLDrdXPeqzGpArEVYgo1lxUTyQThh\/UQ2Z3ru3glroLLrjArjsXLVpkOnbsaC677DKbj9NG+EOjLnNcjwFF4CuITEQFQp0gnf0iLLqWWJGeWH1Z6+XbGhI1lvKx0DLoUdXzqb4MdiYL8iDZkJYQb9asWXa9LRKVeqD+8jnSHUstB9B5J\/k5Ioe0p51oI0wqRODznqzyIypLBi4WQ8OgrB49eljthLb++c9\/tmUecsghueiUWNTBnYmFSZBlBeeiWccz6SlRKxBRtanFEYBIWGjFsAaRmbSQhmGTH1Hd9XjYcvLlU6ImgaKWUW4REGcLQvBg6UZzwEPKu5WSr4FK1JJ3\/\/8DJbK57yiTHXsAAAAASUVORK5CYII=","height":141,"width":234}}
%---
%[output:337c0d48]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAOoAAACNCAYAAAC5ZeasAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQuUFcXRxxtUQBB5qghCMPgAQwwKKghRISpCBIGAghgUjLxRPxF8hIRoAEEiiQgE9KARE0FRIyDmgFExQkBAiEZFRXxAAiZRJAaUGMXv\/Frq2js7c+\/M7MzsLlt9zp7dvbenp\/vf\/e+qrq6urvTll19+aTQpAopAmUagkhK1TPePVk4RsAgUIerOnTvNlVdeaV566aVQ8Lz99tuh8mkmRUARKBkCxYh6yy23mPHjx5u6devmLRlSF8pTsqrp04qAIiAIFCHqf\/\/7X\/P666+b5s2bm6pVq+ZFibyF8ijMioAikAwCxSTqpEmTzJlnnmk6duxo6tSpk8xbtBRFQBEoEQLFJOpzzz1nHnnkEbN69WrTsmVLM3DgQEvcww47rEQv0ocVAUUgPgKBVt\/du3ebjRs3mkWLFpmnnnrKtG\/f3vTr18+0adPGHHroofHfWApPrlmzxlx66aXmwQcfNG3btk20Bt6yp0yZYvhs7ty5ia3hP\/30U3PjjTfaek+ePNn87W9\/sxPoqFGjzCWXXJJoe7yFhcXuoYceMnfddZe57777zPHHH5+3TlHyRmmc4LRkyRIzZMgQc8MNN4R63ItvaY3vzZs3m7Fjx5rbb7+9GIahtmc+++wzawm+5557zL\/+9a9EB2EoJMtRpgOJqLIL0Ldv34ITQhTyRckbpesZ6Exg3bp1C01Syi8rRKUuYLNgwYJiHMtL1H379pm\/\/\/3vZvny5VYdJvXv39\/06dOnmCFJZl4B9rbbbrOdK53dpEkTs3XrVkv473znO0Uq4j7rfieff\/e73zXPP\/+8LTrfTEkjb7rpplzfSl5XKpxwwgl2C0ok6\/bt24tIKf4nST3efPNNK42lPV4J4\/7\/3nvv5d7fsGHDYtLF+6wMLJGMQRi6A4kZd+TIkbktNGkjE8ScOXOKtZ0P5DvqdMQRR9g8IvFdKcTn0k7+pj7XX399rh3e7TsvRvJyNJd69epZ0giegseHH35o8XTz0hdu30E0NAc\/ySaYSbnSfm\/d\/MaJ91mpP+9BY2F8Iogo2x2HXoxcnGTSufnmmw32nV\/84hd2bOXD3FtXV9PzjgnBqRhRISeZ\/\/CHPxhUCFLv3r1N9+7dDWBXqlSp2CQnA0w6WUDn\/\/POO88SAwBQi0jurOc+yzsEMAaSkMQdjNTJT70KqgMgkET1FaLKXjGDQgY\/k4mrWvLd2WefHZqo0kFBqq9XQrmSRQawH4aCC+3wU33zqd+CIThI24Wo7gB18ZY6uNqB5JU6MHhlwkPFdNtyzDHH5FXVvRJVxotbR+kLl6xeiSnt9pKVfvCqva7A8Pbx1Vdfbev74osvFhmjMoHma9vixYuLCIeg8SaY8xvcpH3yvJBV6ultQxGifvTRR2bYsGFmy5YtVmpedNFFVleuXLlyMXK6H3jVPQG0devWxUgA8G5+1GmXfG6nuQSTWTdoHZRPnfKTqK5UCZrFRKqElaiFiOpKRvaq2bOWgT99+vQia1sXw5\/+9Kfm1ltvzeUNWqO6M7VIBPB1Jw4XeyYHd9J0B7N38hJi+mlEVCwIf1dSupOQ9KOQGmkmUp46+k3I3ndIfb0k8CNqvj72qr7eSUjGuiuRReoL0VxNxMsH93+vhuadfLwTikxUxay+O3bsMIB38MEH5yWnfCmNdIF2AZw6daoZM2ZMbgbxI6qrsrkqUZpEdWfsrIjqDmhUJDqQ9Z9Xk8CRxMVwxowZ1sAQJFG9Kh35IKo85\/aNuwYSoooaKdj7aRn0m3dpQX6RBF4SeVVx8uYjqmhvUge\/pYOXwDL2RBKibrtS3h3AJSGqd0lCuV6iCg5+fHAxF6J6yVVIKygmUelc1kCF9lCRvpInikSlgq56653x3QZ4Vbp8UtOVxMyo7rMu4UX98yOqGCFc9aNXr15FLKz51qiFJCr1cGdQV2rkw7CQRPUOYK\/GUkii+lmPg2Z2tw3uWo6dAZGS5HEltZck+VTJfBbXJCSqXx+L6isToStR5TuZ7LxLAK\/qShlhJGrQDoRgJWtd4UMxh4drr73WeiYVMlHTGBbQfuphnDWqVNxPTQiatf1mTOmIoHL8iOqdBd11nRhF3HLRAKROUbdnXMOEazTJt84vtEZ1B7BIFVEHIRAGtnxr1CBJ7WJIHldaedVFBqwQ1SvZ3PGAgdFvjepVhaVOSa1R8\/Ux2oe7\/eVVff2WCyzrWOv6EdVvzS394cXRb\/yIgdNtezHV9+WXXza4B4ZJHTp0yGUrZPUlo59FzSU6f7sqTxSJyrOuahZUjh9RvVLCVdO8Fj+xQAcRVersp7oJWN6BK5+HsfoyOEQrkf3Cq666KneYgkGHtsB37uTH5BLG6utaS71WX2\/9XMuoq37zXtcCzhp\/xYoVRga3rLEZkO4kLBZ7766A34TstfqSJ8gQI897lwiiihdao7rPYVyU8YJRc8OGDbmJ0N2jj2L1lXqUeB81DGmD8uRToUpSrj4bD4Eo+7wygPFMS9uxIl5rysdTUTAPalEoh4eSwKFELQl6JX\/Wld5eFbrQ8sbVdtLw6ip568pmCSXF3K9VqRO1bEKptVIEyhcCStTy1V9a2wqKQF6iYvxZu3at+cc\/\/mG6dOli91bFBS0tvL75zW+mVbSWqwikhsBnTdqbT04dZMtfPPwU0+G42om+K5CoK1euND\/72c\/M6aefbk9riFf\/gAEDzLnnnptoJdzCIKqGeEkHXsU2HVwpdf66982I+ZuyJSpH3CApZn8kqIRnwZMFMz\/fpXU+VQdTeoPpnXfeMccee2x6L6jAJZcKUbHU4ouKR8xBBx2UIyr9EDamUtw+U6LGRa7wc0rUwhjFzVEqRP3888+tb2mjRo2smsvf48aNM08\/\/bSNqYRHUpUqVeK2Ke9zStRUYLWFKlHTw3bKsnfNlGXvZKv68raPP\/7YuoTNmzfP\/O9\/\/7OGpM6dO1spm6ZBSYma3mBSoqaHbakRVZpEdAfWrGyOh9kgDwMFEnvatGnmG9\/4RjGPFyVqGATj5VGixsMtzFMuUf8yrp1pUrdamMdC5\/G1+nJ4\/N\/\/\/rcJuu2Cw+O1atUqeE41qBZLly41v\/zlL62xyuuapkQN3XeRMypRI0MW+gEsvqxTSZkRlSNsnJAn\/EnTpk1NtWrVrCq8bds206xZs1zliQIRNXEyBRcrzrxWr17dl6ishTUljwDbbOCuKXkEzhi\/zHxe\/0QrSSFq0slXoqKa3nHHHTb2i5wWQLpyAuLVV181Q4cONatWrcp9F7ZSqNBEMuA0xQsvvGAfU4kaFr2S51OJWnIMg0qoe92z9qtMicr2jN82jPs5lYpypQVEf\/jhh60hqlOnTlaqKlHTGzh+JStR08F75Vu7TPdZG23hN3Q+1tzQuWniL\/KVqJxHJUYP502x9BIzSSTqY489ZkNqvPLKK5Fi5PpFcqM13mhxukZNvI9zBSpR08G2+8yNZuWWXbbwNNwHKTfQhfD99983EyZMyK1TkaZYfQnVgjGJE\/GPPvpo7JarRI0NXewHlaixoQt80LX2Vv7kA\/PB7D7JvyQfUeVtrCvZoimppddbeyVqKv2Zt1AlanKYb92511p5xcmBtenOxbearWujG1jD1Crv6RnUVX4ksWWDAwQBnwoFPwvzcr88qvrGRa7wc0rUwhiFyQFJR87flFN3IWm\/0442c4Z1Su1ASSBRWYsSwwavJEk1a9a0FluImpTzgxcYJWqYoRIvjxI1Hm48BTlXbdll5q\/dkSMonwtJMSClOXZ9icqeKb69RCRkq4atmMsvv9wsXLjQ+vj27NkzfosLPJlmY1OrdDkpWIkaraOEnCvf+ijnzOCW0O+0BtbKK15IaY5dX6K62zBffPGFDfN\/zTXXWKcH9ldxyj\/88MOjtTpk7jQbG7IKB2w2JWr+rnWJuW3n3iKSU56ElO2b1baqrvdweJpjN+\/2DFcttmvXzrr7DR482OzZs8degsNpGl2jlj8+K1G\/7jNIyc+2j\/YaJCZqLf8HJVFxkaJBfryZE5XKcovb\/fffb+9E5e8RI0aYvXv32j1UVF+\/y6KSGLppNjaJ+pXnMioiUb2ERFJu\/egrkuZLIjk7HFfHStAwTvZpjt3Qwc1Yq+L0cMghh6Q6VtNsbKoVLweFH6hEFdIhFUlIyLCE9Kq0ELNxnWqxYh6lOXYLrlFdN0GCneGxhHthFPfBKGM4zcZGqceBmLc8E9WVjHHJyHMiGZGSJSGl3\/hIc+wWu3uG+0Xk7lC\/yuBEz+Fx3Z4pf1Quq0QViShrRpeI9u\/9kjIK4l5C8mxcSRn2vZkRVSrEMbewt7qFbUTYfGk2NmwdDtR8pUHUIBKCcVT11NsvLhn5TiQkn4dZUybdz2mO3dBr1KQbFVRemo3Nqg1l9T1JE9VLwq\/U008tAUlhjDb5sBKyNalTzTSuy7qxjs2etmSM239pjt3Iqm++m7biNtB9Ls3GJlG\/8lxGWKIGETAJKejiZyXffhKKRCzLRCzU92mOXZWohdA\/QL63679t28yXNY6we4dW4iUsAQUqryT0krC0VNO0u7LUiMr+KWFRCLzNvaI4P6Rl7RUQ02xs2h2Vdfl+6z8hX9LSL4iETeoeateDqKOkA5WEYfo2zbFb8EqLs846y+CM\/8EHHxiuuZg4caI9UJ5WSrOxadU5yXLzkS+ptZ+3vn4SUAkYvVfTHLu+ROUM6o9\/\/GMbJbBly5a5GhPV4Te\/+Y1eaRGhD8NIPauGhvCWifDanHSzUm7\/OpB+vaBVY\/udSMCkLzOKWscDKX\/mRA0TMyktFTjNxiYxKFziWfVy\/3oPbxhRN9MinqiWLvn427WG5lM\/wxqTksCpIpaR5tj1lahEdJg0aZJp3ry56d27t42ST6zfJ554wqxevdpK1KpVq6bSF2k2Nl+FvZ4vaa\/1pC751E6RfEmt+5SoqQzZXKFpjt3ANSrugrgKSmzf7du3m+OPP95MmTLFNG78lfokCR\/gJ5980p6y4bwqFebQOXfXuAlJ7Xo++W31pNVY1x\/UtXYmrXK6xLNE2x8xPazUS3MoKVHTRNdkf3DcbY7ETEKqYlTyOzWD2Z+jb9wAV69ePbNs2TIb\/3f06NFFkGGgELR7+PDhgYglRVSRkPPX7bAb8HHc0MJIvfJk7VSiHmBERfIRKf\/UU0813bt3Nw0bNsx7rI0D5UQtROKSiN9LyFEuPXbTxo0bzV\/\/+tdin7t5IGpJIuVv\/\/hzc\/faXWbJpt0Fe6Xh4QfbPK0bfbW1IL\/l86NrHmzk74KFlYMMGik\/vU763ve+ZwtP6xLuwLtn2ENdvny5efzxx204losvvth06dLFHHXUUYGkXbNmjY2pxNp25syZxS7NZXvnzjvvNEwEXGeBZJW4wQJhXInqDTjldonrAZP0iYn0uj75klWiJo+pV8hkSlRvc3DS5woKDpJDMKI8BEV4kEDdXAT185\/\/vMgpm3Xr1lnDVKtWrcyuXbvsKZwrrrjCtG7dOvfKqET1hm101VXCZXx1nKl2uj1UTkpXoqbbUVHHbpTa5HUhxNKLZCW42aJFi8w\/\/\/lP06dPHxvozD3mhkoLkbmThhT23Opvf\/tbexDdvX8mSmMhKXFV5RYt3o30nNG3hZLTZxQoUaNQI3reKGM3aumBDg9z5swxCxYssEHM2KK58MILrRWX6y28adOmTWb27Nn2UDn5uUxq7dq11piEBCXJLeZt2rQx5513nvnPf\/5jrcqsY7H+RlV9g2KrpnHvR1RQy2p+JWq6PZM5USER2y1nnnlmIDndJsv2DOtS\/maNSqRCLoSiHFLXrl1zkvaNN96wBCYOE5+7luQwjfUjKXd+lMYZxHS7PtnSlajJ4uktLczYjVuDcnl6xr3vA3IqScN1vxI1HE5xcylRHeSQpq0mrLafKEmjDSklajS8ouZWou5HzKvyzuzXwhBnVVM4BJSo4XCKmytzomLtxSPpsMMOK2I8Cvo8bsP8nsvXWKy7I+Zvso91aFbbLB5xSpKvPuDLUqKm28WZERUicmMbe5xTp041Y8aMMbVq1cq1jjOp06ZNsw77WZ+ecaWpbsHEG3BK1Hi4hX0qM6Li2IATAt4V4m6Gk72bunXrZrdUZNslbCPC5gtqrHv9Ououaq+maAgoUaPhFTV3ZkSViqH24jyPyyDqb5YpqLFZXL+eZTtL411K1HRRz4yosgbFTZC9VPZEvSnpm8e95fs11rX06to0\/mBTosbHLsyTmRFVAm9fdtlldi2K+utNxxxzjPVAyvI2N3fflD1T9d0NM2yK51GixsMt7FOZETVshdLM59dYUXsxIv1lXLs0X39Al61ETbd7MyeqWH\/Lguqram9yg0uJmhyWfiVlTlSx\/rqqL3+zdXPOOefYcCxZqb6q9iY3uJSoyWFZJojqVwmk6\/r163MRGrLanlG1N7nBpURNDssyS1Qq9umnn5rp06fbeL9ZODyo2pvswFKiJotnmB2LpN4YGIoFNde7Rt2wYYPdX+UcaVr7q66e7zo53ND5WKNnTUvW7UrUkuFX6OkysUalkmzNDBs2zJx00kmF6hz7e7exuj6NDaPvg0rUZPEsdYmabnPyl+4SVdenyfaEEjVZPMsEUYmVhHWX+EgEH1u8eLHZvHmzGTRoUGoWXxruErXudc9aLNQbKZkBpkRNBsegUjJXfYnJi\/fRt771LdOzZ08byAyXQi6I4vfYsWNjOeVL0DMJxfKjH\/3Ixg12LcjSWNeQpOvTZAaYEjUZHMsMUYMuiQobXTCoITNmzLDr3IsuusgepSOc6KhRo4rE\/xWiuoYkdRtMZoApUZPBscwQlW0YjrtBKPcu1Jdeesk88MADsa9dRHUmgDeRComsj2pNRP4GDb6O0gBRiZR\/9wu7zJy1uywmd\/dqkItiny7UB3bpGik\/vf4tlUj5NOfNN98011xzjVV7CfH5+uuv26vlJ0+ebM4444zYLWYSuPHGG82SJUvsb9a8fqqvGpJiQxz4oErU5DF1S8x8jSov5\/pFYvayLq1Ro4a9Wyap\/VPuq+Gy5P79+5u2bdvm2iuNVUNS8oNKiZo8pqVKVEKuYEz6yU9+YmPzJpEwUM2aNcv06NEjtyYNipS\/Yv1ruUiDakhKAv2vylCiJoelX0mZS1Q8krgcas+ePZZYSUhRv0j5EyZMMAMHDjQtWnwdVoXGzlu+wXSftdFioZEGkxtcStTksCwTRPVeOOxWyu\/y4bDNx2oMOTEqkYIi5Q\/59TP2ThmSWnzDols4nxK1MEYlyZG5RC1JZUv6LI3tMnFp7uKnndM6lrRIfX4\/AkrUdIdCZkQtKzGTWo5+1N4QrhEdkh1YStRk8fSWlhlRy0rMpF095loM1HUw2YGlRE0Wz1IjarrNCFd605anm4\/Pn2Iza\/zecJiFzaVEDYtUvHyZSVS3eqXllN\/k9C5md4extiq6NRNvwAQ9pURNFs9Sl6hpOeWHgcklqlp8wyAWPo8SNTxWcXJmLlHTcsoP0\/iGXf\/P7G3e3WZVooZBLHweJWp4rOLkzJyoaTnlh2l8g94TzGdN2tusujUTBrHweZSo4bGKkzNzolLJtJzyCwFw5IDZ5vP6J+rWTCGgYnyvRI0BWoRHSoWo1C9Np\/yg9tcfutDsq15ft2YiDJCwWZWoYZGKl6\/UiBqvuvGf0vCg8bEL86QSNQxK8fNUSKLq1kz8ARP0pBI1eUzdEisMUTWOb7oDSYmaLr4Vhqjz171vRszfZNHUrZnkB5USNXlMS12iciSNcClEzHdTrVq1zMUXX5zIGVUvbBpwO92BpERNF9\/MJeru3bttPKOjjz7aBjc76KCDci2sWrWqOfnkkw2\/k05IU6QqiXtQOT2jKTkElKjJYelXUuZExTNp\/PjxNhJhUqFYwkAkAc3Iq84OYRCLlkeJGg2vqLkzJyphU+68805z7rnnGiI6ZJU08mC6SCtR08U3c6Ki+hIV\/6GHHjJnnXWWqV+\/fq6FQWtUYv5OmjTJEBitUaNGNvYvFXeTN8SLN6xLqwmrDXupeg41nQGlRE0HVyk1c6Jyeubll182\/PYmvzUq4UQJ\/UkMpBNPPNE899xzZvny5VZ1dteyDBSubRw+fLgvYhoiNN2BpERNF9\/MiRq1OVu3bjULFy60BCRgN2dZua4CCeteeLxx48bcjeXed7heSUNOr20Gn1E7ajU0fwEENFJ+ekOk1CLlc4Jm\/vz55pFHHrFScdy4cebVV181ffr0sWQMSqxvf\/e735ktW7bYZ6pUqZLLunLlSrv2RQWuXr26JXbnzp1N5cqVjTo7pDeIpGSVqOlinLlEhWwzZ860e6i9evUyc+bMMTfffLP5\/e9\/byAwV12411BI85GshAPFUnz99dcXu55x3bp19rlWrVrZS6JQja+44gp7raMrUatvuNdU2boqXVS1dEUgBQTefvvtFEo1ptKXRNv2JIKcobredNNNdg\/1lltusds1X3zxhbnttttsBP06derknqKIJ554wkpSLn3CSISULJT8IuVDWN0\/LYScfl\/REPAlKvfCoLZed9119uY1ISrSdNq0aZa0fC6Jm9m4AgODEhZfv+QXKZ9yBwwYkOkWUEXrYG3vgYGAL1Fp2mOPPWbXqFy9uHTpUvODH\/zAuhReeOGFpnfv3qZSpUo5BFBpBw8ebJo0aZKTpNyDCnlXr15t83Xt2tV4LzL2i5R\/YMCqrVAEkkUgkKios0R5ePjhh82GDRvsTW6Q9bTTTgul1iZbTS1NEajYCAQSFVggK3ukqK1Yb5O4LKpiw62tVwTiIRBIVKxXGJO4wBiVFosuFxizPg1ah8argj6lCCgChRDwJSpGIyy7bOLKPidSlT1VXAVxD0zj9Eyhyur3ikBFRcCXqEFxfYM+Lyl4TAL333+\/efDBB60jBJMEa2HXYFXSd1SU52WrDMcSJtOrrrrKdO\/evZhdYc2aNebSSy\/NwTJkyBC7taYpPgKbNm0yf\/zjH82oUaPiFxLwpC9RIc4999xjTjrpJOuUL4TBs2jVqlVm9OjRvg4PcWuHbzA\/OFXg1D9x4kQ7aLAca4qGAIOFvmPri35D+\/nhD39YbAsM5xXOG7dt2zbaCzR3MQRwDMLoyk7JOeeck8qEF+jwAFGef\/5507RpU1OtWjUbOpR1KuThfxKdnUSaMmWKad++vT2kjkTAqYIJgv81RUOAE09g2LdvX\/vgggUL7G\/5X0qbO3eutTm0bNky2gs0dzEE8Dt47bXXrIstgiYNzcSXqPlOz7i1TIJIrIfZb+3fv39u0DC4kAaXXHKJDouICDDpsWzo1KmTfVK0IHfwQOSpU6faE1JvvPGGzY+W1KxZs4hv0+wuAiwn0AwzI6r7cs6mIk3ZmnEd7JPqIoiKf\/Dll19uTjjhhJwUUKLGQ9jVToKISn8uW7bMErRBgwaGU02oyyw5XNfQeDWouE+VClHxImKd8+STT1rkcabv16+fufbaaxPtTK+qy\/8MNizODCRN0RDwqrr8j1GpZ8+egQUxGTNZXnnlldaxRVM8BDInKjMuZ0mbN29u3QUhKZKPBTNnTceOHZu4MWn9+vX2VA46PkRFfWC21xQNAYxJ8+bNs37XJKQk\/tQtWrTIFSS+2RjvsDmwvpo9e7bNW7NmzWgv1Nw5BDInalBwM\/HVRdK6B8JL2leyPYMhpEaNGtb6q9I0HqqyPQPx+Nv1pxbtCL9r\/LOxCO\/bt88ceeSR1pHFGzonXg0q7lOZExVViE5kD47QKpI2b95sD34jbd3TMxW3a7TlikA2CAS6EGIthKycljnllFPMW2+9ZU\/TEJWBw+SaFAFFIDsE8jrlsx59+umnzYcffmitvmzmHnfcceoxlF3\/6JsUAYtAXqIqRoqAIlA2EFCilo1+0FooAnkRUKLqAFEEygECStSQncTWEamQWyOWcU4CsY+ZL6xqyNfabBwtxIjHKZiw7mlpbhVEqTt55aSO92aEqOUE5Zfyu3XrZiZPnpwY7knVL4lylKghUSxNovLud999NzRJhRwl8TtlD5afMNEkC0EIkdgxSJNEaUyQhdqV5fdKVAdtBuaf\/vQnO6AIIM6heWIP87ec3eRkD\/fD4jzAfjJOIBwVYyurdu3a1g0PCSiz+969e83tt99uj0DhUMABBD9nDpw+Fi9ebO644w7rnYXv89VXX21eeeWV3Lv9zowS2vWuu+6yoVopH0nOYQnIQXlI9QceeMC0a9fO1pfoHLwL7yX2xKkfDhA4mRCPGa8wfnMyirLatGljPdL4nEgfYLJnzx47aYAXuwJgQhR+qbM3ZI9LVODGAwq\/bsi7fft265GG95Q3VrSX4PkIr0TNctoo5XdBEAjHIGRAM5AgwsiRI+2FWaL6btu2zQ5OBjJnOsnHQOU5d8AgjSDmt7\/9beuKyXcMUvxqOT7oJvateQcTA84kM2bMsIfohw4daonip3ZDOAiEl9igQYPshML\/\/CCBqR+TBAHP7733XnuXEG3BCR\/ySqSOX\/3qV7Y+qPU8S1uoJ\/VAKkN4ietMvOejjjrKtpVyeJa8kNuts3vo30tU7t4lXjTv\/+STT2xoWs7Mnn322UUwUaJ+DYdKVGdo4AMrszvHxNw1pktUCMIAwy8WifTMM89YyeclKuUhxSA1N+LJAQQksBxD4\/WUBwEYqPI5ESAhGcfRuHDLj6hIcyQh5G7cuLEth+fEf5cLuYiWgaRy16z4csuJKLzQIC1kF6LKMTm\/+rrH5pD+vEvW7bwb10UmJ1eqeolKnXlGDq0Tjlbwc5mqRFWiBspuDscz2FDpcJ8cM2aMlYguURnc06dPt3lQBxs2bGilj5eoSKaBAwda9c5NENc1SnHgASnD6SQZvDt27LCSikH\/1FNP+RIVCc1EAGG8vtdeY5L7vxtfGXWZCYn3ClGZMPjfr15uOUhfrjtxk5\/ByE\/1RU2WkzpBhi8lqhLVl6iovgxiTg0hTVasWGGlGYcQHn\/88RxZkAAvvvj4NlyDAAACMElEQVSilWZIK1fKuKovEpWbBUSNpABIzrle92wv0g2peP755+ckKhLm7rvvtkTkfX4SFRWc58iDCo5qixSFZKi+rjHJJQMqKtoA60JUVPegPuQTokq9cCOVIAFoDzj0MykxQYDV97\/\/fVs\/V9PIp\/oi5QkUgGsqiUkQ3FHL3YRqzdobbYPJRNeogfKlYn0ha0\/CpLJGhaj8sIbCuCIhTvibY3mQhDUsRIQoPIdP9H333WfXh6xR3WiO7733npWQENx77hPjFCTje0LdYOhBUkOmIIszxCT\/ySefbNfATBgMbMhG5IYgokIwNADWtdQJ0hFcnXAtLlHpfW5HePbZZ3PrSdoIOXnGu64mOB0nr7yXiPmtUevVq2ejSnBZGGXyDJeFuYnb56grRGViYdLDrdXPeqzGpArEVYgo1lxUTyQThh\/UQ2Z3ru3glroLLrjArjsXLVpkOnbsaC677DKbj9NG+EOjLnNcjwFF4CuITEQFQp0gnf0iLLqWWJGeWH1Z6+XbGhI1lvKx0DLoUdXzqb4MdiYL8iDZkJYQb9asWXa9LRKVeqD+8jnSHUstB9B5J\/k5Ioe0p51oI0wqRODznqzyIypLBi4WQ8OgrB49eljthLb++c9\/tmUecsghueiUWNTBnYmFSZBlBeeiWccz6SlRKxBRtanFEYBIWGjFsAaRmbSQhmGTH1Hd9XjYcvLlU6ImgaKWUW4REGcLQvBg6UZzwEPKu5WSr4FK1JJ3\/\/8DJbK57yiTHXsAAAAASUVORK5CYII=","height":141,"width":234}}
%---
