clear;
[model, options] = init_environment('afe_inv_psm');

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
% t_measure = 0.648228176318064;
t_measure = 0.2;
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
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:23214e0a]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:2bba31af]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:05734e77]
hwdata.three_phase_dab = three_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:3721aea2]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:6408583f]

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
parasitic_dclink_data; %[output:6ab00b53]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:060d3403]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:0a19f78c]
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
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:4e9788d7]
psm_ctrl.kp_i = 0.25;
psm_ctrl.ki_i = 35;
%[text] #### Induction Motor Control
im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:097d7557] %[output:4b93a0be]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:097d7557] %[output:4b93a0be]

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
sogi = sogi_filter(omega_set, sogi_delta, kepsilon, glb_time.ts_afe); %[output:2118ab0d]
%[text] #### Current control parameters DQ PI
dqvector_pi.kp_inv = 0.5;
dqvector_pi.ki_inv = 45;
dqvector_pi.pi_ctrl = dqvector_pi.kp_inv + dqvector_pi.ki_inv/s;
dqvector_pi.pid_ctrl = c2d(dqvector_pi.pi_ctrl, glb_time.ts_inv);
dqvector_pi.plant = 1/(s*grid_emu.trafo.Ld1 + 1);
dqvector_pi.plantd = c2d(dqvector_pi.plant, glb_time.ts_inv);

G = sogi.fltd.alpha * dqvector_pi.pid_ctrl * dqvector_pi.plantd;
figure; margin(G, options);  %[output:11375b46]
grid on %[output:11375b46]
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
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage_1, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:290f5cf8]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage_2, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:9a782e7f]
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
%[output:23214e0a]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:2bba31af]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:05734e77]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:3721aea2]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: Three_phase_DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 1.200000e-04 [H] | Internal Tank Cs: 750 [F]\n---------------------------\n","truncated":false}}
%---
%[output:6408583f]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.000000e-05 [H] | Internal Tank Cs: 1.465874e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:6ab00b53]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAV8AAADTCAYAAAAmsBCWAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQeUFMXWvsgCi4+0CJIUliyuBBUMjxVW5YkCIiAgQYIgkkHRAyJJUQmKEiQJEp8K+HggoPhAwqLwiyKSJIngoiTJsCgryf98F3vsbXqme2a7p2tmbp3DYXe2urr6q6pvbn91695sf\/75558kRRAQBAQBQSCsCGQT8g0r3nIzQUAQEAQYASFfmQiCgCAgCHiAgJCvB6DLLQUBQUAQEPKVOSAICAKCgAcICPl6ALrcUhAQBAQBIV+ZA4KAICAIeICAkK8HoMstBQFBQBAQ8pU5IAgIAoKABwgI+XoAutxSEBAEBAEhX5kDgoAgIAh4gICQrwegyy0FAUFAEBDylTkgCAgCgoAHCAj5egB6rN9yz5491L17d8qdOzddd911dPLkSapSpQoNHjyYChcubAueefPmUalSpeiee+6xrL9+\/Xrq3LkzJSYm+u7XokULat++Pf8+ceJEatSoEZUuXdqyLakgCDiFgJCvU0hKO7YRAPnOmjWLBgwYwAR85coVGjduHJNp48aNbbUTLPnOmTOHRowYwff7448\/aPr06XThwgX+EoiLi7N1T3+VEBgQ\/0DkUgQBuwgI+dpFSuo5hoCRfEGGI0eOpNq1a1PNmjVp9uzZNHfuXL4fLNS2bdvyzyDsDz\/8kK3jsmXL0qOPPkp33303LV26lN577z26dOkS1atXjy1akKxWYPnqyRefHz58mIYMGUIvvfQSX9uuXTsqVqwYfwl8+eWXTMytW7fme2dkZNDo0aMpNTWVLXS0\/dRTT9F3331HO3bsoLS0NHr22Wf5\/sOGDWNyRx\/RfoECBfget912G3322WdUpkwZeuihh2jmzJl05swZev311\/kZpMQeAkK+sTfmnj8xyBfkdejQIV9f6tevT8OHD6eNGzfSmjVrmLBQxo4dS0lJSUx4+Lxfv3508eJF6t+\/P5Njrly5+PMePXpQ9uzZmbgLFSpEaC8Q+f72229MfC1btiRY0SDfEydOMJE+8cQTlJ6ezpZyz5496fPPP+f7NG\/enH799VcaNGgQ9e3bl8l37dq13O9s2bLR5MmTqVWrVkziy5cvpwMHDrCc0bVrV+rWrRvVqlWL615\/\/fXUq1cv2rRpE\/3vf\/\/jZ8qq9e35oEoHgkZAyDdoyOSCrCJgJjvAMgXxxcfHsyWYnJzMt4HVCnJFgVWsfQ7LGBouLM\/XXnstU5eg74LQgiVfWNN79+7lf1u2bKEvvviC3njjDfroo4+Y6CtUqMDywvjx4+nhhx9m8kUBWaOAsLdv304\/\/PADW8\/lypWjTp060SuvvMJWcMGCBdnC157DiENWcZXrIwsBId\/IGq+o6K0Z6WifwcJ98MEHfRtpsCy\/\/vprfqU3I99vv\/2WX\/E1AoRccPnyZVuyAyQCkDQsVli+27Zt43+whhMSEtjqRruaZVy+fHnuBz5v2LBhJvI9duwYDR06lGWP6tWr0+7du2ndunWm5At5BRuFQr5RMZ1Dfggh35ChkwtDRcBIOrAmYV3++OOPdMcdd7A127t3b24eBAmyOn\/+PH3zzTcsR+Dn559\/njp06MBkuHDhQrYuc+TIwVap3nLWrGfjhhvq4fUf+jDkB5DvggULfASPPmBDEJbvsmXLmOAhOxw8eJD7AOlBb\/nimWbMmMHXQEIYM2YM99\/M8hXyDXXmRNd1Qr7RNZ4R8TRGVzNsaN1yyy1MarA4zTbcYM1OnTqViRbyAHTVRx55hK1MEDc2sFC0DTq9hhrI1QzXaOSLjTJoudBvodVCk86fPz9LDpA2du7cydJDnjx5qGPHjpnIF9e+8847tGTJEqpYsSLLEugXLGv0TS87CPlGxDR1vZNCvq5DLDeIdARWrFhBJUuWZOI9fvw4vfXWW2x5Y2NPiiAQKgJCvqEiJ9fFDAL79u1jWQO6LrweYKHXqFEjZp5fHtQdBIR83cFVWhUEBAFBICACQr4yQQQBQUAQ8AABIV8PQJdbCgKCgCAg5CtzQBAQBAQBDxAQ8vUAdLmlICAICAJCvjIHBAFBQBDwAAEhXw9Al1sKAoKAICDkK3NAEBAEBAEPEBDy9QB0uaUgIAgIAkK+MgcEAUFAEPAAASFfD0CXWwoCgoAgoBz5IlzgV199xRGhkDUApUiRIhzO7957780Up1WGTxAQBASBSEVAGfJFTNfNmzdzYOv777+fg03ny5ePcQUhI3bqokWLOLwf\/i5FEBAEBIFIRkAZ8j137hynb6lcubLfLLDIcosMAZUqVYpkzKXvgoAgIAiQMuSLsQC57t+\/nzO+IsvAqlWraNeuXRxo+4EHHuBwflIEAUFAEIgGBJQhX6SDmTBhAi1evJiz0IJwQcBIsw0N+IYbbiAkRkSWASmCgCAgCEQ6AsqQ76lTp2jUqFH0wgsvENLKIHj1q6++yrmzIEm8\/fbbnMYbaWakCAKCgCAQ6QgoQ74nT57kHF29evXiDTZ9um38Pm7cOE5GiPTbUgQBQUAQiHQEhHwjfQSl\/4KAIBCRCChFvv379+fMtNB\/U1NTKSUlhf16L168yJ4Qw4cPF8s3IqeZdFoQEASMCChDvki9vXXrVsL\/ZgWeDlWqVHHc46FMmTIyKwQBQSDGEEBSVK+LMuQLN7MzZ84QDluYFXg55M+f368PsD8g0d4nn3xCY8eOZeKGbtywYUNfOyBfFQbC64ng7\/6Cj\/XICEaBMVINH1X6owz5wtth8ODBdODAATp79iwdP36cEhMTeVRPnDhBTZs2paeffpry5MljvRp0NXbu3MkbedjAA4G\/\/PLL1KZNG6patSrXsjsQP\/30E5UuXdr03sa\/Bfpd+5vV\/3Ye0l+fzD7Xf2b2s\/Ez3B\/PG+i5jX10sj9o2wwjrV9ZwUffttaOamOmWn+MmNmdT6gX7jELNL\/xtwcffFAJg0sZ8tUWAWSHSZMmUbNmzahEiRL8MQh5\/vz51LVr16Blh3nz5rE13aJFC25r7ty5\/L\/2u5Dv1S8VId\/MX65WC1iPmRtfUEK+13692jVyrMZOyNeP6QKXM8R36N27N\/3jH\/\/gWr\/99hvLBl26dAl6w23kyJFUo0YNPiGHsnbtWlq3bh3169cvKMvXjqUVjXWCIZZofH47z+Q0RiOXpflu+\/PJ876ffzmZkak7P5\/KoJIJ8fzZzQXjqWTB3NSv7tW3RZWK0\/hk9dnsGlxZvY\/V9cpZvhcuXKBhw4axNVavXj3uv3bM+MUXXwza8gX51qxZk5KTk03Jt2Cf1VYYyd8FgbAhUDxfXKZ7Fcv79+\/GvxXPG0eH0i9x\/UNnL9HGg1fJGfXuLBFPj1bKw\/97XfDmetNNN3nbjVMHiPZ+TZunDaKqeTOozH\/M95bC2UnlyBcPD0sX8gA2yiBD4Ihxhw4dfFHOggHIKDPgd2y8NW7cmJux+y0omq816qL5OodRqLJD9vzFaNLynbT9BNHavaepZMF4Gt+iEiWXK2DauWgcM+2ZLh1Lo58XjqH4Q1soY3sqxRVOpEul7qQ3539BkzYctR4sl2soQ74g2fT0dCpUqFDAR8ZGnFUdfQPYcJs9ezYNGDCAP3799depbdu2vshodsnX5XFQtnnVXhlVBEpVjH4+mUEjl\/1EczYcoeSyBWhx99s9gS9c+IBsz29PpYztayg9dSY\/Kwg3d1IK5U1pR\/FJKUEZXG6DpQz54mAFguogpi8Cp+OwBQ5YoOB48Y4dOzieb7Vq1ahJkya2cdFczaAj4+fu3buznKEF6BHyDQxluBaO7QFVsKLqGK398TT1mLuTQMb96pYOuy7sJj5GwgXZouRNaU+5k2r7CFc\/bVRZ88qQrwbOsWPHCB4KCxcu5B14FEQ4a9SoEbubOR3bQZWBUJBTuEtuLhxVnznYfkUKRtjIgyVsJUUE+\/xW9Z3GB4SbvnoWnd8BK\/eqnADrNj6pNpOuVVFlzStHvlbAOf13uwMhmq818tGoH+q\/gLTn8\/e\/NUL+v8zsulFltT+wfnvM2cl6MKzg5hX+NPVfNxtLMxcu4xe0GTao489H3oiZ2X1BtpeOptGhT8ZT3P6NhN81\/bZ4gx4+69Z4b39YiauZnZkahjpCvuLna2bhW\/mKRrqfr2YFwzNi8pOVr9mQ85p8zfRbSriJEuo87ZMT\/H1hWY2dkK8LxLplyxZ2U8OmHA5o4DQbFokcLw4dbKdfGUPvibpXRipGeiu4ZY2iNKGlO+m57OKjEW566qxr5ATICpqem9WZYNfgyup9rK6PGtkBnhLwaMCGWsWKFWnNmjW0fPlyPsk2Y8aMLB8vtgIyWv9ud+FE6\/Pbea5Ixwgbcg0nbmItGFIEiNjJ4g8fTU44v33NNfpt4e4znOxCpraEfANAi0A3AwcOZC8HHKyAF8Szzz7LWS38lZ9\/\/pn+85\/\/ULdu3dhL4uDBg5wJ46677uJ0RHK8+O\/js4Fey7RXcE2nC4ZYRPO15gu7GAXSgK2051DGDFYwXNKwIQe3tH7Jeahm1fKZHsgJzdefOxg2yhKaDwm4wWtXFxfZwXoemtaAvy9OpdWpU4e9HgYNGkTbt2\/n+L52T7jBbe2DDz7gGMDx8fGchl6OF4c2IMEs5NDuEPlXRRNGere0k2\/f78jgAJ9C++F7+7ecEHdjIvveOikn2O2sWL5+kNLSCT355JP0xhtv0JAhQ9iSNaYRQr2OHTsSdF4k1tRiNcACfu2119hKRj64KVOmBDxerMXzXblypd2xi6l6ShwNVRzxaMRoyten6d1vTvNR5ZfrFArumDKO8p48QH\/u+5oI\/\/auJ2yWUdm7KVvzNz0dTWy2oagQRlY5zReW7\/jx49nynT59OpPvkSNHaObMmRxy0l9ISe0wBSxeEDFCRl533XXXRDEL9XixpzPGw5tHk1XnFozRipHRLS1Q0J5AcsLpghWodJ2WbsEfdLti+Vpovtg8+\/777\/lQBWIxgJArVKjg9yoQ9NChQ3nTTQtFicpOHS8OtMDsalGanmrmpmTmo2g1q+zqh\/r7+vvZqJOJ5nutRm6ltVqNlxF7fX2V55DeLe37l+\/jbmv9xTHeY\/97l61b9r3NV5TKjPgqUyxmbS5lBR8z7MTVzA6iIdSBJQsPhsuXL9vKYLFhwwZ65plnqGTJkr4sFYikhCDqCCGZ1ePFQr7Wg+jkl4G\/L6pwL2SvvjDDveFmHF3j\/WEFD3rvM44M1ugfu6jquY1Epw74DjuU6Tc\/Eynrv6jCPWay4Wa9VjPV0GeyMLsURArLNiEhIciWA1dX5RXE0YdysLFofaV2EKKoP4IN61bbLANuh+KK0if\/qEu5b02hIX2sj\/OqNodUWfPKaL5aDjdYunAtg9yAOLwo8HSA10KDBg18AXGcWjyqDIRTz+N0O6otHKefz4n2ogkjvXaLCGHaUV54Jeh9b\/VxIjYPvDcgjKrho8qaV4Z8tdGDBQwPhR49emTKZAHNF7KCV5avyA7WNCWyg3MYhVt2ODbhKbp4LO3vuLf5ilLCHQ9nigxmfJ1H7GB9nAhsyJnp4iI7mM8L5cj33LlzLC8gWaa2wQZXHria4eBFvnz5LGf47t27acSIEfTWW28xWcvxYkvI\/FZQzWoJ\/UncuzLSMIKMgJi3GtkCGS0yWCgny\/Qn5MwCt6uGj1i+AdYCNs\/gYpYjRw6uhcwWCIJ+9913W64gTTtGaMqJEyfSr7\/+6kj2YssbR2kF1RaOijCrjJE\/ogWOwYRhtMLdeEJOH7hdNXyEfC1GE6fUECAHWjAyV+TMmdNq\/AnXwKuhVKlStGLFCibwzz\/\/3JHsxZY3j9IKqi0cFWH2EiNNowUumjWL8Iv4XLNo9USbo3CiaYBxp3A18w32Eh+z5xLy9TPa\/rwe7Hg7IDMxJIfHHnuM4zqAfKdOnepI9mLRfK2Xp2i+WccIluqlo\/vp1OlTlPfiGV+D6b\/s4k1nPbHq78Y+tpcuUd6qdSj9XDpprl9WPXJrzPrP3ZTphFzTZHsR05xYZ+JqZjXqfv6ueT3Azxfl7NmznL04Li6OWrduTdmzZ+fPjceL27VrR9OmTaNevXrRxYsX2b9XI99A2Yv3NcsWYk\/lMkHABQRwDFdfCv71e0KJq58m3ESn6Hoq+K9OpB1r9ve\/073zd4za3+fIqPzJznNMwsiiPKVJ1qKlGe\/j7\/dAx73leHGQs0ILlIOMw\/423NavX0+tWrXK1HLx4sXpiSee4DgP+qhmoWQvDrLLUVNdtVdGFYEVjAKPyrote6j7kuOe5ZAz9k5khyBWEcJDzpo1i93P7Hg7wCrWLF9suDmRvdiJ1yE8spkrjttZEfT39fez8VUN9fT9sjNcbr3CenlayqsxC7ermXF8zcbS7HXeaj7hGm0u2ckh58Q6E9nBzmo1qaOXE7Q\/I4GmMetwoOb15Ku5msnxYonnq58zgcjNDqG4\/YUZjeQLXPXp7M1CVgr5hkicTlwGzff333\/nAxZaenfov3A3Q1B0RCpzsqjyCuLkMznZlrxSW6MpGAXGyAwfLW4wrmxZo1hY09mrsuaVOmSBzBWITgaZAIcsEMcX5cyZM+y1gPi8kjremgycrCHEYo2mYBQ8+WpX2JEirEcguBpCviZ44XAF\/HI3btzIByq0QxaompSUxNko4PXgZLE7EE68DulfZ620XzvP6KTGKprv37KMyA7mKe6zqvkGmtMNJ2zidPad7ypAw1vcblrVrlQkmq8d9jCpg2DqaWlpVLZs2aCJFqfakMUCMXyLFSvGrmZOZS8W8rUeUCe\/DPx9UeFzjKmdEsljFq2ab6BxgxTR5f1tvO7NpAghXzuzPoQ6OFyBnG04IIH07\/DV0xerQxZwR0PaoerVq9O\/\/vUv+uKLLwjuZ\/Xr1+eMGPB+gIaMdPJt2rThTBcodi3fEB4pKi6RV2rrYRSMQpcdzK7USxGLu93OWZWdLKqseaU036wAfPjwYQ6kg1RDcEcDGWPj7rPPPpPjxVkAVojFGjzByFny1VrTpAiksw+Uwsh6hDLXEPINgBhcxUCaR48e9dXKnz8\/NW\/e3G8Otz179nCqIdRDMsw77riDsx2\/\/\/77crxY51+sf503e7XXPtNe74MhFpEdrGnALkaxKDsY5yZ+10dMW9i6aCbJyR9Govlaz0PTGggpiTxskAX0OdtwKq1KlSqczw3FeLwYm3Hwhpg0aRLdeuuttGbNGpo\/fz4VKVKEUlJSKDk5ma9D\/AekFdKyHdvNXhzoyKLdY4+4v9WR0GAy4QZz3FNf1+xn42foK6Qer\/rjDyutX3amVySPWaA5peoccmvMjMeUkU0ZWZXtHC82YiXHiy2sXriVIUaD5mpmZ6HB8sUpOBA3rtMOWoCw4TMsx4vtoHhtnWAs39DuEPlXCUbuyA7GVoPJphyoRyI7+EEHWi38fO+55x62YO0WLQh7x44dqWLFigS3tUWLFnFshw8\/\/JBJGQVxgdu2bUuVKl2NsqTKQNh9znDXE2KxRlwwCg\/5aneZs+EIdZ+zkzfizIK3W42YKmteuQ03eD1AEkDetptvvtkXy8HK2wGA79u3j70a4HIGVzVovgiug0wWcrxYjhfrF6VdtyVcY+WTHQz5iuYbOlHrsYMVPGn5To6Ylly2AI1vWYkunzl8TTwSs7GD9ACu8LooR77hBsTut2Ak+4zqCcTfz3LIQg5ZBPpysjOH\/H1R4XO3fLP1eeS0Axqy4RYii0I++Oijj\/hIsb7gtBvi8lauXDnowxeRoP+ECJfrlwVj1bneGUVvIBiFbs06NaTBHFO2a3A51Td\/7Shn+eKEG4KiI4zkQw89REglD9mgRIkS7K+LaP4ILelUUWUgnHoep9sRYrFGVDDynnzRg0B55PQ9VGXNK0e+yFwBdzGEkMyTJw9jhs\/gw6tlq8BBCqeK3YEQ2cEacbt6pqqvsPpXY6tXVwkpaW8PATi6KTuYjZk+eLtRikB\/RPP1s5ZBtKNHj2ZXM8TiRcEm3KhRo6hLly40Y8YMPsVmVuDhgKPJsJ7Lly\/PqeaRfNOJ1PFCvkK+Vhtv1giZB6wxfhlZ\/W7Vj2As8Wj8wtSeSS9FDEwpQMgjJ+RrMUsXLFhAc+bMYXczxO\/dvHkzNWjQgPO3HTp0yFR2AGnD06FPnz7s4QA3s71791K9evUkdbwdVvBTJ5iFnIXbRPSlgpEasoNZL\/TB21vWKEoTWlZSxr1UOdlBAxDuYjg4geDq0HthweJnHJgwCysJ8n3ppZeoZ8+efDJO27TDcWNoxfpDFriH9rtd2SGi2SELnRdisQZPMFKXfLWe6YO3n53eWlzN\/A0ZJjOkAhy4QIGM8OOPP9Kbb77pkyLMrl2xYgWTL+rjeDJ0YjuxHdAW4kEEKpF8VBXPJceLM4+u6kfC5XjxtavR7pgFOj5fs\/s4yrilIZmlMLL+mnW2hnKWL7JZ4BTaXXfdRd9++y0HyPnll1+oWrVqdN999\/me3hjbAWnlhw4dynowLGUc0kB8B7io1a5dO2BsBzsO16L5Wk+8aNQP9fqrldZqjZBovlYYObHOrDZL73\/0CUr7\/hurrrj+d+XIF6SqxXZAVgvIAiBTfNatWze\/Uc0Qu3fp0qVMvpAlIFuAjEHaEtsh9Hkkr9TW2AlG6ssO+h6qIjUqR744ZAF5oVOnTpzRAjF5sfGGWL29e\/f2m8MNdUeMGMHB0osWLcpWL\/41adKEPvjgA4ntYM0hpjWEWKyBE4yEfK1nybU1lCNfdBEuY6tWrSJICdhEg9cCNsi6du3q93QbNtWQvWLMmDGs+eLbrX\/\/\/hLb4a8xN3sVM3ul1j7D\/\/ChDMZKiFXZwQ2MjFhavUq77Xesnyt2ftZLNMH41YZDdgimP6GQqt1rlCRffee1jBR58+b1pZK3+3B26mnxfO3UlTqCgCAQHQjY2edx+0mVIV8cpIBea8zdpgFgJ6qZ22BJ+4KAICAIOIWAUuQLTffixYucLgjeDvpg6kh+CZ9dHLqQIggIAoJApCOgDPkCSByiQEAd+PguXLiQ0wY1bdqU3c1y5swZ6VhL\/wUBQUAQ8CGgFPnqxwUbaDhKvHjxYlqyZAl7PODosBZsR8ZQEBAEBIFIRkBZ8tVbwUK+kTzFpO+CgCBghoBS5Cuyg0xSQUAQiBUElCFfeDtgw+3EiRP09NNPy4ZbrMxAeU5BIEYRUIp8xdUsRmehPLYgEIMIKEO+MYi9PLIgIAjEMAJCvjE8+PLogoAg4B0CUUW+Whqh3377jVJSUjgVESKaBUoj5B30cmdBQBCIZQSihnyPHDlCI0eOpH79+lGRIkU4kwUC7NSoUSNgGqFYHnx5dkFAEPAOAc\/JFyEkL1y4wAjgFFuohyhg9SL+74svvshHkJGCaNasWVSxYkUOqO4vjZB30MudBQFBIJYR8IR84c+7bt06TvNz\/PhxypcvH49BRkYGx29A2viaNWsGFccBAXlg+Q4YMIBuvPFGTsD58ccfU1JSEtWqVYseeOABvsfatWv53rCQpQgCgkDkIJCeOtNWZy8d3R+w3qVjaVS4+wxbbblZKezkizRB06ZNI0Qpq1OnzjWWLv6+bNky2rRpE2cjtlu0eL5IMY8A7A0bNuSU83bSCNm9h9QTBAQB+wjUveEcVy6S6xIVyXmRfy6a82peRu1z42f2WzeveeRCnK0m\/rnoan+8LGEnX1i9+GeWgVgPBOoEE8HMGPcX5I20QomJiZxyXi875MqVixo3bsy3CyYQtpcD5dW9BR9r5GMRI80Kzdi+hi4eS6NLR9MIFqWxxBVOpAMHD1Di7cn8pxyFE31V8Le4G0tR3pT21iA7WEOV8Qo7+WoY+ovfGx8fTw899BA9\/vjjPjnCDu6HDx+mYcOGseyQkJDAEgQSZ0KCmD17dpbTCDkRYR\/PYZWEMZiUNE5mjjBmSkBf9dkR7IyBk\/3xh5XWr6z0R9+21o5qmSNU6g+IFiSbvmUF0akDBvJMpIzcN1DePHn5Vd5sfod7zKyyfsR8JovLly9zbjVYodBjIRtAo0WBJLFx40YaNGiQnTXGdXA9LN0JEyZwynlYum3btmWrF65mkydP5jrQk+vVq+fLimH3W1DI13oohHydw8gr8jUjWliouZNS6Fy9wfyFbPzysiI7IV\/zeeGp5TtlyhTq0aMH++KiwD8Xm3BPPvkkeyogf5vbxS75ut0PVdsPxhJX9Rnc7lekYgSZ4Pz2VLZqIR1kbE+lq1JAIhV\/ebVjsKmGjypr3jPy1WcphqWLgkDqY8eO5RTxIN8hQ4Y4NgH8NaTKQLj+oCHeQLWFE+JjuHpZpGHks25TZzLZokB3zZ1Um+KTUhzHSjV8VFnznpEvRhi+uSBYeCRgcy09PZ1Tv+PABCSCZs2aOT4RjA3aHQiRHayHQmQH5zByWnYA4aanzvJZt5AR9BqtsedmY2mVAVsvR+i1X5EdFJMd0B0Q7P79+1mkL1WqFOu\/JUqUYM3WyhvCeprbq2GXfO21Fn21VLNaVERYRYwgKcADAYQL4tV02\/ik2mH3LlANH1XWvKeW74IFCzhFEDbf+vbtS2PGjKH27dtTcvJVt5RwFFUGIhzPGso9VFs4oTyD29eohBFIN331VcJFgX6bN6Vd2AlXj7lK+KBfqqx5z8j37NmzNHr0aGrTpg3rvJAfcMwYG24gYu3Um9sLR5WBcPs5Q21ftYUT6nO4eZ3XGGkbZ3pZARpu3vvb+TRdN5\/fqm2v8TH2T5U17xn5ws8X3g6tW7emN998k8kXsR3wMzJaFCxY0GpMHfm73YEQzdcabtF8ncPISvO9OU82+iV1PuXa\/qlPx71U6k4q3qAHHb6+lM8lzKpH0ThmVq5vMe\/nC713+vTphOPEW7dupa5du9LKlSsJhyy6dOminOYr5Gu1jP8+QGKsGcmbN3gW\/eaR\/uCJ8XNrhOxj5I98IScc+\/oTom\/\/S5RwE+WtWofO3ZhEpZu9cE0\/neyPHgc7P8uGmzX6nlm+6BpCPi5fvpyg\/SKoTv369flkW+7cua177lBU5vv9AAAXZUlEQVQNu5avQ7eLuGZUe2VUEUC3MTr10St0fgf8ca\/64WqeCipiYdYnt\/EJFgdV1nzYyRcxG86cOcOeDmYlW7ZsHNksmLgOwYKvr6\/KQGTlGdy8VrWF4+azhtq2kxjpDz74Ns3+IlwvPBVCxUR\/nZP4ONEfVdZ82MlXH9MBm24IKYngNyjIXNy0aVPOXhxqXF\/j4IDkA2WysDsQIjtYT\/to1A\/dlB0yEe0vu4j2rmeQYd1Cv00oXdl38MFKxzSTQ6xHzL4MYkdq8IcVPteOJVv1yYl1ZoVVzGu+kBwmTZrEByng24uCmLzz589n\/Rc+v06UnTt3BsxkYZd8nehLJLahmtWiIoaBMNIIFv02iwCmnTCDlBCplq3VmKg2h1RZ82G3fLWBOnnyJAe7gWeDPrYD3M6w4eaUt8O8efNY4vCXyUKVgbCawF79XbWF4xUOZveFLIDA3ad+2sZRvRAfAcUsvKKeZFEnWonWDCfV5pAqa94z8oVPL0JA4nUEUcZQVq1aRbt27eJUQE5ZvggtiTxu\/jJZqDIQKpGKvi+qLRxsPvkrZvFkzepqJGn8G0hTK3bbYong0iWKL17OF6sWnyU0dz8uiapzxtgv1eaQKmveM\/LFACGK2dy5c1mThQyBOL4dOnRw9IAFyBcpibRTc8Y0QvuaZYuUORwz\/TwUVzTkZz2c3fraQ3FFTNs\/\/Nd9N1bqzPMRBkDxfHE8T8sXzU\/Yo8Dhn3olznHYU8hk+v\/tdFq7xljX+Hmg34339fe7k\/1BW\/o+Wf2s7xOu1YJnWfXJHz7G+wf63axv2mfQe1H27dtn1RXX\/x528sWkRgCdQoUKBXw4bMRZ1bGDDsgdJauZLJzYCEA\/vPAZ1d\/X38\/GTQrUi5Vg6iOXpdHpU6eoQEICz5WfT54nRN3Dpu8vJzPofEYG5Y6Pp33HztGhs3+nwTHOv5IF4\/mjkgnxdHPBeCpZ8KrLZL+6f2dvsLspaXXIIpC\/cTCWpt3+2JlD\/ua3NpfsrFcn1plsuPlBGq9oixcvps2bN1Pz5s2pbNmyPr9eHLjYsWMHLVq0iKpVq0ZNmjSxM14B62DDzYlMFlnuSIQ2EMxCjtBHzHK3gdH647np55MZPvIGaf98KsP3mXYTEHTNsgVoQstKWb5vpDSg2hyKednh2LFjhM2whQsXsjWIcsstt1CjRo3Y3cypDTfN1SyrmSwiZaI73U\/VFo7Tz+dEe8FgBCt73Y+naO3e075bg5Bb1iiWyUJ2ol+qtBEMPuHoc8yTbzhAtnMPuwPhxOuQv9cyFXw0Y1l2ML5S+3vFtpKM7Mw34zyas+EIW8cjl101QDQiTi56kWpWLe9r0upVWoU55G9+43Px8712doRd87UzQcNZxy75hrNPKt1LNatFJWy0vjiFEUgYZDxnw2Em5GixiJ3Cx6mxV2XNC\/mWKaPEzqdTE8vpdlRbOE4\/nxPtuYGRRsSwiEHCmwfe60RXPWnDDXyy8iBCvllBz8FrVRkIBx\/J0aZUWziOPpxDjbmJkZ6Ek8sWoPEtKzEZR1JxE59QcFBlzXtq+SLOA8JKpqWlcTwHeCY89thjSkY1E83XeppHo9uSPx3TbY3ViOW6LXuo+5LjLEdMaVKUmiZXsnRbtB4xie1gByO36nhGvnA5e+utt9jVbM2aNZwm\/vPPP2dHdhXj+QYagEA+mfrF6\/QgBkN2+ntbWSJmmzt2+u50f\/xtcNnpi5O4W220WeFpt7926sFbAlJEv7qlqXmFPzP5Yhv7aae9YOo4Pb7B3NtsPP2tO6vxiHnLV4vt0LZtW18mC5wocjq2g9UAqzIQVv306u9WE9mrfql033BjhE257nN20sm371cJBr99CTc+VqCosuY9s3xxgmjixIns06vlcMMhC\/w8cOBAR48YBxoMuwMhsoPVlI7OV1hVZAfj28jBywnUcOKmgBaw9YhF55hZueXFfEhJTAzEWYD0cPr0abrzzjtpw4YN9PrrryuZvVjI13opB\/Na6k\/asFo46EU4fUZVJV9gMH\/tTnpmwRF6uU4h6lWvsqQR+umna47Em0lGQr5\/rWVYwHv27OFAJuXKlXMknoM1Tfxdw67lG0yb0VRXtVdGFbH1GqOCfVbT4m63U3K5AirC4\/tSUKVzqqx5z2QHDAQsXWyyIX38gAED2N+2T58+jsR0sDvQqgyE3f6Gu57XxBLu5w3lfl5j1HDCJj6uDF9gFd3QvMbHOKaqrHnPyBcW7xtvvEHPPPMMh6rbvXs3NWzYkMaNG0fPPfdcSJovyBwxghECMCUlhXr16sWB2iWNUGZtL9Crvfaarb3eB7NwRHawpm67GPnbyfcng\/Reepqjr33+\/L1BWZp2+6O\/r52f9a\/74ZaKrKSrmJcd4O0wdepUJsj333+fqlevThUqVAjZ2+HIkSOE2L39+vWjIkWK0EcffcRSBgKp4z6vvPIKITnnyy+\/zJZ21apVeaXY\/RYUzdc5YrGzeP2RTLgXssqar57gsucvRtVe+yrTBpz1iMmGmx2M3KrjmeWLTBYgy99\/\/51AnMOHD2cZ4rvvvgspk4UmYSALBjIfQ0eeNWsWVaxYkXLkyCFphEKcQcFYviHeIuIvUwUjzQVNNf1XFXy0iWbX4HJ7YnpGvngwyAPbtm1jS7Vw4cK0evVqql27dkiSA6QLkDm04xtvvJHmzJlDH3\/8MSUlJVGtWrUCphFCX1auXOk21hHZfqDMAhH5QC50WiWM4P1wOP0SLWl3kwtPGlqTKuET05ks9MOHU27IaoGYuyjw80XmCaQSSvgrq4C\/4YZs0bFjR9qyZQt17tyZ+vbtS1988QWNGjWKrWnoxzi+DKsXhO4vjZDdb0GRHawXXjTqh5EiO2jHnTX5oWWNorYCtkfjmInma7FWQbwIcP7pp5\/ykeICBQowaT711FPUqlUriouLs17tuhpoD9fnzZuXtd1NmzbR0qVLKTExkbJnzy5phP7ygfSntxonrKatBvPKGI0LOdLIVyPhO99Js+V+Fo1jJuRrQZ2wSt955x12LUPW4kqVKrGVCtczEHCw5Hv48GH2dIDsAKsZEgQsXkgQkkYoqO+xTJWDId\/Q7xLZV6qIkUruZ6rhY\/dt1+1Z6ZnmqydfBNa54YYb2AMBrmadOnUKOo0QpAtYuhMmTOBU3kiYibgRsHrhaiZphEKbSqotnNCewt2rVMUIBIyyuPvt7gJg0bpq+MQ8+YIs3333Xd50q1OnDr333nt066230q5du\/iIMTLHhqPYHQjRfK1HIxpfYSNVdoD8gPCTmvuZPoOyfiSjccxEdrBeq3TlyhXWaa+\/\/nrauHEjx\/N95JFH2PMhXEXI1\/w8vGi+V3GJZPJF39f+eJoD8CBbMjbhjEXIN1xMc+19PJMd0BXIA7B0EVhHKwgrWaVKFcL\/4Sh2yTccfVHxHqq9MgpGwSPgtf+vanNIlTXvGfni9NnQoUPZ0wFgaCV\/\/vzUvHlz5WSH4Kd8dFyh2sJREdVIwQgBeBCE3Z8E4Ra2quET8+SrBVPv3bs3x1\/wqtgdCNF8rUcoGl9hI1120I+algVDH4Q9GsdMNF+LtQrLd9KkSdSsWTMqUaKE9cp2qYaQr2i+eoLVppnVAg53Djen+qNpwIh+Nr5FJSqR\/ZRpfGQzUjbrgxE7rY7+f9QJZwxmK6xiNrAOopkh6M2ZM2fo+PHjlJqaSvfffz+7mqGI7ODSt0yIzar2yhjiY7h6WaRhBC+IHnN2chjKcMgQquFj1+ByddIQUdg1X3g4gHi1I8XGB8TpNBAwguOEo6gyEOF41lDuodrCCeUZ3L4mUjHSZAhYwYgF7FZRDR9V1nzYyRcDjChmyN+GoDqIuWAnloNbE8PuQIjmaz0C0agfRpPmazaCSEm\/9kgOzoiMolnCIjtYz\/es1gg7+ULrffXVVzl+71133cWnzyA\/IDBOsEeKs\/rwuF7IVzTfWNJ8jWtGT7KwhNf9eIrliOL54qjNvTf\/RciJ\/L9ovk4wzt9thJ18jV4O+B0ZLV566aWgQknCR3j8+PFUv359Kl++PD+RlskCBI\/PkAW5UKFCjmSycBb2yGlNtVdGFZGLNoygCcM3WCNiYA5pomRCPNUsl0A1yxYIKl+cavjYNbjcnmuekK+WwSJ37twE8kWWiSFDhtiO57B+\/XqO17t161aaMmUKEy38hdEOAvUUL16cFi1aRHv37qV69eo5ksnC7YFQtX3VFo6KOMUCRnqrWD8GelIO9gizV2Mp5NurF4VKvsj3dvDgQfaaeP75533kC+u5Z8+enI5I86jA5h029xBoBwXxglG03+0ORKxqvnbxMXt11xZXJOuH+ucyc6OCC5UbGBkxs3Kfctv1zTi+SFu\/97fcPMQrvj9MuePjWa4wK9f9fpxuuukmtpxvLhjPVUoWvHqtkbCdWGdWWMWsq5k+CLrZQCGy2bRp0yytYAReRwCedu3a+WSHFStWMPlCdkA7kCWQHw553B544AG+3dq1a2ndunWc6w1Ff7rOq29iua8gEK0IZNzymO\/Rrlx\/A125vpDf353AAERvpxyf3MxONVfrhF12COVpEJsXEdD0xGwkX6QqwXHlwYMH86EN+A8jVKVVJotQ+iPXCAKCQPgRwAGRX05lBLwx9Go7JdxHrM36FBHka9ZxI\/lCB0Y8X5AvvCaOHTvGZFytWjU+vqyXHRC0p3HjxnbGSOoIAoKAIOAKAlFDvmlpaTRixAhODV+0aFG2evGvSZMm9MEHH3CGCxRIFQiyjswZZgWkDVc4ZMZ47rnn6J\/\/\/KcrwEdyo0ePHmWMgDl8tBs1asSpm6RkRmDevHlUqlQpuueeewQaAwI46Yp8iwgli1yMmEOxVqKGfLGphgSaY8aMYc0XWm7\/\/v3Z8yFQJgvjgC9cuJBKlizJKednzJjBKY3CFdg9UiYf8u5hAwWZoZE5BFHoihUrFindD0s\/8SWOzeBu3boJ+ZogDsMIHkpIpACPJayzfPnyhWVsVLlJxJKvWwBisw\/uaSAT\/c9u3S8S24ULHwLeIwg+yBdvFzfffNUhX8pVBJYsWcJvT4hNLZbvtbNi1qxZ\/FYJN1Fks4HnU7hCCqgyR4V8dSNx+fJlmj59Oj3++OPsbQFvibp16\/q8KVQZNBX6gUMueK3G6cTu3bt7cjpRBRzM+gCrd+XKlfx2gP0HId9rUYK1iyQKyF7TtGlTNnhiTbqKGfLFIGuuaJgKIA98+3744YdswQ0aNIhd0iA1PProo2zZgYi1n1Vd6E72yy5GsFRwKrFy5cq8cenFsXAnn9tuW3bxgXQFbE6cOMFNxxL52sVo7NixHM0Q0hV+hnQVa29PUU++iKCGAxcLFiyglJQUn3+vtiGHgxmw3rARB99fHFGG5ovXIWzUtW\/fnl+JorkEi9GWLVtYB69du3Y0w+J7tmDwQXKA\/\/73v3wICD7t8LTBFzus4GguwWCEdfbdd9\/xvgzIF3G9YeQI+UbZDIGov2PHDtq+fTuTrHa4Ar7DNWvW5Khq2KwbPnw41apVi0\/HYScfkddixdshWIxwSAWbmzlz5uRNEniY2A2WHYnTK1h8MKdQ4P4YK5ZvsBhhQxtrcP\/+\/dS6dWt67LHHRHaIxMVhp89YCLB2Qb7wEYYPMAb9tttu48tx7Bia0xNPPGGnuaisIxgFHlbBx3raC0bWGGk1ol520B7UOClee+01PpoMS1fI9ypKglFw5Ctz6Fq8ZA4J+V6DgH5S6GUGTXbAKxACbmDTLVaLYGSffGUOmWMlc8g+e8Sk5Qt4IEF8++23hA0SaMEgX0gSOB0Xq0W\/cASjwFad4GNNvoJRYCaJWfLVXM3gq4odaXg9xLLVa5Qd8LtglHnxGL+cBB\/rLyjByD8Bxwz5xqo1K88tCAgCaiIg5KvmuEivBAFBIMoREPKN8gGWxxMEBAE1ERDyVXNcpFeCgCAQ5QgI+Ub5AMvjCQKCgJoICPmqOS7SK0FAEIhyBIR8o3yA5fEEAUFATQSEfNUcF+mVICAIRDkCQr5RPsDyeIKAIKAmAkK+ao6L9EoQEASiHAEh3ygfYC8eD0HEO3fuTL\/\/\/jvH\/NXK4MGD6fbbb\/eiS47dEzFBkGvsvvvuC7rNPXv20LJly6hHjx58LY4rHzhwgNPo6Et6ejqHOG3ZsqUkbw0a5ci5QMg3csYqYnoK8n3llVdoyJAhnAvPrFy5coXjJ0dS3i7kZps9ezZnJNaymyC6Gf7ZSf6oJ19cg5RVIHFkTTEWkDzSq9evXz9ixl06GhwCQr7B4SW1bSDgj3wRxB4xcBHICJl9QdBfffUVvffeexzEB0kUkbYpR44cvvx6SL+TmJhIjzzyCGc9QEHAe7SF1E+IyYwsGiBF5E5DPrk2bdpQw4YNOZ0PspjgXsi43KhRI+ratStlZGTQuHHjKDU1lfAl8OKLL9I333zDmU2QGgmkh+wcHTt2pEqVKvmeGBmJQbIgRARkQttpaWn07LPPcv+HDRtGf\/zxB+f\/wxcP0uQgLRV+Rrn77rv5y6hnz56cNh1JJDt16kSffvopzZw5k+u0aNGC2rZty8klR48ezZH2Yi2luo0pFhVVhHyjYhjVegiQL4gLud60ghxdkB2QQQQpw5EwcfPmzRzaE6\/h2bNnZwItVKgQv2pr4T5Bss8\/\/zx16NDBL\/nCosSrOtpEBmok9wRZ\/\/DDD7R27VpOEQWSxb2RafnLL79ksgTRg5hRH\/0DAfft25fQHpKrgjQ1C\/fChQvcDjJbI\/sJyFdrG9b75MmTqVWrVlSsWDFavnw5ywn4MsE9ETGvRIkSnEsQZA1C3bRpE23bto0tX3z5oE6uXLlo6tSp9PDDD3N9\/f3UGmHpjRMICPk6gaK0kQmBQJavZq3iVRvZoWEJ6wu0YlivsBK1XGjQP2H9mlm+SAWFv\/\/73\/\/O1A6ISytaaqjx48dzxlxkrIaeqqWQggQASxPX9OnTh8kzISGBCVkreksbfQf5alY4\/gf5I08gCB\/kXq5cObaiV61aRf3792d5BX+D5gvLF8lZq1atys8FTDSJAc+Ne6MgxjTaiKXsx7G0lIR8Y2m0w\/SsdskXZIhXdI0cYV3CcoUkoCU3RZdB0rfeeqsp+YJEQbyQFDSSAlHCkoYMoSdI3A9kBrLWUkiBeEGcmtVZqlQp3gjr0qVLpmy6gcgXWjAsXFi61atXp927dxOSjPoj36eeeoomTJjAEggkBfTh119\/ZYkC8sPAgQN5Y1LIN0wT1qPbCPl6BHw039Yu+eK1HQQJ7Rc6L8gRlh\/SkMOKhBWqlx1OnDjBJAUJAqnZX3jhBSY9pCGHVYz68LAYMWIEgeDwuZF869atS6tXr6b8+fOzTAFrdNq0aSyJQBKAFQoCRYYTWOBagUwxatQoatCggU920NqGTIHNswEDBvA1Y8aM4ctglaN\/kC+KFy\/OsgqyYkOPhtaN5\/j+++9p8eLFLHfgCwNfNNCwkUlbf79oni+x+mxCvrE68i4+t13yBaFBBzVuNl28eJEmTpxIS5cuJf2GGzawYBWCgEuWLEmoBz0YdVB\/xYoV\/HoPXRdWKNo2I1\/oyrAqt27dypt\/aBMSAF79QeAgRbNXfWy4oUCO0MsO2GR75513CH9HSnRotrg3NuAOHTrEm3cge2RKwf3QX\/T\/zjvvZO0ZpAxrHOQLDbhXr178bCBxbOZpMoSLQyZNe4CAkK8HoMstg0MARAc5wG3tE1YpCA8bYmaEB3kBG2IgR2wKulmwEYkvscaNG7t5G2nbQwSEfD0EX25tD4FwkC\/0VsgGeP2vU6eO34793\/\/9H\/8NHhtuFWjQ77\/\/PrvMuU3ybj2DtGuNgJCvNUZSQxAQBAQBxxEQ8nUcUmlQEBAEBAFrBIR8rTGSGoKAICAIOI6AkK\/jkEqDgoAgIAhYIyDka42R1BAEBAFBwHEEhHwdh1QaFAQEAUHAGgEhX2uMpIYgIAgIAo4jIOTrOKTSoCAgCAgC1ggI+VpjJDUEAUFAEHAcASFfxyGVBgUBQUAQsEbg\/wGKAs8bsGxnOAAAAABJRU5ErkJggg==","height":211,"width":351}}
%---
%[output:060d3403]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:0a19f78c]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:4e9788d7]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:4b93a0be]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:2118ab0d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAV8AAADTCAYAAAAmsBCWAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQe0FUXyfmPOARUQwQhmUBDU45pdRXEFMYAiK1kFA8qCwopiAETBhOKKKIqriCyGhUVxTbjGXQkGEBUxIqBgRsX1p\/zP1\/7rbr9+3TPdE+7M3Ftzzjv3vXd7OnzV\/U1NdVV1nTVr1qwRfDECjAAjwAiUFYE6TL5lxZsbYwQYAUZAIsDkyxOBEWAEGIEMEGDyzQB0bpIRYAQYASZfngOMACPACGSAAJNvBqBzk4wAI8AIMPnyHGAEGAFGIAMEmHwzAJ2bZAQYAUaAyZfnACPACDACGSDA5JsB6NwkI8AIMAJMvjwHGAFGgBHIAAEm3wxA5yYZAUaAEahY8r322mvFuHHjrBI+4YQTxMiRI8WGG26Y2Cx48MEHxeDBg8U+++wj7rrrLlG3bt3IdS9atEh0795dLF26VNZh6q8+xkmTJokDDzwwcptJ3JgkBi79SaO9V155RXTu3LlG80nI1GU8qtx95Ek4oI2GDRuKu+++WzRt2rRGkzRfosz9qP1yGfOPP\/4oBg0aJKZPny5GjRolbrzxxtK8N93vg4tL+1mVqVryBeBJL6gkiUAnX72v6oSlyZOHSWnCIM2FmyTmwDHsoX3NNdeITp06pbZeo2BlmgumfuaVfGnM22yzjRgyZIjo169fIPkC\/ChyiDP+NARe8eRresqrmk0UIdoEkSQR6OSLNlVyVSfsihUr5GTNA\/masIlCKK6TPS3M1Xnx5Zdfip49e4rXX3\/dqlW69jeNciZNPehNKW+aL5EiMG\/ZsmXpjU+fz+pDJorixOSbxuwz1BkEtEoGOvnqmo\/tFU4nx7PPPlvsuOOORrODvjhcJo5aP+qGCUXtK9WJ1+NZs2bVIl\/TgkQ9l1xySQ209HKmcXzxxRc1FsRzzz1Xw6Sj9ksnwyeffFJiol60+MeMGSPr0cmA6tCxTxvzICJXcdJJwXXORMHaxYxE7WNebb\/99vL13TRv9TUBMqOHCmT44Ycf1pCr6WFPD3l9DuiYmJQHE+nTgw0KBEwluMjcZlIm1LGqpj29PXX86sNTnYdUv2tfk6auqtR8VfsYCcD06qaCbSI+mzBUcrW9xtpInepUJ8Rll10mbcj77bdfyU5N9dJ3quarjk\/vo7oATAStlqdxqORrGzPhkxb5uvYVdvaomJvmRdCCS2PO6A+6MPJViQUPzsMOO6xkr7YpFjQHVPINk6uJoPR7aC0FyUpXPKgs9WnJkiVW8lXHqs7joPmOPu26666lh4xOvvhbt+9TGRclKQ4hVzz5BoFj0tjU13t1cRFZbrXVViVBqsJRJxz9\/9133621ENQ6g17\/1Ml+6623ivHjx8uhgISxSYgNio8\/\/lhqsgMGDChpvmibNi9MRBtlHCohqGNW+0htTZs2rZb2bzM72N5OdM03C8zVeWN6Y8D3YQ\/xuFi7mJF0jVwlGn1+BWm+Nk3R9AAOmwP0RmMrZ1J4grRQfQ3b6lXlZNKQTfPNVC5NM5k6lqomXxIiAKHXL32h6SaKHXbYoUSottdPqpdeuW1P+yDtV58AeM3D6yRezYiM8IrZo0cPce6554bafPVFqj7xw8ahkq\/6wDI9SNIgX7RP2klYX+NgDkzCNGwav65xquacOHPGV\/MNIhl9fgWRrz7v9QcL5hyZA8LmgMmDyGTqU\/ctyITgomFDTtQHkleQiYrmjKvNN8gsGUfT1e+tePIN23DDpDvppJOMEwtg2V7rTMSpv3JDWw1yd1O1bF0wOvl+9NFHUqPEpKMHgN53lZjCXsWIfF3GEUQI+oROk3xd+hoHc10GNvOFvjGkv97HmTM+5Gt7ANg2lIPIV3+o6Q9rlXxtD0BXk5ZuolKJP0jrNL2Jzp07t9aegi5Has9GvkEP3CQ35Jl8hRD6rqnq3hK2kGhTLW0i0CchkaW6IYZFYFoUKmnQeGxk7jKOrMmXFphLX5MkX3Wx6CaWXr16iT59+sg3jiTnjA\/5hmnp6L9KiOUiX3oAo30iVvVBAbzatWsnzWNz5syp4ZMc9sqvP1jQhr6h60O+qpJi62taroVVqfnq5AvH7oEDB0pXoiTNDkQESbj2kC0PEwvmBpq0+FvdHbZpKEmZHcK0niiar26W0W2+PmaHOJjbdtKBsW5iufjii8V5552X+JxxJd+wzT4iIPWBFebtoJKMLgN9nqkbgWq9Ki7qQ8lmitHXRlTyddkccx0\/mx1iGleC7DsqERHZprF5om64qaQVtMhp2PokVDfSUMa0EaJrwjT59YWq7wCHbRympfmaMDf51EbdcPPFXO2Prs2Wa864kq\/uiqi7EJrG4rrhptZtm2cu5GvaACN7Lbm16Tj7mh3Uh4JpA9314RPUV9Z8PcnYZq9Tq1EFE6ZJJO1qpm4amIZmmoTqmGiyhJG0qW4ipSC7cBDB2xYewrXDNF\/1VRhvGjY3H5RT5RP2iu3i3heGedgcUDGBS1tYeZs3jUkmLiSn3hfmFmciUP2twMXVzOSFEPb2Q94OtiULMx82j8m3Vw2Ddt1wU8kyaB7bSBV9wzh0f2WbycKTfpyKV7zZwYaCzRRgIm2Ty49OBlhoZH+yeTeYXgVt\/TORr2kjxVROdyrHBOzdu3fJoyNoY8QlyMKXfDFGFdew9slXVbfxpo05ycJG9LbNl6hzxhdr6p9LpJfpbYeIxuTnqwdZuHgPUH90jRr\/J3dHetgOHTpUXHnllZJ0t9tuO\/Hpp5\/WMvGhrAv5mtaj6b6wwA98r79RAhu1ry7mDCemNRSqWPKNCgjf9z+iTHPiMc6\/IeBigkoLK30TLK3X67T6X\/R6mXyLLsGI\/TcFSMA\/M8yWGLG5qr4tr1gz+WY7LRMn319\/\/VUsXrxYfPbZZ2KttdaSae2QrSjt6\/\/+7\/+kvXHs2LHigw8+EK1bt5aRX61atRJ16tRJu\/lC1h9kFw8Lfy7kgDPsdB6xZvLNcEIIISKTL8ju8ccfl0brDTbYQPTt21fsscceYtiwYWLKlCmlUa277roC7iddu3YV66yzTmqjfeyxx6StBunoDj74YPHII4\/Ivt1xxx2iRYsWqbXLFTMCjAAjEAWByOQLsrvwwgulVgny3XTTTaVD93333Se6desmTj75ZPHVV1+JqVOniocfflhcf\/31om3btlH6GHrPt99+K30uoe3iE3366aefBJLObL755jL\/QZrEH9rBhAt8\/fXXTjUmWW6LLbYwtkn\/t33v1FEuxAhUIQKRyJd2UmFiGD58uNhoo43ELbfcIn8uuuiiEgECT2jII0aMEN988438XH\/99ROHef78+eKcc86R7ata7uTJk8UDDzwg7rzzTqvpY+LEiYn3x5X09Iaj3pf4AGJUqJIwfsfP6tWrZY34xMNQH2eDBg0EfujS\/47RHb6VEcgtApHIl2xFp512WimrPwIKEGkFDVdPgwc\/PBBh3KN1bCg+88wz0o3lnnvuETvttFOpGFyGYPc1HalChXAfX9kgADLGD5ExPul\/1CM8vOkhjt9\/+eWXGn\/jf1Qmm1Fwq0VE4P3338+824mRL+3ojh49uuzkayP3qOSbxit0lDpd74E2qV\/Y8MS1fPlySWj4xE9SF0xN6kV\/q596GZ+2iZTpM+ytgDRn0prr168vm6O\/dW165513FnlYgD6YcNlkEMiL7KuefPMiCH1awWND1eLpeyJQlUwRKUZEmyTJ6oSGNmA22n333Uvd1c0F6jjwHcax5ZZbSu2WflAGWdpwIczU9SLzBWGg1udaB\/UXDzYkSQJJq8S87777ulaVWjmb7OM2GLde3\/tdy7uUCyrj+11e1nxFkO+MGTPkiacwLzRu3Lg0R6H5YrNNN0eokzgvgiDypE\/YsfE6DQ3WRLhxFqKqDaIenYCCbK4uC0Xtm0t5kOjbb79dsg2DmH1JmUwWUQjZhKWKUZgWHUcWpntdMIvSZtx6fe93Le9SzpdgCR\/TfXlZ87HI98QTT5QeDrgwyAsuuEDaXnEInnohpPDRRx9NzeY7b948cf7554vbb79d7L333qWmYWeGVwZOgthss82M87UcgtDJE3+rZoGktFXTq7dKpEXbyCJTw2uvvSZl50PK0JJh9qBP5ECmB1maeKOfNlNHFMLke5JHoBxr3qXXsciXXnddGkozVBUubTjNoU2bNuLMM8+UrmbkkdGkSZMa3hd6X+MIIitSxRiAp77QXeRQCWVAytCM8elDyGRqgI0cxAyzA71pmMw4RPpxMfO1R8dtj+8PRiDOmk8S20jkCx\/aN954Q\/rSul6wFTZv3jwVVzP0Ab7ESA4CMwOICS5mSCSCIItddtnF2k1dEKZXfNJUk379J\/LEJ9kayQRA3yVtg3R5xQuSqe\/9ruVdygWVefPNN6WvOUgZbo2uxEnucNCMQcZEyCoGRMwqQSetRVN7IGrVHq0+YNV54rru1HIuGBdR9r4miUKTbxTBx70HiwtJz2HHhWZ7zDHHyMxJyJCEC6+XV111lXjooYfEzz\/\/LM0MyHDfsWNHa3gxFhPMJoiIS+pVVF1E6sLxsavGxYrv\/w0Bsv8SIZPGHIaPCyGb6ig3SevzS\/27aCamMJkk+X2hyRfBFdAu1qxZ44wJCBOve8j34Hu988474qyzzpIkieNbVq5cKTfYELV28803y930qOHFRxxxhHN3TLv\/TKrO8OWiINmRZ82aJTVMV7OFSsj0u0lLdhlkEEnjflet3aUtImQTUdNbVbURdaHJV88X6zIJotp8QfDQeN977z0ZwIFXS1wLFy6UYcwwNSB5TtTw4kMOOUQ0atRI1qlOQtWmiu+Sfv13wYzLpIeA+qoadWOP\/LBprtjMFlFHYdpTQF1pmjyCtGl9jUQdV9b3FZp8dc0XuRWuvvpqsfbaa8ujzGFjBWnCXer+++8Xm2yyiUyus+2223prvrRxhuxoIFi61Ci7vfbaK3J4cV4EoU\/IuPY52wSPW6\/v\/a7lXcr52vYIA9N9Lu2BlKN4WoCU4xCyS99U+YaRNMqWw6ymkzPMf8i3Ena5jDdJ2edlzUfacFPBhC8qXv1xiivczEC06rVq1Sppe91zzz0lQfqmdwSJf\/fddwLZ0ZBvli7SfGH3hTkjanhxXgTB5GsOKlFxSXIBuix4E2kk4WkRpiFH7ZsLydEaUk0fqjaN38tl9qA3TfQFb5ZqwI7PevCdF3lZ87HJF25esMeecsoppTwPOnAI\/0UgRJC\/bdjE0Z\/0l19+udzZhjfDv\/\/9b2PuCJfwYggC19NPP+3TBS5bcASWLFlSMjfFHQoUDJgC8Kn6cIfVC0Vl4403lqSD3\/FDAR1h95bje+yt4MKhnvid\/san\/r+k+rP11lvLqvCJg1Ppb\/yPIiv1\/\/u0fdRRR8nieQgtj02+9PoPrwGYHEwXgh1AwHET60DLRrAGElNjkw22YLzexcntkJenoM8E4rLxEUhLu6Se6Z4Wrtpk2nbk+MiZawgzfSRt9kAv1A1w+tslGjEvaz42+YIQQYbILAbNFgnV1QuTvH\/\/\/qJZs2YCp5aut956VvnrBxeqJ4\/iwD2YFrBLffrpp0v7L52QUSnhxTpuptwOcRdPXNLxvd+1vEs539dLwiqqzTcu1vr9UTf2UI8aIBJmtrD12wXjoDH73m8rrxM1QssxvrQ2EmlMRNZ4Y8ZhC0FmjqRlb6ovNvmiUqSTBBnCbQcnzxIBwy6LQAc4sIOYd91118Ax\/fe\/\/5WvbnSBqPEq9vnnn8vQZQR1XHHFFTJYQ7Ud5z28uByC5Db8EPAlEr\/a\/UrHsSODtLC+4rq\/+fW4fKXDNGr0xPWtQu31s88+W75BWFqKRL7wdsBOppoY\/fvvv5d21wkTJohly5bJ5uAWhhMt+vTpE\/kcN2y4gbhffPFFMWbMGFGvXr1aQ8kqvDhz6XEHIiOQJ\/I1DUI1W7j6IlM9cb0tIoOa8Y0uRI0ysPc\/\/\/zzGfc24hlusPPCJAC760knnSQOOOAA+TtdIEz8RAmo0BGhDb1jjz1WdOjQoRZg2L3FT1LhxZlL5P93IC1yiFuv7\/2u5V3KFd3s4Dq3bOOMa7agdKDlMlu4yBSYuJRLUvaFtvlC84UN9h\/\/+Ic8o+2TTz4R++23nyRiBC3A6O3rUmabmAAdwRRow3QhyKJTp04y\/SKdXoyyOE1j4MCB0tYcdOVFEHofXSak62JWy8Wt1\/d+1\/Iu5ZJcgC7tRcE3iXt8+5aE2QL9DiNl3365lncpl6Ts87LmI5kd1AkGDReGcmyEIaBiwYIFYrfddpOuZ8i\/gNwLSWjAapvQcnFQJ85mq1u3rvwq6tHxeRFEEouW63BHwGXBu9eWv5K6twVlgXPpaRKbey7tZFUmL2s+NvmazAQvvfSSJMc5c+ZIW++pp54qjjvuOIEotbhEvHjxYulXjMQ5quta1NwOeRFEVhOxWtutdPI1yTWu2aJSNvfysuYTJ19V6PBcgCcC0juuWLFCjBs3rqSpRln08HZA5jJ4V0DTJfKNc3R8XgTBZodiRLhFmbe+96T1YAiyJUfJj4xxBW3uuY7DpRybHXxnkRBy4w02WLh24OQL02GPrtVCu0VABU5NRmQbkW+co+OZfF3R\/62cy0KJYmN2qTfJBejSnh8yyZVOq28+9apmC4zMx+OC3N6w1hE2HJb9zaVfSco+L2s+Nc0Xmi5CdidOnCiQEjJqVjOa0ngyX3TRRQJhxdCAEdhB5Bvn6HgOL06ONIpUU5LhxUUad9y+xgmlRtt0sAECHLIIpa6o8GLdzAB7Lw6yhL0XpgHy9YXdFxtxUWy+qGfEiBFSa8ZxQbNnz65BvhxeHHdJVd\/9LtpW9aESbcRRk9aT2QKaMdZ2mLdFtN7VvqtiNF8QI8gQLmc4KBPBF3Rdd911ol27doEhxS6AQrPFCcTI54sNPIQhq5pvJZJvWuQQt17f+13Lu5RL8tXTpT2XuZlGmbT6Frde3\/tx+gwS8EQ1WyByDxdIGeZLW7i977woNPnCz3fRokUyyc2kSZNkykecKgFf3y5dusiY6X79+onRo0dLf1vXy5TbAfVdeOGFsj6cZIFLJ1\/O7eCKsL\/NVq\/ZdwG6lncp57vIqO+m+1zac0c12ZJp9S1uvb73m8rHjdwzacm+86LQ5EuZzODTS4R70EEHlaLcQMzdu3f3Jl9TbgdspnXu3Nk6uxFpB3\/iPB8dn+zS5NqSQMCXSJJok+swI5CUC5yr2aIiyBdHx+MUCRzX\/vvf\/z42+ZpEoxMyysydO1cmcEeuB5ghYOrI4uh4XkzFRYDJN\/+yixO5h9HZThMpNPliYDjeB8kp4HHw6quvlswO0HiR\/AZZznzNDq7TQTc74D7O7eCGXlzS8b3ftbxLOd\/XyyCzQ14WoElqLli4Sbtmqbj1+t7vWt6lHJVJYnMPHljgi6yvRFzN4FaG\/JjY+AJIdCW14aaDZCLfOLkdshYCt88IMALxEEDq2SZNmshK6ETxoBqxh0SJ6+O1HP3uRMiXmsdGHHx68WR5\/PHH5UYcXM1gs0UC9MaNGyeWcCf6kPlORoARqAYEbGYLkC7IN+srUfJVB0NmCfL5hW047jFCWYPF7TMCjECxESCzRVjUXTlGmRr5qp2HWQL24aOPPlpqwnwxAowAI1DtCJSFfKsdZB4\/I8AIMAI6Aky+PCcYAUaAEcgAASbfDEDnJhkBRoARYPLlOcAIMAKMQAYIMPlmADo3yQgwAowAky\/PAUaAEWAEMkCAyTcE9IULF4qnnnpKJu7hqzoQgI\/6bbfdJp544gmZuL9r164yfwky9\/FV2QjALRZHlSFYDPLu1auXTIubhuyZfC1z6ZtvvhFTpkyRMeCHH364uOSSSyp71vHoSgjMnDlTnsKNKKjVq1eLoUOHyihNZPDjq7IRuPXWW0WjRo1E+\/btBQIyrr76aql42XIJx0Gj4sl3woQJ8vBOnTzDjprHoZxvvfWWXIQrV65k8o0zyzK6N6rsEYnZvHlz0bp1a9lznBeIkxY6deqU0Ui4WV8Eosoe6XCRGwKnoy9fvlwe2gDuQI7ypK+KJt9PP\/1UJmJv1apVLfJ0PWoeSXyee+45Jt+kZ17K9SUhe3QRZwcOGjRIDB48uJSiMOWuc\/UxEYgre5idIHOczIPPHj16sNnBVSaw0eLIocWLF8tz5JBwXdV8fY6aZ\/J1RT0f5ZKSPWy906ZNk+cRYu4ceuihnBQqHyK29iIp2VMD4IlLL71UnHHGGV4n8rjCVJGaL46qx1HXSMQ+fvx4eXy1Sr4+R80z+bpOpXyUS0L2GAkWXbNmzUS3bt04H0k+RBvai7iyxyYr9nlOPPHEko33vvvuE+uuu24qJqeKJF+SEr0+NGzYsAb5+hw1z+QbOudzWSCO7F9++WV5YCM8HOrUqZPL8XGn7AhElT0UNeQlh5kSScCQEnfYsGHySLQ99tgjccirknx9Tjtm8k18zpWlQtsCDJP9uHHjBDZrXnvtNbnpQlfv3r1F27Zty9J3biQeAlFlDxMTcv2CcLHxhgvHk0HuaTyEmXzr1i1JGkQ7YMAAaedr2rRpvBnAd2eKQJwFyLLPVHSxGy+K7KuSfOMcNR97ZnAFZUHAtgBZ9mWBP9NGiiL7qiTfefPmRT5qPtNZxY07I2BbgCx7ZwgLW7Aosq9K8v3qq68iHzVf2BlZZR23LUCWfeVPhKLIvirJF9Mv6lHzlT91K2OEtgXIsq8M+QaNoiiyr1ryjXrUfOVP3coYYdACZNlXhoxtoyiK7CuafCt7ivHoGAFGoMgIMPkWWXrcd0aAESgsAky+hRUdd5wRYASKjACTb5Glx31nBBiBwiLA5FtY0XHHGQFGoMgIMPkWWXrcd0aAESgsAky+hRUdd5wRYASKjACTb5Glx31nBBiBwiLA5FtY0XHHGQFGoMgIMPkWWXrcd0aAESgsAky+hRUdd5wRYASKjACTb5Glx31nBBiBwiLA5FtY0XHHGQFGoMgIMPkWWXrcd0aAESgsAk7kiyPYFyxYIGbPni1wlv3HH38stt9+e3nAII5lx8mem2yySWFB4I4zAowAI1BuBALJd8mSJQLHKU+ePFn8\/PPP1r7hXPsOHTqIvn37SlLmixFgBBgBRiAYASP5fv\/99\/IEXxyjvf\/++4uTTjpJarj169cX66yzTqlGJKXGsSxz5swRf\/vb38R\/\/vMfcfbZZ4vzzjuPcWcEGAFGgBEIQKAW+YJMzz\/\/fNG8eXNx5plnigYNGjgDuHz5cnHvvfeKiy++2PkeLsgIMAKMQDUiUIt8oc3+8MMP0p7LFyPACDACjEA6CDhtuKXTNNfKCDACjED1IhBIvl9++aXo2bOneP3110MR2m677cQJJ5wgOnfuLBo1ahRangswAowAI1DNCASS708\/\/SQ30yZNmiTmzZsn2rRpI1q0aCFWr14tXnjhBTFz5kzRpEkTceSRRwps0j3xxBNigw02EE8\/\/XQ1Y8pjZwQYAUYgFIFQs8PDDz8spk6dKm666SZRr169GhV++OGHon\/\/\/qJLly7SI+Kbb74RAwcOFHfccUdow1yAEWAEGIFqRiCQfFetWiXJFZrtaaedZsQJPsDPPPOMuOGGG2SgBf62la1moHnsjAAjwAioCDjZfEGmnTp1MiL34IMPSsK96667RN26dQX+tpVl6BkBRoARYAR+QyCQfBFWfPnll0t77vDhw2u5n0EzHjp0qIB72siRIwUi3aABs58vTy9GgBFgBIIRCLX5vvvuu6JPnz5ivfXWE6eccorYbbfdZI0rV64UEydOFO+\/\/74YO3asjIC74oorxIwZM8TChQsZd0aAEWAEGIEABELJF\/cuXrxYXHvttWLWrFlSy6Vr2223FX\/+859F27ZtZZjxJZdcIo477ji5+VYpF8aNMGvbBfc6aP0bbrhhYkNetGiR6N69u1i6dKn0NDnwwAMj1x3mLqj3n8abxrgiD8LxRpi8Bg8eXKN0ucZBbe+zzz4lE5xLt9X5ZbpXld8111zjbdJLci7p4\/nxxx\/FoEGDxPTp08WoUaPEjTfeKOes7Yo7l13wLFIZJ\/KlAQFs\/OCqU6eO2HzzzcVaa61VpPF69zWMfFGh74IL60SSCyaMfPX+F5F8VRLIauFHIV+TbHSCyjP50jzdZpttxJAhQ0S\/fv0CyReyifIAKeKcDFvjkkPXrFmzJqzgr7\/+KrXfl19+WXzxxReiffv20v4LP+CGDRtKIq7UK0jwr7zyigwqiTqpbJilRb5IeoS3E7pUTZEWRREnuioHlbxUHJN+QCYx302aui6jPJMvzRXMnZYtW1rf1tSHYxQ5FHFOusyPUPLFptr1118v7bt0YYLjQuKdc889V\/6o2c5cGi5KmSDBq4tbf6KbFpbptUvX2jA5QZADBgyoZXZQ2wN+ePAh+1zTpk2tcKqLV1\/Yan30nTreHj16SNnSq6TpFd70ZmAapwkPkxakEqnrW0WQjKhdHSuT1mkzUeh9R7\/h4w5zFN0zbdo0afJwJRdV7ocddpiU33PPPVfrfhP56v+jvqjrk0xV+oMcbahmNF1W+hxDnSZcqA8rVqyQcxBXkKmMZKTjEyRv21sb9dm1r3nlmlDyxcQbMWKE\/Nlpp53EWWedJUaPHi0nCSYcfsekqyQ7ryosl4WN8uokDjJVqAToYhKgek3kZVps+kSLSr62CUsLEd+Tvc9UViVWVzxs5cIeMnRfWDnqp2nR0nd6HUG4q8TkS776gxt1kb1anUth5GuTE+EfNFZ9\/uhEqNZtI02aD8j9bSNfdQwqkYfJe6uttjKmNyDlj946dQxcH4BZk7JTkEWzZs1kjt733ntPAgzCxZMVFotbb71V\/j\/pTaesgaH2XWy+KtHYTBFqPbS4TP\/TCRllMQlpYqvkbdMmVOyCyDfI7IA6bASq98lULmxR6n2HV41uwlG1w6CNMxtpmDRr2yuwSlKmvqvtq7hF1Xx1jVzVHG0PaBqPKlP1YaH+nwgIZkKaOyopmcY7ZswYqRXbytG8VTE8aTlPAAAb1ElEQVQM0kKDSNG0TmzyNilAprmfpLmuHPzjFWRBgyPyRQf1IItydLqcbbiQL01WeDyQNqiThU6CvXv3Lj3VdXOAbsPEeEFMulbmMtlctGt1sbm8HgbtWuv3q4tff0NQ5WjbsCIswrTaMA2V+myzD9NchvZJbc2dO1dqo3rbJpLw0XxN96tvEqo8wjTfIDt+0EPS9cFmMq2pG20UXOWiYasPdB95u9p8g8yA5eQM17acNF9oubD\/mch3woQJ0gUNGnAl5gB23XDDIggiVH2yIxAFbxPIGKdraDqpfvTRR7VcqHQB23aRw8jX1dXMRPRBXgZEIOinLTOeSYsPmrgurkpBfcL9hKWJzHViprKm11h9XviQr40k1AcIjTWMfHVM9DGob016WdPcDjI90ByjfqryC1IEVJkQ7sgZE+TCqT6obWvQpa+uRJhFuVCbL06mwGYbyBWbamR2OOCAAwQ0A5AIgi\/OOeecivR6CCJf\/RUWvo5ILARC1TUSnXx79eolg1ewmVUu8tX7ZJpwtvHqi2vXXXctkapKZCaNxoWkcVag62L0WSjqAsX4d9xxR6M2izp14qLNqaTJN0xLR19IVuUkX3qABLXfrl07+XaHbIfqZm\/YW5huZtA3CU0y1c1zNtNPEFY+c6XcZUPJF94Ow4YNE3hSIahi2bJlonHjxgKhx59++qk4+OCDpQ1Yz3hW7oGk1Z4P+eIBdd1110mn8yTNDkEaWNi4g2y+ccjX1icXO7RuysEiI6LzDYoIe33Wxw\/PArIt65qgbodNw+wQ9iZCMjG9OZhsvjYvG3ogoj7bRpg6t21vYrqWvsMOO0j8dDlFJV8XeetrEDKntyl1\/BVldqCJAH\/e559\/Xvz1r38Vr732mszni3BiaLyIbtt4443DOKCw3\/uYHeAilvaGmzrZbG5UKthpk6+q9Zr8atWNNJsPrr57bfIcCdrBNm1cEgb6piJpbnhAhm1AqTv4SW24hfmG62NR3zDCNtxM+Ks29yCzg0q+NnOQ6mIX9ramRmWazA7ol+khaHp4B5FvUF\/znuArVPMtLGsm1HGXDTfdfujqWuWiBbm4mgWZE9IiX9WWaILaxeaL+1xcj9SNGlNbLjiq7fi4moXJ39fbIcwtTiVn1G3SSF3Ga\/JCCLP5kreDbekgig0PLfLtVf3LXTfcXO38Qa6K6puSra9RIukSogznamqRL8wMU6ZMkYnRXS+EGXfs2FHm8620y3Xx6bkdXIMK9IWEBacGN9i0RcI5bBMqLfKFZqNveOgaLC0Am83X9NDQ6wzzclDnm82WasLIRGA27TqpIAubv6vtTQVjv\/nmm6XZT92YDQqy8PGI0TVK9EP13cZcRNbCK6+8UpIujgqDqdEkNxfyNcnBRd563agHsgrqaxF8fY1HxyONJF656MLveE3YdNNN5YYFkuvAt\/fnn3+WAoEJ4qqrrhJbbrllpXEvj4cRqIVAmJ05bchMm3Bpt8n1J49AqNkBu5oU7nrMMceUEumAgJ988klxyy23yCOGYJviixGoJAR0TwnKixFmt00bAybftBEuT\/2B5EtP+ObNm8tXYT2BDkW44VUEmi9y\/vLFCFQKAmHZ0rJ6tWXyrYwZ5hXhZhpypUe4VYaYeRSMACOQNwQCyffbb7+VUVg4Hh5J0\/XMZTA9YEMAmwFFjXCbOXOmaNCgQS25qP8zfZ83QXJ\/GAFGoFgIhNp8EeGG3dbzzz9fHhFPm2o4ueL++++XNl8QM9JLFjGv7xFHHOEtMZ2MbURdv379GnVTOdund0f4BkaAESgsAqHkC7sXkmeAZOHdoF7QhBEmC2JO8hidcqIZhXzT6p96RBP9js9ffvml1CT+Vr9T70mrX1wvI1BpCODsyayvUPKlDsKx+p\/\/\/KdYvny5XPw4Jv7www+XJokiarw0LmwmIiSYLoxPvz777DPr92p5073lELCqSeN30rjxu\/pTjr4UpY2dd95ZHv7KV\/UhkBfZ1yJfeDDgp9LPZqMpl5YgbKRM\/1e\/J3JXv8MDDidEJ3mBiFFvo0aNpJM6LvhoE0G7tPXBBx\/IpPqul2t5l3JBZXy\/c2nPdYxJl0urb3Hr9b3ftbxLOV\/5kkxM96W15n3nQS3yhRsLIlpwMsXvfvc7r+OBsLBffPFFQcei+HYmi\/J5EYQ+dnXSqKSskzeIm\/6HvBtxLiJhEDP9DnJWL5eFEqW8S71JLkCX9uJgGefetPoWt17f+13Lu5RLUvZ5WfNGzfeNN96QxwbB26Fr167SvIBXWZN5AYdrYvH\/61\/\/kqknEd4IG3G5LxA\/UuKNHTtWQFCtW7eW56C1atUq0CySF0EkiZdK0ETIRNL4ztc8AiIGCWMO4FMn5CT7Xq66XBZ8ufrC7ZQXgbyseavNF5nMkOYP8flIH0ihxao5AmklKcx4t912ExdddJHUetdff\/3yoimEeOyxx2QcOo6vRprLRx55RJ6rdscdd4gWLVpY+5MXQZQbMCJhIme4C\/oQc9EJmcm33DMuP+3lZc2HbrhBo3zzzTflZtvs2bPlqa3I84CsVthsO+SQQ8RRRx0lT9DNyk5M\/sjQduGXDA0dD4\/LLrtMIOkPwkJtpyvnRRBBZockp60L6YCEQcr4BCm7mjNAyG3atKmhHbu0h\/G5lEvy1dOlvSRx96krrb7Frdf3ftfyLuWSlH1e1nwo+fpMmqzKzp8\/X56kAXc4VcudPHmyeOCBB8Sdd94pttlmG2P38iKIrLBzbVfVlGHCQHBK2EVkjHLdunULK17W710WfFk7xI2VDYG8rPmKIN9nnnlGbhLec889NXbikQAFdl\/1uBNdwgcddJD81w033CA\/9cTwlZgmM6lZDm8MvAW9\/fbb4p133pGfQdfWW28t8AMT1e677y4\/s7qQqQ9eH3xVFwJ4S8eVBzfDiiBfW34JF\/IFaftcW2yxRY3i6t\/q7zB3qBd9p5fX6\/PpSx7LQkOGVuyrHUNLPvbYY8s2JNZ8ywZ17hpizTdBkZSTfBPsdq2qVCLGpiXOzMNFRK4SOH6PQtxxScf3\/ldffVUsWLBAjsPFfqy7u6mEnKTdz3ccacpdrzutvsWt1\/d+1\/Iu5ZKUPZNvgrN5xowZ4sYbb5TmBRzuSRc0X2y26eYItWnkKEaE29dff23skXqih15G\/dt2f4LDtFalk7JK1kTSVMZlogf12fd+vbxqO3YhY\/SFIvhwYjbGYXJ3812cvuMohxypjbT6Frde3\/tdy7uU85VvEJZMvgnO5nnz5sn8ErfffrvYe++9SzVjww0uaEEZ19IQhI2k6f\/q90TuOpGnQeYqERNB42QS\/GR5kanCVTtW+6q6vOH\/rhF7Lgs+S0y47fQQSGPNR+mts80XCXZwEu13330nN0s222wz2V4eEqgjw9q5554r3ZwouxolwoY7HLmfmQDKiyBswtMJG3\/r5E3\/i0PYpBmDiEHM+Ixq2ogyEfV74hAy1UUaMwWIkBaN\/yNJFFwT+ao+BPKy5kPJF36+sKnipArKakYHJULbxHlvf\/jDHzJPrvPwww\/LgBCYGRAeCxczBIkgyGKXXXaxzrC8CELvYBzNTCVjImRo2Pgdftq+F0iYckFssMEGQj0WPKwu13G4lIP9eN111y35HbuaLcL6GJSYSDV7+OTACGsz6HsXLKLUH7de3\/tdy7uUq0qzA9y4oFX27NlTtGzZUgYuXH\/99ZLQcHYbgi9gb0VUWdwL\/rojR44UOMIaWdPUC4Eeo0aNkifmwme3T58+4tRTTy1F061evVo+IB566CH5kIBmPnjwYHmqclDWtbySb1wsg+5XtWkiY0Qx+hCzSsh5MV1QcAjGTuHUrgEivnjbcjKrOZz1MjqR+7bJ5ZNBIC9r3ukMN0SzIWE6nj7du3cXo0ePltoPtGLkgPjhhx9in+GGiDSQOcgVuSFU8gUpnHXWWZL88RBASPPw4cPFGWecIYMrQK4cXpzMxCRiJtJyJWUyUeywww7SZJG1HdlkxqDNPnxinJhzpqRFySDpXostGb\/6f1tifpXQg353703llywE+dJBfaeddpro1KmTWLRoUQ3yhZjinuGGwzeHDBki5s6dK+3JpkMJsWH20ksvyaQ5dJLG9OnTZeTauHHjxEYbbSTtuhxeHL5wXF7xTLWQKQOkDBOGi5YMQgYZ4xPJmeK+aif56hlUl07IevpPPV2ob6KicCmlUwIBLmqYfdCJLDrZo0e4F3XoJG8jfde55lIuSdkXgnwpZ0Lbtm0FCNhEvvAowKs+bKtEjD5TB2289dZbUotGZjTY9VTNd9WqVaJ\/\/\/4CSc9BsHRh8w9a+LXXXisXd6WFF7tMSB+cqWzcetX7yYaMTxcNGXJSN\/RU7dilX0kuQJf2fPE15XC25XXWczijLVOuZ98+5LF82LFbULqQuCtIuzeZbKhe33lRCPKlo+GRoxd2WGg8qtkBWuuf\/vQn0axZs8DkNa4TwqRFL1u2TJoaevfuLTp06FCqStXKYQOOGl4MQeB6+umnXbvJ5QIQwMNy8eLFkkjUE0BMtyB0G+HctJFl0rbSArtI4cV6Un2EdNOlfqeXU\/9W78G9QfelhXma9ZJGTm3Q3zCZ4qK\/cSYlLrxpZ32Fejt8\/vnn4oILLhCffPKJ2H\/\/\/cVTTz0lXbqQxxeEhUGFeRS4DtJEviZtG\/Wp5Iu\/oYHrtmKX8OK8PAVdMSpaOWjFZD+G90nYVS7bcRqab9jYivK9bkYJ+ttkcrEdu2V7C8gCl2effTaLZmu0GUq+KA0\/2gkTJgg8NfCKgAsuP9BE4W623XbbJTIQJt\/\/wZgWOcSt1\/d+vTzZjmEzdjFVABEiZPgfk+lC39DzffX0HUciE9yxkrT6Frde3\/tdy6OcfgCvTtTQ1PXDYonkwUnff\/99CV0X801hyJdGBTMEBorTdLEQfPL3Qgvt3LlzCaCzzz5bmirUy0S+0Lhh6kCi9uOPP75UnDRfBFUg0CNqeHFeNV\/Xieu4nkvF4tbre79LeRDyrFmzJLG6EjKRMj5BxJibFBjiQswu\/fLFNqnyafUtbr2+97uWdynn+3AlWdB9KpkfeeSRcp8p68tJ80W0GOx4SJiOhC\/oOPIl4AL57bXXXqFBFjj1AvZAukCYerpGE\/nSph92y3v06FG6Hxtu8PWF2xuuPIUXZy3Uorcf1d1NH7ctnBrloERgr4Kv6kMgLwpXKPmC5OBlgI0R2FRBhvC5hSkC0U74G+5ePlFPNnGbyJc2\/RBkgeAO7IriQtm\/\/\/3v0v0MV6WGF1ff0rCPWDVZoJSPlmyrVU9KhHJ6YiLSsqNkkWP55Q+BQpAvbCxw5QLxQcNE8mmQHmy\/2GTDJEUUGYgZ0WVx8zzYfIahdYPwEUXXpUsX2R9kIrv44ovlKcu4OLzYbZK7vOIF1eR7v2t5l3K2MiBlmKdgDiOtGcSMy8Uf2Q25\/5Uy5XCm\/6l5nPXcGKaczqa2XbDw7TPKx63X937X8i7l4podVLwKQb7QbkF6J598svTzBRlfffXVchwIM4bTddwgC9RFocNwaYOme\/TRR8ucEepGHg7ERNtYXGi3Xbt20r2MTp6otPBilwnJC\/B\/CAThhfmFVKN6zgtERiaRlCiKHOgeXZvG3zDzUS5nVRM33WN6EJCmrvcr7pzyvd+1vEu5qiNfPcINO44g4z\/+8Y8ln9u45IvjZ0ir7dWrl\/Q\/xOYZCPbmm2+WgRscXhxnefO9PtplUBY5Pf2nKUVoXtE2kbzaV\/V7\/RQWncyD6iqCaaYQmi+lZYQGiigzaKaXXnppKW8u5XZYsWKFTIijn38WNhGh5SJZDnI1qPbchQsXygMXkaUMO5PVGF4chh1\/Hw8BF23LtwU9JzPu98nhTO3FSQ3q2+e0y7sSte3YrSDij0r0hSBfDBxZzRC6i9cguGvgWBcQLSLPoKEi6AJJbsj26iNMInd4Uaihw6rGDfcyDi\/2QTW4bFzS8b3ftbxLuUp89QzTynUiNhG8SvJUn+kEFqw3+NPa6khulpW\/Jpv5Ze2115ab9Cq5IxkYsjFmfYV6O0A7ffnll+UmW7169UTfvn1lOCj8dqEFw8sA9lc1YYfroMhvGAEbqpM1ab6DBg2SXhTlCC927TOXYwQqGQHd\/TPob9PJ3urbr++95cR16NCh5WzO2FYo+Za7h9CusdkGOy88KpCbV80nQf1JKry43OPj9hgBRqAmAnG1+yBzjw3riiBfbLhNnTo1clYzAgf240cffVS6tmGTDbZgpJdMO7cDLwRGgBGobARM5JyHfNOhmi\/czeBTCz9aOkZIFxX8b4MOqUT5oPBiZEeD2xhCTE8\/\/XRp\/0WmMlxphxdX9rTj0TECjEBeEQglX7z6I4rsxBNPlJn\/QZCw8UJTfeKJJ8RBBx0k00rqOTv1AdvCiylrGuq+4oorZN5e9diftMOL8yoY7hcjwAhUNgKB5EuJzBEDD20UWjDyKWAjrEWLFtL7AVFmSHqDv30vPV8wNvT0K+3wYt8+c3lGgBFgBJJAIJB89SALcg075phjShnGkGoS+R+ihBdTBB3c19RE6TQweEDgJ83w4iRA5DoYAUaAEfBFIJB89WOEoIUi8AEmBsowFifCDX6bCKaAXdd0oS2cHYdLPb0YoaK6ixvMINOmTZMmEtQHF7WBAwdy5irfGcHlGQFGoCwIBJIvJdbBSQQIpGjSpIlMJYnNM0SkQSu94YYb5LlrUc9wK8souRFGgBFgBHKGQOiGG7RTJD7H8fB333237D78brEphixm+B4aJh3hnrPxcXcYAUaAEcglAqHki17DUwH5F5BSEqF6s2fPFrfddpsMU4T9F6YD\/RiQXI42QqcQbYcQaiRr56s6EMDeBuY3vHnghdO1a1d5aECUKM7qQKxyRok8Ndi\/QsIvyBvJvqJG8Iah4kS+YZVU4veIjZ8yZYr0b8YpGvqRR5U4Zh7TbwjMnDlTLFiwQPTr108gVSmioXAE1n777ccQVTgCiFeAktm+fXupXCKNLRSvnXbaKfGRh5IvNtmWLl0qTyrWj5+m3iBpRceOHWsdC5R4byNUCG8MPM108lQ36GA6ad26tRgwYIBo1aqVNKlgsxHHJWERIs0lk28E8DO+JarscWIL\/M0xJ3DRwQG0+ZvxsLh5BwSiyh4RtfXr1xebbbaZdKVFxC3Wflgcg0OXahUJJd\/HH39cXHjhhdboNtSIMGD92PYonUn6HkTOoe8gVJ08H3vsManRQLtBhB6StU+aNEkuNNVnGZuLOPKcyTdp6aRbXxKyRw+RYwR+7TixBfOcr\/wjEFf25FI7ffp0KXt4dqVhcnIKssApxYg+S4P90xAlbLTwxoB\/MDRc\/aRkcqGDZoPgEWi6sO3hdA5o8SBaApvJNw0JpVdnUrLHfIDrIjaZMR8OPfTQ0ENi0xsV1+yCQFKyp7bAE8jceMYZZyRyRqU+Bq8gCxcA8lAGfr44wwsbhePHjxf77rtvDc11\/vz50jvjlltuqaHlTp48WTzwwAPizjvvLOWWYPLNg0Td+5CE7NEaFh0iO7GZTIe2uveCS2aBQFzZY5MV+zxIpUA23vvuu08g5W0aJicnzRenSeAMt6Jd9PrQsGHDGuSLBPFI5AOfZdWQDqKF3RfaDhK842LyLZrUf+tvHNkjfzX2OuDhoOYZKSYS1dfrqLKHogbzI8yUOEfyu+++E8OGDZOutXvssUfiQDrZfO+\/\/34ZTGHKvZB4jxKs0CYEW1Qek2+C4GdcVVTZjxs3TmCzBoFF2HShq3fv3qJt27YZj4qbd0EgquyhdOFEDBAuNt5wIZIWck\/jIVyLfJFMB6o3HUPy66+\/ihdeeEEsWbJEtGnTRmy11Va1xp9Xb4c4QiDN10XYXCZ\/CLDs8yeTcvWoKLKvRb6UTOf11193xiqv3g42IcyYMUOeP4cnHfJE0AXNF5srujnCGQgumBsEWPa5EUXZO1IU2YeaHcqOXIIN2oQwb9486Th9++23i7333rvUIjbc4IIWlhg+wS5yVSkhwLJPCdgCVFsU2QeSLyVAp9SOBcC9RhdtQkAqS9hyYEahTRUqi+RB5H5WtPFyf\/+HAMu+emdDUWRvJF\/4xk6cOFEeH4Sjg+DziuN9ELCA89WKctmEgP4jbBgpK2FmgNkELmYIpkCQxS677FKUIXI\/LQiw7Kt3ahRF9kbyBTHRaRX777+\/+Pjjj2W8+ymnnCJPFl5\/\/fULIdkgIXD+30KIMHInWfaRoSv8jUWRvdHboX\/\/\/tLNBkklYHKAzyM8IBCUoPrAFl5KPABGgBFgBDJCwOrtgKAKNaoDRwXB2Rhhuzglgi9GgBFgBBiB6Ag4ky+cjkG+o0ePZvKNjjffyQgwAoyARIDJlycCI8AIMAIZIMDkmwHo3CQjwAgwAlbyRWafE044oYQQEo5fcMEFMiFNy5YtayCHuGeEGCP1JF+MACPACDAC4QhUdHhx+PC5BCPACDAC2SBQi3yRRPqNN96QycVdL\/j94tiVovj\/uo6LyzECjAAjkBYCFZ3bIS3QuF5GgBFgBOIiwOQbF0G+nxFgBBiBCAgw+UYAjW\/JDwIIE7\/33nvFX\/7yF3nkt3qQK75DlObWW28tjj32WOmnjmx2piNhKCQVofR5PAw2P4hzT5JCgMk3KSS5nkwQWLZsmejZs6fYeeedRZcuXeSZfLT3gO\/69u0rhgwZIsPlmXwzERE3akGAyZenRqERoMhLk0aLLHXIzQytGGlEmXwLLeqK6zyTb8WJtHoGhLP4Bg8eXBqweqIKkkGNGjVKfjdw4EDx3nvveZEvcpl07tzZCGZeT26pHslXxkiZfCtDjlU5ChwV\/tJLL8l8Ix06dJAnzpLL48qVK0WfPn1kYvzDDjtMHogIzRcHYarBQwQcbL7Dhw8XMFXA5ouzC99+++0auIKQQejHH398KeNfVQLPg04EASbfRGDkSrJCwGZ2mDNnjhg5cqRMg9qgQYMS+S5dujSwqzat9vPPP5cRnjjjEAn3d9xxx6yGzO1WCAJMvhUiyGodho18YetdsWKFuOyyy+RJLL6ab926dUuQwmsCJI6DVceOHSsOPvjgaoWbx50gAky+CYLJVZUfARP5fvvtt9Lc0K1bN3HkkUfKTgVtzOF7m6sZbMdTp04Vl156qTQ1dOzYUSCXCV+MQFwEmHzjIsj3Z4qAiVTnz58vNd4xY8aIxo0bxyLfd955R7qyHXDAATKp1CabbJLpeLnxykGAybdyZFmVIzGR74QJEwQ2x6666iqx3nrrRSZfsvOiAhB5vXr1qhJjHnQ6CDD5poMr11omBHTyXbVqlcAZhPBoUL0afM0OCMoYMWKEeOihh9jOWyZZVlszTL7VJvEKG69OqgsXLhQDBgwQN910k2jatGlptL7kO2vWLHmCd\/v27UW7du1q2HmxgbfnnnvKqDm+GIGoCDD5RkWO78sFAjqpIvACvr9wM8PJ23T5ku\/48ePFuHHjjGNs2LAhn+KdC+kXuxNMvsWWH\/eeEWAECooAk29BBcfdZgQYgWIjwORbbPlx7xkBRqCgCPw\/jOcvDMLGcy4AAAAASUVORK5CYII=","height":211,"width":351}}
%---
%[output:11375b46]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAV8AAADTCAYAAAAmsBCWAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQm4jdX6f4WSa043lPEYk9kh0SCikKjIrEjm4cpDMiYZShGhCHGrkwau41aXhOqcQubKEEeGIuEYcw+i\/\/N7u2v\/v7Ptffa3v\/3N+13Pc55zzt7ft9b6futdv\/3ud71Dtj\/\/\/PNPkiYICAKCgCBgKwLZhHxtxVsGEwQEAUGAERDyFUEQBAQBQcABBIR8HQBdhhQEBAFBQMhXZEAQEAQEAQcQEPJ1AHQZUhAQBAQBIV+RAUFAEBAEHEBAyNcB0GVIQUAQEASEfEUGBAFBQBBwAAEhXwdAlyEFAUFAEBDyFRkQBAQBQcABBIR8HQBdhhQEBAFBQMhXZEAQEAQEAQcQEPJ1APR4H3LPnj3Ut29fuv766+maa66h9PR0qlq1Ko0ePZpuvPFGXfAsXryYSpYsSbfffnvE69etW0c9e\/akUqVKBcZr164dPf744\/z\/rFmzqFWrVlS6dOmIfckFgoBZCAj5moWk9KMbAZDvwoULacSIEUzAV65coenTpzOZtm7dWlc\/0ZJvUlISTZo0ice7cOECzZ8\/ny5evMgfAjly5NA1ZriLkBgQPyByaYKAXgSEfPUiJdeZhkAw+YIMJ0+eTHfffTfVr1+fFi1aRO+99x6PBw21S5cu\/DcI+91332XtOCEhgR588EGqW7cuffLJJ\/Tmm2\/SH3\/8Qc2aNWONFiSrGjRfLfni9SNHjtCYMWPo2Wef5Xu7du1KRYsW5Q+Br776iom5Y8eOPHZGRgZNnTqV1q5dyxo6+n7iiSdo8+bNtGPHDtq\/fz8NGjSIx58wYQKTO+aI\/gsUKMBj3HbbbfTpp59SmTJlqEmTJvTWW2\/R6dOn6YUXXuBnkBZ\/CAj5xt+aO\/7EIF+Q1+HDhwNzad68OU2cOJE2bdpEX3zxBRMW2quvvkqVK1dmwsPrw4YNo0uXLtHw4cOZHK+77jp+vV+\/fpQ9e3Ym7sKFCxP6y4p8f\/\/9dya+9u3bE7RokO+JEyeYSB977DE6e\/Ysa8r9+\/enzz77jMdp27YtHT16lEaNGkVDhw5l8k1JSeF5Z8uWjV5\/\/XXq0KEDk\/jKlSvp559\/ZnNG7969qU+fPnTXXXfxtblz56YBAwbQli1b6D\/\/+Q8\/U6zat+OLKhOIGgEh36ghkxtiRSCU2QGaKYgvV65crAk2aNCAh4HWCnJFg1asXodmDBsuNM\/x48dnmhLsuyC0aMkX2nRaWhr\/bNu2jb788kt68cUX6f3332eiL1++PJsXXnvtNbr\/\/vuZfNFA1mgg7B9++IF+\/PFH1p7Lli1LPXr0oOeee4614EKFCrGGr54jGIdYcZX7vYWAkK+31ssXsw1FOuo1aLiNGjUKHKRBs1y\/fj1\/pQ9Fvhs3buSv+IoAYS64fPmyLrMDTAQgaWis0Hy\/++47\/oE2XLBgQda60a\/SjMuVK8fzwOstW7bMRL7Hjh2jcePGsdmjdu3atHv3bkpNTQ1JvjCv4KBQyNcX4mz4IYR8DUMnNxpFIJh0oE1Cu9y7dy\/VrFmTtdmBAwdy9yBIkNV\/\/\/tf2rBhA5sj8PfTTz9N3bp1YzJcunQpa5c5c+ZkrVSrOSvtOfjADdfh6z\/swzA\/gHyXLFkSIHjMAQeC0HxXrFjBBA+zwy+\/\/MJzgOlBq\/nimRYsWMD3wIQwbdo0nn8ozVfI16jk+Os+IV9\/racnnibY1QwHWhUrVmRSg8YZ6sAN2uzcuXOZaGEegF31gQceYC0TxI0DLDR1QKe1oWblaoZ7FPnioAy2XNhvYauFTTp\/\/vxscoBpY+fOnWx6yJMnD3Xv3j0T+eLeGTNm0PLly6lChQpslsC8oFljblqzg5CvJ8TU8kkK+VoOsQzgdQRWrVpFJUqUYOI9fvw4vfzyy6x542BPmiBgFAEhX6PIyX1xg8C+ffvYrAG7LrweoKEnJibGzfPLg1qDgJCvNbhKr4KAICAIZImAkK8IiCAgCAgCDiAg5OsA6DKkICAICAJCviIDgoAgIAg4gICQrwOgy5CCgCAgCAj5igwIAoKAIOAAAkK+DoAuQwoCgoAgIOQrMiAICAKCgAMICPk6ALoMKQgIAoKAkK\/IgCAgCAgCDiAg5OsA6DKkICAICAJCviIDgoAgIAg4gICQrwOgy5CCgCAgCAj5igwIAoKAIOAAAkK+DoAuQwoCgoAgIOQrMiAICAKCgAMIuI58UZ\/rm2++4RIsKNONdtNNN3H9rHr16mUqjOgAXjKkICAICAKmIOAa8kURxa1bt3Il2YYNG3J113z58vFDgpBRrHDZsmVcTwvvSxMEBAFBwMsIuIZ8z507R2lpaVSlShW65pprQmJ65coVLsldqVIlL2MucxcEBAFBgFxDvlgLkOuBAweoQIECXNZ79erVtGvXLq5se++993L9LGmCgCAgCPgBAdeQ7x9\/\/EEzZ86k5ORkyp49OxMuCLhJkyZsA77hhhuoZ8+eXNZbmiAgCAgCXkfANeR78uRJmjJlCg0ZMoQyMjK4Wuzzzz9PN954I8Ek8corr1D\/\/v2pYMGCXsdc5i8ICAKCgHvMDunp6TR37lwaMGAAH7CBfMeMGUOFChXi\/6dPn049evTg\/6NpOMj797\/\/Ta+++iqbLdBHy5Ytw9qVo+lbrhUEBAFBwCgCrtF8rSLfnTt3MqmDzGGyGDt2LHXu3JmqVatmFDO5TxAQBASBmBFwFfkOHz6cEhISCPbftWvX0j333MN+vZcuXWJPiIkTJ0at+S5evJig\/bZr147Beu+99\/i3+j9mBKUDQUAQEAQMIOAa8r1w4QJt376d8DtUg8mgatWqUXs8TJ48mRITE9lbAi0lJYVSU1Np2LBhBuCSWwQBQUAQMAcB15Av3MxOnz7NWmqoBpNB\/vz5o7bVgnzr169PDRo0CEm+3bt3j4jkmjVr+BptcEeo18J1JPd7F7+s1i5YJkKtv9wffu3vu+8+yps3L+GwPbjlyZOHFS0oS7\/++mvIvRcL\/vv27Yu4762+wDXkiwUYPXo0\/fzzz3TmzBk6fvw4lSpVip\/\/xIkT9Oijj9KTTz5JWJRoWrCZAf9jUVu3bs3dlClThuxcCLvHiwYrudadCPhVZubPn8\/eS2ovatGHeyn2JVIK4PnNbG7B0zXkq8CF2WH27NnUpk0buvnmm\/llEPKHH35IvXv3jtrsgAO3RYsW0YgRI7ivF154gbp06RKIkrN7IX766ScqXbq0mbIkffkcAb\/KjJBvuO\/5Dgk0vB6Q32HgwIH0t7\/9jWfx+++\/s6tYr169oj5wU65m6BN\/9+3bl5o1axYI1hDydWihg4b1K8GYga5fsYmVfI3iYveeDycDrtN8L168SBMmTGDtECSJpsKMn3nmmag130jCb\/dCGBWYSM8h7\/sXAb\/KTKzka3TF7d7zniFfpenCNovgCJghEGLcrVu3QJYzo6CHus\/uhfDrRjJzTaSvzAi4RWaSvv2VkjYcoeS+NUxZIiFfl5gdQLJnz56lwoULZ7mwOIiLdE00kiHkGw1acq0TCLiBfCev2E9J3x6h+gkFCCSc3KcGNShbICY4hHxdQr4IrEBSHeT0ReJ0BFsgwAIN4cU7duzgfL7Vq1enhx9+OKZF194s5GsalDF15AaCiekBLLzZDdhUH\/8NbR1ZL\/CUhQaviZmAYyVfo7jYvec9Y3Y4duwYISpt6dKlBHDRkOGsVatW7G4WbW6HSHvC7oUwKjCRnsPr7wsu4VfQaWxAtMOalqZhTf9y\/URrOXML\/47FBCHk6xLN1ynyEPJ1CnkZVy8CTpJvViQLUp7ZvhK1Tyyi91F0Xyd+vrqh8u6FQr7eXbt4mblT5KvsvLDvliiUKyTcIOD0V8wv6yXkGwfSLeTrjkV2imDc8fRZz8IJbHCoNnnFT\/Rau0pZHqzBFtw+sWgmk4ReTMXsIGYHW8OLndhIejeDXOdOBOyWGRBv36SdV9l5w6ED7ReHceG043D3xUq+RlfLboUr3DxdF2SBiSKme+TIkezlgMAKeEEMGjSIq1qY3exeCLs3ktl4SX\/2I2CnzMDUAI03GlOC0cM3IV+Xab7w90UmssaNG7PXw6hRo+iHH37g\/L4S4Wb\/xpcRnUfALvJVNt5ozQgpe09Ry1lbYnY90yItNl8H5E5VtOjUqRO9+OKLXEoI\/r5GywhFegTRfCMhZM\/7dhGMPU9j7ihWY6O0XZgN4FJmxHsBZorUtFOZfIFjQUEP+RrFxe497xmzAzTf1157jTVffC0B+SKf51tvvcUpJ6NNKRlJAOxeCKMCE+k5vP6+4BJ+Bc3EBvbclL0n6VB6BqWkneJBQbrRaruhZhvKHzgruYzV7GAUF7v3vGfIV9l8kQLy+++\/56AK5N8FIZcvXz5Ljtm2bRsn5UEIMtJRol4bEvRkVUDT7oUwKjB6yRUaCDbWwZMZfMvB9L9+SxMEFNHiN8KESxS63pCXQjgk1UGd3tDjWMnX6Iravec9Rb6YLEzRyPVw+fJlXRUscC0IGykjK1SoQF988QWtXLmSa7UtWLAgbAFNuxfCbPLFV8bUvSdZi1GnzdhYaHCAl+Z9BMyWGSsRiebwTcjXJQdu2koWoYTjlltuoXHjxnHm+1Dt4MGD9MEHH1CfPn3YRvzLL7\/Q888\/T3Xq1KHcuXOHLaDpVfJVfphKi\/E60XqJYKwkr1B9ewkbfNOC76+eyLdYydcoLnbveddrvqqGGzRduJbB3IDaa2jwdMiVKxe1aNEikAQ9qw2AJD3vvPMOVzzGfbfffnvYApqqRMnnn39uy55CVQ58kMTS5qw\/RW9sOEU96xSgp+rGllkqlnnIvfYgYIbM2DPTv0ZR8rm86y1ULF+OsEN\/+umnfIZz5513XnUNPJyOHDlClStXpqJFi5o2\/UaNGnFfdpYOcz35qglCA54zZw7169cvUyUL2HyfeuqpgOYLrwgUv4Sdt2fPnoFqxNCAx48fzz7BQ4YM4b6yKqBp96eg0U9rhQ++1sGea8YBiWkSLR1ZikCsMmPp5MJ0rsf8EKvma\/S57N7zniHfc+fOsXkBxTLVARs++eFqhsCLfPnyhXwWVS4IGi\/KwlerVo0rHbutgGasGyk4tZ9RAZT7vINArDLjxJPq8f0V8nWJzVcrIN9++y27mOXMmZNfRg03FL6sW7duWDmCOxpIG4duqvAmLvZLAU2l8WaV5MSJTWbWmF4kGLOePVI\/XsUmUrRcrORrFBfRfCNIHOy2cBmDLRiVK6699tos7wBhwyxRokQJ1njRYFt97rnnKDU1lYtyermAJjRevxIv1sroRopEXH5438vYQGkoXihXSM8bIV+Xab7hvB4ieTsY3WR2fwoa2Ugg3kjZpYw+v9znfgSMyIxbnkqZH6LJFYG564lwM\/qMdu95z9h8ldeD+kw4c+YMVy\/OkSMHdezYkbJnz24U85D32b0Q0W6kaKOGTAXHxs6ixcXGqTk+lNexMRJ6rId8jeJi9573DPmGmqhyHWvdurXpFYztXohoBAYaLwImvO7Dq4e9osFFT39+usYP2ISSZTE7uMzsEGrTIGBi4cKF7H4WztvB6GZzK\/mqAzZt0UKjzyj3eRsBP5AvViC46Gas5Gt0Ve3e857RfLX+u2rSKKCJsOFmzZrpCrKIZlHsXgg9G0ml9hPijWYl\/XutHpnxwtMHKxRCvi7TfGHzPX\/+PAdYZMuWjWUKU4S7GcKElSeDWcLmRvKNFzuvdg39QjBmyaVfsdEr22LztUKSsugTlSvgr7to0SIOskCOBrTTp0\/T3LlzOWLN76Xj48nO61eCMXvb+OmDSeV+aJBQIJAMKlS0ppCv2VIUoT\/46n722We0adMmDqhQQRa4DTHe9957L3s9mNncpPkaORU2Ewvpy50I+IV8Qbz9knYG8ghr0QYZJ\/etEXhJD\/kaXS2797xnbL5Ipr5\/\/35KSEgwnWhDgWD3QoTbSHqrxRoVOLnPuwj4hXxVvt9wK6HNhCbka6O8IrgCNdseeughToiOfA7a5ucgC\/VVDCVchjUtZSPq7hnKLwRjBaJ+wQYHbqp6RiickI9aHTLrIV+juNitcHlG87VCeLPq0+6FCCUw4lYm4cVZyahRkrF7L0UaD4dtWTUh30gI2vA+3M2Q6\/O3334LjJY\/f35q27at72q4KbcyP+dtsEFkfD2EX8gXh8lZlbWKlnyNLrrdCpdnNF+klERmMqSE1NZsQx23qlWrcj03M5vdC6HdSCruXU\/WfzOfWfryFgJ+Id9INl+t2U2P2cHoKtq95z1Dvqp0\/IABAwKuZtGCvHv3bpo0aRK9\/PLLnHzdjQU0oQG0nLUlbsKHI62hXwgm0nMaed9P2ISz+2q1XmCkh3yN4iLkG0YKkccBfr4o\/XPrrbdGLasqK9qxY8do1qxZdPToUfYRRmpJBG2gonHnzp1Zs0azeyGUwIidN\/PSGt1IUQuIB2\/wGzbK1Ba8FNqITiFfBwQV5IlKFKjbVrx48UAuBz3eDiBu5O0tWbIkrVq1ihOyw28YEXKoYowWXNnCCfJdd\/x6mrziJ0kT6YB8eXFIv5FvqDUIjnzTQ75G19LuPe8Zs4NRQHFfSkoKweQAdzVULgb5QutNTEx0TQHNzT8eoh4rLkvxy1gWOs7u9VoBTSPLs3znORq76jipoptSQNMIijHegwO3999\/n0OKtQ3RbiiEWaVKFQ6+CC6g2bVrV5o3bx7BVnzp0iU2MyjydUsBTdh5m03\/lu6uUDgu0kRGIwrxoN1Fg4f22njBRlt0U4\/maxQX0XzDSCIi3ECiSCPZpEkTQil5HJihLhvMBygFj9SSwW3dunXUoUOHTC8XK1aMHnvsMa5krDU7wGMCuYHttvlCuPYdO0ffj726VLbRjemX+4xuJL88f1bPES\/YaItuZj+2k8u716tXj89lQjWjuAj5hpE2VK6YPXs2p5DMkycPX4XXUDpeabejR4\/Ocs9BK1aaLw7ccIAH9zU0FOLs0qULVapUyVbyVYcMMx8sTPWrlYsHzpBnNAkBoyRj0vC2dqOKbj5bM4OKZkvPknyNTkzINwvynTp1KpsP4CaGhkO4KVOmUK9evWjBggUUDfkqVzMnC2gq4kUdtpuzn6TSpUsblRu5Lw4RiCfyVctb9plV9FyVo0K+dsv7kiVLKCkpid3NkL9369at1KJFC67fdvjw4ZBmB6NztPpTUBGvSpsXjxtJz9oILuFRikdsBi9aT7Wy7xWzg57NY\/Y18NPds2cPl46HvRfl4\/E3kqybmVbSKvLF4RoiepK+PRJIFgKM4nEj6ZENwUXIV4uAHLjp2TUWXIONiEM2+O2i4RBu79699NJLLwVMEWYNawX5qug1zDG4FJCQjFkrFz\/9xKPM6CFfoxJgxZ43Mpdsf7qsjBCqWeBQrE6dOrRx40aqWbMmHTp0iKpXr0533mm+l4CZC6G0XQRQBIdLqsWJx41kRDDlnv9HIB5lRsjXgR2gze2A6DSQI8wOCJbo06eP67KawbSQsvckpaad4oxN4UhXQWkm2TuwPJYNGY8EoxfMeJQZPeRrVGbcgqfrNF8EWcC80KNHD65ogWKaOHhDkpyBAweaXsMtUo7RSBvkmvPH6ZrzJyjH8d2Ua9eySJfL+4KAIKADAbiCFilShHbu3Ml1Hc1u8CF2urmOfAEIarmtXr2aOnbsSM8++yylpaVxkETv3r1NPWxzGnwZXxAQBOIXAVeSr3Y5cOgG7Tdv3ryBUvLxu1zy5IKAIOAXBFxDvioVZHDtNgW0nqxmflkUeQ5BQBDwPwKuIl\/YdJEUB+WC4O1w\/fXXB1YAuXhRSghBF9IEAUFAEPA6Aq4hXwCJIAok1IGP79KlS7ls0KOPPsruZtdee63XsZb5CwKCgCDw\/wql2\/x81czgfoxQ4uTkZFq+fDl7PAwePNh0VzORBUFAEBAEnEDAVZqvFgCtFizk64RoyJiCgCBgJQKuIl8xO1i51NK3ICAIuAkB15AvvB1w4HbixAl68skn5cDNTVIicxEEBAHTEXAV+SJPr7iamb7G0qEgIAi4EAHXkK8LsZEpCQKCgCBgGQKOky9yOVy8eJEfEO5kqnSQZU8sHQsCgoAg4AIEHCFfHKylpqZyXbbjx49Tvnz5GIqMjAwOpED9NlQcloAKF0iITEEQEAQsQcB28kW+XlQnRrhw48aNr9J08f6KFStoy5YtXARTWmgEUJ5oWNNSAo8gIAh4FAHbyRdaL34ilQLCNXZovuHKUrt5PTMqPkQZFVsS0lnm3jyf01lKEwT8hICVKSXdkE4Sa2U7+SoBCZdIJ1euXNSkSRN65JFHAuYIK4XK7sTKZozXN2knHUrPoOKFcnGdOCRwH9a0NLVPLGIlVNK3QwiYITMOTd3wsHqSqRvt3C14Oka+ly9fpnfeeYeuu+46uvfeewnhxP\/6178YT5gkNm3aRKNGjTKKb+A+9ItcEa+++iqPhSTtLVu2DGjVdi+E0ez7WiBaztzCxDuzfSV+Gf+npJ2KWEUjZjClA0cQMENmspq4qsaC38EtUmUWqwAR8rUKWSKC5jtnzhwuA4+KxGi\/\/\/47H8J16tSJFi5cyInUY23IhI8SRLAfIzPa2LFjqXPnzlStWjXu2qvkW79swatsvrADo34cmlObxuh6WU0wRuflhvvMxkaRLb49qQ\/t+gkFqEHZgpm+PWlrEuKD3s5vVnrI1ygudu\/5cDLkmOarLRcETRcNGc2goaJWG8h3zJgxMcv+4sWLWatGJQy09957j3+r\/+1eCKMCowWi+vhvqH1i0bAHbiBhNEXEioxLFMzFGnOJQtfLYV3MkmVfB0ZlRpEsZqqIVskCfoNw1benSE+DclvJfWpQg7IFIl1qyvt6yNfoQHbvedeRLyaEckEg2Jw5c7IZ4OzZs6yZomYTCLNNmzZG8Q3cN3nyZEpMTGTTBlpKSgq7uQ0bNoz\/L9zrA\/6NelFmtKJ5c1zVTbF8f71WLG8OOnPmDA25r0RMQ9WasZ\/GNi5MD1bKo7ufOetP0eGzf9DhM3\/Qkf\/91n1zhAvV85nVX7z3gzUyo6l1qXVzLu4OMmO0PbXkL5PEnIfN2SeR5vHDDz\/QkSNHqHLlylS0aNFIl+t+v1GjRnytGw7dHNN8AQAI9sCBA4RP9pIlS7JNFpWKUTookjeEXrRBvvAZbtCgAd8STL7RfgoqrTLc+AfT\/3vVW9A6VNt37BwTYHCDmUCvJmK3FqIXa7nOGgSMar5mzwZyZ5f5QTRfs1cvqL8lS5Zwrl4cvg0dOpSmTZtGjz\/+eIAozRg+2MyA\/0HyrVu35u6jJd9Y5xRuI4HUU\/eeZBscGsg4nGnBj+TrFoKJdX2tuN8t2MDLBqaM9FcaWvGYmfrUQ75GcbF7z4cDyzHNF1+\/p06dyodfsPPC\/IAwYxy4gYhV1Fusq4wDt0WLFtGIESO4qxdeeIG6dOlC8CN0E\/lqn1MddCR9e4Twd\/DhGch368h6\/LpfmtGN5Jfnz+o53ISNXR\/8Qr4WSrbydkB5+JdeeonJF7kd8DdSSxYqVMiU0ZWr2euvv85mDoQuN2vWLFAJ2e5PwWg3Esi3X9LOTK5k2AB2aB+mLIB0EjMC0cpMzANm0YHyMU\/uW8PKYUgP+RqdgN173nWaL4hw\/vz5hHDi7du3U+\/evenzzz8nBFn06tXLNJtvpAWyeyGMbqRQJBzp2eR9fyBgVGaseno7tF8hX6tW73\/9XrhwgVauXEmw\/SKpTvPmzTmyTVu12OIpuMbmq\/c5U\/aess3dR++czLjObQRjxjOZ1YfbsIH2m5p2ik1fVjU95GsUF7sVLtdovsjZcPr0aTYBhGp2l4i3eyGMCoxVQi79uh8BN8oMtF+EtFuV3EkP+RpdObv3vGvIV5vTAYduSClZqtRf2blQQgil4lFGyEheX\/gNT5gwgSPl7rnnHhowYABHz\/ktvNio0Ml93kTAjeQLJK00Pwj5WiirMDnMnj2bAyng24uGEkIffvgh23\/hDhZNQ2AGfHoRPHHTTTfR+++\/TxgDARZ+Cy+OBhe51vsIuJV8gSyiLa0wPwj5Wii36enpBA8EeDZoczvA7QwHbtF6O0Dr\/eyzz+iZZ57haLk9e\/ZwiHKFChU4gs5P4cUWLotjXbuZYBwD5X8DuxkbldTJbBOEHvI1ikvcmh2UIMOnFyaC0qVLs+sX2urVq2nXrl1MoNFqvtCaofnCn\/fvf\/87JSUlcZY0hCfeddddYcOL7V4IowLjNAFYPb7gEh5ht2OjEjo1SChAZrmgCflavONgm0XEGWyyMBEgj2+3bt10BVhAc+7evTtt27aNevbsyYEZX375JU2ZMoXOnz\/PaSNhX4bWe\/fdd2cZXozHhJubHQ0fEiqRkB3jyRjeR8ALMrPplwx6Y\/0pwu+edQrQU3VjS8AjuR0skFuQLBLoFC6cdZIPHMRFukY7PeSDAOnmzZuXAyhQhuiTTz7hw7zs2bNnMju4MbzYAqilS58g4HbNVwuz0oIRfflau0qG3SL1aL5Gl9fub7vh5ml7eDFIMjk5mbZu3Upt27alhISEgF8vAi527NhBy5Yto+rVq9PDDz+sG19kQIIZA2aHggULsgkCGi9MEF4LL9b90D660EsEYzfsXsRGkbDRSEw95GsUl7glXyW4x44dI+TaXbp0KWc1Q6tYsSK1atWK3c2iPXCD3zA03ZkzZ3JWNBywIYcDtF6YNbwaXmz3Rpfx3IeAUZJx+kkQENRy1hbOQRKtFqyHfI0+X9yTr1HgzL7P7oXw6kYyG3fpTz8CXpYZFRZ\/8GRGVAQs5KtfPjx7pZCvZ5cubibuZfJViwSXNBCwXp9gId84EG8hX3cssh8Ixiok\/YJNNASsh3yN4mL3nnfNgZtVAmq0X7sXwqjAGH0+r9wnuIRfKT9hAwJGi+QPLORr8c6FHy7SSu7fv5\/zOSDx+UMPPaQrqxkO1ZB4HZnQypUrxzNVuR3gzobXRo4cye5qktvB4oWU7i1FwE\/kC6BARBXNAAAR90lEQVSQEwKVkLMq3qmHfI2CbrfC5TrNF+T58ssvs6vZF198wWXiER6MZDuR8vmuW7eOI9iQBxjl50G0uA\/l4QcPHkzFihVjd7W0tDSOnpPcDkbF1L77\/EYwZiLnR2wiJeXRQ75GcYl78lW5HeAOpipZIPhBT26H3bt3c5l5JM95+umnA+QLAu\/fvz+VL1+e30Pqyvz58\/uudLyZG9stfRndSG6Zv5Xz8CM2key\/Qr4WStS5c+do1qxZ7NOrarghyAJ\/w1wQqYYbrkU9tq5duwbMDqtWrWLyhdmhWrVqbJZ4++23sywdj9BkNFwfrj3wwAP81qeffhq4JNRreu7H3PAhY\/R+NYbcf\/Wa6MHfi\/hVqVKFQ9LNkD83PX+tGfupVeFDVLvAuauWDsUVsMdRWEEVuzXj+ZGOAO2NN96wkN30dW17hJt2WijjDtPDqVOnqFatWmyzBaGqMu\/qWkSrASwQ5Lx58zgAI5h8Ef8+btw4Gj16NKeoXLt2LZszIuV26NSpE1fPyKqpxUdAiGqhXgvXh\/ZahFYjBNro\/bGOL\/f\/hYCX8Ee0JxJQmSF\/blt\/mB\/m3flXxW5tO3r0KOflRoQqiuyimfX8H330EStlTjdHyRcPDw0Y6R+hEZYtW1Z3Podg8oUdGBFuIN8cOXIQIuhAxhBcpKzUppSU3A5Oi93V4\/vxq7VZKPsZm3AVMcTsYJb0hOlH5eDFJxtyMuzbt48PzPTkdAgmX3hMTJo0icaOHUtFihRhrRc\/6Oudd97xVOl4i2F3Zfd+JphYAfc7NqGqcQv5xio1WdwPjffFF1+kp556iitY4BANaSCnT59O\/\/jHP6K2+SK3A1JKTps2jbVonGgOHz6cPR8kt4OFCyldW45APJAv3M7gfqaaHvI1Crx4O6SnswsY6qzB\/lK7dm32UtDj7WAU9FD32b0Qft9IZq6N9PUXAn6XGVRDPpSekSnwQsjXQulHJQscpCEHL+qvTZw4kQ\/cNm\/ebKiShdGpCvkaRc7c+\/xOMLGgFQ\/YBPv96iFfo7jYvefDrb2jB244zfzuu++44OWNN95Ia9as4Ry8kdzMYhHk4HvtXgijAmPmM7uxL8El\/KrEAzbw+y1eKFcg6k3I1+Jdiig3uF7BXouGQzSUFUIpISRED9fgq4fE6ah2AbcyHLL99ttv1KFDh0y3FC9enBYsWEAoSQJzBrwcevTowbZlFNlEE\/K1eJGl+5gRiAfyTfr2V4L5QSVf10O+RoG1e8+7TvMF8SLB+ccff8yhwQUKFGATxBNPPMEkCnexUA1kDc+Ivn37cmVieDSsXLmSXcy0RTeXLFlCly5doqpVq0p4sVEplftcgUA8kC+AhulBHbwJ+VooekiqM2PGDHYtQ9XiSpUqcUAE8juAgMOR78GDB+mDDz6gPn36cPQLwoyff\/551oRV9Qu4neEwD1WQ4fsLzVpKx1u4mCZ0HS8EYwSqeMFGa3rQQ75GcYl7zVdLvtBeb7jhBo5gg6sZTAN6yghBe4YPLxLoICT52muv5RJCs2fPpho1anCkHA71EhMTpXS8kV1v4z1GN5KNU3RsqHjBRmt6EPK1UNygjSJkGIdujRs3pjfffJNuvfVW2rVrF4cY58mTJzB6qPBiaMDjx4\/ng7ohQ4YEbMTQepHpDEl20AfurV+\/vpSOt3AtpWtrEfBC6XizEEC+hzkPF6Fcp9IIRXErV65MRYsWNat7atSoEfeFgC6nm6PeDleuXGE7b+7cuWnTpk2czxcJY0Co4RpIG0ET0HiHDRvG2rI6PMM9qFQM2+9jjz3GXeAAD03Ci50WNRnfKALxovkCn+rjv6H6CQWoU6lTTJD16tXjQ3EzW9ybHQAmTATQdJFYRzUQJw7JtIdnWuDhE4ycDTh0g6eDtsFbAu917NiRbrvtNn4LhC6l480UXWv6iieCiRbBeMIGHg8wP3zcJldE8jWKS9yTL0KAQZTwdNB+siH\/btu2bTOZHbTCikAMhCSXKFEioPEi3R76Qho6HLzhb+WqpjRlKR0f7Za393qjG8neWTozWjxho+y+Qr4WyppKpj5w4EDOOuZUs\/tTMJ42klNr6rdx401m4HIGs0PdG86L2cEKYYbmC6+ENm3aXGU+sGK8cH0K+dqJtoxlBIF4I1+YHmDvBQGLzdeIxIS5B9nMVIkfRKgh6XnDhg3Z1QwtktnBxKlwV0K+ZiNqrL94I5hoUIo3bCav2E+TV\/xEM2odzpJ8jeJi954Pt9a2ezvAwwG11VRIcfDEsmXLxgSs9WCIRlCjvdbuhTAqMNE+l9euF1zCr1i8YaPsvkK+FuxieCygfhuS6iAQIlIuB+0UQNqIWps6dSoHVai8vfB8QPUK+P7CwwG+gWPGjOHyK1I63oJFlC5tQyDeyBfAKrvvoBbVxNXMLEmDrRfhwMjfW6dOHSZGmB+GDh0aNqRYO\/ahQ4c4CTuIFaaKFStWcOIcHNzhdfR73333cWJ1lBZq3rw5zZ8\/n8vKQ6tGEh5UzlAFM0XzNWtlpR+rEIhH8oW\/b9Fs6ZzrQfx8TZKsYC8H\/A\/SRESanlSScE2D5lyuXDmeEezHIHQQLopxIsEO+oEPMQI4UPHUTbkd7CZ7k5bN8m7ikWD0ghqPMoNDt427DlBS1\/JhydeozLgFT9ttviBbVcECiXHwP7RSaLJ68jkogYVWi+xnFStWpJkzZzLZolQ87MWff\/451axZkxPrRCodb\/anqt4NJdcJAoJAeARyV21Oh8s8TBXXjWBly+wWl+HFRsg3VG4HLAY0WnhLIC1lq1atOOoN7mvIEYFkPR9++CEnar\/nnnvC5nYwe1GlP0FAEBAE9CDgiObbvXt3QkL0UA222Hnz5oXVgrds2ULr16+nXr168e2qRPzjjz9OS5cuZQLWatQIVc6qdLwekOQaQUAQEATMRsB28o31AeDJgFBhhBDDtgvNd8OGDdS7d2\/2dACxI8k6wpCXLVvGCXbefffdsKXjY52PGfcj8XuVKlXYjp2cnMzVN\/AhhGxt2uxuZowlfXgfASgcOLRG1i9U+r7jjju8\/1Bx+ASeI1\/lagY7L\/6GzReHdciEBjsO7McQzoSEBLb5Riod7+Sao4goXO7wIYE0mMhHARs1Pki+\/vpr9nVGTTtpgoAWAXzDQ24TKBn4oEbxAfmQ9p6MeI58vQdx5BnjoLBp06Z8Iao3Q1vfs2dP4O\/IPcgV8YQAzHLNmjVjX3bt3\/GEgR+eVcjXBauoyBceG3v37qUHH3yQyRc+zP369XPBDGUKbkHg8uXL7Lf+yCOP8LmIkh3leumWeco8IiMg5BsZI11XKFc3BHWojQDzB8wgsFPDAwOHgSgAiq+JtWrVokmTJvHhoFbzxbWorgzyVX\/rmoBc5HkEsN6rVq2i\/v3787NAphYuXMhnFig4MGrUKC6JBVMDPqBhagMRq789D0CcPYCQrwkLDp\/jpKQk2r59O9tuQb7YOAgeuf3227l+HFzf8AM7NMKitU2Rr9bm+9VXX7EdD\/dL8zcCyHWCYCEcvMItEhVa0JTM4EwDUaAor4X3cJgMmy\/kDBVd4OmDD3Fp3kJAyNeE9dq9ezdXUcYGevrpp3lTwJ8ZgSOIuIOGolzi1P+hyLds2bLsLvfPf\/5TvB1MWBevdIGozR07dnCYPEhWka+2\/iAOlydOnEh33XUXlS9fnr0dEHwg3g5eWeWr5ynka9LaoYQRNJOuXbsy+cJsgAKhyCUBDRaFQvE+TA5inzMJdJ91g29Q0HZBvqFKYqEeIfKTqPqEPnv8uHscIV+TljwU+cI2BzsvgjyEfE0C2sfdBJMv\/NbxYQ5NF03I11+LL+Rr0noGk2+wmQH\/42sjDk1UfTmThpZufIKAlny1ZgakXcX\/MEOg9DkO3aR5HwEhX5PWMJh8ceCGLGs4MEOgBL5OYnPBJpwjRw6TRpVu\/ISAlnzVgdvGjRs5XSpswSBfmCSKFCnip8eO22cR8jVp6YPJF90q7RcHcpUqVaKRI0fy4Zs0QSAUAsHkq1zNFi9ezKYreD2I1usf2RHy9c9aypMIAoKAhxAQ8vXQYslUBQFBwD8ICPn6Zy3lSQQBQcBDCAj5emixZKqCgCDgHwSEfP2zlvIkgoAg4CEEhHw9tFgyVUFAEPAPAkK+\/llLeRJBQBDwEAJCvh5aLJmqICAI+AcBIV\/\/rKU8iSAgCHgIASFfDy2WTFUQEAT8g4CQr3\/W0jVPglzGPXv2pPPnz2dKHI9cxjVq1HDNPI1MBDk6UNj0zjvvjPr24NJQCCf++eef6dFHH83UF6qdIINZ+\/btpTBm1Ch75wYhX++slWdmCvJF+SQkk0edsVDtypUrnJsWP15pyNWxaNEi6tOnT6ByBLKN4QeEHKlpyRf3IOUoSDxUfmeQ\/Llz5whlqaT5EwEhX3+uq6NPFY58kXwIOWqRJObIkSNM0N988w29+eabXHYJFXlREidnzpyB2mW33HILlSpVih544AE6cOAAPxeSiWsTGZUuXZpJEVVAkDGuc+fOXAfvo48+4goRGCstLY1atWpFvXv3poyMDJo+fTqtXbuW8CGA0k4bNmyg+vXrcwY6kB6S4Hfv3p0TIqm2fPlyJlkQIpLdoO\/9+\/fToEGDeP4TJkygCxcucPIkfPCUKVOGS\/7gb7S6devyhxFqtKF6BUpO9ejRgz7++GN66623+Jp27dpRly5d6NSpUzR16lTOYpYvXz5H11MGtwYBIV9rcI3rXkG+IK5t27YFcECRR5gdxo0bR3fccQe1bduWtm7dyqk2UaE5e\/bsTKCFCxfmr9oqlSJIFmk4u3XrFpZ8oVHiqzr6RHVf1M4DWf\/444+UkpLCeZRBshi7b9++hPp4IEsQPYgZ12N+IOChQ4dyFRIUrgRpqtpoFy9e5H5QNfi2225j8lV9Q3t\/\/fXXqUOHDlzOfeXKlWxOwIcJxkQ2sptvvpnLTIGsQahbtmyh7777jjVffPjgmuuuu47mzp1L999\/P1+vHS+uBcqnDy\/k69OFdfKxstJ8taWWUHkXmrC2wVYM7RVaIpKIo8H+Ce03lObbsWNHfh9177QNxKWaKruDQqUNGzbkasCwp4JE0WACgKaJewYPHszkiYT3IGTVglOGgnyVFo7fIH\/UYAPhg9xRjw9a9OrVq2n48OFsXsF7K1asYM0XhS+rVavGzwVMlIkBz62S7SN\/L\/qQIqpOSrN1Ywv5Wodt3Pasl3xBhviKrsgR2iU0V5gEYAJQ5AuSvvXWW0OSL0gUxAuTgiIpECU0aZghtASJ8UBmIGtVngfEC+JUWmfJkiU56X2vXr2oePHiushX5W2Gplu7dm1C\/ubU1NSw5Is6fjNnzmQTCEwKmMPRo0fZRAHzA\/I+42BSyNffW0jI19\/r68jT6SVffG0HQcL2CzsvyBGaH0qpQ4uEFqo1O5w4cYJJCiYIVIseMmQIf63fvHkzEzOuh4fFpEmTuFApXg8m36ZNm9KaNWsof\/78bKaANjpv3jw2icAkAC0UBIrqEdqKIzBTTJkyhVq0aBEwO6i+YaZQ9fpwz7Rp03hcaOWYH8wXxYoVY7MKKg7DHg1bN57j+++\/p+TkZDZ34AMDHzSwYaNKsXY8RxZSBrUUASFfS+GNz871ki8IDXbQ4MOmS5cu0axZs+iTTz4h7YEbDrCgFYKAS5QoQbgO9mBcg+tXrVrFX+9h14UWir5DkS\/sytAqt2\/fzod\/6BMmAHz1B4GDFEN91ceBGxrMEVqzAw7ZZsyYQXi\/QoUKbLPF2DiAO3z4MB\/egexRhQLjYb6Yf61atdj2DFKGNg7yhQ14wIAB\/GwgcRzmSc0\/f+4jIV9\/rquvngpEB3OA1bZPaKUgPByIhSI8mBdwIAZyxKGglQ0HkfgQa926tZXDSN8OIiDk6yD4MrQ+BOwgX9hbR4wYwV\/\/GzduHHZiX3\/9Nb8Hjw2rGmzQb7\/9NrvMWU3yVj2D9BsZASHfyBjJFYKAICAImI6AkK\/pkEqHgoAgIAhERkDINzJGcoUgIAgIAqYjIORrOqTSoSAgCAgCkREQ8o2MkVwhCAgCgoDpCAj5mg6pdCgICAKCQGQEhHwjYyRXCAKCgCBgOgJCvqZDKh0KAoKAIBAZASHfyBjJFYKAICAImI6AkK\/pkEqHgoAgIAhERuD\/AJvhJndNTscHAAAAAElFTkSuQmCC","height":211,"width":351}}
%---
%[output:290f5cf8]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAV8AAADTCAYAAAAmsBCWAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQvcT1X2\/5eSqAjxL5I0BrmUkUiT0CBTSmpCTJFLLwldptyiCLlnGrehokZJVxWpXP+SZjRdXLooUsjklghFN\/1en137ee3nPN\/z\/Z7z\/Z7LPutZ5\/XywvPss\/dea6\/9Puuss\/faRX755ZdfSC7RgGhANCAaiFQDRQS+kepbGhMNiAZEA0oDAl8xBNGAaEA0EIMGBL4xKF2aFA2IBkQDAl+xAdGAaEA0EIMGBL4xKF2aFA2IBkQDAt8E28Dq1aupU6dONHr0aOrQoUPgkmzatIm6du1KV155JQ0YMCDw+s0KDx8+TAMHDqRt27bRzJkzqWzZsqG2l6ryp59+miZPnkyPPvooVatWLfL2s21QjxPu99N3G3RuyqzlmDBhAjVq1ChbdSTmPoFvYoaqYEejhu\/XX39N3bt3p8qVK9OYMWOoRIkSgWnPCQJUHFZbqTrtZ+L71UOY42TqbcqUKWpMvD64bIMvxmXs2LG0YMECXw+RwIww4ooEvi4KhxHMmDEj77dPPvmkehprg8UvrrrqKurRo4cq4\/Q+zfvr1q2b583piXjPPffQ2rVrlaHBs0wHM3hkgwYNUu1UrFgxzzCdkxpt4mc33XQT9enTh3SfNVi+\/PLLfH3VP69fv75qH5fpfe7duzfP80WdgOG6detUuVR9dvYn1eTOpFd4vuPHj6d+\/foVaAv1mX3o2bNnPo9c1w0dodyIESPyjYvunx5UrR896fF77XXrvmN8cOkx1A8Fpx7c+uZsU9tJur44TVLDXrep+wLQYrx0H03bMOtw3q\/1Zo6PthmnLTv1YI67lgHQf\/jhh9X8wBuSaW94Mzt48GC+Nxo3e0TbUb5tRczaAs0JfFOMgAk789eYrDB80+Cdv4cBOgFjwgoTCAbpvNxCB85JaoJg48aN+cIOqcBWvXr1fMAywXPKKacouAYFXz3J9SR0eojz58\/Pe4i46dUNvvfeey8NHz48DzT6fq03tzFzg535IIMenF622xj2799fPdhM+Kbr25lnnplvvNEf58+cD1VTN0746d\/BDgG9cePGpYWv2\/3oR5s2bVLasgnxVHrQ8HbaJn7ufEib\/cWDTT\/QtSPgfBBqm8HP4wo\/RQVlga9D03rw9+zZk+dh6okN47r11luVwb777rspf6+NT7+ao3oNa8AbF+CrPQgN41SepDlxtJemJ4NZlwaM\/p0JcrPv8Er0hIEs11xzjWf44l4vr9va+8bEMR8OLVu2VIDLpFcd89Ueptaj9ixT\/d+En\/MBaerGfJ019aL10LdvXxU7d3rs2sPUY+6Edbq+4Y1Cj7GXvjhj63q8tH047Uk7A26xcuf927dvzxvzdHoz9ajr1nrQ\/9fjm8obdtq39taXLFmiHsDant3kM+dXVDCMuh2Br0PjzldxGJz5NNbeBm7ToQLzHoQhevXqRc4nO8qnAmaq9nSXMnkBqcIOzniZCWvnRww\/YQev8DU\/Wr333nt5H7Agk+llu+nVDb5mfNn0cjHJtTdqekumbtw8PJTHQ6hp06YFPly6hUdwT7qHkLNvTvhm6osTvro+84FqPkC1M+AG31T3a\/tKFxYyQzIob3q5GqTONy+Uc7bnbAPhCTOcp\/vifLgIfKN+FFjQXlDwdVsh4AQmN\/hqeeDlIqatH1Kmx6UfWqkeam7wdXp8OpwCT9grfP0CKlPc1+3txuybG3y9ruqIG77m25R+cDg931RvWvpnqeCb7oNaJofDAkQE1gXxfLMMO8CAnKEAM+blFrPyA19tuNoLqFSpUsoQhtvrbCpPJNXrdvny5VV8TcfjnP\/XDxIvYQfzgyQm6XXXXade5b2Gc9zg63x1N19XzThsurADPC6nR2d6dVqPqeTUesP96eAKWZ2v0qnCDm59cc7soMIOzjgtxlSHHcwHgfmmpL8J6PHXD1ZtH+k830xhB7dvHPLBLTC2J7OiMD64ub2qpfN8TXiamnQadjr4Or8sox79QUXHLvXHI92G7qu52sEMO6B8uhUaWn\/Or+9e9OqEr27LLZyj++H2MS\/dBzcdCtKQ0TFf\/DzVh6ZUekP7mfpmfmR1++Cm++IMDaX74IYHpjMO61xm5lzpoMc4VUwX95rwdftYmy7s4Nae06acYTnThs1VJ8kkiLdei+froicvS6Lcluc4J2+65WGZ4OusK9WytXTwdcbrnJPcjOXp5W8agE74miBIB990MnnRqwkVcynepEmTVLwQ+pw6dSrNmjUr78On+VYAHemxSRUr1UOeztNNBT3nckOvfUN7iHcDOG4Pg3QbZdyWmgGWXtbqui3t8hLzNR+YkP\/1119XY5Dq+4XWazqbQp+d\/dFeuZ+11t7wZnepyOF76NAhGjp0KLVr1y7lLpbly5fnrZ2F6kzY2KBKL8ZuQz8LUx8yrQrxslsKkHnqqafYL28K2y7Sra4IemNO2LKEXX+k8EXe9ueee44GDx5Ms2fPTglfTAB4MY0bNw5b9qzqF\/hmpbbQb0oV0nDbdJCqM9rrMkMPoXeaYQNuYQfnhhiGovsWKVL4bt68mR555BH6+eef1RrTVB7Jgw8+SC1atKA6der4FkZuEA2IBkQDSdFAZPCFx4i4Vvv27WnhwoVqbaUTvt9\/\/70KSSDmiC\/8v\/vd79TX\/SZNmlCRIkWSolPpp2hANCAayKiByOA7b9482r17t1rUji2RqeB74MABeuCBB6ht27Yq1rtmzRoaOXIkjRo1imrWrJlPGIBZLtGAaEA04FcDy5Yto7POOsvvbYGXjwS+W7ZsUfvQEestU6aMWs6SCr5O6RAjhrdctWrVAikTAd\/PPvsscIXEXaHIFfcI+G9fxsy\/zuK8w5bxigS+qZLDQPnOIPznn3+uwg3YSVOsWDHS8IUX3Lp16wKer8A3ThP217YtBu+v195Kc5VN5PI2\/tmWigS+zs65eb47d+5UKemQNQqZtgBihCGwRKVKlSqFAr54ANnwSpStQbndx1UuyMtVNq5y2fJQiR2+emkKoIsPcO+\/\/74KNcBbrl27ttoC2aBBgwJz2hYFCqS8aYDrRBb4eht\/m0rZwo5Y4BvEQNiiwCBkMevgCimucgl8g54B4ddnCzsEvuGPta8WuEKKq1wCX1\/mbUVhgW+Ow2CLAnMUo8DtXCHFVS6Bb9AzIPz6bGGHeL7hj7WvFrhCiqtcAl9f5m1FYYFvjsNgiwJzFEM836AVGEN9XB8sXOWyhR3i+cYwWdM1ydXgucolnq9lE8hDdwS+HpSUrogtCsxRDPF8g1ZgDPVxfbBwlcsWdnj2fPft26fW3OIsLi\/XCy+84KVY1mVsUWDWArjcyNXgucolnm\/QMyD8+mxhh2f4YjMEMoxdffXVVLJkybQaOnjwIF122WWhatEWBQYtJFdIcZVL4Bv0DAi\/PlvY4Rm+8HyRHAdbf5EcJ92FspnK5KpiWxSYqxzO+7lCiqtcAt+gZ0D49dnCDs\/wxfE\/2PJ7wQUXZPR8w1cfqVy\/HBPrcIUUV7kEvlHM9mDbsIUdnuELb\/a2226jt956S51ce+2119L5559PRYsWDVYzHmuzRYEeu+u5GFdIcZVL4OvZtK0paAs7PMMXmjt69Cht3bqVXnrpJXr22WeVMq+\/\/nqVAhLnZUV52oQtCgzaorhCiqtcAt+gZ0D49dnCDl\/wNdXy008\/0caNG9WBmM8\/\/zydffbZ6rjuZs2aReIN26LAoE2FK6S4yiXwDXoGhF+fLezIGr5aRUh4\/uWXX9KIESNo165dkR29bYsCgzYVrpDiKpfAN+gZEH59trAja\/giBrx06VJ1BDwOvOzUqZM6e61atWp0zDHHhK5BWxQYtKBcIcVVLoFv0DMg\/PpsYYcv+GL97sqVK+mJJ56gtWvX0sUXX0w33HCD+vBWokSJ8LVmtGCLAoMWmiukuMol8A16BoRfny3s8AxffeLEkSNHqEuXLtS8eXMqX758+JpyacEWBQatAK6Q4iqXwDfoGRB+fbawwzN8v\/\/+exXTrVSpUiRhhUxDYIsCM\/XT7++5QoqrXAJfvxYef3lb2OEZvvB877vvPho6dCiVLVs2rQZRNlOZXIfAFgXmKofzfq6Q4iqXwDfoGRB+fbawwxd8b7\/9drWkLFN89\/Dhw3T33XeHqkVbFBi0kFwhxVUugW\/QMyDc+lZ9up\/aTlii3uAHtDqLOjY4LdwG09TuGb4IO6xfv57wt5ercePGXoplXUbgm7XqYrlR4BuL2nNqlOOYzX17J\/Weu0HpZf4t9ajx70vnpKNcbvYM31waCeNegW8YWg2vTo4TWWuLq2wc5RL4BjDHBb4BKDHCKjhOZIFvhAYUUFMC3wAUKfANQIkRViHwjVDZATXFcczGLtpCYxd9LmGHXGxE4JuL9qK\/l+NEFs83ejvKtUWBb64aJMnnG4AKI61C4BupugNpjOOYmfBdO+RCqly2eCC6yqaSrD64IbH6pEmT6Omnn1ZJzbGsDNuOu3XrFvoJFlpI8XyzGe747uE4kcXzjc+esm050fBFFrPp06fTnj176JprrqEZM2Yo+OLATKzvRcL1KBKsC3yzNb947hP4xqP3XFrlOGaJhq8+xRjAPf744\/N2vWH976hRo2j48OGBeL\/wrrGbrl27dtSoUaMCNiTwzWVaRX8vx4ksnm\/0dpRri1jjixUPuBIXdnCD788\/\/0yjR4+me+65J2f4wrtGkvbBgwerlJUC31xNLv77Bb7xj4HfHnAcszZT19CqzfuVKr6eeIlflQRa3nfMV4cdtm\/fTu3bt6eHH35YhRpee+01wukWffv2zTnssHnzZnrkkUcIQEdoQ+Ab6JjHUhnHiSyebyymlFOjGr740AbPN87LN3zRWcR2Z86cSZMnT6Yff\/xRwRZnuSH3Q6lSpXKSB3XDgwbYFy5cSE2bNhX45qRRO24W+NoxDn56wXHM\/jDyP7Tt6yPUuGppmt+7nh91BF42K\/jqXsDTRYL1E088kYoVKxZI5+bNm0e7d++mnj170rhx49LCFw0uW7YskHZtqQRvFEj6we3iKhfGiats3OS65MoOdODSsWpqJRK++BD2zDPP0DfffJOSD9WrV6eGDRtmlWh9y5YtNGXKFBXrLVOmDI0dO1Y8XyYU5uhFSdghWcaJjGZtpq1RnUZGswGtqsQqgG\/PF\/CdNm0azZ8\/n1q2bKnCDF999RUtXryYLr30UrUCAmt+cb6b32v16tXqLDjnBS94wIAB+X4sqx38ajfe8gLfePWfTevcxsz82BZ3RjOMh2\/4YknZhAkT6KqrrqI6derkjem6devo5Zdfprvuuos2bdqU73fZDDzuEc83W83Zdx+3iWxqmKtsnORCnBfxXlzHfPcVfTW9XeyTxDd83U60MH8OqYI4yULgG7t9BNYBThPZqRSusnGSy1zfe8J7s2j7iscDs+1sK\/INX3i+2EjRokULatasGRUpUoSw\/AyhBnwsu\/\/++wlLxerWrZttnzzdJ2EHT2qyphCniSzwtcasPHXETCOJJWYHZv2VPvvsM0\/3hlnIN3zRGXR80KBBKtaLmC9ONMbmC4QjTj31VLXu95VXXgmz3yqnhA0KDFpIrpDiKhfGn6tsHORygnfKdTWp86XnWcGOrOALg8MyM8APy8Kw1KxatWp00kknqZ9jcwQ+vIV5CXzD1G7wdXOYyG5a4SpbkuVCjBfg1bl7MXZ6hYMt7MgKvggzYH0vQKsvLD178MEHadiwYTlvL\/Yy9W1RoJe++imTZINPJydXucTz9WPd0ZQFeLGkDH\/ry1xaZgs7soIvYrsDBw7MB9+SJUvSjTfeSLfcckvoXi8UaosCgzYnrpDiKpfAN+gZkH19gG2fuRvycjegJsR4OzaokG9Nry3s8A1frPPFJoibbrpJaWnFihUKuNh4gfjv5Zdfnr32fNxpiwJ9dNlTUa6Q4iqXwNeTWYdWSIcX3vx0Xz7ookEcCw+P15kw3RZ2+IavuaTs22+\/pVmzZlH\/\/v1p\/\/79ancaPsQh9hv2ZYsCg5aTK6S4yiXwDXoGZK4PwH1z836a+98dBYCrvV18WHM7Ft4WdviGLzxfxHVvuOEGqly5slpahhAEEuzg31iGFsQa30xDYIsCM\/XT7++5QoqrXAJfvxbuvzxgq4GbysPVwL2oamkVYnCDrm7ZFnb4hi8EWLVqlUqcjg9sSGyD3Ls\/\/PAD\/fnPf1bbgOUkC\/8Gpu\/gCimucgl8s7d1tzs1aFd9uo+++PpISu9WA7dymeLUsWEFFWLweiUavhASy8lwYcXDRx99pP5dq1atSD62oS1bFOh1wL2W4woprnIJfL1adupy2qv9Yt8RAmwRTjBXKTjvQvwWHm7j35fxBVyzHlvY4dvzRdjh1VdfpcsuuyxfbBc\/RzKdVq1aUYkSJXIbEQ9326JAD131VYQrpLjKJfD1Zt4aqICr9mi37fs1nJDuAmzh3V70+zK\/Qbe0twbTlLKFHZ7hi23F69evV7vacLRP586dCcvL9IUPcThE84EHHpCYbw7mwRVSXOUS+OY3didk8dt0oYN0nu0ZZYpnjN9mM9USB199dht2tekky2YC9eLFi1PXrl1Vmknkewj7skWBQcvJFVJc5SqM8DUBC\/n9eLJ6vujlXzqEEBZoU81PW9jh2fPVQriFHYKGUKb6bFFgpn76\/T1XSHGViyt8Adj\/fvQ5\/Xh8GWXC2QDWBC0giwux2ihBywK+R48eVadXYGux2wWP9+STT6ZjjjnGL3N8lxf4+lZZrDcIfGNVf4HGzQ9dJlzVv3873ddPj01PtnLZEmpjQ9yQdeu\/Lezw7Pkiptu9e3dC0nS3C2kkcbCmrPP1Y7b5y3KFFFe5bPR8dVgAf2MVwa+gPaxir14+crlZrwlYW7zYbGZa4uCbjZBh3mOLAoOWkSukuMoVNXzTgTVbr9UMD+DfWF1wRtniVOPkn9RZjLZ6sNnOPVvY4dnzNQVF6GHjxo20ZMkS2rNnjzptF0vPojx11xYFZmsAbvdxhRRXuYKErzMUYHqsuYJVQbVscWV2Gq6Iv+ICXNWSrt9+r22T65jZwo6s4Kt3uOHAzDPPPJPeeecdlWAHx\/40btw4aB6lrM8WBQYtLFeD5yqXF\/hmgmouoQA3r9UMC\/z6b\/\/rY7mOmS3s8A1fndsB6SPNAzQ\/+OADeuyxx1TeB0mskz2SuRo8N7nM1\/\/3Nn6hXs+dnmoQUE3lsQYBVi8Wym3MtMyJha+XAzTlg5sX005dhqvBJ0EuE6gYHfNjlfp\/jh+snCPuDANoqKYLBWRvWf7vTMKY+ZfKntQEvj3fw4cP07333kvt27enBg0a5Mkunm82ZlDwHq4GH5dcUQPVzVM1l19lGwYIxsK81xLXmHnvYXYlE+v5Qtx3332X7rrrLjr33HOpSpUqasuxxHyzMwTnXVwNPki5nF\/8oUP9yh+Gh5oproqPzudVPyPlR6tgrCKeWoIcs3gkSN1qouELkRB+AHC3bt1Kp5xyCjVv3pxOP\/30yHRsiwKDFpirwaeTK5V3Cr1iV5WGqQKsh0Qsfscj1au\/00tNtRLAbKcwjplfPdtU3hZ2+A474OBMHAuPVQ0VK1aMJI9DqoGzRYFBGxWHiWxmqtIL\/eEdHj62ZN5i\/7Bg6vW1PxNQ\/YwrhzFLJS9XuWxhh2\/4YrXD9OnT6fHHH6ezzz6bOnTooJLpmBnO\/BhutmVtUWC2\/Xe7z0aDd\/NMw\/q679SN6Z3id9gAoL1T9f8yv61fTbFWNejxKUyQstEWgxhPW9jhG75aeCRRx0aL559\/nl577TXC1uIuXbqoj3CS2yF7E4nC4FPFTKN6zdeaCeJ1P3stB3tnFGMWbI+91cZVrsTDVw8fdrsh7vv3v\/+dtm3bJrkdvNm1a6lsDN7LB6gwX\/Mzverj98d9v48a1ip4kmyO6rLi9mzGzIqOZ+gEV7kSDV9kOANwX3rpJVqwYIEaQoQf2rRpQ6eeemokcWBbFBj0JILBH3tyBVWtjpfqf+vkKFGBVEH1t33+vwL212xV+lXfT9yU60SGLrjKxlUuW9jhO+yApOq9evWiDRs2UKdOnaht27ZUrVq1jKEGJGEfMmQIrV69mmrXrq3WCpvrhDXEli9fTj169MhjmlumNFsU6BW+qbxT82t+GF\/y073i43fm3n7tvTr393uVL1M5rhNZ4Jtp5O37vS3s8A1fbLLYvHmz+tjm9ZRifKQbPHiwAnWTJk3U6cczZsygSZMmUbly5fKNzlNPPaUS9GTKEWGLAnXn3bL74\/fZ5EdNZ7Lp4qXaKw0bpn6nlMDXr8biL891zGxhh2\/4ZmMSGMSJEyfS\/fffT6VKlVJrhO+88066++67lddsXjiOvkWLFvnyRqRqM04FpjoMMBfAmjAtU+wnlRsjl1f8bMYo7Hu4TmTxfMO2nODrj5MdpjSRwNdsEB\/oVq5cSXPmzFFZ0MqU+TWtHS4c0jl06FD14Q676KCkgQMHKm\/ZeS5c1AoEcHHy6tz\/7vDsyTo9VC+L97lCiqtcAt\/g4Rh2jVGzw02eSOG7adMmdcjm3r17adCgQSpmbIYuDhw4oE4\/RngCsd41a9bQyJEjadSoUVSzZs18MkCBuJYtWxbaWH154Cd6939HaMGGQ+pvt6tiqaJUoWRRwt\/1Ty+u\/tb\/99s5fTip3\/tsL89VLuidq2zc5MIuXH3hG1Tcl2\/4wjv9+OOPVcz3+OOPz+e1QiCEETLFgjGo\/fv3p549e1LTpk1ddQAvefTo0VS1alW1msK8wnx6wcud+\/ZOGrvo8wJ9U1\/4f1sB0LFBhazypKYbdK4eIle5xPONG2H+2w+THX564xm++gDN\/fv30\/jx46lfv37qsEx9IbkO4rrwUp0pJeHBLl68mO644w7Sx80j5ICkPCZUMUERbsCSNZTT8IUX3Lp169Dh6wZdAFcfcd2xwWl+9Ou7LFdIcZVL4OvbxGO\/IXHwzXSA5nHHHUd9+vRRy9Ccnu+WLVtU7BZLzWrVqqXWRd53330q9GCGE3bu3EkDBgxQ9dSvX1+BGGGIMWPGKFCH6fkCvG2mrVFra\/UF6MK7HdAqf9thWg9XSHGVS+Ab5mwIp+7EwVerAet8p0yZogBpfixLpyb9kQ0Q\/eSTT6hGjRp5H9JQH05FBnQbNWpE77\/\/vgo1ZFoPHKQCxy7aki\/EEAd0tf64QoqrXALfcAAZZq1BsiOXfvoKO2C97gknnEDIbAagOi+sSEAoIkm5HZzgRVhhQKv4tsFyhRRXuQS+ueAnnnsTB199fNCtt96qEqmvW7eugObcdqOFoeIgFGiCN05v19QPV0hxlUvgG8bsDrfOINgRRA89e75BNBZkHbkq0AneKdfVDHzlQjbycoUUV7kEvtlYebz35MqOoHpfKOGLZWS9525QOoTHawt4ZSIHZdbR1sP1wcJVrsTCN92qhySEHbCa4Q8j\/2MleAW+0UIzqNa4QoqrXImFr9Ng8eENO9aQXhIf2\/7yl79Ym1IS4O0zd0Pe9mB8WItyGZmXyc7V4LnKJQ9ML1ZtVxk28NVqxUqIqVOnqnW+SJ4T9pWNAs1wQ+OqpWl+73phd9N3\/VwhxVUuga9vE4\/9hmzYEUanA4v5Yr3uhAkT1EoIr+t\/cxHIrwKd4Yb5t9TLSwyeSz+CvpcrpLjKJfANegaEX59fdoTVI9\/wBWSRCB35GcwL24uvvfZa6t27d8bcDkEI41eB+MAGzxeXjeEGrROukOIql8A3iNkcbR1+2RFW73zDF4l11q9fr9I\/mheOD0ICnCg2WKBdPwp0er1rh1wYlj5zrpcrpLjKJfDN2eQjr8APO8LsnG\/4ojP4yLZr1y6V1Qwhhh07dqgE4FEeH+9HgabXi3BD49+XDlOnOdXNFVJc5RL45mTusdzshx1hdjAr+M6bN49GjBhBs2bNonr16qlDNJG3AZnKMh3\/E5QwXhWYJK9XJnJQ1hFtPVwfLFzl8sqOsK3IN3yR8BzH\/\/Tt21clyNEXths\/\/vjjNGzYMOUFh315VaC5k21qx5oUdkrIXOXmavBc5ZIHZq4WH\/39XtkRds98w1fneMBxP2beXrefhyWAFwUmzeuViRyWtYRbL9cHC1e5vLAjXIv5tXbf8P3hhx9ULt5zzz2X2rVrpz6wIQa8YsUKWrhwoQpHlChRIvS+e1Ggua7X5hUOprK4GjxXueSBGfpUD7wBL+wIvNEUFfqGL+rAcUFIhI6\/ccw7lpkBuMjzW7169Sj67Wm1Q5upa9RuNtvyN6RTEFdIcZVL4BvJdA+0kUTDF5r46aefFHx3796tVjngRAp9RFCgmnKpLJMCzZCDrbvZUonGFVJc5RL4RjHbg20jEzuCbc29Nt+e7549e2j48OFqo0X58uWj6meBdjIp0Aw52L68TMIOsZlRIA1zfbBwlSsTOwIxCg+V+IYvPN7p06dT7dq1qVmzZpEk0UklRyYFmiEHmzdVOGXjavBc5RLP1wNlLCuSiR1Rddc3fLG9ePDgwbRs2TIV7zWT6OD\/8Irjzu2Q1JCDTOSozD7Ydrg+WLjKlVj4um0vhjljxxtWQeDvsK90CkxqyEHgG7bVhFM\/V0hxlSux8A3HfP3Xmk6BSQ05CHz924ENd3CFFFe5EgdffWT89ddfTxMnTiyQ1QyTwIawQ5JDDgJfG1Dqvw9cIcVVrsTBF+GGjz\/+mM466yz65JNPCmQ1syXssOrT\/dRm2ho1g5KysUJWO\/gHnk13cIUUV7kSB1\/T2G3OambmckjSEjOtX64Gz1UueVux6THorS+Jhq\/NWc2SHO+Viext8thWiuuDhatciYWvzVnNkh7vFfjahlVv\/eEKKa5yJRa+Nmc1S3q8V+DrDXa2leIKKa5yJRa+Nmc1S3q8V+BrG1a99YcrpLjKlVj4whzDzGqGuocMGUKrV69WW5iRQ6JBgwYFZkEqBSY93ivw9QY720pxhRRXuRINXxh\/GFnNDh06pLYut23blpo0aUKrVq2qIMepAAAUM0lEQVSiGTNm0KRJk6hcuXL55pxTgRzivQJf27DqrT9cIcVVrsTD15tZ+iuFwcYGjvvvv1\/ljEB8+c4771THFlWrVs0zfJO4vleWmvmzFZtKc4UUV7kEvhlmD9YSr1y5kubMmaMO5nQm63EqkEO8Vzxfm5DqvS9cIcVVLoFvGtvetGkTde3alfbu3atOzOjUqRMVLVq0gOeLHyC7Gq5hS7+iBRsOqX8v6FKJKpbKX977VIq35Pbt29U2bW4XV7kwTlxl4yZX8+bN86YVvi3FfflOKRllhzH4\/fv3p549e1LTpk3Thh04fGwTzzdK6wquLa4eIle5Eu35fvvtt\/TGG2\/Qhg0b8lnwySefTO3bt8\/66Pg1a9bQ4sWL6Y477sg7kgghhypVqlCHDh1c4cvlY5vANzggRlkTV0hxlSux8MU635EjR6rkOg0bNlSnF+srV\/hu2bKFBg4cqJaa1apVizD4OCkZoQecEWdepgI5bK7QsnE1eK5yyQMzysdcMG0lFr5YgYC1t1gSVqFChWC08Vst+iPbmDFjFNxr1KihYIxlZ0WKFPEE3yQm0zEF4woprnIJfANFQCSVJRa+WIs7fvx4uvnmmwOHrx\/NmwrkstJBJrIfC7CnLNcHC1e5Egvfo0eP0tKlS9UONKxIOPHEE\/NmAbxThB7MUERYU8RUIJePbQLfsKwl3Hq5QoqrXImFL8IO3bt3p3Xr1hWw6Lp169LMmTOpbNmy4Vo7EZkK\/MPI\/xA+ujWuWprm964XetthNsDV4LnKJQ\/MMGdDOHUnFr7hqMN\/rVqBnFY6yET2bwc23MH1wcJVrkTDFx\/GPv30U1qxYgUde+yx1LJlS5XrAUvCnB\/GwpocWoGcVjoIfMOylnDr5QoprnIlGr5IeDNs2DC11ExvhMDys86dO9Pll18erqX\/VrtWIKePbQLfSEwn8Ea4QoqrXImFL1Y7ALw33ngjVaxYUa3DHTp0KO3atYumT5+ukuKcdNJJgRu4s8JU8F075EKqXLZ46G2H2QBXg+cqlzwww5wN4dSdWPiaJ1lANRq+5r+j\/ODGaaWDTORwJlvYtXJ9sHCVK7HwxRHyw4cPpxYtWtA555xDI0aMUJ7vF198QXPnzlX\/LlGiRNj2nrfaQa90gMcLzzfpF1eD5yqXPDCTN+MSC1+oeuPGjXTbbbcRthrjQM0zzjiD9u3bRxMmTKD69etHMhpQ4Ip3PiLAFxeHZWYykSMxncAb4fpg4SpXouEL60Vynbfeeos+\/PBD5YVeeOGFkazv1TPHCd8kJ1A3acDV4LnKJQ\/MwJ9loVeYePhCQ\/B8AeGodrWZowIFzl78HrWZtkb9eGrHmtSxwWmhD1zYDXCFFFe5BL5hz4jg6080fPfs2aPivosWLVLre4877jj661\/\/Sn379i1w4kTwqvu1Riiw5z+X09hFn6v\/Jz2hjtYTV0hxlUvgG9YMD6\/exMJXp5QsXrw49erVi0qXLq1OnHjooYcImy8GDBhQ4NSJMNQIBV52\/0Ka+\/ZOVT2HZWYykcOwlPDr5Ppg4SpXYuGLpWZY0YC0kuXLl8+z7B07dqg1vvCIo1pqVufO52nV5v1qbS+HlQ4C3\/BBGUYLXCHFVa7EwhebLMaNG6eO9jn99NPzwRebLPr16xfZJotS3eawSagjYYcwsBhNnVwhxVWuxMIXKSXfffddevLJJ+maa66hs88+m7Zt20aPPfaY2lp8wQUXKIsP2\/utUqchHbh0rGqLyzIz8XyjgWXQrXCFFFe5EgtfrOdFyAE5HdJdL7zwQtA2nq8+E75clpkJfEM1mdAq5woprnIlFr6hWbDPiis3vIwONe6v7uKyzEzg69MILCnOFVJc5Uo0fLG87NVXX6UnnnhC7WxD7Ldbt2500UUXRXKKBeZcpWY30HfndVPTj8syM4GvJTT12Q2ukOIqV6LhO2\/ePHrllVeoS5cudNppp6ldbrNnz6brr79exYGjuCpefgcdObuNwDcKZQfQBteJLA\/MAIwj4ioSC1\/kcsDR7n\/7299U8nR94dj3iRMnqmPlS5UqFbo6\/1\/n6fRTuRqqna8nXhJ6e1E1wBVSXOUS+EY1M4JrJ7HwNVNKmisa3H4enMry16Thy2mNr0zksKwl3Hq5Pli4ypVY+GKH26hRo+i8886jK664QsV4sbMNRwrhVGOshDj++OPDtXYiKnfzs3T0hHKslpkJfEM3m1Aa4AoprnIlFr6wXuTuxTZi\/F2uXDmVVhJnuU2ZMoWqV68eioE7Ky37t\/+vfsRpja\/ANxLTCbwRrpDiKlei4QvrxYqHzz77jHbv3k0lS5akmjVrUrFixQI37FQVmicWc1rjK\/CNxHwCb4QrpLjKlXj4Bm7BPirkdmKxKTpXg+cqlzwwfUxcS4oKfHMYCBO+nNb4ykTOwShivJXrg4WrXALfHCYLt+PixfPNwRgsuJUrpLjKVejgu3PnTpVyEgnYkYry7rvvVol4ihQpkm\/6LF++nHr06JH3s7p169LMmTPzJeox4cslj68WmKvBc5VL3lYsePr57EKhgq8+8fjcc8+ldu3a0datW2nQoEHqD+BqXk899RRVqlSJGjdu7KpSfVw8CnDaYCET2ecssqQ41wcLV7kKFXyxAQPrfwcPHkwVKlRQU2bs2LFqh1yHDh3yTaEHH3xQHUtfp06djPDltsFC4GsJTX12gyukuMpVqOCLZWnfffedSrKOTRkHDx6kO++8k6677jr605\/+lGfq8JBxSgbyAyNnMJQ0cOBAatKkSb7wBI6Lx3Izbmt8Bb4+qWdJca6Q4ipXoYKvOUdw+OaYMWPUj+677758p15gs8YDDzxAbdu2VeGINWvWqFwR2FGHdcT60hssin71Cb11XytLpmAw3UCeZIRduF1c5cI4cZWNm1zNmzfPm1bYoxD3VeQX7A2O4IL3++KLL9LkyZOpc+fOKgNapm3I6Nro0aOpatWqeeEJc4MFjopHLl9OF1dvg6tc8raSvNlXqDxfQPS5556jJUuWqAM2kYYy1YUJinBDmzZt1G45DV94wa1bt1a3cN7dBvlsMYygpxRXuWTMgraU8OuzxRYj8XyxzAzQxQc389BNp5pRDjkj+vTpQ\/Xr11cgRhgCYQqdvtKE7wnvzaJi294Mf7SkBdGAaICVBgpN2OGDDz6grl270t69e\/MNIEIKLVu2pO7duyvoNmrUiN5\/\/30Vali9ejXVrl1brZJo0KBBgYEHhLHaQS7RgGhANJBEDUTi+SZRMdJn0YBoQDQQpgYEvmFqV+oWDYgGRAMuGhD4immIBkQDooEYNCDwjUHp0qRoQDQgGhD4ig2IBkQDooEYNCDwjUHp0qRoQDQgGkgUfLHp4pVXXlHbjQ8dOkQ333wz3XjjjVSiRInEjKRXGY4ePUoLFy5Ua5yxJbtVq1Zq2R3Scdp6eZXN7P+qVavopZdeUuvAbR1HP3KtW7eOhgwZQh9++KFaOont8VjUb+vlVTbkXXniiSdo0qRJdOTIEZUOFmlhbbZHN52DHcOGDaOePXtStWrVYhuaRMF3w4YNNG7cOGXQpUuXphEjRtBFF11EV155ZWwK9NuwVxmQ1wIZ3vCgwSGlTz75JGEHICZ2VGflhSWbrhfn\/91666106qmnqoeMrfD1OmbYJISNRHfccYfKRYL0qFu2bFFr2IsWLepXnZGU9yrb66+\/rnapIic30gLANiETEmQl5YJDg30EL7\/8Mi1evJgeeeQRga\/XwZs1a5Yq2q1bN\/U3NmKYBuG1njjLeZXh6aefpm+\/\/TZP1k2bNikQY8df2bJl4xTBtW2vsqEC5PqYPn26qktDy1b4epULbyrYOYUdmjgkQGfzwwGzzkMDbBlAr7Jp+OIN5YQTTlDwRXpY5GlJygWP95lnnlH29uabbyoZxPP1MHowZHi6TZs2zUtDmQQgmaL5kQGGcuyxx+Z5gzjhA57UxIkT82WC86C6SIr4kU0\/OFesWEFXXXUVzZkzR3mMNsLXj1x4MOLCtvh33nlHhYogl1suk0gGJk0jfmRD2AE5uB977DFVIw47gC3irSxpF\/KLw2NH2ETg62H0Dh8+rHL7duzYUcXScCUNvtnIgFelpUuXqi3XePikO+HDgxpDK+JHNsSw4cXfdttt9OOPP9K\/\/vUva+HrRy7ACdBFaOyMM84gvL189NFHKlafKYNfaAOTpmI\/suFbC\/4g3zbyck+bNk3Ffm0OqbiJLvD1aW0\/\/PCDMuIrrrgiD0CALwwdf8qUKeOzxuiL+5Xhf\/\/7n8p5rD8QVK9ePfpOe2zRq2z4wDNjxgyqXLmy+miDMbQZvl7lgpoA3zPPPFMdEoALMXp8nxg\/fryVoSKvsulyF198cV52wY0bN6p5B9mSMPdMMxb4epzUZrEpU6aoeJMZ812wYIF6Gtv6EcopplcZ8DGqX79+dPXVV6sUmzgBxPbLi2za28K4mVfFihXp0UcfjfU10E2\/XuTCvc74KeCLNxZA2VZAeZFNhycaNmyYD76ImeJDaalSpWw3zXz9E\/hmMVzml1l8xIBXCDAhDpyUy6sMs2fPVvmM8UHD1o81Tp17lc28z3bPF331KhfKAbawSzxMNIyxJNLWMfQqmxl2AGw1tG2WTcIOAVLRuSYRMUPAydZlPKlETycDPCRc+BiA+O7jjz+erwoklZ85c6aVr7DoqBfZECNMGny9yoVyK1euVN4gVj3ANvv27Wu1Z+hVNni\/8+fPV6ttvvrqK+rSpYtaJoj4b9Iu8XyTNmLSX9GAaEA0EKAGErXJIkC5pSrRgGhANBCrBgS+sapfGhcNiAYKqwYEvoV15EVu0YBoIFYNCHxjVb80LhoQDRRWDQh8C+vIi9yiAdFArBoQ+MaqfmlcNCAaKKwaEPgW1pEXuUUDooFYNSDwjVX9djSOBDA672ymHvkpm6ku8\/fYiotF+0g87ie3LzamVKlShTp06OCnuVDLItVpp06d1C63sLZMY2dg165dlRxhtRGqkqRyEviKEajsW3HDF8CaO3euL\/Bi6IKAL3Z54U9Q+TMgC\/LfOnfzBW1qyJOB5ObYbRZnasSg5Sos9Ql8C8lIIzMajoABaJEKUG8P\/eCDD5SXhgsngiA3wdtvv00TJkxQR+GcddZZ1Lt3b5VDA+kSzbLwUJGYGvcgN2\/VqlVV2s8mTZqkzGWAUwRQFnCqUaNGXtm33nqrQL1mbl9z2+7mzZtVSlEcAwMvGfBFsiVk2Vq0aBGdf\/75eUf36C2xU6dOVRnGateuTXfddZfq36effqrKIS8IEvL\/4x\/\/UMnBoSNkWUPdSF+KenWuYZwuApmhH\/Rh0KBBdM455xSwIBO+OpHQH\/\/4R1q2bJnSU4sWLVSdp59+eoF7nQ+TdA8lgW+yJ6\/AN9nj57n3OGXh3\/\/+t0rLics8icD0fLFvHx4b8kvUqlVLweeee+5R0AGIzbLIy4tyeP0FxLZu3aqAhPvr1auXr2\/wrAFmlAcg8X+zbDrIAHqAEsCNlI3PPvusAjhyAgOWOocufodk3+gzTlxAf5D2EKBGCACnFzz88MNKdsiJfvfq1UuFLJC4HidrfPfdd3TLLbeoPMO47+eff1ayb9++XSXfRgY96AUJ05HnAL9D2MO8UsF3165dqjyOTIIO33jjDfWAc+ZGEPh6NunEFxT4Jn4IvQkA+AJMgC\/OFzOTEZlA1UffAAr4NyAJ4ABmeLU1y6JOvF4jCZBOFu5MUah798ILL9DatWsVyHXbyPq1b98+BeR08AW0kJJRpxI9cOAAwQMGBAFSHOJophnV4QsAFV4+MuDhbxxuif4B2Hv37lXZx5CPFh4v+oEUnv379yedNxl9gryAPBIawQPXRwTpHLeXXnpp3skqWlY3z1fHpXfs2KHagl6d4QKBrzd75lBK4MthFD3IgGNg4G0hVSUghINHAT2ECkyg4lUWkMZJtTgiBnBA2kF93pVZFv+G9+q8cCqsM96ZKjZr1uUGX\/O1PdVHtXSwAnAnT56swhHwfCtVqkRffvkl\/fOf\/1TwNc\/EwwcshCHgjeoTec10lwA2ksA7L3jjzn6lgq95Aku6rFoCXw\/GzKSIwJfJQGYS45NPPqFTTjlFARUeLZKZ49h2AOjFF1\/M++AGbxZHucPbhMfoBIUJzFTerPPsOd0vnD+3bds25fEhty28SHid6A+8Vjf46kTe8Nb1CRGIM+NopbZt2xLiueZqB7MeeKvoP8IF8LbNY6ec8EUIBV4v3gwQXsEF\/SDEgdACjs0xvW\/0\/+DBg1S8ePECifxTwRcecuvWrVW98HzxcILnrdvSenLCF+OBk3ZTrQCRmG8mq7f79wJfu8cnsN7B44XXhxzIxx13nPJ2AWQcRY8JDu8WXiySZgO+ACNOBwF88AEK3iJexwFcXRYxVTOOi2OP8CHp9ttvLxDzxT2AIEIU+PD18ccfq\/8D\/gBrurADQhvwOjWsEa4AVAEwPCTSwRdQRTsIEwCgaOehhx6i\/fv35\/N88WaA0ApOhkZoAaEN\/B8rIAA+9FfHndEePkbi5\/CWvcR8UT9k14nI0f9UZ7shLKLDGzi9GmOi++A8YFTgG9j0iKUigW8sao++Ub3aASDF5DZP1sVKAIQKAEHAGB+l9Bd\/eKrr169XsV14kt98801eWcAHqwZSrWBwntyANrFKAHFOQN9ceYCy6eDrTOR97bXXKsAjPJDuNR0yA6B6FQTuwdHh+JjWo0cPdfou4A3g4gKodXmsxoCO4C1DTni4OlE6+u93tUP9+vXptddeyzvVGOBF\/wHh7t27qwcJ6sTxUXhYYEywGgJH9+g+ALZmWYFv9PMoyBYFvkFqU+pKrAZShREQdnj55ZcVkP2cEZgp5huUkgS+QWkynnoEvvHoXVq1TAN69QLW7eLjmD4x+pJLLlHrn\/1cAl8\/2iq8Zf8P2sPQ\/nEYkNkAAAAASUVORK5CYII=","height":211,"width":351}}
%---
%[output:9a782e7f]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAV8AAADTCAYAAAAmsBCWAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQvcT1X2\/5eSqAjxL5I0BrmUkUiT0CBTSmpCTJFLLwldptyiCLlnGrehokZJVxWpXP+SZjRdXLooUsjklghFN\/1en137ee3nPN\/z\/Z7z\/Z7LPutZ5\/XywvPss\/dea6\/9Puuss\/faRX755ZdfSC7RgGhANCAaiFQDRQS+kepbGhMNiAZEA0oDAl8xBNGAaEA0EIMGBL4xKF2aFA2IBkQDAl+xAdGAaEA0EIMGBL4xKF2aFA2IBkQDAt8E28Dq1aupU6dONHr0aOrQoUPgkmzatIm6du1KV155JQ0YMCDw+s0KDx8+TAMHDqRt27bRzJkzqWzZsqG2l6ryp59+miZPnkyPPvooVatWLfL2s21QjxPu99N3G3RuyqzlmDBhAjVq1ChbdSTmPoFvYoaqYEejhu\/XX39N3bt3p8qVK9OYMWOoRIkSgWnPCQJUHFZbqTrtZ+L71UOY42TqbcqUKWpMvD64bIMvxmXs2LG0YMECXw+RwIww4ooEvi4KhxHMmDEj77dPPvmkehprg8UvrrrqKurRo4cq4\/Q+zfvr1q2b583piXjPPffQ2rVrlaHBs0wHM3hkgwYNUu1UrFgxzzCdkxpt4mc33XQT9enTh3SfNVi+\/PLLfH3VP69fv75qH5fpfe7duzfP80WdgOG6detUuVR9dvYn1eTOpFd4vuPHj6d+\/foVaAv1mX3o2bNnPo9c1w0dodyIESPyjYvunx5UrR896fF77XXrvmN8cOkx1A8Fpx7c+uZsU9tJur44TVLDXrep+wLQYrx0H03bMOtw3q\/1Zo6PthmnLTv1YI67lgHQf\/jhh9X8wBuSaW94Mzt48GC+Nxo3e0TbUb5tRczaAs0JfFOMgAk789eYrDB80+Cdv4cBOgFjwgoTCAbpvNxCB85JaoJg48aN+cIOqcBWvXr1fMAywXPKKacouAYFXz3J9SR0eojz58\/Pe4i46dUNvvfeey8NHz48DzT6fq03tzFzg535IIMenF622xj2799fPdhM+Kbr25lnnplvvNEf58+cD1VTN0746d\/BDgG9cePGpYWv2\/3oR5s2bVLasgnxVHrQ8HbaJn7ufEib\/cWDTT\/QtSPgfBBqm8HP4wo\/RQVlga9D03rw9+zZk+dh6okN47r11luVwb777rspf6+NT7+ao3oNa8AbF+CrPQgN41SepDlxtJemJ4NZlwaM\/p0JcrPv8Er0hIEs11xzjWf44l4vr9va+8bEMR8OLVu2VIDLpFcd89Ueptaj9ixT\/d+En\/MBaerGfJ019aL10LdvXxU7d3rs2sPUY+6Edbq+4Y1Cj7GXvjhj63q8tH047Uk7A26xcuf927dvzxvzdHoz9ajr1nrQ\/9fjm8obdtq39taXLFmiHsDant3kM+dXVDCMuh2Br0PjzldxGJz5NNbeBm7ToQLzHoQhevXqRc4nO8qnAmaq9nSXMnkBqcIOzniZCWvnRww\/YQev8DU\/Wr333nt5H7Agk+llu+nVDb5mfNn0cjHJtTdqekumbtw8PJTHQ6hp06YFPly6hUdwT7qHkLNvTvhm6osTvro+84FqPkC1M+AG31T3a\/tKFxYyQzIob3q5GqTONy+Uc7bnbAPhCTOcp\/vifLgIfKN+FFjQXlDwdVsh4AQmN\/hqeeDlIqatH1Kmx6UfWqkeam7wdXp8OpwCT9grfP0CKlPc1+3txuybG3y9ruqIG77m25R+cDg931RvWvpnqeCb7oNaJofDAkQE1gXxfLMMO8CAnKEAM+blFrPyA19tuNoLqFSpUsoQhtvrbCpPJNXrdvny5VV8TcfjnP\/XDxIvYQfzgyQm6XXXXade5b2Gc9zg63x1N19XzThsurADPC6nR2d6dVqPqeTUesP96eAKWZ2v0qnCDm59cc7soMIOzjgtxlSHHcwHgfmmpL8J6PHXD1ZtH+k830xhB7dvHPLBLTC2J7OiMD64ub2qpfN8TXiamnQadjr4Or8sox79QUXHLvXHI92G7qu52sEMO6B8uhUaWn\/Or+9e9OqEr27LLZyj++H2MS\/dBzcdCtKQ0TFf\/DzVh6ZUekP7mfpmfmR1++Cm++IMDaX74IYHpjMO61xm5lzpoMc4VUwX95rwdftYmy7s4Nae06acYTnThs1VJ8kkiLdei+froicvS6Lcluc4J2+65WGZ4OusK9WytXTwdcbrnJPcjOXp5W8agE74miBIB990MnnRqwkVcynepEmTVLwQ+pw6dSrNmjUr78On+VYAHemxSRUr1UOeztNNBT3nckOvfUN7iHcDOG4Pg3QbZdyWmgGWXtbqui3t8hLzNR+YkP\/1119XY5Dq+4XWazqbQp+d\/dFeuZ+11t7wZnepyOF76NAhGjp0KLVr1y7lLpbly5fnrZ2F6kzY2KBKL8ZuQz8LUx8yrQrxslsKkHnqqafYL28K2y7Sra4IemNO2LKEXX+k8EXe9ueee44GDx5Ms2fPTglfTAB4MY0bNw5b9qzqF\/hmpbbQb0oV0nDbdJCqM9rrMkMPoXeaYQNuYQfnhhiGovsWKVL4bt68mR555BH6+eef1RrTVB7Jgw8+SC1atKA6der4FkZuEA2IBkQDSdFAZPCFx4i4Vvv27WnhwoVqbaUTvt9\/\/70KSSDmiC\/8v\/vd79TX\/SZNmlCRIkWSolPpp2hANCAayKiByOA7b9482r17t1rUji2RqeB74MABeuCBB6ht27Yq1rtmzRoaOXIkjRo1imrWrJlPGIBZLtGAaEA04FcDy5Yto7POOsvvbYGXjwS+W7ZsUfvQEestU6aMWs6SCr5O6RAjhrdctWrVAikTAd\/PPvsscIXEXaHIFfcI+G9fxsy\/zuK8w5bxigS+qZLDQPnOIPznn3+uwg3YSVOsWDHS8IUX3Lp16wKer8A3ThP217YtBu+v195Kc5VN5PI2\/tmWigS+zs65eb47d+5UKemQNQqZtgBihCGwRKVKlSqFAr54ANnwSpStQbndx1UuyMtVNq5y2fJQiR2+emkKoIsPcO+\/\/74KNcBbrl27ttoC2aBBgwJz2hYFCqS8aYDrRBb4eht\/m0rZwo5Y4BvEQNiiwCBkMevgCimucgl8g54B4ddnCzsEvuGPta8WuEKKq1wCX1\/mbUVhgW+Ow2CLAnMUo8DtXCHFVS6Bb9AzIPz6bGGHeL7hj7WvFrhCiqtcAl9f5m1FYYFvjsNgiwJzFEM836AVGEN9XB8sXOWyhR3i+cYwWdM1ydXgucolnq9lE8hDdwS+HpSUrogtCsxRDPF8g1ZgDPVxfbBwlcsWdnj2fPft26fW3OIsLi\/XCy+84KVY1mVsUWDWArjcyNXgucolnm\/QMyD8+mxhh2f4YjMEMoxdffXVVLJkybQaOnjwIF122WWhatEWBQYtJFdIcZVL4Bv0DAi\/PlvY4Rm+8HyRHAdbf5EcJ92FspnK5KpiWxSYqxzO+7lCiqtcAt+gZ0D49dnCDs\/wxfE\/2PJ7wQUXZPR8w1cfqVy\/HBPrcIUUV7kEvlHM9mDbsIUdnuELb\/a2226jt956S51ce+2119L5559PRYsWDVYzHmuzRYEeu+u5GFdIcZVL4OvZtK0paAs7PMMXmjt69Cht3bqVXnrpJXr22WeVMq+\/\/nqVAhLnZUV52oQtCgzaorhCiqtcAt+gZ0D49dnCDl\/wNdXy008\/0caNG9WBmM8\/\/zydffbZ6rjuZs2aReIN26LAoE2FK6S4yiXwDXoGhF+fLezIGr5aRUh4\/uWXX9KIESNo165dkR29bYsCgzYVrpDiKpfAN+gZEH59trAja\/giBrx06VJ1BDwOvOzUqZM6e61atWp0zDHHhK5BWxQYtKBcIcVVLoFv0DMg\/PpsYYcv+GL97sqVK+mJJ56gtWvX0sUXX0w33HCD+vBWokSJ8LVmtGCLAoMWmiukuMol8A16BoRfny3s8AxffeLEkSNHqEuXLtS8eXMqX758+JpyacEWBQatAK6Q4iqXwDfoGRB+fbawwzN8v\/\/+exXTrVSpUiRhhUxDYIsCM\/XT7++5QoqrXAJfvxYef3lb2OEZvvB877vvPho6dCiVLVs2rQZRNlOZXIfAFgXmKofzfq6Q4iqXwDfoGRB+fbawwxd8b7\/9drWkLFN89\/Dhw3T33XeHqkVbFBi0kFwhxVUugW\/QMyDc+lZ9up\/aTlii3uAHtDqLOjY4LdwG09TuGb4IO6xfv57wt5ercePGXoplXUbgm7XqYrlR4BuL2nNqlOOYzX17J\/Weu0HpZf4t9ajx70vnpKNcbvYM31waCeNegW8YWg2vTo4TWWuLq2wc5RL4BjDHBb4BKDHCKjhOZIFvhAYUUFMC3wAUKfANQIkRViHwjVDZATXFcczGLtpCYxd9LmGHXGxE4JuL9qK\/l+NEFs83ejvKtUWBb64aJMnnG4AKI61C4BupugNpjOOYmfBdO+RCqly2eCC6yqaSrD64IbH6pEmT6Omnn1ZJzbGsDNuOu3XrFvoJFlpI8XyzGe747uE4kcXzjc+esm050fBFFrPp06fTnj176JprrqEZM2Yo+OLATKzvRcL1KBKsC3yzNb947hP4xqP3XFrlOGaJhq8+xRjAPf744\/N2vWH976hRo2j48OGBeL\/wrrGbrl27dtSoUaMCNiTwzWVaRX8vx4ksnm\/0dpRri1jjixUPuBIXdnCD788\/\/0yjR4+me+65J2f4wrtGkvbBgwerlJUC31xNLv77Bb7xj4HfHnAcszZT19CqzfuVKr6eeIlflQRa3nfMV4cdtm\/fTu3bt6eHH35YhRpee+01wukWffv2zTnssHnzZnrkkUcIQEdoQ+Ab6JjHUhnHiSyebyymlFOjGr740AbPN87LN3zRWcR2Z86cSZMnT6Yff\/xRwRZnuSH3Q6lSpXKSB3XDgwbYFy5cSE2bNhX45qRRO24W+NoxDn56wXHM\/jDyP7Tt6yPUuGppmt+7nh91BF42K\/jqXsDTRYL1E088kYoVKxZI5+bNm0e7d++mnj170rhx49LCFw0uW7YskHZtqQRvFEj6we3iKhfGiats3OS65MoOdODSsWpqJRK++BD2zDPP0DfffJOSD9WrV6eGDRtmlWh9y5YtNGXKFBXrLVOmDI0dO1Y8XyYU5uhFSdghWcaJjGZtpq1RnUZGswGtqsQqgG\/PF\/CdNm0azZ8\/n1q2bKnCDF999RUtXryYLr30UrUCAmt+cb6b32v16tXqLDjnBS94wIAB+X4sqx38ajfe8gLfePWfTevcxsz82BZ3RjOMh2\/4YknZhAkT6KqrrqI6derkjem6devo5Zdfprvuuos2bdqU73fZDDzuEc83W83Zdx+3iWxqmKtsnORCnBfxXlzHfPcVfTW9XeyTxDd83U60MH8OqYI4yULgG7t9BNYBThPZqRSusnGSy1zfe8J7s2j7iscDs+1sK\/INX3i+2EjRokULatasGRUpUoSw\/AyhBnwsu\/\/++wlLxerWrZttnzzdJ2EHT2qyphCniSzwtcasPHXETCOJJWYHZv2VPvvsM0\/3hlnIN3zRGXR80KBBKtaLmC9ONMbmC4QjTj31VLXu95VXXgmz3yqnhA0KDFpIrpDiKhfGn6tsHORygnfKdTWp86XnWcGOrOALg8MyM8APy8Kw1KxatWp00kknqZ9jcwQ+vIV5CXzD1G7wdXOYyG5a4SpbkuVCjBfg1bl7MXZ6hYMt7MgKvggzYH0vQKsvLD178MEHadiwYTlvL\/Yy9W1RoJe++imTZINPJydXucTz9WPd0ZQFeLGkDH\/ry1xaZgs7soIvYrsDBw7MB9+SJUvSjTfeSLfcckvoXi8UaosCgzYnrpDiKpfAN+gZkH19gG2fuRvycjegJsR4OzaokG9Nry3s8A1frPPFJoibbrpJaWnFihUKuNh4gfjv5Zdfnr32fNxpiwJ9dNlTUa6Q4iqXwNeTWYdWSIcX3vx0Xz7ookEcCw+P15kw3RZ2+IavuaTs22+\/pVmzZlH\/\/v1p\/\/79ancaPsQh9hv2ZYsCg5aTK6S4yiXwDXoGZK4PwH1z836a+98dBYCrvV18WHM7Ft4WdviGLzxfxHVvuOEGqly5slpahhAEEuzg31iGFsQa30xDYIsCM\/XT7++5QoqrXAJfvxbuvzxgq4GbysPVwL2oamkVYnCDrm7ZFnb4hi8EWLVqlUqcjg9sSGyD3Ls\/\/PAD\/fnPf1bbgOUkC\/8Gpu\/gCimucgl8s7d1tzs1aFd9uo+++PpISu9WA7dymeLUsWEFFWLweiUavhASy8lwYcXDRx99pP5dq1atSD62oS1bFOh1wL2W4woprnIJfL1adupy2qv9Yt8RAmwRTjBXKTjvQvwWHm7j35fxBVyzHlvY4dvzRdjh1VdfpcsuuyxfbBc\/RzKdVq1aUYkSJXIbEQ9326JAD131VYQrpLjKJfD1Zt4aqICr9mi37fs1nJDuAmzh3V70+zK\/Qbe0twbTlLKFHZ7hi23F69evV7vacLRP586dCcvL9IUPcThE84EHHpCYbw7mwRVSXOUS+OY3didk8dt0oYN0nu0ZZYpnjN9mM9USB199dht2tekky2YC9eLFi1PXrl1Vmknkewj7skWBQcvJFVJc5SqM8DUBC\/n9eLJ6vujlXzqEEBZoU81PW9jh2fPVQriFHYKGUKb6bFFgpn76\/T1XSHGViyt8Adj\/fvQ5\/Xh8GWXC2QDWBC0giwux2ihBywK+R48eVadXYGux2wWP9+STT6ZjjjnGL3N8lxf4+lZZrDcIfGNVf4HGzQ9dJlzVv3873ddPj01PtnLZEmpjQ9yQdeu\/Lezw7Pkiptu9e3dC0nS3C2kkcbCmrPP1Y7b5y3KFFFe5bPR8dVgAf2MVwa+gPaxir14+crlZrwlYW7zYbGZa4uCbjZBh3mOLAoOWkSukuMoVNXzTgTVbr9UMD+DfWF1wRtniVOPkn9RZjLZ6sNnOPVvY4dnzNQVF6GHjxo20ZMkS2rNnjzptF0vPojx11xYFZmsAbvdxhRRXuYKErzMUYHqsuYJVQbVscWV2Gq6Iv+ICXNWSrt9+r22T65jZwo6s4Kt3uOHAzDPPPJPeeecdlWAHx\/40btw4aB6lrM8WBQYtLFeD5yqXF\/hmgmouoQA3r9UMC\/z6b\/\/rY7mOmS3s8A1fndsB6SPNAzQ\/+OADeuyxx1TeB0mskz2SuRo8N7nM1\/\/3Nn6hXs+dnmoQUE3lsQYBVi8Wym3MtMyJha+XAzTlg5sX005dhqvBJ0EuE6gYHfNjlfp\/jh+snCPuDANoqKYLBWRvWf7vTMKY+ZfKntQEvj3fw4cP07333kvt27enBg0a5Mkunm82ZlDwHq4GH5dcUQPVzVM1l19lGwYIxsK81xLXmHnvYXYlE+v5Qtx3332X7rrrLjr33HOpSpUqasuxxHyzMwTnXVwNPki5nF\/8oUP9yh+Gh5oproqPzudVPyPlR6tgrCKeWoIcs3gkSN1qouELkRB+AHC3bt1Kp5xyCjVv3pxOP\/30yHRsiwKDFpirwaeTK5V3Cr1iV5WGqQKsh0Qsfscj1au\/00tNtRLAbKcwjplfPdtU3hZ2+A474OBMHAuPVQ0VK1aMJI9DqoGzRYFBGxWHiWxmqtIL\/eEdHj62ZN5i\/7Bg6vW1PxNQ\/YwrhzFLJS9XuWxhh2\/4YrXD9OnT6fHHH6ezzz6bOnTooJLpmBnO\/BhutmVtUWC2\/Xe7z0aDd\/NMw\/q679SN6Z3id9gAoL1T9f8yv61fTbFWNejxKUyQstEWgxhPW9jhG75aeCRRx0aL559\/nl577TXC1uIuXbqoj3CS2yF7E4nC4FPFTKN6zdeaCeJ1P3stB3tnFGMWbI+91cZVrsTDVw8fdrsh7vv3v\/+dtm3bJrkdvNm1a6lsDN7LB6gwX\/Mzverj98d9v48a1ip4kmyO6rLi9mzGzIqOZ+gEV7kSDV9kOANwX3rpJVqwYIEaQoQf2rRpQ6eeemokcWBbFBj0JILBH3tyBVWtjpfqf+vkKFGBVEH1t33+vwL212xV+lXfT9yU60SGLrjKxlUuW9jhO+yApOq9evWiDRs2UKdOnaht27ZUrVq1jKEGJGEfMmQIrV69mmrXrq3WCpvrhDXEli9fTj169MhjmlumNFsU6BW+qbxT82t+GF\/y073i43fm3n7tvTr393uVL1M5rhNZ4Jtp5O37vS3s8A1fbLLYvHmz+tjm9ZRifKQbPHiwAnWTJk3U6cczZsygSZMmUbly5fKNzlNPPaUS9GTKEWGLAnXn3bL74\/fZ5EdNZ7Lp4qXaKw0bpn6nlMDXr8biL891zGxhh2\/4ZmMSGMSJEyfS\/fffT6VKlVJrhO+88066++67lddsXjiOvkWLFvnyRqRqM04FpjoMMBfAmjAtU+wnlRsjl1f8bMYo7Hu4TmTxfMO2nODrj5MdpjSRwNdsEB\/oVq5cSXPmzFFZ0MqU+TWtHS4c0jl06FD14Q676KCkgQMHKm\/ZeS5c1AoEcHHy6tz\/7vDsyTo9VC+L97lCiqtcAt\/g4Rh2jVGzw02eSOG7adMmdcjm3r17adCgQSpmbIYuDhw4oE4\/RngCsd41a9bQyJEjadSoUVSzZs18MkCBuJYtWxbaWH154Cd6939HaMGGQ+pvt6tiqaJUoWRRwt\/1Ty+u\/tb\/99s5fTip3\/tsL89VLuidq2zc5MIuXH3hG1Tcl2\/4wjv9+OOPVcz3+OOPz+e1QiCEETLFgjGo\/fv3p549e1LTpk1ddQAvefTo0VS1alW1msK8wnx6wcud+\/ZOGrvo8wJ9U1\/4f1sB0LFBhazypKYbdK4eIle5xPONG2H+2w+THX564xm++gDN\/fv30\/jx46lfv37qsEx9IbkO4rrwUp0pJeHBLl68mO644w7Sx80j5ICkPCZUMUERbsCSNZTT8IUX3Lp169Dh6wZdAFcfcd2xwWl+9Ou7LFdIcZVL4OvbxGO\/IXHwzXSA5nHHHUd9+vRRy9Ccnu+WLVtU7BZLzWrVqqXWRd53330q9GCGE3bu3EkDBgxQ9dSvX1+BGGGIMWPGKFCH6fkCvG2mrVFra\/UF6MK7HdAqf9thWg9XSHGVS+Ab5mwIp+7EwVerAet8p0yZogBpfixLpyb9kQ0Q\/eSTT6hGjRp5H9JQH05FBnQbNWpE77\/\/vgo1ZFoPHKQCxy7aki\/EEAd0tf64QoqrXALfcAAZZq1BsiOXfvoKO2C97gknnEDIbAagOi+sSEAoIkm5HZzgRVhhQKv4tsFyhRRXuQS+ueAnnnsTB199fNCtt96qEqmvW7eugObcdqOFoeIgFGiCN05v19QPV0hxlUvgG8bsDrfOINgRRA89e75BNBZkHbkq0AneKdfVDHzlQjbycoUUV7kEvtlYebz35MqOoHpfKOGLZWS9525QOoTHawt4ZSIHZdbR1sP1wcJVrsTCN92qhySEHbCa4Q8j\/2MleAW+0UIzqNa4QoqrXImFr9Ng8eENO9aQXhIf2\/7yl79Ym1IS4O0zd0Pe9mB8WItyGZmXyc7V4LnKJQ9ML1ZtVxk28NVqxUqIqVOnqnW+SJ4T9pWNAs1wQ+OqpWl+73phd9N3\/VwhxVUuga9vE4\/9hmzYEUanA4v5Yr3uhAkT1EoIr+t\/cxHIrwKd4Yb5t9TLSwyeSz+CvpcrpLjKJfANegaEX59fdoTVI9\/wBWSRCB35GcwL24uvvfZa6t27d8bcDkEI41eB+MAGzxeXjeEGrROukOIql8A3iNkcbR1+2RFW73zDF4l11q9fr9I\/mheOD0ICnCg2WKBdPwp0er1rh1wYlj5zrpcrpLjKJfDN2eQjr8APO8LsnG\/4ojP4yLZr1y6V1Qwhhh07dqgE4FEeH+9HgabXi3BD49+XDlOnOdXNFVJc5RL45mTusdzshx1hdjAr+M6bN49GjBhBs2bNonr16qlDNJG3AZnKMh3\/E5QwXhWYJK9XJnJQ1hFtPVwfLFzl8sqOsK3IN3yR8BzH\/\/Tt21clyNEXths\/\/vjjNGzYMOUFh315VaC5k21qx5oUdkrIXOXmavBc5ZIHZq4WH\/39XtkRds98w1fneMBxP2beXrefhyWAFwUmzeuViRyWtYRbL9cHC1e5vLAjXIv5tXbf8P3hhx9ULt5zzz2X2rVrpz6wIQa8YsUKWrhwoQpHlChRIvS+e1Ggua7X5hUOprK4GjxXueSBGfpUD7wBL+wIvNEUFfqGL+rAcUFIhI6\/ccw7lpkBuMjzW7169Sj67Wm1Q5upa9RuNtvyN6RTEFdIcZVL4BvJdA+0kUTDF5r46aefFHx3796tVjngRAp9RFCgmnKpLJMCzZCDrbvZUonGFVJc5RL4RjHbg20jEzuCbc29Nt+e7549e2j48OFqo0X58uWj6meBdjIp0Aw52L68TMIOsZlRIA1zfbBwlSsTOwIxCg+V+IYvPN7p06dT7dq1qVmzZpEk0UklRyYFmiEHmzdVOGXjavBc5RLP1wNlLCuSiR1Rddc3fLG9ePDgwbRs2TIV7zWT6OD\/8Irjzu2Q1JCDTOSozD7Ydrg+WLjKlVj4um0vhjljxxtWQeDvsK90CkxqyEHgG7bVhFM\/V0hxlSux8A3HfP3Xmk6BSQ05CHz924ENd3CFFFe5EgdffWT89ddfTxMnTiyQ1QyTwIawQ5JDDgJfG1Dqvw9cIcVVrsTBF+GGjz\/+mM466yz65JNPCmQ1syXssOrT\/dRm2ho1g5KysUJWO\/gHnk13cIUUV7kSB1\/T2G3OambmckjSEjOtX64Gz1UueVux6THorS+Jhq\/NWc2SHO+Viext8thWiuuDhatciYWvzVnNkh7vFfjahlVv\/eEKKa5yJRa+Nmc1S3q8V+DrDXa2leIKKa5yJRa+Nmc1S3q8V+BrG1a99YcrpLjKlVj4whzDzGqGuocMGUKrV69WW5iRQ6JBgwYFZkEqBSY93ivw9QY720pxhRRXuRINXxh\/GFnNDh06pLYut23blpo0aUKrVq2qIMepAAAUM0lEQVSiGTNm0KRJk6hcuXL55pxTgRzivQJf27DqrT9cIcVVrsTD15tZ+iuFwcYGjvvvv1\/ljEB8+c4771THFlWrVs0zfJO4vleWmvmzFZtKc4UUV7kEvhlmD9YSr1y5kubMmaMO5nQm63EqkEO8Vzxfm5DqvS9cIcVVLoFvGtvetGkTde3alfbu3atOzOjUqRMVLVq0gOeLHyC7Gq5hS7+iBRsOqX8v6FKJKpbKX977VIq35Pbt29U2bW4XV7kwTlxl4yZX8+bN86YVvi3FfflOKRllhzH4\/fv3p549e1LTpk3Thh04fGwTzzdK6wquLa4eIle5Eu35fvvtt\/TGG2\/Qhg0b8lnwySefTO3bt8\/66Pg1a9bQ4sWL6Y477sg7kgghhypVqlCHDh1c4cvlY5vANzggRlkTV0hxlSux8MU635EjR6rkOg0bNlSnF+srV\/hu2bKFBg4cqJaa1apVizD4OCkZoQecEWdepgI5bK7QsnE1eK5yyQMzysdcMG0lFr5YgYC1t1gSVqFChWC08Vst+iPbmDFjFNxr1KihYIxlZ0WKFPEE3yQm0zEF4woprnIJfANFQCSVJRa+WIs7fvx4uvnmmwOHrx\/NmwrkstJBJrIfC7CnLNcHC1e5Egvfo0eP0tKlS9UONKxIOPHEE\/NmAbxThB7MUERYU8RUIJePbQLfsKwl3Hq5QoqrXImFL8IO3bt3p3Xr1hWw6Lp169LMmTOpbNmy4Vo7EZkK\/MPI\/xA+ujWuWprm964XetthNsDV4LnKJQ\/MMGdDOHUnFr7hqMN\/rVqBnFY6yET2bwc23MH1wcJVrkTDFx\/GPv30U1qxYgUde+yx1LJlS5XrAUvCnB\/GwpocWoGcVjoIfMOylnDr5QoprnIlGr5IeDNs2DC11ExvhMDys86dO9Pll18erqX\/VrtWIKePbQLfSEwn8Ea4QoqrXImFL1Y7ALw33ngjVaxYUa3DHTp0KO3atYumT5+ukuKcdNJJgRu4s8JU8F075EKqXLZ46G2H2QBXg+cqlzwww5wN4dSdWPiaJ1lANRq+5r+j\/ODGaaWDTORwJlvYtXJ9sHCVK7HwxRHyw4cPpxYtWtA555xDI0aMUJ7vF198QXPnzlX\/LlGiRNj2nrfaQa90gMcLzzfpF1eD5yqXPDCTN+MSC1+oeuPGjXTbbbcRthrjQM0zzjiD9u3bRxMmTKD69etHMhpQ4Ip3PiLAFxeHZWYykSMxncAb4fpg4SpXouEL60Vynbfeeos+\/PBD5YVeeOGFkazv1TPHCd8kJ1A3acDV4LnKJQ\/MwJ9loVeYePhCQ\/B8AeGodrWZowIFzl78HrWZtkb9eGrHmtSxwWmhD1zYDXCFFFe5BL5hz4jg6080fPfs2aPivosWLVLre4877jj661\/\/Sn379i1w4kTwqvu1Riiw5z+X09hFn6v\/Jz2hjtYTV0hxlUvgG9YMD6\/exMJXp5QsXrw49erVi0qXLq1OnHjooYcImy8GDBhQ4NSJMNQIBV52\/0Ka+\/ZOVT2HZWYykcOwlPDr5Ppg4SpXYuGLpWZY0YC0kuXLl8+z7B07dqg1vvCIo1pqVufO52nV5v1qbS+HlQ4C3\/BBGUYLXCHFVa7EwhebLMaNG6eO9jn99NPzwRebLPr16xfZJotS3eawSagjYYcwsBhNnVwhxVWuxMIXKSXfffddevLJJ+maa66hs88+m7Zt20aPPfaY2lp8wQUXKIsP2\/utUqchHbh0rGqLyzIz8XyjgWXQrXCFFFe5EgtfrOdFyAE5HdJdL7zwQtA2nq8+E75clpkJfEM1mdAq5woprnIlFr6hWbDPiis3vIwONe6v7uKyzEzg69MILCnOFVJc5Uo0fLG87NVXX6UnnnhC7WxD7Ldbt2500UUXRXKKBeZcpWY30HfndVPTj8syM4GvJTT12Q2ukOIqV6LhO2\/ePHrllVeoS5cudNppp6ldbrNnz6brr79exYGjuCpefgcdObuNwDcKZQfQBteJLA\/MAIwj4ioSC1\/kcsDR7n\/7299U8nR94dj3iRMnqmPlS5UqFbo6\/1\/n6fRTuRqqna8nXhJ6e1E1wBVSXOUS+EY1M4JrJ7HwNVNKmisa3H4enMry16Thy2mNr0zksKwl3Hq5Pli4ypVY+GKH26hRo+i8886jK664QsV4sbMNRwrhVGOshDj++OPDtXYiKnfzs3T0hHKslpkJfEM3m1Aa4AoprnIlFr6wXuTuxTZi\/F2uXDmVVhJnuU2ZMoWqV68eioE7Ky37t\/+vfsRpja\/ANxLTCbwRrpDiKlei4QvrxYqHzz77jHbv3k0lS5akmjVrUrFixQI37FQVmicWc1rjK\/CNxHwCb4QrpLjKlXj4Bm7BPirkdmKxKTpXg+cqlzwwfUxcS4oKfHMYCBO+nNb4ykTOwShivJXrg4WrXALfHCYLt+PixfPNwRgsuJUrpLjKVejgu3PnTpVyEgnYkYry7rvvVol4ihQpkm\/6LF++nHr06JH3s7p169LMmTPzJeox4cslj68WmKvBc5VL3lYsePr57EKhgq8+8fjcc8+ldu3a0datW2nQoEHqD+BqXk899RRVqlSJGjdu7KpSfVw8CnDaYCET2ecssqQ41wcLV7kKFXyxAQPrfwcPHkwVKlRQU2bs2LFqh1yHDh3yTaEHH3xQHUtfp06djPDltsFC4GsJTX12gyukuMpVqOCLZWnfffedSrKOTRkHDx6kO++8k6677jr605\/+lGfq8JBxSgbyAyNnMJQ0cOBAatKkSb7wBI6Lx3Izbmt8Bb4+qWdJca6Q4ipXoYKvOUdw+OaYMWPUj+677758p15gs8YDDzxAbdu2VeGINWvWqFwR2FGHdcT60hssin71Cb11XytLpmAw3UCeZIRduF1c5cI4cZWNm1zNmzfPm1bYoxD3VeQX7A2O4IL3++KLL9LkyZOpc+fOKgNapm3I6Nro0aOpatWqeeEJc4MFjopHLl9OF1dvg6tc8raSvNlXqDxfQPS5556jJUuWqAM2kYYy1YUJinBDmzZt1G45DV94wa1bt1a3cN7dBvlsMYygpxRXuWTMgraU8OuzxRYj8XyxzAzQxQc389BNp5pRDjkj+vTpQ\/Xr11cgRhgCYQqdvtKE7wnvzaJi294Mf7SkBdGAaICVBgpN2OGDDz6grl270t69e\/MNIEIKLVu2pO7duyvoNmrUiN5\/\/30Vali9ejXVrl1brZJo0KBBgYEHhLHaQS7RgGhANJBEDUTi+SZRMdJn0YBoQDQQpgYEvmFqV+oWDYgGRAMuGhD4immIBkQDooEYNCDwjUHp0qRoQDQgGhD4ig2IBkQDooEYNCDwjUHp0qRoQDQgGkgUfLHp4pVXXlHbjQ8dOkQ333wz3XjjjVSiRInEjKRXGY4ePUoLFy5Ua5yxJbtVq1Zq2R3Scdp6eZXN7P+qVavopZdeUuvAbR1HP3KtW7eOhgwZQh9++KFaOont8VjUb+vlVTbkXXniiSdo0qRJdOTIEZUOFmlhbbZHN52DHcOGDaOePXtStWrVYhuaRMF3w4YNNG7cOGXQpUuXphEjRtBFF11EV155ZWwK9NuwVxmQ1wIZ3vCgwSGlTz75JGEHICZ2VGflhSWbrhfn\/91666106qmnqoeMrfD1OmbYJISNRHfccYfKRYL0qFu2bFFr2IsWLepXnZGU9yrb66+\/rnapIic30gLANiETEmQl5YJDg30EL7\/8Mi1evJgeeeQRga\/XwZs1a5Yq2q1bN\/U3NmKYBuG1njjLeZXh6aefpm+\/\/TZP1k2bNikQY8df2bJl4xTBtW2vsqEC5PqYPn26qktDy1b4epULbyrYOYUdmjgkQGfzwwGzzkMDbBlAr7Jp+OIN5YQTTlDwRXpY5GlJygWP95lnnlH29uabbyoZxPP1MHowZHi6TZs2zUtDmQQgmaL5kQGGcuyxx+Z5gzjhA57UxIkT82WC86C6SIr4kU0\/OFesWEFXXXUVzZkzR3mMNsLXj1x4MOLCtvh33nlHhYogl1suk0gGJk0jfmRD2AE5uB977DFVIw47gC3irSxpF\/KLw2NH2ETg62H0Dh8+rHL7duzYUcXScCUNvtnIgFelpUuXqi3XePikO+HDgxpDK+JHNsSw4cXfdttt9OOPP9K\/\/vUva+HrRy7ACdBFaOyMM84gvL189NFHKlafKYNfaAOTpmI\/suFbC\/4g3zbyck+bNk3Ffm0OqbiJLvD1aW0\/\/PCDMuIrrrgiD0CALwwdf8qUKeOzxuiL+5Xhf\/\/7n8p5rD8QVK9ePfpOe2zRq2z4wDNjxgyqXLmy+miDMbQZvl7lgpoA3zPPPFMdEoALMXp8nxg\/fryVoSKvsulyF198cV52wY0bN6p5B9mSMPdMMxb4epzUZrEpU6aoeJMZ812wYIF6Gtv6EcopplcZ8DGqX79+dPXVV6sUmzgBxPbLi2za28K4mVfFihXp0UcfjfU10E2\/XuTCvc74KeCLNxZA2VZAeZFNhycaNmyYD76ImeJDaalSpWw3zXz9E\/hmMVzml1l8xIBXCDAhDpyUy6sMs2fPVvmM8UHD1o81Tp17lc28z3bPF331KhfKAbawSzxMNIyxJNLWMfQqmxl2AGw1tG2WTcIOAVLRuSYRMUPAydZlPKlETycDPCRc+BiA+O7jjz+erwoklZ85c6aVr7DoqBfZECNMGny9yoVyK1euVN4gVj3ANvv27Wu1Z+hVNni\/8+fPV6ttvvrqK+rSpYtaJoj4b9Iu8XyTNmLSX9GAaEA0EKAGErXJIkC5pSrRgGhANBCrBgS+sapfGhcNiAYKqwYEvoV15EVu0YBoIFYNCHxjVb80LhoQDRRWDQh8C+vIi9yiAdFArBoQ+MaqfmlcNCAaKKwaEPgW1pEXuUUDooFYNSDwjVX9djSOBDA672ymHvkpm6ku8\/fYiotF+0g87ie3LzamVKlShTp06OCnuVDLItVpp06d1C63sLZMY2dg165dlRxhtRGqkqRyEviKEajsW3HDF8CaO3euL\/Bi6IKAL3Z54U9Q+TMgC\/LfOnfzBW1qyJOB5ObYbRZnasSg5Sos9Ql8C8lIIzMajoABaJEKUG8P\/eCDD5SXhgsngiA3wdtvv00TJkxQR+GcddZZ1Lt3b5VDA+kSzbLwUJGYGvcgN2\/VqlVV2s8mTZqkzGWAUwRQFnCqUaNGXtm33nqrQL1mbl9z2+7mzZtVSlEcAwMvGfBFsiVk2Vq0aBGdf\/75eUf36C2xU6dOVRnGateuTXfddZfq36effqrKIS8IEvL\/4x\/\/UMnBoSNkWUPdSF+KenWuYZwuApmhH\/Rh0KBBdM455xSwIBO+OpHQH\/\/4R1q2bJnSU4sWLVSdp59+eoF7nQ+TdA8lgW+yJ6\/AN9nj57n3OGXh3\/\/+t0rLics8icD0fLFvHx4b8kvUqlVLweeee+5R0AGIzbLIy4tyeP0FxLZu3aqAhPvr1auXr2\/wrAFmlAcg8X+zbDrIAHqAEsCNlI3PPvusAjhyAgOWOocufodk3+gzTlxAf5D2EKBGCACnFzz88MNKdsiJfvfq1UuFLJC4HidrfPfdd3TLLbeoPMO47+eff1ayb9++XSXfRgY96AUJ05HnAL9D2MO8UsF3165dqjyOTIIO33jjDfWAc+ZGEPh6NunEFxT4Jn4IvQkA+AJMgC\/OFzOTEZlA1UffAAr4NyAJ4ABmeLU1y6JOvF4jCZBOFu5MUah798ILL9DatWsVyHXbyPq1b98+BeR08AW0kJJRpxI9cOAAwQMGBAFSHOJophnV4QsAFV4+MuDhbxxuif4B2Hv37lXZx5CPFh4v+oEUnv379yedNxl9gryAPBIawQPXRwTpHLeXXnpp3skqWlY3z1fHpXfs2KHagl6d4QKBrzd75lBK4MthFD3IgGNg4G0hVSUghINHAT2ECkyg4lUWkMZJtTgiBnBA2kF93pVZFv+G9+q8cCqsM96ZKjZr1uUGX\/O1PdVHtXSwAnAnT56swhHwfCtVqkRffvkl\/fOf\/1TwNc\/EwwcshCHgjeoTec10lwA2ksA7L3jjzn6lgq95Aku6rFoCXw\/GzKSIwJfJQGYS45NPPqFTTjlFARUeLZKZ49h2AOjFF1\/M++AGbxZHucPbhMfoBIUJzFTerPPsOd0vnD+3bds25fEhty28SHid6A+8Vjf46kTe8Nb1CRGIM+NopbZt2xLiueZqB7MeeKvoP8IF8LbNY6ec8EUIBV4v3gwQXsEF\/SDEgdACjs0xvW\/0\/+DBg1S8ePECifxTwRcecuvWrVW98HzxcILnrdvSenLCF+OBk3ZTrQCRmG8mq7f79wJfu8cnsN7B44XXhxzIxx13nPJ2AWQcRY8JDu8WXiySZgO+ACNOBwF88AEK3iJexwFcXRYxVTOOi2OP8CHp9ttvLxDzxT2AIEIU+PD18ccfq\/8D\/gBrurADQhvwOjWsEa4AVAEwPCTSwRdQRTsIEwCgaOehhx6i\/fv35\/N88WaA0ApOhkZoAaEN\/B8rIAA+9FfHndEePkbi5\/CWvcR8UT9k14nI0f9UZ7shLKLDGzi9GmOi++A8YFTgG9j0iKUigW8sao++Ub3aASDF5DZP1sVKAIQKAEHAGB+l9Bd\/eKrr169XsV14kt98801eWcAHqwZSrWBwntyANrFKAHFOQN9ceYCy6eDrTOR97bXXKsAjPJDuNR0yA6B6FQTuwdHh+JjWo0cPdfou4A3g4gKodXmsxoCO4C1DTni4OlE6+u93tUP9+vXptddeyzvVGOBF\/wHh7t27qwcJ6sTxUXhYYEywGgJH9+g+ALZmWYFv9PMoyBYFvkFqU+pKrAZShREQdnj55ZcVkP2cEZgp5huUkgS+QWkynnoEvvHoXVq1TAN69QLW7eLjmD4x+pJLLlHrn\/1cAl8\/2iq8Zf8P2sPQ\/nEYkNkAAAAASUVORK5CYII=","height":211,"width":351}}
%---
