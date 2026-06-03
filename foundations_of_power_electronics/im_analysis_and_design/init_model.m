clear;
% [model, options] = init_environment('inv_im_ekf');
[model, options] = init_environment('im_inv_model');

CTRPIFF_CLIP_RELEASE = 0.001;
s = tf('s');
%[text] ### Global timing
% simulation length
simlength = 3.25;

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

use_dq_pll_mode1_modn = 0; % simulink dqPLL
use_dq_pll_mode2_modn = 0; % ccaller dqPLL
use_dq_pll_mode3_modn = 1; % fht simulink dqPLL
use_dq_pll_mode4_modn = 0; % fht ccaller dqPLL

% single phase inverter
rpi_enable = 0; % use RPI otherwise DQ PI
system_identification_enable = 0;
use_current_controller_from_ccaller_mod1 = 1;
use_phase_shift_filter_from_ccaller_mod1 = 1;
use_sogi_from_ccaller_mod1 = 1;

% four modules in parallel connected to a dc microgrid
ixi_ref_mod1 = -0.85;
ixi_ref_mod2 = -0.85;
ixi_ref_mod3 = -0.85;
ixi_ref_mod4 = -0.85;

% common mode voltage control for hard parallelization
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
%[text] ### Settings for CCcaller versus Simulink
use_ekf_bemf_module_1 = 1;
use_ekf_bemf_module_2 = 1;
use_observer_from_simulink_module_1 = 0;
use_observer_from_ccaller_module_1 = 0;
use_observer_from_simulink_module_2 = 0;
use_observer_from_ccaller_module_2 = 0;

% current controllers
use_current_controller_from_simulink_module_1 = 0;
use_current_controller_from_ccaller_module_1 = 1;
use_current_controller_from_simulink_module_2 = 0;
use_current_controller_from_ccaller_module_2 = 0;

% moving average filters
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
grid_nominal_power = 1000e3;
application_voltage = 690;
grid_nominal_current = grid_nominal_power/application_voltage/sqrt(3);

% Transformer Dyn11
delta_star = 1;

if application_voltage == 690
    % trafo data
    us1 = 690; us2 = 690; fgrid = 50;
    eta = 95; ucc = 5;
    p_iron = 1800;
elseif application_voltage == 480
    % trafo data
    us1 = 480; us2 = 480; fgrid = 60;
    eta = 95; ucc = 5;
    p_iron = 1400;
else
    % trafo data
    us1 = 400; us2 = 400; fgrid = 50;
    eta = 95; ucc = 5;
    p_iron = 1000;
end

n2 = 14; n1 = floor(n2*sqrt(3));
core_area = 0.05; core_length = 2.5;
mu0 = 4*pi*1e-7; mur = 10e3;

% two simple calculation:
Lm1 = (n1^2 * mu0 * mur * core_area) / core_length;
% Lm1 = u1_nom/sqrt(3)/i1m/(2*pi*fgrid);
i1m = us1/sqrt(3)/Lm1/(2*pi*fgrid);

% reference for the voltage sequence
up_xi_pu_ref = 1; up_eta_pu_ref = 0; un_xi_pu_ref = 0; un_eta_pu_ref = 0;

% grid impedance
Lgrid_base = us1/sqrt(3)*ucc/100/2/pi/fgrid/grid_nominal_current;
if ~exist('ucc_factor', 'var')
    ucc_factor = 1;
end
eq_grid_inductance = Lgrid_base*ucc_factor; % [H]
eq_grid_resistance = 2e-3; % [Ohm]

grid_emu = grid_three_phase_emulator('Dyn11', delta_star, grid_nominal_power, application_voltage, us1, us2, fgrid, ...
                eq_grid_inductance, eq_grid_resistance, eta, ucc, i1m, p_iron, n1, n2, core_area, core_length, mur, ...
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
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:09b3e567]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:0f357dab]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:76245130]
hwdata.three_phase_dab = three_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:7f5b0e23]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:3cfeda2e]

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
% test_subindex = 1; % type of fault: subindex
enable_frt_1 = 0; % faults generated from abc
enable_frt_2 = 0; % faults generated from xi_eta_pos and xi_eta_neg
start_time_LVRT = 0.75;
asymmetric_error_type = 1;
deepPOSxi = 1;
deepPOSeta = -0.4;
deepNEGxi = 0.4;
deepNEGeta = 0.4;
frt_data = frt_settings(test_index, test_subindex, asymmetric_error_type, ...
    enable_frt_1, enable_frt_2, start_time_LVRT, deepPOSxi, deepPOSeta, deepNEGxi, deepNEGeta);
grid_fault_generator;
%[text] ### Reactive Current References Settings
% reactive current references 
enable_i_react_pos_steps = 1;
if enable_i_react_pos_steps
    time_i_react_pos_ref_1 = start_time_LVRT + error_length + 0.335;
    time_i_react_pos_ref_2 = time_i_react_pos_ref_1 + 0.5;
    i_react_pos_ref_1 = 0;
    i_react_pos_ref_2 = -ixi_ref_mod1*tan(acos(0.95));  % cos(phi) = 0.9
    i_react_pos_ref_3 = ixi_ref_mod1*tan(acos(0.95)); % cos(phi) = 0.9
else
    time_i_react_pos_ref_1 = 0;
    time_i_react_pos_ref_2 = 0;
    i_react_pos_ref_1 = 0;
    i_react_pos_ref_2 = 0;
    i_react_pos_ref_3 = 0;
end
%[text] #### UPQC Series Transformer
name = 'UPQC Series Transformer';
pwr_nom = 250e3;
u1_nom = 690;
u2_nom = 690;
f_nom = 50;
eta = 98;
ucc = 2;
p_iron = 5e3;
n2 = 8;
n1 = 8;
core_area = 0.04;
core_length = 0.25;
mur = 35e3;

Lm1 = (n1^2 * mu0 * mur * core_area) / core_length;
i1m = u1_nom/sqrt(3)/Lm1/f_nom/2/pi;

delta_star = 0;

upqc_st = three_phase_transformer_setup(name, delta_star, pwr_nom, u1_nom, u2_nom, f_nom, eta, ucc, ...
    i1m, p_iron, n1, n2, core_area, core_length, mur);
upqc_st.Lm1 %[output:8b639b47]
upqc_st.Ld1 %[output:0bce0dff]
upqc_ctrl.kp = 2;
upqc_ctrl.ki = 18;
upqc_ctrl.lim = 4;
%[text] #### DClink Lstray model (partial loop inductance)
parasitic_dclink_data; %[output:6101c11b]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:4e50f545]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:7a616d83]
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
%[text] ## **CONTROL Settings and Initialization**
%[text] #### Permanent magnet synchronous motor control with EKF based observer
psm_ctrl = ctrl_pmsm_setup(glb_time.ts_inv, psm.omega_bez, u_psm_scale, psm.Jm_norm);
% psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, psm.Jm_norm, glb_time.ts_inv);
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:2e751708]
psm_ctrl.kp_i = 0.25;
psm_ctrl.ki_i = 35;
%[text] #### Induction Motor Control
im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:3fa569a1] %[output:72f8ccea]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:3fa569a1] %[output:72f8ccea]
%[text] #### AFE control (with sequences)
afe_ctrl = ctrl_afe_setup(glb_time.ts_afe, grid_emu.omega_grid_nom);

kp_udc = 0.5;
ki_udc = 18.0;
kp_idc = 0.5;
ki_idc = 18.0;

%% gain for weak grids
afe_ctrl.res_pi.kp_rpi = 0.5;
afe_ctrl.res_pi.ki_rpi = 18;

%% gains for LVRT
afe_ctrl.res_pi.kp_rpi = 0.6;
afe_ctrl.res_pi.ki_rpi = 35;

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
sogi = sogi_filter(omega_set, sogi_delta, kepsilon, glb_time.ts_afe); %[output:90974ddb]
%[text] #### Current control parameters DQ PI
dqvector_pi.kp_inv = 0.5;
dqvector_pi.ki_inv = 45;
dqvector_pi.pi_ctrl = dqvector_pi.kp_inv + dqvector_pi.ki_inv/s;
dqvector_pi.pid_ctrl = c2d(dqvector_pi.pi_ctrl, glb_time.ts_inv);
dqvector_pi.plant = 1/(s*grid_emu.trafo.Ld1 + 1);
dqvector_pi.plantd = c2d(dqvector_pi.plant, glb_time.ts_inv);

G = sogi.fltd.alpha * dqvector_pi.pid_ctrl * dqvector_pi.plantd;
figure; margin(G, options);  %[output:580f7b78]
grid on %[output:580f7b78]
%[text] #### Single phase inverter - with resonant PI and virtual DQ
single_phase_inverter_ctrl = ctrl_single_phase_inverter_setup(glb_time.ts_inv, pres_ctrl.omega_set, ...
    dqvector_pi.kp_inv, dqvector_pi.ki_inv, pres_ctrl.kp_rpi, pres_ctrl.ki_rpi, pres_ctrl.delta_rpi);
%[text] #### 
%[text] ### Local time alignment to master time
kp_align = 0.25;
ki_align = 18;
lim_up_align = 0.05;
lim_down_align = -0.05;
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
% nominal_battery_voltage_1 = hwdata.afe.udc_nom;
% nominal_battery_voltage_2 = hwdata.cllc.udc2_bez;
nominal_battery_voltage_2 = hwdata.dab.udc2_bez;
% nominal_battery_voltage_2 = hwdata.afe.udc_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage_1, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:0b3c900a]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage_2, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:029435b9]
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
open_system(model); %[output:10e27999]
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
%   data: {"layout":"onright","rightPanelPercent":29}
%---
%[output:09b3e567]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:0f357dab]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:76245130]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:7f5b0e23]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: Three_phase_DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 1.200000e-04 [H] | Internal Tank Cs: 750 [F]\n---------------------------\n","truncated":false}}
%---
%[output:3cfeda2e]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 1.548074e-05 [H] | Internal Tank Cs: 2.840705e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:8b639b47]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"   0.450378722818633"}}
%---
%[output:0bce0dff]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"     6.061893472484110e-05"}}
%---
%[output:6101c11b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARUAAACnCAYAAAA\/m8goAAAQAElEQVR4AeydC5xV4\/rHn9UpojhJ6eLIzJRLDppUFEV1kKKLSx05VHK6i3Cq03FponQjoStCujmjE+V0oehimIxSEVGpoaNCV3SM0pl\/31fv\/Hfb7D17z17v3mvtefv0zFp7rXc9632ftddvP7f3eUvl239WAlYCVgIuSqCU2H9WAlYCVgIuSsCCiovCtKysBKwERCyo2G9BiZaAHbz7ErCg4r5MLUcrgRItAQsqJfrx28FbCbgvAQsq7svUcrQSKNESsKDio8dvu2ol4AcJWFDxw1OyfbQS8JEELKj46GHZrloJ+EECFlT88JRsH60EfCQBY6DiIxnYrloJWAm4KAELKi4K07KyErASsBm1Je478PHHH8sll1wiTZo0kcsvv1zq1asnN998s2zfvj1iWUyaNEmWLFkSUXvanXvuucfc7+GHH5Yff\/xRDhw4IP\/4xz9kw4YNEfGyjfwhAaup+OM5udrL9PR0WbhwoaxYsUJycnKkZs2a6rOrNwlgdtFFFyn+3C8rK0udGTFihBx\/\/PHy6KOPSu3atdWx4vzJz8+Xw4cPF+dSs9eUYO4WVErww2foBw8elO+\/\/15OO+00OXTokIwaNUppMmgz7HMMGjlypDRo0ECuuuoq+fDDD7lUeKGnT5+uNJ7GjRuL1kDUyRB\/TjjhBOnSpYt88sknsmnTJunRo4egPf3www9y7733SsOGDeXiiy+Wxx57TPWH43fddZc6duutt0q3bt1Ue7Sl3r17S+vWrSU7O1uWLl2q+nHppZfKddddJ+vXr5dvv\/1WbrzxRhk8eLDQPzSyl156SZo3b674vf322yF6aQ\/HIgELKrFIz6fXrly5Ui688EJJS0uTP\/7xj\/Ldd9+pF3LRokWybds2QZuA9uzZI3PmzBGO\/+c\/\/1GaDZ8BIYZOG0yXt956S9555x0pV66czJ07l1NhqWLFilKpUiUBMHTD999\/X2rVqqUAgvt9\/vnnsmPHDpkyZYrSZDiPdrNr1y59idC\/WbNmqbG8+eabAsC99957CrTgQcP9+\/dL\/fr1Vf+qV68u8F28eLGMHj1a9fWXX36hmSUXJWBBxUVh+oUV2sBHH30kW7ZsUdoCv+IPPfSQvPvuu9KqVSspU6aMoiuvvFI++OADWb58uTqOlnHSSSepX33GyovOS33OOecoE2r8+PGyZs0aTkVNzZo1U36XzMxMZRIBVt98841s3LhR0D4cx5HTTz9dLrjgggLeTZs2Ffpz8skny4ABA1TbJ554QmbMmCE7d+5U7dDA0Locx5FSpUqpvv\/ud78TgC0vL08gsf9clYAFFVfF6T9mvGBXXHGFerl++uknBSZ6FJg9AIz+HLzF0crLDDhBAEFGRkZws998RsNAS0Fb0Seff\/55mTx5sgBQ99xzj9KgMK\/wlwAGtGMfc439QMLJ3KtXL0GL6dChg3Tv3j3w9DH74cZzTEP7odgSsKBSbNElx4W8uJgxZcuWVb4UojWACbRgwQKlJeBLYZ9je\/fuFcwdRo\/vA3MDgACQ8F1gBnEuFNFu4sSJct555yk\/jm6HWXLttdcKTmRAAoBCM8JkoX\/088svvxQ0LH2N3u7evVtpLPhS0EB0\/\/R5u42vBCyoxFfenrjb2rVrpWXLlsqP0qhRI2X2DBw4UNq3b6\/MAswh6IwzzpBrrrlG2rRpo45jhnTt2lWqVaumxoF5BOCwxRThhWZfnQz4g2NX3w++gAVO2YAm6t5Dhw5V5gk+D3w+L7\/8snLMAlQNjzhwCT9XqVIl8DK1jy\/mlFNOkSuOaFy33HKLAqx169Ypc0g1sH\/iKgELKnEVd+Jvhk8CXwgvKiFenLY4ONEIMA0wZzgPsc8xQGDIkCHKv\/Lqq6\/KhAkTBPDgXN++fYW2kG4fOEraffbZZ8pRyv1Wr14t+G\/Kly8vECYPfcLvgU8HrQTwGDdunAwbNkzQhAA8+D\/55JPKL4LZ1LNnT4G4F\/0jUgVv+gfwAUwA2MyZMws0IiJK9IdruCf3pg98tuSeBPwHKu6N3XLygQRInANkLrvsMsFfQlhZa0o+6H6J7KIFlRL52P0z6PPPP1\/mz5+vTDQ0HaJE\/ul9yeyp50CF9G1yIUhaIo0cYp9jnCuZj8mO2krAPxLwDKjg3cfO79Spk8oxIH179uzZAmELAyiovvPmzfOPdG1PrQRcl4D3GXoGVAhLkjPxyiuvCOnX5CuQuASlpqYKYPOvf\/2rIPLgfdHaHloJlEwJeAZUyIokAkAGJWnj5DPgucdJx5bPgA4hzJL5qOyorQT8IQHPgApzMEie6ty5s5oQRliQCV8kWBFOHDNmjGAi+UOstpdWAiVXAp4BFTI1mVWKpx8TiHRsZr22a9dOzX5lLgdp2CX3Uflx5LbPJVECngEVtBBMIBKZSBmn1kbp0qXVM8Hs4TNt1AH7x0rASsCzEvAMqHhWQrZjVgJWAlFJwFOgwkxX0sGpm\/HFF18IWxy1HONcVCOzja0ErAQSIgHPgAp1MZhHwsS0P\/3pT2p+CFvSsznGOdqYk5LlbCVgJeCGBDwDKscdd5yQkk3IuDDiHG3cGLTlYSVgJWBOAp4BFQr3oI3cdNNNKqSs0\/NJ0acu6tixY1X1dXOisJytBKwE3JCAZ0ClcuXKQv2MN954QxVXpto7afsQ834YLFPt2VqyErAScF0CrjH0DKjoEZGuT\/YsIKOPUZiHY5zTx+zWSsBKwJsS8Byo\/P73v5eff\/5Z1SulrCA0bdo0IRnOOmq9+SWyvbISCJSA50CFJLfhw4cLafs333yzKjPI8hCPPPKIkBgX2Hm7byVgJeA9CXgGVDBvSMVHRGTWUkKQojyUGKSsIDVIOafbsG+pxErADtzDEvAMqJCS\/\/rrr6u6o0wgpH6Klhv7rD1DTVLa6ON2ayVgJeA9CXgGVIjssKTlgw8+KCwHQdlAVtCDrr76alVOkAmGtIlGjMwXYqlLZjtTCHnq1KnC+jHR8LBtrQS8LIFfvsuVvE+Wyd7MIZ7opmdARUuDVehIy2dlPFLzISqqk65PwSbdLtIty0NQWZ1QNWFqyinAW18PaFlKU0ugWjn4Rw6XnltDejU4TSY2P0m+6p0q2zOayYYZQ0M+R\/19j8fWc6Di9qABEIo\/4ZMhekS27qpVq465DcBlgljUasuWLeI33vTXr3032e9Ey2Xj+2\/L6hGdJavTmTL9gm0yqEE5aXt5fanc5wWpnrFULp17KOR3jR+MY770Bj8kPahs3rxZ\/vCHPxSIsGbNmpKbm1vw2a87lNg02XeT\/N3iPfKNXNHUZ9YGgR5bmae27HPOpIzc5l2YXH5Y9qJ8N\/522dLeURoJn8tUTlFAUmPCVgUmJzXtImX\/2NTt7hSbX9KDCpIhTM22MNrXbopUvHepEar3dK4Rvqb66ye+52e8I9OytxXQxu37JJiWrN9RIH\/ab926VWIl0hti5RH2+g\/fka2vPCZb\/t5IAcl3Mx+UH7Z9JnLV3eL0mCmH+78tP7Z6SHadeUVUYynsu2\/qmCdBZf369cJymyxjiQ+EBbdJgiuOEFJSUoQyCvpa9mvXrq0\/SoXX7pA9Y5oZodV9U4zwNdVfP\/Fdn9FEAmnxfY2kMNJjuq3RGQLIXz9jp6ARFJfQeot7bWHXnVHekUpfLpfyCx6W46d2lfzhl8vvlo6Xk844V2kjac9sk7QR2ZLWfaykXtmx2H0v+MLHYScYVOJwy\/C3IF9l8uTJ0q9fP8FpCwC0bt1aWLKDc+Gv\/u3ZRo0ayZo1a4QUf4j99PT03zb02RF+7Ux22ST\/RPAe2CJF1j7QSDo2qKa0l\/Sh2SbFF5Y30RrMGJyrOFkxb376ZJnkVa+jzBnMGvwkmDVhGXn0pOdAhRcfc4UlOpAZpSQJL1NiknMcC0VEiajBQvi4VatWgsZTt25d4Vq2EFm5derUCcXCHk9iCdSoWFY0uNQ4pawCl3j4XQARHfLVQEL4V\/tG0l7JF4DEOWLieMk3UtyvgudAhQgNiXA7duwoGNOGDRuEotcAQsHBoB0KZz\/99NPCwmM5OTnSq1cveeqpp4REuu+\/\/16IAqGlAEzB0Z8gVr74mJqaarSfJvknmjfgMq9PXZnXu+4RR+9WQWvJ2rzPdXmijaCFoI0AJnwmSqNBJFgbMSkX1wcXhqHnQAXgYCXCjIwMWb16tbRs2VLuvPNOueeeewTACTUW1gpKSUmRs846SzWhqNPu3buFNP+iQsrqAvunxEmgca0KBSZRmwlrpM34NfLVnryY5LA3c4jKGSFawz7MAA80EYjPyU6eAxUEDiCQrJadnS2LFi0SzJqiTJazzz5bmIhYvnx5OXTokFCDhfIJeOtxrsEXSpaQskm\/BHIyyd9LvNFatEnEuNFaIjWJMGvQPtBGABGIz5g1wdpI6copsA9LJuUS9sYun\/QMqKBpMCuZim8QkZ927doJld\/wqXCONnr8rBFEm7S0NPnb3\/6mDwsrHHbo0EE+\/\/xzYWYzphQ+moIGhezwME0QgGaCLzwZBltTZJK\/F3kf3r9DnmxVQXpcXEGZRISgZ2dt+E3Ydsenq2TLM\/1UyBezRod8nVFbBNIh3+I8F9NygX88yDOgUrFiRcEnkpmZKW3atJE77rhDVYKjGhzmEI7XSpUqFciElP3ly5erDEL8KPn5+cIcnz59+si9994rEydOFDQVTCLCyPpC9oko6c9ssWVNEBqSCb6WZ2qxQ6tFyW74zXWVSUQIuvucnXL3gn0FId\/8AWlS9cUOUnrtXBXyRRvRId+i+Cb6PN\/zeJFnQIUoDyDAlvIGaCjVq1cX6JZbbpGPPvpIOWtDCQatgBnMzzzzjDRp0kTgQ9tkDSkzNktmJIBJ1H3\/VFn4yz9k7LKLZN3kQbIse63KG0EbwTeCn8TM3f3P1TOgokVJxXwiPaTX62PffPONAhQNFPp44Jb2hJA7deokDRs2FHwsANMZZ5yRlCFl1OvA8bu9b5K\/13gHhnvxi0A\/LHtR0s45V9BG3u2SLT1OGyNNss6R1V\/H5sgN95xMyiXcfd0+5zlQoZxk586dpXfv3krjQOvo2LGjABaYSKEEgN+F0DMOWkyeM888U5lA27Ztk2QMKYeSgz0eXgKBzlXCvAAIW0BEO1gBkkBtRDtySZzDJCJKFP4uJfus50CFxwFA4C+ZOXOmUJ+WivrNmzfnVFgi6jNu3DjB\/4IpRWPyU5IxpIyNzvhMkUn+8eANSEBEZgANCADBuUqolwzWUCBSmEwxiQCX1zv\/OjmVeVKRRokK41fYMZNyKex+po7FDVQiHQARHiI9ZMb+5S9\/URoK6\/5wjHPh+CxZskROPPFEtcSHbodZhMNUfw4OKfdP+U7NAuXL5zblZ\/Y3xtvtvrrNj5fYJAEOmgCLYGI8geARDCCBmohE8a\/6yaWFxLmBLVJVlIgQdBSXl4imngMVTByiQLNnzxbo2WefVYuLaW1DP5XgkDKOWnJamHwY7HsJF1KuetwvbPSMZwAAEABJREFUahYoM0Hdpp93bjHGe9fmdcZ4I4dY+eedcKqEoj3\/OyHkuVDXBB\/\/Jb2tQIRwcZ5q+vquFSq0y2fOQczqhfBZxEp8z+DR4ex8YcJoy1q\/pvsTguZ4LIRGHsv14a7V7008tp4DFQAB04WQMcQcIELE\/\/vf\/5RvRAuFc5hIFM4hpPzZZ58J0Z969eopR+3KlSuVxkKGLmFkfR37gSHl+zZW+3UWKDNBXaayd\/3LGO\/aT3xojDezYmPmP3C2pIWg8x9eEPJcqGt+c7z7WFEzd1NTjwkv44PDjDBFaL2BvHUIOq1yeWEWdOZG55j+BLYtat9k3\/X3Px5bz4FKYYPG2fr1118Xdqrg2JVXXqlyVgAZAKXhkQgQZSRJoNNzfpj3w356enrBdXbHSiBWCeBvwSSa17uuzPpgh2AS\/cbfEutNfHS950Al0KxJS0tTNTfxrVx00UVCSchwsqXmCjkt1157raxbt07QXpiZnIyzlFF1w8ki1nMm+fuVd1EyDZxLNPKNrQpcsqKYqGhSLkX13c3zngOVU089VebNmyeYKWgdEJMCqamCCRRq8Cw+Nnr0aLnhhhskJydHRowYofgwKdGGlENJzR43IYGBAbVb2rg0UdFEP03x9BSosL4Pc3buv\/9+yc3NFbQWCI2D+T3MOg4lCLSUPXv2CKFnx3HkmmuuEfgAKtrJyyznwgpfh+Lp5ePY5yb7Z5K\/X3lHI29MIg0uXBeJSWRSLvQhXuQpUCGn5LnnnpNPP\/1UJkyYIGPHjlVErgo+EyJDoQSzf\/9+NTuZpTwuvPBCue2222Tfvn1SVEhZjvxD7TRBOlJggrdRni7Uck3W\/kX7TAubqDjo5TW\/mahoWl5HvuZx++8pUCHpbdiwYfL444+rMgaUkNT05z\/\/WZhxrCWDBhM4S5nUfrQcJiIyTwg\/zKBBgxTQhAspw49fCBMUHClw8x4m+00\/TfL3K2\/kUtxnqqNETFScnLNPrp+xU74+fMoxkSLTcoF\/PMgzoEJiG4DCduTIkcqMoQSCpuDkt+CQMjOY69evLzpcjMlTqlQpqVatmvLPaGHiq9Ft9DG7tRKIhwQCTSLKWeJvYSmReNw7nvfwDKiQm4IPpEaNGqrkwYoVKySQKIFAm1DCSUlJkZ9\/\/lk2bdqkmjAPCB8K2syaJCx8za+mGqihPyb5+5W3W6IGXHQI+t0v9olO+TcpF7f6Hgkfz4BKYGcxbciqxT+iCe2FKE5gO7V\/9I8GEMykWrVqSd++faVr166CxpKMIeWjw7YbH0ug8dFylgMDUv6jCUF7deieAxUS1Pr16ycsx3HZZZeJJirklylTJqQcifwsXbpUKEOJtkJN29dee00tzwEY4QTWGksyFL7GsRdSGC6cMMnfr7xdEGuhLAYeDUGT8o9JxCzoWGvlFnqjOB30HKgAJhRmovQBSWyacOKSch9OLpg\/RHxok5eXpwplAyZoKyTOoc0kS0iZMVpKHglgEnW\/pIKqOseoIglB086L5DlQIWyM7wSfSDQC4zqcuZg\/zESeNWuWUIclWUPKyIZffFNkkr9feSPraEPKXBMpIRcdgs64spKQlctExfAh6K0RhafhHS\/yHKiwfg9zdwAITJ5Q0R\/8LjhhSeUnMY7MW2rULly4UEV7OMZEQzJtkzGkjFPP0rGTCeMhj+KGlKPt212tLpA9Y5pJuBB0NDzjBSjcx3Oggpby6quvKmAg3V5HgIKjP8EhZUClSpUqQvTIcRwhvEwGbtWqVRUvBgslS0iZXz\/GY4pM8vcrb1Oy1nwLk4v2t\/gpBO05UMGpSqRHR330dsiQIUK9Caq76YcQuKUWLTku1LPlOOn5ABTajHbQ4gRmP3iWMtqOCaLQlAm+8DTJ2zR\/k303yTtRcmla\/zxZ\/\/iNUj5rlPxz+XoVgqYv0RDvRLwo4aACiGDKQOwT4cHhSoYslfAxgSh9gAOXeilDhw4tVDYUuu7SpYswS\/nSSy+V6dOnC3VYGjZsKAALKx22aNFCcPgy41kzQcOxtKWgbIRZWdj7xCLfr3IWyq5J7WXgkRB0cfjo77zpbUJA5fDhwzJ\/\/nzhV4U5PTfddJNAV199tRDtYe4Ps4yZmdyuXTt58skn5eDBg9KrVy8hdFyYUBzHEdpjLrGi4T\/\/+U+VAu04jipJmZWVpVY6ZA0hx3EKY2GPWQn4QgIDj4SgvdzRuIMKM5EffvhhteQGeSSBfhMctFOnThXMlIEDBxbIDRDiOpyuBQftjpWAlYAnJRB3UCHX5IEHHhCW4SBvJFgq+EZYrweH6t\/\/\/nfBp3L77bcrxys+FULHwdfYz1YCVgLekUDcQYUatPhNcKoSNtYhY73FBCKsTJo+tVHIqB01apSwPjLmDYAUH\/HZu1gJWAkURwJxBxXdSTJcyXQlQY1wMclqFFZq2rSpanLnnXfKsmXLhKpvU6ZMkS5duqjjAJLasX+sBKwEPCmBhIEK2gjFrCmmRFr+6aefLnfddZeq9rZ27VphpUKcsmeddZaUK1dOunXrJoSIPSlF2ykrASuBAgkkDFSYOcz8HJ1XQo8wiQgtc46kNULK559\/vppxjNaCA5d2lqwErASMS6DYN0gYqOCkxSGLpsJ6J\/hUmLdDaBkN5swzzxSyZr\/88kvBD3PgwAE1czncSJmlDB\/MKnJUAKH8\/HwhfZ98l8aNGwvRJaJJ4fjYc1YCVgLFl0DCQIUuN23aVF544QXp37+\/TJ48WTIzM1U1fACBFHsWEnv++eeFdgBMuMgPCXL4XmbOnCmEpjGb+Pzhhx8K6\/9QEoF5QW+\/\/bYwc5n7W7ISsBJwXwIJBRWKXDPxb9KkSWr1Qeqo8OKTEMdkQNKQ586dK8uXL5fBgwcfU6M2WBTMRsbnQvlIx3EErYSZziTCobngGEY7sqUPgiVnP1sJuCuBhIEKTlgW\/Bo+fLhUqFBBTj75ZHEcR6XWE0bGjIHIum3btq0qY4DPJdTwMZd27NghEOYNQATQbNy4UZhZqq+jLEJubq7+aLfJIQHfjGJv5hAJR9+Nv11MUbyElDBQ4cXHIXviiScWjPW\/\/\/2vEDLGt4LvY\/bs2aKJvJVw5g\/TwKmgT+4L83127twp5557rlD8OlzpA7QhS7+uBGnlEF4OvRqcJhObn6To1WtOkPfalpEt7Z2IifYbZgyVcLTpzemS9frLRqjgRTO8kzBQofo95grgwWRBfCis10PCG1oM4IBppMPLtMVhG0oepPBj8pB1i8+EnBeW9KBeLdm5+jr2g6vpF2dyViTXvPXWW8Ym6pnkzdhM8vcy73XPPySrR3SWrE5nyrJr8mVJva0FNKhBOWlRq4K0vby+NG59s9T+ywNSuc8LkvZKfkR06dxDEo6qjd2ozl+\/6Cdxm+7bWE2\/Asa3CQMVx3FUXgrmCIDAfCByVZiFnJGRoZytQ4YMUVuAhkLWzGIOJRHMHiYcEjkCpJhQiG+GGcuUOyASBLGfnp4eio1vjqOZmeysSf5e4v3DsheVubE9o5nSODA9OFamcoqc1LTLMWBRY8JWcQatkOoZSxWYnNJhsGrj1nMwKRe3+hgJn7iDCmYPvhHyUXjJ0SiI0lCqgLIFAAJtWDcZsIlkELRhztB1112n0vmJFvEZ3pQ5CFf6gGvjTPZ2CZTAL9\/lCqABeGC6sP3pk2UCiGiNA\/BAAwE0EthV39467qCCaYPWQT4KIIAzlnwV6KqrrhK0EyI9aBiPPPKI4B8hDIxPBWduKEk7jiOUNSDa8\/7778uAAQOUf8ZxHFv6IJTQSsjxQCD5qneqcpQydIADINEgwjFLsUsg7qCCb4S5PuSNACKEkPGDzJkzRzhHAhvRGtoQVsZU2b9\/v6DdoMHEPuTk4FBY6UE3R2aSfzx4AyR5RzQQNJFAIAkEEcwbN2UWKy+Tcom1b9FcH3dQ0Z3D9MHUAUj0MXJJHMeRadOmqdIIaDOa0G7QcnRbu7USCCUBDST4SQ4dMXcCgSTUNfa4exIoClTcu1MQJyYMUjaSTNrt27cLtWdxyOIPWbJkiVDBLZDQXAIBKIhdyI\/5SZqmb9qpZ5K\/m7zRSAJ9JPkD0gQfCUAC4VQN+eXw2Ak35ZLIoSUMVMgdIfENZyzh4\/bt2wtrquBHoZCTW0KxafpuSdI7fAJBJNi0AUjwkXintyWvJ3EHFUweEtMQNY5XykaikVA3ZdiwYYIJxDndhv1YiJyVZEzTN21\/m+QfKW\/MGAgzhkiNJjJS+U4U5miNlDfXe4383PdAWcYdVEhIoyp+z549hSgNtWd1h9gnvZ5ztNHHY9mSqo\/jV\/MgLyYZ0vRNq8rB\/NEOIF7oogggCCaAQdPxU7sKGgakgSJ4yznMGHwigeFerYkAKIU5WoP7rZ97cbfxvM7PfQ+UU9xBhTR8Ci49+OCDsmDBArVkhk4Pp5QkGguJcLQJ7Ggs+5haoa4nYzL4C+3WZ+x7t3glmg8gsW7yIPnk1UlF0rLstRJMr+QeL4E04eAVckfNmdKj4Xsy5OocefbGDxU5o7YIdLj\/2wL93Pl5+bHVQxEt7ckvvWnCRDd9DxP8Q33\/TRyPO6joQZA9S04K5glp4RA5JhS6psyBbhfrNiUlJewKhVeuTj0ma5JfQreIl8MtXl7g03jWDomE2k9fI8F094uLBMrrOEmgU9pnyNnVKyjae7C0fLJbZOHmPKn3dO4xdP2MnQJlbnQE4tc8kYTWm8j7F\/fesb5H0VyfMFCJppOxtKV6HKn5hLAh9sl9iYWnF67l18xkP0zxZ82aDmfnC9vxHWsLNK9PXYHWPtBIrR+8Z0wztZ3Xu650bFBN0bub98rIN7ZKxXuXSvrQbOkza0OhwzfV70Jv5vJBP\/c9UBRJDyolKU0\/8MEmw37jWhUU+ABAgA5gA\/AANNv25BUAzMg3bCkLLz3vpAcVx0nONH3UYJNfJJP8Y+Fdo2JZBTSAjAYYrcEALrHwNinPSHj7ue+B40soqJB6z4Jh5KngTxk3bpwQAQrsoN23EgglAQ0waDADW6Qq8wjTKFR7ezw+EkgYqJD0NnLkSCHEy1BxgJFVO3r0aOEcxyyFloBp+9skfxO8B7ZIUX6YyieI8rmElpx3z5iQSyJGmzBQYR4P5Q0oF0l1NqrAdevWTXCmci4RwvDTPU2ryib5m+Q95fa6UuOUsr4EFpNyied3O2GgQio+ldwoIakHjDkEoJAgp49Fs2WGM6UUyKClIj8Alaxzf6KRS0lqi0mEv8WvwJIMzyphoEJleyYQ9unTRyhOTelIZiRTnzZcLdpQQi9pS3SYVpVN8o8H73FHwtV8V9qMX8PGF2RSLvEUQMJAhUG2bNlS\/v3vf8sTTzyhCiyxzzHORUubN29W9ViqVaumqvJTr9Yu0RGtFJOnPRoL0aGsL\/bJrA92em9gSdyjhIIK5gpV3qpWrSpjx44Vyj+yeFhx5B3LEh38QpggkyndyMhEnzVPk\/zjybvHxRVUotzsrA2upPr79QSQdC8AAA1nSURBVJki83hRwkCFam4Up8b8YWlTHLbUUaFUAX6VogRAjVtqzzJvCNMJJ1dxlujgPlxrgohomeBreaZKpDIYfnNdaVyzggxdti\/ia8Lx9usz5XseL0oYqFCgqXz58lKlShXJyckRnKs4b4877riIQsrMD2JGM3OGKDtJGBqTh9KUzCdC68HhG8kSHfEStpv3QaNwk18wL5P8480b\/8pXe\/LE6\/4Vk3IJfr4mPycMVKj8dvDgQbn77rslOztbUlJShDq1HMOJG+2gS9oSHdHKJ6b2Pr8Y\/wrziPCvZG3e5\/PReL\/7CQMVyhGwxk\/Hjh1l1KhRQsEmxEXZAzQW9qMhluSgOn+HDh2EkpR8RltJ1rk\/qOjRyCfatib5J4I384iYvNhmwhrxKrCYlEu0zz+W9gkDFToNeNSpU0dOPfVUycvLkyZNmiiAIV+F89GQ4zgqgkS6P8Wf7BId0UivZLTt2KCqyroFWJgnVDJGHf9RJgxU8IGgqbRu3VpYSfCGG24Q6tQSCapQoUL8JeGzO5q2v03yTzRvNBYmIXrNx2JSLvH8eicMVPbu3StEeVj\/57777pOJEyfKjBkzlMbiOE7xZWCvtBIoQgJoLPhYvtr7a\/kEq7UUIbAoTycMVAL7SZTmm2++EarB7dq1S4FN4PnC9tF07r\/\/fvn4448LTpP3Qpo+qxvecsstwtIfyZqmb9r+NsnfC7zxsZAcN\/Do7GaKP1H4KZEAY1IuBS9JHHYSBiqVKlUSTJ3hw4er\/IHnnntOZdYCBPhawo2dfJZOnToJWo5uh9bzwgsvyNSpU4U6t6x+yFKp5L0sXrxYtWU1RJZQJeSsr7Pbki2BgUdnN2MSIQnMIgCGEgqJBBj64lcqlaiOO44j1FIZNGiQylHp1auXkLdC6YOiQso4dpknRBZtYP\/Jfdm3b586hOMXPgAIOTAs\/cHnBg0ayKpVq1QbP\/8xbX+b5O9F3phEAAu1WTCNOjaoJu9u3quqywEyEEDTeup\/VL6LCa3GHbkk\/ludMFBh6JQ++Pzzz2XRokWqONPZZ5+tJheyNhDnQ1HdunWFyvtoO7oNkxAp9sSERGq0zJo1SwhXb968WciC1O04l5ubqz+qLVm5JogsYRN84WmSt2n+JvvuBu9OV18kk3s1l\/WP3ygVXrujgL59L1P25MyRnKXz5ZWFy2X0qx8cAzoAT1FUqecrEooajFylzp3WaZK4TT82HqC+6\/H4kzBQAThI0R8zZowyVzBZILJrKdYUOHjS8Pmik5ZPen7gOb1PZu1LL70kmDhffPGFcI3OtCUnRrcL3nKdpS1iZVC0DLYveEKgnbMfkG9f6im7JrVXIWq0m0ip\/\/UNpChq3\/IKMUHB331TnxMGKtQ6wdzBl\/Loo4+KJlYs1IlwetCAA1\/65cuXC+n5+njglvOk\/NeoUUPNUq5fv77s3r1b+W0AGd2W\/dq1a+uPdmslEFcJ4MMpijDDTFC8BpowUMG\/gUO2OIluhQmHDFp4EUXi\/OrVq1UpBLQbluUAxCD209PTaRJ3wgmNSYaK\/uabb8b9\/t67of97RHkNzG6ijWSDo4H7f1SxjSDuoPL9998LtWkfeeQRAQTatm0rPXr0EBYRgzhHm2iHhT+mS5cuwsMlpDx9+nSh+lvDhg0FYKFOS4sWLdSKiKTuR8vfjfb4jnr27KnmOGVlZanSmW7wtTwSJwEKtWPGz5w5Ux566CHhhzJxvfHGneMOKuXKlZOuXbtKv379hIxawsMADJ8hztGmKPFgOk2ePFkuuOAC1dRxHCE7d8WKFUKqPmUVUlNTlSlE+JmXmOO33nqrOqYuivMfQJQ+MZmSMZIAGOcu2Nu5LAG0X3KmXGbra3ZxBxXq0hL6BdUJC+OodRxH+Urwl1SuXFlo42upFtJ5vniHDx+WsmXLqrPU5qWmjPpg\/\/hWAhRteuqpp6Rdu3aCdkyypW8H41LHS7nEJ2I22JxoJixHSrQGpyxOWl66iJkkpGHom+K\/ASR1C6JXmHHkxFDjBW0MoAysFYNPiXwbfY3deksCkTxTAIRJsFOmTFGAsnbtWqFWsrdGEv\/exB1UUBdR\/W+88UZJSUmRv\/71r8q3UBw\/SvzFdewdmVIAIGJTo3nos\/hO+AXD3Hr55ZeFCBeJTeTSbNq0SU1DoD2Aqq+xW29IINpn+umnnypzmrQFTHLHsfPW4g4qfHVY54etJl4wP2oqaB+Erq+\/\/no9FLUl9N2qVSspU6aMms9UvXp1IckPRzETJ5mNjQOZL6G6wP7xjASifab8MJBwSUSPdIbAREvPDCrOHUkIqMR5jMZuR+o\/mb316tUruAfRABywOtvXcRwhcY86vEyYRHN56623VEZwwUV2xzMSiPaZYv6QHkCAgCCD4yStphLxM0oIqDAf58ILL1QvG7\/YK1euFLa8fIR\/Q2XNRjyqBDfk1w7\/SYK7YW\/vogTsM41cmHEHFSI8mAdkwBZGnKNN5EPwVkv8RZg1JLrRM5x5+FPOOeccPlryoQTsM43uocUdVKLrnv9aO46jEuwAR6JAX3\/9tZDlS1V\/\/43G9hgJOI59psghUrKgEqmkomhHwW1sc8LJZPjecccdwjSCKFjYprFJwPWr7TONXKQWVCKXVciW1Nhl0qNuQNSHiZEU4MaB16xZM33Kbn0iAftMi\/+gLKgUX3b2SisBK4FCJGBBpRCh2ENWAlYCxZeABZXiy86tKy0fK4GkkoAFlaR6nHYwVgKJl4AFlcQ\/A9sDK4GkkoAFlaR6nHYwVgKJl0C0oJL4HtseWAlYCXhaAhZUPP14bOesBPwnAQsq\/ntmtsdWAp6WgAUVTz8e9zrHOkgXX3yxsNa0pmHDhrl3gwRxImuZ5W4pOcGUCKrsBXaFcU+aNCnwUME+Ez3Hjh0rzNEqOFjEjj1dtAQsqBQto6RpQZU6pg1ouv\/++wvGxmxqaugWHPDBDkDy2muvFbs2DUXIKVFB6Q0fDNc3XbSg4ptHZaaj\/Ir37t1brUSQnZ0t69evF5ZNYTIk69nwa86dly5dKix9gpbD0icZGRlC3Ru0A7a0QUtAMwCgKAJNW\/iwHg4AwHl4srrBJZdcIt26dVOlRNEURo0aJRyj4BXV8QCLe+65R3RFQDQKiPtooo4stX4pfqWPhdqildEfiGVz6cOBAweEIktz5swRirGHutYej04CFlSik5evW3fv3l0VxqIYVp06deTjjz9W49mzZ4+w9jQ1X8aNGyfjx48XljThpWa1A9ajprhzZmamUNKBlxiQUBcX8odrWWSLCnfvvPOOUI9k7ty5qiXLlKAx0QYtYdWqVUJNXwpGL1u2TAAeTBqKgu\/cuVN27NihgIei0pRsVEyO\/qH9eeedd\/TTr5vAMTJOAIMzaGVoaIyT2sjUFaZfjIXxU56CdpZil4AFldhl6FkOwR175plnRBfGWrduXcGaSU2bNhV+8XNzc4UXj19zXsiOHTvKRx99pLSXKlWqqHq7juMIM3gpRBXMX38GFHh5AamaNWsqkFqzZo06jZZAHVdmcrNmEyYXBcIBDBbiomQEPhK0IhaIW7t2rWzcuFG4X3BNGoqoc1wxPvoncIyM9YYbbjh6RoSVHIYMGSIsLId\/iRMAG8um2OVSkIY7ZEHFHTkmBRdMAJYVAUh4Ian8P2\/ePPVCY9LoQR48eDCscxOzYsCAAQUAhtaSccRc0tcHbzF\/qEavj+\/bt08BQPPmzZVmhOnFki6Ajm4T7Zb+T5s2TVhX6vbbb1cV8KPlYdtHJgELKpHJqUS0wnHJUimACS\/h\/Pnz5fHHH1cFpjBPqGLHcbQZAKh06dJqxYC8vDzhOD4OBIUWgPaBJoF2MHjwYMEM4lxhBJBhVgEuLG3St29foXh4enq68tvg60F7Cr62atWqEs4MC2yPicV4MHvQkvQ5NCX6zqqR+pjdxiYBCyqxyS+prsa\/0L9\/f8HZetlllwmV\/3v27CmYIThkMSUoTA64YHZgqvCZJUdYVQDfC0CDeQRQsMW0Yr0j9kMJ69prr1WaA07TDh06CGtCYSLxousC6XwOvp574BcCFILPBX4GrNBSvvrqK8FRDECxxb+D34axoMEEXmP3iy8Bz4BK8Ydgr4xEAlSmK+zFBjQgzQNfBg5WNI2ZM2cKaxY5jiOsbZSTk6N8LpgPvMiO4wjLUuBDWbx4sTz77LMyYsQIpb2gbXAcwhRCO+D+9EPfi\/tyjJd6+PDhQmiX+3Ivx3FU5IeXvk2bNgp09HV6S6QIjQbtBh70F376PFvuR184h18HLQsCMAESxsoaTbGYVtzH0v9LwILK\/8vC7nlIAoSpARc0HxZsK6xrAMltt90m+FwAucLahDtGuJyF7NC2wrWz56KTQKnomtvWVgKiokZEUUzKgmVaXn\/9daX5hNMiiCaRc+I40S\/ihQ8JUw8tyuRYShpvCyol7Ynb8XpTAknUKwsqSfQw7VCsBLwgAQsqXngKtg9WAkkkAQsqSfQw7VCsBLwgAQsqXngKfuuD7a+VQBgJWFAJIxx7ykrASiB6CVhQiV5m9gorASuBMBKwoBJGOPaUlYCVQPQSSH5QiV4m9gorASuBGCTwfwAAAP\/\/65qYWgAAAAZJREFUAwArgrvLKGfg9AAAAABJRU5ErkJggg==","height":167,"width":277}}
%---
%[output:4e50f545]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:7a616d83]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:2e751708]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:72f8ccea]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:90974ddb]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARUAAACnCAYAAAA\/m8goAAAQAElEQVR4AezdC7yv1ZgH8PU\/Y9QwU8eYoUZyDomETsNwIuZklFxSaJQxM06uiRCVIen0QRfXaHILnTDI\/U4lyj1EpMitEhHhuOReZn\/f07Nb593v\/37Z\/733Op\/z2+9613rWWs961nqf9azrf9lfyr8igSKBIoERSmBZKv+KBIoEigRGKIGiVEYozJJUkUCRQEpFqZRWsKQlUAo\/egkUpTJ6mZYUiwSWtASKUlnS1V8KXyQwegkUpTJ6mZYUiwSWtASKUllA1V9YLRJYCBIoSmUh1FLhsUhgAUmgKJUFVFmF1SKBhSCBolQWQi0VHosEFpAExqJUDj300HTrW986tcN\/\/Md\/pN\/85jcjFdOrX\/3qKr9\/\/dd\/TT\/5yU+GSvuCCy5IO+20U5WeMjTxWy\/jxz72saHyHEXkUcqgF37GkR85knmOUdRpL+XJ6x0fvcRBE3LAs3YjHf45or00taWcrsktPelKvx++mtKq+\/kO8STtN7zhDZu0e3519JL\/WJRKnfH6++c\/\/\/n0wAc+cOiPv57uuN5\/+MMfpt\/+9rezyauIK664YvZ9mh3jbJCjLrcP7\/GPf\/ycZC+\/\/PK0evXq5OOdEzjPHtrCJz\/5yVkufv3rX6fPfOYzs+\/T7rjkkkvShRdemG55y1um29zmNl3ZVT\/d6mGsSkVD+NrXvpa+973vzeK1r31txbiG8u53v7tyT\/sfvCpH8BkVsc0226S\/+7u\/C+95fx544IGVnM8555x0s5vdbN756YcByu\/MM8+sohx++OFVObQbHZAGL+BVr3pVQsc9DtzpTndKX\/3qV6u873vf+\/aUBf4gJ6ZkKJvcb1rdp556aqIIH\/GIR6S\/\/\/u\/n2XTd0r+Ae3f94zgrW99a0eDYKxKBQN13PzmN2\/7IeqpcnOLydfUiPgJC1rx6vnEO3Mt6Dz7NaUf+tCHVkl95zvfqZ7+XHnllVVF7LDDDl7noJ6nfJt4rNOh0QugDz7zsqJHIzyAPhjg5h9xve+9994Vr2j0MkxdDT7SiXfhII40yFfe\/ICbnzAQn38T8IkmEPw00Yaf3l3jpkBC5sIoxyOPPJKzKgfZVy\/X\/cFH5OOJR7xeFzz7qPMkXpQ1+BNPfOmgn43cwfHRj360CsV3fHR6\/ktmLIAqoM0fQ3T5ygsf+OEOdMq\/G21ejkivXs\/YwsMXv\/jF6nu85z3vyast\/vZv\/zb90z\/9U2N4Pb+JK5VoPLjbbrvtPKr5FYWuWy4amY+C0CvCmT+EzU\/YzGv1X7wXvvCFlTv\/Q\/g+pNyP1bH77rv33ONtvfXWldDz3ica0vbbb58nXbnxWs9TAB6V0QftXTnqdGiayoEe0KPhDqCXZ7yP44nXSclc\/egV83KwGqLH5BZGjuRZl4d2gddcJvgnO\/EC4pFdvA\/yjI9S3H\/5l39Jj370ozkr5aedVy89\/MEHfnJS\/OZliDD+TbTKiMZT+cnBe4A1VZ9yIGfy3nHHHdPKlSuDtPGZl\/UWt7hFutGNblTR4bGe31iVioLc+c53np3wpDUJEDcaB\/dee+2Vjj766ISW\/1\/+8hePdO2116Zwh9mrYM973vOqcD2DOBobU63yzP4Qbgg\/zGlC1JsQ+LHHHlspsyxKo3OrrbZKy5cvTzGvojGbT5G\/Xi2PJIzy4Scf+eX8RQ\/WTzmkFZBnlPl973tfpeyEyVPe3DkMh3I6cnrLW96S9Do5XSd3P7wOI3M9ZQwlfTjaCugYmvh785vfPNtmlIucyZvc0Q\/SZsTrB\/LzUYqz5557Jm1dHXlvVyfC6lBu9aQM6jfSaBpmCEODVhxxpWeSVRuIDq+JDq94Ro9WHG7KsN4m8jpQD+QqvnRPOOGEqg2xUMhZGqxLPMFYlYrM2sH4GQMXX3xxete73lWRtVqttN9++6WPfOQjSaO5+93vXikXSoDmJxAFQ8wkZhpzU1AKxR2I4QohRBjBESCa+MC5O8FwTS+0YcOGxPQ2YUvB0NbSy+N699Eql6f3PFw5pNFPOfL4xr1RZj2LHiYPH4e7H16Hkbn5jJe85CVziqBj0KhBr4iAovPBcatb9c9N3s961rMqZUvW\/bYZafSD\/AOmUNSNtiKNXtsXWpaz8nNLQz1za+vkzx0QhsZ7Uxt48YtfXM0JnXNO87xa1FHMC\/o+8C69XoAndYJWWyZniu1Rj3oUrwpjVSq0G6H4yAKveMUrqoxbrVZ6wAMekE477bR0gxvcoPJbtWpVYkHc7na3S7vuumt66Utfmm584xtXYd\/85jerpz8K4WPnDsRQKt5DeISAD40SaGA0hEEo3N0gbfR5IzW+DN7q8TV+eQUizzpdL+XI4+Aj3n1AeIj3oZ49RO6F12FlTjlEO6Es6myxbMlWvVHywnOZeNcuWJbcwQ93L\/yj6xUUm\/kI9BRJfOgsFn7RXri7IeIEXb1M4e+Zh7VrAyzGaHue9eGJdLRlPOa88w+E9Rf14Tv2HQlnnbBSQr7SkYe8YKxKBQN16F1veMMbVt5m2n\/1q1+leF+xYkVqtVpVmD+UTYT9+c9\/TlEIYZNEVKT8Qd71hsAPmOsaP3cMu3IzlX+kwT3tmC9eo8fVqHP5GVb87ne\/61ls4+LfR6bDwoie28cEeQeCV8MMNJMCpRs8UMzkZ7jEIgke8IQ3ija3MCK86UmBhZVPiVBKTXT8Jq5UZJrjH\/7hHypzld+Pf\/xjj1nokaIBUTjxcSuUsFnCGUe7xkO7agCEW4eecSZq1\/\/MQ5WiZ4rK0CPWI9LehnX8aXpzGtx4xTM3DFIO8cYBQzlDunZpD8LrIDKnjH2UVkRYATk\/dTOfVdBkjYhD1rkVMwj\/0ukEH2XMR3Si63UIVG+78e6jb2pn7fK86qqrUgwLdWgUM9pcJt4pGdDBky2\/YeDbkF58XxNXKl\/+8pfTH\/\/4x6oMTC8Fu9\/97le9f+5zn0tMNy8qzlAop42PW7gJ22h84ugt+AfCklBYCP9OjTdo6k8z3eZQ9EzS0qC7VXY0DOWoN8B+ylHnZVTv8bEpE6UrXfKMRukd+uF1GJnn\/NTrkswBP4Z8OhhzC97Rqn9ustZmKHAfpMnffviXRi+I+Qi0YQ3EB+XpgxaGj049OhqI4QS3jsk7dy\/tDF0Tov0Ji70o3BBzQfe+972rCVd+3UC20Y5DtuTLrQ2ph0hjrEpFQ1CpeqDAYYcdVuVtWBOm1\/3vf\/+0bNlGVphuaMUTH\/EWW2yR7AnRQx153Z4FBdEjohUHXQ5WiArnJxwdROE1SukJ7wamn8YcdJRhU1xan5JEZwgkv7wc\/EFc+XN3KweaUYEcLMNqIPERS5s\/XskTP\/wCeDUp7l0YGrTi8MsxjMz\/8z\/\/s9o1K72QnXwg8tIjxgeb0wtHl8v6iU98YjL5if9Ry5qioDDwGoqUOxAfm3dKmrLmbgdpxZyEp3e05I5\/7l7A6tc20WrnZALc\/EDds7gpA3zya0LIVHzIZRsTy+RLzuLndbbxS+Y7QVAo\/\/3f\/5309gT++9\/\/vlr3vstd7jKHC5O5W2655ay\/hmtoMesx49DQYMa5yX\/mX52WMI3RY2iySYQOL3njyT\/IPArlc8LMcpvGH\/4UG+UYfrS9ipV\/nTe0TeWItAZ5qniNoB63SY7yr\/MkXhMtPkF4jkFlTnZWzJryl7688hWNoMez8BzSIN\/w4+YX757iSZO7H6g7Q2Bx1KmPjTtH3rlQxGEJ5jS5Gx\/4Cb9oo+Qefr0+161bN6ucxdEJaH+e3v\/v\/\/4v4Umb0Db49QNyVMcRh2x9T3gOv7EoFZkyA5uggJTJ6173uqrwCnvAAQdUe0bOO++84Gv2+eEPf3jWHQ7CztNWMOCXNzz0dVqTw92EKRyd9MSvpyMvfk10ehY8iAtkkfv5cHwQ4ksbTQBtbraiacqDP6AXN9LEl3f5yxMNBJ2woOXflH\/4Kb+80UH4SwPkBdz1\/Oq09bSk1w71uNIHeTXFycuGDqRRp+UnLCBeP7KO9NQdGUqnXu4mGnTylh+3uNII2nhGOJq6vNQDP2HSijieES\/SlTY3WuDWFjy9f+UrX6mWnMUTP0eeD9om1PMXvx5vLEpFRu1Ao9GmJ554YuoV6MVrlyb\/P\/3pT+nkk09OzD\/m2r\/927+lT3ziE7Mb6NBMC4ybbZzDZwxH8MY\/JnqVQ2PgXzC4BMh0ccl6cFlMKubElcrf\/M3fpN122606pWzbcC9AL14nobz97W9PL3vZy9IjH\/nIavOcjXNPetKT0qc\/\/ek0bf9oduYnvlhuTGgKJsbTFGjMN6EpGFwCRdaDy27QmBNXKtdcc0366U9\/Wp1yNJ\/SC9CL166QP\/\/5z5Ox4sMf\/vD01Kc+Ndk8Z5Lrbne7W3rnO9+Z7HFpF3e+\/JmfTeYlP6auj2G+eFts+U6jrFmhhk\/qu93QbqHWw8SVys9+9rO07777zs6nmFPpBvTitRPyd7\/73epsjvFeq7Vx8xzL5q53vWuyhE1xtYu7YcOGtKEHXHrppenSKcD555+fzu8TnfjOy26fUB3t5Fb8iwTaSWDiSuWmN71pZT0w+8GauV7ZuNfMsuHKpz71qaR34U\/hrF+\/PonXrhA2cNl9m98Hgfb2t799smO3k0J6+ctfnnqBtf5pgJn2ftGJ77zsr3nNa1Ig\/J2lOeSQQxJYPgR+gfA\/+uijU\/gdf\/zxaSOOT+ouh\/oO1JVjrtDUX8HClMDElcpf\/dVfpX\/8x3+sLhGiBE455ZRkyfiNb3xjYmnYD2KjmSU2hwqdVn7Tm940u4+lScwao1lv6\/RN4cWvfwlsvvnmKWATFjixDeHvGf5y8A62CAAryIFRwzk499xzq7NeznsBxUMZAYVkP0nAPFqOe93rXinHLrvskgImtQP\/\/M\/\/nOowZxW4wx3ukHK4viJgXmsxQx1NAhNXKnmhfvGLX6QvfOEL6R73uMfsVv083ITlmjVrqiPuaPOwUbk7DQ1GEeYg5Djgg63DBx2w67QdTGLbDl+HA52BiEtpwCDyDl7EBwop0vVkSUZ+LFJ1DdwB4ejueMc7JvuDwI174ksPdCY6FdBB1cHKDdjOkEMHFrjVrW6VcpBTDhPpOZ7whCekwHOf+9xUxytf+coUsAGtjs9+9rMpYG5lnBik\/gaNM69KxX0pJlFtKGpXAOFXX311x8lWyscZoV\/+8pdzkjEsYh3NCbjOYxSKI9LQO+eID0nD93HkcDQhYKUnxzOf+cwUsKIVsDszEH75M+J45unV3Tby+XjryD+YiGPiG4466qgE3IGgES9XUMrpowcyuE7UPT9CGXmKT34gPaBkQD5AAUVZuAPC0IF4IB2QLsijHWMs4Bz14VoM4zxPPfXUVAdLLGCYWEdYZZ65VcbNr448fqQbz3yIGW58BTqVs135B\/WfV6Vi+KPimcJNG98uuuii9I53vKPaoq\/3aVdIvcsf\/vCHasI1p2Eh6JU0otw\/d1NoPoq99947eT7mMY9JOXygOWxHDtgdm+OYY45JOZ7ylk5\/JwAAEABJREFUlKekgJUpYJV55h+h1aocyhqgLAPOnJhv8uwVvdB3oqmHsRgDdmbqGPCqDoACN9RwrgQe\/OAHp\/vc5z7JDmpDWiADCtU2cTCfBi5eBnUGLI+8rnpx+3gClIa6BwoFKBjQ7oAC6lUhiS8t6ULk0wtf\/dLkyizcuVILZRHPukLzHgrHkzz75WFQ+nlVKqwIx6md+9l\/\/\/3TQx7ykPTsZz+7ArePXM9vcnCzzTZrW0YfpGsfbZ\/WyBFSFkxLy8o3uclNeDXCJO7Tnva09LTr4ExJDr16jhi7e9qO3S\/MC\/QbJ6fvN34v9J1o+g1rog8\/E+9gCONjNocG++yzT9pnBiF3S6zwjGc8o7KOcgspt460j1DOq1atSqtmsGLFirRiBsuXL2+s706eoSQ8xadAQHqBukJSjrpS0l7wwl+50AB\/4A\/ClIdfDnFz4CHQif9pCZu4UrHfRM8bAtBD2UX62Mc+Nlkaftvb3pbgG9\/4RiJo41A0Qd\/0NPH7sIc9LL3+9a9Ptv\/bRWnizzZsy9Gt1sZl5qa4xW9hSMBHDvFxx0fn4wQKBnykYIhGGQF3QBg6GKVCIkXKCHSEeOVn3osf8Af+IOyyyy5L\/IQHlE2ZAJ8xb8NitrAB73nPe6od43aNx5A4nvmQmFs8nac8J4GJKxWFe9CDHlRZJYY9NrY5hazgDl5RLN\/+9reToYtt\/CbRehHE2rVrq2VPy9J6AOm4Zc4EX7f4vQ4lRkFXH070m2a\/8Xuh70TTb1gTfd3vBz\/4Qeq33MPQ62RiyMaSNVwDwzUwXAPDNTBkM1wDwzVoN2TT4dWHbJRDtzZXD6dgAuboYqhzzjnnpICtBIY1YMn\/6JllfLANwOZPeO9731stbFgAkQ5QWiydep6195G9TlypmIV3O9XOO+9c7UVZvXp12mOPPRJloLEpmXG5Zz\/467\/+6\/S4xz0uOdZNoTiIKO3Uwz\/nhCYFPeUwefUbvxf6TjT9hjXR1\/3q78PIo5e4w+THUgDWgnNargSApz\/96Slg1Sfm2U466aRqr48PH9y\/DKeffnoC1yaAJXZgTUPMmxi299Bk55CIBy5kgh\/96EfVbxjJA8xdzYk0Jo+JKxUK47a3vW21\/Eab0sIsC\/tU9BL2HphYYq0YKo2p3LPJjnMZr6R9\/Y\/ILVVZsL7B\/B6cccYZCQzrwR4scDETvOhFL5ozj2ToRjFSbhDDNk9DpRgSxpBrtnHPk2PiSiUvZ6vVSvYcHHTQQdXBP5ujDj744HTWWWelPffcM7FmjjjiiOS49iQUTM5bcRcJzLcEKIkAxUGBwJo1a9Ka60DJUDhA+ZhDCngHYTCp8syrUqkX0oSrU8Y0uTHlcccdl771rW8lV0Cai6nTl\/clK4FS8B4kkCskSqmHKCMhmSqlkpfIhjbXTL7gBS9IJnZbrbKCk8unuIsEplUCU6lUHBC0vdn8imGQGW2z9tMqxMJXkUCRwPUSmBql4rJfd59YCTJh65SyJTaXOHFbNbqe7eIqEigSmFYJzKtScQWkn1ewH8DJUhcAW15jlfzP\/\/xPtSRmr4rdq1aNplWIXfgqwUUCS0oCE1cqVnGs5ph8tWXbmr91+5UrVya\/5WO3oPkUR9A7bc1fUrVUClsksIAkMHGlYhXHORvbjLfddttKkbisyYqPlR+HDBeQ\/AqrRQJFAjUJTFypRP6GOH5u07UG4TeNT6en7ZicRt4WI092hRr6usfE70DZqWqYvBjLOi1luuKKK5IDvWTuOIJfpRhG5nWlMvZyuvD3Qx\/6UNJYDG\/sRVl93VZ9E7VNd6KMnamGDK666qrqGgM38lN+DSTFawwS0DYk61rRs88+O33961+vLjLiVzAeCVAirqIwYnC+yGl\/O9oHzW3iSgWj7slwetjOWYfN3F\/iwJeJWpO2VoJs63ZHCvphYOXI\/E09DZqYME0Cm79xhsSJTxYUWhPDLs52BYP3gv4kMKjcHYCzP8lZLnNrrhpwYr2\/3Jcm9aAyd2DSztxWq5V8c74Bsh9UivOiVHJmnUJ2hwot6dSyC338LCoLxglQCsGuWgXN4\/XidrL1Ix\/5SCNpt98JcgeL5W0meGMCxbOtBIaRu9PqzrRIXL1T9Lamey9oL4FhZG7BxDfnwKTtHO7wHeZU87wrlRATy4DV4D6UL33pS9XwyESuCV2nj12RELTdnuIYUrkAiLDr9Avxd4LqZZjG91HJ3X07Tq2vXbu2ujnPWH9S5V1o+YxK5kYPb3nLW5I5RAcgHfQdVBZTo1TyAiig4ZEVIeM7N4Ixh3OaTm53qPgxMSc+3TJWp3U1gl27lE6rtXH7f6+\/E1RPq7xfL4FRyN2koR7TNRjmV\/baa6\/Uam2so+tzKq6QwLAy\/\/73v1\/dtBhDTBa6Dtl+scij3+dUKpW8EIZHLBWFzf07uV2tYCeu4Yub1uu0FIqrLOvL18bv3X4nqJ5Web9eAqOQu2Hprrvumlw+1E+dX8\/F0nINK3MLIyzDCy+8MJliMLfpMK9Rw6CSnHqlMmjBOsVzIQ5rqEnhdIpXwoaTQDe5u6HML0qaWzOvYnwPbjQbLuelG7ubzHWuhx12WHK3i4ucHN5lwdvhPqjUlqRS6UdYBGxWvZ84k6JdbPm4htFlXS7vMuwN2BS52Mo6TeXxA37mU1wiZXe76YZWa\/Ah55JUKpbLmHxMv3rl0twmjev+5X14CRS5Dy\/DflOYD5kvSaXismPr8U5B55Xksm2\/jzLMclqeXnFvKoEi903lMYm3+ZD5klQqw\/xO0CQawmLNo8h98jU7HzKfmFKZvDjb5+jayvI7Qe3lM66QIvdxSbZ9uvMh8yWpVFSBjVWHHHJI9dMgbvO3d6XX3wkSv2AwCRS5Dya3YWJNWuaLWqlYNjar3bR6YzOd\/S+D\/E7QMBW8FOIWuU++lqdJ5otaqUy+akuORQJtJLCEvItSWUKVXYpaJDAJCRSlMgkplzyKBJaQBIpSWUKVXYpaJDAJCfSkVGwUcxOXO05c9ecUqad39124VGkSzJY8JiSBkk2RwBAS6KhU3L520EEHJcer3cjmjgunSF2o5OndrW0OHx1wwAHJpTpD8FKiFgkUCSwCCTQqFcf\/jzrqqORaP9cErFu3Lrn68eKLL04UTcC76yCf\/\/znJxcfuftCvEUgl1KEIoEigQElMEepuGFtv\/32Sy579kNfLsJ1StTv8tjbkefj3X0n6NGhFy+nKe4igSKBpSWBOUrFL8W7v8Ktabe85S37kgZ68fqKNCxxiV8kUCQwVRKYo1RYH\/Ub0aaK48JMkUCRwFRLYI5Sybn9yU9+ktzA5ScsumHnnXdObpAy35KnUdxFAkUCS0sCHZWKC16OOOKI5L5KZwvc+2rS1nKyy3FdZuTWqIc\/\/OHJvaKnn356Mr+ytERYSlskMJ8SmL68OyoVN8xfdtllyQ1pfj\/nxBNPTJaWH\/\/4xycH9dx2brh0m9vcJgn7+Mc\/nrbZZpvpK2XhqEigSGBiEuioVCiTD3\/4w8lluFZ56lxtv\/32yb0kaNC6SNov\/dXpynuRQJHA0pFAR6ViJ639J+5tbScSYWjQovHuWVAkUCSwNCXQUalsueWWye+K+KFsvwdSF5Ht+W7gZsXc6EY3Sn6f2GVHdbryXiSwUQLl71KQQEelstlmm6UnP\/nJiaKwu\/b4449P5lHg1FNPrYZFfk7hv\/7rv9K1116bHvvYxyab4JaC4EoZiwSKBJolsKzZ+3rfVatWpVNOOSVtvfXW1dWLBx98cAK\/IMd6ec5znpP23HPP5IegbO+3rHx97OIqEigSWGoS6KpUCMSBQj8E\/dWvfjU5TAjnnntu+spXvpLih4dudrObJTSuaBSnoEigSGBpSqAnpXLNNdekCy64IJ188snphBNOSDbF\/fnPf05+3Nnvry5O0ZVSFQkUCQwiga5KxWTsU5\/61LT33ntXe1He9ra3pR\/\/+MfpwgsvTPe9733TkUceWU3QpvKvSKBIoEhgRgJdlYrDhU4fP+95z0unnXZasst2Jl5avXp1OvTQQ9O73\/3utH79el4FRQJFAkUCaVknGdjQZuv9\/vvvn9z25ge0g962\/QMPPDDZou\/2t9\/85jcRVJ5FAkUCC18CA5ego1Kxoc3Gtq222iq1Wq05mbRarSTMRU7lHpU54ikeRQJLUgIdlYp9Kq5B6GSFCLvxjW+cyk7aJdl+SqGLBOZIoKNSsaPW1QfmUs4777xNIlv1Oeecc6p5FvMrN7nJTTYJLy9FAkUCS1MCHZUKkbjY2j4Vcydr165NLJNnP\/vZKS679gPQaFqtucMj8QuKBCYggZLFFEmgq1Kx2vPKV74yuSbS\/ImrDuykNSxyr4olZmd\/pqhMhZUigSKBeZRAV6WCN\/eqPPShD63O\/Xzzm99M3\/72t6vb9d2rssUWWyAZCyxZd7pxzooUy2mUmdvkt9NOOyX5WkofJm2bBA0fpdWEOv9R3rr\/MDxMKu6rX\/3qSmZ5OSdVjsibrMm81zKHvPHcFFda\/IXLo9d0g26UbSnSjKd2T754e8Mb3pCizXpvwrBtOfLt5TlHqTi\/4+CgIU6vQC9eLxmOksZxAbfRqfxRpjuptBY6\/+QUjfuFL3yh102gfHe+853TJBv0Jgx0eNFmvvjFL85SXH755elrX\/va7Pu0Oy655JJqA6rL5l2S1o1fBsAgijEULwWmrrvlI3yOUrGM7EyPXyQMfOADH0iGOTa6+Z2fT37yk+kd73hH5edGOPTiSXAcMBGswt1\/G\/BDZvLSGPDFPc1g6V3He\/XbSYcffnjF7kLhv2K24Q\/FAYLUSZTRaXVDZ\/42TvqIuccB+6Xka+HAGbRe8tBmyD6n\/ehHP5q\/TrXbLQF2uz\/iEY9IpiKC2bwOyMR34\/sR\/ta3vrU6YsM9TsxRKiZeKRCKAzQIw59jjz220oyUCmVz0UUXpWOOOSa5R8XcinjjZLSe9s1vfvPZ3b31MBq5bgI29ZY0Lw0ctEzdK6+8sp5c9Z6bsuiZm\/yqwAH+3POe95zl\/zvf+c6cFL7+9a9vYtLiE785YfQi+Ak0lbNJHvzytLjFjXQ8yaObMogPUcMF6cCd7nSn9MQnPpEzbdiwIeVylaa05RFoKp\/I+AwaT+9R7ojDT5g0pS1eJ5Cjto1mxx13TMDNcukWX7h85Cff4MU7kKG0mtCNVnvSrqQTiDLm6eEBr5S2dpSH1d02qbpHuu7vHa+Rj6dySVuYp3fK17uOI7c6O\/E6R6lIIED4r3nNa6orI63+uOg6wkzYuuR69913T\/\/7v\/+bxmmpRJ758zOf+Uyiqfltt912HhVUXJMpzvwTVhHN\/CE0QyfCmnmt\/uu50EW6lefMH43H2afcn5ufipkhGel\/PGlM8oiE+eFNnYDwqPCg8USDX25Q5iZ58BOGBrjF5Q6Qh\/rVgMKv3dNZMCZ5Hh4WhNPtlIwwaUlT2t4DyucDER5+yoHPePf03lRuYb0Cn\/hFrw0AN8QoUEwAABAASURBVJ707Ny9oIkXMsR3PT7\/Ot\/8ov14ak95nUuDXPCnvXoHPOKVMly5ciWvthCPAkJgQYURwD1MfXfjtaNSsUvWbln7VVqtuUvGrdbGHbUmbm3px+w4QLC0JG0aUKHyMoxwsJFbYaPi+DP\/wNBDuDA03OKrGO4wGeVjjMovoJG\/6lWvql6lIz3g5smSU3Hc\/aCdUow0mvjHH+QfRROdXpjiwfuZZ55ZJRllzHnX2PBOJmSDMNLTcFkeGjkrVXrC63CXDj90Poqon6YPSxrSQkvOyoKfGCrxF44O7yF3fOAHLf7kNwxC9tHTU2bc0gzLi7sbxME7vpRFmcRpGmYIQ4NWHHHRmmRV3si3iU47VX70aMXhfvSjH51YItwBiirqwJPsxJeuGwbQ91LflI\/hZLRz6eDBt9aN145K5QY3uEGyW5ZWd1VkMB5P1x+4bR\/NfO2ojQrMhU0A7nkJPjVEQvVOID4kH5R3QiMobuNxp665A8x2jV0jeNSjHhXeiZufCiPs2YA2Dh+tSg5QakjxhQfuQN0vPtwI1+vr\/TVQ1kD4x1NHoEOId0+NTWPifvGLX1zN62g0yhzDrzxfjU+jRa\/+KTLuOsiOfOv+yhdljXx9VICWnOXNrTwxVIq88g\/\/Wc961uzHo17Vr3iDQDuhdMWNnl5vz81Pu9A+uLuBxYV3dMpifoO7qU0IQyM8z8871OuEX46oI\/VARupKR5vTdHLjSRtEE2lJI9per\/UtfjdeOyoVE0B77LFHclu+HvmnP\/2pNCtw83vnO9+Z3KA\/zh21GpEP10cU0PNihLA0YB+Rj4mfMSQhcQOty\/TjvuKKKxJabsiHTt7rczVRARRL3hNz8xMnaLj7gXK5mjMaW8TFK57jvc4Tfx+HIVB8uJ7RaISDeMuXL+esQLGgA+Zv5TnzJ\/gnSzwJB\/QzwdUwk3LlbgLFpl7Ukfh1GulQLJEPZYy3nC7qgUzzvPCf06pX9ZvH7ccdH6U49773vStlJU1ufmSgHNzdUFf2UYameHmY\/JrKQEbkHsjbWKQZytZvcdXbDRrfhboIKEvUCcuPBRj1oKzCIj\/1JI16HfCroxOvHZWKhNw767d+WAR3v\/vdZ\/cicL\/5zW9O++yzT3rCE57QeOBQ\/HGBMED6ehdKjrsX6IlMHvZCOyoaPUJUdDz9dpIG1m8e+DfO1uv7QJnT0qxbDBodpRVyyvOhgEzESSv3H8atLMqEF9DAIz0WYrh7eUbD74W2H5r4KMXRGcUHxc0P+uVVnGFhuBgfdbQV9cuaiLR1JKwsdc5SDv9OT3USFidlofyd6HsJ68ZrV6WCqaOOOiqdffbZ6ZBDDqmuOlBoAnjve9+bVAaaXpgZJ40hmB5eHqwRFcANLJPcilmxYkXSAwqrN169JOELy6FyVbKPpQ49dU47bjeFoJfBkzmTMMHrZcGHusk\/dHXHH6ShJ+MGysd7vXzeDXPQ5CDjsJY8vefhzHM8hl\/01uRLzuHvGbz7YFgmQUv557TyUL\/i9AsKVOfYLZ5OCi26Tgiegybeowzh3+151VVXpeBLx2B4IY5yKz83aH9gqGYIxW8Y9Fvf8iKXbrzOUSoOCro+UgI5fLAuvD7uuOOSQltG1phbrbkTuHm8cbkJF6TPFKQoQiPzZ0UJA4rPB8TNZNWDi+PdB8qU4yYwQzruQEziiY82\/GlrvZwlQCZl+E\/yqcFpePLEAwXDHVAuPAI3f3XHstHwvQOZeJIbcIMhkrjtLBoKK8x48UC8AHmRm3dKQiMG7+RM3tx4Z5pzxwcTcqeAYvJWuHqt58O\/F1CYwY+Pl7LMEQoXDdpuaeIZ7+g8vXPrsChG7n4Rikm82IvCDWFBGaqRPb9uoIRjYledk+ug9V3Pqx2vc5SKYYSf3LDhrWlytp5w\/o5evNxvFG6NSK+ngQdYStImqDAF9abRMCiSoNW40QpDw61RRS8qLbQavAYlPEBxxiRiniY3mnyyzvskoGEotw\/OuBvvnt7z\/MmsXsacVnmBTMhG3JAFupBbPsmIJkeTHMWFkJE8TLD6EEy64p2c+aML3vkLR0fuZCuvvP4jTf79Ij5K+ZBhPX58bPx9iCxc7nYgb7znZUCbT0J77wa\/7Jl3ctKDkL\/4lAMLqh3vaCCvP2loA+QnjDzJdZD6loa0KNtuvM5RKjax+a0fy0\/3ute9ksOEP\/jBDxILBmN1sGouvfTSig796173ujrJ2N41SmNEgopM9MQaerzHk5+weGet2BUcHx1\/6RkqqDjvAcObeu8uzJxBnia\/SUB5X\/KSl2ySFV6Apw9Wg1TGdnMqlIiy+oDFUY6I7x3IQbmV33sT5GEViXybwqWZ54N3FlUud\/G88xfuHfBUT9c73oX3A1aRj1KcsIa4c\/ho8MHPCouVTe52qPMS8vLRtovTzn\/dunXVFa0Rri36kD35udZVvYZi4NcP1AN5RhxufvHuGfzn9a3D5i880I3XOUql1Wqle9zjHumDH\/xgNYdCYzO3dthhh0RpcAd23XXXdPvb3z7d5z73SW984xsrertxI\/Nhnwqem6d1d95Y87wIpU7LL6fhjg8iaKWnEmO5Nm8cGnv4B30eLr068vSVpR5ef0cjbXzEx44mzzvy9EQb8A7xHuWVjvTCP57yknaOPD465ZV3TtPOLT9x6pBmPU4ul6CnmPjXaevpsnjqcypB0y4NaUpbuPzIg1z458hplN2wL+LII6cNNzlKE8TJ5cXNT1hdDhEveMEPN1rgxo+nd0dhPMWLvOOZ54OmCfX8xeWX0+JVWsIC3vkHnTjdeJ2jVCIxW\/PtmP3c5z6XjO0e9KAHpRve8IbJtQcsF0\/LnoZK73rXuxKLAb14kUZ5FgkMKgFzQMx3MLcT6ei9wbvOTQPnLpgeCbRVKsGi7fgsFBryrLPOqn7\/h9YyMXXGGWek5z73uWnnnXdO+Rb+iFueRQKDSoDFCOIbylEuYM6An2HKIMMgcQvGK4GuSmW82ZfUiwSaJcACCdNfJ1aHYYnhQXPs8fnKU974aTckGl\/uCyPleVMqoxaPlaeTTz45mZnWo9nl66dD2k0wR\/5WBOD8889P5\/cAP6TWDpFmeRYJLGUJLBql8va3vz297GUvS4985COTO17s+H3Sk56UXNOQOvxzwRTY2NcLLK+2w2677ZbaoSlOPT98BNavX5\/WZ6D4IFd8odw6FK8EFQlMXAKLQqn4bSJLbq5n8BOtt7vd7aqfY73b3e6WnE1y8HHikq1lGAogf+YKgpvSCJgczxHKJldEoahCkZn7gl122SUBq80F5WC59A53uEOC7bfffva4Bauu4NZLQh61Jjm210WhVL773e8m2\/Atd7VaG3f4WoW6613vmr785S93vO3K7kebnjphzZo1aU0XmFRswqpVq9KqGrbaaqvqR9jiOaradVIcNt988wTmJVxbATe96U2TXZ5gd\/StbnWrxJoDG7ic3zLpDvYmmRyFz372s9WJZnMIo4IJ\/1GltWDT+d73Ri7XTrIYVRvrJZ2elYodfZbybKb60Y9+VF3KNOmLmdoViELxMTlVndPYQ+Pu3J\/97Ge59yZuH\/zvf\/\/71AmbRGjz4iNuwvLly6tzRsuzJ2WXw3K95fgA5ehDB8O5+93vfikQyi1XYMqgrGB3JrRhcxPvsJrCSgrLiFUUFhFrKCyhf\/\/3f09gh\/FznvOcBDZJBmyhD0vL02YzcDI4hy0J+XtxX5LGLYNNKn7ML12ViglQN7tZNnZozPkfuw1dK+nDsOmt22TomMuQfBx65V4\/pnHz0y19CroOZ3gChmsuvQIKk\/IORNq5AqOwwuqxIRHWrl2b1s5g\/\/33T1YpnCZnja25zuKilCgjiDS7PR18A7+oYF8S2HEbeP3rX58opIAdp+BMVo5XvOIVKb9U3XvA3FgdVluacPHFF6cmkFsnsGLbYeXKlWkxolvdjjK8q1JxlkeFP+QhD0kvfelLq\/snMGCHrXG7A4Z6JX6DggX0gAc8ILkV\/La3vW3yIdDceXo5jfkBvePvfve7WRKKzbyKeQRzBFZ\/7KWZJWjjOP3006sLl2xHzsFK6AY7LtvBx9oEw48VK1akFRkohUAbNnv23rBhQ9owA0cngMKlBPixxiIhSkme+GAptVNAlA9EWSivSGPQJ8WErwBLKaAt1REWVP0Zyqv+DCur3ZP11Q5hlXV7tovPnzXnCe14YO1FWJ1\/76xAT1ifTdhr9\/k7N7+6zMJqDLmq70Hrq994HZWK3tREJ9P7BS94QfXRt1ob5yyMy0866aTqgiY7avWk\/WaO3vZjqzTbbLNNosDciesD4OdwI5pvfetbyQSsE7GuW9DLnXLKKdUPnFEmzibY4evksOFCrP54F95pY56GjaYO8xDdYF6iHSi+JpClnaA5HvzgB6eAu2ty2OAVEBcchgMnpAMUMlBawHIju24ga\/XcTgFpjJvPzNFQQMA6pYQM1WKYRub4gtwSoowgFBKeDdFYAp4sy0A3PqctXLtph1xpxkddf1L04VdXCN5ZgZ6QK1NWYf7OzY\/yyRFWYygubWJSMuyoVJzSZH47gGVnbZ0pfsIGvaPWh2yD07bbbluZzVYmnCOiwIy7CVae7m9ws5wj8M4isGSe8YxnJAfQ0JlLwau4lE+s\/lBCy5YtSxqudNpBI59WKG\/ABwo+Zthnn33SPtfBmRgw1AHycQ8OkAmwxFhfYV2tWrVq1mJqJ5u6PwUEhmoxTNNGdCoQ9KGMPCkj0F5YOhSxp2FaQDnWrl2b1mYwXFPXOZQNbRPQN0E511w37Gv3JNdukE43KFc3hIwW67OjUjH56fIjjaedAISha7U2WjDt6Jr8r7766mrVRkVRGkFjC7Z3PYGxMY1uqOUEddDooYX5KYvIf7PNNovg5PR0wLzQbMA8O+rDun7Z6Tc+eh80rFixIpF1fFgUDEXDaqJ8gPIB\/sLJmRIST3xYvnx5Wj6Dfnmv07OScrCWchiuaQM59PB5nNyNvgm98Er5dYN0uoEFxprrpKAoRJZerkDrbp1GrkzDrU7CLR1gIXrmwEOuYPFdl\/+43jsqFR+2vR7MK71RnQkNVtgd73jHhLYe3u2dwjIJrFfNaR3xNpzRs1Ecjq0bHuU0W2+9dZWn4+mUk8OOTEVXL5hLcS+HlZ9rr7022ceSx627zcFMCuZ6hsmr3\/i90Oc09rQAq8+w6+lPf3oyycqMPvrooxO8\/OUvT8D0BsNfMD8FLMyAE64Bl\/oEckXBzfrJUa+jhfSuHLmya3Kz9HIFWnfrrMmlDt9D+EW6LMRwx9PwN1ewOoVJybCjUmm1Wulxj3tcxcv973\/\/aoiC0Te96U3J6WR+CoGGtVAR9vGn1WpVv65GABHNcOb5z39+MjtvIxchqiQmc9DUn4RsiGOvxWtf+9pEY9u70gtfndb2S1hveylc3AP2tICDpgF7XQLaTcAtaTle9KIXVXNk8WQ11cGCagKrqh307O3AAusFq1ZhsjgSAAALjUlEQVStSqs6YMWMBdgLls9Yd91Qb9sL8b2jUlEgE7LmPazOfOlLX0qGEpaTrcYYklj+Y6mgHQbSdXaHonJfhi33dn72mmar1aq26NsXQaH4BQBzK73GL3TTL4F2H2SnD7qTMohhYLdnO6UU\/u0UWt2\/SSHW\/eqKtP5ep6+\/1\/PM3ydVw12VCkbMZVg6\/sY3vpEolnPPPTcZ21qBMcGJphvy+zGY\/\/kdGYZRttjLg3WiZ3NRlDSNc82VMBe912GIZPXH8jKrph7Oguq0+lOnL+9FAtMsgXaKNfw7KdhJlasnpWL4wQIw9LHSYvz3mMc8Jplc+sIXvtD2qsm8EJQF6yZgO7hwczXuyLDrlWlsmZp1JAxsLze0sYvXe8C7FR\/WiKFRjCsj3JPis5RmNt57QZHA4pfA\/Jewq1IxyWYij2Xhol97Rtxhe9555yVh9lQ4y9GtKKwNd1EEtthii0oZGfKIa9mYddJqbbqKZALYZLGJP5O3aMGNdBQOpQImbv0mimVq4RSh8b240uBXUCRQJDB+CXRUKmag3VFLARjqUAgf\/\/jHkyHH+9\/\/\/uQjNmZ1Ly1LoV92bRIyP2P5TJpWeXJQDK1WK+27777JBK5byq3sUEQm+R72sIclQzPgtuEnX\/2x0iBuq7WpouqXz0JfJFAk0LsEOioVloF9ICakzINQMt4pEntJzGWY5Bp085sds1aP7KJtWte31VhRTASfeOKJ1alOKztuAWctGX4JB27LnsLQmKx1vEBc4QVFAkUCk5FAR6ViKEGRmOzEDiXgMKHDheHHf1DYQWsY1W7p1u7JSJvSsaJDWZjfsVzMuolwbn7C0KAVJ8LLcxolUHhajBLoqFTclm\/S1EdqydfKj41kzrQQBj9KxnwGWn4FRQJFAktbAh2Vik1p5iQs8er17a501scJZcMgG+DsoDSRi3YxipIlFStVi7F801Ym82h+UtflUXe5y12SXyTUeU0bn4uJH\/vCbP0nc4sl5iyHkXlHpUJwrjwwKbpixYrqRLJKpkDsYmXBHHbYYcl8BtrFBJPIxxxzTHJa2tL1YirbNJfFJWD4c7fw2WefnXReVvH4FYxHApSI80O2ezh2YwHGPOmguXVVKq1WK+2xxx7J2Q5XCcQZHCs25i\/MY5jPGJSBccfze0WWw+v50MSEmd+\/kt++b8OcQ1mU6sa45W8\/EhhU7vZA2VWtTVkIcEDPpst+8l6qtIPK3BkvizGtVqu60dFcKtkPKseuSqVbwhSN4Y9J3G60kw63U9fdKk35Ol7gKIC7QNAw\/Vglekj09rZQpkxw7wW9S2AYuT\/zmc9MzuPIzZ4oit5qo\/eC9hIYRuau1nAg182OTqxbQBlmw2hXpUJZ2D3rRjbLynUYDtnYpmdvX+TJhrznPe9J5oBYU4Rdz91ks1viHA1wdsLmOXtgbJRzKZUVr3qc8t5dAqOSuyMXtgYYVj\/lKU+pLujunvvSpBiVzE1pOONnDtHhUNd3DirRrkrFEMFuVmMuPQizyD6QBz7wgdXVkrbf22di+\/6gTIw6nr0plIQTr9ttt92c5M0FOR5A6bRarSrcqWjDnW6371fE5U+jBEYhd5OGekybHc2v7LXXXqnV2lhHjZkucc9hZf7973+\/ui84hpgsdB2yjaM10fb82lGpOKBn3sTMsM1nrquze9UYzLu9ICwZ4+Cec5wAIauK0jN8cW6oniWFYp9NXREav7uDxTmkepzy3l0Co5C7Yanb4NzbooF3z3VpUwwrc984y9DWEHMpNry6FM1c46CS7ahUbL03VDC+arVayV4UW\/Uvu+yyKj+Tti7zcVEP2spzAfyxcsXca1I4C4D9BctiN7k7sMpSPO2006p5FeN7MFRdsIWeZ8a7yVznagXXthF3CLt\/mAXvoq5BWe+oVGTodraYY+CmYJwQjgx9nJafaLzwW0xPAjarvpjKNK1lcc2Fn3xx8t2yZsBk+rTyvBj4cpez+RRL96Y63HXcag0+5OyoVBwkZF6ZDHIi2dWMGJAxM8myrPkJyqfVGpyJSVeMeSEmX5MiVJZpmnSetGzGmd8E5T7OYiyotOdD5h2Vig\/MFY12Obri4KKLLqp+Kc9ZHcMe5pL9KyZwF9JQoty\/Mj\/fRZH75OU+HzLvqFSIwJo1M9SVAuZQzDb78SJu5uoBBxyQbN9vtRaOpWIJ2Xkl5TI5pZwUJ\/PPsnKZICSR0aPIffQy7ZbifMi8q1LBtH0oZoN9bK1WK+22227VD3\/5KC3dmldBt1BgBavcvzL52ipyXxoy76pU9OQ2kNl96pcBm2Cp2VLs5EU2eI42Vk3J\/SuDF2IBxixyn3ylTVrmXZWK5T37PexLccNbE+xXsRw4eXF1zpEFZVa7afXG2RLnluzDMdmsDDb9dE6xhPYigSL3XqQ0WpppknlHpWJ1hFJxf4ptuyZomyDM\/pXRiqmkViRQJLAQJdBRqdjQZvPb7rvvnlwfuRALWHguEigSmKwEOioVE7S2ssfmt5TSZLkruRUJFAksOAl0VCpbbrll2m+\/\/ZKDXc7LLLjSFYaLBIoEJi6BOUrFKo7VnFjlcbjo6quvTnvuuWeyES788yd68SbOfcmwSKBIYOokMEepWMWxEhKrPE6NOoZOsZx55pkp\/PMnevGmrnSFoSKBEUugJNddAnOUilUcqzlNqzzt\/NCL1z27QlEkUCSw2CUwR6nkBbb64xcDbWHP\/Yu7SKBIoEignQQalYrTx+ZJnPNZvXp12mmnndJBBx2UXMjULqHiXyRQJFAkQAKNSmX9+vXJHaHbbrttco+rS6HNpxxxxBHJlQGp\/FsQEhg3kzofu5VdnOykOqs28rQN4clPfnJat25duuCCC6qOySXpEZ4\/WcKukKynkdMU98KRwBylYhetiVdX+vkNkOOOOy65ectJ5HPPPTcNc3flwhFL4bQXCbisy107TrI7WOrujojnrlnKxOHT8CvPpSGBOUrFPIpdtLvsskt1sTUxtFqt5JfLHC688soreRUUCSQdECuD4nBDnsvDQywu9XIfz\/bbbx9e5blEJDBHqbQr97Jly8qt5u2EswT9DWX8ANWvf\/3r6qdJ86GLzse1GO45df1ov+L52Mc+luo\/BRPveT79plvoJyOBZZPJpnsuhWJhScAPyB1++OHJRV1+ssXcSQx\/XLbsAmuXVrda11\/eZa+TeZc6LACYnwkJ7Ljjjsmp+BwWClg+K1asqC5gD9rynD4JFKUyfXWyIDhyd7HrRF0hYYhjCBTDH78sqBD1XxY86aSTqh95s6KYg4LyI1bigFv5\/MRKQDqnn356ckmYxQLH\/NEVTKcE2iqVeq9inoVZazt+U09zzTXXTGcJC1cTl8DZZ5+ddthhh0Q55Jn7WdnPf\/7zqY6zzjortft5WRYMZXT55ZdXK0kUWJ5mcU+fBNoqFRWZ9ybuojUpd+ihh87pbfbdd99UfoBr+ip3PjjS+VglNHFruJLz4Cde7LyuwzWTLJ6cllsndsoppyR3+vh5WufP+C9KLKJCzVEqxsXr1q2bM6bNx7d1N3rxFpFcSlEGlIBb9Fi55kUGTGI2msOsOjfDrGF\/i2Y20eIYuwTmKBXjYuPjGM\/28kQv3ti5LRlMvQScAzPfYuPkMMy6asNEMKvm2GOPTaXTGkaak407R6lMNvuS22KSgH0rfmjOqo8LvgYtm3kUmy7No9h0eYtb3GLQpEq8eZBAUSrzIPQFn2WbAthtbXl45513bkPRm7djInZ1Ox5iXsUlYYEzzjgj\/eIXv+gtoUI1LxIoSmVexL44M\/XLBIY9K1euHKqAF198caJMPvWpTyXnhw4++OAUOOyww5L7fYbKoEQeqwSKUhmreBd34g4S2o5\/4IEHVgX19JMo9X0kdbqKOPuDXjzzMeZQHFJsd3eP\/KSXRS\/OKZNAUSpTViGFnSKBhS6Bxa9UFnoNFf6LBBaYBP4fAAD\/\/2FCGzkAAAAGSURBVAMAiSm7JV2aTIQAAAAASUVORK5CYII=","height":167,"width":277}}
%---
%[output:580f7b78]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARUAAACnCAYAAAA\/m8goAAAQAElEQVR4AeydC7xPVRbH1zGjMcM0GDOFHh5NDankUZ6FMXlFb5NKLxFKg8qdaZRHJCrpM4SMikKjpqIRkjxS8hqPlJTKo1CTZ5lEMve7s2\/H7f6f9\/z\/Z\/\/vf\/lYd++z9z777P3b56z\/2muvvXaxw\/pPEVAEFIEAESgm+k8RUAQUgQARUKYSIJhalSKgCIgoU9G3IKsR0M4Hj4AyleAx1RoVgaxGQJlKVg+\/dl4RCB4BZSrBY6o1KgJZjYAylQwafm2qIpAJCChTyYRR0jYqAhmEgDKVDBosbaoikAkIKFPJhFHSNioCGYRAyphKBmGgTVUEFIEAEVCmEiCYWpUioAioRW3WvQNvv\/22nHvuudK4cWM577zzpHbt2nLllVfK1q1b48ZizJgx8uqrr8ZVnnK\/\/\/3vj3rewIED5auvvpJ9+\/bJXXfdJevWrYurLi2UGQiopJIZ4xRoK2vWrCkzZ86UhQsXytKlS6Vq1armOtCH+CqrVauWqZ\/nLVq0yOTcf\/\/98rOf\/Uzuu+8+qVatmklL5s\/hw4fl0KFDydya2nuyuHZlKlk8+HT9wIEDsnfvXvntb38rBw8elGHDhhlJBmmGOGnQ0KFDpW7duvLHP\/5R\/vOf\/3Cr8EE\/\/fTTRuJp1KiRWAnEZEb48\/Of\/1yuv\/56eeedd+SDDz6Qm2++WZCevvzyS+ndu7fUq1dPzjnnHHnwwQdNe0i\/7bbbTNo111wjnTt3NuWRlrp37y5t27aVxYsXy7x580w7GjRoIBdeeKGsXbtWPv\/8c7nsssukX79+QvuQyCZOnCjNmjUz9b322msRWqnJhUFAmUph0MvQe9966y0588wzpUqVKnL66afLf\/\/7X\/NBzpo1S7Zs2SJIE9DOnTvl+eefF9I\/+eQTI9lwDROi65Rh6jJ37lx5\/fXXpWTJkjJt2jSyolLZsmWlXLlyAsOwBZcsWSKnnHKKYRA8b\/369bJt2zYZP368kWTIR7r54osv7C1C+6ZMmWL68sorrwgM7s033zRMizoouGfPHqlTp45pX4UKFYR658yZIw888IBp67fffksxpQARUKYSIJiZUhXSwJo1a+Sjjz4y0gK\/4vfcc4+88cYb0rp1aylevLih5s2by7Jly2TBggUmHSnjl7\/8pfnVp6986HzUp512mplCjRo1SlauXElWwtS0aVOjd5k6daqZEsGsPvvsM3n\/\/fcF6cPzPKlYsaKcccYZeXU3adJEaM+xxx4rffr0MWUffvhhmTRpkmzfvt2UQwJD6vI8T4oVK2ba\/pOf\/ERgbPv37xdI9F+gCChTCRTOzKuMD+z88883H9fXX39tmIntBdMeGIy9zh+iaOVjhjlBMIL+\/fvnL\/ajayQMpBSkFZv5+OOPy9ixYwUG1atXLyNBMb1CXwIzoBxxpmvE\/YSSuVu3boIU0759e+nSpYs\/+6h4tP4cVVAvkkZAmUrS0BWNG\/lwmcaUKFHC6FJYrYGZQC+\/\/LKREtClECdt165dwnSH3qP7YLoBg4AhobtgGkReJKLc6NGjpXr16kaPY8sxLWnTpo2gRIZJwKCQjJiy0D7auWnTJkHCsvfYcMeOHUZiQZeCBGLbZ\/M1TC8CylTSi7cTT1u1apW0atXK6FHq169vpj05OTlyxRVXmGkB0yHoxBNPlJYtW0q7du1MOtOQG2+8UcqXL2\/6wfQIhkPIVIQPmrjJ9P1BsWufR70wC5SyviLm2YMGDTLTE3Qe6HyeeeYZo5iFUdXLVeCy\/Hzcccf5bzNxdDFlypSR83MlrquuusowrNWrV5vpkCmgf9KKgDKVtMId\/sPQSaAL4UNliRelLQpOJAKmBkxnyIeIkwYTGDBggNGvvPDCC\/Loo48KzIO8Hj16CGUhW97fS8q99957RlHK81asWCHob0qVKiUQUx7ahN4DnQ5SCcxj5MiRMnjwYEESguFR\/yOPPGL0IkybunbtKhDPon2sVFE37YPxwZhgYJMnT86TiFhRoj3cwzN5Nm3gWik4BDKPqQTXd60pAxDAcA4m07BhQ0FfwrKylZQyoPlZ2URlKlk57JnT6Ro1asiMGTPMFA1Jh1WizGl9drZUmUp2jrv2WhFIGQLKVFIGrVasCKQCAffrVKbi\/hhpCxWBjEJAmUpGDZc2VhFwHwHnmApb4tlfwkYwtuZDxEkjz31ItYWKQHYj4AxTwWIS24lrr73W7NtgS\/xzzz0nEPYFMBSWE6dPn57dI5ZRvdfGZiMCzjAVTL3Zh\/Lss88KW9rZA8JmMKhy5coCs\/nXv\/6VZ82ZjYOlfVYEMgEBZ5gKO02xqmRXKlvx2SOCNSSGT4Rcw3QwC88EYLWNikC2IuAMU8GvBRvSrrvuOuNkB1NrnOiwaQ0T7eHDhwtTpGwdKO23IpApCDjDVNj9iqcurCeZArHFHU9iF198sfEohn8MtranDlitWRFQBIJAwBmmghTCFIjNYWzDx3\/pT3\/6U9NHpj1cU8Yk6B9FQBFwFgFnmIqzCGnDFAFFICEEnGIqeA9jiz2+SD\/88EMhRFFLGnkJ9UwLKwKKQCIIBFbWGaaCr1F8c+Ds5w9\/+IPxuUHIlnfSyKNMYD3XihQBRSAlCDjDVI455hhhmztLxgUReZRJCQpaqSKgCASGgDNMBWfISCOXX365WVK25vmY6HPWzIgRI8yJdoH1XCtSBBSBlCDgDFP5zW9+I\/gknT17tjmwihP0MNuH2PdD73FfSKiU9QgoAA4j4AxTsRhhro\/1LEzGpuHsmDTybJqGioAi4CYCzjGVX\/3qV\/LNN9+YM2A4qgF66qmnBGM4VdS6+RJpqxQBPwLOMRWM3IYMGSKY7V955ZXm6AaO3Lz33nsFwzh\/4zWuCCgC7iHgDFNheoMpPhBhWcuxDDg65tgGjmrgXBfybBniGULaTEUgqxBwhqlgkv\/SSy+Zs1zYQIj\/FDsSxBcsWGDyKGPT4wkx7Z84caKwMZFzYCZMmCAcnxnPvVpGEVAEEkfAGabCyk7nzp3l7rvvFo7Y5CiGKlWqCHTBBReYIxrYYEiZRLrJ6XgcLMWqEitK7HxetmxZIlVoWUVAEUgAAWeYim1zxYoVZcCR0\/AwzYc4pQ5zfRw22XLxhjAQ\/LQwfULRi2Hd8uXL471dyykCikCCCORnKgne7n7xDRs2yAknnJDX0KpVq8rGjRvzrjWiCCgCwSJQ5JkKcLGiRFgQderUSWIRUzDIX45ryJ8WKU45yJ\/PNeRPixSnHOTP5xryp0WKUw7y53MN+dMixSkH+fO5hvxpkeKUg2w+cUs2LVJYUDmbRhjpPptOGcheE3JtietoVFA5m0YY7V7yKAMRh\/72t7+ZKT1pEGl33HGH9OnTR5jac+0nykAFpeVP95exccpYKuj9T0VakWcqlSpVEnY8W\/CIV6tWzV7KvHnzZPz48VGJKRjkL8c15E8j3rdv3x\/VRTmIfEtcQ\/Y6Wkg5yF+Ga8ifFilOOcifzzXkT4sUpxzkz+ca8qdFilMOsvnELdm0SGFB5WwaYaT7bDplIHvN+HBtyaZHCgsqZ9MII91n0ykD2etf\/\/rXwrUl0llAOOuss4StKFz7yV\/Opts0QpsWKaQMlPfCpyHiJFNZu3attGzZUs4\/\/3xBsdqlSxfBCC4ZPOrXry8rV64UrHEh4jVr1kymqqy85+OPP86qfsfq7Oad+2Xo7I2GYpVNR76L4+McU8FeZezYsdKzZ09BaYtU0bZtW+HIDvISHahatWoZ5tSqVStp0aKFsKpEWqL1aHlFAIbS7tGVMmXZNtm882sp23ueLNqwW4HJh4BzTAVpAh0IR3TQVlxJwghwMUkeaYmQ53nmeI9FixYJq0icHeR5XiJVZHVZjkfJagCOdN4ylFV96ws0qkM12Tm8qcBkwmQsLo6Pc0yFZV8M4bZt23ZkOEXWrVsnOL1WM\/08SDSSRgSmLNsuNQctloZVS\/\/oqY1y08JmLD9qVMgJzjEVGAfSRP\/+\/WXFihXCtOXWW2+VXr16CQwnZLycenw6GuPinD0d\/bbPQH9yy5R1RipBOrHpNpx+y9km79Zn1tmktIYujo9zTIURwcsbFrCLFy+WWbNmmWkL2nHylNKLgIvidaoRgJG0G7XS6EzQnzDNifXMk8qUEO6JVS7ofBfHxxmmwqmE7ErG4xvEyg9n\/uD5DZ0KeZQJelDCqg+Rml9AXkQI8doSCkCleeajDhqH2n\/fGLNeGMmJZUsYCQT9STzvCBLLog93x2QsvXv3jqe6jC7jDFMpW7as\/P3vf5epU6dKu3btjEEanuAgpkOtW7eWcuXKZTTY\/ALCOPhQhs7+WLbkLk\/y8kId6pY3CkBeYn4ZlZqajzpoHFb0qBSzXsagoKlOrJePtsbDWGLVk+n5aWMqsYBilQdvb4S4N0BCqVChgkBXXXWVrFmzxihrY9XjYj5SCcyEX0BeWF4+Qn7deHmhnBaVXGy6uDhndxKoI42a3v1sicZYhg8ffqRkMIGL4+MMU7EQ4zGflR727Ni0zz77zDAUGI5Ny5QQ6QSpxEoimdJubWdyCDQ6pbSRhKIxluRqzpy7nGMquJO87rrrpHv37tK4cWNDHTp0MLYmTJEyB1oxy5BWOnFVEomFp4uKwFhtdiHfSizozVLZHhfHxzmmwgCgmF2wYIFMnjxZ8E+LR\/1mzZqRlRGEoRTTHRrLNIdQKbsQsBILU18jrc7eaJS4gz45W3g3SCuqiDjHVFjhYaWH0wmvvvpqI6Fw7g9p5Lk+EFv3fmteGpYYiwJDcXHO7vo74G8f7wDTX4gpEXn86HDNqh\/XhSEXx8c5psIUh1Wg5557TqBx48aZw8Wso6XCDECq7+VlaTvhE8lpUVlQwqb6eemo30XxOh39DuoZb+QuM0eqCyaDJBMpP550F8fHOaaCMpZVILy8QewBYm3\/u+++k71798aDcyhleDkQa28+p3QuU3FzJScUYLL8oVOW\/rDdpCAokFgKSs\/kNOeYSkFgbtmyRT799NOCsgpMw70BO5s\/\/\/xzSYfjaxgKCjmWiruc++P9IQU20uVEX9tcFK99zXM+ijSSyka6OD7OMRUYAda01lsVIboV3BXgZzbWAKF3GTp0qOzfv98UTbXja8tQmPKYB+ofRcCHwEllS\/iusiPqHFPBM9b06dONtzY8VkGc\/YPkwRQo2rAcPHhQRo4cKVjfMoWibCodX6PBR0KBoeQ4arwGBoUhF+fshelPuu\/NydWvRXsm9kvR8mPluTg+TjEVzvdZv3694McT59RILdB7770n+PHcsWNHVIxfffVV+cUvfiGsFtmCGNHFcnyNCJko\/fWZlcJ8GLPv9qceNpan1MFpioRKH+dh4hIW6R6feuW+ltoVC5ZWKhz7U+Hd4b3\/3\/\/+J5s2bUoZZvZ7SEfoFFNBqvjHP\/4h7777rjz66KOCz04IW5XmzZsLK0MWFJiNmSZVqWIYDi8LO5pxPYmy15YjxOkTYSSC2ydCU9\/3ZOaG\/bkKr5O8owAAEABJREFU2cqS\/z4YWP60TL4Gs0xuf\/62hzE+c26vb94VpkJ+wm8Q7StVqpT5MTz55JN\/9D6RH43iHR\/KpYuKpetB8TwHo7fBgwfLQw89JEOGDDEuJHEjCf3pT38SBsHWw8oQBnJMjx588EFBmuH0wtq1a0u9evXkrbfeMhIL\/llwdm3vI46LSnudaMiUJ9OtZBPts5YvPAI5udNjbFb8hC0TK4aFr92tGpxhKihYYSiEKFqxoMUFgqVYxm9IMjAYCIYCY+Fkwosvvjgwx9foTyxDcWsYU9cafiVTV3t214wtE4zljrdKJQ2Ei+PjDFNBsYou5aSTThLcHSxcuFD8RBplEkWfVSOmSXiQS9bxNUZt\/KJgyMQvTaJt0PKKQCQEYCx\/OLkwPpMj1RxeujNMxQ8B+hKsau+66y6xhPQSr\/EbUyP2DRF6XvKOr2EmTHdgKPyiZCNDQcnqHxuNB49Aq\/JfJl2pi+PjHFPBY37Pnj2F4zgaNmwols455xwpXrx40uAncqO1PYGZMN3BqI1flETq0LKKQLYi4BxTgZngmAnXB23atBFLKHFRugY9UF816mN2j8JA8MgGsVTMVAdmko3SiR9jF+fs\/vZle9zF8XGOqbBsjO6EYznS8cIU+98XYt05wkQgGAmUjudn1zO0t9mAQOhMBT0JOhSI+K5du8xyMKs9THniXf1JdrB+8Z\/HxWV3jsn2K6j7XJyzB9W3olCPi+MTClM5dOiQzJgxQ\/CZwlLw5ZdfLtAFF1wg+KO96aab5P3335elS5fmrQAlu\/oTz4vD\/qKgiD4FVZcL9Wh\/qkiqx+HZZ5+V1157TerWrZvws+Idn3i+g6DKpJ2pYJI8cOBA43P2xRdfPIpxYF8yceJE4\/ENOxW78kM4YMAAwQMc+3uC6jz1YNei9JEoBuFhMGnSJBk\/fryxJE\/lOPC+p4PSzlRQtvbt21fwQ1vQiYOlS5cWfKiceuqpUr9+fWEKhOsDFLhYzA4aNCgduIiIPkYRUASSQSDtTIV9OSwNYzmL3sTqTGx44YUXCicTYgjHzmQsYh955BE5cOCAdOvWTXbu3JlMP\/UeRUARSBMCaWcqtl\/4RsFFJJ7y0ZdMmTJFWrZsaexS2OPD3h9bFh0M06Zvv\/3WJmmoCCgCjiIQGlNhlQdvbh07djQHhlWsWFFuu+02YQs4ZvVIK3\/5y1+MRe0NN9wgderUMToVlpwdxVKbpQgUJQSS7ktoTKVEiRLGOxsHhdnWMyXiIDGmPWjCUdZiUTts2DBp3769kI4+xpbXUBFQBNxDIDSmgpKWo02RVDg0DJ0K7g0gvOjv2bNH5s+fL3h9QzN+\/fXXG\/TQx5iI\/lEEFAEnEQiNqYBGkyZN5IknnpA777xTxo4dK1OnTjUnEm7cuNFIJShlf\/e730nJkiWlc+fOgqUt9ykpAoqAuwiEylTw8oabyDFjxpjjN9hI+O9\/\/1s4+hQphiXlGjVqSI8ePQSphc2GFkqUtqwQvf322zZJ5s2bJ0g8DRo0MEZ0W7duTYs3\/bwGaCQsBPS5DiEQGlNBClm9erXx8IZtChujcFqNYyXM9dG14Lpg06ZNwjL0vn37zM5lsMMX7bXXXiuzZ8\/m0hD1IfVMmDDBTJnwU4v7hFR70zcP1z+KgCKQh0CxvFiaIywTo6zFUbV9NHoWjONwqDR69GhjBPf4449Lk9xpEgzGrvzgcR+zfnx62nsJv\/nmG9m9ezdRowSmPvzesnTNEjbXKICXL19uyugfRUARCB6B0JhKuXLljI4EyQJrWfb64E4ShsEqD35n2XMxbdo0WbBggfTr1y\/PR+3ZZ58t7BOiDgsJDAdjOhS9VatWFexesIGJ5U0fF5GJ0NDZGwWyz9VQEVAEjkYgNKbieZ6xS4EBoB+Babz55pvC1IalZHQjEBumLrroIoFBsOR8dPN\/uGLPBPuGZs6cac4MQlcDY6LuaN70n525QBKhhybNkgdeWCbluj4r+F4piMir0LpXpM1hml4l9Zv0+EHKFOKwvE6dOkn16tVT+m788LWkNpZ2psK0B+aAqwMUr1jRsmSMPoWp0DHHHCPoS5BgWFq2hH4EaSQSHDCV4447TvBx63meMZbjnKDjjz\/eMBl7X35v+qUWDZPPJ3ZNiL4Yc4VAO4c3lfzEOUB3XlJX6rS5VnZfPN4QbVMKb8OeH\/u5c+c6t3nyiiuuEH5Imar72xpk3L7\/6QjTzlRQqLKag06EfT5II6z0cF4PytnTTz9dWFJmKoO0sWrVKrMaxHIyCttIoJx44okCs6IOyqxYscJMr7DOXblypcDAIOI1a9akSMqI4xhwPwnDyWlRWfAql7KHpbhiF\/11pLjLGVW9i+OTdqYCc2CvDys3rNAwXcGlAfT8888bBsJSMW4QBgwYYBw2wcVhRKwKRRpxdjVjIIc\/FpaUn376aendu7c5AwjGgvI3WW\/6kZ4ZTzqHR23e+f25zvGU1zKKQKYjEIuppKx\/SA0oaGEy9iFMX0gj77vvvhP0ITYvf8ipbhjMnXHGGSbL8zxjMMexHuhm\/vnPf5rT3jwveW\/6puIs\/1O5cuUsR8Dt7rs4PqExFQzcWAKGMWCkBqFHYXqE\/gRp49577zWSBl6xSDv22GPdHuECWtewaukCUjVJESi6CITGVFiRwb0B0gj6E\/QqWNiuX7\/eSBhMkVi9Qf\/BPiD0JSh5Ex2Kw4cPC6tCOHtq1KiRwLiSqSfR59ryTH9sPBNDF+fsmYhjqtrs4viknakwvdm+fbvBGMkjJyfH+KGdPn26MXZjw+CTTz4pLLOhzLWETgUpxtyYwB8XLGpR2LL0jMI202xcXBSvExj+Qhd1vQIXxyftTAUHTLiF7Nq1qyxZskRwvsTAoVtBUmHKw3o9PlTQj1hCcqEMZRMhlulcsKiFsXSoW144nCxTGUwiuGd6WWsQ2W7USrN6xw8C8UzvVzran3amgiTCjuO7775bXn75ZWnatGmewQ9Wsrg6wDE2ZYIAIJZFbRDPiLeOnBaVZFXf+sa2BQbzxoZdAoOxxIsL8fLalxrJBor3GVouOgKPLdltLKItvmANgbsdB8I3PtxtKuJMKMaM8dq8a78ZL5OhfyIikHamYluCpzeWjJEkrJEPqzZ4zmefjy0XRIj+Jlo9zEuDok8++UTiqav9qYflkdalBWM5S61OKSFQmWO+lfe37jb01OItAtXo\/7pAvPBK88zHHS8O4GbphbV7DJ4WX7CGXrj6+KPGgus76pUQiPFkvEjr37ycefZzi9bFNc7cG4uQ1vF4yObZWGXz52OKkT+toOto73\/QeaExlaA7Eqm+SpUqRbWo5T7mpUHRCSecYBTNydQ35MqzBZrQua7Mub2+obX9G4ufmEZFolSkw\/BSUW866\/TjN7PTyQZPiy9YQ\/GO122tz5CcFpWly\/Pb5dNDZZIea\/\/zMI\/AmpwNsv70eOLxvm+85+miIs9U8MmCFS22LxBxVpTSBXCmP4cXO9P7EHT7c3KnsY2qlpZbn1kXdNUJ1+fi+BR5plKrVi0J06I24bdEb8gIBNiGcVKZEkaJmxENTmMjQ2Uq2J7gMR87FfQpbCpkfhlk\/z1PLWoLgyfz88LcX5TvHdmhmsBY0O2EpUx3cXxCYyoYvQ0dOlRwfcCLx9zw4MGD8sADD0Q1z6esUvoQcFG8Tl\/voz8Jw0YkFvRDflOB6HcFm+vi+ITGVDBkY38P\/lKKFSsmeIFjGRm9B3nBQq+1KQKpRYBlZ5gLS89ILlBY0ktqexq79tCYCm4jcWXAUpptJtMhGAoGcjYtUoikw25mdXwdCaFg0l0Ur4PpWWpqQYkLc8nJXSGy0ktqnvR9rS6OT2hMBX+xuDS45ZZbBFeS+E7BJB93kNGcMQEl3uFw5IT7BK4hmJE6vgYJJRcQyMldIUJ6ycllLkgtGNcd1a4ifBEaUwFTfJxwJMfDDz8s11xzjRAnjbxohB9bGBDr+v5y7HpWx9d+RAofd3HOXvhepa+GnFzmguSCQjcVjMXF8QmVqXBOD75pcfk4YsQIwbUk3vNjDXmQjq9hZvfcc49EI0RMyF+Ga8ifRhxpidBPlINipfnz\/XHuhWKl+fP9ce6FYqX58\/1x7oVipfnz\/XHuhWKl+fP9ce6FYqX58\/1x7oVIs+PDNURaLKIc5C\/HNeRPixSnXE6jUnJoz3apdMdM866RBnEPm17Znc\/CBdf5iXKQP51ryJ8WKU65WN9UkPmhMZU9e\/YIjpSY\/mCejMKWaQ0AM5Xxd5KpEU6MsTfBt60\/z8Yx9cfFAZ7k8EPLPbhOQPcSzUwfD3HsNYpG\/BpA\/jJcQ\/404myEJPQT5aBYaf58f5x7oVhp\/nx\/nHuhWGn+fBunP9wL2TRCriHisYhykL8c15A\/LVKccpA\/n2vInxYpTjmIfPpDyDVEPBZRDvKX4xryp0WKU67hWb+Ttx+4RM6sfLysOu6SPEtc7sGW6rTTThN27HOdn7gf8qdzDdn++PPyxylnv5N0hKExFaYqmCfj7W3p0qVy7rnnCspbHF\/DCPydhznANDiqI9K+IPKpK1HH1\/7naDwCApocGAIsQS\/6cLewoTGwSh2rKDSmgue3AwcOyJ\/\/\/GdZvHixsEcHH7WkocRNFCeXHF8n2naXy6f7V85lLIJqGzqWKcu2y6IN3++ELky9Lo5PaEyFKcmgQYPMeT7Dhg0THDYBLqIbEgvxRMhVx9eJ9EHLZg8CruwdSgXioTEVOgPzOOuss4TVnP3790vjxo0FBoO9CvnRiKkT\/m3V8XU0lAqfl24lX+FbnBk1MA3ilAUklsK02MXxCY2poDdBUuGI0+bNm8ull14qHKrESlDp0qWTx1nvVAQyBIEOdY8vkrqV0JjKrl27hFUeDNhuv\/12GT16tEyaNEmQWDzPi\/ha4MiaFRscWeNxH2M5y63xyM+5Pyh927RpI2vXrhXKsypE+TAcX0fsSIZkuDhnzxDoYjZzVIdqpsxrmw+bMJk\/Lo5PaEzFDyBm+ZwsiDe4L774wjAbf74\/vmHDBmMkh6NsXE8i4Tz22GNmEyKbEbnmIDJ84D755JOyfPlymTNnjsC8WG7muA+8zfnr1LgiEBYCSCsvb\/1lWI9PyXNDYyrlypUTpjpDhgwxa\/Y4vcayFmkDXUuk3nLfX\/\/6V2H5mDJIO8S5jzim\/57nGUM69gZx\/CmSS5kyZYRVpbp16xpGw71KsRGwUmDskloiGQSQVnYe+Il88OXPkrldfjw+SVUT6E2hMRXP8wRfKjAIPvpu3boJylekDT7+SL2EOaDcnTt3rnGbwFQIh9kY0+E6AR+3Z555pnTs2FF2794tSDa4VZAj\/3C1sHHjxiNX3wcY1gVFGPEFVZcL9Wh\/quQ5Zk\/VeJT6apO8uHa38IOX6DPiHZ\/v3\/T0\/A2NqdA9XB9gnjxr1ixzVAfLwmwu5Gwg8i1hHQvYfotalLtYzmKFiHkzDqepq1OnTrJmzRpzbhAMC0bD8jAXPJkAAAoPSURBVLWtK3+I0ZzSR6IYhIfB5seul1Ujb5J33303peOQ\/91P1XVoTAXGgYn+8OHDBd2IJaxrYQT+DvstamE6gwcPNtme50nt2rUF61wcB9epU0eqVfte+YX0g5+W8uXLx3R8bSrTP4qAIhAIAqExFZwxMd1Bl3LfffeJJSQPawhXUA\/Rt7zzzjuCQpeVHXQmTG+Y8sBcPvjgA3PbunXrjA4F6QZn1zwPIh6G4+tx48aJ9f0yYcIEOe+88wSmSptMg7PyjxudRh\/XoUMHYSrxyiuvuNGoDG5FaEwFvQkMIh5DNz++bL5q3bq1tGvXTvCUP3XqVOndu7ega2Ga1LdvX2nYsKFZokZnU69evVAdX8PoUBhjqEc\/tm3bZvzHsBrVokULmT9\/PslKISLA9JvVQraJLFq0SJTRF24w0s5U9u7dK+hAON4UhnLRRRfJzTffLChYIfIoE6lbnucZ3ys4ymbpGEVthQoVTPEaNWrIjBkzzHSKY1JZw\/e8cB1fo89huobdDI1EwmKfEgyV9n366ackK4WIAO8hY8F+tJIlSwo2VCE2J+MfnXamwqDdeOON0rNnT8GiFncHMBiuIfIok\/HIRugAGybR\/9hsJBcb1zD9CGDZfejQIeMjmafj3pSVROJKySGQdqaCX1qmBDiUwXsbilrP8wSXBhCHsFMmue6k8i4xBnZMZaxuhKcxH8eqF4tdjhphFYp8lr2x7s1\/5AiuHdAFcS\/EqhahUvAIoG\/jPbM1swCAJMzSLdbV\/KDxrjEmMBfKMS1nLxpxpeQQSDtTYdUHyQR9CObzKGVR0tpBTa4bqb+LFzC\/X1zajF0NTIVVq6uvvtpM7VgaX716tUyePNnY3vhbh\/EeK1jggFIZK2J\/vsYLjwBTTN4pFOFIHrZGdCcwfabOTI9ZJMB4DJ\/IjAXGk5TnnbT3aJg4AmlnKijBmN5cdtllgg+Vm266ySjGoulREu9W8Hfw64Vk5feLy0vI9AVlME9keZv5+Y4dO7gskLAixgAPJS0GfCx9F1hQE5NGAOkD84JLLrnkqDoWLFggKPmLFy8uMHN0cdg2MRbsPWNDK2PJquRRN+pFQgiknanQOuxHCC3x68Cvvr12MSzILy77lRCdYZK0mV84xOeCmMqAAQMENw2e5wl6o4ULF8qoUaPMsjf3KgWHACuBWFnD5G2tTENRwCIpkuZ5nrGUxZUpDAbJBSbPfeQr\/QiBuBNCYSpxty4DCrK6wy9jBjQ165vIOPEjkPVApBiAUJgKu4QxVkNJibjJ0jAh1xirRXJunWIsEq6eKRHTOTt1I8R1A1OchCvTG1KKANIk0xoU6zwIZTn6FBxOc60UHAJpZyqs8DC3jbTXhDzKBNfF1NVEO7HmZZWBpxCyYxrxm2sldxDwPE+aNm0qvF+sAmEfxPT1lFNOcaeRRaQlaWcqRQQ30w38wGDNi3MpFK5Tpkwx1r2kmwL6JywECnwu50rB8FlOZrmfzacYIhZYWBOTRkCZSgLQIT5jbo\/C1d7GCgJKviVLlpglZK5tnobhIsBOdjaj2law6sPeMsYKRTmSi83TMDgElKkEh6XWpAgoArkIKFPJBUH\/KwKKQHAIKFMJDstka9L7FIEihYAylSI1nNoZRSB8BJSphD8G2gJFoEghoEylSA2ndkYRCB+BRJlK+C3WFigCioDTCChTcXp4tHGKQOYhoEwl88ZMW6wIOI2AMhWnhye4xuEUHO90ePG3hO\/c4J4QTk1Yxz7xxBPm3ChM73Gm5W8J\/R4zZow\/KS\/OhsIRI0YIe4HyEmNENDs2AspUYmNUZErgDQ3zdEu4xrSdY9cuvlrtdSaE+Eh58cUXJVkfKDi7xhUCu+Qzob+Z0kZlKpkyUilqJ7\/i3bt3l7Zt28rixYtl7dq1wgkHbLrD5y6\/5jx63rx50qBBA3NeEZso+\/fvL7ioQDogpAxSApIBDIpTDpCIqGfgwIFGkiCfOnHLyQbMzp07G69\/SArDhg0T0nCshBc2mEWvXr2MX2DqRqKAiFtiVzhOsXCyZNMihUhltAfC4RZt2LdvnzRu3Fg4mgO\/yZHu1fTEEFCmkhheGV26S5cuxtsZfmtwzI2DbjqEW0x2WONbZOTIkcYjHeff8FHjmJzzqMePHy+csYTrAD5ipATuLYi4l8Pc8KT2+uuvC75Mpk2bZoribhOJiTJICcuXLxd8x27ZssWcgQTjYUqDr5rt27cL7jrxWbNq1Spz2Jep5Mif+fPnS\/Xq1Y9cfR\/4+0g\/YRjkIJUhodHPSpUqmYPcaBd9of+4QaCcUuERUKZSeAydrSF\/wx577DGxfmxwzG13Wzdp0sS4tdy4caPw4fFrzgfJqX1r1qwx0gt+YvgAPc8Tdv+yYzt\/\/fYapsDHC5PCHy9uM1euXGmykRLwQcOOYZ7PlAtH1JwOyFlIuCZAR4JUhAPxVbnMBEfhPC+\/7xOYDemm4iN\/\/H2kr5deeumRHBGcjePWs1WrVoJ+iQwYW4kSJUSP5QCNYEiZSjA4FolamAJwfAWMhA8SD\/PTp083JwIwpbGd5Owipiz2On\/ItKJPnz55DAyppX\/udCl\/OXtNXbjltNe7d+82DKBZs2bGqRJTL05fgOnYMomGtP+pp54SjoC54YYbxPO8RKvQ8nEioEwlTqCyoRiKS1xiwkz4CDnt8aGHHhIcGTE9wVsa6UgzMCCcUSFx4EKTdHQc4IQUgPSBJIF00K9fP2EaRF5BBCNjWgVz4QiNHj16mFMCa9asafQ26HqQnvLfi9vOaNMwf3mmWPSHYztos81DUqLtnE5o0zQsHALKVAqHX5G6m+nNnXfeKShbOY8a51Ndu3YVpiEoZJlK4EMY5sK0g6kK1xxtwQoMuhcYDdMjGAUhUyvO1SEeCSyOhPU8zyhN27dvLxzfwhSJD936MuY6\/\/08A70QTCF\/nv8aZoWUsnnzZkFRDIMiRL+D3oa+IMH479F48gg4w1SS74LeGQ8CeEAr6MOGaUC2DnQZKFiRNDgMDU92nucJZ+hwYBpSCtMHPmTP88xxI+hQ5syZI+PGjZP7779fkASQNkiHmAqRxvNph30WzyWNj3rIkCHC0i7P5Vme55mVHz76du3aFThdYaWIYzeQbqiD9lKfrZ+Q59EW8tDr0H4Ihgkjoa+cBVSYqRXPUfoBAWUqP2ChMYcQYJka5oLkw8FgBTUNRtKxY0dB5wKTK6hMtDSWyzlzCmkrWjnNSwyBYokV19KKgJhD0VhFSSUWnFTw0ksvGcknmhTBahI2J56XuOIVHRJTPaSoVPYl2+pWppJtI679dROBItQqZSpFaDC1K4qACwgoU3FhFLQNikARQkCZShEaTO2KIuACAspUXBiFTGuDtlcRiIKAMpUo4GiWIqAIJI6AMpXEMdM7FAFFIAoCylSigKNZioAikDgCRZ+pJI6J3qEIKAKFQOD\/AAAA\/\/\/he90\/AAAABklEQVQDANofcCYVwyrVAAAAAElFTkSuQmCC","height":167,"width":277}}
%---
%[output:0b3c900a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARUAAACnCAYAAAA\/m8goAAAQAElEQVR4AeydC7xNZfrHn43qjEoYilL\/pJRKbiGDyiXKuFRTcptMpqSpxIQTXcZMEdJtSonJpNwacm2ki0IukRA1EpUhpIQUHZPqf75v3v1597L2OXvvs\/fa+5zz+HjOWuu9Pu9vr\/Vbz3t7Vomf9Z8ioAgoAklEoIToP0VAEVAEkoiAkkoSwdSiFAFFQERJRe+CYo2ANj75CCipJB9TLVERKNYIKKkU659fG68IJB8BJZXkY6olKgLFGgEllQR+\/jfeeEPOOOMM4ZhA9jyzUKZbdr9+\/eSSSy6RL7\/8UvLMGEfkd999J126dDHC+bp166RWrVoyevToOEpJLKm3fdFKQRd0QrdoaWx4PGltnliOYANO\/B78DrHkIY3NR17OCUuHgN3FF18sHIOsX0klAbRbtmwpn376qXBMIHueWSgzVWXnWXEAkRDj\/fffLwMGDEgJdsluwmeffSYffvihXH311TJy5MhkF5\/y8mrWrGleHLfddltSX0qSz7+MJRX7RuMtgfA2oi3cmLy5eQtwJI4j4cQjbl43zoa3bdvWWBrkzesNRJ2ksWLT2nI4Ui91EIegF28n3g68aW1e0pCWPIRRtqsr4d5r0kyfPl22bt0ql1122RFvHPJQFkfy2jrJxzXhxFux4cRZ2bVrl3DTffvttzJixAihDcRxtPk4ck04wjlhtI83oW0bcbQdDIhH3DrXrl0re\/fulcaNG5PU3OjkJR3CucUIXdCpQ4cOxiK0bSMdQt2E0UZvWgqnXtIh6INehHuFMiiLdAhtIw16WFz4DWw4cVa8ea3+Nv7zzz83baVcNw5d0IlwK+hLPo7oM3HiRGM90j7CqZ+0xHkxR1fKJx6xecgH1mBOG7gOQjKSVAClZ8+e5o3GW5s3GzcOgFtQeIM8+eSTMmvWLHOjEk+cm5eb+JRTTpFrr73W3MDEI+ecc46xNHgDvf7660c8rKShHMqkbqsDPwzhxHuFOIRw+6Ced955gg5eHUkTi\/Tq1cu8JU899VRBT948br4LLrhAiNu0aZMJXrJkiTlyI6FnfhiSuEKFCgKOxx9\/vMGbNzJ5acuYMWPCOL377rsGQzcOnUKhEMUY4WGhzm3btsk777wj5AdD+7vNmzdPypYtKyeddJKQtk+fPsLvA0akpxDSY62BOzqB3UUXXSQPPviguHiSlvZ603JNfZRD\/ZSLPuhFneSzAil069bNEDa\/MelpNw\/wiSeeKFOnTjX4cp+Ai83HkQcZ0nF14uGlXuKRb775RiZMmBC+RymbcMK4f2kbbad9ixYtMpgQD5nee++9wpHrvDBHD+5viyO40VbykBeswdzeI4SlWlJGKgVRnJuPh4Ufk3J4SLjBAP7AgQMEmRusatWqwoPGW9ze9OQlLXmOO+44gdV50\/PjmYy5fy6\/\/PLcvyJnnnmmOfr94Uew5RDPA86Nx03LtVfQlxt40qRJ5mbgBqNudEDH999\/P+kmdOnSpc1DCS7cXBy5ycEFHNDJD8P9+\/d71Y+4po20FdLiDWgfBhLZconjwatfvz7BRmx3gTDiSIMO6LV582bhN+LmR29+Rx52MPvtb39r8i9cuNAXIzAEV4QHEuvFPnAmo\/MH4qA+6qV+9EAfHmL0c5IKpMR19+7dOQjpyYee4GkCo\/zZuXOneZnl9Rvb38L7YHMvcT9QNPcpGHDuCuTAb8BvkRfm3Nfc31YPyuO+JY9b3vbt28Ok5Yan4jzjSIWbAgDcxtofhZswvweCfNxw3HiYgu6bg7hUiX1YUlW+X7k8bNxMPDDcXODDNWnzwvD7778nSVThDY6ZzRuUG9YmhAi85foRMyQE9t78thyOPOydO3fm1HTvSEse+4Y1Ec4frAfiY\/090duWiT5OUeFTXhzhi9wTyI7fkRcCpJEblJL\/tJG2cI9yr\/pVYnH1ex5snJsPXPIq8+STTxbuFzdPqs4zjlRoOAC4DeYH5ofmBz\/22GPdKMO+3hudtw3sD9NbgfEjMsZwwQ9O3SS1NwJHrmMRe9Py1uONz4MBQfImiSV\/LGl4M5GONxMYcZ0fhr\/61a\/IElXGjx9v4jDPwc9aOzx03t\/GttFkOPzHvmXJi2BhkPdwdPjAG5t46rGYMJALXuFEuSeQHF0t9PCmz432\/Q+hQLSkR7AMsBjdxN6HE9KEmOku8Du5aaOd2\/ajs\/2No6UlHJIYN26c6VZxj6IjuhLnJ36\/pa3TTU\/XjXZasd017l\/uC2udu3lSdZ5xpEJDAYA3jX3DYKbygPMWtjcnb2fMWYRzTFzefjYvPxZl8SDzY\/Ojcx2r8HByo\/OwkocjZIWJzHVeQvcD09ea0ehCe9DN5rM3BuXasESOti6wok6uKYe6qJNwrl0MvcRMvCvuwwZutMPGE0e5tMkbR93oQPeDhwcywOJhjIPfht+Ih5aHl7z8Lvw+POzox8PFi8P+xrZOHnAedHtNWu4He+0eeQi5T7gnuDfQg0FRhHM3Lb8x15ZEaRNtQ0\/0JS6a2LaCDW2xecHd5InyB\/1cYkZHdI2S3ATnhTn3I\/elvY946WGxcCQzbeM3AVuug5AIUgEcfmiUikVSpSBWBcxrTTqOvP14s9k6TzjhBGGQDROSG4544mxeBqtoA284BiLzu0nI6wo3+i233CI8lPGWw40zcOBA0+fmx0QX9EM3ezPacj\/66CO32iPOuUm50Rk34iH1JqAuHiLCOXLNOXXlhyHpEKsTOPOQYxGAKdjaQUB04MHhNyCeNqHTzz\/\/TBFGqJs6ueBmJz9pyEMYbeGtyduT34PBSIsD6SGcxx57zJjp9oGnDOqlq2TTrl69WiB8S15uWh4m6uO3I6+3XPSwwm\/MGA33CL8xbaJt9i1v0\/kdaWu039gvvRvGGA44cG8MGjRITjvtNDN1DcG46ew57UEv9ANPF3NwZEAZcrNtsPca98uKFSsEPdHXlpfqYwSpUBlvCswya0ZFO5KG9KkSHgq3boB166pSpYoZaCPNwtwBPsC18W5e1+y14RxJS5luPGGuEE\/5iJuO\/IRxpF7qx8R3fzhuWPKQDqEsyiYNaQlDXn75ZTPDQlnEcySco3tNWZRJmFcomzwc3TjKINyKjbc6oAfnCOek44GybeLato1zyqN80nCNTg0aNCAoLG5ZpCGtjeQh4q2JpUEY5ZHGCnVRN3G0lfKJIx26c44888wzQhw6U583Lfnd9G65xLni5qVsV190Ia8bllde6iQendAN4dxbjlvnnDlzBKE9hFMGetBmyrKCDoSTzou5LZ94hDLIR3kQL0eug5IIUoH9e\/fubd4C+SlAWr80vEXatWtnph+98V5LCKuIMG86vc5cBOjK8EbkSHeC8SxeRN4ui18LeMB69Ohh1sNgUfil0bAjEQDrRDE\/srTUh0SQCv3Up59+2sytf\/XVV3nW7jfYR57hw4dLTk6Ob17Wb7Ro0cK8mWFU3gKwrG\/iKIGkJ599C0RJpsHxIBBHWt6C\/HYcIQl+B4TzWIrhDUx+jrGk1zQiYA1mHMEZvBHOMxGfCFLB+sDKmDt3rjRp0kRatWol06ZNM+su8lP+hx9+MIuo2rRpIxUrVvRNvnv3bilTpoxvnAYqAopA0UAgglSwPq655hqZMWOGrFy5Uvr06SOzZ8+WCy+80KxKhWwwef2ajjmLCcxAkl88YVhC9B8ZXYe0pkyZIj\/++CNRYcHMUzkjvI1AsVAsknEPRBsEDj94STyJIBW3XKwWrI7nn3\/eLDVnSfLYsWOF1Y\/ecRD2ODClxeh0yZIl3WIizrOysmTw4MHCiDTENXPmTFm8eHFEGi4w9TJdCoue4Ki6fhrucoNHsqQw4cqwA\/oGIVFJhcqxIjZu3ChM82VnZ8uOHTvMXhQIh3grTItigdSrV08Y4WdmCIuFKS2bhmOzZs2kadOmEgqFTBeJtOvXrycqdtGUioAikNEIHEEqEAkzOHfffbchCBYNHTx4UCZPnizLli2TO+64Q+gmua1i0M2yP4QCWTD3705lHTp0SPr27WvGaJhn37Nnj6xatUpq167tFlVozufPn6+6pgABxTUFoAZcZASpMHvDAqXrr79e\/ve\/\/5ldppDEfffdJyyQCoVCcavHFm6kVKlS0r9\/f0MqkA7dKAaCGzZsGHeZmkERUAQyF4EIUmGKatSoUcaCeOihh6ROnTqS1xiJX7OY8mW6iyPxXbt2FYRzliczOLt8+XJZunSpWREbCsVPVJSloggUTwQyv9URpMLMDkva9+7dm6\/mWDX5JtIEioAiUOwQiCAVxjrY6IZDHPYk5CWkKXZoaYMVAUUgXwQiSIVZHcY9mH5ik1ZeQpp8S9cEioAiUOwQiCAVZnWY9mUQNRYpdmhpg+NEQJMXRwQiSKU4AqBtVgQUgeQioKSSXDy1NEWg2COgpFLsbwEFQBFILgJ5kgqeuFi4hjuDLVu2CNfJrT6TSlNdFAFFIBkIRCWVV155Ra666ipZsGCB4L+EfT833XSTca+YjIq1DEVAESiaCPiSCh9BYq8PbvuGDBliPgBVtWpVwWKZOXOmEF804dBWKQKKQEER8CUVNhCy9wcv5m4FXLMxkHg3XM8VAUWg0COQtAb4kkr58uWlcuXKZmey\/XgXnt3+9a9\/Sbly5QRP9knTQAtSBBSBIoWAL6mwo5idyQzOspOYncp8\/gGHSrhEOOaYY4oUCNoYRaAoI7B40145ULeH3Dp5vXCe6rb6kgqVYpE8\/vjj8sEHHwikwqcB2GHMTmPiVRQBRaBwILB1T47877TGMvndLwJR2JdUcNTELmTcRtpBWftVOcKII00gGmolisCRCGhIBiPgSyp4vcffLM6p27dvLzjDbtu2rTRq1Ei4xuM+mw0zuF2qmiKgCKQJAV9SoetTrVo1efjhh03Xh6+c4Vjp0UcflSuuuMI4q2ZsJU06a7WKgCIQBwJbdvt\/hyuOIuJK6ksqWCp8ec519RgKhYzP2g0bNgjxblxcNWpiRUARKNII+JIKflVwLcm3fOzYCQ6cGLD96aefhNmfNWvWFGlgktg4LUoRyBgETiuflXJdfEkFvypMKc+aNUvOO+8881mNunXrCqtrBw4cKEw1Dxs2LGHlWPOCM6jRo0cnXIZmVAQUgcxEwJdUULVKlSry4osvmg9\/vfTSS\/Laa68ZZ9W1atUSPr3x5ptvkiwhYREd+4kSyqyZFAFFIC4ElmzaE06fNkvFaoAj7JycXwZ56P58\/PHHwpcKmVK2aeI90m1i0Pe6666LmpXPPPKZxkwWvsqYyfq5uqmun4mLR7LOCwOuPEvvLF9unrUgCIWKvJYKYUbGjRtnPtHBN3qssGs5KyvriI+JmQwx\/Pn2229l\/PjxhpiOPfbYqDn4MBkbGDNZsOQyWT9XN9W1qvlulYtJMs4LA64LVv5HDlU42zxrp5XLMsdU\/\/ElFWZ3+MLgjBkzBLn11ltl48aN5uuENWrUkOOOOy5uvbB0JkyYYMZnqlevHnd+zaAIRVLvCgAAEABJREFUKALxI7Dkk73hTI3PLBc+T+WJL6mwEzkUCgkfBKtYsaJ88cUXQjeoQ4cOZo0KpBOvUmxMfPvtt6Vfv36CSTZixAhBuI63LE2vCCgC+SOwZXeOTF6xwyQscWCXNK5W1pyn+o8vqdgpZayVo446Sui2QCw7d+6UvXv3ip1mjkc5rBu+XEjXBhkwYIAgI0eOjKcYTasIpBSBolQ4e30WH7ZUSu3aIE3OTCOpMKXM1PHKlStl165dgoXCJzs6duwobdq0kQoVKhQl7LUtikCRQgALZfirm2X4q5+ZdjFAm\/XRbHMexB9fS4WKGchiWT5jKCzNX7dunXz44YeCS8lQKESSAkmvXr0EKVAhmlkRUAQiEMA6af\/U6ghCebJTDaH7E5EwhRe+pMJO5C5dughHWzfdIKyWTp06RYTbeD0qAopAehDAMjFkMmq18ZnCNZpgocz+U53Auj3UiUSQCiRyySWXmD0+LMlnKplBVSvsWmYquHTp0uRVSTMCWn3xRQDigEhwvFT7gWWGTOz4CWSS3bqqrLmnkXAeNEoRpMJsz8KFC4XFabg64MigqivPPvtsQlPKQTdM61MEihICkAhe2xgraZ9rkVgigVhsOyGQzvUrCdZJduvTbXDgxwhSsbUzjfz3v\/9dONowPSoCikAwCEAgCISBJWJJxI6VWIvEagOZZB+2TEZ1rpEW68TqwjGCVGz3x3Z3\/I50j0hHZhVFQBFIHAGIA4E8EJdAXEvEj0SwSCAQujhIdhotEy8CEaRiuz9ud8d7TveIdN6C8rvWeEWguCIAcdB1scThRx6EEe8lEDDDErEkQtcGEoFQCCOONJkkEaTiVYyNV0wrDxo0SCZOnKizPl6A9FoRyEUA0kAgBQSCoMuClP\/zW4LVQdeFcOIRP\/LILcp0XSALBOKAQBDOCQtqARu6JCpRSYXPnrKTGEuFwrFQWK9CONcqikBxQACyQKylASFADhBGz+lfiCUNiINwhDSQBhINIyyMJtXKCkQBYWCB7H6kmZmx4RohjnTRysjUcF9SwYP+Cy+8IE888YSRoUOHypgxY+Spp54yHxhj2X6mNkj1UgRiRQCyQLyEATFAGhCFFWtpEGdJ471tOVGrggyQMHHkDqBCHIglj9m31hFLHoXBAonaWE+EL6kcPHjQJGNVrTk5\/IdrNht+\/\/33h0P0oAhkHgIQBZIXWbgWhpcwLGlQRrTWQRgnlyklljSyc2dfIAhIg+6KFS9xFCXyiIaNL6nwWVM86uOhDdePZGYTIV0f9gWx4ZAwFUUgKAR4wBGXKIa\/utks+sJ6sJZFQcnCtgfSsIRBNwTCQFzSmNO9iljSyM6dfSEdpEFeW05xPPqSCo6t+QQHrgpwHdm0aVOpXbu26frcddddCTtpKo4Aa5ujIwBJIC5RjFm+N2aiYMMcVgXC+AVlRa\/tlxgeeCQvwvB2TyATCANR0vgFx7z++pIKGfi8qeujloFaLJWzzjqLaJWijECCbeOhRiAJhIcdwZJAsCYQximiWRTPrNgr5EFiJQrUhSiQWMiCrom1MJQwQC+54ksqLG7DdeTf\/vY3+frrr83K2vLly0soVPDdyclVX0tLJQIQBOIShLfLEY0koo1TJJMobFfEWhZKFqm8G2Iv25dUfv3rXxuvbHSDfv\/735vPnfK1Qhz94hYy9uI1ZaYgADkgEASCJYDEShJYGoi3y5EoSUSzKMZcXclMq8ZCFNoVyZS7K1IPX1IpWbKk0M3Jzs427iPnzJkjVapUkR49esiVV14pBfGmH1m9XiWCAOSAQA4I5IBEI4h6T2w2C7DodmBBIBAEkmqSiGZNROt+1DslyywASwQXzZMZCPiSilWNGR8cXk+dOlXYnYwryQYNGmTWQK1VtpAeIQcEckAgByQaQXjHIpJFEMDHmATiWhHZh6dKGXuAIBC6GV5LIhpJqDUBssVLfEll3759wixP7dwZH5w14Zxp1KhRsmzZMmFWCH+zxQum\/FsLMViJlRyCIgje\/sxcIF6SgCAQL0lAIkj24alS8kIQCMSTPyKaorgi4EsqWCinn366TJ8+XXDWxCdQ6Q7RLSpOQPmRBF0GhCXafoOUtotBGqQg3QuLNQ8xggWB8IAj2TFaEYxTQBBItockKBexdelRESgoAr6kwsI3\/MfGSiQM3vJNH7pGv\/nNb4Q9Q2xG9CrHrBKuE6xLBc4J86YL6hrSsFaF7W74EYXbxaBrgrBEO55BStrEw2slVnLwWhBuN8NLEGpFgHJxk8xrry+pxKvmpk2b5OWXX5bZs2fLkiVL5OqrrzZ7hbzl0I1q0aKFsEkRYe1LkG4UIBEIAQsCiwKxhGEtiliJgiXa8RIE3QwrSg7eu0OviwoCSSEVPtnBJz1OOukkgwsfG7PnJuDwH8LLlClz+Cr6AUsGSycZsuT9jdJ97Lvh3aQQCsQCwfhpAFkg7WocJ8jNDcrK4JYVhC4EwtLs924\/XZ5uITKjayUjj7cpK\/0uyjLSsfrPclGF742cUnKPID9+s0OS0ZZEy2ApQKJ5g86nun6W1HuFZwnxu9dTFeZLKoypQAAc3Yq59gunu1SrVi2ZP3++VKtWTegKtWrVys1qztndzPR0\/fr1BSfaU6ZM8f0wGVYMmxcTlZInVJZ3dv1K7pi7V9qN\/1zmrP\/O1O\/+wcpgXIJxBoRZDboaHwxuKsj4m+oL8mCnOtK7TU25pkkNI41rnWW+y8sUe6L6BZ1PdS2+31LmWULcez\/V5xGkAmmwBmXDhg1y8803C0fGPKzw7R+++8MqWz\/FWrZsKZ988omwvmX48OHy3XeRDzMfdx88eLCsWLHCfKN55syZZh2MX1mJhjFG0v6p1Wb\/CF0ZW44lEUsea+5pFN52DrkwHmHT6jFVCGi5xQGBCFLBCrn99tulZ8+esnbtWnO85pprxErv3r3N6losExecxYsXy5AhQ0xQKBSSevXqCe4TDhw4YMLsn2bNmgmbE0OhkFn6zydA1q9fb6MLdKQ7w2ArhMI5hblEYklEyQNkVBSB1CEQQSp4z6dLMnfuXLnzzjuF46JFi8SVfv36CR8Wc1XCHQJfL2Qglpmg9957z6zAZb+QTYcflr59+8q0adOENHv27JFVq1ZJ7dq1bZKEj5DIbZPXh7\/KRkFYH1gldG2USEBERREIBoEIUqH7g7UCSbChMCcnx\/iltd0fjnSPSOeqV7duXfON5fbt2xtLBj8sf\/7zn6VUqVLGty3+bTnv37+\/IRUsFL7NzLhLw4YN3aLiPodQsE5sVwfrBCJBOI+7QM2gCCgCBUIgglQgFMY8mCKmG2S7Pe6RcNK5tYZCIenWrZssXbrULJZjoBbXCaTp2rWrIJwThiXER8pIS55QKPGdzxAKFgpHyodEsE6wUrgumGhuRUARSASBCFKh+8NHxGrUqCE8\/G63x54TTrpEKkt2HqaGXQuFcROIJdn1aHmKgCIQOwIRpGKz0b2hm0N3xyuEE2\/TpusIobBgjfohEiwUzlUUAUUgvQj4kgrdG7o5breHlbCNGjUyM0LEp1NtujssYkMHCOXJTun\/1CO6qCgCRQiBhJviSyp0b+jm2C4PR6aYx40bJwyyeqeUE649wYzWQiF75\/qVRWd3QEJFEcgMBHxJxU+1UChk1p\/Q\/UmnpbJld47xYYqOWCnZrU\/nVEURUAQyBAFfUmHMBPLwjqfMmzfPTDEz5Zwu\/ZntsXXT7bHnelQEFIHMQMCXVLBEvGMqjK+wcI3wdH33ByvFne3Rbk9m3EQZoIWqkEEI+JKK35gK4yqMs9SvXz9t6rtjKWqlpO1n0IoVgTwR8CUVcrDlvXPnzoLPE67Hjx9vXEzSLeI6aMFKYRqZehlLUSsFJFQUgcxDwJdU+Fbygw8+KHhxY3MgarME\/+ijj5ahQ4cK+3gIC1KWfLI3XB0zPuELPVEEFIGMQsCXVPB7gtsC3EJaJ9dMI+NicseOHcKYS9CtmLxih6kSK6VxtbLmvJD+UbUVgSKNgC+plC5d2uxEZrex2\/rNmzcL1krQsz90fcIDtOWydF2K+6PouSKQYQj4kgrWCbuM\/\/rXvwo+UDhv06aN4E\/lD3\/4gwQ9+xPR9WlQOcMgVHUUAUXARcCXVEiAe8iFCxcKYyss0ccH7VtvvSXNmzcnOlBxuz6d61cKtG6tTBFQBOJDwJdUGDdh5mfXrl1mWT6+T\/DYFrSFQlO8XR\/CVBQBRSBzEfAllUqVKslll10mU6dOFQZt06k+pGLrb3xmOXuqR0VAEchQBHxJhbUorEvBtwrdIFz8Wwn6A2DueIrO+mToXaRqKQIOAr6kwge+GE\/Btb9XCCfeKSOlp0s27THlM5WsC94MFPonQAS0qvgRiCAVNhKyBuWHH34QrBXvhkKuCSdd\/FXFn4OujzuVHH8JmkMRUASCRiCCVCCURHzUxqr09u3bpVOnToKza1brjh07ViCwaPkhFRun4ykWCT0qApmNQASpsJGQcZRU+aiFRFq3bm2cY8+aNct8+mPjxo1REdLxlKjQaIQikLEIRJCKq2UqNhTy4fYOHTpIKBQyHxvj+z9+09QMClP\/Gx\/8sjSfbxvzTWLCMkUy8Zu\/0bBRXT+TaNgUJLww4MqzhLjPdqrPfUklVRsKa9asaZb5d+nSRS6++GI599xzhelrbyMZHC55QmV5b1uOiTqj4nFStWrVjJIqVapklD554aO6pubeKQy48iwh5kEK6I8vqbA2JVUbCtkCMGnSJGFfEX5vmU3ya6uOp\/ihomGKQOYj4EsqqdhQuH\/\/fhk0aJDYbyez6xkn2ps2bfJFScdTfGHRQEUg4xHwJRWsCTYR\/jWJGwqPOeYYoVvFN5cZS9mzZ4+sWbNGonmS27L7+zB4rFEJX+iJIqAIZDQCvqSCxqykpWuSrA2FpUqVkv79+8v06dOlcePG0rZtW2nZsqXUrVuX6o6Qrbt\/GU+BUJAjEmiAIqAIZCQCUUkFbbEu6KIka0PhySefLIynLF26VJYsWSLdunUzM0HU5RVd9OZFRK8VgcKBQJ6kkq4m6CBtupDXegNDoAhXlPGkol2fInz3adOKJAIZSSruzM+p5bKKJPDaKEWgqCIQlVS2bdsmI0eONNPATAVbGT58uOzbty+leCw5vDOZSnRnMiioKAKFBwFfUmHxG1PKW7dulUaNGpnZGmZskAYNGhin2EE0Ubs+QaAcQx2aRBGIAwFfUmE9SSgUkuzsbGnXrp0w+2MFR9ip9qavMz9x\/IKaVBHIMAR8SaV8+fJmXwuf5Aha359KVwhXqe4OwlDoiSJQaBAo4afpgQMHzMa\/W2+9VXr27BkxrpLqMZWfSv86rJJ2f8JQ6IkiUGgQ8CWVo446Si699FJ54IEHBFcFjKVYSfWYyqEK54TBS2jmJ5xbTxQBRSAdCJTwq5QxE8ZO7DiKeySceL98yQ5TSyXZiGp5ikDqEfAlFarF9QFdHdw+4kGffUCPPvqoEE58quRQhS1tY2UAAA+pSURBVLNN0RAKYi70jyKgCBQaBHxJ5dChQzJs2DDBs9Xjjz8ulStXNgO3TDUPGTJEiC80LVRFFYFihUD6G+tLKrgl+O9\/\/ysDBw6U\/\/u\/\/5MSJUpIVlaW3HjjjYZoiE+V6mFLRVfSpgpiLVcRSCkCvqRSsmRJY41gmbi1Hzx40OwqJt4NT9a5biRMFpJajiKQPgR8SYV1Knz29E9\/+pNxVbBr1y7B+\/1tt90mTZo0kXLlyqVEY5dUUlKBFqoIKAIpR8CXVKj1hhtuMHt\/cJqbk5Nj\/J\/ceeed8sc\/\/tFYK6RJtmzd84tjJsrVT5yCQlEXbV9RRCAqqYRCIaldu7YMHjxYpk2bJnwPqHnz5pKqrg\/gqqUCCiqKQOFGICqpfPDBB8bdI1PKfK+HRW9XXXWV+X5KqpqsfmlThayWqwgEh4AvqbAW5aGHHjJdHRxVL168WNatWyctWrQwq2zZcOhVEReRxEM+bdq0EUjJm4ZvMbPmhY8bIZwTZtOpX1qLhB4VgcKLgC+psPeHbxzjmNp2d1i637FjR+MR3zsrxBTzE088YcZgVqxYIbfccovpLnnJhwFfiIdxGoQFdSeeeGIhQU\/VVAQUgVgQ8CUVZn\/OOeccWb58ufA5DVsQHwDDeTXxNozjV199JaeffrqcddZZXMr5558vX3\/9tXjJZ\/fu3VKmTBmTxu+PujzwQ0XDFIHChYAvqWB5rF69WpjtwUkTnyhlKrlPnz5CV4gBW8JsU6tXry58yoPvBWHh8BmOihUrygknnGCTmCMkM2fOHPOtH8qbMmWK\/PjjjybOHaRd8da\/zdhNQb5zm+q8rDZOdR3JKl911W8pm4csoD++pAI59O7dW9jrc++99xpnTayufeSRR8Re48DJq+PHH38sdJE2bNgg999\/v\/CJDzcNq3KZTaKLNGPGDJk5c6YhKTcN531v6ma2BeT1feB0x3Xv3j3jdbQYqa6p+ZZyYcCVYQaE56qAEnN2X1JhF3KzZs2Mx7emTZsay4LxFXe3Mue2FrpIzz\/\/vOB\/BTeUTz\/9tGCp2Hh7pEzKC4VCJp5vCq1fv95Gh49PPHivMJCbyYKymayfq5vqekZK7qfChiv6BiG+pELF27dvl+uuu86sVeHhP\/fcc4UVtoyfEO8K5jXdmjFjxgikYQd33TRsQuzbt69Z8wIJ0cVatWqVKZ907Eje\/UgzWXNPI\/l8wQsCu6p8qjh8qhgk6zngOQtCfEmFPT7sRoYg6MrQqPfff1+qVasmQ4cONfuCXOU2bdpkppCvv\/56YawF6dSpk0BAEydOFMR+9pSFdJAUlk6rVq2kYcOGblECuUQE6IUioAgUKgR8SeWbb74xszcsemMqmRYxztK5c2fZsWOHMItDmBW6NXRj3n77bVm0aJERBmHpAnXt2lUQ0jJzRDizSqxryeuzp6RXUQQSRECzpREBX1JhgPXoo4+WnTt3RqjGNDFdF6yOiAi9UAQUAUXgMAK+pMJUMFbJzTffLHfffbcwXoIXODYTtm7dWrzrVA6XpQdFQBFQBMSXVMDliiuukBdffFEqVKggy5YtM4vgnn32WbnhhhuIVlEEFAFFwBeBqKRCatY5MGPD4Oxdd90lNWvWTJnbA+orpqLNVgSKFAJ5kkpQLWUVLt2r+vXrGydQb7zxhrGMgqo\/Wj2MH7H+hk2SrAAeP358eAWwm2fjxo3CDm5mskj31ltvudGBnMeqq1WGZQDozEZRGxbUMVZdWYHNuidwZUYxk3FlAy0zmnZmk+ug8IylHrbY3HfffbEkLXCajCCVefPmGd+3zAgxO\/SPf\/zDLNMvcOsKWADraF5\/\/XV59dVX5ZVXXpE333xT3n333YhSmX7H10yPHj3MXik2Vj711FNmOj0iYYovYtHVqsBGT0icKX8bFuQxVl3pbleqVEm4L3DADqlDNJmmK\/fA6NGjzfjjO++8I1j3Dz\/8sOzfvz9IVX3rYhMvPQ0WprJR2DdRkgMzglQWLlwouEtg+vqUU04Rpp5ZH5PktsZdHATCWxL3mccff7xZWbxy5cqIcrhxmA0jHRFsrKQdX3zxBZeBSSy6ogxWwoQJE8yao\/POO4+gwCUWXVnWgBWFNcViygsuuEDYJlK6dOlA9Y1FVxRicSezo5xD2izB4L7gOp0CdhdeeKGxpIPSI15SSbpe+G5hdS0DwhQeCoXMkmq8+XOdTmFRX5UqVcIqsPhv8+bN4WtOmAljjxQuHHhgX3vtNeMews1HulRLLLqiA4sYIWy7doiwoCUWXXn7Q9jPPPOM1KtXT9q1ayfsLeMhCVLfWHRlCQYzo\/fcc4+5dzmyEJTwIHX1q4sXIotMwdAvPhVhaScVGsWNwroYzjNNYr0xtm3bZvY+TZ482bxR+TGDbkt+utJ1+Oc\/\/2n0ZH9X0Pq59eWnK2Y7u7QZS8E6ZIU3m1S3bt3qFhPIeX668lIcOXKksOeN1ed03+n+pKt7GQgoeVSSdlI59thjBVORvUboyduem+nss8\/mMq1CV+aTTz4J68B5jRo1wtf2hLEWtiWw6XLq1Klm97KNC+oYi65YWQsWLDAe\/OhOMF7UoUMHYWA8KD2pJxZdsVyxDBn4DIVCxlcP1t\/evXspIjCJRVcGvXkx2nuDI93loLvAgYGST0VpJ5VQKCTNmjUTxlWYBeKNz0reM888Mx\/VUx+NLxn8yvCGRzjHGbhbM+HPPfec8Xrnbmtw0wRxHouuLAmg+8PbdO3atcJnWPj0CmQYhI62jlh0pVvJ2BpbOsi3ZcsWM\/DJwC3XQYlXV797AJ24dyFt9GI2kG49JMh1cZO0kwqAX3755eZbQkzHdunSxfjGPfXUU4lKq9StW1fwo8tCQFYSQ36EoRSbJBHM8Y8++sg4tMJUR5haZC8U6YIS9MpP16B0ya+eWHRlkJPpZLqTOF\/Hnw\/fnWI\/WX7lJzM+Fl3RCYdmjKVgWXG84447zD2dTF0KS1kZQSrMluD0ibcSGxJ5eDMBwFAoJAy44e2OaU13AyQDnQguIXA6RRp0R\/79738LJnCQbQiF8tfV1YcuJ4OgWC9ueBDnoVBsumKpTJo0yUwpY1HxwAahn1tHKBSbrujG786UMkeu3XLSfY41yrhPEHpkBKkE0VCtQxFQBIJBQEklGJzTUotWqgikAwEllXSgrnUqAkUYASWVIvzjatMUgXQgoKSSDtS1TkWgCCOQMaRShDHWpikCxQoBJZU0\/dzsakXyq55NdXjgYzFVfmljjWd6nBW1\/fr1izWLWXUbT\/qYC04gISuA+QQJ63K+\/PLLBErIO4stnzVTycQ971qLTqySStH5LWNuCatp2WQW1LoFFGP7hf0aJdcFFdaBsB6EjZwFLcubnzUdrIthqb03Tq\/zR0BJJX+MEk7Bg4R\/Xx4A9rGwP4g9TrwJR4wYIQjWCulwR4D1QDrcQODkh7cwq0jZo9OzZ0\/hrckmNXbE8t1qyo3muIhl42PHjhXKJC2rU9lSYOvm07R+lgfl33777WavDeXz4FoAcEfAlyvRkVXD7NEijrpoC9\/fpi4WCdJO4qgD8uJzL+hKWnzOkJYV1H\/5y1+ENKQFB75cibMuyrE6ExdNwARL7rHHHhP0pVx0oR5vHtruWh\/ea2\/6QK+LUGVKKin8MdlQBlm88MILZts+DxEPOm\/CAQMGCNKrVy9hez1vRoTt\/ZDKc889J7yFn3zySbNHhw+1sWkNR+R16tSR\/\/znPzJq1ChDTOTxNoMHZv78+ca5FJYJO21x0dCiRQtTL3XzsLv58AnywAMPmK9HkoeHH4dTbEUg3Zo1a+R3v\/ud4DqhcePGglc8wlkJTTcNX8bkYy\/M7NmziTLCLmPIlXbjkAsfJaSFQNiHZBLl\/mFVMo6Z8I1MOVZnyCY3Oup\/yBLPZjjSot1076gnagaNSCkCSiophJeHlC38LOPHcQ9WB29mb5XshIU0OJKubNmy3iTmGssAS4IHm60N7EthOwAEYxIc\/kO9WBhYRji9ws0B53hcQ5\/DyY44YBmxcY8vKZAH8ho8eLDZRU5iHFFBJuzLwcUmDzPhWBYQHO4esBD4GgPhVqibOMgBa4WNl1yzZwbLwaabO3eutG3b1vgkoX7ysTlv3759NonvsUSJEtK9e3ez14ZNfNdee63ZoOqbWANTjoCSSgohZlMkPjYWLFggbIrDAmFviLfKnJwcwVcIDyoPNPuHvGm4ZkyCNzsPNgOVdENmzpwpWETEW6E8PvjG\/h4bxkPMm58H24Z5j3gug5AsKUAeEAZ5SZuVlSWEce4K+eieQTp8bQFLxo23euB0CV3tNWkgR44IhETXhbYheH2D5HDYRHw0wX3GSSedFI52z8OBehIYAkoqKYSarznyNscKYcv8jTfeKGzi4+Fyq6WrwAOF2U43gYfJjbfndH+wTOhOQC4IXRIeZJuGIw8qbhcZb+AawZ0Eb3SIhWs\/gTQIh5Q4YjXh1Al\/IVyHxXNClw7ygTBfeuklM7bhSWIurV601QTk\/nF1JB5ypV0I5ARpYtHkJo36H30Z77EJ6E76DbLSPuqw6fSYGgSUVFKDqymVB2bYsGFmzMQE5P7hobZve6yC3CDhoeBtzDXjFzh4\/umnn8RaFZyT7rTTThPyM16A1UK3h53S3jEV0rDTGyfidJfQg3MsCWuFUJ5X6DpUrlxZ6DqhC+MTEB7ledO611Z\/dGKAmbES8rtpOKccXEOw8xhvaZAVYyjEIcRBqvjUgdBwKI7lQrnERxNIinEYukl0EdH50ksvPSI5PlrQlbLJg0PzIxJpQIERUFIpMITRC8DRFJ7VGANhNgOLhXEVHq7zzz\/fWC3M\/pCGm7xWrVrCjAee+Xnrjxs3Thj0hBjoXlATbhV5aKpXry6kw28HZRHnSseOHYUxF2ZdcHwMQTFr5KbxnjOOMTh3DAWHWXjeY5wElxT5WQrotmTJEuPuAUuM2SMGeBk\/8dbBeArkRVevU6dOxl+NTcM0N11E\/Ncwa0XXhwFlS8I2nd+RLlDz5s2lffv2xskz3U3SgS\/jNhArvwGYUAf+cejWWYuGgWZIjSP5VBJHQEklcezyzRkKhYTpVWYyMOexMCwBMBPEDczsD64T6WZgceCOkoeLrhAkwANNF4C3O2MRPJBYHbi2hHiYUQmFQkfogpnPA4kDKQRP9PYBok7kiEy5AfgwseWjL9O0ucFCPe5skXtdtWpVY90wqMqM0ZVXXmlmp7CWyENaykCwupjBIi2zPRAvuhJH9w7\/NeBFPJ\/l4MEnLj+hDrqFYHrTTTeJLZN2WuwIgySZfcIF6KBBgwyxgyt+ZRjL4phfXRqfNwJFn1Tybr\/GBowADzjWEF0QLDDWy1irImBVtLoUIfD\/AAAA\/\/9OY7ySAAAABklEQVQDANu3N2EtlEyZAAAAAElFTkSuQmCC","height":167,"width":277}}
%---
%[output:029435b9]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARUAAACnCAYAAAA\/m8goAAAQAElEQVR4AeydC7xNZfrHn43qjEoYilL\/pJRKbiGDyiXKuFRTcptMpqSpxIQTXcZMEdJtSonJpNwacm2ki0IukRA1EpUhpIQUHZPqf75v3v1597L2OXvvs\/fa+5zz+HjOWuu9Pu9vr\/Vbz3t7Vomf9Z8ioAgoAklEoIToP0VAEVAEkoiAkkoSwdSiFAFFQERJRe+CYo2ANj75CCipJB9TLVERKNYIKKkU659fG68IJB8BJZXkY6olKgLFGgEllQR+\/jfeeEPOOOMM4ZhA9jyzUKZbdr9+\/eSSSy6RL7\/8UvLMGEfkd999J126dDHC+bp166RWrVoyevToOEpJLKm3fdFKQRd0QrdoaWx4PGltnliOYANO\/B78DrHkIY3NR17OCUuHgN3FF18sHIOsX0klAbRbtmwpn376qXBMIHueWSgzVWXnWXEAkRDj\/fffLwMGDEgJdsluwmeffSYffvihXH311TJy5MhkF5\/y8mrWrGleHLfddltSX0qSz7+MJRX7RuMtgfA2oi3cmLy5eQtwJI4j4cQjbl43zoa3bdvWWBrkzesNRJ2ksWLT2nI4Ui91EIegF28n3g68aW1e0pCWPIRRtqsr4d5r0kyfPl22bt0ql1122RFvHPJQFkfy2jrJxzXhxFux4cRZ2bVrl3DTffvttzJixAihDcRxtPk4ck04wjlhtI83oW0bcbQdDIhH3DrXrl0re\/fulcaNG5PU3OjkJR3CucUIXdCpQ4cOxiK0bSMdQt2E0UZvWgqnXtIh6INehHuFMiiLdAhtIw16WFz4DWw4cVa8ea3+Nv7zzz83baVcNw5d0IlwK+hLPo7oM3HiRGM90j7CqZ+0xHkxR1fKJx6xecgH1mBOG7gOQjKSVAClZ8+e5o3GW5s3GzcOgFtQeIM8+eSTMmvWLHOjEk+cm5eb+JRTTpFrr73W3MDEI+ecc46xNHgDvf7660c8rKShHMqkbqsDPwzhxHuFOIRw+6Ced955gg5eHUkTi\/Tq1cu8JU899VRBT948br4LLrhAiNu0aZMJXrJkiTlyI6FnfhiSuEKFCgKOxx9\/vMGbNzJ5acuYMWPCOL377rsGQzcOnUKhEMUY4WGhzm3btsk777wj5AdD+7vNmzdPypYtKyeddJKQtk+fPsLvA0akpxDSY62BOzqB3UUXXSQPPviguHiSlvZ603JNfZRD\/ZSLPuhFneSzAil069bNEDa\/MelpNw\/wiSeeKFOnTjX4cp+Ai83HkQcZ0nF14uGlXuKRb775RiZMmBC+RymbcMK4f2kbbad9ixYtMpgQD5nee++9wpHrvDBHD+5viyO40VbykBeswdzeI4SlWlJGKgVRnJuPh4Ufk3J4SLjBAP7AgQMEmRusatWqwoPGW9ze9OQlLXmOO+44gdV50\/PjmYy5fy6\/\/PLcvyJnnnmmOfr94Uew5RDPA86Nx03LtVfQlxt40qRJ5mbgBqNudEDH999\/P+kmdOnSpc1DCS7cXBy5ycEFHNDJD8P9+\/d71Y+4po20FdLiDWgfBhLZconjwatfvz7BRmx3gTDiSIMO6LV582bhN+LmR29+Rx52MPvtb39r8i9cuNAXIzAEV4QHEuvFPnAmo\/MH4qA+6qV+9EAfHmL0c5IKpMR19+7dOQjpyYee4GkCo\/zZuXOneZnl9Rvb38L7YHMvcT9QNPcpGHDuCuTAb8BvkRfm3Nfc31YPyuO+JY9b3vbt28Ok5Yan4jzjSIWbAgDcxtofhZswvweCfNxw3HiYgu6bg7hUiX1YUlW+X7k8bNxMPDDcXODDNWnzwvD7778nSVThDY6ZzRuUG9YmhAi85foRMyQE9t78thyOPOydO3fm1HTvSEse+4Y1Ec4frAfiY\/090duWiT5OUeFTXhzhi9wTyI7fkRcCpJEblJL\/tJG2cI9yr\/pVYnH1ex5snJsPXPIq8+STTxbuFzdPqs4zjlRoOAC4DeYH5ofmBz\/22GPdKMO+3hudtw3sD9NbgfEjMsZwwQ9O3SS1NwJHrmMRe9Py1uONz4MBQfImiSV\/LGl4M5GONxMYcZ0fhr\/61a\/IElXGjx9v4jDPwc9aOzx03t\/GttFkOPzHvmXJi2BhkPdwdPjAG5t46rGYMJALXuFEuSeQHF0t9PCmz432\/Q+hQLSkR7AMsBjdxN6HE9KEmOku8Du5aaOd2\/ajs\/2No6UlHJIYN26c6VZxj6IjuhLnJ36\/pa3TTU\/XjXZasd017l\/uC2udu3lSdZ5xpEJDAYA3jX3DYKbygPMWtjcnb2fMWYRzTFzefjYvPxZl8SDzY\/Ojcx2r8HByo\/OwkocjZIWJzHVeQvcD09ea0ehCe9DN5rM3BuXasESOti6wok6uKYe6qJNwrl0MvcRMvCvuwwZutMPGE0e5tMkbR93oQPeDhwcywOJhjIPfht+Ih5aHl7z8Lvw+POzox8PFi8P+xrZOHnAedHtNWu4He+0eeQi5T7gnuDfQg0FRhHM3Lb8x15ZEaRNtQ0\/0JS6a2LaCDW2xecHd5InyB\/1cYkZHdI2S3ATnhTn3I\/elvY946WGxcCQzbeM3AVuug5AIUgEcfmiUikVSpSBWBcxrTTqOvP14s9k6TzjhBGGQDROSG4544mxeBqtoA284BiLzu0nI6wo3+i233CI8lPGWw40zcOBA0+fmx0QX9EM3ezPacj\/66CO32iPOuUm50Rk34iH1JqAuHiLCOXLNOXXlhyHpEKsTOPOQYxGAKdjaQUB04MHhNyCeNqHTzz\/\/TBFGqJs6ueBmJz9pyEMYbeGtyduT34PBSIsD6SGcxx57zJjp9oGnDOqlq2TTrl69WiB8S15uWh4m6uO3I6+3XPSwwm\/MGA33CL8xbaJt9i1v0\/kdaWu039gvvRvGGA44cG8MGjRITjvtNDN1DcG46ew57UEv9ANPF3NwZEAZcrNtsPca98uKFSsEPdHXlpfqYwSpUBlvCswya0ZFO5KG9KkSHgq3boB166pSpYoZaCPNwtwBPsC18W5e1+y14RxJS5luPGGuEE\/5iJuO\/IRxpF7qx8R3fzhuWPKQDqEsyiYNaQlDXn75ZTPDQlnEcySco3tNWZRJmFcomzwc3TjKINyKjbc6oAfnCOek44GybeLato1zyqN80nCNTg0aNCAoLG5ZpCGtjeQh4q2JpUEY5ZHGCnVRN3G0lfKJIx26c44888wzQhw6U583Lfnd9G65xLni5qVsV190Ia8bllde6iQendAN4dxbjlvnnDlzBKE9hFMGetBmyrKCDoSTzou5LZ94hDLIR3kQL0eug5IIUoH9e\/fubd4C+SlAWr80vEXatWtnph+98V5LCKuIMG86vc5cBOjK8EbkSHeC8SxeRN4ui18LeMB69Ohh1sNgUfil0bAjEQDrRDE\/srTUh0SQCv3Up59+2sytf\/XVV3nW7jfYR57hw4dLTk6Ob17Wb7Ro0cK8mWFU3gKwrG\/iKIGkJ599C0RJpsHxIBBHWt6C\/HYcIQl+B4TzWIrhDUx+jrGk1zQiYA1mHMEZvBHOMxGfCFLB+sDKmDt3rjRp0kRatWol06ZNM+su8lP+hx9+MIuo2rRpIxUrVvRNvnv3bilTpoxvnAYqAopA0UAgglSwPq655hqZMWOGrFy5Uvr06SOzZ8+WCy+80KxKhWwwef2ajjmLCcxAkl88YVhC9B8ZXYe0pkyZIj\/++CNRYcHMUzkjvI1AsVAsknEPRBsEDj94STyJIBW3XKwWrI7nn3\/eLDVnSfLYsWOF1Y\/ecRD2ODClxeh0yZIl3WIizrOysmTw4MHCiDTENXPmTFm8eHFEGi4w9TJdCoue4Ki6fhrucoNHsqQw4cqwA\/oGIVFJhcqxIjZu3ChM82VnZ8uOHTvMXhQIh3grTItigdSrV08Y4WdmCIuFKS2bhmOzZs2kadOmEgqFTBeJtOvXrycqdtGUioAikNEIHEEqEAkzOHfffbchCBYNHTx4UCZPnizLli2TO+64Q+gmua1i0M2yP4QCWTD3705lHTp0SPr27WvGaJhn37Nnj6xatUpq167tFlVozufPn6+6pgABxTUFoAZcZASpMHvDAqXrr79e\/ve\/\/5ldppDEfffdJyyQCoVCcavHFm6kVKlS0r9\/f0MqkA7dKAaCGzZsGHeZmkERUAQyF4EIUmGKatSoUcaCeOihh6ROnTqS1xiJX7OY8mW6iyPxXbt2FYRzliczOLt8+XJZunSpWREbCsVPVJSloggUTwQyv9URpMLMDkva9+7dm6\/mWDX5JtIEioAiUOwQiCAVxjrY6IZDHPYk5CWkKXZoaYMVAUUgXwQiSIVZHcY9mH5ik1ZeQpp8S9cEioAiUOwQiCAVZnWY9mUQNRYpdmhpg+NEQJMXRwQiSKU4AqBtVgQUgeQioKSSXDy1NEWg2COgpFLsbwEFQBFILgJ5kgqeuFi4hjuDLVu2CNfJrT6TSlNdFAFFIBkIRCWVV155Ra666ipZsGCB4L+EfT833XSTca+YjIq1DEVAESiaCPiSCh9BYq8PbvuGDBliPgBVtWpVwWKZOXOmEF804dBWKQKKQEER8CUVNhCy9wcv5m4FXLMxkHg3XM8VAUWg0COQtAb4kkr58uWlcuXKZmey\/XgXnt3+9a9\/Sbly5QRP9knTQAtSBBSBIoWAL6mwo5idyQzOspOYncp8\/gGHSrhEOOaYY4oUCNoYRaAoI7B40145ULeH3Dp5vXCe6rb6kgqVYpE8\/vjj8sEHHwikwqcB2GHMTmPiVRQBRaBwILB1T47877TGMvndLwJR2JdUcNTELmTcRtpBWftVOcKII00gGmolisCRCGhIBiPgSyp4vcffLM6p27dvLzjDbtu2rTRq1Ei4xuM+mw0zuF2qmiKgCKQJAV9SoetTrVo1efjhh03Xh6+c4Vjp0UcflSuuuMI4q2ZsJU06a7WKgCIQBwJbdvt\/hyuOIuJK6ksqWCp8ec519RgKhYzP2g0bNgjxblxcNWpiRUARKNII+JIKflVwLcm3fOzYCQ6cGLD96aefhNmfNWvWFGlgktg4LUoRyBgETiuflXJdfEkFvypMKc+aNUvOO+8881mNunXrCqtrBw4cKEw1Dxs2LGHlWPOCM6jRo0cnXIZmVAQUgcxEwJdUULVKlSry4osvmg9\/vfTSS\/Laa68ZZ9W1atUSPr3x5ptvkiwhYREd+4kSyqyZFAFFIC4ElmzaE06fNkvFaoAj7JycXwZ56P58\/PHHwpcKmVK2aeI90m1i0Pe6666LmpXPPPKZxkwWvsqYyfq5uqmun4mLR7LOCwOuPEvvLF9unrUgCIWKvJYKYUbGjRtnPtHBN3qssGs5KyvriI+JmQwx\/Pn2229l\/PjxhpiOPfbYqDn4MBkbGDNZsOQyWT9XN9W1qvlulYtJMs4LA64LVv5HDlU42zxrp5XLMsdU\/\/ElFWZ3+MLgjBkzBLn11ltl48aN5uuENWrUkOOOOy5uvbB0JkyYYMZnqlevHnd+zaAIRVLvCgAAEABJREFUKALxI7Dkk73hTI3PLBc+T+WJL6mwEzkUCgkfBKtYsaJ88cUXQjeoQ4cOZo0KpBOvUmxMfPvtt6Vfv36CSTZixAhBuI63LE2vCCgC+SOwZXeOTF6xwyQscWCXNK5W1pyn+o8vqdgpZayVo446Sui2QCw7d+6UvXv3ip1mjkc5rBu+XEjXBhkwYIAgI0eOjKcYTasIpBSBolQ4e30WH7ZUSu3aIE3OTCOpMKXM1PHKlStl165dgoXCJzs6duwobdq0kQoVKhQl7LUtikCRQgALZfirm2X4q5+ZdjFAm\/XRbHMexB9fS4WKGchiWT5jKCzNX7dunXz44YeCS8lQKESSAkmvXr0EKVAhmlkRUAQiEMA6af\/U6ghCebJTDaH7E5EwhRe+pMJO5C5dughHWzfdIKyWTp06RYTbeD0qAopAehDAMjFkMmq18ZnCNZpgocz+U53Auj3UiUSQCiRyySWXmD0+LMlnKplBVSvsWmYquHTp0uRVSTMCWn3xRQDigEhwvFT7gWWGTOz4CWSS3bqqrLmnkXAeNEoRpMJsz8KFC4XFabg64MigqivPPvtsQlPKQTdM61MEihICkAhe2xgraZ9rkVgigVhsOyGQzvUrCdZJduvTbXDgxwhSsbUzjfz3v\/9dONowPSoCikAwCEAgCISBJWJJxI6VWIvEagOZZB+2TEZ1rpEW68TqwjGCVGz3x3Z3\/I50j0hHZhVFQBFIHAGIA4E8EJdAXEvEj0SwSCAQujhIdhotEy8CEaRiuz9ud8d7TveIdN6C8rvWeEWguCIAcdB1scThRx6EEe8lEDDDErEkQtcGEoFQCCOONJkkEaTiVYyNV0wrDxo0SCZOnKizPl6A9FoRyEUA0kAgBQSCoMuClP\/zW4LVQdeFcOIRP\/LILcp0XSALBOKAQBDOCQtqARu6JCpRSYXPnrKTGEuFwrFQWK9CONcqikBxQACyQKylASFADhBGz+lfiCUNiINwhDSQBhINIyyMJtXKCkQBYWCB7H6kmZmx4RohjnTRysjUcF9SwYP+Cy+8IE888YSRoUOHypgxY+Spp54yHxhj2X6mNkj1UgRiRQCyQLyEATFAGhCFFWtpEGdJ471tOVGrggyQMHHkDqBCHIglj9m31hFLHoXBAonaWE+EL6kcPHjQJGNVrTk5\/IdrNht+\/\/33h0P0oAhkHgIQBZIXWbgWhpcwLGlQRrTWQRgnlyklljSyc2dfIAhIg+6KFS9xFCXyiIaNL6nwWVM86uOhDdePZGYTIV0f9gWx4ZAwFUUgKAR4wBGXKIa\/utks+sJ6sJZFQcnCtgfSsIRBNwTCQFzSmNO9iljSyM6dfSEdpEFeW05xPPqSCo6t+QQHrgpwHdm0aVOpXbu26frcddddCTtpKo4Aa5ujIwBJIC5RjFm+N2aiYMMcVgXC+AVlRa\/tlxgeeCQvwvB2TyATCANR0vgFx7z++pIKGfi8qeujloFaLJWzzjqLaJWijECCbeOhRiAJhIcdwZJAsCYQximiWRTPrNgr5EFiJQrUhSiQWMiCrom1MJQwQC+54ksqLG7DdeTf\/vY3+frrr83K2vLly0soVPDdyclVX0tLJQIQBOIShLfLEY0koo1TJJMobFfEWhZKFqm8G2Iv25dUfv3rXxuvbHSDfv\/735vPnfK1Qhz94hYy9uI1ZaYgADkgEASCJYDEShJYGoi3y5EoSUSzKMZcXclMq8ZCFNoVyZS7K1IPX1IpWbKk0M3Jzs427iPnzJkjVapUkR49esiVV14pBfGmH1m9XiWCAOSAQA4I5IBEI4h6T2w2C7DodmBBIBAEkmqSiGZNROt+1DslyywASwQXzZMZCPiSilWNGR8cXk+dOlXYnYwryQYNGmTWQK1VtpAeIQcEckAgByQaQXjHIpJFEMDHmATiWhHZh6dKGXuAIBC6GV5LIhpJqDUBssVLfEll3759wixP7dwZH5w14Zxp1KhRsmzZMmFWCH+zxQum\/FsLMViJlRyCIgje\/sxcIF6SgCAQL0lAIkj24alS8kIQCMSTPyKaorgi4EsqWCinn366TJ8+XXDWxCdQ6Q7RLSpOQPmRBF0GhCXafoOUtotBGqQg3QuLNQ8xggWB8IAj2TFaEYxTQBBItockKBexdelRESgoAr6kwsI3\/MfGSiQM3vJNH7pGv\/nNb4Q9Q2xG9CrHrBKuE6xLBc4J86YL6hrSsFaF7W74EYXbxaBrgrBEO55BStrEw2slVnLwWhBuN8NLEGpFgHJxk8xrry+pxKvmpk2b5OWXX5bZs2fLkiVL5OqrrzZ7hbzl0I1q0aKFsEkRYe1LkG4UIBEIAQsCiwKxhGEtiliJgiXa8RIE3QwrSg7eu0OviwoCSSEVPtnBJz1OOukkgwsfG7PnJuDwH8LLlClz+Cr6AUsGSycZsuT9jdJ97Lvh3aQQCsQCwfhpAFkg7WocJ8jNDcrK4JYVhC4EwtLs924\/XZ5uITKjayUjj7cpK\/0uyjLSsfrPclGF742cUnKPID9+s0OS0ZZEy2ApQKJ5g86nun6W1HuFZwnxu9dTFeZLKoypQAAc3Yq59gunu1SrVi2ZP3++VKtWTegKtWrVys1qztndzPR0\/fr1BSfaU6ZM8f0wGVYMmxcTlZInVJZ3dv1K7pi7V9qN\/1zmrP\/O1O\/+wcpgXIJxBoRZDboaHwxuKsj4m+oL8mCnOtK7TU25pkkNI41rnWW+y8sUe6L6BZ1PdS2+31LmWULcez\/V5xGkAmmwBmXDhg1y8803C0fGPKzw7R+++8MqWz\/FWrZsKZ988omwvmX48OHy3XeRDzMfdx88eLCsWLHCfKN55syZZh2MX1mJhjFG0v6p1Wb\/CF0ZW44lEUsea+5pFN52DrkwHmHT6jFVCGi5xQGBCFLBCrn99tulZ8+esnbtWnO85pprxErv3r3N6losExecxYsXy5AhQ0xQKBSSevXqCe4TDhw4YMLsn2bNmgmbE0OhkFn6zydA1q9fb6MLdKQ7w2ArhMI5hblEYklEyQNkVBSB1CEQQSp4z6dLMnfuXLnzzjuF46JFi8SVfv36CR8Wc1XCHQJfL2Qglpmg9957z6zAZb+QTYcflr59+8q0adOENHv27JFVq1ZJ7dq1bZKEj5DIbZPXh7\/KRkFYH1gldG2USEBERREIBoEIUqH7g7UCSbChMCcnx\/iltd0fjnSPSOeqV7duXfON5fbt2xtLBj8sf\/7zn6VUqVLGty3+bTnv37+\/IRUsFL7NzLhLw4YN3aLiPodQsE5sVwfrBCJBOI+7QM2gCCgCBUIgglQgFMY8mCKmG2S7Pe6RcNK5tYZCIenWrZssXbrULJZjoBbXCaTp2rWrIJwThiXER8pIS55QKPGdzxAKFgpHyodEsE6wUrgumGhuRUARSASBCFKh+8NHxGrUqCE8\/G63x54TTrpEKkt2HqaGXQuFcROIJdn1aHmKgCIQOwIRpGKz0b2hm0N3xyuEE2\/TpusIobBgjfohEiwUzlUUAUUgvQj4kgrdG7o5breHlbCNGjUyM0LEp1NtujssYkMHCOXJTun\/1CO6qCgCRQiBhJviSyp0b+jm2C4PR6aYx40bJwyyeqeUE649wYzWQiF75\/qVRWd3QEJFEcgMBHxJxU+1UChk1p\/Q\/UmnpbJld47xYYqOWCnZrU\/nVEURUAQyBAFfUmHMBPLwjqfMmzfPTDEz5Zwu\/ZntsXXT7bHnelQEFIHMQMCXVLBEvGMqjK+wcI3wdH33ByvFne3Rbk9m3EQZoIWqkEEI+JKK35gK4yqMs9SvXz9t6rtjKWqlpO1n0IoVgTwR8CUVcrDlvXPnzoLPE67Hjx9vXEzSLeI6aMFKYRqZehlLUSsFJFQUgcxDwJdU+Fbygw8+KHhxY3MgarME\/+ijj5ahQ4cK+3gIC1KWfLI3XB0zPuELPVEEFIGMQsCXVPB7gtsC3EJaJ9dMI+NicseOHcKYS9CtmLxih6kSK6VxtbLmvJD+UbUVgSKNgC+plC5d2uxEZrex2\/rNmzcL1krQsz90fcIDtOWydF2K+6PouSKQYQj4kgrWCbuM\/\/rXvwo+UDhv06aN4E\/lD3\/4gwQ9+xPR9WlQOcMgVHUUAUXARcCXVEiAe8iFCxcKYyss0ccH7VtvvSXNmzcnOlBxuz6d61cKtG6tTBFQBOJDwJdUGDdh5mfXrl1mWT6+T\/DYFrSFQlO8XR\/CVBQBRSBzEfAllUqVKslll10mU6dOFQZt06k+pGLrb3xmOXuqR0VAEchQBHxJhbUorEvBtwrdIFz8Wwn6A2DueIrO+mToXaRqKQIOAr6kwge+GE\/Btb9XCCfeKSOlp0s27THlM5WsC94MFPonQAS0qvgRiCAVNhKyBuWHH34QrBXvhkKuCSdd\/FXFn4OujzuVHH8JmkMRUASCRiCCVCCURHzUxqr09u3bpVOnToKza1brjh07ViCwaPkhFRun4ykWCT0qApmNQASpsJGQcZRU+aiFRFq3bm2cY8+aNct8+mPjxo1REdLxlKjQaIQikLEIRJCKq2UqNhTy4fYOHTpIKBQyHxvj+z9+09QMClP\/Gx\/8sjSfbxvzTWLCMkUy8Zu\/0bBRXT+TaNgUJLww4MqzhLjPdqrPfUklVRsKa9asaZb5d+nSRS6++GI599xzhelrbyMZHC55QmV5b1uOiTqj4nFStWrVjJIqVapklD554aO6pubeKQy48iwh5kEK6I8vqbA2JVUbCtkCMGnSJGFfEX5vmU3ya6uOp\/ihomGKQOYj4EsqqdhQuH\/\/fhk0aJDYbyez6xkn2ps2bfJFScdTfGHRQEUg4xHwJRWsCTYR\/jWJGwqPOeYYoVvFN5cZS9mzZ4+sWbNGonmS27L7+zB4rFEJX+iJIqAIZDQCvqSCxqykpWuSrA2FpUqVkv79+8v06dOlcePG0rZtW2nZsqXUrVuX6o6Qrbt\/GU+BUJAjEmiAIqAIZCQCUUkFbbEu6KIka0PhySefLIynLF26VJYsWSLdunUzM0HU5RVd9OZFRK8VgcKBQJ6kkq4m6CBtupDXegNDoAhXlPGkol2fInz3adOKJAIZSSruzM+p5bKKJPDaKEWgqCIQlVS2bdsmI0eONNPATAVbGT58uOzbty+leCw5vDOZSnRnMiioKAKFBwFfUmHxG1PKW7dulUaNGpnZGmZskAYNGhin2EE0Ubs+QaAcQx2aRBGIAwFfUmE9SSgUkuzsbGnXrp0w+2MFR9ip9qavMz9x\/IKaVBHIMAR8SaV8+fJmXwuf5Aha359KVwhXqe4OwlDoiSJQaBAo4afpgQMHzMa\/W2+9VXr27BkxrpLqMZWfSv86rJJ2f8JQ6IkiUGgQ8CWVo446Si699FJ54IEHBFcFjKVYSfWYyqEK54TBS2jmJ5xbTxQBRSAdCJTwq5QxE8ZO7DiKeySceL98yQ5TSyXZiGp5ikDqEfAlFarF9QFdHdw+4kGffUCPPvqoEE58quRQhS1tY2UAAA+pSURBVLNN0RAKYi70jyKgCBQaBHxJ5dChQzJs2DDBs9Xjjz8ulStXNgO3TDUPGTJEiC80LVRFFYFihUD6G+tLKrgl+O9\/\/ysDBw6U\/\/u\/\/5MSJUpIVlaW3HjjjYZoiE+V6mFLRVfSpgpiLVcRSCkCvqRSsmRJY41gmbi1Hzx40OwqJt4NT9a5biRMFpJajiKQPgR8SYV1Knz29E9\/+pNxVbBr1y7B+\/1tt90mTZo0kXLlyqVEY5dUUlKBFqoIKAIpR8CXVKj1hhtuMHt\/cJqbk5Nj\/J\/ceeed8sc\/\/tFYK6RJtmzd84tjJsrVT5yCQlEXbV9RRCAqqYRCIaldu7YMHjxYpk2bJnwPqHnz5pKqrg\/gqqUCCiqKQOFGICqpfPDBB8bdI1PKfK+HRW9XXXWV+X5KqpqsfmlThayWqwgEh4AvqbAW5aGHHjJdHRxVL168WNatWyctWrQwq2zZcOhVEReRxEM+bdq0EUjJm4ZvMbPmhY8bIZwTZtOpX1qLhB4VgcKLgC+psPeHbxzjmNp2d1i637FjR+MR3zsrxBTzE088YcZgVqxYIbfccovpLnnJhwFfiIdxGoQFdSeeeGIhQU\/VVAQUgVgQ8CUVZn\/OOeccWb58ufA5DVsQHwDDeTXxNozjV199JaeffrqcddZZXMr5558vX3\/9tXjJZ\/fu3VKmTBmTxu+PujzwQ0XDFIHChYAvqWB5rF69WpjtwUkTnyhlKrlPnz5CV4gBW8JsU6tXry58yoPvBWHh8BmOihUrygknnGCTmCMkM2fOHPOtH8qbMmWK\/PjjjybOHaRd8da\/zdhNQb5zm+q8rDZOdR3JKl911W8pm4csoD++pAI59O7dW9jrc++99xpnTayufeSRR8Re48DJq+PHH38sdJE2bNgg999\/v\/CJDzcNq3KZTaKLNGPGDJk5c6YhKTcN531v6ma2BeT1feB0x3Xv3j3jdbQYqa6p+ZZyYcCVYQaE56qAEnN2X1JhF3KzZs2Mx7emTZsay4LxFXe3Mue2FrpIzz\/\/vOB\/BTeUTz\/9tGCp2Hh7pEzKC4VCJp5vCq1fv95Gh49PPHivMJCbyYKymayfq5vqekZK7qfChiv6BiG+pELF27dvl+uuu86sVeHhP\/fcc4UVtoyfEO8K5jXdmjFjxgikYQd33TRsQuzbt69Z8wIJ0cVatWqVKZ907Eje\/UgzWXNPI\/l8wQsCu6p8qjh8qhgk6zngOQtCfEmFPT7sRoYg6MrQqPfff1+qVasmQ4cONfuCXOU2bdpkppCvv\/56YawF6dSpk0BAEydOFMR+9pSFdJAUlk6rVq2kYcOGblECuUQE6IUioAgUKgR8SeWbb74xszcsemMqmRYxztK5c2fZsWOHMItDmBW6NXRj3n77bVm0aJERBmHpAnXt2lUQ0jJzRDizSqxryeuzp6RXUQQSRECzpREBX1JhgPXoo4+WnTt3RqjGNDFdF6yOiAi9UAQUAUXgMAK+pMJUMFbJzTffLHfffbcwXoIXODYTtm7dWrzrVA6XpQdFQBFQBMSXVMDliiuukBdffFEqVKggy5YtM4vgnn32WbnhhhuIVlEEFAFFwBeBqKRCatY5MGPD4Oxdd90lNWvWTJnbA+orpqLNVgSKFAJ5kkpQLWUVLt2r+vXrGydQb7zxhrGMgqo\/Wj2MH7H+hk2SrAAeP358eAWwm2fjxo3CDm5mskj31ltvudGBnMeqq1WGZQDozEZRGxbUMVZdWYHNuidwZUYxk3FlAy0zmnZmk+ug8IylHrbY3HfffbEkLXCajCCVefPmGd+3zAgxO\/SPf\/zDLNMvcOsKWADraF5\/\/XV59dVX5ZVXXpE333xT3n333YhSmX7H10yPHj3MXik2Vj711FNmOj0iYYovYtHVqsBGT0icKX8bFuQxVl3pbleqVEm4L3DADqlDNJmmK\/fA6NGjzfjjO++8I1j3Dz\/8sOzfvz9IVX3rYhMvPQ0WprJR2DdRkgMzglQWLlwouEtg+vqUU04Rpp5ZH5PktsZdHATCWxL3mccff7xZWbxy5cqIcrhxmA0jHRFsrKQdX3zxBZeBSSy6ogxWwoQJE8yao\/POO4+gwCUWXVnWgBWFNcViygsuuEDYJlK6dOlA9Y1FVxRicSezo5xD2izB4L7gOp0CdhdeeKGxpIPSI15SSbpe+G5hdS0DwhQeCoXMkmq8+XOdTmFRX5UqVcIqsPhv8+bN4WtOmAljjxQuHHhgX3vtNeMews1HulRLLLqiA4sYIWy7doiwoCUWXXn7Q9jPPPOM1KtXT9q1ayfsLeMhCVLfWHRlCQYzo\/fcc4+5dzmyEJTwIHX1q4sXIotMwdAvPhVhaScVGsWNwroYzjNNYr0xtm3bZvY+TZ482bxR+TGDbkt+utJ1+Oc\/\/2n0ZH9X0Pq59eWnK2Y7u7QZS8E6ZIU3m1S3bt3qFhPIeX668lIcOXKksOeN1ed03+n+pKt7GQgoeVSSdlI59thjBVORvUboyduem+nss8\/mMq1CV+aTTz4J68B5jRo1wtf2hLEWtiWw6XLq1Klm97KNC+oYi65YWQsWLDAe\/OhOMF7UoUMHYWA8KD2pJxZdsVyxDBn4DIVCxlcP1t\/evXspIjCJRVcGvXkx2nuDI93loLvAgYGST0VpJ5VQKCTNmjUTxlWYBeKNz0reM888Mx\/VUx+NLxn8yvCGRzjHGbhbM+HPPfec8Xrnbmtw0wRxHouuLAmg+8PbdO3atcJnWPj0CmQYhI62jlh0pVvJ2BpbOsi3ZcsWM\/DJwC3XQYlXV797AJ24dyFt9GI2kG49JMh1cZO0kwqAX3755eZbQkzHdunSxfjGPfXUU4lKq9StW1fwo8tCQFYSQ36EoRSbJBHM8Y8++sg4tMJUR5haZC8U6YIS9MpP16B0ya+eWHRlkJPpZLqTOF\/Hnw\/fnWI\/WX7lJzM+Fl3RCYdmjKVgWXG84447zD2dTF0KS1kZQSrMluD0ibcSGxJ5eDMBwFAoJAy44e2OaU13AyQDnQguIXA6RRp0R\/79738LJnCQbQiF8tfV1YcuJ4OgWC9ueBDnoVBsumKpTJo0yUwpY1HxwAahn1tHKBSbrujG786UMkeu3XLSfY41yrhPEHpkBKkE0VCtQxFQBIJBQEklGJzTUotWqgikAwEllXSgrnUqAkUYASWVIvzjatMUgXQgoKSSDtS1TkWgCCOQMaRShDHWpikCxQoBJZU0\/dzsakXyq55NdXjgYzFVfmljjWd6nBW1\/fr1izWLWXUbT\/qYC04gISuA+QQJ63K+\/PLLBErIO4stnzVTycQ971qLTqySStH5LWNuCatp2WQW1LoFFGP7hf0aJdcFFdaBsB6EjZwFLcubnzUdrIthqb03Tq\/zR0BJJX+MEk7Bg4R\/Xx4A9rGwP4g9TrwJR4wYIQjWCulwR4D1QDrcQODkh7cwq0jZo9OzZ0\/hrckmNXbE8t1qyo3muIhl42PHjhXKJC2rU9lSYOvm07R+lgfl33777WavDeXz4FoAcEfAlyvRkVXD7NEijrpoC9\/fpi4WCdJO4qgD8uJzL+hKWnzOkJYV1H\/5y1+ENKQFB75cibMuyrE6ExdNwARL7rHHHhP0pVx0oR5vHtruWh\/ea2\/6QK+LUGVKKin8MdlQBlm88MILZts+DxEPOm\/CAQMGCNKrVy9hez1vRoTt\/ZDKc889J7yFn3zySbNHhw+1sWkNR+R16tSR\/\/znPzJq1ChDTOTxNoMHZv78+ca5FJYJO21x0dCiRQtTL3XzsLv58AnywAMPmK9HkoeHH4dTbEUg3Zo1a+R3v\/ud4DqhcePGglc8wlkJTTcNX8bkYy\/M7NmziTLCLmPIlXbjkAsfJaSFQNiHZBLl\/mFVMo6Z8I1MOVZnyCY3Oup\/yBLPZjjSot1076gnagaNSCkCSiophJeHlC38LOPHcQ9WB29mb5XshIU0OJKubNmy3iTmGssAS4IHm60N7EthOwAEYxIc\/kO9WBhYRji9ws0B53hcQ5\/DyY44YBmxcY8vKZAH8ho8eLDZRU5iHFFBJuzLwcUmDzPhWBYQHO4esBD4GgPhVqibOMgBa4WNl1yzZwbLwaabO3eutG3b1vgkoX7ysTlv3759NonvsUSJEtK9e3ez14ZNfNdee63ZoOqbWANTjoCSSgohZlMkPjYWLFggbIrDAmFviLfKnJwcwVcIDyoPNPuHvGm4ZkyCNzsPNgOVdENmzpwpWETEW6E8PvjG\/h4bxkPMm58H24Z5j3gug5AsKUAeEAZ5SZuVlSWEce4K+eieQTp8bQFLxo23euB0CV3tNWkgR44IhETXhbYheH2D5HDYRHw0wX3GSSedFI52z8OBehIYAkoqKYSarznyNscKYcv8jTfeKGzi4+Fyq6WrwAOF2U43gYfJjbfndH+wTOhOQC4IXRIeZJuGIw8qbhcZb+AawZ0Eb3SIhWs\/gTQIh5Q4YjXh1Al\/IVyHxXNClw7ygTBfeuklM7bhSWIurV601QTk\/nF1JB5ypV0I5ARpYtHkJo36H30Z77EJ6E76DbLSPuqw6fSYGgSUVFKDqymVB2bYsGFmzMQE5P7hobZve6yC3CDhoeBtzDXjFzh4\/umnn8RaFZyT7rTTThPyM16A1UK3h53S3jEV0rDTGyfidJfQg3MsCWuFUJ5X6DpUrlxZ6DqhC+MTEB7ledO611Z\/dGKAmbES8rtpOKccXEOw8xhvaZAVYyjEIcRBqvjUgdBwKI7lQrnERxNIinEYukl0EdH50ksvPSI5PlrQlbLJg0PzIxJpQIERUFIpMITRC8DRFJ7VGANhNgOLhXEVHq7zzz\/fWC3M\/pCGm7xWrVrCjAee+Xnrjxs3Thj0hBjoXlATbhV5aKpXry6kw28HZRHnSseOHYUxF2ZdcHwMQTFr5KbxnjOOMTh3DAWHWXjeY5wElxT5WQrotmTJEuPuAUuM2SMGeBk\/8dbBeArkRVevU6dOxl+NTcM0N11E\/Ncwa0XXhwFlS8I2nd+RLlDz5s2lffv2xskz3U3SgS\/jNhArvwGYUAf+cejWWYuGgWZIjSP5VBJHQEklcezyzRkKhYTpVWYyMOexMCwBMBPEDczsD64T6WZgceCOkoeLrhAkwANNF4C3O2MRPJBYHbi2hHiYUQmFQkfogpnPA4kDKQRP9PYBok7kiEy5AfgwseWjL9O0ucFCPe5skXtdtWpVY90wqMqM0ZVXXmlmp7CWyENaykCwupjBIi2zPRAvuhJH9w7\/NeBFPJ\/l4MEnLj+hDrqFYHrTTTeJLZN2WuwIgySZfcIF6KBBgwyxgyt+ZRjL4phfXRqfNwJFn1Tybr\/GBowADzjWEF0QLDDWy1irImBVtLoUIfD\/AAAA\/\/9OY7ySAAAABklEQVQDANu3N2EtlEyZAAAAAElFTkSuQmCC","height":167,"width":277}}
%---
%[output:10e27999]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Cannot find library called 'litium_ion_battery'.  "}}
%---
