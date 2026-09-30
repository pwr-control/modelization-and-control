%[text] # afe\_inv\_psm: LEGACY flat-workspace init (superseded by init_model.m Rev. 01, 2026-09-29)
% Kept for reference only. It no longer runs: grid_fault_generator is now a function, lithium_ion_battery_setup
% takes the charge capacity as third argument, and the reactive-current section reads param.ctrl.ixi_ref_mod1,
% which is never defined here.
clear;
[model, options] = init_environment('afe_inv_psm');

CTRPIFF_CLIP_RELEASE = 0.001;
s = tf('s');
%[text] ### Global timing
% simulation length
simlength = 1.75;

fpwm = 4e3;

fpwm_afe = fpwm;
trgo_afe = 1; % double update

fpwm_inv = fpwm;
trgo_inv = 1; % double update

fpwm_single_phase_inv = fpwm;
trgo_single_phase_inv = 1; % double update

fpwm_dab = 3 * fpwm;
trgo_dab = 1; % double update

fpwm_psfbc = fpwm;
trgo_psfbc = 1; % double update

fpwm_isop_rail = fpwm;
trgo_isop_rail = 1; % double update

fpwm_cllc = 3 * fpwm;
trgo_cllc = 0; % double update

t_measure = simlength;
tc = 0.25e-6;
tc_factor = 1/fpwm/tc/2;
tc_decimation = 1;
delay_pwm = 0;

dead_time_afe = 3e-6;
dead_time_inv = 3e-6;
dead_time_single_phase_inv = 3e-6;
dead_time_dab = 2e-6;
dead_time_psfbc = 3e-6;
dead_time_isop_rail = 3e-6;
dead_time_cllc = 2e-6;

glb_time = timing_setup(fpwm_afe, trgo_afe, fpwm_inv, trgo_inv, fpwm_single_phase_inv, trgo_single_phase_inv, fpwm_dab, trgo_dab, ...
    fpwm_psfbc, trgo_psfbc, fpwm_isop_rail, trgo_isop_rail, fpwm_cllc, trgo_cllc, t_measure, tc_factor, tc_decimation, delay_pwm, dead_time_afe, ...
    dead_time_inv, dead_time_single_phase_inv, dead_time_dab, dead_time_psfbc, dead_time_isop_rail, dead_time_cllc);
%[text] ## Settings for simulink model initialization and data analysis
%[text] ### Settings Enable devices with thermal model
use_mosfet_thermal_model = 0;
use_thermal_model = 1;

nonlinear_iterations = 3;  % for simscape solver
if (use_mosfet_thermal_model || use_thermal_model)
    nonlinear_iterations = 5; % for simscape solver
end

%[text] ### Enable specific settings for the simulation
load_step_time = 1.25;
transmission_delay = 125e-6*2;
sst_num_of_modules = 2;
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
use_dq_pll_fht_simulink_pll = 0; % 
use_dq_pll_mod1 = 0; % 
use_dq_pll_ccaller_mod1 = 0; % 
use_dq_pll_ccaller_mod2 = 0; % 

% dqPLL
use_dq_pll_mode1 = use_dq_pll_mod1;
use_dq_pll_mode2 = use_dq_pll_ccaller_mod1;
use_dq_pll_mode3 = use_dq_pll_fht_simulink_pll;
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
   u_cm_comp_mod1 = 0;
   u_cm_comp_mod2 = 0;
   u_cm_comp_mod3 = 0;
   u_cm_comp_mod4 = 0;

if en_parallel_mode
   u_cm_comp_mod2 = -1;
   u_cm_comp_mod3 = -1;
   u_cm_comp_mod4 = -1;
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
frequency_set = 50; % default
if system_identification_enable
    frequency_set = 300;
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
%[text] ## Global Hardware Settings
%[text] ### Rail Catenary Voltage Model
catenary_nominal_power = 2*5.4e6; % per 12km
catenary_nominal_voltage = 3000;
catenary_maximum_voltage = 4000;
catenary_nominal_current = catenary_nominal_power/catenary_nominal_voltage;

isop_rail_pwr_nom = 250e3; % nominal per system
number_of_systems = 4;
full_bridge_application_voltage = catenary_maximum_voltage/number_of_systems;
hwdata.isop_rail = isop_rail_aux_application(full_bridge_application_voltage, isop_rail_pwr_nom, glb_time.fPWM_ISOP_RAIL);
%[text] ### Hardware Settings
single_phase_inverter_pwr_nom = 225e3;
afe_pwr_nom = 250e3;
inv_pwr_nom = 250e3;
dab_pwr_nom = 250e3;
three_phase_dab_pwr_nom = 3*dab_pwr_nom;
cllc_pwr_nom = 250e3;
psfbc_pwr_nom = 275e3;
fres_dab = glb_time.fPWM_DAB/5;
fres_cllc = glb_time.fPWM_CLLC*1.2;

hwdata.single_phase_inverter = single_phase_inverter_hwdata(application_voltage, single_phase_inverter_pwr_nom, glb_time.fPWM_INV);
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:63b262a3]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:713c3bf1]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:83b4ff2a]
hwdata.psfbc = single_phase_psfbc_hwdata(application_voltage, psfbc_pwr_nom, glb_time.fPWM_PSFBC); %[output:37ea9137]
hwdata.three_phase_dab = three_phase_dab_hwdata(application_voltage, three_phase_dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:3c4494f2]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:3a0fb825]

% modifications
% hwdata.afe.CFi = 2 * hwdata.psfbc.Cdc_dc1;
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

time_gain_afe_module_1 = 1.0002;
time_gain_afe_module_2 = 1.0015;
time_gain_afe_module_3 = 0.9988;
time_gain_afe_module_4 = 1.0020;

time_gain_inv_module_1 = 1.0005;
time_gain_inv_module_2 = 1.001;
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
enable_frt_2 = 1; % faults generated from xi_eta_pos and xi_eta_neg
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
param.ctrl.enable_i_react_pos_steps = 1;
    param.ctrl.time_i_react_pos_ref_1 = 0; % default
    param.ctrl.time_i_react_pos_ref_2 = 0; % default
    param.ctrl.i_react_pos_ref_1 = 0; % default
    param.ctrl.i_react_pos_ref_2 = 0; % default
    param.ctrl.i_react_pos_ref_3 = 0; % default

if param.ctrl.enable_i_react_pos_steps
    param.ctrl.time_i_react_pos_ref_1 = start_time_LVRT + error_length + 0.335;
    param.ctrl.time_i_react_pos_ref_2 = param.ctrl.time_i_react_pos_ref_1 + 0.5;
    param.ctrl.i_react_pos_ref_2 = -param.ctrl.ixi_ref_mod1*tan(acos(0.95));  % cos(phi) = 0.9
    i_react_pos_ref_3 = ixi_ref_mod1*tan(acos(0.95)); % cos(phi) = 0.9
end
%[text] #### UPQC Series Transformer
name = 'UPQC Series Transformer';
pwr_nom = 3*125e3;
u1_nom = 400;
u2_nom = 690;
f_nom = 50;
eta = 98;
ucc = 4;
p_iron = 5e3;
n12 = u1_nom/u2_nom;
n2 = 8;
n1 = n12*n2;
core_area = 0.04;
core_length = 0.25;
mur = 35e3;

Lm1 = (n1^2 * mu0 * mur * core_area) / core_length;
i1m = u1_nom/sqrt(3)/Lm1/f_nom/2/pi;

delta_star = 0;

upqc_st = three_phase_transformer_setup(name, delta_star, pwr_nom, u1_nom, u2_nom, f_nom, eta, ucc, ...
    i1m, p_iron, n1, n2, core_area, core_length, mur);
upqc_st.Lm1;
upqc_st.Ld1;
upqc_ctrl.kp_p = 1;
upqc_ctrl.ki_p = 35;
upqc_ctrl.kp_n = 1;
upqc_ctrl.ki_n = 35;
upqc_ctrl.lim = 4;
%[text] #### DClink Lstray model (partial loop inductance)
parasitic_dclink_data; %[output:4e5b197c]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:03f6d43a]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:0cdca7b1]
number_of_systems = 1;
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
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:9ed44b05]
psm_ctrl.kp_i = 0.25;
psm_ctrl.ki_i = 35;
%[text] #### Induction Motor Control
im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:3a4d8327] %[output:7c22730c]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:3a4d8327] %[output:7c22730c]
%[text] #### AFE control (with sequences)
afe_ctrl = ctrl_afe_setup(glb_time.ts_afe, grid_emu.omega_grid_nom);

afe_ctrl.kp_udc_ctrl = 2;

kp_udc = 0.5;
ki_udc = 18.0;
kp_idc = 0.5;
ki_idc = 18.0;

%% gain for weak grids
% afe_ctrl.res_pi.kp_rpi = 0.5;
% afe_ctrl.res_pi.ki_rpi = 18;

%% gains for LVRT
afe_ctrl.res_pi.kp_rpi = 0.6;
afe_ctrl.res_pi.ki_rpi = 35;

%[text] #### DCDC Control
dab_ctrl = ctrl_dab_setup(kp_udc, ki_udc, kp_idc, ki_idc);
psfbc_ctrl = ctrl_dab_setup(kp_udc, ki_udc, kp_idc, ki_idc);
cllc_ctrl = ctrl_cllc_setup(kp_udc, ki_udc, kp_idc, ki_idc);

% special settings
psfbc_ctrl.kp_idc = 1;
dab_ctrl.ki_udc = 35;

isop_rail_ctrl = ctrl_isop_rail_setup(kp_udc, ki_udc, kp_idc, ki_idc);
dab_ctrl.kp_idc = 0.02;
dab_ctrl.ki_idc = 1.0;
%[text] ### 
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
%[text] #### SOGI (second order generalized integrator)
sogi_delta = 1;
kepsilon = 2;
sogi = sogi_filter(omega_set, sogi_delta, kepsilon, glb_time.ts_afe); %[output:31c361a7]
%[text] #### Current control parameters DQ PI
dqvector_pi.kp_inv = 0.5;
dqvector_pi.ki_inv = 45;
dqvector_pi.pi_ctrl = dqvector_pi.kp_inv + dqvector_pi.ki_inv/s;
dqvector_pi.pid_ctrl = c2d(dqvector_pi.pi_ctrl, glb_time.ts_inv);
dqvector_pi.plant = 1/(s*grid_emu.trafo.Ld1 + 1);
dqvector_pi.plantd = c2d(dqvector_pi.plant, glb_time.ts_inv);

G = sogi.fltd.alpha * dqvector_pi.pid_ctrl * dqvector_pi.plantd;
figure; margin(G, options);  %[output:58b60e8b]
grid on %[output:58b60e8b]
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
torque_overload_factor = 1.15;
%[text] ### Simulation parameters: speed reference, load torque for driver application
% rpm_sim = 3000;
rpm_sim = 17.8;
% rpm_sim = 18;
% rpm_sim = 15.2;
omega_m_sim = psm.omega_m_bez;
omega_sim = omega_m_sim*psm.number_poles/2;
tau_load_sim = psm.tau_bez/5; %N*m
b_square = 0;
%[text] ### Settings Global Filters
filters = setup_global_filters(glb_time.ts_afe, glb_time.ts_inv, glb_time.ts_dab, glb_time.tc);
%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] ### Diode rectifier
% ABB 5SDF 0131Z0401 (slow recovery)
diode.rectifier.Vf = 0.977;
diode.rectifier.Rdon = 22e-6;
diode.rectifier.Cj = 0;             % F
diode.rectifier.Irm = -75;          % A
diode.rectifier.didt = -30;         % A/us
diode.rectifier.trr = 5;            % us
diode.rectifier.Qrr = 325e-6;       % C
diode.rectifier.Ifm = 2000;         % A
diode.rectifier.Vr = -50;           % V
diode.rectifier.Err = 15e-3;        % J
diode.rectifier.Rth_JC = 0.004;     % W/K
diode.rectifier.Rth_CH = 0.003;     % W/K

diode.rectifier.Rsnubber = 1e4;     % Ohm
diode.rectifier.Csnubber = 1e-12;     % F
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
% used_device = 'mitsubishi_CM1200DW_34T';
used_device = 'infineon_FF1200R17IP5' %[output:7390a984]

igbt.inv = device_igbt_setup(used_device, glb_time.fPWM_INV, hwdata.inv.udc_nom);
igbt.afe = device_igbt_setup(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
igbt.dab = device_igbt_setup(used_device, glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
igbt.psfbc = device_igbt_setup(used_device, glb_time.fPWM_PSFBC, hwdata.psfbc.udc1_nom);
igbt.cllc = device_igbt_setup(used_device, glb_time.fPWM_CLLC, hwdata.cllc.udc1_nom);
igbt.isop_rail = device_igbt_setup(used_device, glb_time.fPWM_ISOP_RAIL, hwdata.isop_rail.udc1_nom);

%[text] ### DEVICES settings (MOSFET)
used_device = 'danfoss_SKM1700MB20R4S2I4';

mosfet.inv = device_mosfet_setup(used_device, glb_time.fPWM_INV, hwdata.inv.udc_nom);
mosfet.afe = device_mosfet_setup(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
mosfet.dab = device_mosfet_setup(used_device, glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
mosfet.psfbc = device_mosfet_setup(used_device, glb_time.fPWM_PSFBC, hwdata.psfbc.udc1_nom);
mosfet.cllc = device_mosfet_setup(used_device, glb_time.fPWM_CLLC, hwdata.cllc.udc1_nom);
mosfet.isop_rail = device_mosfet_setup(used_device, glb_time.fPWM_ISOP_RAIL, hwdata.isop_rail.udc1_nom);
%[text] ### 
%[text] ### DEVICES settings (Ideal switch)
used_device = 'silicon_high_power_ideal_switch';
ideal_switch = device_ideal_switch_setting(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.afe = device_ideal_switch_setting(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.inv = device_ideal_switch_setting(used_device, glb_time.fPWM_INV, hwdata.inv.udc_nom);
ideal_switch.dab = device_ideal_switch_setting(used_device, glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
ideal_switch.psfbc = device_ideal_switch_setting(used_device, glb_time.fPWM_PSFBC, hwdata.psfbc.udc1_nom);
ideal_switch.cllc = device_ideal_switch_setting(used_device, glb_time.fPWM_CLLC, hwdata.cllc.udc1_nom);
ideal_switch.isop_rail = device_ideal_switch_setting(used_device, glb_time.fPWM_ISOP_RAIL, hwdata.isop_rail.udc1_nom);
%[text] ### 
%[text] ### Setting Global Faults
time_aux_power_supply_fault = 1e3;
%[text] ### Lithium Ion Battery
% nominal_battery_voltage_1 = hwdata.cllc.udc1_bez;
% nominal_battery_voltage_1 = hwdata.dab.udc1_bez;
nominal_battery_voltage_1 = hwdata.afe.udc_nom;
% nominal_battery_voltage_2 = hwdata.cllc.udc2_bez;
% nominal_battery_voltage_2 = hwdata.dab.udc2_bez;
nominal_battery_voltage_2 = hwdata.afe.udc_nom;
nominal_battery_power = 250e3;

% nominal_battery_voltage_1 = catenary_nominal_voltage * 1.5;
% nominal_battery_voltage_2 = catenary_nominal_voltage * 1.5;
% nominal_battery_power = catenary_nominal_power;

initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage_1, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:12a6233c]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage_2, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:78c0e0e6]

% special settings for simulation
lithium_ion_battery_1.R0 = lithium_ion_battery_1.R0/2;
lithium_ion_battery_1.R1 = lithium_ion_battery_1.R1/2;
lithium_ion_battery_2.R0 = lithium_ion_battery_2.R0/2;
lithium_ion_battery_2.R1 = lithium_ion_battery_2.R1/2;
lithium_ion_battery_1.C1 = lithium_ion_battery_1.C1/200;
lithium_ion_battery_2.C1 = lithium_ion_battery_2.C1/200;

%[text] ### Setting for Load Transformer
% special settings
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
open_system(model); %[output:347eff3a]
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

%[text] ### CREATE\_POWER\_METER\_BUS
% Crea l'oggetto Simulink.Bus da assegnare all'uscita "m" del
% MATLAB Function block three_phase_power_meter_iec.
% Nel Symbols pane impostare Type = Bus: PowerMeterIecBus.

names = {'Q1p','Q1n','P1n','P1p','U1n','U1p', ...
    'i_act_1n','i_react_1n','i_act_1p','i_react_1p'};

elems = Simulink.BusElement.empty(0,1);
for k = 1:numel(names)
    e            = Simulink.BusElement;
    e.Name       = names{k};
    e.Dimensions = 1;
    e.DataType   = 'double';
    e.Complexity = 'real';
    elems(k,1)   = e;
end

PowerMeterIecBus          = Simulink.Bus;
PowerMeterIecBus.Elements = elems;

assignin('base','PowerMeterIecBus',PowerMeterIecBus);


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":33.8}
%---
%[output:63b262a3]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V\nNominal Voltage: 690 V | Nominal Current: 2.324276e+02 A\nCurrent Normalization Data: 328.70 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:713c3bf1]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:83b4ff2a]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:37ea9137]
%   data: {"dataType":"text","outputData":{"text":"Single Phase PSFBC: PSFBC_1200V\nNominal Power: 275000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls1: 3.030303e-05 [H] | Internal Tank Ls2: 3.030303e-05 [H]\n---------------------------\n","truncated":false}}
%---
%[output:3c4494f2]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: Three_phase_DAB_1200V\nNominal Power: 750000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 4.000000e-05 [H] | Internal Tank Cs: 750 [F]\n---------------------------\n","truncated":false}}
%---
%[output:3a0fb825]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.580123e-05 [H] | Internal Tank Cs: 4.734509e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:4e5b197c]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUEAAADBCAYAAABRyQJdAAAQAElEQVR4AeydC5gVxbXvFwTDQ94BhBHIgBjwICICyXjkgJ6LkZwETfjEiNcgwSuIcK4kIAhKHCWIIHggER\/EECEEouQiSLgZAzlCxIgHORMgiCQII4\/hMX6CDnEwGjnzq1Bjs5k9sx\/d1b33Xn6u6d7VVbWq\/t39Z1XVqtV1T+t\/ioAioAjkMAJ1Rf9TBBQBRSCHEVASzOGbr11XBBQBkZwjQb3pioAioAh4EVAS9KKh54qAIpBzCCgJ5twt1w4rAoqAFwElQS8a2XiufVIEFIEaEVASrBEevagIKALZjoCSYLbfYe2fIqAI1IiAkmCN8OjFzENAW6wIJIeAkmByeGluRUARyDIElASz7IZqdxQBRSA5BJQEk8NLcysCUUNA25MmAkqCaQKYy8V37NghPXv2lM6dO1fJLbfcIidPnkwIFvKRf\/369QnlJ59XF+cTJ040ZZOtyxTSP4pAJQJKgpUg6P+pI9CsWTNZvXq17N27V7Zv324q2rx5szkG8adDhw5C\/VZfaWmpQISNGzeWZcuWycCBA4NQq3VmMQJKgll8c8Pumtdyg6hsezjHihs2bJiUl5fbZHnqqaeqLEryVF2IcwLxTZkyRXbv3i0lJSVirUprFaIDIZ00hHObNnjwYMGaRS\/pAwYMMG3wtpu8\/D527JiQf+7cuaaNWMCkU4Y81BGnmZrsMwJ+V6ck6DeiOVbf+++\/LzfccIMhhssuu0z69+9vrDHI5aGHHjJWIhYiFhtEAXFs2bLFWHMPP\/yw7N+\/3yBG+u9\/\/3tjTdr8pJmLNfy54IILzFVIypxU\/ikrK5MZM2YY6xSrEaLdt2+fLF26tPKqGB0jR46s0k3ioUOHZMWKFXLnnXfy05TF2pw0aZIUFRWZtA8++EAOHz5srl177bUyffp0U4Y8tB2SNRn1T0YhoCSYUbcreo2NHQ5DBpDd0aNHpX379tKpUyfBYoMc9+zZYwilb9++0qZNG3Ote\/fuplNcg7AgUoRzSz4mQxJ\/0Nm6dWtjGRYUFBiyq6ioENpGO2gPOjp27FhV64UXXiiNGjUyvxlSQ8BYeLNnzzZp9s+gQYPMaZcuXcT2g3OTqH8yEgElwYy8bdFsNOQCyUBoqbRwyJAhxsrCAkPmzJlTazWQLZkgVY4IBAbJYe1Bpl6y43pNgjXH0HjRokXGYsTKqym\/Xst8BCJPgpkPce70AALB2sIyYph68OBBYRjqTceSYjjM8JVrO3fuNABRZt26dWaOjvwQERaluRjnD\/lmzpwpXbt2lVatWp2VCwsQYWjNkLthw4ZmqE77KGfTzyrk+QGB8pP8HFWyFwElwey9t056FjsniNJbb71VevToIT\/4wQ\/MfCFWWV5enplvY6jJMBKCmjp1qlgrjfQxY8ack5\/6vHLgwAGhLENVW2+sxUg6c3wcmedjyI3FSLuoi3QsPaubNCvWmh01apRcddVV0rt3b2E+88MPP7RZ9JhlCCgJZtkNddkdiG7btm1nDWFxU4FIaAfExrAW8RIV56StWbNGEPKR\/87KRQnSEfKQ5hXycc0rNh860U0ehsYbN2407Xr66aerXGdsHspj6bHQgcWKXspyHX38Jg99mzBhgimfn58v1En9No\/VTZq3PNdVMgcBJcGo3SttT2AIMLzGgkSw9KZNm2YWaAJTqBVnBAJKghlxm7SRfiBgLTysPAQLzo96tY7MRkBJMLPvn7ZeEVAE0kRASTBNALV4ughoeUUgXASUBMPFX7UrAopAyAgoCYZ8A1S9IqAIhIuAkmC4+Kv23ENAexwxBJQEI3ZDtDmKgCLgFoFIkqDXnwufLoQ0t9CoNkVAEcgFBCJFgmx8h\/AAHj8ur5DGNfJwrqIIKAKZgUDUWxkZEmRDPduUID6cWmOBI41r5Im9pr8VAUVAEUgVgciQIPs92adZW0cSyVNbHXpdEVAEFAGLQGRIkAYRjZiw5Qx7mQNEOCeNa+RRUQQUAUXATwR8J8FUG0eMN2LD8Q0HAmEuX77cVMUQmBBLXCOPSdQ\/ioAioAj4hEBkSNDGayPWG0NjYs4RaJN+EnGYo83DuYoioAgoAn4gEBkS9KMzWocioAgoAskioCSYLGKx+fW3IqAIZDQCkSJBQqLb0OkrV64UAl+yMEIa1zIaaW28IqAIRBKByJAg84CEL2chpDrhGnkiiaI2ShFQBDIWgciQIM7SAwYMMB\/xxvqLFa6RJ2ORzpqGa0cUgexCIDIkiJWHtYcVOGnSJGFFmHOE38OGDQvkexB8FhKXHJXNohgoBn4\/AyXFm6R8w7Oy56WlNT5fvIdhUWtkSNACgLXHt14hPpsGIZLGNZvmxxHg77nnHuEbtyq3KA63KAZ+vAffG3GTPPvv18vhwmvk04f\/RXb8xx1Gaqqb95D30Y\/3Otk6IkeCjRo1Mn1gYcScVP6ZPXt25V8Re838SOIPQRfs8JpdKLYooL\/++uvy6KOPms8q8tnEIOXuu+82ql3pw8H8tttukyVLljjpH9i51ulA31nYudYHppnw3CyZP1MWjrpO5n7psPyixwGZ2vd86Tf4Zmk99mdSd8or0q7w5bNwpF9W6B\/vIe+jeUEc\/4kcCTZu3FjmzZsn7BixxLVlyxaTxrVk8cF6nD9\/vqxevdrI2rVrhTRvPRdeeKGwAh203HDDDQIp9e7d25m+4cOHS58+fZzoAz\/66FJntuuzmEbxuelzUVvpfuot6Vg0RfKW3Cxf2l9URXwdn9hnCLDJ1SNqffa+8pWveF9H5+eRI0EQ8M4PMifIXCFpXEtWtm\/fLh988IFccMEF0qlTJ2nSpImQZuv5tFEreW3H2\/KH7Xtk77GTgcrfPt9crhl8k3xY9\/xA9dh+lH7wiZw+v7VwtGlBH9HlUqcf+jbtOSGxsnzLEfHKrJf2CTLhhX0yZ\/Mp4Th2+S6TRjrlT506JUFI8+bN5dvf\/rY0a9YskPpj28y70a5dOw7n6ivdI8eff1BKK4e6++\/qJGXLpsknTS6QJrc\/KW0e22WO5xXcfG65ONiUlJRI2O5vkSFBrDP2DRvka\/iTSJ7Y4lh63qH0nj17qrJ88NVZMn1Xnnzj2QPS55EtgUrBnGL55tIj0m\/enwLVE3Q\/sq3+658olliB4Lzy89cOCrJx97vy59ITVbL+T0cqibDElAeXwlW7ZOvuA3L48GHf5MiRI1JWViZHjx71rc6a2nfOtTe3SOmSe6X0gaul9O6L5fj6Z+RUwy9IndHLpM6U38vfh8yUk12vS6ltixcvFuYDq17IEE4iQ4JYekSLYQj81FNPnQMFaVwjzzkX00hovGm2PHlDa3nxrl6By\/Mju8nCIW1l5agegeuiP671haHTrz6+cW9fiZXSh68UK\/baK3dfKj\/+egtZcfslsmZsLyPk4fp3rmwvRXtOyeDFB2XJ9lOCNeWHMIpp3bq1tG3b1rc6a2oXludHR94WWf8jOf3UMDk9s7\/U++NqadLxEjO3l\/ejv0je3UulXb9vpd0ehvnMCabxCqddNDIkSE8GDhwoDH85h\/C8QhrXyMN5MoK57Q2+YAMzUEe9d3fLF+uflH5dmjuQFtL7wgZSkN\/Yga7mMvDSdnJjv0vk6m6tnOgDQ9c6\/dLXuU1jiZUGDRpIrLRo0UK6desmEIX3GmUnX9dJ\/nj\/lUbeOHhKsPw5evOlct6wYUOpX7\/+OW1Jpa6aytQrPyIVL86S8vuukM8\/dq18rnhVFfHZOb6G3a\/2tR35+fmic4IwUYzYKNKQnhXSYrIl9JOoNE2bNjVDiX379kl5ebmQllBhzaQIpIBAx5YNjIU4rG9bM0wevKA4hVqCL\/JJWYlU7NwgZQu+K8zvIfj0Nbl6hLH4vMQXfGvC01A3PNVuNDPMxtxmFRHhnDQ32lVLLiNgLUMwuPyHr5nFF87DlFjiO1y5wAEReomvxU0PCBZfmO10qTtSw+GgOs4Q2lqUnAelJ2r1suqH9cvRVdvQ5VJn1PXFWoWsJLu6F1YPxIeFZy0+S3wthhZWWXxe4nONqW1nWMesIkG7eMJconevcTxn6bBAV725h4C1CjftOSEurMJY4oMAsfggPoa5iJf4cu+OfNbjSJIg7jKQGGS2atUqGT16tNT2jRGu4wjN3kesPiJTjx8\/XkpKSuQsZ+lqnKUlS\/9jEhzfSI6uuogulzozSV\/QVmEs8eHP93HlvB+7NjqvOC2W+Oq1zq\/xcXCNaY2NcXAxciTId0QgLwImDBkyRNglMnToUGG7EtfiYdKjRw9Zs2ZNVZCFQYMGmaw7d+6UmpylycSWHciSYUDQ8tFHH0lFRUXCzqRBt0frD8bBuSZc7x7QTjZP7GX8DrEKN7z1bq3PQ9znpnSPCVDgdV6uOFVhnJZxXm55728kGeflmtodxLWSSiPl0KFDvIahSeRI0LqyQIAWFbuaa6\/Z9JqORUVF0r9\/f+NaUJOzNHVgKeK0eY6TqI8Or9Tt2umVB4wthxzR70LQ5VJnpuo776Pjsua29jKoSwMZsnCHFK7aFdfZ+Jzn5s0tcnjN41XOy+zaMM7LNz0q6Tov84y4xJT37p577uE1DE0iR4J2Z0c6ARSYGwTRRN1qCGiA02a7du3Sdv6sqQ7XTq\/4YPXt21c41tQuP6+hi6kIjn7WG68u9GSyvsJvXmKc562T9dt\/bXDOM2iem3ofSf1Xnq5yXv7cywuMDx9D3Srn5cHjzikbD7ea0l1iynuHxwbva1gSORJk+GsDKECEhNjHsiCNaxYo5gDZPcK8ISF67FB54sSJJsucOXPMkT+Y214r0usszXUsRW48cyFBiiun1yD7oHWf60CdLiY4s+NkzY6Tmxa9JfM3HjYOydZ5uWL+kLjOy02uHmHyptuGsMrz3qmzNCwUI\/jxbdy40eweYZGDc9K82ZgD3LZtm8lDSB4IEgJkLtBrATKUVmdpL3J6HlUEJlxRR5b90zZpsHyM7B1axzgw49pSp8WFZp8uc3xYfrnkw+fiXkXGEvSuCGPdxQqrxeSJBwpuMNZytGWxEBleY27jKI1wHkuo8erM9HQmsl367IGXa53V6KMZgYnf+mJXdL\/00vfkW412S\/nAyTK6zWOy\/Ib\/lBajnxG5qCCwPsVW7HcfY+uP2u\/IkCDEhMWH5TcphfD6OEFT1ivWQvRe4zxqN0HbkzsIQHoVOzecFY4KVxYQwMrDjQXpOfoReeB73xXCeRXMKZath06RRSUABCJDgrZvWHuE0ocIbRorxaRxzabVdGS+kJVhjuTDSrTWoV00IT3bhXkelz574OlaZ9T1VUd67NhgmHte6\/yqHRsQIPN7Xh8+AlIwVzi0VysZtfKIzP3dASAOXFxjGniHalEQORJk+EqbGdpyRJIJr88CCT6F77\/\/PkVNFGlcYGqKLG0y6h9FIE0ELOFBcOzQsL57XtKD7Ag1j7XHeSLzexP+VwfjTrOi+N1qd5uk2eycLx45EmSBg5XgVMPrL126VC6++GLp2LGjublEfL48cwAAEABJREFUka7NWdpkzMI\/YcztuNbpQh\/kZqV823rZt2KOlBU9ZaKvWLKzCxkQHsNbhrxYehCdl\/Sw9hIhvtjHMa9pPeNgPexMZJog9yC7wDS2f2H+jhwJAoZ3fpA5PuYKSeNaTcLwt7i4WEaMGHFWNlxgrIXJBW9kaT4M87lnvmPChfMvd5Dy3iNfM35ex354rTN99RePFPQG2S9v3ehyqdMvfYSRihWIDfGml1Xeu9PP3yPlK2cIhFi+f5fY8PKEmG9x72+EVVyE3+zWqFO5qAGxpCN2x4jdbcIuk0R3mySrl3cEP0KOyZZNNn+J7hgB5rOFeT9Wgu0cnj2SxrWzc3\/2i2Hw448\/LuPGjZPzzz\/\/swsJnPGVq3cqb8YptrMFKH89eVJ4mD\/5+GMJWldO1t\/wC8LOiVTkky\/2FiOX3yCfnJE67MCwQih5ZMrvpc7svWZnBrsz6ty5vCq8PCHmT7TsGnfnB7sxUpHYHSPsNnni+lZVu01G\/3yH7zpTaWcqZRYvXpx8eP0E3ulkskTOEsTi2+jxEdy7d6+wSDJt2rSqfcF0EKuvZ8+eAkniCvPmm28KwRNwgykoKJCdO3fKrbfeKvv37zcfconnLD3hz+3k0zuWmgnqvAc3SJDS6r7fCi9NyylFgeqxfcAq+dt3nxVX+tDrWudZ+u5easK+E\/o9ZRn+iOSdkXbswLDS71tCOPnmnS41\/5ARYRpryYWYHSOtzw2vz24Twvq\/97d6JqR\/dbtNUmkfUbP5x9pFH3XHCGyWgLA6vGjRIrPIYbPHOkt\/+ctflm1nnKchw+7duwvzg9\/4xjekNmdphsv5+fnCqliQ4nrHiGt9YOdaZ7brqw1TwvrzrRO722ToT3el\/Ry7xJT3TneMWFar4cjixqEUI01gWeIgjYWIcE5aDeqy5hIvkLrI+Hs7w8A0kR5MPvN9E\/IyV5jOwklU+0jfgpDIDYeZ92P+j2Gukc6dhf3DhNZKlLzIR1gtrEVAw0GaYTXCOWkqikC2IWDjFUKIs14qkah+2yRquEeOBCGw2DlByMu7HzgeiCyOMD8IeTJfyLwheXPVWZqVOt02xxPgn4SBabKtx40GJ2vKtfz+y5KsVZgJfaRvfknkSBBLECLjaDvJeWyaveY9FhYWmhiCkObcuXOFlSfKqrO0FyU9zwUErFXIt6CxChki739Pt95Vd+8jQ4LWimNll4UNjlh0COc03uvrx2+vQHalpaXCIgrpDHsJp8V8YmY5S9N6fySMuR3XOrNdX7pPgt16h3UIESZiFbrGNN0+pls+MiTIThECHlgC5IhFZ4Vr5Kmtw+PHjzduM97hMKu\/XgL1OktTn4bXdx9iniGXSuK447KS6mcZ2jQSwcn6+ZHdkgrp7+L+lJSUGBc23sOwJDIkiCVnh7ysBGP9YQVaYbGEPPGAOnr0qPENHDlypIkxOGbMGPNdEh6eeGVsOsNlhs6pOHsmUybW6TWZsqnk5QEjIC3HVMqnUgZdLnVmuz7ugV\/PzUXnnzJ7kGsL6e8SU947Da9\/holYEMHau\/TSS6W6hRHSyHMmu\/n6HNYeJAl5NmnSxOwXxrGUPESPhkwhQY7xnKXJm63h9cGidTVOtqk40CZaJgWdaYWEz3Z94O53H62TdbyQ\/n7row\/xRJ2lYaBqhHBXkJtXYi1B3F+sczTkidNl165d5dVXXzU1MuRlGHzFFVfkrLM0Hv\/dunUTdgAwz+NCXOvMdn3csyCcl3GyZgXZOlnbkP7oc4kp7606SxvK+uwPQ14iyCxcuNAMa+2cYKwl+FmJz87YXkdZyJMj0WgAGQdpHKURzr0W5Wel9UwRyD0E8CmEDDftOZGzYbrqRvG2Y8HxbZBk2wa5QZYQJ0d+UwcrxaQhnJOWC8LEtvoJ+nun\/cDU3xalX5t1p2EF+fonioWADK6fm\/R7kXoNkSNBiIuo0N6gqol2D+doO0\/oHT7nqrN0orhpPkUABKxVWPrBJyYgw+aSkyRnvUSOBO1wmGjSDGuteEmturuCnyERpVkVxuLjW7TUQX2s\/uZiZGnmd3TvcHVPS+ppYWCaemuTL2mtQuYKhyzckRNb7yJHgliCDGUhMq+QxrV4txUfwry8vLMus0Kcy87SZ4GhPxSBJBCwViFFEnWyJm+1EvHEyJEglhtWn7UAvUeGugx542HKDhFWhSkzaNAgsfuNmWPMRWfpEydOyFtvvSXHjx8X5rJciGud2a7P3jNcvVJ1lrZ1JHq0mDapUyErbr9E\/m\/\/tsLWOwIy7D120tdnCZ9EXNjivdMu0iNHglh7RIxh+5u1BFn1RdgPzJC3OmAseQ6qJD\/KFRUVycSJE6vLek4aw2WcNnFMDVL8cnpNtI2u9dEu1zqzXV8UML2mgxgna0i0zyNbpHDVLt8iWfPeqbN0DCVBZnxeE9KzlyBE0tq2bWuTznGW5l8UgqfaVWXIkJ0L7BvmX5pcdJbGPYi5UY7xnFX9TkeXS53Zro\/749J5GX3VYdq7awcheOvKUT1k4X+dMAsnH9dvkZajO7qy01m6iqZSO7HDVhY1bA12pZhvh9h5v+qcpSE85gApx7CYYXCUnKXff\/99ee655+RE5TCVCXaVBmlHQc4FDKP03FzdrZXgV8jCScGcYvE6WadyLyBcdZaGsTzCAgdOzlhxzO0hOD7PmDFD7rvvPmGxw5O96pRhNB9amjBhggmgQBnqAWQcpHGURjgnb1XByhMCKBCwIWjZunWrCe\/FSnXQuqh\/06ZN8uCDDxri5bcLca0z2\/Vxz6L23JT++Y8yoNlRua\/3R\/LoC28YJ+vnNv7JfOOH9iYrjNQqX8PQ\/o\/cnCBIQFKsBjO3h3COqwfb4+xiB\/lixWsdUoZ6yIODNPUgnJOGtG\/fXvhXiDlB9h8HLXbuw5W+4cOHG9KdMmWKBN03W79rndmuD1yj+twsmHaXNP3tZDn2h+dkzOqylJ8x+sd7yPvIe+la6rpWmIg+FjSwAL3CijHzhYmUTzQPoBM8AXJNXZaJllUMcvkZWPmDm+XXIzqk9R7wHvI+Jvru+pmvrp+V+VEXRLd7925hyMjiCHuIObJibC07P\/TYOgCesF0qBaIYKAapPgP\/fFmXtJ4f3kP7Tro+Ro4EAYCwWKyIMf+Hq4tdHYYgua6iCCgCioBfCESOBFkdtivAEOG6devMvzBhT576BXgW1KNdUASyCoHIkSCrwzfeeKMQKZqFDhyksQxZ+Q1iOJxVd1M7owgoAkkjEDkSpAfMS0CAnLOaS\/BU+5s0FUVAEVAE\/EIgkiToV+e0HkXADwS0juxGIDIkyKIHbjBetxjvOdfIk923Q3unCCgCrhGIDAnGdhzXGJybrXidn2Pz6m9FQBFQBFJFIFQS9H5QiXnAAwcOCD6BbLuZPn262f6GNahWYKq3V8spAikgkGNFQiFBG+4erK2lZ4+kQYjTpk2r+tAS50OHDpXahsO2XoiT7UZEm6Y+bzrES5qKIqAIKAIg4JwEITJWeyG96vYBk8ZukbFjx1ZZgosWLZK1a9dKTS4y1Iv1aIfRfKeksLDQECd7dakToR7y0nkVRUARUASckyBERqSXWOghJoa9WHHjxo2TV155pcoSZF8m\/oOxZby\/8Sv0xhNkt0lpaakQjYYQWzheE4QBn0MbbstbXs8VAUUgNxGoKyH120t6EB9DYOYEac4777xjoruQbgWCpAzXqxNIDrKzBEc8QXaZEJacuILsRLHluGbPDx48mHIIIOYuVTYrfpujhUFJ8SaxUrFzg1gp3\/CsxJPjzz8o8aRswXfFhdh30vXRuSVoO4hFSFAE9gUzNEZYFEGeeeYZufLKK6ssQa7VtjpMfcwdjho1ygyji4uLBcvQ6qvuCAESxof5Q5VbUg6FpNgFi933RtwkyKzRQ+TZf79eiiZ8TYqnXmvkcOE18unMfzHSZu6Vgnz6cOXvM8J1KzUR2a5f\/FDiyV9+u1SQTWt+KUEK72N172nQaXWDVhCvfqw6QuZDejYPhEiaN4y+vZbIkd0lECbCQgpD3\/r168uhQ4ekuvD6gE5AVcL4MOQOWgjoSj9c6eN7LIQvX7JkSVphjpLBxbXObNK3ZP5MWXT\/aFk46jqZ36++zP3SYVnfe5\/8oscBI\/yelF8mI3s2lxv69zHSb\/DNcvFXb5VL\/vf90nrsz6qkXeHLYqXu1FfEyrEJr0l1UndKZZ4zcuCWpbLuivvl03teFm+6rc\/v4\/6vPSIT\/txOeB95P1xLaCRoh6e1hdFPFBBIdfDgwebbI6wKs5jC4gjfu8AiZM6Qr+qXl5eL\/Q6JrZvhMsPxoIXI1pBS7969TVAIF\/oIOtqnTx8n+ugPfXSpM5P19bmorXQ\/9ZZctPVp6Vg0RfKW3Cwdf3OvfGl\/kfTu3FYgOIjtr18vlG0Fk+XTSS9L5xWnpeMT+ySvkuQQriMtbnpAmlw9okoadr9arOT36idWuEe1iWtMCahq38UwjqGRIAsd8+bNMwsXdt6PkPi1hdGPBxLDYSwtbiAkRyQaVpq96VwjD2nx6gkyHQsXguAYpB5bN998YDGIo00L+ogulzozSd8nZSVmfo5haWnlMHb\/XZ3MPNzHlenntc43VhxWliU5yA1iu\/jmyXL5kNHSvkdB0LfP1O8aU6M0xD+hkWBsnx977DHp1q2bYMUx\/ILAYvPU9ts7HOYbxDa\/N51zm65HRSBoBLzEB+kxP1dRuVgB6VVHeFhvQbdJ6z8bgdBIELIbP3682MURLEPm8Zjj4drZzdRfqSDAd2L37dsnHFMpn0oZdDHtwDGV8smWQU8U9UF+rLaWPnCNWOJrMbTQDGWx9LDyokp4rjFN9p77nT80ErQLFSyG2E4xjOXcXuNcJXUEwhjWuNYZJX0QHy4odqjLOcNZSA9h3q5e5bA39TvqpqRrTN30Kr6W0EjQLozYbwrTRLtIYq+RpqIIRB0ByA+rj+EuR+9QN1OIL+oYB9m+0EiQ4S8LIyyGQIT497G7gzSuBdlprTu7EXDRO4gPS89r9dnhbpSHui6wyTQdoZEgQLFKixM0fn0I56RxTSV9BMKY23Gt06U+iO\/4+mdk7+QCiWf1pX\/Xwq\/BJabh91bEOQniz8cWOOsWE3vkGnmiAI62QRGA+LwWH8NdadleWoz+iVnkUKsv858R5ySIpYfFh+XHbhEWRjhH+M1qMXn8hDZXQ2mFMcHtWqff+izpxfryMc8H4XVeeEA6T\/6VtBj4f\/x8RNOry+fSfmPqc\/N8r845CdoeYO2xRQ7is2kQImlcs2npHqlLQ2mli2L2lYfs8NfDyrOEt3donaphLtcgPuvLBwGy0pt9SGiPQiNBuwLMooi9DUGsDhNVhugyRJlhJwP7iUmzOrP5GMbcjmudXn0QGwKBIRAcAskhdhHDkh3+ewxvyQvhQXSW9HBp4XesL59XX7Y+O7nQR++9C40EWQFmJZjVYTsvGNTqMHuDLenSeW8ora9+4aSc3LA4boghXrltxpkAABAASURBVCK\/pGLTUpE3\/p8TXbT5482\/lFbvbBSO\/HYh6EJnxYuzzJYwSKY2gaBqEwjMKyxMIKXfaSinJ3UWjvxGIDfE1LlsmpRvWy\/l+3fJJ00uELnoK9Lk9ielxb2\/MdLmsV2CkHZewc1S56IC41wOEVQnPD\/t2rXjUGO+6sqmk0ZIuIqKCic66ZyrPpaUlJgAJ+hMXPzNGRoJ0g3m\/uz8IHOCnJPGNVcyKb9MOm99ykm8tONP3yGnn79Hyn86xok+QwKOYsHF6jq+olC8svOFpySe7N5UJMiG14olnqwoqS9eeeHDrvLERwNkYdPbpLDl5CpZ2OsJMfKvL8qvv7td\/nv4b+Xw2P+UOncul78PmWnkZNfr5ETLrkYOHz4sUZcjR45IWVmZHD16NPJtTRbLxYsXC+HsXL3v1elxToLM0c2dO7e6tpyVlkieswrU8ONQnFBaFBm4tZMU3\/wrqfvoPsn7eUWg0mLhe\/K36buk5U+OB6rH737IgnJJV\/Lm\/0WsSCUG1Um7wg0SK5\/cXSRIm7HPCnJq2JNipcVND0j5wMnS5Orb5Phl3zZS9OnlsvDdbrLwL42lcP27MmrlERm8+KD0\/nGJOXL+ww0nZMn2U\/LyAZG3\/9pAsHqiLEzltG7dWtq2bRv5tiaLI1GVCGrCuxiWOCdBLL2ePXuawKfVffSINIbH5PEDFLbiJRJKKz8\/Xxo0aBCoNGzYUIhvGLQeWz\/48S8zR5uWyrFzm8aSqOQ1rSd1\/lomHOOV6deluSQqw\/q2Fa9Mvq6TeGXutzrJxIIGsuL2S2TN2F7yx\/uvlPceu8YI58iLd\/WSBcMuke9c2d7IsQ9F5m88IhNXlchNi96SgjnFRuZvPCxvHDxV4zMAln5gmsx9yNTnJpE+8t7lZCgtIrkw\/OWBgvC8QhrXyMO5SHp\/IV3+pSGMFsI5aenVqqUzAYGOLRsIAuFCpJY8IUuIEoJEuIbMeqlErn+iWC7\/4Wsy66V9sv+9U5nQTW1jmgg4twS97SVcFoTnFdK8efw4h1CtDs79qDMT6uBfYlbEObpqL7pc6kxHHwSJWHK0xAghLt9yxJAhhMi5xS8dfbaOqB9zoY\/eexAqCXoboueKQBQQsKSIhYhAiGOX7zKEuGnPiSg0UdvgMwJKgj4D6kN1vlWBS4bLWHs03LXOIPV5CREyZKg8Y+1fxDWm4OpSgsTUZT8S1aUkmChSGZgvjGGNa50u9FkyZIFl7u8OypzN\/1g8ycBHIqEmu8A0oYY4yhQqCeIuQ8AEFkZWrVolo0ePNh9KctR3VaMIJIUACywMkVkwYYicVGHNHFkEQiNBQuiPH6\/h9YN8MsIY1qSiMx0MXOtr00jkR\/\/WXPYeO2lWkNNpe1TLusY0bBxCI0EbQp+gCRYEfPo4t9c4V1EEoojAC6N7CAsluNJEsX3apsQRCI0E7V7eoAMoJA5F9uUMY27Htc4w9eGAjW8hZJhNT49rTMPGLjQS9AZQgAj9CK\/vjRt4yy23mM93ArA3nR0ppKkoAukiwIIJRMiqcVJ1aeZIIRAaCYIC5Ne3b1\/ZvHmzdOjQQQ4cOCDbt2\/nUtLCIsv06dNl4cKFgmN0\/\/79pbCwUEjP1XiCYcztuNYZtj5cZ666qLkMXlCc9DMb1QKuMQ0bh9BIkIWR4uJiYQO1JcPVq1fLihUrqiy4ZMAhwkbTpk3Fzit26dJFSktLhfBctcUTfP3114WQPtz8oMVlSCT64lpfGDpd9zFWH\/uXX337hGx4693AQl3F6gTnIMWVvpKSHA6lxeJHeXm5ECGD+H6QFuekcS0ZAiQvZSE7a0lSJ9FjuJk1xROkLJYiIX3YGB+kuA6JdPz4cROw4cSJE85CMLnWGQV95310XAoHtpIhC3cEgnPmPzfxw5Xx3uVcKC1IB2FhxEZ53r17t1x11VUmXhppXCNPMkJQhGnTpglzi\/gdYmViGSZSx6OPPmos0mTDACWbH6LO1pBIyWKRbflH\/evFwrD4rhff9T3cVTY\/N4wECWqSyHsaVJ7QhsMsjAwdOtSQVteuXY1FOHXqVBk3bpxwrbYO79ixQwi3BeHZRRCCIzAfiFA3hEroKixCr3WJ1emtH0uRkD6sigUp2RwSyeIGrljTHG1akEf0REUfiySvv3Oy1nBcyeKRzc8N711OhtLiwUUsac2ZM0ew5NasWSM9evTgUq1Cvm3btplFkGXLlgkkN3jwYLPjhPnGRYsWCYsjLLxgETJnyJ5Phtt23rBWJZpBEUgCAe9q8X4Nw5UEcuFmDcASTLxDEydONMFVseassI2OFd3Ea\/lHTkgUs5qYgZBcXl6eEJbLm8418pD2j1LZ\/ReLw2VYK9B0rTNq+ob1bWuGxZm8rc41pjw3YUpoJAjRMRfIijCf3cS1heOwYcOMVZgKKNayZDiMdWnr8KZzbtP1qAgEgcCasb1MtbqbxMAQ+T+hkSDIMGfHpC9zdEVFRcIWOr+\/O4yeXBVcKJgC4OgKA3S51BlVfcwPZupuEteYuno24+kJjQRZAWbISsMgwnXr1klBQUHon9+jPUmKZlcEzkGA+UEizoz75S6zx\/icDJoQGQRCI0FWgG+88UbjFsMiB1+XwzJ8\/PHHUx4ORwbViDQkjLkd1zqjrA8iJAYh2+oyaWjsGtOwX5fQSJCOY\/lBgJwzV8dqr\/1NmooikOkIQIRYhARZ4HslHDO9T9nW\/lBJkGAGdlXYHlNdHc62G+NHf4Ka26mpba51ZoI+iJDFElaOsQrZZxxlFxrXmNb0PLm4FhoJsjq8fPnyqoAHrOgiGzdu1OGwizuvOpwjMPm6ToJViGKsQiSThsm0OxslNBIETHZq4NPHeSqCJYmvoS3r3UXitShzNZRWGHM7rnVmmj5rFUKGWIasILf8\/svma3YQYhSGy64xte9vWMfQSBCHZXZ0EEEmlc5DfrNnz64qyi6RmTNnypgxY8wuEnaKcB2LkwAJ+CMia9euNeG1qgo6PGEj\/JIlS4SjQ7WqKl0EAigPGWIZvvfYNcLiiSVEhsuWFBk2Q4xzf3dAfrr2dfmvN0sCaIlW6ZwEISWsNOYAISmEcytcI09NtwYLcNCgQcav0OZjtdm63Ng0\/A+JKkN0Gdxw2D3BCjRpNg9HQmkR0zBo2bp1qxA1AzIOWhf1b9q0SR588EF57rnnTMxG0oIW1zqzQV+9d9+SAc2Oyv+\/qaE8fW0d+U6n96XT398RnstHX3hD+MLdE3svkG8uPSyWIBlKQ5LsTIEorSzfckQQLEorf9i+R6wkcv9dY8reft7DsMQ5CWIBMu\/H\/F91wjXy1AQI2+FYTY7Nwy6RPXv2mK14kCT5yMOwG79EzhHycGzfvr2weRtLkSAMQYsNGeRK3\/Dhww3pTpkyRYLum63ftc5s0zd57EhZ8x\/fl22Lp0rjTbOl6W8nS\/NVt5sjvxv99yI59ofnjGx5ea2s+M1GgSgRhtaQIoJFaeUbzx4QK\/\/2fIXEE5sHsv3x4X+SsS\/XrSrHteuf+KMEIWNWl8nJfpOE95H30rU4J0E66J274+VhKEt6OoL1iBUJ+UGu7EBhyFxTnYBOGC0CMKgsE8Uguhj88pkfyfM\/ekB+9chYWfmDm42smjiwcih9ufx6RAcjWJKI\/c2RobZX2MkSK5Ovyxfknm\/1kX\/IucdvD7hUhn5tQKDC+1jT+xrUNeckCOExd4dzNGTFEHbp0qVx+0d+iJLhMqGzINDqMhMlhmgxdqEFMrRRpTG3iTJjyzFMtucAj7+iSoHZsaM4ZD4O\/3xZF7HSr0tz8Qpzj7EyuXLVujaJJc4gftt30vXROQlaMrJkRVBFGw+uus4z14eFAmHW5EzNnB9zf3a+jyEvw+ArrrhCIEdIkj2tGkqrOpQ1TRHIXQSck2B1ULNaisVX3bVE05hHZMvdhAkTzJwgPojz5s0TgjYSPoswWgjn5E203qDz0W\/aSUTsXbt2Ba1O688SBPiswKxZs4RAxD\/5yU+Ez0hkSdecd6OuiHOdvilkIQSxFbLlDmsRq9G7wMIiCmkI5zZ\/FI647PTq1UseeeSRlD8yFYV+aBvcIrB\/\/3754he\/KA8\/\/LDccccd5lsybluQPdpCsQSZo2PuiXk+rDMiyDA85jeLGyxyZDrE9MFGurZ9wbWHPiI4cJP+9ttvS15enrRs2VLOP\/98EyGbdJXcRCDR54apnRdffFFuvfVWISL76dOncxMwH3rtnAQZimKlYZVVJ1wjjw99C60KFm+uvfZa4V9r2wjSsPrw0yKALOH\/GQp\/\/vOfN+Rn8+kxdxHgGUn0uWGRkOmfn\/3sZ4L\/6cGDB3MXuDR77pwE02yvH8UDrYN\/yXGIfvrpp6VZs2ZVul599VXBURt\/RaxeFmj415yvz5WVlZk5HeZ16tWrV1VGT3IHgWSfG6Z9WGTkealbt64Q9CB30PK3p0qC\/uJpgj8wT8nQNrZqhr2sdpPOSjYr1mwdZGL7oYceko4dO5phMddVcgsBRj\/JPDd4PjCPfP\/995t\/bDt16pRbgPnYWyVBH8FMpSoeXoY1M2bMMPM7qdShZXIPAfbG2+cGjwcswtxDwZ8eKwn6g2NCtZSWlgrzgGTGdxHfRs6DFq0\/sxHQ5ybY+6ckGCy+VbVfddVVwjwg8zg4dDM\/iBVYlUFPFIFqENDnphpQfE5SEvQZ0HjV4cP49a9\/3WxNw6GboAZ2fjBeGU1XBPS5Cf4ZUBIMCGMeXj4fytGqIKoNbkGs7HnT7XU9+oRABlfDc6HPjdsbqCToFm\/VpggoAhFDQEkwYjdEm6MIKAJuEVASdIu3alMEAkBAq0wHASXBdNDTsoqAIpDxCCgJZvwt1A4oAopAOggoCaaDnpZVBBSBMBDwVaeSoK9wamWKgCKQaQgoCWbaHdP2KgKKgK8IKAn6CqdWpggoApmGQCaQYKZhqu1VBBSBDEJASTCDblYUmkoUHPsJVD4TYGXixIlRaF5abSCy8wMPPGAi\/dBHPodgK7T9tp9FsOn2yHUwIDiqTdNjZiCgJJgZ9ylyrZw0aZKwD9oKAUEj18gkGgSJERF87NixSZT6LCvBMPh87IIFCz5L1LOMQEBJMIK3KVObhOWEBcXHsjjHsuJbGFiLpFkrCWuKNK6NHj1arAVFHq7Rf8qTbs\/Jj9g08vEhK4R09EJk5CcPaQj5EHudPJyTRl4rfPulS5cuJjK4TYt39NaPDuqjXoIfUIZ+c1TJDASUBDPjPkWulbNnzxYIAIG8LMHxJcEVK1bIkCFDZOrUqbJ06VJjMQ4bNkwoQ77p06cLH5viuyvEWKypc5AVUVWIwYgQYJQ0yvAhKz45CYGhlyPXtmzZIpyvXr1a5s+fL+3atSO7+ZIf33VBJ995MYln\/hQVFQkkeOanOdBe+oeQnzq5gNVrLWD6OXLkSMES5Bq66BfnKpmBgJJgZtynyLXSOxz2fiGQb1\/wMSm+nwJJ8UlVSARCgZw2bdpk+gKpQBx8Y8W8GJx3AAACsElEQVQkxPmzZ88eQ2jkRyAiCIvszZs3F6Jzow+9pJGf0PN8swPLjM9Rdu\/e3XzWFBKlXV27dj3L4sOKgxipizqsePtIWT4Ta69xxCIcNGiQeL9lDZHSBq6rZAYCSoKZcZ8yspV8OArSslYTZMlnBc7pTC0JWFu2Do5YYrUUOecyZAV5IpyfkyHJBIbrEJ6XAJOsQrNHBAElwYjciGxrBlYVX9RbuXKl6RqkwdwZxEEClhUWGENdfltrDivKm07+devWCfNspFMHdVGmOiE\/FifDbspgaXK0Vtzu3bsFi9JbFouUzx1gJXrT451b\/QTJjc1D+2lDbLr+ji4CSoLRvTcZ3TKGo3wN7cknnzRzh8uXL5d58+ZJfn6+TJs2TUaNGiV8PwPyoaMQEYTFsNmbjqU1ZswYYVgNefHZ0urIhzoQ8jP\/COlRBtJkWEz9kFPsUJgyCNYhViLnNQnETl9oJ8N8xM6JQtLFxcWmXzXVodeihYCSYLTuR+RbA5ksW7ZMqiMi0rhGHjoC+fApAYawDIUhRtIhKtK41qtXL5KMUN6m8\/F6O+y16VyzadRh60QfekmjIm9+zkmDoLA6ITt+xwqkSRoWpK3PliXdpjE0Ry9tscJv+saiS9u2bYV+Uya+6JUoIaAkGKW7oW0JBAGIjY9cUbklO869AslBkFh43vREzyFZLN8bb7wx0SKaLyIIKAlG5EbkajOwtqx1FxQGWGlYa1iLkF08PViSqbaFerFe1QqMh25005UEo3tvtGWKQLYgEOl+KAlG+vZo4xQBRSBoBJQEg0ZY61cEFIFII6AkGOnbo41TBBSBoBEIggSDbrPWrwgoAoqAbwgoCfoGpVakCCgCmYiAkmAm3jVtsyKgCPiGgJKgD1BqFYqAIpC5CCgJZu6905YrAoqADwj8DwAAAP\/\/4+pWJgAAAAZJREFUAwBoC\/w+Pc3oeQAAAABJRU5ErkJggg==","height":193,"width":321}}
%---
%[output:03f6d43a]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:0cdca7b1]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:9ed44b05]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:7c22730c]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:31c361a7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUEAAADBCAYAAABRyQJdAAAQAElEQVR4AezdB7htRXUH8DnYUIM+xQZqhCj2ArFhFH3EqCiioNgbWGNsWKlSpFloCgaNVDWgERWjiKhEELAQ+AAFVBAFUVARaaIi8iXvt2Vd5u239+n3nnPunfe9\/529Z9bMrFkz+7+n7Tmr\/V\/5VyxQLFAssIQtsFoq\/4oFigWKBZawBQoJLuHKL0UvFigWSGnJkWCp9GKBYoFigdwChQRza5TrYoFigSVngUKCS67KS4GLBYoFcgsUEsytsRivS5mKBYoFulqgkGBX85TAYoFigcVugUKCi72GS\/mKBYoFulqgkGBX85TA2bNA0bhYYDALTDUJ\/uEPf0gvfelL0z\/8wz+0Qji5wYrdXfpjH\/tYld9TnvKU9Nvf\/ra7cI\/Qb37zm1VaUYZ3vetdq8TgF+Fc+a8itMAeP\/zhD9OjHvWoSndliOzZg11yvwgbxVVmZZe2PEZJK+LSUZo5xpl+5NPkapPaprzVb5NM3S+PIx796zLu+QtXP+qJX7\/I8+hXr37Tll7oFDrSsw1k+k27Lqe9sK\/y1MMGvZ9qEuynMN\/73vfSpptuOjJZ9ZPXOGT+93\/\/dyVdPfD8xpH2fKfhgXva056WLr300vnOauT0PZCvf\/3rV0mH7sqgLKsETtjj5z\/\/eTrvvPPmtPja1742dz3tF9GOly1blu55z3v2pa76QWZ9CWdC6vaDH\/xg5jPa5cyToOJr2D\/4wQ9cTj2uvvrq9Jvf\/GZOT9f85jym5OIRj3hEOuecc9LPfvaz9C\/\/8i9TolV\/aiC4b3zjG63C1113Xdp7773TOHoRbZn83d\/9XTrqqKMq++2zzz5tYiv5n3baaYlu4enliFzivsWdCu9ox4997GPTPe5xj751+va3vz2v9dCPIjNDgs973vOqBuWhDHzpS19Ka6yxRlXOn\/70p5Ubf3S18254dNMjPFwPjLCQ7fZmqqc5SHecnve5z32qRq6xR\/6uNfwNN9ww3fe+9w3vObeuHz2b8q3L0VVZyMcQ0EMvLj9vUzKuA+Qj4zw9csKe+9znVvqT8RZnN3LSkoa05SEcxOEfcvxAHH7CgBz\/JsibTKCeR1McDySbCvuP\/\/iPldqNdsRfj0vPy3Wgnhcd6Rrh4fITFjqJpwzu22wdcdtcZHf00UdXwXqq2ouX+xe+8IXKr9sfceUrf3pEfbinJ33b4tOdXED8XDZPO2TkxT+Xi3b8gAc8IPeunk\/PaTyz4UY9\/OpXv0p\/\/OMf5+LQlc5NeclT3mETo8BHPvKRSXklEOERl0uev\/A2zAwJthUg\/HPjM4qHNMK4HgoPcV7JGgA\/YWRANxtc5xCvnqZKGGQoLi9pxtsPYbjmt95663FWggbx8pe\/fI54IlC+dBGfX1M5hDeVgzxoSGRcB8grZ9zPh9ukq3yhnh9d6joq+yA2P+yww1bqaeiVeRD1cvV2I89R2gwdm\/SPtPtxjWSQHvJ785vfnBCheNpH1LP7XqCHug05bVu7Y\/fwC5cc3eOeK37IIo8XvOAFq0x\/0JO\/cHFAJ4TuT3ziE932jXvf+97pDne4QyUvX7rSufJY8UdebOFZWHHb+p8udCKfC7nnLzz3z69nhgRVGGbPEQbTiwIFY0iyrt\/znvfM9QLizXPwwQcnBtWwPCDk9MA8XB4OPQd+OciLxy\/SJC8eI0d+wrvhXve6V\/VmjLefN6BrjWejjTZaJeqRRx5ZESDd6QbyJxg9mUHKIV4OZZWmNzUdhLU9dP\/6r\/+acjlx60QifjcMousoNtcWgC7qSW9Bu9HDkC7\/HPPRZvL0+7mO+b+HPexhad11102bbLJJFS3qubrp84+6Ua\/Kro2Kpq2zv+scTbKhi3atfW+4YpSCpKVJXnz+\/FxL97LLLktN84EIzXPK\/jmkLe6rX\/3qZOoASe2+++68UrR36ctbGp4Fw+yTTz45CScojIwXm\/ToFH5tuopXx8yQYF3x\/D4MyS8qkDH0ovgB8tAgGFTX3VBIAxP2kpe8ZG4ew\/xXGFkYkBdP\/AhTIeIJbyMOYTnuf\/\/7J41cZak8cK3xIMhc1rXKVZlcjUTX3ptaWGCQckQcLvuAaz0ib1vX84lBdB3F5h6qAw44YJXpBXUYDyTii7LOR5uJtPtx1a35P7JPfvKTK1JA3NobnRGAsH6gfWrDZLXR9773vS6rBRf2r25u\/qP+wS1Z83muERpi8+LT\/sxt8jcVUe858peuZ0l86fDrB\/IGsvEseBm\/6lWv4lXZwbPthn3YyXUT+tW1Ke6iIEEVY+ik4lSggq699tqVEV2DLreut2tdd24gH0rzq9+HPMJSafFGC0LSm9OrE7cb7njHOyZ6kfHggeu2xqPSEZ\/85Ct\/8m2o612\/z+PRA1nkfgt5Xdetfj+qzT2Meg0eLrarl+2d73zn3IhgPtpMPb9u99GLIaNN1eu7FwGIF6jb0UotYkGm5kpDjturDXieEB99kLKepXh1SFf69bzrcvV76cXURtS3dOJFJV\/PtngWD+Xjugn96toUd2ZI0BvOW6kO\/gpmUvl3v\/udy6lGDHOsXp5++umVruFX3dz8R6Vus802c\/MxyqnserQ3ixSnDwsgej0ZtoMYznnY9Db7SGJeRdSzkUS3TLz8kHk3mfkI23XXXROikrYXCR3CfvwC8TJvIkEEbBqF7XPwE6ZsXgKR1rBuv7o2pT\/1JNikdJuf3p63m3Bvdw3MNeip6bG5rldWvIWEQf0+5A1PNIq8Ml3rceh5iNsL+Zv5l7\/8ZTVHyK8eL9dXwzMkrsvU7+t61+\/r8pO8r+tWvx\/W5uo8ei960XrTeTnZ2sPHT55Icj7ajPT7QQwle8m2zenV4ylT7qf3hPCVWdnzsG7X119\/ffIMkfHi9SJhK\/c52JucZ0NvMQ8b5Drqm54I0nOVo9v8c+ggv266Cm\/CzJCgt4XucR38FcxQFwlGrwpZffrTnxZUwTDDW4eRrWCZfDY\/J9CiR0yYmyuKNIUBefHEjzCGj4fNyiK5fpDnS54O\/Fy3Id60Hmg93lxOXGnwE0bGdVM5+M8HogGzOciDPdnVdSDXVRgZYU26DmtzD6p5NemqLz2YvM0YaiEF4dFWwqV7v21mXLbWG6WP9tX08Huo6WrODWG67gbtkz3JaAux2KCNsD\/\/QaGnqr0DMs7jx8s6nr88zLWysXleB675CSOj\/SBQRMovnwP1bJH3rMmffDd007Ut3syQYFsBwt8EqgfApLChI3\/Ex4CgcfDbd999k4UAsuLwY3iVQi7mIPgHyMfCQaSp0jw0Gm9M5IZ8N1e+8ZCS0wvh5zqHnqW5Qn50p5sH2oPNLyBulEMYGbLK8dCHPjTExuLqSVjEkZj0Y7VVI+YH\/OXPnuzKL5DrKowMWXFCJtxRbG5BjB0irSZXOAgbps2Mw9ZICpnSoY2k4mXAXjk5iNMG9mRX5aMnOW2E\/V33A\/PX0U61c+0dXOfxDZHlQXaQ9CMNOoL2HguN0d6VwTXZXP9ob3Shk6Gw\/MmFH3\/X\/HphrCR44403piuuuKKC616ZjyMcCXmDasiRnqGjIWTcc5vkxBFXGBlAoPH2dR9oStOby9yeBzbk+nGjEslGL8R1heyPytVAwsv1iSeeWK16eij0IoTVy6E8yvXsZz9b8NiQN1SJ0sFwS\/51e7fZkSzd6CgNaJMd1uYeRsO3uk7yAv7CybmHprzoSFc6kwHX\/IS557ofxtZBINLxEOf68AO9NwTpup8FEm0XyEPoR2\/3g8CKq7qJONq79qcd8jNCieF33qaF9QNp5\/UgP7akc8R3zS\/XXzy6hIzhuBcf\/\/ATXtc1wuruUCToDXbggQem97\/\/\/dU+PIl+61vfSk94whPS4x\/\/+AqPfvSj0yc\/+cl00003CR4KGgUj5XMD9eu2uQJGy2Xb5BCYsJD1MKgM9\/W5vnqa9fCmQkYceciLTPjJwzU\/BCM9fvLnVy8\/W3go6nJkpS0P8bnu+efI01POPMy9uPIgJ750+IWO5OnGLxBh3PDjSi9kpSM98cE1P3KQyyobW5CDerr1cDJtqMeVF\/BvisNfeICOdK3L8hNGjuu+LsOGbElG+erh7vP82IpfHXk6UfaI15a3tOQLdZk8vbpe7sWhNzm6hB9\/+Wt\/wt0Li7zoRD7gnkw3iB\/y4bIlnSOea34RztU+6BIy9KGv9MJPeF1XcZswMAmeffbZ1YEF+++\/f\/JG3WKLLdJnP\/vZFPuRsDHc9ra3TXvuuWf6+te\/3pTv2P3Mg+g+B9yPPZMpTNALyQKAcnPdU9P8ifkR14bVGo7rguEtwLZsXGw9vA2nMeZAJPjXv\/41HX744VU5uCZrX\/nKV6btt98+3e52t0vHHntswsbg2reyn\/\/859MNN9xQxZmvPwjPvi\/dZm8C5Ow+Jt7nK99pSBe5ITm6mJsxVPGQ5nMi3Ybc4k0Ws5N7sfXs1NUgmg5Egtdee2268MILk0+8zGHc\/va3T895znPS3e52t+rsOStEkTkC3GCDDZIVLfHCf9yuHo8VKwsX0W3WFXff70TyuHVa6PS8dPJ5oMi\/aT4lwoo7nAWKrYez2zTHGogE9QTtH1pttdVSp9OpynXnO985IcPcrwq4+Y84\/\/d\/\/3fz3fidWKKv93bc9zORPH6NJpNizM3oCQea5lMmo93iynXabK2Hag5MvdNtcVl7\/kuz2vxnMb85WJ1EsrZv1HPq9alNXX4S93QcFBdffHG6eACYx+2FtvRCt0nYZpHmWYo1ZRaYeRIcxZ4f\/vCH06jYbbfd0igYJn\/D\/EFgrrQX2tIL\/aKMcc8V5+Mf\/\/icDT\/wgQ9UthDm+ogjjkhH3AzbKSDI+Iwzzqg+ybKXq+B7S94Wvp4a5VkeJe5QJPjnP\/+5OiLeapnvdW2Dyf34A79RlJvPuIwevZxR3PnUcRrTzm2l9\/jrX\/86hV\/Ut3vXP\/nJTxJ8\/\/vfr3YQ2EWATN\/+9rend7\/73dWCmkU118D\/LW95S4I3vvGNCWz8tVHWApyvBgpeWv3uzmKzg\/r3TE6izQ9Fgl\/96leTVUiwSdRmxdyPP\/CbRKH6yZPBPcDrrLNOWmcesP7666f1h4RtGA9+8IMTLFu2LCEUuLhhCPzjH\/84zQfqeYVNV1999QT0Cr82lxzc6173SuvcbGM2Wb58+Vz7UUYbbS2kkbPfC6QPa665ZvWbFRbd7ne\/+yVwDaZAhD\/jGc9I4MudbbfdNoEtXPaPLRQ+9KEPpbe97W3VkfrjyHPY9AaJ149sL5m28Nw\/t0eTP7t5UXom29rSfPoPRIJWG33FYKN0vyAv3iiFcEyWbR8B3xNGeh6ETqdT7UeMcK5FgWXLuv\/oywknnJAe9KAHJQ\/PuOGTsByxidxDbDsRXHXVVRWBWcCxzeekk05KBxxwQDWkVGaw1SiGixc3kCAi74WwFZIBRARBHMquIept+e0NBGILVI5ddtklAYIha1eAa37uQToIHHzloG4i77qLHIE+CBIZ0ikI0jV\/UI958X+HCQAAEABJREFU\/Fvf+tYJxEeY8RJgv8985jMJ2FEvEwzP9ULBAkLIX3311RWhS0c+XtzDwpFQbELvYdPI4w2b3iDx+pHtJdMWPoi\/ZyOv34W+HogErQJvvPHG1WZphewH5MUbtmCIwMf25rSsfiEEpBFE6NAE6ZubChn7BN3rYVg5GzbvbvGCeMxxmeuCI444IpkLA0M78D0kG3CBn3Awpyaez9\/OPffcJK1ueeZhyAM8dJtsskkCpISMbJfxxvVdqi95uIDcgBxstdVWVTxxpYOIpAl5Xr2ukRQgkuXLl6flK2CEgGTlE0SZk6SeX1u6SCnSkybdpLnhhhsm1\/xg2YqXXFsaub+6Yltgb3YHdaA+QN3U64k\/GfUK4koDpJnnUa5n1wIDkaC5P98Gm+\/rF+TFG8ZE9gD66sHcUOwBRGq+TvHNbr4Z2lYcK8XD5KNBg8YNGjto+B4CDwPEg1J\/WMiAB0s8kA5Itx+dEA94yJESIA0kAsgLkSE1cA38hYM4m2++eXrmM5+ZHvOYxyTp9ZM3mdVXDHPXXXfdqmfkvhv6ka3LICzEhczgNa95TdWzRNqg16wHqfzkmvKXZqRDhqy0kCPENZft2IMM9GsL9QXqDtSlegV1rB1AvS1svfXW6SMf+Ui1SKTdBMSXDjSVqc1PWfutjzyNQeL1I9tLpi18UP+8DAt9PRAJXnnllWnLLbecm8\/R8Oqo35MXb5iCGeqYT6jvfTLMsj9RmvYJ\/ulPf0pvetObkkl0Q2HuG97whuTjbmRNrgnmlQyXNGjQuEFjBw0\/b8QejqZ0mvw8dLD++usnDyN4MJEVIC9AZG2kttVWK\/fUpNeU1yz7ITVgp+UrepDIkJ2aeo\/rrLNOY1E9cCAw3EsuuaSaSxVHutoDexvus7lrUBfyA3VEDxjE1toFIDrtRbsJaEfaFGy8YhQF2hrwAzJNpClNZSqYXwsMRILm9nbdddcU84EHHXRQetaznlX9Psc73vGOOX8N2CkO5BESd5zFMHzsdDrVhLnen32CT3\/606vDHAyZwTFP5nyEt+VtXqktrO7voQAPiIcFPDgeIvBAgQesX1KTFgLwVY2Fj3qew9xLZ5j0BonXj2wvmbbw3J9tgsQ2XDEU9pUSW+s5sr1jptQzmTZbaQPIyVyg6RLzwKZK3Js3RpriI0ppq0Noqkf+ZOQN2oA6BF9NtelQ90duQC\/IiRMhIkZAlEgTXAN\/IJcTpzSkZXGh3\/rPbV3XMe57ybSFD+of+U3CHYgEzb2pkJgLdFKzLTLHH3988jOB4a+BfOUrX0kPechDkiPkb3Ob24ytbHp2GqjvZQ2NR0nYcFtDDmjgoLGDfBAauAb+ZKDeU\/MwIMpBdPIQDjPsactj2PQGidePbC+ZtvB+\/IMcn\/SkJ1Wfa77oRS+aG1pre4bVgNza7IQcLTQhQ9Bzs33HIorrk046KZ20Ah5mdapuQVtR76ANaA\/wuc99LmknoJ0AfzJ0CogvHZBum351f6QJiA6QHj0BIQJyfMUrXpFsKTIlgjQDwsgEcXKV75prrkkWiqRdz9N9W30Ig7bwQf2lNSkMRIK5kr\/\/\/e\/TF7\/4xbTZZpulu9zlLnlQda33J8yEv1XQynPEP0hrm222qVIx+V9djPDHMF1leYj0LpavGI6BBQLwsHkIuuLPf66GXUVmMnZwOIfpEPZXl4hFrxHUKxIyFAYLNVYiTaeYamlqOt3IEUHqTSIPRIRE5VuHdgPakPb04he\/OAW0M0NysPquA8EN8CdD9wBCt4sBhulxIjj65sR55JFHVot4yBGQpQ4OuAb+iBOQJkhDWqDXWS\/7oPdsqDPVVBcL5Tc0CVrs0AAdrNmm7OWXX94WNLA\/AjS0ZjBv3VF7gaGAhq1B0LXg8jRLNvBwW3gz5dFNbyQJyA+RIEPHvdkSxI1htZ9EJRNtI3eRI3j4Tz755IQMtRsE8dGPfjQFQSCJs846ayA7ms4JyN8Qn44BvUhkDk42d2AIN6BDYKHJfGpAmZQVBiFOZWZXUFblAWUF5UWOoNepxwmuQScF9thjj2RnBxxzzDEJ9JLBgmbUlzRtlJbvpDA0Cer9PfzhD09+k0Gl1wtw5plnVl8JPO5xj2vsKdblGcuiRsCOeMRHjttGgBpNp9NJHgSyObyNhed+9WtbSTSytdZaK00CdPQyYc9x5D9seoPE60e2l0xb+CD+6vbud7970vsb1nbrrbdeMqzeZJNNqkW\/rbbaqtpsbaFNu9ALg25Da+1T+zPHCM7QdKCwo+HBRmBfzqhnU0r96Npmh4hrt0RA780CpAVEOiuLLWT2SsKnPvWppMcJrsFQXY8Teb7sZS+r9sqKZ5gObFp\/VrrdmxYD5QTz9l4UcOihhybISduQHeyPtUCpLgeZo++my6BhQ5Mghb2BVKoKUBEMD65f+MIXJg3UXCHZXoqpQAsaAavChiwaWE6AjiT3pvQ2kaZ9goxoOBEEynXfz7yhuBr46qv\/7UuIhXbZz8bpceU7bHr9x1u9OjWol8690msLH8S\/TXYcttR70i6W3zxFglws+CEO0OMKckRY2mId2i4gBURos7ZFGb0prnv7XkHPK9d72LL1imd\/JiA6pKfH6Gj6IH\/kCOY09doQJ\/LyPPEHPVL2gKc+9alJGtJDnFC3Q7d73ADKbptbN9n5ChuaBCmEQByaiujcmyME1wxl5z4Z98Ni1xWr0TEElkb8epZrQJSO8v\/ud7+bdtxxx2qF2Fv8O9\/5TlU5ZKYZKr8sjNxSQ232aPO\/JebCXCE88NAjSESAFIMg3SNHQKJtWhlamw8ztAbDwt12263aZ2iKxiq2bT7IkWxbOnX\/QezUjywZnQllUWZAnkgTdtppp2ToixwRJyBPbkAYPgD2AWlIC5Af1MuyUPcDkaD5F0fmqzRvOEre6U53SuYHFPyiiy5K4FrvTRiZYaG3Z1N0nJhsstu1eUhvYl1pehh6C6ObXqA5Gr93oks+bN4LFc9Ecr9bGvrRadj0BonXj2wvmbbwQf37sclCyNDbAqCeEHIED3sbOSLSJr0QHpiP85whR6QYBOnewgwg0Xoa9Oi3PfUj20umLVz5zMNyER3SAwtEXhBeHMgRTEnp6NTLslD3A5GgrjbFDHkNSe3N84mayrBQImycMOdhjsUQWT72HuoJWnk21+AEaZulGdD8ArmAe5\/X2VIzTp1KWkvOAiMXGBHoSSFGQI5IoI0g2zJEjp415AgIMcgRUXomTj311KT3SK4tneK\/sgUGIkFDT0NOe\/9Ugso0Z+Aoe2z\/1re+NfHXO1s5m9HuEBnys2xvAjVPzYS0lbW6PxmNRrjraYXhRhkO31I7bfZo878l5mSvhtWviSDbyJFsUym1c9B7NPqxMGO+MQgSWeo5Qk6O\/ejcS6YtfFD\/pnItlN9AJBhKdTqdZHLVqpI5QHsBfT3iF+bmo5dohc3cognc0GFcrglrDUO3vmAye\/1GsbsVV8OuUdKYxrhIxNDaNA\/YM2ivo3k1rnt7HqGpAxDPB3LUvnVOACEGOUbvETkiUHILbQt5GsmFvpNwhyLBuqJWCo3z\/QhN9BJf8IIXVMvyluBtSq7H6ffevKCVYt\/46on2G69fOQ1Bw4h9SwvtagSG7dxx5C2dYdIbJF4\/sr1k2sIH8TfsM0+ttz8O242SRpvew6bZLT2k75mzBxDsJ4w9j77a8qVWHGXWRpDIEZAfcjSU9hxYuTanHvseffmlZ0mfprLwb2pv\/frLc2b3CbaRTKfzt16ioaueon1Nw84XGlbbQW\/PoPnBtjxH8Tcpa1hPz0lAI7WVyFt\/HPkPm94g8fqR7SXTFj6If5vsOOw4aBrj1mWY9Ox5tOH7kY98ZDJfbvUW9B7tmNDOdVbAHGXbc+O582IxH3\/eeecl+4ANr43IwOjJth8yVnWb2m+b\/nV\/OpkfbdNlIfzH0hMMRc3NXXDBBcmQ2FFO9hFazb3VrW4VIq1u02bp888\/P6kE+5Ss+oJV51gdFodRO53hN0sbZmsQhh+TgE3SPq8y3zOO\/IdNb5B4\/cj2kmkLH8TfQp0e0TjsNmoabXoPm+6w6bXF63ffo7l9z0PTg4ocAQEiQtt4bJFDkHp0XHsewTfJTe26rp+8DOmb8lsov7GQ4I033lid7OzzGcvg3hbmB50s42Tkfj5xa9os7WsTb6NY8eVaJY7VYXFis7RPe3Kjube\/qZ+883jlenYsYDjsywzu7Gg9XZp6+QLys3Jt65neWdviTJv2htaGwIbWgBTNPYIpJ8PtfO6xLZ1J+K82SqYmNPfaa69kszJC0guUnhVke\/dsosb8\/OYL5glth0G89g3Kh2t\/ocp03w3eaN5ck4ItDRpK\/hYdRZdh0xskXj+yvWTawgfx18Y8bB6wUWw2jrhteg+b9rDpDRKvl6zTZRySgNQMfWP+0UINmMLR03VuZ7cXEYKszz1q88hR\/dlbbMtdt+d0PsMG7gnq9SGZLbbYIjnf7ZBDDqk+j7OFxXYZe\/n8jsY4j8\/qZQDzH3qIhsoxZN53331Tt3lEq9u64SrCnOOkYEJbQ7DwMw4dhk1vkHj9yPaSaQsfxD8m1Cddh+qtTW9hw2DY9AaJ149sk4yT3sEiis\/\/bMsxNNaODY9dIz3oRY56j0hUT1SHptezPh\/hA5GglTiHqCIb1\/\/2b\/+WTjnllHTiiScmiyDz3etjAIRniFwnOP6GywH35NuABC2KWHkuOGpsv5JWbLm0bakzst9++1WHUFiQMRqzev385z8\/geG2hRmkZz4wnk9DcgeyxP1CugORoIWPv\/zlL9VpK37Lwq9r6RIvpMIj55UlgAh16ws2HOgnE4q9ir0GbQMIDpBf09wjcswezQW9HIgE11xzzeRoHt\/lGgZjeMZ43\/vel3yvOOxWmAUtccmsWKBYYKosoBeY9woXWrmBSNBWlw022CDZFH3GGWdU7tprr53MBThSx88sXnbZZcku\/oUuSMmvWKBYoFhgGAsMRIJ5BiYxncprZc7PYr72ta9NFk30Bn1DbOHEAgq\/PF65XmgLlPyKBYoFullgaBLME7XheIcddpjbWW7V2PfEFlAspFhEyeXLdbFAsUCxwLRYYCwkGIUxXLZB2Sm0viG2bcZw2YJKyBS3WKBYoFhgmiwwVhLMC2a7jG0z5gvLVxu5Zcr1PFugJF8sMJAF5o0EB9JiSoWdY7jZZpslJ9lMqYpFrS4WUH\/2pNlAD7417yJegqbUAr5XtuFcHYK1hnGqWkiwxZqIz2Gxv\/jFL1okive0W8DBG04zsoHeAt7BBx+cxv0ATbsNFoN+ftHStJp6tBnbNJsX3LjKtqhJEJHpyTUZzMPgrRJwH0Ylbxjvk6A73\/nO4V3cCVlg2Hq0lcs37dRed911kzP2XC82zEp5hq1Hdaguo5wWYh2cEvejuouWBBGZAxyuvfbaVWyE8N75zncmvYN4u7hXSYTNYTL6He94R7cFE7TAKPWYq20zvyPYnMYqcZYAABAASURBVLWX+5frhbHAqPUYQ2I7ThyYYoveuDRflCSI5HzJ4izDuqEY87DDDkuGuvH9se+M3ev91eXL\/eQsMK569ADawuXEIy+4yZVoaeY8jnpEer5LdyqP4bA0x2XNRUeCjONt4UBXqBsqfp3OuYd5mHvHhHtgcv9yPRkLjKse9e79xMOhhx7a9VShyZRy8ec6rnoMSxkG3\/ve905Ongm\/Ud1FR4J6dYa45hGajONcNPsWnUhdD3fumfC6f7lfeAuMox4RoB8AO\/roo1PpAS58HcpxHPVoVR+kZ1rDGYdPfOIT3Y4Fi44Ex2KVksjMW8CPEfl9GofrmgdsWgCb+UIukQL4lUmf5qpDp83svPPOY+3VFxLs0pDMGTI+t4tYCZpCC\/j9EXNIRgU59EymUN2iUhcLxHxg1OO463D6SbCLcUpQsUCxQLHAqBZYciRoLrDT6aSmuT\/nmgkf1agl\/vxbQD11OqUe59\/S85vDNNTjkiPBWF3ya3R59bp3+EOZQM+tMr3XpR6nt24G0Wwa6nHJkaD5BZsth\/11ukEqeDjZEqsfC5R67MdK0y8zDfW45EhQszCx6htE+wmtOHF7\/TqdeAXTZYFSj9NVH8NqM+l6XNQkaK+g30xtGuIyfKw2cd0PW4kl3vxaoNTj\/Np3oVKf1npc1CS4UJVb8hnJAiVyscBELVBIcKLmL5kXCxQLTNoChQQnXQMl\/2KBYoGJWqCQ4ETNXzJfihYoZZ4uCxQSnK76KNoUCxQLLLAFCgkusMFLdsUCxQLTZYFCgtNVH0WbYoHFZ4EpL1EhwSmvoKJesUCxwPxaoJDg\/Nq3pF4sUCww5RboSYI33XRTuuKKK9LZZ5+djjvuuMp1z3\/Ky1bUKxYoFigW6GmBRhJEcH5v47WvfW31M4WPf\/zj0\/Oe97z0lre8pXLd+\/lC92eddVZyXP1cTuWiWKBYoFhghiywCglecMEFacstt0wvetGL0o9\/\/OP0\/Oc\/P+23337JL7Gdcsoplev+2c9+djrzzDOrcARZyHCGar2oWixQLDBngTkSvPHGG9MHPvCBigAf85jHJIR36qmnpj333DNtvvnmaaONNkp+9Jjr3u\/ynnbaacnx837D4ZWvfGX64Ac\/OJdwuSgWKBYoFpgFC8yR4NVXX53ufOc7J6eu7LjjjhXh9SpAp9NJ97nPfdJuu+1WkaH4veIsvvBSomKBYoFZtsAcCd797ndPjrq5y13uMlR5xBN\/qMglUrFAsUCxwIQsMEeCE8q\/ZFssUCxQLDBRCzSS4G9\/+9v0lKc8JTl1uRf8HOXrXve6ZEFloiUpmS+UBUo+xQKLygKNJLjGGmsk84IPechD0m1ve9vk1OVdd9017boCm266afK7AGuuuWa1Xeaf\/umf0ve+9730spe9rNpDuKisUwpTLFAssOgt0EiCq6++errooovSX\/7yl3T88ccnv8dh9RcOPPDAyg8JPuhBD0of\/\/jHq0WR+9\/\/\/unwww9f9AYrBSwWKBZYXBZoJMGrrroqffnLX672AK677rqrlNhWGdtkvvjFL6bf\/\/73yaLI8uXL07nnnruKbPEoFph1CxT9F7cFGknwr3\/9a7r++uvTbW5zm9bSCyNDltC97nWvFNfuC4oFigWKBWbBAo0kaL\/feuutlz7zmc+kX\/3qV6uUg58wMmR9Nufb4rXWWmsV2eJRLFAsUCwwzRZoJMHb3e526c1vfnOygfq5z31uet\/73lcNjw2RXfOzgvyGN7yhKttOO+2U\/vM\/\/zMZElce5U+xQLHA7FpgiWneSIJssP7661fE9oAHPCB96lOfSm9729sq+IbYPOHnPve59NjHPrYaNl9yySXJNplXvepVohYUCxQLFAvMjAVaSVAJHvjAB1ZDYgsetsHA+eefnxCgMDJ3vetd06c\/\/en0nve8J93+9rfnVVAsUCxQLDAzFuhKgo7UcjrM\/vvvn2x\/sXVGyb75zW+ma6+91uVM4A9\/+EN66Utf2nXzt3By4yzQD3\/4w\/SoRz2qypfNRkk7T6tpA3tdf\/mRk7+4o+S90HE\/9rGPVTajf6BevvnSia3YTL5s2E8+pobi4wJxpdEU713veldVrmHKom2KRy\/pNKU\/rJ\/0Qm9llkc3kBk2L3WrHMozbBrjjrdaaknxuuuuq4a\/jtKyT\/CrX\/1q+vOf\/1yR3957711tn7FA0hJ95rz1cm0E16BnTvkVCs+6\/iuKkDwYHpCm04imuXw\/+MEP0qWXXqoIyXPjdKXqZgb+aO\/ODl22bFm65z3v2ZfGr3\/96xMy60s4E0K2TXWbiUzksrUnaKHDcVr\/\/u\/\/vlKB73a3u6Wdd965WjT5xCc+sagOVNWQNeiJ1MQYMqX\/F77whTGkNJkkEB205a588\/0Q+Qz0nHPOST\/72c+qL6XadMn9v\/a1r+W31ccDCH0lzym9+c1vflM9y+b373GPe\/StpSP0ZqWMvQrVSILXXHNNOuGEE9IWW2yRnvGMZ6TVVrtFrNPpVN8VO3T19NNPTzZW98pkmsIdAKuB5\/jSl76UfCpIz5\/+9KecORjaGCrkw4Om4YAGoRcTcq7to5xLKLuop2ko5Y2ciXS91DPP9Ucc973vfas4df150oM+oVtTft7sER4uP\/ED9TKSYxvlCZlw2Uh4QP7iR3iTG7qrC3USZfRi2nDDDasoei11W+lhRD7ctrxynULviBtxlEWYdMhXmXb5Q\/4b3\/hGJaFtuVAf4LoblENdyIutQxf3dJB2W3y6kQuIn8vmaYeMvPjncnqteq8WQHP\/eh1EXUQZjQL\/+Mc\/zkWp6xP2JCBPeccLmm2cQaq8eXjoySUvnvD5xi3sluV0ww03VF+CeDt0Op0s5JZL3xV7uGZvg\/QtZWi6yhuDhmU7kEaSyxoORAXyV1mG0irXPbjWEOpxNZZ6mno4T3va01K3Ri\/NYSB\/etAn4stvm222qYaf\/JSzqYfFTxgZBKbceTr8pf\/yl798Jd3FISs8IB4bsVX4tbnSPPLII+eCfat+1FFHVb2zk08+OUWPRVoelni4IoK8nvjEJ3bVSR7qoR430ujXDRLxEnrTm96UgqzrvcNe6bF1rkvop73U45Kr21f8kGWXF7zgBXND9Iiv3vkLDz8vHoTHXuHXj+ursTvc4Q6V6Cj1TRc60a1K7OY\/7vkLv9lr3pxGErz1rW+d7njHO6Yf\/ehHjRnbHO2NbGXYnsJGoSn11IC8aXJ4GDQ6DRiojpAOPvhgl9VBEfEmtArOUzrR6FyrNP7RS\/MgejD4BVTo7rvvXt16o0ozejnyzx\/8Sqjljwcg15\/OkX9O4hFduHzkF\/qfd9556ec\/\/3mi09FHH12Jhu5kxeEZwx6y4vALuShjrntuN3nJM+ToyFbSaAKbhM3IRRkRHT3rcTz40vQQR88x8qKTuWvkLW6UUbmUj170q6c5yL202UccHQZbx5785Ce7TZ4P+VY3ff6p21W0ww47bO5l5T7QJBvEy3bskpeVvLj8ld81\/S+77LLUNB\/Ifp6LqINwpS3uq1\/96uoglX7q20vLy0v9iht6OZ1eenQKP\/XSpKt484VGEvQtMKU++9nPrnIyjGP4+ftiZIMNNkh3utOd5ku3BU83KlbG+Rs+f1j0etiGjEanIcWDwB+Eqfj3vve9Lueg8alwD23sqdTLkS+hYR4c8QLypl\/chyt9+bj3xpe\/Rm4+iJ4aqMbntCDEbqiCTMg34Z3vfGfVy8rjatBkc7tFoyf3kpe8RHDX+TJyBx100NzURBVhxR82U7Z8iIhg2GtFcHrjG9+YzOW5lkbYHWkj77C78NwWbCVd\/sNA2vIQd5NNNuGksC+dPeCVZx9\/2Ir9iTaVgX+AzuCeLAJ2jdC0R4cbq0+9Z\/5GAl6crnOE\/uJLJw\/rdi1vIDNKfYvfr65k5wuNJNjpdNLb3\/72ZMirS7rtttsmBlZR\/\/iP\/5h22GGHKsyv0XU6nfnSbcHT1VB07WVsmMDNu\/3ukcnaa6\/tsrKJKYHqZsUf\/sJXXFb\/rbYhnOpmxZ9IEwHlb1n5rgiuJqgRk+tBoW40+jx\/aayxxho9V\/0QX7zpQxdxcyAZQ3Z+uf75tICwKCMS8KBEunptwutzSfxyyMfCBBKOXmGEy9eXTAiQnXzRJKze+w27kydHBtSFMNfAVurM9TDQc5eHuOymrOo1\/LwckZLwXui3DNKhM91dN0GeiI8+3V5obEPXet5NaeZ+6iamNkat7351zfMf93UjCcpEg3FMlreqwxFudatbJUbzxthuu+2qX51TGWRnCcjCW7IO\/sph2OQhcz2tMFyo6x89sUF1NpzRs4t40tZzQmDhF6486mF6Ox62eHmE7KiudhY9VGWN3jhypcOo6Y8aXxuJnmhbWnqJeltt4fPlv+uuu1ZnfEpfPbKXenWfw0jGfRMJev5jioH9A\/yEqQd1L\/4o6FfXUfLoFbeVBEX0BYgzBI877rjk5zcvvPDCdOKJJyZvvcU0DFbWOqJh1Hsu3lx6xeS9BMydugb+wl2Dl4Y3rWuINDUijSkaVrh6QHpCZBcKoaNelzd8DMna8tcD0eMMnesvjyhjpBdy4SI2BFdPH6mY+0OoejG5HclGuq71PvTozGXFPTcQZWJncuGvLoTFvTzUWdwP4iIWRNAtjvz0FrvJRJgyxTWXnuLXyyCsG4xMokxeHOpKndXjRNnVk95iPbzf+6gX6Wg\/Uc\/httW39EMH1910FT6fmCNBX4D4yU1D3X5BXrz5VHDcaXt7edDq4C+vGP7mczsxlBPuE0GV7XqTFfNAGlhMhvMXLsxDHYsg7kFj01g07vzhMKSkT9PDL95CwAPtwZaXcoDrQD5kdh3+8RAYmnpwc7uFTTV2ZVNGZY24uYsYzU3xkzdbkQ948QpDCvLI5S1g6dEKZ\/ew+8Me9rC07rrrJmmxu\/B8oUFdyYv\/oIhelHSlEQ99uPFy0FukU6\/02SrsSr5ehl7xm8JjOM7+yp3L2N7iBR\/tPQ9zrY0a2of9w+UnjIy6VxfqRPtRBv7y61Xf5HJ00zWXm4\/rORL0NYivQix49Avy4s2HYpNKMybO9chMuNND5UYjCEL05opekwbvYSArnKxhiIbBL+DBjQWCPE3XZCJv1wuFnCAQDd259fyVB\/gLJwfKy898IZuBa37CyMgDUXhYYkFIeB1sGnash8W9tOXhPuQ9lB5OedGR3aVzwAEHVCuYud3pQR+yyPM+97mPpAYCwo29gYhb+vUEvCD50SXq1303hF2jDGQHbRNGJvlLWVlBuaUX8MKjG1kv8vDv16UjqAt1Il6v+kaa5OhCJ0Nh+ed+\/IXzWyjMkaCK1HWNN9n3v\/\/9tNFGGyXHZZ155pnVHi1hjPfud7+7WsEzNyjeQik7n\/l4QA1Rg9jkZeWKnzD3AfMrwuKeDUwZaBTh5yEkV48rXj1NMvzyvCOd+XbpXl+RRS50l3fMa3lQDK2E8c\/Bz3xh+LmO+OHHHojDQxN+dZeaRFlMAAAQAElEQVQu2qD06mFhI2lHWMh7CYUfVz2oD+Hugd1znUKfxz3ucYIHQqyIihRk5zqHh1ke\/KKX47oNygwRHuUdpk0oa24TepjGYhfp68XG8DuIiX+\/kLa2oE2Io05y2\/KTZ72+xeMvHAzbrdDzdw\/C67ryn0\/MkWCeiQ3Qe+yxRzWUUDG2zES4gjOyr0kcsaXrG2HT6tJZpSHxNrTNx3loheXxmhpmPQ8PM7mI6zrsU0+TDL8Ib3KFk6NHnlaTLD8yZMURlx+45ieMTN2Pv\/oV5pqsOORAGP8c\/ITliPghxx45KeWy9WvpRbxw63rkcTyEIcdV1+ojl3Gd69Smj7LKSzrkxasj169NRlnlIZ3QJ\/SM+27p0oEuIaM84klPOuHP3WeffapOinByuR95epgWEO5e\/ChDXX\/3ZLpBfHnkqMeTJxvkMu75R9r0oa\/0wk94Xdc8jfm4biRBvxty9tlnV+cFdjqrboHpdDrp4Q9\/eHXqtLmF+VCspFksMC4LGL7aY2gIbK4qXtzm3szZycewzAPpumBpWaCRBH0F4muQU089tfF3Q2yYxthkyC4tk5XSzpoF9CwsktDbfJOhKkI0PDQvZuhpgl94wdKzQCMJ+t2QZz3rWenzn\/98dWLMxRdfnJwtCK532WWX6oCFLbfcMpFdemYrJZ4lC+jhGXrlc0+hvzmo+txVhBV3Wi0wXr0aSVAWW221VXrNa15TnSL9z\/\/8z8mPKoFrJ0u\/9a1vTS984QuJFhQLzIQF8rmnfA7KXNUkCiBfIyq6mKObhA4lz5RaSdBPavpc7jvf+U7SeKzigGtHaPl8icwsG9EqmbnPOn7961+nHLNcxqJ7sUCxQHcLtJJgRLv73e9enaLiV+bAkCJfLQ65WXRt9vaNdB328uXYeOONUx15uOs8DekGjjjiiHREBsQLQbxnnHFGAnNVBd+rPvcqdlh6dvjlL385MQqZI0ErZT6R0\/NzVNYgGpEXz\/B5kHj9yfYnZbe9ye6A+24xRzV63lN0HaTGRXIBX4bkCHIM0rTnErbffvsUcA8h85a3vKU6KcXmbbCp1iZa9WW1s+Cl1W\/IFDvMrh2091GfyW7Pe7ewORLU43NijLm+zTffPPkaJLYStCVgp\/4xxxxTHUMe8dpk59Mf4TkEwIZj8ys2brq3NaItXwa\/8sorkyG+YT\/4mqEJNsTmWH\/99dP6GRwwEWjLbxB\/5zmCH7YCE\/vLli2rzn1btsJdc801q5NhfPJ0v\/vdL4Fr8K2sxSongiuLcvmhLAsD3fChD32o+k2ZbjIR1o9sL5m28EH8\/Qwsu4oTuk3KpQN9xpX\/sOkNEq8f2V4ybeGD+LObjzM8k+pzoTFHgp1OJ2222Wbpf\/7nf5Ljst7xjndUv5T2pCc9KemJ5N8T+4rETnt7r3bbbbf01Kc+tYon\/kIXAFH7LtKnO7G51MZN93pg3fQR19aJILitViwGNQGR5EAqOZw8E\/jWt76VAuHHzeUjLSQVCB2CXJFqN93rYUgTkOayZcuSAy9OOumk6idTfT6ml\/nhD384OQtSGJBdZ511qtOQ\/aCWxmjbSC\/0I9tLpi18EH+fyrHfox\/96KoMvfSez\/A2vYfNc9j0BonXj2wvmbbwQfwf\/\/jH15vzgt7PkWDk6nQYW2DMU2FzK1i2EOTfExv62lpgkeS73\/1udb6geJHGQro2a\/sQfJNN\/naoZeTt3kZYw\/zwW2gXkQWC3Lh0A4T74he\/ODk9BQEhx\/333z8hTMQZZMp1D8LIefhBOtKEXuWLYbuXAxhum88038ndaaed0iGHHLLKQbr1dH0v7ogobj0s7oV1k2kLH8SfbU0JcCPfSblteg+rz7DpDRKvH9leMm3hg\/oPa6dxxFuFBCNRQzCLID7+1mtwjNZZZ52VLrrooupUYXsIhZOLOJNwnVxiTtIwsJ5\/nGxS95\/Few86IDvEh0ABISJGQJbHH3988iH74YcfnoQhShAP2sqOIH0T61cGkWMQo+tY2DHf2Ra\/+BcLzKoFWkmwXiCHqppr6nQ69aCZvjcXYQO4N9ckwHhrrbUWp\/pd51F1kJBpAYS5fPnypKcJfm8DkCSCdO0XA+FBD3qQaKsAMSI+vUZAiHqMW2+9dbK\/zehAeJPOEutWrrbwQf39KNif\/vSnsdiuqRz9+rXp3W\/8utyw6Q0Srx\/ZXjJt4f36e\/aM5MhPCn2T4KQUnM98zWmaN\/OAx2ou1+eCOfSA69AzDlx++eVplhA9Z4snoMdoXlUPEsy1QRM5IkZgM0CMr3jFKxI4XRr0SAexx7Cy9LjiiiuqE8+HTaPEm2zb1YasDM\/nc94r7SVNgoaHwEhO7QgYFub4+te\/nuowTRD45Cc\/mQLhF+4JJ5xQfWLI1euEn\/zkJyngLXj++eena665JunVgBO99aKGgUURadjLOUh88XxD66sgXy+ABRU9R6cFIcoYVrNXjt\/97ncJrM4DIrWFx+IaclV20xa5PvJr0nMQf1MgdjXo9eZpT+K6Te9hdRk2vUHi9SPbS6YtvF9\/bcp8eN6eFvp6akjQSq19Xra71I1gccPiQewBzE8n9iB0Op2KpCKci9BUhPB6evN5rxw5PPyBID5EGHB2mkUo5GHlFvy2S+wndA38AaHEZmJztXpDYDhltRf5PfjBD66207jvF93iOXg05iFj7tEiTTdiZGNzsnrU9LY6\/cxnPjMZSoNtTHk5Qs82PZr8vSwc4BFxJ+k26TeKPsOmN0i8fmR7ybSF9+tvd8LUrQ5rvJPArrvuWn0tUM8bAdq\/aC7KHkAPjlVfwy6yfgDaw4AoEAkZD5h7B0Za3SbXBIs7tjB4E4G3Uh2GhTmQcQ49yYAKzYGEoSnvfv0QCZg7AXNw5uNAeY888sjqR69sf7FdiQv8DfVBHHH7zbMfOb2vYYgRYYNpBzoie0PqjTfeOKljcB+LMeToL04\/es2ETFFyqizQsydoQ\/Qpp5xS7X0zAe3AVZsanSgzjpIgOaRi+NiUXvhbiRaO1PwCnp6IuPyAXnpcrvuFHpvFHkQFOYHFdRBcuBYbcuQEWSdQxAq2HAXcB3J56bBDIPILPejXT7kQJiC9k08+uVrACLLcbbfdEoIEfggSyPaTdi+ZNmLUe4yyKldbOogOkB6ChCBJ5FgnyiiH3wpBmkGY4kunLZ\/iXyyQW6CVBJ0Z6Nh1G6c1YD01hOiHlXy29drXvja5zxMb9BqJ6eX50sFOe3NS9TQcA17\/HQfnwZFz1L99gsj5TW96U\/UreIbC9LOhW1x5kJ0WILMAgkMcFioMYZvIle0BcdaJlH9OngjG0LXbtiUECYgiSBLZIMh99903HXrooQkxCm8jR0PvbnsA2TpklJVetvMAQrRwYoXa3KEve5RDr5Lc3e52N9FbgdyAfkgPmdvWowxBmHqSQZo5cfITBmQBeQakB9IOyAtaFbo5IMrLvdlrJEc6vWzclMEg8fqR7SXTFj6of1NZFsqvlQS\/\/OUvpwMPPDAhO0fth0KIymd1Nkl7YMJ\/GFevzoOIAP1ATD0NPTW\/Q2BYWw9zj+T0\/pDI05\/+9OqIccNhsPLrYRdOdlph3sihn9x+dUQsgEQRx\/Lly9PyFUCIvt9GLAgTcQL\/6GGKI25TXuytl69OkAtiQY56XIgRECNde+ncS4YeXm7PfvazE3I0v4ggHdOGJPX03fNHkhBEqcxN+rf5IbFAkBuyA2UMIEVAkgHECcg0B78AWXOeH\/nIR6qetjQCQbDhyjNH6BNu6KntmuZhx7ZyNfmT71U3Ea8f2V4ybeEr+0eOKbX53yKx8FeNJOhh8E2wRueBQlahmuOzXve611X7zzwsVjUjrLjTZQFkB0gDSQIyQYxBkkGQZNZZZ53GAngg1TUgjCZibIw4gqceMp20QSQJCBExAqK0cq03qVfJTzgoI4gL0gFpwghqzUUNsuIGgXFzgnPNXjmCHMNFoDmCWMPNidd1+OduHt91pB1uEHDu0i0H3XMoV2Cu0Iv0opEEDTFt3WB036PWy97pdJJG5bdIbHOoh8\/SvVVavRvd90kAwVjlveqqq8ay4XeQ9LyVDcMtDukpWqVzEIapBPsH3betrssHKYKHvE6MeundyiV+U\/ig\/kYm5nWVA8mDzeGA7MHGcECWgDzBdUA4kA\/YSB5gn4D9kwHDd1i2bNmCNPsgptzNyct1Tm6u1U8dQZDhIs8cOcnigTrycFNazheI+JFmfIaZk+9XvvKVZA732GOPrT7PpK\/tVU08syAGXZFJIwl2Op1EKUOjFTKN\/zVyQ1hyjQIz4mmop4FMatOsxjzODb\/DppfHM8eK\/DzoyND3uRamTDmYZvDQN1UvAkOKYJ7O1hj7BD0EHkbbZWw6Z+s8P\/eBQfzbZCOtXq5plIDygvIFlD2gZxnQ2wzoiQICdd6m3zJWZuCfwy835jB1EdAjz7HpppumDTbYIPntk4D6yKEecjTVyXz5sX0AiZ177rlzpKauwV5b7cDzFYh5Z89dkCY\/6wLzpWuvdBtJ0B4fvyZnXkiPsJ6I3yHWwG2sXfXghLp0SrazWLAI2A9oyL2q5Mo+JvjXXnvthHBXDvnbnblCDbfT6VRfDfzN95a\/3s7Cb\/FZ9cohERr3sJtaR41nCGpujDtqWuJLZ5j0esVT1x5Kw0sPLhKwGOXB1UMSf1XrpqSezcvat+mhsOncxvLjjjsunXfeeUkv3P5JIwqQTpP+Tf7qdlo2Szfp5\/PFHPXelN\/oCdicnsPJ7XvttVfSmwoccMAB6YAMNrHn0LutIw+Pa1MHOdRlwLNQhzoPGAHmML0QaKr\/WfBrJEG9O28sBXAkjpNFDNfe\/\/73J43eb4usttpqybCJLLluULkWKwIWQhCcOEGQz3nOc6qvJrxB+QcQnb2BjvQKEt1mm22qYKvE9gl6ixjWRDjXvYcpn8+sItX+iKsBGxoWrF5NXPdrB70QD4QhqAcn5hldI0ZhXkQ1k1e3QY6mI8BLFQyl7PEE9+oeDJ2j5xH6WTiYls3SodO0uXYL1KFecgTBcc291hEEyc3J07UFrIB52hzhHy75gLRAW3H0nVFE1TAm8KeRBOmBHAxjEMnZZ59dvdGN433i5TdafTz\/0Ic+lOjQQICGD3qc\/\/3f\/530Kn2Pm38RIi89Cb1TJGpOwWbpddddNyE4ZOo8OavVO+64Y7VCrIfiuC\/DiKGVW6CI5iGH2QrRpt6w6Q0Sr5ss0vNSMc\/oJeVFGeToBYochXsw28rggYCLL7642udoeK2NxJDK\/CMgSvtIDbmEW70OaLPig7SgLb9x+HezyTDpD5veIPH6ke0l0xbO37SK9hA9RcRr7tYcrhdnkK7zSCe5wNpKgioOEdom86Mf\/aj6HQxvbG9kvSwkRGZY6Al8+9vfro6NN2SQTqfTqVadnV8Yp0KTMezRE9XDs6XCga6Iwx5A6Riee+j23HPPROajH\/1oesITnpAMv6Q7zyjJ92EBD4OHQOP3KiYKrwAADU5JREFU9n\/Zy16WzDUiSfdBkA65Vd\/keyWL2NS\/uWukhywDSDFI0\/wTIE5wHQgZ8hAkGq50AZnmkDf00rGET78FupKgDdO\/+MUvkq9D7nrXuyaNwGSmnpo5nlGKpwdnWGyoLB1EKE2T794U\/MB8IIIzqa4nCLrUwmKztHlLH+wLC7jXY0SUZKcVhk9eKNxx6CidYdIbJF4\/sr1kIlwvQc8wCNIcmTZhhTZ6kK4RpV5kQBy9Se1oULshr4A2DYgOgkTDRYwQZBlukChSBcN4c516pxHGDXmudHIE0eYuHUBno9PpJFMA9AuE3uHWyx525dbD6vdkerWVXjJt4YP613VbyPtWEtT7003VAK+\/\/vp0wQUXJJO1hqOGILYO+Jxu3MrqvXU6neo3NLzl+90srfdQ10VDMZSu+5f72bGAHiEgPb3IAFLUPrUR13XC5Be9yyBORAvSAunCOK2hzeUI8uIitxxBtLmbkyTirAOx5kDAOfKwuK6nkefhOifhuM71dE3\/HHkZx2m\/SaTVSIK+wzVfZ0xv35i3rR9ecm+SU49NY2JcK3rjUlyvTfrmIc33jSvdbukY4pt4nxT0cDXifBFgFF2GTW+QeP3I9pJpCx\/E3zSINuhBZjO9J9B7AvNSeiQBL0qIbSb2QUJsg+EadQTMX4HeaiDSClceAXkGkHOObm1wnGE5OcV1Tl6ukVqOnITjmk1zsHOOIFiu9htwH7D1xVdlES\/Ss0\/QaVFBuE6rtwA6TjsMklYjCfo+2AKFt6llfeR3+umnJ9tmzNnYoOqHjMiMa0JTg4lV31iZHqQgg8oaSnkAVJgtO5OCeTGNxGdX49Bh2PQGidePbC+ZtvBB\/OMwznHWoZ80DagTMNQNGO7m8PIKOOYs4ISiHOo4Rx7mOuKFa0SUIyetuDZVlCMIOHc9V4FBn5Fh5IN4ufI1X4t4IfRGesoWhKsMFjE9k8PkOWqcRhLUEzQEDqVsOqWowxT0CmXqTcgdBxjLoQfm9nw7uhC9QGWzR9C8ZMFRqdhgYW3guLcc++23X8ph43UO20nqsAsih05LHba4BfR064iw3M3TMKWQI6YWwjUiDJhiyGGqIdAPT3gm+5Ebt0wjCcbeO8xtR73tJ3p8hgkUsGBi+GFzLll+3WBobdU2oMeD+MThthEgwp3PzdKMrkwFG078Jyunug42XLz2MbqrIyeyILhwY0423Jwgc\/J0bT0hYM42EH7hkgV8MAk0kiDyMQzWO9A47VxnKJ8TGSr7TtAcIUOQ7aW41b5YteVKV7xuBBhpmiuor\/JaFRZuHxoStpXHZzr8Au4Xcm4x8i1usUCxQHcLRO8w3CDd7rHmL7SRBGVn17jPdWxXcXSWY4LMBfpSxByh3tuo7O2Mwl5DYN+s0sc3mFyLJ7vvvnt1CrFhMzK1HcaKtclWMlx7DUfVT1oFxQLFAovbAq0k6Mgsk9S+9fS5nN4WUyAdE7zmJ3y2xG8Y2AyNqC699NJqKBZD5XARmXQRnXlCvUFheqZ6eHqXwsFnN+ZXEDMZrpUpew+FFxQLFAsMYoGlJdtKgmEGw1+9rzr04HysLTxkB3ERlK02hsdNQGyRHiK0khRy++yzTwTNueQjnOt+LrBcFAsUCxQLtFiglQSRnGGwCVG9rzo22mijpIdoj1RL2sW7WKBYoFhg6i3QSIK2yJiDc9TRFltskSyM3Pe+901IcbvttksWK2widcCCo4ymvpRFwWKBYoFigRYLrJbSqiGGuDZC+8jd0NMnck5qsShivs3mUEcY2djZ6XRWTWCR+JgC2GyzzZL5y0VSpCVVDPVnP5t5YrBVa0kZYJEU1i4S2+rUIcR6wbiK19oTtFl6\/fXXT51Op8IDH\/jA5DteCjnWynebTnhxPy5lpikdxOerGAdITJNeRZf+LWA04xh4c8Q+2XJs27gfoP61KZLDWsBndvYLq0cLoHaHeMENm149XiMJOijV0fk+v4kIhsAOI7A9hp+NxuYN\/R6J+2kEItOTazKYh8FbJeA+ykDeCrjPo2wRCv\/iTsYCw9ajUUzsInBaik8+J1OCkisLDFuP6lBdSgPsVLE\/2PU40EiCDjd1nLotLH7\/QkY+PEeAhsm+IvE1CbLsdDqCpw6IzKk3hvY15RLC8yt6egfxdnGvkshajWZ0LwL3BZOzwCj1mGvt\/Mnrrrsu2WCf+5frhbHAqPVoxGlIbDrOvmBb9caleSMJIrett946GQrKGBF6k66zzjrVkfoOLPV7BQ43NTQelzLjSgfJWc22B7GeJmM6wt9Q1zYd4bbTuNf7c18wHRYYVz16AHfYYYdqgc8LbjpKt3S0GEc9Ij1fmvlc13BYmuOyYCMJStx8oAUQZKhnSAlzLD6dc8rzs571rPSOd7yj+lU68tMCxvG2cBIN1PUyfDeMd7R3HubehmwPTO5fridjgXHVo969H4ZypFO89CZToqWZ67jqMaxnGGw47ECX8BvVbSVBCZsv0xO0EuzePKBJSr8O5tj9adweo1dniGsegc51mNc0nDe8r4c5\/kd43X\/W72dR\/3HUIwI86KCDkjMqSw9wMq1gHPVoVR+UwLSG47kcveV+HOhKgjK0GmMo0QRnrDXNuY1DsZJGscAoFjB\/vffeeyfz2uYBvdBBz2SUdEvchbfAy1\/+8mQnivpzas3OO++cxtmrbyVBn8Q985nPTL4K8ctyTXCSzGL+YoShGZ+78FVfchzFAr5rN4dkVJBDz2SUdEvchbeAqbi8Lsddh40kaPHAwoext5\/CdBJs3pDi2ve8ZZix8I2i5NjDAiW4WGAACzSSYCwemA90jmCnM53bYAYo55youcBOp5Oa5v6cbyZ8TrhcTK0F1FOnU+pxaiuoT8WmoR4bSdBCiJ\/YdIJ0n2WZGbFYXXLoaq60e0d0lZ5tbpXpvS71OL11M4hm01CPjSToKwlbYI499thkO8kghZp2WfMLNluWQ1invaa661fqMbfP7F5PQz3OkaBVXqu9sQpsG8yVV16ZLI44Tj\/8c5e8eLNWBSZWrXrbT2jFiVsOYZ21Wkyp1OPs1VmTxpOuxzkStMprtTdWgfWUkKBFkuOOOy6Ff+6SF6+pYNPgZ69g2+INw8cCD9f9NOhcdFjVAqUeV7XJLPpMaz3OkaC5MISBEPoFefFmsUKKzsUCxQIza4GxKj5HgnmqN9xwQ\/L52CwOdfNylOtigWKBYoFeFliJBK0Gx\/fBDiDw\/bCTpX050iuhEl4sUCxQLDCLFliJBP\/rv\/4rOUPP3p0XvvCFycGp5557bnrjG99YHag6iwUsOhcLFAsUC3SzwBwJOknacfm+s7Q1xudyn\/jEJ5IVYHOE559\/frd05jOspF0sUCxQLDBvFliJBC+55JJ0\/\/vfP+VnBD7mMY9JTosZ59E181aaknCxQLFAscCAFpgjwbZ4q6++enVmYCHBNgsV\/2KBYoFZtkBPEpzlws2q7kXvYoFigYWzQCHBhbN1yamLBRx068Be32\/7isfPvMYWLT\/z4Mslh6SC083jkM16kjb3O\/gDXNfDy32xQN0ChQTrFin3E7HA5ZdfXu1M8NOK++23X3r729+efFxPmXPOOSeZlvn7v\/97twXFAmO1wCok6HM5b+KA\/YJ+sKjuL9wPW9tUPVaNSmJLzwIrSuwTTT0\/Pb7NN988aXd+8EsP0cG29qw62GOFaPlfLDBWC8yRoDetk2Ne\/OIXp35BXryxalQSW3IWMLR1bLqfxLRZ3ws2jsH3o17nnXdeGvY3JaQtvSYYVhteLzmDlwKvZIE5EvSLcttuu231s4R77bVXXy558VZKsdwUCwxoAZvy\/TKgF6peoB\/xetjDHlal4lRzn3E+8IEPrO4H\/SNt6eXYcccd0xprrJGQoJ+RHTTNIr+4LDBHgourWKU0s2SB9dZbr+rp3eY2t0nIbtNNN01rrbVWVYQzzzwz+c1rXzFVHjf\/iR5jvYdns7\/fpr1ZLElbeoEnP\/nJKQ7+8ENMyDBkF84tOU2TBQoJTlNtFF1WsoAe4BlnnJFs2HfaeR6oF9c0bfP85z+\/2tyfy8a1b+MR39lnn5322GOP5Dd0Iqy4S9cChQSXbt1Pfcltjbn44ourXmJd2Wc84xmNUza77LJL9dVTXd4Cy+GHH56OOeaY9OY3vzk97nGPq4uU+yVqgUKCS7TiZ6HYFkQMV8exNcY2G9\/Cb7nllmnrrbdOnc7i+fGwGajLqVaxkOBUV8\/SVU7P7bTTTkvj2Brjd3Le\/e53Vz3E7bffPpl7XLqWLSWvW6CQYN0i5X4qLDDq1pgohG03iO\/qq69O2223XbUqHGHFLRZggUKCrFAwdRYYdWuMAt10003VUXCnnnpqdTamHqHfy8lx4YUXEi1YwhaYDxJcwuYsRR+XBdq2xgySvh8BcxamOA4M9quJdZx44omCC5awBQoJLuHKn6aiP+IRj0gWL\/wiGb24hxxySKpvjanLkc3hd2yPOuqoBPYWchFhG+STxy\/XS88ChQSXXp2XEhcLFAtkFigkmBlj2MsSr1igWGB2LVBIcHbrrmheLFAsMAYL\/D8AAAD\/\/0lOda0AAAAGSURBVAMA0Bl3tsG9s4AAAAAASUVORK5CYII=","height":193,"width":321}}
%---
%[output:58b60e8b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUEAAADBCAYAAABRyQJdAAAQAElEQVR4AeydDbRVxZXndzStDyQKCAoIzHtRBlyIBhGjkmCMOLJGaY0RlzAEDVkBQW2NgLQoBrExiqiYQaO4ZA2GBTGuKIgYIq6xMRix0eBHG2REYYDwITbfLSjS9vsV2a+Lwz333fveuffUuW+z2K\/q7NpVp+pf5+y762ufI76yf4aAIWAINGEEjhD7ZwgYAoZAE0bAlGAT7nxruiFgCIg0OSVonW4IGAKGgI+AKUEfDYsbAoZAk0PAlGCT63JrsCFgCPgImBL00ajEuLXJEDAE8iJgSjAvPJZoCBgClY6AKcFK72FrnyFgCORFwJRgXngsMXsIWI0NgeIQMCVYHF4mbQgYAhWGgCnBCutQa44hYAgUh4ApweLwMmlDIDQErD6NRMCUYCMBbMrZ33vvPTnjjDPkm9\/8Zh0NHjxY9uzZUxAsyCH\/8ssvFySPnH8v4mPGjHF5iy3LZbI\/hkAtAqYEa0Gw\/w1H4LjjjpP58+fLxx9\/LO+++64raNmyZS4sxZ9OnToJ5ev9Nm7cKCjCFi1ayJw5c6Rfv36luK2VWcEImBKs4M5Nu2m+5Yai0voQx4obNGiQ7N69W9ny2GOP1VmUyNQlxERQfLfddpusWrVK1q5dK2pVqlXIPSD48CDiyhswYIBgzXJf+Oeff76rg19vZLn+5JNPBPkHHnjA1RELGD55kKGMmGoaO2EEki7OlGDSiDax8nbu3CmXXXaZUwynn3669O3b11ljKJdJkyY5KxELEYsNRYHiWL58ubPm7rnnHlm3bp1DDP6rr77qrEmVh+cS8\/w58cQTXSpKykVq\/2zdulUmT57srFOsRhTtmjVrZPbs2bWp4u4xbNiwunvD\/Otf\/yrPPPOMXHfddVy6vFibt956qyxatMjxdu3aJZs2bXJpF110kdx9990uDzLUHSXrBO1PphAwJZip7gqvstHhMMoAZbdlyxbp2LGj1NTUCBYbynH16tVOofTu3VtOOOEEl9a9e3fXKNJQWChSiLgqHydQxB\/u2bZtW2cZnnPOOU7Z7d27V6gb9aA+3KNz5851pZ500knSvHlzd82QGgWMhTdlyhTH0z\/9+\/d30VNOOUW0HcQd0\/5kEgFTgpnstjArjXJByaDQGlLDK664wllZWGDQ1KlT6y0GZYsQSpUQQoGh5LD2UKa+siM9H2HNMTSeOXOmsxix8vLJW1r2EQheCWYf4qbTAhQI1haWEcPUDRs2CMNQn48lxXCY4Stp77\/\/vgOIPIsXL3ZzdMijiLAoXWLMH+R+8YtfSNeuXaVNmzaHSGEBQgytGXI3a9bMDdWpH\/mUf0gm7wIFyiXyhEaVi4Apwcrt27K0LDonyE2HDBkiPXr0kDvvvNPNF2KVdejQwc23MdRkGImCGj9+vKiVBn\/kyJGHyVOeT+vXrxfyMlTVcqMWI3zm+AiZ52PIjcVIvSgLPpae3hueklqzw4cPlz59+kivXr2E+czPPvtMRSysMARMCVZYh5azOSi6d95555AhLNtUUCTUA8XGsBbyFRVxeAsWLBAIOeSvq12UgA8hA88n5EjzSeW4J\/dGhqHxkiVLXL0ef\/zxuq0zKkN+LD0WOrBYuS95Sed+XCND20aPHu3yV1dXC2VSvsroveH5+Uk3yg4CpgRD6yurT8kQYHiNBQlh6U2YMMEt0JTshlZwJhAwJZiJbrJKJoGAWnhYeRAWXBLlWhnZRsCUYLb7z2pvCBgCjUTAlGAjAbTsjUXA8hsC6SJgSjBd\/O3uhoAhkDICpgRT7gC7vSFgCKSLgCnBdPG3uzc9BKzFgSFgSjCwDrHqGAKGQHkRMCVYXrztboaAIRAYAqYEA+sQq44hUGkIhN4eU4Kh95DVzxAwBEqKQJBK0D\/exBEnCF5JkbDCDQFDoEkiEJQSxA8cCo+e4FiTT\/BIQ4a4kSFgCBgCSSCQuBJsaKXwL4fXDhQfZzyj5cAjDZloml0bAoaAIdBQBIJRgrg\/wm1RfQ0pRKa+MizdEDAEDAFFIBglSIX4OA9f8WLYyxwgRBweacgYGQKGgCGQJALBKEFcnuMqnU8a8l2IuXPnunYyBMbjMGnIOGZIf6wuhoAhkGkEglGC6r4c1+cMjXHBzncnQJcP8BCqDHEjQ8AQMASSQCAYJZhEY6wMQ8AQMASKRcCUYLGINXl5A8AQqCwEglKCfCFMvyT27LPPCt+BYGEEHmmVBb21xhAwBEJAIBglyDwgX\/NiISQXkYZMCKBZHQwBQ6ByEAhGCbJZ+vzzzxcsv1xEGjKVA721JCMIWDUrHIFglCBWHtYeVuCtt94qrAgTh7geNGhQgz+PyFE7VazsPazwPrXmGQKGQBEIBKMEtc5Ye6+++qqg+JSHQoRHmvIKDcnz8MMPy\/z58x0tXLhQ4BWa3+QMAUOgshEITgk2b97cIc7CiIvU\/pkyZUrtXxFNcxcF\/nn33Xdl165dcuKJJ0pNTY184xvfEHgFZjcxQ6DJIdDUGhycEmzRooVMmzZNODGiQ9jly5c7HmkN6aCTTjrpEAW6evXqumI+\/fRTWbVqVV7iBAvky3EN+by4OHKQn8415PPi4shBfjrXkM+LiyMH+elcQz4vLo4c5KdzDfm8uDhykJ\/ONeTz4uLIQX4615DPyxVHBvLTuFby+bniueSUR5grj89DBsrFi\/J9GY0jA+k1IddKXOejXHLKI8yXlzRkIOJKXCspLy7MJac8Qs3He1j3UpY5EpwSpP3+\/CBzgswVwiMtSdqwYYM8+OCDgqWZjwYPHiyQL8M15PPi4shBfjrXkM+LiyMH+elcQz4vLo4c5KdzDfm8uDhykJ\/ONeTz4uLIQX4615DPi4sjB\/npXEM+L1ccGchP41rJ5+eK55JTHmGuPD4PGSgXL8r3ZTSODKTXhFwrcZ2Pcsm9+OKLcscddxz2TD\/00EPy5JNPCvPmWmau\/MojVLm4EBnIT+daSfm8h7yPSb7fhZYVjBJkno5zw\/VVvBCZaBnsMfSP3OlxPECfN2+eexiYg4yjOXPmCOSncw35vFzxvn37uurcf\/\/9bp5TZcgL6XW+EDnIl+Ea8nlxceQgP51ryOfFxZGD\/HSuIZ8XjfOg037kID+da8jnxcWRg\/x0riGflyuODOSnca3k83PFc8kpjzBXHni0e8WKFUK\/R+W4VkI2H+WSUx5hvrykIQMRV+KBvOmmmw57pumvk08+WdiJobLkhfSakGslrqNEObQffi455REigyzvIe8jdSs3HVHuG8bdD0sPbzEMgfklisrBIw2ZaFq+a84iH3vssbJlyxZZs2aN7N69W+Bpnn379skxxxwjXbt2jSU2a0O+DNeQz8sV54G65pprpFevXoeUT14oV54oDznI53MN+by4OHKQn8415PPi4shBfjrXkM+Lxi+88EKh7chBfjrXkM+LiyMH+elcQz4vVxwZyE\/jWsnn54rnklMeYa488Oj3yy67zPV7VI5rJWTzUS455RHmy0saMhBxJZ79jh07SpRPOlNOEHEIGYi4EtdKyvND7Xd4ueSUR4gMxHsoKf0LxhKk\/f369ROGv8RReD7BIw0Z4oUSypVfPR5IiDi8QvM3Vq5du3YydOhQIWxsWZY\/OwjQ30n3+9LVO2TAIyvkvj+syQ4QGahpUEpQ8VIv0ig9JXiaXmyI4tRyiBeb3+QbhgC\/7ljfhA0rwXIpAii+G36zUjq3rqpVgmtl7vLNmhRcSH9nqd+DVIIl7VUr3BDIGAIoPKzA50f1lEcGnSrbHrxA5vzLJrl+7sqMtSTM6poSDLNfKqJWVVVVbm8mYUU0KIVGYAEuXb3dKT+sQK0CylCVo\/JCCelv9uQShlKnfPUwJZgPHUszBFJCYN22fbXD3jVu2Dvu4ho3DPargkLEMvz7R1cIVqKfZvHiEAhSCbJdhtU1FkbmzZsnI0aMEPvGSHEd+1\/S6cWyNjeUHlKH3hkFiHLD0kPRofAOlTh49Z1TWsq4i6uFucKDnML+Tpo0SXr27FmYcAOkstbvwSlBviNy8803Cw4TODPMcv3AgQPFvjHSgKcx5SwMh2pqaoQw5aoEf3sUHxYd83zf+qfXZVDvdvL2HeceZgFGGzKu1krs1KrKrRpH09K6pr+z1O\/BKUHd1IwC1E7UfX2apvxoqHsJsSCxJLEokakELzK8JEb7JCsYbNz1pUDUF+UWJaw8CKXHthcUH9YfzyvWH8qNeCG04PqDVh3lFCJ\/5513Chu5C5FtCjLBKUF1klCsAwWGy3iI4Twi22H4UBMW5dq1ayUrXmR4YaIvRutbXhGIl8TodSkAgyBkzrp3uQyYtUHOmbpCUG5RQvmx6PHaRzuctcdCB5YfIcPcYpUP+dZv3+fmEIvN29Tlg1OCDH+nTTvoQAFFiIv9Qhwo9OjRQxYsWFDnc7B\/\/\/6ub99\/\/\/16vci88cYbsrZWWTKXUQr6\/PPPZe\/evRJX9sef7HHbHXjBJy\/8UJas+lROaC7S66QqmXp5dR39dlg3yRL9+n\/VyP++pJXMGXpKpuqdBMa52r5sTE9R2njPuXXxB35QIz\/o0dL1edwzUh+f5+W3w051z9E\/f\/Bp7LNGObwYX3755WEy+\/fvlwMHDsgXX3xxWBr5CqUdO3bIBx98INu3b6+3nLW17x3HWqlTWhScEgQITnTgNAGLDiIOj7RCadGiRcKZxKOPPlryeZGhPCzFWbNmyaZNmxKnzZs3y9atW92xvWj5b61aLxPnrRSsBuZCJ\/ZrIwuu6ejoju+1FOiCTiJKJx+zT7JGnY\/aLdVV\/565eieBc7Ttf\/f5dlGKPgtJXFP2jCvayRUz3pMX3lwT+yz\/+Mc\/ljZt2hyWzvQRP9hMOzWmPvme+Wi5vHdjx47lNUyNglGCdADzeMzn5SLSkCkEKeYGkSv0lAmH3Dnj2r59e0ma8GPYtm1b4RiVX\/b+o1vJiOc2y6LV+4Q5oMd\/1EOGf79L4vf371nueHV1tTAtQVjue6d9v7h+L0W9\/DIvPatGnh3eQya+\/KnwjPlp9cWPP\/54wWjA52Z9svnS6e9C+533jqOsvK9pUTBKEEtvyZIl7uwwniVYGMEKhLhmtRgZBYo5QJwpoDDxWoElRdqYMWMIZOrUqS7kD+Y2v27EIfUiQxzCUqTjqqqq3EpmkmGzZs3cg+WX+fCSTXLVzJXyo3M7uhVA5oD8dIsn3w\/lxjRXv5erDt\/r1sY9Wzxjxd7zyCOPFKjYfA2V57379re\/zWuYGh2R2p1jboy1hyt9FJ+KoBDhkaY85gDfeecdpzRxycNcIgqwf+1coG8BsrJ8bB4vMlpeuUImw1n8wPorZgWwXPVL8j7MIWXpDGmSbU+7LJ6tuK0zpV4dzlq\/B6cEG7o6zDYYXUjBOoSwECkPcxsPMhBx36Is58PqK8C4DbDlrE+p74V1kKX9YqXGo9zl69YZVqLLee+S93vCjQlOCWLR6eowigwqZHW4X79+zipk+KykFqKfRjxhDAsqThUg2yCarqk99AAAEABJREFUggIsCBQTKjkCbJ1hGw6jj5LfLKM3CE4JgiOWms4PotCIwyMti\/T4Gztk2do9bv4vi\/W3OmcXAX5wp199cOsMFiEbqqfvuUB+vPDg2eTstiy5mgenBJn3YyUYC9AneKQl1\/TylMQiyAsf7JHnRvQozw0DukvW5oYCgi7RqqAI+5zc0m2kxiqkcDbm3\/eHtSU5bpe1fg9OCWLxYflhATr6+GP3bY4JEybUbYSmE7NADIF\/\/foGt+8vC\/VNuo5ZmxtKuv2hlIfig3LVB37SQ+Ws9XtwSjBXR7E6PHPmTMmSJYgC5OFiz1auNhnPECgXAjhgzXcvntV86ZWelgkl+O677wp7\/QrtDPYQclqEkDysHOvQWjdSwy8V8VChANkGw1CkVPcJvdysDYtCx7Oh9cPaa2jehuTLWr8HpwSx9pj\/U6VFyPnh6GbpuM5h0zRut3bu3OlEKI9jcfPnzxcIJwvwXGIJ\/jReAZagUlZkk0agKf8QF9LxwSnBXHOCzA36G6DzNWz27NnSpUsX6dy5sxPDity1a5dwjIk9axwJgucSE\/6D9Qc1dQtQYc3a3JDWu9JCNk7naxO+C\/OlF5uWtX4PTglipbHJmVDBJx7laZofMvzFT9q1117rs+t1oJCEF5lfv75e8ACDJw88ejAkUOJQej4vMipn4b56vY5kCaNQ+h0PNd\/+by0OeSf0osOxX5ebzm\/vcE\/Ki0wxfWReZLQnakOGsSg6PsiMT0BChsIQ8VoR4fQHYS4i\/\/Tp0+WGG25wH1PPJRPHY7g8a1bDvcjM+L8fCtsN8AKDJw\/fU0YxHjX8fJUQ5wFnozthnvYc5s2kEmRD6\/dH\/76NDD+7paD0lAac2sI5flWPMxgbKG7O2TemD+jvQvud9868yPxNM3FShBMeqgAJGQYrkYbM38TdN0d8Bwp\/+ctfhDwcjUNp4kdwyJAhsm7dOreoQsdq3qgDhcZ4kcFTx8w\/7xE2pOLBI+pdg2F4Li8yUblKvLa2H+49KM1+nnj5qfLmP\/auIzwXjb6wo\/M4Q72S8iJTTL+bFxnVSrUhv0JYgoSsBKPIsAKVWCwhrVbU\/Y86UDj77LNFHSqgDLt37y7MD1566aVSnwOFhnqR+eQzcd5gOAqH5w7mQqKUpjeRaF3Kfd2qVSvp1q2btGzZMnHvPOVuS7H3y0q\/335JF6lp20IGPrnS9REeZKBi2+vLF9Pv5kXGqbODf1gQwdo77bTTZMnfXGqpFUgID5mD0oX\/JQ9OE7AQIeLwCi8htyQLILhMxwLMLWFcQyA3AqFxcbTACvKgWR\/Kti+ODK16Ja9PcAsjtJi9fGoBahi1BJGLI5QcrvaxFpHBaQKKFCIOrzHENhgIBYgvwMaUVcl5mSA3V1rZ6GEcLZza6XiZvbZloyuctX4PTgky5J07d67MmDHjEK8wDbUEG92jXgHrth08dI4VyDYYU4AeOBbNPAJ3fr+1DKnekfl2FNuA4JQgDWCODmeoxEMhFCDDX1WADB9CqVuo9WCeiL2ZhKHW0ep1KAKtjzpwKKMBV\/T3If3egDLKmSU4JchQliNvOEgtJxBx90L5MfTlS3BsKmURxBRgHFrGNwSyh0BwSlCHw1OmTBGdDyQsZk6wMd2A0lu6eodzO4TvNZSfWn\/17bxvzH0rMW\/W5oYqsQ\/SaFPW+j04JYglyPwfixg+wSMt6U7d851bZdTzWwVl1\/qWV1zIsBfrD4uPuT+sP5v\/Sxp5K88QCAOB5JVgI9uFJYjVh\/UXJTZHczQu7hacGmGvIfl82XxeZI747FM5t8cpwlCXFTJIFR9xU35xaNfPz9rcUP0tMolCEMhavwenBLH28BiDD0G1BPnyHPTAAw8IHmLiOmLixInCfCL5kOVIDkqVY3F4kIGiXmSa\/3mm\/KhmpzDURRFCSSo+jk899dRTQhhXb+NXHgL0t\/V7Nvo1OCWI0uLzmig9hRCFCK9du3bKOiwk38aNGwVZEtkPyLeH8RhTnxcZHChwyqQU9NZbbwnKGAVcivJDLnPp0qVy1113ydNPP+2ONIZc16TrlsV+X716tWzfvl0IG4NHsf3OCTHe2bQoOCWoThJYGFFQdKX4mGOOkQ4dOig7Z3jzzTe7BRV\/OMyWGy2XTHQyYceOHYUPP2MpMoxuGA2WfPn0cHgp75Hv\/mmmDR061P0A3HbbbXkxSrOOpbp3Fvv9xhtvFLwwLV68uFH9VWy\/gxXvIe8j72W5KTgliJMEPrmJFwrm9iA2T0+ePFluv\/12iTo\/UMC2bNkiOE0YNmyY22Q9cuRIN3TGK4bKRENAx3kCx\/WM5ohh0LQxePTRR+WnP\/2poJTK\/SzwHvI+Rt\/RclwHpwRpNPOCrAYztwcRZ\/MlHaPOVVkgwdpDSfJrjrNUHKniwYIyUJaY2ShBwjgvMgCPswajc8QwMAzSegZ4D3lv06AgleCYMWPckBYFp8SKMfN+ChLngtVrDMoRbxRdu3aV1157zYkw5GUYfOaZZ9brRcZlsD+FImByhkBFIRCcEkTRrVq1yn0PhMURzhATsmKMhZgPfeQYOqM4CRlWoxzxHIMHGYh4feXku4elGQKGQGUhEJwSBF6GtgxrGdIuWrTIrfiyOoyCJD2OUG4MnXUIzTWyrBTDg4jDMzIEDAFDAASCU4Ks4uoKMIqQlSrmKZjXo8JGhkC5EbD7VTYCwSlBVoevvPJKYbWXeT82PWMZ8v0Qtewqu0usdYaAIVBOBIJTgjQeyw8FSJzhKwsgeg3PyBAwBAyBpBAIUgkm1TgrxxAwBBqAQBPLEowSZNGDbTCs7OYi0pBpYv1jzTUEDIESI5CqEvS\/JcIQeP369fLVV1+5JrM1htVcJVZ9bU7QQWN\/DAFDIEEEUlGC6tqKdqiS03DcuHGwhfOmahGaFeggsT+GgCFQAgSOkBIUmq9IhrQsdKD09AicLw+PtKuvvtqdASY+YcIEGThwoJDXl7W4IWAIGAKNRaDsliBD2tGjR+etN+eC8cWmluDMmTMFP4DkzZvREg0BQ8AQKBKBsitBrR9WHcNcVXR+yPG2X\/7yl3WWIGeD2T+oeTVkTpFzxnqN8lSnCpTNPUjT4Tf3IA88I0PAEDAEQCA1JYhVx3lgnKAy5MWJ47HHHusWRlgcufbaaw9xouArNSqO8vN9DuJaH6\/TuNCivN69ewvpKEJ8+eHUFFq4cKENqwHQyBAwBBwCqSlBlBPngXF6QE1Qii+99JKce+65smDBAheizJT81WGsuf79+7szxeSFsBT1uB3XEGePC\/EsjayRIWAINE0EUlOCnBEGcqw1QqhQD9IsnnCShDw+4U4fF1oMe1GSyJGOSy29H9fIEELXz10pDSG+RucTn+WE+FwnhGVrtKzJudW3Pm9Yn2\/YsIHXMRVKTQliueHqqlgP0nEoYVkyZEb5YT3ifYYhc5w8fIB\/5vdLpFh6esm\/yv3PvVlH9\/1hbZ0i5XOd0P\/87V6BLv0\/6+XyqS\/LFZN+I1f9w12NcluO89gGkN1z8GDDoAAMRo0aJU888YTg5bnczxnerHkfeS\/LTakpwWhDH3zwQenWrZswt8dCiFpxUbm4axwuMKd4+umnOxGUIQqWjyzhgSaXZ2lAb7F0ivzqsrby\/KhvFUwvXNtJfHrxqmYC+bznR\/WUcRdXy6De7eU\/mreRfd0ukz3fuVV2\/Y\/7XHj93Y9WvDt7Xib8N9KfTY1oNw8iGGSl7RMnTpSePXsK29MaU2faTPsLLQNZPnbG+whm5abUlCDKjo8i6eIIliF7AVncIK1YIHC7hcJjDpC8DHkZBhfiWRo5Tqw0ls47\/RRR4rOd4y6ukdEXdpTrT94if\/6HGnn7jnNl7A\/Okg7\/\/Qz55x0n1obfqmiX9j\/84Q+FB7yxuGYxPzscrrnmGunVq1dm+piPHbVq1UqgxmBebL9zX97ZtCg1JaiWGavD2ni14jRN+YWELKzgbos9iMwJhuJZul27dsLXtwg7t66qtQ5rai3OnvLaRztk3bZ9hTTNZIpEIARx+lv7PYT6WB3iEUhNCepChS6GUEVdJNE0ePmIhRBIZXC3padR\/NVkFlGYJ4SIq3xaIcqQe6\/fXtlKcN++fbJmzRohpL1GTQMB+jtL\/Z6aEmT4y8IIFhuKcPjw4cIcHjzSKv1xQRGu27a30ptp7TMEgkcgNSUIMgxhsdiw0CDi8EirdOrUqqrih8NVVVVSU1MjhJXen6m2L7Cb099Z6veyK0HdysK8XS5imwsygfVr4tU5aAlW9nA4cdCsQEOgBAiUXQli6WHxYflxWoSFEeIQ16wWI1OCtgZVJErQ5gSD6hKrTEII2JxggUBi7fnH5siGQoRHGteVTOwfpH3f+qfXhZMnxCuNsjYsqjT802pP6fs92ZaV3RLU6usKMIsiyit2dVjz1ReG6EUGS5AN1YN6t6tVgmtFlSFH7uprj6VXPgK2fap8fZyaEmQFmJVgVod1brAUq8NYlaF6kUERsqF624MXCMoQBciROxQi55mxEOGV73GwO5UbAZQdfcy5c\/p8wCMrpPUtr7gfRQ15DpArd92ayv1SU4IAzNyfzg8yJ0gcHmlJESdIOEnCiRJWrPiGMTy\/fI7srF271u1nYz4jafr8889l7969ecu\/6fz28sxPTpVlY3rKD3q0lP+3cYezEFGK+jLwgvCi8FJAvDi8QD59\/MkeqY+Sbl9ceTt27JAPPvhAtm\/fnrftcflD5NeHraYvW7tH3vrrPlm6ervQTxB9Rv9B9KX2K308eeGHrs9PaC4y9fLqOuJZ+PXrGwQZnolSYrJ\/\/345cOCAfPHFF43qr2L6nfeOY63++1jueNmVIJYZH1Svr6H\/JVOfZP3pHIvT4TfSHKkjVMJSnDVrlmzatClx2rx5s2zdulU421xI+X\/3+XYZenqVzLiinbx1Y7UsuKajDD+7pfQ\/pco9mLwIvBT3\/c1pAy+HT2fdu1zqow7jX5dy0DcnrZDzZmySk+9+uyz3K0eb6sNW06+Y8Z4Mf3azXDXzgzrnGvTbklWfOmXX+qgvZWK\/No7oa\/qZ8I7vtZQLOkkd8SyQBp8yX3hzTeLPqD6XvJv8YHNiS3kNCYt55nnvxo4dq69iKmHZlSCWnnp\/xi9gtNXwGB4jE00r1TUHvjnn2b59e0masEDbtm0rHKNqSNm9unaSiZef6mjB9T0FevMfe8vGe851RFyJOcY4emTQqWKULAZxWMP\/7bBuMm9IO1l6cw\/XT\/SX9hN9+PiPesjw73dxdOlZNfU+d8gwZTLx5U\/rlW3Ic0ae448\/Xo4++mhhtMR1Q6m6ulpwakxYXxm8d5wvL9W7XUi5ZVeCVIqjawx\/iaPwfIJHGjLEkyDMbX7dtCycrWqcEEuRDquqqnIbe5MMmzVr5h6sJMv0y\/rmCS1ECacNccQLZNTOzb0mhUMc1gf5rYQN8fSN31+Nid9+SRepadtCBj65MvHnVOt15JFHCqTXpQ5575qsAx+q1e8AABAASURBVAWUz3XXXVf3HREUHwSPtKQIpwy42GI4ynnG3bt3C7ykyi9BORVTJPNXYE5YMY1KuSFY8zjfWLp6R8o1ib89\/Z2lfj8ivimVkcLwG3Mb10YQcXiV0bqwW4EVwWIUYdg1zU7t2FHAcJtFllAVIf2dpX6veCXI483QGisTIg7PyBDIKgIMt9laxYIYyjCr7Qil3k1CCYYCttUjNwLGLR4BFCFOetl6w75SU4bFY6g5UlWCLMnjMIGFkXnz5smIESPkvffe07pZmHEEsjY3lDW4GRqjCFnoYcuUKsO0N1Znrd9TU4K40E\/SvX7WHuCmUN+szQ1ltU\/GXVwjeupILUMUYlpzhlnr99SUoG5ZwWmCPny6aqtpyrfQEKgoBErUGJQhliHU5+SW7pQJJ1PStgxL1NzEik1NCeoJjsY4UGBj9ZgxY+rAYCjNJmuG1wyzGW6TGKIDBepV6ZS1YVGl9AfDZLbSoAxpEwso5bQKs9bvqSlB34ECirBY9\/ooP\/U6Q0czvOZLdSNHjnR7D9mxTjqKkGNx8+fPF2jhwoUCjzxGhkAlI4Ay5HTK9KtPdVahLZ7k7u3UlCDVQfmhrJYtWyadOnWS9evXS9S5AXJRwgLs37+\/+ENplGqHDh0OEeVkCOXV50DhkEx2kRgCWZsbSqzhgRWkK8lYg+VQhIf3e2CARKqTmhLEcluxYoVwdlCVIZbaM888I6RF6nnIJadKcu3348tzOEdgOIySRI6MHIvT4TfXyBBCyL799ttCXXIRpj0UTYMHRfnRa2Q4lO7z4Sn5\/FxxlSPUdOJKyosLVY5QZYgrKS9fGJXVa8J8+TQNOSh67fM0LVeIHKRpxJWUFxfGycXxo+XkklMeYVRer5ma4TlT70HKJySfEtf5KJec8gjz5SUNGejf\/v9KmXSeyAtvrhUUITwIGaUPP\/xQdu7cKRs3bqx7H5CBVIaQayWu81GcnM\/HwQjvIe9jGpSaEmTxgyNsOBjgYcFqIw6PtGLBYIjLPCDKj03RixYtEobM9ZVz3nnnuU5\/7rnnJBepF41oWhw\/KqceNX7\/+9\/Xla95CaPy0WtklDRNrwmVFxcio6Qyek2ovHwhcpDKEFdSXr4Qd0nIqwxxJeXlC6Oyek2YLx9pyEDEfYIH+bxccWQgP41rJZ\/vxxcsWCB\/\/OMf67wH+Wmal9Dn54ojA\/lpXCv5\/FzxqNwFn78iL\/\/rZpk4b6XzRuPneemll4TnlfdR+dH88JVHyHU+QgaKysCD4ONHlPewvne1VOmpKUEsM7xVMFxdtWqV9OnTxz0w8EjTBmMVDh48WPilYNGDX1hN80POBnNGWFeYUYaAy1A4nwOF2bNnC+UOGzZMxo8ffxipF4xoWhw\/Kodix4vMLbfcUle25iWMykevkVHSNL0mVF5ciIySyug1ofLyhchBKkNcSXm5wp\/97GdC36kHHZXRvITKyxciB6kMcSXlxYVxcnH8aDm55JRHGJXX67i2k04+Ja7zUS455RHmy0saMhBxJTzYzPiXHfLRv1fVPZOk\/eQnPxGMEd5FriHyQsSVuFZSnh9q23nm4+SUz3uH9xreQ\/99Lme8BEqwsOozhzdw4EBhQaRr166CsgDIG264QUjTUojPmTPHLXa888470qNHD006JCQ\/Cg+lSgK\/ZgyDzzzzTEE5oiQ51I2lqYoSOQg5vFkwl5E0ldqLTNL1TbK8Vq1aSbdu3aRly5Yl83qSZH2TLCvkfsezDavH+Dr85DM5pG\/wIAM1Boti+p33rkl7kenXr59Tbszl4dSAIUSckkNZ5SPyT58+XUaPHu2sRtz2474fkHGagPMEiDiy+cqyNEOg0hEY1LudsJcQL9eV3tb62peaJUjFmLNjmOsT83rM75FeH6E8IZVDgWItMie4ZMkSUWWnyhY+cZW3sLQIMPmN9U1Y2jtZ6Q1BAGuwFG656O8s9XtqShBFx1wgK8J8b3jGjBlCmMHvDjfk+bM8hkDqCLCPcNzF1XLDb1amXpc0K5CaEqTRLIIwl8dkLKu57Pt79dVXbTMz4FQAMa+UJb9yFQB50U0Yd3GNcKyOM8dFZ47JkLV+T00JsgKsm5tRhIsXL5ZzzjlHWMmNwdbYhoAhUAIExtVag015bjA1Jciq75VXXum2xTCXx9flsAxZ3NC5vBL0txWZAAKFFpG1uaFC21VpcuNqrUHalJQ1mLV+T00JAjqWHwqQOAsWLGroNTwjQ8AQKA8CrBY\/\/5fd5blZYHdJVQk+9thjbjtLsavDvlcYNuOyoRpcCbmmPDZA68ZqX557ImtUegSyNjdUekTCvcOg3u3l7c375cPdRzW6klnr99SUIKvD7OVjVZitK0r+1pZcvUE+9QqjG6N1t\/nEiROlb9++bu8hw+tZs2a5RRaVZyW63F5kOIb01FNPueNIudpjvAwg0IAqZq3fcbLAvsE3\/q15A1qb7SypKUFg46RG9PQG\/HzEfGF0UzWryyhHDn6zwkx+htfsIURRcpKExRdWKpl3hIeM0htvvCF4sikFvfXWW4IyRgGXovyQy1y6dKncdddd8vTTT5cM31Dbn8V+79vhgKAEOW3VGFyL7fe0F0NTU4IoM6w2PMioMiomZIirCpS5Rc2Ly\/7ocBhly2q0ytDJxDt27Cgc2cFSZBhdCho7diy3klLeoxT1TqLMoUOHuh+A2267zZ0hTqLMrJSRxX6fe\/dw96yyU6MxOBfb72DFe8j76CpQ5j9lV4JYbJwKQVHh9BQirkQaMvXhgKXHEBpFyvljPJW8\/\/77woFs+DhXxckqbqziygL0+++\/XzibbDTHcJjTtDF49NFH5cWrmsnYsWPL\/izwHvI+xr2rpeSXXQliAS5ZssTN26GsokQaMtrouMUOTWcojDl9zDHHSOfOnZ0jBtKUjxIk3XfPRRoyEMBjSRqd4\/ZpGg6GQxrPAO8h72MaVHYlSCNZtWX1FusPsxtFBz8XsZ8QKw1lyRYa5vbIo9YiJ03wTt29e3fBG81rr73mimHIyzC4EC8yLoP9MQQMgSaJQNmVIAqPYSqrtyg2To3o6m4hPYCVyJCXXyuUKIshrAqTl7PHrDjDJ5w2bZqE6kUGB5Z\/+tOfqLbg93DUqFHCRvH9+\/c7nv2pPAR49nkmmb5ZubJpn9cNqXePEClvdXRYqosauNfHw2wxtdD5QJQoViLWIvlRkAyn4RNyDd+XJw4vLfryyy9l5syZ8sgjjwhYbNu2TfA6zY8CPwjUO6262X1LiwDbs3r27Cn33nuvFPIZidLWxkpXBMpuCeqN\/ZA9VfxK+rysxNl8jUswrS\/tYLiONRq3yHP55ZfLjTfe6LIwX8l8Jk44u3Tp4o4RugT7kxkEmJoZMGCAMM2jlea54BmA2MkA\/6OPPhJ+6Fq3bi30OT+C8I3SRSAIJZguBA2\/O8qP1W2\/BIb2POhYo8xVsgUISxcr4JVXXhGGu7wEfp42bdr4lxbPEAIovosuukjWrVtXV2t49Dd77TgMgOXPj+NRRx3llF+doEWCQCAVJYj1o3N6eHtmXxLDY34146ynBNFKpCh+6fmOiW7OplAedFyB6eoz6Vzz8KMQTzvtNCGOrBLfV8AS\/uqrr9yXvr7+9a9rkoWBI4AFyEb4xx9\/XI477ri62rI4x6Z89qbyXPNJB5yM8q0ZvqzGjgXI+roOslQjZVeCzNMx74WllItIQyZVVAq4OZ\/zjJtfVCVIMSj8AwcOOC\/XvAR8vwG+Eh+cYWj085\/\/XH73u9\/Jd7\/7XU2yMHAEeE45lUT\/RavKaEDnqjmxxDdu2NP6xBNPyKRJk9x2ruiIIFqGXZcHgbIrwfI0K\/y7oEChr33ta8LHpXgxHnroITdnFH7trYYNQaCmpsbtAJg8ebIMGTKkIUVYnhIgYEqwBKCyR1GLZa8iwyK9TiO0e5YfAbZuMT3CnfnaIftbiRuFh4ApwQT7hOEPQx5Vgmzk5hp+grexogJHoE+fPsI8IKu\/OOtgfhArMPBqN9nqmRJMuOsZ5mAFsMhDyHXCt7DiAkcAx8CXXHKJO4bIJ2BxIGE\/hOF2minBRvYNE+OQFsPDzgZuFn0IudY0C8uEQJlvg9JjFwCh3pqFM54Bjnr6fE23MBwETAmG0xdWE0PAEEgBAVOCKYButzQEDIFwEDAlGE5fWE0MgQYiYNkag4ApwcagZ3kNAUMg8wiYEsx8F1oDDAFDoDEImBJsDHqW1xAwBNJAINF7mhJMFE4rzBAwBLKGgCnBrPWY1dcQMAQSRcCUYKJwWmGGgCGQNQSyoASzhqnV1xAwBDKEgCnBDHVWCFXFM4p+PoDz0Up42Q6hfo2pAx6h8euobcRxrpanPHWVr3wNSQcDHK0qz8JsIGBKMBv9FFwt+bIfZ2OV\/PPTwVW2gAqhxPASff311xcgfbgIZ8T5aBgf0Do81TghI2BKMMDeyWqVsJywEvlEAnEsqzPOOEOwFuGplYQ1BY+0ESNGiFpQyJBG+8kPX+PIQ8pDjo8bQfC5L4oMeWTgQchBmo4McXjIKvE9EDyC4y1aeXGhXz73oDzKVUcJtDsur\/HDQ8CUYHh9koka8YEpFACE8lIFx+cE+Jwk314ZP3688OEprMVBgwYJeZC7++67hQ8Q8S0O\/O7lazDKCg8t+OWDcE8Gjzx83Oiee+4RFBj3JSRt+fLljjd\/\/nx5+OGHhU8YII9\/P771wT359gc8JXw\/ogT1mpD60j4IecqHj9VLmyDaOWzYMMESJI170S7iRtlAwJRgNvopuFr6w2H\/uzDqSZtvaqCk+JAWSgSFgnJaunSpawtKBcWB01nHiPmDg1qUD\/IQcRQW4i1bthQ8NuO5m\/vCQ56PWmHRYZktWLBAunfv7j5bgBKlXl27dnXffEEewopDMVIW10p+G8nLx8E0jRCLkI9p8ZkEriEUKXUgbpQNBEwJZqOfMlnLzp07O4sMiwlCWeJq\/rDG1MPA2iK\/EpZYPVkOS0ZZoTwh4ocJFMlguI7C8xVgkUWYeCAImBIMpCMqrRpYVXxl7dlnn3VNQ2kwd4bigIFlhQXGUJdrteawonw+8osXLxbm2eBTBmWRJxchj8XJsJs8WJqEasWtWrVKsCj9vFikuMDHSvT5cXG9P45TozLUnzpE+XYdLgKmBMPtm0zXjOHo9OnT5Ve\/+pVbGJk7d65MmzZNqqurZcKECTJ8+HDhWxwoHxqKIkJhMWz2+VhaI0eOFIbVKC8+ZZlL+VAGhDzzjyg98qA0GRZTPsopOhQmD4R1iJVIPB+h2GkL9WSYD+mcKEp6xYoVrl35yrC0sBAwJRhWfwRfG5QJnw3IpYjgkYYMDUH54F6eYSxDYRQjfBQVPNJ69uwJyxH5lc8HzXXYq3zSlEcZWib3477wKMiXJw4PBYXVibLjOkooTXhYkFqe5oWvPIbm3Je6KHFN21h0adeundBu8sSTpYSEgCnBkHrD6lISBFBsfPiIwlXZEfcJJYeCxMLz+YXGUbK3q7+1AAAAsElEQVRYvldeeWWhWUwuEARMCQbSEU21Glhbat2VCgOsNKw1rEWUXdx9sCQbWhfKxXo1KzAO3XD5pgTD7RurmSFQKQgE3Q5TgkF3j1XOEDAESo2AKcFSI2zlGwKGQNAImBIMunuscoaAIVBqBEqhBEtdZyvfEDAEDIHEEDAlmBiUVpAhYAhkEQFTglnsNauzIWAIJIaAKcEEoLQiDAFDILsImBLMbt9ZzQ0BQyABBP4TAAD\/\/17SdUIAAAAGSURBVAMAy9dYP1vs270AAAAASUVORK5CYII=","height":193,"width":321}}
%---
%[output:7390a984]
%   data: {"dataType":"textualVariable","outputData":{"name":"used_device","value":"'infineon_FF1200R17IP5'"}}
%---
%[output:12a6233c]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUEAAADBCAYAAABRyQJdAAAQAElEQVR4AeydDYwV13XHjx1XfBgT7EJgwbgQU5EIA0WEgEWKm9YkSJTSIFkthAaJSmtTU4GMgSLFhIQWGrAtUHENSEVCRdCUipgSIltGbbGyqjG1YoNJjLrNbsy3SQkBAosaxeU3cJ7vzpuZN2\/3vZk7bw\/i7Mzce+695\/5n5j\/nfr67P7J\/hoAhYAj0YATuFvtnCBgChkAPRsBIsAfffKu6IWAIiPQ4ErSbbggYAoaAi4CRoIuGnRsChkCPQ8BIMOdbfvz4cRk\/frw8++yzdbHkww8\/lMcee0zmzZsn165dq0sZbqbUg\/pQLzc8i3PqRz2xIYvyalnGoUOH5NOf\/nQg1dqPfl6YhzHQ523r1q3hKG+vjQS9vTU1MiwiGwjqG9\/4RkRM7YOyLOvNN98UZMaMGakqUq1tvNiQVarMq1CCvHfs2CHDhw8P7H\/++eerSO2X6qc+9SmZNGmS7NmzRyBEv6yLtsZIMBqXzELHjh0r7777rtTrweehPHz4sOzevVv69esnvPjz58+XX\/7yl3WvY5ZlUZlXX301IJJx48ZxmSjV2gYBbtiwITFPi7yNAB+hU6dOybFjx24HeP63oUmQZoI2McLNBeIIe\/LJJ4MmCHo86O79QodwhGYWX2zieYE0LeHEc0048VFC3uipqEdBGtJSFulUj2t0ORKueoQhGs7X1m3uYiM2EUYcwjlh7e3tsnjxYrl69ars27cvaCYTT\/6IpkWXc8K0XC2PMM6xAcF2dAh35dKlS5FlUR72kBbhnDDSUiZlE45oOWBCPAJuxCFu2eRx9OjRwAuB+NEljPzRRTgnDInCgXB00EU4J4wylQCbm5tLXReEo4e4tlB2WMiH\/NBFqCf1RcgTDxbimDJlipBv2vSu3tNPPx35LFMG5VEu4trKveOaeOxDsBUb0EVmzZoliMZRpqYjHkGfcGTw4MFy3333CR8lrn2XhiVBXqDXX39d9u\/fLz\/5yU9k0aJFMnv27E4PGGQwYcKEIH7FihXCg643003PA3rmzBlZs2ZNp\/v5\/vvvy6ZNm4IyiNi5cyeHMuElJu\/t27cHZc2ZM0eWLVsWeGVlyk4AduMh8sDhvU2fPj1ITz6QGPk66hVPBw4cKFu2bAkeUGw4fMtDVMIgMZ7itGnTgiYZdSaspaWFgyxYsCA4urhgXxSuKD7wwANlZfXt21eWLl0qw4YNC7wE7s3ly5cD3Emza9euoGzqhxdx9uxZgkvCvYEwuFeUDR6QGS\/thQsXhLxGjRpV0gdzLqiLWxZ1DuOQZNvjjz8ulEle2MY9SbIFPVew74knnijVm7oRP3PmTLl+\/bqQJ+SnzWHKI14lnJ66nDhxotPzyLMMkYEL9\/bll18uPV9r1qwRnl9wQAYMGCDr16\/v1EdM\/N69e4VngnLXrl0r2IStS5YsEcojHAk\/j2DjPs+QIGVw\/yBg0vgsDUmCPDR4BbwkNDe5ATwYPGTu14mv1dSpU4kWjlwTr+nHjBkjI0eOFF4a+jnIk7ggwa0\/hBGXdNN5CN54441OzTReIprAatutrMr+06TQQIiIh1zDeEl42J966ilVqdnRxUFtVxyoOxhUwjXOGEiWZjkfDgiAjxL1Ql\/L4h7RnEV34cKFRJWEe8M9wkYCwQPviRcVEtS8iFMhnpeZeDAHe41zj5QXZ5urp+dJtqiOHrEPO6gP5SCcE0ac6sUd0UGXDxRpeW7CdXFxGTVqVODtU2fypM4HDx4MPkBgQV6Eu6LPMmHh8kiDEIeEn0e9H4QTzweFDx3ECskT5rM0JAly8\/EKooCP+zq5RKY3jq8mLyTuPp4XeZJ3VL5xYeTFw8BDwcMRp5cU3traGnhv2JikV4s4XjBIDrKj\/ngA+vJRdzCIKicOV1cXosNb4YWaO3du4EHz8qKTFieIDvLknuAVkhZ8OIYF71XzR5c0cd5zkm3hfPU6rS1x9pFPUhzxiOpAblxXK3jvPMdDhw4NPG0+NEl5pC1PMeV+gIWmS8rbx7iGJEHIAnc8CnAeBL6m4Th9wYlXsuJl5auI14Xw9YUkwmmTrskLAoQIedGTdOPiePh5yLAxTqeW4eph4bGRr37pu4Ir6VXa2tqCZhVeediLTYsTpEZzkPuhEs5Ly+Necc\/Qo8lGuNtM5FolyTbVCR\/T2sL9C6fV66S4sE4Myaha5FG9d55lmsWRSqFAtalSeTTjwVYFjzOUVSEuG5IEaaLi3tMnSP8FdwJPjmYALzjXCMSi\/Xi48lwTr+nxgng5eJDoFMaLwWMgbVqBcPGkKBtCJR3eCJ3RahthSQIJ8cLRBEOPdKQnnzB5YC92oxclSUSm+ngNeAs\/+tGPBK8QMiFOcamEK7pIXFnqNYI9mKMbxgmcmTZCnAr3Bn3uFWF4OOAAHpQFRoQjpOd+cd+4fxAl5MvHEV2Ec3RdibKNeCUGzpEkW4h3RfGkPtiFcA7GxLm6UefooEu3CmmpD\/Wi\/lH6UWH6Edb3IEpHw8Ll0SJAND78PPIc4mXTT4oOH3vK4+PP80mYz9KQJAjgfJV4gXHVuUF4AHgQ9KcRj\/DSNDU1BSNqdKLjLWi8m56vKPp4RrysnFcjvIDkrc0HynrhhRdEyaVSXugxaAD5UBfqRN3IF3u0fwk76fCnDy8uTyUyXgYlkLCu6hDOy85RxcUFW6JwVV3NR8sinIEUXihetJMnTwq2KvEw+EMdwIkXbeLEiSQpCfcG7wP8KJt8FUclNfVewAVdXkTyRB\/8wAe7ED6U5AEOFJJkG\/ZCRNgGuZIn+UfZQl6uUBaDDhAD+SDE009HHOdJgo6bnrKpVxrPjrTPPfec8BEmHfP3OGILZBVVrqbR+0TZpFHd8PMIBjzf3B90aLHQbUKrivtAmM+SCQnyxUKigOArzkPIQ4rwheNLF6VbbRgvrLrqNIu4eeE8vvSlLwUjruhBKm68m55RMx4O4smH\/IjnmnDi6ViPu+nkTRkq+sCE81I9jSd\/RPU0vZZNHLoavm3btmBOIPZgF8K5axtp0acO5EseYVEd8o6Li8qDdG6+XLt6Wj\/CDhw4IIjaBnacE0cePBeU7Xph2EO8CtfoUE9Ijb5MfX7c\/NAnT7e+1dhG\/uBIPthI3pTNtQrX2BIlbnr0NQ90yYtr8kePsLAQTjxpEfRJhx71cOumGKs9HEmDkAdpOZIneJCWPMhLxU3Dh4ZwiFc9O01HnghlooNAgnjs4Q8ocbWQWudRdxLEReZrG2c4gOEN0FQETL05cfoW3rgI8KHkQ0jzilri1eGt4+VxXUl46fBAeKYq6Vp8PALqmKhDgseI55jWs6PbBq9ZPd74kvyIqSsJ8kWm78N1pcPV5kFPC244rV03FgI0qXh5aF5BhhzxQvA60tSU54wPKn2NafRNJxoB8KZrQJvQ4IoXmKb5zTuPN87oP55mdAl+hdaVBHmIGRSA5OKqDQniKfLQI3iOcbq1DMf9pxnADa9lvpZX1xHgpaElQItAhWZZ2hxpHtLU496mTWN60QjQvNV7wBFcwTda++NQvYek\/zjU77O6kSAuNf0CdHbHQcBIF53ieAAATUezO\/OcdCqnT58O5jjRWWtye6MAw8FwaJRngPdb3\/Wsj3UhQciNUTiWNCV9PYjjC6NfDdxumjM6BULBAKDly5cH20ExMmcyz7CYZxg00nvA+817ru98lse6kCBz1fhCMZWDJi7NXYSO7zSVc0cD0QecI0eOyMaNG4ORT4izaML6S+pS5DqAudVjt1fPYCPcD+rA+817zjuStdSFBOlno7+NJi7CJFUk3FdDJyq7U9B0puIQJ6NQcaNKdM7iLRZR+CCwjIu5b4n2T5kSLFz3Vcfq4df9aYT7MXnyZF7\/3KSMBCEmhsbx4NIK+mlrQP5KfHSirlu3Tug3pCz6A2lGE542v6LoDRkyRL72ta8Jx6LYHGUn9ls9opDJJ6xR7kc+6N0utYwECe7fv3+wuB0vrpKwCgN90sUJHiBCPATHBFm8Ra45qtfIkWvCTQwBQ8AQyAKBMhIMk1QlIyAtSK2SnsUbAtEIWKghkC8CZSRIc5XmLSNPjPLma56VbggYAoZAfREoI0E8QWZ7M0jBAAV9dbqMqb6mWO6GgCFgCGSPQBkJYgLz9rQvkFFdVn5AhniIeIromBgChkCXELBEniEQSYKujQxoQIh4hoQzdQNCTDvnjzQmhoAhYAj4ikBFElTDaSbruk5GhNkLzrxCRceOhoAhUFQEUpMghEdzGC+QCZpscaR7ixW18ma3IWAI1B8B30tIJEFWcrCxJcRHM5i92vACaR6zhIq1v75X0OwzBAwBQyAJgTISDHt87ATD7i4Qn01mToLS4gwBQ6CICJSRoFaCUWGID6lmTzdNb0dDwBAwBIqAQCQJsgyOxf5pKkCTmbXAqmtHQ8AQMASKhEAkCV65ckUY\/KAvsJKgh36RKm22GgKGgCGgCJSRoDsVhqZwGmHqjGZoR0PAEDAEioRAGQkWyXgvbDUjDAFDoNAIGAkW+vaZ8YaAIdBdBIwEu4ugpTcEDIFuIdD2iRFy+Y\/\/QZ469JHsOXq+W3l1JbGRYFdQ69FprPKGQH0Q+OBShwy\/v3d9Mk\/I1UgwARyLMgQMgcZHwEiw8e+x1dAQMAQSEKhIguwuzS7TzBfkyJZaTI5meV1CvhZlCDQKAlaPOiPwwaUbdS4hOftEEoQAm5ubZdq0acL6YbJ65JFHZObMmbJ06VIhnjATQ8AQMASKikAiCV6\/fl3YQGHq1Kmd6sc14cR3irALQ8AQMAQKhkAiCbJ6ZPTo0bJ+\/fqS1\/fee+8FvxNMOPEFq6+ZawgYAhUQyDP6oQc8HB1me\/2FCxfKM888I\/QH0i+4aNEiITxPsKxsQ8AQMARqgUCiJ6gFsJWWu4aYH2LSuFoe+d0SpJZ5Wl6GgCFgCCQhkIoEkzKoVdyhQ4dk3759tcrO8jEEDIGCIMAk6TxNvTupcKbB6O+KMEUmSth+nz0Fk\/KpFEc5O3bsELbwr6Rr8YaAIdC4CHjXJ8jAx6RJk2TFihXiNoe51p2n6R9k4KQ7t2XDhg3BNJyhQ4cmZnPkyBFpb2+Xjo6OQsrNmzflxo0bhbTdxdzq4dfzV+T7wfv8waWO4L3PgwApOLE5jIfGT2syJQZlFa4JJ55zpstoXLVHvEjSz58\/v2LSzZs3y86dO+XcuXOFk\/Pnz8vFixflwoULhbPdxdvq4dezV\/T7wfv8n8dbK7779VRIJEE8QabCsHs0fXYYwpFrwolvaWkRfn6TuGqFydZbtmyRxYsXS5pfrtu4caOw7X9TU5N0WXJKO3jwYBk0aJAMGTKkcLa7WFs9\/Hr2in4\/vjjrT+TXfQcG1JHH5gkUnEiCKDAVhp\/ZXLZsmdAnyJFrwvHiDh48KJs2bUK1amlrawum3UCq5M3ACBI3Qjxs2DAZMWKE9O7du3DSp08f6dWrV+HsDmNt9fDr2Sv6\/bi7\/5ASb3xh1IDSeZYnFUkQzEQfIQAAEABJREFUY8aOHSv83Cb9ghy51vADBw4IHiHX1Qr5kB\/5IvQzIhBstXmZviFgCBQPgW+\/1lYyeurD95fOszxJRYJZGmRl1RoBy88Q8BOBH7Relpb\/uRwYd8\/PTso9P3s\/OM\/6T0USpA+QpmpYmDrDwEgtDcYDRGqZp+VlCBgC\/iHAiPDif\/pxybDe7+8vnWd9kkiCkNzatWvlxRdflOnTpwt9gceOHQvm882dO7fLzeCsK2nlGQKGgD8IQIB\/9Pc\/FI5Y9elf\/\/SWF3iS01wkkQSxqH\/\/\/vLwww8HI8CMBDOKu2rVKmFABJJEx8QQ8AgBM8VjBCA+lwCZG7jlTz+Tq8WJJNi3b9+A\/JjLM2PGDGFS8yuvvBLsKnPlypVcDbfCDQFDoDgIQH4MgvzOX\/9nyQOEAP\/1LybkXolEEsTr2717t9x7773CfCQ2VmU3Gbn1D0+wq6PCt5Lbf0PAEOgBCEB+T+\/5sUB+336tvVTjlV8eIe98\/VGBCEuBOZ0kkqDa9M1vflOYzqK7yUCMEKTG29EQMATyQ8C3kiE+9fogvz1Hz5dMhPTw\/lZ+eWQpLO+TVCSYt5FWviFgCPiLAKQH0anHB\/Hh9RGuVkN+L839bOD95TUpWm0JHxNJkIEPflSJlSFuQq4JJ94Nt3NDwBBofAQgNyW9WS\/9MGjqQoCEEacIQHxzJw0RPD+avpxrnE\/HSBKE5Ngii62tTpw4IbqsTecKcs16YQZOfKqM2WIIGAK1RQBSY1IzBAfR4eUhnBOmk5211DDx4f3V3PPTwmp0jCRB+v9YzsZ2+mPGjAnmB7KszRXrF6zRHbBsDAEPEIDsEIgNgsPDe+CZfw+8PKa0EEYcOq65SnqQnXp8nPtOfG4dIklQFRj9ZW0wpKhhdjQEDIHiIgCJuZ4dZIdnp6JkF\/bwtMZxpEdTt0jEp\/XhWEaC9POxJE6bvnFHdNAlExNDwBDwB4Ew0UFskF2UZwfZoR+2HrJDIDc8O7y8Sy9+MRjY4JrwopJeuK5lJIj3d\/jw4U47SbvNYD1HB91whjW\/tgwNAUOgEwKQlnpze394UdYc+pl8ZdvxoOkaRXQ0YyG7TpncuYDoEEgNgeAgPAYyEK4JbxTCu1PtTocyEuwUaxeGgCGQKQIQHPKD1ssCeTHfTj05mqxhkluyt1UO\/PiaHPnptdJKjCiDIbqpDw8QCA1ic4lOyY5w4huZ8KKwSUWCW7duDTZU1aYx11GZWZghYAjEIwC5IVEE5zZXITsdjGC+HWSIJ0fauNwhOQQSQyA0iA7RZuyBpycI4cRDdOjH5deTwiuSILs879mzJ9gBmqYwI8ZcE96TgMqurlZS0RCAnBAlN0gL7w2B3CA19eA4jyI4SC6p3hAW4npzENo\/L\/yMvP2XI+TNZyeU+usIV6KD7JLytTiRRBJk4IMfVOJ3QLT\/jyPXhBNvIBoCjYoAxIbEkRsEF0VukB9EiEBu5JGEEeSGKMFBYBAZXhyinhzNVtebQ+8Lo\/LZjTmpPkWLSyRBCI8fVOKHkJTwOHJNOPFFq7DZawhASmev\/ErePtMhu986J9rvBnlBbHhracgNgquEJuSGKMGt\/PKIoEkKuSFRBAcB3ia4AfKFnH53o1K9Gik+kQSpKDs9s4Eqq0foE+TINeHEmxgC3USgJskhNqSS16bk9rm\/PSrN+87Ls6+03yLB9mAQIq3nhsEQG6LkBmlBXgjkhtcWRXArvzwyGJyA3BDyMskXgYokiHlPPfVUpykzXBNuYgjUCwEIDXFJLc5jU2LDg9P+Nrw6SA3BY0PS2AqxIdWSG+QHESKQG3mkKc908kcgFQnmb6ZZUHQEIDQEUkMgJwSyQmiGIhBZHKnpSCnpIDXyS4MLhIS4xLbsDx6UNY8PlH3NY0sL\/KM8NyO3NAgXW8dIsNj3LzfrISAEQkMgJgRCQyA0JIrUkrw18kxTKUgNcYltZai\/La5JCrEt+4PhMuuz\/WTKiH5Bvxt5pSm3FjqWh18IJJIggyAsj5s3b55cu3bNL8vNmpoiAPkgLqGFm5+Q2pTnfygT\/65dOEJwEBoC8SEQIYKnhpBnGkMhIQRSQ2hWIhAWQj8bkkRsK0P9beSXpmzT6dkIJJIgo79Mh2E7rXHjxpUmTNtkab8fGogHgdAQSAkJkxokhqRtfnaF1CAil9RWpvDWdBoI5AcRIvSzIeTnN\/pmXdEQSCRBKsMOMmyrxURpFcLxEPEUOTepLwIQGpJEaHhpUYTmemlRfWrkm8Z6yAeZ\/Fv9ZOKw3sEIJ+QEUSF4aUjYU+PaJbWV5q2lgbuxdDyvTUUS1A1WmR6j8vLLLwseIp5iXP3cdEmE6eqRf5JuXFlFCod0kLSEVg8vDbwgNCTKS0site8+OVa2zxkiL3xlZDDfDSJE8NIQ8iR\/E0OgKAgkkiCeHhOjBwwYUFo2hzeIZ4iHGFdJ+g8hyV27dgVTa5hXuHTp0sh+xQsXLggbtx47dizQLeLuNJAa4hIb\/WMIHhpCH1q4L414JOyhVdPs5B5APAiEhkBKyMo7TU8lNbwyRkARzpEoL420EBpCvpRhYgg0KgKJJIinBykhnKcFgV+i27ZtW\/ALdaSZOnWqXL16Va5fv85lJ2ltbZWhQ4cKaTpFeHbhkhzEhUBuNEFV3KYnfXAIhIaQPk2VIB2VJEKLanpCaAikh6y80\/RUUiPfNDaYjiHQkxBIJMEuARGRqKWlReJ+kwQS3LdvX2nQ5dChQxE53A46cuSItLe3S0dHR13lJx9eEwQSg+zCJEc4kobchva\/R5DfGXKPPDLwLvnK2AGBPP\/HIwRhATzCAviz6x4NFsJzjuz9888GzU6ankseawrSkf5zD\/YW5FN9pa44xOF88+ZNuXHjRi5lx9nUlXCrR33fozT3hPf5zJkzt1\/wnP7WlQRpTtPHt2HDBlm4cGGZt0ez+ezZs7JixYqgKbx9+3ZZtmyZ0E8YhcfmzZtl586dcu7cuZrL2ydPyfZ\/+2958h+PC0uqEAgQsovz4iA3BgmYb4Yw+Rahz+zAggeD3T04In\/\/R4Pkb373E7L80T7y9d8bIF8cLoE8fG+HIL9x8+c1r1M9cDp\/\/rxcvHhR6MaoR\/5Z5Wn1qP071JV7x\/u8fPnyqNc9s7C6kiBN6MOHDwf9iWvXrpWwl9evXz\/hB5t0GR7rkukfxHOMQmDjxo2yYMECaWpqqpn8X6\/75a\/\/47LM2nla2KGXDSrDZdOMpElJExOhKYrX9l9\/NUlofm77s7GCNP\/+bwvyh58bKRNHD+9k4+DBg2XQoEEyZMiQTuG1rEsWeVk9avfs1eJ+Ff1+8D4vWbIk\/Mplep1Ignhy\/L5w2DPjmnDi01jLT3MOGzZMaPqm0R81alSkGnmMGDFCevfu3W358Fb35LLvtgWTfr97\/HKn8pT0IDsdRID8IEKEAYOPbUhnS58+faRXr17dtrvacmutb\/VId79rjXtcfkW\/H7zPkydP7vT+ZX0RSYKQ3Pjx4wXPjInSs2fPLvXZMY2F67g+PioAOUKS5MN1W1ubnD59Whgg4VolrMeGrfQPMDFbdWp9pGnLpGH6+Wjqav4Q38pbo6mMmCKQHmSn8XY0BAyBxkQgkgSZ\/sI0GEiJ5un+\/fuDPjumx6jQjKU5GwULzeB169bJ\/PnzA\/KENFevXh2MFrvEF9ajP5CpNYRH5dvdMAiQEdxvv9Zeygryw+OD+FbeGk3luhRpJ4aAIdDwCESSoNYaMurq7w4rkSppPv7440G24TxdPYiX60Cxxn8gQLw\/jmQN2a284\/mZxwciqcUUDYGGQqCMBPHUGNGlOcxABuc0gcNCOLpFQAPigwDVVubf4f2tvOX5aZgdDQFDoGciUEaCeGqM6OKV4b1xrt6ceyQcXd9hCxPgylveHyO6eIK+2272GQKGQP0RKCPB+heZXQkQIH2AWiIEuNK8P4XDjikRMLXGRiCRBGnu0uwNN4W5Jpx4X+GBAJnszBEbmdpiBAgSJoaAIeAikEiCNHdp9rrNYM7nzJkjzz33nBDvZubTOUvaEGyi6cuUF85NDAFDwBBwEUgkQVfRPWeWN0vYfPUE8f7wArEZAmQQhHMTQ8AQSIFAD1PpEgmyrO3KlSveQqUEiIE0gyFCzk0MAUPAEAgjkEiCeHr0\/dEH6AobIvjaHMYLdJvB1g8YvuV2bQgYAi4CiSRIn19UnyD9gkyfcTPy5dz1Arf86Wd9McvsMAQMAU8RuFsqGIY36K4D5keWfP31OdcLZEK0rQSpcHMt2hAwBCTRE2S\/P7bFnzlzZrDuF7zY9mratGmyZs0aLr0S1wu0ZrBXt8aMMQS8RSCRBNkOn23xw7u\/cH3y5EnBS\/SlZq4XyECIeYG+3BmzwxDwG4FEEmQfQLbMYhdhtxpc+zY6rIMh2JnsBaJhYggYAobAbQQSSZCtsmj6Njc3l3aFZlMFrvkFOQZObmeT\/9\/db50LjMALZFpMcGF\/DAFDwBCogEAiCZKWPkD2E2SvP6bJQID8FgjhxPsgP2i9LOoJMiDig01mgyFgCBQDgYokSDXY449dZZgag\/g2Pablf36OmYHMndQUHO1PCQE7MQQMgQQEEkmQgQ93ekxCPrlG4QliAE1hGxABCRNDwBBIi0AiCdLnN3r0aGGZXNoMs9ZzR4WtKZw1+laeIVB8BBJJEE\/w6NGjwjI5+gNdYTkd8XlDsOfo7QER7LCmMCiYGAKGQDUIJJIgnmDcsjnCia+msHroWlO4HqhanoZAz0EgkQR9h8FtCg+\/v7fv5pp9hoAh4CECZSRIE5embhF+aAkSVExtQESRsGOPQ8Aq3C0EykiQJi5NXabEMBWGc6bFhIVwdLtVejcTu1Njpj58fzdzs+SGgCHQExEoI8EwCHiG7jQZn3aRsf7A8N2ya0PAEKgWgUQSzGIXmePHjwtNb0aeaYZDumkqQVNYV4lYf2AaxEzHEGgUBGpbj0QSrPcuMpDsli1bZNeuXUJzm\/XIbN1FeKVqQoKqY\/2BioQdDQFDoFoEEkmw3rvIsEHDtm3bSnsVskUXW3dBvpUqYv2BlRCyeEPAEEiDQCIJQlJZ7iLDyhS27oJ8o4w\/cuSItLe3S0dHh2h\/4ND+98jnHuwdhBHus9y8eVNu3LhRCFuTcLR6dHh1D4t8P3ifz5w5E\/W6ZxaWSIJYwW4x9d5Fhn5A+gNZmbJw4UKBfCn7tnz8d\/PmzbJz5055++Sp0q4xTffdI+fOnfNezp8\/LxcvXhT2YiyCvXE2Wj38etaKfj94n5cvX\/7xS57DWUUSxKZ67yLDVJvDhw\/Lm2++KWvXri3tXUjZrmzcuFEWLFggNz7RrxT8e58ZKE1NTd7L4MGDZdCgQTJkyBDvbU3C0+rh17NW9PvB+7xkyZLS+5zHSSoSzMowmsHDhg2T1tbWyCKJGzFihLx9pqMU\/9joQdK7d2\/vpU+fPtKrVy\/v7ayEpdXDr2et6PeD93ny5Mml9zmPk1xJkGawOwE8fi4AAA9CSURBVAexra1NTp8+LQyQJIHhjgyzfVaSbgHjzGRDwBDIEIFcSZBm8Lp162T+\/PnCPMHZs2fL6tWrS6PFcTgoCUKASJyehRsChoAhUAmBXEkQ47rS32iTpEHOxBAwBGqBQO4kWG0l1AsknU2SBoXii9XAEMgTgYokyK\/L0VQNC1Na6NPL2niXBB96oE\/WxVt5hoAh0GAIJJIgJMeUlRUrVgTL2ljapsKUFvr0ssbDXSlia4azRt\/KMwQaD4FEEqS6\/fv3rzhai15WoitFKM+aw6BgUjgEzGCvEEgkQTy9mTNnBqs0vLL6ljEPPWA7Sd+Cwf4bAoZANxFIJEGaw3v27JF9+\/YFU1jcfsE8+gRP\/byjtFzOmsLdvPOW3BAwBAIEEkkQT5C+P+0HdI+EEx\/kksMfawrnALoVaQh0CQG\/EyWSoG+m28iwb3fE7DEEio9ARRJkg9N58+YFzWGObHLAUjeayllX3x0UseZw1uhbeYZAYyKQSIIQYHNzs7Cn4Pbt2wMEHnnkEWGwJO0O0EGiGv2hT1CzsoERRcKOhoAh0B0EEkmQHZ7Z6Tm8oQHXhBNfXnj9QrQ5DAEi9SvJcjYEDIGegkAiCTLwMXr0aFm\/fr3gFQLKe++9F2x4QDjxhGUlv+77m0FR1hQOYLA\/hoAhUAMEEkmQ\/J9\/\/nlht+dnnnkm2PSUfsFFixYJ4cRnJb\/uO1CQrMqzcgwBQ6BnIFCRBIGBH2F3p8ew5T7heYln02PygsHKNQQMgRogkIoEa1BOt7PQpjAZPWQbJwCDiSFgCNQAgcKQ4K8Gji5V1\/oES1DYiSFgCHQTgYokyIAI\/YDukrlnn322m8V2L\/lDtm64ewB2M7UlNwQaCYFEEoQAmSc4dOjQ0lZax44dk7NnzwrESHxWYPxq4GeCoiBAJLiwP4aAIWAIdBOBRBJkHiDzAflZPC2nX79+smrVKiGceLF\/hoAhYAgUGIFEEmQeIL8J6s4TpK78YDKrRojnOgvRPkHrD8wCbSujEwJ20dAIJJIg64PZWZr1wuPGjQvWD9M3yNZaGzZsKF2zrVY9UXLnB1pTuJ5IW96GQM9DIJEE8fTYMsudIxh1jk49oTMSrCe6lrch0LMRSCRBH6F5yOYI+nhbzKaGQqBnVSYVCW7durXU9KU5zHWWMNmPK2WJtpVlCPQsBCqSIHMC2WKffkGawhy5JjwJqvBPdXIdpX\/8+HEZP358iWTpX6QvMkqXsIdsjiAwmBgChkCNEEgkQcjo5MmTsmXLFqF\/kDI5ck048YSFhfAdO3YEGy5AnOxFyAAL4WHdCxcuyJgxY4T5h+jSv0gZrp67maqRoIuMnRsChkB3EbhbJD4LyIgtsxYvXixKYBy5Jpz4qNSE7969u0ScjCyjB9FxdKW1tVWYjM38Qzc86twIMAoVCzMEDIHuIJDoCZIxW2bNnTtXpkyZEjRZOXJNOPFpBG\/vo48+ksGDB5epQ4JMuaGvEYlqNuuO0qdPn5b29nbp6OgopNy8eVNu3LhRSNtdzK0efj1\/Rb4fvM9nzpwp44UsAyqSIMawdRZNVRWuCU8jLK1jsjXL7MaOHdspCXEswVuxYkWwLI9m87Jly4R+Qlfxg0sdweXd1\/83+A3kc+fOSdHk\/PnzcvHiReGDUDTbXXutHn49e0W\/Hyy8WL58efB+5\/UnFQl21ThITtceRxEnTWCazRqHl0n\/YEtLS6lIJUACHh07SljC19TUJN2QXNJi\/\/e\/\/30OuZRfK7yogNXDn+ev6PeD95lVadQjL6kbCSoB8iNN1TSdAWLUqFEcyoQ+Qb5877zzjrxTMDlx4oR85zvfkddee61wtrtYWz3e8er+Ff1+8D4\/+OCDZe96lgF1IUGXANXLi6oUgyz8fKc2f5l+Q\/+ADqSE03z3H7cFu9fQtC6aqMu\/efPmwtYBzK0e87y6f41wP6jD5MmTJS8yrAsJtrW1CV8od32xDnq4xMco8rp164IfbiKe\/kCm3xCuBIj3d+nFL8o7X39U\/uVvnxaazya7q8LB8DK8fH8GNm7c2FgkyADIu+++Gwx26GAKR36rBII7cOCAoAPRcVRdjlwTHhbIkD5DkynBSL3hYDg00jOQlxcIz9TFEyRjE0PAEDAEioCAkWAR7pLZWB0Cpm0IVIGAkWAVYJmqIWAINB4CRoKNd0+tRoaAIVAFAt6TINNndJeZSjvXVFHvuqqmsZlpREw5YVQc4ZywuhpWZeZp6uFmiT7zQjm64XmeY0ua58fVq7STUR71ce0rfw8+tohlpzxPSJLexyn8OcPerLfpo\/ZekyDTadis4YUXXgh2mWGJXR4gAVRaSWvzrl27BMJg1Fw3liAsbTn11ktbD7UDAmd55C9+8QsNyv2Ytg6uHveDtfFM78q9AncMcO3jWYl7DyDKb33rW7J\/\/\/7CvC93qigQIHsI6HWWR69JkHW2\/fv3FyZPs8QO0njjjTeEFy5LkKopK63NTCJHyNvHuqWtB\/YjEPh9990nn\/zkJ7n0QtLWAWKZNGmSMIULw7kv1a5yIl29JG090LvrrruCjUp4ptidiQ1K6mVXLfLlXaYVRF5z5szhkLl4T4K8WH379g2AYTkdK0p8\/qlPHsSu2MzDykPLwxtUNuc\/1dQDD+TgwYPyxBNP5Gx15+LT1gHs7733XqEZTDOSI95X59zyu0pbD5wFrITUIRc8Rt4ZwnwVnncmclf50alpdbwmQR7OmtY2g8y6YjP9OEePHhV208nAxFRFVFOPnTt3Covgo7ZKS1VYnZSqqcMrr7wSbB5Mcxiv0KfmcNp6sBBh7969wgbGEOLChQsFr7ZO8DZMtl6ToO9fsainoFqbIcCo5YJReWcZlrYe2I9d2pTk3BdJWwfsnT59emkV04wZM4SPki\/eYNp6cC\/wxiFCyPzVV18N+tqon0k8Al6TIJ7F1atXRZu\/fBGHDRsm2jyOr1Z+MdXYzEPLV\/v1118vvYD5Wd655LT14EWjQ5tm5OzZs4WNb+fPn1+2J2Tn3LO5SlsHSIamI03IbCyrrpS09eBe4MXiEVKCb2SOTT6K9yR45cqVYKSLB5RBEQZH6EfwEUxs4oG9cuVKRZuVAPlq60NLel8kbT3oy8HrQBiVZA0ogyRxa8CzrF\/aOtB05GPLxh88Z\/w+jksmWdocVVbaeoRJD1L03WmIqm\/WYV6TIOTArjI0F3lQGTjwvY8jyWamAegUHx7QU6dOBZsh4EUhjJLxEmb9EESVl7YeUWl9CUtbB\/R0NyOeM+xfs2YNBy8E++LeA\/eZokuC6T1srMDzRJN+06ZN4rPT4APAXpMgAOFRsLsMngZeB2G+S5zN2K8kzjl1coVRMp8e2DT1cO8F+njrHN3wPM+xJer5AX+9F9jn6vl2H8L2YTthCOduPTjXZ+rw4cOlHztD13cJ1yUre70nwayA8Kkcs8UQMASyQ8BIMDusrSRDwBDwEAEjQQ9viplkCBgC2SFgJJgd1lZSHAIWbgjkiICRYI7gW9GGgCGQPwJGgvnfg4axgOk9TPNhegZTN9JWDF0krX4WeszjpB5sw8Xa6HqUSb7kTzmUV48yLM\/KCBgJVsaoEBrMP4SAIKJKBlejWykvN15\/MpUj0x3cuCKeM9+upaWlDqt5bqPBtBzyp5zbIfY3DwSMBPNAvYHLtBUKDXxzG7RqRoIFurF4eXh7NJ8QbULSlGLXEzyw5ubmYL9Ft6lVSZeNAtg+Cj2E\/OJgidMlDWVjw9SpUyPXDrs20QzkWsth2ZrWLRxH3tilgidLOtLPmjVLNB16hIOL6nJO3bCbONKQP\/EcuSa8kpDPk08+WSrLzTOcFl1Ew7EXG7l\/GmZHfxAwEvTnXlS0hDW5LB1kRQB7xrHonxef5VJsw0Wzavv27UE+7PLMjtzosqb3rbfeCogpSnfp0qXCcivVZXfiKHLgJXZ1KYsljeiSL9fDhw+XqA0hICHdJZxysI1rwjEY8ly1alXwW9Xs6IL9lEf85s2bg92SSUcZe\/bsEcJJ98EHH5R26MYG8GC5GPkpRughpKFMyiYvjlwTTnwloV5sT0Va1haDBTZWSmfx4jUERoJe355y43gRIR2W17G8ixc\/rBWOY1POuG3v2TQAL0x39aWf6vOf\/7zQVxXOF112icHTIw7SHTNmTKQu8a5ASFzr2lzsdpd1QXyUjQ67unBEWDd74MCBUr8cOwkRrsJO1moPYazJhqBIBw6QFuEINtBcx26uOXJNONeVBH0EvQULFgQ75oAJ1ybFRcBIsED3jnWhixYtErasqtScozmGDoJnB1lEVRWCPHHiRKeNHNgaK0w2pEX3rrvuCrZv57oaIT8Ip9pt0PC0aEpSD+R73\/tebLHo4h27JOoqYwMeIkRMXhy5JtzVizvHC4dYiWdnlzhMiTcpDgJGgsW5V4GlECHNMQTvSZuNQeSdP3iKNH9pBqP30ksv3YkpP\/Ay481BBuiqRI3uovvRRx8JZFieU3IIxNSVn0bALtJxxDaaoHElQVAQVRypYQOeHJ4feamAaVyebjgEC9ESBgZx3jXxJsVBoB4kWJzaF8xSvDtEzeal5qXn5dcwPfKC8qJyzfb3XHMelpEjRwq\/iYL3Rxz9Y3T605nPtSvosl+gNpUhJrxItznq6rvneF1cQ0AcIWr2huTIdZJcvnw5IF4IiL3+knTdPfXC+tighEoelM3gCP2IXFcS6qrNXzAFCzAJp+O+0C8JltjAzjphHbv2BwEjQX\/uRUVL2OMOb4SmHMIAAQMiJISIeElnzpwZNFfxEhmtRW\/ChAkyYMCAgEjCuuzazZ5z5IUunhJ9alHeEWTr6pI\/gwval0fecUIfne6JRzk06VevXl3q64tLhz14quhTR+qX5I3S14j9pEN\/4sSJpayjbKB7gTQlpYQT7GAgBfshObAAE8iOUWpIleTavxplQ1gXfZN8ETASzBf\/qkrnhWMwRJtx7sACRMS+eRpGc1b1vvrVrwrh+rKHdSEH4lWftHGGhXU1T\/Q5Jx90uA6LlqvloI8O5SGcIxAw9aS+COekoX7UBc+KvBA9J50Keak+np7bF0ka8iEeoSxNV+mI1039SMdR68nRHbzhmnj0KIsRdOpAXYhzdSuVafH1R8BIsAYYWxb+IEAzXrsMaIrSfIa8ICB\/rDRLfELASNCnu2G2dBsBmqI0VWmy0gdIhnQjcKxW6POkSa3N3GrTV9InX\/KnnEq6Fl8\/BP4fAAD\/\/3yCfPMAAAAGSURBVAMAqSEZd+rvXLEAAAAASUVORK5CYII=","height":193,"width":321}}
%---
%[output:78c0e0e6]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUEAAADBCAYAAABRyQJdAAAQAElEQVR4AeydDYwV13XHjx1XfBgT7EJgwbgQU5EIA0WEgEWKm9YkSJTSIFkthAaJSmtTU4GMgSLFhIQWGrAtUHENSEVCRdCUipgSIltGbbGyqjG1YoNJjLrNbsy3SQkBAosaxeU3cJ7vzpuZN2\/3vZk7bw\/i7Mzce+695\/5n5j\/nfr67P7J\/hoAhYAj0YATuFvtnCBgChkAPRsBIsAfffKu6IWAIiPQ4ErSbbggYAoaAi4CRoIuGnRsChkCPQ8BIMOdbfvz4cRk\/frw8++yzdbHkww8\/lMcee0zmzZsn165dq0sZbqbUg\/pQLzc8i3PqRz2xIYvyalnGoUOH5NOf\/nQg1dqPfl6YhzHQ523r1q3hKG+vjQS9vTU1MiwiGwjqG9\/4RkRM7YOyLOvNN98UZMaMGakqUq1tvNiQVarMq1CCvHfs2CHDhw8P7H\/++eerSO2X6qc+9SmZNGmS7NmzRyBEv6yLtsZIMBqXzELHjh0r7777rtTrweehPHz4sOzevVv69esnvPjz58+XX\/7yl3WvY5ZlUZlXX301IJJx48ZxmSjV2gYBbtiwITFPi7yNAB+hU6dOybFjx24HeP63oUmQZoI2McLNBeIIe\/LJJ4MmCHo86O79QodwhGYWX2zieYE0LeHEc0048VFC3uipqEdBGtJSFulUj2t0ORKueoQhGs7X1m3uYiM2EUYcwjlh7e3tsnjxYrl69ars27cvaCYTT\/6IpkWXc8K0XC2PMM6xAcF2dAh35dKlS5FlUR72kBbhnDDSUiZlE45oOWBCPAJuxCFu2eRx9OjRwAuB+NEljPzRRTgnDInCgXB00EU4J4wylQCbm5tLXReEo4e4tlB2WMiH\/NBFqCf1RcgTDxbimDJlipBv2vSu3tNPPx35LFMG5VEu4trKveOaeOxDsBUb0EVmzZoliMZRpqYjHkGfcGTw4MFy3333CR8lrn2XhiVBXqDXX39d9u\/fLz\/5yU9k0aJFMnv27E4PGGQwYcKEIH7FihXCg643003PA3rmzBlZs2ZNp\/v5\/vvvy6ZNm4IyiNi5cyeHMuElJu\/t27cHZc2ZM0eWLVsWeGVlyk4AduMh8sDhvU2fPj1ITz6QGPk66hVPBw4cKFu2bAkeUGw4fMtDVMIgMZ7itGnTgiYZdSaspaWFgyxYsCA4urhgXxSuKD7wwANlZfXt21eWLl0qw4YNC7wE7s3ly5cD3Emza9euoGzqhxdx9uxZgkvCvYEwuFeUDR6QGS\/thQsXhLxGjRpV0gdzLqiLWxZ1DuOQZNvjjz8ulEle2MY9SbIFPVew74knnijVm7oRP3PmTLl+\/bqQJ+SnzWHKI14lnJ66nDhxotPzyLMMkYEL9\/bll18uPV9r1qwRnl9wQAYMGCDr16\/v1EdM\/N69e4VngnLXrl0r2IStS5YsEcojHAk\/j2DjPs+QIGVw\/yBg0vgsDUmCPDR4BbwkNDe5ATwYPGTu14mv1dSpU4kWjlwTr+nHjBkjI0eOFF4a+jnIk7ggwa0\/hBGXdNN5CN54441OzTReIprAatutrMr+06TQQIiIh1zDeEl42J966ilVqdnRxUFtVxyoOxhUwjXOGEiWZjkfDgiAjxL1Ql\/L4h7RnEV34cKFRJWEe8M9wkYCwQPviRcVEtS8iFMhnpeZeDAHe41zj5QXZ5urp+dJtqiOHrEPO6gP5SCcE0ac6sUd0UGXDxRpeW7CdXFxGTVqVODtU2fypM4HDx4MPkBgQV6Eu6LPMmHh8kiDEIeEn0e9H4QTzweFDx3ECskT5rM0JAly8\/EKooCP+zq5RKY3jq8mLyTuPp4XeZJ3VL5xYeTFw8BDwcMRp5cU3traGnhv2JikV4s4XjBIDrKj\/ngA+vJRdzCIKicOV1cXosNb4YWaO3du4EHz8qKTFieIDvLknuAVkhZ8OIYF71XzR5c0cd5zkm3hfPU6rS1x9pFPUhzxiOpAblxXK3jvPMdDhw4NPG0+NEl5pC1PMeV+gIWmS8rbx7iGJEHIAnc8CnAeBL6m4Th9wYlXsuJl5auI14Xw9YUkwmmTrskLAoQIedGTdOPiePh5yLAxTqeW4eph4bGRr37pu4Ir6VXa2tqCZhVeediLTYsTpEZzkPuhEs5Ly+Necc\/Qo8lGuNtM5FolyTbVCR\/T2sL9C6fV66S4sE4Myaha5FG9d55lmsWRSqFAtalSeTTjwVYFjzOUVSEuG5IEaaLi3tMnSP8FdwJPjmYALzjXCMSi\/Xi48lwTr+nxgng5eJDoFMaLwWMgbVqBcPGkKBtCJR3eCJ3RahthSQIJ8cLRBEOPdKQnnzB5YC92oxclSUSm+ngNeAs\/+tGPBK8QMiFOcamEK7pIXFnqNYI9mKMbxgmcmTZCnAr3Bn3uFWF4OOAAHpQFRoQjpOd+cd+4fxAl5MvHEV2Ec3RdibKNeCUGzpEkW4h3RfGkPtiFcA7GxLm6UefooEu3CmmpD\/Wi\/lH6UWH6Edb3IEpHw8Ll0SJAND78PPIc4mXTT4oOH3vK4+PP80mYz9KQJAjgfJV4gXHVuUF4AHgQ9KcRj\/DSNDU1BSNqdKLjLWi8m56vKPp4RrysnFcjvIDkrc0HynrhhRdEyaVSXugxaAD5UBfqRN3IF3u0fwk76fCnDy8uTyUyXgYlkLCu6hDOy85RxcUFW6JwVV3NR8sinIEUXihetJMnTwq2KvEw+EMdwIkXbeLEiSQpCfcG7wP8KJt8FUclNfVewAVdXkTyRB\/8wAe7ED6U5AEOFJJkG\/ZCRNgGuZIn+UfZQl6uUBaDDhAD+SDE009HHOdJgo6bnrKpVxrPjrTPPfec8BEmHfP3OGILZBVVrqbR+0TZpFHd8PMIBjzf3B90aLHQbUKrivtAmM+SCQnyxUKigOArzkPIQ4rwheNLF6VbbRgvrLrqNIu4eeE8vvSlLwUjruhBKm68m55RMx4O4smH\/IjnmnDi6ViPu+nkTRkq+sCE81I9jSd\/RPU0vZZNHLoavm3btmBOIPZgF8K5axtp0acO5EseYVEd8o6Li8qDdG6+XLt6Wj\/CDhw4IIjaBnacE0cePBeU7Xph2EO8CtfoUE9Ijb5MfX7c\/NAnT7e+1dhG\/uBIPthI3pTNtQrX2BIlbnr0NQ90yYtr8kePsLAQTjxpEfRJhx71cOumGKs9HEmDkAdpOZIneJCWPMhLxU3Dh4ZwiFc9O01HnghlooNAgnjs4Q8ocbWQWudRdxLEReZrG2c4gOEN0FQETL05cfoW3rgI8KHkQ0jzilri1eGt4+VxXUl46fBAeKYq6Vp8PALqmKhDgseI55jWs6PbBq9ZPd74kvyIqSsJ8kWm78N1pcPV5kFPC244rV03FgI0qXh5aF5BhhzxQvA60tSU54wPKn2NafRNJxoB8KZrQJvQ4IoXmKb5zTuPN87oP55mdAl+hdaVBHmIGRSA5OKqDQniKfLQI3iOcbq1DMf9pxnADa9lvpZX1xHgpaElQItAhWZZ2hxpHtLU496mTWN60QjQvNV7wBFcwTda++NQvYek\/zjU77O6kSAuNf0CdHbHQcBIF53ieAAATUezO\/OcdCqnT58O5jjRWWtye6MAw8FwaJRngPdb3\/Wsj3UhQciNUTiWNCV9PYjjC6NfDdxumjM6BULBAKDly5cH20ExMmcyz7CYZxg00nvA+817ru98lse6kCBz1fhCMZWDJi7NXYSO7zSVc0cD0QecI0eOyMaNG4ORT4izaML6S+pS5DqAudVjt1fPYCPcD+rA+817zjuStdSFBOlno7+NJi7CJFUk3FdDJyq7U9B0puIQJ6NQcaNKdM7iLRZR+CCwjIu5b4n2T5kSLFz3Vcfq4df9aYT7MXnyZF7\/3KSMBCEmhsbx4NIK+mlrQP5KfHSirlu3Tug3pCz6A2lGE542v6LoDRkyRL72ta8Jx6LYHGUn9ls9opDJJ6xR7kc+6N0utYwECe7fv3+wuB0vrpKwCgN90sUJHiBCPATHBFm8Ra45qtfIkWvCTQwBQ8AQyAKBMhIMk1QlIyAtSK2SnsUbAtEIWKghkC8CZSRIc5XmLSNPjPLma56VbggYAoZAfREoI0E8QWZ7M0jBAAV9dbqMqb6mWO6GgCFgCGSPQBkJYgLz9rQvkFFdVn5AhniIeIromBgChkCXELBEniEQSYKujQxoQIh4hoQzdQNCTDvnjzQmhoAhYAj4ikBFElTDaSbruk5GhNkLzrxCRceOhoAhUFQEUpMghEdzGC+QCZpscaR7ixW18ma3IWAI1B8B30tIJEFWcrCxJcRHM5i92vACaR6zhIq1v75X0OwzBAwBQyAJgTISDHt87ATD7i4Qn01mToLS4gwBQ6CICJSRoFaCUWGID6lmTzdNb0dDwBAwBIqAQCQJsgyOxf5pKkCTmbXAqmtHQ8AQMASKhEAkCV65ckUY\/KAvsJKgh36RKm22GgKGgCGgCJSRoDsVhqZwGmHqjGZoR0PAEDAEioRAGQkWyXgvbDUjDAFDoNAIGAkW+vaZ8YaAIdBdBIwEu4ugpTcEDIFuIdD2iRFy+Y\/\/QZ469JHsOXq+W3l1JbGRYFdQ69FprPKGQH0Q+OBShwy\/v3d9Mk\/I1UgwARyLMgQMgcZHwEiw8e+x1dAQMAQSEKhIguwuzS7TzBfkyJZaTI5meV1CvhZlCDQKAlaPOiPwwaUbdS4hOftEEoQAm5ubZdq0acL6YbJ65JFHZObMmbJ06VIhnjATQ8AQMASKikAiCV6\/fl3YQGHq1Kmd6sc14cR3irALQ8AQMAQKhkAiCbJ6ZPTo0bJ+\/fqS1\/fee+8FvxNMOPEFq6+ZawgYAhUQyDP6oQc8HB1me\/2FCxfKM888I\/QH0i+4aNEiITxPsKxsQ8AQMARqgUCiJ6gFsJWWu4aYH2LSuFoe+d0SpJZ5Wl6GgCFgCCQhkIoEkzKoVdyhQ4dk3759tcrO8jEEDIGCIMAk6TxNvTupcKbB6O+KMEUmSth+nz0Fk\/KpFEc5O3bsELbwr6Rr8YaAIdC4CHjXJ8jAx6RJk2TFihXiNoe51p2n6R9k4KQ7t2XDhg3BNJyhQ4cmZnPkyBFpb2+Xjo6OQsrNmzflxo0bhbTdxdzq4dfzV+T7wfv8waWO4L3PgwApOLE5jIfGT2syJQZlFa4JJ55zpstoXLVHvEjSz58\/v2LSzZs3y86dO+XcuXOFk\/Pnz8vFixflwoULhbPdxdvq4dezV\/T7wfv8n8dbK7779VRIJEE8QabCsHs0fXYYwpFrwolvaWkRfn6TuGqFydZbtmyRxYsXS5pfrtu4caOw7X9TU5N0WXJKO3jwYBk0aJAMGTKkcLa7WFs9\/Hr2in4\/vjjrT+TXfQcG1JHH5gkUnEiCKDAVhp\/ZXLZsmdAnyJFrwvHiDh48KJs2bUK1amlrawum3UCq5M3ACBI3Qjxs2DAZMWKE9O7du3DSp08f6dWrV+HsDmNt9fDr2Sv6\/bi7\/5ASb3xh1IDSeZYnFUkQzEQfIQAAEABJREFUY8aOHSv83Cb9ghy51vADBw4IHiHX1Qr5kB\/5IvQzIhBstXmZviFgCBQPgW+\/1lYyeurD95fOszxJRYJZGmRl1RoBy88Q8BOBH7Relpb\/uRwYd8\/PTso9P3s\/OM\/6T0USpA+QpmpYmDrDwEgtDcYDRGqZp+VlCBgC\/iHAiPDif\/pxybDe7+8vnWd9kkiCkNzatWvlxRdflOnTpwt9gceOHQvm882dO7fLzeCsK2nlGQKGgD8IQIB\/9Pc\/FI5Y9elf\/\/SWF3iS01wkkQSxqH\/\/\/vLwww8HI8CMBDOKu2rVKmFABJJEx8QQ8AgBM8VjBCA+lwCZG7jlTz+Tq8WJJNi3b9+A\/JjLM2PGDGFS8yuvvBLsKnPlypVcDbfCDQFDoDgIQH4MgvzOX\/9nyQOEAP\/1LybkXolEEsTr2717t9x7773CfCQ2VmU3Gbn1D0+wq6PCt5Lbf0PAEOgBCEB+T+\/5sUB+336tvVTjlV8eIe98\/VGBCEuBOZ0kkqDa9M1vflOYzqK7yUCMEKTG29EQMATyQ8C3kiE+9fogvz1Hz5dMhPTw\/lZ+eWQpLO+TVCSYt5FWviFgCPiLAKQH0anHB\/Hh9RGuVkN+L839bOD95TUpWm0JHxNJkIEPflSJlSFuQq4JJ94Nt3NDwBBofAQgNyW9WS\/9MGjqQoCEEacIQHxzJw0RPD+avpxrnE\/HSBKE5Ngii62tTpw4IbqsTecKcs16YQZOfKqM2WIIGAK1RQBSY1IzBAfR4eUhnBOmk5211DDx4f3V3PPTwmp0jCRB+v9YzsZ2+mPGjAnmB7KszRXrF6zRHbBsDAEPEIDsEIgNgsPDe+CZfw+8PKa0EEYcOq65SnqQnXp8nPtOfG4dIklQFRj9ZW0wpKhhdjQEDIHiIgCJuZ4dZIdnp6JkF\/bwtMZxpEdTt0jEp\/XhWEaC9POxJE6bvnFHdNAlExNDwBDwB4Ew0UFskF2UZwfZoR+2HrJDIDc8O7y8Sy9+MRjY4JrwopJeuK5lJIj3d\/jw4U47SbvNYD1HB91whjW\/tgwNAUOgEwKQlnpze394UdYc+pl8ZdvxoOkaRXQ0YyG7TpncuYDoEEgNgeAgPAYyEK4JbxTCu1PtTocyEuwUaxeGgCGQKQIQHPKD1ssCeTHfTj05mqxhkluyt1UO\/PiaHPnptdJKjCiDIbqpDw8QCA1ic4lOyY5w4huZ8KKwSUWCW7duDTZU1aYx11GZWZghYAjEIwC5IVEE5zZXITsdjGC+HWSIJ0fauNwhOQQSQyA0iA7RZuyBpycI4cRDdOjH5deTwiuSILs879mzJ9gBmqYwI8ZcE96TgMqurlZS0RCAnBAlN0gL7w2B3CA19eA4jyI4SC6p3hAW4npzENo\/L\/yMvP2XI+TNZyeU+usIV6KD7JLytTiRRBJk4IMfVOJ3QLT\/jyPXhBNvIBoCjYoAxIbEkRsEF0VukB9EiEBu5JGEEeSGKMFBYBAZXhyinhzNVtebQ+8Lo\/LZjTmpPkWLSyRBCI8fVOKHkJTwOHJNOPFFq7DZawhASmev\/ErePtMhu986J9rvBnlBbHhracgNgquEJuSGKMGt\/PKIoEkKuSFRBAcB3ia4AfKFnH53o1K9Gik+kQSpKDs9s4Eqq0foE+TINeHEmxgC3USgJskhNqSS16bk9rm\/PSrN+87Ls6+03yLB9mAQIq3nhsEQG6LkBmlBXgjkhtcWRXArvzwyGJyA3BDyMskXgYokiHlPPfVUpykzXBNuYgjUCwEIDXFJLc5jU2LDg9P+Nrw6SA3BY0PS2AqxIdWSG+QHESKQG3mkKc908kcgFQnmb6ZZUHQEIDQEUkMgJwSyQmiGIhBZHKnpSCnpIDXyS4MLhIS4xLbsDx6UNY8PlH3NY0sL\/KM8NyO3NAgXW8dIsNj3LzfrISAEQkMgJgRCQyA0JIrUkrw18kxTKUgNcYltZai\/La5JCrEt+4PhMuuz\/WTKiH5Bvxt5pSm3FjqWh18IJJIggyAsj5s3b55cu3bNL8vNmpoiAPkgLqGFm5+Q2pTnfygT\/65dOEJwEBoC8SEQIYKnhpBnGkMhIQRSQ2hWIhAWQj8bkkRsK0P9beSXpmzT6dkIJJIgo79Mh2E7rXHjxpUmTNtkab8fGogHgdAQSAkJkxokhqRtfnaF1CAil9RWpvDWdBoI5AcRIvSzIeTnN\/pmXdEQSCRBKsMOMmyrxURpFcLxEPEUOTepLwIQGpJEaHhpUYTmemlRfWrkm8Z6yAeZ\/Fv9ZOKw3sEIJ+QEUSF4aUjYU+PaJbWV5q2lgbuxdDyvTUUS1A1WmR6j8vLLLwseIp5iXP3cdEmE6eqRf5JuXFlFCod0kLSEVg8vDbwgNCTKS0site8+OVa2zxkiL3xlZDDfDSJE8NIQ8iR\/E0OgKAgkkiCeHhOjBwwYUFo2hzeIZ4iHGFdJ+g8hyV27dgVTa5hXuHTp0sh+xQsXLggbtx47dizQLeLuNJAa4hIb\/WMIHhpCH1q4L414JOyhVdPs5B5APAiEhkBKyMo7TU8lNbwyRkARzpEoL420EBpCvpRhYgg0KgKJJIinBykhnKcFgV+i27ZtW\/ALdaSZOnWqXL16Va5fv85lJ2ltbZWhQ4cKaTpFeHbhkhzEhUBuNEFV3KYnfXAIhIaQPk2VIB2VJEKLanpCaAikh6y80\/RUUiPfNDaYjiHQkxBIJMEuARGRqKWlReJ+kwQS3LdvX2nQ5dChQxE53A46cuSItLe3S0dHR13lJx9eEwQSg+zCJEc4kobchva\/R5DfGXKPPDLwLvnK2AGBPP\/HIwRhATzCAviz6x4NFsJzjuz9888GzU6ankseawrSkf5zD\/YW5FN9pa44xOF88+ZNuXHjRi5lx9nUlXCrR33fozT3hPf5zJkzt1\/wnP7WlQRpTtPHt2HDBlm4cGGZt0ez+ezZs7JixYqgKbx9+3ZZtmyZ0E8YhcfmzZtl586dcu7cuZrL2ydPyfZ\/+2958h+PC0uqEAgQsovz4iA3BgmYb4Yw+Rahz+zAggeD3T04In\/\/R4Pkb373E7L80T7y9d8bIF8cLoE8fG+HIL9x8+c1r1M9cDp\/\/rxcvHhR6MaoR\/5Z5Wn1qP071JV7x\/u8fPnyqNc9s7C6kiBN6MOHDwf9iWvXrpWwl9evXz\/hB5t0GR7rkukfxHOMQmDjxo2yYMECaWpqqpn8X6\/75a\/\/47LM2nla2KGXDSrDZdOMpElJExOhKYrX9l9\/NUlofm77s7GCNP\/+bwvyh58bKRNHD+9k4+DBg2XQoEEyZMiQTuG1rEsWeVk9avfs1eJ+Ff1+8D4vWbIk\/Mplep1Ignhy\/L5w2DPjmnDi01jLT3MOGzZMaPqm0R81alSkGnmMGDFCevfu3W358Fb35LLvtgWTfr97\/HKn8pT0IDsdRID8IEKEAYOPbUhnS58+faRXr17dtrvacmutb\/VId79rjXtcfkW\/H7zPkydP7vT+ZX0RSYKQ3Pjx4wXPjInSs2fPLvXZMY2F67g+PioAOUKS5MN1W1ubnD59Whgg4VolrMeGrfQPMDFbdWp9pGnLpGH6+Wjqav4Q38pbo6mMmCKQHmSn8XY0BAyBxkQgkgSZ\/sI0GEiJ5un+\/fuDPjumx6jQjKU5GwULzeB169bJ\/PnzA\/KENFevXh2MFrvEF9ajP5CpNYRH5dvdMAiQEdxvv9Zeygryw+OD+FbeGk3luhRpJ4aAIdDwCESSoNYaMurq7w4rkSppPv7440G24TxdPYiX60Cxxn8gQLw\/jmQN2a284\/mZxwciqcUUDYGGQqCMBPHUGNGlOcxABuc0gcNCOLpFQAPigwDVVubf4f2tvOX5aZgdDQFDoGciUEaCeGqM6OKV4b1xrt6ceyQcXd9hCxPgylveHyO6eIK+2272GQKGQP0RKCPB+heZXQkQIH2AWiIEuNK8P4XDjikRMLXGRiCRBGnu0uwNN4W5Jpx4X+GBAJnszBEbmdpiBAgSJoaAIeAikEiCNHdp9rrNYM7nzJkjzz33nBDvZubTOUvaEGyi6cuUF85NDAFDwBBwEUgkQVfRPWeWN0vYfPUE8f7wArEZAmQQhHMTQ8AQSIFAD1PpEgmyrO3KlSveQqUEiIE0gyFCzk0MAUPAEAgjkEiCeHr0\/dEH6AobIvjaHMYLdJvB1g8YvuV2bQgYAi4CiSRIn19UnyD9gkyfcTPy5dz1Arf86Wd9McvsMAQMAU8RuFsqGIY36K4D5keWfP31OdcLZEK0rQSpcHMt2hAwBCTRE2S\/P7bFnzlzZrDuF7zY9mratGmyZs0aLr0S1wu0ZrBXt8aMMQS8RSCRBNkOn23xw7u\/cH3y5EnBS\/SlZq4XyECIeYG+3BmzwxDwG4FEEmQfQLbMYhdhtxpc+zY6rIMh2JnsBaJhYggYAobAbQQSSZCtsmj6Njc3l3aFZlMFrvkFOQZObmeT\/9\/db50LjMALZFpMcGF\/DAFDwBCogEAiCZKWPkD2E2SvP6bJQID8FgjhxPsgP2i9LOoJMiDig01mgyFgCBQDgYokSDXY449dZZgag\/g2Pablf36OmYHMndQUHO1PCQE7MQQMgQQEEkmQgQ93ekxCPrlG4QliAE1hGxABCRNDwBBIi0AiCdLnN3r0aGGZXNoMs9ZzR4WtKZw1+laeIVB8BBJJEE\/w6NGjwjI5+gNdYTkd8XlDsOfo7QER7LCmMCiYGAKGQDUIJJIgnmDcsjnCia+msHroWlO4HqhanoZAz0EgkQR9h8FtCg+\/v7fv5pp9hoAh4CECZSRIE5embhF+aAkSVExtQESRsGOPQ8Aq3C0EykiQJi5NXabEMBWGc6bFhIVwdLtVejcTu1Njpj58fzdzs+SGgCHQExEoI8EwCHiG7jQZn3aRsf7A8N2ya0PAEKgWgUQSzGIXmePHjwtNb0aeaYZDumkqQVNYV4lYf2AaxEzHEGgUBGpbj0QSrPcuMpDsli1bZNeuXUJzm\/XIbN1FeKVqQoKqY\/2BioQdDQFDoFoEEkmw3rvIsEHDtm3bSnsVskUXW3dBvpUqYv2BlRCyeEPAEEiDQCIJQlJZ7iLDyhS27oJ8o4w\/cuSItLe3S0dHh2h\/4ND+98jnHuwdhBHus9y8eVNu3LhRCFuTcLR6dHh1D4t8P3ifz5w5E\/W6ZxaWSIJYwW4x9d5Fhn5A+gNZmbJw4UKBfCn7tnz8d\/PmzbJz5055++Sp0q4xTffdI+fOnfNezp8\/LxcvXhT2YiyCvXE2Wj38etaKfj94n5cvX\/7xS57DWUUSxKZ67yLDVJvDhw\/Lm2++KWvXri3tXUjZrmzcuFEWLFggNz7RrxT8e58ZKE1NTd7L4MGDZdCgQTJkyBDvbU3C0+rh17NW9PvB+7xkyZLS+5zHSSoSzMowmsHDhg2T1tbWyCKJGzFihLx9pqMU\/9joQdK7d2\/vpU+fPtKrVy\/v7ayEpdXDr2et6PeD93ny5Mml9zmPk1xJkGawOwE8fi4AAA9CSURBVAexra1NTp8+LQyQJIHhjgyzfVaSbgHjzGRDwBDIEIFcSZBm8Lp162T+\/PnCPMHZs2fL6tWrS6PFcTgoCUKASJyehRsChoAhUAmBXEkQ47rS32iTpEHOxBAwBGqBQO4kWG0l1AsknU2SBoXii9XAEMgTgYokyK\/L0VQNC1Na6NPL2niXBB96oE\/WxVt5hoAh0GAIJJIgJMeUlRUrVgTL2ljapsKUFvr0ssbDXSlia4azRt\/KMwQaD4FEEqS6\/fv3rzhai15WoitFKM+aw6BgUjgEzGCvEEgkQTy9mTNnBqs0vLL6ljEPPWA7Sd+Cwf4bAoZANxFIJEGaw3v27JF9+\/YFU1jcfsE8+gRP\/byjtFzOmsLdvPOW3BAwBAIEEkkQT5C+P+0HdI+EEx\/kksMfawrnALoVaQh0CQG\/EyWSoG+m28iwb3fE7DEEio9ARRJkg9N58+YFzWGObHLAUjeayllX3x0UseZw1uhbeYZAYyKQSIIQYHNzs7Cn4Pbt2wMEHnnkEWGwJO0O0EGiGv2hT1CzsoERRcKOhoAh0B0EEkmQHZ7Z6Tm8oQHXhBNfXnj9QrQ5DAEi9SvJcjYEDIGegkAiCTLwMXr0aFm\/fr3gFQLKe++9F2x4QDjxhGUlv+77m0FR1hQOYLA\/hoAhUAMEEkmQ\/J9\/\/nlht+dnnnkm2PSUfsFFixYJ4cRnJb\/uO1CQrMqzcgwBQ6BnIFCRBIGBH2F3p8ew5T7heYln02PygsHKNQQMgRogkIoEa1BOt7PQpjAZPWQbJwCDiSFgCNQAgcKQ4K8Gji5V1\/oES1DYiSFgCHQTgYokyIAI\/YDukrlnn322m8V2L\/lDtm64ewB2M7UlNwQaCYFEEoQAmSc4dOjQ0lZax44dk7NnzwrESHxWYPxq4GeCoiBAJLiwP4aAIWAIdBOBRBJkHiDzAflZPC2nX79+smrVKiGceLF\/hoAhYAgUGIFEEmQeIL8J6s4TpK78YDKrRojnOgvRPkHrD8wCbSujEwJ20dAIJJIg64PZWZr1wuPGjQvWD9M3yNZaGzZsKF2zrVY9UXLnB1pTuJ5IW96GQM9DIJEE8fTYMsudIxh1jk49oTMSrCe6lrch0LMRSCRBH6F5yOYI+nhbzKaGQqBnVSYVCW7durXU9KU5zHWWMNmPK2WJtpVlCPQsBCqSIHMC2WKffkGawhy5JjwJqvBPdXIdpX\/8+HEZP358iWTpX6QvMkqXsIdsjiAwmBgChkCNEEgkQcjo5MmTsmXLFqF\/kDI5ck048YSFhfAdO3YEGy5AnOxFyAAL4WHdCxcuyJgxY4T5h+jSv0gZrp67maqRoIuMnRsChkB3EbhbJD4LyIgtsxYvXixKYBy5Jpz4qNSE7969u0ScjCyjB9FxdKW1tVWYjM38Qzc86twIMAoVCzMEDIHuIJDoCZIxW2bNnTtXpkyZEjRZOXJNOPFpBG\/vo48+ksGDB5epQ4JMuaGvEYlqNuuO0qdPn5b29nbp6OgopNy8eVNu3LhRSNtdzK0efj1\/Rb4fvM9nzpwp44UsAyqSIMawdRZNVRWuCU8jLK1jsjXL7MaOHdspCXEswVuxYkWwLI9m87Jly4R+Qlfxg0sdweXd1\/83+A3kc+fOSdHk\/PnzcvHiReGDUDTbXXutHn49e0W\/Hyy8WL58efB+5\/UnFQl21ThITtceRxEnTWCazRqHl0n\/YEtLS6lIJUACHh07SljC19TUJN2QXNJi\/\/e\/\/30OuZRfK7yogNXDn+ev6PeD95lVadQjL6kbCSoB8iNN1TSdAWLUqFEcyoQ+Qb5877zzjrxTMDlx4oR85zvfkddee61wtrtYWz3e8er+Ff1+8D4\/+OCDZe96lgF1IUGXANXLi6oUgyz8fKc2f5l+Q\/+ADqSE03z3H7cFu9fQtC6aqMu\/efPmwtYBzK0e87y6f41wP6jD5MmTJS8yrAsJtrW1CV8od32xDnq4xMco8rp164IfbiKe\/kCm3xCuBIj3d+nFL8o7X39U\/uVvnxaazya7q8LB8DK8fH8GNm7c2FgkyADIu+++Gwx26GAKR36rBII7cOCAoAPRcVRdjlwTHhbIkD5DkynBSL3hYDg00jOQlxcIz9TFEyRjE0PAEDAEioCAkWAR7pLZWB0Cpm0IVIGAkWAVYJmqIWAINB4CRoKNd0+tRoaAIVAFAt6TINNndJeZSjvXVFHvuqqmsZlpREw5YVQc4ZywuhpWZeZp6uFmiT7zQjm64XmeY0ua58fVq7STUR71ce0rfw8+tohlpzxPSJLexyn8OcPerLfpo\/ZekyDTadis4YUXXgh2mWGJXR4gAVRaSWvzrl27BMJg1Fw3liAsbTn11ktbD7UDAmd55C9+8QsNyv2Ytg6uHveDtfFM78q9AncMcO3jWYl7DyDKb33rW7J\/\/\/7CvC93qigQIHsI6HWWR69JkHW2\/fv3FyZPs8QO0njjjTeEFy5LkKopK63NTCJHyNvHuqWtB\/YjEPh9990nn\/zkJ7n0QtLWAWKZNGmSMIULw7kv1a5yIl29JG090LvrrruCjUp4ptidiQ1K6mVXLfLlXaYVRF5z5szhkLl4T4K8WH379g2AYTkdK0p8\/qlPHsSu2MzDykPLwxtUNuc\/1dQDD+TgwYPyxBNP5Gx15+LT1gHs7733XqEZTDOSI95X59zyu0pbD5wFrITUIRc8Rt4ZwnwVnncmclf50alpdbwmQR7OmtY2g8y6YjP9OEePHhV208nAxFRFVFOPnTt3Covgo7ZKS1VYnZSqqcMrr7wSbB5Mcxiv0KfmcNp6sBBh7969wgbGEOLChQsFr7ZO8DZMtl6ToO9fsainoFqbIcCo5YJReWcZlrYe2I9d2pTk3BdJWwfsnT59emkV04wZM4SPki\/eYNp6cC\/wxiFCyPzVV18N+tqon0k8Al6TIJ7F1atXRZu\/fBGHDRsm2jyOr1Z+MdXYzEPLV\/v1118vvYD5Wd655LT14EWjQ5tm5OzZs4WNb+fPn1+2J2Tn3LO5SlsHSIamI03IbCyrrpS09eBe4MXiEVKCb2SOTT6K9yR45cqVYKSLB5RBEQZH6EfwEUxs4oG9cuVKRZuVAPlq60NLel8kbT3oy8HrQBiVZA0ogyRxa8CzrF\/aOtB05GPLxh88Z\/w+jksmWdocVVbaeoRJD1L03WmIqm\/WYV6TIOTArjI0F3lQGTjwvY8jyWamAegUHx7QU6dOBZsh4EUhjJLxEmb9EESVl7YeUWl9CUtbB\/R0NyOeM+xfs2YNBy8E++LeA\/eZokuC6T1srMDzRJN+06ZN4rPT4APAXpMgAOFRsLsMngZeB2G+S5zN2K8kzjl1coVRMp8e2DT1cO8F+njrHN3wPM+xJer5AX+9F9jn6vl2H8L2YTthCOduPTjXZ+rw4cOlHztD13cJ1yUre70nwayA8Kkcs8UQMASyQ8BIMDusrSRDwBDwEAEjQQ9viplkCBgC2SFgJJgd1lZSHAIWbgjkiICRYI7gW9GGgCGQPwJGgvnfg4axgOk9TPNhegZTN9JWDF0krX4WeszjpB5sw8Xa6HqUSb7kTzmUV48yLM\/KCBgJVsaoEBrMP4SAIKJKBlejWykvN15\/MpUj0x3cuCKeM9+upaWlDqt5bqPBtBzyp5zbIfY3DwSMBPNAvYHLtBUKDXxzG7RqRoIFurF4eXh7NJ8QbULSlGLXEzyw5ubmYL9Ft6lVSZeNAtg+Cj2E\/OJgidMlDWVjw9SpUyPXDrs20QzkWsth2ZrWLRxH3tilgidLOtLPmjVLNB16hIOL6nJO3bCbONKQP\/EcuSa8kpDPk08+WSrLzTOcFl1Ew7EXG7l\/GmZHfxAwEvTnXlS0hDW5LB1kRQB7xrHonxef5VJsw0Wzavv27UE+7PLMjtzosqb3rbfeCogpSnfp0qXCcivVZXfiKHLgJXZ1KYsljeiSL9fDhw+XqA0hICHdJZxysI1rwjEY8ly1alXwW9Xs6IL9lEf85s2bg92SSUcZe\/bsEcJJ98EHH5R26MYG8GC5GPkpRughpKFMyiYvjlwTTnwloV5sT0Va1haDBTZWSmfx4jUERoJe355y43gRIR2W17G8ixc\/rBWOY1POuG3v2TQAL0x39aWf6vOf\/7zQVxXOF112icHTIw7SHTNmTKQu8a5ASFzr2lzsdpd1QXyUjQ67unBEWDd74MCBUr8cOwkRrsJO1moPYazJhqBIBw6QFuEINtBcx26uOXJNONeVBH0EvQULFgQ75oAJ1ybFRcBIsED3jnWhixYtErasqtScozmGDoJnB1lEVRWCPHHiRKeNHNgaK0w2pEX3rrvuCrZv57oaIT8Ip9pt0PC0aEpSD+R73\/tebLHo4h27JOoqYwMeIkRMXhy5JtzVizvHC4dYiWdnlzhMiTcpDgJGgsW5V4GlECHNMQTvSZuNQeSdP3iKNH9pBqP30ksv3YkpP\/Ay481BBuiqRI3uovvRRx8JZFieU3IIxNSVn0bALtJxxDaaoHElQVAQVRypYQOeHJ4feamAaVyebjgEC9ESBgZx3jXxJsVBoB4kWJzaF8xSvDtEzeal5qXn5dcwPfKC8qJyzfb3XHMelpEjRwq\/iYL3Rxz9Y3T605nPtSvosl+gNpUhJrxItznq6rvneF1cQ0AcIWr2huTIdZJcvnw5IF4IiL3+knTdPfXC+tighEoelM3gCP2IXFcS6qrNXzAFCzAJp+O+0C8JltjAzjphHbv2BwEjQX\/uRUVL2OMOb4SmHMIAAQMiJISIeElnzpwZNFfxEhmtRW\/ChAkyYMCAgEjCuuzazZ5z5IUunhJ9alHeEWTr6pI\/gwval0fecUIfne6JRzk06VevXl3q64tLhz14quhTR+qX5I3S14j9pEN\/4sSJpayjbKB7gTQlpYQT7GAgBfshObAAE8iOUWpIleTavxplQ1gXfZN8ETASzBf\/qkrnhWMwRJtx7sACRMS+eRpGc1b1vvrVrwrh+rKHdSEH4lWftHGGhXU1T\/Q5Jx90uA6LlqvloI8O5SGcIxAw9aS+COekoX7UBc+KvBA9J50Keak+np7bF0ka8iEeoSxNV+mI1039SMdR68nRHbzhmnj0KIsRdOpAXYhzdSuVafH1R8BIsAYYWxb+IEAzXrsMaIrSfIa8ICB\/rDRLfELASNCnu2G2dBsBmqI0VWmy0gdIhnQjcKxW6POkSa3N3GrTV9InX\/KnnEq6Fl8\/BP4fAAD\/\/3yCfPMAAAAGSURBVAMAqSEZd+rvXLEAAAAASUVORK5CYII=","height":193,"width":321}}
%---
%[output:347eff3a]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Unable to find system or file 'afe_inv_psm'."}}
%---
