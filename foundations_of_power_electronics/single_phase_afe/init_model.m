clear;
% [model, options] = init_environment('single_phase_afe_ideal');
% [model, options] = init_environment('single_phase_afe');
[model, options] = init_environment('dcdc_buffer');

CTRPIFF_CLIP_RELEASE = 0.001;
s = tf('s');
%[text] ### Global timing
% simulation length
simlength = 2.5;

fpwm = 5e3;

fpwm_afe = fpwm;
trgo_afe = 1; % double update

fpwm_inv = fpwm;
trgo_inv = 1; % double update

fpwm_single_phase_inv = fpwm;
trgo_single_phase_inv = 1; % double update

fpwm_dab = fpwm;
trgo_dab = 1; % double update

fpwm_psfbc = fpwm;
trgo_psfbc = 0; % double update

fpwm_isop_rail = fpwm;
trgo_isop_rail = 1; % double update

fpwm_cllc = fpwm;
trgo_cllc = 1; % double update

t_measure = 1;
tc = 1e-6;
tc_factor = 1/fpwm/tc/2;
tc_decimation = 1;
delay_pwm = 0;

dead_time_afe = 1e-6;
dead_time_inv = 1e-6;
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
use_current_controller_from_ccaller_mod1 = 0;
use_phase_shift_filter_from_ccaller_mod1 = 0;
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
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:3cb1b833]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:53dd822b]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:96573b44]
hwdata.psfbc = single_phase_psfbc_hwdata(application_voltage, psfbc_pwr_nom, glb_time.fPWM_PSFBC); %[output:02772e73]
hwdata.three_phase_dab = three_phase_dab_hwdata(application_voltage, three_phase_dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:5eec19b6]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:4ba27042]

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
    time_i_react_pos_ref_1 = 0; % default
    time_i_react_pos_ref_2 = 0; % default
    i_react_pos_ref_1 = 0; % default
    i_react_pos_ref_2 = 0; % default
    i_react_pos_ref_3 = 0; % default

if enable_i_react_pos_steps
    time_i_react_pos_ref_1 = start_time_LVRT + error_length + 0.335;
    time_i_react_pos_ref_2 = time_i_react_pos_ref_1 + 0.5;
    i_react_pos_ref_2 = -ixi_ref_mod1*tan(acos(0.95));  % cos(phi) = 0.9
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
parasitic_dclink_data; %[output:07139a3c]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:861e219e]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:61077dba]
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
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:933a6d6b]
psm_ctrl.kp_i = 0.25;
psm_ctrl.ki_i = 35;
%[text] #### Induction Motor Control
im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:91983bdb] %[output:7aba7ad5]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:91983bdb] %[output:7aba7ad5]
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
sogi = sogi_filter(omega_set, sogi_delta, kepsilon, glb_time.ts_afe); %[output:7aa03050]
%[text] #### Current control parameters DQ PI
dqvector_pi.kp_inv = 0.5;
dqvector_pi.ki_inv = 45;
dqvector_pi.pi_ctrl = dqvector_pi.kp_inv + dqvector_pi.ki_inv/s;
dqvector_pi.pid_ctrl = c2d(dqvector_pi.pi_ctrl, glb_time.ts_inv);
dqvector_pi.plant = 1/(s*grid_emu.trafo.Ld1 + 1);
dqvector_pi.plantd = c2d(dqvector_pi.plant, glb_time.ts_inv);

G = sogi.fltd.alpha * dqvector_pi.pid_ctrl * dqvector_pi.plantd;
figure; margin(G, options);  %[output:749b0793]
grid on %[output:749b0793]
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
used_device = 'infineon_FF1200R17IP5' %[output:08ce1ec0]

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
nominal_battery_voltage_1 = hwdata.dab.udc1_bez;
% nominal_battery_voltage_1 = hwdata.afe.udc_nom;
% nominal_battery_voltage_2 = hwdata.cllc.udc2_bez;
nominal_battery_voltage_2 = hwdata.dab.udc2_bez;
% nominal_battery_voltage_2 = hwdata.afe.udc_nom;
nominal_battery_power = 275e3;

% nominal_battery_voltage_1 = catenary_nominal_voltage * 1.5;
% nominal_battery_voltage_2 = catenary_nominal_voltage * 1.5;
% nominal_battery_power = catenary_nominal_power;

initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage_1, ... %[output:group:020bcc72] %[output:76286c21]
    nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:group:020bcc72] %[output:76286c21]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage_2, ... %[output:group:840f0463] %[output:38117c63]
    nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:group:840f0463] %[output:38117c63]

% special settings for simulation
% lithium_ion_battery_1.R0 = lithium_ion_battery_1.R0/2;
% lithium_ion_battery_1.R1 = lithium_ion_battery_1.R1/2;
% lithium_ion_battery_2.R0 = lithium_ion_battery_2.R0/2;
% lithium_ion_battery_2.R1 = lithium_ion_battery_2.R1/2;
% lithium_ion_battery_1.C1 = lithium_ion_battery_1.C1/200;
% lithium_ion_battery_2.C1 = lithium_ion_battery_2.C1/200;

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

output_transformer = single_phase_transformer_setup(trafo_load_name, ...
    trafo_load_pwr_nom, trafo_load_u1_nom, ...
    trafo_load_u2_nom, trafo_load_n1, trafo_load_n2, trafo_load_f_nom, ...
    trafo_load_eta, trafo_load_ucc, ...
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
% open_scopes = find_system(model, 'BlockType', 'Scope');
% for i = 1:length(open_scopes)
%     set_param(open_scopes{i}, 'Open', 'off');
% end

%[text] ### Enable/Disable Subsystems
% if use_mosfet_thermal_model
%     set_param('single_phase_afe/single phase inverter/single phase inverter/full_bridge_mosfet_based_with_thermal_model', 'Commented', 'off');
%     set_param('single_phase_afe/single_phase_inverter/single phase inverter/full_bridge_ideal_switch_based', 'Commented', 'on');
% else
%     if use_thermal_model
%         set_param('single_phase_afe/single_phase_inverter/single phase inverter/full_bridge_mosfet_based_with_thermal_model', 'Commented', 'off');
%         set_param('single_phase_afe/single_phase_inverter/single phase inverter/full_bridge_ideal_switch_based', 'Commented', 'on');
%     else
%         set_param('single_phase_afe/single_phase_inverter/single phase inverter/full_bridge_ideal_switch_based', 'Commented', 'off');
%         set_param('single_phase_afe/single_phase_inverter/single phase inverter/full_bridge_mosfet_based_with_thermal_model', 'Commented', 'on');
%     end

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
%   data: {"layout":"onright","rightPanelPercent":31.6}
%---
%[output:3cb1b833]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V\nNominal Voltage: 690 V | Nominal Current: 2.324276e+02 A\nCurrent Normalization Data: 328.70 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:53dd822b]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:96573b44]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 9.167325e-05 [H] | Internal Tank Cs: 2.763107e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:02772e73]
%   data: {"dataType":"text","outputData":{"text":"Single Phase PSFBC: PSFBC_1200V\nNominal Power: 275000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls1: 2.424242e-05 [H] | Internal Tank Ls2: 2.424242e-05 [H]\n---------------------------\n","truncated":false}}
%---
%[output:5eec19b6]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: Three_phase_DAB_1200V\nNominal Power: 750000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 9.600000e-05 [H] | Internal Tank Cs: 750 [F]\n---------------------------\n","truncated":false}}
%---
%[output:4ba27042]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 6.192295e-05 [H] | Internal Tank Cs: 1.136282e-05 [F]\n---------------------------\n","truncated":false}}
%---
%[output:07139a3c]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASwAAAC1CAYAAADlY8JKAAAQAElEQVR4AeydC5xNVfvHn13KKMqIYlwa10ihjKIIRUO5y6AoKffuyUgXo5JbCSEkSm6v+ld4QyW3FIkQJbcQuaSQS6aQ\/3xX9rzHcc6Zc86cfTlzlo\/n7H32Xntdfuvs36z1rOd51nmn9T+NgEZAIxAlCJwn+p9GQCOgEYgSBDRhRUlH6WpqBDQCIpqwYuhXoJuqEYh2BDRhRXsP6vprBGIIAU1YMdTZuqkagWhHQBNWtPegrr9GwBcCOfSaJqwc2rG6WRqBnIiAJqyc2Ku6TRqBHIqAJqwc2rG6WRqBnIiAJixfvaqvaQQ0Aq5EQBOWK7tFV0ojoBHwhYAmLF+o6GsaAY2AKxHQhOXKbnFHpdatWyc33nij1KpVS2655RapWrWqtGnTRnbv3h10BceMGSPz588PKj3pypcvf1Z5L7zwghw9elSOHTsmffr0kQ0bNgSVV\/CJdMpoQkATVjT1lgN1rVKlisydO1eWLFkiK1askNKlS6vvVlXl+uuvV\/lT3tKlS1UxAwcOlNy5c8vLL78sFSpUUNfC+Th9+rScOnUqnEf1My5BQBOWSzoiGqrx999\/y+HDh+Xyyy+XEydOyODBg9UIjFEY51xDBg0aJNWqVZP69evLt99+q5oGWUyePFmN1GrWrCnmyEnd9PORJ08e6dChg3z\/\/feyefNm6dKlizDqO3LkiDzxxBNSvXp1ueGGG+SVV15R9eH6I488oq61a9dOOnXqpNIzyuvevbs0btxYli1bJgsXLlT1uOmmm6RRo0ayfv16+fXXX6Vly5bSt29foX6MJCdNmiS33nqrym\/BggV+aqkv24mAJiw70Y7CspYvXy6VKlWSUqVKScWKFWX\/\/v3qZZ83b57s3LlTGAUhBw4ckA8++EC4vmvXLjUi4zsER7NJw3Tu888\/ly+++EIuvvhimTlzJrcCSoECBaRgwYICGZkJv\/76aylTpowiH8rbuHGj7NmzR9566y01AuM+o7LffvvNfESo37Rp01RbPv30U4E8v\/rqK0WI5EHCP\/74Q5KSklT9EhIShHw\/++wzGTJkiKrryZMnSabFQQSySVgO1lwXbQsCjGK+++47+emnn9Qoh9HH888\/L19++aXccccdcsEFFyipV6+efPPNN7J48WJ1ndFRvnz51GiFikIiEMZVV12lppWjRo2S1atXcytkqVu3rtJzzZgxQ00TIcJ9+\/bJpk2bhFGTYRhStGhRufbaazPzrlOnjlCfSy65RHr16qXSvvbaazJlyhTZu3evSsfIkdGiYRhy3nnnqbqff\/75Ammmp6cLIvqfowhownIU\/ugqnJe3du3a6sU9fvy4IiqzBUwFIS\/zu\/cRpTlEAfEhkExaWpp3snO+MzJidMUoy7w5YcIEGTt2rEB+jz\/+uBr5MeVEPwXRkI5zprCcewoLBt26dRNGXykpKdK5c2fP22edB2rPWQn1F9sQ0IRlG9TRXxCkwNQuLi5O6a5Y1YOokDlz5qjRDborzrl28OBBYQpIy9E1MQWDfCA7dEVMDbnnT0j3xhtvyNVXX630ZmY6pmp33nmnsCAAAUF+jOiYxlE\/6rljxw5hZGg+Yx5\/\/\/13NdJCd8XIyayfeV8f3Y2AJix394\/jtVuzZo00bNgQvZXUqFFDTQVTU1OlVatWaqrEFBEpXry4NGjQQJo0aaKuMzXr2LGjFClSRLWBKSNkxpHpGWTBubrp8YGS3iyPfCEiFOweSVTZL730kpqyoWNCxzZ9+nSlZIcEq2co4zGBuOKKKzwfU+fovuLj44WR4t13363IcO3atWqKqBLoD1cjoAnL1d3jbOXQAaF7ggQwM0ABj7KakQzTJaZ43Ec45xoE069fP6XP+vDDD2X06NECMXHv4YcfFtIiZnrPFpLuxx9\/VEpvylu1apWgL8ubN68gTAOpE3omdGiMpiCmkSNHSv\/+\/YURHGRK\/sOHD1d6KKaSXbt2FYSyqB8rmuRN\/SBVSA9ynDp1auZIjpVH6sMzlEnZ1IHvWpxDQBOWc9jrkiOMAEanENjNN98s6KcwbTBHeBEuSmfnEAKasBwCXhcbeQSuueYa+fjjj9W0lREaq4mRLyU2cnRrKzVhubVndL00AhqBcxDQhHUOJPqCRkAj4FYENGG5tWd0vTQCGoFzENCEdQ4k2b+gc9AIaASsQUATljW46lw1AhoBCxBwJWER\/wjHWbznicOEcM417lmAg85SI6ARiAIEXEVYuFRgpHjvvfcqh1TiH73\/\/vuCYMgHWWFbM2vWrCiAVlcxJhDQjbQVAVcRFn5mONi+9957QvwinFvxoEdKliwpENn\/\/d\/\/Zbp72IqULkwjoBFwHAFXERahP3C7IEwIcZdwfsVdAutljnyH0PBJcxw5XQGNgEbAdgRcRVgESMOL\/7777lORIPHzItIjnv74hw0dOlSYNtqOki5QI6ARcAUCzhKWFwSEIyFULe4VTAuJZ0Qo3WbNmqmQugRaI46R12P6q0ZAIxAjCLiKsBg9MS3Eo56YS2w8kCtXLtUVTAX5Thp1QX9oBDQCMYeAqwgr5tDXDdYIaARCQsB1hEX4XOIpsYnA1q1bhSNKd65xL6TW6cQuQkBXRSOQfQRcRVhsEkCQNyJS3nbbbSp4G0fiG3GNe6TJfrN1DhoBjUA0IuAqwrrwwguFmEaYLfgS7pEmGoHWddYIaASyj4CrCIsdUhhF3XXXXcqswXTJwS2HTTmHDRsm7L6S\/WbrHDQCGgELEbAsa1cRVqFChYTNBD755BOBoObOnavie+Ougx8hKBAbnKMWjYBGIPYQcBVhmfDjooNVOwRmXmMHFK5xz7ymjxoBjUBsIeBKwrr00kvlr7\/+Uptlsu8c8u677wqGpFYo3Xft2iXsCKNlucZhucYg2PeA98ZuunQlYWEgOmDAAMFVp02bNmofOsB58cUXBaPSSIJEvk899ZSwR51LRNfl7rs1Bg5h8HiHFBnUpYWs7lNfHQO9E7w3vD+RfB+zystVhMWUD\/cbKo3FO3vMsfsJe9Cx7xwbYHLPTMN5MIJ1\/KRJkwSfRPafe+edd+TUqVPqUQDHT3HIkCGCg7VV8uijj6ryrCyHvCnHqjaY+epypob8W6Ff+AGAnYmjFUfyp6xQ8p40fICM65wsw2vmlinX7pRXy+2RO8sXlJef6uG3nZTBe8P7Q7vsElcRFm44s2fPVpteAgbxr0wgOF+8eLG6RxrzejBHdhNms0yU+Sjycaj+5ptvznq0aNGiwo7BVknTpk0Fp+6qVataVg6rqfyQrGqDma8up3rIfWhH\/9M\/wfZNUunCUjH9Rykx72lJmNRGSq0aI5Xzpku+Oh2kSNpCKTF6m1Ro94zfdhJV5awXyKYvriIsVgA7deokzz33nMyZM0fYV65UqVKC3H777Wq\/OZyhSRMKPpATADNCQweGjdfKlSvPygKC3L59u6Snp1si+fPnl9atWwv6OavKOHTokLBzMk7kVpVBvtFczk+\/HhVTlm45JMiiH3+TJd9ukPnr96jv3KedkRR\/\/R\/JMsgrYN\/s3iJHFr0tu9Pqys\/dS8r+UfdLesa1PI1TJb73XLl86AbJ98AbYpSuHvAd4D355Zdfznp\/7PriKsIyG81op1+\/fgLR4I6DfPXVV4KLDsH8zHTBHrds2SLFihXLTF66dGkB9MwLGScvbkiQRm\/vlKSB31giNYetk2aT90it4estyZ9621FGNJWT0GeZeAv1N6XJ6NWCtBhH3+yVlAk\/StfJ6zL7p\/Go1TJuwWbZs2dPtgU1BjHe9u3bl+28AtXnnHJ++EZ2T+otu\/vWkd2PlpX9U5+T9OPHxUgZIsbTS5Sk1+oihwpcFXS9UKmgv8p4bWz\/70rCsgIFFPmB8m1brYi0r1HMMmlbrbA0uya\/cLSynPtrJVrWBs96R0M5qcmJMqpthXNkVvfrxJSVvauJp5jXaev+4yJp83+Txu\/skhO541Wk2yJFioR1xCwHM53ChQuH9Xyw5Ra9SOSai47JRR+nyekBtyjJtWam5CtRQQr1mCgJIzZLQr9FUqTxQ1Lk6mph1QXVBqqHQO+TVfdigrASExMFR2oTRM4rVKhgflXHeyrmktTkkpbJk7cVly435heOVpaj8z67D\/kD4S01y+QXU0pdnle8hXvguObZGoKULJRXqr+yWlq9tUHi4uLCEla3+aMZ7vOBnst1ZK+cWD5djrzVTfY\/WUEODmwosvVryVe5XqY+CrLKV6dDWHX3Lpv3CRWLenFs\/ohqwgoWqxo1asjq1asFo1OE8ypVqgT7eNSkQ4exbds2pX+wstKxVE6JAnEyu8e\/I7KdB9OlykvLZNAn26yEN6i8T+7ffo4+6vj3iyRPzXvE6DJV6aMgqTwV6wSVX7Qkci1hrV+\/Xho0aCC1a9cWVvU6d+4sGJCGA+z111+v8mnYsKEkJycrZT7XwslLPxObCDDqYrTFaG3QJ9ulcYZ+6+cD6baCAUkdnNHvLKX5yV+3S3yrtMyRVHyX8SIZSnNbK2ZjYa4kLOyxxo4dK4899piggGf61rhxY2HbL+4FwgflPCFpsLm64447BOIz02Mpf9555ykbrH\/++ce8nGOODN3ZXYijlY0i\/1gth6kiei67RlveJHUkY5WPvmX0hOkBEp\/SV8yRlF19Qx2cEFcSFtM25vts8wUohEfGxIHO4B7XfAnL+a+\/\/rqwh+GKFSukW7duMmLECOVukpUdlq\/89DUXIeCiqpijrdQMpX6kR1sQFKSEycFPrQzB\/IDvFxRKVEpzCCohbaGyl8qVcc1FsNhSFVcSFrZSGJGyfGuisGHDBmEDCpSX5jXvI8vGiYmJUrZsWXWL+Fm\/\/\/67st9CSeikHRZ6H\/wjGSFyboUEtMGJoH2ZLudfW71HaxeRGR3Ly7b9RzN1W4H61W\/\/7\/7XPgqSgqA4Hlk7X3JVby2e9lEXVG+j9JOByrCjbzAJ+kXbYSmOUR+QUrt27SQtLU1WrVol6J4eeughefzxxwUyU4l8fJQrV07wQcybN6+cOHFCCEnDUjLuA8WysMMaPny4YF8CSVoh59jHRMC2x7uedpRBmbqc\/9lllb44XWbfV0w635A\/Qxm\/Xdlwrdq48xybpnMw++Eb2TN75P\/so0bdL5CU1H9UKc2xkTrVYkBI9lF29Q3vibbD8iIgRke40ixbtkzmzZsn6KYqV658Viq2BEMpjyV8z549M++xEWtKSops3LhRcJhmtMYUMzOBjxN8sLAvKRKmnU1Wz9lhh8PoEit+jlnVJzv3yV+XU+QsG6a0ZhWUPRe\/Ney2Jn2XftZ91f+5\/pLcX4yV02PaKvuo8xeOkrg8eTKV5spG6t6BUqRmcwm3f+zoG94Ta+2wfLygZy65akrIlI7oDEQaRSAj9iTEPwodFvdIc6bugtU7\/oVYwqO3On36tODk3KNHD3niiSfkjTfeEEZYdCK2V+ZznKPIN79zRLlPOvRkVgijRkjTirx1nuHZRkUaN+y5GRehRwAAEABJREFUWElEtzV88V5pPnCubJ3ztrKPOti5gFw4tL6cv\/qjTCNOUx9lKs0jXR+r8uM9QcXCe2O3uIqwChQoICjNZ8yYIU2aNJEHHnhARSAlCilTRFb9ChYs6Bcjpn44Ro8bN05q1aolKOtJrO2wQCFygg5F23v5xhOleffci2XuyT4y6vsWkmdaN9m4dJ5420dhxOk7h+xdtatvslfL8J92FWFBMIyIODLnZ2SVkJAgCHF5vvvuO6V499dcfAYxY7j33nuFERrCqKxEiRLaDssfaPp6thCAoFjFQ1G++4xT8cEZ\/aTIyb1qqvdT+w+lSZGp0vBID1kVVyVbZemHRVxFWGaHsDMOK4IQkHlt3759iqwgM\/Oa95FpI6uJxIBHiU5kBMwazFFZbNhhlRSmAt7YRPI7+ceqHZZJUJCSSVCQ1fHvF4mn6QHTPaZ69Zo0U+49xePjpPMHe5V7TyT7wjsvu\/rGu1y7vruSsCAaFHvdu3dXUzumd23bthVGTkwbswIHPdegQYPUEjBpv\/32W9F2WCChJRQEICeIyHsEBUFxzSQoM34Uxpy+pnq497z3QAUZ16KwfLn1UKYJRCh10Wn\/RcCVhEXVGC2hUCdyIvHcGTXdeuut3AoomDOMHDlS0HcxvSQxYWpQEsbHxyuzCFa47I6HhW7Brx1OhGykDh06JDoeVrr6QwXePmX3FhUDijhQkBFycP542fbeK7JneDsVI4qRk2m0uSdjmkdIFkwOTua7QsWL8raNyip+lFmPawoasuih8tL82n9NIPBLjHTsLTt+A9u3b5dftB2WZP5jhITuCRebe+65R42s2PaLa9zLTOjjZP78+XLRRRepbcLM20wts7LDqragp\/zzci3Z\/UhZS+RAr2vl\/CF15WBqJUvyp952lBF15bTPI7s95dGM\/j0jkBFycGwnOTnvVTn+\/WI58vMGSc9zmUhSy39jRnWZKthEIdhFHb0qOWTbKE\/7qAv\/\/kPurRSnbLcK5ckoZuA3kvbRhnPstngmCDnnOXS\/vCOoUMJ5PphntB2WySxnjkz7WC18\/\/33BXnzzTelUaNGYo6SziQTbzssVgmx2cJR2lvXhUmB+ZyvY9nb26lwHPkq17PkmKdibTm\/3M2W5O1Z5\/zXN7C8DMqzs5z4eg9K2NIqTbm0MF0zhSmcKQnDN0vCu8el+KhtmbGiEh6dLIiKGVWzedg2UUU8bPqUHVahQmLGw6p6VXEVBQITiHErDqmYW1uPxWW7rMTERGEGwdGz\/Eieo67RdlgeLALZMJ3DzgrBpxC7KhyWDx8+nJmSe0wbTTsspkOYNZhx09muiJEZNlDYXpkPcu5th7W1apdzftjmDzwSR7zoifJICNpI5BdLecSn9JXsSL46HZTvnXnEUdiUuIQyEhdmjKtQnuM3yB9N72dSk0sKtlvE3CLiaXZibnnnbdV3yJDBg\/k+2Xl0rQ7LG4SdO3fKL1nMm+vVqyeQFwJZVa9eXSnbMT4lBhaO0wjnVapU8S4i6r+jK9H2UaF3o124+asZSnkz5lZ2lfJOt8VfGyN13ZWE5TnVw+0GQZdFDCsU56E2nuewmscn0SMeVqjZ6PQaAUsRMKNAmDG3UMrbHXPL0gZGIHNXEtZll10ms2bNUmGNGS0h7E1ITCymhYHaTZA\/jExJ+8cffyg9l5le22GZSGTvyFQjVu2wsodc1k8z2jKnicXj4wTSCiXCqV19k3VLrEnhOsJi\/0Gclp955hlh+ZTRFoJ+CgdnwsX4g+LkyZOCE3OLFi1UDKyuXbvK22+\/LZgwaDssf6jp625EAOJimpianKiiQEBcS7cccmNVba2T6wgLm6nx48fLDz\/8IKNHj5Zhw4YpwRYLHRUriP4QYnR14MABwV7LMAxp0KCBQHyEqEFJGK\/tsALbKAVpD2aHrQ+6mEiWQ37+xK5ywrHDI+bW8p7XSZF8udSWZIRm9tcOrtvRlu3aDut\/FITBaP\/+\/eXVV19Vsa0Ii2xK69athfAd\/0t99hlTQAxH2b+wUqVK0r59e6EDg7HDwpXnnXfeOceuJRi7lGDS2GEfY0cZtFWXsyfk30l2MLvgr4MyuknBTEt59lX0Z7uVnXLo22CE90THwxIRDN4gK4641jBSwoHZFG\/DUaaKKNNRyvfs2VP5GjKdJMoDjtIo6p9++mkVzI8l5Ywi\/P5nKol9SSTtVTzz8rbD8bwXqfPEROttcKirLufsWFhgkpVEov8bJZVUMbfa1ygm\/my37Ogb3hNth5VBJdheMYUjusL06dNlyZIlZwnXSJORVP33tsPCyTkpKUlMGyumgWw6wY8J2yv1UMYH52aajK\/qf9GiRYXORmlphfizw7GiLJ2nO+JjefZDpPqfmFupXrZbwxfvscWWzGwP7wnvlnpxbP44L5jynEjD6Alrd6Z3pjDq8jQc9a4XQKIn2Lx5s7pF5AZCKjMKw\/YKGyyEc22HpSAK6wNdibb3Cgu6iD3kTylvV99ErCEhZuRKwoJUHnvsMWHDhptvvllMYeuuCy64wG8TISemhs8++6x6hoijvXv3lurVq+t4WH5R0zeiGQFztFU8Pk4p5ZuPXRfNzcmy7q4kLIiKoH2El7nzzjvFFBTyDK0DtQr9F4RHuGSmg566K22HFQi54O8xNdB2WMHjZXVKc7Q1q\/t18vWOo9J8yl5hmmh1uU7k70rCwnQBXRVTulBAwaRh4sSJavcbDE3xI2RaqeNhhYCiThq1CNQsk18ODK0rnpbyOc12y5WEdfDgQWX4yaog00B\/q4S+flnosDBl4B7zeaaJ2HahJIzXdljaDsuHrRm\/FwyT+d3xm7FK+G0ye7Aqf\/KlLU2v\/EuIu2XabmEpz71IibbDgl08hNHVhx9+qFxzVqxYkblS6L1K6PGIOmVkBslhr1W6dGmZNm2aEKlU22GFbjcUyB7HDlsfys9J5djdFuJuYbvluV\/if1duC9l+jH7wFm2Hpejmfx+sBLIiaK4Omsd+\/foJkUcxDiU1K4msAJp2WPgcss3X3LlzFdmhgGf7L1x2PHVZPOst2g4reNsiVmOtjrmEKUpOKicSdlhgkpV4Y5Z2Zr9EwtcQU777rN88Ym4F3+ee5Wo7LC\/2YCWQ4TMbUbBFF9NCwsswnCbe1UsvvaSe8LbDgrD4YWDHZRiGYJOF7yFB07C9Ug9lfHButx0WVvj\/+c9\/lOU9Smst7rOVsrJPnOx\/bLfwS0Qpv+fISUnos0wp5cNtL6SIiiXjVbL9vyt1WBAToyxGPURdaNasmeA6wypft27dBOW6L6SKFy+urOUJD8t9fAiZXjIKw\/aK1UOEc287rK+\/\/lrpzYijZYVQF4bSM2fOtKycpUuXCqNQiNGKNph56nKWh9yHdvQ\/\/ROob3L99qOMqWdI7Uv3ZTpU\/2fx+pDbQjlZxabj\/bNCXElYNBQFIQTFOXLq1CkhkgPTO777knLlykmHDh2E8DI33XSTTJ48We0AHcgOi1jv\/LWAEHnOKjF9r6wsh12FIEXckaxqB\/nqcu5WvzGwCFbs6H\/qEkzfrH2nj1zyaars3rRWus3cH3JbKIf28N7w\/vh6F6265krCQnlO0D0U6Bh+osO6\/\/771RQPHRb3fQFiGIYwIsOl56uvvhJGGtgLGYahNrLgrw\/X2UXaMAyVBYAzkmN3ntBkquj0GoNo\/Q1MHz9CZnWvIqnJiWH\/jnlveH\/Ui2TThysJi7Z37NhRUKDjAI2l++DBgyUlJUUREpbspImUADqjMC3VlVeAxiF2cEhNLhl2n\/PeROodDDYf1xIWRqNjx46VRYsWCUagb731lpru0TCU8hy1aAQ0ArGFgGsICyU7ZgoIdh\/Dhg0TbKlQsJctW1Yuvvhi6dSpk6BEj60u0q11AQK6Ci5BwFHCQpH+8ccfy2233SZEE73rrrsEadq0qSxbtkzy5s2r9iK85ppr5OGHH1ajLVb5XIKdroZGQCNgMwKOERYrfi+88IIKuvfRRx+Jp0X7559\/LthfYVcFoe3YsUPYq\/DYsWMqgoPNGOniNAIaAZcg4BhhEXUB5TlWs\/j7eeLBd1YIsacaMGCATJgwQerUqSMYivpbIfR8Xp9rBDQCORMBOwjLJ3KMmFCeEw4GcjIdnM0jRIU7AFt+YWy5ePFi6du3b8CY7j4L0hc1AhqBHIOAY4RlIkgEBQzQcFIeNWqUQFB4zaN4nzJlitSsWVPpuNBrkQaCM5\/VR42ARiC2EHCcsCAnzPzZ4QafP3wImfrh0IxVOFPB999\/XxBiW2U1JVy4cKEwSoMEn3jiCUFJTzA\/bLrwSYQAsQZH4R9bXa1bqxGIfgQcJywcMHHDQV+FyQLOzei2cFomTDKxrNasWSOXXnqpMmlgKukPdhyksdfC+hh\/J8wh+K4D+PlDLPLXdY4aASsRcJywULC3bNlS7SFYq1YtqVu3rqSmpgq753Tp0kW4B\/lg8Y5pA\/Za\/gDZsmWLIjV0X4ZhqOkkBqi44zDiYvpJeYRGWblyZWY2u3btCssBlHppCd0RWGOWMzDjvcl8iWw6cZywaCfTPkIb41CJdfuMGTOkYcOGwgoihEWaYOTKK6\/MDFDGlA9FPSS2adMm8XQjwCB1+\/btKktAp1wcOrWE7tSrMQsfs8c7pAgyqEsLQd5+uImYMu\/JhoKs7lNfPGVPWl0JJP8MqCWhyuWv1pBwhPeG90e9SDZ9uIKwxo8fLwTbGzNmjBDitWvXroIvIbqoF198Ufk6LViwQNBhXXLJJX6hwdGZzVNZdcQfjiiP5cuXF+\/NKDwzAHBCy+DIyVTSKjE3nrSyHPKmHKvaYOarywnO6XvS8AEy4dkuMq5zsgyvmVueStwvr5bbI5Ov3Snzq26TKRlHhGtIr4z7pnSsnF+QO8sXlKa3JGXKjdWrS83GbfxK2dvbCVLhnmclWIlvlSa+pFCPieJPfqraVb7++mvh\/REb\/zlOWLjerF27VjBjwP0G0wWCne3evVsYbRExlNhVXGOFkJGTP3wIPYNSnYgO6L4aNGigzCDKlCmjIpCaz\/kL4AfJWSWscjJirFq1qiJgK8phd6EmTZoIeFmRv5mnLudc5+ik0oWlYvqPUnrVWCkx72k1YkmY1EZKzO0tpVaNkYpxRyW5WkVFNBCKSQRF0hYKUmL0NkFKvXdaEM5NSchI4ynms76O+R54Q47e8bzkaZIq8Sl9syX56nQQf5K3zn3m62Tr0XHCgoBQvF900UWq4bjjoFhHgd6jRw\/lqoO7DoIOC4JTCX18YApBgD9WHQkCSHgZXH6IjUXQPlYMEc55qX1kYdklVkCJVcTRskJ0xrYhcHL\/djmy6G3ZP+p+2Z0xTfu5e0l1fvz7RaoOvOgQCmQEASWM2CxG12kCoXCd+0ieinUEyVUoURD1sP7wi4DjhFWwYEGlKMfUgBESpggQDASFDorYVqZktQlF8eLFpVGjRioMDXoxvjPKIrYWUUfRiyUnJyvFPtf8ohKlNyB+puJEiY4AABAASURBVMUcrWwC+cdiOSZJeRPUBRlkAwlBToyKGA3x3SQkK\/vCO2+7+sa7XLu+O05YhmHII488oiIzQFj4F7KJKgq9XLlyhYSDYRhCcD5WBdFL9erVS7CmNwz\/AfxCKiCSiXVeUYGASVKMpMxR1Mlftyudj0lQTpFTVAAY4Uo6RlhMBdFJEU6GaRojIWymCGvMyhNTOtJEuL05Ojvs2bZt26b2HrSyobFQDkR1cEY\/MUmKqR6KaZOk0A8xlbMS53DytqtvwqlbJJ5xjLDQRaGTYurHNI4VQUwYkPr16wvxsIjOEIlG6jw0AsEgcCJj5ARJmVO+I4veVkpnt5NUMG3LKWkcIyys2tFJffLJJwJBsZcgq3vIBx98oPBlOqdO9EdQCNilv8hJ5TCSOrF8uuR+p6Psf7KCUqSbOin0UUz3sj+SCqr7IpLIrr6JSGXDyMQxwjLrynSQ6R8EZl5jb0Gucc+8po8agUggAEExvfMcSXFO3pCTSVIozLmmxV0IOE5Y+Aji8IzNFbZXyLvvvits8YUbTSTgOn36tNrQIqc7P9ulv4imciAopnaQkjnVw1Kca+ZI6vKhG+Sv+ybIBdXbROLn5mgedvWNU410nLBy586tjEZZIcRCvVWrVsp6Fgt3gvxFAhjt\/BwJFN2bB6TEqAkSQljRg5x+amVkKs25Tgs8FeeMqPRIClSiRxwjLKZ8uM4AFe42ODxjb8UOOf379xcclbm3d+9eDtkSrN4DOT+TOWYQ+BfyF8oqYSRJu63Kn3YUKVKEg1opdEU5u7dIehYC2XgK5IKgWyq4Y7EcnzVIGWVCRAhkhLCCZ5ISoybu7Z\/6nBxZO19O5rtCclVvrQw143vPFUZRBTKOWIAbpaufhQ+A2YGb1f1Pf9vRFt6TX375haJsF8cICxur2bNnC36DkAUx3s3Wc47RKPdIY14P94gDtD\/nZzPPagt6yj8v15Ldj5S1RA70ulbOH1JXDqZWsiR\/S+rdPo\/szq48moFnFgLZeIoinlH3Z5LU9x+OUSS0cek82fT9OknPc5mSk1WaipEyRIwuU\/+Vp5eIcUZOtRggyNGrkuVQgasyneLxhnBC+OOMGQ9hlJwoP5JlYuSNnaT57th5dIywWAFk267nnntO5syZo6zPCdqH3H777WovQoxISRMJQJh6BsqHqUF8vQfFKslX5z45\/8aMv\/gZRyvLkKSWGUvx90WmHa3OdYo9Ui9VfrhtiKy6+UXZlPxaSLLozmmCvFtjoviSHhU\/EFOSii8QU5oUmSpJeScJx8ZxQyRpXw8lTXa3lMbf3yid1ybKpGNVZOHRYrL1WJycyB0vjJiClfz58wujH0b1wT4TajoWklhYwjUr1GdDSW9HW\/CJxck+0Ptk1T3HCMtsUNGiRaVfv37CtI1dchAs1fv06SNEHjXTZeeYmJgoODybeXBeoUIF86s6HqvxgMRn01k00PP5WjwrRv1HhWOgdNm5R95Wl1G5y0Cp1+EhSWp6j9Rp11UaPPhY0NKxQxtB+j7RQXzJ3LTmMjetuZIDQ+vK7pdryKqHE2V5z+tkzbM1ZFb365SMaltBUpMTpX2NYlL7qoLCUv6H6w5Jz4+2S8qEH6X6K6szZfjiPbJyV7pKQzpfgq6UP2i+7kXqmh1lUFc7yuF9QsWiXhybPxwnLDvay5ZhODxjJoFwbrfzsx3t5AebE338yiXklxIF4qRmmfxK2lYrnEFYJZVAXrN7\/EtokJxJbKQpHh8ngz7ZLk1Gr5YCTyyUKi8ty\/i+7ZyusAu3cwq24EJOaosveGKCsHB0jgXnZ18dHGvXTGJLTS4pEJlJYqkZIzKTwPwRV6xhFY3tjQnCMgxDCO2ydOlSYbrZrl07MQwjGvsrYJ1ZJdK+hOdCBImlniEwRmCMvhh5NR61WpZuOaRWDO3A7dyaRf6KXb+BSNQ8nDxcQVisnvTu3Vuww4JQRo4cKawUhtMg\/YxGIBACJnlBXEwxmS4++t7mQI\/oey5CwHHCwmB00KBBKrwMuBQrVkxOnDghhOHlHte0BIeAXfqLnFCOSVwo8t9b\/Zs8MueQUswHh7R7U9nVN04h4DhhEbXhn3\/+UZulEnsdwDFlQDnOPaeA0eXGBgKMshht0doe0zZw0OJiBBwnLJZhCYn8559\/ZsLEFBGyyhViAL\/MDGLgxFcT7dJf5LRyLr9IpHfNvDLtm72CXssXttFyza6+cQoPxwkLB2f2HCR+O9txsXsOMbJat24tBQoUCBmXhQsXCrG1sBMh3DIjtVhxfg4ZLP1AJgIJl+RS9l5fbj3k0\/QhM6E+cRQBxwmL1hNr\/b\/\/\/a+89tprKsQx51zjXijCxhVELWU7KjbrLFu2rPA9VpyfmU7nRDss2hXK7yDUtOQPbth7odNiBZHVw1DzcUN6sy0c3VCfSNfBFYTFqKhv376C2wKRRgmXPGHChJDbis8g7g+4MhiGEfTOzxSEPyNOnQyprRLcP6x0fraq3rGUb1KxOLnxyrzy0PQNytwhkm0Pu\/\/T0yNel+y0i\/ck5pyfIQmE\/QbZjosp4Y4dO5Tyff78+cKoCD0WaYKVK6+8MtPJlXjwOFBDYkw1WX008yldurQAuvmd4\/DhwwWnzkg6iXrmZYfzK23CxYmjZ9mRPif\/nFzO6CYFhRVq9FmRws6O\/qeudvQN70nMOT9DEgh\/ddiLEOfQFStWCLonFPEXXnih+tGQxp+wgQUW7KVKlZKePXsKw\/p77rlH2XOx6Sc\/kvLlywurj\/iK+cuH65hR4NTJ6MwKoX2M\/hhFWpE\/edpRRqyUM7JNBVn1S7pypqbN2ZWc1De8JzHr\/EzEUaKLAsCyZcsEx0piunMNhTxk4k9wjmYUhcM0O0TzVzE7Oz9TdlxcnLLHifQREoY0I52vZ37x8fECQefPn9+SNphlxUI5dcoXFPwUcabOynnaxCXQ0Y7+p3w7+ob3hIGFv\/fSyuuO67B4iV966SVp27atDB48WC655BLVXkLL0MnqS5AfDIm7desmzK\/RFTHVrFevntx0002CwzMrhgjnOdH5OUiYdLIgEcCF5+bS+fWqYZB42ZHMccKikRBT5cqV5bLLLlPKxVq1ainywh6L+8FK8eLFhS3DUlJSpE6dOsJ3FPix4vyMItUOn7hYKodRFqYO0WJUalffBPtORjqd44TFNI4RVuPGjaVexmioRYsWQlz3woULC1ObUBpsGIYyi8AfkVW\/Xr16CYECDSM2nJ9DwUqnDQ4BXHgwdcCodNAn24J7SKeyDAHHCevgwYPCaiD7Ez755JPyxhtvyJQpU9RIyzByXkQFy3oyI2N0GCw8cMz4atl\/8ndnOeE1Oav24L4DaWGf5XbSyqot4SHknqccJyxPKHDFIeY1UUh\/++03RWSe9\/W5RsApBDxJq\/Go1U5VI+bLdZywChYsKEz\/BgwYoMwSxo8fryze2Z8Q3VbM91AIANilv4jVciAtHKV3Hkz3G700hO6yJKldfWNJ5YPI1HHCMgxDiIX19NNPKxssVvmwy8IuKiuzhiDap5NoBCKKADotSIsVRKaIZvTSnw+kR7QcnZlvBDwIy3cCO64SXmbjxo0yb948FbivXLlygnU6pgmBykdh\/8wzz8i6desyk+Hmg\/Mzpgx33323MFKLFefnuLg4NUrlmAmIBSfkH0s6LF8QEsHUm7hM8lq65ZCvR2y5Zlff2NIYH4U4TliQEm45Q4cOVVt7ffnll+qI1TuB\/HzUWV3CfYewxyjr1YWMD5T3EydOVC425FO\/fn15\/fXXlZvPZ599JqSdO3euLFiwQO3Sk\/GIbf+xup80aZJwtK1QXZClCDDaMokrNTlR\/rXZOnvTC8whUNQv335UJr73se7\/bPaI44SFISdTQHRXL7\/8spiSmpqaaUTqq43YbBGGBv9Bz\/u4+hw6dEhdYj7PtBK\/NyxzsQLme7Vq1WTlypUqjfmBGQQRHqySVatWKSKdOXOmWFUGMevZMg2DWavKIF9dzvKz+nD3pjVS+9J90r7kHzInJY8Svpc8tUPem7tYmDq2GLdORu6tKNeP2Kb0X4zGUN6bhAapIZhPIIzSTPnquy1iCvgHErv6BuNs892x8+g4YUEgKNdDNRK97rrrhA1XUdqbgBE\/q02bNkIsLRycp02bpizocYD25\/zMdcgM52emkFaJ6SxqZTmMOHFMRR+YRTskO\/d1OXdnid\/ad\/oIknfpYMn\/0QNyyaepwnncjzPl16\/+I3vXfC78kYTQhny4UpEaxAaBIcSaN6XR2zvFlDtmHBd\/Qppmk\/fI63uulh4Lz8t8huv+pMnoNRKOdJu5X+mceX\/M98+Oo2OEdfjwYSGW+4svviiQVdOmTaVLly7Sp08fJdwjjQkCzs04OePsjNOzed3ziE8h0y6mfVu3blUO0aaPIS5AnmnNcwBHwU8MLS1TRWNgDQbTx4+QGSP6ygfPt1HyUc96Mqt7FSX\/7VBcjcrM0RnfTZl1ZvNYjljd+5LU5ERBnmqeJIGkde1rxFtaNawt4QrvDe+P+S7ZcXSMsC6++GLp2LGjPPbYY4KlOzopyIvvCPdIY4IA8UBIODvj9Gxe9zxyH6\/4EiVKqG28kpKS5PfffxfMJiAwMy3nFSpUML8KoBPdQUt10Rg4j8FNlcqIKTXL5BdTWJn0JanJ\/24qm9XRF9ll51qxYsUy3yG7ThwjLOK4o296\/vnnBV0USnfDMAQyQgjFQppQgMB3kNEaxqc8h96IfBiV4fCMvgzhvEqVKiRxVFjBxOn7tttuk08\/\/dTRuujCnUFgw4YNKhwSU3Qc\/lmEinRNclJ+jhEWHcOIqkaNGsI0jigNKNwxVQgXYMwhOnTooPQLmDVMnjxZiOvOqAHSIuxycnKy1K1bV3CIDrecSD2HGUfXrl2FcDooSyHTSOWt84kOBNh\/k1VypuL88UafGx01d6aWjhEWLydTvpYtW0piYqI8+OCDwjVPvVVWkLC6OHbsWLn22mtVUsMwBCfqJUuWCA7QrJaVLFlSTQ9RFEMKXG\/Xrp26ph5y8IPRIPUjJhhY4FfpYHV00Q4gwG8+O3+kHaiyo0U6Rli0mkigHE1hq69Y6TzaSRjnuLg41XzaTrho9UV\/xAwCu3btkhEjRkizZs2EGQFGzjHT+DAael4Yz8TUI+E0Ft0Zw3vzWQxgWfXE\/ouIqCwwoJ\/zDAONeQe2ZeYz+hjdCATzG4CciP3Gzk6Q1Zo1a4Sdn6K75dbW3lHCwqCzUqVKgrkCeiYM4jjyHZ2TP\/MFayEJP3ciTKCHQyfBiMnMCV0Vf0mZjk6fPl3Gjx8vBNrDbmzz5s0qKgXp0eOZz+hjdCIQ6m\/ghx9+UOoJzG5QcRiGDqkUqOcdIyxWAjFRwBTBl3CPNIEq77Z7jJqSkpKkefPmZ1WNttxxxx0qmCChcxISEgTfSRYAiP9FwEKImh\/sWQ\/qL1GHQKi\/Af5IYejMSjHmOE6YCkQTyI4RVjSBFGxdcf3B+r4k6XewAAAGrElEQVRq1aqZj7AKhDLdtMg3DEONKHfs2CGQFyOuzz\/\/XFntZz6kT5xBIAKlhvobYEqISQsLRdgeGoYeYQXqBk1YgdCJ0D3+6qKvilB2OpsoRED\/BiLTaZqwIoOj31wwV2Cqh5EoiVC0or+66qqr+KolBhDQv4HIdbImrMhh6TMnwzCUoSp6LFYL8XLHEr9MmTI+0+uLOQ8Bw9C\/gUj1auQIK1I1yoH5sNUYug1MGnDBeOCBB9QWZDmwqbpJfhDQvwE\/wIR4WRNWiIAFk5ztynDWNtOy1RjxvQgngnIV1yDznj7mTAT0b8CaftWEZQ2uOleNgEbAAgQ0YVkAas7PUrdQI+AMApqwnMFdl6oR0AiEgYAmrDBA049oBDQCziCgCcsZ3HWpGoFoQcBV9dSE5aru0JXRCGgEAiGgCSsQOvqeRkAj4CoENGG5qjt0ZTQCGoFACGjCCoRO9u\/pHDQCGoEIIqAJK4Jg5pSs2APyhhtukFtuuSVT+vfvH\/XNw9Ng4sSJQsgfXKSI\/OrZKNo9ZswYz0uZ5zisDxs2TPAHzbyoT2xHQBOW7ZBHR4FETsWNyJRnnnkms+JEnCAefeaFKDiBpD766KOw446xWQghgoiKGwXNzbFV1ISVY7s28g1j9NG9e3e1M9GyZctk\/fr1wo7dOHW3adNGhX2m1IULFwrbrDFCY5u1tLQ0Idw1oxqOpGF0w4gG8iOeOWnJh735IBfukye7Hd14443SqVMntasSI5zBgwerbdIJlEjEVojo8ccfFzb2IG9GQgjnphBjnbj5BE00r\/k7MpqkPsh1110n1OHYsWNCsD22ZGM\/TV\/P6mvWI6AJy3qMo7KEzp07q8ioxNevXLmyrFu3TrXjwIEDMm3aNCGe18iRI2XUqFHC9mkQxtChQ2XLli3CpgozZswQQupAEBCQetjHB89u2LBBiLr6xRdfCLGjZs6cqVKyDRojPdIwulm5cqUQH5+NGhYtWiSQGtM8Nu\/Yu3ev7NmzR5EamzkQclhlcuaD9FdfffWZb\/8ePNtIOyEj7jCaZGRJO9mCjhj91Iu20H7CA5FOi\/0IaMKyH\/OoKHHcuHFixtpfu3Zt5t6PderUEUYq27dvF15qRiG87G3btpXvvvtOjbquuOIKFf7ZMAwhagEBDP01GsKBGCDA0qVLKwJcvXq1Ss7ohhjnRLtg70mmoWzkARmx4Sghe9BJMZpjE901a9bIpk2bhPK8442x\/x\/XVcZnPjzbSFtbtGhx5o4IG\/3269dP2HwXfR43IE22ZdPbsYGGM6IJyxnco75UpkVsWwZJ8bKz+8+sWbMUWTDNMxv4999\/B1RUM9Xq1atXJjky2krLmEKaz3sfmRKyw4x5\/dChQ4pcbr31VjWiYzrKbuIQmpkm1CP1f\/fdd6VQoUJy\/\/33q11tQs1Dp7cGAdcQljXN07lahUDJkiXl8OHDAlHxgn\/88cfy6quvqsCETNmIrMp1RmGQW65cudSuQenp6cJ1dErUjdELoyZGQIxq+vbtK0wNuedLIEmmmhAXW6c9\/PDDwiYfVapUUXoydGuM+ryfLVy4sFod9L7u6zvTTtrDVJDRnZmGER51v\/TSS81L+mgzApqwbAY8pxSHPuepp54SFOc333yzsPtP165dhakZynWmV+wtCXExFWP6xne2NGNnIXRdkBhTRkiII9NN9mrk3B9Od955pxrxoABPSUmRli1bCtNGSMTc45Lv3s9TBno4CMf7nud3iJDR1c8\/\/ywo\/SE\/jujT0JPRFkZens\/oc\/sQ0IRlH9ZRUxLRUn2RBoSEmA1Bd4SynBHS1KlThf0WDcNQ+zKuWLFC6biYUkEShmEI21ihs\/rss8\/kzTfflIEDB6pRF6MkriNMDxnVUD71MMuiXK5BGAMGDBDMCyiXPSANw1ArhBBKkyZNFKGZz5lHVhQZiTEqIw\/qS37mfY6UR124hx6N0SECGUNStJX9JbMz3aQcLeEjoAkrfOz0k+EiEOHnMJWAuBixJSUl+cwdkmrfvr2g44JAfSYKcBHD0T\/\/\/FMYJQZIpm9ZjMB5Fuevs49xBFjdY7XNShguv\/xymT17thqxBRr9sOqITZVhhL5ZKTo7pr+M\/qxsi847MAKasALjo+9qBDQCLkJAE5aLOkNXRSOQ8xCIbIs0YUUWT52bRkAjYCECmrAsBFdnrRHQCEQWAU1YkcVT56YR0AhYiIAmLAvBzX7WOgeNgEbAEwFNWJ5o6HONgEbA1QhownJ19+jKaQQ0Ap4IaMLyREOfawScQ0CXHAQC\/w8AAP\/\/\/45BCwAAAAZJREFUAwADjEg+tTn+wgAAAABJRU5ErkJggg==","height":181,"width":300}}
%---
%[output:861e219e]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:61077dba]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:933a6d6b]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:7aba7ad5]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:7aa03050]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASwAAAC1CAYAAADlY8JKAAAQAElEQVR4AezdCbRlRXU38Lp8EYkk0MaoEBG7EVAkSiMqjSJpRLRNNKCwAKfYGFEEEYyAyiDgAEhQUYJDHGiUOOCAiQZlMo0GZRBFQBSUyGAUjAaiomQp+L3f6d6d6vPOne99777X1av\/79Sp2lW1a1fVPlW7hrve78u\/IoEigSKBOSKB9VL5VyRQJFAkMEckUBTWHKmowmaRQJFASkVhrUOtoBS1SGCuS6AorLleg4X\/IoF1SAJFYa1DlV2KWiQw1yVQFNZcr8HCf5FAkwTmqV9RWPO0YkuxigTmowSKwpqPtVrKVCQwTyVQFNY8rdhSrCKB+SiBorCaarX4FQkUCUykBIrCmshqKUwVCRQJNElg1hTW4YcfnrbYYou2eMELXpB+9atfNfE8sN\/73ve+Kr+\/+Iu\/SD\/96U8HTkfEa6+9Nm233XZVesrRxG+9jBdddJGos4pRyqCXgowjP3Ik8xyjqNNeypPXOz56iYMm5IBn7UY6\/HNEe2lqSzldk1t60pV+P3w1pVX30w\/xJO0Pf\/jDa7V7fnWMOv+cn1lTWDkTTe7LLrss\/dVf\/dXQiqUp7XH4\/ed\/\/mf69a9\/vSZplfzjH\/94zfskO8bZ2Eddbp365S9\/+bRkb7vttrRkyZJEMUwL7Ogx\/kAfx49\/\/ONrMvrlL3+ZLr300jXvk+744Q9\/mL7zne+khz\/84emRj3xkV3bVzyD1oG4pP8pR\/2nKaNYVlkZ2zTXXpP\/4j\/9Yg3\/8x3+seNUIP\/vZz1buSf+DV+UIPqOSN9tss\/THf\/zH4T3rzwMPPLCS8yWXXJIe8pCHzDo\/\/TBAsV544YVVlCOPPLIqh3bj46YzCXjve9+b0HGPA4997GPTt7\/97Srvpz\/96T1loV1oHznxV77ylZHPIPL0R+k+66yzEiX7\/Oc\/P\/3Jn\/zJmqT1U\/IPKKf+jICCpqi5R4lZV1hNhXnoQx\/atpOHFqaJwTC4qYHyE4YGxGvKi58hLJpAv9OL5z3veZJJP\/jBD6qnP3fccUdVydtss43XaajnKe8mHut0aHy90AefeVnRoxEeQB8McPOPuN732GOPilc0vo7xhYt04l04iCMN8pU3P+DmJwzE598EfKIJBD9NtOFnVKLjUE4hc2EU77HHHstZlYPsq5fVf\/AR+XjiEa+rg9c86jyJF2UN\/sQTXzro10Tu4PjSl75UheI7OrQRi49aFdDmjw4vX3nhAz\/cgU75d6PNyxHp1esZW3i48sorq\/74lKc8hVdb\/NEf\/VH6sz\/7s8bwen5kyA+xPJQzBic+QI973ONSlA8denxOpMKKhqkwW265pUf1NSLQKFTlOfVHA9bhVOjUa\/VfQfkJqzym\/oh3yimnTLnW\/q9iddLc19dw99137\/lLvemmm1YVmn81o5FuvfXWedKVG6\/1PAXgURljOKwcdTo0TeUQH9Cj4Q6gl2e8j+OJ15mSufrxNc\/LYbQTX3puYeRInnV5aBd4zWWCf7ITLyAe2cX7IE+dUYcX94lPfGJ66UtfylkpVu28eunhDz7wk5PiNy9DhPFvolVGNJ7KTw7eAxRF3QxDzuS97bbbpkWLFgVp4zMv68Me9rD0gAc8oKLDYz0\/efPDS0XU5o9wdOiRDKmwJDEcCIk2pT0DKkeqhv3R+M4+++yEln8MRQkzvlgxFSC0N7\/5zciqObc4GrI4lWf2hzCiYuWFLtIkoJNOOqlSlFmURucmm2ySFixYkMKOpaOwX\/mi+jLkkYRRbPzwLj\/5Bn\/x5e2nHNIKyDPK\/M\/\/\/M+VIhUmT3lz5zBFzOnw8bGPfSz5WuZ0ndz98DqMzH3hY3qtU0Z78dFp4m8cbaYpn05+6leHR7Ns2bKkrasj7+3qRFgdyq2etBX1G2k0Tb2EoUErjrjSYzDXBuJj2kSHVzyjRysON0VbbxN5HagL7Vl86Z522mlVGzI60jelYVSMJ+DmF331kikTRfhJBw\/6fp3XWVdYmG6HqAwdghudQikINwG+4Q1vqDolBeOLpaCEJtw0wXSBWxxxuQMxhSPgCJOmykETyoO7E0xhfT3vuuuuZDrC+E55+cpIL4\/rnUJQaZ7e83DlkEY\/5cjjszNEmX0RfRnz8HG4++F1GJmzH7397W+fVgQfHR0GfM0RjKvNSLsf5B2OslI32oo0em1faI34lZ9bGuqZW1snf+6AMDTem9rAqaeeWtngKImgQxuIOjJlxaP+gfcI7\/bEkzpBpy1r05TmS17yEl4VuPmhrfNfEaz+U+d15Arr3nvvraZS\/\/qv\/5ouuOCCatSxOu\/GR2hTHTjgK49YYYy2FJoy4BdTRG6gLIxuuEPQ3IQhjDtQjxv08sGHBg++HOIQtLy5u0Ha6HOlaT6\/4YYbNkbVseQViDzrxL2UI4+Dj3inDPEQ7+N+9sLrsDL34Yl2Eh+ZvFzaC9mqt3G0mTyvbm5KM58OhnKIOor20i0d4UZnnoFII97zZx7Wrg0Y6Ubb88ynXZGWtoxHCjZ4jzBP\/TTqwpPi0Y+EGVUZXUV9S0ce8gJufmiDhrsJOa8DK6zf\/va36SMf+Uh6xjOekZ7znOekf\/u3f0v\/8z\/\/k172spclzBxyyCHJdGPp0qXpbW97W0LfxEyTn0KDMBX+3\/\/935w9oVvhe0pkAKJoJPIHSdQbGT8whdGxuGMqmg\/d+Uca3BOCtmzMFq\/x9dVZcvmZav3mN79py289YFz868A+hvIz4tBRIeqeP15NvbhnChR6fCApffIzhTSSCh7whDcfIaOh8O\/0pBxjdkIZUXid6HsJq\/M6sMI655xz0gknnJAMG2+88cZ02GGHJfPRr33ta2mfffZJX\/ziFyuFtuuuu6YPfehDCX0vDDbR\/Omf\/mllIxJWb1z1L2koDgITJk6gHjf8KUeNS8XV4YsedJ2ehswqnIKNiq6P8MT31YmleV8oSp0\/XvHMDYOUQ7xxwPTWNLdd2oPwOojMKXod3oqS0UvOT33qY0TQNPIWh6zz0dcg\/EunE3T4sP90ojPl0oc60Qirt914p1Ca2pk4TfjZz36WwrziY0npo8tl4p0CAyYFsuU3DPQN6dX7l\/foA\/X01XGd14EUlsow5TNUvOKKK9J1111XbfL0FTF\/Zqx+1KMelXbeeed0xhlnpGc+85np\/PPPT3fffXedp8Z3BQOB8iA06XqXhyEiNz7kpaOrOEbZUBzCKVCF5hZHXO5AjIDkBeHfqWMETf1pRYTNyhdVWjpLt4YUjU456o27n3LUeRnVe3RkZaLQpUue0Yi8Qz+8DiPznJ96XZI54Mc0eOHChWkcbUb6vYASoozQxihG5wxQFsK03V5GIjHFEsdHzzt3L+0MXROi\/Qk766yzqpVLbgjb2y677FIZz\/l1Q96Ooz\/qk9zaUF5nRk4+PhallKdb2sHrQArL19ZX13TvgQ98YLrf\/e6X9t1336pglFSr1VqTvzAK55ZbbmlUWBqZBo\/5QAxXFTSGoy960YuqncwSFo5WPPH5vfKVr0yMkr6sjO38CMmXHK04\/HIYPWlM\/ISjgxCsBi894d1gOKyjBB1F2xTX14o80JkayC8vB38QV\/7c3cqBZlQgB1sBNL5QENLmj1fyxA+\/AF5nQuZ5GwjZ4Qnwhx9f8lAGOb1wdLms8zYzallTQpQRnkJJcweiI3v3AfAh4G4HaTG1KIOnd7TkTv7cvcBsRdtEq51LD7j5gbo3U9D\/8MmvCSFT8SGXbSwS6JPkLH5eZ9z8go47oE9Ly0eyzutACisSzp\/rrbdearX+T1HlYYO4dQyVrsDiUwhW1ULB8AvkUyt+FBE\/7oBGDPEeT0PiOq2KYhNpN1SNuPVn3jDzzp7TKcdpU0u+Olb4K5NKCj+jLY1G\/nXe0DaVI9Ia5EnGGk49bpMc5V\/nSbwmWnyC8ByDypzstIGm\/KUvr0su+b8d\/EGPZ+E5pEG+4cfNL949xZMmdz9Qd8wC4qhTnY87R\/7h8gHQOfPwuhsf+An\/aKPkHn69Po8\/\/vg1H39x9DXtz9P7P\/3TPyU8aRPaBr9+QI7qOOKQrf6E5\/DzrNMZnNRp6ryuJ+JsQIFieNz01DA1uDpvTfGaKo1fni6hAb+8UUu\/TuvoRbeKEo5OeuLX05EXvyY6X0Q8iAvKlPvlZZc2mgDaGB5LH5ry4A\/oxY008eVd\/vJEA0EnLGj5N+Uffsovb3QQ\/tIAeQF3Pb86bT0t6bVDPa70QV5NcfKyoQNp1Gn5CQuI14+sIz3tlgylUy93Ew06ecuPW1xpBG08IxxNXV7qgZ8waUUcz4gX6UqbGy1wawue3r\/1rW9V2x7EEz9Hng\/aJtTzF78pXp2uTiO8zuusKSyFGBWsQH7gAx9Iho+Gprvttlu1avn73\/9+VFnMeDrm9eb3yhNTNEzwD6O98mpo\/AsGlwCZFlkPLr+ZjDmUwmJEN\/cGWw8oiF\/84hfVDQv8AujGWSgrkO985zvTC1\/4wmp1cscdd0wHH3xw+vd\/\/\/c0G\/9GkaevjSG5tAzXTSsor7BfGDobQgsvGE4CRdbDyW8mY683TGZWAM17Yf\/996+OsVhh854D3TD5dIpLUZpz20px6KGHJquTDJFPetKT0qc\/\/en0u9\/9rlP0iQ4zJG8acvMz\/NfRJroAc4i5SZS10bMppfpuN92dQyIeCasDKSxfd8aw008\/PfUK9OKNhOsskZtuuqnaTW++22qtMvr\/4R\/+YXrCE56QvvnNb1ajvYx8LefKlSsTY+C4YKl4VHjXu96VxoU6j7k8yChgqftzn\/tc+twUPvGJT6QVK1YkfoGrr746Xb0a3\/jGNxIYHRZcVp2DnW9y+NGPfrRWf5qJl4EUFoWw6667VnuvnO7uBejFG3WhbK\/4gz\/4g7Xu6ZHHox\/96GR6+vOf\/9zrNBC2VcjoYON43nzzzenmEcFGx3GhzmMuC1\/4wOWXX17dBWV0d8MNN6Rbbrkl8RNunx3FF0rVmb+3vvWt1SkH5z2POOKIFHjNa16TwGkIsOwNlsnBbmn4m7\/5mwRseBOLF7wgrau8qU\/9aFrnGqPHQArLecH\/+q\/\/qkYvYafq9kQv3qjLcvvtt1f7v+wv6SdtgoZ+4oyK9p577kmjwLAKrImHQcq4wQYbVCcRbGK0YRN8MBYvXpwWT2Hp0qXJCNgePSNffsL\/\/M\/\/PLngcOHChVX8Bz3oQQlsuAUbceERj3hEAu6AcGAstzE5wK4Hr3vd6xKwbYIVsF5hM7I0HD3rNU4T3SDp9BqnF7pONO3Cmvyb\/JhffKxmug8NpLCMWvbee+9qL0duq+rkRi9emqB\/RgUa+nHHHZfgFa94RQowbgMDfkCn0TkDoTC+973vJVg5NcXsBaOaGsRIiNIOhHhdeRPQqUFZHWvZYYcdqgUKm30DGiBE+XVY+3688\/dOHiANsAGW0qBwIt9OT3JbsGBBwpc4sHjx4rR4CpQaaEPeKbSFU4oMFkzFASPpwAZTShLUAdkHQv6mrGDPGxjlQYwAPT\/5xssEPQAAEABJREFUyU8mMEKESMOmTFcBO8YiD8AH4K9XkJURIiU96ji9pN2Jpl1Yk3+Tnz7Rqa7HFTaQwvIVZNCOjseGwQCsU9gMZnXuq1\/9amLI5K+y2DvEG3VB2MUcdHXwup62xv3\/\/t\/\/q3uvedfZNFzTVbDbOWDKAg5uB0x5lDVQVxhrEu7ToQODjgrLli1LAYrCSCFgtAB2RztwDtwBYRD0nsuXL0\/LpyDNPffcM9kBrhPJKyD\/HAunlIW640dZeA\/apUuXpqVT8BE68MADEx4pfKDcgJ+GTrGB+NBNNJSD\/OSLHur5LlmyJPFrUmrd0g\/F7hn1F\/WpfsEijvOv2oY2ANEutJNA+AkPaCvaOlCYFCElGnl5yhuaeFV+m0o9m8LDT3g3uk407cKa\/Jv8go+Zfg6ksCiBBz\/4wdWd4O54PvPMM9PGG29cHXY29HdExdDdF9olavfdd1\/66Ec\/muyGX6uAI3gx6vnf\/\/3f5EubJ+drSSFp+Ll\/7qbQ2jWcnK6dW9oBHQgohYBOS2EEKBII5ULZQLwLg6D3XL58eYr0POUB8m3H12z6L1g9GgpFs3RKsQFZAKUGs6HUyAx\/eOOGYWSl7QAlFKD8KL0clFgoNM9QdHXFx184iAMUXw7pR16e8odhyjGX4g6ksPIC3nnnnckB6Cc\/+cnVRXp5GLcRkAZrNIaW3yhhG4Mrih2FsA9M2o5GuDXC1gZnHfk1IWw4GrFRQMA0SYcKmMMDxQxuogDugHCIOJ777bdfUvaAEQH4YlGw5BE8DPKUBsXcTzq9xumFrhNNu7DwV1cUBnmEAv7rv\/7r6kwqRU1+FnP22muv6vC8KQj4CEFTfeZ+ZKxe5UFBQeQV7iVTI7U999wzHXbYYdVVSEaL8raHz1TYEx\/aQyDaiHYHbKeQ592vm8IJUEJAMUGu+LgpMQotQMlBKD9P74BGmSxsiAeh\/D74wQ8mgwkrvvID9igLMPU2FXUW7QyNxa5+yzkK+qEVloZnrxMl0Y4h4TaPerajGdTfSE+jNoRXCXYts1U4UmHK0mqt2urQlD6bmms1dBhf\/wBbj+ltQAcBZYWf\/OQnaRhonBYh5D3T6fSady90nWjahTX5t\/Mz1TcKJnvKAdQNsA2BUbw72RzSrddXU53nfjoi6Kw6KTAku2XBKqinbTParniOiWgrEG3FIV5w\/jPgHdwWYn\/ia1\/72vS3f\/u3FUyTA3gG5QKKj5KV1zAgT1Au5XCbCuUHlB40TXtf\/OIXJ\/xasSXXZz3rWYnfAQcckPQp5xkpd6u\/J598clI3w\/A5SNyhFZYpoQpkvLzqqqum8XD99denT33qU8mvx5g2TiPo08MRHF8Rq5IR1Q73nXbaKRGiywTddErwjMJB0+7593\/\/95UNxihtpuAL71iN5zB5it9vOr3G6YWuE830sE2Tsjb5N\/lRUj5GRkjitcNWW22Vtt9+++oqI1NmWL58eVo+BaMlMFKiYCiKGCFpswsXLmzXLNb4+xCDjwvlZUsHaGNgJRHcdmARh8LzIQQd2iifvdDHE4x4Asccc0wCiwLAbMI8YPQe4BdgLgBlAmUKKDcoF5AbrCnIgA4LD0CxGwRQgJ6Ut\/INmOzA0YZWWJi2Z4Z9yhTouc99bjrqqKMqcGskpju09v3vf\/+BGRXRl4JRn03Me+Dcc89NX\/\/615Ohr0ZjxGVDK+N\/0LR7srVpuKYQBRukSZGBPXvayyj4MXJRxzry0tU2Ne1SZ2dPA1M\/4F9Xar2MekKp6dCUFmiLPuTvf\/\/7q\/1onsBPGDMJGA1BU1lt+wjgHygmoJQDFBhQaEDxARtpwHsADYgDZBGQNsgrQPlB3o\/GMWPK029yD6Sw7KcyXI8EDcUdyHU9sq+Q1RH47ne\/mxTc1wdN0A\/ylN+73\/3utNFGG60VfS4ezaHAXfDmuVZh+nwRv990eo3TC10nmnZhTf5Nfn2KYmhySgl00CVTti2LOaaeOjFFlis1frliowyhGxNGKcAGZLpmBRFM0eCEE06obvG16ZaJw4kDK4yAXjzx5dOLzOo0FE6APc+MhwLXR0PxGXTEMTcKDdhmTzzxxMReG8qP35xRWIa7z372s5MRlK8FewxFQlO714fS+v73v5\/YBIx0jGIIeRhQen46y3w6T0deDIBWJ1utVfYqX2fD8G5Hc6TjS6ghqNyZhJVNSnjYPAdJp9c4vdB1omkX1uTfq9+w8uo1fhM\/MQLS6XV4ig3CIK\/9A4M9PyM1dPa9sVGZ5ppKaXedYLTGAE5JUWhAeVFqtlpQbEZq+gRblDBKDbRliHI2laNbWFOc3E\/6+lyHMowtaKARlv1UrjhlO7DXSqUxfJquETRubX3wHAXcGa+CNAbD+zxNgjMtZUvL\/TWUTkdzglYD0BCGMX73G9dqi+mOr2W\/cXP6QdLpNU4vdJ1o2oU1+Tf5mSL5ELId5WWeCXcTP\/3k60Okfo3S2LDYUq1wGrExZsdiAbd+w\/BuBgKUGkT7bPfUdpoUm7YMVgS1bVeZG1TYN8noDkwlZj+uZZJOXramstf9pO9YTjvexuk\/kMKijBg73\/jGN1ZbGnwBLA8zProDmgGcwIyyTB+HKQChGqVFuvW0NGxfrboiq9O1e58No3s7A3LxX2WYJwf1dd5553lUxnp+8wn6D\/joO7K0bPVm4Zia+ThDpwUDU9hKQB3+UGqUvpmIxQJwhtaCgRGaPusJ7GpmHOxwYFQFdbmbEpsmd8h2bEEDKaycm1arVZ0HO+igg6r7pxSY8fviiy9OKkGFHH300ckthoMor89\/\/vOJsKVvJJXnPQq3ESGlZ\/g9U3CLAYVuKD9MnoOk02ucXug60bQLa\/Jv8rOlwMhAJxpGRoPEbeJnptKp521FTvs0jYspqRGYo0NGR2YSS6ZsbsBtuhp04kG3fkKpAcVGUenDQP6grYJZjndKzwDBQKFb2qMOH1ph1RmyFG2bAQ2ukm01MKVzTxbbV50+3v2qjQvqAugZlN2lRQHaPR+0+dPGVEPwfo\/mWHkxTDdsnunT9qYEhtX2tjTk3fPp\/0HS6TVOL3SdaNqFNfk3+cWUo9RP820QucyswIP2FIqFcqHsQVuDz3zmM9WvVxlh6ZsQCpHCyvtVk5uCZL+KeG721Y+aaMflN3KFlTNKmdh8ZqMZI32rtcoontOE+6lPfepadwaZbhpZUVqGxaHIXD\/ignxfFEqNncCwlTAjLU8Gf18iXxzvdRC06WDTKfvi97FUZDD\/ZMDG\/I53vCO96U1vqm6yyKectgKxsXmCaV+3lVCbYfWjet8a5\/vYFBZj+Hve857EnmVq6AiAXeLtCsNI6YbFgFXHuhKzZ4WR32jrC1\/4QqLUrL6YYw9yNIewKb6CJX3dvFHkNX\/l5cofsE3DFg\/bHoACA9s7gA0LXbv+PC7\/kSos13JYjbDywUhOuRj5OBPGbXWxn4LUlRhlRpEx+ptDc5uC+iLYt9Lv0Zx+eCm0RQLjkMBcTbMXg\/84yja0wrKKx\/5kTv34xz8+OW9kXmw09frXv766odIqnyMkFM04CmFlxUFPQ16rlaaSNpnOxhdgHOUraRYJFAmsksBACstqn1U\/NiR3JrErMeQtWrQoOfBpRyz7FbuTUdKqrEbz1wZR2yiMtiLF+93vfsmGUr9WS1lZDjdtifDyLBIoEpgfEhhIYVntc2rbGb7NN9+8UlLsS1YGrRDWN3HOD1GVUhQJFAnMtgQGUljBtGnfr3\/96xTXb4T\/TD1LPkUCRQLrlgQGUlimY7b8u\/PHlM9eK1MwxnZG96Y9UeuWWEtpiwSKBMYhgYEUFkbscnW\/jx3t7m+3J8OZI0Z3Bngrhn4A0h4p9AVFAkUCRQLDSmBghZVn7DYGd2CxY9m6\/7SnPS2tv\/76ycjLgU7GebvdTSHzeJPudiGhvV6Tzuc6wV9WSLuyXz+1Au2kgpsQjPStVmckxTmLEnCrimtq1I+r0126Oar6GYnCCtnYtmD7gv1QzkRpSIzyjPNW8Zy+D9pJfrph0f0\/dtiz0U0yr+sib8wRyu3WAVequNjRHf78CmZfAhSUXfMGMM7L2tTtIoRRcDZShZUzFFNGK4cY9tNSth\/kNON226xqdFfPh7YnVMrV1gtnolxMFiNAitd9Wu77qsct76OTwKD14zybI1\/ak+0zDv26LmV0nJWUSGDQ+nFtjmM9rVYrMQnpV+pJmsNibAorZ8yU0Qir0y\/Y5PSjcDuD6F7sprTOOeec5CZFWzDQGLoaTflio8enBQTTDe8Fo5fAMPXjDJzL8XDF1OBj4xiJ94LRSGCY+rE3k0nIwX4nXh7zmMekdmd6m7lt7zsjCqt99qMPMf20YmmDKaHXc5iLVyrXyzCX30dVP27ocLLBKYdXv\/rVyUdnLstlUngfVf2YYTlAzw7sFmKbvUdRxnmnsBzHOfbYY5ObGLbccstpMrIT3sFsCq3VWnV7RD9XKk9LsHj0JYFR1A+jrq+3u8zYs\/xSUqu1qi77YqYQT5PAsPVz6623Vj9AE1N0sxUDCMf1pmU2gMe8U1hucXTY2pTOAem6TCgrFwHWd+Ozg\/RypXI9vfLenwRGUT+m9G7pdLe5DtEfB4W6kwSGrR97MI1+XcDIdmWrk\/uz2Is75dtr2LxTWN0K7gZGw9UmZdYt7twIn9tcdqsfN2\/6cRHbZ9ix2EjAjzHM7ZLPDe671Y\/BgMsXXbnsrnr34JnNuBhhFCVc5xRWP0IjaCsl\/cQptOOVwAYbbJDcQ37FFVckq88BCyjjzbmk3qsE3FfHfmWriUsR7BBotUYzZV\/nFJblVUNWQ9d6Bfg62NJQ9y\/vMyeBUj8zJ+tBcprt+lnnFNagVyoPUrklTv8SKPXTl8xmnHi262edU1jDXKk8461jHcyw1M9kV\/ps1886p7DKlcqT3SFK\/ZT66SSBdU5hEYbNhuVKZZKYTJT6mcx6Ca5ms37mrcKydcFKRdMqnzNoBxxwQCpXKkcTnPlnqZ+Zl3k\/OU5q\/cxbhdVP5RTaIoEigbkhgaKw5kY9FS6LBIoEpiRQFNaUEMr\/+S+BUsL5IYGisOZHPZZSFAmsExIoCmudqOZSyCKB+SGBorDmRz2WUhQJrBMS6ElhrROSKIUsEigSmHgJ9KSw3Mvs+mC\/guPXSlye5und9bR+0mviS1oYLBIoEpjzEuiosPyu4EEHHZTcQui3Bl1J6\/I0v4bh6d3vEbrrZv\/990\/u157zEikFKBIoEphYCTQqLDdvHnfccckvk7ih8\/jjj09+MPWGG25IlFjAux9Rfctb3pLcle6qWvEmtrSFse4SKBRFAhMsgWkKy28H7rvvvsnv8V100UXJ74q5HG3RokXJkZa8LN79Ig56dOjFy2mKu0igSKBIYFQSmKawFixYkFw360ccHv7wh\/eVD3rx+opUiIsEigSKBHqUwDSFZdRU\/4GGHtMqZEUCRQJzSgJzj9lpCisvwk9\/+tPkov8tttgidcP222+fXD7Pvhd+3lYAABAASURBVJWnUdxFAkUCRQKjkkBHheX+5qOPPjr5iR7XTfj5LAZ4Wxr81pj7z104v88++yQ\/u3T++ecn9qxRMVfSKRIoEigSyCXQUWH5gdFbbrkl+cGGL37xi+n0009Ptje8\/OUvT+6a8iOWppCPfOQjq7Avf\/nLabPNNsvTL+4igSKBIoGRSaCjwqKozjvvvOS3xawG1nPdeuut01577ZXQoPVbf7vttludbJ6+l2IVCRQJzLQEOiosO9ztr\/LzV+0YE4YGLRrvngVFAkUCRQKjlkBHhbXxxhunrbbaKq1cuTLdeeed0\/J2JMePJRp9PeABD0i\/\/e1v00033TSNrngUCRQJFAmMQgIdFdb973\/\/9KpXvapSQna9v+1tb0vsVnDWWWdVU0W\/wPviF7843XfffellL3tZtdF0FIyVNIoEJkgChZUJkcB63fhYvHhxOvPMM9Omm26anB085JBDEpxwwgnVqOuYY45Jy5YtS\/fcc09ypMfWhm5plvAigSKBIoFBJNBVYUnU4edzzz03ffvb304OPsPll1+evvWtb6UXvehFqdVqpYc85CEJzQEHHCBKQZFAkUCRwMgl0JPCuvfee9O1116bPvCBD6TTTjst2VD6u9\/9Lt16663p97\/\/\/ciZKgkWCRQJFAk0SaCrwmJYP\/TQQ9Mee+xR7bX6xCc+kW6\/\/fb0ne98Jz396U9Pxx57bGVsT+3\/lZAigSKBIoGRSKCrwnIQ2i0Mb37zm9MnP\/nJZPe7nJcsWZIOP\/zw9NnPfjatWLGCV0GRQJFAkcBYJbBep9RtBnXcZr\/99ktuGd1ggw3WkDuqc+CBBybHctw6+qtf\/WpNWHEUCRQJFAmMQwIdFZbNoDaFbrLJJpVhvc5Aq9VKwlzyV+7Bqktn3XwvpS4SGKcEOios+7BcNdNp9CRsww03TGWH+zirqaRdJFAkQAIdFZad7q6XYbu66qqr0K+B1cFLLrmksmuxZz3wgQ9cEzaTDna0TlffmMpSqqPkyYrpdtttV125w743TNpWXMm4XRnq\/Ed56\/7D8DBTcd\/3vvdVMsvLOlPliLzJmsx7LXPIG89NcaXFX7g8ek036EbZliLNeGr35Iu3D3\/4wynarPcmDNuWI99xPjsqLBn7kQn7sNiqli9fngjhqKOOSvHDEw9+8IMTmlarhXziYM+Ya3E0rIljrgeG5jr\/iqjN6DinnHKK17WgfI973OPSJHYWyuTCCy9cw+9tt92WrrnmmjXvk+744Q9\/WK3muwnYjSrd+HULS19Kd3WCodTVsbpe7T2WR1eFZVXwPe95T3L1MXuV62ScKzRVdC+WbQ7OEo6Fuz4SNcrTmFwgGLAzXxIamtVM7knG8573vLV+5OPII4+s2J0r\/FfMNvyhlECQOon68TsA2hd\/q9Dj\/KhYIJKvWYFNzvLshksvvTTZ1pPTfelLX8pfJ9rt+Bz+n\/\/85yf9NZjN64BM9Bv9R\/jHP\/7xap8l9ySiq8LCtHuxdCZnCL\/3ve+l73\/\/+9Wv6NDIG220EZKJxEMf+tA12zDqDPqS1IfFTV95XwxfjqA1\/L\/jjjvqyVXvvsj5sJubXxU4wJ+nPOUpa\/j\/wQ9+MC2F6667bq1hPj7xmxPG1y\/492wqZ5M8+OVpcYsrjQB5dFM00cl1CpAOPPaxj02vfOUrOdNdd92VcrlKU9qRj2dT+UTGp\/CA9yh3xOEnXJrSFq8TyPErX\/lKRbLtttsm8HLllVd27dDSl4\/85Bu8eAcylFYTutFqT9qVdAJRxjw9PODVB0E7ysPqbiv+LuKs+3vHa+TjqVzSFubpPQYDPkr5aLlXXqXVK6YpLOcBHXI27esV6MXrNdOZosu\/kFtuueWabDWKpukJBSwsCFWI6aSKCD+jHXS+XOHnqWHaXJv7c\/NT6WhGCTxpqPKIdPnhTWcD4dGYgsYTDX65QZmb5MFPGBrgFpc7QB677757dRIi\/No9bTY2TcnDY+Tj2BcFJkxDl6a0vQeUT+cTHn7Kgc949\/TeVG5hvQKf+EWvDQA3noxIuHtBEy9kiO96fP51vvlF+\/HUnvI6lwa54E979Q54xCtFu2jRIl5tIR7lhsBsyc0r3MPUdz+8yqtXTFNYtjI4I+iXngOf\/\/znk6kfYfodQl+eT33qU5Wfm0jRi9drpuOgU2m0u69AQGORl6mVXfncBKkc3PwNicEIkp8wNNziq3TuGEbLh02AX0AHeu9731u9Skd6wM2zcbojoAvaKdyI1sQ\/\/iDvcE106pBSw3vYaaKMOe8asgZNJmQj70hPpzBi0oFOOumkyr4pvA6H4\/mh0+Gifpo6LZ6khZaclQU\/MX3kLxwd3kPu+MAPWvzJbxiE7GOEQlFySzNGjNzdIA7e8aUsyiRO09RLGBq04oiLlsFceSPfJjrtVPnRoxWH+6UvfWkyguIOUIJRB55kJ750Hb1D30t9U2ym2NHOpYMHfa1XXoOnXp\/TFBYjOuWkQYPOZkqokfjiUFgU2fXXX59OPPHEhGm2LPF6zXSm6aJx5BVJuA5uBy8auQrzTtg6qc7qXYWoBG72D8eRuAOmMjqSBvaSl7wkvBM3P41BRa4JaOOgEDSgAIWJFF944A7U\/UIpRLjRilGLxm8UE\/7xbNo7pyFrqGhOPfXUyp6mQSpzTEnzfDVsHQK9tkFJctdBduRb91e+KGvkq8MCWnKWN7fyxPQx8sqVyhve8IY1HVO9ql\/xBoF2ou2LGyMUoxRuftqF9sHdDUaKeEenLOxJ3E1tQhga4Xl+3qFeJ\/xyRB2pBzJSVz7iOU0nN560QTSRljSi7fVa3+L3yivafjBNYeWRVdz73\/\/+6hpkq4R+dCLCGd\/94IQK+Yd\/+Ic02yMsDZRS0EEDRgz4VRE6h82tOio\/c3YVwA0Ur+Ew949\/\/OPqh2S5IZ9Oeq\/bxqJyKa18BMHNT5yg4e4HysV2GA054uIVz\/Fe54m\/+jMtDKXgGQ1SOIi3YMECzgqUFjowJag8p\/4E\/2SJJ+GAfiq4Mk5T3NxNoDTVizoSv04jHUor8qHo8ZbTRT2QaZ4X\/nNa9ap+87j9uKPDi7PLLrtUijBPkwyUQ3g31D8kUYameHlYnl9OS0bkHsjbWNCFIvfjMfV2g0a\/UBcBZYk6MWI1co16UFZhkZ96kka9DvjV0Quv9Tjd3jsqrOjg9mO1WtO3LbRaq3a6M8I7xtMts5kOJ2iQr6+iX7Xm7gW+oAzBvdCOisaXLBpRPP3Yh8bbbx74Z9cwWtH5TTGkWR\/paNAUYsgpz4dyY1SVVu4\/jFtZlAkvoPNEeka24e7lGZ2qF9p+aKLDi+NDF52VPPhBf7yKMTxMoUNhRFtRv0ZBkbqPlNGhOjfCD\/9OT3USI2WKSPk70fcS1guvvaRTp+mosOxet4vd8NL1x\/XIrpjxqzpo0NbDJ+0dn0Ym+DKKUrncEMqZ29d54cKFyZfbe71j+LqrWGE5NBwNSEeswwgjpx23W+fydcQTG1VMS+plwYcGmysRnYE\/SMMXmBsoNu\/18nk39UOTg4xjlOfpPQ83ZcFj+MUog3zJOfw9g3ed0YgqaH1Yclp5qF9x+gXlzITQLZ4PINpudMFz0MV7lCH8uz1\/9rOfpeDLR8eUSxzlVn5u0P7A9NW0kt8w6Le+5UUuvfCKtl90VFj2bjzjGc+ofhWHLSsfoXDz+\/SnP538Us5s7XTvVGAVB2gMjymh+JLwP\/vsswVV8CXVOb0Yxht5iONd5ze85VYZys0dCIOs+GjD31fG19kytGF2+M\/kU2PWqOWJB8qLO6BceARu\/jqDEZlO5R3IxJPcgBtMG8VtNxKjDH0A0IoH3AHyIjfvFJAOAt7Jmby58W66wh2dMeROubGxUlTC1Ws9H\/69gDIOfigGijhHKHM0aLuliWe8o\/P0zu1jSOly94tQeuLFXituiJFfTGX5dQO5hZFenZProPVdz6sTr3XaXt47KiwJuKfdbxHSmDvuuOOaoxXcGsaee+6ZXvGKVzQejhZ\/pqCB+lrrPIEYPquEGB4bBUSjo6SCVsfBqzA03BpsfP2lhVZn0liFB4xewiCcp8mNhp0PDfdMQaNTbp2ZnQPvnt5zHsisXsacVnmBTMhG3JAFupBbbjBGk6NJjuJCyEgejOUUHAM63smZP7rgnb9wdGRKtvLK6z\/S5N8vosPLhwzr8aMj89fJjcy524G88Z6XAW2+oOC9G\/yEXv4BlR6E\/MWneIz82vGOBvL6k4Y2QH7CyJNcB6lvaUiLIu\/Gq7wGQVeFpWEcd9xxaeXKlek1r3lNdZ2MhqvQn\/vc55LGgWaQzHuLMxyVBm9OrhIiJSMInSje48lPWLwbZdm+ER2av\/RMnzQK7wFTvvqoRBgbTZ4mv5mA8r797W9fKyu8AE\/KQGNXxnY2LPWsrFG\/yhHxpQHkoNzK770J8rDaSL5N4dLM88G7kWAud\/G88xfuHfBUT9c73oX3A6M5HV6cGMVx59Ah8cGPqYRJhLsd6ryEvCiEdnHa+R9\/\/PFJ+4twbkrCk5+769RrKB1+\/UA9kGfE4eYX757Bf17fBgP8hQe68WoARN5B3+tzmsJyqNmVyPUE2H78+MTJJ5+cFMRWBg2n1ZpujK\/HHec7XvIhe92dd4ScDwKv0\/LLabijswWt9DSQ2DKQNzzyCP+gz8OlV0eevrLUw+vvaKSNj1AkaPK8I09PtAHvEO9RXulIL\/zjKS9p58jjo1Neeec07dzyE6cOadbj5HIJekqPf522nq6RWt2GFTTt0pCmtIXLjzzIhX+OnEbZTYUjjjxy2nCTozRBnFxe3PyE1eUQ8YIX\/HCjBW78eHq3H9JTvMg7nnk+aJpQz19cfjktXqUlLOCdf9CJ041XMsN7pNHrc5rCYpvys102izYZ2jsljF68TjQlrEhgVBJgczOlAba0SNeoA7z3Y8tBXzDZEpimsGwA9VuEdrw+9alPTQ4+\/+hHP0pGXk1FMRq7+eabKzr0H\/zgB5vIil+RQC8S6IvGSBdEMr2luIC5gp+p2yBTQ3ELJlMC0xRWq9VKT37yk9MXvvCFymbFsOgrtc022yQKiTuw8847p0c\/+tHpaU97WvrIRz5S0dslP5lFLVzNNwnUpx0xJYnnoNOOYeVkqiNvfLSbJg6bx7oaf5rCCkE4jmMn+9e\/\/vVk6fTZz352Wn\/99asfTzXicsWMndamj5\/5zGcSwzZ68SKNSX9effXV6eop+BWgdpj0MhT+igTWJQm0VVghBEdwjKwY8i6++OLqVL4vhz0lF1xwQXrjG9+Ytt9++5Qf24m4M\/VkO\/ObiZZSTQnsC\/PDGO2mscEXRWzl05J8O+y6666pCe3opZfDTRY5VqxYkVZksIweoDwD3\/jGNxKwxRRctuYHfIssJkcWBi7RlwZ5DhKnq8IaJNGZjnPOOeekd77znemFL3xhcnuEPWIHH3xwckg7tflH2NAmuKt3uxFZKJx4hjKKJyWZI1dmuaI74ogjEthDQLIVAAAQAElEQVRzFPAOQWfVFuwBA7YbG2PB3jk7ywteUP3iU5HD6OWgLQ7Th7p2sgaCOa+w\/KqP\/ScOZx966KHpUY96VLIp70lPelKyC9\/xoYZyJ4K2C9cSrH0kddgg2ITFixenxQ3YZJNNql8QimdTnsP6Of4EG2ywQQI2HFiwYEF1jOhBD3pQsnsabEN5xCMekTyBnx339hc5G\/ra1742ve51r0uWxJtg5ziZsE02hfPrRNMurMm\/yU9dkpebbuU1k2jiZ5D8B0mn1zi90HWiaRfW5N\/kp34uv\/zyqh+pp5nCnFdYN910U3IDA8XTaq3aE8aO9oQnPCF985vf7Hg7pAPbpo\/Lly9Py2vQmZtgJNcEG+FymJLWEeFnnnlmteFWZ4y08rwoikAozVCS\/SpECg4oOMd07IZeMKXgKHMLJEZv73rXuxL4sRE3yoI7wCl9crQS1wQ7uI3kmmjahTX5t\/Mjgx122KHaLNmU\/7j8mvgZJK9B0uk1Ti90nWjahTX5N\/mZxcyUksrzmfMKi7LSIZ17zAtm9dItqD\/\/+c9z77Xczq5ReCtXrkwra4gpXf1pC0cTHE7Ncc8996Q6KAqgdGy28+Me+ISlS5empavhh2sDvmTgKweUHZj6AjcIA7QWP8CmRiNOxzrWKnjtJaa3yhrTVcN9pxie9axnpbDXmb6G\/Q2t8m666aZVavWy8mwKa\/Jv8iMnZTAarKc97vcmfgbJc5B0eo3TC10nmnZhTf51P+1fv+M\/05jzCktnMy3q1imbBGsUYVpoCboOx02aEB26\/jzjjDNSDp27jjycOzq\/pxFPIOxdnuxwYBczuMoHfvKTnySwsACmfGDa98xnPjOBEYqRG8Vje4onOC7il458OaGbUiNjwE+Umx3NCjGF5gmHHXZYcuAblCVGmPjsF\/KzidnB7X7jFvpVbWNcctAGfNCa+tS4\/XpWWKYSVmicOyMIF\/bBuBmcT+mTYQ6dMWCkF3DMIWC7CFiRBRskgW0JLDh4P\/\/88xOwK9xwww0JpK2OwGjHqM6octGiRck2FXuEwCbhj370o9WChadpqpEaZWZKajraqR6MtECeoeQ\/9KEPVdNeCtIiAFCU8KY3vam6rVbDx7M2pRx4xzNIr9Vadd8a3mcSRndk5gaSYfIdJJ1e4\/RC14mmXViTf93Ph1D76NQmxhXWVWHZMuBGUVsXrLRYlXLo01XJbBc6jS\/8uBjslq5Dl7\/5zW8Se1Sd1lSx03YLioBtgvBBRbSDztsE064m6OQ5Fi5cmBauxmabbVbdYqkh1Hnu991UhRLUyYHS0PGBMmCXghjtqS\/TX8dafIDAaEY67Fx4wzdFpbzkQoEZLYUNzjs5WZV1owH6XvimhMCUIka2eLOPz+jN7bZsamDhBJ773OcmozjYf\/\/9Uw50ATa4ANtcDuXNwUZXB\/te4O67765uUCWPYcCW6tfT+0mjMc7qRZY8nV7oOtG0C2vyr\/tpxxNrw3I28N3vfnfScN7xjndUHU3jtPN9p512Sg5Da2z8ZgNWwnwN77rrrrWy1yBNkdiL1grIXnR0NhKKA1REO+iUTQi7U\/2ps+fQwQOmY1bpKAM3YeTgFwj6SCdXjHgJXvEOWdHaOsmJYvPRiWmwkY7OfsIJJ1TGd27K7pZbbknkSMFIkCxB3hYpXD30lre8pdpSQqFBrtRMR6MMy5YtS+KJL61BQLHmYEcLaIMB5ckRyjqeprN1hJ3O0\/TWSJCS3LVhHx6adsjTpUz1Hc\/Iu+m5YsWKtGI1KFr14hl+nlG2eK6csrn6OKmfkEH9KYyyUee53LgpPyNtz7wuvNf9m\/zyODPp7jjC0qHZIthD3vrWtyY2n1Zr1UqcpXJ2GKtsvpCUxkwyHnmxvxi2uxY2Rnr4\/trXvpaschnWB23T00hEhzTCmARoHAGdGxjlwWgwoE4YpcF9ZEBBeAJ\/8CUEcqLAoUkO4adxg8av45ji6fyUGVBm3nUYQEd+Rk7kt2DBgmqLBX4p8XzxwKKABQKLBcAdEAaUNb733nvvRMmFksZ\/gL0yEHzP5FOHbwfyaEIomqYneXZDk6LjlyvIJnc7xdqkiPm1o+efp29kbAYzk3KXV0eF5XIyqwH27tjxLkIOfsIYgZumZDntuNwOa++1116JzcTBazvwfdFMOTT6VmuVgm2Xf3RAdrmZgg7u3iXPYfIUv56O6TGYikB0ckqLkqNEnP1037uDwW6UZagHH6ReFJp8KTMIhebjpQN5glGB6aczqTppLBzYvc+cIA0fGKDomB4WTCk7fODTh9CZVgrPKI0yDrCLBSwm5Aj\/eBop5TC6zRGj1\/xJNkwgprs5QpZNz1Cg+VN52rW7SfVvp4j51xXxxCksDLkHvd3mS0IXhq7V6qwY0DaBDeUv\/\/Ivq9HbVlttlTRQv1qS0+Y0LlA77bTTkk4ZNGwpMT19znOekxin2Too0xREbZ72QukQRmkzBUqBojV6GibPQdKJOIzv5K1jGskABb98+fJqQ6lOrROTbYxyTEG7dUKjWzDtpIxNPS0gsBeqF8e7jH59oSk0CwaM7vbMCTPiNdXxwXGnlY4iLTCKD5jqQF1+FhZyGDXkUMYcFh3qUHanBlxZc8wxx6SAdtcOFivqoEhdiXzuuedWCxpGlTnq9N4dMTPS9GQrbAcjUXwyLYQij6f2DOou6tB7QF37IFDG3GC6DrbaUMjcgVDC2mtAd9L3PWcSHUdYplOmVb6iRlp1xigWYQqJth7e7d0yvSM0DL1sZYyupiP8LGmLf+ONNyaV415wN5weddRRyTSCovF1RqNBOKRtQYDtxYjr9NNP73g0RzwwtdURYxo2E0+yMmXS+YfJb5B0eo2jTjRYim3p6v1hGry6YHPzBB0iFBp6slQusu0FlBuod9NKyorSuuqqqxIbjFGcOgVKLqCtgFFdDn45gj6e0snhY1gHBWvUd\/PNN1c84AMoz3ZAX0er1Uqd6pmM6yDvJUuWJE\/ybAdXk7uo0Grvsin7YA4fHWBjtNXE03uAYqOE2R+5IRSjfmNbSrx7unUXwj7pSalOnMJqtVrpgAMOqNodA6SGoVJ8CRgm+WloaIyyKsIe\/1A2jjtsvvnmSbqPecxjqmtq2Mocm\/FFlhTh6GQE5MtpBOar4qpcdIMezZF2weASoJRAhwqFRnk1KTV+wkKxeYpHuYF0Budk7ZjaYw5KJ0d9WkMhNsGHuI5OdiamhSaw+7VDE33u1ym\/Om\/5O9tiJ9RlEO+5nHJ3Ls9wry31mXvrOMLChhEIxWLaxv7A1sAG4atkGmb\/jBEW2n5g6dioTcOlkCKuS9e8+5KxixGmfEyhgoadQ9h1112X7F2SziBHc6Tna64sMwn2HI3YV3+YfAdJp9c4vdB1ohGm8xnNqEsfOqNJHzYKwrupBzh+oy6MRtivjDCMPMS1KZj\/JqvPakojh3QCt99+e5IXGLXlkP6kITp\/u2euNOpu\/aIdyLcTcuWWu9spSPVYh7pRJzMt064KC0OUhe0L3\/3ud6srT3RyQ2RTM0ugaPoF25j9XUZLeVyX6Ltri0GfUnJRvcab07BbUGqW3SkrnaDfoznSZIhWEfaXzSScv9M4LA4Mk+8g6fQapxe6TjTtwpr8635unjDSpngY8MnJKBzy6R43pR\/QiQJWrnOQdx15eLgjfv402m9CO4XBnw2uHZSrHXIlW3fnfWAS3PqPfjSTvPSksAjOapQvGcVA2zP4mRNfccUVba9P7lSQVquVpOULGnSmeObVjKnu4KKw5G2vVdDUnypeGgyD9bBO7wTNDmb0WPCxtrc2zGfZ+EWYOuw1rMPO\/Caw\/bQDO2w7mCK3A\/trOxh9dkK7ePzb5cffdL0Jpu5NMCsCOkA\/6tTPRh3WVWExQFpRsGLiN9YYwd35zigqzNfR6s4wjJlmWhVhE7MyxNC39dZbD5NkT3EJ2\/SjYMmM34hQZD56mTPNtANbYTtQPk0I22T9GcpNXj11tBESZQpreqpWASzNbrTRRtXKnLuqv\/zlLyfTtX\/5l39JNmsqqF25lpunp7DKxzEQN4EGKL9VISlZaXSXlSmnUZVlbvtvhLNV2EtkSdt7HfgY5mhOPb3yXiRQJDDZEuiosNiSGLZpVMqGAvNOSTGOUxa0b7eNoxRRblx2rTKxsD\/Z7+IKGCuP7BWM\/MLAhXSmevbzeA94t6mVwdZ0kbJkuIxwTzY2e45mwzAo\/4IigSKB0Uugo8Ky9YCSYtSWtb1RNgLabBh+\/LvBKMnoLGDEJm3TQHFtXTCqarXW3nzKsG4fGIMn5YkW7LmizCgsYIQ32pOmcHYvmxPFlQa\/giKBIoG5L4GOCsuv4hjx2DrAzmRbg31PdpsrOj8KjMJAy69X2CRoe4TtCKZ2VgNzUDqtVivZlcwY7+S+YzeUnN3DDIlWL4F70KM5vfI7z+hKcYoE5qQEOiosq28UBrsSI+nf\/d3fJcdd3NRgamjzqOVfRnm0\/UjAaM00ztK0tOs4++yzq+QY9uy+9Us9dvda1WH0t0JZEUz94XYwUxgaCtYpeXGngsv\/IoEigXkigY4KSxldK2NEY4WBYnKolHKynYBicPMghYG2H9jZbqWRImqC812RHmV23nnnVZtEba+ws96oLMK5+QnDE1pxIrw8iwSKBOaHBLoqrFarlZzot7HOGSNbARTdVI6CoCgoDH7zDRRqLBDMt7LN5fIwF7z+9a9PNi7aJe8jyjwxl8s0Tt5nOm1bk\/bbb7+qftimmXFGVT9dFVa3wlJiRl6meN1o50o4+9qJJ56YbPyzGjlX+F5X+HS7g7I6\/uPMHPOERRZ+BbMvAQrKBlc7Axz9sSBmJ8EoOOuqsCgiO1pdRWJrQx2+blYBO11FPApGB0nj1FNPTfmer0iDtifUJz7xiUl5KFy3ZcYqo7K4\/tl0OOKU5+glMGj9OGlhk7GRva01zho6NjZ6DtftFAetH\/es2QrVarWSLUf6lXoahTS7Kiwd27YCGtM2fRkzbLvkjC3LHiuGc8dsRsHQqNKwIdXdQ03pObBtN737gtCYWhhN+WKjtxXCNNh0w3vB6CUwTP04DqMt4srJCx8bewO9F4xGAsPUj1tV1l9\/\/eoXt3fZZZfEXj2q\/ZAdFZazfOxU5qNW6hw+tY2ABvXOuG0E5os3GjENn4q7sRjc2dgIvZ6ibRn9\/FJ0PX55H04Co6ofFzhaFbbg8+pXv7qylwzHWYlNAqOqH4MZ51DZga+55prk9gjpD4uOCstwTgenHVutVrLXyuZPtyTImAHedbtOwaPlN9uwlcGeLQebXYJW58cqoh32FFqrtWqjqsPWpoBuvbQXrB6nvI9OAqOoH0Zdt1zYn8ee5ZbZVmtVXY6O03UzpWHr59Zbb00u2YwputmKAYSbK0Yh0Y4Ky25218DY7S4zbsrL0RjvQJMyqBmNeZ9tsLWZrprSOdZT54eyUq76FJYd5Be\/+EVyTKgep7yPTgKjqB9T+p133jm5R4\/gugAABqtJREFUU0yHGB13JaVh64ceMPq1oZztygkV1+2wF49Cuh0VliM0CmCY6GaG++67L7mqmE0LI4zXRiwUQKs1N75w9o9Rsk3KbBQCLWkMJ4Fu9eOKIyNhd2CxY7GRgGn+cDmnlEoCXSXQrX7oAnszbTZ3Z7wrnM1mHv\/4x3dNuxeCjgpL5n4yyr4X18hcf\/31ifHdRk9TQQzZn6XhzEcFQNBWSnoRZKGZGQm4wcOPV7iHzXJ5wALKzHBQcukmAYMa9itbTQxu3D3fao1mQNNRYWGMhV+j8BNabFbmuKeddlri1nj8Oo0jO63WaBiS5zhhldOQ1dC1ng8FbUtD3b+8z5wESv3MnKwHyWm266erwlIo+6zMQdkLWq1W8rNJfuWGImPgNsVCNxdQrqOZ7Foq9VPqp5MEuioshjPbA+xbYv1vgu0Ov5gyWHfKaFLCynU0k1ITzXyU+mmWy6T4znb9dFVYjJtW3Oy7crNoE+zHYgydFKF24sM+snIdTScJzW5YqZ\/ZlX+33Ge7fjoqLHYeCsv9VzZ+MbY3QZj9Wd0KOynhNhuW62gmpTam81HqZ7pMJslnNuuno8KyGdTG0d133z25EnmShNaNF3Y1KxVNq3zOoLllwi5+2zKMEG1u65ZmCQ8JDP8s9TO8DMeZwqTWT0eFxdhug2VsHB2ngEraRQJFAkUC3STQUWFtvPHGad99902OP9gh3i2xEl4kUCRQJDBOCUxTWI6nWPWL1UDb6u++++60bNmyZBNp+OdP9OKNk9GSdpFAkcCsSGCiMp2msKz2senEaqBzWw6ZUloXXnhhCv\/8iV68iSpZYaZIoEhg3klgmsKy2mfVr2k1sJ0fevHmnXRKgYoEigQmSgLTFFbOnVVC1604S5j7F3eRQJFAkcBsSKBRYbmFgV3KucElS5ak7bbbLh100EHJZX2zweQczrOwXiRQJDBCCTQqrBUrViS3OW6++eZpn332qW5zZL86+uijk4PDqfwrEigSKBKYBQlMU1h2tzOiuyDNL16cfPLJyV1DbmS4\/PLL06huDpyFspYsiwSKBOa4BKYpLHYru9t32mmnZLer8rVareT3xRyEvuOOO3gVFAkUCdQkUF7HL4FpCqtdluutt15qtebGnVftylD8iwSKBOa2BNab2+wX7iddAhZwnOf0009uprXqHDw78vWqV70qHX\/88enaa6+tFnf8MG+E508r1X54op5GTlPc818CRWHN\/zqe1RL6wRK\/CeDmWpc9urEyGPLrNxSVCyHDrzyLBDpJoK3CsrPd1zDArsWG5QhO+MXTdod77723Uz5dwwrB\/JSARRyjI0rJHfl+Ui1K6odNXEu99dZbh1d5Fgl0lEBbhXXGGWcke7AC7m7X8A4\/\/PC1\/IXvvffe5eexOop53Qw0vdtjjz3SL3\/5y3TKKaekfDrn4+eKbb+m4qfj+pXQRRddlLbYYotG5Pn0m26hn2wJTFNYhuxsCm4Y7RXoxZvsohbuZloCu+22WzryyCOTHyvZc889K1tVtBM\/F+XnuvxEV6v1f4s59ZF9PopnD4sybLvttqnePm1uNmJbuHBh9aO\/QVue80cC0xSWIbvhux8j7RXoxZs\/YiklGYUE\/Kaln4JzYaJp35p2MpX4jTfeOPU3pcWLF1fP+FMf2RvBA+V31VVXBVnadNNNU94+pXP++ecnP5Rig3NsyVkToTjmhQSmKax5UapSiImXwMqVK9M222xTKZ6c2YMPPjhddtll03DxxRenHXbYISdd4zbyouhuu+22ahRHOa4JLI55JYGisOZVdc6NwljAcWqCEd4ULud6ww03TG7+qMOPHxippdo\/trAzzzwz+e2BQw89NLm3rUZSXueRBIrCmkeVOVeK4h59tip2qGF5dsGk0ZWp5yh\/YXhYvkr8kMBon0VhjVaeJbUeJOD+NPYth+t7IG9L4tpuRn2jsZNOOimFQb9thBIw5yVQFNacr8K5VQD7si699NJkddCPnAzKPbuVg\/nsVg7mP+xhDxs0qRJvDkmgKKw5VFnzgVW3fdhovP322w9VHFcguVVkxx13TOxYfiglcMEFF6Q777xzqPRL5MmUQFFYk1kvq7maf48rr7wymQouWrRoqMLdcMMNlaL66le\/mpxHPOSQQ1LgiCOOSH6HYKgMSuSJlEBRWBNZLfOHKYeeHcE58MADq0J5fuxjH1tzdVHlOfWnTjfltdZ\/+6rEY\/9is3Kgut1vDMhPemslUF7mhQSKwpoX1VgKUSSwbkigKKx1o55LKSdfAoXDHiTw\/wEAAP\/\/7aWnuQAAAAZJREFUAwClFWpNQUVdwgAAAABJRU5ErkJggg==","height":181,"width":300}}
%---
%[output:749b0793]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASwAAAC1CAYAAADlY8JKAAAQAElEQVR4AeydB5wURfbHX\/M\/OERUlkMliC6CCiqIgrokJaioBBGVAxXjSTwUOXXNZBVMeAdKEFRQUQwkARUEUZKIEj2iBwgHyCEiZ8D851tae83QMzszOzPdvfv48La6qquqq37V\/abq1Xuviv2q\/xQBRUARCAkCxUT\/KQKKgCIQEgSUYYVkoLSZioAiIKIMqwi9BdpVRSDsCCjDCvsIavsVgSKEgDKsIjTY2lVFIOwIKMMK+whq+xUBLwQKaZoyrEI6sNotRaAwIqAMqzCOqvZJESikCCjDKqQDq91SBAojAsqwvEZV0xQBRSCQCCjDCuSwaKMUAUXACwFlWF6oaJoioAgEEgFlWIEclmA0auXKlXL22WdLo0aN5JxzzpE6depI+\/btZdu2bXE3cPjw4TJr1qy48pOvevXqBzyvX79+8vXXX8s333wjd999t6xevTquuuLPpDnDhIAyrDCNlg9trV27tsyYMUPee+89Wbx4sVStWtXE09WUM844w9TP8+bNm2ce89BDD8kf\/\/hHeeCBB6RGjRomLZk\/v\/76q\/z888\/JFNUyAUFAGVZABiIMzfjhhx9k7969ctRRR8mPP\/4ogwcPNjMwZmFckwYNGjRIzjzzTDn\/\/PPl448\/Nl2DWTz\/\/PNmptawYUOxMydzM8qfQw45RK677jr55JNPZP369dK5c2dh1vff\/\/5XevXqJTk5OXLWWWfJI488YtpD+s0332zSrr76arnppptMfmZ53bp1k1atWsnChQtlzpw5ph3169eXli1byqpVq2Tnzp1y2WWXSe\/evYX2MZMcO3asNG3a1NQ3e\/bsKK3U5EwioAwrk2iH8FmLFi2SWrVqyfHHHy+nnHKK\/Oc\/\/zEf+5tvvilbtmwRZkHQ7t275fXXXxfSt27damZkxGFwdJs8LOfeeecdef\/99+XQQw+VyZMncysmlS1bVsqVKycwI5vxgw8+kGrVqhnmw\/PWrl0r27dvl9GjR5sZGPeZle3atcsWEdo3fvx405e3335bYJ4LFiwwDJE6yPjVV19J3bp1TfsqVqwo1Dtz5kx5+OGHTVt\/+uknsin5iEABGZaPLddHZwQBZjErVqyQf\/3rX2aWw+zj\/vvvl\/nz58vFF18sxYsXN3TeeefJhx9+KHPnzjXpzI4OO+wwM1uhoTARGMZJJ51klpXDhg2TpUuXcithatKkiZFzTZgwwSwTYYSff\/65rFu3Tpg1OY4jlSpVkpo1a+bV3bhxY6E9hx9+uNxxxx0m7+OPPy4vvPCC7Nixw+Rj5shs0XEcKVasmGn7\/\/3f\/wlMc9++fQKJ\/vMVAWVYvsIfrofz8Z577rnmw\/3uu+8Mo7I9YCkI87LxyBChOYwCxgfBZPr06ROZ7aA4MyNmV8yy7M0xY8bIiBEjBOZ36623mpkfS07kUzAa8nHNEpZrN7Fh0LVrV2H21a5dO+nUqZP79gHXsfpzQEaNZAwBZVgZgzr8D4IpsLQrWbKkkV2xqwejgqZPn25mN8iuuCbtyy+\/FJaA9BxZE0swmA\/MDlkRS0PuRSPyPfXUU3LyyScbuZnNx1KtRYsWwoYADAjmx4yOZRzto52bN28WZoa2jA2\/+OILM9NCdsXMybbP3tcw2Agowwr2+PjeumXLlslFF12E3Erq1atnloK5ublyxRVXmKUSS0SocuXKcuGFF0rr1q1NOkuzG264QSpUqGD6wJIRZkbI8gxmwbW56fqDkN4+j3phRAjYXVnMswcMGGCWbMiYkLG99NJLRsgOE8zZL4xHBeLoo492FzPXyL6ysrKEmeKVV15pmOHy5cvNEtFk0D+BRkAZVqCHx9\/GIQNC9gQTQM0AATzCamYyLJdY4nEf4po0GEzfvn2NPGvixIny5JNPCoyJez169BDyQja\/u4fkW7NmjRF687yPPvpIkJeVLl1aIJaBtAk5EzI0ZlMwpqFDh8rAgQOFGRzMlPqfeOIJI4diKdmlSxeBeBbtY0eTumkfTBWmB3N88cUX82Zy7DzSHsrwTJ5NG4gr+YeAMiz\/sNcnpxgBlE5hYA0aNBDkU6g22Bleih+l1fmEgDIsn4DXx6YegVNPPVWmTZtmlq3M0NhNTP1TikaNQe2lMqygjoy2SxFQBA5CQBnWQZBogiKgCAQVAWVYQR0ZbZcioAgchIAyrIMgKXiC1qAIKALpQUAZVnpw1VoVAUUgDQgow0oDqFqlIqAIpAcBZVjpwVVrLSoIaD8zioAyrIzCrQ9TBBSBgiAQSIaFS1x8KeFQDde8ENekca8gHdayioAiEF4EAsWwsLLHbu2aa64xPopwifvqq68KhG0XzApziylTpoQXcW25IqAIJI2Avwwrotm4HsHn0iuvvCK4tMXfEU7VoCpVqgiM7LXXXsvzABBRXKOKgCJQyBEIFMPCGySW+HiOxBUv\/pCwoMeglZA4DA03JYV8XLR7ioAi4IFAoBgWPrNx7HbttdeawwFw\/YHzf5y\/4TLkscceE5aNHv3QJEVAESgCCASKYeGhktNLsLhnWYiLW05XadOmjTllBd\/buLYtAuNSCLuoXVIECo5AoBgWsyeWhThZww0vZ9H94Q9\/ML1kKUicPCZB\/ygCikCRQyBQDKvIoa8dVgQUgYQQCBzD4kQVXOxyrtynn34qhAjdSeNeQr3TzIqAIuAHAml7ZqAYFufG4febQwqaNWtm\/HkT4vKWNO6RJ21oaMWKgCIQaAQCxbBKlCghuLlFbcGLuEeeQCOqjVMEFIG0IRAohsWhmcyiLr\/8cqPWYE1yMMs5\/\/zzZciQIcKBnGlDQytWBBSBQCMQKIZ15JFHCufLvfXWWwKDmjFjhjnyCXMd7AhBkuOiCAsxadcUAUUgCgKBYli2jZjooNUOA7NpHIpJGvdsmoaKgCJQtBAIJMM64ogj5PvvvxcOr9y2bZtA48aNExRJVehetF5Q7a0i4EYgkAwLBdEHH3xQMNVp3769OZp869at0r9\/f0Gp1N2BeK5RNh07dqxg4sMJv88995z8\/PPP8RTVPIpAWhHQyhNDIFAMiyUf5jd0AY333Nxcee+992T+\/PkycOBAycrK4pbYPCYSx5+PP\/5YZs6cKcjGkIthn\/jhhx\/GUVKzKAKKQJAQCBTDwgxn6tSp0qVLF8HYGf9XFiyu586da+6Rx6bHE8Kc8AIBw2NJicrEkiVL4imqeRQBRSBACASKYbEDeNNNN8l9990n06dPlyZNmsjxxx9v6IILLjAzLYyhyZMIhhs2bJBjjjkmr0jVqlVl06ZNeXEMqteuXSuxaNGiRQK58xC35E73uvbKZ9MIvcq408gDeaVFprvz2GvyQDZOSNwS8Vjklc+mEcYqyz3yQFxbIm7JpkULvfLZNMJo5Ww6eSAbJyRuiXgs8spn0whjleUeeSCuLRG3ZNOihV75bBphtHI2nTyQjRMSt0Q8FnnlQ0yT9xFl6CJQDMv2uVKlStK3b19hZoQ5DrRgwQLBRAdnfjZfIiFyMa\/8gI7bmsGDB0ssuvLKKwVy5yFuyZ3ude2Vz6YRepVxp5EH8kqLTHfnsdfkgWyckLgl4rHIK59NI4xVlnvkgbi2RNySTYsWeuWzaYTRytl08kA2TkjcEvFY5JXPphHGKss98kBcWyJuyaZFC935nnrqKRk1apR5H93peOUdPny4PPnkkwe9y+589hk2jdCmRQvJA7nv33777cL34\/VdpSstkAwr1Z3Nzs4W7BJtvVzXqFHDRAF80qRJZvDvuOMOiUY4EITc94lbcqfbaxRfly5daur2ymfTCG2ZaCF5IPd94pbc6V7XXvlsGqFXGXcaeaDINDZH8F\/Wq1evqNhRhrIQ15aIW7Jp0UKvfDaNMFo5m04eyMYJiVsiHo0YR16Whx9++IA+2rKE0cradPJANk5I3BLxWOTOd8opp0jFihXFphFSFlFK9erVzeYScTeRB\/JKI71du3ZSq1YtiTaO5IFseTBBbMP3AzaZoiLBsOrVqycwDnS4IK5r166dh\/G+ffvk0EMPFVwyR6OcnByB3PeJW3Kn2+tzzz1XLrnkEmEJ6pXPphHaMtFC8kDu+8QtudO9rr3y2TRCrzLuNPJAkWl\/\/vOfBaeLfETue5HXlIXc6cQtudO9rr3y2TRCrzLuNPJAXmmR6e48XDOOMOU6deoc8I5QzhL5YpE73yFHHmfqsWmEscpyjzwQ1+XKlTPvLnFLpMOsSpcu7fkuu\/ORF7JphM2bN5dbbrlFoo0jeSDKWTKNyPCfIsGwzjjjDOGlu+iii4SBQTZGWrqxLl++vPFDT5juZxX6+n3sIOPHeQKEBWnGZ7v3yaC3NkrtAQsNcV2Q+opi2cAyrFWrVsmFF15oGA1qCJ06dTIKpMkMkuM4hnHMmzdPkIVx8o7jOMlUpWUUgaQQgFl1H79axn+4Q4Z1qCEdziy\/n3ltEtKSqrCIFgokw0IfCy33nj17CgJ45E2tWrUSjv3iXhEdq0B2m+X0xo0bhTCQDQxAo2BWrZ9cKlu+3CdTup1umFVu8yrmev6ne\/Yzro2+t5LxC8M4BpJhIWdiV4+1MiOJe2SWcbhN5h5pSopAGBBgRsUSkLYuu7eeHFu2JJeGGlYrI0Pb19jPsDbJvA17TJr+AYHoFEiGhXInSqTbt2\/Pa\/nq1asFfan8THNY8uH0DzOciy++WFhaqmlOHowpv+BHhDMjCVNeeYgrhAG1GrbULPlym2cLzMqrOzAt7jMDo4xXnkykMX5hGMdAMiyYEnKmPn36yEcffSQIy\/\/617\/KrbfeKjCzaAPIqTv\/+Mc\/BH2UxYsXS9euXeXvf\/+7UfhU05xoqBUsPSxLiYL10rs0Sz1LMBtmU8ikmFHBgFgCIq\/K3b\/8867ht1TuI9OiDPX8lprZv2EZx0AyLIYK76LY\/i1cuFDefPNNIyw\/7bTTuBWVOHw1OztbTjjhBJOHOr744gujIR920xw+DPtB8FHw682HYalsrzniB1W8e6HU+ccmIfTj+Zl4Jn3z6qPFnhBmw7ggk2pQtYyZUTGrghGZlzGfPzA2yv31pdX5Lg8vvfRSQV8qnyoL5e1AMSwYTvv27QWlNAhVhDZt2ggeR5FhcY880UbixBNPFBQZ0UX58ccfBad\/+NRCuS2WaQ71oQSHuQ6\/NKkk3OSwUZBsnZZJ8VHwQcxdu0vWbdsjR5USqVOppFxas4yhW84tL9AjbbLlkTZKqcRg4EUV5K5Gh8lDLY85CNsJN1QXaNFtp8u2B+oJ4aOXVjHjk+iYv3JjDalw2B8EpsUYJ1oe90t4IeHdT7Qs30CFChUIzAZKrPJ8J\/\/+979N3kz\/CRTDKlu2rLCkmzBhgrRu3VpuvPFG44EUL6QsEZFJWaU5gNq5c6fA1LA3vO2220gytG7dOkFzF9soXNIgD0OIb25G+fPEE08IbmeQm6WK8CoBg\/38888l0To\/WrtFOo9baWQgMKlOZ5WRkW3Ly9RrjzHhvY3LCHRNrZLipiaVRZRSzyhlIAAAEABJREFUj0FOue\/knIo\/H4Rt1UP3CVT8+y8THmOvd+LJ1uXkyENE2o5cGbU+vg9EHV7l+YGE2XjdS1Ua3wlmOVE+pbQmB4phsRvIjIiQj52ZFSYIEHZMK1asMIJ3iwh2hXhwwNYQuZUVrnfv3t1MmbG5oj6WiZjj2HJcoyph44SYXaDNXGH\/r0yqCC+pPB+Fw0TqHLtin3SfukuWf\/6T2fpecueZ0qdNDWlZt4okUk8m8pYpU0b4SPCEkYnn+fGMZMcx2bZO7X66VDmytLR6bqvneCPj5Uc4sn6+B36YsdqIvJdfPJFx5DtBK57vJtMUKIZlO1+iRAnDmPCyYNOYpbBLCDOzaZEhSz9cz4wcOVIaNWokNm9+pjnUg74XjI3dklQRLxYvULz17fxW5Im522Xiyj1GVwcZCLtI8Zb3I1+iffSjjQV9ph99tEzritGrJbL9MCve2ZIlSx5wj3byzhcrVuyA9MjyXnHKxvuu8p0gE6YNmaZAMixcJMPFu3XrZhgPzKdDhw5GW51lYzSQYHCoMWBGgQwMQu517LHHmqUju42ZNM2J1k6vdITqyKiQWbFrBHnlC1oaL38YtsODhls87UEQTz42WAgtpUPoHpZxDCTDYmAQsrPcw0Icf+6cnNO0aVNuRSXKoK9FXjyVQsi\/WJbBxIJqmgOzYpeJbfApv2tCR+2k3igyCKBkCtPivYhkWkUGhIiOBpJhIahmZoQC6FVXXWVmVhz7RRr3IvoQ6qhlVnQCZsVLynVYCAFvGEw6woJnZDt5H3gvUJeAaWEw3Wjgbyos7BwTjyyTTDws4xhIhsWyj93CV199VSCclbVs2VJYNyPcTWZAPMr4nmSZVeWskkZvh5fT90ZpAwKHAO8FMy2Y1qC3NsknX\/zWRN4f4jCy31IK\/99AMiwEhyzj2PWAsClEUe6XX36RvXv3FopR4WVjGYiyIALWsHYqLLKPsOIbT7thZMg+48kbLU9YxjGQDMsL1C1btkgiymo46cPDA7paVt0B+8IgHPMFs2I6jxY0v5xe\/dU0RcCNwIuL\/2dX606316laGtr6ghoGkmHBZKxCKEqhELIsnO5l\/X7UVyxAkXMNGjTIaOySL0jHfFlmlds8W3LzsTGj7UGnsMg+go5jfu1jFpVfnoLcD8s4BpJh\/elPf5IpU6YYP+wohUKcTciMiWVhrIHBLGHo0KGCVjzLSvJymIWVf2E87XXMVyZMc8Yt3CLMrDChueXcCoah8qKEnVAcLYj5URj673cfkWPxLkejn376SQpimsMYxNtHNc1xjQLnD2JSc8899wjAMNuC1qxZI7fddptgzOzKftDlrFmzpFSpUsKuor2JflZ+toTpNs0ZOXu93PLKBsHEBlOaVJlJ+F0PHjJQONyzZ09UUxK\/21jQ52N1wawd5eWC1pVs+RvOKG1fZ8+wZfXSBn+YDswn0eckMo5qmuMaAmZDTz\/9tPzzn\/80xxUNGTJEhuwndLHOO+88YQfRZoeR2aUjzAxNdzw74E4Zwb3NR8hHRRiN0mma89zyfdJn1q79S8BswcQmPzMJvV\/B0yTFL1wybZrj1c9OTU8QNmi83l9mX7xXbFDxnidjmuP1zGhpKHUH2jTHC6R0paH8OXDgQHn00UeN5wXcIlvihBZrlsDzGSCUS1kyYkvILAzTHE43ycnJMX6wmGlhdoD9IGUgriNtCdNlmvPssu9kxvrvDLNCZsVuTGEi8OTXnLAw9cvdF94fGIE7zY9rdpNz98s+YVBuqpxV0pji0E5+qJMxzWH84h1HNc0Brf3EtBtmRYjQHM12zGss5ac4ygwM5gUtWrTIHMuF4z5c1LBriHtliOvatWvvf2L6\/iNcxy7wjTVfG7vA3EIgYE8fWlpzvAjwHmFjaskqleouYbwIpjAfQnJkV9j+YVKDaY2bSCNPoo9kd5GlY6ZsCWFW4z\/cLuMWbjUyKwTsibY5LPmZaagtoX+jxUwLpoUCaX6qD7FaGZZxLBarE37eQz6Ftvvdd98tlph1xas4ynIRO0RCx3GMeU8mbAlhViiEosj3eqea0qpGaT9hTPzZWiJ0CODRA32+YfP\/I71XHi27f\/hD6PoQb4MDybBYtvXs2VPYKm\/QoIFYQvGzePHi8fYto\/lgVEzLUVvgwfzq8evHdWEmdqTUltD\/EUYJeVCrynLCYd9L2RI\/JdygsIxjIBkWjAqnfbiXadGihVhCIH\/IIYckPBjpLGAZlZ1V5TbPVrvAdAKudUdFoGG1LLk6e0\/U+4XhRiAZFqoLyKpwFZMoyNu2bRO8k6IoCqPDP1Z+pjm\/lCpnDrmE+USjeRv2mMMBWOpB+K7C6JQZFXG2nJlV5RYx4XpYZB+JvkdFI\/\/\/ehmWcQwkw0KJjV0+dgVZBsa7S4i2L\/pUbdu2NSoNXbp0kWeffVaWLFki7BZyCs+MGTNk9uzZgr6XHa5i3+6SzjN\/NVroMCAvYgYFwaggGBtLPmQHy+6tJ4TEbZ2EKByOHTtWCIkrhRMBxk\/HMRhjF0iGxexq4sSJxjSH8wXtTmF+u4TMrnbv3i2oQziOIxdeeKGw68jZhsy4srKyzLmGkaY5zLCuPfpTuafO9zFpxPmOQNPbHWLydazylVT5eZNhjjDYSOK5aAVPnjw5ap7IMmGLs5HRt29fefnllwttH8Myjlh08GNPmOh7lOg4JuKIIJWsLpAMi51AdgTt7qAN+TDwJoq9oBcIX331lXCP\/LVq1ZKOHTsKJiMMYDTTHNLr1awqk0c8IMPu6xaTcrvfIBBLznjIniyC2U88+cOYB0+uzz33nNx1111mKR7GPuTX5rCMI66TOccTPcP8+hR5P9FxBBMmAXw\/ksF\/gWRY7ARiE8WhExwgwbIQ9zII49FkHzBggIEI1Qf0q\/DmgGkO+bFD5HgwTtjBwwMfEkwMTWVTKOIPgLOMRAUiMXpRNL9iELR3oEePHgJlol18N3w\/EZ9UWqOBZFgwJmZZAIKHhjZt2gizFKzRu3btKiz7QAUdq7lz5wqa7ZjmcGZh3bp1xZrd8AtQrFgxY5eGOQ5lIK5tHuKAnpOTYzTjNVQc9B2I7x3gu+H7ySQFkmEBAHohMCiuIU60xZMDgnXiXpSdnS3MzNavX29us8uIOxlmYUyT0e+CuE63aY5pgP5RBBSBlCIQSIaFWgPmNO3bt5c777zTaLpff\/31wuwJGRb3vVCAObE0vPfee42yKQepUp5fTJhWpkxzvNqmaaFGQBsfEAQCybDA5oYbbhC2ktnxQ9N98ODB0q5dO2GJCEMijxedeuqpMm3aNMHhH7uK2Lk5TuZMc7zapGmKgCKQGgQCy7BYzo0YMULeffddw3xGjx4t1113nek1QnlzoX8UAUWgSCEQGIaFkJ1dPwi\/PEOGDJGqVasaAfsJJ5wgOCW76aabBB2tIjVC2llFQBHIQyATDCvvYZEXCNJZvjVr1kzwZXX55ZcLdMkllwj6JKVLlxZ2+ljmsVXLbAuheWQ9GlcEFIGigYBvDIsdv379+gm6U5MmTRK3Rvs777wj6F+hrgBD27x5s+BJ8ZtvvjEeHIrG0GgvFQFFIBIB3xgWXhcQnuMfmt09d8OIs0OI0\/8HH3xQxowZI40bNxb0rqLtELrL67UioAgUTgR8Y1jMmBCe4w4Z5mQNnG0Io8IJPkd+YYuHgmjv3r3F7dO9cA5JuHulrVcE0omAbwzLdgqDZORUHTp0kGHDhgkMCgNOBO8vvPCCNGzYUJBxIdciDwzOltVQEVAEihYCvjMsmBOW3xgqly9f3miqs\/TDPhBzHJaCr776qkC4TNYlYdF6QbW3ioAbAd8ZFo7DMMNBXoXKAsbNyLY4MLVnz57Gb9WyZcvkiCOOMCoNLCXdHdBrRUAR8AkBHx7rO8NCwH7ZZZcZVzCNGjWSJk2aSG5urvFj1blzZ+Eevn3QeEe1AX0tixN2hffcc4+sXLnSJsmcOXMEOVj9+vWNuxN8ZOXncTSvsF4oAopAoBHwnWGBDsu+Z555RvCxg3b7hAkTBLs\/dhBhWOSJpFmzZpmTcPAiau\/hxYF68M+EaQ6HqLKM\/Pjjj2N6HLXlNVQEFIFgIxAIhvX0008LRsvDhw8XHO7h2hhbQmZK\/fv3N25fcGsM8zn88MMNogjnUTI97rjjTNz+wVsDdRBnqckMDnfICPYR8BOP9DhKXiVFQBEIPgK+MyxmRcuXLzfH0mN+g+oCnkNZyjHbws8VrmBIY4cQ7XhgPf300+WCCy4QfGARhxDIoyLBkfaY9YwfP17YWYzlcZRyW7duFQ6SSCXN+\/3QCne4YMUGgVjiLlq0qNC6FNb+FY2x5bvh+8kk+c6wYEAI3kuVKmX6jTkOgnU8jHbv3t2Y6jCTgpBhweBMRo8\/aMbj4YGDJnDSx6wNhoesK5rHUUBnKcrBEvFQj9HvSzz5OLAiklo+u0Wgiyd8J20emWVkbJGuajV+peJyZXIYsCphRYKL8Ey8R3w3fD8en2LaknxnWMyQ2B1E7gRj6dWrlyAwh0GhLGoPoCDEXQx5o6EBwzr66KPl2GOPFcdxjP8sdhtRl4CB2XJcW4+jAP7BBx\/IqAa7hcMl8qM3rqscMx\/3Ld1XY5sc\/nauObhiSrfTxRKHXv5wbAMZNGxM6N0so+CLrJEfiky45fXjGbfccot5dfCA68fz430mIhNkv\/hnj7eMzZfoOIIJ3w3fjwEnQ398Z1iO48jNN99sPDPAsLAv5BBVuHeiWu2VK1cWlo2oSIAfp53A4HDeh5dRDKchrllmksdSpUqVjKwMZ38Fofq1qomlS89vINdf0ULOrJEtDauVyaMOZ1Ywjz377JyUPLMg7S1oWZbfLONR8C1oXUEtj9IyTLlOnTpJjlfmxhk5bZkyZRJuZ6LjiEzYvMQZ\/uMbw2IpCHPBnQxMhCO58Hn1\/PPPmyUBft3JkwgeJ554ovGZxXSYWRp1MWPjQ4Bp8evTvHlzozqBR9NE6k4mLzM7fu0I3eVhXsTnf7qHQCngCDB+XuMY8GYXyub5xrCQRSGTYunXsmVLozuFCgOEOgL+sPDOEAt15F0I5mvWrGmyOY5jPJKyfFywYIE5K69KlSpmecgLx9lrpF999dUmzRTy6Q+Hrs7b8KVPT9fHKgLhRMA3hsVSDZkUelQwKATl+GuHXn\/9dYMmxtHmohD+6XBmeWGG1WrYUiPEH\/TWRmFHMWxdRXVk48aNQhi2tmt7\/4cA4xeGcfSNYVmoWA6y\/IOB2TQE56RxT8SmFq4wt3kVgWkx0\/ps9z4Z9NYmaf3kUqk9YKEhGFjh6rH2RhEoOAK+MyxsBFH2ZGmH7hU0btw44YgvlDwL3kWRoJrmwLSGdaghU7ufLrsfayLL7q1nmFjlrJKGgZXtNSfwzAuVFJbdhKkYK63DHwQYvzCMo+8MC\/0otlTZIUTp84orrhC2StEnwclfKoYvLKY5zLZgYjAwmFdu82zxYl5BWjqGZSmRivconjqYLT8fM3MAABAASURBVDM+KCEzS0ZnjxDiXjx1+JEnLOPoG8Niybdjxw4zNpjbYPCMsBwbwIEDB0pWVpa5Z\/OYSJJ\/4jHNQadk06ZNRhbD4KWKmD3S10TrO6qUyC3nVpBXbqwhi247ff91eTnyEDEzL5aOdvblloHxUfCh8MFY+tfOryU\/SrRtkfmT7WNkPfv27Us5\/vE8Iz98YDRbvty3X8b45X7ac4BVBAwJYhzsmDA+pI1buFXmrt0ls1btyBs30uNpUzJ5WJWws\/7jjz8mhWO848h3gkuoJD\/HAhXzjWGhY4UrGewGYRb4eLc94RqlUe6Rx6YnG+ZnmkO9+N5CeRXHgakimC2qG+iFFaTO4t9\/KdfUKikj25aXj3pky9Rrj5FOZ5WRC6uVNC\/mum17zAeBHIwPgg\/GUt2HPpT8qOLdCyVZOr7fUqk\/crtU7b8s6TqSfXaqyuWHT84jS6XN8zuk3Zg1Rs4IxtDAaesNQwL\/siV+klY1Skuf88odME6MlR03xosflM7jVkpB3odYZWE6MLtYebzu4ZeO1Q52uF733Wl8J+hJ8t1kmnxjWOwAcmzXfffdJ9OnTze6UTjtg7ARZKaFEil5UgEKgxGrHrSYUQ6sUKGCpIrYPGAzAT2eVNVJPXVOqix92tQwxPIR2vZAPYGW3HmmuMlq10eGyM6UaogXBpFYTbihumFCr3eqmYetG2vwH9GxpkCdmp4gLetW8XyHGDPGZurqr6XblF2eeRjfZAnHl7zn2OQmW0c85fhO0HSP9T2l655vDMt2CA3zvn37Gkd9mNZA6EphD8UA2HwFCbOzswVzHFsH19Y0x6bRDvIhfEwVIYPjBUpVffHUc\/xRpcVNKKl6UYczyxsBf0HDS2uWkZxy3wlhQesKSvmD8cqSOpVKSk72\/7CNZyy88jA2MMQPNn8tT8zdLl55kk3jfcMOt1ixYgnXy3fALIowv+fznaRC051nJUq+M6xEG5xM\/nr16gnmOKhJQFzXrl07maq0jCJQYARgiGyq2CV8gSssQhUUCYaFGY4fpjlF4T3i1zgM2+FBGwt2hGFayLTQvWPDxM82hmUciwTDchzHeCcNkmmOny+nPjsYCFimxVKY2ZbdZfSbeQUDHe9WBIJhsZN25513CnpYyK+GDh0q7BR6N1lTQSAoxI5UGEw6goJXZDtgWrnNqwizrdwoeneRZdIRD8s4+s6wUBgdNGiQcS\/DQBxzzDGCHgm7dtwjLRpxXw+hiIaOpocJAcu42HGMZF5BWDIGBUvfGRZeG3755RdzWKrd3UCVAeE496IBpYdQREMms+lhkX1kFpWCPS2SedklI4wLheCC1e5dOizj6DvDslux3377bR6SLBFhViiX5iVGXOghFBGAaLRQImCY1+9LxspZJY3iKoL6QtnZODrlO8PCwJkzB\/Hfvm7dOnN6Dj6y8IDIoRLR+pDKQyh4BuY7EydOlGjEGh9y3yduyZ1ur63LHLT1vfLZNEJbJlpIHsh9n7gld7rXtVc+m0boVcadRh4oMg3N6DVr1sgrr7wSFTvKUBbi2hJxSzYtWuiVz6YRRitn08kD2TghcUvEoxHjiAY5JlbuPLYsoTvd65o8kPsecUvudK9rTLUw0zqrxEbjjuiul5caKwd3+bfffluwrkDPMLIOdz57z6YRkoab62jjSB6IfJDV2eLbyST5zrDoLJ5A33jjDXn88ccF53pck8Y9SxwogRY86gl4KbXp7hClU0DnBWPQKJPfIRS2\/KpVq2TZsmUpNZnAiBuf8swYGeDCSHwg9A8qjP2jT4wj\/SuoiRV1FZRaH7tPulbZLiM+2CN9Jq0+4H21mx+O4xyQHs8z6WO87yrfyZtvvmk\/nYyGgWBYnNaMX3BMWPA0irvkMWPGHAAEjAeGhI1hNA147mMOk8ghFPYhmANdf\/31wskjXmRNFtz3bBqhO91ew3xhvLhsJg9k7xESt0Q8Fnnls2mEscpyjzwQ15aIW7Jp0UKvfKRlZ2cL5zxGK2fTyQvZOCFxS8RjkVc+m0YYqyz3yANxbYm4JZvmFTKO1sTKfd+WJXSne12TB3LfI27Jne517c53301tBTOhkYv3yJwtYkx8KINXXc41YEyIu8ld3qbbNELkxl27do3r\/ec7Ofnkk+2nk9HQd4bFeYMvv\/yysCTcvHmzEb4jUMclDHKsRNBgsOwvIeXMIRRHHinMytBuR5APcR2p6V5YTHMQniqVTNg0JRZmyFkzbWIVqz3ca1y9nDmF6bZJm2TnfvEvabQzWdMcysdLMMQia5qDbADf7MyMFi9eLAAB8CVKlBDUFmA88VLQDqGIt91hzodcwy5FwtyPMLYdE58GVcsYJ4+f7d5XoC6EZRx9n2HhcRQ\/Plh\/L1y4UODe+HQnDYF8rFGA0eGpNIyHUMTql95TBOJFYGr304WdRNzdxFsmzPl8Z1hMtQcMGCAdOnSQwYMHy+GHH27wxLUMMy0T0T+BRYBlRHBsCQMLU1obNrR9DXOgSUFOYQrLOPrOsBhJGNNpp50m6FYxNW3UqJFhXsijuB9J+GjnzMGzzjpLOH8QFQiWJeTDJzxCbpaWLVq0EHb\/yM\/uIfkbNmwoOCDDMyP5lRSBsCNgl4a5U\/dL4MPemXza7zvDQk7FDKtVq1Zy3nnnSdu2bQW\/7uwYcoKtV\/s3bNggqD5MmTJFcPRHmZEjRxqZFyY9xBctWiR4LH322WdlyZIlMnPmTOFIMVQeZs+ebfxvedWtaYkhwA8MPxaEiZXU3KlEAEeEyLE++KJUUtUyfmEYR98ZFq5Z2Q2Emfztb3+Tp556Sl544QWjFOc4jif45cqVk7vuuksQ1JOB8lwzu+IaRVTHcQT1CGwN2S1kxpWVlSXIxdiGh4lRVkkRKAwIIMdCAP\/8pjKFoTtR++BiWFHzZOwGpjgo56FisGvXLoH5eD0cxsMS8p133jFG0ywP0aNCRQLDabyV1qpVSzp27ChoYjMjw6hafv9XtWpVwZH+71ET4FeeNH5pUknsgqIhnco6g1QX4KHHQxikdqW6LWEYx85n\/8as5n9ewvzgJ4IB4xfvOPKdFLlDKAAIYrbE8o+jvhDePv3000bjndkSsi3yQGitR2q6s4REo50Td\/D4gLbu2rVr5cYbb5QVK1bIVVddZWZiMDGE+9QTjdJxCAXyM5aqn3zyScKax\/FoJ2ue7RnBNSzjeHyp76Rdxc+lcvG9acUFGXCRO4TCMg7HcQRfWCzxWLahbYu6ArIolm82n1vTHZtDjgLjnuM4UqdOHeEXsFSpUlK3bl2x\/tqpDw8Q\/HLA2MgPcW3zEIcuu+wyqV69uqC8miril2jSpEnC7C1VdQatHrCEKXNEW9Dalqr2hGUc9+7dKyeX2Cn7dn2W8HucyDjynfC98N1kmgKxJMS9DDMj7JNw3IcCKEyJpZQXIMy8mLWwbGQHEBkVSz6WgTCu9evXm2KrV682MqtYmu6Ug7G99tprZjZmzRZSEd5\/\/\/2mHcOGDYtq8pCK5yRRR8ra06lTJ8FSgYNE\/GxHOp8dpnFcvny5QInikcg4Mrnge+G74fsxL3mG\/vjOsGBKmOU89thjZsePXT8IrXeWcl444KP94osvltatWwsHTEyYMEF69eolyLZYOt57773SoEEDI8Bn9paTk2PMc7Dra968uTlSjDqoG8CZzb344ouipBiE+R3A\/habwB49emTkXea74fvhO8oU+c6wsO1jCYjs6oEHHhBLyKWsEmkkGI7jGK8OuFNGfQGhe8WKFU22U089VaZNm2aY30svvSTIxRwntk93QIepKeWIYqAYxPsO8N2Yjy6Df3xnWMipWOJFUxLNIBYZe9SoUaNk5cqV5nkIMM855xxj\/A3zNon6JxQIsDGEhUazZs0EX1RBbXRhapdvDAsBITt7\/fv3F5jVJZdcIp07dxZUEiDukacwgY18Db0w7B\/pF7t8yOrQQWOp+u6775KsFBIEkLminIztKycy6Q9O+gfON4Z16KGHGsFvz549BU13XMrAvIhDCA3Jk34IMvcEVCvY3cRkiKeyaYBLHGaYLF390m2hLUqJI8APLeOGAT\/vKkrQideiJRJBwDeGhd8eZhzswOASGaG74ziCcz4Ih2nkSaQzYcuLRwpUMWy7mXHZaw2DjQAmZdijYjRMSzmTAMVlrpXSh4BvDIvdQWZU7PJhmIyAHYE7L0L6upt4zdFK0E6Wd1YWRT5kGhhiY2Tdvn17QZGV+2jlY5CNygb5LOHzC7UMG0cx1l5r6B8CqMnwQ2pbwG41IgpMujCeZzXAjynjx3tAPmSxGO9zrZQ+BHxjWKz3mUajgIYPrL\/85S9CWhjkVrywuKNF9mSHhheXbV4YFioZaNnzkqNThl4M2+Xshtr8hGj5I8OCeaM7hkkS6Ur+IMASnR9N1GyYMdlWIKvix4ddaXae2dHGUJhDUhg3TMjIz4+uLaNhehDwjWHRHbTQCS0x6Hz4Nh7UkF9SlrHHHXdcXhN5aVnSsSVMItr3yDhw7E\/ci8qXL29sIRG4YxeJIp5XPk3LDALMmrCUuPTSSw94IOcIoPdXvHhx4UcFFRoUnRk3jPXxLsK4R\/4gHVCJRlKCgK8MKyU98KESryPGMNpmicCskSbxa8sywYthoRWOl1THcczGA2YtaMOTn7JK\/iCQlZUlGNHzY2NbwDIeYTqzYdIcxxGW7pj9wLyYcfFjQznuK6UXAV8ZFmcBYk7DC8AvFEqghMQxp4l2nFd6IUm+dnYB+ZVOvgYtGUQEGFN+jILYtqLWJt8YFjuBTLU5msuLuEeesAwIy0S3DA5ZHO49WPaFpQ\/azoMRYMbMUo8NFe6ySYL86qSTTiKqlGEEfGNYGe5n2h8Hc8VUgR0mHkaIU0HsG4krhRMBx3GM7Sk\/oOwWoivH8r9atWrh7FDIW506hhVyIArafJwPYoCNt1SE5+PHjzcG2aQXtG4t7y8CeK7lhweVBtRT8LeGwq+\/rSqaT1eGleS4s0zAxAbhua2C3SOEsPi\/Qo2BuL2nYXgQwDEk\/tdsi9kdxBifcWWDpEmTJvaWhhlGQBlWhgHXxykCikDyCCjDSh67IlxSu64I+IOAMix\/cNenKgKKQBIIKMNKAjQtoggoAv4goAzLH9z1qYpAWBAIVDuVYQVqOLQxioAiEAsBZVix0NF7ioAiECgElGEFaji0MYqAIhALAWVYsdAp+D2tQRFQBFKIgDKsFIJZWKribEe8pnKajyV80Ye9f2iqP\/PMM4LLGExscMTo7hP9Hj58uDsp7xqD5yFDhgj2hHmJepFxBJRhZRzycDwQz5uYoVjCHbRtOR4L8Gdu42EIYVKTJk0y\/q6SaS+HTeBiBhdIyZTXMqlBQBlWanAsErUw++jWrZu0atVKFi5cKKtWrRKOZ8MoGB\/2zEIAYs6cOVK\/fn1hdoZBeJ8+fQTfZsxqCMnD7IYZDcyPg3DJSz39+vUzMyDuUyeuqDEm50Rj3Pcwwxk8eLBj5tEwAAAET0lEQVSQhqM9PH7CiG699Vax3mqZCUE8xxLeM3CQiNM9mxYtZDZJeyCcNdKGb775Rho1aiQc6cXhKV5lNS39CCjDSj\/GoXxCp06djGdNnClyiAaHadARXEHjiQJ\/UEOHDhU8pXImHwyDk482bNggo0ePlgkTJgguWWAQzG4o60WUXb16teC18\/333xf8T02ePNlkxcU0Mz3yMLtZsmSJ4F99y5YtwhmOMDWWefgi27Fjh+CiGqa2bNkyadasmanD\/iH\/ySefbKMmdPeRfsKMuMFskpkl\/czOzhZ8vNMu+kL\/cS9DPqXMI6AMK\/OYh+KJI0eOFOtYkUM0rFeKxo0bCzOVTZs2CR81sxA+9g4dOsiKFSvMrAs\/YHzcjuMIng\/wbBGt0zAcGAMMsGrVqoYBLl261GRndoOPMbwl8HyWoRwEATPiLEdcviCTYjbHYR\/L9jMqDvXgeZH+qmBkpJuKf\/\/j7iN9bdu27e93RDgYBFfWF110kSDP4wZMk2O99Dgv0PCHlGH5g3von8qyiGOvYFJ87JweM2XKFIEpsMyzHeTsRZZxNh4ZstS644478pgjsy2WkJH5bJy6cEVt43v27DHMpWnTpmZGx3KUo+NgaDZPoiHtHzdunHA25vXXXy+O4yRaheZPEwKBYVhp6p9WmyYEEELjBhpGxQc+bdo0efTRRwXHdizZ8MxJOrMwmBuODJkp4TaadGRKNI3ZC7MmZkDManr37i0sDbnnRTBJlpowLo7e6tGjh3BIRO3atY2cDNkas77IsriqjrU0dedn2Ul\/WArSZnuPGR5tP+KII2yShhlGQBlWhgEvLI9jyXf77bcLgvMGDRoIjgu7dOkiLM0QrrO84iARGBezLpZvxDkSixNmkHXBxFgywoQIWW5y1h\/X0XBq0aKFmfEgAG\/Xrp1wriXLRpiIPdCEeGR5noEcDoYTec8dhxEyu\/rss88EoT\/MjxB5GnIy+sLMy11GrzOHgDKszGEdmifhbdOLacCQINsRZEcIy5khWQ+rjuMI5\/pxmCyzK5ZUMAnHccyRZsisZs6cKaNGjZKHHnpImMEwSyIdYnlIGs+nHfZZPJc0GMaDDz4oqBfwXJ7lOI7ZIYShtG7d2jA0W86G7CgyE2NWRh20l\/rsfUKeR1u4hxyN9kMwY5gUfeV8woIsN3mOUvIIKMNKHjstmSwCKS6HqgSMixlb3bp1PWuHSXXs2FGQccFAPTPFSERl49tvvxVmiTGy6a00I1AszfVr9UUcAXb32G1LJwycWDR16lQzY4s1+2HXEZ0qx0lciI7MjuUvs7909kXrjo2AMqzY+OhdRUARCBACyrACNBjaFEWg8CGQ2h4pw0otnlqbIqAIpBEBZVhpBFerVgQUgdQioAwrtXhqbYqAIpBGBJRhpRHcgletNSgCioAbAWVYbjT0WhFQBAKNgDKsQA+PNk4RUATcCCjDcqOh14qAfwjok+NA4P8BAAD\/\/7kXHdIAAAAGSURBVAMAiaIHir7j5ygAAAAASUVORK5CYII=","height":181,"width":300}}
%---
%[output:08ce1ec0]
%   data: {"dataType":"textualVariable","outputData":{"name":"used_device","value":"'infineon_FF1200R17IP5'"}}
%---
%[output:76286c21]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASwAAAC1CAYAAADlY8JKAAAQAElEQVR4AeydC7xNZfrHny0zztAFKRRFSSiVxq2bMl3oIjQ1YSpNNyIpl07TZaJCLl1UIqkZXah0UaZ0MZXSxUhCIZeSO4NkJEnmf76v3v1\/z9p77b3POXuvvdc5j4\/nrLXe6\/P+1np\/+3nf9b7PKvc\/\/acIKAKKQEgQKCf6TxFQBBSBkCCghBWSG6VqKgKKgIgSVhl6CrSpikDYEVDCCvsdVP0VgTKEgBJWGbrZ2lRFIOwIKGGl+Q5OmzZNDjvsMBkzZkyaS95T3Pz58+XYY4+Vfv367QnI4N9t27ZJly5d5NRTT5UNGzZksCb\/osGR9tJu\/1S5F4O+6I1wnqqGacM81QqTpEN32sBznSRpINFKWIHAnLlKIBIIBWLhYc9cTWJIK6i6aAedZfTo0XLvvfdK48aNCfKVouJAB8zUDwv3YciQIVK5cmWZPHmyVK9e3VfvXI8A9zPPPFP69u0r3I9s6xsqwsKq4CGzwkMHgDwgdFiEB8TG8+tMvBU3Px2Ph5w4yiEPHYMyOOdIucTHE8omHcIvkN\/NpE7qeuaZZ4zlRV2UR3rykR+hPDfc1o8OnFOG1Zd0CNcXXXSRrFy5Uj755BO55pprhPTEWaE+t3ziveWhI2mskMfmt8f\/\/Oc\/Eq8udEA3m5eybB6OXBNHW8GXc9tW4qmLMCtcE46MHz\/edPpjjjmGS9M2dLdpqZf6kaLoRh1gRaHDhg2LWsOE27I5ck2aeEKd1E86hHPCwJeyuR\/clw4dOsj69etjiiAteciLgJM30dtvv22eGeJdzKjDxYFzwsiPzqTneaN8W677vHXr1i3Gcnbjye\/W17VrV4oW7oc5yeKf0BAWAL700kuFoOLB4AbZQB6SPn362EvhYbTx3Dg3Pw\/TDTfcYDqBzTBq1CjT8bmmrKeffprTGKFMyrYR\/\/3vf+W6664zFogNc4\/Udfvtt0eDeFhJTz4bSHmUa6\/TdaSz165dW5YuXWqK3L59u6xevVqaNWsmBx54oOmsLi4k8uJKWDz54YcfBAxpn42nLO4V1xy55py2gi\/nVmgvddlrjvaXHIxmzZolBx98sFSsWJEoGTBgQPT+EEC91E+buHYlmW5uWs4T6UK8K5AD9VK\/Decc0ty4caMN8j3Gyw9O4GUzUZ77zGBpQirEe3HgWSWMOCvkpQyuwdJ93iBC8hCHUO4ll1wi3COuEfd5xELEWuR+UBbx2ZJQEBYgTZw4UfbZZx955ZVX5Ouvv5abbrrJYPbGG2+YI3\/84skP2C1btpR58+YZ4ZybhpAXIYz4sWPHcinvv\/9+IUIjkIftiSee4FTGFqRDlwsuuMBYOeQ1EXH+oC9pzzjjDOHh5GEiH2GUQxa3LVwnEwhn0qRJAiGhO+XsvffehbKRBnKi\/eCAjtRdr149Q7Cp4EqBBxxwgHjrikQihvyom3LBEl3ADauCI3nRi3jScW2F9nrvGZ2GX3Lyb9myRVq1aiW0CdzXrFlj2ko9trwvv\/zSdLSi6IYe6IQe3Jfu3btLIl1I5wr1I5SDHgjn4MoPA2VzDRaQA8OqRPl5psEBvCBam5ZybNngAiZ+ONj7a\/NSP3lHjBhhnnd0s2GUa9Nx\/PDDDw2GhLvPI8859fGDwQ8H9wMdyJMtCQVhARJgHXXUUVK3bl2DFZ2dB4IbZX\/V3PiTTjrJEBwP+fLly4X8PGRYHAjnpiDnj+0c\/KLwADlR0VN+zbFQqJtyiOCh4EZDRlx7hbLQx4bzUHPetm1bDkI+8lOOCUjzH+qh\/eBI3VYfrgl3cYuHq586kOH06dNlwoQJgjVKh6BjkP7HH380ZGZxgnSuuOIKoozQEbg3dMT27duboQ+\/6iay4A+6EVdwav6T\/6CDDjI\/DNQD9pQ3d+7cuPNbiXQzBTp\/kuniJDWnYMiJfV7QDV0Ig\/g4JhJvfgiNdoBjpUqVTFYXN9ptAgv+UBfpwB1CAodEz3JBlqh1bfUFP4Q4xOqDtctwkCPhuSihIKx0AUdnhBhcgSzSVX6ulgMBo9tbb71lrEaXoAgvrtDRmT\/hIcdio+PQ0YpSHunJ594TiNt2IrcswumktrPRsZinwXJ003FeHN38dKG8XBKmN8AcjFw8SqIjP2JYeu59gBghSPsjzbDQPkslqaskeUNBWIAEWJj\/33zzjWmvHVYx3KlWrZoJ48FHuGBYwS80v0516tQxk7dYY\/EebtKnKl7z2O0YzIOkUg6dm3Q8cByZu+AB5EHkGsGK40GhvbSbsOIKVikkNXv2bGP12F\/aVHH1qxesEYZVED+dx1pYNg\/XhIMTQwwbTkfg3th4G26PFiN7zX2DnCApO3ShXr\/86IWQxk+3PWWLGXIm0sWms0erG0M42oXYtmHN2nR+R5vf3n+eG+4\/5O8OCePlZ76JYSY\/vt0LhrJFeT6svuCC2PLRh77C0NCGuUesXSxx+hqWqxsX9HkoCAuQOnfubMbZ3uGD9wHhgebmQ2iASTz5AZuHm19n4hE6AB2BdKkKHY0Ozw1GF2uSUy6SSjkMD\/lFYwiEHhy55m2MJRGrK3VQV7JyeQBpO53Hm9bqTBoePOonDbikiivprVAOde27775m2I3+tIMwm4YfEcrmmnCLE9dWuDecE09+K3RgcAAT4hF05R5SN2WRlnpJQ1rSIMRTXiLdSGeFMvjBSKSLTWuP3GeEutAF4RwLjXObzu9IGtLyjNIO9CUtw0o7JOQ6ntBWfrxt3lSeD8iN+tCRum19tnyeB3AEC\/SxAi6k4cefOiF\/rrMpoSAsAOLXBOA5t8IvLb+e9pqbctddd9lLMzFv4xlOuPm5QQ8\/\/LB5UxbNkOKJVxfqfeCBB8wvdSpFMGfBK343LdeE0zF5w2PjevbsKXQOe+09WovPG+69tg8llhYWl433toVwL66EId66GjVqJKzRIQ48GUKgKxYhv\/y8eeKaeDBy7w1h3Bvq4twKnYJw2zGtVUA8b8JseVwjFrei6kb56EwZCHX66UK8K\/wAcL9pkw3nnIl\/7p8N8zuShmfPrd+22y+PDSev\/SEgDJ15rvmBw5IlzCvkSfRM8dwxB+nqQ5k8G9aiIz\/leMsO+jojhIVFcNlllwm\/lBLn3+TJk80kq2XyVC0dSMcdY\/OQeYunA9k0AO7Gu\/mZ5ORGEU855LHpCSeeDsjDSRqvuGUxAWpvprcs0lEWZbpl2HTUi3Bt4zknDOE1P3rYOiiH8iiX9OhHPGk5ck24V2y+eGkoi\/xWqJ\/8lEV6W7e9Jh3hXNu86ASZEM45pMgvOb\/qdCrKmDNnDsUKQxBzUvCHuijPir0H4EnHtEPjgqTmB4HybVqO5PfGkSaRbmCBoCdl2Dopi2srNpzyvYJ+tMmm5Zww0lE3OrhhhLvi1k8Ztq54eS3G6EcZpCUPQpgbzzXhpCEtQj\/kXnB\/ILWzzjpL+FHhzR9ETxqvPpTphlMu19mWEhJWrPr\/+9\/\/hFflfuNhcqxatUpYkwOwSKIbS3qV8CFAx2PojOZ0Fn6cGMbwK451Q3gywSpkCEu+ZGk13h8BhoFYgPx4cG6HkczbcZ\/8c+ZeTAxhsaK5U6dOZv0LD1wy8TYJ5p45c6accsop3qjo9dq1a4U5jmhACU8APdkvWgmr0OzFQIBfeX6QXMGq4dc8leJIR3rKSSW9pomPAJYfRoF7Hzi3VlT8XLkZGkNYWEg\/\/fST9OjRQ\/Lz8xMKadxmMeHL2JxfVD9C4i0I628YEx9xxBFmfmbKlClCvW5ZnGOJ8aug8olZ4a04KA659AzQP+mnQUoMYUUiEbOa+PTTT5dzzz03oZDGVZahIKZn8+bN3eBC5zt37hTehNx6662yaNEiGTp0qAwfPlw+++yzQukAo3\/\/\/mbPE697VbpkGwutv4veA7cf0j\/pp4U6boYvYgirQoUK0rp1aylfvnzSqtmuYRMtXrxYPvjgA7MBNxKJ2OCYY5UqVQQrjCHjXnvtZYaeTZs2FdZIuYkBgqElZMZwLyzSu3dv0wzVe4JZAR\/EfVPMg8Pa3k8wp3\/ST80DH9CfGMJiOPjYY48JVpJ90\/fzzz8nVWfFihUyY8YMadGihXkDyEQpQ0N3MSSFLFy40OzBox6urTAZa8\/dI28yeLsRFmFCk\/VUv\/\/9781wV\/VumXEcFPPMY+x9junnbj8N6jyGsLCamFN69dVXzdAQ7wd0vjvvvFNYWxNvrgllee3JRJ4V1nHwOts7sccEOVsAGIv\/8ssvZqsIaz1YFEg5YZcaNWoIRM8xTG1B3zDqDcZh1T2X9QbXXJQYwkJJhmoNGzaUQYMGCds5Hn\/8ceHt4dlnn2026r788suSitVFWQiryVlrxXoQ5riYcMfBGZPuzGXdcsstUr9+fZKqKAKKgCLgi0BcwnJT\/+Y3vzG+kx588EGZOnWqsESfVb7fffedmyzmHMsKq4sI+1rVXmNesqsda4xhJHNmpFNRBBQBRSARAkkJC8uKoR2T5FhYrGCGsPbff\/9E5WqcIqAIKAJpRyAuYWE94WKVJfysNsayYtj26aefCpZTkyZNhGFj2rUpJQVqMxQBRSAzCMQQFvNNHTp0kIceekg4vvvuu8Kc1TnnnGP2cmVGDS1VEVAEFIHkCMQQFssLWNbAXkBWsrOsIHkxmkIRUAQUgcwjEENYeFrAjcemTZuS1o41ljSRJlAESjMC2rZAEYghLGpnh\/w999wjzFslEtKQXkURUAQUgSAQiCEshoTsEWKfIBPuiYQ0QSipdSgCioAiAAIxhPW73\/3O7CVMtvHZxlOIiiKgCCgCQSAQQ1hBVBqtQ08UAUVAESgCAkpYRQBLkyoCikB2EVDCyi7+WrsioAgUAQElrCKApUlLgoDmVQRKjkBCwmJNFu5lGjRoIHhbeOedd+Tmm282nhtKXrWWoAgoAopA0RDwJSz8Xt1\/\/\/3GjcxTTz0lNWvWNJ9n4rNA9913n+zatatoNWlqRUARUARKiIAvYW3cuFEWLFggrMk69NBDpVy5cpKXlyeXX365ceTHBukS1q3ZFQFFoHQikLFW+RKWX434esdHll+8hisCikDpRmDF5h3Sfdr\/ZOtZQ2X699UDbawvYfGZrkaNGsndd98tONrDnfH69etlzJgxxnUyH5MIVFOtTBFQBHIGAUhrd8VqgevjS1iRSESYcMdbw6WXXmq+aoO7GYaCDBOxtALXVitUBBSBrCMAWWVLCV\/CQiE+GHHHHXeYuSw+6cOcFn6y1LoCnYyJFqwIhAaBQ6r+LlBdfQlr69atMnToUOOxAdLijeHAgQPNNR4c8Ei6evXqQJXVyhQBRaBsI+BLWMDCm8LXXntNIC97zSfACOczXR07diRYRRFQBBSBQBDwJSzeBO7cudN89JQvNQ8ePNicP\/roo4JHh2HDhhk3yiXRkoWpfAuPz3+VpBzNqwiEFYEw6r3yux1ZU9uXsCATvphTt27dQsrVq1fPrHQn3htXKGGSCxamTpw4UXDFnCSpRisCikCOIlC7bnWziAAAEABJREFUSl6gmvkSFo78mHRn6Ae5oBVHJt85x8paunQpp8WSL7\/8UiiLz4cVqwDNpAgoAllBYMXmH7NSL5X6EhaEdP3118vIkSOlRYsW0qpVKznhhBPktttuE5Y5LF++3KyCp5CiyrZt24Rh5jXXXCOs90qUH1Kjrh07dkhY5KeffpIff\/wxNPpaXMOqN\/qHVfew6U1f\/GjesmiXLbd9Y\/Q8iBNfwqLyo48+WvhCM1\/Ryc\/Pl+HDhwtfauaDqg0aNJC33nqLZEUWhoJ8sr558+ZJ80KY48ePl7Vr18ZILoatW7fODJlZZJuL+vnpFFa9aU9YdQ+j3vTFj+fvGVlBVkjSTpzGBAkJiyEgc1VsfG7WrJkceeSRsnLlSunSpYvwoYpKlSoVWZXFixfLBx98IFhXkUgkaX5IsmvXrmbzNXrkulSvXl0OOOAAqVGjRmh0BtOw6h1m3cOIeet2F8uuakeafltue\/Iva5mEafyTkLDGjRsnEFXLli3FygUXXCB16tQp9kdVV6xYYaw0hpmHHXaYvPTSS4a8+vXrF7dZrLSnvry8PLP5OtePDKUrVKgQCl1dLMOqN20Iq+5h1Hv1L5Wj\/bT8xkXR86BOfAnr+++\/F776PGHCBHnuuefMvNWSJUvkuuuuk6OOOsosbSiOkmeccYbZm8j+RAQCHDt2rIwYMaI4xWmesoKAtjPrCLAlp+fEhUaPcgVzV3mLXjXnQf7xJSwmA1GEpQuVK1eWTZs2mYnk888\/X6ZPny4QGvEqioAiUPoRcMmK1tbJ+4FD4OJLWJirbHBm6QKExVwWgoaQlSU0rksiWFZYXSUpQ\/MqAopA5hCwZPXhsi2mkkOq5snDnRqY86D\/+BIW67A6d+5sli6sWbNGDj\/8cGnfvr0RJpSrVq0atK5anyKgCASIAEQ1cdY6Of+ROeKS1as9mmRYC\/\/ifQmLLCxfYPjHnBUuZe6991558MEH5Z577hGsL9KoKAKKQOlCwCUq5qy4poVYVpAVR66zIb6ExYZnJsO3b99uyIkhIqvSjz32WHn++eeFxZ\/ZUFjrVAQUgcwgADENffMbOe7uj8VLVPlt6sjnt50g2SQrWh1DWKzQ5u3g66+\/Li+88IJwxGODlZdfftmEQ2QUoKIIKALhRACCYsgHOVXt864hqqFvLo82BnLq3KyGvFowBMxvU3hPcTRRwCcxhIXlhEcGts58++23ZgsNfrGs4AfrkksuMYsjA9ZVq0sJAU2kCMRHwCWodqPmGIKCrCAtNwdElV9gUUFUozo3zLpV5eoWQ1is0n722WeNZdW3b19zfP\/998UK23EuvvhiiUSSr1J3K9JzRUARCA4BS04M8SAna0FZgrKT6FYjSMpaUwz98gssKsJsfK4cYwiLj03gVoYNpR06dDAbeDds2CCuEE+6XGmE6qEIlFUEIKYZS7cIVhJk5CUnhnhecgIryAiCwoLCkoKkOD+5XmWic1ZiCIsFohdeeGF0K47dkuMeiSddzrZKFVMEShECkBICKSEOMZlhHcsOCCPOj5z8CArSynWScm9lDGEdeOCBZiU722b8hKUOpHML0nNFQBEoPgIQkrWUGMZBQFhLvLGzQhjiR0zUDjGddHhlgYiwmKz1ZC0owsNEULTJlRjCciPx1jBnzhzhIxTsIWT9FSTmptFzRUARSIwAZIRYQoJwIJ6LHl8o7cavkoNu+biQpcQwjjRYS+SLV3oiYprSs4lAVmEnp3jtTkhY+MLq2bOn7Nq1S5o0aWI2LTOvNXXq1HhlaZgiUCYRgFS8ZAQheS0kO3QjzhLSmq27fDGDlBCIB4GEsJg239farInyEhNpfQsrJRG+hMW+QRzt8XmvQYMGyZVXXmk+QoEzP8KJDzkGqr4ikBABiAhJREb27ZuXjCwhkd+vkoP2LS+IO4TzkpIdyhEOaYV5OOeHQ1HCfQmLBaRYVnU9H6HgmnDii1KRplUEcgUBSARJhYiYPyoOGdFWLB5LRpANpINYK+nTm5vJlK61ZNKVDaNDONKVdVICOz\/xJayKFSsKn\/pauHBhobzffPNNdKtOoQi9UASyiABDKy8JuZPXdnhmLaKSEBHNTERGWEXeYRtEBRkhSkggWDzxJay9995brrjiCmHxKC6R+dozbo2vvvpqwYsD3hyKV6XmUgRSQwACQlxLKB4JMWnN5HXLEXOMZwHmiBB38jrRBLarDUSEWMsov00dY\/1gFSGpkBH53TL1PH0I+BIWVZx66qlmpXvr1q25lIYNG8orr7wieHEwAfpHESgCApAPAgEhzPMgkAuCFYRg\/fhZQukgIaycREQEKdkJ7fw2dc0SAawiRMmoCDc8A0l9Ceu7776TMWPGyA8\/\/CBXXXWVDB48WG688UZhDisDemiRIUQA8rHiElA8K8hLQH7zQqlaQsAFeSAtDt1b2jXc2xALQy8EawixQzOXhIhXIgLB8IkvYe21115mGQPWFB5BeTsIiYWviapxqgi480AuAWH9IFg\/iNcC4toloOJYQegI+SAMxxDXEoJkICAEEkIgIeTlbo1lwBnV5N6Oe6wh8mENIZSrUnoQ8CWsfffdV4YNGybz588XFoxyZIh40UUXyTvvvCO6lzC3HwJr+XB0ycdaP14C8s4DuQTEsA3B+kEoM9XWQ0CIJaDOzWpIfoJ5IQiI4RgCSeX\/OiQjHwSEpFq3pit9CPgSlm0qbwr51NfIkSNl0qRJ5s3hwIEDzUcpbBo9Zh4BSAJJhXzc4Zef9eNHQJCLX2uIsxKPgCAYLCBIBwsI4RyZ8uvqa9JYEoJ8EMr0q1PDFQEXgYSEhRXFsoZbb71VmjdvLvjBOvroowWfWLihcQvS89QRgHiQVMnHJSDX8rFDLz\/ySaYRRIFAPnYeiHNIBYF8EAjHJR+u4xGQtYIoM1ndGq8IFAcBX8LChUzbtm2lY8eOwhdycJf8ySefCMsbatWqFdcf1hdffCHk4QOpJ598svmuYTylJk+eLKSxwlAT9zXx0oYhDPJBIKBJc\/4jUxZuk3v\/tdK4mWXOB8HSgXgQzpFMkA9kAelAHkj+r8OvRAQE+UTngTrGzgNRZhjug+pY+hHwJSx8uPNmkLkrPsXFXkIm4v0gYasO6Xv06CGLFy+W\/Px8wUvp2rVrY7KsWrVKRo0aZSb12Uydy94fLBFhxdj5HwgIgXwQyAeBgHpPWioDpm0sIKxVxkcRcz4I5cQA4RMAQVgpKvkks350COYDugaHAgFfwmLhKHNXzGGl0pKVK1fK7t27BcuKL+q0aNHCzHdt3LgxJjskVq1atZjwbAZAKJASYiekLRlBRITZIRgEhKSqr5JPqkhpOkUgMQK+hJU4W2xso0aNBNfKfK8QtzQzZ840W3j4hqGbmnVdfOfw9ttvlyOOOMI4CpwyZYqQx01nzyln+fLlxvMpXlDTJV9v2GYsIIgIawkLiXME0kpESGxYRX5\/cJ50bFzZSO9Ta8iIDnXkqT\/XlYfOrSLvXddA1gw+wcgn\/ZoIwp4xXr0jvU+tafKRv2mtPEEOrChpb2dR8GLozx7RouTJlbRh1T27eu8o8vNGX2RqaPXq1baLBnpMG2FZrRlCHnfccWZLD3NTlStXtlHmuHPnTqlUqZIwkb9o0SIzbBw+fLh89tlnJt77h7eT48ePF6yydMjsr1ZKt6fmS9N7Zpk5Jj9ysoTEgkTW+Iy9oIYgs3vVMRtW2bTK9W2nVRbksmPypHVtkcMr7ZBDfvtf+e3O79OmczranayMdevWCfOW69evD5XetCusuodRb\/oiW\/X4Tqm3rwZx7UtY\/NLCpBxdRbiePXu2\/Pzzz25w9Lxx48Yyd+5cwZeWlWhkwUmVKlXMl3j4xiFzYq1atZKmTZvKrFmzCmJj\/0NmXbt2lZo1a5ZIfq5QRa55aZ1xmMakuFsTQzYmqO3ENJaR2Ulf8Cr+0UsbyzV\/OELOa1rXSDI9qlevbr4ohGWZLG0uxYdVbzAMq+5h1Ju+OGHCBOndu7fbhQI7jyEsljLwS4tXBr70zJE3eFa++uorYXLdu+p9xowZ0q9fP\/NGEe15A8gSCK\/pyDIJ3jhiCpPOit9m6oMPPljq1KkjeXl5xZIN20VGTl8rbIydvXqHrU4gKd6g2df2kBWkdXK9ysWqx+rHy4oKFSqUqAxbVpDHsOoNRmHVPYx60xdbtmwpzFFHO1OAJzGExccl+MjEeeedJ1hSHFHQCivdsaKwlFw9WZeFZTVv3jyzCp4lDgsWLBAm7t10TOazgRrrDXLk82EMI73p3DzFPWcinQlzJsttGRCVJan8NnUFgrJxRTtqakVAEQgagRjC4uMS06dPFya7ISuOLD2wsmTJEhkwYIB5A+gqW79+fbn++uulV69eZjKdDdPXXnutHH\/88eYTYcxnTZs2TWrXri1MuA8ZMsSkYy6LtV3kd8sr6TlLEJhIh7QoyyUqJSkQUVEEwodADGFh9WzevFmYLIdYeHtnh4P2yJCRdG5zI5GItGvXTrCcIDeOXEciEbEkyCZq8mCtMb9FOoaS1n0NcekQyMq1qvLb1DE+sJWo0oGulqEIZA+BGMJiSMgXcpirYmgIuXiFcNJlT23\/mnnr5yWr\/IKhn38OjVEEkiKgCXIEgRjCwhriLQAT5gwNsYK8QjjpcqQNUTUY\/rGOygbkF1hW+UpWFg49KgKhRyCGsMLaIiWrsN451VsRSB0BX8JivoqJcpYneIVw4lOvJvMpJ85aK3Z1OssT1LLKPOZagyIQNAK+hMWQj6GfHQ4uW7ZMPvroI+nWrZtxM8MyhhSVzXgyrCs7b8XbwHwdBmYcc61AEcgGAr6E5VUmEokIq7e7d+8uTMh7F4560wd5XXjeqq5ZFBpk\/VqXIqAIBINAyoRl1WEv4NatW83iUBuWzeOMpVuiQ0GsK4aD2dRH61YEFIHMIeBLWKy16tSpk7DXz5WzzjrLrKvyrnTPnIqJS2bNlU3xcKeG9lSPWUJAq1UEMomAL2GxhYb5KhzxufLkk0\/KwIEDjeuYTCqWStnMXdmJdhzd6cLQVFDTNIpAeBHwJSw2Zp522mly7LHHGn\/u5557rrCHsFatWoKXhVxosmtd6UR7LtwR1UERyCwCvoRFtePGjZMzzzxT8FvF9XvvvScsaZg6dSqXWRWsK1a1owRzV2pdgYSKIhAgAlmoypew2E\/IZmVWveO7Ct0uu+wyGTNmjEycOFHw4U5YtsQOBalfJ9pBQUURKP0I+BLWrl27zNAPf1QuDPXq1RPicOTnhgd9PuHfez5ugXXVuVnNoKvX+hQBRSALCPgS1n777SdMvL\/44ovRJQx4bsALAx+mqFixYhbU3VMlw0FrYdWukqfrrvbAon8VgVKPgC9h4TUT\/1YvvPCCHHfccWZ5A76tBg0aJH369DFkli10LFlRf5fmIbWuUF5FEVAEioSAL2FRCh4b8Fv19NNPm+8Mjh49WvAQyptD4rMlhYeDNbKlhtarCCgCASPgS4o3c1gAABAASURBVFh8iouFoywghaBY1oBfLCyvgHUsVJ13OFgoUi8UAUWgVCPgS1hsfm7YsKF8+umnvt8MzAYyEJatV5cyWCT0mNsIqHbpQsCXsNjczIck+vbta76Q4W7PsZZXupQoSjkfLvsumvykw6tEz\/VEEVAESj8CvoTFG8IePXrIAw88IHfccYeZw7JbdNiyQ3w24GGzM\/WynEEtLJBQUQTKDgK+hMXWHD4OwdyVVwgnPmiYGA7aN4QsZwi6fq1PEVAEsotADGExyc5yBj54ytDPHQrac8JJV1zVGWq2bdtW8GR68skny7vvvpugqP+PgrDsle4dtEjoUREoOwjEEBZDvS5duhhnfQz97DDQPRJOuuLAxJaewYMHC8PNxYsXm6Hm0KFDZe3aPSvXE5Xpzl8lSqdxioAiUDoRiCEshnosX6hSpYrE89Zw4oknSkmGhCtXrpTdu3cLllX58uXNhD4r5zdu3JgUYZ2\/SgqRJlAESjUCMYTltjYT3hoaNWokzz77rFStWlXY6sOXpSEu3C+7ddtz4pcvXy5fb9gW9Sxac5\/ysmPHjpyVn376Sdhrmcs6xtMtB\/Qu9j0Nq+5h05u+yPa81atX2y4a6NGXsDLtrWH+\/PnClh+WTeCypnLlynEbPnLkSBk\/frx8tnhlNP7Y6uXNEJJhZK7JunXrhPm99evX56yO8TALq960Jay6h1Fv+iJTRv3794\/2xyBPfAkLjww46suUtwacAc6dO1fY+mMlXsOHDx8uXbt2la++Lx+NbntcbalZs2ZOSvXq1YUvCmEx5qqO8fQKq960Jay6h1Fv+iIup3r37h3tj0Ge+BJWprw1zJgxQ\/r16yeYwjSUN4XsWfQzMSHMOnXqyCfLt5HcyGkNqkleXl5OCnOAbF\/KVf389Aqr3rQnrLqHUW\/6InPcLVq0MH2xJH+Kk9eXsOh0LG9It7cGrA8sq3nz5hm3NSxxWLBggTRr1iwl\/VkwmlJCTaQIKAKlDgFfwqKlWD4M19LpraF+\/foCEfbq1UuOOOIIueqqq+Taa68VXNdQZzxh\/ZUuGI2HjIYpAmULgYSEBRQsOUint4ZIJCLt2rUT3jR8\/fXX5sh1JBKhurgCYdkI3Y5jkdCjIlD2EEhKWLkAibtgVDc877kj+lcRKIsIhIKwXAtL57DK4mOqbVYE9iAQKsKCrJA9qutfRUARKGsIJCSsrVu3ynPPPSe33HJLIWHvH3FBgaUT7kEhrfXkJAKqVBQBX8JindRNN90kY8eONXv\/ojkCPln53Y5ojWpdRaHQE0WgTCLgS1jff\/+94HX073\/\/u9xzzz2ChwUreG7Yd999AwFM568CgVkrUQRCgYAvYbEKd\/\/99896I6yHBhTRN4SgoKIIlF0EfAmLD6XiRoa9fEuWLJENGzZEhc29v\/zyS+CoFWtIGLiWWqEioAhkCgFfwtq0aZM8\/PDD8tprr0mbNm2E\/UNWLrzwQiE+U0q55doJd8gKceP0XBFQBMoWAr6ExWe+pk+fLqxG9wrhxAcB1e6K2R+WBtFOrUMRUASSI+BLWGTFwR5+q5h0ZykDQ0N8vRNOfKZld8VqglBP7Sp5HFQUgQQIaFRpRyAhYbHx+corr5Rly5YJVhVO\/di4jD+coIHRPYRBI671KQK5h4AvYfGxiIkTJ8pDDz0kgwYNEjyC1q1b1yxxYF6LZQ+Zbo47HDyk6u8yXZ2WrwgoAjmOgC9h4ZMcr6OQlNuG2rVrm0sWlpqTDP7ZVe3IaOk6JIxCoSeKQJlFwCGswhjss88+wqe8cAPjLmFYunSpsOQBKZwjs1eHVM3LbAVauiKgCOQ8Ar6ExcLRnj17mhXu7du3FybfL774YuN879JLLzVklunW7arWIFqFElYUCj1RBMosAr6EBSI47nvrrbfk5ptvlo4dOxrvoK+\/\/rrwlRvigxIlq6CQ1noUgdxGICFhoTp7Bs8991zBpXGnTp0kqPVX1G0n3XX+CjTSKlqYIhBKBBIS1po1awSSatSokfCVjAYNGhjiYmtOplvL+isk0\/Vo+YqAIhAeBHwJi7eAd955pxx00EHG7zprsT744APBx\/vdd98tvEEMqpm6BisopLUeRSC3EfAlLNZZbdmyxXxDkE9zRSIRqVGjhtxwww3CF2tZRJrJprnW1SG6BiuTUGvZpRyB0tQ8X8LiLSGLRbG0vA3G7Qzx3vBVq1aZIeThhx8uxxxzjPB5sHjbeCZPniyHHXZYVJjExxuEW95Kx3GfzmG5yOi5IlB2EfAlLNZZde7cWW6\/\/XbhzSDzWWzPwQtpq1athIWlLslwzRAS8lm8eLE8++yzMn78eJk5c2YMuhDbqFGjohurp0+fHjOZv2LzjzH5NEARUATKNgK+hLV582YZPXq0rFixQtj83KngDSHkBXE98sgjgosZxMLHVh6GkbxR3GuvvYSJepZFfP755zZJ9Lh27VqpVq1a9DreietpVOew4iGkYYpA2UPAl7CYt8JKev\/99yWRWMiqVKliCO7ggw82QbhXZqKeeS8T8OufH374QSA9yI8vP+Nja8qUKeIdOlrCKrd9oyxfvlx27NiRFSlOvQyjsTiLkzebecKqN5iFVfew6U1fZPfL6tWrf+3RwR58CQs1fv75Z3nyySfljDPOMMsa2rVrZxz6uVt1SIfw9rBq1aqCdUVjevfuLWzvOf3004mOys6dO6VSpUpy6623yqJFiwS3NXg1\/eyzz6JpvCcMLbHKwiC8kGDZx\/r16yUM+lodw6o3+odV9zDqTV\/s0qWL9O\/f39tNA7lOSFj\/+Mc\/5KWXXpI+ffrIuHHjzDBwxIgR8sQTT8RVDoJ77LHH5Pzzz5ejjjpKHn30UUNabmIsMTyZnnLKKYbcmA9r2rSpzJo1y00m1tPoCY3rSdeuXaVmzZqhEBrBnB\/HsOiMnugbRr3DrHsYMacv4l4KgwT9gxZfwmIO65133jEW0DnnnCONGzc2xDFmzBghnHhXWYZ0fGGHh\/6VV14RvqwT700iDgD5dBimsJsfa8xer9i8w55Kue2bhF8i5sLCIF9++aX5luObb74pYdDX6hhWvdE\/bbp\/\/nmg9yyMetMX6Zy1atXiELiU86uRhaGRSESwiNw0XEcikZiFo7z5e\/vtt+W+++6TRI3BAwSExjiYoSXzY2ysbtasWbSaQ6rmyej2B0jr33wpcybdK5igYZH+v5rKI0eOVL0Lhg5B3DfFvEvgzxqYs\/slUV+Pdug0nvgS1n777WfIaurUqQKxUCdWFEQD6bgWEXEwL78YzFm5a6ywyFj+wHKHadOmCf60mHAfMmSIMOnOXNYtt9wi9evXp5ioXHzq0TKyVwd5\/sE7BBNUZYLiMEExyKV+wNxzzhBWhQoVzKp2vI4yx8RcE4yK91HczniHe1hICxYsiK6tsh+u6N69u1ljxVorJu9hJN4M4n6ZNDNmzBA+J0a4VwCDtCotC321SPFQPHLhGaB\/evtspq99LSwqxgJiyQGT78xJPfjgg2aJA+uriC8keqEIKAKKQIYRSEhY1M1yBQiKBaGwOpYX4SqKgCKgCASNQFLCClohrU8RUAQUAT8ElLD8kNHwBAholCKQHQSUsLKDu9aqCCgCxUAg5wiLpRO4pcE9DR5O8QCxbdu2YjQts1lS1ZMlIWxvoj243enUqZPZS5lZ7fxLT1VvtwSWtrDCOZv3oSh6f\/TRR3LyyScb90Vt27aVL774wm1O4Oep6s7+U7aq8dzzwuuSSy7J6rOSCCgcHVx22WXm4zSJ0qU7LucIiz2Fzz33nLC49N\/\/\/rfxFsH2oHQ3vKTlpaonnef555837aHjsPwDEvau9C+pPqnmT1VvWx77QlkMzLYrG5aNY6p6r1y50uzOuP\/++wU3R+edd56wXYyF0NnQmzpT1Z11inPmzJGPP\/5Y5s2bZxxm8iFjysiiFKqaH2DWYv7tb38T1l0WigzgIucI69133zVf5albt67wAQwskn\/961+Cl4cA8Ei5ilT15KaedtppQntYu3bWWWcJJMAvVMqVpTFhqnpTJZ2cbVQnnnhizJ5Q4oOUVPVm50STJk2EtYPly5eXbt26yYABA8y+1SD1detKVXebB9ztefXq1e1pThzph6ypxMkBz3PQSuUUYXGjcF9Rr169KA7cMPYtAlQ0MMsnRdGTrSksnrUqL1myROhI2VgeUhS90fe9994ThoFsZuc6W1IUvbFicTdz9tlnC0NwvqHJsxOJRLKiflF0Z2E1bp1YoI3zABZi8y3QrCjuUylGBGsyb775ZrMTxidZxoJzirB40CAntv5krMVpKLgoenKDaQ+mNEPdwYMHyzXXXCNsfUqDKkUqoih647OMBcNYKL\/97W+LVE+6ExdFb4aueP5gSIj7opNOOkkYgjM\/lG69UimvKLoz9cE2Nj72wpCQeU+cZ0J6qdRVFtLkFGGxSBXXzDx0LvhYJJFIdn4hXT3seRH0NFm++eYb45oHbxaPP\/648OtvIgL+k6reTBKjK3p693gGrLKpLlW9TeKCPx07dpSGDRuaLzyxtxXfZHjELYgK\/H+qujOnydARaxYnmPzI4aL822+\/FZxhBq54jlaYU4TFMInN0cz7WLx42DCTsVRsWLaPRdGT+apevXoZH2GvvfaasGsgW\/qnqjdDKO4Bm9TZyN6+fXvBEwfWCp41gtY\/Vb3RCw+3WDWcWylXrpxEItn5wUtVd+aE8vLyrMrRI4RHXDSgjJ+Uy7X281aHST2sEn5ZWBKAp1NufC7pmqqekyZNMpu7L7\/88qxO\/FrsUtGbX3e8ArA5HeGN7Zlnnikffvih8YtmywrymIre6MNGevy1Mf\/DMPCFF14QJuGTfUOAvJmSVHRnFHHCCSfIq6++al7KoDsuyvFDh0unTOkWtnJzjrCOP\/54YaKRX3Vu4NFHHy2spck1YBPp2a9fP\/M9R+Yeli5dKnwhiAlgrBUEVzvMVWSjTanonQ29ktWZqt6ku\/baa+WKK64wn5rbtGmT+Vp5JJJ5C8uvDejk90zbZ4W8f\/rTn4w7cobizF8xTLzxxhuzZh2iU65JzhFWJBIRFswx6cikKZ8VwyzONeAiEX89cSON8KuJO2isFFewIA888MCsNCkSSa63VzF+5XF3jeXljQvqOhJJTe9IJCJY5KwV4o0sjhSzbaFEIqnpznN+9dVXmzVY6M76N6\/fuaDwTlYPzy+eXHg2kqVNZ3zOEVY6G6dlKQKKQOlCQAmrdN1PbY0iUKoRyBnCKtUoa+MUAUUgLQgoYaUFRi1EEVAEgkBACSsIlLUORUARSAsCSlhpgTH9hfC1IV55p1JyUdKmUp5Nw1om3LSwH5I9hTY82RG90ck3XRYi8ITAkhIW7mZq8SvlUj7CeRaaWeqrVMIq9be4+A1kP+EhhxwieGwIekkD24PYf1l87WNzXnDBBTJ37tyMLX7lFT+La1u2bBlbuYakBQElrLTAWLxC2N\/Wp08fsQ7bOCcMa2CX3NYmAAAF6UlEQVTYsGHCZlhr3bDmBWsHKwGfWvh4Ys9lvLQLFy4U9tOxWJXOQ14IIJ6WrFdiYS7lumkpl03axHP0WliUR7nkoZ5zzjmnkKM8yA7XQMRRPl4UqB+d0Z02UCdtohzKwyrhYye0nXAsPPAAFxzaUdcDDzxgXMZYfdgozH5ByqIe9KWeREJecMUKtDhddNFFwu6KePm8FiPYkJ9y4qXXsMwhoISVOWyTlsw2DB56nLaxUJY9b2yOxs0IC2axCNgiQ6cdN26c4IGABaic4\/mBxYXetPjZ6tu3r\/zlL38xDuyeeOIJ4fNsfP\/RqxAO7tgv+Ne\/\/lWWLVsmblrKxbKCJDh6LSzKo1zyUA6LNfEsQHuoB0vjjjvuMDr84Q9\/EIiGldssBn799dcFp4bUyYd0IY6NGzeSTfiCOH6WcG3DLgfajLcIMMLz6ezZswU8SAzBDRgwQKgXLGgH7UEf4pPJiy++aL5Ujk58dxNismUny6vxqSKQ3nRKWOnFs8il0WkZprAqntXxN9xwQ0wZ7IPDewJbPOj0WCOQW7whEx2dVcjs\/WPTbKNGjYy74JkzZ8aUi3Vy5JFHCpuaI5GIkLZNmzYCocQk9gSQhrTkoR6sKVzRcE5StpfgMYFr9N6+fbtgXeH9AVKuU6eOccrI6u7du3cLbSLfQQcdJGzLIhwSw1snXgsgTFas46qZtpMWC\/SUU04xzvqoB3I99NBDBcuM+GTyxz\/+0ThWpC7qRMfly5cny6bxWURACSuL4GNB9ezZ0\/hrwppgWGKHTq5akBRbfNjEi+XDlg06l5vGnmMh4HUTB3AMk5Dx48fLunXrbJLokaEj9UKWNhBigFjsdbwjVhRuT0hr4yETyAPriDA3jmsrW7duFSwvJqYhCXxuufWhi\/VagM7E4a3D5sehY6VKlcwlbWVzPMNO2glBsvGZfCZBkj+uo0jqpO4kWTQ6ywgoYWXxBjAUYYM31gpExRwM8ze4d3HVwlsCQ6Lp06cLVhGWGAThprHnWAvMJzEsYviIfP7558ZNsE1jj1g5WHjWuuHIdY0aNWySuEc6d9WqVYXN3TYBQzn27UFINize8ZFHHjHBDO1wfY33SktyJsL5s\/\/++wvW1JYtW6Kh1ANREUBbe\/ToIbQRQfdPP\/3UDIeJTya4LrJpcEmDxYqlZsP8jm4+vzQanhkElLAyg2tKpUJAkA8WCxmwmnBgyC89nXH37v8fKhFPhyLtM888YywmhlKEu2mxcr766ivjCgYCoiMzQQzhkdYVyJKhIp2ctAy\/uG7durWbLOYc\/RgO\/vOf\/zQT1bhCYS6LutA\/JoMngLogO9wHYf1Bclx7khkXvDVr1pSnnnpKqAPfYqS36SDmN9980wwBKZN2\/PnPfxasP5sm0ZEhJWVS9ujRo6VWrVpmiOjNwwZkrDbqgCx5SeBNo9fBIKCEFQzOcWu58sorzdwNQz3eFDJRfdtttwm+vyATrumAvAWrW7euEMaQkHM+ZnH99dcLbmoIt2mZA7rrrrtkyJAhxqc5ZMXcEm\/dvEowt8TXT3AwyLCqd+\/e0r9\/fyHcm9Z7jTXIHBDDOlyhYPlgLUFm3rTuNd4ImJtiKMrcFOVg6THhDXG4abG8BhRMqvP2jvS4aGHOzaahTWCAKxn0Z8Kd4SbzZDZNoiP5uQeUTR3URZ1gigsg3gaS\/6qrrhKsVHyte3XwpiW9SuYQUMLKHLZJS+aXm\/ko3nAxnMFhG7\/yZGQyG2uHt4QM0Ww6Xtt36NBBBg0aJFhoTLC7aZk7YvL5jTfeMEMl0vMGLxKJ9QcViUSM\/yXSYB3x5s9NCzlSP2WikytYdZAPbzfRnzeJkCVpsBrdD2+45ZCGdtJedCQOq4l6mjdvLlgvtIlysGgYfhJPevSjbVhx1B+JRKKuZNCf8ognbyrCuinyUDY6oRv5qB9s0Y1rXBZPnjxZ+Owcw\/eBAwcK+oKLNy3pVTKHgBJW5rDVkkuIwM6dO4XlHazbYjjMEBKiYAkCVmgJi8+17KpPCgj8HwAAAP\/\/VdghywAAAAZJREFUAwCid9NHQhOG2gAAAABJRU5ErkJggg==","height":181,"width":300}}
%---
%[output:38117c63]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAASwAAAC1CAYAAADlY8JKAAAQAElEQVR4AeydC7xNZfrHny0zztAFKRRFSSiVxq2bMl3oIjQ1YSpNNyIpl07TZaJCLl1UIqkZXah0UaZ0MZXSxUhCIZeSO4NkJEnmf76v3v1\/z9p77b3POXuvvdc5j4\/nrLXe6\/P+1np\/+3nf9b7PKvc\/\/acIKAKKQEgQKCf6TxFQBBSBkCCghBWSG6VqKgKKgIgSVhl6CrSpikDYEVDCCvsdVP0VgTKEgBJWGbrZ2lRFIOwIKGGl+Q5OmzZNDjvsMBkzZkyaS95T3Pz58+XYY4+Vfv367QnI4N9t27ZJly5d5NRTT5UNGzZksCb\/osGR9tJu\/1S5F4O+6I1wnqqGacM81QqTpEN32sBznSRpINFKWIHAnLlKIBIIBWLhYc9cTWJIK6i6aAedZfTo0XLvvfdK48aNCfKVouJAB8zUDwv3YciQIVK5cmWZPHmyVK9e3VfvXI8A9zPPPFP69u0r3I9s6xsqwsKq4CGzwkMHgDwgdFiEB8TG8+tMvBU3Px2Ph5w4yiEPHYMyOOdIucTHE8omHcIvkN\/NpE7qeuaZZ4zlRV2UR3rykR+hPDfc1o8OnFOG1Zd0CNcXXXSRrFy5Uj755BO55pprhPTEWaE+t3ziveWhI2mskMfmt8f\/\/Oc\/Eq8udEA3m5eybB6OXBNHW8GXc9tW4qmLMCtcE46MHz\/edPpjjjmGS9M2dLdpqZf6kaLoRh1gRaHDhg2LWsOE27I5ck2aeEKd1E86hHPCwJeyuR\/clw4dOsj69etjiiAteciLgJM30dtvv22eGeJdzKjDxYFzwsiPzqTneaN8W677vHXr1i3Gcnbjye\/W17VrV4oW7oc5yeKf0BAWAL700kuFoOLB4AbZQB6SPn362EvhYbTx3Dg3Pw\/TDTfcYDqBzTBq1CjT8bmmrKeffprTGKFMyrYR\/\/3vf+W6664zFogNc4\/Udfvtt0eDeFhJTz4bSHmUa6\/TdaSz165dW5YuXWqK3L59u6xevVqaNWsmBx54oOmsLi4k8uJKWDz54YcfBAxpn42nLO4V1xy55py2gi\/nVmgvddlrjvaXHIxmzZolBx98sFSsWJEoGTBgQPT+EEC91E+buHYlmW5uWs4T6UK8K5AD9VK\/Decc0ty4caMN8j3Gyw9O4GUzUZ77zGBpQirEe3HgWSWMOCvkpQyuwdJ93iBC8hCHUO4ll1wi3COuEfd5xELEWuR+UBbx2ZJQEBYgTZw4UfbZZx955ZVX5Ouvv5abbrrJYPbGG2+YI3\/84skP2C1btpR58+YZ4ZybhpAXIYz4sWPHcinvv\/9+IUIjkIftiSee4FTGFqRDlwsuuMBYOeQ1EXH+oC9pzzjjDOHh5GEiH2GUQxa3LVwnEwhn0qRJAiGhO+XsvffehbKRBnKi\/eCAjtRdr149Q7Cp4EqBBxxwgHjrikQihvyom3LBEl3ADauCI3nRi3jScW2F9nrvGZ2GX3Lyb9myRVq1aiW0CdzXrFlj2ko9trwvv\/zSdLSi6IYe6IQe3Jfu3btLIl1I5wr1I5SDHgjn4MoPA2VzDRaQA8OqRPl5psEBvCBam5ZybNngAiZ+ONj7a\/NSP3lHjBhhnnd0s2GUa9Nx\/PDDDw2GhLvPI8859fGDwQ8H9wMdyJMtCQVhARJgHXXUUVK3bl2DFZ2dB4IbZX\/V3PiTTjrJEBwP+fLly4X8PGRYHAjnpiDnj+0c\/KLwADlR0VN+zbFQqJtyiOCh4EZDRlx7hbLQx4bzUHPetm1bDkI+8lOOCUjzH+qh\/eBI3VYfrgl3cYuHq586kOH06dNlwoQJgjVKh6BjkP7HH380ZGZxgnSuuOIKoozQEbg3dMT27duboQ+\/6iay4A+6EVdwav6T\/6CDDjI\/DNQD9pQ3d+7cuPNbiXQzBTp\/kuniJDWnYMiJfV7QDV0Ig\/g4JhJvfgiNdoBjpUqVTFYXN9ptAgv+UBfpwB1CAodEz3JBlqh1bfUFP4Q4xOqDtctwkCPhuSihIKx0AUdnhBhcgSzSVX6ulgMBo9tbb71lrEaXoAgvrtDRmT\/hIcdio+PQ0YpSHunJ594TiNt2IrcswumktrPRsZinwXJ003FeHN38dKG8XBKmN8AcjFw8SqIjP2JYeu59gBghSPsjzbDQPkslqaskeUNBWIAEWJj\/33zzjWmvHVYx3KlWrZoJ48FHuGBYwS80v0516tQxk7dYY\/EebtKnKl7z2O0YzIOkUg6dm3Q8cByZu+AB5EHkGsGK40GhvbSbsOIKVikkNXv2bGP12F\/aVHH1qxesEYZVED+dx1pYNg\/XhIMTQwwbTkfg3th4G26PFiN7zX2DnCApO3ShXr\/86IWQxk+3PWWLGXIm0sWms0erG0M42oXYtmHN2nR+R5vf3n+eG+4\/5O8OCePlZ76JYSY\/vt0LhrJFeT6svuCC2PLRh77C0NCGuUesXSxx+hqWqxsX9HkoCAuQOnfubMbZ3uGD9wHhgebmQ2iASTz5AZuHm19n4hE6AB2BdKkKHY0Ozw1GF2uSUy6SSjkMD\/lFYwiEHhy55m2MJRGrK3VQV7JyeQBpO53Hm9bqTBoePOonDbikiivprVAOde27775m2I3+tIMwm4YfEcrmmnCLE9dWuDecE09+K3RgcAAT4hF05R5SN2WRlnpJQ1rSIMRTXiLdSGeFMvjBSKSLTWuP3GeEutAF4RwLjXObzu9IGtLyjNIO9CUtw0o7JOQ6ntBWfrxt3lSeD8iN+tCRum19tnyeB3AEC\/SxAi6k4cefOiF\/rrMpoSAsAOLXBOA5t8IvLb+e9pqbctddd9lLMzFv4xlOuPm5QQ8\/\/LB5UxbNkOKJVxfqfeCBB8wvdSpFMGfBK343LdeE0zF5w2PjevbsKXQOe+09WovPG+69tg8llhYWl433toVwL66EId66GjVqJKzRIQ48GUKgKxYhv\/y8eeKaeDBy7w1h3Bvq4twKnYJw2zGtVUA8b8JseVwjFrei6kb56EwZCHX66UK8K\/wAcL9pkw3nnIl\/7p8N8zuShmfPrd+22y+PDSev\/SEgDJ15rvmBw5IlzCvkSfRM8dwxB+nqQ5k8G9aiIz\/leMsO+jojhIVFcNlllwm\/lBLn3+TJk80kq2XyVC0dSMcdY\/OQeYunA9k0AO7Gu\/mZ5ORGEU855LHpCSeeDsjDSRqvuGUxAWpvprcs0lEWZbpl2HTUi3Bt4zknDOE1P3rYOiiH8iiX9OhHPGk5ck24V2y+eGkoi\/xWqJ\/8lEV6W7e9Jh3hXNu86ASZEM45pMgvOb\/qdCrKmDNnDsUKQxBzUvCHuijPir0H4EnHtEPjgqTmB4HybVqO5PfGkSaRbmCBoCdl2Dopi2srNpzyvYJ+tMmm5Zww0lE3OrhhhLvi1k8Ztq54eS3G6EcZpCUPQpgbzzXhpCEtQj\/kXnB\/ILWzzjpL+FHhzR9ETxqvPpTphlMu19mWEhJWrPr\/+9\/\/hFflfuNhcqxatUpYkwOwSKIbS3qV8CFAx2PojOZ0Fn6cGMbwK451Q3gywSpkCEu+ZGk13h8BhoFYgPx4cG6HkczbcZ\/8c+ZeTAxhsaK5U6dOZv0LD1wy8TYJ5p45c6accsop3qjo9dq1a4U5jmhACU8APdkvWgmr0OzFQIBfeX6QXMGq4dc8leJIR3rKSSW9pomPAJYfRoF7Hzi3VlT8XLkZGkNYWEg\/\/fST9OjRQ\/Lz8xMKadxmMeHL2JxfVD9C4i0I628YEx9xxBFmfmbKlClCvW5ZnGOJ8aug8olZ4a04KA659AzQP+mnQUoMYUUiEbOa+PTTT5dzzz03oZDGVZahIKZn8+bN3eBC5zt37hTehNx6662yaNEiGTp0qAwfPlw+++yzQukAo3\/\/\/mbPE697VbpkGwutv4veA7cf0j\/pp4U6boYvYgirQoUK0rp1aylfvnzSqtmuYRMtXrxYPvjgA7MBNxKJ2OCYY5UqVQQrjCHjXnvtZYaeTZs2FdZIuYkBgqElZMZwLyzSu3dv0wzVe4JZAR\/EfVPMg8Pa3k8wp3\/ST80DH9CfGMJiOPjYY48JVpJ90\/fzzz8nVWfFihUyY8YMadGihXkDyEQpQ0N3MSSFLFy40OzBox6urTAZa8\/dI28yeLsRFmFCk\/VUv\/\/9781wV\/VumXEcFPPMY+x9junnbj8N6jyGsLCamFN69dVXzdAQ7wd0vjvvvFNYWxNvrgllee3JRJ4V1nHwOts7sccEOVsAGIv\/8ssvZqsIaz1YFEg5YZcaNWoIRM8xTG1B3zDqDcZh1T2X9QbXXJQYwkJJhmoNGzaUQYMGCds5Hn\/8ceHt4dlnn2026r788suSitVFWQiryVlrxXoQ5riYcMfBGZPuzGXdcsstUr9+fZKqKAKKgCLgi0BcwnJT\/+Y3vzG+kx588EGZOnWqsESfVb7fffedmyzmHMsKq4sI+1rVXmNesqsda4xhJHNmpFNRBBQBRSARAkkJC8uKoR2T5FhYrGCGsPbff\/9E5WqcIqAIKAJpRyAuYWE94WKVJfysNsayYtj26aefCpZTkyZNhGFj2rUpJQVqMxQBRSAzCMQQFvNNHTp0kIceekg4vvvuu8Kc1TnnnGP2cmVGDS1VEVAEFIHkCMQQFssLWNbAXkBWsrOsIHkxmkIRUAQUgcwjEENYeFrAjcemTZuS1o41ljSRJlAESjMC2rZAEYghLGpnh\/w999wjzFslEtKQXkURUAQUgSAQiCEshoTsEWKfIBPuiYQ0QSipdSgCioAiAAIxhPW73\/3O7CVMtvHZxlOIiiKgCCgCQSAQQ1hBVBqtQ08UAUVAESgCAkpYRQBLkyoCikB2EVDCyi7+WrsioAgUAQElrCKApUlLgoDmVQRKjkBCwmJNFu5lGjRoIHhbeOedd+Tmm282nhtKXrWWoAgoAopA0RDwJSz8Xt1\/\/\/3GjcxTTz0lNWvWNJ9n4rNA9913n+zatatoNWlqRUARUARKiIAvYW3cuFEWLFggrMk69NBDpVy5cpKXlyeXX365ceTHBukS1q3ZFQFFoHQikLFW+RKWX434esdHll+8hisCikDpRmDF5h3Sfdr\/ZOtZQ2X699UDbawvYfGZrkaNGsndd98tONrDnfH69etlzJgxxnUyH5MIVFOtTBFQBHIGAUhrd8VqgevjS1iRSESYcMdbw6WXXmq+aoO7GYaCDBOxtALXVitUBBSBrCMAWWVLCV\/CQiE+GHHHHXeYuSw+6cOcFn6y1LoCnYyJFqwIhAaBQ6r+LlBdfQlr69atMnToUOOxAdLijeHAgQPNNR4c8Ei6evXqQJXVyhQBRaBsI+BLWMDCm8LXXntNIC97zSfACOczXR07diRYRRFQBBSBQBDwJSzeBO7cudN89JQvNQ8ePNicP\/roo4JHh2HDhhk3yiXRkoWpfAuPz3+VpBzNqwiEFYEw6r3yux1ZU9uXsCATvphTt27dQsrVq1fPrHQn3htXKGGSCxamTpw4UXDFnCSpRisCikCOIlC7bnWziAAAEABJREFUSl6gmvkSFo78mHRn6Ae5oBVHJt85x8paunQpp8WSL7\/8UiiLz4cVqwDNpAgoAllBYMXmH7NSL5X6EhaEdP3118vIkSOlRYsW0qpVKznhhBPktttuE5Y5LF++3KyCp5CiyrZt24Rh5jXXXCOs90qUH1Kjrh07dkhY5KeffpIff\/wxNPpaXMOqN\/qHVfew6U1f\/GjesmiXLbd9Y\/Q8iBNfwqLyo48+WvhCM1\/Ryc\/Pl+HDhwtfauaDqg0aNJC33nqLZEUWhoJ8sr558+ZJ80KY48ePl7Vr18ZILoatW7fODJlZZJuL+vnpFFa9aU9YdQ+j3vTFj+fvGVlBVkjSTpzGBAkJiyEgc1VsfG7WrJkceeSRsnLlSunSpYvwoYpKlSoVWZXFixfLBx98IFhXkUgkaX5IsmvXrmbzNXrkulSvXl0OOOAAqVGjRmh0BtOw6h1m3cOIeet2F8uuakeafltue\/Iva5mEafyTkLDGjRsnEFXLli3FygUXXCB16tQp9kdVV6xYYaw0hpmHHXaYvPTSS4a8+vXrF7dZrLSnvry8PLP5OtePDKUrVKgQCl1dLMOqN20Iq+5h1Hv1L5Wj\/bT8xkXR86BOfAnr+++\/F776PGHCBHnuuefMvNWSJUvkuuuuk6OOOsosbSiOkmeccYbZm8j+RAQCHDt2rIwYMaI4xWmesoKAtjPrCLAlp+fEhUaPcgVzV3mLXjXnQf7xJSwmA1GEpQuVK1eWTZs2mYnk888\/X6ZPny4QGvEqioAiUPoRcMmK1tbJ+4FD4OJLWJirbHBm6QKExVwWgoaQlSU0rksiWFZYXSUpQ\/MqAopA5hCwZPXhsi2mkkOq5snDnRqY86D\/+BIW67A6d+5sli6sWbNGDj\/8cGnfvr0RJpSrVq0atK5anyKgCASIAEQ1cdY6Of+ROeKS1as9mmRYC\/\/ifQmLLCxfYPjHnBUuZe6991558MEH5Z577hGsL9KoKAKKQOlCwCUq5qy4poVYVpAVR66zIb6ExYZnJsO3b99uyIkhIqvSjz32WHn++eeFxZ\/ZUFjrVAQUgcwgADENffMbOe7uj8VLVPlt6sjnt50g2SQrWh1DWKzQ5u3g66+\/Li+88IJwxGODlZdfftmEQ2QUoKIIKALhRACCYsgHOVXt864hqqFvLo82BnLq3KyGvFowBMxvU3hPcTRRwCcxhIXlhEcGts58++23ZgsNfrGs4AfrkksuMYsjA9ZVq0sJAU2kCMRHwCWodqPmGIKCrCAtNwdElV9gUUFUozo3zLpV5eoWQ1is0n722WeNZdW3b19zfP\/998UK23EuvvhiiUSSr1J3K9JzRUARCA4BS04M8SAna0FZgrKT6FYjSMpaUwz98gssKsJsfK4cYwiLj03gVoYNpR06dDAbeDds2CCuEE+6XGmE6qEIlFUEIKYZS7cIVhJk5CUnhnhecgIryAiCwoLCkoKkOD+5XmWic1ZiCIsFohdeeGF0K47dkuMeiSddzrZKFVMEShECkBICKSEOMZlhHcsOCCPOj5z8CArSynWScm9lDGEdeOCBZiU722b8hKUOpHML0nNFQBEoPgIQkrWUGMZBQFhLvLGzQhjiR0zUDjGddHhlgYiwmKz1ZC0owsNEULTJlRjCciPx1jBnzhzhIxTsIWT9FSTmptFzRUARSIwAZIRYQoJwIJ6LHl8o7cavkoNu+biQpcQwjjRYS+SLV3oiYprSs4lAVmEnp3jtTkhY+MLq2bOn7Nq1S5o0aWI2LTOvNXXq1HhlaZgiUCYRgFS8ZAQheS0kO3QjzhLSmq27fDGDlBCIB4GEsJg239farInyEhNpfQsrJRG+hMW+QRzt8XmvQYMGyZVXXmk+QoEzP8KJDzkGqr4ikBABiAhJREb27ZuXjCwhkd+vkoP2LS+IO4TzkpIdyhEOaYV5OOeHQ1HCfQmLBaRYVnU9H6HgmnDii1KRplUEcgUBSARJhYiYPyoOGdFWLB5LRpANpINYK+nTm5vJlK61ZNKVDaNDONKVdVICOz\/xJayKFSsKn\/pauHBhobzffPNNdKtOoQi9UASyiABDKy8JuZPXdnhmLaKSEBHNTERGWEXeYRtEBRkhSkggWDzxJay9995brrjiCmHxKC6R+dozbo2vvvpqwYsD3hyKV6XmUgRSQwACQlxLKB4JMWnN5HXLEXOMZwHmiBB38jrRBLarDUSEWMsov00dY\/1gFSGpkBH53TL1PH0I+BIWVZx66qlmpXvr1q25lIYNG8orr7wieHEwAfpHESgCApAPAgEhzPMgkAuCFYRg\/fhZQukgIaycREQEKdkJ7fw2dc0SAawiRMmoCDc8A0l9Ceu7776TMWPGyA8\/\/CBXXXWVDB48WG688UZhDisDemiRIUQA8rHiElA8K8hLQH7zQqlaQsAFeSAtDt1b2jXc2xALQy8EawixQzOXhIhXIgLB8IkvYe21115mGQPWFB5BeTsIiYWviapxqgi480AuAWH9IFg\/iNcC4toloOJYQegI+SAMxxDXEoJkICAEEkIgIeTlbo1lwBnV5N6Oe6wh8mENIZSrUnoQ8CWsfffdV4YNGybz588XFoxyZIh40UUXyTvvvCO6lzC3HwJr+XB0ycdaP14C8s4DuQTEsA3B+kEoM9XWQ0CIJaDOzWpIfoJ5IQiI4RgCSeX\/OiQjHwSEpFq3pit9CPgSlm0qbwr51NfIkSNl0qRJ5s3hwIEDzUcpbBo9Zh4BSAJJhXzc4Zef9eNHQJCLX2uIsxKPgCAYLCBIBwsI4RyZ8uvqa9JYEoJ8EMr0q1PDFQEXgYSEhRXFsoZbb71VmjdvLvjBOvroowWfWLihcQvS89QRgHiQVMnHJSDX8rFDLz\/ySaYRRIFAPnYeiHNIBYF8EAjHJR+u4xGQtYIoM1ndGq8IFAcBX8LChUzbtm2lY8eOwhdycJf8ySefCMsbatWqFdcf1hdffCHk4QOpJ598svmuYTylJk+eLKSxwlAT9zXx0oYhDPJBIKBJc\/4jUxZuk3v\/tdK4mWXOB8HSgXgQzpFMkA9kAelAHkj+r8OvRAQE+UTngTrGzgNRZhjug+pY+hHwJSx8uPNmkLkrPsXFXkIm4v0gYasO6Xv06CGLFy+W\/Px8wUvp2rVrY7KsWrVKRo0aZSb12Uydy94fLBFhxdj5HwgIgXwQyAeBgHpPWioDpm0sIKxVxkcRcz4I5cQA4RMAQVgpKvkks350COYDugaHAgFfwmLhKHNXzGGl0pKVK1fK7t27BcuKL+q0aNHCzHdt3LgxJjskVq1atZjwbAZAKJASYiekLRlBRITZIRgEhKSqr5JPqkhpOkUgMQK+hJU4W2xso0aNBNfKfK8QtzQzZ840W3j4hqGbmnVdfOfw9ttvlyOOOMI4CpwyZYqQx01nzyln+fLlxvMpXlDTJV9v2GYsIIgIawkLiXME0kpESGxYRX5\/cJ50bFzZSO9Ta8iIDnXkqT\/XlYfOrSLvXddA1gw+wcgn\/ZoIwp4xXr0jvU+tafKRv2mtPEEOrChpb2dR8GLozx7RouTJlbRh1T27eu8o8vNGX2RqaPXq1baLBnpMG2FZrRlCHnfccWZLD3NTlStXtlHmuHPnTqlUqZIwkb9o0SIzbBw+fLh89tlnJt77h7eT48ePF6yydMjsr1ZKt6fmS9N7Zpk5Jj9ysoTEgkTW+Iy9oIYgs3vVMRtW2bTK9W2nVRbksmPypHVtkcMr7ZBDfvtf+e3O79OmczranayMdevWCfOW69evD5XetCusuodRb\/oiW\/X4Tqm3rwZx7UtY\/NLCpBxdRbiePXu2\/Pzzz25w9Lxx48Yyd+5cwZeWlWhkwUmVKlXMl3j4xiFzYq1atZKmTZvKrFmzCmJj\/0NmXbt2lZo1a5ZIfq5QRa55aZ1xmMakuFsTQzYmqO3ENJaR2Ulf8Cr+0UsbyzV\/OELOa1rXSDI9qlevbr4ohGWZLG0uxYdVbzAMq+5h1Ju+OGHCBOndu7fbhQI7jyEsljLwS4tXBr70zJE3eFa++uorYXLdu+p9xowZ0q9fP\/NGEe15A8gSCK\/pyDIJ3jhiCpPOit9m6oMPPljq1KkjeXl5xZIN20VGTl8rbIydvXqHrU4gKd6g2df2kBWkdXK9ysWqx+rHy4oKFSqUqAxbVpDHsOoNRmHVPYx60xdbtmwpzFFHO1OAJzGExccl+MjEeeedJ1hSHFHQCivdsaKwlFw9WZeFZTVv3jyzCp4lDgsWLBAm7t10TOazgRrrDXLk82EMI73p3DzFPWcinQlzJsttGRCVJan8NnUFgrJxRTtqakVAEQgagRjC4uMS06dPFya7ISuOLD2wsmTJEhkwYIB5A+gqW79+fbn++uulV69eZjKdDdPXXnutHH\/88eYTYcxnTZs2TWrXri1MuA8ZMsSkYy6LtV3kd8sr6TlLEJhIh7QoyyUqJSkQUVEEwodADGFh9WzevFmYLIdYeHtnh4P2yJCRdG5zI5GItGvXTrCcIDeOXEciEbEkyCZq8mCtMb9FOoaS1n0NcekQyMq1qvLb1DE+sJWo0oGulqEIZA+BGMJiSMgXcpirYmgIuXiFcNJlT23\/mnnr5yWr\/IKhn38OjVEEkiKgCXIEgRjCwhriLQAT5gwNsYK8QjjpcqQNUTUY\/rGOygbkF1hW+UpWFg49KgKhRyCGsMLaIiWrsN451VsRSB0BX8JivoqJcpYneIVw4lOvJvMpJ85aK3Z1OssT1LLKPOZagyIQNAK+hMWQj6GfHQ4uW7ZMPvroI+nWrZtxM8MyhhSVzXgyrCs7b8XbwHwdBmYcc61AEcgGAr6E5VUmEokIq7e7d+8uTMh7F4560wd5XXjeqq5ZFBpk\/VqXIqAIBINAyoRl1WEv4NatW83iUBuWzeOMpVuiQ0GsK4aD2dRH61YEFIHMIeBLWKy16tSpk7DXz5WzzjrLrKvyrnTPnIqJS2bNlU3xcKeG9lSPWUJAq1UEMomAL2GxhYb5KhzxufLkk0\/KwIEDjeuYTCqWStnMXdmJdhzd6cLQVFDTNIpAeBHwJSw2Zp522mly7LHHGn\/u5557rrCHsFatWoKXhVxosmtd6UR7LtwR1UERyCwCvoRFtePGjZMzzzxT8FvF9XvvvScsaZg6dSqXWRWsK1a1owRzV2pdgYSKIhAgAlmoypew2E\/IZmVWveO7Ct0uu+wyGTNmjEycOFHw4U5YtsQOBalfJ9pBQUURKP0I+BLWrl27zNAPf1QuDPXq1RPicOTnhgd9PuHfez5ugXXVuVnNoKvX+hQBRSALCPgS1n777SdMvL\/44ovRJQx4bsALAx+mqFixYhbU3VMlw0FrYdWukqfrrvbAon8VgVKPgC9h4TUT\/1YvvPCCHHfccWZ5A76tBg0aJH369DFkli10LFlRf5fmIbWuUF5FEVAEioSAL2FRCh4b8Fv19NNPm+8Mjh49WvAQyptD4rMlhYeDNbKlhtarCCgCASPgS4o3c1gAABAASURBVFh8iouFoywghaBY1oBfLCyvgHUsVJ13OFgoUi8UAUWgVCPgS1hsfm7YsKF8+umnvt8MzAYyEJatV5cyWCT0mNsIqHbpQsCXsNjczIck+vbta76Q4W7PsZZXupQoSjkfLvsumvykw6tEz\/VEEVAESj8CvoTFG8IePXrIAw88IHfccYeZw7JbdNiyQ3w24GGzM\/WynEEtLJBQUQTKDgK+hMXWHD4OwdyVVwgnPmiYGA7aN4QsZwi6fq1PEVAEsotADGExyc5yBj54ytDPHQrac8JJV1zVGWq2bdtW8GR68skny7vvvpugqP+PgrDsle4dtEjoUREoOwjEEBZDvS5duhhnfQz97DDQPRJOuuLAxJaewYMHC8PNxYsXm6Hm0KFDZe3aPSvXE5Xpzl8lSqdxioAiUDoRiCEshnosX6hSpYrE89Zw4oknSkmGhCtXrpTdu3cLllX58uXNhD4r5zdu3JgUYZ2\/SgqRJlAESjUCMYTltjYT3hoaNWokzz77rFStWlXY6sOXpSEu3C+7ddtz4pcvXy5fb9gW9Sxac5\/ysmPHjpyVn376Sdhrmcs6xtMtB\/Qu9j0Nq+5h05u+yPa81atX2y4a6NGXsDLtrWH+\/PnClh+WTeCypnLlynEbPnLkSBk\/frx8tnhlNP7Y6uXNEJJhZK7JunXrhPm99evX56yO8TALq960Jay6h1Fv+iJTRv3794\/2xyBPfAkLjww46suUtwacAc6dO1fY+mMlXsOHDx8uXbt2la++Lx+NbntcbalZs2ZOSvXq1YUvCmEx5qqO8fQKq960Jay6h1Fv+iIup3r37h3tj0Ge+BJWprw1zJgxQ\/r16yeYwjSUN4XsWfQzMSHMOnXqyCfLt5HcyGkNqkleXl5OCnOAbF\/KVf389Aqr3rQnrLqHUW\/6InPcLVq0MH2xJH+Kk9eXsOh0LG9It7cGrA8sq3nz5hm3NSxxWLBggTRr1iwl\/VkwmlJCTaQIKAKlDgFfwqKlWD4M19LpraF+\/foCEfbq1UuOOOIIueqqq+Taa68VXNdQZzxh\/ZUuGI2HjIYpAmULgYSEBRQsOUint4ZIJCLt2rUT3jR8\/fXX5sh1JBKhurgCYdkI3Y5jkdCjIlD2EEhKWLkAibtgVDc877kj+lcRKIsIhIKwXAtL57DK4mOqbVYE9iAQKsKCrJA9qutfRUARKGsIJCSsrVu3ynPPPSe33HJLIWHvH3FBgaUT7kEhrfXkJAKqVBQBX8JindRNN90kY8eONXv\/ojkCPln53Y5ojWpdRaHQE0WgTCLgS1jff\/+94HX073\/\/u9xzzz2ChwUreG7Yd999AwFM568CgVkrUQRCgYAvYbEKd\/\/99896I6yHBhTRN4SgoKIIlF0EfAmLD6XiRoa9fEuWLJENGzZEhc29v\/zyS+CoFWtIGLiWWqEioAhkCgFfwtq0aZM8\/PDD8tprr0mbNm2E\/UNWLrzwQiE+U0q55doJd8gKceP0XBFQBMoWAr6ExWe+pk+fLqxG9wrhxAcB1e6K2R+WBtFOrUMRUASSI+BLWGTFwR5+q5h0ZykDQ0N8vRNOfKZld8VqglBP7Sp5HFQUgQQIaFRpRyAhYbHx+corr5Rly5YJVhVO\/di4jD+coIHRPYRBI671KQK5h4AvYfGxiIkTJ8pDDz0kgwYNEjyC1q1b1yxxYF6LZQ+Zbo47HDyk6u8yXZ2WrwgoAjmOgC9h4ZMcr6OQlNuG2rVrm0sWlpqTDP7ZVe3IaOk6JIxCoSeKQJlFwCGswhjss88+wqe8cAPjLmFYunSpsOQBKZwjs1eHVM3LbAVauiKgCOQ8Ar6ExcLRnj17mhXu7du3FybfL774YuN879JLLzVklunW7arWIFqFElYUCj1RBMosAr6EBSI47nvrrbfk5ptvlo4dOxrvoK+\/\/rrwlRvigxIlq6CQ1noUgdxGICFhoTp7Bs8991zBpXGnTp0kqPVX1G0n3XX+CjTSKlqYIhBKBBIS1po1awSSatSokfCVjAYNGhjiYmtOplvL+isk0\/Vo+YqAIhAeBHwJi7eAd955pxx00EHG7zprsT744APBx\/vdd98tvEEMqpm6BisopLUeRSC3EfAlLNZZbdmyxXxDkE9zRSIRqVGjhtxwww3CF2tZRJrJprnW1SG6BiuTUGvZpRyB0tQ8X8LiLSGLRbG0vA3G7Qzx3vBVq1aZIeThhx8uxxxzjPB5sHjbeCZPniyHHXZYVJjExxuEW95Kx3GfzmG5yOi5IlB2EfAlLNZZde7cWW6\/\/XbhzSDzWWzPwQtpq1athIWlLslwzRAS8lm8eLE8++yzMn78eJk5c2YMuhDbqFGjohurp0+fHjOZv2LzjzH5NEARUATKNgK+hLV582YZPXq0rFixQtj83KngDSHkBXE98sgjgosZxMLHVh6GkbxR3GuvvYSJepZFfP755zZJ9Lh27VqpVq1a9DreietpVOew4iGkYYpA2UPAl7CYt8JKev\/99yWRWMiqVKliCO7ggw82QbhXZqKeeS8T8OufH374QSA9yI8vP+Nja8qUKeIdOlrCKrd9oyxfvlx27NiRFSlOvQyjsTiLkzebecKqN5iFVfew6U1fZPfL6tWrf+3RwR58CQs1fv75Z3nyySfljDPOMMsa2rVrZxz6uVt1SIfw9rBq1aqCdUVjevfuLWzvOf3004mOys6dO6VSpUpy6623yqJFiwS3NXg1\/eyzz6JpvCcMLbHKwiC8kGDZx\/r16yUM+lodw6o3+odV9zDqTV\/s0qWL9O\/f39tNA7lOSFj\/+Mc\/5KWXXpI+ffrIuHHjzDBwxIgR8sQTT8RVDoJ77LHH5Pzzz5ejjjpKHn30UUNabmIsMTyZnnLKKYbcmA9r2rSpzJo1y00m1tPoCY3rSdeuXaVmzZqhEBrBnB\/HsOiMnugbRr3DrHsYMacv4l4KgwT9gxZfwmIO65133jEW0DnnnCONGzc2xDFmzBghnHhXWYZ0fGGHh\/6VV14RvqwT700iDgD5dBimsJsfa8xer9i8w55Kue2bhF8i5sLCIF9++aX5luObb74pYdDX6hhWvdE\/bbp\/\/nmg9yyMetMX6Zy1atXiELiU86uRhaGRSESwiNw0XEcikZiFo7z5e\/vtt+W+++6TRI3BAwSExjiYoSXzY2ysbtasWbSaQ6rmyej2B0jr33wpcybdK5igYZH+v5rKI0eOVL0Lhg5B3DfFvEvgzxqYs\/slUV+Pdug0nvgS1n777WfIaurUqQKxUCdWFEQD6bgWEXEwL78YzFm5a6ywyFj+wHKHadOmCf60mHAfMmSIMOnOXNYtt9wi9evXp5ioXHzq0TKyVwd5\/sE7BBNUZYLiMEExyKV+wNxzzhBWhQoVzKp2vI4yx8RcE4yK91HczniHe1hICxYsiK6tsh+u6N69u1ljxVorJu9hJN4M4n6ZNDNmzBA+J0a4VwCDtCotC321SPFQPHLhGaB\/evtspq99LSwqxgJiyQGT78xJPfjgg2aJA+uriC8keqEIKAKKQIYRSEhY1M1yBQiKBaGwOpYX4SqKgCKgCASNQFLCClohrU8RUAQUAT8ElLD8kNHwBAholCKQHQSUsLKDu9aqCCgCxUAg5wiLpRO4pcE9DR5O8QCxbdu2YjQts1lS1ZMlIWxvoj243enUqZPZS5lZ7fxLT1VvtwSWtrDCOZv3oSh6f\/TRR3LyyScb90Vt27aVL774wm1O4Oep6s7+U7aq8dzzwuuSSy7J6rOSCCgcHVx22WXm4zSJ0qU7LucIiz2Fzz33nLC49N\/\/\/rfxFsH2oHQ3vKTlpaonnef555837aHjsPwDEvau9C+pPqnmT1VvWx77QlkMzLYrG5aNY6p6r1y50uzOuP\/++wU3R+edd56wXYyF0NnQmzpT1Z11inPmzJGPP\/5Y5s2bZxxm8iFjysiiFKqaH2DWYv7tb38T1l0WigzgIucI69133zVf5albt67wAQwskn\/961+Cl4cA8Ei5ilT15KaedtppQntYu3bWWWcJJMAvVMqVpTFhqnpTJZ2cbVQnnnhizJ5Q4oOUVPVm50STJk2EtYPly5eXbt26yYABA8y+1SD1detKVXebB9ztefXq1e1pThzph6ypxMkBz3PQSuUUYXGjcF9Rr169KA7cMPYtAlQ0MMsnRdGTrSksnrUqL1myROhI2VgeUhS90fe9994ThoFsZuc6W1IUvbFicTdz9tlnC0NwvqHJsxOJRLKiflF0Z2E1bp1YoI3zABZi8y3QrCjuUylGBGsyb775ZrMTxidZxoJzirB40CAntv5krMVpKLgoenKDaQ+mNEPdwYMHyzXXXCNsfUqDKkUqoih647OMBcNYKL\/97W+LVE+6ExdFb4aueP5gSIj7opNOOkkYgjM\/lG69UimvKLoz9cE2Nj72wpCQeU+cZ0J6qdRVFtLkFGGxSBXXzDx0LvhYJJFIdn4hXT3seRH0NFm++eYb45oHbxaPP\/648OtvIgL+k6reTBKjK3p693gGrLKpLlW9TeKCPx07dpSGDRuaLzyxtxXfZHjELYgK\/H+qujOnydARaxYnmPzI4aL822+\/FZxhBq54jlaYU4TFMInN0cz7WLx42DCTsVRsWLaPRdGT+apevXoZH2GvvfaasGsgW\/qnqjdDKO4Bm9TZyN6+fXvBEwfWCp41gtY\/Vb3RCw+3WDWcWylXrpxEItn5wUtVd+aE8vLyrMrRI4RHXDSgjJ+Uy7X281aHST2sEn5ZWBKAp1NufC7pmqqekyZNMpu7L7\/88qxO\/FrsUtGbX3e8ArA5HeGN7Zlnnikffvih8YtmywrymIre6MNGevy1Mf\/DMPCFF14QJuGTfUOAvJmSVHRnFHHCCSfIq6++al7KoDsuyvFDh0unTOkWtnJzjrCOP\/54YaKRX3Vu4NFHHy2spck1YBPp2a9fP\/M9R+Yeli5dKnwhiAlgrBUEVzvMVWSjTanonQ29ktWZqt6ku\/baa+WKK64wn5rbtGmT+Vp5JJJ5C8uvDejk90zbZ4W8f\/rTn4w7cobizF8xTLzxxhuzZh2iU65JzhFWJBIRFswx6cikKZ8VwyzONeAiEX89cSON8KuJO2isFFewIA888MCsNCkSSa63VzF+5XF3jeXljQvqOhJJTe9IJCJY5KwV4o0sjhSzbaFEIqnpznN+9dVXmzVY6M76N6\/fuaDwTlYPzy+eXHg2kqVNZ3zOEVY6G6dlKQKKQOlCQAmrdN1PbY0iUKoRyBnCKtUoa+MUAUUgLQgoYaUFRi1EEVAEgkBACSsIlLUORUARSAsCSlhpgTH9hfC1IV55p1JyUdKmUp5Nw1om3LSwH5I9hTY82RG90ck3XRYi8ITAkhIW7mZq8SvlUj7CeRaaWeqrVMIq9be4+A1kP+EhhxwieGwIekkD24PYf1l87WNzXnDBBTJ37tyMLX7lFT+La1u2bBlbuYakBQElrLTAWLxC2N\/Wp08fsQ7bOCcMa2CX3NYmAAAF6UlEQVTYsGHCZlhr3bDmBWsHKwGfWvh4Ys9lvLQLFy4U9tOxWJXOQ14IIJ6WrFdiYS7lumkpl03axHP0WliUR7nkoZ5zzjmnkKM8yA7XQMRRPl4UqB+d0Z02UCdtohzKwyrhYye0nXAsPPAAFxzaUdcDDzxgXMZYfdgozH5ByqIe9KWeREJecMUKtDhddNFFwu6KePm8FiPYkJ9y4qXXsMwhoISVOWyTlsw2DB56nLaxUJY9b2yOxs0IC2axCNgiQ6cdN26c4IGABaic4\/mBxYXetPjZ6tu3r\/zlL38xDuyeeOIJ4fNsfP\/RqxAO7tgv+Ne\/\/lWWLVsmblrKxbKCJDh6LSzKo1zyUA6LNfEsQHuoB0vjjjvuMDr84Q9\/EIiGldssBn799dcFp4bUyYd0IY6NGzeSTfiCOH6WcG3DLgfajLcIMMLz6ezZswU8SAzBDRgwQKgXLGgH7UEf4pPJiy++aL5Ujk58dxNismUny6vxqSKQ3nRKWOnFs8il0WkZprAqntXxN9xwQ0wZ7IPDewJbPOj0WCOQW7whEx2dVcjs\/WPTbKNGjYy74JkzZ8aUi3Vy5JFHCpuaI5GIkLZNmzYCocQk9gSQhrTkoR6sKVzRcE5StpfgMYFr9N6+fbtgXeH9AVKuU6eOccrI6u7du3cLbSLfQQcdJGzLIhwSw1snXgsgTFas46qZtpMWC\/SUU04xzvqoB3I99NBDBcuM+GTyxz\/+0ThWpC7qRMfly5cny6bxWURACSuL4GNB9ezZ0\/hrwppgWGKHTq5akBRbfNjEi+XDlg06l5vGnmMh4HUTB3AMk5Dx48fLunXrbJLokaEj9UKWNhBigFjsdbwjVhRuT0hr4yETyAPriDA3jmsrW7duFSwvJqYhCXxuufWhi\/VagM7E4a3D5sehY6VKlcwlbWVzPMNO2glBsvGZfCZBkj+uo0jqpO4kWTQ6ywgoYWXxBjAUYYM31gpExRwM8ze4d3HVwlsCQ6Lp06cLVhGWGAThprHnWAvMJzEsYviIfP7558ZNsE1jj1g5WHjWuuHIdY0aNWySuEc6d9WqVYXN3TYBQzn27UFINize8ZFHHjHBDO1wfY33SktyJsL5s\/\/++wvW1JYtW6Kh1ANREUBbe\/ToIbQRQfdPP\/3UDIeJTya4LrJpcEmDxYqlZsP8jm4+vzQanhkElLAyg2tKpUJAkA8WCxmwmnBgyC89nXH37v8fKhFPhyLtM888YywmhlKEu2mxcr766ivjCgYCoiMzQQzhkdYVyJKhIp2ctAy\/uG7durWbLOYc\/RgO\/vOf\/zQT1bhCYS6LutA\/JoMngLogO9wHYf1Bclx7khkXvDVr1pSnnnpKqAPfYqS36SDmN9980wwBKZN2\/PnPfxasP5sm0ZEhJWVS9ujRo6VWrVpmiOjNwwZkrDbqgCx5SeBNo9fBIKCEFQzOcWu58sorzdwNQz3eFDJRfdtttwm+vyATrumAvAWrW7euEMaQkHM+ZnH99dcLbmoIt2mZA7rrrrtkyJAhxqc5ZMXcEm\/dvEowt8TXT3AwyLCqd+\/e0r9\/fyHcm9Z7jTXIHBDDOlyhYPlgLUFm3rTuNd4ImJtiKMrcFOVg6THhDXG4abG8BhRMqvP2jvS4aGHOzaahTWCAKxn0Z8Kd4SbzZDZNoiP5uQeUTR3URZ1gigsg3gaS\/6qrrhKsVHyte3XwpiW9SuYQUMLKHLZJS+aXm\/ko3nAxnMFhG7\/yZGQyG2uHt4QM0Ww6Xtt36NBBBg0aJFhoTLC7aZk7YvL5jTfeMEMl0vMGLxKJ9QcViUSM\/yXSYB3x5s9NCzlSP2WikytYdZAPbzfRnzeJkCVpsBrdD2+45ZCGdtJedCQOq4l6mjdvLlgvtIlysGgYfhJPevSjbVhx1B+JRKKuZNCf8ognbyrCuinyUDY6oRv5qB9s0Y1rXBZPnjxZ+Owcw\/eBAwcK+oKLNy3pVTKHgBJW5rDVkkuIwM6dO4XlHazbYjjMEBKiYAkCVmgJi8+17KpPCgj8HwAAAP\/\/VdghywAAAAZJREFUAwCid9NHQhOG2gAAAABJRU5ErkJggg==","height":181,"width":300}}
%---
