clear;
[model, options] = init_environment('afe_dqctrl');

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

% t_measure = simlength;
t_measure = 0.4;
tc_factor = 200; % tc is ts_afe / tc_factor
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
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:11b45d93]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:6b4e5beb]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:00350d58]
hwdata.psfbc = single_phase_psfbc_hwdata(application_voltage, psfbc_pwr_nom, glb_time.fPWM_PSFBC); %[output:1f005f34]
hwdata.three_phase_dab = three_phase_dab_hwdata(application_voltage, three_phase_dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:968f72be]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:41ba7574]

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
parasitic_dclink_data; %[output:768a1eeb]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:1e676430]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:546837fb]
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
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:40df2e29]
psm_ctrl.kp_i = 0.25;
psm_ctrl.ki_i = 35;
%[text] #### Induction Motor Control
im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:8cf204db] %[output:711af29b]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:8cf204db] %[output:711af29b]
%[text] #### AFE control (with sequences)
afe_ctrl = ctrl_afe_setup(glb_time.ts_afe, grid_emu.omega_grid_nom);

afe_ctrl.kp_udc_ctrl = 4;
afe_ctrl.ki_udc_ctrl = 45;

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
sogi = sogi_filter(omega_set, sogi_delta, kepsilon, glb_time.ts_afe); %[output:2919b5dd]
%[text] #### Current control parameters DQ PI
dqvector_pi.kp_inv = 0.5;
dqvector_pi.ki_inv = 45;
dqvector_pi.pi_ctrl = dqvector_pi.kp_inv + dqvector_pi.ki_inv/s;
dqvector_pi.pid_ctrl = c2d(dqvector_pi.pi_ctrl, glb_time.ts_inv);
dqvector_pi.plant = 1/(s*grid_emu.trafo.Ld1 + 1);
dqvector_pi.plantd = c2d(dqvector_pi.plant, glb_time.ts_inv);

G = sogi.fltd.alpha * dqvector_pi.pid_ctrl * dqvector_pi.plantd;
figure; margin(G, options);  %[output:4ef7c339]
grid on %[output:4ef7c339]
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
used_device = 'mitsubishi_CM1200DW_34T';
% used_device = 'infineon_FF1200R17IP5';

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
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage_1, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:5fc65039]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage_2, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:7a96c431]

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
    set_param('afe_dqctrl/afe_dq_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('afe_dqctrl/afe_dq_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
    set_param('afe_dqctrl/afe_dq_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
else
    if use_thermal_model
        set_param('afe_dqctrl/afe_dq_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_dqctrl/afe_dq_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'off');
        set_param('afe_dqctrl/afe_dq_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    else
        set_param('afe_dqctrl/afe_dq_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_dqctrl/afe_dq_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_dqctrl/afe_dq_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    end
end


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:11b45d93]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V\nNominal Voltage: 690 V | Nominal Current: 2.324276e+02 A\nCurrent Normalization Data: 328.70 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:6b4e5beb]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:00350d58]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:1f005f34]
%   data: {"dataType":"text","outputData":{"text":"Single Phase PSFBC: PSFBC_1200V\nNominal Power: 275000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls1: 3.030303e-05 [H] | Internal Tank Ls2: 3.030303e-05 [H]\n---------------------------\n","truncated":false}}
%---
%[output:968f72be]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: Three_phase_DAB_1200V\nNominal Power: 750000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 4.000000e-05 [H] | Internal Tank Cs: 750 [F]\n---------------------------\n","truncated":false}}
%---
%[output:41ba7574]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.580123e-05 [H] | Internal Tank Cs: 4.734509e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:768a1eeb]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAT4AAAC\/CAYAAABno7D5AAAQAElEQVR4AeydCbxN5frHnyVKUSFzl44h0Y2IU6YyREpCiT8pqS4ZbiWV08wpEgoVhcQlQ0kDbobIUEfmi7gpyZBMkakrhPx9X63dPsfZ2x7XXnvvp0\/PWWu96x1\/79o\/z\/u+z\/u8OU7qf4qAIqAIJBkCOUT\/UwQUAUUgyRBQ4kuyDtfmKgKKgIgSXzZfgQYpAopAYiOgxJfY\/autUwQUgWwQUOLLBhQNUgQUgcRGQIkvsfs3cq3TnBSBBEJAiS+BOlOboggoAoEhoMQXGE4aSxFQBBIIASW+BOpMbYrTCGh58YqAEl+89pzWWxFQBEJGQIkvZOg0oSKgCMQrAkp88dpzWm9FwJ0IxEWtlPjiopvcV8k1a9bIddddJ9dff73ccMMNUrVqVWndurVs37494MoOGzZM5syZE1B84pUvXz5TeS+88IL873\/\/k0OHDsnTTz8t69atCygvjaQIKPHpNxAyApUrV5YZM2bIF198IUuXLpUyZcqY55AzPEvCa665xuRPeRkZGSb2yy+\/LOedd5689NJLUqFCBRMWyp+TJ0\/KiRMnQkmqaeIQASW+OOw0N1b5999\/l4MHD0rhwoXl2LFj0r9\/f6MRohVyTxjSr18\/SU1NlYYNG8p\/\/vMf0xRIZ9y4cUZzrF27ttianHnp48\/5558v7du3l\/\/+97\/y\/fffy4MPPihoob\/++qt0795dqlevLtdee6288sorpj6EP\/zwwybs7rvvlg4dOpj4aJ1dunSR2267TRYtWiTz5s0z9ahZs6Y0adJE1q5dKz\/\/\/LO0aNFCevbsKdQPzXbs2LFSv359k9\/cuXN91FKD3YqA08TnVhy0XiEgsHjxYqlUqZKULl1a\/v73v8vu3bsNacycOVO2bt0qaGXI3r175aOPPhLCf\/rpJ6Mh8gxRUixxGKZ+\/vnn8uWXX0qePHlkypQpvPIrBQoUkIIFCwqkZkdcsmSJlC1b1pAY5X333XeyY8cOeeedd4xGyHu0xD179thJhPpNnDjRtOWzzz4TSPirr74yxEoeRDxw4IBUq1bN1K948eJCvrNnz5YBAwaYuh4\/fpxoKnGCgBJfnHSUG6uJVvX111\/Lxo0bjdaFNvT888\/LwoULpXHjxpIrVy4jDRo0kGXLlsmCBQtMONrahRdeaLQn2gUZQTxXXHGFGS4PHTpUVq5cyaugpV69emYecNKkSWb4C6Hu2rVL1q9fL2hxlmXJpZdeKhUrVvTkXbduXaE+F110kfTo0cPEHTRokIwfP1527txp4qHJor1aliU5cuQwdT\/nnHME8j1y5Iggov\/FDQJKfHHTVe6uKCRQp04dQwCHDx82hGfXmCEuJGg\/Z72yOAHhQKAIZNWrV6+s0c54RlND20Prs1+OGjVKhg8fLpDoo48+ajRRhtLM30FYxOOeoTn33sLCTOfOnQVtsFWrVtKxY0fv15nu\/bUnU0R9cCUCSnwu6JZEqALkwpA1d+7cZm6PVVgID5k+fbrRtpjb456wffv2CUNb2s5cHENLSAzSZC6NIS\/vfAnx3nrrLbnyyivNvKIdjyHorbfeKiy8QGSQKBomw1PqRz23bNkiaKp2Gvv6yy+\/GM2PuT00Obt+9nu9Jg4CSnyJ05eOt2TVqlVyyy23mHm9GjVqmCFuWlqatGzZ0gwBGfoiJUqUkJtvvlmaNm1qwhly3n\/\/\/VKsWDFTZ4bCkCJXhp2QDvfmpdcfFkPs8sgXQmMhwyuKKbt3795mKMocHHOQ7733nlnMgEyrn1r0wPSlSJEi3snMPXOD+fPnFzTXu+66y5Dq6tWrzdDXRNA\/CYOAEl\/CdKWzDWGOjLk5yATzEhY6WBRAs2IYyNCV9wj3hEFU6enpZr7v448\/ljfffFMgON499NBDQlzEju\/dIuJ9++23ZnGB8lasWCHMJ+bNm1cQhrfUiXk45hjR7iC4IUOGSJ8+fQSNElIm\/9dee83M0zFE7tSpkyCURf1YgSZv6gc5Q56Q7IQJEzyaJSvF1Ic0lEnZ1IFnlfhAQIkvPvop+WoZ4RZj\/AwR1qpVS5i\/w6TF1jgjXJRmFwcIKPHFQSdpFcNH4KqrrpJPP\/3UDMfRGFn9DT9XzSFeEVDii9ee03orAopAyAgo8YUMnSZUBJxGQMuLFAJKfJFCUvNRBBSBuEFAiS9uukorqggoApFCQIkvUkhqPoqAIhALBEIq03XEh381NrDjDQM\/bwj3hPEupFZqIkVAEVAEvBBwDfGxlQhj2Hbt2pmN4fhXmzx5siAYjEJ62F5NnTrVq\/p6qwgoAopA8Ai4hvjYp8lG9w8++EDwj8YmczxiIKVKlRII8cMPP\/Rscwq+qZpCEVAEFIHTCLiG+HAJxHYj3Afh141N6GwTwtqeK88QI3s6T1c9oL8aSRFQBBSBMxBwDfHhyBGvHPfee6\/xfMs+STzb4rmD\/ZUDBw4UhsNntEADFAFFQBEIEgHXEB9uinDxzbYihrv4S8MFefPmzY0rchxC4ictyPZpdEVAEVAEzkDANcSHNsdwFw8Z+HTjAJmcOXOaCjPE5Zk4JkD\/hIWAJlYEkh0B1xBfsneEtl8RUAScQ8BVxIfbcfy1cRjMDz\/8IFxZ3CCMd87BoiUpAopAIiPgGuLjsBecUeKB98YbbzROJrniP40w3hEnkTtD2xZDBLTopELANcR37rnnCj7TMFfJTnhHnKTqHW2sIqAIRAUB1xAfJ2ah1d15553GnMXeqsZ2tYYNG8rgwYOF07iigoJmqggoAkmFgGuIr1ChQsKhMLNmzRKIbsaMGeZ8BbaxsU+XXuFsBq4qioAi4AQCiVuGa4jPhpita+zSgAjtME7EIox3dpheFQFFwJ0IHN+9WX6d\/y\/ZPfQ+d1bwVK1cR3wXX3yxHD16VDi5inNRkXfffVcwaA5lcQPbv7Fjxwo7QDgta8yYMXLixIlTTT\/9f+nSpUVFMdBvILxvoGb5ktI5tbC8Vf9C+bFLKVkzqINkTHvvrL+t079C5\/+6jvgwVO7bt6+wha1169bmnNSffvpJXnzxRcG4OViIOIuVIwIZQjN8ZhvcsmXLMmWDqYwTQqFOlGOXwYHY9r0TVy1vo0QSZ7d\/L+uXzJXVo56XjHaXybiKW+Wp1DxyZ4s7pVDX0VJzyjG5feZhv3jQvlhJGMQX2SozlGVbGrmygyMtLU04DWvhwoXSp08fyZ8\/P6\/EjmMeAvgDyeH8gPRojKwYL1++PICUkY8CMUQ+V9854tXG99vIv9HyIoupG78Xexi7vVc9o9ntm5QuuQqlGLIr+eYmc72wbvvIAhGF3FxDfGxPmzZtmnTq1ElwSoD\/Pbu93C9YsMC8I44dHsh1w4YN8re\/\/c0TtUyZMrJ582bPs94oAr4Q6Ddrs3SduC6TNB26UhDCee8rbSKFJwrZefeJa4iPFdsOHTrIc889J9OnTxfOPbXnXW666SZB88NpAXG8GxDIPcNnX\/H2N39HCnSf54hUfWOzI+U41Z5ELeeqXl8K8u6irbLguz2yfvt+I2s27ZL85x43QtictTs8\/Un816evkU2bNkVMmOKJZH5nywsLCk+c\/3wpG0d0k41P1jCa3e4Jz8mR8y8R68EJcuKJufK\/xs\/LnsvqhNxWftu+fpNOhLuG+OzGXnrppZKeni4MUe35kq+++krYuoZTUjteoNeUlBRh+5sdn\/sKFSrYj5Lvkwdk78B6kRK\/+ax4KMXve6fqoeX47++1va4Xb5n9WA1Bvni6rozpkGqEZwQsp3apInWuKCi95uyR28fvlG0n8gvD\/nCFkUq4eQSTvkb5EpJv2Vg5b8z9crLvDZJz1RS5sER5Kf3BSSk9YquUTpsspRq0iUjb+G17foQxuHEd8UUagxo1asjKlSsFUxiE+8qVK0e6GM0viRGoXTafDG1TQVY9W0NK5s8tTd9caYbHboeEISzCPB0rsQhmKOdfWVeK95on9pyd29sRSv0SnviuueYaqVOnjtxyyy3SqFEjM4QmLBSwNI0i4A+BkgVyy9SuVSStUSmZuGynVO69SDI27PeXxPF3EN2R\/843NnYQHQLZsSBhk13+Vj0l99\/rOl43JwtMeOKzLMuc15GRkSEMmTmwyLIsJzGOWVnM1zhZuK\/yolUHt5aX1iglk\/YX60UQyA5yw6AYomNF9vAp8svfspdHs4PsdlxwWbS6ynX5upL41q5dKzfffLPR1LC769ixo2DIfDb0IDY8umCs3LhxYyGfkydPij8D5rPlqe8VgVAQsLU\/hsD9Zm2KifbnTXYMZ4\/t3mzMTRjCIpBdomt2vvrOdcSHPR+7Nrp16yYsdLAQcdtttwnHTfLOV0NwXf\/GG28IR1EuXbpUOnfuLK+\/\/rosXrxYzmbA7CvPeA9nYtvJNmh5Z6LdJrWoWdCy5\/6iqf3Zmh0a3caWluyblG4qhEExRMdQliFtzkIpJjzrH6f7L2v5Tj67jvhYgMD8hOMlAQK385i24I6ed4RlJ5zMlpKSIpdffrl5jRurX375xZjBuMWA2VRM\/yQlAvbcn0f7i9DcnzfZMYyF7OLRoNjpj8J1xMfuCoyZd+zY4cFi3bp1wkFD\/raslStXTtjqljdvXjl27Jjg0QVHB9hCYRZgZ5ZMBsxunQOz+yLca7y1L+vcXzjth+DQ7LzJrvQHJz0rsWh2webvNJ7B1i+S8V1HfJAbCxC9evWSFStWmNXYf\/7zn\/Loo48KpGg3nhPZWK3FEPLxxx+3g2X9+vXSqlUr+e6778z+XkgUDdITIZsb8qDToy2QcLTL8M6fpno\/R\/tey9t0VoPeEwd2yGuN88mD1+Yzxs9X9frSZ5pM34uXQbEZxs4ZedqguP9Gj0FxuP3rZP\/xm6O8WInriA8gGKbiVGDRokUyc+ZMsxp79dVX88ojGDOzjQ1DSOb17EWMrl27Svfu3eWtt94SND6Gvxgt2wm5Z97QfuZKHsxvRFvQPKNdhuZfKiIGttHGsW\/rKmblt3ShvMKOnknrrUz1LpHXkr8d\/UnyTn9BzhlQ32NQnP+am81KrMeguFR8tDcrnvzm+O1lFueeXEN8zNG1bt1a8LyMoM01b95c8MDMHB\/viOMLGv51ZB\/viBEj5PrrrxfmBomrBsygoOJGBOyV37RGpcSe+\/vmo2EeG7uTw+8SzE4urNve7J5ggSKZV2Ij2YeuIb4CBQoIq7KTJk2Spk2bygMPPGA8MuOVmaEv5ikFCxb02XacEWC+0q5dOw95QpYlS5Y0ZjHJaMDM0McnYFF4oeUFDyqLE13Omy8zjj8tH62uKXs\/SJf5i1YZorNODWNtsgs+5+BTON1\/wdcwcilcQ3xoaAxNueJ6Ck2vePHigtx1113y9ddfmwUOX01HK2QRhI3WuLNCIE3yhAyT0YDZF1YaHlsE2DmRdXGi9BXljY3dwvaL5MHCA43dX2xrmdilu4b4bJg5SY0VXDQ4O2zXrl2yZ88ez\/DVDvd1ZT8utn8sgNhzfxg1Z+eB2VceiRDOqnH\/9AAAEABJREFUvIqT7dDyskcbrQ5jYlZhWZjgyjNmJ1lXYtMapZi5P+z+mPvLPsfohDrdf9FpRWC5uo74cD1\/7733SpcuXcxcHfN1bdq0MdvOGA6frVnMA\/br10+OHDliogbigdlE1D+KQIQQQKOD2NgiBtF5m5xgTOxNdtkVac\/92Su\/btzzm1294ynMdcQHeAxbWbGdMGGCcN4Gw9f69evzyq9gvzdkyBBhPpAhLpGXLVsmyWrA7PScTTKWZxMcJIcmB9FxZSjL95eV6FioIDwQ6XhdPo\/254THF6f7LxAMohXHdcSHxsaiBHtu27ZtazQ9jpskjHf+gJgzZ45ccMEF5nhKOx5DZsxI7OcEM2C2m5Xt1emhSyKVB3EhEBoCmeGnDu0NcrOFd6y80gH20NVbowuG6Mgjq9jaH3t+o+3xxen+y9pWJ59dR3wMZ1ndnTx5siBvv\/22NGnSxKO12eAwf4fJC4aQGDBjzoLNHw4NWCCx43H1Z8A8p+omsT\/iaF9P9ijtWFnRbktC59+xhOybM9LIr1u\/FQTvw8jxys3EajVAWHG1xfZIjFditKZICd+0nVf1godl2r1\/k0Lni6D9NXx1kU\/DZzuNm6\/8bvltxkpcR3yQFsNUDJQR9uxikPzHH3\/IwYMHPTjxjuEwhpAYMH\/77beCHV\/VqlWlevXqxjkBmiI7QTBathNy723A3GBFKWM6wL\/S0RZ+KNEuwzt\/Le9kaH2Lt2FbXl4kpZG0yYbwSnccLKVaPp7J2BhNKRrCSMU731pXX248QU\/tUkVWbDtivD17vw\/3nt9IuHkEmp7fLeXFSlxHfNkBsXXrVtm2bVt2rzxhDRo08Bxlh0cWyA+vLM2bN1cPzB6UkuMm0VuJx2e8PbdJLSYFus8TDj9K9DZHun2uIz7vISzqMMJcH16TOSIyWABIx5A4GQ2Y+dc3WLzCia\/lhYNecGmZ+8P0Be3vx31HDAGG6\/LK6f4LrsWRje064rvkkktk6tSp5oAg1GGEE9awy2O4G0jzGQazIszVspLXA3MgWGmc+EbA1v7SvLa9ZWzYH9+NcqD2riI+zs\/Fq8ozzzwjnH2L9ocwf8cCBv71\/GGCl2Z2eWC+cuuttya9B2Ymt\/3hFel3Wl6kEQ08vzQvw2cWP87Q\/gLIyun+C6BKUYviKuLD5m7kyJHyzTffyJtvvimDBw82gi0fc3is+PpC4vjx4zJgwAC54447zMJGp06d5F\/\/+pcsX75cmOvD28uMGTMEV\/aU4ysfDVcE4hUBhr\/Rcngar5j4qreriA\/D5T59+sirr75qnIribt6W\/\/u\/\/xN86\/lqCNre3r17BUNny7KEMzvQHPHphwbI\/CD+\/FJTUw0Z+sonkcKdnrPR8tzx9YSq\/Tndf7FEyzXEh3EypMeVLWcQGO6pbDmbAfOBAweM5+Wnn35aKlWqJPfcc4\/s379fktmAWfS\/pEUgO+0vacHIpuGniS+bF04HYbuHhoYbKbyq4F3FWwgjjl0v5v5YrWXVl\/k\/HBswP4g7Kzy5sBL81FNPGTL0Z8BMfuTB\/Ea0xdsgNdplkT9b\/bg6JVrepogaFUfie2lV7qTH8BnTF3+Gz072H785fnuxEtcQnzcAkBq7N9DebEEL9GfAjK++atWqiW2czPA2R44cUqxYMbNCbOef1YCZcFaOUfOjLVkNUrW88LwHK56B4Zed4fOkLB6f+RadxJPfHL+9WInriI+T1Lp16yYcJVmrVi2xBbdSuXLl8olTSkqKHD16VL7\/\/nsTB998zOmhFeKminwR7itXrmziJPofPmYn26jlOYl28GXVLpvPOD3A8BmPzxg+\/7j3tBcjcnO6\/ygzVuI64oPwcD6KWypMUmxh4YPtZ76AguQY8j777LOGLDlz48knnzTb1yC\/4A2YfZWk4YpA\/CLA3B+LH1O7VBEMn3F51W\/W5vhtUIg1dx3xYbLCXB4aW7Bt4pCiTz\/9VDB4Zk6Qf8EsK3kNmJnbCxbDcOJreeGg52xaW\/tL8zJ8npyxztlKxLA01xHfvn37jB0eq7gMbwNd1QXDefPmmfM2atasKRgyY+KSzB6YwURFEfCHQJqX4XPHj3ZKsmh\/riM+tL2PP\/7YLEgsXbpU7JVdNDje+epEbPhGjx4tY8aMMRofnllYIElmD8xovL7wCjHcbzItzy88rn3J8DfZDJ9dR3ys3LKCa6\/m2tf09HRhuR0vy76+IBY3sN3jPa7nmfdjlwYrvMlowAwOKopAoAh4a39N31yZ0Nqf64iPlVsIDLs8zsRluItbKhY98LfXu3fvbPuRuUGGx+zwwMvyxIkThbM6ktmAWefcsv1UQg50Gs+QKxpiQtqXnfYXYnauTuY64oPg0PrYd4tHlubNm8trr70mv\/\/+u3Tu3FkY0oIotn6s1mIIyWoudkFjx44V9uNiq0cYDkrZw5usBsyRMIDlxxCoJGt5geITbLxY4tkqCMPnYNtFfH63\/I5jJa4jPoBgmArRcY+cOHFC8NwCifGM4HJqwYIFxvkoBAfxFSlSRNj5YVmWYMyMN5eiRYua+ULSIJCibeTMM0Ja5qeiLU4aiNIWTqjj6pRoeYEZFAfaH7H+XgI1fA60Pd7x+M3x24uVuI74GLLiPJRhK3Z4zPHdd999hsiY4+N9dmCVKFFC2Oe7a9cu8xrnBCyGoBVitIzxMsJ9shgwGyD0jyIQJgK26Ysvw+cws49JctcRHyjcf\/\/9wrAVRwXs3Ojfv7+0atVKGPpioEycrFKuXDlp3769MWPBnGXcuHHCWR3Vq1cXyM8NBsycHJe13tF8ZkgRzfyz5q3lZUUkvGc3fS\/M\/bH4EbDhc3hNj3pqVxIfxsvDhw+X+fPnG9OUd955x5AaaLD4wTWrWJZliBHzl6+++kref\/99cyCMZZ3dgJn5BieEOjtRjl0GPxz73omrlldaIomzG7+XdjddIwdHtZXc3049teq7Kaz20r5YieuIj7m8wYMHCyuzLGRcfvnlkidPHunQoYMwdI00UMw1qGw0c6WKg+IQ6Dewffogs+830Pi+4kX69xxofq4jvt9++02wv2vRooVgzsI2tIceeshof8zRBdowjacIKALRRYDhb3RLiF7usSE+P+3BEQGruixSsHK7ZcsWOeecc+TQoUPGY4ufpPpKEVAEFIGAEHAd8dnaHt5VrrjiChk1apTUrVtXIEFfK7oBtVQjKQKKgCLwJwKuIz7qVa9ePcE2j4niKVOmCPZ6PXv29HvmBulUFAFFQBEIBAHXEN\/u3bsF2z3bG4t9ZaWwWbNmZvsZcQJpVHzG0VorAoqAUwi4hvhoMFvVcufObUxX8LIyefJksQVPKzrUBSUVRUARCBeBmBMfZMe+W\/zmjR8\/XoYOHSrsUUT7Y7\/tqlWr5OKLLzamLCxyhNtgTa8IKAKKQEyIj723eEpmGMtB4Xfeeacg7K7A5XzVqlWN4XJ6erpxSsoODkxaIEntsqRCQBurCEQFAceJDwPlF154QXA79cknn4i3s9HFixcLTkjRAB955BHxdkoQldZrpoqAIpCUCDhOfNjpsd\/23nvvNYbK3qijCe7YsUPwv4cD0RdffNEcFjR37lxhju+iiy7yjn7Gve16Hsej7NPF4JkhNPt+8etXu3Zt46GZcs5IrAGKgCKQNAg4TnzM07HflhVa5vHs1VucEeAuqnHjxoLz0BEjRhiTFjypHDhwwHhe8UdYkCV7eidMmGCGx2x14zmZXc8nzVecZA3V5oaPgOPEZ1cZV\/BoZnhJ5jyNYcOGySWXXCKs6q5du9Z4WWHezxbm+Ni7a6fPeoUs2ctbrFgxsSxL0O5wdoDDAsqhPIyjU1NTZfny5VmT67MioAgkEQIxIz5OU9u2bZvcc889Urx4calYsaLMnj1bGjVqJB9++KHcfPPNnoOG8LgCOUJsvvrmsssuE4bJCJohRs+Q4fr16wWHjnY6nB9s3rzZftSrIpA0CPw6\/1+SVfZNShdfsnvofRJtiRX4MSM+NDt7T67deIa\/LHrgfRmxwwO54t21bdu2xgi6evXqsnPnTilfvrzkyJFD\/LmeZ3eISmTdKSmekcezc2pheav+hUY+vvl8Qb5qlks2trQClqwktmZQB1k3vrdP+f6zcZIx7b2oSiC\/7YDiBBkpZsTHsBMPLGh8uCxnro+DghA0vEqVKgXVFFaAGd7ipZmFETTGnDlzStmyZQNyPe\/LbU4kw2lQJPM7W16ff\/65o+6mtLzQ3VqtHvW8LHmsjpGMdpfJnKqbMslTqXnk1vIFpdkN1aT2ba2NVGj7rJT+4GTIUnPKMfGWYoPXZ3rm3e0zD0u05LH1xfhJxERiRny0FucDo0ePlieeeEJwPDpp0iTBlg9X8x07diRKwMIQl8OIGD5zYBGOSLERxBsz7uZZ4UW4Z8Ek4IwjGBFiiGB2Z80KLfiskSIYQcsLDMwj\/51vhpfbe9XzaGtoY4dPhZNDrkIphsys\/hvNFXIr+eYmQYr3mieFuo42kr9VT6JHTJzuv4hVPISMYkp8I0eOFHZnsLCBcXK3bt3MKWloasG2hTM3mjRpYlzUQ6g8o\/VxfodbXM8H2yaNnxgIHN+92cytQW4MTSE85togOEgMYkMgNp6RxGi5e1sRM+JjhXb16tXSt29fyZcvn3ETP2TIEMGomXcS5H+WZcndd98trOIuWbJEevToIZjNWJYl7dq1k4yMDPOOOJZlBZl7fEbXMzAi22\/B4gm5QXI\/dillNDxqA6l5k9yFddsT7AoJtn2uqHSIlYgZ8bHyygLHBRdc4Kk6836QFfN1nkC9CRkBp4cuyV5eVs2O1VK0OrcSXdYPy+n+y1q+k88xI76CBQsaxwNjxowxnpUxO+nTp4+x5VMvLE5+AlpWuAhAeJDc9p71Mml29tA13Pw1feQRiBnxWZYlDz\/8sDlUCA2P\/bvY87HQEcocX+ShSfQctX3hIADZeQ9luYfoEIazbhrChtPORE3rOPExxMVeD0cErLKyAMHWsnHjxpndGqzIEidRAXeyXU7P2SRDeRAe2p09b+c9lHWyb6NRltP9F402BJqn48THwgXbz9iKxios9nvY8yENGzYUjpY8dOhQoPXXeIqAIwhAdif73iAQHtpd\/pa9jHkJ2p0jFdBCIoqA48THtjO2n82aNUsguhkzZghGx8hHH31kGscCh7nRP2Eh4PRkdQKU58EbcsP8BKLDBIXn\/A3+YezqGM5G2obOU3AMb5zuvxg2VRwnPruxDHMZ1kKEdliRIkXMQgfv7LBwrydPnhR1SxUuismRHqLD\/ASiQ8PDoBiSs1dlE5HskqNnz2xlzIgPd\/JHjx41Oza2b98uyLvvvivs0cWs5cyqhhaSzG6pnJ6ziZfy0N68SQ6iQ7OD6JizY\/gK4SHeX53T7fMu24n7RG+fN4YxIz4cB2C8zIpu69atpWXLluasDZyP4qzUu5Lh3LNvN1ndUjk9dIlleZAZWhqEhqC5IRAaxOYtxOGzIr8AABAASURBVDu2e7PYJGdrdBAdpOdrRdbp9oXz3YeS1lf7QsnL7WkcJz6Gt3hOARg8KqelpRn3UwsXLhTs+PCbxzs7DvfhCK6p\/LmleiJld9Rd7\/BDRE5OeiKksvgBJ4pARP7Em6CCuYfMID+0NpvUIDZIDGLzFgjO3vPKe9H\/kg4Bx4kPG71p06ZJp06dhK1lnMFho879ggULzDvi2OHhXtEufeVR9Nzjxu3Or1u\/lWjL0Z0bQypj07GLJR5l4d48klU+Oe9G8ScjUp6R9AJpRqqVmCuBSuPCYwV5oMwEuffCl+Txc7vIwEselv2p7YQhXDwKpw3GY70DqTOuy3z9Jp0Id5z4WLHt0KGDPPfcczJ9+nSpV6+eAAJy0003CZofxszEiQQAKSkpft1S4RoHtzulX14k0ZbcD38YUhm1+nwioUjxfwwKKV0oZZEma3ktX\/+3ZJWHB40Uf\/LygN4yZvjLRvYOrCf+ZMVDKZ73a3tdL\/fUKCENriom5Yrnk32\/55QZG45I1Tc2Z5Lbx++UR6bvl0nrLbM\/nOFdoML3GGjcSMRjpBKJfALNw8n24VKN8mIlYRNfqBW\/9NJLJT09XZiDAwQEBwNPP\/20FC5cONRsz0hXo0YNwRUVK8UI97FyS3VG5TQgogikNUoRZGibCjK1axVZ9WwNDzHaBNomtZjUKptfFm7YJwW6zzNSufciaTp0ZUTropm5G4GYEZ9TsCSzWyr+pXcKZ8qJh\/IgRgRi9CZD6u9NhBOX7SQokzjdvkyFO\/CQ6O3zhjDhic+yLGnXLjndUnl3tN77RsCbCKd2qSJohV0nrhM0wX6zNvtOqG\/iFoGEJ76Y9IxLCmWS2cmqJEJ5tcvmM8NltEEIsN+sTYYAf9x7xCySOImn02U53X9Ot8+7vJgSH84KnnzySXNAEPN7OCJlZde7gnofOgJOD10SrTw0QQiwZP7chvwW7zk\/9M6Ig5RO918sIYkZ8WG43K9fP+OWCgBYwTp27JgMGDBAeEeYiiLgBgSYD0xrVEoY\/mY39+eGOmodgkMgZsSHl5Y\/\/vhDbrzxRnMEJN6YMWFh5ZV3wTVDY2eHgNNDF\/\/lZVfD8MKcLC\/t1IpxrwYFDfmFV2v3pnYSz1ijEDPiY1vaOeecI7\/99psHA4a+kB5Gzp5AvVEEXILAbRXyCosfrP5mbNjvklppNUJBIGbEhyOC+vXrS9euXQW385y2ho8+ztVV1\/OhdOWZaZyes0mG8k4vfpSSpm+ulEQjP6f778wv1rmQmBEfTeQM3X\/\/+98yaNAgc0Ia94TxLhSZN2+e4NgUpwTdu3cXhs3qlioUJDWNPwQY9tYuk0\/++d46f9ECeadxYoRATIkPourZs6cULVrUeF7GDf2oUaNCgmLr1q2CC\/sJEybI4sWL5fLLLzfP6pYqJDhDSuT0HFEsyxvSpoL8uPdIQs35OY1nSB9ZhBLFjPgOHDgg77\/\/vhnqbtmyxSxyzJkzRyAq5vmCbd+GDRsEp6bFihUTy7Kkdu3asm7dOnOWLhogXl8YXqempsry5cuDzV7jKwKZEChZILfZDscqb4bO92XCJh4eYkZ8OCHNmzev4HV56dKlAjmx4HHuueeGZM5y2WWXyY4dO4xwWBFeXiBD5g8xlbE7o0yZMrJ582b7MaGvTs\/ZJGN5bVKLJsyQ1+n+8\/XjcyI8ZsSHB2a8LT\/yyCOyaNEiwYsKZ24QhmZ2tsZzSludOnWMZxcWRui0tm3bGmPo6tWrC\/78ypcvb0xl\/Lmlohw8w6DmR1sS2c1QtLFza\/6PV89t\/qF+6r2VEuk6JvL3wm+O316sJGbEBxn17t1b2rRpI\/3795eLLrrIYIBLKjQ\/8+DnDx5c0Orw6vLKK6+Yj4\/hLYcW4fGF+ULMYsqWLSs\/\/PCDJyfuK1So4HnmhjwgzmgLmme0y\/DOn7Z5P0f7PlnLwyXW8KX7ZduJ\/EG7uvLXJ4n8vfCb43uJlcSM+GgwBHf11VfLJZdcIkeOHJHrr7\/ekCD2fLwPRhjmdu7cWbZt22YOLGL+sEGDBlKzZk11SxUMkBo3JAR0lTck2GKWKGbEx7Y0NL7bbrtNIKg77rjDnLvBCm++fPl8AuLrRYkSJaRJkybSqlUrqVu3rvCM1qduqXwhFvlwtJfI5+o7RzeVx7Y2ahrPfv2cxhO8YiUxI759+\/YJq7ezZs2Sxx57TN566y0ZP3680fwsywoaD8uyjC0gzg5wad+jRw\/B27NlWeqWKmg0NUEoCAxpXUEyftgv6soqFPScTRMz4vNuJnNxu3btErwy79mzxxCi93u9Dw0BJttDSxlaqmQvj10deH\/GlVU8mrg43X+hfWWRSRUz4itYsKAxXOaISVTskSNHmh0cnK\/L3F9kmpckuWgzXYMA5i24skrELW2uATkCFYkZ8VmWJfjie+qpp4wNHwsT2PXhlioQc5YItD3hs+AfFCcbqeX9hXZao9P7eeNp2Ot0\/\/2FlvN3MSM+mopbqu+++05mzpwpOCAtV66ccVjA2bu8V1EE4hWBtEYpYg97cWEfr+1I1HrHjPggNzyzDBw40BwpybGSCLs4cEiaqIA72S6n52zcVV7kkQ62fQx7Vz1bQ\/DgjCuryNcosjkG277Ilu5sbjEjPjynMLRlbu+ll14SW9LS0jzGzM5CoaUpApFHgD29mLqknRr6Qn4Iw994XPyIPDqxyzFmxMc8HosYoRgrAxd2gM8884ysWbOGRyN4e8EtFUbLd911l7BQksxuqZyes9HyzGeY7Z+0U0NfFj1wZPrj3sPGnx9DYDfZ\/Tndf9kC5VCg48R38OBB4ayNF198USC9Zs2ayYMPPigcJI7wjjj+2o8XF46MxAbQjrd3714ZPXq0jBkzxgydGzZsKG+88Ybx9jJ79mwh7owZM2Tu3LnmEHM7nV4VAScRsE1eIME2qcVM0WiBCESIoBEi5mVk\/2hufyLgOPHlyZNH7r\/\/funWrZuwcwMSgwR5RnhHnD\/rl+2FLW54a8Yji3cEPL7s37\/fBLEFDq2Sfbt4fnGDWyrOFzGVc+iP03M2Wl5wHZt2SgtkGAwJIhAhsnDDPsEWsOobmwVCRCBEBA2RQ48gRiS4Ev3Hdrr\/\/Ncmum8dJz7O2YCgnn\/+eYG8WNywLEtwOoDgU484\/ppdpUoVuemmmwRbQDse7upbt24tuK7H9dTEiRONA4QNGzYIm73teLzL6pYKTxFOCHVwohy7DIjWvnfiquWVNt6CQsV6eOf6gqx9tYXk++SBTHJwVFv5+atJsnTep\/LBjAXy6viZMuDjZR5ihByDlYKdPhBvSe23PNMz7wq3GybRkv\/V7sFPIibiOPGxmouGV6NGDRk7dqxZyGBhgzk7XwjgdoqPCTdUuKPKLh7eHsiP4SweWEhje23BE0x2aQgjncpGUQzcj8H26YNk5+Rn5eexnYzsGdbSOENFWwxEMK\/xliduT5WzSctb6kg0hd9gLMSb+Bwpn9VchrItWrQQfPD94x\/\/MGdj+JvXg8D4YS5YsMBohtlVlPc4NS1ZsqTxwFytWjX55ZdfzO4QiNBOw31Wt1T2O70qAomMQJvUouItDLXPJt5EGY37WOHtOPHR0Bw5MhfLEZP+ND7SnE3wxrJ7925hzy9xV6xYYVzRoyWuXLnSkCuky33lypWJ4gph5RmfhAwTP\/vsM1fUSSvhXgQ4ToEpHawW8F3JCMq9tXVvzTIzkHvredaaseujffv2wgeBOcu4ceOEk9bwxgz5cXpbo0aNpF69eoKrqrNm6FAEdq106tRJ8D6dkZFhCNqhorWYOESAHU4Y\/nOoFvPkmITFYTNiXuWYEB8rrZUqVTITwRATp6JxPds8njdaGD8PHz5cKlasaIItyxJ8+33xxRfmgCEckZYqVcoMezF9gVRwWXX33XebMJMogD\/RjoKWSj1xxc8UAO66ol2m5h+\/CDBqCXd0FL+tj1zNHSc+Vm6Zq2NOLjvhHXEi10T35sQHzMFIuXPnNpVkyM\/pc+ZB\/ygC2SDAORyvv\/66NG\/eXBjVYKCfTTQNOgsCOc7yXl+HiABzjAxF7OTsP8Y4m+MtORsE+0XMdrxPlcPuEBtFO41ekweBQL4XSI7jGTg\/GtJbtWqVcJ508qAUuZYq8UUOS5MTjlQxz2EeBg3OBJ76w1we\/1oz3H7vvfeEPcoYjGJ\/+P333xvnq8S3D106lcS9\/2vNIoZAsN\/LN998Y6ZqMNFiuseygvdWHrHKx3FGSnwR7jy0OExpbr\/99kw5M4Rv3LixcYePp+nixYsLLrlYcMHtfsuWLYV5Tj7mTAn1IaERCPZ74R9GjPSxAsB0y9s4P6GBinDjlPgiDChb49hVUrVqVU\/OrMSxaGHvNLEsyyzsbNmyxbjbRwP8\/PPPzW4UTyK9SQoEgv1eGOpi9sQiHts7LUs1vlA+FCW+UFALIQ3\/sjOfF0JSTRIXCES2kvq9RBbPrLkp8WVFJArPmKkwhMVYmeyZpGZ+74orruBRRRHIhIB+L5ngiMqDEl9UYM2cqWVZxnCaeT5Wd7dt22Z2mJQtWzZzRH1SBE4hYFn6vZyCIar\/K\/FFFd6\/Mudwc+ZzMGVhd8kDDzxgDj3\/K4beKQJ\/IZAg38tfDXLZnRJflDqkQYMGgnMFO\/tcuXIJbvWXLFkiTEyzdc5+p1dFQL8XZ7+BHM4Wp6UpAoqAIhB7BJT4Yt8HWgNFQBFwGIFYEp\/DTdXiFAFFQBE4jYAS32kc9K8ioAgkEQJKfEnU2dpURUAROI2AEt9pHFzzVyuiCCgC0UdAiS\/6GGsJioAi4DIElPhc1iFaHUVAEYg+Akp80cdYSwgXAU2vCEQYASW+CAOa6Nk9\/vjjcu2118oNN9zgkT59+sR9s9lRM3r0aMGFGFsK8ZDt3SjaPWzYMO8gzz0OJwYPHizsw\/YE6o2rEVDic3X3uLNyeJhm250tzzzzjKeieJ7hHBFPQBzcQHaffPJJyP4QOSwKl2McmhUHzdUqnkJAie8UCPp\/+AigDXXp0sWcdLdo0SJZu3atNGvWTHDKwDmwaEWUMm\/ePOH4TzRGjv\/s1auX\/Pzzz+ZYUK7EQdtCw4JEOVuCuOTDObKQFO\/Jk9PzrrvuOunQoYM5lhONq3\/\/\/kIYjmDxbA2hPfroo8LBTuSNZoZwbwvnXXDeCZ6x7TBfV7Rb6oNUqVJFqMOhQ4cEB6EcEXr06FFfSSMcrtmFg4ASXzjoJWnajh07Gg\/SpUuXlquvvlrWrFljkNi7d69MnDhR8DM4ZMgQGTp0qGRkZAjEM3DgQNmwYYNwUM6kSZMEF10QDURmEmfzh7Tr1q0TvFN\/+eWXgp+6KVOmmJgcy4nmSRy0reXLlwvnmnD4zvz58wVyZPjK4U07d+6UHTt2GHLkgB7ctptM\/vxD\/CuvvPLPp9MX7zbSTkiNN2i3aLq0MyUlRThbhXrRFtq\/a9cuoqm4HAElPpd3kBurN2LECLGPBl29erVUrFjRVLNu3bqkWzY9AAADfklEQVSC5rR582bjgQatCNJo06aNfP3110YLLFKkiHG3b1mW4JEEB60mcTZ\/IC4IBiItU6aMIdKVK1eamGhbnDeB1xvKZ3jNQU6Q2vnnny+4AGPODu2yXLlysmrVKlm\/fr1QXlY\/iJxVS7jJ+M8\/3m2krXfcccefb0QOHz4s6enpcsstt5j5Tl5AvhwTqseDgob7RYnP\/X0UdzVkuMcxmpAdpMEpclOnTjWkw\/DVbtDvv\/\/ud0GAIWSPHj08JIv21+vU0NhOn\/XKUJfTx+zw\/fv3G5KqX7++0TAZZteoUUMgRjtOsFfq\/+6770qhQoXkvvvuMyeeBZuHxo8aAgFnrMQXMFQaMVAESpUqJQcPHhQID6L49NNP5dVXXzWOVxmK4oGacIaMkGTOnDnN6XNHjhwRwplzoyxWj9Hi0MjQsnr27CkMeXmXnUC2DKEhwJ9++kkeeugh4ZCnypUrm3lE5h7RQrOmLVq0qFnNzRqe3TPDadrDEBdt046DxkndL774YjtIry5GQInPxZ0Tr1VjvuuJJ54QFihq1aolnCLXqVMnYciJqQjDxjp16ggEyBCTYSnPHLHJCXXMBUKGDIUhM64MowsUKGCGx75wufXWW40GxkJDq1atpEWLFsJwGDKqVKmSmZfkOWt6ymCeEuLK+s77GUJF2\/vxxx+FxRVIlCvzjcwj0hY0Qe80eu9OBJT43Nkvrq0VXqUhoqwVhNgQO5y5NRYl0NgmTJggnCNsWZZw3vDSpUvNHCBDRcjGsizhqETm9GbPni1vv\/22vPzyy0YLRGsjHGHYi5ZF+dTDLotyCYN4+vbtK5iVUC5lWZZlVnQhpqZNmxpitNPZV1aA0QzREsmD+pKf\/Z4r5VEX3jHPiLaKQOqQHW3l3ORwhtGUo+IMAslEfM4gqqW4CgFMZCBANMhq1aplWzfI7p577hHmACHibCP5CcRU57fffhO0Vj\/R9JWLEMjhorpoVZIMAVZjWR2NZrMLFy4s06ZNMxqkP22MVWJs8izLCro6zGkyrEcbDTqxJogJAkp8MYFdC1UEFIFYIqDEF0v0XVC2VkERSEYElPiSsde1zYpAkiOgxJfkH4A2XxFIRgSU+JKx17XN\/hHQtwmPgBJfwnexNlARUASyIqDElxURfVYEFIGER0CJL+G7WBuoCEQCgcTKQ4kvsfpTW6MIKAIBIPD\/AAAA\/\/9Z69F5AAAABklEQVQDAF14yMBNoBoPAAAAAElFTkSuQmCC","height":191,"width":318}}
%---
%[output:1e676430]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:546837fb]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:40df2e29]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:711af29b]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:2919b5dd]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAT4AAAC\/CAYAAABno7D5AAAQAElEQVR4AezdC7yvU50\/8PXzb0o1U6fpxnQ7roXKEYWoIZdUjBOKJunoRi7RFBFChKTpIsV0QaXSTSXkOuRSiJJLiAqNSSUmJVPRf78fvmfWefbzu+7f3vu3917ndT77Wc9a37XWd33Xej6\/dXvWs9Tfyr9igWKBYoE5ZoGlUvlXLFAsUCwwxyxQiG+OVXgpbrFAsUBKhfgaWkHxKhYoFpjdFijEN7vrt5SuWKBYoMEChfgajFK8igWKBWa3BQrxze76HV7pSkrFArPIAoX4ZlFllqIUCxQL9GaBQny92alIFQsUC8wiCxTim0WVWYoy1RYo+c1UCxTim6k1V\/QuFigWGNgChfgGNl2JWCxQLDBTLTDSxPfOd74zLb\/88m3xr\/\/6r+kPf\/jDUG1\/7LHHVvn98z\/\/c\/r1r389obSvvvrqtPrqq1fpKUeTvvUynnPOORPKcxiRh2mDXvSZjPzYkc1zDKNOeylPXu\/06CUOmbADnbUb6fDPEe2lqS3lck1u6UlX+v3o1ZRW3c9zSKfll18+feYzn1mi3cuvjmHnX9en2\/1IE1835b\/\/\/e+nV7ziFRMmqG75DCv8v\/7rv9K99967ODmN5fbbb198P8qOyXxohl1u5PCWt7xlXLK33XZbWmeddRKCGRc4zR7awne\/+93FWtxzzz3p4osvXnw\/6o6f\/\/zn6dprr01Pe9rT0gorrNBVXfUzSD2oWySKZNmsa0ZtBGYE8WmsP\/7xj9PPfvazxfiP\/\/iPqkga89e\/\/vXKPep\/6KocoWc0lqc+9anpH\/7hH8J72q8777xzZecLLrggPelJT5p2ffpRAEGfffbZVZS99967Kod240fSQyngE5\/4RCLHPRl4znOek6666qoq74033rinLOgHuTAinMjDnac12e4TTzwxIevXvOY16R\/\/8R8XZ+c5Zf+A9u95JvDFL35x2jotM4L4GKmOJz\/5yW3JIn4V\/DKA7n1TQ+cnjAyIV88n7nXNyQT6HTZttdVWVVI33XRTdfXnjjvuqBrLKqus4nYc6nnKu0nHuhwZv6bkQ8+8rOTJCA+QDwW4+Udc91tuuWWlKxm\/1vGLG+nEvXAQRxrsK29+wM1PGIjPvwn0JBMIfZpkw08vyQOI5MLmwhD4AQccwFmVg+2rm4f+0CPycaUjXR8KXnyp6yRelDX0E0986ZBfHLmD4zvf+U4VSu8gBj0oP45VQJs\/pmPkKy960Ic70Cn\/brJ5OSK9ej1Tiw6XX3559Tyut956vNri7\/\/+79M\/\/dM\/NYbX82NDfoTloZzRyfEj8dznPjdF+ciRDz1dm3SVFkw18clzKIgGLrEVV1zRpZrvU9gwTuU59seD4MHVMMZuq\/8Mxk9Y5TH2R7wjjzxyzLXkfw3Ew5776r1tsskmPfccll122aph5L\/i0dhXXnnlPOnKTdd6ngLoqIzRE1COuhyZpnKID+TJcAfIyzPuJ+NK16myufrRu8jLofcVPQ9uYezInnV7aBd0zW1Cf7YTLyAe28X9IFcPNeIQ9\/nPf356wxvewFkRtHZe3fTwhx70yUXpm5chwvg3ySojGVflZwf3AYRTn15iZ\/ZebbXV0nLLLReijde8rE95ylPSox71qEqOjvX85M2PLpVQmz\/CyZHPRZp0jfAZQXwKgN2xeEAlK4ThTDTiz3\/+84ks\/+hiq5T4BY0hDuMfcsghxKo5CXE8EOJUntkfRo0GIi9ykSZDH3744RXhZlEancsss0yaN29eink+D5z5Pb\/wfqnySMIQJD+6y0++oV\/0BPoph7QC8owyf\/Ob36wIWZg85c2dw9A3l6PHF77wheTXO5fr5O5H14nYXI8jpg083NFe\/Hg16TcZbaYpn05+6hdxkNlss82Stq6O3LerE2F1KLd60lbUb6TRNKQURoasOOJKz8KENhA\/yk1ydKUzebLicCPsepvI60BdaM\/iS\/fDH\/5w1Yb01jyb0tBLpxNw84tn9YKxqZfwkw4dPPu96iqtwIwgvlC26RqV6sHiJsM4DMKtIvbdd9\/q4UZUfkEZjPGFG\/4YBnGLIy53IIamKirCpKmSyQQJcXeCoblf87vvvjsZZlnkQIJ+9aSXx3WPWFS+q\/s8XDmk0U858vjmYaLMfqH9Uufhk+HuR9eJ2Nz82gc\/+MFxRfDj5cEDvQsCk9VmpN0P8gcX6akbbUUavbYvskYgys8tDfXMra2zP3dAGBn3TW3gqKOOquYokU3IkQ1EHRmK09HzQfcI73alkzohpy1r08j39a9\/Pa8K3PzI1vWvBB7606uuD4lXlxlBfMHuiCCg16EEjKL3x3hIhV8MfbkB6ehtcUeFcTOqMO5APW7Iy4ceHhzwSyaOCpM3dzdIm3xOvuY7Hv3oRzdG9YDKKxB51oV7KUcehx5xj1TpEPeTfe1F14na3A9YtJP4scrLpb2wrXqbjDaT59XNjXzzYW6QjJ6fuNFeuLsh4oRcXs\/hF9c8rF0b0POOtufaNJzUlumIqEP3yMPVcxp14YrAPEfC9PL09qK+pSMPeQE3P7Ihw92EXnTN480I4ssVDjfjgXsN53e\/+x1nT+hmxJ4SGUAoGpv8QRL1xsoPDM08oNwxxM6HJPwjDe5Rx3TpGr0BD11uP0PIP\/3pTz2bbbL0RwR+VCmiB+SBh\/xHjq6GlGSmCn4YQgc\/HuxnaKxnFzrQiW5+zPTOwr\/TFcnGaAmpIc5O8r2E9aJrPZ0ZS3z1gjzhCU+o5tD41xtp\/Zc9CIjhhYkTqMcNfySrkWoAdehhhFynq6GAhoOoo8HUe5zi+xWMLRl+Mc2x8acrnblhkHKINxkwbDd8b5f2ILoOYnM\/GIjDCqDeVK5PfUinh9I0EhCHrfPe4CD6S6cTEEfMj3WSM5Q0pOwkI6zeduMeMTW1M3Ga8Nvf\/jbFtJEfXT8e5HKbuEeEYKqEbflNBJ4N6dWfL\/fxDNTTV8e96FqPN3Tiu\/\/++6uVztNOOy2dddZZ1WR+PdNh3DMQSEs3m\/HNW7j3y6nry61xWYBAGBqAye8gIOEmThmPWxxxuQPRI5MXhH+nByxk6lcrWOb0\/MJLy0PXrUFG41WO+kPSTznqugzrPghBmfwwSJc9ozG6h350rWw+FomNYMxZ\/e\/F5rk+9bqUFkjM8H7+\/PlpMtqM9HsBMkNqZKNX5SEPIB1h2m4vPaMYOorjx9M9dy\/tjFwTov0Ji7163BBzky9+8YurRQp+3ZC343gePZPc2lBeZ3pyfsQs\/ilPt7Q76VqPOzDx\/eUvf0mf\/exn06abbpq22GKL9J\/\/+Z\/pf\/7nf9Kb3vSmZGy+++67Jyy9wQYbpPe\/\/\/2JfD3zXu81Vg8OIwSiG85g0c3efvvtq5350hVOVjzx+b31rW9NJn\/90lvU4MfYehZkxeGXQ29Oo+QnnBxEBXlwpCe8G3TzPXAhh7Cb4vr1ROTkDHfll5eDP4grf+5u5SAzLLCDLSAacRCNtPnTlT3pwy9A16mwed4GwnZ0AvrRR88iSCWXF04ut3XeZoZta2SG1OgUZM8dCEJw74fEDwp3O0jLs6cMru7Jsjv7c\/cCoydtk6x2Lj3g5gfq3sjF80dPfk0Im4oPuW1jMcYzyc7i53XGzS\/kuAOeaWn5se2ma8TJrwMT35e\/\/OV08MEHJ79aN954Y9pzzz2T3tMll1ySXv3qV6czzjijIsYNN9wwffrTn07k84yH4faAaTwMJz3EYhU0iIpfIB8y8kNo\/LgDHgaI+7jq6tdlVbg5I+Qecr1c8waek0YeVzks9XtAw1+ZVHb46f1pfPKv60a2qRyR1iBXNtYA63Gb7Cj\/uk7iNcnSE4TnGNTmbKcNNOUvfXnlK5UhT2fhOaTBvuHHzS\/uXcWTJnc\/UHemO8RRpx5i7hz5D6AfEg95Hl5304M+4R9tlN3Dr9frQQcdtLgTIY5nTftzdX\/SSSclOmkT2ga\/fsCO6jjisK3nic7h51qX08mpy3TTtelHYyDiU2mGspj2sssuS9dcc031zqxfBL+KhpbPfOYz0\/rrr5+OOeaY9NKXvjSdeeaZ6Y9\/\/KOy9AyGiW5\/01UD13DrCTbFa6p8fnm6jA\/88odD+nVZryR1q3Dh5KQnfj0defFrkvMLTQdxQZlyv7zs0iYTIJt3+9vlwR\/Iixtp0su9\/OVJBkJOWMjyb8o\/\/JRf+chB+EsD5AXc9fzqsvW0pNcO9bjSB3k1xcnLRg6kUZflJywgXj+2jvS0WzaUTr3cTTLkNt544yQ\/bnGlEbJxjXAydXupB37ClCPiuEa8SFfa3GSBW1twdf\/DH\/6w2u4invg58nzINqGev\/hN8epydRnh3XRtsu9SMuwXJrFNZhvGPu5xj0t\/93d\/l7bddttqnI\/sWq3W4iSFGbbdcsstfRPf4kR6cBhKf\/KTn0zIWJd6o402qobff\/vb33qIPTNFzHuY\/1DeGHoqCf9YHGEPDZZ\/weAWYNNi68HtN2oxByK+pkIstdRSqdX6P8JrkplMP0PpD33oQ+m1r31tNcxee+2106677pouuuiiNFv\/+fUz1FA+wxDDJSQY8zuGBIYGwgsmZoFi64nZb9RiLzVqCg2ijz185hzMLe6xxx7JMNuE7gte8IL01a9+Nf31r38dJNkZEcdQo2kowc+wxgM7IwoyA5QcRVvrzRvKqe92w\/gZYNqJqDhQ3FlBfDfffHO1bcZ4v9V6sNf5yEc+Mq211lrpyiuvnLajb5pqxN6wQfGLX\/wi\/aJP\/OhHP0o\/6gHt0g1dm8pS\/IoFZqoFJkR8FisssYNel\/m03\/\/+9xXR8AuQm0wDmW982MMetsQ5YPJ71rOelehz5513um3ERz7ykRSwSj3ZiLwGuZ544ompX1gp6wXt0g09c7uEn21KIMz70Pzze2444YQT0gkPwd4vCDL+1a9+1VgvxbNYYDItMCHis2JreRt23HHH6pQSm0zd5yA3mYXw8FjZsf+o33yiR+Pab9y5Ks9WcN999yVgh6WXXjqFX9wL43fDDTdUB3NecMEF6eSTT07HHXdctRXq7W9\/e7WBeMMNN0zwohe9KMG6666bwMLM8573vATmL1ddddUEjvEyl1nQ\/rMMM8U22sp0YCDiM2lu78zRRx+degV58aa4kF2zQ5p28A+CBQsWpAUThOOqAvPmzauIBGG0G3rm\/tdff32aLMiHHoEw5Pz589P8h0Df8G93RYhANuKFzTbYYINqr5gfSb1z+xqdRs0e4vgxe+xjH5vg8Y9\/fPKWC3j75RnPeEYCi1hgQQd22mmn9J73vKfCxz\/+8WSLFdhfah6s4GfVNpRRsEO7NjMV\/gMRn\/kzv9AOJOwV5MWbjEIhVC+ce3Oknr4h8P\/7f\/+v7r343v5CK5+DwIOWwwMIHkh4xCMekeCuu+5aTFDnn39+On8MX\/rSl5JNyq6Bb3zjG9V5glZoEU83IO12WFzAMQciCbKxpzIv67ve9a4EVsTBZk84\/vjjk\/2YgQMPPDBBHtdCEj9XGsJDkAAAEABJREFUEMYe3pMFeSK7efPmjWnR\/B\/BAR3JIkDxNthgSVIURqaeVpQ\/hs6G0TFsN8zWqwT7S7VB4Ab+QC6G4q7SiPSk36x58Z3JFhiI+LyP+5vf\/GbcXF7M6TVdyYs3qLGQwctf\/vLqQyYrrbRS2m677ZK3RqSHZP73f\/83fe5zn1tiH9\/pp59e9RI8MOR6hcYO0fg9COCh8JCABwY8QB4m4Ab+QA48iOJDpCn9XvWhfwApbLbZZgkQDdICpAVIy1sdrgH+ZGDRokVVXPFBeiB9Px696hS2J4+MADlJa4Mx0gLDUzoixSBI98jRnjiy4kijCQgR6EauX1JsSpMf20PUhXpRRwF1pv5Afapb4Ab+QE6b8APmKp1IU\/ryGgS5bfuJ30+8XmQ7yfQb1km+nzIOS3Yg4rNYsM022ywepqyzzjpd3eTFG0Rxu8TtyTMMOvXUU6s5InNH\/BCq7St6kx50G3m9LrfGGmskk\/ri2GTdLl9DJ41Yg9a4gRv4gwYOHgyNGwZt4B5i8NAD8gGEgJgAUYHyeAfaNcCfDOQkJi2Q9nJdjv9uZ4t+4vUiW5fJyXHhwoXVO93KHaSIIJGi3iIgu3a6IkRQXnJNpIh8tU124ZYXW7sHcdul3+SPzCDqXjvQJrQzV21EewHtR1sCbuAP5BBlQDqRpvS15ab8u\/nV7d1JvhfZTjL9hnWS76TnZIUNRHzmW+yP0wsDFWe\/mF9x79bZNHzhhRdWr9fw1\/hUsnj9FsRKsddknv70p1eHHZjcfslLXpLe9773pV\/+8pfVJ\/gMZR944IHkn0M9\/\/znP1efcfTWiJ5gp56mh0ej0+DE7xUemoCHyAMFHi6kBEgKgrRyEuMP5GDRov\/riUkPpN+rPjNdDimCcm\/wUI+RLSdCimyifqXLfcsttyT3iFIe5gPVgWF91JF79QHyB\/VKL1AnIL1eoW2BdgaeF0QZQIQIERAkIEzgBmFA1rMUkJY0QR696jTX5QYiPkTzxCc+sfr0oE\/JaTgmoJ3WYi+dE0hMQHth2jcNkJJhqLc7+jW4rTC2q2h0ec\/Ni93uVbZ9fCbhX\/WqVyWvrelNmB\/TezCJa+jdLl\/xNGTpg0YOGrzGDx4GiIcjJzB+wshBOwKTRzsdin9nC8ybNy\/NG4P6QVigfnbYYYeklwjqWi8REFu7FI0UAFFYZY7empVmK878\/FjmBKle1TGob9AGgBuEkQO6gXYE9IZB2oD2DfQFRBeE6YoIESIgSGQZcA\/CyEEQpqu0pAnyaGez2eg\/EPHlhjBx76CCF77whdV3LfIwbgsPGqqeIVl+\/UAP7mMf+1h6xzvesUQ0J0NIT68OMbo6YshROYjQ\/J7Xubrt43O44qWXXpre9ra3VfD2B3iADKEBqYM5MDBfMUzoHY9Cev3o0YtsJ5l+w5rk9fi1AVA\/5nrB+XBI0Q8vODbJaARMbSzRkOJm7IoQAREgQEAuBx98cLX9xrc8nBFnEQq0aVMtP\/3pT6uVZ20FtB3QjiDa1mGHHZbMv4IjlwBRwhvf+MZq6I\/AERedDd9hkG1aY8VJyAyUB8mB8gQQIVIEBCnfgE7EK1\/5yuS4KNh\/\/\/2TuczARz\/60SQ98MxBtOGmuqr72W5Dx+nChInPUNQrYU5saVcI4Xpuru1k2vm3Wq1qY7KtDSGjwR966KHJXIh9XypX+KANRK\/QHMR0wTzkMPMeNL1+4vUi20mm37Am+Sa\/3I6mWcAoxCgAdt5552p1Wi8REA2Sgk49RW1PG\/dD6VVAsCPAVpnzzjsvfetb30qOmTKchlarVb27nuuTu+1RBD1C8KO95557Vse7ve51r0vat4NE4Stf+Up14IYeJuhhQr2XqSzR09TDhKZeZpOf8uVwCrMfgdgu5fg3vePAKaecUk09IU\/HYYEj5QEpvvvd706u4D16tvJDAvZ12mqkV53nOZXuCROfoS4DGyZcccUV43S\/7rrrkopbZZVVql\/FcQJ9eMQJLC972cuSTzOqeJtZ+0hiJEU9EMNUbND0+onXi2wnmX7DOskPYjtDZ9B2jUgAaZhTBKToHiECUiTfLi8kYXrFgw3Rq9Jb9EYLIA1bmfTAyLZLq1tZERfQHWmCKZY9x4jTVQ\/SswEIElkCN\/AHcqCcAWlJF+TRTsdO\/khTZ0Q5Qa8QcYZNkCV06n13Sn8YYRMmPvvksLz5u+222y7pHmN74PYrZB5Nd9metk5KO\/pdFzjgLZCQ1402bDjiiCOSX3pDWxtehRtOG4J+6lOfWmI7i6OE6GdOktyoQtmGqdug6fUTrxfZTjL9hnWSH6btIi0kh+wQIiAGZNhEih5gshG3fkWKgASQIgJEAjkpukeKYAhNvp5Ot\/tuNkJkAVMDSA6QZQARIkU47LDDeuppsg1IC2Eankc+3XServCBiM8qKaIJpc2dOP\/NsfPm12JD7k9+8pPEGIYDZEK+3dWwVaUH7MAnaw7PEda2w7z3ve9NflFyMjOv45htjSc\/lsqrcobAKkE6BcUCw7BAnRRjU3hOin7w9RQRQS+kqAeIFMGwUA8xiFG7RoiAPMkOoxz9puE5AmUyl+nZDsJ0DdI07NWzBD1NMJ\/pHqECWc9zvzoMS34g4qPw5ptvXvXuDHFN8D7mMY+p3gBwPDbyM+FrfsArbVZ4e1FYj9AxOwFpmkO0Uiu+1WMNg7\/7gF9cbr80JpJViglbPUE9Plfho4puQ5t+9R40vX7i9SLbSabfsE7y\/dpn2PK5bkGKyEFPEQHqDQUp6jXyQ4qAFMVpp5OeH6LT7qGpt8gPKZpbJNsurdw\/1zn3z92dZPoNM58ZpMk2SNOcaZ7fVLoHIj778axu2STsjLJ11lmn+uiQPXwWHhQg75G5HxR6d1aETFDrVd56663VWxt6ncIYT+OwZQbR+r6HIa6eoTlBPUHEPGj+JV6xwLAsgODAg48UASkiwyBG9wgRkCK0y1+7B71ApAh6h3qKoNfoHikCUoR26c0l\/4GID6mttNJK1YvgtrIwuBUz+\/hsJXCyhslLvT4ENRGDIi2Va5+V7rIPG3l\/1OKGXqd9gobCtr1YDUO+dNHrNK9oT5Ye6kR0mOy43eZm+s1\/0PT6ideLbCeZfsM6yfdrn2HLD0s3pIjoTN0EKSLCIEUEGb1F5Em2U1k8N4jO8wlIEJCirTnc0VtEnmTz9DqVq9+wTvJ5nlPlHoj4cuVarVa12LDLLrtUx7zbE+fTkueee27SndUr3G+\/\/ZLXzgYhQW9qmPNzGILX0RAfcrOBmb\/tCVaQDGfNA9pPhPTs4zPkzXVt57aYomKmC3rJw8x70PT6ideLbCeZfsOa5Jv8hmnHXtMath5N6dmrCBYlEKP3oHUy8v2KnjVz6WD6x\/x2uzZvpIToEB5SRICIMEgRMXo91H5F8KwZSYVNmnTsFFaX98y1020q\/CdMfHUlvdFhgeGss86qTv61CousrNAO2vNSAcgMuZqzq+c50fvp3sdnUcecybAwaHr9xOtFtpNMv2FN8lb3h2WziaTTpNtUpmevIvzLv\/xLMtoBHQKb\/qO3qOfYa28RKQKitV8RLLhYpDSqs2fRM2NOEZyK1Gq1UpS5yR51P\/En+txOJP7QiS9XRi\/Mnjvv1RqWtlqtPHicu2k7i18Rq7N6kV6FGxdpzEM+VplVwNjtEv8RpaH5Ep6Td1NSLhYYOQvEEHrBggXJEBoBIsIgxXwIHXOL4rQrSH0InfcWzSsCP\/OKepR6lu3Smi7\/SSM+8252Z5vvM+TVXbZC26mgfhV0qQO2s+jpIT8nsegegyGtV9bWGVtU0ZPU9TeXp0Ly9C126PJbTcr9R82tfMPUadD0+onXi2wnmX7DmuR9QnSYdhs0rSbdBk1LvEHT6ydeyCI4CFJEjEgRGRpGu7pHiDB\/\/vw0fwz0bIJnEBBefQhtGI0UDamRojyb0pgKv6ESnxVUp7ZsuummyfyDFV9GMD\/HbTW4U6GatrPUyRApSkvv79vf\/na1wGIub9lll61eGQpy1VV36q4vrTnMoFO+yHQ64QEeZv6DptdPvF5kO8n0G9Ykr06HabdB02rSbdC0xBs0vX7i9SJLxrH\/TkPykgIgL0BeX\/va15IhsLcyDIdvuummZL5dvbQDPtADRIrmItvJTbb\/hInPlhFDVL8OjGTzIgMgoH322af61oK9fPbxDDLkrJOhPX728UnLu7nc5hW33nrrZCuLtzdMwvr4DT2cA9hqtR9im2somNzjyIt9Z6d97dnVuTCfb\/7PCUzeLc6H0PXeot5lkFruDr+pug5EfFZnrdIaZppUNfTE+iY3DznkkGSjsXk3v16IayoKY+e4TctWfBcuXJgMkb0g\/exnP3sqsi95FAsUC2QWQGpgWGz4DEjQ0DmI0VxjFmVKnQMRn9VZL0Q7ocEBocjOEBTzW9F1cMFklsJmZl1lvb\/Ix7u7b37zm1O+ncUcYCr\/igWKBUbOAkGK06XYQMQXyhrO3nvvvcmRU+FXrsUCxQI9WKCITKsFBiI+Pa3TTjsteZPCUNZePb0rixoWN5q2lUxrKUvmxQLFAsUCmQUGIj7x7Qq3cOANDe\/SOkHWhkeLGxY6rPCa1LbNhHxBsUCxQLHAqFhgYOLLC+D0FefvmedzWovl74c\/\/OFJT9CStUUQb28YGufxirtYoFigWOD\/LDB1rqEQX6hri4ltK7aU\/OAHP6iGwhY\/LIJYeHDgQMiWa7FAsUCxwHRZYKjElxcihsJWen2LwDcFrLzmMsVdLFAsUCwwHRaYNOLLC2MorMfX7Q2KPM6ouH1HxKtzo6JP0aM3C3hzZ5999klrr712WnPNNavRh832vcUuUtNtAcfObbfddlX9+YKjw4iHWX9TQnzDMeLUpuKQU58D9I6wLTtTm3vJbaIWsOtAGhdddFHyXug111yTvGXAr2D0LYDoHOlv3cCBB0aNzvccluaznvi812txpW4wvx6Ma07SGybeS\/RtgFiAMV+51lprVcfr1+OW+6mzwKD1531QJwOZXvEWkc8S+AbM1GlecmKBQetvq622qr4z3Gq1kp0hnkv1KM1hYFYTnxMozjjjjEY7+danj55404SMIZHenR6CCIbl9iUaJrkvmHoLTKT+fMzGaSK0tqPAj9p0ngZCj7mGidSfV2HtDHH4sANPHEg8zFOWZiXxWUW2odqrbYxfb3C\/+93v0kknnZR8rtK7g053OeCAA5KTXGzAHuTD5\/U8puh+VmYzrPpzRqN3txctWpR8hMqP26w02IgValj1Z4H0C1\/4QjLP7kAEr6kOq6izkvgcTIDIPvCBD6QVV1xxnK0cYOC8QMTYaj14cssjH\/nIZGh75ZVXpl\/\/+tfj4hSPqbPAMOrP5LjegiPPzfdtscUWqdV6sK6nriRzM6eJ1p8PitkXHFMTRl86Mk5bGpZFZyXxrbTSSskZgIaqjq5KtX9Iz8nM9cMUzAP9\/ve\/Tw5hqEUpt1NogWHUn6mM9ddfPzk7zgOjjtMAABAASURBVIMzherP+awmWn9eedVbv\/baa5O5PW+EOdjUfPywjDsria+bcRyWqBvdRIrd4pbw6bdAt\/q77777kp67t4jM85kjAtMb06990aBb\/emU7LXXXskZf+utt17y2QqjM+d9Dst6c5L4+jEeg1uZ6idOkZ1eCyy99NLJR3F8+tQ2iICFrOnVrOTeqwWcsG5+zxYkZ316AaLVGt5UxZwkPsviutK61PWK8GtjK0vdv9yPjgVK\/Y1OXQyiySjU35wkvpn+caJBGttsilPqb4DaHKEoo1B\/c5L4bF+ZyMeJRqgNzUlVSv3N7Gofhfqbk8Q3kY8TzewmNzu0L\/U3s+txFOpvThKfZmNTa\/k4EUvMTJT6m5n1FlpPd\/09SHyhzSy72rJiZahpVdY7nE6MKR8nGt1KL\/U3unXTi2ajXH+zmvh6qZwiUyxQLDD3LFCIb+7VeSlxscCct0AhvrZNoAQUCxQLzFYLFOKbrTVbylUsUCzQ1gKF+NqapgQUCxQLzFYLFOKbrTU7OeUqqRYLzAoLFOKbFdVYClEsUCzQjwUK8fVjrSJbLFAsMCssUIhvVlRjKcR0WqDkPfMs0BPx+cqRj\/AcccQRaZ999kmO9HZ17yMu99xzz8wredG4WKBYYM5aoCPx\/exnP0u77LJLcob+DjvskHy4xZHevnXp6v6Nb3xjcjLqjjvumHzNas5ashS8WKBYYMZYoJH4fHfiwAMPTL5L6vsUBx10UDr33HPTDTfckJBhwP2FF16YDj300OTLZT7oIt6MKX1RtFigWGByLDDiqY4jvt\/85jdp2223Tffee28655xz0je\/+c3kyO7lllsuebE\/L4\/7pzzlKZU8OfLi5TLFXSxQLFAsMGoWGEd88+bNSz7K4tOMT3va0\/rSl7x4fUUqwsUCxQLFAlNsgXHEpxdX\/+ziFOtUsisWKBYoFphUC4wjvjw3H9b2eb7ll18+dcMaa6yRfBLO\/F+eRqO7eBYLFAsUC0yjBToSn68h7bfffun5z39+cqjgK17xinTQ2EKHrSzrrLNO8jUyn4F79atfndZff\/105plnVvN901ieknWxQLFAsUBXC3Qkvkc+8pHplltuST7DeMYZZ6Sjjz462dbylre8JTnZ+LTTTqsWPFZYYYUq7LzzzktPfepTu2ZaBIoFigWKBabTAh2JD+Gdfvrp1ZfMrd7WFV155ZXT1ltvnciQfcITnpA22mijuli578kCRahYoFhgqizQkfi8sWF\/3sMe9rC2+ggjQ5aQe9eCYoFigWKBUbVAR+J77GMfm1ZaaaV0\/vnnp7vuumtcGbyqdskllyS9wUc96lHpL3\/5S7r55pvHyRWPYoFigWKBUbJAR+J7xCMekXbbbbeKzLzF8f73vz+Z14MTTzyxGgJfdtll6XWve1164IEH0pve9KZqw\/MoFbDoMqMtUJQvFpgUCyzVLdUFCxak448\/Pi277LLVu7q77757goMPPrjqBe6\/\/\/5ps802S\/fdd1\/yqpstLd3SLOHFAsUCxQLTaYGuxEc5hxSccsop6aqrrkoOKIBLL700\/fCHP0zbb799arVa6UlPelIi41u14hQUCxQLFAuMqgV6Ir77778\/XX311emTn\/xk+vCHP5xsbP7rX\/+abr311vS3v\/1tVMtW9CoWmJUWKIWauAW6Ep8FjD322CNtueWW1V69L33pS+lXv\/pVuvbaa9PGG2+cDjjggGpRI5V\/xQLFAsUCM8QCXYnPgQVOXTnkkEPSySefnLzNoWzrrLNOeuc735m+\/vWvpxNOOIFXQbFAsUCxwIywwFKdtLQp2Wto2223XXXq8tJLL71Y3CtsO++8c\/K6mlOY\/\/CHPywOGzUHgu70rrETpYetv6mB1VdfvXrH2Q\/HRGxiaqHTO9N1\/aO8df+J6DBVcY899tjKZnl9TVU5Im+2ZvNeyxz2pnNTXGnxFy6PXtMNuWG2pUgzrto9+9LtM5\/5TIo2674JE23Lke\/Qr30m2JH4bEq2OXmZZZapFjDqabdarSTMYaUz+Rw+izXeQ9ZA62WcCfczXX82jgfwyCOPdLsElO+5z31udT7kEgEjcKPNXH755Ys1ue2229KPf\/zjxfej7vj5z39eTVs5Us6rp9309brqIOQdPw5IVl13y2eywzsSn318jqjqpKiwRz\/60WkmvLFheK5ROkEm4Ph8RtZgDdu5RxlbbbXVEqdg77333pW6M0X\/StmGP8gNBKmTqB8H3Mb0iukWRENmMmAEI98LLrig2qXQSx7aDNvnst\/5znfy25F2249rHv81r3lN8qyHsnkdsInnxvMj\/Itf\/GK1wMk9U9GR+Ly5oYtubu+KK65YooxWczUQYQzyuMc9bonwmXLz5Cc\/efG8ZV1nv2z17n5TVx\/5+yULWTa744476slV9\/mwhbyhBb8qcIA\/66233mL9b7rppnEpXHPNNUsMX+hJ31wwfo3pE2gqZ5M9+OVpcYsb6biyRzfCCrLQlkA68JznPCe99a1v5Ux33313yu0qTWnLI9BUPpHpGTKu7qPcEYefMGlKW7xOYMfvfve7lchqq62WwI0eYLf4wuUjP\/mGLu6BDaXVhG6y2pN2JZ1AlDFPjw509cOiHeVhdbepLScx1f3d0zXycVUuaQtzde8Hwr0ft7z33quu4g4THYlPRj4mZB+fubxFixYllf3ud7978QeGnvjEJyYyrVaL+Giig1YXX3xx8otHZMUVV3SpoHE1Dbt09YVVQmN\/VKxhsgodu63+6wGQi3Qrz7E\/GrjV8dyfm5\/GMyYy1P900uDlEQnzo5t6BOHRKEPGlQx9uUGZm+zBTxgZ4BaXO8Aem2yySbUlKvzaXe0WMPzKw6MnZh8pIhTmgZGmtN0HlM9DLDz8lIOece\/qvqncwnoFPelLXhsAbjrpIXH3giZd2JDe9fj863rzi\/bjqj3ldS4NdqGf9uoe6EhXhL3ccsvxagvxkCSBeEWVeyL13Y+u8homuhKfX4OPf\/zjyZHy5vOc0Oy9Xd1i5\/LZ3sIQw1RqstJS+X5t\/CoFNDr5GTLansOtQqJx8dfVB8NM4cLIcIuv8XDH8EA+5kz4BTyIn\/jEJ6pb6UgPuHkOOoxrR9zShCb96Qf5g9skpzeDHOl+9tlnS656e4feELp7IDwYbMI2BCM9D5cenAfx8MMPr344hdfh7R9+5Dy4UT9NDz+dpEWWnZWFPjEs5i+cHN3D7vSgD1n6yW8iCNt7RpAtcEszerDc3SAO3emlLMokTtOQUhgZsuKIS9bChPJGvk1y2qnykycrDvcb3vCG6rxN7gAyjTpwZTvxpWsvrx5gL\/XtHX4jw2gr0qGDZ61XXUOnYV67Ep\/MnMtHce\/oXn\/99emnP\/1p9dU1xnnMYx5DZMYjGlneIFSSN1OicB4WFe9epXnYPfTu2UdlcnuLxf5G7oAhmgdSQ339618f3ombn0alQSwOaONALBpiAPESpRcduAN1vyCXCNd70ovyEOlVhX9cmxat1LkGT+aoo46q5hs1bGWOoXaerwfEg0Ve7wjZctfBduxb91e+KGvk68EHsuwsb27liWFx5JWT07777rv4AVev6le8QaCd+GEQN3pMek3c\/LQL7YO7G\/Rc6U5OWcy3cTe1CWFkhOf5uYd6nfDLEXWkHthIXekM5DKd3HTSBslEWtKIttdrfYvfq65kh41xxOd9W4cRGM72CvLiDVu5YaenoSMXD3pAL00+KtRDZnXaA8\/PnIaK5Aa\/XtG7vf3226sv0fGHfJjsvj53GI0E+eU9Gm5+4oQMdz9QLj9K8UBEXLrSOe6X1OlBXw+w4W6Qi2s07AclUhJv3rx5cZuQHzkw1ImA0J8t6SQcyJNRTj8A3E1AvupFHYlfl5EO8ot8\/GDQLZeLeqjnRf9cVr2q3zxuP+4gDnFe\/OIXV4QqTW5+bKAc3N1Q\/0GKMjTFy8Pk11QGNmL3QN7GIs34QXC6er3dkPFcqIuAskSd6EHrSUc9KKuwyE89SaNeB\/zq6EXXepxh3I8jPltYvIN70UUXpcCpp56aDGk9EBdeeGHyS\/eVr3yl8nMyM3nxhqHQVKehwkC+fqV9XpO7F\/hFN+Hei+ywZPyyRmOMq9OwPQT95kF\/8z56T0jE0Ema9Z6XBwOxhp3yfLQJk9fSyv0n4lYWZaILeAgjPT3tcPdyjYezF9l+ZII4xPGDGQ89Nz\/oV1dxJgpTA0E80VbUr15ZpO3HzjOszo04wr\/TVZ1Ezx2hKX8n+V7CetG1l3QGkRlHfBYrkBzDgLknQ11zJrrGiA8hXnfddemwww5LehTm+sQbRIFRi2Nrjp4SvfTqNBJuqPcG58+fn\/QkhNUfMD0bDURYDg1QQ\/RA16HHk8tOthtp+bWmkzm8GG7Vy0IPDT8nIw8Vf5CGHgE3IEj39fK5N6Qlk4ONo9fp6j4PNxSjY\/hFr4d92Tn8XUN3D7UeXsj6gcpl5aF+xekXSN7USLd4fkjJdpMLnUMu7qMM4d\/t+tvf\/jaFXn68DCXFUW7l5wbtDwzLDZf5TQT91re82KUXXclOBsYRX56JxnHcccdVx8tb1fVxoQi3yOHD4+YnPvaxj6WZ2uPTAEC5dPuRWfyy8f\/85z8vqIJfcw+5G8MTPSFx3CMR3XZuleoHgzsQE9\/ikw1\/v3p6C7YfGD6E\/1RePRQeDnnSAQlyB5SLjsDN30Olh+jhdA9s4spuwA2Gw+K26xki1RiyiQfiBdiL3dwjMg8auGdn9uamu2EYdzzUYXck6cdbmxauXuv5pJQEdQVSD30QDELPET8KZMh2S5DOdCfn6p7bjyry5u4XQZ7ixV49boieqGE52\/PrBnaLxRB1zq6D1nc9r0661mWHdd+R+KKHYz9fqzV+u0qr9eCbGxY7vN42LKUmKx0NXe\/BQxiIYYHKjG6\/Xkk0XmQXsh5Augkjw63hR29EWmQ9lBq98IDeVEy852lyk\/EDQoZ7qqDxKjdSMA9Ed1f3uQ5sVi9jLqu8wCZsI27YglzYLZ+YJ5OjyY7iQthIHhYlPKwWKujOzvzJhe78hZNjU7aVV17\/kSb\/fhHEIR82rMcPQuCPLDxH3O3A3nTPy0A2X7hx3w2+eZP\/EEsPwv7iIzA90Xa6k4G8\/qShDbCfMPZk10HqWxrS8oPQTVd5TRY6Ep+3MQz9DHEdK19XwtFUvsJGhmw9fKbce3DMWajM0FmPxsMY93HlJyzu9frMdwYx8JeeYaHG5T5gKFvvJQkzh5WnyW8qoLwf\/OAHl8iKLsATqXholLHdHB+iU1YkI45yRHz3wA7KrfzumyAPq8Ps2xQuzTwfuuuZ5nYXzz1\/4e6BTvV03dNdeD\/Qu0Qc4kSvkjuHB5se\/Dw7nhHudqjrEvZCLO3itPM\/6KCDkvYX4dzIxpWfQ0fUa5AXv36gHtgz4nDzi3vX0D+vb50K\/sID3XQ1FGbvkB\/mtSPx2au36aabVl9RM6TIJ\/65+X31q1+tvqw2ym9uqJypbOWUAAAQAElEQVR8KFJ35w9UblwVV5fll8twx0MbstLT0GKrSN6APZDhH\/J5uPTqyNNXlnp4\/Z6MtOkRhEQmzzvydCUbcA9xH+WVjvTCP67yknaOPD455ZV3LtPOLT9x6pBmPU5ul5BHnvzrsvV09Rzrc3wh0y4NaUpbuPzYg13458hllN0QP+LII5cNNztKE8TJ7cXNT1jdDhEvdKEPN1ngpo+rewuRruJF3nHN8yETyK\/1\/MXll8vQVVrCAu75h5w43XRlM7pHGsO8diQ+GfmOxg477FBNmq699tqLT87gNk+ycOHCtNNOOzUeYiB+QbHAdFrAnKShGphrDF30gsB9P3Nd5AtmvgW6Eh9WPvDAA9P555+f3v72t1fHUBkimAP4xje+kcyVkJn5piglmI0W0PMGZTNsR4Cg\/fIzJNWeuQvmjgXGEZ\/DBxw1XzeBLR4+MnTEEUck3WRbWHRfW63xix71uOW+WGC6LOBHOYZ5MczKr5M5nOpUZkM4edOl3fC3U\/wSNjELjCM+c3c+F2nTctOCRj27\/J68eLnfqLut0P3oRz9KP2qAI\/brGPXyFP2KBYoFultgHPHZiLzbbrslLyK\/6EUvSg4o+OUvf5n0BJuS0zv8xS9+UcmR\/9SnPtUkNrJ+XrczhG+C7Rd1bLjhhilHPdx9npb0AyeccEI6IQPShZx0g2hH1mBFsWKBWWCBccTXarXSC1\/4wvTtb3+7mtOzD8nk7yqrrJIQG3dg\/fXXT8961rPSS17ykvTZz362kvfWxyywS89FCKLKrzmRIbaAjaQ5ghBzokScEOTK5oF111032fsEz3ve86qjwWydWHXVVROsvPLKixefzGMVLF\/ssfxo26DnB23IguOIL9L3mpo3M773ve8lD+vmm2+eHv7wh1cfEdcDdDSV19UMi7\/2ta8l++DIixdpTOXVMNvnL5GCB36jjTZKvgXSrqcaut15553pXe961xKw56gJNqbmWLBgQVqQYZlllqmO4l9m7BrpT\/Rqf2Rg6aWXXvwyvE3l8PjHP746RMAOf\/Owz3jGM9Kaa66ZrLrbFGvF\/T3veU8Ck\/tgXqkJ5557bnXaSlNY3a8X2U4y\/YY1ybNtXa\/puG\/SbSJ6DJpeP\/F6ke0k029Yk7z6my60Jb5QyKtpehwWNCjvlRqV6nrWWWdVD9Qaa6yR8tfZIu5UXr\/85S+nD33oQ+m1r31tcnCCB3\/XXXetDlpIHf7ZyZ6TGfeiRYvSogbUCVJ+OWy4DCDdHOEf14gXadaJlh4LFixICx4CUu1QjCWCvLOpB6rnqbfphwuiZ6k3qVcJep2G3+T6eftmuS4HV1Kok0y\/YU3y2qN8phtNuk1Ep0HT6ydeL7KdZPoN6yQ\/EVsNGrcr8Q2a8FTG80EkO9K9T+wbwM985jOr7\/2+4AUvSDZYe8NkKvVpygtx5QhCQ3BQJ1qEGOToijCDSLmBP5AN4pSWtOXVpEf4IUZAeEgRAUKQIpJEiIBAI165FgvMBgvMCuK7+eabkzP07AZvtR7cXmPIvdZaa6Urr7xy5D+M4my3fhoTUgMEB8guiBMJvu1tb6s2nCNKBAk5QZIXr54nIuTniuwQIiDBOiHavE6GfDssWa4lpfoN6yS\/ZMpTfzds3QZNr594vch2kuk3rJP81NdYSrOC+JCeeTCv2OVGtPDigFTzeLl\/7kYANrDajB2w278Ou\/zrMNzPoXIHAX0GidcuTp7en\/70pwTmA\/WEvT6lZ4wcLVzZgA6+m2JO0Ev37CaNOnJC\/PSnP10tZiHEV73qVWn\/\/fevdgLoQYZe4oe7fu03rEneXHM93em4b9JtInoMml4\/8XqR7STTb1hd3jw8v+nCrCA+D6SNqk6n6NeQjoMC7xEGLNTUceaZZ6Y6LBTksLKd41vf+lYKONsw4GSKgBfYA+bYoNVqVa8Atlqt5B1o8yOTBYtB4N3VPffcMx166KHJsUjRW9RTNIxu10tkb3OK7OUgAsNlx3rF6d3mXm3UVS4YZjme+tSnpmGmV9JabsrsaZ1A25kuzArimy7jdcvXOXcBex0DhogBpBBAHGB4GfjIRz6SDj744ArcAeFkzz\/\/\/HT+GKQX6cuzm269hMdw2jDaEBoJBiG6R4h6zE1p+TECPUC6Gi6DHqJFFW4kaQ6RDP3JN6U1i\/1K0abJAj0Tn9VPQz3HE\/33f\/93dfDosA8fddzPy172ssY5OXm\/\/OUvTyussEKyd80Ga0M4dnPcDbfN03ovutG2sxiGGgJ3WnG2FceCCHiQ6zD8q8NwsQ4EEJg\/f36an2HevHnVSc3zxq70HRQILYDkkEVOmggGkCOydAV+QZLiiNukg+FZk3\/dDyEaNiPEnAy5w35sUY8X9wiOHgiPbggQESJExGjozM1PGHJU3+TFA2lEetN97dVuveo5aHr9xOtFtpNMv2Gd5Hu1yzDluhKf\/XFOWLZlxbHg3td1xpgj6C0eGNp12yvXi8KI6+STT17iAz4R78Ybb0yIySm95uEMo44\/\/vjqk5fytnfNYY4eonw7yzHHHFPte\/OgRlr1K0JHSJCTVbg9wHVssMEGaYMacnKMhz+udA848CEQfuYYQzbSyYk18g+d6FovR9N9E0kiQHZCjBDEqNdoqN+OFJvSz\/3YmJ7IEJCgOUQLKxA9xE5D5kjP0BmxIThkR196I0FkCIjRkBpRcgN\/IAcIMyAdkCZIHyLPcp1bFuhKfN69\/ehHP5pe+cpXpn\/\/93+viISJvMnhTQKHFmhQ\/AaBX4ItttgiIVbzZU1peHDMdTk63MEI2223XXrHO96RHDhpgtvmXfFMypu01xvxAOgJ6vG5Ch81IDBQpiA15AE5sQYZBjkizDp5CiMXhCkNaUq\/U7nvvvvuhOz0HBEfkglCRIoIEcjk6ZgPy++b3GQQIiA8hIgAkWIMmdWte\/oDObqL05Rmkx8CA4QG2iMoSwARgnYBiBIQZw5+AXIBcQMnnHBCOiED28kvQIc66JejqRzhx27h7ufaT7xeZDvJ9BvWSb5exqm470h8ekP2wb30pS9N73vf+6phZqvVqvTyloAelSGl4eKgw14Ppl6kk4Cd7Vclnv0xIa4RIVnvEUeQBQlh11xzTfLwPvDAA+n6669PVhsNcd\/73vcmvVU9QQcvRLxRvCL\/QfViP0ByCANh6gEjQUQSJOkK\/IIcxRG3KW82BQ81IJAgRG497\/PH5hbrhJin1alcwpAb0HvRokVp0RgQox8vhIgcgZs\/gnRVBrACLS5IB\/L8B3Hn5KTdBYLUXJW\/jiBF1yDL\/BpkGtecbMMdYTHUd5+nwS39OoKETQeE25WuOaIsrvxd8\/Jy5zZTR\/l97u43rJN8nu5UuTsSn28F2CriiG1vcNSV4idsIt\/c0JNzyrPPHHrXtJ4HcnP8tBW8PGzZZZetVjxvueWWag+f4++9PucY7IULFyZ7+3zjAiF32s6Spzmb3QgO5s+fnzZ4aJiOPJCh3qOrH7icFFPDP2SI7PQO64RoOBpkSK4h+kBeCE0vHsFtttlmadGiRRVswUGGgBwBUQJ3QDggTVDugPQC0g\/IMzCQ0gNEQjwQQ31u5JQDYdURJMz+4XatEyTiDAjjRq45goRdTSXkYeEWzxYoV5BWAOHSwzVAX6v+rnlZjMYGMNNQonQkPoohlE5vPggj12o92BMcilZZIohPz1MvJvNewqmBGM46XNICCdI7\/fTTk4dlCcE2NxZD\/CJNF6g1zLwHSc+716YM2BkcROHkbfOPCFEPG8hIvw5Ep1EHGRomm\/\/Vk7fZWQ\/R3kiLVMoqvmsTmsKa\/ExzNMXnZ844YA8jaA8Q5O5qT2NATzPg06kBc5UBD3wORJoDGefQ865DTzWHKZo6bM3KofzTAc9WHerZ6MoVEFoA4SI+1wBSNBJzRZQBo8bpKJM8OxKf3pjXvhREz0+EHBqYsGc\/+9lV7ysPmyz3ZKRrT5E5iILx+7jMPzrBeOHChWnhGByaGT1EDzTymD9\/fttq8aPlh0gP0a++vZAIMd\/fqNcOfuRard73LhoFTHWd2TWQI3qLcbUfMoe9kXXYK5nDvsk6fMAqh15sHdGjrV\/1bnPkxBzu6PHm1yhDXBcsWJAW1BC9YNe2lT4DAjoSX6vVSm9+85urYthmgrHvu+++9LnPfS4ZVvLzS09Gr68SbPPHr72eVSD\/\/kGbKJW3E0ke8YhHpDvuuKO6r\/8x3Nbb8+vuwamH06vTdpa6\/HTc+wEZZr6DptdPPD1ED8UGY8NmD0+QoSEzMoR2vUNl1W7AsFkvEfyI6iXoLcZ8Ijc\/vUbD6ICehjYhvnSkOV3ox2696Nhresgnh2fAvXrJEUSWX9XPokWL0qIMQYpx1fvNCZQ7SFZvuB0R6xGTIx+INOOqzTQ9r73YZxgyHYlPBrqjju62h+4HP\/hBtWBgK4shiwUHO\/P1+Mh2ghNexAk4Jink+Ukfsd56661JryIq37FLuvzyJhP7+Kww\/\/GPf6yGs4Zm5vIQsl9j5GrRxXDXw6cxRF7lOnkWmDdvXrVfcYMxMgRDZIQICDF6iB469TJvTL6TNggNkJteI3IMIEm9R6SIHBElcAeEAdkgzLgiTpB2QF7QSacS1t4CnjPwvLrm5GuKISdehDudtu5KfIppNdW2lZ\/85CcJAV166aXJGN9eOkMNMt2g1+Y7A4HHPOYxVRSfu3N8lGELY8jL+7X8rMYablv0cDCqMHNF9vG5WskVxqgOJfArY6+hY6lsj9HgpSuNKrMR\/dOrDduqXwsYNL1+4vUim8sgOQ8CQgQ\/bsgwSJE7J0ay8+fPr4i0VryOtx6mQBAaggvCjKu2AYgxEISJQHOEf1xD3lUaXj90hSDW\/Cr\/QOiUX0PfuOZ261jYWmA\/8XqR7STTb1gn+VoxpuS2J+IzT2PRwDDXQQAqzQQuorrsssvaHkvfrQQ2H+tNPv3pT096e3oBCMyHjExcmxNqtVrJ8fbSchCqqy0qrdaD\/np6hrJIUJjFmD\/\/+c\/VRmjDYOERX3jB6FkAKcKCBQsSUgQkaDiEEJEjcAN\/4RZb9B5B3Pnz56f5D0F6wyppEFJctf9AEFpcg1jzK0IMIMs6glDjmpNuuCOsfq2nFfnk15yEwx365tcoU1yjvHEdlj1HIZ2uxGdS2rDRnBzC8RaFb3JcccUVSZiVv0EPhDRUtWii0ea9Mm9ouLeaZB4ACdKBrAl2W1ZsYkZ29vEZBiFl+5+cwkxG5Xk4LFzYDjMKxm6nQwzr24X36z9oev3E60W2k0y\/YeSRGSA3bebf\/u3fEpIEdY0QAwgSWQbcB0JGHECcOaQdkFdA3oF+62Si8kE+9at2niMnsnDnJBzunBjDXSfROskGCce1Hu5eGscee2x1anuk62qEFqTrSrcVV1xxomYZOH5H4rNVxTK+YalhrWHqeeedl\/SkrMo5bUQD8Z0NPat+tdA78zqctzDENfxRMSZpTZ7LB\/EhLnN35uyQnN6nPXp6aNwsxgAADClJREFUn1YDESJZq2nCyJDdZJNNkmFzt3185gSnE0h9mPkPml4\/8XqR7STTb1iTvDbT3m7LL\/G9jfhGiatvxICtOmC\/Wo7YbuEaD7mrBzvgAc9hE38d5iBzGMHk0HFowk033ZRy6ADUYRSWgy2mGnUSdo+EY9EJuQWU07MdQIa29Ey1zpFfR+JDPnpUfhU1METoHtk97WlPS1ZT\/doOuoG51Wol5OVIqVBI785SvyGvBRHEp4ItYIRM\/apRSMOkaj2s270eYcHPev7WRrFVs63M89VxySWXpBw+1ZDDK5pNsEiXo77Vxf0HPvCB6l31uEbPNr9GD7d+jR5vXD3fOfLeb7g98zmiFxzX6Am7dnvmRiG8I\/GZg0N2toRQ1mKDAwosHIQf\/2HA62WGqbbI3H777dX3MyxcDCPtkkaxwFy0ABJqQpBVXHNC49aZqSMnRu4gzbjm5JqTL3cexh1xXKerXjoSn6+o2c5i6IiYrOj6voVjoSjMDxF6fYwsv3botI\/P\/I3d81aO9fL8CvrEpbTmwj4+5SwoFpgCC0xLFnXyDcJ1nRaFxjLtSHyGj9tss01CRHbvm0z2bq6TWQx5bWI2h2H+hexYem3\/IzT79QKxj8\/8nFfNzMPp3jv4ANlGQrGPzxmA4efq3rvEtrIYBptjNMcgLGDLjZVie4rCr1yLBYoFigU6Eh\/zOI7KnAJ2RnB2ZSM582p6gnvttVeyrYVsJzTt4zOUNrwVzx48vbxWa8l3fq3uem3OpLA5R7Lwve99LyFFxAd6nRZbpCncvKD5FXGlwa+gWKBYoFiABboSX6vVSk5PsWJlmdqGYBF90cwKqtfVrKjy6xdOofAWiLSkYfU2B\/JqtVpJr9OixwEHHJAcOYUskfHWW2+dbGoGbi9CO4WZzL777lutjInbai1Jpv3qWeSLBYoFZpcFcuIbqGTIUE\/Qwke\/CYhjeHrcccclQ+k6vJ8pTa\/EHX300dXKoz169vHZS5j3NLltPxBGRm\/UAariSqOgWKBYoFggLNCV+JCTtzRWWmmlJfZF2d4Chr6Gsd6eiER7va666qrJRuh2WyTs64u0kKK9eQitqaepx6j3KYwMWXEifrkWCxQLFAuEBboSn2Gl+TUvnNvTY++eHpWDQ831WbTQY7MfLxIt12KBYoFigVG2QEfis3lYD8rrYYaa3qc1n2a3u3u9Kj1Cu7VHuZAT0U2PNFagJ5JOiTu1FjA\/7J3vtddeO6255prJyMT2q6nVouQ2qAXs5cU76s+ipw7YMOuvI\/HZImLfnu0grVYr2avntTWviSmQhQ6v\/nh1hyy\/2QILL84cc0qMbTOzpVxzpRw+g6qsF110UfJuqO1XVvn5FYy+BRCdUabtb15vs2PDG2LD0rwj8Xk7w\/u03t6QITcStIfOPRjuUkjv0P2o4aijjkoOWKjr5deDcb0DbK7SAo2DFWM7jDlLn8+0nacet9xPnQUGrT+jEG8Bmfs1PeNod8eqTZ3mJScWGLT+jCq9IdJqtapveHsu1aM0h4GOxOdwAosap5xySnUSi9NQnJxizs+eOuRhIQFBtlqjt2XEGyHO5msylANUnQ6bf4dX704Pgby9f7bxGCa5L8gsMEXOidSfk37NSVPViUJ+1LyO5b5gaiwwkfrzyQPH0Dlf0\/dfLITqdA1L847Eh9B22mmnZL7E8VPXXXdd0v20CmuI63QF+\/s0sEEOCBhWIerpIGoruvYHMn493PD9pJNOSl6T8+6gDdD2CNrs7HOa0cOtxyv3U2OBYdWfU35sb7LVyTHq5oumpgRzO5dh1Z\/RpPM6zbM7AMLJLsOybEfikwmmNb62Mdicnn1xvt\/J7T3aHXfcMXmVrdUanR4fHRGZkyuazvzSS\/WqHGJstR7U22kwhrZXXnllsola2QumxwLDqD+T43oLNr6b79tiiy1Sq\/VgXU9PqeZOrhOtP5+fcMp6TE0YfenIOKprWFbsSnwysk\/PXBgFWq1W2nDDDdOpp56aECKCwczkRgWG57bbGKo29USRnt5sfQuOeaBezu8blXLOVj2GUX+mMtZff\/3kHD3tdvJtVXIIC0y0\/qwX6K07AMXcnmk15\/rhoMhjoteuxCdjw0XzYVi4Cba5IIyJKjNV8b1njKybSHGqdCj5DG6BbvXnNG4995NPPjmZhjFHBKY3Bs+1xByWBbrVn06JMwAcjmI6bfPNN09GZw6RHZYOXYlP49Fzsm\/PSctNsJ9PYxuWUqOUDoNbmRolnYounS1gCsa3e30PxqgkYCGrc8wSOioWsIhqfs8WJIupTldvtYY3VdGR+HQ5EZ\/z90wsWtRogjD7+0bFaN30sCyuK618dVm\/Nray1P3L\/ehYoNTf6NRFF00ag0eh\/joSn03JVkB9u8JR842lmIGe5fy+GVhpmcql\/jJjzEDnKNRfR+KzqGEBYLZt77B9pZzfNwOfmIdULvX3kCFm6GUU6q8j8T32sY9N2267bbIdwEroDLXzOLW9b1zO7xtnlhnjUepvxlRVo6KjUH\/jiM\/qrFXaWL21jOz7t5tttlmymTn88yt58RpL2d5zWkNsai3n901rFUwo81J\/EzLftEee7vobR3xWZ63Sxuqt\/VA2gSK\/s88+O4V\/fiUv3rRbs6aALStWhppWZb3DWc7vqxlsxG5L\/Y1YhfSpzijX3zjiszprlbZp9badH3nx+rRLES8WKBYoFpgWC4wjvlwLq7pe3\/Kubu5f3JNngZJysUCxwORboJH4nLpi3s47d96RW3311dMuu+ySHDo6+SqVHIoFigWKBSbXAo3Ed8IJJySnWjz96U+vTjBxqoX5vf322y\/Z+JvKv2KBYoFigRlsgXHE520GixVe8Hby6RFHHJG84+gElksvvbT6ZOMMLm9RfSZaoOhcLDBkC4wjPvN63tZYd911k1UZ+bVareTcewcW3HHHHbwKigWKBYoFZqwFxhFfu5IstdRS5TyzdsYp\/sUCxQIzygJLzShti7LFAsUCD1mgXCZigUJ8E7FeiVssUCwwIy3Qlvi8qWEPX8C8nzk+r6aFX1xtc7n\/\/vtnpAGK0tNvAdunvF3jAzMODtWuQisHZOy2227poIMOSldffXWyterYY4+N4CWu9ps6br6exhJC5aZYYMwCbYnvmGOOSfbwBXxbQ8Pyqcbwi+s222yT7rzzzrHkyv9igf4t4HOlPlDj+y4+ZeC8tkjFtzMQns8dhF+5Fgu0sUDP3uOIT6Pz6+rE5V5BXryecy2CxQKZBWyh8qOK3Jx47cNPEXzVVVclh8OuvPLK4VWuxQITtsA44tPoNEAf6+kV5MWbsDYlgTlnAcPWLbfcMt1zzz3pyCOPrL6REUNdUyuOjfethUG+qXrOOeek5ZdfvhFlODznmtoSBR5HfEuElptigUm2wEYbbZT23nvv5DsZCxcurObyYvTgozQ+GuRDQa3W\/31voT7\/jCjBXLP5wlB5tdVWS\/VRi1cv9SDnz5+fHvWoR4Vouc4xC8wl4ptjVTsziutThL6k5Zgww9l89HDjjTdWhViwYEF1jT\/1+eeYa0aiV1xxRYglp2znoxbpnHnmmelxj3tc8vplbNBfHKE45owFCvHNmaqeeQU9\/\/zz0yqrrFIRWK79rrvumr7\/\/e+Pw7nnnpvWXHPNXHSxW08QYd52221VrxLJLg4sjjlngUJ8c67KZ0aBbZ\/ybrjFDkPTXOtHP\/rRyfmPdTjSXM8x1f6ZKzz++OOTLwbuscceyWniNZFyO8csUIhvjlV4vbijen\/zzTcnc3nm6Saqo88n6O0ZUm+\/\/fbl1cuJGnQWxC\/ENwsqcTYWwane5v8cjTaR8vlIlsUTvcPDDz88xcLJRNIscWe+BQrxzfw6nHUlsK\/v4osvTlZzfeJ00AKa13Osmnk9x6o95SlPGTSpEm+WWaAQ3yyr0NlQnJtuuqk67XuNNdaYUHEcqOtsSQfpmufzmdTAWWedle66667m9IvvrLdAIb5ZX8Uzr4CXX355MsRdbrnlJqT8DTfckBDehRdemLzvu\/vuu6fAXnvtlXw9cEIZlMgz1gKF+GZs1c0exR1O4NW0nXfeuSqUq8+C1vfZ1eUq4ewPefHMD5rTc\/BBuy8Dyk96WfTinEMWKMQ3hyq7FLVYYHALzK6YhfhmV32W0hQLFAv0YIH\/DwAA\/\/8X9FP+AAAABklEQVQDACxGfc7O3GKWAAAAAElFTkSuQmCC","height":191,"width":318}}
%---
%[output:4ef7c339]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAT4AAAC\/CAYAAABno7D5AAAQAElEQVR4AeydC7yVw\/rHnzeik3QhVCo7pQ5\/UpRTKpRLN+USkUuS05WcdLDJpaKkkBwlUSi6SCfUSaWb6KL7RUSF0t1R0jkll\/jv7zD7vG1r7b32Xu9a7\/uu9fTpWe+8M\/POO\/ObtX77mZlnnin0q\/5TBBQBRSDNECgk+k8RUAQUgTRDQIkvzTpcm6sIKAIiSnwRvgUapQgoAqmNgBJfavevtk4RUAQiIKDEFwEUjVIEFIHURkCJL7X717vWaUmKQAohoMSXQp2pTVEEFIHYEFDiiw0nzaUIKAIphIASXwp1pjYl2Qjo+8KKgBJfWHtO660IKAIFRkCJr8DQ6YOKgCIQVgSU+MLac1pvRSCYCISiVkp8oeim4FXyo48+kr\/85S\/SoEEDueCCC+Tcc8+V66+\/XrZv3x5zZZ9\/\/nmZNWtWTPnJ9+c\/\/\/mw9z3yyCPy3\/\/+V\/bv3y89e\/aUdevWxVSWZlIElPj0O1BgBGrUqCHTpk2T999\/X5YsWSKVK1c29wUuMI8HzznnHFM+75s\/f77J\/fjjj8vRRx8tjz32mJx++ukmriAfv\/76qxw6dKggj+ozIURAiS+EnRbEKv\/444+yb98+OfHEE+Wnn36SgQMHGo0QrZAwcciAAQOkdu3acumll8qKFStMUyCd1157zWiO9evXF6vJmcQoH3\/605+kXbt28vHHH8uGDRukU6dOghb6n\/\/8R3r06CF16tSR8847T5588klTH+LvvPNOE3fTTTdJhw4dTH60zq5du0qLFi1k0aJFMnfuXFOP888\/Xy6\/\/HJZu3atfP3119KqVSvp1auXUD8029GjR0ujRo1MeXPmzIlSS40OKgLJJr6g4qD1KgACH374oVSvXl1OPfVU+b\/\/+z\/597\/\/bUhj+vTpsmXLFkErQ\/bs2SOTJk0S4rdu3Wo0RO4hSl5LHoaps2fPlg8++ECOOeYYefvtt0nKVY477jgpXbq0QGo24+LFi6VKlSqGxHjfZ599Jjt27JCRI0cajZB0tMRvvvnGPiLUb9y4caYt7777rkDCCxcuNMRKGWT87rvvpFatWqZ+5cqVE8qdOXOmPPHEE6auP\/\/8M9lUQoKAEl9IOiqI1USrWrNmjXzxxRdG60Ibevjhh2XBggXSrFkzKVy4sJFLLrlEli5dKvPmzTPxaGvHHnus0Z5oF2QE8VSrVs0Ml4cOHSorV64kKd\/SsGFDMw84YcIEM\/yFUHft2iXr168XtDjHceTkk0+Ws846K7vsiy66SKhP8eLF5d577zV5n376aRkzZozs3LnT5EOTRXt1HEcKFSpk6n7EEUcI5Hvw4EFBRP+FBgElvtB0VbArCglceOGFhgC+\/\/57Q3i2xgxxIUF7n\/PK4gSEA4EikFXv3r1zZvvDPZoa2h5an0186aWXZPjw4QKJ3nXXXUYTZSjN\/B2ERT7CDM0Ju4WFmS5dugjaYOvWraVjx47u5MPCubXnsIx6E0gElPgC0C2pUAXIhSFrkSJFzNweq7AQHvLOO+8YbYu5PcLEffvtt8LQlrYzF8fQEhKDNJlLY8hLWjQh37Bhw+SMM84w84o2H0PQ5s2bCwsvEBkkiobJ8JT6Uc\/NmzcLmqp9xl53795tND\/m9tDkbP1sul5TBwElvtTpy6S3ZNWqVdK0aVMzr1e3bl0zxM3MzJRrr73WDAEZ+iIVKlSQJk2aSMuWLU08Q8727dtL2bJlTZ0ZCkOKXBl2QjqETaLrg8UQ+z7KhdBYyHBlMe\/u27evGYoyB8cc5Pjx481iBmRaJ2vRA9OXk046yf2YCTM3WKpUKUFzveGGGwyprl692gx9TQb9SBkElPhSpiuT2xDmyJibg0wwL2Ghg0UBNCuGgQxdSUcIEwdR9enTx8z3vfnmm\/Lcc88JBEdat27dhLyIze9uEfk+\/fRTs7jA+5YvXy7MJxYrVkwQhrfUiXk45hjR7iC4IUOGSL9+\/QSNElKm\/GeeecbM0zFE7ty5syC8i\/qxAk3Z1A9yhjwh2bFjx2ZrlqwUUx+e4Z28mzpwrxIOBJT4wtFP6VdLj1uM8TNEWK9ePWH+DpMWq3F6\/CotLgQIKPGFoJO0ivEjcOaZZ8rUqVPNcByNkdXf+EvVEsKKgBJfWHtO660IKAIFRkCJr8DQ6YOKQLIR0Pd5hYASn1dIajmKgCIQGgSU+ELTVVpRRUAR8AoBJT6vkNRyFAFFwA8ECvROJb4CwaYPKQKKQJgRUOILc+9p3RUBRaBACCjxFQg2fUgRUATCjECqE1+Y+0brrggoAglCIHDExxkKOKnE4y1nOSCEiSMtQThosYqAIpBGCASG+HAXxIb3tm3bGuePnKEwceJEQdgUDumxv3Ly5Mlp1D3aVEVAEUgEAoEhPnyx4czyjTfeEM5AwJEkXm+RSpUqCYT4z3\/+M9uVUSLASJcytZ2KQLojEBjiw+03LoVwEc7ZDTiaxBUQHjW4cg8x4rct3TtN268IKALxIRAY4uOwFjzv3nLLLeZ0K3yhcXoV3nnxoTZo0CBhOBxfc\/VpRUARUAREAkN8uCLnGD9cBzHc5UwEjhm88sorzXGDHPrCWQjaaYpAQhDQQtMKgcAQH9ocw1284HJuw9FHHy1HHnmk6QyGuNyTx0TohyKgCCgCcSAQGOKLow36qCKgCCgC+UIgUMTH0YKcycCBz59\/\/rlwZXGDONLy1TLNrAgoAnEikLqPB4b4jj32WOHAGU7Zuvjii81BMlw5I4E40siTul2hLVMEFIFkIRAY4jvqqKOEcxEwV4kkpJEnWcDoexQBRSB1EQgM8e3Zs8dofNdcc40xZ7Fb1diudumll8rgwYNl\/\/79qdsT2jJFQBFIGgJxEJ+3dTzhhBNk\/PjxMmPGDIHopk2bJmxhQ9iny9sKFy7MJV\/CSvDo0aMFe8D69evLqFGj5NChQ\/kqQzMrAopAaiEQGOKzsLJ1jV0aEKGNO+mkk4Q40mxcrNcVK1bIzJkzDaFCphhFL126NNbHNZ8ioAikIAKBI74SJUrIDz\/8IMOHD5ft27cbefXVVwWD5oIsbkBybIUrVaqU8Dzzh8uWLUvBrtQmKQKKQKwIBI74MFTu37+\/sIXt+uuvl2uvvVa2bt0qjz76qGDcHGvDbL6NGzdK+fLl7a1UrlxZNm3alH3vcUCLUwQUgRAgEBjiYyjLtjQwYwdHZmamvP\/++7JgwQLp16+foLGRZvMQjlUg02h5b7vtNslLTj31VEHc+bhH3HHRwuRD3OncI+64aGHyIe507hF3XLQw+RB3OveIOy5amHyIO517xB0XLUw+xJ3OPeKOixYmH+JO5x5xx0UKkwdxp3FvxR0fKRwpn43jGukZdxx5kEhxOePdeWyYPIi958q9Fe5zE5vvrrvukgceeMB8120cV569++675d5775UOHTqYdOKskAex91y5t8J9bhIpn43jGu13mej4wBAf29OmTJkinTt3FpwS4H\/PNp7wvHnzTBp5bHws14yMDMEY2uYlfPrpp9tbmTt3rowcOTJXwXgacefjHnHHRQpTPvkQdzr3iDsuWph8iDude8QdFy1MPsSdzj3ijosWJh\/iTuceccdFC5MPcadzj7jjooXJh7jTuUfccZHC5EHcadxbccdHCkfKZ+O4RnqGPrfx5EHsPVfurXCfm0TKZ+O45vYsaeRBWrZsKccff7z5nnNvhTws+p199tnGcoJ7t7jz2Xgbx9XGRbuSB3Gnc49k\/wh9CASG+Fix5S\/OQw89JO+88440bNjQaFn8VbjsssuM5ofTAvLkB6e6devKypUrhYURhHCNGjXyU0RceWfPnh3X86nwcLq1Qfs8+D0eGOKzUJ188snSp08fYVGCvwrIwoULha1rOCW1+WK9nnPOOXLhhRdK06ZNpXHjxoZQiYv1ec1XcAS+\/PLLgj+sTxoEvtpzUAbM2CQ1+i6S28etM3FB\/QhTfweO+LzuVMdxjPfm+fPnCwSK+3rHcbx+jZanCCQEgZbPrZRxS3dIm9pls647E\/KOdCw05YkvHTs1KG3myICg1CVw9cijQmh6aHlkW\/VgXclsnCGTu9aU43rMNRog8UGTMPV3oaCBp\/VRBNIdAUgPTQ\/CQywe9auUNOQ3YMaXMn\/jXhut1wIgEEjiW7t2rTRp0sTMzbHTomPHjsaQuQDt00d8RCBMcz4+wnTYqyE0NL16lUseFm9vIL\/MxpUEYrRxQbmGqb8DR3zY87Fro3v37sJCB6YnLVq0EI6bJC0onaz1yBuBMA198m5N4nOwiAGhQWxD2\/zP5CrnmzOzhr17BjU0Cx450\/y8j7+\/k1f7wBEfJicYHFerVs2ggNt5TFtwR0+aiYzyweIFPvxwSNCsWTNBc1QnBVHA0uhAIICGB+Exd8ciBoQGscVSuYqlikjLoStjyWqsGXr06BFT3nTIFDjiYz8txsw7duzIxn\/dunXCQUO5bVnjsKJnn31WnnzySVmyZIl06dJF\/vGPf8iHH36oTgqykdSAHwhAbAjmKAhkxXAWskPDg\/Ays4av7vm8WOo5JEsrnP\/53pjJL5Yy0yVP4IgPcsPkpHfv3rJ8+XJjf3fHHXcIW24gxWgdw1m8GRkZctppp5ksOC7dvXu3MXwOi5MCfhz8KPhBqMw1K5hBx8ESGFcrLUZtNcNQew+xIVv2HBSkwnFFhDk8tDsEwsvMGr6aL24+PipmlcNKL+THdye3R9lNwhGtueWJN03n+OJEENLCL9+iRYtk+vTpxv7u7LPPzrXUqlWrCs4NihUrJj\/99JPgww\/XVjg4CLKTgnFLd5q\/2PzA+XHwo+DHkAqyvFuGpEI7cmsDpEU6VytTbikvNuy+Tr69piDM3yG5fqFjTGSxg7J0pTdGwH7PFhiND40Nbyx4XkbYbXHllVcKHpiZ4yONPL\/XW77++muz6suWNjZZ2\/j169dL69at5bPPPjMeXRg2M2do0yNdKYO\/VokQiDdSuQtWb5BLn\/rNGr\/UUT8LJPHmjWXk7jpFJFL+MMaBdRjrHW+do\/V5vOVGe75O6e\/N94dh88T56yJ+f5gKwuNRpDLYC3\/gwAHZvHlzxGcjPRMpLj\/9zW+O\/H5JYIjvuOOOE+boJkyYIGyoxuMDHpkRhr4sVpQuXTobJ7av4biALW3M69lFjNtvv12YxB02bJig8TH8xTGBfZAwK8X2nitlsCKVCEHbzFnuESXKCsOhf38vRiMa1aG25Myj95VCi0mkPk9Gf6J5dpy0U7YdKvUH7K6++mrjgSVSPRglFS1aVE455ZQ\/PBcpvxdx\/Ob47fklgSE+Vm8hKq64nkLTK1eunCA33HCDrFmzxixwRAOKv7J4bnnhhRekQYMGQjnk9dtJAXVwC3MxzP3wJWUY5E4rQDjQj6AVBLqCKVi5+pVLyh3j1\/nSsjD1d2CIz\/YUJ6mxgosDURu3a9cuQ3qWzGy8+0p+zFfatm0rDJURhscVK1YUhs1BcFIA6TGPl5m1gueue6qG0QxStW1BgsYFBQAAEABJREFUbRdziJHMXJKxuBGm\/g4c8eF6\/pZbbpGuXbsazQ3trU2bNsbRAMPhaF845gExe+FwIhyYIgyT0SIhQ7+dFFjSQ8vLLMAKXrR2a7wikBMBNXPJicgf7wNHfFQREmP+buzYscJ5G5BZo0aNSAqluEkvlA0IWaXTvbqYuTCVgpkLdoN8\/x5eKNJ3a01hmoX7dMcocMTHyi1DVHZg3HjjjUbT47hJ4kgLY4cxvB1y\/elhrHpcdQ7TnE9cDQ3ow4wuMJfC1OXj3b9VEgcI3GMv+luMd59h6u\/AER\/DWVZ3J06cKMiLL74ol19+uVgjZO+6KfEl8ZeV1VtID3urxL8xWG8I05xPsJDzpjYLPt8btSC0QUgxaoYCJISpvwNHfCxgMC+HuQrCnl3MU3755RfZt29fAbrDn0cgPTS9Fn8uJulIev6grm91IzBuyf+2fYo74fcwmt\/vwbS7BI74IvXAli1bZNu2bZGSAhsH6eE1t+NfIrsXCmzFPaxYmIY+HjY7MEWh1SWzMmHq78ARn3tHBtbdCHN9nJNhj5jMqzM5UAhXVpRlDZvx2MJpUqNGjZJDhw7lVUSB09H0mECG9HT1tsAw6oMeIMAihwfFpGQRgSM+jsCbPHmyORIS626Es3UhMoa7efUCCyADBgyQgwcPmqwrVqxImncWSI\/hg5Kegd7sAvgtpJ9+IJCZh70o31Mv6xW+OT4vWx9HWewZZI\/tAw88IJs2bTL7cdHaPv30U2E\/Lt5Wcise5wRDhgwRtrcxT0heTmuzCyN4d6ldu7YsW7aMJE\/Fkh5ftky10\/MUWy2sYAi0qV1G2MkR6Wm0wXT+ngZK44OkRowYIZ988ok899xzMnjwYCPY8l1yySXCim+kTrRxs2bNEvYcYv5i49jRUb58eXsrlStXNqSaHZEVYDjN\/ERB5f7xKwVNr9N5JaV11V8P2+jNVrqClhv257C\/DHsbClL\/IPX5M81KCt\/LcsWPFCtZX3lpWuV\/zjBQOLxwUpCf\/uY3Rz38kkARH4bL\/fr1k6eeesq4mMLdvJXrrrtO8LRigUITZCsaAKIN8mXDhRXnc7AybPNxzcs7C8Np1PSCyvAleyUza1jR\/\/qaZnjnLgfSdd+nUzhd2x60dvO9XNu7gVjZM6ih8J21zgy8clKQn3bzm+O36ZcEhviYm4P0uDJHx04N9ttayWnAjKnLvHnzBADxzsJwGCcF5557rtSpU8d4Xkbzw7EpHlkswIRzemexaYdf877DDgo\/epBepg5v\/wAYJP+HSI0IBAIMgVs+t9LT09rC1N+BIT7m5Jjbw6kAe2zZa+sW4sgT7VvDUBgSRHA3D\/nNnDlT8Om3cuVK4bwOhHCNGjWiFRNzPHN6bAfir6eSXsywacaAIIAzA767fnly8RuGwBCfGwiGseze6Nmzp1hBCyyIATNmMAyJvfTOAulhp5eZNbx111vDhyPA\/NjhMXoXNATw5NL+\/RKeVCtM\/R044kMr6969u3CUZL169cQKdniFCxeOqYMYBuPggKvjOGa\/r1feWSzptaldNmteLyOm+qRQJm1KiiGA5tfznN9Mv1Ksabk2J3DEB+HhfBS3VM2bNxcrLHwwX5draxKciGEymt6qB+sq6cWAdZjmfGJoTspmqVLsB0\/aFqb+LuRJiz0sBJMV5vLwredhsXEVhZbHIgaFQHpcVRQBRSC8CASO+DgUhcUJVnEZ3kZb1U0G5CxeQHhoecznKenlD\/Uwzfnkr2WH59a73xAIU38HjvjQ9t58802zZW3JkiViV3bzWtX9Dfr8fx44p7053hH\/ZAxlITqE8JY9B81hQBBeZmOdz8s\/uvqEIhBMBAJHfKzcsoJrV3PttU+fPoJlONvSokG5fft24WAitqgxN8gZHHk5KfilaGnhLFuEBQuW+BHIjonfaO\/S+LwRCNOcT96t0Rx5IRCm\/g4c8bFy+8MPP5jDhTghjeEubqlY9MBAuW\/fvhHx58zQJ554QjhGj6Fy586d5ZVXXjH7crHnmzFjhkybNk3mzJkjbI2zhRSbP1A4kBlRrc6ioldFIE4EAv544IgPgkPrg8TwyIIB8jPPPCM\/\/vijdOnSRfbs2RMRUrQ90tjx4TiONGnSRDCIXr58uaAB4tIqkU4KIlYqKxIX+lmXtPwfpjkfLzsoXfs8TP0dOOLjC4hLKYiOMIL\/PDZSo9VxH0m+++47YRjM0Lh69epy8803y969eyVWJwXs+U2ESNa\/RJQbhjIhgDDU0+s6ZnW5eF1mIst74403zEgIz0XxvCe\/\/Q1OfkngiA9zFnZbsKp73333mZ0bt956q9SqVcvM8ZEOWOzuYEcGHYWTAs7ixaXVbbfdJmvWrBGcl95\/\/\/2GDHNzUsAWN5UvzJ5nxSE9cRgzZoyMHDnSeEVK9neA37If4g\/x5dHS9u3by+jRo4VhKzs3Bg4cKK1btxaGvg8++KB5ml0ZbicFpUuXNuRoHRAwvC1UqJCULVvWrBCbh7I+vHRSkFWc\/lcEFIEQIhBI4sN4efjw4fLee+8J3pf5a9SuXTsDL4sfJpDjIyMjQ1gU2bBhg0mhDOb00ApxTMBWOISwF04KzEv0QxFQBEKJQOCIj7m8wYMHG4ehLFacdtppcswxx0iHDh0EG79oKENyDHnRCNEShw0bJgyV69SpI5Cfl04KotVB4xUBRSAcCASO+PAEC4m1atVKMGc588wzpVu3boL2h8aWG6zknTp1qtESMXjGrshxvHVSkNv740vTpxUBRSBZCASO+HBEwKrurl27hHm8zZs3Cx6V9+\/fbzy2JAsYfY8ioAikLgKBIz6r7TFUrVatmrz00kty0UUXGRK0K7qp2x3aMkVAEUgGAoEjPhrdsGFDwZ08pipvv\/22sHrbq1evw87cIJ9KyiOgDVQEEoJAYIiPszaw3bPeWOwVo8grrrhC2rRpI+RJCApaqCKgCKQVAr4TH9vTMEbGABmXVEWKFBFMV0aNGiUTJ07MFlzR61A3rb6b2lhFIGEI+EJ8bEFj9RVtjkOCrrnmGunUqZOw7QxbPLywcJwk5imrVq2SEiVKGFMWFjkShoQWrAiEBAGtZvwIJJ34sNN75JFHjPeVt956S9w+9\/CqAiEyzMUrC66oiGMHByYtaIfxN1lLUAQUgXRHIOnEh7kKRsa33HKLsIKbswOKFSsm7M0dNGiQ5OaUIOdzeq8IKAKKQKwIJJ34GK6y7YyFikiLGZiuXHnlleZ0tUcffVTYeYEPPeb4ihcvHmu7NJ8ioAikEwL5bGvSic\/WD\/94OBJgtZZdFkOHDhWMlFnkOPnkkwXXUpi0sK+WuT+IkrlB+zzaIP72PvroIxslc+fOFYbJ559\/vvHEjI++vDwwZz+sAUVAEUgbBHwjPlZwt23bZvzmcZxkmTJljA8zkMeRwOLFi4VFDyvM8bF3l\/RZs2ZJ27ZtBa\/K3COkvfzyy8JqMI4NLr30UkFLXLFiheTmgZlnVRQBRSC9ECjkV3MxW7Fb06gDDghwP4XfPQiMIa89aIgrWiF5yHv88ccbUjzllFO4zRZWhHE+SgRlM4eIm3k0SzRM7nG2uGzZMrKoKAKKQJoi4BvxQUI4IsBTcoMGDcwQFRMWBKJjqButT2rWrCmXXXaZ4IPP5sHGjzlDnq9cubKMGzfOGD0bD8zly9tsxuvLpk2bsu9vH7dO8isDZmwSt2QXpgFFQBEIBQK+ER\/ooNWh3d1zzz2C\/70JEyYI7qNY1e3YsSNZYhY8x+K8lAOFcDaKDSBzhMwF5uaB+Y1p8yS\/8tSY6fLEm0uzheMoI0npzm8IUq7ZXWYYzxY8lVMVi1ODhQHeyvFcfsYZZyS9b2L+gXuc0VfiGzFihEBQzz\/\/vGCj1717d3MS2pFHHpnvZkJ8J510klSsWFEcxzHemHfv3i3MHUKEtkDC1kszcZyy9vXozpJf+eb5a8XKnkENzfm7Oa\/3XFVbjvpqodRq3lb2XjnSSPH2Y6TTsDnq6v2L1HXzzveK72NY5NprrzXezpkWSmadwckv8Y34WIxYvXq19O\/fX0qWLCmVKlWSIUOGCEbNpEk+\/1WoUMHs5cWdFY9yuhpzgjghZbEEX34I4Ro1apAl4ZLZOEMWDL1TOJ\/XkmKb2mVl3NId5hDzhFcg1xdoYqIQmD17dqKK1nI9QsA34sM0pUiRIlK0aNHspjDvh40fw9PsyBgDVatWlXbt2hkzFsxZXnvtNenRo4exA4T8GEI3btxY8PzCYUYxFut5NsgQ8vvq24Oelx20AsN03GDQsAtjfcLU374RHwsTaGSYn3CW7vr166Vfv37Cii0LFXl1PDs8mBc866yzTFbHccxhRCyMLFy4UF5\/\/XWjRTpO8DwwVzyuiHy1J\/WJz3SMfigCAUTAN+JzHEfuvPNOs8qKhsf+Xez5WOgoyBxfALHNs0qpTn5MX+QJQrAyaG3iQCBM\/Z104mOIyy4MXFEx59akSRPhFDWGpjfccINxL0+eOPAP\/KP1Kpc0dUx14jON1A9FIIAIJJ34WLhgFwY7Mi6\/\/HJjv4c9H8JuC05YY+taALHyrEoMdSlsS4rP84Vpzof+UIkPgTD1d9KJj3k9dmGw3Qyiw+4O\/3vIpEmTDPIscJhACn9Afqmu8YVp6JPCX7W4mxZrAWHq76QTnwWRYS6LGhChjcMOjzjSbFy816A6KVj1YF0ZMONLqdF3Udb1fztJ4m2vPq8IKAJ5I+Ab8eFVmb21rMziRQV59dVX5ccff4zopy\/vpkTOEWQnBZAfpi0QIDs\/IMHIrdBYRUAR8BIB34iPbWQYL7Oiyx5brMe3bt0q+ODDWalXjcQaPahOChjuYtfnNm6GABFIkP3AXuHgRzlhmvPxAx\/2iLcculLo70jCd2D+xr1+VK1A7wxTf8dNfPlFiKHszp07zWM4Fs3MzBRs73AlhR0fXlRItHkIxyN5OSmIp2yvn7UkOLlrTUETXLDx28N+FPwQEH4sCD8cBIJEvK6Plpc\/BOgDpPesb7IdX9BPCP3mJjfuF3y+VyocVyTidkf+GPIdaPncyvxVQnPHhEDSiQ8bvSlTpkjnzp0Fn3ucwWFrSnjevHkmjTw2Pt4r2mVuZeA4gL9WiRC02PyWe\/IR30rrqr\/KM81KyvJuGdny5o1lpGmVIlLqqJ+NrN++V5BXF20R5MzeH4hb3D80P8LnPrvpMOL2ow6Jfqcbb\/oAWbx5v+kX+sb2FX2Xsy+Ju7tOEYn2\/eA78MLVZQyGE+evi5ov2vOxxvO7O3DggGzevDmud0jWv1jfyW8uK7tv\/5NOfKzYdujQQR566CF55513zBYyQEBwNYXmhzEzebxAJSMjQ3BMYMsi7HZSQDwbs1mRSoSUL1\/e7CDxquz+19eUUR1qG5n597qCrO3dQCIJWoNKw6galRfYRMJ92m2nmH6hb2xfFbT\/r6l\/uql\/x0k75YgSZT39Ltk6sQuKraP4t7Rxib7ym+O355cknfhsQ3Ev3+asF9AAABAASURBVKdPH2EODhAQtpr17NlTTjzxRJst7mvdunUFxwSsFCOEE+6kIO5ap0YB\/PVPjZb434r6lUvKHePW+V+RXGoQpv72jfhywc\/TJBwSBMlJgaeNC3hhaA0Br2JoqjekzekyP2tOkDndoFY6TP2d8sTnOMFzUhDUL67WK7gIYAHA0Hzc0t8WBoNb03DULOWJLxzdkA611DZ6gQAr\/izYsCrsRXnpWoavxIezgvvuu0+w42N+D0ekrDCla2ekWrvDNOcTFuzrVykpGL5XLFXErPYGiQDD1N++ER+GywMGDDBuqfjSsfr5008\/yRNPPCGkEacSbgTCNOcTJqQZ9uLVGwLE1s9qgNgQ+tmOMPW3b8SHl5ZffvlFLr74YilUqJDgjRkTFlZeSfOzA\/XdikCSEIjrNRAgRu+WAO3WR78JMK5GJelh34iPbWlHHHGEYDhp28rQF9LDyNnG6TW8CIRp6BNelEUsAbL4kdm4kjnTxY8hcJj62zfi43yNRo0aye233y64nee0NXz0XXfddRKL63mGww888IB89NFH2d\/ZuXPnGv9+nLmBU1McHwTVO0t2pTWgCHiIQGbjDDMHaIfAbJfzsPiUKco34gNBDgD617\/+JU8\/\/bTcdNNNQpg40nKTWbNmSdu2bQWffjYfmiJn9HKGB7s\/8PX37LPPSpC9s9i6p+o1THM+qdYHlgA51CpZ2p9X\/Z2MviiUjJdEewcaWq9evaRMmTKC52Xc0L\/00kvRsmfHcyAR2iFbbLIjswK4udq7d29WSOTgwYPGvRU7Q4LqncVUVD8UgQQhwBAY8xdWgJNFfglqiufF+kZ83333nTkJjaEum6NZ5ECTQ0NDe8utpTVr1hT29XJSm83H8BizGIbKlStXlnHjxkmbNm1k48aNwoqxzUfapk2b7K3RNB9++GHJTZi7QNx5uEfccZHCaKHkQ9zp3CPuuGjhSPmIQ6I9444nH5JXnDvdHc75LGnEIYTzEvIh7nzcI+64aGHyIe507hF3XLQw+RB3OveIOy5aOFI+4pBIz9Dn7njyIXnFudPdYZ5F8opzp9vwiMGPGWcXJ\/xJJOPuadnfc8pDyMdv7rPPPhOsLLjPKTafO544xB0XLRwpH3vzs3+EPgR8Iz60MzZH43V5yZIlglbGgsdRRx31B3MW5v8Aiq1nHFIUCSf2+o4ePVpwZY8jAp558sknTVm5eWfhkCOcIuQmlSpVMpvD3XkixbnTbfjWW281z5LfxnHlHiGcl0TKRxyS17Okkw8hbIV7xN7ndo2Ujzgkt+doO+nkQwhb4R6x97ldyYe483CPuOOihcmHuNO5R9xx0cKR8hGHRHrGttumkQ+x91y5RwjnJeRD3Pm4R9xx0cLkG3lrTSlZsqSsOukqIR9xCGG2dVarVk1wEcd9TrH53PHEIe64nO22aTnzEc\/vNdLvOFlxvhEfHpjxtvy3v\/1NFi1aJHhR4cwN4lj4cAMAgQEULquiOTAgHRKtWLGiOI4jtWrVkt27d5thNERoyyOc0zuLTdOrtwjwhfe2RC2toAgw7MXshf2+idr2Fqb+9o340ML69u1rhqMDBw6U4sWLmz7lrwGan7nJx0eFChUEc5hdu3aZp5YvXy6c54GWiEcW7AMRwuqdxUCkH2mIQGbjSsZJaho2\/bAm+0Z81AKCO\/vss4XFChYjGjRoIJAgBEZ6fqRq1arSrl07wYwFcxaGsD169JA6deoI5MdqcePGjY3\/P1T7\/JSteQuGAHM7BXtSn0oUApmNMwQXV4kwcwlTf\/tGfNjhofG1aNFCLrnkErn66quFczdY4WUuIq+OZ36Qg4rOOussk9VxHKEs3Niz7\/f11183c2uOkwbeWQwC+qEIxIYA290Y8s4P0XkesbUs9ly+Ed+3334rrN5ii\/f3v\/9dhg0bJmPGjDFmKI7jxN4CzRlYBMI05xNYEBNUMbS+gTO+9LT0MPW3b8TnRpwtaszN4ZX5m2++MYToTtewIqAIeIvAvVlzfems9flGfNjgMazt37+\/GZKOGDHC7OBgmxlzf952s5bmBwL+zvn40eLwvLN+lZJmrs9LrS9M\/e0b8TmOI\/jiu\/\/++40NX5cuXYR5O9xS5TRnCc\/XSWuqCIQHgTbnlTXu7MNTY+9q6hvx0QTcUmExPn36dMEBKSuzOCzg7F3SIwlOB1ixPe+884TVW3Zq2L80aIus6mIM3bx5c1m7dq2QH8Nm8tevX19GjRolhw4dilS0xnmMQJjmfDxueiiKa1O7jNH6hn9c2JP6hqm\/fSM+yI3taoMGDRKcClhhFwcOSaP1BFvQcGYwefJk8xyrwS+88ILZoYG2yP2HH35ozuZ95ZVXZNmyZTJz5kzj0IBdHXPmzDEnu0UrX+MVgRRG4A9NY4V38e6if4hP9QjfiA9jYoa2zO099thjYoVtM9aYORL4zA0yPGaXBumsDBNG2yOMqyvHcQSHB7itwpAZDbBUqVLGaUHt2rUNGfJsMoQ9yMl4TxDfYTXxINYtkXUKY597QX5h6m\/fiI95PBYx8musDIFh9Dx79mzjtp5hLw4LcHqAptizZ0+pXr263HzzzbJ37948nRRI1j\/2ASdKsoqXRJUd9HIhgKDXMRH1C1ufn7jvY3l\/\/W5BKYgHj\/z2Nzj5JUknvn379hkvEI8++qjZYnbFFVdIp06dBMJC8BBBHjcgOBygQ9iBYZ0UYPTMvls0RJ7ZunWrMF942223yZo1a+TGG28UNEPIkO1x7vLcYfb4qnwhikH6YvDpiDtk1ZC\/yieffJL074H7t5jMsJv4kvLeY445Rtq3by\/du3cXdm7gigoS5B4hjTzuyridFLD40a9fP5PsOI6ce+65gqeXokWLGscE1gEBw1vO8ihbtqxAkOaBrA\/CNk\/Wrf5XBBSBNEQg6cTHORsQFb67cCbK4objOILXFQTHAuSJ1hcMjz\/++GPB0JkVW+bw8LfH8JZyN2zYYB5dt26dmdNDS8QxAXOKCOEgOCl48cUXs93ms9J8wQUXGDf81NE0QD9SBgHmn\/ENyVDw3XffTZl2hbkhSSc+VnPR8OrWrSuYmbCQwcIGe3djARIHA82aNZOWLVsKZUyYMEFwRsDcH0PiBx98UOrVq2e2wGEnWCdgTgogZxZd2GdMe3fs2GHOHJkxY4bgROG9994jWiWFEMBcq3PnzoLbtfnz54v+cfO\/c5NOfHQ6Q9lWrVpJRkaG\/PWvfzVfhJzzetGgcRzHeE3GEQFmKyxulCtXzmQ\/88wzZerUqcbMZfz48WZHiOPE56TAFOzhB\/ONDNWxM6RYNFdcaqHJYge1bds2olVSCAEW8OhbfFDy3Wefego1L5RNSTrxgRJzb1ytcMRkrBqffSZVrjheZX7StgcN0Ib1Gn4E+F5jMF+kSBHTGL7rWCCYG\/3wDQFfiM+31nr8Yr7UDFvdR1wyn8NuEnaKcAYIq82kY4LDrhJ2qLirgat95iptHKvXNqzXYCPA\/DJz1baWWBBgYYBZCLuEWLhjvpo+5rtCPsy48D9JWMU\/BHwhPk4+YzGCHzlzcAxZuXLPYoQ1WfEPlrzfzJc65xGXfLnZPQLxsQMFkxp+CGzFW716tYwdO9bsR3aXjkE2K9XMfbIwg4cad3ogw2leKaYnmJdm5xEanIWDuTz+0DENw1QLxvkY9XIQFn2LgT35mde2z+jVHwSSTnys3M6bNy+qvRBp5PEHjtjfyl9tVqXdR1zyxWaoColTEqY2zO9w9gf3kQQPNZz8xsIGRtmY4UTKp3HBQQAtjjNdrrrqqsMqxXeXhbfChQsLf8CYe8a2lL7F3ySOdvlusGPpsAf1JukIJJ34kt7CBL0w0hGX+BRkWMMENq\/lLztDm0jE16dPH8F7tOM4xq4Rz9FDhw41Jjg8qxJcBLAgYLcQf9hsLZnCYNECDZ44x3HMjh2OToUE0QD5w8ZzpKv4i4ASn8f4s2qLRuBxsVpc4BEQod\/5wxeCqqZ9FZX4PPwKMPzFXMea5nDlECWGsx6+RosKIAJo+QxhWdyieixYMb9XrVo1blUChoASn4cdwtwku0hY7aNYrniOYWjEvUrqIuA4jjnBj3k+Vnexx2Tqo0qVKqnb6BC3TInPw87j7BB2kXBoEosU48aNM7tKiPfwNVpUQBHAFRp\/5DBlwXQJhxkYpwe0usmoVmDfocQXR9cwtGHrGYsUthhW8pjIXrx4sTFf4d6m6TW1EMBDEA40bKtYzcVbEH3PYlXDhg1tkl4DhoASX8A6RKujCCgCiUdAiS\/xGOsbFAFFIGAI+El8AYNCq6MIKALpgoASX7r0tLZTEVAEshFQ4suGQgOKgCKQLggo8QWsp7U6ioAikHgElPgSj7G+QRFQBAKGgBJfwDpEq6MIKAKJR0CJL\/EY6xviRUCfVwQ8RkCJz2NAU704DnTCuzSnwlnhDJGwt5vdFi+\/\/LLgXortZjiadbeJdj\/\/\/PPuqOwwzggGDx4s7NHNjtRAoBFQ4gt09wSzcngfZkuWFdzv25rilYQzJux9GK6Q3VtvvSUF9ZXHQUK4o8KTeBjaq3UUUeLTb4EnCKANde3aVVq0aCGLFi2StWvXyhVXXCFs2OfsEbQiXjR37lw5\/\/zzBW0Rhw69e\/cWjhpAy+JKHrQtNCxIlFP0yEs5jzzyiNHISKdMXP\/jDKJDhw7mpD40roEDBwpxOAnF6zGEdtdddwnHAlA2mhlC2ApedHAYi8NQGxftinZLfRCc0VKH\/fv3S4MGDczxkRwfGu1Zb+O1tHgQUOKLB700fbZjx47GuzBnpHCIEocpAQWu9\/FIgw+6IUOGCB6l58+fLxDPoEGDZOPGjTJy5EiZMGGC4L4JokHb4tlIwrPr1q0TPBd\/8MEHgs+7t99+22TFpT+aJ3nQtpYtWyacebFlyxZ57733BHJk+IqPxJ07dwpHAuArcdWqVXLxxRebMuwH+c844wx7a67uNtLOSZMmmXi0WzRd2pmRkSGcu0G9aAvtxxWVyagfgUZAiS\/Q3RPMyr3wwgvZZ6ZwiJL1TnPRRRcZ1\/mbNm0SyAGtCNJo06aNrFmzxmiB+CeEJBzHEbyb4OEmWishLggGIuVcEoh05cqVJjvaFr4P8YjC+xlec8gPpMYZxbiHYs4O7ZLDnlZlER6HOvG+nD7yIETiTcG\/f7jb+MUXX8jVV1\/9e4oIB0NxdEDTpk2F+U4SIF+OkNSjI0Ej+KLEF\/w+Cl0NGe5xxCJkB2lwwtjkyZPNCXMMX22DOFOY4am9z3llCHnvvfdmkyzaX++soXHOfPaesnD9b+\/37t1rSKpRo0ZGw2SYXbduXYEYbZ78Xqn\/q6++KieccILceuut4jhOfovQ\/IlDIOaSlfhihkozxooAk\/243YfwIIqpU6fKU089JTjlZCiKd2Li0QohSRy1ornhpp945tx4F9oUWhwaGVpWr169hCEvaZEEsmUIDQFyzGO3bt2EA4Bq1Khh5hGZe0QLzfksRwMu9WnmAAACEUlEQVTkNuR252c4TXsY4lJnm4bGSd1LlChho\/QaYASU+ALcOWGtGkPZe+65R1igqFevnuCYtXPnzsKQk0UMho0XXnihQIAMMRmWcs\/xi6ysMhcIGTIUhsy4MozmfFrC0XBp3ry50cBYaGjdurW0atVKGA5DRtWrVzfzktznfJ53ME8JceVMc99DqGh7X331lbC4AolyZb6ReUTagibofkbDwURAiS+Y\/RLYWuFxOBL5QGyIrThzayxKoLGNHTtW8ETtOI5wFu2SJUvMHCBDRcjGcRxzxCZzejNnzpQXX3xRHn\/8cUGjQmsjHmHYSxzvpx72XbyXOIinf\/\/+glkJ7+VdjuOYFV2IqWXLloYY7XP2ygowmiFaImVQX8qz6Vx5H3UhjXlGtFUEUofsaCtn6sYzjOY9KslBIJ2ILzmI6lsChQAmMhAgGmStWrUi1g2yu\/nmm4U5QIg4YqZcIjHVOXDggKC15pJNkwKEQKEA1UWrkmYIsBrL6mgim83Jd1OmTDEaZG7aGKvE2OQ5Tv4XK5jTZFiPNprItmjZ3iGgxOcdllqSIqAIhAQBJb6QdFSiqqnlKgLpiIASXzr2urZZEUhzBJT40vwLoM1XBNIRASW+dOx1bXPuCGhqyiOgxJfyXawNVAQUgZwIKPHlRETvFQFFIOURUOJL+S7WBioCXiCQWmUo8aVWf2prFAFFIAYE\/h8AAP\/\/XPhkBwAAAAZJREFUAwBmh+Yqn8pTCAAAAABJRU5ErkJggg==","height":191,"width":318}}
%---
%[output:5fc65039]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAT4AAAC\/CAYAAABno7D5AAAQAElEQVR4AeydCbxN5frHn03dRImTpAwhhFRXZahQbgOS0K0bbtNtjkbkdKmbBoRGQ1QaNNCcckOjcFSuJCrKEBmra8pVJOl\/vm\/e\/V9nnbXX3uecvfdae5\/Hx3PWWu\/4vL+11m8\/7\/SsMr\/rP0VAEVAEShkCZUT\/KQKKgCJQyhBQ4itlN1ybqwgoAiJKfB5PgQYpAopAdiOgxJfd91dbpwgoAh4IKPF5gKJBioAikN0IKPEFdH\/fffddqVu3rowdOzYlGnz++edyzDHHSN++fZNTvk8p27Ztkx49esjJJ58sP\/zwg0\/K1EWBI+2l3amrJfkloy96I5wnWkMYMHfqiu60gefaGR7WcyW+sN6ZJOsFIUFMEBQvTZKLL1BcOuuiYl66MWPGyH333SdHHXUUQTGlqLrxIqfqB4r7MGTIEKlUqZJMmjRJDj744Jh6hz0C3E8\/\/XTp06ePcD\/Crm9WEh9WDg+rFR5ebgQPGi8+woNm47EWiLfizA9Z8LIQRznk4QWjDM45Ui7xXkLZpEP4RYz1UFAndT333HPGEqQuyiM9+ciPUJ4z3NaPDpxThtWXdAjX5513nqxevVo+\/vhjufLKK4X0xFmhPmf5xLvLQ0fSWCGPzW+P\/\/3vf8WrLnRAN5uXsmwejlwTR1vBl3PbVuKpizArXBOOjB8\/3pDH0UcfzaVpG7rbtNRL\/UhRdKMOsKLQYcOGRa1zwm3ZHLkmjZdQJ\/WTDuGcMPClbO4H96VLly7y\/fffFyqCtOQhLwJO7kTvvPOOeWaId2JGHU4cOCeM\/OhMep43yrflOp+3q666qpAl74wnv7O+iy++mKKF+2FOQvwn64iPG\/Hqq68WgJwHjBttA3nYevfubS+Fh9rG8wA48\/NQ3njjjeZlshlGjx5tCIRrynr22Wc5LSSUSdk24n\/\/+59ce+21MbuD1HXbbbfZ5CYd6clnAymPcu11so6QRs2aNWXZsmWmyJ9\/\/lnWrl0r1atXl\/Lly5uX3okLidy4EuYlP\/30k4Ah7bPxlMW94poj15zTVvDl3ArtpS57zdFaFhDD3Llzo3oSN3DgwOj94Zp6qZ82ce2UeLo503JeUBdCJKaVA8lQL\/X\/kVLMjw\/ku2HDBhsU8+iVH5zAy2aibOczg+ULORHvxoFnlTDirJCXMrgGS+fzBqGShziEci+44ALhHnGNOJ9HLFasV+4HZREfVskq4gPsiRMnyv777y+vv\/66fPPNN9KvXz+D\/bRp08yRP7Hiyc9Na9mypSxcuNAI59x8hLwIYcQ\/+uijXMrMmTMLECOBPLRPPPEEp0I6dDnnnHPMg09eE+HxB31Je9pppwkPOQ8l+QijHLI428J1PKlataq89NJLArGhO+Xst99+BbKRplmzZkL7wQEdqbtNmzYCYSSCKwUedNBBheqKRCKGRKmbcsESXcANK4cjedGLeNJxbYX2uu8ZLx+WBfm3bNki6EmbwH3dunWmrdRjy\/vyyy\/NC+vGIRKJrRt6oBN6cF+uvvpq8dOFdE6hfoRy0APhHFz5gaFsrsECkqG76JefZxocwAvCtmkpx5YNLmASCwd7f21e6ifvvffea553dLNhlGvTcZw9e7bBkHDn88hzTn38QPJDyf1AB\/KEVbKK+AAb0I888kipU6eOwRzS4MHihttfWWf8SSedZIiSl2XlypVCfh5WLCCEc1OQ4499yfiF40F0REVPIQssJuqmHCJ4uHhgIDWu3UJZ6GPDeTk4b9++PQchH\/kpxwQk+U+9evVM+8GRuq0+XIOLEzcvXGOpA6nOmDFDJkyYIFjHvFi8YKTfvn27IUWLE+R16aWXEmWEF4p7wwvduXNn06XDyjCR+X\/Qjbj8U\/Of\/Iceeqj5gaEesKe8BQsWeI7\/+elmCnT8iaeLI6k5BUNO7POCbuhCGATK0U\/c+SFG2gGOFSpUMFmduNFuE5j\/h7pIB+4QGzj4Pcv5WaLWvtUX\/BDiEKsP1jfdXI6EZ6JkFfEl6wbwUkMwToF0klV+WMuxpPv2228bK9ZJdCXRGcJgfImXBXLlBeSFLUqZpCef857wA2BfRmdZhPOy25eWF5RxLCxZZzrOi6NbLF0oL0zCsA2Yg5ETj5LoyI8hlqfzPkCwEO2eH3sz3opRUJJ6Up03q4gPsBljoFuzYsUKg53tLtKNq1KlignjBUK4oLuExcCvZe3atc1Nwzr0eklIn6i4zX7nC8Y4USLlQBKk48HlyNgODzIPNNcIViUPHO2l3YQVV7CSIbt58+YZK8z+8ieKa6x6wRqhu8gPCC+htfhsHq4JBye6TjacF4p7Y+NtuD1ajOw19w2Sg+xsl4x6Y+VHL4Q0sXSzZcfTxaazR6sbXVPahdi2WSvepvU62vz2\/vPccP\/5EXF2db3yMh5H95kf8avzu+hFeT6svuCC2PLRh3eFLq8Ncx6xvukZ8K5hSTvjwnaeVcQH2N27dzfjEO5ukftB48XgIYIYuSnEk5+bxkuCtUA8wovEC0W6RIWXBOLgQUEX29WgXCSRcrDA+IWla4ceHLlm9sySkdWVOqgrXrk8yLSdl9Cd1upMGh5g6icNuCSKK+mtUA51VaxY0QwnoD\/tIMym4ceIsrkm3OLEtRXuDefEk98KRAAOYEI8gq7cQ+qmLNJSL2lISxqEeMrz0410ViiDHx4\/XWxae+Q+I9SFLgjnWIyc23SxjqQhLc8o7UBf0tJdtl1drr2EtmIE2LyJPB+QJPWhI3Xb+mz5PA\/gCBboYwVcSIMRQZ38iHAdZskq4gNoft24gZxb4ZefX3N7zc2966677KWZALHxdJOc+bnRo0aNEl6oaIYET9y6UO+DDz4oEEwiRTCmw9IOZ1quCUcfZuRsXK9evYSXzF67j9YCdYe7r+3DjeWHBWjj3W0h3I0rYYi7rsaNGwtrvIgDT7pG6IqFiiXCTCHXxIOR894Qxr2hLs6t8HIRbl9wa6UQz8ylLY9rxOJWVN0oH50pA6HOWLoQ7xTuM\/ebNtlwzplg4f7ZsFhH0vDsOeu37Y6Vx4aT1\/6gEIbOPNf8UGJZE+YW8vg9Uzx3jNE69aFMng1rYZKfctxlh+06pcSHBXLRRRcJv8zyx78CfydNmmQGq+0vR3EsqwIF7rmAvJxjEDyse6KiB15Em4YbF43IP3HmZzCZG54fHJ1csOkJJ54XmYecNG5xlsVAs30o0In6bVmkoyzKdJZh05EW4drGc04YwvIO9LB1UA7lUS7p0Y940nLkmnC32HxeaSiL\/Faon\/yURXpbt70mHeFc27zoBCkRzjnkimWBlcHLSRnz58+nWKFrZU7y\/1AX5VmxuIEnL7jt8ucnNT8slG\/TciS\/O440frqBBYKelGHrpCyurdhwyncL+tEmm5ZzwkhH3ejgDCPcKc76KcPW5ZXXYox+lEFa8iCEOeO5Jpw0pEV4T7kX3B\/I8YwzzhB+nJip5QeDNG59KNMZTrlch11SRny\/\/\/67sAQi1ngAwKxZs0ZYs8UNQPweANKrZB8CvMAMCdAyXjp+BOmeYVVgbREeT7BS6ZqTL15ajY+NAN1bLFJ+hDi33WPGWLlPsXNmXkzKiI9fijlz5kjr1q1jorJ+\/XphjCdmgiRHcPPi\/cImuUotLgEEsDr44XMKVhbWRQLZzTIV0lNOIuk1jTcCWKIYH877wLm16rxzZWZoSoiPgXPGJvgFj0VszEqxPosxgfr165vxqcmTJwuWYmZCWXytNacioAikF4GUEB9dXEzm5s2bx2zNzp07hZmpAQMGyFdffSVDhw6V4cOHy6efflooD90flboFxkMVD8Uj058BJrYKvexpCkg68S1ZskRmzZplNsJHIpGYzahcubJgFdIVLlu2rNlydPzxx5stU16ZMLkzWWhTJuuP7tnQhmxpRzbci1NPPZVmBCKexIeHjW7duhkyYuA5njg1X7VqleTl5UmLFi2MhcKAM11e56Jb0i9evNjsYf3ll1+4jAqD2tELPSm9CGjLFYEUIuBJfIyzQUg9e\/aU3NxcXyGNUz+ms\/lVtcI6H5YpuAdImWhg6wszSL\/99pvZIsVaIBafOsvLlvP33nsv45uSDW3gJmRDO7KhDdyLoMST+CKRiPFugSnasWNH8RPSJKo8ux9Yq8d6IcYAmdjAESOTG4z19e\/fXxo0aJBocZpOEVAEFIFiIeBJfPvss4+0bdtW9tprr7iF4obILxGWHlYgaex0ub1moSReKrAO6R5TJ+lUFAFFwAsBDUsWAp7ERzf3scceE2Zl7c6LX3\/9NVl1ajmKgCKgCASKgCfxYcWxpu6NN94wXV68FR933HFy5513ClPQjAEGqrVWrggoAopACRDwJD7KY4lJo0aNZNCgQYKboscff1yY7e3QoYPZs\/raa6+JWoEgpaIIKAIBIlCsqmMSn7O0vffeW5htHTFihEydOlVwPYPXic2bNzuT6bkioAgoAhmBQELEh6XHkhQWG2Px4VED4jvwwAMzopGqpCKgCCgCTgRiEh\/WHJ+ewzUN3i+w9Fhu8sknnwgztU2bNhW6w87C9FwRUAQUgUxAwJP4WG\/XpUsXGTlypHCcPn26MKZ35plnGl9nmdCwPTrqQRFQBBSBQgh4Eh\/bxljOgi89dmbgiLBQTg1QBBQBRSBDEfAkPjwn475748aNcZuFdRg3kSZQBBQBRSBECHgSH\/rh0faee+4RxvX8hDSkV8kcBFRTRaC0I+BJfHR1b775ZmEfLhMbfkKa0g6itl8RUAQyCwFP4tt3333NXl0\/5wTOuMxqsmqrCCgCpR0BT+Ir7aBo+0shAtrkUoWAEl+put3aWEUgHAis2rRDdlU5QvKWbRHO062VEl+6Edf6FAFFQGYv3yLbWvWTsx+er8Snz4MioAiECYHs1SWuxceaPtxSNWzYUPCe\/P7778stt9xiPLVkLyzaMkVAEchmBHyJD797DzzwgHE\/9cwzz8ghhxwi9erVk\/Lly8v9998vu3btymZstG2KgCKQpQj4Et+GDRtk0aJFwpq+ww47TMqUKSPlypWTSy65xDgkxZFBluKizVIEFIEUIhDEhIazOb7E50zoPOdbHHvvvbczSM8VAUVAEcgYBHyJr0qVKtK4cWO5++67hQ8C8RnI77\/\/XsaOHWtc0leuXDljGqqKKgKKQDgRqJVTLu2K+RJfJBIRJjbwznLhhRfK3LlzjZsqurh0f7H80q6xVqgIKAKKQAkR8CU+yt5vv\/3k9ttvN2N9c+bMMUf89Km1BzqFRAMUAUUgAxDwJb6tW7fK0KFDjYcWyI8Z3jvuuMNc47EFD81r164tcTNZMmM\/Y1niwrQARUARUATiIOBLfORlZvfNN98USNBe8+lJwj\/++GPp2rUrwcUWlsxMnDhRcHpa7EI0oyKgCGQUAqs2bQ9UX1\/i4+tqO3fuFD40NGrUKBk8eLA5f+SRRwQPLsOGDTPu6UvSgi+\/\/FLoQvMho5KUo3nDi4Bqpgi4EVi9aUc0KHSTrMIobAAAEABJREFUG3RB+cJanTp1okpywiJmwol3xxGfqGzbtk0g1CuvvFKYQU40n6ZTBBSB7EAgCNIDOV+LD4ekTG7QpaVLSgaOWGicY\/UtW7aM02IJXdyaNWtK8+bN4+avW7euWTS9YsWKjDyuWbMmI\/V24p0NbaA92dCOTG5D7SbNJW\/5FvPO16qc\/qUsVOxLfBDb9ddfLw899JC0aNFC2rRpIyeccILceuutwvKWlStXml0dFFRUWbJkicyaNUuw9iKRSNzsrCPEusxUqVGjhmSq7lbvbGgDbQlFO\/J7UehSXMnkNjw76e3o+35SvWDWAvsSH9o1adJEpk2bJnx1LTc3V4YPHy55eXnSoUMHadiwobz99v83gvSJyqpVq0w5ECrW3KuvvmpIsG\/fvokWoekUAUUgAxHAJZVV+6TDK9nTtB7jEh9dW8bycFDQrFkzOeKII2T16tXSo0cP4YNEFSpUKJbCp512mtkNgiWHnHPOOWbi5N577y1WeZpJEVAEwo8Ae3Qnzl1vFC3z8wZpVS+kxDdu3DiB8Fq2bClWIKnatWsL43+mBfpHEVAEFIEEEJg497uo49G9NnztypG+S1+L78cff5Tp06fLhAkT5IUXXjDjekuXLpVrr71WjjzySLOkJVmqYulhBSarPC1HEVAEwoXA0LdWytC3VhilmM0t\/+kT5jyIP77E98svvxidGICtVKmSbNy4UbZv3y5nn322zJgxQyBGk0D\/KAKKgCIQAwG6t0PfKkh6o7o1ipE6PcG+xMesLo4IWLIC8THWh6AapGeJkWsVRUARUATcCEB6fFfDael1b3ZIYGN7Vj9f4mMdX\/fu3c2SlXXr1snhhx8unTt3NlKtWjXJycmx5aThqFUoAopApiDA19POHj1f\/nz3R9ExPbq3kF5uu9qBN8OX+NCOZSt0axnTwxXVfffdJyNGjJB77rlHsAZJo6IIKAKKANYdXVoIDyvPLlIGme7NqskbPZtKGEgPfXyJD8cE7NP9+eefDcnR9WVP7THHHCMvvviisOWMQlQUAUWgdCLgJDusO7q0TsLDynsjn\/BGd28knIcFJU\/iYwKD2dwpU6bIyy+\/LBzx0GLltddeM+EQYlgaUkr10GYrAmlDAJJDWJLSa+Jiyek93XRlvcjOWnif3XpC4ON5XgB5Eh+WHB5YcCDw7bffGkcC+OWzgh++Cy64QA466CCvMjVMEVAEsgABJ8nRfcWiQyA9yM\/ZRKw5yA7LDrLjGNTiZKdesc49iQ9Ce\/75542l16dPH3OcOXOmWGGb2vnnny+RSPw9trEq1nBFQBEIBwIQHAKZQWqQnLXmuCbc2X21WluyoytryQ7ys\/FhPnoSHx8Vwu3Ujh07pEuXLsLxhx9+EKcQT7owN051K50IaKu9EYDcmG2FyCA0CA4LzgphxMUiuVaHVxKIDWtu0\/1txZJdmC07byREPImPhcrnnntudIua3armPBJPulgFa7gioAikHwEnuTHDCplBcNaCY7aVMEtwpHdriSXnJDlr0b3Rq6lAepCfO0+mXXsSX9WqVc3ODJwHxBKWuJAu0xqs+ioCmYwARIVMXrxNIC9IDPEiNyYdSONlwYEBBIdAZLnt6hhS8yK5TLToaJ+feBKfMwPeWebPn2++tMYeXdbvQYbONHquCCgCyUEAUrPdUUgLUkOcxEbXdOC7G4Rw0iCxyA2tILdWjm4q5IbQVUWw4nLb1Tbd2GwkOTBwS1ziwxdfr169ZNeuXdK0aVPjSopxv6lTp7rL0mtFQBHwQcCP1CAzd3c0UWKjSje5QWaQmx2Lc3ZTITeEfKVVfImPfbm4h+ezkoMGDZLLLrvM+MzDKSnhxJdW4LTdioBFAEJD8pZtMd1PLDBIC8FSS4TUyG\/L8zo6iY2u6cDTqpidEFhsXuRGmtJObl442jBf4mMhM5ZenToFPzbENeHE24L0qAhkGwKQEZIIoUFuduIAwoP8ELqglOGHDaSGOCcUrMXmRWzEdWq0n1kYTD6\/sjXOG4E\/iM87TsqXLy977723LF68uEAKPtjCPl22sBWI0AtFIOQIQELrtu4SJ5nZ2U8Iy22hlYTQgAJisoSGFZbrmkSw1hoE5+yOkhaLjfyUo5JcBHyJDw\/Ll156qbCIGVfz\/fv3N9\/FuOKKKwSvLXhvSa46WpoiUHQEIDMkUTLrNH6NOK0zO\/uZqIWGhhASYknNEhrWGGNrEJklNUtoxOU6JhHIT1kq6UfAl\/hQ5+STTzY7N9q2bculNGrUSF5\/\/XXzsSEToH8UgRQgAJEhXmSGVYZgjdkJAc5LSmY0AzJCEiE0yM2SmiU0tdRAMfziS3ybN2+WsWPHyk8\/\/SSXX365DB48WG666SbzmcTwN62kGmr+ZCIAiSFeRGa7mImSGeNmCOUloiNEhlgyY3wM6wvBOkOwzhDIDFFCSwTZzE3jS3xly5Y1y1fwycf3MJjNhQwzt7mqebIQgHSQZBCZ7WKWhMywtCAyBCJDIDAvMmNGlPQI42hIsnDRcjIDAV\/iq1ixogwbNkw+\/\/xz43iUI13f8847T95\/\/33RvbqZcZPjaQmBWfEjsitf\/c64IUq0e1lSIsvdMxHgJDOIDIHUEGuZkaZ7s2rRRbiQGVZevLZrfOlEwJf4LCTM7PKJyYceekheeuklM9N7xx13mI8P2TR6DA8CiZCYV7cy3jjZvLU7om7E47UW0kFs9xJS8iIyyCsWkeXumQggL0SGxKtX4xWBRBCIS3xYdSxnGTBggDRv3lzww9ekSRPBJx\/uqxKpRNMUHwEniTmtMcbFEAgMgbSwxBDOEb\/B\/qJYY2gPiR1acS9JNpFRLuWrKALpRMCX+HA91b59e+natavwRTXc0H\/88cfCspYaNWqoP74i3ilLYhydJOa1jgzicpOYk8gYF0MgMIQyE1UHsrFSFCKbfHENcXYtvSwyyk1UD02nCASFgC\/xsUCZmVzG9vjgN3t1mfCIp+wXX3whEGbdunWlVatW5qPkXnkmTZokpLHC+CE+\/7zShjkM0vEiMiwxBBI7buRKMz7GOeIkMa91ZJSZaJshG6QoJEYXE1EiSxTl2Ok0JvMQ8CU+FjAztscYX6JNY\/8uZNmzZ09ZsmSJ5ObmCi7r169fX6iINWvWyOjRo83MMR5fwurqChKyxEb3ErGEZq0yLyLDEkPIX6jxMQIgMCRREivK+BjlxqhWgxWBUoWAL\/EVB4nVq1fL7t27jaXHtrYWLVqYyZANGzYUKg4yrFKlSqHwIAMgKUtyltycFhrdSyRRQoNskOOqlzMzjgzU57pmK7G8IDCEc0QtsSCfAq072xFIOvE1btxY+F5HTk6O4Mtvzpw55tOU1apVK4Ali6LXrVsnt912m9SvX994e548ebLJUyDhngu6w+wRToXMXrBURkz5XE6\/7yPTHbXWmx+5MdAPmbEYFmFtGPLoOdUEmXddbUFe+3s1Qf7VfJf0bVnOyN8a\/C4tq2w3Ur3sZvntx\/WSinYlu0ws9GSXGUR52dCO0LdhxYqYzzTvMrLn1Q7k4Et8eF9hMoOjUzuu582bJ7\/++qszuMA544J\/\/vOfzT5fxu4qVapUIH7nzp1SoUIFYbb4q6++Mt3h4cOHy6efflognb2gK4xXmGTKxxv2lRumbBH2buLYcd7aHbY6c8RSQ7DSWCeGsDB20\/1t5YuBreWdPifI+CuaGbn+zKMEObdVI0HcejIZ5A7LtOtsaAOYZ0M7MrkNvMuIeckC+uNJfCxhYUaXX+P77rvPMDeTDla+\/vprs33NbxfHUUcdJQsWLBAcmVpxtrFy5crms5WtW7cWJkzatGkjxx9\/vMydO9eZLCXndGXpxjJWh1VnK4HkGFujKwrBfXbrCYJAeJAf0qpeQQK3efWoCCgCmYOAJ\/HxESE+JnTWWWcJlh3Hli1bmu4oR3ZuQGyQl7upeXl50rdvX2H5C3GYtKz7W7t2LZdRYW0gy2NsOhuRSo8vjN+xdOTsh+eLm\/AgN0iOsbXcdrVFCc7eET0qAtmHgCfx8REhZlgZn4P0OGKaWlm6dKkMHDjQTFq4IWFRM5bewoULzZY2lrYsWrRImB2Ops0\/YcYYLy90pbEwZ86cabbGudPlJ03Kf0gPwmPpiC0QC85adpzbcD0qAopAdiPgSXwQ0aZNm4RxOSYfmKSw3Vx7pCtMOjc8DRo0kOuvv16uu+46M2mBV5drrrlGjj32WCEv433vvvuu1KxZ00xsDBkyxKRjrI+F0eR3l1nSa6w8ZmYhP8qiSwvhYeWpZQciKopA6ULAk\/jo6vJFNcby6PLSvXUL4aRzwxWJRKRTp06CJYeFyJHrSCQi1pLE0wv5KJPxP9LRRbY+\/4hLlkB6Tisvt10doUurhJcshLUcRSDzEPAkPghqwoQJwtgcXV6IyS2Eky7MTWa9nSU9rDxIL7dd7ZCqrGopAopAuhDwJL50VZ7KeujWMmtLHZBe92aHiJIeaKgoAoqAL\/HZMTlmZt3CWB3xYYQQ0mMiw+qmpGeR0KMioAiAgC\/x0ZWlS2u7ucuXL5cPP\/xQrrrqKuOeihlcCgmb0L2F\/NCre7NqaukBRGaKaq0IpAQBX+Jz1xiJRIStZ1dffbUw8eG3gNmdN13Xecu2CGN71EcXl5lbzlUUAUVAEbAIFIn4bCa2m23dutWs07NhYThi5Q17a0VUlVHdGkXP9UQRUAQUAYuAL\/GxVq9bt27CdjKnnHHGGWZpitfODVtwEMfZy7dEd2TQxdUlK0HcBa0z1Qho+SVHwJf42F3BeB4+9Zzy9NNPC9\/cwO1UyVVITglYexP\/84fPP7q4ue3qJKdgLUURUASyDgFf4sMD8ymnnCLHHHOM+d5Gx44dhT26NWrUMI4FwoSG09o76fBKAvmFST\/VRRFQBMKDgC\/xoea4cePk9NNPF1xHcf3BBx8IS1mmTp3KZWjEae11b3ZIaPRSRRQBRSANCBSxCl\/iY78u+2rZxYH7KMq+6KKLZOzYsTJx4kTBzTxhQQszudbbSq3K5UTH9oK+I1q\/IhBuBHyJb9euXaZLW7169QKtqFevnhCHQ9ICEQFdOGdy++nYXkB3QatVBDIHAV\/iO+CAA4QJjldeeSW6dAVPLTge4ANE5cuXD7ylTGpErb0ctfYCvyGqgCKQAQj4Et8+++xjXEy9\/PLLght5lrTgXmrQoEHSu3dvQ4pBt5FJDatDzLE9m0CPioAioAjkI+BLfPnxxkMLrqOeffZZ86nIMWPGCE5DmeklPmixkxrowWwuRxVFQBFQBPwQ8CU+voLGAmYWMkN0LGfBhx6WoF+h6YpzdnP5VoZOaqQLea1HEchsBHyJDycFjRo1kk8++STmZx+DbD7EZ+s\/qV5le6rHhBDQRIpA6UXAl\/hwQsA3M\/r06SMtWrQosHXNWoJBQueczdVubpB3QutWBDILAV\/iY0a3Z8+e8uCDD8rtt99uxvjs1jW2shEfVHOx9nQ2Nyj0tV5FILMR8E2cF8UAABAASURBVCU+tqzxHQzG9txCOPFBNR\/is3XrbK5FQo8lRECzlxIEPImPyQy+lLZ48WKhS8syFrcQTrqgcHIuY9FublB3QetVBDITAU\/iowvbo0cP43SULq3t3jqPhJMuqGbPXrbZVI0zAp3NNVDoH0VAEUgQAU\/iowvLshX87Xl5ZznxxBMlyK4u3dzo+F7lcgk2VZMpAopAcRDIxjyexOdsaCq9szBj3L59e+FDRq1atZLp06c7q455DvHZSF3GYpHQoyKgCCSKgC\/xpdI7C55dBg8eLMwaL1myxMwYDx06VNav\/8OZqF8DdHzPDx2NUwQUgXgI+BIfHljKli0rqfDOsnr1atm9e7dg6eHJmXWCOD7YsGFDPJ1Fx\/fiQqQJFAFFwAcBX+JLxDuLT9m+UY0bN5bnn39ecnJyBI8vc+bMEQiQr7j5ZsyPXLV5R\/5f\/a8IKAKKQPEQ8CU+9uSyrCWV3lk+\/\/xz4\/mF3SF4dq5UqZJnSxgHXLFihcxesFTsGF+HeuWEsEyQNWvWZIyusfDMhjbQtmxoRya3gXcZ8XzR0xToS3zo0KRJE0mldxa+4bFgwQJTB\/Ug1OuWb775RurUqSO\/VzgoGnX04YeYMMLDLjVq1MgYXWNhmQ1toG3Z0I5MbgPvMhJ9kQM4iUt86MTYW7K9s+Tl5Unfvn3ll19+oQozswvJrl271lzH+uOc2KgZ1qUssZTXcEVAEQgFAgkRXyo0PeiggwRLb+HChca7M0tbFi1aJM2aNfOtTic2fOHRSEVAEUgAgcCIr0GDBsa783XXXSf169eXyy+\/XK655hrBw7Of3rpw2Q8djVMEFIFEEAiM+CKRiHTq1En4fgf9fY5cRyKRmHrbSQ0S1Mwpx0ElYxBQRRWB8CAQl\/i2bt0qL7zwgvTv37+AsNiYuHQ2xUl8tXL2TWfVWpcioAhkEQK+xMfEQ79+\/eTRRx81i42DbrdzYkM9sgR9N7R+RSBzEfAlvh9\/\/FHwwvzkk0\/KPffcI2wxs4KnlooVK6a15XZig0rVIwsoqGQ4Aqp+QAj4Eh9eWg488MCAVItdbS0d34sNjsYoAopAXAR8ia98+fLG\/dTw4cNl6dKl8sMPP0QFJ6S\/\/fZb3AqSmUBndJOJppalCJReBHyJb+PGjTJq1Ch58803pV27dtKyZcuonHvuuUJ8uqBzTmyoK6p0oa71KALpRyAdNfoSX9WqVWXGjBnCchO3EE58OpSkDifxca2iCCgCikBxEfAlPgrFcwqOBJjcYAkLXV6+xUE48ekSndFNF9JajyKQ\/QjEJT6cBlx22WWyfPlyY\/3hnBSPLRMmTEgrOqs2bY\/Wp5MbUSj0RBFQBIqBgC\/x4SV54sSJMnLkSBk0aJDgMgrvFlh\/jPux3KUYdRYry+pNf\/jgg\/SQYhWimRQBRUARyEfAl\/i2b98ueGGG7PLTRv\/XrFnTnLPA2Zyk4Y86H00DyFqFIlBKEPAlvv3331\/4hCT7aJ1LV5YtWyYsdUHSgdPu8lWizkdrqSuqdECudSgCWY2AL\/GxgLlXr15mx0bnzp2FSY7zzz\/feFW58MILDSmmGx1dyuKBuAYpAopAkRDwJT5KwgHp22+\/Lbfccot07drVuI+aMmWK4Cae+HTI7vL\/v3tEx\/fSgbjWoQhkNwJxiY\/msye3Y8eOgu+8bt26STrX71H\/rioNORhRr8sGBv2jCCgCJUAgLvGtW7dOIDu+isYnIBs2bGgIkC1rJai32FnV4is2dKUsozZXEYiNgC\/xMWt75513yqGHHmochrKWb9asWcI3OO6++24z4xu76OTF7KpyhCkM0kPMhf5RBBQBRaCYCPgSH+v0tmzZYj4KxDcyIpGIVKtWTW688Ub57rvvZNOmTcWsVrMpAoqAIhAcAr7Ex6wui5ax\/Nwq4q6KeHd4Kq6jFp8uZUkFvFpm6UFAW7oHAV\/iY51e9+7d5bbbbhNmchnvwzkBXpnbtGkjLHDGVdWeslJycDon0O9spARiLVQRKHUI+BIfXdkxY8bIqlWrhG1q3fJndCFBCPDhhx8WXFMh6UKtln5nI11Qaz2KQFYj4Et8jOs9\/\/zzMnPmTF9JJUJOi6+Wel5OJdRatiJQahBwEp9no3\/99Vd5+umn5bTTThOWs3Tq1Mk4JnVuYfPMmKRApzsqXcOXJFC1GEWglCMQl\/ieeuopefXVV6V3794ybtw4072999575Yknnkg7dGrxpR1yrVARyEoEfImPMb73339fcEB65plnylFHHSUXX3yxjB07Vggn3guVNWvWmEXPhx9+uBx99NHy7LPPipfj0kmTJkndunWjwjY492TJKvXD5wWxhikCikAJECjjlxeXVJFIRCpXrlwgGdeRSMRzATMzvSx6hsSWLFkijBGOHz9e5syZU6AMLiDI0aNHR13bz5gxo9B2OKcfPvKkW7Q+RUARyD4EfInvgAMOMKQ3depUsWN6WG64qcJdFW6r3JDgvJSFz+ztLVu2rLDVDUcHn332mTuprF+\/XqpUqVIo3BmgfvicaOi5IqAIJAMBX+LbZ599zC4NvDAff\/zxwto9Jjjwxoy7Kq8FzFiDLIGpXr260W\/z5s3GbT07PkzAnj8\/\/fSTsCyG5TH169c3X2+bPHlyoS7xqj2el9ctWSArVqzIWMG6zWT90T0b2pAt7cjke2GHt\/ZQQSAHX+JDI0gJQmKSIzc3V0aMGGGWtmDFEe8W9vHm5OQI1t7atWvlhhtuECzDU089tUDSnTt3SoUKFWTAgAHy1VdfmXFEvt\/76aefRtNZ0iPgvA4nC56gM1Vq1KiR0fqDe7QNdepkdFuyoR2Z3Ab7xUbe66AkLvGhGGQG0dF9bdmypWAJEh5LWALz2GOPydlnny1HHnmkPPLII4b8nOmxDEeNGiWtW7c2JIk1iVU5d+5cZ7Loea0MX7zsJv5owzLoJBvaANzZ0I5saAP3IihJiPiKohxjgE8++aTZ4vb6668LVqJXl5hPVD766KPi3geMdehV38ght0Vnf62pnElH2pRJ+nrpmg1toF3Z0I5saQPtCEKSTnyMPbzzzjty\/\/33C+Z4rEYxOQIxMlHCxAm7Q3Bt36xZs2iWWjnlZNP9beWzW0+QNR88E539taayHr9RTL4JCwaqR3Hex+jLnuaTpBMf7qq+\/PJLwRTn19UKa\/9Yo8cyl3fffVf4UhsTG0OGDBHGERnr69+\/vzRo0KAQBBBgoUANUAQUAUWgmAgknfiw2BYtWlTIErn66qvNGj3W6rH9DX0ZL5w2bZpJm5eXJ23btiVYRRFQBBSBlCKQdOJLqbZauCKgCGQSAqHVVYkvtLdGFVMEFIFUIaDElypktVxFQBEILQJKfKG9NaqYIqAIpAqBIInPt02sB8SrC95dGjZsKDg+2LZtm2+eICMT1ZelO\/g3pF14r8GrNVv3gtTdWXei7XDmYS83XnvCcn+K0oYPP\/xQWrVqZdaItm\/fXr744gtn0wI9T7QdOAbBgxLvCSskLrjgArMdNFDlE6icPf0XXXSRsIwtgeRJTRJa4mPr2gsvvCCs9fvPf\/5j3N\/jFzCprU9iYYnqy4v24osvmnbxkjELDqm7F3InUbUiFZVoO2yhbEtkzSa7dWxY0MdE27B69WqzVfKBBx4QPAmdddZZwo4jvBIF3QbqT7QdLA+bP3++fPTRR7Jw4ULzJcSRI0dSRCiFH3\/W7\/7rX\/8Slr4FoWRoiW\/69OnCmj\/2iFasWNH493vvvfcE5wZBABWvzkT15UafcsopZq8rO1rOOOMMgTz49YtXRzriE20HukAQ7L458cQTC21JJD4oSbQNLJpv2rSpsFVyr732kquuukoGDhxotlAGpbuz3kTbYfNwP+z5wQcfbE9Dd+QdZlkb+\/l5B4JQMJTExw1cuXKl1KtXL4oJN3LTpk2hJL6i6NujRw9hTaNt2NKlS4WXzu5\/tuFBHIvSDvT74IMPhO4te7K5DoMUpQ1Y3Dt27JAOHToIww4XXniheb4ikUjgTSlKO1gXy\/dx8JzE3njW0Z5\/\/vmBtyGWAhgybGW95ZZbjNu7WOlSGR5K4uNhhOTY1pbKxier7KLoy02nXZj7dOUHDx4sV155peD7MFn6FLecorSDcUk89mAl\/elPfypulUnPV5Q20D3HKQZdXTwEnXTSSWYsmTGzpCtWxAKL0g6GgNgVNWvWLNPVZfyYryJCnkWsttQkDyXx4Q2Gb\/ryYDrvBJZRJBL8r7FTJ86Lqi8+4fgsJ84cHn\/8cWNxUE7Qkmg7GHRHdywlry2GQbYj0TZYHbt27SqNGjUS8rHN8vvvvxec6dr4oI7ok8g7wNgwXWKsbnxg8qPKt7C\/\/fZbwRdmUPqHvd5QEh\/dPvbyMh5mAeSBxJzHYrJhYTkWRV\/G86677jrjsuvNN98U3H1lWjsYo+HesNeavdidO3cWHFNgMaVkhq4IABXlXuAcF8vKWXyZMmUkEgn+xzXRdjBOVq5cOWcTzDnESZy50D+FEChTKCQkAcywMQCKdcQvF0tA+LQlD0RIVCygRqL6vvTSS2ZP8iWXXBKaQXRnQxJpB1bFhAkTzB5rPHIw83766afL7NmzzQepnOUFcZ5IG9CLveF8NIsxMbq3L7\/8sjDZEe9zCORNhyTSDnpBJ5xwgrzxxhtmkox28J0bPgyGz8t06JmJdYSW+I499lhhgBZrghvbpEkTYZ1VWEH207dv376CMOaybNky4QNLDKZjLSHMXjNGE4a2JdKOMOjpp0OibSDdNddcI5deeqn5GuDGjRsFazwSCd7io33oF+sd4HlCSPe3v\/3NfPeaoQfG9+j+3nTTTaGwXNEvjBJa4otEIsJCTNYlMfDcr18\/Mw4TRhDRKRKJrS\/fIUb4dcbrNFaSU7Bsq1atSjGBSyQSvx1uJbEu8LKNJeiOC+I6EkmsDZFIROhFsKaM2fWHHnoosFlGL5wiEb923Cs8U+SjW3vFFVeYiQ3awbrKWA59SR8W4ZnnsxY8P+nWKbTEl24gtD5FQBEoPQgo8ZWee60tVQQUgT0IKPHtAUIPioAikPEIJNwAJb6EodKEioAikC0IKPFly53UdigCikDCCCjxJQyVJlQEFIFsQaA0EV+23LMC7eDrdXY9V4EIj4uipPXIHjOIBcD4tMMBA04LYiZ0RaA3OrmCA73ExRNrK9lRk6pdKJRL+QjngTa4lFauxFdKb3wym43Dglq1agkuqtK9lo99wzh8SGZ7zjnnHFmwYEHKdqGwbo1dLi1btkym2lpWERBQ4isCWEElZdN87969xXrY5ZwwrJNhw4YJ3jmstcWCUKwvrBacnOJYE2cPXmkXL14sbNJnFwkvIXkhEq92ssiXnTOU60xLuXiXIZ6j2+KjPMolD\/WceeaZBbwcQ5p4oSaO8nEVRf3ojO60gTppE+VQHlZSx44dhbYTjsUJHuCCB2LqevDBB41\/PasPnktwQkBZ1IO+1OMn5AVXrFKL03nnnSdso\/TU3kRZAAAFZklEQVTK57ZgwYb8lOOVXsOCQ0CJLzjsE66ZfZi8PHjZZScLG+nx6oIfNna0YKGwd5aXf9y4cYKbJXaGcI7rK1bzu9Pi+LRPnz7yj3\/8w3gffuKJJ2TEiBGSl5dXSC+8E+OQ4J\/\/\/KcsX75cnGkpF0sPsuHotvgoj3LJQznslMBlEu2hIiyf22+\/3ejwl7\/8RSAstlyxW2fKlCmCt2rqHDBggEBAGzZsIJusWbNGcGKJT0C2M9Jm3GOBEa7w582bF\/WyAlEOHDhQqBcsaAftQR9TWJw\/r7zyirAbAp3atGljth+CdZxsGh1iBJT4QnxznKrx8tP9YtsbW5VuvPFGZ7Q5Z3M97qLY4wl5YB1Bkl5dQQiDLUM4F8CLR+PGjc23J+bMmWPKcv7BWjriiCME7yuRSERI265dO4GYnOm8zklDWvJQD9YdPvw4Jz37S3ELxTV6\/\/zzz4K1h7sryL127drGOSjbsnbv3i20iXyHHnqosI+bcMgQN+24Y4J42ZzPN0BoO2mxiFu3bm08LVMPJH3YYYcJliLx8eSvf\/2r8ZhNXdSJjitXroyXTeNDjIASX4hvjlUNi65Xr17GSSbWDd0t2yW0aThCduwFxsMIlhhWCi8pcW7BYsH1Oh576f4h48ePl++++86dVOgSUy+kayMhGAjKXnsdseq+\/fZbIa2Nh5QgIaw1wpxxXFvZunWrYAkyAQDZPPXUU4YQbTy6WHdM6IwuuC2z8XjsrlChgrmkrXj3oTtNOyFavLKQzySI88fpCZw6qTtOFo0OOQJKfCG\/QahHFwsPNVhPEB5jVIxv4RePeCu4h6KrN2PGDMFKwzKEaGy884j1wngb3T26xchnn31mvjnhTMc5VhcWp7W2OHJdrVo1omMKJJGTkyN4pbGJ6KLiDABis2Fex4cfftgE02XlWyu4KrdkaSIcfw488EDButuyZUs0lHogPAJoa8+ePaNutND9k08+Ebr5xBcSVwC+IG0Q\/vuwoLEcbVisozNfrDQaHgwCSnzB4F6kWiEySAwLioxYcXjnxfLgpXZ2AYnnxSTtc889Zyw44gl3psXq+vrrr40PPYgM4mMgHuIkrVMgXbrAkAVp6VZy3bZtW2eyQufoRzf33\/\/+t5kQwFccY33Uhf6FMrgCqAvSxB8j1ihkybUrmfGocsghh8gzzzwj1IGzV9LbdBD8W2+9Zbq2lEk7\/v73vwvWqE3jd6SrTJmUPWbMGKlRo4bp+rrz4BEFK5I6IF0mY9xp9DocCCjxheM++Gpx2WWXmbEturDM7DIhcOuttwpOWSElrnmRmbWsU6eOEEZXl3O+4nb99dcL\/v4It2kZI7vrrrtkyJAh5kM7kB5jb8ySupVh7I1PAeKrju7iDTfcIDfffLMQ7k7rvsY6ZYyM7iq+4rDEsN4gRXda5zVulhi7o4vN2B3lYHkycwoBOdNiCQ7Mn7xgtpX0+LBjTNKmoU1ggN899Gdig24044g2jd+R\/NwDyqYO6qJOMMWXIrO35L\/88ssFq5mP\/rh1cKclvUpwCCjxBYd9wjVjSTBex4wk3TQ87GJ1UACTBlhfzOrS9bTpWK7RpUsXGTRokGAxMpHhTMvYGoP806ZNM11A0jPjGokUdsIZiUSMo0vSYK0xU+tMC8lSP2Wik1OwMiExZqPRn5lfSJc0WLHOL845yyEN7aS96EgcVhz1NG\/eXLCmaBPlYGHRrSae9OhH27AqqT8SiUT97qE\/5RFP3kSEdXfkoWx0QjfyUT\/YohvXfPNi0qRJwnegGZa44447BH3BxZ2W9Jkl2aWtEl923c9S2ZqdO3cKy3pY90c3n64xhMPSE6ziUgmKNtoXgf8DAAD\/\/zkz5ogAAAAGSURBVAMA7ZyvQU46Yj8AAAAASUVORK5CYII=","height":191,"width":318}}
%---
%[output:7a96c431]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAT4AAAC\/CAYAAABno7D5AAAQAElEQVR4AeydCbxN5frHn03dRImTpAwhhFRXZahQbgOS0K0bbtNtjkbkdKmbBoRGQ1QaNNCcckOjcFSuJCrKEBmra8pVJOl\/vm\/e\/V9nnbXX3uecvfdae5\/Hx3PWWu\/4vL+11m8\/7\/SsMr\/rP0VAEVAEShkCZUT\/KQKKgCJQyhBQ4itlN1ybqwgoAiJKfB5PgQYpAopAdiOgxJfd91dbpwgoAh4IKPF5gKJBioAikN0IKPEFdH\/fffddqVu3rowdOzYlGnz++edyzDHHSN++fZNTvk8p27Ztkx49esjJJ58sP\/zwg0\/K1EWBI+2l3amrJfkloy96I5wnWkMYMHfqiu60gefaGR7WcyW+sN6ZJOsFIUFMEBQvTZKLL1BcOuuiYl66MWPGyH333SdHHXUUQTGlqLrxIqfqB4r7MGTIEKlUqZJMmjRJDj744Jh6hz0C3E8\/\/XTp06ePcD\/Crm9WEh9WDg+rFR5ebgQPGi8+woNm47EWiLfizA9Z8LIQRznk4QWjDM45Ui7xXkLZpEP4RYz1UFAndT333HPGEqQuyiM9+ciPUJ4z3NaPDpxThtWXdAjX5513nqxevVo+\/vhjufLKK4X0xFmhPmf5xLvLQ0fSWCGPzW+P\/\/3vf8WrLnRAN5uXsmwejlwTR1vBl3PbVuKpizArXBOOjB8\/3pDH0UcfzaVpG7rbtNRL\/UhRdKMOsKLQYcOGRa1zwm3ZHLkmjZdQJ\/WTDuGcMPClbO4H96VLly7y\/fffFyqCtOQhLwJO7kTvvPOOeWaId2JGHU4cOCeM\/OhMep43yrflOp+3q666qpAl74wnv7O+iy++mKKF+2FOQvwn64iPG\/Hqq68WgJwHjBttA3nYevfubS+Fh9rG8wA48\/NQ3njjjeZlshlGjx5tCIRrynr22Wc5LSSUSdk24n\/\/+59ce+21MbuD1HXbbbfZ5CYd6clnAymPcu11so6QRs2aNWXZsmWmyJ9\/\/lnWrl0r1atXl\/Lly5uX3okLidy4EuYlP\/30k4Ah7bPxlMW94poj15zTVvDl3ArtpS57zdFaFhDD3Llzo3oSN3DgwOj94Zp6qZ82ce2UeLo503JeUBdCJKaVA8lQL\/X\/kVLMjw\/ku2HDBhsU8+iVH5zAy2aibOczg+ULORHvxoFnlTDirJCXMrgGS+fzBqGShziEci+44ALhHnGNOJ9HLFasV+4HZREfVskq4gPsiRMnyv777y+vv\/66fPPNN9KvXz+D\/bRp08yRP7Hiyc9Na9mypSxcuNAI59x8hLwIYcQ\/+uijXMrMmTMLECOBPLRPPPEEp0I6dDnnnHPMg09eE+HxB31Je9pppwkPOQ8l+QijHLI428J1PKlataq89NJLArGhO+Xst99+BbKRplmzZkL7wQEdqbtNmzYCYSSCKwUedNBBheqKRCKGRKmbcsESXcANK4cjedGLeNJxbYX2uu8ZLx+WBfm3bNki6EmbwH3dunWmrdRjy\/vyyy\/NC+vGIRKJrRt6oBN6cF+uvvpq8dOFdE6hfoRy0APhHFz5gaFsrsECkqG76JefZxocwAvCtmkpx5YNLmASCwd7f21e6ifvvffea553dLNhlGvTcZw9e7bBkHDn88hzTn38QPJDyf1AB\/KEVbKK+AAb0I888kipU6eOwRzS4MHihttfWWf8SSedZIiSl2XlypVCfh5WLCCEc1OQ4499yfiF40F0REVPIQssJuqmHCJ4uHhgIDWu3UJZ6GPDeTk4b9++PQchH\/kpxwQk+U+9evVM+8GRuq0+XIOLEzcvXGOpA6nOmDFDJkyYIFjHvFi8YKTfvn27IUWLE+R16aWXEmWEF4p7wwvduXNn06XDyjCR+X\/Qjbj8U\/Of\/Iceeqj5gaEesKe8BQsWeI7\/+elmCnT8iaeLI6k5BUNO7POCbuhCGATK0U\/c+SFG2gGOFSpUMFmduNFuE5j\/h7pIB+4QGzj4Pcv5WaLWvtUX\/BDiEKsP1jfdXI6EZ6JkFfEl6wbwUkMwToF0klV+WMuxpPv2228bK9ZJdCXRGcJgfImXBXLlBeSFLUqZpCef857wA2BfRmdZhPOy25eWF5RxLCxZZzrOi6NbLF0oL0zCsA2Yg5ETj5LoyI8hlqfzPkCwEO2eH3sz3opRUJJ6Up03q4gPsBljoFuzYsUKg53tLtKNq1KlignjBUK4oLuExcCvZe3atc1Nwzr0eklIn6i4zX7nC8Y4USLlQBKk48HlyNgODzIPNNcIViUPHO2l3YQVV7CSIbt58+YZK8z+8ieKa6x6wRqhu8gPCC+htfhsHq4JBye6TjacF4p7Y+NtuD1ajOw19w2Sg+xsl4x6Y+VHL4Q0sXSzZcfTxaazR6sbXVPahdi2WSvepvU62vz2\/vPccP\/5EXF2db3yMh5H95kf8avzu+hFeT6svuCC2PLRh3eFLq8Ncx6xvukZ8K5hSTvjwnaeVcQH2N27dzfjEO5ukftB48XgIYIYuSnEk5+bxkuCtUA8wovEC0W6RIWXBOLgQUEX29WgXCSRcrDA+IWla4ceHLlm9sySkdWVOqgrXrk8yLSdl9Cd1upMGh5g6icNuCSKK+mtUA51VaxY0QwnoD\/tIMym4ceIsrkm3OLEtRXuDefEk98KRAAOYEI8gq7cQ+qmLNJSL2lISxqEeMrz0410ViiDHx4\/XWxae+Q+I9SFLgjnWIyc23SxjqQhLc8o7UBf0tJdtl1drr2EtmIE2LyJPB+QJPWhI3Xb+mz5PA\/gCBboYwVcSIMRQZ38iHAdZskq4gNoft24gZxb4ZefX3N7zc2966677KWZALHxdJOc+bnRo0aNEl6oaIYET9y6UO+DDz4oEEwiRTCmw9IOZ1quCUcfZuRsXK9evYSXzF67j9YCdYe7r+3DjeWHBWjj3W0h3I0rYYi7rsaNGwtrvIgDT7pG6IqFiiXCTCHXxIOR894Qxr2hLs6t8HIRbl9wa6UQz8ylLY9rxOJWVN0oH50pA6HOWLoQ7xTuM\/ebNtlwzplg4f7ZsFhH0vDsOeu37Y6Vx4aT1\/6gEIbOPNf8UGJZE+YW8vg9Uzx3jNE69aFMng1rYZKfctxlh+06pcSHBXLRRRcJv8zyx78CfydNmmQGq+0vR3EsqwIF7rmAvJxjEDyse6KiB15Em4YbF43IP3HmZzCZG54fHJ1csOkJJ54XmYecNG5xlsVAs30o0In6bVmkoyzKdJZh05EW4drGc04YwvIO9LB1UA7lUS7p0Y940nLkmnC32HxeaSiL\/Faon\/yURXpbt70mHeFc27zoBCkRzjnkimWBlcHLSRnz58+nWKFrZU7y\/1AX5VmxuIEnL7jt8ucnNT8slG\/TciS\/O440frqBBYKelGHrpCyurdhwyncL+tEmm5ZzwkhH3ejgDCPcKc76KcPW5ZXXYox+lEFa8iCEOeO5Jpw0pEV4T7kX3B\/I8YwzzhB+nJip5QeDNG59KNMZTrlch11SRny\/\/\/67sAQi1ngAwKxZs0ZYs8UNQPweANKrZB8CvMAMCdAyXjp+BOmeYVVgbREeT7BS6ZqTL15ajY+NAN1bLFJ+hDi33WPGWLlPsXNmXkzKiI9fijlz5kjr1q1jorJ+\/XphjCdmgiRHcPPi\/cImuUotLgEEsDr44XMKVhbWRQLZzTIV0lNOIuk1jTcCWKIYH877wLm16rxzZWZoSoiPgXPGJvgFj0VszEqxPosxgfr165vxqcmTJwuWYmZCWXytNacioAikF4GUEB9dXEzm5s2bx2zNzp07hZmpAQMGyFdffSVDhw6V4cOHy6efflooD90flboFxkMVD8Uj058BJrYKvexpCkg68S1ZskRmzZplNsJHIpGYzahcubJgFdIVLlu2rNlydPzxx5stU16ZMLkzWWhTJuuP7tnQhmxpRzbci1NPPZVmBCKexIeHjW7duhkyYuA5njg1X7VqleTl5UmLFi2MhcKAM11e56Jb0i9evNjsYf3ll1+4jAqD2tELPSm9CGjLFYEUIuBJfIyzQUg9e\/aU3NxcXyGNUz+ms\/lVtcI6H5YpuAdImWhg6wszSL\/99pvZIsVaIBafOsvLlvP33nsv45uSDW3gJmRDO7KhDdyLoMST+CKRiPFugSnasWNH8RPSJKo8ux9Yq8d6IcYAmdjAESOTG4z19e\/fXxo0aJBocZpOEVAEFIFiIeBJfPvss4+0bdtW9tprr7iF4obILxGWHlYgaex0ub1moSReKrAO6R5TJ+lUFAFFwAsBDUsWAp7ERzf3scceE2Zl7c6LX3\/9NVl1ajmKgCKgCASKgCfxYcWxpu6NN94wXV68FR933HFy5513ClPQjAEGqrVWrggoAopACRDwJD7KY4lJo0aNZNCgQYKboscff1yY7e3QoYPZs\/raa6+JWoEgpaIIKAIBIlCsqmMSn7O0vffeW5htHTFihEydOlVwPYPXic2bNzuT6bkioAgoAhmBQELEh6XHkhQWG2Px4VED4jvwwAMzopGqpCKgCCgCTgRiEh\/WHJ+ewzUN3i+w9Fhu8sknnwgztU2bNhW6w87C9FwRUAQUgUxAwJP4WG\/XpUsXGTlypHCcPn26MKZ35plnGl9nmdCwPTrqQRFQBBSBQgh4Eh\/bxljOgi89dmbgiLBQTg1QBBQBRSBDEfAkPjwn475748aNcZuFdRg3kSZQBBQBRSBECHgSH\/rh0faee+4RxvX8hDSkV8kcBFRTRaC0I+BJfHR1b775ZmEfLhMbfkKa0g6itl8RUAQyCwFP4tt3333NXl0\/5wTOuMxqsmqrCCgCpR0BT+Ir7aBo+0shAtrkUoWAEl+put3aWEUgHAis2rRDdlU5QvKWbRHO062VEl+6Edf6FAFFQGYv3yLbWvWTsx+er8Snz4MioAiECYHs1SWuxceaPtxSNWzYUPCe\/P7778stt9xiPLVkLyzaMkVAEchmBHyJD797DzzwgHE\/9cwzz8ghhxwi9erVk\/Lly8v9998vu3btymZstG2KgCKQpQj4Et+GDRtk0aJFwpq+ww47TMqUKSPlypWTSy65xDgkxZFBluKizVIEFIEUIhDEhIazOb7E50zoPOdbHHvvvbczSM8VAUVAEcgYBHyJr0qVKtK4cWO5++67hQ8C8RnI77\/\/XsaOHWtc0leuXDljGqqKKgKKQDgRqJVTLu2K+RJfJBIRJjbwznLhhRfK3LlzjZsqurh0f7H80q6xVqgIKAKKQAkR8CU+yt5vv\/3k9ttvN2N9c+bMMUf89Km1BzqFRAMUAUUgAxDwJb6tW7fK0KFDjYcWyI8Z3jvuuMNc47EFD81r164tcTNZMmM\/Y1niwrQARUARUATiIOBLfORlZvfNN98USNBe8+lJwj\/++GPp2rUrwcUWlsxMnDhRcHpa7EI0oyKgCGQUAqs2bQ9UX1\/i4+tqO3fuFD40NGrUKBk8eLA5f+SRRwQPLsOGDTPu6UvSgi+\/\/FLoQvMho5KUo3nDi4Bqpgi4EVi9aUc0KHSTrMIobAAAEABJREFUG3RB+cJanTp1okpywiJmwol3xxGfqGzbtk0g1CuvvFKYQU40n6ZTBBSB7EAgCNIDOV+LD4ekTG7QpaVLSgaOWGicY\/UtW7aM02IJXdyaNWtK8+bN4+avW7euWTS9YsWKjDyuWbMmI\/V24p0NbaA92dCOTG5D7SbNJW\/5FvPO16qc\/qUsVOxLfBDb9ddfLw899JC0aNFC2rRpIyeccILceuutwvKWlStXml0dFFRUWbJkicyaNUuw9iKRSNzsrCPEusxUqVGjhmSq7lbvbGgDbQlFO\/J7UehSXMnkNjw76e3o+35SvWDWAvsSH9o1adJEpk2bJnx1LTc3V4YPHy55eXnSoUMHadiwobz99v83gvSJyqpVq0w5ECrW3KuvvmpIsG\/fvokWoekUAUUgAxHAJZVV+6TDK9nTtB7jEh9dW8bycFDQrFkzOeKII2T16tXSo0cP4YNEFSpUKJbCp512mtkNgiWHnHPOOWbi5N577y1WeZpJEVAEwo8Ae3Qnzl1vFC3z8wZpVS+kxDdu3DiB8Fq2bClWIKnatWsL43+mBfpHEVAEFIEEEJg497uo49G9NnztypG+S1+L78cff5Tp06fLhAkT5IUXXjDjekuXLpVrr71WjjzySLOkJVmqYulhBSarPC1HEVAEwoXA0LdWytC3VhilmM0t\/+kT5jyIP77E98svvxidGICtVKmSbNy4UbZv3y5nn322zJgxQyBGk0D\/KAKKgCIQAwG6t0PfKkh6o7o1ipE6PcG+xMesLo4IWLIC8THWh6AapGeJkWsVRUARUATcCEB6fFfDael1b3ZIYGN7Vj9f4mMdX\/fu3c2SlXXr1snhhx8unTt3NlKtWjXJycmx5aThqFUoAopApiDA19POHj1f\/nz3R9ExPbq3kF5uu9qBN8OX+NCOZSt0axnTwxXVfffdJyNGjJB77rlHsAZJo6IIKAKKANYdXVoIDyvPLlIGme7NqskbPZtKGEgPfXyJD8cE7NP9+eefDcnR9WVP7THHHCMvvviisOWMQlQUAUWgdCLgJDusO7q0TsLDynsjn\/BGd28knIcFJU\/iYwKD2dwpU6bIyy+\/LBzx0GLltddeM+EQYlgaUkr10GYrAmlDAJJDWJLSa+Jiyek93XRlvcjOWnif3XpC4ON5XgB5Eh+WHB5YcCDw7bffGkcC+OWzgh++Cy64QA466CCvMjVMEVAEsgABJ8nRfcWiQyA9yM\/ZRKw5yA7LDrLjGNTiZKdesc49iQ9Ce\/75542l16dPH3OcOXOmWGGb2vnnny+RSPw9trEq1nBFQBEIBwIQHAKZQWqQnLXmuCbc2X21WluyoytryQ7ys\/FhPnoSHx8Vwu3Ujh07pEuXLsLxhx9+EKcQT7owN051K50IaKu9EYDcmG2FyCA0CA4LzgphxMUiuVaHVxKIDWtu0\/1txZJdmC07byREPImPhcrnnntudIua3armPBJPulgFa7gioAikHwEnuTHDCplBcNaCY7aVMEtwpHdriSXnJDlr0b3Rq6lAepCfO0+mXXsSX9WqVc3ODJwHxBKWuJAu0xqs+ioCmYwARIVMXrxNIC9IDPEiNyYdSONlwYEBBIdAZLnt6hhS8yK5TLToaJ+feBKfMwPeWebPn2++tMYeXdbvQYbONHquCCgCyUEAUrPdUUgLUkOcxEbXdOC7G4Rw0iCxyA2tILdWjm4q5IbQVUWw4nLb1Tbd2GwkOTBwS1ziwxdfr169ZNeuXdK0aVPjSopxv6lTp7rL0mtFQBHwQcCP1CAzd3c0UWKjSje5QWaQmx2Lc3ZTITeEfKVVfImPfbm4h+ezkoMGDZLLLrvM+MzDKSnhxJdW4LTdioBFAEJD8pZtMd1PLDBIC8FSS4TUyG\/L8zo6iY2u6cDTqpidEFhsXuRGmtJObl442jBf4mMhM5ZenToFPzbENeHE24L0qAhkGwKQEZIIoUFuduIAwoP8ELqglOGHDaSGOCcUrMXmRWzEdWq0n1kYTD6\/sjXOG4E\/iM87TsqXLy977723LF68uEAKPtjCPl22sBWI0AtFIOQIQELrtu4SJ5nZ2U8Iy22hlYTQgAJisoSGFZbrmkSw1hoE5+yOkhaLjfyUo5JcBHyJDw\/Ll156qbCIGVfz\/fv3N9\/FuOKKKwSvLXhvSa46WpoiUHQEIDMkUTLrNH6NOK0zO\/uZqIWGhhASYknNEhrWGGNrEJklNUtoxOU6JhHIT1kq6UfAl\/hQ5+STTzY7N9q2bculNGrUSF5\/\/XXzsSEToH8UgRQgAJEhXmSGVYZgjdkJAc5LSmY0AzJCEiE0yM2SmiU0tdRAMfziS3ybN2+WsWPHyk8\/\/SSXX365DB48WG666SbzmcTwN62kGmr+ZCIAiSFeRGa7mImSGeNmCOUloiNEhlgyY3wM6wvBOkOwzhDIDFFCSwTZzE3jS3xly5Y1y1fwycf3MJjNhQwzt7mqebIQgHSQZBCZ7WKWhMywtCAyBCJDIDAvMmNGlPQI42hIsnDRcjIDAV\/iq1ixogwbNkw+\/\/xz43iUI13f8847T95\/\/33RvbqZcZPjaQmBWfEjsitf\/c64IUq0e1lSIsvdMxHgJDOIDIHUEGuZkaZ7s2rRRbiQGVZevLZrfOlEwJf4LCTM7PKJyYceekheeuklM9N7xx13mI8P2TR6DA8CiZCYV7cy3jjZvLU7om7E47UW0kFs9xJS8iIyyCsWkeXumQggL0SGxKtX4xWBRBCIS3xYdSxnGTBggDRv3lzww9ekSRPBJx\/uqxKpRNMUHwEniTmtMcbFEAgMgbSwxBDOEb\/B\/qJYY2gPiR1acS9JNpFRLuWrKALpRMCX+HA91b59e+natavwRTXc0H\/88cfCspYaNWqoP74i3ilLYhydJOa1jgzicpOYk8gYF0MgMIQyE1UHsrFSFCKbfHENcXYtvSwyyk1UD02nCASFgC\/xsUCZmVzG9vjgN3t1mfCIp+wXX3whEGbdunWlVatW5qPkXnkmTZokpLHC+CE+\/7zShjkM0vEiMiwxBBI7buRKMz7GOeIkMa91ZJSZaJshG6QoJEYXE1EiSxTl2Ok0JvMQ8CU+FjAztscYX6JNY\/8uZNmzZ09ZsmSJ5ObmCi7r169fX6iINWvWyOjRo83MMR5fwurqChKyxEb3ErGEZq0yLyLDEkPIX6jxMQIgMCRREivK+BjlxqhWgxWBUoWAL\/EVB4nVq1fL7t27jaXHtrYWLVqYyZANGzYUKg4yrFKlSqHwIAMgKUtyltycFhrdSyRRQoNskOOqlzMzjgzU57pmK7G8IDCEc0QtsSCfAq072xFIOvE1btxY+F5HTk6O4Mtvzpw55tOU1apVK4Ali6LXrVsnt912m9SvX994e548ebLJUyDhngu6w+wRToXMXrBURkz5XE6\/7yPTHbXWmx+5MdAPmbEYFmFtGPLoOdUEmXddbUFe+3s1Qf7VfJf0bVnOyN8a\/C4tq2w3Ur3sZvntx\/WSinYlu0ws9GSXGUR52dCO0LdhxYqYzzTvMrLn1Q7k4Et8eF9hMoOjUzuu582bJ7\/++qszuMA544J\/\/vOfzT5fxu4qVapUIH7nzp1SoUIFYbb4q6++Mt3h4cOHy6efflognb2gK4xXmGTKxxv2lRumbBH2buLYcd7aHbY6c8RSQ7DSWCeGsDB20\/1t5YuBreWdPifI+CuaGbn+zKMEObdVI0HcejIZ5A7LtOtsaAOYZ0M7MrkNvMuIeckC+uNJfCxhYUaXX+P77rvPMDeTDla+\/vprs33NbxfHUUcdJQsWLBAcmVpxtrFy5crms5WtW7cWJkzatGkjxx9\/vMydO9eZLCXndGXpxjJWh1VnK4HkGFujKwrBfXbrCYJAeJAf0qpeQQK3efWoCCgCmYOAJ\/HxESE+JnTWWWcJlh3Hli1bmu4oR3ZuQGyQl7upeXl50rdvX2H5C3GYtKz7W7t2LZdRYW0gy2NsOhuRSo8vjN+xdOTsh+eLm\/AgN0iOsbXcdrVFCc7eET0qAtmHgCfx8REhZlgZn4P0OGKaWlm6dKkMHDjQTFq4IWFRM5bewoULzZY2lrYsWrRImB2Ops0\/YcYYLy90pbEwZ86cabbGudPlJ03Kf0gPwmPpiC0QC85adpzbcD0qAopAdiPgSXwQ0aZNm4RxOSYfmKSw3Vx7pCtMOjc8DRo0kOuvv16uu+46M2mBV5drrrlGjj32WCEv433vvvuu1KxZ00xsDBkyxKRjrI+F0eR3l1nSa6w8ZmYhP8qiSwvhYeWpZQciKopA6ULAk\/jo6vJFNcby6PLSvXUL4aRzwxWJRKRTp06CJYeFyJHrSCQi1pLE0wv5KJPxP9LRRbY+\/4hLlkB6Tisvt10doUurhJcshLUcRSDzEPAkPghqwoQJwtgcXV6IyS2Eky7MTWa9nSU9rDxIL7dd7ZCqrGopAopAuhDwJL50VZ7KeujWMmtLHZBe92aHiJIeaKgoAoqAL\/HZMTlmZt3CWB3xYYQQ0mMiw+qmpGeR0KMioAiAgC\/x0ZWlS2u7ucuXL5cPP\/xQrrrqKuOeihlcCgmb0L2F\/NCre7NqaukBRGaKaq0IpAQBX+Jz1xiJRIStZ1dffbUw8eG3gNmdN13Xecu2CGN71EcXl5lbzlUUAUVAEbAIFIn4bCa2m23dutWs07NhYThi5Q17a0VUlVHdGkXP9UQRUAQUAYuAL\/GxVq9bt27CdjKnnHHGGWZpitfODVtwEMfZy7dEd2TQxdUlK0HcBa0z1Qho+SVHwJf42F3BeB4+9Zzy9NNPC9\/cwO1UyVVITglYexP\/84fPP7q4ue3qJKdgLUURUASyDgFf4sMD8ymnnCLHHHOM+d5Gx44dhT26NWrUMI4FwoSG09o76fBKAvmFST\/VRRFQBMKDgC\/xoea4cePk9NNPF1xHcf3BBx8IS1mmTp3KZWjEae11b3ZIaPRSRRQBRSANCBSxCl\/iY78u+2rZxYH7KMq+6KKLZOzYsTJx4kTBzTxhQQszudbbSq3K5UTH9oK+I1q\/IhBuBHyJb9euXaZLW7169QKtqFevnhCHQ9ICEQFdOGdy++nYXkB3QatVBDIHAV\/iO+CAA4QJjldeeSW6dAVPLTge4ANE5cuXD7ylTGpErb0ctfYCvyGqgCKQAQj4Et8+++xjXEy9\/PLLght5lrTgXmrQoEHSu3dvQ4pBt5FJDatDzLE9m0CPioAioAjkI+BLfPnxxkMLrqOeffZZ86nIMWPGCE5DmeklPmixkxrowWwuRxVFQBFQBPwQ8CU+voLGAmYWMkN0LGfBhx6WoF+h6YpzdnP5VoZOaqQLea1HEchsBHyJDycFjRo1kk8++STmZx+DbD7EZ+s\/qV5le6rHhBDQRIpA6UXAl\/hwQsA3M\/r06SMtWrQosHXNWoJBQueczdVubpB3QutWBDILAV\/iY0a3Z8+e8uCDD8rtt99uxvjs1jW2shEfVHOx9nQ2Nyj0tV5FILMR8E2cF8UAABAASURBVCU+tqzxHQzG9txCOPFBNR\/is3XrbK5FQo8lRECzlxIEPImPyQy+lLZ48WKhS8syFrcQTrqgcHIuY9FublB3QetVBDITAU\/iowvbo0cP43SULq3t3jqPhJMuqGbPXrbZVI0zAp3NNVDoH0VAEUgQAU\/iowvLshX87Xl5ZznxxBMlyK4u3dzo+F7lcgk2VZMpAopAcRDIxjyexOdsaCq9szBj3L59e+FDRq1atZLp06c7q455DvHZSF3GYpHQoyKgCCSKgC\/xpdI7C55dBg8eLMwaL1myxMwYDx06VNav\/8OZqF8DdHzPDx2NUwQUgXgI+BIfHljKli0rqfDOsnr1atm9e7dg6eHJmXWCOD7YsGFDPJ1Fx\/fiQqQJFAFFwAcBX+JLxDuLT9m+UY0bN5bnn39ecnJyBI8vc+bMEQiQr7j5ZsyPXLV5R\/5f\/a8IKAKKQPEQ8CU+9uSyrCWV3lk+\/\/xz4\/mF3SF4dq5UqZJnSxgHXLFihcxesFTsGF+HeuWEsEyQNWvWZIyusfDMhjbQtmxoRya3gXcZ8XzR0xToS3zo0KRJE0mldxa+4bFgwQJTB\/Ug1OuWb775RurUqSO\/VzgoGnX04YeYMMLDLjVq1MgYXWNhmQ1toG3Z0I5MbgPvMhJ9kQM4iUt86MTYW7K9s+Tl5Unfvn3ll19+oQozswvJrl271lzH+uOc2KgZ1qUssZTXcEVAEQgFAgkRXyo0PeiggwRLb+HChca7M0tbFi1aJM2aNfOtTic2fOHRSEVAEUgAgcCIr0GDBsa783XXXSf169eXyy+\/XK655hrBw7Of3rpw2Q8djVMEFIFEEAiM+CKRiHTq1En4fgf9fY5cRyKRmHrbSQ0S1Mwpx0ElYxBQRRWB8CAQl\/i2bt0qL7zwgvTv37+AsNiYuHQ2xUl8tXL2TWfVWpcioAhkEQK+xMfEQ79+\/eTRRx81i42DbrdzYkM9sgR9N7R+RSBzEfAlvh9\/\/FHwwvzkk0\/KPffcI2wxs4KnlooVK6a15XZig0rVIwsoqGQ4Aqp+QAj4Eh9eWg488MCAVItdbS0d34sNjsYoAopAXAR8ia98+fLG\/dTw4cNl6dKl8sMPP0QFJ6S\/\/fZb3AqSmUBndJOJppalCJReBHyJb+PGjTJq1Ch58803pV27dtKyZcuonHvuuUJ8uqBzTmyoK6p0oa71KALpRyAdNfoSX9WqVWXGjBnCchO3EE58OpSkDifxca2iCCgCikBxEfAlPgrFcwqOBJjcYAkLXV6+xUE48ekSndFNF9JajyKQ\/QjEJT6cBlx22WWyfPlyY\/3hnBSPLRMmTEgrOqs2bY\/Wp5MbUSj0RBFQBIqBgC\/x4SV54sSJMnLkSBk0aJDgMgrvFlh\/jPux3KUYdRYry+pNf\/jgg\/SQYhWimRQBRUARyEfAl\/i2b98ueGGG7PLTRv\/XrFnTnLPA2Zyk4Y86H00DyFqFIlBKEPAlvv3331\/4hCT7aJ1LV5YtWyYsdUHSgdPu8lWizkdrqSuqdECudSgCWY2AL\/GxgLlXr15mx0bnzp2FSY7zzz\/feFW58MILDSmmGx1dyuKBuAYpAopAkRDwJT5KwgHp22+\/Lbfccot07drVuI+aMmWK4Cae+HTI7vL\/v3tEx\/fSgbjWoQhkNwJxiY\/msye3Y8eOgu+8bt26STrX71H\/rioNORhRr8sGBv2jCCgCJUAgLvGtW7dOIDu+isYnIBs2bGgIkC1rJai32FnV4is2dKUsozZXEYiNgC\/xMWt75513yqGHHmochrKWb9asWcI3OO6++24z4xu76OTF7KpyhCkM0kPMhf5RBBQBRaCYCPgSH+v0tmzZYj4KxDcyIpGIVKtWTW688Ub57rvvZNOmTcWsVrMpAoqAIhAcAr7Ex6wui5ax\/Nwq4q6KeHd4Kq6jFp8uZUkFvFpm6UFAW7oHAV\/iY51e9+7d5bbbbhNmchnvwzkBXpnbtGkjLHDGVdWeslJycDon0O9spARiLVQRKHUI+BIfXdkxY8bIqlWrhG1q3fJndCFBCPDhhx8WXFMh6UKtln5nI11Qaz2KQFYj4Et8jOs9\/\/zzMnPmTF9JJUJOi6+Wel5OJdRatiJQahBwEp9no3\/99Vd5+umn5bTTThOWs3Tq1Mk4JnVuYfPMmKRApzsqXcOXJFC1GEWglCMQl\/ieeuopefXVV6V3794ybtw4072999575Yknnkg7dGrxpR1yrVARyEoEfImPMb73339fcEB65plnylFHHSUXX3yxjB07Vggn3guVNWvWmEXPhx9+uBx99NHy7LPPipfj0kmTJkndunWjwjY492TJKvXD5wWxhikCikAJECjjlxeXVJFIRCpXrlwgGdeRSMRzATMzvSx6hsSWLFkijBGOHz9e5syZU6AMLiDI0aNHR13bz5gxo9B2OKcfPvKkW7Q+RUARyD4EfInvgAMOMKQ3depUsWN6WG64qcJdFW6r3JDgvJSFz+ztLVu2rLDVDUcHn332mTuprF+\/XqpUqVIo3BmgfvicaOi5IqAIJAMBX+LbZ599zC4NvDAff\/zxwto9Jjjwxoy7Kq8FzFiDLIGpXr260W\/z5s3GbT07PkzAnj8\/\/fSTsCyG5TH169c3X2+bPHlyoS7xqj2el9ctWSArVqzIWMG6zWT90T0b2pAt7cjke2GHt\/ZQQSAHX+JDI0gJQmKSIzc3V0aMGGGWtmDFEe8W9vHm5OQI1t7atWvlhhtuECzDU089tUDSnTt3SoUKFWTAgAHy1VdfmXFEvt\/76aefRtNZ0iPgvA4nC56gM1Vq1KiR0fqDe7QNdepkdFuyoR2Z3Ab7xUbe66AkLvGhGGQG0dF9bdmypWAJEh5LWALz2GOPydlnny1HHnmkPPLII4b8nOmxDEeNGiWtW7c2JIk1iVU5d+5cZ7Loea0MX7zsJv5owzLoJBvaANzZ0I5saAP3IihJiPiKohxjgE8++aTZ4vb6668LVqJXl5hPVD766KPi3geMdehV38ght0Vnf62pnElH2pRJ+nrpmg1toF3Z0I5saQPtCEKSTnyMPbzzzjty\/\/33C+Z4rEYxOQIxMlHCxAm7Q3Bt36xZs2iWWjnlZNP9beWzW0+QNR88E539taayHr9RTL4JCwaqR3Hex+jLnuaTpBMf7qq+\/PJLwRTn19UKa\/9Yo8cyl3fffVf4UhsTG0OGDBHGERnr69+\/vzRo0KAQBBBgoUANUAQUAUWgmAgknfiw2BYtWlTIErn66qvNGj3W6rH9DX0ZL5w2bZpJm5eXJ23btiVYRRFQBBSBlCKQdOJLqbZauCKgCGQSAqHVVYkvtLdGFVMEFIFUIaDElypktVxFQBEILQJKfKG9NaqYIqAIpAqBIInPt02sB8SrC95dGjZsKDg+2LZtm2+eICMT1ZelO\/g3pF14r8GrNVv3gtTdWXei7XDmYS83XnvCcn+K0oYPP\/xQWrVqZdaItm\/fXr744gtn0wI9T7QdOAbBgxLvCSskLrjgArMdNFDlE6icPf0XXXSRsIwtgeRJTRJa4mPr2gsvvCCs9fvPf\/5j3N\/jFzCprU9iYYnqy4v24osvmnbxkjELDqm7F3InUbUiFZVoO2yhbEtkzSa7dWxY0MdE27B69WqzVfKBBx4QPAmdddZZwo4jvBIF3QbqT7QdLA+bP3++fPTRR7Jw4ULzJcSRI0dSRCiFH3\/W7\/7rX\/8Slr4FoWRoiW\/69OnCmj\/2iFasWNH493vvvfcE5wZBABWvzkT15UafcsopZq8rO1rOOOMMgTz49YtXRzriE20HukAQ7L458cQTC21JJD4oSbQNLJpv2rSpsFVyr732kquuukoGDhxotlAGpbuz3kTbYfNwP+z5wQcfbE9Dd+QdZlkb+\/l5B4JQMJTExw1cuXKl1KtXL4oJN3LTpk2hJL6i6NujRw9hTaNt2NKlS4WXzu5\/tuFBHIvSDvT74IMPhO4te7K5DoMUpQ1Y3Dt27JAOHToIww4XXniheb4ikUjgTSlKO1gXy\/dx8JzE3njW0Z5\/\/vmBtyGWAhgybGW95ZZbjNu7WOlSGR5K4uNhhOTY1pbKxier7KLoy02nXZj7dOUHDx4sV155peD7MFn6FLecorSDcUk89mAl\/elPfypulUnPV5Q20D3HKQZdXTwEnXTSSWYsmTGzpCtWxAKL0g6GgNgVNWvWLNPVZfyYryJCnkWsttQkDyXx4Q2Gb\/ryYDrvBJZRJBL8r7FTJ86Lqi8+4fgsJ84cHn\/8cWNxUE7Qkmg7GHRHdywlry2GQbYj0TZYHbt27SqNGjUS8rHN8vvvvxec6dr4oI7ok8g7wNgwXWKsbnxg8qPKt7C\/\/fZbwRdmUPqHvd5QEh\/dPvbyMh5mAeSBxJzHYrJhYTkWRV\/G86677jrjsuvNN98U3H1lWjsYo+HesNeavdidO3cWHFNgMaVkhq4IABXlXuAcF8vKWXyZMmUkEgn+xzXRdjBOVq5cOWcTzDnESZy50D+FEChTKCQkAcywMQCKdcQvF0tA+LQlD0RIVCygRqL6vvTSS2ZP8iWXXBKaQXRnQxJpB1bFhAkTzB5rPHIw83766afL7NmzzQepnOUFcZ5IG9CLveF8NIsxMbq3L7\/8sjDZEe9zCORNhyTSDnpBJ5xwgrzxxhtmkox28J0bPgyGz8t06JmJdYSW+I499lhhgBZrghvbpEkTYZ1VWEH207dv376CMOaybNky4QNLDKZjLSHMXjNGE4a2JdKOMOjpp0OibSDdNddcI5deeqn5GuDGjRsFazwSCd7io33oF+sd4HlCSPe3v\/3NfPeaoQfG9+j+3nTTTaGwXNEvjBJa4otEIsJCTNYlMfDcr18\/Mw4TRhDRKRKJrS\/fIUb4dcbrNFaSU7Bsq1atSjGBSyQSvx1uJbEu8LKNJeiOC+I6EkmsDZFIROhFsKaM2fWHHnoosFlGL5wiEb923Cs8U+SjW3vFFVeYiQ3awbrKWA59SR8W4ZnnsxY8P+nWKbTEl24gtD5FQBEoPQgo8ZWee60tVQQUgT0IKPHtAUIPioAikPEIJNwAJb6EodKEioAikC0IKPFly53UdigCikDCCCjxJQyVJlQEFIFsQaA0EV+23LMC7eDrdXY9V4EIj4uipPXIHjOIBcD4tMMBA04LYiZ0RaA3OrmCA73ExRNrK9lRk6pdKJRL+QjngTa4lFauxFdKb3wym43Dglq1agkuqtK9lo99wzh8SGZ7zjnnHFmwYEHKdqGwbo1dLi1btkym2lpWERBQ4isCWEElZdN87969xXrY5ZwwrJNhw4YJ3jmstcWCUKwvrBacnOJYE2cPXmkXL14sbNJnFwkvIXkhEq92ssiXnTOU60xLuXiXIZ6j2+KjPMolD\/WceeaZBbwcQ5p4oSaO8nEVRf3ojO60gTppE+VQHlZSx44dhbYTjsUJHuCCB2LqevDBB41\/PasPnktwQkBZ1IO+1OMn5AVXrFKL03nnnSdso\/TU3kRZAAAFZklEQVTK57ZgwYb8lOOVXsOCQ0CJLzjsE66ZfZi8PHjZZScLG+nx6oIfNna0YKGwd5aXf9y4cYKbJXaGcI7rK1bzu9Pi+LRPnz7yj3\/8w3gffuKJJ2TEiBGSl5dXSC+8E+OQ4J\/\/\/KcsX75cnGkpF0sPsuHotvgoj3LJQznslMBlEu2hIiyf22+\/3ejwl7\/8RSAstlyxW2fKlCmCt2rqHDBggEBAGzZsIJusWbNGcGKJT0C2M9Jm3GOBEa7w582bF\/WyAlEOHDhQqBcsaAftQR9TWJw\/r7zyirAbAp3atGljth+CdZxsGh1iBJT4QnxznKrx8tP9YtsbW5VuvPFGZ7Q5Z3M97qLY4wl5YB1Bkl5dQQiDLUM4F8CLR+PGjc23J+bMmWPKcv7BWjriiCME7yuRSERI265dO4GYnOm8zklDWvJQD9YdPvw4Jz37S3ELxTV6\/\/zzz4K1h7sryL127drGOSjbsnbv3i20iXyHHnqosI+bcMgQN+24Y4J42ZzPN0BoO2mxiFu3bm08LVMPJH3YYYcJliLx8eSvf\/2r8ZhNXdSJjitXroyXTeNDjIASX4hvjlUNi65Xr17GSSbWDd0t2yW0aThCduwFxsMIlhhWCi8pcW7BYsH1Oh576f4h48ePl++++86dVOgSUy+kayMhGAjKXnsdseq+\/fZbIa2Nh5QgIaw1wpxxXFvZunWrYAkyAQDZPPXUU4YQbTy6WHdM6IwuuC2z8XjsrlChgrmkrXj3oTtNOyFavLKQzySI88fpCZw6qTtOFo0OOQJKfCG\/QahHFwsPNVhPEB5jVIxv4RePeCu4h6KrN2PGDMFKwzKEaGy884j1wngb3T26xchnn31mvjnhTMc5VhcWp7W2OHJdrVo1omMKJJGTkyN4pbGJ6KLiDABis2Fex4cfftgE02XlWyu4KrdkaSIcfw488EDButuyZUs0lHogPAJoa8+ePaNutND9k08+Ebr5xBcSVwC+IG0Q\/vuwoLEcbVisozNfrDQaHgwCSnzB4F6kWiEySAwLioxYcXjnxfLgpXZ2AYnnxSTtc889Zyw44gl3psXq+vrrr40PPYgM4mMgHuIkrVMgXbrAkAVp6VZy3bZtW2eyQufoRzf33\/\/+t5kQwFccY33Uhf6FMrgCqAvSxB8j1ihkybUrmfGocsghh8gzzzwj1IGzV9LbdBD8W2+9Zbq2lEk7\/v73vwvWqE3jd6SrTJmUPWbMGKlRo4bp+rrz4BEFK5I6IF0mY9xp9DocCCjxheM++Gpx2WWXmbEturDM7DIhcOuttwpOWSElrnmRmbWsU6eOEEZXl3O+4nb99dcL\/v4It2kZI7vrrrtkyJAh5kM7kB5jb8ySupVh7I1PAeKrju7iDTfcIDfffLMQ7k7rvsY6ZYyM7iq+4rDEsN4gRXda5zVulhi7o4vN2B3lYHkycwoBOdNiCQ7Mn7xgtpX0+LBjTNKmoU1ggN899Gdig24044g2jd+R\/NwDyqYO6qJOMMWXIrO35L\/88ssFq5mP\/rh1cKclvUpwCCjxBYd9wjVjSTBex4wk3TQ87GJ1UACTBlhfzOrS9bTpWK7RpUsXGTRokGAxMpHhTMvYGoP806ZNM11A0jPjGokUdsIZiUSMo0vSYK0xU+tMC8lSP2Wik1OwMiExZqPRn5lfSJc0WLHOL845yyEN7aS96EgcVhz1NG\/eXLCmaBPlYGHRrSae9OhH27AqqT8SiUT97qE\/5RFP3kSEdXfkoWx0QjfyUT\/YohvXfPNi0qRJwnegGZa44447BH3BxZ2W9Jkl2aWtEl923c9S2ZqdO3cKy3pY90c3n64xhMPSE6ziUgmKNtoXgf8DAAD\/\/zkz5ogAAAAGSURBVAMA7ZyvQU46Yj8AAAAASUVORK5CYII=","height":191,"width":318}}
%---
