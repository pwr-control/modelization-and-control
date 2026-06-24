clear;
% [model, options] = init_environment('inv_im_ekf');
% [model, options] = init_environment('im_inv_model');
[model, options] = init_environment('im_gyro_inv_model');

CTRPIFF_CLIP_RELEASE = 0.001;
s = tf('s');
%[text] ### Global timing
% simulation length
simlength = 3.25;

fpwm = 4e3;

fpwm_afe = fpwm; % for PWM
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
tc_factor = 100; % tc is ts_afe / tc_factor
tc_decimation = 1;
delay_pwm = 0;

dead_time_afe = 0e-6;
dead_time_inv = 0e-6;
dead_time_single_phase_inv = 0e-6;
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
use_thermal_model = 0;

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

% im
use_im_encoder = 1; % 
use_vector_control = 1;
use_uf_control = 0;

% load
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
cllc_pwr_nom = 250e3;
psfbc_pwr_nom = 275e3;
fres_dab = glb_time.fPWM_DAB/5;
fres_cllc = glb_time.fPWM_CLLC*1.2;

hwdata.single_phase_inverter = single_phase_inverter_hwdata(application_voltage, single_phase_inverter_pwr_nom, glb_time.fPWM_INV);
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:5241b3d8]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:78925dd3]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:0cb6f422]
hwdata.psfbc = single_phase_psfbc_hwdata(application_voltage, psfbc_pwr_nom, glb_time.fPWM_PSFBC); %[output:4f7978c1]
hwdata.three_phase_dab = three_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:944891ec]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:3fca8e19]

% modifications
hwdata.afe.CFi = 2 * hwdata.psfbc.Cdc_dc1;
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
parasitic_dclink_data; %[output:0cc84fa7]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:9750d0aa]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:5fdf10dc]
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
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:38fdd183]
psm_ctrl.kp_i = 0.25;
psm_ctrl.ki_i = 35;
%[text] #### Induction Motor Control
im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:9dbfb0f1] %[output:836c868a]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:9dbfb0f1] %[output:836c868a]
%[text] #### AFE control (with sequences)
afe_ctrl = ctrl_afe_setup(glb_time.ts_afe, grid_emu.omega_grid_nom);

afe_ctrl.kp_udc_ctrl = 2;

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
psfbc_ctrl = ctrl_dab_setup(kp_udc, ki_udc, kp_idc, ki_idc);
cllc_ctrl = ctrl_cllc_setup(kp_udc, ki_udc, kp_idc, ki_idc);

psfbc_ctrl.kp_idc = 1;
dab_ctrl.ki_udc = 35;

isop_rail_ctrl = ctrl_isop_rail_setup(kp_udc, ki_udc, kp_idc, ki_idc);
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
%[text] #### Sogi
sogi_delta = 1;
kepsilon = 2;
sogi = sogi_filter(omega_set, sogi_delta, kepsilon, glb_time.ts_afe); %[output:45d673a9]
%[text] #### Current control parameters DQ PI
dqvector_pi.kp_inv = 0.5;
dqvector_pi.ki_inv = 45;
dqvector_pi.pi_ctrl = dqvector_pi.kp_inv + dqvector_pi.ki_inv/s;
dqvector_pi.pid_ctrl = c2d(dqvector_pi.pi_ctrl, glb_time.ts_inv);
dqvector_pi.plant = 1/(s*grid_emu.trafo.Ld1 + 1);
dqvector_pi.plantd = c2d(dqvector_pi.plant, glb_time.ts_inv);

G = sogi.fltd.alpha * dqvector_pi.pid_ctrl * dqvector_pi.plantd;
figure; margin(G, options);  %[output:6c0eaee5]
grid on %[output:6c0eaee5]
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
% ABB 5SDF 0131Z0401
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
nominal_battery_power = 250e3;

% nominal_battery_voltage_1 = catenary_nominal_voltage * 1.5;
% nominal_battery_voltage_2 = catenary_nominal_voltage * 1.5;
% nominal_battery_power = catenary_nominal_power;

initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage_1, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:67464f46]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage_2, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:2eabceb5]
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
%[output:5241b3d8]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V\nNominal Voltage: 690 V | Nominal Current: 2.324276e+02 A\nCurrent Normalization Data: 328.70 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:78925dd3]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:0cb6f422]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:4f7978c1]
%   data: {"dataType":"text","outputData":{"text":"Single Phase PSFBC: PSFBC_1200V\nNominal Power: 275000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls1: 3.030303e-05 [H] | Internal Tank Ls2: 3.030303e-05 [H]\n---------------------------\n","truncated":false}}
%---
%[output:944891ec]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: Three_phase_DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 1.200000e-04 [H] | Internal Tank Cs: 750 [F]\n---------------------------\n","truncated":false}}
%---
%[output:3fca8e19]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.580123e-05 [H] | Internal Tank Cs: 4.734509e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:0cc84fa7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANsAAACECAYAAAAZW15iAAAQAElEQVR4AeydB5wURfbHXyuei+FERGU9iYYjSGYliLBw3qGgSJAVvAMJEgThEGXXSFCQpKAnyCoChyQlKOFABBQ4QWUNsCp\/EJAgEgxHEFGM+99vYa3tOLM7M90z0zNTfHjbPd3V1VWv+9fv1av3Xp2SZ\/4ZDhgORIUDp4j5ZzhgOBAVDhiwRYXN5iaGAyIGbOYtMByIEgcM2KLE6CS4jeliERwwYCuCQea04YBbHDBgc4uTph7DgSI4YMBWBIPMacMBtzhgwOYWJ009hgNFcMCArQgG\/Xra7BkOOOOAAZsz\/pmrDQeC5oABW9CsMgUNB5xxwIDNGf\/M1YYDQXPAgC1oVkWu4AcffCD16tWTq6++Who3bix16tSRDh06yP79+4O+aXZ2tqxatSqo8pSrVKnSb+730EMPyddffy3Hjx+X++67T7Zs2RJUXaZQ8Bz4FWzBX2NKRoADNWvWlJdffln++9\/\/Sk5OjlxyySXqdwRupaqsXbu2qp\/7rVu3Th0bNWqUnH766fLII49I5cqV1bFw\/uTl5clPP\/0UzqUJfY3nwMbX9cUXX5R27dqprzxfevY5xrmEfhq\/dO7777+Xr776Si644AL54YcfZMyYMUryIf3Y5xg0evRoSUtLk7\/+9a\/y3nvvqat50WfOnKl416hRI9ESS50M8Kd48eLSpUsX2bx5s2zfvl169eolSNtjx47JwIEDpX79+nLllVfKo48+qtrD8f79+6tj\/\/jHP6RHjx6qPNK1T58+csMNN8ibb74pq1evVu1o2LChXH\/99fLhhx\/K559\/rp7tkCFDhPYhwZ977jlp1qyZqu+1114L0Mr4P+wZsPGSvP7669K5c2c5ePCg+rrOnz9fIB4yQOPBLl68OP657qcHb731llSvXl0qVqwoVatWlS+++EK9qMuXL5e9e\/cK0gc6dOiQ8OHh+KeffqokIb8BJ9VSBhXw1VdfFfh55plnyqJFizhVKJUsWVJKlSolAEkX3LBhg1x66aUKONzvo48+kgMHDsiUKVOU5OM80vDLL7\/UlwjtmzNnjurLihUrBOC\/8cYbCszUQcGjR49K3bp1Vfsuuugiod6VK1fK2LFjVVt\/\/PFHiiUceQZsPORTTz1V5s2bJ3wd\/\/znP6svO1\/3ChUqKBAuWLBAUlNTE+4h0CGkx\/vvvy87d+5U0oWv\/uDBg2X9+vXSokULOe200xRdc8018vbbb8vatWvVcaTS2WefraQE9QAAXnb4hyo6ceJE2bhxI6dCpqZNm6px3dy5c9XHDxB\/9tlnsm3bNkFaWZYlf\/rTn6RatWoFdaenpwvt+eMf\/yiZmZmq7Pjx42XWrFnqI0pBnilS2rIsOeWUU1TbefYA\/sSJEwJJAv7zDNh4ODwAHiRf9W+\/\/VZmz56tButs+c0DQW1KwOfwmy7RzyZNmqiXjn4DNF0A9dH+Wx\/XWwwcvOSAFgIgQ4cO1acDbpFIfPCQbrrQ1KlT5emnnxaAe+eddyqJiwbCeAyQUI591F72Cyh\/B+PO7bffLki9jIwM6dmzZ\/5R\/\/8L64\/\/K+LzqGfAhuqAHn\/rrbcq\/b5bt26C\/s5Yga\/1uHHjhAcdn2wOrdX0E3UwJSVFjdWwHgIyaNmyZUqq8NFhn2OHDx8W1EbuAr9Q2wAOQIWnqJOcC0SUmzRpklSpUkVpE7oc6l3Lli0F4w3gAbhIUlQ\/2kc79+zZI0hkfY3e\/u9\/\/1MSjrEaEku3T59Pxq1nwMYLw+B56dKlSpXka8ngvnXr1mqQzziOr2SiPqRNmzbJddddp8ZpDRo0UOpjVlaWtG\/fXnhZUSuhMmXKyLXXXiutWrVSx1Hn+DBp9Ro1EyCyRaXjWvZ9+YZBRd+PegERxhB7Oe49fPhwpeYxpmJM+fzzzyuDCACun284YZrgwgsvtF+m9hnrnXvuuYKEvuWWWxSQc3NzlVqpCiThH8+Aja8kqiQPnS86JuhixYqpR4JaxW\/KqAMJ9ocxD9KbFxhTPMYSDAtIEFQs1ELOQ+xzDD4NGzZMjd9eeukleeqppwRQca5fv35CWUiXt7OMclu3blUGCu737rvvCuPDs846SyBUR9qEWs+YESkGqCZMmCAjRowQJCcfAup\/4okn1LgL9bN3794CcS\/ah+WUumkfHwQAC7AZFjBuoxzGL9rDPvfk3rSB34lGngFbojE2kfvDhDjgu+qqq4TxGFZiLVkTud9O++YpsDGg52uNOfnjjz8WtjxUjnHOaWfduj7Z67niiisEdR+ph2TEapnsPAmm\/54BG+Zi1B\/GGX\/5y1+UWsOWryfHOEeZYDplyhgOeJEDngHbH\/7wB+GLyeDeH3GOMl5kommT4UAwHPAM2JjnQXrddNNNyvSv3bRw1cId6fHHH1dOssF0ypQxHPAiBzwDtvPPP18wK7\/yyivK1w+nXKxzEO5IMA9LG1tDhgPxyAF\/YItpP\/RkLODTDWEeh4lXzuljZms4EG8c8BzYzjnnHPnuu++UmxBeC9CMGTOESW5jIIm318u0184Bz4GNyeuRI0cK7lsdOnRQHhR4tz\/88MPCRKm98WbfcCCeOOAZsKEm4pIF8\/AkwUOBORzmcvBawPWHc7oM+4YMB+KJA54BG65ZS5YsUe4+uAERv6YZyT4hJbgCUUYfN1vPc8A00MYBz4ANSyMRvw8++KDgzY5XAoGU0N\/+9jflmItjMmVs7Te7hgNxwwHPgE1zjGBE3LMIkMRFC8LxFbct7byqy5qt4UA8ccBzYIsn5pm2Gg6EwgEDtlC4ZcoaDjjggAGbA+aJmIsNB4LngCfBRsozopGJ8iU1AvkrmNwOvlumpOGA9zjgObAx30a07oABA1TmJpKFkoeQxKGc8x4LTYsMB4LjgOfAhv8jXiRkdKILpERgGoBUCZzjmCHDgXjkgOfAhv8jE9wkA9UMJasTyX7CcdcibwkZd8k6Rf6L6dOnm9TYmrFmG1UOFA62qDbl5M0AFDktyHVIshgyQN1xxx1C3kKAeLJU8H\/JIkWiGUJ3CNthDMgcXvA1mJKJxoEfv9gt0LE1\/5YvJnaNWvc8BzZ6TlQ24CBfPCmrmdSuUaMGp0ImgEWWKHwrAStR4O+8805BPXioGKqo0p4nKh8aViort6ddIJOanS0vXVtcPulTQdEH43vI9hUzf9f3gpfD5R3PgI0syHj5E6ENYYls3bq1WoSBMRvnKFNY\/wEleUtQGUnZjVVzx44dsnv3brVoA2okefN37dpVUM2R1lPEUGR4cEbn6XLzIwtUAlmStEaF5k+XV0f3kVW9asuqjAtlVZ1dMrPaXhlU63RpfmkJqdG0pVgZY8XqNVtSH9+myLddfHQKXhAXdzwDNpKJPvnkk0JeeRKQdu\/eXUVuE72NWgl4yE0YqO8keeV68hCy5BKpr\/\/1r3+pBSoAHZISNZLVWRj\/6XpKLOwuh8Y1DZve7Vc+7Gud3Nfr1y7uU0s6NSgjL+84IXWe3C1tZh2Ut74sLqzb4CaV2rNWzlr2kJw+vZvkZVaUvJGN5dTVE6XYnnfl7DKVpOK8vJP0zF6pCGXNlwrt75YK13QM2Bb9bri99QzYsDoSnc2WMBpyj5CkFCKjLimu7SDxZQRSr3z58nLZZZepU6iipMAmEBWQajUSUOs89aqg+RMRDjTKlyJZzcvLpgcaqI9Rx7RU6Ttni9Qc\/qaMfmV3WPfUY6z9Q5vKzvaWIsZc325eI6edX17O7ztNLhq6Wso+tUsRv8O6UYQu8gzYdP\/IoAWoUP\/0MVZO4RhA1Md8t5dffrkQdEo2XcZpN998sxAXRzQBzsyM1UiLhwqZl5fne3lMf9Mmpw1wWofT62l\/YXUAPKRxx3zQjX5ll5QcuNov6Ox1HJ47TIoCFpILcAGss9O7SErVdLHXQbu8Qp4DG2kRWFyDZaOuvvpqtWRRx44d1ZJRSCXNONYFaNKkiRrc3n333fqwkLq7a9euKhNXZmamMI1A1Deg+\/nnnwXyBRsPJ1wiijzca\/V1XqgjWm3IuDxPUL2HXlNKZry5V4HunkEPyM7RN8nOexooVVBLrcOrnpUTxc87OcYas1OsfPpp0GvydYvB8mW5JnLgjHIKWJqPeuu0LwUvk8s7ngMb\/WvatKlaf4yc8OQfIcMWK1NyThPhNmvXrlXrmTFOA0DTpk1Twadt27ZVi\/Gdd955ak4NqYfxBOsma5b5qpFOxhAXX3xxQN0\/2Hr5qARbNlA5p3U4vZ52BVvHWfljrJty+svizY3lnb3NpOfuEbJ\/03pZcvAcmXfbrpNjLMZa9jFWhQpB8znYdtBmf6TfMbe3MQcbK2YipSD2GXthecSq+Pe\/\/11JNPJGcoxzgRjA14wF95iXwzhiWZYqCrCOHDmiVtTEA4V7AEx10vxxnQOofoyjIFRAzOxaUrHl9w\/581yMsVABNZWduEtOdMyWOW8fUNKOsZ3rjYtxhTEBGwvokSseQLGCCYlZISKyMYbceOON8sILL6glfidPnqyStuq5Ms0vwGlXI1EfUSPIMYmJn9+ooyxzyziwVq1aAqFOIvF0PZiGeQnCJSxgXGvIku3dUwXV71juKjm2d6tSAX8sV0epf6iAEGrgd7dOVaogz0vTT0cPCCrmk01+lCW3XizXXZqiQHfF0Nf9qor6On9bNCF\/x4M9pt8Nt7dRBxv5REhvgMFj4cKFkpOTIyT2gQAIywudOHFCLU2EqoiPJOuGMdZCKmkGcM6uRmIMQWKxOCBbyrE4B+M1rJIYTTZu3CjsA0DOQ9e8W+FXtQXVJUTiBdJf52TfXjblgCjzOurfqDelYr6ZHaoQogp4VY3LZGSHWsqKyfRBm\/xpA6YPHn3rRFCqpFEjebPzCXesBx54QJA6eHTkH\/rNf6yJGDhY90ufYCJ63759+qffLdEBMBnvEEBbr149tdIK0wkYSSAuxKIJINk35H0OYMVk+gCitSXzrZiomOFOH1BHrCjqko2XHVWO8RfjMLxF7IQhpGbNmsLKlczkQ4zdateuLcyVBWLU0aNHBdUSFbJ+\/fpqkUAkG1ISUKNCQiVKlFAJX+31BKte+CvHWNHf8VCOOVV7uJfTOpxe70YbCqsDNfPu+ikFKqaePrj3+Y0yf92W36iaTvtifzfc3I862HTjAQ7SB7M+XiJz5swRAkYZx+F4TAwb82MQuSP5jSqprwdY9jEbDwpwsaYbW8Z9rKbJHB2A02okMXGosLoetqGoOb5l3bBGeqEOL7QB3hbVDruKObFjZfkqL0V6vnhQeajM3WYpNbOoOrhPYcQ7EQkKFmyu3xv3KlTDTp06CV4iZNXCRQvQAC4AAqCgrVu3CnNpjLd0Q3zHbBg9GIuxpUzp0qXVHBtpy73uQcKDp81OyGkdTq+n7dGuo2NaaQFwqJgdbZPlgO6TQydokqcoZmAjGBRAIXk0Rwh\/wXdx+\/btQgwby0RBzLVhtbRPautr9LZ69eoCwBgLMpc2b948YUsgqjH9ay4l5rZs7wTnYwAAEABJREFUyRRhbIeHSlbzCrJ+x2HlFsbYzks9jhnYGEfh\/4hkw7DBuG38+PFCfkiAxTiNVAiacL\/SRg4YiMSzq5GMAVEzy5cvL6mpqcpVi\/EhAK1ataow\/9a8eXNhH0lHHZqQpuGSGbPtKhgvOR0r8Qyc1sH0waDqXxeM7TCoMH3A2G597vaCtnKvQKTfC7e3MQMbHWH5Xrw+Bg0aJIy1+I0RA6mG9wgA1IQxBUBxHeSrRjK2Q43Bsx81lOhsJCfSrkyZMrJu3TrBiwR1FcsldWhinBguIUnDvVZf54U6vNAG+OFWOzq1aS4v3NdOShDVkfOiTF6zQ26Y\/qmkjX5HuE9hpN8Lt7cxBduzzz6rxmLZ2dmCFNq2bZts3rxZTWgz72YnjCiY8QMxAEABRq2WEuVNeaTfxvz5NbxHIPaxdup6AKmhncrtLUp8iPq99i8bL19mt1cRCIPapAV1f\/1+uLmNGdgOHTokubm5ylMfczxSacKECbJw4UIBdMSmoVJqGj16tNgntX2ZgP9jly5dBA+Uhg0bysyZM4XJ8Pr16wuA02okfpdMI\/heb34nPgf02C5WPY0Z2HDZSklJkTPOOKOg74zjLMtS4zZM9ITEaCL6mvm5gsI+O5ZlCdMDSEPURdy9ALBlWcq\/UquRBKJaluVztflpOBB5DsQMbBgpUPOmT58uAAtpxjpsuF2VLVtWCLFp2bKlaEIi4X0SeZaYOxgORIYDMQObZVnSv39\/ZZ7HfxF\/SebbUBtZQxsjSWS6bGo1HIgNB6IONtRHDBmY7jFY4DUyZcoUNcZivEVKBOLOsD6iOgayRsaGXcHc1ZQxHPDPgaiDDcNIv379hJCa66+\/XgBTu3btVBYt4taYwMZsj8uVPSKgKGuk\/+6Zo4YD3uFA1MHGOA3gkO0KcDEvxkQmRCwa7lVkxUKdtNOwYcOEMoV57K9evVqBF59LLJFITsJtAC9SEidlxohIV+88AtOSZOFA1MGmGQsQMIwAPn2MsRpgI+YNZ+EGDRqofI+E2FB2yZIlMnz4cF38N1vKoI4yGU6IDVm2+G0yIv+GTeZHDDkQM7CR2Ic0c6xYw3JQECokoMLzY+zYscqUT6JWYtsAIekOUEP98WvHjh0CcHHVsixLkGIYWZgGQNIRZcDUQlpamhDz5q8Oc8xwIJIcCB1sLrUGB+GRI0cKlkiMIe3btxf8DO+55x51DHDpW6H2Ie0oq4\/5bsuVKycsxgFRfu3atQIAmVIg5EKXxzmZDMn6t9nGFwcOzx0mmshzEimKBFeiDjYkFxZHOsOcWlZWlkqLgD8j82wEi+LhgdEE4DFuI3K7bt26asyGYzHX+hIT2DgvA1y8RrhHpUqVhIQ\/ANu3vP7N\/QxVVCkBo8kHe+598u8XlcPljRtPE2jLrOGiafuKmbJuyfMRIf1+uLmNOtjw3Gfs1bt3b9mwYYMgsXSH2Eci5eTkCCnIidrGg2TMmDGSkZGh1EpSKujy9i1SD9URIwqBokwpcC8ivrFs6rLsV65cWf9UWyc+geSJd3J9MlybO3WwbLiriazrXE7l3ifJ0r1pZ0rLSqWk0Q0dFBWVv6Xhoh\/EH7VZ\/q24TXdtS1Xvhdt\/og42XK569OghDz74oCxbtkzwDNFfVLJrIeEIu8EPcs2aNcJvDB34PdJ5rmfrS6iPjOkISEV64q5FqA5GFsDNWA3gsk\/sm+\/15rd7HDi25t9qKSYtrQ7nq34\/fLFbildJL0iuRBZjiEzGkHt3925NUQebZgWhLsPyzflIIf11x5iBVwmmesZWGEOwKhKBDUAxgOjrfbd4\/TNvhwRMT08XfiPdKIfUA6TEu0H85rgh5xzQwNpvy78PuKhZSytAddHQ1XJuxhAO\/0pJthczsAXi8zfffCNYDRmzIZVYIINJcKQc0wWBrrMsS3AyBrCop5mZmQLAsDy2adNGxbLhmUJuEsJvAtUTi+MEMTq9r9M6\/F0PkACONkIAKEhLLLac\/3bzyYUtWIoJgAGucKWVv3aEyhs36gj1nsGU9xzYcDbG9E9cGgGie\/bsUbFux48fVw7LwXTKXgaLZGHWSMYPvDThkhtJWr1Qh7825D59r2x+KVvsSVdV7v0xJ\/PukzOTpKsQ+fc\/LV0vqEhowBCIsEgHOhfscad12N8fN\/c9BzYt1SZNmiQkaJ06daqgFgK8QJbIohhSmDUymZO0zrttl8qtr7ePtf0\/0TQg\/T0Z02iZ3FH1Ral71nNS9\/tHpO5nfRWRMJXEqSTWwQqsifQWej\/crRfqKOp9Cvd8TMGGQzLmfcz1qH8Ej2KRxGjCYhkYThYtWqQW2RgyZIjKllVYR\/25a5GTZP78+coTBWsliYAAcWH1JMu5rOblVaIcvSVTlabFfWsJROYqEunYKat5BemYlqoS65DjA\/Jach0vPsOYgQ0jBdHXGEJgDEGk+EwSZc3XDQdliFwRjLPILwk4KeuPArlr4TmyadMmWbBggQA0FlUk97+\/Otw+Fmx9qEfBlg1UzmkdoVyf9QtIASMgzMoHH+0CdK0mbmQ3bAqlHYFu4kYdgep2cjxmYMPSiGUQMOmJZ6yOlmUJ3iQ4DCORNJEmoTA1krEZ1srU1FSxrF\/dtcg1Sc4RwIrRhX2mFZwwze1rUbmc1um0DifXZ+WDDwm4uE8tWffxEZVGLtz+OGmHvqcbdei63NzGDGwYQkjyg\/WRDpFcdeLEiUIuEbw\/UC1JzIpUwo8SIFGesv4okLsWOShJl6DTIgA4jC7+6jDHnHGg0aUlVFKdsuemKMCt23HEWYUJdnXMwIYhBA+Rvn37qgQ\/AIvckN26dRNyReK5zzwcW8ph\/rdLJIJPSeTDuI5r+ZqF467F80TtCJecWr64L14vbJ2Q0zqcXk\/bqYOc\/E+0KCHnFxdp9dTG3+Xhp1xhRB2FnQ\/mnNM6eCciQTEDG50h49V\/\/vMfITkrc2Tsc4xzqJiM69j3R1gnce1iQhxjCmUxgMBoJsqZ0C5WrJhaoAMXLV0H+77uWgA1XGJaIdxr9XVeqMPtNqy8q4E0uqSEysO\/76dzVQ5+3d\/Ctm63o7B7BTqn3xW3t07A5rgtWA+xMpJIlTTjAASjCflIMJQ8\/PDDglMxackZs+G4rG8KuO6\/\/34hXTnHcNdiXIZLFtcMGDBA6tSpI0yM46LlZXctHjp9cEJO63B6PW33rQMDCoBDwvWds4UiRZJvHUVe4KeAG3X4qdbxoZiBjSWe8F9EjWQMBSiQVhhGMIRgmURiYdCgLJZIQmfo8apVq1R6OqK9+Q1hXOE8IMzLyxM8\/kn4yjmO4U2CtIT4zXFDkecAgMNwsj7fcOKGtTLyLY7cHWIGNgJHWfiQ6OycnBwhrIZkrVgmWT6KNNTkKdHEmA0LJqzAmMJxjCL8hizLEkBKMCrjPCQchhXjrgV3iibGQkWXKrxEoDq04QSLZZmSKQLoIObmmCpgYUOI2gPVwblgyY06gr1XKOViBjaAQIDoP\/\/5T8FnsUaNGiqEBm8RJBfJVu2EpMMiSedYD4AIAXJP8hsCaFgwMbIwdwdgUSuZEmAcQBmIc7t372a3gDCyhEtMXYR7rb7OC3VEow3pdavIy\/e3lBILuyv6\/I25krN6qTw2a7noxQ3xTgGIvlSq9zzxRxd0zhZfqjfkld8d8y1T2O+vG2UWvBtu7sQMbLhQkU8EQBCvpsdjjNeYFrB3EmsjLybWR6yQ9nN6H0MJ0QIkEMIIwjWooaiM3EuX891ynaHY5PonB\/\/B+Q\/I58\/1FibHCyNy9UPk6rdT++uaSCTI9z1x43fMwEbjARUSDbUQ52M8RwAe4zPOawI0AALrI+M6fdy+5TwqKdmULcsSIruZ0C5durQAPl2WfV9rpD5ntt7nABPodtLuZW5vI8GJmIENiYNkY8KZIM+2bdsqzxHAwdgt1M4SvwZIiRbgWsJoUDuRhqxcQ3gOxD5GF8pEikhehMRGNVuxYkWkbmPqjTMOxAxsLPOLwQOL4l133SV4+c+aNUuQcJZlhcxGPE+I5r7llluEaYNYrmKzfPlyIe0DeTDxXAHkIXfIXJBwHIgZ2OycZPIZiUT0NvkiAaH9vL99LJlYHqtVq6ZOW5alDCwYVYggYFqB+RbLstQ0AS89x5k8t6zQwaxuEuQfJCz3xgjElAQfliAvNcUSmAMxAxuWRFRG0tnxYrIwIp4kqGCM5eKV56jHzPexHBZ9wPfz6NGj7BpKcg7EDGyWZQmxbPfee6+QRJVkPUgrkrPiN+nF58I4cPDgwQVNIxU6Hi94p+AqxpQFztKE8AA6CtIXDEDsG0puDsQMbLAdb46PPvpIGOMQNMq4i6SqZMfivFcI1RbnaLxdkFS6XbQbR2TUU+YBkc5MqDLnR7QB6jDl9bSGvs5sk5MDMQMbgOLlHTdunEpXR8o6CG8SJIaXHgfSiqkEEgfZ28VURIsWLVRiIcabrC\/Hx6N58+bK4ENcHn6aSGz7dWY\/OTkQM7BhoeMlRBogNTSRIdlrkoBobzxWcGzWrwmSGMMHY0+OWZalsgrj5wnwkHQkcOU6zhsyHHAHbGHwkbEMhhAsd2Fc7olLkHiMzzzRGNMIz3Mg6mAjABSjAuEzAI38Ir169RJy+kOco4zXOYdJH8mM9ZS2EmnAeM0kE4IbhvxxIOpg4yUlGnvAgAFqrTUseACP3xDnKOOvsV46ZlmWSp3OuI0x5r59+4S5QtYW8FI7TVu8w4Gogw3Vi\/AaTOiEyWAgsSxL8HmEcLGijHdYFLglBLsynsPsj+dK9+7dVdrzwFeYM8nMgaiDDSskkoxgUbz0MYZgHNHzUl5+GPhw4hSt20hAKgYd0p3juUK+S33ObMPkQAJfFnWwYYVETSTLFQlUb7vtNuFYPIzTEvg9MF2LAgeiDjb6RDQ2W01M\/MaDZNPtNVvDgXA4EBOwhdNQc43hQLxzICZgI9UcCxISfY2HBTlD2PKb+LNA0djxzmzT\/uTmQNTBhsURczmR1f6Ic5RJ1Mdi+pW8HIg62JKX1abnyc4BA7ZkfwNM\/6PGAQO2qLHa3CjZOWDAluxvgOl\/1DjgNtii1vBEuBG5La+88kpp3LhxAY0YMSLuu4ZHzbRp04QwJNzY8H+1d4p+Z2dn2w8V7OPMzboP+JsWHEyQHQO2GD9IXNVw9dLEYiG6SUQSkM9E\/46HLQBbuHChhBvHRz4awpaYDoqH\/obSRgO2ULgVpbJ89fv06aOyhZGa\/cMPPxRCkXB4JsU6X3+awipApO1DMg4cOFCGDh0qzFEiTdhSBqmCJAG4pPejLPWQeRpgcJ46O3furHLB9OjRQ7nPIVlImEt+GIJmSTUIiO68807R3j5IIIj7aCJPC7GKBNDqY4G2SHHaA5FSnjYcP35cSNZLGkAc1gNdG4\/HDdhi\/NR69uypIryZ0Cc7tF4Ci\/wlrFdAfNyECROEVVlJx8fLTqQEaxhMmTJF5s6dK8xN8nIDnkDd4dotW7YI0eOsYUDHgm0AAAN1SURBVId\/6qJFi1Rx4gqRsJRBqrAYCflV9u7dK2vWrBEAiWpI4iJWhWV5LvxZWRWWRLSqkl\/+UL5KlSq\/\/Dq5sfeRfgIkziDFkej0Ez9Z0mTQLvpC\/wlZolyikAFbjJ\/kM888I3pyPzc3V3QezPT0dEFCsAgILyRff15UMi2\/\/\/77grQj3TovpmVZQkQCwayBugNYeKkBL4uLAF6yQ1MeqcLiI0QxcH9UV5IYASSi6QkjYgyGFCUp06ZNm9RqsdzPN34PEHKcejXZ+0hfyX6tzxEFwgqzLILJ+JXjAD4lJUVYKozfiUIGbB5\/kqhSpMoDYLyoZO1avHix8EKjGurmsyIQqp\/+7btFPcvMzCwANlJuaL7a6VtO\/6Yu+4IkR44cEYDRrFkzJUlRYQmTAoz6mlC3tH\/GjBlCDGPXrl3FsiKbPDfU9rld3oDNbY66XB8GA8KPABkv59KlS+Wxxx5TQaqoeUSIcxzpBzDJLo2EIo07xxlD0SSkBtIKyQNoWPEVdZJz\/giAo54COtL1sT4eCY5YJ4HxIGNJpK3vtSTeLUydtZdHVaU\/qI+0WZ9DstJ2MkrrY4mwNWDz+FNETRw0aJBg5GCBR7J29e7dW1DnMISgkuG8DeiQdqh8\/CaNHhZBxnYAEDUTALFFRSW3JfuBut+yZUslaTBWZGRkCPGHqJoAQDuR89v3eu7BuBOw+J6z\/wbESLVPPvlEMNAAXLaMHxkX0hcknv2aeN83YIvhEyTq298LD5gg3TTGShg2kEyzZ88W8lNaliXksSTPJlINNYwX3LIsIY8LY7SVK1fK5MmTZdSoUSq3JdKJ4xAqJdKE+9MOfS\/uyzFedlLDY4LnvtzLsixliQQMrVq1UmDU1+ktlkskINKQOmgv9enzbLkfbeEc40baD\/EhAWD0lXycTlRU7uM1MmCL2ROJvxujPgI6JCVJa\/31AIB16tRJGNMBfn9lCjvGtAbBxEjnwsrF47lT4rHRps2\/5wBWRKx6vz\/j3hFCn5YsWaIkZWFSB+smc2aWFbrBgzEqKjNS172We6MmAzZvPAfTiiTggAFbEjxk00VvcMCAzRvPwbQiCTgQObAlAfNMFw0HQuGAAVso3DJlDQcccMCAzQHzzKWGA6FwwIAtFG6ZsoYDDjhgwOaAeebSKHEgQW5jwJYgD9J0w\/sc+H8AAAD\/\/3iRmnwAAAAGSURBVAMAU2Wfm7DCHvAAAAAASUVORK5CYII=","height":132,"width":219}}
%---
%[output:9750d0aa]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:5fdf10dc]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:38fdd183]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:836c868a]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:45d673a9]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANsAAACECAYAAAAZW15iAAAQAElEQVR4AezdB7AlRfU\/8L7PnzlhDhgWBbWMiChGXHNWzAqWrlpamFHMZaIUcyBICQoiEiwUxASogAoCihRBQiGCCCIooqIgwcj\/fQbO+\/fO67kzd+69j7e7d2u\/b3q6T58+faa\/0z09PX3nNthgg6s333zzqy+66KKr\/TvssMOuFgcRn8d94QtfIHb1ZZdddvVLX\/rSStbReZVQ+3PKKadc\/cAHPrCS22effaqyQq888jpXRsgJRxo7yDinWpo4snSzW35xjs5zuTyevHx5HNk8PspSz2Fy7JAXtt1226p+kVcchA5lKoNtbKzrpUscSCeXx9FDHx8og5yjc\/F1KEuZ5HKwsy7rnP6QU644upUhni462cU+cY7OycojDiKevHzipJMbBjJkQZhuupyzgz2RXz3y+FxWmcomm8fT5VwaGfkjjmweH+VFOU1y7JSXbfLQGXG5PvEwl+b\/vf\/970+3v\/3t50MpPfGJT0zPe97zqnD8Ofvss6vgXe9614W0m93sZulVr3pVFX\/66aen3\/72t1V42J873OEO6aEPfWj629\/+luYrnq644op0wQUXpPXXXz\/Rl+d1vt9++6VzzjknOTrP0+crWOmYr1Q6\/\/zzq6S2elRC1\/6Zd85Cnecdke53v\/tdmzK9wyi2juPzBzzgAekzn\/nMoop885vfTPe4xz0q7LrrrlX6n\/70p\/S1r32tCrvurr8T\/n7Pe96Tbn7zmye+PuaYY9Io9tMxCr7\/\/e9X4trYPBmqa6OtiOzavsg+6UlPSuovrE27zsLaCPuFA9LIOC+1gU9\/+tNV+zvyyCMre8jliGuk7bMxbM9l8vAcZyJBHrnhhhvmpymUMvjhD394dbFctNe+9rWVnIuBPNVJyx+6yecX7853vnO66U1vWsypUSgrEGXWhbvUI8\/DjjjXsNgQ59M+drF1XJ8jjRsVIFG9Tp\/85CcT37pubn7Sc5841y7WW289wYU24KSL\/eS6AuGPP\/74ShzBggBPfepTq7hoL9VJy5\/IE2L1OkW8Y57W1AYOP\/zwhfauDT7nOc+pbj7yB7RlNua2R5rjF7\/4xYq0Vc8mYqkQFdSYQLl1B4mDt7\/97UmjEH7nO99ZGfztb3+7utuKg9AhvNxxXdkad2jEy\/131FFHpSuvvLKz26Zlvx7HjZwhee+b31jZ+o9\/\/IPIksHNKGxww+K\/n\/\/850kPFkawiW1uQK94xSsiunicw0h3tzy1yal6NY5RaB3upLmOprAhAmPdycJId9C6\/Kmnnprmx79VtDvD1ltvXYXZyubqZP5PkFectPmohf9N9VgQmHDAkNjQuEltH1v7+NxNyl34sY99bNJr5PbUh0t6kVLvJQ9\/5r1ei\/2yjAyN9ctf\/nJrPsM0w7U2wfo1j3NkKLWzJn1\/\/vOfF4bXbvRuWGRznzhHPvAYwrfimlD1bB\/+8IcXLopu090lzxA9D6UQacMuasjUjze5yU2qZzR3Mrpc6DYnhMNKFybIq5y2epCZBqIRqpObkTI08ngWcg6j2DqOz3N76teSz4E9hs4rVqxInl2ck3X9hfn6Yx\/7WDVk0lAf9ahHpVHsp6MLEAiRyEbvkd\/INXRpbqaGa8LDMD\/Zk9yoyTg6F+7SzsiVEO1P2l577VX5RBjiWXN+EmXRvIP0HBXZNBJ3UHfD6DZzIb0WR4iTTg5cHHEuljukcBvqY+Omca67hLsFfYaSynOxo6GIB+UqX7itHmQmBX7Ycsstk0YZjZtu8WzlT\/aIC7DVJI5zaWTIyiMuxzg+f9nLXpbopi98pxyIsowuoiHn8tLJ5b5+3eteV006sH\/SvkYgRGJr3GCEA0iO7M7dvNzEhJtAl+cqdXB0Tpbf2S\/cBbe97W2ryTyy2jl9ICwOXHsjNPaxU1wJ4dM5Q7RcwAWAPE5YN1qXVYhngBjikeuC3Kl5Q83zIuUOO+yw2vgY4ZFNQyFr+KHCyq\/bRrZUD\/n6wiyX2a56fsQolV+Pk68ky06QnqOvz\/nODG6pfPqVdWQ2wxbyfCY9Bx38G3HC4uLcUT46hUeBa+dRQh7XFMGFc+Q3XTeoGDnkMnmYHeyJuGij\/B5xXY8f+tCHFm5a8riBaX+Ozvfdd99qJlyb0DbEDcMcI\/JumzNBXH5BKKnL\/vKXv6zueNKawAhy9MlPztE5KEtcSc6diA3kQOPL4zQoDUX+XGfI5t0\/mVIZ4oFu+UInu5wrX5lkIOSkhaz4UvkRp\/7KJgcRTwcoC4Tr5dVl67roa0I9L\/2grFKevG7kgI66rDhpAflG8XXoc+34kJ56vUsy5JStPGF56QjZOEY6mbq\/XAdx0uiKPI6RL\/TSLUwWhLUFR+cnnXRSNWknn\/w58nLIQjWMzIXWtLBx+YMe9KBqejaGdeogPiZYmoaq5LpjJsmnM1\/3bwe9yfbvf\/87felLX6rGtcayT3jCE9KPf\/zjNP9Gvb81PXK6g+jGZdXFG4qwJ8brhhFtU7LyztDugZmv2300TKI32b7+9a+nz33uc2mrrbZKhx56aNpss83SG97whnT00Uenpf6nG9dNl2DIoJEstU1ra3nL0deGdoahrn\/TEHk5XI+5Pkb89a9\/TR4OX\/SiF6W3vOUt6d73vncy2\/Owhz0sHXDAAek\/\/\/lPH7VLksd7oxLOPffcdG4BJ598cjq5hrocfUti\/KyQNdoDvcj2m9\/8plrT6AFzMBhUDrjxjW+cNt1003TiiScuvLOrEmp\/zF52gfcZXbDjjjumJmy33XapjibZprJKttZl6YxyhKXL95Of\/CR5D\/OVr3ylOgoHcWtumZ2uAx74\/2QbobJWSvzf\/\/1fuvWtb71arvvc5z7p0ksvTX\/5y19Wi89PorG1Heu9R9O5XqUJeblLFWYLW9XP0Oa4445L5513XhL+wQ9+UN0YEPNxj3tcgsc85jHpEY94RNpkk00q3Pe+960mezx3znDNounrwg\/TaC+9yPbHP\/6xelvuxd80jCrptAJgGKyEaMLGG2+cNi7AcqYcnjtXrFiRbnSjG6Vf\/epXnYBYwCeIBiX76VxvvfUS\/WxZuXJl9Q7He0bxt7zlLRNYTXP3u9+9egZmzwc+8IEEXqZ6JqnjiCOOqKaf6\/Gz83N6+yVN6V8vso1jiyHWBz\/4wTQqPBsOgxnHJpiZzKERw1VXXZX0PIZ8eptPfOITKYZ8yNOEqD\/S6M0Rlv4oX\/jFL35x9TwrLB3JIl8cEVC8l6QgjHjSo2z+gre+9a1VT2gFh3DYeeaZZxKfYQ3wQC+ymU63Wvzvf\/\/7oioaXl7vetdbFL+UEdbbRXkarSGd5yVk0lA1WHAuXnrI58c73vGOCaGe+tSnpne9613V7KvXG5YNgQki8atWrUpkyAaQ8JJLLqnyr1y5MiGiG4wbBvLpwVL2L4gn\/8Mf\/vCqB1xvvfUyiWuCUZ+99torsR8MR9UHAaGpPtdo6Pc39+moGkbJ2yY7LL2UNk7cqPVsk+9FNsOcf\/7zn9VHoHkBhl4akUaaxy9FWCME5PFaAqmiEQprlNJKDZG9gDDI45VGkEpYnLSNN954tapYSrRaRO2klI5AyLf11ltXPR8SIl+etU68LbbYIslDtm5D5FN3BAT1zeuOgE11j\/xtx1JdGvPUEkbJ2yY7LL2UNk5crRpjn\/Yim6n+O93pTsm6tniJbZ3bsccem0z\/3+pWtxrbsCYFGhVoPBoREmlc7uzgXFqJVHQilQaLPEiETHopcC5eOtlpA\/FWrFiRECnv9cSna\/8hXjwHGkkYlu65557V5x\/sdd5kLz\/xAwLyCz8FCZ3zn\/Rri5odpuyBXmS73e1ul57\/\/OenPfbYI+2+++7VJw0+obc+7gUveEEaDK55HTCu7RoL4oDGobEgFDjXiKSVGgxSAfJolEjVpbca1+Zx8iMZ4hlqgh5PXOhEuiOPPDKp9\/77719N5Bhy1uumB4RhJOQ3evg0CBh+nZEwPD7ZYy+yMWHVqlXJhbICfIsttkjeve20007p\/ve\/v+SRgFQIowEgEb3RAJyDNDIlxUGqIBZy6alAWHxTw6vrK43x6zJx3iY7LL2Ulsch2cr5Z71nP\/vZ1XCziXjxOuEn8+\/0TPio56pVq9KqeeQk5Ic2AroO\/FwioS8w+pIwr1f4runYJjssvZQ2TlyTjX3je5Pt+te\/fnrNa16TfM+DaIccckg1lZ2G\/HMxraR2QREoJ5WwOGnDSKUxIY\/GU29M4qQZ5g4xY41LCuLp7cBi4LwSeY\/npTriiQuZuBnVCehmhIDAryGfH10zMGNbIiECQtM1y3Wt6+FGslnU+\/SnPz3d8573TBtttFF6yUtekvK7RJ+FyC76bW5zm2oWrS+pECxI1dRASg\/FXS90l7yhq012WHoprUsc4hlJxDNeE\/GQDurEC9tdC0BA4Nd8mI2A0ORjBAQEBDfLGI3Uh6NWF0W5bceSD\/I8w9JLaePE5eVOIlwkm+90LCq+y13ukr773e+m3XbbLblTirv44ourcs34uUCjLkTO10262C6m3giB6MsvuDhpZKpCZ39W8wDiIVsQz1AzF3DNPOMhHTQRL88T1wQBoX5NEBCargkCgptpkBD5gohGL3pC6etab7iIbGYXfRx3t7vdreqBLB96\/OMfn7bffvv0+9\/\/PvmMfZyFyGbU6hdw0qTKe+A04r9R8rbJDksvpY0Th3gr55\/xJkm83HXeqyIYAkL9GiIgkMnz5eGchEiX94bCSOjZcBgJSz6KMkpp48SF3kkdF5Ht8ssvrxYZc1o+he+zdecc5hnN+sg+C5G9IqB7UhUo6SkNHUpypbhR8rbJDksvpY0Tl9dlGsQr2abMYT1hl2dCOrQpBNMTejZEvOgJhZEQyDTZQU8pbZw4OieJRWSzWernP\/\/5tO22265Wjv0frIgwMYJoVor0WYi8mtLZydQ9kBNPzzOJoWYXo5EQ9IKQ94RIaDTDHjdeciWdOQkREfGChIamescgISKWdCynuEVkGwwG1Wp+s4ZhqOHjRz7ykeRB1yp1TpDedyGyrn2G31YTTuP6wbXpqsMjgtU\/L3\/5y9NTnvKU5HkvrrFj\/oxn+\/JvfetbyURZV\/1d5AxHwYwx4r\/5zW9OH\/3oR5PNm+wE9upXvzp5UW+3Kkve2FWH9gee+4KEQcQXvvCFyW5ghqNApotduUy9vEmdLyJbrjhmHJ\/2tKelCy+8sFobeK973SsX6RXWtU8TjOqrf5S8bbLD0ktpfeJMYvWpq5fhW2yxRbUg3OsEDV\/5AcN9X7l7j\/ed73wnCQ8Gg9SnLDq75LNXDJJts802yc3dno\/5hJmeEBH1hnSWYHNVywYNR0Hv5zcp3vve9yYwYSTe3IHOI+yiKw87nzTm8m+FbLoaBWC6hbYf\/\/jHk97MJx6PfOQjq+TlvhC5MnL2p7MH8qFmiXh6PGTTi8Ssps+KOhcwpqBhJoKtWrWq6vWahqR6QrKl4vSEoA7Ipic0FI1hqV7VkBQskSvpGqZFNwAAEABJREFUGDduzjAh4LspCj2T2VjSR6B777132mWXXapdjKWBochyW4jMroA7VIRHPY6St012WHopbZy4UevZJN+FeHoHjbYr8Ur1aiq\/TTZPRyzweggR9YSeB+u9oR4RWZvKREK9oTqBxfTimuT7xs\/ZLCVwi1vcIhnX2zWLQobrzQaD1dc6Gm9fVwuR2dUGvXKbTFP6KHnbZIell9LGiWuqzzjxOfFsfFofaurx6sSrTVRUxZfqVSUU\/rTJDkvP05AQwdiMiHlvGJMzQUKyuSneBdfj8vS+4UXPbMa8P\/3pT6vfaTPzaLvnHMbyS7UQuW+lZvkm7wETYitr7\/GQMUoK4hmi6fH0EF6iR\/pyOSKRzgIBAQl1KiZoHJ17npuGvYvIZoUIx1k14iG6jn322aeyg6HGvZNYiFwpnOCffKgxqtpR8rbJDksvpY0TN2o9R5XPbUMyxPN8B3oPcaFT+\/FMp9fbbrvtqk+xuhIvLyf05cdh6aW0UeIQUW+oQ8nLnFR4EdmsGDnhhBMa92\/w0aPC9XqjLkSWb4a1ywNINox4apsTT6\/XlXjyrk1YRLa1oXL52H3U+oySt012WHopbZy4Ues5qnzJtrqOEvFWrFixmpheL3q8IJ64EGorZ1h6KW2cuLBpUsdOZDNhYorUs1sUHO\/gvBvx+uC62n487LlujrNSmzwQxDMJEZMrTcRDOtDjTWsI12TnUsa3ku20005Lnsv+97\/\/rWZX31X\/qymZ0klpnN61qFHytskOSy+ljRPXtX595Uq2ddVl+3dDTcSLZzzPRnl+vZsez\/vcIJ7hZy4jPMyOUto4ccqbJIaSzbIaX197JZAXOs6q\/1zPLLzueSB6PCtBgngmWHJPBPHMaA4jXp5nTQgPJZu7jGVaJkLyyoyz6j\/XM61waZzetaxR8rbJDksvpY0T17V+feVKtnXV1ZQ3iKfX82mQnq9pvWYQz3rNUo\/HllI548TROUk0ku3Xv\/519dGoF4D1BcdWmIyz6p8DpolRFufW7Rglb5vssPRSWp+4Up56nSZxPk45XfNaUOFLfgulPeeViBfLxmKhNPJF\/Url9ImbJMFyXUWymfzYeeed0+abb17tQ59nELaUxUvOOgmldYFx9DRhLWdf\/aPkbZMdll5K6xPXdyHyqP4p2dZVxyh5Q9ZzXn2htJ4w2peJFMSDr371q9W7vP\/+97+LFkqHvtzWtrgoY9LHItlshWCo+PrXvz7pwSZd6EzfzAOjeADJDDU944FnPHG5DkNLEyxeosdzXp6+HMKLVv3rki08ftOb3pTufOc7F21c7qv+1aFoeIfIUfK2yQ5LL6WNE9ehamOJlGzrqnCUvG2yPmBeuXJltb1fEK\/plQLiGW56pZDbWiqjFJfnmUR40ap\/PZqCbe7j\/Rn4AsCX2pZu+Qxnua\/6n4RjZjqWvwf0bohnYgXxPOPViWe4We\/xxF0XtVu06t94Nj65iaOfdtXLfe9736t+vshCzuW86t\/4vK8zR8nbJjssvZQ2Tlzf+nbNV7JtpLwdhdvKaUpHPM94QTxDzTrx4pWCWfYYaopjWpNeaZPCome2G97whik+uYmj92x+mcaEiPBs1f+k3D\/TMw0PIF7e4yFe00t0pANDTc9907AndC4iWyS0HZfzqn\/D4Db7m9JHydsmOyy9lDZOXFN9JhVfsq2r7lHytskOSy+lxTOel+hd3+UhZvR4XevYRa4T2WxZZ9yrpwul46z69xw4TVin2Vf\/KHnbZIell9L6xJXy9K37sHzjlDNK3jbZYemltHqcPVDf9ra3VT9OYn8Vrw6iTTsimWc\/R+eTRCeyTbLA2c\/P9v\/52ZnvJus7P3Hm+U2PZ4LFcFNbNwytP++JHxdLTrZxDV6u+Wd2rdkeQDDPeUG8adRmRrZpeHWmc+aBggdmZCs4ZRY188A0PDAj2zS8OtM580DBAzOyFZwyi5p5YBoe6Eq2aZQ90znzwDrlgYWFyFtuuWWa9JqxU089tfrxBu9vDj\/88LEca\/8TU7N0lVC33xpOcvX4sYxYosy77rprYnuOpapHlM3XfN61yuFvNpfy0iVeujK66g25Sbal0BlH7Z5\/2Wb\/SO\/ZhJvQty0v9GzWQT7jGc9InBJGrEnHNd1+vo6Lbt955znU74EPfGDqe6FzXZMOazPHH3\/8glqL1k855ZSF8+UesPLk9NNPT36D8J73vGeruRbm97lhzHlRakMfJXCSl3zC1xXsWRm\/OdBkg6942R145zvfWYkuB\/srQ3r+QSiQ3TWJ+tll2GdN4j\/84Q93viEi77vf\/e602WabpYc85CEJiX0YTE8T7Auq3PqKoSZ58doM3wsH\/FRThJf70ZYLl112WfJDG\/lvDubXgE\/cQB784AdX1fnUpz5V+dXOc20+rTLM\/6l6Nj8kEBdzPm61\/xhc705Ld1cXNrpi8oYMF1100Wq64iQfEpDVbdvy3O90+bTniiuuCNFORz8zFPafffbZi\/LYIUwZygJ2sjcXzIdBZKBUz5I\/xOW6hOWlI8AfegBpTYgG6lMmCDlfLfvNMeeWEeV+pZPuKMcx6nfwwQfLko4++uhkoe2hhx6afLFBBtgd9Y484qTRSXelYMgffjzqqKMqifvd734JnOjpGvJLriBdOcpTbtjiHPiwEiz8aZOttzH6oo65OjawVfvRjvK0etjuBJdeemkV7Qt5v3ij7meddVY14lBGQL3oJuzovCLbMccckzBbwoYbbuhQQYXcDauT7I9uVFpEUWYIGndl8e505EKvT9bdBVwMi0IjnqywTyOst3zuc58ramJgEycrI5SKY5uGAtLdnSM9jmQ0gjhX55I\/xEnL5eSNc0f+eNKTnpQ0AufDYEhjaBMyPnFyE3F3tZYP+aSddNJJafPNN090Ow+on4ajEfltPX7dd99903nnnRci1ZHdpXpXiR3\/sJO9xLUBEGaTnkC4C0q28GHu\/9Ajvm63uCCnY72Nycsv7NNe+dQ1YyNbtcsNNtgg+VENv9BE3kZX1lb6VRz7o8invUhbf\/3109zcXPVDNLYQUb74AJ31611NkKgoIcMxi46FGRwVEu9CgyGcdGlkhOWnXNhiZTIqZgwsDvRcvppFOufWnvkdLF04eXG+l8vJLq4Lmm4WkbdkP\/sgbywlOXcuDkaSww47rFKZDy\/Cdg3bxeATviEY+lxQPRXCf+xjH2uciPLTR\/KR01jiLrnffvuJXg1s0lj+9a9\/JZ89IdNLXvKSdIMb3KD6hVg67Ixm+Mh2X99TYHt59riW7BM3DsL30TMguTCd0VMLt0Eew2V2uS7RdvzYBb\/m+aWRISuPvNJNbvBLlFuS005dR7284Z888vrBRD2XtCDbYDCofh3Wr6F6lnMNjSoQjd\/dyG5605smC5rp4E828S9Z18D1vslNbpIMy+cIBaJiDA4jZHrZy14WIolClRChUhxhiOIcnDsi3fvf\/37BCgpzIRjJOQj6sIc9LB1wwAGJfnEcce6551byw\/5ozNEQHekiz65o\/M6hHhcNWhroJfQWnOR5RVwOO4nVh7XuYkhFzh1SXvVTZz2Q+LxcF9HFFK8XQHDhOtzo+Lceb+gYdY5yNYrQ4\/nhEY94ROJv18uX9HQoy++Offazn02XX355cnMzVGePdH4nL9wH2ombkbzRM+gdhMXFDUi4DXoB14IcP3p+EtYmNF7hgDQyzvPynEP9mogD5HJ83\/veV5GI\/XzkWpl8sh8qEhoJkGuCNuFamV8444wzKjE6ou3xb+l6L5og0XA1Lgpp8YW2zMKApZgt7M5J1t3U+dOf\/vSU90z5s6AfVvTRKTmM96umLpTNhRgpThqdjqNCo\/GMEhch8rOVzXGe2xRxnG4oibgBjTvSHeWzWFUYEC5k9TDiIMimkbApZMhLV093R+ESEB55DzrooLTRRhstEqEH4VxoiX6q1q+vRHjTTTddmEBRFvLGTyDxf8iSd11dX+E+QHaNVV7DWfpAWBwf1IkivoT6TTBvR3X5PE15pTrwUfje0UjhqquuqlQZStKhY+AjW+hrN7YE8StOQbYYwfgkRzvyc8d8SIkfXrRNnvKdq2vb9a56NkIgk7uRAoW7gMHummT93ravuYXr8KW3hlGP73OOnBpkDkOtqPgoOtnP+YYlelfDEno10lyPi4HM4ac8DTE9ANOVUp7SP2xCx\/BEY1UmhDYjiiDQYDCI6Op4n\/vcJ7kBVifzfwwbNbb54MT\/xxCSYjdp5YCwOGCr41LCc56bkjKjrbi+eh9x2qkbp5uea26+QLwORjsdDBb71MSInlFPShZJEVpv6LwLKrLVBY1DMVm8nsadXxhcSEYJu6N49mK4c43UMaAyjHJuCEOvsEqrvDuHykYDl984mMxSAVHcldhkaBZDmeihcjuQGanZCS5kpNOBGHGOHM7J1WEIEnJx5OPoXR2dR5qjKWk2CkOMJlwPfhYXiN9l8ACPEBqFNDaaQRMGZbi+wqPCjcVjR1s+N2+ybXJ1f8e59mFU0ZY\/0v2YZ9jlhmlIKY2PDMeFAXnIGvIGgeyHahTEb2TqsK1jfs333HPPhR3oulzvimwaPlCuS0WgGHOKjx9AlO6u5aIJ6\/rd8eVxrpsNpnOwd0LiA+Q4T36NPOIPPPDAasWEuznHhoMifSmOLoQLoiwTCognHNBg3bVBWDw73SjUyTnwiSO\/gTAYasrb1AMispsXWflAOGAmkd+cGwLd7W53qyZDnPMzfwuTiwd8z8Qmpvbee+8UNp588skLEzSua70cOrrAjSTs0ajrN5RolGTItuk0LON3co7Ohd3ItQnhURGElc9EXNz4nUc7NeTle3FtcGOzAzM5\/jQpZW9V5\/wIwlC63tVsZHS5FESX6u4bDkMwDQWCJNLIUMzZcde1bTkZTOdo6WBiRGOK90V0KpcD7GhLJn9Idr4UMGmj3uwwrldHR+d5+R6go47sJgchq77AJ3wjby7HJ+Lyh3vnOXI\/Rl7luojxbKQMExuG64blRgv8LJ4922+\/fRoMBskd+j3veU\/SkPTWfKssuugk6xqI64MYHvIdH9Z1xE1HvMk2DVW4CfzNl+xydE7WpI8bunAX8IubOlk+pw+ExYEeXc+mp8ptVxfPdab5ycU1cGSPa8d\/0viTX0e53lXPJrOLZchBgXNw59YAhHOIkxZxnPGNb3wjxdBTPH2GXCrg3HhX72ECoN4bSDekzHWKWwqor1cSeVlsAXEasguljk3PbMilrhq2POoR+Z0DP6i3+jsvQRlmNfm3lE5nlGPG0XCR7XETiDwaEZvVLeLYVNfrnO0h0\/WoFzU8JJ8Pw5wHEDrscqPQ40Za6Vi3JfylMZfkh8V96EMfStpfyAgjiaM4P3cWN\/\/cR3yqnQbZyJbgOvBnpAmLi3PHsD+\/3tVspCFAXESCOQhLX8A55yRxuYywhmLaVIVcQPqETasfd9xxybDH7KOKqCBiS\/cjCmaB2pxKv4bIDpVT5jCQIb1O49MAABAASURBVMuOIAF5ZbNJWpTp6DzgHOI86ksPfREfR2XRnSPPT06Zys5lmsLKkwcMwfiJT+mMPFaC2LvTkMurl1zWM19pJjPXS14PWX9mCxm+5vMoLz+Kl04Hf\/BLni6cy6i74XPkUQaZOviRTpAn95ewOGm5H+iIfGELe4TJgjB7HJ1bUcOnZm7lD\/CplSGvfOUrkzZJNvwf7VRcvXz5xUkLsJXN0gILPVtETONouvT5z39+2mOPPdLuu+9eraIwxDGmfsELXlANe6ZR7tqscxSfesY0lALPEuEXd3twPsqzC\/m1EaP4tE\/9l4RsDFvO+0yyb01EV5+6i4M6Gl4iHXgWEWe4p+cUXtfR1ad9\/DRRskX3rVuvG+NFobVmxvq66EMOOWS1cXVdfnZ+jQcm4dPQEUOc+tHwzjDrmhKX7q8ylc2epqHlNKwJfyx1O50o2bo4xtRzwHsN6JJvJrMWemAdq9KSk+2tb31rCphKhcc97nEp4BxC5hOf+ETyIjZgyjnIuo5dq1l113APLDnZ2vylp4MgFHJ5IRlAviBiENQv74DFuN6xbLLJJsnUs6VKluZ4PpnhHtXCgZkfuvmhrZ32SV9yslmQ\/K53vSt5eR7wAhT8oAFYLAtdK+S9ElgSZjx+y1veMt3mNrdJVh549+f9ia+VvSz1FTiYKPCsUMcRRxyR6nFN522yw9JLaX3iSnma7B0nfpxyRsnbJjssvZTWJ65ruxtVbsnJ5u09Yq1atSqtuhbIB5\/73OcSWNsGPtoDYZBGLkjqSFcQdFjlo7eMHlLvqGeMIWsMU2Od3DBdkdYmOyy9lDZOXNg0rWPJtq5ljZK3TXZYeiltnLiu9esqt+Rk62pYLqeXA6RCriCpI\/IhITLWiYmMII+8uc4I10lYJ6D0kB31OJOfeSD3wBpBttzgtjBSXnnllQm5kBGCkEFG5ERCIFfXiWCeGfWCej6Ini+X9T1Xfl4PD0svpY0TVy970ucl27qWMUreNtlh6aW0ceK61q+rXG+yWUNmTxETEh66870auhY+LbnS0CHKQkYEQ0JAvCAh8oH0kHdEPsQDPZ8hKPINK0e+YemltHHilDdNlGzrWt4oedtkh6WX0saJ61q\/rnK9yWYxp4a61VZbJWsiTUDYGcu6s7QG\/kNC5AP1Mix1RD5peZWi10M8pDNjmqfPwjMPlDzQi2y+BfItj60N\/IicBZw+hfD9lD1F7FBUKmxNikMwPRzyIR4gnri8Hno7ryNiqJmnzcIzD+Qe6EU2y60uuOCCZKXzYHDNJ+S+rbKK+sQTT1zYAyMvaCnDpXF61\/Kb8iIf4untgni+nQq9MdTU2yGe3q5JlzyltHHi6JwmSrZ1LW+UvG2yw9JLaePEda1fV7leZLMtgvdaPtXPC7L\/hY\/yvEvL49e2cBDPx5dBvLyOiKe3842WYWaeNguvux7oRTaNycvj\/M7e1YU+6fdZR8D3WO4+kwRb+uobJS9ZM5++1fI1spfmuU\/scWGYqbfbYYcdkkXYYZe8EY5jnzg7PEX+aR5LtnUtb5S8bbLD0ktpfeLkmQZ6kW0cQ3yGbteogJUctkUI2McEfGgKPt4DX\/qCXaUGg0H1DdytbnWrZLZpuWCbbbZJvlgv9Xa+0tbTvfe976027JyUzT52nJSumZ4NqvY0TvselrcX2Xzy7Y6u4deVG17aDqwe3\/Xc1glw7rnnpnPnYeYPfIoBGq3eAnbccce03XbbJUcQJ92mLL5elh+6lk3O3dqxC5pk82GmSRXnoc+ogJ16O68QPNtFWklf17jQMa1jyY6uZXXIu6CqTXZYeiltnLgFoyYU6EU2aw3t4IQUuR1237UeMW9cebqw7RAMtwy9Amb4YMWKFWnFtVhvvfWIdwI7ALEQ0yfpiKlRQxBSGBmDiCXl7u6l+FJcm6x3kCZV9HSAeLketnq2M6Hi2e7MM8\/Mk6twqYxSXCU8xT\/jlDlK3jbZYemltHHiJu3OXmQz1W\/\/C8M8e4owyprHY489Npn+N7wTV4KtEBBr5cqVaeW1QD7QGANeKXzwgx9MIAyRRjYnKoK2kTMnYxAxSKhXREBA2JLd48a5AQXx1MN56IzeDvH0eEE+ZAyZ2XHN90Avsk17r4a6WxEJkApysiKexouMdWJKQ0ry8tX1OkdCQEDQ+yGf4zTIh2RBuujtxLElEOQzzEQ8Rz3fjHzhoTXz2ItsqqrBaAS28Npiiy2q3Yh22mmndP\/731\/ydQakvOSSSxJyIZneE+kQMsgojIRArm4s8unhcvIhHuSypeeBrunyItmqVasS0gE72ZzrQDwkQ37+1vO98IUvTHpBBCwNPfP8kwqzt6+uUfK2yQ5LL6WNE9e3vk35epNtTd1TBBkRbOXKldUwFvGChBq7XZml5w5DPsSDGHrWiZfLLw63xyCe8uOluSPbvLus5\/ZKwcQKAiIdAuoBwTkSSkfSet7Z+XXngd5ku+5Mbi+59FDclgsJ9Sp6aY3csNRR7yctzx\/k87pCg29q1MPsKKVFHOKxRa9nG269XpBPfG5LhPWAgGRsQrroCYOMzsUjI7Ab5As9TcewrSl9WPwoedtkh6WX0saJG1anPmlzfsjCDrf1zF46+wkoPwJniwEvZU33h9xyXvVfGjqE3W3HyItgeriV8z0g4gHiict1GG6a4fScp7dDxEgPXXGeH0tpTXE5+ZDOC\/QgoN7QTaGJhFEmQiFWkBEhkQ\/0iEFIYXFICUgJfhdAfqAr9HY5lurVlK9Ndlh6KW2cuCYb+8bPlfZgt1+\/xmVvfu+svIj1ix1+dC9mH9e2Vf9tDkQ+xNOw+QbxrKKJfEhmmIl0oFFG2jSOQUBk0wMiYXwqhIi+4WMrICL5NjuQCNiOlICUYINdJASERE4QDkhDUEDQgNc9dAaU0WbL2pheHEa6WKbv\/USpLZT9fOy2226b\/LKL5UHLfdV\/aejQ9eJ1yRvE45MgXq4f8fR2VsLo7fK0CJfKGScu9CIV+DodCQERXdOcjOJyQpLvSsooyxFxAsiEoICggZyoCImciBpwDtJMsiErBFkd6aTfQoooT\/k5xvFfKW+uexLhRWRTGZWyU5Up\/ijExIG00047rZp5tBh5YdX\/vNByWvU\/b86S\/Q\/imWTRy+TDTKTT25lUQTpDziUzrKEgRATEQjBkBMRDwCBlnZjSyOgpQV6gB+hsKLI1Osij3QFiQZDVEfmQERATSmSVThZBA3TRC8pqNWhKAkWyeYaz5i4v00tsvd15552XEM2yrL6r\/o2jpwm\/SdZX\/yh567J29bJnvq283Zxy\/yGdRuNXZwzN63nZ2yfOSEPeacAzOqiXn0yymMHwGXzLCG9+85sTfPSjH02eJcHXEAEE9UMVr371q5ObUcAaWTDbChZwQ+6zrmEEAmQC5OLrAPIhIeQk9QoF\/IzZ2972tmReAvLHg642dJErks1qEEuymhSoGIP6OkeXPU24UfTVP0reJllDb7OaSKdh6v3Cl3xrOdmPfvSjZJjp5hW2lvS1xZXSQ98kj6OUY5laQO9nT0+\/mLPNNtukba7FRz7ykQRmW8ECbkDWvFfV00avirjIqmcFuvWqoGcdtT16hQKWGZ500knJ0B9sgxjXa5LHRWSbpPLrSpdG1rfsUfK2ySKdSRXPdYB4uV0xzDShYpiJeHm6cKmMUhzZaWKcMkfJG7LIE0AmxAJkNewF5ENEQEpkRVRwDtKALIICPXSCMup+m9ZOA4vIZqPTG97whil+8rZuiJfZ01z1Xy9vbTnXuwXxkK70bGeo6fkuyLccnvGW3P8TKhCJAKEAwRAUEA8BASGRE4RhWh8\/LyKbLlR3\/Ic\/\/GG1ajv3msC43RCz76r\/1ZRO6cTzS1\/Vo+Rtky2lB+k820VvJy63N3o8zxye8Rz1fEG+kt48\/zTC45Q5St422WHppbRR4pATrrrqqmm4MC0im0kQK\/e9G7HGMEr92c9+Vm3pjWxgwqTPqv\/QNzumhGTR2yGeSZW8x+Mjz3hIptdDOj2fD249WyBg0whE3hmWlwcWkW0wGCS\/BmqWy45Zti2wP6SHWL8e6nUACHt\/shx\/STTG\/X1cPUreNtlh6fU0xDOp4pkC8Rzrw82oDwKadUNAX7wjoKEnBAmlI2nkGfdYt3cUfaPkbZMdll5KGydulDp2kV1ENpms3N95552rH5jQAKzsf+Mb35iMd6WDsKlUaWTsuOWFpLzSZ+jvAcRbsWJF0ushXRv50vw\/Q09AMiREuugJg4zOxesRgSxCyjevYvZ\/yh6Yc2H8AmS9HD8L69dBkchGNX411ORIyAmLk0aGrDyRfl0eS+P0rvaMkrdNtpQedpTSmuLq5Hv5y1+egoCGnk09YJTliFCIhWCuOSAeAuoRERKEQbx0pITDDz88yUsH0EdvF5Tq1ZSvTXZYeiltnLgmG\/vGz2200UbJcqy6UWvyQuS+zliT8gUBkS16QKtYkBC8j0JC0EtCl\/ohESAUciEleIZHPiQEhEROEAbxQA5BA27GdNEJ9EMXe9Ymmbnddtstqbitwy+++OKqbmv6QuTSOL2qWIc\/o+Rtkx2WXkobJy6qhoRguhsJwVAUkBGQ0XlOSPIICfKHvi5H7QcQCRALQQMloiInooJwwKRbibB0wmAwSMpQXt22cfxXylvXP+753OMf\/\/i0\/fbbJxMinJLm\/3nXYFZyTV2IPF+F2f8hHkAmpEIwZATEQ0BAxiClMIgno6cMyE8P0AlDim1MQpwAIgFiBVkd9ZaAiICciArCAWnkIHpWuoBeUFajMVNMqCZI7nrXuybksgzLYmOGrckLketD4lH8N0reNtlh6aW0ceJGqWNXWeQJ+LQKuRAzgHxICAgJSArCIO0pT3nKwrrIIKojfYgaUFZX23I55Akgk\/YLSAqIB4gIiJmTVNyuu+5aLddCUHbl+icVrsh2\/vnnJ+\/UTHog2xgLkVOX7cf9xNQ04eer+uofJW+b7LD0UlqfuFKevnUflm\/UcjbZZJMERk5bbrllsk4UXvWqV6WA2WyNPoAEGn7gwAMPTOD1Bhh5gbWlYKc20EmAVyKjECMnqPeVQVDPwaPo6So7Z\/hoQahPZCwYRTZGWyXSpETF+i5EPuecc6pXCrPjzA9tbcBO2GCLRPjhD3+YwEt92HvvvRN4Bww+btargl41oHfVC+tNA3qvph61bw\/bxJeIr7ZFuPDCC5N1Yve6170ifnaceWCN9gDCBJAKuVauXFm9u1w5f0Q+JASkzAkqbRqVn9ObuUs88pGPrPTPFiJXbpj9WQc9kJNzGtWf22WXXdL666+\/oHttWIi8UJlZYE32wFpn+9xgMFitUmYlZwuRV3PJ7GTmgYl4YM7MY8DEyGAwWOMXIk\/EMzMlMw9M2ANz1jMG7A1Iv8XEs4XIPDHDzAOT88BcPv269dZbL2hGQIuLrWuz2NiiY+\/hQkBYnDQyZOWJ9Nlx5oGZB1b3QPVSe\/Wo2dnSeGBWyrrmgWU0PnRPAAAClklEQVRHthNOOCF94AMfWNeuw1Tq6xn83e9+d9pss83SQx7ykGR7OdvGT6WwdUSpd9K+kuFTr8t8WN3Vp8uGbLYUs\/egrw\/sdbKOXLupVvPggw+u9B999NHJmj8b7FqJUUXO\/vTyAHJZ6+kTNOstfaVw1llnddI1cbJ9+tOfTm9\/+9sXFY79DLWfoDV41trZ0cgCV8LXu9710qabbpqe+9znOp0h80Bfn1qU64dTPF\/bEc1mqGeccUamed0N9vWp9Z1WmAwGg2TTK+2Xb7t4cqJks2r90EMPLZbb9kMc3u89+clProY7RQXraOQ4PrVlm7WAXOcbRTc3y5acr8sYx6f2Ar3BDW6Qttxyy2SHtPve977Jjlxd\/DkRsh100EHJTKS9\/1WkKjj7s9x\/iCMzddkEJ+VT24fbJ8aeMbYJ96yxbCq5xIZMyqcW4e+3337J\/IKF0j7j6VKViZDNezk7cVl1veGGGy4q16sBvw+AjIPBNStWfGVg2HjiiScmL9UXZVrHIybhUw\/z7sC+7PD89qxnPSsNBtf4f11077g+\/d3vfpf8fFoMxY3GdDI+8+niz4mQzT4mz3jGM5JhoA1e6wUj2jg\/xFHXty6cT8Knhu6PfvSjk+\/FNIx1wW\/D6jiuT31+ZqRw+umnJ89qvgH1DZx5iGHlRtpEyBbKmo7jfP\/WpHNdj2\/zqV19jRr233\/\/5LnN8wXsu+++67rrGuvf5lMdxjve8Y7kKxm\/wPPMZz4zGa35SLZRaZawJGTLymsNMt5MUavgTGCoB250oxslv\/n9i1\/8IpmeDmy11VZD8y3LxGVklF\/j9bzmFYqvxv06z2DQbWi+JGQzNar71Q3X\/eZuYdq\/Hj87H+6BmU+H+6dP6rR9+v8AAAD\/\/7yAVnUAAAAGSURBVAMA0vv\/sNvryLMAAAAASUVORK5CYII=","height":132,"width":219}}
%---
%[output:6c0eaee5]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANsAAACECAYAAAAZW15iAAAQAElEQVR4AezdB5hdRRUH8PNii4I0UUAFE8BGDU1DU0AUNIgogiBSxRADIj0qUiKg9CJVERQpUXoREMQSBZEmIKA0qdKkI0hoRn7jN\/Gy2bcl+\/aV3cmXs\/fembl3Zs69\/3fOnDlzZsT08q9woHCgKRwYEeVf4UDhQFM4UMDWFDaXSgoHIgrYyldQONAkDhSwNYnRw6Ca0sVeOFDA1guDSnbhQKM4UMDWKE6W5xQO9MKBArZeGFSyCwcaxYECtkZxsjyncKAXDhSw9cKg\/2eXs8KBgXGggG1g\/Ct3Fw70mQMFbH1mVSlYODAwDhSwDYx\/5e7CgT5zoICtz6wavII33XRTfPjDH45VV101PvKRj8Ryyy0XG220UTz44IN9rvS4446Lyy67rE\/llfvABz7wmvq+853vxLPPPhvPPfdcfOtb34q\/\/e1vfXpWKdR3DvwfbH2\/p5QcBA6MGTMmLr744vj9738fV199dSyyyCLpehCqSo9cdtll0\/PVd\/nll6e0\/fffP970pjfFd7\/73fjgBz+Y0mblz\/Tp0+OVV16ZlVuH9D0FbG34el988cV45pln4h3veEe89NJLceCBBybJR\/o5l4YOOOCAWGGFFeLjH\/94\/PnPf0498aGfcsopSUKussoqkSVWyqzz581vfnNsscUWccstt8Qdd9wR22yzTZC2\/\/rXv2KnnXaKsWPHxoc+9KE4+OCDU3ukb7\/99intS1\/6UnzlK19J5UnXiRMnxqc\/\/em48sor47e\/\/W1qx0orrRTrrLNO3HzzzfHPf\/4z1l9\/\/dhrr71C+0jwn\/70p7HGGmuk5\/3mN7+p08rOTy5ga5N3+Kc\/\/SmWWmqpWHjhhWPxxRePRx99NH2ov\/zlL+P+++8P0gc98cQTcfbZZ4f0f\/zjH0kSugZOXVGGCvjrX\/86\/vCHP8Rss80W5513nqweaZ555ol55503ACkXvOqqq2LRRRdNwFHfbbfdFg899FCccMIJSfLJJw0fe+yxfEto35QpU1JfLr300gD8P\/7xjwnMnqHg008\/Hcsvv3xq3zvf+c7w3F\/96ldx0EEHpba+\/PLLig05KmBrk1dKevzlL3+Ju+66K0kXv\/p77rlnXHHFFfGpT30q3vCGNyRac80145prrompU6emdFLprW99a5ISugIAPvb3v\/\/9SRU9+uij4\/rrr5fVb1p99dXTuO70009PqiUQP\/LII3H77bcHaVWr1eJd73pXLLnkkjOevdpqq4X2zDHHHLHbbrulsocddliceuqp8fDDD6dyJDYpXavVYsSIEantr3vd6wLgp02bFiiG4L8CtjZ8qT68j370o+mje\/755xPIcjOpj4CXr7seGTh85ECLAGTvvffuWmymaxKJVCPdcuaJJ54YP\/jBDwJwd9xxxyRxqanGY0CinHNqr\/MZ9OoJ485Xv\/rVIPU23HDDGD9+\/Kup3f\/vqT\/d39GZqQVsbfjefNDUwZEjR6axGushkKGLLrooSRVjNefSnnzyyaA26oqxFbUNcADV2Ig6Ka8eKXfsscfGYostlsaJuRz1bty4ccF4AzyAS5JS\/bRPO++9994gkfM9+fj4448nCWesRmLl9uX84XhsO7AxPxuDGEQzgyPn0uQN1Zd0ww03xCc\/+ck0TltxxRWT+jhp0qTYYIMNknpFrUQLLrhgrL322rHuuuumdOrcVlttFQsssEBiDTUTEB2pdD505ymz8odBJdfnuUDEGFIpkured999k5pnTGVM+bOf\/SwZRAB47KuGE9ME8803X\/W2dG6sN\/fccwcJ\/cUvfjEB+cYbb0xqZSowDP+0Ddj8SnqBm222WdLtmZ\/PPPPMQKxggMbydf755w+512TMY6yl\/0zxjCUMCyQIFYtaKB85lwYckydPTuO3c845J4455pgAKnlf+9rXQlmUy1eZptytt96aDBTqu+6668L4cPbZZw9EddQm4ypjRlIMqI466qjYb7\/9guT0Q+D5RxxxRBp3UT8nTJgQSF3ax3Lq2drnBwFgAfu0006bIUG9W+1xjzrVrQ2uhxq1DdioPcYqZ5xxRjAfGycYSKPRo0cHEJ511lkzfsGH2ovopP6YEAe+lVdeOYzH\/AhmydpJ\/Wh2W9sGbKxXfklZupi9jSP8Anqpjq6BkYrUbCZ1rW+4Xy+xxBJx4YUXJlWXZGS1HO486Uv\/2wZs5lYM5jfffPM0AUrtMMFpwE9dOfTQQ4Oq2ZdOlTKFA+3IgbYBG4sa7wK\/mFRJ5mTeD+utt17ygjBHw4zcjkwsbSoc6AsH2gZspBZV0sCayZuP3utf\/\/rUB+qja2VSQvlTONCBHGgbsHUg70qTCwf6xYHuwNavBzSyMI8H5mz+dn\/\/+9\/DkYFEmrxG1lWeVTjQbA60Ddj405kfMhH7sY99LM37ODIvS5OnTLMZVOorHGgUB9oGbG984xuDSZlpvzuSp0yjOl6eUzjQbA60Ddg4wpJen\/\/855PpP7tpcdWyXuvwww9Pq4ibzaBSX+FAozjQNmB7+9vfHvzuLrnkkrQY0qpl7kuIX6QOc0VyLNQxHCgNrXCgbcCW28Rti7cI8OU0jq7S5OW0ciwc6DQOtB3Y5pxzznjhhRfSOirLOtDJJ58cJrmLgaTTPq\/S3ioH2g5sJq+\/973vBfetjTbaKC3zsPx\/n332CRPe1caX88KBTuJA24CNmsglC\/N4kljCwcnVEg\/LOqyNkpfLOC9UONBJHGgbsHHNuuCCC9J6KI7H1q9lRjoXc8NaKWVyeuuPpQWFA33nQNuAjaVRSLQ99tgjLPe3bEOkKfSJT3wiLefgmKxM37tXShYOtA8H2gZsmSWiNXHPEkGKixayMpjbloWkuVw5Fg50GgfaDmydxsDS3sKBvnKggK2vnCrlCgcGyIGewTbAh5fbCwcKB\/7PgbYEm5jwwrUJgyY0ggCfJrf\/3+xyVjjQeRxoO7CZbxPObIcddkihre2mYqMGoe3kdR6LS4sLB\/7HgbYDG\/9HXiRC2WmikAimAYRKkCetPySUgl1SBA4Ss\/Ckk04q2xn1h4GlbMM40HZg4\/9ogttuKbmXwl4L9jMr7loi\/woOajWBlQTUUtMK+dnl2CQOlGqi7cAGUIJ+2gxCNF0hsrfbbruwsQMg9vedAZZ4lNy93G9h6rXXXtvfx5TyhQMD5kDbgU2PrMomiWyoZ08vk9pLL720rH7TnXfeGe9+97tn3GdHz3vuuWfGdTkpHGgWB9oGbKIg8\/K3QhuxRIoZaaW2MZs8ZWaFMcaA9e778pe\/HL0RlzFULecaVdPqnSuHqvmuUTWt3rlyqJrvGlXT6p0rh6r5rlE1rbtzZVA1z3Wmanp3592Vy2mO3d1TTVMGdZdmh9Rddtllxvtzbm8DLn25vHtRvnZ0ncl1V5JX73sZSHrbgM1uK0ceeWTYeM8OLRhg5TaiVtoQ0OYN\/e3sqFGjQqSufJ9zFs58bStaO2n2RFzGULWMLZC6plXzq+fKod7SqvnVc\/ei3tLkf\/vb3047gzrP5F6Urx1dI+c9kTKoWsZ1pmp6d+fK4VU1T1qmavqaE\/aL60ZvHefM+aVEjy67XdocUtlqOdfI8EBAqJzHAEYDEkIjpymH8rWja21ydN2V8rfR6GPbgI3V0epsR8toSDS7uCBbDtkDjJGkvwyw\/ZKdN1kykXP7jfX3Ob2VL\/mzzoEp1zwc6x59fWw75W+x8iJzxROHrp7IE8fse6XDkKCWg+2ZZ54JYceRcxG0gMpYK3P4kUceSTtYAmJO6+tx2WWXTXuEMbSstdZaQSWV1tf765Wzs069PB+PDwf5iJpJX7\/oqfThNrPO3uoaf\/b\/wFQtB0Tz7PTbQAdccncsOM\/IBLCjN\/7gDLaev+0ysfEKC6QyMxIbdNLT+2tQFTM9piVge+WVV9IuKOJC2ptLRC1kKY2jHTBtEbvqqqsG2njjjdOWUVTNmXrQS0KtVkv32mOMoYVKWqvVermr\/9mA5cNBPp77n5gWyEc03Omdc7w+ganKByDKEuyGb68YVZBVuT9prVFx\/sRlEuAA9L5X+XrAJfekH5Rdbh4dk2+ZP1xX72nX86aDzUJQ69JIr3PPPTeuvvrqsCIb2QTQxnnUyWWWWSZsFSX+iAhba6yxRp95SFXkdUJaDtak9t13353ak0F2xd+fSr\/MPiAfj19l5CNqJu0ydmT6cJtZZ2917b3mvDO1CYgSA\/vwZ5VF5wo8XWjukQFwfswuf5Xfbv0f+O5O4HPdV8rvr6\/lG1Gu6WAzj2YQb2so815dO8Eli9n\/pptuik022SRJJXEj+2qNZLE84IADYtq0aenRgzGp7QWffnst\/dpmkPkYUoUt\/jN69OgWt2Bwql\/oVTVz4w\/9byvj7moAvt\/cN727rG7TWsGnvoKt2wbPSqJxl1XZQAFAzPxVArDFF188WdTOPPPMOP7441PQVpYnE9M91fnSSy\/FUUcdFSyXpKOyjZ7UNh5b95jrY8o1DyVJ1i4g09ehTlOufqjHLl704Ft7zG91ZtPBljsMOABkPMa8P2XKlODpL66\/eQ6gsTKbj+ROO+0U\/\/nPf4IBJd\/f3fGyyy6Lt7zlLSnIa85naGnUpLYBPhUGwM7ZZP5cRVsdW6EeNYsBpFej6moFn1oGNpsfPvDAA7HpppsG875wCNtvv30IQ77SSisF\/8jM2Pvvvz+UzdeOxmMmvgHTZKZwd7xNxo8fH6SnMpl6mtRWBuN7om\/+7PqkMt716LMBZMqqz7HdqB3b1ag2MbR4X\/Vo+vT\/xL\/\/\/e+49957o7f30lOb6j1\/oOktAxsvfuMqZv3cCaqldWvm1RhKAAlRLZnrScNcltSbOnVqmvQ8+OCD49Zbbw2Rt5ZbbrkYO3ZsMLYY6xkjmsjO9zmvTmpLp793R8Zlyx15T1x857SkMt6896qRy7GS5vN2OrZju2gWjeDR7uPe63XVpSdfekPSbN7znvfMeE\/16u2JTzFI\/1oGNsYRE9ckm44bt33hC19IRhFg+8Y3vpGAZJZf7EjWRapkPT6YQlAWARrA8fZfb7314vrrrw8T2sj5mDFj6j3m1bHYw2lylQm\/jMvqsmkgGbN878YrzB+rLDJXt\/czoJz4kae7zWuXxJaBDQOMz3784x\/HrrvumsKN\/+QnP0m\/SJbUmHOjKiJSi6r4+OOPu61fRCJSN+tNaj+13glJRQQuZEymgmzCd94dUVO6S291Wru2q1F8MZ0yaa3RAVyZXBtH96eOVvCppWD70Y9+FEB03HHHJeOHLaMmT56cVMJjjjkm+Lghc20kV18ntamY5ugca7Vamj6oN6k917lfjio9c+ImcfHu44L62hOZkO8pv1V57diuRrfpB19dI7ynTK7x+4wzzgjrFS2jct0T9dSmGKR\/LQMbQ8iNN94Y4vrPNddcSaJZRU293GuvvVK6UAiZqJhVo0k06B+1s9BdM1T2TubFqaeemqaM\/vrXvw64Pw36vF7zmJaBjcsWIwlTvRZRJHnsMQAAEABJREFUEX\/4wx8mE\/8RRxwRPEaM4zKZk3v00UcVLVQ40JEcaBnYLJcx8Uya8RoBNtbJBRdcMC2z4b5VJXNxyrc\/l0sLCwe650DLwFar1cK8mpXTtofiL2m+jbGEimltm5Djmbhg9Tap3X0XS2rhQHtwoOlgoz5SB1kZmeLXXnvtpGefcsopweQv7+tf\/3qQdhYGZhIdi5tXe7CttKJwoP8caDrYSC1WR0tp1llnnTAmM9+GTEKTaKyIEydOjHHjxs0g69BMUPe\/i+WOwoH24EDTwWbcZfzFsx+4hJezhAadffbZwbVKGXNt7cGi0orCgcZwoP9ga0y9yaODqghY+ZHzzTdfPP3008nVivWR6kjyIddUzFzWOG\/33XcPS3FymngiyvKtpJJy\/Zo+fXqUIK2ZQ+XYSg60DGxzzjlnvPDCC8lzBCiQyesRI0aEyUk+jNWFpaRhBibv\/s022yxIx8w86ilvlJNOOiltnEhqUkkHYz1brrMcCwf6w4ER\/SncyLLURRPaJBSptcEGGwRPbKHIvv\/970e2QuYjzxKqpjVrb3vb28KYj8NptU3A+9RTT6Uk0wgmyBu9ni09vPwpHJgFDjQdbFRH0bO0dY455ohJkyalsAicjffbb78gvYAG8ETGokpaYuM+Xv377rtvCJnAd9JcnecgrlxAy9PEdIL1cdbK9baeTViDvtIBl9wTSH2FCgf6y4Gmg43LFdBMmDAhrrrqqhCTJDfauXh+1EP+bTz9ee3zKHnxxRdDECDqYi5fPXIzMjZjcKGC8rm09IbkJEWrZavnZ1w8NfpKh5z6yzjonGte47jMeRn15IdX8hbu3td04camW4ol3qiAUQPlefUbadR508FmrkzE2j322CMuuuiiFFouM4a0Mk6z3oyUyp00NweIgJPTuh6BjYFloYUWilqtFssvv3zwSpl\/\/vlDeAUSUhBP40Grv\/P9s19+YPzzpxP6TI8dt0Fa23bd10alo9UBG68wfyyx81kD9sfTh4GSH6uBPqPR9zerTYYi3PwMHXrrQ09tyt9Go49NB1vugJXZxmFVxgg1x5OENRHYrGkzZttyyy0TeIzZqIv5GdUjNy\/WyrwY1aYcVNIFFlggbrjhhjjrrLOS4UWwV7Epq\/cO9HyVReeORi7ZH2h7yv3tyYGWga0ndmy11VbJXO9XigfJgQceGBtuuGFQK0Xm6u7e973vfbHFFlskLxRg5ZEidoll8haLGr+tv\/764bwRbl9WAOd2kGzOqZNIuDXjQGnNpmq7ml13f+sTPAmfkPguqL\/PmNXyreBTW4LNhLbdR3\/3u98lM75Y7ICEsdRQx9lnnz1NGyy55JIuk+oIjJyXScif\/\/znadmO8Zv0vJ4N4O699950T\/5jIeFAiVqZ6ZOLjozbH3yq27EdMA43Elqiuz7vd+EdMfW2xxKv5n7jy\/H8tGmJZ0vs\/YeYlfdhqOHH1fudlfvzPfm7aPSx7cCGYRaMsigyhrz3ve+N2WabLYzzqIWZAcZvfZnUVt7WU3nMBsAmuqVn8ivXX3JvvXu+t9Ey8audVwzjuWYTwDe7zt7qq9cmMV0QXp30lRVm8Oyj7583ALQef+ul+wG2ZMuUUL0yOb2n9ydvMKilYDPGMi4zPiONhK+TZn6MBGL6t1cbX0og4biMCf2Z1CYJxSRh4WSpZAEFXs8ZCHlpA7l\/sO5t13b1p785wnKWhtTLRk+5tIJPLQMbyWTZDAnmRYjAZMJaUFaqAEMHh2QqgdB0zz33XFoJoGx\/JrXda7oBude1epwXal8OGAdnibnyqwYosWGAz3i40cBrFhdaBjYqomhZYkFw0bJqm6rI84Or1bHHHhtM9CeeeGKsttpqAXjZEtmfSW3PYxTJAX+ok40AG\/2+WS+pP\/W0a7v604euZSetVZ1mWSCuuPPJNLYDvK5l+3rdCj61DGyWy5AypFhmEBUSCEXDMiFt\/u28885LYDNhDZjWweXy1aN5FWWoiowi1UltwYKygYSjcvU+u9rsueeeUY+8FNQ1n4dLd+ldy7lWDjnP5Brl656OyqFqGdeomuacf6hjlZRDvaVV86vn7kW9pVXzq+fd8crzULVcvXPlkPznrjgxxjxyThgH0o5IO+mIH+xtt90WNCbXmdyL8rVjd22SjnwT1W+kUecDAduA2mBcxrS\/7bbbxu23355Wbds\/7eabbw5eI0CBAMw8GYlkcpqRhFmftJs6dWp885vfTJYrYOPczGgyduzY4IR83333RW+T2p5lbq8e0e1R13yxLrtL71rOtXLIeSbXKF\/3dFQOVcu4RtW0eufKoWq+a1RNq3euHKrmu0bVtHrnhghdy7pG9e6ppiuHuqYxrkxaa3Qc9ezqIU\/YQtoQF0DXmdyL8rVjT+\/PNxGD8K9lYNMXqt0vfvGLOOyww5JjMe8PqiIzP+994MoEPPLuvPPOcA\/TPgm4+uqrh0BBQiqYsAZOBhHzc5ySeZUA62BOautLodZwYNKrKqbxHQl3w8Mvt6YRfay1pWCz\/kzYOtLH2IwaCSxEPAslVRBQSCwSjdrJ+Zg0M4bTRxPUAMXC6AikgCacGetmHrM1elKbWqL+dqN2bddg8on1UqTk79\/+tj5X0wo+tQxsFomSTtRIFkcgY9Ln4b\/ddtulBaSTJ09OR+om8z9gife\/9NJLJydm4cXdw6cy55kqMFfH6GJVgfFbb5Paxob9Je3t7z3NKN+O7WpGm24+ZP0Y8e\/H4qArnw9O7L3xuqc29Rmx\/SzYMrBZRmMSkjTifGz7KEYTfosGviyVjvpDHbzwwgtTPJJsIGH0ACT6uQExaWhwzOubOskDnARkeezJ699Yr1DLg7Q2xImbk\/gNR20dtJqBvlPfXaOpZWCjGlo2I5IWD49Ro0Yl73zWyCOPPDL4N+6zzz5pRxpWJszjiMyYYt0bRtRqtbBrDeDyHODpb8WAPOAl3TgiA6U05DyXcV2ocKBZHGgZ2EgbC0GNpTgaA56Y\/6QTy5EwCMz\/Y8aMSXFJjOcstSH9brnllnjssceC2xXvftaupZZaKoVZuOOOOxLv+FeyeDKi2LmG9wly7pmpUPnTEg4IgeG9U+UuvfTSlrShFZW2DGw6CzjGXzxCSCeGEmMvG25svvnmyUIp\/AEyZiP1mHdt47vuuusGd67TTz89ePcbyzGoWBXAQGJSnCuYaQCAY\/k0tcB66RnqL9QaDti00uJh0dTMf\/oRbE1Lmltry8BmPEayMV4Yf\/EeMe5yNH\/Ge79KJB2LZK1WC5OOfCmZ+M2JMPtjG+OIsZ0QC8qTkLVaz7vYuK9QczlAS\/FuDCVYke1C29wWtKa2loENg0kqDsI777xzkER2IWGqr9VqreHGoNVaHpw54EfWcGDkyJEpiQcRy3S6GOJ\/Wga2Kl85CXM8tnrbWAwIq\/nlvDM4YPzM3Sm3liWYpZgpXkgK0zTmSrPFWTnjasMI50OdWgY2k9PGaMLZUSmM03iSGDwbyw11xg+l\/vmBtI+eOVOSKvfN2MzQgMpPrfeOTSbzBGLI8qOqvPnQfM9QPrYMbLVaLRgwzIUx04ucZd7toIMOCr92Q5npQ61vpJVpl89+9rOv6ZqxN2OWNYW0FmNr1maGKsMGAXoYsLz319w4RC9aBjb8NHGN+X4BeX2II2IeTYxI+YU6gwMswbx4zHnmFnufxuU0GGm1Wi2Fs+MtBHgknQhX7pM\/HKgxYJsFTgEUtePQQw9NcUZYEBFvErr+LDyy3NJmHCDxjM\/arFkta07LwGZuhfpAj6fvZ+J+NVx0+Ja99SZUzKTv\/RqDq44DgvGaJTCuhyO1DGzGZQwh5lyGI+OHep9rtVoKwGvcRlN54IEHgsV50UUXHepdr9u\/poONhwhzML9HQPvMZz4T22yzzYyNNOQpU7fFJaNjOGBXWeM5Zn9beHESF0y3YzrQ4IY2HWzUC0FYd9hhh+BBYu4F8Fwjeco0uJ\/lcU3gAE8g\/qy5KlZIwwIRzXgDcZXLeXWPQzij6WAzaOYHafKTzyMDSa1WSwF9LAjlkqXMEOZ56dow5UDTwcYKSZJxIhaghzGEcYQbzzB9B6Xbw4QDTQcbKyQ1URDWUaNGxdZbb522\/C3jtGHyxQ3jbjYdbHhtUadjJi47RbJlbpTjUOVAS8A2VJnZl36VMsOXAy0Bmz3ZrKwWlIVvnHVpjq4t9MxxRobvayk9H4ocaDrYWBxNdIop0h3JU2YoMrv0aXhzoOlgG97sLr0fzhwoYBvOb7\/0vakcaDTYmtr4Tq9MgCJ7GNjTIFMO09fJfeMxYoMPy2y4afESqvZHv0VSq6blc87Khx9+ePCnzGlD5VjA1uI3aUKfK1MmG4PkJvGUF68jX3fCEcDOPffcmNV1albtW5bDaNYJ\/e1PGwvY+sOtJpX1qz9x4sQQeUwAWzv7cNjm0GsPBL\/+mmKvBMFsSUXh\/Pbee+9gySVNHJUhVUgSwBWJTFnPsZMLYMj3zM022yysmBfdjOMBySKepzSLQq2sBqIdd9wx8pwoCYTUk0kcEis6LBDNafWOpLj2IHvuaYNNL+0wI8wdt75693ZiegFbi9\/a+PHj0wpm0x5iaN50002pReJzTJkyJW0IaYOQo48+OsRY9LHzJ7WbzwknnBDiZrLg+riBJ93czR\/3ClxrdbTI0rx47H2nqNUXJKwypMq1114bVs\/bd8H2ygBJNRSY5+GHH46HHnooef3Y9ESgVc\/IpPxiiy2WL9Ox2kf9BCQZpDiJrp+8iSwm1i590X9LcpQbKlTA1uI3aburPAVy4403xpJLLplaZP85EuKee+4JH6Rffx+qSML2MiDt7JPgw6zVasHj3mLNdHM3f4DFR23xpq2VgVd0aEVJFVGleemrn+oqSA8gWXNomYwxGCkqdAWQCV+hvq7r00hF6Z6bqdpHff3c5z6Xs9LWzTZQEUTX+FUGwI8cOTJFwnY9VKiArc3fJFVKKDgA86GKSnX++eeHD5pqmJsvfDvVL193PVLPdttttxkbWJBy1M6u5fK1ZwkRn69tbsKJ3I5CJCkVljM5MOYy\/T1q\/8knnxxWemy55ZZRqw3teKEFbP39QppcnsGAkzaQ+ThFfD7kkEPCIkxqnhXQ0kk\/wBSDk4QS7Fa6MZQmkxqkFckDNPbFo07K644AHKiATjg64d8F8LFPgvGgsSRp2\/Ve4Ql7Umer5amq+kN91OacR7Jqu4jJOW0oHAvY2vwtUhN33XXXYOSwh4GoVBMmTAjqHEMIlYyLG9CRdlQ+18LEsQga2wEgNROAHKmoYjc6r9f9cePGJUnDWLHhhhuGVRpUTQDIrnauu96vDuNOYOmaV70GYlLNVswMNIDraPxoXKgvJF71nk4\/L2Br4Ru0qrm7Dx6YUG6asRLDBsl02mmnhfiLtVotxGkUjYxUo4b5wGu1Wljtboxms8jjjz8+9t9\/\/yA5SCfpiEopTf3aketSrzQfuwC6TPDqVVetVkuWSGCwsfDiOssAAADuSURBVEmtNrPax3JJApKGnqG9npef76g+bZFn3Kj9yA8JgOmreJMDUVHV025UwNayN9J5FVMfgY6kFJS1ux4A2KabbhrGdMDfXZme0kxrWHJFOvdUrhPzRnRio0ubZ+YAKyKr3sw5jUvhIH7BBRckSdmT1GHdNGdWq80s+XprjTEqlZnU7a1sp+UXsHXaGyvt7VgOFLB17KsrDe80DhSwddobK+3tWA4MHtg6liWl4YUDg8OBArbB4Wt5auHATBwoYJuJJSWhcGBwOFDANjh8LU8tHJiJAwVsM7GkJLQdB4ZIgwrYhsiLLN1ofw78FwAA\/\/\/l37YlAAAABklEQVQDAKH0fFAZmakMAAAAAElFTkSuQmCC","height":132,"width":219}}
%---
%[output:67464f46]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANsAAACECAYAAAAZW15iAAAQAElEQVR4AezdB5xdRfUH8HkJYggICQQBkZjQQeldEI10pNcEEASlg4AgQfmDUaQEkCKEKmioIi0EpUgvkSqEorRQpCSAAgElAiL+93vJvNz39r7dfZu3my2TT87OvTNn2rnzmznTzuvzv9y\/adOm\/e++++77HzfnXfjYFp7CiMkzSaCXSqBPyP375z\/\/Gc4+++xwySWXhL\/\/\/e+5kOaPc8wxR3PP5JMkkCRQUwIVYPvc5z4XNt9883DDDTeEddZZJ2y44YbhqquuCkBYM4UUkCSQJNAmCVSAzWi13XbbhWuvvTY8\/PDD4eCDDw7jx48Pq666ath+++0zEP7rX\/9qU8KJKUkgSaBSAhVgywcZ5TbddNNw0UUXhccffzwccMAB4fzzzw\/f+ta3wptvvplnTc9JAiSQqBUJ1ASbeP\/973\/Dc889F0477bQwcuTIMGXKlLDNNtsEQBSeKEkgSaDtEmgGNgB79NFHw5FHHhnWXHPNsNNOO4UPP\/wwXH755aFppTIcdNBBgbrZ9iwSZ5JAkgAJVIDNCuTGG28cdt111\/DRRx+F8847L9x\/\/\/3h6KOPDkOHDg2lUkmcLkm33nprWHTRRQO30QWUZj7tww47LHz9619vqDptLqxjQ56feOKJsMIKK4Rzzjmn0dVpll51\/ZoxTPdQFmVStuleNZ16eGsmUhBANmTke\/gOBSyFXjGeuJ4LmTrYswJsc801VxgzZkx45JFHwkknnRRWWmml0Ldv3w4uQmOSX3\/99cMLL7wQuI1JcUYq0uyotGfkMmuezL+POeaYcPjhh3eI7BpdqxdffDH85S9\/yaYzJ598cqOT79D0KsAG8WeeeWaYOnVqYaaxB9SrIL0XRh9MT6\/X4Arj8heO8nHzYdF\/s802y0YmcVvqseSJJ1Lkjelw5SsPYUi51E2PrGeOcfHgFYeftPNl5T\/j\/dNRE88111wTXnnllbDBBhsEaeKJJI60uPyEy1M87\/yFR4r+wiL94x\/\/yBakbLmceOKJQR2EcWM8rnf+yDM\/ea277roVI6+6k4FwlM\/T4pfvvfbaa0smG63JBR\/yHGWkLMq05ZZbZhpErBs+JG9+6ljNK3H54kPKo1z8q0ka0sKH1A2PclioUwbfIPoLi1QdN5Y\/hr\/66qtBXaWbD1MWZeIfSXnF4yrPpZdemmkb6sdf\/niFVctcWaUvHIlTAbamjf0wadKkcPzxx4cf\/\/jHFUS13GuvvcLyyy8fhg8fHpZddtlAoAoiY6THAdbrrrsuA6xw\/jISV+\/p4y688MLZVoICCUdLL710NjJZgLnllluaNWI80pGmdIw0XELnL7yahCH+sQF\/+ctfzlZXq8uIpy20zz77ZL3qIossEpRzueWWq4hGPsLIUcCECRM42QdWziiHWH71ycsQ86BBgwI5WohSRz24uOpCtReXnB566KEMHPkwZSqVZqj7GpE8X3vttWxKIH4+z5tuuikMGDAgLLDAAgHvwU3bPb6P72QKoTz4je7KokxkZz6vneTliVd9q3m9q6N05C9d5VEueYoXCVh22WWXrCNTT\/zqrWF\/\/vOfD1deeWUgX\/UnlxiPqz0BY75MOhL5CkfvvvtudmhDHYRJm7+DHNovf3VXv7vvvjuTiXAAP+qoo8p7zi3JXDlslUU5kpu69pFQJIL84Q9\/GNZbb72scegBIpnDzTfffGGHHXbIwmwB4FegadOmZUmo5NCmuZ0GqNePjcEHxSstqqpewMigUlnEpj\/mik1OWHzxxTmFpAHHdDBo+D6Ij+m9mnwUH\/ayyy7LhES48lYGZXzsscdC9QerTqPe9\/79+wdCJhdC50a5kIMyaSjSJQ\/1wfP+++\/zqknqqK7ArMeMjUSEmK4wDXK11VbjnVFUu\/gJw6MM8nzppZeCb6S8yu07AgGZ+b4SuOuuuwplRIbkijRUo50GKU41AZT85Ct\/5VAejVv58vzA6n233XbjZJ27eMpJnplnjT9vvPFG1sm39I3jt9C56GS0KclpS9qDZ9+FDDznCWh8A9+iJZlr19p3LIf0fOcKsFllHDZsWLaXRtiRfNw+ffoEH0RG\/LfddtusR\/RxWmsoCuxD+CCG1HxPI6yjKDaijkq\/KF2NkJA1JEInH+94J0+ezClT\/OB4\/v3vf5f9ix70+NQVPa4PGXkApDrdog4LOMm+On5MhwsEI0aM8JipyXjF0YtnnlV\/jDbC2\/o9lTumqTxVyWWvsfFnL01\/tDnfUUcJTE1eHfJfHdVFG9VWizKJctV5tEXm5JJPswJsRRnw04C+8IUveCyTihMAQcw555xlfw9FhdE76S30DJEAF389RBDyFicKiOu9LRQ\/pl5SJ6LBaPR6nrbEbwuPngyf3o+MvLcmQx2dOLVo7NixWRA1h\/zi6KgxVn+bWMcswvQ\/sVcWFxmRxJ0eXHb08MLlE2ViAYW8ykxND8BPZVWOav6m4ML\/gKYDwo+MJDSMPHNs0NFPZ6IzMgr5TtG\/JTfWX5njN26JX3u98MILM\/VUG1VGZa0Vp+hbxjzzcajA6hlpBtjyXAXP1Dw9U+yRDPcavl47fjS9ObUAeaYq6C1jXJWQtAZOCIThva2k0WoAGrE4XCCmmnhviai3VIiojiiL+ihbjBcFJt3o1x435kVW8vQuHXnJk7\/3vAyrOyzheco3QnJTjxguTLrqVB0mb2WgxmlUQGKENIfybXwjjVmjFtd38X2AQPk0Oh1q\/MYxTw0fAOI7Xu0hvuddjVM70Sa0DeWwGIE853l9Y++xc1EndVNO5RVWi2JdyUZdYlxyrxWHv\/LlOyxlVFZhtaglmWuP2mVsRwYDI1ybwWYUgtQ4NHL1lnrCWKB55pknmNwain0I4cJiXJNEmeoRLQC0Jjxx86QB7LvvvkFjrTcdAv3Rj36U6fQakLIon7LFjxTTffrpp\/PZNnv28TQA81KNt5pBXhoXf653z\/JqTYb4UCwTOWv8RhAyJds4+VYGDco3EK5OymShSxpI3vL0rBGIj0ccfupi9KUt+B4WAaIc8APiaaedFqQTgSAN+VI5I6+DEDrCCOo8r8YmP99O3Op0lSOSb2wOqI34xuqkbm2ZWytjrW8c06\/lmiOSg7ZhcXDw4MHZFgPgFcVRH+VSPvLMy5wcLeQAfayDttYi2Ajacufo0aPDyy+\/HJZZZplsxTAOizLMF+SLX\/xi0MMJv6tpYi3TGK6h8Ud59SH6c\/FKMx\/OL0\/CpYHyfOLz48pX\/lQlHyDG9yHFwYekJQwPXn7o97\/\/fVZPaQnn8ufm36UlTX7VJG1xuPkwafCPFMNjGZTDM\/KMT0OLdfIe6+ZZetLH412ZVl99dV5lyqeFB28M1LiMfL4bP+nhiSQveQtTV+kLw6fsntG5554bhCmz\/Kp5xc\/z59MVlqd8XGnny6ss4ub9WoorT+HKpGzIc3U6+Tyvv\/76gNSHvzSUQ52lFUkZ+OOrlnlMXziSRk2w3XjjjWHrrbcOd955Z1A55yL33HPPbFSJmSW3a0iASqgH5VLLTN6LVL+i0mp4e+yxR7aNYwQq4kl+zSVA1vXKvBBs9iKchdRbHXvssdmqI7XGCDdu3LggPJ99RHHsNfJh6bnjJaDXjL0n8PgOyHNbctdji89tC3+rPL2AoT0yLwSbg8f21UyC83Lz\/vHHH2cHk\/P+6TlJIEmgdQkUgm3eeecNCy20UHbSP+6h\/ec\/\/wm\/+93vwsCBA4OFkNaTbh+HoTnRouWja0kWnSuLL37j22HeH9yR0eUPvd6+RlwjViHYZptttuykv0URphHsPVhVu\/fee7OrN5\/97GdrJNcYbypNoheyRZp65eAL1Bsn8c+QtQP4ZIgWGdiP0zAqBJvUBzaNYKeffnp48sknszN1Vlx++9vfhvx+BL5EXUsCt912W9cqUCpNWQKFYHOB1N02G4NxMSRuevIThqecSi97SNVNEmiPBArB9vbbbwebdSxsbbHFFoERIFdg1lprreCdBa64admeTFOcJIHeKIFCsFEhF1tssfCLX\/wiUyGdCnjggQfCqaeeGjbZZJMQ5269UWCpzj1bAi+\/\/UGHVbAQbEY2G6NrrLFGOeNSqZTZJHnmmWeC8HxYmamND1Y2XeWxMdjGKIktSaDbS6AQbM642RB1oiDOzZz9sir5ySefBKuREydObHflbSE4ldLuBFLEJIFOkMDgeTt+NTKznsXIj2sWzs197WtfCyuvvHJwmsRBT1sCJ5xwQruqC6RU0h133LFd8VOkJIGOlMCESe+Uk+8UsMnNoeIrrrgiPPjgg+Hqq68Of\/zjH8Of\/vSnzAaDw5m33347trrIFQxXJ1xdb+lKiZPWiV4M7ZEBGxvtiZfifCrv+5vWJjTqRgNNmoVqpADkUOsHH3w6YaRGPvvss5khGkv\/wush8V2dMEouueSSLUZ1DjPR0Mx8YL1y0EnWGyfxfyrrvvMsFD4etFTWNgc3eENbojXB5uYqU3auYERyC6Bfv36ZmilyPeTY1z333JNZinIEyT0t5K5WPekk3iSBjpJA\/njWiNUXang2hWCz2ujynh\/YQPvvv39mhpw1ZHfa5pprrroLIo6T6PFokMt0yJ2guhNLEbqLBLpNOe+dNDWMvvnFrLx9pv0jjFhtwey5kX8KweZkf6lUCq7OzD\/\/\/OH1118P1Em3bO2xAWMjC5HSShKYVRKwrzb65pfCFmc9Wi5Cv6fHl58b+VAItrj0b3T7zGc+k5mBAzhX56dOnRridsDMFMR9IDQzaaS4SQLtlQCQURuBLI5o0hq50dAw+8uf2vr03kgqBBtLT5b4\/UYb46ZGNObr2Iz0M1KMiDayECmtJIHOkACAURf3v\/ypsOLP7wtcfvK2+ghoIzca4rVDqBBscrJC5XiWOZojWgzbsDjENEKpVMKSKEmgy0sAmIxggAVgRjLvseBAZn42fr+VQkcCTX6FYHOyn5kxLiZEnTTKDR8+PDN5zS9RkkBXkkAeWFuMeTS7AApggJYHmDID2cgmlRHIxoxYJnjn35FUATbgYjfQUr+jWVzL9JHcArAZXW1DsCMLmNJOEqiWAFBRBwEIkIqAde\/zU6ujZYCKAJv4f2tlI1lngCwWpE984Fp9dGbRcSpXarhxqT66F1xwQWZDEH+iJIGOkkA1oKpBRR3kB3C1gAVIVEQj19unDAsRYOssPqCjit1iuhVgi5yW+3\/5y18GbvRLbpJAIyWQBxPAAI4RitrHBgg3Dyg8RaBSJqBaZ7EB2d4YYFENAQt5Bzh8s5oqwBbVyKg2FrnUTHyzuuAp\/64rgSIgtQQmYRFM4taqWRGo4og1fv+VQgTWrBq5apU7+leALaqRUWV84YUZhlCiHzUTX0wgub1HAoCA4nwJQAAF1RqVhOFDRibxa0kMmJCRCAGPUQp1J1DVql8F2KqZnAS3\/M\/2OTPkaUSrllDPeAcA1BYQ5dW7eoBEUoCEAAkBE8qDKap+\/PEYpZD43Z1qgo35cXfOjGgq+m2I0AAAEABJREFUaUSz38bfe6KuLQHgQbUAVD0SzQyISAKI4rwJSIAFARIAxZHJM3+ED\/UUMJFDS1QINha1Lr744nDGGWdkdNxxxwW\/hHLWWWdlhlvdS2sp0RTWeAkADgIeRC1DRhcEPGjzsa+W95daAlBrKl2sARChCCRL54CCqoEU503CgAgBkvgxvd7sFoKN+XFCcYqEG8m7Q8qt\/Upm5E9ubQkATqQ8eByKzYMHYOLqnGcrdAgPAjgEPGjyex\/XznR6iMaPIoCAIoIIgJARKD8aRSCN3GhItuonTl1Amp53b3YKwca8OAtbbIUwzkNADh9TIZ2bdFCZX6IZEsgDpx7wRAABDnIoNg8e6c7IpfYT8KBVFu5XBsPIjYZmK3RGmpYAJHzkdBABEJJW7dxSSHskUAg2Bn2OPPLI4LInEwhuV6+44oqZCnnEEUe06\/Joewo3K+No5KgIOEBBZUPA0tLI017wqLsGj+oZgc7bZsEywCKA0ihEmrOeCsGmWMyM522QWCAxsi2xxBKCKyiaPPCDcF\/96leDhRUrmRVMTS9WM+3Txf07z\/yagjrtPwAhowdqi9oGXCgCRzwqG5JWWwsPOKgIPEaX\/OhTpMLhiQAy+iDptTX\/xDdrJVAINgBgAuFnP\/tZeOutt7KTJH7ZplQqPu0\/adKk4Nc6x48fn\/3yqJ8\/taBSXTUHmddbb73yD0YAcEft2QFBHJUAxSiUH4H4oQggwEHiVZe76F0jR4CDjB5oZA3VrTXwiJvAUyTpnuNXCLb55psv+yVK6uS3v\/3twOw468gsNxnFqqvvfpv7b36\/TZib3PHZeyT+c889d3xtuAsoRh3Aot7FhQR+gNRShoCDAAdp\/Mhogow6qGjhwOIBHjRy+txH3ASeliTe+8IKwda3b99AXRw5cmRmavz6668PrDbtscceYauttgrV1rUspqywwgrBL6gwW86Klp+aqhanLQNprbbaasENAr+KY+Glmo8KWg9NeOy58KPfPlq+EFgErC\/MPVuweLD5MnOFvVcfEEatPyiY36A\/HzgkXLvzghmdvumAgA5bs19Aaw76d0AL930noP++O6VdJubqqc\/M8OoQZyZ+b49rilPdHhv1Xgi2mDggPPfcc+HKK68MTvtPnTo1mJdZkYw8eXf99dcPzz\/\/fADS0aNHB6bw8uEsc40aNSqzRcmQ0Lhx4zIw53k822JoCzE99rtnS8He0rkPThU1ozhCjWxS6cbvt1Kgwj056mvhlkPXCmP3XC0cP3yl8P1NlwvbrbNMRm3Jq7vw6BS7S1m7YjnjIY6sITX4TyHY3nvvvWDVccWmFUiXSM21xowZE+67777sxxBZysqXgxEg1pL5lUqlsMoqqwR7dX5mil+kYcOGBSubpVIpmwe6L\/fUU0\/F4LpcKuMBlz9VtogkMpABGFWPajeySaWjyglLlCQwqyVQCDYj2pAhQ8I111yT\/YoNU+TUSuplUYGNdEwmAKU53Z\/\/\/OdM7bSoEvlthh9yyCHhqquuCnjeeeed8MgjjwSAjjxtdQHNfCyqi3mQAVhb00l8SQKdKYFCsJmDsXyVA1iLZfI7AAwB+e02iyk2w3\/wgx+E2WabLTjAjDz75RpgM6IxIGRet0bul3JazGR6IKAZ0bi8AI2qmEBGGom6sgQKwVZvgUulUthll12y3wJgTsECiX066ey8884BeeZnUcQNcL8bIE6pVLydgL+I8iuLgEZl5BbxJr8kga4kgYaArbMqZN\/Mvpj8AMyI5jlRkkB3kEAh2MzZ7Ilx85XwXuSf5+moZ2rjidPNQ8vjzOGdYxFJXomSBBohgQqwAZM9NL8uuvfeeweu0ySR2I5kN9KpkkZkXk8aE56fGuKCSNwwrid+4u0yEui1BakAm1HrwAMPDHvttVd4\/PHHM3e77bYLkb7\/\/e9np0ksoHSmxIxqlz84JcuS+jiyaf8se0l\/kgS6kQQqwMaalgWMG264IRx66KGBe\/fdd4c8+YknBls7s47AFke1tRcbEACuM\/NPeSUJNEICFWCjRhrd7Js5iOyXa6IKGV1qJr5GZN7WNPJztRGrNf53s9pajsSXJDAzEqgAG6A5TuUUP3Uyqo95lz++mcm0nrj5Uc0B4XQipB7pJd6uJIEKsFEjGWf1YxrUybz6GJ\/54+usSthXi3kd3i3marG0yU0SqJRABdhiEDWRuhhVx7zLX3jk7Wh3wqR3sizM09Koloki\/emmEigEGzWRuphXH136dBTLSqXwzqhvXoUcPLBfZ2SZ8kgS6DAJFIKNmkhdjKoj11bAhRdeGJxr7Kyl\/5ff\/qBc8RGrp4WRsjDSQ7eUQCHYimpSKpWyqzPUyM4a2fKrkIukka3osyS\/biSBQrCZkwFVfq7m+aabbgpvvvlmp1nXevmdT0e2NF\/rRi0qFbWmBArBZuSqnrOZv7kew78z7EZSIZGS28jmJkoS6M4SKARb0ZzNvM08jv2Q9lZ48uTJYfjw4cEdNibvzj\/\/\/BCNwFanGYHGP21kk0Ki7i6BQrCpFMMvI0aMCMzNeR87dmxmKoF66b09BFwbbbRRdvv7uuuuy46BsXFSlJaDx0X+yS9JoLtKoBBsbPkff\/zxwejDnojKuYU9++yzBz+ywcQBv3qJPcktt9wylEqlzEYJ8wi1VNK0v1avdLshfy8rciHYmJxjGYtl42jcZ+DAgYGphClTpgRzuvbIiSlzgGVEaN111w3LLrtsWHDBBZslxTRdPHg8\/xyhS5uOowF0JUqm7F6cqfbS6abs+vfvH5zsZ7gnj4SXXnopAIuDynn\/ep6B97LLLgvStnd31113NYv+vznnL\/ut\/5WFQlc0edZVy5RM2Q2dqfbS6absAILBnp\/+9KeB+TnPDPq4z\/ad73wn1FL9yggpeHj\/\/feDXzCNpuuMlDbIJ02aVMA9wyutRM6QRXrq3hIoVCNViYVjo465m6NazIvfcccd4Zvf\/Kbguokpc3NBJu\/M1ZiymzhxYiha3UyLI3WLN0XoBhIoBJt5mZVIdiCNPszOMa7anhEtyiCasmOLcu211w6bbbZZYEGZGbzIE90JPeTwcaxPcpMESKAQbBYtNthgg8zsuMUSjI0gpuzM15ixmzBhQmb+rlRqbsqufHIkHdFqhNhTGl1EAoVgs5dmX83dNuqkFZpIHf2bap\/0HxTyG9pdRE6pGEkCMy2BQrD5zTTzNSsz1cRf+EznXCOBT\/rPVw5Ze\/GB5ef0kCTQ3SVQATYHkO2hOUJldHP4uJr44+uMiqeVyM6QcsqjsyRQATZAGzVqVJjUtBzvwLHDx9XEH18IoUPK+PGgpTsk3ZRoksCslkAF2BxANk+blTZIPh60VFkmyQxCWRTpoQdIoAJs+fo4gmT53xyNvwUTv9lGjfTe0eQOW0fnkdJPEuhMCRSCzeazzexGH0RuS8XiyJZsjrRFWomnO0mgEGz21jriIHJrgskv+S8yb7\/W2FN4z5FAr6hJIdg68iByW6U6eN452sqa+JIEuoUECsHWEQeR2yKN\/Mg2OI1sbRFZ4ulGEigEm\/I7OWJxxNytEQeRpdka5Q8gL5KOarUmrhTezSRQE2zq4aR+ow4iS68eSiNbPdJKvN1BAi2CrbMrMGH6aX\/59iywqVGi3i6BLgW2+DES0KIkktuTJNAwsLk2Y263+uqrB7e6n3zyyWZycs7SrYFaNwjS1ZpmIksePUgCNcH22muvhZNPPjkzZcCcQaTRo0eH9957r0IEbl2fccYZGf+DDz4Y9t133+DYl83xPKPLqAAZbxJYgMnfIMivRubjpeckgZ4ggUKw2dRmd+SVV17JfkPbzepIRi7GgPKVd4RryJAhYYkllsi8v\/KVr4S33norSCfzmP7HAea55557+lulkwfawNk\/nikLSY6a9VZK1rW6mXUtI1KpVAojR44Mm2++eWAWIRIDQNXWtZZccslgi8D+nOs5TB841DzPPPNUIAr4rr\/++szuyDrrrBNYWC66rrPs4PlnykJSV7V81RnlSta1upl1rXnnnTdr7EzXVaCllZdnn3027LDDDuGZZ54JxxxzTLB1kI\/Sr1+\/4AoPVfPaa68N48aNC\/fee2+eJXs+4\/ijQpzXJXfRumRBTU8yq09m1fLKGmEH\/OlTlOa0adMy+5D7779\/2GuvvSrmbUVzNtayLrroooCf+nn22WcHI1t12kZFhoNKpVIWbg8vmrazAvn2KcPCxP9bK7x658UhzuuS+0KSxQudL4PqttuI90KwmZN94xvfCD\/\/+c8Dc+FxvsYtmrOZJ1APzzvvvABMffv2bVY2JssPOeSQ4JdwgNOiyiOPPBJWXHHFCl6gq\/BIL0kCPUQCfYrqYU5mFIrztLzLX3g+3qRJk4Kl\/l133TUwK478Wo2Fk0svvTSgaMoO2Ixo0txwww2zX7TJp5Wee7kEenD1C8Gmvq7YUBndabM3Zpn+1FNPDfyF5wkAqYP33HNP9ss08eelqJI777xzQPiZsrMo8sADDwT7crvsskv2IxvCEiUJ9HQJFIKNynfCCScE6uHpp58eFlpooWzBxGriscceG4T3dMGk+iUJNFoChWAzn\/rb3\/4WmBz\/0pe+FPr06ROsJH7ve9\/LACi80QVJ6SUJ9HQJFILNAofRy0iWF8CHH36YqX3C8\/7pOUkgSaB1CRSCzT4b8+P77bdfYC7cMSu\/FHrAAQcEm9EDBzbWeKqNcPNDP7Ih\/VtvvTVYsWy9+N2PoxElJhtbLVaGyYsxpqLDAeSY30M67LDDGpF9j0\/Dz5kdffTRDa9nIdjksvvuu2dnHe1zffDBB4Ft\/kMPPTR897vfzUY3PI2im266KVNPLZpYQPnVr36VHddqVPo9LR1bJrfccku4+eabw4033hhuv\/328NBDDzWrpqmA7RjfEDnr2owpeZQlYFA57rjjsv1ie83lgAY91ARbqVTK9sCc+LBc72Cxn4vqCBXSSqebAvb3Fl544WDV0imUBtWxxyUDWGussUagYfhlIRrBww8\/3KyeGk\/1kblmTMmjLAFte9VVVw1bb7112a+RDzXBZt\/MTzpZ+t9mm20ClUUhHPBtZAFsJVhwGTRoUJZsqVTKjifplTOP9KeZBOxrOgMZAxZbbLFQfbSOqgls1CE\/r7z99ttne6ExTnKbS0DnZe93lVVWaR7YAJ9CsAHASSedlKmMfrzQ+cUnnngiOHfnVImDyg3Iu5yEHsXPB5c90kOrEqg+d1od4aOPPgpAdv755wffzq\/G+iVZBw2qedN750igEGz0VYsWRjZAUBQqnkPGgFa9Sil8OtXtzDnnnMFtgcmTJ2dx9chGz6WWWip7T3+aS2DIkCHh+eefLwd4ZjK+7NH0AIxO9MQR0LE4cn799debQtP\/WSGBQrBZjVx66aWDkx4afyyYVRrzKeHRb2bdUqkUhg0bFszbANyl1TfeeCMsvvjiM5t0j42\/1lprhUcffTS7L6jj8wxM+Qq7i0h1BET+Tz\/9dPHuHHcAAAd3SURBVLDQFcHHL1HnSqAQbOZQPqDVRx\/WWUdLzAcffHB2JcZCCb9GFXXjjTfOJvvy2GmnnTL1dZFFFmlU8j0unZVXXjk4QrfJJpuEjTbaKOus+Kmoc6iI\/Gzd7LbbbsFZVAtdRx55ZCZnfIk6XwKFYKNu0PGdhTzqqKOyS6ROk5xyyikhvrtY2qjiUlGlZyR1rtJI16i0e2I6pVIpUBHNpW2X5M+YOoeK1Jsc8dx\/\/\/3hD3\/4Q3CDnn+iliVg+tQR2ySFYHOq34dyMt+VGUvLCuA9Ty0XOYUmCcyUBHpc5EKwqaUFix133DHba6OGLLvssoFaklazSCdRkkD9EigEmzOQTvcb1WwuO33w2GOPBfs5dtidm6w\/qxQjSaB3S6AQbO+++25mHctmtvkUEZnH+XHEKVOmBFay+CVKEkgSaLsECsFmj8YmsyX4fFLM09kKcOs675+ekwSSBFqXQCHYnKcziu29997BcjH7Ik7lO4RsqbmR+2ytF7HzOM4555yAWsvRiQyycdKmNd7a4ZUhVhWXX375UM\/JfKf66+GvzLGxb8rihoEtCZavG5t6CDF9W0ONlHujy9lSeoVgE8EezhVXXBGcWbzvvvuyKy8XXHBB2H333QUnarAEHn\/88eBcXkcsOdcqKi2l6GpOLf7W\/C2k2WLIW7luLU5bw62Gu+bl4HVb43Q1vppgU1BGRVnEsihyxBFHZGftSqWSoG5LGpiRWsOw4MMwkZVXPeeJJ54YkNEN3yWXXBKMNvjcSnA4W6\/tXp8rLsz86WWt0Br1WYSW7h133FEoHydknFWUJl5m\/5wAiXkzbls0Ukn\/wAMPzCxOS1+DjhmYX9sTVUbbMo66CZOXujgJJC97ceopTB5AbQFMWfEyH4\/XwYKf\/OQn5RGWHNj4tP0jnVhm6dQiMjHyn3baadmGunSVRT7VcdQ9P1pVv1fzd+f3FsHWnStWq+zOBgLRxRdfHBiV1bgAQM95+OGHB7TPPvsEJ+v1pAgfsP3mN78Jeu0zzzwzuFzrrpizo1TtlVZaKfz1r38NY8aMyQArTnUZNKTbbrstu4NmJDM3dnDAAW\/5IiDIx7Py6\/A340niAMVZZ50VHMfCN3HixLDttttmhnGZGnSplL8DAtRdWol4Cy64YBg\/frygjFzJ0emot\/uEru3gBSyrzxlT0x+b4jQaWo50YpmBsCm45n+diON97tupNzVZPjUj9IKAXgc2jdfVE1aZHao2SunJq7+1w77AxMU3YMCAapbs3Uhi5NHgrdw6NuVQMOBlDNP\/yNeIZCR1Z8\/BAc8ugirPdLZmjpH05ZdfDubQ4gD1qFGjssPbmN1rAzKLVq5BaeT8jUSA79qIEcU8nH8keQsDGqOblWfvQG2kiXw33HBD2GyzzbJrT\/IX77nnnmv24yqRP7rs1jgqJk3nMZ3TdP41hvdGt+PA1kWl6cwgi8133nlncFfPiOU4U3VxHdplQl0D1tAdI6vm8W7OYyTQ4C0QUOfGjRsXjKDCI0nPloktlOinIRopNPjoV+1aAQbUCBagAiRx8TLExM9znsSj5gKjebb90nx4LMf777+flTW+49FpcBGgUgHVDbnTCPz2YoXXIrc5FlhggXJw\/rns2cseeh3Y7BPq\/Y1aDluzGHbuuecGjS7\/7alcGhr1h7qlkeXD4zM10khGLQM6RLXTwCMPVwPu379\/hd1NWytGAIDDU0TAxB9YuUbZX\/\/615kZCe+1iGoMlDqSq6++Ops7FfHGcqlrDDfnis\/CdTrqhYBWZ2IEjDxFrvKaT8YwannR4ob6ySPy9WS314FNQ2IT08ePH1Zjj6ODUYS\/xqL39m5+NHbs2PDJJ5+EOAp5xjd48ODsB0TMR4xy1EcHgavnbPIYNmxY9ss91E7lYG\/FyBNHLelVExWM3U4qqLKY\/+gIpFfNm3+P5VcmCzvmYuLneTxLxw0Ohp3c9mAr1BxNGBKms3H1CdCZxzDSSVd4LQJe8zy\/5UfVVmYm7av5bSMpq7TFYVelmqenvPc6sLkn5\/cLzLGsrhnhzNs0OqfijXJWI\/H4+CussEKwArfHHnsEo8SFF14YLDYADDVNQ3C0TWNacsklA76DDjqo8IS9y7fmdFYB2boAXKuY0qhF5kmjmuZod911V3Ch1jzMDYnWRhZlY6TJqKtOVjMtrJifVedlvgbUVObhw4dn13cij+0IqratIKuoVEgLObFzinxFLlXSdawtttgis+tBbcdHvuaFOhzfgEzkYQ+XehxHQAs8wM4Vr7tTrwNbqVQKlsGtrFGLjEhA5kNamfRhrUbaX6SuGaGuvPLKbA+MSgkcGjpVymhgrqOhGqVc1ARIK3ylUvMtEuqShuoiJ3JlKTYseSLlqCYXdmP6ymv5H4988quX+fehQ4dm12osZljB3GqrrbLVUqOrOHilgYzSVlTxWn3UISmrMGqy6zzkJZyFbIAQ1hrJg3pNpnvuuWeIaapnlB0\/nYfVUFbC\/MKtzoFcmXUwV15uueVay6pbhPc6sHWLr9LJhdTwjZ5UOSO2\/b44CnVyUXp0dv8PAAD\/\/8AGPcMAAAAGSURBVAMAknmGfM5hE0QAAAAASUVORK5CYII=","height":132,"width":219}}
%---
%[output:2eabceb5]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANsAAACECAYAAAAZW15iAAAQAElEQVR4AezdB5xdRfUH8HkJYggICQQBkZjQQeldEI10pNcEEASlg4AgQfmDUaQEkCKEKmioIi0EpUgvkSqEorRQpCSAAgElAiL+93vJvNz39r7dfZu3my2TT87OvTNn2rnzmznTzuvzv9y\/adOm\/e++++77HzfnXfjYFp7CiMkzSaCXSqBPyP375z\/\/Gc4+++xwySWXhL\/\/\/e+5kOaPc8wxR3PP5JMkkCRQUwIVYPvc5z4XNt9883DDDTeEddZZJ2y44YbhqquuCkBYM4UUkCSQJNAmCVSAzWi13XbbhWuvvTY8\/PDD4eCDDw7jx48Pq666ath+++0zEP7rX\/9qU8KJKUkgSaBSAhVgywcZ5TbddNNw0UUXhccffzwccMAB4fzzzw\/f+ta3wptvvplnTc9JAiSQqBUJ1ASbeP\/973\/Dc889F0477bQwcuTIMGXKlLDNNtsEQBSeKEkgSaDtEmgGNgB79NFHw5FHHhnWXHPNsNNOO4UPP\/wwXH755aFppTIcdNBBgbrZ9iwSZ5JAkgAJVIDNCuTGG28cdt111\/DRRx+F8847L9x\/\/\/3h6KOPDkOHDg2lUkmcLkm33nprWHTRRQO30QWUZj7tww47LHz9619vqDptLqxjQ56feOKJsMIKK4Rzzjmn0dVpll51\/ZoxTPdQFmVStuleNZ16eGsmUhBANmTke\/gOBSyFXjGeuJ4LmTrYswJsc801VxgzZkx45JFHwkknnRRWWmml0Ldv3w4uQmOSX3\/99cMLL7wQuI1JcUYq0uyotGfkMmuezL+POeaYcPjhh3eI7BpdqxdffDH85S9\/yaYzJ598cqOT79D0KsAG8WeeeWaYOnVqYaaxB9SrIL0XRh9MT6\/X4Arj8heO8nHzYdF\/s802y0YmcVvqseSJJ1Lkjelw5SsPYUi51E2PrGeOcfHgFYeftPNl5T\/j\/dNRE88111wTXnnllbDBBhsEaeKJJI60uPyEy1M87\/yFR4r+wiL94x\/\/yBakbLmceOKJQR2EcWM8rnf+yDM\/ea277roVI6+6k4FwlM\/T4pfvvfbaa0smG63JBR\/yHGWkLMq05ZZbZhpErBs+JG9+6ljNK3H54kPKo1z8q0ka0sKH1A2PclioUwbfIPoLi1QdN5Y\/hr\/66qtBXaWbD1MWZeIfSXnF4yrPpZdemmkb6sdf\/niFVctcWaUvHIlTAbamjf0wadKkcPzxx4cf\/\/jHFUS13GuvvcLyyy8fhg8fHpZddtlAoAoiY6THAdbrrrsuA6xw\/jISV+\/p4y688MLZVoICCUdLL710NjJZgLnllluaNWI80pGmdIw0XELnL7yahCH+sQF\/+ctfzlZXq8uIpy20zz77ZL3qIossEpRzueWWq4hGPsLIUcCECRM42QdWziiHWH71ycsQ86BBgwI5WohSRz24uOpCtReXnB566KEMHPkwZSqVZqj7GpE8X3vttWxKIH4+z5tuuikMGDAgLLDAAgHvwU3bPb6P72QKoTz4je7KokxkZz6vneTliVd9q3m9q6N05C9d5VEueYoXCVh22WWXrCNTT\/zqrWF\/\/vOfD1deeWUgX\/UnlxiPqz0BY75MOhL5CkfvvvtudmhDHYRJm7+DHNovf3VXv7vvvjuTiXAAP+qoo8p7zi3JXDlslUU5kpu69pFQJIL84Q9\/GNZbb72scegBIpnDzTfffGGHHXbIwmwB4FegadOmZUmo5NCmuZ0GqNePjcEHxSstqqpewMigUlnEpj\/mik1OWHzxxTmFpAHHdDBo+D6Ij+m9mnwUH\/ayyy7LhES48lYGZXzsscdC9QerTqPe9\/79+wdCJhdC50a5kIMyaSjSJQ\/1wfP+++\/zqknqqK7ArMeMjUSEmK4wDXK11VbjnVFUu\/gJw6MM8nzppZeCb6S8yu07AgGZ+b4SuOuuuwplRIbkijRUo50GKU41AZT85Ct\/5VAejVv58vzA6n233XbjZJ27eMpJnplnjT9vvPFG1sm39I3jt9C56GS0KclpS9qDZ9+FDDznCWh8A9+iJZlr19p3LIf0fOcKsFllHDZsWLaXRtiRfNw+ffoEH0RG\/LfddtusR\/RxWmsoCuxD+CCG1HxPI6yjKDaijkq\/KF2NkJA1JEInH+94J0+ezClT\/OB4\/v3vf5f9ix70+NQVPa4PGXkApDrdog4LOMm+On5MhwsEI0aM8JipyXjF0YtnnlV\/jDbC2\/o9lTumqTxVyWWvsfFnL01\/tDnfUUcJTE1eHfJfHdVFG9VWizKJctV5tEXm5JJPswJsRRnw04C+8IUveCyTihMAQcw555xlfw9FhdE76S30DJEAF389RBDyFicKiOu9LRQ\/pl5SJ6LBaPR6nrbEbwuPngyf3o+MvLcmQx2dOLVo7NixWRA1h\/zi6KgxVn+bWMcswvQ\/sVcWFxmRxJ0eXHb08MLlE2ViAYW8ykxND8BPZVWOav6m4ML\/gKYDwo+MJDSMPHNs0NFPZ6IzMgr5TtG\/JTfWX5njN26JX3u98MILM\/VUG1VGZa0Vp+hbxjzzcajA6hlpBtjyXAXP1Dw9U+yRDPcavl47fjS9ObUAeaYq6C1jXJWQtAZOCIThva2k0WoAGrE4XCCmmnhviai3VIiojiiL+ihbjBcFJt3o1x435kVW8vQuHXnJk7\/3vAyrOyzheco3QnJTjxguTLrqVB0mb2WgxmlUQGKENIfybXwjjVmjFtd38X2AQPk0Oh1q\/MYxTw0fAOI7Xu0hvuddjVM70Sa0DeWwGIE853l9Y++xc1EndVNO5RVWi2JdyUZdYlxyrxWHv\/LlOyxlVFZhtaglmWuP2mVsRwYDI1ybwWYUgtQ4NHL1lnrCWKB55pknmNwain0I4cJiXJNEmeoRLQC0Jjxx86QB7LvvvkFjrTcdAv3Rj36U6fQakLIon7LFjxTTffrpp\/PZNnv28TQA81KNt5pBXhoXf653z\/JqTYb4UCwTOWv8RhAyJds4+VYGDco3EK5OymShSxpI3vL0rBGIj0ccfupi9KUt+B4WAaIc8APiaaedFqQTgSAN+VI5I6+DEDrCCOo8r8YmP99O3Op0lSOSb2wOqI34xuqkbm2ZWytjrW8c06\/lmiOSg7ZhcXDw4MHZFgPgFcVRH+VSPvLMy5wcLeQAfayDttYi2Ajacufo0aPDyy+\/HJZZZplsxTAOizLMF+SLX\/xi0MMJv6tpYi3TGK6h8Ud59SH6c\/FKMx\/OL0\/CpYHyfOLz48pX\/lQlHyDG9yHFwYekJQwPXn7o97\/\/fVZPaQnn8ufm36UlTX7VJG1xuPkwafCPFMNjGZTDM\/KMT0OLdfIe6+ZZetLH412ZVl99dV5lyqeFB28M1LiMfL4bP+nhiSQveQtTV+kLw6fsntG5554bhCmz\/Kp5xc\/z59MVlqd8XGnny6ss4ub9WoorT+HKpGzIc3U6+Tyvv\/76gNSHvzSUQ52lFUkZ+OOrlnlMXziSRk2w3XjjjWHrrbcOd955Z1A55yL33HPPbFSJmSW3a0iASqgH5VLLTN6LVL+i0mp4e+yxR7aNYwQq4kl+zSVA1vXKvBBs9iKchdRbHXvssdmqI7XGCDdu3LggPJ99RHHsNfJh6bnjJaDXjL0n8PgOyHNbctdji89tC3+rPL2AoT0yLwSbg8f21UyC83Lz\/vHHH2cHk\/P+6TlJIEmgdQkUgm3eeecNCy20UHbSP+6h\/ec\/\/wm\/+93vwsCBA4OFkNaTbh+HoTnRouWja0kWnSuLL37j22HeH9yR0eUPvd6+RlwjViHYZptttuykv0URphHsPVhVu\/fee7OrN5\/97GdrJNcYbypNoheyRZp65eAL1Bsn8c+QtQP4ZIgWGdiP0zAqBJvUBzaNYKeffnp48sknszN1Vlx++9vfhvx+BL5EXUsCt912W9cqUCpNWQKFYHOB1N02G4NxMSRuevIThqecSi97SNVNEmiPBArB9vbbbwebdSxsbbHFFoERIFdg1lprreCdBa64admeTFOcJIHeKIFCsFEhF1tssfCLX\/wiUyGdCnjggQfCqaeeGjbZZJMQ5269UWCpzj1bAi+\/\/UGHVbAQbEY2G6NrrLFGOeNSqZTZJHnmmWeC8HxYmamND1Y2XeWxMdjGKIktSaDbS6AQbM642RB1oiDOzZz9sir5ySefBKuREydObHflbSE4ldLuBFLEJIFOkMDgeTt+NTKznsXIj2sWzs197WtfCyuvvHJwmsRBT1sCJ5xwQruqC6RU0h133LFd8VOkJIGOlMCESe+Uk+8UsMnNoeIrrrgiPPjgg+Hqq68Of\/zjH8Of\/vSnzAaDw5m33347trrIFQxXJ1xdb+lKiZPWiV4M7ZEBGxvtiZfifCrv+5vWJjTqRgNNmoVqpADkUOsHH3w6YaRGPvvss5khGkv\/wush8V2dMEouueSSLUZ1DjPR0Mx8YL1y0EnWGyfxfyrrvvMsFD4etFTWNgc3eENbojXB5uYqU3auYERyC6Bfv36ZmilyPeTY1z333JNZinIEyT0t5K5WPekk3iSBjpJA\/njWiNUXang2hWCz2ujynh\/YQPvvv39mhpw1ZHfa5pprrroLIo6T6PFokMt0yJ2guhNLEbqLBLpNOe+dNDWMvvnFrLx9pv0jjFhtwey5kX8KweZkf6lUCq7OzD\/\/\/OH1118P1Em3bO2xAWMjC5HSShKYVRKwrzb65pfCFmc9Wi5Cv6fHl58b+VAItrj0b3T7zGc+k5mBAzhX56dOnRridsDMFMR9IDQzaaS4SQLtlQCQURuBLI5o0hq50dAw+8uf2vr03kgqBBtLT5b4\/UYb46ZGNObr2Iz0M1KMiDayECmtJIHOkACAURf3v\/ypsOLP7wtcfvK2+ghoIzca4rVDqBBscrJC5XiWOZojWgzbsDjENEKpVMKSKEmgy0sAmIxggAVgRjLvseBAZn42fr+VQkcCTX6FYHOyn5kxLiZEnTTKDR8+PDN5zS9RkkBXkkAeWFuMeTS7AApggJYHmDID2cgmlRHIxoxYJnjn35FUATbgYjfQUr+jWVzL9JHcArAZXW1DsCMLmNJOEqiWAFBRBwEIkIqAde\/zU6ujZYCKAJv4f2tlI1lngCwWpE984Fp9dGbRcSpXarhxqT66F1xwQWZDEH+iJIGOkkA1oKpBRR3kB3C1gAVIVEQj19unDAsRYOssPqCjit1iuhVgi5yW+3\/5y18GbvRLbpJAIyWQBxPAAI4RitrHBgg3Dyg8RaBSJqBaZ7EB2d4YYFENAQt5Bzh8s5oqwBbVyKg2FrnUTHyzuuAp\/64rgSIgtQQmYRFM4taqWRGo4og1fv+VQgTWrBq5apU7+leALaqRUWV84YUZhlCiHzUTX0wgub1HAoCA4nwJQAAF1RqVhOFDRibxa0kMmJCRCAGPUQp1J1DVql8F2KqZnAS3\/M\/2OTPkaUSrllDPeAcA1BYQ5dW7eoBEUoCEAAkBE8qDKap+\/PEYpZD43Z1qgo35cXfOjGgq+m2I0AAAEABJREFUaUSz38bfe6KuLQHgQbUAVD0SzQyISAKI4rwJSIAFARIAxZHJM3+ED\/UUMJFDS1QINha1Lr744nDGGWdkdNxxxwW\/hHLWWWdlhlvdS2sp0RTWeAkADgIeRC1DRhcEPGjzsa+W95daAlBrKl2sARChCCRL54CCqoEU503CgAgBkvgxvd7sFoKN+XFCcYqEG8m7Q8qt\/Upm5E9ubQkATqQ8eByKzYMHYOLqnGcrdAgPAjgEPGjyex\/XznR6iMaPIoCAIoIIgJARKD8aRSCN3GhItuonTl1Amp53b3YKwca8OAtbbIUwzkNADh9TIZ2bdFCZX6IZEsgDpx7wRAABDnIoNg8e6c7IpfYT8KBVFu5XBsPIjYZmK3RGmpYAJHzkdBABEJJW7dxSSHskUAg2Bn2OPPLI4LInEwhuV6+44oqZCnnEEUe06\/Joewo3K+No5KgIOEBBZUPA0tLI017wqLsGj+oZgc7bZsEywCKA0ihEmrOeCsGmWMyM522QWCAxsi2xxBKCKyiaPPCDcF\/96leDhRUrmRVMTS9WM+3Txf07z\/yagjrtPwAhowdqi9oGXCgCRzwqG5JWWwsPOKgIPEaX\/OhTpMLhiQAy+iDptTX\/xDdrJVAINgBgAuFnP\/tZeOutt7KTJH7ZplQqPu0\/adKk4Nc6x48fn\/3yqJ8\/taBSXTUHmddbb73yD0YAcEft2QFBHJUAxSiUH4H4oQggwEHiVZe76F0jR4CDjB5oZA3VrTXwiJvAUyTpnuNXCLb55psv+yVK6uS3v\/3twOw468gsNxnFqqvvfpv7b36\/TZib3PHZeyT+c889d3xtuAsoRh3Aot7FhQR+gNRShoCDAAdp\/Mhogow6qGjhwOIBHjRy+txH3ASeliTe+8IKwda3b99AXRw5cmRmavz6668PrDbtscceYauttgrV1rUspqywwgrBL6gwW86Klp+aqhanLQNprbbaasENAr+KY+Glmo8KWg9NeOy58KPfPlq+EFgErC\/MPVuweLD5MnOFvVcfEEatPyiY36A\/HzgkXLvzghmdvumAgA5bs19Aaw76d0AL930noP++O6VdJubqqc\/M8OoQZyZ+b49rilPdHhv1Xgi2mDggPPfcc+HKK68MTvtPnTo1mJdZkYw8eXf99dcPzz\/\/fADS0aNHB6bw8uEsc40aNSqzRcmQ0Lhx4zIw53k822JoCzE99rtnS8He0rkPThU1ozhCjWxS6cbvt1Kgwj056mvhlkPXCmP3XC0cP3yl8P1NlwvbrbNMRm3Jq7vw6BS7S1m7YjnjIY6sITX4TyHY3nvvvWDVccWmFUiXSM21xowZE+67777sxxBZysqXgxEg1pL5lUqlsMoqqwR7dX5mil+kYcOGBSubpVIpmwe6L\/fUU0\/F4LpcKuMBlz9VtogkMpABGFWPajeySaWjyglLlCQwqyVQCDYj2pAhQ8I111yT\/YoNU+TUSuplUYGNdEwmAKU53Z\/\/\/OdM7bSoEvlthh9yyCHhqquuCnjeeeed8MgjjwSAjjxtdQHNfCyqi3mQAVhb00l8SQKdKYFCsJmDsXyVA1iLZfI7AAwB+e02iyk2w3\/wgx+E2WabLTjAjDz75RpgM6IxIGRet0bul3JazGR6IKAZ0bi8AI2qmEBGGom6sgQKwVZvgUulUthll12y3wJgTsECiX066ey8884BeeZnUcQNcL8bIE6pVLydgL+I8iuLgEZl5BbxJr8kga4kgYaArbMqZN\/Mvpj8AMyI5jlRkkB3kEAh2MzZ7Ilx85XwXuSf5+moZ2rjidPNQ8vjzOGdYxFJXomSBBohgQqwAZM9NL8uuvfeeweu0ySR2I5kN9KpkkZkXk8aE56fGuKCSNwwrid+4u0yEui1BakAm1HrwAMPDHvttVd4\/PHHM3e77bYLkb7\/\/e9np0ksoHSmxIxqlz84JcuS+jiyaf8se0l\/kgS6kQQqwMaalgWMG264IRx66KGBe\/fdd4c8+YknBls7s47AFke1tRcbEACuM\/NPeSUJNEICFWCjRhrd7Js5iOyXa6IKGV1qJr5GZN7WNPJztRGrNf53s9pajsSXJDAzEqgAG6A5TuUUP3Uyqo95lz++mcm0nrj5Uc0B4XQipB7pJd6uJIEKsFEjGWf1YxrUybz6GJ\/54+usSthXi3kd3i3marG0yU0SqJRABdhiEDWRuhhVx7zLX3jk7Wh3wqR3sizM09Koloki\/emmEigEGzWRuphXH136dBTLSqXwzqhvXoUcPLBfZ2SZ8kgS6DAJFIKNmkhdjKoj11bAhRdeGJxr7Kyl\/5ff\/qBc8RGrp4WRsjDSQ7eUQCHYimpSKpWyqzPUyM4a2fKrkIukka3osyS\/biSBQrCZkwFVfq7m+aabbgpvvvlmp1nXevmdT0e2NF\/rRi0qFbWmBArBZuSqnrOZv7kew78z7EZSIZGS28jmJkoS6M4SKARb0ZzNvM08jv2Q9lZ48uTJYfjw4cEdNibvzj\/\/\/BCNwFanGYHGP21kk0Ki7i6BQrCpFMMvI0aMCMzNeR87dmxmKoF66b09BFwbbbRRdvv7uuuuy46BsXFSlJaDx0X+yS9JoLtKoBBsbPkff\/zxwejDnojKuYU9++yzBz+ywcQBv3qJPcktt9wylEqlzEYJ8wi1VNK0v1avdLshfy8rciHYmJxjGYtl42jcZ+DAgYGphClTpgRzuvbIiSlzgGVEaN111w3LLrtsWHDBBZslxTRdPHg8\/xyhS5uOowF0JUqm7F6cqfbS6abs+vfvH5zsZ7gnj4SXXnopAIuDynn\/ep6B97LLLgvStnd31113NYv+vznnL\/ut\/5WFQlc0edZVy5RM2Q2dqfbS6absAILBnp\/+9KeB+TnPDPq4z\/ad73wn1FL9yggpeHj\/\/feDXzCNpuuMlDbIJ02aVMA9wyutRM6QRXrq3hIoVCNViYVjo465m6NazIvfcccd4Zvf\/Kbguokpc3NBJu\/M1ZiymzhxYiha3UyLI3WLN0XoBhIoBJt5mZVIdiCNPszOMa7anhEtyiCasmOLcu211w6bbbZZYEGZGbzIE90JPeTwcaxPcpMESKAQbBYtNthgg8zsuMUSjI0gpuzM15ixmzBhQmb+rlRqbsqufHIkHdFqhNhTGl1EAoVgs5dmX83dNuqkFZpIHf2bap\/0HxTyG9pdRE6pGEkCMy2BQrD5zTTzNSsz1cRf+EznXCOBT\/rPVw5Ze\/GB5ef0kCTQ3SVQATYHkO2hOUJldHP4uJr44+uMiqeVyM6QcsqjsyRQATZAGzVqVJjUtBzvwLHDx9XEH18IoUPK+PGgpTsk3ZRoksCslkAF2BxANk+blTZIPh60VFkmyQxCWRTpoQdIoAJs+fo4gmT53xyNvwUTv9lGjfTe0eQOW0fnkdJPEuhMCRSCzeazzexGH0RuS8XiyJZsjrRFWomnO0mgEGz21jriIHJrgskv+S8yb7\/W2FN4z5FAr6hJIdg68iByW6U6eN452sqa+JIEuoUECsHWEQeR2yKN\/Mg2OI1sbRFZ4ulGEigEm\/I7OWJxxNytEQeRpdka5Q8gL5KOarUmrhTezSRQE2zq4aR+ow4iS68eSiNbPdJKvN1BAi2CrbMrMGH6aX\/59iywqVGi3i6BLgW2+DES0KIkktuTJNAwsLk2Y263+uqrB7e6n3zyyWZycs7SrYFaNwjS1ZpmIksePUgCNcH22muvhZNPPjkzZcCcQaTRo0eH9957r0IEbl2fccYZGf+DDz4Y9t133+DYl83xPKPLqAAZbxJYgMnfIMivRubjpeckgZ4ggUKw2dRmd+SVV17JfkPbzepIRi7GgPKVd4RryJAhYYkllsi8v\/KVr4S33norSCfzmP7HAea55557+lulkwfawNk\/nikLSY6a9VZK1rW6mXUtI1KpVAojR44Mm2++eWAWIRIDQNXWtZZccslgi8D+nOs5TB841DzPPPNUIAr4rr\/++szuyDrrrBNYWC66rrPs4PlnykJSV7V81RnlSta1upl1rXnnnTdr7EzXVaCllZdnn3027LDDDuGZZ54JxxxzTLB1kI\/Sr1+\/4AoPVfPaa68N48aNC\/fee2+eJXs+4\/ijQpzXJXfRumRBTU8yq09m1fLKGmEH\/OlTlOa0adMy+5D7779\/2GuvvSrmbUVzNtayLrroooCf+nn22WcHI1t12kZFhoNKpVIWbg8vmrazAvn2KcPCxP9bK7x658UhzuuS+0KSxQudL4PqttuI90KwmZN94xvfCD\/\/+c8Dc+FxvsYtmrOZJ1APzzvvvABMffv2bVY2JssPOeSQ4JdwgNOiyiOPPBJWXHHFCl6gq\/BIL0kCPUQCfYrqYU5mFIrztLzLX3g+3qRJk4Kl\/l133TUwK478Wo2Fk0svvTSgaMoO2Ixo0txwww2zX7TJp5Wee7kEenD1C8Gmvq7YUBndabM3Zpn+1FNPDfyF5wkAqYP33HNP9ss08eelqJI777xzQPiZsrMo8sADDwT7crvsskv2IxvCEiUJ9HQJFIKNynfCCScE6uHpp58eFlpooWzBxGriscceG4T3dMGk+iUJNFoChWAzn\/rb3\/4WmBz\/0pe+FPr06ROsJH7ve9\/LACi80QVJ6SUJ9HQJFILNAofRy0iWF8CHH36YqX3C8\/7pOUkgSaB1CRSCzT4b8+P77bdfYC7cMSu\/FHrAAQcEm9EDBzbWeKqNcPNDP7Ih\/VtvvTVYsWy9+N2PoxElJhtbLVaGyYsxpqLDAeSY30M67LDDGpF9j0\/Dz5kdffTRDa9nIdjksvvuu2dnHe1zffDBB4Ft\/kMPPTR897vfzUY3PI2im266KVNPLZpYQPnVr36VHddqVPo9LR1bJrfccku4+eabw4033hhuv\/328NBDDzWrpqmA7RjfEDnr2owpeZQlYFA57rjjsv1ie83lgAY91ARbqVTK9sCc+LBc72Cxn4vqCBXSSqebAvb3Fl544WDV0imUBtWxxyUDWGussUagYfhlIRrBww8\/3KyeGk\/1kblmTMmjLAFte9VVVw1bb7112a+RDzXBZt\/MTzpZ+t9mm20ClUUhHPBtZAFsJVhwGTRoUJZsqVTKjifplTOP9KeZBOxrOgMZAxZbbLFQfbSOqgls1CE\/r7z99ttne6ExTnKbS0DnZe93lVVWaR7YAJ9CsAHASSedlKmMfrzQ+cUnnngiOHfnVImDyg3Iu5yEHsXPB5c90kOrEqg+d1od4aOPPgpAdv755wffzq\/G+iVZBw2qedN750igEGz0VYsWRjZAUBQqnkPGgFa9Sil8OtXtzDnnnMFtgcmTJ2dx9chGz6WWWip7T3+aS2DIkCHh+eefLwd4ZjK+7NH0AIxO9MQR0LE4cn799debQtP\/WSGBQrBZjVx66aWDkx4afyyYVRrzKeHRb2bdUqkUhg0bFszbANyl1TfeeCMsvvjiM5t0j42\/1lprhUcffTS7L6jj8wxM+Qq7i0h1BET+Tz\/9dPHuHHcAAAd3SURBVLDQFcHHL1HnSqAQbOZQPqDVRx\/WWUdLzAcffHB2JcZCCb9GFXXjjTfOJvvy2GmnnTL1dZFFFmlU8j0unZVXXjk4QrfJJpuEjTbaKOus+Kmoc6iI\/Gzd7LbbbsFZVAtdRx55ZCZnfIk6XwKFYKNu0PGdhTzqqKOyS6ROk5xyyikhvrtY2qjiUlGlZyR1rtJI16i0e2I6pVIpUBHNpW2X5M+YOoeK1Jsc8dx\/\/\/3hD3\/4Q3CDnn+iliVg+tQR2ySFYHOq34dyMt+VGUvLCuA9Ty0XOYUmCcyUBHpc5EKwqaUFix133DHba6OGLLvssoFaklazSCdRkkD9EigEmzOQTvcb1WwuO33w2GOPBfs5dtidm6w\/qxQjSaB3S6AQbO+++25mHctmtvkUEZnH+XHEKVOmBFay+CVKEkgSaLsECsFmj8YmsyX4fFLM09kKcOs675+ekwSSBFqXQCHYnKcziu29997BcjH7Ik7lO4RsqbmR+2ytF7HzOM4555yAWsvRiQyycdKmNd7a4ZUhVhWXX375UM\/JfKf66+GvzLGxb8rihoEtCZavG5t6CDF9W0ONlHujy9lSeoVgE8EezhVXXBGcWbzvvvuyKy8XXHBB2H333QUnarAEHn\/88eBcXkcsOdcqKi2l6GpOLf7W\/C2k2WLIW7luLU5bw62Gu+bl4HVb43Q1vppgU1BGRVnEsihyxBFHZGftSqWSoG5LGpiRWsOw4MMwkZVXPeeJJ54YkNEN3yWXXBKMNvjcSnA4W6\/tXp8rLsz86WWt0Br1WYSW7h133FEoHydknFWUJl5m\/5wAiXkzbls0Ukn\/wAMPzCxOS1+DjhmYX9sTVUbbMo66CZOXujgJJC97ceopTB5AbQFMWfEyH4\/XwYKf\/OQn5RGWHNj4tP0jnVhm6dQiMjHyn3baadmGunSVRT7VcdQ9P1pVv1fzd+f3FsHWnStWq+zOBgLRxRdfHBiV1bgAQM95+OGHB7TPPvsEJ+v1pAgfsP3mN78Jeu0zzzwzuFzrrpizo1TtlVZaKfz1r38NY8aMyQArTnUZNKTbbrstu4NmJDM3dnDAAW\/5IiDIx7Py6\/A340niAMVZZ50VHMfCN3HixLDttttmhnGZGnSplL8DAtRdWol4Cy64YBg\/frygjFzJ0emot\/uEru3gBSyrzxlT0x+b4jQaWo50YpmBsCm45n+diON97tupNzVZPjUj9IKAXgc2jdfVE1aZHao2SunJq7+1w77AxMU3YMCAapbs3Uhi5NHgrdw6NuVQMOBlDNP\/yNeIZCR1Z8\/BAc8ugirPdLZmjpH05ZdfDubQ4gD1qFGjssPbmN1rAzKLVq5BaeT8jUSA79qIEcU8nH8keQsDGqOblWfvQG2kiXw33HBD2GyzzbJrT\/IX77nnnmv24yqRP7rs1jgqJk3nMZ3TdP41hvdGt+PA1kWl6cwgi8133nlncFfPiOU4U3VxHdplQl0D1tAdI6vm8W7OYyTQ4C0QUOfGjRsXjKDCI0nPloktlOinIRopNPjoV+1aAQbUCBagAiRx8TLExM9znsSj5gKjebb90nx4LMf777+flTW+49FpcBGgUgHVDbnTCPz2YoXXIrc5FlhggXJw\/rns2cseeh3Y7BPq\/Y1aDluzGHbuuecGjS7\/7alcGhr1h7qlkeXD4zM10khGLQM6RLXTwCMPVwPu379\/hd1NWytGAIDDU0TAxB9YuUbZX\/\/615kZCe+1iGoMlDqSq6++Ops7FfHGcqlrDDfnis\/CdTrqhYBWZ2IEjDxFrvKaT8YwannR4ob6ySPy9WS314FNQ2IT08ePH1Zjj6ODUYS\/xqL39m5+NHbs2PDJJ5+EOAp5xjd48ODsB0TMR4xy1EcHgavnbPIYNmxY9ss91E7lYG\/FyBNHLelVExWM3U4qqLKY\/+gIpFfNm3+P5VcmCzvmYuLneTxLxw0Ohp3c9mAr1BxNGBKms3H1CdCZxzDSSVd4LQJe8zy\/5UfVVmYm7av5bSMpq7TFYVelmqenvPc6sLkn5\/cLzLGsrhnhzNs0OqfijXJWI\/H4+CussEKwArfHHnsEo8SFF14YLDYADDVNQ3C0TWNacsklA76DDjqo8IS9y7fmdFYB2boAXKuY0qhF5kmjmuZod911V3Ch1jzMDYnWRhZlY6TJqKtOVjMtrJifVedlvgbUVObhw4dn13cij+0IqratIKuoVEgLObFzinxFLlXSdawtttgis+tBbcdHvuaFOhzfgEzkYQ+XehxHQAs8wM4Vr7tTrwNbqVQKlsGtrFGLjEhA5kNamfRhrUbaX6SuGaGuvPLKbA+MSgkcGjpVymhgrqOhGqVc1ARIK3ylUvMtEuqShuoiJ3JlKTYseSLlqCYXdmP6ymv5H4988quX+fehQ4dm12osZljB3GqrrbLVUqOrOHilgYzSVlTxWn3UISmrMGqy6zzkJZyFbIAQ1hrJg3pNpnvuuWeIaapnlB0\/nYfVUFbC\/MKtzoFcmXUwV15uueVay6pbhPc6sHWLr9LJhdTwjZ5UOSO2\/b44CnVyUXp0dv8PAAD\/\/8AGPcMAAAAGSURBVAMAknmGfM5hE0QAAAAASUVORK5CYII=","height":132,"width":219}}
%---
