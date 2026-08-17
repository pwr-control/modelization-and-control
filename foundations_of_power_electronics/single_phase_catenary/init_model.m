clear;
[model, options] = init_environment('single_phase_catenary');

CTRPIFF_CLIP_RELEASE = 0.001;
s = tf('s');
%[text] ### Global timing
% simulation length
simlength = 1.25;

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

t_measure = simlength;
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

use_single_phase_inverter_based_FHT = 1;
use_single_phase_inverter_based_SOGI = 0;
use_single_phase_inverter_based_PHSH = 0;
use_single_phase_inverter_based_SOGI_ccaller = 0;
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
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

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

%[text] ### 

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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANEAAAB+CAYAAAC6cPTUAAAQAElEQVR4AeydB5QURRPHaxAVBXPEeCAqoJKjqAQDCEgUBBQkSEYlyGECjqQgAiYkKYgkJShByQooJgSJfpIkShBQEERR0fvu19r7hmV32d3Z25nd63uvbmZnOtbMf6q6uro6W7r5MxwwHHDEgWxi\/gwHDAccccCAyBH7TGbDAREDIvMWGA445IABkUMGmuwJw4FMa6gBUaax1hScVThgQJRVnrTpZ6ZxwIAo01hrCs4qHDAgyipP2vQz0zhgQJRprDUFR8eBxMtlQJR4z8y02GMcMCDy2AMxzUk8DhgQJd4zMy32GAcMiFx4IGvXrpXSpUvL7bffLnfccYcUL15cGjRoILt37w67NcOHD5eFCxeGlZ50+fPnP6G+3r17y6+\/\/ipHjx6Vp59+Wr777ruwyjKJTuaA50DEg33vvfekbt266gXjJeOca9w7uQuJeaVIkSIyZ84c+eSTT2TZsmVy3XXXqd+Z1ZtixYqp8qlv6dKlqpr+\/fvLmWeeKc8995wUKFBAXQv7ny1henq6\/P3337YrWevUMyDiQXz66afSpEkT2bt3r3qwU6dOFejFF19UX82HHnpIZs6cmXRP6M8\/\/5TDhw\/LpZdeKn\/99Ze88MILSlIhrTjnGjRgwAApWbKk3H333fLNN98oPsC38ePHqw\/ObbfdJlrCqJtB\/p111lnStGlT+fbbb2XTpk3SunVrQToeOXJEOnfuLGXKlJFSpUoJfKderj\/22GPqGs+gZcuWKj3SsF27dnLffffJF198IYsWLVLtuPXWW6V69eqybt062bdvn\/og9uzZU2hfgwyJ+\/bbb0ulSpVUeR9\/\/HGQVibOZc+AiAd12mmnyZQpU4QHc+ONN6qXihcrT548ClzTpk2T3LlzJw53Q7T0yy+\/lEKFCknevHnlpptukv3796sXcO7cubJz505BWkA\/\/\/yzIIW5\/sMPPyjJxW9AR\/GkQRX76KOPhI9Qzpw5ZcaMGdwKSRdeeKFcfPHFAt91wq+++kry5cunAEF9GzZskD179sibb76pJBX3kV4HDhzQWYT2TZo0SfVl\/vz5AqA\/\/\/xzBVLKIOEvv\/wiJUqUUO274oorhHIXLFggAwcOVG09fvw4yRKWPAOic889V319N27cqF6o33\/\/XSZOnKj0dY78BmR8iROW27aG87Vfs2aNbNmyRUkDvtI9evSQzz77TKpWrSqnn366orvuuku+\/vprWbJkibqOFDnnnHPUV53ieLF5ifnooBIOHTpUVq5cya2IqWLFimrcNHnyZKUJAM4ff\/xReCZIF8uy5Morr5RbbrnFV3aFChWE9vD8UlNTVdohQ4bIhAkTBI2ChHwIkaqWZUm2bNlU23mWAPnYsWMCSQL\/eQZEfI0Q+Q8\/\/LBSBZo3by6IetQKXpTBgwcLqksC8zpo03mhypcvr14mPhYASCdGnbL\/1tf1EcMALy9ghHjx09LS9O2gRyQIUghppBONHj1aRowYIQCyU6dOSkLCc8Y7vPyk4xz1k3M7YRRp27atIKXq168vrVq1st8+4TxUf05ImCA\/PAOigwcPKv35ww8\/VCodDwr9vlatWkrP56vGA0oQvkbUTF5U1LIcOXIoaYw1DfBAs2fPFqQAEphzrsEr1Dcq4SOD+gQgACAfItQ67kGBiHTDhg2TggULKpVZp0HNqlatmmD0ABQAEsmHCkb7aOf27dsFCarz6ONPP\/2kJBJjISSMbp++n8xHz4CIB4RKwEPjZcJqlD17dsV7vtT8Jo26kAT\/Vq1aJffee68aB5UtW1apcd26dZN69eoJLyHqHXT11VdLlSpVpEaNGuo6gEJK67Eh6h4A44hqRV7O\/VmEIULXR7nwGSOCPR119+3bV6lbjFkYs73zzjuCIQFglskwOGAOv+yyy+zZ1DljqQsuuECQqI0aNVIAXb16tVLvVIIk\/ucZECUxj0\/qGmMKVFReTEzOGBkYkPPFR9VBPeM+xDnXeOl79eqlxkfvv\/++vP766wJYuPfoo48KaSGd3l4p6davX68G9tS3YsUKYfyVK1cugVDhaBPjFsZkSB3A8tprr0m\/fv0ESQfAKf\/ll19W4xrUwDZt2ghEXbQPSyJl0z6ADhABLGNaxkWkw+JHezinTuqmDfxOVPIUiNDpeVGwAH3\/\/ffCkYfJNe4lKpMTvd1M1PIcypUrJ4x3MHNrSZjofYtF+z0DIiw8fFFRSe688071peTIg+Ma90gTi06bMiLjwM033yyMVZFSSDKseJGVkNypPQOiM844Q3hY6PeBiHukSdLHYbqVwBzwDIgwuSJt7r\/\/fmXi1u4+uPwwQ\/\/SSy8pP68E5rVpepJywDMguuSSSwRL0Lx585RbC35lDLwhZujhP4NojoYMB7zEAc+ASDNFz3cAKn0NkypzG9zT18zRcMArHPAciM477zz5448\/1Mw5E37QuHHjhMlXY1jwymtj2mHngOdAxKTq888\/L7gBNWjQQE0+4njZp08fYS7C3vj4n5saDQdO5oBnQIS6hmsPTcRzgck9zKmYVZnwYzacezoN54YMB7zAAc+ACBefWbNmqRlwZsbtC\/A4x4uZ2XHSeIFxpg2GA5oDngERljd8tLp37y44WjKhx1ob6J577lG+ZTikkkY33hwNB7zAAc+ASDOD9Sq4+bCGBlcfCN8t3E60\/5VOa46GA17ggOdAFBOmmEIMB+LIAQOiODLbVJWcHDAgSs7nanoVRw54EkREiWEhGgu8WCLOUmMmXePIF1OV4UDYHPAciJgvYqFWx44dVVAM4qERkonYaNwLu2cmoeFAnDjgKogC9RH\/OLwWCJbBfZaGY+5myTj3uGbIcMBLHPAciPCPY+KVeGeaUQTMIEhJNG4\/xGUgWCABPViqPHbs2CwdrVPz1BxjxwHPgQigsPyYsE+s1ye4RocOHYQQTgAs0q4ToIO1\/iyxYHkFYyzmoCItx6Q3HAjGAc+BiIayipWXntC0RNFksrVw4cLcipgADAE48L0DhKyaXb58ecTlmAyJwYHj+7cJdGTxW7J\/aLO4NNozICKMLl7brGiFsMwRc46VrYyJuEeaUFwBbMRlQHUjiihWvs2bN8u2bdtU3GfUOUL0bt261VcMbkWG8qpwxonIh9qFr5C2JS+VYZXOkfernCU72uVRBIA2zR9\/Ur98Dz6GJ+GAKIbVBS+KeGmvvvqqEMKWGGstWrRQK11Z7Yp6BygI0xSsBAIakp+QTOyyQDTOV155RYUkBkxINtQ5ArczvtLlHKr1phiKjAdnNxkrZzQafQLfzs649sBz04SgjacixqWnSnPC\/alj5SNoQDtZ2LqYfFjzfFlYfKuiQTfska5Fz5TKpW6WwhWriVV\/oFitJ4r1whbJ\/dLGk9rDh0I\/+1gdPQMirHCsZuXIcgckEHHYIIIBEnXT\/vL7MwAplZKSItdff726hUpIVE4W+AE+rc4BVh0Sl4TnT28hPw+uGBWteDQlqny6Pif5neSlfif516XdLnNaXOvr+6pny0rjslfLtz+JFH91m9SesFcmb7SEjQgC0VVXXRX0nk5\/8fYlkmt2bzlzbHNJf\/4ORactGirZt6+QMy\/PK5e0HyNXpC2SvFPSJe\/InZK3\/xeSt9tUyVPvCclzV8Og5fPMY02ug4jdDdh+A+KciD6ABTVMd5ag6lwDYPqa\/\/GGG24QFvMRCJBx0AMPPCCsS8I7HCdWxkKE30KVS09P98\/uym9emGgrdpKXOmOZ\/5oLc0i3yikys31RAVANS+aWAfO2yoWdF0mRvl9knG+jSh\/Z69Zjl91pFWVLPctHByf3kr8yxjenX5LyL1AywHLN61sFAiznVGgqOW6q4CvTzRNXQERQdOKYMX4hGiYRfiCWPHAkRjTqGDvJQQ0bNlRbqyBFNLMAXfny5ZXO+8QTT+jLQjTRZs2aqchAqampgrmcVbKA6Z9\/\/hHIH0QAKxpixW00+ZI5z9+\/7JH6N6QLkg66N1+ODBBtlZvTPpUtA+6XLU+WlS2trvaBZf\/E7nJk53o5dtZFSgVDDYP+7vqx\/PHwaPm1ag+JJb98L0oMT+IOIhbYsS4IyTJ9+nS1SxwrWCEAQAha1LqiRYuqrVWIr0DEHzaFsvebZRFLlixRW5MwDgIYY8aMUYv66tSpo\/bLueiii9ScEFIKowPWPrYfsatzlJknT56g4j\/UvXDUklD5ndTtJC9tyqz8djXstIGVpOW0YrJ8ZyWZva+JUsVm7T1PelkPypRHtv4rYeyqWJjPwUnbyRuCoroVdxAxD\/Tss88KW6hgcvZvNa49GAEwADz44INKAhF37lTWOaQCA1a8HZByjKHWr1+v4kaHUue6puxXplCsOZFS+uSuUeelrlwZOj\/HaMhJXupjrIEKFQ1hAQMgdvVLn9vVMKV6ZahhjFs4hx5\/a64UrNNGJn29x6fuSYR\/+iMQYbZMSx53EDGuQbXCEAAwMGfbCeCwcxy7s7HV5KhRo1QwRz3Xoznhr84habZv366iAuEiRHQgJmgxLIRS5y4\/47hSJ1ApIqU\/9m6JOm+kdcU6PepTtHS8SE1B3ULt8ieun0oNQ917\/8HLZdbDVwnqHmMnCJUvlqpboLL0+xPLY9xBpBuPtQxgMN7BjM1ub3huE3cbMyQ7EqCy4UPHFiCMZQ4fPqyzq3117Ooc9y3LErY2xGWIcRd7+bALWyh1rsvG3P9adrDuREg5HpsWdV6sSZhiOUZDTvJSH6ZgBuhRUauX1HNAIkRDZCZfucLXy\/MNiiorHwYJLHxY9yCse18eOCugmq3zU0akRN5Yk2sgYl5n165d0rhxY8GMzbJwNtclnDB78GAQ0J1lgpS0+negIyZtAIcU4j7jHrb6II5dKHWOtIG+WOFcQ4UMJ12wNE7yO8lLe9zMH6huu0GidamMeaB1e6T9pO+UQeKpd1bKZ6s3+QwMgfLTp3CI5x1rcg1EvOxICczXulOoeKwbYl4IAwMSCULFK1asmCC9dFp\/dQ4G6u1YMCBgyWPshUQKpc5RXqRfM53eqWEBy6MuK9Kjk7zU5Wb+U9WNdFrQpaySUEinOZuPyX1jf\/DNP117U8mAEop+nYp43rEm10CEUYEJVSQRTGVcxNwOgAFETz75pLK8IUWIPceaIlQ2zQBUPbs6hyUuZ86cwpF77PrGnBNjo1DqnC7PHL3JAeafVmVM5kLMP2GQYO4JGjDvxPknt3qQza2KqZfxD2bprl27qrDBb731lvrCsPSBOSOkDYSVjbkgPBDIF4jYGvHyyy9XVj\/M2FOmTBGOWOsOHTokrEWCGFdhDreXgRSLhpyoFdSH6Z5jNOQkL\/W5mT+aurW6h0FiVOXTlEFCT+hikEDlo1+nIvtzj9W5qyB64403BHAMHz5ceLnZWqVXr14CaNhOke1UIOaKmJRFRdMdB1z2yVZUQSQVrj\/s4oa3ApZA8mDtY0lF5cqVhXPcgHQ5HJn0jYYw00eTT+dxkt9JXup3M7\/Turu2bSrvhJVepAAAEABJREFUPl1XcNnKtfQFObBuiYxYdkhKDlgu9C0U8bxjTa6BCAMCG+PiqnP++ecrCcQ8D2pez549lQsPS8I1oerZjQ2obHZ1DrUPfRgnU9Q\/FuIx5kI6sXkw+5BiBseAUaBAAdF\/5DO0xac6JxovdiybI3unPqvcjbrWLhlWP\/Szj9XRNRAx4Me4cPbZZ6u+oKqNHDlSueWwuS4eCoyTNDGnhLRRiQP8Ayjc14YKFvTh+YC0WrlypU+d47xIkSIBSjCXYs6BOBao\/ffiWKWvKtdAhErFS470wUsBECE5AAPLIXADshNzSaT3tdzvBONB06ZNBaMEJvLx48cL80tlypQRgKTVuYoVKwqWPr\/s5qfhQNQccA1ElmUJ80IM\/jFB40\/HfBFGBlQ91gYROljTgAED1LgpWE8tyxIseAAPte3dd99VKqJlWcp1SKtzrE2yLCtYMea64UDEHIg7iFDjULswDGAtq1KliuDig+RAinDv8ccfF6QTSxc0lSpVSnAXiriHJoPhQCZzIO4gQspghWPJQ\/Xq1YUxT926dQXC0RQJhNGgXbt2Uq1aNR+hhuG8msn8MMUbDkTMgbiDiHEN4xs8tQEN1jTmDSA2OGZehzTMFUXcG5MhKg6YTM44EHcQ6eaiyqGyARh97bLLLlMOpKwrwhqHCoekgviNqqfTBjouWrRISbbSpUsrowJ1MLGKuZuyCFSCIQOVMlB+c81wIBoOuAYiHENZpkDIYPzlICZVcRzF2wA\/uGXLlgmGAgjpZQecf2dxUmVsNXHiRLW6lVgL\/DZx5\/w5ZX7HmgOugQi1jYlWLHNIGXzdcKNJTU0VovRoq5w+4smAysfyhkBMICYDIMNbwbIsQeqgEmKpQzLhvMpEbsmSJcXEnQvEQXMtWg7EHUSocETzocG45vhvcAwQkFAAqmzZsipeHFKGfOzX2rdvX7KeRNdee62wjghCXcObAWBt3LhR8LbWGTCpb9u2Tf80Rw9x4ODkXqKJ1beZRbHuctxBhOsOYGAT40AbHBNvDKMDEoN5n1q1agkeDHhjE7wE614gJuDygwc4Uo0JVoCaP39+tTwcqRcoD9dYapEgpIKyJEpb7QEVCaqol48HO35e83T5bkJfH22aP16WznonU4jnHkuKO4iY62nZsqV0795dZs+eLZiu9YuB5\/ayjHEQvm2AQXcUyUKAE1Q\/fc3\/yD1UOFQ+QmYx\/wRg8+XLJywz14YFxlss3rPnj9ZfDMBHm5d8TvI7yeu0bv\/8q0f3kK+6lJelTa5VARUJrPhUyZxSLf\/Fctt9DRQRJ45YCxBLyjna6dYZf4k\/1Z77uwSiIsP+F\/B6oLT+11jJbH\/2sTiPO4h0o3EEZZzDC89DgRi\/4LmA2w4gYk0RYyJCYJUoUUIACF7Zugz7ETUOScUKWFQ\/PBa05\/eqVatk2rRpAoAIYMI6I3tecx4+B3ScuPQRjXxhrw5mqGHBYsQBHog4ceHXklgpXQNRKDY1b95cMEvjhIrHAsu869evr9x6WK0aKC8+d0zekq5ChQrCb6QRSyxwOCWWAxO6nHMtUBnxvoYKGm2dTvJSZ6j8eiyyO62iQHYVDMD8\/u1iOefq\/P+GvJqSrgIqXpG2SAALZZ+KQtV9qrzcd5qfMmJJngQRVjVM34sXLxaWNWCqbtq0qeo36qA68ftnWZbgF4c0Y6yFlY+0mMoZW2nfOYBEVCCdHdXD\/pJEcp6emtf3NY4kn5fTLm2YW46sXqiiGOloQKhfmojmA8U6qOKpFtPF6r5+7rE8eg5EjH1YiIcVDSMC8z0s+2YcheUuVOcDTbaSnpgLekwEMNNtYYTvWpHH90W16+jhnPNinZwuPezynOSPNi9BE6FRdb+RQXX+56OOFb4RqMNN70mJXG9LiT+fkxI\/tldE9B3\/+NrwFYkQDTnJS31O8pM31uQqiPBAYNzD+AcJQpgsrjGfg8TAxE0UH3ztePnxQAjGAMzgSCz\/yVakER4QWPxwMUJKAcpg5cTzun4hoqkz2rzELIAIBjK0YQHRRBxtiFgGBLy3U7fKeUTHNyA+HPENoq2fvjrJG4v8lBFLcg1EWNNY3oDEoUPM5TCRSrDG3377TVhchyMqqhfLvI8ePao8u0kbiJgTQlL5T7YidbJnz65icpOPsqiHc0PhcaBb5RQVsB6AQQBKg2nHz8fCKySJU7kGIlQ1YiKwHh5XH1a5orKxMA\/H1GHDhgmm6NGjR0uFDEMBgApmmeP5BJtsBXwYE\/SiPNQ6r4AIPZ+2R0NO8lJftPn1ClKil15zQQ6p8fpKWbr5EEWGTdHWrStwml+XE6ujayBiWQNSAamjO4MqB7hYiUqQeuaPZsyYIXgfEHcBiaLTsh6JdKQh2AkqQrDJVkzd2rCAM6suQx95KNEQXhXR5NN5nOR3kpf6neb\/5\/Beebnq+SrqDkCizHDJad1O8utnHsujayBi3IMJu3379oJrDqtcicbDrnZ4KfCyQ0iqmjVrCiZqQKY7j1RichWQATjUQ+4h1RgH4XiKxAk62Uri\/wgARkOooNHk03mIt6fPIz06yUtdscrP2KpbxpgJ48Pjsw+p1cSUH4piVXeoOoLd+++Rx\/TgGojoBSrWBx98IEOGDBEW6V1zzTUCODBns2QBTwNNLNbjHvkWLlyolnxjLOA3hFmcOSTGWdwHTKhy5FllJlthUaZRt4wxE2OlHQePCWMlrwRVzLQO+xXsKogwSaOmEdaKsQ+SBsmDuMZih5oGAFg2gdEA9Y\/2E+UU0DEO4jeEBwSAQaLhSsSOefjOMbHKmAhJhsWPc66Rx21C\/Ym2DU7yUmes8zNWAkjdMqTSZ5sPKjABKCx5xNS2AyvWddMfN8k1ELF7A645qHNY4AAPEgRTdYcOHdSaINyCME+j9mHm1i8\/G4DhZ0fEIM08gAWAKBcwsj6JyddTTbaSn3FVNESbo8mn8zjJ7yQv9WdW\/hFtK8m6QXWFwIrQvs8ny5Q5S9RueYAKQvXjaKeL20wRf7q0yXAJRKV7zgt4PVBa\/2u\/3pbKI48puQYiljsgLVjNumzZMmHND8YG\/NoY32C540hvDx06JGzcRcwFDApc8yd873AVYi4I4CDF9FgplBc3+QxlXvDG3bOHyL6326jg9Pa5J\/\/zA8PrCcEX7VTv3vKSGeT\/7jj97RqIUNFY3kBkHzwKUlJSlLc11jnGPzih9unTR1jWgJGAFx0HVEzdgTrNfQDJuMqyLMFhlVh2qIqASufhHC9x\/dscvcOBbhljKzvpieBYH2PdY9dAhHRggR1jFRxMARQxuTds2KAsPCwHR5IUKVJExV1ARWNJRDAG4HBKGiZpSeNWBFTUSPqEujR\/\/nyaYijJOeAaiOAr6lvhwoXVdiiod0gNxj0EuifoOcYDTYyJkFLkC0ReiYA6d+5cYcEhkYuYmwrlqhSoH+Za4nHANRAx3kES4WHNZCjeCljlODK5SnASOyGZsNBpFjOewtP7lltuUZcsyxsRUJGGzFGgruKjx46AqoHmX9JywDUQ8XIhWZjr6dKli+DmM2HCBMHtx7ISM8wvHwZUTlyYeGPwxsBayLmh5OVArEDkiEO48zCWYa7nwIEDArgcFZgJmRlj9ejRw1cy3hBM7BILAs8JzPPMY2nrIgnxysD0zrmh5OWAayBijocxEGGzUH8YB+G5wMCcsZJXWA6o2SOJ+Swki24XYx\/UT5ZwoGrSfiYRmfDdtGmT+hCQnohGOo85JicHXAORZVnCWqKnnnpKzRERH4FxzsCBA4UvuFfYjXTBXF67du0TmsS4rWrVqirIPhKUHS2wLOL\/h2pKHD3M8\/TphIzmR9JxwDUQwUkmVHnx+KqzohULG86oBBrhvheIoI94RxQvXtzXHNrKmA5pykXLslQ4KzwvABSSiWg85OO+oeTmgGsgAiioSIMHD1ZxFIilAOG9wHjD62xHQjH+8Xo7TftizoGTCnQNRMyfoOowlmDMoYmIqF4fR2C6pu2M3+Aoq2cZD7GIkN+GshYHXAMR4x4MCMyrJBrLLctSQScZFyE1d+3apZazs3Yp0fpi2uucA3EHER4JmIbxiwNALLhr3bq1EKQR4h5pnHctc0sgph3jJczbjRo1khYtWgiuR5lbqyndixyIO4hQhQjO2LFjR8FjgfkVAMVviHuk8Rqz8KrAl0+3i9WzqJ5ED8KzgjVM+p45Zi0OxB1EDMjxk2PiEr84DAuWZQne2RCuPaTJWo\/B9Db2HIhfiXEHEVY5JA8x5Vj\/gxEBowIuM\/HrtqnJcCB2HIg7iLDKoa6xVDslJUUeeeQR4VoijINix3ZTUjJxIO4ggnkEEeGoCfcYI4k0N8wx0TjgCogSjUmmvYYDoTjgCojYk6hQoULKVQb\/MoKRcCSABgEZg8VRCNURcy9BOZAEzY47iLDAMUlJTIRAxD3SJAFvTReyCAfiDqIswlfTzSzEAQOiLPSwTVczhwMGRJnD14ClEguPXSmIMa6pX79+AdMm0kW8NsaMGSMsEcEFCi8Ue\/vpN5Gc7Nf0OY67bOqGD6K+lmhHA6I4PzEmlnET0vTMM8\/4WoA3ODEafBc8dBKsKQBn+vTpEu3aKVY1s6QE41KwOrx+3YDIA0+Ir3S7du2EyEcEsmRnDBxzcW4lJjlfa5pJ7HKCWiLFOnfuLGlpaYIlk68\/R9IgBfjyA8jx48cLaSmnd+\/eSlJwnzKbNGmiVhQTXYnJbiQB8f+IRMsCRFbnAo5OnTqJnsNDYkDUo4nYE3jksxhRXwt2ROrSHohQ0LSBTQfYJYIQY7iDBcvr5esGRHF+Oq1atVKmfcz5xNxbu3atagHBWSZNmqQ2NmPbzaFDhwpx63iJ8S\/cvHmzsJ3m5MmT1X5NvLRIAZU5wD\/yslMGK2yJHIuXCNvQkBTveSQiaZACy5cvF1YXEwd98eLFAtBQ0QiysnfvXtmzZ4\/yKmFzAYJSUoYm0hcsWFD\/VEd7H+knAOEGUhcJTD\/xVmFRJu2iL\/SfYDWkSzQyIIrzExs5cqRo0\/7q1atFx81jN0C+6Nu2bRNeNL7WvIBEU12zZo0gnQiTzAtnWZbgVc7CwGDNBwS8rCwUZEtPQLly5UqVHCnA3kp4olM\/KiQBVwAIa7xY4sEYB6nHkv1Vq1apPaSoz3\/NFFKM66rg\/\/7Z+0hf69Sp898dUVuGslEB2+owPuQGQCbMWKKGFzMg4il6iFBpCMMFcHgBiRw0c+ZM4UVFRdNNJewyKpj+7X9ETUpNTfUBFqmUlqH++afTvymL0M76N5sI4CzMjhzM3aFK4jQMyHSaSI+0f9y4cYKnfrNmzcSyEjO+oH+\/DYj8OeLybwbaOOMCHl46dsMYNGiQWvCHusUqWq4jrQAcMfuQKAS9TE9PF8YodIGvPNIFSQEY2AcKtY57gQjgAhbARCgwwjYTjIVY6Iy3GKshHf3zEvYslFppT4\/KSH9Q42izvockpE9EjdXXEuloQOSxp4W61rVrV94qmIkAAAGvSURBVME4UK5cOSFyUJs2bQS1CgMCqhGuUYAJ6YTqxW9CdGEhY+wEsFD3AAZHVEXi4XEerLtsW2NZljDIr1+\/vuBlj8rHi61dtPjtn586GNcBAv979t+AEym0Y8cOwbABIDkyPmPcRV+QUPY8iXJuQBTHJ8XK2EAvMiCBdFMYi2AQQJJMnDhRiGlnWZYQ+45oSEgh1CFeXMuyhNXAjIEWLFggo0aNkv79+6t4eEgTrkOodnz9qZ926Lqol2u8xATSxNRMvdRlWZayzPGS16hRI6D6hSUPiYX0ogzaS3m6fI7UR1u4x7iM9kN8IAAOfSWGnxNVkXrcIgMitzifAPWixgEmJBsBLAM1GeA0btxYGDMB6kBpQl3DfM9SGKRpqHRevpfNy40zbQvOAaxqWLmCp3B+B0fgWbNmKckWSkpg7WPOx7IiNxQwBkR1RUo6b7E7JRgQucJ3U2kyccCAKJmepumLKxwwIHKF7abSZOKAAVEyPU3TF1c4YEDkCttNpcnEAQOiUE\/T3DMcCIMDBkRhMMkkMRwIxQEDolDcMfcMB8LggAFRGEwySQwHQnHg\/wAAAP\/\/n3QRkwAAAAZJREFUAwCFy6yoK6CM+QAAAABJRU5ErkJggg==","height":126,"width":209}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANEAAAB+CAYAAAC6cPTUAAAQAElEQVR4AezdB5QlRdUH8Jr5DGvOEcOiYA4cRcG8ZjGuYgSPrhHMIiDgOQgIiCgeRAyooARBQVAMgApIBhEBBREJKgiCihEUMfLNr5c71vTU6+735r2ZWfbt2f909a1b6Xb9K3V1vcnNN9\/8+jXXXPP6Jz\/5ydf\/9re\/vf7Tn\/70jPvrb\/h39NFHV\/JHPOIR159zzjk3SGdeyPmLj7643ZOLWxqvetWrrj\/ttNOuJw+9mbH8704cdIAbuIH7f5rXXy8tcmlIS5qRhjKF7jXXXHO9PNB1dc\/ffYSlm4fP04p0IixdiDikKaww4gRuOoGII9KLsHEfesIJH3GGvH4NPboliF8YdpEGHXkgC8izdPjRz+PkDj1XYemJS5x52Lou\/Trq4fmHTB7ERwYhD3tLT7rS50cnIN\/kIB\/iEZ97fqHnmYuP3NV9+OXXUviQyYO80J9ca621Uv7v4osvrm4vu+yytP7666f73e9+Fd785jdX8qkE01Tgyt32R9z0TznllDSVeBLnPe95z3SrW92qGHSvvfaq0qqnWVe+zW1uk+52t7vNEEtrhiC7yf1ufetbJ3nIvEfq7JLXudr8Gc94RvrFL35R4SUvecms8nz4wx9ObOu5\/fnPf678c5sQsOftb397zhT5cdMl\/\/S64ne\/+10644wzKvXHPOYx6a53vWvlfs5znlNdo75UNy1\/Ikyo1csUctfcr1cdOOaYY2bUvxe96EVJfoQPqMtked4nw3MU18i4hwLSqBecDLbYYovkYXO\/973vrSrE17\/+9eQhkkHEwb3YsVB53W233SrbIVVuvxNPPDH9\/e9\/72y2UeU\/GlMZ+epXvzpdaaORJpfXv\/71r5zzBo1M5EFDxH7f\/\/73073vfe\/pPMiTvKmTr33ta6flk72MpRdSYJHVoeWbjqHBMdWVVpnQ8kTiWrx6kHPPPTdNdb+V+LOf\/WzadNNNK7eWE+urm6k\/QUoyflOi6f+9yjGtMGTHr3\/963Tttdf2jHWQvA5ic42PnvspT3lK0srnGZoarqSHPvSh0yKtfqm3ocCeeS81SP7F0wSV8POf\/3yTSuV33nnnpV\/+8peVu+lP\/ZnHvUpeqme94vr973+fvvSlL1XeGnANkZvcJu6RCtiUbclgUmvAEYiegjKEvOlhhU79estb3jKtscYa1TBOXB5gW+HCECWDBymls+OOO05XGt1wvRx0RoGoXIamGhlpqLzxENxDP3mdi83z\/NRtwOYgP4awS5cuTVNzALeJLru5YetddtmlGrqogE94whNSP\/kXRxcgBoLQjdY+b6BVYH4aScMm7iZMzXOSBpiOq3vuHvWMVyui\/lHcb7\/9Kptww7e\/\/W2XNDUfSoaE1c3Un1nDOb2MAk75Jd2bVg4YncxD0KJxt0FCHl7o5ePIkLliNXZzG9JJz0OMCkAO0pU+t0qs5aYrn2Sjgvg32mijpLJFpZUWufTlQ37IAvK67bbbVrf86NAVphJmf+Zi81e\/+tXV3FV0YTvpQKRlSBIVNNfnTy+39Vve8pb08Ic\/vJqrDNvWiIEg8hoNB3cAeZHYvUZJ48TdC+Iyb1EGV\/d02Z39ubvgzne+c1I36arn4gNuMvDsjajkTz7JApNh3BC46s4Mq7gDAhtjx1Ar5G3X3Fh5BczDIdvHPvaxaugXckRGIhWAzDBAQaRfzxvdUjmEGxQq0jOf+cxZwVX4Uvp1mYAlXfkE\/jkGtTnbHXTQQamUvvildcIJJ1SkcB\/6bOY+hzjYN2TcZHHvKpw4ufuBZ2dIL4xnirjcOfLGVMMTPX2uk7vlQ35CFnWU3UPW9br99ttPN0bCaPTUP1f3Bx54YDWiUifUDbLAJEPpUnND85QR8sCPf\/zjqoXi1wsipyeM8PRc3YO0yEp6Wg55oAcqVS5TUVQA4fM4QzfvhumU0iAHcQsXccqXe+lLkw6EHr\/QJS+lHzLllzY9CLk4QFrAXU+vrluPS3y9UA8rfpBWKUxeNnogjrouGb+AcP3YOuLz7NhQPPVyl3ToSVt63MKKI3TjGv506vbyHMj4iSvCuEa4iFfc3HSBW11wdX\/22WdXCzbCCZ9j1nAu91xsbuPeRz7ykdWKTgyv5JE8FiZ0ywpPPsbgFmDTsa272W8gEv3rX\/9Kn\/vc56pxpLHj05\/+9HTcccelqRdP3VIdUEvLojsVXFdrSCD9GA\/rzvOlR3pjDGaBhbP1YPldyFADkeiQQw5Ju+++e9p4443TUUcdldZbb730tre9LZ188slp1P90p7rXEnTdHv6o87C6xL8YbW2UYTjo+fcaqs7385nsN8E\/\/vGPySTr5S9\/eXrXu96VHvjAByarIY997GPToYcemv7973\/3G+XI9b3\/KOGSSy5JlxTwox\/9KP2ohlwv4hp5xscJrBIW6JtEP\/\/5z5MXjSZqExMTVSFvcYtbpHXXXTedddZZ0+9uKo\/aH6t7TbAu3wV77LFH6oUddtgh1dFLt1dapTzmuhGfdMLN\/\/jjj0\/HT+Hwww9P++67b\/JeIchYM8X49kZkgb5JhEA3uclN0h3veMcZZnjQgx6Urr766vSHP\/xhhjy\/iQrV65q39k3u6AlK1zy9+XBHHuTXMAMMKy+99NLEjWTI9tSnPjXBk570pPS4xz0uPepRj6rwkIc8pFooMbcbY+U+zVHbYdj1om8S\/eY3v6ne1npBNezM5PF569wF3sKXsM4666R1CrA9Jof53JIlS9LPfvazTkCWAFsg0XXXXZdnfdot3qVLl1b5WLZsWfUewruy29\/+9ul2t7tdBTs47nvf+1bzSgsk73\/\/+xN40Wfcn+PYY4+tlllz2YK5b9jwuqqlP\/1whujom0RzSduQZ7vttktdYL7VBVbjArZjhFuFzIEsKqsKf\/rppydDNj3ErrvuWg27EKIXoswqvx4XCV\/xilekTTbZJEkv3F78SZM\/8kS4uAapvMADOuLkL209NBvBZpttVvVcL3vZy5I8Gh5ecMEFVEcKW3MGSaBruDa9Jv+SXxdZSWeQMvYK0zeJLCPbDfyXv\/xlVpyGef\/3f\/83Sz5fAm+8paVCmo+ACqhC2sLi6p5chaWb4+53v3uC5zznOWmrrbaqYBXSFhSwN4t8xYoViU7e0wlnZZBs2bJlFbk0FhqCEqmWTPV+SER\/\/fXXT9xBqDxPNkfK73777VeRyZAwyoJY\/EplyePoxx027CcM3a7h2vSa\/Et+XWQlHXkeFvomkdb8H\/\/4RzKMyTNhOGRoojLl8lG6kQVUJORAkqhk7oFfqZLJ5zrrrJOQATFysrgnBzpzKQNiBKkQSs+FVHmcdUItX748LZ\/COlP5y\/XCrczKFMTKy41YwD\/0x9fRWqBvElnSvsc97pHsg4qXq\/ZFnXrqqcky9x3ucIeR5FjFQQhADhVHiwzuyXtVHIRBCOTIycJNxq9XhR1mYRBq6dQcCanyXipPA6E0UECXnh5QXg0VkbBXXtkIsYB9okHhRizoZaM8D2N3fxbom0R3uctd0oYbbpj22WeftPfee1db0bfZZpvqa8iXvvSlaWJi5bJ3f9n4n7aK4EEjBXKoAFEZ3AM\/Ov8LtdJlsUMFQwrkUPHspDAUc0\/OH6lWhlj5t+uYuU2vyb\/k96c\/\/Sk1EQqRTjjhhHTwwQdXK31ItWLFiupFd16uJmKFPREL6vYMYtFjjVI+ydvQNVybXpN\/ya+LrKTTVp5+\/Psmkcg9SA\/DDt\/ly5cn744+\/vGPp4c97GG8O8FDQwSEQAzxBVm4yfjRKUWICEgBCIIwPgVwdU+OMKWwi1Gml1o2NZd6zWteU73Erg\/5EMrS+Q5T78Esmx8\/9T4qbLBixUxisQFiQS8bsD\/7BrH06OxvZzTbB7kWo60WW54GItFNb3rT9KY3van6Vh6BjjzyyGr5NjX889DudKc7JQTxsDw0bg\/Mw2wii4qAFMihguStMBk\/OoNOILuGa9Nr8i\/59ZIFoQzlEKFEKD1UTqgwPWKxBWJB3V7i4x\/69Wu+kOH5xLPyvBALSs+qVJZ63O7b9Jr8S35dZCUdeRkWiiSyufO5z31uuv\/975\/WXnvt9MpXvjLlXeIgG1A9XNvNSw9AYfh7uAiBGPnD5ybjR4f+6gJDuGVTPZRFCbuq6+XXQwWh9Cp6qJJt2Jf96sQy1EUsqMcd8WgAQfxQJ5eGsBe5Io4b83UWiXw3YTPpve51r\/TNb34zfeYzn0keFNlVV11V2WLQDaj21XmY4IEiBoJ4kMBNxq\/XA60y0ONPTvQeKkVx13Btek3+Jb8ustDROyGRxQWE0jshWF4gL4GDUMjUq8GKMJ4DIBawv48f41kgFvR6FogFRhJdyRXliTzUr03+Jb8uspJOPd253M8gkdU2HyHd5z73qd5J2JLytKc9Le28887p8ssvTz7vncsGVAc\/eECQk8WDTEP4N2i33TVcm16Tf8kvk02Xvi6r31NEKL2TCt6LUMjkhXLMn9oIJV6QnueBOIgFyBVDaG7pAh1h6kAsKJHrfe97X1W3oueil4eXfn6fu0t+XWQlnTzeubpnkOhvf\/tbtbmUcfKlap\/zuldgcyD75wbZgKonmmuGx+FnWqBEKLLQMoqoE0qPFf79XEdBLnOtmHfFsBD5upK+n\/yPSncGiRyq+IlPfCJtvvnmM9LzvbvlWAsKCGRnwiAbUEWqax3jl9Ucs187GA00hfGMvAx\/4QtfmGxBMvwzD2V3CEIZen30ox9Ndpub\/zbF2cXPDhZ7AQ0x4Z3vfGcyLLRaasRhCAq2TMlHCRpo5JE3ZIp5l21PDk9xBgfQ6ZKnXjqltOcqm0GiiYmJand2bngPbqeddko+d7ADWWH5eyczSOK61lFBfgaJu2u4Nr0m\/5JfF1muY57atXy2IC1fvrxqEGPIl\/dQXpBbMv\/Od76TvvGNbyS7zicmVr7j65pGrpfnM+Q+1Te\/ffe7353ePQUvjZHLcB4MCcHIR\/gSrBbaDWNoCgj2+te\/PhkWeq1Cpqe1DS3SFU+4XfN77mFjBonyyGMFboMNNkhXXHFF9YLvAQ94QK4ydq8iFkAecyhkAr0FWWQ\/76HsHrcoMeiQL+JsuhoWwooVK9KKKZhnxZwreq8gF71SXBpzQz4k0ntFz2V4WH\/XhYilOIYlm4xvNxzOGJHqCn25+qEPfSjpfRj28Y9\/fOW9KmxArTLax59ordqCrLnmmo0qTfGU\/LrISjqNmWjxRJ5lNyyZlwilh9Kyq5ixKNGFUF3z2aSHMNF7Bbn0WEEwQ0Pk0rv16r2QC2kM+5QBuZAq5l3eVdJpMVNf3pPGxOAbFiHNeRzq5+O6Aw44IH3yk5+sTjHlB8bci2UDqvzkQP78vqu7a7g2vSb\/kl8XWUmna7na9EqEMlSPcHkP1Uaorvls0+vlj2D2bSIXMkXvpefiRi7oRS5lQh7lE5f7YWHSwQ9w29veNlnidoqPyLUAep+JiZXjZDJQkIXYgCrtMUZngSCUBYlSD1UilhA1VgAAEABJREFUlOHU6HLULWZzc8RBLkComHdxI5ZFDTpiHMUK8Yw5kW7wpJNOSpavrcQ5xjWHrn7UG1AVdFA0DRWa4uwark2vyb\/k10VW0mkqy1z9pBeEQiboNYcyH4keysS+S9rib9Jr8i\/59ZLpbRAHsSxqIJRhoXeVTekP4jeDRHYkaHHsUlh\/\/fWr\/XD59Ytf\/GKVhowZa85lA2oV0fjPorfAIIRaDD1UL8OOvCeyQ+HMM8\/s+R3\/pjf85Ileqt8NqL0KNUz59Hi6z0i7hmvTa\/Iv+XWRlXT6LF5f6k3p9UMom2NN7K305Rloip9ek3\/Jr4uspCOtYWFGT1SK1BzJyoZhXfjH8reVFKt783UCaqQ\/vi68BeqE8mKXLM+ZVT0rfb0Ileuuyu5GEv3kJz+pfm3gv\/\/974wyDroBdUYkI7gpjY+7JNM1XJtek3\/Jr4uspNOlTIPqDJIe8nixa\/4E9TmUvASh9t9\/\/+rMwHoPRQea0i\/5dZGVdKQ1LPQkka0c3ghbtcsTm8sG1DyesfvGaQGEqr+HWrp06YzCmndHDxULEzMUVrGbniTygtVOBXOfvExz2YCaxzMK96Bj367h2vSa\/Et+XWQlnVHYLuIcNL1SuCCUZebooRxmE2m51gllPx95CaU0ushKOqX4B5UVSXThhRdW3xF5qWUdPo\/cy9i5bEDN4xq7Vx8LBKGe\/exnT3\/+Xuqh7Oczh1qVeqhZJLJosOeee1a\/S+m42\/pjjre+dXLV9XrdaxVGBWkOEnfXcG16Tf4lvy6yXMdm4EHK10+YPL1RhBN\/7DZ32KaXuxYlmnqo2HHuPWU9T+Jrk+U63MPGLBL5mtWQ7a1vfWvS4ww7QZO8MdZMg9ign13cg8S\/EGFit7nXJzHkq\/dQyKOHMsWIHee+bxskv8Ouz+KbsQEVo+2Ve8c73pHyHyymGFjMG1DlP\/LZz7VruDa9Jv+SXxdZSaefsvWrO2h6XcM16Rny2ZuZz6HsOsjLEHMowz0whyLLdepp1O9z3WG4Z2xA1QNJ0HkK3v+Azag+yrNzwU5vhVysG1CHYZBxHIvDAgi1bNmyZN9b9FAOzslzhzx6KGQCy+Zkuc58uGdsQPXZgx3dOXbbbbeqV\/rWt75V\/VrBYt6AqnsfxGhdw7XpNfmX\/LrISjqDlLFrmEHT6xquTa\/kH4R69atfXf0Ygp7Ku6i8TMhj2RyZDPkQyrspOqU4yYeFGXOim9\/85smO7hzeEzmk3kIC92LegDoso4zjWdwWMGfSS7Wdy2fbEVLlhBpFyWaQqGsCi3UDqqFo1zLkel3Dtek1+Zf8ushKOnneh+0eNL2u4dr0mvxLfj7fCULFsC+3SfRQQShzLLJcZ67uVhL5LEI3qXeKxOayAdU8a1Swh2+QuLuGa9Nr8i\/5dZHlOrl7kHJ2CTNoGl3Dtek1+Zf8cplfH3T+AsIcdthhyXwp6qwr8lhOd3U\/LLSSaFgJiWdV+1W1cX5\/0XNH\/2K3zTnnnJMsiRvy5T2U+ZXhoPpYwECieSXRQDkcBxpbYI4WQJx8yDfH6GYFH5NolknGgrEF+rPAmET92WusPbbALAuMSTTLJGPB2AL9WWBMov7sNdZeFBZYXJkYGonOPffcZPnQMuoxxxwzp1L6FN0baXGVsNFGGyWbEiMR25Ho1eXhv5ive+21V5L3HPNVjkibrdm8q53C3vJcCisucv7S6Bpv6A2zLkWccVVv2FfeHK0VddZ9CV3q8tBIFJmcj6ttSc973vOShzUf6Y0ijXiYDh+sx698j3jEI1KXB1gPO+p7Nj\/jjDOmk7Gv0nLytGCRO7ywPe+885JfOqnvxStl3d7RtoZg0ZPI9yb5OwlHwiqsh+c9APeqCEQBeXf0WJTRWW52ypPvuOOOI20ofH4g3frLdGn3Apuzfe7vyN78fjG7vYi95pprkjO78182yZ8Bm2gYbLpWFgeZajy4S5hBIoyrd2ml1jBa0dDVdfc6FC\/vmunrPslKmXFcVxxnXPIne8ITnpCikl188cVEM+BwFWlIC3Td8psr5cMROlAqZ8keZHlc3MKKI8AeTUYXJiqeBwVk4PsaPyXC7c16bldxijvScY3yKePWW2+d1ltvvfToRz+6qiT8A\/Id5Y4wZPzFKW5pNkEaJ554YqXy0Ic+NIEbPVNbeP7SkZ50Iy\/ugQ3FVUKbrvqUP3PxRRnz+ORBXtUf9Sj343Ycgp9WZcNnPetZM6YM\/APyKo3ANIlktDS00J3xiwhkxFAqWlFyLRM9DHcfYCxb2XM5N5mMhJ6TVz\/4wQ8mn2Bce+21Ie77Kk+MJ40ITCZvKgDw15qGf1zpyG\/cK3PJHmT8cj1h496VPZ75zGcmD9d9EwwtDDFyneghbFtBKn7iEqe43QeUT4Vw4CbZySefnPwC+emnn+52GvJdKve0QgeHfMovVXUAuOVJy809Az1uSnlhw9z+EZS8nm+yqD+u6lP+zIVlF\/lTX92DPMor8pd2djsezufrwu6zzz7poosuEqw6i\/6Wt7xl5fbspV\/d3PCnIpGMREYNl3RnYChFjx8dbgaQEe7oAiVqjEkW8ND9Ho17xylts802yYn8NgyS+X2ZaGXtEl933XXTi1\/8Yl6NOOWUU1IYbK211pqlW8q\/\/EFeCUp6WllEk\/ejjz66ijvKmNtDa+bhsAnbUIz4PCg9izzusssuPVszv2wgHD2VIFo1JPeg+AXkSSNDd2Ji5dnoKsG2225b9crk8uGtvN\/yMWEW1jORH3mXP7K5IGwfLTnycoszelbuNghj2Cpfnot8ClMaNvGjQ1cYYekqI7tEurneS1\/60uqrbPVU+enTRQxuoxVzTj1+nIiKGIZ6htDmSgjouznnzvtxMQfh93rekyKXIZF7+L7Z4AaGlzlumVVxVCD3CGZzKrfNqTLBHUAQD1ehnYvsLOSNN944Ob\/BZ+d6ny9\/+cuVuk99dZ+GIZUg+6NyRAVzRWLe8iUP3IG6LCpq+GvVte4eiNY+5HF1CEu9J1QuxqOz2267VXvJYg4Rw8k8Xca2CZK+VhtxuetgO\/aty1WYKHOkSxY9i4d71FFHVcO2j3zkIynK6DmuscYaKSr6xMRE2mSTTZL8SMNzXX\/99TkHgvg1MgJHS47I3GTqhfrB3QY9qmdBT90xP+HOK7174EeHO0\/PPdSfCXufeuqpvCrEMyKP441f8YpXJDZ82MMelurPuwqU\/bnyyiurPXhEEVf9eU+KROWh5JPwMLp7XZgHw228SJcbohf42te+Vp3ZrVXMD3mMBBFJN33dddcl59j5KcJgv19pC7c4u0JlOOKII6pvn\/Iw8irPIXP4BRLHvavKoLVHyIBKyy8gnP1WcY9IoZv3ElFGD1+eQoe+sMquMeEuAZEReuedd65azrqOeBApWlONzxvf+Mbkw0j2fuxjH5viwzNpyUeQTcNkh3PE6bl6vnHf71Ul1CgI54AR8QE3mbQjn+6bEMQPnahLcZ9fcz\/plcrARmF7ZVZX6\/XKyELPQs8oiA3Z0ihIekZNnkVAgxF1h5\/RSa\/nPekQe5F0gZbGZDfXxWYP1E7ZyFDu3+RGXnE26ehtomBxPeigg6Zb2KawdT9pacm17AxkeCDOeo+g5UNSxKjHgXAmyOKq+w16byirp9aqqjR5ukYADswUt09QJiZWDuf8\/KdwPunnB8KrzNx+f3diYqWu+7kiejjxGA2ojMBNBvLqOp\/QQGtspKlRYQO7tPPGVMP5ve99r\/rpIF\/FTkystIuPUDVMwuIBcp1\/\/vluk0NhPGc3Gijl5y5hkrG14DwxWILcoOdR0bm1ADIXLXSwcu21104qpnFkzHdS9s8yoh\/F9Xm5Chsw16Dmx8Rc5wMIoJLpjrVMMaSIsuR50Ooha+QXmcNfHHmrq9K7D938atgW4eLKxtEbuhp3s6EhrS+I2UweQ3\/JkiWV06m0ec\/mh4TFxfNmN7tZcjb6hhtu6DZp7HJdep5v5Vn703arwTBfadPrOqSr2zvuNWxGAW3phL8pQeRLQ2h6wIae3T\/\/+c9QSxrNn\/70p1Vv\/+AHP3hazhENv7rOvnpb9dixXp4jnTrqz3sSMWIML7H4+RQBtTIqDLcuWAvtQblXIXWj3Iy86667ck4jJp1aUZlSOXhqObRgeq7\/\/Oc\/RPOOvILpphEqz4RyySNw89PC6bk8aPfAJq7sBtxgyCesloxtyHJ4yBolMuGAO3DppZemsLueSZpIwt\/EN+KkZ4hCjmjHHntsMjl2r\/X0uzzI495zradD3gUaiMiPyqpy5YgGhg7dtjhjeESP\/d1za6D7IZEwgSCie2cj5sO56CF1GPe5z32ozAIybbnlltX8R93dYIMNpudL7E\/W63lXq3NayzAE4qgAgChS40eHmxGjldSN0sPM6LHogFbe6gf31VdfXc2b6IqfzGe6Ms4dkIbKGvfDvjIEg6hgsSLm6j5Py8pNvYzyHrrKC\/LLNsKGLeiF3fJJMZ0cJTtKV0XXGtKVhgUBww5zHORTUcmlYy41MTGRDF38NOgPf\/jDqtWNPJnHiJNu2F28\/SIqIduxYT18VC5yi1Rade5eYG+2lC9X93RNCzTU3F2gYc4bdfEprx4qwmtE9JAaIUPgkNevbPnEJz6xsh8SaqguueSSSi0WQno974pENFVeD5Y7Bxm\/kCnkV77ylWrbRMg8VMuHk5PT0VVeJs5Q3WR\/DOVe97rXVd1rnUiZ2tCdiO00zTxieQEyFRQBlLHXnEgFNcxToYVhmwjvHlQ2vVap7PxBGlb52Nd9HeKMdMSn1\/YeI8gd+hMTE0nvrmwhk6d6vO7lPXS6XlUmlZC+lTgrZNw5VNzIlwZAD5n71931vCgfe6mkdd22++23375qoEMPqXwmjmBkBx54YNWr++0tNuz6i37CgufAntzATcYdmFHrPfS8m+YmC+W4RgXgDx62zJvYeVC5MfzuKyMdfPDB1fIwff7eZ+i6LX9HvHHN45fpkPe60hGvfETlpqtixZK2NMlc6QbcQ9xHecUjvpDHVVriyZGHpydNaec6vdzSEwYMhTRIdRvGWX8aKcSjC+95z3uqlToVpB5\/Hi9dPVp9ThQ64mTzehzuyfmLgz3YhTxHrqPshrERRhq5brjZUZwgTG4vbjJ+bBthXCNc5EV+uOmCxlyvY9XQ\/dlnn13VO7Yy9DWUFw9Ix2jJ3FIvTr+OevrCkeV6M0hEYdiwlGjibMnQ3Ej8ulhr+UhnmEI2Rm8L3GDD1MWG5nCGNWBuFrEaJoJ7FUzl415d0I8N+7XJyEk0Pqeu30cyW78fG+rJQCyGpsgE5mxkhl16Ou7VCf3YsF+7jJxEMrRYz6mTt1UFXW2oh8mHN\/mwg9swy\/BrvsstTWnLQ68h3qjz1NWG\/eZjaCSKh2fMWs+El4R+LMzE2Hlg9oFZWrcKUtddne+72NAk3wvWI488csaEenW2W172hbDh0EiUF6SXe7PNNksBy7\/w1Kc+NQXcQ6SRsmEAAA\/qSURBVOh497TvvvumfW+ApVb7n6BXGmP52ALzbYF5JVFb4fRMgCSANF4eBpAqCBbEcwg\/+EEyS5tgydWKlbnA\/ON+sz73HudhcdmkrR726z+vJLLFZ6uttkr2LwW8qAMvX8GSN3QtiL1P4I297UXgkwvL55aGfWAFdjT74O9Tn\/pU9Vba2HxVgx0Jq1qeF1t+u9arfvTmlUSWthFmxYoVacUNQCqwARPshYLjjjsuATfwoxfkc0U6aCKdng2iV4veTE9m6Gio6HsRPV+b4ewAaNJp8i\/5dZGVdJryMFe\/QdPrGq5Nr8m\/5NdFVtKZq53y8PNKojzhrm4EAWSpExCxAMlywuVkE65XWshlqOhtuWEiUrkiFr96uNLb+lynyb\/k10VW0snTHLZ70PS6hmvTa\/Iv+XWRlXSGabdFT6J+CotskJMNyRAMkC0IViIX4uiREAuhAKHI+snHWHf1ssBAJPIL45arbfUxafYhlEoaOxIWqwlzgu2+++7Tw8VexEIqhNI7BaEMCxdr+cb5WhgLDESiQw45JGnhfe7tM1sTd4eMOCRjYYqxMtVBxr6IZUuIOZoy6a1czblWxrrybxDKnCoIRbbSd+XfpvRLfl1kJZ2VqY3m76DpdQ3XptfkX\/LrIivpDNN6fZPI90F2xr785S9PvglSAW1htw\/u0EMPTfl3HMPM6HzFhVSGekild0UqhCKPPCCPHgqZwJAv\/MbX1c8CfZPI23LfDtnJOjGx8jNb32n4VPmss84a6WGDbY9n0AlkUzjkQShkAoTK85ETyrYW97l\/uEtpdJGVdCLOUVwHTa9ruDa9Jv+SXxdZSWeYtuubRAjkvYxPmPOM2E7u4zvvgnJ53a1rXVXh82Hb\/H145iO3\/AM15Ine6WUve1nyJemwy3n55ZenYce5usVXr4\/DuO+bRCqL\/Unx0VM\/mdCK+3n0gK394Dsa8DFXwMdTExMTCXwuoTVpg7y06ZT8u4bL9Syq7LTTTqnUO\/my0nctDr7QO+mp87CRhy6yXMfhGRF2VNc8vX7S6BquTa\/Jv+TXRZbrcA8bfZNorhnwUVTA57dgCRlUuIB3N1p22GOPPdIOO+xQgRvI6Rx\/\/PHp+CkIj3Di7jePKkuXMCW9GO6ZP1nlM5+KuDQ48mnehFD1uVMpvrqsfh9xj+o6aHpdw7XpNfmX\/LrISjrDtF\/fJPKVqmGNnqKeEcO8ps+9VaqlS5empVNwKAXU42i7RxII8tVJh2AI56oCB9GQTJi2+Ofi7\/2Ulb1S76Ts8mOnhCXzOqHmku447MJaoG8S2Y9W\/8xWEZo+9+YPDms0pAMre7DddtulgHvgDw6xMAcJaOWDgOJrQp1oyKQSB8Hc68FGQay8d1IO5MrzitDyglB6KYQiy3XG7lXHAn2TyJL2qD731jMBogDSLFu2LC27AUilUiJanXj8\/DIAwgknfK\/HgGAqrV5MZQ5iIRX0CmcS3suPvORvZc8wz0KEvMsb3UD0UHonhPLNf06qUpwRdhTXQdPrGq5Nr8m\/5NdFVtIZpu36JlGPz2yTc78cJG5eMswMtsWFdKByIhHCIZQKi2gIB2QI1otciIVUkJNqWD2VhRiEyod7VjTz8iGUHh2xkUpP5WQcL3gR64ILLsjVx+5FYoG+SSTfKoOH7Oig5cuXJ++OnLPtSGH+C4XSBBLBAMkQrE4uxOJXz3OQSoU2v9JDleaBebhS+uGf+8Vwz6GF5k+IJV+lfFjps9VIPpAJsUCv5R65QM+KhJHeoNc8n\/3E0TVcm16Tf8mvi6yk00\/Z2nQnnStWV3IqzHOf+9zkaGAfuPlUwGJCXc+9\/XLmSPy5yRYK\/XTbiAWIpYzRayEV5GUIQplHBaHIch3upvRLfmQIhTwrVqyotlIZ9iFWL1JJBxAmyIVgGjXECoJxk9WJ1kY2eRJ\/v+gark2vyb\/k10VW0um3fE36s3qiCy+8sNrO45jbww8\/PFma\/cIXvpD8jEeQZLHunWsqaBe\/INWyqTlYTiryCI88hnzIBHqo8BvG1bAPsYJUXuwiFRiStpFLHhAMEKZONMRCMGQDbiBHOO+39GwgLIgHxCn+MWZaYBaJPCwvN\/1AlcPt\/Pze5ptvnpxX7Y35Yt47N2i33Ssc8iCUORXoocjChEEocyhkYrfwq19LaXSR0UEqQKIgl\/dS4HnFcBDBrATq2ejX81C6RwxAEoTx6wd6NkAqQDBANsQLuAd+hvN0AQED4gTxgyGx9KCUH+UtyclKfl1kJR3xDYhZwWaQSAEV1HkFFhBC24Sdn18YM\/+x9Wcx7p2L\/I7iijx1QuXpjLJ3ytOpu5EFaZALrAQiFXIFybjJ+CEaBNmEFUc93i73iADqDCALIGAAqQDRAogHdTKShY4wEGR0FTdIC6QNXfI6Sp1ZJDJHsr0kT9SStlb20ksvTQjkpeqge+fyeIftHnTs2zVc6OWE6tU7xVBPb6WcEZY70EVW0onwXa4IAsiCOIgGCIVYgGQIBzEnIwd6gHggDhAfiBu65KWXDiIEkAOQBYKMrkgFQTSkgzoZvSagQxfM6YOEzuLolY9B5bNI5BwEL1R7Raiwg+6dE6dKMSoYbg4Sd9dwdT2\/YcNWL3zhC5NTRfNhA\/JE7+QQ\/ZNOOmnW5tF6fPJel+X3uZvuKOAXDS0SOfAFvBcEjQX4BAb84iE4QxA+8IEPJPM3QERAPnjDG96QwHBUhXcFG3jBUj+YD2qg1JNBoG6C1wQ5ES0IBQkdYDNI3E1hZpCoSXFYfiraqODorEHi7hquSc\/88TWveU21KKOy5ZVBw6TC77\/\/\/snmWz26fJbiq8vyeyME4UaJPL1+0snD2ZwLeixwmD74vSQnLrmCDbxgqR\/82oifL9Ujgh4yoFcEpAS9YkAaesVAU884iu\/dZpBIV+e3cPJfWMvJ4yTTueydy+O6sbqRJ587ebB5WaOHshhRH\/LlemN3SsgQYEdAGDAkDSAVggUQ77jjZp8Wxd\/cfti2nUEi57XpUq+88soZ6bj3w026dcMX74VUhlxJF6qrVOhcPp9urf0g6XUN16ZX90cowxYrexZn6rsl2DCGfEEorxXIoxz1OEM+quug6XUN16bX5F\/ya5Opj4anCAhGBcO23QwSWTzwmbdlTuP9SOy0005LCIZEYKHBd0Dx3kjGxj+VEtaafUUmJDL8QCjDvV6E8rs8CAXG8e5tPcqJNTuFsWQhLTCDRPa92f9mAuvcBL+n6VQf49UNN9wwWfYGbi\/l9t5770Rnm222WbC9c7nxjOHz+67uruHa9Jr8ww+hDPeCUHoqpKrnFWmQB4mQCan8VIqribL3UibPdOph53If+ew3jq7h2vSa\/Et+XWQlnX7L16Q\/g0QU7X\/bc889q18Xsy\/O\/ri3v\/3tyfiTP3BbQuRHx7sjL9uE5T9GNwsglCEGUsUOiSDV0qVLZ0Wix0cu5DEMRCYEi\/kVgrlHMKCHZDArsrFgaBaY9Lsx9dj8SJSf7kAOP+XhZ1EsKoQeNxk\/OnSFCf+FupbGx13y0jVcm16Tf8mvLkMqy8pIpadCLEvneiow5+xVHuQChEEwCJIhFqIBogEZf2QLIJ19k+IQV6+0SvJ6WUo6ZLP0CDM0+Zf8ushKOlmSc3ZO2tZTT4QhmzagrqqHN87ZWgsQgXdySAXPfvazqw8Yzasgei0E03MhYVsWkQMQBWmQLYBUPpxEMERDOuAO8AO6QT5Xjan4xBuQTlt+bgz+kwrq4MWrrrqqKs+qvAF10LFv13Btek3+Jb8uspIOskAMBRFMz4VYei9XIKsTrSvZqspwwx91JBAEQZggn6vFKMRCsADiIWHAvfdkrqEjDCAieIcmboi0XKVfskUXWUnnhqIN5TK58847JwsJjCBGa+xW6VbFDajyP0ZKCAYIUycaYiEYsgE3kCMc6NlAWBAPiHMu9kWEAGIAsgAiBpAKgmiuiBdkdHUP\/IA+ICKIE6QB0p1L3pvCTvohXKSxXcKLKAmvqhtQ68PSpoLnfl3Dtek1+Zf8ushKOnne5+pGDEAShIk5md4NoQDBANkQL+Ae+Blq0gUEDIgTxA\/mddKDueQdKQBBQL2FXkREOOST17mkWwo7edlllyXvhCwWINGoN6A6AH9UcLD+IHF3Ddem1+Rf8usiy3Vy9yDl7BKmVxqlsI961KMSPO1pT0sbbbRRtX\/QQsjrX\/\/6FLCKCyow2FOnQoMeBA477LAUMCcDIyOwxB9wBAFo8MFqJZQq9nzKJu1dcrigvU9IJFN2JfTKhMyb7NrZ0Eunl3yx\/WraOD+\/qF5lLLQdHNwZ8NIevvvd7ybwbixwwAEHJPDeEnwoCvWeMXpHvSLkvaLesFf9HFQ+ecUVV1SfJT\/gAQ8YNI5xuLEFFoUFDBEDyGIYCYamgFAjGc5h+eMf\/\/jKCEuWLEnjDaiVKcZ\/xhbobIHJNdZYY1rZ\/jjDNBtOp4VTDverwgbUqayO\/48tMO8WmLHtxyrdeAPqvD+DYSY4jmsBLDBpNQ4sKKzqG1AXwH7jJMcWSJP2vIHf02EPm0jHG1BZYoyxBbpZYDKWNzfddNPpEEhlU6n9UDaZ2mzqPVIocJPxo0NXmPAfX8cWWJ0sMGNOtDoVfFzWsQWGZYFFRaIzzzwzOchiWIVbvPEMP2fmtFtvvXVab7310qMf\/ehkZ4Dd9sNP6cYbo3emvmpgQ699fJDaxYaLgkQObXfskt3kltJvvI9pdCU74ogjqshPPvnkZAOmgza9+a+E4z+dLIA09gD6FMhmVkcgXHTRRa1hh0qi3XbbLW2xxRazEsVmGXSMkj1Y9mc5EinOaPDreuuuu2568YtfPCvs6iYY1IY2YW6wwQbJfNWJTM5xO\/\/881c381XlHdSG9v3Z1WCV2mE86idbVpE2\/Bkaiew2Puqoo4pJtR2A7\/3Us571rGoYUoxgNRHOxYaOjbJHjKl8E6aRsuXF\/eqEudjQ2YE3u9nNqs20T37yk9NDHvKQ5LSgNvvNmUQO27My52xuBagnuJgPwK\/ndaHuh2VDR0M598IZGE4nNbZfqDLNd7rDsqHN1QcddFAyP7cp1qcVbWWZM4m8V3IykN20a6211qz0LIE7vxvJdJMU7Bo3fDvrrLOSF71kixLzlKlh2NCkeKONNqo+sDQ\/esELXpDC3vNUjAVNZq42\/NWvflX9jFAMgY2OdA4+vWgr2JxJtPbaa6fnPe95yXDMvrtU+4dAi\/UA\/FpWF+x2GDY0ZH7iE5+YfLOjAixYYRYo4bna0GdAevLzzjsvmQv5xs5HfubxbUWaM4naEpjL90dtca8u\/m02dAi9Xv3ggw9O5kXG83DggQeuLiZqLWebDTX0W265ZfJVg0P2n\/\/85yejJx8dtkU+chK1ZSD3l2krK7ls7G63wJIlS5LD8n\/wgx8ky7KBjTfeuD3wWGPaAn4d0nzIqwFf1TqEv8uQ+P8BAAD\/\/696LBIAAAAGSURBVAMAWJ5S0q8GVfcAAAAASUVORK5CYII=","height":126,"width":209}}
%---
%[output:749b0793]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANEAAAB+CAYAAAC6cPTUAAAQAElEQVR4AezdCbxVVfUH8HWpDHMIrUwpDc2hnELNciyszIEybCDNRLRyzMLSrDSFtHI2P6VSln1MjXIIxMScUkrDJAtFI4cUopRyQtMciz\/fnef9j4973\/zu8N7mw3rnnL332Wfvdc7vrrXXXnvtIUvyv8yBzIFecWBI5H+ZA5kDveJABlGv2JdvzhyIyCDKX0HmQC85kEHUSwbm21uGA\/3W0AyifmNtrniwcCCDaLC86dzPfuNABlG\/sTZXPFg4kEE0WN507me\/cSCDqN9YmyvuGQda764MotZ7Z7nFTcaBDKImeyG5Oa3HgQyi1ntnucVNxoEMoga8kLlz58Y73\/nO2H777eNd73pXbLHFFrHHHnvEAw880OXWTJ48Oa699toulVfuLW95y0ue9\/Wvfz2efPLJeOqpp+KrX\/1qzJs3r0t15ULLciCDaFme1CVl5MiRceWVV8avf\/3ruOWWW+LNb35zuu6vh2+++eapfs+78cYb02NOOOGEeOUrXxnf\/OY3461vfWtK6\/KfUsElS5bEf\/7zn1LK4DrNIGqC9\/3cc8\/FE088Eauttlo8\/\/zzcdJJJyVJRVo5l4ZOPPHE2HLLLWPHHXeMP\/zhD6nlPuALLrggSbTtttsuCgmTMmv8WX755WP8+PFx5513xj333BMHHHBAkI7\/+te\/4gtf+EJstdVW8Y53vCNOOeWU1B7pn\/vc51LaJz\/5yfjMZz6TypOGBx98cHzwgx+MWbNmxfXXX5\/asc0228QHPvCBuOOOO+Kf\/\/xnfOQjH4ljjz02tI\/E\/fGPfxzvec97Un2\/+tWvarSydZIziBr0rm6++ebYdNNNY5111omNNtooHnroofQB\/vKXv4yFCxcGaYEeffTR+PnPfx7S\/\/a3vyXJ5RroNF0Zqth1110Xv\/nNb2KFFVaIyy67TFaHtOqqq8ZrX\/vaAJCi4O9+97tYd911EyA876677ooHH3wwfvjDHyZJJZ\/0evjhh4tbQvumTJmS+nL11VcHQP\/2t79NIFWHgo8\/\/ni8\/e1vT+0bPnx4qPeaa66Jk08+ObX1hRdeUKxlKYOoQa\/Or\/3tt98e9913X5IGfqWPOeaYuOmmm2LXXXeNV7ziFYne9773xezZs2PmzJkpnRRZaaWV0q+6pvuwfcQbbLBBUgnPPPPM+OMf\/yir27TDDjukcdNFF12UVDzg\/Mc\/\/hF33313kC6VSiXe8IY3xCabbNJW96hRo0J7Vl555fjSl76Uyp5++ulx4YUXxqJFi1I5EpZUrVQqMWTIkNT2l73sZQHIzzzzTKBo4X9NByKDXb+0VACDbuRcmrwW5nXNpvug3v3ud6eP6emnn07gKQpT4wCquG5\/ZBjw8QIj8uFPnDixfbFlrkkQUog0KjLPPffc+N73vhcAedhhhyUJSV003vHxK+ec+um8TIwiBx10UJBSY8eOjf3337+c\/ZLzjvrzkoItctE0IPKyqCPjxo1Lv2AGu5dcckkgujkA0cenT5\/eIqztejP1nVo2dOjQNBZiTQMeNGPGjCQFjIWcS3vssceC+uYJxi7UJ4AAQGMPfJSHqpFyZ599dmy44YZpHFaUoWaNHj06GD2AAiBJPiqY9mnnggULggQt7imOjzzySJJIxkIkTNG+In8gH5sGRD4Cv8gXX3xxGKz6NaQGoLXXXjuA69JLL4011lhjQLyPOXPmxC677JLGQVtvvXVS44488sj42Mc+ltQc6h1ac801Y+edd47ddtstpVOr9ttvvzY+UPcAzJFq5QN23p5JDBHF89QLHIwI5XKeffzxxyd1y5jFmO2nP\/1pMiQA5lZLDQ7M4a9\/\/evLt6VzY6lVVlklSNRPfOITCaC33XZbUu9SgQH8p2lARKemN9O\/DbL9Wv7kJz9JcxiOroHMB9Pq78OYwljGh8nkzMhgQO4Xn6pDPZOPnEvz0U+aNCmNj6ZOnRpnnXVWAIu8Qw89NJRFRfkyj5T785\/\/nAb2nnfrrbeG8deKK64YiAqnTfhvTEbqAMt3v\/vd+MY3vhEkHYCr\/4wzzkjjGmrggQceGMiztI8lUd3aB+iACLDenx9D5WgV2uPcMz1bG1y3KjUNiFhoqCL77LNPMo96Ccyf1BUv77TTTgvqRKsyupXbbaIWqLbddtsw3qFWDxSNoC\/eS9OAiJ5vTuGKK64IKp3BqzmPMWPGpLkPlh6D1r7odK6jexzYeOONw3shpUgyVrzu1TCwSzcNiEgZKh21wADbTPrLX\/7yxH1qnGtlUsLA+5N71MIcaBoQtTAPc9MHOQeaCkTmOQyezYr\/5S9\/CUe6uDR5g\/xd5e43KQeaBkRmvVmZmGnf+973JuuRo8GsNHnKNCkfc7MGMQeaBkTLLbdcGMAyYVcjecoM4neVu96kHGgaEHFDIW0++tGPJhN34e7D5YfX8re\/\/e209qWxfMxPzxxYlgNNA6LXve51YXb8qquuSq7+1tqYjET85jTdxKJjpsyBZuJA04CoYAr3H94JQFWkcTORJq9Iy8fMgWbhQNOB6NWvfnU8++yzyZuYEyQ6\/\/zzw+RrNiw0y2eT21HmQNOByKTqt771reAGtMceeySHTIvRjjvuuDARW258Ps8caAYONA2IqGtcezCF5wKHRy4mXE04QfIQlleUcV6TckbmQB050DQg4uJz+eWXJ69gDqfWDxV8cG5lJ49hZYr0fMwcaAYONA2IWN4EwPja174WFp9xchR\/AL3\/\/e9P6204pCrTDIzLbcgcKDjQNCAqGmQNPzcfcQW4+iDrWbj\/FGtSirL5mDnQDBxoOhA1A1NyGzIHusOBhoKoOw3NZTMHmpUDGUTN+mZyu1qGA00JIpEzBecQ9MISceGXTLq2DFdzQwcVB5oOROaLBK+YMGFCChQoRrQwtUJoyRtUbyd3tiU40HQg4h\/Ha0HILBy0NJy525JxedIyZQ40Ewe6AqK6tpd\/nIlXMaCLBwsiKEhJdvspOJKPzcSBpgMRoAjJJBSuGGYCDn72s58NYW0BrLvME9zELgRCb4mBdt555w3qbUC6y79cvnMONB2INNkqVuuKbNdhZwGTrW9729tkdZtE\/hREUH3WKDFUmMjtdkX5hsyBGhxoOIhsESLeHJVNGFsrWhHLnJhzVrYaE\/HoFhm1Rj9qJgOMyJ4cWEkyS89\/\/\/vf1yyfMzIHusuBhoDIzgKCAQpEIqSsJeE2mlqwYEEwJAi0vu+++6aVrla7Uu9sNyJ0bXc7eO+998Yb3\/jGttvsSDd\/\/vy263ySOVDiQI9O6w4iHtkcSRkKpk2bFrfcckvauMqyB+c\/+tGP0sZSPLnFpkYCpNuJwD096SVrX637PvWpT0VnxAkWlcu5RuW0WufKoXK+a1ROq3WuHCrnu0bltFrnyqFyvmtUTqt2rgwq57kuqJxe7bxaOWnDdz0sULV7yml+XP2IFmlijTtXB3LeESmDijLOa30LPU2vO4gYDo4++ugQc5t61b7h1C5kgrXIs9EUAJFSRVpXjyNGjAgx7Iryzs09Fde2SLQTXEfECRaVy7hG5bRa58qhcr5rVE6rda4cKue7RuW0WufKoXK+a1ROq3auDCrnuS6onO69lq+dty839H2Hx+IxP4zVthmbaOqrP5l24lO2GlHFqfRFnrGxrSrb11vktz+2L1e897481h1EgGDZg\/GNcY7xT5mMiyzKs73K9ttvn3Zu23PPPdPWKrYN6W7nbVti5zhzTMj5yJEju1vNS8rff\/\/9L7nuysWJV80PdMiUedGXtM85s\/u0vt60beK1Dy\/Tlt3O\/GOgVb9wfaCb\/rI4Hj1th5hz9NaJjtxp7ZQ+8vhZbWz866PPJF657\/A71o5Jd66ertsKdOGkJ++oC9VWLVJ3EBWtIG38ygCIcY8tE7n6CNQo3R6ftuQQX0HEH78+xb2dHQGFlwODxWabbRYmah0RSejXrLM6Ospfe+21O8pOeT5GH01BU2Y\/GMhHtHDpR9JX9NhzL4++qqu39TzwxAvLtGXNVYcGAhwEPIlBL\/45cqcRCVR7brlGG5g+u\/SH5sSr7o8blwJOsf+B6v4ERtddoa68o67U05UyDQORXSD+\/ve\/x9577x3GPdYR2aGaj5xdIW644YbYa6+9kgQSd47UIr0665Qydtku9gEFKBZAVjrnpFF\/WeemzF6UXjTg+CB9NAX5eAqafshmMRDp+x9efZl+nbnnWwN19t4KMK21ytA28LS\/B6h+98irIqJ9TmOvGwYi0sGHbrxTsAAAfORf\/vKXgxp2ySWXxDnnnJOCOZJOpFdRttrx+eefDxtTseQVIbeAp7jXGKw\/TNzAQx3x67ntuqukX1YgqdbGvk6rp9rS123vSX0zHlipS7fVky8NA5EP2oCRJDL2MS76+Mc\/HiTO3Llzw8e+2mqrBR862yL+97\/\/DRKlIw5ee+218apXvSoFfyzK9YeJu3hBwENvT+B587AXdfwRxaPzsQccIG16cFtDb2kYiPTa+IdJ+4gjjkhx5i666KK0j6k5orJ1buHChUH1c09BxjsmZJksDz\/88BBWi3eD+xgvinKOHZm45QNFd+jyeU\/GjqfOSoPoVZZ7IabutXocvtXQ6E4dfVW2J+3vq2e3r8c7aJ\/W3evhK\/9vTyr9qkZLlvw3\/v3vf3fKa\/dWe7b0vqaGgugHP\/hBAMDkyZOTlLH8gZHBJCxwFGRstPnmm0dZnSOlZs6cGUyY9gG1J6lIQFtssUXYoNc+qMZSDAnM2gXjnJdN3NINQrtCNz+8fHx+xuJghVp\/+LCktp33mS2jK\/cOhjImtXvbz6NGr+eV1KRKZUjSNnr6nJoV9yKjYSASwN7u0gI1Dhs2LH2IxjN82ziJ+tgBBIk9x9pGpYsa\/3g+KIsACJD4zI0ZMyYKg4LxlvORI0fWqOX\/k8vmaCobY0FS25aOeW49dESXBsv\/X1v\/nfm17b\/ae1lzD27fc8vVY7ulqnG1W9daaulbb6Vnq2Utk1ZPvjQMRFx\/GBeMYQoODBkyJBgbjj\/++OCaQ2VDpAyJ9cgjjxRFu3wkwah9vMF32mmn2GGHHUJaUYGJPwB57YEXR5lOnjo7fjbzjrj4yplxy\/VXxLBpn4onzt0rvnfQe6K9pCwkZiOOA7Etd5z6kRj65+kx5N8Pt5Fr\/B9+38\/DD21nvK7Fl+K99+WxYSDiB8eCRupYsXr33XeHqKfGPqTQWWedFbZTQeaKSJquTrZS9cwxOVYqlWQmv\/HGG4M3OBeSSqXSxsPCBP3w5I9FNfrnjw+MRZccndRGUi7TfXXhxQMzTn\/J+3CN94VHgvOeUtvL76OThoGoUqmEeSEOoeJu86cznjEmOu2004KaZ0l4QSx3Fuv1Ub9zNZkDfcaBuoOIGvfQQw8FNc0YZeedd06+U2eccUZIJ4ns1cpDgdm7IKZv+X3W81xR5kAfcaDuIGJQqLYjHtP0L37xi+CtYIKVV3eZSCjqXx\/1O1dT4kA+7R0H6g4iQAAIK02ZluVNvQAAD9ZJREFUoK025RuHih3xFi9eHN\/5zndC6OCCuPJ0NtnaO1bkuzMHesaBuoOoaCZVjkEBqIo0O+IBCg8Fedtuu20UJEYC7++ibD5mDjQLBxoGolo74jFxm7Q7+OCDY\/To0W3ENL388ss3C99yOzIH2jjQMBBxxWGBY5ljNLCOiNsIa9waa6wRYi60tbLKifuOOuqo4GdXZFtgxxCxzTbbhNWwPMKXLFkSOdpPwaF87A8O1B1E1LRitzuL78wNMSDwSrAjnk7yOGDSpsIBBQK0wjrH0XTcuHFhXKU8YrDgh2feSV3GW8ZVAyjaj25makIODKl3m8z18HGz6504CmIuFG1w\/qc\/\/SmMjb7yla+8JP4CY0QxfnrNa14Tgpu86U1vKm5Nx2effTYYJVxQC3mK12MphOdlGrwcqDuIGAfsdldrRzwqGRDxfSosc46TJk0KFjxrhqxQtXser4fi1fFmIK1IMBO4VspaNWspBBciUk3wRh7h6i7uswK1t1TUlY+DkwN1B1HBZitZAYOkKNw3uOVQ7yxlEJjEwjwfvw+fGkiC8asr6igf1WHsw2TObYivHe9uk7p2maD6yTOGUndxL9+43hLfuxEbvyM68+fK+ev0mkflqD095Wfx7vvq2DAQ1eoAsDBzn3zyycFze8yYMcGb4bnnnouDDjoojH2q3QtEJNhaa60VlUolxGjgsErFI7Eso6DekVgcXYs6VrzxpOAf1xviXTxsjzNi5f0ujF2+cUUccPav6uJfps\/NRNddd12\/95snC+ppv4v33pfHpgORzhnPAI1zxFXIeIlFznU1WnPNNZPbULHcXBxvYyhWQOMk81IIQFnsqtXR1bSyOuieOUdvHXtuuUaihY8+Ezfd+1gKukFCIUvHLaegNp541Xy39Bm1b0ufVdwHFemrPiP9RwUv+qD6DquoJ1+aDkQkhaUKxjdiLRgPWelKshgTycc9gGKEuPfee13G+uuvn5aUWy277rrrhhhoPLZ5ci+33HJhHIWMyRgm0k0v\/sHw7pDb2pcfu\/6SQGfsOiyQNUcF7bLu0LAC9u4HFsf5sxbGxhN\/8xKQAVpPaYvvzO+zunrahuK+clv0UV9n3vVw6Lf+I7x4+plnUpuVac\/Hzq6takWdlYul\/6qVWZrc5\/8bCiIma0ABGOMhi\/JInP322y\/N7RDbPBZOOumkGDt2bFLvgIOJ22Qs1Q9gcEX0IKCyEO+ee+6JCRMmxLRp0wJoqHXGXhbkOV9hhRXc0kY9XSXZ1fu+tcdmYQXsNV\/cOu6YuH2iYgnGQDr60Sj6U\/TTUb\/1H+GFa9J7ndetGIC3+4WL4pSbn0kLMzvjqfVnqLNytfLbXnofnjQMRCQJfziWNP3hpcDyZizEEGC3PGGzzPlYQzJ+\/HjFEihIkq6auBkpmNWRClx7jvOBQH5tW7EfxpEiIgEdVViIsUKiOVL7etOvevKlYSBiILDc2wpEA32rXJm+SRRWNeBSZr311guSQ54xjvHMnXfeGTwbHM0nYRg1z7IK4HLv6aefHtYnGV8xKFDl0LBhw6I83tp9992DpKtF6kblfJ4W0lA5vdq5Mqh9njTUPr39tTKoVrr5s\/Z57a\/dj8rprlE5rda5cqic7xqV02q1RTlULusaSavcdlFs99Q18eXVZ7fR6k\/OixGHX5nUvh1PnZUCkyjL8orci6RVo3JblEPKed+9AWe1exsGIn5wpAL9tmgY9Y752UcvnBYTt72KgMnyCaG1BL23ZOJnP\/tZWPbNp+773\/9+UOF4KHzxi18M4yRuP8zhDA2AVKhzVEDPiBcfOnXq1BAkvRYVakE5H3CrpZfLFOe1ytVKL+4rjrXKFenaUpStdSzKlvOrpZXzy+fVylZLo5aX7yvOq5WtllaUd\/zlCfvF\/FN2iekHbxYPPR1J7Vtzh3HhBxJ1dn+ZL+Wy3veLr77PDg0DEaAY8xxyyCFhabh5HVIEkTY+fkaBBQsWhFWvpMyll14aDAgYJA8XWNuYtkU1JdGAz9G4Sjqp05GJWx2ZmpcD2607LIyfjtxp7RSirBlb2jAQYYbgIaQK1YslzfmHP\/zhAISzzz47BW4899xzY9SoUQE0VDbzPWJp33DDDcl3bsaMGcF7ASjNCfkV23TTTQOI+ObVw8StL40iakqjnl3P5x650\/8iLB166\/C451+v7PTR9eRLQ0HExefYY4+N1VdfPQQkIW2AhopmXGRG+rLLLksg4o1g\/MQDAQeVEb1n\/PjxwUBB+pBgIgOtuOKKiqSJWaDbaKONUlBI5Z2TTKnAi388pzukHd0p359lB1Nbjvr4NrH74xfEwtlXdur5UIsvL77yPj00DESPP\/54GNdQ56hsxj903VNPPTUtxOO5jTBjzpw5wf1HbG6qH29vQGHBGzNmTJBAjA\/M4WbNmctF+2EKtazCRGwR7Ye7UTl4Y09nvvN99Yn6057PLLWofXp3rvsUQUsraxiIfPiAYNzCWGCC9amnngqmaE6k5513XgBNQYwG1DkGCVY5xgGSh2cC8zgVTp0MDEv7ldYjUfEYH8wP8VZAzkeOHKlIpsyBPuFAw0BkZatB\/+c\/\/\/mYNWtWGOfwumaGZqVj6WFsIIWUZd5mzQM2uz7stttuaecI8bstJ6e2KW8ylkQypjKRu9VWWyUrnvEXdY4aqI4+4V6upEMOWBTpndImrr766g7LtnJmw0BkwM8Ejcns94wAxjVizh133HFhYR4vb0dWPCZulrhKpRKMEFQ2eRdccEHa38hLYA6\/4oorwgSteQKmzUql4+CN7svUPxywwYB1YwLQUKdpAv3zpMbW2jAQ6TbVjATigWAuxzwQQJFEJmJ5NSiXqTU54D36IaNJGLOaSG\/NnnTc6oaBCEBIIssdhAhm2hZngbMoHzpxEkgk6pjYy8ZEpFXH3cm5zcIB75f3\/dChQ1OTTKozJqWLAfanr0DUbbb4VeKJYLEcLwNuPEBiRpk1jTrGzM0IgPl+1byUbj8o39DnHGDMOeaYY9rq5YtomsHGbFYPcxA2fvWDCEwKMvLQOJwPNGoYiMqMZJEDEtY3FjcBR\/bZZ58UR4EHAzImArryffm8vhxgEfVjZ1qCZCmebuwjUpNxqh8\/+06Z7PQ+WUu9N+X9SBb3DKRjw0BkwtMkK2dOerPwwcZHTNVeighAZfJyWOgGEvNbrS+ki3Vd7Z04Z86cGSymlp2Yhxs+fHjcddddwRrKSkpNp5ab0mi1PnelvQ0DUaVSCSZofnA2Jrb0G5MthSD6u9L4XKa+HDCNwMWKd3zxZOu\/qOZ+FKVVKpXkTWACHaD8+JkAd5\/8gUgNAxFmssD5xSJ5vAzOpTwSeFrLz9QaHCChjH9ao7W9buUyFTQMRIBCtzYvZF6nIN4LBqrLtDQnNCUHmK5pECZWNdCY1nhogw02cDkoqGEgMvGG+QahBqsFCZk1UAegA\/GLqlQqaQtP4yI\/fvaXsoylWLY\/EPvcvk8NA5Fxj8lWVrn2jcrXrcUB3vfGS8zbFkOKDWeaorV60fPW1h1EXHfMKZhIBaAPfehDccABB7TtRSRPmZ53Kd\/Z3xwwOW4Or3gOqxwNQlhoFlX+iUXeYDjWHUR0aAvmJkyYEDwWTMwBlGskT5nBwPzcx\/7kQP3qrjuIWHIsWTDjbRKVYaFSqaSVq1avmgtSpn4syE\/KHOgdB+oOIlY5ksciPKtVGREYFQr3kN51J9+dOVB\/DtQdRKxy1DVxFEaMGBGf\/vSnQ1oeB9X\/5ecn9g0H6g4izbZuyLEgflVZEhXcyMdW40BDQNRqTMrt7UcODICqGwIigRQ5moqCwzHRClVH12IiFBF9BgB\/cxcGAQfqDiIWOLPbtaKzyFNmEPA+d3GAcKDuIBogfMvdyBxo40AGURsr+v9ENCLx88TTK0gMvf5\/cv8+gaeChZQ88bn9mEAvP1G\/J0+eXE5qO+esKnAnv7u2xBY7ySCq8wszJ8Y1pqCjjjqqrQU8oJt1CXxbI9udAM60adNSKOd2WV26tCDTMgrj4i7d0ISFMoia4KX4lbZpmaAtYvDZn4lPIYdO8ff8WmumsMsCuJBiYu1NnDgxGGH8+jsqQwr45QdI4cSUVY9NAXzw8tU5bty4sBjSljXm6UgCkZakWXRnRSpwHHbYYVFMP5AYyHMKEm+BM7EFeEVarSOpqz1IfEFtELBTlCdhtXiy1Lq3mdMziOr8dvbff\/+08pMl0nL4uXPnphaIQzBlypQUxF+0ozPPPDPEavMRc42yXYzwuYJVMr74aIEi3Vzlj3vnzZsXVpXaptMEt7jminL8JRGVIQXsqGFh5MKFC8NGAYBGRRNYZNGiRfHggw+mCXGBNAViVEdBym+44YbFZTqW+6ifACKD1CWB9dNEu\/Vk2qUv+m8JhXKtRhlEdX5j9lIqLJO33XZbbLLJJqkFo0aNCr\/o8+fPDx+aX2sfoOCWt99+e5BOQi774CqVSvCkth4r3VzlDxD4WC2OE+McKIVQVpQUEHqZ97XnUyEFGQEQy1MsazDGIfWsNp4zZ07a\/sbz2q8TIsWkq7egch\/1VTi0Io\/b16RJk9IGA8aH0gFZaC1RnVy3GmUQNdkbo9IIPQU4PkDRcqZPnx4+VCpa0VwhmKlgxXX7IzXJNjPqQKQS9a99ueJaXaLSFteLFy8OH7zosyQfVZK\/I5AVZbp71P7zzz8\/OBnvu+++UalUultFU5bPIGqy12KgzY8QeHx0wiLbKcMiN+qWlaPSSSuAE26MRBFBdsmSJWGMokt+5UkXkgIYbGFDrZNXjQAXWIBJ+CshygQgEffPeMtYjXRsf6+ITR2pleXyVEb9ocZpc5FHEuqTSKlFWisdM4ia7G1R14444ohgHBCYX7Qc8aypVQwIVCNeHcBEOlG9XAtLJaKOsRNgUfcAw5GqKAac81rdHT16dJIMBvljx45NG61R+XzYhXeJ6\/b3e4ZxHRC0zytfAycp9Ne\/\/jUYNgDS0fjMuEtfSKjyPa1ynkFUxzdlNWi1DxlIUNEUYxEGAZLEPkviuFUqlRDvTSAXUog65MOtVCppV0BjoGuuuSbOOeecOOGEE8IvPWkiHVHtpHm+dhTP8lxpPmIxAJmaPdezKpVKssz5yO3CUaksq36x5JFYpJc6tFd9Rf2Onqct8ozLtB\/5gQAcfRW3rjeqouc0ijKIGsX5FnguNQ6YSDZBG6s1GXD23nvvMGYC6mplOkpjvufFT5p2VK6Z84Y0c+Ny22pzgFWNlat2id7n8GG8\/PLLk2TrSEqw9pnzqVSWlVSdtcIYkOpKSnZWtlnzM4ga8mbyQwcSBzKIBtLbzH1pCAcyiBrC9vzQgcSBDKKB9DZzXxrCgQyihrA9P3QgcSCDqKO3mfMyB7rAgQyiLjApF8kc6IgDGUQdcSfnZQ50gQMZRF1gUi6SOdARB\/4PAAD\/\/25IXK0AAAAGSURBVAMAjPORXdvvOXYAAAAASUVORK5CYII=","height":126,"width":209}}
%---
%[output:08ce1ec0]
%   data: {"dataType":"textualVariable","outputData":{"name":"used_device","value":"'infineon_FF1200R17IP5'"}}
%---
%[output:76286c21]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANEAAAB+CAYAAAC6cPTUAAAQAElEQVR4AezdCbzmU\/0H8PPQn0liMMqasY4JNZYpY2smMfZdiAyFxCgKY5CllC2SXciLQhLT2NeiTIiIUjITGoTIVi9b4n\/fZ+55+t3n99x7n3vnuXfucuY133t+v7N8zznf53zO8j3nfH9zvdf679VXX33vhBNOeG\/y5MkNUWuy7GQJDHoJzBVa\/7355pvhnnvuCR\/72MfCuuuu2yGJ05osO1kCg14CVRBVKpWw9NJLhw033DBsvvnmHZI4g15yWQBZAq0SqIKoZUwOLVO68POf\/zy88MILrcH1nUUXXbR+QPbNEhiEEqiC6IMf\/GDYcsstww033BDWW2+9sPHGG4ef\/exn4V\/\/+tcgFEuu8gCUQI9VqQqi97\/\/\/WGHHXYIU6ZMCffff3848MADwzXXXBPWWmutsOOOO0Zw\/fvf\/+6xgmTGWQL9VQJVEBUrYFTabLPNwiWXXBIefvjhMHHixHD++efHddI\/\/vGPYtT8nCUw6CVQF0Sk8t\/\/\/jdMnz49nHbaaWHSpEnh2WefDdttt10AMOF9hW677baw3HLLBW6zymTE\/dznPheQ53PPPTd8\/OMfD3\/4wx+alUU4+OCDw6c+9amgU8IXf\/k0LYMaRvKRn3xrgtq8ChdP\/DYBNS89UebEsyu\/p9\/H74Q81xSzx1\/9Zm1ABDgPPvhgOOKII8Laa68dG9Fbb70VLr\/88nD33XeHr371q8G0r8dL1oUMPvOZz4THH388cLuQrMOo888\/f7jssssiee4wcj8JPOmkk8KSSy4ZjjnmmD5b4mnTpsU1+A9+8IOm\/p49WeHddtstVEFEI7fJJpuE3XffPbz99ttBRewbHXXUUWHZZZcNlUol9sR6TD0F0mspoF5L76U34Arj8i+G80dp1OB632KLLeJo4jnxlK5IxV5KvMQ\/8eAiYZdeemns5fUSyiAuf6T8eKW44sgnvXP1aOqCrr\/++qABUrBsvfXWpRFPfHy5+OAtD3wTH+FIOZRHvETkbrqMv3zUP\/GQBuHHTxp8+SFyK4bhLQ9hKJWJ\/3333Rc22GCDkDoF+YiTSFy8r7766vDUU0+FjTbaKP7etfG841db5tq64qW8tVQbT3nxk7\/6hxDCPvvsU5IzPvJO5eUW83j66afj3ib\/xFMafPklSmHSkp22whVPOYSLS1b85YlPbbml50+eVRB5Oeuss8IDDzwQTj755LD66quHueeeW7xIfkSoI1w9P5AReMpEpEceeSSceeaZYerUqeGVV16JjU\/BKCb0gtZXhx56aElIK6+8chxNTBdvvfXW+OPhlwgPP9oqq6wS12hF\/ilO0f3GN74RG8J\/\/vOfcGCLgkSYDgENHTo0HH\/88cEIy78zIlRlNo2Vb+2IZ+PZ\/tqMGTMiK72phzXWWCPW85lnnomb2PLmTxbq4xnZLiAz\/OVjpFC+Yl3Fw9dvcM455wTxyHKBBRYQFAlPvOvJWVzAWGGFFWJcDYCc1cdvSe7f+ta34nTds\/oIf\/7554Pf2G+d4gEjJrVl1vBTXcUHCPmIm0hDLMYryoQcpROXWytnvGrLQhZ\/+tOfJInbMz\/+8Y+rbU9cMkn1Un58yYE8JNJxaStc78rMVS6DR\/KvLTc+4iqT+FUQSeAHfPnll\/mXyI\/Ic8KECXEer9KETah6U2F++GVbRq3VVlst9mTC7rrrrtigIRtQnYbQYG666SZJIhkBPaQf2XOR\/JhAmXjg\/9BDD4Xvfve7xWjVZw2B0Hbaaafghx09enT40Ic+FOnOO++M07R55523Gn92Huabb744TfrVr34V5cIlBzx1KrvsskvMV\/7KUfwRxaklMkpTSY3C6Oe3ES\/9BmQoHnnwRxoG3vyEiZPkTNaeP\/zhD4saAB5PvPXA5EguyhgjtP7RkMnRb60z0TBbg9o4TzzxRFBX9cNDfG2DLDTAFDnF66pMpFdmPPH2rszawEc\/+lGvgcy1PXXUUYqvLOolrs4egGPkwh8dUqqj9lpbB1FTuWvDUv2qIBJZY9UA\/RC1pOch+L333jv2WKnxaNwvvvii5B0S5Bom\/XD4dBh5NgPbA+Nssq2bXIMlK41IQwZa76+99lqc3xcTpXL5gYv+tc9+cLIis2JYbbrErxhHGmk7krOOEKik07DEl6f3Ihn5TGmsjwG0GFbvGcjwai++9lX726c61NatHv+u+gGwKbkyKVu99Cl\/ZdOWU5zUvtM7Fw+8autXBdEiiywSfvKTn8Th2yZrLe2xxx5xPn3iiSfGOK+\/\/nrs5aF+2LBh8qiSwv\/973+vvnswBEJ8Ir0D\/65QErRhWs9Y74evx09ZlAkRKjIVSg2pXpqu+On1xdfj+yG86xFr+afypx9OmlrScE2ldGZkZcqV+NSmS\/yKPBqRcxrJgV6DkF4DMSp5TnTxxRfHR2VQFmWKHu38Sb26uMiIqpNJ0bsrk5SebDV276ZSAJ6mc\/xqybQMpXKRTW2c9K5s2nJ6T+07vXMTH3VDqX5VEFn\/mJ8bAuvR+PHjo3Lh2muvjdMTP4DeyRAnnUz0xoY+5FmY0w+GYQ1MHD8UNHO9N0KGacO14RaAUt5pGtgeD\/VQBmVRJuTZSJG0jKkhpvK1x6sj\/1Q+DVE5vSPPNJvKjJSfLNKUpB7P2h\/TFC713kDkmZ8OwXQi8cAT71QP8k1yJifpUgPU+eiENBQNQeMAVHknflz5cVEqv+dIhT+prsqjXDoCDVxDL0SLCqruyASPVIdi3fFaZpllBNcl9VGvFJhkk96Lbhp5\/EbqmtqYOB3VT9wqiETuiPRe5uh6ST+OaYCeqTiiLLjggoHywVQCqv04GvKVV14ZFC6l42++3VF+xTC92eTJk6OyQs8p70Z5iKcsyoQoRvbdd9\/qD6rhK9ejjz5azLLNs5GFh\/Qap+ciKR9g8uN6R3o+C31lRsLJgkw8J0o\/kqkYsmZI5bLloCFooHiojzhAY8qYeOCJdz05iwtgqcPAQzr81B2\/U045JfiNNVadIzmNHDkyJNklpYUwDaxYZmtpdcVTXuQkPTnzS9QVmaQ0ydVetDdllYdp82kte5gf+MAHUpSSqz7KIY16pgj1wKRs+IlDLt\/85jere6LC2quf36lhEGGuUBZzhjJUBJDwpZZaKugphN3ZsoD3w\/LneuePknAJxjtXPP74y8d7kfgJEx+JK1xa71zkOYUJr807lZlg9MLio+uuuy5qCPFIYcI9F\/MWjm8tyRMfbgqTFg\/+iAyUR7hypPdiPP54iI\/OO++8oN74iFcMo+LGKxHeeEqHxBXGHzAB0UjhvRhP3FQvrnd5Gq1SPK4yCBNHWdK7MhffxeEn71qqjYev8oiHr7Rc77WEp3CU0iV+yuIZL2HiSs8VH6VnLtnwK+aV0vK3nNGB4IHwlocwhAd\/fNoFEfXwY489Fn75y18GPZ69I4kyzTkJGAX1qKZjSmFk8UObtnjviPTiem+jRkfxBmuYzsVa2TTUdNTU1xqsOKVtTzZ1QaQXWn\/99ePU7LjjjgumFNtss02499576\/JJCIZUiK0bKXvOtgT0msVeUG+o1yX\/zpiLI640ncUdjOHarfar7Zt5IM9Gms7kUQIRRJ566qnh6KOPDqY4ejlzY+fnzj777CC8M6Y5vO9IIJek5yVQAhGNjancmmuu2SZ3QOIvvE1AE15MUTItVz36lGXRM7JYauznw8Jf+2Wku2a80oSWO4tFCUQ0QYY2+nUHUmdFC+F3v\/td+L\/\/+79AFRh64J9pSqbHo3KjETn4CRqJl+P8T6aOs5Fbs6kEIvsnhxxySDB1o\/0BHu6xxx4bvva1r8UN12YXIvPrugRuv\/32rifKKXpEAiUQyWXFFVcMTi87wWB95ELer3\/963inRnh3iR0HhwaTdqm7fHK6LIHuSGDmS292J1mnaUogos52tMfp1osuuiju+1xxxRVR0XD44YcHI5JDpZ1yrhNBOhqQOkGD3SvXvx9LoAQi6x5rIcdj6Mzt1o8YMSKqtxdeeOHg2oKj+l2tsz0KdzccYO1q2hw\/S6DZEvjIwkOaxrIEonfeeSc4Y+aohKsEbNA59WuN9Je\/\/CWwvWCK15US0Oo5NrHrrruGJZZYoitJc9wsgaZJYOZLb1R59SiIWEJ1WJGCoZpjy4N3U7033nijeqaoxbuh\/84qGcUcRm0vAeBmeiJ2YI3IwU3ORuLlOP+T6ZU33hmbXzMBhGFpJHKtwQG\/gw46KF7RdY2Ahs6BPAqHd999N3znO9+RtmGyU\/79738\/LL\/88vG2p0N7jlgUGTjQmGnZeDC2ETk4p9hIvBxnlkznXnDx8M6wEbHJfWSh5k3lMCyBqFKpROs+e+21V6BJ23nnnYMT1Kuuumo47LDDgumeq+MSN0qOmqT9CtM657j6p4Kh0RrneH1NAtP++r\/N1XVXWKipxSuBCHd3izbddNNw1VVXBSd\/b7nllvCFL3whXqt2me2LX\/yiaJmyBPqFBKi297\/8z7Gsc73+Ypg0fnh8btafuiByctudiuLxk1GjRoUbb7wx0NzNTuYOURqZZodHTpsl0KgELr\/vuTDquLur0eeZ+Zvqc7MeSiCiVLA\/xNqJqZcLcDZaTes8L7RQc4fCZlUk88kSKErA6LPVWQ+GNAIJ22X0YmHIo1M9NpVKIKJ9s+5hg3ullVYK7lRQNjgS7mo4kDW1BJlZlkCTJAA4J978ZAAeo89dresg2rhJ45cNZ+0yskk5tWVTAhFV9vve975oVmn++eePVx+otk3jXFQCsrYs+uBbLtKgkADQmK4ZbRJwTrz5iZDAQwgAdM1+qzd9HYR3ohKInOLec889o7lZ9+mpuzfZZJNoJst1CPs9KXF2swR6SwIAg2pBA0D8aoFj6gY8vz9yTACknixnCUQyGzduXKCRc+yHJo6Wzt6Oc3NGKXEyZQn0lAQSWIADSNIoY4rmnX8RNMoBKIBjygY43PVWGCqox6kEIiaAbIRy5U7dPXz48MBiqGM7yV9YpiyB7koAUFyMAwjAQMDi0lwCCz\/htYCRZxE0acQBHEAS3ptUBRFwsO5Cte1CHreo4nZkh3minrqU15uVznn1vASABAEBAgggQQkoW509S3smHNUDi5ImwAAIoBhpkGd+vTXiKEs9qoIoGbJgjMQlPG46ZZDcCy+8sKmX8uoVKPv1fQkAB0ojCQDUA0m9EaU9oKg1sKy3\/NAAGABihHnp1HEhAYafMPHE7ytUBVEqEGump59+euAmv+wOHgkAB6oFSAIJYCRKI4kwQAIQ1J60NH5UBApgFMFyzf6rR1U0sMzpEaa9etT6V0GUpnPFKVzts+meeLVM8nvflwBgoI7AkaZZQFILkAQSPNqrLYCgBJK0NwMkRpNERaD0J7C0V+8qiNJ0Lk3d6rlOY4vXHrPs3\/sS0KhRT4Ij1aoIEI3fKILaA8mk8cPj1MyIIm3iM9DcKoiKFWNbzseRGLAzGq2zzjrxw8d5o7UopZ57BgrUHjAszre8+OkwW3hQmgAAEABJREFUuyNHqoEGjtIIUg8gaW2SRhHgEQ8NdJAkObXnlkDkyM8JJ5wQP1jlE\/z33HNPuOCCC+L1cCaHGBtpj1n2ry8BgEBAgUyNkLUEAgpkGtUIMKw7\/v7aO\/UzK\/gCBmoEHKZaGSAF4XXhsQQiX8pj45nZLBe6TN98jcyBVN+CaeSDXl3Iv99GBQpUBIVzW52Bor21BmDg15lAgAKtueSQOFUyEkxqPRdmWoUAIo0cnjM4OpPq7IWXQGRzdZ555ildeXB2rl5WRiafXPnEJz4RTPvYZXAluRjX9NAGrqkhchKC0fBinDn9rAGjeqBIwEgjRXujhXNbRhiAQPg1Ui+gQMURIwHDtKkeMH6w3WJRiyV8UmHtMdinVo3Iu9lxSiBy1cHGqg8N33DDDcH1cAoFhhuNSE50Fwth1GKz29TPZ1XcWnWFohjHWsp1Zt+1obBgKNx6qxinp541ZKRxo0ZGC6BBCRTSdQUU6gIUqAgMDR4BBTJKtDdiJGAYaTIwSLTvUglElUolOC8HRDZX3SNyiW777beP18QrlUqb2gCV6+MM3wt46aWXQnr2jlynMLoNGdLcu+14FymBBQCsMYojBj+UgAEUSJoij\/aeAQIVQTGpdRqVgJFA0R4wAAIBBcKvvfyy\/xyRQLcyLYHohRdeCEceeWQwrWMHwfVw94h8AY9NutpcjFymZ8zaMkRiarfxxhu3icYI\/v333x\/Gjh0bRo8eHXyRzOjUJlI3XgDAKAEcCTCe+QFIRyw1YNQRKIwWAIF+f+SYuHNeXF9Map1GJWDg11GeOWxgSqAEIneIfBGCkRIX8xgscbOV7biORODa91\/\/+tdo5IQFVeugFB8g3Yo1LXT13JTOpxFTONc6qlGa9tD0MPknD4bNTr8v3lwEGjyKtMQC7wsW31uOnD986RNDwzGfGRasI66dsFT43QHDw5RdF4v0\/c2GhoPXHhLpsyu9F9Ye9kakJed+OaBGy9Tb8bLJrP+ZwmpU9tbjxTbSrOcSiFzK22GHHcKUKVOC0ePzn\/980OAdSPWtTxf0ipkzDfztb387elUqlQCAb731VjD6RM+WPywFWSsBE5COHz8+WB+1BFX\/0wQ2Qkwf7X\/ti+G8374Simpeo4ARwdTKqPHHY9YPt359TLh479Hh+J1XD1\/ZbLWww3ojw7ofX7Fhs1SNlGdOxbHGnFN599d8dd7VBtfEhxKIanlr9C7qAcDMmTNLWjugY3KY6pumjo06P3Dx8p7pG4uqNHymcXfccUec1tXm1dm7EYeGzDRO3AQc0y7AASBA4i88U5ZAb0igBCKN3Edft9122+BWK6ON3Jtvvjn4SoQ1ULFga6yxRjQtvNVWW4UxY8aEn\/70p4Emz37TV77ylWCNRVHx3HPPBaPZ2JZ1kcOtRqYin86eAch6J8Wzlrlmv1mHFS3Sk392B4ME+lYdSyCylrntttsCrZx7RVOnTg32fmrBk6pRqVQCpcNvfvObID7FAnvbgJJOgxvJ2O++77774skH08J6SorEs9Y18hQBNKlFK2aBn0ecWknl9zkhgRKINP5zzz23Q+D0ZkEBaKuzH6xmCUCTWrRiVY\/8kCUwhyVQAtEcLk8pe\/s6gCTAeicDiCQy9SUJ9GkQAY+1EIGZulEceM6UJdCXJFACEcWCtQ23WFDv9fyLcZr9PLHVfjK+Z+48kpOpP0lgkJS1CiIgsRFKkeDENvf6668PiWjd7Af1lgVUo1A6dUATlzVwg6RF9sNqVkHkRMJvf\/vbgF599dXoOlCayFfyDjjggF6zvWAtlOR5aIs2Lj1nN0ugr0mgCqIFFlggHtkx2vggF9fHvIrkTFyl0vYAak9UyChUXAvlUagnpJx5NksCVRA5zuMKwz\/\/+c\/gjJvr4YcffngokjNx4jUr8\/b4AFEK22X04ukxu1kCfVICVRDZ\/HQygYFGF+x8NbyW+IvX0zU56eYnqlmsu3zvmIKtZjgIH3KVZ08CVRA5A+dYjpMJbHFvvvnmoZb4izd7WXac2iiUFArU2nkq17G8cuicl0AVRKko7MqxL+fYeC3xF57i9oQLRIlvnsolSWS3L0ugBCKGSayJHBtP5EoEwyUTJkzoce1cmw\/U5qlcX247uWytEiiBqNW\/jeNag3tFvWHtZ9qMl2PeeSoXxZD\/9AMJNAQi9XjyySejLTr3irz3BJnKVddDCw3piSz6M89c9j4qgRKIrHmsfWrXQ0xe+WKeUamrdXHKwR2j0aNHh09+8pPBHSVXLmr5AFHyW3eFhdJjdrME+rQESiBKayL2EqZPnx6six577LF4nRuQulMb5rRo9Zy9c6uVGS53i2p55fVQrUTye3+QQAlECu0MnVGD650ZYaPQH\/\/4R69dJvtL++23X7QgxEyxI0ZOSNQyyuuhWonk9\/4ggRKITL0uuuii4LjPhhtuGOuw9957h9133z24qeqgavTswp8VV1wxLLnkkuHggw8OzGv57uuIESNKHPJ6qCSS7NEPJFACEZAYLUaNGhUqlVnn5CgTmMSyjgGy7taLEUhGTVwXd428yIcZrPS+yiIhNGoGqe\/F67opp+7UIZvM6rqcrfNTG2umWwLRggsuGJxacPXBtEtmrPSwu5As\/\/DrCjnMyrSWNNZGTj7MmDHDa5Xe+8Ci1eePLb\/4gDBr1ZOmpVhU6kn+A5G39X21kTXxoQSieeedNxxxxBGBwUa2t52fW2WVVcIll1wSbLgCQVfzdx6PIRNgNNJRLtAAFvkUlQpLZ\/V2UTT5uY9LoAQio49DpqZbbMgx4ggAN954Y7C26U59ksksgAQetropKoq8prVusvLL5+VIIVN\/kUAJRNY8FADPPvtsoEGj8rY3VKnMWh91p2LWQExmUXG79Ic\/oNbj5aRCPf\/slyXQVyVQApE10RZbbBHcHTIS2XxNxBCjKVmzK\/PufMNCr2vmml2JzG\/QSqAEIp9Gueqqq4KPcB100EGBXe5ErocL70lp5ZMKPSndzLsnJFACEeONzAX7pEot8Rfe7IK8O1+LTruVaZ7OtQoiO\/1GAlUQmaqxnf3nP\/85mhDeYIMNQi0xLSxes2v3zrCVqyyzZq4qivzQTyRQBRHlwTHHHBNWWGGFcMYZZwRG7WuJv3g9Wbc8EvWkdDPvnpBAFUROJQAIrZm9oiuuuCI888wzgXbOYVFqbput4jW7IO8Mm3UECIBQkX9+zhLo6xKogigV1JGfo48+OioWAIi\/3euLL744nHzyycE3iPg1k4prombyzbyyBHpDAiUQ0b7ZIzK1c2hUIZxcYIuup262UnHL5yP5pAIxZOpnEiiByFRunnnmCc8\/\/3ybqjiuU6lU4nWG0MR\/+SJeE4WZWc0RCZRAZLN1l112CV\/60pfi1QXKBTdR2VjYaKONgnVTM0taBFEz+WZeWQK9JYESiGS86aabBooF07kHHnggGJkuvPDCsOeeewquR03xy4YamyLGzKSXJVAXRMpAmeDEgst5hx12WFhttdWq94uEN4uKp7ebxTPzyRLoTQm0C6LeLETKK6u3kySy258kMMdBVLwCkUHUn5pOLmuSQFNAZCOWPQYGSTbbbLNQa9DEtXKWglzPRewsOOCaCsHNACKFTP1AAqUilkBkM9UtVmsgDb5ILtS5FlHk8vLLL8djQuwnuCv05S9\/uWTQhHrcdeaHH344muB66KGH4hoLn5kvv8kJeY8oiiH\/6YcSKIHoxRdfDFdeeWU4++yzA9tz7qUnuvPOO+MxoGI9HUgdPnx49dbrqquuGnzjyOW+FO+VV16JGr4hQ4Ykr6qbVdxVUeSHfiqBEoicjRs2bFhYfvnlG9LGrbTSSuH4448PztW5Wn711VdHo\/f2m5JMXn\/99cAo\/tixYwN7dqeddlowOhUBtNA872QLP080bsEmW\/tpXFbJmpJZVWqTzXRLILKZyjzWj370o2At02hmrKR+9rOfDb7t6it7Tj6ktIC5zz77xC\/wMQhpZDPaFUH00Y8smi38LLtswzIwPbYNkalxmWl3qU020y2ByJrHZyfPO++84Mt50Juo3pooraH233\/\/wN72OeecE0eiYiFN8bbbbrt4ZMiINX78+GiWuKhMOOP4b4SUT3aX61QWFDlZTh3JqX5YsV0267kEIie3rX2gtpb4Cy9mblpx7bXXBsBbf\/31I1CK4Z5N30466aTAPoNpHJNZpnVA9NKp48LvjxwTnr7jR1HpUJtnfn88y+Xx5spAm2wmlUCEuWkcQyXrrLNOMPoAz\/e+972607sZM2ZElTYzw+kmrBuwjz76aHBTluIhmczyOUvrIlfMjUzyQsDEzZQl0B8lUAKR+0QnnHBCMMK4\/rD44rOskdK2sWQqvFjRcePGBVfKGXtMNhnYYlh55ZWjqhtgksksl\/vuvffecOihhwaX\/4p88nOWQH+VQAlE9n3+9re\/hcmTJ4dlllkmzDXXXIFqeq+99orAEt5fK5vLnSXQExIogYgmzWhj5Clm+NZbb0WVt\/Cif3efk0LCKYf11lsvuDlrzdRdfgM1nW857bTTToGcTJPNEGrryk56UcnAOGZtnD773ssFe+qpp6ICzJKlWVmXQETF7d6Q7wlddtllwebr1KlTw8SJE4PGvtBCzfmCnSsWt956a7j55psDE8W\/+MUvgulesyo2EPjozFzJByKnQXbddddoVFOHVqyfmQPFTlLCOD1SDM\/PIe5LnnnmmWGPPfYIzz33XFNFUgIR7u4N+SH8KG+++WaYNm1a+PrXvx4oCCqV7psTxjsRwPj0JFBaM9HW2ZBN4dkNIV3Vp5AhjzXXXDNQ1DgR4j2Rjq64uZ38s9tWAj7MsOOOO4bufJShLae2b3VBVKlUouksGU6aNCnsu+++gTH6Zk3lFIFWz4ahZ+SExJNPPukxU6sEXNF3IdJXNXixja7DKYLItBiIjjrqqHge0W9WewBY2sFOgEMJph37yFwz5VEXRKZXMnOcxwkDP5CpnOdmZl481dBMvgOJFxl11Hm9\/fbbETznn39+tNBkW+HYY4+NI9ZAkkNfrksJRK+++mpw5Ofcc88NN9xwQzCtu+WWW+IXv\/1QtQqH7lZu+PDh8YBrSu+w68iRI9NrdlsksMgiiwTyfu2111reQuCaXi+22GLx3R8gs0eXRvVRo0YFp0KaPe+XV6b6EiiBSM9muGMJtZjEfJy\/H7Ho393nMWPGhAcffDA2Eg3FswbQXX4DMZ3TIcDh6xzqx\/VtJ+tI74i2yRROJ+TdJrffSDrvmXpeAiUQOcHt+A7NnCM6ikD17EsRNlCF85tdWmONNYLTEIyiOEtnvspvdvkOpPQ6LecRL7300kAJc\/nll0f1LH9+aOmllw40qRMmTAgUEOwF+tJhEWgDSSZ9sS4lENH++Ere6aefHmgz7D\/4Qh5V6wUXXBCvSPCb3cpUKpVgGuJbrm7G7rbbbnEfanb5DrT0SyyxRHACxEkPHZt3daTuRp51QOToI2rXX399cFq5QHUAAAX\/SURBVOCXf6ayBFw2dbjalLcc2j2fEojMw9ma84N0RN3LLqfKEhh4EiiBiCaIOhBaHbc3Rdhmm22CDVfqVfN0NPBEMSBqlCsxByRQApFdcgdQ3Su65pprgtHINM50wpTOvsQcKGfOMkugz0qgBCIHTG2EHnLIIfGWpVGHQXt7RT1l0L7PSicXLEugAQmUQGQ6Z5ecRq6Yvva9GNafn+2Hoc7q4FCng56dxWsknPJm++23j9pJI35naRyWZBu91sxYZ+maGa4MyexZI\/LqSt7kSlmFv3y6krYvxC2BiGrU6QQHTm22OkXsUh5VqxGpWSruvlD5OVUGG6H249iZMNL3dDlMwZvVCTro6hhYM8vMpkdaczeTb2\/xKoGoUqnEg6ZAxIj9zjvvHE8t6DndMapUmnMAtbcqmPKxtttkk02i7QKu82V6QNfWkd6VtSLP9sOo9anddSLCWDFibEUavSX75OKxQ2FLQENNeSWXlRkboc4FUtA4NmXkmThxYnjkkUeCMO8pPreYRjkd1OWPbr\/99rDWWmsF+bJlYf3aXpmV0ejlhrIbx\/KbMmVKtJuhzKbn6iMeaqQ+ypCITPDQPlh84pJVCufia3Qhs3rv\/AYClUCkUqZ0NkFtsLqtyoaCBtVfb6P6MR1ZsglpZ9\/mpGPxNpXdskV6V8oTU6a77747GlJZbLHFAuWKMNfZ9cI0lqeeeioxRTNgelD7OPZpomfrH3na+Nx2220DS0hnnXVWAFA2+OTtuok9neJI5OSGxrzFFltEq0nKecopp8Rr+cJmzpwZXBnxe1x33XXRxFh7ZVYMaUwdHdsCNkY5lde+nJFQGcVrpD7i1ZIzltbOjHGapQAsYNfGG+jvdUE00CptlNBg3MmhONlqq62CntS5s2JdXcfQ2E1pNbp61wucmHZEyUanDTsmq4wYGnORlyvzGurmm28ejbc4jeFsoGv0xXjFZ1dP3n333bDlllsGpxIA7cADD4xX6W07+G6Uk9yu7DvW43hPR2WWxl0kro7BoWLlVW68uE6EN1KfYjnT89ixY4N64Y8fkLu+kcIHizvXYKio\/S29rV7ZSGLNpzcHrmL9NShTHEds9txzzzgaFMM9S6OhbL311nFqaEGMt3WO8EQO8lYqlQig0PKvUqkEpw2As+W17n\/lM9pT7IigcZoGAjtQuabPv0gdlbmYxjUToElpXfuvVCrxG7yN1CelK7qmqZXKrOm9Dkd5i+GD5XlQgMjo88wzzwRGJd2o1eh\/+MMfxlu7xR\/alE\/Pbv1kKqsBF8M9VyqVaErZjVwjB2Jj3NRNeCIN3mIe8TPN0ZBNEb3Xo9o0ym3ENC2rF59fI2UWT0eiDJ6RUcx7pdJYfaSpJfedkp9ndQXc5KdDmG+++dLrgHUHBYiMHtYWpi2mS35Nvb21n2eNiathuXqtMVA8TGlZiKcw4UYRUz0n3K2VHNAFTkZcaqdpI0aMCHhRBuABvJQGTsPjVY+kMQW0DpXGWks6Za0Xn19nZRYHmXrRtiqDKxVOpCj\/0KFD4wXMzuqDRy1Zn9k7xMe5PtNV8knxjKCsPSWFw\/Tp06NCJYUPFHeugVKRjurBbgTlgAtrtG5c7\/wd1tSg9Pimcq7Cawz8DjjggGjYn1aNVky6O+64I5r8Mv+n5aKA8TmZjTfeuE0RNJ7jjjsuGrWkvbJFYK\/Jqes2EQsv0hgtrcsASiOlEdUYC9HaPLZX5lpQsyGorMh0lhyMTjoSipXO6tMm09YXazMXAMnQ9JUV3EqlEr\/1q66iWTvSXlqPGf19Voc\/raRT\/Elzx6+\/0qAAkR9n3Lhx8QiT6Zfpmnf+GhSNHFBZdNOY6THPOOOM4MygnlZcKlz+9jT0tmzyeTeVcxpdY8SvSBrXTTfdFC2Y0t7hI7yjk8QpDS2ifSRlspYBaumkL74Lr1dmYCmmMSq6LuG+kTJ\/+tOfjuuhSqUSGq2PvItEueHbvsrqKyI6AeEuciLP6kM7qHMiUxpCMqSVtP\/oWbz+TIMGRP35R2pG2XUcRi3rLJpKaz5q6WS\/oRl5DFYe\/w8AAP\/\/R5Jy3QAAAAZJREFUAwDqVFCPo6mPuwAAAABJRU5ErkJggg==","height":126,"width":209}}
%---
%[output:38117c63]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANEAAAB+CAYAAAC6cPTUAAAQAElEQVR4AezdCbzmU\/0H8PPQn0liMMqasY4JNZYpY2smMfZdiAyFxCgKY5CllC2SXciLQhLT2NeiTIiIUjITGoTIVi9b4n\/fZ+55+t3n99x7n3vnuXfucuY133t+v7N8zznf53zO8j3nfH9zvdf679VXX33vhBNOeG\/y5MkNUWuy7GQJDHoJzBVa\/7355pvhnnvuCR\/72MfCuuuu2yGJ05osO1kCg14CVRBVKpWw9NJLhw033DBsvvnmHZI4g15yWQBZAq0SqIKoZUwOLVO68POf\/zy88MILrcH1nUUXXbR+QPbNEhiEEqiC6IMf\/GDYcsstww033BDWW2+9sPHGG4ef\/exn4V\/\/+tcgFEuu8gCUQI9VqQqi97\/\/\/WGHHXYIU6ZMCffff3848MADwzXXXBPWWmutsOOOO0Zw\/fvf\/+6xgmTGWQL9VQJVEBUrYFTabLPNwiWXXBIefvjhMHHixHD++efHddI\/\/vGPYtT8nCUw6CVQF0Sk8t\/\/\/jdMnz49nHbaaWHSpEnh2WefDdttt10AMOF9hW677baw3HLLBW6zymTE\/dznPheQ53PPPTd8\/OMfD3\/4wx+alUU4+OCDw6c+9amgU8IXf\/k0LYMaRvKRn3xrgtq8ChdP\/DYBNS89UebEsyu\/p9\/H74Q81xSzx1\/9Zm1ABDgPPvhgOOKII8Laa68dG9Fbb70VLr\/88nD33XeHr371q8G0r8dL1oUMPvOZz4THH388cLuQrMOo888\/f7jssssiee4wcj8JPOmkk8KSSy4ZjjnmmD5b4mnTpsU1+A9+8IOm\/p49WeHddtstVEFEI7fJJpuE3XffPbz99ttBRewbHXXUUWHZZZcNlUol9sR6TD0F0mspoF5L76U34Arj8i+G80dp1OB632KLLeJo4jnxlK5IxV5KvMQ\/8eAiYZdeemns5fUSyiAuf6T8eKW44sgnvXP1aOqCrr\/++qABUrBsvfXWpRFPfHy5+OAtD3wTH+FIOZRHvETkbrqMv3zUP\/GQBuHHTxp8+SFyK4bhLQ9hKJWJ\/3333Rc22GCDkDoF+YiTSFy8r7766vDUU0+FjTbaKP7etfG841db5tq64qW8tVQbT3nxk7\/6hxDCPvvsU5IzPvJO5eUW83j66afj3ib\/xFMafPklSmHSkp22whVPOYSLS1b85YlPbbml50+eVRB5Oeuss8IDDzwQTj755LD66quHueeeW7xIfkSoI1w9P5AReMpEpEceeSSceeaZYerUqeGVV16JjU\/BKCb0gtZXhx56aElIK6+8chxNTBdvvfXW+OPhlwgPP9oqq6wS12hF\/ilO0f3GN74RG8J\/\/vOfcGCLgkSYDgENHTo0HH\/88cEIy78zIlRlNo2Vb+2IZ+PZ\/tqMGTMiK72phzXWWCPW85lnnomb2PLmTxbq4xnZLiAz\/OVjpFC+Yl3Fw9dvcM455wTxyHKBBRYQFAlPvOvJWVzAWGGFFWJcDYCc1cdvSe7f+ta34nTds\/oIf\/7554Pf2G+d4gEjJrVl1vBTXcUHCPmIm0hDLMYryoQcpROXWytnvGrLQhZ\/+tOfJInbMz\/+8Y+rbU9cMkn1Un58yYE8JNJxaStc78rMVS6DR\/KvLTc+4iqT+FUQSeAHfPnll\/mXyI\/Ic8KECXEer9KETah6U2F++GVbRq3VVlst9mTC7rrrrtigIRtQnYbQYG666SZJIhkBPaQf2XOR\/JhAmXjg\/9BDD4Xvfve7xWjVZw2B0Hbaaafghx09enT40Ic+FOnOO++M07R55523Gn92Huabb744TfrVr34V5cIlBzx1KrvsskvMV\/7KUfwRxaklMkpTSY3C6Oe3ES\/9BmQoHnnwRxoG3vyEiZPkTNaeP\/zhD4saAB5PvPXA5EguyhgjtP7RkMnRb60z0TBbg9o4TzzxRFBX9cNDfG2DLDTAFDnF66pMpFdmPPH2rszawEc\/+lGvgcy1PXXUUYqvLOolrs4egGPkwh8dUqqj9lpbB1FTuWvDUv2qIBJZY9UA\/RC1pOch+L333jv2WKnxaNwvvvii5B0S5Bom\/XD4dBh5NgPbA+Nssq2bXIMlK41IQwZa76+99lqc3xcTpXL5gYv+tc9+cLIis2JYbbrErxhHGmk7krOOEKik07DEl6f3Ihn5TGmsjwG0GFbvGcjwai++9lX726c61NatHv+u+gGwKbkyKVu99Cl\/ZdOWU5zUvtM7Fw+8autXBdEiiywSfvKTn8Th2yZrLe2xxx5xPn3iiSfGOK+\/\/nrs5aF+2LBh8qiSwv\/973+vvnswBEJ8Ir0D\/65QErRhWs9Y74evx09ZlAkRKjIVSg2pXpqu+On1xdfj+yG86xFr+afypx9OmlrScE2ldGZkZcqV+NSmS\/yKPBqRcxrJgV6DkF4DMSp5TnTxxRfHR2VQFmWKHu38Sb26uMiIqpNJ0bsrk5SebDV276ZSAJ6mc\/xqybQMpXKRTW2c9K5s2nJ6T+07vXMTH3VDqX5VEFn\/mJ8bAuvR+PHjo3Lh2muvjdMTP4DeyRAnnUz0xoY+5FmY0w+GYQ1MHD8UNHO9N0KGacO14RaAUt5pGtgeD\/VQBmVRJuTZSJG0jKkhpvK1x6sj\/1Q+DVE5vSPPNJvKjJSfLNKUpB7P2h\/TFC713kDkmZ8OwXQi8cAT71QP8k1yJifpUgPU+eiENBQNQeMAVHknflz5cVEqv+dIhT+prsqjXDoCDVxDL0SLCqruyASPVIdi3fFaZpllBNcl9VGvFJhkk96Lbhp5\/EbqmtqYOB3VT9wqiETuiPRe5uh6ST+OaYCeqTiiLLjggoHywVQCqv04GvKVV14ZFC6l42++3VF+xTC92eTJk6OyQs8p70Z5iKcsyoQoRvbdd9\/qD6rhK9ejjz5azLLNs5GFh\/Qap+ciKR9g8uN6R3o+C31lRsLJgkw8J0o\/kqkYsmZI5bLloCFooHiojzhAY8qYeOCJdz05iwtgqcPAQzr81B2\/U045JfiNNVadIzmNHDkyJNklpYUwDaxYZmtpdcVTXuQkPTnzS9QVmaQ0ydVetDdllYdp82kte5gf+MAHUpSSqz7KIY16pgj1wKRs+IlDLt\/85jere6LC2quf36lhEGGuUBZzhjJUBJDwpZZaKugphN3ZsoD3w\/LneuePknAJxjtXPP74y8d7kfgJEx+JK1xa71zkOYUJr807lZlg9MLio+uuuy5qCPFIYcI9F\/MWjm8tyRMfbgqTFg\/+iAyUR7hypPdiPP54iI\/OO++8oN74iFcMo+LGKxHeeEqHxBXGHzAB0UjhvRhP3FQvrnd5Gq1SPK4yCBNHWdK7MhffxeEn71qqjYev8oiHr7Rc77WEp3CU0iV+yuIZL2HiSs8VH6VnLtnwK+aV0vK3nNGB4IHwlocwhAd\/fNoFEfXwY489Fn75y18GPZ69I4kyzTkJGAX1qKZjSmFk8UObtnjviPTiem+jRkfxBmuYzsVa2TTUdNTU1xqsOKVtTzZ1QaQXWn\/99ePU7LjjjgumFNtss02499576\/JJCIZUiK0bKXvOtgT0msVeUG+o1yX\/zpiLI640ncUdjOHarfar7Zt5IM9Gms7kUQIRRJ566qnh6KOPDqY4ejlzY+fnzj777CC8M6Y5vO9IIJek5yVQAhGNjancmmuu2SZ3QOIvvE1AE15MUTItVz36lGXRM7JYauznw8Jf+2Wku2a80oSWO4tFCUQ0QYY2+nUHUmdFC+F3v\/td+L\/\/+79AFRh64J9pSqbHo3KjETn4CRqJl+P8T6aOs5Fbs6kEIvsnhxxySDB1o\/0BHu6xxx4bvva1r8UN12YXIvPrugRuv\/32rifKKXpEAiUQyWXFFVcMTi87wWB95ELer3\/963inRnh3iR0HhwaTdqm7fHK6LIHuSGDmS292J1mnaUogos52tMfp1osuuiju+1xxxRVR0XD44YcHI5JDpZ1yrhNBOhqQOkGD3SvXvx9LoAQi6x5rIcdj6Mzt1o8YMSKqtxdeeOHg2oKj+l2tsz0KdzccYO1q2hw\/S6DZEvjIwkOaxrIEonfeeSc4Y+aohKsEbNA59WuN9Je\/\/CWwvWCK15US0Oo5NrHrrruGJZZYoitJc9wsgaZJYOZLb1R59SiIWEJ1WJGCoZpjy4N3U7033nijeqaoxbuh\/84qGcUcRm0vAeBmeiJ2YI3IwU3ORuLlOP+T6ZU33hmbXzMBhGFpJHKtwQG\/gw46KF7RdY2Ahs6BPAqHd999N3znO9+RtmGyU\/79738\/LL\/88vG2p0N7jlgUGTjQmGnZeDC2ETk4p9hIvBxnlkznXnDx8M6wEbHJfWSh5k3lMCyBqFKpROs+e+21V6BJ23nnnYMT1Kuuumo47LDDgumeq+MSN0qOmqT9CtM657j6p4Kh0RrneH1NAtP++r\/N1XVXWKipxSuBCHd3izbddNNw1VVXBSd\/b7nllvCFL3whXqt2me2LX\/yiaJmyBPqFBKi297\/8z7Gsc73+Ypg0fnh8btafuiByctudiuLxk1GjRoUbb7wx0NzNTuYOURqZZodHTpsl0KgELr\/vuTDquLur0eeZ+Zvqc7MeSiCiVLA\/xNqJqZcLcDZaTes8L7RQc4fCZlUk88kSKErA6LPVWQ+GNAIJ22X0YmHIo1M9NpVKIKJ9s+5hg3ullVYK7lRQNjgS7mo4kDW1BJlZlkCTJAA4J978ZAAeo89dresg2rhJ45cNZ+0yskk5tWVTAhFV9vve975oVmn++eePVx+otk3jXFQCsrYs+uBbLtKgkADQmK4ZbRJwTrz5iZDAQwgAdM1+qzd9HYR3ohKInOLec889o7lZ9+mpuzfZZJNoJst1CPs9KXF2swR6SwIAg2pBA0D8aoFj6gY8vz9yTACknixnCUQyGzduXKCRc+yHJo6Wzt6Oc3NGKXEyZQn0lAQSWIADSNIoY4rmnX8RNMoBKIBjygY43PVWGCqox6kEIiaAbIRy5U7dPXz48MBiqGM7yV9YpiyB7koAUFyMAwjAQMDi0lwCCz\/htYCRZxE0acQBHEAS3ptUBRFwsO5Cte1CHreo4nZkh3minrqU15uVznn1vASABAEBAgggQQkoW509S3smHNUDi5ImwAAIoBhpkGd+vTXiKEs9qoIoGbJgjMQlPG46ZZDcCy+8sKmX8uoVKPv1fQkAB0ojCQDUA0m9EaU9oKg1sKy3\/NAAGABihHnp1HEhAYafMPHE7ytUBVEqEGump59+euAmv+wOHgkAB6oFSAIJYCRKI4kwQAIQ1J60NH5UBApgFMFyzf6rR1U0sMzpEaa9etT6V0GUpnPFKVzts+meeLVM8nvflwBgoI7AkaZZQFILkAQSPNqrLYCgBJK0NwMkRpNERaD0J7C0V+8qiNJ0Lk3d6rlOY4vXHrPs3\/sS0KhRT4Ij1aoIEI3fKILaA8mk8cPj1MyIIm3iM9DcKoiKFWNbzseRGLAzGq2zzjrxw8d5o7UopZ57BgrUHjAszre8+OkwW3hQmgAAEABJREFUuyNHqoEGjtIIUg8gaW2SRhHgEQ8NdJAkObXnlkDkyM8JJ5wQP1jlE\/z33HNPuOCCC+L1cCaHGBtpj1n2ry8BgEBAgUyNkLUEAgpkGtUIMKw7\/v7aO\/UzK\/gCBmoEHKZaGSAF4XXhsQQiX8pj45nZLBe6TN98jcyBVN+CaeSDXl3Iv99GBQpUBIVzW52Bor21BmDg15lAgAKtueSQOFUyEkxqPRdmWoUAIo0cnjM4OpPq7IWXQGRzdZ555ildeXB2rl5WRiafXPnEJz4RTPvYZXAluRjX9NAGrqkhchKC0fBinDn9rAGjeqBIwEgjRXujhXNbRhiAQPg1Ui+gQMURIwHDtKkeMH6w3WJRiyV8UmHtMdinVo3Iu9lxSiBy1cHGqg8N33DDDcH1cAoFhhuNSE50Fwth1GKz29TPZ1XcWnWFohjHWsp1Zt+1obBgKNx6qxinp541ZKRxo0ZGC6BBCRTSdQUU6gIUqAgMDR4BBTJKtDdiJGAYaTIwSLTvUglElUolOC8HRDZX3SNyiW777beP18QrlUqb2gCV6+MM3wt46aWXQnr2jlynMLoNGdLcu+14FymBBQCsMYojBj+UgAEUSJoij\/aeAQIVQTGpdRqVgJFA0R4wAAIBBcKvvfyy\/xyRQLcyLYHohRdeCEceeWQwrWMHwfVw94h8AY9NutpcjFymZ8zaMkRiarfxxhu3icYI\/v333x\/Gjh0bRo8eHXyRzOjUJlI3XgDAKAEcCTCe+QFIRyw1YNQRKIwWAIF+f+SYuHNeXF9Map1GJWDg11GeOWxgSqAEIneIfBGCkRIX8xgscbOV7biORODa91\/\/+tdo5IQFVeugFB8g3Yo1LXT13JTOpxFTONc6qlGa9tD0MPknD4bNTr8v3lwEGjyKtMQC7wsW31uOnD986RNDwzGfGRasI66dsFT43QHDw5RdF4v0\/c2GhoPXHhLpsyu9F9Ye9kakJed+OaBGy9Tb8bLJrP+ZwmpU9tbjxTbSrOcSiFzK22GHHcKUKVOC0ePzn\/980OAdSPWtTxf0ipkzDfztb387elUqlQCAb731VjD6RM+WPywFWSsBE5COHz8+WB+1BFX\/0wQ2Qkwf7X\/ti+G8374Simpeo4ARwdTKqPHHY9YPt359TLh479Hh+J1XD1\/ZbLWww3ojw7ofX7Fhs1SNlGdOxbHGnFN599d8dd7VBtfEhxKIanlr9C7qAcDMmTNLWjugY3KY6pumjo06P3Dx8p7pG4uqNHymcXfccUec1tXm1dm7EYeGzDRO3AQc0y7AASBA4i88U5ZAb0igBCKN3Edft9122+BWK6ON3Jtvvjn4SoQ1ULFga6yxRjQtvNVWW4UxY8aEn\/70p4Emz37TV77ylWCNRVHx3HPPBaPZ2JZ1kcOtRqYin86eAch6J8Wzlrlmv1mHFS3Sk392B4ME+lYdSyCylrntttsCrZx7RVOnTg32fmrBk6pRqVQCpcNvfvObID7FAnvbgJJOgxvJ2O++77774skH08J6SorEs9Y18hQBNKlFK2aBn0ecWknl9zkhgRKINP5zzz23Q+D0ZkEBaKuzH6xmCUCTWrRiVY\/8kCUwhyVQAtEcLk8pe\/s6gCTAeicDiCQy9SUJ9GkQAY+1EIGZulEceM6UJdCXJFACEcWCtQ23WFDv9fyLcZr9PLHVfjK+Z+48kpOpP0lgkJS1CiIgsRFKkeDENvf6668PiWjd7Af1lgVUo1A6dUATlzVwg6RF9sNqVkHkRMJvf\/vbgF599dXoOlCayFfyDjjggF6zvWAtlOR5aIs2Lj1nN0ugr0mgCqIFFlggHtkx2vggF9fHvIrkTFyl0vYAak9UyChUXAvlUagnpJx5NksCVRA5zuMKwz\/\/+c\/gjJvr4YcffngokjNx4jUr8\/b4AFEK22X04ukxu1kCfVICVRDZ\/HQygYFGF+x8NbyW+IvX0zU56eYnqlmsu3zvmIKtZjgIH3KVZ08CVRA5A+dYjpMJbHFvvvnmoZb4izd7WXac2iiUFArU2nkq17G8cuicl0AVRKko7MqxL+fYeC3xF57i9oQLRIlvnsolSWS3L0ugBCKGSayJHBtP5EoEwyUTJkzoce1cmw\/U5qlcX247uWytEiiBqNW\/jeNag3tFvWHtZ9qMl2PeeSoXxZD\/9AMJNAQi9XjyySejLTr3irz3BJnKVddDCw3piSz6M89c9j4qgRKIrHmsfWrXQ0xe+WKeUamrdXHKwR2j0aNHh09+8pPBHSVXLmr5AFHyW3eFhdJjdrME+rQESiBKayL2EqZPnx6six577LF4nRuQulMb5rRo9Zy9c6uVGS53i2p55fVQrUTye3+QQAlECu0MnVGD650ZYaPQH\/\/4R69dJvtL++23X7QgxEyxI0ZOSNQyyuuhWonk9\/4ggRKITL0uuuii4LjPhhtuGOuw9957h9133z24qeqgavTswp8VV1wxLLnkkuHggw8OzGv57uuIESNKHPJ6qCSS7NEPJFACEZAYLUaNGhUqlVnn5CgTmMSyjgGy7taLEUhGTVwXd428yIcZrPS+yiIhNGoGqe\/F67opp+7UIZvM6rqcrfNTG2umWwLRggsuGJxacPXBtEtmrPSwu5As\/\/DrCjnMyrSWNNZGTj7MmDHDa5Xe+8Ci1eePLb\/4gDBr1ZOmpVhU6kn+A5G39X21kTXxoQSieeedNxxxxBGBwUa2t52fW2WVVcIll1wSbLgCQVfzdx6PIRNgNNJRLtAAFvkUlQpLZ\/V2UTT5uY9LoAQio49DpqZbbMgx4ggAN954Y7C26U59ksksgAQetropKoq8prVusvLL5+VIIVN\/kUAJRNY8FADPPvtsoEGj8rY3VKnMWh91p2LWQExmUXG79Ic\/oNbj5aRCPf\/slyXQVyVQApE10RZbbBHcHTIS2XxNxBCjKVmzK\/PufMNCr2vmml2JzG\/QSqAEIp9Gueqqq4KPcB100EGBXe5ErocL70lp5ZMKPSndzLsnJFACEeONzAX7pEot8Rfe7IK8O1+LTruVaZ7OtQoiO\/1GAlUQmaqxnf3nP\/85mhDeYIMNQi0xLSxes2v3zrCVqyyzZq4qivzQTyRQBRHlwTHHHBNWWGGFcMYZZwRG7WuJv3g9Wbc8EvWkdDPvnpBAFUROJQAIrZm9oiuuuCI888wzgXbOYVFqbput4jW7IO8Mm3UECIBQkX9+zhLo6xKogigV1JGfo48+OioWAIi\/3euLL744nHzyycE3iPg1k4prombyzbyyBHpDAiUQ0b7ZIzK1c2hUIZxcYIuup262UnHL5yP5pAIxZOpnEiiByFRunnnmCc8\/\/3ybqjiuU6lU4nWG0MR\/+SJeE4WZWc0RCZRAZLN1l112CV\/60pfi1QXKBTdR2VjYaKONgnVTM0taBFEz+WZeWQK9JYESiGS86aabBooF07kHHnggGJkuvPDCsOeeewquR03xy4YamyLGzKSXJVAXRMpAmeDEgst5hx12WFhttdWq94uEN4uKp7ebxTPzyRLoTQm0C6LeLETKK6u3kySy258kMMdBVLwCkUHUn5pOLmuSQFNAZCOWPQYGSTbbbLNQa9DEtXKWglzPRewsOOCaCsHNACKFTP1AAqUilkBkM9UtVmsgDb5ILtS5FlHk8vLLL8djQuwnuCv05S9\/uWTQhHrcdeaHH344muB66KGH4hoLn5kvv8kJeY8oiiH\/6YcSKIHoxRdfDFdeeWU4++yzA9tz7qUnuvPOO+MxoGI9HUgdPnx49dbrqquuGnzjyOW+FO+VV16JGr4hQ4Ykr6qbVdxVUeSHfiqBEoicjRs2bFhYfvnlG9LGrbTSSuH4448PztW5Wn711VdHo\/f2m5JMXn\/99cAo\/tixYwN7dqeddlowOhUBtNA872QLP080bsEmW\/tpXFbJmpJZVWqTzXRLILKZyjzWj370o2At02hmrKR+9rOfDb7t6it7Tj6ktIC5zz77xC\/wMQhpZDPaFUH00Y8smi38LLtswzIwPbYNkalxmWl3qU020y2ByJrHZyfPO++84Mt50Juo3pooraH233\/\/wN72OeecE0eiYiFN8bbbbrt4ZMiINX78+GiWuKhMOOP4b4SUT3aX61QWFDlZTh3JqX5YsV0267kEIie3rX2gtpb4Cy9mblpx7bXXBsBbf\/31I1CK4Z5N30466aTAPoNpHJNZpnVA9NKp48LvjxwTnr7jR1HpUJtnfn88y+Xx5spAm2wmlUCEuWkcQyXrrLNOMPoAz\/e+972607sZM2ZElTYzw+kmrBuwjz76aHBTluIhmczyOUvrIlfMjUzyQsDEzZQl0B8lUAKR+0QnnHBCMMK4\/rD44rOskdK2sWQqvFjRcePGBVfKGXtMNhnYYlh55ZWjqhtgksksl\/vuvffecOihhwaX\/4p88nOWQH+VQAlE9n3+9re\/hcmTJ4dlllkmzDXXXIFqeq+99orAEt5fK5vLnSXQExIogYgmzWhj5Clm+NZbb0WVt\/Cif3efk0LCKYf11lsvuDlrzdRdfgM1nW857bTTToGcTJPNEGrryk56UcnAOGZtnD773ssFe+qpp6ICzJKlWVmXQETF7d6Q7wlddtllwebr1KlTw8SJE4PGvtBCzfmCnSsWt956a7j55psDE8W\/+MUvgulesyo2EPjozFzJByKnQXbddddoVFOHVqyfmQPFTlLCOD1SDM\/PIe5LnnnmmWGPPfYIzz33XFNFUgIR7u4N+SH8KG+++WaYNm1a+PrXvx4oCCqV7psTxjsRwPj0JFBaM9HW2ZBN4dkNIV3Vp5AhjzXXXDNQ1DgR4j2Rjq64uZ38s9tWAj7MsOOOO4bufJShLae2b3VBVKlUouksGU6aNCnsu+++gTH6Zk3lFIFWz4ahZ+SExJNPPukxU6sEXNF3IdJXNXixja7DKYLItBiIjjrqqHge0W9WewBY2sFOgEMJph37yFwz5VEXRKZXMnOcxwkDP5CpnOdmZl481dBMvgOJFxl11Hm9\/fbbETznn39+tNBkW+HYY4+NI9ZAkkNfrksJRK+++mpw5Ofcc88NN9xwQzCtu+WWW+IXv\/1QtQqH7lZu+PDh8YBrSu+w68iRI9NrdlsksMgiiwTyfu2111reQuCaXi+22GLx3R8gs0eXRvVRo0YFp0KaPe+XV6b6EiiBSM9muGMJtZjEfJy\/H7Ho393nMWPGhAcffDA2Eg3FswbQXX4DMZ3TIcDh6xzqx\/VtJ+tI74i2yRROJ+TdJrffSDrvmXpeAiUQOcHt+A7NnCM6ikD17EsRNlCF85tdWmONNYLTEIyiOEtnvspvdvkOpPQ6LecRL7300kAJc\/nll0f1LH9+aOmllw40qRMmTAgUEOwF+tJhEWgDSSZ9sS4lENH++Ere6aefHmgz7D\/4Qh5V6wUXXBCvSPCb3cpUKpVgGuJbrm7G7rbbbnEfanb5DrT0SyyxRHACxEkPHZt3daTuRp51QOToI2rXX399cFq5QHUAAAX\/SURBVOCXf6ayBFw2dbjalLcc2j2fEojMw9ma84N0RN3LLqfKEhh4EiiBiCaIOhBaHbc3Rdhmm22CDVfqVfN0NPBEMSBqlCsxByRQApFdcgdQ3Su65pprgtHINM50wpTOvsQcKGfOMkugz0qgBCIHTG2EHnLIIfGWpVGHQXt7RT1l0L7PSicXLEugAQmUQGQ6Z5ecRq6Yvva9GNafn+2Hoc7q4FCng56dxWsknPJm++23j9pJI35naRyWZBu91sxYZ+maGa4MyexZI\/LqSt7kSlmFv3y6krYvxC2BiGrU6QQHTm22OkXsUh5VqxGpWSruvlD5OVUGG6H249iZMNL3dDlMwZvVCTro6hhYM8vMpkdaczeTb2\/xKoGoUqnEg6ZAxIj9zjvvHE8t6DndMapUmnMAtbcqmPKxtttkk02i7QKu82V6QNfWkd6VtSLP9sOo9anddSLCWDFibEUavSX75OKxQ2FLQENNeSWXlRkboc4FUtA4NmXkmThxYnjkkUeCMO8pPreYRjkd1OWPbr\/99rDWWmsF+bJlYf3aXpmV0ejlhrIbx\/KbMmVKtJuhzKbn6iMeaqQ+ypCITPDQPlh84pJVCufia3Qhs3rv\/AYClUCkUqZ0NkFtsLqtyoaCBtVfb6P6MR1ZsglpZ9\/mpGPxNpXdskV6V8oTU6a77747GlJZbLHFAuWKMNfZ9cI0lqeeeioxRTNgelD7OPZpomfrH3na+Nx2220DS0hnnXVWAFA2+OTtuok9neJI5OSGxrzFFltEq0nKecopp8Rr+cJmzpwZXBnxe1x33XXRxFh7ZVYMaUwdHdsCNkY5lde+nJFQGcVrpD7i1ZIzltbOjHGapQAsYNfGG+jvdUE00CptlNBg3MmhONlqq62CntS5s2JdXcfQ2E1pNbp61wucmHZEyUanDTsmq4wYGnORlyvzGurmm28ejbc4jeFsoGv0xXjFZ1dP3n333bDlllsGpxIA7cADD4xX6W07+G6Uk9yu7DvW43hPR2WWxl0kro7BoWLlVW68uE6EN1KfYjnT89ixY4N64Y8fkLu+kcIHizvXYKio\/S29rV7ZSGLNpzcHrmL9NShTHEds9txzzzgaFMM9S6OhbL311nFqaEGMt3WO8EQO8lYqlQig0PKvUqkEpw2As+W17n\/lM9pT7IigcZoGAjtQuabPv0gdlbmYxjUToElpXfuvVCrxG7yN1CelK7qmqZXKrOm9Dkd5i+GD5XlQgMjo88wzzwRGJd2o1eh\/+MMfxlu7xR\/alE\/Pbv1kKqsBF8M9VyqVaErZjVwjB2Jj3NRNeCIN3mIe8TPN0ZBNEb3Xo9o0ym3ENC2rF59fI2UWT0eiDJ6RUcx7pdJYfaSpJfedkp9ndQXc5KdDmG+++dLrgHUHBYiMHtYWpi2mS35Nvb21n2eNiathuXqtMVA8TGlZiKcw4UYRUz0n3K2VHNAFTkZcaqdpI0aMCHhRBuABvJQGTsPjVY+kMQW0DpXGWks6Za0Xn19nZRYHmXrRtiqDKxVOpCj\/0KFD4wXMzuqDRy1Zn9k7xMe5PtNV8knxjKCsPSWFw\/Tp06NCJYUPFHeugVKRjurBbgTlgAtrtG5c7\/wd1tSg9Pimcq7Cawz8DjjggGjYn1aNVky6O+64I5r8Mv+n5aKA8TmZjTfeuE0RNJ7jjjsuGrWkvbJFYK\/Jqes2EQsv0hgtrcsASiOlEdUYC9HaPLZX5lpQsyGorMh0lhyMTjoSipXO6tMm09YXazMXAMnQ9JUV3EqlEr\/1q66iWTvSXlqPGf19Voc\/raRT\/Elzx6+\/0qAAkR9n3Lhx8QiT6Zfpmnf+GhSNHFBZdNOY6THPOOOM4MygnlZcKlz+9jT0tmzyeTeVcxpdY8SvSBrXTTfdFC2Y0t7hI7yjk8QpDS2ifSRlspYBaumkL74Lr1dmYCmmMSq6LuG+kTJ\/+tOfjuuhSqUSGq2PvItEueHbvsrqKyI6AeEuciLP6kM7qHMiUxpCMqSVtP\/oWbz+TIMGRP35R2pG2XUcRi3rLJpKaz5q6WS\/oRl5DFYe\/w8AAP\/\/R5Jy3QAAAAZJREFUAwDqVFCPo6mPuwAAAABJRU5ErkJggg==","height":126,"width":209}}
%---
