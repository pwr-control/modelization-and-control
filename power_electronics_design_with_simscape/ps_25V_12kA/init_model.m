% clear 
[model, options] = init_environment('ps_25V_12kA_r');

CTRPIFF_CLIP_RELEASE = 0.001;
s = tf('s');
%[text] ### Global timing
% simulation length
simlength = 1.75;

fpwm = 4e3;
fpwm_afe = fpwm; % for PWM
trgo_afe = 1; % double update
fpwm_inv = fpwm; % for MPC
trgo_inv = 1; % double update
fpwm_dab = 3 * fpwm;
trgo_dab = 1; % double update
fpwm_psfbc = 2.5 * fpwm;
trgo_psfbc = 0; % double update
fpwm_cllc = 5 * fpwm;
trgo_cllc = 0; % double update
% t_measure = 0.648228176318064;
% t_measure = simlength;
t_measure = 20/60;
tc_factor = 200; % tc is ts_afe / tc_factor
tc_decimation = 1;
delay_pwm = 0;
dead_time_afe = 3e-6;
dead_time_inv = 3e-6;
dead_time_dab = 2e-6;
dead_time_psfbc = 3e-6;
dead_time_cllc = 2e-6;

glb_time = timing_setup(fpwm_afe, trgo_afe, fpwm_inv, trgo_inv, fpwm_dab, trgo_dab,  fpwm_psfbc, trgo_psfbc, ...
                fpwm_cllc, trgo_cllc, t_measure, tc_factor, tc_decimation, delay_pwm, dead_time_afe, ...
                dead_time_inv, dead_time_dab, dead_time_psfbc, dead_time_cllc);

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


%[text] ## Settings for simulink model initialization and data analysis
%[text] ### Settings Enable devices with thermal model
use_mosfet_thermal_model = 0; % this enables MOSFET based bridge
use_thermal_model = 0; % this enables IGBT based bridge

if (use_mosfet_thermal_model || use_thermal_model)
    nonlinear_iterations = 5; % for simscape solver
else
    nonlinear_iterations = 3;  % for simscape solver
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
ixi_ref_mod1 = -1.00;
% ixi_ref_mod1 = -0.88;
ixi_ref_mod2 = -0.88;
ixi_ref_mod3 = -0.88;
ixi_ref_mod4 = -0.88;

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
use_current_controller_from_simulink_module_2 = 1;
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
grid_nominal_power = 1750e3;
application_voltage = 480;
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
afe_pwr_nom = 325e3;
inv_pwr_nom = 250e3;
dab_pwr_nom = 250e3;
cllc_pwr_nom = 250e3;
psfbc_pwr_nom = 325e3;
fres_dab = glb_time.fPWM_DAB/5;
fres_cllc = glb_time.fPWM_CLLC*1.2;

hwdata.single_phase_inverter = single_phase_inverter_hwdata(application_voltage, single_phase_inverter_pwr_nom, glb_time.fPWM_INV);
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:4a50f0a2]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:2db503b0]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:398e68dc]
hwdata.psfbc = single_phase_psfbc_hwdata(application_voltage, psfbc_pwr_nom, glb_time.fPWM_PSFBC); %[output:50ab329d]
hwdata.three_phase_dab = three_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:58c3d75e]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:1998f882]

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
upqc_st.Lm1 %[output:5c827f36]
upqc_st.Ld1 %[output:1328f9a8]
upqc_ctrl.kp = 2;
upqc_ctrl.ki = 18;
upqc_ctrl.lim = 4;
%[text] #### DClink Lstray model (partial loop inductance)
parasitic_dclink_data; %[output:502f41c7]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:58b41b2c]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:0fd658eb]
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
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:623dffc8]
psm_ctrl.kp_i = 0.25;
psm_ctrl.ki_i = 35;
%[text] #### Induction Motor Control
im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:43a7054c] %[output:578aead0]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:43a7054c] %[output:578aead0]
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

psfbc_ctrl.kp_idc = 1;
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
sogi = sogi_filter(omega_set, sogi_delta, kepsilon, glb_time.ts_afe); %[output:69d2f95a]
%[text] #### Current control parameters DQ PI
dqvector_pi.kp_inv = 0.5;
dqvector_pi.ki_inv = 45;
dqvector_pi.pi_ctrl = dqvector_pi.kp_inv + dqvector_pi.ki_inv/s;
dqvector_pi.pid_ctrl = c2d(dqvector_pi.pi_ctrl, glb_time.ts_inv);
dqvector_pi.plant = 1/(s*grid_emu.trafo.Ld1 + 1);
dqvector_pi.plantd = c2d(dqvector_pi.plant, glb_time.ts_inv);

G = sogi.fltd.alpha * dqvector_pi.pid_ctrl * dqvector_pi.plantd;
figure; margin(G, options);  %[output:50fb542d]
grid on %[output:50fb542d]
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
% infineon_FF900R12IE4;
% mitsubishi_CM1200DW_24T;
used_device = 'mitsubishi_CM1200DW_24T';

igbt.inv = device_igbt_setup(used_device, glb_time.fPWM_INV, hwdata.inv.udc_nom);
igbt.afe = device_igbt_setup(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
igbt.dab = device_igbt_setup(used_device, glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
igbt.psfbc = device_igbt_setup(used_device, glb_time.fPWM_PSFBC, hwdata.psfbc.udc1_nom);
igbt.cllc = device_igbt_setup(used_device, glb_time.fPWM_CLLC, hwdata.cllc.udc1_nom);

%[text] ### DEVICES settings (MOSFET)

% wolfspeed_CAB760M12HM3
% infineon_FF1000UXTR23T2M1;
% danfoss_SKM1700MB20R4S2I4
used_device = 'danfoss_SKM1700MB20R4S2I4';

mosfet.inv = device_mosfet_setup(used_device, glb_time.fPWM_INV, hwdata.inv.udc_nom);
mosfet.afe = device_mosfet_setup(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
mosfet.dab = device_mosfet_setup(used_device, glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
mosfet.psfbc = device_mosfet_setup(used_device, glb_time.fPWM_PSFBC, hwdata.psfbc.udc1_nom);
mosfet.cllc = device_mosfet_setup(used_device, glb_time.fPWM_CLLC, hwdata.cllc.udc1_nom);
%[text] ### DEVICES settings (Ideal switch)
used_device = 'silicon_high_power_ideal_switch';
ideal_switch = device_ideal_switch_setting(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.afe = device_ideal_switch_setting(used_device, glb_time.fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.inv = device_ideal_switch_setting(used_device, glb_time.fPWM_INV, hwdata.inv.udc_nom);
ideal_switch.dab = device_ideal_switch_setting(used_device, glb_time.fPWM_DAB, hwdata.dab.udc1_nom);
ideal_switch.psfbc = device_ideal_switch_setting(used_device, glb_time.fPWM_PSFBC, hwdata.psfbc.udc1_nom);
ideal_switch.cllc = device_ideal_switch_setting(used_device, glb_time.fPWM_CLLC, hwdata.cllc.udc1_nom);
%[text] ### Setting Global Faults
time_aux_power_supply_fault = 1e3;
%[text] ### Lithium Ion Battery
% nominal_battery_voltage_1 = hwdata.cllc.udc1_bez;
% nominal_battery_voltage_1 = hwdata.dab.udc1_bez;
nominal_battery_voltage_1 = hwdata.afe.udc_nom;
% nominal_battery_voltage_2 = hwdata.cllc.udc2_bez;
% nominal_battery_voltage_2 = hwdata.dab.udc2_bez;
nominal_battery_voltage_2 = hwdata.afe.udc_nom;
nominal_battery_power = 350e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage_1, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:70f94d54]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage_2, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:5339cedf]
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

%[text] ### Remove Scopes Opening Automatically
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

%[text] ### Enable/Disable Subsystems
if use_mosfet_thermal_model
    set_param('ps_25V_12kA_r/full_bridge_converter/full_bridge_converter_with_iso/full_bridge/full-bridge_mosfet_based_ with_thermal_model_zvs', 'Commented', 'off');
    set_param('ps_25V_12kA_r/full_bridge_converter/full_bridge_converter_with_iso/full_bridge/full_bridge_igbt_adv_thermal_model', 'Commented', 'on');
    set_param('ps_25V_12kA_r/full_bridge_converter/full_bridge_converter_with_iso/full_bridge/full-bridge_ideal_switch_based', 'Commented', 'on');
else
    if use_thermal_model
        set_param('ps_25V_12kA_r/full_bridge_converter/full_bridge_converter_with_iso/full_bridge/full-bridge_mosfet_based_ with_thermal_model_zvs', 'Commented', 'on');
        set_param('ps_25V_12kA_r/full_bridge_converter/full_bridge_converter_with_iso/full_bridge/full_bridge_igbt_adv_thermal_model',  'Commented', 'off');
        set_param('ps_25V_12kA_r/full_bridge_converter/full_bridge_converter_with_iso/full_bridge/full-bridge_ideal_switch_based', 'Commented', 'on');
    else
        set_param('ps_25V_12kA_r/full_bridge_converter/full_bridge_converter_with_iso/full_bridge/full-bridge_mosfet_based_ with_thermal_model_zvs', 'Commented', 'on');
        set_param('ps_25V_12kA_r/full_bridge_converter/full_bridge_converter_with_iso/full_bridge/full_bridge_igbt_adv_thermal_model','Commented', 'on');
        set_param('ps_25V_12kA_r/full_bridge_converter/full_bridge_converter_with_iso/full_bridge/full-bridge_ideal_switch_based', 'Commented', 'off');
    end
end


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":12.2}
%---
%[output:4a50f0a2]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe480V\nNominal Voltage: 480 V | Nominal Current: 4.343492e+02 A\nCurrent Normalization Data: 614.26 A\nVoltage Normalization Data: 391.92 V\n---------------------------\n","truncated":false}}
%---
%[output:2db503b0]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv480V_250kW\nNominal Voltage: 400 V | Nominal Current: 470 A\nCurrent Normalization Data: 664.68 A\nVoltage Normalization Data: 326.60 V\n---------------------------\n","truncated":false}}
%---
%[output:398e68dc]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_800V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 800 [V] | Normalization Current DC1: 350 [A]\nNormalization Voltage DC2: 800 [V] | Normalization Current DC2: 350 [A]\nInternal Tank Ls: 1.697653e-05 [H] | Internal Tank Cs: 2.590412e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:50ab329d]
%   data: {"dataType":"text","outputData":{"text":"Single Phase PSFBC: PSFBC_800V\nNominal Power: 325000 [W]\nNormalization Voltage DC1: 750 [V] | Normalization Current DC1: 4.333333e+02 [A]\nNormalization Voltage DC2: 45 [V] | Normalization Current DC2: 12000 [A]\nInternal Tank Ls1: 4.006410e-06 [H] | Internal Tank Ls2: 1.386301e-08 [H]\n---------------------------\n","truncated":false}}
%---
%[output:58c3d75e]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: Three_phase_DAB_800V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 800 [V] | Normalization Current DC1: 1050 [A]\nNormalization Voltage DC2: 800 [V] | Normalization Current DC2: 1050 [A]\nInternal Tank Ls: 5.333333e-05 [H] | Internal Tank Cs: 1050 [F]\n---------------------------\n","truncated":false}}
%---
%[output:1998f882]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_800V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 800 [V] | Normalization Current DC1: 350 [A]\nNormalization Voltage DC2: 800 [V] | Normalization Current DC2: 350 [A]\nInternal Tank Ls: 6.880327e-06 [H] | Internal Tank Cs: 6.391587e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:5c827f36]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"   0.450378722818633"}}
%---
%[output:1328f9a8]
%   data: {"dataType":"textualVariable","outputData":{"name":"ans","value":"     6.061893472484110e-05"}}
%---
%[output:502f41c7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGYAAAA9CAYAAABflhxWAAAMu0lEQVR4AeycBXAVTRLHe\/nQwgqnkCqCFu7u7hDcIbhDcA\/Bg0PhnuBWHO6S4FpoFe7u7pBcfnPfvMvlEkjykt18j1B01mZmZ+Y\/LdPd+6L5WfTP19fXz8PDw8\/Nzc3P2dnZ79ChQ379+vVT19+\/f7eoV5HntdHEon+GYUiPHj0kQ4YM8uPHDxk5cqSkSpVK\/MGR6NGjW9SryPNa04H5+fOnPH\/+XJ49eybv37+XKlWqyKJFi2T58uXStGlT+fz5s1Am8kyRNT0xHZhXr15J9+7dpX79+lKjRg0pVaqU1KtXT1HFihVl2rRpcvr0aQVY6dKlZf\/+\/dKhQwd59OiRNTNk0VtNByZZsmSyevVq2bVrlwDEjh07xF+\/KNqwYYMSa56enuLq6iqpU6eWrFmzSs2aNWXs2LGKmyyaJ9NfazoweoSIMcQWQOl7KVKkUOLtr7\/+kixZsqjbnJctW1Zix46tnqmbf8Afy4BJmDChfP36VebNm6fEFKJq2bJl4m8XKRAeP35sm\/7Lly\/LixcvJE6cOLZ7jn5iGTCxYsWScePGKdHVuHFjadCggTx48ECJLBcXF3F3d5czZ85I1apVpVu3btKrVy+JHz++o+NhG5\/pwCC+njx5ojqQIEECGTBggBw8eFCOHDkiY8aMkUSJEkmOHDnEy8tLjh07Jjt37pSjR49K7ty5VZ0\/5Y\/pwLBH2bJli3Tq1ElOnDghHz58UHONCY2lVqhQIcmWLZviFP+Np7LW0DFwFWVU4T\/gz3+AMXGgMWLEkPbt28uwYcNk+\/btwqSnT59eChcurEQZllqjRo2U6MJ6g5o3by7VqlWTpEmTmthTa19lOjB6uJjCI0aMkFOnTsmtW7cUHT9+XJnJWGzsbfAEQGw8L1y4oAwAXd\/Rj5YBE9zExowZUwFw48YNW5GnT5+qe5jOtpsOfhLpgMGMbtWqlXTp0kVKliypqEmTJtKyZUtJnDixg8Px3+FFOmDoGnrHx8dHVq5cKext8AyUK1eOR38MWQoMVtbAgQMFiwuTeObMmXLnzh11Xb58eWnWrJniFAwCylA+vJEZv+uOhJW6rros0Pl5A+X5rNa\/pdD03TJgcPWPHz9euf3pcJo0acQ\/DiOLFy+WqVOnyvr16xUtWLBAOTux2tjjUDY8qcFCJwkr9dmQTZpsLifvvb3E+9i531Jo+m0ZMHiZfX19Bc6IFi2acsNgRrOvQcknT55cIHxmvXv3Fsq+e\/cuNGMLUdn06\/zEHiqx6rFAt1r8S7yrrwySlhVdLFCIOvR3IcuAwe8FAJ8+ffq7K6LiNADGJtR20\/\/k\/v378vDhQ\/+zyPt\/QOV0MqtJ1iBpc9e8AoWm95YBg98Lhd61a1e5du2a9O3bV8VoCJzVqVNH2HRqQtfky5dPuWtCMzh7y+JYxSKEq3fv3m1vc6GqHxCYUFUMj8I4KLdu3ap0Crt7znFmbt68WW7evKk2nWw+8aMRk0Gchcd7Q9oGfjpcR8SJDh8+bGrYwVJgDhw4IMOHD5eUKVMKkcvKlSsLBsGQIUOUdUb4Gbpy5YriqJcvX4Z0TsOlHFagk5OTsLeKGzeuvH79OlzaDUkjlgHz9u1bWbNmjSDK7t69q4wAEjLwn128eFFmz56twAIw9jIVKlQwdYOJ1UjuQWz\/AB0TiS6kz5ybQZYBQ5AsXrx4QtTy5MmTyonJ5OfPn19xEbEawsmacGwGNgrCOkHEedzc3GzVMdPh1IIFC0qJEiVk7969gmGCewiAKIhOTJIkCaemkGXAIB6+ffsmPXv2VHEXYjMdO3ZUcnzOnDmCYUCihqbw2GASBQVouBQO0DOMLiFIxyYXb\/bChQvl9u3bikOvX78uWIqUp4+6TkQfLQOGCObo0aMFq2fChAmSLl06QcdMmjRJ1q1bp4JnBNA0MWEB8wPCMjFwQYECBQSrL2B93D+EFQhJ4PXGo3316lXVHxYJBkmRIkUEDg9YLyLPLQOGQbGXITKJiPjy5YtyWAISMf4ZM2bI4MGDbYSosXeDieegUqVKgrjk\/RAbWpS6jvUYhqFMdfQeILEg9u3bJ9SjvFlkGTDIbjgGMxjdUrduXRX3x4M8wj9OQwi6ePHioonIJis6IiYGTkKfRETbYW3TMmBYpchu8sv69OkjiIwVK1aoUDOiBLd\/9erVRRMeZzgsrAMNrh5mMCKKzSRlyNJBv+AK4toqsgyYgAPG2iIYhuj4+PGjyoZBnAUsE1HnhmGo8DZ6BusM1w99yZgxY0S9MkTt\/j8wIapmfyFkOhtLzGI2cVhCeJXv3bsnZ8+eVa5\/xFd4WmXB9Ro3EPoHU5kwdtu2bSVt2rTBFTflvmXAGIYhxGIGDRqk9jCdO3dWVg8bSu2SYX8TnlaZnlF0GtafvkZ3kUZF1g7vQ2zqZ1YdLQOGAeP7wixlH4F1lDlzZjl37pxgFAS0yDjHICCSibihrqOTZcBgdbHRmzJlikr2w1EJAQymM5vBokWLCuIMtz\/lyUcDNEcHhfFZBgwpSlhD6BZ245oQaQAzceJEleXv7Ows06dPF7wEPMOSo+OOTpYBg+8J8xcPbuBJBhiA0PdxJiLq2Pvoe45+NB0Ydu\/s4keNGqUilrVr1xZ8ZOgRiBg\/KbL4xjAOuNe6dWvBlYKOYQPq6KAwPtOBYUPXpk0bcXV1VUoeTy4gcQ3xDKCWLl2qHJns\/HHTNGzYUIm2oUOH0m+HJ9OBwf2Byx+3O0nkKH\/DMFTiBckXOCoJNfPdjLe3tzIM+EbTxcVFgYFpq04c\/I\/pwGBdwSFYXHAFrnQUv9Yf6BL2MnzNjKLPlCmTwGVk0ACag+NhG15wwNgKhPcJ1hgTTdI4rv527dqpGAy6h3cR98Aw4Dng8a0MH9PCPdSlzJ9ApgPDpJJHxlETYGiOwVLDKsNfhWi76x92RvzxEVPZsmXVV87aTcOHTrqNf+oRb8OSJUuU8xZ3EDqXsVgCDC8OjjS34G3Gw0tmZpkyZdT3lwCBy0QTSRu6HbzCmNX6+p9wRGxv3LgxyFiPJcDwTUyuXLlUQIrIIN\/FcCSPjG\/7s2fPLviyuN60aZPg+SWgBucEnvC5c+eqLwOI68BVly5dEkxwHJKY3LjwqUNGTrFixRTHkdnp7u6ufgSCVUomDmVYreS3ATI\/CAFn0g5JIkwiz2mTLw9I2UXvIV5xE2E5co8gHIuKCee7US0J0JsQ79FE7gELEa+6vqePpgODeGKiyRcLSLA07hcmBWuNSYFItmOiqYM+AiwIoMimYSAYCatWrVKfoJOYPmvWLCEPjInB6uNbGyy7tWvXKpCZCCaaukERdQk7ELlk74ROZIFQlg0xxgplCK7xYxH4+nAboQcBj7EQleVbU76+BjxcTYyFNjRRnj2bvubIj00wPtOB4eXBEQYA6UKYxl5eXiqpXCeXA9r8+fNtSYDnz5+XnDlzqqYQdaw8vhRAzAEogyOfgC\/R4CKycQDEMAzBu4w7SFUO4g8Tq4HGOgRoQhEUzZs3r5AAj9nO+xGfJHEw6ehHwgfoDLhTO2Ux\/3lf4BgPgHGfdjXpMUYaYDCFcVIyCWSsIDIQK6w0MmoYdGCjQQ9GH9kfkYIEGHAjGS6EEBg8nKjL4e5B\/OjrwEeCdf3797ctArgH0Re4nL6mLZJL9PWbN2\/Ur3iQ6ePj4yOIUSxMxqDL\/O4YaYCho+gQ9i1sPtE7I\/xj\/xwZIKsY641ywZGTk5PAdQACENu2bZPJkyeroBeihugk9+EqQCRyysrHCuQ+Mp+24U64gBXNvotsUUQaz4IiFgMAABCLCvOe0HmePHmUHkP3wcWB6xIoDE6kRipgAnacWI1WnAHv\/+ocUcXPasFpuHLIcCH3GJGCkifhA+MCgOAixA7XpCeRBYMuAixEHZPNETGJf47z4N5NXoJhGCrLB9cRezDEHZyujRyuA9fnHehJFkXgZ5EKGOQ1qx0rCBmNhwBrjV9gYsXWqlUrcP\/V7wUw+foB9VDarHg+FSSxwzAMlUumI6I4RZkMwzAE3xzcuGfPHsGB6uHhIXARq577EGKNe4CDtajfxXu5B8iEyOFu3kvemmEY6lc\/MADot2EYuprtiAUHZ8FltEF\/aY8CvwaGEiYR1g4mL5PG6mK1MwmIA3KGeQ5wJnXH7tdgggMQHIhnPKgGAaNFixZKB7FQApaJFvDC6nP8Zih4T09P4ctlzGZNrGDM4vDoI9YU+is82gquDbYFGDNw4K+UPlYe+yLD+F+OijTAYJXBJSjmoIhnlAluIhztfqQBxtEm1t7xRAFj7wxGUP0oYCJoYu1tNgoYe2cwgupHARNBE2tvs1HA2DuDEVQ\/CpgImlh7mw0JMPa+I6p+GGbg3wAAAP\/\/htjc3gAAAAZJREFUAwDN\/0lbUd8VaAAAAABJRU5ErkJggg==","height":61,"width":102}}
%---
%[output:58b41b2c]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:0fd658eb]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:623dffc8]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:578aead0]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:69d2f95a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGYAAAA9CAYAAABflhxWAAAQAElEQVR4AezcBYxeRdcH8Hke3N2txZ2FYA2htDiB4JDgBQKBQJDgBElxDRbcWjxYkOBa3J3gUJzi7vLt7\/KdZfbuvd1n2+4LL+82\/HfsjNw5c86cOTMPzT\/6\/vXKDOy1115\/9O\/f\/4+BAwf+MWrUqB730Tz33HPTMsssk+aee+60yiqrpHvuuSe1t5L6\/nU\/A88\/\/3xaYoklirnbfPPN07fffltUkn\/HHXcUcXM744wzFvGe\/GmedNJJaYsttki33HJLWm655dIuu+ySHnjggdT3r\/sZWGyxxdJqq61WED7yyCNp8cUXL5i03nrrpW+++SZNMcUUaZtttinKe\/qnuemmm6bdd989LbDAAunggw9Oyy67bLr66qvTr7\/+2tO2\/ifpTzjhhPTmm29W4tlnn02YNyYT01x11VVTo9Eo6k4yySRp6aWXTk899VT6+OOPi7ye\/vnyyy\/TlzUYOXJkGlnCM888k57JkJf3tO9\/E31z2mmn7fQ9Cy64YPr666\/TZ5991ik\/TwwdOjTV4ZRTTkl1GD58eCrj+uuvTznycn1EWwcccECCPffcMw0bNqwAhubj+jfFm9NPP\/0\/+ntC+iaeeOIEU089dXrllVfSiBEjCkYNHjw4rbjiimnAgAFpqaWWKvT8\/PPPX+h6Bs0\/CW+99VbLc91smTIjfP3111NbW1stZp555gQfffRRF9VFVb388stpdMCMH3\/8MU3dzoR+\/fplPf8ZnbidSfKXX375BPPOO29BO9VUU6XpppsuzTbbbIUhw5g55JBDElx77bWV+0Dd\/tAb+aze1OK\/5nnnndfJXGbqjT\/++Gm88carbeLBBx9MLA8fDhNNNFECk33vvfemK664ooA0RpSBYaCDtra21NYO1st+++2XLr\/88sRSPProowujRP6hhx5axFdaaaWCVr1AmUn9+vUrJEv7EKqRCiRdm222WRIfNmzYP1odNg08N5dPP\/30NPnkkycrPlX887FWpY\/zkXDssccmuPXWW4uNvFxNW7Dmmmsmk2\/inZcwQRqUYVC5bqSnbpeeQYMGFQsCozAMo6JcOHFJkjBJfg7jtzf5bvAdZYbVfUfeTm\/Hmzb73XbbrTCXDZLtTWKEVZ2bYIzzcXXlJhlMOGAAiMsPBlTp3HJens7jDsEYtfXWWxdnBQe9fDzBJDTUHSblfee04jnDLDJzUWbYsHYpwzT0gXxMEY8QTcQjlNcKmtTN+eefn6iwww47LP3yyy\/F4eiTTz6pre+Mg0Em2Mea8DopcD6qbWgcFZh0jHEeExpX3nQwyb6FlsTFQqGSfYPvyetEPGcYCcO0nGEsSgyrW6jRTk\/D5lZbbZXOOeectP7666c33ngj7bzzzumnn34arbn8\/vvvp6OOOirt1i5pDqgm38ZrVZRhQOW8SFeVlfPydFU88oRffPFFsfk7gW+44YaFu2SmmWZS1AGGBYvu7LPPTi+++GLCKN\/gey644IJ03HHHpe23375QmSussEKiUToqZ5FgGMZgGAnbbrvt0iabbJL23XffdNBBB6WTTz65sCAff\/zxrGZr0eaOO+6YVMSUm2++uVBprVRtd9ClfyJmn332YlxO3BbbTjvt1GE4YEL+bXxbTucXXXRRuuGGG9Lbb79dGEJbbrll2mOPPdIRRxyRzjzzzJTvh\/Y2EtbW1pY31RH\/9NNPEzCQMI2EYRSmsRYxtIN4NJHmpZdeWgyGvc+co9JGQ9\/jItJRV6mqrJyXp6vikRdhVV9hOJhU6o7R0K\/destpQ5KGth+eHWrvbbcu8zZnbj8CYIi6ueomYdS4tqnFOobpy96tHfHu0DzrrLM6OTGlbayjM5e7azQvJ1V5Oo9XlZXz8nRVPPIizNuviveESSQpmBRtlfvhPcaMIUOGFFKGSSFh4hgGaOzN0U53YXPllVcuRN0+wYk566yzpmazmf5JHgGLhUTnyN3s3X1kXXkrTLr\/\/vsTNWTfOvLIIxNJIl11bUY+56Z9jkF14YUXFl4Ke3OU80WSPt\/k+yI\/wqaDYSR+++23FGCdRf7YhLk6KLdTVZbn2QN8HHVRrhtudqpYWV5PulVEvSomzTPPPJ2a4UNkOJAiwKRc9UdbTzzxRLFvR+V33303xf1M5HUXNtnlTv864CTU+e+\/\/54+\/\/zz7ur2ernJDzOU5XjXXXcVbhWb6mSTTVb0f8YZZ4yxJ7xooOJPMIlVZm5Ya4cffniadNJJO6hJDSZx9QSTLCQEt99+e8IM8QDJi3grYfPnn38uTMR11123MJd32GGHZJNi6hGzHHfeeWdHm1YHMDNzGnF5ykAFoZWtLOCjebCV5Qh6ee6FpOnnWWaZRTTJd8DlcZBhIT399NOiRdkLL7yQtB39CEkd01jdHMbJ4EED0qw4cXW+\/\/77hPFUGakd1O550LazUtHh\/\/\/JmaQNkqSIDw\/ELXzh6JCrt8KJSX0NGDAgXXnllWnOOedM3333XWEilhthWoc+tAly31hRZTp5ytCAuP0rp\/vwww+L84JVhqYKGKAOU54U5zQHHnhgIT0+eI011ihMZG3J17Z6AVIXe1K0cdtttyXjDBqhtNUu7lA611xzpbgWsVgXWmih5P6KGe6QalO3T6AP6JvZLe0MZVGJy2dUibeC4qIMIbUBuIZR8tjf4WW1guTdd999ySSZkNCb1EyZztlIWySFuKsb7T333HOFV9j1K2elCVUOVrQQmKdCdExRKxksjqCLkERoy6KaY445km8xJmqPe0kbyvVl7M4n2jZxxoPW+OQFYoIjLYz+xJncmIdJzHCShAH25wkmmCBRiT\/88EMSR4\/JpEu8OzSXXHLJTjQasjJsYMS6U2F7gmXBaqP6vvrqq\/aclEhS\/\/79izuQYAIdy0flBIzIaqEOTCwL5+GHH5adHnrooeL6VT7kqkW76hWE2R\/tBB2rUr111lmnYAayd955p2C8fAyl7uRHX8aOURYYNWg8aI855pgU\/Rnf2muvXThn1Y02o1\/0EGl3QQ6lTz75JPI04YQTpimnnLIAhsjEMNIv3h2aI0eOLB4NIORLMvi2tra06667FhOtc4gJR2c1Pvroo8lqkYZGoyEoQLqsQNhoo42KvEbjr3IZjcafaUz2UgdtFaxQ+VY1Rqubo9FoFC4lhz75pMMpXp2A9pVFX0Fr\/L4j6MxFjNfFm\/0qaKl43x20VeGNN95YMEJfHrUwr\/fZZ5\/iIk8eaeU2Eu8OTaLON4Zw+PDh6dVXXy32F4PwkcoNoizmnuTcdNNNxcpUNwcm0r1UWZ4\/NnH7zWWXXVbsK8ZDfUZ7LMuItxK66GuFrqc03DAkUT1SbUGTpDBi5Lc61qZNMRjDj7P66qunUaNGJXraHsLnpMGqjylPFklBC1SZVS4OVru0SS3Dhoomh9VlbD5OKJ2XUz\/GGHlh\/ZgY4498YYzdQrMhBy19n9Pq44MPPlClx7AIeay7qxh7b3d0zQ022KBD\/GxiRFalfNA2S0ySH2A6mzQQl++0S8JMgDTEBk4CQR7svffehaqskyxM54VAqx6IB0gl5kubaIwHaWcOEyVu7LHRL7LIIoX1xuQ1RkwMgwDtJZdc0rFPSfcEFl2Mh3YpL75YtGjQdtd2YS7nRPmgbZwmXugjcrp8xdqk0UHQmiQgDTGonM7Eas95hFoUL8MHhlTkdfVDVaDXB28wRjogm3AfLx9djEe+cnS0QNVDvWhTuz1FqCj9mMNy\/Vig8h1YnZHE69CFMQZ94okndqKnz0GmjzapJrNuj8EI+4FJUIckRX1p8AGkq8ryUw76GDFiRHG\/IV2GNvN+jJ1kBzODXlq+8sgzJoyPtFDa2MV7AtJJRakTUimeI1\/ITPsqUzynbxqsOwmiZ3UrFEoHpCHSMZkm3sREfoQ+Wjs58vro9KnvnKYurj91ytBmuU4wM6fFXPll2nK7JK+8xwRNXRva1LZyfZoP8yI\/R07j26nwqKOPnFa8i8Qkuf9y2BOpObDXxefax0B64MCBxaMU8b8D\/5OMsf+ACaeWMQjsY\/KovjFRaeqOK\/QKY5x+PbIbV4McV+0wh\/fff\/\/i5yZec1IhQiooBxVD9Yyrfltphwr1hsIh3y3nGDGGUzDg7AM6d9fNu+rU253Vgf4\/DcaKPv3MhAfYyZ6nQ97fDd4JzliqlFFUyRibd657Y9B8PRrwIiTA3AVPerwQeeyxxwqPLA+Chwie9kAwMtoam7C78bnupZr4sVzzhv+Ly2WttdYqnIqsQi9gXnrppbEZSpe6Yzo2qpNp32g0ildKXX5R5vTqR0xdemzPcC3gHrs9WvsfxyZHqHMP256bB4KRGJg\/Al944YWLg6aJbBXuSPK9Ieq5HndgdM9jgPxsniK5iUTjydK2227b0R83v7OLsnGF0Y3NXVf+6pVmIb3GykLl+OTlYHh0\/KKMhHBZO4TRt4hzuNHkwvcGy4dz7rmPCDhAcX4CHZnXLcfdbbjv8BaNi4Qz0Rto+xLVov8xRdxyjmn93qyH+a4HLCD3U+UfiTGzmdv26I5flDkJ84Zy+SPgEs8nlLuay9\/ZwSaKEV6GBDCKNAGpo0KEIE85JtYxzj5FsqhFoP7y\/v8t8UbjT696\/iMxVwou+EKtTjPNNKm4KGs0Gmm++eZL\/GYYZC+hjvLJwBQrPe4W8rK6OMkBzAhGYhJmBYJZeRvBpMGDBycMks7L\/y1xe5y7olD\/PAL2Q1cDXX5R5kc\/3j95C5BPgMkhSZ41ubNwiwgcf3Ddddel69rh8MayAA5EjxDo+jIw\/rXXXktOwMxEup5UYUbeb0gRw8JToGgn2i2H7733XnH3H3TdhVG\/iq6qrJyXpyMeoTYjLsy\/K49b8O5t7Jv8bC796n5RltfrEscg7nKg4oCbAdxJuE8HHd19993F81OXV7yqfERWSKPRSPGc1e0nS4pUeUtNmkgSaYvOmeLUo4cRxJ4eVi\/aKIfKWkHUq6KtKivn5emIR6jNiAvjW6pCnnT7iz3WHFaay1UVmZdWuUm1x7jPrqKrynOFAExmhzd2OknIXzoyZdX1MRgyZMiQ4kdMwSRlAdKrDdKFSdSddrUfNK2G+qujrSor5+XpiEeo3YhHKC8Haal69do00TmhuCvYRuPPTUoaWE4u1EzwNddcU7zedH8DLA2wyoE9TkWB\/aVfv36aqIT2TCpGxfusfIJzJmlbe3lDmKQuMMlJFONBHMNAe4A2r\/t3x\/0EhlXqG8tjaZqYPNPBkD3NasjzmXjednklk+eLkx7AADB5gwYNSoPawd1tQjEQMDAYZ1DqB4wFk0hDMMk+pdzgMTqMB21oV1kZGIARmAWYBBhGyoQgz89O8oMwRjp\/qS\/UFkQf9o2IC\/N0xCPMyyPP5i6f1qG2mMysMHk5miY6J\/Y4weuO\/OmsCjPMMEOy6fuRk3SrKIswBgbj+KqCWSZdWbQbTLJPYZKJirYwyYsU6s6+Q90xHDCKpaM82qkKTTRo06rFAAwMYBSmsLq4WgAAAzNJREFUCTEQMBSoTmlAc+qppxYvaTDUohLyv2kbYoHH2B1qLTbnRVfeG2+8ccf\/ZyEfa9NE58QOWCyvnCjiJsJgpK2AcQXmIVXpWRHXhPdZ+ghgEilygcfy82F53\/Y+Eo25Ts78dW4Jw9Jz+idhwOrBvLpvjD7rQgwNmHioY6y5wkAMBb+PMdceDDKYMHXRRRet7KqpspvAID7ttNMKK4rPp1zDwzVuBflWQCsYHW1VGYvPWEiSieROQQfEn+VHioCF5wONQ7mQ9SOEsPRcgJEwiB8jXXXVVcVrIAwkcQGqkvSB\/kkhsBgBU0l8d1JpPGXY6GkkY\/Yjsbh6KNNJd1hl1JnN3eoTV\/h3wwRY4fYl0pCPB5OoDqbl0KFDE0aRKoyzisPKy+vUxU1yQJ8YABhDSwBGgetnzMNIahRjhdIkVBk6wFBtaAtDufTrxlDO7\/CVcVzyV+WOtTLxmKSt3Lp6VWXlPGl7D0OCFFUxSfsYhSEYg0H2CwyzP4G0fO5+QIt5jUYjUZXaKEPf3eUFDcaS0JyxGEpKMclLHee+cnt16Q5fGR1d5Virq\/h35VcxiTSxBqvGZNIBEzCDlAEmYRZgHCaCeEAZOsDMgHZAm6D9qr7HJq\/DV6YRFkSn\/\/uSzLGETbquiaqycl6eLscxidFAmrjKSRPjgfrALKsXw9DVjaGcb5IDJh0DADMDGAUYBxiJqSDOSBEq467CUEaL8ZT7q0t38ZXRhRxrXPt1leRzYbcCl1V1dFVl5bw8XRWPPKHnqJiCOQ6aDJuYLJPkYAzcRfYmoPqA6RrWlpBq9J09BaaqK8RYGz2G6qNsbY6u7THylTGp\/xvBVwcOdn4HA4wGuPjii4ufjtsL4Pjjj0\/2NCCJARs6WABAMoE0AAmFKimtyqtjTodVVkfQl5+SCQ2YdMAEoEYBkwDTACMxFcRBeavzWekrY29XOdZabbSPrvMM5EztXFKf6uIr46Lgw2L+1VfrK+ntGejiK6N\/6xxrvT2Yvvb\/moEuvjLWSZ1j7a9qfbHenoEuvrLROdZ6ezB97f81A01OST8hYG9351j7q1pfrLdnoBVzubfH0Nd+xQz8HwAAAP\/\/48BiZQAAAAZJREFUAwDKM4Hi5SMUmwAAAABJRU5ErkJggg==","height":61,"width":102}}
%---
%[output:50fb542d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGYAAAA9CAYAAABflhxWAAANSUlEQVR4AeycBZAUyRKGsznj7jg43F\/gPNxdF3cWd3d3h2Vxt8DdnSBwCdz9oRG4u7vbm68uapgbdvaGndnpuWEJcru7urq6qv5KqczsCffZpH+fPn36PHjw4M8BAQGf\/f39P+\/atetzp06d1PX79+9N6pX3vDacmPTPMAxp3bq1JE6cWD58+CB9+\/aVOHHiiAUc+fHHH03qlfe81uPAfPz4Ue7fvy\/37t2T58+fS7FixWT69Okyb948qV69urx+\/Vqo4z1TZE5PPA7Mo0ePpFWrVlKxYkUpVaqU5M2bVypUqKCocOHCMnr0aDl8+LACLF++fLJ161Zp3Lix3Lp1y5wZMumtHgcmevTosmjRItm4caMAxPr168WiXxQtX75cibVZs2ZJ27ZtJW7cuJIiRQopXbq0DBw4UHGTSfPk8dd6HBg9QsQYYgugdFnMmDGVePvhhx8kefLkqphzPz8\/CR8+vLqnCr+DP6YBEylSJHn79q1MnjxZiSlE1dy5c8ViFykQbt++bZ3+06dPy4MHD+TXX3+1lvn6iWnA\/PLLLzJo0CAluqpWrSqVKlWSGzduKJFVt25dCQwMlCNHjkjx4sWlZcuW0q5dO\/njjz98HQ\/r+DwODOLrzp07qgMRI0aULl26yM6dO2XPnj0yYMAAiRw5sqROnVpmz54t+\/btkw0bNsjevXslXbp06pnv5Y\/HgWGPsnr1amnatKkcOHBAXrx4oeYaExpLLWvWrJIyZUrFKZaNp7LW0DFwFXVU5e\/gz1\/AeHCgP\/30kzRq1Eh69eol69atEyY9UaJEki1bNiXKsNSqVKmiRBfWG1SzZk0pUaKERIsWzYM9NfdVHgdGDxdTuE+fPnLo0CG5dOmSov379yszGYuNvQ2eAIiN54kTJ5QBoJ\/39aNpwDia2J9\/\/lkBcOHCBWuVu3fvqjJMZ2uhF55ce\/RGhmy8ImXGH5Uo7bdJ+v771HVIuup1wGBG16lTR5o3by558uRRVK1aNaldu7ZEiRIlJGP0yDOA0nLhaQsQl2X3xSfqnZQN2XhZAaUKvuGP1wFD39E7O3bskAULFgh7GzwDBQoU4JbX0h4LGBoQ+05SvvDQX5ao\/T1H16YCg5XVtWtXweLCJB43bpxcuXJFXRcsWFBq1KihOAWDgDrUdzSQkJa3sKxyWwppOwsPftkQB9UGnBNUuaMy04DB1T9kyBDl9qdz8eLFE0scRmbMmCGjRo2SZcuWKZo6dapydmK1scehrjtp5\/\/OiC2hG0JCcIV9v1pG2CbQnysaSJOox+xvB3ttGjB4mT99+iRwRrhw4ZQbBjOafQ1KPkaMGALhM2vfvr1Q99mzZ8EOJiQ382b8r0Anh5UT6NFIPwkJ\/SdK+K9eP+6Fn0BP\/KfL5Ifpv7ofXIFpwOD3AoBXr15Z+4eoAjA2odZCy8n169fl5s2bljP3\/x9fLYVArrbcpWjCYJuoliV2sPftb5oGDH4vFHqLFi3k3Llz0rFjRxWjIXBWrlw5YdOpCV2TMWNG5a6xH4C3XFfLEktyJ\/7zS3dePZTfdg2ViJu7S+znp6RL0QRf7jlxZguME9XdWwUH5Zo1a5ROYXfPOc7MVatWycWLF9Wmk80nfjRiMogz9\/bAva2tapHBAkBCQazFenJSImUqK\/UDJ0uF2Pe\/OWRhKjDbtm2T3r17S6xYsYTIZdGiRQWDoEePHso6I\/wMnTlzRnHUw4cP3TuTodAanHGsZw6pk+E3Wd+9hASUTyu\/\/\/67PH78+JveZhowT58+lcWLFwui7OrVq8oIICED\/9nJkydlwoQJCiwAYy9TqFAhr95g2s46Fid5C+EtwT3K0aOMl3NnyTRgCJJFiBBBiFoePHhQOTGZ\/EyZMikuIlZDOFkTjk17o8DZQbqzHjGigIAAa5OY+HB5lixZJHfu3LJ582bBqMG1BEBURJ9GjRqVU6fJNGBwvbx7907atGmj4i4RLbGZJk2aKFk8ceJEwTAgUUNTaG0wnZ0pIqgsEjgcDtDPES8iwMcGGU\/4tGnT5PLly4q7z58\/L1iZ1Gd8+hlnjqYBQwSzf\/\/+gh9s6NChkiBBAkHHDB8+XJYuXaqCZwTQNDFo2\/wAZwbnzjpwQebMmQWL0bZdXEeEJAhn4DHHG3727Fk1FhYYxkz27NkF6WD73D+dmwYMHWMvQ2QSNn\/z5o1yWAISMf6xY8dK9+7drYS4CI0NJv1whvA6FClSRBC1uj6bYZS6jhMZhqHMfHQmILGYtmzZIjynn3H2aBowyF84BjMY3VK+fHkV98eD3McSpyEEnStXLtFEZJNV6ezAPFUPTkKfuPt9pgHDSkP+kl\/WoUMHge3nz58vrELEAW7\/kiVLiiY8znCYuyfAlfYwgxFRZPjQDhk+6BfcSFy7QqYBY9tprC2CYbD\/y5cvVTYM4sy2jjeeG4ahQuPoGawz3EaMI0mSJC5392tgXG7SuQaQy2wsMYsTJkwoWDN4la9duyZHjx5Vrn\/El7dYZY5GhQsJ\/YOpTAi8QYMGEj9+fEfVnS43DRjDMIRYTLdu3dQeplmzZspyYUOpXTLsb7zFKtMzij7EctTX6D1SsMj4oa+IXH3PlaNpwNBpfF+YluwF0C3JkiWTY8eOCUaBrUXGOQYBkUxEBs\/6OpkGDFYXm7WRI0eqZD8clRDAYDqzocuRI4cgznD7U598NEDzdVAYn2nAkKKERYNuYUetCZEGMMOGDVNZ\/v7+\/jJmzBjBS8A9LDk67utkGjD4jzB\/CY7ZTzLAAIQuxyGIqGPvo8t8\/ehxYNi9s4vv16+f+rKsbNmygo8MPQIR4ydFFt8YxgFl9erVE9wh6Bg2oL4OCuPzODBsyurXry9t27ZVSh5vLCBxDXEPoObMmaMcmez8cdNUrlxZibaePXvSb58njwODCwOXP65zkshR\/oZhqMQLki9wVBJq5ruZ7du3K8OAbzTr1q2rwMA8VSc+\/sfjwGBdwSFYXHAF7nAUv9Yf6BL2MnzNjKJPmjSpigCSQQNoPo6HdXiOgLFWcPcJ1hjijKRxXP0NGzZUMRh0D+8idoFhwH3A41sZPqaFe3iWOt8DeRwYJpU8Mo6aAENzDJYaVhk+J0TbVUvYGfHHR0x+fn7qK2ftpuFDJ93Gv\/WIx2DmzJnKeYtLB53LWEwBhhc7Is0teJvx0pKZmT9\/fvX9JUDg9tBE0oZuB88uZrW+\/jccEdsrVqwIMl5jCjB8E5M2bVoVVCK6x3cxHMkj49v+VKlSCf4orleuXCl4bwmowTn2Ez5p0iT1ZQBxHbjq1KlTggmOUxGTGzc8z5CRkzNnTsVxZHYGBgaqH4FglZKJQx1WK\/ltgMwPQsCZtEOSCJPIfdrkywNSdtF7iFfcRFiOlBFIY1Ex4Xw3qiUBehPiPZrIH2Ah4lXXZfrocWAQT0w0+WK2BEvjfmFSsNaYFIgUWiaaZ9BHgAUBFNk0DAQjYeHCheoTdBLTx48fL7t371ZfpWH18a0Nlt2SJUsUyEwEE82zQRHPEnYg+sjeCZ3IAqEuG2KMFeoQIOPHIvD14TZCDwIeYyEqy7emfH0NeLiaGAttaKI+ezZ9zZEfm2B8HgeGlzsiDABSfjCNZ8+erZLKdXI5oE2ZMsWaBHj8+HFJkyaNagpRx8rjSwHEHIAyOPIJ+BINLiIbB0AMwxA8xLiD1MNB\/GFiNdBYhwBNKIKqGTJkEBLgMdt5P+KTRAwmHf1ICACdAXdqpyzmP++zj9MAGOW0q0mP0WuAwRTGSckkkHWCyECssNLIqGHQ9kaDHow+sj8ijQgw4EayVAghMHg4UdfD3YP40df2R4J1nTt3ti4CuAfRZ19PX9MWySX6+smTJ+pXPMj02bFjhyBGsTAZg67zT0evAYaOokPYt7D5RO\/0scT+OTJAVjHWG\/UcUUJLwA2uAxCAWLt2rYwYMUIFrhA1RBgph6sAkcgpKx8rkHJkPm3DnXABK5p9F9miiDTuBUUsBgAAIBYV5j2h8\/Tp0ys9hu6Di+2fJVDoSKR6FTC2HSdWoxWnbXlw54gqflYLTsOVQ5YKn60jUlDyJHxgXAAQXITY4ZoUIzJZ0EWAhahjsjkiJvHPce7o3eQlGIahsnxwHbEHQ9zB6drI4dr+ed6BnmRR2N\/zKmCQ16x2rCBkNB4CrDV+gYkVW6ZMGfv+q98LYPL1DZ5DabPi+VSQxA7DMFQ+mI6I4hRlMgzDEHxzcOOmTZsEB+rgwYMFLmLVUw4h1igDHKxF\/S7eSxkgEyKHu3kvuWeGYahf\/cAAoN+GYejHrEcsODgLLqMN+kt7VAgeGGp4iLB2MHmZNFYXq51JQByQ98t9gPNQd1x+DSY4AMGBeMaDahAwatWqpXQQC8W2TjjbC7PP8Zuh4GfNmiV8uYzZrIkVjFnsjj5iTaG\/3NGWozbYFmDMwIHBKX2sPPZFhvF3jvIaYLDK4BIUc1DEPeo4mghfK\/caYHxtYl0dTxgwrs5gKD0fBkwoTayrzYYB4+oMhtLzYcCE0sS62mwYMK7OYCg9HwZMKE2sq806A4yr7wh7PgQz8H8AAAD\/\/4Z7GfsAAAAGSURBVAMAwJ0vW9tBHtoAAAAASUVORK5CYII=","height":61,"width":102}}
%---
%[output:70f94d54]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGYAAAA9CAYAAABflhxWAAAQAElEQVR4AezcB5CVxbIA4D6YxYA5XxFzRkyYxRwxK2Yps6gPLRXDU8FYxipzDvjMscSIEUxgzjmBimLGUIqK8vaby5z3c87ungPrwn33LkXvpJ6eme6Znp6e+U+7sRX\/vvzyy7E777zzWOHw4cPH9unTZ+zo0aPHih944IFjf\/zxx4oabcnW4EC7qPg33XTTxZRTThkffPBBzDDDDPHzzz9HgzDizz\/\/jAZhxa+\/\/lpRoy3ZGhyoEsyMM84YPXv2jL59+8ann34ayy23XGyyySax7bbbxpJLLhmzzjpra\/SjjWYFB6oEo7xbt27x0EMPxfLLLx9777133HHHHXHnnXdGv3790mqC0waty4EqwXz11Vexyy67hFDTU0wxRXTs2DGmmWaa2HXXXcv5ytqg9ThQFgxBrLPOOtG1a9cYOnRoCjt16hQZ1lxzzWjfvn1MP\/30rdebNsplDpQFM+ecc8bgwYPj2WefjS222CKFH330URThqquuSgZBuXZbpNU4UBZMbmGOOeaI888\/P4Q5r7mQ1Ub1AfFLL7007U2vv\/56c9UmqOyII44Iq9mqRtfep50JIjIByNrRnnabq6YcHvzm8Camz2XBIK6RrLoaC5XDK3aCSX3jjTcGEC+W\/X+Nn3nmmTHffPMly3RyjaHdDTfckGYjq+u2226LBRZYoNwXpvPdd98dl19+ecprOHgGlffII4+kvUdolVgt4L777guD+umnn2KrrbYK5aniuD\/SBC6UVZxJmY5y0Ngk+Prrr+Pggw8O9LVjxmYa6gCrSR76VpU8QD0Xy0wwbSgDuU\/yn3\/++Vh77bXLals7cDLARRvPHCk23HDD0GYlnjR6lX2uHCta+luEdscff3x82nBe+eOPP6J3796p7NFHH42ddtopMaB79+5x5JFHpnw4KdLEHwM96qijIgt0gw02GA\/TmYjgHV4VPP3004Lo0qVL7LfffjFixIhkeDA+FOywww7jWYHU64UXXpjoa8dZ6\/TTT4+ll146XnvttTCJ1EMXoy655JKAp2ymmWZSlACz0LYqlMHRPoZLY\/YiiyyScDHt4YcfTrTtt85zJ598cjrXiRuPcodvgjKJMx4BI1LZZ23lscI3ybQDN0M7xBEiCMgrrbRS2ORJlYAYA4ceemjCZxw0uB9SfGL+sOgw44knnkgMF2IqWm+++WbkFWlVrrzyyumAi1HKGwOqkwoF119\/fVqlVhNcwhGuscYaaeZbAdIATcyXhwYck+nBBx8MID7XXHNBTR4QNLMGOPvss5ORpI8JYdwfkxAfTT4TlJDGFY0XfPzxx2GsxocGfMLFCzzPyO3yzMgZv\/32W+qMVbLQQgsl1aWicjPtm2++EZ0owATM0DHMMRGkuXwMvkg09yuvrmJZMU5dUDFmXTG\/sl6mV8RRR11Mr2w\/4+25555phUqb6fC1KV0EK5SqdNwg9GJZY3GCQ6sp\/PLmnytbku3atUtqzL4BqAFnmIzTktDsVN\/MHDVqVEibnWap\/AyZsY0xNONgBjWSVz1VlulU1sv0cl0hNWKWZ7Aa5Bdh2WWXjVdffTWpSkxUhqnUnniG\/v37p6g+oKdPKaOJP9QnvAxWvYmb0cuCsawsr\/feey8WX3zxOOSQQ+KVV16Jzp07x9tvvx2\/\/PJLOv3PPvvsablnAhMaWoXUl8EJpYH4TTfdlFSclUk\/W6mWelNtEGiHDh3KxdRXnvkEIy6PiqAqMiKaaJsc8jDZ7BXyC6pngiqzOqgm48c8DCV8bSvPoL0cz\/3P6WKYx6o\/+mVyWWlVe0yxkkYNlPSHDRuWTvn33ntv3HzzzbHgggvGhx9+GAsvvHC88847xWrjxa0AGdSDgYoXwaygvuQJpYHZa\/8xK4FyVqIJI54hD4waAvYlQsbYl19+Oakdg0bDeOAQBHWZaaCJNuGrR0XBtU\/AJbS8wuSrhx5c9M4555ywkjZpcO5SW6wyDl68M+5sWCijsot9ZrAYK5ragq\/+AQccIKsM7YoZOnz77bcni4wEzTaDfOCBB+Kxxx4rewEIyxI0EEw1k4C4Dlv6ubzcUiGiTeXCnK0uGvLB4AYvhP4op2JyuognHw344LLLLktqBx14xTLmMloZ0EZTPQBXmXzCNm4zWrqIB9e44QqljdeqynhCfVAGR19yWp+LaTjy0CtCWZXlTJVWXHHFYGqy0PbZZ5948skno5apnOv\/q4RWqxlOFemTFWBGV6ogZZVgf2CYmN2VZZMqXSUYF2Xbb7993HXXXfHCCy\/E7rvvHpa9pWxZF1VCsZPMaMLMjCiWTY64mVqcjWalmWwF1OoPHLjq1MJtrfIqwVQ2ZAXZ7Lj\/P\/nkk3STWYkj\/dRTTyW3jHgbVHPgk+9GV2c2k1MUTEJzdWyf2WabbdLt5UknnZTCgQMHJiNglllmSXjFP5Y9186+++5bzE5x6qQNOkXX3Y6JWQ9\/PDqfMiTqEVKVYGx49HOPHj2Se4RdzivQmEBw3t7DynCJNu+888qqAh6EyQmszNZs\/3\/uGhgZzrzuvuh94d0JdjrtjljrmFti4V43xOglupf58o9Zpy3Hm4pUCYY\/ikXWnDCKxJwFvANwkVbML8aZi5MT5p9\/\/phi5nnGgxF\/zhIZhn4zXRTh1vdKAc4eOjqK8F\/3j4oM29wwMsCKFwyLLft\/Vob97hwZfR\/5JsFlz42Ke97+OV4c8X9q7B+z1BYK3lUJRuaEgE3yvPPOS+cb5wFnCt6CemhY0uCpD0ZFhpueHxlFOGPgsMjQ66a3owjdL3o5ikBNFIHqAJhXzBfvfnFD3XFQpCl+xsCPG9r8eLx+6NNTHzb0cxzoN6hnnFbImgt3iGnfGRADeq1QT5VosWBYLqwfQKUxNdnsjbVuIAaImRiGQaAeJmGWukUoMkoc\/SI01oeW5GFwBozOsPPKcwfos\/FCcdHOSyYYcNAKAb47t1u88t+rBYFM+87ddTdfJRibP7e7sEhFurF87ovDDz88uHOYyg5b9qliXXGznhDMSEyU93dAZlQOM7NyiGFbLjlDYpw45mXITMwhRmbAzAyYC3JaiNEZcv0+G3cst7PmIh0CTOwYy4LB+McffzzdO5xwwgkpdPGV4dZbb41TTz01OTeLjQ0YMCCcfQhtyJAhSaVxdRRxCMWMz3mZiRgF+hRmmkEOGDfbcogRGTCoCDk\/h5lZOUSv7wazp1ks3qeBeRm0XQSMzJD7KMz9npRhWTCsq+eeey7ADz\/8kEIumQzvvvtucmwyDoodXGWVVeKggw4K55wxY8YkDwFvdMYZM\/viSV9LG2SfBiFkJmIU6NPArKYYhFHqZUDnPwHKgsHMPn36pFVhM7c6TjvttCjCRhttFKVSaTy+LLrooul+nBrjJfW8lnc6I41eYqscjU0XmTZ2XGxsuCyalPDZZ59N8jYbG1+ZEXVEyoLharF5f\/vtt+mGzvXpscceG0U444wz0jvmxugyAlyA8RK4TYTD0rJixOn803usEJPDbJ6\/wVyeHO1WtokP9UJZMFNNNVU64bdv3z6oJ+77SpAPr0jcyuKOkWev8byWw1D6zAazUwiOalBhwjaojwNlwWBq165dwwkfczfffPOoBPnwiqTdbD7zzDPJh8aAGDRoUHCBM1uz9WV\/sFcU67XFm+dAWTAZze0bxjbm35KvPOMKPTofOXJkuiJWzq2+ScMFkrIMXz1za3ru1BjNSZG3\/vrrT9b28xgzP+oJqwSTXd4OjBm4\/z3O8DCh0iqzp5x77rnJr8aiYwRQd1YJs5YFNvT608uXbJnmf2pYj1DgVAlGZiXwhbmXeeutt2JCX8kQEKik2ZZungN1CQaJYcOGpYcSzivSbdC6HKgSjD3EXpH1Yg45Ju0dVk\/rdqmNOg5UCSbvMV7EvP\/++2lv8KTJaw\/CUakWuGa+7rrrktntOqB\/\/\/7JaqtVr6Xln3\/+eXpIwqx3n+RgWUnTXVOebEJ7YiVOa6W9muFXbMyXWNlmlWAg8JlxSgqlr7zyyrBa3njjDcma8NJLL4WHeAMbbj3zC5tK\/1lNIhOIwB101llnJcEwQlzcORB7WVokNXz48PRIPhsfDsbF8taIO0Z4v7zXXnsFC7aeNqoEw1t8zTXXJFcMMxMRV8Z77LFH+m5GI\/KaA0JYddVV05mI1UbILLvm6rS07LvvvosvvvgifQmHlpc+vg7gyZDOwHiZeeaZc3KShR40em9WeQ5sqgNVgsH4MWPGROfOnct+MRu+VyeWIME1RSznO\/lzg+S0R4KMh5xujdDLyamnnjp9jog+359JURQMFUswvOfev2FUvVoAzYkFwnA450nhS6yHTpVgzCanf25+HmdEfONPN+cXM\/JqgY9pa+H83eXaNImaovv777+nF5RXXHFF+p7FVwz9+vULK6upOpMrv0owBnfcccelR35LLbVUOtFbhjZzh0zSr9XZjh07pue0GY8h4QlpTrdGONtss6W7Is5Y9IWjR4+OueeeWzKBsVHJeTXTCiZbvXo\/EZlEf6oEY5U4ufMQv\/jii+nhH1+YTZyLv55+rbbaauEdMbUHxDGhnroTi8OaxHB9RkPIPWT1SwNWEfVlokh7g0146kn\/K0GVYDCSCWkjpacN2NmlVBr\/Hqa5QXTp0iU5MjfddNPYeOONg36V11ydlpbR3UxR79sYHr4ckJYvD3gs7lKPa6lrg8O2b9++QTsUhdfSfvxd9asEY4\/xAJupadY5cGagi+03tRovlUpBZbgOsNp22223siFRq25Lyr1r82WCr+A8CJFGj+kMxE0S\/XIV7tp8mWWWkT1JgMHh4Tv1WavBKsEwO\/1Eie82DjvssPCOOYNvZpTXItpW3nIOVAmG99is8xlCJchX3vJm2yjU4kBZMNQU89HXYz169EifU\/uwqAjy4dUi2lbecg6UBWODtxn6ZO2CCy4ID8srQT68ljfbRqEWB8qCcTDDdKYye\/+WW26JESNGpK+WuVhs4lSZw1ktoqw6B9JaePWUW6HbbbddsvIYIbXq8E7sv\/\/+6QBZC7e1yvWBw5eT1DvwiWmnLJhcmTPwxBNPTANjKsv32oOHePDgwcGtIW9SgcOfE7uPp3J\/WrNt46vH8qynD14d5U8I68Ev4lQJhtXlcR9\/11prrRW8yn\/99VfsuOOO6QszH4eaBQ6i4ksssUQ4eDKJud2VeVjugblVY\/YcffTRAc\/HoA6uBl\/shLh3WA5\/\/GpdG84YPNtWiJ\/78CxKmTTcDMU6+mll5zKfXfhUUbu+dDPhmuqzPlpljgj2VO35ok5\/Ad+a8cAD9Ywn90OIJ2jYoxdbbLEQ4pWyDOhaZXgmr0owBsAh6CrZCdmBjMuave\/LZSazWeCswKQeMmRI+gZ+7gbXh+eyyjwsN1t4p70H0BDvsm9tqEPnCHkZdMr+5mMpdz8XXXRR+k0avwOgbV\/1OnMUV4yDMAY5c5lI+ulrYrSU+frNB7333HNP+JiXEJvqs36oQ236ZUMCPYioDQAAA1JJREFU5ILSXyrcikUXXj3jgVcJPCdcWt52c3WZBHhdiZfTVYLhkbXXuKdgpemUg5rXMAwDPzuiMlc+Bjo1G4iDqfwi8ORyxzjcOVRRiWY2BhXxWILa8VzKXsdLwLfmo9wiXjHuPsVK3nLLLdPPQRJe7969wx7Jn+fLY56LeeaZJ7hcuF6a67M6vgkSmmw8wfqr32gJearrGU+xnzm+7rrrpt\/MQR89E4d2yuWVYaOCufbaa9Nma1mZyYi4LKMaMgGdtLy5P3r27BlmbS7LIZWlcd+y2wiBGWffyDhCb6VLpVJ6\/xwN\/0qlUpgMBN6QbPS\/2U0IXP0QDJgKZLhww0w7bfUHQs31uVjHFQVBoAv8UkipVEr7az3jUacSqOhS6Z9uLZNYfytxiukqwXz\/\/ffpV5C8X\/Yq5uqrrw4DMvNKpX8SRoB1ZgZybfAUYIr8IpRKpWTVuck0w4ErasIu4mGiDRfIt8Qxh3qUbgwq6+g3XU4lNYYvr54+w6M19EEcWG3SpVJ941GnEmwPOU\/cWE2GnGeSZW0kr0owZjldbclSFZDMSipGXAeFOuvaVgMum2yWuUy52U7NUX\/2HhdwPqLNvxsAJ4NH6GjZsNFwNW1PcAuZcSpDdag\/3gl17F3q6Wslbk7X6nPGo3buv\/\/+9BDd9QH\/lv536NAh6hlPplMM7XcmOjr8eFQ1\/mQcK51XJRsF7XJBDu0vNnD7C2tLKC2fAaCTZiY15hMNDchjFFx88cXBmqLy1Bs0aFD6vTCqkHXD27zZZpuFrwZye0IdOuWUU9JdPKuFV9hZiDdYeWOgjofv9jlCMvBjjjkm\/d5NY\/jymupz5V62+uqrh74Cj0nwwSoyOf3WQa3xaKsSaJx+DZdyeEh19+rVKzl2jRPAtxezWu1vVYKB0K1bt\/SykuqhqqTl6yRLjKBsjCwlL2l4BLbeeuswI+AyB+W7jjYrqEVpaozX2QDRK4IO+9BWm6w2dJQ355HNdViPzjn6ZG8wUdRTv5hW3lifCaBYx+p1NeC+Rp\/XW2+9tL+USqX0jqGe8Wi7CAwQh3Z9NYFNLOWMLCBuPKxCE75KMFQZU9HAbNadOnUqv\/v13qzyLIHgvxuYjFaXfYuZbA9l4npAP6nG+r8AAAD\/\/9pZBCwAAAAGSURBVAMAWyR8gPsYuCsAAAAASUVORK5CYII=","height":61,"width":102}}
%---
%[output:5339cedf]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAGYAAAA9CAYAAABflhxWAAAQAElEQVR4AezcB5CVxbIA4D6YxYA5XxFzRkyYxRwxK2Yps6gPLRXDU8FYxipzDvjMscSIEUxgzjmBimLGUIqK8vaby5z3c87ungPrwn33LkXvpJ6eme6Znp6e+U+7sRX\/vvzyy7E777zzWOHw4cPH9unTZ+zo0aPHih944IFjf\/zxx4oabcnW4EC7qPg33XTTxZRTThkffPBBzDDDDPHzzz9HgzDizz\/\/jAZhxa+\/\/lpRoy3ZGhyoEsyMM84YPXv2jL59+8ann34ayy23XGyyySax7bbbxpJLLhmzzjpra\/SjjWYFB6oEo7xbt27x0EMPxfLLLx9777133HHHHXHnnXdGv3790mqC0waty4EqwXz11Vexyy67hFDTU0wxRXTs2DGmmWaa2HXXXcv5ytqg9ThQFgxBrLPOOtG1a9cYOnRoCjt16hQZ1lxzzWjfvn1MP\/30rdebNsplDpQFM+ecc8bgwYPj2WefjS222CKFH330URThqquuSgZBuXZbpNU4UBZMbmGOOeaI888\/P4Q5r7mQ1Ub1AfFLL7007U2vv\/56c9UmqOyII44Iq9mqRtfep50JIjIByNrRnnabq6YcHvzm8Camz2XBIK6RrLoaC5XDK3aCSX3jjTcGEC+W\/X+Nn3nmmTHffPMly3RyjaHdDTfckGYjq+u2226LBRZYoNwXpvPdd98dl19+ecprOHgGlffII4+kvUdolVgt4L777guD+umnn2KrrbYK5aniuD\/SBC6UVZxJmY5y0Ngk+Prrr+Pggw8O9LVjxmYa6gCrSR76VpU8QD0Xy0wwbSgDuU\/yn3\/++Vh77bXLals7cDLARRvPHCk23HDD0GYlnjR6lX2uHCta+luEdscff3x82nBe+eOPP6J3796p7NFHH42ddtopMaB79+5x5JFHpnw4KdLEHwM96qijIgt0gw02GA\/TmYjgHV4VPP3004Lo0qVL7LfffjFixIhkeDA+FOywww7jWYHU64UXXpjoa8dZ6\/TTT4+ll146XnvttTCJ1EMXoy655JKAp2ymmWZSlACz0LYqlMHRPoZLY\/YiiyyScDHt4YcfTrTtt85zJ598cjrXiRuPcodvgjKJMx4BI1LZZ23lscI3ybQDN0M7xBEiCMgrrbRS2ORJlYAYA4ceemjCZxw0uB9SfGL+sOgw44knnkgMF2IqWm+++WbkFWlVrrzyyumAi1HKGwOqkwoF119\/fVqlVhNcwhGuscYaaeZbAdIATcyXhwYck+nBBx8MID7XXHNBTR4QNLMGOPvss5ORpI8JYdwfkxAfTT4TlJDGFY0XfPzxx2GsxocGfMLFCzzPyO3yzMgZv\/32W+qMVbLQQgsl1aWicjPtm2++EZ0owATM0DHMMRGkuXwMvkg09yuvrmJZMU5dUDFmXTG\/sl6mV8RRR11Mr2w\/4+25555phUqb6fC1KV0EK5SqdNwg9GJZY3GCQ6sp\/PLmnytbku3atUtqzL4BqAFnmIzTktDsVN\/MHDVqVEibnWap\/AyZsY0xNONgBjWSVz1VlulU1sv0cl0hNWKWZ7Aa5Bdh2WWXjVdffTWpSkxUhqnUnniG\/v37p6g+oKdPKaOJP9QnvAxWvYmb0cuCsawsr\/feey8WX3zxOOSQQ+KVV16Jzp07x9tvvx2\/\/PJLOv3PPvvsablnAhMaWoXUl8EJpYH4TTfdlFSclUk\/W6mWelNtEGiHDh3KxdRXnvkEIy6PiqAqMiKaaJsc8jDZ7BXyC6pngiqzOqgm48c8DCV8bSvPoL0cz\/3P6WKYx6o\/+mVyWWlVe0yxkkYNlPSHDRuWTvn33ntv3HzzzbHgggvGhx9+GAsvvHC88847xWrjxa0AGdSDgYoXwaygvuQJpYHZa\/8xK4FyVqIJI54hD4waAvYlQsbYl19+Oakdg0bDeOAQBHWZaaCJNuGrR0XBtU\/AJbS8wuSrhx5c9M4555ywkjZpcO5SW6wyDl68M+5sWCijsot9ZrAYK5ragq\/+AQccIKsM7YoZOnz77bcni4wEzTaDfOCBB+Kxxx4rewEIyxI0EEw1k4C4Dlv6ubzcUiGiTeXCnK0uGvLB4AYvhP4op2JyuognHw344LLLLktqBx14xTLmMloZ0EZTPQBXmXzCNm4zWrqIB9e44QqljdeqynhCfVAGR19yWp+LaTjy0CtCWZXlTJVWXHHFYGqy0PbZZ5948skno5apnOv\/q4RWqxlOFemTFWBGV6ogZZVgf2CYmN2VZZMqXSUYF2Xbb7993HXXXfHCCy\/E7rvvHpa9pWxZF1VCsZPMaMLMjCiWTY64mVqcjWalmWwF1OoPHLjq1MJtrfIqwVQ2ZAXZ7Lj\/P\/nkk3STWYkj\/dRTTyW3jHgbVHPgk+9GV2c2k1MUTEJzdWyf2WabbdLt5UknnZTCgQMHJiNglllmSXjFP5Y9186+++5bzE5x6qQNOkXX3Y6JWQ9\/PDqfMiTqEVKVYGx49HOPHj2Se4RdzivQmEBw3t7DynCJNu+888qqAh6EyQmszNZs\/3\/uGhgZzrzuvuh94d0JdjrtjljrmFti4V43xOglupf58o9Zpy3Hm4pUCYY\/ikXWnDCKxJwFvANwkVbML8aZi5MT5p9\/\/phi5nnGgxF\/zhIZhn4zXRTh1vdKAc4eOjqK8F\/3j4oM29wwMsCKFwyLLft\/Vob97hwZfR\/5JsFlz42Ke97+OV4c8X9q7B+z1BYK3lUJRuaEgE3yvPPOS+cb5wFnCt6CemhY0uCpD0ZFhpueHxlFOGPgsMjQ66a3owjdL3o5ikBNFIHqAJhXzBfvfnFD3XFQpCl+xsCPG9r8eLx+6NNTHzb0cxzoN6hnnFbImgt3iGnfGRADeq1QT5VosWBYLqwfQKUxNdnsjbVuIAaImRiGQaAeJmGWukUoMkoc\/SI01oeW5GFwBozOsPPKcwfos\/FCcdHOSyYYcNAKAb47t1u88t+rBYFM+87ddTdfJRibP7e7sEhFurF87ovDDz88uHOYyg5b9qliXXGznhDMSEyU93dAZlQOM7NyiGFbLjlDYpw45mXITMwhRmbAzAyYC3JaiNEZcv0+G3cst7PmIh0CTOwYy4LB+McffzzdO5xwwgkpdPGV4dZbb41TTz01OTeLjQ0YMCCcfQhtyJAhSaVxdRRxCMWMz3mZiRgF+hRmmkEOGDfbcogRGTCoCDk\/h5lZOUSv7wazp1ks3qeBeRm0XQSMzJD7KMz9npRhWTCsq+eeey7ADz\/8kEIumQzvvvtucmwyDoodXGWVVeKggw4K55wxY8YkDwFvdMYZM\/viSV9LG2SfBiFkJmIU6NPArKYYhFHqZUDnPwHKgsHMPn36pFVhM7c6TjvttCjCRhttFKVSaTy+LLrooul+nBrjJfW8lnc6I41eYqscjU0XmTZ2XGxsuCyalPDZZ59N8jYbG1+ZEXVEyoLharF5f\/vtt+mGzvXpscceG0U444wz0jvmxugyAlyA8RK4TYTD0rJixOn803usEJPDbJ6\/wVyeHO1WtokP9UJZMFNNNVU64bdv3z6oJ+77SpAPr0jcyuKOkWev8byWw1D6zAazUwiOalBhwjaojwNlwWBq165dwwkfczfffPOoBPnwiqTdbD7zzDPJh8aAGDRoUHCBM1uz9WV\/sFcU67XFm+dAWTAZze0bxjbm35KvPOMKPTofOXJkuiJWzq2+ScMFkrIMXz1za3ru1BjNSZG3\/vrrT9b28xgzP+oJqwSTXd4OjBm4\/z3O8DCh0iqzp5x77rnJr8aiYwRQd1YJs5YFNvT608uXbJnmf2pYj1DgVAlGZiXwhbmXeeutt2JCX8kQEKik2ZZungN1CQaJYcOGpYcSzivSbdC6HKgSjD3EXpH1Yg45Ju0dVk\/rdqmNOg5UCSbvMV7EvP\/++2lv8KTJaw\/CUakWuGa+7rrrktntOqB\/\/\/7JaqtVr6Xln3\/+eXpIwqx3n+RgWUnTXVOebEJ7YiVOa6W9muFXbMyXWNlmlWAg8JlxSgqlr7zyyrBa3njjDcma8NJLL4WHeAMbbj3zC5tK\/1lNIhOIwB101llnJcEwQlzcORB7WVokNXz48PRIPhsfDsbF8taIO0Z4v7zXXnsFC7aeNqoEw1t8zTXXJFcMMxMRV8Z77LFH+m5GI\/KaA0JYddVV05mI1UbILLvm6rS07LvvvosvvvgifQmHlpc+vg7gyZDOwHiZeeaZc3KShR40em9WeQ5sqgNVgsH4MWPGROfOnct+MRu+VyeWIME1RSznO\/lzg+S0R4KMh5xujdDLyamnnjp9jog+359JURQMFUswvOfev2FUvVoAzYkFwnA450nhS6yHTpVgzCanf25+HmdEfONPN+cXM\/JqgY9pa+H83eXaNImaovv777+nF5RXXHFF+p7FVwz9+vULK6upOpMrv0owBnfcccelR35LLbVUOtFbhjZzh0zSr9XZjh07pue0GY8h4QlpTrdGONtss6W7Is5Y9IWjR4+OueeeWzKBsVHJeTXTCiZbvXo\/EZlEf6oEY5U4ufMQv\/jii+nhH1+YTZyLv55+rbbaauEdMbUHxDGhnroTi8OaxHB9RkPIPWT1SwNWEfVlokh7g0146kn\/K0GVYDCSCWkjpacN2NmlVBr\/Hqa5QXTp0iU5MjfddNPYeOONg36V11ydlpbR3UxR79sYHr4ckJYvD3gs7lKPa6lrg8O2b9++QTsUhdfSfvxd9asEY4\/xAJupadY5cGagi+03tRovlUpBZbgOsNp22223siFRq25Lyr1r82WCr+A8CJFGj+kMxE0S\/XIV7tp8mWWWkT1JgMHh4Tv1WavBKsEwO\/1Eie82DjvssPCOOYNvZpTXItpW3nIOVAmG99is8xlCJchX3vJm2yjU4kBZMNQU89HXYz169EifU\/uwqAjy4dUi2lbecg6UBWODtxn6ZO2CCy4ID8srQT68ljfbRqEWB8qCcTDDdKYye\/+WW26JESNGpK+WuVhs4lSZw1ktoqw6B9JaePWUW6HbbbddsvIYIbXq8E7sv\/\/+6QBZC7e1yvWBw5eT1DvwiWmnLJhcmTPwxBNPTANjKsv32oOHePDgwcGtIW9SgcOfE7uPp3J\/WrNt46vH8qynD14d5U8I68Ev4lQJhtXlcR9\/11prrRW8yn\/99VfsuOOO6QszH4eaBQ6i4ksssUQ4eDKJud2VeVjugblVY\/YcffTRAc\/HoA6uBl\/shLh3WA5\/\/GpdG84YPNtWiJ\/78CxKmTTcDMU6+mll5zKfXfhUUbu+dDPhmuqzPlpljgj2VO35ok5\/Ad+a8cAD9Ywn90OIJ2jYoxdbbLEQ4pWyDOhaZXgmr0owBsAh6CrZCdmBjMuave\/LZSazWeCswKQeMmRI+gZ+7gbXh+eyyjwsN1t4p70H0BDvsm9tqEPnCHkZdMr+5mMpdz8XXXRR+k0avwOgbV\/1OnMUV4yDMAY5c5lI+ulrYrSU+frNB7333HNP+JiXEJvqs36oQ236ZUMCPYioDQAAA1JJREFU5ILSXyrcikUXXj3jgVcJPCdcWt52c3WZBHhdiZfTVYLhkbXXuKdgpemUg5rXMAwDPzuiMlc+Bjo1G4iDqfwi8ORyxzjcOVRRiWY2BhXxWILa8VzKXsdLwLfmo9wiXjHuPsVK3nLLLdPPQRJe7969wx7Jn+fLY56LeeaZJ7hcuF6a67M6vgkSmmw8wfqr32gJearrGU+xnzm+7rrrpt\/MQR89E4d2yuWVYaOCufbaa9Nma1mZyYi4LKMaMgGdtLy5P3r27BlmbS7LIZWlcd+y2wiBGWffyDhCb6VLpVJ6\/xwN\/0qlUpgMBN6QbPS\/2U0IXP0QDJgKZLhww0w7bfUHQs31uVjHFQVBoAv8UkipVEr7az3jUacSqOhS6Z9uLZNYfytxiukqwXz\/\/ffpV5C8X\/Yq5uqrrw4DMvNKpX8SRoB1ZgZybfAUYIr8IpRKpWTVuck0w4ErasIu4mGiDRfIt8Qxh3qUbgwq6+g3XU4lNYYvr54+w6M19EEcWG3SpVJ941GnEmwPOU\/cWE2GnGeSZW0kr0owZjldbclSFZDMSipGXAeFOuvaVgMum2yWuUy52U7NUX\/2HhdwPqLNvxsAJ4NH6GjZsNFwNW1PcAuZcSpDdag\/3gl17F3q6Wslbk7X6nPGo3buv\/\/+9BDd9QH\/lv536NAh6hlPplMM7XcmOjr8eFQ1\/mQcK51XJRsF7XJBDu0vNnD7C2tLKC2fAaCTZiY15hMNDchjFFx88cXBmqLy1Bs0aFD6vTCqkHXD27zZZpuFrwZye0IdOuWUU9JdPKuFV9hZiDdYeWOgjofv9jlCMvBjjjkm\/d5NY\/jymupz5V62+uqrh74Cj0nwwSoyOf3WQa3xaKsSaJx+DZdyeEh19+rVKzl2jRPAtxezWu1vVYKB0K1bt\/SykuqhqqTl6yRLjKBsjCwlL2l4BLbeeuswI+AyB+W7jjYrqEVpaozX2QDRK4IO+9BWm6w2dJQ355HNdViPzjn6ZG8wUdRTv5hW3lifCaBYx+p1NeC+Rp\/XW2+9tL+USqX0jqGe8Wi7CAwQh3Z9NYFNLOWMLCBuPKxCE75KMFQZU9HAbNadOnUqv\/v13qzyLIHgvxuYjFaXfYuZbA9l4npAP6nG+r8AAAD\/\/9pZBCwAAAAGSURBVAMAWyR8gPsYuCsAAAAASUVORK5CYII=","height":61,"width":102}}
%---
