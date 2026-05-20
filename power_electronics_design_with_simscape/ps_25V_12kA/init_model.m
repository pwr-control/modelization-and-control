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
diode.rectifier.Cj = 0;        % F
diode.rectifier.Irm = -75;         % A
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAM4AAAB8CAYAAAAhHY+6AAAQAElEQVR4AeydCbxV0\/fA1+lHMleIorz3hKTymjRPpjRpTqE0aFCipDIkr4kGDWgiSUVRIhFJVDQQqVSmaJAGiiIplPfvu9n3fzude9+99913zrnv7T6td87dZ+999l7nrLOGvdbaudLNP4MBg4GoMZBLzD+DAYOBqDFgCCdqlJkGBgMihnDMW2AwEAMGDOHEgDTTJEdgIOwkDeGERY+5aDDgjAFDOM54MaUGA2ExYAgnLHrMRYMBZwwYwnHGiyk1GAiLAUM4YdFjLmZvDMQ+O0M4sePOtMzBGDCEk4Mfvpl67BgwhBM77kzLHIwB3xHO77\/\/Lq+++qo0bdpUqlevroBzyriWyM9q\/fr1UqFCBalWrZqaV9myZaVly5ayc+fOiKc1ceJEWbRoUUT1qVesWLHj7jdw4EABjwcPHpQHH3xQvvzyy4j6MpWOx4BvCCc9PV0+\/PBDadOmjezevVseffRReeWVVxQ8\/vjj6mHfdtttMm\/evONnkGC\/UlNT5e2335YPPvhAVq1aJZdccon6nVXTKFOmjOqf+y1btkzdZujQoXLKKacoHF9xxRWqLJY\/PLOjR4\/G0tS9Nll0J98QzoEDB+R\/\/\/ufzJ49W7p27SqXX365FChQQEFycrIiqDlz5kjBggWzCBXud\/vXX3\/Jb7\/9pub4999\/y\/DhwxVHgitxThkwbNgwKV++vFx\/\/fXy2WefqYHy0r7wwguKc1WtWlU0J1EXQ\/w59dRTpW3btrJx40bZtGmTdO7cWeCC4P7ee++VihUrytVXXy18qLgv5Xfffbcq46PVsWNHVR+uxzNq0KCBrFy5UhYvXqzGUblyZalfv75s2LBBfvrpJyU1PPLII8L4Wh7jrNOmTZNrrrlG9ff++++HGGViFPuGcM466yz10nzzzTeyZ88eOXTokMyYMUOJExz5DWHxAiUGap1H+dFHH0mpUqUkJSVFrrzySjVXRNIFCxbI9u3bBa4A\/PLLL0pkpfyHH35QHApxFUKjZ+ogZr333nuKU59++uny+uuvcyks5M+fX84991yBKHTFjz\/+WIoWLaqIgPt9\/fXXsmvXLpk8ebLAkbgOl9q7d69uIoxv5syZai4LFy4UiHjFihWKMOmDir\/++quUK1dOja9QoUJCv++++66MGDFCjfXIkSNUS0jwDeGARL5Ot99+u\/pqtW\/fXvgq8QXkwY0aNUr4yiYkloMGzVf9888\/l82bN6uvPl\/j\/v37y\/Lly6Vu3bpy8sknK7juuuvkk08+kaVLl6pyuMWZZ56pvt50B054ceHMiHvjxo2TNWvWcClqqFWrltKDZs2apcQ3CPLHH38UPmJwEcuy5MILL5SSJUsG+q5Zs6YwHj54ffr0UXVHjx4tL774oiBqUxGJAe5pWZbkypVLjZ2PH8R7+PBhASRB\/\/mGcPbt26fY+\/z585W4hhiD+NGoUSMlhvAwgr94CYrv44bNS1SjRg31AsFRIRpdAVEp+Lcu10eUe15YCBDgZU9LS9OXQx7hFHAbuI6u9Nxzz8nTTz+txOOePXsqTshHCv2FF556nPNMOA8GDBt33nmn8GxatGghnTp1Cr583Hm4+RxXMQF++IZweFB8vfiy5smTRymvJ510kkIhLxjKLHVUQTb5w3wQuZgvX2asYBAM8NZbbwlfe0RTzinj44JoxvThxIhGEAFEB7fGuMK1UEC9CRMmSPHixZVepeshQtWrV08wXEAIECHPAfGK8THObdu2CZxSt9HHn3\/+WXEedBs4iR6fvu7G0Yt7+IZwvJi8F\/dcu3at1KlTRynTlSpVUiJa3759pXnz5sKLh+gGFC5cWG688Ua56aabVDlEhPiqjSOIchAVR8Qm2nJunxPGBH0\/+oUgMAQE1+PegwcPVqIUOgg62EsvvSQYAyDGiseMBpiuzz\/\/\/OBm6hzdKF++fALnvOWWWxRRrlu3ToluqkI2\/eMrwkHkGDBggKCIfvfdd+rIA6OMa4n+DNAR0E14GTEPYyhAqebLjhiD6MV1gHPKeNGZP\/rOa6+9JuPHjxcIhGvdu3cX6gK6fjCOqPfVV18p5Zz7rV69WtCnzjjjDAEQzxgT3A4dC+4CvseOHStDhgwROBpETf9PPPGE0lMQ8bp06SIA92J8WADpm\/FB3BAfRIpRBz2HeljqGA\/n3JN7MwZ+JyL4hnBQNHkR+Hpee+216gFzrFKlilDGNeokIpITdcwsnkJIPAP0F0zSmuMl6pziNW7fEE7u3LmlRIkSar0CEcQOXKNOvCZu+skYA+AcYw3cCI6F9S3jVjmjhm8IB2sPXKVZs2bKHM3aBq42AAt\/Y8aMESxJOeOxmFnaMOC7n74hnPPOO09QSN955x21Qo5bCroAwMIfmEOu52jAYMBrDPiGcDQitHkVQtJlWHMwpXJNl5mjwYCXGPAd4Zx99tny559\/qgU51hSA6dOnC4tvxjjg5ati7h2MAd8RDgudjz32mOCC07JlS7W+ga\/WoEGDBNNn8ODNucGAVxjwDeEgiuFWAyLwIGD9AEsOFh3WFFhk45quw7mB7ISBxJqLbwgH95o33nhDLayx4EawlUYl5zg7suhGHV1ujgYDXmHAN4SDxQwXj4cffljwzWLNANd74IYbblCuKTh9UscrZJn7GgxoDPiGcPSAcF\/XLia42QC4frCCrd03dF1zNBjwCgO+IxyvEGHuazAQDQYM4USDLVM3cxjIRq0N4WSjh2mm4h4GfEk4JHsgFoUYD8KniSpkIdQ9tJg7GQyEx4DvCIf1HGI1evTooeLcSRZBNhXSRXEt\/HTMVYMBdzDgO8LBHw3vAZJQgALCpjFNE17MNcoMGAx4jQHfEQ7+aCyGkp5II4cYeJJBGJcbjRH\/HnPKyHxHOBAHkYZkbCEcl3j5u+66S8i+AlHllAdj5ulvDPiOcEAXkYfE5ZAlkuR2LIBeddVVXIoayNBCBkmywhAHP3XqVCHVUdQdmQYGA0EY8A3hkL0Tb2giPwEsauRUIwIUHYdr1Akae0SnZHkheQSESHAcVjoSX0TU2FRKGAwc2bNVDm9cIgeWPO\/KmH1DOKQ3euqpp4RskqRE6tChg4oIJSoU0Y0sl2RYiRYrEAlZXPCuRtQjl8Gnn34a6AZfOAMpKiVvtHgoVLenXNBssBRoM1HO7TJbHaPtI5b6lYsVEeDO8gVkwjVnyms3nirfd02WnWm1ZP3ojqosuN\/Aw47jieeEQy5kEnST2A5rGlY0QgfgNKRNAsjXRTI8DATh5o5IR2YcxDIIjfWgb7\/9VrZu3aoSfSOqkZ95y5YtgW72N5osBqLCQQBf+a9uIiUrXSt1qpWVjjWLSoGUK9W1mx+dIyQmtANisr0sot+vTJXZ97eURZ3LyKIW58sLJbcr6F36FKl9dQm5qlY9sVqMEOuBD6TgmG\/kxsmfH3d\/iCjwwON04gnhoGOQPYWXnFxbJOgA8IJGJMOKRt5iPUfyGEM0EJUusx\/JcgnHIn8X22eQlvXJJ59USc0hIC2qkZ2fvnT7vHM7yC+jasUEq7snxdQu1vv5rd2GtGrybq9KMrVjeXmsZWnh97hWV8jTq\/ZL4xd3C7tMBAP7AgX\/Dnee95NpcsrU9pLeJ0XSH6su562dKSdtWy1nFi4mKbPT\/4VntkvK0JWS0vcVSW5+nySXqXbCPbmHftbxPLpOOMTWEB7Ayzt37ly1RwwBawAJ+si4jyEA8QxEA61atVLbfCDOhZo8+k9SUpJceumlqgoGBrgYYdiIePny5VOpWukjVy7Xp63GlBP+tCp\/gfqYtCpfUPLfu1hSB68MO22tl+wZ106JWpubWwKgq5x8XpKc122KFEpbLEXGb1HA77AdunTR9TcIc3O\/fv2EXQnQOezzJLsjXgIo9WSCJN8AmW7YV8VeN\/j3ZZddJoRc0x695uabbxYiSYnzITQB3YbEeohp6enpwU2FsliAkO5Y2tGGOXGMBbxqy1gjvXeLy9LljdsvkjpF8ygCKpH2ocooumX247J5WDPZfH8lRSDoJXtmPCwHtn8lh089R6zhmxUc7f2+\/F63v+y9uIbsOu3if9seE7EZQ7Rw3MOO0w\/XCQdxi5cZDoFYhgUtGBDf0EU43nrrrYrTkFeNurTR80YvqlGjhlJq77vvPl0scK127dqpHGykhWUxlfwF3POff\/4RwE44sPNY4KKLLnIUDSLpC04aST2nOl61ZSzR3LvUvuVy789PyttHHpR5G6vLhU9Wlx2vjpSda5eHFrmSkx1xGs19GWcwBF6OOJ64Tjh67IhOFSpUEMQwLGfs9YJjJ5yldevWaqsJtjKcNGmSSlBIXdro9gS1LV26VO0zg14DMUyZMkWFXjdp0kRteHTOOeeoNRu4EYYD1oXYS8aIahqLoY7RlyNa2cWtfbMGyKGNSyTl8v\/XS5a3XSl3XfmqXP5dmwzFuOhH4V4LzwgHZX7Hjh0CkWA5I\/KTbfMox0iAyAVx4LNGdn04BRa4UKhBbMJqg2Vu\/vz5aksKEo5DJOFEtTaF9gsPOBZIf\/eJmNtufqaH6215sRGTOEYLiFQAIhamX\/SQYAB\/fx9bS0Ev0cq7XS9BxOpbO0nW9qukoMoleZUYl5EexDOnLUe\/gGeEg9MmO3JhMdPIQBTDaEAONUCXY0KGyPRvjnZRDY6ybds2lX+NvmmPmw7GgXCi2g3nHJB9i56NCY5+\/HJM7bifF23RI\/7cvVnpE5xHA+gfwB+588mR1IZKD9H6CEd0kj9vf07pJbzkTsDHTZcf\/XWX3Fcxzwl60AMvrXHUOYPb6j4iPfK+xBs8IxwMA6zVwHGQX9FzUOgR1xDfSNiB\/R1A1ylTpoyEE9XgSJZlCftO4iCKyZvNmCDOcKLabesLSwpmzRjgpH4rYm576eRd7rc9Zrq9YvRn\/5pwj50rU26kx2MmX8y+JQa+JSmdxjjqIcF6hdM5z9leXuWqSwVTNqb21pUKy9vfHpayT21V5uyP9p4auI9TW3tfoX7Hm2jozzPC4eY1a9YU9JLevXurzJ14DbDJES4y7I+DiAWQW42YHIiDdk6A+blixYqCmMdGshAmW\/SRGXT\/\/v1qs1jCEhD30Iec+jBl3mJAi3EQEWJct5lf+lYP8pRwnn32WcEixvbfvNDoOLzsbMvHaj\/iGICuQj3WZfSjpTzYqoaIx5Z8HTp0UKvG5JvGrA3hsLszXta1a9dWRgfWdXQ\/HCNl+fZ6mREfIjXr2u\/J77i0jdG069a9EeNYYNbm7GL9lghi3PJ1mxxFOfASCnjG8QbPCIdtPdjyjrWXvHnzKpaM2AYHYnWfncfGjBkjY44Bazl4GLB4Kf\/9w3AQbFWDGMqVKydEjHKNnc7QddgIiW0B2W0MPQgjBHX+60YdMH3HAqxFxdKONonY1otxv\/xgU8G747fPXpdJS76VBlN\/EMYRDaiHHOc\/nhEOOggv9mmnnRaYEmIWXgNwHAgKjqEB\/Yc1mUBl2wleAxgCNm3apK7gaRqgpQAAEABJREFUtoO4Bldas2ZNQFTjPDU1VdXhD6Kggc3KrO9nPOx8a7TsndhcWeNiGSfPOp7gGeHAIdjKAxMyuQRY7We7PDwLyDnAeg4GAw32BVA7EiASxDm8EvAQmDBhgtx\/\/\/2C3gPxaFGtVq1agqHB3t78TgwMFMmfxxcD9YxwLMsSdBoWJDEXszkrLzd+bC+\/\/LLguxYMLJJCaOGwhoGANRyMCdTHymJZlvI+0KIaPnCWZYXrxlwzGMgQA64TDiIa6zUo91i5MD9PnjxZ0EkIH4D7YE7G05m0txqGDRsmGBAynJGpEMCAOck6DLhOOBgFunfvLoQR1K9fXxDFmjZtKgA+aSNGjJB77rlHICBELg3E2OBvlnWoMD0bDESOAdcJB3ELMYr4GAiFcGZMnAB7fSK24YLTtWtXqVevXgDQTdB\/Ip+aqWkwkHUYcJ1w9FQQ0+AqEJIuY+2FxUnWXrCK6XJzNBjwGwY8IxyIA\/MxFjTS2wKs1xDohskYKxriGaIcwG90o3AIXLx4sRL9KlSoIDiGQpwQoslyEw5r5losGPCMcPBiZq0G0QyiwNWGlfjhw4fLvHnzBJebVatWBaxriHfB3Mk+WRxBMTIQ\/EZMDpGg\/CYgDhceREPEwmyW5caOBvPbJQy4TjiIZyTjYH5EaPbt21cRBybkIUOGCIFuWNB69Ogh2qLGccCAASoKEMdN2tqBpBwQFp4ClmUJwXCIe3gLwIFwEGWtp3z58hKc5cbej\/ltMBAJBlwnHFb\/2ceT\/Tyd9vqkDM7wxRdfSKVKlVR2GrgJBEe7wYMHO87r4osvFszYACZv3HEgJpJ+EKmpG7FutHXrVv3THH2AgX2zBgRik6KNE4q0fryn6TrhYFLu2LGjhNrrE+saC5mzZ88WPKIbNWokLI4SX0PmGszZTkhgsZPwA8Q+vAXgasWKFRMC2Yj8RF+CCy1ZskTQe3QfKSkpKvzaHDOPB\/KcpVXNr\/KaketsRcOTVV6B4IC34HOuA1++OFg0LHvjJckK0M87XkfXCUcPHGdLxC9cbbTvEWIVIhp1IBSOABwEowH6EL+dgGsQBoRHnyyswt3whUPn0ToOHI2wg+A+9P2jPZITLNo2un4itmXsetzrnusvH\/eqIcvaXCyLym5R0Dtpj7QvdbY0rF5OqjZoKZVf\/\/vfNE7\/pXMi4E1Hh3Lkuh0aLzgkTpA64QvHcqe69rJe3xSUeP\/zjHBCTQQPaHzJ4Bz4mqHfkHwDz2eIgutObRHR4EhEiiLW4baDRzU6EwQE0I7fofQkrrsFcMjI7nViLbfbBucTIM8ZXAPx6tDGJRIcKg0xEC5dKG2xOKVxcnvcJ2IufiW+Ixym1r59e8GEjKMnngNY2nAARXTDiZM6diB0AE8E6hEgx2+4DhGgqampgh8c8TiIbH4gHPv43fjNy64hWDcgl4CGWPMJuDF+P93Dl4SDNYz1HfQRrG2Yldu2bavwho6kTmx\/LMsSHDgR9xDHSA2l68J5tJMna0LBTREz+ILGAvrrG0tbt9osa1VQNGx8baJoWLJyrWiYvfUU0TD+rxpSrvD7x0HdAtPkBhkotxzuJM3GLFEBZaGCxsKVIzGEux7uWmbaBj\/veJ17SjgsaCKOIZbxwo8dO1ZI3kHwGtYvDAGsx6CTdDxmUMDcHG7iTgugSUlJQpopOA06EEYHMufofq5bnXycHI64ESnYZfZI27lZr+rMXeIEzV9YIxrueX6BaBj67PMqEyfhyxo2pFUT8gFcV6Kg\/HXy2SrFrc4LgPgVKWQmb0Bm2upnHc+jZ4SDMs96DQTChDAZI0KNHDlSIBScPjFHY2HDKRTugycAdZ0AkzWcyb4AyvrN2rVrZc6cOQLRkLw9d+7cTl2YsjAYIB8AMK9b6QBhre1XSfJHkOY2TLcJe8kzwoGbkHyDEFhMxkSDwlVQ7AkfgPMQAk3KJxT6gwcPKo\/pUJhmzQaOZF8AJU8BOg6ZcyBGzuk\/VD9ulSOWBO4V5YlXbRmm\/d4QT5F8eeSmcWu4HBbsbcNWtl3MTFtbV3H56Rnh4OkMQfzxxx+BiSC6wVXYH4cITkQqknfUrFlTIKJQFjU6CLUASih1gwYNROs4EA\/ESBsNPJRYABehWNrRJhHbOo2b\/GhP1M0ry77bn6Hu49Wc9XOO59EzwsH9BatZt27dhNV9wp6J0SG3AIRDWlsWJdm9AC8A8hBokzIIwCm0ZMmSavGStsjZcBOIDIIjMA5igpuFWwClL9rGAoiXsbSjTWZkdq\/ahhs3+hDbe+w4mk8lXqGuHbwaN8843uAZ4TARTMRvvvmmjB49WiCWIkWKqB0HsHxpQJRr2LChIGrBkWi3aNEilVYKR1EICyIjohRdZuHChQKXIdYHR9FIFkDp00DmMQDx3DQ+Y5Et83fyvgdPCQcrGJzkggsukGeeeUble0bHwfRMEg+sYRoIpdaiGsnU4U5wFI1CdCL82yAa+sCRs2jRosppFE4FUBfxECME514CYk+s9\/eqLePN6N59aycHDAZ2vSejtvQfCjLTNlSfmSn3jHBIVcvqPqIaOgeLk6zZkDwQRR8TNSIYXITYHRR\/XnomW7p0aZWxk0w5\/AYQ20gvhfdA8eLFZceOHYKpO5IFUETCWABuGGk7e71EbMscMhr303deI+RB+2nFLPl0\/jQ5t8tsRUhY3zBhc9TANTsUaDNRnKDCI+84ljvVtZf9XrUPr0hcwTPCIYiNHQmI+ly1apXg+s9vuAr+aviX4cvGERccHhoyMiKZEwb4IiGaEXvDOV7UrAdh9g63AIr\/lYH451UjDxpALjREOCfgGtC7cXnR0LxODckKcHpnMlPmGeHARXDkJDEHynvSsYVKcg5QhuEAUzUvPZPLmzevyj1A6iesa5TZgZcfIkRPsixL8G3DFI0YCEHp+pzbM3nqa+boDQb61k4SDewhmhUQ75l5Rjgo9nAFlH580Qhqg1jQb8h0U7lyZRk0aJBKKEjUJjoOdUIhAN80jAfoOtRZvXq1IN6RjJBQbMzcAOepqalUiTtg6WM+iDMYKeJ+A9OhbzDgGeGAAdZySHmLso9Oggm5TZs2AjciVBprGS85+hBEQXgB7ZyArTwgOnKzQXT0Rd4BYnMgHix46FFky0H0c+ojs2ULFixQO8LBOVk3glAz26dp708MeEY4iGFwHBYn0UHYZeD7778XHDOxpJGUHMuZBtxu8DbQaEQfwhEUowBllmWpwDeyf+L3huEhOTlZLMu1TJ4CcXNPCB+3IXaXY2wGsh8GPCMcXioIgQCzXr16qf1xCI1G1MFMDQEEAxwI0cuvj4APARwR1yHGiEcEnJJzA9kPA54RTjAqWWNBNyEqlH1uIKjg616foy\/1798\/MAzWgXBQJfEHHtcsyGIqx3kUAqIiBg5EUM4NZD8MeEY4rMFg8SJFFOINm0zhQYCCje7jB1RDxGwzwloTHESPCV0GvytEQjghY8cEjimdBVgIn\/rhjBm6L3NMTAx4RjiWZakFygceeECt4bBwid6CRY2vtR\/QCRfBrN24cePjhoPvXN26dZU+BpckZS+7wWF8wDmVHHEYJZjPcQ3Nj2yDAc8IBwxifuaF4wtOMg4sYzh84jLDda+BWB68EcqWLfvfUEQYJ\/oZHJNCy7KUoyneDxARHIiEFrTjuoHsiQHPCAfiQAQaNWqU4GqjAS8CdAg\/oxtOhD7j5zGasWUtBjwjHNY4EGXQD9AjNJDZ08+6AWZmxo0uxqNJT08X9BtCGfhtIGdgwDPCQY\/BCMDaRyKh2rIsYREVPQfOyMItFkE8sRNpHmasmcOA64RD2DKmXNxpIBpibTp37hzIE8016mRuWlnbmrRT6D+YovFUYPEWl5+svavp3U8YcJ1wEHXIm9ajRw\/Bc4A1EIiI3wDXqOMZkhxujGcD7j\/6Et4NiJSkoWKRFg6kr5ljzsCA64SDYk1IAQuKuNNgHLAsS+UUwPMZ7wDq5Az0m1kmKgZcJxysaXAYUj+RrRNDAIYBveKeqIg0485ZGHCdcLCmIYqRbSYpKUnuuOMOoczvek3Oei3MbDPCgOuEw4DIPMNRA+4phuNobJhjImDAE8LxGjHm\/gYDmcWAJ4TD\/jWlSpVSrir4dJFXgCN5BQg6C5VXILOTNe0NBuKFAdcJB8sZi4fkCHACrlEnXhM0\/RgMZAUGXCecrJiE6dNgwG0MGMJxG+PmftkCA9mVcHz1cEisyP48Oq0vxyFDhvhqjLEMBs+JKVOmqFALXI\/wAgnuh3lPnDgxuChwjmMsee\/w9wsUJtCJIRyXHhaLvLjnaHjooYcCd8bDmnwFgYIEOCEuae7cuSqjaizDJeqX0AwMQ7G097qNIRwPnwBf465du6rsPCRl3LBhg+D0ivMoKYD5KjM8kpeQ8gpORcqrtLQ0wfLIV54jdfja84WHCEmNRV36GThwoOIIXKdP0m+RNZW9iFh45otPXjvKCNgjghWC6Nmzp+i1NTgDwH00kIcBD3eC93RZqCPclfEApC9mDOx3RGZWUmnhghWqrV\/LDeG49GQ6deqkzO+Y3Mklt379enVn8hPMnDlTiOdhK8dx48apvXx4cfHjI482O83NmjVLsDjyovK1V40d\/pDPjT1UiUJl30y8NNjRgap4o8P5qMPXnsT0RN+ymx073kFciF8kGdm9e7fs2rVLeXWQv5ski\/Shgfrk6Na\/OQbPkXlCFJTDXeG0zBNvEQIYGRdzYf6EZVAvkcAQjktPi90YtPl93bp1ovPBsZ8PX+6tW7cKLxdfZV460mSx7SJciNS+vGSWZQme2gTShRo2Lz4vKITINpEQItlLqc\/Xnj198O7m\/oiHJByBKIiNIlQCnQXuRhg7BEMoO\/ezxxvBrSinXw3Bc2SuTZo00ZfUbnrkAicxJPoeFyBe0mklYhqtBCYcUJ99AHGFdFMQCy8d2XLmzZsnvJyIX3qm5NZGvNK\/7UdEoD59+gh9AHAfRDt7Pf2bvkhHrH\/v379fveRs+gWHQ0zEIRfC0nWiPTL+6dOnq5TE7dq1U0kio+3Db\/UN4fjkiaAs4+gKwfCikWB+5MiRQoAcohSRppTDlSAyctHBOdjGhHJ0DqbC1xwuAkfAE539hxDZuOYEECsEAgGR8oqMqSQjIfUw+hO6F1zQ3pbUXuFExuD6iIPMBxGNMetrcDzGTuZTXZYoR0M4PnlSiGK9e\/dWO81VqVJFyJbTpUsXQWTCCIDYgzsSBAQXQqziN6moyKiDLgQxIcpBDBwRA8n1xnmoadarV09xABT1Fi1aCF7riHO8zNotit\/29twDPY0X334t+DcECbchvTHGCYiQI\/oWehRzIQYruE0inBvCceEpET3q9PJCGIAeAroFSj0cY8aMGUK+NsuyhLxuZP+B2yDq8LJaliVEy6LTsCfQpEmTZOjQoSrXG1yDcgCxja8892cc+l7clzJeXJJCYhbmvtzLsixlUePFZotJy7J0s8ARCxycCYJwaX0AAADmSURBVC5FH4yX\/gIVjp1wP8bCNfQsxg\/wUYBYmCv56TIjBh67jSf\/DeF4gnZ\/3xQRDQKCg5GQ0Wm0EEvr1q0FHQhCdqoTrgxTO+EkcM1w9fx6LZdfB6bGZf6cgAGsYVinTrgQxwKcbEmADwcLxw2w0rEmY1kncqSMhoNOx7oT3DCjun68bgjHj0\/FjMn3GDCE4\/tHZAboRwwYwvHjUzFj8j0GDOH4\/hGZAfoRA54Tjh+RYsZkMJARBgzhZIQhc91gwAEDhnAckGKKDAYywoAhnIwwZK4bDDhgwBCOA1JMkcFARhj4PwAAAP\/\/LyTpEQAAAAZJREFUAwDYSpSSI6NPeAAAAABJRU5ErkJggg==","height":124,"width":206}}
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAM4AAAB8CAYAAAAhHY+6AAAQAElEQVR4AezdCbhuU\/0H8HXOv3nUPNclqqe5VJq7KTIlIomG+yTRJFPCk\/AgKT2JelJRhugRRQMSMkaSIfJISUIaNIhIqfzPZ+t3rLPu3u\/e73ve9zj33vc+93v2Wr\/1W9Nvr+9ew157vZPLL7\/8Ha985Svv+MMf\/nDH5z\/\/+Tty\/x3\/+3fyySdX8mc\/+9l3XHLJJf+TzryQCxef\/nbbbXcHP7m05fGWt7zljnPPPbeSh97MVO7ySYMOcAM3cN+leccd8iKXh7zkKW8ydQrdm2+++Q5lIHflF84fcenm8fO8Ip+ISxciDXmKK440gZtOINKI\/CJu+ENPPPEjzZCX19CjWwfpi8Mu8qCjDGQBZZaPMPp5mtyh5youPWlJM49b6tIvUcYXHjJlkB4ZhDzsLT\/5yl8YnYByk4NySEd6\/MJCzz2XHrkrf4Tl17r4IVOGyZT9u\/LKKyvftddem1784henFVZYocK73\/3uSj6VSZoqfOVu+7Piiism+j\/84Q\/TVIZJmo997GPT\/e9\/\/9qoBx54YJVXmWep\/MAHPjA96lGPmiGW1wxB5snDHvCAByRlyIJH6uxS1tna\/LWvfW266qqrKrzxjW9crD6f+MQnEtu6bzfeeGMVntuEgD2XW245zhTl4elSfnpd8cc\/\/jGdf\/75lfoLX\/jC9MhHPrJyr7HGGtU12kvlafkTcUKtrFPIXfOwpjZwyimnzGh\/b3jDG6r2K35AW1ZGZZ9BnFAYxjUK60aANMvKksH222+f3GDuHXbYoWoE3\/rWt5IbRwaRBvd8x91V1n333beyHSLl9jvzzDPTP\/7xj85mG1X54wGqIN\/85jenG2o8mMmV9e9\/\/zvnnMGDJcrg4cN+P\/rRj9ITnvCE6TIok7Jpk+94xztSLXH0NiopgRKecNOp9XBMdZNVxp4wkaEnWxnl0ksvTVNdayX+4he\/mLbccsvK7QmJ3ZVn6k8QkUzYlGj6\/6hu9HQGheO3v\/1tuvXWWwvpXd5ByjqIzT1w9NCvetWrkqf5XSVIaWookp7xjGdMizzd63oVCuyZ90aDlF86vaDhffnLX+6lUoVddtll6de\/\/nXl7vWnvOfh17Dr2llTWn\/605\/S1772tSrYQ9vDhye3CT8iAZuy7QziRI9AAUSAXjdIeB3ud7\/7pcc97nHVEE1ablpbhaLydUYOIsprjz32mG4oulhPL\/JRIxqUYacHi\/w02DA8P\/RT1tnYPC9PaQM2B+UxPF2wYEGaGtPzJrrsxsPWe++9dzUs0ehe9rKXpX7KL40uQAakoBtP9fyhrNEK82A0JOLuhal5S\/LQpePKz92lndGrQ7Q\/YYceemhlE2743ve+55Km5jfJcG8GcfQmKkVD1+VpBgxNxvCeXNxtkLgbFnrGhXVxsReL6Rmuyc+Ni5tODuLKn1vD9YSmq5xko4L0N9lkk6SBRUOVF7n8lUN5yALKussuu1ReYXToilMJsz+zsflb3\/rWai4qubCdfCDyMtyIRpnrC6eX2\/o973lPetaznlXNPYZta2RACmWNhwV3AGERl9+DyAOJuwnSMg9RB1d+uuzO\/txd8PCHPzxpm3S1c+kBNxm490ZOyqecZDOIQ6CrMmTiDohgzBzDqJC3XXMD5Y0uj4dg++23XzWsCznyIo6bTqaLV3j5l2WjGw2D7jCg8ay22mqLJaWR1+VfykSs01VOEJ5jUJuz3ZFHHpnq8pe+vM4444yKCPyhz2b8OaTBviHjJgu\/q3jS5O4H7p3hujjuKbJy58gfoB420aPnOrlbOZQnZNFG2T1kXa+77bbb9ANIHA867c+V\/4gjjqhGTtqEtkE2qbvMjUsoc\/LAT3\/60+pJJKwJEqQnjvj0XPnBjSCr0\/OEUAZ6oCHlMo3DTRc\/TzN08y6WTl0e5CBt8SJN5eKXvzzpQOgJC13yuvxDpv7ypgchlwbIC7jL\/ErdMi3pNaGMK32QV12cvG70QBqlLpmwgHj92DrSc+\/YUDplvet06MlbftziSiN04xrhdEp7uQ9kwqQVcVwjXqQrbW66wK0tuPJfdNFF1aKLeOLDYj0O4XyEcexznvOcaiUmhk7KSR6LC7pcFSYfY3ALsOnY1r3tNxBxbr\/99vSlL32pGhsaD77mNa9Jp512Wpp6gdQ7t1mEeoLoKiWhG9XdyzvGt7pqy4TCx5idBZYdWw9up4GI8\/Wvfz19+tOfTptuumk68cQT0yqrrJLe9773pbPPPjuN8p+uUtdZB92yGz7K\/JeltOejrY0mDPXc\/6Zh6Fzdo8l+M\/rLX\/6STJY22mij9MEPfjA99alPTVYyXvSiF6Vjjjkm\/fvf\/+43yZHqez9Rh6uvvjpdXYOLL744XVyg1JPeSAs9TnzeW6Bv4vzqV79KXgCacE1MTFQVvO9975te8IIXpAsvvHD6\/UoVUPyxMleHQw89NHXFZz7zmdSE3XffPZVo0m3Kr0v5pCkfV5DW6aefnk6fwnHHHZcOOeSQZN0fkPD3v\/99YYmxd0m3QN\/EQZp73OMe6aEPfeiMuj\/taU9LN910U\/rzn\/88Q557NKI6lE\/0Xn5P+ybkec2FO8qhvGeccUYCQ8bf\/OY3lfuoo46qiOydyKtf\/er0ile8Ir3kJS9Jz3\/+8ys8\/elPrxY7zNXGuHNf5KjsMOz20DdxPD0t33lxNOzCRHre\/naBt+EBuxLC\/dznPjc9twa2pgQQ3dxMnj\/\/+c9TFyBIgB0Q57bbbpPEYrjPfe6TFixYkJRj4cKF1XsC77Ie\/ehHpwc\/+MEVlPlJT3pSNUdceeWV00c\/+tEKXr6deuqp1RKo8TyUfrJhYtD0u8Zr02sKr5N3kU3rXHXVYvdmGIK+iTObTA1pdt1119QGc6cusIoW2HLLLVO4rbTlQBANVCM\/77zzkuGYjX377LNPOn1qeIUETYj6IgCyvfnNb06wxRZbVPkpJ7er\/BETYSJeXHMiebFGZ7nllquC5W3PFPvANttsk975zndWW2S4Df2uuOKKSndUf7yAHCTtrvHa9JrC6+RdZHU6g9SvKU7fxLHsa6ft3\/72t8XSNIT7v\/\/7v8XkcyXQAMHcAik0OkMkQyVuMmGGi2WZ9ASwxhprpA9\/+MPVqqGtHwEyEA6IBOIAEiyY6mEWLlxYEcrDAZnqiNSLRHm51EVZDz300KTsURf1QSYQnscZu+fGAn0Tx5P7n\/\/8ZzJMyYtoqGPooRHl8lG5o1EhgkaFGBoU8JM3NSpl1PgBGSyt5wQhD1LMpvzIlBMpeqQ8zZJE6623XhJHGZQz1+VWb0AmUO8glHoHmejQH2M0FuibOJafH\/OYxyR7j+KFp71I55xzTrIk\/ZCHPGSoJdUAEAARNIy8oXCTCaNTZqzhgUYYBPGiFkn4QX2QpIxb+u3uLWW5vyk8l0ePpDeyz0pvhFyRDhLFA0nvbQho57LyGnoiXlNZ2YkdgkweIAFkgjobRd55OUPW5do1XpteU3idvIusTqdLfbrq9E2cRzziEWmDDTZIBx98cDrooIOqrd077bRT9eXghhtumCYm7lyi7lqA0HPjwc1HBqSIG89NJqzp5lus0KhykmhwgCDkwiO\/u\/tqgUXPYjgHtrjkZfIwOmNqpe6kk05KVueQbr2p3kjvGOTnDjJ5QOTxudkTkAnYMXonbmSCJptKY0nA3VHGvomjkIsWLUoMb\/esm+ndzv7775+e+cxnCu4JNxKQABmkEwRxJRPmZtIrE9NAEAAREELj0ZCOPvroal5CJoxOGbfO33US2abXFN5FrtdhRz0REumJ8rLqhSxze2cEFjT0TuroXrCBB0QAmUB4nk642ZV9kQncg3IxoiuhmuoXecW1Ta8pvE7eRVanE2UZxrWWOPaCrbXWWunJT35yWmmlldLGG2+cmro+wzVzHgsG3E2FcqN82IYc0IUgSIIEyJA3Dm4yYU2No6kc812ORAunFhh6kUhPhECARIilXuwFyATs5KGCUNzIBE02qyPUuIdi2cWxGHFsobbv7PGPf3z6zne+k77whS8kN4bshhtuqFIYZK+am2XcXiWQ\/XGjAQkAIdxkNxv4ycWnl0Wd4Wwi9gylGk\/XeG16TeFd5aUef06imBPlVXBfShLl4eFmN\/ZDJmDfklDmUHQiTn7tRSgfv+mdwMMxj5e71Sf3l+6m8Dp5F1mdTpnnbPwziKPH8A3CE5\/4xGr505vtVVddNe21117puuuuS77im81eNe9RkAAQwg1EDuAHYU03MPX4N2jX3DVem15TeFd5qVf6Y06kJ9Jr1A3nkGj33XevtiTpiXqYqwrKCbX11ltXQ92SUL3uB0JZTTXcA0O+6KGMKpAJEKqsT1WA7E9TeJ28i6xOJ8tu1s4ZxLnllluqfWgabr465qs9foYyn7HtZpC9ag5AQA7odUNmXaulPAELBYZzMR9yv\/IqR0\/UD4ny+Dmh3CsPuJJQyFvmG2loJ4BMUBLKMB2hYi4b8Xpc513QDOI48+yzn\/1s2m677WYU1Kesf\/3rX9M973nPiliGXIPsVZuR6NgzawvEUM4wa1QkyguZE6ppyNeFUAiDUAiUk4p7SSHUDOJMTExUmzcNC8Jghmh77rlnsgPaJkVPEuGWf0Onn6ux5yhw1llnVQsY\/abdNV6bXlN4V3mpV\/rdh15182DzctoHfuZDlrct7uT3Ju+JPvWpT6Xjjjtu2mZlfr3yysPEszBk\/53hI2y11VbJOREOENFjIbYhnC1LeXlyt3ZlSFcSav3110\/mUc6lAKSTZ14G7lKW+\/N8huWeQZw80fjKc80110zXX399Nf59ylOekqsM5Db2HAUsZgySbtd4bXpN4V3lpV6bv1ddfdC33nrrJafaNPVE3hNZ4j7ssMPSt7\/97WpXe680m8LKcuZ6PmU3JDd\/etvb3pYc4RRDvpjXRg\/V9CBGdvMo+wtBL2Uz7M4775zAQpW5nR3ptoFF\/nm5BmqoLZEmYxu3s9NCF4N9qPbxj3+82gpvt+5LX\/rSKni+7lVjsKqAff7pGq9Nrym8q7zUK\/19VmtavetwDonyOZEGO51ID0fXcuZ6hnyAVIsWLaoeyt7DBan0UgglvG0epQfSS3khb6inZ7Mw4b2iYR9459WjCgMFTXpnA1gsBRN\/Z275rubwww9Pn\/vc56qDBYWB4YD3NqVhPRXmcq+asozRnwW6ksgT3DsisDpX3uv+cu2qfadeTigE6ndhQir5sA+ptEsyYcPCpO+44UEPelCyHO0QDonrSvUyExMzt9DY2zWXe9WUpQv0kl30Sp2u8dr0msK7yku90l+We7b+kkTmROXTHWHaSNS1nG16TeEhRyjl00OZR+WE0lY322yzalc6Hcjt43N+8XPZbN0z5ji+CTGpstRsBc1pijmMi0e1V222FcmHAv2k1TVem15TeFd5qVf6+6lTv7pIZE5kEt80J2oiUddytuk1hdfJcxlCgPkcUiEUxLCP29ynX5u06c8gjp0BDGS3gI+tkAcYAQAAEABJREFUSnz1q1+t0lNA48lB9qpVCYz\/zFsLINHChQsTErW9bDWUA8M5X8bOt0ohlN7HA3\/YZZtBHDsFLrjgghmf7Oaf5\/rKUgH0Rptvvnn1WydeiJ5wwgnVp8HpbvwXXXq\/Regar02vKbyrvNQr\/f3Wq1\/9pvziZWsbicwlepGoKf0oZ1N4nbyLrE4n8hrGdQZx6hI057FSYcgW4bFUbbnRqtxcHEgYeY+vd58FchLFkC4vjdGKOVEjiXLlJdzdkzg\/+9nPqgO9\/\/vf\/86oprVzY8e5PpBwRiEKTz7uLYJ6ervGa9NrCu8qL\/VKf89KDCGw3\/xiSKcn6kIi71m84GwqalP+dfIusjqdprwHkTcSx9tga+FW2\/KEZ7PJM09n7F56LNCVRF5gGs65mhctyRZoJI6XnnYMmMvkFTSn8a7HytvExJ1L1bbjdDmQME9n2O5Bx7Rd47XpNYV3lZd6pX\/Y9irTGzS\/Ml4dicgiP8M5PY8hnReuhnVIVKYT+nXyLrI6nUhzGNda4vziF7+ovsPxAqrcCoE0402ewzD90p8GwlihM5Szf877F7K85lbjkMjWH70REuXh89W9GHFM\/A844IDqJ9ucOlkW3BvY+bjJUzk9ZfpF13htek3hXeWlXulv2+TZb71L\/TK\/MrzJX8Zr0tNm7DpZd911ExJ54eqNvvgBvRES6YliE6qwMs0uslyHe9hYjDi++jQce+9735v0LMPO0KRtjOVTvzbINy32G3e+6ccmVK839EZ6Iit2eVvz7sX+OT2RTagWF3wTNkhd8nSH5Z6xyROz7U37wAc+kPLf78wzm6+bPJU9L2dXd9d4bXpN4V3lpV7p71qfQfUGza9rvCY9Q7eFUy9c\/ShtE4miJzKUA59DkOV1LdMv\/bnuMNwzNnnqaWTofAHvZ8CGTx+y2UVgB7XudrzJcximH6dRWiBIZGd0kKjumyI9EQKBOVFJojLdUfhnbPL0oZqd0jn8wJDe57vf\/W51IPh83eSpCx\/EQF3jtek1hXeVl3qlf5C69RNn0Py6xmvTK8ODRPagBYlsn8nrhDDmRAhkOIdEFhvolOmRDRMz5jj3vve9q18otls64D2O86CtrnHP102ewzTKOK2eFpjzwCBRl\/1zlrcRKSfRKAo8gzhdM5iPmzwNMbuWP9frGq9Nrym8q7zUK\/15mUfhHjS\/rvHa9JrC6+Q+fzEvsmvBsM7iQm6T6ImCRHoqslxntu5W4njRqTvUA0Vms9nkad40CtgvN0i6XeO16TWFd5WXem3+QeraK06ZXy\/dPKxrvDa9pvA6eS5zfJlTSJHkG9\/4RjL\/iXbqijCWvl35h4VW4gwrI+nkO63H7qsad6GPbTOYbS655JJkx4ueKOZF2p2hXrncTT4bzClxZlPQcdyxBfqxALLEcA6J+onbRXdMnC5WWuZ0xhVus8CYOG0WGoePLVBjgTFxaowyFo0t0GaBMXHaLDQOH1ugxgJDIc6ll16aLPlZpjzllFNqsuku8om2dXlp1WGTTTZJNgBGirYB0SvlET6fr375WtlzzFU9Im+2ZvOudgp7K3NdXGmRC5dH13RDb5htKdKMq3bDvsrmiN5os\/x16NWWh0KcKNhcXG0HWnvttZMbNBf5jSKPuIHOVi7TVz\/nP\/e6aWWcgfwDRGLz888\/fzqmPYyWgKcF89zhZepll12W\/PpGuQeuruj2aTaRf14Tx3cb+TuNHXbYoaqfG2a9vvIsgX+QAxTdEVtRR58U231Ovscee4z04WBLv3zLl9vybgKbs30e7gja3D+f3V6S3nzzzckRufmvbeT3gE08DGxqVheHHXpgcOeYJg5mld1V3VMvnpahq1v2uzd5ouHOu136ukayCC+vjqb65Cc\/WYqn\/S972ctSNKwrr7xyWh4Oh4vIQ16gW1beCHfNhxp0oK6edfYgk0YOcaURYI86Q+dxorG5ORBhvlNxMj+\/N925XaUp7cjHNeqnjjvuuGNaZZVV0sorr1w1DOEB5Y56Rxwy4dKUtjx7QR5nnnlmpfKMZzwjAY8eqC2+cPnIT75RFn5gQ2nVoU1Xe8rvufSijnl6yqCs2o92lIdpc2zn5FqnOtmzaWNzrhNuZZVHRRyFqxs26KqERSSZGybF05LcE4geJvMHGMimvFzOTSbz0HN1gujHPvax5HMGh4SQ9QtlYjB5RFwyZXPTQbinZoTHlY7yhl+d6+xBJizXEzf8ruyx2mqrJTeUvxcMGwwfcp3oCWwdQSRh0pKmtPkD6qcROECS7Oyzz05vf\/vb03nnncc7DeWuq\/e0QgeHciovVW0AuJXJE5q7C+rKwoa5\/SMd8rLcZNF+XLWn\/J6Lyy7Kp73ygzIqK8KXO6ef97znJXH0+B4O7Idk4vnd2vvd736cyb2XP8+kzKNwhkK6KjBMoiCMDrdKy5w7ujcZGjOSgUmXG+0nHfilc8UVV6Sddtop3ete9yKqfu\/k2GOPrc6qJrD72mEffguFvwl+SjGMtOKKKy6mVld+5YP8xtfpMRhyKfvJJ59cpR11zO3BoG4Im7ANxUjPzdGDKOPee+89YxGDXsAp\/Nz03HhPMEBsN0dYQJk8VOhOTNx5OMryyy+fdtlll6r3JVcOb8kdfM\/+4ronyqPsykc2G4Tt44mNsNzSjB6Uuw3iaKDK5b4opzh1QyJhdOiKIy5ddWSXyJeezZ7aWuhpp+rvKAA\/lhY9ubTcQxtFpQWOOzPn0euop4eP+NL1mzw++y7v96RCiOyG+\/aBGxhbRG4F1FhkyK+ANn9y2\/zpJnIHDDHcUBVVIQVzDhtiYO9\/\/vOfZGjhCSmOT2JXX331apjBH9AgNKgA4gpTLmXgDpSyaJwR7untKc5wnuohj6tDSG699dbwVldPFwbj8V2SuDEniKFini8D23BI39MZWblLsB37lnKNJOoc+ZJ5AtL1FD3xxBOrIZnhRdRRI\/JkdNPZfWJiIm2xxRZJecRzX91f7kEgfQ8WceOJjbzcZNqF9sHdBj2ne0FP2zHf4NZQNXTugDA6\/Hl+\/BD3xE95RPnIA+6Rtoc4Hs6OxHWOhpGNk3ZCr+mqTO6HcGm5xv2e1GAIjOnC0PwauJvB7ZiovFHlT3s9h9Nw8kMLIxM30RMVsW677bZ01FFHpUhH2sccc0xykrw8ukIDOP7446vvhvI40lPmkDkIAnHD76oBeKoHEV3DMMJBPPucuAF56EHeG0QdGVeZhAN98dTdA4S7DsiLiHvttVft2Q7SQZ5oTM5\/eNe73pV8SMieL3rRi1J8tCUv5QiCeRDZQRz5uq\/ub\/j7vXoAeBCI5xNn6QE3mbyjnPy9EGQPnbwthSyueZj8yjpoez4ZUFdldC+1N\/aQhrZ2xBFHVPOxf\/3rX8kcRu\/NdqeeeiqVCjGyMMcW5nse91SgkZNRSHm\/qzkOhTZ4opiwlnrPfOYzkxuaN9pSp87v8IkLL7yw58qRXkXjynHkkUcmRqxLs5dM+T2xPcERSpcu3fLJ7wmHmGG4PE03xiRXWrl8Nm5DVL2xp6eGkuerp3cApPR9yjExcedQLc6x86m7MBBfA+b2W64TE3fq8s8W0ZNJR6\/vAQHcZKCsrnMJZL7pppuqLN1T7cX91SsQsh2beGgLN\/oJ2xnWxsOeDn0PHPbXmcTIAQnVX3iOSU9qAr2KJzI3YKsEuDHdtux4Egf7hK200koJFI4\/hwrstttuSYF8eq2hBowjVdoPWOVxRuXW6BlImcxhYriQ1yXyRkwEjbK6IREmjfzpytD8oZtfDckiXlzZOHo9V781hNCGqr6ytUyqjKEfvyZm0STvwZ72tKdNz6HMHZ3jvcEGG1TRbrzxxpTrytP9rQL7\/OMhYf7RFq3rcK20d\/g1bL19Wz4RbkHJfeT3U4kxbFRv9SePn\/e4\/PLLq17HcI+c7W655ZbpOfY111xTuf2OqiEcW9KrQ9zvyWAWpsbPeIjgaaKRcOtePYkjQY3QMEIYw3rnwB2IiaP4P\/jBD6oeQqOwcrLCCitUuwwc9xP6c3llVMaVpy44jM8P6qWMwE3maa6HcnP5gU1c2Q24wXBO3KaeCSk9iOiKB9wBdmE3fj2QPBGDn53Zm5ueQ1O4kcvQw3sKfk9JjQlh+N3XMh\/yLvBQiPLonfMHA3c8VOjQbUszhj702J+f20O5H+KIE1COcLOB+vO7xiKAYSXbk5c499xzkza7zjrrJA87v3Ubc3\/2F1be70mKUXlkcdMBOWQgjA43w8XT0BicHgYymvCAp3msYpi0CacnfTomiE7L4c4hn\/hJxVw+DLfKMwJjGgcruyt\/nr639mUdc131AGVlG3HDFvTCbvnElk6OOjvKV+M2\/KArD5N67xT02G562FE+5kYTExPJEPnwww9PP\/nJT6ol1SiTMb806YbdpdsvYgjGdmxYxo8GRa6xGalwN4G92V25XPnpmrd5OHN3gQdxPMgd1MF2bA8R34MF2K6u7KHn5FoHbf7ud7+rDqTxXkh6wrVV7bm839UcxxPVzaSYg0xYyFTs6KOPrrYshMwNNqxRuJC5mvx6Srvx\/AETsUjThNdqR4SN8qryTofM81AWINMoGV0dm+Y4GmVeV\/WI+NIADUy91Z+\/DvKwOse+deHSjHykZxXSi7kgdMSZmJhIenF1C5kylenyK3vodL3q3QzB6BsKxVCHP4CcUS6k1xNGWN21LIv6sZeGWaffS2YaoP2FDjJp8CGzisp2\/LmN6Hdpd+4De9IHbjLuijgcbrQuLweZsBxx00PPDVawc845p\/pxqRiGiKOwbjjjWFETJwxkcqZrtkRIN5Cnr6Ahb7rSka5y5OSVN8MJizxd+QP8EP6or3SkF\/K4yqssRx6fnjzlXerV+eUnDhjmsKMGLs3Q1zN7ak5OTiZkowvbbrtttcLmx8BCN655unT1XOUcJ3SkyeYRN7+SC5cGe7BLHs6d66i7IWrEkQedEuwoTRAntxc3mbDcDtKIeFEW5eEO2xmOKQ+Z+Bq5tlf+cIC2Z25vNESvCWX+ykBGf5o4hKOA5VMTYIWM8aaxN6JZ+jMMGUW+S0ua\/djPnMwQCMy1wgaewsCvcWlw3Es7+rFdv7YYOXHG57D1e0tm6vdjPz0WSMGwE4HAHIzMkEqPxr0soB\/b9WuPkRNHgebjOWzKtWQgpa7205PEMMVwooQhlKHMXNdbnvJWnqbh26jK1NV2\/eY\/NOLETTMOLQvh5Z1xpvmOs6+8vV2wYEGyklHqLqv+LvYzUffS84QTTqjmk8uqrcp63x22GxpxysrU+bfZZpsUsFwLr371q1OAH0Jnn332SYcccsg0LI1efPHFY8LVGXcsm1MLzClx2mqmBwLkAETxQiuASEiFXEE2B8WDzXvPf\/7zE1gitdpkfD\/GCmlsgxXaml7f4XNKHNtrbAi1ZyjgBRrYrAeWp6FrTbwLAm\/PH\/zgByd42MMelix1W8q1VRzsFvZy1aTZip7xNnjj7tqGNr2m8MFxEwcAAA9LSURBVK7yUq\/N31befsPL\/LrG7xqvTa8pvE7eRZbrdG1L\/ejNKXEsQyPJokWL0qL\/AZHARkewLwpOO+20BNwgjF4QzhXRoBfR9GAQvVf0WHotMGk9ZGo42Ga0upd\/eZym8E7yqYRKvdI\/pTLS\/4Pm1zVem15TeJ28i6xOZ5gGnFPiDFJwpAAEKUmHTIBYOclygolXly8yQQwDDf0QCYmgLs5YNrZAWGDeEycK2uWKYJATDLFyUumpoI5QTUQi75L\/WGfZscBAxPE5qqVlm+xMPH1IpHHGzoC7w3w2NfbKF6EghohBKBsgEQnK+AijR9ITvelNb0p6IrJSj78p\/67yUq\/0y2OUGDS\/rvHa9JrC6+RdZHU6w7TfQMTxOaqGt+mmm6b4lNfnqPEpdFqC\/tkYuOh\/8y3kN+xDIr1WXg3ffwSJEAmJ8vCxe9myQN\/E8VWdz1E32mij5OcT7AeyJdy+s0E+hR6WuQedDJbxolcyT0IkDwhEysup1wkSIRB\/mU7ohzz8cS3lbf6IN6prmX\/XfLrGa9NrCq+Td5HV6XStUxe9vonjzbUvQ+0SnZi48\/Pc+By17VPoLgWabzrmQnokJEKmvCdCmCDQuBeab3dutOXpmzhI472JT3zzovkcte1TaE9zu3TB139gLDoMnHXWWWmQdLrGo6d31dOaF\/kIyzAvbBAkMhfab7\/9ku0xyiOea4lS3ua\/7rrrBqpfmW+Tv8y\/Sa+Ud43XptcUXifvIst14h4N89o3cTQQe4PyRtO1QK973evSSSedVMGLSDjssMMS+Iov4BME8J2FD6MCviGfmJiozjDQFeeweyD3d3V3jZfrWRTxabKP+syJ8l7IXMiHWT7Y2nnnnZMeuq4seXrC2\/wON6E3KpT5d82na7w2vabwOnkXWa7TtX32o9c3cfpJvB9dZwEErr766uroI9tuvKAMaJCGRo7v2X333RNwg7DTTz89nT4F8a6eSqOf\/AfV1YsawiFQ01zIOyJzIS9hB81nHG9+WaBv4viizokrnv5lVQzhen2S6pgdQxxfCQbMIWDBggVpwRSWW265Mtme\/iAbspQEqyPWqAj16Ec\/OsVcCInyXkgFEN5eu5gLIRL5GEumBfomjv1fPuXVYPMq+xzV\/jBP4Fyeux0FhCQLFy5MC\/8HRAJPa7BSt+uuu6YAPwijlxNuwYIFacEU8jxKt3JCEEsDzgl13HHHJWFlvNJvbF\/Kcn8ezgbRCykzf+ga6ioD6ImCSGVvlKcXcUd5HTS\/rvHa9JrC6+RdZHU6w7Rf38QxQZ7LT6H1QIAgOek0SGQCJHNOWx3BxKszGDKB79sN84JMhnpQF6dfGcIopx4IlJUsTyeIpDfKiaRMXQidpzV2z50F+ibOKD9HnU21TWbrCKaxIhZSQfRYdYRCpBju5UQil36v8jWFhxxhDOUQCJTLg6BMM4iEODakBpkOPvjgZHinZxoFoaKcZXna\/F3jtek1hdfJu8jqdNrq0k9438SR+KJFi6oP0pwist5661UrR\/vvv39yHK7w+QikgoULF6aFU9Bwg1BBprLcCINIFh9AT0RW6vXrDxJ5uer9UC8iSRuZzA8N7\/RMOaEM9ciQCpAKxBtjdBaYdHZWmbz3LGuttVby0wc+Cttvv\/2SBYFSj9\/+NHMe4dxkdwcGHdM69hSRICdS2SMhzBlnnJEQCJAor2dT\/l3kiIS8OZE222yzhNx1vVLki1CgF0IqQCrQUwFiAVlOMHEQDKTRVM7Iq+naNV6bXlN4nbyLrE6nqQ6DyBfrcZxqaEjjfDQTZ+8ivvKVryQ\/KxHEmI971Qbtmst40StptIjkqlHnxs1JhED8ZTqh31We6yGSs9AWLVqUgkxeuuqZLDook1W7XqSK\/JECEARZkAuQCJkAsRyFXBJNGD09WUAaID2wayTy6nXN61en1xReJ+8iq9Opy3dQ2WLEcXOcdeaHkRwOt\/HGG6ftttsuOWPZ2+v5uldtUAO0xdPzLJwa2iERAuWNFWHKoVxbeoOGe+GMUAiDUAgUpIrhHj85YoGygnhd80UyQApAEkQLIBIgFSAdwgX4QRjQhSCeqzRB+iC\/ruWbL3oziOPdjIr4ft8iQBTSWbrC\/MamN+G23cy3vWqDds1d49FDIKtkemQk0juFjXISRS8kTDzXEqW8zV\/GL\/3IgSRBLORCJPAwDHJxk0FOMmcriw\/SgjKPLn4kAO0IEASCeK6IBIgFiBbEc+UHYc4gp4twAYfISxvkpVyztZ80+sFixDHnsb0jT8Tys17I1hek8aJzkL1q0lTBUUBvOEi6XePleuZF3metu+66yQF\/5oLqBjmBnFWd75nKy5enR96vX5x+YR4KzmUArxY8AMB2qK222iqB47vAIepgmAiIBuZf4CGioSMd2K8Y0EOyxyBABkAM7wdL4llhRCpAMGUw1LRPEJDtgAMOSB\/5yEeS+Xn+gBukPHVxFiOOcwE0ijplMhUadK+a+Maeo0C+N6mf9BeLt\/zyqS5+k57hrPlI9ELqGGBLjdtePA8dvXakXabX5vcwi7ijuJb553nYmwd6M1Bf2HrrravT\/ffcc88EfrIjYB+fXg70cgE9HSAgGFKCdEGPB3o8CFt2udonCMh20UUXJSuRlvU9JLrE70dnBnH6iTjWnWkBTzVDOXMhJHLzcw1zITcxVuQM5\/LwpdmNAAF2ASQBQ0pAIkAqCKIhHoRfGNBFOJAOSFc+pS37\/bnMMn6dfwZxHLHkZznih5fKCE7knM1etTK9Yfo92QdJr2u8Nr08HIkMYxDI\/JA\/yhZDOUTKX7J6BRA6rnl6\/KPGoPl1jdem1xQecoQA5NCDIArCARL53AOhEAzRDDG5wbFkw7bfDOI4j8zY1A\/s5Bnx+8EgY2LDOO9tNIBcR\/fYtlct118W3AiDOAgE5hJked3ZEYl8bpETyVagq+doh3denqXJjWhQ9zObs63nDOJYAPAJtLGhCXAk7qfekApxwGKB72XivY6xvEP+xJVGxJvLqzH5IPl1jdem1xQecoQxlEMgQCJL3WWZg0iIYwUqyOSeGN6ZMI+CUFHOsjxt\/q7x2vSawuvkXWR1Om116Sd8BnEmJibShhtumKzwOEfAF5pOszHh88OslqiB28rGQQcdlOjstNNOyc5ncScm7vycup9CzLXu3Z1fkMj4POZETURSVmTyGkDPZJ6UE4qbDKkAqUC8MUZngRnEkY39ZpbyHIFqH5r9aO9\/\/\/uTsaRw4LYUKIyOm3p371WLsbDy9YOu8dr0msK7yBHJEDgnkuXdXmRSR4RCEr0QUgEigZ4KLEYAWU4wccQF6TSVUz690DVem15TeJ28i6xOp1c9+g2btG7fKxPzmVtuuSU1rUwYrtHxfoC73wKM9Re3ACJ5N2RoF2TyvsgQz6IDQpkkL1iwYPHIhQQpAEGQBbkAiZAJEMuSeUk0YfT0ZAFpgPTAML3IcpnwTjKqM9FuuOGGqsLL+l61ygg1f9rGzE3hXeWlXun37gyhEAahEChIhVDAT45YgFggXk2VakXaAyAFIAmiBRAJkAqcG4FwASQEYUAXvMdqIp\/8yvpG4erkXWR1OpHmMK6Te+21VzWnMfmUoOU7E\/zxXjXWWDKAGIAkQSzkQiRAqphLcZNBTjLxxAdpwSC1RwJAOkA8COK5IhIgFiBaEM+VH4QB3SCdq\/SkDfIapJyzjTPpdyERxY4Ab7YVainYq9bZLr2GqXkibXpN4V3lpV7pz8syqBsZADkAWRAMfD6CTIBcgGzADcIA4UCvZrldOiDNgHwGLScyAGKANolwAURCKEAwZANuID\/wwAMTPURTtkHL0hRv8tprr02Wnr3cRJxR71Vz1vQo4PzqQdLtGq9Nrym8q7zUa\/MPUtdeccr8cl0\/1gWrrrpqAvMtsD9s2223rT5qtFik8QaiAWvEfr4y4H0VGOGAZfeAlVkPcDB3gqaGWydHNkA2L\/GDcMhdpz8b2aQ9Rr6psFcJcRTWCk9ToiplvO1FaZNOk9xK3RhXpWXNBs7HC3jfB9\/\/\/vcTmCMFDj\/88OTVB\/j+C\/R4oMcDPR5Ej6fXA71K9HZl+5tN71emFf7J66+\/vvpY6ilPeUrIxtexBeadBTR+CHIgiiFmAJEQChANEA2EDbtCk9j+0pe+tEp3Sd6rVlVg\/GdsgcwCiAbIlomH4px83OMeN52QbTWGYPamTQunHPzzb6\/aVMHG\/8cWuJssMGPngNU1+81M2iwYRJmWhL1qUdbxdWyBubBAdcqNlTSLAhMT471qc2H0cR5LvgUmX\/ziFyfwHbfqLKl71ZR9jLEF5soCk7E0uuWWW07niUgnnHBCddCg33nZfPPNk\/c8ocBNJswGT7riRPj4OrbA0m6BGXOceVfZcYHGFpinFphXxLnggguqwx\/mqa3mbbHMT3fccce0yiqrpJVXXjk5keb222+ft+WdTwXzHtPZgWzntYzvz7rYbl4Qx8kkjiOyS9uy93wy7JJQluOPP74q5tlnn53szXL+nbfzlXD8p6cFEMXRWM58sLfNl82\/\/OUve8YROFTi7Lvvvmn77beX7gxgsAI6YsgeKPuiHKgQ3+\/4MaoXvOAFaf31158Rb1nzDGo\/e7PWXHPNah7qMBVnm11++eXLlPkGtZ09d3YWTExMJN+VaZNs2Ga8oRHHbt4TTzyxNr+2s6a9P1p99dWrYUZtAsuAcDb2c8qL\/VrM5HsqDyVbUviXBczGds7Fu9e97pU22WST9MpXvjI9\/elPTw74aLPbrIlz7LHHVsvZjsRVgTLDtrOmm74sLdNZWv3Dsp8vcH3K7rN2X\/Uasy+tNot6Dct2Ni0feeSRyRzbZlSfL0QeTddZE8d7Hwd72Mm64oorLpaP5WrH5iLWxMSdB3nYjW1oduGFFyYvXxeLtAwJhmE\/E1xPTIesmO+8\/vWvTxMTd9p6aTblbG13zTXXJL\/GEcNaIx+vVXze0Ga3WRNnpZVWSmuvvXYy1LLPLRX\/kGY2Z00XyS113mHYz1D45S9\/efItjJu\/1BmpoUKztZ3PaPTUl112WTK3sc3MNzzm4g1ZTotnTZzplBocs\/l+pyHJZUrcZj+H7em5jzrqqGSeY5wORxxxxDJlp7rKttnOA\/1DH\/pQ8oWAU4XWWWedZGTko7269HLZyImTZ9bmVmirI2164\/C7LHCf+9wnOaHmxz\/+cbKUGth0003vUhq7Gi3gB9TMbyzf29zsMPmJifZh7v8DAAD\/\/3y\/k+oAAAAGSURBVAMAqhYayzCErC4AAAAASUVORK5CYII=","height":124,"width":206}}
%---
%[output:50fb542d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAM4AAAB8CAYAAAAhHY+6AAAQAElEQVR4AezdC\/xlU\/UA8HVHSZEkFZWa0fTyqBFqiBq9CHmUSnkkCk0pKqZ3M0UMDT0oJfogpppE00OoNCEVhWZUzKgRMUWM9yOP\/3z3\/79\/\/zs\/997f\/F73nnt\/ez6zfueevffZZ+91zjrrsddae9wj5V\/BQMHAoDEwLsq\/goGCgUFjoBDOoFFWLigYiCiEU96CgoEhYKAQzhCQVi4ZExhoOclCOC3RUyoLBhpjoBBOY7yU0oKBlhgohNMSPaWyYKAxBgrhNMZLKS0YaImBQjgt0VMqexsDQ59dIZyh465cOYYxUAhnDD\/8MvWhY6AQztBxV64cwxioHOHcdddd8YMf\/CDe\/OY3xytf+coEfitT183Pav78+fHyl788ttpqqzSvTTbZJHbbbbe48cYbV3haJ5xwQvz85z9fofbavfCFL1zufp\/97GcDHu++++74+Mc\/Hn\/5y19WqK\/SaHkMVIZwHnnkkbjwwgtjr732iiVLlsTnP\/\/5+P73v5\/gC1\/4QnrYe+yxR8ydO3f5GXTZ2aRJk+Kcc86JX\/\/61\/H73\/8+nvvc56bz0ZrGS1\/60tS\/+1100UXpNkceeWQ87nGPSzh+0YtelMqG8scze+ihh4ZyafuuGaU7VYZw7rzzzlhppZVizpw5MXXq1HjBC14QT3va0xJMmDAhEdSZZ54Z66yzziihov3dPvDAA3HHHXekOf73v\/+No446KnEkXMlvZWDmzJmx2Wabxete97r44x\/\/mAbqpf32t7+dONeWW24ZmZOkyiZ\/Hv\/4x8fee+8dV111VSxcuDD233\/\/wAXh\/kMf+lBMnjw5Xvayl4UPlfsq\/8AHPpDKfLTe8573pPa4nmf0xje+MS655JK44IIL0ji22GKL2GGHHWLBggXx73\/\/O0kNn\/nMZ8L4dlvGWU899dR49atfnfr75S9\/2WSU3VFcGcJZffXV00tzzTXXxM033xz33ntvnHHGGUmccHSOsLxA3YHaxqP87W9\/Gy9+8YtjvfXWiw022CDNlUj6s5\/9LK6\/\/vrAFcCtt96aRFblN9xwQ+JQxFWEpmdtiFm\/+MUvEqdeddVV44c\/\/KGqlrDmmmvGWmutFYgiN\/zd734XEydOTETgfldffXXcdNNNcdJJJwWOpB6XuuWWW\/IlYXyzZ89OcznvvPMCEf\/mN79JhKkPDW+\/\/fbYdNNN0\/ie8YxnhH7PP\/\/8OProo9NYH3zwQc26EipDOJDo6\/TOd74zfbX22Wef8FXyBfTgjjnmmPCV7Uos1w3aV\/1Pf\/pT\/O1vf0tffV\/jT3\/603HxxRfHdtttF4997GMTvPa1r41LL7005s2bl8pxiyc+8Ynp6607OPHi4szEveOPPz4uv\/xyVYOGrbfeOulB3\/ve95L4hiD\/9a9\/hY8YLlKr1eKZz3xmbLTRRn19T5kyJYzHB+\/QQw9NbY899tg4\/fTTg6itIYkB96zVajFu3Lg0dh8\/xHvfffcFiC79VxnCue222xJ7\/8lPfpLENWIM8WPnnXdOYoiHUf\/F61J8LzdsL9GrXvWq9ALhqIgmNyAq1Z\/n8nyk3HthESDwsk+fPj1XNz3iFLgNrpMbnXzyyfH1r389iccHH3xw4oQ+UvQXL7x2fnsmftcDw8Z73\/ve8Gze+ta3xn777VdfvdzvVvNZrmEXnFSGcDwoXy9f1lVWWSUpr495zGMSCr1glFltUkGP\/DEfIpf5+jKzgiEY8NOf\/jR87YmmfivzcSGamT5OTDRCBIgOt2ZcUdcMtPva174W66+\/ftKrcjsi1Pbbbx8MFwgBEXoOxCvjM87rrrsucMp8TT7+5z\/\/SZyHboOT5PHl+nYcO3GPyhBOJybfiXteccUV8YY3vCEp05tvvnkS0aZNmxZvectbwotHdAPrrrtubLvttrHjjjumckREfM3GEaIconIkNrnW7\/5zYkzI99MvgmAIqG\/n3ocddlgSpeggdLDvfOc7wRiAGCcvMxowXT\/96U+vvyz9phs9+clPDpzzHe94RyLKK6+8MoluqUGP\/qkU4RA5ZsyYERTRa6+9Nh09MGXquv0Z0BHoJl5G5mGGAkq1LzsxhuilHvitzItu\/vSds846K7761a8GAlF34IEHhrYgt6\/HkXZ\/\/etfk3Lufn\/4wx+CPrXaaqsFIJ4ZE25Hx8Jd4Pu4446Lww8\/PHA0RK3\/L33pS0lPIeIdcMABAdzL+FgA9W18iBvxIVJGHXqOdix1xuO3e7q3MTjvRqgM4VA0vQi+nq95zWvSA3Z8xSteEcrUadONSO7WMVs8RUieAf2FSTpzvG6d00iNuzKEs\/LKK8eGG26Y1iuIIP1BnTYjNfHSz8AYgHPGGtwIx2J9G\/iqsdGiMoTD2oOr7LrrrskcbW2Dqw2w8PfFL34xWJLGxmMps+yHgcqdVoZwnvrUpwaF9Nxzz00r5NxS6ALAwh\/MkesdCxQMdBoDlSGcjIhsXkVIuYw1hylVXS4rx4KBTmKgcoTzpCc9Ke6\/\/\/60IGdNAZx22mlh8a0YBzr5qpR712OgcoRjofOII44ILji77bZbWt\/gq\/W5z30umD7rB19+Fwx0CgOVIRyiGLcaiOBBYP2AJYdFx5qCRTZ1uY3fBXoJA901l8oQDveaH\/3oR2lhzYKbYKuMSr85O1p00yaXl2PBQKcwUBnCYTHj4vGpT30q+GZZM+B6D17\/+tcn1xROn9p0ClnlvgUDGQOVIZw8IO7r2cWEmw3g+mEFO7tv5LblWDDQKQxUjnA6hYhy34KBwWCgEM5gsFXaDg8DPXR1IZweephlKu3DQCUJR7IHsShiPIRPiyq0ENo+tJQ7FQy0xkDlCMd6jliNgw46KMW5SxYhm4p0UepaT6fUFgy0BwOVIxz+aLwHJKGAAmHTTNPCi9UpK1Aw0GkMVI5w+KNZDJWeKCNHDLxkEMXlJmOkusexMrLKEQ7iEGkoY4twXPHy73\/\/+0P2FUQ1Vh5MmWe1MVA5woEukYficmSJlNzOAuhLXvISVQUKBiqBgcoQjuydvKFFfgIWNTnVRIDScdRpUwmslUGMeQxUhnCkN\/rKV74SsklKibTvvvumiFBRoUQ3WS5lWGn1xHAmCT7kHNOeWVtOMDmLlcm8csopp4TkerkfvnAF1kspeUcTD+M3fFmsdcCcBGt+6IIB7ycbj\/FIGu9d8AF1PhTIz3okjx0nHLmQJeiW2I41jRVN6ABESZsE5OuSDI+BoNnkJetDeNIQ2QVAdskvf\/nLIQWTdEVEP+HY1oWkWqrvhz\/cYEDSvcG0H622HRpHSt9bP6eBxrH\/134Zd7x+ZtxywlsSzJ26cSzd+aRH9VPfp+fj3DOTqN3zdN4Kmo0DselvJKEjhOOLL3sK7iDXlgQdgBc0kYwVTd7iPFF5jBENospl\/Y\/EuPHjx8fznve8VEVPQozieeQNE8\/DuCB7zmWXXZbaDPWP3ROGeu1IXleVcTSa0+xLl8T7Zv8lJh12Scy+9Ka49Zit+5ptOXGNQDw4jzZ9FcP80U58tJ1wxNYID0AIZ599dtojRsAawB1k3GcIIJ7ZgAm8\/e1vT9t8EOea4fb5z39+iByV5E66WAk+5C0QPfqsZz2r7zIJyhcvXtx3PtI\/vAheiLEEm3xlcfSf78xz\/x7X33pfXPHJzRP0x3MmHgTWv64bzttOOMzNn\/zkJ8OuBDhAfyR58XkJSN0qE6R8AzLdYNf1bYl3DAjY8Ec+8pG+KpxK8jz5kIVbWxMiAvY1aPDj73\/\/ewwGjKe+\/cVXLowNp1+YXp5rblwafzhwfFvgh7s8pi33GWg+jcZx1u5rx5e2W6MlXp+50m3xjTetnfD2ulmXBDx+7DuXh9\/H3bV1wukR51wb99xzTyxdurRlX55H\/+eiDDR45MMuajvhELcErRGtiGUsaPVAfKPEO+6+++6J08irpq1r8ozF5sybNy\/JyfSabAR43\/veF3IjSy6O4xDfpNPN1\/nNjSefO2LxgwFcMLf\/3jW1eOMpNwQCJY6c\/+HNI9eN9rF+HKN9r1b94+it6lvV7brlixJHuvneSHj8+u+XfXj+eZ\/HEjfe8WCc+qf745s3rBtrrLHGgHhtho8YhX9tJ5w8BzoH3YMYxnJmrxeOnTjLnnvumbaasJXhiSeemBIUauuafH3\/I5FMWPU3vvGNlBd55513TtuGSBiunG4jlavfkor3v34o5zPPXdwnvxNJhtJHNa9p76ieveYq8fbNmu+0t\/DOx8Xv\/vP49g5qgLt1jHBYwf75z38GImE5E\/lp2zzljARENlyFzxoO8vDDD6dt\/5rNZ9GiRWkLPVwKMUo2zhKDxcuYg8vpAzhv1s+KlGP\/WentJMEYx4qMtxvaXLzotpbDnLNw4Fe1nfgYeDQtpzP0Sk6bduRiMcu9EMUYDeRQA7ncFn+ILJ83OlokZbIm1gmzxmGIa4uWEdQuu+ySMu\/zRNhpp52CK0+jPlakDJehDGvbSaJx\/16Ci65d2lXT6RjhMAxYq8FxyKb0nLe97W1BXMMxJOyg+ANcxEJYK1EN1m3M9IQnPCGl0HUOEA4Z3G8wHKsaomEtostUgWjoDubUC0BcG+482omPjhEOJNm+41vf+lYccsghKXMnrwGbHFmwpMTnxS5rMWJyiFmuA\/2tanQcfm2C3hggtMkwXKvaO0+8NJhbT7vk+mTFIhIU+H9LJNwPFx\/7vHS1\/LgaHievde+AVrVmY2jY4TALO0o43\/zmN4Mp2fbfPAjoOPajtC3f4sWLk3KPQOgr2lnQzPOl\/9Rb1bSh+G+yySZp23FrQsQ25m9EmK\/zu96qZgWb6NUMdjl9SVz5rwfTAt6C6Vsly46+fN06DcMax4QJaS4jMQccfbj9fGC7jWLL565hSo+CNVd+KHad+PCA43Vho3EoH2noGOHY1sOWdxYts6mR2IYDzZ8\/P+08ZmsPYC2Hh0GrBVD1mUMhmsmTJwfOxbp2+eWXhyA44PekSZP68LjG2ftGK7jj5N0DEBkzMJXn35089to4Fsx6c6zy17kx7p5b+mDtJb+OV908O+irA+G6GT76HvYI\/ugY4XC7YSCgk+T5sKbxGsBxEJSF0Az0H2slue2KHulGFkrF9WyzzTbBiKAsX5+JrRz\/ltbEOo2HG396bPJny35tfz7jM3HSSSfFmWeeOazx5ec9UseOEQ5PZwuUvJXlEuB4acWfaCXngPUcBoMM\/RdAWyGAGMfrwLFWq6VFVPtb8p7eY489olartbq81BUMDIiBjhFOrVYLOg0rl3UVm7PiCvzYvvvd7wbftXqwSIrQBpxRaVAw0AYMtJ1wiGg333xzUvzpHNtuu21ixXZfFj6A+8g3IETAekyGmTNntlwAbQOuuu4WZcCjh4G2Ew6jwIEHHhjCCHbYYYcgilnPAaxgRx99dHzwgx8MBMRFJoNANKv\/o4eK0nPBwIpjoO2EQ9widgksQygClXi1AqEAxDYuOFOnTo3tt9++Dyj19J8Vn1ppWTAw6aP4MAAAD4FJREFUehhoO+HkqRDTcBWElMvs9cnL2XaGgtlyeTkWDFQNAx0jHMTRaK9PgW7WWljRiGdEOeCcbpQRiDN94hOfCGs+ueyCCy5Iot8WW2wR9CVpcxFiq5wD+dpyLBgYDAY6RjjcYKzVIABEwdWG68ZRRx0Vc+fODSv8cgdkyxrxLnMnPml77bVXEPfyZOlOFk+Zt7noEAMZGATEWQjVlljYKOdA7qMLj2XIHcLAuHbfl3gmGYf7rr766jFt2rRkevayH3744cHPjAXtoIMOimxRc5wxY0bQg4RFP+UpT0nGhec85zm66QMcTBiBAp7XHEmtD+VYHue8poebc0D\/BcY2BtpOOFb\/+ZTZz7PRXp\/KcIY\/\/\/nPsfnmmwdxTVgBgnPdYYcdFhtvvHHwMrCImh8fdxyci4eBtSGBcbysB\/KOliNgMPC\/HtKL823LcYxioO2Ew6RsH89me33iKjLUzJkzJ3hE8zWzOCo+R2AakazRs+IqQpdBdMQ8TqFCqomC4nAQoJDsX\/3qV0HvyX3MOWdeDAZmnf6zOPqsS5O3NI9pMJAPValfb8A8as1wxD1quHnV9J2f90gd2044eeAiPolfRCkvPeASQ0TTBqE4AoumjAaIwHkjcD2r3LOf\/ezkUrPppptG9qbm9Jl1HBxt1VVX7etitYuOin+fesCg4NJpmyZvaXE5OnLvTkCzPGLtHstojsOHkPuVj+ZA82o2Ds9opKFjhNNsIkQuXxli10c\/+tGk57zrXe8KhIAbqW907brrrhusbjmiVJQnY4K1H+IhcB0dip7k91BhwoQJy12K6wilJsYtVzEiJ8076T+O5i3HRk078VE5wvGI99lnnyB2+dLwHGBp4wBKdJNaSpv+IK\/a3nvvnczQzNFceOQqQCTCCPjB8Y4msinrf\/1Qz3EdINnExYtu6xPhENKOx1+ekvK1m6CGOpfhXmeeWV80d+CjkgFOhnuPqlxfScKx+MlDmj7C2satHFFAGh3JUTIPbTbaaCOnSTxDWMzXRD6OovkLJFYne0dbE0oX1P1pFjnYrBzn61\/31uc\/kvKI5Rxkb5i4Sjx55QdDnrWfL7gpNpx+YYL8EvXSURCg+YmQnXf1LWnO5g4yPhzhxLz7467VuVwTJa9aLP+PaEUcI5Z52Y877rggagleYxljCJDSlk7CoED0yj3Qd1ZkAXT8+PEhzRROwzjA6CBzTu7HEYENBuRIGKj9EbttHKe8Z7OQZw0smL5VANyp1wBRmFsG8zV3UI8nOJk7deNAaCJr6+ua\/aYLi9nKwY7N2ilv9lw845GGjnEcL771GgRiUsJviVCzZs0KhMLpkzmahY1TKO7DTUfbwSyASvBxxRVXpEAoRCMTzsorr6ybAh3AwJYT10gJCIm2uE+GbhPjOkY4uInkG8Jdx40bF6JBcRXrNfIP4DwC0a677rq0KHr33Xcnj2nPejALoPqaNGlSWNNBjH4r089QgWgx1GsbXjfEwqqMY7DDl9Fm2jbj+yyTODBCQjwIyXGwfWrfTnx0jHBYu1i4yK8mDYhuuIr9ccSYE6kk75gyZUogomxRG8wCqDUduk\/WcRAPYnQ\/ICLUXizNwMMA9fVce5SB+vJmv7UD9fXOQX1Zs9\/agfp651yU6sua\/dYW1Nc7B\/VlzX5rB+rrnQNl8OHYDLQD9fXOQS67++KTY8cHzon3r3ZBPHT7kmRkUXf88cf35Y52nsG1IJ87NhuH5zzS0DHC4f7CaibXs0TpFizF6Fj5RzgWLy1c2b0A4bCy4U44EYuZMlluPvaxjwUEsvHLmrPBBhsEXQg3g0wiYasFUH2JOm0GZGfQv14Z6F\/e6Fw7UF\/nHNSXNfutHaivd06mry9r9ltbUF\/vHNSXNfutHaivdw6UWS5wbAbagfp656C+LP+ef\/QuMW2bCSHx+qRtd4+s4+R6R9cCvweCkSYa\/XWMcNycifjHP\/5xHHvssYFYLF5y\/GT5yoBY6CiUe0o+Mcs1rGaScGy99dYhX\/TChQtD+lyGBguexD4po3gJOG+2AGocBaqHgWn\/J8q9+\/s3xq0PrFS5AXaUcIQByGiz9tprp5ef4k7HYXrm5YxQMvB0JqrxT8NliG6wiZB4DPC25l2A49RqtRC2YCHV18riJ9CeeMgI4fdQAYcb6rUjeV1VxjGSc+rfl1xr317cON9a\/7btxEfHCOf2228PXIOoRuewOGnNxou\/aNGiwDmIb7gNIiB+eelZyaSQYmXDRaTK5fCJqBCghVKWOmIa\/QWRMAjgbu6BcymrRzqRcDCACw6mvbajAWNhHDectG\/YrYDOOxAOm+Gj\/lmP1O+OEY4QAIuYuIW4G67\/zhEAfzXiFV82R5wD0sj0MnuaPIXfF4azKLO2hTK+aaecckqK5RGiQLFEJK0WQOlGBaqRU63RcxBPxepW8qp565cBLsKRU2IOyvv4ZQuVcg4oYzig3FPslzVNmwrJP2DfUIYERKG8VquFlLeIkEjGny2nt0WIzNzrrLNOIiTtAStbbuO8QMHAUDDQMY5DJxFbY32FL5qgNsRCv5Hphr+ZrQgnT54cojbpONowY1911VVhOxCKP2dOi6c2i0JAjAQQwW0HATIgCMVm5gZ+E920KTA6GBCy7rkSnc4777zRuUmHe+0Y4Zg3IqCvWNAkajENC4nGjYRKM0l7yelD1niEFxDbtttuu2SF41lghwPOnHQfOhEnUI6hZGLuPAgP8WQdhxVOH+5fYHQwYNcIgYokCOtnPlijc6fO9doxwiGG4Th0FTqIYKV\/\/OMfwYmTJc3mutZ1MnC74W1Qq9WC0s+3jf6D2KSTgkLuOcQ5RgaEx85fq7UtBa4hFFiGAR85uPcB5D5lmWBZcU\/97xjhQCZCYBn78Ic\/nPbHERqNxTNT83KuB4TAstZT2O\/Byfggkgy4UJkezxASg9+9BB0jnHokWmPhEcATlu6CoOrry+9qYIA+ycyfR8NiyaIpAQrPc863lgw40SIg7eiZRHG\/ewk6RjgWMq278BTA1rnL8CCgWNJ9egnJ3T4XHzPbrVhzw0HyfOgy\/OWIzSQCz9ASgSUFRhofQO0ZdfI1vXLsGOHUarWgvPMCYDoWU24dh0XNV6pXENwL88BFmPptQlw\/n3nz5gVDDb2UtEDXvPrqq8NCM+OMXHmMM55r\/XW98LtjhAN5zM8Q7cvFXUb4s3Uabjfqqwdjc0QslrwzrJllDHhe9FSSg7JarZYy2fACQUQ4kOQZrlPfa9AxwkEcWP8xxxwTrGAZeBGQnXsN0b04H5yIPtOLcxtoTh0jHLZ9LJxcTH7OILNnL8rEAz2IbqtnZvb86KTGbjGafiOGynmvQ8cIhx7DCMDm3+tI7sX51Wq1tJ8qPYeEYAGbZXTixIm9ON1HzanthCMMgAmTOw2i2WmnnWL\/\/fdP+dPkiFanzaNGWgoqhwG76dF\/mKLtDmERW367yg10FAbUdsLB4uVN4wHNc4DtHxE5B+q0GYW5rliXpVVTDPDw4AaVG7CmEa1lR7VYzZ0p1\/X6se2EQ6HkjGkhjTsN40CtVks5BQSn8Q7QptcRX+bX3RhoO+GwpuEwHDTlEWAIYBjIK83djc4y+rGCgbYTDmsaUUy2GTE47373u0NZ0WvGyivXG\/NsO+FAmwAzxwzcMgrHydgox27AQEcIp9OIKfcvGBguBjpCOPbEEbEpjwBfJnE1js4FneW8AsOdXLm+YGC0MNB2wmE5s2jWKDGDMnXajNaES78FAyOBgbYTzkgMuvRRMNBpDBTC6fQTKPfvSgz0KuFU6mFIIiIRYk7r65hTXFVqoIMcDI8Bic6FGHC54QVS34V5n3DCCfVFfb85hNoHiZ9bX2EX\/SiE06aHZZGXW0oGm2LlW\/MsFqefz7vhiFjOPvvsGGq8jahfIQkMQ90w3\/5jLITTHyNtPPc1njp1asj0IynjggULgtMrp0kpgH2VDUfyEnnmcCqpsKZPnx4sj77yjtr42vvCI0KZf7TVj0z+XnL1+pR+S8StpPQWnn3x5bVTJlBN5CaCOPjggyOvreEMwH0yyD\/Aw13QWi5rdsRdjQfYosUY7HckM6sUUlywml1b1fJCOG16Mvvtt1+KkGRyl0tu\/vz56c7i8mfPnh3iWGzlKG2vXGReXH588mjbA1X+OBZHLypCSBc3+ONayRhFX9qrlJeGrVI05Y2O82nja3\/ZZZeF6Nvrr78+5OJGXMQvyTWWLFkSN910U\/LqkL9bckF9ZNB+\/fXXz6fpWD9H80QUKnBXnNY8eYsIYDQuczF\/4QjadRMUwmnT07IVCXM7uPLKKyNv+jtlypTw5V68eHF4uXyVvXTSZNm9AReSX9tLVqvVgoeyALJmw\/bie0ERouTzCFH2Uu197WU95dXs\/sRDiTYQhdgoIQJ0FtxNGDuCEcrufv3jbHAr5frNUD9H83zTm96Uq9JuejNmzAiJIel7KhCvNFLdmD6qiwkH6nsHiCvSLCEWL50sMXPnzg0vJ\/Erz1RubeJVPu9\/JAIdeuihoQ+A+xDt+rfL5\/qSjjifL126NL3kNv3C4YiJHHIRVm4z2KPxn3baaWnDL5tQ1Wq1wXZRufaFcCrySCjLHF0RjBdNRtJZs2aFwDCilAhL5bgSIpOLDue47777Qjmdw1R8zXERHIEnuv2HiGzqGgFiRSAISKonGVMl4ZB6mP5E98IF+18rtVcrkbG+PXHQfIhoxpzrcDxjl\/Ezl3XLsRBORZ4UUeyQQw4JCr7c17LEHHDAAUFkYgQg9nBHQkC4ELHKuRRMLFt0IcRElEMMjsRAOc78bjZNu0DUarWgqNtbiNc6cc7LnN2inPe\/3j3oaV78\/nX15wgSt5HemHECETrSt+hR5iIGq\/6abvhdCKcNT0nUZKOXF2GAPAS6BaUexzjjjDNCnrJarRbymcn+g9sQdbystVotRMvSac4\/\/\/w48cQT48gjj0y5t3EN5YDY5ivv\/saR7+W+yry4kkIyC7uve9VqtWRR82LbYrJWe7RoxQKHM+FS+jBe\/eX+Hd3PWNTRs4wf+CggFnOVl204YqD7dAIK4XQC6xW\/JxENAeFgEhE2Gi5i2XPPPYMOhJAbtWlVxtQunATXbNWuqnXjqjqwNK7y51EYYA1jnXpUxQgWcLKVAB8Ha8UNWOmsydRqj+ZIAw2HTkcsxQ0HalvF+kI4VXwqZUyVx0AhnMo\/ojLAKmKgEE4Vn0oZU+UxUAin8o+oDLCKGOg44VQRKWVMBQMDYaAQzkAYKvUFAw0wUAinAVJKUcHAQBgohDMQhkp9wUADDBTCaYCUUlQwMBAG\/gcAAP\/\/OB5FPwAAAAZJREFUAwASc5NlfXFufQAAAABJRU5ErkJggg==","height":124,"width":206}}
%---
%[output:70f94d54]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAM4AAAB8CAYAAAAhHY+6AAAQAElEQVR4AezdCZhcRfEA8J6IiKgQIGq4\/gQ5IyDhiBAuE5H7DiBELlFA5FBQIFxyKMoliHIFEfkAAREhEm4QBSUicgmKIAmHQAQUgaAfBlT4769JDy\/zZndndmd3s7udL7X9Xh\/V3TVd3dXV1fWGvDX73wsvvPDWtttu+9bll1\/+1nXXXdchyDO7WA4yBQYlBYaE2f8qlUpYcsklw4Ybbhi22GKLDkGe2cVykCkwKClQZZy2aSPMnDkz\/OxnPwt\/\/\/vfOyTGBz\/4wQ7Tc2KmwECnQJVxPvCBD4Stttoq3HDDDWG99dYLG2+8cfjpT38a\/vnPfw50GuT+ZQo0TYEq47z3ve8NO+ywQ5g8eXK49957w0EHHRSmTJkS1lxzzbDjjjtGhvrXv\/7VdAW5QKZAP6VAh82uMk4xl9Vn8803DxdffHF46KGHwgEHHBDOP\/\/8uO\/529\/+VsyanzMFBiUF6jIOSvzvf\/8L06ZNC2eccUaYOHFieO6558L48eMDppKeIVNgMFNgDsbBLA888EA46qijwtprrx0+85nPhNdffz20qZ\/DXXfdFb785S8HIt3cQLCf\/\/zn4SMf+UgQtqo9RFF9Bp4nTZoUVl111fCHP\/yhVVWEQw45JHziE58IVm544VdPyyqoQaQe9am3JmmOV+nyyT9HQs1LT7Q54Wzm9\/T7+J2A55pm9vhrlXFo0jbddNOw++67hzfeeCN8\/\/vfD7\/97W\/DMcccE5ZeeulQqVR6vDHNVPCpT30qPPHEE0HYTLmO8r7\/\/e8Pl112WQTPHeXtL2mnnHJKWHzxxcNxxx031zZ56tSpUQllzLXy9+zJDlcZx0A5++yzw\/333x9OPfXUsNpqq4V3vetdse7ijGBWMDtJMDuZpXC9UJpQfDFdPEirg9D7lltuGVcNzwmnckWorTvhTziEAI5LL700zuZmcG2QVzwws8OV8sqjnvQuNHPpC7j++uuDQUeruM0225RWNvnhFcIDtzrgTXikA+3QHvkSmKjsHeFXj\/4nHMoA+MQpA684gG7FNLjVIQ2kNom\/5557wgYbbBD8vvCoR54E8sJ99dVXh2eeeSZstNFGcYWtzecdvto21\/YVLvXUQm0+7YVP\/fov\/z777FOis3h1p\/YKi3U8++yzYd11143jKOFUBl55E6Q0ZdHOWFl11VVjfdohXV60kq5OeGrbrbz4KuP4Ac1KL7\/8svgq+OF23XXXSFAzvFkBkRNiGR9++OFw1llnhWuuuSa88sorccBpDG2c2Y6C4bDDDgu1hFlxxRXjqmHvdOutt8YfDL4EcPihVlpppaikKOJPeYrh1772tfjj\/+c\/\/wkHtWkFpVk1wdChQ8OJJ54YRU\/xnQFCarM9nXprZ8KPfexj8cB4+vTpEZVZ08Pqq68e+zljxoy4YqtbPFroj2fgLAzN4FcP2mtfsa\/ywes3OPfcc4N8aLnAAgtIigAn3PXoLC9mWHbZZWNePzo664\/fEt2\/8Y1vxL2rZwfg0l944YXgN\/Zbp3wYEJLaNvtNU1\/lxwTqkTeBwVfMV6QJOionr7CWznDVtgUt\/vSnPykSzx5\/9KMfVceevGiS+qX98KIDeihkrBsrQu\/aLNQuElaKr203PPJqU5VxFEQwBMR1CXbbbbe4jN55551x5jr22GPDkCFDAkKaNZXzYy\/dJs6tssoqkcGkya+x8JjtzAoGyU033aRIBKKhh\/TDei6C9mDEhAP+Bx98MHz7298uZqs+aztC7bTTTsGPOXr06PChD30owh133BFFsPe85z3V\/N15mH\/++aMI9Ktf\/SruV4ToAKeJZMKECbFe9WsHWqQfTp5aQKMkJhoIVrn0A2Ie+dFQPvTwDuCEW5w0eRKd0drzhz\/8YVkDJocTbjMyOqKLNsYMs\/8YvOhoUJtADMbZSXMETz75ZNBX\/YNDfsyHFgZdypzyNUsT5bUZTri9a7Mx8NGPftRrQHNjTx9NjvJri37Ja4LHtDFz4Y9JKPXReK3tg6yp3bVp+ldlnEUWWST8+Mc\/jjONg88EDkP9ICeffHI8EPWjjho1Kq4sL774IvwdAg61BPqx\/GgdZu5mYnsM2E20dYujicFq4Bi8GNX7q6++GieaYqHULj9qMb722Y+MVmhWTKstl\/AV8yijbEd03mOPPapaUYNJfnUW8Xi2whFXKIgwpbiOAGPB1V5+E2Dtb5\/6UNu3juppNA3TEre1SdvqlUv1a5vJOeVJE2J6F8IBV7F\/VcaxnyE+4NYirLzyylExsPDCC8cZ9H3ve180ycHdw4YNg7cKGvzXv\/61+u7B8oazE5gFxDcDibiWYDNgvR+7Hj5t0SaAkICYYxaul7\/ZOLO7MmZ2xPdu5qvFn9qffixlasFgJSalVZM4lfDUlkv4ijgaoXNasTG6QaC8QWH18Zzgoosuio\/a4HfTphjRzp80e8sLrJwmlpS9qzRJ5dHWAPdOTMLUSVQTVwtELpDahTa1edK7thnL6f21116L0kp6FyY8+gb0r8o4MtQDg0F8Iiaim4UsXxhNmlnXsgY8S7NSWWINKnn8OLhW6L0RsARbii2lmCbVnUS89nBgfG3QFm0Cnq0ISZ2eBl9qX3u4OopP7TP4tNM78EyFr81A+9EiiRv1cNb+gMSzNEtjHM\/iTAJEhYQDTrhTP9A30RmdlEuDzoRj4jE4\/PgGBOZUd8InVJ8QpPZ7roXUV+3RLsxvUBvcxbwpX7M0gSP1odh39F1qqaUk1wX90a+Y2PYn0abtsfQ\/rTB+I31NY0zG1O56\/euUccxSxDOzoR\/EEm8GKq4cCy64YKBAICbgXj+IwXvllVfGvVAqJ578rFGNgFnriCOOiGKhGVLdjeKQT1u0CdAW7bvvvlG1jvAGu3Y9+uij7TYlTRrKG5C1GbUPM4oXegdmOJt1bQbS0QJNPCdIPwwxC9gDpHY5T\/Pj+9Hg0B95MApxMOGAE24\/vP4UaSQvpkqTBBzKwScvfKeddlrwGxugJkR0GjlyZEi0S4oHaQZVsc0UGvoKp7rQSXl0FpegGZqkMik0Xow3bVUHkfiMtkN5kk\/KUxvqj3Yoo58pvR4DaRt88qDL17\/+9ao4K629\/rXLODRTjz32WPjlL38ZcLcfxjIFikyjwiWWWCKYEaTd0bYJ92OKF3oXDxJBEcO7UD7xNnw67L0I4qTJD+SVrqx3IfCc0qTX1p3ajBhmW\/nBddddFzV7cKQ06Z6LdUuHtxbUCY8wpSkLh3iABtojXTvSezGfeDjkB+edd17Qb3jkK6ZRR8OVAG44lQPyShOPGTGfFcF7MZ+8qV9C7+q0KqV8Qm2QJo+2pHdtLr7LI07dtVCbD17tkQ9eZYXeawFO6SCVS\/i0xTNc0uRVXig\/SM9CtBFXrCuVFW9vb9KAA8CtDmkADvF1GQfx1l9\/\/biKnHDCCcHst+2224a7775bmQy9TAGrnZmTqKVqK4gfl0jivSMwW5ulrQ4d5RusaSYUe18iJlGTWGtPVRRX69GmxDgQnX766YHa2Wzsx7F0s1c755xzgvSEKHEqjsSZKT6HraWA2bE425n1zK7o31lN8sirTGd5B2O6cWv8WixIGMCzlakjepQYx8aRmLbGGmvMUQ7ziJc+R0J+yRToYwr0RfUlxrEhxYXUeYw+U6Puu+++8O53vzvQQoQW\/iOCZPhINBnJdGg9HZYYu1tY+Cu\/jHD5Pc+3bOSWGIe69tBDDw3EMptQDCM8\/vjjw1e+8pWqzVPLWtCGiBiS4YmopOiMDm3kaihfZ3gGSzq7SzQDSy40n6AlUGIcWJdbbrnAyJElgf2OS2y\/\/vWvo4m99Ax9R4Hbbrut7yrPNVcpUGIcZwTMaxjBXXjhhVHNfMUVV0RlwZFHHhmsPOzQqhjyQ6bAIKRAiXHsY+xtnLRT0TkEXGGFFaIqmtkNi2ZWvIOQVrnLfUeBua7mEuP897\/\/DUxUnLqyMuZjjXGgPc+f\/\/znwBcB8a3ZnnA\/xRw8nUU0Wz7nzxToCgWefmlWV4p1WqbEOLNmzYrWvZQExdLeiXH\/\/ve\/qyYJxfTOnol39OWd5cvpmQL9gQIlxmHxzCbo4IMPjrfjWBjTrLHhoTR48803w7e+9a2m+ubk2o27vffeu6lyOXOmQCsp8H8L96BWrVKpRK82e+21VyBa7bzzzoGhpesFhx9+eCDKuVbdaIccmjKU22WXXcJiiy1WtxjRMMOTUUTujA6uCneWJ6e\/Q8vTLr0pjjlMA+JLC\/6UVhw43c3ZbLPNwlVXXRUYCN5yyy3hc5\/7XLxB6T7L5z\/\/edkaAhaplAquGbRXgMVthqWj5XZndGBQ21me\/pneWP+b7dvaa63V3rDrVnxdxmERzcS6eJLt1ueNN94YaNyaqZGd1He\/+92wzDLLxLv4zOYZ1TWDI+fNFOgKBe6c\/kq48\/FXYtF1lxkaw1b9KTGOi0\/ObzgtIGK53+Hwk8jmeaGFFmqqbsaF6ZQaPta6WUnQFAlz5i5QgDbtlJufrJacMHrR6nMrHkqMQ2tmH8Nn9PLLLx+YWFMYsBa99tpro8atFRVnHJkCPUUBTLP1OQ9UV5t5XvxzWG\/ZHl5xqJ3nmWee6BGFsadrBNTQRDR3FTBWVzvMPN4K1NXyuVymQEcUwDAn3\/xUGHXCXcGzvBQC89\/\/Q48thdKKwzp6zz33jJ4fXZelmnatlojlaoGNfktbkJENHgr0QE8xiL3M\/pc\/Ehnm5IJ4hmmm7LdaGPLaiy2vucQ4ahg3blygSWNyQ4NGu2ZTz07NaiRPhkyBvqIAZnFFIDELscx7ag+GmbjJ0uH3R48JnlN8K8MS4\/D0QeslVBHV9IgRIwJHfs5iUry0DJkCPU2BxCREsK3PfiC4W0MUwzRFZtEOTJIYZuImI0T1GFQZB0Nw0kAN7RKbsKiOdg7Ds0irL7L1WM8y4n5FAQxC5MIMmKKWSYhgSbVc7FhiFiKZFaanGSbVXWWcdDedQw4X14RJjZzCCy64oEcusqXG5HBgU6CWOWoZhMglDvO0xyQYZcLo4eHsCSPDS6ePi+IYZmm11qyzX6LKOCkjJ4Pf+973gjDF5TBToBEKYIy0ahj8mMDKQbRKIlaROeSpxyDqwiDrLTM0JCZJK4pVBdOIl6+voMo4SVQrime1z0Q5+fqqsbnevqNAPaboiDGkJcZQtr2Wd8QgU\/ZfLa4smKS3V5T22pviq4yTRLUkltULmc\/IlwrnsP9TwKAGaaWwCTforRSg3mohHVMAK4by7VECY4Di6mHFsIIkUauWQeRvD9\/cEl9lnGKDHHr6vggfU1adddZZJ348tzuHn0X8+blnKWAgg8QMBnhHDIE5kghlEy4\/hgDwdNRagxxYFQCmAEXGIF4VmUO+uW0F6aiP9dJKjMPc5qSTTorffJkyZUr8ONIPfvCDeHWaxxA3OeshynE9SwEDGNw3Y1YwsEFiBitA7epQZAbpXWWItFIY7BgCYArMkFYMz+KBfKC\/M0ZncqDTrgAAEABJREFUv2aJcXyRjYtVLqKYcBPNfMSH0adPKzTyTZzOKs3pb1MAI4BmVoZ9rn4+YASQmAETNbI6qNXqAIoMMbHtsNCgxxAAIyRIK4V0DDFh9PBo9wUHfIMVSozjwHPeeectXR9gqzZYidRovzEBqMcIBnpxVUhapu6uDKltBjIoMoTBDjADKK4QRYaY2HZYiCmsEgCehDeH9SlQYhzXBhx2+vbmDTfcEFydphTgjNDKw1K6PqqBFYsBACYAZnWQxCNMAAx8TAA8g7RfwCxdWRVQ0uBNUGSG4z41LGqaMAKwMrTHEJgBYAYAb4bWUKDEOJVKJbBPwzgOPN3DYdG8\/fbbxyvUlUqlWrP9jm\/nfPzjHw8UCLziuLZbzdD2QNHAhIeSAbB\/4xW+LanX\/2MEUGQEgxtgAmDg1zJBPUYgGgH4Gu1IPUaYOFtMSitDLSN4L64OW418fzzbwAgAzkbrz\/laR4ES4\/gg7tFHHx2IbC6cuTrtHo4PR\/G5VqzaXsgXDSgRfB9n\/PjxwWW1Yh6auCWWWCJ+NZqKmyd42rpinlY9G8SJKTADaI8ZpAGrCMAEAI5G22PQguKKUMsIna0KE2eLSWllgK\/R+nO+vqNAiXHcwfGlAo46XGbjtMMNUE43aptJbOPIw6dApL300kshPXsHLsLZM803X+s8jMALDHKDHgMkBkmrg3jQKDMYsAATAAMZTJy9ImAAYAUgGgHPoLgiTKxhhP65KqBuho4oUGIcF9l22GGHMHny5HDvvfeG3XbbLfhUHqNPn8JzqS0htB8ievFnzKcAsW3jjTdOyTH0WRB4xo4dG0aPHh18Ns4qFBNn\/yHeNQNTH5wW9jj\/nnj\/AtMkBpmNbo5gsQXmCWssPl8g4oAvfHxosE\/4\/vjhAVy7xxLhvgNHhMm7DI\/w3c2HBnDI2vMF8Onl3wprD\/t3WPxdL0f438znGvJG00x\/msmbvdy848GmEbrZHswxIFr0UmKcWrxWIJfbiG5PP\/10Sdsmv5udjz\/+eHQrxe+0fY14oBxfBRQMnIAQ1zCitATU3o3AuxZcNPzksUrY6qJnw7WP\/CsVj6HVwgphrwCm7LdaNAL843Hrh1u\/OiZctPfoCCfuvFr40uarhB3WGxlh3VWXa8i7TCPt6408xN7eqGeg1GG8xQHS4j8lxrEa+A7idtttF9z+5IhQePPNNwdfL7DKpDbwzvnNb34zvlYqlUDEe\/3114NVJka2\/eGPzd4HA2HCTTbZJO532pKa+k8s2\/qcBwItVSqIWSa2iVKY5PdHj4naJswDiEgpXw4zBVpNgRLjWC18c5I2zb2ca665JtCWFRkmNYJYxzm7Q1EaNh4\/zYjF69VEM36onQNhyttvvz2KbAlHIyFRjLYL88iPYawqmGVi254iMwmqZOhNCpQYx3WCSZMmtcssxcatvvrq0Qn71ltvHcaMGRN+8pOfBOc9rA++9KUvBRo6qu3nn38+2CONbdvnwG8FKuLp6Bmz2MekPBPbVhgMY1VJcTnsAgVykW5RoMQ4zWCrVCqBmvo3v\/lNtGmjHODmFnOkOz32R75ucM8990R7NwqGWrV2e3ViGuJZSsc0E9tWmPSew0yBvqJAtxinpxt9wOWPVN38WGEy0\/Q0xTP+RilQYhz7EHsbYRGJ93rxxTytfHaQ6QwGzrSn8ZwhU2BuoECVcTAGdTFlAEtooe+AJrB\/oUHjIrc3Gn5KwT\/WWTuP7I0qcx2ZAg1ToMo4LAN+97vfBTBz5swYMqNJ4GtsBx54YK\/4IiiuNkS0rDVr+PecI2N+6TkKVBlngQUWiAeYVhVfFxD6gFQRWAVUKu8YefZUs4qrzYQWO8vuqTZnvIOLAlXGYUrDQPMf\/\/hHcMrv6vSRRx4ZisAqQL6eJBFNWtrbsBnLq01PUjvj7ioFqoxDRcxCgNNB1wTWXXfdUAvi5etqZY2Uc9iZ8h3WdmaTnnOYKTA3UaDKOKwAHFKyEOA72tema0G8fD3ZganTX47oadLyahNJkf\/MhRSoMk5qG79p\/KexKq0F8dJT3laHRTHt\/xaar9XoByK+3Kc+okCJcTjnsMdhVZrAtQDOO\/bYY48e1aplMa2PRkGutmkKlBinHgZGm+7l9LSXmyym1aN+jpsbKdAQ42j4U089FX2tuR7gvdWQxbRWUzTj60kKlBjHHsZepnZ\/w+GGL7NZfZppEEsDFtOjR48Oa621VnC\/x9WFWhwYJ8Wtu2xzH+hN5XKYKdBbFCgxTtrjuNE5bdq0YJ\/z2GOPxctnmKfZhnHkQRPHzs1dHO6mWErX4pn6+Nuf1Rbf6k9rw9l\/Ibd8bqRAiXE0ks2aFULonQtcq80f\/\/hHr02Bs5\/99tsves3hXpdpDyuFWiR5f1NLkfw+N1OgxDhEqwsvvDAwtdlwww1j2\/fee++w++67B3dsGIPGyAb\/LLfccmHxxRcPhxxySODYwzdEV1hhhVLpZC2Q1dAl0uSIuZACJcbBGFaGUaNGhUrlbbs0CgEOOexNMFZX+sGpoWvWLra58FbEwWtNel9pkdCnXmQa8ZzSl3myl5u51MvNggsuGFgPuEZArDKg+Qvgh4CzDQNfXKPAWJRTD\/ntdVgfTJ8+3WsV3nrfB6vPH1tm0X7ldaa3vcHw6dDbdfbn+uzRq4OrhQ+lFcfXpY866qjACSFf0ezVVlpppXDxxRcHh6AGfzP1s31ztRrzWc0oCGjtijiKioElB6PFQJEY+blfUKDEOFYZhpzEKV5rOCY08G+88cZgv9Jsr5KzDgyIYXj6pGgo4pma7dOK5MjP\/YACJcaxh7GRf+655wLtF\/W0s5tK5e39TrN9Itpx1kEd7ZIc3BizWTw5f6bA3ESBEuPY42y55ZbB3RsrjgPRBNw9Ebla2YE35x8WskatlRTNuHqDAiXG4Tj9qquuCj7FcfDBBwd+pBO4Oi29pxqWLQZ6irIZb6spUGIcPtG4uvV5j1oQL72VjXhz\/jb982yEA9liYHYXczBAKFBlHGIY75uPPPJI4P52gw02CLUgXr5W9v2\/w1ZsJbqMK1OgVyhQZRwKgOOOOy4su+yy4cwzzwwcr9eCePla2bLiipNvfLaSshlXT1KgyjisAzAFjZeznCuuuCLMmDEj0KoxyqSSdgAqXysbRDkAn6vSwgyZAv2BAlXGSY1lbnPsscdG5QCmEe\/k+KKLLgqnnnpq8FUCca2CtOJkG7VWUTTj6Q0KlBiH1swZDrGNcaZGsCDga63VN0DdwUkrjnr6OeTmDyIKlBiHmDbvvPOGF154YQ4yMJepVCrxekDogX9ZFd0DRM0oe4wCJcZxADphwoTwhS98IV4FoCBwa5PPgY022ijYB7WqNVachCvvcRIlctgfKFBiHI3ebLPNAuUAUe3+++8PVqALLrgg7LnnnpJbBtm4s2WkzIh6mQJ1GUcbKARYDrjQdvjhh4dVVlmlej9HeqshrzitpmjG15MUaJdxerLShDtZRXvvH4yjpRkyBULoNuM433HFmm+BzTffPNT6JXBrlJOP5DXH9Wl2cEXiZ6YpUiM\/9wcKdItxXn755Whl4Fq0KwNf\/OIXS34JaOPcWnzooYeix5wHH3wwin2I8\/TLswQhn+FEMuQ\/\/YgCJcZxwOm2pz1NWiVS6CKaKwapf+zWRowYUb3gtvLKKwefCXGnJ+V55ZVXonJhvvnmS1HVsKhVq0bmh0yBfkCBEuO8+OKL4corrwznnHNO4FvNne0Ed9xxRzTBSf1afvnlw4knnhiY4rg5evXVV0ff0lTaKc9rr70W+J4eO3Zs4HLqjDPOCFahItMsuXCZqVL5HGYKzI0UKDEOW7Rhw4aFZZZZpmEtGoeFn\/70p4PPHfoglUPU1Fn49tlnn\/ixKn7aMCHGTOnCBSqz5h7PNk8250Wltz3eZC83zf0+pCVjrNVQYhwHnFxBXXLJJcHGvqMKk1i3\/\/77B25uzz333LjiFMsQ38aPHx8tDqxMm2yySfQKWsxzyaTTAwVDhg07pYMvRmQ6dU6nRKPiOGvlc4lx7GF80vC8884LvtCGYxPU7nHMftdee22Qf\/3114\/MUds4otkpp5wSXLkmovFyQ2SjSXvp9HHh90ePCc\/efklUHFiNMjyRafFE62lQOy67+15iHBbR9jL1BrB46anS6dOnR\/UzL5\/p0pvLbo8++mhwKY7yIHm58bU3+xw3SK1ACQcGSs85zBToLxQoMY6GE9E461hnnXWCVQbDfOc73ymJbuPGjQtujPLBlq5Zu1694oorRrU0Jklebtzpufvuu8Nhhx0W3PlRT4ZMgf5KgRLjuI9z0kknBWKYqwSLLvq2Z00qZl45pffXzqZ25zBToLsUKDGOQ82\/\/OUv4YgjjghLLbVUGDJkSHAGs9dee0Vmkt7dSpNSgbXBeuutF1ySswfqLt6BVN7nUHbaaaeARsRfE1lt\/7glTvtPIZ91tXnyewjPPPNMVF6RpFpFjxLjUB9bVawwxUpef\/31qJ6WXozvyjOL61tvvTXcfPPNgYfQX\/ziF4Eo1xVcA7EM+rtti3FYZOyyyy7Rz53foNhfExzFTNqPsuAopg\/2Z8qos846K3z2s58Nzz\/\/fEvJUWIc6mj3bnzT5rLLLgsORK+55ppwwAEHBKvDQgt1\/2tpmMTX2eCyB6Jlc0ja0p71Y2TpFi6Fim6sscYagaKFVYb3BH6b4mFzis\/hOxTg93zHHXcMzfo8fwdD\/acS48jm3o3Zy0w2a9asMHXq1PDVr3410JBVKhVZugW0cezXEhKHrU899VR6HfSh27fuQHFYjxhcEZtgioxD3MU4xxxzTLT9MzhqDWyVHcyAWSiw+C33XaZW0qIu41Qqlegmyo8xceLEsO+++waVt0JMS40vWhekuBaG\/R4V+nRE7zfeeCMyzPnnnx8dq1D\/H3\/88XFl6ved7wcdqMs49h0YhR0aMxmzGjHNcyv6NGLEiGgHl3CxiRs5cmR6HfThIossEuwxX3311UgLoZV\/+PDh8d0fjOX8LK3co0aNijaDrZbl1ZWhTIES48ycOTMwt5k0aVK44YYbApHtlltuCfwOmN38oGU0zcWMGTMmPPDAA3FwwOfZD98cloGb2yEzhuD0Xi+FPo9iT+gd0BSRCEw63h06Yy7lvGfoWQqUGIcIQB7k0bNYtQ2qeD9OMb4rz6uvvnpwsMq3Ads1cqi4ruAaiGXQme3fpZdeGihRLr\/88qhOFS8OLLnkkoECh+0aJQJ3Xj4IVmSugUibuaVPJcZhGc3ujEaNOk9DnbH4ggGLAOniugOVSiV+jNcnDt0g3XXXXaOquzs4B1rZxRZbLLDCYG3ht\/Cuj1TTwLMJBw19e+j6668PDGrFZ5iTAu6Wsb1kZDxnStffSoxD7elrbL4wTZXnYM2X2Jwr+Gw7DZi4rlfZqpIZT6ZA31GgxDg2pnypmcU6gr5rcq45U6DvKVBiHCpQ+m9LmzsN5Odtt902OAR1lmDjCvq+6bkFma6sUTIAAAWlSURBVAJ9R4ES4zD3YOTpXs6UKVOCVYeIRtYmrjl467vm5pozBeYOCpQYhxGnk\/1DDz00cEpodeF03VlOq52u9yUJqNtBZ21gOMmYsrN8jaTbP26\/\/fZRo2hi6qwMo0SuiGvdaXVWrpXp2pDcezVCr2bqRlf7ZfjV00zZvs5bYhyiGnMPmrRi42rfi2k99jzAEDucpO7nc8GE1NPdIx206ndjTMqCpJVtdkU\/bQFaibc3cJUYxzkAKwFGnQ5Ambe7yOZcwcrTCnV0b3Qs1UHU3HTTTYOZTciey0znOjcwi\/LQ45m6nQaRely\/pfHcw9mIMmZF7oDlc62c9tHgTHWlkAMPh5M0kPaILC6sMGj68MMPB2neU35hsYx2MoQVD2677baw5pprBvXy60Ccbq\/N2miVchHRrVz1TZ48OV6D12aSg\/7IBxrpjzYkQBM4dt5558DLkRCtUroQXqsImtV7F9ffocQ4lUolGnP6kTlaRxjWA0QMd3Qqle4befYW0fyArB0cDDphd2DIzNw5lZuowCxq\/0Ycuuuuu6IjkeHDhwf7O2mueZttKUpOP\/302HSW3GZK5yzOUWLk7D\/qdBi53XbbBd5\/zj777IAp+ZdTN8tzZy7FFYf1hAG85ZZbRk9B2nnaaafFG7fSnn766eDqBf8O1113XfQI1F6bNUMZYiGLDwzGT572OjOz4mmjfI30R75aYJJFlOdc0mSKSTFzbb6B\/F5iHJ0lrjnVd+jpSrQfzCzc3648Ww0MEnda7N223nrrYMZk56WfCVxrMMCttgZaPVN9lshMgxw+Okiz\/7MyGMAJj9BVcoNziy22iM5LWESww3O9XHo9YIX+5ptvhq222iqwDsBcBx10ULxiTsPpsysspN3GZVLDeqOjNivjLo\/QZMDuUHu1Gy4hS+tG+lOvvWPHjg36BT98GNtViHp5B2rckIHaMf2iPjermn2tGERQkwCGkp7AICK+MG\/Zc88946yf0lKojMGxzTbbRLGP6Ae3fUvKI2TrV6lUItOEtn+VSiU49ceQba91\/2ufScneUgYDkoiHwTGSG7jii9BRm4tlXNfAKKmsG72VSiV+krKR\/qRyxZAIWqm8LXmYZLS3mD4YnvuccXqSyFaZGTNmBE4S3To10H\/4wx\/Gy3nFeolzZnD7IausQVtM91ypVKIXU7dWrRCAP2ximfQEBrkNORBHhDF4iX\/e60FtGe22MhK56uUX10ib5TN5aINnYLXyXqk01h9lasF9oRTnWV8xa4ozCcw\/\/\/zpdUCGA5pxrBL2CkQSopBf0KxOFPVsAAkNJteSDQDKA5vplCbdakGMY\/hq78OGD0Pyw1Argq2wwgoBLht6ODCsjT8jWbjqgTLEO2KxMvZOymlrvfziOmuzPIBYRcmjDa4nONjW\/qFDh8Y7V531B45asN9yNAEPOzqiKPqkfFZKHo6S0mDatGmBkiKlD4RwyEDoRHt9cA3cBt8lL9oyoXfxDCINIjM7Mc0tVwNA3IEHHhh9Z9OG0WYpd\/vtt0fXVuR52il7QJ812Xjjjeeo3oA54YQTopNGWifaSGdBrJnnyFh4UcaqaJ+FiQxMihgDsJBtjsf22lzLyFx8aSsgqqKDVcjkQTnSWX\/mqHT2i72WS3NoSDTlybVSqcRPX+qrbPaCtI72V1Z5n3cRT5vIMj5p3MT1RxjQjOMHGTduXLR+IFoRxbyLN4ho0jCSjTNNl5nxzDPPDEyMzKjy0iqKd+ZgVuUyyzsxzUUyAxC+IhhQN910U\/TISesGj\/SOrHRTGdo\/5zzaZG+CkZVTvvguvV6bMUixjNXP1QP3dbT5k5\/8ZNzfVCqV0Gh\/1F0ECgqfutRWzvkxvnTaV+BZf2j1TEhoSrOHhrSJjjc8y9dfYcAzTn\/9YVrVbpOF1cm+iYbRHo4KOfkzaFU9gw3P\/wMAAP\/\/MQe17AAAAAZJREFUAwAAo9aIWV3h3AAAAABJRU5ErkJggg==","height":124,"width":206}}
%---
%[output:5339cedf]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAM4AAAB8CAYAAAAhHY+6AAAQAElEQVR4AezdCZhcRfEA8J6IiKgQIGq4\/gQ5IyDhiBAuE5H7DiBELlFA5FBQIFxyKMoliHIFEfkAAREhEm4QBSUicgmKIAmHQAQUgaAfBlT4769JDy\/zZndndmd3s7udL7X9Xh\/V3TVd3dXV1fWGvDX73wsvvPDWtttu+9bll1\/+1nXXXdchyDO7WA4yBQYlBYaE2f8qlUpYcsklw4Ybbhi22GKLDkGe2cVykCkwKClQZZy2aSPMnDkz\/OxnPwt\/\/\/vfOyTGBz\/4wQ7Tc2KmwECnQJVxPvCBD4Stttoq3HDDDWG99dYLG2+8cfjpT38a\/vnPfw50GuT+ZQo0TYEq47z3ve8NO+ywQ5g8eXK49957w0EHHRSmTJkS1lxzzbDjjjtGhvrXv\/7VdAW5QKZAP6VAh82uMk4xl9Vn8803DxdffHF46KGHwgEHHBDOP\/\/8uO\/529\/+VsyanzMFBiUF6jIOSvzvf\/8L06ZNC2eccUaYOHFieO6558L48eMDppKeIVNgMFNgDsbBLA888EA46qijwtprrx0+85nPhNdffz20qZ\/DXXfdFb785S8HIt3cQLCf\/\/zn4SMf+UgQtqo9RFF9Bp4nTZoUVl111fCHP\/yhVVWEQw45JHziE58IVm544VdPyyqoQaQe9am3JmmOV+nyyT9HQs1LT7Q54Wzm9\/T7+J2A55pm9vhrlXFo0jbddNOw++67hzfeeCN8\/\/vfD7\/97W\/DMcccE5ZeeulQqVR6vDHNVPCpT30qPPHEE0HYTLmO8r7\/\/e8Pl112WQTPHeXtL2mnnHJKWHzxxcNxxx031zZ56tSpUQllzLXy9+zJDlcZx0A5++yzw\/333x9OPfXUsNpqq4V3vetdse7ijGBWMDtJMDuZpXC9UJpQfDFdPEirg9D7lltuGVcNzwmnckWorTvhTziEAI5LL700zuZmcG2QVzwws8OV8sqjnvQuNHPpC7j++uuDQUeruM0225RWNvnhFcIDtzrgTXikA+3QHvkSmKjsHeFXj\/4nHMoA+MQpA684gG7FNLjVIQ2kNom\/5557wgYbbBD8vvCoR54E8sJ99dVXh2eeeSZstNFGcYWtzecdvto21\/YVLvXUQm0+7YVP\/fov\/z777FOis3h1p\/YKi3U8++yzYd11143jKOFUBl55E6Q0ZdHOWFl11VVjfdohXV60kq5OeGrbrbz4KuP4Ac1KL7\/8svgq+OF23XXXSFAzvFkBkRNiGR9++OFw1llnhWuuuSa88sorccBpDG2c2Y6C4bDDDgu1hFlxxRXjqmHvdOutt8YfDL4EcPihVlpppaikKOJPeYrh1772tfjj\/+c\/\/wkHtWkFpVk1wdChQ8OJJ54YRU\/xnQFCarM9nXprZ8KPfexj8cB4+vTpEZVZ08Pqq68e+zljxoy4YqtbPFroj2fgLAzN4FcP2mtfsa\/ywes3OPfcc4N8aLnAAgtIigAn3PXoLC9mWHbZZWNePzo664\/fEt2\/8Y1vxL2rZwfg0l944YXgN\/Zbp3wYEJLaNvtNU1\/lxwTqkTeBwVfMV6QJOionr7CWznDVtgUt\/vSnPykSzx5\/9KMfVceevGiS+qX98KIDeihkrBsrQu\/aLNQuElaKr203PPJqU5VxFEQwBMR1CXbbbbe4jN55551x5jr22GPDkCFDAkKaNZXzYy\/dJs6tssoqkcGkya+x8JjtzAoGyU033aRIBKKhh\/TDei6C9mDEhAP+Bx98MHz7298uZqs+aztC7bTTTsGPOXr06PChD30owh133BFFsPe85z3V\/N15mH\/++aMI9Ktf\/SruV4ToAKeJZMKECbFe9WsHWqQfTp5aQKMkJhoIVrn0A2Ie+dFQPvTwDuCEW5w0eRKd0drzhz\/8YVkDJocTbjMyOqKLNsYMs\/8YvOhoUJtADMbZSXMETz75ZNBX\/YNDfsyHFgZdypzyNUsT5bUZTri9a7Mx8NGPftRrQHNjTx9NjvJri37Ja4LHtDFz4Y9JKPXReK3tg6yp3bVp+ldlnEUWWST8+Mc\/jjONg88EDkP9ICeffHI8EPWjjho1Kq4sL774IvwdAg61BPqx\/GgdZu5mYnsM2E20dYujicFq4Bi8GNX7q6++GieaYqHULj9qMb722Y+MVmhWTKstl\/AV8yijbEd03mOPPapaUYNJfnUW8Xi2whFXKIgwpbiOAGPB1V5+E2Dtb5\/6UNu3juppNA3TEre1SdvqlUv1a5vJOeVJE2J6F8IBV7F\/VcaxnyE+4NYirLzyylExsPDCC8cZ9H3ve180ycHdw4YNg7cKGvzXv\/61+u7B8oazE5gFxDcDibiWYDNgvR+7Hj5t0SaAkICYYxaul7\/ZOLO7MmZ2xPdu5qvFn9qffixlasFgJSalVZM4lfDUlkv4ijgaoXNasTG6QaC8QWH18Zzgoosuio\/a4HfTphjRzp80e8sLrJwmlpS9qzRJ5dHWAPdOTMLUSVQTVwtELpDahTa1edK7thnL6f21116L0kp6FyY8+gb0r8o4MtQDg0F8Iiaim4UsXxhNmlnXsgY8S7NSWWINKnn8OLhW6L0RsARbii2lmCbVnUS89nBgfG3QFm0Cnq0ISZ2eBl9qX3u4OopP7TP4tNM78EyFr81A+9EiiRv1cNb+gMSzNEtjHM\/iTAJEhYQDTrhTP9A30RmdlEuDzoRj4jE4\/PgGBOZUd8InVJ8QpPZ7roXUV+3RLsxvUBvcxbwpX7M0gSP1odh39F1qqaUk1wX90a+Y2PYn0abtsfQ\/rTB+I31NY0zG1O56\/euUccxSxDOzoR\/EEm8GKq4cCy64YKBAICbgXj+IwXvllVfGvVAqJ578rFGNgFnriCOOiGKhGVLdjeKQT1u0CdAW7bvvvlG1jvAGu3Y9+uij7TYlTRrKG5C1GbUPM4oXegdmOJt1bQbS0QJNPCdIPwwxC9gDpHY5T\/Pj+9Hg0B95MApxMOGAE24\/vP4UaSQvpkqTBBzKwScvfKeddlrwGxugJkR0GjlyZEi0S4oHaQZVsc0UGvoKp7rQSXl0FpegGZqkMik0Xow3bVUHkfiMtkN5kk\/KUxvqj3Yoo58pvR4DaRt88qDL17\/+9ao4K629\/rXLODRTjz32WPjlL38ZcLcfxjIFikyjwiWWWCKYEaTd0bYJ92OKF3oXDxJBEcO7UD7xNnw67L0I4qTJD+SVrqx3IfCc0qTX1p3ajBhmW\/nBddddFzV7cKQ06Z6LdUuHtxbUCY8wpSkLh3iABtojXTvSezGfeDjkB+edd17Qb3jkK6ZRR8OVAG44lQPyShOPGTGfFcF7MZ+8qV9C7+q0KqV8Qm2QJo+2pHdtLr7LI07dtVCbD17tkQ9eZYXeawFO6SCVS\/i0xTNc0uRVXig\/SM9CtBFXrCuVFW9vb9KAA8CtDmkADvF1GQfx1l9\/\/biKnHDCCcHst+2224a7775bmQy9TAGrnZmTqKVqK4gfl0jivSMwW5ulrQ4d5RusaSYUe18iJlGTWGtPVRRX69GmxDgQnX766YHa2Wzsx7F0s1c755xzgvSEKHEqjsSZKT6HraWA2bE425n1zK7o31lN8sirTGd5B2O6cWv8WixIGMCzlakjepQYx8aRmLbGGmvMUQ7ziJc+R0J+yRToYwr0RfUlxrEhxYXUeYw+U6Puu+++8O53vzvQQoQW\/iOCZPhINBnJdGg9HZYYu1tY+Cu\/jHD5Pc+3bOSWGIe69tBDDw3EMptQDCM8\/vjjw1e+8pWqzVPLWtCGiBiS4YmopOiMDm3kaihfZ3gGSzq7SzQDSy40n6AlUGIcWJdbbrnAyJElgf2OS2y\/\/vWvo4m99Ax9R4Hbbrut7yrPNVcpUGIcZwTMaxjBXXjhhVHNfMUVV0RlwZFHHhmsPOzQqhjyQ6bAIKRAiXHsY+xtnLRT0TkEXGGFFaIqmtkNi2ZWvIOQVrnLfUeBua7mEuP897\/\/DUxUnLqyMuZjjXGgPc+f\/\/znwBcB8a3ZnnA\/xRw8nUU0Wz7nzxToCgWefmlWV4p1WqbEOLNmzYrWvZQExdLeiXH\/\/ve\/qyYJxfTOnol39OWd5cvpmQL9gQIlxmHxzCbo4IMPjrfjWBjTrLHhoTR48803w7e+9a2m+ubk2o27vffeu6lyOXOmQCsp8H8L96BWrVKpRK82e+21VyBa7bzzzoGhpesFhx9+eCDKuVbdaIccmjKU22WXXcJiiy1WtxjRMMOTUUTujA6uCneWJ6e\/Q8vTLr0pjjlMA+JLC\/6UVhw43c3ZbLPNwlVXXRUYCN5yyy3hc5\/7XLxB6T7L5z\/\/edkaAhaplAquGbRXgMVthqWj5XZndGBQ21me\/pneWP+b7dvaa63V3rDrVnxdxmERzcS6eJLt1ueNN94YaNyaqZGd1He\/+92wzDLLxLv4zOYZ1TWDI+fNFOgKBe6c\/kq48\/FXYtF1lxkaw1b9KTGOi0\/ObzgtIGK53+Hwk8jmeaGFFmqqbsaF6ZQaPta6WUnQFAlz5i5QgDbtlJufrJacMHrR6nMrHkqMQ2tmH8Nn9PLLLx+YWFMYsBa99tpro8atFRVnHJkCPUUBTLP1OQ9UV5t5XvxzWG\/ZHl5xqJ3nmWee6BGFsadrBNTQRDR3FTBWVzvMPN4K1NXyuVymQEcUwDAn3\/xUGHXCXcGzvBQC89\/\/Q48thdKKwzp6zz33jJ4fXZelmnatlojlaoGNfktbkJENHgr0QE8xiL3M\/pc\/Ehnm5IJ4hmmm7LdaGPLaiy2vucQ4ahg3blygSWNyQ4NGu2ZTz07NaiRPhkyBvqIAZnFFIDELscx7ag+GmbjJ0uH3R48JnlN8K8MS4\/D0QeslVBHV9IgRIwJHfs5iUry0DJkCPU2BxCREsK3PfiC4W0MUwzRFZtEOTJIYZuImI0T1GFQZB0Nw0kAN7RKbsKiOdg7Ds0irL7L1WM8y4n5FAQxC5MIMmKKWSYhgSbVc7FhiFiKZFaanGSbVXWWcdDedQw4X14RJjZzCCy64oEcusqXG5HBgU6CWOWoZhMglDvO0xyQYZcLo4eHsCSPDS6ePi+IYZmm11qyzX6LKOCkjJ4Pf+973gjDF5TBToBEKYIy0ahj8mMDKQbRKIlaROeSpxyDqwiDrLTM0JCZJK4pVBdOIl6+voMo4SVQrime1z0Q5+fqqsbnevqNAPaboiDGkJcZQtr2Wd8QgU\/ZfLa4smKS3V5T22pviq4yTRLUkltULmc\/IlwrnsP9TwKAGaaWwCTforRSg3mohHVMAK4by7VECY4Di6mHFsIIkUauWQeRvD9\/cEl9lnGKDHHr6vggfU1adddZZJ348tzuHn0X8+blnKWAgg8QMBnhHDIE5kghlEy4\/hgDwdNRagxxYFQCmAEXGIF4VmUO+uW0F6aiP9dJKjMPc5qSTTorffJkyZUr8ONIPfvCDeHWaxxA3OeshynE9SwEDGNw3Y1YwsEFiBitA7epQZAbpXWWItFIY7BgCYArMkFYMz+KBfKC\/M0ZncqDTrgAAEABJREFUv2aJcXyRjYtVLqKYcBPNfMSH0adPKzTyTZzOKs3pb1MAI4BmVoZ9rn4+YASQmAETNbI6qNXqAIoMMbHtsNCgxxAAIyRIK4V0DDFh9PBo9wUHfIMVSozjwHPeeectXR9gqzZYidRovzEBqMcIBnpxVUhapu6uDKltBjIoMoTBDjADKK4QRYaY2HZYiCmsEgCehDeH9SlQYhzXBhx2+vbmDTfcEFydphTgjNDKw1K6PqqBFYsBACYAZnWQxCNMAAx8TAA8g7RfwCxdWRVQ0uBNUGSG4z41LGqaMAKwMrTHEJgBYAYAb4bWUKDEOJVKJbBPwzgOPN3DYdG8\/fbbxyvUlUqlWrP9jm\/nfPzjHw8UCLziuLZbzdD2QNHAhIeSAbB\/4xW+LanX\/2MEUGQEgxtgAmDg1zJBPUYgGgH4Gu1IPUaYOFtMSitDLSN4L64OW418fzzbwAgAzkbrz\/laR4ES4\/gg7tFHHx2IbC6cuTrtHo4PR\/G5VqzaXsgXDSgRfB9n\/PjxwWW1Yh6auCWWWCJ+NZqKmyd42rpinlY9G8SJKTADaI8ZpAGrCMAEAI5G22PQguKKUMsIna0KE2eLSWllgK\/R+nO+vqNAiXHcwfGlAo46XGbjtMMNUE43aptJbOPIw6dApL300kshPXsHLsLZM803X+s8jMALDHKDHgMkBkmrg3jQKDMYsAATAAMZTJy9ImAAYAUgGgHPoLgiTKxhhP65KqBuho4oUGIcF9l22GGHMHny5HDvvfeG3XbbLfhUHqNPn8JzqS0htB8ievFnzKcAsW3jjTdOyTH0WRB4xo4dG0aPHh18Ns4qFBNn\/yHeNQNTH5wW9jj\/nnj\/AtMkBpmNbo5gsQXmCWssPl8g4oAvfHxosE\/4\/vjhAVy7xxLhvgNHhMm7DI\/w3c2HBnDI2vMF8Onl3wprD\/t3WPxdL0f438znGvJG00x\/msmbvdy848GmEbrZHswxIFr0UmKcWrxWIJfbiG5PP\/10Sdsmv5udjz\/+eHQrxe+0fY14oBxfBRQMnIAQ1zCitATU3o3AuxZcNPzksUrY6qJnw7WP\/CsVj6HVwgphrwCm7LdaNAL843Hrh1u\/OiZctPfoCCfuvFr40uarhB3WGxlh3VWXa8i7TCPt6408xN7eqGeg1GG8xQHS4j8lxrEa+A7idtttF9z+5IhQePPNNwdfL7DKpDbwzvnNb34zvlYqlUDEe\/3114NVJka2\/eGPzd4HA2HCTTbZJO532pKa+k8s2\/qcBwItVSqIWSa2iVKY5PdHj4naJswDiEgpXw4zBVpNgRLjWC18c5I2zb2ca665JtCWFRkmNYJYxzm7Q1EaNh4\/zYjF69VEM36onQNhyttvvz2KbAlHIyFRjLYL88iPYawqmGVi254iMwmqZOhNCpQYx3WCSZMmtcssxcatvvrq0Qn71ltvHcaMGRN+8pOfBOc9rA++9KUvBRo6qu3nn38+2CONbdvnwG8FKuLp6Bmz2MekPBPbVhgMY1VJcTnsAgVykW5RoMQ4zWCrVCqBmvo3v\/lNtGmjHODmFnOkOz32R75ucM8990R7NwqGWrV2e3ViGuJZSsc0E9tWmPSew0yBvqJAtxinpxt9wOWPVN38WGEy0\/Q0xTP+RilQYhz7EHsbYRGJ93rxxTytfHaQ6QwGzrSn8ZwhU2BuoECVcTAGdTFlAEtooe+AJrB\/oUHjIrc3Gn5KwT\/WWTuP7I0qcx2ZAg1ToMo4LAN+97vfBTBz5swYMqNJ4GtsBx54YK\/4IiiuNkS0rDVr+PecI2N+6TkKVBlngQUWiAeYVhVfFxD6gFQRWAVUKu8YefZUs4qrzYQWO8vuqTZnvIOLAlXGYUrDQPMf\/\/hHcMrv6vSRRx4ZisAqQL6eJBFNWtrbsBnLq01PUjvj7ioFqoxDRcxCgNNB1wTWXXfdUAvi5etqZY2Uc9iZ8h3WdmaTnnOYKTA3UaDKOKwAHFKyEOA72tema0G8fD3ZganTX47oadLyahNJkf\/MhRSoMk5qG79p\/KexKq0F8dJT3laHRTHt\/xaar9XoByK+3Kc+okCJcTjnsMdhVZrAtQDOO\/bYY48e1aplMa2PRkGutmkKlBinHgZGm+7l9LSXmyym1aN+jpsbKdAQ42j4U089FX2tuR7gvdWQxbRWUzTj60kKlBjHHsZepnZ\/w+GGL7NZfZppEEsDFtOjR48Oa621VnC\/x9WFWhwYJ8Wtu2xzH+hN5XKYKdBbFCgxTtrjuNE5bdq0YJ\/z2GOPxctnmKfZhnHkQRPHzs1dHO6mWErX4pn6+Nuf1Rbf6k9rw9l\/Ibd8bqRAiXE0ks2aFULonQtcq80f\/\/hHr02Bs5\/99tsves3hXpdpDyuFWiR5f1NLkfw+N1OgxDhEqwsvvDAwtdlwww1j2\/fee++w++67B3dsGIPGyAb\/LLfccmHxxRcPhxxySODYwzdEV1hhhVLpZC2Q1dAl0uSIuZACJcbBGFaGUaNGhUrlbbs0CgEOOexNMFZX+sGpoWvWLra58FbEwWtNel9pkdCnXmQa8ZzSl3myl5u51MvNggsuGFgPuEZArDKg+Qvgh4CzDQNfXKPAWJRTD\/ntdVgfTJ8+3WsV3nrfB6vPH1tm0X7ldaa3vcHw6dDbdfbn+uzRq4OrhQ+lFcfXpY866qjACSFf0ezVVlpppXDxxRcHh6AGfzP1s31ztRrzWc0oCGjtijiKioElB6PFQJEY+blfUKDEOFYZhpzEKV5rOCY08G+88cZgv9Jsr5KzDgyIYXj6pGgo4pma7dOK5MjP\/YACJcaxh7GRf+655wLtF\/W0s5tK5e39TrN9Itpx1kEd7ZIc3BizWTw5f6bA3ESBEuPY42y55ZbB3RsrjgPRBNw9Ebla2YE35x8WskatlRTNuHqDAiXG4Tj9qquuCj7FcfDBBwd+pBO4Oi29pxqWLQZ6irIZb6spUGIcPtG4uvV5j1oQL72VjXhz\/jb982yEA9liYHYXczBAKFBlHGIY75uPPPJI4P52gw02CLUgXr5W9v2\/w1ZsJbqMK1OgVyhQZRwKgOOOOy4su+yy4cwzzwwcr9eCePla2bLiipNvfLaSshlXT1KgyjisAzAFjZeznCuuuCLMmDEj0KoxyqSSdgAqXysbRDkAn6vSwgyZAv2BAlXGSY1lbnPsscdG5QCmEe\/k+KKLLgqnnnpq8FUCca2CtOJkG7VWUTTj6Q0KlBiH1swZDrGNcaZGsCDga63VN0DdwUkrjnr6OeTmDyIKlBiHmDbvvPOGF154YQ4yMJepVCrxekDogX9ZFd0DRM0oe4wCJcZxADphwoTwhS98IV4FoCBwa5PPgY022ijYB7WqNVachCvvcRIlctgfKFBiHI3ebLPNAuUAUe3+++8PVqALLrgg7LnnnpJbBtm4s2WkzIh6mQJ1GUcbKARYDrjQdvjhh4dVVlmlej9HeqshrzitpmjG15MUaJdxerLShDtZRXvvH4yjpRkyBULoNuM433HFmm+BzTffPNT6JXBrlJOP5DXH9Wl2cEXiZ6YpUiM\/9wcKdItxXn755Whl4Fq0KwNf\/OIXS34JaOPcWnzooYeix5wHH3wwin2I8\/TLswQhn+FEMuQ\/\/YgCJcZxwOm2pz1NWiVS6CKaKwapf+zWRowYUb3gtvLKKwefCXGnJ+V55ZVXonJhvvnmS1HVsKhVq0bmh0yBfkCBEuO8+OKL4corrwznnHNO4FvNne0Ed9xxRzTBSf1afvnlw4knnhiY4rg5evXVV0ff0lTaKc9rr70W+J4eO3Zs4HLqjDPOCFahItMsuXCZqVL5HGYKzI0UKDEOW7Rhw4aFZZZZpmEtGoeFn\/70p4PPHfoglUPU1Fn49tlnn\/ixKn7aMCHGTOnCBSqz5h7PNk8250Wltz3eZC83zf0+pCVjrNVQYhwHnFxBXXLJJcHGvqMKk1i3\/\/77B25uzz333LjiFMsQ38aPHx8tDqxMm2yySfQKWsxzyaTTAwVDhg07pYMvRmQ6dU6nRKPiOGvlc4lx7GF80vC8884LvtCGYxPU7nHMftdee22Qf\/3114\/MUds4otkpp5wSXLkmovFyQ2SjSXvp9HHh90ePCc\/efklUHFiNMjyRafFE62lQOy67+15iHBbR9jL1BrB46anS6dOnR\/UzL5\/p0pvLbo8++mhwKY7yIHm58bU3+xw3SK1ACQcGSs85zBToLxQoMY6GE9E461hnnXWCVQbDfOc73ymJbuPGjQtujPLBlq5Zu1694oorRrU0Jklebtzpufvuu8Nhhx0W3PlRT4ZMgf5KgRLjuI9z0kknBWKYqwSLLvq2Z00qZl45pffXzqZ25zBToLsUKDGOQ82\/\/OUv4YgjjghLLbVUGDJkSHAGs9dee0Vmkt7dSpNSgbXBeuutF1ySswfqLt6BVN7nUHbaaaeARsRfE1lt\/7glTvtPIZ91tXnyewjPPPNMVF6RpFpFjxLjUB9bVawwxUpef\/31qJ6WXozvyjOL61tvvTXcfPPNgYfQX\/ziF4Eo1xVcA7EM+rtti3FYZOyyyy7Rz53foNhfExzFTNqPsuAopg\/2Z8qos846K3z2s58Nzz\/\/fEvJUWIc6mj3bnzT5rLLLgsORK+55ppwwAEHBKvDQgt1\/2tpmMTX2eCyB6Jlc0ja0p71Y2TpFi6Fim6sscYagaKFVYb3BH6b4mFzis\/hOxTg93zHHXcMzfo8fwdD\/acS48jm3o3Zy0w2a9asMHXq1PDVr3410JBVKhVZugW0cezXEhKHrU899VR6HfSh27fuQHFYjxhcEZtgioxD3MU4xxxzTLT9MzhqDWyVHcyAWSiw+C33XaZW0qIu41Qqlegmyo8xceLEsO+++waVt0JMS40vWhekuBaG\/R4V+nRE7zfeeCMyzPnnnx8dq1D\/H3\/88XFl6ved7wcdqMs49h0YhR0aMxmzGjHNcyv6NGLEiGgHl3CxiRs5cmR6HfThIossEuwxX3311UgLoZV\/+PDh8d0fjOX8LK3co0aNijaDrZbl1ZWhTIES48ycOTMwt5k0aVK44YYbApHtlltuCfwOmN38oGU0zcWMGTMmPPDAA3FwwOfZD98cloGb2yEzhuD0Xi+FPo9iT+gd0BSRCEw63h06Yy7lvGfoWQqUGIcIQB7k0bNYtQ2qeD9OMb4rz6uvvnpwsMq3Ads1cqi4ruAaiGXQme3fpZdeGihRLr\/88qhOFS8OLLnkkoECh+0aJQJ3Xj4IVmSugUibuaVPJcZhGc3ujEaNOk9DnbH4ggGLAOniugOVSiV+jNcnDt0g3XXXXaOquzs4B1rZxRZbLLDCYG3ht\/Cuj1TTwLMJBw19e+j6668PDGrFZ5iTAu6Wsb1kZDxnStffSoxD7elrbL4wTZXnYM2X2Jwr+Gw7DZi4rlfZqpIZT6ZA31GgxDg2pnypmcU6gr5rcq45U6DvKVBiHCpQ+m9LmzsN5Odtt902OAR1lmDjCvq+6bkFma6sUTIAAAWlSURBVAJ9R4ES4zD3YOTpXs6UKVOCVYeIRtYmrjl467vm5pozBeYOCpQYhxGnk\/1DDz00cEpodeF03VlOq52u9yUJqNtBZ21gOMmYsrN8jaTbP26\/\/fZRo2hi6qwMo0SuiGvdaXVWrpXp2pDcezVCr2bqRlf7ZfjV00zZvs5bYhyiGnMPmrRi42rfi2k99jzAEDucpO7nc8GE1NPdIx206ndjTMqCpJVtdkU\/bQFaibc3cJUYxzkAKwFGnQ5Ambe7yOZcwcrTCnV0b3Qs1UHU3HTTTYOZTciey0znOjcwi\/LQ45m6nQaRely\/pfHcw9mIMmZF7oDlc62c9tHgTHWlkAMPh5M0kPaILC6sMGj68MMPB2neU35hsYx2MoQVD2677baw5pprBvXy60Ccbq\/N2miVchHRrVz1TZ48OV6D12aSg\/7IBxrpjzYkQBM4dt5558DLkRCtUroQXqsImtV7F9ffocQ4lUolGnP6kTlaRxjWA0QMd3Qqle4befYW0fyArB0cDDphd2DIzNw5lZuowCxq\/0Ycuuuuu6IjkeHDhwf7O2mueZttKUpOP\/302HSW3GZK5yzOUWLk7D\/qdBi53XbbBd5\/zj777IAp+ZdTN8tzZy7FFYf1hAG85ZZbRk9B2nnaaafFG7fSnn766eDqBf8O1113XfQI1F6bNUMZYiGLDwzGT572OjOz4mmjfI30R75aYJJFlOdc0mSKSTFzbb6B\/F5iHJ0lrjnVd+jpSrQfzCzc3648Ww0MEnda7N223nrrYMZk56WfCVxrMMCttgZaPVN9lshMgxw+Okiz\/7MyGMAJj9BVcoNziy22iM5LWESww3O9XHo9YIX+5ptvhq222iqwDsBcBx10ULxiTsPpsysspN3GZVLDeqOjNivjLo\/QZMDuUHu1Gy4hS+tG+lOvvWPHjg36BT98GNtViHp5B2rckIHaMf2iPjermn2tGERQkwCGkp7AICK+MG\/Zc88946yf0lKojMGxzTbbRLGP6Ae3fUvKI2TrV6lUItOEtn+VSiU49ceQba91\/2ufScneUgYDkoiHwTGSG7jii9BRm4tlXNfAKKmsG72VSiV+krKR\/qRyxZAIWqm8LXmYZLS3mD4YnvuccXqSyFaZGTNmBE4S3To10H\/4wx\/Gy3nFeolzZnD7IausQVtM91ypVKIXU7dWrRCAP2ximfQEBrkNORBHhDF4iX\/e60FtGe22MhK56uUX10ib5TN5aINnYLXyXqk01h9lasF9oRTnWV8xa4ozCcw\/\/\/zpdUCGA5pxrBL2CkQSopBf0KxOFPVsAAkNJteSDQDKA5vplCbdakGMY\/hq78OGD0Pyw1Argq2wwgoBLht6ODCsjT8jWbjqgTLEO2KxMvZOymlrvfziOmuzPIBYRcmjDa4nONjW\/qFDh8Y7V531B45asN9yNAEPOzqiKPqkfFZKHo6S0mDatGmBkiKlD4RwyEDoRHt9cA3cBt8lL9oyoXfxDCINIjM7Mc0tVwNA3IEHHhh9Z9OG0WYpd\/vtt0fXVuR52il7QJ812Xjjjeeo3oA54YQTopNGWifaSGdBrJnnyFh4UcaqaJ+FiQxMihgDsJBtjsf22lzLyFx8aSsgqqKDVcjkQTnSWX\/mqHT2i72WS3NoSDTlybVSqcRPX+qrbPaCtI72V1Z5n3cRT5vIMj5p3MT1RxjQjOMHGTduXLR+IFoRxbyLN4ho0jCSjTNNl5nxzDPPDEyMzKjy0iqKd+ZgVuUyyzsxzUUyAxC+IhhQN910U\/TISesGj\/SOrHRTGdo\/5zzaZG+CkZVTvvguvV6bMUixjNXP1QP3dbT5k5\/8ZNzfVCqV0Gh\/1F0ECgqfutRWzvkxvnTaV+BZf2j1TEhoSrOHhrSJjjc8y9dfYcAzTn\/9YVrVbpOF1cm+iYbRHo4KOfkzaFU9gw3P\/wMAAP\/\/MQe17AAAAAZJREFUAwAAo9aIWV3h3AAAAABJRU5ErkJggg==","height":124,"width":206}}
%---
