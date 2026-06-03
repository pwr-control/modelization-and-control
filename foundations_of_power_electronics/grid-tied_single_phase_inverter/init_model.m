clear;
[model, options] = init_environment('single_phase_afe_ttype');

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
trgo_dab = 1; % double update
fpwm_psfbc = 2.5 * fpwm;
trgo_psfbc = 0; % double update
fpwm_cllc = 5 * fpwm;
trgo_cllc = 0; % double update
% t_measure = 0.648228176318064;
t_measure = 0.2;
tc_factor = 100; % tc is ts_afe / tc_factor
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
use_mosfet_thermal_model = 1;
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



%%
%[text] ## Global Hardware Settings
single_phase_inverter_pwr_nom = 225e3;
afe_pwr_nom = 250e3;
inv_pwr_nom = 250e3;
dab_pwr_nom = 250e3;
cllc_pwr_nom = 250e3;
psfbc_pwr_nom = 275e3;
fres_dab = glb_time.fPWM_DAB/5;
fres_cllc = glb_time.fPWM_CLLC*1.2;

hwdata.single_phase_inverter = single_phase_inverter_hwdata(application_voltage, single_phase_inverter_pwr_nom, glb_time.fPWM_INV);
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:95145386]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:8b784893]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:9dfbe8df]
hwdata.psfbc = single_phase_psfbc_hwdata(application_voltage, psfbc_pwr_nom, glb_time.fPWM_PSFBC); %[output:5b15adeb]
hwdata.three_phase_dab = three_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:6224a599]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:2505f764]

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
parasitic_dclink_data; %[output:8439afaa]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:49e6705c]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:6f30676f]
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
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:06528851]
psm_ctrl.kp_i = 0.25;
psm_ctrl.ki_i = 35;
%[text] #### Induction Motor Control
im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:63b6cc67] %[output:600f6b83]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:63b6cc67] %[output:600f6b83]
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
sogi = sogi_filter(omega_set, sogi_delta, kepsilon, glb_time.ts_afe); %[output:58e44192]
%[text] #### Current control parameters DQ PI
dqvector_pi.kp_inv = 0.5;
dqvector_pi.ki_inv = 45;
dqvector_pi.pi_ctrl = dqvector_pi.kp_inv + dqvector_pi.ki_inv/s;
dqvector_pi.pid_ctrl = c2d(dqvector_pi.pi_ctrl, glb_time.ts_inv);
dqvector_pi.plant = 1/(s*grid_emu.trafo.Ld1 + 1);
dqvector_pi.plantd = c2d(dqvector_pi.plant, glb_time.ts_inv);

G = sogi.fltd.alpha * dqvector_pi.pid_ctrl * dqvector_pi.plantd;
figure; margin(G, options);  %[output:0c53d3ae]
grid on %[output:0c53d3ae]
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
nominal_battery_voltage_1 = hwdata.dab.udc1_bez;
% nominal_battery_voltage_1 = hwdata.afe.udc_nom;
% nominal_battery_voltage_2 = hwdata.cllc.udc2_bez;
nominal_battery_voltage_2 = hwdata.dab.udc2_bez;
% nominal_battery_voltage_2 = hwdata.afe.udc_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage_1, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:54cd2728]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage_2, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:920312f6]
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
%   data: {"layout":"onright","rightPanelPercent":38.2}
%---
%[output:95145386]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V\nNominal Voltage: 690 V | Nominal Current: 2.324276e+02 A\nCurrent Normalization Data: 328.70 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:8b784893]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:9dfbe8df]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:5b15adeb]
%   data: {"dataType":"text","outputData":{"text":"Single Phase PSFBC: PSFBC_1200V\nNominal Power: 275000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls1: 1.212121e-05 [H] | Internal Tank Ls2: 1.212121e-05 [H]\n---------------------------\n","truncated":false}}
%---
%[output:6224a599]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: Three_phase_DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 1.200000e-04 [H] | Internal Tank Cs: 750 [F]\n---------------------------\n","truncated":false}}
%---
%[output:2505f764]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 1.548074e-05 [H] | Internal Tank Cs: 2.840705e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:8439afaa]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATYAAAC7CAYAAADv5bIbAAAQAElEQVR4AeydC5xNVfvHnyPVKMowbhMadyohU80UuUSDcs+geJNyfwslI12MIrmkiERKUfSqv0JFpVCKV7xIQrmMS0Mp12SU8p\/vYk3HNGfmnJmzL+fM8vGcfVt7PWv99t6\/WetZz3pWgdPmn0HAIGAQCDMECoj5ZxAwCBgEwgwBQ2xh9kBNdQwCBgERQ2wOvAVGpUHAIGAtAobYrMXX5G4QMAg4gIAhNgdANyoNAgYBaxEwxGYtviZ3tyBgypGvEDDElq8et6msQSB\/IGCILX88Z1NLg0C+QsAQW7563KayBgE7EXBOlyE257A3mg0CBgGLEDDEZhGwJluDgEHAOQQMsTmHvdFsEDAIWIRAPiY2ixAN42w3btwo119\/vdSvX19uuukmqVu3rnTq1ElSU1P9rvWLL74oS5Ys8Ss96apXr36OvieeeEJ+\/fVXOX78uAwdOlQ2b97sV14mUf5CwBBb\/nreea5t7dq1ZdGiRfLZZ5\/J6tWrpVKlSuo4zxn7yOCaa65R+aNvxYoVKtXTTz8tF154oTz11FNSo0YNdS43P6dPn5Y\/\/\/wzN7eae1yOgCE2lz8gNxfv999\/l6NHj0rJkiXljz\/+kDFjxqgWHa069jmHjB49Wq699lpp2rSp\/O9\/\/1NVglRef\/111fKrV6+e6JaYuujjp1ChQtKtWzfZtGmTfP\/999KrVy+hFXns2DF54IEHJC4uTq677joZN26cKg\/n77\/\/fnWuS5cu0qNHD5WeVmPfvn2lZcuWsnLlSlm6dKkqxw033CC33XabfPPNN\/LTTz9J+\/btZdiwYUL5aJnOnDlTGjdurPL79NNPfZTSnHYDAobY3PAUQqgMq1atkquvvloqVqwoV155pRw4cECRwuLFi2XPnj1Cqwo5ePCgzJs3Tzi\/d+9e1cKbN2+eIkKqSxq6kZ988ol8\/vnncvHFF8v8+fO5lK0UK1ZMoqKiBNLSCf\/73\/9K5cqVFUmhb+vWrbJv3z55+eWXVYuO67Tyfv75Z32LUL45c+aounz00UcCyX755ZeKOMmDhEeOHJHY2FhVvujoaCHfjz\/+WMaOHavKeurUKZIZcSEChthc+FDcXCRaRV9\/\/bXs2LFDtZpozTz++OPyxRdfSIsWLeT8889X0qRJE\/nqq69k+fLl6jytrSJFiqjWD\/WDbCCWatWqqe7s5MmTZd26dVwKWBo1aqTscHPnzlXdUwjzxx9\/lO+++05ohXk8HrnsssukZs2aGXk3bNhQKM8ll1wigwcPVmmfffZZeeONN2T\/\/v0qHS1RWp8ej0cKFCigyn7eeecJ5JqWliaImH+uRMAQmysfS2gUio+8QYMG6gM\/ceKEIjRdcrqgkJw+zrzF+A+hQJAIZJScnJw52T+OaWnRWqPVpi++8sorMnXqVIEkBw4cqFqSdHWxn0FIpGOfrjP73sLAR58+fYTWXGJiovTs2dP78jn72dXnnITmwHEEDLE5\/ghCtwCQB13KiIgIZVtjFBNCQz744APVWsK2xj7nDh06JHQ9qTG2MLp+kBSkiC2LLinXfAnppkyZIldccYWy6+l0dBFvvfVWYWADooIkaSHSfaR8lHPXrl1CS1Pfo7e\/\/PKLarlhW6Mlpsunr4fDNj\/WwRBbfnzqeajz+vXrpXnz5squFh8fr7qgSUlJ0qFDB9VFo2uKlCtXTpo1ayatWrVS5+kSdu\/eXcqUKaO001WF9NjSLYRU2FcXvX4YbND6yBfCYqDAK4nSPWLECNVVxAaGDfDNN99UgwWQZVz6oAKuIaVKlfK+Te1jm4uMjBRannfccYcizQ0bNqiuqUpgfkISAUNsIfnYnCk0NipsY5AF7hcMJGB0p2VEN42uJdcR9jkHEQ0fPlzZ29555x154YUXBALj2n333SekRXR675qRbsuWLcp4j761a9cK9rzChQsLQveTMmEHw8ZH6wwCmzRpkowcOVJoEUK65D9hwgRlJ6ML27t3b0HQRfkYwSVvygf5Qo6Q6OzZszNahoy0Uh7uQSe6KQPHRtyHgCE29z0TU6IgIYBzL0R34403CvYzXD50izFIKkw2LkXAcWJzKS6mWGGAwFVXXSXvv\/++6i7T4mP0NAyqZargBwKG2PwAySQxCBgEQgsBQ2yh9bxMaQ0CBgE\/EDDE5gdIYZfEVMggEOYIGGIL8wdsqmcQyI8IGGLLj0\/d1NkgEOYIGGIL8wdsqucWBEw57ETAlcRGIEEiQxA2hoCGCPuc45qdABldBgGDQOgh4CpiY04fXu3\/+te\/VIQFAgm+\/fbbguD5DanhZLlgwYLQQ9qU2CBgELANAVcRGxOiiRjx1ltvCYEAidZA6BikQoUKAuH93\/\/9X8Z8Q9tQMooMAgaBkELgLLG5o8zExmLeH3G0CGBINAfm6zEthi3HEB+Tp91RYlMKg4BBwI0IuIrYiEhK+Jq77rpLhWhmQjIhmAlxw0Tm8ePHC91VNwJpymQQMAi4BwFXERvxuog1z\/w+uqMEBiQWfps2bVRMfCKbEhDQPfCZkhgEDAJuRMBVxEZrjO4ooWQIXshKRAULFlS40QXlmDTqROj\/mBoYBAwCFiHgKmKzqI4mW4OAQSCfIeA6YiP+PYEJWVVo+\/btwpbBA85xLZ89H1Ndg4BBIBcIuIrYWDWIqKqEir755ptVtFS2BArkHNdIk4t6mlsMAgoB85M\/EHAVsV1wwQVCcEDcObISrpEmfzwaU0uDgEEgtwi4ithYWo1W2e23367cPfRUKqZTNW3aVJ577jlh2bbcVtbcZxAwCOQPBFxFbCVKlBBWF\/rwww8FIlu0aJFayINpVswT5ZGwCAhbIwYBg0CIIOBAMV1FbLr+TK1ilgFEp8+xdBrnuKbPBWu7d+9eYcUlI6sMDqsMBsH8Dvi2gvWdBpKPK4nt0ksvlZMnTwpLnKWmpgoya9YswWE32IMHAP\/QQw8Ja0oaucPgcIfBIKfvYGC3RHn1vlayL7mR2maXnm+LbywQUgpGWlcSG464o0aNEqZYderUSS2ICzhPPvmk4LwbaMVx6p05c6YwNYv1Il977TX5888\/VTbky3StsWPHCvNRrZT+\/fsrnXboAj+mplFvK+tE3nbpskuPnXVCVyi8FzMnjJJpPRPkmar75I2ae2RwzAGpW7G0UHbqkJVwjW+Lb0y9+Db+uIrY6GoybYr6MwOBxW5ZNo3FcFkAlxW7uabTsO+PsJo4i+Biu8Nux\/zTr7766sytZ38vu+wyiYuLs1Rat24tkE3dunUt1UM90EU0lNjY2LDRFY510s\/Kje9FbKXSUm33h1J+8cMSPbOTVN29WK5P\/0YiOyRL+Rd2KqnR5RGf7xcBLc5+XrZvXEVsTJ9auHCh9O7dW2B64q9pRNhfvny5ukYafd6fLSQGyBAjXVlcSdasWXPOrehLSUmRtLQ0y6Ro0aLSsWNHoattpR7ypnJlypRhY1l90IOgxA5ddujZ8dOvgqQePSWnLy4ha\/amyYpth9U56mqFuOa9SN0maelyaO5wSU3vZu7uW0EOLZkup4qUkiL3TJGS4zdLsSGLpFCrpPRzpbN9r\/iWfvjhBx6ZI+IqYmPEs0ePHvLYY4\/JBx98ICxwW7FiRUFuueUWtfAtk+JJEwha27Ztk7Jly2bcUqlSJQF4feLXeoNl5NoLpdUL66TlZOukw8ubpee8\/dL+pW8s1WNlHcIh79invxJviR66UrR4n2efdwJhnzTUP\/ndzbJv376gCT0QwnT9+OOPQcvT7\/J9+5XsWzhJUid0kdT+VZRAZmmFiosncax4Hv5M\/mw3Sn6tlhBQ2TD3YF\/T35jdW1cRm6483cLhw4cLLS2mUSFffvmlMLWKoJM6XSBb7Ha+0hdeMUY6NG8g9etUl6rRRS2TiiULS4USF0uNclGW6dDlR1f0JQUt14O+wHXlDuNg6WlQLUq6xpfNkKSEGJncuUaGLOhbR+Z2ry7T2pWWeT1rCscI6ZpcVVqmrT4sLV\/bK30X\/Cx\/XBipAp\/SYs2tlCpVSvAAKF26dJ7zyqkMtA5P7t8uBTfMl\/PmPSynR90kp+c+JAV3rZUiDbtJmeSlEj3xe4nu\/7qUafnvXJeHrjU2Nl\/fnNXnXUlswa50TEyMMO9U58t+jRo19KHadq1wJOPF9n7Jg7k\/oUMVSW4SJc+0rWB0eRGJvxgHE7+khAriLZ2vLS1a6lUuKvUqR0rdyyIkLqZw+j7HRTPSHxzfSJHivmOnJG7cOpmwfJ8QjSa3woAYf3hze78\/9xU8tl\/+WPWmnJjQTi4Y31T+eGOgyPb\/CvYyyAybWYl+M6TQlQ3zVBddFr45zD\/q43LgJ18QW3x8vKxbt07wgUPYr127tgNw26eSF4xw6myt1ooOO3TZpQe8ctIFKa5\/ND6d7GJk9IcpUnvESmWL4163yKkDKXJs2asZ9rIDk++WUz+lnENmkYnDFJm5pczBKkeBYGXk5nyuueYaadCggTRv3lwSEhKU7Y5zbi6zKVtoIKAJrlxkhLLRjv5wp6MFh8zOMf6nDwRQoBL9ZgitMiRcyYx6anEtsX3zzTfSrFkzRUi4Z\/Ts2VM56uqC+9piiyMiCD5rLVq0EPLRaXHwLVCggPJh++uvv\/TpsNwyerdz5041cmV1Be3SZZce8ApEV\/liEbKwXx1JSrfV2d16g8hObFomkBmjmMix9Fba+SVihK4lRBadbjfDflYw\/Vwg9QKHUBVXEhv+bMw6GDBggDCQgD2sZcuWwnJ8XPMFNqHFn3\/+eWGpvtWrV0ufPn1k4sSJappQTn5svvI05w0C\/iKQlG63o3tqdesNMoO86FpCZMwA4Bj7mCYztpCZv2UPt3SuJDbsYBhTWX4PwAkLjusHdg+ucS4rYcg8JiZGqlSpoi4T5uiXX35RbiIYMiMjI8WXH5u6IYx+wMoOuxeQ2aXLLj15qZNVrbfMZAap0VKDvLyN\/xxTfl9iJ4a+ymDHebcQ2zl1hXxw1sUXR1\/YvHmzsJALI0j6XOZt1apVhWk3hQsXlj\/++EOICMIwOlM6ymbjx0Y+djjo0g1gDiytTvaNWOcM7TS2\/RuUkVWD6kiZIgUFPzhsb9mVKcv3InXbP4z\/xzYskUItkyRyyCLBYRbHWU+lOGVyyC5\/u6\/hJ\/qDcdCFWv4WyKtLly6SnJwsa9euVUb\/f\/\/73zJw4EDV4tIpWdGKQQEceAcNGqRPC+uSJiYmytatW4X5pZAkLcCMBFnsTJgwQXAqhEytEkcdMYPoUGoVPuGW7\/knD8kLraKk53VF1cgpTr7vrdn5D0fXc96Lb7+S1JlDJHVYQ+UsS8uM2QDStL94es0WHGbT6veSw8Wq\/SMfN+HHt2QcdLMgGrqRzO1cuXKlLF68WBgUqFWr1jkpcdZlmhUOvNjVTp8+LUz67tevnzzwwAMyZcoU5fhI9xTfNX0z+9jt9DFbJqbjVFimTJlcOyXmdK\/tjpgnTwrd75zKldfryunTBl126QGPYOpK+IRBfQAAEABJREFUblND1gy5ViqUKKxmnsz8Oi3jHYs676QUP7JNotbPlvOmd1UOswXXz5ci5Wso43\/0rBNnHGb\/9bSUqdc24z7KmBsJZr2y08+3ZBx0YZZ0wUZGNA8i5yK0xlhTlAi62Ni4Rpr0pFn+p8vJPNJp06ZJ\/fr1BdscCf3xY2OQAgLEBmGV0BKl5WhV\/t75+qPLO31e9u3SZZcesAi2LmZN6JHTuZ+slxHDn5ZjL\/eRAw\/WkGPjWsqFG9+TohWuUmTGSKY2\/lOWYEqw6+WrbHxL2LX5\/pwQV9nYihUrJoxqzp07V1q1aiX33HOPiqhLVF26prhvREVF+cRp27Ztyr2DqBYQIwIZli9fXrmN5Cc\/Nl44M3jg81XJ8UKw8dPG\/64ru8vCfXdI923DZeuKxbLw4gTBXkYXs8g9U9S0phwLl4cEwa5XHopi6a2uIjZaWBj72WJ3oKUWHR0tCMHsvv76azWA4AsRWnUMMhBKnHBHCKRInpDdihUrVJcWkvR4PL6yMecNAnlGQBMZNjJcMpBDZ51l9TQmeXKzDC+WJI0WXSpT\/3s4zzpNBn8j4Cpi08ViJSpGQGmB6XM\/\/vijIjVIT5\/Lbsu0KXzfGGDQtjecduvVq6cGCXSgyezyCOVrjIIZB93cP8FA8fMmMh3yB1I7sWmZePuXRScvFe35X69yUcHvrUOdKDWxnugvuw+m5b7QftwZaL38yNKVSVxJbMQrw\/jYt29fZSvDXta5c2eh1UV3NSckscONHj1aDYGT1p9Ak6QLJ7Gzy2GXLrv08B5kpwsSg7BwioW8MhOZ9vr3x78Mv7cHby6nIonsOZSm5pziGkIZrJDs6mWFPqfydCWxAQbdSkY8CTnMegd0Lxs3bsylbAX\/tUmTJgn2OLqgJCb8EYZMRgjxkXMq0CR\/LbP0V7IwuCU6jaSpP3LZ4pC6TQVZxLUC0kIgLgTyQiCwHR08QrcSb\/8Dsx8T\/Mp0IEZsZdq37Py4ThKIf9lVUR5Z2q+69G9QWrmGMKl+2Zafcy63S9+dlJQU+cH4sUE\/fwstLoz+zPm88847VUuN5fg4x7W\/U\/5zb8mSJXLRRRep5fv0Vbq02TnoPhRzQMovGqIWp1D+Q8ManvEjCvL24KhmcvrFzvLzyFssyd\/Ksods3vdXkVRf0rWQpGo5G2SRYIuQFgKZafI6tnuzEHxR+ZMRgPGsTxlG\/z\/bnQnEmFvfMuzJvNeYW\/51dYQsvKuslCgk0m7aRkkOclBLu3zd3OXHJu74R3eT0dG3335bkJdeekluu+020a0uXUrsZ7iEaAdd3D3weWPCfGZbHG4W+r7M27EpJaRey05S+dqGyn8IHyIrpFB0ZbmwTCU1rG9F\/t55okuKlbW0PlqfXbpypadWEylyViKb3CvnSIdk5V6Ba4WWMuk2MCRy9Eb5Pd24X2zMxjN+ZMOXqeCL0fiTEYCxXt59ysqc9ZnM7N9Yt1q5jEn1Oqjl9uMRefZhQ5\/xY8v89dt4DCnRjcQBF2HOKA63ROQ4evRoRkm4RndVO+hu2bJF8GPTi6WwPiItPXx3cMrVN7Kf2UF3e91e\/3jJ9cserG1kr+lCuGWG9YOVp698wlFXXusUmThMMkuRht2Ui4XeYuhXwh+hCy8UbFJWC+8nf3gz69GT6nHsTXxlizC4kDlNoMe+dAWaT07pjR+bZpsctnv27Mmxz96kSROB5BBILS4uTojq0aZNGxNoMgd883KZl9wOnzm79ICFnbrQ50sYXFjYr44s6FtHvth+WLC95WVwwS318lXfYJ135eCBdxeTbiaCrY3gkAwABFp57qPL6k4H3UBrY9LnRwS0awjhy0d\/mKIIzmrXkFDG2ZXEVrx4cVmwYIFap4DWF\/LFF18Ifml0R7MDPDU1VXDmJe2RI0cEktTpTaBJjURwt4w22uEzZ5ce0LFTF\/r8EVpvuntK+ty03txYL+oSbHEdsbF+KFE5HnnkEWHIGGJCsJ8NGjRIiK\/mC4RTp04Jk9nbtWungkv27t1bXn31VWENUbqkTKpftGiREJEXFxBf+YTDeTu7HHbpsksPz99OXegLRCA4HHuTEgJfb8HN9QoEg5zSFsgpgd3XIZzp06fLt99+Ky+88II899xzSvBlw4bGiKmvMtFaO3jwoODv5vF4hNDiECShj\/SIqi8\/Nl95mvNhiUBYVCrJpoi9oQiW64gNx9yRI0fKM888o4JGEg5cS8eOHYXYar6ApuuJg+7QoUPl6quvlq5du8rhw4clJz82Sf9nAk2mhawzKN2rUJfcOm6XvEjkrXtqyLg2f7fe5ny13\/FnmWIcdNNZ5ex\/nBQhNbZMiaLlRYQOLZkddOmiMijA4ALdVOaX0o0lKggT5hlwePjhh1U0XYbTz6rJchNugSZ5sWj9srXaKRMdduiySw942anL20EX3bmRRuVEOfY2qxwh\/eZslpaT18narXv+EYwyJZ1w7HhWr732mphAk2epBt81uo6EGSIqB9E5vIVzpDmbXDL7sRHSKDY2VrSPGt1PVqXCMRHfNX0f+zqNPodtjvmppLVKMjtiWqWHfMNRVzjWKZjPCsfe5DY11Or1B06IWq3eO6hlMHWRV3bCt5Q50KT+1uzYuq4rqitNa4zZB3QrtdCK83bQ1Wn1FqdAmvTff\/+9OkUII2xqtOqI9nHs2DFB2K9du7ZKo3\/CLdAkbjHVq1cXPM0xGFspdumySw9Y2akr2E6zDatHqaghSemDCxOW75e4cetkzd405WxsV71iYmLUTCH9fdm9dSWxQT4DBgwQFj258cYbRQthh84\/\/3yfGEFidEkfffRRdQ+hwYcMGSJxcXECuRk\/Np\/QmQthiEBSPh5ccCWxQWgElyRs0a233ipaGFjgr1t27yBrJbz\/\/vtqyT26rnjEezweNZE+PwWaxJhuh28Zz8IuXXbpsbNO6LJScA1h5kJSeutNO\/a+vWKzGlywUq\/TebuS2HDpwJZGVzJQgJYuXSoMNtxwww3KURcXkNNnF3mhxecr0GSgetyenu4UpM7W6rKiww5ddukBLzt1oc9qSfJqvfWct1\/NO7Vap5P5u5LYDh06pBxsGQWFjCAqhGNGTH0Bhg\/bjBkzVIRcZiowAR47XX4MNOkLI3M+\/yKgW2\/BmnfqZiRdSWy01t555x01pWr16tWiR0bpWnItO0AZPMB3jTR0XbC7MbzNCCmGU45NoEnjs8a74SbhvcUEY0eZYstGqMWc29YsmhHUcsdPv6ruabD041ZiAk3CQl7CyCcjoHo0VG+HDx8uRNLFCdcrecYuXVhadTjyVqpUSebMmSOEFPfHQdf4se37h8+Tv\/5UvMT88WDr7z0q3b7AdJK\/HXoom526guHHRpn9EV2vv47uFx3UkqmIsU9\/FdSglsaPLYOW\/t5h5JO\/YDjcxsfHC91RwhbxF414ayNGjFCJcQlhtFM76DJZfubMmcJ8UHzVGCEdN26c8OByctA1fmxlch3I0C7\/Mrv04J+VX3Th+7ZmyLWSlD64EMyglsaPTVHUuT8QGK02yIYoHW3atBFaVETn6NOnj2BL447MDroQGy8kDr4ej0dw1mXSfOnSpVW3lnsQSC+zg64dfmxM+frPf\/4jhw8fVj5FGKitErrdxo8tItc424mfG96LpLODCzqo5YPv7Mw1drzTxo8NpslC6OtDZPoSy+UR+YPWlz6XeVuuXDlhcIHY8Vxj8js2OVp1OOXiH4ewn9lBl\/RWC10OmuhsrdZl8g8dBHgf7HsvfOOiBxcmd64hzDfNTVgk37nbe8WVgwfYyggOib0MB1tsbHfffbdqgWFj43pWMFWtWlW6deum3Dxw93j99deFkOL+OOgyCZ6ou1aKNqbaoQufPWyStBCtrBN526XLLj121gldbnsvKvyZIlObeqTCn7vU4ALzTilnoKLrldW3avU5VxIble7evbtgL2MiPDMPxowZI4mJiSrYJDMLSJNZPB6Pus4o6pdffil81PhXeTy+HXTLli2rpn7Q1SVApZWiJwXboYs1WGkFEATAyjqRt1267NJjZ53Q5cb3Iqlfd9nw2lApvGKMbHqmvWosUNZAhHrhjcA3lvlbtfrYtcSGc+7UqVNl2bJlahbByy+\/rFpjAMLgAttgCKBjy2P9UiOzxWBgMPB+B+ZOHJand4Jvi28sGN9qIHm4jNjOFB1b2nPPPSe4bDBQUKVKFbn44oulR48egs3sTKrg\/QI83VUjcWIwMBgE8x3g2wrel+p\/Tq4ktt9++01wpG3fvr3g7sH8z\/vuu0+13jD++189k9IgYBDIjwi4ktiY6M6oKKObuHTs2rVLWGv0+PHjKuJHfnxQps4GAYOA\/wi4kth0a42wQyyW\/Morr0jDhg1VYElfI6L+V9mkzISAOTQIhB0CriA2nHGZRYCwD8qNGjUSZg0wq2D+\/PmyfPlyGTZsWLZrHnCfEYOAQcAg4Bix4XBL3LSbb75ZWH3q9ttvl7Zt2ypfNQYLmDVQv359FYKINK1bt1bzPnHANY\/NIGAQMAhkh4AjxMao5xNPPCHMBX333XdFR\/Bgv3LlysKUqLp16wr7b7\/9tmghBJHpimb3OM21UEHAlNNaBBwhNgYHcLJloiz2NF1FXDmY5P7iiy8Kczc3bdokTGRfv369XHrppcrVg0EEnd5sDQIGAYNAVgg4QmyQE062dCuZNkUQSS3Y1vr166f81ojSMXz4cBV0khkIuHxoG1xWleGcjqCLxzPTqXAPyY8RdMHCiEEgvyLgCLFpsImgAAERM40gksRPa9asmRoBJSJHcnKyCjmk0+e0JbQRMxTwnGZeG7Y6jk0E3ZyQM9cNAuGFQJbEZlcVCQHORNmuXbsKi7fQ\/aS1lpKSIkTgWLJkiTz55JPKG\/7TTz8VbGyXXHKJz+IRUJLuLPG0PB6PsL4BU7OYNwqBQqR0fTNH0N27d69qFUKGRlYZLFYZDIL1HfBt+fxgLbzgKLERt0k74lJHuqbt2rVTocCjoqIUoeHyQYghYlZxndFU0mYll19+eUYUWNLhIgLZfffdd+I9tYOpWpAneQA8k3UDmdxr0t6Rq0nRBrfAcBvYLVGQ0b3aCfLqfa1Ey+IHmwuybmhT8ZZ9yY0kO\/lrVH3xV0o+Ey95Fb4tvjG+NTvFUWKj9cS0KVpsuHa0adNG6E4yL5TIHAwa4AaiBRsbc0d9AUQkjzvvvFOw2zHfjThXBFssUKCA+IqgC+iEEWKyLl1YK6V\/\/\/6q6HboGjVqlDA4Q4QUK+tE3nbpskuPHXWaOWGUINN6Jsjjt14pD8UckAn1LpRnqu6T12vukSV1d8ob6VuEc8jg9DRautcqKsit1aOk9U2xGXJ9XJzUa9nJp1Ro3FFK39BaqnRMkhp3PpqtRHZIlqykRL8Z4o\/sqNtbLt32ifCNic3\/HCU26sqMAlaWgtmnT58uH330kRAgktj2dEcJQaQFOxxdTe7LSghCSfeTmG3cj72uYMGCym2EqBB9rPYAABAASURBVLn6HvaziqALGVop+OJBNriy\/ENP+gsZzHPoIswP\/oDBzDervOzSZZce6hhMXbGVSkutImlyZdoWqbR2qpRf\/LBEz+ykpOLaF+Wmk+sloXJRqVuxtCKkKrd0ySCOMslLBSn\/wk6p+NbpDOFYS3R6Gm\/JjnQie00XT+JYKdLuUYlMHJYrKdKwm\/gjhRveJR\/9Ulh\/drZuC9iqLQtlkBkuHbh4MOI5YMAAtWYBhJRF8mxPsZgFocOx2xFenFYfzr8EnYQkGSFF2Kd7m21mFlxkQASyYWtB9udkSTefFizbcy5YcIAOO3TZpQeI8qLr1IEUQQ7NHS6p6V3D3X0rqO4hxyc2LZPzS8Qo4tKEFT3xe\/E8\/JkUG7JInYeYNHEUurKhIAXT76FceZW81Cuvuu2831Fio1u5YcMGoYtRtGhR4eOYNGmS4KjLNQnwX7ly5eS2225TASlpCXJMq41ovIQHb968uSQkJAguJZwLMHuT3CDgEwGI7NiyVwXygsgQjrmB7pwmMVpZmriCSVjoMfI3Ao4SGwZ+\/oJcdNFFGSXC7oaPG93KjJN+7ng8HunSpYswCordbPDgwUJeHo\/vCLp+Zh1yyRiU2blzp1or0urC26Ur+Hp8I+OvLgjtwOS7BSJjeyyd3GhtaSKji0iXDxLzrc2+K\/7Wy74SWaPJUWJj5BObGSGs6Toyejly5EgpXry4mKlT1jxwk2veEYDMvFtmdC+9yYwWmVuILO+1Dc0cHCU2j8cj999\/v+B+QQuN+aP4szGQkBsbW2g+AmtKTUuYrj1bazT8nSs67NBllx5qllkXZEZrTNvM2Ie8dMssVMgsc72oaziKI8RGFxSfNMIUYczHDsYMAVaVwteJ1htpwhFwU6fQQgBC060zupqnfkpRLhDaVga5hVaNHC+tLQVwhNgYGMAnDf80jP3ME8WfDWnatKmw3gHRcm1BIEyV2GlLsUuXXXo0me3oWU7ZzmideXc1sZmF6mtjF4ZO4+MIsWFXwyftww8\/FIiMye74niHz5s1TmGD0VzvmxyBgMQIQGeRFi4xBAOTQkukila6XyF4viWmdWfwALMjeEWLT9aAbSrcTotPnSpUqpdY14Jo+l9ft6dOn1Rql1113nZo\/ymBFuHd17bSl2KUrGHogMUQTmbaZQWoMAtC1xG5WcdoeqZj0tkQ2uTevr5+r7g8Ghq6qkI\/COEpsxFg7efKksH5oamqqILNmzZLff\/9drVLlo8wBnw4gukfAeZsb3IcAxIVAVJrAIC5NYrTIOOZ6ZmfZUBkEcB\/q7iqRo8TG\/E2ccxkRZX5nhw4d1LwyInoQjDJYUDG9KrvoHujB742J8dggrBSInFaqlTrI+\/Dhw7JlyxYhggrHVkrAulK3SVo2AuF4C+SE0D3c+dY4OTD7UeUICzkhEBYCYe3o4FF2MfaZDM71YxuWyLHdm+VUkVJS5J4pEjlkkUTPOiElx29Wx+fHdRJPpTjl86dxCrhOaWnn3K\/z8Xcbbu8F39IPP\/zAp+WIOEJsfNhMUKfGhCFKSkpSET2++OILwY+N8EJc02nYz4sQ4cNXdA\/yHRhbXK79dJD89VR9Sb2\/imVycHBNOW9sIzmUdLVlOnT5LdfVtZCknpWDPSLlgsdqyKGexTLO6WtZbvunY5yNQEjeAjkhh6b2kNNzH5KtC1+WTe+8KFtXLJbvNm2UtELFlZy6vK4wD1JJr9nCNCXPmB1ntr3nyJ\/tRsmv1RLkcLFqGVFgmIaXlfDuMXLPEpBZXQ\/muXDUhbkHty2+LyfEEWLDR23hwoXSu3dvoaXEGgi68uwTbohrpNHn87qldegrj2fX\/KIm9WJPsVKKNLxLzru+Y7quu5Ttxkpd5B2V0Duoeo7VvVMypEmSHMtCvkt4VvyVWfEzBMksI6tNEaTflfMEaVlmtsSW+zRDWqUfa2kZMVZif+ynpNWJ\/tJz941KZh6vLTP3RMn24xFKypQpI4FITEyMELePbSD35SYtdmXszMwhzs39gdxDfeyoF8EedDQbX9+dlecdITZGPHv06CGPPfaYfPDBB2ruZsWKFQW55ZZbhJYbzrqkCUbleZjbt2\/PyIr9zNE9jsffI5G5jHbg731F2j0qnqb9ha2\/97gpXa1eT0tO0uzeAeKvDHugm2QlL4\/oLcii5LaCbJrYWQ6Ob6Rk\/aPxomVB3zqCTO5cQ5ISYqRrfFmpGl1UMJC\/s\/GwTFi+XxJf2aIkeuhKiRu3Th58Z2f6+X3y02+i0pHWacHswh9ep8sRTP18c5h\/Mj46m3ccITZdRyLmsqYBNrAdO3YIwjzPoUOHSsmSJXWyPG\/j4+NVRF5GWhGnonvkuSIBZJCWbvMJx7mifx7ZJyUvEilfLELqVS6qpPO1pdOJrYISSG5hvzqK\/CBDSBDyg\/hIt\/tgmoz+MEVqj1ipZPSHO2XFtsP\/QNZO\/P6h3MIT4VqvzJA5SmyZC2PVMZE8THQPq9B1d76aAJMSzhCfJj3IDqKD5P795uYMknN3bYJYujDPKl8Qm8fjEeKgrVixQkX+IAKIx+MJ60dLt8KO+ZuAaJeuYOnxJjtadZDcjZWKZrTkaMEFSxf4uEnCtV6ZMc4XxJa50ubYIOCNAERHF5ZuKwTX6oV10m\/OZu8kZj\/EEHCc2BhSHzJkiFqnAPsagSYZGQ0xHF1XXDttKXbpslqPJjhacHO+2i9XJX+ufNNc93DzUCCrMcxD0YJ6q6PEhmPu6NGjVdgialW2bFn5448\/ZOzYscI1zhkxCNiNAIMSqwbVkTJFCqqRVLqmdpfB6MsbAo4SG1E+\/vrrL7n55puFlaTo\/+Piwcgl1\/JWtfx9N1gaG1vu3wHcRj5+MF7KRUYIgwu5z8ldd9r5XjhZc0eJDf+d8847T3777bcMDOiaQmo48WacNDsGAYcQYBQVcms5eZ1DJTBqc4OAo8TG+gaNGzcWVn8nLDirVRGjrWPHjrkODb506VIhvhvOgQ888IDQ+suP0T0Ct6Xk5vU5c49duuzSQ628dTGw8MX2w+mjpju5FNLiXa+QrkgOhXeU2CgbK0e999578uyzz6qFWNjnHNcClT179giReGfPni2rVq2SKlWqqGMT3SNQJE16bwT0oAI+b8be5o2Me\/cdJzZaWMOGDRPmyRE5t1mzZvLKK6\/kCjEmuzPnjrlzHo9HxV7bvHmz8l2jBRcZGanCITFXbs2aNefoYM4qEQn4i2al2BXFgcqBA1sr60Pe6LBDl116sqpT25pF5frLCyt7G9eDLeH2XvAt5bvoHrygyJEjR4RFjemK7tq1Sw0iLFmyRGhhYWcjTSBy+eWXZ0RtIJAkk+khO7q5jLjqvFg8BuD1MdsJEybIa6+9lnF\/MKM36LzsjOKgdZrtvqA90xdaRcnug2nSa9bGYOWp8gnH94JvKd9F94BIEP5KFS5cWIhusHr1aqFVxYDCBRdc4Je7B4vBMFWqYsWKMmjQIGEU8M4771Q+cXFxccILU716dTXiyiRjdPqSsWPHChEJaHlYJdSTFiWtU6t06HyLFi0q4EsrVZ+zamuXLrv0gJMvXfN61pSFm3\/NVcQQ8s1KwvG94FvKd9E9NLkQQZdouQCwcuVKISIAax5wjoEFnc7Xlonyy5cvV5Pnx40bp8iwXr16wtoJTKynW8voauXKlYWIHjof9jNH92BCPvojIiIsi\/oAaUOwVurQeYejLjfUqWH1KGF2wqB3U4L2nrihXvq9CdaWb4mGiv7m7N46amPjIx8xYoR07txZxowZI5dccomqPyGLeNjqIIAful19+vQR+vYEs6Sb26RJE7nhhhvyXXQPXlBasGwDgDBXSdFhhy679ABCdrpwASFNKLqAZFcv6hQukh2x2VJHCKxWrVpSvHhxNX2lfv36iuTwZwu0AOXKlROW80tMTJSGDRsKx7TaTHSPQJE06XNCYFKnGoILyIosQh7ldK+5bj0CjhIb06ZosbVs2VJoWbVr105Y9wAbFDaOQKvv8XiUywhzThnlHDx4sBDU0uPJf9E9GLULx3hsbqkT067wb2PCfCiRm53vRaDfbzDTO0psLDTC6Cfriz744IMyZcoUeeONN1TLzeMJ77BCwXyIJi9nECCeGxJq5OYMWvZqdZTYvKuKkZ+FMzDi\/\/zzzwLheV\/PL\/vBqqedthS7dNmlh2fgry5abQwmhAq5+VsvMAhlcZTYoqKihG4nS\/BhfJ4+fbqagZCamirY3kIZWFP2\/IMAgwlJCTECuRFqPP\/U3L01dZTYPB6PEIvt4YcfVj5sjGji14ZPmT\/uHu6F1fmS2WlLsUuXXXp4eoHqSkqoILTemHbFegpuJbhA6wUWoSiOEhuAEbZo69atsnjxYiHAZNWqVYWZArhrcN2IQcByBIKkAHvb+kfjlZ8bgSo1wTFbIUgqTDZ+IuAosUFeTKcaP368WnKPZfcQZiEQcDKnOjCq+sgjj8jGjRszkjL3lOge+K7dcccdQrc2P0b3iIiIUDMx2GaAY9EOOjAlsLVIhcqW\/O3Qg7Lc6tIT5onCC9HpFpwmOadHUHNbLzAJJXGU2AgpRNcT29pTTz0lWpKSkjKcdX2ByZxSFmhhRFWnYcBhxowZas4nBNm0aVN5\/vnn1dzTjz\/+WEi7aNEi+fTTT4WZCfo+u7ZM8Zo5c6aa6mWXTqPHGQQgOLqntOCS0u1v5SIjBJLDDlfsgaUC0bGuAl3WVSm\/yoy33jfvRRAflaPEhh2NQYLcOOPi0EvsNia+e+PB\/MjDhw+rU9gT0AGJMb0jMofoHoQ6slLWrl2rSHf+\/PkqrJKVuliRa\/jw4SrIgJV6yNsuXXbpCWadUr9bLw0u\/VEeqXtSPkgsJFObetRxwZ+3yluLliuyazdto0zaf6VcM3GnIjxIr+XkMwvKQHwIXVuEFp+WL7\/eJgjl9VfsxJAZQOpDzPLH2pOOENvRo0eFtQ6efPJJgdRat24tvXr1kqFDhyrhGmmyq3qdOnXklltuEUZWdbpixYqpCfAEqiSCx5w5c9R0LSJ8+IruwXlIj+gedF2tFB3twA5dtGaJsMDAjJV1Im+7dNmlx8o6JfXrLhteGyoHFzwhhVeMkaLv3iOXfJSk9iO2zJefvvyP7F\/\/ieBgDvGNfWeNIj9adwgtPi23vbpHkBZzT0hm4XxW0ub1ffL8viuk39IC6aO46\/MsbcYtEV9Cefm2+Mb0N2rX1hFiu\/jii6V79+4yYMAAYeYB3UpIjmOEa6TxBoHoHUTxIJoHUT28r+n9HTt2CF09uptMdOcePTmeeak6nfcW0BmFJTilkdliMLAfgzenT5S5E4fJvMc7KXl3UBNZ0Le2kve6lVMtPVp77GtZ0LdO+vUzwmislqSEGEEeahsrvqRjg6sE6dC8gfgrEFRWcm2jWyU74dviG\/P+5uzYd4TYWOeALuPjjz8udCcZPPB4PEK0DoTQPqTxBgCCgriI5kEa72t6n+uEgClfvrx4PB6JjY3wcR+eAAAJRklEQVSVX375RfCVg+h0Ova9o3sAPGGOjMSJwcC9GNxwdWXRUq9yUdHCIIWWpIQK6cSWvWgSDGSLr15upGzZsvqzs3XrCLExGkoLLT4+XrWwiOrBwAGjnHmpPZPe6doyg4F8sGlBkrTy1q1bp9Y\/YMCC\/dq1a5PEh9h3mlFbopuwUtdHH31kn2KjyfUIEP25U6dOQteYiDd8N64vtEsK6AixQS50Ndu3b69isN17772KdHKyq+WEGT5w3bp1Uy8C7h6vv\/66sKALrRDIjbUUEhISpFGjRkLEj5zys+M6\/nu9e\/cW4tBh2AUbO\/QaHe5HAL9O3KEwD9C7YaDN\/aV2RwkdITaqzjqibLWwBF+gLTZcRaZOnSo1a9ZU2Xg8HiFSyGeffabWOSAeW4UKFVS3FMMzxEHkjy5duqhz6iaHf2hhUkaCbkL2BAZwuEhGvUsQ4I9coN+ES4rueDEcIzbHa+6CAvDSsjZDRESEKg3kzjoQ6iD\/\/ZgaZ0Jg7969MnHiRGnTpo3Q+8DRPFMSc+gDgQI+zlt+Gt+yq6++WhjppKuIHw5bjuk2+hr5tLxgQVKAfY\/ug86OmRS4sbBCFuHLGQlmgMR7fQd87vDP0\/eYbXgi4M+7AYkRdJXlJCG19evXC8tLhiciwa+VI8TGqCajm4xiZiVcI03wq2t9joRcYiAE2wgtMK0RWxp\/gekKv\/nmmzJ9+nQhaCK+d99\/\/70K00R6BlL0PWYbXggE+m58++23ymSCqxJmF4\/HxCj0941whNj8LVwopqMVFhsbK23btj2n+JB1ixYtVERfYs5FR0cLk\/8ZzCDAJpGDabHyAp9zozkIGwQCfTf4I4ezOSPmuDBZ4ToRNuBmqoghtkyA5PWQaVvMiKhbt25GVoxuMSigZ0l4PB7VBd+1a5dAcrTgPvnkEzWTIuMmsxN2CAT6btAVxQWIwTCc1j0e02Lz96UwxOYvUnlMx19r7Gl5zMbcHoYImHcj+A\/VEFvwMf1Hjrhx0MXEGZeLGIaxr1WrVo1DI\/kYAfNuWPPwXU1s1lTZ\/lw9Ho9yCsbOxugoUQ+YHVG5cmX7C2M0ugoBj8e8G1Y8EENsVqCaRZ6sb4qNBVcPpsjcc889at3TLJKaU\/kMAfNuBP+BG2ILPqYqR9ZJZeK+Okj\/YX1TAmgSjgZjMNO60k+b\/\/kQAfNuWP\/QDbFZj3FoaTClNQiEAQKG2MLgIZoqGAQMAuciYIjtXDzMkUHAIBAGCBhiC4OHaKoQ6giY8gcbAUNswUbU5GcQMAg4joAhNscfgSmAQcAgEGwEDLEFG1GTn0HAIOA4An4Qm+NlNAUwCBgEDAIBIWCILSC4TGKDgEEgFBAwxBYKT8nhMrI+63XXXSc33XRThowcOdLhUuVdPbNAZsyYIYSVYpobUY29c6XeL774ovepjH2CGDz33HPC3N+Mk2bHNQgYYnPNozinIK47ICowU8G0PPLIIxllJFoJazdknAiBHcjs3XffzXUMPBbgIQwVIe1DoLr5roiG2PLdIw9ehWnN9O3bV60MtnLlSvnmm2+kdevWwkR\/1sOkVYO2pUuXCssh0uJjOcTk5GRhTQtaSWxJQ2uJFhIkSYx\/0pIP62lCQlwnT1Ybu\/7666VHjx5qyUZaTGPGjBHOEdyTaMQQ1sCBA4XFcsiblhXCvhbWHWCNCQJ96nO+trROKQ9Sp04doQzHjx8XAkGybOLJkyd93WrOO4SAITaHgA81tT179lRRf1lsp1atWrJx40ZVhYMHD8qcOXOE2HKTJk2SyZMnC8scQizjx4+Xbdu2CQuSzJ07VwjbBJFAVOrmLH64l4WCiSj8+eefC\/HK5s+fr1KyVCEtR9LQWlqzZo2wlgSLnCxbtkwgP7qXLIizf\/9+2bdvnyI\/FkIhvLbK5OwP6a+44oqzR2c23nWknpAWV2id0lKlnjExMcJ6FpSLulB\/QlCRzoh7EDDE5p5n4eqSTJs2TfTCOxs2bMhYy7Vhw4ZCyyclJUX4+GnVQAqsbv\/111+rVlypUqVUCHSPxyNEtiDopq\/KQkwQCERZqVIlRZTr1q1TyWktEfefSCmsJUv3l8VxIC0WEyYsFDYzWocsnr1+\/Xr57rvvBH2ZY9+xZifnVcZnf7zrSF3btWt39orIiRMnZPjw4dK8eXPB3sgFyDUiIkKOHDnCoREXIWCIzUUPI5SLQneMpQUhM0iBlbcWLFigSIXupa7b77\/\/nq3BnS7e4MGDM0iU1ltyetdV3595S1eUVZz0+cOHDysSaty4sWoh0g2Oj48XiE+nCXRL+WfNmiUlSpSQu+++W60cFWgeJr29CBhisxfvsNVWoUIFOXr0qEBoEMH7778vzzzzjAqmSVeRqMGcp1UHCRYsWFCt2JWWliacx+YFOLSGaIXRoqKVNGzYMKFLyrWsBDKliwvBsbzhfffdJyycU7t2bWXHw\/ZHKzLzvaVLl1ajoZnPZ3VMd5f60AWltajT0GKk7Jdeeqk+ZbYuQcAQm0seRKgXA3vTQw89JAwA3HjjjcLKW7179xa6hAwS0K1jIWwIji4g3UaOWXaQVb2wxUF2dFUhK7Z0c1l3lX1f+Nx6662qBYUhPzExUdq3by90VyEbvSA3x5nvRwd2Qogp8zXvYwiT1tru3buFwQtIki32Pux41IWWnPc9Zj8ABCxKaojNImDDKVsiAWdFLhAXouuKbQujPy2u2bNnS3R0tCKdtm3byurVq5UN7u70rhxk4vF4hCXlsKl9\/PHH8tJLL8nTTz+tWnG0ujiP0C2llYR+yqF1oZdzEMuoUaMEtwv0osvj8agRUYinVatWqgz6Pr1lBJWWHa088qC85Kevs0UfZeEadj5amwikDZlR1xYtWuSpm4seI8FHwBBb8DE1OTqMAC4kEBwtwNjY2CxLA5l17dpVsMFBtFkmyuYkriy\/\/fab0OrMJpm55BACBRzSa9TmUwQYzWR00crqlyxZUhYuXKhagNkNGjDKik+axxP4QsTYFOl205q0si4m79whYIgtO9zMNYOAQSAkETDEFpKPzRTaIGAQyA4BQ2zZoWOuGQQMAiGJgCG2kHxs4VxoUzeDQN4RMMSWdwxNDgYBg4DLEDDE5rIHYopjEDAI5B0BQ2x5x9DkYBAIdQTCrvyG2MLukZoKGQQMAobYzDtgEDAIhB0C\/w8AAP\/\/kOo3EwAAAAZJREFUAwAokYJPwOOYsAAAAABJRU5ErkJggg==","height":187,"width":310}}
%---
%[output:49e6705c]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:6f30676f]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:06528851]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:600f6b83]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:58e44192]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATYAAAC7CAYAAADv5bIbAAAQAElEQVR4AezdC9yuU5k\/8PX4z0SZQdOJIW1ESbKlqS0yDik1fFJ8opJIB5FoQgeECBlNBylNB3Smg5pKOTVbYpxTpCgKTUWJSUqfov\/+3lx71r7f+zk\/z\/s+7\/uu\/dm\/9173Wtda61rXWuu6r3V8lvtr+VckUCRQJDDHJLBcKv+KBIoEigTmmASKYptjFVqKUyRQJJBSUWwz0ApKlkUCRQLjlUBRbOOVb0m9SKBIYAYkUBTbDAi9ZFkkUCQwXgkUxTZe+ZbUJ0UChY95JYGi2OZVdZfCFgnMDwkUxTY\/6rmUskhgXkmgKLZ5Vd2lsEUC0ymBmcurKLaZk33JuUigSGBMEiiKbUyCLckWCRQJzJwEJkqxHXjggWnttddui5e+9KXp97\/\/\/UildfLJJ1f5\/fM\/\/3O6\/fbbh0r7mmuuSRtttFGVnnI08Vsv43nnnTdUnqOIPEoZ9MLPOPIjRzLPMYo67aU8eb3jo5c4aEIOeNZupMM\/R7SXpraU0zW5pSdd6ffDV1NadT\/9EE\/S\/vjHP75Mu+dXx6jzr\/NTf58oxVZnrv5+ySWXpH\/5l38ZWgE9kO74\/\/7P\/\/xP+sMf\/rA0I43hF7\/4xdL3SXaMs1OMutw6\/2te85opyd56661p0aJFiQKZEjjDHj6in\/3sZ5dycffdd6eLLrpo6fukO37605+mH\/zgB+mxj31sWmeddbqyq34GqQd1S0lSovpP14weJJhIxaYxfv\/730833XTTUvzHf\/xHxbLG+qUvfalyT\/ofvCpH8BmNYY011kh\/\/\/d\/H94z\/tx7770rOV9wwQXp0Y9+9Izz0w8DFPC5555bRTn44IOrcmg3PoI6nYAPfehDCR33OLDhhhum733ve1Xez372s3vKQrvQPnLib3\/72yMfkeTpj9J92mmnJcr4JS95SfqHf\/iHpUnrp+QfUE79GQFFTqFzjxsTqdiaCv2YxzymrTIIrU6zA\/O7qSHzE4YGxGvKix\/TGU2g32HNi170Ismkn\/zkJ9XTn9tuu61qDOuvv77XKajnKe8mHut0aHwN0QefeVnRoxEeQB8McPOPuN5f8IIXVLyi8bWNL2akE+\/CQRxpkK+8+QE3P2EgPv8m4BNNIPhpog0\/Vo4ORomFzIVR0IcddhhnVQ6yr14e\/IOPyMcTj3h9MHjpo86TeFHW4E888aWDfmnkDo5vfvObVSi+o+OzgHz8qoA2fygG+coLH\/jhDnTKvxttXo5Ir17P2MLD5ZdfXvXHzTbbjFdb\/N3f\/V36x3\/8x8bwen5kyA+xPJQzjBgfqqc85SkpyocOffDpmfM6axRbNGCFfvzjH+9Rfd0UJgpfeS75o6HrmCp+yWv1n0D4Cas8lvwR7\/jjj1\/iWva\/BqAz576+rttuu23PX\/7VVlutqvj8KxyNeb311suTrtx4recpAI\/KGGa4ctTp0DSVQ3xAj4Y7gF6e8T6OJ15rMk\/4kHc9v2Flrn5YB3m6rKewHLiFkSN54sN7QLvAay4T\/JNd0HiK18S\/sF6h01IM6P\/pn\/4pvfKVr+SsFLB2Xr308Acf+MlJ8ZuXIcL4N9EqIxpP5ScH7wEKpT79Q87kvcEGG6S11lorSBufeVlXX3319LCHPayiw2M9P3nzw0tF1OaPcHToc5Kc14lUbBiknWnhgEpUCMONaKSf+tSnElr+YQITenwBYwhCuEcddRSyak5AHA1enMoz+0No0QDkhS7SJMhjjz22UqhZlEbnqquumlZZZZUU82w6lPk1X2hfmjySMAqQH97lJ9\/gL77k\/ZRDWgF5Rpm\/8pWvVApXmDzlzZ3D0DSnw8dnPvOZ5Oub03Vy98PrMDJnMcSwXueN9kJRNvE3jjbTlE8nP\/VLMaDZbrvtkraujry3qxNhdSi3etJW1G+k0TTkE4YGrTjiSs\/EvzYQH90mOrziGT1acbgp5HqbyOtAXWjP4kv3ve99b9WGWFv6pjRY2XgCbn7RVy9YMjUSftLBg77fC68TqdgUrh2i0nQcbnQKr8DcBP3Wt7616rwUkS8ggRCucMMTwxRuccTlDsTQUUVEmDRVIppQMtydYOjsa3zXXXclwyCLCJScr5b08rjeKQ6V6+k9D1cOafRTjjy+eZAosy+sL20ePg53P7wOI3PzW+9+97unFMHHSccC1gGCcbUZafeDvGNSaupGW5FGr+0LrRGE8nNLQz1za+vkzx0QhsZ7Uxs44YQTqjlCyiTo0AaijgyV8ah\/4D3Cuz3xpE7QacvaNOX6ile8glcFbn5o6\/xXBA\/+6YXXiVRsoZ119ACrQbkUmvVGOJQGvxiacgOlwlrijgrhJjRh3IF63KCXDz50DPAlEkeFyJu7G6SNPleu5htWXHHFxqg6oLwCkWeduJdy5HHwEe+UJh7ifdzPXngdVuY+UNFO4mOUl0t7IVv1No42k+fVzU255sPQUCJRR9FeuqUjnLXnGYg04j2ennlYuzbAco6259k03NOW8UgRB+\/SD+inUReeFJR+JJyVxlqL+paOPOQF3PzQBg13E7rxOnLFdt9991XzUF\/\/+tfTOeecUw3Fmhjr149wQDwN47e\/\/S1nT+gmpJ4SGYAoGpP8QRL1xsgPDJ10QO4YAudDBv6RBvekY6Z4ja+5TpXLzxDvj3\/8Y89iGxf\/OrqPJkZYMDo0RN3zx6shH\/d0geKPD6mPA\/kZurLMggc84c3HinUV\/p2elGiMdigtirETfS9hvfA6sGL785\/\/nD7xiU+k5zznOWmHHXZI\/\/Vf\/5X+93\/\/N73qVa9KNO9+++2XzNVsueWW6V3veldC3wvTvdI88pGPrOaw0NcbYf3LHAqGYIWJE6jHDX9KVCNUwXWwEIKu05OprmFQxNEg6haj+L5isWXBF4\/c+OMVz9wwSDnEGwcMqw2v26U9CK+DyNwHgWKwgsYayvmpD7lYGE2WvDhknVtzg\/AvnU6gGGJ+qhOdoZ4hXycaYfW2G+8UT1M7E6cJv\/nNb1JM6\/io+jigy2XinaIDUxlky28Y6BvSq\/cv79EH6umr4154HVixnXHGGenII49MKuCGG25IBxxwQDLpd\/HFF6cXv\/jF6Rvf+Eal+Lbaaqv0sY99LKGvM9nPOwGAOMxgwjVv4N2Xj2nKrfGY4KcQVLDJ5VAwwvFIONziiMsdCItKXhD+nTpQ0NSfVoDMqflCS0un6tbgonEqR70T9FOOOi+jeo8Or0wUv3TJMxqbd+iH12FknvNTr0syB\/wYfi9YsCCNo81IvxfoK5QW2rCKdOIApSJM2+3FsomhnTg+jt65e2ln6JoQ7U\/YaaedVq3UckPMDW6xxRbVIgC\/bsjbcfRHfZJbG8rrjCXmI2VxTXm6pd2J14EUG2YNNSmYyy67LF177bXViQBMajgUyxOe8IS0+eabp5NOOik997nPTWeffXa65557pvDa5KEx6hgKGQgzmUDCDN5tt92qneXSEI5WPPH5ve51r0smV32pLRrwI0yWAVpx+OVgjWl0\/ISjA2Xjp3zS4+4GZrgOFXTk1RTX14+iRmdIIr+8HPxBXPlzdysHmlGBHGyRUO+hSKTNH6\/kiR9+AbxOh8zzNhCywxPgDz8sg1AaOb1wdLms8zYzallTVpQWnkKZcweiw3v3ofDB4G4HaRkdKYOnd7TkTv7cvcDoR9tEq51LD7j5gbo38tD\/8MmvCSFT8SGXbSx26JPkLH5eZ9z8go47oE9Ly8e0G6\/iDKTYDEEMRQwzH\/7wh6e\/\/du\/TbvsskulxSmzVqsl7QrCdNqbb765Z8VWRWz4owNpHAQjmOKwihiKiF8gH9Lxo7D4cQc0doj3eDLF67Qq1JxNOxM54tafeQPOlUJOpxzvXbIUrgOGvzKpzPBjvWlc8q\/zhrapHJHWIE8y1sDqcZvkKP86T+I10eIThOcYVOZkpw005S99eV1wwf+dqAh6PAvPIQ3yDT9ufvHuKZ40ufuBujMdIY461Um5c+QfOB8KnTgPr7vxgZ\/wjzZK7uHX6\/OII45YaiSIo69pf57eP\/3pTyc8aRPaBr9+QI7qOOKQrf6E5\/DzrNMxYuo03Xj1URhIsWGgjuWWWy61Wv+n0OrhvbwreJjlTU8NWMOsp9UUr6ly+eXpEi7wyxu\/9Ou0jsx0q1Dh6KQnfj0defFrovOFxYO4oEy5X152aaMJoM3N8nZ58Af04kaa+PIuf3migaATFrT8m\/IPP+VXPnQQ\/tIAeQF3Pb86bT0t6bVDPa70QV5NcfKyoQNp1Gn5CQuI14+sIz3tlgylUy93Ew06ecuPW1xpBG08IxxNXV7qgZ8waUUcz4gX6UqbGy1wawue3r\/73e9W20HEEz9Hng\/aJtTzF78pXp2uTiO8G6\/ku5wMZjssTHzkIx9JTFTm7zbbbFMtZvz1r3+d7UVbhn\/zDuYflDGGhgj4x+IDGWiQ\/NuiBHSVAJkWWXcV08QSzAnFZmHiPe95T3rZy15WLVo84xnPSPvuu2\/6zne+k+bSP18vQwFlMkwwnKHkYn6Fyc50F14wnASKrIeT30zHXm4YBiwGmOAE+8pYSL\/73e+qa4X4BdANk0+nuPI1\/rcSu\/\/++yeLFiZPn\/70p6cvfOEL6S9\/+Uun6LMuzFCgydTnZ9ihQ866Qk0ow5Moa9a4oZb6bjfMnlBxTitbQyk2K54mF2HPPfeszlDaFuE9B7pxlerGG2+sNgEbe7daD8zxPfShD01Pe9rT0lVXXVUp2XZ5L168OJnAHCUskY8K73vf+1K\/OPLII6ttOL0+26WflyHkQ15g2R++\/OUvp8997nPp1FNPTd4DV199dbr6QVxxxRUJWJjzG5dU55rnmwx+\/vOft+t+Y\/UfSLEZ8hyxZBXlxBNPTL0CvXijLo3V2b\/5m79Z5k4oeTzxiU9MrMc77rjD6xQQuC0r0QFH9fzZz36WfjYi2DDaL6YUtItHu\/TzMoRsWApw6aWXJmAhXn\/99enmJSve3oXZ1vPhD394qUJ2jvOd73xncn4XDjrooARvfOMbE9jIDZb\/wXYBO9UDu+++ezKfWPDSWSkHda2vdWmGIw8eSLGxiLbaaqtq75orTXoBevFGXYJf\/epX1TYTe3H6SXsmhG3jZODee+9N\/aKdEhqlf85TP\/JEu8IKK1SnQdxsYjMs+MAsXLgwbbnllhVsB2JN8xNmC4yLN0Muj3jEI5JNzAEbnB\/3uMcl4IYIM7lvj2TA\/OKb3\/zmBOZcreiNEvZnysOJm2HTHTStXuP1QteJpl1YP\/6mhnzwZqKvDaTYnAf99a9\/XQ3zYh6t2xO9eGmAf8z35z\/\/+dUVxOuuu27addddk13ckZS5PfNsVgRNplsVtaoV4e2eX\/ziF6vNw4cffngaBK997WtTwAS+RYuAjqijQyge5TB8A+5+EZZTL08KPxDlp3CAQoBQCJ46LNiPCBpllM3TewCd8jrGBJTUggUL0oIliLzaPcljlVVWSfhAT7mJv+WWW6Ytl8AUhndhgA699FjmsMIShhaU4AAAEABJREFUBQrk+qMf\/SgFFi9eXA2NDY\/tC2QhQj7cPv300xPLEiKedECaIF981EEu2gnlXA\/r933QtHqN1wtdJ5p2Yf346wvqbSbwoGLrL2vDu5133rna0NdrhaIXr7+cUrJ\/xgqnr\/pXv\/rVZJijEfKjLA1v77zzzupe+3xV1PEMCu\/\/\/b\/\/1zZLncR5OAoglEXME5k3csY1YNgEdqOzPoE7ICxoPc1RRVqRtnzaMtMmQMcGnR222267BJQLywRYJ2BjIji36xkQBmhhjz32SHssgXQC0s4hzxyUS2DBggUJ7ZZLFBFQcvgBHR8oQe\/CKD8QD9oUtfKmWFZZovjQQa74Fi1aVLU5eQsDtFBFbvOH3APqIupFHYH6AnUI6lT9AndAGKDVPgLSk25AXm1YKd7TJIGBFJvhghXHsDhUrNU4VoCdw7ZZXHjhhcmqEn8NUiMQr59yUUxM\/jXXXLM6SP+kJz0pbb311smcDfPWKQT3rTsJwV9nilVRR5lsGu40RDWk8WXXcDVY0GhBg1euwDCNNlcQOiVlwqp0NIUypmyA8gEKiXICbuAP6CBXTDr\/yiuvvHQYmHr8Z9jJ8vXsFgVNJ9p6OGVD8Siv+mfB+tpTdqH41BdrUbshC\/TQjhdKDyJttNIHClY+QB7CINxo1EO7tNv5U1KBaAPahPYR0F60nYD2RCkCdyDC0X\/0ox9N2p6n9CJtT\/m146cu52HoOqXVLqxf\/3b8jdt\/IMXGCnrUox5V\/fAHxXLKKackHcvcg9VJSsVciOMebiy9\/\/770yc\/+clE0fRTINtELA5olI5uRVxHUrxrAKyuVquVll9++QhOhrwBm3eXBgzh0CkC+KGcAjorZQOUD1BGkCsn78LQWT3ea6+9krOLkY50QT5DsDorolJOoLwUm+EdOQL5aDuGwGHxUXw+ROK0KyClB+RHqUG4xaP0fBAOOOCA6uYZ+ZgzijrxLn8YVZ1oowFKCygyUydWmz0pulB6nhQhpQjcAWGG1i6V0K8YCyA9kDbIz40d7eQ0H\/yXG7aQhoEOwj\/zmc+sbq2tp2eo6GvKukNbD+\/07kLGD3zgA+lNb3rTMmTOrEnLOVTK7yEPeUi13cDXz9yayrciSqHa57ZM5OzF+T0NHn+sh4DGDhS2W0qAOyAsaD3N+UkDWAggXfCFa0JKKfldBOw0hffjJ41B0uonXjfaTuH9hqE3l0p+ZEkhWXAgX6ukFBClxwJk8ZnLYalTfOK2gykMq706v\/k1E9tWcSkYbiu8rVYr+WhKU9rqV32re+0g4B2EATr8BAy9AV\/QaeTQjl\/+lFQA32CkQiHmFmOuHCnCl7\/85dVvKTzvec+rbjThRzECWjAXaa7RM1eM8qAYm9oUnnrxJ2dGCfqZwNCKzXDRJlhKol0BhFNAnu1omvxbrVa1jcPZsAg3BD366KOTFdZnPetZSaVrNBq6YfCOO+6Y7G179atfnVhzEa\/p6f44WxaEsRoCOggo2y9\/+ctUMPMyUM\/mVM2Jqg+XRrLS1RPFEYqI4mPtgbsC63WqrtshV3w6PIVHeVACYD8mC8k0jM7vzKg2ggf5UIQBVh9QwuDmChcZBLyDg+zAemedBliooGygjUM73jv5kx3gGSgxULaA8lF6AYqRUgx4B9Yu6INgLjtALkYoYMGGgdGJr3GGDa3YDEUNJ6w2XXnllVN4ve6669LnP\/\/55CfnDFenEPToYUjpPChB+1EUw4f4tadWq1Udp3KtCqV21llnVScQekn63\/7t35IG6Cs0nWCN\/OlPf6qsg2HzHTStfuJ1o+0U3m9YEz3lYfrD0LKbvFh7sPHGG6cYUu6xxx5pjyWgZCw8gXpnVcGCJQsi0KnN+HhTrNqYDyLLyS3RpmDANT8swLD+0KpjH+E6z7a5GM1suummyZDT4tree+9dDZE9Dz300ATm4cBUDsjDvWv6gnf9AJQLlAnM3VKM+maA7DqVr1MYCw5YtaDsQPkHDJEpa6BATUcN0+c78dMtbGjFxiqymdL82a677ppe+MIXpre97W0VuH2BDKtsvvSFbceQSx9t1QiYgwpak9aOTB133HHJ6qghqIYhXOPw9TYMrW\/3wJv5QHTtQPgatLmZ6YTGTh6jyHPQtPqJ1422U3i\/YU30TX6DyE4clg+o9y0fXNmlDCBf2NB2KT1Aq+21a0f8Q\/Hp+JQbJeeDbyWfReTpnT8jwHCYJaV\/4KsXhBzwAqG06grc3C0r0DCZ4gNzvKwp4DaUpoR83IVTjEAOoRilG3lQjOSmrL3CXHevtKOkG0ixYZYyCUaY4W6XcC24r5kxO\/zwhz+svpi+MmiCvulpWGkeLvD2t7+9IjNOtxvdVpF3vOMdyVcjV1ZW21ywx6RWGeZADEsMGwxhVUaV0IT90YjXWmut5Dksa9IYJK1+4nWj7RTeb1gn+mFl1Ut8FiPo0Lnioyg6KT7Kr1P6+VCXtcPqo+gM2xyB8wQWkD15YK5KvEi3V9l0o9Mv8MsYMH+prJQYsGz1ZUNNio7SA8rQ6ItiBO8B4YAeKEcLiZR28D6dz4EUGyWz\/fbbV9aZijH3sdJKK1U7vl2OR7n9+Mc\/Tr5IjlyxiroVivXigG9AeuYvmNzi+rqY9+DvPWCIwm2S+Q1veEM1BDVPwJJjsXkKLygSGJUEKD2gDHLFpzPnis87aw8oEejEAwUGhnHaOvhgU3adFB\/l1yndcYVRjgGygO0e3GdJOdrS1O+8+qh4HUix2Y9mwtAchr1qixYtqn7UxeS9yX3M5VaV90HAOrMfjuZnEd5yyy3VqQMWozCmv4Zg9ZMSNca3KsqyMyfHkqN0O+VtyKBh+LJMJ\/CNZ6u7w+Y7aFr9xOtG2ym837B29OarjBSGldeg8dvxVU+PtQQ6vb4BsVrKmrHQFUpvk002SUYYFgjiI92uvcof6oqP8ssVn3dTO3HCpV37lla7NtgurFd\/eRptVWWZgT8DKTZKy+Ss4aKtHr4sViNNoPqhBxOi5hRYbZTQoOWilAjS3IS5AD8aY87A4gGL0V4ewrMtxIoNxYoPFqM5PR2Bddkpf19DDcFK23TC3IrymWAeNt9B0+onXjfaTuH9hjXRN\/kNK7d+44+Kh1DO+pG5YkqNcrOialUX6qu666yzTnWGtlNb1leAUomhrvk87Vt\/ND0DVnatilJ+RliGxHVZtCtrr\/7ydAC+E7\/jDBtIseUMtVqtakJ\/n332qS52ZAG5reH8889PzFJW3SGHHFIdjepXyTlpYM7NIXu3O1BslJcNuvytHhG04aZ5uNm0KmpYYn7Ds75i1u+7NAZJq5943Wg7hfcb1kTPmul1VbRf+fVK38RXr3HrdN3SYjjoO\/oQWDU1vGPxQazqxgLHwoUL04IFC5Ihct4\/c7cRDviYUmZ+Mcs8OMOEUQLmwylDVqEtWhQlA8GihTLIo6mt1f1ZpPb25flPp3toxVZnVuMziW8ZnHCsZFJIVjm7WU\/1tLw7H8oCozjNmfEbJcz\/qRRDh4IVqsWMSZSDPYfmc3W0SeRvunmyOqndLly4MJnno+BCmcQ8n3f+5vhgwYIFHRWffhWKr5eVXcaFoSzjwnA8l4G8WKHSnAmMXLHlhWBJ2XfmbKehY6v1wEWQOU24mcWx1cOTIrTNg+nMAnRMK2jzpzyY9hp+7s9NETL3udvh0ksvnZELAJ2nNS+is2ogw2DQtPqJ1422U3i\/YU30tkcY3lgxHEZWw8Rt4mvQ9AZNq9d41157bSIvCioUTj7kXbRkXtz8nk3G5qvRUFDQrq\/w92Ex1GW0sPTUh3xM6WjPnt75M0gMtcWbboxNsZn7+uAHP5jMtzGlTWRa5WxXwKbtHgRDuTG7KTsw5HSkSsVQfrZ7MJUJPE\/bl8TwxQRu7h9ue4B8UVSEYe50wzyKBmB39rB5D5pWP\/G60XYK7zesiT7ma2aqvtRRE1\/8B8GgafUarxudOWi7CMzFuXDVPLYhKGiXrvQC4ZQYUHoQfajpqR+G4qM0rYzqa0204\/QbqWKzCulYhaMsFhGsmCqoOTJuq6ntCtO03aOu7HwdpcN6+9rXvpYsXvgKGfv7zcZQnMzpiy++OPndA+f+mvIkbBsT3R5S8JlUZFBkkLcBc9lgh4E5PTC0BYscO+20UwLvhrmwcOHCtGDBgmW6G1p9bRnPaXgZWrHZVmEY6Qvx1Kc+NTn3RlNTMm95y1uSSUp72Uw4dhsW1stbV3b2uNnHJh1zDNzm9AjYVg+nD2z3YAXhwYRrq9V++EvgLL+CB+45mz45lPxmu6yf\/OQnJ6DIzPGB+TyKLub4LB4Ir\/fr6XgfSLFZ3XQBpKGg+9YMD5msdr8fddRRyWZac1+GjpTTuAtitcimXF+Y2O7x\/ve\/vxL8uPMu6RcJFAlMlYDV2cDU0PH7DKTYrG464X\/mmWcml0BSZoaJVkKtiDoYPy7WbdY1acl6izycHXWbR77dwxcxwsuzSKBIYH5JYCDFFiIy3HR7rf0u4VeeRQJFAhMpgXnF1ECKjbVkJcVpAENNe9VYSBYNLB40bb2YV1IthS0SKBKYUQkMpNhw7OYMk\/NOGDjP6eYD5x4tHlhIsEJ60003JVsx0BcUCRQJFAlMlwQGVmw5g3bvu4PNPJsNp35YxXXdLDnXFVlkcPrA0DWPV9xFAkUCRQLjkMBIFFswZhuGbR22XVxxxRXJUNXigkUGk\/sOfQdt7VleiwSKBIoERiaBkSq2nKsYqloptXnWjZ5WL3OaSXc7xmMT8KTzWfhLyaZs+yadJnFUyEfVHssim8mUgBt63LitvvwQlHsXR1lfY1NsuTgNVVls7U4B5LST4HZ27phjjkmOcln1nQSeCg+dJWAxC4WzlIsXL07OSjp9wq9g8iRAkTmVYPrKuVLGj2vORsXptCi2UTHbbzqOX5nfq8fzZSBYw2abiJ1nc9VxzAEaUrsu2W821OPOpfdJLMugdeZ8ogsXjApsDnejsit5JrGMc4mnQevLfXNOKrRarWqBUd9Tb6OSzZxVbA7P+\/2DJkGdccYZyf3sNhOjYQ6zznzt0bMsbV0xpPFeMD0SGKbOnGV0XhGnFqp8qJxd9F4wHgkMU19OLFlgdIGAc+XuXmx3YcUg3M85xWahwp46JxQIvi6U3\/72t8mPzfrVK2fZHKI\/7LDDqgPz9uDN1B3tdT7n0\/uo6sz1VY7VOWLn5gofrPkkx+kq66jqyzy8g\/fmst3k60TRqMow5xSbg7kUlZs7mu6CchWSK5UovlbrgQPybgc19LzqqqvS7bffPirZlnR6lMAo6sxktK+\/39ww37bDDjukVuuB+u2RjekhmwO5DFtffrvE9rCYKjBCYoy4uGJU4plzis2Vyq5JMpR0A0hdUJSaCyjr51nNyfzud79LzsHW45T38UpgFHVmemHzzTdPLjvUUcbL8fxOfdj6cjKJde1qcnNrNva7782c96gkO+cUWzfBuCiPCdyk9LrFLeEzI4FudeZaaiYWzcIAABAASURBVNa2zeHm2czZgCmHmeF4fufarb4YFi4O9fsKm222WXK7thGUa89GJbl5p9j6ERxhW\/XpJ06hnX4JuNbaD5H4xTTbBgIWh6afm5JjLxJwWaz5NVtyXHlmn2urNbqpg2UVWy8czXIaS8rMYOZwvSi+JLZ61P3L+8xKoNTZzMq\/39wnob7mnWIb9DcS+q3cQj86CZQ6G50spyOlSaiveafYbO8Y5DcSpqNBlDyaJVDqrFkuk+o7CfU17xTbML+RMJ6GVFLtJoFSZ90kNFnhk1Bf806xaQI2cJbfSCCJ2YNSZ7OnrnA60\/U1ZxWbLR1WXZpWNZ0ndCi\/\/EaCJjg5KHU2OXXRCyeTXF9zVrH1UjGFZt5KoBR8jkugKLY5XsGleEUC81ECRbHNx1ovZS4SmOMSKIptjldwKV6RwKRIYDr5KIptOqVd8ioSKBKYFgkUxTYtYi6ZFAkUCUynBIpim05pl7yKBIoEpkUCRbE9KObyKBIoEpg7EiiKbe7UZSlJkUCRwIMS6Emx\/elPf0p+6OS4445LfrvRFcye3v1oxt133\/1gcuVRJFAkUCQw8xLoqNhuuummtM8++yR3nO++++7JD2W4gtlvAXp632uvvZKbL\/fcc8\/k14FmvkiFg1kjgcJokcCYJNCo2Nz9f\/jhhye\/0+g3Ao444oh0\/vnnp+uvvz5RdgHvF154YTr66KOTX3\/yAxrijYnXkmyRQJFAkUBPEpii2H7961+nXXbZJf3hD39I5513XvrKV76SXLG81lprJYfH81S9r7766hU9OvTi5TTFXSRQJFAkMN0SmKLYVllllep3N\/183WMf+9i++EEvXl+RCnGRQJHANEpgfmQ1RbGxwuo\/TTc\/RFFKWSRQJDBXJDBFseUFu\/3225OfM1t77bVTN2y88cbJT2qZf8vTKO4igSKBIoHplkBHxebXZg455JDkh0xdKueHiI9YspBgq4dfbvaLTn5G68UvfnHyY7Vnn312Nd823YUo+RUJFAkUCeQS6KjYHvrQh6abb745+am6b3zjG+nEE09Mtn285jWvSW6n\/frXv14tKKyzzjpV2Le+9a20xhpr5On36S7kRQJFAkUCw0ugo2Kj0M4666y0\/fbbJ6uf9ezWW2+9tNNOOyU0aB\/5yEembbbZpk5W3osEigSKBKZVAh0VmxMH9qf5IeF2XAlDgxaNd8+CIoEigSKBmZJAR8W28sorp3XXXTctXrw43XnnnVN4dJTq4osvrqy5hz3sYenPf\/5zuvHGG6fQFY+JlkBhrkhgzkmgo2Jbfvnl0+tf\/\/pKWTmF8K53vSuZV4PTTjutGqJedtll6eUvf3m6\/\/7706te9apqQ++ck1IpUJFAkcCsksBy3bhduHBhOuWUU9Jqq61WnRXdb7\/9Ehx55JGVFXfooYem7bbbLt17773JUSxbPrqlWcKLBIoEigTGKYGuik3mDsGfeeaZ6Xvf+15yAB4uvfTS9N3vfjfttttuqdVqpUc\/+tEJjd\/rFKegSKBIoL0ESsh4JdCTYrvvvvvSNddckz7ykY+k9773vcnG3b\/85S\/plltuSX\/961\/Hy2FJvUigSKBIoE8JdFVsFgj233\/\/9IIXvKDaq\/a5z30u\/epXv0o\/+MEP0rOf\/ex02GGHVYsGqfwrEigSKBKYEAl0VWyf\/vSnq1s+jjrqqHT66acnpxHwvmjRonTggQemL33pS+nUU0\/lVVAkUCRQJDAREliuiYvws+nWMaldd901uTV3hRVWiKDkiNXee++dHKdyi+7vf\/\/7pWHFUSRQJFAkMJMS6KjYbLq1+XbVVVetFgjqjLZarSTMZZSTdA8bS7LToX1KetSK2BzkRhttVF0W4F66uqz6eTeH2enygTr\/Ud66fz95zhTtySefXMksr6\/pKkfkTdZk3qsMQt54boorLf7C5dFrukE3yrYUacZTuydfvH384x9P0Wa9N2HYthz5Tvezo2Kzj80VRoTRjjFhK664YppNJw6s6jrQrwG2K9ck+892\/slWu9HBjj\/+eK\/LQPme8pSnVFMgywRMwAulc+655y7l5NZbb03f\/\/73l75PuuOnP\/1pNT\/u7kRnvLvx61z4IMo5lL86Vtfd8hl1eEfF5uSBL4+5tSuvvHKZvK2GXnDBBdW8m\/m2hz\/84cuET8ILvjQ6VykF\/E4D3jRI84PcE4E2TLzoRS9a5jr2gw8+uKKcLfxXzDb8obxAkDqJ+nETc8zjmtcd58fHVIp8tWPblfDSDRdddFGyoJbTffOb38xfJ9ptYz3+X\/KSlyRGSzCb1wGZ6Df6j\/DPfvaz1U4I7tmCjopNIfxYi31s5tL22GOPRPu+7W1vW\/oDLo961KMSmlarhXzi8ZjHPGbpAkidWV+mujneZIqTgS9R0FL+t912Wz256t0XPjf3uflVgQP82WyzzZby\/5Of\/GRKCtdee+0ywwt84jcnjK9p8O\/ZVM4mefDL0+IWVxoB8uimkEIZ6DwgHdhwww3T6173Os501113pVyu0pR25OPZVD6R8Sk84D3KHXH4CZemtMXrBHL89re\/XZFssMEGCbxcfvnlXTu+9OUjP\/kGL96BDKXVhG602pN2JZ1AlDFPDw949eHQjvKwutscuivJ6v7e8Rr5eCqXtIV5eg+jwccrt7575VVaw6CrYiOED37wg8mV3+bT3LDr3Cht71422z+abv4Yhqlxxs2\/uI9\/\/OOXZqXxNA2LmOLCglDFGcaqsPBjPaHzJQw\/Tw3YNpncn5ufxoFmlMCTBi2PSJcf3nRKEB6NLmg80eCXG5S5SR78hKEBbnG5A+Sx7bbbVnsfw6\/d07Yhw6M8PCwpG8IpOmE6hDSl7T2gfDqp8PBTDnzGu6f3pnIL6xX4xC96bQC48cTC4e4FTbyQIb7r8fnX+eYX7cdTe8rrXBrkgj\/t1TvgEa8U8lprrcWrLcSjBBHo386Ccw9T3\/3wKq9h0FWxSdy9bIZEzoj+6Ec\/Sj\/+8Y+rX60i4JVWWgnJRELl+lr4qgQ0Kswa0tmHx03g0Xj4M8VBmYULQ8MtvsbBHea7fMxZ8AvoaB\/60IeqV+lID7h5DjrMaqeYpQlN\/OMP8o7ZRMcaofzwHvNIUcacdw1ewycTssnz1XlYYDrascceW1n4wutwDI8fOh0z6qepc+NJWmjJWVnwE8NW\/sLR4T3kjg\/8oFVe+S2L\/t5C9j72lClwSyUsUO5uEAfv+FIWZRKnacgnDA1accRFa+JfeSPfJjrtVPnRoxWH+5WvfGW1q4E7oC9HHXiSnfjStSmfBddLfVOAhvbRzqWDB32tV16Dp2GeUxSb854Ouxtu9gr04g3DyHTHjUaUV7hKcEQseNEZVKx3laIz69TeVZzK4jY\/Y6Myd8AQSofTEF\/xileEd+Lmp9Go8KUBbRwUh4YWoFiR4gsP3IG6XyiPCGf9sIJ0ElZR+MezaXVbg9eg0ZxwwgnVfJ+Gq8wxFM7z1QF0HPSsG8qUuw6yI9+6v\/JFWSNfHRvQkrO8uZUnhq2RV6583vrWty7twOpV\/Yo3CLQTil\/csHhYPdz8tAvtg7sbWJ54R6cs5ru4m9qEMDTC8\/y8Q71O+OWIOlIPZKSufOxzmk5uPGmDaCItaUTb67W+xe+VV7TDYopis8XDGdDvfOc7KfDVr341GXIqoN8RVcGf\/\/znKz8366IXb1hmRh1fQ6Y8dOQAC0Q+Kkwnsk1Fh+ZnTkFFcYOvDzOc+xe\/+EX1k4TckA9jvdfn7qIRUG65RcLNT5yg4e4HysV6jgYfcfGK53iv88RfBzUcDeXhqV6FBcRbZZVV4jVRbujAUCQCgn+yxJNwQI9GOSl47iZQrupFHYlfp5EO5Rb5+CDgLaeLeqjnhf+cVr2q3zxuP+5QDOJsscUWlcLM0yQD5RDeDfUPTpShKV4elueX05IRuQfyNhZ0ofBd819vN2j0C3URUJaoExYwSzjqQVmFRX7qSRr1OuBXRy+81uMM8j5FsVkMoMQoLzBkMhRl6tP4FBuFd91116Vjjjkm6Ujm2sQbhIHpjqNCQL6+sn5HlbsX+CKb0O6FdlQ0vozR2OLpWnaNvN888G\/ehfVDSRjaSLNuOWn4FGfIKc+HEjQ5LK3cfxi3sigTXkAni\/RYyuHu5RmdrxfafmhCMYjjgxidmjz4Qb+8ijMsDN1DsURbUb+sqkjbx0xfVudGDOHf6alOwvKmsJS\/E30vYQ\/yWpG247UKHMGfKYotT5NAPvzhD1fXf1sV9eMtEW4RYZdddknM6g984ANpEi224LXT0x48lg4aVpkyc0PdmluwYEFiCQirdyCWiQYgLIcGpqHpsHWwWHLacbt1Ql9bPJlDi+FQvSz40LBzZaMh8gdp+KJzAwXovV4+74acaHKQcViNnt7zcEMlPIZfWC3kS87h7xm867QstKD1Acpp5aF+xekXlLipi27xfCjRdqMLnoMu3qMM4d\/t+Zvf\/CYFXz5OhnriKLfyc4P2B4bNhrP8hkG\/9S0vcumFV7SjQEfFFh3bfrZWa+p2jlbrgZMHFhMcvxoFQ+NOQwWDfJjllFV8mfh\/6lOfElTBl1kn9mL4wJIRxzslwazmVmksW+5ATCyLjzb8fbV87S3PM+\/DfzqfGr3GL088UHLcAeXCI3Dz12lYeDqfdyATT3IDbjBcFbedZUdpxrBQPBAvQF7k5p2i0pHAOzmTNzfeDZO4o9OG3ClBowwKTbh6refDvxdQ2sEPBUJh5wiljwZttzTxjHd0nt65fTQpZ+5+EcpRvNirxg1hScYQml83kFssNqhzch20vut5deK1Tjvoe0fF5jQBi8YQ1LXf9UxcXeRXrNCgrYfP9LuG7OuvkwXCbFdZYZazKqJxUmZBq4MpgzA03Bp2WBPSQqvTadTCA6yhmNjO0+RGw9JFwz1d0DiVW6c3D4N3T+85D2RWL2NOq7xAJmQjbsgCXcgtn\/hGk6NJjuJCyEgeJv0pQgsBeCdn\/uiCd\/7C0ZEp2corr\/9Ik3+\/CMUgHzKsx48Oz58yYBBwtwN54z0vA9p8YcR7N\/jxpPxDKz0I+YtPQbEk2\/GOBvL6k4Y2QH7CyJNcB6lvaUiLwu\/Gq7xGhY6KzV615zznOdWvUPlS5vNR3Py+8IUvVL9MNYknD9oJSccwZ6CygoZFcvDBB8fr0qcOKCw8WG0WTqLj85eeYZvG4z1gqFm3coSZQ8rT5DcdUN53v\/vdy2SFF+BJaegUythujo0iU1ZKRBzliPjegRyUW\/m9N0EeVlfJtylcmnk+eGdZ5nIXzzt\/4d4BT\/V0veNdeD9gHVIM4oRVyJ1Dx8UHP0aAjz13O9R5CXlRHO3itPM\/4ogjkvYX4dyUiSc\/t\/Oo11BO\/PqBeiDPiMPNL949g\/+8vhkN\/IUHuvFqqEreQT\/Ms6Nik7DfMdh9992rsfwznvGMpQeWuZn3O+64Y3rta1\/beEhe\/JkA4edDhbo77zA5fyqmTssvp+GOThm00tOQYitwTyUmAAAQAElEQVRF3kB1uPAP+jxcenXk6StLPbz+jkba+AiFgybPO\/L0RBvwDvEe5ZWO9MI\/nvKSdo48PjrllXdO084tP3HqkGY9Ti6XoKcc+ddp6+my\/OpzbEHTLg1pSlu4\/MiDXPjnyGmU3RA84sgjpw03OUoTxMnlxc1PWF0OES94wQ83WuDGj6d3OxY8xYu845nng6YJ9fzF5ZfT4lVawgLe+QedON14JTO8RxrDPLsqNswcfvjhafHixemNb3xjdU2RLx\/T9ctf\/nJi4qMZhokSt0hgVBIwJ2goBeb6Il1WDHjvZ64JfcHsk8AUxeZwu6vA60WxcuhHXI477rhE+9viQSu3WlMXFepxy3uRQCcJjDKM5QzSNKym4MCHmJ8how8zd8HclcAUxWbuzM\/p2ZTbtGDQSRToxetEU8KKBMYpAaOHGIbFMCh\/jnK40085DLHkjZd2w9N+0iu0nSUwRbHZaOu3RJ0Pe9aznpUcgP\/5z3+eWHJNSbHufvazn1V06D\/60Y82kU2k39VXX52uzuC3HJowkcwXpooEigTaSmCKYmu1WumZz3xm+trXvlbNqVm+Niex\/vrrJ4qLO7D55punJz7xiWnrrbdOn\/jEJyp6pxba5jZhAc64mjcM2J7QhK222io1oYk20oqnPAKnnnpqOjWDbQSQK9crrrgigfmggkuW\/txjkcWIZXHJ9KTHKJqJbj9FsQUTjlE5WfDf\/\/3fyYa\/7bffPj3kIQ9JrizCrKfjVIatX\/ziF5PtE+jFizSm62kI7KcB7ZMxn7LNNtskv8PQzsrElzIA96Bosu5yJcVNcQXIMUcovFCCngcddFAC+7IC3kE4mOsE++TA\/JFNxmAF227+gpdWv9NR5DCzctBuh+1ng\/TPtootEnN0iqVmweD888+v7tgyT2DH9DnnnJPe\/va3p4033jjlx60i7nQ9zzjjjPSe97wnvexlL0sO5duKsu+++1aH+FObf4TttIRtAPbctIPNl3UsXLgwLaxh1VVXrX7\/IZ5tsh3I2+ZnWGGFFRKYR4JVVlmlOuL1iEc8ItmxDhZ5Hve4xyVP4OeUw3Of+9wEb37zmxOYh7IzX7lZ2947oRttp\/B+w5ro\/QQk4bkXsBOf4wxr4mvQ\/AZNq9d4vdB1omkX1o+\/Orv00kuTvqbuphNdFdt0MjNIXn5sxiZEZ1kJ8glPeEL1W6dPf\/rTk83DTke0S9eubJsu99hjj7RHG1ACdVCiddhcmIPFmCMP4474edqUTCCUaSjQfhUmRQgUoSNU7tEDUwXAGrRdhx+L2xPQW1WsgzVu24\/ph3qY907h\/YY10dupTzabbLJJtSFVntONJr4G5WHQtHqN1wtdJ5p2Yf34MzDa9b1x+0+mYuuj1DfeeGNy7ZANgK3WA1tPDIef9rSnpauuuqrrlc19ZDUUaSimeIbCCgXmmSvXUHihAClDCGXJDRGOXscHaUlfXp2YjqG0IXMMkQ11Yz7RHKLhsnlBw2l0ndIbZ5iyGGZ7jjOfkvbckMCsV2yUGsvE8a+8SixquPzyjjvuyL2XcTtgTTEuXrw4LW6AjlyHFeA63LKQ4957703dEFaUucputE3hqzw4DFXOBQsWpAVLwKrxG7AsV0OGU045pRqaG55z8xPmqwvKL51lhJK9UHwUGqVHwYXSo\/C4KTwgIzJh8TWVp1NZm8Ka\/MjADTJ\/\/OMfu8oW7TjQjq9B8ho0rV7j9ULXiaZdWK\/+2oO+mTWnaXXOesWm85lvciC4X8mZe3LTgP1FTXDesQ6dvI6TTjop5aAEcuRh3JSB4aAfo\/7P\/\/zPashMgeRw5x04EgNuUIFf\/vKXqQ4ysP\/QjR31sHi3kFKfb3vTm96U\/vVf\/zW5XcIJEmcYKUcwpG8nT\/lRZiEHSm7PPfesfoTFQsYBBxyQ3GICpgOUox1\/0qqH9eoXZZuuZxNfg+Y9aFq9xuuFrhNNu7Be\/bUNCwft2tC4\/We9Yhu3gEaRvrm8HBQQ+Pr5srEanavLYZUZLNCAXfRgoh+4gXKkZHwdpXX99ddXv+zEujEkX2211VI7sPKsJHtuuOGG1ZYWm0fBPkaWHnzyk5+sFhzyYW47ubBc8RAfhI997GPJwXuLTFZwKU\/Q8PHOUmZVkwWZtFqtyvoMvoJ3StkeS0PR8JvuJznV+RqUh0HT6jVeL3SdaNqF9eqvrRgdtGsn4\/bvWbHpmPYSufXBV0rHgXEz2C19NwgYnljhrNMaonZardWxTP6qgIAKaQIrpg6Hneswt5VDQ8hh6Ad1Xvt9Vx9AGVAkYAUKlOv0009PLgllOXoCP2Hq0bDR19cwyoJBJ6yxxhrJvJ05QHN55vXM9cUcX8gryt2pLBQfsJDxA+973\/uSxQxg+T3vec9LwA383\/GOdySXLrAAWbtAKYKyBChI6FSeEvbA6vo45aDNT\/TigT1ibsi1pcOeIPunXM3iinAT9KwHw5xOjXmcYbY2ULAac56Phu4r7wuf++duisElmhRNQIU0ITpt\/txyyy3TljXUlV90+niGAqUgTIZ7Wm0Mf8+gjbRy5Sn\/4A\/PeXnauckGWHSsO0qFRcVqolSOPPLI5OmdogC0eXoUoHv\/PcOfbPFD4Znbe8Mb3pDM44XSo\/iUzzYcZUELEb\/bk+IFPAML1go4PoHSBkPhgPk\/2CrbVO09R9B6ih8wRVBHPj1g0zoL1ocBPwE85uhWLuHkWJcn\/27oNV4vdJ1o2oX169+tPOMK72qxOfv5\/ve\/P73whS9M\/\/7v\/179iAVmnETYdNNNk0PxKp\/fTMBckOGAO91DwVJYF198cbLlY7bcE0dJBUJxUQKQK08KIhQfJUgpUh4Odrs+ShiaUIbiS0\/anernrrvuSpQZpQcUB4UHlB5Lz1BZZ0bXKS1hlB6w9OwVxBOlF9ZeKD+LGoam5vvwDjZYq1e8SwOkOShypcOtDAFtN6DMdYTS8zSkNhfJgqQUA7nS5M6Varj559hzzz3TwQcfXG3GjnTyp\/zqOPXUUyur1UeJ9eodgv\/8ee211yYf96uvvrrtsUGWMwwq10mO11GxURBMfxs73\/nOd6Z11lln6b1rNn+aCNcI7YNiNc1EQc277LTTTslcjnOqNg5reBYFdt5556X8tuPN0E1H9SWaTuCHQvYcNl9prLvuutVmXUrASumiRYuq\/V7qzgoopUcBegI\/QwVg2YJ0mkDpXb2kg1BsFJzOHwqPe\/GSFWVWTKvVShRHvTzSbCorZYvfrZZYV7br7LrrrglcN2SOjyKk+ODMM8+sFjksdHgXBpR7QJkCodgpyBwWmQBP0wlyqYNSoYDIto5cSYWbrFmsFJund6grQO8m7rspYaeGfFQM+9VBDorXz296hkKmePUtho6nfAI+fj6IpggoWzCyMx00nXKOvDoqNlccm5S2idUJhIgUT37CrNY1zXEF3bifhkKE7mZPF1+ajCf8Jz\/5yV2zViEah3nD+QDzkbD88sun6PCUHxgas\/zcmmzFGLopPB8FDRrIETR2Hz3wYYyOaXUXBpEzpWD1VIWyzPEFeAwoQ4DlB5R5Dp0dnIHOwS+HDp9jr732SjlYoE1wdXgdIef8SbnWoWyTAvKuo5vyVfc5WJV+NmAmytRRsdG2GOu0e18Yulbrgc2xM1EICvbVr351coUzpXbWWWdV1krq4Z8jOjoAi2I6wVph5RoqD5vvoGk1xWP5mU\/dbrvtEvhoUAzmz3RkP9\/HGjJMNMRtJ2LWPqgPlh5Y3QXzsmBVF2x5MXVgAQR8TG+++ebkYxkru5QY65yFN6y8muK7WzBHbr1ws\/6V3byyc9NWjptw6KGHpjpYn3X4CLta3xlnq85g\/q4JwgLoWaqehvVNUF+sWIrYEN+7Nl6HuU9KWD0H1CswCihibjIPUMbt6rzJ3+0\/Tf7j9uuk2JJOZ56K6aux1Zkx+SmMENDWw3t5N4R5\/vOfXw1zdSpDEenmcXMa98trJKyOoLHAoaItxfd6CD7iGlLroONcIWpKW4dlNTWF9es3aFr9xENroUVdm\/Oj5HQUc3w6kSGgTgI6A5mGjNs9KT4w1DV1YFoAnElmAWpbVnNZgCxwK6gsAn7cYGgM2kjA3FIgtzryIXK\/MkZPBjNRZ1alc1Dy3sm5CaGk1IUhvncfqDrMfVJ+6o\/yg1CUPvjeKVGLQAEKlqVLAZsnBWGmB1i8nt6loy+Tebv6H6d\/R8XWarUSSwgDxuEaGEYVzvicn0aJhtWGrh8YljisrpIsUmiw0uMXw44bbrghEbyfa3O20bif8AjekER+gxyCF28moaOstdZa1aH2YfkYNK1+4nWiZfmZ19OJIFd6ofgoQf6sPaD4QNx+yq99gCEwxPCIIgxQiAGKMGDaIWCOMBB++TPixFN6FKhfW\/I0r1hH8JI\/8VgH\/vUjxoCn915l0Kke8jR6oetE0y6syZ81py4ZFp7eKVxTBEZ0OV\/T5e6o2DDBonGDAavKPWGsI1s9fB2tilIqvuJo+wGlJN0111wzUZhPetKTqnvdLFK4DcDSvvRofw3Al8NQwVeAee2XidANcwhe+gXjlwDlpcFr7Kw9oOjARyuUHzc\/yJUgRWgIxPqSjvRgVJxTLHXUlVGurLhDieZPyq+OUIz5M1eg4Q4lW39GeP2Zp5e76\/l7ryvgeFeOHPUyx3sum1HJfNzpdFVsGDC3YVvHD3\/4w+oSRMMFpj7LidWBpl\/cc8891eF1jZ3iivjupPeuEZtjIXgKFA9B40sgzIqSORzDZJZCq\/XAPJ8hg7mQXg7BKwslPd1wzEgjNpzK8x7EPWha\/cTrRtspvJcwVpA2pd59sHRIm49ZCMAKQKOTWcmFWChY9OAKsCfLMSAOiJ+DlRSQXw5D4xzR5mbqqbxNCKVTf+ovdeTKN3eTcY5cQebuXKlqs3VEuO0w4RbffKnzyDMhu54Um4o2Ma8xOGxOmMbmxuyXXXZZ22vDOxXIooSNv6yvnM5vIDpIbUGA8vI7g4aqOY3JX8rPBDOlZhiMr5xG4+50CF6aOoeKsPF4umEFUuVbNh8270HT6ideN9pO4f2GNdHbvqB+m+rLUa0AeQaMBMAURw4fkwBlmcPWpRzqKEceFu48friNOOqoK5x4N7\/YhFzhhltfrINcZhKhePEVbjoCbAfT16abv66KzWoW5uwtuvvuu5M5L7+JcOWVVyZhGqHJ3n4Zb7VaiTJygD3i+lIfffTRicXlckuKjbCcLgia+lOFS8NQpR7W6Z2wzdMZDhd8JhUZ9CYDq5l12LhehyNgdZiMb4I55SYYktdhz2YdYbnWn3W6eK+n6d3QP4fhfxOMsHKYGshhiiCg\/+FJX+OeTnRUbCb+rICstNJKybDTL+1861vfSqwpS\/R2+yukTXm2LgzKuHk7q5oWI\/yorRWV9dZbb9Dkeo5H4IYvBQ9s5p23csiGsnNJBua+m5AronDrxznMgzYhV37clGIO86QBc6fS7LlDjpCwo2IzJDSPpQC266KP+wAACs9JREFUUVB03jFrLswBdIXvtkH3vPPOS+IHWH9RBls73H5rDo+VZl+TH5MRbl7E8rq5Fu91ULB4sPWDdVcPN0TtdAi+Tl\/eiwSKBOaGBDoqNiuXlBkFobi2YDgmYQNn+PHvBgornxR3hY045sfc3+XaGltI7FS3CisM3OVviGmnuveAd6cibCA0TGUtGttHuKeJaPt9TB57LygSKBKYPxLoqNj8ChVFY+XRcNF2D9srbJIlIn4Uncl8tPyawOoyjA0Y2lKahp\/obelgpbVaD6xq8gMLBDYIm4RlPfIDv5xF6VFsIH\/DYmkKNy9nJ7u40uBXUCRQJDBJEhgvLx0Vm0l5R0kMD809uG3V2VA3exiS2qRrFcjiAtp+WHUA2H442zQMKa1+5qCcWq1Wkr9FhcMOO6z6hSzK0EFoE6G2gAD3oIfg++G50BYJFAnMDgl0VGyK4LoiisQkIwXm2AQlZjWSJWcZ3rYPtP3AsNbw0TI8pVmHA7TSM\/l54oknJj\/554C7FSmrsnme3IMegpdHQZFAkcDckkBXxdZqtZLbHuzZcYe9lUQiYGnZ2+Y4FYuLXz9w0sCWEQqrCQ4YR3qUnoPtFGlTnvLHhzA0aMWJ+OVZJFAkML8k0FWxdRMHZceSY4F1o5094YXTIoEigdksga6KjcJyysDNG7FdI38amlocmIvbKliUsYI7myt5PvBuTvYtb3lLcppkk002Sdqlxa35UPbZWEb7VZ37Vl8WDs2dj7K+uio2GVqVtIPYTmT7xsx1uZvKXJutHObJnCKYjQJu4tnCxjHHHJPsBretpImm+E2WBPzIEI6cS3XI2+KWlXF+BZMnAXqFTrENzHlVuxrshx0Vpx0Vm02v5q1oVhP4zt1ZhXTLqndzWSw6Z8JGxdAo0znhhBNSvhk40vZlIFjXrLA+DaXdKxXbRVifDtFbOIk45Tl2CVQZDFpn2qCTK+ZbfXydFXZpQ5Vo+TM2CQxaX3SIjf+tVivZh6rvqbdRMdpRscnQvjWbXFutVrJXzV40h88xYCFh6623Tg4Jo+U3KXCiwWV4Tfy4asmxLTeIomEOs8587dHb+2bBxJDGe8H0SGCYOnMG04gCp84z+1A5IeO9YDwSGKa+XEH2kIc8JLkAYosttkgWE+mZUXHaUbE5XeAWDqcPZMgtczv\/vYPhKBOSded9puFHP6yIWrUl+Do\/FLUfwnCMy5k2G3ztkbOZ1\/38UdZ6vPI+PgmMqs4crbMdyPYfV5n7YI2P6\/mb8qjqi+5w+YK5bBd4ulJpVFLtqNicELBooCBu8rj\/\/vuTm2zNuTkJYEhnewUF2Gote2pgVAz2m459bxSVmzua7oLCr6NcFF+r9QDPbhMx9Ozl\/rZ++Sn03SUwijozGe3rbzO3+bYddtih6y+Udeds7lMMUsJh6+uWW25JbsKOqQIjJMaIq5sG4acpTkfFRmH5qTYrTq4nuu6665IJP\/vODEHdqW5\/myGAM51NGUy3H0VsYcNQsoknSk256osd5mQ63d823eWYT\/mNos5ML2y++ebJJYg6ynyS33SXddj6MrpjXTuOaW6NkeRuOnPeoypLR8UmE2NfKxZ+s9OcGm3tx1S43b7hdwcdtWq1HrB+xJlkODHBBG5SepPM93zmrVuduQCVte3ySB9ZczZgymE+y22myt6tvhgWTiw5qsk48qtfRlBPfepTR8ZyV8UmJ\/vUaFNfwlarlfwcmR9fofAM+ygKdHMNhG3VZ66Va66VxwfWz\/m5zVmbDFgcmmtlnSvlMaVlfs2WHFNbfjGr1RqdcdRVsTEVTcJbRTQuboJtIIZxoxb6ONKzpMwMZg7X0\/clsdWj7l\/eZ1YCpc5mVv795j4J9dVVsTHvzVfZt+am3CbYz2Y40K8AZoK+3N82E1IfLs9SZ8PJb7pjT0J9dVRsrBqKzf1rlmItGjRBmP1t0y3AQfKzvaPc3zaI5GYuTqmzmZP9IDlPQn11VGw23dr3te222yZXgQ9SyEmL4+REub+tQ61MYFCpswmslA4sTUJ9dVRsFg1si5hrm1Zt4Cz3t3VomRMYVOpsAiulA0szXV8dFdvKK6+cdtlll2TDo\/1fHcoxcUFWaq26NK1qOk9Y7m+buCpLpc4mr046cTTJ9TVFsVndtMoZq582zt1zzz1pu+22Szbrhn\/+RC9eJyGUsCKBIoF2Eij+o5bAFMVmddMqZ6x+2tHtmArldu6556bwz5\/oxRs1cyW9IoEigSKBQSQwRbFZ3bTK2bT62c4PvXiDMFDiFAkUCRQJjFoCUxRbnoFVUb8c5axo7l\/cRQJFAkUCkyyBRsXm1g7zZs6FLlq0KG200UZpn332SS6VnOTCFN6KBIoEigRIoFGxnXrqqcm9VmuuuWZyb5l7rcyvHXLIIclxpFT+FQkUCRQJTLAEpig2pw0sBrgCxl3kxx13XHJLghs8Lr300jTKO5MmWC6FtSKBIoFZLIEpis28mtMGm266abWvSNlarVbySzIOxN922228CsYrgZJ6kUCRwBASmKLY2qW13HLLlRtJ2wmn+BcJFAlMlASWmyhuCjNFAkUCRQIjkEBRbCMQYkmiPwlYdXfUzS8VufHWlqJIwbnk17\/+9emII45I11xzTbUif\/LJJ0fwMk\/bkPzOQT2NZYj6eCmkc0cCbRWbkwYaXMC8mzk2R6fCL562gdx3331zRyqlJGOVgF858wNBrp13A7OLCSNDP8pCobmlOfzKs0igXwm0VWwnnXRSWrRo0VL4bQNfyAMPPHCpX4TvvPPO6Y477ug370I\/TyVg5V1borxcv+5XwkIUfg3NTcbrrbdeeJVnkUDfEpii2Hw9DQPcmNsr0IvXd+4lwryTgGGlXwC\/++670\/HHH5\/yYaQRgd8r8KMefr+2X+Gcd955ae21125Enk+\/6Rb6MUpgTElPUWy+nr6kfsKuV6AXb0w8lmTnkAS22WabdPDBByc\/wLLjjjtWc2nxUfTrRn5tyi9MtVr\/98Me9WmRfArEfF2IZ4MNNkj1j7ETMyzABQsWpIc97GFBWp5zXAJTFNscL28p3gxLwG9S+sk1d+IZbuYfxRtuuKHibuHChdUz\/tSnRWIKhJK88sorgyy58j3\/GEvn7LPPTn5dzakZ94ctJS6OOS2BotjmdPXOrsItXrw4rb\/++pWCyjnfd9990yWXXDIF559\/ftpkk01y0qVulhyFeOutt1ZWISW6NLA45rwEimLrVMUlbNokYNXdkT2LCYaOecYrrrhici1WHe7WZ\/ml2j9zdaecckryQ0T7779\/cklqjaS8znEJFMU2xyt4thTvxhtvTObSzJMNy7Nbn1lrhry77bZbOTEzrEBnYfyi2GZhpc1Fll1Wav7NjTLDlM9vc1icYN0de+yxKRYmhkmzxJ19EiiKbfbV2Zzj2L62iy66KFkNXX755Qcun3k1t9GYV3Mbzeqrrz5wWiXi7JZAUWyzu\/7mBPeuwnJ6ZeONNx6qPO4RdOWW+wPNs\/l1tcA555yT7rzzzqHSL5FnjwSKYps9dTVnOb388suTIehaa601VBmvv\/76RKFdeOGFyXnT\/fbbLwUOOuig5EeJhsqgRJ41EiiKbdZU1dxh1OF3R6f23nvvqlCefgO2vs+sTlcRZ3\/Qi2d+zpyag\/XtfnBIftLLohfn\/0lgzrmKYptzVVoKVCRQJFAUW2kDRQJFAnNOAv8fAAD\/\/6lkQRgAAAAGSURBVAMAjIPYfOLMtTQAAAAASUVORK5CYII=","height":187,"width":310}}
%---
%[output:0c53d3ae]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATYAAAC7CAYAAADv5bIbAAAQAElEQVR4AeydC7xOxfrHn9eJHLrYpVxCW0hKIuoQyq2UO5UouugQusmpdhcVRUU3OiKXropSuR5SlJQiuYUOoiIO6rgfOkp1\/r6j2f+1t\/W++333e1lrvfvx8exZM2tm1sxv1nreZ2ae55lC\/9N\/ioAioAikGQKFRP8pAoqAIpBmCChjS7MB1e4oAoqAiDI2D94CfaQioAgkFwFlbMnFV2tXBBQBDxBQxuYB6PpIRUARSC4CytiSi6\/W7hcEtB0FCgFlbAVquLWzikDBQEAZW8EYZ+2lIlCgEFDGVqCGWzurCKQSAe+epYzNO+z1yYqAIpAkBJSxJQlYrVYRUAS8Q0AZm3fY65MVAUUgSQgUYMaWJETTuNqVK1fKX\/7yF2nYsKFceOGFUrt2benUqZNs2bIl6l4\/\/\/zzMmfOnKjyk++MM87I8byHH35Y9u3bJ\/v375f77rtPVq9eHVVdmqlgIaCMrWCNd9y9rVmzprz77rvy8ccfy6JFi6RSpUomHnfFYSo499xzTf08b\/78+SbX448\/LkcffbQ8+uijUq1aNZOWnz\/\/+9\/\/5LfffstPUS3jcwSUsfl8gPzcvF9++UX27t0rJ598shw8eFCGDBliJDqkOq5JgwYPHiznnXeeXHzxxbJ06VLTJZjKa6+9ZiS\/Bg0aiJXEzM0wf\/785z\/L9ddfL1999ZWsW7dObrrpJkGK\/M9\/\/iN9+\/aVunXryvnnny9PPvmkaQ\/pt912m0nr0qWLdO\/e3eRHauzdu7e0bt1aFixYIHPnzjXtuOCCC6RVq1ayatUq+fHHH+Xyyy+Xhx56SGgfkumrr74qTZo0MfV9+OGHYVqpyX5AQBmbH0YhQG1YuHCh1KhRQ0477TQ566yz5N\/\/\/rdhCrNmzZJNmzYJUhW0c+dOmTRpkpC+efNmI+FNmjTJMEK6Sx6mkR988IF88sknUrx4cZk6dSq3ItIJJ5wgJUuWFJiWzfj5559L5cqVDZPieWvXrpWtW7fKCy+8YCQ67iPlbd++3RYR2jdhwgTTl\/fff19gsp999plhnNRBxj179kidOnVM+8qWLSvUO3v2bHniiSdMW3\/99VeyKfkQAWVsPhwUPzcJqWjFihXy7bffGqkJaebBBx+UTz\/9VFq0aCGFCxc21KxZM\/niiy9k3rx5Jh1p69hjjzXSD\/2D2cBYqlataqazzz33nCxbtoxbMVPjxo3NOtzEiRPN9BSG+cMPP8jXX38tSGGhUEhOOeUUOfvss7PrbtSokdCe4447Tu6++26T95lnnpHXX39dtm3bZvIhiSJ9hkIhKVSokGn7n\/70J4G5HjhwQCDRf75EQBmbL4clGI3iI7\/ooovMB\/7f\/\/7XMDTbcqagMDkbzx2y+A9DgUFCMKP+\/fvnznZEHEkLaQ2pzd588cUXZdSoUQKTvOOOO4wkyVSX9TMYEvm4ZurMtZPY+OjVq5cgzXXs2FF69OjhvJ3jOlJ\/cmTUiOcIKGPzfAiC2wCYB1PKokWLmrU1djFhaNDMmTONtMTaGtek7dq1S5h60mPWwpj6waRgiqxlMSXlXjgi38iRI+XMM88063o2H1PEli1bChsbMCqYJBIi00faRzs3btwoSJq2jA137NhhJDfW1pDEbPvs\/XQIC2IflLEVxFGPo8\/Lly+Xyy67zKyr1atXz0xBs7Ky5MorrzRTNKamUPny5eXSSy+VNm3amHSmhN26dZMyZcqYpzNVhekRMi2EqXBtbjr+sNlgn0e9MCw2ChxZzLMHDhxopoqsgbEG+MYbb5jNAphl3UObCqiGlCpVylnMXLM2l5GRIUieV199tWGaX375pZmamgz6J5AIKGML5LB502jWqFgbg1mgfsFGAovuSEZM05hach\/imjQY0YABA8x62+TJk2XEiBECA+PerbfeKuSFbH5nz8i3Zs0as3jP85YsWSKs5x1zzDECMf2kTayDscaHdAYDGz58uAwaNEiQCGG61D9s2DCzTsYUtmfPngLxLNrHDi510z6YL8wRJjp+\/PhsyZCdVtpDGZ7Js2kDcSX\/IaCMzX9joi1KEAIo98Lo6tevL6yfofJhJcYEPUKr8SkCnjM2n+KizUoDBKpXry4zZsww02UkPnZP06Bb2oUoEFDGFgVImkURUASChYAytmCNl7ZWEVAEokBAGVsUIKVdFu2QIpDmCChjS\/MB1u4pAgURAWVsBXHUtc+KQJojoIwtzQdYu+cXBLQdqURAGVsq0dZnKQKKQEoQUMaWEpj1IYqAIpBKBJSxpRJtfZYioAikBIE\/GFtKnqUPUQQUAUUgJQj4krFxWAfeV3HNzKEhENekcS8lyOhDFAFFILAI+Iqx4TcLzxHXXnut8WLKYR1vv\/22QHhXgKlhyDxt2rTAAq4NVwQUgeQj4CvGhtNBvLK+9dZbwmEbeETFPTNUsWJFgeG988472T69kg9PUp+glSsCikCSEPAVY8P\/PL618FXPISF4TMUnFq5nCInD+HBQmCQ8tFpFQBFIAwR8xdg49QcX0dddd505Bg2nfxxzhhtpnAU+\/fTTwnQ1DXDXLigCikASEfAVY8MnPuc54kOL6SiHb3DeZLt27cy5k5wexKEbScRDq05zBLR7BQMBXzE2pDGmo7hr5oCQo48+Wo466igzEkxBiZPHJOgfRUARUATCIOArxhamjZqsCCgCikBMCPiOsXHGJId\/cHL3N998I4RsHpDGvZh6p5kVAUXAewQ8aIGvGNuxxx4rnFzEcWxNmzY1JxIRchgHadwjjwc46SMVAUUgQAj4irEVKVJEOIADdQ434h55AoSvNlURUAQ8QMBXjG3nzp1GYrviiiuMuoc1pcKc6uKLL5ahQ4fK\/v37PYBJH6kIKAJBQsBXjO2kk04STvB+7733BEb27rvvmsNyMbPCThRgOWiXMGGkFSkCikDaIeArxmbRxbQKKwMYnU0rVaqUkMY9mxZtiIrIq6++Kij6NmjQQF555RX57bffoi2u+RQBRSBgCPiSsR1\/\/PHy888\/y6hRo2TLli2Gxo0bJyjs5mfzYOnSpTJ79mxBEkQKxJrhiy++CNhQaXMVAUUgWgR8ydhQxH3ssccEE6tOnTrJlVdeKZs3b5ZHHnlEUN6NtnM2H0wMG9SMjAyBMbIxsXjxYntbw5QjoA9UBJKLgK8YG1NNzKboMhYIWVlZ8vHHH8unn34qgwYNEhgT92werqOh9evXS7ly5bKzVqpUSTZs2JAdx0xr7dq1EokWLlwokDMPcUvOdLdrt3w2jdCtjDONPJBbWu50Zx57TR7IxgmJWyIeidzy2TTCSGW5Rx6Ia0vELdm0cKFbPptGGK6cTScPZOOExC0Rj0Ru+WwaYaSy3CMPxLUl4pZsWrjQLZ9NIwxXzqaTB7JxQuKWiEcit3w2jTBcWb6t7A8thRe+YmyYT02fPl169uwpGL3jf81iwfW8efPMPfLY9GhDpEC3vEiCGNcPGTJEItHVV18tkDMPcUvOdLdrt3w2jdCtjDONPJBbWu50Zx57TR7IxgmJWyIeidzy2TTCSGW5Rx6Ia0vELdm0cKFbPptGGK6cTScPZOOExC0Rj0Ru+WwaYaSy3CMPxLUl4pZsWrjQLZ9NI7TlRo4cKWPGjDniXcYN2N\/\/\/ncZMWJE9j3KWbLlw4Vu+WwaYbhyfFt8Y27fXjLTfMXY2PHs3r27PPDAAzJz5kxp3LixnHbaaYYuueQSI7lhFE+eWEDJzMwUrBhsGa6rVatmooA+ZcoUw7TuvvtuCUe4TYKc94lbcqY7r1FZWbZsmTzxxBNCXuc94pac6W7XbvlsGqFbGWcaeSC3tNzpzjz2mjyQjRMSt0Qc6tixo9SoUUP69u2bA8vc+chr0wiJRyLyQM48xC05092u3fLZNEK3Ms408kDOtNtvv928Q4ytM93tmrKQ8x5xS850t2u3fDaN0JY566yz5Mwzz8yBPff69+8v9erVM98UcYhylohHIvKxPOQcW9IsuZXl3efb4hszQKXwj68Ym+33KaecIgMGDBDWxjCjgj777DPBtAqnkzZftCEDCnNhRxXiumbNmtnFDxw4IMWLFxccW4ajunXrCuS8T9ySM915fdFFF0nbtm2ldu3a+Spv63J7jk0jtPnCheSBnPeJW3Kmu1275bNphLZM8+bNhQ+eD8ymEZIH4toScUs2LVzols+mEYYrZ9PJA9k4IXFLxCORWz7GFRdbjG2kstxzK2\/TCMkTicgDOfMQt2TTS5Ysad5rG7ehXWM+5phjst9zW5bQ5gsXkueqq67KMbakWQpXjm\/LNCjFf3zJ2BKNwbnnniswmMsuu0z48JAESUv0c9zqK126tPH8S+h2X9OCiwBjildnwkT0YvB738kJfefKzRNWy\/z1uxNRZYGto0AwtlAoZJjL\/PnzBcmPcxNCoVCBHfRkdpxf6O+++04Ik\/mcdKsbpgYz63xeaalwQlFpM2KZTPhim6+6yZgGZWz9wth8NYDamPwjgB89zqcgzH8tBafk9zsPCEwNJpbVvKI817maEE7rXct3khtjGpSx9S1jW7VqlVx66aVmColCbY8ePYyibsF55bWn6Y6Ak6kt71dPGlQukd1lrrOaZxrJDUku+4ZeRIWALxkb+mxYHfTp00fYSGAHs3Xr1sJxfNyLqmeaSRHwMQJIaEw3P\/1mtyCduTU165AEV79SCbnljdVutzUtAgK+ZGzsXKJ3xk4LbcctOAv+iMLcIy0csYaGDzfsQlu0aCFIfmor6o5WMlKDtA6TjP5HqhMJDYbW+rllZpoJ00JSY00tXLnpN9eS8hlFhTLh8qQqPUhj60vGhtkTyrpbt27NHrPVq1cLWsyRTKo4DAYlxCeffFIWLVokvXr1kmeffdZYDATVVpSPITcxNfErLdywT5b868ChXb1dh2h3WlOkvsLAINbP2OWEMdUcuMAwtE27Dpi1NNbTsl\/wCBfkQ7KjjgjZzK327dsLp7uZSAH+40vGBvNi57J\/\/\/6yZMkSQU3jlltukTvuuMPYeoYbL84izczMlCpVqpgsOKbcsWOHUey1ejwwTb\/aivIh2I+AbX+IjyE3MYXxK3UYvVJ6TNomHV9cY9aH\/NrORLQrUl8ZR4gxhSnZNTOmnUhp7H6alzSKP0h0lKEe6oyiSFKyMGPSzYM4oYUp4Y1jwYIFMmvWLKOmcc4550Ss9fTTTxe0o1FCPHjwoODDDddHaD6Xi2ArSqWYcGE\/iridaMJTCWuDbvV+++M+8ysOE+Olnbd2u5xcTKT92SXkyXaZhiZ2O0OioYV31hKl1GHw0S3V5O1OJ8lHt5zhivuWR+tlp99+URmB6pQralRh3N6FSGm8E7wDMMpBM9aFreOtt96S0aNHH3Gf7wFXXXjIifScRN3jW\/rXv\/7Fp+UJ+UpiQ+LCmwemGBBKte3atRM86LLGxj3yWKR+\/PFHIQ9mV3feeadNlq+\/\/low7cEwF48gTGtZs8vO4HIxbNgw46eN6W8is9ydkQAAEABJREFUCYN92vzDDz+Is94lazdJ\/ymrpc7jXwjMrMf5JWT6deUM9WtUQqDG5UWgSsUPSDRU+Odd4jX9vneb\/PDNSiH0ui3Jfn6RX3YbvIv8sseEuZ\/nHO9EXPMOjO5QWp76YLOM\/nBdjvfJ1s8PKF5xbNyGfCv8wP7000+u5Wy+SCHMCmsgwkj5uIfPw7vuusvlS0tNkq8Y2wknnCCskU2cOFHatGkjN954o\/Goi1ddpqZsBliTEeDBvArDeEyuWFezmwQ333yzYKuIQTASG9NT7EMpA3HNTivXlrD3wzymTJkykkjCQSZtQDvd1nvw6Ay5afI2mbX+gGQd2tJffM950r9dNaldtXxCn22fl8owMzNTmOoTpvK5XjzLbWyT3Y5WdSrKsCsrS\/852+Wb\/UWPeF9YxmHKmLsdJ554ovDjzlJM7nvRxhnTaMeWbwnTOvt9pTr0FWNj9xMmQIikg6RWtmxZgfAgsGLFCrOBEA4kppx4\/kAUb9iwoVAPefOyFSUPaiUMHC9FIokXjRfK1jl55W6p++QycxD0tN61DjG2imLvaVg0UFjkHttUjV\/XeuUPvTeZZh1z8eYDOTDDdyGbB25t4XuA3O4lOo1viXVtvi0vKCdj86IFLs8sUqSIYWD4UbO3mcqxK8rA2LTcIflR78B+j6ksxPS1QoUKZsrKJkSqbUWdbbQ7ZEhpLAazKOy8nw7XrNEExewmyHhnNa8obECg4zY\/RXalQRpbXzI2XIMjyvbu3VuQvKDOnTsbe0+mq+FeSNbhUAvh8BccVEJMY5ECYXZe2orC1Fj4zTo09cw69FKG64OmKwLRIoAaCDpuMDdUgig3efJkefHFF7ks0ORLxsaIwKRYP8PfE+cdwKyaNGnCrcDRqM93C1NQfmHTnakxpQmKSkDgXiSXBk\/\/Q4EX9RV+PJ\/+Zwm5c1VFQUWIuEuRfCcFaWx9ydjYRWQKiQXBNddcYyQ1juMjjXv5HhkPCg6bt1X+sWafDO902Lj5yCZoiiIQHwL2x3Lwexvkqx2H6\/reGNdv8IXFwuEWpfavLxkb0012R99++22BcHXcqlUrYTEyIyMjtQjF8TR+Mcct2Cz9m5WUOuWKxlFTcIoGaR0mOKhGbimWDDAyt1wo9bIE4nYv1rQgja0vGRsbBKyLoc4BYTOK+sbvv\/8ue\/fujXU8PMkPU+OFGnpFZal9SsFgap4ArQ+V8Yu2RkSBdzFihjS86UvG5obzpk2bJBZNZtx\/4xEExUSr34ZhfCoOTOZFgqkx\/WxQOTgSphvusaYFZB0m1m75Oj9SWSoaGKSx9SVjgxlZiwKsCiDW2nDnHc1UlHW4wYMHG7MSBnzp0qWSKiP4nEytBI9XUgSSikA6qg3FC5gvGRta0tOmTTMnS2FVAHG2KBIY09FIncYmbvjw4YKVAtNZ8mIGYtfn0LxGe3rx4sXcyqZE2IrC1FhTe7JdpllTY00CwpQFUxeu0512794ta9asETytpHtf6Z8fxvZvTf\/\/zNzsF9pxgd0x30W8tqKxjC1mV7HMsBzNTcil7xgb54di43n\/\/fcL4CC9QXws2IPirSNSz+fMmSPFihUTdlFtPhR38zKCj8dW1Np9wtTYKMCmD3s5CAsKJEgUjImnOxWk\/vqlrw1K\/xp2HbfscUfJtTWKCt8QTDgeW9FY+utmK2q\/x1SEvmNsSFdjx46Vf\/7znzJixAgZOnSoIXTZmjVrJuyYWmAYLDtlhelhUjVr1izBjTgbEDYfIWZNhOEov7ai2H3O2fCrsftkTQ1bPqftnRf2hM7np\/oaUxokYsJUPzvVz\/PT2KLPltU80xwEw9QUwpHllr2\/GptSZkF8A8xY8osTYxrt2KJgr7aiDm6DYu6gQYPkqaeeMi6IcAduiXMN8dRhs7NjihIvU1WM4JHqsBXlnMe6desaB5NIbtj0Yfhuy3Gd2wg+P7aiP\/4kMnnlrkO027h3bnRGyRx2eyy28mxeKK6VgmULmtd4+W1ss5pXlOX96mUTzA7mdueUDea95MceyqtfibgPE2T5x35zqQ59JbExZYOpEbL4j6UB9p6W8lLQRaKDyUELFy4UmBubBrg+YpcUt+IQ1zVr1owLa\/SG8J\/G7ifG7PxCxlVhmhRm3UltRf0zmDC38hlFpfmYb2XmlmPjaliQxtZXjI3FftbWMFrHxhNbTyeRRp5YR4fdVKaskYzgY6kTpoYJC4qRytRiQU7zeoGAZW7r9xXx4vGePNNXjM2JAOtnWB\/cd999YgkpLloFXaap2JkShkIhY5aVCCN4dj6xw6OtytRAIScxjVFb0ZyY+CE29oqyctvpf9hb5bNBQRpbXzI2pot9+vQRVCTq168vllCwLVy4cD6HJb5iSGkcpoE9XlbzTLOOodPP+DDV0opAshDwJWODoeFcErdFLVu2FEtsLLBgmywwctcLM2MNjbU0pDQ79cw6tEibO6\/GDyMQ0zrM4SL6NyAIBGlsfcnYUOlgLQ3faqkY818q1JfXv\/pVmGbCxJDMYGQQcUxWsv6Q0hpULpGKJukzFAFFIA4EfMnY0FpnV5NdUKaf0e6KgsOWLVsEN+JsNSPp4VE3L1vRn87tJp9s\/ZMgncHEqIdtchz52e3zLJXSgCVPCtI6TJ6d0Qw5EAjS2PqSsSGt4QkUfTMOPrY7o3ntinI6D4q2HTp0MDpsPXv2lJdfflkwn0Ltg+P83n33Xfnwww8FRWA7aiWm3CjPNwuZdTMYGbtIMDUcQ8a7joa29quvviqE9nkapgcCjGnwxjY9sM+rF75kbOx8sgNqd0NtOGDAAMGTLnZvbh1DWtu5c6eg\/xYKheTSSy8V1EeWLFmS7csNzWu0p2F2zjqwFUVKTDTxbMxLpk6daphtouv3W33sPDNOb775Ztr3N0hji1khMyHC\/L4zsY6t2oo6Ocyha3Y+sWvj8BZOmGI6itsiNhWwLBg4cOChXEf+37Nnj8D0YIQ1atSQrl27Coa7DGY4W1HSmbZiK8oUNtFkz1ZMVv2Jbm+89XG2xCuvvCL33nuvWRKItz4\/lw\/S2Hbv3l0WLFhgvNzkF9NYxxZ8+Lb4xiTF\/3wpscHAkNqYVuLRo127dgJj4BTrXr16CVIZOKHrhuItbo2wFYURYkDPeaQc1YerIz4wmB1mTZTJTYDOc9B5UxovikF6YsABL7feeqvAbFI5xnxbfGO5v7tkx33G2P6\/u2wtw8hsCi5X8PzBOppNQ\/l23rx5ggkVtqIlS5aUOnXqiLUD5deiUKFCgtEv63W2HNc2D2kAX7duXWOCpaHioO9A4t4Bvi2+sVSTLxkb6h6YQXXq1EnuueceY3lwww03GKbFGhv33YDKzMwUprDr1q0zt1EXYU0NqQ77UBR\/Ia7jtRU1D9A\/ioAi4EsEfMnYQKpbt27CjhMbAVgeDBkyRDp27ChMTfv160eWIwgmxpSU+5QZOXKkYYz8AsPcEmUresSDNUERUAR8hYBvGRvS1qhRo+Sjjz4SvOe+8MILcv311xvw2FwwFy5\/qlevLjNmzDBlUA\/BbjEUSpytqMsjg56k7VcE0g4BXzI21tKGDh0qlSpVMhsFVapUkeLFiws7O+i4pd0oaIcUAUUgoQj4krHhvphp5eWXXy6oeyCFsaOD9MYaWUIR0MoUAUUg7RDwJWPD0J1dUc4JYOdz48aNgufP\/fv3G48faTcK2qECh4B2OLkI+IKxobOGThrEtZXWWPyvWrWqoIPTqFEjgcmF2xFNLkxauyKgCAQJAc8YG3ppLPI3bdpUcOl9xRVXCHTJJZeYE6aYjmJWhfIt5kjoqz300EPiPPMgSEBrWxUBRSB1CHjC2NgcePjhhwVLgSlTpog1dH\/nnXckMzNTYGqobGAWhWcPmF\/btm2lc+fOwnkIqYNHn6QIKAJBRMCVsSW7I6yhwbg4ootpp\/N5TEXZAWWzAEnt7bffFku4CtepqBMtvVYEFAE3BDxhbGwEoIuG9IV1AVIZxC4o0hymVNOmTTMSGgq3y5cvl+OPP15Q9aCsW0c0TRFQBBQBi4AnjM0+PCMjw7gTYoqJMu2ECRMEV0Oss1155ZVG1WPAgAHG\/Q0WCEhxSHS2vIaKgCKgCLgh4Cljwz8UPptwL8QZBxxafNtttxml3AsuuMCoeDiN3nN3gHv4W1u5cmX2rblz5wrSH+Vxz4KPtrw86GYX9vJCn60IKAIJQ8BTxoarYauvZnuEV1L01p599lnB2+0jjzxivG7g9ZY1tuOOO85knTNnjjlSD6+4JuHQH9wZvfTSS4I\/MMywLr74YqHM0qVLjR8q8lIndTk96B4qqv8VAUUgjRDwlLGxccC6GhJbw4YNzTF7TEVxK4QyLrahuCPCEwdOJFmTQ00E\/E888USjHnLqqacSzSa8e+BckgSYJs+AieHCiKkvcTcPuuRXUgQUgfRAwFPGBoQo3iJl4QAPp3RnnXWWHHPMMYIR\/MyZMw3zQr8NYo0NqYxytWrVEtbi8MFGHGLHlM2Iq666ytiZsmbH+l0kD7qU4xCXWGn++t2Smz5bsd6sB+bX9bKWW5hi\/PR5yX7nNm\/ezCeWcvKcsY0dO1bY+Xz++efNmlooFDJupfG7Zg9xsSEbDOyMhkMJh5O4OmK6idRHvUh8rMWF86AL8ByxFyu1GbFM3KjFxP8K1OrlTQK1GbFcOjz8hqFOf70t7d1ls66pdLXvxrl3796C4IDb\/FSODwIL31i4bzZZ6Z4yNqSvL7\/8Uh577DEpUaKEVKxYUYYPHy5TpkwxGwgS4z8YW6lSpaRChQoSCoWMY8odO3ZI6dKlBUZnq+PaetAFdE6pGlN\/p8zs+Oc86R\/Xlxc3mta7lky582LhdKus5plyV\/s6ctVF1aVcuVPkwBltDe29ZLBA51z3qNz8yIi0dMPNWKKfyA9MKl1Qe\/Gs22+\/3bxSMAwvnh\/LM\/v37y+NGzcWXOvHUs6ZN9axBR8OSeIbM0Cl8I+njI31sqJFi0qxYsWyu8waGDpuSFnZiVFelC9f3lgmYDxPEU4RQsLDySRec\/EMAnHNuh15LLEjWzcK9+AX1KgsbtSgcgmBOp9XWrKaVzQEk5vU42y5\/fiPZUqXMgLzg+F996dT5aPdpaTs6TXNxkg0zw1KHpYBMH1r0KBB2vUt9xhgDQMTr127tu\/7ateYWWfO3Y9o47GOLc+035cjTMmlp4yN9TEYD7uYHODy9ddfy6BBg4SNAdbLYkXg9NNPFzYcELVR93jttdekb9++5qWDuXnhQbf0IWmR033OPzPTMD6Y3vBO1eTTb3bH2j3N7zME7NgS+qxpBb45njK2UCgk6K3hUBIJDfvRsmXLmpN0ojF2Z5MBL7tnn322GchQKGRch7Mm99lnnwlnWzK9DYVCRjWEcxFJ79Kli5mqmkIe\/KlwQlHz1HRkbuxEf\/fdd0JoOql\/0gYBxjQoY+sJY2MKiuoGboqYGqLigetvJCykLaQ38qTNG5GrI5ax5UpOixbeNm0AABAASURBVChLC\/yYEKZFh7QT2QgwpkEZW08YG5sGqG6gwtGqVStjKYA+G4RSLW7B0WPLRjR1Fyl7Esxt\/vpdKXuePkgRKEgIeMLYWFdDdQNLABgZ6hmod0CTJk0y+LOBYC7S9E\/5jMPT0TTtnnZLEfAUAU8Ym+0x01CmnTA6m4a6Bmncs2npGLKDilLw4Pe+k+93HkibLgZpHSZtQE9RR4I0tp4yNlwRYQLFBgDG6tC4ceMEt0WofSRqvPxoBM\/uaFbzTBn83gapOXCBIZhcovocpHq0rYpAohHwlLFhDYDSHzuimELhqghlPgzfcUaZqM761Qg+q3lFWd6vnmQdYnBMTWFyJ\/Sdm83k5q8PnkpIkBaYE\/V+5VUPEjljiYSOhUvr55YJ42yJOPfzqsfr+0Ea20JegMVUEy8ePBtvHVlZWYKKBh450GPLyMjgltg8JhLHn2iM4NGQ3rBhg1FTQOROJCGV0me3Ok8uJnL7RWXkrRurycI7ax26Li0n\/VmMJIfJFi+\/lej4APgwkOwgPhRLfBjR0Lc\/7hOl6DDIC8+FG\/bJ9NX7ZPyircI4MCaMD8RYQXb8GMtBM9bJvLXb5fCYl5Yn22Wa8a5TrqgxzyO\/2zuSiLSDBw8KmgbMhhJRX1518C3hkiyOzzauop4wNnTUpk+fLj179hQYCl5zbS+45uAW7pHHpscT5mUET93Dhg0z7o62bt0qiSSYM6otWEPkVW\/hn3fJtTWKyugOpWXJrZky\/bpy0r9ZSbm0clE5p9RRhul+vWW3jFuw2TA+PiBLfDjRUJ3HvxCl6DDIC88Oo1dK\/znb5c4pG4RxmLNqm2FcjBEf\/glFfjXjxxgypown1K9RCTPOjcuLCRlz0ikDc8vrPcnPfVSr+IHlPBFTPh\/vOcwKIYEwrzpQusdOlG\/LC\/KEsbHj2b17d3nggQcEDx7YsHEaFXTJJZcIkhvKuuRJFChMeyPVhb0f5jFlypSRRBKbIWyOoJ0ea721q5aXHk2qSP921WRU17Nl+s21DC2+5zzZ8mi9bCIejjDjUqplzNniwcEN3\/l9qsuULqXl2wdrmbFgfMhHCDFmjB\/Uqk7FiO8VY03+Jf86II\/P3xcxb6zvEfmx5uEbYO2aeH4oMzNTcPlFmFd5viVsRSN9c8m85wljsx3CPhPX3\/wKYMAOYRmABwLOELX54g0ZCAzfbT1cWyN4m0ZbyMc6QiKJtUJeqETWmbuu004+RsIRu69Kh+1448HBHd9jpXxGUck9HvmN8wyY71vLtsvizQcSVq9tD+eFQDaezJBvqcDailqmkuywXr16guE7KiQQ1zVr1kz2Ywtk\/UyngmJ248cBgvniSIFp8HyfbR4FaWw9ldjCvlgJvnHuueeKV0bwCe6KVlcAEMArTP1KJcyGAhsSBaDLCe9igWBsoVDIV0bwCR9FH1XI9CYo9oQ+gu2IprBGhxoQu63sihMekSnFCUEaW88ZGzuG99xzj3Tq1ElYX8PRJDujKR4zfZwi4DsEsppXNJserOOx6wqDUwkuumHylLGhmDt48GBzPgHNLVeunKBvww4l90hTShUCiXlOkNZhEtPj5NaCswSkNxS5WXuzStyohaR6DS5IY+spY8PLx++\/\/y5NmzaVQoUKmV0gVDxY4Odecl8ZrV0RCA4CMDgkOBhcVvNM2bTrgFmDg8Fh2RCcnqSmpZ4yNlQh2H5GadB2l6kpTA0lXpsWLkSqu\/\/++2XlypXZWebOnWvcIOFBF99u2J\/60VY0u8FpdhGkdZggQu9kcKiGwOBSNUUN0th6ythQFmzSpIncfPPNgltwTpXCRxu+1fNyDa4HJgfxs9Q250IgriiqIVaCY4rKOlxcFaZRYU8ZGzhyDsE\/\/vEPeeaZZ6RLly7CNWnci0RoUsMETz311BzZMBvZvXu3SWNNAOaJAjDKghkZGUIc7enFixebPPYPpl2YilAm0USbwtmKJvpZXtcH9mvWrJFdu3YZEzCv25Ps5\/thbLE1ntjtDGOvetO4la64s3Ydr61oLGPLt1TgbEUtMyFk6sipRpgc4TkXN+EvvvgityJSrVq1BPMrDoSxGZHy2F1F4uMchQkTJkjnzp0lGlvRjRs3ysSJE+XZZ591JWsb57xv0wid6c7r0aNHy+TJk8XaijrvUc6SM93t2i2fTSN0K+NMIw\/klpY73ZnHXpMHsnFC4paIQ7a\/Y8aMyYFj7nzktWmExCMReSBnHuKWnOlu1275bBqhWxlnGnkgZ5rTDtiZ7nZNWch5j7glZ7rbtVs+m0a4dOY4ubbYAsFqofHgT3JgT33WVpQfdOIQ5SwRj0Tko7+8y3ZsSbPkVhbTSMwk7feZytBTiW3Pnj3mwBWmojAWNhGYYuJmiHU2JxBMUwEJRVsGyXnPXmOSxXmWeOTFbIoyeR2YbMvCKJHkwpG1jXPet2mEznTnNfVWrVpVYNy58xG35Czjdu2Wz6YRupVxppEHckvLne7MY6\/JA9k4IXFLxC3VrVvX2BTaOKFbPptGSJ5IRB7ImYe4JWe627VbPptG6FbGmUYeyJnmtAN2prtdUxZy3iNuyZnudu2Wz6YRUqZt03rybIsSsnbPUbL09yo5xoAZDqZ9mA6SF6KcJeKRiHyZmZniHFvSLOUu27p1a+Hd5wfdfmOpDCMxtqS3AzGek6Z4QRYtWiRMF9lQKFKkiLAx4GwADArGheePcHak3KeuChUqmFOo6tSpIzt27DBMBUZn6+M6t61olSpVBGP8cFS06GGbQOd9m0boTHdeN2zY0AwweSDnPeKWnOlu1275bBqhWxlnGnkgt7Tc6c489po8kI0TErdEPBK55bNphJHKco88ENeWiFuyaeFCt3w2jTBcOZtOHsjGCXlXYRa507mXm8gDOdOJW3Kmu1275bNphLZMl2aHDf7f\/7GEFC5fK\/udJg8bdSzd2LykWbJp4UK3fDaN0K0c35RXZ5d4ytjwoIt\/KLwALFiwQPhF4MwD0lgLs4wo2rB8+fLCrqr9lViyZIngWQMpD\/tQ1EggrtVWNFpUY8vHmpbaisaGWaJzN6hcQqxJ1vcJdDsfpLH1lLHxazdw4ECzDjZkyBDB6SSDjMsifg25joX8eGByLO3XvIpAohBAqRfVkIK6U+opY2MQYWDnnHOOOf2dXwSmbjA5JC\/uRyKmsZyXELQDkyP1Kej3mJZUrFjRKFsHvS9Bb39W84ry6Te7JVEWCkEaW08ZG+toSGwsNDZr1kw6dOggnHvAQnuJEiWC\/l5p+xUBTxHABIsp6YQvtnraDi8e7iljQ9eJ3U\/OF\/3b3\/4mI0eOlNdff93o4YRCIS\/w0GfGiQBSt66xxQliAosjteEZBMkt3mqDNLaeMjYn0JhQsejPdvT27dsFhue8r9eKQNIQSOOK7UbC8Pn\/TuNeHtk1TxkbyrVMOzmCj3WZsWPHGgsE7DtZezuyuYdTsP187bXX5PzzzxdsQlHIRUrgLmWxEUV1pGXLlrJq1SohP\/pt5G\/QoIE5tAUtbPIrJRaBIK3DJLbn\/q3t6vPLyPJtB2Xdf4rE1cggja2njC0UCgm+2O69916jw9arVy9hQwC3RZHUPbAkwPRq2rRp5uAX1ubQeGfNjrLEFy5caE7BevnllwVt69mzZwtTXpR3P\/zwQ8HMKq5RjrIw2towVcIoi2i2gCDAmAZhbFlrY4f08x3FAoJs\/M30lLHRfNwWrV27VmbNmiU4mERlA4N4bCu570ZIejBDlHG5z7SVa6Q1rjGsD4VCgnkW3j\/QZ0OCy8tWFGaYaOLZr7zyikydOlUSXbcf65s\/f75wQM+bb76Z9v0N0tieevR+gbEhFOT3vYl1bCPbivLlJo88ZWwwL8ypnn76aSN5YVsGYYWA0W64bsOgUBH54IMPjJNKpqXYjWKiRTlOuapRo4Z07dpVMNxlMHFiKX\/8w44UI12ipMP0hg0bJkxhE032bMVk1Z\/o9sZb37XXXmum+vzwxFuX38sHaWw3Tbib113GfvRNvt\/zWMcWfPi2+MbMw1P4x1PGhhUAU0\/W1h599FGxlJWVla2sa7HA7jO3rSgqIphHkR9PvJs3bxakvxtvvFFWrFgh11xzjfCBwexQBrZ1OUNAZ\/o6fvx4UVIM0vUdGDFihHEzPrzP5Sl9z\/m2+MYkxf88ZWyso7FJEI0yrtNWlKnqoEGDDFShUEhq164t2J0WK1ZMsA+1dqD8WuCZF0NdGKApcOgP1zbPoagAfN26dY2Br4aKQ7q+A+yQprpvfFt8Y6kmTxjb3r17BQnrkUceMbadbdu2lZtuukmYQkLcI084MGCGX331laAWwo4nax0AyPQTBrdu3TpTdPXq1cb\/Wuy2oqZ4Uv\/g+sV6\/mUN7sILLzQON5Fik\/pgrTzpCLDWi7ssvNW8\/\/77SX+ePuBIBDxhbMWLF5du3bpJnz59BMsDXBXB5IhD3CPPkc09nMI5oS1atJA2bdpIvXr1jB+1vn37CmtvTFn79esn9evXNwq\/7LryKwVzw4Fl8+bNjccD6jhcW2r\/wnjZ0MAUjCfjzwoJlB1b2vbRRx+RrBRgBNgI69mzp+DQgQV3\/bFK\/WB6wthwn8IH\/uCDDwpecNk8CIVCgjsiCI8c5AkHRygUMt52Oa6PHR42D8qWLWuyV69eXWbMmGE2I9544w1BPy4U8s+5oqz1MY1Gx44GI3XilQQplLZ6uZNEe5TiR4ClFcYS7zX8QGNhE3+tWkMsCHjC2NgNRUJD2kIPCK8ebByghxZL49MhLy6aWBu0fUGCs9cFLEyL7vIOo\/yNMisd4qAiduu5VkodAp4wNkRzfskuv\/xywQfbX\/\/6VyEt0rpa6iCJ\/km8xEwr7VoZJVlfwRICKwfclLNTy33UU1BfQFePfJZwqsk6oY2z82uvNfQXAqzlMsuwrWK3nfVgvMdi0cKSCjMNxpR3g3xskOG9lmul1CHgCWOje+xWElril82+DDbNzyEvMXo9rI3ZdtJ+trdhbOjioW7Ci4\/S8Zdffmm22VFvsfkJUTZmjQ0plk0PbGVJV\/IPAiwXMKNA55L31LaMtTR+uFgSYdkDtSVM+zh7g7FEWZz8zEhsGQ1Tg4BnjC013UveU\/gVZn0QV8v2KbzITCXZrCANNRTWW3BPTtyNsJVFYZiNAxSOUVFxy6dp3iGAFIYaUfv27XM0Yt68ecImVuHChYUfJNZ50aNkLPFUgwsu3oXcP2Y5KvE4kq6P94yxYauJegZTLwafTQBC4uxghjuwxS8DwUEVWDsgcdk24Z2EaQjTbNL4pWYq4sbYBgwYIDjIDIVCZof4448\/lueee86op1BWyT8IsNvOWPNDZVvFkgKbAnb8Q6GQ8O5u3LjRMDkkOH6oKGfLaJg6BDxhbOx88mvH4StuxD3ypA6GxD2JXU9+4RNXo9bkVwQYZ37I\/Nq+gtwuTxhbugLO9NS5CcJmCM75mG6ma58Lar+QyplislkEBmwAsb7GUYvElbxFwNeq1E3XAAAGM0lEQVSMzVtoYn86UiYWEOyeUZoQryNMZYgrpQ8CoVDIKHozu2B3FP1DliIqV66cPp0McE+UsSVw8PACjAUE7s3ZBJgwYYIQJz2Bj9GqfIIAbrH40ULVA1UenC+gbO2T5hXoZihji2P4mYpgGsUmgK2GnTEWjj\/\/\/HOj3kHc3tMw2AjgTQZnDLYX7IbiWYaxZvOHQ4PtPQ29RUAZm7f4++\/p2iJFIA0QUMaWBoOoXVAEFIGcCChjy4mHxhQBRSANEFDGlgaDqF0IOgLa\/kQjoIwt0YhqfYqAIuA5AsrYPB8CbYAioAgkGgFlbIlGVOtTBBQBzxGIgrF53kZtgCKgCCgCMSGgjC0muDSzIqAIBAEBZWxBGCWP28gBOXgE5iQtS5zb4HGz4n48FgMvvfSS4IIIkyichzorpd\/PP\/+8Myn7GoP3oUOHCnai2Yl64RsElLH5ZihyNMR3ETzIYjZkCZfotpF4tsDPv40HIYSZTZkyRfLrL43DWnBZhB\/BIPS3oLVRGVtBG\/EE9hdppnfv3tK6dWtZsGCBrFq1Stq2bSsYhXPeA1INj5s7d65ccMEFgrSHU4D+\/fsLjkSRkgjJg7SEhAST5NQx8lLPww8\/bCQq7lMn7thxMNC9e3dzTgYS05AhQ4Q0HEHiuRaGdccddwiu2qkbyQri2hKeV3ACiudbmxYuRDqlPRAORmnD\/v37pWHDhuaIPU5cC1dW071BQBmbN7gH7qk9evQwHmLxEsvBNBxQQydwh44XE\/yQDR8+3HgB5ixNGAvHKq5fv15eeOEFc\/YrLn5gJEhLlHUjyq5evVrwPvvJJ58Ifs+mTp1qsuJmHcmRPEhLixcvFs4d2LRpk3AeK8yP6SV+8bZt2ya4acc\/3vLly4XDi00lf\/wh\/5lnnvlH7HDg7CP95FxQ7iCdIqnSz8zMTHOwNe2iL\/Qfd0XkU\/IPAsrY\/DMWvm7J6NGjxXo75mAa69GkUaNGxp35hg0bhI8fqQamwEnoK1asMFIcPulgAqFQSPCQgVeUcJ2FMcFAYJScBYG79GXLlpnsSEv4u8OrBs9n+stBKjAtzmXFhRBrZkiHHKCz\/BBD46AcnpfbTxoMj3RT8R9\/nH2krx06dPjjjgiH7eDO\/bLLLhPWG7kBc+WYPT1eDzT8RcrY\/DUegW0N0zGOoYOZwRQ4pWnatGkC82B6aTv2yy+\/RFxwZ4p39913ZzNRpDemrrZ87pCpKO7Ybfru3bsNE2rSpIkgITINrlevnsD4bJ5YQ9o\/btw4Oemkk+SGG26QUCgUaxWaP8UIKGNLMeDp+jgW03GFDkODEcyYMUOeeuopwfEiU0U8zJKOVAcTxPkmkheu00lnzQtskIaQwpCokJIeeughYUrKPTeCmcLAYHAchXfrrbcKh6zUrFnTrOOx9ocUmbss7tojTYmd+Znu0h+O36PN9h4SI20\/\/vjjbZKGPkFAGZtPBiLozWCqeddddwkbAPXr1xecbfbs2VOYErJJwLSO08dgcEhxTBuJc0QdO5OsxcHsmKrCrAiZ5nJGJ9fh8GnZsqWRoFjI79ixo3AIN9NVmI09BY147vI8g3VCGFPue844DBNp7fvvvxc2L2CShKz3sY5HX5DknGX0OgYEkpRVGVuSgE2navEa68ZcYFyQ7StrWyz6I3GNHz9e8B4cCoWkffv2smjRIrMGd8OhqRzMJBQKmWMHWVObPXu2jBkzRh5\/\/HFBIkLqIh1iWkoaz6cd9lk8lzQYy2OPPSaoXfBcnhUKhcyOKIynTZs2hvHZcjZkBxXJDimPOmgv9dn7hDyPtnCPdT6kTQimDTOjry1atIhrmstzlBKPgDK2xGOqNXqMACokMDgkwDp16ri2BmbWtWtXYQ0ORuuaKUIiqiw\/\/fSTIHVGyKa3PEKgkEfP1ccWUATYzWR3MZnd57Sw6dOnGwkw0qYBu6zopIVCsW8GsKbItBtpMpl90brzh4Aytki46T1FQBEIJALK2AI5bNpoRUARiISAMrZI6Og9RUARCCQCytgCOWzp3GjtmyIQPwLK2OLHUGtQBBQBnyGgjM1nA6LNUQQUgfgRUMYWP4ZagyIQdATSrv3K2NJuSLVDioAioIxN3wFFQBFIOwT+DwAA\/\/8jKGR5AAAABklEQVQDAGDxYl+VXcDKAAAAAElFTkSuQmCC","height":187,"width":310}}
%---
%[output:54cd2728]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATYAAAC7CAYAAADv5bIbAAAQAElEQVR4AeydC7xNZfrHn43\/OEMJyaUoRwil0iAVynRTKjQ1YSpNSkVRLp2GmqgQ0kXJJV10odJFmdJtSNQwkqiQS07ujEpGksr8z\/fl3a2zrL323s7ee619PD6es9Z6r8\/7W2v99vPenlXif\/pPEVAEFIFihkAJ0X+KgCKgCBQzBJTYitkN1eYoAoqAiBJbAE+BVqkIKALpRUCJLb34aumKgCIQAAJKbAGArlUqAopAehFQYksvvqb09957T2rVqiVjxowx16n+89lnn8kJJ5wgffr0SXXR+5S3fft26dSpk5x++umyefPmfeIzEQCOtJd2J1xfCBKiL3ojnCeqUhgwd+qK7rSB59oZHqZzJbYw3Y0U6QLhQDwQEC9Fior1LCaTdaEAL9Xo0aNlxIgR0rBhQ4JiSrK68aKm6weI+zBkyBApX768TJkyRapUqRJT77BHgPvZZ58tvXv3Fu5HGPXNemLDSuFhtMLDCdA8SLzYCA+SjefXnngrzvyQAS8DcZRDHl4gyuCcI+US7yWUTTqEX7RYN506qeu5554zlhx1UR7pyUd+hPKc4bZ+dOCcMqy+pEO4vvTSS2XNmjUyZ84c6dq1q5CeOCvU5yyfeHd56EgaK+Sx+e3xP\/\/5j3jVhQ7oZvNSls3DkWviaCv4cm7bSjx1EWaFa8KRCRMmGHI4\/vjjuTRtQ3eblnqpH0lGN+oAKwodNmxY1Lom3JbNkWvSeAl1Uj\/pEM4JA1\/K5n5wX9q1ayebNm3apwjSkoe8CDi5E7377rvmmSHeiRl1OHHgnDDyozPped4o35brfN6uu+66fSxxZzz5nfV17tyZooX7YU5C9ieriQ2gX3nllUKQ8gBxI20gD1OvXr3spfDQ2nhusDM\/D93NN99sXhabYdSoUYYguKasZ599ltN9hDIp20b897\/\/lRtvvDFmd4267rjjDpvcpCM9+Wwg5VGuvU7VEVKoUaOGrFixwhS5Y8cOWbdunTRp0kQqV65sXmonLiRy40qYl\/zwww8ChrTPxlMW94prjlxzTlvBl3MrtJe67DVHaxnw4s+bN0+OOOIIKVOmDFEyYMCA6P0hgHqpnzZx7ZR4ujnTcu6nC\/FOgUSol\/ptOOeQ65YtW2xQzKNXfnACL5uJ8pzPDJYr5EO8GweeVcKIs0JeyuAaLJ3PG4RJHuIQyr388suFe8Q14nwesTixPrkflEX8vhJcSNYSG2BOmjRJDj74YHnttdfkq6++kltvvdUg+dZbb5kjf2LFk5+b0qxZM1m0aJERzrm5CHkRwogfN24cl\/LBBx8UIj4CeSifeOIJToV06HLxxRcbq4m8JsLjD\/qS9qyzzhIeYh468hFGOWRxtoXreAIxTZ48WSAudKecgw46qFA20kBitB8c0JG6a9eubQg2EVwp8LDDDhN3XZFIxJAkdVMuWKILuGGlcCQvehFPOq6t0F73PePlwjIg\/9atW6Vly5ZCm8B9\/fr1pq3UY8v74osvzAuZjG7ogU7owX25\/vrrxU8X0jmF+hHKQQ+Ec3DlB4SyuQYLSITunF9+nmlwAC8I2aalHFs2uIBJLBzs\/bV5qZ+89913n3ne0c2GUa5Nx\/HDDz80GBLufB55zqmPHxZ+YLgf6ECeMEnWEhtgAuqxxx4rubm5BlNIgQeHG2p\/JZ3xp512miFCXob8\/HwhPw8jFgzCuSnI8ce+RPxC8aA5oqKnWAdYPNRNOUTw8PBAQFpcu4Wy0MeG8\/Bz3rp1aw5CPvJTjglI8R\/qof3gSN1WH64Jd+LmhWssdSDNmTNnysSJEwXrlheHF4j0P\/74oyE9ixPkdPXVVxNlhBeGe8ML27ZtW9PlwkowkQV\/0I24glPzn\/yHH364+QGhHrCnvIULF3qOv\/npZgp0\/ImniyOpOQVDTuzzgm7oQhgEydFP3PkhPtoBjmXLljVZnbjRbhNY8Ie6SAfuEBc4+D3LBVmi1rrVF\/wQ4hCrD9Yz3VCOhGeLZC2xpQpgXloIxCmQSqrKD2s5EDW6vfPOO8YKdRIZ4fsrEALjO7wMWIC8YLyQyZRHevI57wkEb182Z1mE8zLbl5IXkHEkLFFnOs73R7dYulBemIRhFTAHIyceRdGRHzssR+d9gEAhUvtjTnfUPktFqSvVebOW2AATUOl2rFq1yuBiu3N0sypVqmTCeEEQLujO8IvPr13NmjXNIDTWnddLQPpExW2WO18gxmkSKQcSIB0PJkfGVnhQeWC5RrAKeaBoL+0mbH8FKxcymz9\/vrGi7C93orjGqhesEbpz\/EDwklmLzebhmnBwomtjw3lhuDc23obbo8XIXnPfIDHIzHaZqDdWfvRCSBNLN1t2PF1sOnu0utF1pF2IbRvWsU0X62jz2\/vPc8P950fC2RX1ys94GN1bfqSvL+hCJ\/N8WH3BBbHlow\/vCl1SG+Y8Yj1j2fOuYQk748JwnrXEBpgdO3Y04wDubov7QeLB5yGB+ACdePJzU3gJ+LUnHuFF4YUhXaLCSwAx8CCgi+0KUC6SSDl0S\/mFpOuFHhy5ZvbJko3VlTqoK165PKi0nZfMndbqTBoeUOonDbgkiivprVAOdZUrV85099GfdhBm0\/BjQ9lcE25x4toK94Zz4slvhRcdHMCEeARduYfUTVmkpV7SkJY0CPGU56cb6axQBj8sfrrYtPbIfUaoC10QzrH4OLfpYh1JQ1qeUdqBvqSlO2u7olx7CW3lR97mTeT5gASpDx2p29Zny+d5AEewQB8r4EIajATq5EeC67BJ1hIbQPLrxA3i3Aq\/3Pwa22tu3t13320vzQSDjacb48zPjXzkkUfMzGA0Q4Inbl2o98EHHzSD3IkUwZgKSx+cabkmnBeYGS0b1717d+Elstfuo7Ug3eHua\/vwYrlhwdl4d1sId+NKGOKuq0GDBsIaJ+LAk64LumJhYkkw08Y18WDkvDeEcW+oi3MrvDyE2xfYWhnEM\/Nny+MasbglqxvlozNlINQZSxfincIPBfebNtlwzpnA4P7ZsFhH0vDsOeu37Y6Vx4aT1\/5gEIbOPNf8EGIZE+YW8vg9Uzx3jJE69aFMng1rIZKfctxlh+E6LcSGNXHllVcKv7Li8W\/KlClmYNj+CuyPlWSLhZycYwA8jDbOHnnRbBpujA3n6MzPYC03lHDKIY9NTzjxvKg8xKRxi7MsBnLtTXeXRTrKokxnGTYd9SJc23jOCUNY\/oAetg7KoTzKJT36EU9ajlwT7habzysNZZHfCvWTn7JIb+u216QjnGubF50gHcI5z83NNevqsBJ4+ShjwYIFFCt0fcxJwR\/qojwr9h6AJy+w7ZIXJDU\/HJRv03IkvzuONH66gQWCnpRh66Qsrq3YcMp3C\/rRJpuWc8JIR93o4Awj3CnO+inD1uWV12KMfpRBWvIghDnjuSacNKRFeDex0rg\/kN8555wj\/Pgw08kPAmnc+lCmM5xyuQ6jpJzY\/ve\/\/wnLBWL1zQFh7dq1wvolwEb8bjbpVYoHArygdNlpDS8VP2x0n7AKsJYIjydYmXSdyRcvrcbHRoDuJxYlPzKc2+4rY5zcp9g5syMm5cQG68+dO1datGgRE4ENGzYI4y0xEzgiIEHAT1Y+\/\/xz6dGjhwwdOtSscUs2v6afYxa+phqHE0880SwFwXqxMnbsWGGAPJG6SEd6ykkkvabxvo8YFLwb9h7Y4yWXXJLS+87763idM3aaUmJjkJpxAn6NYxEXDyZrleif16lTx4wVTZ06VbD03K0GlL59+5qtHswOqXRSLDopBsm+B0Gm5\/3lPXa\/2+m+Timx0QXFvG3atGlMvXft2iXM8vTv31+WLl1qLKrhw4fLJ598sk8eAMH6I97+omTTsWfPnqZNqv\/Efay0TNxHxT8Y3O29BX\/eX95j8yJk8I8nsbG5uUOHDmbrCmMi8QR9ly1bJrNmzTKDw5FIhCBPqVChgmDV0VUtWbKkqaNx48bCejLPDAWBDGgyyJltcvLJJxdoL2ZvY7bpjr6qfzPTowCLIKS44G9eggz\/8SQ2uoU\/\/fSTdOvWTfLy8nyFNOi8evVqmT17tnAz7KAwXVLnAlPSLVmyxOynpHyurTCAbM+Ly7Fq1arCOjSO2dgm9Fb9g7tz2Y5\/cMiJ9zcPIpGI2Vh85plnSps2bXyFNDSAqV8GJK2w5oUpfTtFTBqEGRe2aTCo++uvvwprklgXw0JL4ouT8GCy7IVjNrYLvVX\/4O5ctuMfHHIxiK106dLSqlUrKVWqVFzd8PAQLxEr+VmrxtoZxuCYOMDpHpMHjLX169dP6tatG68YjVcEFAFFICEEPLuidBMfe+wxYRKAX2wI6eeff06oQJsISw0rjmsWKbJWzV4zXoHHA6w7uq+QKOlUFAFFQBFIBQKexIYVxhKM119\/3XRJcdT4hz\/8Qe666y5hWwxjcKmoXMsICAGtVhEo5gh4EhttZsayfv36MmjQIMEDxOOPPy7Mlp533nnGV9irr74qyVpxlKuiCCgCikC6EYhJbM6K\/+\/\/\/s+4jR45cqRMmzZN2NXPht\/vvvvOmUzPFQFFQBEIBQIJERuWGjOcrD3DYsMTBMR26KGHhqIRqoQiEH4EVMNMIhCT2LDG+KoNu\/7ZeIylxuzlxx9\/LEwMNGrUSOiuZlJZrUsRUAQUgUQQ8CQ2lme0a9dOHn74YeE4Y8YMYUzt\/PPPN25iEilY0ygCioAiEBQCnsTGLgCWe+B6iJ0FbGkKSkGtVxFQBBSBZBHYS2yFs+EoEs+k33zzTeEIjyusO49gDVIEFAFFIDAEPIkNbXDmd++99wrjan5CGtKrKAKKgCIQFgQ8iY2uKH6U2AfKxIGfkCYsjVE9FAFFQBEAAU9i+\/3vf2\/2isbbAG\/jKUglaQQ0gyKgCKQJAU9iS1NdWqwioAgoAhlBQIktIzBrJYqAIpBJBJTYMom21hU4AqpA5hB4ZtUhsr35rfLcF79krtK9NSmx7QVCD4qAIpBaBD5cuVV+qXSM5O8sm9qCEygtLrGxpg23RfXq1ROcRU6fPl1uu+024+kjgfI1iSKgCBygCKz+dmdgLfclNvyuPfDAA8Y90TPPPCPVqlUzX+zmS9H333+\/\/PJL5k3MwJDSihUBRWD\/EAggly+xbdmyRRYvXiysaTvqqKOkRIkSkpOTI1dddZVxOMlG+QB01ioVAUUgixA4smJOxrX1JbZY2vAtBHy0xYrXcEVAEVAEgkTAl9gqVaokDRo0kHvuuUf4PgFfldq0aZOMGTPGuAznG6FBKq91KwKKgCLghYAvsUUiEWHiAO8eV1xxhfmoMW6M6ILSPcVy8yo0q8JUWUVAEUg5AkFOHNAYX2IjwUEHHSR33nmnGWubO3euOeKnTa010FFRBBSBMCLgS2zbtm2ToUOHGg8fkBszpAMHDjTXePzAw+66devC2C7VSRFQBEKCQCgnD5gZfeONNwSSAyeu+TQfxzlz5kj79u0J3i9hjZz9bul+FaCZshQBVVsRp6XXMQAAEABJREFUSC8CvhYbM5+7du0SPuTyyCOPyODBg8352LFjBQ8gw4YNM+7D90dF1shNmjRJ8NK7P\/k1jyKgCIQXgVCPsWFR8YWq3NzcQgjWrl3b7Dwg3h1XKKHPxRdffCGM2fHlK59kJop0+fn5snPnzqyTn376SX788ces09tirfoH+8xlK\/7OxfunHV3BvMeZ\/ONrseFwkskDupxYWCjGEaLhHKttxYoVnCYl27dvFyzArl27CktK4mV+6KGHZMKECbJhw4asko0bN5ofAJbIZJvu6Kv6B\/u8pRj\/jL47b326Jt5rndZ4X2KDuHr06CEQy8knnywtW7aUU045RW6\/\/XZh+QdWFMs+ktWQLmiNGjWkadOmCWUdPny4dO7c2WzpYltXtkiVKlXksMMOk6pVq2ad7mCs+lcL9L5lM\/4LN+3ZbllixxYptWVpQu95KhP5EhsVHXfccfLWW28JX63Ky8sTSGb27Nly3nnnSb169eSdd94hWcKybNkymTVrlmCtRSKRhPKxjq5mzZpmO1dOTk7WHPlhKF26dNbo68ZW9Q\/2WctW\/DfvEJn79XbzbpfYEf+DUCZhiv\/EJTa6noyl8QvepEkTOeaYY2TNmjXSqVMn4YMvZcuWTUql1atXC8SIBVirVi155ZVXDMn16dMnqXI0sSKgCIQTAdwVWc2CsNaoOy6xjR8\/XiC0Zs2aiZWLL75YsKAYf6OQZOSss84y27PYooVQ1rhx4+S+++5LphhNqwgoAiFEgNnQ7pOWGM1KFHRDc5a+bs4z\/ceX2L7\/\/nuZMWOGTJw4UV544QUzrrZ8+XK58cYb5dhjjzVLPjKtsNanCCgC4UQAUrvo0QVR5VpU+zV6nukTX2JjqhmFWNJRvnx54QPKLF246KKLZObMmQLxEV8UwVLDiitKGZpXEVAEgkXAkhpHNGG3Qd65NTkNRHyJjcFLNrqzpANiY6wNQVNIzRIf1yrZh4BqrAgUFQGIbOjbq+TEe\/4lnFMepPbp7adwGpj4Ehvr2Dp27GgcTa5fv16OPvpoadu2rZGqVatKxYoVA1NcK1YEFIHgEIDELKENfTs\/qghWWtCkhjK+xEYClnXQ7WRMjTVrI0aMkJEjR8q9994rWHOkUVEEFIHijwBkNmneRrlw1AJjoTkJDSsNUss7t\/AupaBQ8SU2Nr6zT3THjh2GxOiasgXqhBNOkBdffFHYQRCU4lqvIpCVCGSZ0m4yY8bTuZzDEhpWWlhIDYg9iY0JAmZD33zzTXnppZeEIx4+rLz66qsmHMKjEBVFQBHIfgQgMQSrDAKr2GuGscw4d5IZLT3t6PIyqmN9CRuhoRviSWxYYnjwYD\/n119\/bfZ14pfNCn7YLr\/8crNdiEJUFAFFIPsQcJKY7V4yCQCRQW7OFmGZQWZ5BTOdkNnU7o2kY5OqziShOvckNvY3Pv\/888ZS6927tzl+8MEHYoVtVJdddplEIoltiQpVi1UZReAAQwACm71iq0BWkBYk5rTGCHdbZEAEmUFe1jKDzPIKxtAIJz7MUpjY9mrKR1twV4Trmnbt2hmXO5s3bxanEE+6vVn0oAgoAgEiAHkhkBTiJjAWzhJGXCwSwyKzRPbt\/a1MNxNSIyzApu1X1Z7ExkLcSy65JLqFym6lch6JJ91+1aqZFAFFIGkEIC4EckIgKqwvuo9WCEOI9yIwKsXicpLY690aGRLDIstWIqNdTvEktsqVK5udBezljCUsASGdszA9VwQUgf1HANJCbLex96urZMB7W6T92M\/Edh0hMIgLseRFHq9avQjMWmJOEmteu7xX9qwO8yQ2Z4vw7rFgwQLzpSr2iLJ+DbJzptHzoiCgeQ8EBCAfZPbesS5ICXLC4kKcxOXsNk5dsj3qAsgLJ8gLobuIYHFhgR0oBOaFCWFxiQ1fbN27dxdc\/TZq1Mh45mDcbdq0aeRXUQQOeAQgLMRNWpa4sLKsWNIiDnKju4j4gQhx2a5jXsGspJu8mKUkDIHciqMF5oePV5wvsbEvFG+3fHZv0KBB0qVLF\/MxF5xOEk68V6EapggUBwQgK8SPsLwsLUtalrgoIxYekBbiRVxz+jSS+TfVFI6265hXMCup5BULzd\/CfYmNhbpYarmuj7lwTTjxvxWlZ4pAdiCw+tudZsN2LMKK1TVMhrAsEk7SgpCwqhC6i1haVryIi7y2HD0mh4AvsZUpU0b4BN+SJUsKlbpq1aroFqtCEXqhCASEAFYR4iSrEf9cI3bwHbKiO5iIhRWva0gTIR3EWlpepOUe54LQSIfQXSQ\/ZamkHgFfYsND7tVXXy0s0sUVeL9+\/Ywb72uvvVbw+oH3j9SrpCUqAnsQgKgQJ1nhUQLLCaJC\/MhqxD\/Xih18h6woa0\/Jsf9CNkgihIW1ZS0tJa3YmAYR40tsKHT66aebnQetWrXiUurXry+vvfaa+ZiLCdA\/ikCCCEAsCESFMAaFQFQIRIX4kRUeJcgDUSGUl0j1kBViCStv7yA8hGS7hdbCUsJKBNHk02Qyhy+xfffddzJmzBj54Ycf5JprrpHBgwfLLbfcIoyxZVJJrSucCEAqCCSFQDgIJIVAUogXUblnB8kHUSGUmUiLISrEkhVdPIgKgaxevLqeGXxfP\/gUswDVSVh5ewfhyaPdwkTQzq40vsRWsmRJs7wDn2y472Y2FLLLriaqtokiAKEglqQmL\/iP6cr1nLxcgiKqPJdlBWFhWSEQFTK1eyPjaQJCg6gQyKp57QqJNl3TFTMEfImtXLlyMmzYMPnss8+MY0mOdE0vvfRSmT59uuhe0fA+DRCUFUtUWEV2jCoeURHfc\/IKM\/g+ecEWs4Eaawqh3ERajjVlJZZVBVFBTrGIKs9lWTUvhqvkE8FS0ySHgC+x2aKYGeUTfHwRfvLkyWamdODAgebjLjZNth\/Drj9kgiRLUnQDnd0+O0YFyUFSCOUm0n4nSTmJKm+vVQVJIW6i4npqTKuqvFBuIvVrGkUgUQTiEhtWGcs9+vfvL02bNhX8sB133HGCTzbcGyVakab7DQGIBEmUpGItUSgKSaENhII4Sapjk6rRbh1jVOMurmoWiLotKidR5e21qrCmEMqkfBVFICgEfIkN10StW7eW9u3bC1+kwk34nDlzhGUf1atX38cf2+effy6kr1WrljRv3tx8k9SrYVOmTBHSWKF7i0skr7TZEAZJIW6icm5ijjWITpcPSRVJQSpOosrba00x\/oQ1hWBBuYmKeCuQG9K8YIzqD0fkqEWVDQ+h6lgIAV9i4xsHzIQytsb3P9kryoRCoRL2XrC9irTdunWTZcuWSV5enuBxd8OGDXtT\/HZYu3atjBo1ykxMsKE+7J5CnKRlx6ggKiSeNZXsOipQgpysJEtSkJaXNbWHqMpL84IxKsqmnsBFFVAE0oSAL7GxQJexNcbY4tW\/Zs0a2b17t7HU+HrVySefbMbitmzZsk9WyK5SpUr7hAcdAIEhjD9hRTmJy45TWcuKsSkkEZ0hEkRJKhG0NI0iUHQEfIktmeIbNGgguBPnW6O4Opo7d67ZdsX3R53lsCaOb5TecccdUqdOHePMcurUqUIeZzrnOWXl5+cbT7549U2lfLV5u5nxg8QYaEcgNcjNj7gOL1dKELpq7RuWF+S+djUFYWwKef\/G+vJR12oyo3s9M041uUt9GdE+10jP06uZPORrXD1HkMplJC1tLApeDEGwJ7goZQSZV\/XfGcgzxfu6bt0652uc0fOUEZvVmm7riSeeaLZhMXZWvnx5G2WOu3btkrJlywqTEUuXLjXd1eHDh8snn3xi4r3+MBs7YcIEwdJLhcz\/co2Mm77cfB+x8b3zzBotLxKzxHVh\/YNkwFmVjDCYjseFqZ2rC8L17WeUF6RVDRHk6LI7Bfndrq3COOWmTZtSpnsq2p9oGRs3blT9C4ZSEsUr1enSg\/+GjDyLvK99+\/b1ep0zEuZLbPxSM1nA0akN1\/Pnz5eff\/7ZGWzOGzZsKAsXLhT8uFkxEXv\/VKhQwXz1qkWLFsJ4XcuWLaVx48Yyb968vSn2PUB8nTt3lmrVqhVJfi5dQZ5etFOue3WjWZ81f93OaGV0FRHGohhE\/\/i2JoJM7d5Ixl7RULr+sY6RCxrnJqxDlSpVzJe8sFqLqnsQ+VX\/oj1vRb1n2Yw\/72vPnj2j71emTzyJjSUeWBp48RgxYoRwZNbSypdffmm2Vzl3IcyePVv69OljZk9pBDOeLAtxm6MsHWF2lS4C6az4bag\/4ogjpGbNmpKTk7PfsnmHyJ+fWCIPzdwo67f9Yqp1EhmD7gikBrnVqnzQftdl9WTypXTp0kUux5aX6aPqv\/\/PWyruVTbjz\/vKOLt50QL440lsfKSFj7VccMEFgmXGsVmzZmKFnQdYZlhfVmfWtGGpLVq0yOxIYOnH4sWLhckHm4YjExJsoscShED5pB\/dV3c60qZCmAxgJpOxM84pE0JjGQRLHyyREa6iCCgCxQMBT2LjIy0zZ84UBu0hNY4sy7CyfPlyGTBggJn1tDDUrVtXevToITfddJOZFGDT\/A033CAnnXSS+Wwf423vvfee1KhRQ5g4GDJkiEnHWBvr4sgvYktLzREig9SYybQlWkLLOzdX12dZUPSoCBQzBDyJDUvq22+\/FQb+ISFmLG031B7pqpLO4hGJROTCCy8ULDEIkCPXkUhELFGykZ70WH6Mv5GOLqx1iURcqgRSs7OblGmtNCU00FBRBIo3Ap7ERleUL1IxlkaXFCJyC+GkCyM8ltTsTCekRrcTUgujvqqTIqAIpBYBT2LDwpo4caIw+E+XFMvKLYSTLrXqpKa0SfM2iJvUILfUlF7sStEGKQLFDgFPYsvmVrKw1o6pQWZYahyzuU2quyKgCCSHgC+xMZ7GoD9LN9xCOPHJVZfe1LYLSi2Q2SMd6usEAWCoKAIHGAK+xEZXky6n7YauXLlSPvroI7nuuuuM+yKWeIQJLyYLrD6sRWteu7y91KMiECoEVJn0IuBLbO6qI5GIsIr++uuvFyYWnAt03WkzfT17xdZC42p55+ZmWgWtTxFQBEKCQFLEZnVmv+e2bdvMQlwbFvSR9WpWB7qg9lyPioAicOAh4EtsrFXr0KGDsJ\/TKeecc45Zm+bceRAkdE5rDddA2gUN8m5o3YpA8Ah4EptVi+1PjKfhNNIpTz\/9tAwcONC4JbJpgzw6rTXtggZ5J7RuRSAcCPgSG5twzzjjDDnhhBPM9w7atGkj7BGtXr268cwRhiaotRaGu6A6KALhQsCX2FB1\/PjxcvbZZwu+07h+\/\/33haUe06ZN4zJwYTGuVUKtNYuEHhWBAxsBX2Jjvygb19mFgP80oLryyivN1+EnTZokfOeAsKCEdWssyKV+1q1l9dgajVBRBBSBlCDgS2y\/\/PKL6XLiD81ZW+3atYU4HE46wzN9brdNUS\/r1jiqKAKKgCLgS2yHHHKIMIHw8ssvR5d24OkDzx184KVMmTKBIjjx33u+gLbvCEEAABAASURBVIW11rFJtUB10coVAUUgPAj4EhveX\/Gx9tJLLwnfMWDJB\/7VBg0aJL169TKkF1RT6IZai61GhRzdOhXUjcjqelX54oqAL7HRaDx84Dvt2WefFZZ8jB49WvB6y0wp8UGJJTXq79RUrTVwUFEEFIE9CPgSG5\/JY4EuC3UhMpZ74JcNS25P9uD+Fu6GVg1OEa1ZEVAEQoeAL7GxCb5+\/fry8ccf+373M9OtcndDM12\/1qcIKAL7jUBGMvoSG5vc+ShL7969hS\/OMMZmxVpyGdHSVQnEZoN0iYdFQo+KgCJgEfAlNmZEu3XrJg8++KDceeedZoyNcTaErVbE24Iyefxw5XfR6k47ukL0XE8UAUVAEQABX2JjSxUfWmFszS2EE08hmRa2UVEnyzzUYgMJFUVAEXAi4ElsTBawzIOPG9PltN1P55Fw0jkLy8Q53VA7I8oyjxTXqcUpAopAMUDAk9joYnbq1Mk4laTLSdfTLYSTLtMYQGy2TrXWLBJ6VAQUAScCnsRGF5NlHRUqVBAv7x6nnnqqFKUryoRE69athe8oNG\/eXGbMmOHUyfdcx9d84dFIRUARKEDAk9gKwqP\/U+3dg43zgwcPFiYlli1bZiYkhg4dKhs27NkeFa04xomOr8UAJouDVXVFINUI+BJbOrx7rFmzRnbv3i1YaqVKlTLLSNh3umXLlphtmzt3ruTn58tXm7dHv2tQ7eBSsnPnztDLTz\/9JDgLyAZdvXRU\/YN9xrIVf97XdevWxXyn0x3hS2x48ChZsqSk0rtHgwYN5Pnnn5eKFSsKG+ohLQiOj8TEauxDDz0kEyZMkE+WrYkmOaFKKWPlYemFVTZu3ChMsGzatCn0unphqPpvCPS+ZTP+vK99+\/aNvq+ZPvEltnR69\/jss8+EjfUs\/sVxZfny5WO2ffjw4dK5c2f58vtS0TStT6wh1apVC7VUqVJF+EQhpB12Xb30U\/2Dfb7Sin+a3x3e1549e0bf10yf+BIbe0JZ9pEO7x64GF+4cKGwwd5KrMZjMdasWVPm5G+PJjmjXiXJyckJtTAJA4Zh1zOWfqp\/sM9XNuPP+8pupegLm+ETX2JDl1R795g9e7b06dNHGDugfGZGqSOZ\/jgLc8mroggoAoqAFwJxiY1MDO6nyrsHXTMstUWLFhnnlSz9WLx4sTRp0oSqYgrr13Rhbkx4NEIRUAQcCCREbI70RT6tW7eu0L296aabpE6dOnLNNdfIDTfcIDiwjBbucQKx2WBdmGuR0KMioAh4IZBxYotEInLhhRcK7sW\/+uorc+Q6Eol46RcN04W5USj0RBFQBOIgkHFii6NPQtE6xpYQTJpIEThgEYhLbNu2bZMXXnhB+vXrV0jYLUBcppBz7jhQYksH6lqmIlB8EPAlNmYub731Vhk3bpzZLRBks3XiIEj0tW5FILsQ8CW277\/\/XvCi++STT8q9994r7PG0grePcuXKZaS1a77bGa1HrbUoFHqiCCgCMRDwJTYWCB566KExsmYu2DkjqsSWOdy1prQjoBWkCQFfYitTpoxxT8SWpuXLl8vmzZujwh7IX3\/9NU1qFS7Wjq8Rqq7AQUFFEVAE\/BDwJbZvvvlGHnnkEXnjjTfk3HPPFXy0WbnkkkuEeL\/C0xGnFls6UNUyFYHihYAvsfH5vZkzZwrrzdxCOPGZgMNOHEBqSCbq1DoUAUUgexHwIzbTKlwL4YmDyQOWeNAl5VsIhJsEGfizu0zw43wZaKZWoQgoAilCIC6x4XmjS5cusnLlSsFKw\/kkW6ImTpyYIhX8i9ldppIgpKpRIYeDiiKgCCgCvgj4EhtuvCdNmiQPP\/ywDBo0SPCZlpuba5Z+MO7GchDf0lMcqXtEUwyoFqcIFFMEfIkNl9Z40YXMnO2vUaOGuWQBrzlJ4x9nN\/TIir9PY03hKFq1UAQUgaIj4EtsBx98sPCJPTasO5d2rFixQlgKghRdBf8Sfql0TDSBdkWjUOiJIqAI+CDgS2ws0O3evbvZcdC2bVthEuGyyy4zboeuuOIKQ3o+Zac86siKOSkvUwtUBBSB4oeAL7HRXBxMvvPOO3LbbbdJ+\/btjf+0N998U\/hOAfHpll8q1YtWocQWhUJPUomAllXsEIhLbLSYPaFt2rQRnEN26NBBMrV+jbqtKKlZJPSoCCgC8RCIS2zr168XyIzP5vFxhnr16hmCY0tVvMJTEW\/H2HR8LRVoahmKwIGBgC+xMet51113yeGHH2483bKWbdasWcI3EO655x5hxjSdMNn1a9ShFhsoqCgCxQWB9LbDl9hYp7Z161bzVSk+whKJRKRq1apy8803y8aNG4XFuulV77fSldh+w0LPFAFFwB8BX2JjVpRFuVhu7mJwZ0S8OzyV14UtNl3DlkpstSxFoDgj4EtsrFPr2LGj3HHHHcJMKONtbKvCq27Lli2FBby4MkoXQM4PuOgYW7pQ1nIVgeKHgC+x0dUcPXq0rF69WtgE36FgRhSSg+AeffRRwXURkglYMtsVzUSLtA5FQBFIFwK+xMa42vPPPy8ffPCBr1jl1q5da2ZQjz76aDn++OPl2WefFS8vIFOmTJFatWpFhTVxXpafes61yOpREVAEkkHAl9go6Oeff5ann35azjrrLGG5B98AZQO8c4sV6eiWMoMKSS1btkwgxAkTJsjcuXOJLiQQ4KhRo6J+3mbOnOm5Ns4Sm1prheDTC0VAEYiDQFxie+qpp+SVV16RXr16yfjx403387777pMnnniiUNF4AmEWlYW8JUuWFNa9sWvh008\/LZSOiw0bNkilSpU49RXnR1x8E2pkcUBA26AIpAwBX2JjjG369OmCg8nzzz9fGjZsKJ07d5YxY8YI4cRbTSpUqCCMxx1xxBEmiK9bse6N5SEmYO+fH374QRijY6yuTp06xt341KlTPbusq7\/d83WqEju2SH5+vuzcuTPrhBllrNls1B2dVf9gn7lsxZ\/3dd26dXvf+swfSvhVyQLcSCQikJYzHdeRSKTQAl0W7VasWFGw1mhQz549Be8gZ555pjOr7Nq1S8qWLSv9+\/eXpUuXGtLkYzGffPJJoXSW1Ahcv2yh0K3F0ssmYa0fOzQ2bdok2aS31VX13xDofctm\/Hlf+\/bty+sbiPgS2yGHHGJIbdq0aWLH1JgMwI0R7owgLqfWjMc99thjctFFF8mxxx4rY8eONeTmTAMp8oGYFi1aGBJk2Ujjxo1l3rx5zmSFzi897wxjKVarVk2ySWgEy2Q4ZpPeVlf0Vv2De+Yygb+916k+0rPDuKENQYgvsZUuXdrsMsCLLuQDCTGBgDdd3Bk5F+hCeE8++aRZ7\/baa69JXl6eOONt4\/hewrhx4wQT24ZxdJOk02Jj8oBfL8brskm++OILeeGFF+Ttt9+WbNLb6qr6fxrofctm\/Hlfq1evzqsdiPgSGxoxDsYYGJMIkNXIkSPN0g8mBoi3wkznu+++K\/fff7\/4NQhLD+LD6sMKZCkJft6aNGliizJHyOxvLQ6W0w\/ZJKPu6CadOnXKOrGm+EMPPZR1uoO36h\/sM1cc8McQ8uMD87Kn4U9cYqNOxs8gMmY8+a4olhzhToGh+YVhTM25Ro2JBtaosQzkvffeE9yKM3EwZMgQgTQZa+vXr5\/UrVvXWZxAbH3bN5aHuzSX58ePFD4eozJRcZioGGTTe8D4eWiJrRDjxLjA4lq8eHF0bZr9Dun1119v1qixVo21cGSHHPn6FWlmz55tvjZPuFu4BhTSqzQzM8iKg+KQTc8A7y\/vcaYlIYst00ppfYqAIqAIFAUBJbaioKd5FQFFIJQIKLGF8rYEqJRWrQgUAwSU2IrBTdQmKAKKQGEElNgK46FXioAiUAwQCCWxsdgXl0e4PuLjMXgN2b59eyjhTlRX1uzhJYU24dYJ33bsmQ26UYnq79STnSisLA\/DPUlG\/48++kiaN29u3GW1bt1aPv\/8c2ezAjnfo\/+zxs2X37POfmP2bJOGZVKXX3652XMdiNIJVopTjCuvvFJYp5pglpQlCyWxsW+UFfss5P33v\/9tHF3iYSRlrU5hQYnqykv14osvCm3ihWJ5DITt3oGRQtUSKipR\/W1h7ANmETbb52xYkMdE9V+zZo3Zl\/zAAw8IbrUuuOACYfsf+6GzQX\/WgC5YsED+9a9\/yaJFi8y3Rx5++OEgVY9ZNz\/iLMD\/+9\/\/LqxtjZkwjRGhJLYZM2YIC3pzc3OlXLlyxnnlP\/\/5T8EzSBqx2K+iE9WVG3zGGWcIbWKr2TnnnCOQBL9q+1VxijIlqj\/VQQJshzv11FP32QNMfBCSqP7scGnUqJGwNbBUqVJy3XXXyYABA8x+5SD0tnUmqr9Nzz2w51WqVLGnoTrynrJuFYcYPOtBKBc6YuPG4fKkdu3aUTy4gbhIArBoYAhOktGVLUosVrZqL1++XHjBvHZx2DTpPiajP7q8\/\/77QvcTJwdcBy3J6I+VjBum8847TxgKuOKKK8wPZSQSCawZyejP4nY8WrNFCQcTLIa\/7LLLAtPdr2KMEbZf3nbbbcaJhl\/adMUlQGzpqtq7XB4+SIw9pd4pwhOajK7cbNqEmU43e\/DgwdK1a1fBg0pQLUpGf8YD2S+MpfO73\/0uKJUL1ZuM\/nSd8SBDVxR3WaeddpowFMDYVaFCM3iRjP4MxbA1cdasWaYrylgt3yGBHDOoctZUFTpiY18qX8fiQXSiiHUTiQT36+rUxZ4nq+uqVauMB2K8oDz++OOC9WDLCuKYqP4McKMz+rr39Aaht60zUf1t+vbt20v9+vWFfOxpxk8enp9tfKaP6JHIs844LF1WLGUcufID2bFjR\/n6668Fh66Z1jsb6gsdsdE1Y6M8Y1IWQB5AzHCsHhsWhmMyujKedtNNNxlfdXwzAqcCQbchUf0ZAuB+4LwABwdt27YVPLlg9QQx42VxS1R\/0uPJGQuJcyslSpSQSCS4H8tE9WesKicnx6odPUKMxEUD9CSKQInoWYhOmLFi8BELh18klknwERkehBCpaVRJVNfJkyebzf5XXXVVIgPWpuxM\/ElEfywEPErgtABhZvfss8+WDz\/80LiLz4SesepIRH\/ytmrVyrizZ2yK7udLL70kTCYk8u0N8qdLEtGf3sopp5wir7\/+uplwQn8+loSrfhy3pku3bC43lMR20kknCQOjWAbc0OOOO05YdxRGoP107dOnjyCMg6xYsUL4MhcD11g9CDO\/jJsE2a5E9A9Sv3h1J6o\/6W644Qa5+uqrzZqxb775RrCgI5HgLDbahl6xnnWeHYR0f\/7zn82X4hgOYHyN7uktt9wSqMWJXmGVUBJbJBIRFiCyXoeB3ltvvdWMi4QRxEgktq58zQvhFxd36Fg7TsEqrVy5cqDNikTi6+9WEEsBt+9Ycu64TF9HIonpH4lEBKuf9VXMSOP8MwzWTiSSmP50O6+99lozcYD+rCV0e53ONPbx6uPZxkktz0u8tKmODyWxpbpxbqgsAAAGu0lEQVSRWp4ikAgCmqb4IKDEVnzupbZEEVAE9iKgxLYXCD0oAopA8UFAia343EttiSKQfQikSWMltjQBq8UqAopAcAgosQWHfdI188UvO\/0fL3MyaeOV5YxnHRiuf9j7yr5RZ5zfOXqjk1+aTMfhMYNlNyyWTtdCY8qlfITzTLfxQK1Pie1AvfP72W72jB555JGCl49ML\/dgaxd7bfdTdc9sF198sSxcuDBtC41Z6sBC5mbNmnnWr4HpQUCJzQ\/XAOLYu9irVy+xDgU5JwzrYtiwYcJmaGstsUYI6wmrA\/9u+Bdjj61X2iVLlgh7JVkgzEtGXojCq4ms9WJBNOU601IuG\/eJ5+i22CiPcslDPeeff34hZ46QIg42iaN8PG5QPzqjO22gTtpEOZSHldOmTRuh7YRjMYIHuOBwkboefPBB44bI6sNGcfaCUhb1oC\/1+Al5wRWr0uJ06aWXCrtfvPK5LVCwIT\/leKXXsMwioMSWWbzj1sa2GV4OnAqyQJn9jGyYx20NC5WxMNjexMs9fvx4wVsFi345x2sIizfdafH51rt3b\/nrX\/9qnCw+8cQTMnLkSJk9e\/Y++uCEkT2hf\/vb32TlypXiTEu5WGqQCUe3xUZ5lEseymFBLB4oaA8VYbnceeedRoc\/\/vGPAiGxgp5F2G+++abgiJM6+\/fvLxDMli1byCZr164V\/HrhNoldKLQZDyNghDff+fPnC3iQGCIcMGCAUC9Y0A7agz7Ex5OXX35ZWPyKTi1btjQ7R2zZ8fJqfHgQUGILz72IasLLTfeIHQvsXLj55pujcfaEPY543GBLDuSAdQMJenXVIARWgbO\/k03TDRo0MC6y586da4uLHrF2jjnmGGGDeyQSEdKee+65AvFEE8U4IQ1pyUM9WGe4OeKcLGwHwrsG1+i9Y8cOwVrDYwjkXbNmTeMjjVX2u3fvFtpEvsMPP1zYXkc4ZIfXXLxbQKzsHsBNOW0nLRZtixYtjENJ6oGEjzrqKMHSIz6e\/OlPfzLOQKmLOtExPz8\/XjaNDxkCSmwhuyFYZN27dze+wrBO6A7ZLptTVciMbVps5MaSwsrgJXSmsedYHHiQxUEh3TNkwoQJsnHjRpskeqTLSr2Qqg2EQCAge+11xCrDjQ5pbTykA8lgbRHmjOPayrZt2wRLjgH2tm3bylNPPWUIz8aji\/Vugc7ogrcXG48j0rJly5pL2orTBLq7tBMinT59umdbTQbXH6eDU+qkblcSvcwCBJTYQnaT6AKx8R\/rB0JjjIjxJVwHOVXFwwZdsZkzZwpWFpYdROJMY8+xPhjvojtGtxX59NNPjWtsm8YesZqwGK21xJHrqlWr2iSeR0igYsWKwoZ\/m4AuJHsyIS4b5nV89NFHTTBdSlzA433VkqGJcPw59NBDBets69at0VDqgdAIoK3dunUT2oig+8cff2y64cTHE1xk2TS4OcICxvKzYbGOznyx0mh45hBQYssc1gnVBFFBUlhAZMAKwxkhlgMvrbOLRjwvHmmfe+45Y5UQT7gzLVbTl19+adwMQVS88Ax0Q4ykdQqkShcVMiAt3T6uW7Vq5Uy2zzn60Q39xz\/+YQbcca3DWBt1of8+GVwB1AUp4qYKaxIy5NqVzLiarlatmjzzzDNCHfi5I71NB4G\/\/fbbputJmbTjL3\/5i3HKaNP4HenKUiZljx49WqpXr266pu48bEDHeqQOSJXJDncavQ4OASW24LD3rLlLly5mbIkuJjOjDLjffvvtgi86SIdrXlRm\/XJzc4UwuqKc84GYHj16CK6QCLdpGaO6++67ZciQIcbfP6TG2BezjG4lGPvi60K49KE717NnT+nbt68Q7k7rvsa6ZIyKsSlc62BJYX1Beu60zmu8VjB2RheYsTPKwXJk5hGCcabFkhtQMDnAbCXpcfnDmKBNQ5vAAPdE6M\/EAd1cxvFsGr8j+bkHlE0d1EWdYIqbKWY\/yX\/NNdcIVi\/fIHDr4E5L+pBLsVNPiS1ktxRLgPEyZvToRuFQEKsBNRmUx3piVpSuoU3HcoZ27drJoEGDBIuPiQJnWsa2GER\/6623TBeN9MxYRiL7+iKLRCLG7xdpsLaY6XSmhUSpnzLRySlYiZAUs7noz8wppEoarFDnx2yc5ZCGdtJedCQOK4x6mjZtKlhDtIlysJDo9hJPevSjbViF1B+JRKLuidCf8ognbyLCujPyUDY6oRv5qB9s0Y1rXHRPmTJF+DwkwwYDBw4U9AUXd1rSq2QWASW2zOKttRURgV27dgnLXlj3RjecriuEwtIMrNoiFq\/ZiwkC\/w8AAP\/\/RWDrkAAAAAZJREFUAwChLzVJ3ZUvUQAAAABJRU5ErkJggg==","height":187,"width":310}}
%---
%[output:920312f6]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATYAAAC7CAYAAADv5bIbAAAQAElEQVR4AeydC7xNZfrHn43\/OEMJyaUoRwil0iAVynRTKjQ1YSpNSkVRLp2GmqgQ0kXJJV10odJFmdJtSNQwkqiQS07ujEpGksr8z\/fl3a2zrL323s7ee619PD6es9Z6r8\/7W2v99vPenlXif\/pPEVAEFIFihkAJ0X+KgCKgCBQzBJTYitkN1eYoAoqAiBJbAE+BVqkIKALpRUCJLb34aumKgCIQAAJKbAGArlUqAopAehFQYksvvqb09957T2rVqiVjxowx16n+89lnn8kJJ5wgffr0SXXR+5S3fft26dSpk5x++umyefPmfeIzEQCOtJd2J1xfCBKiL3ojnCeqUhgwd+qK7rSB59oZHqZzJbYw3Y0U6QLhQDwQEC9Fior1LCaTdaEAL9Xo0aNlxIgR0rBhQ4JiSrK68aKm6weI+zBkyBApX768TJkyRapUqRJT77BHgPvZZ58tvXv3Fu5HGPXNemLDSuFhtMLDCdA8SLzYCA+SjefXnngrzvyQAS8DcZRDHl4gyuCcI+US7yWUTTqEX7RYN506qeu5554zlhx1UR7pyUd+hPKc4bZ+dOCcMqy+pEO4vvTSS2XNmjUyZ84c6dq1q5CeOCvU5yyfeHd56EgaK+Sx+e3xP\/\/5j3jVhQ7oZvNSls3DkWviaCv4cm7bSjx1EWaFa8KRCRMmGHI4\/vjjuTRtQ3eblnqpH0lGN+oAKwodNmxY1Lom3JbNkWvSeAl1Uj\/pEM4JA1\/K5n5wX9q1ayebNm3apwjSkoe8CDi5E7377rvmmSHeiRl1OHHgnDDyozPped4o35brfN6uu+66fSxxZzz5nfV17tyZooX7YU5C9ieriQ2gX3nllUKQ8gBxI20gD1OvXr3spfDQ2nhusDM\/D93NN99sXhabYdSoUYYguKasZ599ltN9hDIp20b897\/\/lRtvvDFmd4267rjjDpvcpCM9+Wwg5VGuvU7VEVKoUaOGrFixwhS5Y8cOWbdunTRp0kQqV65sXmonLiRy40qYl\/zwww8ChrTPxlMW94prjlxzTlvBl3MrtJe67DVHaxnw4s+bN0+OOOIIKVOmDFEyYMCA6P0hgHqpnzZx7ZR4ujnTcu6nC\/FOgUSol\/ptOOeQ65YtW2xQzKNXfnACL5uJ8pzPDJYr5EO8GweeVcKIs0JeyuAaLJ3PG4RJHuIQyr388suFe8Q14nwesTixPrkflEX8vhJcSNYSG2BOmjRJDj74YHnttdfkq6++kltvvdUg+dZbb5kjf2LFk5+b0qxZM1m0aJERzrm5CHkRwogfN24cl\/LBBx8UIj4CeSifeOIJToV06HLxxRcbq4m8JsLjD\/qS9qyzzhIeYh468hFGOWRxtoXreAIxTZ48WSAudKecgw46qFA20kBitB8c0JG6a9eubQg2EVwp8LDDDhN3XZFIxJAkdVMuWKILuGGlcCQvehFPOq6t0F73PePlwjIg\/9atW6Vly5ZCm8B9\/fr1pq3UY8v74osvzAuZjG7ogU7owX25\/vrrxU8X0jmF+hHKQQ+Ec3DlB4SyuQYLSITunF9+nmlwAC8I2aalHFs2uIBJLBzs\/bV5qZ+89913n3ne0c2GUa5Nx\/HDDz80GBLufB55zqmPHxZ+YLgf6ECeMEnWEhtgAuqxxx4rubm5BlNIgQeHG2p\/JZ3xp512miFCXob8\/HwhPw8jFgzCuSnI8ce+RPxC8aA5oqKnWAdYPNRNOUTw8PBAQFpcu4Wy0MeG8\/Bz3rp1aw5CPvJTjglI8R\/qof3gSN1WH64Jd+LmhWssdSDNmTNnysSJEwXrlheHF4j0P\/74oyE9ixPkdPXVVxNlhBeGe8ML27ZtW9PlwkowkQV\/0I24glPzn\/yHH364+QGhHrCnvIULF3qOv\/npZgp0\/ImniyOpOQVDTuzzgm7oQhgEydFP3PkhPtoBjmXLljVZnbjRbhNY8Ie6SAfuEBc4+D3LBVmi1rrVF\/wQ4hCrD9Yz3VCOhGeLZC2xpQpgXloIxCmQSqrKD2s5EDW6vfPOO8YKdRIZ4fsrEALjO7wMWIC8YLyQyZRHevI57wkEb182Z1mE8zLbl5IXkHEkLFFnOs73R7dYulBemIRhFTAHIyceRdGRHzssR+d9gEAhUvtjTnfUPktFqSvVebOW2AATUOl2rFq1yuBiu3N0sypVqmTCeEEQLujO8IvPr13NmjXNIDTWnddLQPpExW2WO18gxmkSKQcSIB0PJkfGVnhQeWC5RrAKeaBoL+0mbH8FKxcymz9\/vrGi7C93orjGqhesEbpz\/EDwklmLzebhmnBwomtjw3lhuDc23obbo8XIXnPfIDHIzHaZqDdWfvRCSBNLN1t2PF1sOnu0utF1pF2IbRvWsU0X62jz2\/vPc8P950fC2RX1ys94GN1bfqSvL+hCJ\/N8WH3BBbHlow\/vCl1SG+Y8Yj1j2fOuYQk748JwnrXEBpgdO3Y04wDubov7QeLB5yGB+ACdePJzU3gJ+LUnHuFF4YUhXaLCSwAx8CCgi+0KUC6SSDl0S\/mFpOuFHhy5ZvbJko3VlTqoK165PKi0nZfMndbqTBoeUOonDbgkiivprVAOdZUrV85099GfdhBm0\/BjQ9lcE25x4toK94Zz4slvhRcdHMCEeARduYfUTVmkpV7SkJY0CPGU56cb6axQBj8sfrrYtPbIfUaoC10QzrH4OLfpYh1JQ1qeUdqBvqSlO2u7olx7CW3lR97mTeT5gASpDx2p29Zny+d5AEewQB8r4EIajATq5EeC67BJ1hIbQPLrxA3i3Aq\/3Pwa22tu3t13320vzQSDjacb48zPjXzkkUfMzGA0Q4Inbl2o98EHHzSD3IkUwZgKSx+cabkmnBeYGS0b1717d+Elstfuo7Ug3eHua\/vwYrlhwdl4d1sId+NKGOKuq0GDBsIaJ+LAk64LumJhYkkw08Y18WDkvDeEcW+oi3MrvDyE2xfYWhnEM\/Nny+MasbglqxvlozNlINQZSxfincIPBfebNtlwzpnA4P7ZsFhH0vDsOeu37Y6Vx4aT1\/5gEIbOPNf8EGIZE+YW8vg9Uzx3jJE69aFMng1rIZKfctxlh+E6LcSGNXHllVcKv7Li8W\/KlClmYNj+CuyPlWSLhZycYwA8jDbOHnnRbBpujA3n6MzPYC03lHDKIY9NTzjxvKg8xKRxi7MsBnLtTXeXRTrKokxnGTYd9SJc23jOCUNY\/oAetg7KoTzKJT36EU9ajlwT7habzysNZZHfCvWTn7JIb+u216QjnGubF50gHcI5z83NNevqsBJ4+ShjwYIFFCt0fcxJwR\/qojwr9h6AJy+w7ZIXJDU\/HJRv03IkvzuONH66gQWCnpRh66Qsrq3YcMp3C\/rRJpuWc8JIR93o4Awj3CnO+inD1uWV12KMfpRBWvIghDnjuSacNKRFeDex0rg\/kN8555wj\/Pgw08kPAmnc+lCmM5xyuQ6jpJzY\/ve\/\/wnLBWL1zQFh7dq1wvolwEb8bjbpVYoHArygdNlpDS8VP2x0n7AKsJYIjydYmXSdyRcvrcbHRoDuJxYlPzKc2+4rY5zcp9g5syMm5cQG68+dO1datGgRE4ENGzYI4y0xEzgiIEHAT1Y+\/\/xz6dGjhwwdOtSscUs2v6afYxa+phqHE0880SwFwXqxMnbsWGGAPJG6SEd6ykkkvabxvo8YFLwb9h7Y4yWXXJLS+87763idM3aaUmJjkJpxAn6NYxEXDyZrleif16lTx4wVTZ06VbD03K0GlL59+5qtHswOqXRSLDopBsm+B0Gm5\/3lPXa\/2+m+Timx0QXFvG3atGlMvXft2iXM8vTv31+WLl1qLKrhw4fLJ598sk8eAMH6I97+omTTsWfPnqZNqv\/Efay0TNxHxT8Y3O29BX\/eX95j8yJk8I8nsbG5uUOHDmbrCmMi8QR9ly1bJrNmzTKDw5FIhCBPqVChgmDV0VUtWbKkqaNx48bCejLPDAWBDGgyyJltcvLJJxdoL2ZvY7bpjr6qfzPTowCLIKS44G9eggz\/8SQ2uoU\/\/fSTdOvWTfLy8nyFNOi8evVqmT17tnAz7KAwXVLnAlPSLVmyxOynpHyurTCAbM+Ly7Fq1arCOjSO2dgm9Fb9g7tz2Y5\/cMiJ9zcPIpGI2Vh85plnSps2bXyFNDSAqV8GJK2w5oUpfTtFTBqEGRe2aTCo++uvvwprklgXw0JL4ouT8GCy7IVjNrYLvVX\/4O5ctuMfHHIxiK106dLSqlUrKVWqVFzd8PAQLxEr+VmrxtoZxuCYOMDpHpMHjLX169dP6tatG68YjVcEFAFFICEEPLuidBMfe+wxYRKAX2wI6eeff06oQJsISw0rjmsWKbJWzV4zXoHHA6w7uq+QKOlUFAFFQBFIBQKexIYVxhKM119\/3XRJcdT4hz\/8Qe666y5hWwxjcKmoXMsICAGtVhEo5gh4EhttZsayfv36MmjQIMEDxOOPPy7Mlp533nnGV9irr74qyVpxlKuiCCgCikC6EYhJbM6K\/+\/\/\/s+4jR45cqRMmzZN2NXPht\/vvvvOmUzPFQFFQBEIBQIJERuWGjOcrD3DYsMTBMR26KGHhqIRqoQiEH4EVMNMIhCT2LDG+KoNu\/7ZeIylxuzlxx9\/LEwMNGrUSOiuZlJZrUsRUAQUgUQQ8CQ2lme0a9dOHn74YeE4Y8YMYUzt\/PPPN25iEilY0ygCioAiEBQCnsTGLgCWe+B6iJ0FbGkKSkGtVxFQBBSBZBHYS2yFs+EoEs+k33zzTeEIjyusO49gDVIEFAFFIDAEPIkNbXDmd++99wrjan5CGtKrKAKKgCIQFgQ8iY2uKH6U2AfKxIGfkCYsjVE9FAFFQBEAAU9i+\/3vf2\/2isbbAG\/jKUglaQQ0gyKgCKQJAU9iS1NdWqwioAgoAhlBQIktIzBrJYqAIpBJBJTYMom21hU4AqpA5hB4ZtUhsr35rfLcF79krtK9NSmx7QVCD4qAIpBaBD5cuVV+qXSM5O8sm9qCEygtLrGxpg23RfXq1ROcRU6fPl1uu+024+kjgfI1iSKgCBygCKz+dmdgLfclNvyuPfDAA8Y90TPPPCPVqlUzX+zmS9H333+\/\/PJL5k3MwJDSihUBRWD\/EAggly+xbdmyRRYvXiysaTvqqKOkRIkSkpOTI1dddZVxOMlG+QB01ioVAUUgixA4smJOxrX1JbZY2vAtBHy0xYrXcEVAEVAEgkTAl9gqVaokDRo0kHvuuUf4PgFfldq0aZOMGTPGuAznG6FBKq91KwKKgCLghYAvsUUiEWHiAO8eV1xxhfmoMW6M6ILSPcVy8yo0q8JUWUVAEUg5AkFOHNAYX2IjwUEHHSR33nmnGWubO3euOeKnTa010FFRBBSBMCLgS2zbtm2ToUOHGg8fkBszpAMHDjTXePzAw+66devC2C7VSRFQBEKCQCgnD5gZfeONNwSSAyeu+TQfxzlz5kj79u0J3i9hjZz9bul+FaCZshQBVVsRp6XXMQAAEABJREFUSC8CvhYbM5+7du0SPuTyyCOPyODBg8352LFjBQ8gw4YNM+7D90dF1shNmjRJ8NK7P\/k1jyKgCIQXgVCPsWFR8YWq3NzcQgjWrl3b7Dwg3h1XKKHPxRdffCGM2fHlK59kJop0+fn5snPnzqyTn376SX788ces09tirfoH+8xlK\/7OxfunHV3BvMeZ\/ONrseFwkskDupxYWCjGEaLhHKttxYoVnCYl27dvFyzArl27CktK4mV+6KGHZMKECbJhw4asko0bN5ofAJbIZJvu6Kv6B\/u8pRj\/jL47b326Jt5rndZ4X2KDuHr06CEQy8knnywtW7aUU045RW6\/\/XZh+QdWFMs+ktWQLmiNGjWkadOmCWUdPny4dO7c2WzpYltXtkiVKlXksMMOk6pVq2ad7mCs+lcL9L5lM\/4LN+3ZbllixxYptWVpQu95KhP5EhsVHXfccfLWW28JX63Ky8sTSGb27Nly3nnnSb169eSdd94hWcKybNkymTVrlmCtRSKRhPKxjq5mzZpmO1dOTk7WHPlhKF26dNbo68ZW9Q\/2WctW\/DfvEJn79XbzbpfYEf+DUCZhiv\/EJTa6noyl8QvepEkTOeaYY2TNmjXSqVMn4YMvZcuWTUql1atXC8SIBVirVi155ZVXDMn16dMnqXI0sSKgCIQTAdwVWc2CsNaoOy6xjR8\/XiC0Zs2aiZWLL75YsKAYf6OQZOSss84y27PYooVQ1rhx4+S+++5LphhNqwgoAiFEgNnQ7pOWGM1KFHRDc5a+bs4z\/ceX2L7\/\/nuZMWOGTJw4UV544QUzrrZ8+XK58cYb5dhjjzVLPjKtsNanCCgC4UQAUrvo0QVR5VpU+zV6nukTX2JjqhmFWNJRvnx54QPKLF246KKLZObMmQLxEV8UwVLDiitKGZpXEVAEgkXAkhpHNGG3Qd65NTkNRHyJjcFLNrqzpANiY6wNQVNIzRIf1yrZh4BqrAgUFQGIbOjbq+TEe\/4lnFMepPbp7adwGpj4Ehvr2Dp27GgcTa5fv16OPvpoadu2rZGqVatKxYoVA1NcK1YEFIHgEIDELKENfTs\/qghWWtCkhjK+xEYClnXQ7WRMjTVrI0aMkJEjR8q9994rWHOkUVEEFIHijwBkNmneRrlw1AJjoTkJDSsNUss7t\/AupaBQ8SU2Nr6zT3THjh2GxOiasgXqhBNOkBdffFHYQRCU4lqvIpCVCGSZ0m4yY8bTuZzDEhpWWlhIDYg9iY0JAmZD33zzTXnppZeEIx4+rLz66qsmHMKjEBVFQBHIfgQgMQSrDAKr2GuGscw4d5IZLT3t6PIyqmN9CRuhoRviSWxYYnjwYD\/n119\/bfZ14pfNCn7YLr\/8crNdiEJUFAFFIPsQcJKY7V4yCQCRQW7OFmGZQWZ5BTOdkNnU7o2kY5OqziShOvckNvY3Pv\/888ZS6927tzl+8MEHYoVtVJdddplEIoltiQpVi1UZReAAQwACm71iq0BWkBYk5rTGCHdbZEAEmUFe1jKDzPIKxtAIJz7MUpjY9mrKR1twV4Trmnbt2hmXO5s3bxanEE+6vVn0oAgoAgEiAHkhkBTiJjAWzhJGXCwSwyKzRPbt\/a1MNxNSIyzApu1X1Z7ExkLcSy65JLqFym6lch6JJ91+1aqZFAFFIGkEIC4EckIgKqwvuo9WCEOI9yIwKsXicpLY690aGRLDIstWIqNdTvEktsqVK5udBezljCUsASGdszA9VwQUgf1HANJCbLex96urZMB7W6T92M\/Edh0hMIgLseRFHq9avQjMWmJOEmteu7xX9qwO8yQ2Z4vw7rFgwQLzpSr2iLJ+DbJzptHzoiCgeQ8EBCAfZPbesS5ICXLC4kKcxOXsNk5dsj3qAsgLJ8gLobuIYHFhgR0oBOaFCWFxiQ1fbN27dxdc\/TZq1Mh45mDcbdq0aeRXUQQOeAQgLMRNWpa4sLKsWNIiDnKju4j4gQhx2a5jXsGspJu8mKUkDIHciqMF5oePV5wvsbEvFG+3fHZv0KBB0qVLF\/MxF5xOEk68V6EapggUBwQgK8SPsLwsLUtalrgoIxYekBbiRVxz+jSS+TfVFI6265hXMCup5BULzd\/CfYmNhbpYarmuj7lwTTjxvxWlZ4pAdiCw+tudZsN2LMKK1TVMhrAsEk7SgpCwqhC6i1haVryIi7y2HD0mh4AvsZUpU0b4BN+SJUsKlbpq1aroFqtCEXqhCASEAFYR4iSrEf9cI3bwHbKiO5iIhRWva0gTIR3EWlpepOUe54LQSIfQXSQ\/ZamkHgFfYsND7tVXXy0s0sUVeL9+\/Ywb72uvvVbw+oH3j9SrpCUqAnsQgKgQJ1nhUQLLCaJC\/MhqxD\/Xih18h6woa0\/Jsf9CNkgihIW1ZS0tJa3YmAYR40tsKHT66aebnQetWrXiUurXry+vvfaa+ZiLCdA\/ikCCCEAsCESFMAaFQFQIRIX4kRUeJcgDUSGUl0j1kBViCStv7yA8hGS7hdbCUsJKBNHk02Qyhy+xfffddzJmzBj54Ycf5JprrpHBgwfLLbfcIoyxZVJJrSucCEAqCCSFQDgIJIVAUogXUblnB8kHUSGUmUiLISrEkhVdPIgKgaxevLqeGXxfP\/gUswDVSVh5ewfhyaPdwkTQzq40vsRWsmRJs7wDn2y472Y2FLLLriaqtokiAKEglqQmL\/iP6cr1nLxcgiKqPJdlBWFhWSEQFTK1eyPjaQJCg6gQyKp57QqJNl3TFTMEfImtXLlyMmzYMPnss8+MY0mOdE0vvfRSmT59uuhe0fA+DRCUFUtUWEV2jCoeURHfc\/IKM\/g+ecEWs4Eaawqh3ERajjVlJZZVBVFBTrGIKs9lWTUvhqvkE8FS0ySHgC+x2aKYGeUTfHwRfvLkyWamdODAgebjLjZNth\/Drj9kgiRLUnQDnd0+O0YFyUFSCOUm0n4nSTmJKm+vVQVJIW6i4npqTKuqvFBuIvVrGkUgUQTiEhtWGcs9+vfvL02bNhX8sB133HGCTzbcGyVakab7DQGIBEmUpGItUSgKSaENhII4Sapjk6rRbh1jVOMurmoWiLotKidR5e21qrCmEMqkfBVFICgEfIkN10StW7eW9u3bC1+kwk34nDlzhGUf1atX38cf2+effy6kr1WrljRv3tx8k9SrYVOmTBHSWKF7i0skr7TZEAZJIW6icm5ijjWITpcPSRVJQSpOosrba00x\/oQ1hWBBuYmKeCuQG9K8YIzqD0fkqEWVDQ+h6lgIAV9i4xsHzIQytsb3P9kryoRCoRL2XrC9irTdunWTZcuWSV5enuBxd8OGDXtT\/HZYu3atjBo1ykxMsKE+7J5CnKRlx6ggKiSeNZXsOipQgpysJEtSkJaXNbWHqMpL84IxKsqmnsBFFVAE0oSAL7GxQJexNcbY4tW\/Zs0a2b17t7HU+HrVySefbMbitmzZsk9WyK5SpUr7hAcdAIEhjD9hRTmJy45TWcuKsSkkEZ0hEkRJKhG0NI0iUHQEfIktmeIbNGgguBPnW6O4Opo7d67ZdsX3R53lsCaOb5TecccdUqdOHePMcurUqUIeZzrnOWXl5+cbT7549U2lfLV5u5nxg8QYaEcgNcjNj7gOL1dKELpq7RuWF+S+djUFYWwKef\/G+vJR12oyo3s9M041uUt9GdE+10jP06uZPORrXD1HkMplJC1tLApeDEGwJ7goZQSZV\/XfGcgzxfu6bt0652uc0fOUEZvVmm7riSeeaLZhMXZWvnx5G2WOu3btkrJlywqTEUuXLjXd1eHDh8snn3xi4r3+MBs7YcIEwdJLhcz\/co2Mm77cfB+x8b3zzBotLxKzxHVh\/YNkwFmVjDCYjseFqZ2rC8L17WeUF6RVDRHk6LI7Bfndrq3COOWmTZtSpnsq2p9oGRs3blT9C4ZSEsUr1enSg\/+GjDyLvK99+\/b1ep0zEuZLbPxSM1nA0akN1\/Pnz5eff\/7ZGWzOGzZsKAsXLhT8uFkxEXv\/VKhQwXz1qkWLFsJ4XcuWLaVx48Yyb968vSn2PUB8nTt3lmrVqhVJfi5dQZ5etFOue3WjWZ81f93OaGV0FRHGohhE\/\/i2JoJM7d5Ixl7RULr+sY6RCxrnJqxDlSpVzJe8sFqLqnsQ+VX\/oj1vRb1n2Yw\/72vPnj2j71emTzyJjSUeWBp48RgxYoRwZNbSypdffmm2Vzl3IcyePVv69OljZk9pBDOeLAtxm6MsHWF2lS4C6az4bag\/4ogjpGbNmpKTk7PfsnmHyJ+fWCIPzdwo67f9Yqp1EhmD7gikBrnVqnzQftdl9WTypXTp0kUux5aX6aPqv\/\/PWyruVTbjz\/vKOLt50QL440lsfKSFj7VccMEFgmXGsVmzZmKFnQdYZlhfVmfWtGGpLVq0yOxIYOnH4sWLhckHm4YjExJsoscShED5pB\/dV3c60qZCmAxgJpOxM84pE0JjGQRLHyyREa6iCCgCxQMBT2LjIy0zZ84UBu0hNY4sy7CyfPlyGTBggJn1tDDUrVtXevToITfddJOZFGDT\/A033CAnnXSS+Wwf423vvfee1KhRQ5g4GDJkiEnHWBvr4sgvYktLzREig9SYybQlWkLLOzdX12dZUPSoCBQzBDyJDUvq22+\/FQb+ISFmLG031B7pqpLO4hGJROTCCy8ULDEIkCPXkUhELFGykZ70WH6Mv5GOLqx1iURcqgRSs7OblGmtNCU00FBRBIo3Ap7ERleUL1IxlkaXFCJyC+GkCyM8ltTsTCekRrcTUgujvqqTIqAIpBYBT2LDwpo4caIw+E+XFMvKLYSTLrXqpKa0SfM2iJvUILfUlF7sStEGKQLFDgFPYsvmVrKw1o6pQWZYahyzuU2quyKgCCSHgC+xMZ7GoD9LN9xCOPHJVZfe1LYLSi2Q2SMd6usEAWCoKAIHGAK+xEZXky6n7YauXLlSPvroI7nuuuuM+yKWeIQJLyYLrD6sRWteu7y91KMiECoEVJn0IuBLbO6qI5GIsIr++uuvFyYWnAt03WkzfT17xdZC42p55+ZmWgWtTxFQBEKCQFLEZnVmv+e2bdvMQlwbFvSR9WpWB7qg9lyPioAicOAh4EtsrFXr0KGDsJ\/TKeecc45Zm+bceRAkdE5rDddA2gUN8m5o3YpA8Ah4EptVi+1PjKfhNNIpTz\/9tAwcONC4JbJpgzw6rTXtggZ5J7RuRSAcCPgSG5twzzjjDDnhhBPM9w7atGkj7BGtXr268cwRhiaotRaGu6A6KALhQsCX2FB1\/PjxcvbZZwu+07h+\/\/33haUe06ZN4zJwYTGuVUKtNYuEHhWBAxsBX2Jjvygb19mFgP80oLryyivN1+EnTZokfOeAsKCEdWssyKV+1q1l9dgajVBRBBSBlCDgS2y\/\/PKL6XLiD81ZW+3atYU4HE46wzN9brdNUS\/r1jiqKAKKgCLgS2yHHHKIMIHw8ssvR5d24OkDzx184KVMmTKBIjjx33u+gLbvCEEAABAASURBVIW11rFJtUB10coVAUUgPAj4EhveX\/Gx9tJLLwnfMWDJB\/7VBg0aJL169TKkF1RT6IZai61GhRzdOhXUjcjqelX54oqAL7HRaDx84Dvt2WefFZZ8jB49WvB6y0wp8UGJJTXq79RUrTVwUFEEFIE9CPgSG5\/JY4EuC3UhMpZ74JcNS25P9uD+Fu6GVg1OEa1ZEVAEQoeAL7GxCb5+\/fry8ccf+373M9OtcndDM12\/1qcIKAL7jUBGMvoSG5vc+ShL7969hS\/OMMZmxVpyGdHSVQnEZoN0iYdFQo+KgCJgEfAlNmZEu3XrJg8++KDceeedZoyNcTaErVbE24Iyefxw5XfR6k47ukL0XE8UAUVAEQABX2JjSxUfWmFszS2EE08hmRa2UVEnyzzUYgMJFUVAEXAi4ElsTBawzIOPG9PltN1P55Fw0jkLy8Q53VA7I8oyjxTXqcUpAopAMUDAk9joYnbq1Mk4laTLSdfTLYSTLtMYQGy2TrXWLBJ6VAQUAScCnsRGF5NlHRUqVBAv7x6nnnqqFKUryoRE69athe8oNG\/eXGbMmOHUyfdcx9d84dFIRUARKEDAk9gKwqP\/U+3dg43zgwcPFiYlli1bZiYkhg4dKhs27NkeFa04xomOr8UAJouDVXVFINUI+BJbOrx7rFmzRnbv3i1YaqVKlTLLSNh3umXLlphtmzt3ruTn58tXm7dHv2tQ7eBSsnPnztDLTz\/9JDgLyAZdvXRU\/YN9xrIVf97XdevWxXyn0x3hS2x48ChZsqSk0rtHgwYN5Pnnn5eKFSsKG+ohLQiOj8TEauxDDz0kEyZMkE+WrYkmOaFKKWPlYemFVTZu3ChMsGzatCn0unphqPpvCPS+ZTP+vK99+\/aNvq+ZPvEltnR69\/jss8+EjfUs\/sVxZfny5WO2ffjw4dK5c2f58vtS0TStT6wh1apVC7VUqVJF+EQhpB12Xb30U\/2Dfb7Sin+a3x3e1549e0bf10yf+BIbe0JZ9pEO7x64GF+4cKGwwd5KrMZjMdasWVPm5G+PJjmjXiXJyckJtTAJA4Zh1zOWfqp\/sM9XNuPP+8pupegLm+ETX2JDl1R795g9e7b06dNHGDugfGZGqSOZ\/jgLc8mroggoAoqAFwJxiY1MDO6nyrsHXTMstUWLFhnnlSz9WLx4sTRp0oSqYgrr13Rhbkx4NEIRUAQcCCREbI70RT6tW7eu0L296aabpE6dOnLNNdfIDTfcIDiwjBbucQKx2WBdmGuR0KMioAh4IZBxYotEInLhhRcK7sW\/+uorc+Q6Eol46RcN04W5USj0RBFQBOIgkHFii6NPQtE6xpYQTJpIEThgEYhLbNu2bZMXXnhB+vXrV0jYLUBcppBz7jhQYksH6lqmIlB8EPAlNmYub731Vhk3bpzZLRBks3XiIEj0tW5FILsQ8CW277\/\/XvCi++STT8q9994r7PG0grePcuXKZaS1a77bGa1HrbUoFHqiCCgCMRDwJTYWCB566KExsmYu2DkjqsSWOdy1prQjoBWkCQFfYitTpoxxT8SWpuXLl8vmzZujwh7IX3\/9NU1qFS7Wjq8Rqq7AQUFFEVAE\/BDwJbZvvvlGHnnkEXnjjTfk3HPPFXy0WbnkkkuEeL\/C0xGnFls6UNUyFYHihYAvsfH5vZkzZwrrzdxCOPGZgMNOHEBqSCbq1DoUAUUgexHwIzbTKlwL4YmDyQOWeNAl5VsIhJsEGfizu0zw43wZaKZWoQgoAilCIC6x4XmjS5cusnLlSsFKw\/kkW6ImTpyYIhX8i9ldppIgpKpRIYeDiiKgCCgCvgj4EhtuvCdNmiQPP\/ywDBo0SPCZlpuba5Z+MO7GchDf0lMcqXtEUwyoFqcIFFMEfIkNl9Z40YXMnO2vUaOGuWQBrzlJ4x9nN\/TIir9PY03hKFq1UAQUgaIj4EtsBx98sPCJPTasO5d2rFixQlgKghRdBf8Sfql0TDSBdkWjUOiJIqAI+CDgS2ws0O3evbvZcdC2bVthEuGyyy4zboeuuOIKQ3o+Zac86siKOSkvUwtUBBSB4oeAL7HRXBxMvvPOO3LbbbdJ+\/btjf+0N998U\/hOAfHpll8q1YtWocQWhUJPUomAllXsEIhLbLSYPaFt2rQRnEN26NBBMrV+jbqtKKlZJPSoCCgC8RCIS2zr168XyIzP5vFxhnr16hmCY0tVvMJTEW\/H2HR8LRVoahmKwIGBgC+xMet51113yeGHH2483bKWbdasWcI3EO655x5hxjSdMNn1a9ShFhsoqCgCxQWB9LbDl9hYp7Z161bzVSk+whKJRKRq1apy8803y8aNG4XFuulV77fSldh+w0LPFAFFwB8BX2JjVpRFuVhu7mJwZ0S8OzyV14UtNl3DlkpstSxFoDgj4EtsrFPr2LGj3HHHHcJMKONtbKvCq27Lli2FBby4MkoXQM4PuOgYW7pQ1nIVgeKHgC+x0dUcPXq0rF69WtgE36FgRhSSg+AeffRRwXURkglYMtsVzUSLtA5FQBFIFwK+xMa42vPPPy8ffPCBr1jl1q5da2ZQjz76aDn++OPl2WefFS8vIFOmTJFatWpFhTVxXpafes61yOpREVAEkkHAl9go6Oeff5ann35azjrrLGG5B98AZQO8c4sV6eiWMoMKSS1btkwgxAkTJsjcuXOJLiQQ4KhRo6J+3mbOnOm5Ns4Sm1prheDTC0VAEYiDQFxie+qpp+SVV16RXr16yfjx403387777pMnnniiUNF4AmEWlYW8JUuWFNa9sWvh008\/LZSOiw0bNkilSpU49RXnR1x8E2pkcUBA26AIpAwBX2JjjG369OmCg8nzzz9fGjZsKJ07d5YxY8YI4cRbTSpUqCCMxx1xxBEmiK9bse6N5SEmYO+fH374QRijY6yuTp06xt341KlTPbusq7\/d83WqEju2SH5+vuzcuTPrhBllrNls1B2dVf9gn7lsxZ\/3dd26dXvf+swfSvhVyQLcSCQikJYzHdeRSKTQAl0W7VasWFGw1mhQz549Be8gZ555pjOr7Nq1S8qWLSv9+\/eXpUuXGtLkYzGffPJJoXSW1Ahcv2yh0K3F0ssmYa0fOzQ2bdok2aS31VX13xDofctm\/Hlf+\/bty+sbiPgS2yGHHGJIbdq0aWLH1JgMwI0R7owgLqfWjMc99thjctFFF8mxxx4rY8eONeTmTAMp8oGYFi1aGBJk2Ujjxo1l3rx5zmSFzi897wxjKVarVk2ySWgEy2Q4ZpPeVlf0Vv2De+Yygb+916k+0rPDuKENQYgvsZUuXdrsMsCLLuQDCTGBgDdd3Bk5F+hCeE8++aRZ7\/baa69JXl6eOONt4\/hewrhx4wQT24ZxdJOk02Jj8oBfL8brskm++OILeeGFF+Ttt9+WbNLb6qr6fxrofctm\/Hlfq1evzqsdiPgSGxoxDsYYGJMIkNXIkSPN0g8mBoi3wkznu+++K\/fff7\/4NQhLD+LD6sMKZCkJft6aNGliizJHyOxvLQ6W0w\/ZJKPu6CadOnXKOrGm+EMPPZR1uoO36h\/sM1cc8McQ8uMD87Kn4U9cYqNOxs8gMmY8+a4olhzhToGh+YVhTM25Ro2JBtaosQzkvffeE9yKM3EwZMgQgTQZa+vXr5\/UrVvXWZxAbH3bN5aHuzSX58ePFD4eozJRcZioGGTTe8D4eWiJrRDjxLjA4lq8eHF0bZr9Dun1119v1qixVo21cGSHHPn6FWlmz55tvjZPuFu4BhTSqzQzM8iKg+KQTc8A7y\/vcaYlIYst00ppfYqAIqAIFAUBJbaioKd5FQFFIJQIKLGF8rYEqJRWrQgUAwSU2IrBTdQmKAKKQGEElNgK46FXioAiUAwQCCWxsdgXl0e4PuLjMXgN2b59eyjhTlRX1uzhJYU24dYJ33bsmQ26UYnq79STnSisLA\/DPUlG\/48++kiaN29u3GW1bt1aPv\/8c2ezAjnfo\/+zxs2X37POfmP2bJOGZVKXX3652XMdiNIJVopTjCuvvFJYp5pglpQlCyWxsW+UFfss5P33v\/9tHF3iYSRlrU5hQYnqykv14osvCm3ihWJ5DITt3oGRQtUSKipR\/W1h7ANmETbb52xYkMdE9V+zZo3Zl\/zAAw8IbrUuuOACYfsf+6GzQX\/WgC5YsED+9a9\/yaJFi8y3Rx5++OEgVY9ZNz\/iLMD\/+9\/\/LqxtjZkwjRGhJLYZM2YIC3pzc3OlXLlyxnnlP\/\/5T8EzSBqx2K+iE9WVG3zGGWcIbWKr2TnnnCOQBL9q+1VxijIlqj\/VQQJshzv11FP32QNMfBCSqP7scGnUqJGwNbBUqVJy3XXXyYABA8x+5SD0tnUmqr9Nzz2w51WqVLGnoTrynrJuFYcYPOtBKBc6YuPG4fKkdu3aUTy4gbhIArBoYAhOktGVLUosVrZqL1++XHjBvHZx2DTpPiajP7q8\/\/77QvcTJwdcBy3J6I+VjBum8847TxgKuOKKK8wPZSQSCawZyejP4nY8WrNFCQcTLIa\/7LLLAtPdr2KMEbZf3nbbbcaJhl\/adMUlQGzpqtq7XB4+SIw9pd4pwhOajK7cbNqEmU43e\/DgwdK1a1fBg0pQLUpGf8YD2S+MpfO73\/0uKJUL1ZuM\/nSd8SBDVxR3WaeddpowFMDYVaFCM3iRjP4MxbA1cdasWaYrylgt3yGBHDOoctZUFTpiY18qX8fiQXSiiHUTiQT36+rUxZ4nq+uqVauMB2K8oDz++OOC9WDLCuKYqP4McKMz+rr39Aaht60zUf1t+vbt20v9+vWFfOxpxk8enp9tfKaP6JHIs844LF1WLGUcufID2bFjR\/n6668Fh66Z1jsb6gsdsdE1Y6M8Y1IWQB5AzHCsHhsWhmMyujKedtNNNxlfdXwzAqcCQbchUf0ZAuB+4LwABwdt27YVPLlg9QQx42VxS1R\/0uPJGQuJcyslSpSQSCS4H8tE9WesKicnx6odPUKMxEUD9CSKQInoWYhOmLFi8BELh18klknwERkehBCpaVRJVNfJkyebzf5XXXVVIgPWpuxM\/ElEfywEPErgtABhZvfss8+WDz\/80LiLz4SesepIRH\/ytmrVyrizZ2yK7udLL70kTCYk8u0N8qdLEtGf3sopp5wir7\/+uplwQn8+loSrfhy3pku3bC43lMR20kknCQOjWAbc0OOOO05YdxRGoP107dOnjyCMg6xYsUL4MhcD11g9CDO\/jJsE2a5E9A9Sv3h1J6o\/6W644Qa5+uqrzZqxb775RrCgI5HgLDbahl6xnnWeHYR0f\/7zn82X4hgOYHyN7uktt9wSqMWJXmGVUBJbJBIRFiCyXoeB3ltvvdWMi4QRxEgktq58zQvhFxd36Fg7TsEqrVy5cqDNikTi6+9WEEsBt+9Ycu64TF9HIonpH4lEBKuf9VXMSOP8MwzWTiSSmP50O6+99lozcYD+rCV0e53ONPbx6uPZxkktz0u8tKmODyWxpbpxbqgsAAAGu0lEQVSRWp4ikAgCmqb4IKDEVnzupbZEEVAE9iKgxLYXCD0oAopA8UFAia343EttiSKQfQikSWMltjQBq8UqAopAcAgosQWHfdI188UvO\/0fL3MyaeOV5YxnHRiuf9j7yr5RZ5zfOXqjk1+aTMfhMYNlNyyWTtdCY8qlfITzTLfxQK1Pie1AvfP72W72jB555JGCl49ML\/dgaxd7bfdTdc9sF198sSxcuDBtC41Z6sBC5mbNmnnWr4HpQUCJzQ\/XAOLYu9irVy+xDgU5JwzrYtiwYcJmaGstsUYI6wmrA\/9u+Bdjj61X2iVLlgh7JVkgzEtGXojCq4ms9WJBNOU601IuG\/eJ5+i22CiPcslDPeeff34hZ46QIg42iaN8PG5QPzqjO22gTtpEOZSHldOmTRuh7YRjMYIHuOBwkboefPBB44bI6sNGcfaCUhb1oC\/1+Al5wRWr0uJ06aWXCrtfvPK5LVCwIT\/leKXXsMwioMSWWbzj1sa2GV4OnAqyQJn9jGyYx20NC5WxMNjexMs9fvx4wVsFi345x2sIizfdafH51rt3b\/nrX\/9qnCw+8cQTMnLkSJk9e\/Y++uCEkT2hf\/vb32TlypXiTEu5WGqQCUe3xUZ5lEseymFBLB4oaA8VYbnceeedRoc\/\/vGPAiGxgp5F2G+++abgiJM6+\/fvLxDMli1byCZr164V\/HrhNoldKLQZDyNghDff+fPnC3iQGCIcMGCAUC9Y0A7agz7Ex5OXX35ZWPyKTi1btjQ7R2zZ8fJqfHgQUGILz72IasLLTfeIHQvsXLj55pujcfaEPY543GBLDuSAdQMJenXVIARWgbO\/k03TDRo0MC6y586da4uLHrF2jjnmGGGDeyQSEdKee+65AvFEE8U4IQ1pyUM9WGe4OeKcLGwHwrsG1+i9Y8cOwVrDYwjkXbNmTeMjjVX2u3fvFtpEvsMPP1zYXkc4ZIfXXLxbQKzsHsBNOW0nLRZtixYtjENJ6oGEjzrqKMHSIz6e\/OlPfzLOQKmLOtExPz8\/XjaNDxkCSmwhuyFYZN27dze+wrBO6A7ZLptTVciMbVps5MaSwsrgJXSmsedYHHiQxUEh3TNkwoQJsnHjRpskeqTLSr2Qqg2EQCAge+11xCrDjQ5pbTykA8lgbRHmjOPayrZt2wRLjgH2tm3bylNPPWUIz8aji\/Vugc7ogrcXG48j0rJly5pL2orTBLq7tBMinT59umdbTQbXH6eDU+qkblcSvcwCBJTYQnaT6AKx8R\/rB0JjjIjxJVwHOVXFwwZdsZkzZwpWFpYdROJMY8+xPhjvojtGtxX59NNPjWtsm8YesZqwGK21xJHrqlWr2iSeR0igYsWKwoZ\/m4AuJHsyIS4b5nV89NFHTTBdSlzA433VkqGJcPw59NBDBets69at0VDqgdAIoK3dunUT2oig+8cff2y64cTHE1xk2TS4OcICxvKzYbGOznyx0mh45hBQYssc1gnVBFFBUlhAZMAKwxkhlgMvrbOLRjwvHmmfe+45Y5UQT7gzLVbTl19+adwMQVS88Ax0Q4ykdQqkShcVMiAt3T6uW7Vq5Uy2zzn60Q39xz\/+YQbcca3DWBt1of8+GVwB1AUp4qYKaxIy5NqVzLiarlatmjzzzDNCHfi5I71NB4G\/\/fbbputJmbTjL3\/5i3HKaNP4HenKUiZljx49WqpXr266pu48bEDHeqQOSJXJDncavQ4OASW24LD3rLlLly5mbIkuJjOjDLjffvvtgi86SIdrXlRm\/XJzc4UwuqKc84GYHj16CK6QCLdpGaO6++67ZciQIcbfP6TG2BezjG4lGPvi60K49KE717NnT+nbt68Q7k7rvsa6ZIyKsSlc62BJYX1Beu60zmu8VjB2RheYsTPKwXJk5hGCcabFkhtQMDnAbCXpcfnDmKBNQ5vAAPdE6M\/EAd1cxvFsGr8j+bkHlE0d1EWdYIqbKWY\/yX\/NNdcIVi\/fIHDr4E5L+pBLsVNPiS1ktxRLgPEyZvToRuFQEKsBNRmUx3piVpSuoU3HcoZ27drJoEGDBIuPiQJnWsa2GER\/6623TBeN9MxYRiL7+iKLRCLG7xdpsLaY6XSmhUSpnzLRySlYiZAUs7noz8wppEoarFDnx2yc5ZCGdtJedCQOK4x6mjZtKlhDtIlysJDo9hJPevSjbViF1B+JRKLuidCf8ognbyLCujPyUDY6oRv5qB9s0Y1rXHRPmTJF+DwkwwYDBw4U9AUXd1rSq2QWASW2zOKttRURgV27dgnLXlj3RjecriuEwtIMrNoiFq\/ZiwkC\/w8AAP\/\/RWDrkAAAAAZJREFUAwChLzVJ3ZUvUQAAAABJRU5ErkJggg==","height":187,"width":310}}
%---
