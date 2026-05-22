clear;
[model, options] = init_environment('three_phase_dab');

CTRPIFF_CLIP_RELEASE = 0.001;
s = tf('s');
%[text] ### Global timing
% simulation length
simlength = 2;

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
t_measure = 2;
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
% ixi_ref_mod1 = -0.88;
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
frequency_set = 50;
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
dab_pwr_nom = 750e3;
cllc_pwr_nom = 250e3;
psfbc_pwr_nom = 275e3;
fres_dab = glb_time.fPWM_DAB/5;
fres_cllc = glb_time.fPWM_CLLC*1.2;

hwdata.single_phase_inverter = single_phase_inverter_hwdata(application_voltage, single_phase_inverter_pwr_nom, glb_time.fPWM_INV);
hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, glb_time.fPWM_AFE); %[output:08f7ea95]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, glb_time.fPWM_INV); %[output:7108570f]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:58216e3e]
hwdata.psfbc = single_phase_psfbc_hwdata(application_voltage, psfbc_pwr_nom, glb_time.fPWM_PSFBC); %[output:9129a1ff]
hwdata.three_phase_dab = three_phase_dab_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_DAB, fres_dab); %[output:81726121]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, glb_time.fPWM_CLLC, fres_cllc); %[output:682be03b]

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

time_i_react_pos_ref_1 = 0;
time_i_react_pos_ref_2 = 0;
i_react_pos_ref_1 = 0;
i_react_pos_ref_2 = 0;
i_react_pos_ref_3 = 0;

if enable_i_react_pos_steps
    time_i_react_pos_ref_1 = start_time_LVRT + error_length + 0.335;
    time_i_react_pos_ref_2 = time_i_react_pos_ref_1 + 0.5;
    i_react_pos_ref_1 = 0;
    i_react_pos_ref_2 = -ixi_ref_mod1*tan(acos(0.95));  % cos(phi) = 0.9
    i_react_pos_ref_3 = ixi_ref_mod1*tan(acos(0.95)); % cos(phi) = 0.9
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
upqc_st.Lm1;
upqc_st.Ld1;
upqc_ctrl.kp = 2;
upqc_ctrl.ki = 18;
upqc_ctrl.lim = 4;
%[text] #### DClink Lstray model (partial loop inductance)
parasitic_dclink_data; %[output:3111a41f]
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus(); %[output:9be51af7]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:4721b059]
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
psm_ctrl.ekf = ekf_pmsm_setup(psm.Rs_norm, psm.Ls_norm, 1e6, glb_time.ts_inv); %[output:5d9ad9f7]
psm_ctrl.kp_i = 0.25;
psm_ctrl.ki_i = 35;
%[text] #### Induction Motor Control
im_ctrl = ctrl_im_setup(glb_time.ts_inv, im.omega_bez, u_im_scale, im.Jm_norm);
im_ctrl.ekf = ekf_im_setup(im.alpha_norm, im.beta_norm, im.gamma_norm, im.sigma_norm, ... %[output:group:77cc7e40] %[output:1ef39550]
        im.mu_norm, im.Lm_norm, im.Jm_norm, glb_time.ts_inv); %[output:group:77cc7e40] %[output:1ef39550]
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
sogi = sogi_filter(omega_set, sogi_delta, kepsilon, glb_time.ts_afe); %[output:3a544745]
%[text] #### Current control parameters DQ PI
dqvector_pi.kp_inv = 0.5;
dqvector_pi.ki_inv = 45;
dqvector_pi.pi_ctrl = dqvector_pi.kp_inv + dqvector_pi.ki_inv/s;
dqvector_pi.pid_ctrl = c2d(dqvector_pi.pi_ctrl, glb_time.ts_inv);
dqvector_pi.plant = 1/(s*grid_emu.trafo.Ld1 + 1);
dqvector_pi.plantd = c2d(dqvector_pi.plant, glb_time.ts_inv);

G = sogi.fltd.alpha * dqvector_pi.pid_ctrl * dqvector_pi.plantd;
figure; margin(G, options);  %[output:5c15d3a5]
grid on %[output:5c15d3a5]
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
nominal_battery_power = 750e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery_setup(nominal_battery_voltage_1, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:6b1bf100]
lithium_ion_battery_2 = lithium_ion_battery_setup(nominal_battery_voltage_2, nominal_battery_power, initial_battery_soc, glb_time.ts_dab); %[output:7ceb089b]
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
open_system(model); %[output:50391e58]
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
%   data: {"layout":"onright","rightPanelPercent":33.7}
%---
%[output:08f7ea95]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V\nNominal Voltage: 690 V | Nominal Current: 2.324276e+02 A\nCurrent Normalization Data: 328.70 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:7108570f]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:58216e3e]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 750000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 1.273240e-05 [H] | Internal Tank Cs: 3.453883e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:9129a1ff]
%   data: {"dataType":"text","outputData":{"text":"Single Phase PSFBC: PSFBC_1200V\nNominal Power: 275000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls1: 1.212121e-05 [H] | Internal Tank Ls2: 1.212121e-05 [H]\n---------------------------\n","truncated":false}}
%---
%[output:81726121]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: Three_phase_DAB_1200V\nNominal Power: 750000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 4.000000e-05 [H] | Internal Tank Cs: 750 [F]\n---------------------------\n","truncated":false}}
%---
%[output:682be03b]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 750000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 5.160246e-06 [H] | Internal Tank Cs: 8.522115e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:3111a41f]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAScAAACyCAYAAAAAhgkFAAAQAElEQVR4AeydCbxNVfvHny0VUYaQ4cW9lwYqEbfMU0TKUKL0ZkiR4W2gF80oZSwqRCmRoVca8FZkLqVEFKUklEQTUuoi+d\/velvnf9zOOfece89ee59zV5+eu\/fZe+31rPXb5\/w861nPela+Y\/Y\/i4BFwCLgQwTyif3PImARsAj4EAFLTj58KbZJFgGLgIglJ\/stiAoBW8giYBoBS06mEbf6LAIWgagQsOQUFUy2kEXAImAaAUtOphG3+iwCyYxAHPtmySmOYNqqLAIWgfghYMkpfljamiwCFoE4ImDJKY5g2qosAhaB+CFgySl+WPqvJtsii0ACI2DJKYFfnm26RSCZEbDklMxv1\/bNIpDACFhySuCX50bTN27cKBdffLE0aNBAGjZsKDVr1pRrr71Wvv3226jVTZo0SZYsWRJVecqdc845x+l74IEH5Ndff5WDBw\/K3XffLZs3b46qLlsopwj48zlLTv58L562qnr16vLGG2\/IW2+9JWvWrJFKlSqpz2416sILL1T1o2\/VqlVKzYgRI+Tkk0+Whx9+WKpUqaKu5eTPsWPH5OjRozl51D7jMQKWnDx+AX5Xf\/jwYTlw4ICUKlVKjhw5IqNGjVKWFdYV51xDRo4cKenp6dK8eXP58MMPVbcghhkzZigLrH79+qItInUzzJ+CBQtKt27d5JNPPpEvvvhCbr75ZsGa++WXX6R\/\/\/5Su3Ztueiii2TMmDGqPVy\/9dZb1bXrr79eevToocpjvfXp00dat24tq1evluXLl6t21K1bV6644grZtGmTfP\/999K+fXsZPHiw0D4sxOnTp0vTpk1VfcuWLQvTSnvZBAKWnEygnGA63nvvPalWrZqkpaXJueeeKz\/88IP6YS9cuFB27twpWDfI3r175eWXXxauf\/PNN8rS4jNkRpcpw5Bs6dKl8vbbb0uhQoVk3rx53IooxYsXlxIlSgjEowu+\/\/77UrlyZUU06Pv8889l9+7d8swzzyjLivtYWz\/++KN+RGjf7NmzVV\/efPNNgSjfffddRX7UQcGff\/5ZatWqpdpXtmxZod7FixfL6NGjVVv\/+OMPilnxAAFLTh6AHlml93exTj7++GPZtm2bsl6wKu6\/\/3555513pFWrVnLiiScqadasmXzwwQeycuVKdR2r59RTT1VWCL2AMCCHs88+Ww0NJ0yYIOvXr+dWzNKkSRPll5ozZ44a6kF63333nWzZskWwhhzHkXLlysn5558fqLtx48ZCe0477TQZOHCgKjt27FiZOXOm7NmzR5XDIsQKdBxH8uXLp9p+wgknCASZkZEhiNj\/PEHAkpMnsCeOUn6ojRo1Uj\/S33\/\/XZGSbj3DOYhKf856xKENKUByCIQyZMiQrMX+9hmLB6sJ60nffPbZZ2Xy5MkC0fXr109ZdAwb8SdBKpTjnGEo58GCM793796CVdWxY0fp2bNn8O3jziP157iC9oPrCFhych3ixFYAATA8K1CggPI1MbsGKSGvv\/66slrwNXHOtX379gnDOHqNb4hhFEQDseHbYXjHvXBCuSeffFKqVq2q\/Fy6HMOtyy+\/XHDWQzYQHZYaQzHaRzu\/+uorweLTz+jjTz\/9pCwofE1YRLp9+r49+hMBS07+fC+etmrDhg1y2WWXKT9TnTp11HBu0KBB0qFDBzXcYZiHlC9fXlq2bClt2rRR1xlede\/eXcqUKaPaz7AP4uLIEAti4FzdDPqDA13ro15IB+d3UBGle9iwYWrYhU8In9gLL7ygHOAQXu1MRzlhB2eccUbwY+ocX1WxYsUEC\/C6665TxPfRRx+pYZ4qkJh\/kr7VlpyS\/hXH1kF8NviK+MEztY9zHEcyFgpDHoZp3Ec45xpkMnToUOV\/euWVV2TixIkCCXHvlltuEcoiunxwiyj32WefKYc0+tatWyf4twoXLiwIQznahF8InxdWEiQ0fvx4eeihhwTLDOKk\/scee0z5jRgO9urVSxB00T5mFqmb9kGgEBxEOGvWrICFxgwg7eEZdKKbNvDZinkELDmZx9xqjCMCBHBCVvXq1RP8SYQTaMstjmpsVR4gYMnJA9CtyvghcN5558lrr72mhp5YXszqxa92W5OXCFhyig\/6thaLgEUgzghYcoozoLY6i4BFID4I+I6cWPBJlDHLClh4inDONe7Fp9u2FouARcDvCPiGnIhTYYaoS5cuKnqXBZ9z584VhFkUiAln5\/z58\/2OqW2fRSAsAvZG9Aj4hpwI1CMa+cUXXxQWbBIJzNICJDU1VSCtl156KRBDE30XbUmLgEUgERHwDTmx\/olYFtZKsdCUSGFiUJgm5shnyIugvkQE2rbZImARiA0B35ATq79Z3tC1a1eV0oJAOVJWsASCALtHH31UGPrF1j1b2iJgEUhUBHxDTqzJIr8OMSsM7VjASf6fdu3aqTxArCJn4WY8gbZ1WQQsAv5FwDfkhFXE0I6lBiwyJQti\/vz5FXIM5\/hMGXXB\/rEIWASSHgHfkFPSI207aBGwCMSEgK\/IiZw\/LCAlo+GXX34pHHGIc417MfXMFrYIxAsBW48nCPiGnMhYyAp2UmtccsklamU6RxZ0co17lPEEJavUImARMI6Ab8jppJNOEhZxEioQSrhHGeMIWYUWAYuAJwj4hpxIzYp1dPXVV6tQAr1shaUr7Ogxbtw4tY+ZJyhZpRYBi4BxBHxDTiVLlhQyGy5atEhtL8S+aSxnQVhXBzIkLxNOrFgELAJJj4BvyEkjzTIWosEhK32N1Ktc456+Zo8WAYtAciPgO3IqUqSIHDp0SO20QSJ75PnnnxeCMq1DPLm\/jLZ3FoFgBHxHTgRbDh8+XFjOcu2116rE9mzY+OCDDwoBmsGNt+cWgfAI2DuJjoBvyIlhG0tUAJRIcZLWk3aVpPYksmf3DO7pMpxbsQhYBHKPwB8\/7JBfVjyX+4riXINvyImlKgsWLFA7ZrDQl\/xNuq+cs6ssu2lQRl+P5siSl+nTpwsLiNltY9q0acLmi\/rZtLQ0sWIxyGvfgbrnVJAh9YvLKy0Lytd9UmXj2B7Z\/g70b8bU0TfkxExcjx495L777hM2aCRRvf7CXHrppSqBPQuBKRMLOOyJxjZAzAIyA0imA7bQDq6D6HM3BV1u1m\/r3qa2TvcjDn5591veXyafLnhW1o3oKqu6VJQZ5++U7tWKSP3W10rZIcul7rwjETGkH6bFEDlF361y5coJy1UgEP1lY28ylrGQeC76mv5XknrIE8WwEIc6AZ5r1679301Df5cuXeq6pu3bt7uuAwUm9CSLDvDy8t3r4doPE25Q1tG3Q5qo4VvBqo0VIVWYuF1K9p0qBc5tTFN9J74jp3gjtHXrVvnHP\/4RqLZSpUqyY8eOwOdf6w+UUl0mSfNHVrsmXWd\/5Vrdut0mdKDLhJ7sdJw35G2JRor3Xy7hpOYTO\/52jzohxngKkznxrC9UXcfp+PBt2f7iGNl2Zx1FSD\/Muk9+2fmZSPPbxLl5lhwdsEz2p3eR3adUlFB1hbrGCCbwgzF4kvTkBJbMAHIMJfl++1E6XNZIzipb1DVJLVnItbp1u89PPcN1HegyoSc7HZ3rlJdIsmlIA0H2PtpEYhHqvHLmHoG4xryXIaSHzq3wD2Nu68ju+TrnlJeiH0yXk6d1l2PDG8oJyyfIqeXPUVZR2lM7JW3EaknrOU5Sm3XKUZ8YwYT67bh9LenJKSUlRchwoIHkvEqVKvqjnPLhszKhUxVXZUizEq7W73b7DdYfFU6DWqRIJAm83BhPqHPDvXUy606Vd77cL9WHrZaRi\/7fyo6xOteKM1zL+GSF7JszVL7NHKrh0Ga27cSSKccN105t3M21NpioOJ8JJbHq2LRpk7Rs2VIaNWokOLB79uwpBGPGWg\/l69SpI+vXrxeiyxHOq1evzq2kEsxxEx0yocdrHZqkOqWXySSn7Yqkvt6bYQLesDogJAgolP+I4Zrf\/UdhOxbhhu\/IiXinyZMny+233y44x7FyWrduLWwVxb0IfQl568ILL1Qkd9lll0mLFi2EWUCuhSycwBcx\/U0034Qev+jQJFWhWAFFUCbwDdahCUlbR1hKR37YoYZrkBFSrONgNVwLfi5Zzn1HTlg3+IjYGgqQSdELoZC6l3tcCyfM6pEDipimVq1aCRaYLsvyl3z58qkYpz\/\/\/FNftkeLQEQEKhQvIPP71lDDTZzrDPUiPpDLmxASJBRMSAzX0l48JpAR0\/4M1\/KXTMmlJv8\/HomcPGk90\/0EZO7evTugf\/PmzcLmBpGWr7BBwhNPPCFjxoyRNWvWSO\/eveXxxx+X9957T7KLcwoosicWgTAIdEovLfijsKIgqTDFcnQ5Gv9RjipO8Id8R04Q0PXXXy9DhgyRdevWCcOxf\/3rX9KvXz+BuMLhzV53KSkpcuaZZ6oiJKf76aefVPCm13FOqkEu\/zHhp6ELJvT4VYe2oub3qaHCEHJqRWEdaf\/Rtg6OcmrzmfijtL8spFjij0zgxbs3Lb4jJwCAWIjoXr16tSxcuFAYrl1wwQXcCitnnXWWsGC4cOHCcuTIESEHFGlXiAFhOlc\/mDXOietpaWlRx3zwRYhVaEOsz8Ra3oQO2mRCj991lDthnyzo+g+5rHIBRVJ3vbA+7Pcn0Jcw8UdOx9HijNoWiD8C41gloGP79rDtiLXO4PL8PvidmBbfkBOWD1kIyICJMFPXrl07IRMmPifuUUYDxB53lAG4f\/\/73\/qysGNwx44d5fPPPxcyGTBExIcVKBDihDgOnLBuCeToVt263gYNGuQohkU\/H+0xlJ5on422XCLoqHfBmTL82hqCFfXG1gy5cuYe2XW02HHvoHxhR8ptfulv8UdYRzr+KLXDv497JlqMgsu5jRe\/jxA\/G9cv+YacihcvLviM5syZI23atJEbb7xRZcYkOybDPBzcJUqUCADCUpaVK1eq9UD4mY4dOyYs8O3bt6\/0799fnnzyScFyYqhHbJN+kHNmAPVne7QI5AaB+pWLKl9Up\/Qy0uvxN+Sp0SOE6X6Ga8QfydqXJNihzXAtN\/ry0rO+ISdm5SATjqRFwWIqW7asINddd518\/PHHyike7uVg2pKx4KmnnhL+JaEeyto4J1CIn2Dux6+20DUlkg78R\/vmDJXOq2+Q+buvk6ofPiYrVm+QhSk3CxaSc9dbauo\/dE\/jc9UEXvFpaWy1+IacdLPZYYWZOdbE6WvfffedIiZNOPp68JHyhA506dJFateuLfigILjy5csLYQg1atQQBId7dv6r4HoT5Tw1NdVIU03o8bMOyAjndbB1xGdtHdWfvVu2dX5F7j16jeAwX7fL\/eBNE3jF5csVYyW+IyfS9Hbt2lX69OmjLCCsoE6dOgmkw9AvXP\/wSxFygCOcoVzFihXV0G7nzp1y4MABITvB+r8ixU1nJQjXZns9MRCAfCCj4NgjgiGLdRiirCPij4KHa8HBmz1f3iN+XAKTCMj7jpwADaLBnzRr1iwhm1rq\/AAAEABJREFUfzg7sDRt2pRbEYVZuvHjxwv+KYaIFIaULr74YvEyZQrtsJIYCAQTEX4jhGEbZBQ81U8wJNHZ4Xqlww6euqp0JjltF6yoVVv3hytur4dAwHfkxIwcM3NEev\/zn\/9UFhP71nGNeyH6ELi0ZMkSOeWUU9TWUvoiwz1my\/TnUKEE+l4iH035HUzocUMHBINgASGkFNGWEASkhTIQkR6m4TfCMsqOjMJ9d2qWK6AyI+AwbzNxvbSZsF7ivU7PDbzC9cfkdd+RE0M3Zu3mzp0ryNNPPy1XXHGFaOtHg5M1lACHODFRLBLO6puKFEqwpOZ20V9Mt47HBqb5T0cHJ0dtSsi+9Cwv+5ZMUUJuI0SKlZOMgqfL0SZ9VZwRsUYI+Y4OdX1Wfm11f1xihvheQh4dzzqmYqN+z8hQVlSk2CjKxyJaRyzPxFI2LS1N\/+yMHn1HThALQzJCBRDW2BEawHo4fEcaHe4x9CMGg1CCzz77TJitq1mzpnKIs2wFiwsHOOED+jnOg0MJmq1LVX4D\/oV0S\/jSu1W3rfdY9u+PnEZayG2EDJoraZkSjzgjHNLhBKtd3yM2avEddWRQi1SZvGZ\/yNgoXTaWI37ZWMrHWpbfmP79mDzmM6ksp7pwau\/atSvi482aNVMxTwAJMTFjx5q6du3a2ZQpEZGL7Sb\/4sb2ROylk0VHuJ4PapESiI1qkznU6zt7c7iiUV03gVdUDYlzId+RU\/BwDXMSwfdEmhOc2rH2n+eIJGeNXjKnTIkVF1veWwRwmENS8\/vU8HViOxHvcPIdOZ1++ukyf\/58lb0SKwhh7zpyOjG0iwQVCekI2KTszz\/\/LBCdLp\/sKVMw1XVf3Tya0JMsOqJ5D8ER5iMX\/W9WL5rngsuYwCtYn6lzX5ET+9OxJu6ee+4RNiGAXBD8SayfI8tAOGD++OMPGT16tFx11VUqTUqvXr3kueeeE2KaGN6xkDjc1lDh6rTXLQKmEMCKcisli6k+xFuPr8iJmKQpU6bIp59+KhMnTpRx48YpIdYJnxIzeeEAwGrau3evEA\/lOI6Q5heSI+2Knukj5Up6eroirHD1JOp1U34HE3qSRUes3yWGeiS2Y6hHzqhoY6NM4BVrX+JR3lfkRPAlW48\/8sgjKv0JqXm1XHPNNUKGgXCdZhhHECb721WrVk06d+4s+\/fvl+SKcxL7Xx5AIHioh8Oc2Kg80O2\/ddE35ESAJcTEceTIkcoCInWKlqxBmAz3cHTjMGfIx3o8hoRkM2CRME70u+66S+V2ihTnBCLUwb8+bonbcSi0m35wdFtM6EkWHbyLnL77oz\/vlo5BsVFYUuFio9zGi98HOkyLb8iJ2CaGYRUqVFCpUt566y0JFlKnUEYDlDXOiXQqtWrVEh3DxFCOnOFlypRRznX9XNY4J67jdMep6JYEx7q4pcPWm5rrvEhuYJjbd+9WbFQsfeX3we\/EtPiGnII7jlVElDhDNC1YU8FBmMHlOU9JSZFDhw7JF198wUdhETA+JqwrveCXDRI4r169uiqTTH\/4V9pEf0zoSRYd8Xwf2mGul8EEx0aFwCueqj2ry3fkBIHcfvvtwjZQ9erVEy3sqHLiiSeGBQoiYnh37733qmdINnfnnXeqaHEIysY5hYXO3kgQBHCYQ1I4zP286We84PQdOUFKJJgjZcrll18uWnCWsxQlUsfxV0FuZMVkSBfsa7JxTpGQi\/4ew4HoS+esZLLoyFnvs3+qflD2TWKjSBEc78XE2bfC\/RK+IyfCBfAtMSyLpfuEEUydOlWmTZumdlxhXR1Dww8\/\/FBsnFMsSNqyiYIAVpSOjSLsYOSiHYnS9Kja6TtyYv851sYxO8dQLtxsXaje4XMifIB7GRkZaispYqdwjrP0haGfB3FONMd1MeV3MKEnWXS4\/tIzFTDUe6xVUbXRAlYUJLVq6\/7MO4n\/v+\/ICavplVdeUTNsa9asCczYZZ2tywo9FheERjwUOZtmz54tZNC0cU5ZkbKfkxGB4KFem4nu5I0yjZvvyIkZOWbm9CydPg4dOlTIiEmgJSAxo4ejmxgMHOFMd7L7CktUCBfgGqlUWNYS7Hvi2axCHfxr7ZbkNNYllvbQp1jK57SsCT3JogOMTb77rLFRWFHhYqNoW7TC74N3Ylp8R07MyDE8I6iSnVMY2pEyBUc5+ZqGDRumMMoa5wQ5nXHGGUKclOM4QswTa\/FKly6trDD1UOYfiEvHQmV+VP\/zLE5YtyS3sS5utcvW635sVCzvPl7vI96xUfw+1A\/F8B\/fkRMkhPXEIl6yC7Rr104ee+wxYbatd+\/eguM7FEbsssJsHTu1cJ81dQwRsa6IbWIWD+HcdJwTKYdpk5vCv4Ju1q\/rNqEnWXSAmZfvXjvMQ8VG0Ta\/i+\/ICcBwZkNGnCNHjx4VMhYwRONzKGErqG7dugkpU+rWrSszZsxQm2vWrl1bIKhIcU6YrW4K7XWzfurmR8DRbTGhJ1l08C68fveNa1WVyb2bSuFVo+Q\/KzdJiV4vCu2KVeiHafEdOeHYJkEczm2CKPE53XDDDWqYhs+J+6FAchxHsLRY8vLuu+\/Kf\/7zH7WcwXEctUnCqlWrhOvsHuw4TqAKTFYr2wJZRC0WyYnF12vekB8ndZABV6bn+F0HfjSGTnxHTvS7e\/fuamtx0p8QIT5q1Cjp2LGjIh8iwCnjttj6LQLJiABDvUTply\/JiQDMyZMny4oVK1RA5TPPPCMM2QAVhzlHKxYBi0ByI+A5OeH8JiwA4Rzf0rhx44RYJZzfZ555phQqVEh69OghOLiT+3XY3lkELAIaAU\/ICQf3a6+9Jjg+yXB59dVXC3LppZeqoRuJ45ilI5TgvPPOk1tuuUWwopht0w23R4uALxCwjXANAePkhGX0wAMPCHFMr776qgRHgbNshchuSOi2224TYpm++uorYS+7gwcPqkwFriFhK7YIWAR8hYBxciKzAE7trl27qrVvWdEgaJLobnZhYUPNZ599Vho3bqyIKtxMXdY67GeLgEUg8REwTk5YQTi1CZgkXEAv7NVHhnZkEiArJrEY8+bNk5UrV8rgwYMj5hBP\/Fdhe2ARsAgEI2CcnLRysgSQLYDFuSzqnTBhgsr3vXv3bpk5c6bUr19fICz8Um3btlWLeCE0\/XzIo71oEbAIJA0CnpETqVF27dqldkkhuRzDudNOO01IEkeUN+Q0d+5c0UJupuyGdcuXL1eEBun1799f8F2ReI4FwazRo07yPeGQT5o3aDtiEUhSBDwjpwIFCgjLVPRaOMIEsJ5YhgJpEemN72nDhg1SpEgRFUbAkDDce2BxMPFQs2bNUptqEoLAZ4aINtlcONTsdYuAfxHwjJxI\/Na+fXtlOTVo0EBZPFhMRIdXrVpV+vXrJ0OHDlVEQ6Q44QTEQYWDkrxNEBy7rTiOo4aFBHOyZAVLimEkOpM12Vw4XOz1UAjYa4mAgGfkBDjMwpFad8CAAUJE+Jw5c4QFuqylu+mmmyTSQl+eD5aKFSsK\/iqEYRtOdAhry5YtQtoKXZbgzh07duiP9mgRSAgEflnxnOybMzSi\/DDhBnFTTAPlKTlNmTJFGLpNmjRJsIqIbSJ04OGHHxYyCzz44INq95Rly5YJPid8UuEAIhcOG2kyA0gmgj179sg555yjfFiRks0xI2glLUcr1S1u2ePWO72UDKlfXJ5seqq80rKgknfbnijbOjgxycaxPWTzzGER5Ys3Z8iqBS+4JuF+e25d94ycWJry0UcfyfDhw6Vo0aIC8fz2229CHicc38zgkcmS3EtEjDNTh0UUDgisLBzeZC4gb3jLli1V6EHlypWjSjbn5mp82uxm\/bZubzMJfPTs\/bJuRFdZ1aWikiU1t4uWzmX3SfdqRaRtw1pSv\/W1SurOOyJpLx6LSXgmGrly4e\/ihtyxpQxfY6PiGTlBNDjFTznllECHcXwzW0eUOEGaLGnRgs8JQgsUznLCcI5kdDjTSVhHyhSWxmCBkWCOmTuE8+rVq2d52t2PS5cudVdBZu0mErRlqhETevys448fdghDLIZP3w5poqwfzrl2YskUQYKJp8y4LVJh4nYpO2S5lOw7VQk4xlNM4BXP9kZbl2fkVKJECcGBzdQ+ZEJKXYYJkBH+ImbrggVLivLhOkYmzCuuuEKlVsGXxWesJ3JDZZdsLlyd9rpFAAQgJPw9kNHXfVKV3+dIJkkVrNo4YP1AQG6RD23Ii+IZOTmOI7feeqvKPsCQjPV2xDsNyHSO58+fP+Z34TiOkEiO2bn3339fBg4cKESiO44TMdlczIp8+gA+NxNNM6HHDzqyEpK2jLCKICIsoWIdB5uAPFsdJvDKthEuFDBOTgzn8B+RIoVhFtYN8Uik1SWUACuKMi701VaZ2AgYaT0kpC0kzvUwDULCMjLSCKtEIWCcnPAb4T9i+MYwjCUqxDshzZs3F3I5HTx4UDXO\/okeAVN+BxN6TOvI+GSFmoJnBo3hW6IRkgm8ov8mxq+kcXLCb4T\/aNGiRQIZsc8cM2zIyy+\/rHrGcEyd2D9RI2DKtDehx5QOiAgfEpbS75kEhWWUiBaSCbyi\/iLGsaBxctJtZ0jHEA6y0tfYd45r3NPX7NEiEC8EtIUEIWElMWyDjLQf6dTG3eKlytYTBwRySk65Vk3YwKFDh1Rk+LfffivI888\/r\/anY5lJrhX8VcGxY8fUZgl24e9fgOSxA9YRlhFkxBELCRLCoQ0x5TE4Eqq7npETUdsEYDJTR1R3hw4d5JtvvhGiwklIFy8U88rCX1N+BxN6cqqDeCMEEoKMEKwjfEgM2bSFxCzb7lMqxusr5nk9OcXL84Zn0wDj5MSwjaUltIuo8EGDBgnxTO+884489NBDwgJd7ukynOdGiBa3C39zg6D3z0IwWEAQDwL56KEZBKQFq4jWQkYQEYJ1BDFhLXHPSuIgYJyciGFasGCB9OrVS4hHIqe4hotzAjC5Rxl9PTdHFv9GWvg7IOUHNVPDl94tOTZnQI508COMVk6e1l2iLZubctHogTi0hDpqMgl3PDYwTUVe6\/sQEwRF4CMC+RQ8t7GKtoaAtGgigowkm\/+SyYmcTH0Jfm3GyYmZOLZ5uu++++T111+XJk2aBBadkqIXC4qATMoENzQ35wwhwz1f+qQ\/1ELJX3Z+Jm7JoT3bclT39iNFJFHknb2FRMurJ18ikaRW+WUSi7QqNV2Qrqc+LHeVGyWPnn6r\/NrqfvmxYiO1nIZhjV8FV4Vf2xZtu1i5Ee734+Z14+SkO1OuXDmVr4lhl164SnT33XffrTYz0OVye0xJSYm48JcFjSyUTBuxWtySAre+lKO66z30qkQrZW8aG3XZaOsMVS6cng6P\/1e03Dp2ikSSvY82kUiy7paU4+53rlNekLPKFpV9h\/PLJz+J1Hxix3Fy5cw9ctvr+2XOFkewJLITvlfZlYnHfaz2eNQTqQ63+8LvEx2mxTNyMtVR9r5jsS\/hCQjnprSA+QoAABAASURBVBf+mugrX95k0ZO1L2yhjUzoVEXm962hJCu5dUovI+WLF5CRi7ZL8f7LlVQftjrzc+jcXVl1mMDOLR1J0ZcQ4CQ9OdmFvyHeehJe0uQFaW24t45AZBCWJquRi3bI13szkrDnydulpCcnx8kbC3+T9ysae88qZFpQndJLC4QFWUFUX+\/9XbCk2kxYH3uF9glPEPCUnFgAfOeddwpxTvibxo8fL8zYeYJEgivFuWmiCyb0xFsHRAVBze9TQ77el6FIau6qzSbgMqIj3ngZaXQUSjwjJ4IvR44cqVKm0E4ch0eOHFGZMLnHNSvRIxBXv0MEtab0RGhCjm\/Vr1xUGPJ1yvRP9Xx5T47rsQ+aQcAzctq7d6\/8+eefcskll6g832TFJHwApzX3zHTfavEjAm4TIMM9LCkc57M\/SHySchsvr74jnpFTwYIFhX3oyBuuO88wD2IiUFNfs8foEDBl2pvQY0JH7RK\/K6d539mbZdXW\/dGB7NNSJvDyouuekROLe9mPrm\/fvsL2TezCQo6na665RtjgICdg2B1\/c4Ja3n2mU6bTHId5m4nrE56gvHmL7mr1jJzoFnvU\/fe\/\/5WxY8eqFLucc417scrOnTuFjJqzZs2S9957T84880z1Oa8s\/DVl2pvQY1pH\/UpF5V8vJK6D3AResf4e41HeU3LC0hk8eLCULl1aZcAkZS\/71uWkY6yhIzdUmTJlxHEcYZuozZs3C7OAduFvThDNO8+M71RFiIFiiJd3eu3\/nnpGTuxFx\/ZNDOu++uor5RhfsmSJYOngd4oVury+468pv4MJPaZ1EBdFmAHO8VUJ6H8ygVesv8d4lPeMnEg0V7hwYSH75Zo1awTrBif5SSedFNU25GyQ0KhRI7VoGH8Vpm1OdvwFRBY28oLdEhOLP7\/55hsji2CTpS9Z+1HuhH1Ss1wBwf8Uz+9BVj3xrFvX5bYOfh\/8TkyLZ+REJszDhw\/LbbfdJqtXrxYW6JJDnGs4y7MDolSpUkJ6FRYljhkzRhEaQzlykbOYmCEis36VK1eWL7\/8MlAd51WqVAl85oQ6IDe3hBgut+rW9TZo0CCqBa+6fE6PJvR4pWPxHXUEK4oFxDnFJ+tzyfDu+X3wOzEtnpETaUyGDRsmnTp1klGjRslpp52m+k66FCwo9SGGP37e8TeGbtiiHiMw\/toqsurL\/cIQz+Om5Hn1npETyENCF1xwgZx++umSkZEh\/IsJURHvxP1YhB1+2WqqY8eO0rhxY+Ez1lNeWfiLiR8LXjkta0KPlzqIIie8AOd4ovifYsQrp6\/e+HOekRNLVLCcWrduLc2aNZOrrrpKyCPOzF3RokVjBsJxHBWOwOwcGTYHDhwoJLZzHEe6dOkiq1atUjN3119\/vZrNi1mBfSBPIUB4Af6nRCGoZHw5npHTvn37hFk59q+744475Mknn5SZM2cqC8pxnGTE2tU+4etwVcFflZvQ4wcd5I3CgiL+ye8EZQKvv16\/0YNn5BTcSxzX3333nZAd88cff1SkFXzfnlsEvEKgQrECagbP7wTlFT5u6vWMnEqUKKGCL4cPH65mmaZMmaIixdm\/Dl+Um532Ud1xa4opv4MJPX7SgQXFMheGeH5NWGcCr7h9UWOoyDNychxHyOV01113qRin3r17C3FPo0ePlmhCCWLooy1qEcgVAmQwINXK7A92Cwnr\/EpSueqkDx\/2jJzAgpQpn3\/+uSxcuFAlmTvrrLPUImD2tuN+JMGhfs8998jGjRsDxVgO07BhQ6lbt65cd911ahfhvLLjrym\/gwk9ftRB\/BMENahFqmiSgqgCXz4PT0zg5UX3PCMnCIilK48++qiwHZQWosVJOhcJDJa5MAOHM12Xw7k+depUmTZtmqqvefPm8sQTT6jlMIsXLxbKvvHGG7Js2TIhSFM\/Z48WgVgQGNQiRSWsw1neKb2MsqTIC4VAVlhVsdQXbdm8WM4zciKpHMM4fE0PP\/ywaBk0aFAgIDPcCyEuivQqrKcLLsOSmP3796tLxE0xPISIWBpTrFgxNVxMT0+XtWvXqjKm\/pBQz21dpvwOJvQkio5gomLoB1m9s3Wf2vkFskJqPrFDERi5y4mdQuJNYCbwcvv7G6p+z8gJ4sDxnZOAyxo1aggbcOJU150iBxS5yMkHValSJZk9e7aKPt+6dauwhECX496OHTv0R3Vk7ZCbghI366duCJCj22JCTyLquOeaujK5d1PZ9Eh7KfrqjcfJ9+\/OkbWvTZcX31ipZPQrHxxHYJBYNFKi14sSStJHrg1cL9Vlkrghv9YfyNfYqBgnpwMHDgi5wx988EGBmNq2bSs333yzsJkmwj3KBKPAwl5+dCz0ZcFv8D19zvqf6dOnC0M31s\/xjF5zx1IZXS7rkeesbBOLgXsYfPv6WEG+n95LkB8ndThu01CGiNkJltmAK9MlO+lwWSNxS7L+dtz+bJycChUqJN27d5fbb79diBDHfwRR8RnhHmWCOw7J8ONZuXJl2N2AuU+GgwoVKqgI8Fq1aslPP\/2kwhUgK10f51kX\/up70R5tOYuAaQQ6pZdWW10NyvR5RRJIzC0x3Wfj5ETecHxD999\/v+A3wiHuOI4iHTINkDCOMrECwVo6LDGCOXl23bp1Ql1YW+vXrxd8XAjn1atXp4hnQiwXC54Zvrz55puetcMqNo8ACRBxPzCbzCJ3JobMtyIxNBonJ14GllKdOnWEYRjZCHCGExqQG8gIQ+jWrZsKISCUYMaMGdK\/f3+pXbu2QFCk\/23RooU0adJEWAycG125fZbQiV69egkpYljzB2nmtk77fGIgwL6MzFKTTpp\/oPG7JkbLzbfSODnxQ2TY1r59e0lJSZGbbrpJWTVZ\/UzZQcFM3+TJk+X8889XRR3HERYRv\/XWW2qBL1k2U1NT1RCPsANIgEXBflj4i4VH28hpBRasM1SdsH+SC4EQveH7n9t\/iENUm5SXjJMTKObLd7xatofKKy+Mfh49elQKFCgAFELfSVmsPtg\/SY8AWSsff\/xxadeunWDdEySc9J3OYQfz5fA5+1gIBPBzYarrWwSTMvtIbBVZOnH+408LTkVMSAVxW\/oZe0xMBKJ59xAROcvYJQhi2rBhg7BrUGL22P1We0JOBEZWq1ZN5f\/GJ8RWThyzCxdwH46caSCTAn4zfAlYQroWfEv8S8lw8oUXXhACTgmYIybriy++UNkXKI\/fTT9jj4mFQKzv\/tNPP1WuBsJbcE04jpNYHTbYWuPkxIwcIQFM\/YcS7lEm7hi4WCHWEKELV1555XFa6EurVq1U0jvSwZQtW1ZYS4hjnvxVJNeDlPmSHveg\/ZAwCMT67vmHiEBhZmoJewkOEE6YThtqqHFyMtQvo2pYGkPEes2aNQN6mZXB0a2j2B3HUZYi22BBVFhSS5cuVZHugYfsScIhEOu7Z1hH+AgTN8T0OY61nMK9dEtO4ZCJw3X+VcW\/FIeqbBUJhoB997l\/YZacco9hyBoIEWC4RsAlBXCG4m86++yz+WjFtwjkvmH23eceQ2qw5AQKLojjOCrgE78Ts3a7du0SotfZR88FdbZKHyHgOPbdx+N1WHKKB4ph6mBrKnwShBGwXOHGG29UW1aFKW4vJxEC9t3n\/mVacso9hoEa2OKKRcr6AltTkZ+KrapwgLJ0Rt+zx+RCwL77+L9Pn5NT\/Dtsa7QIWAQSAwFLTonxnmwrLQJ5DgFLTnnuldsOWwQSAwFLTonxnmwrIyNg7yYhApackvCl2i5ZBJIBAUtOyfAWbR8sAkmIgCWnJHyptksWgWRAwA1ySgZcbB8sAhYBjxGw5OTxC7DqLQIWgdAIWHIKjUueu8o+fxdddJE0bNgwIA899FDC40B0\/tSpU4UUNiwhIhtpcKfo96RJk4IvBc5ZqD1u3DhhbWTgoj0xhoAlJ2NQ+18R2TxZZqPlnnvuCTSarArkPg9ciMOJ21VASK+++mqOc2axCQUpb8jU6nZbbf1\/R8CS098xsVeCEMCq6NOnj9rZZvXq1bJp0yZp27atsJiZ\/dewLii+fPlyYUsuLC+25BoyZIh8\/\/33aqsujpTBasFSgejIoU1Z6mH\/NoiE+9TJbjkXX3yx9OjRQ+3Mg+UyatQo4RoJ\/cgiCun069dP2DCCurFwEM61kNebHO0k99PXwh2xEmkPUqNGDaENBw8eFJLDsYUXey2Ge9ZedwcBS07u4JqQtfbs2VNl6ySX+wUXXCAbN25U\/di7d6\/Mnj1byEU1fvx4mTBhgrDVFuTApqhbt24VkvbPmTNHSBEDGUA26uEQf3iWzSXJBPr2228L+Y\/mzZunSrJtFhYcZbBa1q5dK+RiZyOAFStWCATGUI1NIfbs2SO7d+9WBMZmAaS+VZX89YfyVatW\/evT\/w7BfaSfEA93sBKxGOknW5aRD5520Rf6T7obylkxh4AlJ3NY+17TU089JTqv+0cffRTYE7Bx48aCBbJjxw7hB4x1wQ+bXYs\/\/vhjZU2dccYZwg\/ZcRxhhT6J9sJ1GHKBBCC7SpUqKbJjJ2bKY7WQV5uMDuxJyFCSDSIgHjagJAUNPiSsNDZS3bBhg2zZskXQlzVXFnvEcZ16tQT3kb5eddVV+paw4evQoUOFDVjxv3EDgmQbL7t9F2iYlQA5mVVrtSUiAgxt2OYKQuKHzQ4y8+fPV8TAUE336fDhwxGdyAyXBg4cGCBCrKghmcNA\/XzWI8M6divR1\/fv36+IpGnTpspSY0hZp04dgbx0mViPtP\/555+XkiVLyg033KB2SIm1Dls+vghYcoovnkldW2pqqhw4cEAgJX7Mr732mjzyyCMqgR7DLrJ9ch3rCiLLnz+\/2nkmIyNDuI4PCICwSrCGsGywVgYPHiwM77gXSiBEhouQFFtt3XLLLcLmEdWrV1d+LXxhWHNZny1durSapct6PdRnho70h+EcVpsug+VG24sUKaIv2aMhBCw5GQI6GdQwbBswYIDg1K5Xr56wg0yvXr2E4RXT9AyRGjVqJJAUwymGYHxmCyx2p8E3BWEx7INwODJkZB8\/zsNhdPnllytLBud0x44dpX379sLQD8LQ+x\/yOevz6MBvBrlkvRf8GdLDavr6668FhzxExxH\/F34t+oJFFfyMPXcfAUtO7mOcEBrI4BmKICAfRHeibt26giMby2fWrFnCXnyO4wh79q1Zs0b5pBgWQQiO4wjbH+FjWrx4sTz99NMyYsQIZU1h\/XAdYYiHtYJ+2qF1oZdrkMPw4cOFKX30ostxHDVTB3m0adNGkZd+Th+Z2cPCwtqiDtpLffo+R\/TRFu7h98LqQyBeCIm+svdgboaM6LESOwKWnGLHzD7hAwQIT4CksMTY0DRUkyCkzp07Cz4pyDJUmUjXCJP47bffBOsvUjl7zx0E8rlTra01LyPALBuzXm5iUKpUKVmwYIGyxCJZNcz+EbPkOLFvXomPjSEsVp2bfbF1h0bAklNoXOxVi4BFIFcI5P5hS051LZGKAAAAbElEQVS5x9DWYBGwCLiAgCUnF0C1VVoELAK5R8CSU+4xtDVYBCwCLiBgyckFUP1XpW2RRSDxELDklHjvzLbYIpAnELDklCdes+2kRSDxELDklHjvzLbYIuAWAr6q15KTr16HbYxFwCKgEfg\/AAAA\/\/\/\/LEATAAAABklEQVQDACU9nca71dFzAAAAAElFTkSuQmCC","height":178,"width":295}}
%---
%[output:9be51af7]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:4721b059]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:5d9ad9f7]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:1ef39550]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:3a544745]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAScAAACyCAYAAAAAhgkFAAAQAElEQVR4AezdC7yu1ZwH8PWemamGmTrGRI3knBAVOk2GIuYk5RhFiHKZcTIoJULlUqmIQkhNLuPSCaFcMi7RzZwIFSUqFKEyRmg0QzQfyuzv0\/nvWefZz3t\/373fvc86n\/Pbaz1r\/dda\/\/Vf6\/k\/\/3V9F\/2x\/CsSKBIoEphACSxK5V+RQJFAkcAESqAopwlslMJSkUCRQEpFOZVe0JMEClGRwGxLoCin2ZZ4Ka9IoEigJwkU5dSTmApRkUCRwGxLoCin2ZZ4Ka9IYCFLYIR1K8pphMIsWRUJFAmMTgJFOY1OliWnIoEigRFKoCinEQqzZFUkUCQwOgkU5TQ6WU5eToWjIoF5LIE5UU6HHHJI2mKLLdrimc98ZvrNb34zUrG+613vqsr7+7\/\/+\/Tzn\/98qLyvvPLKtO2221b5qUcTv\/U6nn\/++UOVOYrEo5RBL\/yMozxyJPMco2jTXuqTtzs+ekmDJuSAZ\/1GPsJzRH9p6ks5XZNffvKVfz98NeVVD\/Me4kne73\/\/+9fq98LqGGX5c6Kc6gKoP1988cXpCU94wtBKpJ7vuJ7\/4z\/+I\/32t7+dzl6D\/vSnP51+nmTPODv2qOvtBX7BC14wI9sbb7wx7bDDDokSmBE5xwH6wpe+9KVpLn7961+nr3zlK9PPk+750Y9+lK6++up073vfO933vvftyq72GaQdtC1FRxGSmYLmVDnpUN\/+9rfTD3\/4w2n867\/+K76SDvfJT36y8k\/6H7yqR\/AZDbrZZpulv\/zLv4zgOXf333\/\/Ss4XXnhhusc97jHn\/PTDACV63nnnVUkOO+ywqh76jQ+ZF0fEO9\/5zoSOfxx48IMfnL71rW9VZT\/2sY\/tqQj8QU5MWcULmIfPnb99yaeddlqiUJ\/xjGekv\/qrv5om9J6Sf0D\/9z4j+MhHPjISw2JOlZOK1HHPe96z7Qsd2pWGBaZsU2cUJg4NSFcvJ56ZoWgC\/Q4RnvKUp1RZ\/eAHP6hcf2666aaqQbfaaiuPM1AvU9lNPNbp0PgqoQ8+87qiRyM+gD4Y4BceaT0\/6UlPqnhF46sXX67IJ57FgzTyIF9lCwN+YeJAeuFNwCeaQPDTRBthrA0vCUUUMhdHyR555JG8VT3IvnpY8wcfUQ4Xj3hdEz3t1HmSLuoa\/EknvXzQTyfu4PnCF75QxeI7Xl6WiA9YFdHmj6kH5SoLH\/jhD3QqvxttXo\/Ir97O2MLD17\/+9ep9fOQjHymoLf7iL\/4i\/c3f\/E1jfL08MhSGWBnqGYYIRf6Qhzwkqd\/EKafohBi\/3\/3ux6nmnwgvKlAFTv3RWb1cGm\/qsfqvUsLEVQFTf6R705veNOVb+79G9ELmoaygXXfdtecv8Kabblo1Xv41jA655ZZb5llXfrzWyxSBR3WML6p61OnQNNVDekCPhj+AXpnxPA4Xr7Mlc+3jK53XgxUTX3B+ceRInnV56Bd4zWWCf7KTLiAd2cXzIK4Xz8st7d\/93d+l5z73ubyVEtXPq4ce\/uADPzkpfvM6RJzwJlp1RMNVf3LwHKAU6lMp5Eze22yzTVq6dGmQNrp5Xe91r3ulu9zlLhUdHuvlKVsYXiqiNn\/mVDkRCC0Z2purIfDKdI+O9qEPfSihFR7mJMHFlyjMeQJ63eteh6waI0uj00pTBWZ\/CCYaUVnoIk\/CO+644yqlmCVp9G6yySZp8eLFKeadvBTmm3wpfSHyROIoMWF4V55yg7\/4ovZTD3kFlBl1\/rd\/+7dKaYpTprL5cxjm5XT4+PCHP5x8BXO6Tv5+eB1G5r7cMUT2Auor4APTxN84+kxTOZ3CtK+XG82KFSuSvq6NPLdrE3F1qLd20le0b+TRNHwShwatNNLKz2S2PhAfziY6vOIZPVpp+CnVep\/I20A76M\/Sy\/fEE0+s+hDryLspD9YunoBfWLyrF05NM0SYfPDg3Z9T5YTBdgjB6\/z86FQA0\/yE9apXvap6ASkTXyKVIiDxTH0mP7800vIHYhhGmBEnTw2BJhQFfycYhvoq3nLLLcmQwsQ4ReXrIb88rWcvvwbies7j1eOmm25K\/dQjT29eIOrsS+eLl8ePw98Pr8PI3HzPW97ylhlV8IHxcoCvNIJx9Rl594NcEVBM2kZfkUev\/QstS179+eWhnfn1dfLnD4hD47mpD5xwwgnVnBmFEHRoA9FGhp149H7gPeK7uXjSJui8D\/o0Bfmc5zxHUAV+YWjr\/FcEa\/6MXDndfvvt1ZDoc5\/7XDr33HMri2JNWTOc0JJe1oCvN0KMs6JU0IsvLIZ5\/EAxsFr4Q6j8Ki6OP1BPG\/TKwYfODb4I0hCqsvm7Qd7ocwVp\/H3Xu961MamXSFmBKLNO3Es98jT4iGeKDw\/xPG63F16HlbmPTPST+KDk9dJfyFa7jaPP5GV181OQ+ZAuFAELStroL\/zdEGmCLm\/nCAs3j2vXB1iw0fe4hlj4iTy4+rIwyjR4Fx7wnkZbcCkZ75F41hKrKdpbPspQFvALQxs0\/HUMrJx+\/\/vfpw984ANpt912S3vssUf693\/\/9\/Tf\/\/3f6XnPe15S+EEHHZQMG5YvX57e+MY3JvT1wpueVRDEadz\/+q\/\/4u0JnSraUwYDEkWHUD7Ipt6hhIFhiJeIP4aTufktPPLgn3TMFa9hAXgxcvkZLv3ud7\/rWWzj4t\/L6sOHEZaElxLyDxFeDZ\/QzBYo7+CBgic\/w0AWUvCAJ7z54LByIryTSxHGqIPiodw60fcSN7ByOvPMM9MxxxyTmH\/XXnttOvjgg5Mx5Fe\/+tX09Kc\/PX3+85+vlNfOO++c3ve+9yX0vTBUp\/nrv\/7rak5HeL0j1b+QoSQIR5w0gXraCKcIdSSNVIcvddB1cpm9GpcyjUatW27S+5rEcrgvD+UtHK945odB6iHdOGCIaqjaLu9BeB1E5pS6l9vKDqsk56c+fPGlb7KopSHr3KoahH\/5dIKXO+ZrOtEZNnl\/OtGIq\/fdNc\/VlEZTP5OmCb\/85S9TTJH4MFLw6HKZeKaswLQA2QobBt4N+dXfL8\/xDtTz18YDKSfCN2xj8l166aXpqquuqjZN+kIY85pMfsADHpB22mmndMopp6THPe5x6Zxzzkm33nprnYcZzyoBIuRPQPL0LH8mKT8elOOlpuFNmIaSEE9RqiC\/NNLyB8KyURZEeKeXIGjqrpUJc0y+lPLyYnTrNNHB1KPekfupR52XUT3HS6tOlLd8yTM6t2foh9dhZJ7zU29LMgf8GMouWbIkjaPPyL8XUDgUD9qwTryIAYpBnL7bi4URwyRpfOA88\/fSz9A1IfqfuNNOO61aQeSHmCt79KMfXU1sC+uGvB\/H++id5NeH8jZjvfnQWDBSn3Z5D6ScfEl9UQ3Z7na3u6U\/+7M\/S3vvvXdVEQqp1WpNlyeOgrn++utnKCcdSufGaCBMTpUKk\/LZz352tQNYpuLRSie9sBe+8IXJhKEvpolwYQTiC41WGmE5WEU6jjDx6CCEqHPLT3w3MGm9FEFHqTal9RUiC3SGdsrL6yEcpFU+f7d6oBkVyMHyu44WykDewvFKnvgRFsDrbMg87wMhOzwB\/vDjCx0vfk4vHl0u67zPjFrWFA7Fg6dQyPyBeGk9U\/aUPn87yMtUiTpwPaMld\/Ln7wVGIfomWv1cfsAvDLS9EYD3D5\/CmhAylR5y2cYEvneSnKXP24xfWNDxB7zT8vJBHEg5RUa5u2jRotRq\/b9SyuP69XsJNLDKSevlt7oVykRYIB8eCaN0hPEHdFiI53CZtXVajWIOo525GWnrbt4J8xc7p1OPE6eWWb1EEa5OGiTCWFE6iPLrvKFtqkfkNYhLxjpJPW2THJVf50m6Jlp8gvgcg8qc7PSBpvLlr6wLL\/z\/ne9Bj2fxOeRBvhHGLyyeudLJk78faDtDe2m0qReNP0f+kaLsvYh5fN2PD\/xEePRRco+wXt2jjz56+kMvjXdN\/+N6Pv3006vTGfqEviGsH5CjNo40ZOt9wnOEcet0DJE6zSKEsw3Mh4nb5OqEOledr6Z0TQ0kLM+XgEBY3oHlX6d1PKFbo4hHJz\/pU0qJ6xmUJe8mOl86PKADdcrD8rrneQZtbo63K0M4yFu6yBNfnpWvTDQQdOKCVnhT+RGm\/uqHDiJcHqAs4K+XV6et5yW\/dqinlT8oqylNXjd0II86rTBxAen6kXXkp9+SoXzq9W6iQads5fFLK4+gDTfi0dTlpR2EiZNXpOFGushX3vxogV9f4Hr+5je\/WW01kE76HHk5aJtQL1\/6pnR1ujqN+DlRThgeNawGvuc970nMVmbmLrvsUq0g\/vGPfxx1UWPPzzjceFw9YpilUOExoa6eOpXwgsElQKZF1oPLb5wph1JOJriNl8GSP0XwP\/\/zP9WhP2EBdOOshLytBr7tbW9Lz3rWs6qVwoc\/\/OHpwAMPTBdddFGab\/98RZjV+GZyGxpQVDHfwPxlBosvGE4CRdbDyW+cqRcNk7mVOGNV2HfffavjHla7POdAN0w53dJSjMbKtjC85CUvSVYKTRY+7GEPSx\/\/+MfTH\/7wh25ZTFw8s7rJbBbGhPdSTRzT85Sh2ZR1ryJiFRsWau92Q9Ze85qvdAMpJ19uE2snn3xy6hXopRuHoK677rpqJ7pxaqt156T8n\/\/5n6eHPvSh6fLLL68suX7KtQ9mNvHjH\/84\/XhIXHHFFemKHtCpnKhzP7IqtEUC45LAQMrJi7\/zzjtXe5ucZO4F6KUbR0Vsa\/jTP\/3Tte6bUc4DH\/jAZJh58803e2zEMcccU20mzd23v\/3taTZhn8mwsCLSCzqVE3XOZeH8YuClL31pJat45uZhTgIEVq1alVatgX0zEMrzZz\/7WWNblMAigVwCAykn5+d+8YtfVBZJzCt1c9FLlxc+Kr\/ObhXCPo5R5VnyuVMCG2ywQQrY9Cc0nrl52G233ZbC+jL0vOSSS5KhyRlnnJHe\/e53V4qNMrOvyMcq8KhHPSrBjjvumMBkP\/zt3\/5tCph3g6233jqB62jMwxW0v+561LLR9rOJgZQTS2Svvfaq9kvkc0ud\/Oilm83KdStLvOXiZcuWpWU9whUpAemBcgQvZ6dhUz9x3\/ve99KoEYrDMQZ+ygUWL15cHRFaPOWyNsnClaxLlixJS9ZAnLp2g\/wAPTlFennC8uXL0\/Ip6CuelWdfGKCVxocGNtpooxS4+93vnsDOe7Aj\/z73uU8KWAABiwaw3377JXjNa16TAu94xzsS2HQYcNzKvE7gggsuqJbS43kQd5A8ek3Tja5TfLu4pvB6WLd2H0f8QMpJJzHRbCUJmOwmaC3J2lxlhezLX\/5yMtEoXEdk4ks3jkqYy3LY08Hjev6Ge3\/yJ39SD55+9oX\/1a9+lSBXBqtXr06rp\/DRj340gc2TwB8QD5GOLPpRQJ1oKbvANLNTHi8veLHBC+l4kNU7eMUrXpECVi\/BLmTgh5e97GWJG3QWEQJ2+nu57Q6WX0D8UUcdlYDfpkBxaJ13A\/xQA78+HQAAEABJREFUMBTLFKsd\/zcpMIpKHsuXL5+hwOQL6k7x1TMPWcXQUZ+EfBgbQ07WW6BuxdnwKCziuZFOHw7IO8riRvl1vsrz4BIYSDl52TfeeOPqHmr3Cp966qnVF84tBSalHeXwZdOBXfp1xx13pA9+8IPJLvLBWW2f0tfzf\/\/3f6shRU5FafjK6tB5eO6nMKMD63ABHQ6G7XTKDnjxYMWKFQm83BBKgsIIUCZuegD+QMRHmoMPPjitXLmyyk+e8g9EuXl9+e1Q5nZCJxrKwUdnyZIlSVnLly9Py6dAUanPy1\/+8mklRpEJE0eBgXTQqXxxwyow6eXTD1iV2lzbB6JPRD\/hUlgUV4BCA0NVp\/P5A0EjDYSCC1f+PqxRnvKhie9O7YK+U3y7uKbwpjD5zyYGUk45gywOh38f8YhHVKek8zh+Vo2Oy6pAK2zUsHXAdbmODdhrJX\/HCJjsthM4\/yesH8SLzfUCgpc\/4IULBcENpRFKhFKBeOYGDXqgVCDyVEZAuf3wO4m0ixcvTounQBGpl34AZAesMKgrMBa4NNCtXhSQMsgLPYQFtsMOO1RKkwt44AI\/Xvbcc8\/UVf7dmGiIp1wCoXQoIaDcclBYocC4odQoukCEcdGAdBBKjit\/iDK5+GhgceKDhlZOlIF9RJRBu9qKtxGT245mmHBW3FOf+tTqapb3vve91WV3VpLMJ5nrarXu3F7QVIbrIsBhRHC2Dd7whjekwItf\/OIE9lEFWACUYiDmRgwvwcn0TjDs7RTfa9wg+fSSphtNp\/h2cU3hwny09CMyZAWzZp2Ih3\/6p39KwAo3fDXcpLyAVQxN7ZqHUWCweEpRcoEfjbk3c4WG\/34th8Ww\/fbbVwfZKU03X4C+EfBhgX\/+539OwCoM4G+77bZLFCRYpAFlDQMKJkDhACUEdUVHYVFeAQotlBz3aU97Wgo4mMvSO+KII1LA9MVrX\/vaxDXysRBBZsPwP0jaoZWTYZ2vkBWZyy67bAYP3\/nOd9LHPvax5JdIdL4ZBAMEOKZCyFYII7md4VZ6jj\/++OryO7dw2hgatwAEXd3VMfFodQiWLl1aXeY+btfq1CjKGCSfXtJ0o+kU3y6uKbwpLJRELh\/DyB2mLCHTBqwdsDkRWF9AmbDIKAofD9A3WVOLpxRTve3zZx9XoCjtmzMXCa76gbPPPjsBa\/yaa65JphHwCWGBrVy5Mh177LHprW99a3KtCej7wIoOsKIDdWsa\/4FxWHTqbOgaMPXhQ+igfcCWFLxy3cXmY9HLR0Deo8TQyskXh+Y1n7TPPvukJz\/5yenVr351BX4dhQKgoddff\/2heXd3lEl381h5ZmeddVb62te+ltzAqTOxpGwQNTmf0xX\/wpUABUQRUUjLp+bAQP\/zslNcocD4hVFegF466btJh6UFLBfbJMBLzHqxT8weMS4IE2fRBGIBZJNNNkkBZUMoIgouwDqDUGBcSo3iCHgOiAdpQB1zKENZgeChW53nKn4g5WS\/kqFLMM3EdiDVFb2+OrGa9d3vfjcRiGVbNEE\/qKvMk046KW244YZrZTEfj6\/4Wq1ViQEfBsmnlzTdaDrFt4trCm8KG1AUPSWjgIAyorwgV2CGkJQXeLHFUWAgDXQriPICyihXYpQVUGBAgYGwT33qU4kCQy8dNJVTl1coGC6lY3TinYNQcuGakqC8AqHUTGPUlZ0hbNBRdHOxDWgg5YTR3XffvbKSDOdssKQwVMLdNBTU97\/\/\/cRkZL1YuWsSdL9hlJyfXXr+85+\/VlLl2SXO5G+17pxfsht90OMra2VeHtY5CVBeQBF54ZevscIoK2CBAQUGwigvQG8IJH03wVFgQBEZQuZWGIVVV2CsMHS5EpO+Wzn9xFNyYJ5MXWDF1OqyIW8\/+YyCdiDlZAzqqk0Tf\/Yy7TA1F+CHDgy3fvKTn1R82W5QeUb0xz3ldhlTgASXZ0sxGV6a\/8rDTUh2O76C3k5aX6TZhHJHUd4g+fSSphtNp\/h2cU3hTWH60ChkM0geTfy0y8f8FJjMN5EPJo9N3D\/xiU+cnshvmsy3yZUS67YnjPIBCoxVRTnlSozlRYn52SwwpWESmyXmemqr5KAO7erWFJ6HeT88zzYGUk4Uz\/3vf\/9q561tBIRlgtI+JyssJqatGLCeDAGHrZS7mlhgkXc9P6sYGrmutOp07Z7tAs4nX4t\/dhYF2snZJHO7uN7D57YOwaeJfPABZ9l7T8A1wibzm\/aEscCA1cJ6g3Z9N8JZNmDl2UiCEjPBTVmBdxM+\/elPJ1tujHCuv\/76BPZYtVqtZMtN8J273o8oZzbdgZRTzmCr1Uo60wEHHFDdneQ8lUlp29+Zg6yrww8\/PLlhb1BF9ZnPfCYRuDJYSHn589UfX7Jh+R8kn17SdKPpFN8urim8KcxFgcPKZdD0Tfz0m9cgebDADAUpohhGmu8yZATDSGCFeRZHgUEoMem78ZpbYYwKMFw0jAwrjMsKEybOMFIZ3fIedfzQyqnOkD1HlvUt5TNDLe0bkrnnyVxVnT5\/ZoYyIQPSaGj3QVF4dp7n9OG30dNkuS9AhIVLmbH04rnJjfJm0\/UCjqK8QfLpJU03mk7x7eKawpvCtNEoZDNIHk389JvPIHn0moZSesxjHpMoKavkYD+ToR2lQqF84hOfSMBiYj2xosAIA8i3EygwVlgMJSmwUSxodSqzKW7kyikvhNJ4\/OMfn17\/+tcnE+it1p2T1TlN7rfnxfg44MAmi4mCcqtldBK\/\/OBieKYyBWasb98Joeb5mZA3rjfBl4fnfiZrwQ+HPuxaZDg5MjRkA3uyGAkWksC+K2CBQUzmU3gsMGAhsd7qVlj9OX+HxuUfm3IySe0EuPmnFStWJBN0Jg47VcQ+KDcABqwA1hUWxWUSnhX12c9+tpr3skt71MdXOvFZ4ooEFoIEKByKiEKqDyUprlBg\/BTYbNd5pMrJ72m5rcDKnclrSoQ14zI6fqt8\/VawrrAoLkrLUM0EOL+hpE2XdrP2e3ylX34KfZHAuiQBCgwosdmu99DKyUqauSKb11wM5soJZ9pYSa985SuTsa6VNkdDKJRxVdBGM2Nv2xmshhgOnnTSSelBD3rQuIos+RYJFAmMUQIDKSerblbfzPdYJjUHZOLN8qOfAXeFivkmc0Qsn1Hzb0nWJB0rKvL2y8I2Z\/q1UorJOShzUhFf3HkhgcJkkcC0BAZSTlbdDj744OQ82+abb54oJHNBJt+s1NU3Q06XVjxFAkUCRQI9SmAg5RR5G7r99re\/Ta5DibDiFgkUCRQJjEICAyknw6nPfe5zyeFAwzZ7mQyhTISbEG\/abzQKZkseRQJFAuuOBDopp45ScFzERW52grswzKVbdrmaEDc5buXO3hf7jzpmVCKLBIoEigQaJDCwcsrzcuuAO5zMO7mlwA7W9dZbL7Go7Cw1cW6XuGFgnm4++F2gZzPofOB1ofNo1\/Irp1aA\/aiD2ypZ7laLF3q9J71+bgpxl5t2cV23yyBH0S4jUU4hPFsFbBmw1+gb3\/hGNewzYW7i3Eqaq1WCdtJdNwW6ptfOdPNqk87vusCfqQT1dIGg814uHrQLWljB3EmAMnITA+PEWTwHix36H5ajkSqnnJkY9lnBw6xT2Jb7c5rZ8Nv8yXKrl0WzEyplasuDs00u3ArrjqJ1H5TbPOtpy\/PaEuj3adA2cdbLcSj9yFYVV+K40LDf8gt9swQGbRfn\/Owgb7Va1fXF3iHt01xK76FjU045C4Z9LCdXMuTh4\/Y7k\/f5z3++sZgzzzyz+u02Wx\/QMElZSb7KEuDVBL\/hg+eC0UhgmDZxl5fzXzgxTeBj4uiF54LhJDBMu9jraBrnmc98ZnIyZOutt66uIR6Oo5RmRTkNy2S\/6Q0jrR7arEno9fTz8Vrfeh3m2\/Oo2sTtE04BOBHgF3F8VOabLCaJ31G1i5HShz\/84WSO1qFjm6SHreeCVE6OrBx55JHpzW9+c\/Iz13Uh2UHuYDLl1Wq1quhyrW8lhrH9GUWbmHj1dXZTpvmnPfbYI7Vad7bf2Bhf4BkP2y433HBD9WMmMbw24mAYOMI2Q3R9BixI5eSWToeNDcscDq7LhGJyz1N9J7s5jF6u9a3nV567S2AUbWIovtNOOyV3F3kJupdaKLpJYNh2saeRNXv11Vcnc022E7nHzVxut7K7xS9I5dSt0i7cYoY2Ka5uaUv8eCTQrU38vNjll1+ebFUx72RuA04\/\/fTxMFRyrSTQrV185A899NDkvig\/KOreNiMSlwBUGQzxZ51UTv3Ii6CtYvSTptCOXgIbbLBBcge2O+ut\/gYsaIy+tJJjPxJwt5r5Jts6XABgZb7VGn64vU4qJ8ucTFEmab0RfAlsI6iHL7jnCatQaZMJa5A17Mxlu6yTymmYa33XtFlxRiyB0iYjFuiIspvLdlknlVO51ndEPXeE2ZQ2GaEwR5jVXLbLOqmcyrW+I+y9I8qqtMmIBDnibDbeeOM0V1dgr5PKSfvZxFeu9SWJyUFpk8lpi5yTuWqXBa2cbBewitC02uZ8liM15VrfvBuO31\/aZPwyHqSESWyXBa2cBmmkkqZIoEhgMiRQlNNktEOPXBSyIoF1RwJFOa07bV1qWiQwryRQlNO8aq7CbJHAuiOBopzWnbYuNV13JLAgalqU04JoxlKJIoGFJ4GinBZem5YaFQksCAn0pJz8vJPra\/2ail+\/cOEX17OrUv0M1IKQRqlEkUCRwMRIoKNy8rtzBxxwQHJbnt+icz2qC7\/8ygLXs9+rc3fLvvvum9zrPDE1mxxGCidFAkUCA0igUTm5DfKoo45KfunCrZFHH3108uOZ11xzTaKwAp79oOaxxx6b3Mvt2lTpBuCjJCkSKBIoElhLAjOUk9+W23vvvZPfajv\/\/POT36FyodfSpUuTIx95as9+WQU9OvTS5TTFXyRQJFAkMIgEZiinxYsXJ1ef+nGAe9\/73n3liV66vhIV4iKBIoFKAuXP2hKYoZxYQ\/WL\/9dOUp6KBIoEigTGL4EZyikv8uc\/\/3lymfwWW2yRumG77bZLLjo3H5XnUfxFAkUCRQKDSKCjcnJ\/8OGHH578zIsrFfzckslx2wj8NpW7tl1u\/vSnPz35yZ5zzjknmX8ahJGSpkigSKBIIJdAR+Xkhyavv\/765IcA\/GT3ySefnGwpeMELXpDck+SHDQ0D73vf+yZxX\/ziF9Nmm22W5z9v\/YXxIoEigbmVQEflRCmdffbZyW9RWZWrs7rllltWV3iiQet34HbZZZc6WXkuEigSKBLoWwIdlZOd4fYv+bmkdjmLQ4MWjWduQZFAkUCRwDAS6KicNtpoo+TnilevXp38zHC9IMdW\/JAeq+oud7lL+v3vf5+uu+66Oll5LhJYuBIoNRubBDoqp\/XXXz+96EUvqhSO3eJvfOMbk3kmOO2006rh3qWXXpr+8R\/\/Md1xxx3pec97XrVpc2zcloyLBIoE1hkJLOpW02XLliDsXcwAABAASURBVKVTTz01bbrppslZuoMOOijBMcccU1lTRxxxRFqxYkW67bbbkmMvthN0y7PEFwkUCRQJdJNAV+UkAwd\/zzrrrPStb30rOfQLl1xySfrmN7+Znv3sZ6dWq5XucY97JDR+0USagiKBIoEigWEk0JNyuv3229OVV16Z3vOe96QTTzwx2Zz5hz\/8Id1www3pj3\/84zDljzZtya1IoEhgwUigq3Iy6f2Sl7wkPelJT6r2Mn30ox9NP\/vZz9LVV1+dHvvYx6YjjzyymghP5V+RQJFAkcAIJdBVOTkE7LaB173udemMM85Ido0rf4cddkiHHHJI+uQnP5lWrVolqKBIoEigSGBkEljUKScbKx1J2WeffZLbLzfYYINpcsdZ9t9\/\/+Toitswf\/Ob30zHFU+RwGRLoHA3HyTQUTnZWGmD5SabbFJNetcr1Gq1kjgX0pV7nOrSKc9FAkUCw0igo3Kyz8n1KZ2sInF3vetdU9kZPkwzlLRFAkUCdQl0VE52iLsyxVzTZZddtlZaq3QXXnhhNQ9l\/ulud7vbWvGz8WDOq9NVLoailOcoebFque2221ZXyJiLGyZvq57k264Odf6jvvXwYXiYrbTvete7KpnldZ2tekTZZE3mvdY55I3nprTyEi5eGb3mG3Sj7EuRZ7j6Pfni7f3vf3+KPuu5CcP25Sh3lG5H5aQgP2Bgn5O5pZUrVyaVfvWrX53iRw023njjhKbVaiEfAONLYj+Wa150ovGVMr6c5zv\/JKO\/eEne9KY3eVwL6veQhzwkTeKLoc98\/etfn+b3xhtvTN\/+9rennyfd86Mf\/ahaUXc7rVtDuvHrppFBFGwocG2srbuV0098V+Vkde4d73hHcv2u+SVXpDhnZ7jnXidbC5yt66fQUdOy3HQcF90F7GZXjk5lRZF\/kvGUpzxlrR+POOywwyp25wv\/FbMNfyggEKVNon3cOa9vCbcSTBnwjwMWbpTL0rdZuJcy9Bmyz2m\/8IUv5I8T7Xe8zDagZzzjGcm7GszmbUAm3hvvj\/iPfOQj1R5G\/klAV+WESfc6eXmcqfve976Xvv\/971e\/xkLbbrjhhkgmDve85z2ntz3UmfOFqJu2TV9vXwJfhKBlwt9000317Krn3ERHz4wWVkUO8OeRj3zkNP8\/+MEPZuRw1VVXrWWq4xO\/OWF81fATaKpnkzyE5XnxSxv5cMmjm1KJF9oLAPKBBz\/4wemFL3whb7rllltSLld5ylsZgab6SYzPoOF6jnpHGmHi5Clv6TqBHL\/0pS9VJNtss00CDyypbunFK0d5yg1ePAMZyqsJ3Wj1J\/1KPoGoY54fHvBK+etHeVzdb9XdhZH1cM94jXK46iVvcVzPlLhnH6DcCu6VV2nbYYZycj7OAV9Dt16BXrp2hcxF+Fe+8pXky6Hs+93vfpwKOkDTEIOiFVcRTf0hfENCQp96rP77kqKLfKvAqT86oU2qeTi\/MA08RTLS\/3jSKZURGQvDmxcLxEfHCRouGvzygzo3yUOYODTALy1\/gDx23XXX6vRAhLVzbdo11Mjjw6JxLIqyEqdTy1PengMXX3xx8qKJjzD1wGc8cz031Vtcr8AnftHrA8CPJ5YGfy9o4oUM8V1PL7zOt7DoP1z9KW9zeWh3\/OmvngGPeKVUly5dKqgtpKPIEBgBuV2Ef5j27odXZbXDDOVk+4AzcxdddFEKfOYzn0mGb4Tnd+p8VT72sY9VYW7IRC9du0LGHa6BaG3aPaBjKNfwyE52fkJTB37hzFpgFQoTh4Zfeg3MH6awcozhhQW8LO985zurR\/nID\/gFDjpkaadc5QlN\/OMP8periU77UWB4P++882RXHerGNwTvOq3OSyZkgzDy8wKwhLwsxx13XDUXKb4Oh8KFofNyRfs0vaB4khdaclYX\/MQQULh4dHgPueMDP2jxp7xhELIPy4NS5JdnWIL83SAN3vGlLuokTdPwSRwatNJIi9ZktvpGuU10+qn6o0crDf9zn\/vcxDLiD1B40QZcspNevo6moe+lvSkxw+ToK\/LBg3etV16Dp3buDOVkgpsi0oHBy2VYp1P4mlBOlNZ3vvOd9IY3vCFh0tyTdO0Kmcvw6Ah5oxGkA8vBlw6tcTwTrBfSi+mZ8Amc33yF4zr8AcMRL43O9JznPCeCE78wDa\/RpiPaeLz8OkuAckSKLzzwB+phoQAinhXCGtHRWScRHm7TvjSdVqdEc8IJJ1TzXzqfOsewMi9XJ9b50esXFCJ\/HWRHvvVw9Yu6RrleTkBLzsrmV58YAkZZuQJ51ateNf0SalftK90g0E\/0e2nD8mB98AvTL\/QP\/m5gAeIdnbqY\/+Fv6hPi0IjPy\/MM9TYRliPaSDuQkbbywc5pOvnxpA+iibzkEX2v1\/aWvlde0XbCDOWUE2uod7\/73dVVvFbr\/KBBxJsY92MGGuBf\/uVf0lxaTjojBeBlDLB28EroXgSbRL2UwoyxCZsfKFgmLf9Pf\/rT6gdF+SEfEnquz2VFQ1JQuWXAL0yaoOHvB+plni86baTFK57juc6TcG1naBcKgBudTzxIt3jxYt4KFBQ6YNZXgVN\/gn+yxJN4QD8VXQ2fKWn+JlCQ2kUbSV+nkQ8FFeVQ6njL6aIdyDQvC\/85rXbVvnnafvzxckvz6Ec\/ulJ68uQXRgbqwd8N9Y9G1KEpXR6nvKY6kBG5B\/I+FnmG0vajJPV+g8Z7oS0C6hJtwhJlkUY7qKu4KE87yaPeBsLq6IXXepr6c0flFC+0\/U6t1lpbBap8Wq07d4ibIHfUpQqckD+ECtjxtfvFL37B2xN8GU3S9kQ8IiJfqOgw4foRCR213yLwbx6CFeJFN0yQZ92C0Xkpv5BTXg5FZsJTXnn4MH51USe8gBcl8mOxhr8XN16gXmj7oYmXWxoftXgx+YVBv7xKMywMg0M5RF\/RvqybyNsHidWnzVnuEd7J1SZhAVM66t+Jvpe4XnjtJZ+Oysmub7u\/mYmu4K1n6NoUv86CBm09fpKe8cjiwBPrSEPyQyhhfl+sJUuWJF9kz\/WXwFdbI4rLoZPoLF66OlgOOe24\/RSLrx6ezCnF0KJeF3zonLnC0PGFgzx8WfmBEvNcr59nwzc0Ocg4rDeu5zzesAOPERbWA\/mSc4Rzg3cvHkspaH1EclplaF9p+gVFbBqgWzofO7Td6ILnoIvnqEOEd3N\/+ctfpuDLB8awSRr1Vn9+0P\/AENTQUNgw6Le9lUUuvfCKths6Kif7I3bbbbfk11XMPeXWB7+wj3\/848kvrszFDvFOldNIgIaJS+HEF0L4hz70IVEVfBW9iB6Y4iwKaTx70Zmo\/ASvzvyBmCyVHm2E+3r46lr6ZSpH+Gy6Oq4OrEw8UFT8AfXCI\/AL1\/FZWl4gz0AmXHIDfjD0k7adhUXxUfZopQP+AHmRm2fKxssAnsmZvPnxbsjBHy9eyJ0iMx9KKYnXrvVyhPcCijf4oQQo3RyhuNGg7ZYnnvGOjuuZ34ePguXvF6HgpIu9TPwQFp0hKNkL6wZyiwl0bU6ug7Z3vaxOvNZpm547KicJ3Avut+pow4c\/\/OHTRxD4dYQ999wz7bfffo0Hg6WfDeiMvsJelECYwAQeJq6ve3QwCilovST4FIeGX+eMr7q80HpxdEzxAVZJTNbmefKjMSeHhn+2oIOptxfXvATeuZ5zHsisXsecVn2BTMhG2pAFupBbPpmLJkeTHKWFkJEyTGR7oUxu452chaML3oWLR0emZKusvP0jT+H9Il5u5ZBhPX28tMK90Cxu\/nYgb7zndUCbT\/Z77gY\/uZZ\/LOUHIX\/pKRkWXTve0UDefvLQB8hPHHmS6yDtLQ95UdrdeFVWL+iqnHSEo446Kq1evTq99KUvra5I0VFV8lOf+lTSGdD0Uths0+jcxtAEHmWzDLww8RyuMHHxzHqyXSJeXuHyMwTSATwHDNvq1oY4cyp5nsJmA+r7lre8Za2i8AICvfg6tjq2m3PSxuoabasekV4eQA7qrf6em6AMq37k2xQvz7wcvLPwcrlL51m4eM+Ap3q+nvEuvh+w0rzc0oR1xp\/Dy4cPYaY6TGnwt0Odl5CXl79dmnbhRx99dNL\/Ip6fQuAKc++adg0FI6wfaAfyjDT8wuKZG\/zn7e3DL1x8oBuvDB3yDvp27gzl5ECva3nrCczX+GGD448\/PmHc9gEdpdWaOVFeTzuuZ3zkZnfdn3f6nAfCrdMKy2n448UKWvnpDLFMn3cysojwoM\/j5VdHnr+61OPrz2jkjY9QGmjysqNMLtqAZ4jnqK985Bfh4SpL3jny9OjUV9k5TTu\/8qSpQ571NLlcgp6CE16nrefLAqvPOQVNuzzkKW\/xyiMPchGeI6dRd8PZSKOMnDb85ChPkCaXF78wcXU5RLrgBT\/8aIEfP1zP9hpypYuyw83LQdOEevnSCstp8SovcQHPwoNOmm68khneI4927gzlZC7JTz3ZeNk0Cd4uI+HopeMvKBIYpwTMkRmWgLmvKIs1AZ77mXtBXzCHEmgoeoZyspnSb9XZLfqoRz0qOfT7k5\/8JLGoGtInVtaPf\/zjig79e9\/73iayElYkMFIJsGBBpoaolBSYbhBm+DXI8E7agsmQwAzl1Gq10iMe8Yj02c9+tppjMvHnC7TVVlslyoc\/sNNOO6UHPvCB6TGPeUz6wAc+UNHbXT4ZVStcLGQJ1IcOMawIt9ehw6hlZLiibHy0G+qNusyFmt8M5RQVdWTFDvCvfe1ryZLl7rvvntZbb73qhzRZUq5NsUvZEPATn\/hEMvGMXrrIYz64VmiuuOKKdEUH+LWZJsyH+hUeiwTmqwTaKqeokGMqLCYTbRdccEF1At1Xwb6Nc889N73mNa9J2223XcqPtkTa2XTNd\/ldPcuYzHt7r\/zwQrvhaPDmRgWrkJ1gqbwJO++8c6qjia6etzIDq1atSqsyUJaQK8tQjMHz2N1SQJHABEigq3KaAB57YuHMM89Mb3vb29KznvWs5KYE+7AOPPDA6maFNIv\/QpHkbq5o+CmfAKs0RyitXKGFwgtF6GMR2HHHHROF7GbSgCXvrbfeOm255ZbT+9Io7IItijy2GFwGs\/gaVUUtCOXkF2Ls83A42Q+APuABD6h+7PNhD3tYsoPdMZuqtg1\/br755vSKV7xiBuzf6AQb8upYtmxZWlbDJptsUv1CDbeh+IGCHBUKbLDBBtXhVOcfA3e\/+92THci2f9znPvdJQFnbEGjD7Itf\/OLK4jWRDCzhOljJ9bD8uVN8u7im8KYwQsnLmk1\/Ez\/9lj9IHr2m6UbXKb5dXFN4PUybzDYWhHK67rrrkhsH7LFote7cd2Xu66EPfWi6\/PLLO149amdtXcl4XrlyZVrZAU0KjeVWhw1nAcPMgIWGCA83Txv55woSX7ny60fhseTCarNxkrUW1hmLLKwzYYaZaNBLN9ud0osx22WW8gaRwHjTLAjlRDGxJJwFzMVlJdENnayjPDz3P+5xj6ueu\/yLAAAQAElEQVSui3UWrg7Hc5pgZ3w72H+Tw56bHObqgFL83e9+l3KE5cNl\/YGNfgGWIasn4D4tSg7s1AdKzQ9OsJLAMQxygLzedT8lBBQSxUU5UVSUFuX1tKc9rZLTEUcckWwzEW9oale1a0ZyyDt\/Dn9TeFOYBZdIM9tuEz\/98jBIHr2m6UbXKb5dXFN4HmY6wPNsY0EoJy+VpWVnkPoVIOsDHJCtg0XWBDti28GqZY5zzjkn5TCMCth+kePTn\/50Crj6ApxVAkclAq6ngVarVZ1pdOjavBOwruySPvjgg9PBUzj22GOTA6cQVhtLjZVGkbHMpGGRkUM7+TkZ7\/54daOYwPyYIxoOVMeVzieddFIy\/2c5Ha655pqEV2DNLl26NHXDZptt1pWmWx4lvruce5WRYW27fjHO8J6Vky89C8BZrP\/8z\/+sLpebywvmRiUU9Vq8eHF1RcriKXdU+Q6Sj1sEAja2AksGvOgBigFYOPD2t789HXPMMRX4QTia1atXp9VTkIf85M9ao4woJUNXSoqyorQoMK4wigvQoe9UJx8IUA6LSvlAgbHAIKwwbsChaXHoDCcB3\/IA+QXkH+jEyzBxrKRh0ks7SB69pulG1ym+XVxTeFOYus0muionS\/RuurRdwJ08ztc59Oi6XnM6vvzdluvHXSEHD71wvs71sgz3Om1zsEfLJHrAIed2CJom10ucw5AqRwzNctcLH1iyZElasgaUJNTr0vn5zljKBygiL3Wu0CgLikubcYEioLxAGmlZUBQSxQUUFeUViovy8hz1Rase0t3JRee\/oWC4rDF8UkT4AzxRVkBxBUKhcQ01A55zBH248slBATYBDyw9bgBvTcB7HVFrFkn4e3V7TdONrlN8u7im8KawXusyKrquyslZOab6k5\/85PTWt761WhlSuB3jlrEdBNaQwuYKVqNYcbfccstaLOj4Vq16fWnWStzwQGG0QyiWcL2sOZYvX56W15Arr3jRuaH8ciUZYeIh0oayU1aUjccG9tcKIivw4oUCoxgoLFYY1zNFQXGho7zIEpRHcUEoL0qrrsDE4TcQikx6+azF1IAPdSWB1xz6Zw71akKuwMIfCq7u5sow\/KEsm9ygaXLreefPwUfdXbVqVVrVBnldw5\/LI\/fXZTdgE4wlWUflZMhjKd6k8etf\/\/rkl0NbrVbFiGXqU045pbpojvVBOVQRc\/DHxPGmm26azNGEFYf3r371q8l2AnMyndgy4ceMnU2wPPspz458UD8waQ6sQsrZ3qY4VvTEJz4xuYPL2TLQfibGXXwHZEVpd5IJxUUZ6ciUFyXlhaa4wMtiAcGCQSwOGPaDa5tZsvjTNqFA9Z98Ut+E\/mtf+9pkQh9M6MO+++5bbe0wsR8IZcxVl4CJ\/hzmHQOd6jfbcXUlkD+TcTuEcqm72qIdtE0ducLL\/XVluXPDxmJ76rzvsy2zjsrJZVpWwtxvY6d4nTlh4nTGpiFVnb7dsw79D\/\/wD5Xyu\/\/975\/22Wef5MXN6XMaL6IVIy8AGoeV99xzz+THGFzhwCR1KNQvxDz1qU+tJo3RtYMJP2lmE6Oa9O2UD1kAWdhmQUbgzh\/nvsIyY5WxbEKJsGq6Ka\/bbrut+iFMCwaxOBAT\/1\/84hers5b55L5Jfaum119\/fQL9pdVqJX3IxwNM6IOXgYVlYj9gcj9gkj9goj+HO7gCrLg6WHd1GKIGWHr83DrIqB3wG6A4w0+W7cByDFCo4W\/XT0cU3nc2pkag74RDJuionDDk7u1OmxjFoWu17rSo+uXHPTR2cnvJDCEpGF9tYa5vkd+1116bvECufPWVtjJ06qmnpje\/+c3TtyU458eigEWLFlWdnvKSV5rAfxThKNgaJJ96GsNAQ8IYdrJOQnmROwgL5YUWuvFP9hAWGCXGCoOwxCg0Q0hgkYHVTM8Q1gH6wOqpCf4cudWhrBzKzxEKIHdz5UGpeObWsXLlyrSyDXJFRnHGM0XXDrmSpFDjua5Q8+egqbvtygg+crcXBavu5BDwMerW3qOO76icfM0Mi3QKFlS9cNaNuAc96EEJbT2+2zNF4rKszTffPDFDHblww4EhpL0ulq3loSHk765olgDL6uUvf3lyMyI6O8TdhuDoylVXXZV8zQ3pzIkZllKg8inoXwIUF+iky9fMmUXnZnlRXCAsV2DolyxZUq2C9luqIXkolFA0uQKi3HLog4FQZuFScDkovzry+NwfeeRulFN3c2VZ9+e85\/6oW5Mb9a+7uVLN\/cuWLUvLGkDJ1NGLgqXMcoVnm02\/7TgsfUfl1Gq10vOf\/\/yqjMc\/\/vGVAqFBP\/jBDya3EQgjPDSsp4qwjz+33nprtbObUCmfSOouHs\/G5cx\/DUrRGL4FjbkTcaGMKE9Dl1brTgvOnhqrid12iEd+s+1S7KMoc5B8eknTjSbiFy9eXCmgJVOKSDtSYObAKCoKi+KixMD8l2fh4kE7hkUmvXy0O3fxmrxHIadOeejDTWhSGvpiE3JlWffXFVk850qv7s+VZO6vK9b8GZ3rmbk5Im9zhOEPHrjmDbm5UjVC8Rx19QMUnWQ4jriOykmBJsJYN+aEvvGNbyRbC0zmmgOiMGy4Yzmh7ReGjLYpsILytO5CNvlrPoICct+wYV9OY1KXAjN\/QTFRjoPsEJenCfHZhlsTtthii6EPog6STy9putF0im8XRxk5nMw6pqjgZS97WbKJE0zUetkM9bjxgsULxbX4kgNtgKUdMISsw6+B5KAEm8Byq0M\/mXRQsPjm5ggly\/oJfygdrpEGN1eqZOeZ0gLzaLNd\/67KCUMsFlsGvvvd7yYK6pJLLkmW6c371Ocv0PeKVquVKBS7uyONYZoxO8vHxCjlROC+xkFTd3UweZhUrMd1ezYZXvDDNF9kYGI9h+F7wBU+AfNWdbD4c+QT6bnfXGYdLL8msATbgYXYBEq6CWFBtnNZlk1gZTYhLM\/c7fY+TFJ8T8qJcnCGypCOMqF9LfEau1566aXTk9LDVIxF5j4mQ0UX1BvvuvJjmDxL2iKBcUsgf\/Hr\/iaFIaxJwQgzJO6EJoUmrEkBCmtSmv0oWHnkGLcs6\/l3VU7MO2a6S+T9DpeVM3eMX3bZZUmc\/TS9nCJ3GDYfOskvmDF\/Yf8L64y15IvnqmDxrgRZf\/31q2Vrz3UY+g2zQ7yeX3kuEphECYyTp7pSjWeKNMc4eWjKu6Nyssplc9yGG26YDOHcj2wPC4VgD4tNjzS+lbJumzApHfNUATdoYsh8kUvp7YFhctvYaZ5LHLibyHDNeT7PAc\/2YdnkZ8infOPsiOcaetqvY1XDc0GRQJHA\/JFAR+VkUtpqGNOR1UNZeaaQrKixWJihvWzCZP1QbgEKz1YCQznisl2AtdRq3bnaJgxMetvOYKITP8LA3eYUF+UEJsgpS3mKNxQ1FyGtPIQVFAkUCcwfCXRUTl50CslKmCrZFOnQr0PAESZ8ULiGw8qfLQCsMatyOSiYVquV9tprr2Si3M84uwuJQjOBafe3yXrgf9\/73pf8NBUaP1ttZUbaVmtthTcovyVdkUCRwOxJoKNysuvaEMtSowlrK3U2PDo+gkVhlBWrBa2wfkDZGYrZFe6IRR32X8jPVoWTTz65WlFy\/MLPJJv3MiEvHvgtRYtDg2cHlqUVP0qUvIoEigTGL4GOysnyPMvDBDXFYU+Ks3RuJDC8sxHTHhMT5mj7ZdeOcBPr7ZaxHaGIPJV\/9tlnV7u\/rRza+Mnainh+YeIoJrTSRHxxiwSKBOaXBDoqJ1VxVYohlFl7SsipcYrI3iJK4NBDD02sFrQFRQJFAkUCo5JAV+XUarXSbrvtluzKtf09dmqbJ2KlsFZYLaNiaNLyYdnFyuKk8bau8WMO8pWvfGXySzLbb7998qE0tTCxclhHGLMv0XlX7WJRy5zwKNqlq3LqJl8Ki0Vl\/qgb7XyKN1nvviG3I9iyMJ94X6i8uiJa3S666KLk3JepBSuywgrmTgKUkTvDbBNy1MWquRX8YTnqqpwoHbvB3bNkO0Edvl62Cbj0bFhmxpH+hBNOSPmGzyiDZidU9wepEwXragorlGjUx8Fhw1rPBaOTwKBt4mSCEwQsddtYXDLnSNXoOFu3cxq0XZyRtN2o1WpVvy3gHdI+w0qzq3LyAttjRDM686NQq2FPeMITqit7ba602uZYy7DMjDq9ned+\/bcpXweWHZFxzQoaJikryVcZvb1RhrOGD54LRiOBYdrENR76IE6cVPAxsefOc8FwEhimXVxjtN566yW\/MeA2Vgtdo9j43FE5OXRrXsl40lK+O5fsKaIpPVsRY1n5og0nmtGmPuuss5KVOvNihF7lnv2xHeL0009Pjsw4f2QTpz1UNmyW+58yQY3QO6o2cYGg7SIWYfx+n4\/KCNlc57IaVbtYJHN7iTlaB7PdaDCsMDsqJ0dCvMi0YKvVSvYy2eHtmhIFmxx3\/YWrLNAKmwTY20TZOFnedA+NVUbHZiivVuvODZpuQTCMm9T7nyZBrsPwMIo2MfHq62xDrvmnPfbYo+sVzMPwvC6kHbZdbrjhhuRm2hheG3EwDGyAHlZ+HZWTXeDuXLJLXEH8FJVzbZ6BxjT5xcryPAkwP2bYaVjmXF6dJ4pJ3epDUXMY3X4huJ5Xee5NAqNoE0PxnXbaqfp9Pi9BbyUXqk4SGLZdvPesWZuxzTU5YuZuKHO5ncrtJa6jcnL+DfNMPzcQ3HHHHck93uagMGFSmRXiRW+17rRAeil0rmns0aJUmxTXXPO2rpbfrU1c18OqPeOMM5J5J3MbYHg+WpmV3HIJdGsX7769jjZqu5Bu9913T0YkLhXM8xnE31E5KXi\/\/fZL9pe4GsWvmZgYt6PbcA4z9j\/pLAv1RSdoqxiDCLekGZ0EXJ3jx0DdH2apOmBBY3SllJwGkQCDxXyTbR0MF7+Y02oNb6x0VE4YNfOuIzhQa47JGPXEE09M\/DqM3xhzrKXVGp4Z5c0GrDgyRZmk9fIoZNsI6uHlebwSKG0yXvkOmvtctktX5aRS9jEZQxrnt1qt5If3\/IwTpWXi2RAJ3XxBuf9p8lqqtMnktQmO5rJduionk1yW4+0JMivfBFsMTCSrzGgxntxsHXCTAuWqfkoxdGWW2k5ACQsrmD0JlDaZPVn3U9JctktX5WQC0qqXfU1uvGyC\/U4mLPup9FzS2qtV7n+ayxaYWXZpk5kymYSQuWyXjsrJnAzl5P4mm6pMhDdBnP1PkyDMXnmwia\/c\/9SrtGaHrrTJ7Mi531Lmql06KicbK23C3HXXXZNrefut1FzTmwuzitC02uZ8lhsV7IC3HYL1Z\/PYXPO80MsfU5ssdLGNvX6T2C4dlZOJcBsVYxPm2CVUCigSKBIoElgjgY7KaaONNkp77713clTAruo1aYpTJFAkUCQwdgnMUE5W3ay+xaqcrei33nprWrFiRbIhM8JzF710Y+e2FFAkUCSwzkhghnKy6mb+JVblnGdyTSBGmwAABCJJREFU0JKCOu+881KE5y566dZIrThFAkUCRQJDS2CGcrLqZvWtaVWuXRh66YbmpmRQJFAkUCSwRgIzlNOa8MqxWud35GxQrALKnyKBIoEigVmSQKNyctuAeSTn6HbYYYe07bbbpgMOOCC5WG6W+CrFrAMSKFUsEugkgUbltGrVquS2wc0337y6LdJtg+abDj\/88OTAbCr\/igSKBIoExiyBGcrJrnAT3C718ksKxx9\/fHJnjpsHLrnkkjSKG+7GXKeSfZFAkcACkMAM5WSeya7wHXfcsfoBA3VstVrJ71E5JHvTTTcJKigS6CoB0wN257sA351f5i8jkY29L3rRi9LRRx+drrzyymrqwM+MRXzumvN0PW89j5ym+BeeBKaVU7eqLVq0qNzX3E1IJX4tCbjO2S2q7gRztY67gYLAfeCUkut3Iqy4RQK5BBblD8VfJDBKCZgiYPVQQG4U9SMSkb9rn13st+WWW0ZQcYsE1pJAUU5riaM8jEoChmh+aPHXv\/518sOr+ZDM9IC7tNwz7Qcz+i3z\/PPPT34ItQl5Of3mW+gnSwJtlZMd4eYIAuahdCrHVCIsXFsMbr\/99smqWeFmPBLoMddddtklHXbYYclVznvuuWc1txTDOpfm+7ECP1DQav3\/9c71Ppf3L\/NXUfQ222yT3C+Ww1YXltiSJUuqnzAL2uLOXwm0VU6nnHJKsscp4K5wJvohhxyyVrj4vfbaK918883zVwqF85FLwK\/2+AEMV9MYuhnaxbDu2muvrcpbtmxZ5cafep\/Tt4Ci82ONQecWUz\/9FZDPOeeck9xgaruL6z+CtrjzVwIzlJOvmxWU\/KvUzY9euvkrhsL5bEpg9erVaauttkqUTF7ugQcemC6++OIZuOCCC9L222+fk077WVSU2o033lhZZxThdGTxzGsJzFBOvm6+cvFV6sVFL928lkRhflYkYHrAfjkT5IZheaF+tNUZzTpcFcsCy2n5TTOceuqpyW2tflbezRnCCyZBAsPzMEM5DZ9lyaFIoL0E3Dpqbsm8UXuq3mJc58NqMnwc1W+l9VZyoZoNCRTlNBtSLmVMS8ANFuajHI2aDhzA4\/JDE+6srOOOOy6VaYUBhDjhSYpymvAGWkjs2ffkF2Gt0rkCetC6mWdyrMo8k2NV97rXvQbNqqSbYAkU5TTBjTM61iYjJ+cybTvZbrvthmLIwXTnPx1IN+\/kGunAueeem371q18NlX9JPBkSKMppMtphneDCL90Yzi1dunSo+l5zzTWJUvryl7+cnM876KCDUuDQQw9Nbm4dqoCSeCIkUJTTRDTDwmTCgV\/HVPbff\/+qglw\/1VXfh1Snq4izP+ilM19ljslh4na3sipPflny4p2nEijKaZ42XGG7SGAMEpioLItymqjmKMwUCRQJhAT+DwAA\/\/8CQGJeAAAABklEQVQDAEEIPpgkyAWFAAAAAElFTkSuQmCC","height":178,"width":295}}
%---
%[output:5c15d3a5]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAScAAACyCAYAAAAAhgkFAAAQAElEQVR4AeydC7yNVfrHn1eTUZpQKpcuR7pM\/VVKyrXSmOR2ahTRhWIIpYkuqlEoErqoIZeiUDQyFIOEpOQu10bRRSHMRDKjpNTfd03rzGvbe5+zz977fdfe5zmfz3Pe9a613vWu9Vt7P\/tZz3qeZxX7Wf8UAUVAEXAQgWKif4qAIqAIOIiAMicHJ0W7pAgoAiLKnPRTUCAEtJIiEDQCypyCRlzfpwgoAgVCQJlTgWDSSoqAIhA0AsqcgkZc36cIZDMCKRybMqcUgqlNKQKKQOoQUOaUOiy1JUVAEUghAsqcUgimNqUIKAKpQ0CZU+qwdK8l7ZEikMEIKHPK4MnTrisC2YyAMqdsnl0dmyKQwQgoc8rgyUtH19esWSMXX3yx1K1bVy655BKpVq2atGzZUr788ssCv27YsGEye\/bsAtWn3m9\/+9uD3vfwww\/Lf\/7zH9mzZ4888MADsm7dugK1pZUKi4CbzylzcnNeQu1V1apVZcaMGfLOO+\/IkiVLpHLlyuY+XZ264IILTPu8b\/78+eY1jz32mPz617+WRx99VM466yyTV5h\/P\/\/8s+zfv78wj+ozISOgzCnkCXD99fv27ZPdu3fL8ccfLz\/88IMMGDDASFZIV6TJg\/r37y\/Vq1eX3\/\/+9\/L++++bYcEYXnrpJSOB1alTR6xEZApj\/DviiCPk5ptvlg8++EA2bNggt956qyDN\/fvf\/5Zu3bpJjRo15KKLLpLHH3\/c9If8O+64w+TdeOON0r59e1Mf6a1z587StGlTWbhwocydO9f0o1atWtKkSRNZu3at\/POf\/5RrrrlGevbsKfQPCXHMmDFy+eWXm\/beeuutGL3U7CAQUOYUBMoZ9o5FixbJueeeK6eeeqr83\/\/9n\/zrX\/8yX+w33nhDNm3aJEg30M6dO2XSpElC\/ubNm42kxT3MjCFThyXZnDlz5N1335WSJUvK66+\/TlFcOuaYY6Rs2bIC47EVFy9eLKeddpphNLzvo48+kq1bt8rIkSONZEU50tZXX31lHxH6N378eDOWN998U2CUCxYsMMyPNqj4zTffyIUXXmj6V6FCBaHdWbNmycCBA01ff\/zxR6ophYCAMqcQQI\/\/yvBLkU5Wr14tn376qZFekCoeeughee+996RRo0Zy+OGHG6pfv74sXbpU5s2bZ\/KRen7zm98YKYRRwDBgDmeeeaZZGg4ZMkRWrFhBUcJUr149o5eaMGGCWerB9LZv3y7r168XpCHP86RixYpyzjnn5LV92WWXCf05+uij5d577zV1n3rqKXn55Zdl27Ztph4SIVKg53lSrFgx0\/fDDjtMYJB79+4VSPQvFASUOYUCe+a8lC\/qpZdear6k3333nWFKtvcs52BU9j7yikIbpgCTg2AovXr1iqx2yD0SD1IT0pMtHDVqlAwfPlxgdF27djUSHctG9EkwFeqRZhlK2k8o8zt16iRIVS1atJAOHTr4iw9KxxvPQRX1Ju0IKHNKO8SZ\/QIYAMuzEiVKGF0Tu2swJWj69OlGakHXRJq8r7\/+WljGMWp0QyyjYDQwNnQ7LO8oi0XUGzp0qJx99tlGz2Xrsdxq3LixoKyH2cDokNRYitE\/+vn5558LEp99xl537NhhJCh0TUhEtn+2XK9uIqDMyc15CbVXK1eulIYNGxo9U82aNc1yrnv37tK8eXOz3GGZB5100kly5ZVXSm5urslnedW2bVspX7686T\/LPhgXV5ZYMAbSptD3DwW6fR\/twnRQfvuqmHf36dPHLLvQCaETe+WVV4wCHIZX44CiHLODE044wf+YSaOrKlOmjCABXn\/99YbxrVq1yizzTIXM\/Jf1vVbmlPVTnNgA0dmgK+ILz9Y+ynEUyUgoLHlYplEOkSYPZtK7d2+jf5o8ebI8++yzAhOirEuXLkJdyNb394h6H374oVFI877ly5cL+q2jjjpKIJZy9Am9EDovpCSY0ODBg6Vv376CZAbjpP2nn37a6I1YDnbs2FEg3kX\/2FmkbfoHA4XBwQjHjRuXJ6GxA0h\/eIZ38m76wL1S8Agocwoec31jChHAgBNmVbt2bUGfhDmBldxS+BptKgQElDmFALq+MnUIVKlSRaZNm2aWnkhe7OqlrnVtKUwElDmlBn1tRRFQBFKMgDKnFAOqzSkCikBqEFDmlBoctRVFQBFIMQLKnFIMqDanCMRDQMsKjoAyp4JjpTUVAUUgQASUOQUItr5KEVAECo6Ac8yJIGN4thPKgmBnEGnyKCv40LSmIqAIZDICzjAnfKOwSm7durXxGCfI2MSJEwXCchfGhIHdlClTUoa3NqQIKALuIuAMc8I5FA\/4V199VQgShvc54SygSpUqCUzrb3\/7W57flruQas8UAUUgFQg4w5yIuYP\/FPF5CG6Gdzp+T7gmcOUe5oUjaSoGrm0oAoqA2wg4w5yIOEhIjTZt2pgwqjhnEiaVsBs4dT755JPC0s9tOLV3WYmADioUBJxhTsQBIqYzflIs7QgaRszpq6++2sSeJnIhwcJCQUlfqggoAoEj4AxzQipiaUd4CwKbcfLGr371KwMIyznuqWMy9J8ioAhkPQLOMKesR1oHqAgoAgkh4BRzIs40Qcs4ReOTTz4RrijEyaPMjEz\/KQKKQJFAwBnmxCkZRE0knOvvfvc7Ew2RK0HEyKOMOkViVnSQioAiIM4wp+LFiwuBwzAViEaUUUfnTBFQBIoGAs4wJ44DQjq69tprjSmBdVvBdYVTZAcNGiQcNVQ0pkVHmTwC2kKmI+AMczruuOOE0zRmzpwpMKMZM2aYoPe4tOBXB9AEzOeqpAgoAtmPgDPMyUKNGwvW4DArm8dxP+RRZvP0qggoAtmNgHPMqVSpUvL999+b0105PBEaO3asYJSpCvHs\/jDq6BQBPwIBMSf\/K+OnMbbs16+f4M7SsmVLad68uWzevFkeeeQRwUAz\/tOHlmK4OWbMGMENhnPKRo8eLfv37z+0ouYoAoqAUwg4w5xYtuGiAjpYinNQIkf9cJAihyeWKVOGIrF1zE0B\/r3\/\/vvCAYrostBj4a+3dOnSAjypVRQBRSBMBJxhTriqTJ061ZzSiqMv8ZssMKTnzZtnyqhj8wtyhRER7QDmxrIQM4Vly5YV5FGtowgoAiEi4AxzYieuffv28uCDD8r06dOFwxFPPfVUga644gpzaCKOwNRJBK+PP\/5YTjzxxLxHKleuLBs3bsy714RTCGhnFIE8BJxhTrZHFStWlN69ewsSDy4r0IIFCwQ3FgLP2XqJXNFjxarfrl07yY9gkJC\/HveQPy9WmnqQv5x7yJ8XK009yF\/OPeTPi5WmHuQv5x7y58VKUw\/yl3MP+fNipakH+cu5h\/x50dLUgfxl3Fvy50dLR6tn87hGe8afRx0oWl5kvr+OTVMHsvdcubfEfTyi3kMPPXTQZ5Q8iOfuvffeg8rI8xP1oGh5kfn+OjZNHUuxvkPpyneOOaV6oDk5OYKfnm2X9FlnnWVvZe7cuTJy5Mi4BIOE\/PW4h\/x5\/rS\/XepB\/nLuIX9erDT1IH8595A\/L1aaepC\/nHvInxcrTT3IXz5nzhyJzPOX+9PUg\/LL85fbNM9B9p4r95a4j0fR6tk8rvGeZQ6pA\/nrcW\/Jnx8tHa2ezeMa7Rl\/HnX4sEbmkX\/55ZfLeeedV+jPLnPobzdamvdA9CFoynrmVLNmTVmxYoVgIwWRrlq1atpxZuLT\/pIQX0Do5BBfH8irdQ4DgTnmS+Ixp5gPZVLBBRdcIJdeeqk0bNhQGjRoYHRZ5GXSGLSvmYtA\/5kb5Zhucw1V7bMwcwcSQs+dZE5r166VK6+80jAVtv47dOggGGMWBh\/P88zhCPPnzxd0V5zg4nleYZrSZxSBhBCAMY1fulV2PlnPUO3KpQ2Tmv\/xroTaKaqVnWNO2DsNHz5c7rzzTkE5jn6oadOmwlFRlBXViXJt3J999plrXXKqP5YxrexRM69fQ1qdZZhU7rMrxAUG5focOsec0Auxu8bRUMwqIXoxKyB0L2XkKbmFgPbmfwh8sXOvsHxDYvIzpv\/VEKlzQIK6\/ZV1\/ixNR0HAOeaEoSQGmVu3bs3r7rp164TDDfJzX2HZRoA6XFUaNWokLA\/VfSUPxpQmioJCPFHAkJZgTDwXizFRNviABMU1d8gKLqGR63PoHHOCAaEX6tWrlyxfvtwosm+\/\/Xbp2rWrwLhizSSnt\/zlL3+Rxx9\/XJYsWSKdOnWSZ555RhYtWqTuK7FA0\/ykEBi\/dJvcNn6dwGRQeiMtoV+Kx5h44cnHlBDqzP9kl3mWPKVDEXCOOdFFol7iC7dw4UJ54403jCIbew7KYhEHcebk5Mjpp59uqtDGjh07jGW5y+4r\/g84v7p8yJX+u7sVNg7V\/rLRKLBj9aP\/zM9k04Fl3EkHmE1BmJL5YPr+8QwMCgbnyz4kiWfEIZkpyAhc55Rgn51hTjCXli1bChEwIbb\/ObOOSJjonCijTqzxnXHGGUI0g6OOOkp++OEHIUAdMaGIaOCa+woMyf7a+j\/g\/JrygVX67+5W2Dgs75JjFNix+sF8TbntfEHRHetzmV\/+lM7nC58HloT51U11uS7rCojoMcccIyzLJkyYILm5ucYkn8iYEMs8dEhly5bNa40DOGFgmNbffffdefnr16+XFi1ayEcffWTCrKC\/QsGeVyFKgjb4FUklwRSjtff7JxZK32kbpEzxH4UP\/+QbysnTjUrL3TVKSLT6mvdZaLjEmsNUzknFw76WXvXLCj9S97+yIupYcV+J9s5vv\/1WoGhlqczj+xHla5P2LGckJ3blkHS4EhYFialChQoCXX\/99bJ69WqjFLeI4GdHpAJM69EzWcX3bbfdJt26dZOhQ4cK7bHUw2XFPkca8wR7z5U2+BVJJSGt+dubsN4Tlgn\/+k5kba+6Mrp9dfGXa7qSc3hEzmG65uiORucYCW34kl2y6KsjDsGBz2i0dx955JECRStLZR7fD\/oQNDnDnOzAOWGFnTmiCdi87du3G8YE47J5kVd+5QinMmLECKlbt67YumG5r\/j7h8iOsrR7g0pGEeovCy2d5Iv5ZU6yCX08AgGWh+ifgrKBcn0OnWNOhOlt06aNdO7c2TAZGE2rVq2MlTdLv4j5zLuFmWE60Lp16zy9FXqqk08+2Viah+W+wocNxrSyR03p3iAnr7+aUAQiEWhVvZxAkUaa6VKIR77ftftirnWI\/qAAZ8k2btw4IX44J7DggU1ZLOIZ7KGoSwRNCH0VSzsYVtDuK1\/u\/tEY4713YLt4ygGlZ6x+Z2o+y4ZM7bvL\/UZ6QgEPg+KHzW6csJOLBJ7Kvrs+h84xJ3bkkHgwprzhhhuMxMRRUeRRlsrJSVdbWAnfOnmbaR6JCbsWc6P\/FIECIMDnh88Mu3iYGvAIeSjNYVbcB0fhvck55sTSjV27iRMnCvTcc89JkyZNxNoqhQdVwd7Mh4hfPWrDmLhmI7mur8hkzJG2+RxFGwPMCqYVrSzRk+baKAAAEABJREFUPNfn0DnmhCKbpRi7cRA+duy+\/fTTT7J79+5E8Q+0Ph8oGBNMaWqb\/4UGDrQTAb3M9SVBQDCk5TXjl\/zPdSvaC5CgouUnmuf6HDrHnKIBvGnTJtmyZUu0oqh5K1asECIZYAtlTQzwt0v30VAwpqgd0kxFIAEEkI4SqJ61VZ1jTjAUa1yJ8ReE7okAcZygkt9MoJfq37+\/7N2711QN4mgoJCYUlrwQqYlresidVl1fEriDVOI9Qd+U+FOJP+H6HDrHnI499liZMmWKifuN8RfE2XVIQizt4k0BbiuDBw8WrMlZGlKXgxKsvgrH4VQfDQVjQmIikFhRYUzgqpQ+BLo3qBS38VbVy8ctz5ZCp5gT59PhdvLnP\/9ZNm7cKEhR0Icffii4qODIGw\/42bNnG4tZdvdsPeyfsPS199GOhkI641ckUXpv1QZp9MxSOe+EXx3ifoJRaKLtZVJ98Myk\/hamr2HNYY2y30m1iiWAOCq1OONn4+aC6wpUmLHxDI1zzY\/4flA3aHKKOSHlPP\/88\/KPf\/xDnn32WRk0aJAhbJ3q168v7ORZgGBadvkH4+KDRAQDQvqiVLf1uObnW4d0hnIwETqsVHlpOnqzXHpm2aiuKDDERNrTukXXfSXa3M+6q6YgQbHEs8Q9n+cJ6z3j4oLrChTleVOeqny+H7w3aCoW9AvjvQ9Dyr59+8oTTzxhIgwQmtfSddddJzjx2ufZycNQE+DwrUO6wn2lWrVqUqNGDRPHCQmK+FD409nnSEf61tmygl7ZykXHxIcFo7mCPpdN9fi1zabxuDiW7g1yBFWBJe6ndD7fOAnzGUy2z67PoTPMCUU2jIkrCm0swgmdYik\/I0wkKxgVRIA5GNSsWbOEsCvs3hHiFyJdtWrVQs8rVrpY7nY\/oBfofuDDU+iG9EFFoBAI1DmttHES5jM448vfFKKFzHnEGeaEAhtdE75wuJ3gfuIn8qiTKLTs8rH8S9a3DsU3jAk\/OdwLijpjYsmQ6Fxo\/dQhgAS14T\/Fk2rQ9Tl0hjn5UUafhJU4R5BbQpoqqBEmSz788rh6XmqOhsp9doXAmAqxU+IfmqYVgZQggAR1xxk7UtKWq404x5xYet15553CMVC1a9cWSxhRHn744YHiaKUlwrTyYtb+RV1iAgfIdX0FfVSKj4Drc+gcc4IpEWCOkCmNGzcWSyjLUW7Hhzv5UhSNrOdxsETpjbTU\/YB+CcaUfOvagiKgCBQUAeeYE+YC6JYIf1LQQdh6nApM1EyMLmFqxHfKz31l19UjBcnIEn5LOF76g9Z3V8W3hTjv6rq+Iq+jGZQIuquuz6FzzIkjnthtY3eOpVxBd+t+\/PFHGThwoDRr1syYEXTs2FFefPFFWbZsWdyjoUq\/1s7sfqDkhlb2qGm2b4uqiUDQXxB9nyIQCwHnmBNS0+TJk437CufP2R27\/HbrkJp27twpmCB4nidXXnmlsPvH2XdIUvjlpcN9JRawxKOKVZYN+a7rK1KBsc5hKlAsfBvOMSd25NiZs7t09tq7d28hyiX+c9GG+80335gjoah\/7rnnyk033SS7du2SgrqvYKKfSpIDf6lsz7W2+OK61qdU9+fAFEqq20xle2+99ZZAhW0zkTkEi6DJOebEjtz3339vDjTgcAKWdoRMQVGOBXifPn0MRpgbYL\/ExOC+wqEI+OW1a9fOnNRCJIP777\/fMKx47isYbUYjzftUFAO3MRg5cqRAQc2T+eIF+M855gQTQnpCf0Qkgquvvlqefvpp2bdvn3Tq1ElYuoEPNkx+9xXOtLvwwgvFuqawlCtWrJiUL1\/eLBF5BkqF+wrtKCkCikB6EXCOOTFcYjHBjEhD+\/fvFyIWoPTmPhrl5OQIEteGDRtMMbt96JiQrnBZwX4KIp2M+4ppXP8pAopA2hFwjjlhSoDLCbt19913n6BDuuWWWwSpCJ0T5dFQgRGxvOvRo4cx3ORQTZ6vUaNGqEdDReur5mURAjqUtCHgHHNipG3btpUxY8aYnTcsxAcMGCAtWrQQlnkwH+pEoypVqsi0adOE4HTs7mHH4XmpcV+J9j7NUwQUgfQhEDpzQr+EchsizVBZkg0fPlzefvttw2hQ+t18880UCQpzk9B\/ioAikNUIhMKc0CEh4bCVSaiTa6+9VqArrrhCyGOXDakH5ffpp58uJUuWlPbt2ws2UFk9Gzo4RUARyEMgcOaEYpvjldn6f+2118RvaIllOJEwcTlZvHixYErAUq1Lly6CFIVCO6\/n0RKapwgoAlmDQODMCedd9EZt2rQRlNiRSGIScNJJJwln1WEu8Pnnnwthd\/fs2WMiFUTW13tFQBHITgQCZ04wGvRGRLxkR876ztnrNddcI9gnEc+JAzVHjRoll112mcCoYu3UZefU6KgUgaKNQODMycKNrxuGkq1atRJ21saPHy\/4w8GIyPc8z7gOvP7664KxZc+ePQ+KIW7b0asikDgC+kQmIBAacyL6wJYtW4wPHPGbihcvLhhIEsFyyJAhxmcIaQoF+VVXXSUwMaStTABV+6gIKALJIxAacypRooRgCb59+\/a8UbA7hzI8NzdXOPxy4sSJYollni7r8qDShCKQ9QiExpxQhqNfInpA3bp1hTS7cXfddZe8\/\/77Jg4TFt8rV66UUqVKGTMC9FV2RnBlISTKmjVrbJbMnTtXkLZq1aolBJ0jjArMDoNOHIjr1Kkjo0ePFkwZ8h7ShCKgCDiJQGjMCTTQL73wwgtyzz33CEaXEyZMEKzDWdqtXr1aCJOCeQExmjAnsEaas2fPltatW8vMmTNpxhBS1wsvvGCYDxbinFmHtAWj44go6s6YMcMsF5cuXWqe0X+KgCLgLgLFwuwaNk1IR8OGDRMYDwcbwEA4PPOnn34SpKNo\/Tv22GON0eYpp5xyUDGOv8RwIpMlI9IZjAgFOwp47lkuEh2TOkqKgCLgLgKhMScknVWrVpmTfUuXLm2OTyY0yksvvWScfVmaPfLII+b0XgJqIQUdffTRwt\/5558vWJNjE8U9hD4K0wROBq5cubKw+4cSPb9gcxxmkCj1n7lRLPFuJUVAEUg9AqExJ\/Q+KMWPPPJIM6odO3ZIhw4dBB1TuXLljHkBx4wT3oQol+zU8YypHOUfAbfQLSF5EbMJiYznkb7iBZt7dcY8SZQGTl4qluzBCJHXsh1fleNbD5MKjboakwiC4imdmgwW+uypB+NHYEUoqM9VlK9dWrNCY05IPfjKoaAmwBzMBWkKpferr74qWJDjb2cJnRPlsdDg+RNOOEFOPvlk8TzPhFiB4cHoYFb2OdI2IB15R80fIP8c0zEh+mpYc7G088l6Bx2QYO+Lf7FAmje8VPb+Nld2XT1Sjm77clZFlpwzZ05WjYfPTyTx+YjMc+keXSxU2D4lModgETSFxpw8z5M77rhDWIIh3bCkI7TuggULTKxwe7CBvWKoCTOLBRAuL0hX1jSBgw2oH1awufeG3CGc4GKZVavq5QXpKlb\/My2\/UqVKmdblhPvLlzfhhzLoAdfnMHDmxNIMJkKIFEwHsAonJAq6Jrb\/kaKok+gcn3HGGUJYFdpAX0V7+OfVcCTYnD377oudexMdmtZXBIokAoVlToUGi6UZSzSWa02aNDF2Sdg4QWz\/Dxo0SHDyze8FRx11lDE\/OOecc0xVz\/NMMDokLaSvv\/71r0bJ7nluBZtT5mSmS\/8pAvkiEDhzYqnFEg27I5gRCmzC70KTJk0yHcYx2CSy8N+mr7NDcioK59Zl4cfvoCG5PoeBMyeLDks6lnAwK5uHQps8ymxeNl1PPqaEqOSUTTOqY0knAqExJ1xSMJrEMhw3E2js2LHmCCiMJVM1aJfcV04uU0LGL92aqqGF2k5BlamhdlJfHhcB1+cwNOaE7VG\/fv2MFTjGk82bN5fNmzcLhpcEpIuLagKFLrmvTLntfKldubTZtavaZ6Ex5ExgKFpVEShSCATOnFi2bdu2zYCMxXf37t0FJTb+cH379hXcTCi0dUgnQ665r1jzglbVyxspCvMCGFXukBXJDDPwZ13XVwQFyPil2wQPA+aPeWQ+LXE\/\/+NdQXUl4fe4PoeBMyf85qZOnSodO3YU4oQTU9yiSnrevHmmjDo2P5lrfu4rybSdzLOYFqzsUdMYcMKoaMt+qO2VDzcfeogvgHWZ4Ur9MMn1JUGy2IDxiMW7DOMBe4h5gOz8cO0\/8zN575NdctIBfaKdT2zbujeoJMxr7rPu\/ug4NYdRJixw5sROHCepPPjggzJ9+nSpV69enlsC\/nJIUByAQJ0o\/S1UFkvIeA9i\/s+vSCqJJWpB22txxs\/ydKPSsrxLTh7delFpaXhaCSlT\/Ef5bu9eWf\/lLpm9dquMXbjJUJVe7wrEF0RprlkqJ4sDeFoC58lrvzG4gz3EXED+eZp8QzmB7q5RQvzzzZxCI5qVM317Zvqag8r9dQub\/vbbbwUq7PMFfY7vh4TwFzhzsmOsWLGiCYnCssua32Of9MADD5h44bZestecnBzBZcW2Q9rvvkI+7+dXJJV04oknGjurwrbZr+X5Ao1uX11m3VUzj9b2qit+4ldaqZ6RQJPFwY8r6RntTsnDnTlgLqBE5vTaOmeZvvWa\/ZVs2V8mqc9E5HvxS4Ui81N9z\/eD70nQFBpzCmqgHC9F+F\/MEyDSVatWDer1WfsefnWzdnBpGFidAxsht7+yzilTEtfnMOuZ0wUXXCD41zVs2FAaNGhglpHkpeHzlz1N6khSjgA7teikch3WQaV80Ek2mPXMyfPccl9Jcr6ceZylgzOdyaCOYOvGRgcK97C77fochsqccAC+7777BDsn9E2DBw8WduzCnjR9vyKQLgSQoNjFY5fPFSaVrrEm225ozIkwKf379zchUxgECuQffvhBBg4caAwzyVNyFwHX9RXuIifSvUGOUZLDpPAYYJcRRhV0n5Ofw\/T2ODTmRHQC4oRzLl2xYsWEqJiYD6C0piy9w9bWk0XA9SVBsuML4nmYFHoodhlhVEEzKdfnMDTmhIsKUS+x07AfBJZ5MCYMNW1erCuSlx4NFQsdzc80BGBUfiaFsWemjSHV\/Q2NOeHcS4jR2267TdavXy\/E\/CbG03XXXSccVhBvoHo0VDx0gilzfUkQDAqpfwtMCmmKllnqzU+j+4vrcxgacwJ8tvf\/\/ve\/y1NPPSU33nijkCaPsnikR0NFQ0fzsgUBQutYxXlRNj0IlTnNnTtXevbsKeXKlRMiYBKyd9SoUfl+xlJ5NBRM8aGHHpJ4xC8M5K\/DPeTP86c54NPeUw+y91y5h0jnR9SD\/PW4h\/x5sdLUg\/zl3EP+vFhp6kH+ciYpMs9f7k9TD8ovz19u0zxnyeZxjZZHfiTZelz9ZdxD\/rzINHNIHSiyjDwoMj\/afWQ97i1Fq2\/z9rw3SuqXWGfcXybOX3fQZ5TnUYl89NFHB+XbZ\/1X6vrvSZPHHJLOj3Bf4XtC\/SApNObEcU+E0lRcx+MAAA4aSURBVGVZ9\/nnnwuKcZZrhDhB7+QHgSUfAGFMSexxf5lNY2JfmKOhiDWOL188QnEI+etwD\/nz\/OlbbrlF7D31IHvPlXuIdH5EPchfj3vInxcrTT3IX8495M+LlaYe5C\/nHvLnxUpTD\/KXcw\/586KlqWPJXx4tz19u07YeV5vHlXuIdCxiDqkDRdYhD4rMj3YfWY97S9Hq+\/MmPNr5wO5eJekwaZtc0brbQZ8pXFfOPPPMvDz\/c\/407\/LfkyYPIp0f8d3ie2K\/a0FdQ2NOBJojDjjRL5csWSIXX3yxoCQvXrz4IaYEnD8HQEQsOP7446NiQzltJXo0VNTGNDNfBOwvb74VtULSCKCHsu4vSTcmkteE63MYGnMiEua+ffvkT3\/6kyxcuFBycnKEGOLkoSzPQ7CACdeOhipgt7WaIlAgBNBBUbEo7eKFxpwIY9KnTx9p1aqVDBgwQAg8B\/iImEhQpBMh14+GSmQsmVCXJUEm9DOb+sgu3vxPdkmqdvBcn8PQmBMfGpjQeeedJ+y+7d27V+rWrWsYFfZOlMcjloTEH8+0o6HijUnLFIH8EGB5N2DmZ\/lVy4ry0JgTRpRITk2bNpX69etLs2bNhDji7NyVLl06K8AtwCAytorr+oqMBTafjg9udZYgPREeOJ+q+Ra7PoehMaevv\/5a2JWbOXOm3HXXXTJ06FB5+eWXBQnK87x8gdUKikBRRAAbqFbVywmOw9k+\/tCYkx9Y3FW2b98uRMf86quvDNPyl\/vTHPXEtuZFF10ktWrVEizK7S8Ax0tdf\/31ZuevcePGsnbtWqE+JgbUr1OnjowePVr279\/vb1LThUDAdX1FIYaUMY9wSMYXO\/fKhn\/\/Oqk+uz6HoTGnsmXLGuPLfv36mdClzz\/\/vLEUh8Ggi4qFOgcWYEk+ZcoUId44y8ERI0YY8wMiGnC\/aNEic0jCiy++KMuWLZNZs2YJEhqnC7\/11ltCaOBY7Wu+IuAiApF9Qve0eMcRkdlZdR8ac\/I8T4jldP\/99xtJp1OnToKSGwYTz5QApsYz2DQxEywNScPUSOOv53meYG2OY\/Dy5ctN+xw5RbvVq1c3DItn00kYlaaz\/bDbttJq2P1I5\/tdnkNMCxbvOFJ27jus0BC4PoehMScQJWQK5vdvvPGGCTKHOQBOwJxtR3k0gsmwwzdnzhwTC4olHqe2YHFOPCgOSDj33HPlpptukl27dgmSFrGi5Je\/ypUry8aNG3+5++8F6\/NUEy2nuk2X2uOL61J\/0tEX1+fwyJ\/2yJQln+adXpQoBonMIVgETaExJxgQritPPvmkWZ6xRIOwFofJ+IGI5r7CDh8nqXAoJ0HrOIoJRteuXTtZvXq13HDDDYKERVvYVPnb86exLFf6VBSDzMNg86AmMmvoA4HNnf97E0Q6NOZEUDmWceiaHn30UbEEs7EGmRYAv\/sKkhUnA1PmeZ5Uq1ZNcIXBz+jCCy8Ue+wT7jAEsStfvny+R0PRViKkdRUBRSD9CITGnND\/oPguiMGlHwae+eCDD4RdPXbi0CmxbGMpB5PasGGDqb5u3TrhHTgLcxwUzBAiHebRUM8995ysWbPG9JGdw0suuUSQIOmbydR\/ziKAXhOPBpZDb775prP9zJaOBc6cdu\/eLSzDHnnkEYExXXXVVXLrrbcKuiKIMurEAphjnRo1aiS5ubnCmXQTJkyQbt26Cbooln89evSQ2rVrG7spFO41atRw4mgoGCcKeqzaGdvWrVtNkD12ETmy6u233yZbyWEE0I127NjR+IDOnz9f9AclvZMVOHMqWbKktG3bVu68807BQpwwKTAq7iHKqBNr2J7nmcB0nNaCyQAK8QoVKpjqVapUkWnTphkd1iuvvGJMFDzPjaOh0HuxHMX+is4i+eGsjCSIvcmWLVvIVnIYAX5MmSuc1vmMYkgct7tamBQCgTMn4oYjRRDgirC8KMQ9zxNCoUDHHXecUCepUWXAw0RfQE9mu4okZdN6dQ8B3K0w3i1RooTpHIHe2CE2N\/ovLQgEzpzYpUNSYkmG5TbKb5ThTH5aRpiGRukrSzSrO+IV6COwVscSnXP42D2kHLMHrNYjz+MjbhU6M56F2AbmqhQ8Augt+bG0b2aHF\/UCNnF4FSDd84PJnDH31EOficM6aaX0IBA4c2Kdjkh8zTXXCDGc\/vjHP5q1ezw9U3qGXrhW+aC2bt3aWJzbFvjAYjwKc8IUAjMGPtzYba1atUrGjRtnDExtfa4Yk7LzCLNGiY\/rDvlKwSHA0pofRjYkkITsm9Et8eOC6gD1ADvKGCxy8AZzhbEv9flhtc\/oNfUIFEt9k\/m3yBa\/vxYTzRfcn5fydIoa5NeS5egpp5yS1yIfVpZlKN\/JxLwB\/cSOHTu4jUpEX8AgFGU4BqWYPkStqJlpQwBpCPOTP\/zhDwe9Y968ecKmy+GHH278PdFpYkPHXOGgTvQM5hpTmIMe1JuUIhAKc0rpCAJuLNrhCjgtI\/IjEdIdflER+6Mxp969ewsxqDzPMxsD77zzjgwZMsSYPfCsUnAIsMOLdwE\/JvatLL9RdCPZkud5nrHAJs490i2SFD8mPEe5UvoQCIU54XiLXRJ6Fn6B2HXjyj12SbEOMUgfDMm3zG4cv8TJt6QthI0A88iPTdj9KOrvD5w5sSOH2BzLXYIy6mTSxLDUQ5dm9WZciUvF0i2TxqF9FSlZsqTRD7LBAR5sWqBv4pQT7pWCQyBw5hTc0IJ7E8wUK3V2fXgrVyIlsGzgXilzEPA8T+rVqyf8SLJrh\/0Zy\/bTTjstcwaRJT1V5pSCiSRYHlbqRPJEsT1+\/HhjtU5+CprXJgJGgHA7\/LBgRoAZCM7kGMwG3I0i\/zplToX4CLBLgxsKim37ODs6KEsXL15sTAe4t2V6dRsBIlzgXG57yS4dDujMJRsWSFK2TK\/BIeA4cwoOCH2TIqAIuIWAMie35kN7owgoAr8goMzpFyD0oggoAm4hoMzJrfnQ3hQOAX0qCxFQ5pSFk6pDUgSyAQFlTtkwizoGRSALEVDmlIWTqkNSBLIBgXQwp2zARcegCCgCISOgzCnkCdDXKwKKQHQElDlFx6XI5XI4BFE8OQ3GEjHPMx0IrLxfeOEFc2grrigEC\/SPiXEPGzbMn5WXxuF30KBBgo9dXqYmAkNAmVNgULv\/IqJC4q5hiVDEttd45xND296n4pruNojN9Nprr0lhYy9xmAGhUwjpk+6+avuHIqDM6VBMNMeHAFJF586dpWnTprJw4UJZu3atcJwXTrHESke6oPrcuXOlVq1agtSFE3SvXr2EuFxIK1ypg9SCpAKj49Qc6tLOww8\/bCQbymmTMMg4ULdv396EcEZyGTBggJBHYDiiUcJ0unbtKjaCKhIOxHssER2CoH8EibN5sa5IifQHIqAgfdizZ4\/UrVvXHAXFoRyxntX89CCgzCk9uGZkqx06dDBRHwn6x8EMHNDAQAhDTKQFYhoNHjzYRO7k3DaYA6fnfPzxxzJy5EjhDEFCjcAMkFp4NhrxLIeeElHy3XffFSKIvv7666Yq4Y2R4KiD1LJs2TIhpvemTZvk7bffFhgYSzViaG3btk0Ij0wsrZUrVwqHXZpGfvlH\/bPPPvuXu\/9e\/GNknJMmTTIFSIlIjIwzJyfHHHRKvxgL4ydsiqmo\/wJDQJlTYFC7\/6IRI0aIDQLIwQw26sJll11mwghv3LhR+AIjXfDF5vTb1atXG2mK+FV8kT3PE7z8idwQa8QwF5gAzI446oQp5iRm6iO1EBuLyAC8n6UkBw3AeDjjj1Am6JCQ0jhAYuUBpsRBEbwvMuYSTIt82rXkHyNjbdasmS0SDpsgjHLDhg0F\/RsFMEiOg9JjoEAjWMpjTsG+Vt+WiQiwtOG4JBgSX2xOIpkyZYqJHMlSzY6JM\/lYitn7yCvLpXvvvTePESJF9TqwDIysZ+9pizDI9n7Xrl2GkVx++eUmKBxLSo4ag3nZOole6f\/YsWOFcxNvueUW8Twv0Sa0fooRUOaUYkCzuTkUxIQghinxZZ42bZo88cQTQiA2ll1EjSQf6QpGRrA9JCBCFpOPDgh8kEqQhpBskFZ69uwpLO8oi0YwRJaLMCmObOrSpYtwCEHVqlWNXgtdGNJc5LOESY63vPTXZ+nIeDgmij7bMiQ3+l6qVCmbpdeAEFDmFBDQ2fAalm333HOPoNSuXbu2EFyvY8eOwvIKxTdLJA6ogEmxnGIJxj1HKbFjhm4KhsWyD4bDlSUj58GRjoVR48aNjSSDcrpFixbCmYcs\/WAY9qAM7iOf5x3ozWAukWX+e5geUtMXX3whKORhdFzRf6HXYixIVP5nNJ1+BJQ5pR\/jjHgDkSCjMQiYD2QHUevAjhyKbCSfcePGCRE\/Pc8Tzn5bsmSJ0UmxLIIheJ5njr9CxzRr1ix57rnn5LHHHhMkE6Qf8iGWeOTxfvph38V7yYM59OvXT9jS5728y\/M8s1MH88jNzTXMyz5nr+zsIWEhbdEG\/aU9W86V99EXytB7IfVBMF4YEmPlDLtkloy8RylxBJQ5JY6ZPuEAApgnwKSQxDgYM1qXYEg33XSToJOCWUarEy8PM4lvv\/1WkP7i1dOy9CBQLD3NaqtFGQF22dj1SicGnHgzdepUI4nFk2rY\/cNmyfMSV3CjY2MJi1SXzrFo29ERUOYUHRfNVQQUgaQQSP5hZU7JY6gtKAKKQBoQUOaUBlC1SUVAEUgeAWVOyWOoLSgCikAaEFDmlAZQ3WtSe6QIZB4Cypwyb860x4pAkUBAmVORmGYdpCKQeQgoc8q8OdMeKwLpQsCpdpU5OTUd2hlFQBGwCPw\/AAAA\/\/+QFlFnAAAABklEQVQDAEEHFE4vT\/lxAAAAAElFTkSuQmCC","height":178,"width":295}}
%---
%[output:6b1bf100]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAScAAACyCAYAAAAAhgkFAAAQAElEQVR4AeydC7xNVR7H\/4dmEpOQJI9yC6FkNBF5lEnoLVMTRmpKJUqFuk0y3R4IvVTSQ83oQe+UiR5KiiYhUSSuiItqvBOSNL4r69h3n7PP495z9tnn3r+P\/917r+d\/\/fbev\/Nfa6\/1X2V+1X+KgCKgCAQQgTKi\/xQBRUARCCACSk4BvCmqkiKgCIgoOelTkBACmkgR8BsBJSe\/Edf6FAFFICEElJwSgkkTKQKKgN8IKDmlEPGpU6fKkUceKY888kgKS91X1Oeffy5NmjSRgQMH7gtM09nWrVule\/fucvLJJ8v333+fplpiFwuOtJd2x04ZrFj0RW+E80S1CwLmTl3RnTbwXDvDY56nMFLJKYVg+l0UpAF5QCI82Oms38+6aAcvxpgxY+See+6Rxo0bE+QpyerGy5auHxHuw7Bhw6RSpUoyceJEOfTQQz31DnoEuJ922mkyYMAA4X74rW\/WkBPWAg+UFR4wwOJh4OVEeBhsPL+6xFtx5ueF5oEmjnLIw0tAGZxzpFziowllkw7hl8XrxlEndT377LPGoqIuyiM9+ciPUJ4z3NaPDpxThtWXdAjXF1xwgaxatUo+\/vhjueKKK4T0xFmhPmf5xLvLQ0fSWCGPzW+P\/\/vf\/yRaXeiAbjYvZdk8HLkmjraCL+e2rcRTF2FWuCYcGTdunHnBjzvuOC5N29DdpqVe6keS0Y06wIpCR4wYEbZyCbdlc+SaNNGEOqmfdAjnhIEvZXM\/uC+dO3eW7777LqII0pKHvAg4uRO988475pkh3okZdThx4Jww8qMz6XneKN+W63zerrzyygiL2BlPfmd9F198MUUL98Oc+PgnK8gJsF555ZVCsPAQcDNsIA9E\/\/797aXw4Nl4bpIzPw\/OddddZx54m2H06NHmJeeasp555hlOI4QyKdtG\/PDDD3L11Vd7dn2oa\/DgwTa5SUd68tlAyqNce52qIy927dq1JT8\/3xS5bds2Wb16tdSsWVPKly9vXkwnLiRy40pYNPnxxx8FDGmfjacs7hXXHLnmnLaCL+dWaC912WuO9heal3f27NlhPYnLy8sL3x+uqZf6aRPXTomnmzMt57F0Id4pEAH1Ur8N5xyCXLdunQ3yPEbLD07gZTNRnvOZwYKEQIh348CzShhxVshLGVyDpfN5g\/TIQxxCuT169BDuEdeI83nE8sMK5H5QFvF+SeDJCUAmTJggBx54oLz22mvy9ddfy4033mjwefPNN82RP17x5AfYFi1ayIIFC4xwzg1CyIsQRvxjjz3GpXzwwQeFyItAHqwnn3ySUyEdunTp0sVYL+Q1EVH+oC9p27dvLzyIPDjkI4xyyOJsC9fxpFq1avLiiy8K5IPulPOHP\/yhULZqe9I0a9ZMaD84oCN1t23bVnipE8GVAg855JCIukKhkCE66qZcsEQXcMNa4Ehe9CKedFxbob3ue8YLwi80+Tdt2iToSZvAfc2aNaat1GPLW7hwoXmp3DiEQt66oQc6oQf3pXfv3hJLF9I5hfoRykEPhHNw5UeAsrkGC4iArlGs\/DzT4ABekKpNSzm2bHABEy8c7P21eamfvHfffbd53tHNhlGuTcdx5syZBkPCnc8jzzn18SPGjxn3Ax3I45cEnpwABGCOOeYYycnJMbjwYnPzuSn218oZ36pVK0NmPNArVqwQ8vNAYUkgnJuCHH\/si8AvBQ+LIyp8yguN5UHdlEMEDwA3FeLh2i2UhT42nAeY806dOnEQ8pGfckxAiv\/UrVvXtB8cqdvqwzW4OHGLhquXOpDj9OnTZfz48YKVycPPS0D67du3G+KyOEEwl156KVFGeOi5N7x05557rum+8GttIvf8QTfi9pya\/+SvUaOG+RGgHrCnvPnz50cdj4qlmynQ8SeeLo6k5hQMObHPC7qhC2GQHMdY4s4PedEOcKxQoYLJ6sSNdpvAPX+oi3TgDvmAQ6xneU+WsNVs9QU\/hDjE6oMVS5eOI+FBkMCTU6pA4sWDBJwCMaSq\/KCWY4nx7bffNtagk4yKozMvNeMdPNAQIC8JL1UyZZKefM57AknbF8ZZFuG8kPbF4iViXAWL0JmO86Lo5qUL5QVJGKIAczBy4lEcHfnBwoJz3gdIEDK0P8h07fjhLk49yeYNPDkBCMBgwi9fvty0z3aN6LJUrVrVhPGQI1zQNeCXl1+dOnXqmIFVrKxoDzLpExW3iet8CRi3SKQcXmTS8XBxZKyBh42HjmsE64yHgvbSbsKKKlibENLcuXONNWN\/QRPF1atesEboGkHyvCjWcrJ5uCYcnOgm2HAeeu6Njbfh9mgxstfcN4gIQrLdD+r1yo9eCGm8dLNlx9PFprNHqxvdMNqF2LZZa9imjXa0+e3957nh\/kP0zm5dtLyMD9FV5Ie2957uaDLPh9UXXBBbPvr88MMPQvfOhjmPWLFY2LxrWKTOuHSfB56cAKRbt26mX+zuArgfBh5ebjTkBXDEkx9geZD51SUe4WHnoSddosKDzMsN8aGLNaspF0mkHCwZfqnoxqAHR675KmIJw+pKHdQVr1weNtrOi+JOa3UmDQ8Z9ZMGXBLFlfRWKIe6KlasaLrO6E87CLNp+MGgbK4JtzhxbYV7wznx5LfCywoOYEI8gq7cQ+qmLNJSL2lISxqEeMqLpRvprFAGPw6xdLFp7ZH7jFAXuiCcY3lxbtN5HUlDWp5R2oG+pKVraLt1XEcT2soPtc2byPMBkVEfOlK3rc+Wz\/MAjmCBPlbAhTT80FMnRM+1nxJ4cgIMfiUAmXMr\/ILyq2ivuQF33HGHvTSD5jaeLoEzPzfjoYceEh76cIYET9y6UO\/9998vkEAiRTDGwGd1Z1quCUcfvrTYuL59+wovgr12H60l5w53X9sHEAsKS8rGu9tCuBtXwhB3XY0aNRLmwBAHnnQD0BVLj190vgBxTTwYOe8NYdwb6uLcCi8A4fYltL\/2xPNFypbHNWJxS1Y3ykdnykCo00sX4p3CfeZ+0yYbzjmD8tw\/G+Z1JA3PnrN+226vPDacvJb0CUNnnmt+zLBQCXMLeWI9Uzx3jBk69aFMng1rqZGfctxlp\/s6gpyY09K1a1fzpQQrIZ7EUpBf\/Z49ewq\/hhLl38SJE81gqGXrWNYMBOPsE\/NAuYvkZbFpANcZ78zPACQ3hXjKIY9NTzjxvGw8iKRxi7MsBiftjXOXRTrKokxnGTYd9SJc23jOCUP4tI4etg7KoTzKJT36EU9ajlwT7habL1oayiK\/FeonP2WR3tZtr0lHONc2LzpBHIRzDgHyC82vNS8QZcybN49ihW6EOdnzh7ooz4q9B+DJS2i7t3uSGvKnfJuWI\/ndcaSJpRtYIOhJGbZOyuLaig2nfLegH22yaTknjHTUjQ7OMMKd4qyfMmxd0fJajNGPMkhLHoQwZzzXhJOGtAjvHveC+wOBdejQQfgB4QscpE4atz6U6QynXK79lghy+vXXX+Wnn36SPn36SG5ubkwhjZfClMOnaq++LPkKCgqE+S8AisS6oaQvHZL9reQl40eNlvBi8ONDV4RfZ6wWwuMJ1h7dUPLFS6vx3gjQlcOy44eCc9sVZMyP++SdM\/MxEeQUCoXMfJJTTz1VzjzzzJhCGq8mwM6zZs2SNm3aeCWRtWvXCuMTngkSiADgeL9UCRSjSVKMAL\/e\/OA4BWuFX+lEqiId6SknkfSaJjoCWHT86DvvA+fWOoqeKxihEeS0\/\/77S7t27WS\/\/faLqyGT86IlYmCWfjW\/ml7kw5cJ5rrQn61Xr54ZW5k0aZJgcUUrU8MUAUWgdCEQQU506R5\/\/HFp3ry52PGin3\/+OSlU6M5hSlKGV8adO3cKXycGDRokixcvluHDh8vIkSPl008\/jchCt0DlyELjc4pHqcfD1+eBjxwRL2aaAyLICWsIC+b111833TvWq\/3pT3+S22+\/XVAwnmWzZMkS+fDDD81C1FAo5Kl+5cqVBeuKbl\/ZsmXNAPwJJ5xgllpEy4Qpmk1CG7JJX3RVnb82y6PAIp2SjTjHGsKhPemQCHKiEsiiYcOGMmTIEGHy3hNPPCF8xTv99NPNcotXX31VvKyplStXyowZM+TEE080zM6AJt075yRD6vjyyy\/N+jQsNa6tMGhqz\/WoCCgCpReBqOTkhON3v\/udMAHugQcekClTpggTspjnsXHjRmey8DmfHZ2\/OsyZ4HOyewCOgWymzPMV4ZdffjFLK5hXQV3hwrL45N1338067VVnf25ZNuLsDzKFa4lLTlhMkAvdLywn5rBATgcffHDhkhK4YkY2c5mYe8GYFIPhOOZiQJyxp5tvvlnq16+fQEmBS6IKKQKKQIoRiEpOWEU4rGLCFvNNsJggjjlz5ggWUNOmTYWuXyK6kB5rirT2s6a9ZmIYK7mxtOgK8pWQdCqKgCKgCESQE9ZN586d5cEHHxSO06ZNE8aYzjjjDDNLVyFTBBQBRcAPBCLIiQFpphIws5sZ4Exz90MRrUMRKA0IaBsTRyCCnFgPxyLL9evXxy0FKytuIk2gCCgCikAREIggJ8pgTdNdd90ljDPFEtKQXkURUAQUgVQjEEFOdOtuuOEGYdIVg+GxhDSpVkjLUwQUAUUABCLI6YADDjBr6+It+rXxFJKtonorAopAcBGIIKfgqqqaKQKKQGlCQMmpNN1tbasikEUIKDll0c1SVTOEgFabEQSUnDICu1aqCCgC8RCISU7MecJlSoMGDYQ1ce+9957cdNNNxkNBvII1XhFQBBSB4iDgSU74bbrvvvuMa5Snn35aDjvsMOOcHqfo9957r+zatas49WpeRUARyBIEVm7YIbvLVxWOfqrsSU5s871o0SJhztMRRxwhZcqUkXLlyskll1xinM6xONhPRcN16YkioAj4isDMZZtkS4fh8sc7\/ysz8jf5VrcnOXlpgG9xfDx5xWu4IqAIKAKpQMCTnNiYgI0T77zzTuO6FIdwbE3MTqD4YsLNbioU0DIUAUVAEYiGgCc5hUIhYTAcrwQXXXSR8e2NCxW6c3T1sKCiFahhikAwEFAtsh0BT3KiYbjSvfXWW4WxJ\/ag44ifJ7WaQEdFESgdCPg9EG5R9SSnLVu2mO2a8EoAQfHl7rbbbgt7KsBT5urVq205elQEFAFFIKUIeJITtfDF7o033hCIyl6zbRThbExw3nnnEayiCCgCpQSBw6uU862lnuTEFzk2vmRzA\/aXGzp0qNnK6dFHHxU8F4wYMcK48k1MU02lCCgCikByCHiSE7PD2XklJyenUIl169Y1M8SJd8cVSpjEBWX17NlT2JVF9J8ioAgoAnsQ8CQnnM4xIE73jdnie9IKRwbGOcd6ys\/P57RYQpkTJkwQfJYXqyDNrAgoAiUKAU9ygnz69esno0aNMrv3tm3bVlq2bCm33HKLMLVgxYoVZvZ4cdFYuHChQHjsi1fcsjR\/1iOgDQggAjPzN4a1CsSYE9oce+yxwr5y7MaSm5srI0eOFPaXO\/30CA8JDAAAEABJREFU06VBgwby9ttvk6zIsnXrVmE864orrhAmfRa5IM2oCCgCaUfAT2KiMZ6WE5F0uRgPYtFvs2bN5Oijj5ZVq1ZJ9+7dhU0QKlSoQLIiC905Zps3b948bhlHHnmkWdO3fPnyrDgWFBRkhZ5OPFVnf56tbMK5zrHNZcay39bTHV7Zvy91EEJMcho7dqxASuzMa6VLly5Sp06dYm+wuWTJEvnwww8FqykUCqFLTGFXYAbgs0Vq1aol2aKr1VN1zvHlnmUTzs9M3Nc7alW3csx3NNWRnuS0efNmmTZtmowfP16ef\/55M860dOlSufrqq+WYY44x0wmKo8zKlStNF\/HEE08UrKJXXnnFENXAgQOLU6zmVQQUgRQiMGH22nBprY6qFD7348STnH766SdTP7+qlSpVEjbZ3L59u5xzzjkyffp02bx5s4kv6p\/27dubBcVYRAgWGXOq7r777qIWqfkUAUUghQiwbGXC7G9NiWW2rZPWdSuZc7\/+eJITX+tY3Jufny+QE2NPCIpBTJa8uFZRBBSBkoUAxHT1hC\/DjSq3+PXwuV8nnuTEPKdu3bqZ6QJr1qyRo446Ss4991wj1atXlypVqqRURywmrKmUFqqF+YKAVlKyEICYznl4XnggvPWe7tzvV870vZGe5IQmTBmgC8cYE25S7rnnHnnggQfkrrvuEqwq0qgoAopAyUAAUhr+1grB4yXntIrpAw91a8ip7+JJTiz2ZQxo27Zthojo5jFRskmTJvLCCy8Ic5R811YrVAQUgZQjABFBSlhLw99aHi4fYnq9T1PhGA708SSCnBj05ivd5MmT5aWXXhKOeCaw8uqrr5pwSMtHPbUqRUARSCECEBKD3X33jCthKUFKhFEFZJTbMUc+u6VlaomJwpOQCHLCIsLzADO3v\/nmGzODe\/jw4ca3E0f8OPXo0UMOOeSQJKrRpIqAIpBpBCAfJyFBTFxbvSwpYS3ldqxjgzN2jCAnSOe5554zFtOAAQPM8YMPPhArLFm58MILJRSKP3EyY63SihWBUo4ARIRAPpBQlf7TzFgS54Q54bGkhKUEKXHtjM\/UeQQ5sZEBrlJ27NghnTt3Fo7ff\/+9OIV40mVKaa1XEVAE9iEACbFlE6QD+Zwzep4hIrprXBO+L\/VvZxBQ7p6uG1aSJaXfYoLzN4KcmGx5\/vnni12uEu1IPOmC0wzVJGkENEPWIQAJIZANpINYImIwm2vi7Fo4ZwMho27NqsvoPV\/eNtzbzown5e7puvk9sdKpU7zzCHKqVq2amQHOrG0vYXoB6eIVrvGKgCKQHAKQDwLJIBCOJSAsIYQw4hAvImJukpuMICbCktMoc6kjyMmpCl4J5s2bJ2xwwJo65jdBWM40eq4IKALJIbBmyy6x3TA+4UM20QiIcEtAEJa7FqwhBMLJ3dNFg3zooiGv921qrCTi3Pmy5TomOeHLqW\/fvrJr1y5p2rSpWQvHONSUKVOypX2qpyLgKwKQCOJFPgxMnz2uQGw3jE\/4sQgI5SEgBKKxJGTHiiAiSCl3TxeNeNKRJ3WSuZI8yYl1dPhbYkuoIUOGyGWXXWY2OMDxHOHEZ05trVkR8B8BSAeJRjxYPhAP3S4kGfKhJZAKAsEgEI6TgNwkFOSxItqTCvEkJyZjYjHluDY44Jpw4lOhgJahCGQaAQgHsaSDJUOXCoF0IJt4xBNt7MfZLogHgXjObvgH0+WyBGQHqC0BEU46CIg8znJK07knOZUvX17YHurLL78shAeeE1lXx3KWQhF6oQgEDAEIB0mWdCAlCAqBdCgjVtMgEMQOQkMsEAyWD+Imn7z2VYU0CAQUq+zSHOdJTuy8cumllwoTMXHLy86\/eK28\/PLLBW8FeC0ozcBlpu1aK0SBWMKZ9OVWcQ4qx7J0kiUd0IZ0EEs8uXsHniEdxEk8dhAaYrLEo+QDikUTT3KiuJNPPtnMEG\/Xrh2X0rBhQ3nttdcEbwUmQP8oAilAALJBLOFgsUAkCGSDeHWt8qau20NOy4U8WDkIZcVTC8JBLOlAJpAKAunQxYpGPLl7B54hHSRePRpfdAQ8yWnjxo3yyCOPyI8\/\/ii9evWSoUOHyvXXX298LBe9Os1ZGhCAHJAZ+ZsMaUAc1rqxhGPJJtpYDmnIg0A2COXFww6yQZyEk7vX0olHOsRDUAikQznx6tP49CLgSU5ly5Y1UwewknACx1c6CCu96mjpQUQAYkBikU0s6waysZ\/MLeFQXiJthSQQCAeBPJyE81iX6ma2czQrB8LJ3WvpkK8UkU4i0AY+jSc5VaxYUUaMGCGff\/65cS7HkW7eBRdcIO+9957o2rrA39uoCkIKCESDQBYIlg1dJIjESTRelo2bbBK1blAKskEs2XRrVl1yE7Bw7JhOroNw\/lSznFAW5aqULAQ8yck2ky92bA81atQoefHFF80XvNtuu81seGDT6DEzCEAyVtxEA8kgEA1iu1EcEebhIKRBIBsGlyGqZIiGlkMOSCyyiTWOoxYOKKq4EYhJTlhHTCUYNGiQsPElfpzYBRifTrhWcRem10VHwJIMx2SJxpINJINANJAMAtEglJuodhANkkqy0S5VouhrOouAJznhFqVTp05y3nnnCTut4LL3448\/FqYU1KpVS\/05WQQ9jqyfghBm5G\/yHBSOZtGkk2hikQ1fp+ZeU0cSHbtRsvG48RqcMgQ8yYlJlnyhY6yJnVFYW8cgeSI1f\/HFFwKxsVlm69atzeac0fJNnDjRbKhJOoQxLfxGRUsbhDDIBnFaNlgqCESDQC6M07B+inN31wlrBsGaQSgv0bZhzViJRTReXSg7ZqPdqEQR13SZRMCTnJiEyVgTY07JKMiaO0itT58+smTJEsnNzTUufteu3bdzqC2voKBARo8ebb4K4u0gKK5YIAwIBLHEA9FYcRIOaRCIBiGvbV+soyUZjskSDVZOLKJRqyYW8hqXLQh4klNRG7Bq1SrZvXu3YDGxzIXtxiG4devWRRQJYVWtWjUi3M8AyAThaxWWD1YPJAQpIZZ4SBNLL0gGsUTD+qlc1xeoaBaNEk0sVEtXnLa2MAIpJ6dGjRoJPsirVKki+IOaNWuW2VqqevXqhWpmcueaNWtk8ODBUq9ePeN5c9KkSSZPoYR7L+j2sa4vVTJz\/lK5+PHZYXemDCJj+eytrtChRsX9hE\/WEA7C2iiEOTaTLq4ljNW8+rfqgow6o5IMbFFOejXYIX+t\/6u0qLrdSM2yGwX5ZfNaSVUbUl0Olmyqy0x3earz8rQ+T7x3SKEXwqcLT3LC6wAD4BydunA9d+5c+fnnn53BEeeMVf3xj380a\/MYS6pUqVKhNDt37pQKFSoIXwIXL15sun4jR46UTz\/9tFA6e0G3D48IxZWP1x0g107eJIwJ8encls8Rywdh3g3jMlg6G+5tJ1\/ktZF3BrSUcZc3M9LvjMaCnN+6obRqUs\/MmnfrxUcDd1jQr1XnnKj3MtX3LZtw5r1DeD\/8lghyYvoAX+r4xWOHX44MUlv56quvzFKWeLPFGzduLPPnzxcc1llxNq5y5cpm2yk26mSgvW3btnLCCSfI7NmznclSdk63jG4bXTWnhWTJCCL67JaWgkBMEFTruoUJNWXKaEGKgCIQF4EIcmLjAjYwOOusswQLiWOLFi1Mt4sjM8QhHsglWukzZsyQgQMHCtMPiMckZG7U6tWruQwL86eYnmDT2YhUezuAlBhPYhzJkpKbkCAjNxFZffSoCCgCmUEggpzYuICvZowVQUwcMeusLF26VPLy8sxM8WgqMzkTi2nBggVmiQvTChYtWiTNmjUrlJyvgXg4oOuItca+eHQF3ekKZUryAmJiQJvxJJuVAWusJCUki4geFYFgIhBBThDFhg0bhDEiBqsZ1LZdOnuk20e6aE2qX7++9OvXT6655hoz0I1Hg6uuukqOP\/54s\/cd409Tp06V2rVrm8HwYcOGmXSMPTHBk\/zRyk02DGK6esKXxp0GebGW+HrGJ3jOCVNRBBSB4CIQQU5069hphbElund05dxCOOmiNSsUCsnZZ58tWERYWxy5DoVCYq0yvByQl3IZjyId3UHrN4q44oglJmc3Dmspt2Pmt1guTrs0bwARUJXShkAEOUEg48ePF8aJ6N5BHG4hnHRp06qYBdOVcxOTWkvFBFWzKwI+IxBBTj7Xn\/LqICY7xgQhYTFxTHlFWqAioAikFQFPcmJ8ifEhvra5hXDi06pZEQqnO+ckpoe6NlRfP0XAUbMoAkFAwJOc6LbRfbNdumXLlslHH30kV155peA6ha9yQWiAU4fhby0XCIqwVkdVEp0eABIqikB2IuBJTu7mhEIhYQlK7969hcHyeJMw3fnTfQ0p0aWjHrpxTBXgXEURUASyE4GEyck2j2UnW7ZsMXOYbFgQjkwbsHrQnbPnelQEFIHsRMCTnJjL1LVrV2FZiVM6dOhgpgR4zRDPBAxYTfbrHJMstTuXibuQTXWqrtmAgCc5MYOb8SX8MTnlqaeeEnyI4w4lKA10Wk03dswJilqqhyKgCBQDAU9ywhPmKaecIk2aNDH+w88880xhTV2tWrWEhbrFqDOlWZ1WE2NNajWlFF4tTBHIGAKe5IRGY8eOldNOO01wacL1+++\/L0wjmDJlCpeBkJnLNoX1yFWrKYyFnigC2Y6AJzmxvo41cMwWx60JDe3Zs6fZBXjChAmCO17Cii\/FK2HCJ7+5\/8Vqws1J8UrT3IqAIhAUBDzJadeuXab7VrNmzUK61q1bV4jD6VyhiAxczMjfJHYgnHlNGVBBq1QEFIE0IeBJTgcddJAwKP7yyy+Hpw3goYCFvPgEL1++fJpUSrxYZ5euW7PDEs+oKRUBRSDwCHiS0\/77729cn7z00kuCu12mE+D2ZMiQIdK\/f39DXJlu3cz8jUYFunQ6EG6gKMl\/tG2lDAFPcgIHPBPg0uSZZ54xWzyNGTNGcArHFzziMynOr3TapcvkndC6FYH0IOBJTuyMwiRMJmNCRkwlwP8SFlV6VEmuVLtUhVzapQMFFUWgZCHgSU4s\/G3YsKHMmTPHc7umTEKhXbpMoq91KwLpR8CTnFjYi\/\/vAQMGCBtjMuZkpWvXroJFlX71otfg7NIdXrlc9EQaqggoAlmNgCc58aWuT58+cv\/998utt95qxpzsMhaWtRCfqZZDTrbuVnUr21M9KgKKQAlCwJOcWL6CT2\/GmtxCOPGZwsE5hUAHwzN1F7ReRSC9CESQE901dk9hXzkGxG1XznkknHTFVY1uY6dOnQRPm61bt5Zp06YlVKSONyUEUyASqRKKQFERiCAnumvdu3c3juXovtmunPNIOOmKWin5WP4ydOhQoeu4ZMkS020cPny4rF3723IU0kQTunR2VriON0VDSMMUgZKBQAQ50V1jygD+mqJ5JTjppJMkFd26VatWye7duwWLCfcrDLoz83zdunUxkYWcbAIdb7JI6FERKHkIRJCTs4np9ErQqO1SKs0AABAASURBVFEjee6556RKlSrCshh2FoakcAXs1MGe0\/Vbvny5vDFnuQ2SoypsF8KCKAUFBYHVzQsv1Xm5L\/csm3DmvUPCL126TqKU60lOfnkl+Pzzz83yGKYs4I6lUqVKUdQUYaOFnJwcWbh+X\/T5rRsKYUEU\/F4FUa9YOqnOOb48T9mEM+8dsu+t8+\/Mk5zwPIBTuXR7JcCB3fz584VlMlZiNX\/lxh0mmvV05kT\/KAKKQIlEwJOc0u2VYMaMGTJw4ED56aefDLCYjqzlW716tbmO9ofxJoQ4HQwHBRVFoOQi4ElOrKFjSkG6vBKw7x0W04IFC4xLFqYVLFq0SJo1a+aJtiUmEuhgOCikSbRYRSAACHiSE7phydDVSodXgvr16xuXLNdcc43Uq1dPevXqJVdddZXgloW6o4lOvoyGioYpAiUTgZjkRJP5vJ8OrwShUEjOPvtswXkdA24cuQ6FQlQbVezkSyLVfxMoqCgCJReBuOQUxKbrYHgQ74rqVPoQSG+Ls4acGG\/SmeHpfRi0dEUgSAhkDTk5QdPBcCcaeq4IlEwEYpLTli1b5Pnnn5ebb765kLAGjjg\/IcFysvWpJwKLhB4VgZKLgCc5Mf\/oxhtvlMcee8ysgcs0BM4vdZnWJZj1q1aKQMlCwJOcNm\/eLHjD\/Ne\/\/iV33XWX4EHACh4KKlas6CsS+qXOV7i1MkUg4wh4khPeCQ4++OCMK+hWQL\/UuRHRa0WgZCLgSU5smolrlJEjR8rSpUvl+++\/DwuO5n755RdfEdEvdb7CrZWVXASypmWe5LR+\/Xp56KGH5I033pCOHTsKPp6snH\/++UK8X63cXb5quKraVcqFz\/VEEVAESi4CnuTE1lDTp083rkqYwe0Uwon3C5bd5fd1Lw+vcoBf1Wo9ioAikEEEPMkJnXACh78lBsSZPkD3Dt\/ihBPvlzgtJ51G4BfqWo8ikFkEYpITi34vu+wyWbZsmWAt4YAOTwXjx4\/3VWsnOflacfor0xoUAUXAAwFPcmIDggkTJsiDDz4oQ4YMETxU4kURK4pxKKYaeJSZ8uBdVY8Ol6kLfsNQ6IkiUKIR8CSn7du3C94wISQnArVr1zaXTNI0Jz7+OVwHw31EW6tSBDKLgCc5HXjggcL2T7gycU4byM\/PF6YZIH6pbgfE1fulX4hrPUFDoDTq40lOTMLs27evmRl+7rnnCgPjF154oXEQd9FFFxni8gMw1tTpmJMfSGsdikCwEPAkJ9TEydzbb78tN910k5x33nnGW+XkyZOFXVKI91vUG4HfiGt9ikDmEIhJTqjFGrozzzxTcKfbtWtX8XN+E\/VjOXFEdMwJFFQUgdKBQExyWrNmjUBIbIDJjrwNGjQwJMXyFb\/gWbV3Kyjqq125HIeMiyqgCCgC6UfAk5z4Gnf77bdLjRo1jJ9v5jp9+OGHgk\/xO++803zJS796Ik7LyY\/6tA5FQBEIBgKe5MQ8pk2bNpm95djGKRQKSfXq1eW6666Tb7\/9VpiQ6dWEgoICY3EdddRRctxxxwm7t0SbVT5x4kRhvzorjGWxwNhZ7soN28OXOscpDIWeKAIlHgFPcuJrHRMvsaDcKOBKhXh3ONfMj8LigmiWLFkizz33nIwbN05mzZpFdCGBxEaPHh1evzd9+vSIMa1VG3aYPDreZGDQP9mEgOpaLAQ8yYl5TN26dZPBgwcLX+gYf4I88I7Ztm1bgYTcVg6aMLMcq4tB9LJlywrjVXz1++yzz4guJGvXrpWqVfd5HCgUufdipWPMaW+QHhQBRaAUIOBJThs2bJAxY8bIypUrhSUrXfd8qYOoIKmHH35YcJuCuDGqXLmyyVezZk0ThTdNxqvoEpqAvX9+\/PFHoSzKZFNN3LFMmjRJ3N0\/O+akEzD3AqcHRaCUIOBJTowz0SX74IMPJJa4cWLAvEqVKoLVtHr1arn22muF2eannnpqoaQ7d+6UChUqyKBBg2Tx4sWC1wMc23366afhdJaYCPhk2huyfPnyrBG6rNmkL7qqzv48X9mEsx0P5h30WzzJCUV+\/vlneeqpp6R9+\/bCVAJ25GXRr3M5C+ncQr7HH39czjnnHDnmmGPk0UcfNQTlTIeFhTO7Nm3aGCKjq3jCCSfI7NmzncnC59df3kNY51dkycnxNf\/FF1\/sa32pwEV19ucZySacrR+38Ivo40lMcvr3v\/8tr7zyivTv31\/Gjh1runJ33323PPnkk54q0i1jUwTGqV577TVhM4Rog+f4hWJnF\/eAO1ZWtMIfHDa40Jc9y+hBPdKGoOrmpZfqfKQvz1i24ozefoonOTHm9N5775nu1hlnnCGNGzcWGP+RRx4RwomPpigm6zvvvCP33nuv1KpVK1oSE8aiYsjLLiym68j6vWbNmpl4\/vCFbsO97eSzW1pKwftPh7\/qWTbX49eKydeKgV\/vAe+kn1LGqzLcpYRCIaH75UzDdSgU8pyEyRyohQsXCmNMzl9oSI2ve0wxmDp1quB6hcHwYcOGCQPijD2xeWf9+vWd1ZlzSMqc6B9FIO0IaAVBQcCTnA466CBDTFOmTBE7xkSXDUsHq8er+4Xls2jRoohf9N69e5s5TExHYAwLAPhCh7dNmH\/GjBnCbi+EqygCioAi4ElO+++\/v5kNjjdMBqoZsGZQHK+YuFKJNo6kcCoCioAikCoEPMmJCuhuMfeIgXEGth944AEzrYBJlcSrKAKKgCKQLgRikhOVMm8JMmLGN90wLCrC\/RGtRRFQBEorAnHJqbQCo+1WBBSBzCIQWHJi8B1vBng1wI8Ui4m3bt2aWbQctSeqHx8TmMhKO\/DSwDIglu04ivLtNFGdnQrxQYQpJJnEPhm9P\/roI2ndurWZr9SpUyf54osvnM3x7TxRnVmjyuoInnGGUXr06GGWdfmmaIIVsV62Z8+exl13glmKnSyw5MQylueff16YC\/XJJ5+YNX5MCC12i1NUQKL68bK88MILph28KHzNhGjdk09TpFbMYhLV2RbC8iPmqzHj34Zl4piA3katVatWmXl59913n+AR46yzzhJWKjAtxiTw8U+iOjOtZt68efLf\/\/5XFixYYNwSsR2bj6rGrIofV77Q\/\/Of\/xSmCMVMnOLIwJLTtGnTjK9ylmXgKhiL49133xUWDKcYgyIVl6h+3NBTTjnFLGXhC2eHDh2El55foiJVXIxMiepMFbzQzOA\/6aSTIpYeEe+nJKo3E3mbNm0qfF3eb7\/95Morr5S8vDyzPMpPfakrUZ1Ji4A3R+TQQw\/lEAjhfWP6D2tleX79VCqQ5MSNWrFihdStWzeMBTeMWemAFQ7M0Eky+nXv3l2Y42VVXbp0qfDi+P1hIRmd0fX9998XunKsj+Q6U5KM3limO3bskNNPP13oQrNLEM9LKBTyVf1kdGbOH4vsmabDOlTmCLLLka8Kx6gMw4Av9TfddJOZ9xgjacqjAklOPGAQEZM9U97iFBSYjH7cXNqBeUw3dejQoXLFFVcIk1xToErCRSSjM2NiTB\/B8vj973+fcB3pSJiM3nQ\/WThOtw5PF61atRK60IzrpEM3rzKT0ZmhClZO4AKbbh1jk7goguC8yi8t4ekgp2Jjx\/QFnN3xsDkLw+IIhfz9FXTWb8+T1Q93JPi+YkH0E088YX7ZbVl+HRPVmYFc9MT6iLaUyC99bT2J6m3Ts4VZw4YNja97llB99913ggNEG+\/HMVGdGXek+4d1iv8zfsRw8PjNN98IftD80DXIdQSSnOjysPaO8RoLHg8Z5i+WiA3L1DEZ\/RhfYlstHkDczTBnLBN6J6oz3SBwZ90jayPZUJWF3FghLMz2W\/dE9UYvHBpitXBupUyZMhIK+fuDlqjOjOOUKxe5oxDkRpxtQ2k9lglqw\/nSwkAcVge\/InyOx58UNz4IOieq34svvmjWDF5yySUZGZh1YpWIzvx6jx8\/Prw2kq+lp512msycOdN4pnCW59d5InqjC2sz8ZjBuA1duZdeekkYII\/nCpq8qZZEdKYn0LJlS3n99dfNRxJ0xsEjHkBYYJ9qnbKtvMCS0\/HHHy8MDPLLzQ089thjhXkrQQE4ln4DBw40u9YwbpCfny9s4sAALZYIgmcGxhn8bksiOvupU6J1Jao36a666iq59NJLza4\/69evF6zWUMhfy4l2oYvX82ufD9L99a9\/Nc4c6UYz3kRX7\/rrr\/fd2kOXoElgySkUCgkT0hgkZHCTjRUwd4MCYCjkrR8O+RB+GfH2idcFp2ARVqtWzfemhELxdXYrxa84nkyxqNxxfl2HQonpHQqFBOuaeTl8FR01apTvX5gsJqFQYjrzTF9++eVmjhM6M6\/My+OHLTsTR55X1tnyPPhVf2DJyS8AtB5FQBEIJgJKTsG8L6qVIlDqEQiTU6lHQgFQBBSBQCGg5BSo26HKKAKKgEVAyckioUdFQBEIFAJKToG6HVmgjKqoCPiEgJKTT0AnUw071TAXJpE8yaRNpDybhomM+EVi4TILgG14vCN6o1O8dH7G45aE+WXMzk\/XLHfKpXyEcz\/bV1LrUnIqqXe2mO1i8e\/hhx8uuE3xe44T6\/tYKF3MJhTK3qVLF5k\/f37aZrkz\/4dZ9C1atChUr14UHQElp6JjV6ycLEZlJ2XrAZFzwviVHzFihNlp2VotTH7DiuHXH2d1OFBjUXS0tOykzOJXZqTzopCXlz2askxWZNY95TrTUi6eE4jn6LacKI9yyUM9bLqKuxJbB8SG\/y3iKN\/GoTO60wbqpE2UQ3lYG\/ipp+2EY7mBB7jgIZK67r\/\/fuOjyerDSn4W91IW9aCv1cHrSF5wxbqzOF1wwQXCMqloedyWINiQn3Kipdcwi0Dxj0pOxcewSCWwnooHHC+IzIJngSoeC\/Dvw2x4fulZ48YLOnbsWMENCLPMOcf1CrOJ3WlxYDdgwAD5+9\/\/bjxBPvnkk8KOOewJ6FYST5Es7v3HP\/4hy5YtE2daysVighA4ui0nyqNc8lAOs7Jx80F7qAcL4tZbbzU6\/PnPfxZIhWUZzPSfPHmy4BmUOgcNGiSQxLp168gmBQUFgkMzfEmxXIk247IFjHAXPHfu3LCHAcgsLy9PqBcsaAftQR9TWJw\/L7\/8sjAbG53Y9gwSAus42TTaRwSUnHwE210VLyhdDZa5sNzluuuucycRFq3iwoS1WrzgWBkQWbRuDy81ywxYqMuq9kaNGhl\/2rNmzYooF6vj6KOPFrwNhEIhIW3Hjh0F8ohI7AogDWnJQz1YSfh+4pykrBPDbQnX6L1t2zbBasIFCwRcp04d49GUpRu7d+8W2kS+GjVqCGspCYewcHWLCxHIkYWw+DKn7aTFD1KbNm2M10vqgUiPOOIIweIiPp785S9\/Md5JqYs60XHFihXxsmm8jwgoOfkItrMqLCM2J8UZGlYCXQvb\/XGmg5BYn8fqeiwafu15kZxp7Dm\/\/LiqxaMiXR1k3Lhx8u2339ok4SPdP+qFGG0gJACJ2OtoR6yQfnAlAAADa0lEQVSjb775Rkhr4yEOiAKrhzBnHNdWtmzZIlhUDBpDCDi0c9aHLtaFCDoTh5scmx9vqBUqVDCXtBVPFXQdaSdkiEcC8pkEcf44vaxSJ3XHyaLRPiOg5OQz4LY6uhN4W8AKgZQYM2G8BX9KNg1HXJbQrZk+fbpg7WBhQQbEuQUrgPEfujZ0AZHPPvtM8vLy3EkF6wXLzVotHLnGJ1JEYkcAL3KVKlUEjws2mO4Yi2whHxsW7fjwww+bYLpn+IPH\/aslNBPh+HPwwQcLVtKmTZvCodQDKRFAW\/v06RN27YLuc+bMMV1a4uMJ\/sFsGnxAYYligdkwr6Mzn1caDU8NAkpOqcEx6VIgG4gGS4TMWEN4\/+QXnBfP2d0hnpeHtM8++6yxhIgn3JkW6+Wrr74yvpcgG8iJwVvIjbROgRjp7vFCk5YuFNft2rVzJos4Rz+6dP\/5z3\/MIDI+iBh7oi70j8jgCqAuiA0fXVh1EBrXrmTGm8Bhhx0mTz\/9tFAHTvtIb9NBwm+99ZbpxlEm7fjb3\/4mWHU2Tawj3ULKpOwxY8ZIrVq1TDfPnQcPAVhj1AExMoDvTqPX6UFAySk9uMYt9bLLLjNjLXTX+GLHIPItt9wiONODOLjmZeNrVE5OjhBGt45zdnDp16+f4BOKcJuWMZs77rhDhg0bZhz8Q0yMBfH1y60QY0Fs94O\/I7pG1157rdxwww1CuDut+xorjzEbumb4IMKiwQqCuNxpnde4BmEsie4kY0mUgwXHYDQk4UyLRZWXlyd8RSM9vpEYI7NpaBMY4LsJ\/RkMp8vIuJZNE+tIfu4BZVMHdVEnmOJvi69y5O\/Vq5dgfbIBgVsHd1rSq6QOASWn1GGZVEn8IjN+xJcmuiR4QOTXm0IYaMaK4Wsd3Sybjk\/lnTt3liFDhgiWF4PfzrSM9TAw\/Oabb5ruDun5khYKRTpbC4VCxskZabB6+ALnTAsRUj9lopNTsNYgGr4yoj9f9CBG0mANOnebcZZDGtpJe9GROKwh6mnevLlgldAmysFSoQtJPOnRj7ZhnVF\/KBQK+25Cf8ojnryJCPOSyEPZ6IRu5KN+sEU3rvHtPXHiRGHvRLrgt912m6AvuLjTkj7LJVDqKzkF6naoMhaBnTt3ClMqmBdFl5ZuIKTAZ3+sS5tOjyUXgf8DAAD\/\/\/at3GAAAAAGSURBVAMAqJlgZZRaJ4sAAAAASUVORK5CYII=","height":178,"width":295}}
%---
%[output:7ceb089b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAScAAACyCAYAAAAAhgkFAAAQAElEQVR4AeydC7xNVR7H\/4dmEpOQJI9yC6FkNBF5lEnoLVMTRmpKJUqFuk0y3R4IvVTSQ83oQe+UiR5KiiYhUSSuiItqvBOSNL4r69h3n7PP495z9tnn3r+P\/917r+d\/\/fbev\/Nfa6\/1X2V+1X+KgCKgCAQQgTKi\/xQBRUARCCACSk4BvCmqkiKgCIgoOelTkBACmkgR8BsBJSe\/Edf6FAFFICEElJwSgkkTKQKKgN8IKDmlEPGpU6fKkUceKY888kgKS91X1Oeffy5NmjSRgQMH7gtM09nWrVule\/fucvLJJ8v333+fplpiFwuOtJd2x04ZrFj0RW+E80S1CwLmTl3RnTbwXDvDY56nMFLJKYVg+l0UpAF5QCI82Oms38+6aAcvxpgxY+See+6Rxo0bE+QpyerGy5auHxHuw7Bhw6RSpUoyceJEOfTQQz31DnoEuJ922mkyYMAA4X74rW\/WkBPWAg+UFR4wwOJh4OVEeBhsPL+6xFtx5ueF5oEmjnLIw0tAGZxzpFziowllkw7hl8XrxlEndT377LPGoqIuyiM9+ciPUJ4z3NaPDpxThtWXdAjXF1xwgaxatUo+\/vhjueKKK4T0xFmhPmf5xLvLQ0fSWCGPzW+P\/\/vf\/yRaXeiAbjYvZdk8HLkmjraCL+e2rcRTF2FWuCYcGTdunHnBjzvuOC5N29DdpqVe6keS0Y06wIpCR4wYEbZyCbdlc+SaNNGEOqmfdAjnhIEvZXM\/uC+dO3eW7777LqII0pKHvAg4uRO988475pkh3okZdThx4Jww8qMz6XneKN+W63zerrzyygiL2BlPfmd9F198MUUL98Oc+PgnK8gJsF555ZVCsPAQcDNsIA9E\/\/797aXw4Nl4bpIzPw\/OddddZx54m2H06NHmJeeasp555hlOI4QyKdtG\/PDDD3L11Vd7dn2oa\/DgwTa5SUd68tlAyqNce52qIy927dq1JT8\/3xS5bds2Wb16tdSsWVPKly9vXkwnLiRy40pYNPnxxx8FDGmfjacs7hXXHLnmnLaCL+dWaC912WuO9heal3f27NlhPYnLy8sL3x+uqZf6aRPXTomnmzMt57F0Id4pEAH1Ur8N5xyCXLdunQ3yPEbLD07gZTNRnvOZwYKEQIh348CzShhxVshLGVyDpfN5g\/TIQxxCuT169BDuEdeI83nE8sMK5H5QFvF+SeDJCUAmTJggBx54oLz22mvy9ddfy4033mjwefPNN82RP17x5AfYFi1ayIIFC4xwzg1CyIsQRvxjjz3GpXzwwQeFyItAHqwnn3ySUyEdunTp0sVYL+Q1EVH+oC9p27dvLzyIPDjkI4xyyOJsC9fxpFq1avLiiy8K5IPulPOHP\/yhULZqe9I0a9ZMaD84oCN1t23bVnipE8GVAg855JCIukKhkCE66qZcsEQXcMNa4Ehe9CKedFxbob3ue8YLwi80+Tdt2iToSZvAfc2aNaat1GPLW7hwoXmp3DiEQt66oQc6oQf3pXfv3hJLF9I5hfoRykEPhHNw5UeAsrkGC4iArlGs\/DzT4ABekKpNSzm2bHABEy8c7P21eamfvHfffbd53tHNhlGuTcdx5syZBkPCnc8jzzn18SPGjxn3Ax3I45cEnpwABGCOOeYYycnJMbjwYnPzuSn218oZ36pVK0NmPNArVqwQ8vNAYUkgnJuCHH\/si8AvBQ+LIyp8yguN5UHdlEMEDwA3FeLh2i2UhT42nAeY806dOnEQ8pGfckxAiv\/UrVvXtB8cqdvqwzW4OHGLhquXOpDj9OnTZfz48YKVycPPS0D67du3G+KyOEEwl156KVFGeOi5N7x05557rum+8GttIvf8QTfi9pya\/+SvUaOG+RGgHrCnvPnz50cdj4qlmynQ8SeeLo6k5hQMObHPC7qhC2GQHMdY4s4PedEOcKxQoYLJ6sSNdpvAPX+oi3TgDvmAQ6xneU+WsNVs9QU\/hDjE6oMVS5eOI+FBkMCTU6pA4sWDBJwCMaSq\/KCWY4nx7bffNtagk4yKozMvNeMdPNAQIC8JL1UyZZKefM57AknbF8ZZFuG8kPbF4iViXAWL0JmO86Lo5qUL5QVJGKIAczBy4lEcHfnBwoJz3gdIEDK0P8h07fjhLk49yeYNPDkBCMBgwi9fvty0z3aN6LJUrVrVhPGQI1zQNeCXl1+dOnXqmIFVrKxoDzLpExW3iet8CRi3SKQcXmTS8XBxZKyBh42HjmsE64yHgvbSbsKKKlibENLcuXONNWN\/QRPF1atesEboGkHyvCjWcrJ5uCYcnOgm2HAeeu6Njbfh9mgxstfcN4gIQrLdD+r1yo9eCGm8dLNlx9PFprNHqxvdMNqF2LZZa9imjXa0+e3957nh\/kP0zm5dtLyMD9FV5Ie2957uaDLPh9UXXBBbPvr88MMPQvfOhjmPWLFY2LxrWKTOuHSfB56cAKRbt26mX+zuArgfBh5ebjTkBXDEkx9geZD51SUe4WHnoSddosKDzMsN8aGLNaspF0mkHCwZfqnoxqAHR675KmIJw+pKHdQVr1weNtrOi+JOa3UmDQ8Z9ZMGXBLFlfRWKIe6KlasaLrO6E87CLNp+MGgbK4JtzhxbYV7wznx5LfCywoOYEI8gq7cQ+qmLNJSL2lISxqEeMqLpRvprFAGPw6xdLFp7ZH7jFAXuiCcY3lxbtN5HUlDWp5R2oG+pKVraLt1XEcT2soPtc2byPMBkVEfOlK3rc+Wz\/MAjmCBPlbAhTT80FMnRM+1nxJ4cgIMfiUAmXMr\/ILyq2ivuQF33HGHvTSD5jaeLoEzPzfjoYceEh76cIYET9y6UO\/9998vkEAiRTDGwGd1Z1quCUcfvrTYuL59+wovgr12H60l5w53X9sHEAsKS8rGu9tCuBtXwhB3XY0aNRLmwBAHnnQD0BVLj190vgBxTTwYOe8NYdwb6uLcCi8A4fYltL\/2xPNFypbHNWJxS1Y3ykdnykCo00sX4p3CfeZ+0yYbzjmD8tw\/G+Z1JA3PnrN+226vPDacvJb0CUNnnmt+zLBQCXMLeWI9Uzx3jBk69aFMng1rqZGfctxlp\/s6gpyY09K1a1fzpQQrIZ7EUpBf\/Z49ewq\/hhLl38SJE81gqGXrWNYMBOPsE\/NAuYvkZbFpANcZ78zPACQ3hXjKIY9NTzjxvGw8iKRxi7MsBiftjXOXRTrKokxnGTYd9SJc23jOCUP4tI4etg7KoTzKJT36EU9ajlwT7habL1oayiK\/FeonP2WR3tZtr0lHONc2LzpBHIRzDgHyC82vNS8QZcybN49ihW6EOdnzh7ooz4q9B+DJS2i7t3uSGvKnfJuWI\/ndcaSJpRtYIOhJGbZOyuLaig2nfLegH22yaTknjHTUjQ7OMMKd4qyfMmxd0fJajNGPMkhLHoQwZzzXhJOGtAjvHveC+wOBdejQQfgB4QscpE4atz6U6QynXK79lghy+vXXX+Wnn36SPn36SG5ubkwhjZfClMOnaq++LPkKCgqE+S8AisS6oaQvHZL9reQl40eNlvBi8ONDV4RfZ6wWwuMJ1h7dUPLFS6vx3gjQlcOy44eCc9sVZMyP++SdM\/MxEeQUCoXMfJJTTz1VzjzzzJhCGq8mwM6zZs2SNm3aeCWRtWvXCuMTngkSiADgeL9UCRSjSVKMAL\/e\/OA4BWuFX+lEqiId6SknkfSaJjoCWHT86DvvA+fWOoqeKxihEeS0\/\/77S7t27WS\/\/faLqyGT86IlYmCWfjW\/ml7kw5cJ5rrQn61Xr54ZW5k0aZJgcUUrU8MUAUWgdCEQQU506R5\/\/HFp3ry52PGin3\/+OSlU6M5hSlKGV8adO3cKXycGDRokixcvluHDh8vIkSPl008\/jchCt0DlyELjc4pHqcfD1+eBjxwRL2aaAyLICWsIC+b111833TvWq\/3pT3+S22+\/XVAwnmWzZMkS+fDDD81C1FAo5Kl+5cqVBeuKbl\/ZsmXNAPwJJ5xgllpEy4Qpmk1CG7JJX3RVnb82y6PAIp2SjTjHGsKhPemQCHKiEsiiYcOGMmTIEGHy3hNPPCF8xTv99NPNcotXX31VvKyplStXyowZM+TEE080zM6AJt075yRD6vjyyy\/N+jQsNa6tMGhqz\/WoCCgCpReBqOTkhON3v\/udMAHugQcekClTpggTspjnsXHjRmey8DmfHZ2\/OsyZ4HOyewCOgWymzPMV4ZdffjFLK5hXQV3hwrL45N1338067VVnf25ZNuLsDzKFa4lLTlhMkAvdLywn5rBATgcffHDhkhK4YkY2c5mYe8GYFIPhOOZiQJyxp5tvvlnq16+fQEmBS6IKKQKKQIoRiEpOWEU4rGLCFvNNsJggjjlz5ggWUNOmTYWuXyK6kB5rirT2s6a9ZmIYK7mxtOgK8pWQdCqKgCKgCESQE9ZN586d5cEHHxSO06ZNE8aYzjjjDDNLVyFTBBQBRcAPBCLIiQFpphIws5sZ4Exz90MRrUMRKA0IaBsTRyCCnFgPxyLL9evXxy0FKytuIk2gCCgCikAREIggJ8pgTdNdd90ljDPFEtKQXkURUAQUgVQjEEFOdOtuuOEGYdIVg+GxhDSpVkjLUwQUAUUABCLI6YADDjBr6+It+rXxFJKtonorAopAcBGIIKfgqqqaKQKKQGlCQMmpNN1tbasikEUIKDll0c1SVTOEgFabEQSUnDICu1aqCCgC8RCISU7MecJlSoMGDYQ1ce+9957cdNNNxkNBvII1XhFQBBSB4iDgSU74bbrvvvuMa5Snn35aDjvsMOOcHqfo9957r+zatas49WpeRUARyBIEVm7YIbvLVxWOfqrsSU5s871o0SJhztMRRxwhZcqUkXLlyskll1xinM6xONhPRcN16YkioAj4isDMZZtkS4fh8sc7\/ysz8jf5VrcnOXlpgG9xfDx5xWu4IqAIKAKpQMCTnNiYgI0T77zzTuO6FIdwbE3MTqD4YsLNbioU0DIUAUVAEYiGgCc5hUIhYTAcrwQXXXSR8e2NCxW6c3T1sKCiFahhikAwEFAtsh0BT3KiYbjSvfXWW4WxJ\/ag44ifJ7WaQEdFESgdCPg9EG5R9SSnLVu2mO2a8EoAQfHl7rbbbgt7KsBT5urVq205elQEFAFFIKUIeJITtfDF7o033hCIyl6zbRThbExw3nnnEayiCCgCpQSBw6uU862lnuTEFzk2vmRzA\/aXGzp0qNnK6dFHHxU8F4wYMcK48k1MU02lCCgCikByCHiSE7PD2XklJyenUIl169Y1M8SJd8cVSpjEBWX17NlT2JVF9J8ioAgoAnsQ8CQnnM4xIE73jdnie9IKRwbGOcd6ys\/P57RYQpkTJkwQfJYXqyDNrAgoAiUKAU9ygnz69esno0aNMrv3tm3bVlq2bCm33HKLMLVgxYoVZvZ4cdFYuHChQHjsi1fcsjR\/1iOgDQggAjPzN4a1CsSYE9oce+yxwr5y7MaSm5srI0eOFPaXO\/30CA8JDAAAEABJREFU06VBgwby9ttvk6zIsnXrVmE864orrhAmfRa5IM2oCCgCaUfAT2KiMZ6WE5F0uRgPYtFvs2bN5Oijj5ZVq1ZJ9+7dhU0QKlSoQLIiC905Zps3b948bhlHHnmkWdO3fPnyrDgWFBRkhZ5OPFVnf56tbMK5zrHNZcay39bTHV7Zvy91EEJMcho7dqxASuzMa6VLly5Sp06dYm+wuWTJEvnwww8FqykUCqFLTGFXYAbgs0Vq1aol2aKr1VN1zvHlnmUTzs9M3Nc7alW3csx3NNWRnuS0efNmmTZtmowfP16ef\/55M860dOlSufrqq+WYY44x0wmKo8zKlStNF\/HEE08UrKJXXnnFENXAgQOLU6zmVQQUgRQiMGH22nBprY6qFD7348STnH766SdTP7+qlSpVEjbZ3L59u5xzzjkyffp02bx5s4kv6p\/27dubBcVYRAgWGXOq7r777qIWqfkUAUUghQiwbGXC7G9NiWW2rZPWdSuZc7\/+eJITX+tY3Jufny+QE2NPCIpBTJa8uFZRBBSBkoUAxHT1hC\/DjSq3+PXwuV8nnuTEPKdu3bqZ6QJr1qyRo446Ss4991wj1atXlypVqqRURywmrKmUFqqF+YKAVlKyEICYznl4XnggvPWe7tzvV870vZGe5IQmTBmgC8cYE25S7rnnHnnggQfkrrvuEqwq0qgoAopAyUAAUhr+1grB4yXntIrpAw91a8ip7+JJTiz2ZQxo27Zthojo5jFRskmTJvLCCy8Ic5R811YrVAQUgZQjABFBSlhLw99aHi4fYnq9T1PhGA708SSCnBj05ivd5MmT5aWXXhKOeCaw8uqrr5pwSMtHPbUqRUARSCECEBKD3X33jCthKUFKhFEFZJTbMUc+u6VlaomJwpOQCHLCIsLzADO3v\/nmGzODe\/jw4ca3E0f8OPXo0UMOOeSQJKrRpIqAIpBpBCAfJyFBTFxbvSwpYS3ldqxjgzN2jCAnSOe5554zFtOAAQPM8YMPPhArLFm58MILJRSKP3EyY63SihWBUo4ARIRAPpBQlf7TzFgS54Q54bGkhKUEKXHtjM\/UeQQ5sZEBrlJ27NghnTt3Fo7ff\/+9OIV40mVKaa1XEVAE9iEACbFlE6QD+Zwzep4hIrprXBO+L\/VvZxBQ7p6uG1aSJaXfYoLzN4KcmGx5\/vnni12uEu1IPOmC0wzVJGkENEPWIQAJIZANpINYImIwm2vi7Fo4ZwMho27NqsvoPV\/eNtzbzown5e7puvk9sdKpU7zzCHKqVq2amQHOrG0vYXoB6eIVrvGKgCKQHAKQDwLJIBCOJSAsIYQw4hAvImJukpuMICbCktMoc6kjyMmpCl4J5s2bJ2xwwJo65jdBWM40eq4IKALJIbBmyy6x3TA+4UM20QiIcEtAEJa7FqwhBMLJ3dNFg3zooiGv921qrCTi3Pmy5TomOeHLqW\/fvrJr1y5p2rSpWQvHONSUKVOypX2qpyLgKwKQCOJFPgxMnz2uQGw3jE\/4sQgI5SEgBKKxJGTHiiAiSCl3TxeNeNKRJ3WSuZI8yYl1dPhbYkuoIUOGyGWXXWY2OMDxHOHEZ05trVkR8B8BSAeJRjxYPhAP3S4kGfKhJZAKAsEgEI6TgNwkFOSxItqTCvEkJyZjYjHluDY44Jpw4lOhgJahCGQaAQgHsaSDJUOXCoF0IJt4xBNt7MfZLogHgXjObvgH0+WyBGQHqC0BEU46CIg8znJK07knOZUvX17YHurLL78shAeeE1lXx3KWQhF6oQgEDAEIB0mWdCAlCAqBdCgjVtMgEMQOQkMsEAyWD+Imn7z2VYU0CAQUq+zSHOdJTuy8cumllwoTMXHLy86\/eK28\/PLLBW8FeC0ozcBlpu1aK0SBWMKZ9OVWcQ4qx7J0kiUd0IZ0EEs8uXsHniEdxEk8dhAaYrLEo+QDikUTT3KiuJNPPtnMEG\/Xrh2X0rBhQ3nttdcEbwUmQP8oAilAALJBLOFgsUAkCGSDeHWt8qau20NOy4U8WDkIZcVTC8JBLOlAJpAKAunQxYpGPLl7B54hHSRePRpfdAQ8yWnjxo3yyCOPyI8\/\/ii9evWSoUOHyvXXX298LBe9Os1ZGhCAHJAZ+ZsMaUAc1rqxhGPJJtpYDmnIg0A2COXFww6yQZyEk7vX0olHOsRDUAikQznx6tP49CLgSU5ly5Y1UwewknACx1c6CCu96mjpQUQAYkBikU0s6waysZ\/MLeFQXiJthSQQCAeBPJyE81iX6ma2czQrB8LJ3WvpkK8UkU4i0AY+jSc5VaxYUUaMGCGff\/65cS7HkW7eBRdcIO+9957o2rrA39uoCkIKCESDQBYIlg1dJIjESTRelo2bbBK1blAKskEs2XRrVl1yE7Bw7JhOroNw\/lSznFAW5aqULAQ8yck2ky92bA81atQoefHFF80XvNtuu81seGDT6DEzCEAyVtxEA8kgEA1iu1EcEebhIKRBIBsGlyGqZIiGlkMOSCyyiTWOoxYOKKq4EYhJTlhHTCUYNGiQsPElfpzYBRifTrhWcRem10VHwJIMx2SJxpINJINANJAMAtEglJuodhANkkqy0S5VouhrOouAJznhFqVTp05y3nnnCTut4LL3448\/FqYU1KpVS\/05WQQ9jqyfghBm5G\/yHBSOZtGkk2hikQ1fp+ZeU0cSHbtRsvG48RqcMgQ8yYlJlnyhY6yJnVFYW8cgeSI1f\/HFFwKxsVlm69atzeac0fJNnDjRbKhJOoQxLfxGRUsbhDDIBnFaNlgqCESDQC6M07B+inN31wlrBsGaQSgv0bZhzViJRTReXSg7ZqPdqEQR13SZRMCTnJiEyVgTY07JKMiaO0itT58+smTJEsnNzTUufteu3bdzqC2voKBARo8ebb4K4u0gKK5YIAwIBLHEA9FYcRIOaRCIBiGvbV+soyUZjskSDVZOLKJRqyYW8hqXLQh4klNRG7Bq1SrZvXu3YDGxzIXtxiG4devWRRQJYVWtWjUi3M8AyAThaxWWD1YPJAQpIZZ4SBNLL0gGsUTD+qlc1xeoaBaNEk0sVEtXnLa2MAIpJ6dGjRoJPsirVKki+IOaNWuW2VqqevXqhWpmcueaNWtk8ODBUq9ePeN5c9KkSSZPoYR7L+j2sa4vVTJz\/lK5+PHZYXemDCJj+eytrtChRsX9hE\/WEA7C2iiEOTaTLq4ljNW8+rfqgow6o5IMbFFOejXYIX+t\/6u0qLrdSM2yGwX5ZfNaSVUbUl0Olmyqy0x3earz8rQ+T7x3SKEXwqcLT3LC6wAD4BydunA9d+5c+fnnn53BEeeMVf3xj380a\/MYS6pUqVKhNDt37pQKFSoIXwIXL15sun4jR46UTz\/9tFA6e0G3D48IxZWP1x0g107eJIwJ8encls8Rywdh3g3jMlg6G+5tJ1\/ktZF3BrSUcZc3M9LvjMaCnN+6obRqUs\/MmnfrxUcDd1jQr1XnnKj3MtX3LZtw5r1DeD\/8lghyYvoAX+r4xWOHX44MUlv56quvzFKWeLPFGzduLPPnzxcc1llxNq5y5cpm2yk26mSgvW3btnLCCSfI7NmznclSdk63jG4bXTWnhWTJCCL67JaWgkBMEFTruoUJNWXKaEGKgCIQF4EIcmLjAjYwOOusswQLiWOLFi1Mt4sjM8QhHsglWukzZsyQgQMHCtMPiMckZG7U6tWruQwL86eYnmDT2YhUezuAlBhPYhzJkpKbkCAjNxFZffSoCCgCmUEggpzYuICvZowVQUwcMeusLF26VPLy8sxM8WgqMzkTi2nBggVmiQvTChYtWiTNmjUrlJyvgXg4oOuItca+eHQF3ekKZUryAmJiQJvxJJuVAWusJCUki4geFYFgIhBBThDFhg0bhDEiBqsZ1LZdOnuk20e6aE2qX7++9OvXT6655hoz0I1Hg6uuukqOP\/54s\/cd409Tp06V2rVrm8HwYcOGmXSMPTHBk\/zRyk02DGK6esKXxp0GebGW+HrGJ3jOCVNRBBSB4CIQQU5069hphbElund05dxCOOmiNSsUCsnZZ58tWERYWxy5DoVCYq0yvByQl3IZjyId3UHrN4q44oglJmc3Dmspt2Pmt1guTrs0bwARUJXShkAEOUEg48ePF8aJ6N5BHG4hnHRp06qYBdOVcxOTWkvFBFWzKwI+IxBBTj7Xn\/LqICY7xgQhYTFxTHlFWqAioAikFQFPcmJ8ifEhvra5hXDi06pZEQqnO+ckpoe6NlRfP0XAUbMoAkFAwJOc6LbRfbNdumXLlslHH30kV155peA6ha9yQWiAU4fhby0XCIqwVkdVEp0eABIqikB2IuBJTu7mhEIhYQlK7969hcHyeJMw3fnTfQ0p0aWjHrpxTBXgXEURUASyE4GEyck2j2UnW7ZsMXOYbFgQjkwbsHrQnbPnelQEFIHsRMCTnJjL1LVrV2FZiVM6dOhgpgR4zRDPBAxYTfbrHJMstTuXibuQTXWqrtmAgCc5MYOb8SX8MTnlqaeeEnyI4w4lKA10Wk03dswJilqqhyKgCBQDAU9ywhPmKaecIk2aNDH+w88880xhTV2tWrWEhbrFqDOlWZ1WE2NNajWlFF4tTBHIGAKe5IRGY8eOldNOO01wacL1+++\/L0wjmDJlCpeBkJnLNoX1yFWrKYyFnigC2Y6AJzmxvo41cMwWx60JDe3Zs6fZBXjChAmCO17Cii\/FK2HCJ7+5\/8Vqws1J8UrT3IqAIhAUBDzJadeuXab7VrNmzUK61q1bV4jD6VyhiAxczMjfJHYgnHlNGVBBq1QEFIE0IeBJTgcddJAwKP7yyy+Hpw3goYCFvPgEL1++fJpUSrxYZ5euW7PDEs+oKRUBRSDwCHiS0\/77729cn7z00kuCu12mE+D2ZMiQIdK\/f39DXJlu3cz8jUYFunQ6EG6gKMl\/tG2lDAFPcgIHPBPg0uSZZ54xWzyNGTNGcArHFzziMynOr3TapcvkndC6FYH0IOBJTuyMwiRMJmNCRkwlwP8SFlV6VEmuVLtUhVzapQMFFUWgZCHgSU4s\/G3YsKHMmTPHc7umTEKhXbpMoq91KwLpR8CTnFjYi\/\/vAQMGCBtjMuZkpWvXroJFlX71otfg7NIdXrlc9EQaqggoAlmNgCc58aWuT58+cv\/998utt95qxpzsMhaWtRCfqZZDTrbuVnUr21M9KgKKQAlCwJOcWL6CT2\/GmtxCOPGZwsE5hUAHwzN1F7ReRSC9CESQE901dk9hXzkGxG1XznkknHTFVY1uY6dOnQRPm61bt5Zp06YlVKSONyUEUyASqRKKQFERiCAnumvdu3c3juXovtmunPNIOOmKWin5WP4ydOhQoeu4ZMkS020cPny4rF3723IU0kQTunR2VriON0VDSMMUgZKBQAQ50V1jygD+mqJ5JTjppJMkFd26VatWye7duwWLCfcrDLoz83zdunUxkYWcbAIdb7JI6FERKHkIRJCTs4np9ErQqO1SKs0AABAASURBVFEjee6556RKlSrCshh2FoakcAXs1MGe0\/Vbvny5vDFnuQ2SoypsF8KCKAUFBYHVzQsv1Xm5L\/csm3DmvUPCL126TqKU60lOfnkl+Pzzz83yGKYs4I6lUqVKUdQUYaOFnJwcWbh+X\/T5rRsKYUEU\/F4FUa9YOqnOOb48T9mEM+8dsu+t8+\/Mk5zwPIBTuXR7JcCB3fz584VlMlZiNX\/lxh0mmvV05kT\/KAKKQIlEwJOc0u2VYMaMGTJw4ED56aefDLCYjqzlW716tbmO9ofxJoQ4HQwHBRVFoOQi4ElOrKFjSkG6vBKw7x0W04IFC4xLFqYVLFq0SJo1a+aJtiUmEuhgOCikSbRYRSAACHiSE7phydDVSodXgvr16xuXLNdcc43Uq1dPevXqJVdddZXgloW6o4lOvoyGioYpAiUTgZjkRJP5vJ8OrwShUEjOPvtswXkdA24cuQ6FQlQbVezkSyLVfxMoqCgCJReBuOQUxKbrYHgQ74rqVPoQSG+Ls4acGG\/SmeHpfRi0dEUgSAhkDTk5QdPBcCcaeq4IlEwEYpLTli1b5Pnnn5ebb765kLAGjjg\/IcFysvWpJwKLhB4VgZKLgCc5Mf\/oxhtvlMcee8ysgcs0BM4vdZnWJZj1q1aKQMlCwJOcNm\/eLHjD\/Ne\/\/iV33XWX4EHACh4KKlas6CsS+qXOV7i1MkUg4wh4khPeCQ4++OCMK+hWQL\/UuRHRa0WgZCLgSU5smolrlJEjR8rSpUvl+++\/DwuO5n755RdfEdEvdb7CrZWVXASypmWe5LR+\/Xp56KGH5I033pCOHTsKPp6snH\/++UK8X63cXb5quKraVcqFz\/VEEVAESi4CnuTE1lDTp083rkqYwe0Uwon3C5bd5fd1Lw+vcoBf1Wo9ioAikEEEPMkJnXACh78lBsSZPkD3Dt\/ihBPvlzgtJ51G4BfqWo8ikFkEYpITi34vu+wyWbZsmWAt4YAOTwXjx4\/3VWsnOflacfor0xoUAUXAAwFPcmIDggkTJsiDDz4oQ4YMETxU4kURK4pxKKYaeJSZ8uBdVY8Ol6kLfsNQ6IkiUKIR8CSn7du3C94wISQnArVr1zaXTNI0Jz7+OVwHw31EW6tSBDKLgCc5HXjggcL2T7gycU4byM\/PF6YZIH6pbgfE1fulX4hrPUFDoDTq40lOTMLs27evmRl+7rnnCgPjF154oXEQd9FFFxni8gMw1tTpmJMfSGsdikCwEPAkJ9TEydzbb78tN910k5x33nnGW+XkyZOFXVKI91vUG4HfiGt9ikDmEIhJTqjFGrozzzxTcKfbtWtX8XN+E\/VjOXFEdMwJFFQUgdKBQExyWrNmjUBIbIDJjrwNGjQwJMXyFb\/gWbV3Kyjqq125HIeMiyqgCCgC6UfAk5z4Gnf77bdLjRo1jJ9v5jp9+OGHgk\/xO++803zJS796Ik7LyY\/6tA5FQBEIBgKe5MQ8pk2bNpm95djGKRQKSfXq1eW6666Tb7\/9VpiQ6dWEgoICY3EdddRRctxxxwm7t0SbVT5x4kRhvzorjGWxwNhZ7soN28OXOscpDIWeKAIlHgFPcuJrHRMvsaDcKOBKhXh3ONfMj8LigmiWLFkizz33nIwbN05mzZpFdCGBxEaPHh1evzd9+vSIMa1VG3aYPDreZGDQP9mEgOpaLAQ8yYl5TN26dZPBgwcLX+gYf4I88I7Ztm1bgYTcVg6aMLMcq4tB9LJlywrjVXz1++yzz4guJGvXrpWqVfd5HCgUufdipWPMaW+QHhQBRaAUIOBJThs2bJAxY8bIypUrhSUrXfd8qYOoIKmHH35YcJuCuDGqXLmyyVezZk0ThTdNxqvoEpqAvX9+\/PFHoSzKZFNN3LFMmjRJ3N0\/O+akEzD3AqcHRaCUIOBJTowz0SX74IMPJJa4cWLAvEqVKoLVtHr1arn22muF2eannnpqoaQ7d+6UChUqyKBBg2Tx4sWC1wMc23366afhdJaYCPhk2huyfPnyrBG6rNmkL7qqzv48X9mEsx0P5h30WzzJCUV+\/vlneeqpp6R9+\/bCVAJ25GXRr3M5C+ncQr7HH39czjnnHDnmmGPk0UcfNQTlTIeFhTO7Nm3aGCKjq3jCCSfI7NmzncnC59df3kNY51dkycnxNf\/FF1\/sa32pwEV19ucZySacrR+38Ivo40lMcvr3v\/8tr7zyivTv31\/Gjh1runJ33323PPnkk54q0i1jUwTGqV577TVhM4Rog+f4hWJnF\/eAO1ZWtMIfHDa40Jc9y+hBPdKGoOrmpZfqfKQvz1i24ozefoonOTHm9N5775nu1hlnnCGNGzcWGP+RRx4RwomPpigm6zvvvCP33nuv1KpVK1oSE8aiYsjLLiym68j6vWbNmpl4\/vCFbsO97eSzW1pKwftPh7\/qWTbX49eKydeKgV\/vAe+kn1LGqzLcpYRCIaH75UzDdSgU8pyEyRyohQsXCmNMzl9oSI2ve0wxmDp1quB6hcHwYcOGCQPijD2xeWf9+vWd1ZlzSMqc6B9FIO0IaAVBQcCTnA466CBDTFOmTBE7xkSXDUsHq8er+4Xls2jRoohf9N69e5s5TExHYAwLAPhCh7dNmH\/GjBnCbi+EqygCioAi4ElO+++\/v5kNjjdMBqoZsGZQHK+YuFKJNo6kcCoCioAikCoEPMmJCuhuMfeIgXEGth944AEzrYBJlcSrKAKKgCKQLgRikhOVMm8JMmLGN90wLCrC\/RGtRRFQBEorAnHJqbQCo+1WBBSBzCIQWHJi8B1vBng1wI8Ui4m3bt2aWbQctSeqHx8TmMhKO\/DSwDIglu04ivLtNFGdnQrxQYQpJJnEPhm9P\/roI2ndurWZr9SpUyf54osvnM3x7TxRnVmjyuoInnGGUXr06GGWdfmmaIIVsV62Z8+exl13glmKnSyw5MQylueff16YC\/XJJ5+YNX5MCC12i1NUQKL68bK88MILph28KHzNhGjdk09TpFbMYhLV2RbC8iPmqzHj34Zl4piA3katVatWmXl59913n+AR46yzzhJWKjAtxiTw8U+iOjOtZt68efLf\/\/5XFixYYNwSsR2bj6rGrIofV77Q\/\/Of\/xSmCMVMnOLIwJLTtGnTjK9ylmXgKhiL49133xUWDKcYgyIVl6h+3NBTTjnFLGXhC2eHDh2El55foiJVXIxMiepMFbzQzOA\/6aSTIpYeEe+nJKo3E3mbNm0qfF3eb7\/95Morr5S8vDyzPMpPfakrUZ1Ji4A3R+TQQw\/lEAjhfWP6D2tleX79VCqQ5MSNWrFihdStWzeMBTeMWemAFQ7M0Eky+nXv3l2Y42VVXbp0qfDi+P1hIRmd0fX9998XunKsj+Q6U5KM3limO3bskNNPP13oQrNLEM9LKBTyVf1kdGbOH4vsmabDOlTmCLLLka8Kx6gMw4Av9TfddJOZ9xgjacqjAklOPGAQEZM9U97iFBSYjH7cXNqBeUw3dejQoXLFFVcIk1xToErCRSSjM2NiTB\/B8vj973+fcB3pSJiM3nQ\/WThOtw5PF61atRK60IzrpEM3rzKT0ZmhClZO4AKbbh1jk7goguC8yi8t4ekgp2Jjx\/QFnN3xsDkLw+IIhfz9FXTWb8+T1Q93JPi+YkH0E088YX7ZbVl+HRPVmYFc9MT6iLaUyC99bT2J6m3Ts4VZw4YNja97llB99913ggNEG+\/HMVGdGXek+4d1iv8zfsRw8PjNN98IftD80DXIdQSSnOjysPaO8RoLHg8Z5i+WiA3L1DEZ\/RhfYlstHkDczTBnLBN6J6oz3SBwZ90jayPZUJWF3FghLMz2W\/dE9UYvHBpitXBupUyZMhIK+fuDlqjOjOOUKxe5oxDkRpxtQ2k9lglqw\/nSwkAcVge\/InyOx58UNz4IOieq34svvmjWDF5yySUZGZh1YpWIzvx6jx8\/Prw2kq+lp512msycOdN4pnCW59d5InqjC2sz8ZjBuA1duZdeekkYII\/nCpq8qZZEdKYn0LJlS3n99dfNRxJ0xsEjHkBYYJ9qnbKtvMCS0\/HHHy8MDPLLzQ089thjhXkrQQE4ln4DBw40u9YwbpCfny9s4sAALZYIgmcGxhn8bksiOvupU6J1Jao36a666iq59NJLza4\/69evF6zWUMhfy4l2oYvX82ufD9L99a9\/Nc4c6UYz3kRX7\/rrr\/fd2kOXoElgySkUCgkT0hgkZHCTjRUwd4MCYCjkrR8O+RB+GfH2idcFp2ARVqtWzfemhELxdXYrxa84nkyxqNxxfl2HQonpHQqFBOuaeTl8FR01apTvX5gsJqFQYjrzTF9++eVmjhM6M6\/My+OHLTsTR55X1tnyPPhVf2DJyS8AtB5FQBEIJgJKTsG8L6qVIlDqEQiTU6lHQgFQBBSBQCGg5BSo26HKKAKKgEVAyckioUdFQBEIFAJKToG6HVmgjKqoCPiEgJKTT0AnUw071TAXJpE8yaRNpDybhomM+EVi4TILgG14vCN6o1O8dH7G45aE+WXMzk\/XLHfKpXyEcz\/bV1LrUnIqqXe2mO1i8e\/hhx8uuE3xe44T6\/tYKF3MJhTK3qVLF5k\/f37aZrkz\/4dZ9C1atChUr14UHQElp6JjV6ycLEZlJ2XrAZFzwviVHzFihNlp2VotTH7DiuHXH2d1OFBjUXS0tOykzOJXZqTzopCXlz2askxWZNY95TrTUi6eE4jn6LacKI9yyUM9bLqKuxJbB8SG\/y3iKN\/GoTO60wbqpE2UQ3lYG\/ipp+2EY7mBB7jgIZK67r\/\/fuOjyerDSn4W91IW9aCv1cHrSF5wxbqzOF1wwQXCMqloedyWINiQn3Kipdcwi0Dxj0pOxcewSCWwnooHHC+IzIJngSoeC\/Dvw2x4fulZ48YLOnbsWMENCLPMOcf1CrOJ3WlxYDdgwAD5+9\/\/bjxBPvnkk8KOOewJ6FYST5Es7v3HP\/4hy5YtE2daysVighA4ui0nyqNc8lAOs7Jx80F7qAcL4tZbbzU6\/PnPfxZIhWUZzPSfPHmy4BmUOgcNGiSQxLp168gmBQUFgkMzfEmxXIk247IFjHAXPHfu3LCHAcgsLy9PqBcsaAftQR9TWJw\/L7\/8sjAbG53Y9gwSAus42TTaRwSUnHwE210VLyhdDZa5sNzluuuucycRFq3iwoS1WrzgWBkQWbRuDy81ywxYqMuq9kaNGhl\/2rNmzYooF6vj6KOPFrwNhEIhIW3Hjh0F8ohI7AogDWnJQz1YSfh+4pykrBPDbQnX6L1t2zbBasIFCwRcp04d49GUpRu7d+8W2kS+GjVqCGspCYewcHWLCxHIkYWw+DKn7aTFD1KbNm2M10vqgUiPOOIIweIiPp785S9\/Md5JqYs60XHFihXxsmm8jwgoOfkItrMqLCM2J8UZGlYCXQvb\/XGmg5BYn8fqeiwafu15kZxp7Dm\/\/LiqxaMiXR1k3Lhx8u2339ok4SPdP+qFGG0gJACJ2OtoR6yQfnAlAAADa0lEQVSjb775Rkhr4yEOiAKrhzBnHNdWtmzZIlhUDBpDCDi0c9aHLtaFCDoTh5scmx9vqBUqVDCXtBVPFXQdaSdkiEcC8pkEcf44vaxSJ3XHyaLRPiOg5OQz4LY6uhN4W8AKgZQYM2G8BX9KNg1HXJbQrZk+fbpg7WBhQQbEuQUrgPEfujZ0AZHPPvtM8vLy3EkF6wXLzVotHLnGJ1JEYkcAL3KVKlUEjws2mO4Yi2whHxsW7fjwww+bYLpn+IPH\/aslNBPh+HPwwQcLVtKmTZvCodQDKRFAW\/v06RN27YLuc+bMMV1a4uMJ\/sFsGnxAYYligdkwr6Mzn1caDU8NAkpOqcEx6VIgG4gGS4TMWEN4\/+QXnBfP2d0hnpeHtM8++6yxhIgn3JkW6+Wrr74yvpcgG8iJwVvIjbROgRjp7vFCk5YuFNft2rVzJos4Rz+6dP\/5z3\/MIDI+iBh7oi70j8jgCqAuiA0fXVh1EBrXrmTGm8Bhhx0mTz\/9tFAHTvtIb9NBwm+99ZbpxlEm7fjb3\/4mWHU2Tawj3ULKpOwxY8ZIrVq1TDfPnQcPAVhj1AExMoDvTqPX6UFAySk9uMYt9bLLLjNjLXTX+GLHIPItt9wiONODOLjmZeNrVE5OjhBGt45zdnDp16+f4BOKcJuWMZs77rhDhg0bZhz8Q0yMBfH1y60QY0Fs94O\/I7pG1157rdxwww1CuDut+xorjzEbumb4IMKiwQqCuNxpnde4BmEsie4kY0mUgwXHYDQk4UyLRZWXlyd8RSM9vpEYI7NpaBMY4LsJ\/RkMp8vIuJZNE+tIfu4BZVMHdVEnmOJvi69y5O\/Vq5dgfbIBgVsHd1rSq6QOASWn1GGZVEn8IjN+xJcmuiR4QOTXm0IYaMaK4Wsd3Sybjk\/lnTt3liFDhgiWF4PfzrSM9TAw\/Oabb5ruDun5khYKRTpbC4VCxskZabB6+ALnTAsRUj9lopNTsNYgGr4yoj9f9CBG0mANOnebcZZDGtpJe9GROKwh6mnevLlgldAmysFSoQtJPOnRj7ZhnVF\/KBQK+25Cf8ojnryJCPOSyEPZ6IRu5KN+sEU3rvHtPXHiRGHvRLrgt912m6AvuLjTkj7LJVDqKzkF6naoMhaBnTt3ClMqmBdFl5ZuIKTAZ3+sS5tOjyUXgf8DAAD\/\/\/at3GAAAAAGSURBVAMAqJlgZZRaJ4sAAAAASUVORK5CYII=","height":178,"width":295}}
%---
%[output:50391e58]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Unable to find system or file 'three_phase_dab'."}}
%---
