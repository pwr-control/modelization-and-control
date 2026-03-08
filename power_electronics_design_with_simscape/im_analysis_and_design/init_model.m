clear;
[model, options] = init_environment('inv_im_ekf');
%[text] ### Global timing
% simulation length
simlength = 3.25;

% switching frequency and tasks timing
fPWM = 4e3;
fPWM_AFE = fPWM;
tPWM_AFE = 1/fPWM_AFE;
TRGO_AFE_double_update = 1;
if TRGO_AFE_double_update
    ts_afe = tPWM_AFE/2;
else
    ts_afe = tPWM_AFE;
end

fPWM_INV = fPWM;
tPWM_INV = 1/fPWM_INV;
TRGO_INV_double_update = 1;
if TRGO_INV_double_update
    ts_inv = tPWM_INV/2;
else
    ts_inv = tPWM_INV;
end

fPWM_DAB = 3*fPWM;
tPWM_DAB = 1/fPWM_DAB;
TRGO_DAB_double_update = 0;
if TRGO_DAB_double_update
    ts_dab = tPWM_DAB/2;
else
    ts_dab = tPWM_DAB;
end

fPWM_CLLC = 5*fPWM;
tPWM_CLLC = 1/fPWM_CLLC;
TRGO_CLLC_double_update = 0;
if TRGO_CLLC_double_update
    ts_cllc = tPWM_CLLC/2;
else
    ts_cllc = tPWM_CLLC;
end

s=tf('s');
z_afe = tf('z',ts_afe);
z_inv = tf('z',ts_inv);
z_dab = tf('z',ts_dab);
z_cllc = tf('z',ts_cllc);


% deadtimes
dead_time_AFE = 0e-6;
dead_time_INV = 0e-6;
dead_time_DAB = 1e-6;
dead_time_CLLC = 1e-6;

% minimum pulse
minimum_pulse_time_AFE = 1e-6;
minimum_pulse_time_INV = 1e-6;
minimum_pulse_time_DAB = 1e-6;
minimum_pulse_time_CLLC = 1e-6;

% delays
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayINV_modB=0;
delayDAB_modB=0;

% maximum simulation and simscape step
tc = ts_afe/100;

% t_misura = simlength - 0.025;
t_misura = 0.6;
Nc = ceil(t_misura/tc);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);
Ns_dab = ceil(t_misura/ts_dab);
Ns_cllc = ceil(t_misura/ts_cllc);

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

%[text] ### Local time allignment to master time
kp_align = 0.6;
ki_align = 0.1;
lim_up_align = 0.2;
lim_down_align = -0.2;
%[text] ### Enable one/two modules
number_of_modules = 1;
enable_two_modules = number_of_modules;
%[text] ### Settings for speed control or wind application
use_torque_curve = 1; % for wind application
use_speed_control = 1-use_torque_curve; %
use_mtpa = 1; %
use_psm_encoder = 0; % 
use_load_estimator = 0; %
use_estimator_from_mb = 0; %mb model based
use_motor_speed_control_mode = 0; 

use_dq_pll_fht_pll = 1; % 
use_dq_pll_ddsfr_pll = 0; % 
use_dq_pll_mod1 = 0; % 
use_dq_pll_ccaller_mod1 = 0; % 
use_dq_pll_ccaller_mod2 = 0; % 

use_dq_pll_mode1 = use_dq_pll_mod1;
use_dq_pll_mode2 = use_dq_pll_ccaller_mod1;
use_dq_pll_mode3 = use_dq_pll_ddsfr_pll;
use_dq_pll_mode4 = use_dq_pll_fht_pll;
%[text] ### Settings for CCcaller versus Simulink
use_ekf_bemf_module_1 = 1;
use_observer_from_simulink_module_1 = 0;
use_observer_from_ccaller_module_1 = 0;
use_observer_from_simulink_module_2 = 0;
use_observer_from_ccaller_module_2 = 0;

use_current_controller_from_simulink_module_1 = 0;
use_current_controller_from_ccaller_module_1 = 1;
use_current_controller_from_simulink_module_2 = 0;
use_current_controller_from_ccaller_module_2 = 0;

use_moving_average_from_ccaller_mod1 = 1;
use_moving_average_from_ccaller_mod2 = 0;
%[text] ### Settings average filters
mavarage_filter_frequency_base_order = 2; % 2 means 100Hz, 1 means 50Hz
dmavg_filter_enable_time = 0.025;
%%
%[text] ### Grid Emulator Settings
grid_nominal_power = 1600e3;
application_voltage = 690;

if application_voltage == 690
    % trafo data
    us1 = 690; us2 = 690; fgrid = 50;
    eta = 95; ucc = 6;
    p_iron = 1800;
elseif application_voltage == 480
    % trafo data
    us1 = 480; us2 = 480; fgrid = 60;
    eta = 95; ucc = 4.5;
    p_iron = 1400;
else
    % trafo data
    us1 = 400; us2 = 400; fgrid = 50;
    eta = 95; ucc = 4.5;
    p_iron = 1000;
end

n2 = 14; n1 = floor(n2*sqrt(3));
core_area = 0.05; core_length = 2.5;
mu0 = 4*pi*1e-7; mur = 10e3;

% two simple calculation:
Lm1 = (n1^2 * mu0 * mur * core_area) / core_length;
% Lm1 = u1_nom/sqrt(3)/i1m/(2*pi*fgrid);
i1m = us1/sqrt(3)/Lm1/(2*pi*fgrid);

% rererence for the voltage sequence
up_xi_pu_ref = 1; up_eta_pu_ref = 0; un_xi_pu_ref = 0; un_eta_pu_ref = 0;

grid_emu = grid_three_phase_emulator('Dyn11-690V-690V-1600kW', grid_nominal_power, application_voltage, us1, us2, fgrid, ...
                eta, ucc, i1m, p_iron, n1, n2, core_area, core_length, mur, ...
                up_xi_pu_ref, up_eta_pu_ref, un_xi_pu_ref, un_eta_pu_ref);

%%
%[text] ## Global Hardware Settings
afe_pwr_nom = 250e3;
inv_pwr_nom = 250e3;
dab_pwr_nom = 250e3;
cllc_pwr_nom = 250e3;
fres_dab = fPWM_DAB/5;
fres_cllc = fPWM_CLLC*1.25;

hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, fPWM_AFE); %[output:4afd7a36]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, fPWM_INV); %[output:9269bdd1]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, fPWM_DAB, fres_dab); %[output:0b84da60]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, fPWM_CLLC, fres_cllc); %[output:0c8e3447]
%[text] ## AFE Settings and Initialization
%[text] ### Behavioural Settings
time_gain_afe_module_1 = 1.0;
time_gain_inv_module_1 = 1.0;
time_gain_afe_module_2 = 1.0;
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

use_vector_control = 1;
use_uf_control = 0;
%[text] ### FRT Settings
test_index = 25; % type of fault: index
test_subindex = 4; % type of fault: subindex
enable_frt_1 = 1; % faults generated from abc
enable_frt_2 = 0; % faults generated from xi_eta_pos and xi_eta_neg
start_time_LVRT = 0.75;
asymmetric_error_type = 1;
frt_data = frt_settings(test_index, test_subindex, asymmetric_error_type, enable_frt_1, enable_frt_2, start_time_LVRT);
grid_fault_generator;
%[text] ### Reactive current limits for grid support
i_grid_pos_eta_lim = 1;
i_grid_neg_xi_lim = 0.5;
i_grid_neg_eta_lim = 0.5;
%[text] ### Reactive current Limits - Red. Dyn. grid support
i_grid_pos_eta_red_lim = 0.1;
i_grid_neg_eta_red_lim = 0.1;
i_grid_neg_xi_red_lim = 0.1;
%[text] ### Grid voltage derivate implemented with double integrator observer
Aso = [0 1; 0 0];
Asod = eye(2)+Aso*ts_afe;
Cso = [1 0];
omega_rso = 2*pi*50;
p2place = [-1 -4]*omega_rso;
p2placed = exp(p2place*ts_afe);
Kd = (acker(Asod',Cso',p2placed))';
l1 = Kd(2) %[output:22dbc57b]
l2 = Kd(1) %[output:41924656]
%[text] ### Linear double integrator observer
Aso = [0 1; 0 0];
Asod = eye(2)+Aso*ts_afe;
Cso = [1 0];
omega_rso = 2*pi*50;
p2place = [-5 -20]*omega_rso;
p2placed = exp(p2place*ts_afe);
Kd = (acker(Asod',Cso',p2placed))';
kv = Kd(2)/ts_afe;
kx = Kd(1)/ts_afe;
%[text] ### Current sensor endscale, and quantization
adc_quantization = 1/2^11;
adc12_quantization = adc_quantization;
adc16_quantization = 1/2^15;
Imax_adc = 1049.835;
CurrentQuantization = Imax_adc/2^11;
%%
%[text] ### Voltage sensor endscale, and quantization
Umax_adc = 1500;
VoltageQuantization = Umax_adc/2^11;
%[text] ### DClink Lstray model
parasitic_dclink_data; %[output:3dfc571b]
%[text] ### DClink voltage control parameters
Vdc_norm_ref = 1;
kp_vs = 0.85;
ki_vs = 35;
%%
%[text] ### AFE current control parameters
%[text] ### Resonant PI
kp_afe = 0.2;
ki_afe = 45;
delta = 0.015;
res = s/(s^2 + 2*delta*grid_emu.omega_grid_nom*s + (grid_emu.omega_grid_nom)^2);

Ares = [0 1; -grid_emu.omega_grid_nom^2 -2*delta*grid_emu.omega_grid_nom];
Bres = [0; 1];
Cres = [0 1];
Aresd = eye(2) + Ares*ts_afe;
Bresd = Bres*ts_afe;
Cresd = Cres;
%%
%[text] ### Grid Normalization Factors
pll_i1 = 80;
pll_p = 1;
pll_p_frt = 0.2;
Vmax_ff = 1.1;
ixi_pos_ref_lim = 1.65;
ieta_pos_ref_lim = 1.0;
ieta_neg_ref_lim = 0.5;
%%
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso = [-1 -4]*omega_rso;
Lrso = acker(Arso',Crso',polesrso)';
Adrso = eye(2) + Arso*ts_afe;
polesdrso = exp(ts_afe*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:165672fd]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1;
pll_p_ddsrt = pll_p;
omega_f = grid_emu.omega_grid_nom;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
tau_ddsrf = 1/omega_f;
%%
%[text] ### PLL FHT
pll_i1_fht = pll_i1;
pll_p_fht = pll_p;
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega_fht0 = grid_emu.omega_grid_nom;
delta_fht0 = 0.05;
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:3a052c4f]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:8e63741a]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:0c55784a]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:2b7fc8fb]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid_emu.omega_grid_nom;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:83f70987]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:081b02ac]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:24a492ca]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:91c2d5cb]
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;
kp_rc_pos_grid = kp_rc_grid;
ki_rc_pos_grid = ki_rc_grid;
kp_rc_neg_grid = kp_rc_grid;
ki_rc_neg_grid = ki_rc_grid;
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = 2*pi*rms_perios/grid_emu.omega_grid_nom/ts_afe;
rms_perios = 10;
n10 = 2*pi*rms_perios/grid_emu.omega_grid_nom/ts_afe;
%%
%[text] ### Online time domain sequence calculator
w_grid = grid_emu.omega_grid_nom;
apf = (s/w_grid-1)/(s/w_grid+1);
[napfd, dapfd]=tfdata(c2d(apf,ts_afe),'v');
apf_z = tf(napfd,dapfd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(napfd,dapfd);
ap_flt_ss = ss(A,B,C,D,ts_afe);
% figure;
% bode(ap_flt_ss,options);
% grid on
%%
%[text] ### INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.125;
%[text] ### IM Machine settings
im = im_calculus(); %[output:3bc87194]
%[text] ### PSM Machine settings
psm = psm_calculus(); %[output:54e494a4]
n_sys = psm.number_of_systems;
run('n_sys_generic_1M5W_torque_curve');
torque_overload_factor = 1;

% load
b = psm.load_friction_m;
% external_load_inertia = 6*psm.Jm_m;
external_load_inertia = 1;
%[text] ### Simulation parameters: speed reference, load torque in motor mode

%[text] ### Double Integrator Observer Inverter Side
Aso = [1 ts_inv; 0 1];
Cso = [1 0];
% p2place = exp([-10 -50]*ts_inv);
p2place = exp([-50 -250]*ts_inv);
Kobs = (acker(Aso',Cso',p2place))';
kg = Kobs(1) %[output:37bb0420]
kw = Kobs(2) %[output:95289131]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/psm.Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/psm.Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:8400c6b8]
luenberger_l2 = Klo(2) %[output:17818007]
luenberger_l3 = Klo(3) %[output:29702271]
omega_flt_fcut = 10;
phase_compensation_omega = 0; % for generator mode
%[text] ### Control settings
id_lim = 0.35;
%[text] ### Rotor speed control
kp_w = 2.5;
ki_w = 18;
iq_lim = 1.4;
%[text] ### Current control
kp_i = 0.5;
ki_i = 18;

kp_id = kp_i;
ki_id = ki_i;
kp_iq = kp_i;
ki_iq = ki_i;

kp_inv_d = kp_i;
ki_inv_d = ki_i;
kp_inv_q = kp_i;
ki_inv_q = ki_i;

CTRPIFF_CLIP_RELEASE = 0.001;
%[text] ### DAB current and voltage control
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 0.25;
ki_v_dab = 45;
%[text] ### Field Weakening Control 
kp_fw = 0.05;
ki_fw = 1.8;
%[text] ### BEMF observer
emf_fb_p = 0.2;
emf_p = emf_fb_p*4/10;

emf_fb_p_ccaller_1 = emf_fb_p;
emf_p_ccaller_1 = emf_fb_p_ccaller_1*4/10;

emf_fb_p_ccaller_2 = emf_fb_p;
emf_p_ccaller_2 = emf_fb_p_ccaller_2*4/10;
% omega_th = 0.25;
omega_th = 0;
%[text] ### EKF BEMF observer
kalman_psm; %[output:6b1f64f5]
k_kalman = 0;
%[text] ### Motor Voltage to Udc Scaling
motorc_m_scale = 2/3*hwdata.inv.udc_nom/psm.ubez;
inv_m_scale = motorc_m_scale;
%%
%[text] ### Settings Global Filters
filters = setup_global_filters(ts_afe, ts_inv, ts_dab, tc);
%[text] ### Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] ### HeatSink settings
heatsink_liquid_2kW;
%[text] ### DEVICES settings (IGBT)
% infineon_FF650R17IE4D_B2;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.inv = device_igbt_setting_inv(fPWM_INV, hwdata.inv.udc_nom);

% infineon_FF650R17IE4;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.afe = device_igbt_setting_afe(fPWM_AFE, hwdata.afe.udc_nom);
%[text] ### DEVICES settings (MOSFET)
infineon_FF1000UXTR23T2M1;
mosfet.inv = device_mosfet_setting_inv(fPWM_INV, hwdata.inv.udc_nom);

infineon_FF1000UXTR23T2M1;
mosfet.afe = device_mosfet_setting_afe(fPWM_AFE, hwdata.afe.udc_nom);

wolfspeed_CAB760M12HM3
mosfet.dab = device_mosfet_setting_dab(fPWM_DAB, hwdata.dab.udc1_nom);
%[text] ### DEVICES settings (Ideal switch)
silicon_high_power_ideal_switch;
ideal_switch = device_ideal_switch_setting(fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.afe = device_ideal_switch_setting(fPWM_AFE, hwdata.afe.udc_nom);
ideal_switch.inv = device_ideal_switch_setting(fPWM_INV, hwdata.inv.udc_nom);
%[text] ### Setting Global Faults
time_aux_power_supply_fault = 1e3;
%[text] ### Lithium Ion Battery
nominal_battery_voltage = grid_emu.udc_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab);
lithium_ion_battery_2 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab);
%[text] ### C-Caller Settings
open_system(model);
Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dsmavgflt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'mavgflts_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
Simulink.importExternalCTypes(model,'Names',{'rpi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'linear_double_integrator_observer_output_t'});

%[text] ### Remove Scopes Opening Automatically
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
%[output:4afd7a36]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:9269bdd1]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:0b84da60]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\n","truncated":false}}
%---
%[output:0c8e3447]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.880000e-05 [H] | Internal Tank Cs: 1.407239e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:22dbc57b]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:41924656]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:3dfc571b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAUwAAADICAYAAACDONglAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQeUFNXSLmDBhQdLlihZoiAqyQdKFJ6ABCUjWSQs6YGPIBnJiogoKCLpSJTHCvhAEHBR+AVlSQpI0gVJEpZF0kryP1+tPQ7DhJ6Znr4903XP4TC703373u9Wf1t1q25Vmj\/\/\/PNPkiYICAKCgCDgE4E0Qpg+MZILBAFBQBBgBIQwRRAEAUFAENCJgBCmTqDkMkFAEBAEhDBFBgQBQUAQ0ImAEKZOoOQyQUAQEASEMEUGBAFBQBDQiYAQpk6g5DJBQBAQBIQwRQYEAUFAENCJgBCmTqDkMkFAEBAEhDBFBgQBQUAQ0ImAEKZOoOQyQUAQEASEMEUGBAFBQBDQiYAQpk6g5DJ9CBw9epRiY2MpY8aMlDZtWkpKSqIKFSrQqFGjKHfu3Lo6Wb58ORUuXJiqVavm8\/odO3ZQjx49qEiRIo7ntWnThjp37sw\/z5o1i5o1a0ZFixb12ZdcIAj4QkAI0xdC8r1fCIAwFy5cSMOHD2fSvHfvHr377rtMgM2bN9fVl7+EuXTpUpo8eTI\/748\/\/qB58+bRrVu3mLijoqJ0PdPTRUjmhX8gX2mCgBCmyIChCLgSJghsypQpVLNmTapevTotWrSIli1bxs+EJtixY0f+DJJdsmQJa6HFixenF154gapWrUrr1q2juXPn0p07d6hhw4asOYIYtQYN05kw8fuzZ8\/S6NGj6fXXX+d7O3XqRPny5WPi\/uabb5hM27dvz89OSUmh6dOnU3x8PGvC6LtLly60e\/duOnjwICUmJtKAAQP4+RMnTmRCxhjRf7Zs2fgZjz32GK1fv56KFStG9evXpwULFtCVK1dowoQJPAdpkYOAEGbkrKUlZgLCBOGcOXPGMZ5GjRrRpEmTKCEhgbZu3cokgzZjxgwqV64ckxR+P2TIELp9+zYNGzaMCe2hhx7i3\/fp04fSpUvHZJsrVy5Cf94I8\/r160xWbdu2JWirIMxLly4x+bVu3ZquXr3KGmnfvn3pyy+\/5Oe0atWKfvvtNxo5ciQNHjyYCXPbtm087jRp0tAHH3xA7dq1Y+LduHEjnTp1ik39Xr16Ue\/evenZZ5\/lazNlykT9+vWjPXv20BdffMFzClbLtcTCyiAYASFMEQRDEXBnkkMDBFlFR0ezxlWjRg1+JrRDECIatE\/t99BAsScJDW\/8+PH3jQ\/7lSAhfwkTWuvx48f53759++jrr7+mqVOn0ooVK5icS5Ysyab3e++9R\/\/617+YMNFAsGgg2QMHDtCRI0dYSy1RogR1796dxo4dy9pmjhw5WJPW5uGKg6EgS2fKEBDCVAZ9ZD7YHVFov4MmWbduXYczBxrczp072dx1R5i7du1i81cjLZjSd+\/e1WWSw3wGsUIzhIb5ww8\/8D9ondmzZ2ftFv1qGuijjz7K48DvmzRpch9hXrhwgcaNG8dbApUqVaLDhw\/T9u3b3RImth7grBLCjEz5FsKMzHVVNitXooDWBi3u2LFj9OSTT7LW2L9\/fx4fSA0Ec\/PmTfruu+\/YVMfnQYMGUdeuXZnA4uLiWItLnz49a3\/OGqqmpbo6fXAdTGPsd8I0B2GuWrXKQcoYA5xS0DA3bNjApAyT\/PTp0zwGmOXOGibmNH\/+fL4H5vU777zD43enYQphKhM9Ux4shGkKzPZ5iGtYEZwqpUuXZiKCZufO6QOt8aOPPmJyhOmMfcLnn3+etTmQLZwoaJqTyHlP0FtYEe7RCBPOGuxNYj8Se4\/YY82aNSub4zD7Dx06xGZ55syZqVu3bvcRJu6dOXMmrV27lkqVKsUmO8YFDRZjczbJhTAjW9aFMCN7fWV2PhDYtGkTFSpUiMny4sWLNG3aNNZw4VySJgi4IiCEKTJhawR+\/vlnNvmxTwlvOTThypUr2xoTmbxnBIQwRToEAUFAENCJgBCmTqDkMkFAEBAEhDBFBgQBQUAQ0ImAEKZOoOQyQUAQEASEMEUGBAFBQBDQiYAQpk6g5DJBQBAQBIQwRQYEAUFAENCJgBCmTqDkMkFAEBAEhDBFBgQBQUAQ0ImA5QgTyRe+\/fZbPquL\/IRoefLk4eQITz\/99H2ZanTOUS4TBAQBQcAQBCxDmMhqs3fvXk7HVbt2bU6RFRMTw5MEiSJ7zOrVqzlZAr6XJggIAoKA2QhYhjCvXbvGyV3Lly\/vsX4K6sMgF2GZMmXMxkmeJwgIAoKAtTKugxBPnDjBtVKQz3DLli30008\/cXqwOnXqcHIEaYKAICAIqELAMhomksW+\/\/77tGbNGq7fApIEaaKoFPY0c+bMyeVUkc9QmiAgCAgCKhCwDGFevnyZ3nrrLXrttde4kh9Sbr3xxhucDRvm+ttvv81Fq5CEVpogIAgIAioQsAxhJiUlcdZtVNyDk8e5uBR+RolUlARAsSlpgoAgIAioQEAIUwXq8kxBQBAISwQsRZioR42aLtjPjI+Pp1q1anHcJWpVw4OOus+iYYalnMmgBYGIQMAyhIlCU\/v37yf8767BQ16hQgXxlEeE2MkkBIHwRMAyhImQoitXrhAC2N01eMdR5S9t2rThibSMWhAQBNwicOdCIt08EE9ZanW2PEKWIUx4yUeNGkWnTp2i33\/\/nSv4FSlShAG8dOkStWjRgl555RUug2pkK1asmJHdSV+CgCCgA4G8Ge7Q41lSqEKWm9Qg5zU6dyuKXv7hEZ93omidymYZwtRAgEk+e\/ZsatmyJRUoUIB\/DRJduXIl9erVy3CTHIRp9iKoeKZKIZNnhw6BcJMlaJNXv1pIV+NTa81nLFeLosvV1KVdWmGuliNMhBfhPHn\/\/v3pH\/\/4B4N6\/fp1mjFjBvXs2dNvpw9M\/M8\/\/5zvxz4oQpOaNGniMO1VLMIvv\/xCRYsWDd1bJD3bBoFwkCXN5L4av5BSDsRTVO4iTJBZanfiz3qbinfVdWyWI8xbt27RxIkTmVAaNmzI49WOSA4dOtRvDfPQoUMc34m4TuyDjhkzhjp06ECPP\/44961iEcJByPUKsVynFgGryhJI8s75RAJJQpsEMUKbzFKrE0WXqxUQaCreVcsTpqZRLlu2jDVDmOg4Htm1a1dH9iJ\/0F6+fDk7ktq0acO3oV807WcVi2BVIfcHV7teu\/T7c3QyKYWnfzLppgOGX\/\/6nTMu1UtkpyEN9GtQgWBqNVlyNbmjHoY22UmXye1r\/ireVcsSJojx6tWrlCtXLq+4wRnk6xrnDqZMmUKVK1fm5B1o27Zto+3bt9OQIUP45xwDv\/K1TvK9IOBAIH9M1H1o5MuS+rPr78\/8focSTqcSK757oXRmerVqNsORxP5+wYIFDe\/Xrw4vnyI6vpP+TPgv0fEdRNkLElV6idJUein1swGtbt263IvZ\/gbLEiaC1ZF4AzkxkSwYAewIWkfD0ciDBw9yPsyKFSvSiy++qHsJQJjVq1enGjVquCVMFX+1rKYV6AZTLvQbAWij0EqnbPiFCuWIpraV8xmqdaqUJexHuprcuWPn+42R3htUvKuWJUxtYBcuXCCY0XFxcQRhQEPmombNmnFokb8nfVxNcPwM50\/z5s25bxWLoFLI9QqnXGc8AlM2JDJx1iiejdbEPmHIA8yWJWeTG5+xH2mUye0LEBXvquUJ0xdo\/n4Pp8+iRYto+PDhfOuECROoY8eOjiTEKhbBbCH3FzO5PnQIQONsMmsPP+C9NmWoRongzHQzZMkoL3ewqKp4V21HmFpYEUKV8Dk2Npa971peTRWLYIaQByuccn\/oEABp9ll6iLYdT6b325ahtpXzBvywUMoSiPLyirGGebkDnuRfN6p4V21HmL4WScUihFLIfc1XvrcOApqJDsIEcQbSjJYlV5Mb4UDZW402xMsdyPyc71HxrgphuiCgYhGMFvJgBVHuV4fAtmPJbKIHuq9phCxZxeT2tQoq3tWwIEyEDowYMYK94whWh\/d8wIABnH3d6KZiEYwQcqNxkP7UIaCZ6Ccvp9DeEU\/7NZBgZMlqJreviat4Vy1PmIjHRChQvXr12Fs+cuRIOnDgAOfH9HXSZ9++fXxKCLGaOIeOUz04MSRHI32JonxvBQSavL\/H733NQAhT25cEYcLkLjQrNRrF6k0I080KaaUqXn75ZZo6dSqNHj2a4zF9lahA0Ds84XDqlCpVirZu3UobN27kEz3z58+Xo5FWfxtkfIyAv\/uaegjTk8mNvclwakKYblYLGuZ7773HGua8efOYMM+dO0cLFizg9G+e0rudPHmSPv30U+rduzcT7OnTp7mIWpUqVbj6pByNDKdXw95j9Wdf0xthhpvJ7WvVhTA9IIQ9TGiLP\/74IweqI9AcJFqyZElfmPL3ODW0ePFiLmsRHR1N1apV83g0UsuHuXnzZl19G3GRJY6zGTER6SNkCOBo5ZhNF+ns1Ts0pl4ueqpAtNtnuZOlP7+cQbTrv0Q4spi9IKUZ9nXIxmlWx3I00gfSiJmEmX337l23mdZhunfr1o2wb4l65drZcGia48ePZwcRSvbOmTNHjkaaJdXyHMMRiF16iI9WDmlQ1O2RSmiYj2ROwxnLXdOnhZvJ7Qs80TCdEHLOuO4OOCQYGDdunMe65FqAOjRLkCfSt6GchRyN9CWG8r3VEQBhgjhdQ4+QNu3Czs9ZmzQifZrVcRDCdFohraYPNEqEEcEUR9IMNHjIYVo3btzYcULHdXGxzwlChSmvZWrHNXI00uqvgYxPDwIIPao4\/luqGpNMk\/PtpCybpjBJ3onJS8Umf6uni7C\/RgjTzRJC04QZ3adPn\/syrmMP89VXX\/WoYX7\/\/ff8faFChRzZ1KGVInEw0rnJ0ciwf19sOwHt9M3Ng\/GcsfxMVF5OePF4j8mcoMYu2fuFMN28AteuXWNNEQXPNCcPNrYRVoRg9piYGENfHBWLYCchN3SxbNYZTG7XfcmM5WrSjFNFOOsRjlS+Vi1aCNNEubBciQrMHdoiwonSp0\/PUKCmD7IMVa1a1XBohDANh1Q6DBABV01S25d0l2NSCz1CcuJ1\/Spzrs1IbyreVVdMLUmYGCRCg3BiB3ubyLCeIUOGkMiDikUQDTMkSxl2nWp1b24e2Eqaue2v8+a5ad\/ShZvGpIqzOoAq3lXLE6Ynb7kvL7nzxA4fPkyTJ0+madOm8Z6nHI20+qtgr\/E5m9qYuVZFMZAwIPzxXXEkDZvonkKPIgVdIUw3K6l5yxEmhPb7779z1cioqChq3749pUuXzuv6a4SLzO2zZs2i3377TapGRsobE2bzgIPm9oVESjmw9a\/\/43kG3kxtf6eoWSv+nA7y9xlWuV4IU+dKaCd3UFbCm9MH18EbXrhwYdq0aRPvg3755ZdSNVInznKZ\/whAW7xz\/gTBvAY5ooEotabV3UaJ2VDUu3Hd3kECD2Q9MiKbu\/9ohPYOIUyd+OJc+MKFCznUyBthoiIkzPGmTZvyOXIQJmqSe6sa+X9NUx1LefMEnvVa5zQcl929e4fSpbu\/+qC\/fcj1QSCAI4NGNa0qYo6CRNkLcK9pWr1pVO8++3F3NHLOzmT68Ltk6lElW0gqVfocVAgukKORHkB1PvKoXYIiaK6lJVyPRnbq1Ik+\/vhj6tevH92+fZvjLzXClKqRIZBg6dISCHhyIHo6HWSJQQc4CNEw3QCHPcwbN25w0LpWdwf7mQgtQtYhHHd013bs2EHt2rW776v8+fNT69at+Vy5c7YiqRoZoMTKbZZDwFfERSSZ6EKYLuKHDOs44ogqjwhc1+qSX7lyhU1rJNPQU2YX2qemYcLpI1UjLfeey4AMQsAXYeIxWgKPpLdrG\/RUNd0IYbrgjoB1OGkSEhI4SF0LXMdl5cqV4xRt8Jb7as6EqYUVydFIX6jJ9+GIgB7CxLwiwUQXwnQjoUggnJiYSMWLF9dFjsEKuYpF0Cvkwc5N7o98BPyRJefaQeHoRVfxrrpKkGVO+iB+EjV84OFGXR54\/5ybP4Hr\/rwmKhbBHyH3Zy5yrf0QCESWtNpB4Waiq3hXLUuYqkRdxSIEIuSq8JHnWhuBQGUpHE10Fe9qWBAm9iDXr19P58+fd4w3a9as1KpVK481fXAhTvcg2zpyYObLl4\/DiqRqpLVfeBldcAgESph4Kkx01ERHCwcTXQjTjawgvRuSACNjunMNH4QCVahQgev7uGs45YMqk5UqVaLnnnuOvv76a0KoUaNGjbiYGrzmCFNC6d0OHTpw\/2gqFiEYIQ\/u9ZK7Iw0BI2RJM9GtfhZdxbtqeQ1TK7OLAHQtrEiPkJ89e5aTbaCyJE4DgUARzwlNFXGcUjVSD4pyTbghYARhYs7hYKILYbqRThAd4iZR6bFs2bK65ffo0aNcWRKmOypAPvnkkzR06FD65JNPvB6NlKqRuiGWCy2IgJEVSFGpskfcOZ6lt0qVKmCQo5EeUIe3HEXMUMfnkUcecZwdd\/WSux6NRIwmAttnz57NRLt161ZauXIl5cmTh2rVqkU1atTgJ+K8OUpWaFUmVfzVMkorUCG48kxrIWC0LGmhR9uOJ1suXZyKd9XyJnmg4ggNEwk6sP8JU14LXse+J45ZytHIQJGV+6yMgNGEqc11yoZEzrHpWqlSJRZCmG7Qh9NnxYoVhOOQzg2nfpBEo3z58m4D2rVaQKhVXqpUKS5zsXr1aj5LvmTJEiZSNJS66NixI5UpU4Z\/VrEIoRJylcIsz1aDQChlyWpedBXvquU1TJz0QdYhpHSrX78+oewuMqajdC6cNyi3izRv7trPP\/\/M3nCEF+GkEPYwkYAD98vRSDUvtDw1tAiEkjAxciuZ6EKYbmQJGdaxD4l0bpkzZ+Yr8Ds4dLQUbvCEG9VULEKohdwobKQf6yNglixZwURX8a5aXsMEOU6fPp3zWiJxBhocQW+99Rb17NmT5s+fz6FDRjUVi2CWkBuFkfRjXQTMlCWtDAYqVKoIdFfxrlqeMDHAVatW0dKlSzm0CPkv9+7dS40bN+Z6PmfOnPFokgci1ioWwUwhDwQTuSd8EDBbllSa6Cre1bAgTAwS+5DwfCOhMPYvUWoXn+Hx9pTiDY4eJO7APuijjz5KI0aM4PukamT4EICM1D8EzCZMbXQqTHQhTA+yASEAySGIHQ0EeOzYMXrzzTcdZrrrrTDl4fAZOHAgO3rgIT9+\/Dg1bNhQqkb69w7K1WGEgCrCBERmm+hCmG4EE1nXEfpTpUoV2rVrF5\/Y+fXXX6lixYr0zDPPeBRlEObrr79Offv25TPoWmgSTv5Y7WikFRY+jDhBhuoFAdWyZKaJrnquWAbL5MPUZML5LDmyrwMkmOQoUdG7d2+v2YpQWheECY0UyTXgWdd7NFLeSkFAEAgcgZTSTSmldBPK9lm3wDvRcSdCB1U2yxEmAtBhenfv3p0zryOBBpw\/SKzRv39\/R00f16OR7du3p3HjxrEHHQSLo5U4HomA95o1a3o8GqkSfHm2IBBJCMBEr1EiWyRN6YG5WI4wMUI4b7Zs2UIgQZjZ2IvE0cZevXp5dPggldu6deuYMOEUgtMIBApT3tvRyIheXZmcICAIGIqAJQnTeYZamrYsWbI4yu66QwDa6OTJkznfZd68eVm7xL8XX3yRFi9e7PFopKFoSmeCgCAQ0QhYhjARnA7t0LWWj4a+r5o+cOwgafA777zDe5jY+xw2bJjPo5ERvboyOUFAEDAUAUsRJvYob9++zaUo4CV3TiCMbOnweCOQXZogIAgIAioQsAxhYvIITEfSDcRgxsXFcUmKFi1acGhRhgwZVOAjzxQEBAFBwIGApQjTeV1gYuMY5Jo1a2jt2rXsKUdQupaQQ9ZQEBAEBAGzEbAsYTprm0KYZouFPE8QEATcIWApwhSTXIRUEBAErIyAcsJEoPqtW7coOTmZk2Ug0\/orr7wiTh8rS42MTRCwKQJKCBOaJAqR4ejixYsXHWVxT548yY4fFC7DXiU841rzFVZk0\/WTaQsCgoCJCJhOmEiugRIUIMB69eo94MTB9xs2bKA9e\/Zw9iFpgoAgIAhYBQHTCRMaJP55ymmpAYNrJObSKmIi4xAEBAEgYDpharB7OtmDImcofvbSSy85apLLUgkCgoB5CFyNX+D2YXfOn\/A6iDsXEkM+yNyx80P+DG8PUEaYqAaJM94PPfQQ1alTh3NWfvbZZzxWmOsJCQk0cuTIkIODI5TSBIFIQKBBzms8jTwP3aE8GW5T3gypCbjxM5r2cyBzPXcryudtv\/3h+xqfnfi4oPkXN4PtIqj7lREmNMw5c+ZwfR5kE0K7fv06O4JefvllWrhwIWcqCnVTkZRUxTNDjaP0H3oEoMHdPBBPKQe28sNuX0iklAPx9z04KncR\/jnq4SKU\/q\/P\/HPuIpS91ejQDzKET7DCe6OMMJ3zXkKjRMOxyBkzZnCiYBDm6NGhX2AVi6CyrEAI5Vm6NhgBmMYgR2di1AgxY7la\/LRrD5ejoi1fM\/jJ1uxOxbvqioQywsRAkPcSpIgkv3DwXL16ldOznTt3jk30li1b+rVyy5cv5wxFWps0aRIn8pAiaH7BKBcrQgAa5NWvFtLNg9AiUzVHECTIMbpcTcpSq\/MDI7PTH1\/bEyZI8cSJE4RFL1y4MO9nIls6cmD68qK7k+kPPviA6tatyxUjtXbo0CEpgqaIAOSxvhHQSBLaJD77IkjXHoUwfWNs5BVKNUzUH8c5cTiABg8ezLksO3fu7Cgn4c9EcVpo5syZ1K5dO8qXL5\/jVmidViuCZich92cN7XKtO5KE9hjIHqOdZMnWGiaqPE6fPp06dOjA+5YwzUF6cPqAPGNiYvx6f+AwQqA7EhCjnjlCk1AQDXuhlStXZk882rZt2\/iU0ZAhQ\/hnFYtgJyH3axEj\/GJokVfjF7K5DU2y0Kxfgp6xnWRJxbtqmT1MzUuOuj0oegbCRM5LfHYudqZXouBEQpVJmOQoZ4HTQt988w1lypTJaxE0Laxo8+bNeh8V9HUgdc3RFXRn0oG1Ebh8iv7c9V+iL2cQZS9IVLwqpWn1pmFjtoss4b1Gs23VSJjJ8+bNIxyF3L9\/Pxc4A2khcL1nz54B7WE6S6HeImgq\/mrZSSswjBnCqCMt\/MdZmwzU5PY1bTvJkop31TIaJgaC2jsbN24k7GWmpKRQo0aN+ISPc2kKXwKjfQ\/nDpw+qBQJcx5ldqFh+iqCpmIR7CTketcvEq7T9iYvfzrG4bzJUqsTRf8VAhSKOdpJllS8q8oJE2fEkcINGqa7FmjtHvSHMrvYF4VpD0850sXlypWLw4pAprgmNjaWGjZs6MiEpGIR7CTkoSAJK\/XpGgpk1N6k3jnaSZZUvKvKCdP5DDkcP0jvVqRI6umES5cucQ0f5MM0qxSFikWwk5DrffHD7brLK8Y64iVBkqEyuX3hYidZUvGuKidMbQAwx2fPns3B6Yi9RMMG9sqVK3k\/EzGZRjRolRK4bgSS9u7DmSCBhBYvqToZhBCmuXKpLA4zKSmJzWR4xJ3PkiPECE6fHDlyGIKEBK4bAqNtOtGOI2LCrkcStRM3OKMdyn1Jf8AWwvQHreCvVUaYiLmcOHEiFS1alPcU0bZs2UI\/\/fQTDR061DANUwLXgxeSSOqBT9ScP8GnakCIaO4SWGhntVVrkL6wF8L0hZCx3ysjTEwDwebLli1jkxkmOoLNu3bt6nfQujdIpkyZIoHrxsqMIb3BxPXUfOVV1IjO+f4751PJz9e97rL5eDqnbchEQ9yJEGaIAXbp3nTCBDEiyQa8194anEG+rtEDFQizevXqjuOWrid9fm75d90gPf3JNYEhcCYqr183nk3n+fozUXke6OvsX\/0nlOnB3+WPSc3NmD9L6v\/5YqLohTKZ\/RpDOFwsgevmrpLphInEGmvWrKG9e\/dyJqHixYs74i4RxH7w4EFavXo1VaxYkWMog23QYNHatGnD\/+NnOJSaN2\/OP6vwvNlJKwh2\/fTcP2XD35m+TyalJpj9NSmF\/z95OYVO\/vXZta9COaL5V9WLZ6NCOTLy5yENUiM2wqXZSZZUvKuucmA6YWoDwEkc7C\/GxcVxtiK00qVLU7NmzTi0yEinz6JFi2j48OH8jAkTJlDHjh2pTJkyQpjhwgohGOfS7885iHT7scsPECvItG3lfJYnUCHMEAiHly6VEaZZ09TCiiRw3SzEw\/s50Ea3H09mMgWRbjuezBOyKoEKYZorbxFPmL7gVKHm20nIfeEfDt9vO5bMJDplQ6ol1LZyXnq\/baqForrZSZZUvKuWMclDIWiScT0UqEqfzghA8wRxwqSvUTwbrYl9QilAQpjmwh9RGqZkXDdXeOz8NBBnn6WH2GQf0qCosr1OIUxzpVApYeJcOVK8JSYm8vlxnMpp2rRpQNmKJOO6uYIjT0tFAJpm7NJDvMe5d8TTpsMihGku5MoIE+FF06ZN47CirVu3ckldJABGQo5A8mFKxnVzBUee9jcC0DabzNrDvzCbNIUwzZVEZYSpnSVHiI+WcR3xkXrPkuP+bt260b59+6hHjx6ctk0yrpsrPPK0vxE48\/sdGrPpIp29eofWdkotG21Gk8B1M1D++xnKCBMlJWbNmsUxl1pNHwSu4zPyWPpb08cVNsm4bq4gydOIQ5Gwr4lmljNINExzJU8ZYWKaOKYIszw5OZmeeuoprlOOwPIaNWr4jYJkXPcbMrkhBAiANCuO\/9Y0R5AQZggW0UuXSgkT44KmiSqPOGNeokSJgM+PS8Z1cwVHnuYdgRwDv6I1vZ+gGiWyhRQqIcyQwvtA50oJExol9h1RahdHF1ERbuDAgYacIdcLo4pgWDsJud51iLTrcL596fdnQ+4EspMsqXhXXeVSGWFCs5w6dSq9+uqrnGn98OHD1KRJE3r33Xfp3\/\/+t889THjZUcMchdNQvwcNhAviRamLZ555hpMTo+SuZFyPNDoKj\/k0eX8PPZIjOqSngoQwzZUFZYQJL\/dHH31E\/fr1o08++YQqVapEJUuW1OUl37FjBy1dupTL886ZM4cJEwT+ZRtQAAAMsUlEQVQ8ZswY6ty5M5UrV45WrFjBxdZAnHjO2LFjufAZroFG+\/jjjzPSKv5q2UnIzRVn6z0t1Ka5nWRJxbtqGQ0TgebIVXnjxg06d+4cTZo0iZ0+u3fv9plxHdro6dOnmRQHDRrEhIl90Pnz57OGiZIXR44c4aB4kGf69OnvS+8GELR0byoWwU5Cbj0KM3dECGrHOfRQxWfaSZZUvKuWIUwMBMHmP\/zwA+XJk4dy585NX331FdWsWdOnOY57EYIEj3qnTp2YMKF1rl+\/nkaOHElRUVGkhRXlzJmTnn32WapTpw7P3TWBsIpFsJOQm0tP1nwavOahShVnJ1lS8a5aijCxD4ns61qNcpAgEvyiTEX27NkdY4Um+uGHH7IZ\/fHHH3OuTHeEiZpAw4YNY9PbmTDr1avnMeM6FgFt8+bNpr1tdgk2Ng3QMHjQUzMTac6LeempAqlJi41qdpGlunXrMmTwU6hsyvYwQZZIlvG\/\/\/2Pj0Nmy5aNzfMuXbpQu3btWEv01lwJUzPBoWFqJvnixYtZ+0RfknFdpZjJs+EAQvZ3o01z0TDNlS1lhInEGzNnzuQwImiGyICOvUaEGYE0\/SVMOH3GjRvHxyXhPML+JmI7K1euTJJx3Vyhkqe5RyAUprkQprnSZgnCRPIN7DXC5EZYUffu3X2WqHDVMDV1HV5wOJFq1arFHnhomwgrkozr5gqWPM09AkZ7zYUwzZU0ZYSJfUvsS8Lxgz3GuXPnUtmyZbkuOZw5mTObU+FPxUaynYTcXHG2\/tMQ0I4ExEadArKTLKl4V10lShlhYiD37t3jfctMmTJRQkIC58N8\/vnn2WNuVlOxCHYScrPWMdyeA\/P8vTZlgj46aSdZUvGuWoow4fiBRonkG1pDircKFSpwKVxvzd1JHylREW60Yd\/xwgmEbO1Jb9cOCgQhzKDg8\/tmZRomHDJw0sBDroX2YPRZs2bleuXeTHJ3J31wr5So8Hv95QaFCGikGUyJCyFMcxdQGWFqCYRx3huOGX+au5M+UqLCHwTlWqsgoO1posQF9jXxvz9NCNMftIK\/VhlhQsOcPXs2tWzZkgoUKOD3TFy95MGUqMDDJXDd7yWQGwxCQMvWnnA6hXpUyUaNy2Sm\/DHe45C1R0vgukGLoLMb0wkT8ZJaYoyLFy9SfHw81a5dm8OKPJnkek76oN9AS1SYfXrATlqBTjmUy3Bs91gy9Vl2iDO3Q9PUc5zSTrJkS6cPPOPIIqQdh3R9U3CsEfuYadOm9foSuYvDdL5BSlQIB4UzApqpjjlo5Jn6f977piWEae4qm65hYnoILEc9HyTeQDkK17PjeiBwJUwpUaEHNbkm3BCAtolSvtuPXWavukaghbJHU\/US2Sn58mVqVKlo0OFJ4YCLLTVM7F2+8cYbnP+ySpUqfAoHpvngwYN9Hod0XlRXwgynEhVWWPhweEFkjA8iALMd6eLQnEnU+UpooiBUNCQwLpQjo+PrIQ2KhC2sVnhvTNcwXb3j+BmZ11GXPNhKkYFIgnNIUyD3yz2CgJUQSCnd1DGce5ly0r1MuQj\/o+FzMC3tjYteb09741Iw3eu69\/yinrquC9VFSghTy7SeMWNGAmEiG\/ro0aN9nh8PFQjSryAgCPyNALYA3DVsD3hrJ5NuhhzG99uWCfkzvD1ACFMp\/PJwQUAQCCcElBAmUrDt27fPLU7OSYLDCchgxgqPPvZ1z549ywXg\/vnPfwbTndxrcwTgOX\/77bc50qRUqVKc8tDXUWObQ6Z7+qYTpu6R2ejCuLg4KlSoEAs36hIhH6hZ2ZpsBLNtpvrjjz\/S7du36YknnrDNnM2aqBCmWUh7eQ7KbjRs2JDy5cvHJTi0zxYYmgwhDBHYuXMna5dIni3NWASEMI3F0+\/e7t69y9UtX3rpJXZ6odZ6gwYNHLXW\/e5QbrA9AmvXrqU1a9bw4ZDmzZvzH2AcCJEWPAJCmMFj6LEHBNNv2rSJ+vbty9cgJd3ChQtpyZIlnAMU9YegBcAMf+GFFzgPKMhT+xzCoUnXYYiAXnk6c+YMZcmShUu+oIJBx44d2XqRFjwCQpjBY\/hADzj6ifPyq1at4lIZQ4YM4WtQigP\/EHOKYH1klsd3qMeOPUwUbEPhts6dOxNCrqQJAkDAX3lKTEzkci+QIS3BTd689x+pFGQDQ0AIMzDcvN6FHJ8HDx6kAwcOMDFqhIkkItWrV+fjoDCXJk2axDXTUbQNXnIcGRUveQgWJMy79FeekC4Rh0FSUlKoQ4cObJaLSW6MEAhhGoOj216Q6BgaJQgTRzmRMLl9+\/b02GOP8fWowQ5Bbt26dQhHIV1HCgIiT+pXUggzhGvgKuDjx4+nTp06sUYphBlC4CO0a5En9QsrhBnCNXAWcGcTXDPJYaLXrVtXwj9CuAaR1LXIk\/rVFMIM4Ro4C7jm9Nm1axehLAf2NkGYMNdlQz6EixBBXYs8qV9MIcwQroGrgGthRahuiY15eMsluDiECxBhXYs8qV9QIUz1ayAjEAQEgTBBQAgzTBZKhikICALqERDCVL8GMgJBQBAIEwSEMMNkoWSYgoAgoB4BIUz1ayAjEAQEgTBBQAgzTBZKhikICALqERDCVL8GMgJBQBAIEwSEMMNkoWSYgoAgoB4BIUz1ayAjEAQEgTBBQAgzTBZKhikICALqERDCVL8GETECnIvfuHEjxcTEOOaDUhs9e\/YM6\/mhoNixY8e4bMjQoUOpbdu2VK1aNcecMO8iRYq4TdF3+vRp2rx5M7Vr146ioqLCGgcZfCoCQpgiCYYgAOKoWbPmfWTi3DGyNeEfinOFS0MO01mzZnGJB1Tx9JcwMU\/kPEU1UKngGC6r7n2cQpiRsY7KZ+GJMJFoBNnnUTZhwIABlDVrVi7Ncf78edbMQEIFChTgMh2jR4\/m+tlISIKaNEi2PHbsWP49CsRpyScGDx5M69ato7lz53KdJBT5QlkP1LpHiQ\/0sX\/\/fqpUqRInOImOjqZFixYxef3xxx\/Uu3dvrp+E\/tAXtD98jwZy1FpCQgLt3r2bunfvzgmgvRHm5cuXacOGDXwrMlFVqVKFJk6cSCdPnqRPP\/2UXnvtNcqQIYPydZIBBIeAEGZw+MndfyEAwvzwww8deOTPn5+Lu4Fwtm3bxuU4UCt7+vTpTFhIaQdzF3WPunbtStOmTeNUd3ny5GHyQrkOEJU7wqxfvz5nsu\/Tpw+lS5eOr8+VKxflzJmT+5k5cyb\/jITNjRo1ohs3bjgy39+6dYvLgcC0xvhAqNAekQ0fz0NdJa2hgmfZsmWpTp06DsJERUbXhrlpWfPxh2D48OGcwg+Z9VFeAn0PGjRICpFFwNsihBkBi2iFKXjTMDE+EAoIskuXLnTp0iXHkFGsKzY2lpAnFISJkh1Hjhxhbc2ThgntEGTo3Hr06MFbAlpJEHwH7bZw4cK0fft2NolBfGja9gDM7fLly9PDDz\/M2ifIE9qp1pzn5EvDxPygvYIcK1asSC1atOC54D5o1Mi070zGVlgzGYP\/CAhh+o+Z3OEGAT2EuWfPHjaZoeGhouG9e\/dY+wORbtmyhYYNG8Ykg59RnhjmsTsNE7lEQZqaVgetEfXdYZK7I0z8Tis+h6FD60MJWmwV4DnoC5ottFHn5g9htmrVilauXElw9EDz1Zw8QpiR9boIYUbWeiqbjR7CvHDhApurME9R1yg+Pp7JsXHjxvTmm2\/SqFGj7jPJe\/Xq5TBnYcKjZjv2B0F+cXFxTKYgPpjOVatWZZJyR5jJyclcwdM10z1M8TFjxtC1a9f4f9fM93ge9lmdTXJPXnLsw8LEnzx5MhOw1tA3THaQqNQGVyaehj1YCNMwKO3dkR7CBELQAmFOX79+nU3UESNG8H4jwm9ANnD2aE4fkMxnn33GtbXh9ClWrBjvU4L4UPd9wYIFDHqbNm1YG4VZ744wYfbD\/IY2CQ124MCBXEsJnzUS\/s9\/\/vNAKVo4fb777jsOjULJWk9OH+zX4tnYr8Vc0AoWLMhkj5riS5Ys4T8Szua+vaUlfGcvhBm+axexIz969CjvYYIwQ9ngYYeTCE4kd2E\/MKfhpIJTKtC6S9gbhZbqHLsZyjlJ36FFQAgztPhK7wEgYAZhJiUlsdYHTzaI2ZP2d\/jwYXZCYdsAGqk\/DfuZIH5ovxK47g9y1r1WCNO6ayMjEwQEAYshIIRpsQWR4QgCgoB1ERDCtO7ayMgEAUHAYggIYVpsQWQ4goAgYF0EhDCtuzYyMkFAELAYAkKYFlsQGY4gIAhYFwEhTOuujYxMEBAELIaAEKbFFkSGIwgIAtZFQAjTumsjIxMEBAGLISCEabEFkeEIAoKAdRH4f9+nmk2HsLS1AAAAAElFTkSuQmCC","height":200,"width":332}}
%---
%[output:165672fd]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:3a052c4f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:8e63741a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:0c55784a]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:2b7fc8fb]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:83f70987]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:081b02ac]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:24a492ca]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:91c2d5cb]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:3bc87194]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: GM355L6-250kW\nIM Normalization Voltage Factor: 326.6 V | IM Normalization Current Factor: 610.9 A\nRotor Resistance: 0.00434 Ohm\nMagnetization Inductance: 0.00533 H\n---------------------------\n","truncated":false}}
%---
%[output:54e494a4]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00756 H\nPer-System Quadrature Axis Inductance: 0.00624 H\n---------------------------\n","truncated":false}}
%---
%[output:37bb0420]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:95289131]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:8400c6b8]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:17818007]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714299e+02"}}
%---
%[output:29702271]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -3.927176478925923e+03"}}
%---
%[output:6b1f64f5]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Unrecognized function or variable 'kalman_psm'."}}
%---
