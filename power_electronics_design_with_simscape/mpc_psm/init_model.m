clear;
[model, options] = init_environment('mpc_psm');
%[text] ### Global timing
% simulation length
simlength = 1.25;

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

fPWM_INV = 6*fPWM;
tPWM_INV = 1/fPWM_INV;
TRGO_INV_double_update = 0;
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
dead_time_AFE = 1e-6;
dead_time_INV = 1e-6;
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
tc = min(ts_afe)/400;

% t_misura = simlength - 0.025;
t_misura = 0.648228176318064;
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
use_torque_curve = 0; % for wind application
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
us1 = 690; us2 = 690; fgrid = 50;
eta = 95; ucc = 5.6;
i1m = 5; p_iron = 1200;
up_xi_pu_ref = 1; up_eta_pu_ref = 0; un_xi_pu_ref = 0; un_eta_pu_ref = 0;
grid = grid_three_phase_emulator('wind_grid', grid_nominal_power, application_voltage, us1, us2, fgrid, ...
                eta, ucc, i1m, p_iron, up_xi_pu_ref, up_eta_pu_ref, un_xi_pu_ref, un_eta_pu_ref);
%%
%[text] ## Global Hardware Settings
afe_pwr_nom = 250e3;
inv_pwr_nom = 250e3;
dab_pwr_nom = 250e3;
cllc_pwr_nom = 250e3;
fres_dab = fPWM_DAB/5;
fres_cllc = fPWM_CLLC*1.25;

hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, fPWM_AFE); %[output:2f676552]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, fPWM_INV); %[output:385a30cc]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, fPWM_DAB, fres_dab); %[output:155469a7]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, fPWM_CLLC, fres_cllc); %[output:2f0fae9c]
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
l1 = Kd(2) %[output:46e2a67e]
l2 = Kd(1) %[output:689b3ee0]
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
parasitic_dclink_data; %[output:2ce4b10c]
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
res = s/(s^2 + 2*delta*grid.omega_grid_nom*s + (grid.omega_grid_nom)^2);

Ares = [0 1; -grid.omega_grid_nom^2 -2*delta*grid.omega_grid_nom];
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:8b7a3c75]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1;
pll_p_ddsrt = pll_p;
omega_f = grid.omega_grid_nom;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
tau_ddsrf = 1/omega_f;
%%
%[text] ### PLL FHT
pll_i1_fht = pll_i1;
pll_p_fht = pll_p;
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega_fht0 = grid.omega_grid_nom;
delta_fht0 = 0.05;
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:316ce442]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:0fe8f485]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:2b7bb8c2]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:846b087d]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid.omega_grid_nom;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:3a877cb5]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:6277dd77]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:73a0a02c]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:4a1ff22d]
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;
kp_rc_pos_grid = kp_rc_grid;
ki_rc_pos_grid = ki_rc_grid;
kp_rc_neg_grid = kp_rc_grid;
ki_rc_neg_grid = ki_rc_grid;
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = 2*pi*rms_perios/grid.omega_grid_nom/ts_afe;
rms_perios = 10;
n10 = 2*pi*rms_perios/grid.omega_grid_nom/ts_afe;
%%
%[text] ### Online time domain sequence calculator
w_grid = grid.omega_grid_nom;
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
time_start_motor_control = 0.075;
%[text] ### Machine settings
psm_sys = 6;
psm_pwr = 1600e3;
psm_i = 2062;
psm_rpm = 17.8;
psm_np = 104;
psm_eta = 96;
psm_Lq = 1.26e-3;
psm_Ld = 1.04e-3;
psm_Jm = 900e3/psm_sys;

psm = pmsm_setup('WindGen', psm_sys, psm_pwr, psm_i, psm_rpm, psm_np, psm_eta, psm_Lq, psm_Ld, psm_Jm, 0, 0);
displayInfo(psm); %[output:5b1728ee]

n_sys = psm.number_of_systems;
run('n_sys_generic_1M5W_torque_curve');
torque_overload_factor = 1;

% load
b = psm.load_friction_m;
% external_load_inertia = (psm_sys-1)*psm.Jm_m;
external_load_inertia = 1;
%[text] ### Simulation parameters: speed reference, load torque in motor mode
% rpm_sim = 3000;
rpm_sim = 17.8;
% rpm_sim = 15.2;
omega_m_sim = psm.omega_m_bez;
omega_sim = omega_m_sim*psm.number_poles/2;
tau_load_sim = psm.tau_bez/5; %N*m
b_square = 0;

%[text] ### Double Integrator Observer Inverter Side
Aso = [1 ts_inv; 0 1];
Cso = [1 0];
% p2place = exp([-10 -50]*ts_inv);
p2place = exp([-50 -250]*ts_inv);
Kobs = (acker(Aso',Cso',p2place))';
kg = Kobs(1) %[output:0445a065]
kw = Kobs(2) %[output:436a1909]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/psm.Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/psm.Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:098ea60f]
luenberger_l2 = Klo(2) %[output:0a3c7cf9]
luenberger_l3 = Klo(3) %[output:3d975c78]
omega_flt_fcut = 10;
phase_compensation_omega = 0; % for generator mode
%[text] ### Control settings
id_lim = 0.35;
%[text] ### Rotor speed control
kp_w = 2.5;
ki_w = 18;
iq_lim = 1.4;
%[text] ### Current control
kp_i = 0.25;
ki_i = 18;
kp_id = kp_i;
ki_id = ki_i;
kp_iq = kp_i;
ki_iq = ki_i;
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
kalman_psm;
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
nominal_battery_voltage = grid.udc_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery_1 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab); %[output:4c275a97]
lithium_ion_battery_2 = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_dab); %[output:86468a60]
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
if use_mosfet_thermal_model
    set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
    set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
else
    if use_thermal_model
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'off');
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    else
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
        set_param('mpc_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    end
end

if use_torque_curve
    set_param('mpc_psm/fixed_speed_setting', 'Commented', 'off');
    set_param('mpc_psm/motor_load_setting', 'Commented', 'on');
else
    set_param('mpc_psm/fixed_speed_setting', 'Commented', 'on');
    set_param('mpc_psm/motor_load_setting', 'Commented', 'off');
end


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:2f676552]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:385a30cc]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:155469a7]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:2f0fae9c]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.880000e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:46e2a67e]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:689b3ee0]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:2ce4b10c]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAiAAAAFICAYAAABpxkW2AAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQ1wVtd55suW1shjXJBNQoRAUmzROJ6Ogh03ikwqXCWl3URsi9OIj2ZKNDJ1CQyZRgwSMrOYqa0fJvLEBJuVbVXVTiqZJGanaKdb4uDADFG1TWLMdqlbsIXACskGIxjjrUjDVjvvxefz1cf3c3\/Ovfec93vuDIP0fefnPc9z3nsevedvzszMzAzhAQJAAAgAASAABIBAjAjMgQCJEW1UBQSAABAAAkAACDgIQICgIwABIAAEgAAQAAKxIwABEjvkqBAIAAEgAASAABCAAEEfAAJAAAgAASAABGJHAAIkdshRIRAAAkAACAABIAABgj4ABIAAEAACQAAIxI4ABEjskKNCIAAEgAAQAAJAAAIEfQAIAAEgAASAABCIHQEIkNghR4VAAAgAASAABIAABAj6ABAAAkAACAABIBA7AhAgsUOOCoEAEAACQAAIAAEIEPQBIAAEgAAQAAJAIHYEIEBihxwVAoH4EDhz5gw1NzfThQsX0pU2NjZSd3c3lZSUFDRkenqa2tvbKZVKUW1tbcH0Y2NjtH79+lnpHn30UWprayO\/ZRWsDAmAABCwGgEIEKvpg\/FAID8CLEC2b99Oe\/bsoerqat8iwK9oYAHS09ND\/f39VFpamq6vrKzMESF4gAAQAAIKAQgQ9AUgIBiBQgLEHbFQkQqGg0VEX18f1dfXO+jwdxwBOXDgAO3YsSP9WaaoyBQgnJBt6OzspCeeeMIRQhxNqampcSIrIyMjTlkqKsM\/q8\/5s3feeYc6Ojro1VdfpdHRUTp\/\/jytW7eOKioqZkVahoaGaPny5dTa2kq\/\/du\/TX\/xF39BLHq+9rWvOW05efIkdXV1UVNTk2C20TQgYBcCECB28QVrgYAvBLJNwaiB2C1OysvLnYG\/rq7OGdxVFOPSpUvOFA4P5PwMDw870zdKKGROzWQTIFNTU44w+OpXv0ovvPCCI0CWLl3qlLFkyRJS37uFBtfBomHbtm00MDDgCJAXX3wxHVnhetSUEIuiiYkJ2rhxI7W0tDifszDiNnA6jsa8\/PLLjoDxOvXkC2QkBgJAIBACECCBYEMmIGAHArkiIEpoKEHB60HUQK5alrlu49y5c+noh0rjjprwZ14FCIsENb3DURCOVuzfv98RKGwbRypyCRO1diUzeqMECNutojUsTPh3Tutuqx3swUogIBsBCBDZ\/KJ1RY5ApgBhOJTQ4OkVvwJEDei5YPU6BcP5ebGqe+pERUgKCRAVfeH\/OaJx6NChWREQCJAi7\/RovjUIQIBYQxUMBQL+EcgXAbnvvvvSC1S9TsGoKRF3eve6inyLULdu3ZreUcMtUeInc6pFTZXk+lwJEPdaEo6gIALiv38gBxBIEgEIkCTRR91AIGIECm3DjWIRqpdtuLxglNdrsMjg9FevXr1pcWq2RahqDYdaDMvCg8t57bXXHDG1ZcsWZ8oFUzARdywUDwQ0IAABogFEFAEEgIB+BLJN5+ivBSUCASCQFAIQIEkhj3qBABC4CQH3Nl\/+kteIeDkADVACASBgHwIQIPZxBouBABAAAkAACFiPAASI9RSiAUAACAABIAAE7EMAAsQ+zmAxEAACQAAIAAHrEYAAsZ5CNAAIAAEgAASAgH0IQIDYxxksBgJAAAgAASBgPQIQINZTiAYAASAABIAAELAPAQgQ+ziDxUAACAABIAAErEcAAsR6CtEAIAAEgAAQAAL2IQABYh9nsBgIAAEgAASAgPUIQIBYTyEaAASAABAAAkDAPgQgQOzjDBYDASAABIAAELAeAQgQ6ylEA4AAEAACQAAI2IcABIh9nMFiIAAEgAAQAALWIwABYj2FaAAQAAJAAAgAAfsQgACxjzNYDASAABAAAkDAegQgQKynEA0AAkAACAABIGAfAhAg9nEGi4EAEAACQAAIWI8ABIj1FKIBQAAIAAEgAATsQwACxD7OYDEQAAJAAAgAAesRgACxnkI0AAgAASAABExG4PrFCZo+dZTmr\/qSyWbGbhsESB7IP\/zhD8dOCCoEAkAACAABGQgs\/rXr9Lt3vku\/e8dVp0Ff\/MeloRo2Pj4eKr9pmSFACgiQoISzePGSN1e6bJ9nfpbvd\/d36mevNuWCxEv+fGl0tSlb24I6Vpg2eeUul72SOWI+bO13ufxKV7\/z0ufc+GX2bS9+lJm\/UB\/0ahPeDbMRyIUbRzye+MP76E8+dJnmLqqkkntX0cIv7KLln\/idWeNCIV50+lHQd2SU+SBAIhIgUZIWtOywL5mg9SKfdwTAkXeskkwJnpJE31vdSXDEwuPq9wfp8rcfd4QHT7mw8NDxJNEeHXbnKwMCBAIk6j6G8n0gIPEl46P51iQFT+ZTFSdHLDwuf2s3XT36V9qFh0I6zvbExS4ESBEJkLNnz1JVVVVcfQv1BEAAHAUALYEs4CkB0H1WGQdHLDiuHh2ka6eOOsKDox1RLTSFAPHZAWxPLo3wOBzSds6Tth8cJc2At\/rBkzeckkwVFUdqRwtHPPjnefeuotI\/2uX8H+UjbTxirBABQQQkSp9B2T4RiOql6dMMJC+AAHgyv4vo5kit7+CoBz+8sHT+qg2RCw9MwZjf1yKxUJri1O2QkYBe5IWCIzs6AHgynyddHGVbWDr\/oQ3OlEucj7TxCBEQIhobG6P169c7\/airq4uamprSfUoa4bocMk6nK7a6wJEdjIMn83kKy1GUO1qCoCdtPCp6ATI1NUWtra3U0dHh9IfOzk7q7e2l0tJS53dphId1yCBOgzz+EABH\/vBKKjV4Sgp57\/UG5SiOHS3eW\/F+SmnjUdELEI5+9PT0UH9\/P5WUlFB7ezulUimqra2FAAniIcgTGoGgL83QFaOANALDP\/wZnZ+6RuenptOfvTV1zfn5\/OX3\/n\/vd\/5sWem8dLrUAx9yfm5bHW94HvTdjIAfX1ILS+Pa0RKELwiQIKgZnIcFyPDwMHV3dztWsgCpq6tLT8Ms\/vwT9PDDDxvcAn+mvfvuu3Tbbbf5y4TUsSIAjgrDrcRAvpRKKKg0LCi8Pm5BsWzhDXGx9D2Rsay0xPm95P9dpUWLFt0QJa6yf\/DGZTr+5hXncy6nbXUVpR5Y7LVqpNOIgBcBkrmjhbfQxrmw1E9zIUD8oGVB2kIC5AN\/8l9mtaKmpsaCVuU28Re\/+AXdcsstVrdBuvHgqDDDZbfPLZiobP7sNH\/6iQUF8\/hJMDk5SeXl5TmzXHjnOv3319+lvn+4Qmxv40duI902+LG3GNPm5ejyJM386CUi\/sfPXZ+gOfc\/THTXjei3KU9DQ8MsU7xc72GK7V7sKOptuJiC8dJFkCZOBLz81RanPagrOwJeeeLoCE\/p9Bw+SyvvWkCHNq8ApDEhkI0jU3a0BIEAEZAgqBmcB4tQDSanSE3zOrAVKTzGNNsvTyxE1jx7wrF\/37p7aOXdeiMyxgBjkCFujkxdWOoHLggQP2hZkta9DXdoaCi9AJXNl0a435emJRSKMhMc2UFnEJ5YhGwZft1ZI8JrQ7BQNVqumaOlt82hnz\/TnD4qXeflcNFaf3Pp0sYjbmFRT8EU6kDSCA\/y0iyEEb7XiwA40otnVKWF4YmnZDYPv+4sTn0mdU9UJhZtuWph6cW\/6yN6cyzyO1riAlraeAQBUqDnSCM8zEszLicr9nrAkR09ICxPx9+44kzJYF2IPr7dR6Xzz\/Txh6nsc1tiOypdX0uylyRtPLJSgBw4cIB27Ngxi6HME0x1dQRphId9aerCFeXkRgAc2dE7dPDEUzIfe+LvIUJCUp65sFTd0fLTWytE3f4tbTyySoCotRrZxIYSJZlrOEL2a6wBCQsg8vtGQMfA5rtSZPCNgC6e1LoQNgA7ZPzRUGhHiy6O\/FkVXWoIkOiwzVsy71YZGRmhDRs25E03ODhYMI2fJkgjXJpD+uHSlrTgyA6mdPKkRAgfnvbazk\/aAUCCVnrd0aKTowSbm65a2nhkVQQkiQ4gjXBpDplEn4i6TnAUNcJ6ytfNE0RIYV6uHv0rch+VXmhHi26OClsYbQpp45FVAuTMmTPU3NxMFy5ccG6t5YfXgpSVldHAwABVV1drZ18a4dIcUjvhBhQIjgwgwYMJUfG05pkTzn0ziITcICHMHS1RceShe0SSRNp4ZI0AmZ6eTl8Ut3z5cmppaaF169Y5d7bw+o\/R0VHnPhe+UE7nI41waQ6pk2tTygJHpjCR344oeYIIuYH95W\/tJo568KMWls67d5XnDhIlR56N0JhQ2nhkjQDhNSC7d++mXbt2UWlpqXODbX19vXNoWOZ3GvnGIlSdYKIsTwhIe2l6arSFiaLmqVhFiHsrrRIeC7+wyznLw+8TNUd+7QmbHgIkLIIB80OABAQuI5s0h9SDilmlgCOz+MhlTRw8KRFSDEe3Z9vRwsIjzBMHR2Hs85sXAsQvYprSQ4DoAVKaQ+pBxaxSwJFZfCQpQLhu6SIkCuGhOJPmSxAgCb0bWIDwuo+TJ09mtaCmpob6+\/ud6RmdjzTCpTmkTq5NKQscmcJEfjvi5EmiCMncSsvRDt7VovOJkyOdducqS9p4xO3EXTB5eo40wqU5ZBxOH3cd4ChuxIPVFzdPUkSI2kp7\/ecTgRaW+mErbo782BYkrbTxyBoBgghIkO56cx5pDqkHFbNKAUdm8ZH0FIy7fltFiNpKy7ta+OEdLUEXlvrpHdJ8CQLED\/sRpeVttxMTE9TW1ubUwL\/zw1tydT\/SCJfmkLr5NqE8cGQCC4VtSIonm0RIlOs7CjNElBRHXmwLkkbaeGRNBESRlW3LLbbheu\/K0hzSe8vtSQmO7OAqSZ5M36KbtPBQPShJjqLoxRAgUaDqo0x1IFldXV064sFngvDpqIUOInPfoutetKouuWMzMi+6k0a4NIf00XWsSQqO7KAqaZ5YhBx\/8wod+vIKWnn3AiNAi2NhqZ+GJs2RH1u9pJU2HlkXAWGDM9eDeNkBw8e4d3Z2Um9vb\/ogMxYt27dvp507d1JHR4fDvzsN\/y6NcGkO6cVpbUsDjuxgzASeNg+\/TsM\/\/FniIsR9RwvvZFm0ecAIEk3gSCcQ0sYjKwWIDkI56jE8PExr166lr3\/9684WXj7Gvb29nVKplHPCKgSIDqRRhl8EpL00\/bbflvSm8JSUCHHf0RLHjpYg\/cIUjoLYni0PBIguJH2Ww1GPkZER2rBhQ96cg4ODBdNwATxtU1lZSRUVFY4Q4ekbfliAuKd3pBEuzSF9diMrkoMjK2gyaoFjnCIk86j0QjfSJsmmNF+SNh5ZFQFRazUy12lwI9T6jqGhoXT0IlfHd++iUZGQfALEXc6RI0eS9KfQdU9OTlJ5eXnoclBAdAiAo+iw1VmyaTw99z+vUN8\/XKHn1i6m+5fM09nUG2VdnqSZH71E9PLTRAvLiT7+MM35zFf016OxRNM4CtK0hoaGWdnGx8eDFGNsHusOInMvJlWoZooSXvPR3NzsLE5tbGxML1BVkQ+1ZZcFCH+GKRhj+2fRGSbtrzapBJrIE68H4WjIM6l7KPXAYi3QZy4sNTnikdlgEzkKQwoiIGHQSziv+wZdZQpP7bS2tmIRasLcoPr3EZD20pTKrak8KREy9dRDoaB3Lyzlm2iXPXs2VHlJZDaVo6BYQIAERS7hfO6ttsoUFRnh+2XWr1\/vfJw5hSONcGkOmXC3iqR6cBQJrNoLNZmn429coTXPnqC21VXUttr7NfY2LCz1Q6TJHPlph0orbTzidlk3BROEuKB5pBEuzSGD8mpyPnBkMjv2RKqUCOGpGJ6SyfdkW1g6\/6ENxJEPmx9pviRtPIIAKeBd0giX5pA2vxxz2Q6O7GDVBp68iBC+n+Xytx93xIaN0yz5eosNHPnp7dLGIysFiPsgshdeeIFeeeUVZ+ttdXW1Hy49pZVGuDSH9ESiZYnAkR2E2cKTEiEr71pAhzavcMA17cTSqBi3hSOv7Zc2HlknQNxHsfOFdPX19Q536iwPPkxM5yONcGkOqZNrU8oCR6Ywkd8Om3hiEfJne\/8HbV16jj73fw+TqQeH6WbeJo68tF3aeGSdAHFfPPf88887AmT58uW0e\/du2rVrl3PMus5HGuHSHFIn16aUBY5MYUKGAHGv7zh\/+Rr9r\/L\/RH+6vd369R1eeok0X5I2HlknQLJFQI4dO+bpMjovHTYzjTTCpTlkEE5NzwOOTGfohn2m85Tt\/I6rn25zdscsWzgvPR1jB9rBrDSdI7+tkjYeWSdA2OAgl9H5JVqll0a4NIcMyqvJ+cCRyey8b5uJPLm30V47dTTrwtLzU9doy\/DrxNGQ13Z+0g6wA1ppIkcBm+JkkzYeWSlAwhDoN680wqU5pF8+bUgPjmxgyawISOY22pJ7V9H8VRto3r2rsoLpFiGHvryClpVGcHS7ATRK8yVp45E1AiQz6pHZt2tqapzj1LEGJL\/XS3NIA95x2k0AR9ohjaRAE3jiKMfVo4PEp5byNlq\/x6SveeaEEwnZt+4eWnn3gkhwSrJQEzjS2X4IEJ1oBizLfZkcF8G\/86PudwlYbNZs0giX5pA6uTalLHBkChNmivls0Y5FmwcCgyZZhEjzJWnjkTUREOVd7l0wKtqR7bPA3piRURrh0hxSF88mlQOOTGIjty1J8HTxmeZZ0Q5dp5WyCDn+5hXi6RhJkZAkOIqy90obj6wTIO5dMCriwZfM8a233d3dhHNAzPyrLUqnlFa2tJemNH5Ue+LgSUU71EmlhdZ2hMGab9Hli+zCXmIXxgbdeePgSLfN+cqDAIkT7Rx1YRdMcBKkOWRwJMzNCY7M5cZtWVQ8KdEx\/U9HSe1k8bu2IyiCPYcnqOfwWTGRkKg4Copv2HwQIGERtCy\/NMKlOaRl3cmTueDIE0yJJ9LJU5KiIxNIjoJwNIQvsOOL7Gx+dHJkAg7SxiPrpmBy7YbBLhhv7iHNIb212q5U4MgOvsLwxIKDj0PnHSy\/vDiRjnTwFEuYBaW6kFMipG11FbWttvdG3DAc6cJSZzkQIDrR1FQW74KpqKig2tpaTSW+X4w0wqU5pHbCDSgQHBlAggcT\/PDEgoOf6fe2zfLUCj9q62zJvfU5z+zwYEokSbzcpBtJxRoL9cORxmojK0raeGRdBCQbs9gF472\/S3NI7y23JyU4soOrfDypCMf0qWOk1nIowcFRjnn31jtndpj+ZLtJ13Sb3fZJ8yUIEAN739jYGPFOGK8HkZ05c4a2b99Oe\/bsoerqauL869evd1rW1dU16zwRaYRLc0gDu2Nok8BRaAhjKSCTJ94iy9MpPLWiIh4c4TBlWiUoKDaLEGm+JG08si4CkmsNSKZwyOVsahvvj3\/8YxoYGKA77riDWltbqaOjw8nS2dlJvb296RNVpREuzSGDvlRNzgeOzGFHRTJYWPBz7dQx53+1bkNZykKDH5uiG35Q5qPbP\/bE39PKuxZYdYmdNF+SNh5ZJ0D8OE22tLxe5OLFi8QChEXHpUuX0tETPkOkvb2dUqlUej2JNMKlOWTY\/mBifnCkn5XL39qdLlRFJ9QHSlxw5IKfzO+ziQz+7N3\/+J+pqqpKv7GGlmjjJXbSfEnaeGSdAAlzEipPvQwODtKmTZto586daQEyPDzsHGLGDwuQurq69DTM+B\/NMfR1ALOAABDwhMDC8puTlbo+W7jkxvfvpZvzma94KnZycpLKy7OU7Sm3nYkuvHOdHv\/e2\/TTq9ep7w8XU9ntc41uiASOGhoaZmE8Pj5uNOZ+jZszMzMz4zdT3OnV1MnIyEjWqhsbG\/OehMr5n3zySdqwYcOsaReOgOQTINIUp7S\/COLuh3HUB47iQDl8HcXMky33x0jjSNp4JCICku1VwtGO5uZm54h2FiePPPKIE\/ng39VTVlZGf\/7nf07f\/OY3nQWsmIIJ\/1JGCeERkPbSDI+ImSUUO082iBBpHEGAJPQuUFMvW7dupW3bttHJkydnWeLnIDIuSy08xSLUhAhFtTkRkPbSlEo1eCIy\/RI7aRxBgAh4m7gFSOY23KGhoVkHmkkjXJpDCuiONzUBHNnBKni6wZPJl9hJ40jaeGTdFAwbzDtZduzYETgC4uf1Jo1waQ7ph0tb0oIjO5gCT+\/zpC6xM+3+GGkcSRuPrBMg6hyQtra2SI5ez3z1SSNcmkPaMVT5sxIc+cMrqdTgaTbyJt4fI40jaeORlQJk9+7dtGvXrvRhYVG+gKQRLs0ho+Q+qbLBUVLI+6sXPN2Ml2n3x0jjSNp4ZJ0AUVMw\/H9TU5O\/N0aA1NIIl+aQASg1Pgs4Mp4ix0DwlJ0nk45ul8aRtPHIOgGS6yh2P7tg\/LzepBEuzSH9cGlLWnBkB1PgKTdPpogQaRxJG4+sEyBxv5qkES7NIePuD3HUB47iQDl8HeApP4Ym3B8jjSNp45F1AiRXBIQbwgeL8QVzvLVW1yONcGkOqYtnk8oBRyaxkdsW8FSYp6Tvj5HGkbTxyDoBwgbzNtyJiQninTDqd\/6\/oqLCOVb96aefLuwZHlNII1yaQ3qk0apk4MgOusCTN57cIuTQl1fQstJ53jJqSCWNI2njkXUCJN9ldHxK6t69eyFA8jiuNIfU8I4yrghwZBwlWQ0CT\/54SuLodmkcQYD463PaU6tL6Xi6xR0BGR0dpe3btzv3uqjPdVQujXBpDqmDY9PKAEemMZLdHvDkn6e4RYg0jqSNR9ZFQNjgzHUgvANm3759tGfPHqqrq9O6PVca4dIc0v8r0Pwc4Mh8jthC8BSMpzjvj5HGkbTxyEoBEqzbB8sljXBpDhmMVbNzgSOz+VHWgafgPKn7Y3hNyMq7FwQvqEBOaRxJG4+sFCA9PT3U19c3q+vhHBBvPizNIb212q5U4MgOvsBTOJ7U\/TFTTz0UrqA8uaVxBAESWVfxVrD7JttXX33V2fly7tw5J3MUJ6NKI1yaQ3rrNXalAkd28AWewvMU9f0x0jiSNh5ZFwFx74I5ffo0HTt2jDZu3EhR3Q8jjXBpDhn+FWheCeDIPE6yWQSe9PAU5f0x0jiSNh5ZJ0B4FwxvtWXRcenSJWpubqYLFy4QpmC8vQykOaS3VtuVChzZwRd40sdTVEe3S+MIAkRfnwtc0smTJ+nWW291TjwdGxujbdu2eToBVW3hHRkZmXVqKpexfv16x56urq5ZUznSCJfmkIE7kcEZwZHB5LhMA096eYpChEjjSNp4ZF0EJEyX58WrlZWVjsBg0aGmb1pbW6mjo8MpurOzk3p7e6m0tNT5XRrh0hwyTH8wNS84MpWZ2XaBJ\/086b4\/RhpH0sajohEg2U5Q5cazEGFh0t\/fTyUlJdTe3k6pVIpqa2shQPS\/X1CiBwSkvTQ9NNnKJOApGtp03h8jjSMIkGj6XMFS811Cx5kLrQFRAoTTuqdgeB0J3x\/T3d3t2MACxH2YGRPufo4cOVLQVpMTTE5OUnl5uckmFr1t4MiOLgCeouPpwjvX6fHvvU0\/vXqdHv\/0nXT\/kmD3x0jgqKGhYRbQ4+Pj0QGfQMlzZmZmZhKo11eVmQJkaGgoHaXwUtCZM2ecBatf+9rXnHx8oR0f37527Vo6ePBgXgEiiXBpfxF44d62NODIDsbAU\/Q8hT26XRpHiIBE3+cK1pDtKHaeQlHrNrgAJTh4h0xjY6NzT8zOnTudtR5q8SpPvWzatIn279+PKZiCqCNBXAhIe2nGhVvc9YCneBAPc3S7NI4gQOLpc75qca\/jcIuQzELci1BVBMQtTDg9FqH6gh6JI0BA2kszAoiMKBI8xUdD0KPbpXEEARJfn8tZkzu6wYk4wsFrOHgRab7HHTlxrxlxb8PNnNqRRrg0hzSgO2o3ARxphzSSAsFTJLDmLDSICJHGkbTxiMm2bg1IoQWnOt1CGuHSHFIn16aUBY5MYSK\/HeApfp7U0e3PpO6h1AOLCxogjSNp45GVAiRbr4tKlEgjXJpDFnwDWZgAHNlBGnhKhiclQliAsBDJ90jjSNp4ZI0ASaar4yCypHAv5nqlvTSlcgmekmPW6\/0x0jiCAEmuzyVSszTCpTlkIp0i4krBUcQAayoePGkCMmAxXo5ul8aRtPEIEZACnV8a4dIcMuC7y+hs4MhoetLGgafkeSp0dLs0jqSNRxAgECDJv0VgwSwEpL00pdILnsxglkXImmdP0LKF8+jQ5hWifQkCxIw+F5sV0gjHSzO2rhO4InAUGLpYM4KnWOHOW1mu+2OkcSRtPEIEBBEQc94isMRBQNpLUyqt4MksZt0iZN+6e2jl3QvE+RIEiFl9LnJrpBGOl2bkXSZ0BeAoNISxFACeYoHZdyXu+2OW\/Mplqqqq8l2GqRmkjUeIgCACYqqvFa1dGNjsoB48mcuTuj\/mubWL6fMr858VYm4rbrYMAsQmtjTYKo1wvDQ1dIqIiwBHEQOsqXjwpAnIiIoJcnR7RKZoK1baeIQICCIg2pwDBelBAAObHhyjLgU8RY1w+PJ3vHiC+v7hinNiqpej28PXGG0JECDR4mtc6dIIx0vTuC52k0HgyHyO2ELwZD5PzNHY2yXE0ZC21VXUtrrSfKPzWChtPEIEBBEQqx1SovEY2OxgFTyZz5PiyOvR7aa3CALEdIY02yeNcLw0NXeQCIoDRxGAGkGR4CkCUDUX6ebIy9HtmqvXXpy08QgREERAtDsJCgyHAAa2cPjFlRs8xYV08HoyObJdhECABO8Liec8c+YMNTc304ULF6impob6+\/uptLSUxsbGaP369Y59XV1d1NTUlLZVGuHS2pN4p4rAAHAUAagRFAmeIgBVc5HZOCp0f4xmE7QWJ7HPzZmZmZnRipKBhU1PT1N7ezvV1dU5AqOnp8excuPGjdTa2kodHR3O752dndTb2+sIE36kES6tPQZ2tdAmgaPQEMZSAHiKBeZQleTiKNfR7aEqiyGzxD5XFAKE+waLjsrKyrQA4Z8rKiqczzkaUlJS4oiUVCpFtbW1oQWI186SK122zzM\/y\/cf4eqhAAAgAElEQVS7+zv1s1ebcvmSl\/z50uhqU7a2BfX\/MG3yyl0ueyVz5BbwXjDOx5+X\/Dr7XS6\/0tXvvLQn3x9AXvwoM3+hPujVJinvhmxHt2e2zQsmUfW7KN4NQd+RUeYrGgGiREhfXx8NDQ05IoOnX4aHh6m7u9vB2B0lUQ4cJfgoGwgAASAABJJD4N2V2+nfb72Dbv9uW3JG+Kh5fHzcR2rzkxaFAJmamqKWlhZqa2tzhIeagqmvr88rQMynDxYCASAABIBAGAT46PZDm1eEKQJ5AyIgUoC4F5w2NjbSI488Qk899VR6fQdHPliEbNq0ifbv359zCiYgpsgGBIAAEAACQAAIFEBApADJbHNmBOTAgQM0OjpK27dvp507d+ZchIreAwSAABAAAkAACESDQFEIEIbOyzZctTYkGqhRKhAAAkAACAABIKAQKBoBAsqBABAAAkAACAABcxCAADGHC1gCBIAAEAACQKBoEIAAKRqq0VAgAASAABAAAuYgAAFiDhewBAgAASAABIBA0SAAAVI0VKOhQAAIAAEgAATMQQACxBwuYAkQAAJAAAgAgaJBAAKkaKhGQ4EAEAACQAAImIMABIg5XMASIAAEgAAQAAJFgwAESNFQjYYCASAABIAAEDAHAQgQc7iAJUAACAABIAAEigYBCJCioRoNBQJAAAgAASBgDgIQIOZwAUuAABAAAkAACBQNAhAgRUM1GgoEgAAQAAJAwBwEIEDM4QKWAAEgAASAABAoGgQgQIqGajQUCAABIAAEgIA5CECAmMMFLAECQAAIAAEgUDQIQIAUDdVoKBAAAkAACAABcxCAADGHC1gCBIAAEAACQKBoEIAAKRqq0VAgAASAABDIhsDlb+3WAsz1ixNayslVyKLNA5GWH3fhECB5EP\/whz8cNx+oDwgAASAgFoE\/KbvitO2Dv\/bLdBsX\/9r19M8fvOX9n92fRwnIz\/5trrbi\/88v9JWVzag\/\/LtpbbaaUBAESAEBMj4+HognFi9e8uZKl+3zzM\/y\/e7+Tv3s1aZcDfaSP18aXW3K1rZAJBFRmDZ55S6XvZI5Yj5s7Xe5\/EpXv\/PS59z4ZfZtL36Umb9QH\/Rqk7KFIwbqr\/1fXpygiRPHqXxJefqzbP7IAz2n4WfuByrTSX51USV956Xv0Ocf\/jzNXfT+5\/e3D856h3p53+niKB\/+7rbpfN\/p5ijoOzHOfBAgEQmQOEn0Wpffl4zXcpFOHwLgSB+WUZYkkScWFNd\/PkEsKK6dOub8z8+1U0dvgpKFghIRLCD4mXdvvfP\/\/FVfihJ6z2VL40hae5hICBAIEM8OjYTRIyDxJRM9avHXYDNP2YSGW2SoKETJvavSwsIUUeGHaZs5ytZOae2BACnQm6URfvbsWaqqqvLjw0gbMwLgKGbAA1ZnC0+ZYuPq0b9Kt1hFMTiCwdELG0VGPvps4chrF5Q2HkGAQIB47ftIFxMC0l6aMcEWezUm88TRjOlTx2j6n46mp09YbHBEQ9ouCgiQ2Lu+1goxBVNEUzAmvzS19mqLCwNHdpBnEk8c5Zg+dZSuHh0sasGR2XNM4khHr0YERAeKFpUhjXBpDmlRV\/JsKjjyDFWiCZPmiUXH1e8PpqMcKsIhcSolKNFJcxTU7lz5pI1HmILBFIxuH0F5IRGQ9tIMCYex2ZPgKTPSUYzTKn46RBIc+bHPb1oIEL+IWZ5eGuHSHNLy7pXVfHBkB6tx8aREB2+L5QWkEB3e+0dcHHm3KFxKaeMRIiCIgITzCOTWjoC0l6Z2gAwpMA6e+LAvtWuFF5BiesUf+XFw5M+icKkhQMLhZ11uaYRLc0jrOpQHg8GRB5AMSBIVTxzxUMKDox28NXb+QxtmnRBqQPOtMCEqjpJqvLTxqKgiIGNjY7R+\/fp03+nq6qKmpiZyf64+U4mkES7NIZN6EURZLziKEl19ZevmiSMdahfLvHtX0fxVG8Sdy6EPfW8l6ebIW63RpZI2HhWVADlw4ABNTExQW1tbuodMTU1Ra2srdXR0OJ91dnZSb28vlZaWOr9LI1yaQ0bn6smVDI6Sw95Pzbp4UsKDj0AvtnM6\/OAdJK0ujoLUHUUeaeNRUQmQnp4eqqysdKIe6uHoB3\/e399PJSUl1N7eTqlUimprayFAovAglFkQAWkvzYINtjRBGJ7UwlJ1BTymWaLpBGE4isaicKVCgITDL7HcHOloaWmhkydPOjbU1NQ4ouP06dM0PDxM3d3dzucsQOrq6tIiRRrh0hwysQ4VYcXgKEJwNRYdhCd1dodaWMrCY+EXdmm0CkW5EQjCkckIShuPiioC4u5YPB0zOjpKa9eupYMHD+YVIO58R44cMbl\/FrRtcnKSystvXIeNx0wEwJGZvGRa5Yuny5M086OXiPgfPx9\/mOZ85it2NNRiK31xZGg7GxoaZlk2Pj5uqKXBzCrKo9jV1MumTZto\/\/79mIIJ1neQKwIEpP3VFgFERhTphSdEPJKlygtHyVror3ZEQPzhZUxqnoLZvXs37dq1y1lgyus++Nm4cSMWoRrDEgxhBKS9NKWymo+nbMIDW2nj7wnSfAkCJP4+pK1G93ZbtQaExYj786GhofQCVK5YGuHSHFJb5zCoIHBkEBl5TMnGExaXmsWdNF+SNh5xbynKKRivbiKNcGkO6ZVHm9KBIzvYyuSJF5aqXS3Lnj1rRyOEWynNl6SNRxAgBRxQGuHSHFLi+xMc2cEq87T0tjnOjbTY1WImZ9J8Sdp4BAECAWLmm6OIrZL20pRIJU+1nP9vXyd6+WnniHRspzWTZWm+BAFiZj+LzCpphEtzyMiIT7BgcJQg+AWqVotLL3\/7caKF5bTw04\/gHA9z6RK3oFvaeIQICCIgBr8+itM0CBCzeFcLS9U9LRzx4MPD3q6op6qqKrOMhTWzEJDmSxAgRdbBpREuzSEldkdwZAarmVtpM+9pAU9m8JTPCmkcSRuPjI6A8GmlO3bsmNW\/Mm+rjdoFpBEuzSGj5j+J8sFREqi\/X6f7Vtp86zvAU7I8ealdGkfSxiMjBYg6lyOb2FCiJPO8Di+dMUgaaYRLc8ggnJqeBxzFy1C2KRaOdsxftYHm3bsqpzHgKV6egtQmjSNp45FxAoRPLB0ZGaENGzbk7W+Dg4MF0wTpsJl5pBEuzSF1cGxaGeAoekbU9Mr0Px2la6eOOjtZWHTMu7fe2dHi5QFPXlBKNo00jqSNR8YJkGS76821SyNcmkOa1l902AOOdKA4uwwV5bh26lj6zA4lOhZtHghUIXgKBFusmaRxJG08MlaAcCSkpaWFTp48eVOHLSsro4GBAaquro68M0sjXJpDRt4BEqgAHOkBnddyZAqOuR+opJKPrtKydRY86eEpylKkcSRtPDJWgLBhvN5jYmKC2tranD7Kv\/NTUVFBw8PD9PTTT0fZd52ypREuzSEj7wAJVACOvIOuIhucg8XGLy9OOFMq\/HCEQ6fgyLQKPHnnKamU0jiSNh4ZK0Ayb69lQ9VnW7dupb1792oTIO7L6DIXvkojXJpDJvVii7JecPQ+uiww+Ln+84n3xMUx53e30HCLjV9dVOms47jxf+4FpDr4A086UIy2DGkcSRuPjBUg09PT1N7eTjzd4o6AjI6O0vbt2+mb3\/xm+vMwXZhFTWtrK3V0dDjFdHZ2Um9vL\/EtuYiAhEEWeYMiIO2lWQgHdY\/K9Z+fIxYcLC5uRDRuRDLUwxENR2x8oPI9geF9wWghG4J8X2w8BcEo6TzSOIIAibFHZa4DqampoX379tGePXuorq6OmpqaQlvD0Y+enh7q7++nkpISR\/SkUimqra2FAAmNLgoIgoCtL00WEiwi+FGRC\/5ZCQqOYrg\/zycu+Ds\/O1KC4Bw2j608hW23TfmlcQQBEmPvcwuQF154gV555RVn663OxacsQHg9SXd3t9MyFiBucSONcGkOGWN39FyV+oveGYjfG5A9Zyaiy1cu08IFC3NmyTWIuzOoQT9bISwEsn7+XuTBj63ZRAR\/xlEKfngqRD031mRUOL963eoa1JY48sGX4kA5XB3SOJI2HjG7c2ZmZmbC0aw\/t5qCYTHAC1Hr6+udSpRY4GiFjqeQABn\/ozk6qkEZRYjAhbmLI2v1T38lf9kX5n4wZ90\/zWFXzjI\/\/nC6rLL5c52f\/\/QTCyJrmy0FT05OUnl5uS3mFqWdEjhqaGiYxd34+LgoLo0UIO5FqM8\/\/7wjQJYvX067d++mXbt2pddohGUCUzBhEUR+3QjE+Vdbz+HZ0ZDzU9OzmvPW1LVZv5+\/\/P7v5zO+y4XDstJ5zlfLFt74f2npPFpWWkJtq9+PjujGMI7y4uQpjvZIrEMaR4iAxNRLs0VAjh07RhcuXHCmS3RFQLAINSZCUY1nBCS9NId\/+DNyCxUWOErUsJjJJmLcguXBu29MRZkoViTx5LlzWpZQGkcQIDF2wGyLUHmxqNqhossU9zbczDtmpBEuzSF19QGTyilWjo6\/cYXecomSH7xx2aHl+JtXZtHDAoWjKSxO+OfUA9FNdeXrF8XKk0m+UsgWaRxJG4+YPyOnYAp1rLi+l0a4NIeMqx\/EWQ84uhltFSn5wZtXnKiJiqQocaKiJqkHPhSbKAFPcXpFsLqkcSRtPDJOgOQ7gp2N5a24UURBcnVvaYRLc8hgryWzc4Ejf\/ywIOGpHn44auIWJSxIHrxrAa28W\/+iWfDkj6ckUkvjSNp4ZJwAcXfSXEex6zj\/w6szSCNcmkN65dGmdOAoHFs3IiTXiKMlbkGy8q4FzrSNrvUk4CkcT3HklsaRtPHIWAGS7yh2nbtgCjmBNMKlOWQh\/mz8HhzpZc0dIek5fNYpvG11VWghAp708hRFadI4kjYeGStA3LtgVMSDTyzVvQumUKeXRrg0hyzEn43fg6PoWFNiREVGbixi\/VAgMQKeouNJV8nSOJI2HhkrQNiwuHbB5Ovs0giX5pC6XlQmlQOO4mFDiRGOigQRIuApHp7C1CKNI2njkdECJEzH05VXGuHSHFIXzyaVA47iZSOoEAFP8fIUpDZpHEkbj4wTIBz1GBkZce58yfcMDg4WTBOkw2bmkUa4NIfUwbFpZYCjZBjJFCL71t2Td\/cMeEqGJz+1SuNI2nhknABhg9TBYF1dXTfdeMs7Y3bs2EGZB4b56ZR+0kojXJpD+uHSlrTgKFmmWIhsGX7d2c6bb7EqeEqWJy+1S+NI2nhkpABRHUuJDXdHyyZKvHTEoGmkES7NIYPyanI+cGQGO3xPDq8P4e27hzavuMko8GQGT\/mskMaRtPHIaAFiQveWRrg0hzShj+i2ARzpRjR4eSoawvfWZE7JgKfguMaVUxpH0sYjCJACniCNcGkOGdeLLM56wFGcaHura80zJyhThIAnb9glmUoaR9LGIwgQCJAk3w+oOwsC0l6aUkhmEcLrQqaeeshpEngyn1lpHEGAmN\/nclrovvWWE6n1JO7PM9eYSCNcmkNa3B1zmg6OzGV18\/Drzr0zh768gpb8ymWqqqoy11hYJk4kShuPjI6AuA8ie+GFF+iVV15xtt5WV1cHcq3Mu2W4EK6jtbWVOjo6nDI7Ozupt7eXSktLnd+lEY7BLVDXiTUTOIoVbt+VqUjIc2sX0+dX3uM7PzLEh4A0X5I2HhkrQNxHsU9MTFB9fb3Ta4eHh6m7u5tKSkp892I+yr2ysnLW1l6OfvDnfMMul9ne3k6pVIpqa2shQHwjjAw6EJD20tSBiWllsAgZv\/gu\/e\/HP2WaabDHhYA0X4IAial7uy+je\/755x0Bsnz5ctq9ezcFuYwu17Hup0+fTosabhoLkLq6urRIYcLdz5EjR2JCIJpqJicnqby8PJrCUaoWBMCRFhgjLeTCO9epcXCS7l8yjzgSgsdMBCT4UkNDwyxwx8fHzQQ7oFVzZmZmZgLmjSxbtgjIsWPHtF1Gx9Mxo6OjtHbtWjp48KATVcklQCQRLu0vgsg6YIIFg6MEwfdRNfN0\/zcmtNys66NaJPWBgDRfQgTEB\/lhk4a5jO7MmTPU3NzsCJbGxsabpm3U1MumTZto\/\/79mIIJSxbya0NA2ktTGzCGFcQ8jb1dQrwwlRelrrx7gWEWwhxpvgQBEmOfVotGN27cSC0tLXTy5MnAR7C7p3R4gSmv++CHy8Yi1BhJRVUFEZD20izYYEsTKJ54PQg\/2U5LtbRpYsyW5ksQIDF1TZ6CefLJJ51dL6+++irxQlSeLuFL6B577LFAi1Dd221ramqcqAeLEffnmXfMSCNcmkPG1B1jrQYcxQp34MoUT3xa6see+HtMxQRGMrqM0nxJ2njEzBu5BiRzESrvXvnMZz4TeBFq0C4ujXBpDhmUV5PzgSOT2XnfNjdP6t4YTMWYxZ00X5I2HhkrQFQE5HOf+xz19fWlz+kIEwEJ4hrSCJfmkEE4NT0PODKdoRv2ZfKEqRjzeJPmS9LGI2MFCBumpkYeffTRWWs1gh5EFsQ9pBEuzSGDcGp6HnBkOkPZBQh\/WvrV72NBqkH0SfMlaeOR0QLEhH4sjXBpDmlCH9FtAzjSjWg05WXjiXfE\/ODNK\/Tazk9GUylK9YWANF+SNh4ZLUB4pwpPv7gf9+JRXz0xYGJphEtzyIC0Gp0NHBlNT9q4bDypBanPpO6h1AM4oCxpJqX5krTxyFgB4r6jhXfBVFRU0Llz55z+3NTUFFu\/lka4NIeMrSPEWBE4ihHsEFXl4kldWKduzQ1RBbKGRECaL0kbj4wWIOrYdT4unU9B5TM7gh7FHrQfSyNcmkMG5dXkfODIZHbety0fT7wWpG11FbWtrrSjMUKtlOZL0sYjYwUI74LZu3evIzouXbqUPtUUUzDh3hTSHDIcGmbmBkdm8pJpVT6ehn\/4M+eEVF4Lsqx0nh0NEmilNF+CAImxk\/LJp7feeivxrhfeEbNt2zYaGBhwfo\/rkUa4NIeMqx\/EWQ84ihPt4HUV4okPJ3vwrgXE60HwJINAIY6SsSp4rdLGI2MjIMEp0ptTGuHSHFIv22aUBo7M4KGQFYV4QhSkEILRf1+Io+gt0FuDtPHIaAHCd8Hs2LFjFoOYggnXoaU5ZDg0zMwNjszkxc8UjEqLKEiyXErzJQiQmPqTugm3ra2NamtrY6r15mqkES7NIRPrGBFWDI4iBFdj0V54QhREI+ABivLCUYBiE8sibTwyNgKSeXttEMbVbbosYvhRoobXljQ2NlJ3d7dzqZ37Mrqurq5Z23ylES7NIYP0C9PzgCPTGbphn1eeEAVJjk+vHCVnob+apY1HxgoQNowFBD9Bzv1Qh5jxMe5KgPBnfKndmjVrqL29nVKpFC1fvpxaW1vTd810dnZSb2+vc0suP9IIl+aQ\/tzXjtTgSBZPiIIkx6c0X5I2HhknQNxRimzd1ssaEBYufHAZnx3CDwuQzCkdFR2pr68nFib9\/f1ONEQJEzXtI41waQ6Z3KstuprBUXTY6izZD0+IguhE3ntZfjjyXmpyKaWNR8YJEJ3UsrBwCxAV6eBtvCxARkdHae3atXTw4EFnOoYfFiB1dXXpqAsT7n6OHDmi08TYy5qcnKTy8vLY60WF3hEAR96xSjKlX57u\/8YEjWwop7Lb5yZpdlHV7ZcjE8FpaGiYZdb4+LiJZga2ac7MzMxM4NwRZDxz5kz64DH3Wg2\/VekSIJIIl\/YXgd8+YUN6cGQDS97XgKjWrHnmBC0tnYdzQWKkV5ovIQIScefhE1Dd0yBq3UaudSAq\/cjICJWVlc06qCxTgLS0tDjTMTy9gimYiIlE8YERkPbSDAyE4Rn98nT8jSu05tkTdOjLK2jl3QsMb50M8\/xyZHqrIUAiZihz9wtHQw4fPkxbtmzxXbNbgHBmLEL1\/1ebb9CRITQC0l6aoQExtIAgPHEUhJ9Dm1cY2ipZZgXhyGQEIEAiZiebABkcHKTHHnvMWSTq58kUIO4Fru7dMe5tuENDQ7POHZFGuDSH9NMfbEkLjuxgKghPiILEy20QjuK10F9t0sYjbr1Ra0B0ChB\/1GZPLY1waQ6pg2PTygBHpjGS3Z6gPCEKEh+\/QTmKz0J\/NUkbj4wUILxWgw8Ly\/Z42Ybrj9L8qaURLs0hdXJtSlngyBQm8tsRlKfzU9eIt+XyJXWpBxbb0VhLrQzKkanNlTYeGSdATCNeGuHSHNK0\/qLDHnCkA8XoywjDU8\/hCeo5fJZe2\/lJWlY6L3pji7SGMByZCJm08QgCpEAvk0a4NIc08SUR1iZwFBbBePKH5QlTMdHzFJaj6C30V4O08QgCBALEnwcgdeQISHtpRg5YQhWE5UktSMVUTHQEhuUoOsuClQwBEgw3a3NJI1yaQ1rbsfIYDo7sYFUHT5iKiZZrHRxFa6G\/0qWNR4iAIALizwOQOnIEpL00IwcsoQp08YSpmOgI1MVRdBb6KxkCxB9e1qeWRrg0h7S+g2VpADiyg1WdPJV+9fvUtrqK2lZX2tF4S6zUyZEJTZY2HiECggiICX4FG1wISHtpSiVXJ09YDxJNL9HJUTQW+isVAsQfXtanlka4NIe0voMhAmIthbp9Sa0HwV0x+rqEbo70WRasJGnjESIgiIAE8wTkigwBaS\/NyIBKuOAoeOL1IMffvIIL6zRxGwVHmkwLVAwESCDY7M0kjXBpDmlvz8ptOTiyg9UoeeKTUvetuwe35obsClFyFNK0QNmljUeiIyAHDhygiYkJamtrc8h2XzrHv3d1dVFTU9Osz9VnqndII1yaQwbyYsMzgSPDCXrPvCh5QiRETx+IkiM9FvorRdp4JFaA8E24fX195L71NlOQcOP58rvW1lbq6OhwekJnZyf19vZSaWmp87s0wqU5pD\/3tSM1OAJPjIBaEzL11EN2AGKgldJ8Sdp4JFKAsNCoqKigY8eOOS6hIiAsSiorK52oh3o4KsKf9\/f3U0lJCbW3t1MqlaLa2loIEANfKMVgkrSXplTO4uBp+Ic\/o83Dr9PKuxbQoc0rpEIZWbvi4Cgy47MUDAESJ9oh62JhoQQIRzrct+yqW3VPnz5Nw8PD1N3d7aRlAVJXV5cWKUy4+zly5EhIq5LNPjk5SeXl5ckagdrzIgCO7OggcfH0459co8e\/97YDyuOfvpPuX4LL67z2kLg48mpPkHQNDQ2zso2Pjwcpxtg8c2ZmZmaMtS6EYW4BklkMR0lGR0dp7dq1dPDgwbwCRBLh0v4iCNE9jM0KjoylZpZhcfOkpmQ4GrIvdQ9u0fXQTeLmyINJoZIgAhIKPv2Zp6ennajFyMgIlZWV0cDAAFVXVzsV5RMgaupl06ZNtH\/\/fkzB6KcGJQZEQNpLMyAMxmdLgic+sGzLi6\/T+alrODnVQw9JgiMPZgVOAgESGLr4M2ZOwezevZt27drlLDBV323cuBGLUOOnBjXmQUDaS1Mq2UnyxNGQ4R\/+1IE29cCHKPXAYkREsnS0JDmKot9DgESBakRlZkZA3Ntw1RoQFiPuz4eGhtILUNksaYRLc8iIuk6ixYKjROH3XLkJPCkhwhGRZaXznKgIixE8NxAwgSOdXEgbjxgbsWtAdBAvjXBpDqmDY9PKAEemMZLdHpN4YgHCO2Z6Dp91hAhHRXCxHQSIDZ4EAZKHJQgQG7qwLBtNGthkIau3NSbypITID9647Bzpzg8vWn3w7oVFKUhM5ChML5Q2HiECUqA3SCNcmkOGcWZT84IjU5mZbZfpPCkxwlZzZIQfjo4sWzivaM4UMZ0jvz1d2ngEAQIB4tcHkD5iBKS9NCOGK7HibeMpW3REiZIH71pAy0pLiP9fefeCxDDVXbFtHBVqPwRIIYSEfS+NcGntEdbdnOaAIztYtZ0nFiQ\/ePOKs6XXPWWj0FfREp6+UULFtgWutnOU6QnS2oMISJFFQCR2YDuGK+9WgiPvWCWZUipPbmFyfmqa3pq6ll5P4sabBYojTBbOo6U8tVNakhYqSxfOMyKSIo0jae2BAIlQgHjtLLnSZfs887N8v7u\/Uz97tSkXLF7y50ujq03Z2hZ0MArTJq\/c5bJXMkfuaI4XjPPx5yW\/zn6Xy6909Tsv7ckXDfPiR5n5C\/VBLzaxOOF\/b12+8T8\/SqSM\/uMb9O+33pnXDf\/Dv76dvgqChYt6WMC89NJL9PDDD6eFDH\/Xt+l3yH0StZf3nS6OvEYjo+p3Ubwbgr4jo8yHXTB50OVOgAcIAAEgAASCI\/Bvyx50MrsFyr\/fesdNn91Ic+PzzPTBay+ck4WRruc\/\/OslXUVlLefn\/\/XPIi0\/7sIhQOJGHPUBASAABICAMQjwgW46H44KRfU8k7onqqITKRcCJBHYUSkQAAJAAAgAgeJGAAKkuPlH64EAEAACQAAIJIIABEgisJtTKd8o\/O1vf5v+5V\/+hR555BGqqqoyxzhY4iBw+fJl+uu\/\/muanJykdevW0cc+9jEgYygChw4doqVLl9KKFSsMtbB4zbp+\/Tp95zvfoR\/96Ef0x3\/8x+DIgK4AAWIACUma8N3vfpfuuOMOqq6udga5L33pS1RScmNLHR4zEDhw4AB95CMfcf4999xz9MUvfpEWLrxxPgMecxA4ffo0Pf\/8885ujtraWnMMgyUOAt\/73vfolltuod\/6rd+iv\/3bv6Xf+73fw7su4b4BAZIwAVFVPzU1Ra2trdTR0eGIC354INuxY4fzs7r5lwe03\/\/933f+ahscHKTGxkbiW4LxRI+AV46UJRwJ+cu\/\/Et69NFH6bbbboveQNRAXjl699136W\/+5m\/ogx\/8oMMNBEh8nccrR+w7LEBOnTrliPiPfvSj8RmJmrIiAAEisGOcOXOGmpubnZYNDAw4AoQ\/6+zspN7eXuK\/1IaHh6m7u9v5n0XHokWLIEBi7At+OOKIFIuPvXv3UiqVouXLl8doafFW5ZWjrq4uOnr0KP3Gb\/wGXbx40QEMAiSefuOVI37XPfvss\/SpT32KfvM3f5P6+vpow4YNiCTGQ1POWiBAEiZAdyGc2GgAAAZSSURBVPX81wCHgTm8+Pjjj9OePXscAcLRj9HRUUd08LoPFR3hvwb4L4HKykpHgDQ1NdHtt9+u2yyU50LAL0e\/\/uu\/7nDK02NLliwBljEg4Iejbdu20euvv05vvfUW\/eQnP3EGta985SuIUkXMkx+OOBJ8\/PhxevDBB533If9h9gd\/8AeI9kbMUaHiIUAKIWTp9\/yXwfbt22cJkImJCWpra3PCyi0tLc7PPPXC0zD80mTH\/OxnP2tpi+0z2wtHzOGJEyecge3OO+8kFiNf+MIXMLjFRLcXjtiPVMRjbGwMEZCYuFHVeOWIp8dYeHC0d968eU6UeO7cuTFbi+rcCECACO0PXp0SoeLkOgA4Sg57rzWDI69IJZcOHCWHfdiaIUDCImho\/mxOmW0KRi1QNbQZos0CR+bTC47AkfkI2GshBIi93OW1PPPFmWsRKrbcJtcBwFFy2HutGRx5RSq5dOAoOezD1gwBEhZBQ\/NnOiWbqbbhlpWVpXfHGGp+UZgFjsynGRyBI\/MRsNdCCBB7uYPlQAAIAAEgAASsRQACxFrqYDgQAAJAAAgAAXsRgACxlztYDgSAABAAAkDAWgQgQKylDoYDASAABIAAELAXAQgQe7mD5UAACAABIAAErEUAAsRa6mA4EAACQAAIAAF7EYAAsZc7WA4EgAAQAAJAwFoEIECspQ6GAwEgAASAABCwFwEIEHu5g+VAAAgAASAABKxFAALEWupgOBAAAkAACAABexGAALGXO1gOBIAAEAACQMBaBCBArKUOhgOB7AhMT09Te3s7jYyMzErw6KOPUltbm9Ww8d0shw8fppaWFqeNdXV11NTU5LRJtTuVSlFtbe1N7eTv9+7dSxs3bqTS0lKrcYDxQEACAhAgElhEG4CACwE1ELsHZwkAuQUE3+LsV4AwBkrAbNmyRQIkaAMQsBoBCBCr6YPxQOBmBPIJEL4ReXR0lM6fP0\/r1q2j++67j5qbm+nChQtUU1ND\/f39TnRgbGyM1q9fT3xz8qpVq2j+\/PlO5IAjDxxF4QgDlzUxMeH8rm5aZmtUpIXL6Ovrcww8duwYNTY2Und3N7F46OnpSX83NDTkpBkeHna+54fFRWYkg8s7d+6cE\/HI1kZ3BITrU3Vzee669+3bR6tXr6bq6mp0HyAABBJEAAIkQfBRNRCIAoFsUzBKXLz88sv04osvOkKDn9bWVuro6HAGYyUo3EKD87EYYCGSS4DU19dnFQ9c\/rZt22hgYIDuuOOOtHjhz1mAsA2XLl2izs5Oeuyxx+gb3\/gG7dq1K\/1Zb2\/vrKkSzsN1sfjJNc3EZbOgcU\/BuPPx99xOftTUTRQcoEwgAAQKIwABUhgjpAACViHgJQLCkYbJycl09EM1kAXHpk2baP\/+\/eloSDZh4o6AVFZW0o4dO2ZhxFEQFgtKaKgpE45qcBRDRU7cmZRQ4M84guFer8JtevLJJ2nDhg2OWCoUAVECJFN8cNkcScks3yqCYSwQEIIABIgQItEMIKAQ8CNAOPqQGWngAVoJB56O8SJAsgkKdzleBIgSBtwOFelwt8mvAMkV6YAAga8AATMQgAAxgwdYAQS0IeBVgHA6XtPBa0F4OkKtD9m+fTvxIk2OQLinYLZu3Zpe+LlmzZr01AyLBTXVUl5enk5TUVGRNQLCDXVPwXB9e\/bsIc7Lu1T++Z\/\/+SZRpPJkTsHk2gXDURZ+sk2zYApGW1dDQUAgFAIQIKHgQ2YgYB4CXgUIRyV4V4jXRajuxabuxan5FqFmm4JR0zdq2qarqystFNwLWzORdUcu8k3BfPazn3WmkE6ePJkuQq2B4Ta7IynmsQeLgEDxIAABUjxco6VAIBAC+URBoAJzZIrjHA9sw9XJGMoCAuEQgAAJhx9yAwHxCMQhQKamppzpoGXLlqW36mYDNsz6jcyFrOKJQwOBgOEIQIAYThDMAwJAAAgAASAgEQEIEImsok1AAAgAASAABAxHAALEcIJgHhAAAkAACAABiQhAgEhkFW0CAkAACAABIGA4AhAghhME84AAEAACQAAISEQAAkQiq2gTEAACQAAIAAHDEYAAMZwgmAcEgAAQAAJAQCICECASWUWbgAAQAAJAAAgYjgAEiOEEwTwgAASAABAAAhIRgACRyCraBASAABAAAkDAcAT+P02DBPmXtrf1AAAAAElFTkSuQmCC","height":328,"width":544}}
%---
%[output:8b7a3c75]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:316ce442]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:0fe8f485]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:2b7bb8c2]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:846b087d]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:3a877cb5]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:6277dd77]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:73a0a02c]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:4a1ff22d]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:5b1728ee]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00756 H\nPer-System Quadrature Axis Inductance: 0.00624 H\n---------------------------\n","truncated":false}}
%---
%[output:0445a065]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.012443765785704"}}
%---
%[output:436a1909]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.517590710054527"}}
%---
%[output:098ea60f]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.152987786896029"}}
%---
%[output:0a3c7cf9]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"  93.745795204425832"}}
%---
%[output:3d975c78]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -1.529419475013042e+03"}}
%---
%[output:4c275a97]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAiAAAAFICAYAAABpxkW2AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ90HtV55i8p26CEsLYMlAjFMTFygE1qY5ccoTghqQ54u1uZNG0qyWd3U0fNuidQ9tS4khwCGxqCJR2b3Rz+NCpVdNjdWvZuC6nd05YmLiFQ4YYarDQbUgTCGCFIDMIFgmnD1nveIVdcjWbm3jvzzDfzjZ7vHA62584zM7\/3\/e59vvtvTjl58uRJxQ8JkAAJkAAJkAAJ1JDAKTQgNaTNS5EACZAACZAACQQEaECYCCRAAiRAAiRAAjUnQANSc+S8IAmQAAmQAAmQAA0Ic4AESIAESIAESKDmBGhAao6cFyQBEiABEiABEqABYQ6QAAmQAAmQAAnUnAANSM2R84IkQAIkQAIkQAI0IMwBEnAkMDs7q3p6eoLSIyMjqrGx0fFM\/2KTk5Nq8+bNat26dWpgYEA1NDT4i6Q4o5bP6HJ7msNv\/\/Zvq87OTpdTSl1mcHBQDQ8PB\/e4ZcsW1dfX53W\/e\/fuVdu3b1c7duwoJQ95voMHD+b+\/fCCxsKlJUADUtrQ8MbKRqCWjXOcAZEK\/rLLLlOtra254Il7xryvG\/cwaRo0aQDvv\/9+r8Y9zTm+AZBrbNq0ae60KhqQqhlG3xizvB8BGhA\/XixNAoUQOHHihOrv71f79+9Xu3fvrpkBkZ6XWlw3CqpuzDo6OpzNhO4h8Gnc05yTJgm0AfG5t\/B1yt4DovP06NGj7AVJkySL7BwakEUW8LI8rtkVLfdkdimbv\/7XrFmjvvSlLwW3LQ2RORyhf61PTEwsOG5qXHnlleo3f\/M3gzJNTU1qdHRUtbS0RKIwG\/pw+XDvgBzXQzLt7e3qlltuUatXrw4qXrPhtunIUI6+7qFDh4L7k48egrnhhhvU7\/3e7wXmQ3\/CLOTfNVPToOhGzyyvGzGtZTaI5jPedtttamhoKPK609PTwf3NzMzM3ZMZQ5OjML\/99tvV1772NaWfT\/iHWWt2emgrqrHVcTWvq583\/Fw61s3NzXMmKsxv3759wZCG\/pj5EdazGb9wPiZpJeVhUk+JyUTuWd972NSEv18mW62xdetWdeDAASXfHx0785nl3\/Q1zNjauETlYVnqHd5HuQjQgJQrHpW\/m3CjYz6wrkSjGplwwyE60vhr8xE+HtVAJjXecizu3nRjsWzZsnlzQLQBMe9BGvoow2CakLAOyoBE\/cIONwbhhimOq\/x7nAHp7e1VV1999QL2SQ2+HDvrrLPUsWPHAoMVZQrkmmZDGb73uLzQ133kkUcizcTdd989N+\/CzDezgQ0bkLCWPh5nQpJyVs55+umnY42OeU9h8xE2ibrx\/8hHPqIeeOCBefVFlImI+n6FDYSUibpH+Xd9HZu2yaXsvTSVr2Tr6AFpQOooWFW4VV3Bmr8AdeUtz2f++pdfubpiMyt4s7I0f\/mZDZY08voXutbQ1w7\/0tZco46blenll18ea0DMX4i+OjYDIr0+8rENhST10EivzIsvvriAifmrXTitWrVq3jO6DMGEh4c0ex1P6e0Ix13uReZDRPXMCMuNGzcGz2v2mLhMzHUZTgmXCf9dM9FmSe7fdm2de1HPo\/9NjKo8c9wQjMlR51M4pt\/4xjcCIxPVoxGnG+4F070+pkbStXUPic5\/GxfEUFMV6jo+g50ADYidEUsACcT9OtIVuFS8a9eujVwBYpY5cuRI5K9auVVTQ3516xUrtkmktl9ucQ28WSHL9X11UAbEvLaYCfmYDV5cw2A2wJ\/97GedDUhUj1HUdeU+wkNMcT0MUlYaUn0fJtuo64WHopIMSNzQU\/icpN6MKPMafjY9vBc2Mtp0xRkFW36a8TU14uIa7k3RrLQBiRt6M1d4mbmsv5fm8JeuGkwuUcN+wCqEUhUiQANSoWDWw6PU2oCYy1htFbyvcRDeUctyXXXMxjXcWIm2uQzXpQdEypgTN+XvsuQz3AMUbgB9DUiYY7iXJGx8UAZE53ec8ZGVQVEGJDyUY+sBqQcDEtXjpuMazr+4HhBTI+67QQNSD7Vq\/d4jDUj9xq4u7zztEEx4qECPqcf9mozqMrcZkKShE\/NXuYCXX4lxBsRVR7q2w+ZAD02FDYg08i6T+8KNs9lDEB7GkgbbNgQjvTO2BjyskXYIxkzouF6FcNKHzUS4N0A\/s9kTpp9H5074nKghGNuXLe8hGG1Wdc9RnAGJ6jnSjMI9IHGThsPDP0lDMFFcOARjyxYe1wRoQJgLNSWQ9yTUpAbcZkCS7i1qfkScAbHpSHe1ns8Rhu9iQOScqFUwWiu8ksHcwMtnEqruijfPket+8pOfDHpnoj7CKer5XCehiqY2ZWHjEzdB0zzHLCPX\/MpXvqJuuummBRNm5ZywAZF\/i5vQqp\/VZnijhidsPVAmx7hnTDIPZoN\/zTXXxOZWkobcQ9TkVNdJqCYXWw9gTSscXqzUBGhASh2e6t6c6zJccwmtbRlu1MRWnyEYoZ3UvW+b5GnujJqkI9cJL9m8\/vrr1eHDh+cmXUb1gJiNU9xEWlM7PDclyqCYDbF5rvxZG5Co6955553zdvSUzdHM+SZpluGaRsJsEKOWaMct\/w1zNeekiKZw27lzp9q2bVuAw+zJ0quZ4pb12vbvSFqGK9dy7RmIm7shvWBRjXtcr48wiloCHdWLEmde5d\/DO6\/GzaXRGi49ddWt2fhkPgRoQHxosWxNCNhWHNTkJniR1ASihnrCK53i9mExL5pmI7LUN73ITzQNozZaUStjbJi4EZmNEI+bBCplQKTCkj0KZPOkqAouaeMqpkV5CNCAlCcWae4kaQgqaego6lpptmJPc888J3oIRrjYNu+LMo1VeXcP8yJfApUxILZJa\/p4W1tb8BIn\/Xf5cvm+ECrfkFCdBqT+cyBs9uWJfM2HnMN3i9Q2F8JDoz7mQ+6UhrG28ar3q1XGgMg4qCS\/fOJ6QMLBkrHM8fHxmr5ttN4ThvdPAiRAAiRAAggClTAg8mvrxhtvVN3d3YEJoQFBpAY1SIAESIAESCA\/ApUwINKTIR\/ZqS9pDoiJUXcRd3V1BUMy\/JAACZAACZAACdSOQN0bEBkjvuuuu9R1112n5AVkLgZEz\/8QzObbVcPY3\/e+99UuErwSCZAACZAACcQQkDcXn3feeZXiU\/cGRIZcZA8C2dXRtgpGIudqPqSsGJCpqalKBbzIhyFPLH3yJE8sAawa85M8bQTq2oBEzbTXDxz12mzflS\/8AtnSx+84efrxspUmTxshv+Pk6cfLVpo8bYT8jleRZ10bkHD4bD0g0lsiuwMmDbuYmlUMuF\/KY0uTJ3liCWDVmJ\/kiSWAVatiflbagOgeD1kds2rVquDNpXq7ZZ0aSVtaVzHg2K+Enxp5+vGylSZPGyG\/4+Tpx8tWmjxthNyPP\/jEcfVr\/bep5\/\/4C+4n1UHJShkQNG9+gbBEn3rqqcpNosIS8lMjTz9ettLkaSPkd5w8\/XgllR57+Hl11dhjavaWj+NES6BEA5IQBBoQbIayQiJPLAGsGvOTPLEEcGo0IDiWdaNEA4INFSt48sQSwKoxP8kTSwCnRgOCY1k3SjQg2FCxgidPLAGsGvOTPLEEcGo0IDiWdaNEA4INFSt48sQSwKoxP8kTSwCnNnjvETV471OcA4JDWn4lGhBsjFjBkyeWAFaN+UmeWAI4NRoQHMu6UaIBwYaKFTx5Yglg1Zif5IklgFOjAcGxrBslGhBsqFjBkyeWAFaN+UmeWAI4NRoQHMu6UaIBwYaKFTx5Yglg1Zif5IklgFOjAcGxrBslGhBsqFjBkyeWAFaN+UmeWAI4NRoQHMu6UaIBwYaKFTx5Yglg1Zif5IklgFOTXVBlKS53QsUxLb0SDQg2RKzgyRNLAKvG\/CRPLAGcGg0IjmXdKNGAYEPFCp48sQSwasxP8sQSwKnRgOBY1o0SDQg2VKzgyRNLAKvG\/CRPLAGcGg0IjmXdKNGAYEPFCp48sQSwasxP8sQSwKnRgOBY1o0SDQg2VKzgyRNLAKvG\/CRPLAGc2sbbH1UPPnmck1BxSMuvRAOCjRErePLEEsCqMT\/JE0sAp0YDgmNZN0o0INhQsYInTywBrBrzkzyxBHBqNCA4lnWjRAOCDRUrePLEEsCqMT\/JE0sAp0YDgmNZCqXJyUnV29urhoaGVEtLS+Q90YBgQ8UKnjyxBLBqzE\/yxBLAqa256SF1dPZ1zgHBIS1O6cSJE6q\/v18dOnRIjY6O0oDUKBSs4LGgyZM8sQSwasxPHM\/GrfcFYtwJFce0MKWDBw+qwcHB4PrsAaldGFghYVmTJ3liCWDVmJ8YntLzIT0gNCAYnoWqzM7OqhtvvFF1d3cHJoQGpHbhYIWEZU2e5IklgFVjfmJ40oBgOJZCZe\/evcF9rF27lnNAahwRVkhY4ORJnlgCWDXmJ4bng08cVxvveJQ9IBicxanIxNO77rpLXXfddWp6etrJgJh3e+DAgeJuvgJXFubNzc0VeJJyPAJ5YuNAnuSJJZBNrb29PRD45+UfVq+t\/QwNSDacxZ8tQy6XXXaZam1tVVwFU\/t48BcRljl5kieWAFaN+YnhOfbw80q2YpcPJ6FimNZcReZ+9PT0qImJiQXX3r17d2BKwh8uw8WGiRUSeWIJYNWYn+SJJYBR03uAvO21F9QLX\/0URrQkKqecPHnyZEnupaa3wR6QmuIOLsYKHsucPMkTSwCrxvzE8NR7gNCAYHiWQoUGpPZhYIWEZU6e5IklgFVjfmbnaa6AefvUN9VzX\/9ydtESKSzaHhCXGHAIxoWSexlWSO6sXEqSpwsl9zLk6c7KpSR5ulBKLmOugDn9wSF19Dt\/kV20RAo0IAnBoAHBZiorJPLEEsCqMT\/JE0sgu5qe\/yFKS77eo6amprKLlkiBBoQGpGbpyAoei5o8yRNLAKvG\/MzOU2\/Bvn7lEvW9Xb9KA5Idaf0osAcEGytWSOSJJYBVY36SJ5ZANjVz+e3t3Req6zrbaECyIa2vs2lAsPFiBU+eWAJYNeYneWIJpFeTyaey+6n8f3njaerwFy5VVWyPOATDIZj03xLPM1nBewKzFCdP8sQSwKoxP9PzNCef9m5Yofo3nEcDkh5nfZ5ZRcdZZCRYIWHpkyd5Yglg1Zif6XiaS2+l92Pf5y4OekGq2B6xB4Q9IOm+JSnOYoWUAlrCKeRJnlgCWDXmpz9PMR9Xjz2mHnzyeHCyDL2I+ZAPDYg\/z7o+o4oBLzIgrJCw9MmTPLEEsGrMT3+e5sRTPfdDq1SxPWIPCHtA\/L8lKc9ghZQSXMxp5EmeWAJYNeanH8+w+dBDLzQgfhwrU7qKjrPI4LBCwtInT\/LEEsCqMT\/dedrMB4dg3FlWpiQNCDaUrJDIE0sAq8b8JE8sAbuazPn4hx\/+WHXe+d2gsAy79G04T3Vfcs6Ck6vYHnEIhkMw9m8JqAQreBDIn8qQJ3liCWDVmJ\/JPMV8DN77lJLeD5v5YA8INjfrQq2KjrNI8KyQsPTJkzyxBLBqzM94nrLPx9V7Hgs2GtPmIzznI3x2Fdsj9oCwBwRb6ySosULCoiZP8sQSwKoxPxfyDPd6aPMhy21tHxoQG6GKHa9iwIsMESskLH3yJE8sAawa8\/MtnmI87v3+C6rv7sm5f5T5Hrd86v3qF9\/f6AS+iu0Re0DYA+KU\/IhCrJAQFN\/SIE\/yxBLAqjE\/VTDE8uSx19SvDk\/Mg2vucOpKnQbElVRFylUx4EWGhhUSlj55kieWAFZtMeenGI8\/\/94L6vNff6vHQw+3xK1ysdGvYnvEHhD2gNjyHnZ8MVdIMIiGEHliqZIneWYhIKZD\/hu696m5rdS1nvR43NZ1oVp\/\/pLUl6ABSY2uPk+sYsCLjAQreCx98iRPLAGs2mLJzzd7O46pz3\/9iQUAEcZDi1axPar7HpATJ06o\/v5+tX\/\/\/iBOW7ZsUX19fbHfpL1796rt27cHxzs6OtTAwIBqaGiILF\/FgGOrGD+1xVIh+VFJX5o807OLOpM8ydOVgJiO+\/5hVv23A0\/PLaU1ezs+cv5S9btXrJh7kZyrblK5KrZHdW9ABgcHg5iJ6ZidnVU9PT2qq6tLdXZ2LojlwYMHlZQfGRkJTIcYl6ampljDUsWAI74IaTVYwaclF30eeZInlgBWrWr5Kabjf\/3tc+p\/H3p+gekQcsjejqhIVLE9qnsDEg6UaUjCx6T3Y3x8fK7XI\/z3cPkqBhxbxfipVa1C8nt6fGnyxDIlT\/IME5ANw8RwiPGI+ojp+MTqs9VnPnwutLeDBgSbizVR0z0g0hvS2trq1APS1tYW2VsiJ9OAYMPGCp48sQSwasxP8pRejhde\/Wf1xf1PLphIqumI6ej4+bPUZ9c35246zIhUsT2qTA+I9HwMDw9b53VMTk6qzZs3q5mZGbV79+5Io6KDLgE3PwcOHMB+QxeZ2vT0tGpubl5kT53f45Inli15Lk6e33nmhPrDh\/9RHXr2zW3Roz5NZ5yq\/mv7mUr+L\/\/V4tPe3r7gMlNTU7W4dM2uUXMDonspJibmb8xie+LVq1ere+65x1YsmOMh5iJqcqkMuezZsyeYA9LY2BiUlU\/cpNUqOk4rwBwL8BcmFi55kieWAFatjPkpPRwT06+oOx+Yju3hEArSy\/HhlUtU9yXvDv4s\/xX9qWJ7VJgBiRsmiQqynjzqYkCkh6O3t1cNDQ2plpaWOTm9WsYccokrq0+qYsCL\/BKVsUIqkkfWa5NnVoLzzyfPavHUL3rb9c0j6qljJ6yGY\/nS09Rt3RcGEMpgOMLRqGJ7VDkDYq50kV4O\/aEBwVYuadRYwaehFn8OeZInlgBWrdb5KYbjoanj6o\/+9rlEs6ENhhiO3g3nlaaHw0afBsRGqIDj5jCKNhlxS2ujhmDihmvkUaoY8AJCNHfJWldIRT5rLa5NnljK5Fk\/PGV1yuxrP1F\/aBlK0U+kh1RkG\/Sy9nDY6FexPSqsB0TmgNg2DbMFRI6HNyIzNxfTx7q7u+cmm+rJqnIuNyJzIYwrwwoex1KUyJM8sQSwalnzU3o0xDiI2ZAXuv3JIz+09myYvRvbrlih3nbKKZm2P8cSyaZGA5KN37yzTSMgPRajo6Pz5mwAL5VaqooBTw0DcGLWCglwC5WSIE9sOMmzOJ56vob8f+zh59Qzs697mY3uD71bvWfpm5NFyzh\/A0G2iu1RzXtAwoEIr4pB9Ioggi0aVQw4ik0aHVbwaajFn0Oe5IklgFVLyk8xGl8bf1Y98vTLTkbD7Nn4jQ+fq84+\/Wcr07PhSr2K7VHhBsSEb+7RUYZekSoG3DXZ8yjHBhNLlTzJE0sAq\/Y3E5PqPe95j7rn8I\/UgcdedDYa2mzIMtj15y8NejayvEUW+1TFqVWxPSqVATFDG7eapZbhr2LAa8kvfC02mFj65EmeWALp1GSOhny+fvhH6vEf\/tjbaMhqlF9b93PqfWe+g0YjIQRVbI9KZUDMHhCJg22n0nRfF\/ezqhhw96fHl2SDiWVKnuSJJRCvpudo\/OD5H6t93z2mjr6YvK9GWCmYm7H0NHXpyiVK3hRb5bkaecWkiu1R4QYkaRVLXoF01a1iwF2fPY9ybDCxVMmTPFEEzBUnYg7++4Gn1RM\/es2rN0MPnYjR+PVfOEed9pPj6kMX1e+yVxRblE4V26PCDIi5CqYMvR1RSVLFgKO+DGl02GCmoRZ\/DnmSZxoCMmQiJuP2bz2jHnvuVXX0pdcjXy8fp61XmcgcjYvfc4a64qJlQdHw6hPmZ5roxJ9Txfao5gbEXPVi24cDGz5\/tSoG3J8C7gxWSDiWokSe5BlFIO2S1rCWHjb56KqlqvW8JcFhn8mgzE9sflaxPaq5AXniiSfUNddco2644YbEN9GaofN5Fwwy5FUMOJKPrxYrJF9iyeXJc3HyNIdLZE7Go8+87LxvRpzJ+OC571K\/9IEz53oyEHtpMD+x+VnF9qjmBkT3gOT1MjpkyKsYcCQfXy1WSL7EaECwxOqDp2kw5I4PHX05WMbqO1Sin1b3ZKw4s0H9+rpz5oZKECYjiSi\/79jsrWJ7VJgBka3YfT6rV69WLm\/D9dG0la1iwG3PnOdxVkhYuuRZnzzDBuOUU5Ta\/R333T+jnlqbjHXvPUNtbjs3mNNR9EoT5ic2P6vYHtXcgGBDkq9aFQOeL7H6+IVZJAPktVnBI2ni5tToxl\/+L\/81vvNUdce3ngn+\/OCTb+6Z4fsxJ36u+rl3ql9Zc\/acRN49Gb73qsszP9OSiz6viu0RDUhCjlQx4NivhJ8aKyQ\/XrbS5Gkj5HfclWfYYJxx2s+o4QemU8\/D0HepezFWnv0O9Tvt7y1FL4YfwfmlXXlmucZiOreK7RENCA1Izb7DrJCwqMkzH57hIZKXX39DffX+Z4KLpe3BkHO1wXhP42lKXgtfhmESLEEakDx50oDkSbeE2lUMeJGY2WBi6ZNnNp56C\/G\/\/odZ9XdH\/lFNHXtVzbz8RmrRxWYwbKCYnzZCfser2B6xB4Q9IH7fggylWSFlgBdxKnlG8zT3wfinN\/5F3f3oD4PhkbSrSMJDJMuXNajeK1ZUvgcja7YyP7MSnH8+DQiWZ+nVqhjwIqGzQsLSX6w8TYPx5987pr73rP9unuFISO\/FG2+8od531ulK9sTY8tFmGoyM6bpY8zMjttjTq9gelaIHZO\/evWr79u0BeHkB3dNPP63Gx8fVwMCAamhoyCueVt0qBtz60DkWYIWEhVtFnnrliBiCP\/ybaXX46CsBtCxzL+R8cy+MD557utpw0ZkLDEYVeWIzzk+NPP142UpXsT0q3IDIO2FmZmZUb2+vuvrqq5VsUCZ7fvT396umpqbg70V9qhjwoljKdVkhYenXI09tMB544iX10E+XpCLMRWAylp6mrrjoTLXmPe+a914S12Wq9cgTm1FYNfLE8qxie1SoATF3RV21apXq6ekJDEdra6vS26+PjIyoxsbG2EiG36a7ZcuWRNMiups2bQr0xOgk6Vcx4NivhJ8aKyQ\/XrbSZeBpLknV93vXQzPq4SP\/COu50ObioqbT1S9\/8KxAV95Joq9t4+R6vAw8Xe+1HsqRJzZKVWyP6t6ASA+KfMS4aEPT1dWlOjs7F0R\/cnJSbd68We3cuTMwOTL0kzTUU8WAY78SfmqskPx42UrXgqe5JPXcJW9XX\/22vEH1x3BzIUtTuy55tzrFeKuqa8+FjZPr8VrwdL2XKpQjT2wUq9geFWpAJDzaBJhDMLo3JM5IJIXVNCThcnKtI0eOOA\/rVDHg2K+EnxorJD9ettJZeerJnHKd706\/oh544niq17NH3ac2DzIsIhtrXbn6bHXq206p2XtIbOyijmflmeaaVT6HPLHRrWJ7VLgBkRCZwyI6ZDt27IjsxUgKadKL7vRQTVtbm7NuFQOO\/Ur4qbFC8uNlK23jKftciBE48IMX1T2P\/iiQy7oUVTRMc6GHRczeilr3XNg4uR638XTVYbk3CZAnNhOq2B6VwoAgwiQ9H8PDw6qjoyNy9Yw2IBs2bFB33nmnkpfhcQ4Igry7Biskd1ZJJbWx+PZ3p9ThF05Vj\/\/wxxBjETYX7RcuU+uWn5FqQifmSWurwvzE8iZPLE8aECzPXNT0qprwEl5tQI4ePTo38TSurL4xCbj5OXDgQC73vFhEp6enVXNz82J53FTPKTtxPvfKG+o7z5xQj878U6Bx6NnXU2mZJzWdcWrw13e\/61Qlf\/6Vf\/Mu9ca\/nAz+bd25p2XWr4IA8xMbRfLMxrO9vX2BwNTUVDbRkp1daA+IHjKR3oikj21li3muTDSV+SRDQ0OqpaVl7lDUEExcWdOAVC3gRebfYvxFFH6vyPHXfqL+4v++ANmZU8fSHBK5ePkZ6v3nvFOtX7lkLtT1OiRS61xdjPmZJ2PyxNJlDwiWZ6AmE0P37NkzbzmsuZpl48aNXnuCJC3flR6PFStWzM0BEQNy8803q127dkUu9a1iwHMIobNk1SokcxLno8+8rP7myePqB8\/lMxwiq0Rk86zGd\/6rgLcsQ60aT+dEyqkgeWLBkieWZxXbo1L0gOi9P8xwmUbi8ccfV2Ie7rnnngURNVe96F6OuA3MwuYkacWMXKiKAcd+JfzU6qVCMrf6fsfPvk3t\/s7z0HkWQs3stfjFC5apX3jvGQHMuX9vtA+L1AtPvywprjR5YtmTJ5ZnFdujujcg4Y3IzEmo+lh3d3ew74d8zBU3cRNWOQSD\/eJotTJUSHonzhM\/+X\/qz\/\/+BfXksdeC28u6G6d+RtNYrHvvGar9gmWBqcjj1etl4JlPphSjSp5Y7uSJ5UkDguUZqNmGYGRDMb1XyFe+8pUc7iBesooBrynA0MXyrJD0q9Wblrxd3f6tZ9TkDzGbZUUZCzEUWy9foWaOvzlJVIZDivjkybOI5yn6muSJjQB5YnlWsT0qtAdEhydqHxB5KZ3erfTWW29Vo6Oj8yaVYkMbrVbFgNeCW9w1fCskc46F\/Pmbj72oHjn6cm49Fu9d1qD+\/QfPUqe\/\/WfqYumpL88iY18P1yZPbJTIE8uziu1RKQwINkw4tSoGHEfHXylcIen9LP7y+y+oP5s4FggiNsoSHXMoZMWZDeqKi5apJQ1vTuD0mWfh\/5S1O4MVPJY1eZInlgBWrYrtEQ1IQo5UMeDYr0S0mjmJ88\/+\/pj6\/syruRiLVT\/3TvWx9y9VP3\/uu3KZY1ELVlmuwQYzC72F55IneWIJYNWq2B4VbkD0C+JmZmYWRMu2Uyk2vAvVqhhwBDNtMB6YfEmNTx0P9rTIOonT7LGQJaefWndO8O4Q+RQ1xwLBKk8NNphYuuRJnlgCWLUqtkeFGhBzczC934esWNEvo4tanosNabJaFQPuws\/swfj25Evq4NTxTD0Y2lyc1aBU26qz1QXnvFN9mBtluYQisQwbzMwI5wmQJ3liCWDVqtgeFWpAwi+PMzcKk4mpY2Njke91wYY1Xq2KATef1tyl86++\/4I6\/Mwr3j0ZZs+FDImc9HsoAAAgAElEQVR8Ys3ZsRM4WcFjM5c8yRNLAKvG\/MTyrGJ7VCoDIsttjxw5oqTnI2lHU2xYF5cBkYmf+797LHjtuuuwiTYZHzl\/qer8hXNST+JkhYTNXPIkTywBrBrzE8uTBgTLM1AzdyM1Tcc3vvENNT4+zh6QjMyll0Mmgv7l916wGg4xGsuXnqb+Q2uTavrXbw+MBvI9IqyQMgYzdDp5kieWAFaN+YnlSQOC5RmohV8SJ4ZkeHhYyXbqRez9YT5ivQZcejnGHn5OjT38fGzExFjIPIzuS94NNxpxF2WFhP0CkSd5Yglg1ZifWJ712h4lUSh0CAYbHrxavQX8zgenVd\/dk5EgxHBs\/Pmz1BUXnVnYqhJWSNgcJU\/yxBLAqjE\/sTzrrT1yefpCDUh4Eqp5w5wD4hI+paS3Y+jepyKHV8R03NZ1Yc16OGx3zArJRsjvOHn68bKVJk8bIb\/j5OnHy1aaBsRGyPM4DYgnMKO4zO24euyxBcZDTMe+z10clETO30h\/p2+dyQoJQZE8sRTJkzzzIoDVpQEB8ZTVLtu3b7eqbdmyJVgRU9SnjAEX4zF471ML5nesX7lE3db9Zm9HWT80INjIkCd5Yglg1ZifWJ5lbI+yPmFph2CyPhji\/LIFXIZbNt7x6LxH2\/Shd6vbui5APG7uGqyQsIjJkzyxBLBqzE8sz7K1R4inK9SAIB4gT42yBDxquEUPtZS5xyMcG1ZI2GwlT\/LEEsCqMT+xPMvSHiGfigYkgWYZAi7mQ3o99PboemJpPb4fhRUS8qurFHmSJ5YAVo35ieVZhvYI+0RK1dyA6ImnExMT1mdZ7C+jk308rhp7bI5TPfZ6mEFmhWRNea8C5OmFy1qYPK2IvAqQpxcua2EaECuiahUoOuCNW++bA3r3b61RH1u1tK4Bs0LCho88yRNLAKvG\/MTyLLo9wj7Nm2o17wHJ4yHy0iwq4GbPh\/R69G04T3Vfck5ej1kzXVZIWNTkSZ5YAlg15ieWZ1HtEfYp5quVwoBELcvdsWOH6uzstD673sp9\/\/79QVnXpbuTk5Oqt7dXDQ0NqZaWlsjrFBHwqpoPAcwKyZrOXgXI0wuXtTB5WhF5FSBPL1zWwkW0R9abyligcAMi5mPPnj1qZGRENTY2Bo+j54l0dXVZTYj5MjvX87RpOXToUOL7Zmod8CqbDxqQjN\/UiNNZwWOZkid5Yglg1WrdHmHvPlqtUAOSx06opiGJA6i3eZfjZekBkVUua256aO6WZTfTelzpkpS0rOCxX2nyJE8sAawa8xPLkwYEy3Oup0N2O21tbZ2nnuZdMEmGRotLmRtvvFF1d3crMStlMCDhpbaHv3BpqXc0TZsGrJDSkos+jzzJE0sAq8b8xPKkAcHyDNSyDsHoWxIzMTw8rDo6OtTAwIBqaGiIvFu5nnzWrl3rNAfEFDlw4EAOBJT64jdfUPsfezXQ\/sRF71LXty\/L5TpFi05PT6vm5uaib6My1ydPbCjJkzyxBLKptbe3LxCYmprKJlqyswsdgtEsskxCDfMUIzIzMxNpQmTi6V133aWuu+46JZVNGSahhud9SO9HVT\/8RYSNLHmSJ5YAVo35ieXJHhAsz1zUkla3iDm57LLLguGeMqyCMed91PMOp66BZIXkSsqtHHm6cXItRZ6upNzKkacbJ9dSNCCupBzLua5acZQLisXNHUnagXX37t0L5qCIVt4B\/8TvH1bfnnwpuO\/buy+sxF4fSbFiheSTyfay5Gln5FOCPH1o2cuSp52RT4m82yOfe0GVLXwIJjz8EmcG4h7YXPWil9c2NTUpmdia9Cm6ByTc+1HloRcdB1ZIqK\/tmzrkSZ5YAlg15ieWJw0IlucCNT2RVA6IiRgdHY3dJEyfHN6IzJyEqo\/JipfwKpsiDUj47bZVXHIblSqskLBfIPIkTywBrBrzE8uTBgTLM1FNzIgMp5gblNXw8sGl8gq4OfH0Ny5tUrd86v21frRCrscKCYudPMkTSwCrxvzE8syrPcLepZ9a4UMw5u2aPSBFvwk3LwNi7vlR72+39Us1Dhn48rKVZwVvI+R3nDz9eNlKk6eNkN9xGhA\/Xk6l0wy7OAkDCuUR8AefOK423vFocHfygjmZfLpYPqyQsJEmT\/LEEsCqMT+xPPNoj7B36K9WaA+Iy86l\/o+EOwMd8HDvx2KYeGpGgxUSLjdFiTzJE0sAq8b8xPJEt0fYu0unVqgBSXfLtTsLHXBz7sdi6\/1gg4nPW1bwWKbkSZ5YAlg1dHuEvbt0ajQgCdyQATdXviy2uR8aMSv4dF\/SuLPIkzyxBLBqzE8sT2R7hL2z9Go0IDUyIObcj2t+cbn64i+vTB+1Oj2TFRI2cORJnlgCWDXmJ5YnDQiWZ+nVkAHfePuj6sEnjwfPvFj2\/QgHmBUSNuXJkzyxBLBqzE8sT2R7hL2z9GqF9oAkTUKN21I9\/aP6n4kKuLnr6frzl6p9n1vjfzMVOIMVEjaI5EmeWAJYNeYnlieqPcLeVTY1GpAaDMH8z799Tv2XvT9Y1L0f8vCskLJ9WdmjhOVHnuSZLwGsOg0IiGf4\/S9xslu2bLG+0wV0S5EyiIAvxne+xMWEBgSbreRJnlgCWDXmJ5Ynoj3C3lF2tdL2gGR\/tOwKiICbBqRvw3mqb8OK7DdWpwqskLCBI0\/yxBLAqjE\/sTwR7RH2jrKrFWpAst9+vgqIgHPy6VsxYoWEzVfyJE8sAawa8xPLE9EeYe8ouxoNSAJDRMAbt94XXGH9yiVq31UXZ49YHSuwQsIGjzzJE0sAq8b8xPJEtEfYO8quVnMDYq58WbVqlerp6VETExORT1L0C+myBtzc+VTe+SK7ny7mDyskbPTJkzyxBLBqzE8sz6ztEfZuMGo1NyCY266NStaAc\/hlfpxYIWHzljzJE0sAq8b8xPLM2h5h7wajRgOS0xDMvL0\/OPwSUGaFhPnSahXyJE8sAawa8xPLkwYEy1Pp4ZgqDsFw9cvCZGGFhP0CkSd5Yglg1ZifWJ40IFiesWpiTK699lr1+c9\/XrW0tNToqgsvkyXg5vDL4S9cquQFdIv9wwoJmwHkSZ5YAlg15ieWZ5b2CHsnOLXSDsHIVuxjY2NqYGBANTQ0xD7xiRMnVH9\/v9q\/f39QJmnzsnCPS0dHR6J+2oBz87HocLFCwn1xOaSFZUme5IkngFVM2x5h7wKrVmoDMjg4qEZGRlRjY2PsU0sZ+fT19c0N6XR1danOzs5552ij0tbWFhzTf29qaordbTVtwDn8QgOC\/ZqSJ3nWggD2GvzBgeWZtj3C3gVWrbQGRIzFzMyMtQckjMM0JDZUsiX8+Ph47DXSBnzw3iNq8N6ngstz+OWtKLBCsmWk33Hy9ONlK02eNkJ+x8nTj5etdNr2yKZb5PFCDUjSJFTpmRgdHfWaA5L0dt0oyHkZED3\/Q+Z9iAHh500CrJCwmUCe5IklgFVjfmJ50oBgeQZqcUMjeqjE9ZLS8zE8PKxs8zq0njYrUcM1ukyagHP5bXzEWCG5ZrNbOfJ04+RaijxdSbmVI083Tq6l0rRHrtpFlSu0B0QeOmqoxcUcxAFzGbrRpkc0kia5SsDNz4EDB6xxOvTs6+o\/3\/18UG7\/p5tV0xmnWs9ZLAWmp6dVc3PzYnnc3J+TPLGIyZM8sQSyqbW3ty8QmJqayiZasrMLNSBJQyauq2DCPCcnJ1Vvb68aGhqKHL5xNR+im8Zx\/upXJ9R9j88Gt7Xvcxer9ecvKVnIi7sd\/iLCsidP8sQSwKoxP7E807RH2DvAq5XagLisggkjEeMSd57LyhdTL03A19z0kJJhGM7\/WJisrJCwX2DyJE8sAawa8xPLM017hL0DvFqhBiQ8\/8N8PNsEUV3WXPViMxguwzNZDIg5\/+N3r1ihtv\/b8\/ARq2NFVkjY4JEneWIJYNWYn1ieNCBYnoGa9Fhs27Zt3ooXGUbZvHmz2rlzp2ptbU28angjMnMSqj7W3d2t4t68m\/TGXd+AP\/jEcbXxjkeD++XwC3tAcvi6zJNkBY8lTJ7kiSWAVfNtj7BXz0et0B4Q\/UhiQjZt2jTvCXfv3m01H\/kgeUvVN+Dcfj05IqzgsRlLnuSJJYBVY35iefq2R9ir56NWCgOSz6NlV\/UNOOd\/0IBkzzp3BVbw7qxcSpKnCyX3MuTpzsqlpG975KJZdJlCDYg5RGIbaikClE\/Auf+HPUKskOyMfEqQpw8te1nytDPyKUGePrTsZX3aI7taOUoUakB8dy6tNTKfgHP+hz06rJDsjHxKkKcPLXtZ8rQz8ilBnj607GV92iO7WjlKFGpABIHrapcicPkE3Hz\/CyegRkeLFRI2i8mTPLEEsGrMTyxPn\/YIe+X81Ao1IEnvgpFHTlqhkh+St5R9As73v9gjwgrJzsinBHn60LKXJU87I58S5OlDy17Wpz2yq5WjRKEGpBwI4u\/CJ+CNW+8LhNavXKL2XXVx2R+tkPtjhYTFTp7kiSWAVWN+Ynn6tEfYK+enRgOSwNY14Jz\/4ZagrJDcOLmWIk9XUm7lyNONk2sp8nQl5VbOtT1yUytHqZobEHPiadzmYBpNvQzBcP6HWzKzQnLj5FqKPF1JuZUjTzdOrqXI05WUWzkaEDdOlSnlGnBzA7LZWz5emedHPwgrJCxR8iRPLAGsGvMTy9O1PcJeNV+1mveAhB8n\/D6YpPfD5ItiobprwDkB1S0yrJDcOLmWIk9XUm7lyNONk2sp8nQl5VbOtT1yUytHqcINSNQL4vQwTVdXl+rs7CyMlEvAuQGZe3hYIbmzcilJni6U3MuQpzsrl5Lk6ULJvYxLe+SuVo6ShRqQpI3I5P0wY2NjamBgQDU0NBRCyyXg5gTUvg3nqb4NKwq513q4KCskbJTIkzyxBLBqzE8sT5f2CHvF\/NVKbUCkd2RkZEQ1NjbmTyLiCi4BH3v4eXXV2GPB2Ye\/cKla3nhaIfdaDxdlhYSNEnmSJ5YAVo35ieXp0h5hr5i\/WqEGJGm+Rxl2SHUJON+A656krJDcWbmUJE8XSu5lyNOdlUtJ8nSh5F7GpT1yVytHyUINiCCQoZZt27ap0dFR1dLSElCZnJxUmzdvVjt37lRFvqTOJeCcgOqeyKyQ3Fm5lCRPF0ruZcjTnZVLSfJ0oeRexqU9clcrR8nCDYg2IZs2bZpHZPfu3YWaD7kZW8A5AdUviVkh+fGylSZPGyG\/4+Tpx8tWmjxthPyO29ojP7VylC6FASkHioV34RJwvQU7J6Dao8gKyc7IpwR5+tCylyVPOyOfEuTpQ8te1qU9squUqwQNSEI8bAHnFux+ycwKyY+XrTR52gj5HSdPP1620uRpI+R33NYe+amVozQNSAYDwi3Y\/ZKYFZIfL1tp8rQR8jtOnn68bKXJ00bI7zgNiB+v0pXWq272798f3NuWLVtUX19f7H3aAs4JqH4hZoXkx8tWmjxthPyOk6cfL1tp8rQR8jtua4\/81MpRelH1gMi+IvIR0+Gy26ot4GtuekjJRFTZ+0P2AOEnmQArJGyGkCd5Yglg1ZifWJ629gh7tdqoLSoDEkZqGpIo3EkB5woY\/wRlheTPLOkM8iRPLAGsGvMTy5MGBMszUNN7fszMzCxQX716dW47oSZtA69vJCng3ILdPxlYIfkzowHBMiNP8qwdAeyVaECwPJWek9HU1JQ4FwN8WSU9H8PDw6qjoyPxXTMScPNz4MCBub\/uf+xV9cVvvhD8\/Q8+eY5ady63YLfFaXp6WjU3N9uK8bgjAfJ0BOVYjDwdQTkWI09HUDHF2tvbFxyZmprKJlqyswsdgnHphciTV9SbeM3rJTlOcwUM3wHjFiX2gLhxci1Fnq6k3MqRpxsn11Lk6UrKrRx7QNw4OZfSPSDd3d2F7Hoqwz+9vb1qaGhobht4VwPCFTDOYZ4ryArJn1nSGeRJnlgCWDXmJ5YnDQiWZ6Am74Ip6q23tmsnBZwrYPyTgRWSPzMaECwz8iTP2hHAXokGBMtzbinsxMREpDJ6Eqq56sVl\/klcwLkCJl0i0ICk4xZ3FnmSJ5YAVo35ieVJA4LlWXO18EZkLpNQoyb9mAaE74BxDyMrJHdWLiXJ04WSexnydGflUpI8XSi5l6EBcWdViZJxAec7YNKFlxVSOm7sAcFyI0\/yrA0B7FVoQLA859T27t2rtm\/fHvx99+7d6umnn1bj4+OJS2RzupV5snEB5ztg0tGnAUnHjQ0mlht5kmdtCGCvQgOC5Rmo6aWwshrl6quvDvYDkbkf\/f39qtb7g4QfLy7gV409psYefj4oPnvLx3OgUk1JGhBsXMmTPLEEsGrMTyxPGhAsz7lJqGI6Vq1apXp6egID0traWujqGP2YcQHnEtx0icAKKR03\/mLHciNP8qwNAexVaECwPOvWgDRuvS8gsX7lErXvqovBVKorRwOCjS15kieWAFaN+YnlSQOC5RmoyfwPme9hDsHo3pCuri7V2dmZw1XdJKMCbq6A6b7kHHV794VuYiylWCFhk4A8yRNLAKvG\/MTypAHB8pxTkw3BNm3aNE99x44dhZoPuZmogHMFTPokYIWUnl3UmeRJnlgCWDXmJ5YnDQiWZ+nVaECwIWKFRJ5YAlg15id5Yglg1WhAsDxLrxYVcC7BTR82VvDp2bEHBMuOPMkzfwLYK9CAYHnOqZn7gOh\/lP1AZDVMkZ+ogG\/748fV18afDW6LS3D9okMD4sfLVpo8bYT8jpOnHy9bafK0EfI7TgPix8uptJiPPXv2qJGREdXY2Phmwz47GyzJLeMkVC7BdQprZCFWSOnZ8Rc7lh15kmf+BLBXoAHB8py3DDfc22F7Uy34ViLlogKu34LLJbj+EaAB8WeWdAZ5kieWAFaN+YnlSQOC5Vl3BoRvwc2WAKyQsvELn02e5IklgFVjfmJ50oBgeQZq0tOxbds2NTo6qlpaWko9BMO34GZLAFZI2fjRgGD5kSd55ksAq04DguU51wMyMTFhVZb3w9xzzz3WcsgC4YCbe4DIBmSyERk\/7gRoQNxZuZQkTxdK7mXI052VS0nydKHkXoYGxJ1VJUomGZB9n7tYrT9\/SSWes1YPwQoJS5o8yRNLAKvG\/MTypAHB8iy9Wjjg5h4gh79wqVreeFrpn6FMN8gKCRsN8iRPLAGsGvMTy5MGBMtzTi1qH5AybsWul+DKjXMPEP9kYIXkzyzpDPIkTywBrBrzE8uTBgTLM1Crp31AuAdItgRghZSNX\/hs8iRPLAGsGvMTy5MGBMsTsgxXb1qmJ7J2dHSogYEB1dDQEHm3Zm+LrWw44NwDJFsCsELKxo8GBMuPPMkzXwJYdRoQLM\/MBuTEiROqv79ftbW1BW\/O1X9vampSfX19C+7W3NxMDIqcG1dWTjYDzj1AsgefBiQ7Q1OBPMkTSwCrxvzE8qQBwfLMZQhGejjGx8cje0HCx5LKJhmQvg3nqb4NK3KgUW1JVkjY+JIneWIJYNWYn1ieNCBYnnNqyEmoSaYiqgdE955EPZoZcHMPEC7BTZcIrJDScYs7izzJE0sAq8b8xPKkAcHyhKu5vMRucnJSbd68Wc3MzCjbG3fNgI89\/Ly6auyx4J5pQNKFjhVSOm40IFhu5EmetSGAvQoNCJYnVE3P\/xDRuEmo4RU3g4ODwT1EzReRf5eA68\/rF1ypXr9gY\/DXP\/jkOWrdudwDxDeA09PTqrm52fc0lo8hQJ7Y1CBP8sQSyKbW3t6+QGBqaiqbaMnOPuXkyZMnS3ZP3rfjYj7CE1blItIb0tvbq4aGhubeQ2Ne3HSc0vshvSDy4R4g3iEKTmAPSDpucWeRJ3liCWDVmJ9YnuwBwfKEqNlWvuiLZDUg3AMke7hYIWVnaCqQJ3liCWDVmJ9YnjQgWJ4QNRlGkfkcSXt\/6AtFDcEknWsGXO8BItuvyzbs\/PgTYIXkzyzpDPIkTywBrBrzE8uTBgTLM7NaeBMyLShvzh0ZGQk2I5O9Prq7u1Vra2twWAzL8PBw8Gefjcgat94XnLN+5RK176qLM9\/7YhRghYSNOnmSJ5YAVo35ieVJA4LlWXo1M+DagHRfco66vfvC0t97GW+QFRI2KuRJnlgCWDXmJ5YnDQiWZ+nVdMDNPUC4CVn6sLFCSs8u6kzyJE8sAawa8xPLkwYEy7P0alEGRHo\/pBeEH38CrJD8mSWdQZ7kiSWAVWN+YnnSgGB5ll5NB3zw3iNq8N6ngvvlJmTpw8YKKT079oBg2ZEneeZPAHsFGhAsz9Kr6YB\/8wez6tf\/YCK4X1kBIyth+PEnQAPiz4w9IFhm5EmetSOAvRINCJZn6dV0wLkJGSZUNCAYjlqFPMkTSwCrxvzE8qQBwfIsvZoOODchw4SKFRKGIw0IliN5kmc+BLCqNCBYnqVX0wHnJmSYUNGAYDiywcRyJE\/yzIcAVpUGBMuz9Go64NyEDBMqGhAMRzaYWI7kSZ75EMCq0oBgeZZeTQL+rb\/7vpIeEPlwE7JsIaMBycYvfDZ5kieWAFaN+YnlSQOC5Vl6tbAB4SZk2ULGCikbPxoQLD\/yJM98CWDVaUCwPEuvJgH\/H3\/1iNp4x6PBvdKAZAsZDUg2fmwwsfzIkzzzJYBVpwHB8iy9WtiAcBOybCGjAcnGjw0mlh95kme+BLDqNCBYnqVXk4Bv+f2\/5i6ooEjRgIBA\/lSGPMkTSwCrxvzE8qQBwfIsvVrYgHAX1GwhY4WUjR9\/sWP5kSd55ksAq04DguVZejUJ+Aeu\/RP14JPHg3udveXjpb\/nMt8gDQg2OuRJnlgCWDXmJ5YnDQiWZ+nVTAMi73+RHhB+0hNghZSeXdSZ5EmeWAJYNeYnlicNCJZn6dUk4Gd85o\/U0dnXgxfQ0YBkCxkrpGz8OGSA5Uee5JkvAaw6DQiWZ+nVJODHPzES3Of6lUvUvqsuLv09l\/kGaUCw0SFP8sQSwKoxP7E8aUCwPEuvtuIDH1IvXzEY3Cd3Qc0eLlZI2RmaCuRJnlgCWDXmJ5YnDQiWJ0RtdnZW9fT0qImJiUCvo6NDDQwMqIaGhkj9gwcPqk2bNgXHVq9erUZGRlRjY2NkWdOAcBOy7OFihZSdIQ0IliF5kmd+BLDKNCBYnpnVTpw4ofr7+1VbW5vq7OxU+u9NTU2qr69vgf7k5KTavHmz2rlzp2ptbVV79+5V4+PjsYZl+Yd+Sb26vjfQub37wqAXhJ\/0BGhA0rOLOpM8yRNLAKvG\/MTypAHB8sxFLclUyLEjR45EmpOom6EBwYaIFRJ5Yglg1Zif5IklgFWjAcHyzEUtzoCEe0tcLt78sf+oXlv7maAot2F3IZZchhV8doamAnmSJ5YAVo35ieVJA4LlCVfT80G6urqCIRnzow3Ihg0b1J133hnMGbHNAWn6d7+jXr9gYyBz+oND6v69d8DveTEJTk9Pq+bm5sX0yLk+K3li8ZIneWIJZFNrb29fIDA1NZVNtGRnn3Ly5MmTJbunVLejDYacHDUJVR8\/evTo3MTTwcFBNTMzEzsH5Jxfu0n98\/IPB\/fDbdhThWXeSfxFlJ0he0CwDMmTPPMjgFVmDwiWJ0zNZj7kQlFDMDIptbe3Vw0NDamWlpYF93P2f\/qqeuPM9wf\/zm3Ys4eLBiQ7QzaYWIbkSZ75EcAq04BgeULUbCtfzItIj8eKFSvmhmfEgNx8881q165dkUtxtQHhLqiQUCkaEAxHrUKe5IklgFVjfmJ50oBgeULUbMMo5kVkDxApr\/f+kD\/LJ2rJrvz7mb\/1f9S\/vONM7oIKiZSiAQFxpAEBg\/ypHBtMLFfyxPKkAcHyzKwW3oRMC+rJpbIZmewT0t3dHez7IR9zIzLbpmWNW+8LzuE27JlDFQiwQsJwpAHBciRP8syHAFaVBgTLs\/Rq2oBwG3ZMqGhAMBzZYGI5kid55kMAq0oDguVZejVtQLgNOyZUNCAYjmwwsRzJkzzzIYBVpQHB8iy9Gg0INkQ0IOSJJYBVY36SJ5YAVo0GBMuz9GragPA9MJhQsYLHcOQvdixH8iTPfAhgVWlAsDxLr6YNCLdhx4SKBgTDkQ0mliN5kmc+BLCqNCBYnqVXowHBhogGhDyxBLBqzE\/yxBLAqtGAYHmWXk0bEG7DjgkVK3gMR\/5ix3IkT\/LMhwBWlQYEy7P0atqAcBt2TKhoQDAc2WBiOZIneeZDAKtKA4LlWXo1MSDchh0XJhoQHEtRIk\/yxBLAqjE\/sTxpQLA8S69GA4INESsk8sQSwKoxP8kTSwCrRgOC5Vl6NTEg3IYdFyZW8DiW7AHBsiRP8sQTwCrSgGB5ll6NBgQbIhoQ8sQSwKoxP8kTSwCrRgOC5Vl6NTEgfA8MLkys4HEs+Ysdy5I8yRNPAKtIA4LlWXo1MSB8DwwuTDQgOJZsMLEsyZM88QSwijQgWJ6lV6MBwYaIBoQ8sQSwasxP8sQSwKrRgGB5ll5NDAjfA4MLEyt4HEv+YseyJE\/yxBPAKtKAYHmWXm3FBz6kjnzvO6W\/z3q5QRoQbKTIkzyxBLBqzE8sTxoQLM\/Sq1Ux4EVCZ4WEpU+e5IklgFVjfmJ5VrE9OuXkyZMnsZiqo1bFgBcZHVZIWPrkSZ5YAlg15ieWZxXbo7o3ILOzs6qnp0dNTEwE0e7o6FADAwOqoaEhMfqTk5Oqt7dXDQ0NqZaWlsiyVQw49ivhp8YKyY+XrTR52gj5HSdPP1620uRpI+R3vIrtUV0bkBMnTqj+\/n7V1tamOjs7lf57U1OT6uvri42uLnfo0CE1OjpKA+L3PUhduopfoNQwACeSJwCiIUGe5IklgFWrYn7WtQGJCu\/evXvV+Ph4Yi\/IwYMH1eDgYHA6e0CwX5IktSp+gWpHb+GVyBNLnzzJE0sAq1bF\/Fx0BkSGbG688UbV3d0dmBAaEOyXhAaEPGtHAHulKlbwWEJ+auTpx8tWuoo8K2VA9HyQrq6uYEgmrodE\/n3t2rVOc0BsScHjJEACJEACJMOjwVAAAAkZSURBVFALAlNTU7W4TM2uURkDoud1CLm4Sagy8fSuu+5S1113nZqenrYakJpFgRciARIgARIggUVGoBIGxMV8SFxlyOWyyy5Tra2tymUVzCLLBT4uCZAACZAACdSMQN0bENeVL+Hluibh3bt3B6aEHxIgARIgARIggdoQqHsDIr0aMzMzTnt\/mEjZA1KbBONVSIAESIAESCCKQF0bkLhejdWrV6uRkZFgMzLZJ0RWvIR7OGhA+IUgARIgARIggeII1LUBKQ4br0wCJEACJEACJJCFAA1IFno8lwRIgARIgARIIBUBGpAIbDKvZHh4ODjCCarueSXDWps3bw7m5NjeyWMylq3zk7bEd7+DapX04amfPPx6gmoRyf40PkzDQ7ysCxby9+FpluV33j+X9Xc7akqBv1o5zqABCcVBb9Muc0gef\/zxYOmu\/LmxsbEcESvpXZgN38aNG+e9oyd8y+Ht8uXve\/bsIWcDlA9Pk6+w3L59u9qxY0fsZnwlTaHcb8uHaXh1HeeMLQyPD09t5uQdXTIfj995v3TXrPfv31+pH8U0IKE80O+IkS9KFR2nX9q7lw5X0GLkxsbGnFYnsXKP\/mVpvq3ZhadU8tdee606fvy4StoN2D2q1Srpk6NS9uabb1a7du3ij4+YNPDlaeYzv\/Pu3y3dc7Ru3Tp19OjR4EWrVdk2ggYk5len+XZd\/bZd95RZfCXNniPpLQr\/PYkIK6OFdNLwFPN8ySWXqD\/90z+de0P04svE+Cf2YeryUsvFztaHZ1QPiO2loYudr37+Z599NvijrOrs6emhAalqYkT1eEilvmLFCnZnW4Ie\/oXu8wsy7V4uVc1DeS5fnvo1A1u3bg1etkjTHG3qzF65pBwVA3LkyJFAhPPBor9pvjlqDiNs2bIlaEj5cScQNnHuZ5a3JHtAInpAzEk+NCBuyetbGWlVqehvvfVWTkINYfbhKRX7l7\/8ZfXpT39aNTc3J86\/cYtmNUv5MNVzafTEUzl327ZtzFMjNXx46mGEnTt3BsMHPj2k1cxG\/6eiAfFnVldnhFcQcEWBe\/h8umNpPuxcfXhK2fvvvz\/4RcmcjWfrwzQ8BEOu0T1K5iT9JFNBnvbvvK0EDYiNUAWOmz0enITqHtBwd7Zt0iRnwSez9eFpLmk2VdnNPZ+xD9Nw\/rIuWJivPjxpQNzr0riSNCDZGZZegctw04XIZ0keu7PtjH14mmr8pR7P1odpuLLnkMFCrj48o4ZgOKRlrwfMEjQgfrzqtjQ3IksXuqRNifSkPhkmiPvFzo2eFv5ij9vYzeRJA+Ker645KormRmTcOCuasQ9PMXGbNm0KhMjTPWd1SRoQf2Y8gwRIgARIgARIgAQWEOAqGCYFCZAACZAACZBAzQnQgNQcOS9IAiRAAiRAAiRAA8IcIAESIAESIAESqDkBGpCaI+cFSYAESIAESIAEaECYAyRAAiRAAiRAAjUnQANSc+S8IAmQAAmQAAmQAA0Ic4AECiTw5JNPqqVLlzq\/8l32AnjppZfUypUrc7trvU\/L6tWr1cjIiPO91ctbjfXz1WovilpfL7fEoDAJgAnQgICBUo4EXAn47q5Zi42IspiILOe6MkOUE0Mgn1q+jbVe2CD4UoMEXAnQgLiSYjkSABMoowHxvScTSb00sjQg4ESmHAmkJEADkhIcTyMBFwLmdt5SXg9rPP7443PbUsu\/623ow9vU62GCZcuWqZ6eHjUxMRFcVr9oTr+PY\/\/+\/cG\/uwybmNcwhyH0K+j1c+3YsUN1dnYueMy4ctqAbNy4UX3pS18KzgsPc5jbcYevI6yuvfZa9dGPfjQ4Xz\/Liy++qPSW9HLO9ddfr\/bt26eGhoZUS0tLIBP3TFExChsQ+fsrr7wS\/Kc5Jr3IL\/xiNblG1L\/VozlzyWmWIQEUARoQFEnqkECIQNSL4czGL9zbEPfGUJEdGBhQoicmRIYOWltb595V0tXVNWcUkt4yrO9H6zU0NAQN56233qpGR0eDxtzWAxIub75kTEySGIV169YF96v19+zZE8wlESPR29s7zziYetpkLV++fO58beD0M+q\/Hzt2LLjn5uZm1d\/fHxgdPaRie9lhlAEZHh5W2nCFrxlObBoQftVJAEOABgTDkSoksICArSGzNfbhX9ZhAxJ+ZbyUT3obbtQQSbh80j3Z3rQbfuOp3I9tWMY8rg1I2FCNj4\/PGRLRNA2G\/P3mm29Wu3btmjdZNmmYJcqAzMzMLLiGlIuahEsDwi87CWAI0IBgOFKFBCIJmMMV4eGRuMY+PEzR0dER2QMSHgoxbyBq+CTueuabdZMMiG0SbJTZiPq38LBUeJhJ9\/DI80QZCVNTelX0G1bDAYgbRokyIHKuOSk1yTjRgPDLTgIYAjQgGI5UIYFEAmaja84DMX9la0MRnpehewDCPSBybviXe9JNxJmLpGEhUy+rAREtPZdDG6SoHhAfA\/LII48oPcTT2NjolIU0IE6YWIgEcidAA5I7Yl6ABN4iYDbi+he+dPPLfAmZy9DW1jZv4qdpMsIGJGm+RxTzWgzBhOd4mNcUs5A0nKKHYEwDEtXbYA7BSA\/Itm3b5uawuORaHkMwNjNoG4pyuW+WIYGqEaABqVpE+TylIRA1B8TshTAnZUZNptQ9InoIRh7MNClaXyak6uGDqHkYGghqEqrZ42DOC1m7du2CSaZhA2Keq+9V7k8mlEYZENdJqKKhJ77a5t5knYSqh8j0yiX9HObk23AS0oCU5mvJGykRARqQEgWDt1I9ArpxkqES+ZjDK+YSWhmSuPzyy+cttRXjceWVV6obbrhh7hd+2JToXhG9PFeuoRvGOJpJS1ZdJ8Zu3759Tj5qOEUvjw03vOFr79y5M5jnIRNP9fObPSBykTDD8DLc8FJkOSduCbHudZL\/a9MWtQzXPD\/KPJjzbyROa9asUYcPHw5MUNgo6mcI9w5VL9v5RCTgR4AGxI8XS5MACRRMQAxB1MoX19tymQPiquVajj0grqRYbjERoAFZTNHms5JAnREIz3PRvR3mvh++j0QD4kuM5UkgHwI0IPlwpSoJkACIQHh32KRdSl0uGX453N133x2clte7YfgyOpeosMxiJPD\/AdnkG4+rX8aoAAAAAElFTkSuQmCC","height":328,"width":544}}
%---
%[output:86468a60]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAiAAAAFICAYAAABpxkW2AAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ90HtV55i8p26CEsLYMlAjFMTFygE1qY5ccoTghqQ54u1uZNG0qyWd3U0fNuidQ9tS4khwCGxqCJR2b3Rz+NCpVdNjdWvZuC6nd05YmLiFQ4YYarDQbUgTCGCFIDMIFgmnD1nveIVdcjWbm3jvzzDfzjZ7vHA62584zM7\/3\/e59vvtvTjl58uRJxQ8JkAAJkAAJkAAJ1JDAKTQgNaTNS5EACZAACZAACQQEaECYCCRAAiRAAiRAAjUnQANSc+S8IAmQAAmQAAmQAA0Ic4AESIAESIAESKDmBGhAao6cFyQBEiABEiABEqABYQ6QAAmQAAmQAAnUnAANSM2R84IkQAIkQAIkQAI0IMwBEnAkMDs7q3p6eoLSIyMjqrGx0fFM\/2KTk5Nq8+bNat26dWpgYEA1NDT4i6Q4o5bP6HJ7msNv\/\/Zvq87OTpdTSl1mcHBQDQ8PB\/e4ZcsW1dfX53W\/e\/fuVdu3b1c7duwoJQ95voMHD+b+\/fCCxsKlJUADUtrQ8MbKRqCWjXOcAZEK\/rLLLlOtra254Il7xryvG\/cwaRo0aQDvv\/9+r8Y9zTm+AZBrbNq0ae60KhqQqhlG3xizvB8BGhA\/XixNAoUQOHHihOrv71f79+9Xu3fvrpkBkZ6XWlw3CqpuzDo6OpzNhO4h8Gnc05yTJgm0AfG5t\/B1yt4DovP06NGj7AVJkySL7BwakEUW8LI8rtkVLfdkdimbv\/7XrFmjvvSlLwW3LQ2RORyhf61PTEwsOG5qXHnlleo3f\/M3gzJNTU1qdHRUtbS0RKIwG\/pw+XDvgBzXQzLt7e3qlltuUatXrw4qXrPhtunIUI6+7qFDh4L7k48egrnhhhvU7\/3e7wXmQ3\/CLOTfNVPToOhGzyyvGzGtZTaI5jPedtttamhoKPK609PTwf3NzMzM3ZMZQ5OjML\/99tvV1772NaWfT\/iHWWt2emgrqrHVcTWvq583\/Fw61s3NzXMmKsxv3759wZCG\/pj5EdazGb9wPiZpJeVhUk+JyUTuWd972NSEv18mW62xdetWdeDAASXfHx0785nl3\/Q1zNjauETlYVnqHd5HuQjQgJQrHpW\/m3CjYz6wrkSjGplwwyE60vhr8xE+HtVAJjXecizu3nRjsWzZsnlzQLQBMe9BGvoow2CakLAOyoBE\/cIONwbhhimOq\/x7nAHp7e1VV1999QL2SQ2+HDvrrLPUsWPHAoMVZQrkmmZDGb73uLzQ133kkUcizcTdd989N+\/CzDezgQ0bkLCWPh5nQpJyVs55+umnY42OeU9h8xE2ibrx\/8hHPqIeeOCBefVFlImI+n6FDYSUibpH+Xd9HZu2yaXsvTSVr2Tr6AFpQOooWFW4VV3Bmr8AdeUtz2f++pdfubpiMyt4s7I0f\/mZDZY08voXutbQ1w7\/0tZco46blenll18ea0DMX4i+OjYDIr0+8rENhST10EivzIsvvriAifmrXTitWrVq3jO6DMGEh4c0ex1P6e0Ix13uReZDRPXMCMuNGzcGz2v2mLhMzHUZTgmXCf9dM9FmSe7fdm2de1HPo\/9NjKo8c9wQjMlR51M4pt\/4xjcCIxPVoxGnG+4F070+pkbStXUPic5\/GxfEUFMV6jo+g50ADYidEUsACcT9OtIVuFS8a9eujVwBYpY5cuRI5K9auVVTQ3516xUrtkmktl9ucQ28WSHL9X11UAbEvLaYCfmYDV5cw2A2wJ\/97GedDUhUj1HUdeU+wkNMcT0MUlYaUn0fJtuo64WHopIMSNzQU\/icpN6MKPMafjY9vBc2Mtp0xRkFW36a8TU14uIa7k3RrLQBiRt6M1d4mbmsv5fm8JeuGkwuUcN+wCqEUhUiQANSoWDWw6PU2oCYy1htFbyvcRDeUctyXXXMxjXcWIm2uQzXpQdEypgTN+XvsuQz3AMUbgB9DUiYY7iXJGx8UAZE53ec8ZGVQVEGJDyUY+sBqQcDEtXjpuMazr+4HhBTI+67QQNSD7Vq\/d4jDUj9xq4u7zztEEx4qECPqcf9mozqMrcZkKShE\/NXuYCXX4lxBsRVR7q2w+ZAD02FDYg08i6T+8KNs9lDEB7GkgbbNgQjvTO2BjyskXYIxkzouF6FcNKHzUS4N0A\/s9kTpp9H5074nKghGNuXLe8hGG1Wdc9RnAGJ6jnSjMI9IHGThsPDP0lDMFFcOARjyxYe1wRoQJgLNSWQ9yTUpAbcZkCS7i1qfkScAbHpSHe1ns8Rhu9iQOScqFUwWiu8ksHcwMtnEqruijfPket+8pOfDHpnoj7CKer5XCehiqY2ZWHjEzdB0zzHLCPX\/MpXvqJuuummBRNm5ZywAZF\/i5vQqp\/VZnijhidsPVAmx7hnTDIPZoN\/zTXXxOZWkobcQ9TkVNdJqCYXWw9gTSscXqzUBGhASh2e6t6c6zJccwmtbRlu1MRWnyEYoZ3UvW+b5GnujJqkI9cJL9m8\/vrr1eHDh+cmXUb1gJiNU9xEWlM7PDclyqCYDbF5rvxZG5Co6955553zdvSUzdHM+SZpluGaRsJsEKOWaMct\/w1zNeekiKZw27lzp9q2bVuAw+zJ0quZ4pb12vbvSFqGK9dy7RmIm7shvWBRjXtcr48wiloCHdWLEmde5d\/DO6\/GzaXRGi49ddWt2fhkPgRoQHxosWxNCNhWHNTkJniR1ASihnrCK53i9mExL5pmI7LUN73ITzQNozZaUStjbJi4EZmNEI+bBCplQKTCkj0KZPOkqAouaeMqpkV5CNCAlCcWae4kaQgqaego6lpptmJPc888J3oIRrjYNu+LMo1VeXcP8yJfApUxILZJa\/p4W1tb8BIn\/Xf5cvm+ECrfkFCdBqT+cyBs9uWJfM2HnMN3i9Q2F8JDoz7mQ+6UhrG28ar3q1XGgMg4qCS\/fOJ6QMLBkrHM8fHxmr5ttN4ThvdPAiRAAiRAAggClTAg8mvrxhtvVN3d3YEJoQFBpAY1SIAESIAESCA\/ApUwINKTIR\/ZqS9pDoiJUXcRd3V1BUMy\/JAACZAACZAACdSOQN0bEBkjvuuuu9R1112n5AVkLgZEz\/8QzObbVcPY3\/e+99UuErwSCZAACZAACcQQkDcXn3feeZXiU\/cGRIZcZA8C2dXRtgpGIudqPqSsGJCpqalKBbzIhyFPLH3yJE8sAawa85M8bQTq2oBEzbTXDxz12mzflS\/8AtnSx+84efrxspUmTxshv+Pk6cfLVpo8bYT8jleRZ10bkHD4bD0g0lsiuwMmDbuYmlUMuF\/KY0uTJ3liCWDVmJ\/kiSWAVatiflbagOgeD1kds2rVquDNpXq7ZZ0aSVtaVzHg2K+Enxp5+vGylSZPGyG\/4+Tpx8tWmjxthNyPP\/jEcfVr\/bep5\/\/4C+4n1UHJShkQNG9+gbBEn3rqqcpNosIS8lMjTz9ettLkaSPkd5w8\/XgllR57+Hl11dhjavaWj+NES6BEA5IQBBoQbIayQiJPLAGsGvOTPLEEcGo0IDiWdaNEA4INFSt48sQSwKoxP8kTSwCnRgOCY1k3SjQg2FCxgidPLAGsGvOTPLEEcGo0IDiWdaNEA4INFSt48sQSwKoxP8kTSwCnNnjvETV471OcA4JDWn4lGhBsjFjBkyeWAFaN+UmeWAI4NRoQHMu6UaIBwYaKFTx5Yglg1Zif5IklgFOjAcGxrBslGhBsqFjBkyeWAFaN+UmeWAI4NRoQHMu6UaIBwYaKFTx5Yglg1Zif5IklgFOjAcGxrBslGhBsqFjBkyeWAFaN+UmeWAI4NRoQHMu6UaIBwYaKFTx5Yglg1Zif5IklgFOTXVBlKS53QsUxLb0SDQg2RKzgyRNLAKvG\/CRPLAGcGg0IjmXdKNGAYEPFCp48sQSwasxP8sQSwKnRgOBY1o0SDQg2VKzgyRNLAKvG\/CRPLAGcGg0IjmXdKNGAYEPFCp48sQSwasxP8sQSwKnRgOBY1o0SDQg2VKzgyRNLAKvG\/CRPLAGc2sbbH1UPPnmck1BxSMuvRAOCjRErePLEEsCqMT\/JE0sAp0YDgmNZN0o0INhQsYInTywBrBrzkzyxBHBqNCA4lnWjRAOCDRUrePLEEsCqMT\/JE0sAp0YDgmNZCqXJyUnV29urhoaGVEtLS+Q90YBgQ8UKnjyxBLBqzE\/yxBLAqa256SF1dPZ1zgHBIS1O6cSJE6q\/v18dOnRIjY6O0oDUKBSs4LGgyZM8sQSwasxPHM\/GrfcFYtwJFce0MKWDBw+qwcHB4PrsAaldGFghYVmTJ3liCWDVmJ8YntLzIT0gNCAYnoWqzM7OqhtvvFF1d3cHJoQGpHbhYIWEZU2e5IklgFVjfmJ40oBgOJZCZe\/evcF9rF27lnNAahwRVkhY4ORJnlgCWDXmJ4bng08cVxvveJQ9IBicxanIxNO77rpLXXfddWp6etrJgJh3e+DAgeJuvgJXFubNzc0VeJJyPAJ5YuNAnuSJJZBNrb29PRD45+UfVq+t\/QwNSDacxZ8tQy6XXXaZam1tVVwFU\/t48BcRljl5kieWAFaN+YnhOfbw80q2YpcPJ6FimNZcReZ+9PT0qImJiQXX3r17d2BKwh8uw8WGiRUSeWIJYNWYn+SJJYBR03uAvO21F9QLX\/0URrQkKqecPHnyZEnupaa3wR6QmuIOLsYKHsucPMkTSwCrxvzE8NR7gNCAYHiWQoUGpPZhYIWEZU6e5IklgFVjfmbnaa6AefvUN9VzX\/9ydtESKSzaHhCXGHAIxoWSexlWSO6sXEqSpwsl9zLk6c7KpSR5ulBKLmOugDn9wSF19Dt\/kV20RAo0IAnBoAHBZiorJPLEEsCqMT\/JE0sgu5qe\/yFKS77eo6amprKLlkiBBoQGpGbpyAoei5o8yRNLAKvG\/MzOU2\/Bvn7lEvW9Xb9KA5Idaf0osAcEGytWSOSJJYBVY36SJ5ZANjVz+e3t3Req6zrbaECyIa2vs2lAsPFiBU+eWAJYNeYneWIJpFeTyaey+6n8f3njaerwFy5VVWyPOATDIZj03xLPM1nBewKzFCdP8sQSwKoxP9PzNCef9m5Yofo3nEcDkh5nfZ5ZRcdZZCRYIWHpkyd5Yglg1Zif6XiaS2+l92Pf5y4OekGq2B6xB4Q9IOm+JSnOYoWUAlrCKeRJnlgCWDXmpz9PMR9Xjz2mHnzyeHCyDL2I+ZAPDYg\/z7o+o4oBLzIgrJCw9MmTPLEEsGrMT3+e5sRTPfdDq1SxPWIPCHtA\/L8lKc9ghZQSXMxp5EmeWAJYNeanH8+w+dBDLzQgfhwrU7qKjrPI4LBCwtInT\/LEEsCqMT\/dedrMB4dg3FlWpiQNCDaUrJDIE0sAq8b8JE8sAbuazPn4hx\/+WHXe+d2gsAy79G04T3Vfcs6Ck6vYHnEIhkMw9m8JqAQreBDIn8qQJ3liCWDVmJ\/JPMV8DN77lJLeD5v5YA8INjfrQq2KjrNI8KyQsPTJkzyxBLBqzM94nrLPx9V7Hgs2GtPmIzznI3x2Fdsj9oCwBwRb6ySosULCoiZP8sQSwKoxPxfyDPd6aPMhy21tHxoQG6GKHa9iwIsMESskLH3yJE8sAawa8\/MtnmI87v3+C6rv7sm5f5T5Hrd86v3qF9\/f6AS+iu0Re0DYA+KU\/IhCrJAQFN\/SIE\/yxBLAqjE\/VTDE8uSx19SvDk\/Mg2vucOpKnQbElVRFylUx4EWGhhUSlj55kieWAFZtMeenGI8\/\/94L6vNff6vHQw+3xK1ysdGvYnvEHhD2gNjyHnZ8MVdIMIiGEHliqZIneWYhIKZD\/hu696m5rdS1nvR43NZ1oVp\/\/pLUl6ABSY2uPk+sYsCLjAQreCx98iRPLAGs2mLJzzd7O46pz3\/9iQUAEcZDi1axPar7HpATJ06o\/v5+tX\/\/\/iBOW7ZsUX19fbHfpL1796rt27cHxzs6OtTAwIBqaGiILF\/FgGOrGD+1xVIh+VFJX5o807OLOpM8ydOVgJiO+\/5hVv23A0\/PLaU1ezs+cv5S9btXrJh7kZyrblK5KrZHdW9ABgcHg5iJ6ZidnVU9PT2qq6tLdXZ2LojlwYMHlZQfGRkJTIcYl6ampljDUsWAI74IaTVYwaclF30eeZInlgBWrWr5Kabjf\/3tc+p\/H3p+gekQcsjejqhIVLE9qnsDEg6UaUjCx6T3Y3x8fK7XI\/z3cPkqBhxbxfipVa1C8nt6fGnyxDIlT\/IME5ANw8RwiPGI+ojp+MTqs9VnPnwutLeDBgSbizVR0z0g0hvS2trq1APS1tYW2VsiJ9OAYMPGCp48sQSwasxP8pRejhde\/Wf1xf1PLphIqumI6ej4+bPUZ9c35246zIhUsT2qTA+I9HwMDw9b53VMTk6qzZs3q5mZGbV79+5Io6KDLgE3PwcOHMB+QxeZ2vT0tGpubl5kT53f45Inli15Lk6e33nmhPrDh\/9RHXr2zW3Roz5NZ5yq\/mv7mUr+L\/\/V4tPe3r7gMlNTU7W4dM2uUXMDonspJibmb8xie+LVq1ere+65x1YsmOMh5iJqcqkMuezZsyeYA9LY2BiUlU\/cpNUqOk4rwBwL8BcmFi55kieWAFatjPkpPRwT06+oOx+Yju3hEArSy\/HhlUtU9yXvDv4s\/xX9qWJ7VJgBiRsmiQqynjzqYkCkh6O3t1cNDQ2plpaWOTm9WsYccokrq0+qYsCL\/BKVsUIqkkfWa5NnVoLzzyfPavHUL3rb9c0j6qljJ6yGY\/nS09Rt3RcGEMpgOMLRqGJ7VDkDYq50kV4O\/aEBwVYuadRYwaehFn8OeZInlgBWrdb5KYbjoanj6o\/+9rlEs6ENhhiO3g3nlaaHw0afBsRGqIDj5jCKNhlxS2ujhmDihmvkUaoY8AJCNHfJWldIRT5rLa5NnljK5Fk\/PGV1yuxrP1F\/aBlK0U+kh1RkG\/Sy9nDY6FexPSqsB0TmgNg2DbMFRI6HNyIzNxfTx7q7u+cmm+rJqnIuNyJzIYwrwwoex1KUyJM8sQSwalnzU3o0xDiI2ZAXuv3JIz+09myYvRvbrlih3nbKKZm2P8cSyaZGA5KN37yzTSMgPRajo6Pz5mwAL5VaqooBTw0DcGLWCglwC5WSIE9sOMmzOJ56vob8f+zh59Qzs697mY3uD71bvWfpm5NFyzh\/A0G2iu1RzXtAwoEIr4pB9Ioggi0aVQw4ik0aHVbwaajFn0Oe5IklgFVLyk8xGl8bf1Y98vTLTkbD7Nn4jQ+fq84+\/Wcr07PhSr2K7VHhBsSEb+7RUYZekSoG3DXZ8yjHBhNLlTzJE0sAq\/Y3E5PqPe95j7rn8I\/UgcdedDYa2mzIMtj15y8NejayvEUW+1TFqVWxPSqVATFDG7eapZbhr2LAa8kvfC02mFj65EmeWALp1GSOhny+fvhH6vEf\/tjbaMhqlF9b93PqfWe+g0YjIQRVbI9KZUDMHhCJg22n0nRfF\/ezqhhw96fHl2SDiWVKnuSJJRCvpudo\/OD5H6t93z2mjr6YvK9GWCmYm7H0NHXpyiVK3hRb5bkaecWkiu1R4QYkaRVLXoF01a1iwF2fPY9ybDCxVMmTPFEEzBUnYg7++4Gn1RM\/es2rN0MPnYjR+PVfOEed9pPj6kMX1e+yVxRblE4V26PCDIi5CqYMvR1RSVLFgKO+DGl02GCmoRZ\/DnmSZxoCMmQiJuP2bz2jHnvuVXX0pdcjXy8fp61XmcgcjYvfc4a64qJlQdHw6hPmZ5roxJ9Txfao5gbEXPVi24cDGz5\/tSoG3J8C7gxWSDiWokSe5BlFIO2S1rCWHjb56KqlqvW8JcFhn8mgzE9sflaxPaq5AXniiSfUNddco2644YbEN9GaofN5Fwwy5FUMOJKPrxYrJF9iyeXJc3HyNIdLZE7Go8+87LxvRpzJ+OC571K\/9IEz53oyEHtpMD+x+VnF9qjmBkT3gOT1MjpkyKsYcCQfXy1WSL7EaECwxOqDp2kw5I4PHX05WMbqO1Sin1b3ZKw4s0H9+rpz5oZKECYjiSi\/79jsrWJ7VJgBka3YfT6rV69WLm\/D9dG0la1iwG3PnOdxVkhYuuRZnzzDBuOUU5Ta\/R333T+jnlqbjHXvPUNtbjs3mNNR9EoT5ic2P6vYHtXcgGBDkq9aFQOeL7H6+IVZJAPktVnBI2ni5tToxl\/+L\/81vvNUdce3ngn+\/OCTb+6Z4fsxJ36u+rl3ql9Zc\/acRN49Gb73qsszP9OSiz6viu0RDUhCjlQx4NivhJ8aKyQ\/XrbS5Gkj5HfclWfYYJxx2s+o4QemU8\/D0HepezFWnv0O9Tvt7y1FL4YfwfmlXXlmucZiOreK7RENCA1Izb7DrJCwqMkzH57hIZKXX39DffX+Z4KLpe3BkHO1wXhP42lKXgtfhmESLEEakDx50oDkSbeE2lUMeJGY2WBi6ZNnNp56C\/G\/\/odZ9XdH\/lFNHXtVzbz8RmrRxWYwbKCYnzZCfser2B6xB4Q9IH7fggylWSFlgBdxKnlG8zT3wfinN\/5F3f3oD4PhkbSrSMJDJMuXNajeK1ZUvgcja7YyP7MSnH8+DQiWZ+nVqhjwIqGzQsLSX6w8TYPx5987pr73rP9unuFISO\/FG2+8od531ulK9sTY8tFmGoyM6bpY8zMjttjTq9gelaIHZO\/evWr79u0BeHkB3dNPP63Gx8fVwMCAamhoyCueVt0qBtz60DkWYIWEhVtFnnrliBiCP\/ybaXX46CsBtCxzL+R8cy+MD557utpw0ZkLDEYVeWIzzk+NPP142UpXsT0q3IDIO2FmZmZUb2+vuvrqq5VsUCZ7fvT396umpqbg70V9qhjwoljKdVkhYenXI09tMB544iX10E+XpCLMRWAylp6mrrjoTLXmPe+a914S12Wq9cgTm1FYNfLE8qxie1SoATF3RV21apXq6ekJDEdra6vS26+PjIyoxsbG2EiG36a7ZcuWRNMiups2bQr0xOgk6Vcx4NivhJ8aKyQ\/XrbSZeBpLknV93vXQzPq4SP\/COu50ObioqbT1S9\/8KxAV95Joq9t4+R6vAw8Xe+1HsqRJzZKVWyP6t6ASA+KfMS4aEPT1dWlOjs7F0R\/cnJSbd68We3cuTMwOTL0kzTUU8WAY78SfmqskPx42UrXgqe5JPXcJW9XX\/22vEH1x3BzIUtTuy55tzrFeKuqa8+FjZPr8VrwdL2XKpQjT2wUq9geFWpAJDzaBJhDMLo3JM5IJIXVNCThcnKtI0eOOA\/rVDHg2K+EnxorJD9ettJZeerJnHKd706\/oh544niq17NH3ac2DzIsIhtrXbn6bHXq206p2XtIbOyijmflmeaaVT6HPLHRrWJ7VLgBkRCZwyI6ZDt27IjsxUgKadKL7vRQTVtbm7NuFQOO\/Ur4qbFC8uNlK23jKftciBE48IMX1T2P\/iiQy7oUVTRMc6GHRczeilr3XNg4uR638XTVYbk3CZAnNhOq2B6VwoAgwiQ9H8PDw6qjoyNy9Yw2IBs2bFB33nmnkpfhcQ4Igry7Biskd1ZJJbWx+PZ3p9ThF05Vj\/\/wxxBjETYX7RcuU+uWn5FqQifmSWurwvzE8iZPLE8aECzPXNT0qprwEl5tQI4ePTo38TSurL4xCbj5OXDgQC73vFhEp6enVXNz82J53FTPKTtxPvfKG+o7z5xQj878U6Bx6NnXU2mZJzWdcWrw13e\/61Qlf\/6Vf\/Mu9ca\/nAz+bd25p2XWr4IA8xMbRfLMxrO9vX2BwNTUVDbRkp1daA+IHjKR3oikj21li3muTDSV+SRDQ0OqpaVl7lDUEExcWdOAVC3gRebfYvxFFH6vyPHXfqL+4v++ANmZU8fSHBK5ePkZ6v3nvFOtX7lkLtT1OiRS61xdjPmZJ2PyxNJlDwiWZ6AmE0P37NkzbzmsuZpl48aNXnuCJC3flR6PFStWzM0BEQNy8803q127dkUu9a1iwHMIobNk1SokcxLno8+8rP7myePqB8\/lMxwiq0Rk86zGd\/6rgLcsQ60aT+dEyqkgeWLBkieWZxXbo1L0gOi9P8xwmUbi8ccfV2Ie7rnnngURNVe96F6OuA3MwuYkacWMXKiKAcd+JfzU6qVCMrf6fsfPvk3t\/s7z0HkWQs3stfjFC5apX3jvGQHMuX9vtA+L1AtPvywprjR5YtmTJ5ZnFdujujcg4Y3IzEmo+lh3d3ew74d8zBU3cRNWOQSD\/eJotTJUSHonzhM\/+X\/qz\/\/+BfXksdeC28u6G6d+RtNYrHvvGar9gmWBqcjj1etl4JlPphSjSp5Y7uSJ5UkDguUZqNmGYGRDMb1XyFe+8pUc7iBesooBrynA0MXyrJD0q9Wblrxd3f6tZ9TkDzGbZUUZCzEUWy9foWaOvzlJVIZDivjkybOI5yn6muSJjQB5YnlWsT0qtAdEhydqHxB5KZ3erfTWW29Vo6Oj8yaVYkMbrVbFgNeCW9w1fCskc46F\/Pmbj72oHjn6cm49Fu9d1qD+\/QfPUqe\/\/WfqYumpL88iY18P1yZPbJTIE8uziu1RKQwINkw4tSoGHEfHXylcIen9LP7y+y+oP5s4FggiNsoSHXMoZMWZDeqKi5apJQ1vTuD0mWfh\/5S1O4MVPJY1eZInlgBWrYrtEQ1IQo5UMeDYr0S0mjmJ88\/+\/pj6\/syruRiLVT\/3TvWx9y9VP3\/uu3KZY1ELVlmuwQYzC72F55IneWIJYNWq2B4VbkD0C+JmZmYWRMu2Uyk2vAvVqhhwBDNtMB6YfEmNTx0P9rTIOonT7LGQJaefWndO8O4Q+RQ1xwLBKk8NNphYuuRJnlgCWLUqtkeFGhBzczC934esWNEvo4tanosNabJaFQPuws\/swfj25Evq4NTxTD0Y2lyc1aBU26qz1QXnvFN9mBtluYQisQwbzMwI5wmQJ3liCWDVqtgeFWpAwi+PMzcKk4mpY2Njke91wYY1Xq2KATef1tyl86++\/4I6\/Mwr3j0ZZs+FDImc9HsoAAAgAElEQVR8Ys3ZsRM4WcFjM5c8yRNLAKvG\/MTyrGJ7VCoDIsttjxw5oqTnI2lHU2xYF5cBkYmf+797LHjtuuuwiTYZHzl\/qer8hXNST+JkhYTNXPIkTywBrBrzE8uTBgTLM1AzdyM1Tcc3vvENNT4+zh6QjMyll0Mmgv7l916wGg4xGsuXnqb+Q2uTavrXbw+MBvI9IqyQMgYzdDp5kieWAFaN+YnlSQOC5RmohV8SJ4ZkeHhYyXbqRez9YT5ivQZcejnGHn5OjT38fGzExFjIPIzuS94NNxpxF2WFhP0CkSd5Yglg1ZifWJ712h4lUSh0CAYbHrxavQX8zgenVd\/dk5EgxHBs\/Pmz1BUXnVnYqhJWSNgcJU\/yxBLAqjE\/sTzrrT1yefpCDUh4Eqp5w5wD4hI+paS3Y+jepyKHV8R03NZ1Yc16OGx3zArJRsjvOHn68bKVJk8bIb\/j5OnHy1aaBsRGyPM4DYgnMKO4zO24euyxBcZDTMe+z10clETO30h\/p2+dyQoJQZE8sRTJkzzzIoDVpQEB8ZTVLtu3b7eqbdmyJVgRU9SnjAEX4zF471ML5nesX7lE3db9Zm9HWT80INjIkCd5Yglg1ZifWJ5lbI+yPmFph2CyPhji\/LIFXIZbNt7x6LxH2\/Shd6vbui5APG7uGqyQsIjJkzyxBLBqzE8sz7K1R4inK9SAIB4gT42yBDxquEUPtZS5xyMcG1ZI2GwlT\/LEEsCqMT+xPMvSHiGfigYkgWYZAi7mQ3o99PboemJpPb4fhRUS8qurFHmSJ5YAVo35ieVZhvYI+0RK1dyA6ImnExMT1mdZ7C+jk308rhp7bI5TPfZ6mEFmhWRNea8C5OmFy1qYPK2IvAqQpxcua2EaECuiahUoOuCNW++bA3r3b61RH1u1tK4Bs0LCho88yRNLAKvG\/MTyLLo9wj7Nm2o17wHJ4yHy0iwq4GbPh\/R69G04T3Vfck5ej1kzXVZIWNTkSZ5YAlg15ieWZ1HtEfYp5quVwoBELcvdsWOH6uzstD673sp9\/\/79QVnXpbuTk5Oqt7dXDQ0NqZaWlsjrFBHwqpoPAcwKyZrOXgXI0wuXtTB5WhF5FSBPL1zWwkW0R9abyligcAMi5mPPnj1qZGRENTY2Bo+j54l0dXVZTYj5MjvX87RpOXToUOL7Zmod8CqbDxqQjN\/UiNNZwWOZkid5Yglg1WrdHmHvPlqtUAOSx06opiGJA6i3eZfjZekBkVUua256aO6WZTfTelzpkpS0rOCxX2nyJE8sAawa8xPLkwYEy3Oup0N2O21tbZ2nnuZdMEmGRotLmRtvvFF1d3crMStlMCDhpbaHv3BpqXc0TZsGrJDSkos+jzzJE0sAq8b8xPKkAcHyDNSyDsHoWxIzMTw8rDo6OtTAwIBqaGiIvFu5nnzWrl3rNAfEFDlw4EAOBJT64jdfUPsfezXQ\/sRF71LXty\/L5TpFi05PT6vm5uaib6My1ydPbCjJkzyxBLKptbe3LxCYmprKJlqyswsdgtEsskxCDfMUIzIzMxNpQmTi6V133aWuu+46JZVNGSahhud9SO9HVT\/8RYSNLHmSJ5YAVo35ieXJHhAsz1zUkla3iDm57LLLguGeMqyCMed91PMOp66BZIXkSsqtHHm6cXItRZ6upNzKkacbJ9dSNCCupBzLua5acZQLisXNHUnagXX37t0L5qCIVt4B\/8TvH1bfnnwpuO\/buy+sxF4fSbFiheSTyfay5Gln5FOCPH1o2cuSp52RT4m82yOfe0GVLXwIJjz8EmcG4h7YXPWil9c2NTUpmdia9Cm6ByTc+1HloRcdB1ZIqK\/tmzrkSZ5YAlg15ieWJw0IlucCNT2RVA6IiRgdHY3dJEyfHN6IzJyEqo\/JipfwKpsiDUj47bZVXHIblSqskLBfIPIkTywBrBrzE8uTBgTLM1FNzIgMp5gblNXw8sGl8gq4OfH0Ny5tUrd86v21frRCrscKCYudPMkTSwCrxvzE8syrPcLepZ9a4UMw5u2aPSBFvwk3LwNi7vlR72+39Us1Dhn48rKVZwVvI+R3nDz9eNlKk6eNkN9xGhA\/Xk6l0wy7OAkDCuUR8AefOK423vFocHfygjmZfLpYPqyQsJEmT\/LEEsCqMT+xPPNoj7B36K9WaA+Iy86l\/o+EOwMd8HDvx2KYeGpGgxUSLjdFiTzJE0sAq8b8xPJEt0fYu0unVqgBSXfLtTsLHXBz7sdi6\/1gg4nPW1bwWKbkSZ5YAlg1dHuEvbt0ajQgCdyQATdXviy2uR8aMSv4dF\/SuLPIkzyxBLBqzE8sT2R7hL2z9Go0IDUyIObcj2t+cbn64i+vTB+1Oj2TFRI2cORJnlgCWDXmJ5YnDQiWZ+nVkAHfePuj6sEnjwfPvFj2\/QgHmBUSNuXJkzyxBLBqzE8sT2R7hL2z9GqF9oAkTUKN21I9\/aP6n4kKuLnr6frzl6p9n1vjfzMVOIMVEjaI5EmeWAJYNeYnlieqPcLeVTY1GpAaDMH8z799Tv2XvT9Y1L0f8vCskLJ9WdmjhOVHnuSZLwGsOg0IiGf4\/S9xslu2bLG+0wV0S5EyiIAvxne+xMWEBgSbreRJnlgCWDXmJ5Ynoj3C3lF2tdL2gGR\/tOwKiICbBqRvw3mqb8OK7DdWpwqskLCBI0\/yxBLAqjE\/sTwR7RH2jrKrFWpAst9+vgqIgHPy6VsxYoWEzVfyJE8sAawa8xPLE9EeYe8ouxoNSAJDRMAbt94XXGH9yiVq31UXZ49YHSuwQsIGjzzJE0sAq8b8xPJEtEfYO8quVnMDYq58WbVqlerp6VETExORT1L0C+myBtzc+VTe+SK7ny7mDyskbPTJkzyxBLBqzE8sz6ztEfZuMGo1NyCY266NStaAc\/hlfpxYIWHzljzJE0sAq8b8xPLM2h5h7wajRgOS0xDMvL0\/OPwSUGaFhPnSahXyJE8sAawa8xPLkwYEy1Pp4ZgqDsFw9cvCZGGFhP0CkSd5Yglg1ZifWJ40IFiesWpiTK699lr1+c9\/XrW0tNToqgsvkyXg5vDL4S9cquQFdIv9wwoJmwHkSZ5YAlg15ieWZ5b2CHsnOLXSDsHIVuxjY2NqYGBANTQ0xD7xiRMnVH9\/v9q\/f39QJmnzsnCPS0dHR6J+2oBz87HocLFCwn1xOaSFZUme5IkngFVM2x5h7wKrVmoDMjg4qEZGRlRjY2PsU0sZ+fT19c0N6XR1danOzs5552ij0tbWFhzTf29qaordbTVtwDn8QgOC\/ZqSJ3nWggD2GvzBgeWZtj3C3gVWrbQGRIzFzMyMtQckjMM0JDZUsiX8+Ph47DXSBnzw3iNq8N6ngstz+OWtKLBCsmWk33Hy9ONlK02eNkJ+x8nTj5etdNr2yKZb5PFCDUjSJFTpmRgdHfWaA5L0dt0oyHkZED3\/Q+Z9iAHh500CrJCwmUCe5IklgFVjfmJ50oBgeQZqcUMjeqjE9ZLS8zE8PKxs8zq0njYrUcM1ukyagHP5bXzEWCG5ZrNbOfJ04+RaijxdSbmVI083Tq6l0rRHrtpFlSu0B0QeOmqoxcUcxAFzGbrRpkc0kia5SsDNz4EDB6xxOvTs6+o\/3\/18UG7\/p5tV0xmnWs9ZLAWmp6dVc3PzYnnc3J+TPLGIyZM8sQSyqbW3ty8QmJqayiZasrMLNSBJQyauq2DCPCcnJ1Vvb68aGhqKHL5xNR+im8Zx\/upXJ9R9j88Gt7Xvcxer9ecvKVnIi7sd\/iLCsidP8sQSwKoxP7E807RH2DvAq5XagLisggkjEeMSd57LyhdTL03A19z0kJJhGM7\/WJisrJCwX2DyJE8sAawa8xPLM017hL0DvFqhBiQ8\/8N8PNsEUV3WXPViMxguwzNZDIg5\/+N3r1ihtv\/b8\/ARq2NFVkjY4JEneWIJYNWYn1ieNCBYnoGa9Fhs27Zt3ooXGUbZvHmz2rlzp2ptbU28angjMnMSqj7W3d2t4t68m\/TGXd+AP\/jEcbXxjkeD++XwC3tAcvi6zJNkBY8lTJ7kiSWAVfNtj7BXz0et0B4Q\/UhiQjZt2jTvCXfv3m01H\/kgeUvVN+Dcfj05IqzgsRlLnuSJJYBVY35iefq2R9ir56NWCgOSz6NlV\/UNOOd\/0IBkzzp3BVbw7qxcSpKnCyX3MuTpzsqlpG975KJZdJlCDYg5RGIbaikClE\/Auf+HPUKskOyMfEqQpw8te1nytDPyKUGePrTsZX3aI7taOUoUakB8dy6tNTKfgHP+hz06rJDsjHxKkKcPLXtZ8rQz8ilBnj607GV92iO7WjlKFGpABIHrapcicPkE3Hz\/CyegRkeLFRI2i8mTPLEEsGrMTyxPn\/YIe+X81Ao1IEnvgpFHTlqhkh+St5R9As73v9gjwgrJzsinBHn60LKXJU87I58S5OlDy17Wpz2yq5WjRKEGpBwI4u\/CJ+CNW+8LhNavXKL2XXVx2R+tkPtjhYTFTp7kiSWAVWN+Ynn6tEfYK+enRgOSwNY14Jz\/4ZagrJDcOLmWIk9XUm7lyNONk2sp8nQl5VbOtT1yUytHqZobEHPiadzmYBpNvQzBcP6HWzKzQnLj5FqKPF1JuZUjTzdOrqXI05WUWzkaEDdOlSnlGnBzA7LZWz5emedHPwgrJCxR8iRPLAGsGvMTy9O1PcJeNV+1mveAhB8n\/D6YpPfD5ItiobprwDkB1S0yrJDcOLmWIk9XUm7lyNONk2sp8nQl5VbOtT1yUytHqcINSNQL4vQwTVdXl+rs7CyMlEvAuQGZe3hYIbmzcilJni6U3MuQpzsrl5Lk6ULJvYxLe+SuVo6ShRqQpI3I5P0wY2NjamBgQDU0NBRCyyXg5gTUvg3nqb4NKwq513q4KCskbJTIkzyxBLBqzE8sT5f2CHvF\/NVKbUCkd2RkZEQ1NjbmTyLiCi4BH3v4eXXV2GPB2Ye\/cKla3nhaIfdaDxdlhYSNEnmSJ5YAVo35ieXp0h5hr5i\/WqEGJGm+Rxl2SHUJON+A656krJDcWbmUJE8XSu5lyNOdlUtJ8nSh5F7GpT1yVytHyUINiCCQoZZt27ap0dFR1dLSElCZnJxUmzdvVjt37lRFvqTOJeCcgOqeyKyQ3Fm5lCRPF0ruZcjTnZVLSfJ0oeRexqU9clcrR8nCDYg2IZs2bZpHZPfu3YWaD7kZW8A5AdUviVkh+fGylSZPGyG\/4+Tpx8tWmjxthPyO29ojP7VylC6FASkHioV34RJwvQU7J6Dao8gKyc7IpwR5+tCylyVPOyOfEuTpQ8te1qU9squUqwQNSEI8bAHnFux+ycwKyY+XrTR52gj5HSdPP1620uRpI+R33NYe+amVozQNSAYDwi3Y\/ZKYFZIfL1tp8rQR8jtOnn68bKXJ00bI7zgNiB+v0pXWq272798f3NuWLVtUX19f7H3aAs4JqH4hZoXkx8tWmjxthPyOk6cfL1tp8rQR8jtua4\/81MpRelH1gMi+IvIR0+Gy26ot4GtuekjJRFTZ+0P2AOEnmQArJGyGkCd5Yglg1ZifWJ629gh7tdqoLSoDEkZqGpIo3EkB5woY\/wRlheTPLOkM8iRPLAGsGvMTy5MGBMszUNN7fszMzCxQX716dW47oSZtA69vJCng3ILdPxlYIfkzowHBMiNP8qwdAeyVaECwPJWek9HU1JQ4FwN8WSU9H8PDw6qjoyPxXTMScPNz4MCBub\/uf+xV9cVvvhD8\/Q8+eY5ady63YLfFaXp6WjU3N9uK8bgjAfJ0BOVYjDwdQTkWI09HUDHF2tvbFxyZmprKJlqyswsdgnHphciTV9SbeM3rJTlOcwUM3wHjFiX2gLhxci1Fnq6k3MqRpxsn11Lk6UrKrRx7QNw4OZfSPSDd3d2F7Hoqwz+9vb1qaGhobht4VwPCFTDOYZ4ryArJn1nSGeRJnlgCWDXmJ5YnDQiWZ6Am74Ip6q23tmsnBZwrYPyTgRWSPzMaECwz8iTP2hHAXokGBMtzbinsxMREpDJ6Eqq56sVl\/klcwLkCJl0i0ICk4xZ3FnmSJ5YAVo35ieVJA4LlWXO18EZkLpNQoyb9mAaE74BxDyMrJHdWLiXJ04WSexnydGflUpI8XSi5l6EBcWdViZJxAec7YNKFlxVSOm7sAcFyI0\/yrA0B7FVoQLA859T27t2rtm\/fHvx99+7d6umnn1bj4+OJS2RzupV5snEB5ztg0tGnAUnHjQ0mlht5kmdtCGCvQgOC5Rmo6aWwshrl6quvDvYDkbkf\/f39qtb7g4QfLy7gV409psYefj4oPnvLx3OgUk1JGhBsXMmTPLEEsGrMTyxPGhAsz7lJqGI6Vq1apXp6egID0traWujqGP2YcQHnEtx0icAKKR03\/mLHciNP8qwNAexVaECwPOvWgDRuvS8gsX7lErXvqovBVKorRwOCjS15kieWAFaN+YnlSQOC5RmoyfwPme9hDsHo3pCuri7V2dmZw1XdJKMCbq6A6b7kHHV794VuYiylWCFhk4A8yRNLAKvG\/MTypAHB8pxTkw3BNm3aNE99x44dhZoPuZmogHMFTPokYIWUnl3UmeRJnlgCWDXmJ5YnDQiWZ+nVaECwIWKFRJ5YAlg15id5Yglg1WhAsDxLrxYVcC7BTR82VvDp2bEHBMuOPMkzfwLYK9CAYHnOqZn7gOh\/lP1AZDVMkZ+ogG\/748fV18afDW6LS3D9okMD4sfLVpo8bYT8jpOnHy9bafK0EfI7TgPix8uptJiPPXv2qJGREdXY2Phmwz47GyzJLeMkVC7BdQprZCFWSOnZ8Rc7lh15kmf+BLBXoAHB8py3DDfc22F7Uy34ViLlogKu34LLJbj+EaAB8WeWdAZ5kieWAFaN+YnlSQOC5Vl3BoRvwc2WAKyQsvELn02e5IklgFVjfmJ50oBgeQZq0tOxbds2NTo6qlpaWko9BMO34GZLAFZI2fjRgGD5kSd55ksAq04DguU51wMyMTFhVZb3w9xzzz3WcsgC4YCbe4DIBmSyERk\/7gRoQNxZuZQkTxdK7mXI052VS0nydKHkXoYGxJ1VJUomGZB9n7tYrT9\/SSWes1YPwQoJS5o8yRNLAKvG\/MTypAHB8iy9Wjjg5h4gh79wqVreeFrpn6FMN8gKCRsN8iRPLAGsGvMTy5MGBMtzTi1qH5AybsWul+DKjXMPEP9kYIXkzyzpDPIkTywBrBrzE8uTBgTLM1Crp31AuAdItgRghZSNX\/hs8iRPLAGsGvMTy5MGBMsTsgxXb1qmJ7J2dHSogYEB1dDQEHm3Zm+LrWw44NwDJFsCsELKxo8GBMuPPMkzXwJYdRoQLM\/MBuTEiROqv79ftbW1BW\/O1X9vampSfX19C+7W3NxMDIqcG1dWTjYDzj1AsgefBiQ7Q1OBPMkTSwCrxvzE8qQBwfLMZQhGejjGx8cje0HCx5LKJhmQvg3nqb4NK3KgUW1JVkjY+JIneWIJYNWYn1ieNCBYnnNqyEmoSaYiqgdE955EPZoZcHMPEC7BTZcIrJDScYs7izzJE0sAq8b8xPKkAcHyhKu5vMRucnJSbd68Wc3MzCjbG3fNgI89\/Ly6auyx4J5pQNKFjhVSOm40IFhu5EmetSGAvQoNCJYnVE3P\/xDRuEmo4RU3g4ODwT1EzReRf5eA68\/rF1ypXr9gY\/DXP\/jkOWrdudwDxDeA09PTqrm52fc0lo8hQJ7Y1CBP8sQSyKbW3t6+QGBqaiqbaMnOPuXkyZMnS3ZP3rfjYj7CE1blItIb0tvbq4aGhubeQ2Ne3HSc0vshvSDy4R4g3iEKTmAPSDpucWeRJ3liCWDVmJ9YnuwBwfKEqNlWvuiLZDUg3AMke7hYIWVnaCqQJ3liCWDVmJ9YnjQgWJ4QNRlGkfkcSXt\/6AtFDcEknWsGXO8BItuvyzbs\/PgTYIXkzyzpDPIkTywBrBrzE8uTBgTLM7NaeBMyLShvzh0ZGQk2I5O9Prq7u1Vra2twWAzL8PBw8Gefjcgat94XnLN+5RK176qLM9\/7YhRghYSNOnmSJ5YAVo35ieVJA4LlWXo1M+DagHRfco66vfvC0t97GW+QFRI2KuRJnlgCWDXmJ5YnDQiWZ+nVdMDNPUC4CVn6sLFCSs8u6kzyJE8sAawa8xPLkwYEy7P0alEGRHo\/pBeEH38CrJD8mSWdQZ7kiSWAVWN+YnnSgGB5ll5NB3zw3iNq8N6ngvvlJmTpw8YKKT079oBg2ZEneeZPAHsFGhAsz9Kr6YB\/8wez6tf\/YCK4X1kBIyth+PEnQAPiz4w9IFhm5EmetSOAvRINCJZn6dV0wLkJGSZUNCAYjlqFPMkTSwCrxvzE8qQBwfIsvZoOODchw4SKFRKGIw0IliN5kmc+BLCqNCBYnqVX0wHnJmSYUNGAYDiywcRyJE\/yzIcAVpUGBMuz9Go64NyEDBMqGhAMRzaYWI7kSZ75EMCq0oBgeZZeTQL+rb\/7vpIeEPlwE7JsIaMBycYvfDZ5kieWAFaN+YnlSQOC5Vl6tbAB4SZk2ULGCikbPxoQLD\/yJM98CWDVaUCwPEuvJgH\/H3\/1iNp4x6PBvdKAZAsZDUg2fmwwsfzIkzzzJYBVpwHB8iy9WtiAcBOybCGjAcnGjw0mlh95kme+BLDqNCBYnqVXk4Bv+f2\/5i6ooEjRgIBA\/lSGPMkTSwCrxvzE8qQBwfIsvVrYgHAX1GwhY4WUjR9\/sWP5kSd55ksAq04DguVZejUJ+Aeu\/RP14JPHg3udveXjpb\/nMt8gDQg2OuRJnlgCWDXmJ5YnDQiWZ+nVTAMi73+RHhB+0hNghZSeXdSZ5EmeWAJYNeYnlicNCJZn6dUk4Gd85o\/U0dnXgxfQ0YBkCxkrpGz8OGSA5Uee5JkvAaw6DQiWZ+nVJODHPzES3Of6lUvUvqsuLv09l\/kGaUCw0SFP8sQSwKoxP7E8aUCwPEuvtuIDH1IvXzEY3Cd3Qc0eLlZI2RmaCuRJnlgCWDXmJ5YnDQiWJ0RtdnZW9fT0qImJiUCvo6NDDQwMqIaGhkj9gwcPqk2bNgXHVq9erUZGRlRjY2NkWdOAcBOy7OFihZSdIQ0IliF5kmd+BLDKNCBYnpnVTpw4ofr7+1VbW5vq7OxU+u9NTU2qr69vgf7k5KTavHmz2rlzp2ptbVV79+5V4+PjsYZl+Yd+Sb26vjfQub37wqAXhJ\/0BGhA0rOLOpM8yRNLAKvG\/MTypAHB8sxFLclUyLEjR45EmpOom6EBwYaIFRJ5Yglg1Zif5IklgFWjAcHyzEUtzoCEe0tcLt78sf+oXlv7maAot2F3IZZchhV8doamAnmSJ5YAVo35ieVJA4LlCVfT80G6urqCIRnzow3Ihg0b1J133hnMGbHNAWn6d7+jXr9gYyBz+oND6v69d8DveTEJTk9Pq+bm5sX0yLk+K3li8ZIneWIJZFNrb29fIDA1NZVNtGRnn3Ly5MmTJbunVLejDYacHDUJVR8\/evTo3MTTwcFBNTMzEzsH5Jxfu0n98\/IPB\/fDbdhThWXeSfxFlJ0he0CwDMmTPPMjgFVmDwiWJ0zNZj7kQlFDMDIptbe3Vw0NDamWlpYF93P2f\/qqeuPM9wf\/zm3Ys4eLBiQ7QzaYWIbkSZ75EcAq04BgeULUbCtfzItIj8eKFSvmhmfEgNx8881q165dkUtxtQHhLqiQUCkaEAxHrUKe5IklgFVjfmJ50oBgeULUbMMo5kVkDxApr\/f+kD\/LJ2rJrvz7mb\/1f9S\/vONM7oIKiZSiAQFxpAEBg\/ypHBtMLFfyxPKkAcHyzKwW3oRMC+rJpbIZmewT0t3dHez7IR9zIzLbpmWNW+8LzuE27JlDFQiwQsJwpAHBciRP8syHAFaVBgTLs\/Rq2oBwG3ZMqGhAMBzZYGI5kid55kMAq0oDguVZejVtQLgNOyZUNCAYjmwwsRzJkzzzIYBVpQHB8iy9Gg0INkQ0IOSJJYBVY36SJ5YAVo0GBMuz9GragPA9MJhQsYLHcOQvdixH8iTPfAhgVWlAsDxLr6YNCLdhx4SKBgTDkQ0mliN5kmc+BLCqNCBYnqVXowHBhogGhDyxBLBqzE\/yxBLAqtGAYHmWXk0bEG7DjgkVK3gMR\/5ix3IkT\/LMhwBWlQYEy7P0atqAcBt2TKhoQDAc2WBiOZIneeZDAKtKA4LlWXo1MSDchh0XJhoQHEtRIk\/yxBLAqjE\/sTxpQLA8S69GA4INESsk8sQSwKoxP8kTSwCrRgOC5Vl6NTEg3IYdFyZW8DiW7AHBsiRP8sQTwCrSgGB5ll6NBgQbIhoQ8sQSwKoxP8kTSwCrRgOC5Vl6NTEgfA8MLkys4HEs+Ysdy5I8yRNPAKtIA4LlWXo1MSB8DwwuTDQgOJZsMLEsyZM88QSwijQgWJ6lV6MBwYaIBoQ8sQSwasxP8sQSwKrRgGB5ll5NDAjfA4MLEyt4HEv+YseyJE\/yxBPAKtKAYHmWXm3FBz6kjnzvO6W\/z3q5QRoQbKTIkzyxBLBqzE8sTxoQLM\/Sq1Ux4EVCZ4WEpU+e5IklgFVjfmJ5VrE9OuXkyZMnsZiqo1bFgBcZHVZIWPrkSZ5YAlg15ieWZxXbo7o3ILOzs6qnp0dNTEwE0e7o6FADAwOqoaEhMfqTk5Oqt7dXDQ0NqZaWlsiyVQw49ivhp8YKyY+XrTR52gj5HSdPP1620uRpI+R3vIrtUV0bkBMnTqj+\/n7V1tamOjs7lf57U1OT6uvri42uLnfo0CE1OjpKA+L3PUhduopfoNQwACeSJwCiIUGe5IklgFWrYn7WtQGJCu\/evXvV+Ph4Yi\/IwYMH1eDgYHA6e0CwX5IktSp+gWpHb+GVyBNLnzzJE0sAq1bF\/Fx0BkSGbG688UbV3d0dmBAaEOyXhAaEPGtHAHulKlbwWEJ+auTpx8tWuoo8K2VA9HyQrq6uYEgmrodE\/n3t2rVOc0BsScHjJEACJEACJMOjwVAAAAkZSURBVFALAlNTU7W4TM2uURkDoud1CLm4Sagy8fSuu+5S1113nZqenrYakJpFgRciARIgARIggUVGoBIGxMV8SFxlyOWyyy5Tra2tymUVzCLLBT4uCZAACZAACdSMQN0bENeVL+Hluibh3bt3B6aEHxIgARIgARIggdoQqHsDIr0aMzMzTnt\/mEjZA1KbBONVSIAESIAESCCKQF0bkLhejdWrV6uRkZFgMzLZJ0RWvIR7OGhA+IUgARIgARIggeII1LUBKQ4br0wCJEACJEACJJCFAA1IFno8lwRIgARIgARIIBUBGpAIbDKvZHh4ODjCCarueSXDWps3bw7m5NjeyWMylq3zk7bEd7+DapX04amfPPx6gmoRyf40PkzDQ7ysCxby9+FpluV33j+X9Xc7akqBv1o5zqABCcVBb9Muc0gef\/zxYOmu\/LmxsbEcESvpXZgN38aNG+e9oyd8y+Ht8uXve\/bsIWcDlA9Pk6+w3L59u9qxY0fsZnwlTaHcb8uHaXh1HeeMLQyPD09t5uQdXTIfj995v3TXrPfv31+pH8U0IKE80O+IkS9KFR2nX9q7lw5X0GLkxsbGnFYnsXKP\/mVpvq3ZhadU8tdee606fvy4StoN2D2q1Srpk6NS9uabb1a7du3ij4+YNPDlaeYzv\/Pu3y3dc7Ru3Tp19OjR4EWrVdk2ggYk5len+XZd\/bZd95RZfCXNniPpLQr\/PYkIK6OFdNLwFPN8ySWXqD\/90z+de0P04svE+Cf2YeryUsvFztaHZ1QPiO2loYudr37+Z599NvijrOrs6emhAalqYkT1eEilvmLFCnZnW4Ie\/oXu8wsy7V4uVc1DeS5fnvo1A1u3bg1etkjTHG3qzF65pBwVA3LkyJFAhPPBor9pvjlqDiNs2bIlaEj5cScQNnHuZ5a3JHtAInpAzEk+NCBuyetbGWlVqehvvfVWTkINYfbhKRX7l7\/8ZfXpT39aNTc3J86\/cYtmNUv5MNVzafTEUzl327ZtzFMjNXx46mGEnTt3BsMHPj2k1cxG\/6eiAfFnVldnhFcQcEWBe\/h8umNpPuxcfXhK2fvvvz\/4RcmcjWfrwzQ8BEOu0T1K5iT9JFNBnvbvvK0EDYiNUAWOmz0enITqHtBwd7Zt0iRnwSez9eFpLmk2VdnNPZ+xD9Nw\/rIuWJivPjxpQNzr0riSNCDZGZZegctw04XIZ0keu7PtjH14mmr8pR7P1odpuLLnkMFCrj48o4ZgOKRlrwfMEjQgfrzqtjQ3IksXuqRNifSkPhkmiPvFzo2eFv5ij9vYzeRJA+Ker645KormRmTcOCuasQ9PMXGbNm0KhMjTPWd1SRoQf2Y8gwRIgARIgARIgAQWEOAqGCYFCZAACZAACZBAzQnQgNQcOS9IAiRAAiRAAiRAA8IcIAESIAESIAESqDkBGpCaI+cFSYAESIAESIAEaECYAyRAAiRAAiRAAjUnQANSc+S8IAmQAAmQAAmQAA0Ic4AECiTw5JNPqqVLlzq\/8l32AnjppZfUypUrc7trvU\/L6tWr1cjIiPO91ctbjfXz1WovilpfL7fEoDAJgAnQgICBUo4EXAn47q5Zi42IspiILOe6MkOUE0Mgn1q+jbVe2CD4UoMEXAnQgLiSYjkSABMoowHxvScTSb00sjQg4ESmHAmkJEADkhIcTyMBFwLmdt5SXg9rPP7443PbUsu\/623ow9vU62GCZcuWqZ6eHjUxMRFcVr9oTr+PY\/\/+\/cG\/uwybmNcwhyH0K+j1c+3YsUN1dnYueMy4ctqAbNy4UX3pS18KzgsPc5jbcYevI6yuvfZa9dGPfjQ4Xz\/Liy++qPSW9HLO9ddfr\/bt26eGhoZUS0tLIBP3TFExChsQ+fsrr7wS\/Kc5Jr3IL\/xiNblG1L\/VozlzyWmWIQEUARoQFEnqkECIQNSL4czGL9zbEPfGUJEdGBhQoicmRIYOWltb595V0tXVNWcUkt4yrO9H6zU0NAQN56233qpGR0eDxtzWAxIub75kTEySGIV169YF96v19+zZE8wlESPR29s7zziYetpkLV++fO58beD0M+q\/Hzt2LLjn5uZm1d\/fHxgdPaRie9lhlAEZHh5W2nCFrxlObBoQftVJAEOABgTDkSoksICArSGzNfbhX9ZhAxJ+ZbyUT3obbtQQSbh80j3Z3rQbfuOp3I9tWMY8rg1I2FCNj4\/PGRLRNA2G\/P3mm29Wu3btmjdZNmmYJcqAzMzMLLiGlIuahEsDwi87CWAI0IBgOFKFBCIJmMMV4eGRuMY+PEzR0dER2QMSHgoxbyBq+CTueuabdZMMiG0SbJTZiPq38LBUeJhJ9\/DI80QZCVNTelX0G1bDAYgbRokyIHKuOSk1yTjRgPDLTgIYAjQgGI5UIYFEAmaja84DMX9la0MRnpehewDCPSBybviXe9JNxJmLpGEhUy+rAREtPZdDG6SoHhAfA\/LII48oPcTT2NjolIU0IE6YWIgEcidAA5I7Yl6ABN4iYDbi+he+dPPLfAmZy9DW1jZv4qdpMsIGJGm+RxTzWgzBhOd4mNcUs5A0nKKHYEwDEtXbYA7BSA\/Itm3b5uawuORaHkMwNjNoG4pyuW+WIYGqEaABqVpE+TylIRA1B8TshTAnZUZNptQ9InoIRh7MNClaXyak6uGDqHkYGghqEqrZ42DOC1m7du2CSaZhA2Keq+9V7k8mlEYZENdJqKKhJ77a5t5knYSqh8j0yiX9HObk23AS0oCU5mvJGykRARqQEgWDt1I9ArpxkqES+ZjDK+YSWhmSuPzyy+cttRXjceWVV6obbrhh7hd+2JToXhG9PFeuoRvGOJpJS1ZdJ8Zu3759Tj5qOEUvjw03vOFr79y5M5jnIRNP9fObPSBykTDD8DLc8FJkOSduCbHudZL\/a9MWtQzXPD\/KPJjzbyROa9asUYcPHw5MUNgo6mcI9w5VL9v5RCTgR4AGxI8XS5MACRRMQAxB1MoX19tymQPiquVajj0grqRYbjERoAFZTNHms5JAnREIz3PRvR3mvh++j0QD4kuM5UkgHwI0IPlwpSoJkACIQHh32KRdSl0uGX453N133x2clte7YfgyOpeosMxiJPD\/AdnkG4+rX8aoAAAAAElFTkSuQmCC","height":328,"width":544}}
%---
