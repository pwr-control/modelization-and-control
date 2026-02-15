preamble;
model = 'inv_psm';
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

fPWM_INV = fPWM;
tPWM_INV = 1/fPWM_INV;
TRGO_INV_double_update = 1;
if TRGO_INV_double_update
    ts_inv = tPWM_INV/2;
else
    ts_inv = tPWM_INV;
end

fPWM_DAB = fPWM;
tPWM_DAB = 1/fPWM_DAB;
TRGO_DAB_double_update = 0;
if TRGO_DAB_double_update
    ts_dab = tPWM_DAB/2;
else
    ts_dab = tPWM_DAB;
end

s=tf('s');
z_afe = tf('z',ts_afe);
z_inv = tf('z',ts_inv);
z_dab = tf('z',ts_dab);


% deadtimes
dead_time_AFE = 3e-6;
dead_time_INV = 3e-6;
dead_time_DAB = 3e-6;

% minimum pulse
minimum_pulse_time_AFE = 2e-6;
minimum_pulse_time_INV = 2e-6;
minimum_pulse_time_DAB = 2e-6;

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
nominal_power = 1600e3;
application_voltage = 690;
vp_xi_pu = 1;
vn_xi_pu = 0;
vn_eta_pu = 0;
grid_emu_data = grid_emulator(nominal_power, application_voltage, vp_xi_pu, vn_xi_pu, vn_eta_pu);
%%
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
l1 = Kd(2) %[output:59afecfa]
l2 = Kd(1) %[output:6273c9b1]
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
Imax_adc = 1049.835;
CurrentQuantization = Imax_adc/2^11;
%%
%[text] ### Voltage sensor endscale, and quantization
Umax_adc = 1500;
VoltageQuantization = Umax_adc/2^11;
%%
%[text] ### DClink, and dclink-brake parameters
Vdc_ref = grid_emu_data.Vdclink_nom; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi = 900e-6*8;
%[text] ### DClink Lstray model
Lstray_dclink = 100e-9;
RLstray_dclink = 10e-3;
C_HF_Lstray_dclink = 15e-6;
R_HF_Lstray_dclink = 22000;
Z_HF_Lstray_dclink = 1/s/C_HF_Lstray_dclink + R_HF_Lstray_dclink;
Z_LF_Lstray_dclink = s*Lstray_dclink + RLstray_dclink;
Z_Lstray_dclink = Z_HF_Lstray_dclink*Z_LF_Lstray_dclink/(Z_HF_Lstray_dclink+Z_LF_Lstray_dclink);
ZCFi = 7/s/CFi;
sys_dclink = minreal(ZCFi/(ZCFi+Z_Lstray_dclink));
% figure; bode(sys_dclink,Z_Lstray_dclink,options); grid on
%[text] ### LCL switching filter
LFu1_AFE = 0.5e-3;
RLFu1_AFE = 157*0.05*LFu1_AFE;
LFu1_AFE_0 = LFu1_AFE;
RLFu1_AFE_0 = RLFu1_AFE/3;
CFu = (100e-6*2);
RCFu = (50e-3);
%%
%[text] ### DClink voltage control parameters
Vdc_nom = grid_emu_data.Vdclink_nom;
Vdc_norm_ref = Vdc_ref/Vdc_nom;
kp_vs = 0.85;
ki_vs = 35;
%%
%[text] ### AFE current control parameters
%[text] ### Resonant PI
kp_afe = 0.2;
ki_afe = 45;
delta = 0.015;
res = s/(s^2 + 2*delta*grid_emu_data.omega_grid_nom*s + (grid_emu_data.omega_grid_nom)^2);

Ares = [0 1; -grid_emu_data.omega_grid_nom^2 -2*delta*grid_emu_data.omega_grid_nom];
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:82a1ee3d]

%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1;
pll_p_ddsrt = pll_p;
omega_f = grid_emu_data.w_grid;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
tau_ddsrf = 1/omega_f;
%%
%[text] ### PLL FHT
pll_i1_fht = pll_i1;
pll_p_fht = pll_p;
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega_fht0 = grid_emu_data.w_grid;
delta_fht0 = 0.05;
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:8da9fcc7]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:9d901226]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:2b6324c3]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:1d0e7351]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid_emu_data.w_grid;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:5ad4d344]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:6fac1d01]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:1ec31383]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:8e6905b6]
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;
kp_rc_pos_grid = kp_rc_grid;
ki_rc_pos_grid = ki_rc_grid;
kp_rc_neg_grid = kp_rc_grid;
ki_rc_neg_grid = ki_rc_grid;
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = 2*pi*rms_perios/grid_emu_data.w_grid/ts_afe;
rms_perios = 10;
n10 = 2*pi*rms_perios/grid_emu_data.w_grid/ts_afe;
%%
%[text] ### Online time domain sequence calculator
w_grid = grid_emu_data.w_grid;
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
time_start_motor_control = 0.25;
%[text] ### MOTOR Selection from Library
n_sys = 6;
run('n_sys_generic_1M5W_pmsm'); %[output:9565d738] %[output:2efd1d55]
run('n_sys_generic_1M5W_torque_curve');
% n_sys = 1;
% run('testroom_eq_psm_690V');
% run('testroom_torque_curve_690V');

torque_overload_factor = 1;

b = tau_bez/omega_m_bez;
external_motor_inertia = 5*Jm;
% external_motor_inertia = 1;

%% inverter filter
% LFi = 40e-6;
% RLFi = 5e-3
%% inverter filter
LFi = 230e-6;
LFi_0 = 20e-6;
RLFi = 5e-3;

%[text] ### Simulation parameters: speed reference, load torque in motor mode
% rpm_sim = 3000;
rpm_sim = 17.8;
% rpm_sim = 15.2;
omega_m_sim = omega_m_bez;
omega_sim = omega_m_sim*number_poles/2;
tau_load_sim = tau_bez/5; %N*m
b_square = 0;

%[text] ### Double Integrator Observer Inverter Side
Aso = [1 ts_inv; 0 1];
Cso = [1 0];
% p2place = exp([-10 -50]*ts_inv);
p2place = exp([-50 -250]*ts_inv);
Kobs = (acker(Aso',Cso',p2place))';
kg = Kobs(1) %[output:2e86d8c0]
kw = Kobs(2) %[output:8374d010]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:5e758c7f]
luenberger_l2 = Klo(2) %[output:154790c7]
luenberger_l3 = Klo(3) %[output:0ed06d2a]
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
Vdc_bez = grid_emu_data.Vdclink_nom;
motorc_m_scale = 2/3*Vdc_bez/ubez;
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
igbt.inv.data = 'infineon_FF1200R17IP5';
igbt.inv = device_igbt_setting_inv(fPWM_INV);

% infineon_FF650R17IE4;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.afe.data = 'infineon_FF1200R17IP5';
igbt.afe = device_igbt_setting_afe(fPWM_AFE);
%[text] ### DEVICES settings (MOSFET)
infineon_FF1000UXTR23T2M1;
mosfet.inv.data = 'infineon_FF1000UXTR23T2M1';
mosfet.inv = device_mosfet_setting_inv(fPWM_INV);

infineon_FF1000UXTR23T2M1;
mosfet.afe.data = 'infineon_FF1000UXTR23T2M1';
mosfet.afe = device_mosfet_setting_afe(fPWM_AFE);

%[text] ### DEVICES settings (Ideal switch)
silicon_high_power_ideal_switch;
ideal_switch = device_ideal_switch_setting(fPWM_AFE);
%[text] ### Setting Global Faults
time_aux_power_supply_fault = 1e3;
%[text] ### Lithium Ion Battery
nominal_battery_voltage = grid_emu_data.Vdclink_nom;
nominal_battery_power = 250e3;
initial_battery_soc = 0.85;
lithium_ion_battery = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_inv); %[output:6a130e76]
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
    set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
    set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
else
    if use_thermal_model
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'off');
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    else
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_adv_thermal_model', 'Commented', 'on');
        set_param('inv_psm/inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    end
end

if use_torque_curve
    set_param('inv_psm/fixed_speed_setting', 'Commented', 'off');
    set_param('inv_psm/motor_load_setting', 'Commented', 'on');
else
    set_param('inv_psm/fixed_speed_setting', 'Commented', 'on');
    set_param('inv_psm/motor_load_setting', 'Commented', 'off');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":36.3}
%---
%[output:59afecfa]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:6273c9b1]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:82a1ee3d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:8da9fcc7]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:9d901226]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:2b6324c3]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:1d0e7351]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:5ad4d344]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:6fac1d01]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:1ec31383]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:8e6905b6]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.181909345636866","29.587961813168029"]]}}
%---
%[output:9565d738]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:2efd1d55]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:2e86d8c0]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:8374d010]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:5e758c7f]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:154790c7]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714302e+02"}}
%---
%[output:0ed06d2a]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.994503273143434e+02"}}
%---
%[output:6a130e76]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10XsV55x9a2lobQmzxURDCMXHkhDSpjL0kQjhxsir1tl2ZHNocSd5Ns4qSdRscThd8JDlOOHUgWNLadCmQxmUV1e1WsrcbSOxmuy4xhIYIN8RgnWZDQCBsI4QD2BhIMKEk3vNcM2J0dT9m7p3nfe8d\/e85HCO9zzx35vfMO\/PXfJ528uTJk4QHBEAABEAABEAABEpE4DQImBJFC1kFARAAARAAARAICEDAoCKAAAiAAAiAAAiUjgAETOlChgyDAAiAAAiAAAhAwKAOgAAIgAAIgAAIlI4ABEzpQoYMgwAIgAAIgAAIQMCgDoAACIAACIAACJSOAARM6UKGDIMACIAACIAACEDAoA6AgCMCx44do66ursDb4OAg1dbWOvI82834+Dh1dnbS8uXLqa+vj2pqasTepTuuZBlNCqQ4fPazn6W2tjaTJIW26e\/vp23btgV5XLt2LfX09Fjld+fOnbRhwwbavHlzIXlw+fbt2yf+\/bCCBuPSEoCAKW3okPGiEahk5x4nYLiDWLlyJTU1NYngiSuj9HvjCpOlQ+QO9L777rMSB1nS2AaA37FmzZrpZD4KGN8Ep22MYe+WAASMW57wBgJVIXDixAnq7e2l3bt30\/DwcMUEDI\/8VOK9UVBVZ9ja2mosRtQIhY04yJImSyVQAsYmb+H3FH0ERtXTw4cPYxQmSyVBmhkEIGBQIQpJQB9K5wzqQ+L66MPSpUvphhtuCMrAHZk+naJGC8bGxmZ9rvu48sor6VOf+lRgU1dXR0NDQ9TQ0BDJRRcKYfvw6AR\/rqaUWlpa6Oabb6bGxsag4dY7\/jQ\/PBWl3rt\/\/\/4gf\/yoKaTrr7+evvjFLwbiRT1hFvx7xVQXOKrT1O1VJ6h86R2qXsbbbruNBgYGIt87OTkZ5G9qamo6T3oMdY7M\/Pbbb6evfvWrpMrH\/MOsFTs1NRfVWau46u9V5Q2XS8W6vr5+WoSF+e3atSuYklGPXj\/C\/tKEY7g+JvlKqodJIzU6E86zyntYFIW\/Xzpb5ePaa6+lvXv3En9\/VOz0MvPv1Dv02KZxiaqHhWyEkKnCE4CAKXyI5lYGw52WXnrVCEd1UuGOh\/2weFDiJfx5VAeb1PnzZ3F5U53NWWedNWMNjBIweh5YKEQJDl3EhP24EjBRf+GHO5NwxxbHlX8fJ2C6u7tp3bp1s9gnCQb+7JxzzqHnnnsuEGhRooLfqXe04bzH1Qv13oceeihSjNx5553T6070+qZ30GEBE\/alPo8TMUl1ltMcOnQoVijpeQqLl7DIVOLhgx\/8IH3nO9+Z0XhEiZCo71dYgLBNVB759+o9ab51LkUfJZpbLW65SwsBU+74eZd71UDrf4Gqxp8Lq48+8F\/ZqmHUOwi9sdX\/8tQ7PBYJaoRA+VDvDv+lryBHfa43xldccUWsgNH\/QrX1kyZgeNSJn7SpnKQRIh4VOnr06Cwm+qgBc1qyZMmMMppMIYWntxR7FU8ebQnHnfPC60GiRoaY5erVq4Py6iM2JgubTaaDwjbhnxUTJbY4\/2nvVnUvqjzqdyx0ucxxU0g6R1WfwjG9++67AyEUNaIS5zc8CqdGnXQfSe9WIzSq\/qdxcTFV5l3DhwJlIgABkwkbEkkRiPvrTHUA3HAvW7YscgeObnPw4MHIv6o537oP\/qtf7RhKW4Sb9pdjnEDQG3R+v60fVwJGfzeLEX70DjOuY9E78E9\/+tPGAiZqxCrqvZyP8BRZ3AgH23JHrPKhs416X3gqLUnAxE2dhdMkjaZEid9w2dT0ZFgIKdEWJzTS6qceX91HXFzDozmKlRIwcVOH+g47vS6r76U+fafaCZ1L1LSlVHsCv34TgIDxO76lK12lBYy+DTmtg7AVHgw\/alu1qR+9cw53duxb30ZtMgLDNvrCV\/6Zt+yGR6DCHaitgAlzDI\/ShIWTKwGjKnuccOKdWVECJjwVlTYCUwYBEzXip+Iarn9xIzC6j7jvBgRM6ZpYrzIMAeNVOMtfmKxTSOGpDrWmIO6v2agh\/zQBkzT1o48KcBT4r9Q4AWPqh4fmw+JCTa2FBQyLBJPFkeHOXR+hCE\/DcYefNoXEo0NpAiDsI+sUkl6740Y1wt+AsBgJj0aoMusjcao8qu6E00RNIaV986SnkJTYVSNXcQImauRKMQqPwMQtug5PXyVNIUVxwRRSWm3B56YEIGBMScGuIgSkF\/EmCYA0AZOUt6j1IXECJs0PD7er9Sxh6CYChtNE7UJSvsI7SfQD4GwW8aqpBD0Nv\/eqq64KRoeiHuYUVT7TRbzsU4m6sHCKW+Cqp9Ft+J233HIL3XjjjbMWHHOasIDh38UtCFZlTRPMUdMraSNgOse4MiaJD10wXHPNNbF1K8kH5yFqca\/pIl6dS9oIZEUaGrzECwIQMF6E0b9CmG6j1rdAp22jjloYbDOFxJSTpifSFsnqJ\/Mm+eH3hLfcfuELX6ADBw5ML1qNGoHRO7e4hci67\/DanCiBo3fkelr+fyVgot57xx13zDhRlg\/X09fbZNlGrQsRvUON2mIft307zFVfk8M+mduWLVto\/fr1AQ59JE3tJovblp12fkvSNmp+l+nIRNzaFR6FixIHcaNOzChqC3vUKE6c+OXfh0\/+jVtLpHyYjBT616KhRBIEIGAkqMKnKIG0HR+iL4fz3ASipqrCO83izuHRX57lILvcmZ+jDnTBqYRa1M6kNDw4yC6NED63IeClgOGGjc+i4EO2ohrCpAPObODBtjoEIGCqw93VW5Om0JKmvqLen+UqAVflmGt+oqaQmEHa4Y9RotOXu6vmWh0oWnm9EzBpi\/vU583NzcFlZ+pn\/hLaXpxWtGDOlfxAwJQ\/0uE\/IrhEtuKF0+BuncrWhfDUro144ZxCcFY2Xr6\/zTsBw\/O9\/CXhJ24EJhxU\/stidHS0orf6+l6xUD4QAAEQAAEQkCTglYDhv+o2bdpEHR0dgYiBgJGsOvANAiAAAiAAAtUj4JWA4ZEUfvhEyKQ1MDpuNZTd3t4eTCnhAQEQAAEQAAEQKD4BbwQMz4Vv376dNm7cSHxRn4mAUetfOEz6LcbhsL3jHe8ofiSRQxAAARAAARCIIMC3il900UXesfFGwPCUEZ81waeHpu1C4iiaihe2ZQEzMTHhXfCrXSBwlYsA2IKtHAE5z6i3Mmx95eqFgIna0aCqQdT19rY7j3wNvsxXxdwruJqzsrUEW1ti5vZga87K1hJsbYmZ2fvK1QsBEw5h2ggMj9bwKZRJ00a6T1+Db1b15azAFWzlCMh5Rr0FWzkCMp59rbNzQsCoERfenbRkyZLghmB1LLiqLklHr\/safJmvirlXcDVnZWsJtrbEzO3B1pyVrSXY2hIzs\/eVq5cCxiyk5la+Bt+cgIzlk08+6eXCMhladl7B1o6XjTXY2tCyswVbO16m1r72YRAwBjXA1+AbFF3UBI2VHF6wBVs5AnKeUW9l2Prah0HAGNQXX4NvUHRREzRWcnjBFmzlCMh5Rr2VYVv\/4Y\/T5Lf\/RsZ5Fb1CwBjAh4AxgJTBBI1VBmiGScDWEFQGM7DNAM0wCdgagrIwG3nwCF098ggdu\/kjFqnKYQoBYxAnCBgDSBlM0FhlgGaYBGwNQWUwA9sM0AyTgK0hKAszCBgLWD6aQsDIRBWNlQxX9gq2YCtHQM4z6q17thAw7pmWyiMEjEy40FjJcIWAkeMKtmArS8C99\/49B6l\/z5OYQnKPthweIWBk4gQBI8MVnawcV7AFW1kC7r1DwLhnWiqPEDAy4YKAkeGKTlaOK9iCrSwB994hYNwzLZVHCBiZcEHAyHBFJyvHFWzBVpaAe+8QMO6ZlsojBIxMuCBgZLiik5XjCrZgK0vAvXcIGPdMS+URAkYmXBAwMlzRycpxBVuwlSXg3jsEjHumpfIIASMTLggYGa7oZOW4gi3YyhJw750PseOt1DjIzj3bUniEgJEJEwSMDFd0snJcwRZsZQm49w4B455pqTxCwMiECwJGhis6WTmuYAu2sgTce4eAcc+0VB4hYGTCBQEjwxWdrBxXsAVbWQLuvUPAuGdaKo8QMDLhgoCR4YpOVo4r2IKtLAH33lff\/jDd\/8RxrIFxj7YcHiFgZOIEASPDFZ2sHFewBVtZAu69Q8C4Z1oqjxAwMuGCgJHhik5WjivYgq0sAffeIWDcMy2VRwgYmXBBwMhwRScrxxVswVaWgHvvEDDumRbG4\/j4OHV3d9PAwAA1NDRE5gsCRiZcEDAyXNHJynEFW7CVJeDe+9IbH6DDx17FGhj3aKvr8cSJE9Tb20v79++noaEhCJgKhwMCRg442IKtHAE5z6i37tnWXntv4BQH2blnW1WP+\/bto\/7+\/iAPGIGpfCjQWMkxB1uwlSMg5xn11i1bHnnhERgIGLdcq+7t2LFjtGnTJuro6AhEDARM5UOCxkqOOdiCrRwBOc+ot27ZQsC45VkYbzt37gzysmzZMqyBqVJU0FjJgQdbsJUjIOcZ9dYt2\/sfP06rv\/wwRmDcYq2uN164u337dtq4cSNNTk4aCRg9x3v37q1uATx5O7Ovr6\/3pDTFKgbYysUDbMFWjoAbzy0tLYGj1xZeTq8s+yQEjBusxfDCU0YrV66kpqYmwi6k6sUEf23JsQdbsJUjIOcZ9dYt2\/49B6l\/z5MQMG6xVs8br33p6uqisbGxWZkYHh4ORE34wTZqmXihsZLhyl7BFmzlCMh5Rr11y1adAfNLrzxPz3\/lY26dF8DbaSdPnjxZgHxULQsYgakaenSygujREcjBBVuwlSPg1rM6A+b05x+lZ\/\/6j9w6L4A3CBgcZFe1aoiOQA492IKtHAE5z6i37tjqO5Bq\/mUHPX33NnfOC+JpzgsYkzhgCsmEkr0NGit7ZqYpwNaUlL0d2NozM00Btqak0u10AXPG\/QN0+Hv\/kJ6oZBYQMAYBg4AxgJTBBI1VBmiGScDWEFQGM7DNAM0wCdgagjIwU+tf2HT+17toYmLCIFW5TCBgDOIFAWMAKYMJGqsM0AyTgK0hqAxmYJsBmmESsDUElWKmj76sWDyffrD19yFg3KAtnxcIGJmYobGS4cpewRZs5QjIeUa9dcN25MEjdPXII4Gz2zsupo1tzRAwbtCWzwsEjEzM0FjJcIWAkeMKtmArSyC\/dx59WTfyCN3\/xHFaWDuPDnz+MvK1D8MUkkF98TX4BkUXNYGAkcMLtmArR0DOM+ptfrb66MuftLydrv+9d0DA5MdaXg8QMDKxQ2MlwxWjBHJcwRZsZQnk886jL3z3Ef\/Loy+3tV9MK945HwImH9Zyp4aAkYkfBIwMV3SyclzBFmxlCeTz\/lcPTNG1f\/do4KR71SLqXXVR8P++9mGYQjKoL74G36DooiYQMHJ4wRZs5QjIeUa9zc5WnzpSa1+UN1\/7MAgYg\/ria\/ANii5qgsZKDi\/Ygq0cATnPqLfZ2OrbpvWpIwiYbDy9SgUBIxNONFYyXDHNIccVbMFWloC9d33dC6fe9ZlLgnUv+uNrH4YRGIP64mvwDYouagIBI4cXbMFWjoCcZ9RbO7b6yAunXPuhetr80YZZTnztwyBgDOqLr8E3KLqoCRorObxgC7ZyBOQ8o96as73\/8ePBjiP1\/KcPnE9\/3vbuSAe+9mEQMAb1xdfgGxRd1ASNlRxesAVbOQJynlFv09nyqMtdB56lTX\/\/xLTxzk\/\/Jl1x8VmxiX3twyBg0uuLt1vQDIouaoLGSg4v2IKtHAE5z6i3yWx51GXdjkeCc1744QW7Pasuoo5Lz0tMCAEjV2cL79nX4FcbPBoruQiALdjKEZDzjHobz7Z\/z0Hq3\/PktAGLF16wy\/+mPb72YRiBSYu8x4cAGRRd1ASNlRxesAVbOQJynlFvZ7LlkZbRieP0meFTFzOq5398\/DfoqkvONQ4EBIwxKv8MfQ1+tSOFxkouAmALtnIE5Dyj3p5iy8Jlzw+PUs+dj82AbTPqoif0tQ\/DCIzBd9HX4BsUXdQEjZUcXrAFWzkCcp5Rb4mu2fkj+p\/\/\/Mws4aLuNcpC39c+DALGoDb4GnyDoouaoLGSwwu2YCtHQM7zXK23vDh3YM+TdP8Tx50KF+XM1z4MAsbgu+hr8A2KLmoyVxsrUahvOAdbOcpgC7YuCLBo+cbYszT43adnuYu6DiDPO33tw7wRMCdOnKDe3l7avXt3EOe1a9dST09PbMx37txJGzZsCD5vbW2lvr4+qqmpibT3Nfh5vhAu0qIjcEEx2gfYgq0cATnPvtdbXtvytYd\/TDd8cyJStFy+eH6wLdpkZ5FNFHztw7wRMP39\/UE8WbQcO3aMurq6qL29ndra2mbFed++fcT2g4ODgWhh4VNXVxcreHwNvs0XQMLW98ZKgpmpT7A1JWVvB7b2zExT+MiWR1rufuQo3Xrv4UgMrkdbol7iax\/mjYAJB00XNOHPePRldHR0etQl\/HPY3tfgmzYqUnY+NlZSrGz9gq0tMXN7sDVnZWvpC9u4NS2KB4uWW9vfTW+vrXE+2gIBY1vrCmavRmB4NKapqcloBKa5uTlytIYTQ8DIBNiXxkqGTj6vYJuPX1JqsAXbMAGeGhr+3jM0+sTxWQtxddFyS9u76aKzKiNa9Dz62od5NwLDIy\/btm1LXdcyPj5OnZ2dNDU1RcPDw5FCR1UADr7+7N27V+4bPIc8T05OUn19\/RwqceWKCrZyrMF2brOdeun1AMCRl1+nr\/zzcdr\/9Klj\/cNP3Zmn0\/lvPZ3WfmA+Lb8g\/bRcl1RbWlpmuZuYmL3uxuU7q+HLOwGjILKQYXEStTiXp4x27NgRrIGpra0N1sPwE7fo11f1Wo0Kp78Tf8nKRQBswVaOgJznotZbtY7l4cMvxY6wMBWeGvrY8vNoZcMCWvHO+XKgLD372od5K2B4hKW7u5sGBgaooaFhOtxqt5I+ZRRnqxL5GnzL74Bz86I2Vs4LWgWHYCsHHWz9Z8uC5c6Hf0z3PHps+uLEqFKzYOGdQx2Xnl8owRLOq699mLcCRt9pxKMs6oGAkWt8bD2jI7AlZm4PtuasbC3B1paYuX012LJYOfrT12jw\/qcTR1fUCIsSLCxeXG93NidlZwkBY8cr1VottB0bG0u11Q0aGxvprrvumpVGnwZSIiVua3TUFFLcdBO\/yNfgW4EXMK5GYyVQjEK6BFu5sIBtedmyWHn+J6\/RV7+bLlaUYLmy8VzquvyCoNBlESwYgZGro4HntJ1CUa9XoypRAiZ8kJ1+OJ36rKOjY3qxrlrsy+\/BQXbCwY5xj45AjjvYgq0cATnPLusti5Xjr\/wr\/eV3JlNHVpQ4WbhgHnW\/cZBcWcVKVHR8\/SO8alNIrgWM3FcKIzBSbF02VlJ5LKtfsJWLHNgWhy1vX2ahwf\/e\/K1DNPHcK1Zi5ZOXX0Bnn\/GrgQ+fBAtGYOTqaOk8+6peqx0IdARyEQBbsJUjIOc5rd7yqMpjz\/6Uvv7ws0ZCRR9Z+XfvrqV\/+\/a3FXqxrRRZX\/uwqo\/A8BqYtHuLpIJq6tfX4JuWX8ourbGSeu9c8Au2clEGW1m2v\/y284MRle8fepHu+dExOvzCq4k7gfTc6LuCpsVLbWXPYJGjk92zr31Y1QSMCoW+FoUX3Q4NDc3Y9pw9ZO5S+hp8d4SyeUJHkI2bSSqwNaGUzQZss3ELp2KRws+9jx6jrz30Y2uhwutVPrRkATVdNN\/7KaC8xH3tw6ouYFRgwruSijQq42vw834p8qZHR5CXYHx6sAVbOQJ2nnnah59\/+H\/P079Mvmw89aNGUPjfMpy1Ykelsta+9mGFETB6OPVj\/oswKuNr8Cv7FZr9NnSychEAW7CVIzDbM4+m8H\/P\/eQ1Gvru04HB\/U+cEi4mj1pA2\/jrp9MNv\/++wJfvC2tNuLiy8bUPK6SA0YMWdyCdq8Ca+PE1+CZll7RBJytHF2zB1jUBJVImnn+F\/vnJF+mpY69aiRQ1osJTP7\/1nrNo2YVnTu\/8UQIG9dZ11E7587UPK6SA0UdgGH7aZYsyIX\/Tq6\/Bl+aW5h+NVRqh7J+DbXZ2aSl9Z8tTPr9+5q\/SLfccpsNHT2QSKcyQp32u\/a1F9MyLPzMeTfGdbVrdkvrc1z6sMAIm6SA6qaCa+vU1+Kbll7JDYyVFlghswTaKgJqaUYe88bqULCMp+mjKf\/zA+XTB\/DfPUclzngrqrUy99bUPq7qA0XchFWG0Jar6+Bp8ma+KuVc0VuasbC3B1paYuX0Z2KodPiMPHqHvPv6C1Q4fnUSwDmXBPGpePJ9WvHNB8JHk2pQysDWvKcWx9LUPq5qA0XcdpR3lX+1q4Gvwq80VjZVcBMDWb7ZKoPzoyE\/pG2PPZh5F0UdSll74VvrUivqqLqBFvZWpt772YVUTMI8\/\/jhdc801dP3110\/fT5QWuqS7kNLS5vnc1+DnYeIiLRorFxSjfYBtednq0zxcipEHnwkEis2BbuFRFP6ZR1Defd5baGn9W0VHUfKQR73NQy8+ra99WNUEDO5CkqmoZfKKxkouWmBbbLZqRw9Px9z27cP0o2d+mlmg6KMov1F3Bv3e+84JCr\/infPlIAh5Rr2VAQsB45hr+OA6U\/eNjY0UdRu1afosdr4GPwsLl2nQWLmkOdMX2FafLS+UZYFy672H6dEj+QVKIEoWz6c\/+a2305EXX5u1BVmuxJXzjHorw9rXPqxqIzAyYZLx6mvwZWiZe0VjZc7K1hJsbYmZ2zNbvq+Hn5+9\/otAoBx8\/kTuEZRToyYLaPHZNXTporcF\/iUXzJqXuHKWqLcyrH3twyBgDOqLr8E3KLqoCRorObxgm4+tmuL5xx8+TweeejlwZnOybPjtamsxn43ykXfV0vsXvS1YLFvGaZ58ZJNTo97K0PW1D4OAMagvvgbfoOiiJmis5PCCbTxbJU7q5v8a\/dm3DtGho\/lGT6ZHShbMowtr59GVjecGi2XVk+dcFLkaUkzPqLcycfG1D4OAMagvvgbfoOiiJmis5PDOVbZqezH\/+53HX6AH3riPx8XoCZ+H8t4LzqBLznqdzj\/\/\/Dk3vSNXW9\/0PFfrrTRbX\/swCBiDmuNr8A2KLmqCxkoOr29sw1uLf\/jMT2hs8uVc55+ER0h4\/ckF83+NPqgd2KZGV\/RI+cZWrhbaewZbe2YmKXztwyBgDKLva\/ANii5qgsZKDm9Z2CphoqZ1mMjLr75Of\/8vz+U6+yQsTtToye++99QW4zyLY8vCVq52yXkGWxm2vvZhhRIwO3fupA0bNgQR5AscDx06RKOjo9TX10c1NTWJkQ3fpbR27Vrq6emJTcOH4q1Zsyb4nLdmDw4OUm1tbaS9r8GX+aqYe0VjZc7K1rJIbPUzT\/7xh0fpwOTLwSWBWQ9m01moo+4XnV1Drb95DtX8yi+Lby8uElvbelF0e7CViZCvfVhhBAzfiTQ1NUXd3d20bt26QHywsOjt7aW6urpEMcIh5\/T8cDp1xkx7ezu1tbXNqhHqtustW7YEpwCzcEoSSr4GX+arYu4VjZU5K1vLSrHV15x8\/9CLdM+PjgVZzbPmRB894ZGTJb\/+Flp8Tk1wQJsasVGjKLZcXNhXiq2LvJbNB9jKRMzXPqwQAkY\/lXfJkiXU1dUVCBEWF+r6gKQRkqiQ64Im\/DkLloMHD6aKIpXO1+DLfFXMvaKxMmdla5mXrb7m5CSdpH\/4wVH6wdP5txOHxQnv2vn0inr6yc9+Lj5yYsswzj4vW1f58NEP2MpE1dc+zEsBk3RNgZpqam5ujhydiao+vgZf5qti7hWNlTkrW8s4tuHFsEde+hnd++gxJ+tN9FERHjlZeNY8av3Nc+ni897i1ZknqLe2tdHcHmzNWdlY+tqHFULAcCDUNI4+haRGY+KmguJGXrZt20ZxN1wrAbNq1Sq64447aGxsDGtgbL4JDm3RWDmE+YYrtd7kyJFn6MlXaug74y8En7ia0glEyoJ59J7z30L\/\/r3n0Om\/dFpwGJuaSpoLZ56g3rqvt8oj2MqwhYCR4TrDq76wVn2wefNm45ES3ZlaUxNeAKwEzOHDh6cX7sbZKn8cfP3Zu3dvBWj4\/4rJyUmqr6\/3v6COSrj\/6VcDTz8\/eZL+z49+SlMvvU7PvPx68G\/ep+7M0wMX57\/1dHrfeb9GzW+vCf5fPerzvO\/xIT3qrVwUwdYN25aWllmOJiYm3DgvkJfCjMC4ZsILdXk0Z2BggBoaGqbdR00hxdnqAsbH4LtmbusPf22dIsaX\/gXi4W2\/Srfe+xRNPPfKqd+\/cQibLVfdXo2I8KgJ31S8cskCes\/5ZxRiMWyeclUzLeqtHH2wlWGLERgZrmJekxb\/8ojLokWLpkd2WMDcdNNNtHXr1sit1L4GXwy+oWPfGys1rcI4hkanaP+hFwMyLrYPs59pcVJbQ++rO4PWfqh+Wpg89dRTdHnjm8LdMCQwMyDge701QCBmArYyaH3twwoxAqMW3fJ6lKQn6WwXfdeRGmWJ234dFjdJO5Y4P74GX+arYu61bI2VfuiaKuXXDzxL33rkqIwweWMh7PorFtHkCz+z2qVTNrbmtab6lmArFwOwlWHrax9WCAHDIeNFvDt27JhxoJx+nsvq1asTz4QJH2SnL+JVn3V0dARbs\/nR19vELfhVVcnX4Mt8Vcy9FqWxihImDx58ke5xuDtnxojJG5f+fbypjl7\/+UkrYWJKtyhsTfNbJjuwlYsW2Mqw9bUPK4SASdr2rI+WPPbYY8GBdXfddZdMlGO8+hr8ikKMeJl0YxU+pp6nXH4w9RP6pqNj6vUiqRNh+VyTSy48M7iN+M0pnnkVRy3NtuIFKtALwVYuGGArw9bXPgwCxqC++Bp8g6KLmuRtrPT7c37+i5P0v\/YfcXaeiSq4vgj2w++qpfcvelvwUZ67dEShvuE8L9tK5LGs7wBbuciBrQxbX\/uwQggYDlnaFBJfCaDOirnllltkoowRmIpyTTpsTS2APfuMX6Hbv\/1Pmz6QAAAgAElEQVQUHXJ0d06UMLls8fwZNxD7cJYJOgK5qgy2YCtHQMYzBIwM1xleo86B4Usd1X1Ft956Kw0NDc3YFl2BbGERr0PI+sV+f\/XtR+n7z\/wi8O58Z86CebT87WdSZ\/MFMw5Z80GcmIQDnawJpWw2YJuNm0kqsDWhZG8DAWPPzJsUvgZfIkBKoPzrz0\/S1x7+sdNbh4Opm+AU2DPoMx++cE4KE9OYoSMwJWVvB7b2zExTgK0pKTs7X\/uwwkwh2YWjsta+Bj8LRf324f\/90I+DQ9fyHrimL4D9xGV19NrrMjtzspS3rGnQEchFDmzBVo6AjGdf+7DCCBg+TK6zs5OmpqZmRbCxsXHG9mqZEMd79TX4cSXWL\/x75MhPaffYs5lFii5Ormw8N9idw\/75\/hx0BHI1GWzBVo6AnGfUWxm2vvZhhRAw+vH+6rwXPrNFXebY09MzfX6LTHiTvfoafL3UfJw9T\/k88azdiIoSKEsvPJMuPv8tdPni+dNu09aboLGSq81gC7ZyBOQ8o97KsPW1DyuEgAmfA6Mf9c8Le0dGRih8KaNMmKO9+hj8kQeP0Mj3njEaWVFC5Lffcxat+\/DC6RGUvDFAY5WXYHx6sAVbOQJynlFvZdj62IcxqUIKGN4uffDgQeKRl6Q7jWRCPdtr2YOvFtaOPPgMsXBJelisrGxYQB9bfp74WSdorORqMNiCrRwBOc+otzJsy96HxVEphIDhzOn3Eemi5e6776bR0VGMwGSo1yxctn7rEP3Nvtnritidmv65rePiwHvalE+GLCQmQWPlmuib\/sAWbOUIyHlGvZVhCwEjw3Xaq74Ohg+tY0Gzbds24gsZq3H2i17csgWf17Os2\/HI9DZjvSwsUtZ+sJ7+eOWFwhFNd4\/GKp1RVguwzUouPR3YpjPKagG2WcklpytbH2ZKoTAjMKYZroZdGYLPoy18+eC1f\/foLEQsWm5rv1h8Ssg2NmisbImZ24OtOStbS7C1JWZuD7bmrGwsy9CH2ZRH2RZCwJhe5lhbW5uljLnTFDn4an3L6i8\/PKOcLFo+uvRc+mTzBRWfGjIFjsbKlJS9HdjaMzNNAbampOztwNaemUmKIvdhJvmPs4GAMaBX1OBHTRWp0RY+Z6XoDxoruQiBLdjKEZDzjHorw7aofVje0lZVwPBuow0bNqSWYe3atcGOpGo9RQs+j7rwbqL+PU9OIymTcFGZRmMlV6PBFmzlCMh5Rr2VYVu0PsxVKasqYFQhkqaQXBU0j5+iBX\/pjQ\/MuAeo\/6oltOo9Z+UpYlXSorGSww62YCtHQM4z6q0M26L1Ya5KWQgB46owUn6KEnyeMtLXupRx1EWPERorqRpLuKZBDi3Ygq0gARnXRenDXJcOAsaAaLWDz1NGe374PPXcOT6d27\/ufC\/9h\/edY5D74ppAwMjFBmzBVo6AnGfUWxm21e7DZEpVxZN41bTR2NhYatnm8mWOLF52fv8Ibf6\/p9a7lH3UBSMwqdXdiQE6AicYI52ALdjKEZDxDAEjw7UUXqsV\/PBiXRYvuz5zSWG3RdsGEx2BLTFze7A1Z2VrCba2xMztwdaclY1ltfowmzxmsfVmCkmd5Lt79+6Ag+nOpfHxceru7qaBgQFqaGiIZFit4N9x\/9PUc+dj0yMvPokXLhQaqyxfWbM0YGvGKYsV2GahZpYGbM042VpVqw+zzaetfaEETNS26s2bNxNfLZD26Hcpqemp9vb2xLRK9Ozfvz\/xuoJqBJ+3SV898oi34gUCJq1G5\/scHUE+fkmpwRZs5QjIeK5GHyZTkpleCyNgWLzs2LGDBgcHSZ24aypEokDpgiYOpLo0kj8v0gjMXBAvEDCyX290snJ8wRZs5QjIeIaAkeEaeHV9lYDJuTJss2nTJuro6AgujiyKgOF1L3zOi3oOfP4yb9a8hKsQOgK5LxXYgq0cATnPqLcybCFgZLg6FzDqFuvW1lbq6+ujmpqayJzziA8\/y5YtM1oDozvZu3evCI2pl16nP\/3W87T\/6VcD\/3951Xm0\/IJ5Iu8qgtPJyUmqr68vQla8ywPYyoUUbMFWjoAbzy0tLbMcTUxMuHFeIC9eTyFNTU1FihheuLt9+3bauHEjcWNUlEW8\/XsOTl8PsGLxfNp19SUFqirus4K\/ttwzVR7BFmzlCMh5Rr2VYYsRGBmuM7zmWcQbzl7S7iIepVm5ciU1NTVRUXYh6VNHvm2Xjqs6aKzkvlRgC7ZyBOQ8o97KsIWAkeEq5lUt0NUXBfPLkg7QGx4eDkRN+JEOPouXdSOP0P1PHA9ezduly3CbdN7gobHKSzA+PdiCrRwBOc+otzJspfswmVyney3EFFKe3UaqiPquI7U9uq6uLvUW6yKMwOi7jubC1BGmOdK\/mHkt0BHkJQhxKEcQbCvNFgJGmHh4+ihuNCQuG+GD7PRFvOoz3nEUHmGptoCZi1NHEDDCXyYcEigKGOJQDi\/YyrCFgJHhGulV7STiD3kUZWhoKPaU3EpkSzL4X\/zmBP33vYeCYvSsuoh6Vi2qRJEK8Q40VnJhAFuwlSMg5xn1VoatZB8mk2Mzr4WYQkrKKosZXs8SXstiVjw3VlLBD4++8Jkvc+lBYyUXbbAFWzkCcp5Rb2XYSvVhMrk191pIAaOPwFT7JmpGKRX8z339cfrKPz0VROv2joup49LzzCPngSUaK7kggi3YyhGQ84x6K8NWqg+Tya2518IImKJNG+kIpYJfe+29wWt42\/RcG33hcqOxMv+i2lqCrS0xc3uwNWdlawm2tsTM7KX6MLO3y1kVQsCYHP0vhyDds0Tw\/\/Tvn6A\/v+fwnB19gYBJr3d5LNAR5KGXnBZswVaOgIxniT5MJqd2XgshYOyyXHlr18Gf62tfVATREcjVZbAFWzkCcp5Rb2XYuu7DZHJp7xUCxoCZ6+DrAmauHFoXhRmNlUHly2gCthnBGSQDWwNIGU3ANiO4lGSu+zCZXNp7hYAxYOYy+CxeVn\/5YeJ\/5+raF4zAGFS6nCboCHICTEgOtmArR0DGs8s+TCaH2bxCwBhwcxl8ffTlv\/3+Euq6\/AKDHPhpgo5ALq5gC7ZyBOQ8o97KsHXZh8nkMJvXQgiYpEW8cXcaZStutlQug7\/69ofn3J1HcdTRWGWrjyapwNaEUjYbsM3GzSQV2JpQsrdx2YfZv10uBQSMAVtXwddHX+bSnUcQMAaVzLEJOgLHQDV3YAu2cgRkPLvqw2Ryl91rVQVM+P6juGKsXbs29VLG7AjSU7oKfv+eg9S\/58nghXN58a4ijo4gve5ltQDbrOTS04FtOqOsFmCblVxyOld9mEzusnutqoBR2Z4r58Co6aO5vngXAib7F9Y0JToCU1L2dmBrz8w0BdiakrKzg4Cx4+WVtYvg3\/\/48WD3ET98ZQBfHTDXHzRWcjUAbMFWjoCcZ9RbGbYu+jCZnOXzWogRmHxFkE\/tIvj6ybuYPjoVMzRWcnUXbMFWjoCcZ9RbGbYu+jCZnOXzWjUBo08bLVmyhLq6umhsbCyyNNW+0NFF8Of6vUdRgUVjle\/Lm5QabMFWjoCcZ9RbGbYu+jCZnOXzWjUBky\/blU2dN\/j69NHWP1hCnc1z9+wXPXJorOTqMdiCrRwBOc+otzJs8\/ZhMrnK7xUCxoBh3uDf8M0J+rO9h4I3YfroTeBorAwqX0YTsM0IziAZ2BpAymgCthnBpSTL24fJ5Cq\/10IIGDWd5OsUEnYfRVdUNFb5v8BxHsAWbOUIyHlGvZVhCwEjwzXRKwub6667jj73uc9RQ0NDFXJw6pV5gq9PH332IwtpU+viqpWjaC9GYyUXEbAFWzkCcp5Rb2XY5unDZHLkxmshRmCSisJXCYyMjFBfXx\/V1NTEmp44cYJ6e3tp9+7dgU3S4XfhEZ\/W1tZE\/3mCj8Pr4qOLxsrNlzjKC9iCrRwBOc+otzJs8\/RhMjly47UUAqa\/v58GBweptrY2ttRsw09PTw8pgdLe3k5tbW0z0iih09zcHHymfq6rq4s97TdP8DF9BAHj5qtq5wUdgR0vG2uwtaFlZwu2drxMrfP0YabvqIZd4QUMC5OpqanUEZgwPF3QpIHlKw1GR0dj35E1+PrdRx9qWEBf\/+OlaVmZU5+jsZILN9iCrRwBOc+otzJss\/ZhMrlx57UQAiZpES+PjAwNDVmtgbG9mkBKwOjrX\/jkXT6BF8+bBNBYydUGsAVbOQJynlFvZdhCwMhwnfYaN7WjpnpMX88jL9u2baO0dS3KX9J0k7LJGnw1fcR+sH16dgTRWJnWans7sLVnZpoCbE1J2duBrT0zkxRZ+zAT39W0KcQIDAOImioyERdx8EymnpRoYh9Ji4Q5+Pqzd+9eo5i1bp+kqZdep7ozT6fdn6g3SjOXjCYnJ6m+HlwkYg62ElRP+QRbsJUj4MZzS0vLLEcTExNunBfISyEETNKUj+kupDDT8fFx6u7upoGBgcjpJ1Pxwn6zqFd9\/Qsub4yu8fhrS64lAFuwlSMg5xn1VoZtlj5MJiduvZZCwJjsQgpjYeETl85k55HuL0vwsX06vaKisUpnlNUCbLOSS08HtumMslqAbVZyyemy9GEyOXHrtRACJrz+RS9i2gJbZavvOkoTKCbTS3kFjL7+5djNH3EbNU+8obGSCyTYgq0cATnPqLcybCFgZLhOe+URk\/Xr18\/YccTTQJ2dnbRlyxZqampKzEH4IDt9Ea\/6rKOjg+Juvk668TpL8HH7dHqFQWOVziirBdhmJZeeDmzTGWW1ANus5DACI0POwiuLmDVr1sxIMTw8nCpeLF6RydRWwOjrX3pWXUQ9qxZleq\/vidBYyUUYbMFWjoCcZ9RbGba2fZhMLtx7LcQUkvtiufVoG3z9\/Bdsn46PBRort\/VU9wa2YCtHQM4z6q0MW9s+TCYX7r0WQsDoUzxpU0XuEaR7tA2+vv7lwOcvo4W189JfMgct0FjJBR1swVaOgJxn1FsZtrZ9mEwu3HsthICxPTnXPYZkj7bBX3rjA8TTSCxcWMDgiSaAxkquZoAt2MoRkPOMeivD1rYPk8mFe6+FEDBcLNPdRu4RpHu0Cb6+\/mXF4vm06+pL0l8wRy3QWMkFHmzBVo6AnGfUWxm2Nn2YTA5kvBZCwCTdhcTFTtohJINlpleb4OvrX7CANzk6aKzkai\/Ygq0cATnPqLcybG36MJkcyHgthICRKZo7rzbBxwF25tzRWJmzsrUEW1ti5vZga87K1hJsbYmZ2dv0YWYei2EFAWMQB5vgqwW8WP+SDhaNVTqjrBZgm5VcejqwTWeU1QJss5JLTmfTh8nkQMZr1QSMvnA37nA5VeSyTCFh\/YtdJUVjZcfLxhpsbWjZ2YKtHS8ba7C1oWVuCwFjzso7S9Pg4wA7u9CjsbLjZWMNtja07GzB1o6XjTXY2tAytzXtw8w9FsOyaiMw4eKH70NKuh+p0uhMg48D7Owig8bKjpeNNdja0LKzBVs7XjbWYGtDy9zWtA8z91gMy8IImKgLFtU0U3t7O7W1tVWNmGnw\/\/Z7R+izOx4J8okTeNPDhcYqnVFWC7DNSi49HdimM8pqAbZZySWnM+3DZN4u57UQAibpIDu+H2lkZIT6+vqopqZGjkSCZ9PgYwGvXXjQWNnxsrEGWxtadrZga8fLxhpsbWiZ25r2YeYei2FZCgHDozODg4NUW1tbFWqmwVc3UOMAO7MwobEy45TFCmyzUDNLA7ZmnLJYgW0WaulpTPuwdE\/FsiiEgEla71KEE3pNgo8FvPYVG42VPTPTFGBrSsreDmztmZmmAFtTUnZ2Jn2YncdiWBdCwDAKnipav349DQ0NUUNDQ0BnfHycOjs7acuWLVTNSx5Ngo8FvPYVGo2VPTPTFGBrSsreDmztmZmmAFtTUnZ2Jn2YncdiWBdGwCgRs2bNmhlkhoeHqypeODMmwddP4MUN1GaVG42VGacsVmCbhZpZGrA145TFCmyzUEtPY9KHpXspnkWhBEzx8JzKkUnwsYDXPnporOyZmaYAW1NS9nZga8\/MNAXYmpKyszPpw+w8FsMaAsYgDibBX3rjA8TrYLCA1wDoGyZorMxZ2VqCrS0xc3uwNWdlawm2tsTM7E36MDNPxbKCgDGIR1rwcYWAAcQIEzRW2biZpAJbE0rZbMA2GzeTVGBrQsneJq0Ps\/dYjBQQMAZxSAs+diAZQISAyQYpYyp0BBnBGSQDWwNIGU3ANiO4lGRpfZjMW+W9zkkBo7Zt7969OyC8du1a6unpiaWdFnws4M1WUdFYZeNmkgpsTShlswHbbNxMUoGtCSV7m7Q+zN5jMVLMSQHDB+Pxw6LF5LqCtOBfPfIIjTx4JPCJHUjmFRuNlTkrW0uwtSVmbg+25qxsLcHWlpiZfVofZualeFaFETDqzJepqalZlBobG0VP4tUFTVSI0oKPHUjZKjYaq2zcTFKBrQmlbDZgm42bSSqwNaFkb5PWh9l7LEaKQggYNaVTV1eXOJUjgSzpHib1vrTg4wqBbJFBY5WNm0kqsDWhlM0GbLNxM0kFtiaU7G3S+jB7j8VIUQgBYyIiJHDxyMu2bduotbU18bJIDr7+7N27d\/rHqZdep9btk8HPa98\/n\/7LB+ZLZNVLn5OTk1RfX+9l2apdKLCViwDYgq0cATeeW1paZjmamJhw47xAXgohYNQITEdHR1VO3WUhw1NXcTdeJ6lXXCGQvTbjr63s7NJSgm0aoeyfg212dmkpwTaNULbPMQKTjZtxKr4LqVq3TvP6m+7ubhoYGJi+h0nPeFLw9R1Iuz5zCa14J0ZgTIOOxsqUlL0d2NozM00Btqak7O3A1p6ZSQoIGBNKGW3UFNLY2FikB+lFvGniKSn4G7\/+OP3FPz0V5Bs7kOwqABorO1421mBrQ8vOFmzteNlYg60NLXNbCBhzVoW31HcdmSwgTgo+diBlDzcaq+zs0lKCbRqh7J+DbXZ2aSnBNo1Qts8hYLJxK2Sq8EF2Jot44xZA4Q6k7CFGY5WdXVpKsE0jlP1zsM3OLi0l2KYRyvY5BEw2blapdu7cSRs2bAjSDA8P06FDh2h0dDRxh5DVCzIaJwUfW6gzQiUiNFbZ2aWlBNs0Qtk\/B9vs7NJSgm0aoWyfQ8Bk42acSu0E4sW069atC86D4bUvvb29VI3zYfSMxwVf34HUs+oi6lm1yLi8MISAkawD6Ajk6IIt2MoRkPEMASPDNfCqnwOzZMkS6urqCgRMU1MTpS2wFczWtGsTAYMdSPaRQEdgz8w0BdiakrK3A1t7ZqYpwNaUlJ0dBIwdLytrCBgrXN4Yo7GSCyXYgq0cATnPqLcybCFgZLhOe+X1L7zeRZ9CUqMx7e3t1NbWJpyDePdxwVc7kIJRpJs\/UrX8lfXFaKzkIge2YCtHQM4z6q0MWwgYGa4zvPJ00Zo1a2b8bvPmzVUVL5yZNAGzsHZecAYMHjsCaKzseNlYg60NLTtbsLXjZWMNtja0zG0hYMxZeWcZF3zsQMoXajRW+fglpQZbsJUjIOcZ9VaGLQSMDNdSeI0K\/uFjrxKfAcNPx6Xn0e0dF5eiLEXKJBoruWiALdjKEZDzjHorwxYCRobrDK\/6OTDqAz4PhncjVfOJCj62UOePCBqr\/AzjPIAt2MoRkPOMeivDFgJGhuu0VxYvO3bsoMHBQaqtrQ1+r3YnFXERrz4Cgy3U2SoHGqts3ExSga0JpWw2YJuNm0kqsDWhZG8DAWPPzDiFvo06PNpS1HNgcAu1cXhjDdFY5WeIERg5hmALtpUnIPNGCBgZrjNGWtThdfqriipgrh55hEYePHIq\/9hCnal2QMBkwmaUCGyNMGUyAttM2IwSga0RJmsjCBhrZHYJWKisX7+ehoaGqKGhYYawKeIUEm6htotvlDUaq\/wMMUogxxBswbbyBGTeCAEjw3WGUBkbG0t9C9+PdNddd6XauTSICj5uoc5PGAImP0N0snIMwRZsK09A5o0QMDJcS+E1HHx9Ae+KxfNp19WXlKIcRcskBIxcRMAWbOUIyHlGvZVhCwEjw7UUXpMEDG6hzh5CNFbZ2aWlBNs0Qtk\/B9vs7NJSgm0aoWyfQ8Bk42aVKuocmCJeJaCfAcNXCPBVAnjsCaCxsmdmmgJsTUnZ24GtPTPTFGBrSsrODgLGjpe1dZnOgeHdR7wLiR+cAWMd6ukEaKyys0tLCbZphLJ\/DrbZ2aWlBNs0Qtk+h4DJxs0oVdnOgcEZMEZhTTVCY5WKKLMB2GZGl5oQbFMRZTYA28zoEhNCwMhwDbyWTcCs\/vLDxNNIQd5xBkzmmoHGKjO61IRgm4ooswHYZkaXmhBsUxFlMoCAyYTNPFHeKSQlgtRW7NbWVurr66OamprITOjrbdJsw8HHGTDmcU2yRGPlhmOUF7AFWzkCcp5Rb2XYQsDIcJ3hNesi3hMnTlBvby81NzdTW1sbqZ\/r6uqIT\/cNP\/rpvixwOG2cLacNBx9nwLipDGis3HCEgJHjCLZgW1kCMm+DgJHhKuaVxdDo6GjkKEz4syTbsIDBGTDuQgYB445l2BPYgq0cATnPqLcybCFgZLiKeU0SJVEjMGr0JipDevB1AXPzx95F\/\/myOrEy+O4YjZVchMEWbOUIyHlGvZVhCwEjw1XEq1oPk3SH0vj4OHV2dtLU1BQNDw9T+BZsPWN68PUzYHCIXb7wobHKxy8pNdiCrRwBOc+otzJsIWBkuDr3qta\/sOO4RbzhBcP9\/f1BPqLWy\/DvOfjqeW3h5fTKsk8GP\/7lVefR8gtwiF3WIE5OTlJ9fX3W5EiXQABs5aoH2IKtHAE3nltaWmY5mpiYcOO8QF5OO3ny5MkC5SdXVkzES3jBL7+QR2O6u7tpYGBg+ibsuBEYnAGTK0QzEuOvLXcsw57AFmzlCMh5Rr2VYYsRGBmuzrym7TxSL8orYPgEXj6Jlx9cI5AvfGis8vFLSg22YCtHQM4z6q0MWwgYGa7OvPI0EK9nSTr7Rb0sagopKa0efJwB4yxkhMbKHUuMwMixBFuwrRwBmTdBwMhwdeI1fIidctrY2EiDg4PBYXZ81ktHR8f0Yl0WPNu2bQtMbQ6yg4BxErLACQSMO5boZOVYgi3YVo6AzJsgYGS4lsKrHvzaa+8N8rxi8XzadfUlpch\/UTMJASMXGbAFWzkCcp5Rb2XYQsDIcC2FVxV8\/QyYjkvPo9s7Li5F\/ouaSTRWcpEBW7CVIyDnGfVWhi0EjAzXUniNEjA4AyZ\/6NBY5WcY5wFswVaOgJxn1FsZthAwMlxL4VUFXz\/EjkdfeBQGT3YCaKyys0tLCbZphLJ\/DrbZ2aWlBNs0Qtk+h4DJxs2LVFECZtdnLqEV75zvRfmqVQg0VnLkwRZs5QjIeUa9lWELASPDtRReVfBxiJ3bcKGxcstT9wa2YCtHQM4z6q0MWwgYGa6l8KqCj0Ps3IYLjZVbnhAwcjzBFmwrQ0DmLRAwMlxL4VUFH2fAuA0XBIxbnuhk5XiCLdhWhoDMWyBgZLiWwisEjEyYIGBkuLJXsAVbOQJynlFvZdhCwMhwLYVXFfylNz5AfBYMDrFzEzY0Vm44RnkBW7CVIyDnGfVWhi0EjAzXUnhVwccpvG7DhcbKLU9Mc8jxBFuwrQwBmbdAwMhwLYVXDv63v\/9D4hEYfnCInZuwQcC44YgRGDmOYAu2lSUg8zYIGBmupfDKwf\/rf3yIVn\/5YQgYhxGDgHEIM+QKbMFWjoCcZ9RbGbYQMDJcS+E1LGAOfP4yWlg7rxR5L3Im0VjJRQdswVaOgJxn1FsZthAwMlxL4ZWD\/6Wdo8TnwPCDU3jdhA2NlRuOmOaQ4wi2YFtZAjJvg4CR4VoKrxz8tX9xD\/XveRICxmHEIGAcwsQUkhxMsAXbihGQeREEjAzXUnjl4P\/Ol75JIw8eCfKLKSQ3YYOAccMRowRyHMEWbCtLQOZtEDAyXEvhlYP\/3uu+Rvc\/cTxY+8ICBk9+AhAw+RnGeQBbsJUjIOcZ9VaGLQSMDNdSeIWAkQkTGisZruwVbMFWjoCcZ9RbGbYQMDJcS+GVg3\/mJ\/8Wp\/A6jhYaK8dANXdgC7ZyBOQ8o97KsIWAkeFaCq8c\/OMfHQzyimsE3IUMjZU7lmFPYAu2cgTkPKPeyrCFgJHh6szrsWPHqKuri8bGxgKfra2t1NfXRzU1NZHv2LdvH61Zsyb4rLGxkQYHB6m2tjbSdtF7308v\/XZ\/8FnHpefR7R0XO8v3XHaExkou+mALtnIE5Dyj3sqwhYCR4erE64kTJ6i3t5eam5upra2N1M91dXXU09Mz6x3j4+PU2dlJW7ZsoaamJtq5cyeNjo7GCh5dwOAMGCchC5ygsXLHEiMwcizBFmwrR0DmTRAwMlzFvCaJEv7s4MGDkeImKkML3\/879JMV3cFHPPrCozB48hOAgMnPMM4D2IKtHAE5z6i3MmwhYGS4inmNEzDh0RqTDNR\/+OP0yrJPBqYYgTEhZmaDxsqMUxYrsM1CzSwN2JpxymIFtlmopaeBgElnVBgLtR6mvb09mFLSHyVgVq1aRXfccUewZiZtDUzd7\/5XevXdqwM3Z9w\/QPft\/HJhylrmjExOTlJ9fX2Zi1DYvIOtXGjAFmzlCLjx3NLSMsvRxMSEG+cF8nLayZMnTxYoP7mzogQKO4paxKs+P3z48PTC3f7+fpqamopdA6MLGJzCmztE0w7w15Y7lmFPYAu2cgTkPKPeyrDFCIwMV6de08QLvyxqCokX9XZ3d9PAwAA1NDTMytN5f3Ajvbbw8uD3x27+iNM8z2VnaKzkog+2YCtHQM4z6q0MWwgYGa7OvKbtPNJfxCMuixYtmp5eYgFz00030datWyO3Up\/7h1+h189+F64RcBatU47QWDkGqrkDW7CVIyDnGfVWhi0EjAxXZ17TpoH0F\/EZMGyvzn7h\/+cnass1\/x4CxlmYZjhCYyXDFeJQjivYgq0sARnvEDAyXJ14DR9ip0kymDcAAAzPSURBVJyqxbl8mB2fE9PR0RGc+8KPfpBd2qF3Z\/\/R39Ev\/s3ZOIXXSbTedAIB4xgoRmDkgIIt2FaEgMxLIGBkuJbCa+219wb5xDUCbsMFAeOWp+4NbMFWjoCcZ9RbGbYQMDJcS+FVCRhcI+A2XGis3PKEgJHjCbZgWxkCMm+BgJHhWgqvSsD0rLqIelYtKkWey5BJCBi5KIEt2MoRkPOMeivDFgJGhmspvELAyIQJjZUMV\/YKtmArR0DOM+qtDFsIGBmupfCqBAzuQXIbLjRWbnlimkOOJ9iCbWUIyLwFAkaGaym8KgGDe5DchgsCxi1PdLJyPMEWbCtDQOYtEDAyXEvhVQkYXCPgNlwQMG55opOV4wm2YFsZAjJvgYCR4VoKrxAwMmGCgJHhyl7BFmzlCMh5Rr2VYQsBI8O1FF6VgME9SG7DhcbKLU+MEsjxBFuwrQwBmbdAwMhwLYVXFjALa+cRTyHhcUcAAsYdy7AnsAVbOQJynlFvZdhCwMhwLYVXCBiZMKGxkuGKKSQ5rmALtrIEZLxDwMhwLYVXFjC4RsB9qCBg3DNVHsEWbOUIyHlGvZVhCwEjw7UUXiFgZMKExkqGK0YJ5LiCLdjKEpDxDgEjw7UUXlnA4B4k96GCgHHPFCMwckzBFmzlCci8AQJGhmspvLKAwT1I7kMFAeOeKTpZOaZgC7byBGTeAAEjw7UUXiFgZMIEASPDFdMcclzBFmxlCch4h4CR4VoKr+f+4Vfols9+NJhGwuOOAASMO5ZhT2ALtnIE5Dyj3sqwhYCR4VoKr74Gv9rw0VjJRQBswVaOgJxn1FsZtr72YaedPHnypAwyf7z6GvxqRwiNlVwEwBZs5QjIeUa9lWHrax8GAWNQX3wNvkHRRU3QWMnhBVuwlSMg5xn1Voatr32YNwLm2LFj1NXVRWNjY0ENaG1tpb6+PqqpqUmsEePj49Td3U0DAwPU0NAQaetr8GW+KuZe0ViZs7K1BFtbYub2YGvOytYSbG2Jmdn72od5IWBOnDhBvb291NzcTG1tbaR+rquro56entgIK7v9+\/fT0NAQBIzZd8GZla9fKmeAcjgC2xzwUpKCLdjKEZDx7Gud9ULARIV8586dNDo6mjgKs2\/fPurv7w+SYwRG5ouT5NXXL1XlSc5+I9jKRQFswVaOgIxnX+vsnBUwPOW0adMm6ujoCEQMBIzMFwcCpvJc+Y2+NljVoTnzrWArFwWwlWHrK1cvBYxaD9Pe3h5MKcWN0PDvly1bZrQGRqZawSsIgAAIgAAIyBOYmJiQf0mF3+CdgFHrWphj3CJeXri7fft22rhxI01OTqYKmArHBK8DARAAARAAARBIIeCVgDERL8yDp4xWrlxJTU1NZLILCbUIBEAABEAABECgWAS8ETCmO4\/C2631cAwPDweiBg8IgAAIgAAIgECxCXgjYHhUZWpqyujsFz0kGIEpdgVF7kAABEAABEAgioAXAiZuVKWxsZEGBweDw+z4nBjecRQeYYGAwRcDBEAABEAABMpHwAsBUz7syDEIgAAIgAAIgEAeAhAweeghLQiAAAiAAAiAQFUIQMAkYOd1Ndu2bQsssMA3W\/3kKbrOzs5gfVLa\/VQ6b74GIul6h2y58SuVDVtV8vC1G34RcVMaG67h6Wu0E8kxsGGr26I9yFe31fc+ahlFPs\/VTQ0BE8NfXTPAa2gee+yxYOs1\/39tbW11I1ait+ud5erVq2fcVxUuRvjqB\/55x44dYB4Tbxu2ugvmumHDBtq8eXPsIY8lqmLOs2rDNbzzEevpksNhw1YJQ77Ljtctoj3IXtUV9927d3v3hzgETEy9UHck8RfIV\/Wa\/SthljLcoLMoHBkZMdophs4g\/S9Z\/RZ1E7bcKVx33XV0\/PhxSjql2iy6flrZ1Fm2vemmm2jr1q34w8agOtiy1es32gMDwBEmahRr+fLldPjw4eByY5+OCoGAiQh63O3W6rbrbFVp7qXSR7F45Cr8cxIRNFjJ9SULWxbll156KX3jG9+Yvrl97tVKd1xNLowF3zcJ2NTZqBGYtMt5wXo2gaeffjr4Je\/E7erqgoCZC5UkasSFG\/9FixZh2N2iAoRHBWz+Ys16ro9F9kptastWXZ9x7bXXBpeYQoxHh9+GKwuYgwcPBo6wVi7962TDlr3pUx9r164NOl882QiEBWE2L8VLhRGYhBEYfcETBIx95bVtsNQbuGO49dZbsYg3AbkNW+4IvvSlL9EnPvEJqq+vT1yLZB9lv1LYcFXridTCXU67fv161NuYKmHDVk19bNmyJZjysBm99atGuikNBIwbjqXwgikkN2GyGTKGeLFjbsOWbe+7777gL1jsQkrmbMM1PIUEtmBr9y2unDUETOVYF+JN+ogLFvFmC0l4yihtoSl2GphztmGrb0\/X34Bh+dm8bbiG6zPaieT6a8MW4tC8LTCxhIAxoeSRDbZR5w+mzbZJDL\/b8bZhq3vGKEEyZxuu4U4B0xzu2EZNIWF6zq6N0K0hYLKzK21KHGSXP3RJB1epRZA8tRE3SoCDweJjYMoWAsauHttw1Q+yw2Fr6Zxt2LIgXLNmTeAUbNPZJllAwOTjh9QgAAIgAAIgAAIg4IwAdiE5QwlHIAACIAACIAAClSIAAVMp0ngPCIAACIAACICAMwIQMM5QwhEIgAAIgAAIgEClCEDAVIo03gMCIAACIAACIOCMAASMM5RwBAIgAAIgAAIgUCkCEDCVIo33gAAIgAAIgAAIOCMAAeMMJRyBgHsCTzzxBC1YsID4Nm+Th897eOGFF2jx4sUm5pls1Jk9jY2NNDg4aJy3stwwrspXqbNHKv2+TEFHIhAoIAEImAIGBVkCASZge7JrJQ6ryiNC8qStZI1gQcFPJW8\/LgubSsYB7wKBNAIQMGmE8DkIVIlAEQWMbZ50dGXppCFgqlTh8VoQsCQAAWMJDOYg4JKAfhQ9+1XTMo899tj0Mer8e3WlQvjKBTXNcdZZZ1FXVxeNjY0F2VMXNaq7fXbv3h383mTaR3+HPo3CVz9s2LBhuvibN2+mtra2WTji7JSAWb16Nd1www1BuvA0jX58vHKs3sOsrrvuOvrQhz4UpFdlOXr0KHV2dtLU1FSQ5Atf+ALt2rWLBgYGqKGhIfhdXJmiYhkWMPzzyy+\/HPynOCZdhBm+iJDfEfW7Moo7l3UfvkAgLwEImLwEkR4EMhKIulhR7zzDox1xN\/Ty6\/v6+oj9sYjhqY+mpiZS4qi9vX1aaCTd+K3yo\/zV1NQEHe+tt95KQ0NDgRhIG4EJ2+uX8rHIYqGxfPnyIL\/K\/44dO4K1NCxEuru7ZwgP3Z8SaQsXLpxOHy6j+vm5554L8lxfX0+9vb2BUFJTQmkXh0YJmG3btpEupJizzlWvAhAwGb8QSAYClgQgYCyBwRwEXBGIEhi67zSxEP7LPixgOP3IyMh0Z8\/2SbdRR03xhO2T8pR203X4hmHOT9q0kv65EjBhQTY6OjqjjLpA4XfcdNNNtHXr1hmLjZOmiaIEDI\/uKNHFPpM4QMC4+obADwgkE4CAQQ0BgSoS0KdbwtM7cZ1keJqltbU1cgQmPJWjFzNq+ifuffqt4Ukdd9oi4iixEvW78LRaeJpMjTBxeaKEiO6TR3XUjcbhMMdNA0UJGE6rL+pNEl4QMFX8QuHVc4oABMycCjcKW1QCeqetr4PhzlRtVVaCJLwuRY1AhEdgOG145CCp\/HHiJGlaS\/eXV8CwL7WWRQmsqBEYGwHz0EMPkZqiMt2KDgFT1G8J8gUCMwlAwKBGgECBCOgiQI0wsIDh9SK8lqO5uXnGwlldpIQFTNJ6l6giV2IKKbzGRX8ni42k6SA1haQLmKjRDn0KiUdg1q9fP72GxyTUElNIaWIybSrNJN+wAYG5RgACZq5FHOUtDIGoNTD6KIi+qDVqMaoakVFTSFwwXeQo\/7ygV01\/RK1DUUBcLeLVRzz0dTHLli2btUg3LGD0tCqvnD9ekBslYEwX8bIPtYYlbe1R3kW8aopP7RxT5dAXL4crIQRMYb6WyEiJCEDAlChYyKp\/BFTnprYA69ND+hZonlK54oorZmyVZuFy5ZVX0vXXXz89whAWNWpURm2vZoKqY42jmbTl2HRhcdR2a5M1MOF3b9myJVjnwgt3Vfn1ERguQ5hheBt1eCs5p4nbAq5GvfhfJfqitlHr6aPKpa8\/4jgtXbqUDhw4EIiosNBUZQiPTvlX21EiEHBLAALGLU94AwEQqDIBFhRRO49Ms2WyBsbUl6kdRmBMScEOBN4kAAGD2gACIFBaAuF1Pmq0RT\/3xbZwEDC2xGAPAtUhAAFTHe54KwiAgCMC4dOJk07JNXll+HLFO++8M0gmdTcSLnM0iQpsQGA2gf8PBN88g4boy7oAAAAASUVORK5CYII=","height":0,"width":0}}
%---
