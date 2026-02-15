preamble;
model = 'mpc_psm';
%[text] ### Global timing
% simulation length
simlength = 1.25;

% switching frequency and tasks timing
fPWM = 2.5e3;

fPWM_AFE = 2*fPWM;
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

fPWM_DAB = 6*fPWM;
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
l1 = Kd(2) %[output:75480c5c]
l2 = Kd(1) %[output:3356c104]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:4a13890b]

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
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:3ae14074]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:7858ff8a]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:8b993c89]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:4f0a8057]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid_emu_data.w_grid;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:2be5a5f7]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:0aedf096]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:93d9fff2]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:09c73bc1]
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
run('n_sys_generic_1M5W_pmsm'); %[output:2788be2b] %[output:1ed1daa1]
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
kg = Kobs(1) %[output:3b2d4c40]
kw = Kobs(2) %[output:6142afe8]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:513fbeb6]
luenberger_l2 = Klo(2) %[output:72f2672f]
luenberger_l3 = Klo(3) %[output:5b8b0b72]
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
k_kalman = 0;
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
lithium_ion_battery = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_inv); %[output:6705b52e]
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
%   data: {"layout":"onright","rightPanelPercent":37.9}
%---
%[output:75480c5c]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  36.521945502464568"}}
%---
%[output:3356c104]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.149016195397013"}}
%---
%[output:4a13890b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.149016195397013"],["36.521945502464568"]]}}
%---
%[output:3ae14074]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:7858ff8a]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:8b993c89]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:4f0a8057]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.147445399070218","24.336274188752061"]]}}
%---
%[output:2be5a5f7]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:0aedf096]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:93d9fff2]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:09c73bc1]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.147445399070218","24.336274188752061"]]}}
%---
%[output:2788be2b]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:1ed1daa1]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:3b2d4c40]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.019856330123859"}}
%---
%[output:6142afe8]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.825051459217296"}}
%---
%[output:513fbeb6]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.237172904101351"}}
%---
%[output:72f2672f]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     1.436150968933706e+02"}}
%---
%[output:5b8b0b72]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -1.779725838133451e+02"}}
%---
%[output:6705b52e]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10XsV55x9a2qCGEFt81AjFMTUykEJlzJIKAXGyLvW2XZke2q4k72lSRc26DU5OF3wkOU44JRAsqTZbykfjsorq9FSy2w1O7CYNpS4hZYUPxGBtaQgIhG2EMMEYAhSThMR7nmtGjK7ux8zced733vv+7zkcI73PPHfm98w789d8nnDs2LFjhAcEQAAEQAAEQAAECkTgBAiYAkULWQUBEAABEAABEAgIQMCgIoAACIAACIAACBSOAARM4UKGDIMACIAACIAACEDAoA6AAAiAAAiAAAgUjgAETOFChgyDAAiAAAiAAAhAwKAOgAAIgAAIgAAIFI4ABEzhQoYMgwAIgAAIgAAIQMCgDoCAJwJHjhyh7u7uwNvQ0BDV19d78jzXzcTEBHV1ddHFF19M\/f39VFdXJ\/Yu3XEly2hSIMXhU5\/6FLW3t5skybXNwMAAbdmyJcjjmjVrqLe31yq\/27dvp\/Xr19PGjRtzyYPLt2fPHvHvhxU0GBeWAARMYUOHjOeNQCU79zgBwx3E8uXLqaWlRQRPXBml3xtXGJcOkTvQ+++\/30ocuKSxDQC\/Y\/Xq1TPJyihgyiY4bWMMe78EIGD88oQ3EKgKgaNHj1JfXx\/t2rWLRkZGKiZgeOSnEu+Ngqo6w7a2NmMxokYobMSBSxqXSqAEjE3ewu\/J+wiMqqcHDx7EKIxLJUGaWQQgYFAhcklAH0rnDOpD4vrow9KlS+nGG28MysAdmT6dokYLxsfH53yu+7jqqqvoD\/\/wDwObhoYGGh4epqampkguulAI24dHJ\/hzNaW0YsUKuuWWW6i5uTlouPWOP80PT0Wp9+7duzfIHz9qCun666+nz3\/+84F4UU+YBf9eMdUFjuo0dXvVCSpfeoeql\/H222+nwcHByPdOTU0F+Zuenp7Jkx5DnSMzv+OOO+hLX\/oSqfIx\/zBrxU5NzUV11iqu+ntVecPlUrFubGycEWFhfjt37gymZNSj14+wvzThGK6PSb6S6mHSSI3OhPOs8h4WReHvl85W+bj22mtp9+7dxN8fFTu9zPw79Q49tmlcouphLhshZCr3BCBgch+i2spguNPSS68a4ahOKtzxsB8WD0q8hD+P6mCTOn\/+LC5vqrM59dRTZ62BUQJGzwMLhSjBoYuYsB9fAibqL\/xwZxLu2OK48u\/jBExPTw+tXbt2DvskwcCfnX766fTiiy8GAi1KVPA79Y42nPe4eqHe+8gjj0SKkbvvvntm3Yle3\/QOOixgwr7U53EiJqnOcpoDBw7ECiU9T2HxEhaZSjxcccUV9K\/\/+q+zGo8oERL1\/QoLELaJyiP\/Xr0nzbfOJe+jRLXV4ha7tBAwxY5f6XKvGmj9L1DV+HNh9dEH\/itbNYx6B6E3tvpfnnqHxyJBjRAoH+rd4b\/0FeSoz\/XG+Morr4wVMPpfqLZ+0gQMjzrxkzaVkzRCxKNCL7300hwm+qgBc1qyZMmsMppMIYWntxR7FU8ebQnHnfPC60GiRoaY5apVq4Ly6iM2JgubTaaDwjbhnxUTJbY4\/2nvVnUvqjzqdyx0ucxxU0g6R1WfwjG99957AyEUNaIS5zc8CqdGnXQfSe9WIzSq\/qdx8TFVVrqGDwVyIgAB44QNiaQIxP11pjoAbriXLVsWuQNHt9m\/f3\/kX9Wcb90H\/9WvdgylLcJN+8sxTiDoDTq\/39aPLwGjv5vFCD96hxnXsegd+Cc+8QljARM1YhX1Xs5HeIosboSDbbkjVvnQ2Ua9LzyVliRg4qbOwmmSRlOixG+4bGp6MiyElGiLExpp9VOPr+4jLq7h0RzFSgmYuKlDfYedXpfV91KfvlPthM4latpSqj2B33ITgIApd3wLV7pKCxh9G3JaB2ErPBh+1LZqUz965xzu7Ni3vo3aZASGbfSFr\/wzb9kNj0CFO1BbARPmGB6lCQsnXwJGVfY44cQ7s6IETHgqKm0EpggCJmrET8U1XP\/iRmB0H3HfDQiYwjWxpcowBEypwln8wrhOIYWnOtSagri\/ZqOG\/NMETNLUjz4qwFHgv1LjBIypHx6aD4sLNbUWFjAsEkwWR4Y7d32EIjwNxx1+2hQSjw6lCYCwD9cpJL12x41qhL8BYTESHo1QZdZH4lR5VN0Jp4maQkr75klPISmxq0au4gRM1MiVYhQegYlbdB2evkqaQorigimktNqCz00JQMCYkoJdRQhIL+JNEgBpAiYpb1HrQ+IETJofHm5X61nC0E0EDKeJ2oWkfIV3kugHwNks4lVTCXoafu\/VV18djA5FPcwpqnymi3jZpxJ1YeEUt8BVT6Pb8DtvvfVWuummm+YsOOY0YQHDv4tbEKzKmiaYo6ZX0kbAdI5xZUwSH7pg+PSnPx1bt5J8cB6iFveaLuLVuaSNQFakocFLSkEAAqYUYSxfIUy3UetboNO2UUctDLaZQmLKSdMTaYtk9ZN5k\/zwe8Jbbj\/3uc\/Rvn37ZhatRo3A6J1b3EJk3Xd4bU6UwNE7cj0t\/78SMFHvveuuu2adKMuH6+nrbVy2UetCRO9Qo7bYx23fDnPV1+SwT+a2adMmWrduXYBDH0lTu8nitmWnnd+StI2a32U6MhG3doVH4aLEQdyoEzOK2sIeNYoTJ3759+GTf+PWEikfJiOF5WvRUCIJAhAwElThU5RA2o4P0ZfDeWYCUVNV4Z1mcefw6C93Ocguc+Zr1IEuOJVQi9qZlIYHB9mlEcLnNgRKKWC4YeOzKPiQraiGMOmAMxt4sK0OAQiY6nD39dakKbSkqa+o97tcJeCrHLXmJ2oKiRmkHf4YJTrLcndVrdWBvJW3dAImbXGf+ry1tTW47Ez9zF9C24vT8hbMWskPBEzxIx3+I4JLZCteOA3u1qlsXQhP7dqIF84pBGdl41X2t5VOwPB8L39J+IkbgQkHlf+yGBsbq+itvmWvWCgfCIAACIAACEgSKJWA4b\/qbrjhBurs7AxEDASMZNWBbxAAARAAARCoHoFSCRgeSeGHT4RMWgOj41ZD2R0dHcGUEh4QAAEQAAEQAIH8EyiNgOG58K1bt9KGDRuIL+ozETBq\/QuHSb\/FOBy2X\/qlX8p\/JJFDEAABEAABEIggwLeKn3322aVjUxoBw1NGfNYEnx6atguJo2gqXtiWBczk5GTpgl\/tAoGrXATAFmzlCMh5Rr2VYVtWrqUQMFE7GlQ1iLre3nbnUVmDL\/NVMfcKruasbC3B1paYuT3YmrOytQRbW2Jm9osu+CDtf+whM+MCWZVCwIR5p43A8GgNn0KZNG2k+8SXSqZGg6sMV4waynEFW7CVJeDf++jDh+ia0cfpyC0f8e+8yh5rQsCoERfenbRkyZLghmB1LLjin3T0OjpamVoKrjJc0cnKcQVbsJUl4N87BIx\/poXyiI5WJlzPPPNMKReWydCy8wq2drxsrMHWhpadLdja8TKxhoAxoVRiGwgYmeCisZLhyl7BFmzlCMh5Rr31zxYCxj\/TQnmEgJEJFxorGa4QMHJcwRZsZQn49w4B459poTxCwMiECwJGhis6WTmuYAu2sgT8ex+4Zz8N3PMMFvH6R1sMjxAwMnGCgJHhik5WjivYgq0sAf\/eIWD8My2URwgYmXBBwMhwRScrxxVswVaWgH\/vEDD+mRbKIwSMTLggYGS4opOV4wq2YCtLwL93CBj\/TAvlEQJGJlwQMDJc0cnKcQVbsJUl4N87BIx\/poXyCAEjEy4IGBmu6GTluIIt2MoS8O8dAsY\/00J5hICRCRcEjAxXdLJyXMEWbGUJ+PfO1wjwVmpcJeCfbSE8QsDIhAkCRoYrOlk5rmALtrIE\/HuHgPHPtFAeIWBkwgUBI8MVnawcV7AFW1kC\/r1DwPhnWiiPEDAy4YKAkeGKTlaOK9iCrSwB\/94hYPwzLZRHCBiZcEHAyHBFJyvHFWzBVpaAf+8QMP6ZFsojBIxMuCBgZLiik5XjCrZgK0vAv3cIGP9MC+URAkYmXBAwMlzRycpxBVuwlSXg3\/uqOx6lB55+BbuQ\/KMthkcIGJk4QcDIcEUnK8cVbMFWloB\/7xAw\/pkWyiMEjEy4IGBkuKKTleMKtmArS8C\/dwgY\/0wL5RECRiZcEDAyXNHJynEFW7CVJeDfOwSMf6aF8ggBIxMuCBgZruhk5biCLdjKEvDvHQLGP9PceJyYmKCenh4aHBykpqamyHxBwMiECwJGhis6WTmuYAu2sgT8e19604N08MibWMTrH211PR49epT6+vpo7969NDw8DAFT4XBAwMgBB1uwlSMg5xn11j\/b+mvvC5ziLiT\/bKvqcc+ePTQwMBDkASMwlQ8FGis55mALtnIE5Dyj3vplyyMvPAIDAeOXa9W9HTlyhG644Qbq7OwMRAwETOVDgsZKjjnYgq0cATnPqLf+2WIExj\/Tqnvcvn17kIdly5ZhDUyVooHGSg482IKtHAE5z6i3ftk+8NQrtOrORzEC4xdrdb3xwt2tW7fShg0baGpqykjA6DnevXt3dQtQkrcz+8bGxpKUJl\/FAFu5eIAt2MoR8ON5xYoVgaM3z7uK3jxvFQSMH6z58MJTRsuXL6eWlhbCLqTqxQR\/bcmxB1uwlSMg5xn11i\/bgXv208A9z0DA+MVaPW+89qW7u5vGx8fnZGJkZCQQNeEH26hl4oXGSoYrewVbsJUjIOcZ9dYvW3UGzM+8cZgOf\/H3\/DrPgbcTjh07diwH+ahaFjACUzX06GQF0aMjkIMLtmArR8CvZ7WA98TDT9D3v\/xHfp3nwBsEDA6yq1o1REcghx5swVaOgJxn1Ft\/bPUt1Cc\/+Od08MGv+XOeE081L2BM4oApJBNK9jZorOyZmaYAW1NS9nZga8\/MNAXYmpJKt9N3IJ38wCAdfOgf0xMVzAICxiBgEDAGkBxM0Fg5QDNMAraGoBzMwNYBmmESsDUEZWCm1r8srD+JXv3Sf6fJyUmDVMUygYAxiBcEjAEkBxM0Vg7QDJOArSEoBzOwdYBmmARsDUGlmOnTR5cvnkePbf4dCBg\/aIvnBQJGJmZorGS4slewBVs5AnKeUW\/9sB19+BBdM\/p44OyOzvNpQ3srBIwftMXzAgEjEzM0VjJcIWDkuIIt2MoSyO6dR1\/Wjj5ODzz9CvH00b7PXkpl7cMwhWRQX8oafIOii5pAwMjhBVuwlSMg5xn1NjtbffSld+XZ1LtyEQRMdqzF9QABIxM7NFYyXDFKIMcVbMFWlkA27zz6wncf8b88+rLzkxcF\/5a1D8MIjEF9KWvwDYouagIBI4cXbMFWjoCcZ9TbbGy\/+O0p+sxXJwInavSF\/7+sfRgEjEF9KWvwDYouaoLGSg4v2IKtHAE5z6i37mz1qSO19kV5K2sfBgFjUF\/KGnyDoouaoLGSwwu2YCtHQM4z6q0bW33bNIuX2zvOp8vPmTfjrKx9GASMQX0pa\/ANii5qgsZKDi\/Ygq0cATnPqLf2bHXxwql53YsuXjCFZM+0VCkgYGTCicZKhit7BVuwlSMg5xn11o5tWLz8+X87lz7a0jDHSVn7MIzAGNSXsgbfoOiiJmis5PCCLdjKEZDzjHprzla\/64hTdbU20ObfPTfSQVn7MAgYg\/pS1uAbFF3UBI2VHF6wBVs5AnKeUW\/T2fKoy9\/tPUQ3\/+MzM8Y7\/ngpLW+aH5u4rH0YBEx6fSntFjSDoouaoLGSwwu2YCtHQM4z6m0yWx51Wbvt8eCcF354wS5vl+68ZEFiQggYuTqbe89lDX61waOxkosA2IKtHAE5z6i38WzXbvsejTz0\/IyBflBdWkTK2odhBCYt8iU+BMig6KImaKzk8IIt2MoRkPOMejubLY+0fOvJI\/Qnf\/fErA\/+putC+q0LTzMOBASMMaryGZY1+NWOFBoruQiALdjKEZDzjHp7nC0Ll288dnjmVF1FPHxAnWkkytqHYQTGoAaUNfgGRRc1QWMlhxdswVaOgJxn1Fui8FQR0446nM4mCmXtwyBgDGpBWYNvUHRREzRWcnjBFmzlCMh5rtV6y4tzB+95hh54+pVZcLMKF+WsrH0YBIzBd7GswTcouqhJrTZWolDfdg62cpTBFmx9EGDRsmPfCzQ8Nj3HnS\/hAgHjI1IV8HH06FHq6+ujXbt2BW9bs2YN9fb2xr55+\/bttH79+uDztrY26u\/vp7q6ukh7CBiZAKIjkOHKXsEWbOUIyHkue73ltS3bv3OINn7znTNcFE0WLZctnhdsi+b\/9\/mUtQ8rzQjMwMBAEG8WLUeOHKHu7m7q6Oig9vb2OfVgz549xPZDQ0OBaGHh09DQECt4yhp8n18QF19lb6xcmPhKA7a+SM71A7Zga0OAR1ru+e5huuNbz0Ym8z3aEvWSsvZhpREw4aDpgib8GY++jI2NzYy6hH8O25c1+DZfQglbdAQSVI\/7BFuwlSMg57ks9TZuTYs+2sI3RrN48T3aAgEjVz8r4lmNwPBoTEtLi9EITGtra+RoDSeGgJEJW1kaKxk62byCbTZ+SanBFmzDBHhq6Mt7pumhZ34wZyGuLlpuaz+P3n9qXUVEi57HsvZhpRuB4ZGXLVu2pK5rmZiYoK6uLpqenqaRkZFIoaMqAAdff3bv3i33Da4hz1NTU9TY2FhDJa5cUcFWjjXY1jbb6VffCgBM\/eDH9L8f\/gHtfe74sf7hp+GUE+nM95xIa351XvAv\/1ypZ8WKFXNeNTk5WanXV+w9pRMwihwLGRYnUYtzecpo27ZtwRqY+vr6YD0MP3GLfsuqXitWy2JehL9k5SIAtmArR0DOc17rrVrHMv7sa7EjLEyFp4M6LzkzWIx7+Tnz5EBZei5rH1ZaAcMjLD09PTQ4OEhNTU0z4Va7lfQpozhblaiswbf8Dng3z2tj5b2gVXAItnLQwbb8bFmwfOXRF2jrg3O3OOulZ8Fy+eL51HHJglwJlnCEytqHlVbA6DuNeJRFPRAwco2PrWd0BLbEzO3B1pyVrSXY2hIzt68GWxYrL7z6w0CshA+SC+dcbXXmUZZKLcA1pxdvCQHjg6LmQy20HR8ft\/Lc3NxMO3bsmJNGnwZSIiVua3TUFFLcdBO\/qKzBtwIvYFyNxkqgGLl0CbZyYQHb4rJlsfL9135Efz32XKpY4VKySLn6ol+kP7i0ISh0JXYMSdAtax9WtRGYtJ1CUUFUoypRAiZ8kJ1+OJ36rLOzc2axrlrsy+\/BQXYSX5l0n+gI0hm5WoCtK7n0dGCbzsjVwidbFiuHX\/8Rfen\/mouVhfNPop63D5IrqliJYg8B41ojY9L5FjCeszfLXVmDL8nMxLfPxsrkfbVkA7Zy0Qbb\/LDl7cssNPjfzffup2cOHzUeWWGx8okrGmn+L\/xcoaaDXOiXtQ+r2giMSxCqlaaswa8WT\/VedARyEQBbsJUjIOc5rd7yqMr3XvgP2rnv+0ZCRU37sFj5tfNPpWULT8n1YlspsmXtw6omYPQ1MGn3FkkF1dRvWYNvWn4pu7TGSuq9teAXbOWiDLaybH\/2vWcGIyoP7f8BfeuJI3Tw5TeDn00efZHtjHjxfK+QST7yZlPWPqxqAkYFWF+Lwotuh4eHZ217zkNFKGvwq80WHYFcBMAWbOUI+PGsRMnu7x2hHY++YC1UeFTlw+fW0wcXvbf0U0BZiZe1D6u6gFGBCe9KytOoTFmDn\/VLkTU9OtmsBOPTgy3YyhGw88zTPvx847EX6bHnXjee+lEjKPwvHwzHW5fzdDicHYXqWpe1D8uNgNHDqx\/zn4dRmbIGv7pfKVw4KMkfAkaOLtjOZcujKfyf2qLMFmlnquhe1I6f5l88kW78nQsDX0U6Z0WutvnxXNY+LJcCRg9Z3IF0fsJq5qWswTcrvZwVOgKwlSMg57lW660SKU8ffiO4tPDZI29aiRQ1osJTP5edMz8YVVHCRf1bq2zlautxz2Xtw3IpYPQRGIafdtkigi9NQMY\/GisZruwVbMHWlQBP+Zzxnp+nv7jvIB18yWxbctRoCguU665cRNOv\/NB4NAX11jVqyekgYGS4znhNOohO+NWp7ssa\/NSCCxugsZIDDLZgG0VATc2wSHn5jR\/TN\/\/9sNNIij6a8tFLG2jBKe+aM5riEgHUWxdq6WnK2odVfQRG34WUh9GWqKpQ1uCnV3tZCzRWcnzBtrbZqh0+Iw89T2NPv2K1wyc8mnJ8ymceXbZ4fvCR5NoU1FuZelvWPqxqAkbfdZR2lL9MSM29ljX45gRkLNFYyXBlr2BbbrZKoDx+6HXaOf6i8yiKPpJy0cJTqPuys6q6gBb1VqbelrUPq5qAeeqpp+jTn\/40XX\/99TP3E6WFLukupLS0WT4va\/CzMPGRFo2VD4rRPsC2uGz1aR4uxejDzwcCxeZAt\/AoCv98+Tnz6fwF76bmxveIjqJkIY96m4VefNqy9mFVEzC4C0mmohbJKxoruWiBbb7Zqh09PB1z+30H6XuH\/sNZoOijKBecdTL95gWnB4Uv4pkpqLcy9RYCxjPX8MF1pu6bm5sp6jZq0\/QudmUNvgsLn2nQWPmkOdsX2FafLS+Ufd\/bAuUJDwIlECWL59Of\/NpCOvSDH3lZNCtHyc0z6q0bt7RUZe3DqjYCkwY8T5+XNfjVZozGSi4CYCvLlu\/r4efNH\/+UbrvvIB146WjmEZTjoybz6ZzT6+g\/vf+9gX\/JBbNyhNw9o966s0tKWdY+DALGoL6UNfgGRRc1QWMlhxdss7FVUzz3fPcwjT\/7WuDM5mTZ8NvVQW18Nsp\/Pq+eLnn\/e4PFskWc5slGNjk16q0M3bL2YRAwBvWlrME3KLqoCRorObxgG89WiZOGee+iW\/75QHBYm+sCWfWWYKRk\/knBlNFVzWfQeQvePZMBJV7kol0ez6i3MrEsax8GAWNQX8oafIOii5qgsZLDW6ts1fZi\/vfbT71Me54+fpGgj9ETFigXNr6HLjr1x7RgwZk1N70jV1vf8Vyr9VaabVn7MAgYg5pT1uAbFF3UBI2VHN6ysQ1vLf7u86\/TP\/y\/FzOLE3agRkh4\/UnjvHcF61D034dHUMrGVq4W2nsGW3tmJinK2odBwBhEv6zBNyi6qAkaKzm8RWGrhIma1mEir775Fn39344fzuZjaicQIzx6ctZ76DcuOG1GnLhO7RSFrVztkvMMtjJsy9qH5UrAbN++ndavXx9EkC9wPHDgAI2NjVF\/fz\/V1dUlRjZ8l9KaNWuot7c3Ng0fird69ergc96aPTQ0RPX19ZH2ZQ2+zFfF3CsaK3NWtpZ5YqufefJP3z1M+6Ze97LuRI2SsDhZdFodtf3K6VT3cz8rvr04T2xt60Xe7cFWJkJl7cNyI2D4TqTp6Wnq6emhtWvXBuKDhUVfXx81NDQkihEOOafnh9OpM2Y6Ojqovb19To1Qt11v2rQpOAWYhVOSUCpr8GW+KuZe0ViZs7K1rBRbfc3Jw\/t\/QPc9cSTIapY1J6qsamHskgXvpsWn19FvXXD6zDH3SrzYcvFhXym2PvJaNB9gKxOxsvZhuRAw+qm8S5Ysoe7u7kCIsLhQ1wckjZBEhVwXNOHPWbDs378\/VRSpdGUNvsxXxdwrGitzVraWWdnqa06O0TH6x8cO02PPve5dnPCunU9c0Uivv\/kT8ZETW4Zx9lnZ+spHGf2ArUxUy9qHlVLAJF1ToKaaWltbI0dnoqpPWYMv81Ux94rGypyVrWUc2\/Bi2EOv\/jAYNfGx3kQfFeFpnYWnnkRtv3JGcP9Omc48Qb21rY3m9mBrzsrGsqx9WC4EDAdCTePoU0hqNCZuKihu5GXLli0Ud8O1EjArV66ku+66i8bHx7EGxuab4NEWjZVHmG+7UutNnj\/0PD3zRh09MPGy11GTQKTMP4k+0HAy\/ZdfPo1O\/JkTgsPY1FSS68JY\/yTkPKLegq0cARnPEDAyXGd51RfWqg82btxoPFKiO1NrasILgJWAOXjw4MzC3Thb5Y+Drz+7d++uAI3yv2JqaooaGxvLX1BPJdz73JuBp5\/89Bh944n\/oOlX36LnX3sr+Dfr03DKiYGLM99zIl244F3U+v664P\/Voz7P+p4ypEe9lYsi2Pphu2LFijmOJicn\/TjPkZfcjMD4ZsILdXk0Z3BwkJqammbcR00hxdnqAqaMwffN3NYf\/pI9Towv\/eNnwXt\/nm6\/71mafPGN479\/+xA2W666vRoR4VGTX244mZYvqacPnHl8Smfms\/qTsryi5tKi3sqFHGxl2GIERoarmNekxb884rJo0aKZkR0WMDfffDNt3rw5cit1WYMvBt\/QcdkbKzWtwjiGx56jvQdeDchkPdtE4X1HgNTRhWedTGuuaJwRJs8++yxd1vyOcDcMCcwMCJS93hogEDMBWxm0Ze3DcjECoxbd8nqUpCfpbBd915EaZYnbfh0WN0k7ljg\/ZQ2+zFfF3GvRGiv90DVVyh37vk+7H39JRpi8vRB23ZWLaOrlH1qNmBSNrXmtqb4l2MrFAGxl2Ja1D8uFgOGQ8SLebdu2zTpQTj\/PZdWqVYlnwoQPstMX8arPOjs7g63Z\/OjrbeIW\/KqqVNbgy3xVzL3mpbGKEiYPvX2mia\/dOUxFn87h7cO\/39JAb\/3kmJUwMaWbF7am+S2SHdjKRQtsZdiWtQ\/LhYBJ2vasj5Y8+eSTwYF1O3bskIlyjNeyBr+iECNeJt1YhY+pZwHxb8+9Tt94zM8x9XqR9NuIly08hc79xXeLCBPTmEmzNc1HGe3AVi6qYCvDtqx9GASMQX0pa\/ANii5qkrWx0u\/Peeunx+jv9x7ydp6JKrg+avKRc+vpkkXvDT4KBEuOF79mZSsa+II7B1u5AIKtDNuy9mG5EDAmU0h8JYA6K+bWW2+ViTJGYCrKNemwNbUA9tSTf47u\/NazdOClo94Wv4ancy5dPI+u0G4gzrMwMQ0QOgJTUvZ2YGvPzDQF2JqSsrODgLHj5WQddQ4MX+qo7iu67bbbaHh4eNa2aKcXWSYqa\/AtMXgx1y\/2++vtGiaZAAAgAElEQVRvPUHfef6ngV\/vO3Pmn0QXLzqFui49a9Yha2UQJyaBQEdgQsnNBmzduJmkAlsTSvY2Ze3DcjMCYx+SyqUoa\/AlCCqB8uOf\/JS+8uj3vd46HIycvH0K7CeXv68mhYlpzNARmJKytwNbe2amKcDWlJSdXVn7MAgYg3pQ1uAbFH2OiX778P955IXg0LWsB67pC2D\/4NIG+uFbMjtzXMpb1DToCOQiB7ZgK0dAxnNZ+7DcCBg+TK6rq4ump6fnRLC5uXnW9mqZEMd7LWvw40qsX\/j3+KHXadf4i84iRRcnVzWfQedpF\/uhI5CryWALtnIE5Dyj3sqwLWsflgsBox\/vr8574TNb1GWOvb29M+e3yIQ32WtZg6+Xmo+z\/8qjL9DT37cbUVECZen7TqHzz3w3XbZ43ozbtPUmaKzkajPYgq0cATnPqLcybMvah+VCwITPgdGP+ueFvaOjoxS+lFEmzNFeyxj80YcP0ehDzxuNrCghsvIDp9E1Hz6+9oRvIM76oLHKSjA+PdiCrRwBOc+otzJsy9iHMalcChjeLr1\/\/37ikZekO41kQj3Xa9GDrxbWjj78PLFwSXpYrCxvmk+\/d\/EC8bNO0FjJ1WCwBVs5AnKeUW9l2Ba9D4ujkgsBw5nT7yPSRcu9995LY2NjGIFxqNcsXDb\/8376mz3PR6ZW0z+3d54ffJ425eOQhcQkaKx8E33HH9iCrRwBOc+otzJsIWBkuM541dfB8KF1LGi2bNlCfCFjNc5+0YtbtODzepa12x6f2Wasl4VFypoPNdIff+h9whFNd4\/GKp2RqwXYupJLTwe26YxcLcDWlVxyuqL1YaYUcjMCY5rhatgVIfg82vIvTxyha\/\/+iTmIWLTc3nG++JSQbWzQWNkSM7cHW3NWtpZga0vM3B5szVnZWBahD7Mpj7LNhYAxvcyxvr7epYyZ0+Q5+Gp9y6o7H51VThYtv730DPp461kVnxoyBY7GypSUvR3Y2jMzTQG2pqTs7cDWnplJijz3YSb5j7OBgDGgl9fgR00VqdEWH7uEDNBkMkFjlQlfYmKwBVs5AnKeUW9l2Oa1D8ta2qoKGN5ttH79+tQyrFmzJtiRVK0nb8HnURfeTTRwzzMzSIokXFSm0VjJ1WiwBVs5AnKeUW9l2OatD\/NVyqoKGFWIpCkkXwXN4idvwV9604Oz7gEavHoJ\/foHTs1SxKqkRWMlhx1swVaOgJxn1FsZtnnrw3yVMhcCxldhpPzkJfg8ZaSvdSniqIseIzRWUjWWCGzBVo6AnGfUWxm2eenDfJcOAsaAaLWDz1NG9\/z7YerdMTGT2y93XUD\/9cLTDXKfXxM0VnKxAVuwlSMg5xn1VoZttfswmVJV8SReNW00Pj6eWrZavsyRxcv27xyijd88vt6l6KMuGIFJre5eDNAReMEY6QRswVaOgIxnCBgZroXwWq3ghxfrsnjZ+cmLcrst2jaY6AhsiZnbg605K1tLsLUlZm4PtuasbCyr1YfZ5NHFtjRTSOok3127dgUcTHcuTUxMUE9PDw0ODlJTU1Mkw2oF\/64HnqPeu5+cGXkpk3jhQqGxcvnKmqUBWzNOLlZg60LNLA3YmnGytapWH2abT1v7XAmYqG3VGzduJL5aIO3R71JS01MdHR2JaZXo2bt3b+J1BdUIPm+Tvmb08dKKFwiYtBqd7XN0BNn4JaUGW7CVIyDjuRp9mExJZnvNjYBh8bJt2zYaGhoideKuqRCJAqULmjiQ6tJI\/jxPIzC1IF4gYGS\/3uhk5fiCLdjKEZDxDAEjwzXw6vsqAZNzZdjmhhtuoM7OzuDiyLwIGF73wue8qGffZy8tzZqXcBVCRyD3pQJbsJUjIOcZ9VaGLQSMDFfvAkbdYt3W1kb9\/f1UV1cXmXMe8eFn2bJlRmtgdCe7d+8WoTH96lv0p\/98mPY+92bg\/6+uXkAXn3WSyLvy4HRqaooaGxvzkJXS5QFs5UIKtmArR8CP5xUrVsxxNDk56cd5jryUegppeno6UsTwwt2tW7fShg0biBujvCziHbhn\/8z1AJcvnkc7r7koR1XFf1bw15Z\/psoj2IKtHAE5z6i3MmwxAiPDdZbXLIt4w9lL2l3EozTLly+nlpYWyssuJH3qqGzbpeOqDhoruS8V2IKtHAE5z6i3MmwhYGS4inlVC3T1RcH8sqQD9EZGRgJRE36kg8\/iZe3o4\/TA068Er+bt0kW4TTpr8NBYZSUYnx5swVaOgJxn1FsZttJ9mEyu073mYgopy24jVUR915HaHt3Q0JB6i3UeRmD0XUe1MHWEaY70L2ZWC3QEWQlCHMoRBNtKs4WAESYenj6KGw2Jy0b4IDt9Ea\/6jHcchUdYqi1ganHqCAJG+MuEQwJFAUMcyuEFWxm2EDAyXCO9qp1E\/CGPogwPD8eekluJbEkG\/\/Nfn6Q\/330gKEbvyrOpd+WiShQpF+9AYyUXBrAFWzkCcp5Rb2XYSvZhMjk285qLKaSkrLKY4fUs4bUsZsXzYyUV\/PDoC5\/5UksPGiu5aIMt2MoRkPOMeivDVqoPk8mtuddcChh9BKbaN1EzSqngf+arT9EXv\/1sEK07Os+nzksWmEeuBJZorOSCCLZgK0dAzjPqrQxbqT5MJrfmXnMjYPI2baQjlAp+\/bX3Ba\/hbdO1NvrC5UZjZf5FtbUEW1ti5vZga87K1hJsbYmZ2Uv1YWZvl7PKhYAxOfpfDkG6Z4ngD\/7Tfur\/5jM1O\/oCAZNe77JYoCPIQi85LdiCrRwBGc8SfZhMTu285kLA2GW58ta+g1\/ra19UBNERyNVlsAVbOQJynlFvZdj67sNkcmnvFQLGgJnv4OsCplYOrYvCjMbKoPI5moCtIziDZGBrAMnRBGwdwaUk892HyeTS3isEjAEzn8Fn8bLqzkeJ\/63VtS8YgTGodBlN0BFkBJiQHGzBVo6AjGeffZhMDt28QsAYcPMZfH305c9+Zwl1X3aWQQ7KaYKOQC6uYAu2cgTkPKPeyrD12YfJ5NDNay4ETNIi3rg7jdyK65bKZ\/BX3fFozd15FEcdjZVbfTRJBbYmlNxswNaNm0kqsDWhZG\/jsw+zf7tcCggYA7a+gq+PvtTSnUcQMAaVzLMJOgLPQDV3YAu2cgRkPPvqw2Ry5+61qgImfP9RXDHWrFmTeimjO4L0lL6CP3DPfhq45\/jW6VpevKuIoyNIr3uuFmDrSi49HdimM3K1AFtXcsnpfPVhMrlz91pVAaOyXSvnwKjpo1pfvAsB4\/6FNU2JjsCUlL0d2NozM00Btqak7OwgYOx4lcraR\/AfeOqVYPcRP3xlAF8dUOsPGiu5GgC2YCtHQM4z6q0MWx99mEzOsnnNxQhMtiLIp\/YR\/D\/9h6fpL\/7lYJBZTB8djxkaK7m6C7ZgK0dAzjPqrQxbH32YTM6yea2agNGnjZYsWULd3d00Pj4eWZpqX+joI\/i1fu9RVGDRWGX78ialBluwlSMg5xn1Voatjz5MJmfZvFZNwGTLdmVTZw2+Pn20+XeXUFdr7Z79okcOjZVcPQZbsJUjIOcZ9VaGbdY+TCZX2b1CwBgwzBr8G78+Sf9r94HgTZg+egc4GiuDyudoAraO4AySga0BJEcTsHUEl5Isax8mk6vsXnMhYNR0UlmnkLD7KLqiorHK\/gWO8wC2YCtHQM4z6q0MWwgYGa6JXlnYXHfddfSZz3yGmpqaqpCD46\/MEnx9+uhTH1lIN7Qtrlo58vZiNFZyEQFbsJUjIOcZ9VaGbZY+TCZHfrzmYgQmqSh8lcDo6Cj19\/dTXV1drOnRo0epr6+Pdu3aFdgkHX4XHvFpa2tL9J8l+Di8Lj66aKz8fImjvIAt2MoRkPOMeivDNksfJpMjP14LIWAGBgZoaGiI6uvrY0vNNvz09vaSEigdHR3U3t4+K40SOq2trcFn6ueGhobY036zBB\/TRxAwfr6qdl7QEdjxsrEGWxtadrZga8fL1DpLH2b6jmrY5V7AsDCZnp5OHYEJw9MFTRpYvtJgbGws9h2uwdfvPvpQ03z66h8vTctKTX2Oxkou3GALtnIE5Dyj3sqwde3DZHLjz2suBEzSIl4eGRkeHrZaA2N7NYGUgNHXv\/DJu3wCL553CKCxkqsNYAu2cgTkPKPeyrCFgJHhOuM1bmpHTfWYvp5HXrZs2UJp61qUv6TpJmXjGnw1fcR+sH16bgTRWJnWans7sLVnZpoCbE1J2duBrT0zkxSufZiJ72ra5GIEhgFETRWZiIs4eCZTT0o0sY+kRcIcfP3ZvXu3Uczatk7R9KtvUcMpJ9KujzUapaklo6mpKWpsBBeJmIOtBNXjPsEWbOUI+PG8YsWKOY4mJyf9OM+Rl1wImKQpH9NdSGGmExMT1NPTQ4ODg5HTT6bihf26qFd9\/Qsub4yu8fhrS64lAFuwlSMg5xn1VoatSx8mkxO\/XgshYEx2IYWxsPCJS2ey80j35xJ8bJ9Or6horNIZuVqArSu59HRgm87I1QJsXcklp3Ppw2Ry4tdrLgRMeP2LXsS0BbbKVt91lCZQTKaXsgoYff3LkVs+4jdqJfGGxkoukGALtnIE5Dyj3sqwhYCR4TrjlUdM1q1bN2vHEU8DdXV10aZNm6ilpSUxB+GD7PRFvOqzzs5Oirv5OunGa5fg4\/bp9AqDxiqdkasF2LqSS08HtumMXC3A1pUcRmBkyFl4ZRGzevXqWSlGRkZSxYvFK5xMbQWMvv6ld+XZ1LtykdN7y54IjZVchMEWbOUIyHlGvZVha9uHyeTCv9dcTCH5L5Zfj7bB189\/wfbp+FigsfJbT3VvYAu2cgTkPKPeyrC17cNkcuHfay4EjD7FkzZV5B9Bukfb4OvrX\/Z99lJaWH9S+ktq0AKNlVzQwRZs5QjIeUa9lWFr24fJ5MK\/11wIGNuTc\/1jSPZoG\/ylNz1IPI3EwoUFDJ5oAmis5GoG2IKtHAE5z6i3Mmxt+zCZXPj3mgsBw8Uy3W3kH0G6R5vg6+tfLl88j3Zec1H6C2rUAo2VXODBFmzlCMh5Rr2VYWvTh8nkQMZrLgRM0l1IXOykHUIyWGZ7tQm+vv4FC3iTo4PGSq72gi3YyhGQ84x6K8PWpg+TyYGM11wIGJmi+fNqE3wcYGfOHY2VOStbS7C1JWZuD7bmrGwtwdaWmJm9TR9m5jEfVhAwBnGwCb5awIv1L+lg0VilM3K1AFtXcunpwDadkasF2LqSS05n04fJ5EDGa9UEjL5wN+5wOVXkokwhYf2LXSVFY2XHy8YabG1o2dmCrR0vG2uwtaFlbgsBY86qdJamwccBdnahR2Nlx8vGGmxtaNnZgq0dLxtrsLWhZW5r2oeZe8yHZdVGYMLFD9+HlHQ\/UqXRmQYfB9jZRQaNlR0vG2uwtaFlZwu2drxsrMHWhpa5rWkfZu4xH5a5ETBRFyyqaaaOjg5qb2+vGjHT4P\/tQ8\/Tp7Z9L8gnTuBNDxcaq3RGrhZg60ouPR3YpjNytQBbV3LJ6Uz7MJm3y3nNhYBJOsiO70caHR2l\/v5+qqurkyOR4Nk0+FjAaxceNFZ2vGyswdaGlp0t2NrxsrEGWxta5ramfZi5x3xYFkLA8OjM0NAQ1dfXV4WaafDVDdQ4wM4sTGiszDi5WIGtCzWzNGBrxsnFCmxdqKWnMe3D0j3lyyIXAiZpvUseTug1CT4W8NpXbDRW9sxMU4CtKSl7O7C1Z2aaAmxNSdnZmfRhdh7zYZ0LAcMoeKpo3bp1NDw8TE1NTQGdiYkJ6urqok2bNlE1L3k0CT4W8NpXaDRW9sxMU4CtKSl7O7C1Z2aaAmxNSdnZmfRhdh7zYZ0bAaNEzOrVq2eRGRkZqap44cyYBF8\/gRc3UJtVbjRWZpxcrMDWhZpZGrA14+RiBbYu1NLTmPRh6V7yZ5ErAZM\/PMdzZBJ8LOC1jx4aK3tmpinA1pSUvR3Y2jMzTQG2pqTs7Ez6MDuP+bCGgDGIg0nwl970IPE6GCzgNQD6tgkaK3NWtpZga0vM3B5szVnZWoKtLTEze5M+zMxTvqwgYAzikRZ8XCFgADHCBI2VGzeTVGBrQsnNBmzduJmkAlsTSvY2aX2Yvcd8pICAMYhDWvCxA8kAIgSMGyTHVOgIHMEZJANbA0iOJmDrCC4lWVofJvNWea81KWDUtu1du3YFhNesWUO9vb2xtNOCjwW8bhUVjZUbN5NUYGtCyc0GbN24maQCWxNK9jZpfZi9x3ykqEkBwwfj8cOixeS6grTgXzP6OI0+fCjwiR1I5hUbjZU5K1tLsLUlZm4PtuasbC3B1paYmX1aH2bmJX9WuREw6syX6enpOZSam5tFT+LVBU1UiNKCjx1IbhUbjZUbN5NUYGtCyc0GbN24maQCWxNK9jZpfZi9x3ykyIWAUVM6DQ0NiVM5EsiS7mFS70sLPq4QcIsMGis3biapwNaEkpsN2LpxM0kFtiaU7G3S+jB7j\/lIkQsBYyIiJHDxyMuWLVuora0t8bJIDr7+7N69e+bH6VfforatU8HPaz44j\/7Hr86TyGopfU5NTVFjY2Mpy1btQoGtXATAFmzlCPjxvGLFijmOJicn\/TjPkZdcCBg1AtPZ2VmVU3dZyPDUVdyN10nqFVcIuNdm\/LXlzi4tJdimEXL\/HGzd2aWlBNs0Qm6fYwTGjZtxKr4LqVq3TvP6m56eHhocHJy5h0nPeFLw9R1IOz95EV1+DkZgTIOOxsqUlL0d2NozM00Btqak7O3A1p6ZSQoIGBNKjjZqCml8fDzSg\/Qi3jTxlBT8DV99iv7y288G+cYOJLsKgMbKjpeNNdja0LKzBVs7XjbWYGtDy9wWAsacVe4t9V1HJguIk4KPHUju4UZj5c4uLSXYphFy\/xxs3dmlpQTbNEJun0PAuHHLZarwQXYmi3jjFkDhDiT3EKOxcmeXlhJs0wi5fw627uzSUoJtGiG3zyFg3LhZpdq+fTutX78+SDMyMkIHDhygsbGxxB1CVi9wNE4KPrZQO0IlIjRW7uzSUoJtGiH3z8HWnV1aSrBNI+T2OQSMGzfjVGonEC+mXbt2bXAeDK996evro2qcD6NnPC74+g6k3pVnU+\/KRcblhSEEjGQdQEcgRxdswVaOgIxnCBgZroFX\/RyYJUuWUHd3dyBgWlpaKG2BrWC2ZlybCBjsQLKPBDoCe2amKcDWlJS9HdjaMzNNAbampOzsIGDseFlZQ8BY4SqNMRoruVCCLdjKEZDzjHorwxYCRobrjFde\/8LrXfQpJDUa09HRQe3t7cI5iHcfF3y1AykYRbrlI1XLX1FfjMZKLnJgC7ZyBOQ8o97KsIWAkeE6yytPF61evXrW7zZu3FhV8cKZSRMwC+tPCs6AwWNHAI2VHS8ba7C1oWVnC7Z2vGyswdaGlrktBIw5q9JZxgUfO5CyhRqNVTZ+SanBFmzlCMh5Rr2VYQsBI8O1EF6jgn\/wyJvEZ8Dw03nJArqj8\/xClCVPmURjJRcNsAVbOQJynlFvZdhCwMhwneVVPwdGfcDnwfBupGo+UcHHFursEUFjlZ1hnAewBVs5AnKeUW9l2ELAyHCd8criZdu2bTQ0NET19fXB79XupDwu4tVHYLCF2q1yoLFy42aSCmxNKLnZgK0bN5NUYGtCyd4GAsaemXEKfRt1eLQlr+fA4BZq4\/DGGqKxys4QIzByDMEWbCtPQOaNEDAyXGeNtKjD6\/RX5VXAXDP6OI0+fOh4\/rGF2ql2QMA4YTNKBLZGmJyMwNYJm1EisDXCZG0EAWONzC4BC5V169bR8PAwNTU1zRI2eZxCwi3UdvGNskZjlZ0hRgnkGIIt2FaegMwbIWBkuM4SKuPj46lv4fuRduzYkWrn0yAq+LiFOjthCJjsDNHJyjEEW7CtPAGZN0LAyHAthNdw8PUFvJcvnkc7r7moEOXIWyYhYOQiArZgK0dAzjPqrQxbCBgZroXwmiRgcAu1ewjRWLmzS0sJtmmE3D8HW3d2aSnBNo2Q2+cQMG7crFJFnQOTx6sE9DNg+AoBvkoAjz0BNFb2zExTgK0pKXs7sLVnZpoCbE1J2dlBwNjxsrYu0jkwvPuIdyHxgzNgrEM9kwCNlTu7tJRgm0bI\/XOwdWeXlhJs0wi5fQ4B48bNKFXRzoHBGTBGYU01QmOVisjZAGyd0aUmBNtURM4GYOuMLjEhBIwM18Br0QTMqjsfJZ5GCvKOM2CcawYaK2d0qQnBNhWRswHYOqNLTQi2qYicDCBgnLCZJ8o6haREkNqK3dbWRv39\/VRXVxeZCX29TZptOPg4A8Y8rkmWaKz8cIzyArZgK0dAzjPqrQxbCBgZrrO8ui7iPXr0KPX19VFrayu1t7eT+rmhoYH4dN\/wo5\/uywKH08bZctpw8HEGjJ\/KgMbKD0cIGDmOYAu2lSUg8zYIGBmuYl5ZDI2NjUWOwoQ\/S7INCxicAeMvZBAw\/liGPYEt2MoRkPOMeivDFgJGhquY1yRREjUCo0ZvojKkB18XMLf83rn0B5c2iJWh7I7RWMlFGGzBVo6AnGfUWxm2EDAyXEW8qvUwSXcoTUxMUFdXF01PT9PIyAiFb8HWM6YHXz8DBofYZQsfGqts\/JJSgy3YyhGQ84x6K8MWAkaGq3evav0LO45bxBteMDwwMBDkI2q9DP+eg6+eHy28jN5Y9vHgx7+6egFdfBYOsXMN4tTUFDU2NromR7oEAmArVz3AFmzlCPjxvGLFijmOJicn\/TjPkZcTjh07dixH+cmUFRPxEl7wyy\/k0Zienh4aHBycuQk7bgQGZ8BkCtGsxPhryx\/LsCewBVs5AnKeUW9l2GIERoarN69pO4\/Ui7IKGD6Bl0\/i5QfXCGQLHxqrbPySUoMt2MoRkPOMeivDFgJGhqs3rzwNxOtZks5+US+LmkJKSqsHH2fAeAsZobHyxxIjMHIswRZsK0dA5k0QMDJcvXgNH2KnnDY3N9PQ0FBwmB2f9dLZ2TmzWJcFz5YtWwJTm4PsIGC8hCxwAgHjjyU6WTmWYAu2lSMg8yYIGBmuhfCqB7\/+2vuCPF++eB7tvOaiQuQ\/r5mEgJGLDNiCrRwBOc+otzJsIWBkuBbCqwq+fgZM5yUL6I7O8wuR\/7xmEo2VXGTAFmzlCMh5Rr2VYQsBI8O1EF6jBAzOgMkeOjRW2RnGeQBbsJUjIOcZ9VaGLQSMDNdCeFXB1w+x49EXHoXB404AjZU7u7SUYJtGyP1zsHVnl5YSbNMIuX0OAePGrRSpogTMzk9eRJefM68U5atWIdBYyZEHW7CVIyDnGfVWhi0EjAzXQnhVwcchdn7DhcbKL0\/dG9iCrRwBOc+otzJsIWBkuBbCqwo+DrHzGy40Vn55QsDI8QRbsK0MAZm3QMDIcC2EVxV8nAHjN1wQMH55opOV4wm2YFsZAjJvgYCR4VoIrxAwMmGCgJHhyl7BFmzlCMh5Rr2VYQsBI8O1EF5V8Jfe9CDxWTA4xM5P2NBY+eEY5QVswVaOgJxn1FsZthAwMlwL4VUFH6fw+g0XGiu\/PDHNIccTbMG2MgRk3gIBI8O1EF45+N\/6zneJR2D4wSF2fsIGAeOHI0Zg5DiCLdhWloDM2yBgZLgWwisH\/8v\/9AituvNRCBiPEYOA8Qgz5ApswVaOgJxn1FsZthAwMlwL4TUsYPZ99lJaWH9SIfKe50yisZKLDtiCrRwBOc+otzJsIWBkuBbCKwf\/C9vHiM+B4Qen8PoJGxorPxwxzSHHEWzBtrIEZN4GASPDtRBeOfhr\/vJfaOCeZyBgPEYMAsYjTEwhycEEW7CtGAGZF0HAyHAthFcO\/m984es0+vChIL+YQvITNggYPxwxSiDHEWzBtrIEZN4GASPDtRBeOfgXXPcVeuDpV4K1Lyxg8GQnAAGTnWGcB7AFWzkCcp5Rb2XYQsDIcC2EVwgYmTChsZLhyl7BFmzlCMh5Rr2VYQsBI8O1EF45+Kd8\/G9xCq\/naKGx8gxUcwe2YCtHQM4z6q0MWwgYGa6F8MrBf+W3h4K84hoBfyFDY+WPZdgT2IKtHAE5z6i3MmwhYGS4evN65MgR6u7upvHx8cBnW1sb9ff3U11dXeQ79uzZQ6tXrw4+a25upqGhIaqvr4+0XXTBB+nVXx8IPuu8ZAHd0Xm+t3zXsiM0VnLRB1uwlSMg5xn1VoYtBIwMVy9ejx49Sn19fdTa2krt7e2kfm5oaKDe3t4575iYmKCuri7atGkTtbS00Pbt22lsbCxW8OgCBmfAeAlZ4ASNlT+WGIGRYwm2YFs5AjJvgoCR4SrmNUmU8Gf79++PFDdRGVr4wd+g1y\/vCT7i0RcehcGTnQAETHaGcR7AFmzlCMh5Rr2VYQsBI8NVzGucgAmP1phkoPHDv09vLPt4YIoRGBNiZjZorMw4uViBrQs1szRga8bJxQpsXailp4GASWeUGwu1HqajoyOYUtIfJWBWrlxJd911V7BmJm0NTMNv\/k9687xVgZuTHxik+7ffmZuyFjkjU1NT1NjYWOQi5DbvYCsXGrAFWzkCfjyvWLFijqPJyUk\/znPk5YRjx44dy1F+MmdFCRR2FLWIV31+8ODBmYW7AwMDND09HbsGRhcwOIU3c4hmHOCvLX8sw57AFmzlCMh5Rr2VYYsRGBmuXr2miRd+WdQUEi\/q7enpocHBQWpqapqTpwW\/exP9aOFlwe+P3PIRr3muZWdorOSiD7ZgK0dAzjPqrQxbCBgZrt68pu080l\/EIy6LFi2amV5iAXPzzTfT5s2bI7dSn\/HRL9Jbp52LawS8Reu4IzRWnoFq7sAWbOUIyHlGvZVhCwEjw9Wb17RpIP1FfAYM26uzX\/j\/+Ynacs2\/h4DxFqZZjtBYyXCFOJTjCrZgK0tAxjsEjAxXL17Dh9gpp2pxLh9mx+fEdHZ2Bue+8KMfZJd26N1pf\/T39NNfOA2n8HqJ1jtOIGA8A8UIjBxQsAXbihCQeQkEjAzXQnitv\/a+IJ+4RuFMKfQAAAyMSURBVMBvuCBg\/PLUvYEt2MoRkPOMeivDFgJGhmshvCoBg2sE\/IYLjZVfnhAwcjzBFmwrQ0DmLRAwMlwL4VUJmN6VZ1PvykWFyHMRMgkBIxclsAVbOQJynlFvZdhCwMhwLYRXCBiZMKGxkuHKXsEWbOUIyHlGvZVhCwEjw7UQXpWAwT1IfsOFxsovT0xzyPEEW7CtDAGZt0DAyHAthFclYHAPkt9wQcD45YlOVo4n2IJtZQjIvAUCRoZrIbwqAYNrBPyGCwLGL090snI8wRZsK0NA5i0QMDJcC+EVAkYmTBAwMlzZK9iCrRwBOc+otzJsIWBkuBbCqxIwuAfJb7jQWPnliVECOZ5gC7aVISDzFggYGa6F8MoCZmH9ScRTSHj8EYCA8ccy7AlswVaOgJxn1FsZthAwMlwL4RUCRiZMaKxkuGIKSY4r2IKtLAEZ7xAwMlwL4ZUFDK4R8B8qCBj\/TJVHsAVbOQJynlFvZdhCwMhwLYRXCBiZMKGxkuGKUQI5rmALtrIEZLxDwMhwLYRXFjC4B8l\/qCBg\/DPFCIwcU7AFW3kCMm+AgJHhWgivLGBwD5L\/UEHA+GeKTlaOKdiCrTwBmTdAwMhwLYRXCBiZMEHAyHDFNIccV7AFW1kCMt4hYGS4FsLrGR\/9It36qd8OppHw+CMAAeOPZdgT2IKtHAE5z6i3MmwhYGS4FsJrWYNfbfhorOQiALZgK0dAzjPqrQzbsvZhJxw7duyYDLLyeC1r8KsdITRWchEAW7CVIyDnGfVWhm1Z+zAIGIP6UtbgGxRd1ASNlRxesAVbOQJynlFvZdiWtQ8rjYA5cuQIdXd30\/j4eFAD2traqL+\/n+rq6hJrxMTEBPX09NDg4CA1NTVF2pY1+DJfFXOvaKzMWdlagq0tMXN7sDVnZWsJtrbEzOzL2oeVQsAcPXqU+vr6qLW1ldrb20n93NDQQL29vbERVnZ79+6l4eFhCBiz74I3q7J+qbwByuAIbDPAS0kKtmArR0DGc1nrbCkETFTIt2\/fTmNjY4mjMHv27KGBgYEgOUZgZL44SV7L+qWqPMm5bwRbuSiALdjKEZDxXNY6W7MChqecbrjhBurs7AxEDASMzBcHAqbyXPmNZW2wqkNz9lvBVi4KYCvDtqxcSylg1HqYjo6OYEopboSGf79s2TKjNTAy1QpeQQAEQAAEQECewOTkpPxLKvyG0gkYta6FOcYt4uWFu1u3bqUNGzbQ1NRUqoCpcEzwOhAAARAAARAAgRQCpRIwJuKFefCU0fLly6mlpYVMdiGhFoEACIAACIAACOSLQGkEjOnOo\/B2az0cIyMjgajBAwIgAAIgAAIgkG8CpREwPKoyPT1tdPaLHhKMwOS7giJ3IAACIAACIBBFoBQCJm5Upbm5mYaGhoLD7PicGN5xFB5hgYDBFwMEQAAEQAAEikegFAKmeNiRYxAAARAAARAAgSwEIGCy0ENaEAABEAABEACBqhCAgEnAzutqtmzZElhgga9b\/eQpuq6urmB9Utr9VDpvvgYi6XoHt9yUK5UNW1Xy8LUb5SLipzQ2XMPT12gnkmNgw1a3RXuQrW6r733UMopsnqubGgImhr+6ZoDX0Dz55JPB1mv+\/\/r6+upGrEBv1zvLVatWzbqvKlyM8NUP\/PO2bdvAPCbeNmx1F8x1\/fr1tHHjxthDHgtUxbxn1YZreOcj1tMlh8OGrRKGfJcdr1tEe+Be1RX3Xbt2le4PcQiYmHqh7kjiL1BZ1av7V8IsZbhBZ1E4OjpqtFMMnUH6X7L6LeombLlTuO666+iVV16hpFOqzaJbTiubOsu2N998M23evBl\/2BhUB1u2ev1Ge2AAOMJEjWJdfPHFdPDgweBy4zIdFQIBExH0uNut1W3XblWp9lLpo1g8chX+OYkIGqzk+uLClkX5JZdcQl\/72tdmbm6vvVrpj6vJhbHg+w4BmzobNQKTdjkvWM8l8NxzzwW\/5J243d3dEDC1UEmiRly48V+0aBGG3S0qQHhUwOYvVtdzfSyyV2hTW7bq+oxrr702uMQUYjw6\/DZcWcDs378\/cIS1culfJxu27E2f+lizZk3Q+eJxIxAWhG5e8pcKIzAJIzD6gicIGPvKa9tgqTdwx3DbbbdhEW8Cchu23BF84QtfoI997GPU2NiYuBbJPsrlSmHDVa0nUgt3Oe26detQb2OqhA1bNfWxadOmYMrDZvS2XDXST2kgYPxwLIQXTCH5CZPNkDHEix1zG7Zse\/\/99wd\/wWIXUjJnG67hKSSwBVu7b3HlrCFgKsc6F2\/SR1ywiNctJOEpo7SFpthpYM7Zhq2+PV1\/A4bl5\/K24Rquz2gnkuuvDVuIQ\/O2wMQSAsaEUolssI06ezBttk1i+N2Otw1b3TNGCZI523ANdwqY5vDHNmoKCdNzdm2Ebg0B486usClxkF320CUdXKUWQfLURtwoAQ4Gi4+BKVsIGLt6bMNVP8gOh62lc7Zhy4Jw9erVgVOwTWebZAEBk40fUoMACIAACIAACICANwLYheQNJRyBAAiAAAiAAAhUigAETKVI4z0gAAIgAAIgAALeCEDAeEMJRyAAAiAAAiAAApUiAAFTKdJ4DwiAAAiAAAiAgDcCEDDeUMIRCIAACIAACIBApQhAwFSKNN4DAiAAAiAAAiDgjQAEjDeUcAQC\/gk8\/fTTNH\/+fOLbvE0ePu\/h5ZdfpsWLF5uYO9moM3uam5tpaGjIOG9FuWFcla9SZ49U+n1OQUciEMghAQiYHAYFWQIBJmB7smslDqvKIkKypK1kjWBBwU8lbz8uCptKxgHvAoE0AhAwaYTwOQhUiUAeBYxtnnR0RemkIWCqVOHxWhCwJAABYwkM5iDgk4B+FD37VdMyTz755Mwx6vx7daVC+MoFNc1x6qmnUnd3N42PjwfZUxc1qrt9du3aFfzeZNpHf4c+jcJXP6xfv36m+Bs3bqT29vY5OOLslIBZtWoV3XjjjUG68DSNfny8cqzew6yuu+46+tCHPhSkV2V56aWXqKuri6anp4Mkn\/vc52jnzp00ODhITU1Nwe\/iyhQVy7CA4Z9fe+214D\/FMekizPBFhPyOqN8VUdz5rPvwBQJZCUDAZCWI9CDgSCDqYkW98wyPdsTd0Muv7+\/vJ\/bHIoanPlpaWkiJo46OjhmhkXTjt8qP8ldXVxd0vLfddhsNDw8HYiBtBCZsr1\/KxyKLhcbFF18c5Ff537ZtW7CWhoVIT0\/PLOGh+1MibeHChTPpw2VUP7\/44otBnhsbG6mvry8QSmpKKO3i0CgBs2XLFtKFFHPWuepVAALG8QuBZCBgSQACxhIYzEHAF4EogaH7ThML4b\/swwKG04+Ojs509myfdBt11BRP2D4pT2k3XYdvGOb8pE0r6Z8rARMWZGNjY7PKqAsUfsfNN99MmzdvnrXYOGmaKErA8OiOEl3sM4kDBIyvbwj8gEAyAQgY1BAQqCIBfbolPL0T10mGp1na2toiR2DCUzl6MaOmf+Lep98antRxpy0ijhIrUb8LT6uFp8nUCBOXJ0qI6D55VEfdaBwOc9w0UJSA4bT6ot4k4QUBU8UvFF5dUwQgYGoq3ChsXgnonba+DoY7U7VVWQmS8LoUNQIRHoHhtOGRg6Tyx4mTpGkt3V9WAcO+1FoWJbCiRmBsBMwjjzxCaorKdCs6BExevyXIFwjMJgABgxoBAjkioIsANcLAAobXi\/BajtbW1lkLZ3WREhYwSetdoopciSmk8BoX\/Z0sNpKmg9QUki5gokY79CkkHoFZt27dzBoek1BLTCGlicm0qTSTfMMGBGqNAARMrUUc5c0Ngag1MPooiL6oNWoxqhqRUVNIXDBd5Cj\/vKBXTX9ErUNRQHwt4tVHPPR1McuWLZuzSDcsYPS0Kq+cP16QGyVgTBfxsg+1hiVt7VHWRbxqik\/tHFPl0BcvhyshBExuvpbISIEIQMAUKFjIavkIqM5NbQHWp4f0LdA8pXLllVfO2irNwuWqq66i66+\/fmaEISxq1KiM2l7NBFXHGkczacux6cLiqO3WJmtgwu\/etGlTsM6FF+6q8usjMFyGMMPwNurwVnJOE7cFXI168b9K9EVto9bTR5VLX3\/EcVq6dCnt27cvEFFhoanKEB6dKl9tR4lAwC8BCBi\/POENBECgygRYUETtPDLNlskaGFNfpnYYgTElBTsQeIcABAxqAwiAQGEJhNf5qNEW\/dwX28JBwNgSgz0IVIcABEx1uOOtIAACngiETydOOiXX5JXhyxXvvvvuIJnU3Ui4zNEkKrABgbkE\/j9+RD2Drn6KOgAAAABJRU5ErkJggg==","height":0,"width":0}}
%---
