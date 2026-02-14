preamble;
%[text] ### Simlength, model name, thermal model enabling
simlength = 2;
transmission_delay = 125e-6*2;

model = 'mpc_psm';

use_mosfet_thermal_model = 0;
use_thermal_model = 1;
if (use_mosfet_thermal_model || use_thermal_model)
    nonlinear_iterations = 5;
else
    nonlinear_iterations = 3;
end
load_step_time = 0;
%[text] ### Local time allignment to master time
kp_align = 0.6;
ki_align = 0.1;
lim_up_align = 0.2;
lim_down_align = -0.2;
%[text] ### Number of modules
number_of_modules = 1;
enable_two_modules = number_of_modules;
%[text] ### MOTOR Selection from Library
n_sys = 6;
run('n_sys_generic_1M5W_pmsm'); %[output:0db165d8] %[output:255f588d]
run('n_sys_generic_1M5W_torque_curve');

% n_sys = 1;
% run('testroom_eq_psm_690V');
% run('testroom_torque_curve_690V');

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
%[text] ### Settings for speed control or wind application
use_torque_curve = 1; % for wind application
use_speed_control = 1-use_torque_curve; %
use_mtpa = 1; %
use_psm_encoder = 0; % 
use_load_estimator = 0; %
use_estimator_from_mb = 0; %mb model based
use_motor_speed_control_mode = 0; 
use_advanced_pll = 0; % advanced pll should compensate second harmonic
use_dq_pll_ccaller_mod1 = 1; % only module 1
use_dq_pll_ccaller_mod2 = 0; % only module 1
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

use_moving_average_from_ccaller_mod1 = 0;
use_moving_average_from_ccaller_mod2 = 0;
%[text] ### Settings average filters
mavarage_filter_frequency_base_order = 2; % 2 means 100Hz, 1 means 50Hz
dmavg_filter_enable_time = 0.025;
%[text] ### Grid Emulator Settings
nominal_power = 1600e3;
application_voltage = 690;
vp_xi_pu = 1;
vn_xi_pu = 0;
vn_eta_pu = 0;
grid_emu_data = grid_emulator(nominal_power, application_voltage, vp_xi_pu, vn_xi_pu, vn_eta_pu);
%[text] ## AFE Settings and Initialization
%[text] ### Switching frequencies, sampling time and deadtime
fPWM_AFE = 6*4e3;
tPWM_AFE = 1/fPWM_AFE;

dead_time_AFE = 3e-6;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;

TRGO_AFE_double_update = 0;
if TRGO_AFE_double_update
    ts_afe = 1/fPWM_AFE/2;
else
    ts_afe = 1/fPWM_AFE;
end

tc = ts_afe/100;

s=tf('s');
z_afe=tf('z',ts_afe);

% t_misura = simlength - 0.025;
t_misura = 10/omega_bez*2*pi;
Nc = ceil(t_misura/tc);
Ns_afe = ceil(t_misura/ts_afe);
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
grid_fault_generator; %[output:4a633a6c] %[output:8701c749] %[output:73d68fef]
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
l1 = Kd(2) %[output:793416a0]
l2 = Kd(1) %[output:75687a85]
%[text] ### Grid fault generator 
grid_fault_generator;
%[text] ### 
%[text] ### Current sensor endscale, and quantization
adc_quantization = 1/2^11;
Imax_adc = 1049.835;
CurrentQuantization = Imax_adc/2^11;
%%
%[text] ### 
%[text] ### Voltage sensor endscale, and quantization
Umax_adc = 1500;
VoltageQuantization = Umax_adc/2^11;
%%
%[text] ### 
%[text] ### DClink, and dclink-brake parameters
Vdc_ref = 1070; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi = 900e-6*8;
%[text] #### 
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
%[text] ### 
%[text] ### LCL switching filter
LFu1_AFE = 0.5e-3;
RLFu1_AFE = 157*0.05*LFu1_AFE;
LFu1_AFE_0 = LFu1_AFE;
RLFu1_AFE_0 = RLFu1_AFE/3;
CFu = (100e-6*2);
RCFu = (50e-3);
%%
%[text] ### 
%[text] ### DClink voltage control parameters
Vdc_nom = Vdc_bez;
Vdc_norm_ref = Vdc_ref/Vdc_nom;
kp_vs = 0.85;
ki_vs = 35;
%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
kp_afe = 0.6;
ki_afe = 45;
delta = 0.015;
res = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);

Ares = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Bres = [0; 1];
Cres = [0 1];
Aresd = eye(2) + Ares*ts_afe;
Bresd = Bres*ts_afe;
Cresd = Cres;
%%
%[text] ### 
%[text] ### Grid Normalization Factors
Vgrid_phase_normalization_factor = Vphase2*sqrt(2);
pll_i1 = 80;
pll_p = 1;
pll_p_frt = 0.2;
Vmax_ff = 1.1;

Igrid_phase_normalization_factor = 250e3/Vphase2/3/0.9*sqrt(2);
ixi_pos_ref_lim = 1.6;
ieta_pos_ref_lim = 1.0;
ieta_neg_ref_lim = 0.5;
%%
%[text] ### 
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso = [-1 -4]*omega_rso;
Lrso = acker(Arso',Crso',polesrso)';
Adrso = eye(2) + Arso*ts_afe;
polesdrso = exp(ts_afe*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:93a00ed0]

%[text] ### 
%[text] ### PLL DDSRF
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### 
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:832d52f7]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:4c95808e]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:6cc95407]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:99aa498b]
%[text] ### 
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;
kp_rc_pos_grid = kp_rc_grid;
ki_rc_pos_grid = ki_rc_grid;
kp_rc_neg_grid = kp_rc_grid;
ki_rc_neg_grid = ki_rc_grid;
%%
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = rms_perios/f_grid/ts_afe;
rms_perios = 10;
n10 = rms_perios/f_grid/ts_afe;
%%
%[text] ### 
%[text] ### Online time domain sequence calculator
w_grid = 2*pi*f_grid;
apf = (s/w_grid-1)/(s/w_grid+1);
[napfd, dapfd]=tfdata(c2d(apf,ts_afe),'v');
apf_z = tf(napfd,dapfd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(napfd,dapfd);
ap_flt_ss = ss(A,B,C,D,ts_afe);
% figure;
% bode(ap_flt_ss,options);
% grid on
%%
%[text] ## INVERTER Settings and Initialization
%[text] ### 
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
%[text] ### 
%[text] ### Switching frequencies, sampling time and deadtime
fPWM_INV = fPWM_AFE;
% fPWM_INV = 2500;
dead_time_INV = 3e-6;
delayINV_modA = 0;
pwm_out_lim = 1;

TRGO_INV_double_update = 0;
if TRGO_INV_double_update
    ts_inv = 1/fPWM_INV/2;
else
    ts_inv = 1/fPWM_INV;
end

t_measure = simlength;
Ns_inv = floor(t_measure/ts_inv);
s=tf('s');
z=tf('z',ts_inv);
%[text] ### 
%[text] ### Simulation parameters: speed reference, load torque in motor mode
% rpm_sim = 3000;
rpm_sim = 18;
% rpm_sim = 15.2;
omega_m_sim = omega_m_bez;
omega_sim = omega_m_sim*number_poles/2;
tau_load_sim = tau_bez/5; %N*m
b_square = 0;

torque_overload_factor = 1; % wind torque overload

%[text] ### 
%[text] ### Luenberger Observer
Aso = [1 ts_inv; 0 1];
Cso = [1 0];
% p2place = exp([-10 -50]*ts_inv);
p2place = exp([-50 -250]*ts_inv);
Kobs = (acker(Aso',Cso',p2place))';
kg = Kobs(1) %[output:9ba2d56b]
kw = Kobs(2) %[output:55740fe5]
%[text] ### 
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:68225c64]
luenberger_l2 = Klo(2) %[output:1030e557]
luenberger_l3 = Klo(3) %[output:1ba1541d]
omega_flt_fcut = 10;
% phase_compensation_omega = -pi/2-pi/12; % for motor mode
phase_compensation_omega = 0; % for generator mode
%[text] ### 
%[text] ### Control settings
id_lim = 0.35;
%[text] #### 
%[text] #### Rotor speed control
kp_w = 2.5;
ki_w = 18;
iq_lim = 1.4;
%[text] #### 
%[text] #### Current control
kp_i = 0.25;
ki_i = 18;
kp_id = kp_i;
ki_id = ki_i;
kp_iq = kp_i;
ki_iq = ki_i;
CTRPIFF_CLIP_RELEASE = 0.001;

k_kalman = 0; % for model predictive
%[text] #### 
%[text] #### Field Weakening Control 
kp_fw = 0.05;
ki_fw = 1.8;
%[text] #### 
%[text] #### BEMF observer
emf_fb_p = 0.2;
emf_p = emf_fb_p*4/10;

emf_fb_p_ccaller_1 = emf_fb_p;
emf_p_ccaller_1 = emf_fb_p_ccaller_1*4/10;

emf_fb_p_ccaller_2 = emf_fb_p;
emf_p_ccaller_2 = emf_fb_p_ccaller_2*4/10;

% omega_th = 0.25;
omega_th = 0;
%[text] #### 
%[text] #### EKF BEMF observer
kalman_psm;
%[text] #### 
%[text] #### Speed obserfer filter LPF 10Hz
fcut_10Hz_flt = 10;
omega_flt_g0 = fcut_10Hz_flt * ts_inv * 2*pi;
omega_flt_g1 = 1 - omega_flt_g0;
%[text] #### Motor Voltage to Udc Scaling
motorc_m_scale = 2/3*Vdc_bez/ubez;
inv_m_scale = motorc_m_scale;
%%
%[text] ### Settings Global Filters
setup_global_filters;
%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] ### HeatSink settings
heatsink_liquid_2kW;
%[text] ### DEVICES settings (IGBT)
infineon_FF650R17IE4D_B2;
% infineon_FF650R17IE4
% infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.inv = device_igbt_setting_inv(fPWM_INV);
igbt.inv.data = 'infineon_FF650R17IE4D_B2';

infineon_FF650R17IE4;
% infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.afe = device_igbt_setting_afe(fPWM_AFE);
igbt.afe.data = 'infineon_FF650R17IE4';

%[text] ### DEVICES settings (MOSFET)
infineon_FF1000UXTR23T2M1;
mosfet.inv = device_mosfet_setting_inv(fPWM_INV);
mosfet.inv.data = 'infineon_FF1000UXTR23T2M1';

infineon_FF1000UXTR23T2M1;
mosfet.afe = device_mosfet_setting_afe(fPWM_AFE);
mosfet.afe.data = 'infineon_FF1000UXTR23T2M1';

%[text] ### DEVICES settings (Ideal switch)
silicon_high_power_ideal_switch;
ideal_switch = device_ideal_switch_setting(fPWM_AFE);
%[text] ### Lithium Ion Battery
ubattery = Vdc_nom;
Pnom = 250e3;

typical_cell_voltage = 3.6;
number_of_cells = floor(ubattery/typical_cell_voltage)-1; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 50; %Hr*A

Vbattery_nom = ubattery;
Pbattery_nom = Pnom;
Ibattery_nom = Pbattery_nom/Vbattery_nom;
Rmax = Vbattery_nom^2/(Pbattery_nom*0.1);
Rmin = Vbattery_nom^2/(Pbattery_nom);

E_1 = -1.031;
E0 = 3.485;
E1 = 0.2156;
E2 = 0;
E3 = 0;
Elog = -0.05;
alpha = 35;

R0 = 0.035;
R1 = 0.035;
C1 = 0.5;
M = 125;

q1Kalman = ts_inv^2;
q2Kalman = ts_inv^1;
q3Kalman = 0;
rKalman = 1;

Zmodel = (0:1e-3:1);
ocv_model = E_1*exp(-Zmodel*alpha) + E0 + E1*Zmodel + E2*Zmodel.^2 +...
    E3*Zmodel.^3 + Elog*log(1-Zmodel+ts_inv);
figure;  %[output:89229919]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:89229919]
xlabel('state of charge [p.u.]'); %[output:89229919]
ylabel('open circuit voltage [V]'); %[output:89229919]
title('open circuit voltage(state of charge)'); %[output:89229919]
grid on %[output:89229919]
%[text] ## C-Caller Settings
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

%[text] ## Remove Scopes Opening Automatically
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);

%[text] ## Enable/Disable Subsystems
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

if use_torque_curve %[output:group:60574c07] %[output:53539ee2] %[output:31f5f64d] %[output:8a760b15] %[output:722122d1] %[output:42fd435f] %[output:16ccb877] %[output:34a90421] %[output:184f8e5e] %[output:7a95fc70] %[output:71e15026] %[output:1cc21879] %[output:0e59b047] %[output:2e649be8] %[output:3d018231] %[output:71ecb134] %[output:0730e8cc] %[output:1478abb6] %[output:2f0e84db] %[output:1e7366c9] %[output:219bebc6]
    set_param('mpc_psm/fixed_speed_setting', 'Commented', 'off');
    set_param('mpc_psm/motor_load_setting', 'Commented', 'on');
else
    set_param('mpc_psm/fixed_speed_setting', 'Commented', 'on');
    set_param('mpc_psm/motor_load_setting', 'Commented', 'off');
end %[output:group:60574c07]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":37.9}
%---
%[output:0db165d8]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:255f588d]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:4a633a6c]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepPOSxi","value":"   0.500000000000000"}}
%---
%[output:8701c749]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGxi","value":"     0"}}
%---
%[output:73d68fef]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGeta","value":"   0.500000000000000"}}
%---
%[output:793416a0]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  15.921682195383369"}}
%---
%[output:75687a85]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.064017382188212"}}
%---
%[output:93a00ed0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.064017382188212"],["15.921682195383369"]]}}
%---
%[output:832d52f7]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:4c95808e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:6cc95407]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000041666666667"],["-4.112335167120566","0.999345501530502"]]}}
%---
%[output:99aa498b]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.064795348480289"],["11.319202547499357"]]}}
%---
%[output:9ba2d56b]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.012443765785704"}}
%---
%[output:55740fe5]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   0.517590710054527"}}
%---
%[output:68225c64]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.152987786896029"}}
%---
%[output:1030e557]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"  93.745795204429072"}}
%---
%[output:1ba1541d]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -1.166194503484206e+02"}}
%---
%[output:89229919]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAhMAAAFACAYAAAAGUf7gAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ2UVsWZ50snH7QaA68foy0SlDTRxASUMdNBE5Mwxs1OwNmcmaGbPbsZ0sl2TkRnVlgacHSO0fA14BlHzQ6bbXt1d2jIntENzJ6McYjjCdMhMaidTKKxFQGhYYMgUSNsvth9bltNdfX9qHvv\/7633vv+33NyIv1WPbfq99St+r9PfZ1y4sSJE4ofEiABEiABEiABEshI4BSKiYzkmI0ESIAESIAESCAgQDHBhkACJEACJEACJJCLAMVELnzMTAIkQAIkQAIkQDHBNkACJEACJEACJJCLAMVELnzMTAIkQAIkQAIkQDHBNkACJEACJEACJJCLAMVELnzMXDUCR44cUV1dXUG1ent7Va1WK6yKQ0NDauHChWrWrFlq9erVqqWlpbBnmYbrWUeXCmkON954o5o\/f75LFq\/TrFmzRm3YsCEoY3d3t+rp6UlV3s2bN6vly5erVatWeclD6rdjx47C349U0Ji4dAIUE6W7gAXwiUA9B9ooMSGd9TXXXKPa29sLQRNVx6KfG1WZLIOTDGaPP\/54qoE6S560DpBnLFiwYDRbFcVE1cRfWh8zfTgBigm2DBLwhMCxY8fUsmXL1NatW9XGjRvrJiYkIlKP54Zh1gPT3LlznYWB\/uWeZqDOkidLs9BiIk3Z7Of4HpnQ7XTv3r2MTmRpJBXNQzFRUcf6UC0z3CvlMcO25q\/ymTNnqjvuuCMosgwqZshf\/4oeHBwc971p4\/rrr1ef+9zngjStra2qr69PtbW1hWIwB207vf2rXb7X0x5z5sxRd911l5oxY0bQiZqDcJIdmS7Rz925c2dQPvnoaY7bbrtNfelLXwqEhP7YLOTvmqkpNvQAZqbXA5K2ZQ5uZh3vvfdetXbt2tDn7tu3Lyjf8PDwaJlMH5ochfl9992n7r\/\/fqXrJ\/xt1pqdnj4KGzi1X83n6vra9dK+njx58qggsvlt2bIlmDbQH7N92PaSRJzdHuNsxbXDuAiGyUTKrMtuCxT7\/TLZahs333yz2rZtm5L3R\/vOrLP8TT\/D9G0Sl7B26EOfwzKUR4Biojz2lX2yPYCYFdUdYtiAYQ8CYkcGci0k7O\/DBru4gVi+iyqb7vjPOuusMWsmtJgwyyCDdtjgbwoK2w5KTIT98rU7dnuQieIqf48SE0uXLlWLFi0axz5u8JbvzjnnHHXo0KFALIUN8PJMc9Czyx7VLvRzn3zyyVBh8NBDD42uUzDbmzlY2mLCtqW\/jxIUcW1W8uzZsydStJhlsoWELfj0QP7hD39Yffvb3x7TT4QJgrD3yxYDkiasjPJ3\/Zwk2yYX36Mnle1cPa4YxYTHzmnUounO0vxlpjtiqZP5q1x+fepOyuyszY7P\/EVmDj4yYOtfztqGfrb9C1izDPve7BivvfbaSDFh\/nJLaydJTEg0Rj5J0w1xkROJlhw+fHgcE\/PXtHCaPn36mDq6THPYUzCavfanRCFsv0tZZP1AWMREWM6bNy+orxnJcFmU6jJlYaex\/62ZaOEj5U96tm57YfXRfxPRKXWOmuYwOer2ZPv00UcfDURJWKQhyq4dndLRGNNG3LN15EK3\/yQuiOmcRu3fWO5wAhQTbBlwAlG\/WnRnLJ3oFVdcEbqTwUyze\/fu0F+bUmDThvwa1jsvkhZQJv2iihqszc5Vnp\/WDkpMmM8WYSAfc\/CK6uTNwfTzn\/+8s5gIi+SEPVfKYU\/jRP3yl7QyKOpymGzDnmdP98SJiajpHTtPXJQhTIjaddNTaLYo0QIqatBPap+mf00bUX61oxyalRYTUdNb5k4lsy3r99KcYtIdhMklbGoN3pHQYEMRoJhoKHc1RmHrLSbMrZVJnXVaESDEw7aKutoxB0p74BHb5tZQl8iEpDEXLcq\/ZRuiHZmxB7O0YsLmaEcvbBGDEhO6hUeJGNnhEiYm7OmSpMhEI4iJsEiY9qvd\/qIiE6aNqHeDYqIx+lXfS0kx4buHGrB8Wac57HC8noOO+pUXFpZOEhNx0xPmr2XBLr\/eosSEqx0JH9sDvZ7+scWEDNguC9vsgdb85W5PFcngmzTNIVGTpMHYtpF1msNszlG\/9u0mbwsD+1e6rrMZodL10W3HzhM2zZH0qhU9zaGFp47oRImJsIiOZmRHJqIWzNpTLHHTHGFcOM2R1Fqa73uKiebzeeE1LnoBZtxgnCQm4soWtp4gSkwk2ZGQsF7\/YAN3EROSJ2w3h7Zlr8g3D3tKswBTh7vNPPLcT3\/600HUJOwjnMLq57oAU2xqgWWLmKjFiWYeM4088+6771Z33nnnuMWikscWE\/K3qMWcuq5J4jVsCiApMmRyjKpjnBAwB++bbropsm3F2ZAyhC3MdF2AaXJJiswV3snwAd4RoJjwziXVKZDr1lBzW2fS1tCwRZ1ppjmEblwIPWmBo3kiZpwdeY69jfDWW29VTz\/99OiCw7DIhDnQRC0iNW3baznCxIY5qJp55b+1mAh77le\/+tUxJznKQVrm+owsW0NNUWAObmHbhqO2pNpczTUcYlO4rVu3Ti1ZsiTAYUaY9K6cqK2mSedDxG0NlWe5\/mKPWusg0amwgToqGiOMwrblhkU3ooSo\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\/iZ\/\/YvausPDlFM5MMYH5mQbzdu3Oh8FkVSWeIOy4p6DreGJlF1+54djhunpFTkmETI\/XuydGcVl5Ic83HUZ0yc+sbL6uW\/+aN8xjzJfcqJEydOlFEWvU1z69atwePnzp2rVq9erVpaWgotDiMTheIdY5wdDoY1OWI4ihWyxLAkx+wczZ0cb3n5J+qnD34huzGPctZdTJi7OdBRCBeuFBMulDBp2OGQI4YAzgrbJIYlOWbn2P\/EQSU3hsrnjO1r1d7vfSO7MY9y1k1MmNMN9YpCZOXMaY6s5MbmY4dDjhgCOCtskxiW5Jido57imFKboF69\/9+qXbt2ZTfmUc66iYnnn39e3XTTTeq2225zXg\/Buzk8aikZisIOJwO0kCzkiOEoVsgSw5Ics3E0t4R+pG2S+sFffppiIi1KX+7mcCk3IxMulJLTsMNJZuSSghxdKLmlIUs3TkmpyDGJUPj3craECAr5PP3nH1If\/Z33UkykRckryNMSa\/z07HAwPiRHDEdGJsgRRyC9JXOtxNXTJqotN1yuqvTDtW7THOnRl5ejSg4ujyJDyij2FBMokmyTKJJsk+lIyg4OiUrI\/8taiXs7LlVXv3sixUQ6jI2XmmIC4zN2OOSIIYCzwjaJYUmO7hxFQCzqf0Ztf2FkeuO7y35XtZ17WvDfVRprGJkIaRNVcrB7k8enZIeDYUqOGI6c5iBHHAE3S7aQkKiErJXQnyqNNRQTFBNub0WGVBwEM0ALyUKOGI4UE+SII5BsyZzakNQiJLZ88fLg\/ykmkvlVIkWV1GKZDuEgiKFPjhiOFBPkiCMQb8lFSIiFKo01jEwwMlHY+8VBEIOWHDEcKSbIEUcg2tL\/evqn6rMP\/mg0QVhEgpEJsCc2b96sli9fHliVS7f27NmjBgYG6nJHR1xVqqQWwS5LZY6DYCpckYnJEcORYoIccQTGW5LzIxZteibYsaE\/D\/zJZWruB86JfGyVxprSIhNyR8fw8LBaunSpWrRokerp6VEzZsxQy5YtU62trcG\/y\/pUycFlMWTHjSNPMUGWOAIYS2yTJzmKiFj7yIujuzXkG3P7Z7P8cC1FTJinYU6fPl11dXUF4qG9vV3pI7R7e3tVrVbDtPyUVigmUgKLSM4OhxwxBHBW2CYxLMlRBSdZ2pEIERHr\/vA96vcucRu7qjTWUEyEvFtVcjCm68hmhR1ONm52LnLEcGS0jBwRBL72\/YPqCxtHbv3UHxERV02bqHquu2jMbo2k51VprClFTAhgWS8h6yPMaQ4dpejo6FDz589P8kNh31fJwYVBcjDMQdABkkMScnSA5JiELB1BJSRrJo56DYR58JQpIj46vab+6o\/fkwlslcaa0sSEkJcpjQULFoxxwqpVq0oVElKYKjk4UwsHZWqmDgeELNQMOeLokiWGZTNwDFsLYYoI+8yILGSrNNaUKiaywK9Hnio5uB68op7RDB1OPfiSI44yWWJYVo2jvjPjH585rP76W3vHLKY0BUSWqYw44lUaaygmQjxdJQdjuo5sVqrW4WSjkD8XOeZnqC2QJYZlVTjax13bdPRaiM4rzw8u5kJ\/qjTWlCImXK8j7+7uLmWLaJUcjG78aexVpcNJU+ci0pIjjipZYlg2KkeZuuh\/4oCS68CjPiIg5lxylvo3M88tRECYz63SWFOKmBCYsgBz06ZNytwCqkWGLMCcN29eaWdOVMnBmK4jm5VG7XCy1ba4XOSIY0uWGJaNwDFu4WRYBEKuBRchYd6dgaEVbaVKY00pYsI8Z0LOljA\/5jkTzz33nJLDrR5++OGifTrGfpUcXFdw1sMaocMpk4\/rs8nRlVRyOrJMZuSSwjeOWjis\/eZutffwsdA1D2a9RDD8p09MVe+qtRQefYjjWaWxhmIixNNVcrBLx1BUGt86nKLqWbRdcsQRJksMyzI5inCQ\/31t50G1+2U34TBl0gR148enqJa3\/lap4sGmX6WxphQx4TLNIedM6LMo7r77bswb4GilSg52rHIhycrscAqpUElGyREHniwxLOvBUUcbdr38hrrr0T1q7ysjIiLpYx4gJWnrOW2RVDaKibSEHNOHnTMhF37J1IcIiXvuuUf19fWptrY2R4uYZBQTGI716HAwJfXbCjni\/EOWGJZojrIwUgZ912kKXYtGEg5h5Ks01pQWmcA06WKsVMnBxRBys4rucNyeWr1U5IjzKVliWGblqKcotv7gkHrmwOuJaxtM0SD\/LTdwfv7qycGffY44uFKu0lhDMRHi9So52LVRF5Eua4dTRFka2SY54rxHlhiWSRwl0vDd3T9Tj\/\/kiPP0hCkcZI3Doo9NUae9za81Dhh6J61UaawpTUwMDQ2phQsXBteQ2x+5ipy3hqKbbf3tJXU49S9RYz6RHHF+I0sMS+G4\/9eTlKxn+O6LP1MvHTnuHGWwRcNnr7pAnX3G2+q+LRNDIp8Viol8\/NSxY8eCMyRmz549ep5EZ2ensq8jz\/mYzNmr5ODMEAAZ2XEDICqlyBHDUayQZTqWEmHIKxjkiXIM9fzfOU+desopXu2mSEcDn7pKY00pkQn7nAk5S2Lq1KnBBV+yKLO\/v1+tXr1atbS04L3nYLFKDnaobmFJ2HFj0JIjhiPFxHiOeg3D9\/e8qr717OHUUxJ2lOGT7z9Hvb\/1DAoGxyZbpbHGCzEhOzd2794dHJ1tHlpVq9UcXYJNViUHY8mks8ZBMB2vqNTkiOHYTGJCX1ylt1L++MDr6h9+dFjtOvRG6ukIWzB89D01NWXCG+q8885vyqkJXGus1g3VpYgJcYZEI+RjC4hHH31UDQwMMDKBbLEl2eIgiAFPjhiOVRUTMhXx2vFfqb\/\/4aFMaxdMsSD\/LYsfOz94vrpw0smjpe2dE2yTmDZZpR+upYkJc92ETG+IuNiwYYNqbW0t5WwJs2lUycGYJp\/NCjucbNzsXOSI4dhoYsI8oGnbs4eVTEVkWeho0gvunpg0QV16\/ulq7gfODb7SQiHNVku2SUybrNJYU5qYwLiiGCtVcnAxhNysssNx45SUihyTCLl\/7xtLEQx7jhxTcu7Cswd+nnnNgj0VcWFtguq57qLgxMgiLq\/yjaN7C\/ArZZXGmlLEhOtFX1wz4VfDT1sadjhpiYWnJ0cMx3pHJmT6QT4Du46q7UOvBP+9\/YWRv2X96MhC22+frmZNeYe6+t2TRk2liSxkfb7OxzaZl+BIfoqJnBwpJnICbJDs7HAwjiJHDEekmNDHP\/cN7Fc797wKEwpiSKYhPnnZ2er9F7wjsHv1uyfiAIAssU1iQFJMZOQouzaWL1+emLu7uztYmFnWp0oOLoshsuMusw4+PJsdN84LSSy1SPjNb06ov35sr3r+p2\/knnoIBEJtQlAJEQrvu+AM9fuXnTPyb\/33N\/8fV9NiLSVxLPbp1bFepbHGu2kOH5pJlRxcJk92OBj65IjhKELhlJ8fUq+cOjHYJrn38DG4UJj+26erT33gHHXx2SfPyKnn9AOGVLIVtslkRi4pqjTWlCImXCCXmaZKDi6TIzscDH1yjOaodzzI\/7\/xi18HCxn3gESCHVGQaYf3WQcyVVEouLRatkkXSslpqjTWUEyE+LtKDk5uzsWlYIeDYduMHM1tkb\/49W\/U\/f+8X\/3L\/tcDoHkXMWqv6MWMsvPh8gvPVJecd\/ro1EOzigTXFtuMbdKVTZp0VRpr6iYm9KLLwcHBRNa86CsRUUMkYIeDcVNVOOptinpdgtz5sHPva8HNkmiREEQVJk1Q0887XX3ggjPUR6fXgm2SMs1x1Yw2jGOa2EpV2mTZLqSYKNsDBT+\/Sg4uGFWseXY4GPqNwlGLhJ++9gv14I5htftlzJoEM5KgRYJEDv7V+85WE097a6pFjI3CEtNyirNCjhi2VRpr6haZwKCvj5UqObg+xMKfwg4HQ79MjuaahKdeelU9e\/DnwSmMe185HvzSR3zMnQ4y5bBw9gXq+C9\/MzrlEAgI0G6HMlkiWPligxwxnqjSWFOqmAjbKrpq1arg9tAyP1VycJkc2eFg6BfBUUcRjr7xS9U3MKxeOPQGdKrBFAAy3XDJ+aer37vkLHXa234reE5ZZycUwRLj5cayQo4Yf1VprClNTIiQ2LRpk+rt7VX6pEu9rqKjo6NUQVElB2OafDYr7HCycbNzuXLUAkEiBnKXgz5MCRlFsEXCh9smqfaLJ6p31SYUdnQzhuKIFVeWyGdW0RY5YrxapbGmFDHBEzAxDdF3K+xw8nlITyN878cvqg++96Jg26NcJY2eZrAFwrvPPU21X\/RO1Tpx5KAliSLosqCmG\/KRyZ6bbTI7OzMnOWI4Ukzk5EgxkRNgg2RnhxPuKBmY9c6GTd8\/GJyLULRAkLUIcqW03OUggqCoC6B8b5pskxgPkSOGI8UEgCOnOQAQPTfRTB2OOcXwvd0\/U\/\/05nZH9BSDHUWY9a4z1cfeU1OnnnLKmChCo0cQimrazdQmi2LI6SIcWYoJEEsuwASB9NRMI3fc5k4FOQ9h27NH1OBLrwWkixYIciSzXPT09recGkQRXnrpJXXhhRfCdjR42lzqUqxGbpN1AeT4EHJ0BJWQjGICw9FbK1VycJmQfepwTHEg\/73\/6HH17aFXRtcCFC0QLrvgjCCC0PLWkd0MwemLxnRDnJ984lhme0I8mywRFLmQFUORV5Dn5ujLro2oilBM5HZxYKCojlvP95tnIMjA\/Pc\/PKR+NFzMAkV7ekH+\/bsXvVNdM702CksvVERPMRTFEePlxrJClhh\/kSOGY5XGmlJ2c4gb7CmOjRs3qvb2doyHclqpkoNzosiVPWuHo9cfHPvlr9V\/3b5f\/eTgz4NyFB09mFJrUWed8VbVddUFI887cjzV6Yq5YMVkzsqxqPI0sl2yxHiPHDEcqzTWlCYmTFesWbNGbdiwIfhTa2ur6uvrU21t5Z2fXyUHY5p8Nitmh6MFwg\/3v64eHzqinj1QH4FQ9ImK2ciky8WOOx2vuNRkiWFJjhiOVRprvBATtrDYsWPHmMOsMG5zt1IlB7vXOltKEQny+eHw6+obPzxUSATBPG5ZnaLUFVPOVB9\/cwcyLbxuAAAgAElEQVSDOaWAnl7IRgSfix03jilZYliSI4ZjlcYaL8SEGZlwvTH02LFjatmyZWrr1q2BV7u7u1VPT0+oh+20Semr5OAsTd683VF+2f\/VP+4JjltGTjPY9zHI2oPWd769qc9AiPIVO+4srTg8D1liWJIjhmOVxprSxETeqQ3JLx8REEkLOuX7xYsXqxUrVjhNn1TJwXFNXi9g3P7CUbXpewcgYiHYpTBpghIRcvZb\/6+6dubU0SKUdR8D5rUvzwo7bhx7ssSwJEcMxyqNNaWIibgTMLO6yBQXto2hoSG1cuVKtX79+tF7QOKeUyUH63qKcJDjmB\/50ctKxEPajxlJmHPpWWrWlDPHnHsQNsXADictZf6axhCLtsI2iSFMjhiOVRprShETGDectJIkTmQNhogN81KxKosJEQ67Dx9T676521k4aDFw1bSJ6n2tZ6hPvf+cUURZ1yKww8G0dHLEcBQrZIlhSY4YjhQTGI4QK3q6ZO7cuWr16tWqpaVlnF17G2rSuoxGc7CerljU\/0yieNDTEPNmnKs+8d6zcguGOCeyw4E0cQ6AGIyBFbZJDExyxHBstLEmrtaViExIBUVUDA8PhwoK+7u4tGJLHGx+tm3bhmk5QCvDr\/5KHXjtV2rDd4+qnfuPh1puPfMt6vx3vEX9yax3qre\/5RQ164KRWyDr9dm3b5+aPHlyvR5X2eeQI861ZIlhSY7ZOM6ZM2dcxl27dmUz5lmuyogJWRexdOlStXbt2sRFlklpfVaLEoX4xo9eVssfHhrXlPR0xL0dl44e11xme+OvFwx9csRwZGSCHHEEMJZ8HmvS1rAUMVHEFeRp1kUkLcj01cE39D+j+p84OMbHetri3s4RAeHTh4MgxhvkiOFIMUGOOAIYS76ONVlq17Biwty9oc+RkNMz7bMm9HezZ89W8+fPV3FpNUCfHCyRCBEQax55cZyIkAiEz9stOQhmeSXH5yFHDEeKCXLEEcBY8mmsyVujuoqJsCvHwyoQdwCVTm8fRGUuwNTfdXZ2Bvd9xKUNe74vDpbTJed95amGExG6wBwE876eI\/nJEcORLMkRRwBjyZexBlGbuooJXeCkrZyIiuWxUbaDJRohIsK8NlumMLZ88XLvpjLiOHMQzNMKT+YlRwxHiglyxBHAWCp7rMHUYsRKKWICWYEibJXpYDsaISLC9+mMKB9wEMS0TnLEcKSYIEccAYylMscaTA1OWqGYCCFaloO3PXtE\/dF\/GRwt0dXTJiofF1a6NkIOgq6k4tORI4YjxQQ54ghgLJU11mBKP9ZK3cSEObUxffp01dXVpQYHTw6cZrGSDpUqAoRps94OlumMv3vq\/6g7\/vfIfuNGjkaYHDkIYloqOWI4UkyQI44AxlK9xxpMqcOt1E1MFFkJtO16OtjerVEVIcGOG9cqKSbIEkcAY4ltEsOxnmMNpsTRVigmQtjUy8EiJDZ9\/6Ba\/Q8j2z4bcZFlXANlh4N5fckRw5EClxxxBDCW6jXWYEobb6UUMaGnPJp9muO\/7zig\/vRrz1ZSSLDjxr2+FBNkiSOAscQ2ieFIMYHhOM6KiIzFixerFStWJB6JXVARArP1cLAcRCUnWlYxIqF9ww4H00rJEcORApcccQQwluox1mBKmmyllMhEXLHkWOz+\/v7IG0CTq5Q\/RdEOlumNmXd+p9JCgh13\/nZIUYZjSJZYlhS4GJ5FjzWYUrpZ8VJMyFHZvb29qlarudUCnKpIB9sHUt3XeanqvPI8cA38MMcOB+MHcsRwpMAlRxwBjKUixxpMCd2teCcmkq4Hd69a9pRFOvgvtr6g7nlsb1C4f9d+vrr7jy\/JXlDPc3IQxDiIHDEcKSbIEUcAY6nIsQZTQncrpYiJuAWYcllXX19fJddMmKdbys6Np\/\/8Q+6easCUHAQxTiNHDEeKCXLEEcBYopgAcIy6zVPf7gl4RGYTRTjYnt6QezZ8vvEzMzwjIwdBBEVe9IWhOGKFbRJDkxwxHIsYazAlS2+llMiEFDNsOkNHLDo6OoLrwsv6FOHg\/\/HdA+qmzSPbQP\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\/\/SvCgI3d9LN8DK5qbHQ7GseSI4cjIBDniCGAsuYw1mCcVb6VUMbF582a1fPnyoJZy9fiePXvUwMCAWr16tWppaSm+9hFPcHEwF18mu4eDYDIjlxTk6ELJLQ1ZunFKSkWOSYTcvncZa9wslZ+qNDEhiy\/lwKqlS5eqRYsWBedNyFqJZcuWqbLPn0hysLn48sPvnqS+\/sWZ5XvSwxKww8E4hRwxHBmZIEccAYylpLEG85T6WClFTJjnTEyfPl11dXUFYqK9vd2LXR5JDubiS7fGyUHQjVNSKnJMIuT+PVm6s4pLSY4YjkljDeYp9bFCMRHCOcnB\/U8cVDf0PxPk5GFV0Q2VHQ7mJSZHDEdGJsgRRwBjKWmswTylPlZKERNSNVkvIesjzGkOHaXo6OhQ8+fPhxHQh2Rt3bo1sNnd3R17jHeSg7lews01HATdOCWlIsckQu7fk6U7K0YmMKzirCSNNcWXAPeE0sSEVEEOrlqwYMGY2qxatQoqJMS4rM+Qj0yl6CmWOMGS5OCZd35HybqJKbUJQWSCn3AC7LgxLYMcMRwZmSBHHAGMpaSxBvOU+lgpVUzUp4rjn2KKi7AyxDmYJ1+6e42DoDsr\/grEsEqywjaZRMjte3J045SUimIiiZDH37tcMhbnYHPx5X2dl6rOK8\/zuLblFo0dDoY\/OWI4MjJBjjgCGEsUExiOwboJfc6ENinnTciujiI+EpHYsGGDmjt3buxZFnEO5smX7p7hIOjOipEJDKskK2yTSYTcvidHN05JqSgmkgg5fC9CYtOmTaq3t1fVarUgh8t6BgfTiUn0GRdRh2OJg83Ptm3bRv\/5Hx46qHbuPx78e+eNUxOf1cwJ9u3bpyZPntzMCCB1J0cIxsAIWWJYkmM2jnPmzBmXcdeuXdmMeZarlDUTcVMN9bhNdGhoKNhFsnbtWtXW1jbOJXFqkTs53Fswf724s4pLSY4YjmKFLDEsyRHDkZGJnBzLFhNJgiXOwbWbHwtqf\/W0iWrLDZfnJFHt7OxwMP4lRwxHiglyxBHAWKKYAHCUAX3JkiWqr69vNDpQ1DSHuXtDnzkRd2R3lIN58mU6x3MQTMcrKjU5YjhSTJAjjgDGEsVETo5aNAwODiZakvs6Hn744cR0cQnsQ6uyLsA0T77ktePJLuEgmMzIJQU5ulByS0OWbpySUpFjEiG37ykm3Dg1bKooB5s7OXiMdrJ72eEkM3JJQY4ulNzSkKUbp6RU5JhEyO17igk3Tg2bKsrBXHyZzqXscNLxikpNjhiOYoUsMSzJEcORYgLDMfSciSKO005b3CgH62O0ufjSjSg7HDdOSanIMYmQ+\/dk6c4qLiU5YjhSTAA4lnnORFLxwxzMY7STqI3\/nh1OemZhOcgRw5GRCXLEEcBYopjIybHsraFJxQ9zMHdyJFGjmEhPyC0HxYQbJ5dUZOlCKTkNOSYzcklBMeFCKSZNI4oJcycHF1+6NQB2OG6cklKRYxIh9+\/J0p1VXEpyxHCkmABwbLRpDu7kSO90djjpmXGaA8MsygrbJIYvOWI4UkxgODbUAkzu5EjvdHY46ZlRTGCYUUyQY7EEMNYpJjAcvbUS5mC9k2NKbYKSaQ5+kglQTCQzcklBji6U3NKQpRunpFTkmETI7XuKCTdODZvKdjB3cmRzJTucbNzsXOSI4ShWyBLDkhwxHCkmMBy9tRInJniMtrvb2OG4s4pLSY4YjhQT5IgjgLFEMYHh6K0V28HmttD7Oi9VnVee523ZfSoYB0GMN8gRw5FighxxBDCWKCYwHL21YjvY3MnByIS72zgIurNiZALDKskK22QSIbfvydGNU1IqiokkQg3+ve3gG\/qfUXLOhHyO3PWxBq9d\/YrPDgfDmhwxHBmZIEccAYwligkMR2+t2A7mttBsruIgmI2bnYscMRwpJsgRRwBjiWICw9FbK7aDecFXNldxEMzGjWICwy3MCtskhi05YjhSTGA4emvFdLC5LfSj02vqoS\/M8LbcvhWMHQ7GI+SI4cjIBDniCGAsUUxgOHprJUpM9Fx3keq5bqq35fatYBwEMR4hRwxHiglyxBHAWKKYwHD01kqUmOC20HQu4yCYjldUanLEcKSYIEccAYwligkMR2+tmA7mttDsbuIgmJ2dmZMcMRwpJsgRRwBjiWICw9FbK6aDzW2hvHo8ncs4CKbjxcgEhlecFbZJDGNyxHCkmMBw9NaK6WBuC83uJnY42dkxMoFhZ1thm8RwJUcMR4oJDEdvrZgO5m2h2d3EDic7O4oJDDuKCXIshgDGKsUEhqO3VkwH125+LCjn1dMmqi03XO5tmX0sGMUExivkiOEoVsgSw5IcMRwpJjAcvbWiHWyeMcFtoendxQ4nPbOwHOSI4UgxQY44AhhLFBMYjt5a0Q42bwvlBV\/p3cVBMD0zigkMsygrbJMYvuSI4UgxgeHorRXtYLncS3ZzyIdiIr272OGkZ0YxgWFGMUGOxRLAWKeYwHD01op2MM+YyOciiol8\/HRucsRw5DQHOeIIYCxRTGA4emtFO\/jGTc+qv\/3egaCcvHo8vbs4CKZnxsgEhhkjE+RYLAGMdYoJDEdvrWgH84yJfC6imMjHj5EJDD\/TCtskhik5YjhSTGA4emtFO5hnTORzETucfPwoJjD8KCbIEU8AY5FiAsPRWyvawTxjIp+LKCby8aOYwPCjmCBHPAGMRYoJDEdvrYiD\/+n7P1YSmZAPz5jI5iqKiWzc7FzkiOEoVsgSw5IcMRwpJjAcvbUiDn7wm0+qeV95imIih5fY4eSAZ2QlRwxHiglyxBHAWKKYwHD01ootJnjGRDZXcRDMxo2RCQy3MCtskxi25IjhSDGB4eitFXHwlzcP8MCqnB5ih5MT4JvZyRHDkZEJcsQRwFiimMBw9NaKOLj7P39LrXnkxaCMT\/\/5h9SU2gRvy+trwTgIYjxDjhiOFBPkiCOAsUQxgeHorRVx8GWL\/05tf+FoUEYeWJXNVRwEs3Gzc5EjhiPFBDniCGAsUUxgOHprxRQTEpGQyAQ\/6QlwEEzPLCwHOWI4UkyQI44AxhLFBIajt1bEwWd+9m+VXEF+9bSJassNl3tbVp8LxkEQ4x1yxHCkmCBHHAGMJYoJDEdvrYiDj\/5Bb1A+ionsbuIgmJ2dmZMcMRwpJsgRRwBjiWICw9FbK1Mv+6B69RNrgvLxwKrsbuIgmJ0dxQSGnW2FbRLDlRwxHCkmMBy9tUIxgXENOxxyxBDAWWGbxLAkRwxHigkMR2+tTPngJ9XrVy8NyscDq7K7iR1OdnaMTGDYMTJBjsUQwFilmMBw9NZK67\/+j+r4JfMoJnJ6iGIiJ8A3s5MjhqNYIUsMS3LEcKSYwHD01oopJnhgVXY3scPJzo6RCQw7RibIsRgCGKsUExiO3lo57w\/vVL+YclVQPh5Yld1NFBPZ2VFMYNhRTJBjMQQwVikmMBy9tXLuv\/8b9auz3xMcoc0Dq7K7iWIiOzuKCQw7iglyLIYAxirFBIajt1bO\/sL\/VL857WyKiZweopjICfDN7OSI4ShWyBLDkhwxHCkmMBy9tVK7+bGgbDywKp+L2OHk46dzkyOGI8UEOeIIYCxRTGA4emtFi4nOK89T93Ve6m05fS8YB0GMh8gRw5FighxxBDCWKCYwHL21osUET7\/M5yIOgvn4MTKB4WdaYZvEMCVHDEeKCQxHb61QTGBcww6HHDEEcFbYJjEsyRHDkWICwzGXlSNHjqiuri41ODgY2Jk7d65avXq1amlpGWf32LFjatmyZWrr1q2j33V3d6uenp7QMmgxwdMvc7mIi93y4RvNzY4bBJILMGEg2SYxKCkmMBwzW9HiYPbs2Wr+\/PlK\/7u1tTVUIIjwWLx4sVqxYoVqa2tLfC7FRCIipwTscJwwJSYix0REzgnI0hlVbEJyxHCkmMBwhFrZvHmzGhgYCI1ODA0NqZUrV6r169erWq2W+FwtJnj6ZSIqdjj5EDnlZsfthMkpEVk6YUpMRI6JiJwSUEw4YapvojgxsWPHDrVmzRrV29ubSkzw9Mt8PmSHk4+fzk2OGI5ihSwxLMkRw5FiAsMRZkWvn+jo6AimPeyPCI3ly5eP\/nnGjBmxwkIiEzz9Mr972OHkZ8gBEMOQwowcsQQw1igmMBwhVvR6CTEWtQBTohLDw8Oj39v\/tgsiYuLUN15WZ35zZIHmtm3bIGVtNiP79u1TkydPbrZqw+tLjjikZIlhSY7ZOM6ZM2dcxl27dmUz5lmuU06cOHHCszI5F8dFSIQZkzUUS5cuVWvXrg1dkCligqdfOrshMiEjE\/kZMjKBYcjIBDliCWCsMTKB4ZjLStIOjjjjSQsyKSZyuWY0c5VeFAyRbFbIMRu3sFxkiWFJjuRoE2jYyETSVIWuaNptpJJPxARPv8z\/srDDyc9QLJAjhiNZkiOOAMZSld7thhQT9oFV2q16YaUcXCWHVHV2dqr29vbRcyj0oVVxB1xRTGBeEnbc5IgjgLNUpc4bRyW9JXJMz6zqkbKGFBMYN0ZbkcjEaU\/er96295+LfhTtkwAJkAAJNDEBLsBsYuez6iRAAiRAAiRAAicJMDLB1kACJEACJEACJJCLAMVELnzMTAIkQAIkQAIkQDHBNkACJEACJEACJJCLAMVELnzMTAIkQAIkQAIkQDHBNkACJEACJEACJJCLAMVELnzMTAIkQAIkQAIkQDFhtAE5VXPDhg3BXzZu3BgceMVPNAE5lnzhwoXBJWpJB4GZbFtbW1VfX1\/ovSjNyjsNS83IPt21WdmZ9U7D0T78ju\/82BaUhqWZlu+3+5uo32F9wKJ7Tv9SUky86ZMdO3YoGfB6e3vVc889N\/rftVrNP695UCJzIJs3b15w4ujs2bMjr4AfGBgYvbVVroTftGlT7DXwHlSxbkVIw9IslHBcvny5WrVqVSj3ulXAkwel4Wjf7ZN0+Z8nVaxbMdKw1KKsp6cn+AHG99vNTZqxnMxcBSFLMfGm30VIyEdeiCqpRbdmnT6V3fmKGOvv74+8Bt7+9Rh3a2v60jR2jiwspQNfvHixOnr0qOro6KCYUEql4Zh02V9jt6j8pU\/L0nyfKcyS+etIzqxZs9TevXuDcafRI+EUE\/9fQERdBhb1Szu5qVQ\/hRnJkeiN\/e84AuxsxtLJwlLE75VXXqm+\/vWvR0aEqt8Ks3OUX89mtKzZWCXVN02bDItMkG084f379wcJ5B6prq4uiomkBtko34dFIqSznjp1Kn\/xRTjRjkSk+aXneuNro7SfvOVMy1JYP\/DAA+rmm29Wt99+O8XEmw5Iw1HExO7du4OcXCc1vgWnYSm5zZB9d3d3MDjyk0zAFmLJOfxNwciE8SKYi2AoJuIbbdrORluTTvyee+7hAkwDbxqW0ml\/+ctfVp\/5zGfU5MmTY9eq+NvtFFOyNBz1ehM9Vy15lyxZwnaZQZjpkP26deuCUH2aKGUxLaFxrFJMNI6vnErKaQ4nTGMSpQmDUkgkCzO9+Ddpyki4P\/7442PW9nA6boRvmjZpT3NwZ8zYNkqW6fvELDkoJrJQ8zyPGYngAsxkZ9nTGkkLMLnCO5ppGpbmFlvTIkPLIwswV65cqdavX6+0KItaFGy3V77zY9tnGpYUZsn9ZVQKions7LzNya2h6VyTZusYQ8jxbNOwNC3x1\/RYrmk42p04Q\/PZWYZNc3DKyK0\/pZhw49RwqXhoVTqXxR1qoxe4yUKsqF\/TVdhbnY5YfHQi6gAwkyXFRDxx1zYpVsxDq3jQ0niuaViKGFuwYEFghCzdewWKCXdWTEkCJEACJEACJFBxAtzNUXEHs3okQAIkQAIkUDQBiomiCdM+CZAACZAACVScAMVExR3M6pEACZAACZBA0QQoJoomTPskQAIkQAIkUHECFBMVdzCrRwIkQAIkQAJFE6CYKJow7ZMACZAACZBAxQlQTFTcwaxe+QReeOEFNWnSpOBURpeP7D1\/5ZVX1LRp01ySZ0qjz\/6YMWOG6u3tdS5bo9z4qutXrzMP6v28TE5nJhIokADFRIFwaZoE0p6sWI9DbPIIgjx569kaZHCXTz1vr2wUNvX0A5\/VPAQoJprH16xpCQR8FBNpy2Ria5QBk2KihMbORzY1AYqJpnY\/K48gYB7LLPb01MFzzz03esSw\/F0fH24fL65D8WeddZbq6upSg4ODQbH05V36zomtW7cGf3eZmjCfYYb69dXbut6rVq1S8+fPH4chKp0WE\/PmzVN33HFHkM+eSjCPVrafI6wWL16sPvKRjwT5dV0OHz6s9HHikufWW29VW7ZsUWvXrlVtbW2Bmag6hfnQFhPy79deey34n+YYdzmafXmVPCPsb40otBBtnjZIwCZAMcE2QQI5CIRdtmUOZHYUIOqGRSnC6tWrldgTQSHh+fb29tH7Izo6OkYH\/bgbWHV5tL2WlpZgELznnntUX19fMDAnRSbs9OZFTiJ4ZNCfNWtWUF5tf9OmTcHaCxEFS5cuHSMCTHtaME2ZMmU0vxZjuo7634cOHQrKPHnyZLVs2bJAtOhpi6TL48LExIYNG5QWT\/Yz7SZAMZHjpWDWpiRAMdGUbmelUQSSBqWkgdv+xWuLibCr3eNuCw2bhrDTx5Up6SZS+4ZIKX\/S1If5vRYTtjgaGBgYFRdi0xQL8m\/zanHtu7ipjDAxMTw8PO4Zki5sASrFBOoNoZ1mIUAx0SyeZj0LI2BOCdhTEFEDtz0VMHfu3NDIhD3dYFYibIoi6tJSDEUAAAJVSURBVHnmzaNxYiJpAWiYcAj7mz31Y0\/l6MiLnr6Q\/zcXS5o2Jdqhb6S0nRg1VREmJuKeoadStH2KicJeFxquKAGKiYo6ltWqPwFzADXXTZi\/frU4sNcx6F\/mdmRC8tq\/qONqFiUU4qZeTHt5xYTY0msftNgJi0ykERNPPvmk0tMorttrKSbq3\/75xOYmQDHR3P5n7QsgYA7I+pe3hNJlfYHM\/c+ePXvMokdTMNhiIm59RFjR6zHNYa+JMJ8pA3\/clIWe5jDFRFgUwJzmkMjEkiVLRtd8uLisiGmOJGGXNN3jUm6mIYFGJUAx0aieY7m9IBC2ZsKMDpgLEsMWEupIhZ7mkEqZgkPbl8WYehogbN2ChoFagGlGAsx1FFdcccW4BZa2mDDz6rJK+WQxZZiYcF2AKTb0os+ktSp5F2DqaSi9A0fXw1x4ajdAigkvXkkWoiQCFBMlgedjq0NADzQyHSEfcwrD3NYpYf9rr712zPZPERHXX3+9uu2220Z\/edsCQ0cr9JZReYYe5KIoxm2jdF0Uunz58lHzYVMWep2BPYjaz163bl2wrVMWXer6m5EJeYjN0N4aam+PlTxR21p1NEj+XwuwsK2hZv4wIWCuVxE\/zZw5Uz399NOBoLFFn66DHbWpTitnTUggngDFBFsICZCAdwRkcA\/bweFaUJc1E662XNMxMuFKiumqSIBioopeZZ1IoIEI2OtCdBTCPFcibXUoJtISY3oSyEeAYiIfP+YmARIAELBPBY07ndLlcfbFWw899FCQrai7OnjRl4tXmKbKBP4fFzKiXNPEMDIAAAAASUVORK5CYII=","height":0,"width":0}}
%---
%[output:53539ee2]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Error resolving Custom Code."}}
%---
%[output:31f5f64d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: C:\\Git\\GitHub\\modelization-and-control\\power_electronics_design_with_simscape\\mpc_psm\\include specified in custom include directory paths string does not exist in any of the following search directories:\n\""}}
%---
%[output:8a760b15]
%   data: {"dataType":"warning","outputData":{"text":"Warning: C:\\Git\\GitHub\\modelization-and-control\\power_electronics_design_with_simscape\\mpc_psm\\src specified in custom include directory paths string does not exist in any of the following search directories:\n\""}}
%---
%[output:722122d1]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"Goto\" for \"From\" '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/global timing\/From9','error')\">mpc_psm\/global timing\/From9<\/a>' not found"}}
%---
%[output:42fd435f]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/battery\/single_phase_ui_measure\/Goto','error')\">mpc_psm\/battery\/single_phase_ui_measure\/Goto<\/a>' not found"}}
%---
%[output:16ccb877]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/battery\/single_phase_ui_measure\/Goto1','error')\">mpc_psm\/battery\/single_phase_ui_measure\/Goto1<\/a>' not found"}}
%---
%[output:34a90421]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/global timing\/Goto3','error')\">mpc_psm\/global timing\/Goto3<\/a>' not found"}}
%---
%[output:184f8e5e]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/Goto1','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/Goto1<\/a>' not found"}}
%---
%[output:7a95fc70]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/EKF_bemf_observer\/extended_kalman_observer\/Goto1','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/EKF_bemf_observer\/extended_kalman_observer\/Goto1<\/a>' not found"}}
%---
%[output:71e15026]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/EKF_bemf_observer\/extended_kalman_observer\/Goto3','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/EKF_bemf_observer\/extended_kalman_observer\/Goto3<\/a>' not found"}}
%---
%[output:1cc21879]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Matching \"From\" for \"Goto\" '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/speed_reference_generator\/Goto','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/speed_reference_generator\/Goto<\/a>' not found"}}
%---
%[output:0e59b047]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/From'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/From','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/From<\/a>' is not connected."}}
%---
%[output:2e649be8]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/From10'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/From10','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/From10<\/a>' is not connected."}}
%---
%[output:3d018231]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/From11'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/From11','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/From11<\/a>' is not connected."}}
%---
%[output:71ecb134]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone<\/a>' is not connected."}}
%---
%[output:0730e8cc]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/double_integrator_observer\/Dead Zone<\/a>' is not connected."}}
%---
%[output:1478abb6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone'], 'Inport', 1);\">Input Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone<\/a>' is not connected."}}
%---
%[output:2f0e84db]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/encoder-sensorless-switch\/double_integrator_observer\/Dead Zone<\/a>' is not connected."}}
%---
%[output:1e7366c9]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/psi_alpha_hat'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/psi_alpha_hat','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/psi_alpha_hat<\/a>' is not connected."}}
%---
%[output:219bebc6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: '<a href=\"matlab:slprivate('open_and_hilite_port_hyperlink', 'hilite', ['mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/psi_beta_hat'], 'Outport', 1);\">Output Port 1<\/a>' of block '<a href=\"matlab:open_and_hilite_hyperlink ('mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/psi_beta_hat','error')\">mpc_psm\/inv_psm_mod1\/inverter_ctrl\/inverter_control\/inner_ctrl\/bemf_observer\/bemf_observer\/psi_beta_hat<\/a>' is not connected."}}
%---
