preamble;
%[text] ### 
%[text] ### Simlength, model name, thermal model enabling
simlength = 2;
transmission_delay = 125e-6*2;

model = 'inv_psm';

use_mosfet_thermal_model = 0;
use_thermal_model = 1;
if (use_mosfet_thermal_model || use_thermal_model)
    nonlinear_iterations = 5;
else
    nonlinear_iterations = 3;
end
load_step_time = 0;
%[text] ### 
%[text] ### Local time allignment to master time
kp_align = 0.6;
ki_align = 0.1;
lim_up_align = 0.2;
lim_down_align = -0.2;
%[text] ### 
%[text] ### Number of modules
number_of_modules = 1;
enable_two_modules = number_of_modules;
%[text] ### 
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
%[text] ### 
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
%[text] ### 
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
%%
%[text] ### 
%[text] ### Grid Emulator Settings
grid_emulator;
%%
%[text] ## 
%[text] ## AFE Settings and Initialization
%[text] ### Switching frequencies, sampling time and deadtime
fPWM_AFE = 4e3;
tPWM_AFE = 1/fPWM_AFE;

dead_time_AFE = 3e-6;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;

ts_afe = 1/fPWM_AFE/2; % Sampling time of the control AFE as well as INVERTER
tc = ts_afe/300;

s=tf('s');
z_afe=tf('z',ts_afe);

% t_misura = simlength - 0.025;
t_misura = 10/omega_bez*2*pi;
Nc = ceil(t_misura/tc);
Ns_afe = ceil(t_misura/ts_afe);
%[text] ### 
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
%[text] ### 
%[text] ### FRT Settings
enable_frt_1 = 0;
enable_frt_2 = 1-enable_frt_1;

% deep data for frt type 2
deepPOSxi = 0.5 %[output:57086c1f]
deepNEGxi = 0 %[output:08bc4522]
deepNEGeta = 0.5 %[output:858ca950]
%[text] ### 
%[text] ### FRT, and other fault timing settings
test_index    = 25;
test_subindex = 4;

asymmetric_error_type = 0;  
% 0 -> Variant C, two phase, 
% 1 -> Variant D, single phase

start_time_grid_switch_open = 1e3;
start_time_LVRT = 2.0;
time_start_motor_control = 0.035;

time_aux_power_supply_fault = 1e3;
time_phase_fault = 1e3;
start_load = 0.25;
%[text] #### 
%[text] ### FRT gain factor for grid support
settle_time = 0.175;
k_frt_ref = 2;
%[text] #### 
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
l1 = Kd(2) %[output:55bd5b41]
l2 = Kd(1) %[output:079a03a1]
%[text] #### 
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:72559c6d]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:1f057a07]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:62267490]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:5c6ec4ae]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:5060bba0]
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

ts_inv = 1/fPWM_INV/2;
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
%[text] ### 
%[text] ### Luenberger Observer
Aso = [1 ts_inv; 0 1];
Cso = [1 0];
% p2place = exp([-10 -50]*ts_inv);
p2place = exp([-50 -250]*ts_inv);
Kobs = (acker(Aso',Cso',p2place))';
kg = Kobs(1) %[output:8a628839]
kw = Kobs(2) %[output:8eb13f99]
%[text] ### 
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:6936d375]
luenberger_l2 = Klo(2) %[output:978c2b65]
luenberger_l3 = Klo(3) %[output:6c39f3fc]
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
% infineon_FF650R17IE4D_B2;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.inv = device_igbt_setting_inv(fPWM_INV);
igbt.inv.data = 'infineon_FF1200R17IP5';

% infineon_FF650R17IE4;
infineon_FF1200R17IP5;
% danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.afe = device_igbt_setting_afe(fPWM_AFE);
igbt.afe.data = 'infineon_FF1200R17IP5';

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
figure;  %[output:530d5a39]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:530d5a39]
xlabel('state of charge [p.u.]'); %[output:530d5a39]
ylabel('open circuit voltage [V]'); %[output:530d5a39]
title('open circuit voltage(state of charge)'); %[output:530d5a39]
grid on %[output:530d5a39]
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
%   data: {"layout":"onright","rightPanelPercent":13.3}
%---
%[output:0db165d8]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"1.4559e+05"}}
%---
%[output:255f588d]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"789.7124"}}
%---
%[output:57086c1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepPOSxi","value":"0.5000"}}
%---
%[output:08bc4522]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGxi","value":"0"}}
%---
%[output:858ca950]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGeta","value":"0.5000"}}
%---
%[output:55bd5b41]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"44.7824"}}
%---
%[output:079a03a1]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"0.1839"}}
%---
%[output:72559c6d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.1839"],["44.7824"]]}}
%---
%[output:1f057a07]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.0001"],["-9.8696","-0.0016"]]}}
%---
%[output:62267490]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.0156"],["2.7166"]]}}
%---
%[output:5c6ec4ae]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.0000","0.0001"],["-12.3370","0.9980"]]}}
%---
%[output:5060bba0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.1944"],["33.9576"]]}}
%---
%[output:8a628839]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"0.0370"}}
%---
%[output:8eb13f99]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"1.5335"}}
%---
%[output:6936d375]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"0.4140"}}
%---
%[output:978c2b65]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"243.8383"}}
%---
%[output:6c39f3fc]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"-299.4503"}}
%---
%[output:530d5a39]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAANEAAAB+CAYAAAC6cPTUAAAAAXNSR0IArs4c6QAAGLtJREFUeF7tXQuQlUV2Pq5agArKS3kJozwWsylFcaOL+NhyWVPLMu4S4zDUloZXEEE2hlGeQmQF5bUVQVRUisVNCcpGS0hShcboFIbFiuCwKYUFsyLiANFBgsorxEl9zZ7Lmb79\/7f\/vn\/fO\/dO\/1VTM3Nvd\/99Tp+vz+nTp0+f1dDQ0Pjwww\/TnDlzqEOHDhT1HDp0iGzKRTYQvggcKFMOnNXY2NhYyrRt2bKFRo4cSS+88AJdf\/31qZBy7NgxmjZtmmrrscceozZt2tATTzxBt912G\/Xt2zeVdyxYsIDQ95UrV6rJC39\/\/PHHVFVVlUr7eiO7d++mBx98kBYuXBhLA2hfunQpjRs3LnZSRftp9xl9HDVqFNXX19OwYcMyvI9jiGmsvDAwolHw4CxoojFjxqgiPKCF7EQpvOvFF1+kZcuW0apVq7yAqKGhQQnPfffd5w1EAC2EkyeFKL7r4I4qxwKfZp\/B5+nTpyeaEIsNIlhoShNxRzZs2KB41q1bN6PAoAIAt337dlXu0UcfVYPO9Y8cOaI+r62tbdKGXo+1BmuRhx56SAEYg8xt6oOn93H8+PE0depUNRuyJurYsaMSxltuuYXeeustGjhwoBIazKwrVqxQTfIMt2\/fviaCK9u56qqrMppo+PDhqhw\/UuOZBlAXQvyvvxuajcstWrSIHnjggQxPQRe0gOQz04o+sKDhb3yOtrlPUXw2CbypX+vXr1dCLGVg27ZtWZ+Bz3r\/Jk+erHjGMhQ1jmjb9G7IFMaRH1N9HiPZvx49eqj3StnD+LFCkNpNyiy3xTzU+X7zzTerrrRr107JEJ4o+ozmnBwofvGQIUMU42Ay6cLLQrd161YFPmZyz549afbs2TR37lzVCXQGA8WzOmZgMI6FJG4WlDMpMxzCg0cHEYMHwgpa1q5dq5gqZ\/xrrrnGCkR6n3VzTraPvkgeSXrivtM1EepVVFSoCUrOzjxJYCLAGLAwgg8S+CY+19TUZCZG08TDGkX2Gf2aP38+LVmyRPEZtGFM0b4+Ccl6u3btijSx43gSp4kYDIsXL25CK8sXyx76yVq9srJSCX51dbWSW7wbIIeM6rLH4ObvuR05EbP5rdOXc03EDoWhQ4fSPffck5n15CysA4WFF2ABgPA9D5JkBgOAZ50os4ln2BEjRmSZO3ECwX2EZoXQyUefnaM0US4QmeiBUPfr168JoFiL8ATy8ssvZ9ZEJnPOpFWwZpLmThztsl+oJ81ROaPra0nTRCYnVZMm1yfYqPHiz3ki1nnCWs+0vpWTlXSA8Rjv3btXTZT6ZIX\/pTZiK4tBpGtxniRY8+A31pKTJk3KKBCdviwQ6YPHLwWBciBsQXT\/\/fcrc0V\/AJxevXo1mbGiQBRnf8cJkmnQuB9pgUgCFW3zbGUChqQvCkQ8e2JG1DVtFBhkOROf9+zZk5mBoUl10xh1WJhMGgXAgfcW3lm2KKQmYq0Oc1w+0gyVwizXUZIncSCKslJ0kxr\/szXAZvFnn32mtI+U4SgQMcBlu2PHjqUJEyao5YaJvoxjgdc5Uesh3QtmCyJdE8lO6G3m0kRyBuN2bDQRDzw0ZNogkrNp586ds2arqFk3CkRxZmYSTST5HOcYYS3D64hnn302MxHIv8E76bGUINI1UdaM+ccP8tVEJudOHIiwrpGeW8mHfDSRTt9ZR48eVY6FuD0iVNIZYDJ\/ePZkU8Z2TcSzYNxg67Y6271Sm\/GaIWqm47UaBJtnKRZy2\/WFycUtzQVpirisiaRAysU62pVrIv27XGuiKGHihTlPNHDCsDZ9\/fXXs9avUY6ZQq2J9LUbth7gtuf+S03ETiHUYb6yVtJBhPqmNRPTK\/mStSZKstk6Y8YM+vTTT+n9999XnWaBkbNB27Zt1efSQ5LLO2cDoiTeOd3tGuUhi\/J06QLJs26U91C3y+WEJN8tzRspdDzLYxLCoF177bXKfMKDvmDgmSYf3jlpfcj10tNPP01PPfWU8hyiDDQtHqw99D7zujipd87keYza84vzzplAJJ0v+P7GG2+kTZs2ZZnJvL8o5TSRd473idici1LFPKCmvaRi++rj+lxO3+nruCT7VwAtHt3BUk78yZeWOMshqm3Ifk7vnE3HAohsuJROGanZ0GLcfox8o23EQjq9LM1W9D0laU1FUaQiFko97Kc0hyv0upw4EEBUTqMZaCkKB4oKIizY8ROewIFCcgAeSfyk9RQNRAAPNmHfeeedtGgJ7QQO5OTAyZ430MWD7qT1916dGpCagEi6keFqRbjDrFmzCK7ttI4AMJXsrkQAZvfu3XMSHwqc4QA2XXfu3Em33nornXPOOYE1CTjw6KavaNP+s+lf7myT2tGZDIhk+Ao2qVavXk0zZ85UAaObN2\/OGUKfgA5V1Mc5oKR9KNXyx48fp\/3791PXrl2pdevWpUpGUfo9cc0OWvMfB\/yASJ5cxW4ugwjg8nGiNYDIXYYCiNx55xVE6BYfNxg9ejStW7dOBd3J6FX3rp8+ByM1WgCROzcDiNx55x1E0szibtpu5sWRxZtY8pxPAJG7IAQQufNu2PL36N\/\/67Afc869W\/E1YQ7OmzePevfuTXV1dZm1FYMIIL399tt9vb4s2wWIDhw4QJdccomKYQuPPQf+cuUOfyDSg0T1bkUdkcjVfZhxeBBtvWbNmiwQ3X333XTXXXflaiZ8Lzhw4sQJFZSKgNBWrVoF3iTgwMBle1RpL945NAyBxwEuGaTInyGqVYLApt8w49hBgQBXE4ief\/55FbUcHnsOQLsfPHiQunTpErxz9myjvYeO0\/WL3\/MHoqi8cvw5zq\/gTEWu\/HSSJj1XA77D\/tPjjz8eXNwJBl8vGtZEbsx7+8PDVPmkRxDxPhEnfMDmqnQIYN3y6quvOu8XYQ1k0kRp5otzY23p1QogchuzAY\/8Vmmjbx39nP7pry5Nf7OVuyUPPuEzTroxZcqUvCIXAojcBt5UK4AoGS8BHLi24ZXDc\/6WZbTulw\/6A1Gy7rmXDi5ud94FENnzDgCCCYffeKCF2r02NVGCyFxvK1oAagBRrqGJ\/j6AyI53z2zaR9Ne2Z0p3LNDa5o58ARNnTjaH4hMjgD0wOaEnx1ZZ0oFECXl2JnyAUTRvNNNNy55Y5\/2tGxEf6rfVZd67vaMJoIXjtc9SOcElzZnjeRsnO7Dnl0zgMidmwFETXkXBRyUgvbBsQf8xuND7pqAiANNkSqJ94t8Xanigxh3sSytmi0dRHGgYeDc0PsimnrbZRnw8Aj7kLusoxCDBg0iJBDkaziQNZJzWefKTZdEFH0Qk+T9pVy2pYCInQELNn6kPGv8v2nsoGkubd+alldfkQUcWd6H3DVxLEitA23ENwT42MvxQUwpAyNJ38sNRAyOJ97cSzsOfJ1xRcfxhM2zVyYMoMs62scP+pC74J1LIr3NpGwpgghAgeD\/ZttBWv3bevrki+OxmkWymgFTeWVnGju4R6ymyTVEXkGUK+wnSbhPLkJ8LfBs3lsOZZoriACU\/\/umkf7+3\/bSHz47aqVR9PFgs+yhoZdTl3at8gKMaay9gMgmA6rt1X9JBNQHMUneX8pliwEi1iQf7P+anqr9hPY0HEukTUyaZXDvi2jpiCto3xentVQhHh9yZ\/TOpelAiGKMD2IKMQjN4R0+QMRaBADBuiSJuRW30L+8Uxt6vKq\/Mt0KBZS4MfIhd2FN1BxQkbAPSUEEAf7dvi\/ptR0N9NHn7hpE1ybwhn27y\/k04aYedO7Z32oWIMnFSi8gsjHnQsRCrqEp7PcMov9t1Z4+P0b0j+8dpF0Hj+atPZgK1hgAyfibetCV3duqr5qDJsmX015AVKxc3D6IyZfBxa4vTZ7VW+rpN1sPqi7la1qZNAgA8RfXXEJ9Op9XNgCxGT8fcheZvJE75MOpgLZ9EGPDxGKU4X2Q+v85Qf\/wzn76OI9FeVT\/pfb48+90omFXnr5LqBy0R5pj5kPujMkb5fFweWt3mkkxfBCTJrNt2gI4fn\/wa3rtgwbamcJiXH+nBAb+\/t7lF9FNfdsTzLlzT3wRkjfaDJJWxofc5fTOxcXOyQN8URpL3qdjupHNRzSEA28zG38QVqwx3tx5iD4+dDxVU8q05ujd+Tya9aPL6OjJb6w1R1LHggs\/yrWOVxCBafLOSnk8HADRb1iTUd987yfi7qqqqjL85yPn1dXVWacIfRATNfDQGJ0uOJfm\/vMf6P36r7wD40+6nk+VV11MCIL04doNIHKHuA+5y3Jx62eKbJI3RoFFAk1PiO+DGAjswo0f0ds5ghVth0CaU3\/a\/QL62Z91pbatzyn6OiOAyHYEs8v5kLu894m4UyZzTpp7IEdecsvf5ZO8EaCZ8srpCF\/bp1u707codG17DvW9+Dz6ux\/3psPHTlmbUrbv8VkuJG905+67776r8hymuYzIG0RMjp5rWyeTtRWbfAwil+SNv\/\/sJI1cW2\/kJIPk+kvb0F0DL6RLLyy\/q0dC8kZ3ECHPIXIhegERZ0Dt2bOnU1osgAJrKtPt4kyyvMGaQZQkeSM0z\/Bn\/pPqj5zWHHg4YHHJT7MPYLmzunnXDMkb3ccHad9wxMcLiNAteckXd1OaYLLrutfOdMU7gFJbW6ucEgxS\/I1j50lt03\/d2UB3PvO7JuCRx37d2Vp6NcOayH3MksqdzZtymnNxGsbk4tbvM5IubtOayGZGWPvuAbr3hR0ZzfPqvVdTrwJF\/dowsdBlAojcOe4dRCZNhO7aCHpSspIQ0+Fv38wAqKVqH8nfAKKk0namfBK5s31Lk83WMWPGqNsGVq1alfodrXqHbIjBGgipX3ntEwB0mosBRLbinV3ORu6Stp7TnEvaoG35XMTomSvrZn2v6PsztrT5LhdA5M7hXHLn0nKzBhFroV9U9qGJt1zqQl9Z1gkgch\/WFgMi3YyDFgrPGQ4EELlLQ4sEUTDjsgUmgKiZgqg5Zfvhe2SwkRq0UACRO2QK5FiwOR7u42BelFqVptzGydfQdysuTJOHZdFW0ETuw+jVnPOVczuK3Chifvp0HdXu+kJVC6acmXsBRM0MRNBESGSPO1lramoIFxTrT6ESlQSHgp1wBBDZ8clUyqsmcu+WW00TMRJE2Fgd3Ocit8bLvFYAkfsAlz2Ixv76fXr5vf8OplwOGQkgaqYg4ijrYppzwStnJxwBRHZ8ajbmHKKw+dY8967ndjVKU2720Mvpb27tlebryqqtACL34SyKOefLa6cT8\/aHh9Utz3jCeiheSAKISgxENidWXUjSQTRs+XuZXAmHfvl9lyZbTJ0AIveh9qqJ4tZEhThPFNZD9oIRQGTPK72kVxC5dyu+pjzZKtNvSWK69RuQOTd0a\/8OtO6vr\/LVnbJoN4DIfRi9g2j37t2ZC4+RJw4ZfPK59BgdXrNmjUp8sm\/fviZtS2JOdeqfWQ9Nva1C3focnmgOBBC5S4dXEEUlYMyVCsuWHAB0\/vz5tGTJEsIlYkwMtNPJnoPo5+s+VE29NLo\/De7T3rbZFlku5J1zH3aveed8RnGzSWcy55B37u0LfkhbPz1OyBm34e4e7hxqITVD3jn3gfaadw7dgtZZtmxZJscCOxuQ4krPxZ2UjKjkjSBq0ltnZ3JWb6m5OmnTLa58yDvnPuTe886hazC7Ro0aRfX1pzOM2uTitiXJlLwRnr8fvXRMNfGDKzrSS+OutG2uxZYLayL3ofe6JnLvVnRNdNgmeSODKDgV7EYhgMiOT6ZSXkHkKzIhl4t74i+epHlbWyl6Q6SCnXAEENnxqeAgwgsh8BUVFU3uGHLvbnxNnhF+NmMpPfHB+QFECRgdQJSAWVpR75oIyRsLHcX91eAH6VSnb6ucciGfgp1wBBDZ8akomsi9a8lr8owQQJScdwFEyXnGNbxoomIfDz\/8k5WKPlzNuGFicG\/biEcAkQ2XzGW8gKixsbHRvUvuNUHMiLGT6cgPF6hGgmfOnpcBRPa80kt6B1HasXNxpIKYOyc\/TDDn8ATPnL1gBBDZ86qgIPIdO2ciRoIopMeyF4wAInteFRREPmPnorwkd\/58Ln11wwNBEyWUiQCihAwTxb2bcz5j50wzwk8WvU4new1WX4XTrPaCEUBkz6uCaiJ+mc\/YOUkQZoTKJ+vCHpGDPAQQOTDtj1W8ayL3riWvCWJ+\/KtP6JvzOoWN1oTsCyBKyLBCmnPuXUteEyDiwNOwR5SMfwFEyfilW0AjR45M9R7iot6UF6K33YQhgMiNb6hVduZcAJGbMAQQufGtrEEUNlqTCUUAUTJ+FdScY1Wnd9HX1SqsiQKIkglFAFEyfhUMRJxPAbkUkFPB5sG+0vTp01XRKKDJQ3ndunXL5G+QjoUAIhtunykTQJSMXwUFES77mjNnjkpplevRU2ABLMjLgBxzbdq0UdWjQonYNmVNFDZac3G76fcBRMn4VTAQ4UXQLHv27HHK7CMTNTKIoN2mTJlCM2bMICSD1IkBiMJhvOQCEUCUnGdcw6t3Lt\/7iUxHy\/U11vjx4zMA5YiFiy\/\/Dr077bvuXGmBNUPyRvdB95q80b1bdhrMlHcOmmhg99b0zPAu+by+xdUNyRvdh9x78kYW9A0bNtCwYcNU7uxZs2YZzTEmI0lyEz3vHEDU97yvacEPLnDnSguseerUKXrjjTeof\/\/+1KtXuAwtiQhs3bqVli9f7idigQEED9rw4cNp9erVNHPmTFq\/fj1t3ry5icNAAijuFr24vHNIcI9LvQ7UvUGtd65PwodQNnAgLw5cd911tGjRIurRI52U1ZmwH3meqKGhIQMigMvktTPtKbH2WrhwYcbLJ13cck0ELgBI+AlP4EAhOQDwpAUg9LtJ7By7qUePHk3r1q2jCRMm0KRJk9S+Ub65uAvJpPCuwIFCciArAFXXMGnm4i4kYeFdgQOF4kDRorgLRWB4T+CAbw4UBURS2wVNFz\/E0mMaFVqln0bW156+hajU2o+LpHGhpQmI5IBxY3AWyFAel5fIOjKKAZ\/L2\/Pybbsc68sokqjtBFO0SDnyIg2aZFBBWhd6Z0AkXdzsRODP0Pm0gIQBhzCsXLlSxdhNmzaNqqurrYNe02BkqbRh2qDmO3A5tAq05BOuVSq8SKOf4OfSpUvpjjvuoJqaGuUssw22jnu\/0cUtA1DTvnJFzproGEA0aNCggtxEkcZAFLIN3eyQExCPkW49yEj5Qva1lN7lcmLBCkQoZDILkkQk2DAygMiGS6fL2IBIb80ENPs3toyS3kAUF4DKrE3jcF4w5+wF1dacky3qqaDt39ZySnoDUaFYGBwLyTidy7EAoM2bN49wCzuOm6B8VJhWsjeXb2mvICqEd47NRqQtwpOWh6Rch1wPCmYHD8CCp6qqqsll1WFNlFsSvIHI5J1Dd0wnVnN3M5QIHGg5HCi4d67lsDZQ2lI4kBWAirNEq1atUvY174RjwzUEoLYUkQh0JuVAVtiPzOCDxso9LMd2ozJNV79c59iE6KQdppJUSFBe9jltmeAwsLSjY1zodKlTlNg5l476qlMMECXdwG5OIPIVXQKrhw+CymgMX+OeZrstBkQySJM9WDh8yF5C1ghSE3O5bdu2ZfLr8SwsDxtGzcy6txOeyH79+tGYMWNo+\/btFOVJk22jDvbnENkxYMAAFS6F1GTynaY+49AZ6hw+fJg2bdqkvKB4JL0yxVkuekwbvwhBatu2rWo7Lu9gRUWF8iLGTQYBRGnC2kNbJgGora1V6zypifTdfpkTQppzci8GJ3NHjRpFixcvzorDkp5NgAbxWlhvduzYMTKVmOwP5\/Z75JFHCKeF8cDFLdvCRMCxiAgF4j5PnjxZgQhABZ3Srcug5PZkCoAoekw8BCDlpIL29LWz5FsAkQfhLlSTcYMXZ86ZNjorKyuz4v1M6yX9nTL6YMiQIUYQRfVTj1yIy+fHfWYQcVyiHtLF\/8+ePZvmzp3bJH7Rhh59wtGTefLYBhAVSsoL8B4Z1iRNDwkU02Yzm3ksDAwieDHlozsITBt63EYUiKI2AXVwSRCx2Sb7g74wiHgNo0cy6CDKRU+UOcebvwFEBRDi5vQKOSvDlOGsr7qgxWmiXAtsH5qI3ylBhPWaDPPRNRHXyaWJktKja6KowFepieIiBcKaqDkhxNAX3fyRqbwkUCSIAAI4ADhJS9SaiMuNGDEi6ziHy5pIrsM4mBTm1nPPPZc5dxUFItlnXRPZromi6IlaE3HYluy3HAKdB1G31AUQNXMQoXvySLo05\/hzmEDjxo3LeM5QBv9v3LhRLeahsXADhsk7F7XXY\/LOAZRxaxpTHXYEmDQRnBTs7ZN95rWO1DBMK5wNqFNXV5c5bBmX2kzuE0nNtmLFCjXycNLIPR6T9oEzBAD68ssv1WQAL6XM0x5AVAIgCl1syoEozRHFp1xronz5G0CULwdDfe8c0JOZJI0O0CMWkL7YdFTdhZAQseDCtVAncKCMOPD\/541bbCsgOrUAAAAASUVORK5CYII=","height":126,"width":209}}
%---
