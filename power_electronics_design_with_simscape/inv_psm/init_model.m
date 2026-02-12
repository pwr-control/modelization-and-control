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

torque_overload_factor = 1; % wind torque overload
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
% infineon_FF1200R17IP5;
danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.inv = device_igbt_setting_inv(fPWM_INV);

% infineon_FF650R17IE4;
% infineon_FF1200R17IP5;
danfoss_DP650B1700T104001;
% infineon_FF1200XTR17T2P5;
igbt.afe = device_igbt_setting_afe(fPWM_AFE);

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
figure;  %[output:472cd3f3]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:472cd3f3]
xlabel('state of charge [p.u.]'); %[output:472cd3f3]
ylabel('open circuit voltage [V]'); %[output:472cd3f3]
title('open circuit voltage(state of charge)'); %[output:472cd3f3]
grid on %[output:472cd3f3]
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
%   data: {"layout":"onright","rightPanelPercent":33}
%---
%[output:0db165d8]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:255f588d]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:57086c1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepPOSxi","value":"   0.500000000000000"}}
%---
%[output:08bc4522]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGxi","value":"     0"}}
%---
%[output:858ca950]
%   data: {"dataType":"textualVariable","outputData":{"name":"deepNEGeta","value":"   0.500000000000000"}}
%---
%[output:55bd5b41]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  44.782392633890389"}}
%---
%[output:079a03a1]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.183872841045359"}}
%---
%[output:72559c6d]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.183872841045359"],["44.782392633890389"]]}}
%---
%[output:1f057a07]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:62267490]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:5c6ec4ae]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000125000000000"],["-12.337005501361698","0.998036504591506"]]}}
%---
%[output:5060bba0]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.194386045440868"],["33.957607642498068"]]}}
%---
%[output:8a628839]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.036997274900261"}}
%---
%[output:8eb13f99]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.533540968663871"}}
%---
%[output:6936d375]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.414020903616658"}}
%---
%[output:978c2b65]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.438383113714302e+02"}}
%---
%[output:6c39f3fc]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.994503273143434e+02"}}
%---
%[output:472cd3f3]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAcwAAAEVCAYAAABt6tVhAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnX+QVtWZ549GJ6BioP2x2hLTGJtIYqVNXN0OyQxmGIOzu42z\/rEtnd2wyFqESKidhaL5lSJUKdCUzG6pySxrdbqY3dCwzJhIT02WcZmR1SAxAWVNgiURWtJpjQpRlCFlZYet5+p59\/Tt+77vfd\/3nHvvOe\/nVk1lbO59znk+z7nn+z7nnh\/nnTt37pziggAEIAABCECgIoHzEExaCAQgAAEIQKA6AQSzOiPugAAEIAABCCgEk0YAAQhAAAIQSEEAwUwBiVsgAAEIQAACCCZtAAIQgAAEIJCCAIKZAhK3QAACEIAABBBM2gAEIAABCEAgBQEEMwUkboEABCAAAQggmLQBCEAAAhCAQAoCCGYKSNwCAQhAAAIQQDBpAxCAAAQgAIEUBBDMFJC4BQIQgAAEIIBg0gYgAAEIQAACKQggmCkgcQsEIAABCEAAwaQNQAACEIAABFIQQDBTQOIWCEAAAhCAAIJJG4AABCAAAQikIIBgpoDELRCAAAQgAAEEkzYAAQhAAAIQSEEAwUwBiVuKTeDAgQOqp6dHbd++XXV2dlqtbNz22bNn1UMPPaTuvfde1dLSYqUssbly5crI1qZNm9TEiROVlPvKK6+o7u5uK2WUM7Jz5061f\/\/+UrmVCnvkkUfUnDlzVHt7e9U61XJvVWMf3KA5DQ0NqdbWVjUwMJCqLkl805Zp875Tp06phQsXqt7eXuvt1GY9sVWeAIJJ64BADQT6+voiMevv73cmmCMjI2rBggXq61\/\/ulPB1B343XffXbUcEdaHH344lUjVcm8N6NXRo0cjLl1dXZHopL2KIphSX2k\/o6OjqX6gpPWP+7IjgGBmx9rrknRnJS+7XDqb053R6dOno7\/v27dv3K9\/naXJv3d0dJTERv\/9G9\/4RvQ3sb1x48aynXe5OphZoNiXbE3XR56RTGTq1KnR3yU7kWvRokVRp6ttanGKZ5Tmf0vGt2rVquj5eIaT1CnHxbUaQ7G7YsUKtWTJEnX48OEx9RQRKle2lLN169aoTrfddpt68sknS8JmZmVi0OQbFzYtoLpsfa8ZPx376dOnR9lSvJ5J90rWb9ZfBE9n0vGXolwd4n9PshH3Vcc4qY2a7VALmTAs10bFlvy7tlmJebyu5siHy9EQrzsYTyqPYHoSqDyrGRcVs6PVQnTw4MGok77sssuijvTaa6+NOkUzW5o7d+6YoUfpbGUo1eyEymVv8WzIFKOXXnqpNCSrBVPXRw8fmr\/sdbnSkUl9zWyukmBKx18pwxQuO3bsiMRfLuEgzyQJcxJDeSbOTIZkhf+GDRvUli1bSnY1X+2LiJvma\/pejpP2xcx2zHufeOKJMRllXFzl3ra2tujHjRZDLQzxe02mWmg1F7Nd6xjrf4vHolqGWS7G8TYhZcZjPjg4OIa9zmJ1HXQblWf135KY6\/dBx3L37t1jONaS1ef5zlN2MgEEk5ZRlUC5TEQ6tqVLl477\/mbef+jQoXEdrxYVLXQ6k6k0lCed1PLlyxOHBJMyTN1hyffASp1ULRlmNcHUth588MGIqfldtRaG5YZkk7I0+Z4qWbP+nmeWo3+8aAEyOcR\/vAgnnTXFsy\/xJSk25TKpJHE1fwiVG5ZM+l5sftvVXJKGZCvFWGeYJ06cSPwxo18A7b\/8txbIpIxQ7ivHPC7GZpuQOMR\/FFR9+bihUAQQzEKFo5iViWdV5ktfTTAfe+yxaCjLvPRw5smTJyuKivlMNTHV4qSzCVMw46Jo2rUpmLpjFv90JqK\/ddbCMC6YutMWoVi3bp1av359ZF+yURFMU4xMTrrz1sPo2m8RA5m0ZI4EiGDGh4xN4UzKiEU0JKus9OMgPhSu65BGlOPD3JUEs1KM43bkv83sX\/8QMbmUy3Kl\/vFYmmx0m46\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\/wDbNBjkom\/Mj\/udiMIKlufX190Z97e3vH\/DOC2WgkeT5rAghm1sQpr14CCGa95GLPpf1Gmfa+StU6cOCA6unpUYsWLUIwLcUPM\/kRQDDzY0\/JtRFgSLY2XmXvzmprPC24slTlzJkzCKal+GEmPwIIZn7sKbk2An17hlXfnuMMydaGLb+7ZSh21qxZ6pVXXlHDw8MIZn6hoGRLBBBMSyAx45yAfL+UYVm2xnOOuvECZCh23759kUjKGk8Es3GmWMifAIKZfwyoQToCc7\/1nHr65bcQzHS48r1LssutW7eOqUT8O6ZM+tm7d2++FaV0CNRAYGRkRE2dOrWGJ7gVAvkQuPnh4ahgMsx8+NddKhlm3eh4sGAEyDALFhCqk0hA7yMr\/8jGBZ41EgTTs4BR3bIEEEwahw8E9JISqeslT29WJ579gQ\/VVl7s9JMXSdZh5kWecuslgGDWS47nsiSgv19e2zJBnf7OlzkP0xZ889SSrq4utWLFCrV27Vq1evVq5yeXIJi2ooidrAggmFmRppx6Cej1l\/L8H824TP1k458gmPXCNJ\/TYil7x951111q27Ztas2aNWr37t1q\/\/79atOmTWrixIk2ikq0gWA6Q4thRwQQTEdgMWuFgHy7XDJ4JJodK9fur31GfeVLn0UwbdA1d\/I5efJkSTBFSNevX6\/WrVvndPs8BNNGFLGRJQEEM0valFUrAb1ZgTzXc+vV6pG7b1A+9bOF\/4Ypyz9GR0fVPffco3bt2qUWL16slixZojo7O8dtNFBr8Krd71Mgq\/nCvzcHAQSzOeLso5emWMq3S8ku5X996mcLL5jSMPR+r7qRZHF4tJTlUyB9fIGos30CCKZ9plhsjIAMwz5++HW1bujlyJCI5CN3z1BfuH5y9N8+9bNeCGZj4ar\/aZ8CWb+XPBkSAQQzpGj674uI5dxvP6fkf7VY6sxSe+dTP4tgVmiTPgXS\/1cLD2wQQDBtUMRGowREIGVjdVlvqS9zGNa071M\/W2jBrHZqicyeHRgYcLa8xKdANtrAeT4MAghmGHH02Qu9qbrpw\/13Xq++NuujiW751M8WWjCFbtIuPPpvctLI4OCgs+UlPgXS5xeMutsjgGDaY4mldAT0cKu5XKRaVkmGmY5tTXeVOyBa\/33p0qXqoYcecra8BMGsKVzcXAACCGYBgtAkVZANCDbvOV5aU2m6HZ\/YUwmJT\/1soTNMvXHBwYMHS0OvR48eVQsWLFA333yzuvPOO9Xjjz9OhtkkLyhuVieAYFZnxB31EZBM8tCJ0+o7P\/xVWZH8FzdeoR74k+trKgDBrAlX9Zvjy0q2b9+upk+frpYtW+Z0izyfAlmdInc0AwEEsxminI2PIpBvvvue+ubQy4kCKbWQTPLzH5+s5t1ydWmZSK2186mfLXSGWSt42\/f7FEjbvmPPTwIIpp9xy7vWIo7yfzt+8po6cfJsWYHUIvmHN7So\/\/CHH4sEs9HLp34WwawQbZ8C2Wij5fkwCCCYYcTRtRcijn++75fqZ6PvVhRHLZDXTpmgVsyZFgmkDZE0\/fOpny28YMqM2FWrVo1rPx0dHaq\/v5+9ZF2\/Wdj3igCC6VW4nFdWJua8+NoZtfvw61WFUVdGBPGL01vUn\/6RnQyympMIZjVCKf9dZsPq75SPPfaYkmUksoes7C\/b1tamuru7U1qq7zafAlmfhzwVGgEEM7SIVvdHL+v47rOvqmdefiu1MOrsUb5B9s6ZFhVkO3usXnu2xkvDKNU95rKSJ554Qg0PD0cbrpdbbpLKaA03IZg1wOLWQhBAMAsRBieVEGH80fDb6r89M6pO\/Ob9b45pLy2E8z\/Xqm752EecDK2mrUv8Pp\/62UIPyeplJTNnzlSf\/exno8OjN2\/erA4dOqR27NjBkGy9LZTngiWAYPobWi2A\/3nvK+oXr\/9DzaKoM0T53jjj6kvUfbd9NJeMsdYIIJi1EqtwfzzL1N8zZWmJDM+6vHwKpEsO2PaHAIJZ3Fjpmaj\/82dvqv8z8k5dgmgOm8pQ6uJZH1WXTrggt+FUG7R96mcLnWHaCEYjNnwKZCN+8mw4BBDMfGIpk2vkOv98pb77o1fVL2WZRo3DpmbN9RCqiKJMvnnt7fcKNYxqk7JP\/WyhBbPa1njr1q1jlqzNlost7wkgmHZDqLPCSAzPU0om1jQqhmaWKMOnXR1XqBlXXVLKEvOYeGOXWm3WEMzaeI27u9opJfJAV1eXsy3xdIV8CmSDyHk8EAIIZvpAmmL41C9+E80wlauRzFCXHq1XnDJBfbRlgrqz40p1w1UXR5N09KHJ6WsZ\/p0+9bNeZphZNSGfApkVE8opNgEE8\/346CHSgydOq71HTloTQjM7lOHS32+fomZeNxkxbOC18KmfLbRgNhADK4\/6FEgrDmPEewIhC6aeRfrSr8+ov37hTXXsjX9wIoQ6M\/yXn75CferqSxBDx2+FT\/1sIQUzzZAsO\/04bsWY95KAb4KpRfCcUupvfvqG+sELb1oVwaQh0vYrL1L\/9IO1iGbG6GXAA6g0ghlAEMUFnwIZCHLcaJBAEQRTi+A\/njundh38tXrq6G+ciWAkeB98K5zzyctVy8UXRmXpiTPNNoGmweaTy+M+9bOFzDBziVpCoT4FsijMqEe+BGwLpjkxRsTnhV+9q37wszfUiZPv7zJjY4JMnFhJ7KZMUDd\/7FL172Zeo86Tsk79FiHMt3k5Kd2nfrbwgql3+xkaGioFK4sZsmSYTt4NjDomkFYwtRCePPOe2vviKTX85tlMBFAKufGaS9TCz09VF35IZPD9bNAUQ8eIMF8wAgimpYBosWxtbY32kNWXbL4+OjrKshJLnDHjNwFzT9G\/e\/5ldfT0h9ULv3rHmQCOGfKcMkF9\/IqLVOd1H1HXTH7\/bEQXR0D5HSFqX4kAgmmpfbBxgSWQmPGGgM789LDk\/zpyUv109N1ob1G5XAyBxgVQ\/ntG6yXq9hta1IQLP8QwqDetx8+KIpgW4ybZpAzHDgwMqPb2dnX06FG1YMGCaOMCM+u0WGTJlE+BdOE\/NhsnYGZ\/ctLE\/37pN+qVk+6GP3WNze+A8rdPXXOJ+vKtV5f2HY3f17inWIBAfQR86mcL\/w1TQhA\/RHrjxo3Oz8KUcn0KZH1NlafSEogfpXT2vf+r\/vK5X6sfHXvbaeaXJIBtl09UM666WN1w9cXqwvPPHzME+sPDR9XnO9rTusV9EMidgE\/9rBeCmVdEfQpkXox8LVcLoPzvm2feizI\/18OeSVmdLImQbFAWyV\/y4bGnTtSzJCLtpB9f40a9wyPgUz+LYFZofz4FMrzXKL1HZvYnW6EdOPa2OvLqu5lkflKIOfz5scsmquuvnKj+1U3\/JNGBekQwPQmlEMxaaHFvEQj41M8WWjD1jj\/XXnut8xmxSQ3Hp0AWoeE3Wgdzzd81kz+s\/sfB16LlBnI6hFyuJryY9TbFTzbOnnb5RNU5bXJp6UPRN89GMBtthTyfNQGf+tlCC6YELmkd5qJFi5xP+JGyfQpk1o28Wnlm1iezPCXre\/6XpzMXvigDnDJBSeZ32\/Qp6pa2j7xfh0AXwSOY1Vom\/140Aj71s4UXzKTgHjhwQMns2f7+\/sTzMOXfe3p6okfLbXIQF+KkvWl9CqSrlyC+08vwybPqLw\/92ulC97gv8Rmf1195kbrjU5eri37vQ9GtrPv7\/8QQTFdvAnZdEfCpny28YCZlmLKRgV5mEg+iDOMuW7ZMrV69Wk2dOlWtXLlSzZw5c9ysWvM+Wa6SdPkUyHoasxbDv3\/plPrxcfezPc3vd3r\/z09fM0n98Y2XB5311RObep9BMOslx3N5EfCpny20YOpvmBLIctlkpSBrsZ03b57q7Owcc6us59ywYYPasmVLYpYqN\/sUyDgHPST6vedfL50H+PQHB+TaeDHi4nfjNZPU7TNa1IUfOn9c1se2ZzaIp7OBYKbjxF3FIeBTP1towWwkpHpYttyQbJq1nb4EUg7L3fmT16IF8Y2Iojn0+cnWS9Q9n79GvX76vZIA6uHPRuLCs24JIJhu+WLdPgFf+lnxPFjB1GEVYdy\/f3\/FWbY6k5Wdg8xMtIiBlGzt7bO\/U2u+fzS1OJpC2HndZPVv\/tnVwU56sf86+2URwfQrXtTWr5G84AWz2gQhabB66Db+rVMEc+\/evbm36dHTv1OPPvuW2n3k\/bWF5a7WSy9QrZMuUPNv\/oj68AXnqasnXaDkb1zNQ2BkZCT6ds8FgaITmD17dqmKx44dK3p1o\/oFJ5jxDdtlNq1c8X1nJfOUq7u7O9qfdsWKFWrz5s3RfrX6yjPDlExSzh78twMvJDakaGbolAnqkXkzSgvnvWhxVNIpATJMp3gx7oBAnv1sre4UWjDrPa2k3LISUyTjs2+T9qfNK5BLdryotj\/76rhYikju\/tpnEMhaW3kT3Y9gNlGwA3E1r362HnyFFEz9TfHw4cNlfcriEOksAykZpaxvvP9vxg5NiEg+cvcMVfQdZuppfDxjnwCCaZ8pFt0SyLKfbdSTQgqmdqpchtmo02mfzyqQgz9+Td03eGRMtRDKtFHiPpMAgkl78I1AVv2sDS6FFkwbDjZiw3UgJasUsezbc7xUTYSykYjxLIJJG\/CNgOt+1iaPQgqmziyXLl2qli9frpKGZpO2srMJRmy5DKSI5dxvPxct75ALobQdvea0h2A2Z9x99tplP2ubSyEF07aT9dpzFcgksXx+7efqrSbPQaBEAMGkMfhGwFU\/64IDglmBqotAiljedP8zpVL\/oH2K+v7im1zEFptNSADBbMKge+6yi37WFZJCC2al2bI+DsnGM0uZ\/dpz61WuYovdJiSAYDZh0D13GcF0HEDZjGDWrFnjNlS3XazNQMbF8j\/960+o+Z2ttquMvSYngGA2eQPw0H2b\/axr9wudYZZzPqvlJrYCKWK5ZMcRJZukyzXvlqvUt+bNcB1b7DchAQSzCYPuucu2+tksMHgpmGn2h7UBz1YgzXWWMhuWCT42ooONJAIIJu3CNwK2+tks\/C60YFb6hrl9+3YvhmTNST5sbZdFk27uMhDM5o6\/j94jmD5GLaHOjQYyGoodPFI6hkv2gWWLu0AaR0HdQDALGhiqVZZAo\/1slmgLnWEKiPhJIrKB+o4dO1R\/f79qaWlxyqrRQJpDsXy3dBoqjH9AAMGkKfhGoNF+Nkt\/Cy2Y+kSRefPmjRl+TXMotA2IjQSSoVgbEcBGrQQQzFqJcX\/eBBrpZ7Oue6EFs97jvWxBbCSQq753VG19aiSqCkOxtiKCnWoEEMxqhPj3ohFopJ\/N2pdCC6bAkGzy4YcfVgMDA9HhznoiUGdn57hDoW3DqzeQ8eySWbG2I4O9cgQQTNqGbwTq7Wfz8LPwgilQ5DvmggUL1OjoaMQo6bBnF\/DqDeTK7x1V\/5Xs0kVIsFmFAIJJE\/GNQL39bB5+eiGYeYCRMusJJNllXtGiXCGAYNIOfCNQTz+bl4+FFsysdvQpB7+eQG74wXH14BPDkUm+XebVrJu3XASzeWPvq+f19LN5+VpowRQosm9sW1ub6u7uzpxRPYFs+Y9\/H9WTHX0yDxcFkmHSBjwkUE8\/m5ebhRZM304rkb1i5VBouXrnTFO9c9ryiivlNikBMswmDbzHbiOYHgfPrHqtgZz7reeiXX3YAi+QBuChGwimh0Fr8irX2s\/miavQGWaeYKTsWgJpTvb5wscnq933fSbv6lN+ExJAMJsw6J67XEs\/m7erhRRMPdln6dKlavny5erw4cPjOBXtAGlzso+su5QskwsCWRNAMLMmTnmNEkAwGyVYkOfTBpKlJAUJGNVgWQltwDsCafvZIjhWyAzTBOPD5uumYK74Uptaece0IsSWOjQhATLMJgy65y4jmJYC6Mvm60t3vqj++49ejbxmONZS8DFTFwEEsy5sPJQjAQTTEnwfNl9nONZSsDFjhQCCaQUjRjIkgGBahF30zddNwXz47hvUl2+92qL3mIJAbQQQzNp4cXf+BBBMyzEo8ubrfXuGVd+e4wzHWo455uojgGDWx42n8iOAYObH3mrJaQJpblbAMV5W8WOsDgIIZh3QeCRXAmn62VwraBRe+FmyeYKqFkhzOPaLn2hRf7WoI8\/qUjYEWFZCG\/COQLV+tkgOIZgVolEtkIM\/fk3dN3gkssDJJEVq1s1bFzLM5o29r55X62eL5BeC2YBgMhxbpKZMXYQAgkk78I0AgulbxMrUt1og9VFe7B0bSMADcAPBDCCITeZCtX62SDgKn2EeOHBA9fT0jGOW916y5lFej3\/tJvX7108pUlypS5MSQDCbNPAeu41gWgqePg+zt7dXdXZ2WrKa3kylQPL9Mj1H7syOAIKZHWtKskMAwbTDUZXb6ceGeb3t3tDQUGRu+\/bt40S5UiD5fmkjCtiwTQDBtE0Ue64JIJgWCctOP8PDw0qyTJuXaVeGffv6+lR\/f79qaWkpFVMukJx9aTMS2LJJAMG0SRNbWRBAMC1R1kOyrs\/DFMEcHBxUmzZtUhMnTqwqmOb3y+8vvkn9QTvfLy2FHDMNEkAwGwTI45kTQDAzR15fgeawbK1DsnqGLOsv62PPU24IIJhuuGLVHQEE0x1bJ5bLTS4qF0j9\/VIqc+rPvuikThiFQD0EEMx6qPFMngQQTIv0zSywq6tLrVixQq1du1atXr1atbe3WylJlzFz5kzV3d09Zkh2796948ro2jaiRk\/\/TrVeeoEamj\/VSh0wAgEbBEZGRtTUqbRJGyyx4ZbA7NmzSwUcO3Ys6Hg\/AAAOLElEQVTMbWGWrBd6HaYWstbWVnXXXXepbdu2qTVr1qjdu3er\/fv3j\/vmWAsTmfQjlwiknIYiQrx58+YxIlzulw8bFtRCmnuzJECGmSVtyrJBgAzTBkUZ7jx1Sq1fv16tW7dOnTx5siSYIqT67+as1lqKrXdZiTnhp3fONNU7p62WYrkXAk4JIJhO8WLcAQEE0yJUWe4xOjqq7rnnHrVr1y61ePFitWTJkmjNpO2lJvFqJwVy+7OvqiU7XoxuZcKPxUBjygoBBNMKRoxkSADBtAw7vj3exo0bx3xrtFxcyVxSINmwwBVt7NoggGDaoIiNLAkgmFnSdlhWUiBvuv8ZJRsXsOG6Q\/CYrpsAglk3Oh7MiQCCmRN428UmBVJP+OH7pW3a2LNBAMG0QREbWRJAMC3Sjk\/OEdOyvCS+K4\/FIssOyZoTfvh+6YI4NhslgGA2SpDnsyaAYFoibi4r0RN89N+kCNeiGQ8kJ5RYCixmnBFAMJ2hxbAjAgimJbDlTitxeYqJWfV4INnhx1JgMeOMAILpDC2GHRFAMC2CTdoYXZaatLW1OZ8pW04wr22ZoJ5f+zmLXmIKAnYIIJh2OGIlOwIIpiXWlU4r0UV0dHSMO5bLUvHKDKR5pNeXPnmZ2vHvP22rGOxAwBoBBNMaSgxlRADBzAi062LKCSYzZF2Tx369BBDMesnxXF4EEMy8yFsu1wwkM2Qtw8WcEwIIphOsGHVIAMG0CLcoy0r++oU31FcGfhp5xpISiwHGlFUCCKZVnBjLgACCaQly0rISMa33l81yWQlb4lkKKmacEkAwneLFuAMCCKYlqEVaVqK3xGOGrKXgYsYJAQTTCVaMOiSAYFqEK9nk0NCQGhgYiM6qlLMrFyxYEO32k9VpJeYM2VnTp6jvffUmix5iCgL2CCCY9lhiKRsCCKZlznLY86pVq0pWsz6thDMwLQcUc84IIJjO0GLYEQEE0xHYrM3qQJqCKRsWyLAsFwSKSADBLGJUqFMlAghmIO1DB7Jvz7Dq23M88ooZsoEEN1A3EMxAAxuwWwhmIMHVgWQP2UAC2gRuIJhNEOTAXEQwAwloXDCZIRtIYAN2A8EMOLiBuoZgBhJYHUh9aPQXPj5Z7b7vM4F4hxshEkAwQ4xq2D4hmIHEVwL55E9+rmQNplzzbrlKfWvejEC8w40QCSCYIUY1bJ8QzEDiK4H8i789pOZ++7nIIzZdDySwAbuBYAYc3EBdQzADCawE8oGd+9V9g0cij1hSEkhgA3YDwQw4uIG6hmAGElgJ5KI\/\/zuWlAQSz2ZwA8FshiiH5SOCGUg8JZA3Lvsr9fTLb0UenfqzLwbiGW6ESgDBDDWy4fqFYAYSW1MwWVISSFADdwPBDDzAAbqHYAYSVAnkpfd8V8nm6ywpCSSogbuBYAYe4ADdQzADCWrbjbeq01\/qi7xBMAMJauBuIJiBBzhA9xDMQIJqCiZLSgIJauBuIJiBBzhA9xDMQIJqCqZsWCAbF3BBoMgEEMwiR4e6JRFAMANpF63\/\/E\/Vb2+YG3nDKSWBBDVwNxDMwAMcoHsIZiBBNQWTTQsCCWrgbiCYgQc4QPcQzECCeuVX\/ov63eWfiLxhDWYgQQ3cDQQz8AAH6B6CGUhQtWCyBjOQgDaBGwhmEwQ5MBcRzEACevlXd6l\/vOhylpQEEs9mcAPBbIYoh+UjghlIPPU5mLdNb1GPfbUjEK9wI2QCCGbI0Q3TNwQz57ju3LlTrVq1KqpFR0eH6u\/vVy0tLWNqdfbsWbVy5Uo1NDRU9j4tmKzBzDmgFJ+aAIKZGhU3FoQAgpljII4ePao2bNigtmzZEolkX1+fGh0dVZs2bVITJ04s1ezUqVNq2bJlavXq1aq9vT2xxlowWYOZY0ApuiYCCGZNuLi5AAQQzAIEQVfhwIEDanBwcJxgxoU1qcpaMFmDWaCAUpWKBBBMGohvBBDMAkVMMsy2tjbV3d09plbmsK38w8aNG8fdg2AWKJBUJRUBBDMVJm4qEAEEsyDBEFEcHh5Wvb29FWskw7MLFy6M7uvs7CzdqwXz4NfbCuIR1YBAZQIjIyNq6tSpYIJA4QnMnj27VMdjx44Vvr5SwfPOnTt3zoua1ljJcpllkhk9AWjmzJljskwRTNZg1gie23MlQIaZK34Kr4MAGWYd0Gw+ImI5a9asMdli3L5kn3LJUK18z1yxYoXavHnzmAlACKbNqGArCwIIZhaUKcMmAQTTJs0abckkn56enjFPdXV1RZN+du\/eXRLJ+LKSct8wOQezxgBwe64EEMxc8VN4HQQQzDqgFfERyTB\/78QP1UWHvlPE6lEnCEAAAkEQ4BtmEGHECQhAAAIQgMD7BIKd9EOAIQABCEAAAjYJIJg2aWILAhCAAASCJYBgBhtaHIMABCAAAZsEEMwEmuYuQNu3b6+4PMVmMLAFgTQEzJngSbO7xYYslVqwYEG0j7JcixYtqrqBR5qyuQcCNgmk2dPbZnmN2kIwYwTNPWZfeumlxH1oG4XO8xCol4DZwYgN86AB02a5PZTrLZfnIGCbgP5RJ3YHBgbKHoJhu9xG7CGYMXqSXe7fvz9atylrNaudaNIIfJ6FQK0ERAhlYw45sk5O35Ej6ubNmzduFCTttpC1ls\/9ELBBQH74Pfroo+qOO+5Q3\/zmN8dtGmOjDBc2EMwEwdT7z5bbY9ZFILAJgTQEzMxR7hfBjG\/pGN+Uo7W11Ztf8GkYcE84BMrtslZUDxFMBLOobZN6JRBII5jxx8ysNH6QOpAhkCcBBDNP+hbKZkjWAkRMOCOQdkjWrIBvnZIzeBguHAHf2iYZZqwJMemncO8UFTIIpJn0I0OyDzzwgJo\/f340kcL8ESjfPbkgUBQCCGZRItFAPfSyEr79NACRR50RMJeVmMue4ifw6GUltGNnocBwgwQQzAYB8jgEIAABCECgiAQYki1iVKgTBCAAAQgUjgCCWbiQUCEIQAACECgiAQSziFGhThCAAAQgUDgCCGbhQkKFIAABCECgiAQQzCJGhTplTiDtVnLmsiNbmwCYG6Wn3ey\/SHvF6lm7HR0d0ZZ9trjoRqDtd3V1RVtWsjQm89eDAj8ggGDSFCCgVLRWUW+JWAmIC8EUQdi3b19Np4kUTTAHBwediplw37Ztm1qzZg2CyRubGwEEMzf0FJwHAXMNo86I5FSanp6eqDr6GCzzPvm7ZH7Tp09XCxcuVIcPH1b6Wb0B+tDQUPR8pQzRtKmzJbGlyy6XQZnHzenjvLRgTpo0KSoznt2Zz5jrMGXj9p\/\/\/OfqqaeeUtqWue74tttuU2Kzt7c38iepzvEMLy7eUsbFF1+s9u7dG7Eqd7RY\/MdHpR8BCGYebwtlxgkgmLSJpiEQP3vPXOhvZpjxxdTmTjkjIyNjjtQScZBLBEY6\/OXLlydudK6HXR988MFI3GTTdBEy\/Vy5DM0UEfP0nJMnT0ZCq8Uybk+faCLDo2Yd5f+XMzL10Kbp62WXXRb9IOjs7IzqZf7b1KlTx9TZbDRJgik\/IOTIJm1T7Ild80Iwm+bVC8ZRBDOYUOJINQKVDqutNCRrCoIpmFKeCIwWA31KSNJxW3FRMfeErXTuqghcW1ub6u7uHuNefEP1SvU3\/80UTzEYf8787\/iWeuUywCTB1D8iksrQjiCY1Vos\/140Aghm0SJCfZwSMCfYmEOgceEQYdm6dWupLvreJMGUYUfz0kOd5t\/i4pNmz2ItwPHju\/RQqZmVmvWPH+8l9+th0bgAVxJQc1hX+5I0sSdJME2RLyfmCKbTpo5xBwQQTAdQMekHgXg2pSf9xLO3Shlm2gPGXWSY5QQzLs7xDLOSmJVjUimi1TLMuCiXyzArbRLPN0w\/3qnQa4lghh5h\/CsRiGc05b5hJh2hJUbku1+lb5jmd8qk73WyGXqt3zDjx83pIWCpTxrBlGzT\/C4ZzzDTfsOUU0\/i3z812CTBlL\/JEhO5zGFrszkmfdfVnOMTixBMXuQiEEAwixAF6pAZAXOY0RyS1bNBZehy6dKl0QQXmbgiE3NWr16tdu3apbZs2VISAPl\/RBDis2SThmNNYUmaEVttiYg5PByfJasn75iZoXyrNWfz3nvvvWrPnj2R4D\/00EPjvolqJjLcOnv2bHXmzJnEWbLl1lkmCeY777yjnnzyyWiCkckk6ZvpqlWrIs5S5+effz7xhwmCmdkrQkEVCCCYNA8IQCAiUOmbaa1DskkTlRrBjGA2Qo9nbRFAMG2RxA4EPCQQX29abs1kNcGUzFlnoI8++mjizN568bDTT73keM42AQTTNlHsQQACEIBAkAQQzCDDilMQgAAEIGCbAIJpmyj2IAABCEAgSAIIZpBhxSkIQAACELBNAMG0TRR7EIAABCAQJAEEM8iw4hQEIAABCNgmgGDaJoo9CEAAAhAIkgCCGWRYcQoCEIAABGwTQDBtE8UeBCAAAQgESQDBDDKsOAUBCEAAArYJIJi2iWIPAhCAAASCJIBgBhlWnIIABCAAAdsEEEzbRLEHAQhAAAJBEkAwgwwrTkEAAhCAgG0CCKZtotiDAAQgAIEgCSCYQYYVpyAAAQhAwDYBBNM2UexBAAIQgECQBBDMIMOKUxCAAAQgYJsAgmmbKPYgAAEIQCBIAghmkGHFKQhAAAIQsE0AwbRNFHsQgAAEIBAkAQQzyLDiFAQgAAEI2CaAYNomij0IQAACEAiSAIIZZFhxCgIQgAAEbBNAMG0TxR4EIAABCARJAMEMMqw4BQEIQAACtgkgmLaJYg8CEIAABIIkgGAGGVacggAEIAAB2wQQTNtEsQcBCEAAAkESQDCDDCtOQQACEICAbQIIpm2i2IMABCAAgSAJ\/D\/0b4KxgmtXLQAAAABJRU5ErkJggg==","height":277,"width":460}}
%---
