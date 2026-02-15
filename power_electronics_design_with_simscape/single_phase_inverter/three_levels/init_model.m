preamble;
model = 'single_phase_inverter';
%[text] ### Global timing
% simulation length
simlength = 1.25;

% switching frequency and tasks timing
fPWM = 10e3;
fPWM_AFE = fPWM;
tPWM_AFE = 1/fPWM_AFE;
TRGO_AFE_double_update = 0;
if TRGO_AFE_double_update
    ts_afe = tPWM_AFE/2;
else
    ts_afe = tPWM_AFE;
end

fPWM_INV = fPWM;
tPWM_INV = 1/fPWM_INV;
TRGO_INV_double_update = 0;
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
dead_time_AFE = 1e-6;
dead_time_INV = 1e-6;
dead_time_DAB = 1e-6;

% minimum pulse
minimum_pulse_time_AFE = 1e-6;
minimum_pulse_time_INV = 1e-6;
minimum_pulse_time_DAB = 1e-6;

% delays
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayINV_modB=0;
delayDAB_modB=0;

% maximum simulation and simscape step
tc = min(ts_afe)/100;

% t_misura = simlength - 0.025;
t_misura = 0.648228176318064;

% storage data length and decimation over tc sampling 
decimation_tc = 1;
Nc = ceil(t_misura/tc)/decimation_tc;
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
% afe inv psm models
use_torque_curve = 1; % for wind application
use_speed_control = 1-use_torque_curve; %
use_mtpa = 1; %
use_psm_encoder = 0; % 
use_load_estimator = 0; %
use_estimator_from_mb = 0; %mb model based
use_motor_speed_control_mode = 0; 

% advanced dqPLL
use_dq_pll_fht_pll = 1; % 
use_dq_pll_ddsfr_pll = 0; % 
use_dq_pll_mod1 = 0; % 
use_dq_pll_ccaller_mod1 = 0; % 
use_dq_pll_ccaller_mod2 = 0; % 

% dqPLL
use_dq_pll_mode1 = use_dq_pll_mod1;
use_dq_pll_mode2 = use_dq_pll_ccaller_mod1;
use_dq_pll_mode3 = use_dq_pll_ddsfr_pll;
use_dq_pll_mode4 = use_dq_pll_fht_pll;

% single phase inverter
rpi_enable = 0; % use RPI otherwise DQ PI
system_identification_enable = 0;
use_current_controller_from_ccaller_mod1 = 1;
use_phase_shift_filter_from_ccaller_mod1 = 1;
use_sogi_from_ccaller_mod1 = 1;

% psm observers
use_ekf_bemf_module_1 = 1;
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
enable_frt_1 = 0; % faults generated from abc
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
l1 = Kd(2) %[output:5c73ba2d]
l2 = Kd(1) %[output:90780375]
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
%[text] ### Load Transformer Parameters
m1_load_trafo = 50;
m2_load_trafo = 1;
m12_load_trafo = m1_load_trafo/m2_load_trafo;

ftr_nom_load_trafo = 50;
I0rms_load_trafo = 5;
V1rms_load_trafo = 330;
I1rms_load_trafo = 600;
V2rms_load_trafo = V1rms_load_trafo/m12_load_trafo %[output:4f708157]
I2rms_load_trafo = I1rms_load_trafo*m12_load_trafo %[output:4c6e9b3c]
lm_load_trafo = V1rms_load_trafo/I0rms_load_trafo/2/pi/ftr_nom_load_trafo;
rfe_load_trafo = 2e3;
rd1_load_trafo = 1e-3;

if frequency_set < 160
    ld1_load_trafo = 400e-6; % for f <= 80Hz output
else
    ld1_load_trafo = 100e-6; % for f > 400Hz 
end
% ld1_load_trafo = 400e-6; % for f <= 160Hz output
% ld1_load_trafo = 100e-6; % for f >= 160Hz output
rd2_load_trafo = rd1_load_trafo/m12_load_trafo^2;
ld2_load_trafo = ld1_load_trafo/m12_load_trafo^2;
%%
%[text] ### DClink, and dclink-brake parameters
Vdc_ref = grid_emu_data.Vdclink_nom; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;

CFi_dc1 = 1400e-6*4;
CFi = CFi_dc1;
RCFi_dc1 = 1e-3;
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
%[text] ### Sensor endscale, and quantization
adc_quantization = 1/2^11;
Imax_adc = 1049.835;
CurrentQuantization = Imax_adc/2^11;

Umax_adc = 1500;
VoltageQuantization = Umax_adc/2^11;

Pnom = 200e3;
margin_factor = 1.25;
adc12_quantization = 1/2^12;
adc11_quantization = 1/2^11;
adc16_quantization = 1/2^16;
adc15_quantization = 1/2^15;
Vdc_FS = Vdc_nom * margin_factor;
Idc_FS = Pnom/Vdc_nom * margin_factor;
Vac_FS = V1rms_load_trafo*sqrt(2) %[output:02d04b5d]
Iac_FS = I1rms_load_trafo*sqrt(2) %[output:4e347a24]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:07dc0e95]

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
Afht0 = [0 1; -omega_fht0^2 -delta_fht0*omega_fht0] % impianto nel continuo %[output:3c13489f]
Cfht0 = [1 0];
poles_fht0 = [-1 -4]*omega_fht0;
Lfht0 = acker(Afht0',Cfht0', poles_fht0)' % guadagni osservatore nel continuo %[output:623149ec]
Ad_fht0 = eye(2) + Afht0*ts_afe % impianto nel discreto %[output:9f3a46ea]
polesd_fht0 = exp(ts_afe*poles_fht0);
Ld_fht0 = acker(Ad_fht0',Cfht0', polesd_fht0) %[output:9972cb02]

%[text] ### First Harmonic Tracker for Load
omega_fht1 = grid_emu_data.w_grid;
delta_fht1 = 0.05;
Afht1 = [0 1; -omega_fht1^2 -delta_fht1*omega_fht1] % impianto nel continuo %[output:03b4dbba]
Cfht1 = [1 0];
poles_fht1 = [-1 -4]*omega_fht1;
Lfht1 = acker(Afht1', Cfht1', poles_fht1)' % guadagni osservatore nel continuo %[output:45176e25]
Ad_fht1 = eye(2) + Afht1*ts_afe % impianto nel discreto %[output:278dad4b]
polesd_fht1 = exp(ts_afe*poles_fht1);
Ld_fht1 = acker(Ad_fht1',Cfht1', polesd_fht1) %[output:4efa0306]
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
run('n_sys_generic_1M5W_pmsm'); %[output:19ae2427] %[output:8267420f]
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
kg = Kobs(1) %[output:5a4ef0d7]
kw = Kobs(2) %[output:4e1ed4e5]
%[text] ### Rotor speed observer with load estimator
A = [0 1 0; 0 0 -1/Jm_norm; 0 0 0];
Alo = eye(3) + A*ts_inv;
Blo = [0; ts_inv/Jm_norm; 0];
Clo = [1 0 0];
p3place = exp([-1 -5 -25]*125*ts_inv);
Klo = (acker(Alo',Clo',p3place))';
luenberger_l1 = Klo(1) %[output:6dfbc3d1]
luenberger_l2 = Klo(2) %[output:5d960356]
luenberger_l3 = Klo(3) %[output:973a4095]
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

%[text] ### Sogi
sogi_delta = 0.05;
omega_set = 2*pi*frequency_set;
sogi = sogi_filter(omega_set, sogi_delta, ts_afe); %[output:5345c40a]

% gain for active sogi
kepsilon = 2;
%[text] #### Resonant PI
kp_rpi = 0.25;
ki_rpi = 45;
delta_rpi = 0.05;
res_nom = s/(s^2 + 2*delta_rpi*omega_set*s + (omega_set)^2);

Ares_nom = [0 1; -omega_set^2 -2*delta_rpi*omega_set] %[output:496c28ac]
Aresd_nom = eye(2) + Ares_nom*ts_inv %[output:04a211a4]
a11d = 1 %[output:42b17330]
a12d = ts_inv %[output:0ebfe05b]
a21d = -omega_set^2*ts_inv %[output:08f2e94a]
a22d = 1 -2*delta_rpi*omega_set*ts_inv %[output:13a8d493]

Bres = [0; 1];
Cres = [0 1];
Bresd = Bres*ts_inv;
Cresd = Cres;
%[text] ### Current control parameters DQ PI
kp_inv = 0.5;
ki_inv = 45;
pi_ctrl = kp_inv + ki_inv/s;
pid_ctrl = c2d(pi_ctrl, ts_inv);
plant = 1/(s*ld1_load_trafo + 0.25);
plantd = c2d(plant, ts_inv);

G = sogi.fltd.alpha * pid_ctrl * plantd;
figure; margin(G, options); grid on %[output:237b32fa]
%[text] ### Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] ### Diode rectifier
Vf_diode_rectifier = 0.35;
Rdon_diode_rectifier = 3.5e-3;
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
danfoss_MOSFET_SKM1700MB20R4S2I4;
mosfet.inv.data = 'danfoss_MOSFET_SKM1700MB20R4S2I4';
mosfet.inv = device_mosfet_setting_inv(fPWM_INV);

danfoss_MOSFET_SKM1700MB20R4S2I4;
mosfet.afe.data = 'danfoss_MOSFET_SKM1700MB20R4S2I4';
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
lithium_ion_battery = lithium_ion_battery(nominal_battery_voltage, nominal_battery_power, initial_battery_soc, ts_inv); %[output:795534a0]
%[text] ### 
%[text] ### Load
uload = 3;
rload = uload/I2rms_load_trafo;
lload = 1e-6/m12_load_trafo^2;

% rload = 0.86/m12_load_trafo^2;
% lload = 3e-3/m12_load_trafo^2;
%[text] ### C-Caller Settings
open_system(model);
Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
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



%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":36.3}
%---
%[output:5c73ba2d]
%   data: {"dataType":"textualVariable","outputData":{"name":"l1","value":"  36.521945502464568"}}
%---
%[output:90780375]
%   data: {"dataType":"textualVariable","outputData":{"name":"l2","value":"   0.149016195397013"}}
%---
%[output:4f708157]
%   data: {"dataType":"textualVariable","outputData":{"name":"V2rms_load_trafo","value":"   6.600000000000000"}}
%---
%[output:4c6e9b3c]
%   data: {"dataType":"textualVariable","outputData":{"name":"I2rms_load_trafo","value":"       30000"}}
%---
%[output:02d04b5d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     4.666904755831214e+02"}}
%---
%[output:4e347a24]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     8.485281374238571e+02"}}
%---
%[output:07dc0e95]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.149016195397013"],["36.521945502464568"]]}}
%---
%[output:3c13489f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht0","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:623149ec]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht0","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:9f3a46ea]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht0","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:9972cb02]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht0","rows":1,"type":"double","value":[["0.147445399070218","24.336274188752061"]]}}
%---
%[output:03b4dbba]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht1","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:45176e25]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht1","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:278dad4b]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht1","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.998429203673205"]]}}
%---
%[output:4efa0306]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ld_fht1","rows":1,"type":"double","value":[["0.147445399070218","24.336274188752061"]]}}
%---
%[output:19ae2427]
%   data: {"dataType":"textualVariable","outputData":{"name":"tau_bez","value":"     1.455919822690013e+05"}}
%---
%[output:8267420f]
%   data: {"dataType":"textualVariable","outputData":{"name":"vg_dclink","value":"     7.897123558639406e+02"}}
%---
%[output:5a4ef0d7]
%   data: {"dataType":"textualVariable","outputData":{"name":"kg","value":"   0.029677608778985"}}
%---
%[output:4e1ed4e5]
%   data: {"dataType":"textualVariable","outputData":{"name":"kw","value":"   1.231423274931087"}}
%---
%[output:6dfbc3d1]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l1","value":"   0.341393507746001"}}
%---
%[output:5d960356]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l2","value":"     2.034713425359889e+02"}}
%---
%[output:973a4095]
%   data: {"dataType":"textualVariable","outputData":{"name":"luenberger_l3","value":"    -2.508520275253434e+02"}}
%---
%[output:5345c40a]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQvYVUXZ9miIoJiJSoqkkEGWJnkqNBMPWabiWfGQFhKa5Snl46D\/hYXFKbA8fYQKqF8qWGmpaWqe+1CzUDwHSiqIB0TziKjof91js795F2vtPbNnZu1Zs+91XVy877ufeeaZ+5k1c+9nnplZ7aOPPvpI8CECRIAIEAEiQASIQIUQWI0EpkLeoqlEgAgQASJABIiARIAEhh2BCBABIkAEiAARqBwCJDCVcxkNJgJEgAgQASJABEhg2AeIABEgAkSACBCByiFAAlM5l9FgIkAEiAARIAJEgASGfYAIEAEiQASIABGoHAIkMJVzGQ0mAkSACBABIkAESGDYB4gAESACRIAIEIHKIUACUzmX0WAiQASIABEgAkSABIZ9gAgQASJABIgAEagcAiQwlXMZDSYCRIAIEAEiQARIYNgHiAARIAJEgAgQgcohQAJTOZfRYCJABIgAESACRIAEhn0gKALLly8Xo0aNEtdff31hPYMGDRITJkwQXbt29WbL7NmzxejRo0X\/\/v3F9OnTRffu3ZvWfd9994kjjzyyVv74448XI0eO7KBv4sSJYtq0abW\/jR8\/XgwePLjpOn0UXLBggRgyZIhYsmSJuPLKK8WAAQOk2ldffVUMHTpUtkH9zUd9PjFX9mSxx999+NSkvXrfzfN5no5sf9dx1+VVu3r27Clmzpwp+vbta2KSlGnGLlPl6Md4V2HTsmXLOvT7PB1F7TOpD\/1lzpw53t99k7opkwYCJDBp+DHaVpgQmBCTks\/JNDuJZidQRQjmzZsXPYEpIjU+OpBPzGFPlhTqNjYz8du2sRmioOOL+oqIT4wERvVj2A3SP3\/+\/IYEBrLNkHXl2xBfXmz9TPnqIkACU13fVcJyUwKDxrh8m8uC4XMyzRKY7OSZnbSaHdTLcGhVCEwepll8Ypz8VL9TthZFi2IkMApz4IroXF70K6+PNuMHEpgy3vb06yCBSd\/HLW1ho2+x+kSV\/SbXiDiohmUnO+jBk7eElNVpMvjqk82GG24oEGnRbVWTFnQ999xzq3yeNxnn1ZuVA6F79tlnO7QDy2xqSQ7f7gcOHNjhW7JuV5asKF15kYxrrrlGLoFl7VJta0TaQmGeR2zV5JcXhWm2z5hgnV02zL5YeiQOS4533nmnXL7Li1BkCcz6668vl\/VU33rmmWdqS5LZdmbfqXp9ADbmRQjziJXytbK3EclSfsjqyvZj\/fM8W\/RIlamtLR3UWHk0CJDAROOKNA2xITB6BKbe8oE+ITT6lqgPntlvx42+JavP9YEck8zZZ59dm+ghowjFaaedJm677bYOBKZeJEEnC6bt0AlMUY9R+IQiMKa2Iu\/IFXO00YRkQs53n8mSxUYEJjvhF5FC2FqPwBT5Vb0fJlFNJVtEGFBHlnjo+S\/IyTElMCb9WJEwnajp7QQZHzZsWI3EZTEoK+8pzVE43VaRwKTr2yhaZjLYZicpfYLUyUr2m3evXr1q5EEf4PTy6u9ISFQJrUqnPrjXW8fXB\/IzzjhDjBs3TiASgzwBPCA1S5cuFZMnT5aTqB6hUTbruRDZqIZNO7IERk1UOllRE8rixYtXSeItWkIqCum72OqCeVG\/Kcp9CdFnbAlMFkP0A0Ri6kWKiib2PAKi\/KqTZj1Cpfdn1d\/06KBKlNexypIiRBBV0nsjoqoGmDxbVf1FX2Dy+puprVEMbDQiCgRIYKJwQ7pGmBKYvOhL9pt3lnBsu+22q5AShWQ2vH3rrbfmLinlDZpZb+gE5sILLxQzZsyQOzVgMx5MUiBKIDDDhw9fZQlJ6ct+G1aTFz7PkquiduiTahaf7KQQgsDY2OqCOeqpFz3QJ249+uKzz9gQmDwynEcolF\/rRWCyib9ZWZ3wNuoD+s6+vHcxS4BV\/oseJao3OuVFX7KETdmvf8lolANTz9Z0R0u2zBYBEhhbxChvhYApgYFSREH222+\/Djkeetg++23uoIMOyt0mDF3ZJN6LL764wzbnbCPqhaiLlgYw0eBB7kg2BJ4X5cnWmUdgsonM2XbUm1TLJjCNbHXBXMeqUTRGn9Czk79Ln7EhMEVLZaod2f7VKAdG34KfjZpBl54Hpb8j2T6A+usdY6B8qOyxWZ7Ntk0R1qIBQic2eQSm0VjhM8nfahCjcLQIkMBE65o0DGuUA6N\/e8bAfMEFF4hJkybJCIfPychlMs1ONup8DAzIKqkXg2u\/fv06JGLqZAztzIb1SWCaO6Mnu1wUivSaEphGE696k\/UJuCwCc95559WIu4qWqKUtPYqlCEU9G7Nn1WQT8KEPifPNEhg9h6merWmMjGyFDwRIYHygSB2FCNgSGKy\/K7LhczmgaDnDxHXZyQZl1JIPfs7LY8A32T333LNGaPSJIZtXouvL5uJkl8JiisA0srVZzPU+kxcZ0ydORQqLliRclh1NCYzJlm\/4uN5yi57carOElJXVcRgzZowYO3as\/DJQFFlBv1QRHT3\/BfY2SuLNEpjNNtusMOcn+57VixTVs9XnwYsm7z5l4kaABCZu\/1TeOtNvp\/oAr39DNE3i1cPTpkm8JuQqbyDXlyx0u6FP3wqrExg10egTqrK52STeepMXEjZtcmD0JZC8xOA8W0Ni3mhJJhvVaGUSb9FWc2Vj3ueNdiGZJvGaEph6CcAqcrjpppt2OBXXNIlXRXL0CKRuVx65rEdgGiUrV35QZAO8IUAC4w1KKspDwIbAmG6jzgtzF6GflziYlW10qmveN1F9gs0jJ4p4mZ4mWzRZfP3rXxf33HNPbcurawSmKJG40bHxRWQlD3cfmJv0m6IE1jybTPqMDdaqDh3Pou3eeRGjRgSmXhvqEe8sMbjuuuvqLuvoieiNzmEqesf0dtcjnnlRSKUT71Dv3r0b2soIDOcZHQESGPaHoAiYTEQmW2NhZJFc3vHtajAsSp5UjTY5XyKPwORtRc3biZJtPwb7ESNGiBNPPLHugXeqrXPnzq17kF29BM68CAzanZ1ksomceZNKo4Ps9AnIB+bKhiJi1+iOIVXepM80g3WefUVb8fOWxNQx\/UXLj9Cv8kkaHWRXrw+A8OokOi\/PTL0rWUxNIjB5VyVk38c8H2SJtCJBes5Oo5y4oAMXlVcCgbYjMNmXkpntleinbWukzysR2hZEw4a3EmvTM4kMm0IxItAWCLQVgQF5wTkd6vbX7O9t4XE2MjoE9MlLj16Y5uhE16CIDYoVaxKYiDsNTYsWgbYhMGoyQDgzG3KFdxodEx6tB2lYEgjUy5VBAxkp9OfmGLEmgfHnX2pqHwTahsCoAQJERU8EQxQGA5o6Prt9XM+WxoZAXgJkowTj2NpQFXtiw5oEpio9h3bGhEDbEBgkliF5Eoek6QcycRkppu5IW4gAESACRIAImCFAApPJizGDjVJEgAgQASJABIhAKxEggSGBaWX\/Y91EgAgQASJABJpCgATGgMB89rOfbQpcFiICRQh861vfEhtttFHt43fffVeez8KHCBABIhACgYULF4ZQ21KdbUNgXJJ4QWBSdL5Nz4sVgzLt8lXXk08+mUtW9t9\/f\/HlL3\/Zxi1S1sUu27Ih5G11WgNUkQKx4lCmXSHq8qHTRYdt2RDytjor8sqItiEwLtuoU3W+TSf917\/+Jfr06WNTpBTZMu3yVddll10mnnnmGYkPIjE333yz\/BlRGJzmavu42GVbNoQ836+PPW6LrW0\/aVa+TLtC1OVDp4sO27Ih5FN9x9qGwODlVafw6ken6wfbFb3gqTrfZkCzfalsdLvIlmmXj7r+\/e9\/i3PPPVc2uUuXLuKwww6TExfuO1KExva+Fxe7bMuGkOf7RQKjxgDb\/mUydvjQ6aLDtmwI+VTfsbYiMDqJUR3f5ICwVJ1v8vJTxi8CevRl4MCBYtddd5XRGPxdPWeddZbfSiPXZjtgR94cmkcEokMg1Tms7QhMMz0rVec3gwXLNI+AHn351Kc+JU455ZSaMp3YDB48WGyxxRbNV1SxkiQwFXMYza0cAqnOYSQwBl0xVecbNL0mEuskU6ZdrnVdfvnlcrkIz+c\/\/3lx+OGH1\/Ie9CgMdifhll\/Tx8Uu27Ih5G11muJSNblYcSjTrhB1+dDposO2bAj5VOcwEhiDUS5V5xs0nSKeEMjmvuTdvYXcGMjh+epXvyr22msvT7XHrcZ2wI67NbSOCMSHQKpzGAmMQV9L1fkGTaeIJwR0coIbpw844IBVNOtRmJUrV4qxY8d6qj1uNSQwcfuH1lUfgVTnMBIYg76ZqvMNmk4RDwjccMMN4h\/\/+IfUtGLFCjFu3LhCrTrRwRZr2x1JHswtXQUJTOmQs8I2QyDVOYwExqAjp+p8g6bXRGKdZMq0q5m69KgKznnBziN9aSirU19qws9YatJP7M3zWTN2KT22ZUPI2+q06bdVko0VhzLtClGXD50uOmzLhpBPdQ4jgTEY4VJ1vkHTSWA0kGwHFp2MgLzgyR5Ul6dT35H04osviqlTp9Z1la1dujLbsiHkbXXa9NsqycaKQ5l2hajLh04XHbZlQ8inOoeRwBiMcKk636DpFGkSAZ28QIUJEVFVZcs2WnZq0sRoitkO2NEYTkOIQEUQSHUOI4Ex6ICpOt+g6RRpAoHswXQgLzicrtFSkF6VrgOEBvkwqe5KIoFpopOxCBGwQCDVOYwExqATpOp8g6ZzCclyCSlLXubPny+OPvroQvJRb\/LWl5JwASTyYfIue3QhALZlQ8jb6rTpt1WSjRWHMu0KUZcPnS46bMuGkE91DiOBMRjhUnW+QdMpYogAoiQPPfSQuOuuu2ol8Pt3v\/vd3C3TJmqzS0kgMSNGjBDbbLONSfHKyNgO2JVpGA0lApEgkOocRgJj0MFSdb5B0ynSAAGQDPzT7zJCwi7IxgknnCC++c1vOmGYJTG4kHTrrbcWEydOdNIbU2ESmJi8QVtSRCDVOSwpAjN79mwxevToWv\/DcezZE0\/VjdRKiJc5pvi6ltMmLBf98Y9\/rJ2ei1pBXhB5AXnZb7\/9vBiSR2Jwl9Ivf\/lLq7waL8YEUEICEwBUqiQCGgIkMJF3B5CX888\/X8ycOVP07dtXvPrqq2Lo0KHyIDBFYkBehg8fXpPJ\/l7UxFSdb+PSWCeZMu1CXeutt54kLFgqAoFRj4q6dOnSpTBXJQ9vU\/uzJAYkSdW17rrrij59+ti4syZrWr8qEELeVmdTDa1AoVhxKNOuEHX50Omiw7ZsCPlU57AkIjDLly8Xo0aNEjvttJPATb7q0QlKr169pEzPnj07RGVUKD7vbhqlJ1Xn24zpti+VjW4X2TLteuSRR8Ttt9\/eIeIC27HLCGQGUZGiRNuiNtrYn0di8LcvfvGL4sILL2wKRpv6UUEIeVudTTW0AoVixaFMu0LU5UOniw7bsiHkU53DkiAwRWPTggULZNLjpEmTxPrrry8jMphg9OPZQXJAYqZPny66d++eqypV51dgTG+5iSAIWCbSoy3KKEVcEH3BFukylnQQeYE96lHJw6jfljy1HNz\/GGA7YMdiN+0gAlVBINU5LGkCg2WlWbNmSXKybNmyGpnBElNelEb\/u94xU3V+VV6+Mu1Ut0EXkRaQleeee04uUSriUjZxyEZiVN5Nq+xx9Q8JjCuCLE8E6iOQ6hyWLIHJ5sDo0RgSGPvXPdZJxoddiK7gvJYXXnghN9ICtFSOiyI4+BsOlsM2aZsD6rLIN2u\/2vmk24OdT4gK4TGNyNjWH0LeVqd9761GiVhxKNOuEHX50Omiw7ZsCHkSmGqMAdJKlRODb8pqaYgEpkIODGiq2vacTcLNVol8FvzDEiMIjrrLSJGDMpaLGsGAttx4440CfVs9aklJ\/W5KZBrVFfJz2wE7pC3UTQRSRIAEpiJezSMvMN2VwOjNv+222yqCRvua+dZbb4m3335bPP\/882Lp0qXipZdeaghGt27dBF70119\/Xdx\/\/\/3yLBf92WCDDWQe1ec\/\/\/mGusoUQNtuvvnmDlUiV0aPzsD2H\/zgB+KTn\/ykwM8xPYsXLxZIsudDBIiAPwT22GOPDsoWLlzoT3kkmiq3hJQ962XQoEFiwoQJomvXrrmRF4WzWlJiEm8kPc\/RDEzOiJAgOrL66quLBx98UE7Yecm2eVWhLJ7ddttNTuoo9\/DDD4trr712FfEqRDHyko313Bi9UbG1hxEYx5eBxYlAAwQYgYm8ixRFXpTZ6nNuo27OkWVPMoqIfPDBBzI\/BVEUPKYERW+lIiuInGyxxRaid+\/etY8vvfTSDqfoqg8wyW+\/\/fbiqKOOquW4hMDAh05dR3aXEtqDyNJf\/vKXDstgqp2Ixpx55pm5dyzl9RRbe03kTWSa67XVKhUrDmXaFaIuHzpddNiWDSFPAhP5WICt0MhXqLcdWp3Cq07f5UF25k41falUZASa9SgJfgeRWLFihZg3b56cTF977bXaMoe+3GFuVUdJlbeCJG2cjaIeRWDwO5Jcf\/vb34prrrkmt5p60QlTDGzs96EzT8ff\/vY3cdNNN3UwZautthJ\/\/\/vf5XuS9+CiSCQl510YqeRt7TWRN5GxwbSqsrHiUKZdIeryodNFh23ZEPIkMB5HBXyrfvPNN6XGddZZR3Tq1MlJO\/JbhgwZIpYsWZKrR78ugFcJ5EOdJRBZIvL+++8L5Cq88cYbHXIrVDkfBKReJ1AkRJGU\/v37S0KUR1KyehCVQI7In\/\/850LSghOaN9lkE6cdRU6dOEDhojNstt12W5kfhDwfYJP3KCKH\/112WZk0y3bANtFJGSJABP4PARKYJnoD8k7wLfCdd94R3\/72t2Wi3gMPPCDPY3n22Welxo033liekLvPPvvIXIYYnzznqwkbk2jRz2iLOqEVP2dJAf6mJuHsZ2uttZYA0UPEAMtfiFjg\/w8\/\/FBChOhF3pNHRGLEVG+7IiXIY+rXr5\/ERMdDJyombcGk\/PLLL8u+V2+CHjZsmIzUhJ6gTWwOKZO35Rr1ISKz5ZZbyr6FyyjrYYUbsI855pggWJHAhPQ+dRMBITcnMInXoicgbwEX2mFwwoPckzFjxohzzz1XEhqcoYEH34pfeeUVMXXqVLHzzjtb1FCe6E9\/+tPyKqtITXnRD\/wNxKtHjx7io48+qpEzPXriu3kgxDvssIOMDk2ZMqVwEka9ICpqiaQZ0hJiovWh01QHCAq2j2dJLvwDXEBkQJjrkRmF43e+8x2x3XbbNSQ0JraZyPjuNzHqixWHMu0KUZcPnS46bMuGkCeBsXjjETlATgrujcEx\/vimh2Wcs88+Ww56uHRRTSAYME866SSx+eabi7Fjx4rOnTtb1FSOaMwEpig6kf07LiEEqQDBwK4bXASISwDzIkh5Om2jICE9g4kYL\/ndd9\/dkLDsuOOOYpdddjFOUg1pdyy6iyIy8PH+++9fS3LGu4lI6a9+9avaAXl5bcC7jGWpPffcU77XtuTQdsCOBUfaQQSqggAJjIWnkCdx4oknyh0fo0ePFquttppYtGiRzFPZfffda39TKkF2kFx40UUXydt+Y3u+9rWviSuuuKKDWWrizy51ZJeTsgShqG26HGRMy8WGlW97MIni36233ioPbav3qMlTnY5rO5H6tj12feiriMjkLR0dcMABAnlG6lF+eOqpp4wujgT2p556qthss80aEhoSmNh7Cu2rOgIkMBYezB7jj6J5f9MJTKMdRBbVexdN1fnegXJUqI7B\/5\/\/+R+ZkF2Uk6GqwSSJHTPf+ta3mvrm72huMsVBZB577DG51Tr7ZKMyWUKDw\/5wb5TyXREoilwi\/w1faHRySQKTTFdiQyJFINU5LMhBdiQwkfZiB7NCTDIgKJg0cVpuI7IC0zHpHXTQQQLbpNWEGMKuIphC1OVDp4sOvay6ZgGEJG9X2Te+8Q25\/PilL31pFYgUgYEvf\/Ob3xj7E37EshWite0eMXPxo8Or3bBomXaFqMuHThcdtmVDyJPANOzm\/ydAAmMBVkVEbV8qvVkgJ\/iH819MiIoiK1\/96lfFrrvuWje64mKXLfQh6vKh00VHUdl6y0uIyuy3335yubdebhRIDQhq9+7d5RKgb9\/b+i92eRc\/hmxbmXaFqMuHThcdtmVDyJPAWLwhisAgvI8EXTy4X+bkk0+Wp5uqvymVSOrF4FbvEDqL6r2Lpup830DBh5iscNGhypkwqYNLQSYotUamUVRGLTGprfCNrFSRmr\/+9a\/if\/\/3f61IDXTjLiqcHtxMsnAj2\/g5EUgVgVTnsKBLSPjGbfogYZAExhSt1smpCQjn0CDp2paowPIBAwaIgQMHcmdQ69zYdM15VxUoZUX5Mo0qw1b4NdZYQ+6Su\/zyy41JjYrU4X\/sgNp7771lVe2+FNUIb37efgiQwFj4HMfF42I8\/G\/6rLnmmmLrrbcW+D+2J1Xn18MZExUmgt\/\/\/vcCO09Mw\/\/6BKKOpgfJCfWN2Tbc6tK3QtTlQ6eLDtuySr5e4i8wVmQGpKRPnz51Yc+zQfUZ9LvHH39cHoBp0wdVP8QOwgMPPFDepRWqD7r0Kb2srS981dtIT5l2hajLh04XHbZlQ8inOocFicA0eiGq9nmqzlfRFJwWjFu+baIpWaKCnUDqb\/wGXLUe3ry9aompXn5To2UmmwFb3+10xx13CNz51AyxQYtxEzlyeUIS7OaRZUki4A+BVOewIAQGx90j5wXfvkwfbK3EwWoxXidQZecrUrJy5Uq5OwRPMwM+SMnnPvc5gW+0JCqmvbq95EzJzL777ivWX3\/9WgKwDYEpQlQRG\/y\/bNkyccMNN1gTcr1f4+ZyLHWyr7dXH061tVWew+r5JAiBUUm8zIEp53VQyz3Y8fGPf\/yj6YEb1mLZB99MN91009rgXU4rWEtKCIDMILKHk3yLCDMiM9hptvbaa+duzfaFhx5ZVJEiWxKvRxxxmji2lKv3xZed1EMEQiFAAmOBbDYHBpEYTK4I9+IOFZzOiQd3IOHelRdeeEHek7THHnt4y4HBwXi4YXjmzJny3BD1VO02aj0XAJEtbEfF31wGYERT1DZY07wAH9+SLbqQsWiZdoWoy4dOFx22ZZuVB6FBXhyWfYqeRktNxp3CQlCRG7wHGIuafbf0SI1+ro16f01MssXWRKcPmTLtClGXD50uOmzLhpAngXF4E6655ho5cP385z+XB2Hpz1tvvSWvFsAAMnLkSNGpUyeHmj4uqiJASNzTCUyW1BSRnKwBoZ2vD6J\/+tOfxKOPPtpUFEX\/lohICu4Awg3PKifFJTfF9qVydqKhgjLtClGXD50uOmzL+pAHmUFiOLZR5x2YB9eDzGBX0YYbblj3rBnDbmItpi9JofDNN9\/sTG6gB9GbwYMHy2Wu7JcHW2ytG9VkgTLtClGXD50uOmzLhpAPPYc12bWciwVZQtKtUvci4aXdZ599cg3GpH3ppZd6uwsJdytNmzZN3oCtCMzy5cvFqFGj5N9AlNQDWTz630IQGLXMg0EbZ2DgaSaKokiKfu+P\/i3PhaQ49yYqIAKWCIDA4B40nOCLJae8R50xg633pufNWJphJa4nEuPnf\/7znwJfhmyT4PVK1fuMpVss4eLBlxA+RMAHAiQwTaKI80KOO+44ccghh8hvHnnPjBkzxE033eSFwGAgASkZNmyYGDduXI3AqKgMiIpKzoMtSr7eGTSNnK9\/WwN7xi3JrgRFDWAgXDi63UcUpUkXshgRCIqA+sapkoCRN3PnnXcW1gkSg+XmXr16tSQ60wiMLMGxuS6jSLciOGjzUUcdVRPjF5ZG3uDnQKDRHFZVlIJHYD744ANJKObMmSPOOeccgex+\/XniiSfkzdUIF59yyilOS0iKpBx++OEyz0bPgVmwYIHARXKTJk1aJScmL1dGt7Ffv37i6quvliQC7bjnnnu8EBTo23bbbWsJjDF\/47INa5b1QpRpV4i6fOh00WFbNoR8kU6VCIzE26LoDPpZK3JnXPu3itZgSf23v\/2t0\/IUbFEEp3fv3nLpWF2Y6YPg2PrcBZsQdfnQ6aLDtmwIeRIYh1758ssvy2sE5s6dK7baaiu58wDP\/fffL\/M9EBGZPHmy6NGjh0MtQp5lAoIxYcIEee+OLwKjQrqmxunRki9+8Yti0KBBtbMm1GBjqotyRCB1BEwGbEVmkDN322231Y3OYBJXy01Vwy4bvUEE+7rrrnP6wqSPORibDj30UPmNXD0+SE7VcG43e0lgHD3+9ttvy1Nd\/\/CHP0jSggdk5ogjjpDRF2yldHmyEZZsgq5LBCZLYHAXCx78D4KCS+3wM862UJ+5tIVliUA7IbB48WK5HGTzgMhgTEEe2UsvvVRYtFu3bmKnnXYS+B\/\/qv5g5ybGGPyP3BucnYXdnfj9ySefdGoe9OIfznvacsstpa5sxNypAhYuFQEss+rPwoULS62\/jMqCLCHhWwOiITvssIOc4LETJuSjEnQxUKk8G58EBm24\/fbbZRPa9duKybfkkD4u0l2mXSHq8qHTRYdt2RDytjqzfUHtZMKuwxtvvLFwZxPKYbkJEQjklsX2uOKg2qNHcZAcjRwctWSlf2bbfjX2YakbXzw7d+5cU+FrXPSFgd42HzpddNiWDSHPCIxFbwehwJkKWNtdtGiR3Dp40EEHyVNc0dFdTtsFMcK2a\/Vgeeb73\/++OOGEE8SSJUtyrRw\/fry87A032YZI4rWAprKiti9VWQ0t064QdfnQ6aLDtmwIeVudjfqWSgbGeIBzk+o9MeXP+MahqN3ZZSqVNO2yi0rVpXJxMOYjwm56zpQqHwIDHzpddNiWDSFPAtNo1Mj5HAfYIbyL02H\/+Mc\/irvuukueA4ND1LBsFDI6k43AtHobdRPwsQgRaAsEbAdsW1AUocHuQNRVFUJj205f8lmCA\/yQr+hyyJ9OcPAziM33vvc98elPf5p3UflyXB09JDAeQH7vvffEI488IkOat9xyi\/fojG5i3iF16hTeK6+8UiYOx3KQnQdoqYIT5x0dAAAgAElEQVQIVBaB0AQmC4wiNPhCVW93E8rFFKGJzcF6xAZ5TEiu9hnF2XjjjeWVDVjuA+GxOdU4NqxabQ8JjGcPqOgMdg39+c9\/FsibweFz3bt391JTKlcJeAHDg5KyJxlTk8u0K0RdPnS66LAtG0LeVqdp37CRA5FpRGjUIXq4KgA\/+35iwCGvTc3YlY3igOA89thjXnNxQGr0SzeLjqFoxv4sDi46bMuGkCeB8f22\/kcflnZwrgpIDHJbcHR4bE+qzrfB2falstHtIlumXSHq8qHTRYdt2RDytjpd+otJWdPzZ1SE5rvf\/a5U60pqYsNBYRXSLj1i89xzz8kDTbt06dL0KeXKZv0oi7322kv079\/faanKBQPbsiHkU53DguxCajRIIPqC5N6rrrpKYDnnzTfflB2s3mm4jXSG\/DxV54fEjLqJgCkCtgO2qV5fclxy8oWkvZ5sJOfpp5+WV7H4WqqCRSA72AyCnMwsAbK3OM4Sqc5hpRIYnMqLMwsuuOACmX+CB+ucP\/zhD2Vi7zrrrBOl91N1fpRg06i2QyB2AqM7RG3Zxv+NlpxUVEbd\/O4aoWm7jmHY4Oy2cVxDgSUrnyQHJ6YjkrNy5UrrnVWGzQgqluocVgqBwfkM1157rdxa\/cILL9QcNWXKFMl8fdxAHdL7qTrfBrNYJ5ky7QpRlw+dLjpsy4aQt9Vp02\/LkLWN0Bx44IFyN2aW0MSKQ5l2hajrgQcekGeSKUKDw\/9wNYyPs3FUBAf\/Iwdn3333lQeaqoRjRHds2xRCPtU5LBiBQbQFt8ziokYwYvyOTnTMMcfIUx5\/\/OMfr3ImSxmDTTN1pOp8GyxsXyob3S6yZdoVoi4fOl102JYNIW+r06W\/lFXWJClYRWjUxZTYyNCnT5+yTDSup0z\/hKjLRKd+IS9+BsnBtnsfW8cVyVFn4uCaC1x3oZ7sIYAm9urOM5FPdQ4LQmDwIuJgOSwXgazgEDswU5WgW3QztPEbVbJgqs4vGUZWRwRyETAZgKsMnW1S8Oabby4PgVO7nqrc9pRs10kO2oV8nKeeesoryVGRnK9\/\/etirbXW8rZcleocFoTAKIKyYsUKccghh8hTcDfZZJPaCbwkMCm91mwLEXBDIHUCk0VHLTmpE3DroadIjLqcknk0bn2tjNJ67o1+GjTu7XJ9VLRmxx13lLeOq6doC7n6nATGAvkPP\/xQ4PJEXNyodhkhNAoyg4S2NddcUwwbNoxLSBaYtlo01kmmTLtC1OVDp4sO27Ih5G11tvpd8F2\/IjQPP\/ywzJdQicJF9YDEYOt2WWSmTP+EqMuHThcd2bJ5Jx3jxnHszvVJckB2jjvuOHnZ8O677y4ef\/xx31235fqCRGD0VuGcl3vvvVdcccUV8rwX5MJ85jOfkWG3Sy65RCBUFvuTKnu1wd3lBbapx1a2TLtC1OVDp4sO27Ih5G112vaRqsgrHBShef311+WEZnJaML4Yrrbaah1yK3y1u0z\/hKjLh04XHbZlIa8uQFZkB0eN4NBXl5ycO+64w1eXiEZPcAKjtzS7Gwm7j3bddVdx7LHHiu233z7a3UgkMNH0VxqSIAK2A3yCENRtUjNbt\/v27SvPNWEeTdq9RS1XIdryxBNPyLzTvN1VCBwggJDaUyqBUeBhiemf\/\/yn3FaN7dXvv\/+++MpXviKmTp0qw12xPSQwsXmE9qSEAAmMvTdtt26rO51QU1lLT\/atYgnfCKjt3P369RPz58\/3rb7l+lpCYPRWIzSG7WoIb51xxhne7kLyiSwJjLA+y8An\/vV0lTn5hajLh04XHbZlQ8jb6iyrb5VdjwsOitCo7duNbAeJOeKII0SPHj0aiZb67rtgUNQQHzpddNiWDSGf6hzWcgLT8O2xEFC7m+bNmydL4ZC8CRMm1NYT8Td1I7VSq26mrldNqs63gLbUQSxWu2wHFpN2+NDposO2bAh5W50muFZRxjcOIDWIdD\/55JNGeTQ77bSTPOoiu+zk266yv5D4sN9Fh23ZEPKpzmHJEBhFXgYMGCB3NyF5eNSoUfJdUSQme0N10Y3V2RcsVedXcZCnzekhYDtgp4dA+BY1k0cDIsPt2+F9U0YNqc5hyRCY2bNnyyxtPeKCrdwjRowQkyZNEr169ZKEpmfPnpLgqGfixInyR\/1vJDBlvFKsgwh8jAAJTGt6glp2QsTaZPsuCA2i2pgM+VQLARKYiP2loi0IgQ4ePDjX0qLD8xCFAYmpdxN2qs63cWmsk0yZdoWoy4dOFx22ZUPI2+q06bdVkm01DorQvPzyy3JHi8n27Z133lne\/eNrt1MIDHzodNFhWzaEfKpzWBIRGJ2c4IbYadOmyXFLz4HRozHYYqgek2WkVJ1vM7jbvlQ2ul1ky7QrRF0+dLrosC0bQt5Wp0t\/iblsbDioZSfk0CCXxoTQuC47hcDAh04XHbZlQ8inOoclQWBAToYMGSJwbLNKys3mwOB6dbWcRAIT8zBO29oNAdsBu93wiam9KkrzyCOPiLlz5zY0DYTmqKOOEhtssEFDWQqEQ4AEJhy2zpoVgTnppJM6LCGpv0+ePFmGOV0IjG7kbbfd5mwzFRABIvAxAvhygRw1PtVD4K233hJvv\/22vP4AEZqXXnqpbiO6desmtthiCzker7322gK\/8wmDAG4515+FCxeGqaiFWpOKwICoYBeSetTS0uGHHy623XZbJwKTovNt+l2s35LLtCtEXT50uuiwLRtC3lanTb+tkmysONja1cwhe2q302uvvSZwb57Px9b+vLpddNiWDSHPCIzPHuWgC7uNRo8eXdOAPBdEVk488cRVLofUCQxuxB46dOgqMkziNXOG7UtlptVdqky7QtTlQ6eLDtuyIeRtdbr3mjg1xIqDq12K0CDSZhK9xrLToYceKtZaay0vpwa72o\/e4qLDtmwIeRKYON\/5mlV526H1JaT+\/ftzG3XkPqR57YmA7YDdniil02pFaHAKO\/JoTJKDN998c7HVVlt52+2UDppmLSGBMcOpZVI6WcEykoq+bLrpph0OsjvyyCNrib4mO5DQoFSd3zJnsWIioCFAAsPuoHJocCZNI0IDtNTdTr62b6fugVTnsCRyYFTny14lcPzxx69yQB2vEmjuVY11kinTrhB1+dDposO2bAh5W53N9eD4S8WKQ5l2qbr0M2nuvfdemSRc7wGRUVch9O7du4OoD\/tddNiWDSFPAhP\/+x\/MwlSdbwOY7Utlo9tFtky7QtTlQ6eLDtuyIeRtdbr0l5jLxopDmXYV1aUIzLJly8Rf\/\/rXhlEaFZnZf\/\/9xaJFi8SXvvQlJ9e7YGBbNoR8qnNYUhEYpx5ap3Cqzg+FF\/USARsEbAdsG92UTRcBtez07LPPGl+FgM0cSA7ORmnSRenjlqU6h5HAGPTcVJ1v0HSKEIHgCJDABIe4LSpQy07vvPOOeOCBB4yiNCAy2OCROqFJdQ4jgTF4tVN1vkHTayKxTjJl2hWiLh86XXTYlg0hb6vTpt9WSTZWHMq0y2ddtlchoK9g6ek73\/mO+MQnPtFhC7eLXbZlQ8inOoeRwBiMcKk636DpJDAaSLYDiwm+PnS66LAtG0LeVqcJrlWUiRWHMu0KUVdWJ3Y5vfjii+L+++83Sg7eYYcdxOqrry5PEAbBsX1s2xRCPtU5jATGoDem6nyDplOECARHwHbADm4QK2gbBFSUBv\/jIuBGW7j15OBmyEyrgE11DiOBMehRqTrfoOkUIQLBESCBCQ4xK7BAQOXSgMyA1DR6QGT22Wcf0alTp2hzaVKdw0hgGvXOhDO4DZrOJSQuIXXoJrZkw0TeRMamr1ZVNlYcyrQrRF0+dOL27eXLl4snnnjCKEqjkoNBbmzvd7K110SeBKaqo4IHu1N1vg00Ji+JjT5fsmXaFaIuHzpddNiWDSFvq9NX34lNT6w4lGlXiLp86NR12C47oZ+ByBx99NGie\/fuDbudrb0m8qnOYYzANOxO6e6hN2g6RYhAcARMBuDgRrACIuCAgCI1jz76qHj66aeNojQ77rij6NGjRyn3O5HAODi36kVTdX7V\/UL700CABCYNP7IVHRGwud9JJQcPHDgwCKFJdQ5jBMbgrUvV+QZNr4nEOsmUaVeIunzodNFhWzaEvK1Om35bJdlYcSjTrhB1+dDpokOVVcnBzz33nLjjjjsadk2Qmm9\/+9uic+fOdZODTWxLdQ4jgWnYjbiEBIhMXhIDKL2LlGlXiLp86HTRYVs2hLytTu+dKBKFseJQpl0h6vKh00VHUVlFaN58800xd+5co2WnzTffXOy8886yx6pt3Ca2kcBE8pK3woxUnd8KLFknEcgiwPeLfYIIfIyAIjUmZ9IoErPffvuJ9dZbr+4he6m+Y4zAGLw5qTrfoOkUIQLBEeD7FRxiVlBRBBShWbp0qXj88ceNojRf+cpXxBe+8IUOhCbVd4wExqBjp+p8g6bXRGLFoEy7QtTlQ6eLDtuyIeRtddr02yrJxopDmXaFqMuHThcdtmXryavdTjhb5u677zYiNIjOTJ48WTz88MNVeh2MbCWBMYAJHYoPESACRIAIEIEYEdhoo40E\/uFm7aLnrLPOitF0J5tIYJzgY2EiQASIABEgAkSgFQiQwLQCddZJBIgAESACRIAIOCFAAuMEHwsTASJABIgAESACrUCABKYVqLNOIkAEiAARIAJEwAkBEhgn+FiYCBABIkAEiAARaAUCJDCtQJ11EgEiQASIABEgAk4IkMA4wcfCRIAIEAEiQASIQCsQIIFpBeqskwgQASJABIgAEXBCgATGCb6PC7\/66qvi9NNPF2eccYbo27evB41UQQSIgHq3hg4dKubNmycBGT9+vBg8eDDBIQJEwBMCy5cvF6NGjRLXX3+91HjllVeKAQMGeNIeVg0JjCO+CxYsEEOGDJFaZs6cSQLjiCeLEwEdgYkTJ4revXtL0qLeNRyLXpUBlt4kArEjMHv2bHklwciRI8V9990n8M5Nnz5ddO\/ePXbTBQmMEHJgHDdunJgyZcoqToNDjzzyyJojdXaKyMvFF18s9tprL\/GTn\/xETJo0iQQm+i5PA1uBQLPvmG6r+qZ4xBFHkMC0womsM2oEfLxjmO+uuuoqMWHCBNG1a9eo2wvj2p7AgIQgRI0nyzrhzOHDh9ciK9nflXfRcUaMGEECE313p4GtQMDHOwa76w3QrWgX6yQCsSDg+o7py0hcQorFqw3s0KMruARLJzDKoT179pShNfUgvIZH\/xsJTEUcTjNLR8DXO8Y8s9JdxworgoCvdwzNVUQI81sVlmnbNgKjnI6kQDyzZs3qQGCKHJm3RkgCU5E3nWaWioCvd4yRl1LdxsoqhICvd0w1WX1x32mnnSqRLN+2BEbvo0hiyhKYIlKSt4xEAlOhN56mtgSBZt8xGHvZZZeJM888sxJr8i0Bl5USASFEs+\/Y3LlzJX4qUb5K6RAkMA6OV1umSWA4fhCB+gg0M7hOnTpVXHLJJbXtnaqGKq3Rs18QgbIQaOYdw87ZXr16cRt1WU4KUU+zjueZLyG8QZ0pIsB3LEWvsk0xIdCO7xgjMB4iMDF1YtpCBGJEoB0H1xj9QJvSRaAd3zESmAICY5PEm+4rwZYRAT8I5A2ufMf8YEstRAAItOM7RgJT4HibbdR8fYgAEbDPgeE7xl5DBPwhkEdgUn\/HSGAKCAy6ldqippIGiw6y89cFqYkIpIlA3uDKdyxNX7NVrUGgHd8xEpg6BEYfYFWX5A6I1rycrLXaCBQNrnzHqu1XWh8PAu34jpHAxNP\/aAkRIAJEgAgQASJgiAAJjCFQFCMCRIAIEAEiQATiQYAEJh5f0BIiQASIABEgAkTAEAESGEOgKEYEiAARIAJEgAjEgwAJTDy+oCVEgAgQASJABIiAIQIkMIZAUYwIEAEiQASIABGIBwESmHh8QUuIABEgAkSACBABQwRIYAyBohgRIAJEgAgQASIQDwIkMPH4gpYQASJABIgAESAChgiQwBgCRTEiQASIABEgAkQgHgRIYOLxBS0hAkSACBABIkAEDBEggTEEimJEgAgQASJABIhAPAiQwMTjC1pCBIgAESACRIAIGCJAAmMIFMWIABEgAkSACBCBeBAggYnHF7SECBABIkAEiAARMESABMYQKIoRASJABIgAESAC8SBAAhOPL2gJESACRIAIEAEiYIgACYwhUBQjAkSACBABIkAE4kGABCYeX9ASIkAEiAARIAJEwBABEhhDoChGBIgAESACRIAIxIMACUw8vqAlRIAIEAEiQASIgCECJDCGQFGMCBABIkAEiAARiAcBEph4fEFL6iCwfPlyMWrUKHH99dcXSg0aNEhMmDBBdO3a1RuWCxYsEEOGDBFLliwRV155pRgwYEDTunVdeUqy9t93333iyCOPFD179hQzZ84Uffv2bbrusgvOnj1bjB49ukO1IfyT165mfPbqq6+KoUOHinnz5tXFe+LEiWLatGmimbboffj4448XI0eO9OYW2IV3A\/1k2bJlst\/Ue1z6Mnw7Z84c7++aNzCoqG0QIIFpG1dXu6EmBAYt7N+\/v5g+fbro3r27lwY3MxkWVdyIwGTtryKBaeQn3\/7xRWAU1krf+PHjxeDBg1dRHyOBUeQLxqLvz58\/vyGBgWxRG+u9OC7t9\/JCUgkR0BAggWF3qAQCjSZGvREu3y6zYJRNYPSJpYoEJksE8jqX7+iDjw6sJmalqyjC4jKBh4rAqD4KmxHVMfEB2tlMFMml\/T78RB1EQEeABIb9oRIINBr8daKR\/WaZF\/nIIzlZkoQB\/thjjxU\/+tGPVllCyuo0iSzUI0P6Eoaa4HUCc+GFF4oZM2bUltDy6stbtslikUcEi5aoshOhyYSnbMjq1OvNs92UQOg2qTquueaaDss6ixcvtlr20\/0C7LFEhCevj2QncLRLLT0B62eeeaZWvh4GqGfgwIEdIiVZX+l9Qr2k9fyuyjcivqoNWV31\/J1nC2xSfdXU1koMNjSyMgiQwFTGVe1tqA2B0SeevEldIalHAooG6LzITtE33Ea5Ks0SmCLP64SiXjvVxFYvipW1vUhfI6KmlzOJtNTD3dSmbNTElsAom9G2Cy64QEyaNEkSxTz76xGYIj+p\/mgSRVSy9XDJ+kDPf0GelCmBMek\/qi60TRE1vZ3AaNiwYbmfQa5Rf2nvUY2td0WABMYVQZYvBQGTwT8bFs9+s1ZJk\/okqyaMvL9lJxHI9uvXrzZYqwmuEblSAJnkwEA2+006265slGP99dev2ZQ3WaqJKm9iz4v85EWzdLl6uRNFE2\/RRKYIgU5WdB3Kdj3SUTTx1mtnUSfN851OaLL5VI0ITB4BUXbBBj0RPU9W9Sllg95WnThn\/fzcc8\/Vcr9Ml5CUDht\/5y0hmdpaykDBStoKARKYtnJ3dRtrSmDyoi\/ZyTM7aZ188sm1iaVoJxCQg248eTuD1KRR7xunCYFpNGGh\/kZ5OUVLATqBqRctKprA8yaqvB5Vr51FRCVLirJRBH1nje5j3ZfNEJg8LOstR9YjMNmITbYNvXr1Kuxn9XJL8vp+lnyo\/Bf4w4TA5JHAbN\/N83ejHJh6tlZ39KHlsSJAAhOrZ2hXBwRMCYwewag32OqfjRkzRowdOzZ32SA7wT377LOrbA\/WDa1HDBoRmEYToNpGnTfp1pu09Mkqm2sC27P15snobTRdFmgUjQEpKdqinm1jEXHE37N+tllCatTWLKFtlAOj71zKtgG4qQhMEeZF0ZrscKAIjPK7TgBNCAz0KT9efPHFtbydvGFH93feO9Xo3fSZVM9hkQjoCJDAsD9UAgGTZZpscqIamPOST1tNYEwG9aJchuzEiCUkRQRUpEifLLPtL5pw1CTYaFI3JTDZjpVdptPtzuJRBoExyXvKEtKyCMx5551XIxXKfzijRp3vovBS9uj4NcqByUaY9MTjZgiM3l\/q2VqJgYZGVgoBEphKuat9jW2GwNx6660yWlLGEpKJZxot\/WR1mBIYFZ3Q25m3tFJ0wF+W+BXh1qiNebkrep16ZABkac899+ywg0ePXpSxhGQaqdCjJb6WkOpFYPSIYFFkBYRFkVQ9\/wU+siUwKJP3nuT5O9t+yKioUj1bXQ6AbNTv+Hn7IkAC076+r1TLG4Wp9cbk5ULoE0ajJF41EJsm8eYtY+SRhdAERkVfMFnok3Ojb8XZpGDoURGdvB1M9XYXNYreQLce1WhlEm\/RdmLVl\/I+95XEa0pg6iUAq4TyTTfdtMOpuKbETPUXPRLWyN\/1CEyjZOVKDTg0thIIkMBUwk000obAmG6j1r8xmiwn5O1Yynqm3tJQKAKj74zK6ymmeRV5kYasvkZbxU1wNN2+XpRUqtsEezbccEN5BYBNEm\/RDjVddzZihAhRIwKTh3\/ezrBGOTDXXXdd3VwrPaG8KAG60ajRKDcqSzbxe3Z7PdrRu3fvhrYyAtPIG\/y8GQRIYJpBjWVKR8CEwBRNrs0eZIcJFFuvhw8f3vAgu0YTOwALRWAwOWTbiElts802y90xZXLgHezNfpO3yX3Jq6MeRtnITdGhebpN2SRUGwKTF4XLduq8JTGVm5K3vRuY41F3QDU6yE6\/CykvOVbHJO+MGkUcsqTZJAKTF0Uz8XeWoCoc9Jwdk\/N0Sh9AWGGSCJDAJOlWNooItA8Cjbb2hkTC9HyckDZQNxFoVwRIYNrV82w3EagQAnqESY\/OtJpAtLr+CrmQphIB7wiQwHiHlAqJABHwjUCjJUSTJTzfNkEfCUwIVKmTCJghQAJjhhOliAARiACBvF1ONrk5vptAAuMbUeojAuYIkMCYY0VJIkAEiAARIAJEIBIESGAicQTNIAJEgAgQASJABMwRIIExx4qSRIAIEAEiQASIQCQIeCMwH374oXj99dfF0qVLxcsvvyx69OghD5had911xeqrrx5Jc5sz47Of\/WxzBVmKCHhEoEuXLmKttdYSa6yxhsDPfIgAEQiHwAcffFCoPO+z999\/X8p\/9NFHAp+r3\/E3Jd+pU6dVfsZn+LsuF6JVCxcuDKG2pTqdCAxIyz\/\/+U9x2WWXiZtuukm8+eabqzRmnXXWEQcffLDAKZY4MXS11VZraYObqRwEJkXn22ARKwZl2hWirno6X3zxRflu\/fnPf7ZxVQfZjTbaqO7vOLzsgAMOkIRoxYoVAsfKd+vWTbz99tti8803F507d5aDK0gTdO2+++4CB6fhZ9iHf6oO9Tf1Oz477LDDxNVXX72KDfgMj9J5++23S11Fj65bL5st04wc6lTtKNKnbC2S0+3Os0F9ruOl\/qbkcVGjwkHJYTLEOIsvgRtssIFYtGiR9MenPvUpgTuQlixZIn327rvvyh1Ra665pnjvvffkl0n4TI2377zzjqwOeqBPtdmkY6GPqJN06\/koq8tG1sQOynyMQPadxu3r22+\/fQ0efK7e1y9\/+cuyP6BvPf7448lB2DSBwcvzs5\/9TPzlL38RW265pdhjjz3E1ltvLdZbbz2xySabiOeff1689tpr4oEHHhB33XWXePLJJ8U3vvENcfrpp1eOyISYuJLrSWyQNwQw8GO3zUMPPZSrEwMU\/uG969Wrl8AgpSa87ODmzaiAivh+BQS3Aqp1QqoTK0Xssk3I+7tOluqRWPWegKC\/9dZb8kv3ypUrJeHDP5C7esQr+1mVSNodd9xRgd5gZ6I1gUG4C98KZ8yYIY4++mgZXcFSUb0H3yJeeukl8bvf\/U6W\/cEPfiBvoa3KwwFWiH\/961+iT58+0bmsTLtC1JXVCdLy4x\/\/uAPOGJBB\/EFW8giKi122ZUPI8\/362N222Jb1MpZpV4i6fOi01aGTMpQtugleETY9aomlp0984hMyCIAIGubOV155Rbo7j0CpJahGZIoERggZVbnlllvE3nvvLbA8ZPuA8d54441ySakqDwdYDq6hJhh9YMQAdMQRR3QIBf\/yl7\/MJS36u2M7uLqUta3LRN5EpipjhYudseJQpl0h6vKh00WHbVlf8jqJSnUOs47AuLygVS2bqvOr6o8U7cZgg8iLGnRAXLA01A6P7YDdDpiwjUTAJwKpzmEkMAa9JFXnGzSdIiUhcMUVV4hLLrlE1nbsscfK5dl2eUhg2sXTbGerEEh1DnMiMNmr1es5B5nye+65p8x\/2XTTTVvlx6bqTdX5NmDEOsmUaVeIutT6uFo6wlr4VVddZeMap9wJ2zaFkLfVaQVOhYRjxaFMu0LU5UOniw7bsiHkU53DnAgMtl3+4x\/\/ENOmTZM7Jvbdd9\/adrsHH3xQ3HzzzXJ75je\/+U2Z7Y3fP\/nJT4qpU6fKnUhVeVJ1flXwbwc7d9ttN9nMkSNHir322qsdmlxro+2A3VbgsLFEwAMCqc5hTgQGu4twxsO1114rpkyZIrdP6w8OtDv11FPFIYccIg466CDxxhtviDPPPFMmJeL\/qjypOr8q+KduJ3Jf8AWgmehLCtiQwKTgRbYhZgRSncOcCAwIyYknnij22Wefwl1Fs2fPFn\/605\/EBRdcIKMv+B3bqX\/729+2xN84lAmH+qgHh3KpQ5qKDErV+S1xACvtgIC+8whJu0jebbeHBKbdPM72lo1AqnOYE4FROTDHHHOMOPDAA3N9gujM5ZdfLqZPny66d+8uT5o8\/\/zzZdSm7AfkZfjw4WLmzJmib9++Ivs7CUyxR2KdZMq0K0RdOGUXh9bhaXbnkYtdtmVDyNvqLHvcKKu+WHEo064QdfnQ6aLDtmwIeRKYnLcYRxSPGTNGPPPMM+K8886T9x9ll5BOPvlk0bt3bzF27Fh5vDEiMfPnz5ckpsxn+fLlYtSoUaJnz54yz0A9avLQ\/5a1K1Xn2+Bv+1LZ6HaRLdOuEHWdcMIJ8pRqPEjebeYkXRe7bMuGkLfV6dJfYi4bKw5l2hWiLh86XXTYlg0hn+oc5hSBwWAAMoJBGEcwH3roofJ4czyPPfaYXDxuByYAACAASURBVCbCqYK\/\/vWvZdLuf\/\/3f8sEXpCZsg+yU9EiEBV9yQhRGJAYFSHKG+BSdX7Mg3m72IbdR+p4c9vdR6lgZDtgp9JutoMIlIVAqnOYM4GBA3Av0i9+8Qt5Qq+6gRMXjuHuI0Q9sG0a+TJnnXWW+NKXviSOOuooeelYmc+CBQvEiBEjxKRJk+TykXpMlpFSdX6Z+LOuVRFg\/svHmJDA8O0gAmERSHUO80JgFPRYUsIFWXhwoy1uso3lIYFx80Ssk0yZdvmuS7\/3yGX7tItdtmVDyNvqdOvJ8ZaOFYcy7QpRlw+dLjpsy4aQJ4Gp895j+QgE4e6775aXUB122GGSvGCAxpLS2muv3fJRgwSm5S6gARkELr30Unm5KZ5mE3hTANV2wE6hzWwDESgTARKYArQRccEZMGog7t+\/v8wnwS2axx13nCQ0eQm+ZToPdbkSGN3e2267rWzzWV+CCGA5Ewm8G2ywgVzabNdn8eLF8qZtPkSACPhDYI899uigbOHChf6UR6LJeQkJ57pgR5EagJELAwKz3nrriblz58q8kwMOOECeFwNS06qHSbytQp71FiGgTt9t1\/NfFC6MwPAdIQJhEWAEJgdfRF9OO+00mZgLgnL\/\/fevsqNnxowZ4s4776wdZBfWTcXauY3aDflYJ5ky7fJZl57AC5L\/7W9\/u2kHudhlWzaEvK3OpoGKvGCsOJRpV4i6fOh00WFbNoQ8CUzOy6+iGiAxX\/\/61+XBcNktyffcc48455xz6m5TLmtcUafwqtN3TXYgwbZUnW+Du+1LZaPbRbZMu3zW5eMAOx8RDNs2hZC31enSX2IuGysOZdoVoi4fOl102JYNIZ\/qHOa0hKSuEth1113FscceuwqBwV1JOLjukUcekSQGO5Na\/fAqgVZ7gPUDARwvgIglnmYPsEsFSdsBO5V2sx1EoCwESGBykAZBwSF1uNsIB9QhIqMiMLj36NZbb5UD9Q9+8AP5r5U5MC4dJVXnu2DCsm4ItPsFjjp6JDBufYmliUAjBFKdw5wiMAANeTA4oO6GG26Qx6Bjbf9zn\/ucwE3Uy5Ytk4fZ\/fznPxcbbrhhI4yj\/TxV59sAHuskU6ZdvurS81\/23Xdfcfrpp9u4YhVZF7tsy4aQt9XpBFbEhWPFoUy7QtTlQ6eLDtuyIeRTncOcCQzGgxUrVshTeBGJwdkv7777rsDOikMOOUTsvffeUZwD4zJupep8F0xYtnkEfOa\/NG9FPCVtB+x4LKclRKAaCKQ6h3khMNVwYfNWpur85hFhyWYRQPQFy6wg+ohYtuv9R1xCarYHsRwRsEcg1TnMmsBgyejqq68Wr7\/+ujGK6667rjydN4YkXmOjNcFUnd8MFizjhoB+fcDBBx8sjx9o94cRmHbvAWx\/aARSncOsCYzaOj1v3jxjzNXpvN27dzcuE5Ngqs63wTjWSaZMu1zrQvQFybv67dOuOuFDFx22ZUPI2+q06bdVko0VhzLtClGXD50uOmzLhpBPdQ6zJjDZAeG1114T48aNE1\/4whfEoYceKtZZZx0pgoPjrrvuOnlK74QJE8TOO+9cpbGkg62pOt\/GIbYvlY1uF9ky7XKpC6Tl5ptvFrj\/CM9BBx0kTjrpJCfyoXBzscu2bAh5W50u\/SXmsrHiUKZdIeryodNFh23ZEPKpzmFOBOaDDz6Q6\/nYMp13VYA6B+app56SJKZr164xjx+FtqXq\/Eo6o4JG63kvMJ+5Lx2daDtgV7AL0GQi0FIEUp3DnAhM9iTePA9hdxLOiMH9SFxCamkfZuUlIwDi8oc\/\/EHgvjD1kLys6gQSmJI7JqtrOwRIYHJcru5C+vznPy9OOeUU0alTpw5SiNBgeQm3zcZyEm8zPTdV59tgEeskU6Zd9eoCWcGzaNEigasqkKybfUBefvnLX8oIjI\/lHx86bPELIW+r06bfVkk2VhzKtCtEXT50uuiwLRtCPtU5zCkCg8HhmmuuEWeeeaY47rjjxODBg2uDMwZ0fPO86KKLxJQpU+R5MFV9UnV+Vf3h225FPtRBjNnfP\/3pT8vrMJ588kmZ26U+x\/\/q53o2Qe\/IkSPl2Uh8GIFhHyACZSOQ6hzmTGAQZbnsssvEpEmTxPvvv9\/BL2ussYYYM2aMJDbZ6EzZDnSpb5ttthGnnnqq+PDDD8UnPvEJ2ZZ33nlHrLnmmvLQPuT6oK3AAjL4XbUXh\/x16dJF\/g1lILdy5Ur5D1crQJ\/6HRPlK6+8It577z2pG\/rwD7ogi7\/jf+hDvairc+fOtXpXX311+Tn04UE5lMej6snDwWQSdsGvUVmX+l3KNrKr2c9VhGX48OFik0026RBxaVZnyuVsv3GmjAXbRgRCIEAC0wBV7EZ68MEHxWOPPSYlt9xyS7HDDjvUdiWFcEpZOnfbbbeyqmI9FUBAERT8j3\/o67169ZKWM8pi70ASGHvMWIII2CBAAmODVmKyJDCrOlTP4wjlbpM6EIlCREo9ehlEndZee225+w1LP3369JE\/Qx4RLrVkhLL6z+r3rM4QE60PnS46bMuGkLfVGaq\/tVpvrDiUaVeIunzodNFhWzaEPAnMf95u7DxCYu7RRx8ttt56a6sbprGM8vDDD4tZs2aJ8ePHt3q8MK6\/X79+4u67715lwjNWkICg7UtVVpPLtCtEXT50uuiwLRtC3lZnWX2r7HpixaFMu0LU5UOniw7bsiHkSWD+8zaDhGAy\/8lPfiIAyve+9z2x\/fbb1z3jBbuV5syZI7dS45ZqlB04cGDZ40PT9aXq\/KYBYUEi4BEB2wHbY9VURQTaAoFU57Cmk3jffvttMXPmTHHxxRfL8Dy2UmdzXpYtWybuvfdeedro+uuvL4YNGyaOOuqoyt1Onarz2+LNZSOjR4AEJnoX0cCKI5DqHNY0gVH+BHkBScGW6fvuu0+8+eabNVeDtGy33XbyIscdd9yRJ\/FW+CWIdZIp064QdfnQ6aLDtmwIeVudFX6N6poeKw5l2hWiLh86XXTYlg0hTwJjOGpgay+2CyN5Elt6U3hSdX4KvmEbqo+A7YBd\/RazBUSgXARSncOcIzDluqE1taXq\/NagyVqJQEcESGDYI4hAWARSncNIYAz6TarON2g6RYhAcARIYIJDzAraHIFU5zASGIOOnarzDZpeE4l1kinTrhB1+dDposO2bAh5W502\/bZKsrHiUKZdIeryodNFh23ZEPKpzmEkMAYjXKrON2g6CYwGku3AYoKvD50uOmzLhpC31WmCaxVlYsWhTLtC1OVDp4sO27Ih5FOdw0hgDEa6VJ1v0HSKEIHgCNgO2MENYgVEIDEEUp3DvBEYHFa3YMECeXkgTujFMe64mLBHjx4ClwxW+UnV+VX2CW1PBwESmHR8yZbEiUCqc5gzgQFhwRkwY8eOlbdR9+\/fX564i5uQTzzxRNGzZ0\/x\/\/7f\/xPdunWL07MGVqXqfIOmcwmJS0gduokt2TCRN5Gx6atVlY0VhzLtClGXD50uOmzLhpBPdQ5zJjC33367OOWUU8SPf\/xjsfHGG4uLLrpIEphPfvKT4tprrxU\/+9nPxOmnny6OOeaYqo4r8sqEhQsXVtZ+Gk4EYkbAdsCOuS20jQjEiECqc5gTgcEpvKNGjZLLRCNHjhR\/\/\/vfxcSJEyWB6d69u8C9SRdccIF45JFHxDnnnFPZKEyqzo\/xRaNN7YcACUz7+ZwtLheBVOcwJwKDm6mHDh0qTjrpJLH77rvLqwR0AgMXIUJz\/vnn10hNs27DMtXo0aNrxY8\/\/nhJmvRH2TNv3jz55zwZ2HjkkUfWil155ZViwIABdc1K1fk2voh1kinTrhB1+dDposO2bAh5W502\/bZKsrHiUKZdIeryodNFh23ZEPKpzmFOBOa1114Txx13nDj44IPF4YcfnktgZsyYIe68804ZicGyUjMPyAtIEC6P7Nu3r1BEBcRDkRj1N9gxePDgmoz6HfWCvAwfPrymJ\/t7kW2pOt\/GF7YvlY1uF9ky7QpRlw+dLjpsy4aQt9Xp0l9iLhsrDmXaFaIuHzpddNiWDSGf6hzmRGCQwIuIC5aIzjvvPJknokdgnnjiCZnIi+gMiAYSe20ftUy10047SWKiniz5AMmZNWtWh0iPHhHq2rWrXO5CUrEeuYG9eLLRHN3OVJ1v6wvKx4\/Av\/\/9b\/GpT31KPPPMM9JY\/IwHf8c7sHjxYvH666+LFStWyKR7LPPis7yn6O+usvGjGMZC5Quf2l11brjhhrIf4MH4jP6gfl5jjTVEly5d5N\/WXHNN2Zdw190666wjZXHnXbaP6f1N\/1nJqT6VlcPv+mdZOZ+YtaOuVOcwJwKDjvDyyy+Lk08+WQ6Kn\/vc58Tf\/vY3sf\/++4unn35a3HPPPaJ3795i2rRpok+fPl77DbZsjxgxQkyaNElGZfKIiIrKgJz069dPLnfhZ33JKG\/ZK2toqs736hAqKwUBEBNMKPjSgAgoBnobolGKkayECESEQD2SV\/TZeuutVyNy+NILwoZ\/yO0EscN814i8ZUlZKyFJdQ5zJjBwytKlS8XUqVPF73\/\/e\/Hmm29KP8HJBx54oMyP2WSTTbz7To+4qOhKNkqjLyttu+22HQiPMshkGSlV59s4xTasaaPbRbZMu0LUVU+nIifI6XrooYdcYGpYttE3efU5IqJ430wfnAeFpWP1zR431Gd\/xhlSkMGYgcgQzo3CZPHee+\/J86TwO6IE+BtkoAN2ICqgfsbfIYfy+Bmy+Azl8D8+e\/fddzvow98hp2RU5EFFJFAGEYe8R2+HLwJZD1vXOlzLm\/qbcm4I4D3DyoZarci+l\/gdnyEyhp\/xD5Gwz3zmM\/KLTFFka5dddhEPP\/ywm3ERlvZCYFS7MDCBwKxcuVKsu+66wQ6wy+bAFC0zkcD463EhJm8f1pVpV4i68nRiIEKk5a677qobXVGDFSbyLbbYQob2N9988w6heB1j9Y1QHxRt2xRC3lanj34To44YcUBfRKRPRSRslojyloR03LOfQ\/eTTz4pNtpoo7ruwZliL730kjwoFRM5xv833nijVia7LJolhj6WTGPsP41sOuussxqJVO5zLwQGjBFRGKyngh0+\/vjjMhrTuXNnGYXB8o2vR5GV5557rpbvUgaB0e2\/7bbbfDWHeohABwQQjbjlllsE\/tcfDNpf+9rX5J\/WXnvtyh5JkOdu5OX06tWLPYEIRImAehfxDub9rBv99ttv195R\/Ix3Ff+jHMojogeyhQgeIidKPvu+Q4n6DD\/nfW4LFglMDmL41nDaaadJ8oKzXsCKsTPpqaeektL4VnjppZeKbbbZxhbvVeTzyAuEyiAwPMjO2X1U0AABfDO87LLLOkRQ9ttvP+\/5Y7E5IsbIQ2wY0R4ioCOQXS7KoqM+V\/\/vvffeYs6cOcmB6BSBUbuQcNbLhAkTxHbbbSd+85vfiAsvvFBGR5DAi7NbwEJx1QAiMvWe7FkvgwYNknqx5l5EXpQ+JvGG7ZuxTjJl2hWiLl0nyIu+e+i73\/1ubU27nndd7LItG0LeVmfYnt467bHiUKZdIeryodNFh23ZEPKp5nE6ERiEwrBNGuwO560okoEhQBEPXCdw+eWXOx1k14i8oD5uow478Nq+VGGt+T\/tZdoVoi6lE8QFBAYPcgFwPYfp42KXbdkQ8rY6TXGpmlysOJRpV4i6fOh00WFbNoQ8CUzOaJA9iXfRokViyJAh8qTbY489Vpb405\/+JC655BInAoPoCnYLqSsK8gambGJv9mA7lFGn8KrTd012IKFcqs6v2gCfqr360hHIC44hQPSyXR7bAbtdcGE7iYAvBFKdw5wiMCoygvNfEIm54YYbxBlnnCEjLsh5wRLTuHHjZIIvIjJYSrJ9cN4LSNGSJUtyi+pXAfAqAVt0KR8DAiAw5557rjQF2x132223GMwqzQYSmNKgZkVtigAJTIHjr7nmGnHmmWfKfejY2rbjjjuKyZMnS2kcMofTcc8+++wOp+hWrQ+l6nwbP8Q6yZRpV4i6oPPuu++u5b4g78U2+uJil23ZEPK2Om36bZVkY8WhTLtC1OVDp4sO27Ih5FOdw5wiMBgcEGW56aabBCIh22+\/vTjmmGPkjiREZ8aMGSPPp\/jOd74jD52q6pOq8238YftS2eh2kS3TrhB14URdfAnAA+ICAmP7uNhlWzaEvK1OW3yqIh8rDmXaFaIuHzpddNiWDSGf6hzmTGCqMji42Jmq810wYVk\/COjLRwMHDhS77rqrH8UV0mI7YFeoaTSVCESBQKpzmDcCg4N2cPR39kGEZv78+eLLX\/5yZQ\/fStX5UbxZbW4EbmrHibt4sPOo0ZH+KcJFApOiV9mmmBBIdQ5zJjC4zBGJuzgLpujp37+\/0y6kVneEVJ1vg2usk0yZdoWo69e\/\/rXMHbPdOq37zsUu27Ih5G112vTbKsnGikOZdoWoy4dOFx22ZUPIpzqHOREYdZAddh1hpxCIykUXXSR22GEHeUM01vZxsdr48eMFbvTE8clVfFJ1vo0vbF8qG90usmXa5bsuffmo2fwXYOdil23ZEPK2Ol36S8xlY8WhTLtC1OVDp4sO27Ih5FOdw5wIjDrIDoQF26jxgKzgBtmRI0fKix1PP\/10sccee3AXUswjJ21rCQLMf\/kYdtsBuyXOYqVEoMIIkMDkOC97kB1EcCIu7lxQJ\/FiG\/W9995b+72KfSBV51fRFynZrOe\/NLN9OhUsSGBS8STbESsCqc5hXiIw++yzTy3CgtNtp0yZIpeScAU7cmPOP\/985sDE2rMN7Yp1kinTLt916XcfudwU62KXbdkQ8rY6Dbts5cRixaFMu0LU5UOniw7bsiHkSWByhgOVA4MrBLB0BMKCk3OxmwK\/b7311uKCCy6QB3UpQlO5UYVXCUQd5rd92V36n++6cPquui3W5u6jbBtc7LItG0LeVqeLD2MuGysOZdoVoi4fOl102JYNIU8CU\/DmY4v0CSecIA+vmzp1qujSpYsYNWqUXEbaYIMNxNNPPy2OP\/54SWo6deoU8\/hRaFuqzq+kMxIx2lcCbwpw2A7YKbSZbSACZSKQ6hzmtISkHLB48WKBE0WRrNu5c2fx4osvClzAiOWjgw8+WJx00kkyOlPVJ1XnV9UfKdit3z7drgfYKT+SwKTQo9mGmBFIdQ7zQmBidpwP21J1vg02sU4yZdrls65HH31U\/P73v5cucE3gdbHLtmwIeVudNv22SrKx4lCmXSHq8qHTRYdt2RDyqc5hXgjM888\/L5eMEInJe9Zdd11x2GGH8STeKo2mGVttX6qymlqmXT7rUgm83bp1k0cNuDwudtmWDSFvq9MFq5jLxopDmXaFqMuHThcdtmVDyJPAFLz5f\/3rX8WPfvQjeeZL0cOTeGMeNmlb2Qgw\/6Uj4rYDdtn+Yn1EoOoIkMDkeBA3TiNhd8mSJTLnpU+fPpU9bbdeB03V+VV\/Katqv57\/sv\/++8t7wtr5IYFpZ++z7WUgkOoc5rSEpA6yO+aYY8SBBx5Yhh9aUkeqzrcBM9ZJpky7fNWllo9w\/9F+++0nib\/L42KXbdkQ8rY6XbCKuWysOJRpV4i6fOh00WFbNoR8qnOYE4HBDdSnnXZa5a8KaDSoper8Ru3WP7d9qWx0u8iWaZePurLLR7vssgsJzL\/+5YyBSx+KpayP\/hWiLWXaFaIuHzpddNiWDSGf6hzmRGDwsuDCxt\/97nfiV7\/6lejRo0eI96flOlN1fsuBbUMD9NN3XXcfpQKf7YCdSrvZDiJQFgKpzmHWBAZRl6uvvlq8\/vrrEvv33ntP3HLLLeLdd98V3\/rWt8Q666yzik+4C6msbsp6YkbgoYceEn\/84x+liVg+cjl9N+Z22tpGAmOLGOWJgB0CJDD\/wUvlvcybN88YQe5CMoYqWsFYJ5ky7Wq2Liwb4R+iL1ny0qxOvaO46LAtG0LeVme0L4mjYbHiUKZdIeryodNFh23ZEPIkMI4vZ5WLp+p8G5\/YvlQ2ul1ky7SrmbpAXBB1wc4j9eg7j5rRmcXLRYdt2RDytjpd+kvMZWPFoUy7QtTlQ6eLDtuyIeRTncOsl5D0AQDLR1hSwvUBOJAr1SdV56fqr1a2S0VbEKHEkpH+YNkI5KV3796tNDG6um0H7OgaQIOIQOQIpDqHNUVgcAv15ZdfLnCbrjrA7hvf+IY466yzxCabbBK5K+3NS9X59ki0Zwl1YzSiKCAheF577TV58\/oLL7wgf9cjLHkobbPNNgI7jlT59kQyv9UkMOwNRCAsAqnOYU0RmBtvvFGceuqpYssttxQ777yzWLZsmbzXZbvttpOkBjdTp\/TA+XPnzpVNwgSkvmWryUhNcPgcPys59XvZco1s0Nthaisu6Nxiiy1k27Lt1dtnqg9ySO5ebbXVJBlQETwkh+NGc+RaffjhhwJkGf9WrlwpE8bx4PcVK1bI283R91AWej766KMa\/kX9T2Gjf573Nx\/91yTi4mPydtFhWzaEvK1OH76JUUesOJRpV4i6fOh00WFbNoQ8Ccx\/3nhMHGeccYaMvEyZMqW26wg3T+NKgZkzZ4oBAwbEOD40bdNPf\/rTpsuyYNoIKPKG\/3v27Cn69u0rSa5ppMV2sMpD00WHbdkQ8rY6U+1RseJQpl0h6vKh00WHbdkQ8iQw\/xk11C4kkJSRI0fWxhKE0ocOHSpv1h08eHBSYwwJTPXc2YhA5H2+3nrrySgOIjr4GUcCrLHGGrWomx5daqS\/eoi1zmLbAbt1lrJmIlBNBEhgGhCYImITwt333XefGD58uIz24BuverJbvI8\/\/vgOJAtyKHvkkUfWylx55ZUNI0Y77bSTgJyawNQSSr2lB7XUpP7Xy+qY2MohAVRfIjJZ\/tBtqOePZuTqlcm2HXXn2d5oiS27JBaiT1Fn6xAggWkd9qy5PRAggYmEwCiSsnTp0g4ERv398MMPlxGg7O+KvOjEp4gIZbt0qs63eXVjnWTKtCtEXT50uuiwLRtC3lanTb+tkmysOJRpV4i6fOh00WFbNoR8qnOYdRJvUaSlrAgMbr2eNm2azDfQIzCzZ88Ws2bNEtOnTxfdu3eX4xYICuTxt65du8qbs1FOX\/rC53j0v5HArDrs275UZU0cZdoVoi4fOl102JYNIW+rs6y+VXY9seJQpl0h6vKh00WHbdkQ8iQwEURgFCEZNmyYGDduXAcCk0dEFKkCOenXr5\/M0cHPepKxTnIU8SGBKXvoZn3tjECqg2s7+5RtjwuBVN+xpiMwZV8loC8JbbbZZh1yYJYvXy6jK8hV0ROI9TLbbrutGDFihJg0aVKHvBmTZaRUnR\/XK0Zr2hUBvl\/t6nm2uywEUn3HrAlM9jJHEwf4uMwRS0Rz5swREyZMECBPei4LCYyJF9xkYn0ByrQrRF0+dLrosC0bQt5Wp1tPjrd0rDiUaVeIunzodNFhWzaEvK3OeN+SjpZZE5hWNAwnnurRk2zUpAwC04p2s04iQASIABEgAj4QWLhwoQ81UemInsDkkZOyCUxUHqMxRIAIEAEiQASIgIiKwGCZaPTo0TW3DBo0SHz\/+98XJ5xwgliyZEmuu8aPHy\/zXkIm8bKfEAEiQASIABEgAnEhEBWBMYUmL\/E25DZqU7soRwSIABEgAkSACJSDQDIEJnsOTdFBdjiFV52+a7IDqRw3sBYiQASIABEgAkTABoFkCAwaHeoqARtAKUsEiAARIAJEgAiER6CSBCY8LKyBCBABIkAEiAARiBkBEpiYvUPbiAARIAJEgAgQgVwESGDYMYgAESACRIAIEIHKIUAC48FlyL05\/fTTxRlnnNHhmgIPqqmCCLQ1Atm8NnVsQluDwsYTAY8IqLPWrr\/+eqlVbXLxWEUwVSQwjtDilOAhQ4ZILfrt2I5qWZwIEAEh5PlOvXv3lmc9qXdt8uTJHS5kJVBEgAg0jwCOIHnmmWfkRccmlxs3X5P\/kiQwQsiBEbdbT5kyRWRvpIZDsfVaPTo7xbfDiy++WOy1117iJz\/5ySoXRfp3FzUSgWoi0Ow7prdWfVM84ogjSGCq2Q1odUAEfLxjmO+uuuoqeedg165dA1rrR3XbExgVogac06dP70BgsufEFJ0bk72ryY9rqIUIpIGAj3cMSNQboNNAiq0gAs0h4PqO6ctIXEJqzgell9KjK\/379+9AYJRDe\/bsKUNr6sm7soAEpnTXscKKIODrHWOeWUUcTjNLR8DXOwbDFRHCnDdgwIDS22JbYdtGYJTTkRSIZ9asWR0ITJEj89YISWBsux3l2wEBX+8YIy\/t0FvYxmYQ8PWOqbrzLk9uxq6yyrQtgdEBzrtHqYiU5C0jkcCU1V1ZT1URaPYdQ3svu+wyceaZZ1ZiTb6q\/qHd1Ueg2Xds7ty5svEqUX7EiBGVyeckgRFCNOv4vn37SseTwFT\/5WcLwiLQzDs2depUcckllwi1vVNZWKU1+rCoUjsR+D8EmnnHsHO2V69eYtSoUbX3rErvFwmMBwLDl4gIEIH6CDQ7uKovCcSXCBABvmNZBEhgSGA4LhCB4AiQwASHmBW0OQLt+I6RwBQQGJsk3jZ\/b9h8ItAQgbzBle9YQ9goQASMEWjHd4wEpoDA2GyjNu5hFCQCbYpA3uDKd6xNOwObHQSBdnzHSGAKCAx6mNqippKaig6yC9IbqZQIJIRA3uDKdywhB7MpLUegHd8xEpg6BEYfYFXvrFKGdsvfKBpABP6DQNHgyneMXYQI+EGgHd8xEhg\/fYdaiAARIAJEgAgQgRIRIIEpEWxWRQSIABEgAkSACPhBgATGD47UQgSIABEgAkSACJSIAAlMiWCzKiJABIgAESACRMAPAiQwfnCkFiJABIgAESACRKBEBEhgSgSbVREBIkAEiAARIAJ+ECCB8YMjtRABIkAEiAARIAIlIkACUyLYrIoIEAEiQASIABHwgwAJjB8cqYUIEAEiQASIABEoEQESmBLBZlVEIGUE7lRpmwAAA3xJREFUPvroI3HjjTeKc845R\/zrX\/8Se++9t5g4caJYe+21xQsvvCD+67\/+S5x11lkSgiFDhoiTTjpJDB48eBVI1B1J+GDChAmia9euKcPGthEBItAkAiQwTQLHYkSACHREYOnSpeL73\/++2GCDDcRRRx0l1l9\/fbHllluKTp06ibvuuktcccUVktyAzJDAsPcQASLgigAJjCuCLE8EiIBEYMGCBbnEBJGZX\/ziF2KttdYSJ554YqGcgpERGHYoIkAETBAggTFBiTJEgAjURQAXyY0ePbqDjLr49I033pDE5Yc\/\/KEYMGCANYG57rrrVtGtKurZs6eYOXOm6Nu3Lz1EBIhAmyFAAtNmDmdziUAIBBYtWiTmzJkjJk+eLA488ECxyy67iC222EIuJz366KPi7LPPFr\/61a\/ExhtvbE1gXnnlFfHss892MBvLVciv+drXviZ++tOfim7duoVoFnUSASIQMQIkMBE7h6YRgSohULSEhOjMgw8+KMaOHSs6d+5cIzBLliyp27xBgwblJvG+9dZbMhn4+eefF+edd57o0aNHlWCirUSACHhCgATGE5BUQwTaHYE8AvPee++JMWPGiG222aa240jJfeELXxBf\/OIXV4Ht\/fffF3\/5y18EPs\/uQvrggw\/E+eefL2bNmiWmT58uttpqq3aHne0nAm2LAAlM27qeDScCfhHIIzBYWho+fLj42c9+VstTKYrUKGuKkniRDPy73\/1OEiJEcw455BCx2mqr+W0EtREBIlAZBEhgKuMqGkoE4kYgj5jcfvvtMlqC7dMqT6VZAjN\/\/nxxwgkniH333VeeIYPt2XyIABFoXwRIYNrX92w5EfCKQJaYZLdPq8qaITAvv\/yyOPnkk8WGG24oxo8fz6Rdr56jMiJQTQRIYKrpN1pNBKJDIEtMstunmyUwK1eulEm7Dz30kJg6daro169fdG2nQUSACJSPAAlM+ZizRiKQJAJZApPdPt0MgUHuzIwZM8S5554rfvSjH4mvfvWrq2C32Wabic985jNJYspGEQEiUIwACQx7BxEgAl4QyBKY7PbpZgiMSti9\/vrrC23EklLenUpeGkUlRIAIRIsACUy0rqFhRIAIEAEiQASIQBECJDDsG0SACBABIkAEiEDlECCBqZzLaDARIAJEgAgQASJAAsM+QASIABEgAkSACFQOgf8PEVUT+vwDWxgAAAAASUVORK5CYII=","height":337,"width":560}}
%---
%[output:496c28ac]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Ares_nom","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.003141592653590"]]}}
%---
%[output:04a211a4]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Aresd_nom","rows":2,"type":"double","value":[["1.000000000000000","0.000100000000000"],["-9.869604401089360","0.996858407346410"]]}}
%---
%[output:42b17330]
%   data: {"dataType":"textualVariable","outputData":{"name":"a11d","value":"     1"}}
%---
%[output:0ebfe05b]
%   data: {"dataType":"textualVariable","outputData":{"name":"a12d","value":"     1.000000000000000e-04"}}
%---
%[output:08f2e94a]
%   data: {"dataType":"textualVariable","outputData":{"name":"a21d","value":"  -9.869604401089360"}}
%---
%[output:13a8d493]
%   data: {"dataType":"textualVariable","outputData":{"name":"a22d","value":"   0.996858407346410"}}
%---
%[output:237b32fa]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ2MVtd5558kbjJjYgfGsWJP+BjMx5ZKLXUimmFgTbzEwU0Ci1C7MKwlSihlExNr5UEMjCNj3BhmUIgUjGMRPDtC68yYJLIcZiuF2jhLy8dsvbVN1RQHMB4wwa4aD8QJgbbOevVcfF6fudz3fc+95557n3Pv\/0pRzMz5\/P3PPfc\/5\/MD77777ruEBwRAAARAAARAAAQ8IvABGBiP1EJRQQAEQAAEQAAEAgIwMGgIIAACIAACIAAC3hGAgfFOMhQYBEAABEAABEAABgZtAARAAARAAARAwDsCMDDeSYYCgwAIgAAIgAAIwMCgDYAACIAACIAACHhHAAbGO8lQYBAAARAAARAAARgYtAEQAAEQAAEQAAHvCMDAeCcZCgwCIAACIAACIAADgzYAAiAAAiAAAiDgHQEYGO8kQ4FBAARAAARAAARgYNAGQAAEQAAEQAAEvCMAA+OdZCgwCIAACIAACIAADAzaAAiAAAiAAAiAgHcEYGC8kwwFBgEQAAEQAAEQgIFBGwABEAABEAABEPCOAAyMd5KhwCCQHoGTJ0\/SypUr6fz585VEFy5cSN3d3dTY2Fg3o8uXL9OGDRuovb2dWltb64YfGhqi5cuXjwq3Zs0a6uzspLhp1c0MAUAABApNAAam0PKiciBQmwAbmPXr19O2bdto2rRpsU1EXNPBBqanp4d6e3upqampkl9zc3NgYvCAAAiAgCkBGBhTUggHAgUkUM\/A6CMmaqSEMbAJ2bVrF82bNy+gwr\/jEZi9e\/fSxo0bKz8Lm5KwgeGAXIYtW7bQN77xjcBI8WjOzJkzg5GdwcHBIC01KsT\/rX7OP3v77bepq6uLXnzxRTpy5AidPXuWli1bRpMmTRo10tPf30\/Tp0+njo4OuuOOO+gv\/\/IviU3TN7\/5zaAux44do61bt9LSpUsLqDKqBALFJAADU0xdUSsQMCIQNYWkPuS6uRk\/fnxgHNra2gJzoEZR3nrrrWAKio0APwMDA8H0kzIa4amlKAMzMjISGIv777+fnnjiicDATJgwIUjjk5\/8JKnf60aF82DTsW7dOurr6wsMzFNPPVUZ2eF81JQWm6rh4WFavXo1rVq1Kvg5GyuuA4fj0aBnn302MECmU2dGcBEIBEDAKQEYGKd4kTgIyCZQbQRGGRVlSHg9jDICqkbhdStnzpypjL6oMPqoDf\/M1MCwyVDTUzwKw6Mljz\/+eGBwuGw8UlLN2Ki1O+HRI2VguNxqtIiNDf+bw+p1la0aSgcCIMAEYGDQDkCgxATCBoZRKKPC00NxDYwyBNWQmk4hcXxe7KtP\/agRmnoGRo3+8P\/ziMq+fftGjcDAwJS4waPqhSIAA1MoOVEZEIhHoNYIzKc+9anKAl\/TKSQ1paOH19eV1FrEe99991V2NHEtlHkKTxWpqZ5qP1cGRl9LwyM4GIGJ1zYQGgSkE4CBka4QygcCDgnU20btYhGvyTZqXnDL61XYpHD4X\/3qV9cs7o1axKvWsKjFxGxcOJ2XX345MGNr164NpowwheSwUSFpEMiIAAxMRqCRDQiAQLoEoqaj0s0BqYEACEgmAAMjWR2UDQRAYBQBfZs2\/4LXyJgcoAeMIAACxSMAA1M8TVEjEAABEAABECg8ARiYwkv8fgXDx7jzYWHqRNQ8MKjD0DjvqO22aq1EvXLq6fCaBz4XhE+VrffwNlxeH6HWROjpqLj18uZwzPXgwYPWJ8mG9YlTl3p11de6hK8K0POtlycz4id8QF21n4fLxeF4d1OSUROlF6+L0Z84Vx\/U42T6+2pl0duLzrweV5Vv3O3caqu52pllWn7TcGm1bT2\/qLaij6yZ6lmPVdQOOy5HtZ+HmXC4PXv20AMPPGB0rYYpU4RLjwAMTHosRafEHcSjjz466uMe9bOsKsF5q90k3AmzkeATVHnHiur4+XA0tdiy2iFjejrqrBL9QLNa9YkyMOGPM3e2fE9QtQPOOI3NmzfTpk2bgqPxaz31Ok61fTntI\/X1j9yiRYsqB9Ip1vpVAlyGWvxsDUwcXmGWYb3496pueV9FEDYS4bLW46qMMJt20w+4Xn8XBiaOVvXatm7QuI76Hyz6WiZ+h\/nAxHp6KtNdi5WtgeEys2784ITmrL4M8fKBgYnHy8vQYUOgKuH6r7dasMIfQv3fNh9y046Uy2ZiYOotFI3TwdUrGzNoaWmx6iyj0gjnW+svV5Myhk0e\/9t0BMbmoxBlYNSHX79fKY+XNGykwxxNuPKBfZ\/97GeDHVemJwK7fIfTbNvKbD3yyCP005\/+tHIasmoP+h8oYZZhPVlrE1ZpGJg4Ji6Pdlf2PGFgStAC6nUICoF64fmvdL4rhh8+Vp4fdb9NtftiwosrVZrhqSH186gRGB55UHfgJP2Lst6HQp8u4b\/e+O4cfQop\/HGu9WHmjwd3yCtWrKhMWemnx3JaakqB\/1ttC46aTjD5EJkMs0cZmLAJq2XK6rUVkxGYqKm48F\/cSU69NTEwfLUBjyhl0YZVW44qV9QITK2rCtTVB\/X4c5761JS6i+ruu++uGF\/Tqdlw+9e7wjTbtv7O83\/zYYf6exY1AlPr\/TdlZWJgbrrppsp7qddfLQ6Pe1lpCT4noqoIAyNKDjeFMf3rWHWM3LGp49V5uFeZFpNh8Dg1UGZC\/6CrDmPBggW0e\/fu4BwQk3UoKt9aUz5RHxU2Zqqzivrw1hui5ksIt2\/fXpk+0kePwqaklrmKWlOhm8Ww6ahlJMKjOOERF3V5ol5u\/aNYa2dPFKNaZjWqzvoVACZrlWoZhfAUUtZtOGoEQZVXN7PVjHz4falnYMJtSr1D+juqjBKnre6vUlcv6Gu+lCGO0juqjSRt21wOfSSD3+vwHwqm7U\/nVY9V1BlHKn61NUlRadqMCMfpDxE2PgEYmPjMvIsR\/tiFX2z1kT537lzl5FX+sERNPaQ1VK8bDb2jVWs0eGRELTCutw5F\/wsvvM5HFytsAkymkGqZtnqLCFX6+toefb2JXrbwNF\/43+GRFf0Dc+LEiVE3L6t0lfHj3+sjHlEfJxWn2nSjbhDDHx\/+d5ShqjZiUu3n9V4sk0W89aZu6k0J1itD+PfVRs7CHE3zNfko620oan2TfoGm3kb5jwH9\/a2lQ5ptW7UPtXg73FbC75jpH1xxWeltPOo9rKZRvXzithmET48ADEx6LMWmVOsF1H9nY2DiTCFFdZyq89i5cydt27atcmEfQ603LcRhOP9a5kWF0Yfxw8PDcT7CnF5UJx\/e0XPzzTePWpxczcBENR5VHnXEPs\/760\/UX5G2U0jVzEhcA1NrSizpsLyJ8bExMHHasOJRbTQp\/M6ZTBFGtdFwu4j6yCrN77rrrsjpEGVkwzduxzUwSds2a6Lv5tHfsyguJu+7CSuTKSQ1Algrz3pmTmzHX4KCwcCUQORaf1WnZWDiYKxlYHjUhYeY9WmQWiMGqiMz2XmUZAQmTicf7ozjTCFVMzCKg+kC32qLePWprlodcr0PrckamHomyMSIRPEwiWdjYOK0YRXW9K\/2elxVevX+2g\/Xr94IjF6neu0\/HFYftbNp29WMIY\/8Pvjgg\/Twww\/H\/oMlTQNTr13V0yRJu0GcdAjAwKTDUXwqUSMU4a2INiMwcQFETSGprZOm6z3UKIi6QLDeeorwlI6qv74GhtPUtzHXmkIKG6vweozw+oR6f+Xpw\/scV69XmEm1ckUZmDjbqMP5hnU1MTD1Ovw018CEy5e1gam2PiJqCsmkndZjV23Nz9e+9rVgEW84vv6eqeMK9Lug9DVgOss023a9NhQ1hVTr6IKkZk\/F09tItUtH9TJjDUzc3j278DAw2bHOPaeoRW36Ar4sO\/\/wbp1aB9npC2mjpn127dp1DdtqC1F1BmoHB+fN581ELVCtdQBZ1E4NfZid01UPfzT0OkeVT4\/L8cJh9L9kTQ9G0zvtlStXBmfa1DrILipfHW49A6NufK512Jw+AsRp80JTfd1GtRel3l\/KHC\/LNsz51VqvEZ5yUQcs1ppCq2dgOE99LRBPD02cOHHUCIbejsML4PUy3X\/\/\/XTgwIHKLjyde9ptu14b0stc630PG4taO7tMppB411r4clHOQy2KTjrdmXtnX5ICwMCURGhU0w0B\/uDwg4OuzPmGR4n4o8pPktN5zXNFyDCBeiNhaNtXDXF4pyFakhwCMDBytEBJPCSAg67iiRbFixd48l\/d9U4yjpcTQocJhD\/GtaZH1UiP6SnTRaUNEydbWRgY2fqgdB4Q4BGENO5C8qCq1kXk0ZekdyFZZ44ERk2TmkxDlrlth3dPofnIIwADI08TlAgEQAAEQAAEQKAOARgYNBEQAAEQAAEQAAHvCMDAeCcZCgwCIAACIAACIAADgzYAAiAAAiAAAiDgHQEYGO8kQ4FBAARAAARAAARgYNAGQAAEQAAEQAAEvCMAA+OdZCgwCIAACIAACIAADAzaAAiAAAiAAAiAgHcEYGC8kwwFBgEQAAEQAAEQgIFBGwABEAABEAABEPCOAAyMd5KhwCAAAiAAAiAAAjAwaAMgAAIgAAIgAALeEYCB8U4yFBgEQAAEQAAEQAAGBm0ABEAABEAABEDAOwIwMN5JhgKDAAiAAAiAAAjAwKANgAAIgAAIgAAIeEcABsY7yVBgEAABEAABEAABGBi0ARAAARAAARAAAe8IwMB4JxkKDAIgAAIgAAIgAAODNgACIAACIAACIOAdARgY7yRDgUEABEAABEAABGBg0AZAAARAAARAAAS8IwADYyDZbbfdZhAKQUAABEAABIpEYPbs2dTQ0EBHjx6lK1eueF2106dPe13+qMLDwBhIygYmbfFdpMlV8Sldn8rqii0Y+NVmXbUDV+mifdm1r2eeeYYuXbpEixcvpjFjxoz6WvjE1lVZDT6fToPAwBjgLar4BlV3GgRcneJ1kjg0c4LVWaLQyw5tLQNjl3L12C40c5Gmq\/rHSRcGxoBWUcU3qLrTIODqFK+TxKGZE6zOEoVedmhhYOz4uY4NA2NAGJ2AAaQEQV577TWaPHlygpiIkhcBaJYX+WT5StSr6f6f0GPtM6h91i3JKpVhrDwMjAvNivoNg4F572UYGhqi5cuXB\/\/aunUrLV26tPKaFFX8DPuByKxcvKh516no+UMzvxSWqBcbmM4Fk6lzQYt4mDAwsiWCgSGikZER6ujooK6urkCtLVu20Pbt26mpqSn4NwyMm0YssXN1U9PipArN\/NJSol4wMLXbkAvNivoNg4EhIh596enpod7eXmpsbKQNGzZQe3s7tba2wsA47K9dvKgOi4ukiQia+dUMJOrFBoanj3gaSfqDERjZCsHAvGdgBgYGqLu7O1CLDUxbW1tlGqmo7jXvpimxc82bifT8oZl0hUaXT6JeMDAYgUnrLYKBMTQwd955Z4W5GpnRRfjc5z4X\/PO5556r\/DjqZ9WEK2N8PhiKD4nip4z197H96JqlXf6odyf8TkW9P7XaTtnjnzt3jl555ZWa34ss+f39L2+g75\/\/BN12\/WVaM+nndd\/9vPU7fPgwvf766zRu3Dj60Ic+FMnRhB+nMWHCBKP4\/I596UtfqoTV34s47X\/+\/PlBcP52\/eQnP0n9LLO0TIhNOjAwmEKyaT9WcSX+dWhVoRJEhmZ+iSxNr579w9Sz\/zWaO2Us7bv3dvEw05pCevDBB+nhhx82qq8LzYo6iwADg0W8Ri+Vi0AuXlQX5USa7xOAZn61Bml6KQMzsamBXv76bPEwYWBkSwQD854++jbq\/v7+ygJe\/nVR3WveTVNa55o3Dx\/yh2Y+qCTXcN47cJxeH7lCh169SCPfen9aXipVGBipylwtFwyMgT4wMAaQEgTBxzABtJyjQLOcBYiZvTS9Fj32UlADGJjqQrrQrKjfMBgYgw6hqOIbVN1pEBcvqtMCI3Fso\/asDUh7x9jAzJk6LlgHgxGY6MbkQrOifsPEG5i9e\/fSxo0bRykdPinXdZ9SVPFdc6uXvosXtV6e+L0dAWhmxy\/r2NL04i3UbFz4\/3kNDK+FkfxgCkmyOoKnkNSalCizokxNeK2KK9QwMG7ISutc3dSyWKlCM7\/0lKaXbmB8uA8JBkZ2exc5AsNH+w8ODtKKFStq0tuzZ0\/dMGngh4FJg+K1aUjrXN3UslipQjO\/9JSmlzIwf\/iNo9Q+61bx9yHBwMhu7yINjDRkMDBuFJHWubqpZbFShWZ+6SlJL95CPfDCG8HUEa+FmdDUIP46gbQMTJxW40Kzon7DxBqYkydP0sqVK+n8+fPB7dD88FqY5uZm6uvro2nTpsVpE1Zhiyq+FZQUIrt4UVMoFpKoQQCa+dU8JOnFBubwqQvBAXZqO7X0w+xgYGS3d5EG5vLly5ULFadPn06rVq2iZcuWBXcT8fqXI0eOBPcW8cWLWTwwMG4oS+pc3dSweKlCM780laSX2oHUuaAlgKimkyQTTcvA4CReNyqLNDC8Bmbz5s20adMmampqCm6KnjdvXnC4XPh3brCMThUGxg1lSZ2rmxoWL1Vo5pemkvRiw7Lvq7fT3KljYWAyHuUs6jcMBsagPyqq+AZVdxpEUufqtKIFShya+SWmFL0OnbpIa586Pur6ADY00nciYQRGdnuHgTHQBwbGAFKCIFI61wRFL20UaOaX9FL0Uifw6mtefFjIm5aBidNqXGhW1G+YWAPD616OHTsWqfvMmTOpt7c3mF7K4imq+Fmwq5WHixc17zoVPX9o5pfCUvQKTx8pitLXwcDAyG7vIg2MNGQwMG4UkdK5uqldMVOFZn7pKkUvPvcl6vZp6SfypmVgsIjXzXsj0sDwQl2MwLgRXFKqUjpXSUyklwWaSVdodPkk6MUmpX3WLZFnvrCxmTNlrNjzYGBgZLd3kQZGR8bbpoeHh6mzszP4Mf+bH95SndWDERg3pCV0rm5qVtxUoZlf2uatl354XRQ5\/r3kix1hYGS3d9EGJmrLNLZRy25QcUqXd+cap6wIe5UANPOrJeStl8lOI8nrYGBgZLd30QZGHWjX1tZWGXHhM2H4dF4cZCe7YZmULu\/O1aSMCCNvSgKamBPI6x07O3KFFn3nJZo4riE4ebfWI3k3EgyMeVvLI6RoA8NAwuthst6BxGXAFJKbpplX5+qmNuVIFZr5pXNeevFVAYdfvRgcXDexqaEmtIEX3gyuFhj51p3i4MLAiJNkVIHEGxgJ+GBg3KiQV+fqpjblSBWa+aVz1nrxyAuvaWHzsnPZjMqpu\/WoSV3MCwNTT7l8fy\/SwPCoy+DgIK1YsaImnT179tQNkwZeGJg0KF6bRtadq5talCtVaOaX3lnrxUaEnzjmhcOrURjeal1vxCZLBWBgsqQdPy+RBoarMTQ0RMuXLw9uog7vOOKdSHwzdX9\/f3A\/kusHBsYN4aw7Vze1KFeq0MwvvbPSiw0Ij7zwE3Xeiwk1iaMwMDAmyuUXRqyBUUiUWdERRZkalwhhYNzQzapzdVP6cqYKzfzS3bVeasqIDUzngsmkbppOQkniKExaBiYODxeaFfUbJt7AxBHeVdiiiu+Kl2m6Ll5U07wRLhkBaJaMW16xXOnFZuPQqQvB1A9P+SQddQlzkbYjCQYmr5Zrli8MjAEnGBgDSAmCuOpcExQFUQwJQDNDUEKCpakXHzp3+NQFOvTqxcC08Am6c6eOC07ZTfOpdm9SmnmYppWWgcFVAqbE44WDgTHgBQNjAClBkDQ71wTZI0oCAtAsAbQco1TTi82Ies6OXK789+sjVyr\/zUZFf1yaFj2fqJur80IIA5MXebN8YWAMOMHAGEBKEKRoH0M1rM4fgbMXrhCvD8ADAlkSCO\/geeedd+i6666rWQQ+bE49E7QzW1yMrpiw4PeGF\/Q+1j4j9dEdk\/z1MDAwcYllG168gdEPsnviiSfo+eefD7ZOT5s2LTNSMDBuUPtsYLiT5bMu1DoAJsQfD\/4Y8EdgYlOj1YJGN8TtU\/VZM\/va+5eCr3pJuSMpLQMTp+W40Kyo3zDRBka\/SoAvdJw3b17QDgYGBnCVQJw3QmhYFy+q66qycVFbRtWQOv+lWJbHR83Kok1UPX3Wi0dh2mfdmusfAjAwst8e0QZGv7hx9+7dgYGZPn06bd68mTZt2kRNTU2Z0C2qe80EXo1MfOpc9e2iae66yFuDuPn7pFncuhUxvO965b2gNy0Dg0W8bt4u0QYmagTm4MGDuMzRTVvIPFUfOtfwiAufdZH2rovMwVtk6INmFtUrXFTf9cp7QS8MjOxXQrSBYXS4zFF2A7IpnfTONY3TRW34SIwrXTOJzPIsk+96HTp1MbjVmi+FnDt1bOYoYWAyRx4rQ\/EGJlZtHAXGFJIbsJI7V15EOPDCG6kd0OWGYPapStYsexrycyyCXupm67QOy4ujGgxMHFrZhxVpYMKjLmEsM2fOpN7eXqyByb69pJqjxM6Vp4z4Lz5+8ugwUwXsIDGJmjmoZmGSLIpeeS3ohYGR\/SqINDA6Mr4LiXcgdXZ2Bj\/mf\/MTvuDRJWaMwLihK61zVVs3be90cUNLRqrSNJNBRW4piqQXL+jN+rZqGBi5bZtLJtrA6LuQ1I6jqJ+5RgwD44awlM5V7TDic10w6lJbaymauWmRxUu1SHrlsaAXBkb2OyHawOi7kNSIS09PD3YhyW5TxqWT0LliyshYriCgBM3ilbjcoYukVx4LemFgZL8\/og0Mo8MuJNkNyKZ0eXeuvMuIFwjyuS68yyF8DLtN3YoaN2\/NisrVVb2KplfWC3phYFy1zHTSFW9g0qmmXSqYQrLjVy12np0r1rsk0zRPzZKVuNyxiqgXr4XJap1aWgYmTit0oVlRv2GiDUy13UjYhRTndZAb1sWLWq+26mA63iKd9zHl9coq8fd5aCaRgy9lKqJeauQ0iwW9MDCyW7poAxOFjnchTZo0iVpbWxORHRoaouXLl1fibt26NdjRpP9c\/UwFKqp7TQQwxUhZd676epeyn6ibVMasNUtaTsS7SqCoemW1oDctA4OrBNy8kd4ZGNtdSOFt2YyV0+zo6KCurq6A8pYtW2j79u2Vc2ZgYNw0viw7VyzWTUfDLDVLp8TlTqXIemVxTxIMjOz3xzsDwyMlvBMp6UF2HLelpWXUOTJ6mo2NjbRhwwZqb2+vjPLAwLhpxFl1rmrIeeRbd7qpSIlSzUqzEiF1WtUi65XFgl4YGKfN0zpx0Qam2hqY8BSPKYVqO5pOnDhBAwMD1N3dHSTFBqatra1icmBgTAnHC5dF54rFuvE0qRc6C83qlQG\/NydQdL1cL+hNy8CYK+Zm2q+o3zDRBiaO6EnC8nTSkSNHaMmSJfT000\/XNDB6+gcOHEiSHeKECJw7d47Gjx\/vhMv5t9+h\/3X81zT4yq9p4e9+lP7iM9lfBOekYjkn6lKznKtWyOyLrtfg8V\/TQ8\/9ggZXjKfmG69LXcPDhw8Tn0c2Z84c4tH5LJ60NJs\/f\/6o4p4+fTqL4meah2gDY3sS78mTJ2nlypXBwXcLFy4MDIreCNXU0Ve+8hV6\/PHHg2kpTCFl1\/5c\/nWohpex0yhdPV1qlm5JkRoTKINevKB3QlMDPdY+I3XR0xqBwSLe1KUJEhRpYNQJvIODg5G1jjIjJnjChojXw\/CzevVqLOI1AZhyGBedKxbrpixSKDkXmrktcblTL4terhb0wsDIfn9EGhiFzHbHURR6fbu0fp6M\/vP+\/v5R27SLOn+Yd9NMu3OFeXGvaNqauS9xuXMoi16utlXDwMh+f0QaGGVc7rvvPlq3bh0dO3ZsFEUcZCe7UZmWLs3OVe00mjtlLO2793bTIiBcTAJpahYzawRPQKAsevEfL3\/4jaPBNFL7rFsSkIqOAgOTGkonCYk0ME5qapEoRmAs4NWImlbnip1GbvSJSjUtzbIrcblzKpNeqh9I87gEGBjZ7494A8M7hTZu3IgRGNntKFHp0uhcudPCtQCJ8CeKlIZmiTJGpEQEyqYXj8LMmTI2tQW9MDCJml1mkUQbGHVuS2dnZ+KrA9IgiRGYNChem4Zt56rMC9+JgicbAraaZVNK5KIIlFGvNBf0wsDIfpfEG5jNmzfTpk2bKsf654ETBsYN9aSdKxbrutHDJNWkmpmkjTDpEyijXmku6IWBSb9NppmiaAPDFeUpJH74wsW8HhgYN+STdK66edn31dtpYlODm8Ih1UgCSTQDyvwIlFEvtaCX+4e5U+0OsISBya\/tmuQs2sBUu0oAu5BMpJUfJm7neujURVr0nZcIO43y0zauZvmVFDkzgbLqldaC3rQMTJzW6EKzov4RLtrAxBHdZdiiiu+SmUnacV5U7DQyIeo+TBzN3JcGOdQjUGa90ljQCwNTr4Xl+3vRBqbaCAwja25upr6+Ppo2bZpzgjAwbhCbdq4wL274J0nVVLMkaSNO+gTKrpftgt60DAyuEki\/bXOKog0MF5DXwAwPDxPvRFL\/5v+fNGlScIP0t7\/9bTdktFRhYNwgNulcsdPIDfukqZpoljRtxEufQNn1sl3QCwOTfptMM0XRBqbWZY58Su+OHTtgYNJsDRmnVatzxWLdjMUwzK7sH0RDTGKClV0vtW4u6YJeGBgxTTmyIKINjLrUkaeL9BGYI0eO0Pr16+nJJ5+s\/NwlZozAuKFbrXOFeXHDO41Uy\/5BTINhlmlALyJ1M32S86LSMjBxNHehWVG\/YaINDIseXgfDO5B27txJ27Zto7a2tky2VxdV\/DgvlYuwUS+q2gLJ26OTdDguyok03yfgonMFX3cEoNdVtrygt33WrdS5oCUWbBiYWLgyDyzewGROJCJDGBg3KoQ7VyzWdcM5zVTxQUyTpvu0oNf7jJMs6E3LwGARr5u2Lt7A9PT00K5du0bVHufAuGkMWaeqd65pnduQdR3Klh8+iH4pDr3e1yvJgl4YGNntXbSB4emjjo4O6urqohdffDHYeXTmzJmAaJYn82IExk0j5s71Qx84Dw\/2AAAgAElEQVS7lQZeeBMXMrpBnHqq+CCmjtRpgtBrNN64ozAwME6bp3Xi4g2MugvpxIkTdPDgQVq9ejVlfT8SDIx1O4tM4PCxk3Tv4C+C3+1cNsP62G83pUSqOgF8EP1qD9BrtF5xR3phYGS3d9EGhnch8VZpNi1vvfUWrVy5ks6fP0+YQpLdqExKx4t1v7DjBbruuusIdxqZEJMRBh9EGTqYlgJ6XUuKp5ImNDXQY+0z6mKEgamLKNcAog0Mkzl27Bhdf\/31wYm7Q0NDtG7dusxO4FXKYAQm3TaqzmZovvE6+seH\/mO6iSM1pwTwQXSKN\/XEode1SOOcDQMDk3qTTDVB8QYm1domTAwGJiG4iGj6EC461\/S4ZpUSNMuKdDr5QK9ojqZnw8DApNMOXaUCA2NAFgbGAFKdIDxlxIt1e\/a\/Rp0LJgfnMaBzteeadQrQLGvidvlBr+r8TM6GgYGxa3+uY4s0MLUucWQgWAPjulmkm74yLwMvvDHqMCl0rulyziI1aJYF5fTygF7VWZpMJcHApNcWXaTkhYHp7++n1tZWF\/U3ShMjMEaYIgPVuhYAnWtyrnnFhGZ5kU+WL\/SqzY0X9J69cKXqqd9pGZg46rnQrKjfMJEGRhc76iqB3t5eampqitMmrMIWVXwrKAaRdfMSdS2AixfVoFgIYkEAmlnAyyEq9KoPvdauJBiY+vzyDCHewITh8E4kPp03SxMDAxO\/iarFuu2zbqm6XRGda3yueceAZnkrEC9\/6GXGq9oBd2kZGFwlYKZD3FDiDczJkycr579w5RYuXEjd3d3U2NgYt66Jw8PAxENnelgUOtd4XCWEhmYSVDAvA\/QyY6X6LD6Tau7UsZVIMDBm\/PIKJdLA6NNGWS\/YjRICBsaseepTRiY3v6JzNeMqKRQ0k6RG\/bJAr\/qMVAi1HkY\/WNPWwPDOy0OnLtDfvPgKTW5pofY\/upV4VLrW40Kzon7DxBuYKKGzNjVFFd\/81a4fstZi3WqxXbyo9UuKEDYEoJkNvezjQq94zMOLem0MjBrVCZeg1rQ6h3WhWVG\/YSINTLwm5z50UcVPg5x+vsvcKWNp3723Gyfr4kU1zhwBExGAZomw5RYJesVHr0wM38\/2i3\/833Tp0iVavHgxjRkzxjgxtUW7WoTwVJUezoVmRf2GwcAYNMmiim9Q9ZpB2LysHTgebEM0mTIKJ+biRbWtE+LXJgDN\/Goh0CuZXmxiDr16kZo+\/Fva\/Pv\/HNvAqPjVcs96c0NRv2EwMAbtu6jiG1S9ahA1PDqxqSHxZYzoXG0UyCcuNMuHe9JcoVdSckQ8ivI\/n\/17uuPGN1M3MNxvRh0tgSmkeHrBwBjwgoF5H5LtqIvroVIDORHEggA+iBbwcogKveygJ10Dw3ct8QLeak+t6XYXmhX1GwYDY9C+iyq+QdUrQfS1LjajLjAwcajLC+uic5VXy+KUCHrZaZnUwGANjB1309gwMAakym5g1HwuGxe+iLHeNkADpEEQdK6mpOSEg2ZytDApCfQyoVQ9TFIDwylWG4VRl9lWy9WFZkX9hsHAGLTvoopfq+rqunkeeak1X2uAr2oQFy+qTXkQtz4BaFafkaQQ0MtODRsDwznzSMy2\/a8FhZjQ1EBzp46r+wegC82K+g2DgTFo30UVn6uuDlp6feRKsJuIDQs\/rkyLjtvFi2ogJ4JYEIBmFvByiAq97KDbGpgkubvQrKjfMBgYgxb28f\/2Axo\/frxByPyDKANiWhI2KvzMmXL1+OzH2meYRrUO5+JFtS4UEqhJAJr51UCgl51eMDB2\/FzHhoExIFxU92pQdadB0Lk6xeskcWjmBKuzRKGXHVoYGDt+rmPDwBgQhoExgJQgCDrXBNByjgLNchYgZvbQKyawUHAYGDt+rmMX3sDs3buXhoeHqbOzM2CpXxSp32w9NDREy5cvD8Js3bqVli5dWmEPA+OmGYKrG64uU4VmLummnzb0smOah4FxoZmLNO3IphO70Aamp6eHdu3aRWvWrKkYGP5ZS0sLLVq0iDZs2EDt7e00ffp06ujooK6uroDqli1baPv27dTU1BT8u6jip9OEkqcCrsnZ5RUTmuVFPlm+0CsZNxULBsaOn+vYhTUwPPIyadIkOnjwYMCQR2DU6Av\/d2trK6nRmXnz5hEbm97eXmpsbKwYGw7jysC46lh8StensqIdXO2KXGjmIk1XZfUtXbC1a7O1DIxPbF2V1bVBqZd+YQ2MqjgbE93AqJGWadOmBQbmyJEjtGTJEnr66aepu7s7CMsjM21tbZVpJBYfDwiAAAiAQLkIzJ49mxoaGujo0aN05crVIyZ8fU6fPu1r0auWGwbGwMAUTnVUCARAAARAAAQ8J1AIA3P58uVg1GRwcJCam5upr6+PeISFn\/AIzKpVq4LppDhTSJ5rjOKDAAiAAAiAQOEIFMLA1FJFNzDK0MRdxFs41VEhEAABEAABEPCcQOkMjL6NWt+dpG+j7u\/vD0Zo8IAACIAACIAACMgkUHgDIxM7SgUCIAACIAACIGBDAAbGhh7iggAIgAAIgAAI5EIABiYX7MgUBEAABEAABEDAhgAMjA09xAUBEAABEAABEMiFAAxMLtiRKQiAAAiAAAiAgA0BGBgbeogLAiAAAiAAAiCQCwEYmFywI1MQAAEQAAEQAAEbAjAwNvQQFwRAAARAAARAIBcCMDC5YEemIAACIAACIAACNgRgYGzoIS4IgAAIgAAIgEAuBGBgcsGOTEEABEAABEAABGwIwMDY0ENcEAABEAABEACBXAjAwOSCHZmCAAiAAAiAAAjYEICBsaGHuCAAAiAAAiAAArkQgIHJBTsyBQEQAAEQAAEQsCEAA2NDD3FBAARAAARAAARyIQADkwt2ZAoCIAACIAACIGBDAAbGhh7iggAIgAAIgAAI5EIABiYX7MgUBEAABEAABEDAhgAMjAG92267zSAUgoAACIAACGRB4J577qEnn3zSeVazZ8+mhoYGOnr0KF25csV5fi4zOH36tMvkc0kbBsYAOxuYtMV3kSZXxad0fSqrK7Zg4FebddUOXKWL9mXXvp555hm6dOkSLV68mMaMGTPqa+ETW1dlNfh8Og0CA2OAt6jiG1TdaRBwdYrXSeLQzAlWZ4lK1mvRYy\/R2QtXaOK4Btp37+3OGNgkXMvA2KRbK64LzVyk6ar+cdKFgTGgVVTxDaruNAi4OsXrJHFo5gSrs0Ql6\/WH3zhK7bNupbMjl2nghTdp7pSxxkbmwQcfpIcfftgZN5UwDIxzxFYZwMAY4JPcCRgUX2yQ1157jSZPniy2fCjYtQSgmV+tQqpeTff\/hDoXTKbOBS0B0EOnLtLap44H\/\/3y12fXhVxkA+NCs6J+w2Bg6r4qdnOoBsmXNoiLF7W0MDOqODTLCHRK2UjViw3Mvq\/eTnOnjh1V03sHjgejMbq5iUIBAxOvgcDAxONVqNBFFT9vkaR2rnlzkZw\/NJOsjj8jZmxgRr51ZyTMnv3D1LP\/NWqfdQs91j4jMgwMTLx2WNRvGEZgDNpBUcU3qLrTIPgYOsXrJHFo5gSrs0Sl6sXrX2pNFfGU0qLvvFR1XQwMTLwmU9RvGAyMQTsoqvgGVXcaRGrn6rTSnicOzfwSUKJePMIy8MIbdde6nB25Qmx0ohb3wsDEa4dF\/YbBwBi0g6KKb1B1p0Ekdq5OK1yAxKGZXyJK1IsNzOFTF4x2HLGJWTtwPNhuzSM2vD7m0KkL9DcvvkKTW1qo\/Y9uDaaaXD157EJyoVlRv2EwMAYtv6jiG1TdaRAXL6rTAiNxgmZ+NQKJevFC3ddHrhgZGEVbnRnDhib81ForY6sWDIwtQbfxYWDe4zs0NETLly8P\/rV161ZaunRphTwMjJtGKLFzdVPT4qQKzfzSUqJebEb4iXN4nVoTU41+1I6mNJSCgUmDors0YGCIaGRkhDo6OqirqysgvWXLFtq+fTs1NTUF\/4aBcdMAJXaubmpanFShmV9aStSLDcyEpoaqO4yiCHOcQ69erArf1SgMDIzs9g4DQ0Q8+tLT00O9vb3U2NhIGzZsoPb2dmptbYWBcdh+JXauDqtbiKShmV8yStTLhYGZ2NRQd1FwEuVgYJJQyy4ODMx7BmZgYIC6u7sD8mxg2traKtNIPAKjPwcOHMhOoQLndO7cORo\/fnyBa1i8qkEzvzSVqNdfPP0mNd94HT30uY8bw3zouV\/Q4PFfVw3\/6U820HeXpL+Y9\/Dhw3T58mWaM2dO8MdtFk9ams2fP39UcdO+kDgLFvXygIExNDBFFL9e43D9e4l\/Hbqus+\/pQzO\/FJSoV5IRGKyBsWt3RV0GAQODKSS7N8MitsTO1aI6pYgKzfySWaJebGDmTB1XuQfJlKi6ZiAcnqeP+Nm5bMY1VxOYpl0tHKaQbAm6jQ8Dg0W8bltYjdQldq65wfAkY2jmiVDvFVOiXuoWanWRYxyiPBKzbf9rQRReCDx36jiaM2XsqLNi4qRXLywMTD1C+f4eBuY9\/vo26v7+\/soCXv51UYff8m16hDNF8hYgQf4SP4gJqlGaKBL1sjEwtYRTZ8WkORIDAyP7VYGBMdAHBsYAUoIgEjvXBNUoVRRo5pfcEvVyZWCUMpx+WiYGBkZ2e4eBMdAHBsYAUoIgEjvXBNUoVRRo5pfcEvVybWDUSEytyyJNVYSBMSWVTzgYGAPuMDAGkBIEkdi5JqhGqaJAM7\/klqiXawPDCiU57TdKWRgY2e0dBsZAHxgYA0gJgkjsXBNUo1RRoJlfckvUKwsDU+sm6zgKwsDEoZV9WBgYA+YwMAaQEgSR2LkmqEapokAzv+SWqFcWBkap1HT\/T8jmniQYGNntXbyB2bt3L23cuHEUxfBli64Rw8C4ISyxc3VT0+KkCs380lKiXmwqHmufQXx\/keunZ\/8w9ex\/LbGJgYFxrZBd+mINjNrWHGVWlKkJb3e2Q1E9NgyMG7ISO1c3NS1OqtDMLy0l6pWlgWG1bNbDwMDIbu8iDQzfDj04OEgrVqyoSW\/Pnj11w6SBHwYmDYrXpiGxc3VT0+KkCs380lKiXlkbGFaM8+xcMDn26b8wMLLbu0gDIw0ZDIwbRSR2rm5qWpxUoZlfWkrUKw8Do0wMb61WVw+YKAkDY0IpvzCiDQyPxKxatYqOHTt2DaHm5mbq6+ujadOmOacHA+MGscTO1U1Ni5MqNPNLS4l65WVgkkwlwcDIbu+iDQyj4\/Uuw8PD1NnZGZDkf\/MzadIkGhgYoG9\/+9vOCcPAuEEssXN1U9PipArN\/NJSol55GRh1o\/XIt+40FhEGxhhVLgFFGxgegdm8eTNt2rSJmpqaAkDqZ\/fddx\/t2LEDBiaXZpNOphI713RqVtxUoJlf2krUiw1MHBORJvG4ozAwMGnSTz8t0Qbm8uXLtGHDBuLpIn0E5siRI7R+\/Xp68sknKz9PH837KWIExg1diZ2rm5oWJ1Vo5peWEvXK08CoURjTs2FgYGS3d9EGRo246OtgZs6cSTt37qRt27ZRW1sbLV261DlhGBg3iCV2rm5qWpxUoZlfWkrUK08Dw+rdO3CcXh+5Qvvuvb2umDAwdRHlGsArA\/PEE0\/Q888\/H2ydzmLxrlIGBsZNG5XYubqpaXFShWZ+aSlRr7wNjLpmwGQUBgZGdnsXbWDUFBKPtPBC3nnz5gU0efFud3c3NTY2ZkIXBsYNZomdq5uaFidVaOaXlhL1ytvAsIK8FmZCU0NwInCtBwZGdnsXbWD0Rby7d+8ODMz06dOvWdjrGjEMjBvCEjtXNzUtTqrQzC8tJeolwcCY7kiCgZHd3kUbmKgRmIMHD9L58+cxAiO7XRmVTmLnalTwEgeCZn6JL1EvCQaGVeRLJedMGVtzFAYGRnZ7F21gGF34MDtexNvb21vZVp0FXozAuKEssXN1U9PipArN\/NJSol5SDMzAC28GC3prbemGgZHd3sUbGAn4YGDcqCCxc3VT0+KkCs380lKiXlIMDCvJZam1mBcGRnZ7F2lgal0hwDizHoWBgXHTiCV2rm5qWpxUoZlfWkrUS5KB4REYfqot5oWBkd3eRRoYHVm1qwSyOP9FlQMGxk0jlti5uqlpcVKFZn5pKVEvSQam3jQSDIzs9i7awNS6SkC\/XsA1YhgYN4Qldq5ualqcVKGZX1pK1EuSgVHTSDwC0z7rlmvEhYGR3d5FGxh9F5Iacenp6cEuJNltyrh0EjtX48KXNCA080t4iXpJMzC1ppFgYGS3d9EGhtFhF5LsBmRTOomdq019yhAXmvmlskS9pBmYWtNIMDCy27t4A2OLL7yGRjdECxcurJwnMzQ0RMuXLw+y27p166g7ljCFZKtCdHyJnaubmhYnVWjml5YS9ZJmYGpNI8HAyG7vIg0Mm4zBwcHgzqNaz549e2qG4emmXbt20Zo1ayq3VvPPWlpaaNGiRcFN1+3t7cHpvh0dHdTV1RVkt2XLFtq+fXvlrBkYGDeNWGLn6qamxUkVmvmlpUS9JBqYalcLwMDIbu8iDQwjUyMi4dEQ\/h2PqmzcuJH6+\/uptbU1kjCHmTRpEvHJvfx0dnZWpqP4vzmeGp3hKwrY2PABeXy\/kjI2Km0YGDeNWGLn6qamxUkVmvmlpUS9JBqYatNIMDCy27tYA6OwKbOiY4wyNdUwszHRDYwaaeHbrDntI0eO0JIlS+jpp58OppP4YQPDF0iqhcNsYPTnwIEDslX1pHTnzp2j8ePHe1JaFJMJQDO\/2oFEvT796DD9\/ddaxIHkcg2uGE\/NN15XKdvhw4eJN5PMmTMns8uD09Js\/vz5oxifPn1aHHPbAok3MLYVTMvAFFF8W7a28SX+dWhbp6LHh2Z+KSxNr3rnruRJl+9Gap91K3UueN9cYQQmT0Xq510IA6O2W\/O6mebmZurr6yMeYeEnbGBWrVoVTCdhCql+43AdQlrn6rq+RUgfmvmlojS9evYP08ALb9DLX58tDmTUdmoYGHEyjSpQIQxMLcS6gVGGBot4ZTRKaZ2rDCqySwHNZOsTLp00vSQbmKjRIRgY2e29dAZG30at707St1GHFwdjEa+bRiytc3VTy2KlCs380lOaXjzK8frIFdp37+0iQYYvd4SBESlTpVDiDYxuOJ544gl6\/vnng63TaoooC7wwMG4oS+tc3dSyWKlCM7\/0lKYXb1fmR6qBCa+DgYGR3d5FGxj9KoHh4WHi7c78DAwMVA6gywIvDIwbytI6Vze1LFaq0MwvPaXpxQZmztRxoxbKSiIaXgcDAyNJnWvLItrA6Jc57t69OzAwfOjc5s2bCZc5ym5YJqWT1rmalLnsYaCZXy1Aml4Sz4DRFQ2vg4GBkd3eRRuYqBEYPpju\/PnzGIGR3a6MSietczUqdMkDQTO\/GoA0vaQbmLMjV4inkXiX1MSmBoKBkd3eRRsYRofLHGU3IJvSSetcbepSlrjQzC+lpejFIxs9+1+jOVPG0mPtM0RDZAPTuWAytc+6BQZGtFJE4g2MOu5\/9erVxGe4HDt2rOYVAi54Yw2MC6pEUjpXN7UrZqrQzC9d89SLRzPYuPC5L\/xIPPslSk19HQxGYGS3d9EGhqeQHnnkkWDX0Ysvvki8kJeP\/edLHB944IHMjnaGgXHTiPPsXN3UqPipQjO\/NM5DL2VceMSFp2F8MS5KWT6rhss+8q07MQIjvLmLNjDhRbx8AN1dd92FRbzCG5Vp8fLoXE3LhnDRBKCZXy0ja73Ux5+NS\/hYfp\/I8VodNl4v\/s2P6dKlS7R48WIaM2ZMJlVwoVlR\/wgXbWDUCMyXvvQl2rVrF3V1dQUNCCMwmbxHzjNx8aI6L3TJM4BmfjWArPTiUZdF37l6xotvIy5RirKB4bU6jT8fgoER3ORFGxjmpk7I5VNzeR2Mfpt0VlyL6l6z4lctn6w617zrWaT8oZlfamahlxp1mTtlrNgD6uKqpg60+w+XX4aBiQsvw\/DiDUyGLKpmBQPjRoUsOlc3JS9vqtDML+1d6qUv0i3CqIuurFrIe1fjz2BgBDd58QaGL2Pk6SP9mTlzJvX29lJTU1MmaGFg3GB22bm6KTFShWZ+tQFXehVtyiisqhpV+h93\/BIGRnCTF21geBGvmjLiXUiTJk2iM2fOBDiXLl2aGVYYGDeoXXWubkqLVJkANPOrHbjQS9+l4xcN89KqE3lhYMyZ5RFSvIFR1wacOHGC+BReXgeDqwTyaCrp5+mic02\/lEhRJwDN\/GoPaerFoy5rB47T2QtXvN5hZKogL+SFgTGllU840QaGdyHt2LEjMC1vvfUWrVy5MrhGAFNI+TSWtHNNs3NNu2xIL5oANPOrZaSllxqR4O3R+756e3C+S9EfXsh7x41v0mdu+g22UQsVW7SBYWZ88u71119P06ZNC3YkrVu3jvr6+oJ\/Z\/VgCskN6bQ6VzelQ6pRBKCZX+3CVq+yjbro6vLN2f9+8Tzd03IRBkZosxdvYCRwg4Fxo4Jt5+qmVEi1FgFo5lf7sNFLP5SuLKMuurq8E+n\/vnKG7pv+FgyM0GYv3sDwXUgbN24chQ9TSEJbU8xi2XSuMbNC8JQIQLOUQGaUTFy9eMSFj9HnKSMfrwFIE6uaNnv00+dhYNIEm2Jaog2Muom6s7OTWltbU6x2vKQwAhOPl2nouJ2raboI544ANHPH1kXKJnqp81wOn7pAh169SHwgXfsf3RrcxlzmRxmYzb\/\/z\/Tl\/7IQVwkIbAziDUzWO46iNIKBcdNyTTpXNzkj1aQEoFlScvnEq6YXf5wPnboQjLTw4\/vdRa7o8k4kXgOzddXnYWBcQbZIV7SB4XrxFBI\/WZ77EuYJA2PRwmpExcfQDVeXqUIzl3TTT3vjUy\/R2+820OsjV4LtzzzaogzLnCljae7UcaUfaalFXe1EgoFJv22mkaJIA6OmjngHUtSDNTBpSJ9\/GvgY5q9B3BJAs7jErobnBbFxnrMjl0cFZwMSftiQ8KNMSVT6zTdeR7fd\/FGa0NQQXE6IJx6B\/7TtEH3k339JP\/zvn8UITDx0mYQWaWAyqXmMTHgYMY9HnbUwcdzVMxe4E5rY1EidC1ryKE7qeeJjaIaUh\/n5I8UfNf1DZvIBM8sBodImYHtOinrnVbn43Q8\/3Bfwc3X6J3q9Ct4xO2WfeeYZXCVgh9BpbLEG5uTJk5WD6xYuXEjd3d3U2Hj1hc36yWsKSf3Vpv4aCw8DKw6qs2yfdatX5gad6+iWzNs2WWNeSKk\/upHVP2T6B0wPP+E9w8s\/mzt1bKqvCzRLFafzxKCXHWIYGDt+rmOLNDB8Au+GDRuovb092H3EFzq2tLTktg4mLwMTR3w2O2oXgfqrjOe4JY\/YlLlzjVpEybpJX5dQZs3ivI9SwkIvOyVgYOz4uY4t0sDwGhh99xGPxuzfv5\/Wrl1rzEM\/P0ZfM6Ovr9FHdviU3+XLlwfpb926dZRZ8sHAhMGEDQ0PMfMITdp\/kRsLEhGwbJ2r2q7K52zwqApPE8yZOg6jZjaNCHFrEijbO5Z2c4CBSZtouul5Y2D27NlDDzzwgNE0EhueLVu20Pbt26mpqSkYweE7lHgaiu9W4tGcRYsWVUZ5pk+fXrn1mvHqcfnfPhoYvZmwmeFpKGmHU5Wlc2XuA3\/3RjA15PvhYGXRLN1uNr\/UoJcdexgYO36uYxfSwISh8ejKwMAArV+\/PhjFUQfj8SjN8PAwzZs3LzA5vb29gUHSp6+KYGDCZkaNAOxcNiPXEZmid67qHhk2LjwCVoRdIEXXzHWHm3X60MuOOAyMHT\/XscUamFWrVgUXOUY9cbdRqzU0d911V2WkhS+DZANz5MgRWrJkCT399NPBCA0\/bGDa2toq00g8AqM\/Bw4ccK2L0\/TPv\/0OfffvLtLg8V\/Tpz\/ZQN9dks+Jm+fOnaPx48c7rWteiX\/3\/1ykXX93kXgb60Of+3jAuQhPkTUrgj7hOkAvO1UPHz5MvCZzzpw5RqP\/drldjZ2WZvPnzx9VnNOnT6dRPFFpiDQwaRJSoyw86sLrXzo6Oqirqyu4zTqOgSmi+IdOXaS1Tx0PcOcxGlPEvw551GXRd14KmL789dlpNmURaRVRMxFgHRUCetmBxQiMHT\/XsQthYNSupcHBQWpubqa+vr7AoIR3L4XvVirjFFK4QemXt3UumJzpgtKida767b1FNC\/cdoqmmesONu\/0oZedAjAwdvxcxy6EgYmCxOaF17aEL4FUpqZMi3hNGpG6uGzkW3eaBE8lTJE6V2VesuSXiggxEymSZjGr7mVw6GUnGwyMHT\/XsQtpYPQt0Qqg2jLNozVqfc2aNWuCBb386HH6+\/tHGR\/fdyGZNiKeUuLpD76Ndt+9t5tGSxyuCJ2r2ho98MIbwTb1opySXE3UImiWuMF6GBF62YkGA2PHz3XsQhqYtKGVxcAwtyzXcBShc+WRFzYveawhSrudm6RXBM1M6lmUMNDLTkkYGDt+rmPDwBgQLpOBUSZm7cDx4PZal2s5fO9cFz32UsBo31dvD853KcPju2Zl0EivI\/SyUxwGxo6f69gwMAaEy2ZgsjIxvnau6nwX1wbPoGlmHsRXzTIHJSRD6GUnBAyMHT\/XsWFgDAiX0cAoLGqUwcVIjI+dq25eyjTyotqDj5oZvOKFDQK97KSFgbHj5zo2DIwB4TIbGMbjysT42LmWcdoIUxIGnYTQID6+Y5JQwsBIUuPassDAGOhTdgPjatTBp861zNNGMDAGnYTQID69YxIRwsBIVOX9MsHAGOhTdgPjajrJl85V35lVxmkjGBiDTkJoEF\/eMaH4CAZGqjJXywUDY6APDMz7kNQUShrbhn3oXGFeRr8gPmhm8EqXJgj0spMaBsaOn+vYMDAGhGFgRkNiE8M3LPNoxNypYw0IRgeR3rlmeSZOYogZR5SuWcY4xGcHvewkgoGx4+c6NgyMAWEYmGshKRPzWPsMap+V7DZryZ1rHlcrGDTF3INI1ix3OPVk1KwAAAukSURBVAILAL3sRIGBsePnOjYMjAFhGJhoSLb3\/0jtXFW9sr7c0qAp5h5Eqma5gxFaAOhlJwwMjB0\/17FhYAwIw8BUh6TuT+KTaOOeFSOtc9V3GpXhXiODpn9NEGmaJalDmeJALzu1YWDs+LmODQNjQBgGpjYk\/vD37H+NeNolzm3MUjpXdSEj14GNWNl3GtVSW4pmBq8tghAR9LJrBjAwdvxcx4aBMSAMA2MAiYjUaAyHNpl+kdC5qssYucwYdamvswTN6pcSIRQB6GXXFmBg7Pi5jg0DY0AYBsYAkhZErSGpZ2Ty6lzDIy5xp77i0ShW6Lw0KxbF7GoDvexYw8DY8XMdGwbGgDAMjAGkiCBsZA6fuhBsueapmfAIR5ZceXrr0KkL9PrIlarlSVbLcsXKUrNykXVTW+hlxzUPA+NCMxdp2pFNJzYMjAHHoopvUPXUguhmhhNlQ3P+xDHq+K93U+eCltTyUQmFDYvKE9NEdqjxLtjxyzo29LIjDgNjx891bBgYA8IuOgEXaXJVfEiX18ocfvUibf\/ej6l5+kziKR39YXOjnonj3v\/vKKnOXrhyTXxlVjjuhKYG4rNq0nhcsHWRpi\/tQGkCBldJuODgIk1XZXWVrg2DWgbGJt1a\/ZGLdF2kmUafapsGDIwBQRYfT7YErvzufw4y\/H\/X31Qz4w\/+5i1qeOVH2RYOuYEACJSCwOzZs6mhoYGOHj1KV66M\/kPLNwCnT5\/2rch1ywsDUxcRAoAACIAACIAACEgjAAMjTRGUBwRAAARAAARAoC4BGJi6iBAABEAABEAABEBAGgEYGGmKoDx05MgRGjNmDM2cOZP+6Z\/+ifbu3UszZsygP\/mTP6HrrrsOhIQRePfdd+nHP\/4xvfLKK\/SRj3yElixZQrfckuyCT2FVK2RxLl++TD\/4wQ\/oZz\/7Gf35n\/85TZ48uZD1LEqlfvWrX9H3v\/99evvtt2nChAm0aNEi+vCHP1yU6lnVAwbGCh8ip0ngnXfeoX379tFf\/dVf0erVq+n3fu\/36Hvf+x59+ctfpr\/927+lj370o9Ta2ppmlkgrBQKXLl2iH\/7wh7Rs2bLAwOCRTeCv\/\/qv6aabbqJp06YF79ef\/dmfUWNjo+xCl7h0J06coFdffZX++I\/\/uMQUoqsOA4Mm4YwAj5wMDw9TZ2dnkAf\/5bdhwwYaHBwMRld6e3upqampkj8bmN\/85jfBqAs\/3MnyaMyKFSvo5MmTwf++8IUvOCsvEr5KYGhoiAYGBqi7uzv4sNXT7Y033qAtW7YEo2ZTpkyhe+65Bx\/EHBrTyMgIdXR0UFdXV2BO+OF3cOPGjcF\/9\/f3B38AfPe73w0+hvzX\/J49e2jhwoWj3sMcil7KLE31ev755+lHP\/oRffCDH6TFixfTHXfcQR\/4wAdKySxcaRgYNAMnBHp6emjXrl20Zs2aioHRDQ3\/vqWlhebPn1+ZeviDP\/iD4C94\/oAqA3P8+PFgyBQGxolM1ySqPnj8UVMGJko37kj\/4R\/+gf71X\/+Vxo8fH0zt8f8\/99xz9Du\/8zs0b968bAqMXAIC\/H6sXLky+O++vr7AwPDP2Fhu376d+K94ZUr5\/1nfm2++GQYmp\/YTRy82OjfccANdf\/319Pjjj9Of\/umfYor2Pd1gYHJqwEXOlj94kyZNooMHDwbV5BEY9Vd8W1sbLV26tPJX\/sMPP0y\/\/e1vg78oPvaxjwV\/ZSgDw3\/N85QSTyG9\/PLL9C\/\/8i\/0+c9\/vsjocq0bcz9z5kxQBh75YgPDD4+ahXXjD+O\/\/du\/Ea9\/YV14fn7WrFl06NAh+uUvf0lf\/OIXc61LmTLnD9zu3bvp7rvvpoceeoi2bdsWGBh+D5WO\/P6p0Zmf\/vSnwfQs\/wHBIzD8Pt54441lQpZrXePq9frrrwd6feITnwgMDJtPHj3DQwQDg1bgjACPsoQNTHt7ezCMzR9L\/n14GonDKwPzmc98JuiE+QPJC9l4XQz\/1YjHLQH9w6cMTC3dWJsdO3YEfxX+\/Oc\/p7Vr12JKwq1EkanzX\/Xr168fZWDUFC5\/NFetWhX8McEfP55GGjduXGB0YDZzEOu9UTMTvdhc8jvJen384x8P1pphM8NVzWBg8mm7pcg1qYEpBRzBlYxrYARXpVRFMzUwWAgvo1lAL3sdYGDsGSKFKgSiDEx4KkKtswBEOQSiDAx0k6NPtZJEfRCjppDUAl\/5NSp2CaGXvb4wMPYMkYKBgeEgUYtBef4djywCuoHhXUjQTZY+pgam2iJebJmWoWfYwECv+LrAwMRnhhiGBPQRGI6ib8fVd7kYJodgGREIGxjolhF4y2zCH0T1RwNvo25ubq7sTrLMBtFTIgC97EHCwNgzRAogAAIgAAIgAAIZE4CByRg4sgMBEAABEAABELAnAANjzxApgAAIgAAIgAAIZEwABiZj4MgOBEAABEAABEDAngAMjD1DpAACIAACIAACIJAxARiYjIEjOxAAARAAARAAAXsCMDD2DJECCIAACIAACIBAxgRgYDIGjuxAAARAAARAAATsCcDA2DNECiAAAiAAAiAAAhkTgIHJGDiyAwEQAAEQAAEQsCcAA2PPECmAAAiAAAiAAAhkTAAGJmPgyA4E8iag322kl2XNmjXU2dmZd\/Gs8uf7Zfbv30+rVq2iDRs2kLpFmxNV9W5vb6fW1tZr8uHf79ixg1avXk1NTU1W5UBkEAAB9wRgYNwzRg4gIIqA+pDrH3dRBUxYGN2A8I3LcQ0MZ6sM0Nq1axOWAtFAAASyIgADkxVp5AMCQgjUMjDqJuqzZ8\/SsmXL6FOf+hStXLmSzp8\/TzNnzqTe3t5gdGJoaIiWL18e3HL82c9+lm644YZg5IJHPngUh0c4OK3h4eHg3\/zffCsyP2qkh9PYtWtX8LODBw+SfkM532Suftff3x+EGRgYoO7u7uC\/2ZyER1I4vTNnztDSpUsroy3VRmA4P5U+p6fnvXPnTlqwYAFNmzZNiGIoBgiAQBQBGBi0CxAoGYGoKSRlTp599ll66qmnAqPCT0dHB3V1dQUfc2VIdKPC8dhMsJGpZmDmzZsXaT44\/XXr1lFfXx\/ddNNNFfPDP2cDw2V46623aMuWLfTAAw\/Qo48+Sps2bar8bPv27aOmejgO58Xmqdo0GafNhkifQtLj8e+5nvywEcIDAiAglwAMjFxtUDIQcELAZASGRzrOnTtXGX1RBWHD8pWvfIUef\/zxymhMlLHRR2BaWloqoy8qHR6FYbOhjIqa8uFRFR5FUSM3OgBlNPhnPIKir9fhOj3yyCO0YsWKwGxF1TFqDUzYvHDaPJITTt+JEEgUBEDAigAMjBU+RAYB\/wjEMTA8+hEe6eAPvDIePJ1kYmCiDImejomBUcaCiauRFkU\/iYGpNtICA+Nfm0aJy0kABqacuqPWJSZgamA4HK9p4bUwPJ2i1sesX7+eeJErj4DoU0j33XdfZeHsokWLKlNLbDbUVNH48eMrYSZNmhQ5AsPS6FNInN+2bduI4\/IuoVdeeeUaU6XihKeQqq2B4VEefqKmiTCFVOKXA1X3igAMjFdyobAgYE\/A1MDwqAjvyjFdxKsv1tUX99ZaxBs1haSmn9Si361bt1aMhr4wOExCHzmpNYX0xS9+MZgCO3bsWCUJtQaI66xPRdnTRgogAAKuCMDAuCKLdEGgJARqmYo0EWRxjgu2UaepGNICAbcEYGDc8kXqIFB4AlkYmJGRkWA6a+LEicFWah4piXps1q+E19EUXjhUEAQ8JwAD47mAKD4IgAAIgAAIlJEADEwZVUedQQAEQAAEQMBzAjAwnguI4oMACIAACIBAGQnAwJRRddQZBEAABEAABDwnAAPjuYAoPgiAAAiAAAiUkQAMTBlVR51BAARAAARAwHMCMDCeC4jigwAIgAAIgEAZCcDAlFF11BkEQAAEQAAEPCcAA+O5gCg+CIAACIAACJSRAAxMGVVHnUEABEAABEDAcwL\/H9OF1YMFTtZsAAAAAElFTkSuQmCC","height":337,"width":560}}
%---
%[output:795534a0]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10Xkd55x\/atERLCLaSUEcRxqkj09BQOc6GCmEw1Jt6264cDm1Xks+2rBCsW2I4u4lXkgnkNCTEktZOmyahuKnQprtY9rYkYJdCABMoqXAJTqyWEhIniu3IismHYxKIAw14z3OdUUZX92Nm7jzve+\/of8\/JcaT3mefO\/J55Z\/6az1ecPHnyJOEBARAAARAAARAAgQoReAUETIWihayCAAiAAAiAAAhEBCBgUBFAAARAAARAAAQqRwACpnIhQ4ZBAARAAARAAAQgYFAHQAAEQAAEQAAEKkcAAqZyIUOGQQAEQAAEQAAEIGBQB0AABEAABEAABCpHAAKmciFDhkEABEAABEAABCBgUAdAwBOBY8eOUW9vb+RtZGSEGhsbPXme6+bAgQPU09NDl1xyCQ0ODlJDQ4PYu3THtSyjSYEUhw9+8IPU2dlpkqTUNkNDQ7Rt27Yoj+vXr6f+\/n6r\/O7cuZM2bdpEmzdvLiUPLt\/evXvFvx9W0GBcWQIQMJUNHTJeNgK17NzTBAx3EKtWraK2tjYRPGlllH5vWmFcOkTuQL\/+9a9biQOXNLYB4HesW7duJlmIAiY0wWkbY9j7JQAB45cnvIFAXQicOHGCBgYGaPfu3bR9+\/aaCRge+anFe5Ogqs6wo6PDWIyoEQobceCSxqUSKAFjk7f4e8o+AqPq6eHDhzEK41JJkGYWAQgYVIhSEtCH0jmD+pC4PvqwfPlyuu6666IycEemT6eo0YKJiYk5n+s+Lr\/8cnrf+94X2TQ1NdHo6Ci1tLQkctGFQtw+PjrBn6sppdWrV9ONN95Ira2tUcOtd\/x5fngqSr133759Uf74UVNI11xzDX3sYx+LxIt64iz494qpLnBUp6nbq05Q+dI7VL2Mt9xyCw0PDye+d2pqKsrf9PT0TJ70GOocmfmtt95Kn\/rUp0iVj\/nHWSt2amouqbNWcdXfq8obL5eKdXNz84wIi\/PbtWtXNCWjHr1+xP3lCcd4fczylVUPs0ZqdCacZ5X3uCiKf790tsrHlVdeSXv27CH+\/qjY6WXm36l36LHN45JUD0vZCCFTpScAAVP6EM2vDMY7Lb30qhFO6qTiHQ\/7YfGgxEv886QONqvz58\/S8qY6m7POOmvWGhglYPQ8sFBIEhy6iIn78SVgkv7Cj3cm8Y4tjSv\/Pk3A9PX10YYNG+awzxIM\/Nk555xDTz75ZCTQkkQFv1PvaON5T6sX6r333Xdfohi54447Ztad6PVN76DjAibuS32eJmKy6iynOXToUKpQ0vMUFy9xkanEw9ve9jb6xje+MavxSBIhSd+vuABhm6Q88u\/Ve\/J861zKPko0v1rcapcWAqba8Qsu96qB1v8CVY0\/F1YffeC\/slXDqHcQemOr\/+Wpd3gsEtQIgfKh3h3\/S19BTvpcb4wvu+yyVAGj\/4Vq6ydPwPCoEz95UzlZI0Q8KvT000\/PYaKPGjCnZcuWzSqjyRRSfHpLsVfx5NGWeNw5L7weJGlkiFmuXbs2Kq8+YmOysNlkOihuE\/9ZMVFii\/Of925V95LKo37HQpfLnDaFpHNU9Ske0y9\/+cuREEoaUUnzGx+FU6NOuo+sd6sRGlX\/87j4mCoLruFDgZwIQMA4YUMiKQJpf52pDoAb7hUrViTuwNFtDh48mPhXNedb98F\/9asdQ3mLcPP+ckwTCHqDzu+39eNLwOjvZjHCj95hpnUsegf+\/ve\/31jAJI1YJb2X8xGfIksb4WBb7ohVPnS2Se+LT6VlCZi0qbN4mqzRlCTxGy+bmp6MCyEl2tKERl791OOr+0iLa3w0R7FSAiZt6lDfYafXZfW91KfvVDuhc0matpRqT+A3bAIQMGHHt3Klq7WA0bch53UQtsKD4Sdtqzb1o3fO8c6OfevbqE1GYNhGX\/jKP\/OW3fgIVLwDtRUwcY7xUZq4cPIlYFRlTxNOvDMrScDEp6LyRmCqIGCSRvxUXOP1L20ERveR9t2AgKlcExtUhiFgggpn9QvjOoUUn+pQawrS\/ppNGvLPEzBZUz\/6qABHgf9KTRMwpn54aD4uLtTUWlzAsEgwWRwZ79z1EYr4NBx3+HlTSDw6lCcA4j5cp5D02p02qhH\/BsTFSHw0QpVZH4lT5VF1J54maQop75snPYWkxK4auUoTMEkjV4pRfAQmbdF1fPoqawopiQumkPJqCz43JQABY0oKdjUhIL2IN0sA5AmYrLwlrQ9JEzB5fni4Xa1niUM3ETCcJmkXkvIV30miHwBns4hXTSXoafi97373u6PRoaSHOSWVz3QRL\/tUoi4unNIWuOppdBt+50033UTXX3\/9nAXHnCYuYPh3aQuCVVnzBHPS9EreCJjOMa2MWeJDFwwf+tCHUutWlg\/OQ9LiXtNFvDqXvBHImjQ0eEkQBCBggghjeIUw3Uatb4HO20adtDDYZgqJKWdNT+QtktVP5s3yw++Jb7n96Ec\/Svv3759ZtJo0AqN3bmkLkXXf8bU5SQJH78j1tPz\/SsAkvfe2226bdaIsH66nr7dx2UatCxG9Q03aYp+2fTvOVV+Twz6Z25YtW2jjxo0RDn0kTe0mS9uWnXd+S9Y2an6X6chE2toVHoVLEgdpo07MKGkLe9IoTpr45d\/HT\/5NW0ukfJiMFIbXoqFEEgQgYCSowqcogbwdH6Ivh\/PCBJKmquI7zdLO4dFf7nKQXeHMz1MHuuBUQi1pZ1IeHhxkl0cIn9sQCFLAcMPGZ1HwIVtJDWHWAWc28GBbHwIQMPXh7uutWVNoWVNfSe93uUrAVznmm5+kKSRmkHf4Y5LoDOXuqvlWB8pW3uAETN7iPvV5e3t7dNmZ+pm\/hLYXp5UtmPMlPxAw1Y90\/I8ILpGteOE0uFuntnUhPrVrI144pxCctY1X6G8LTsDwfC9\/SfhJG4GJB5X\/shgfH6\/prb6hVyyUDwRAAARAAAQkCQQlYPivumuvvZa6u7sjEQMBI1l14BsEQAAEQAAE6kcgKAHDIyn88ImQWWtgdNxqKLurqyuaUsIDAiAAAiAAAiBQfgLBCBieC7\/99tvp6quvJr6oz0TAqPUvHCb9FuN42H75l3+5\/JFEDkEABEAABEAggQDfKn7++ecHxyYYAcNTRnzWBJ8emrcLiaNoKl7YlgXM5ORkcMGvd4HAVS4CYAu2cgTkPKPeyrANlWsQAiZpR4OqBknX29vuPAo1+DJfFXOv4GrOytYSbG2JmduDrTkrW0uwtSVmZh8q1yAETDyEeSMwPFrDp1BmTRvpPkMNvlnVl7MCV7CVIyDnGfUWbOUIyHgOtc7OCwGjRlx4d9KyZcuiG4LVseCqumQdvR5q8GW+KuZewdWcla0l2NoSM7cHW3NWtpZga0vMzD5UrkEKGLOQmluFGnxzAjKWjz76aJALy2Ro2XkFWzteNtZga0PLzhZs7XiZWi+56M108DvfMjWvjB0EjEGoIGAMIDmYoLFygGaYBGwNQTmYga0DNMMkYGsIysJs7N6jdMXYA3TsxndapKqGKQSMQZwgYAwgOZigsXKAZpgEbA1BOZiBrQM0wyRgawjKwgwCxgJWiKYQMDJRRWMlw5W9gi3YyhGQ84x6658tBIx\/ppXyCAEjEy40VjJcIWDkuIIt2MoS8O8dAsY\/00p5hICRCRcEjAxXdLJyXMEWbGUJ+Pc+dNdBGrrrUayB8Y+2Gh4hYGTiBAEjwxWdrBxXsAVbWQL+vUPA+GdaKY8QMDLhgoCR4YpOVo4r2IKtLAH\/3iFg\/DOtlEcIGJlwQcDIcEUnK8cVbMFWloB\/7xAw\/plWyiMEjEy4IGBkuKKTleMKtmArS8C\/dwgY\/0wr5RECRiZcEDAyXNHJynEFW7CVJeDfOwSMf6aV8ggBIxMuCBgZruhk5biCLdjKEvDvnU\/h5a3UOInXP9tKeISAkQkTBIwMV3SyclzBFmxlCfj3DgHjn2mlPELAyIQLAkaGKzpZOa5gC7ayBPx7h4Dxz7RSHiFgZMIFASPDFZ2sHFewBVtZAv69Q8D4Z1opjxAwMuGCgJHhik5WjivYgq0sAf\/eIWD8M62URwgYmXBBwMhwRScrxxVswVaWgH\/va2+9n+555DgW8fpHWw2PEDAycYKAkeGKTlaOK9iCrSwB\/94hYPwzrZRHCBiZcEHAyHBFJyvHFWzBVpaAf+8QMP6ZVsojBIxMuCBgZLiik5XjCrZgK0vAv3cIGP9MS+PxwIED1NfXR8PDw9TS0pKYLwgYmXBBwMhwRScrxxVswVaWgH\/vy6\/\/Jh0+9gLWwPhHW1+PJ06coIGBAdq3bx+Njo5CwNQ4HBAwcsDBFmzlCMh5Rr31z7bxyrsjpziJ1z\/bunrcu3cvDQ0NRXnACEztQ4HGSo452IKtHAE5z6i3ftnyyAuPwEDA+OVad2\/Hjh2ja6+9lrq7uyMRAwFT+5CgsZJjDrZgK0dAzjPqrV+2EDB+eZbG286dO6O8rFixAmtg6hQVNFZy4MEWbOUIyHlGvfXL9p6Hj9PaT9yPERi\/WOvrjRfu3n777XT11VfT1NSUkYDRc7xnz576FiCQtzP75ubmQEpTrmKArVw8wBZs5Qj48bx69erI0U8Wv5WeX\/FeCBg\/WMvhhaeMVq1aRW1tbYRdSPWLCf7akmMPtmArR0DOM+qtX7ZDdx2kobsehYDxi7V+3njtS29vL01MTMzJxPbt2yNRE3+wjVomXmisZLiyV7AFWzkCcp5Rb\/2yVWfA\/NzzT9FTn\/x9v85L4O0VJ0+ePFmCfNQtCxiBqRt6dLKC6NERyMEFW7CVI+DXs9pCfdpTD9ITf\/1Hfp2XwBsEDA6yq1s1REcghx5swVaOgJxn1Ft\/bPUdSA3\/\/Gk68pW\/8ue8JJ7mvYAxiQOmkEwo2dugsbJnZpoCbE1J2duBrT0z0xRga0oq304XMGfcM0yHv\/WF\/EQVs4CAMQgYBIwBJAcTNFYO0AyTgK0hKAczsHWAZpgEbA1BGZip9S9suuCzvTQ5OWmQqlomEDAG8YKAMYDkYILGygGaYRKwNQTlYAa2DtAMk4CtIagcM330ZeXSBfSdrb8LAeMHbfW8QMDIxAyNlQxX9gq2YCtHQM4z6q0ftmP3HqUrxh6InN3afSFd3dkOAeMHbfW8QMDIxAyNlQxXCBg5rmALtrIEinvn0ZcNYw\/QPY8cp8WNp9P+j7yFQu3DMIVkUF9CDb5B0UVNIGDk8IIt2MoRkPOMelucrT76svGyJfTh3zofAqY41up6gICRiR0aKxmuGCWQ4wq2YCtLoJh3Hn3hu4\/4Xx59uaXrQlp5wQIImGJYq50aAkYmfhAwMlzRycpxBVuwlSVQzPvIPx6h\/\/mZhyIn\/WvOp\/41S6L\/D7UPwxSSQX0JNfgGRRc1gYCRwwu2YCtHQM4z6q07W33qSK19Ud5C7cMgYAzqS6jBNyi6qAkaKzm8YAu2cgTkPKPeurHVt03rU0cQMG48g0oFASMTTjRWMlwxzSHHFWzBVpaAvXd93Qun3vWBi6N1L\/oTah+GERiD+hJq8A2KLmoCASOHF2zBVo6AnGfUWzu2+sgLp9zwjtfRx9ZeMMdJqH0YBIxBfQk1+AZFFzVBYyWHF2zBVo6AnGfUW3O29zx8PNpxpJ4\/bGuiP\/vPb0h0EGofBgFjUF9CDb5B0UVN0FjJ4QVbsJUjIOcZ9TafLY+6fOa+79N1f\/\/y3UZ\/u76VfuMNjamJQ+3DIGDy60uwW9AMii5qgsZKDi\/Ygq0cATnPqLfZbHnUZcOOB6JzXvjhBbu8Xbr70kWZCSFg5Ops6T2HGvx6g0djJRcBsAVbOQJynlFv09kOf+kgDX7x0RkDFi+8YJf\/zXtC7cMwApMX+YAPATIouqgJGis5vGALtnIE5Dyj3s5myyMt9zz8DG3Y8b1ZH4y+5yK6vPUc40BAwBijCs8w1ODXO1JorOQiALZgK0dAzjPq7Sm2LFy++K9P0cCdB2bBthl10ROG2odhBMbguxhq8A2KLmqCxkoOL9iCrRwBOc+ot0Qf3PE9+vS3Hp8jXNS9Ri70Q+3DIGAMakOowTcouqgJGis5vGALtnIE5DzP13rLi3OH73qU7nnkuFfhopyF2odBwBh8F0MNvkHRRU3ma2MlCvUl52ArRxlswdYHARYtn93\/BH1q\/Mgcd0nXARR5Z6h9WDAC5sSJEzQwMEC7d++O4rx+\/Xrq7+9PjfnOnTtp06ZN0ecdHR00ODhIDQ0NifahBr\/IF8JHWnQEPigm+wBbsJUjIOc59HrLa1v+Zt\/36eNfePkMF0WTRctbly6ItkWb7CyyiUKofVgwAmZoaCiKJ4uWY8eOUW9vL3V1dVFnZ+ecOO\/du5fYfmRkJBItLHyamppSBU+owbf5AkjYht5YSTAz9Qm2pqTs7cDWnplpihDZ8kjLl777FN3ytccSMfgebUl6Sah9WDACJh40XdDEP+PRl\/Hx8ZlRl\/jPcftQg2\/aqEjZhdhYSbGy9Qu2tsTM7cHWnJWtZShs09a06KMtN3ddSK9vPN37aAsEjG2tK5m9GoHh0Zi2tjajEZj29vbE0RpODAEjE+BQGisZOsW8gm0xflmpwRZs4wR4auj\/\/tPjtHfy+JyFuLpo+fPOX6ElZzXURLToeQy1DwtuBIZHXrZt25a7ruXAgQPU09ND09PTtH379kShoyoAB19\/9uzZI\/cNnkeep6amqLm5eR6VuHZFBVs51mA7v9lOP\/tiBID\/\/ctvHad9R04d6x9\/ms48jc599Wm0\/tcXRP\/yz7V6Vq9ePedVk5Nz193UKj9S7wlOwChQLGRYnCQtzuUpox07dkRrYBobG6P1MPykLfoNVb1KVSpTv\/hL1pSUvR3Y2jMzTQG2pqTs7crKVq1j2f\/Yc6kjLFxaXs\/S+e8X0dsuWEgrL1hgD0AoRah9WLAChkdY+vr6aHh4mFpaWmaqhdqtpE8ZpdmqRKEGX+i7Yuy2rI2VcQFKbAi2csEB2\/DZsmD5zP3fp7sfPDZzcWJSqdXOoe5Lzy2VYInnNdQ+LFgBo+804lEW9UDAyDU+tp7REdgSM7cHW3NWtpZga0vM3L4ebFmsPPnDn9DoPx7JHF1RIyy81ZkFC4sX39udzUnZWULA2PHKtVYLbScmJnJtdYPW1la6884756TRp4GUSEnbGp00hZQ23cQvCjX4VuAFjOvRWAkUo5QuwVYuLGBbXbY2YkUJlnctfy29t\/28qNBVESwYgZGro5HnvJ1CSa9XoypJAiZ+kJ1+OJ36rLu7e2axrlrsy+\/BQXbCwU5xj45AjjvYgq0cATnPPusti5Vjz\/8b\/dU3pnJHVpQ4WbzwdOp76SC5qoqVpOiE+kd43aaQfAsYua8URmCk2PpsrKTyWFW\/YCsXObAtD1vevsxCg\/\/d+pVD9OiTz1uJld6VzXTWq36hUtNBLvQhYFyoBZIm1ODXOzzoCOQiALZgK0dAznNeveVRlQe\/\/yP63P4njISKPrKy+sKz6JLFZ5Z6sa0U2VD7sLqPwPAamLx7i6SCauo31OCbll\/KLq+xknrvfPALtnJRBltZtj\/\/mnOjEZVvH\/oBffV7x+jwMy9k7gTSc6PvCpoRL42ny2W4Ip5D7cPqJmBU3PW1KLzodnR0dNa25zLUj1CDX2+26AjkIgC2YCtHwI9nFin8fPXBY3THfd+3Fiq8XmXVskb69fNfE\/wUUFHiofZhdRcwKjDxXUllGpUJNfhFvxRF06OTLUowPT3Ygq0cATvPPO3Dzxe+8xT9y5Hsg+DintVCWrV1uUyHw9lRqK91qH1YaQSMHl79mP8yjMqEGvz6fqWI0MnKRQBswVaOwFzPPJrC\/z353E9odPxIZHDPI6eEi8mjhErrL51G1\/3umyJfVTpnxaSM9bQJtQ8rpYDRA512IF0tK0Oowa8lw6R3oZOViwDYgq1vAkqkTD71PP3Toz+gx469YCVSOD+RKFl4Ov3mG8+i5a87c+ZcFSVgUG99R+2Uv1D7sFIKGH0EhuHnXbYoE\/KXvYYafGluef7RWOURcv8cbN3Z5aUMnS1P+fzSmb9IN331MB1++oSTSGGGPO1z1WVLaPr4j41HU0Jnm1e3pD4PtQ8rjYDJOohOKqimfkMNvmn5pezQWEmRxfScHNlqs1VTMyxSnnn+3+iL\/\/qU00iKPpryX9qaqOk1r5wzmuISA7QJLtTy04Tah9VdwOi7kMow2pJUFUINfn61l7VAYyXHF2znN1u1w2fs3qP0jw8\/Y7XDRyenpnzaL1hAK5cujD6SXJuCeitTb0Ptw+omYPRdR3lH+cuE1NxrqME3JyBjicZKhit7Bduw2SqB8sDRH9GuiSecR1H0kRRek\/K+lefVdQEt6q1MvQ21D6ubgHn44YfpQx\/6EF1zzTUz9xPlhS7rLqS8tEU+DzX4RZj4SIvGygfFZB9gW122+jQPl2Ls3scjgWJzoFt8FIV\/XnnBQvqVRa+i5c2vFh1FKUIe9bYIvfS0ofZhdRMwuAtJpqJWySsaK7logW252aodPTwdc8vXDtP3Hv+Rs0DRR1F+9bwz6HcuOicqfBXPTEG9lam3EDCeucYPrjN139raSkm3UZumd7ELNfguLHymQWPlk+ZsX2Bbf7a8UPZ1LFDuPkwPHi0uUCJRsnQh\/ff\/sJiO\/uAnXhbNylFy84x668YtL1WofVjdRmDygJfp81CDX2\/GaKzkIgC2smz5vh5+fvziz+jmuw\/TwadOFB5BOTVqspCWntNAl77+NZF\/yQWzcoTcPaPeurPLShlqHwYBY1BfQg2+QdFFTdBYyeEF22Js1RTPl777FO1\/7LnImc3JsvG360fi\/8YbGunSJa+JFstWcZqnGNns1Ki3MnRD7cMgYAzqS6jBNyi6qAkaKzm8YJvOVomTpgWvpD\/9yiE69HSx0ZOZkZKFp0dTRpe3vjZaLKseJV7koh2OZ9RbmViG2odBwBjUl1CDb1B0URM0VnJ45ytbtb2Y\/\/3Gw8\/QN1+6j8fH6Akfgf+m886g5We\/SOcuOnfeTe\/I1daXPc\/XeivNNtQ+DALGoOaEGnyDoouaoLGSwxsa2\/jW4u8+\/kP6u39+svDUjho94X95\/UnzgldG\/+q\/j4+ghMZWrhbaewZbe2YmKULtwyBgDKIfavANii5qgsZKDm9V2CphoqZ1mMhzL7xIf\/cvTxY6+yQ+fcOjJxed92r67YvOnhEnrlM7VWErV7vkPIOtDNtQ+7BSCZidO3fSpk2bogjyBY6HDh2i8fFxGhwcpIaGhszIxu9SWr9+PfX396em4UPx1q1bF33OW7NHRkaosbEx0T7U4Mt8Vcy9orEyZ2VrWSa2+pknX\/ru07R\/6rnokkDXg9l0Fuqo+yVnN1DHr51DDb\/w8+Lbi8vE1rZelN0ebGUiFGofVhoBw3ciTU9PU19fH23YsCESHywsBgYGqKmpKVOMcMg5PT+cTp0x09XVRZ2dnXNqhLrtesuWLdEpwCycsoRSqMGX+aqYe0VjZc7K1rJWbPU1J\/ce\/AHd\/eCxKKtF1pzooyc8crLsl14VbS3+nTedM3PMPdu4jqDYsozb14pt0XxWMT3YykQt1D6sFAJGP5V32bJl1NvbGwkRFhfq+oCsEZKkkOuCJv45C5aDBw\/miiKVLtTgy3xVzL2isTJnZWtZlK2+5uQknaQvfOcp+s6RH3oXJ7xr5\/0rm+mHP\/6p+MiJLcM0+6JsfeUjRD9gKxPVUPuwIAVM1jUFaqqpvb09cXQmqfqEGnyZr4q5VzRW5qxsLdPYxhfDHn32x9GoSZG7dvS8qVERHjlZfNbp1PFrr6ULF70qqDNPUG9ta6O5Pdias7KxDLUPK4WA4UCoaRx9CkmNxqRNBaWNvGzbto3SbrhWAmbNmjV022230cTEBNbA2HwTPNqisfII8yVXar3J40cfp0efb6B7DjzjddSEnbE4eWPTGfQff\/VsOu3nXhEdxqamkuo1reOfZLpH1Fs52mArwxYCRobrLK\/6wlr1webNm41HSnRnak1NfAGwEjCHDx+eWbibZqv8cfD1Z8+ePTWgEf4rpqamqLm5OfyCeirhviMvRJ5++rOT9PcP\/oimn32RHn\/uxejfok\/TmadFLs599Wn0pkWvpPbXN0T\/rx71edH3hJAe9VYuimDrh+3q1avnOJqcnPTjvEReSjMC45sJL9Tl0Zzh4WFqaWmZcZ80hZRmqwuYEIPvm7mtP\/y1dYoYX\/rHz6LX\/CLdcvdjNPnk86d+\/9IhbLZcdXt9SudXm86gVcsW0hvPPaMUi2GLlKueaVFv5eiDrQxbjMDIcBXzmrX4l0dclixZMjOywwLmhhtuoK1btyZupQ41+GLwDR2H3lipaRXGMTp+hPYdejYi42P7MPuZESeNDfSmpjNo\/dubZ4TJY489Rm9tfVm4G4YEZgYEQq+3BgjETMBWBm2ofVgpRmDUoltej5L1ZJ3tou86UqMsaduv4+Ima8cS5yfU4Mt8Vcy9Vq2x0g9dU6X87P4n6CsPPC0jTF5aCLvxsiU09cyPrXbpVI2tea2pvyXYysUAbGXYhtqHlULAcMh4Ee+OHTtmHSinn+eydu3azDNh4gfZ6Yt41Wfd3d3R1mx+9PU2aQt+VVUKNfgyXxVzr2VprJKEybdeOtPE1+6cWSMmL1369wdtTfTiT09aCRNTumVha5rfKtmBrVy0wFaGbah9WCkETNa2Z3205KGHHooOrLvzzjtlopziNdTg1xRiwsukG6v4MfU85fKd6R\/S5z0dU68XSZ0Iy+eaXPy6M6PbiF+e4jm95qil2da8QCV6IdjKBQNsZdiG2odBwBjUl1CDb1B0UZOijZV+fw7vzPl\/+456O89EFVxfBPvONzTSpUteE30UCZbG2gsT04AUZWv6nvloB7ZyUQdbGbah9mGlEDAcsrwpJL4SQJ0Vc9NNN8lEGSMwNeWaddiaWgB71hm\/QJ\/42mN0yNPdOUnC5C1LF9DbtBuIyyxMTAOEjsCqqStJAAAgAElEQVSUlL0d2NozM00Btqak7OwgYOx4OVknnQPDlzqq+4puvvlmGh0dnbUt2ulFlolCDb4lBi\/m+sV+\/\/trD9K3H\/9Z5Nf7zpyFp9Mlrz+TetrPm3XIWgjixCQQ6AhMKLnZgK0bN5NUYGtCyd4m1D6sNCMw9iGpXYpQgy9BUAmUf\/vpz+gz9z\/h9dZhzm90Cuy5Z9AH3vG6eSlMTGOGjsCUlL0d2NozM00Btqak7OxC7cMgYAzqQajBNyj6HBP99uG\/ve\/70aFrRQ9c0xfAvuctTfSTF2V25riUt6pp0BHIRQ5swVaOgIznUPuw0ggYPkyup6eHpqen50SwtbV11vZqmRCnew01+Gkl1i\/8e+Doj2j3xBPOIkUXJ5e3vjbancP++f4cdARyNRlswVaOgJxn1FsZtqH2YaUQMPrx\/uq8Fz6zRV3m2N\/fP3N+i0x4s72GGny91Hyc\/Wfu\/z498oTdiIoSKMtfdyZdeO6r6K1LF8y4zVtvgsZKrjaDLdjKEZDzjHorwzbUPqwUAiZ+Dox+1D8v7B0bG6P4pYwyYU72GmLwx+49SmPfetxoZEUJkTVvPJuueGntCY+gFH3QWBUlmJ4ebMFWjoCcZ9RbGbYh9mFMqpQChrdLHzx4kHjkJetOI5lQz\/Va9eCrhbVj9z5OLFyyHhYrq1oW0u9fskj8rBM0VnI1GGzBVo6AnGfUWxm2Ve\/D0qiUQsBw5vT7iHTR8uUvf5nGx8cxAuNQr1m4bP3KQfo\/ex9PTK2mf27pvjD6PG\/KxyELmUnQWPkm+rI\/sAVbOQJynlFvZdhCwMhwnfGqr4PhQ+tY0Gzbto34QsZ6nP2iF7dqwef1LBt2PDCzzVgvC4sUvrX4j9\/+OuGI5rtHY5XPyNUCbF3J5acD23xGrhZg60ouO13V+jBTCqUZgTHNcD3sqhB8Hm356oPH6Mq\/eXAOIhYtt3RdKD4lZBsbNFa2xMztwdacla0l2NoSM7cHW3NWNpZV6MNsyqNsSyFgTC9zbGxsdClj4TRlDr5a37L2E\/fPKieLlnctfy29t\/28mk8NmQJHY2VKyt4ObO2ZmaYAW1NS9nZga8\/MJEWZ+zCT\/KfZQMAY0Ctr8JOmitRoi49dQgZoCpmgsSqELzMx2IKtHAE5z6i3MmzL2ocVLW1dBQzvNtq0aVNuGdavXx\/tSKrXU7bg86gL7yYauuvRGSRVEi4q02is5Go02IKtHAE5z6i3MmzL1of5KmVdBYwqRNYUkq+CFvFTtuAvv\/6bs+4BGn73MvrNN55VpIh1SYvGSg472IKtHAE5z6i3MmzL1of5KmUpBIyvwkj5KUvwecpIX+tSxVEXPUZorKRqLOGaBjm0YAu2ggRkXJelD\/NdOggYA6L1Dj5PGd313aeo\/44DM7n9656L6D+96RyD3JfXBAJGLjZgC7ZyBOQ8o97KsK13HyZTqjqexKumjSYmJnLLNp8vc2TxsvPbR2nzF0+td6n6qAtGYHKruxcDdAReMCY6AVuwlSMg4xkCRoZrJbzWK\/jxxbosXnZ94OLSbou2DSY6Alti5vZga87K1hJsbYmZ24OtOSsby3r1YTZ5dLENZgpJneS7e\/fuiIPpzqUDBw5QX18fDQ8PU0tLSyLDegX\/tnuOUP8dD82MvIQkXrhQaKxcvrJmacDWjJOLFdi6UDNLA7ZmnGyt6tWH2ebT1r5UAiZpW\/XmzZuJrxbIe\/S7lNT0VFdXV2ZaJXr27duXeV1BPYLP26SvGHsgWPECAZNXo4t9jo6gGL+s1GALtnIEZDzXow+TKclsr6URMCxeduzYQSMjI6RO3DUVIkmgdEGTBlJdGsmfl2kEZj6IFwgY2a83Olk5vmALtnIEZDxDwMhwjbz6vkrA5FwZtrn22mupu7s7ujiyLAKG173wOS\/q2f+RtwSz5iVehdARyH2pwBZs5QjIeUa9lWELASPD1buAUbdYd3R00ODgIDU0NCTmnEd8+FmxYoXRGhjdyZ49e0RoTD\/7Iv3JV56ifUdeiPz\/5bsX0SXnnS7yrjI4nZqaoubm5jJkJbg8gK1cSMEWbOUI+PG8evXqOY4mJyf9OC+Rl6CnkKanpxNFDC\/cvf322+nqq68mbozKsoh36K6DM9cDrFy6gHZdcXGJqor\/rOCvLf9MlUewBVs5AnKeUW9l2GIERobrLK9FFvHGs5e1u4hHaVatWkVtbW1Ull1I+tRRaNul06oOGiu5LxXYgq0cATnPqLcybCFgZLiKeVULdPVFwfyyrAP0tm\/fHoma+CMdfBYvG8YeoHseOR69mrdLV+E26aLBQ2NVlGB6erAFWzkCcp5Rb2XYSvdhMrnO91qKKaQiu41UEfVdR2p7dFNTU+4t1mUYgdF3Hc2HqSNMc+R\/MYtaoCMoShDiUI4g2NaaLQSMMPH49FHaaEhaNuIH2emLeNVnvOMoPsJSbwEzH6eOIGCEv0w4JFAUMMShHF6wlWELASPDNdGr2knEH\/IoyujoaOopubXIlmTwP\/b5SfqzPYeiYvSvOZ\/61yypRZFK8Q40VnJhAFuwlSMg5xn1VoatZB8mk2Mzr6WYQsrKKosZXs8SX8tiVjw\/VlLBj4++8Jkv8+lBYyUXbbAFWzkCcp5Rb2XYSvVhMrk191pKAaOPwNT7JmpGKRX8D3\/2YfrkPzwWRevW7gup+9JF5pELwBKNlVwQwRZs5QjIeUa9lWEr1YfJ5Nbca2kETNmmjXSEUsFvvPLu6DW8bXq+jb5wudFYmX9RbS3B1paYuT3YmrOytQRbW2Jm9lJ9mNnb5axKIWBMjv6XQ5DvWSL4w186SINffHTejr5AwOTXuyIW6AiK0MtOC7ZgK0dAxrNEHyaTUzuvpRAwdlmuvbXv4M\/3tS8qgugI5Ooy2IKtHAE5z6i3Mmx992EyubT3CgFjwMx38HUBM18OrUvCjMbKoPI5moCtIziDZGBrAMnRBGwdweUk892HyeTS3isEjAEzn8Fn8bL2E\/cT\/ztf175gBMag0hU0QUdQEGBGcrAFWzkCMp599mEyOXTzCgFjwM1n8PXRl\/\/1u8uo963nGeQgTBN0BHJxBVuwlSMg5xn1Voatzz5MJoduXkshYLIW8abdaeRWXLdUPoO\/9tb7592dR2nU0Vi51UeTVGBrQsnNBmzduJmkAlsTSvY2Pvsw+7fLpYCAMWDrK\/j66Mt8uvMIAsagknk2QUfgGajmDmzBVo6AjGdffZhM7ty91lXAxO8\/SivG+vXrcy9ldEeQn9JX8IfuOkhDd53aOj2fF+8q4ugI8uueqwXYupLLTwe2+YxcLcDWlVx2Ol99mEzu3L3WVcCobM+Xc2DU9NF8X7wLAeP+hTVNiY7AlJS9HdjaMzNNAbampOzsIGDseAVl7SP49zx8PNp9xA9fGcBXB8z3B42VXA0AW7CVIyDnGfVWhq2PPkwmZ8W8lmIEplgR5FP7CP6f\/N0j9OdfPRxlFtNHp2KGxkqu7oIt2MoRkPOMeivD1kcfJpOzYl7rJmD0aaNly5ZRb28vTUxMJJam3hc6+gj+fL\/3KCmwaKyKfXmzUoMt2MoRkPOMeivD1kcfJpOzYl7rJmCKZbu2qYsGX58+2vp7y6inff6e\/aJHDo2VXD0GW7CVIyDnGfVWhm3RPkwmV8W9QsAYMCwa\/Os+P0l\/uudQ9CZMH70MHI2VQeVzNAFbR3AGycDWAJKjCdg6gstJVrQPk8lVca+lEDBqOinUKSTsPkquqGisin+B0zyALdjKEZDzjHorwxYCRoZrplcWNldddRV9+MMfppaWljrk4NQriwRfnz764DsX07UdS+tWjrK9GI2VXETAFmzlCMh5Rr2VYVukD5PJkR+vpRiBySoKXyUwNjZGg4OD1NDQkGp64sQJGhgYoN27d0c2WYffxUd8Ojo6Mv0XCT4Or0uPLhorP1\/iJC9gC7ZyBOQ8o97KsC3Sh8nkyI\/XSgiYoaEhGhkZocbGxtRSsw0\/\/f39pARKV1cXdXZ2zkqjhE57e3v0mfq5qakp9bTfIsHH9BEEjJ+vqp0XdAR2vGyswdaGlp0t2NrxMrUu0oeZvqMedqUXMCxMpqenc0dg4vB0QZMHlq80GB8fT32Ha\/D1u4\/e3rKQPvvHy\/OyMq8+R2MlF26wBVs5AnKeUW9l2Lr2YTK58ee1FAImaxEvj4yMjo5arYGxvZpASsDo61\/45F0+gRfPywTQWMnVBrAFWzkCcp5Rb2XYQsDIcJ3xmja1o6Z6TF\/PIy\/btm2jvHUtyl\/WdJOycQ2+mj5iP9g+PTeCaKxMa7W9HdjaMzNNAbampOztwNaemUkK1z7MxHc9bUoxAsMAkqaKTMRFGjyTqSclmthH1iJhDr7+7NmzxyhmHbdP0fSzL1LTmafR7vc0G6WZT0ZTU1PU3AwuEjEHWwmqp3yCLdjKEfDjefXq1XMcTU5O+nFeIi+lEDBZUz6mu5DiTA8cOEB9fX00PDycOP1kKl7Yr4t61de\/4PLG5BqPv7bkWgKwBVs5AnKeUW9l2Lr0YTI58eu1EgLGZBdSHAsLn7R0JjuPdH8uwcf26fyKisYqn5GrBdi6kstPB7b5jFwtwNaVXHY6lz5MJid+vZZCwMTXv+hFzFtgq2z1XUd5AsVkeqmogNHXvxy78Z1+oxaINzRWcoEEW7CVIyDnGfVWhi0EjAzXGa88YrJx48ZZO454Gqinp4e2bNlCbW1tmTmIH2SnL+JVn3V3d1PazddZN167BB+3T+dXGDRW+YxcLcDWlVx+OrDNZ+RqAbau5DACI0POwiuLmHXr1s1KsX379lzxYvEKJ1NbAaOvf+lfcz71r1ni9N7QE6Gxkosw2IKtHAE5z6i3Mmxt+zCZXPj3WoopJP\/F8uvRNvj6+S\/YPp0eCzRWfuup7g1swVaOgJxn1FsZtrZ9mEwu\/HsthYDRp3jypor8I8j3aBt8ff3L\/o+8hRY3np7\/knlogcZKLuhgC7ZyBOQ8o97KsLXtw2Ry4d9rKQSM7cm5\/jFke7QN\/vLrv0k8jcTChQUMnmQCaKzkagbYgq0cATnPqLcybG37MJlc+PdaCgHDxTLdbeQfQb5Hm+Dr619WLl1Au664OP8F89QCjZVc4MEWbOUIyHlGvZVha9OHyeRAxmspBEzWXUhc7KwdQjJYZnu1Cb6+\/gULeLOjg8ZKrvaCLdjKEZDzjHorw9amD5PJgYzXUggYmaL582oTfBxgZ84djZU5K1tLsLUlZm4PtuasbC3B1paYmb1NH2bmsRxWEDAGcbAJvlrAi\/Uv+WDRWOUzcrUAW1dy+enANp+RqwXYupLLTmfTh8nkQMZr3QSMvnA37XA5VeSqTCFh\/YtdJUVjZcfLxhpsbWjZ2YKtHS8ba7C1oWVuCwFjzio4S9Pg4wA7u9CjsbLjZWMNtja07GzB1o6XjTXY2tAytzXtw8w9lsOybiMw8eLH70PKuh+p1uhMg48D7Owig8bKjpeNNdja0LKzBVs7XjbWYGtDy9zWtA8z91gOy9IImKQLFtU0U1dXF3V2dtaNmGnwP\/2to\/TBHQ9E+cQJvPnhQmOVz8jVAmxdyeWnA9t8Rq4WYOtKLjudaR8m83Y5r6UQMFkH2fH9SGNjYzQ4OEgNDQ1yJDI8mwYfC3jtwoPGyo6XjTXY2tCyswVbO1421mBrQ8vc1rQPM\/dYDstKCBgenRkZGaHGxsa6UDMNvrqBGgfYmYUJjZUZJxcrsHWhZpYGbM04uViBrQu1\/DSmfVi+p3JZlELAZK13KcMJvSbBxwJe+4qNxsqemWkKsDUlZW8HtvbMTFOArSkpOzuTPszOYzmsSyFgGAVPFW3cuJFGR0eppaUlonPgwAHq6emhLVu2UD0veTQJPhbw2ldoNFb2zExTgK0pKXs7sLVnZpoCbE1J2dmZ9GF2HsthXRoBo0TMunXrZpHZvn17XcULZ8Yk+PoJvLiB2qxyo7Ey4+RiBbYu1MzSgK0ZJxcrsHWhlp\/GpA\/L91I+i1IJmPLhOZUjk+BjAa999NBY2TMzTQG2pqTs7cDWnplpCrA1JWVnZ9KH2XkshzUEjEEcTIK\/\/PpvEq+DwQJeA6AvmaCxMmdlawm2tsTM7cHWnJWtJdjaEjOzN+nDzDyVywoCxiAeecHHFQIGEBNM0Fi5cTNJBbYmlNxswNaNm0kqsDWhZG+T14fZeyxHCggYgzjkBR87kAwgQsC4QXJMhY7AEZxBMrA1gORoAraO4HKS5fVhMm+V9zovBYzatr179+6I8Pr166m\/vz+Vdl7wsYDXraKisXLjZpIKbE0oudmArRs3k1Rga0LJ3iavD7P3WI4U81LA8MF4\/LBoMbmuIC\/4V4w9QGP3Ho18YgeSecVGY2XOytYSbG2JmduDrTkrW0uwtSVmZp\/Xh5l5KZ9VaQSMOvNlenp6DqXW1lbRk3h1QZMUorzgYweSW8VGY+XGzSQV2JpQcrMBWzduJqnA1oSSvU1eH2bvsRwpSiFg1JROU1NT5lSOBLKse5jU+\/KCjysE3CKDxsqNm0kqsDWh5GYDtm7cTFKBrQkle5u8PszeYzlSlELAmIgICVw88rJt2zbq6OjIvCySg68\/e\/bsmflx+tkXqeP2qejn9W9eQP\/t1xdIZDVIn1NTU9Tc3Bxk2epdKLCViwDYgq0cAT+eV69ePcfR5OSkH+cl8lIKAaNGYLq7u+ty6i4LGZ66SrvxOku94goB99qMv7bc2eWlBNs8Qu6fg607u7yUYJtHyO1zjMC4cTNOxXch1evWaV5\/09fXR8PDwzP3MOkZzwq+vgNp1wcuppUXYATGNOhorExJ2duBrT0z0xRga0rK3g5s7ZmZpICAMaHkaKOmkCYmJhI9SC\/izRNPWcG\/+rMP01\/8w2NRvrEDya4CoLGy42VjDbY2tOxswdaOl4012NrQMreFgDFnVXpLfdeRyQLirOBjB5J7uNFYubPLSwm2eYTcPwdbd3Z5KcE2j5Db5xAwbtxKmSp+kJ3JIt60BVC4A8k9xGis3NnlpQTbPELun4OtO7u8lGCbR8jtcwgYN25WqXbu3EmbNm2K0mzfvp0OHTpE4+PjmTuErF7gaJwVfGyhdoRKRGis3NnlpQTbPELun4OtO7u8lGCbR8jtcwgYN27GqdROIF5Mu2HDhug8GF77MjAwQPU4H0bPeFrw9R1I\/WvOp\/41S4zLC0MIGMk6gI5Aji7Ygq0cARnPEDAyXCOv+jkwy5Yto97e3kjAtLW1Ud4CW8Fszbg2ETDYgWQfCXQE9sxMU4CtKSl7O7C1Z2aaAmxNSdnZQcDY8bKyhoCxwhWMMRoruVCCLdjKEZDzjHorwxYCRobrjFde\/8LrXfQpJDUa09XVRZ2dncI5SHefFny1AykaRbrxnXXLX1VfjMZKLnJgC7ZyBOQ8o97KsIWAkeE6yytPF61bt27W7zZv3lxX8cKZyRMwixtPj86AwWNHAI2VHS8ba7C1oWVnC7Z2vGyswdaGlrktBIw5q+As04KPHUjFQo3Gqhi\/rNRgC7ZyBOQ8o97KsIWAkeFaCa9JwT987AXiM2D46b50Ed3afWElylKmTKKxkosG2IKtHAE5z6i3MmwhYGS4zvKqnwOjPuDzYHg3Uj2fpOBjC3XxiKCxKs4wzQPYgq0cATnPqLcybCFgZLjOeGXxsmPHDhoZGaHGxsbo92p3UhkX8eojMNhC7VY50Fi5cTNJBbYmlNxswNaNm0kqsDWhZG8DAWPPzDiFvo06PtpS1nNgcAu1cXhTDdFYFWeIERg5hmALtrUnIPNGCBgZrrNGWtThdfqryipgrhh7gMbuPXoq\/9hC7VQ7IGCcsBklAlsjTE5GYOuEzSgR2BphsjaCgLFGZpeAhcrGjRtpdHSUWlpaZgmbMk4h4RZqu\/gmWaOxKs4QowRyDMEWbGtPQOaNEDAyXGcJlYmJidy38P1Id955Z66dT4Ok4OMW6uKEIWCKM0QnK8cQbMG29gRk3ggBI8O1El7jwdcX8K5cuoB2XXFxJcpRtkxCwMhFBGzBVo6AnGfUWxm2EDAyXCvhNUvA4BZq9xCisXJnl5cSbPMIuX8Otu7s8lKCbR4ht88hYNy4WaVKOgemjFcJ6GfA8BUCfJUAHnsCaKzsmZmmAFtTUvZ2YGvPzDQF2JqSsrODgLHjZW1dpXNgePcR70LiB2fAWId6JgEaK3d2eSnBNo+Q++dg684uLyXY5hFy+xwCxo2bUaqqnQODM2CMwpprhMYqF5GzAdg6o8tNCLa5iJwNwNYZXWZCCBgZrpHXqgmYtZ+4n3gaKco7zoBxrhlorJzR5SYE21xEzgZg64wuNyHY5iJyMoCAccJmnqjoFJISQWordkdHBw0ODlJDQ0NiJvT1Nnm28eDjDBjzuGZZorHywzHJC9iCrRwBOc+otzJsIWBkuM7y6rqI98SJEzQwMEDt7e3U2dlJ6uempibi033jj366LwscTptmy2njwccZMH4qAxorPxwhYOQ4gi3Y1paAzNsgYGS4inllMTQ+Pp44ChP\/LMs2LmBwBoy\/kEHA+GMZ9wS2YCtHQM4z6q0MWwgYGa5iXrNESdIIjBq9ScqQHnxdwNz4+2+g\/\/qWJrEyhO4YjZVchMEWbOUIyHlGvZVhCwEjw1XEq1oPk3WH0oEDB6inp4emp6dp+\/btFL8FW8+YHnz9DBgcYlcsfGisivHLSg22YCtHQM4z6q0MWwgYGa7evar1L+w4bRFvfMHw0NBQlI+k9TL8ew6+en6y+K30\/Ir3Rj\/+5bsX0SXn4RA71yBOTU1Rc3Oza3KkyyAAtnLVA2zBVo6AH8+rV6+e42hyctKP8xJ5ecXJkydPlig\/hbJiIl7iC375hTwa09fXR8PDwzM3YaeNwOAMmEIhmpUYf235Yxn3BLZgK0dAzjPqrQxbjMDIcPXmNW\/nkXpRUQHDJ\/DySbz84BqBYuFDY1WMX1ZqsAVbOQJynlFvZdhCwMhw9eaVp4F4PUvW2S\/qZUlTSFlp9eDjDBhvISM0Vv5YYgRGjiXYgm3tCMi8CQJGhqsXr\/FD7JTT1tZWGhkZiQ6z47Neuru7ZxbrsuDZtm1bZGpzkB0EjJeQRU4gYPyxRCcrxxJswbZ2BGTeBAEjw7USXvXgN155d5TnlUsX0K4rLq5E\/suaSQgYuciALdjKEZDzjHorwxYCRoZrJbyq4OtnwHRfuohu7b6wEvkvaybRWMlFBmzBVo6AnGfUWxm2EDAyXCvhNUnA4AyY4qFDY1WcYZoHsAVbOQJynlFvZdhCwMhwrYRXFXz9EDsefeFRGDzuBNBYubPLSwm2eYTcPwdbd3Z5KcE2j5Db5xAwbtyCSJUkYHZ94GJaecGCIMpXr0KgsZIjD7ZgK0dAzjPqrQxbCBgZrpXwqoKPQ+z8hguNlV+eujewBVs5AnKeUW9l2ELAyHCthFcVfBxi5zdcaKz88oSAkeMJtmBbGwIyb4GAkeFaCa8q+DgDxm+4IGD88kQnK8cTbMG2NgRk3gIBI8O1El4hYGTCBAEjw5W9gi3YyhGQ84x6K8MWAkaGayW8quAvv\/6bxGfB4BA7P2FDY+WHY5IXsAVbOQJynlFvZdhCwMhwrYRXFXycwus3XGis\/PLENIccT7AF29oQkHkLBIwM10p45eB\/7dvfJR6B4QeH2PkJGwSMH44YgZHjCLZgW1sCMm+DgJHhWgmvHPy\/\/tJ9tPYT90PAeIwYBIxHmDFXYAu2cgTkPKPeyrCFgJHhWgmvcQGz\/yNvocWNp1ci72XOJBorueiALdjKEZDzjHorwxYCRoZrJbxy8D++c5z4HBh+cAqvn7ChsfLDEdMcchzBFmxrS0DmbRAwMlwr4ZWDv\/4vvkpDdz0KAeMxYhAwHmFiCkkOJtiCbc0IyLwIAkaGayW8cvB\/6+Ofp7F7j0b5xRSSn7BBwPjhiFECOY5gC7a1JSDzNggYGa6V8MrBv+iqz9A9jxyP1r6wgMFTnAAETHGGaR7AFmzlCMh5Rr2VYQsBI8O1El4hYGTChMZKhit7BVuwlSMg5xn1VoYtBIwM10p45eCf+d5P4xRez9FCY+UZqOYObMFWjoCcZ9RbGbYQMDJcK+GVg3\/8XSNRXnGNgL+QobHyxzLuCWzBVo6AnGfUWxm2EDAyXL15PXbsGPX29tLExETks6OjgwYHB6mhoSHxHXv37qV169ZFn7W2ttLIyAg1NjYm2i656M307G8ORZ91X7qIbu2+0Fu+57MjNFZy0QdbsJUjIOcZ9VaGLQSMDFcvXk+cOEEDAwPU3t5OnZ2dpH5uamqi\/v7+Oe84cOAA9fT00JYtW6itrY127txJ4+PjqYJHFzA4A8ZLyCInaKz8scQIjBxLsAXb2hGQeRMEjAxXMa9ZooQ\/O3jwYKK4ScrQ4jf\/Fv1wZV\/0EY++8CgMnuIEIGCKM0zzALZgK0dAzjPqrQxbCBgZrmJe0wRMfLTGJAPN7\/gDen7FeyNTjMCYEDOzQWNlxsnFCmxdqJmlAVszTi5WYOtCLT8NBEw+o9JYqPUwXV1d0ZSS\/igBs2bNGrrtttuiNTN5a2Cafvt\/0Au\/sjZyc8Y9w\/T1nZ8oTVmrnJGpqSlqbm6uchFKm3ewlQsN2IKtHAE\/nlevXj3H0eTkpB\/nJfLyipMnT54sUX4KZ0UJFHaUtIhXfX748OGZhbtDQ0M0PT2dugZGFzA4hbdwiGYc4K8tfyzjnsAWbOUIyHlGvZVhixEYGa5eveaJF35Z0hQSL+rt6+uj4eFhamlpmZOnRb93Pf1k8Vuj3x+78Z1e8zyfnaGxkos+2IKtHAE5z6i3MmwhYGS4evOat\/NIfxGPuCxZsmRmeokFzA033EBbt25N3Er92j\/8JL149htwjYC3aJ1yhMbKM1DNHdiCrRwBOc+otzJsIWBkuHrzmjcNpL+Iz4Bhe3X2C\/8\/P0lbrvn3EDDewjTLERorGa4Qh3JcwRZsZQnIeIeAkeHqxWv8EDvlVC3O5WRecsQAAAzMSURBVMPs+JyY7u7u6NwXfvSD7PIOvTv7j\/6GfvbvzsYpvF6i9bITCBjPQDECIwcUbMG2JgRkXgIBI8O1El4br7w7yieuEfAbLggYvzx1b2ALtnIE5Dyj3sqwhYCR4VoJr0rA4BoBv+FCY+WXJwSMHE+wBdvaEJB5CwSMDNdKeFUCpn\/N+dS\/Zkkl8lyFTELAyEUJbMFWjoCcZ9RbGbYQMDJcK+EVAkYmTGisZLiyV7AFWzkCcp5Rb2XYQsDIcK2EVyVgcA+S33ChsfLLE9MccjzBFmxrQ0DmLRAwMlwr4VUJGNyD5DdcEDB+eaKTleMJtmBbGwIyb4GAkeFaCa9KwOAaAb\/hgoDxyxOdrBxPsAXb2hCQeQsEjAzXSniFgJEJEwSMDFf2CrZgK0dAzjPqrQxbCBgZrpXwqgQM7kHyGy40Vn55YpRAjifYgm1tCMi8BQJGhmslvLKAWdx4OvEUEh5\/BCBg\/LGMewJbsJUjIOcZ9VaGLQSMDNdKeIWAkQkTGisZrphCkuMKtmArS0DGOwSMDNdKeGUBg2sE\/IcKAsY\/U+URbMFWjoCcZ9RbGbYQMDJcK+EVAkYmTGisZLhilECOK9iCrSwBGe8QMDJcK+GVBQzuQfIfKggY\/0wxAiPHFGzBVp6AzBsgYGS4VsIrCxjcg+Q\/VBAw\/pmik5VjCrZgK09A5g0QMDJcK+EVAkYmTBAwMlwxzSHHFWzBVpaAjHcIGBmulfD62j\/8JN30wXdF00h4\/BGAgPHHMu4JbMFWjoCcZ9RbGbYQMDJcK+E11ODXGz4aK7kIgC3YyhGQ84x6K8M21D7sFSdPnjwpgywcr6EGv94RQmMlFwGwBVs5AnKeUW9l2Ibah0HAGNSXUINvUHRREzRWcnjBFmzlCMh5Rr2VYRtqHxaMgDl27Bj19vbSxMREVAM6OjpocHCQGhoaMmvEgQMHqK+vj4aHh6mlpSXRNtTgy3xVzL2isTJnZWsJtrbEzO3B1pyVrSXY2hIzsw+1DwtCwJw4cYIGBgaovb2dOjs7Sf3c1NRE\/f39qRFWdvv27aPR0VEIGLPvgjerUL9U3gAVcAS2BeDlJAVbsJUjIOM51DobhIBJCvnOnTtpfHw8cxRm7969NDQ0FCXHCIzMFyfLa6hfqtqTnPtGsJWLAtiCrRwBGc+h1tl5K2B4yunaa6+l7u7uSMRAwMh8cSBgas+V3xhqg1UfmrPfCrZyUQBbGbahcg1SwKj1MF1dXdGUUtoIDf9+xYoVRmtgZKoVvIIACIAACICAPIHJyUn5l9T4DcEJGLWuhTmmLeLlhbu33347XX311TQ1NZUrYGocE7wOBEAABEAABEAgh0BQAsZEvDAPnjJatWoVtbW1kckuJNQiEAABEAABEACBchEIRsCY7jyKb7fWw7F9+\/ZI1OABARAAARAAARAoN4FgBAyPqkxPTxud\/aKHBCMw5a6gyB0IgAAIgAAIJBEIQsCkjaq0trbSyMhIdJgdnxPDO47iIywQMPhigAAIgAAIgED1CAQhYKqHHTkGARAAARAAARAoQgACpgg9pAUBEAABEAABEKgLAQiYDOy8rmbbtm2RBRb4utVPnqLr6emJ1ifl3U+l8+ZrILKud3DLTVipbNiqksev3QiLiJ\/S2HCNT1+jnciOgQ1b3RbtQbG6rb73Scsoinmub2oImBT+6poBXkPz0EMPRVuv+f8bGxvrG7EKvV3vLNeuXTvrvqp4MeJXP\/DPO3bsAPOUeNuw1V0w102bNtHmzZtTD3msUBXznlUbrvGdj1hPlx0OG7ZKGPJddrxuEe2Be1VX3Hfv3h3cH+IQMCn1Qt2RxF+gUNWr+1fCLGW8QWdRODY2ZrRTDJ1B\/l+y+i3qJmy5U7jqqqvo+PHjlHVKtVl0w7SyqbNse8MNN9DWrVvxh41BdbBlq9dvtAcGgBNM1CjWJZdcQocPH44uNw7pqBAImISgp91urW67dqtK8y+VPorFI1fxn7OIoMHKri8ubFmUX3rppfS5z31u5ub2+Vcr\/XE1uTAWfF8mYFNnk0Zg8i7nBeu5BI4cORL9knfi9vb2QsDMh0qSNOLCjf+SJUsw7G5RAeKjAjZ\/sbqe62ORvUqb2rJV12dceeWV0SWmEOPJ4bfhygLm4MGDkSOslcv\/OtmwZW\/61Mf69eujzhePG4G4IHTzUr5UGIHJGIHRFzxBwNhXXtsGS72BO4abb74Zi3gzkNuw5Y7g4x\/\/OL3nPe+h5ubmzLVI9lEOK4UNV7WeSC3c5bQbN25EvU2pEjZs1dTHli1boikPm9HbsGqkn9JAwPjhWAkvmELyEyabIWOIFzvmNmzZ9utf\/3r0Fyx2IWVztuEan0ICW7C1+xbXzhoCpnasS\/EmfcQFi3jdQhKfMspbaIqdBuacbdjq29P1N2BYfi5vG67x+ox2Irv+2rCFODRvC0wsIWBMKAVkg23UxYNps20Sw+92vG3Y6p4xSpDN2YZrvFPANIc\/tklTSJies2sjdGsIGHd2lU2Jg+yKhy7r4Cq1CJKnNtJGCXAwWHoMTNlCwNjVYxuu+kF2OGwtn7MNWxaE69ati5yCbT7bLAsImGL8kBoEQAAEQAAEQAAEvBHALiRvKOEIBEAABEAABECgVgQgYGpFGu8BARAAARAAARDwRgACxhtKOAIBEAABEAABEKgVAQiYWpHGe0AABEAABEAABLwRgIDxhhKOQAAEQAAEQAAEakUAAqZWpPEeEAABEAABEAABbwQgYLyhhCMQ8E\/gkUceoYULFxLf5m3y8HkPzzzzDC1dutTE3MlGndnT2tpKIyMjxnmryg3jqny1Onuk1u9zCjoSgUAJCUDAlDAoyBIIMAHbk11rcVhVERFSJG0tawQLCn5qeftxVdjUMg54FwjkEYCAySOEz0GgTgTKKGBs86Sjq0onDQFTpwqP14KAJQEIGEtgMAcBnwT0o+jZr5qWeeihh2aOUeffqysV4lcuqGmOs846i3p7e2liYiLKnrqoUd3ts3v37uj3JtM++jv0aRS++mHTpk0zxd+8eTN1dnbOwZFmpwTM2rVr6brrrovSxadp9OPjlWP1HmZ11VVX0dvf\/vYovSrL008\/TT09PTQ9PR0l+ehHP0q7du2i4eFhamlpiX6XVqakWMYFDP\/83HPPRf8pjlkXYcYvIuR3JP2uiuLOZ92HLxAoSgACpihBpAcBRwJJFyvqnWd8tCPthl5+\/eDgILE\/FjE89dHW1kZKHHV1dc0Ijawbv1V+lL+Ghoao47355ptpdHQ0EgN5IzBxe\/1SPhZZLDQuueSSKL\/K\/44dO6K1NCxE+vr6ZgkP3Z8SaYsXL55JHy+j+vnJJ5+M8tzc3EwDAwORUFJTQnkXhyYJmG3btpEupJizzlWvAhAwjl8IJAMBSwIQMJbAYA4CvggkCQzdd55YiP9lHxcwnH5sbGyms2f7rNuok6Z44vZZecq76Tp+wzDnJ29aSf9cCZi4IBsfH59VRl2g8DtuuOEG2rp166zFxlnTREkChkd3lOhin1kcIGB8fUPgBwSyCUDAoIaAQB0J6NMt8emdtE4yPs3S0dGROAITn8rRi5k0\/ZP2Pv3W8KyOO28RcZJYSfpdfFotPk2mRpi4PElCRPfJozrqRuN4mNOmgZIEDKfVF\/VmCS8ImDp+ofDqeUUAAmZehRuFLSsBvdPW18FwZ6q2KitBEl+XokYg4iMwnDY+cpBV\/jRxkjWtpfsrKmDYl1rLogRW0giMjYC57777SE1RmW5Fh4Ap67cE+QKB2QQgYFAjQKBEBHQRoEYYWMDwehFey9He3j5r4awuUuICJmu9S1KRazGFFF\/jor+TxUbWdJCaQtIFTNJohz6FxCMwGzdunFnDYxJqiSmkPDGZN5Vmkm\/YgMB8IwABM98ijvKWhkDSGhh9FERf1Jq0GFWNyKgpJC6YLnKUf17Qq6Y\/ktahKCC+FvHqIx76upgVK1bMWaQbFzB6WpVXzh8vyE0SMKaLeNmHWsOSt\/ao6CJeNcWndo6pcuiLl+OVEAKmNF9LZKRCBCBgKhQsZDU8AqpzU1uA9ekhfQs0T6lcdtlls7ZKs3C5\/PLL6ZprrpkZYYiLGjUqo7ZXM0HVsabRzNpybLqwOGm7tckamPi7t2zZEq1z4YW7qvz6CAyXIc4wvo06vpWc06RtAVejXvyvEn1J26j19Enl0tcfcZyWL19O+\/fvj0RUXGiqMsRHp8Kr7SgRCPglAAHjlye8gQAI1JkAC4qknUem2TJZA2Pqy9QOIzCmpGAHAi8TgIBBbQABEKgsgfg6HzXaop\/7Yls4CBhbYrAHgfoQgICpD3e8FQRAwBOB+OnEWafkmrwyfrniHXfcESWTuhsJlzmaRAU2IDCXwP8HbWE8g10v1boAAAAASUVORK5CYII=","height":337,"width":560}}
%---
