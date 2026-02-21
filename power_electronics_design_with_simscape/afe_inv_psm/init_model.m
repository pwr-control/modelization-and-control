clear;
[model, options] = init_environment('afe_inv_psm');

CTRPIFF_CLIP_RELEASE = 0.001;

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

hwdata.afe = three_phase_afe_hwdata(application_voltage, afe_pwr_nom, fPWM_AFE); %[output:8c83c3d9]
hwdata.inv = three_phase_inverter_hwdata(application_voltage, inv_pwr_nom, fPWM_INV); %[output:8adacb88]
hwdata.dab = single_phase_dab_hwdata(application_voltage, dab_pwr_nom, fPWM_DAB, fres_dab); %[output:89f5ee0a]
hwdata.cllc = single_phase_cllc_hwdata(application_voltage, dab_pwr_nom, fPWM_CLLC, fres_cllc); %[output:5fbbf7c6]

%[text] ### Sensors endscale, and quantization
adc_quantization = 1/2^11;
adc12_quantization = adc_quantization;
adc16_quantization = 1/2^15;

Imax_adc = 1049.835;
CurrentQuantization = Imax_adc/2^11;

Umax_adc = 1500;
VoltageQuantization = Umax_adc/2^11;
%[text] ## 
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


%[text] #### DClink Lstray model (partial loop inductance)
parasitic_dclink_data; %[output:7fb84835]
%%
%[text] ### INVERTER Settings and Initialization
%[text] ### Mode of operation
motor_torque_mode = 1 - use_motor_speed_control_mode; % system uses torque curve for wind application
time_start_motor_control = 0.25;
%[text] ### IM Machine settings
im = im_calculus();
%[text] ### PSM Machine settings
psm = psm_calculus();
n_sys = psm.number_of_systems;
run('n_sys_generic_1M5W_torque_curve');
torque_overload_factor = 1;

% load
b = psm.load_friction_m;
% external_load_inertia = 6*psm.Jm_m;
external_load_inertia = 1;

%[text] ### EKF BEMF observer
kalman_psm;
k_kalman = 0;
%[text] ### Motor Voltage to Udc Scaling
u_psm_scale = 2/3*hwdata.inv.udc_nom/psm.ubez;
u_im_scale = 2/3*hwdata.inv.udc_nom/im.ubez;
%[text] ### 
%[text] ### Simulation parameters: speed reference, load torque in motor mode
% rpm_sim = 3000;
rpm_sim = 17.8;
% rpm_sim = 15.2;
omega_m_sim = psm.omega_m_bez;
omega_sim = omega_m_sim*psm.number_poles/2;
tau_load_sim = psm.tau_bez/5; %N*m
b_square = 0;

%[text] ### 
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
if use_mosfet_thermal_model
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'off');
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
    set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
else
    if use_thermal_model
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'off');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'off');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'on');
    else
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_mosfet_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/afe/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_igbt_based_with_thermal_model', 'Commented', 'on');
        set_param('afe_inv_psm/afe_abc_inv_psm_mod1/inverter/inverter/three_phase_inverter_ideal_switch_based_model', 'Commented', 'off');
    end
end

if use_torque_curve
    set_param('afe_inv_psm/fixed_speed_setting', 'Commented', 'off');
    set_param('afe_inv_psm/motor_load_setting', 'Commented', 'on');
else
    set_param('afe_inv_psm/fixed_speed_setting', 'Commented', 'on');
    set_param('afe_inv_psm/motor_load_setting', 'Commented', 'off');
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright"}
%---
%[output:8c83c3d9]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V_250kW\nNominal Voltage: 690 V | Nominal Current: 270 A\nCurrent Normalization Data: 381.84 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:8adacb88]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:89f5ee0a]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:5fbbf7c6]
%   data: {"dataType":"text","outputData":{"text":"Single Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.880000e-05 [H] | Internal Tank Cs: 250 [F]\n---------------------------\n","truncated":false}}
%---
%[output:7fb84835]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAfcAAAEvCAYAAABYGIhlAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQl0VdXV3kCAMA+GyiSDKBhRwJYo\/UkFwcJf0GgQY4ASEESUQaz6GxmUgAwGpYgiICgIS2TQQhWLRQaDJksqomILiAhGC4KEeZAZ\/rVPuM+XkOTd+d5933fWYoWXnPH7zrnf2+ees3epixcvXiQkIAAEgAAQAAJAIDAIlIK4B4ZLDAQIAAEgAASAgEIA4o6JAASAABAAAkAgYAhA3ANGKIYDBIAAEAACQADijjkABIAAEAACQCBgCEDcA0YohgMEgAAQAAJAAOKOOQAEgAAQAAJAIGAIQNwDRiiGAwSAABAAAkAA4o45AASAABAAAkAgYAhA3ANGKIYDBIAAEAACQADijjkABIAAEAACQCBgCEDcA0YohgMEgAAQAAJAAOKOOQAEgAAQAAJAIGAIQNwDRiiGIxOB7du30+DBg6lChQpUunRpOnjwILVo0YKeeeYZqlWrlq5BLV68mBo2bEht2rSJmH\/9+vU0cOBAatSoUai91NRU6tu3r\/o8ffp0uvvuu6lx48YR60IGIAAE\/IcAxN1\/nKBHUYgAi\/u8efNo5MiRSuAvXLhAL730khLr5ORkXYgYFfeFCxfSc889p9o7ffo0zZkzh86cOaO+ZMTExOhqs7hMHGyS\/\/EXBSQgAATcRwDi7j7maBEIXIZAYXFnsc3MzKR27dpR27Ztaf78+bRo0SJVji3stLQ09X\/+QvDWW28p675JkyZ055130i233EIrVqyg1157jc6dO0ddunRRFjmLuJbYcg8Xd\/79nj17aPTo0TRixAhVtk+fPlSnTh31JeOTTz5Rwt+rVy\/V9qlTp2jKlCmUlZWldhi47vvvv5+++OIL2rJlC+Xm5tKjjz6q2p8wYYL68sB95PqrV6+u2rjhhhvogw8+oKuvvpo6depEb7zxBh05coTGjx+vxoAEBICAeQQg7uaxQ0kgYBsCLO4sjj\/99FOozq5du9LEiRNp48aNtG7dOiWInKZOnUrNmzdXgsq\/T09Pp7Nnz9Lw4cOV+JYvX179fsiQIVSmTBn1xSAuLo64vpLE\/cSJE0pYe\/ToQbwLwOJ+4MABJdT33XcfHTt2TFn6Q4cOpVWrVql2UlJS6Oeff6ann36annzySSXu2dnZqt+lSpWimTNnUs+ePdWXhA8\/\/JB27dqltvsffvhhGjRoEN16660qb8WKFemRRx6hL7\/8kv75z3+qMVndPbCNHFQEBAQiAHEXSBq6HDwEitqWZ8uahTU2NlZZsomJiWrgbHWzeHNiq177PVv2\/A6dLedx48YVAInfr7NgGhV33g3YsWOH+rdp0yb6+OOPadKkSbRkyRL1RaJp06Zq+33atGn0v\/\/7v0rcOfGXAU78hWDz5s307bffKuv\/mmuuoQEDBtCYMWOUFV+zZk21Q6GNozAOwWMaIwIC7iAAcXcHZ7QCBEpEoChR037HFnrHjh1DB+XYMv7Xv\/6ltryLEvfPP\/9cbYFrAsvb6efPn9e1Lc9b6PwlgC1uttz\/\/e9\/q39szdeoUUPtGnC9mmV\/7bXXqn7w75OSkgqIe15eHo0dO1a9FmjdujVt27aNcnJyihR3fv3ABwEh7lgoQMAeBCDu9uCIWoCAJQQKixpbw2wdf\/fdd\/Tb3\/5WWePDhg1TbbAAsxiePHmSPvvsM7Vdz\/9\/\/PHHqV+\/fkpsly1bpqzjsmXLKqs63PLXrP\/CB+o4H2+P8\/t53p5ncV+6dGnoCwT3gQ\/8seW+cuVK9QWCt+V3796t+sBb8+GWO49p7ty5qgxvsb\/44ouq\/0VZ7hB3S9MHhYHAZQhA3DEpgIAPECh8FY4PrF133XVKNNliLupAHVvjs2fPVkLO2+f8XvtPf\/qTspL5iwEfUOOkHcALf4dd0lU4LqOJOx+E43fp\/P6c35XzmYBq1aqpLXne+t+6davamq9cuTL179+\/gLhz2ZdffpmWL19OzZo1U9v23C\/eGeC+hW\/LQ9x9MAnRhUAhAHEPFJ0YDBBwB4HVq1dTgwYNlLDv37+fJk+erHYO+OAeEhAAAt4jAHH3ngP0AAiIQ2Dnzp1q25\/fq\/Oped5hSEhIEDcOdBgIBBUBiHtQmcW4gAAQAAJAIGoRgLhHLfUYOBAAAkAACAQVAYh7UJnFuIAAEAACQCBqEYC4Ry31GDgQAAJAAAgEFQGIe1CZxbiAABAAAkAgahGAuEct9Rg4EAACQAAIBBUBiHtQmcW4gAAQAAJAIGoRgLhHLfUYOBAAAkAACAQVAYh7UJnFuIAAEAACQCBqEYC4Ry31GDgQAAJAAAgEFQGIe1CZxbiAABAAAkAgahGAuEct9Rg4EAACQAAIBBUBiHtQmcW4gAAQAAJAIGoRgLhHLfUYOBAAAkAACAQVAYh7UJnFuIAAEAACQCBqEYC4Ry31GDgQAAJAAAgEFQGIe1CZxbiAABAAAkAgahGAuEct9Rg4EAACQAAIBBUBiHtQmcW4gAAQAAJAIGoRgLhHLfUYOBAAAkAguAicy8ulmFqNgjvACCODuBPR1VdfHbUTAAMHAkAACAQFgdrlzlHLKqeo0xXH6Mry52jTsVh6PreWqeHt3LnTVDm\/FIK4XxJ3yUTylxPJ\/ffLYrDaD\/BgFUHr5cGBdQztqMFtHthKP\/bRPDqW9YbqfpX2fanKbX1MW+5u998OzAvXAXGHuDsxr6KyziA8EKQTBw78waBbPGiifujtDCXkLOo1UkZbBsGt\/lvuaAkVQNwDIO7ff\/89NW7c2Ml5grp1IAAedIDkcBZw4DDAOqt3modTm7Po4NtjiH+yqLOgs7DblSDudiHpcT3SiXR6IXlMj5jmwYP3VIED7zngHjjBA1vpJzdn0bGseXRuXy5VaN6eqrTvQ7HN29s+aOmawIDAcoflbvvCiNYKnXigRSuWZscNDswiZ285O3koauvdyvt0PSOFuOtBSUAe6UTauZAE0OXbLoIH76kBB95zYJfl7tT7dD0ISdeEqLHcL168SO+\/\/z5NnTqVypcvTwMGDKCkpCQqXbq04lk6kXig6VmuzucBD85jHKkFcBAJIXf+boUHPvHOW+9OvU\/Xg4B0TYgacd+6dSvNnj2bxowZQ6VKlaKMjAzq3bs3tWzZEuKuZ6Yjjy4ErDzQdDWATBERAAcRIXIlg1EetPfph5aMUf1z8n26HgAg7npQ8kGexYsXE1vvqampqjeLFi1SP7XP0ok0upB8QEkguwAenKE1c2WuqvjHgyfVz\/8ePJX\/+dCln5c+l9R6g5qx1COhDqV3jl6PZc6wU3SteteCF+\/T9eAgXROixnLPzMykhIQE6tChg+I1OzubcnJyKD09XX2Oe+jtEN+1a9fWw72v8pw7d55iYsr4qk\/R2Jmg8vDT0XOe0Vm3akyo7TpV8v8f\/ru6VWKozqU8d8ZXpl27dlH9+vUL9Hf51uO05+g5+nz3Kdpz7BzxeAbeXJ3uiK9coC7PBhnAhoviocAwD+2ii6umEn3+N6Ia9Yla30Ol\/jjMcyQ6duwY6oN0x2BRcVqexb1t27aUmJhYpLhL\/5am91uy5ysn4B0AD94TrIcD3glYuGEP\/XjwFCU2qU7vDb7J+44HrAdF8RB+lY3fp\/MVtroZH\/ly5NI1wbeW+8mTJ+nTTz+lJUuW0M8\/\/6zIv\/LKKyklJYV+\/\/vfU4UKFQxNiMLb8PyZD9YlJyereqQTqeeBZggwZDaFAHgwBZuthYxwwOKeNP1L1f601HhKvKa6rX2J5srCeSjsGtbr9+l6eJGuCb4Td34v\/tVXX9HMmTPptttuozZt2lDVqlUVFyz4X3zxBb377rvUq1cv9Xe9iQ\/UzZ8\/n0aOHKmKjB8\/ntLS0ig+Ph7irhdE5IuIgBFhiVgZMphCwCgHLPALN+ylzJXfw4o3hXjRhZiHqyqXUv7ew13DOn0\/3a4hQNztQvJSPcePH6cdO3bQjTfeGLqmVriJCxcu0LZt20LCrKcL2lU4\/tLA\/x88eDB16dJFnZznJJ1Iow80PZghj3EEwINxzOwuYZaD7O8OKyse2\/TWGWFL\/cc5T6j36Xb6e7feM\/01SNcE31nu3CEW7x9++IGqV69OFStWpLVr19I333xD1113nToQx9vpdifpRJp9oNmNY7TXBx68nwFWOAjfpv9q1O+9H4ygHhR+n84H5OreMcQR17BuwCJdE3wn7ufOnaNXXnmF3nvvPSpTpowSdBb4Tp06qXfwV1xxBQ0cODBkcdtFsnQirTzQ7MIQ9TjjTxu4GkPA6lpggR+ycKu6Zof38JGxL+59+p6KDUUHs5KuCb4T90OHDtELL7xATzzxBJ06dUo5nXn22WepVq1axFv2f\/3rX2no0KFUo0aNyLPOQA7pRFp9oBmACllLQAA8eD897OIg6ZUvKXvHYXpv0E04aFcErZFcw9rFg1czSrom+E7cDx48qDzJPfLII+oAHYv76NGjqWbNmurzSy+9pFzH8mc7k3QipS8kO7n0si7w4CX6+W3bycHghVvVYTsI\/K+8sqizFzl2EVtSqFU7efBiVknXBIj7pVkjnUjpC8mLxetEm+DBCVSN1Wk3B3wnnk\/Sv9IjnnokyHNwZQy9onObCbVqNw92jMNIHdI1wZfiPnz4cGrSpAnx+\/esrCxq3769utd+9uxZdZJ+4sSJsNwLzVLpC8nIovNzXvDgPTtOcMDWO1vx0Sbwhd+nV2nfl\/ReZXOCBzdnF8TdZrRPnz5NX3\/9NfHPohKflG\/RooXtJ+alEyl9Idk8jTyrDjx4Bn2oYac40K7KpXduHHj\/9JHep+th2Ske9LRtRx7pmuA7y52vwR05ckTdRS8q8b30atWqFXsH3iyp0omUvpDM8ua3cuDBe0ac5EATeN6eZys+aIldwh58e0wo1GqD6d+bHqKTPJjulIGC0jXBd+LOp+WfeeYZFfzh6NGjtH\/\/fmrUKD+K04EDB6h79+70wAMPUOXKlQ3QFDmrdCKlL6TIDMnIAR6858lpDoIm8Gbep+th2Wke9PTBSh7pmuA7cdfI4G35GTNm0L333kv16tVTv2bBf+edd+jhhx\/GtnyhWSt9IVlZhH4qCx68Z8MNDoLgzc7pUKtu8ODkbIO4O4QuX4ljV7HDhg2jSpUqqVZOnDhBU6dOpYceeggH6iDuDs08a9VKf6BZG70\/SrvFgWSB56ts4f7ea6SMtp08t3iwveOXKoS4O4TsmTNnaMKECcrDEfuA56S5oX3qqadguUPcHZp51qqV\/kCzNnp\/lHaTA\/Zm12rcpyL80fO99GNZ80Lv01nQ+fS7U8lNHpwYA8TdCVQv1cmWOodmff\/999XpeXZB269fv1CUODublk6k9IVkJ5de1gUevEQ\/v223OdD80TeoEeu7uPBehlp1mwe7Z550TWA8Sl0s7mi63WjpqI9F\/NixYxQXF1dibj5oV1yeTZs2Kauf8\/D7+oyMDLUDwF8SeFufr9Oxl7ukpKTQqXvpREpfSDqmhogs4MF7mrzgINwfvR8CzhR+n27l1LtZRr3gwWxfiyonXRN8J+7suIaDxnBM95SUFOXMhh3YcGL3s1u2bFHx3Fu1akXdunW7jBP+YsAx2zmka7NmzWjdunX04YcfUmpqKs2dO1e5s+XrdCz4vXv3ppYtW6o6pBMpfSHZuSi9rAs8eIm+N5a7NmI\/BJwp7BqWt92deJ+uh2Xpa0G6JvhO3LVJk5eXR4sXL6Zly5apbTZOHCHu7rvvVtfhivMt\/+OPP9Lbb79NgwYNUl8Kdu\/erQLP3HzzzSq6HIs8J97u56R9lk6k9IWk52EhIQ948J4lrznggDNuRpQrHGpVeZFr38fzUKte82B1JkrXBN+Ku1ViuDzvAixYsEC5rI2NjaU2bdqoePCcsrOzKScnh9LT02G52wE26lAISH+gBYFGP3DgRkS5ot6n1xo81zcU+oEHK2BA3K2gZ0NZvjLXv39\/4vfsHOddE2u24MeNG6dCxXL42FmzZlHbtm0pMTGxWHHXurNmzRobeuZuFewDoH79+u42itYuQwA8eD8p\/MJBxur9tHzrcZrVrTb9rl6sfcAc2kUXP\/8b0aqpRDXqE7W+h0r9cZh99dtUk194MDqcjh07hors3LnTaHFf5ffVgTqryPDZQD44xxY7Cz2\/Uy9duvRl2\/C8Lc8H65KTk2G5WwUd5UMISLdWgkClnziwM2Ss21fZrM4FP\/FgZiyw3M2g5mCZvXv30tixY9WhOs2zHTe3detWmj9\/vvo9p\/Hjx1NaWhrFx+f7h5ZOpPSF5OCUcLVq8OAq3EU25jcOrESUK8o1rJ+23kti2288GJ2Z0jWBx+tby523REaNGqVOybPjGj5F\/+ijj6qt9uLShg0b6MEHH6QGDRqErrnxdjWfkud37Oz1jq17Pk3PznH45DzE3ei0R\/7iEJD+QAsCs37kQBN4IwFnCnuR0xtq1S8c+pEHI9hA3I2gZSAv33fPzMyk22+\/XZ2af\/rpp2nz5s0qvjs81F0OpPSFZGBq+DorePCeHr9yoMddrZ+uslll0q886B0XxF0vUgbz8UG52bNn05\/\/\/GeaNGkSjR49Wl1te+mll5QDmuKuwhlsJpRdOpHSF5JZ3vxWDjx4z4ifOShK4J2KyuY1E37mQQ820jXBt9vybLlPmzZNWe5z5sxR4s7v09944w0VEhYhXwtOT+kLSc9ik5AHPHjPkt850PzR39voND1X+1+OB3DxihG\/8xAJF4h7JIQs\/J3fufMBuP\/85z\/KUufT7Sz4TZs2tVBr0UWlEyl9IdlOqEcVggePgA9r1u8c8Kn3LUtfpVp71tNPMbWVw5mWA5\/zHjibe+B3HiINV7om+NZy14Dnw2\/sUvb8+fNUrVq10CG5SMQY\/bt0IqUvJKN8+TU\/ePCeGT9yUFwAl5Ssaq56s3OTHT\/yYGT80jXBd+J+6NAhte3ODhCKSnzyna+61ahRwwhPEfNKJ1L6QopIkJAM4MF7ovzEgZ4Dcm54s\/OCFT\/xYGb80jXBd+J+4cIFOnLkiLLU+eobb8ezZzlOfFKe3cjecccdoStsZkgrqox0IqUvJLt49Loe8OA1A967ANas9JNbslTs9Njm7aluxkclAmOnsxvvGcjvgfS1IF0TfCfu2sRkC55dxg4ZMoQqVaqkfs3x3fmdO99jh+VecAlLX0h+eSBZ7Qd4sIqg9fJeccBCfixrHvE79ZhajUgFcLmtj\/q\/npS5MpcyV35Pr\/SIJ74PLz15xYNduEHc7UKyUD3Hjx9X2+8PPPBA6AAdb9XzVTh2bFO1alVbW5ZOpPSFZCuZHlYGHjwE\/1LTbnIQ\/i6d\/89WOh+QY2E3k8w4uzHTjhtl3OTBifFI1wTfWu7cMfY2x1fgypYtG7Lc2W3sLbfcYjuX0omUvpBsJ9SjCsGDR8CHNes0B4VDrGpWul1x0\/U4u\/Ee5cg9cJqHyD2wlkO6Jvha3LlzHLZ1\/\/79xO\/i4+LiqFy5ctYYK6a0dCKlLyRHSPWgUvDgAeiFmnSCg6IEvULz9uSUn\/cgCLwTPLg5u6Rrgm\/FvbhT8zgtX\/T0lr6Q3Fy0TrYFHpxEV1\/ddnHgtqAXHp3m7KZBzVj6atTv9Q3eR7ns4sGrIUHcHUJeOzXP99w5HT16lNauXUsxMTHUq1cvKlOmjK0tSydS+kKylUwPKwMPHoJ\/qWmzHLCYn9uXSyc3ryPtpDtvubOFHtu8nen36FYQYYEfsnCrugsvTeDN8mAFLzvLStcE31ruRZHEW\/Qcp51jsONAXUGEpC8kOxell3WBBy\/Rz2\/bCAd8wj1czLm8Juh8MI4PyPkh8V14FvhpqfGUeE11P3QpYh+M8BCxMg8yQNxdBH337t00b948dT1Oj7hv27aNnnvuOZo8ebK6Ovf+++\/T1KlTlRtbDj6TlJQU8ngnnUjpC8nFaeRoU+DBUXh1VV4SB3xN7dTmdXQ2L1fdQQ8Xc6fen+vqtI5M0pzdSF8L0jXBt5Y7R4Xr378\/bdq0KTTtr7vuusvisBe3JrR39nl5eTR9+nT6+eefVZQ5juvOMdwzMjKod+\/e1LJlS1WFdCKlLyQdzzYRWcCD9zQxB3E\/rAuJOG+185a7JuQxv2lEZWs18myr3QpCmrObg3+9zUo1rpSVvhaka4JvxZ3fuf\/yyy\/KgQ2LMSd+\/86ObCpWrFiij3nevp85cyY1bNiQVq9era7TrVq1SpVPTU1VdS1atEj91D5LJ1L6QnLlaeNCI+DBfpDZ2tbSuX0\/hISarW9O4eKt5eOtdU3E\/W6RG0VMuwuf3rkxpXfW5yDHaBt25Je+FqRrgi\/F\/eTJkyq86\/z585UTG47jzond0rL1\/cQTT5QYzz07O5t4S\/6uu+6iZ599Vok7l0tISKAOHTqoujhPTk4Opaenq8\/L\/je\/DU4tW7WyY267WgeHyOXXDUjeIiCWh4NFx3KwDc1DNtRfo35+d2pe+lmjXv7nGvWpFP+t9T3qIzu74ls1QU4bd5+iB5fupTvjK1PG7XG+HKpUHjp27BjCkyOTSk6lLmpH0n0yCnZew5b2xo0blcMazYkNd6958+ZKoPnUPKfw7fuBAwdSnz596PXXX6dHHnmEzp49q7bhNXFnH\/WJiYmqXGFxn9GhCnW\/p7tPEDDejWPHj1GVylWMF0QJWxGQyoNeF6l6wLLLmYuetorKI91i1Dtu7S68X7fopfMAy13vTDSYjy2g3NxcatKkSUjI9VSxfv166tmzZ4GsdevWpfvuu49q1apVYFueLV0+ec9JOpHSF5IebiXkAQ\/esxRNHPj5Lrx0HqRrAq9EX1nufBBu8eLFakt9woQJl4V+NeLEhq16zXLnA3W8zT9y5Ej19GE3tmlpaRQfHw9x9\/55HJgeSH+gBYGIaOMg\/C68n67KSecB4u7jp0G4uGtX4figHb+FGDx4MHXp0iV0WE86kdIXko+nkaGugQdDcDmSORo58KOzG+k8SNcE31nu4audxfmDDz6gffv2hX5drVo1SklJocqVK9v6YJBOpPSFZCuZHlYGHjwE\/1LT0cyBn+7CS+dBuib4Vtw55CtvofM99KZNm4aeGPyevEWLFrafDJdOpPSF5L0k2NMD8GAPjlZqiXYOtLvwXseFl86DdE3wrbiz1c7X1\/jUu3YVzsqCj1RWOpHSF1IkfqT8HTx4zxQ4IPLDXXjpPEjXBN+KOzui4QNwbdq0oeuvv97xJ4Z0IqUvJMcJdqkB8OAS0CU0Aw7ywfE6bKx0HqRrgm\/FnU\/Ns4OZrKwsuuqqq0K+5I2cljfymJFOpPSFZIQrP+cFD96zAw5+5cBLgZfOg3RN8K24u\/2IkE6k9IXkNt9OtQcenEJWf73goCBWXt2Fl86DdE3wrbjzgbolS5Yol7Phib3Vsae5G2+80ZBzm0iPBulESl9IkfiR8nfw4D1T4OByDry4Cy+dB+ma4FtxZw917EaWw7x26tSJzp8\/r0K21qtXT91Tj42NVaFf7UrSiZS+kOzi0et6wIPXDBiL5+59b93tgZtx4aWvBema4FtxP3r0KM2YMUM5m9HutPPvpk2bFvIf\/8wzz9i2MqQTKX0h2UakxxWBB48JIIh7JAbcugsvfS1I1wRfi\/uUKVPUVTj2LseJD9m98MIL9NBDD9HcuXMJ4v7rMpa+kCI9kKT8HTx4zxQ4iMxB5spcylz5PTl5F146DxD3yPPIdI6lS5fSwoUL1XW40qVL01dffUV33HEHlSlThn766Sdsy4chK30hmZ4kPisIHrwnBBzo40C7C98jobYSebuTdB4g7nbPiEL15eXl0fbt2+nChQvqfXtcXJz6f6VKlXCgDuLu8OwzXr30B5rxEfuvBDjQz4mTV+Wk8wBx1z+PDOfkycGH6NihDSc+ZPfdd9\/R888\/H9qqN1xpMQWkEyl9IdnFo9f1gAevGcA7d6MMOCXw0teCdE3geeCrkK\/axDx58qQKy3rzzTfT559\/Tr\/97W\/pv\/\/9L7Vq1Yr+8Ic\/lDh\/2dofN24cbd26lerUqUOjR4+mxo0bqy8KU6dOVX7pBwwYQElJSWq7n5N0IqUvJKMPJL\/mBw\/eMwMOjHPAV+WSpn+pCn416vfGKyiihHQepGuCb8U93Lf8qlWrlPjytjz7mx80aFCxUeHYyp80aRK1bt2a\/vjHP9LHH39M69evp65du9KcOXNUfPdSpUpRRkYG9e7dWwWmgbjbspZRCU5q+2IOSBcVr0C0+y68dB4g7g7NRHZiw9vvbGHn5ubSL7\/8og7WTZ48mYYNG0Y1a9YssuU9e\/aoPHySvmrVqmpLn8ty6Fi+H5+amqrKLVq0SP3UPksnUvpCcmgauV4teHAd8ssaBAfWOLDrqpx0HqRrgm8td+7Yhg0baO3atdSrVy8aMWIE7dixQ4nxww8\/XOxhOj58x3fhOe77mjVr1Hb+U089RW+++SYlJCRQhw4d1MzPzs6mnJwc5b8elru1hwFK\/4qA9AdaELgEB9ZZ1MLGHvzrbaYrk84DxN009cYKahZ4lSpV1La6lnj7vn\/\/\/rRp0yYaOHCgEu8nnnhCOcDhaHLr1q2jd955h6688kpq3749JSYmFivuWp38pUBa2rVrF3FQHSRvEQAP3uLPrYMDeziY9a\/D9Opnh2ngzdXpwVuqG65UKg8dO3YMjXXnzp2Gx+2nAr46UMeOanhLnSdGUSlSVDi23OfNm0cjR45UceBZ\/Pk9e4sWLdT1ufBteT5Yl5ycDMvdT7NReF+kWyvC4VfdBwf2sWjlLrx0HmC52zePVE0s7vxO\/ezZs5SSkqJOy7NIa4mtdt5y1065F26e39WPHTtWWfPNmjVTW\/vvvvsu3XffffTWW28p0efEJ\/HT0tIoPj7feYN0IqUvJJunkWfVgQfPoA81DA7s5cDsVTnpPEjXBJ4FvrLcuUPspIYDxvDVtWXLlimru3v37ur9ebly5SLOXN5KYWudr8Q1adJEvXOvW7euqm\/mzJnqYB37rO\/SpUtoi186kdIXUkRbAMIDAAAgAElEQVRShWQAD94TBQ7s58CMwEvnQbom+FLcw6cmCzG7mn3vvfdo+fLl6sT8Y489VuxVOLPTWjqR0heSWd78Vg48eM8IOHCGA6NX5aTzIF0TfC\/u4VY8xL34RSt9ITnzOHK\/VvDgPuaFWwQHznFgROCl8wBxd2AeWd2WN9Ml6URKX0hmOPNjGfDgPSvgwHkO9NyFl86DdE3wneWuHag7cOAAPfDAA4YP1Jmd1tKJlL6QzPLmt3LgwXtGwIE7HGh34YsLGyudB+ma4Etxt3IVzuy0lk6k9IVklje\/lQMP3jMCDtzjQIsLn965MaV3blSgYek8SNcE34m7e9OyYEvSiZS+kLzi3e52wYPdiBqvDxwYx8xKCe0kfeG48NJ5kK4JEPdLs1o6kdIXkpWHi5\/Kggfv2QAH7nNQ1FU56TxI1wSIO8Td\/SdBgFuU\/kALAjXgwBsW+SR9q3GfUoOasSpsrHQeIO7ezCPbW5VOpPSFZDuhHlUIHjwCPqxZcOAdB+FX5Ua1r07dE\/M9gEpM0jUBljssd4nrzrd9hrB4Tw048J4Dviq3M+84zfzzjZR4jfGgM96PQL5Lcog7xN0P6ygwfYCweE8lOPCeA+7BHyd\/Sht3n6L3Bt0kUuBhuftjHlnuhXQi8UCzPAVsqQA82AKjpUrAgSX4bCvMPCz5thRlrvyeirsLb1tjDlQkXRNgucNyd2BZRG+VEBbvuQcH3nPAPdB4sBI21suRQNy9RN\/GtqUTiQeajZPBQlXgwQJ4NhUFBzYBabGacB7MRJWz2Lzl4tI1IZCWO8dwnzBhAp0+fZquvfZaGjVqFMXFxamQr1OnTqXy5cvTgAEDKCkpKRQXXjqReKBZXsu2VAAebIHRUiXgwBJ8thUuzIM0gZeuCYET96NHj6pY7hwWlmO4v\/vuu7Rjxw4Vu3327Nnqb6VKlaKMjAzq3bs3tWzZUk1m6URK779tTxSPKwIPHhMQgLXsPYL29KCotcBX5ZKmf6ka4Lvwfk5BWMulLnLQ9IAkFvcRI0bQ0KFDqWnTprRkyRI6cuQIVatWjXiYqampaqSLFi1SP7XP0omU3v+ATD\/xXxKDwAPWgj9YLI4HI2FjvRxJEOZRoMSdJ8Pq1auVuPO2PFvm06ZNozfffJMSEhKoQ4cOar5kZ2dTTk4Opaenhyx3LycS2gYCQAAIRBMCxxOfpHNxzaj63\/v7dtg7d+70bd\/0dEy0uB88eJD69+9PmzZtooEDB1KvXr1o7NixxJHl6tWrR1lZWbRu3ToqW7YstWvXjhITE4sUdz1AIQ8QAAJAAAjYhwCHjeVrckjOICBa3AtDsn79elqxYoUS95iYGMrLy1Ni36pVK6pUqVKBbXk+WJecnOwMqqgVCAABIAAEgICHCARK3HNzc+m5555TB+Zq166trHb+161bN1qwYAGNHDlSQT1+\/HhKS0uj+Hh8a\/Rw7qFpIAAEgAAQcAiBQIk7H5r7+OOP6cUXX1Tv3PlQxPDhw9XJeb4KN3PmTHWwbvDgweoEPZ+cRwICQAAIAAEgEDQEAiXuQSMH4wECQAAIAAEgYAYBiLsZ1FAGCAABIAAEgICPEYC4+5gcdA0IAAEgAASAgBkEIO5mUEMZIAAEgAAQAAI+RgDi7mNy0DUgAASAABAAAmYQgLibQQ1lgAAQAAJAAAj4GAGIu4\/JQdeAABAAAkAACJhBAOJuBjWUAQJAAAgAASDgYwQg7j4mB10DAkAACAABIGAGAYi7GdRQBggAASAABICAjxGAuPuYHHQNCAABIAAEgIAZBCDuZlBDGSAABIAAEAACPkYA4u5jctA1IAAEgAAQAAJmEIC4m0ENZYAAEAACQAAI+BgBiLuPyUHXgAAQAAJAAAiYQQDibgY1lAECQAAIAAEg4GMEIO4+JgddAwJAAAgAASBgBgGIuxnUUAYIAAEgAAQsI3BoyRhTdZzLyzVVzkihWoPnGsnuu7wQdyK6+uqrfUcMOgQEgAAQsAOBzlccpyvLn1NVXVnubKjK2uXyf6d+f+nv\/P\/w39vRfuE69p6JsVztz6et1xGpE8n\/PBkpi6\/\/DnG\/JO47d+70NVEldY6\/nEjuv1jgC3UcPHjPZNA5OJb1hgL51OZ16ufZSxbsuX25VJw1G1OrUYiYmN\/8+v+y4b8vkKehyl+lfV\/ThErnQXr\/mTiIO8Td9AJGwYIIBOGBIJ1TyRywOLNIs2CzePPPokRbE2sWak2gY5u3syTGdvMumQfGQnr\/Ie6XZrR0Ir\/\/\/ntq3Lix3esT9RlEADwYBMyB7JI4YCtcE\/FTm7N+ta4vWdEVmrdXv5P47lcSD0VNQ+maAHGHuDvweI3eKqU\/0ILAnB850Czyk5vX0cktWaQJuWaBs4j7zfK2Ohf8yIORMUHcjaDl47zSiZS+kHw8NQx1DTwYgsuRzH7goCQx14Sct9NjL1nmjgDhcaV+4MEKBNI1AZY7LHcr8x9lCyEg\/YEWBEK94oAF\/eRmtsrXkXbojS1zFnOJ2+pW54JXPFjtt1Ye4m4Xkh7XI51I6QvJY\/ptax482Aal6Yrc5KCwoLOY8yG3uhkfme5\/UAq6yYMTmEnXBFjusNydWBdRW6f0B1oQiHODAxb1Yx\/No0NvZ1A0W+clzRc3eHByvkLcnUTXxbqlEyl9IblItaNNgQdH4dVVuVMcaIKuHYhjUW8w\/XtdfYrGTE7x4BaW0jUhkJb74sWLafjw4aE5MHHiREpJSaH333+fpk6dSuXLl6cBAwZQUlISlS5dWuWTTqT0heTWgnW6HfDgNMKR67ebg8JWOjt2qdC8XaAPw0VGOXIOu3mI3KK9OaRrQiDFfebMmdSxY0e69tprQ2xv3bqVZs+eTWPGjKFSpUpRRkYG9e7dm1q2bAlxt3dNRHVt0h9oQSDPLg5Y1Pe9cr+6tsZWeo2U0b5yEuN3ruziwatxQty9Qr6Yds+cOUMvv\/wy9ezZk+rUqRPKxdb8xYsXKTU1Vf1u0aJF6qf2WTqR0heSz6aR6e6AB9PQ2VbQCgfaAbljWfOUZ7hoPeluBxlWeLCjfat1SNeEwFnuJ06cUNb5rl27aPv27dSpUycaOnQozZs3jxISEqhDhw6K8+zsbMrJyaH09HRY7lZXAcqHEJD+QAsClWY54Ohk2hU23nqvclsfZbEjmUPALA\/mWrO\/FMTdfkwt1Xj8+HFatWqV2pavUqUKrVy5kj755BOqWLEitWvXjhITE4sVd63hNWvWWOqDF4X5y0z9+vW9aBpthiEAHryfDoY4OLSLLn7+N6JVU4lq1CdqfQ+Van1P\/v+RLCFgiAdLLdlbmLVDS9KDcQU6cExeXh6NHTuWWrVqRZUqVSqwLc8H65KTk2G527s2oro26dZKEMjTw0FRh+T4nTqSfQjo4cG+1uyvCZa7\/ZhaqpEPzvGBOhb0qlWrUlZWlrLcu3XrRgsWLKCRI0eq+sePH09paWkUHx8PcbeEOAqHIyD9gRYENkviAKLuHsPS1wLE3b25oqslPjS3YsUKmjJlCpUrV06dmB81ahTFxcWpq3As\/Jxn8ODB1KVLF3VynpN0IqUvJF3kCsgEHrwnqSgOIOru8yJ9LUjXBGY80Nvyeqe0dCKlLyS9PPk9H3jwnqFwDljUtYNycDrjLjfS14J0TfCNuPNBOL7Gxokt7sqVK7s6E6UTKX0huUq2g42BBwfB1Vk1c3BV5VIFRJ1Pv+Oduk4AbcomfS1I1wRPxf3ChQvqOtq0adNo\/\/796h05p1OnTlG1atXU1nnbtm1DXuRsmnNFViOdSOkLyUlu3awbPLiJdsG2tDvqef98lWjHeuVwpkr7PvAk5xEl0teCdE3wTNxPnjxJr7\/+urq+dfvtt19mqfPf+Rrbl19+qe6tO52kEyl9ITnNr1v1gwe3kP61ncLv0881\/B1dnf6O+x1BiwUQkL4WpGuCZ+LOVjv\/i4mJKXFJcB7N\/7uTa0c6kdIXkpPculk3eHAPbXY4w57kNPew2tY7OHCPg5Jaks6DdE3wTNy1SXHo0CF65plnlEe58BQbG6u8y91zzz2h7Xonp6x0IqUvJCe5dbNu8OAs2izopzavU57ktFCrhbfewYGzHOitXToP0jXBc3E\/f\/68un\/ODmXYNSxfU\/v73\/+u5g9v2W\/cuJGefvppvfPJdD7pREpfSKaJ81lB8GA\/IYUFPeY3jdS7dLbUi0rgwH4OzNQonQfpmuC5uLPlPmvWLBoyZIjyIMeJ\/cPzIbs\/\/\/nPyif8iBEjzMwtQ2WkEyl9IRkiy8eZwYN1csKDt2hb7pEEPbxVcGCdAztqkM6DdE3wXNz5Ctzzzz+v4qtrvtF3796t4q4PGjRIifvo0c67hZROpPSFZMfDxA91gAdzLIS\/P+catC332ObtDIdZBQfmOLC7lHQepGuC5+LOHdiwYYMS8LJly6rDc8eOHVPx1vfu3au26e+99167591l9UknUvpCcpxglxoADyUDzZb42bxc9d48\/2eWKmBFzAu3CA5cmuwRmpHOg3RN8IW4s4D\/8MMPxJOhYcOG6v17vXr16Ny5cxFP0+udxtwGu5\/lHQGun3cKkpKSQifxpRMpfSHp5dHv+cADEW+rcyz0cBHnz\/x7Tch5m71srUbElnn+z\/a2UQsObIPSUkXSeZCuCb4Q96VLl9Ly5cuJD9c9+eST9OKLL1Lfvn1D4VktzbBLhTmgzOzZs9WdefYnzzsDvXv3ppYtW6oc0omUvpDs4NgPdUQTD9pBN8adhTxcwDUR558VmrdXIs6puENwdnIXTRzYiZvddUnnQbomeC7uR48eVUFeWGjZqubteXZDywfqWOg1r3VWJ97ixYvVFn9qaqqqatGiReqn9lk6kdIXklV+\/VJeKg\/sf12zrDWx5p+FBTscZ95K18Sbf7op4CXxLZUDv8xhu\/ohnQfpmuC5uGun5Xv16qUO1rG4s295\/v+wYcOoZs2atsy1zMxMSkhIUNftOGVnZyvXt+np6eqzdCKlLyRbSLapErZIw9O5fT\/orvnQ4UNUo3qNiPnDhbRwZraCi0sstsX+rYRyJXVIE2nOw9vlnHirnFOtwXMjjsVvGbAW\/MGIdB6ka4Ln4s7W9Jw5c4jdzX799df08MMP05o1a4id2Dz00EO2vXNncWc\/9YmJiUWK+85780O\/IgGBcAR+iqntCCB7yhRf708xV5bY5p5i+qTV+VPTu6hu1YKeH+tW+fXzg7dUd2RMfqmUHWJpN2\/80qdo7IdUHjp27Biia+fOnaKp8zzk6+nTp+nDDz8kfvfOQWO6du2qPNNVqFDBNmALb8PzZz5Yl5ycrNqQ\/i1N+rdk24j2uCKnechcWbTl\/uPBkwVG\/t+Dpwp8\/vHQr59\/LPS3oiBrUDNW\/bpBjfyfV9WMpQY189djeud8q96vyWkO\/Dpuv\/VLOg\/SNYHngyfizj7jjxw5ot6DF5X40BtHhrPLrzwfqJs\/fz6NHDlSNTd+\/HhKS0uj+Ph4iLvfngqC+yP9gaZBr32J0L408JcF\/oJQ1BeD8C8Cba\/JfyXh5ReAoHAgeBmorkvnAeJucgaG+5TnQ3Uc8rVRo3yL4MCBA9S9e3d64IEHbIvrrl2FmzlzpvpCweFku3Tpok7Oc5JOpPSFZHIa+a5YtPGwcMPekODnfHdI8ZG943CIF034eyTUUb9r26Q6JV7j7GuBaOPAd4vgUoek8yBdEzyz3LUJyVvyM2bMUI5q+G47J35X884776j377x17kaSTqT0heQGx260AR7yUWYLn\/\/lXBJ6Fv7Cos+C74SFDw7cmOmR25DOg3RN8FzcDx48SGxN88n4cN\/yfC2OD9TZdVo+0lSUTqT0hRSJHyl\/Bw+RmdK2\/Bdu2BOy+tnC\/2rU7yMX1pEDHOgAyYUs0nmQrgmeizvfaZ8wYQI1btxYbZNzWrt2LX3zzTf01FNPwXLXuQilLySdw\/R9NvBgjCK27nlrn1Pmyu\/VTxZ6K1Y9ODDGgVO5pfMAcbdhZnAUOD69zu5heZue47j369fPNgc2eroonUjpC0kPRxLygAdrLGlirwl9eufG1COhthJ8vQkc6EXK2XzSeZCuCZ5Z7iziHCAmLi6uxBnGB+0i5bFjikonUvpCsoNDP9QBHuxjIfu7w8Rb92zZs7hrQh+pBXAQCSF3\/i6dB+ma4Jm4c1CY9957j7766itKSUmhJk2ahO61s0ObLVu20LvvvkutWrWibt26OT4bpRMpfSE5TrBLDYAHZ4AevHBrSOQjvZsHB85wYLRW6TxI1wTPxF2bKHl5ecR+35ctW6buRXK67rrr6O6771bX4XCgTt+Skr6Q9I3S\/7nAg3Mc8Zb9kIVb1al7tuKLO2kPDpzjwEjN0nmAuBth28d5pRMpfSH5eGoY6hp4MASXqcy8XT9k0VZVdlpq\/GX35sGBKVhtLySdB+ma4LnlbvuMMlmhdCKlLySTtPmuGHhwjxK+UscH7\/jA3Ss98j1NcgIH7nFQUkvSeZCuCRD3S7NTOpHSF5I\/HkfWewEerGNotIaaj31EiU2q03uDb4K4GwXPwfzS14J0TQikuPM7\/OHDh4em7cSJE9WhPb5qx85x2OvdgAEDKCkpKeS7XjqR0heSg88YV6sGD67CrRrjbfqk6V+GBB4cuM9BUS1K50G6JvhC3NnPPId9zc3NVf7kOcjLXXfdZToqHHu847B91157bWjOcZ2zZ8+mMWPGKH\/yGRkZ1Lt3b2rZsqXKI51I6QvJH48j670AD9YxNFMDH7ZjgeeT9ODADIL2l5HOg3RN8Fzc+Urc5MmT1VW4devW0YgRI2jVqlXEwWTMxHNnj3cvv\/wy9ezZk+rUyQ9WwYmteQ4Yk5qaqj4XDgErnUjpC8n+R4s3NYIHb3DnVlngW437lH5XL5ZWPW6PK1vvRiO\/ZelrQbomeC7umm95Dr\/6\/PPP0+jRo9W2uVnf8uztjq1zDj6zfft25e1u6NChNG\/ePEpISKAOHTrkb+VlZ1NOTg6lp6fDcpf\/HPHNCKQ\/0HwDpMmOaAIf\/g7eZFUoZhEB6WsB4m5xAhw\/fpymT5+u7rSzoLO4sxMb\/v+oUaMiuqDlLwf9+\/enTZs20cCBA1UoV7b8eVu+SpUqtHLlSvrkk0+oYsWK1K5dO0pMTCxW3LWhrFmzxuKo3C\/OX2bq16\/vfsNosQAC4MH7CfHPr\/5LIz85TwNvrk4P3uJseFnvR+vfHkhdC6wdWtq5c6d\/AdbRs1IXeb\/aw8RWNG\/NHz58mH73u9\/Rhg0baPz48SEhttI1dpIzduxY5emOo86Fb8vzDkFycjIsdysAo2wBBKRbK0GgU+OAT9G\/N+gmx+PHBwEzJ8YgfS3AcrdpVrAFz9vo7HP+mmuuMe1Png\/O8YE6FvSqVatSVlaWstzZhe2CBQto5MiRqsf85YFfBcTH59+PlU6k9IVk0zTyvBrw4DkFoQN1Sa98ST8eOmVbKFnvRyarB9LXgnRN4NniueXOljpvpfPpdRZf3gp57LHHTPmU502IFStW0JQpU6hcuXLqxDxv73PwGb4Kx8LPeXj7nkPM8sl5iLush4afeyv9geZnbPX2LZwDtt4LO7nRWw\/yWUNA+lqAuFvjn9hinzRpEj344IPqENy2bdvU\/fOXXnqJ\/vKXv0R8526x+VBx6URKX0h28eh1PeDBawYKeqjT7sBje959XqSvBema4Lnlzgfi+P75I488Qm+++Sa1bt2amjZtavq0vNkpLJ1I6QvJLG9+KwcevGekMAe8Pc\/BZg7+9TbvOxdFPZC+FqRrgufizvfSMzMz6ZdffqG9e\/cSe5PjbfovvviCnnrqKXUtzo0knUjpC8kNjt1oAzy4gXLJbRTFAd9\/b9ukegEf9N73NNg9kL4WpGuC5+LOHeC76f\/+97\/pyiuvpFq1atFHH32krq3xgTi3knQipS8kt3h2uh3w4DTCkesvioOFG\/YSx4RnD3YNasZGrgQ5LCMgfS1I1wRfiDt7qTt27Jg66MaJ77mzB7l+\/fpRjRo1LE8yPRVIJ1L6QtLDkYQ84MF7lorjgLfnr6oZC+vdJYqkrwXpmuC5uLOw8wn2f\/zjH8rlbPXq1dUW\/f33369cyMbExLgyFaUTKX0huUKyC42ABxdAjtBEcRzgcJ273EhfC9I1wXNx56Ax7Auer76tXbtW3TsvW7asuhrHAg9x17cgpS8kfaP0fy7w4D1HJXHA1jsnLTys970Nbg+krwWIu8W5GS7uHDjmiiuuUJHa+Coch2WtWbOmxRb0FZdOpPSFpI8l\/+cCD95zVBIHmu\/5V3rEq\/vvSM4hIH0tSNcEzy13fs\/+6quvqkN1t99+O7322mt0\/fXX0zfffKO8yFWuXNm52RdWs3QipS8kV0h2oRHw4ALIJrfltWJ8sC5nx2F4rnOYKulrQbomeC7u3IELFy6o9+wc3GXjxo0qnvuf\/vQndXLerSSdSOkLyS2enW4HPDiNcOT69XDAnutgvUfG0koOPTxYqd\/pstI1wRfizofq2FLnwDFa4vvtLVq0iHjPnctOmzaNunbtqlzNcmL3tezG9sCBA\/SHP\/yBhg0bpiLEsftZjjbHdfOWP3vCK126tCojnUjpC8nphepW\/eDBLaSLb0cPB2y98\/U4OLZxji89PDjXuvWapWuC5+LOgWI4yAuflGcwtVStWjVKSUkpcVt+\/fr1tHDhQvr6669p1qxZStzZnW1GRgb17duXmjdvTkuWLKEjR44okWdPeBzrnf3Jcx72Zc\/v9yHu1hcCashHQPoDLQg86uWArff0zo0pvXOjIAzbd2PQy4PvOn6pQxB3i8yw+1m+CsfWNYdkNZLYD\/3u3buVgD\/++ONK3Dmy3Ny5c5XlzvV9++23NGfOHCX0fAo\/POQrt6V9lk6k9IVkhHc\/5wUP3rOjl4PMlbmUufJ7WO8OUaaXB4eat1ytdE3wheU+Y8YMuvfee6levXqGCWGHN3zwrk+fPkrc2Zr\/4IMP6Omnn1bX6LR47nwK\/9Zbb6UOHTqoNjiGfE5ODqWnp8NyN4w6ChSHgPQHWhCYNcIBrHfnGDfCg3O9MF8zxN0kdrx9rm2Z79+\/X8Vdv+2229RVOE5FbcuzD3o+Wc9b6a+\/\/rq6JleUuPN9+eHDh6vt93Bx59P4iYmJxYq7NpQ1a9aYHJV3xTiiXv369b3rAFpWCIAH7yeCEQ6Wbz1OGav30\/I+9aluVXccZnmPkDs9MMKDOz3S10rHjh1DGfn8luTkSTx3PiHP78I1l7OFAWRhZoHXDrwVB3Bhcde24dly17blFyxYoKx6tuTDt+X5YF1ycjIsd8mz12d9l26t+AxOU90xygHc0pqCOWIhozxErNDlDLDcLQDOUeCmT5+ugsawRW3Gl3xhcecdAT6g179\/fxU6lncH+NBeQkICzZ8\/X72L58Rb+WlpacojHifpREpfSBamka+Kggfv6TDKAdzSOsOZUR6c6YX5WqVrAo\/cE8udBffZZ59V8dtvvvlmdU2Nt+effPJJQy5nC4s7D4i3Uvg0PH95aN++vYoVz1Y8t8GH93i3YPDgwdSlSxe1dQ9xN78AULIgAtIfaEHg0wwHcEtrP\/NmeLC\/F+ZrhLibxK7wKXn+PGnSJBoxYoSroV617ksnUvpCMjmNfFcMPHhPiRkONLe07w26iRKvqe79IALQAzM8+GnY0jXBM8udxZzvnbNVXaFCBeLPfAd99OjRrvmTD59I0omUvpD8tKit9AU8WEHPnrJWOGg17lO4pbWHBvE+H6RrAsT90kSWTqSVB5pNaxnVwImNL+aAlbXA4t6gRiyixtnApBUebGjechXSNcFTcedDb5s2bSqShPDrbpZZ0lGBdCKlLyQdFInIAh68p8kKBzhcZx9\/Vniwrxfma5KuCZ6Ju3nInSkpnUjpC8kZVt2vFTy4j3nhFq1yoPmd\/2rU76lBzVjvByS0B1Z58HrY0jUB4o5tea\/XUKDal\/5ACwIZdnCA0\/PWZ4IdPFjvhfkaIO7msfNVSelESl9IvpoMFjoDHiyAZ1NRuziAa1prhNjFg7VemC8tXRNgucNyNz\/7UfIyBKQ\/0IJAqV0c4P27tdlgFw\/WemG+NMTdPHa+KimdSOkLyVeTwUJnwIMF8GwqaicHWuQ43H83To6dPBhv3XoJ6ZoAyx2Wu\/VVgBpCCEh\/oAWBSrs50A7YQeCNzQ67eTDWuvXcEHfrGFqq4dy5czRt2jTq2rWrCg7DafHixSoqnJYmTpxIKSkpyv3s1KlTiQPGDBgwgJKSkkKBaaQTKX0hWZoEPioMHrwnwykO+B38Kz3iqUdCbe8HKaAHTvHg1tCla4Joy51jty9cuJC+\/vprmjVrVkjc2X88h+3TxJ4HuXXrVuURj73gsT959j3fu3dvFT6Wk3QipS8ktxas0+2AB6cRjly\/UxxoW\/TpnRtTeudGkTsS5Tmc4sEtWKVrgmhx37ZtG+3evVtFfnv88ceVmJ85c4Zefvll6tmzJ9WpUyc0D9ia54Ax4SFf+Y\/aZ+lESl9Ibi1Yp9sBD04jHLl+JzlYuGEv8TY9W+9sxSMVj4CTPLiBu3RNEC3u3PnCUeFOnDihrPNdu3bR9u3bqVOnTjR06FCaN2+eCvvaoUMHNS+ys7MpJyeH0tPTYbm7sVKipA3pD7Qg0OQ0B3yKfsiirQqqaanxCDRTzKRxmgen5yrE3WmEw+rPzMykV199VW2lv\/766yrATFHx3FetWqW25atUqUIrV66kTz75hCpWrEjt2rVTceOLE3etqTVr1rg4Knua4i8z9evXt6cy1GIaAfBgGjrbCrrFwax\/HaZXPztMv6sXSxm3x1HdqjG2jSEIFbnFg91YsXZoicOHS06exHO3C7Ci4rmH152Xl0djx46lVq1aqZju4dvyfLAuOTkZlrtdZKAe8ZGwgkChmxYjh4pNmv4l8U+8iy84e9zkwYl5C8vdCVQN1FlY3PngHB+oY0GvWrUqZWVlKcu9W6LpMq8AAAxhSURBVLdutGDBAho5cqSqffz48ZSWlkbx8fnvzaQTKX0hGaDc11nBg\/f0eMGBtlWviTy\/k492v\/Re8GDn7JOuCYxFoCx3PjS3YsUKmjJlCpUrV04dshs1ahTFxcWpq3As\/Jxn8ODB1KVLF3VyHuJu55KI7rqkP9CCwJ6XHPCBu+zvDhH\/ZHHvkVAnak\/We8mDHfMY4m4Hij6oQzqR0heSD6aALV0AD7bAaKkSP3DAFjwLfObK79VYNKFv26R61BzA8wMPViaSdE0Qb7lbIS+8rHQipS8ku3j0uh7w4DUD5LtzD0UJPYt8g5oVKMhiL30tSNcEiPulZ5F0IqUvJO8lwZ4egAd7cLRSi5850IQ+57tDlL3jcMiqb1AjltpeUyNQW\/h+5kHP\/JKuCRB3iLueeY48OhGQ\/kDTOUxfZ5PGAXu+4xQu+NpWvib6\/FmaVzxpPBSe1BB3Xy9z\/Z2TTqT0\/utnyt85wYP3\/EjngK37nB2H1fW6Hw+epP8ePBWy8jV0tZP4LP5X1YxVW\/zaFwK\/+L6XzoP0\/sNyD4jlHoSJ6L0sWO8BeLCOodUaooEDzdrXxJ8x+\/EQfxk4VSx84Vfz+EsBJ\/5ikP+lIP\/LQeEvD1a+KEjnQXr\/Ie5h4m71oYLyQAAIAAG\/InDqurtCXbtQ8Qr1\/wsV40r8nVNjKf3LfluqLv3LAVvqKa6SffMfcrR+pysXfc\/daXBQPxAAAkAACDiHgLYLYaYF3rlwMkkPDgRxd3J2oG4gAASAABAAAh4gAHH3AHQ3mmS\/+s8++yzt2bOH\/vKXv9D\/\/M\/\/uNEs2ghDYN++fYqD3Nxc6tevH919990hr4gAyn0EOPRzw4YNqU2bNu43HuUtHj9+nF544QXauHEj9e\/fX60FJGcRgLg7i69ntS9btowaNGhAzZo1o7lz59L9999PlStX9qw\/0djwP\/7xDxWtr3nz5vTKK69QSkoK1alTJxqh8HzM\/GX38ccfp0GDBkHcPWBj3bp1dPToUbr99ttp1qxZ6nnE8T+QnEMA4u4ctp7WzGFx2X8+i0n4\/z3tVJQ1vmPHDqpVq5YKOczizgGMrrrqqihDwR\/DXb58udrFatGiBcTdA0rmzZundg853seJEyeoQoUKVLp0aQ96Ej1NQtwDyPX58+dpzpw5dM8996i499OmTaPOnTurhYXkLgLnzp0j3g7ev3+\/ClgUE4O43+4yQMRW+5o1a9QuCuOPbXm3GSBlrR8+fJg4cmf37t0LBO5yvzfR0SLEXRjPvDhWr15NQ4cOVT1n8eBvxW+99ZayEJ9++mlKSEhQW\/F33nmnshxZ6LX\/CxuuL7urlwO2UCZNmkQ33ngjJScnQ9htZlMvD\/yKijk4cCD\/6hTE3T4i9HIwdepUuu2229QrKv4\/v6LCLpZ9PBRVE8TdWXxtq\/3IkSO0ZMkSWrp0KbVv357S09NV3fwui\/+NGDFCWYccq57\/tmHDBvXOna11jmXft29ftRWGZB4Boxxs2rRJnXNo166d+UZR8jIEjPAwbNgw+tvf\/ka7d++mgwcPUqVKldQXYLbikcwjYIQDfh598cUXxI5hWNxnzJihjA2Iu3n89ZSEuOtByQd5+DDKli1baPPmzUrENXHPzMyktm3bUmJioopVP3HiRLr11lupadOm6qT23r17cVreJv6McpCTk0Mff\/wxlStXTh0eysjIoMaNG9vUm+itxigPvDY4rV+\/Hpa7TdPGKAd8sJefVT\/88AP16tWL7rrrLtwcsYmL4qqBuDsMsN3V8wOKLXUW95MnT9LYsWPVYrnhhhtUU4sWLVKL5r777rO7adR3CQFw4I+pAB685wEceM8BxN2\/HBjqWeHFNG7cOOrTp4+y1CHuhqA0nRkcmIbO1oLgwVY4TVUGDkzB5kohWO6uwGxfI+GLKXwbXtuW562vjh07qkN1SM4gAA6cwdVoreDBKGL25wcH9mNqV40Qd7uQdKme8MXETfIW\/eeff058cIjfxbO485Z97dq1XepR9DUDDvzBOXjwngdw4D0H2Jb3LweGelZ4MWlX4fguNZ8E5lPzsNoNQWo4MzgwDJkjBcCDI7AaqhQcGILL1cyw3F2FG40BASAABIAAEHAeAYi78xijBSAABIAAEAACriIAcXcVbjQGBIAAEAACQMB5BCDuzmOMFoAAEAACQAAIuIoAxN1VuNEYEAACQAAIAAHnEYC4O48xWgACQAAIAAEg4CoCEHdX4UZjQAAIAAEgAAScRwDi7jzGaAEIAAEgAASAgKsIQNxdhRuNAQEgAASAABBwHgGIu\/MYowUgAASAABAAAq4iAHF3FW40BgQKIsCxAD788EMV711LnTt3poceekg0VP\/5z3\/ou+++Ix7LU089RT169KA2bdqExsTjbtSoUZGhiXfv3k1r1qyhnj17UkxMjGgc0Hkg4BUCEHevkEe7QIBIBfpp165dAeELB4Yj\/\/G\/0qVLi8Hr5MmTNH36dEpLS6PKlSsbFnce6KJFi6hZs2Z00003iRk3OgoE\/IQAxN1PbKAvUYdAceLOgYC2bNlCubm59Oijj1K1atVo\/PjxtG\/fPmXxsjVcr1492rBhA40ePZrKly+vAgZVqVKFevXqRWPGjFG\/r1mzJmnBPZ588klasWIFvfbaa8QBh7p06UJ9+\/alTZs20YIFC1QdX3\/9NbVu3VoFIIqNjaX58+croT19+jQNGjSIatWqperjutiq5r9zYiHX0saNG+mLL76gAQMGEAt9SZb7oUOHaOXKlaooRzW8+eabacKECfTjjz\/S22+\/TU888QSVK1cu6uYFBgwErCIAcbeKIMoDAQsIsLi\/+uqroRrq1q1Lc+fOVeKYnZ1NEydOpLNnz9KUKVOUuHIoX97yXrp0KfXr148mT56sQvxeeeWVSmj37t2rRLUoce\/UqZMKETxkyBAqU6aMyh8XF0dXXHGFqufll19Wn8eNG0ddu3alX375ReXn+s+cOUPPPvus2l7n\/rH4s1U+duxY1d61114bGsO0adPo+uuvpw4dOoTEffny5ZehxGO777771O\/5S8vIkSNV6OIbbriBjh49qup+\/PHHqU6dOhYQRlEgEJ0IQNyjk3eM2icIlGS5cxdZ\/FjM77\/\/fjpw4ECo1y1btqTBgwfT559\/rsS3VKlS9O233yoruDjLna1uFu7wNHDgQPVaQBNx\/hvvGjRs2JBycnLUtjiLNCftFQFvud944430m9\/8Rln1LPRs9WspfEyRLHceH+8KsJC3atWKunfvrsbC5Xinok+fPgW+OPiENnQDCPgeAYi77ylCB4OMgB5x\/\/LLL9W2OVvOFSpUoAsXLiirmkV\/7dq1NHz4cCWI\/Hn16tVqi7woy71SpUpqW12zltkaP3\/+vNqWL0rc+Xdt27alxMRERQFb02XLllWvC7gdrot3DNjKD09GxD0lJYXeeecd4kN0vKOgHaCDuAd51mNsbiAAcXcDZbQBBIpBQI+45+XlqS1r3qJu2rQpZWVlKSG\/44476Pnnn6dnnnmmwLb8ww8\/HNrS5m38OXPmqPfZLNTLli1Tws8izdvnt9xyixLUosT98OHDtHnzZrVVzuW5r7xLwNvxGRkZdPz4cfWT2whP3B6fCwjfli\/utDyfG+Bt\/ueee059WdAS183b9iz42JbH8gECxhGAuBvHDCWAgG0I6BF3boyta95SP3HihNqmHjVqlHo\/zlfGWBj5IJ12oI4F8e9\/\/zvNmDFDHai7+uqr1Xt1FuklS5bQG2+8ofqfmpqqrHze2i9K3Hnrn7fg2UrnnYHHHnuMOnbsqP6vfWH4v\/\/7P\/U5PPGBus8++0xd5zt16lSxB+r4fAG3zecLeCyc6tevr76YHDlyhN566y31hSZ8y9824FEREAg4AhD3gBOM4UUPAtu3b1fv3FncnUx80p4P4PEBvaKuqvGWOh8A5AN\/ha16vf3id\/ls\/YffjddbFvmAABAggrhjFgCBgCDghrgfPHhQWdN8op2\/RBRnVW\/btk0d8ONXB4Ut+0hw8\/t3\/pLCuwpwYhMJLfwdCBSNAMQdMwMIAAEgAASAQMAQgLgHjFAMBwgAASAABIAAxB1zAAgAASAABIBAwBCAuAeMUAwHCAABIAAEgADEHXMACAABIAAEgEDAEIC4B4xQDAcIAAEgAASAAMQdcwAIAAEgAASAQMAQgLgHjFAMBwgAASAABIAAxB1zAAgAASAABIBAwBCAuAeMUAwHCAABIAAEgMD\/AwpSEwhynu3KAAAAAElFTkSuQmCC","height":303,"width":503}}
%---
