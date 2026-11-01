%[text] # Wind generator: inverter-fed permanent magnet synchronous machine (inv_psm)
%[text] Initialization of inv_psm.slx. The machine is a 1.6 MW, 17.8 rpm, 104-pole permanent magnet synchronous
%[text] generator with six separate three-phase systems (WindGen). The model simulates ONE system (about 267 kW):
%[text] Simscape block permanent_magnet_synchronous_machine, shaft driven at fixed speed by an ideal speed source
%[text] (fixed_speed_setting). The system is connected through a 230 uH three-phase inductor to a three-phase
%[text] two-level IGBT inverter (FF1200R17IP5, loss and thermal model, 4 kHz PWM with double update, 3 us dead time)
%[text] on a DC link of 2 x 14.4 mF across a 1070 V lithium-ion battery. The control runs at 8 kHz in the per unit
%[text] of the machine: torque reference from the torque curve of the turbine (speed -> torque, generator), d current
%[text] reference from the MTPA block, d/q current PI with a voltage-vector limit (C-Caller dqvector_pi), alpha/beta
%[text] voltages to the space vector modulator (C-Caller sv_pwm). The rotor angle comes from a 6-state extended
%[text] Kalman filter (EKF) on the alpha/beta currents and voltages (sensorless); the encoder angle is also there.
%[text] Variant: speed control instead of the torque curve (use_speed_control, with motor_load_setting).
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, inv_hw_param, psm_machine_param, psm_ctrl_param, battery_param, device_igbt_param,
%[text] device_ideal_switch_param, check_model_param_refs). The library is needed only for the Simulink/Simscape
%[text] blocks, the C-Caller sources and the device data sets other than the embedded ones. Run this script from
%[text] this folder (the functions of ./private are visible only to the files of this folder).
%[text] The old flat init is init_model_legacy.m (kept until the merge). It no longer runs with the library:
%[text] lithium_ion_battery_setup now takes the charge capacity, and the battery block reads
%[text] param.battery.nominal_battery_voltage_1 from its mask.

clear;
model = 'inv_psm';

% Simulink/Simscape library blocks (blocks, not parameters), C-Caller sources and device data sets
library_path = 'C:\github\library';
if exist(library_path, 'dir')
    addpath(genpath(library_path));
else
    warning('%s init: library path %s not found, the library blocks and data sets must already be on the path', model, library_path);
end

% environment (was init_environment of the library)
close all; clc; beep off;
try
    pm_addunit('percent', 0.01, '1');       % custom Simscape unit used by the library blocks
catch ME
    if ~contains(ME.message, 'percent')     % already registered in this session: fine
        rethrow(ME);
    end
end
param = struct();

% local parameter functions of this model (./private)
local_functions = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
fprintf('%s init: local parameter functions in private/: %s\n', model, strjoin({local_functions.name}, ', '));
clear local_functions library_path
%[text] ## Simulation
param.sim.simlength = 2.25;                 % [s] model StopTime
param.sim.tc = 0.5e-6;                      % [s] logging base, MaxStep of the solver (ode23tb) and step of the
                                            %     Simscape local solver (glb_time.tc): ts/250
% inverter model: exactly one of the three inverter blocks of inv_psm_mod1/inverter/inverter is active
param.sim.use_mosfet_thermal_model = 0;     % (variant) MOSFET bridge with thermal model (values are literals in the block)
param.sim.use_thermal_model = 1;            % (variant) 1: IGBT bridge with loss and thermal model, 0: ideal switches
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration (block battery_1/Solver Configuration1)
if param.sim.use_mosfet_thermal_model || param.sim.use_thermal_model
    param.sim.nonlinear_iterations = 5;
end
param.sim.load_step_time = 1.25;            % [s] step time of the load torque (motor_load_setting, speed control only)
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
% one converter clock domain (inverter and control): 4 kHz PWM, double update -> ts = 125 us
param.time.fpwm = 4e3;                              % [Hz] switching frequency of the inverter
param.time.trgo = 1;                                % double update: control at 2*fpwm = 8 kHz
param.time.dead_time = 3e-6;                        % [s] dead time of the PWM generator
param.time.t_measure = 20/(17.8/60*52);             % [s] logging window: 20 electrical periods at 17.8 rpm (1.30 s)
param.time.tc_factor = 1/param.time.fpwm/param.sim.tc/2;    % ts/tc = 250, so that glb_time.tc = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;

% sample time of the battery state-of-charge estimator (legacy ts_dab of the shared template: 3*fpwm, double update)
param.time.fpwm_dab = 3*param.time.fpwm;            % [Hz]
param.time.trgo_dab = 1;

param.glb_time = timing_param(param.time);
%[text] ## Inverter
%[text] ### Hardware
% 250 kW data set of the library for the 690 V class (inv690V_250kW: udc_nom 1070 V), output inductor 230 uH
% (block output_filter/threephase inductor); DC link 2 x 14.4 mF of the AFE data set of the same class (block
% dc_source/dclink)
param.inv.application_voltage = 690;                % [V] voltage class of the data sets
param.inv.pwr_nom = 250e3;                          % [W]
param.inv.hw = inv_hw_param(param.inv.application_voltage, param.inv.pwr_nom, param.time.fpwm);
%[text] ### Behaviour of the clocks and of the PWM
% local clock of the inverter: time = k*t (block global timing/time_inv1). The block reads the gain of
% module 2 (1.001, legacy time_gain_inv_module_2), so the PWM carrier runs at 4004 Hz while the control
% runs at the exact ts; the correction from phase_compensation_1 is not connected (see the README).
param.inv.behav.time_gain_module_1 = 1.0005;        % legacy value, read by no block
param.inv.behav.time_gain_module_2 = 1.001;         % read by global timing/time_inv1
param.inv.behav.pwm_phase_shift_mod1 = 0;           % [rad] phase of the PWM carrier (sv_pwm/Constant1)
% clock of an AFE that this model does not have (block global timing/time_afe1, output not used)
param.afe.behav.time_gain_module_1 = 1.0002;
%[text] ### Power semiconductors
% IGBT bridge (default): six igbt_simple_2th cells with loss and thermal model, Infineon FF1200R17IP5
% (1700 V / 1200 A, values at 125 degC). The data set is embedded in private/device_igbt_param; any other data
% set of the library is read through device_data_load.
param.dev.used_device_igbt = 'infineon_FF1200R17IP5';
dev = device_igbt_param(param.dev.used_device_igbt, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
param.dev.igbt_name = dev.name;
param.dev.igbt_type = dev.type;
param.inv.igbt = rmfield(dev, {'name', 'type'});
clear dev

% ideal-switch bridge (variant use_thermal_model = 0)
param.dev.used_device_ideal_switch = 'silicon_high_power_ideal_switch';
param.inv.ideal_switch = device_ideal_switch_param(param.dev.used_device_ideal_switch, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
%[text] ### Thermal model of the IGBT bridge
% aluminium plate, liquid cooled, sized for PrimePACK2, liquid flow > 28 l/min. "A" (ambient) is the water:
% HA is the temperature rise from water to heatsink surface; water in/out rise max 5 K with 2 kW total losses
param.thermal.weight = 0.150;                                   % [kg]
param.thermal.no_weight = 0.150/10;                             % [kg] /10: thermal inertia not accounted
param.thermal.cp_al = 900;                                      % [J/(kg K)] specific heat, aluminium
param.thermal.heat_capacity_hs = param.thermal.cp_al*param.thermal.weight;   % [J/K]
param.thermal.thermal_conductivity_al = 160;                    % [W/(m K)] aluminium
param.thermal.Rth_switch_HA = 15/1000;                          % [K/W]
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;       % [K/W]
param.thermal.Tambient = 40;                                    % [degC] water temperature (mask of the bridge)
param.thermal.DThs_init = 0;                                    % [K]

% heatsink data read by the bridge masks (fields no_weight, cp, Rth_switch_HA, Rth_diode_HA)
param.thermal.heatsink.name = 'liquid_cooled_plate_2kw';
param.thermal.heatsink.weight = param.thermal.weight;
param.thermal.heatsink.no_weight = param.thermal.no_weight;
param.thermal.heatsink.cp = param.thermal.cp_al;
param.thermal.heatsink.heat_capacity = param.thermal.heat_capacity_hs;
param.thermal.heatsink.thermal_conductivity = param.thermal.thermal_conductivity_al;
param.thermal.heatsink.Rth_switch_HA = param.thermal.Rth_switch_HA;
param.thermal.heatsink.Rth_mosfet_HA = param.thermal.Rth_switch_HA;
param.thermal.heatsink.Rth_diode_HA = param.thermal.Rth_diode_HA;
param.thermal.heatsink.Tambient = param.thermal.Tambient;
param.thermal.heatsink.DThs_init = param.thermal.DThs_init;
%[text] ## Battery on the DC link (block battery_1/litium_ion_battery)
% lithium-ion pack across the whole DC link: 1070 V nominal (udc_nom of the 690 V AFE data set in the legacy
% init), 750 kW, initial state of charge 0.85. The library block reads battery_voltage_init_m =
% param.battery.nominal_battery_voltage_1 from its mask default. The block battery_2 is commented out; it
% reads pack_2, the same pack.
param.battery.nominal_battery_voltage_1 = 1070;                 % [V]
param.battery.nominal_battery_power = 750e3;                    % [W]
param.battery.initial_battery_soc = 0.85;
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;   % [Ah] ASSUMPTION: 1C rating
param.battery.plot_ocv = false;
b = param.battery;
param.battery.pack_1 = battery_param(b.nominal_battery_voltage_1, b.nominal_battery_power, b.charge_capacity_1, ...
    b.initial_battery_soc, param.glb_time.ts_dab, b.plot_ocv);
param.battery.pack_2 = param.battery.pack_1;                    % block battery_2 (commented out)
clear b
%[text] ## Permanent magnet synchronous machine
% data of psm_calculus of the library (WindGen): six three-phase systems on one rotor; Ld, Lq, Jm of the whole
% machine. The per-system values and the bases are computed in private/psm_machine_param (pmsm_setup):
% per system Rs 30.1 mOhm, Ld 6.24 mH, Lq 7.56 mH, ubez 365.8 V, ibez 486.0 A, tau_bez 143.1 kNm, 15.43 Hz
param.psm.machine_in.name = 'WindGen';
param.psm.machine_in.number_of_systems = 6;
param.psm.machine_in.pwr_nom = 1600e3;             % [W]
param.psm.machine_in.i_nom = 2062;                 % [A] rms, all systems
param.psm.machine_in.rpm_nom = 17.8;               % [rpm]
param.psm.machine_in.number_poles = 104;
param.psm.machine_in.eta = 96;                     % [%] (sets Rs)
param.psm.machine_in.Ld = 1.04e-3;                 % [H]
param.psm.machine_in.Lq = 1.26e-3;                 % [H]
param.psm.machine_in.Jm = 900e3/6;                 % [kg m^2]
param.psm.machine_in.h5_percent = 0;               % [%] 5th and 7th flux harmonics (the block uses literals 0)
param.psm.machine_in.h7_percent = 0;
param.psm.machine = psm_machine_param(param.psm.machine_in);

% voltage scales of the observers: control voltage (pu) -> machine voltage (pu of ubez)
param.psm.u_psm_scale = 2/3*param.inv.hw.udc_nom/param.psm.machine.ubez;                 % back-EMF observers (not active)
param.psm.u_psm_scale_ekf = sqrt(3)/2*2/3*param.inv.hw.udc_nom/param.psm.machine.ubez;   % EKF: udc/sqrt(3)/ubez
%[text] ## Mechanical side: torque curve, fixed speed, speed reference
% energy production (default): the shaft turns at rpm_sim (block fixed_speed_setting: step at t = 0, 0.1 s lag);
% the torque reference is the torque curve of the 1.5 MW generic generator (legacy library script
% n_sys_generic_1M5W_torque_curve, total torque divided among the systems) times torque_overload_factor
param.load.rpm_base = [6 7 8 9 10 11 12 13 14 15 16 17 17.8 18.0 18.2];        % [rpm]
param.load.torque_base = 1e3/param.psm.machine.number_of_systems* ...
    [50 75 100 125 150 175 200 225 275 350 450 600 750 885 885];                % [Nm] per system
param.load.torque_overload_factor = 1.15;
param.load.rpm_sim = 17.8;                          % [rpm] speed of the shaft (fixed_speed_setting)

% drive (variant use_speed_control): speed reference step to omega_m_sim at time_start_motor_control, 1 Hz
% filter; the load is the inertia external_load_inertia plus a 1 Nm torque (motor_load_setting)
param.load.omega_m_sim = param.psm.machine.omega_m_bez;     % [rad/s] speed reference
param.load.external_load_inertia = 1;                       % [kg m^2] (block motor_load_setting/Inertia)
%[text] ## Control
%[text] ### Mode of operation
param.psm.mode.use_torque_curve = 1;                                        % (variant) energy production
param.psm.mode.use_speed_control = 1 - param.psm.mode.use_torque_curve;     % (variant) drive
param.psm.mode.use_mtpa = 1;                                                % (variant) d current from MTPA, else 0
param.psm.mode.use_psm_encoder = 0;                                         % (variant) 1: encoder, 0: sensorless
param.psm.mode.time_start_motor_control = 0.25;                             % [s] step of the speed reference
%[text] ### C-Caller versus Simulink implementation
% back-EMF observer: exactly one of the three is active (Variant Source blocks of current_ctrl/bemf_observer)
param.psm.impl.use_ekf_bemf_module_1 = 1;                                   % (variant) EKF
param.psm.impl.use_observer_from_simulink_module_1 = 0;                     % (variant) Simulink back-EMF observer
param.psm.impl.use_observer_from_ccaller_module_1 = 0;                      % (variant) C-Caller back-EMF observer
% current controller (dq_pi_with_vector_antiwindup mod1)
param.psm.impl.use_current_controller_from_simulink_module_1 = 0;           % (variant)
param.psm.impl.use_current_controller_from_ccaller_module_1 = 1;            % (variant)
if param.psm.impl.use_ekf_bemf_module_1 + param.psm.impl.use_observer_from_simulink_module_1 + ...
        param.psm.impl.use_observer_from_ccaller_module_1 ~= 1
    error('%s init: exactly one back-EMF observer must be active', model);
end
if param.psm.impl.use_current_controller_from_simulink_module_1 + ...
        param.psm.impl.use_current_controller_from_ccaller_module_1 ~= 1
    error('%s init: exactly one current controller must be active', model);
end
%[text] ### Controllers and observers
% gains of the legacy init (library defaults of ctrl_pmsm_setup, current PI ki changed to 35)
ctrl_in.kp_w = 2.5;                                 % [pu/pu] speed PI
ctrl_in.ki_w = 18;                                  % [1/s]
ctrl_in.iq_lim = 1.4;                               % [pu] limit of the q current (torque) reference
ctrl_in.id_lim = 0.35;                              % [pu] lowest d current reference of the MTPA block
ctrl_in.kp_i = 0.25;                                % [pu/pu] d and q current PI
ctrl_in.ki_i = 35;                                  % [1/s]
ctrl_in.emf_fb_p = 0.2;                             % gains of the two non-EKF back-EMF observers
ctrl_in.emf_p = ctrl_in.emf_fb_p*4/10;
ctrl_in.omega_th = 0;                               % [pu] speed threshold of the encoder-sensorless switch
ctrl_in.jm_ekf = 1e6;                               % inertia of the EKF model: the speed is a free state
ctrl_in.q_ekf = 1/10;                               % EKF noise (ekf_pmsm_setup): Q = ts*q_ekf*diag(q_ekf_diag),
ctrl_in.q_ekf_diag = [param.glb_time.ts_inv^2 param.glb_time.ts_inv^2 1/2 1/2 1 1];   % R = ts*r_ekf*eye(2)
ctrl_in.r_ekf = 1;
param.psm.ctrl = psm_ctrl_param(param.glb_time.ts_inv, param.psm.machine.omega_bez, param.psm.machine.Jm_norm, ...
    param.psm.u_psm_scale, param.psm.machine, ctrl_in);
clear ctrl_in

% decay rate of the current PI integrators while the voltage vector is clipped
param.ctrl.CTRPIFF_CLIP_RELEASE = 0.001;            % [1/s]

% local time alignment to the master time (phase_compensation_1/alignment_ctrl, output not connected)
param.ctrl.kp_align = 0.25;
param.ctrl.ki_align = 18;                           % [1/s]
param.ctrl.lim_up_align = 0.05;
param.ctrl.lim_down_align = -0.05;

% filters (were part of setup_global_filters of the library): first-order low-pass y(k) = g1*y(k-1) + g0*u(k)
% with g0 = 2*pi*fcut*ts, and the second-order filter of the scope data (continuous state space)
ts = param.glb_time.ts_inv;
param.ctrl.filters.inv.g0_1Hz = 1*ts*2*pi;          % speed reference filter (speed control)
param.ctrl.filters.inv.g1_1Hz = 1 - param.ctrl.filters.inv.g0_1Hz;
param.ctrl.filters.inv.omega_flt_g0_10Hz = 10*ts*2*pi;      % filter of the speed in pu (omega_pu/flt)
param.ctrl.filters.inv.omega_flt_g1_10Hz = 1 - param.ctrl.filters.inv.omega_flt_g0_10Hz;
tau_sof = 1/(2*pi*50);                              % [s] 50 Hz second-order filter (scopes/inverter_data LTI System)
s = tf('s');
param.ctrl.filters.inv.SOF_50Hz.fcut = 50;          % [Hz]
param.ctrl.filters.inv.SOF_50Hz.time_constant = tau_sof;
param.ctrl.filters.inv.SOF_50Hz.ss_c = ss(1/(tau_sof*s + 1)^2);
clear ts tau_sof s
%[text] ### Design figures (informative)
% per-unit chain: currents in pu of ibez of the machine (one system), current PI output in pu of udc/sqrt(3)
% (linear limit of the space vector modulator; the C-Caller sv_pwm has no udc input, so 1 pu is udc/sqrt(3) of
% the actual battery voltage: no udc feed-forward, and the EKF scales its voltage with the nominal 1070 V).
% Current loop plant: machine inductance plus the 230 uH inductor, Rs + RLFi; no decoupling and no back-EMF
% feed-forward. MTPA block: id_ref = a - sqrt(a^2 + iq_ref^2) with a = speed/(2*(Ld_norm - Lq_norm)),
% saturated in [-id_lim, 0]. Operating point: rated speed, torque curve times torque_overload_factor.
mc = param.psm.machine; hw = param.inv.hw; c = param.psm.ctrl; ts = param.glb_time.ts_inv;
d.udc = param.battery.nominal_battery_voltage_1;                % [V] nominal battery voltage
d.u_ocv = param.battery.pack_1.number_of_cells*interp1(0:1e-3:1, param.battery.pack_1.ocv_model, ...
    param.battery.initial_battery_soc);                         % [V] open-circuit voltage at soc_init
d.u_ctrl_bez = d.udc/sqrt(3);                                   % [V] 1 pu of the current PI output
d.Ld = mc.Ld_m + hw.LFi;                                        % [H] d axis inductance seen by the current loop
d.Lq = mc.Lq_m + hw.LFi;                                        % [H]
d.R = mc.Rs_m + hw.RLFi;                                        % [Ohm]
% current open loop (w in rad/s): PI of dqvector_pi (u = kp*e + x, x = x + ki*ts*e), R-L plant, 2.5 samples of
% delay (1 sample of current measurement delay, 1 sample of voltage output delay, half a sample of PWM hold)
z_w = @(w) exp(1i*w*ts);
L_i = @(w, L) (c.kp_i + c.ki_i*ts*z_w(w)./(z_w(w) - 1)).*(d.u_ctrl_bez/mc.ibez)./(d.R + 1i*w*L).*exp(-1i*w*2.5*ts);
d.wc_d = fzero(@(w) abs(L_i(w, d.Ld)) - 1, [1 pi/ts]);          % [rad/s] crossover
d.pm_d = 180 + angle(L_i(d.wc_d, d.Ld))*180/pi;                 % [deg] phase margin
d.wc_q = fzero(@(w) abs(L_i(w, d.Lq)) - 1, [1 pi/ts]);
d.pm_q = 180 + angle(L_i(d.wc_q, d.Lq))*180/pi;
% operating point at rated speed
d.T_curve = interp1(param.load.rpm_base, param.load.torque_base, mc.rpm_nom);    % [Nm] torque curve, per system
d.iq_ref = -param.load.torque_overload_factor*d.T_curve/mc.tau_bez;              % [pu] torque reference = iq reference
a = 1/(mc.Ld_norm - mc.Lq_norm)/2;                              % MTPA block gain on the speed (speed = 1 pu)
d.id_ref_block = min(0, max(-c.id_lim, a - sqrt(a^2 + d.iq_ref^2)));            % [pu] d reference of the block
a_ok = mc.psi_m_norm/(2*(mc.Lq_norm - mc.Ld_norm));
d.id_ref_mtpa = a_ok - sqrt(a_ok^2 + d.iq_ref^2);               % [pu] textbook MTPA for the same iq
id = d.id_ref_block*mc.ibez; iq = d.iq_ref*mc.ibez; we = mc.omega_bez;
d.torque = 3/2*mc.pp*(mc.psi_m*iq + (mc.Ld_m - mc.Lq_m)*id*iq);                 % [Nm] per system
d.P_gen = -d.torque*mc.omega_m_bez;                             % [W] per system (generated)
ud = d.R*id - we*d.Lq*iq;                                       % [V] steady-state voltage at the inverter
uq = d.R*iq + we*(d.Ld*id + mc.psi_m);
d.u_inv = hypot(ud, uq);                                        % [V] peak phase voltage
d.u_inv_pu = d.u_inv/d.u_ctrl_bez;                              % [pu] of the control voltage (linear limit 1)
d.i_rms = hypot(id, iq)/sqrt(2);                                % [A] rms
d.cosphi = (ud*id + uq*iq)/(d.u_inv*hypot(id, iq));             % < 0: power flows to the DC link
param.psm.design = d;
fprintf('Per unit: 1 pu of control voltage = udc/sqrt(3) = %.0f V = %.2f ubez (battery %.0f V nominal, %.0f V open circuit at soc %.2f)\n', ...
    d.u_ctrl_bez, d.u_ctrl_bez/mc.ubez, d.udc, d.u_ocv, param.battery.initial_battery_soc);
fprintf('Current loop: Ld = %.2f mH, Lq = %.2f mH (with LFi), R = %.1f mOhm | crossover d %.0f Hz (PM %.0f deg), q %.0f Hz (PM %.0f deg) | Nyquist %.0f Hz\n', ...
    d.Ld*1e3, d.Lq*1e3, d.R*1e3, d.wc_d/2/pi, d.pm_d, d.wc_q/2/pi, d.pm_q, 1/(2*ts));
fprintf('Rated speed: iq_ref = %.3f pu, id_ref = %.3f pu (MTPA block; textbook MTPA %.3f pu) | torque %.1f kNm, %.0f kW per system, %.0f A rms, %.0f V peak (%.2f pu), cos(phi) %.2f\n', ...
    d.iq_ref, d.id_ref_block, d.id_ref_mtpa, d.torque/1e3, d.P_gen/1e3, d.i_rms, d.u_inv, d.u_inv_pu, d.cosphi);
clear mc hw c ts d a a_ok id iq we ud uq L_i z_w
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;               % [pu] quantizer of the acquisitions (pu signals, 12 bit)
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;
param.meas.Imax_adc = 1049.835;                     % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;
param.meas.Umax_adc = 1500;                         % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%[text] ## Model setup
open_system(model);
Simulink.importExternalCTypes(model, 'Names', {'bemf_obsv_output_t', 'dqvector_pi_output_t', 'sv_pwm_output_t'});
%[text] ### Scopes closed at start
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### Active inverter and mechanical setting
% the three inverter blocks and the two mechanical settings are switched by commenting them in and out (as the
% legacy init). The blocks that read the voltages of the ideal-switch inverter (Goto inverter_voltage_ee_phase_*
% inside it) follow it: when it is commented out, their From blocks would have no Goto.
inv_blocks = [model '/inv_psm_mod1/inverter/inverter/'];
if param.sim.use_mosfet_thermal_model
    inv_active = 'three_phase_inverter_mosfet_based_with_thermal_model';
elseif param.sim.use_thermal_model
    inv_active = 'three_phase_inverter_igbt_adv_thermal_model';
else
    inv_active = 'three_phase_inverter_ideal_switch_based_model';
end
for name = {'three_phase_inverter_mosfet_based_with_thermal_model', 'three_phase_inverter_igbt_adv_thermal_model', ...
        'three_phase_inverter_ideal_switch_based_model'}
    if strcmp(name{1}, inv_active)
        set_param([inv_blocks name{1}], 'Commented', 'off');
    else
        set_param([inv_blocks name{1}], 'Commented', 'on');
    end
end
ideal_switch_readers = {'inverter_measurement_sim/From2', 'inverter_measurement_sim/From3', 'inverter_measurement_sim/From4', ...
    'inverter_measurement_sim/To Workspace5', 'inverter_measurement_sim/To Workspace6', 'inverter_measurement_sim/To Workspace8', ...
    'scopes/inverter_data/From56', 'scopes/inverter_data/From57', 'scopes/inverter_data/From58', 'scopes/inverter_data/inverter_voltage_ee'};
for k = 1:numel(ideal_switch_readers)
    if strcmp(inv_active, 'three_phase_inverter_ideal_switch_based_model')
        set_param([model '/' ideal_switch_readers{k}], 'Commented', 'off');
    else
        set_param([model '/' ideal_switch_readers{k}], 'Commented', 'on');
    end
end
if param.psm.mode.use_torque_curve
    set_param([model '/fixed_speed_setting'], 'Commented', 'off');
    set_param([model '/motor_load_setting'], 'Commented', 'on');
else
    set_param([model '/fixed_speed_setting'], 'Commented', 'on');
    set_param([model '/motor_load_setting'], 'Commented', 'off');
end
clear inv_blocks inv_active name ideal_switch_readers k
%[text] ### Check of the param.* references of the model
if param.sim.check_param_refs
    [missing_refs, model_refs] = check_model_param_refs(model, param);
    if isempty(missing_refs)
        fprintf('%s init: all %d param.* references of the model blocks resolve\n', model, numel(model_refs));
    else
        warning('%s init: %d param.* references of the model blocks do not exist in param:', model, numel(missing_refs));
        disp(missing_refs);
    end
    clear missing_refs model_refs
end
