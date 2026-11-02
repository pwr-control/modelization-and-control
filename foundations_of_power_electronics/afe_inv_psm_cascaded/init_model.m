%[text] # Two AFE + inverter modules in parallel on one PMSM wind generator (afe_inv_psm_n)
%[text] Initialization of afe_inv_psm_n.slx. A 690 V grid (three-phase emulator + Dyn11 1 MVA transformer, star point
%[text] of the secondary grounded) feeds two identical back-to-back modules connected in parallel on both sides
%[text] ("hard parallel"): on the grid side at the transformer secondary, on the machine side at the terminals of
%[text] one permanent magnet synchronous generator. Each module: LCL filter (500 uH, 200 uF star, then the
%[text] transformer leakage), two-level AFE, DC link 7.2 mF with a brake chopper (4 Ohm), two-level inverter,
%[text] 230 uH output inductor; ideal-switch bridges, 4 kHz PWM with double update, 3 us dead time; each converter
%[text] has its own local clock (time gains 1.0002 / 1.0015 AFE, 1.0005 / 1.001 inverter) with an alignment loop
%[text] to the master time. Machine: two of the six 267 kW systems of the 1.6 MW, 17.8 rpm, 104-pole WindGen
%[text] generator, driven at fixed speed (fixed_speed_setting); the machine block is the one-system machine with
%[text] La, Lb, Rs divided and Jm, friction multiplied by number_of_modules.
%[text] AFE control at 8 kHz (per module): FHT-PLL (module 1 C-Caller, module 2 Simulink), DC-link voltage PI ->
%[text] active current reference, grid current sequence PIs and FRT (VDE 4110) -> current references,
%[text] proportional-resonant control of the converter current per phase, zero-sequence current PI (it reaches
%[text] the modulator of module 2 only: u_cm_comp_mod2 = -1), space vector modulation (C-Caller sv_pwm_cm).
%[text] Inverter control at 8 kHz (per module): torque curve -> q current, MTPA -> d current, dq current PI
%[text] (module 1 C-Caller dqvector_pi, module 2 see below), EKF for the angle, encoder angle selected
%[text] (use_psm_encoder = 1), space vector modulation (C-Caller sv_pwm).
%[text] Note: the folder name says "cascaded" but the two modules are in parallel.
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, grid_emulator_param, grid_fault_param, afe_hw_param, afe_ctrl_param, inv_hw_param,
%[text] psm_machine_param, psm_ctrl_param, device_ideal_switch_param, device_igbt_param, check_model_param_refs).
%[text] The library is needed only for the Simulink/Simscape blocks, the C-Caller sources and the device data
%[text] sets other than the embedded ones. Run this script from this folder (the functions of ./private are
%[text] visible only to the files of this folder).
%[text] The old flat init is init_model_legacy.m (kept until the merge). It no longer runs with the library:
%[text] grid_fault_generator is now a function with arguments and lithium_ion_battery_setup takes the charge
%[text] capacity.

clear;
model = 'afe_inv_psm_n';

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
param.sim.simlength = 1.25;                 % [s] model StopTime
param.sim.tc = 0.625e-6;                    % [s] logging base, model MaxStep (ode23tb) and step of the Simscape
                                            %     local solver (glb_time.tc): ts_afe/200
param.sim.use_thermal_model = 0;            % only sets the Simscape nonlinear iterations: the bridges that run are
                                            % the ideal-switch ones; the IGBT and MOSFET bridges with thermal model
                                            % are commented out in the model (the legacy init did not switch them)
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration (block grid/grid_emulator/Solver Configuration)
if param.sim.use_thermal_model
    param.sim.nonlinear_iterations = 5;
end
param.sim.load_step_time = 1.25;            % [s] step of the load torque (motor_load_setting, commented out)
param.sim.time_aux_power_supply_fault = 1e3;    % [s] the brake chopper of each module is enabled until this time
                                                %     (brake/Step: 1 -> 0); 1e3: never
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
% two clock domains, the same numbers: AFE and inverter at 4 kHz PWM, double update -> ts = 125 us
param.time.fpwm_afe = 4e3;                          % [Hz] switching frequency of the AFE
param.time.trgo_afe = 1;                            % double update: control at 2*fpwm = 8 kHz
param.time.dead_time_afe = 3e-6;                    % [s]
param.time.fpwm_inv = 4e3;                          % [Hz] switching frequency of the inverter
param.time.trgo_inv = 1;
param.time.dead_time_inv = 3e-6;                    % [s]
param.time.t_measure = param.sim.simlength;         % [s] logging window
param.time.tc_factor = 200;                         % ts_afe/tc: glb_time.tc = 125 us/200 = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;

param.glb_time = timing_param(param.time);
if abs(param.glb_time.tc - param.sim.tc) > 1e-12
    warning('%s init: glb_time.tc = %g s differs from param.sim.tc = %g s (FixedStep of the model)', model, ...
        param.glb_time.tc, param.sim.tc);
end
%[text] ## Grid
% three-phase grid emulator (ideal voltage sources) feeding a Dyn11 1 MVA 690/690 V transformer made of three
% single-phase transformer blocks (star point of the secondary grounded); the LCL filters of the two modules
% are connected to the secondary
param.grid.application_voltage = 690;               % [V] rms line-to-line, primary and secondary of the transformer
param.grid.f_nom = 50;                              % [Hz]
param.grid.omega_nom = 2*pi*param.grid.f_nom;       % [rad/s]
param.grid.pwr_nom = 1000e3;                        % [VA] rated power of the transformer
param.grid.ucc = 5;                                 % [%] short-circuit voltage of the transformer
param.grid.eta = 95;                                % [%] efficiency of the transformer (copper losses)
param.grid.p_iron = 1800;                           % [W] iron losses of the transformer
param.grid.n2 = 14;                                 % turns of the secondary (star) coils of the transformer blocks
param.grid.n1 = floor(param.grid.n2*sqrt(3));       % turns of the primary (delta) coils: 24 (ratio 1.714, not sqrt(3):
                                                    % the secondary is at 697 V, +1.0 %)
param.grid.core_area = 0.05;                        % [m^2] core of the transformer blocks
param.grid.core_length = 2.5;                       % [m]
param.grid.mur = 10e3;                              % [-] relative permeability: Lm1 = 145 mH, i1m = 8.8 A
param.grid.R_eq = 2e-3;                             % [Ohm] equivalent grid resistance (informative, not in the model)
%[text] ### Fault test of the grid emulator
% legacy test 25.4 (75 % dip on two phases, variant D, 915 ms from 0.75 s). The abc fault (enable_frt_1) is off;
% the sequence fault (enable_frt_2) is ON. The emulator SUBTRACTS the deep* values from the pre-fault sequences
% (positive xi = 1, the others 0): from 0.75 s the positive sequence is 0 + 0.4j (0.4 pu, +90 deg) and the negative
% sequence -0.4 - 0.4j (0.57 pu); at the transformer secondary the phase voltages are 0.97 / 0.40 / 0.61 pu. The
% fault lasts until 1.665 s, after the end of the simulation (1.25 s)
param.grid.fault_in.kind = 'two_phase_D';           % fault of the abc amplitudes (enable_frt_1)
param.grid.fault_in.dip = 0.75;                     % [pu]
param.grid.fault_in.error_length = 0.915;           % [s]
param.grid.fault_in.start_time = 0.75;              % [s]
param.grid.fault_in.enable_frt_1 = 0;               % faults generated from abc
param.grid.fault_in.enable_frt_2 = 1;               % faults generated from the xi/eta positive and negative sequences
param.grid.fault_in.deepPOSxi = 1;                  % [pu] positive sequence xi during the fault: 1*(1 - deepPOSxi)
param.grid.fault_in.deepPOSeta = -0.4;              % [pu] positive sequence eta during the fault: 0 - deepPOSeta
param.grid.fault_in.deepNEGxi = 0.4;                % [pu] negative sequence xi during the fault: 0 - deepNEGxi
param.grid.fault_in.deepNEGeta = 0.4;               % [pu] negative sequence eta during the fault: 0 - deepNEGeta
param.grid.fault_in.t_ramp = 20e-3;                 % [s]
param.grid.frt = grid_fault_param(param.grid.fault_in);
param.grid.emu = grid_emulator_param(param.grid, param.grid.frt);
%[text] ## AFE (both modules)
%[text] ### Hardware
% 250 kW data set of the library at 690 V: ubez 563.4 V, ibez 328.7 A (10 % margin on the power), udc_nom 1070 V;
% DC link 7.2 mF, filter 500 uH + 200 uF in star, brake resistor 4 Ohm
param.afe.pwr_nom = 250e3;                          % [W] per module
param.afe.hw = afe_hw_param(param.grid.application_voltage, param.afe.pwr_nom, param.time.fpwm_afe);
%[text] ### Behaviour of the clocks and of the PWM
% local clock of each converter: time = k*t (blocks global timing/time_afe1, time_afe2), corrected by the alignment
% loop of the converter (AFE_control/afe_control/phase_compensation_1)
param.afe.behav.time_gain_module_1 = 1.0002;        % [-] global timing/time_afe1
param.afe.behav.time_gain_module_2 = 1.0015;        % [-] global timing/time_afe2
param.afe.behav.pwm_phase_shift_mod1 = 0;           % [-] phase of the PWM carrier (afe/sv_pwm/Constant1)
param.afe.behav.pwm_phase_shift_mod2 = 0;
% zero-sequence voltage of the controller into the SVM (ctrl/space_vector/Gain3): the zero-sequence current PI of
% each module computes a voltage, the gain decides whether it reaches the modulator. Hard parallel: module 1
% keeps 0 (reference module), the other modules -1
param.afe.behav.en_parallel_mode = 1;
param.afe.behav.u_cm_comp_mod1 = 0;
param.afe.behav.u_cm_comp_mod2 = 0;
if param.afe.behav.en_parallel_mode
    param.afe.behav.u_cm_comp_mod2 = -1;
end
%[text] ### Variants of the AFE control
% PLL whose phase and frequency feed the control (the PLLs always run, Variant Source1/2 of
% grid_voltage_phase_lock_loops pick one): mode1 Simulink dq PLL, mode2 C-Caller dq PLL, mode3 Simulink FHT-PLL,
% mode4 C-Caller FHT-PLL. Module 1 reads dq_pll_mode1..4, module 2 reads dq_pll_mode1_modn..4_modn
param.afe.variant.dq_pll_mode1 = 0;                 % (variant) module 1
param.afe.variant.dq_pll_mode2 = 0;                 % (variant)
param.afe.variant.dq_pll_mode3 = 0;                 % (variant)
param.afe.variant.dq_pll_mode4 = 1;                 % (variant) FHT-PLL, C-Caller
param.afe.variant.dq_pll_mode1_modn = 0;            % (variant) module 2
param.afe.variant.dq_pll_mode2_modn = 0;            % (variant)
param.afe.variant.dq_pll_mode3_modn = 1;            % (variant) FHT-PLL, Simulink
param.afe.variant.dq_pll_mode4_modn = 0;            % (variant)
v = param.afe.variant;
if v.dq_pll_mode1 + v.dq_pll_mode2 + v.dq_pll_mode3 + v.dq_pll_mode4 ~= 1 || ...
        v.dq_pll_mode1_modn + v.dq_pll_mode2_modn + v.dq_pll_mode3_modn + v.dq_pll_mode4_modn ~= 1
    error('%s init: exactly one PLL variant per module must be active', model);
end
clear v
% moving average filters of the voltage and current sequences: C-Caller (1) or Simulink (0) version
param.afe.impl.use_moving_average_from_ccaller_mod1 = 1;   % (variant) module 1, voltage and current sequences
param.afe.impl.use_moving_average_from_ccaller_mod2 = 0;   % (variant) module 2, voltage sequences
param.afe.impl.use_moving_average_from_ccaller_mod3 = 0;   % (variant) module 2, current sequences
%[text] ### AFE control
% gains of the legacy init: library defaults of ctrl_afe_setup, except the udc PI kp (0.85 -> 2) and the
% proportional-resonant current controller (kp 0.2 -> 0.6, ki 45 -> 35: "gains for LVRT")
ctrl_in.udc_norm_ref = 1;                           % [pu] of udc_nom
ctrl_in.kp_udc_ctrl = 2;                            % [pu/pu] DC-link voltage PI
ctrl_in.ki_udc_ctrl = 35;                           % [1/s]
ctrl_in.kp_rpi = 0.6;                               % [pu/pu] proportional-resonant current controller
ctrl_in.ki_rpi = 35;                                % [1/s]
ctrl_in.delta_res = 0.015;                          % [-] damping of the resonator (library value)
ctrl_in.kp_rc_pos_grid = 0.25;                      % [pu/pu] PI of the positive sequence (eta) of the grid current
ctrl_in.ki_rc_pos_grid = 35;                        % [1/s]
ctrl_in.kp_rc_neg_grid = 0.25;                      % [pu/pu] PI of the negative sequence (xi, eta)
ctrl_in.ki_rc_neg_grid = 35;                        % [1/s]
ctrl_in.kp_is_z = 0.5;                              % [pu/pu] zero-sequence current PI
ctrl_in.ki_is_z = 18;                               % [1/s]
ctrl_in.pll_p = 1;                                  % PLLs (the FHT-PLLs read the same kp, ki)
ctrl_in.pll_i1 = 80;                                % [1/s]
ctrl_in.pll_p_frt = 0.2;
ctrl_in.ixi_pos_ref_lim = 1.65;                     % [pu] limits of the current references
ctrl_in.ieta_pos_ref_lim = 1.0;
ctrl_in.ieta_neg_ref_lim = 0.5;
ctrl_in.i_grid_pos_eta_lim = 1;
ctrl_in.i_grid_neg_xi_lim = 0.5;
ctrl_in.i_grid_neg_eta_lim = 0.5;
ctrl_in.i_grid_pos_eta_red_lim = 0.1;
ctrl_in.i_grid_neg_eta_red_lim = 0.1;
ctrl_in.i_grid_neg_xi_red_lim = 0.1;
ctrl_in.fht.delta = 0.05;                           % first harmonic tracker: damping of the resonant model
ctrl_in.fht.poles = [-1 -4];                        % observer poles, times omega_base
ctrl_in.di_obs.poles = [-1 -4];                     % double integrator observer of the phase
ctrl_in.vr_obs.poles = [-0.1 -0.4];                 % voltage rate observer
param.afe.ctrl = afe_ctrl_param(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom, ctrl_in);
clear ctrl_in
%[text] ### FRT control (VDE 4110) and reactive current references
param.afe.frt.k_frt_ref = 2;                        % [-] gain of the reactive current reference on the voltage dip
param.afe.frt.settle_time = 0.175;                  % [s] FRT detection enabled after this time (FRT-Detection/Step3)

% reactive current reference: steps of the positive sequence eta reference after the fault (cos(phi) 0.95 of
% the active current ixi_ref_mod1). The steps come at 2.0 s and 2.5 s: after the end of the simulation (1.25 s)
param.afe.ref.ixi_ref_mod1 = -0.85;                 % [pu] active current reference (constant blocks i_xi_ref/Constant1,
param.afe.ref.ixi_ref_mod2 = -0.85;                 %      commented out: the udc PI gives the reference)
param.afe.ref.enable_i_react_pos_steps = 1;
param.afe.ref.time_i_react_pos_ref_1 = 0;           % [s]
param.afe.ref.time_i_react_pos_ref_2 = 0;           % [s]
param.afe.ref.i_react_pos_ref_1 = 0;                % [pu]
param.afe.ref.i_react_pos_ref_2 = 0;                % [pu]
param.afe.ref.i_react_pos_ref_3 = 0;                % [pu]
if param.afe.ref.enable_i_react_pos_steps
    param.afe.ref.time_i_react_pos_ref_1 = param.grid.frt.start_time_LVRT + param.grid.frt.error_length + 0.335;
    param.afe.ref.time_i_react_pos_ref_2 = param.afe.ref.time_i_react_pos_ref_1 + 0.5;
    param.afe.ref.i_react_pos_ref_2 = -param.afe.ref.ixi_ref_mod1*tan(acos(0.95));
    param.afe.ref.i_react_pos_ref_3 = param.afe.ref.ixi_ref_mod1*tan(acos(0.95));
end
%[text] ### Filters of the AFE control
% moving averages (dmavg_flt of the sequence filters and of the islanding detector): enabled after
% dmavg_filter_enable_time, window of 1/(mavarage_filter_frequency_base_order*f_grid) (2: 100 Hz)
param.ctrl.dmavg_filter_enable_time = 0.025;        % [s]
param.ctrl.mavarage_filter_frequency_base_order = 2;
% first-order low-pass y(k) = g1*y(k-1) + g0*u(k), g0 = 2*pi*fcut*ts (FRT reactive current references, 161 Hz)
param.ctrl.filters.afe.g0_161Hz = 161*param.glb_time.ts_afe*2*pi;
param.ctrl.filters.afe.g1_161Hz = 1 - param.ctrl.filters.afe.g0_161Hz;
%[text] ## Inverter (both modules)
%[text] ### Hardware
% 250 kW data set of the library for the 690 V class (inv690V_250kW: udc_nom 1070 V), output inductor 230 uH
% (block L_filter/threephase inductor). The inverter sits on the DC link of the AFE of its module.
param.inv.pwr_nom = 250e3;                          % [W] per module
param.inv.hw = inv_hw_param(param.grid.application_voltage, param.inv.pwr_nom, param.time.fpwm_inv);
%[text] ### Behaviour of the clocks and of the PWM
param.inv.behav.time_gain_module_1 = 1.0005;        % [-] global timing/time_inv1
param.inv.behav.time_gain_module_2 = 1.001;         % [-] global timing/time_inv2
param.inv.behav.pwm_phase_shift_mod1 = 0;           % [-] phase of the PWM carrier (inverter/inverter/sv_pwm/Constant1)
param.inv.behav.pwm_phase_shift_mod2 = 0;
%[text] ## Power semiconductors
% ideal-switch bridges (the ones that run) and the brake chopper: one data set, embedded in
% private/device_ideal_switch_param
param.dev.used_device_ideal_switch = 'silicon_high_power_ideal_switch';
param.afe.ideal_switch = device_ideal_switch_param(param.dev.used_device_ideal_switch, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.inv.ideal_switch = device_ideal_switch_param(param.dev.used_device_ideal_switch, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);

% IGBT bridges with thermal model (commented out in the model): Mitsubishi CM1200DW-34T, embedded in
% private/device_igbt_param; any other data set of the library is read through device_data_load
param.dev.used_device_igbt = 'mitsubishi_CM1200DW_34T';
dev = device_igbt_param(param.dev.used_device_igbt, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.dev.igbt_name = dev.name;
param.dev.igbt_type = dev.type;
param.afe.igbt = rmfield(dev, {'name', 'type'});
dev = device_igbt_param(param.dev.used_device_igbt, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
param.inv.igbt = rmfield(dev, {'name', 'type'});
clear dev
%[text] ### Thermal data
% aluminium plate, liquid cooled, sized for PrimePACK2, liquid flow > 28 l/min. "A" (ambient) is the water:
% HA is the temperature rise from water to heatsink surface; water in/out rise max 5 K with 2 kW total losses.
% Read by the brake resistor (brake/Rbrake: Tambient, DThs_init) and by the IGBT bridges (commented out)
param.thermal.weight = 0.150;                                   % [kg] (the IGBT bridges read it: legacy "weigth",
                                                                %      a name the legacy init never defined)
param.thermal.no_weight = 0.150/10;                             % [kg] /10: thermal inertia not accounted
param.thermal.cp_al = 900;                                      % [J/(kg K)] specific heat, aluminium
param.thermal.heat_capacity_hs = param.thermal.cp_al*param.thermal.weight;   % [J/K]
param.thermal.thermal_conductivity_al = 160;                    % [W/(m K)] aluminium
param.thermal.Rth_switch_HA = 15/1000;                          % [K/W]
param.thermal.Rth_mosfet_HA = param.thermal.Rth_switch_HA;      % [K/W]
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;       % [K/W]
param.thermal.Tambient = 40;                                    % [degC] water temperature
param.thermal.DThs_init = 0;                                    % [K] initial heatsink rise
%[text] ## Permanent magnet synchronous machine
% data of psm_calculus of the library (WindGen): six three-phase systems on one rotor; Ld, Lq, Jm of the whole
% machine. The per-system values and the bases are computed in private/psm_machine_param (pmsm_setup):
% per system Rs 30.1 mOhm, Ld 6.24 mH, Lq 7.56 mH, ubez 365.8 V, ibez 486.0 A, tau_bez 143.1 kNm, 15.43 Hz.
% The two modules drive two systems in parallel: the machine block divides La, Lb, Rs by number_of_modules and
% multiplies Jm and the friction by it
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
param.psm.number_of_modules = 2;                   % modules (machine systems) in parallel on the machine block

% voltage scales of the observers: control voltage (pu) -> machine voltage (pu of ubez)
param.psm.u_psm_scale = 2/3*param.inv.hw.udc_nom/param.psm.machine.ubez;                 % back-EMF observers (not active)
param.psm.u_psm_scale_ekf = sqrt(3)/2*2/3*param.inv.hw.udc_nom/param.psm.machine.ubez;   % EKF: udc/sqrt(3)/ubez
%[text] ## Mechanical side: torque curve, fixed speed, speed reference
% energy production (default): the shaft turns at rpm_sim (block fixed_speed_setting); the torque reference of
% each module is the torque curve of the 1.5 MW generic generator (legacy library script
% n_sys_generic_1M5W_torque_curve, total torque divided among the six systems) times torque_overload_factor
param.load.rpm_base = [6 7 8 9 10 11 12 13 14 15 16 17 17.8 18.0 18.2];        % [rpm]
param.load.torque_base = 1e3/param.psm.machine.number_of_systems* ...
    [50 75 100 125 150 175 200 225 275 350 450 600 750 885 885];                % [Nm] per system
param.load.torque_overload_factor = 1;
param.load.rpm_sim = 17.8;                          % [rpm] speed of the shaft (fixed_speed_setting)

% drive (variant use_speed_control): speed reference step to omega_m_sim at time_start_motor_control, 1 Hz
% filter; the load is the inertia external_load_inertia (motor_load_setting, commented out)
param.load.omega_m_sim = param.psm.machine.omega_m_bez;     % [rad/s] speed reference
param.load.external_load_inertia = 1;                       % [kg m^2] (block motor_load_setting/Inertia)
%[text] ## Inverter control (both modules)
%[text] ### Mode of operation
param.psm.mode.use_torque_curve = 1;                                        % (variant) energy production
param.psm.mode.use_speed_control = 1 - param.psm.mode.use_torque_curve;     % (variant) drive
param.psm.mode.use_mtpa = 1;                                                % (variant) d current from MTPA, else 0
param.psm.mode.use_psm_encoder = 1;                                         % (variant) 1: encoder, 0: sensorless
param.psm.mode.time_start_motor_control = 0.25;                             % [s] step of the speed reference
%[text] ### C-Caller versus Simulink implementation
% back-EMF observer of each module: exactly one of the three is active (Variant Source blocks of
% current_ctrl/bemf_observer)
param.psm.impl.use_ekf_bemf_module_1 = 1;                                   % (variant) EKF
param.psm.impl.use_observer_from_simulink_module_1 = 0;                     % (variant) Simulink back-EMF observer
param.psm.impl.use_observer_from_ccaller_module_1 = 0;                      % (variant) C-Caller back-EMF observer
param.psm.impl.use_ekf_bemf_module_2 = 1;                                   % (variant)
param.psm.impl.use_observer_from_simulink_module_2 = 0;                     % (variant)
param.psm.impl.use_observer_from_ccaller_module_2 = 0;                      % (variant)
% current controller (dq_pi_with_vector_antiwindup mod1). Module 1: Simulink (input 1) or C-Caller (input 2).
% Module 2 reads only the Simulink flag: 1 -> Simulink dq_vector_pi, 0 -> C-Caller dq_vector_pi_ccaller, which is
% commented out in module 2: with 0 (legacy value) the voltage reference of the inverter of module 2 is zero
% (see the check in "Model setup" and the README)
param.psm.impl.use_current_controller_from_simulink_module_1 = 0;           % (variant)
param.psm.impl.use_current_controller_from_ccaller_module_1 = 1;            % (variant)
param.psm.impl.use_current_controller_from_simulink_module_2 = 0;           % (variant)
fl = param.psm.impl;
if fl.use_ekf_bemf_module_1 + fl.use_observer_from_simulink_module_1 + fl.use_observer_from_ccaller_module_1 ~= 1 || ...
        fl.use_ekf_bemf_module_2 + fl.use_observer_from_simulink_module_2 + fl.use_observer_from_ccaller_module_2 ~= 1
    error('%s init: exactly one back-EMF observer per module must be active', model);
end
if fl.use_current_controller_from_simulink_module_1 + fl.use_current_controller_from_ccaller_module_1 ~= 1
    error('%s init: exactly one current controller must be active in module 1', model);
end
clear fl
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

% local time alignment to the master time (phase_compensation_1 of the AFE and inverter controls)
param.ctrl.kp_align = 0.25;
param.ctrl.ki_align = 18;                           % [1/s]
param.ctrl.lim_up_align = 0.05;
param.ctrl.lim_down_align = -0.05;

% filters of the inverter control (were part of setup_global_filters of the library): first-order low-pass
% y(k) = g1*y(k-1) + g0*u(k) with g0 = 2*pi*fcut*ts, and the second-order filter of the scope data
ts = param.glb_time.ts_inv;
param.ctrl.filters.inv.g0_1Hz = 1*ts*2*pi;          % speed reference filter (speed control)
param.ctrl.filters.inv.g1_1Hz = 1 - param.ctrl.filters.inv.g0_1Hz;
param.ctrl.filters.inv.omega_flt_g0_10Hz = 10*ts*2*pi;      % filter of the speed in pu (omega_pu/flt)
param.ctrl.filters.inv.omega_flt_g1_10Hz = 1 - param.ctrl.filters.inv.omega_flt_g0_10Hz;
tau_sof = 1/(2*pi*50);                              % [s] 50 Hz second-order filter (scopes/psm/LTI System)
s = tf('s');
param.ctrl.filters.inv.SOF_50Hz.fcut = 50;          % [Hz]
param.ctrl.filters.inv.SOF_50Hz.time_constant = tau_sof;
param.ctrl.filters.inv.SOF_50Hz.ss_c = ss(1/(tau_sof*s + 1)^2);
clear ts tau_sof s
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;               % [pu] quantizer of the acquisitions (pu signals, 12 bit)
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;
param.meas.Imax_adc = 1049.835;                     % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;
param.meas.Umax_adc = 1500;                         % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%[text] ## Design figures (informative)
% Operating point of one module at rated speed (17.8 rpm, torque curve times torque_overload_factor), losses
% neglected; AFE at unity power factor on the grid side (no reactive current reference before the fault).
% AFE current loop: proportional-resonant controller on the converter current, plant LFu + transformer
% leakage (LCL seen as L1 + L2), voltage in pu of udc/sqrt(3) (SVM), 2.5 samples of loop delay (1 measurement +
% 1 output + 0.5 PWM hold). A loop on the converter current damps the LCL resonance only below
% 1/(4*delay) = fs/10 = 800 Hz: with the two modules the resonance is at 1039 Hz, above it (see the README,
% design check: damping of that mode 0.9 %, kp_rpi limit 0.71).
% Forward-Euler resonator: |z| = sqrt(1 - 2*delta*w*ts + (w*ts)^2) > 1 when delta < w*ts/2.
mc = param.psm.machine; ha = param.afe.hw; hi = param.inv.hw; tr = param.grid.emu.trafo;
ts_a = param.glb_time.ts_afe; w = param.grid.omega_nom; c = param.afe.ctrl;
d.T_curve = interp1(param.load.rpm_base, param.load.torque_base, mc.rpm_nom);    % [Nm] torque curve, per system
d.P_module = param.load.torque_overload_factor*d.T_curve*mc.omega_m_bez;         % [W] mechanical power per module
d.u_peak = ha.ubez*tr.n2/tr.n1*sqrt(3);                                          % [V] peak phase voltage at the secondary
d.i_grid_peak = d.P_module/(3/2*d.u_peak);                                       % [A] per module
d.i_grid_pu = d.i_grid_peak/ha.ibez;                                             % [pu] of ibez of the AFE
d.L_grid = tr.Ld2 + tr.Ld1*(tr.n2/tr.n1)^2;                                      % [H] transformer leakage, secondary side
d.f_res = sqrt((ha.LFu + d.L_grid)/(ha.LFu*d.L_grid*ha.CFu))/2/pi;               % [Hz] LCL resonance (one module alone)
d.f_res_2 = sqrt((ha.LFu + 2*d.L_grid)/(ha.LFu*2*d.L_grid*ha.CFu))/2/pi;         % [Hz] the two modules together (the
                                                                                 %      leakage is shared: 2*L_grid per module)
d.f_damp_limit = 1/(4*2.5*ts_a);                                                 % [Hz] highest resonance damped by the
                                                                                 %      converter-current loop (2.5 samples)
d.k_i = (ha.udc_nom/sqrt(3))/(ha.ibez*(ha.LFu + 2*d.L_grid));                    % [1/s] plant gain of the current loop (pu/pu)
d.f_c_i = c.res_pi.kp_rpi*d.k_i/2/pi;                                            % [Hz] crossover estimate of the current loop
d.res_pole = sqrt(1 - 2*c.res_pi.delta_res*w*ts_a + (w*ts_a)^2);                 % [-] |z| of the discrete resonator
d.k_udc = ha.norm.Pbez/(ha.CFi*ha.udc_nom^2);                                    % [1/s] plant gain of the udc loop
d.f_c_udc = c.kp_udc_ctrl*d.k_udc/2/pi;                                          % [Hz] crossover estimate of the udc loop
d.u_brake_on = 1.175*ha.udc_nom;                                                 % [V] brake chopper hysteresis (MATLAB
d.u_brake_off = 1.15*ha.udc_nom;                                                 %      Function of brake/, literals)
% inverter of module 2 with zero voltage reference (legacy setting): the machine EMF drives the current through
% the machine inductance (two systems in parallel) and the 230 uH of module 2, module 1 still in current control
d.Ls = (mc.Ld_m + mc.Lq_m)/2/param.psm.number_of_modules;                        % [H] machine, two systems in parallel
d.i_mod2_peak = mc.psi_m*mc.omega_bez/(mc.omega_bez*(d.Ls + hi.LFi));            % [A] EMF / reactance (module 1 current
                                                                                 %     neglected; the design check with Ld,
                                                                                 %     Lq and module 1 gives 1029 A)
param.design = d;
fprintf('Design at %.1f rpm: %.0f kW per module (torque curve x %.2f), grid current %.0f A peak per module (%.2f pu of ibez)\n', ...
    mc.rpm_nom, d.P_module/1e3, param.load.torque_overload_factor, d.i_grid_peak, d.i_grid_pu);
fprintf('LCL: LFu %.0f uH, CFu %.0f uF, transformer %.1f uH: resonance %.0f Hz (one module), %.0f Hz (two modules), damped by the current loop only below %.0f Hz | current loop crossover estimate %.0f Hz | udc loop crossover estimate %.0f Hz\n', ...
    ha.LFu*1e6, ha.CFu*1e6, d.L_grid*1e6, d.f_res, d.f_res_2, d.f_damp_limit, d.f_c_i, d.f_c_udc);
fprintf('Resonant controller: delta = %.3f, |z| of the forward-Euler resonator = %.6f (unstable open loop if > 1, delta < %.4f) | brake on at %.0f V, off at %.0f V\n', ...
    c.res_pi.delta_res, d.res_pole, w*ts_a/2, d.u_brake_on, d.u_brake_off);
clear mc ha hi tr ts_a w c d
%[text] ## Model setup
open_system(model);
Simulink.importExternalCTypes(model, 'Names', {'mavgflt_output_t', 'dsmavgflt_output_t', 'mavgflts_output_t', ...
    'bemf_obsv_output_t', 'bemf_obsv_load_est_output_t', 'dqvector_pi_output_t', 'sv_pwm_output_t', ...
    'sv_pwm_cm_output_t', 'global_state_machine_output_t', 'first_harmonic_tracker_output_t', ...
    'dqpll_thyr_output_t', 'dqpll_grid_output_t', 'rpi_output_t', 'phase_shift_flt_output_t', ...
    'sogi_flt_output_t', 'linear_double_integrator_observer_output_t'});
%[text] ### Scopes closed at start
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### Mechanical setting
if param.psm.mode.use_torque_curve
    set_param([model '/fixed_speed_setting'], 'Commented', 'off');
    set_param([model '/motor_load_setting'], 'Commented', 'on');
else
    set_param([model '/fixed_speed_setting'], 'Commented', 'on');
    set_param([model '/motor_load_setting'], 'Commented', 'off');
end
%[text] ### Current controller of the inverter of module 2
% the variant selects the C-Caller dq_vector_pi_ccaller when the flag is 0; in module 2 that block is commented
% out, so the voltage reference of the inverter would be zero (the machine is short-circuited through 230 uH)
cc_mod2 = [model '/afe_abc_inv_psm_mod2/INV_control/inverter_control/current_ctrl/current_ctrl/' ...
    sprintf('dq_pi_with_vector_antiwindup\nmod1') '/dq_vector_pi_ccaller'];
if ~param.psm.impl.use_current_controller_from_simulink_module_2 && ~strcmp(get_param(cc_mod2, 'Commented'), 'off')
    warning(['%s init: module 2 selects the C-Caller current controller of the inverter, which is commented out: ' ...
        'the inverter of module 2 gets a zero voltage reference (about %.0f A peak of short-circuit current). ' ...
        'Set param.psm.impl.use_current_controller_from_simulink_module_2 = 1 to use the Simulink controller.'], ...
        model, param.design.i_mod2_peak);
end
clear cc_mod2
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
