%[text] # afe\_uvw\_three\_phase (afe\_rst\_ctrl.slx): model initialization (param structure by subsystem)
% Rev. 01, 2026-09-29. Replaces the flat-workspace init_model.m (kept as init_model_legacy.m).
%
% Everything the model needs is in the struct "param", one group per subsystem:
%   param.sim       simulation settings (stop time, fixed step, solver, module count, thermal model choice)
%   param.time      timing settings shared by all stages, and the PWM data of the stages not in this model
%   param.glb_time  timing_setup object (built from param.time, param.afe.pwm, param.inv.pwm)
%   param.grid      grid emulator inputs;              param.grid.emu     grid_three_phase_emulator object
%   param.frt       FRT test inputs;                   param.frt.data     frt_settings + grid_fault_generator data
%   param.afe       active front end: .pwm .hw_in (hardware numbers) .hw (hw_afe_three_phase_setup object)
%                   .ctrl .uvw (parameter struct of the library blocks afe_uvw_ctrl) .ref .behav .igbt .mosfet .ideal_switch
%   param.inv       machine-side inverter: .pwm .hw_in .hw (hw_inverter_three_phase_setup) .behav .igbt .mosfet .ideal_switch
%   param.psm       machine and its control: .machine_in (machine numbers) .machine (pmsm_setup object)
%                   .mode .impl .ctrl (.ctrl.ekf) and the voltage scaling
%   param.load      torque curve, speed reference, load torque, external inertia
%   param.meas      ADC end scale and quantization
%   param.ctrl      control settings common to all modules (time alignment, filters, PI clip release)
%   param.thermal   heatsink inputs;                   param.thermal.heatsink  liquid_cooled_plate_2kw_setup object
%   param.dclink    DC-link stray inductance (Bode check only, not read by the model)
%   param.dev       names of the semiconductor datasets
%
% Naming rule: the leaf name of the old workspace variable is kept (kp_align -> param.ctrl.kp_align),
% except that inside param.afe and param.inv the afe_/inv_ tag becomes redundant and is dropped
% (fpwm_afe -> param.afe.pwm.fpwm, afe_pwm_phase_shift_mod1 -> param.afe.behav.pwm_phase_shift_mod1).
% Library objects keep their name as a field: hwdata.afe -> param.afe.hw, afe_ctrl -> param.afe.ctrl,
% psm -> param.psm.machine, psm_ctrl -> param.psm.ctrl, frt_data -> param.frt.data, grid_emu -> param.grid.emu.
%
% Library dependence (2026-09-30): the library provides classes and algorithms (timing_setup,
% hw_afe_three_phase_setup, hw_inverter_three_phase_setup, pmsm_setup, ctrl_*_setup, device_*_setup,
% liquid_cooled_plate_2kw_setup...), the model's numbers stay in this file (param.afe.hw_in,
% param.inv.hw_in, param.psm.machine_in, the torque curve in param.load); the *_hwdata, psm_calculus and
% torque-curve functions of the library are no longer called. The device datasets (device_igbt_setup...)
% stay in the library: they describe components, not this model. A function that must be frozen or
% changed for this model only goes in ./private with the same name: MATLAB resolves private functions
% before the path, for the files of this folder only, and the list of such overrides is printed at
% every run (see below).
%
% The last code section ("Transition: legacy workspace names") exports the old names still read by
% afe_inv_psm.slx. migrate_afe_inv_psm.m rewrites the model to param.*; once done, set
% export_legacy_names = false and delete that section.

clear;
[model, options] = init_environment('afe_rst_ctrl');
s = tf('s');
param = struct();

addpath(genpath('C:\github\library\library\masked_models\afe_uvw_ctrl\private'));

% functions overridden for this model: ./private takes precedence over the library
overrides = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
if ~isempty(overrides)
    fprintf('%s init: local overrides in private/: %s\n', model, strjoin({overrides.name}, ', '));
end
clear overrides
%[text] ## Simulation
param.sim.simlength = 1.75;                 % [s] model StopTime
param.sim.tc = 1e-6;                     % [s] fixed step (model FixedStep) and logging base
param.sim.load_step_time = 1.25;            % [s]
param.sim.transmission_delay = 125e-6*2;    % [s]
param.sim.sst_num_of_modules = 2;
param.sim.time_aux_power_supply_fault = 1e3;    % [s] never, in this run

% one/two modules
param.sim.number_of_modules = 1;
param.sim.enable_two_modules = param.sim.number_of_modules;

% power stage model: MOSFET with thermal model, IGBT with thermal model, or ideal switch
param.sim.use_mosfet_thermal_model = 0;
param.sim.use_thermal_model = 0;

param.sim.nonlinear_iterations = 3;         % Simscape solver
if (param.sim.use_mosfet_thermal_model || param.sim.use_thermal_model)
    param.sim.nonlinear_iterations = 5;     % Simscape solver
end

% white noise power used by the module behaviour settings (clock and PWM phase noise)
param.sim.wnp = 0;

% stages of the shared init template not present in afe_inv_psm.slx (DAB, CLLC, PSFBC, ISOP rail,
% single-phase inverter, UPQC, IM, battery, load transformer): see add_template_extras at the end
param.sim.build_template_extras = true;
%[text] ## Timing
param.time.fpwm = 4e3;                              % [Hz] base PWM frequency
param.time.t_measure = param.sim.simlength;         % [s]
param.time.tc_factor = 1/param.time.fpwm/param.sim.tc/2;    % ts_afe/tc, so that glb_time.tc = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;
param.time.trgo_th_generator = 0.025;

% stages not in this model, needed by the shared timing object only
param.time.fpwm_single_phase_inv = param.time.fpwm;
param.time.trgo_single_phase_inv = 1;               % double update
param.time.dead_time_single_phase_inv = 3e-6;
param.time.fpwm_dab = 3*param.time.fpwm;
param.time.trgo_dab = 1;                            % double update
param.time.dead_time_dab = 2e-6;
param.time.fpwm_psfbc = param.time.fpwm;
param.time.trgo_psfbc = 1;                          % double update
param.time.dead_time_psfbc = 3e-6;
param.time.fpwm_isop_rail = param.time.fpwm;
param.time.trgo_isop_rail = 1;                      % double update
param.time.dead_time_isop_rail = 3e-6;
param.time.fpwm_cllc = 3*param.time.fpwm;
param.time.trgo_cllc = 0;                           % single update
param.time.dead_time_cllc = 2e-6;
%[text] ### AFE and inverter PWM
param.afe.pwr_nom = 250e3;                          % [W]
param.afe.pwm.fpwm = param.time.fpwm;               % [Hz]
param.afe.pwm.trgo = 1;                             % double update
param.afe.pwm.dead_time = 3e-6;                     % [s]

param.inv.pwr_nom = 250e3;                          % [W]
param.inv.pwm.fpwm = param.time.fpwm;               % [Hz]
param.inv.pwm.trgo = 1;                             % double update
param.inv.pwm.dead_time = 3e-6;                     % [s]
%[text] ### Global timing object
t = param.time;
param.glb_time = timing_setup(param.afe.pwm.fpwm, param.afe.pwm.trgo, param.inv.pwm.fpwm, param.inv.pwm.trgo, ...
    t.fpwm_single_phase_inv, t.trgo_single_phase_inv, t.fpwm_dab, t.trgo_dab, ...
    t.fpwm_psfbc, t.trgo_psfbc, t.fpwm_isop_rail, t.trgo_isop_rail, t.fpwm_cllc, t.trgo_cllc, ...
    t.t_measure, t.tc_factor, t.tc_decimation, t.delay_pwm, param.afe.pwm.dead_time, param.inv.pwm.dead_time, ...
    t.dead_time_single_phase_inv, t.dead_time_dab, t.dead_time_psfbc, t.dead_time_isop_rail, t.dead_time_cllc);
clear t
%[text] ## Grid
param.grid.grid_nominal_power = 1000e3;             % [W]
param.grid.application_voltage = 690;               % [V] 690, 480 or 400
param.grid.grid_nominal_current = param.grid.grid_nominal_power/param.grid.application_voltage/sqrt(3);

% coupling transformer Dyn11
param.grid.delta_star = 1;

if param.grid.application_voltage == 690
    param.grid.us1 = 690; param.grid.us2 = 690; param.grid.fgrid = 50;
    param.grid.eta = 95; param.grid.ucc = 5;
    param.grid.p_iron = 1800;
elseif param.grid.application_voltage == 480
    param.grid.us1 = 480; param.grid.us2 = 480; param.grid.fgrid = 60;
    param.grid.eta = 95; param.grid.ucc = 5;
    param.grid.p_iron = 1400;
else
    param.grid.us1 = 400; param.grid.us2 = 400; param.grid.fgrid = 50;
    param.grid.eta = 95; param.grid.ucc = 5;
    param.grid.p_iron = 1000;
end

param.grid.n2 = 14;
param.grid.n1 = floor(param.grid.n2*sqrt(3));
param.grid.core_area = 0.05;                        % [m^2]
param.grid.core_length = 2.5;                       % [m]
param.grid.mu0 = 4*pi*1e-7;
param.grid.mur = 10e3;

% magnetizing inductance from the core geometry, line no-load current from it (the delta primary coil Lm1 sees the line voltage)
param.grid.Lm1 = (param.grid.n1^2 * param.grid.mu0 * param.grid.mur * param.grid.core_area) / param.grid.core_length;
param.grid.i1m = sqrt(3)*param.grid.us1/param.grid.Lm1/(2*pi*param.grid.fgrid);

% reference for the voltage sequences
param.grid.up_xi_pu_ref = 1;
param.grid.up_eta_pu_ref = 0;
param.grid.un_xi_pu_ref = 0;
param.grid.un_eta_pu_ref = 0;

% grid impedance (ucc_factor can be preset only if the clear at the top is removed, e.g. in a sweep)
param.grid.Lgrid_base = param.grid.us1/sqrt(3)*param.grid.ucc/100/2/pi/param.grid.fgrid/param.grid.grid_nominal_current;
if exist('ucc_factor', 'var')
    param.grid.ucc_factor = ucc_factor;
else
    param.grid.ucc_factor = 1;
end
param.grid.eq_grid_inductance = param.grid.Lgrid_base*param.grid.ucc_factor;    % [H]
param.grid.eq_grid_resistance = 2e-3;                                             % [Ohm]

% transformer of the grid emulator: block configurable_trafo of the library (Dyn11, one delta primary, star secondary
% with the star point grounded, three-limb core). It reads the per-unit data param.grid.emu.trafo.pu.* (r1, l1, r3,
% l3, lm, lm0, rfe, clock3) that the class three_phase_transformer_setup computes from the nameplate above, next to
% the SI data of one coil (Rd1, Ld1, Lm1, Rfe1, Rd2, Ld2) of the former three single-phase transformers.
param.grid.lm0_pu = 0.5;                            % [pu] zero-sequence magnetizing inductance of the three-limb core
                                                    % (lm0 = lm would be a bank of three single-phase units)
g = param.grid;
param.grid.emu = grid_three_phase_emulator('Dyn11', g.delta_star, g.grid_nominal_power, g.application_voltage, ...
    g.us1, g.us2, g.fgrid, g.eq_grid_inductance, g.eq_grid_resistance, g.eta, g.ucc, g.i1m, g.p_iron, ...
    g.n1, g.n2, g.core_area, g.core_length, g.mur, ...
    g.up_xi_pu_ref, g.up_eta_pu_ref, g.un_xi_pu_ref, g.un_eta_pu_ref, g.lm0_pu);
clear g
%[text] ## FRT
param.frt.test_index = 25;                  % type of fault: index
param.frt.test_subindex = 4;                % type of fault: subindex
param.frt.enable_frt_1 = 0;                 % faults generated from abc
param.frt.enable_frt_2 = 1;                 % faults generated from xi_eta_pos and xi_eta_neg
param.frt.start_time_LVRT = 0.75;           % [s]
param.frt.asymmetric_error_type = 1;        % 0: variant C (two-phase), 1: variant D (single-phase)
param.frt.deepPOSxi = 1;
param.frt.deepPOSeta = -0.4;
param.frt.deepNEGxi = 0.4;
param.frt.deepNEGeta = 0.4;

f = param.frt;
param.frt.data = frt_settings(f.test_index, f.test_subindex, f.asymmetric_error_type, ...
    f.enable_frt_1, f.enable_frt_2, f.start_time_LVRT, f.deepPOSxi, f.deepPOSeta, f.deepNEGxi, f.deepNEGeta);

% grid_fault_generator is a function: it returns the fault waveform data (amplitudes, phase shift,
% error_length, end_time_LVRT, t_ramp, k_lvrt, load levels), merged here into param.frt.data
frt_gen = grid_fault_generator(param.frt.data, f.start_time_LVRT);
for fn = fieldnames(frt_gen)'
    param.frt.data.(fn{1}) = frt_gen.(fn{1});
end
clear f frt_gen fn
%[text] ## AFE
%[text] ### Hardware
% The numbers are the model's own (param.afe.hw_in), the library provides only the setup class
% hw_afe_three_phase_setup (normalization ubez/ibez). Same data sets as three_phase_afe_hwdata of the
% library, selected by the application voltage; DC-link capacitors and their ESR are the same for all.
if param.grid.application_voltage == 690
    param.afe.hw_in.name = 'afe690V';
    param.afe.hw_in.uafe_nom = 690;                 % [V] line-to-line
    param.afe.hw_in.iafe_freq = 50;                 % [Hz]
    param.afe.hw_in.udc = 1070;                     % [V] DC-link
    param.afe.hw_in.LFu = 500e-6;                   % [H] grid-side filter inductor
    param.afe.hw_in.LFu_cm = 500e-6;                % [H] common-mode inductor
    param.afe.hw_in.CFu = 200e-6;                   % [F] filter capacitor
    param.afe.hw_in.Rbrake = 4;                     % [Ohm]
elseif param.grid.application_voltage == 480
    param.afe.hw_in.name = 'afe480V';
    param.afe.hw_in.uafe_nom = 480;
    param.afe.hw_in.iafe_freq = 60;
    param.afe.hw_in.udc = 750;
    param.afe.hw_in.LFu = 330e-6;
    param.afe.hw_in.LFu_cm = 330e-6;
    param.afe.hw_in.CFu = 360e-6;
    param.afe.hw_in.Rbrake = 2.5;
else
    param.afe.hw_in.name = 'afe400V';
    param.afe.hw_in.uafe_nom = 400;
    param.afe.hw_in.iafe_freq = 50;
    param.afe.hw_in.udc = 660;
    param.afe.hw_in.LFu = 330e-6;
    param.afe.hw_in.LFu_cm = 330e-6;
    param.afe.hw_in.CFu = 360e-6;
    param.afe.hw_in.Rbrake = 2.5;
end
param.afe.hw_in.iafe_nom = param.afe.pwr_nom/sqrt(3)/param.afe.hw_in.uafe_nom/0.9;    % [A] at cos(phi) = 0.9
param.afe.hw_in.CFi = 7.2e-3;                       % [F] DC-link capacitance; the two banks of the three-level stages
param.afe.hw_in.CFi1 = 2*param.afe.hw_in.CFi;
param.afe.hw_in.CFi2 = 2*param.afe.hw_in.CFi;
param.afe.hw_in.RCFi = 1e-3;                        % [Ohm] ESR
param.afe.hw_in.RCFi1 = param.afe.hw_in.RCFi/2;
param.afe.hw_in.RCFi2 = param.afe.hw_in.RCFi/2;
param.afe.hw_in.RLFu = 157*0.05*param.afe.hw_in.LFu;        % [Ohm] 5 % of the reactance at 25 Hz
param.afe.hw_in.RLFu_cm = 157*0.05*param.afe.hw_in.LFu_cm;  % [Ohm]
param.afe.hw_in.RCFu = 50e-3;                       % [Ohm]

h = param.afe.hw_in;
param.afe.hw = hw_afe_three_phase_setup(h.name, param.afe.pwr_nom, h.uafe_nom, h.iafe_nom, h.iafe_freq, h.udc, ...
    param.glb_time.fPWM_AFE, h.CFi, h.RCFi, h.CFi1, h.RCFi1, h.CFi2, h.RCFi2, h.LFu, h.RLFu, h.LFu_cm, h.RLFu_cm, ...
    h.CFu, h.RCFu, h.Rbrake);
param.afe.hw.displayInfo(); %[output:7b3b50a3]
clear h
%[text] ### Control (with sequences)
param.afe.ctrl = ctrl_afe_setup(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom);
param.afe.ctrl.kp_udc_ctrl = 2.5;

% resonant PI gains for weak grids
% param.afe.ctrl.res_pi.kp_rpi = 0.5;
% param.afe.ctrl.res_pi.ki_rpi = 18;

% resonant PI gains for LVRT
param.afe.ctrl.res_pi.kp_rpi = 0.6;
param.afe.ctrl.res_pi.ki_rpi = 35;
%[text] ### Current references and parallel operation
% four modules in parallel connected to a dc microgrid
param.afe.ref.ixi_ref_mod1 = -0.85;
param.afe.ref.ixi_ref_mod2 = -0.85;
param.afe.ref.ixi_ref_mod3 = -0.85;
param.afe.ref.ixi_ref_mod4 = -0.85;

% common mode voltage control for hard parallelization
param.afe.ref.en_parallel_mode = 1;
param.afe.ref.u_cm_comp_mod1 = 0;
param.afe.ref.u_cm_comp_mod2 = 0;
param.afe.ref.u_cm_comp_mod3 = 0;
param.afe.ref.u_cm_comp_mod4 = 0;
if param.afe.ref.en_parallel_mode
    param.afe.ref.u_cm_comp_mod2 = -1;
    param.afe.ref.u_cm_comp_mod3 = -1;
    param.afe.ref.u_cm_comp_mod4 = -1;
end

% reactive current steps after the LVRT
param.afe.ref.enable_i_react_pos_steps = 1;
param.afe.ref.time_i_react_pos_ref_1 = 0;   % default
param.afe.ref.time_i_react_pos_ref_2 = 0;   % default
param.afe.ref.i_react_pos_ref_1 = 0;        % default
param.afe.ref.i_react_pos_ref_2 = 0;        % default
param.afe.ref.i_react_pos_ref_3 = 0;        % default

if param.afe.ref.enable_i_react_pos_steps
    param.afe.ref.time_i_react_pos_ref_1 = param.frt.data.start_time_LVRT + param.frt.data.error_length + 0.335;
    param.afe.ref.time_i_react_pos_ref_2 = param.afe.ref.time_i_react_pos_ref_1 + 0.5;
    param.afe.ref.i_react_pos_ref_2 = -param.afe.ref.ixi_ref_mod1*tan(acos(0.95));   % cos(phi) = 0.95
    param.afe.ref.i_react_pos_ref_3 =  param.afe.ref.ixi_ref_mod1*tan(acos(0.95));   % cos(phi) = 0.95
end
%[text] ### Module behaviour (clock mismatch, noise, PWM phase shift)
param.afe.behav.time_gain_module_1 = 1.0002;
param.afe.behav.time_gain_module_2 = 1.0015;
param.afe.behav.time_gain_module_3 = 0.9988;
param.afe.behav.time_gain_module_4 = 1.0020;

param.afe.behav.white_noise_power_mod1 = param.sim.wnp;
param.afe.behav.white_noise_power_mod2 = param.sim.wnp;

param.afe.behav.pwm_phase_shift_mod1 = 0;
param.afe.behav.white_noise_power_pwm_phase_shift_mod1 = 0.0;
param.afe.behav.pwm_phase_shift_mod2 = 0;
param.afe.behav.white_noise_power_pwm_phase_shift_mod2 = 0.0;
param.afe.behav.pwm_phase_shift_mod3 = 0;
param.afe.behav.white_noise_power_pwm_phase_shift_mod3 = 0.0;
param.afe.behav.pwm_phase_shift_mod4 = 0;
param.afe.behav.white_noise_power_pwm_phase_shift_mod4 = 0.0;
%[text] ## Inverter
%[text] ### Hardware
% Same data sets as three_phase_inverter_hwdata of the library (the 480 V set keeps the 400 V machine
% side with a 750 V DC link), class hw_inverter_three_phase_setup for the normalization.
if param.grid.application_voltage == 690
    param.inv.hw_in.name = 'inv690V_250kW';
    param.inv.hw_in.u_inv_nom = 550;                % [V] machine-side line-to-line
    param.inv.hw_in.i_inv_nom = 370;                % [A]
    param.inv.hw_in.udc = 1070;                     % [V] DC-link
    param.inv.hw_in.LFi = 230e-6;                   % [H] machine-side filter inductor
    param.inv.hw_in.LFi_cm = 230e-6;                % [H] common-mode inductor
elseif param.grid.application_voltage == 480
    param.inv.hw_in.name = 'inv480V_250kW';
    param.inv.hw_in.u_inv_nom = 400;
    param.inv.hw_in.i_inv_nom = 470;
    param.inv.hw_in.udc = 750;
    param.inv.hw_in.LFi = 160e-6;
    param.inv.hw_in.LFi_cm = 160e-6;
else
    param.inv.hw_in.name = 'inv400V_250kW';
    param.inv.hw_in.u_inv_nom = 400;
    param.inv.hw_in.i_inv_nom = 470;
    param.inv.hw_in.udc = 660;
    param.inv.hw_in.LFi = 160e-6;
    param.inv.hw_in.LFi_cm = 160e-6;
end
param.inv.hw_in.i_inv_freq = 15.6;                  % [Hz] nominal machine frequency
param.inv.hw_in.RLFi = 157*0.05*param.inv.hw_in.LFi;        % [Ohm]
param.inv.hw_in.RLFi_cm = 157*0.05*param.inv.hw_in.LFi_cm;  % [Ohm]

h = param.inv.hw_in;
param.inv.hw = hw_inverter_three_phase_setup(h.name, param.inv.pwr_nom, h.u_inv_nom, h.i_inv_nom, h.i_inv_freq, ...
    h.udc, param.glb_time.fPWM_INV, h.LFi, h.RLFi, h.LFi_cm, h.RLFi_cm);
param.inv.hw.displayInfo(); %[output:4194a2b3]
clear h
%[text] ### Module behaviour (clock mismatch, noise, PWM phase shift)
param.inv.behav.time_gain_module_1 = 1.0005;
param.inv.behav.time_gain_module_2 = 1.001;

param.inv.behav.white_noise_power_mod1 = param.sim.wnp;
param.inv.behav.white_noise_power_mod2 = param.sim.wnp;

param.inv.behav.pwm_phase_shift_mod1 = 0;
param.inv.behav.white_noise_power_pwm_phase_shift_mod1 = 0.0;
param.inv.behav.pwm_phase_shift_mod2 = 0;
param.inv.behav.white_noise_power_pwm_phase_shift_mod2 = 0.0;
param.inv.behav.pwm_phase_shift_mod3 = 0;
param.inv.behav.white_noise_power_pwm_phase_shift_mod3 = 0.0;
param.inv.behav.pwm_phase_shift_mod4 = 0;
param.inv.behav.white_noise_power_pwm_phase_shift_mod4 = 0.0;
%[text] ## Power semiconductors
param.dev.used_device_igbt = 'infineon_FF1200R17IP5';          % alternative: 'mitsubishi_CM1200DW_34T'
param.dev.used_device_mosfet = 'danfoss_SKM1700MB20R4S2I4';
param.dev.used_device_ideal_switch = 'silicon_high_power_ideal_switch';

% IGBT
param.afe.igbt = device_igbt_setup(param.dev.used_device_igbt, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.inv.igbt = device_igbt_setup(param.dev.used_device_igbt, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);

% MOSFET
param.afe.mosfet = device_mosfet_setup(param.dev.used_device_mosfet, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.inv.mosfet = device_mosfet_setup(param.dev.used_device_mosfet, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);

% ideal switch
param.afe.ideal_switch = device_ideal_switch_setting(param.dev.used_device_ideal_switch, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.inv.ideal_switch = device_ideal_switch_setting(param.dev.used_device_ideal_switch, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
%[text] ## PSM and its control
%[text] ### Machine
% Machine data of psm_calculus of the library (wind generator, six three-phase systems), class pmsm_setup
param.psm.machine_in.name = 'WindGen';
param.psm.machine_in.number_of_systems = 6;
param.psm.machine_in.pwr_nom = 1600e3;             % [W]
param.psm.machine_in.i_nom = 2062;                 % [A]
param.psm.machine_in.rpm_nom = 17.8;               % [rpm]
param.psm.machine_in.number_poles = 104;
param.psm.machine_in.eta = 96;                     % [%]
param.psm.machine_in.Ld = 1.04e-3;                 % [H] per system
param.psm.machine_in.Lq = 1.26e-3;                 % [H] per system
param.psm.machine_in.Jm = 900e3/6;                 % [kg m^2]
param.psm.machine_in.h5_percent = 0;               % [%] 5th and 7th back-EMF harmonics
param.psm.machine_in.h7_percent = 0;

m = param.psm.machine_in;
param.psm.machine = pmsm_setup(m.name, m.number_of_systems, m.pwr_nom, m.i_nom, m.rpm_nom, m.number_poles, ...
    m.eta, m.Ld, m.Lq, m.Jm, m.h5_percent, m.h7_percent);
param.psm.machine.displayInfo(); %[output:5e8021e3]
clear m
param.psm.number_of_systems = 1;
param.psm.n_sys = param.psm.machine.number_of_systems;
%[text] ### Motor voltage to Udc scaling
param.psm.u_psm_scale = 2/3*param.inv.hw.udc_nom/param.psm.machine.ubez;
param.psm.u_psm_scale_ekf = sqrt(3)/2 * 2/3 * param.inv.hw.udc_nom/param.psm.machine.ubez;
%[text] ### Control modes
param.psm.mode.use_torque_curve = 1;                                        % (variant) energy production
param.psm.mode.use_speed_control = 1 - param.psm.mode.use_torque_curve;     % (variant) drive
param.psm.mode.use_mtpa = 1;                                                % (variant)
param.psm.mode.use_psm_encoder = 0;                                         % (variant)
param.psm.mode.use_im_encoder = 1;
param.psm.mode.use_load_estimator = 0;
param.psm.mode.use_estimator_from_mb = 0;                                   % mb: model based
param.psm.mode.use_motor_speed_control_mode = 0;
param.psm.mode.motor_torque_mode = 1 - param.psm.mode.use_motor_speed_control_mode;   % torque curve for wind application
param.psm.mode.time_start_motor_control = 0.25;                             % [s]
%[text] ### C-caller versus Simulink implementation
% observers
param.psm.impl.use_ekf_bemf_module_1 = 1;                                   % (variant)
param.psm.impl.use_ekf_bemf_module_2 = 1;
param.psm.impl.use_observer_from_simulink_module_1 = 0;                     % (variant)
param.psm.impl.use_observer_from_ccaller_module_1 = 0;                      % (variant)
param.psm.impl.use_observer_from_simulink_module_2 = 0;
param.psm.impl.use_observer_from_ccaller_module_2 = 0;

% current controllers
param.psm.impl.use_current_controller_from_simulink_module_1 = 0;           % (variant)
param.psm.impl.use_current_controller_from_ccaller_module_1 = 1;            % (variant)
param.psm.impl.use_current_controller_from_simulink_module_2 = 0;
param.psm.impl.use_current_controller_from_ccaller_module_2 = 0;
%[text] ### Control with EKF based observer
param.psm.ctrl = ctrl_pmsm_setup(param.glb_time.ts_inv, param.psm.machine.omega_bez, param.psm.u_psm_scale, param.psm.machine.Jm_norm);
% param.psm.ctrl.ekf = ekf_pmsm_setup(param.psm.machine.Rs_norm, param.psm.machine.Ls_norm, param.psm.machine.Jm_norm, param.glb_time.ts_inv);
param.psm.ctrl.ekf = ekf_pmsm_setup(param.psm.machine.Rs_norm, param.psm.machine.Ls_norm, 1e6, param.glb_time.ts_inv); %[output:83178410]
param.psm.ctrl.kp_i = 0.25;
param.psm.ctrl.ki_i = 35;
%[text] ## Mechanical load, speed reference and torque curve
param.load.b = param.psm.machine.load_friction_m;
param.load.external_load_inertia = 1;       % alternative: 6*param.psm.machine.Jm_m

% energy production: torque curve of the 1.5 MW generic generator (was the library script
% n_sys_generic_1M5W_torque_curve), total torque divided among the n_sys systems
param.load.rpm_base = [6 7 8 9 10 11 12 13 14 15 16 17 17.8 18.0 18.2];                        % [rpm]
param.load.torque_base = 1e3/param.psm.n_sys*[50 75 100 125 150 175 200 225 275 350 450 600 750 885 885];   % [N*m]
param.load.torque_overload_factor = 1.15;

% drive application: speed reference and load torque
param.load.rpm_sim = 17.8;                  % alternatives: 3000, 18, 15.2
param.load.omega_m_sim = param.psm.machine.omega_m_bez;
param.load.omega_sim = param.load.omega_m_sim*param.psm.machine.number_poles/2;
param.load.tau_load_sim = param.psm.machine.tau_bez/5;  % [N*m]
param.load.b_square = 0;
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;

param.meas.Imax_adc = 1049.835;             % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;

param.meas.Umax_adc = 1500;                 % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%[text] ## Control settings common to all modules
% local time alignment to master time
param.ctrl.kp_align = 0.25;
param.ctrl.ki_align = 18;
param.ctrl.lim_up_align = 0.05;
param.ctrl.lim_down_align = -0.05;

% filters and PI clip release
param.ctrl.CTRPIFF_CLIP_RELEASE = 0.001;
param.ctrl.mavarage_filter_frequency_base_order = 2;   % 2 means 100 Hz, 1 means 50 Hz
param.ctrl.dmavg_filter_enable_time = 0.025;           % [s]
param.ctrl.filters = setup_global_filters(param.glb_time.ts_afe, param.glb_time.ts_inv, param.glb_time.ts_dab, param.glb_time.tc);
%[text] ## AFE control in uvw: parameters of the library blocks (afe\_uvw\_ctrl)
% The five MATLAB Function blocks of the library package masked_models/afe_uvw_ctrl
% (grid_voltage_sequences, frt_vde4110, grid_current_sequences_q_ctrl, current_ctrl_uvw and
% islanding_detector) read one struct: the mask parameter p of every block is param.afe.uvw.
% The field names are those of afe_uvw_ctrl_param_example.m of the library (see the README of the
% package), the numbers come from this init (param.afe.ctrl, param.frt.data, param.ctrl); the FRT
% thresholds were constants inside the old subsystems. This section comes after "Control settings
% common to all modules" because it reads param.ctrl. The blocks inherit the sample time of the
% triggered subsystem afe_control: uvw.ts must be its period.
c = param.afe.ctrl;
uvw = struct();
uvw.ts        = param.glb_time.ts_afe;              % [s] control period: integration step of all blocks
uvw.omega_nom = param.grid.emu.omega_grid_nom;      % [rad/s] nominal grid pulsation: base of omega_pu

% grid voltage sequences: xi-eta of the positive and negative sequence, moving average
uvw.vseq.avg_order    = param.ctrl.mavarage_filter_frequency_base_order;  % window = grid period/avg_order (2: half period)
uvw.vseq.omega_pu_min = 0.1;                        % [pu] limits of the measured pulsation used for the window
uvw.vseq.omega_pu_max = 2;
uvw.vseq.t_enable     = param.ctrl.dmavg_filter_enable_time;   % [s] averages are zero before this time

% grid current sequences and reactive power control: PIs on the averaged sequences
uvw.iseq.avg_order    = uvw.vseq.avg_order;
uvw.iseq.omega_pu_min = uvw.vseq.omega_pu_min;
uvw.iseq.omega_pu_max = uvw.vseq.omega_pu_max;
uvw.iseq.t_enable     = uvw.vseq.t_enable;
uvw.iseq.kp_pos       = c.kp_rc_pos_grid;           % PI of the positive-sequence eta grid current
uvw.iseq.ki_pos       = c.ki_rc_pos_grid;           % [1/s]
uvw.iseq.kp_neg       = c.kp_rc_neg_grid;           % PIs of the negative-sequence xi and eta grid currents
uvw.iseq.ki_neg       = c.ki_rc_neg_grid;           % [1/s]
uvw.iseq.lim_pos_eta  = c.ieta_pos_ref_lim;         % [pu] limits of the stack current references
uvw.iseq.lim_neg_xi   = c.ieta_pos_ref_lim;         % [pu] (the old subsystem used the positive limit here too)
uvw.iseq.lim_neg_eta  = c.ieta_neg_ref_lim;         % [pu]
uvw.iseq.err_sign     = -1;                         % error = -(reference - measured): grid current measured towards the converter

% FRT (VDE-AR-N 4110): voltage-rate observers, fault detection, power limitation, reactive current references
uvw.frt.obs_kx        = c.vr_obs.kx;                % gains of the voltage-rate observers of ctrl_afe_setup (poles [-0.1 -0.4]*omega_base)
uvw.frt.obs_kv        = c.vr_obs.kv;
uvw.frt.t_obs_release = 0.025;                      % [s] observers held at zero before this time
uvw.frt.v_pos_low     = 0.85;                       % [pu] fault when the positive sequence is below this ...
uvw.frt.v_pos_high    = 1.15;                       % [pu] ... or above this ...
uvw.frt.v_neg_max     = 0.1;                        % [pu] ... or the negative sequence is above this
uvw.frt.v_hyst        = 0.1;                        % [pu] band narrowing to leave the fault
uvw.frt.v_pos_dot_max = 2.0;                        % [pu/s] the voltage must also change faster than this
uvw.frt.v_neg_dot_max = 2.0;                        % [pu/s]
uvw.frt.t_rearm       = 0.125;                      % [s] idle time needed before a new detection
uvw.frt.t_hold_max    = 7.625;                      % [s] a fault longer than this ends in the state EXCEEDED
uvw.frt.pwrlim_v_pos  = 0.8;                        % [pu] active power limited (pwr_lim_enable) when v_pos < this ...
uvw.frt.pwrlim_v_neg  = 0.1;                        % [pu] ... or v_neg > this
uvw.frt.pwrlim_hyst   = 0.1;                        % [pu] released when v_pos > pwrlim_v_pos + hyst ...
uvw.frt.pwrlim_v_neg_release = 0.7;                 % [pu] ... and v_neg < this
uvw.frt.reddyn_enabled = 0;                         % reduced dynamic grid support (off, as in the old subsystem)
uvw.frt.reddyn_v_pos  = 0.7;                        % [pu]
uvw.frt.reddyn_v_neg  = 0.2;                        % [pu]
uvw.frt.k_frt_ref     = param.frt.data.k_frt_ref;   % [pu/pu] reactive current per voltage change
uvw.frt.t_avg_prefault = 0.5;                       % [s] window of the pre-fault voltage averages
uvw.frt.t_avg_start   = 0.05;                       % [s] the pre-fault averages start here
uvw.frt.lim_pos_eta   = c.i_grid_pos_eta_lim;       % [pu] limits of the grid current references
uvw.frt.lim_neg_xi    = c.i_grid_neg_xi_lim;
uvw.frt.lim_neg_eta   = c.i_grid_neg_eta_lim;
uvw.frt.lim_pos_eta_red = c.i_grid_pos_eta_red_lim; % [pu] the same with reduced dynamic support
uvw.frt.lim_neg_xi_red  = c.i_grid_neg_xi_red_lim;
uvw.frt.lim_neg_eta_red = c.i_grid_neg_eta_red_lim;
uvw.frt.f_lpf_ref     = 161;                        % [Hz] low pass of the references

% current control in uvw: resonant PI per phase, zero-sequence PI
uvw.ictrl.kp        = c.res_pi.kp_rpi;              % resonant PI kp + ki*s/(s^2 + 2*delta*w*s + w^2)
uvw.ictrl.ki        = c.res_pi.ki_rpi;              % [1/s]
uvw.ictrl.delta_res = 0.005;                        % damping of the resonator. The class value 0.015 was tuned
                                                    % with the forward Euler resonator of the old subsystem (poles
                                                    % outside the unit circle, gain 10.8 at omega); the library block
                                                    % uses forward-backward Euler (poles inside) and 0.005 gives
                                                    % about the same gain at omega (11.1): same current loop as before
uvw.ictrl.omega_res = c.omega_base;                 % [rad/s] resonance pulsation
uvw.ictrl.mag_lim   = 1;                            % [pu] the alpha/beta output vector is scaled down above this length
uvw.ictrl.mag_scale_max = 10;                       % largest scaling factor
uvw.ictrl.out_delay = 1;                            % 1: alpha/beta output one step late (as the old subsystem), 0: no delay
uvw.ictrl.kp_z      = c.kp_is_z;                    % zero-sequence current PI
uvw.ictrl.ki_z      = c.ki_is_z;                    % [1/s]
uvw.ictrl.u_z_lim   = 1;                            % [pu] limit of u_gamma
uvw.ictrl.iz_ref    = 0;                            % [pu] zero-sequence current reference

% islanding detector (passive): U<, U>, f<, f>, ROCOF and 5th/7th harmonic criteria with on-delays.
% Thresholds of the network protection of VDE-AR-N 4105 / 4110 for a 50 Hz grid; the slow U< stage
% (2.7 s) lets the converter ride through a grid fault (frt_vde4110) without an islanding trip.
uvw.island.h_avg_order  = 1;                        % harmonic averaging window = grid period/h_avg_order
uvw.island.omega_pu_min = 0.5;                      % [pu] limits of the measured pulsation used for the window
uvw.island.omega_pu_max = 1.5;
uvw.island.t_enable     = 0.1;                      % [s] no detection before this time
uvw.island.tau_f        = 0.02;                     % [s] low pass of the frequency
uvw.island.obs_poles    = [-20 -80];                % [rad/s] poles of the ROCOF observer (double integrator, as vr_obs)
Ad = [1 uvw.ts; 0 1];
L = acker(Ad', [1 0]', exp(uvw.island.obs_poles*uvw.ts))';
uvw.island.obs_kx = L(1)/uvw.ts;
uvw.island.obs_kv = L(2)/uvw.ts;
uvw.island.u_valid      = 0.5;                      % [pu] frequency and harmonic criteria only above this voltage
uvw.island.u_min        = 0.8;                      % [pu] under-voltage ...
uvw.island.t_trip_u_min = 2.7;                      % [s] ... for this time
uvw.island.u_max        = 1.15;                     % [pu] over-voltage ...
uvw.island.t_trip_u_max = 0.1;                      % [s] ... for this time
uvw.island.f_min        = 47.5/param.grid.fgrid;    % [pu] under-frequency (47.5 Hz) ...
uvw.island.f_max        = 51.5/param.grid.fgrid;    % [pu] over-frequency (51.5 Hz) ...
uvw.island.t_trip_f     = 0.1;                      % [s] ... for this time
uvw.island.rocof_max    = 2.5;                      % [Hz/s] rate of change of frequency ...
uvw.island.t_trip_rocof = 0.1;                      % [s] ... for this time
uvw.island.h_max        = 0.1;                      % [pu/pu] sqrt(u5^2 + u7^2)/u1 above this ...
uvw.island.t_trip_h     = 0.1;                      % [s] ... for this time
uvw.island.latch        = 1;                        % the output stays 1 until enable goes to 0

param.afe.uvw = uvw;
clear c uvw Ad L
%[text] ## Thermal model
% aluminium plate, liquid cooled, sized for PrimePACK2; liquid flow > 28 l/min
% "A" (ambient) is the water: HA is the temperature rise from water to heatsink surface;
% water in/out rise max 5 K with 2 kW total losses
param.thermal.weight = 0.150;                                   % [kg]
param.thermal.no_weight = 0.150/10;                             % [kg] /10: thermal inertia not accounted
param.thermal.cp_al = 900;                                      % [J/(kg K)] specific heat, aluminium
param.thermal.heat_capacity_hs = param.thermal.cp_al * param.thermal.weight;   % [J/K]
param.thermal.thermal_conductivity_al = 160;                    % [W/(m K)] aluminium
param.thermal.Rth_switch_HA = 15/1000;                          % [K/W]
param.thermal.Rth_mosfet_HA = param.thermal.Rth_switch_HA;      % [K/W]
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;       % [K/W]
param.thermal.Tambient = 40;                                    % [degC] water temperature
param.thermal.DThs_init = 0;                                    % [K]

t = param.thermal;
param.thermal.heatsink = liquid_cooled_plate_2kw_setup(t.weight, t.no_weight, t.cp_al, t.heat_capacity_hs, ...
    t.thermal_conductivity_al, t.Rth_switch_HA, t.Rth_mosfet_HA, t.Rth_diode_HA, t.Tambient, t.DThs_init);
clear t
%[text] ### Library workaround: thermal sources read the base workspace
% Inside the thermal bridges of the library (masked_models/semiconductors_bridges/*.slx) the temperature
% source has T = DThs_init+Tambient and the initial temperatures read Tambient, instead of the mask
% parameters DThs_init_m / Tambient_m that the instance sets: the two flat names must exist in the base
% workspace. Delete this section once the library is fixed (tools/fix_library_thermal_source_refs.m).
Tambient = param.thermal.Tambient;
DThs_init = param.thermal.DThs_init;
%[text] ## DC-link stray inductance (partial loop inductance)
% same data as the library script parasitic_dclink_data.m; not read by the model, Bode check only
param.dclink.Lstray_dclink = 100e-9;                            % [H]
param.dclink.RLstray_dclink = 10e-3;                            % [Ohm]
param.dclink.C_HF_Lstray_dclink = 15e-6;                        % [F]
param.dclink.R_HF_Lstray_dclink = 22000;                        % [Ohm]
param.dclink.Z_HF_Lstray_dclink = 1/s/param.dclink.C_HF_Lstray_dclink + param.dclink.R_HF_Lstray_dclink;
param.dclink.Z_LF_Lstray_dclink = s*param.dclink.Lstray_dclink + param.dclink.RLstray_dclink;
param.dclink.Z_Lstray_dclink = param.dclink.Z_HF_Lstray_dclink*param.dclink.Z_LF_Lstray_dclink/ ...
    (param.dclink.Z_HF_Lstray_dclink + param.dclink.Z_LF_Lstray_dclink);
param.dclink.ZCFi = 7/s/param.afe.hw.CFi;
param.dclink.sys_dclink = minreal(param.dclink.ZCFi/(param.dclink.ZCFi + param.dclink.Z_Lstray_dclink));
figure; bode(param.dclink.sys_dclink, param.dclink.Z_Lstray_dclink, options); grid on %[output:65c69798]
%[text] ## Template extras (not used by afe\_inv\_psm.slx)
if param.sim.build_template_extras %[output:group:7a6c4b56]
    param = add_template_extras(param, options); %[output:4ab8b4aa] %[output:6debe77d] %[output:32ef08ab] %[output:6ac8e065] %[output:98c8ccfa]
end %[output:group:7a6c4b56]
%[text] ## Model setup
open_system(model);
%[text] ### C-Caller types
ctypes = {'mavgflt_output_t', 'dsmavgflt_output_t', 'mavgflts_output_t', 'bemf_obsv_output_t', ...
    'bemf_obsv_load_est_output_t', 'dqvector_pi_output_t', 'sv_pwm_output_t', 'sv_pwm_cm_output_t', ...
    'global_state_machine_output_t', 'first_harmonic_tracker_output_t', 'dqpll_thyr_output_t', ...
    'dqpll_grid_output_t', 'rpi_output_t', 'phase_shift_flt_output_t', 'sogi_flt_output_t', ...
    'linear_double_integrator_observer_output_t'};
for k = 1:numel(ctypes)
    Simulink.importExternalCTypes(model, 'Names', ctypes(k));
end
clear ctypes k
%[text] ### Scopes closed at start
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### CREATE\_POWER\_METER\_BUS
% Simulink.Bus for output "m" of the MATLAB Function block three_phase_power_meter_iec
% (Symbols pane: Type = Bus: PowerMeterIecBus). Bus objects stay in the base workspace.
names = {'Q1p', 'Q1n', 'P1n', 'P1p', 'U1n', 'U1p', ...
    'i_act_1n', 'i_react_1n', 'i_act_1p', 'i_react_1p'};

elems = Simulink.BusElement.empty(0,1);
for k = 1:numel(names)
    e            = Simulink.BusElement;
    e.Name       = names{k};
    e.Dimensions = 1;
    e.DataType   = 'double';
    e.Complexity = 'real';
    elems(k,1)   = e;
end

PowerMeterIecBus          = Simulink.Bus;
PowerMeterIecBus.Elements = elems;
assignin('base', 'PowerMeterIecBus', PowerMeterIecBus);
clear names elems e k
%[text] ## Local functions
function param = add_template_extras(param, options)
% Parts of the shared init template not used by afe_inv_psm.slx, kept so that nothing of the
% legacy init_model.m is lost. Enabled by param.sim.build_template_extras = true.
% Same convention: one group per stage (param.spinv, param.dab, param.psfbc, param.cllc,
% param.isop_rail, param.upqc, param.im, param.battery, param.trafo_load), each with .hw, .ctrl, devices.

s = tf('s');
ts = param.glb_time.ts_inv;

% ---------------------------------------------------------------- single-phase inverter
% global behaviour: system identification versus normal operation
param.spinv.system_identification_enable = 0;
param.spinv.frequency_set = 50;             % default
if param.spinv.system_identification_enable
    param.spinv.frequency_set = 300;
end
param.spinv.omega_set = param.spinv.frequency_set*2*pi;

param.spinv.rpi_enable = 0;                 % use RPI otherwise DQ PI
param.spinv.impl.use_current_controller_from_ccaller_mod1 = 1;
param.spinv.impl.use_phase_shift_filter_from_ccaller_mod1 = 1;
param.spinv.impl.use_sogi_from_ccaller_mod1 = 1;

param.spinv.impl.use_single_phase_inverter_based_FHT = 0;
param.spinv.impl.use_single_phase_inverter_based_SOGI = 0;
param.spinv.impl.use_single_phase_inverter_based_PHSH = 0;
param.spinv.impl.use_single_phase_inverter_based_SOGI_ccaller = 1;
param.spinv.impl.use_single_phase_inverter_based_PHSH_ccaller = 0;

param.spinv.impl.use_system_identification_based_FHT = 0;
param.spinv.impl.use_system_identification_based_SOGI = 0;
param.spinv.impl.use_system_identification_based_PHSH = 0;
param.spinv.impl.use_system_identification_based_SOGI_ccaller = 0;
param.spinv.impl.use_system_identification_based_PHSH_ccaller = 1;

% current references
param.spinv.ref.iph_grid_pu_ref_1 = 1/3;
param.spinv.ref.iph_grid_pu_ref_2 = 1/3;
param.spinv.ref.iph_grid_pu_ref_3 = 1/3;
param.spinv.ref.time_step_ref_1 = 0.025;
param.spinv.ref.time_step_ref_2 = 0.5;
param.spinv.ref.time_step_ref_3 = 1;

param.spinv.pwr_nom = 225e3;
param.spinv.hw = single_phase_inverter_hwdata(param.grid.application_voltage, param.spinv.pwr_nom, param.glb_time.fPWM_INV);

% ---------------------------------------------------------------- rail catenary and ISOP rail
param.isop_rail.catenary_nominal_power = 2*5.4e6;   % per 12 km
param.isop_rail.catenary_nominal_voltage = 3000;
param.isop_rail.catenary_maximum_voltage = 4000;
param.isop_rail.catenary_nominal_current = param.isop_rail.catenary_nominal_power/param.isop_rail.catenary_nominal_voltage;
param.isop_rail.pwr_nom = 250e3;                    % nominal per system
param.isop_rail.number_of_systems = 4;
param.isop_rail.full_bridge_application_voltage = param.isop_rail.catenary_maximum_voltage/param.isop_rail.number_of_systems;
param.isop_rail.hw = isop_rail_aux_application(param.isop_rail.full_bridge_application_voltage, ...
    param.isop_rail.pwr_nom, param.glb_time.fPWM_ISOP_RAIL);

% ---------------------------------------------------------------- isolated DC/DC stages
uapp = param.grid.application_voltage;
param.dab.pwr_nom = 250e3;
param.dab.fres = param.glb_time.fPWM_DAB/5;
param.dab.hw = single_phase_dab_hwdata(uapp, param.dab.pwr_nom, param.glb_time.fPWM_DAB, param.dab.fres);
param.three_phase_dab.pwr_nom = 3*param.dab.pwr_nom;
param.three_phase_dab.hw = three_phase_dab_hwdata(uapp, param.three_phase_dab.pwr_nom, param.glb_time.fPWM_DAB, param.dab.fres);
param.psfbc.pwr_nom = 275e3;
param.psfbc.hw = single_phase_psfbc_hwdata(uapp, param.psfbc.pwr_nom, param.glb_time.fPWM_PSFBC);
param.cllc.pwr_nom = 250e3;
param.cllc.fres = param.glb_time.fPWM_CLLC*1.2;
param.cllc.hw = single_phase_cllc_hwdata(uapp, param.dab.pwr_nom, param.glb_time.fPWM_CLLC, param.cllc.fres);  % legacy init passes dab_pwr_nom (same value)
% modifications
% param.afe.hw.CFi = 2 * param.psfbc.hw.Cdc_dc1;

% ---------------------------------------------------------------- UPQC series transformer
param.upqc.name = 'UPQC Series Transformer';
param.upqc.pwr_nom = 3*125e3;
param.upqc.u1_nom = 400;
param.upqc.u2_nom = 690;
param.upqc.f_nom = 50;
param.upqc.eta = 98;
param.upqc.ucc = 4;
param.upqc.p_iron = 5e3;
param.upqc.n12 = param.upqc.u1_nom/param.upqc.u2_nom;
param.upqc.n2 = 8;
param.upqc.n1 = param.upqc.n12*param.upqc.n2;
param.upqc.core_area = 0.04;
param.upqc.core_length = 0.25;
param.upqc.mur = 35e3;
param.upqc.Lm1 = (param.upqc.n1^2 * param.grid.mu0 * param.upqc.mur * param.upqc.core_area) / param.upqc.core_length;
param.upqc.i1m = param.upqc.u1_nom/sqrt(3)/param.upqc.Lm1/param.upqc.f_nom/2/pi;
param.upqc.delta_star = 0;

u = param.upqc;
param.upqc.st = three_phase_transformer_setup(u.name, u.delta_star, u.pwr_nom, u.u1_nom, u.u2_nom, u.f_nom, ...
    u.eta, u.ucc, u.i1m, u.p_iron, u.n1, u.n2, u.core_area, u.core_length, u.mur);
param.upqc.ctrl.kp_p = 1;
param.upqc.ctrl.ki_p = 35;
param.upqc.ctrl.kp_n = 1;
param.upqc.ctrl.ki_n = 35;
param.upqc.ctrl.lim = 4;

% ---------------------------------------------------------------- induction machine
param.im.machine = im_calculus();
param.im.u_im_scale = 2/3*param.inv.hw.udc_nom/param.im.machine.ubez;
param.im.u_im_scale_ekf = (2/3)^2 * param.inv.hw.udc_nom/param.im.machine.ubez;
param.im.ctrl = ctrl_im_setup(ts, param.im.machine.omega_bez, param.im.u_im_scale, param.im.machine.Jm_norm);
param.im.ctrl.ekf = ekf_im_setup(param.im.machine.alpha_norm, param.im.machine.beta_norm, param.im.machine.gamma_norm, ...
    param.im.machine.sigma_norm, param.im.machine.mu_norm, param.im.machine.Lm_norm, param.im.machine.Jm_norm, ts);

% ---------------------------------------------------------------- DC/DC control
param.dcdc.kp_udc = 0.5;
param.dcdc.ki_udc = 18.0;
param.dcdc.kp_idc = 0.5;
param.dcdc.ki_idc = 18.0;
d = param.dcdc;
param.dab.ctrl = ctrl_dab_setup(d.kp_udc, d.ki_udc, d.kp_idc, d.ki_idc);
param.psfbc.ctrl = ctrl_dab_setup(d.kp_udc, d.ki_udc, d.kp_idc, d.ki_idc);
param.cllc.ctrl = ctrl_cllc_setup(d.kp_udc, d.ki_udc, d.kp_idc, d.ki_idc);
param.isop_rail.ctrl = ctrl_isop_rail_setup(d.kp_udc, d.ki_udc, d.kp_idc, d.ki_idc);
% special settings
param.psfbc.ctrl.kp_idc = 1;
param.dab.ctrl.ki_udc = 35;
param.dab.ctrl.kp_idc = 0.02;
param.dab.ctrl.ki_idc = 1.0;

% ---------------------------------------------------------------- resonant PI
param.spinv.pres_ctrl.kp_rpi = 0.75;
param.spinv.pres_ctrl.ki_rpi = 45;
param.spinv.pres_ctrl.delta_rpi = 0.025;
param.spinv.pres_ctrl.omega_set = param.spinv.omega_set;
w = param.spinv.pres_ctrl.omega_set;
dr = param.spinv.pres_ctrl.delta_rpi;
param.spinv.pres_ctrl.res_nom = s/(s^2 + 2*dr*w*s + w^2);
param.spinv.pres_ctrl.Ares_nom = [0 1; -w^2 -2*dr*w];
param.spinv.pres_ctrl.Aresd_nom = eye(2) + param.spinv.pres_ctrl.Ares_nom*ts;
param.spinv.pres_ctrl.a11d = 1;
param.spinv.pres_ctrl.a12d = ts;
param.spinv.pres_ctrl.a21d = -w^2*ts;
param.spinv.pres_ctrl.a22d = 1 - 2*dr*w*ts;         % legacy init writes it to apres_ctrl (typo)
param.spinv.pres_ctrl.Bres = [0; 1];
param.spinv.pres_ctrl.Cres = [0 1];
param.spinv.pres_ctrl.Bresd = param.spinv.pres_ctrl.Bres*ts;
param.spinv.pres_ctrl.Cresd = param.spinv.pres_ctrl.Cres;

% ---------------------------------------------------------------- SOGI
param.spinv.sogi_delta = 1;
param.spinv.kepsilon = 2;
param.spinv.sogi = sogi_filter(param.spinv.omega_set, param.spinv.sogi_delta, param.spinv.kepsilon, param.glb_time.ts_afe);

% ---------------------------------------------------------------- DQ PI current control
param.spinv.dqvector_pi.kp_inv = 0.5;
param.spinv.dqvector_pi.ki_inv = 45;
param.spinv.dqvector_pi.pi_ctrl = param.spinv.dqvector_pi.kp_inv + param.spinv.dqvector_pi.ki_inv/s;
param.spinv.dqvector_pi.pid_ctrl = c2d(param.spinv.dqvector_pi.pi_ctrl, ts);
param.spinv.dqvector_pi.plant = 1/(s*param.grid.emu.trafo.Ld1 + 1);
param.spinv.dqvector_pi.plantd = c2d(param.spinv.dqvector_pi.plant, ts);
param.spinv.dqvector_pi.G = param.spinv.sogi.fltd.alpha * param.spinv.dqvector_pi.pid_ctrl * param.spinv.dqvector_pi.plantd;
figure; margin(param.spinv.dqvector_pi.G, options); grid on

% ---------------------------------------------------------------- single-phase inverter control
param.spinv.ctrl = ctrl_single_phase_inverter_setup(ts, param.spinv.pres_ctrl.omega_set, ...
    param.spinv.dqvector_pi.kp_inv, param.spinv.dqvector_pi.ki_inv, param.spinv.pres_ctrl.kp_rpi, ...
    param.spinv.pres_ctrl.ki_rpi, param.spinv.pres_ctrl.delta_rpi);

% ---------------------------------------------------------------- devices of the other stages
dev = param.dev;
param.dab.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_DAB, param.dab.hw.udc1_nom);
param.psfbc.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom);
param.cllc.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_CLLC, param.cllc.hw.udc1_nom);
param.isop_rail.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_ISOP_RAIL, param.isop_rail.hw.udc1_nom);

param.dab.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_DAB, param.dab.hw.udc1_nom);
param.psfbc.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom);
param.cllc.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_CLLC, param.cllc.hw.udc1_nom);
param.isop_rail.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_ISOP_RAIL, param.isop_rail.hw.udc1_nom);

param.dab.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_DAB, param.dab.hw.udc1_nom);
param.psfbc.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom);
param.cllc.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_CLLC, param.cllc.hw.udc1_nom);
param.isop_rail.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_ISOP_RAIL, param.isop_rail.hw.udc1_nom);

% ---------------------------------------------------------------- diode rectifier
% ABB 5SDF 0131Z0401 (slow recovery)
param.diode.rectifier.Vf = 0.977;           % [V]
param.diode.rectifier.Rdon = 22e-6;         % [Ohm]
param.diode.rectifier.Cj = 0;               % [F]
param.diode.rectifier.Irm = -75;            % [A]
param.diode.rectifier.didt = -30;           % [A/us]
param.diode.rectifier.trr = 5;              % [us]
param.diode.rectifier.Qrr = 325e-6;         % [C]
param.diode.rectifier.Ifm = 2000;           % [A]
param.diode.rectifier.Vr = -50;             % [V]
param.diode.rectifier.Err = 15e-3;          % [J]
param.diode.rectifier.Rth_JC = 0.004;       % [K/W]
param.diode.rectifier.Rth_CH = 0.003;       % [K/W]
param.diode.rectifier.Rsnubber = 1e4;       % [Ohm]
param.diode.rectifier.Csnubber = 1e-12;     % [F]

% ---------------------------------------------------------------- lithium-ion battery
param.battery.nominal_battery_voltage_1 = param.afe.hw.udc_nom;   % alternatives: param.cllc.hw.udc1_bez, param.dab.hw.udc1_bez
param.battery.nominal_battery_voltage_2 = param.afe.hw.udc_nom;   % alternatives: param.cllc.hw.udc2_bez, param.dab.hw.udc2_bez
param.battery.nominal_battery_power = 250e3;
% alternative: both voltages catenary_nominal_voltage*1.5, power catenary_nominal_power
param.battery.initial_battery_soc = 0.85;
% lithium_ion_battery_setup takes the charge capacity [Ah] as third argument.
% ASSUMPTION (not in the legacy init): 1C rating, capacity = rated current x 1 h
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;
param.battery.charge_capacity_2 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_2;

b = param.battery;
param.battery.pack_1 = lithium_ion_battery_setup(b.nominal_battery_voltage_1, b.nominal_battery_power, ...
    b.charge_capacity_1, b.initial_battery_soc, param.glb_time.ts_dab);
param.battery.pack_2 = lithium_ion_battery_setup(b.nominal_battery_voltage_2, b.nominal_battery_power, ...
    b.charge_capacity_2, b.initial_battery_soc, param.glb_time.ts_dab);

% special settings for simulation
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_2.R0 = param.battery.pack_2.R0/2;
param.battery.pack_2.R1 = param.battery.pack_2.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/200;
param.battery.pack_2.C1 = param.battery.pack_2.C1/200;

% ---------------------------------------------------------------- load transformer and RL load
param.trafo_load.name = 'Load Single Phase Transformer';
param.trafo_load.pwr_nom = 225e3;
param.trafo_load.u1_nom = 400;
param.trafo_load.n1 = 50;
param.trafo_load.n2 = 1;
param.trafo_load.u2_nom = param.trafo_load.u1_nom/param.trafo_load.n1*param.trafo_load.n2;
param.trafo_load.f_nom = param.spinv.frequency_set;     % alternative: 50
param.trafo_load.eta = 98;
param.trafo_load.ucc = 5;
param.trafo_load.i1m = 10;
param.trafo_load.p_iron = 2e3;

tl = param.trafo_load;
param.trafo_load.output_transformer = single_phase_transformer_setup(tl.name, tl.pwr_nom, tl.u1_nom, ...
    tl.u2_nom, tl.n1, tl.n2, tl.f_nom, tl.eta, tl.ucc, tl.i1m, tl.p_iron);

param.trafo_load.uload = 2;
param.trafo_load.rload = param.trafo_load.uload / param.trafo_load.output_transformer.i2_nom;
param.trafo_load.lload = 250e-6 / param.trafo_load.output_transformer.n12^2;
% alternative: rload = 0.86/m12_load_trafo^2; lload = 3e-3/m12_load_trafo^2;
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":14.8}
%---
%[output:7b3b50a3]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V\nNominal Voltage: 690 V | Nominal Current: 2.324276e+02 A\nCurrent Normalization Data: 328.70 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:4194a2b3]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:5e8021e3]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:83178410]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:65c69798]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAHkAAABkCAYAAACisp8MAAAQAElEQVR4Aezde6hVVR4H8EVF0Duipz3IEGqyBxJRTVYzUIwQUdPjD6Ohmf4oo6BmikLT0iSNyMaghxY45UQ2E5mNCUKCWTr2+EMqRGTMIstM\/4gyioKa8bPsZ9sz59579vXes8+5d4u\/s9f+rd96\/b7r91trr732unv9t\/435DWwV6r\/DXkN1CAPeYhTqkGuQR4GGhgGTawtuQZ5GGhgGDSxIy15zpw56aSTTtqN8IYBHoPSxI4CedmyZRlYLd24cWMqEh7gyQj3QXV0QQMdA\/LWrVvTe++9l4GdMGFCoYo7g3hAJ7OTU\/+2qoGOAfnII49Md9xxR5\/1bkWmz0yGmUDHgEzvH3zwQTrzzDOzyzYGIy4aTxyZmsproGNA\/uabb9LMmTPTrFmz0ltvvZUWLFiQW8NF33zzzTmOTGbWP6U00DEgf\/vtt7niZ5xxRuK6zz777DRq1KjMu\/LKK\/M1ZPJNLz+snvXzAkHXXnttarWTkCPf6iSPXJQT1zvvvDPXsGxeOdEA\/3QMyAPcrnTIIYekV155JU\/k3n\/\/\/Zw9D5EDg\/Bz\/PHHZw\/E8yhv8+bNCdAHHnhgev7559PFF188CKW2luWQBbm35hctDxAhK8wSx48fn7Zv3x7sFHMDcWR2RfQQAOzEiRPT+vXr08cff5zCK4RVywfh4yHh4F122WWJN1Iu\/kUXXZTrUKw3WfeeSsgb5vB4MHxp3Mujo0D+7LPP0rnnnpsnXgsXLkw33nhjDuOJ60GnTdlfffVVuvzyy3N6Q8CFF16YrYny7r\/\/\/mzlYXEUQTHvvvtutsYZM2akTz75JOeL\/8YbbySyiIXi5chefo466qgcC4Qc2PGzbdu29MADD2TvwqvoSB999FF67rnndsSmXMYNN9ywq2xM7X7xxReTR8iUUk7LW9x1111p6dKlRNLXX3+dPv\/88xx3ySWXpOnTpydpyKh7x4BsHF6xYkWuqEY0kjgyuVUt\/DS6a40F5hdffJGOO+64NHLkyMTigL9hw4asMPMAZYgbPXp0LkUcQHQUJBzKzQIlfuR7xBFHZMvWcXWk7777LqmbeqiPMk444YRduR577LFp\/\/33z\/dcvg7GQh966KHMi59x48bloHlMtEMYs2NA1uPDxWhEI4kjo9JlifIoEWBl05I38St2uocffhi7V9KZCOg0rghAQGStOksRTPG9Ubj0efPmZYtnpb3JF+M6BmTKYK2UqQFFxbo3TpIpVr7VMAWxFj2bG\/30008TN1nkswTuWkcSt3bt2py9NK+99loeI8kbI3mEHNnDDzmPgyeffHI6\/PDDd5NiwYjrZ8n77bdf0gHVT7rg75aocKODuCXv2gp1DMhRWUrWAMAGD+B44oLX17VxTCZ\/3XXXpdNPPz3de++9ebxmVSNGjMjjHVfIzQFg0qRJKawM33O68b0oL78ibdq0add8IuQaLR7fGOtqzDQksHj1khc+S42y8YLCG5mnnH\/++emss85K5getPFZ2HMgx\/ph4RQNj\/Im44Pd0BaQ1bl4hyGMMRUkDuOAXgRDGX7x4cULkyE\/YsZaOj8jgFYmcuCKFnDKVTYYnCm81d+7cXY9WISM9SzWR4nGUK6145bkno22Wd8WdeOKJSZ7yD5koG49Mx4GsQbNnz84rXjEuc6N44jRkqBH3H21lqVOmTMkLQgPVzo4DWcOKPV7P1VPxxA1FCgvVVsQCB7KdHQOy8dYDfV+Na0WmrzyGW3zHgMxSrdZwW9zXL0DsDOGJI7OTU\/+2qoFKQQ7ggIeMRzGrdl8kDRoMVybfoU6VgGxRAICUC7gi4ZlNP\/XUU7utfhm3xNVUXgNtB9nY6xEAsM2AwxNHpnxz6hTNNNB2kI29nvGaVabIa0WmKF+He9ZA20GOqrBo69HcdiOZXHlbFLL1dc80UBnILNp6tCVL7hmZdCHLjtdcc01+Tbho0aJ000035bXjPWvq8E1dGcgs2Xo0UEP9AF++fHmaP39+sqzn3ioXwC34W8AP2fraugYqAznWoc2ko7rWq3\/88ce099575xcJwbdwL9zKYjy57qD21bIykFmo9Wjr0jEm26E5bdq09OGHH+626S46QnSM9qlnaJRUGciN6nvkkUfSKaeckn766afk\/a13uizbAomOoEPoGI3p6vu+NVAZyMbX22+\/PcXkC4Ax9rJYLyVMxpCwiVrfzaklmmmgMpBjfDW5UjEv+adOnZpWr16djMHhwuPqcctkjWxN5TRQGcisVVW5ZFcb7+zKOO+889Jtt92WgM+KkRk4i6+tmabKU2Ugc8\/GWZMtQMfYO3ny5PT2228nwEZzAO5xq7bk0Ei5a2UgqybLNN6yViQc+5sATwbVs2ta6D+1HWTWaHyNsbbxeumllybWzMIjrp5d9x9gKdsL8o4Si9bLJXPFrBi5N\/aeeuqpeXMaHmLh0u1IPuD\/5z3zQqqClv1rUQra\/vozqUhf\/nNaCtr2+J9SkVYu\/keypbiMItoOclSORRtnARs8gHvXPHbs2LxuHZbsyvqlCdmBuv5myfhUBZ3099+noCKIwmtfnpOC1q9cml5fvSbTysUvpFmT\/5xeeumlUs2vDOTG2bVaG3v33Xff5DMUFhykIwz0DkbloWOmLk\/tpr0mvZkaacuU\/yS0eeaXadNf1mRa9cd\/J\/T6pQsSUs+75y5MV111laq3TJWBXJxds1TU09jLwm06HwxL3m\/0b1K76cQxY1Mj\/fqMUQmNHXVoGn\/20Znu\/t3IhB4f\/6uEPGIi33K1jPAOwcpA3lF23ltsvA2LFW429vp0xJcH0tRUXgNtB5k19ratVrzx1xeArBt5hjYha9YByjd5+KVoO8iAsvMDeHZrNqrc87Hvip5++ul6I1+jcvp533aQ1dMXAly0MLCL5MWFccf6tXjEun1N6Oq+pnIaqATkqOKEwodkxl3gPvHEE\/lrf+EAX1iamJELdxR1eGUqBbmoG7NtX+D5OBuorqw9SByZYpo63JoGOgZkrjhcspk0oMOSXU3GyLTWrFqqqIGOAdmEjLWedtppuy1phiX39HhVbEx\/wsXlxHaEe1qutNIVtHnqb1NfVKatlYLMMlkoSy1uvTXrxisSOfJlGteKbCi2Xddmy5WxbBnXVRu+TH1RK20LmcpANotutv3HRj5HHjV+CzVYlrzvXzelrXesHlSyXBm018Q3U5q+Li9XbvzDy3m50pJlkeaf97fUFwWArVwrA7lx+4\/KxmPT0UcfnbcA4Q02WSI0\/g8mWa4skqVLy5XIcmUjLb5lTOqLyuilMpDjccjiR1Q4XlBccMEFqciP+PraPw1UBrLHodk\/nw0CUEuXXlDYMOAe4O0Yk\/untu5K1W+QB6KZwHSskmdiB5BaznRskfE3ZtVxxTMDH4hy+5OHOYROqTOuW7euP1lUlqYykCltzZo16frrr8+uGdhOtTXparZpgFVb867qa8clS5akMWPGpAcffDCfW6n+laFWsuDKQDbxcoCoD9s2bNiQz7YW\/uGHH9IVV1zxf1tybRzw9sqHbyXb2Ku4xzKnzBY7T\/ERzk4VGfh0Z8SIEemwww5LBxxwQFJ\/\/G6gykA28TrooIPyOZHr169PTplzOp2dIe+8807TLblm3QOpVMA6Vdbxh5EvHqs1hHiMs1mB1aoXcEOum66VgWzi5bMYY5wzKFmx4w4dcbjPPvskE69QpLFbmJJZk\/CeEgt+9tlnk5PxbOyP\/FatWpV0Pp3QIx1v47ssp9s6yvj7779PSB0jTadfKwOZYuKVo2MCTaocdXjOOeckExwzbeMwsj3XOdH33HNPduvS7ikpT7k6TmNeOpJOiO8IRB7GIafecTsr295wblt8N1ClIDsFHohFsnxJcStW\/HL2tfDIkSPzWZReT4pvNyn\/sccey4eSx4Gn7a5Df8urDGTu0lhsRm1SZfxztc2Hqy4CLwx8afrb0DLpPMYZh6U5+OCD86kHwt1KlYFMYcY+Y7EzpW3DtSvTbNZZ083A52KlG0wyATQOmz3byKCOrHgwyxzsvCsD2cTG2KeBgPbhufXjLVu25EcUvCL4NuK3w5Ido+xTHXVxzJQ\/GBLjs7p2I1UGMsVdffXVyaSGYj0DsxpXZzhTJqADfBsJ8AaalK0DuUbexn0rbQ6MK\/IjvtuulYFMUawllGimTalm183AN+lph7tWr6FGlYJcXFkyuUImWK6N4Mf9UAOgHe2pDGTjq+dfs2quMcjjUm2xAwt9ZSBrhr95ZFVJGOhhxSy5kcSRIVtTOQ1UBjJrtYoUS5aN1a4tvFEj\/b9vO8iskVWyVIseSNgkzPtk75VfffXV\/Kfo8BF56frfzOGdEsht1QALNu7GGNx4Fdd40oBvk73MqIHuH1RtB1k1vc6zAYCV2lAfS4jiUDGejNd9Xv\/pIOJrKqeBtoMMUC\/+LXqwYqtedoOwUm4ZqLfeems+elE8sune4km5ptXSoYG2g2xNWOExq7b9x5+FxQsyNhujAR6kA+gIIVNfW9dA20FuVjXr1dayjccstxmJ681d8xBcf3SKuHqd2azMbuIZvu677758crA2WkSK+ke7vdgJXvEqviNALlZqT8NeVxY7iY0Be5pnlemBZAfLLbfc0q9qGOYqAdnLhnDH\/pqplxDcN+sbDLes57MAeQuzjJj44cUwwBrUQZw\/icALiCMjjpalx48weRQ8cjYGInzlAoo8GTxEDkU8GWE8skH2mnkb15sXC9li\/sqQn3zbDrLKcr1FayuGxZGJipe9xnO3RgIHSPLQsfw5W++s7SUz2VNubFIg58\/EW4Sxz8s7Zel6ImB4e+WdM7LRAI+8jYEzZszIH9MrF1DibGkS9q780UcfTccccwzxvPPTPjJl6uyZ+fOP9+xA\/vk2X4ptJC9PEbyWNiHt9BdcK7NkFRosKrjr\/AlsdBhLqMZ9rzaBwIPoCBRG+StXrsxVojSKsRqXGT382EZMueSRMECIH3rooXk3ifKUi0fe3nL18bLFfrbRo0cnTxc6iXrZ0CiePGKFgPfK1X1QsY3S8ooR58qix40bl7zZc992S1Zo1WQjHlD0eMR72OZTtl6sRfogllQ2D2DoHEi4bPpGecMJyw+AxQ87kFmFHZixZk4pxi6KoRCWwYK4YvdhjSyxyCdvLmF8x5eHvKRpRuR5DMOCNDyFa1ih\/W48QjEtj2IjBSsv8nsKR\/k2PRRlhh3I3KENCE8++WQ+v9PrTluA7UaxfGofuH1elEtRFA0Qbr3IZyn2iHP7wOF2G5UrfRB54z9QpdEpuG356wCNrjrSsW5WHvc9XXVcbVFPwxCKOcmQAZmyrIw1UzSeODKURLl2oXCzXDXg8QGBJ853T3hI+uDbjB9uOfjigiePyFN5ysUr5kNeWjxegNcApvtG0inweIDIL9LiB8\/QoVx5B7nXtiEDsgZ3GwHOpkH1DjCFiwREHYCFFvmthnWiGuQetMVawjp7ENljNitjbawdmD1lyBP0ty7ykJLE2AAAAEdJREFUrUHuSbNDiF+D3H1glq5xDXJplXVfghrk7sOsdI1rkEurrPsS1CB3H2ala1yDXFpl3ZegBrn7MCtd4xrk0irrvgT\/AwAA\/\/+B9hFoAAAABklEQVQDAIyi8ZlPZRD\/AAAAAElFTkSuQmCC","height":100,"width":121}}
%---
%[output:4ab8b4aa]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\nSingle Phase DAB: Three_phase_DAB_1200V\nNominal Power: 750000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 4.000000e-05 [H] | Internal Tank Cs: 750 [F]\n---------------------------\nSingle Phase PSFBC: PSFBC_1200V\nNominal Power: 275000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls1: 3.030303e-05 [H] | Internal Tank Ls2: 3.030303e-05 [H]\n---------------------------\nSingle Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.580123e-05 [H] | Internal Tank Cs: 4.734509e-06 [F]\n---------------------------\nInduction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:6debe77d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAHkAAABkCAYAAACisp8MAAAQAElEQVR4AezdB7BdN9EAYN1LCxAG0zu80BmaGZoZAjjUkMDQBwgMGELvYDpDCb1j2tDBoYTOUEMfYggMDC3UIZPm9N57\/+8ne9+vJ59z7rmv+sXP472SVqvVSiutpD065w132WWXy2vYc889M+6BD3zg5SeeeOLlZ5999uVPfepTM279+vWXl\/+klZeP7p\/\/\/Ofld7vb3TLtL3\/5y0yKB17ogHx08qXbIOgwGUeLv3rQhkw135IfuhrIrx11uTItH12UbatLGTIHnThcDZP0dfDSTu3Fq5YnaPSv9qIZ\/uAHP0jXuta1kn9C6Uc96lGS8wY3vOEN00iYaX6jTkojQdNDH\/rQ9NnPfnYaL\/L4xz8+vfa1rxWdAWjJRsbIaKP94Ac\/uA3fW9ziFmnU0emud71rFN8m3HnnndN+++23TdkgJKt8dIFrqouMZCVz0InDyYMTSs+mr0cKTEcffTQ2aaTsVMqTkaOfkXLTne9851EspaFG\/+Mf\/0iHH354EkrnnOIHE41Do1FFVpKGl49OeXzgNCxoX\/CCF+Q64EHkCaUD8AtafPALHuJwTbSbNm1KBlPQ1nzr\/KBrCuuyUR98H3oykrWmhZOHn1C6ptGH+hKNvqjzpckhH+gruBpKPsNb3\/rW6UEPelA66aSTMt0555yTfvvb3+b4ve997xkdl5ErPxP3gL7Vx0vV10MSm\/pr1qxJhBjZ8fTHP\/4ROt3kJjfJ4crP3HqAhTFhcGnr69133132gsCwaf2zXrzpTW9KX\/7yl9O\/\/vWvBal4R2PK9Lb1tbWZCV6oPhmy6RZyM9lGhp23Xuy9997pYQ97WNp3330Xqu4djq++1r8l6OumtXk+Oyeb6\/POOy8de+yxqTYZ0n\/+85+n1+v5rHiF1+L1QFay48zofJVudKMbbVPzGWeckY8722TMEwL\/EjZv3pw2F3DQQQelg7ZCiR9T\/Up20QNZyUV64ihzXsNHP\/rR1AT77LNPqqGmq3lZrwLKPHyi7De\/+c30mc98Jm3cuDGDQWHzeEWFY445ZiI9zUnJKitnV8TLmVnGJ5KsB3Hw\/t\/\/\/pdOOOGEdOSRR+az\/vve9770hje8Ib3mNa9JL33pS9MLX\/jC9OxnPzvttddeVwjQLn3fo4syyZyV7Fy9atWqtKoBpqam0lQDrF69Oq2u4I53vGPmccEFFyRK64JyMOVWFD+rVq1KeK0ZHQlve9vbZm8QnKXoZje7WV6SHvKQh6TnPe95+fTA8bCc4OUvf3n605\/+lCZWsg4YDAaNa290UNGPM6Lf\/e53033ve9+k8hqe+cxnpiZ4zGMek8sod7WrXS0dcMAB6dOf\/nT6\/ve\/n8\/oZmUJaes\/AwM\/9bznPe9JH\/nIR9Jb3\/rWXDdnA1m3kqaddtopTU1NpbVr1+ZQ+spXvnLGH3rooennP\/95+uQnP5k2bdqUBxVaA2N7B32WJvyXZ\/I1rnGNZJT\/7Gc\/m1Fc2iHeYX5GxiwSoTQ8X\/nKV2ZfNn8203rQaGNVsrz+9a+f7nCHO+QBQpFf\/\/rXE6BceDP1xje+cTLrjzjiiKw4ynz+85+fHAPvf\/\/7l+yykinPIKHsyCQTeaz1ZAEbN25MG0cgL+jKMOoUlviIw5NJGDihdInvSpd5ET\/99NNTWR7PvpCVzM9pzfre976XOLZ5voT7779\/7ui+zEo6nUR5OmycUimMAsxSSv32t7+dZ\/a6desSfPClIHIJ4YR12pmTY8EMr2f3qlWrEmUDM0K9+ASQmcIBhQODUBu0BV1dJ1wJbfk1vitd5kX8Ote5TtaNdFlfn3hWchBiMBgMcnIw2BLmRI8fHWRW6JRQqlCHRQcFG50Ldh+58ijVLBXWSg362YZrR6ba7Lc8UHDw0c6rX\/3qWeGve93rEjDAyBQ0wmiTNmgLpQtLpaPb3mHIbWnz9MUvfjHtscceM54USWtgWyOscde73vXShz70oWx+KZiim5RqRpZKpVidC9\/Gf77w6qBsYO0NvnbnNjEHH3xwQkMmQK42pWubPqHs3XbbLbd7EZUeok8U5pk8W4\/XJZdcknevp5xyyoxKzQidpqPMUB0n1Hnwsc4IZxTcmoBvWn9qfFe6zIs4h4\/nr+S6zW1us7W2lCjbWdy5267eYGRVyA3IrgzZpwttjZjttdI3bNiQPv\/5z2cnzlay6f0DWeCEZRvLdFN8TmuynfVcPF4UTak6hhJ1SHSMjmrqGI1cSrj2ta+dbM5s1Er5KNtum7IPOOCALKK2odEWbfvSl76ULzU87WlPy7M\/ExU\/lG7AfO1rX0tme5j4r371q8kAKkgXLTqc687ZAGEGKXj30RqrQ8ZJb00sN0w1fVt+je9Kl3kRj82L9Vidq0YbMes1+W3S4ECTsuEBc\/\/IRz4yPec5z8lHuN\/85jd552+mAzQlUDoTT+nvf\/\/7k7IU\/41vfCMf39SFPmQUguifiIfs0ugngWyuJylQ05rJzMtygwsvvDCdf\/752YzqODtuM9uuO9pIAWb2PiN3rJltWWpqp8HylKc8JYGf\/vSnyWw3cJ785Cfno2Dwi5DimXhA4cCsj7W9qY7A6e\/g0zecs5JVpDOOP\/741Bc2jx5AeLolbCoD35Rf47vSZV7EbbDw5f48+eSTs\/OnrJ\/SncOdte9+97tr2jRQNufJj3\/84\/S73\/0u4VmWjTi8iwHO8o94xCPyzt2m9h3veEeyUcN\/munWCKWb7ZRO2THbP\/GJTyTwrW99K4XsnhZuLdY7yEq2Lg8Gs\/N4qYnzwi2SvsDkcbIIm8rAN+XX+K50mRdxHYzvLW95y3SDG9wgWW+b6r\/d7W6XLD0vfvGL89UobQz4z3\/+k\/7whz+kn\/zkJ3mQ1OWjLmGZd6973Suv5TZlZvtXvvKVPACY+KYlzsRRB2AZ3vnOd6aPfexj+RLHTiNvXsjTJ8xKnqvHy1FKxcsFrMncqePkNXidtZscK46dXKOefnnaNY5XnX\/zm988D6TY0JVre5PSLRXg97\/\/ffa\/49dHwWiykkuP169+9Sv4JHSF1UjLiHn8sb6Ux4eadVt+je9Kl3kRj2MIs1zXOS5N2dZZ6zbLF\/RmHFMeO\/KoSxg0QumyzU1pclnX+eWZeDNYnTaHd7nLXbDJYF1WPid6\/AwpEx1XIPPk6Qy3pvAZz3hG5z1l5RYetq8abLKstZRdzzjKNrNd6ZkPqS0nlg1Kfvvb354oXr1nnnnmROyHlKsEZbu454znDpKL5NI8YvLnE5iaOCI08W3Lr\/Fd6TIv4nEMYa6b6u2DC14UoPPNNMcvM1p5ISXz\/tmRS8NHOeG4NJron4iH7NZ6SwUefSGba4WMEhf3OPgVpnzpfffdV3IFWnrAzA5TTtkl2aZNm6ZvyNg9l3mLGc9Knq1bc7aCWk\/K9anm05Zf47vSZV7E57Imh4zBSxg4YSibU8jxSxoemM0sJB8\/iwmnfNkHZbopHrLLU34SyErmteLXLTcUwYSA8iO9Eo7vAUq2djLlzGuUYDHtjm3SDjzwwEAveJiVPNdajK5JQH3OkMKmcvBN+TW+K13mRdxajO9gMEilx6tJhjZc8BI20cCrQ2g95fWi8NqTRtn2PI5fjkboo1xTPGR3XJU\/CcyLks328Pgsh5CHqcnjtVCyOxpxxHR50rg0DznkkLFewxgQi65kTgOjsC9Yr8wkO8amMm35Nb4rXeZF3GxQr3SXx6tJpsApi8dsZL\/KVa6S75w5qnoKxj8RyrIkupkDuC6jjqgvZBdGmb7h0AU6a\/FgMHu3JhPCNPUFgnZ5nNrya3xXusyLuDpL6CtvSRe8SlwZb8sv8SbFrrvuOn0fjSJDYdbtX\/\/619lt+pe\/\/CVFuVLuoO0b5neh5urW7FtZ0OmUOAcGrgzb8mt8V7rMi7jZp14dV9Y3STx4CZvKwatDWOZLl\/hIO6raoIHSuULZ1m3OFQ9UQnblSr594nlNZjbiIl9s8YUL5dbsI9iORmM2l86VaL\/9jvP2Plsfd07q7cJn6K0Co8aI4uV67nOfm99T9lD8ZS97WS+3pntSdol9wfHBHWseoaYybfk1vk6\/\/vWvz7JzywZ4bGewqs\/AFSrHK9VU9zicsnjUsrug7\/MN6vXEqM6vy0lzNN3nPvfJMiv78Y9\/PF8Zsvy5niSkpKD1VogPBHCr8l\/LGwd0O9SoPffcM7+5SNGf+9zncjlXZAiQEy0\/nqQ4Gjj3GSx9gU\/8Va96VeurK235NT7SnuR48O65ay2q57D88OqrQ9arr8xBF3XWZb3Pfe655+bqv\/CFL2zTtrqc9Nve9rYUu2U78A9\/+MO5nLy3vOUtCR8D4bTTTst844dLVd9HelyYzbWH3HZ144jrfBV94AMfyB9TMZKXCrwbNBgMavGm04PBID3gAQ9YUDnda3OlRx+IC8eBS4XTQo4iZjPF1uXiCdR1r3vdZPJNTU2lqRGMivT6P\/RVHJReHRGWYGQawUwQ4Jv1\/YuSxk0JNCXw065ZsybfaxYS0gPvkoaD4JrXvOY0DTrAvJR0yikvrw2udKUrZZF8IYH70AMWEC\/Xy3RjQxtKHt\/5zndmvADXVlcpU8gdZaOMtsgjO\/qynqY4ev1ENudn4X\/\/+9906aWXbtMnjnvyKfhFL3pRvidGF9rjdPTqV786m3xp3rb6oVKeyRjU4DNMBGbOI8+Mf8UrXpE0Ak4FLqeJlwAnDw4tM1nygcf\/6U9\/er7pIA2UQSseoFwsJ4FrC\/Fk3iLfhtKsoHCbl7i0aKDqpNp6qcv5teykWiZ12CDVZaPOvqGdM14mmXOzgaCse+vCvqCvS1nwJJ\/9R\/AYUpyENwCFJajYbNBJ8b0L11841nUUs4Tehg0NWmXgbBAoGK0ycEGnMzWOQKEUHfupT30KWb4mg1\/QkbFsSCYqfswE\/KDQGdGAIskJX4KOwbOc+VEXmTy0J7uy0Ubt0j5yRV+UPCeJ461\/lHEdydEqTLc7aOqV1xfqflXOU0X1iOeZrAFmFUQJNhdmA5wRrlN0Au+MWWF2aLQNm5FTfjlImRrWr1+fZ25Z1gdT0JUjm9Lg0NlQieuUEFq6BHQuvJGvxFOktpUmTAfqSHTeW45Hq3i8+c1vhk4GpcFJqXhAln2hr\/CFnw3grQ5lXQoQRv+qz0CF6wP6Sv+jbWoD\/BARkxbKhAQ6jCdMvA0o1owBtZmNMjrRc2lpA4QpQW8dgQuIPYFG6kA0wKxDw9Xnkah4E6gnjkUxq4NOvS95yUvyCcIAdfaUV1sv7dVu9OjQADh54kBf3fSmNxWdFbBe6lBYv2mnfglc14BWpoQ+bRjGTCoL9okzr+tHMzNomQwjn4ICF6E66jyjVeOseUE3H6HRHBaGlQnTavCQYT7qmAuP0pK08THLzfa2\/Enx2VxPWgi9kW7kmTXWszAZ8prA6GcxdDxgQdBZ8zQ8RmTwQ1MCxVGgMiUoa+01YGwUa5MefJVhLcxI3qVICwOiTfXM1U55QaeO4447dzQygAAADqdJREFULpIThQaaAddVSH1mexdN5GlTxIXkVL5sw6yVjCEgMMHFKRuIB\/gSAQUA5j3w0flMJ8Ga1iSdSXHK1uY9+FC8zYu0uu0L0Acwh\/I0Wh0lvY0eiyTfYHEBXty9a+u1si5TwJUbGe81qQt+Uojdc9tgjsFv30CmcfxZxOhX9Nyfypx11lnp0Y9+dHLVd9ZK1pkExVBn6FShdACzw6szHG6pRj46EGut9dp6CsSVlYdGHTqTgrquBjPJIYvyTYC3OuQFvRFvLVSXZcaAxYdHyeaKJYl6yUEetAYHRxBek4ABxcWqjIFpwImXEBsxslBgmdcWj37VBnsXdDyX3qx0oXBL78O2AMGMiNq1RkDut8suu2y65OMe97jpzwj\/+9\/\/znGV2aSgnSbcGtHZ1uutyfzFXWt7pIU6XceEguBqIAtzjl+dNxgM8rtJZT3o7aRjlkYZneSNBTt1X8+DZ0Fuf\/vbi2YIeca5fDNx9RMnCOhQpngJBpI64MjRtDZ7Teeoo45CkrRBv+fE6MeE4BCK5VPfb6NkmUawnar1y67U1L\/Tne6UP58kDw0Twdvzox\/9KL+4TjlwyqBhwrnoeHZGdacnPOEJmU5eQHSk\/AC8I19IeZQS+V0hfsoAM09nmXFkL8uR06bxhz\/8YZaJ7DrHAwB7h5JWJ3vYjx++bfIYhPoMjTaUPCJeytdGo60+p0F2L\/jrPwMUX3sacrCOrA2+zLv2UCwaMjh3W+bMcLCNkhUEChrZzIZ0CdZKa1RpAgkt3XfDUPKb7\/h8yU6hj33sY\/O5ufT0WfusmeTmxNh5551F5wX6yG6jedWrXjXXZ\/9Q97uBGgPCPqNRySoyApg\/kLkVP86rbH9tcqQ1XicU5IsanS\/ZLVPvfe970z3ucY8svxnElFqTY\/Cb\/TZzmWAefvrK7u3Isrq2fo\/LII1KNitNfealZBZxu2FrAdMcuAhjtxzpxQ7nQ3YKth4ymx5EMIl1O5jTcXuFusy4dF\/ZXR+qeUW\/8zsA+ZYaH3VrVDKCHRU81+W7psCYuXa51m0DP4Apt34uRT955Kh+stQT0amAx4zFcXLwXHrBlWxDolLhUnTIpHW6\/2U904ElmGWT8loK+liPQ3Zyz1nJnokCd5mXolErdY7vgVkp2Vo8GGy5wuvzB8DTone9612JKbGjk4Z3GHdxHBgMrpnaxCwGOEu6v\/y3v\/0tf7NTnTaMF110Uf5jKtIBrgmZxfIDt5Rhl+wuF9Sq5aqllxovPSslx67NWoVJDd5OMLMplRvPsQpQuqs6zqPChf5Mses4dvpuUzg3AnE4L59JB0jDyw\/cUoZdsrtIaEMV\/a6P2zxoaGYo2Rbegh0gjagGdt\/zVcxPPfXU\/Ee8uP+A7bz7w3FYr8tG2k1EfGIE3upWt8ofcXUT0aeKXWrzrpD1McDZvMR3pcu8iNs8KR9hzT\/q6QqDV1vZtvwa35Uu8yIeMrtF4nYs54e+pCObRH0v3QTTSkbMC6SwRRtTaTOyqaAFXT7HCKWsW7curRuBVzd91MzTJc5xIYAjCGgbBBTvg2aerLgB6ms70s6lYO3atYlzYtddd833oLrSZV7EyRzlXSTwsRZ8J4HgFTLUZdvya3xXusyLeMge53KWx2TkzxDv2thmJVOUEcNzEsSYSsuzXW86Ltzznvds0v8MnBkNVq9enQwCQOEUH0DxIFX\/DDBm3vpuTZeuSHbYJGtjMgK66uqIrOTF9mB55dNBnak2ACgexMz3SUNvAYbglBvKdi7cvHlzzgo+QgghvkJQx+NFbmdh9LOBkm9T+bb8Gt+VLvMiHrJLN9XbhctKXmwPlvd5dtlll\/wx8lo4Svf2Bq+NmW6GwwWdXfCznvWsZGbXfMp0UzzeJ7KLDn6ThiXfprJt+TW+K13mRTxkl26qtwuXldxFsJR5lLtu3br87UomXjrkMbN32223RNmBWwmbe2BelMyELDT4jKHvWnE5lr5byrZm2+lPIoPzM7M9SZntgbbvO1CluuesZM+J\/\/rXv+YZZVZx6Otw4EWtv\/\/976l+g96a6mmVsOntfvimfPiLL744cbbwy2qIdd2azeni3O04gQ4Ej4gz9XBeBXWWt0w11d+FC17CJjp4dQjLfOkS35Uu8yIesnPWaPckkJXMUzIYbPFg1YV1ovwaH2lnXTtwHQYOO+ywfLnAw2s3IX7xi18kThPnSqFPFToiuQhnZsQb\/GWoPq+GMM9teBswt048Ww1Z8HUO9hI3XPAIfnVY8y\/raosHj7aybfk1vitd5kU8QvrQtkkgKzk8WGZfWVi6y5OC1iyamppKfSqPwWDmO\/86k5uBXrT2V9q4EfGzyWCe8bTRCKjxLgN6mG+Dtnr1auJkYE1cxMPfQIpyFBN8vbkffMtwXDx41bJFubb8Gt+VLvMiHrJ7Hyo3coKfrGSzkQfLTOMUUV7I9NndSreBmem44015HxoVAuWYVDMNUMLUaDA08fEslFlyHrfGcoRQkHd8m+hrnA6wMQPikW+QWq+BpSTwO1qYlazRDtRcZjwo4Ulx+S6cI2j6gBEOKJRi1468VIDCKd5A8Pbfwx\/+8PwnAgwAtCVvSuf3pnQKp2w4szLOvujrtFn67ne\/Ozlnl8pmHQweu\/EnPelJef\/AkuAxKdR11uXb8mt8V7rMi\/icz8kENXOtm2YQLwq3prRbEvLnEyjgfve7X\/Z5GwCUH7Of0g2SqI9yQ9nMus8pMI3yheV5O9LO2Uw4wBttgJe+LQ2u\/foKQcxyM93ACrq2MOoQNtHAlzIFTY3vSpd5EWe2m\/gG\/65w6MmLtbLNrWkGdDGYrzyKNaMpncJBk8INQrObeR9Xt8G0bt2WczZle3hSl4lZrp2ekpntFC9O8YDyQV12uaSHrs4ulVuTKWrqKHjmifOfsgHTH7RmN6W48svSoJcnDHNexg0gA8ZSZDC7HG\/5KHkqH0DxlKoOQOHAAABMvuOaTaNBAKz\/ynhe7qik\/uAnlA7ZxqVL2ojrj7I8Hn1h6Njh6LO9XszTEEqiFGu5D6bAAc9UbRadx6X7guMIfhwr4S\/39Cxme5vygz+T7+UBijUIAIUbCJ6Tu+EaS0FYBc\/QDUqv2BgUlkf7AgMj+C5UOL3xmksFRtskoC7nUGFTOfimfMp+4hOfmCjbpXl0wHmcCXcEi3LwdZzPGm4wGMz4tia+a0cbRF+NZzUo39+JCA8bnG9kAicJgP84YBEARQJy+tNBBoXHg\/YFlM86GBRvfOMbE788nI\/cuDgA7EM4cMjucey4euv8eVEy81l6dxY6zh354Ac\/OHkmHA0igw2aZ9C8bF0y6Pg+Hq+wbupxmQGY8YDZBxQFzN699947sRDAc1+DAYSM40JyAQOitBIsBDAQDGZ\/IXcSZVPyuLrH5vMlG2V9wczhO7ZjbCrTll\/jrbPRoSGkTWR42dyFUkeUM5PVKx3esKb6u3DK4oEvOkdMYDayMmHyvUO1YcOGtGEErIJlhYXgpXOeZx3I7rinjI\/fGBA2i9GWptBSQcGT+LCHvEbWqMFgdm5NgqjUVr8v6Owuj1Nbfo2X5gFyS0On2Z2TB1C2TRDTyMOmvhr6ylvSqROfElfG2\/Lh9bNlxppvp0\/J7kk7AfhUFgthiTAoWAdxA4OFsFSwDibUJArWF0MXyOfi1sRkUtApXWe+tvwaX6bNMDOiVjYzbi105gfWNh0+qcxBX9YZuDJsy6\/xXWl53MkGrROGgcBX70t\/rlxN+pAi\/63G0q2p83m8hPvvv3\/ScWUjtvd4qWzmvJSXwq3bdsLMp105b9rmzZtLsitcfMaabAQNBlu+bDcYbAkXosV21F1nvrb8Gt+V1hY3QO3EmTmmsmwLc26GUzqT7g19mxrOFoq3+WlSfl1nyVO8Lb\/Gd6XLvIjP6Zxcerz22GOP\/L4utyaQ1gGEX65gZluz7YxZJTtl3wNtao+ZTrkUT9naHso3AKTNfo9R0RkEoInX9oRbEo+XWWY5EDZ1BnxTfo3vSpd5EbfGuZLrTOzhi1lu88Osy2uSBY7yAYWa\/ZTs6RulAwMBGAj86979ks8iAAPC0WgwGOS\/8opnyCSs03DR\/ojbzQcO\/SSwLDxekzRoElqz3E6X0s1yT8hs3EAoX74BgHYcbwMBGAwUyyKAsAoGhIFgQABxAI\/GgADK4oHXuDr75M9Yk\/sUaKJxo5+3qS8weV710KCmMm35Nb4rfeCBB6aoI+i4EuGkzcamurkagbXQLLKWO78y8Uy+u+Zcq46ezrRogJkKrPVNfdSEo0RAoRRrQAAKp3gDwGDgHRN3g9U1IEtOE7823JyU7Myn8QTYa6+9ZnxxtivNVPr+tIsKTXRt+TW+K13mRdwDCvVG2FZ\/k0yB86F37\/z6sIxdOvMMmGfgk1YUBMQBPDNPkcDzAgMC9BkUaAwGzxgMCNZF37cptcbPWckO8W7zr8B+23xP2zN54N6ZP6jJ2QFczrA8ABchWQghgIulwl4B1MuFpWMiJTM7TNJgMDuPl8oc2Ffg\/7\/vPZu+4NYEFGqm2icEULx9QuwZDIJ6tnall8Tj1SXQSl53D5jFBkI31czcbTxeNidIhH0u8qFdge27B6bXZBf5rB82JdyaQrs6T1i27yasSDeuB6aVjJCieboCpOFXYHn3wAwlL++m7DDST9zQFSVP3GXLr8CCK9kDEF\/ZdatyuXQPmZ1P7U0AT9NykZ3jhOOG3MAGekGVTLE+SeFzRculk8jpVoZblvYmXIyu8Ooseds7uA3q88dkt5H2BuhYJVOUmWh01w3UcKMlQDpo0HPtcfm5ohP4xQxnK7tPD7uJQVZPfvzlNfHFhNnKTm7yh6wu\/XUqmaLie9dRKEIKdRXFSI9RI004ND4kozLfa5ZebJiL7KWsLjf4wr1rUiV+IeNzlT1MtmMw\/3yrkimRe25H\/t61znYX2kt0Bu1CKjZ4z0e\/u87lWYKnbK3mWkVGgbvEIASIcLFfq4l6+4TzJTuL5Jaktx4WyyE0X7JHP8UFzf8DAAD\/\/wTZsiwAAAAGSURBVAMAvNBHkVR\/1WwAAAAASUVORK5CYII=","height":100,"width":121}}
%---
%[output:32ef08ab]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAHkAAABkCAYAAACisp8MAAAQAElEQVR4AezdeaidxfkH8PkFEdeYX9yiRkmsNpZUbaqCIS5\/RGmsMeJWjFhcoJoYBalLUauC1mjdUDSJC0oVMVJRtKK1LhRFrSg0VSvW1uW6r5S6oFWRNp+pz+W9x3PuPSf33POec+4b8px33meemXdmvvM887wz886d8J\/qX9+3wIRU\/ev7FqhA7nuIU6pArkAeBy0wDqpYaXIF8jhogXFQxUqTK5B7tgWqghdaoNLkQmP0a7ACuV+RLdSrArnQGP0a7EqQr7nmmrTtttsOIbx+BWGs69VVID\/00EMZWJV+5ZVXUpHwAE9GeDh67rnn0s4775zzkgYdccQR6dNPPx0u2WAcOfLNPEsicp5RpFNPPVVUfmYreeVEbf7pGpDff\/\/99Mwzz2RgFy1a9K1q4gGdzLci6zA22mijdPfdd+f8nn322Szx5JNP5utY\/Gy99dZJ\/sroeW+\/\/XYC9AYbbJBuvfXWtM8++4zFY5vKs2tA3myzzdIpp5wyYqGbkRkpk6LmASLkhWnjwoUL0yeffBLsZKjAR2QGIxoEAHvGGWekF198MQ0MDKTQ5LAQ8kH4eEg4eAcccEBijTwXf++9985lKJabrHvKQf6yyy7LlosFw5eGjDy6BmTtpWIKGYVTQGE8cWSapY8++igdeOCBueI77bRT2muvvbI2yee8887LWh4a5zka5umnn87auHTp0vT666\/nR+E\/+uijiSyioXg5cpifzTffPMcCIQdW\/3zwwQfpggsuyNaF1utIr776arrllltWx6b8jGOPPXbw2ZhvvfVWuv322xNLllLKaVmL008\/Pd1\/\/\/1E0scff5zeeeedHLfvvvum888\/P6cho+xdA7LefOGFFyY9UgOsXLkyV0CFFi9enMSRycwmfmrNtcoC87333ktTp05N06dPTzQO+C+99FJusN122y2xKOJmzpyZnyJOeXQUJByNmwVa+JHvpptumjV79913z2B+\/vnnSdmUQ3k8Y5ttthnMdauttkrrrbdevmfydTAd\/+KLL868+Jk3b14ObrfddinqIYzZNSB\/9tlnypNUUkMraBTy4IMPznEhk29a+NF4GhFgLSQbFPV8nS3o0ksvHYxrFNCZxKmLKwKQ+tFWnaUIpvjhSAdnum+88cas8bR0OPliXNeAXCxUu8MaiLboNMzom2++mZjJIp8mMNfMq7jnn38+F0OaBx98MI+R5DU0i5AjG\/yQY3lmzJiRNtlkkyFSNBgx\/YaEddddNw8lyidd8IckKtzoIG7JuzZDfQty7ZisMY488si04447pnPOOSeP17Rqyy23zOMdU8h6AODMM89MoWX4hgvje1FefkV64403krRMacjVajy+MdbVOGtIoPHKJS98mhrPxgsKa3TcccelOXPmpF122SXxD5qxbl0FsgaIhrrzzjuTCmk0PHFR4ZGugPSqFebV1WuMhpIWcHioCIQw3j333JMQOfKLVr\/S4SMyeEUiJ65IIeeZnk2G6X7kkUeyg3TttdcOvlqFjPQ0lSPF4niutOI9zz0ZdfOWIW7atGlJnvIPmXg2HpmuAbnYACpSSypCRkX6jZh\/nRnp2GeffXZ2ANtVz64B2VgY73YqW0viyLSr4t2UT2hodGwa2M7ydQ3ItJS2qijPsejRujdBQaadlR8veXUNyNHgtJXnCNj\/8VICOJ644FXX5lugVJCLY1GY59tuuy2XnuOVA6t\/4sU\/JgVWs6r\/LbRAKSCbFACqcjLPRVp77bXz1OINN9yQpyTJeX+94oor8gyVNBW11gIdB5nJ9QoAWA5HbXHxxB1++OH5VUPYWF2Nx7Ut1fx9x0EGlne82iIC37x18OvJiCvKuK9o5BboOMhRJKB6LWKOkQmPZcuWZRO9ww475GnEkHWN8duKlPuKmm+B0kCm0V6LeM5MMuJRo\/333z8tWLAgA64DIFUi0+53SPn2O5UGMk32WgTUaGSA4x1zzDFp9uzZg2MycI3VIVddW2uB0kCO16F4PVLseG1af\/31k4UDvP6lztWsNJBNunst8nrEHCMbBeycOOuss5Ilvs41Q38\/qTSQa5v18ssvTxwua6pWTirzXNtCa35fGsjAPPnkk1M4XzT7sMMOy9t8LNqH533XXXel448\/\/lve9ppXefylLA3kWOzmbEWzWzT\/+uuv02mnnVYXfB0jZKtr8y1QGsjheIWzpcicMCBPmDAhL0rgIeC7RscQrqj5FigNZOaZ48XZArTFck7YJZdcktZaa62EF9UAvnB0DOGKmm+B0kBWRBMi5qW9ByNh21brgY+nY0hXUWst0HGQTYKEU+W1qZbEqQLAAY+EdQj8ilpvgc6CvLp8wAIa8Mx2cbyE7UOeOHFi3nBuHrse+DrI6iyq\/y22QMdBjvIBzBQmoPGA\/8ADD+TpzBNOOCE7XsBHZLxqkSFbUWstUBrI4UTVOlhffvlleuqppxJgoyq0XYfQMYLXj9fX\/\/nv9Os\/vJoOWLYqTf75H9MPfvWnfD\/aupYGMieKM8W7DtM8nr1rAC9Z+cJqUAfS4y\/\/K+OK9+s\/DGTQM2MNf0oDWXmZ3xifmWXhonddBF+H0DGk60cCLKpXN3waXi+uGV7HQWZyR9rdAXzryYBHwMdrpkKtynz44Yf5O2LfEqNV\/3gzPfbSv9LKp9\/tGAEQDVf2q37\/18Fy+pZrONnauI6DDCy7O2ip3R61BfKJh7irr756yKYBr1Y6SK38aO8XLP9LNodzV\/w9zb727TR3xT\/SguWrEtPZKQIg0zxSXUwKoTvuuGMk0SHxHQfZ0+3uoKHCAC0SkyzOIoUr4oS1+9MRz0aztt8q7Tt753TSft9PyxZ+L\/3uhFnp4cXbpz8dv+Ug3feTdRMaiVeMj7B0KO5d3Qe5f3jxd9M2k9dRnIbk60jtgA455JCGcvUiSgE5CmI5EYhFwov4uPKufe03FpoMWPSLH01PC3ebkvbYblKatf3U5LPTIO\/tKO5d3SPhGTNmDJEv8sigerwi37OjvvWu4iOPqVOn1hNpyCsV5IalqonwzW4rXzXWJO+JW51szncm1S0rLRdfN7IJZteBTFuNv0UTbvFiPEyG3LNkVvrFj6Zl0w1Y5P4vv5zdBJSNRboOZI4Zb7powoXrmfHG1Wo+xsRDkcLZ4u120sP2LF79nO\/8f1q+YNN0zT7\/l2nvjd7LX5SY9g3qeu+62PxFrY0dIAB2ZIO4kBWu5UXcaK8DAwNpoECP\/vlvCZmECMA7deXVo\/m\/eSP9+Lefp5\/d8U666qqr8kEy9913XwrqCe8aMHZ5FLf\/WEN+7bXX0tFHH52eeOKJwaMZmG0OijQxFSrcLrr5iGmpHj3800kJhRcsHBQ81+A1upJBxXj3QUX+XUdOGSwLZ\/DMn8xOJ510Uj6R4KCDDkpBPeNdxy4PnjPA1llnnXTzzTfnszB23XXXbKKY6SCb+7xekW0nzZo1Kw1HOhgqyrgPKvLrhevJBc+1mGavH34vzd91eibeNCKDinI9412HVsYChYNc5s+fn7766qv07rvvDtFk2swZY7bbCfB4yas0x4tWmo+2QAFom\/ccd3T99denxx57bMjXE7TZWM0pGy\/AtLOeawxyOwoBXMcq8RodQOqYJO\/Epjtpb5HGgybzU3R8r4wvvPBCO5o451EayCq0atWqdNRRR+VNe8B2qq1zJtF11103RJvHgybfe++92T+46KKL8tmY2iijNMqf0kDmeDlA1HlVjkP0WYywik2ZMiUfvzjKunVNcr6Ek2wd3hqFKlorJy\/gv\/zyy\/kbsMmTJyffg2kj\/NFSaSBzvDbccMN8TqQlPqfMOZ1u4403TnvuuWfW7tFWrhvSA9bJtY5YjPLg0VrDFItlXl7ndpQGcEOuXdfSQOZ4+SzG+GPinRY77tCkh7Haklqvj8k0+KabbkpO33Nqb4D2+OOPJx1cR\/fhAItm1c0Juo5L\/uKLLxIydxBpRnMtDWSFjiVHa8g8Z0cdcrAe+eZoQl51EB4Z6XqFlFfd6mmnT3N1dHVxzCIr5iRfbxfO43a+JrMtfrRUKshOgS9qq\/Aee+yRkHAt2WzA1I220t2afvr06clmCZ\/vxqGq7ShraSAzZcZiHrWFcGOTq8ohM2GhxfjItiFHDLej4mXn4YRb47ByTJw4MRmuhMeCSgNZZYxLKsezdho8YHmaCKhkEL4tubxu92NJncibk2kc5j2bF9AOtHisnl0ayJwO45KKAdrB4eZojU14HC9XxBFzNbZFGve9So5qdviN+jrKyh8lifF5LOpUGsgqdeihhyagqjRTrEcvX748IXuwY0w29Wmc6tVjJtSPJXINEBd9c4a2g+uK\/Ihv57U0kFVCT44K8rSjwrxS3nSMycLMmZUojSNtRc23QKkgF2d9Qmu9Qp144olDtuOKw+esNV+1SjJaoDSQAcYM86pDY139bQabB2q97vGwxytAafe1NJBVxN88MuMjXCRjM2es6HUb03SMolwVbq4FSgPZuGuGJzznKG4jr7vft+RG\/cfi2nGQaaPx1TjrNQkJB3m1mDt37re8bjNBOsZYNEK\/5wnkjtYRULxl4289Erfffvulel53RwvaRw\/rOMjazvyzeWjaa9UppvfEVdT+Fug4yAA1\/2zygyabwbITpGjGgV9LTDyZ9jdB\/+fYcZDN12rW8Kpt\/\/FnYfGKVPtqxYwz9UWZKtxcC3Qc5HrFsgWXVw1I2m3HhL8BHNrcjBazEEx\/pImr5cx6z+wlnuHt3HPPTVFHk0hR\/uBZ1Ale8Sq+K0AuFkqYxgbgQPdtsl0kzZhrq1fSBFm0l2evEpDsLlmyZMkaVcEaQSkge+c1b03b\/DVTK1DMt3taC+BwzPDsgbInCvhrUlM9n5bLW5hmRP540Xlog+eJczIvKyCOjDjPlh4\/wuRR8MjZtIfwPRdQ5MngIXIo4skI45ENYtVMCjVT92L+niE\/+XYcZIUFYmhaXFUm9l7707eAjzgLE3pkVHy4a\/G9GzhAIq9jmTK1Nm0vGWdP\/qZLpSFniOAL2INlvVe6RgQMs3DWg5FNAHjkbdpbunRp\/tTHc9VNnJU1YVO2V155Zdpiiy2IJ36KPV6eqbNn5jc\/1tmB\/M1tvigvEBF5eYpgtdQJqae\/4KrdOg6ywoxENtmHpqsIKgI2XPqCuU46k05F3hSqcd\/SJhBYEPlqMI3vqw1yGk3DmI1z34hsI9a45JEwQMhPmjQp7\/TwPM\/FI29vufKYA7CfbebMmXkLrk6iXDY0iiePaCHgTfG6DyrWUVptFXGuNHrevHnJyp77rgFZ5YCiF9YjcWQUerRkkxxQ4jnytgWn1XxpS+ThSpNazQMYOgcSbjV9rbzhhOYHwOK7BmSF6QTRCrsjY85coxi7NIzn0wwaxBS7D22kiUU+eUOK8R1fHvKSph6RZzEMC9KwFK6hhfa7sQjFtCyKxRpaXuQ3Csfza9fcxx3IrIF58BUrVuQ1a8udvj+aNm1a4sXbu+GckQAAAQxJREFUB24PlsbVmBoaIMx6kU9TFi9enJh94JjUqW1c6YPIG\/+BKo1OwWzLXweoNdWRjnbT8rhvdNVx1UU5DUMohri+AVljcdDqNTSeODIaSePahcLEMtWAxwcEnjjfA+Mh6YNvo3yY5eCLC548Ik\/P81y8Yj7kpcVjBVgNYLqvJZ0CjwWI\/CItfvAMHZ4r7yD36tY3IKtwrxHgrLopd4ApXCQg6gA0tMhvNqwTVSA3aC3aEtrZQGTUbFpG22g7MBtlyBKsaVnkW4HcqGX7iF+B3HtgtlziCuSWm6z3ElQg9x5mLZe4ArnlJuu9BBXIvYdZyyWuQG65yXovQQVy72HWcokrkFtust5L8F8AAAD\/\/yKb+AMAAAAGSURBVAMAa5fHma9x33AAAAAASUVORK5CYII=","height":100,"width":121}}
%---
%[output:6ac8e065]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAHkAAABkCAYAAACisp8MAAAQAElEQVR4AezdeexV1XYH8B3jPKJ1VkgcmvrEoUSJGhpNHaKJ8DREE1EjahOHipGoGKkTCTXWAaOJiBprgjXyh3V6xMQXTRpMiRNKHfDV1Cf4QHGKGBHFlDwfnwPrl81553fvOb\/f\/cEFLmH99rT23muv715rD+fce7f5tfdvi9fANqn3b4vXQA\/kLR7ilHog90DeCjSwFQyxsSX\/+OOP6cILL0yHHnpoES5evDiNGzcuffDBB1uBujbPIbYF+dVXXy0AFQL4iiuuSAceeGB68cUX02677Zb22WefdPbZZ6e77rorKa+jBnwmChJX54477ujoRLnxxhvTKaeckr7++mvNJ\/I\/8sgjRXwo\/pjkJ598ctsxGC\/ZQq5WsnRK5rYgn3766enTTz9Nwp9++qmQ6aabbirC+DN+\/PgiGuVFosWfXXfdNT399NMFiVP+Cy+80KLG4IoAcMMNNwyukTa1Z8+enQ4++OB0yCGHtOScNm1aevvtt1vyKOykzH0gm2EsixtGZpvOzCZp4VdffZUWLFiQzjrrrHTBBRcUwn722Wfp1FNPTW+88UY68cQTCxeuLUIee+yxCYDldpTrC7300kvpnnvuSStXrkznnHNOYXH4Uc4nLo9cuYVKkw9pbz0f1oK++eabNGnSpKJ9\/eBnRdpQB8krmNf+Ia88JF9o7GuLCq+Q14t8Y33llVcSSzZp8UZd9UMubT\/33HNp6dKl6YwzziisXh4eRF\/aIl9ZZuPSDj6knn6qqNx3H8jT1s4wFd5\/\/\/302GOPJcLEIOQH7bTTTmnfffdNq1atSt9\/\/30aO3Zs2m677QqQufBFixalp556KtjbhpTGM3D96vMYUYnCKO7zzz9PvITBs4LRo0cXMhiMtAmG8MU4og3LyUMPPVQsLfq57777ikk1YcKEwkPJi7FS8KxZsxLPxHtFG0JKnjx5cjrooIMSHanHO6hj8uMZM2aMoJio2qRHYwqdXHXVVUXbw4cPTyYFZpOc7GjYsGHFsrfzzjunsszGZXz4tGvCVuFTpZNtdBTKo1CKpWiDFCov07nnnpuWLFlSDPakk05Kp512WqH0o48+Or333nvJYMp1BpqmOJOJYhEr4ElC5gDcxAMc5VF8q\/4ADUgTjLKCd\/78+YXFa19ehOI2mMAKHZFLPoA\/+eQT0b8i+xfl\/emEvubOnVsYFC9obH\/VyNqM8liPOeaYZKKU+y3zhU4KkAlCkWvba\/n\/u+++K5Rw9913F5sxnb3++uvp2WefLdLhblo20rDQGjdy5Mj08ssvF2Rw+g2ZDz\/88EYtskhuj1Jvu+22wmvlDfAo++23X55VxPVnSTEpuEtLizRFo4Jp\/R9tI0lA46+yOnnKXnvttcITRh31ctI3fHgH\/PhMiHK\/wVfWSQGyQQ1b6yrKlfKOxPfaa6+0\/fbbp6OOOqpwdSyLJY8YMaI4Rs2YMaNwNxSJvxPEs7AeFvrWW2+lsNy6MpdlCIvkbqs8FeAoq1xPfyaAerxc0FVrXXBZqWS2scSDX1vTp08v1nTxIBPXpH3ggQcSFx355VDf8OF9tBnEI+W8wVfGsQCZWVOe9Y3Jc3essry4s+Q1a9YURxONGwwAVqxYkSjPBHj33XfTzTffnModGpA6A6FwjcuWLUvhQssyk3vOnDnFhoYb7K+fXC6T8Yknnuhj1Q8gyVouC4\/C6pTRDR3RFZDzyZGXXXzxxcWG1FpeBlI9FmpSWWtRnzBZpDzW8ADCjK1YMnMcQycFyBinTZtWbCpOXLtD5ors\/sxSZUE77rhjEX388ceLnaEExaxevTp9+OGH6dJLL0077LBD4gYJZo0MF4O3P9KGMv1SkHhOoWCznquOMjPZoMiMxOVFeYRRn6udPHlyOu+884p1UFsmDmDNfpPj6quv7itzHxBtmNAszuZHPW3xXOqYONEGfnqjP+PBay2fOnVq0oZJytWSl45YKD6bOEcw7dtk5jJbXqZMmdKHjyWAh6jyRMZPD9pH4n0gEyBcDHeAmcAakhaecMIJiTLs\/AhmfRA+\/PDDxZpi7bSzNmh1DVZdpD2hdqIv\/YlTlM2JcnXUzQkP3nnz5hWzNS+LdtUVjzLx4I\/6eLRz7bXXFsuNNMvUt35Zpd01BSpjacCL8QBFm8qQseiP\/EClG2mkfzxI+3jkqyMPcb\/RHh6ySOunLDMZyK4eIq\/2qijvW3ybKqZ2ebmgOpQmmJ1iDKZdG91YTnZgsVITWMiy5beTd+LEicmewURpxzuI8gFVHRDIA+ppM6lk5pu4Qa0sJh+SicAShXl+N8Qbg2zNNMvL5Mxpoe+GQfVk2FADjUAGohsatzjWEzcvZry4TRaXvWHzrVM2PXaUPXqj2NPU1QO9tdbshqWNQFY1NiI2JY4a8mxUuCqTQLoOEdSO0c6xRxcWd\/519HDezTPTMff\/X9rr+v9Kc97+so6qm70ZEuc8xyLHnthoSNv61+pxPROQ33zzzXTvvfcWT6PsHLuFnJ3vv\/\/+9OSTT3adbNddd916DaY0fM91R9q+jH4ijSzZtv6BtbczrNaW3sxzhLILjbNxP\/0UT6Pwu0jIeVwSOM91G7noOP7444uLjG6SzVk611+deCOQNWjdZXFCO09rMnKMUl5FjhXOn1Vlvbyh10AjkK25rFGYiyZdlY+H5bo8ueSSSyR7tAk0UAtkIDoicVt2gML8CCVN9lizxYPcgI0aNSpxf5GXh65EW9PqtCnKf\/nll\/Tzzz9vkr5bjffTr1fm6qsVrwUy1+y6LQAWctE5ceHW7LxXbnrhwoXJJX2en8dd0C9fvjx1E3355ZfJGyUeHHSTXGRZufLHPvWN2GsINl7ABqawr6cWEQ\/hPSJ0Se9S3eQQ5lX23nvvdMABB3QV2VR6o2T\/\/ffvKrn+f4c903tfrSnUB2BUJNr8qWXJ4a5zF12Oc+f48v7yjZmLE25dmPNsu+22ydOtbiKvOHma1k0ykeX5D1akNz9bZ8ljDhuW6v6rBTLL5a5z91yOK8dXt+MeX30N\/Om71enu3y9eS0uKStv89G26aOR2RbzOn1og5w3ZLdtJ55YsLT\/nK8cdsbj68rpd5uul12kAsG60xs1cmP7+X1\/fAOAd\/\/d3CdDrONv\/bQQyIK2pHqbnliwtX3n7LnscVRoA6n9\/8v1aMBenAPaaOX9I8\/\/4fR+7NXjWOfuk7f80vy+vTqQRyN5Y0Ki7amFQpKM88nthtQYCUJYKyAD1tw8vXAvykg2A1YL1d+aE36T\/ufWk2leZ6gU1Atmay2o9WHc80ohQWr5yeT1ap4EqMD1Y4H5\/+\/DCBGBA59aqJovNgZ17zag0YfT+igZEjUDWg4fq3pZwZ21dFkrLV761URlIwLHMOmCGroAKRNb6u38eVVhsAKss+AYaNgZZR\/nRyNosLX9LpBxEVhcgssYqIPGULTP0AjAWmgP63f3\/WIAKYPn\/cHj9o1G02y4cEMjtGt1cygGIbHiA45gCxPP\/\/Q9p3Oxl6cB\/eb3Y2Vor5SN8QFSvv3FWgWk9RSx0KAGtkqkRyC47XHrUOTJVdbYx84CAAkDgAIkrRWGFLDJAvPv3S4oH8UD84od1N0tVMgMRsTwENG4WhWWWwcRf1dbGyGsEso2V138I5qrSmox8yErexqRWAAIuKAAEMKABiFrJeuDu2yaUu9YAMkBklfIQoLlZ1KrdTVXWCGRCusxwqWEtDvLCOAtn6Xg6ScAEDpBYIPDCCvsDUJ1WMrCqABBAgGKFCIgLbh6d5k48OD3zT79JyvCgbgWx1ViVNQbZhQd3zYKDvP3vmTFL1+hgCUiADVABLM0ClbVqvwpAQAGQ9QFRGO5UWQC4uYLYSh\/KGoHMUn11hPe5PFEKS\/b2f6feNwYiUBFQCRkEQAQUBCDUDkC8AFQ32tqawkYgs1QPIpB4Y0W1qQBgLjjABQqAciBZoTRShrZmANuotChuBHJRY4j+AJj1CnUBYBYaYAJSfo+aa6BrQGa9yBAAzGKF0j0anAa6AmTW6yLCUADLgsV71BkNNALZxsvOWph3L12Vn\/O0igMZ4XG0AbR4jzqjgVogA9E52Os7dtXCOD4JpYlT9bam\/HYUVoxvwugDBD3qoAZqgWwnbUcdAAvj+BShCxIXJU1lW7pidd\/zU1bc22A11WB7\/logs+Rwx87ILJcF58TS8bXvckOOcNNyewDTQuepFsgsmaX61h8WHdabh\/LxNRXRA4SoM+awPSO6eYVdLm0tkIdyDPmxqWfJQ6PpRiBzx9xy7qYjLl95WUxPqILHtxTk5X\/eee+0Zu+\/K7LqfgyzYO79aaSBRiBzx9xy7qbFx48fn6o+uup7pr744ovi6xl9OwGX752wKgl7Vlyllc7kNQK5vy69remD22VLLr9rvcceexRf4hbt\/Hnnv4lo6q3HfaroeKQjIPv6Rbvu\/qTjsr3w5wkWbxB84aqlV6\/+uas+Qditn2r0jYj01YQagcxSrb2xxkboxfoqdx2CeJPTRPANBfm6vGbvI4IlHbbL6q75ZGM3f6rx22+\/7dNZ3UgjkFlh1ZpsXeaaW3W66667Fj9rkH9rXfC7xuymTzZ266ca6WjYsGGhttphI5C1auPkS1OF0iyTdbNy6ZxsvLhqecp9QatXhaRRuGs7a5\/a6xbq1k81hn7orgk1AtmrP35Q5Pbbb0\/xJoh3rn2Hly8mVZ53HtbNrbslwxd5jk\/By5Ij3gs7r4FGIMdnnbypmYviCCUd5eJB1mPuHJkQkd8DOTQx9GEjkK3JXLXPPoW75obPP\/\/8Yr1VPhCRR+y100Cq9erU1EAjkLXJGn3Zi+8Byd0wi1Vel+b\/cUUfqzW5L9GLdFwDjUEmgfXYG5pcMAK8\/IHSiJpfcDLQ9rf2eo1A5prjkeNgFZc\/YuyBPFhttq7fCGRrrs8h+y7N1s22Lw2QewC319VgORqBzJKddX2XpvU4p\/7OyoMVsFd\/8BpoBDJL7u\/GS77yuiJ57Qdvt226yLSlUS2QWbC12C\/GsNjcgiMuH19dBYW7rsvf4xu4BmqBzEI9C+7U6z85wL01eeDg1a1ZC+S8MZcgLkSE8lvdXStvRz2Q22lo8OWNQHY33eTuuj\/xcktetuzzRr+\/4HXgoaYFCxYUPyw61P0MpP1Wz+3703cjkONuusnddVXHufU+\/x+P1v79BfuCjUG+m\/v6669Pl19+edfJNmXKlORH2Jp8g30jkK3NXPVg766B7MPg71\/\/t+k\/\/+2arvudB\/uPbia\/2zFkILNKV5iduLvWFkE9guzRiY1+64Le6K8uNbLkaLTTd9fRbi8cGg0AeWha7rXaNRrYJCB7LSguURzBNqU2nBhs5sjT34WO46Lf0MCD4pWmTSV3yEyPdWTY6CC7FXvwwQeTl+2R7wWTV0fYoeCxv\/DQxSPT0aNHp6qHL36LYuTIkcWHBPA1fXbeSbnpyqvNjl91293oIHs194cffihesj\/kkEOSnwWUV1fgTvKxUzfszQAABe5JREFUCK8J+0lC7XrJUFq+dJA3TE0Eb5xG3qYIyeXh0MyZM9Pw4cNri9AYZC6CyypTf66uShK\/6pZ\/YJ0Sq\/g2Vl6ArD+XDXEfIE2xgGfhxsxtc9\/KNjaZZLyIo2yTvhuBzFVMnz49+bEQbiunpk+hmgi5KXkp1pk5xjpjxow0adKkRBebUq4mfTcCWcOssHzjJb8Jla0lt6Qm7XSKN\/ckxpd7mXIfXrz\/9ddfk3W6XNat6UYgcxNuvKwLAx2QCbL77rsXSlq8eHFauXJlkjfQ9gZTj5UaT4Dsp4Kl5Ue73PWVV16ZwkX7rSuXEfYTwdPtYSOQuag5c+YUO1DrU05112QTxc\/D+gAcEpc3BIqq1aS3Tn281liE0io62iGAc8\/y8cyaNStNnTo1yce3OVAjkIFh7Y31KQ\/lK68zaJ+iiLrideoMFQ+wYs0VSuvL9S0Sz2\/4vKUqLX9TET3Td13dNQLZoLivuDwQcrnjxo3rc2d4etRdGmgEMoB9TNWZ0UWGM67fNHQ495xZeXcNrycNDTQCOc6PvllA5aBWn4WyrtW5BsSDN9ocTGiy8TLWUO22ayv43QG04x2q8pCBzJ3SQ8jaCGRrASv2CcZVq1YVbfgJ2sF+FqpoqIN\/YjK6+nN50MGmh7wpdxCxF+hUZ41A1imlxRsTfjb3tNNOSz6SOnHixOQ2yEwUOnKwDMctt0VhUWYpHhR88vDgFdcPfjwo8uTnlM9+fPpzAjDpAJy\/3JDXy9tWJ8psqsikrbxPcXlIubFF32PHjk3ytYPEkT54E3zal5aPtCevFWlLfcc3dfo7vWg32tOXOurmbTcGWWW7utgdCx0vrMlug6SFs2fPTvi4du7c5NC5IxgA8AEBn5mLB684oR1n3Glb+x1b1NV3TtOmTSuS+FiAn0lwSfHMM88kDxQ8fCjvhLWtkv617WFJfEXDO++8k5yDtaVPYOq3SmZtoCOOOCJpy1k\/bgPJQ37lSJ\/S8vWpbe0qa0X0NGrUqKJ9D08YQSv+\/soGBHJ\/jUU+cIEa6Qjlz5s3L3H78qpuusxGd8VxKQEkkyEuLNRD+CiOV3Hs8XYJYIGkvIrU0bYHEcq1PXfu3OSHuqWjT4CNGDFCVjFRW8kcYwCg2zJykIdcGrCs6TPa1mfVePCWyUMIk18+T\/nxxx+7TpVsRI1BpigugQsJsuO+8847k5ksrz\/XUq7bamYq0xbiyssgW3ddj8ZoKdZ+IdJVYblOmScAy\/PbyRx1yvLlbYi3Gw+eMpk0ra5Yy\/z9pRuBbMAApUwuCpnBGr\/llluSZ8PyfBPQ5MmTE35lQdynuDr4uGfpKuIy8QSVPYPBU0LU1RfLjnRVWK5TxVPOqytzgF2uH+l24wm+PDSJTUx5liGPaMX7I7zqlMsbgawRDeTgsKBbb721eKBuTVOOTARl4lVkg2StK5epw7X58jfA4eMZrGs5Lz59BJ\/1a9GiRWnMmDE52wbxqOOOWoE1V18fffSRZFsiS5XMKnLxFEwOcpNL\/i677JL0IS1fG1XjwVumpUuXFnqVT2brcix18oIsB9pmPOpEfoSNQNaBtaZspQZw5plnpssuu6zYaXLbMRHMcO7WLtD6QhEUYgfMK7A+AuLj0oBp8wVAfNY4g5MXQkcYGy982rLhs+ZFeVWojj4tA+7OfcnNkUceWcVa5PUns7W2YFj\/h254MHKYaMcdd9z6kpTIXmc8fRXWR+wxbAzJSmayK6JLehKn59CpDSd9yTeZ4iayEcgqAtBspVidIyA+\/\/zz2i7eWNCZQcuw2eJyuVt5NjHSwosuuqh455qFUYR8oXr4pZG4vDKp574ZD9IXHv3YUFUBXlUn8qrqa4us2heGzB456jvq6Fccn6PY8uXLi+9R0bYyY1CGxOW1IzeKPIc6+srbCj3l8j366KOFPskhP3TQCGQVDVSn\/ZFyfO0GsKWVc\/3O0CY9yi2vyVh5grDSJvVa8TYCWUNca3l3LS1f+dZKvAYLjsmfW14dnbBSddRnpaxRWn6d+q14\/gIAAP\/\/wgmxvAAAAAZJREFUAwC4fKE10uDVpAAAAABJRU5ErkJggg==","height":100,"width":121}}
%---
%[output:98c8ccfa]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAHkAAABkCAYAAACisp8MAAAQAElEQVR4AezdeexV1XYH8B3jPKJ1VkgcmvrEoUSJGhpNHaKJ8DREE1EjahOHipGoGKkTCTXWAaOJiBprgjXyh3V6xMQXTRpMiRNKHfDV1Cf4QHGKGBHFlDwfnwPrl81553fvOb\/f\/cEFLmH99rT23muv715rD+fce7f5tfdvi9fANqn3b4vXQA\/kLR7ilHog90DeCjSwFQyxsSX\/+OOP6cILL0yHHnpoES5evDiNGzcuffDBB1uBujbPIbYF+dVXXy0AFQL4iiuuSAceeGB68cUX02677Zb22WefdPbZZ6e77rorKa+jBnwmChJX54477ujoRLnxxhvTKaeckr7++mvNJ\/I\/8sgjRXwo\/pjkJ598ctsxGC\/ZQq5WsnRK5rYgn3766enTTz9Nwp9++qmQ6aabbirC+DN+\/PgiGuVFosWfXXfdNT399NMFiVP+Cy+80KLG4IoAcMMNNwyukTa1Z8+enQ4++OB0yCGHtOScNm1aevvtt1vyKOykzH0gm2EsixtGZpvOzCZp4VdffZUWLFiQzjrrrHTBBRcUwn722Wfp1FNPTW+88UY68cQTCxeuLUIee+yxCYDldpTrC7300kvpnnvuSStXrkznnHNOYXH4Uc4nLo9cuYVKkw9pbz0f1oK++eabNGnSpKJ9\/eBnRdpQB8krmNf+Ia88JF9o7GuLCq+Q14t8Y33llVcSSzZp8UZd9UMubT\/33HNp6dKl6YwzziisXh4eRF\/aIl9ZZuPSDj6knn6qqNx3H8jT1s4wFd5\/\/\/302GOPJcLEIOQH7bTTTmnfffdNq1atSt9\/\/30aO3Zs2m677QqQufBFixalp556KtjbhpTGM3D96vMYUYnCKO7zzz9PvITBs4LRo0cXMhiMtAmG8MU4og3LyUMPPVQsLfq57777ikk1YcKEwkPJi7FS8KxZsxLPxHtFG0JKnjx5cjrooIMSHanHO6hj8uMZM2aMoJio2qRHYwqdXHXVVUXbw4cPTyYFZpOc7GjYsGHFsrfzzjunsszGZXz4tGvCVuFTpZNtdBTKo1CKpWiDFCov07nnnpuWLFlSDPakk05Kp512WqH0o48+Or333nvJYMp1BpqmOJOJYhEr4ElC5gDcxAMc5VF8q\/4ADUgTjLKCd\/78+YXFa19ehOI2mMAKHZFLPoA\/+eQT0b8i+xfl\/emEvubOnVsYFC9obH\/VyNqM8liPOeaYZKKU+y3zhU4KkAlCkWvba\/n\/u+++K5Rw9913F5sxnb3++uvp2WefLdLhblo20rDQGjdy5Mj08ssvF2Rw+g2ZDz\/88EYtskhuj1Jvu+22wmvlDfAo++23X55VxPVnSTEpuEtLizRFo4Jp\/R9tI0lA46+yOnnKXnvttcITRh31ctI3fHgH\/PhMiHK\/wVfWSQGyQQ1b6yrKlfKOxPfaa6+0\/fbbp6OOOqpwdSyLJY8YMaI4Rs2YMaNwNxSJvxPEs7AeFvrWW2+lsNy6MpdlCIvkbqs8FeAoq1xPfyaAerxc0FVrXXBZqWS2scSDX1vTp08v1nTxIBPXpH3ggQcSFx355VDf8OF9tBnEI+W8wVfGsQCZWVOe9Y3Jc3essry4s+Q1a9YURxONGwwAVqxYkSjPBHj33XfTzTffnModGpA6A6FwjcuWLUvhQssyk3vOnDnFhoYb7K+fXC6T8Yknnuhj1Q8gyVouC4\/C6pTRDR3RFZDzyZGXXXzxxcWG1FpeBlI9FmpSWWtRnzBZpDzW8ADCjK1YMnMcQycFyBinTZtWbCpOXLtD5ors\/sxSZUE77rhjEX388ceLnaEExaxevTp9+OGH6dJLL0077LBD4gYJZo0MF4O3P9KGMv1SkHhOoWCznquOMjPZoMiMxOVFeYRRn6udPHlyOu+884p1UFsmDmDNfpPj6quv7itzHxBtmNAszuZHPW3xXOqYONEGfnqjP+PBay2fOnVq0oZJytWSl45YKD6bOEcw7dtk5jJbXqZMmdKHjyWAh6jyRMZPD9pH4n0gEyBcDHeAmcAakhaecMIJiTLs\/AhmfRA+\/PDDxZpi7bSzNmh1DVZdpD2hdqIv\/YlTlM2JcnXUzQkP3nnz5hWzNS+LdtUVjzLx4I\/6eLRz7bXXFsuNNMvUt35Zpd01BSpjacCL8QBFm8qQseiP\/EClG2mkfzxI+3jkqyMPcb\/RHh6ySOunLDMZyK4eIq\/2qijvW3ybKqZ2ebmgOpQmmJ1iDKZdG91YTnZgsVITWMiy5beTd+LEicmewURpxzuI8gFVHRDIA+ppM6lk5pu4Qa0sJh+SicAShXl+N8Qbg2zNNMvL5Mxpoe+GQfVk2FADjUAGohsatzjWEzcvZry4TRaXvWHzrVM2PXaUPXqj2NPU1QO9tdbshqWNQFY1NiI2JY4a8mxUuCqTQLoOEdSO0c6xRxcWd\/519HDezTPTMff\/X9rr+v9Kc97+so6qm70ZEuc8xyLHnthoSNv61+pxPROQ33zzzXTvvfcWT6PsHLuFnJ3vv\/\/+9OSTT3adbNddd916DaY0fM91R9q+jH4ijSzZtv6BtbczrNaW3sxzhLILjbNxP\/0UT6Pwu0jIeVwSOM91G7noOP7444uLjG6SzVk611+deCOQNWjdZXFCO09rMnKMUl5FjhXOn1Vlvbyh10AjkK25rFGYiyZdlY+H5bo8ueSSSyR7tAk0UAtkIDoicVt2gML8CCVN9lizxYPcgI0aNSpxf5GXh65EW9PqtCnKf\/nll\/Tzzz9vkr5bjffTr1fm6qsVrwUy1+y6LQAWctE5ceHW7LxXbnrhwoXJJX2en8dd0C9fvjx1E3355ZfJGyUeHHSTXGRZufLHPvWN2GsINl7ABqawr6cWEQ\/hPSJ0Se9S3eQQ5lX23nvvdMABB3QV2VR6o2T\/\/ffvKrn+f4c903tfrSnUB2BUJNr8qWXJ4a5zF12Oc+f48v7yjZmLE25dmPNsu+22ydOtbiKvOHma1k0ykeX5D1akNz9bZ8ljDhuW6v6rBTLL5a5z91yOK8dXt+MeX30N\/Om71enu3y9eS0uKStv89G26aOR2RbzOn1og5w3ZLdtJ55YsLT\/nK8cdsbj68rpd5uul12kAsG60xs1cmP7+X1\/fAOAd\/\/d3CdDrONv\/bQQyIK2pHqbnliwtX3n7LnscVRoA6n9\/8v1aMBenAPaaOX9I8\/\/4fR+7NXjWOfuk7f80vy+vTqQRyN5Y0Ki7amFQpKM88nthtQYCUJYKyAD1tw8vXAvykg2A1YL1d+aE36T\/ufWk2leZ6gU1Atmay2o9WHc80ohQWr5yeT1ap4EqMD1Y4H5\/+\/DCBGBA59aqJovNgZ17zag0YfT+igZEjUDWg4fq3pZwZ21dFkrLV761URlIwLHMOmCGroAKRNb6u38eVVhsAKss+AYaNgZZR\/nRyNosLX9LpBxEVhcgssYqIPGULTP0AjAWmgP63f3\/WIAKYPn\/cHj9o1G02y4cEMjtGt1cygGIbHiA45gCxPP\/\/Q9p3Oxl6cB\/eb3Y2Vor5SN8QFSvv3FWgWk9RSx0KAGtkqkRyC47XHrUOTJVdbYx84CAAkDgAIkrRWGFLDJAvPv3S4oH8UD84od1N0tVMgMRsTwENG4WhWWWwcRf1dbGyGsEso2V138I5qrSmox8yErexqRWAAIuKAAEMKABiFrJeuDu2yaUu9YAMkBklfIQoLlZ1KrdTVXWCGRCusxwqWEtDvLCOAtn6Xg6ScAEDpBYIPDCCvsDUJ1WMrCqABBAgGKFCIgLbh6d5k48OD3zT79JyvCgbgWx1ViVNQbZhQd3zYKDvP3vmTFL1+hgCUiADVABLM0ClbVqvwpAQAGQ9QFRGO5UWQC4uYLYSh\/KGoHMUn11hPe5PFEKS\/b2f6feNwYiUBFQCRkEQAQUBCDUDkC8AFQ32tqawkYgs1QPIpB4Y0W1qQBgLjjABQqAciBZoTRShrZmANuotChuBHJRY4j+AJj1CnUBYBYaYAJSfo+aa6BrQGa9yBAAzGKF0j0anAa6AmTW6yLCUADLgsV71BkNNALZxsvOWph3L12Vn\/O0igMZ4XG0AbR4jzqjgVogA9E52Os7dtXCOD4JpYlT9bam\/HYUVoxvwugDBD3qoAZqgWwnbUcdAAvj+BShCxIXJU1lW7pidd\/zU1bc22A11WB7\/logs+Rwx87ILJcF58TS8bXvckOOcNNyewDTQuepFsgsmaX61h8WHdabh\/LxNRXRA4SoM+awPSO6eYVdLm0tkIdyDPmxqWfJQ6PpRiBzx9xy7qYjLl95WUxPqILHtxTk5X\/eee+0Zu+\/K7LqfgyzYO79aaSBRiBzx9xy7qbFx48fn6o+uup7pr744ovi6xl9OwGX752wKgl7Vlyllc7kNQK5vy69remD22VLLr9rvcceexRf4hbt\/Hnnv4lo6q3HfaroeKQjIPv6Rbvu\/qTjsr3w5wkWbxB84aqlV6\/+uas+Qditn2r0jYj01YQagcxSrb2xxkboxfoqdx2CeJPTRPANBfm6vGbvI4IlHbbL6q75ZGM3f6rx22+\/7dNZ3UgjkFlh1ZpsXeaaW3W66667Fj9rkH9rXfC7xuymTzZ266ca6WjYsGGhttphI5C1auPkS1OF0iyTdbNy6ZxsvLhqecp9QatXhaRRuGs7a5\/a6xbq1k81hn7orgk1AtmrP35Q5Pbbb0\/xJoh3rn2Hly8mVZ53HtbNrbslwxd5jk\/By5Ij3gs7r4FGIMdnnbypmYviCCUd5eJB1mPuHJkQkd8DOTQx9GEjkK3JXLXPPoW75obPP\/\/8Yr1VPhCRR+y100Cq9erU1EAjkLXJGn3Zi+8Byd0wi1Vel+b\/cUUfqzW5L9GLdFwDjUEmgfXYG5pcMAK8\/IHSiJpfcDLQ9rf2eo1A5prjkeNgFZc\/YuyBPFhttq7fCGRrrs8h+y7N1s22Lw2QewC319VgORqBzJKddX2XpvU4p\/7OyoMVsFd\/8BpoBDJL7u\/GS77yuiJ57Qdvt226yLSlUS2QWbC12C\/GsNjcgiMuH19dBYW7rsvf4xu4BmqBzEI9C+7U6z85wL01eeDg1a1ZC+S8MZcgLkSE8lvdXStvRz2Q22lo8OWNQHY33eTuuj\/xcktetuzzRr+\/4HXgoaYFCxYUPyw61P0MpP1Wz+3703cjkONuusnddVXHufU+\/x+P1v79BfuCjUG+m\/v6669Pl19+edfJNmXKlORH2Jp8g30jkK3NXPVg766B7MPg71\/\/t+k\/\/+2arvudB\/uPbia\/2zFkILNKV5iduLvWFkE9guzRiY1+64Le6K8uNbLkaLTTd9fRbi8cGg0AeWha7rXaNRrYJCB7LSguURzBNqU2nBhs5sjT34WO46Lf0MCD4pWmTSV3yEyPdWTY6CC7FXvwwQeTl+2R7wWTV0fYoeCxv\/DQxSPT0aNHp6qHL36LYuTIkcWHBPA1fXbeSbnpyqvNjl91293oIHs194cffihesj\/kkEOSnwWUV1fgTvKxUzfszQAABe5JREFUCK8J+0lC7XrJUFq+dJA3TE0Eb5xG3qYIyeXh0MyZM9Pw4cNri9AYZC6CyypTf66uShK\/6pZ\/YJ0Sq\/g2Vl6ArD+XDXEfIE2xgGfhxsxtc9\/KNjaZZLyIo2yTvhuBzFVMnz49+bEQbiunpk+hmgi5KXkp1pk5xjpjxow0adKkRBebUq4mfTcCWcOssHzjJb8Jla0lt6Qm7XSKN\/ckxpd7mXIfXrz\/9ddfk3W6XNat6UYgcxNuvKwLAx2QCbL77rsXSlq8eHFauXJlkjfQ9gZTj5UaT4Dsp4Kl5Ue73PWVV16ZwkX7rSuXEfYTwdPtYSOQuag5c+YUO1DrU05112QTxc\/D+gAcEpc3BIqq1aS3Tn281liE0io62iGAc8\/y8cyaNStNnTo1yce3OVAjkIFh7Y31KQ\/lK68zaJ+iiLrideoMFQ+wYs0VSuvL9S0Sz2\/4vKUqLX9TET3Td13dNQLZoLivuDwQcrnjxo3rc2d4etRdGmgEMoB9TNWZ0UWGM67fNHQ495xZeXcNrycNDTQCOc6PvllA5aBWn4WyrtW5BsSDN9ocTGiy8TLWUO22ayv43QG04x2q8pCBzJ3SQ8jaCGRrASv2CcZVq1YVbfgJ2sF+FqpoqIN\/YjK6+nN50MGmh7wpdxCxF+hUZ41A1imlxRsTfjb3tNNOSz6SOnHixOQ2yEwUOnKwDMctt0VhUWYpHhR88vDgFdcPfjwo8uTnlM9+fPpzAjDpAJy\/3JDXy9tWJ8psqsikrbxPcXlIubFF32PHjk3ytYPEkT54E3zal5aPtCevFWlLfcc3dfo7vWg32tOXOurmbTcGWWW7utgdCx0vrMlug6SFs2fPTvi4du7c5NC5IxgA8AEBn5mLB684oR1n3Glb+x1b1NV3TtOmTSuS+FiAn0lwSfHMM88kDxQ8fCjvhLWtkv617WFJfEXDO++8k5yDtaVPYOq3SmZtoCOOOCJpy1k\/bgPJQ37lSJ\/S8vWpbe0qa0X0NGrUqKJ9D08YQSv+\/soGBHJ\/jUU+cIEa6Qjlz5s3L3H78qpuusxGd8VxKQEkkyEuLNRD+CiOV3Hs8XYJYIGkvIrU0bYHEcq1PXfu3OSHuqWjT4CNGDFCVjFRW8kcYwCg2zJykIdcGrCs6TPa1mfVePCWyUMIk18+T\/nxxx+7TpVsRI1BpigugQsJsuO+8847k5ksrz\/XUq7bamYq0xbiyssgW3ddj8ZoKdZ+IdJVYblOmScAy\/PbyRx1yvLlbYi3Gw+eMpk0ra5Yy\/z9pRuBbMAApUwuCpnBGr\/llluSZ8PyfBPQ5MmTE35lQdynuDr4uGfpKuIy8QSVPYPBU0LU1RfLjnRVWK5TxVPOqytzgF2uH+l24wm+PDSJTUx5liGPaMX7I7zqlMsbgawRDeTgsKBbb721eKBuTVOOTARl4lVkg2StK5epw7X58jfA4eMZrGs5Lz59BJ\/1a9GiRWnMmDE52wbxqOOOWoE1V18fffSRZFsiS5XMKnLxFEwOcpNL\/i677JL0IS1fG1XjwVumpUuXFnqVT2brcix18oIsB9pmPOpEfoSNQNaBtaZspQZw5plnpssuu6zYaXLbMRHMcO7WLtD6QhEUYgfMK7A+AuLj0oBp8wVAfNY4g5MXQkcYGy982rLhs+ZFeVWojj4tA+7OfcnNkUceWcVa5PUns7W2YFj\/h254MHKYaMcdd9z6kpTIXmc8fRXWR+wxbAzJSmayK6JLehKn59CpDSd9yTeZ4iayEcgqAtBspVidIyA+\/\/zz2i7eWNCZQcuw2eJyuVt5NjHSwosuuqh455qFUYR8oXr4pZG4vDKp574ZD9IXHv3YUFUBXlUn8qrqa4us2heGzB456jvq6Fccn6PY8uXLi+9R0bYyY1CGxOW1IzeKPIc6+srbCj3l8j366KOFPskhP3TQCGQVDVSn\/ZFyfO0GsKWVc\/3O0CY9yi2vyVh5grDSJvVa8TYCWUNca3l3LS1f+dZKvAYLjsmfW14dnbBSddRnpaxRWn6d+q14\/gIAAP\/\/wgmxvAAAAAZJREFUAwC4fKE10uDVpAAAAABJRU5ErkJggg==","height":100,"width":121}}
%---
