%[text] # afe\_inv\_psm: model initialization (param structure by subsystem)
% Rev. 01, 2026-09-29. Replaces the flat-workspace init_model.m (kept as init_model_legacy.m).
%
% Everything the model needs is in the struct "param", one group per subsystem:
%   param.sim       simulation settings (stop time, fixed step, solver, module count, thermal model choice)
%   param.time      timing settings shared by all stages, and the PWM data of the stages not in this model
%   param.glb_time  timing_setup object (built from param.time, param.afe.pwm, param.inv.pwm)
%   param.grid      grid emulator inputs;              param.grid.emu     grid_three_phase_emulator object
%   param.frt       FRT test inputs;                   param.frt.data     frt_settings + grid_fault_generator data
%   param.afe       active front end: .pwm .hw_in (hardware numbers) .hw (hw_afe_three_phase_setup object)
%                   .ctrl .pll .impl .ref .behav .igbt .mosfet .ideal_switch
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
[model, options] = init_environment('afe_inv_psm');
s = tf('s');
param = struct();

% functions overridden for this model: ./private takes precedence over the library
overrides = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
if ~isempty(overrides)
    fprintf('%s init: local overrides in private/: %s\n', model, strjoin({overrides.name}, ', '));
end
clear overrides
%[text] ## Simulation
param.sim.simlength = 1.75;                 % [s] model StopTime
param.sim.tc = 0.25e-6;                     % [s] fixed step (model FixedStep) and logging base
param.sim.load_step_time = 1.25;            % [s]
param.sim.transmission_delay = 125e-6*2;    % [s]
param.sim.sst_num_of_modules = 2;
param.sim.time_aux_power_supply_fault = 1e3;    % [s] never, in this run

% one/two modules
param.sim.number_of_modules = 1;
param.sim.enable_two_modules = param.sim.number_of_modules;

% power stage model: MOSFET with thermal model, IGBT with thermal model, or ideal switch
param.sim.use_mosfet_thermal_model = 0;
param.sim.use_thermal_model = 1;

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

% magnetizing inductance from the core geometry, no-load current from it
param.grid.Lm1 = (param.grid.n1^2 * param.grid.mu0 * param.grid.mur * param.grid.core_area) / param.grid.core_length;
param.grid.i1m = param.grid.us1/sqrt(3)/param.grid.Lm1/(2*pi*param.grid.fgrid);

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

g = param.grid;
param.grid.emu = grid_three_phase_emulator('Dyn11', g.delta_star, g.grid_nominal_power, g.application_voltage, ...
    g.us1, g.us2, g.fgrid, g.eq_grid_inductance, g.eq_grid_resistance, g.eta, g.ucc, g.i1m, g.p_iron, ...
    g.n1, g.n2, g.core_area, g.core_length, g.mur, ...
    g.up_xi_pu_ref, g.up_eta_pu_ref, g.un_xi_pu_ref, g.un_eta_pu_ref);
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
param.afe.hw.displayInfo(); %[output:904db47a]
clear h
%[text] ### Control (with sequences)
param.afe.ctrl = ctrl_afe_setup(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom);
param.afe.ctrl.kp_udc_ctrl = 2;

% resonant PI gains for weak grids
% param.afe.ctrl.res_pi.kp_rpi = 0.5;
% param.afe.ctrl.res_pi.ki_rpi = 18;

% resonant PI gains for LVRT
param.afe.ctrl.res_pi.kp_rpi = 0.6;
param.afe.ctrl.res_pi.ki_rpi = 35;
%[text] ### dqPLL selection
%[text] Flags marked (variant) drive Variant Subsystem conditions in the model.
param.afe.pll.use_dq_pll_fht_pll = 1;
param.afe.pll.use_dq_pll_fht_simulink_pll = 0;
param.afe.pll.use_dq_pll_mod1 = 0;
param.afe.pll.use_dq_pll_ccaller_mod1 = 0;
param.afe.pll.use_dq_pll_ccaller_mod2 = 0;

param.afe.pll.use_dq_pll_mode1 = param.afe.pll.use_dq_pll_mod1;                 % (variant) simulink dqPLL
param.afe.pll.use_dq_pll_mode2 = param.afe.pll.use_dq_pll_ccaller_mod1;         % (variant) ccaller dqPLL
param.afe.pll.use_dq_pll_mode3 = param.afe.pll.use_dq_pll_fht_simulink_pll;     % (variant) fht simulink dqPLL
param.afe.pll.use_dq_pll_mode4 = param.afe.pll.use_dq_pll_fht_pll;              % (variant) fht ccaller dqPLL

param.afe.pll.use_dq_pll_mode1_modn = 0;    % simulink dqPLL
param.afe.pll.use_dq_pll_mode2_modn = 0;    % ccaller dqPLL
param.afe.pll.use_dq_pll_mode3_modn = 1;    % fht simulink dqPLL
param.afe.pll.use_dq_pll_mode4_modn = 0;    % fht ccaller dqPLL
%[text] ### C-caller versus Simulink implementation
param.afe.impl.use_moving_average_from_ccaller_mod1 = 1;                        % (variant)
param.afe.impl.use_moving_average_from_ccaller_mod2 = 0;
param.afe.impl.use_moving_average_from_ccaller_mod3 = 0;
param.afe.impl.use_moving_average_from_ccaller_mod4 = 0;
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
param.inv.hw.displayInfo(); %[output:3ee4328d]
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
param.psm.machine.displayInfo(); %[output:16a9bfea]
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
param.psm.ctrl.ekf = ekf_pmsm_setup(param.psm.machine.Rs_norm, param.psm.machine.Ls_norm, 1e6, param.glb_time.ts_inv); %[output:3269560b]
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
figure; bode(param.dclink.sys_dclink, param.dclink.Z_Lstray_dclink, options); grid on %[output:3e92312c]
%[text] ## Template extras (not used by afe\_inv\_psm.slx)
if param.sim.build_template_extras %[output:group:791a2e6f]
    param = add_template_extras(param, options); %[output:34dfda30] %[output:13e9fd7b] %[output:248d5970] %[output:3c743fad] %[output:83771a39]
end %[output:group:791a2e6f]
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
%[text] ### Enable/disable subsystems
% power stage model: MOSFET thermal, IGBT thermal or ideal switch (the other two are commented out)
if param.sim.use_mosfet_thermal_model
    active_bridge = 'three_phase_inverter_mosfet_based_with_thermal_model';
elseif param.sim.use_thermal_model
    active_bridge = 'three_phase_inverter_igbt_based_with_thermal_model';
else
    active_bridge = 'three_phase_inverter_ideal_switch_based_model';
end
bridge_models = {'three_phase_inverter_mosfet_based_with_thermal_model', ...
                 'three_phase_inverter_igbt_based_with_thermal_model', ...
                 'three_phase_inverter_ideal_switch_based_model'};
bridge_paths = {[model '/afe_abc_inv_psm_mod1/afe/'], [model '/afe_abc_inv_psm_mod1/inverter/inverter/']};
commented = {'on', 'off'};                  % index 2 (off) for the active bridge
for ip = 1:numel(bridge_paths)
    for ib = 1:numel(bridge_models)
        set_param([bridge_paths{ip} bridge_models{ib}], 'Commented', ...
            commented{1 + strcmp(bridge_models{ib}, active_bridge)});
    end
end
clear active_bridge bridge_models bridge_paths commented ip ib

% load: torque curve (energy production) or fixed speed with load torque (drive)
if param.psm.mode.use_torque_curve
    set_param([model '/fixed_speed_setting'], 'Commented', 'off');
    set_param([model '/motor_load_setting'], 'Commented', 'on');
else
    set_param([model '/fixed_speed_setting'], 'Commented', 'on');
    set_param([model '/motor_load_setting'], 'Commented', 'off');
end
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
%   data: {"layout":"onright","rightPanelPercent":33.8}
%---
%[output:904db47a]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V\nNominal Voltage: 690 V | Nominal Current: 2.324276e+02 A\nCurrent Normalization Data: 328.70 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:3ee4328d]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:16a9bfea]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:3269560b]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\n","truncated":false}}
%---
%[output:3e92312c]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPIAAACSCAYAAAB\/nJf+AAAQAElEQVR4AeydCXhVRZbHD24EBVmaYBIBAXWisomAxBYFP02L40SUT2hx3BpniCwtKCCCIiCyCii2qNDqNIpBpcdBaRVFW7Cx2XTSoohLgBBWjSPQoQ0oNpNfwYk3L29L3n3v3fde5ftO7r2n6p5bde79v1N16lTVcUfsn9WA1UDCa+A4sX9WA1YDCa8BC+SEf4W2AlYDIhbI9iuwGkgCDSQzkJPg9dgqWA2EpwEL5PD0ZHNZDXhaAxbInn49tnBWA+FpwAI5PD3ZXFYDntaABbKnX0\/AwtkEq4EqGrBArqIOe2E1kJgasEBOzPdmS201UEUDFshV1GEvrAYSUwMWyIn53pK51LZutdCABXItlBbPWz755BPp2LGjtGnTppJuvPFGOXDgQFjFIh\/533nnnbDyk8\/5LM5Hjhxp7q2pLHOT\/RcVDVggR0Wt0RXasGFDefXVV2XLli2yYcMG87A1a9aYYzT+tWjRQpCvz9u1a5cA5vr160tBQYFcccUV0XislVkDDVgg10BZiZDVaUEBm5aZc6xp\/\/79paysTNny1FNPVVp28lQmBDgBvGPGjJEvvvhCiouLRa27WmeeAcGHB3GuvLy8PKFVwXPh9+jRw5TBWW7ycv3NN98I+WfNmmXKSEsEPveQBxkBiplybAvkBHzl+\/fvl969e5uPu0OHDnLppZcaqwhAHnzwQWOtsdRYTj52Pv7169cbqzplyhQpKSkxtYb\/\/vvvG6uu+eGZxCD\/TjvtNJMK0MxJxb\/S0lKZPHmyaSVgvfmx2Lp1qyxcuLAiVcwzBgwYUPlsmDt37pTFixfLHXfcwaW5F6t\/zz33yLJlywzv73\/\/u+zevduk5ebmyqRJk8w95KHs\/FCYjInxL2qltECOmmqjJ9i3ac0HDWC\/\/vprad68ubRu3VqwnAC8qKjIgKJr167SrFkzk9a2bVtTONIAHT8GEOcKIJOhBv94Znp6urHQOTk5BrDl5eVC2SgH5eEZLVu2rJR6+umny8knn2yuaZ7zI4KlnTFjhuHpv169epnTs846S7QenBum\/Wc0YIFs1JC4\/wAIQAGUtalFnz59jLXDEkIzZ84MKYYfDDLxw8ARAoQAFavLD4ITsKQHI6wqzexnn33WWG6sbbD8Nq26BjwJZKwLv8xOgle9+JYDCLB6WCiavDt27BCatE4+Fo2mNU1h0jZu3GgUxz3Lly83fVbyA6ZQeibf1KlTJTs7W5o2bWrk6D8sMUQzneZ7vXr1TLOf8nGf8jW\/75EfAXjk52gpfA14Csj8qgNeio91cBI80sjDeSqTbx8ZXdx0003Svn17eeCBB0z\/GeuYlZVl+p80W2mSArKxY8eKWkv4gwYNqpYfeU7avn27cC\/6V7m+lhs+fV6O9HtpvmO5KRey4GNx9dnwlLRVMXDgQLn44oulc+fOQv\/++++\/1yz2GEIDHgDy0RJiLT7++GPTzLvjmPPjaMrR\/\/AANnmOclLzP2BFB+hCiSEgwIBGAKfynWDjHP7SpUsFIh\/5Va+kkQeek8hHmpM0H8\/k2eShmb1y5Urz\/ubNm1c5LKV5uB+Li\/OKlgPP5V7SeR7X5KFuI0aMMPe3atVKkIl8zaPPhue8n\/RUJs8AmQ+BFxjqZYSTJ5QMmx47DdBUx5JDWNxx48YZp1vsSpAaT\/IMkFE3wyeMFfLS+QAgzuGRRh5LiaUBtbRYWwhLmlg1SIzSegbIOENwojD4j9dz0aJFRoO8fPpxpJHHMO0\/qwGrgSoa8AyQ1bGBU4RmNs4ZvKqUliESjpqH88QgW0qrgdhowDNAjk117VOsBpJTAxbIyfleba1STAOeAjLjkDpe+corrwheTpxd8EhLsXdjq2s1ELYGPANk+sWMGeLc8kekkSfsmtmMUdaAFe8lDXgGyASE6PQ0rLAvkUYeLynPlsVqwCsa8AyQsbZYXawxQfN4qjmHuGYeLXm8ojhbDqsBL2nAM0BWpWB1CZoHvMoD1PBIU549Wg1YDfysAc8BWeen4uzSYur8VE1Tvj1aDURHA4kn1XNAJoj+0UcfFSK7tJ\/MFDx4pCWeim2JrQairwHPAZkq0xfW\/jJ9ZM7hkWbJasBqoLoGPANk+r\/EWVcvYlVOOHmq3mGvrAaSXwOeATIWl1lONKeZ9eSrenikkcc3zV5bDaS6BsIHcgw0xRQ3mtI8CtA6CR5p5OHcktWA1cDPGvAUkLVYvnNYATA8TbdHqwGrgaoa8CSQqxbRXlkNWA2E0kBKAJkF+7SZTl87lFJsutVAomkg6YGMN3zOnDlm9wX2S3r99dcFnr4olo9ds2aN2YWBlUksWV3U5hsoLlwl5RtXyN6XJ8qqpS9V+574zvSbi8bRk0AGaEySwIouWbJE8vPzzdrLtVEAaynryo3shtCgQQOzCDqyUO6oUaPM7gis6WzpRquLG8PTwV239ZPp+X1k2YirZPeEy+SfUy4xx00vPCSz7r+rmh75zvje+O6iQZ4DMutyDR8+XJgkQYw10Vx9+\/aVSNbscm5NghJ1VwYU+8HmfdI7f6wMmfRERISMtpf3k\/zxj0UkJ9JyxOv+vLtmS7jU8dYpotTkmgcE0mvKz\/rXnqNZD8qz9+fL3OuyZda\/7JYX2m83x1va1pfOub2lwe1PSuN735Qf7louw+YUiLP8Q4YMkbVr1wrfG99fNMhzQNZ1uQCxVph1vDjXNM7dogPd75EFX58pkz+qGxEh44MGV8r0j0+JSE6k5YjX\/c9vbSjh0ldHThelg43OFIjrlftPM7rr89oRWbanoZzY+HQ544wz4kOnHi9nlH4oLdc+Li3fvFdaFtxsjtlln0rHy66WOv0eljr5BVJnzPvyU5+pciD7StnXJNtvWTFObn2vgeR4Dsg6MaI2kyZwZNEch3r06FHZF8YC82PQrl072blzp+iifijl1LdHy7zcOvLhvV0jolXD28mSmzJk1fD2EcmhHKtclPVhRb3clFcTWbumXCT+iDpCfxnWTv54Q3qlzsgLf\/SVrWRZ0UHJW7BDBr\/2rfxYt7FkZmaGpEaNGsmhQ4ekcePw8jtlNj3+kNT\/4i05\/pUxcuSp\/nJk6qVy5OVRcsK2j6RBy3Mlfch\/Sdbz5ZL12FeSNWyhZOYNlczu14UsE88455xz+NSiSp4DMk1pJkgwaQIws9xPOJMmWPcaRxaOCsadWYWTJjovtaysTAjtfOSRRwy42UpFtXrc999Ki8Zp0qZZ\/QipgUtyKIebstyWF37Z0tLSJBixN1TdunWr5OE9jL6ytfzt\/osMnXDCCZIzs1DmrNxdJZ8\/uf7k+csH74SyPfLjmhel7JlB8t20q6R0xLnmXDavNcClqVxnxhZpNnuTAXGDnreFfD5y\/RGtCv3eonX0HJCpKOGaTJQAkBDn8EgLRGylwlYomo+Ny8i7Z88es8UoO1RAKJV9hUjzKvEx4Jjj6EYZkeOWvFjKatkkTZYO6WQAvaponzS5+z2Z\/tbWgCoJVrbDpcVStuIPUjr3N7KrwjlVMri1OS\/fuEJOTG9lwNpm8RFp+cRWc57e6w6zBS0yAz7QQwmeAbLTU03T2JecTeVw9Mc+v2w3yi\/+2WefbTzVeLCbNGkiRUVFVURs27ZNDh48GDHRrGNPYDdkWRk\/v49mJ4ssvv1ceXnAOfL86h2VgA6qo11FfoFb9vE7crjBacY5RVMZi4v1PTHnhojff6DyHD58uMr3Fo0LzwAZS4rlxQKzOgjOLs4hrvFikyccJdBXJt8dfjaDg+9Ln3\/+uezevTsiwvKXlpbK119\/HZEcylFcXCx0JzhyHSkhxy158ZR15ikHZemtzWXghY0MoLtMWy8Tlmw6qu\/P1sv2JY9K0ZQ+sv3+S2TXsLONxQW4B+v94qhzqqKp7HROBdOrm\/V89913fT851689A2StGZaZZX0Ar\/IANTzSlEefmJlQWG7Gf9UzOHLkSAFU69atqxx7xgIHcnZJxd\/ll18eltMCx0UgYofB9PR0ycjIiFgWuxDSx+cY6Hk14SPHLXlekHXfL+vJ6gHpcu2hdyVzWYVz6p42xjlV9\/15cuq3m6RR63aSOeG96s6pMBxmqlc365mSzq5wvdb0idmCE4tdUFAgOMkAcc+ePeXLL7+U\/fv3V0BUBMdWMGcXmXCopIVwzIRKr4mjJZQsm17VSebPMYVzauD+BdK31SEpu2K05DebLddkFcii3n82fdx6bXvW2jnltv7xy\/CdRZM8Z5EBpHqtsbYQzUJ4pAVSBvHUeLnvvPNOEx4HeO+77z7ZvHmzATmOLgilet3ZFaiOycrHEQXheCqrcEjtfXmiaRbjlNrSt474c0xhcXFMZVVY3o7502T53Lukf9cMmf5WsZz\/0OqK49ZkVZffenkOyJSSvrD2l7G4nMMjLRAxT5lY6tzcXKFP0rZtW5k8ebJgKYM5u4jSOf7pm40nkw+ntsQQBuOP3zyUG7EsyrBldI6U0Ner8LByHSm5KS8SWYBSCZBCek2oI17lve88LWUlm0SdUkRM+Tqm6pyZU805ld+tsfx1YKbknVu\/Asg\/AzqQEyoYf9++fYLvZO\/evdWeE+w+f2m1cnYF+tAD8D0HZPrBeKixxE6CR1qAegh95Mcff1yGDh0qp5xySqBsfvko+mB5uURC\/zhwQPBaH\/7xx4jkaBkoqFuykOmmvGqyKpxJOJTCocNndJbD5\/c2ZKKjNELqWJQUY7c4pOrcsahKxFQwx5SmATpGKQZcUF8++m0r6XVWmgF0F6dTLEynpsraVwFolV\/bI4YFnUWTPAdkLC8WGEushONr3LhxQpoqw9fZ9dlnn8nq1auld+\/ekpOTIxs3bjTx2iUlJWa4KZCza8SXmfLTfzwvWRNXRERN73tb+PiajFkWkZxIyxGX+4ctNNFORDyFRbdMk6wKMtFRGiFFlNR5XSN2FKqziuOEa881kWU3X9Rc5q\/bZyLFnttw0NVn8JxQlJLOLgWq84jXmiB0p0X2dXZdeOGF8qtf\/UoAPdFdLVq0kG7duhlQ018OFNnFc7zm7KJM\/PpzdMPxghy35HlVFnoKVDYixb6bfZkA6P\/5ZF9YkWKBZPGcmhJ+GeRFkzxnkf1VlkCOnTt3+kuq5AFynFiAHuapp54qw4cPF3g4yXB0QSgVHnmUvBgQQjPdzeASN+V5VRb902BlG9YjU9aM7CTXtT86Dq1OMe7zR8Fk+csfiEfXTb+1aB09B2QASX\/Y2T8m3pqAEGfTOpBCAC9Na5rUmieYs4s8ODWwWJEQY9duBYS42T+jTm7K86qsmtTzlg5pJrCEPjSRYv760G7W0\/aRt2wR7SeHitIioop+8YABA8w9gwYNMnOY+VUFrMHIawEhofpcNj3TlX4ufWhmW9Hk1tlW0ehDp2QfGYtMpBZHBR\/nvjxfZxcrfxBZhUXGmj\/zzDOCVQbIoSK7bB+5agBGsD4g7wTrxzFYvnDSkOGWLJ5XW3n0oZltBaC1Dz3r3e0m9BOZqplCCAAAEABJREFUyI6E6M4hJ5oU16Y1MdGAzkk4qHBW0TxWPucoQaO+OPd1djVt2lRY0qdLly7GIhMq+cMPP5iJ3qGcXcizZDWggCawZHHht9L5d8UCoBNBM3EBMlFYgBQFadNZjyyLAv\/hhx82gFS+hmGS5o9waOG5JmwT2Vu3bpV+\/foJfR3ScHRB\/Dp60dnldJRQP5rPHJ382p4jxy15XpWFbtwqmzrFhvXIkEXr94Q32yrI7LmkdHbRTAZsANTZ74VP87lXr17C0BFx0wBSCQcYeXhZgYiAEPq7pBPSiQzOE83ZRXPT0m7TtI2nHtQpprOtssau\/nm2VZiBJZQ\/KZ1deJ6xjADMSfCxuizHQ0AIFhWwK8Ejj\/Me5zkgB+z8EHAP85EVyM58\/s6x2AxBRUIsrPbGG2+YRdYikcO9xIevW7fOTP7gOlJyU55XZaGjaJWtV8Z+ebznTzK64z\/k6RVFpsmNU4xnhkN8X\/6+Ozd5cWlaUwEFnlpc5xEvHytaOHmAlHu4F\/J1dhUXFwtjx0RwkQ6gmWxBv5kxaOfCfbpmV\/PmzU3QyNy5cwVvdyTEENlLL70kEydOdEXWmDFjBM97JGXSeymbW\/K8Kou6Rrts8ybeKazxVn\/VDPndm58KiwTy3FDE94Xvh++NbzMaFDcgY10ZGyaAAwsKEZU1ePBgwQOdnZ1dpY\/sa5F9nV3MHwW0BI+gKDzVLIN7wQUXGIAzPIWVx\/GlYEex9MVpCVgqkGjpINnkvvzYePnTbS3kyd7pYeuM74zvjW8zGhQ3IGNdWSwA8GrFADUea+YQ17Q5wg8DfWSa7VhyFu+jnwzAhw0bZmKwicPmnLz6TJSLV9xSjglntXoIXw+\/7tEubJ3xnek3F41j3ICsQ0kzZsyorBfziU866STT3GX8tzIhzBOnlXZacKY4YvEhzsMUZ7NZDSSMBuIGZCwuFpN+LBYUworefffd8txzz8mnn34q8JR8+8gJo2FbUKuBGGggbkD2rdvs2bMFJxfzSYl9pj+LBVVyWljfe+211UC8NOCV58YNyCwEMHz4cDNnmL4xFpo9nvD6du\/evYo1VqvMYnt4q72iPFsOqwGvaCBuQNbhIECsysCbTB\/52muvFfhqjXGIQcwpZjM3zW+PVgNWA0c1EDcgq7MLB9fRoojg+CI+mmAIgKt8QI2HOyMjQ1l+jxr6iQUnSgyrT0Ynn\/hueJasBpJJA3EDMk1pnF04uAAzg\/k4vhhvYzYSoFZFk845a3ExNMW5LzGcNWnSJJk\/f74Zf2aXiQkTJpi9nubMmSMszAexPxR5fe+311YDiayBaAA5bH0wnosTS5vQnBPRBcABNZYVAuysiMnythqV5fsQAj6ckV3kY3IEcggUYQF5ZBNsokEjvjLstdVAomog5kDGGjKUBED9EWkoE1D7ApwIIedEC\/IpAVQAqyAlsovQTMajifDSpjz5SeMIESNNEIqlNWY98GTRQ3HhKoHKN64QiPWynbT35Ynij1iONxq0a8JlfG5Ro5gD2WmF6QfT\/1XAck3YJs1qX5ADcH4EAmkCuay0SROdewsLC01oZqD88AHxqFGjhP60pRsTSg933dZP\/vDbawwVjs0V1sSG\/jn1Emk26yL555RLDMGDfMG56YWHxB999fZCgVYtfVHcpLVr1gjfG99dNCjmQNZKAEocWIBXeYAaxxRL9tCfJY0+L0cADlg1r78jUVv6o8BQFs1oxqV37twp6iXnPprdHFHs2rVrhX451j4Swpt+6623mmCWSOTYewuqxS8\/N2eqzB94pczpXlfYUOCdzlvlhfbb5Z5WpTK26yly9TlNpXveDYbO\/ff7zZYxbEyeOeE9swfUcWP\/ItA3I1aL0nFjKnhBSO9141hy1TRh2WW+N767aFDcgKxNXXVkUTksMUcASFMZwDEdEYADesBPuj8iLS8vz2zchrea5XNxeHXt2tVYZvrQBJk4J02oHJreYcYYB4ytJY77lltuEVYoiVSWvT9HupyZIdklb0nLZWMk67kbpM1HT0nH+gcNWBWkbRYfEbaNgeBBjfuNlwY9bzPE\/k9Qq07dBYqXXpn5pN9atI5xA7LTa01TGMIxhXVs2bKlqS9gXr58uQEPVtUwA\/zDWjMhAkAxHo13m\/60k08aeeAFEOMJNqtd8KPD0Y0CIcctedGSxd5P9GFpAus2MlyfmN5KAKiClnOACkB9dROtsvk+x4vXcQMyygBQ\/pxa119\/vWBBmQRBEAgWmplN5Oe+QORsWs+cOdNvNqejy28Gy4ydBr7bIfLhfwv7ZbH\/EyAur3BONe47wTSJ1dIC3NgVKjGfFHMg0wQGnMHURRPo7bffNlkAJ0sDAWrDqOE\/npdo48hpaWnCUBnHGlbXb3bkuCWvNrKwthAWFk8xHlysLpuRH\/\/eXKmXeVYVq0vz2J\/F9Vs5B7M2ZXPcXuXUTVlVBEfpIuZAxqoSM01T2l+UFTzSyONGnRmOYliKZjofM9Ydnhuyk0xGjaoDMJWwohBAhQAr1hXAYmmV4JGuzWUcSdbq1kjtATPHHMiUBCuLd5lzQOuk6dOnwxYdRtK0UMNP5qYA\/3BmqXONLM7mNV7Q45++WfjoIqHvpl0lR57qb5qJkcjRe7eMzpGS+y+JuFw1laeg83fEiio50xnegQBqacE42fvO01JWskkONzhN5Mxu0uD2J6XxvW9K1vPl0mz2JnN9Ys4Ncuj084WZbqx0Sv82UmLnRLfkuSkrKVfRBEhKOKMANEQgQosWLaROnTommWEn+Er0pbHmJtHlfyiarUcjoXhuq7r1x4YSinafkCHbj\/wiZL7tjTvK2uPbypK6l1ehJw71kAlNRleh\/GazRSkvs0C6tPizXJNVYCi\/2SPyxOmjDa2s30P2NckWVpR0EgBmeBDQOPm1PXdTnpuyknIVzUAYBKSAFeACauKm3bDGPG9ngHFk0hjfS\/RtVTuPe1HcpLyH\/ygDpz5ehSbMfUrmz5lQSRMm3S\/zxv+nofF3\/Ubm3XmVzO1\/rtnxkB0b6GNu\/D8x25kOfGWPWXkyb8EOYfXJH+s2FtbZThVinj3fWjQpLk3rUBVyghpgE7HVt29fMwEi1L2+6QxFEYONF5whGH\/jyEzS4MOLhOrVqydYl0hkJNK9bZrVF6XuZzUSqH\/XDGG3BmjpkE4CsZ0p27EoyHVLlpyZhcJ5jOos8XwOmyL4fpduX3sSyCwegLNLLTLBHcxaAuA1VQD3MHbMGDLEObyayollfvqK\/OhwdOO5yHFLXm1ktWySJgpyQA1xPWTRJsmdtVqQmQz1dKMOtZURVyAzNIQTC8CyTtd5551nhl2GDh0qBIJgjSHCFgkgCVVJwjuRBRE7TYSX7z1OR5dvmleusR542Dm6USbkuCXPDVkAG6sNoJGHdV5VtC\/iqiLLS\/WMuEI1EBA3IAMy51I\/zDWGKPu2bdtMNBeAVALwAJ90f0Qa\/Wp1khGeaecj+9OUd3gAmuY3oJ7+1lbvFCwBSxI3IOskhj59+hi1NWzYUGg+X3TRRbJu3TqzOADWWAlHWLAmMX1g+sL0iRFInHaizkemqelWUxhduCkvGrKua9\/I9K3Pf2i1RGKZo1E2ZKJDD5MpWtyArOO6\/iZNaJopYZj\/CPgg8GPDhg3mDprQeKtDzUcmMy0AXlikxLPKy8tNn89LsiiLV8um5erSPE36dmoqQ1\/cFJH+VB51jpTcksXwJt9ZNCluQKbP++ijjwqrfwBmAkCYNAGPtJpWGmuNdxs5NMfDmY+szyCIoLZjl3rfnj17pLS01MSIK6+2RzfHMCmDm\/KiKevG806Q9HoieXMLq405U49QFM2yhXp2sPSkH0cGwEwzZNyYYJDt27eLWlQFWaCj07Otji2NGKM5znAV4ZgMCWGZtSmPPJrdHJXYijUzMzOisU1aBOnp6cICgZHKSuX76TOnpaXJ4Ne+jeh9eEmHST2OjLMLq8lkfAU0iwksXrxYSFOQBToyiYLJFIAWrzZAzavlfGSvjSNTZ37hOfJRR0rIcUteLGQtrRiD7nlOU+n7zKYajf\/Gomy1eRdJPY4M8AjOwJIVFRUJVpJzeKTxUmpCNK0ZI2asuEOHDpLI85FrUu9kzYsnm7pF6gBDRipQ3PrIOLRo+tKU\/uKLL+Tiiy82\/Ut4pIWjfGZKjRw5sjIrPwTcD4P+NkNSnDuJHw3ntRfP+dV3azyU+rkpL5aysMwEjlzzRKEA6EXr91CdgBTLsgUsRJwS4gZkHFr0Y3FOZWdnCyAcO3asEAxCWih9AGBdGoi8NMdZN2vQoEFm6Iq+N+mAOdHmI1MfS0c1gGUmcOTiMxsJkWAAmiPjzpEMVR2Vnjz\/4wZkVKjOKVbzoGm8dOlSoe9LWjDCEvfq1ctsK6P5AD\/Nab3mSHMdi8+wFD8UWDksNjzSlViAD4dbJPThhx+ahffo50cih3tXrVolS5YskRUrVriyRK2b8uIha9eXf5ObW++Xebl1pPVP22Txmytl+lvFgqVucvd7xlrj6Qbg975YKJNf\/0qw3gAd+uuGIoHQbbjkZj1xtuq3Fq1jXIGMVWWoyEmhIrhQBNMf+RHg3En8INB0Rh5AJx\/pgeYjN2\/eXLp16yZYbDzfkRAL7y1YsEDGjBkT8bKyyGJ72QEDBviRVfNla92UF09Zo4cMkI8XjJX6q2ZIoyW3y6lvj5aT\/\/dZ+eavL8n69143AJ+\/bp\/MeneHsd4AHfq3P2wX6F9fLhclroNRn\/mfCrO2+j37ecUPxt8iokGvlsqB7vcI3xvfYzQobkCmyUvfGAvGcreEVnIMZ9lbf4pAHj8CABhPNqtv8kPhL6\/yUCyL\/eH1tlRQbRlar+vkxacfkz9OGyKvPHCDLBl5hbw2+Hx5o189Q3+6rYUovTa4U0VaJzPNkllY0Kjrukgg+nWPdtL3qh6uEgaD702i9Bc3IFMfmrk0eWkCAzzCNX2XvaXvi6XEyjIjivFj7vUl3xBNAI3Di2Y1TRunJ5zn6f0olzXCLOVIMunglx3OEiWmWEI4zpToewcigO424bjTby4ax7gBGc90VlaWqRNgZrYTHxKgM8xj\/+j7YhmwsowbB+pDIwPQav+XJjZN6gsuuCDkutbHHmUPVgMJq4G4ARmA1nbZW3\/axlnGkrkjRowwm6QT+km4Z6tWrUTHlxlj5py8\/mS4zaM1QRnwzG\/atMlt8Ukoz1apthqIG5ApMBZYLSzOq2AWl\/y+hHMLUj6ykIH1XrlypShgkQ0P4lzzR\/vIbK5OnTrJtGnTJNyItWiXycpPTg3EFcgMI9H3dRIOKxxXXlY35dNwUC2nsy4scAB\/8+bNJsKsSZMmwlxrZz+ddEtWA25pIG5ABgw0f\/FWYymVnJbUrUq6KQdnW25urpSUlFSKhYf1ZYyS+rA0Ec3qk046yQC4MqM9sRqIkgbiBmTqgzNKFwLg2uvEjw9jxfPmzRMWQtDyfvDBB4IHHgce9SFenIUBmJknuEEAAAV0SURBVA3F1EbmtUJMztB77DHVNBDd+sYNyPRfWY6HmU\/RraJ70ikzfXKayb5S8cDjwIOP95zhMOr3+9\/\/Xh588EFhYzqa2KRbshpwWwMxBzJWjX4w\/WJioSHOlUgjj9sVjYe81q1bC570yZMny0033RSPIthnpogGYg5krBr9YO0T+x5JI0+i6Z\/1wegXU27WDmNcm3NLVgOx0EDMgUylcA4RpYUVJmpLAUBaIhJTMOkX45UmIIX+MtY4Eetiy5yYGog5kAEt0w3ZWnXLli1meGbhwoWJqb1jpWb8+uqrrzYhjgSkMHFC+8vHstiD1UBUNRBzIGO1qBHeXY4s9aPL0HCdCARwiQnnqOVlphXdBAJSnHxNt0ergWhqIOZA9lcZVqDEUvtLszyrAauB0BrwBJBDF9PmsBqwGgimgbgAmRlOxFnj7GIiAzOfaGpznTTDT8G0btOsBlzWQMyBzNASQ0z0J\/0RaeRxuZ5WnNVAUmsg5kBOam3aylkNxEkDFshxUrx9rNWAmxqwQHZTm6kiy9bTcxqwQPbcK7EFshqouQYskGuuM3uH1YDnNGCB7LlXYgtkNVBzDVgg11xnMb+DqDcmlzDO7qRQ63bHvKC1eCATaMaPH2924KSOLJmkYrTeunSS8vVIOjpwddqrCk+wowVyAr0wFvB3jr2zyEECFb9aUQEiK64MGTKkWlo4DCamEKs\/d+7ccLIndR4L5AR\/vVgwLBkRcZxj4XSKKDy1Vlg1rDlp+fn5opaMPKShBu6Hr+fkh5RHPhYdhODzXMBIfvLAg8gHaTp5OIdHXiXWOGOzgHACgJzyeQbykKsTVKi3yk3FowVyAr1152oqAFBBSsgry+2yUwc7WjItFMvN9jvcQ75JkyYJCwOyvhhzp4NVG8Axu4u51RCLJsDjHhYdnDJlitlcjucCRtLY1YNztgBiL63MzEyyC7PdWL+MZxKGa5jH\/rG7CEA+dmkOlBegQuRHJgm0PqgTRD3ZFwuLTBrPol6cpypZICfQm3c2rZ2hrCxiyMJ\/rBMG0IhfBwiAAoCxsyDVBBh8\/KwlxnUgYpcOAER+iHNAR\/5GjRqZLXB5Hs+FR362scWyYiHZVbNt27Zmrjk\/BJSLrXNJJz+ENQXcviupOOvIvcTkk18Jy8x2QM71yfkxoAyaJxWPYQE5FRWTqHVmkT+Ah+WCADxLD9W0Plg97lfCItZUBoDjBwDivKb3++an6Q9onSD2zZOq1xbISfTmsW6s4Kkrk\/Lh05fk46eaWDgsIc1mrtWqYs2cfPIzI41+J3xkIIt7\/BH5sfw04bkHi89RrSm7bmLZnffSMmBJJKy1kx\/oXJ\/PAg6+eSg\/ZfDlp9K1BXISvW2arqza+eSTT1bb\/2rcuHHCHlSsLwaAqDZgAnQ0wZ18LN6gQYOEJjoAZKlffwBCBkR++uMAl3sAPk1s5AMw32Y190BYaaw158GIHyc2M6CcdBkg9RHwQ1NYWCiUP5iMZE+zQE6ANwwg2JHSH5jgkUYeqgKAWG6IJjHNasANH7DBI439qOBB3K98Ft7XJrTySVMeMlQmz+O58JxyyM+98AAZ1h\/Acu1LAB8ellzl6b3wlUczn+ciW4lr6oYjLSMjQ6g396QqpTyQU\/XFR7vegJMFCXmOApZzJwFUQI6ldfLDPeeHghbI9ddfH+4tSZvPAjlpX23gimH11MoGzhVZCtYSq4nVBrCBpGHRa1sW5NKKSHVrjG4tkNGCJauBBNeABXKCv0BbfKsBNGCBjBaSlGy1UkcDFsip865tTZNYAxbISfxybdVSRwMWyKnzrm1Nk1gDFshJ\/HKTuWq2blU1YIFcVR\/2ymogITVggZyQr80W2mqgqgb+HwAA\/\/8N2ZRmAAAABklEQVQDANhbCIeObqfCAAAAAElFTkSuQmCC","height":146,"width":242}}
%---
%[output:34dfda30]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: ","truncated":false}}
%---
%[output:13e9fd7b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPIAAACSCAYAAAB\/nJf+AAAQAElEQVR4AezdfchlVfUH8D0\/isY3nH8y8yXG1P5I1PmjbEAag5QmJlEsKUWQFCLJ0EGTxtAxDAdFQ0lQKw1BFCMC\/xACmz+UBqYsmFEGgiwHdUylaPA9IvzN5zTrYT\/nOfc+59yXZ+7LHmbdvffaa6+99nfvtV\/Ofe4+\/\/dB+VcQKAhMPQL\/l8q\/gkBBYOoRKI489V1YGlAQSKk4chkFBYEZQGCWHXkGuqc0oSDQDoHiyO1wKlIFgYlGoDjyRHdPMa4g0A6BRY789ttvp0svvTR98pOf7EnyybVT307q\/vvvr+o755xz0htvvNGuUA+p3\/72t5WuaMP111+\/RBIv8oXqXyK0woznn38+nXnmmZXt2hDVwwMuOS\/yhgm1WdvpVscwuqIsG+nMaZT6o56m0Jg0NtWtf5tk6ry8jHLsr8tI48vXP\/oJry3ldbS1q61u+sKmRY7cRsHOnTvTpk2bhna4NnWNQubZZ59dZKtBizcK3ePWYdCcd9556eWXX65XNXFpg+pb3\/rWErvYrg3asiTzEDNefPHFtGfPngUrfvOb3yzEJz0S43jNmjXpYx\/72GBPrXXOc889N+ltrezbv39\/ev3116u4D3E88Umi008\/Pe3evTv97W9\/S+eee+4kmbasLZz0qaee6in31ltvpW3btiWrU0+hITOOPPLI9Oijj1b43Xnnna207dixI7EthE3wHCTSkxzGOP7sZz+bjjnmmN6OfNFFF1WgGFhBTzzxRDrqqKOq9r3wwgtVGB+x\/bAFQbHkR36EOl0eGWSLF3n1sK7T1qntYGDnCSecUHWUDgvd4jpv\/fr16cQTTwz2Qli3j41N9dbl2Kot5GM7yVZl8axYZMSDyEfFuT5y8i644ILKfjJWO7iRo4sOutUhHymDH3J4SBk8eYgcfhOpm0xQvY6mMgYVTOX99Kc\/XTRujCN8K58VUDyoXhcb2Rr5EeLJC5uU0wbpXlhH2V4hh33ssceqbDsG48UC9etf\/7ri9ftQVr3qZ0f0hzQ72durPNvJBSmfy+a6Q0Zd+LlcjONTTjmlYnfeWlelDnyEggPRpCEGmniQjjUQc0M1Ak9eyN1xxx0JRTpC5eo6u27r1UXfM888U60GBr043qmnnipYRDrgsssuW3CeyFQvW5THa2qH\/KZ2kEcGCBnxIPLaGelxhE22qhfV62NL3UZt73KUeuihhyqsQ7fV0UJgt2HXEfxhxgwbm+wP3W1CO0qOy4GvvvrqxJmVMz6in6WXI3bo25Azto07uAcvQnJsj7RQ+ZDlrBdffPGSoxQ78eUrgyykbD\/77LMle6\/IKo0ZIUIGMtRqhmhgBFnxG264YWE2jpn4vvvuSxwEODqZnJXQANHBZnC8nMgrhxc6ySunUVGf\/H507LHHVjuIffv2pXfffbcicQB8\/vOfX1L04YcfrpyY7WxD6icYK0qXdiiXk7bSme9seg2cb3\/72ymXU7buDLnupngXW4fB3FhAbNBPZ5xxRvXQrtfqtMyYSfqePV3sV3cXivPwaaedlk466aS0cePGqnj0c5Vo+aFv9Ku2G6OKGevsF8+pSTZsMa6N7\/UHdosmGjrJK4+PJ07vq6++muJ8jDfQinzFFVckZxIKwgiVW83wEAfQKI5vG2BbBSR5l1xySbWvF3ce5DjiQeSVUz7ynAOUI9Nr8MvL6eSTT046KkAAhDgAOHkuKx6rh9DsZ0tjxpQX1KUdUUYIHyRuZYoVQHpc1MXWYTA3Fu6+++4lRxV9aPK3EHDeaOc4xkzobhPqW+dhshs2bKjGssnHeGOzCV1eGzI+jWGyxuhNN90kWj1Eg3+VOPih\/5EkWedbcU7JOU3enNdZH9+xpr6C49PLl5SnB28gR6bcNkzljKDouOOOqwARR4cffng6\/vjjRZNtQBU5+JFvy7Hq6ZDndBpuIKBwKquqFVbZfnTEEUckdpExeJB4DoB0kA7mvOpSr\/ojryms211P52XYYcDnvJWM122rp4fF3IB6+umnk8kSdvW2XXfddQs7s3GMmXp9\/dKx8pExpur9zcmNBfnLUR1HT5Dt+EwInh3k5ZcbA\/yJ87LHxGKFz8tHnF7687p7OrKZxuxQJ3wKPSj4xz\/+ITrRFFsmT1X\/8Ic\/VLYGr0oc\/ADitddeu3A+0U5tt7M4KFKCFgiYrKwosEOxNTTwrPotVIxVRD\/b0fWrxARuQuonM468W265JYXzmgzZEPjl9cWC1MqR84K94lZds4x8syyQxJEV08opnlcoHbO\/OKqnQ95WR8MMiJzM\/FYAZZejfIZ85ZVXqjMzXr1cbi\/wbK\/rMvV03e56ui5\/KNN12+rpQTHX57GK2M3UVzJYW6G0XZ0cfRxjhv42FNvS5WR7nXHr5bQp58Vqqc3anuellHom33nnncSHCFg8TIawks4J3uT4hlU78nquyLYflvg64Sts28yRY3XjcI888oisimxZzGwa5MmaBwrOqzLjYYa4s1PolEbklVM+8jQgBownnuTaUF4veTbgifeimPEMSjuPXE5ZOvDkkRFvagf+OCicDuZIHR4OwVU8KLdVHhl5TbYOirnB5pxJr\/6ykuRjxhnZaiw\/xkqEbG87ZkaFtV0Be4wvDxPzBUKcE7HVGZTTi\/cj4xOeZIyFW2+9VbR6NgP\/KtHxw47BeEcmlLx4LDjhf5HX05FDoFcYD7wc9G1DyXHe6EQNxLvrrruShzs6XBk8QOpgss7beDmRj4dBodPso+N1wOWXX56L942rNwYaQasBnnhOVnhnZzy2s82gNDjxgpSNdsgjQ1Y7Pv3pT4fYSEIzugdzlNEfT4HDkYOvfnjCFS8ot1UeGbJ0hUyEw2DuISccQldTKB\/JG2TMjAJrjmZCYIPJuMnRYkKDV9uHXvCEq\/axk35jBP7ibcjznBinxrnxjsTz8rbb6iCb6+\/syBzJTKYzogLbUNvRSAub5JRRVh4ZZBKIWVA6qEmn7YSzrkEXcm3CfODHatBUzhlFZ0Se+Pbt26unsTrWbC6v3g7t0a6vfOUrskdGJpd4Uk8pG2zd1F\/HuxeOZNnGRjpQL9lBMTegbAXrNqkL4csnJ42a6mIjW9lMBonjyZMWSg+CdTgBPXVHwEOcm5OLt3noZewi8ijsY7d0F\/LUWt9EGePd+DMO8ewUYyufj2l5ixwZ0AC3xehFvb7LZHheppccJ5QXsjpUA6TrZ9+6znq+BtQpyqhDXfKDpw5xPE5CH5768erth4WOrcuRpVsdygul8XPK9WlnnietrDrIKU8PXthInm14QZEnDJ6QvpClhz7lkTgeOZTLahssyKG63no+mV5UL6suhN9UBl9+EBvZWpfFk0dOKF2XgSEsyWhfPV86rw9WeHXK9UTbo1yvuulSL6rL5Prqdkkrw25ybAkevvqNP\/nS8qIuNpEPWuTIwVzZcLpqsz3zUMdWSiitBc4zzjbitui5c+AV6o4AbGFcsF4eu+LIy2O0SIKDclRMZxXbHgMtP8\/0274rV6gdAgXrdjiRKo4MhY5ki5Ofi6L4MOej0FHCxQgUrBfj0StVHLkXMsvw46zi7BJUPx8to6Jkt0Rg0rC2U3B+1e9sa9mMsYrNtSPv3bs37a3Rrl270q6R0a5Doqvepq7pP\/\/5z6kL7TqA12uvvZbGSWP1ghlQPreO7K+8fE9YJ19tTDvV29Q1\/fjjj6cuBK8HHnggtaV77rkndaXNmzenrnTVVVelLrRly5Y0CLW1y5gb15wx1448LlCL3v4IrF69OnUlfxjTlfzCrQt1tSnk29pVHLn\/uBgo1+0ha9euTWtb0uc+97k0LvIXV4eKfL3TldatW5fWdaC1LTHO5dasWVP93nZNy3CgQbDChYy5cVU5tysyUP2pZ1vauHFjGhd1cYpRy37hC19IX+hI\/5t0Lkhtw7YY53LXXHNN6kJbt25NXamL\/pDNbWwbD5yMueLI40Kg6J1bBNqu9rlcvmtoG4\/Jd5xAz+2KPE5Qi+6CwEojUBx5pREv9RUExoBAceQxgFpUTjsC02d\/ceTp67NicUFgCQLFkZdAUhgFgelDoDhynz7zM7rzzz+\/uv2xj1jJOogAvHwn7ddgyE2rB7NK0AcBP4GNa6zgFlcH9SmyJKs48hJI\/sdwv5Xrhl566aX\/Mcrnsgi4lsmNJn5M4M823RM2yKBctqIZE3BvmSuo4OY2Ffd+mRS7NLO9I3fROiGynNGK2gSKAWb2C5IOs8n7+2R\/O3z00UcHe27CQXHzk8P4NZCbLeLKnHkBblDcYAa7wKl+sV7w+4Uz68ic0Tt93nzzzSXt57QuTLdqxCworSMI+5kaYF2IJj1PNAxuOU5uoHTHmAsXcv6sxofFLbbXLvLrenEfTGfSkTnq+vXrFy6b19AggLli1LY57n5y\/5G0VTjk5jEcFW4G9Y033phuu+22hVcDzTKeo8DNnV3u5nJrpq01nV0wmzlHBoBZzQ0eqA5G3Avs76bzPOk2tybmZWYpPirc7GquvPLK9OCDD1bXIM8SRk1tGRVuodtd8bbWcVtm8JcLZ86Rra62y84dTY13newHH3xQveV9If9gZP\/+\/Un+weRcBaPAjRPfe++9yd3RjifzAOAocPN0H8HLkcTPHd2vLd2WZs6R2za8yI0Wgffeey9t27YtuXfcubjpIeJoa5wdbS74dwMrzPxS6uabb+68mymO3Gc8OEMDWNhHrGQdQOCwww5Lznh2QzlZsQ5kl\/99EIjzceA2CGbFkfsAXLIKAtOCwNw5svcprVq1qvEs7Hen8qel81rZOSIhuKxaVXDrCudK4TZ3jhxPBb1HJ+8UaRfPz8tDmrztbeIFtzYoLZVZKdzmzpGdR3zh7o2LvjoAvdBDGle3SBdaikDBbSkmbTgrhdvcOTLwPUzwN62+b\/akUBivf5VfqBmBglszLstxVwK3mXZk3yV7I0DTdhm48ZRQKL1ch8xL\/tTgNmEdcihxm2lHnrB+LuYUBMaGQHHksUFbFBcEVg6B4sgrh3WpqSAwNgSKI48N2qK4IDAEAh2LFkfuCFgRLwhMIgLFkSexV4pNBYGOCBRH7ghYES8ITCICxZEnsVeKTQWBjghMlSN3bFsRLwjMDQLFkeemq0tDZxmB4siz3LulbXODQHHkuenq0tBZRqA48oT0bjGjIDAMAsWRh0GvlC0ITAgCxZEnpCOKGQWBYRAojjwMeqVsQWBCECiOPCEdMctmlLaNH4HiyOPHuNRQEBg7AsWRxw5xqaAgMH4EiiOPH+NSQ0Fg7AgURx47xKWCWUZgUtpWHHlSeqLYURAYAoHiyEOAV4oWBCYFgeLIk9ITxY6CwBAIFEceArxStCAwKQiMw5EnpW3FjoLA3CAwFkd+++2306WXXpq8V6kXySc3SqSff\/75dOaZZ1b1ejHbMLpzXU1tqNuvPnLqV3aYule67P33319hxv6gevvGZROsYKZeGLap54033kjnnHNOZbOydDSVu\/766yuZuwInqQAAEABJREFUQdpibCrHLnqa9A\/Koy\/s1mZ19CMyy9U1FkderlL5O3fuTJs2bUo6RXraaNrth3cM1jvuuENyEU1y+5577rn08ssvV\/a+9dZbaceOHVV8Gj6M92effTatWbMmeXdyG5u9ZNBk20\/2kDkyo3SGThGfRmK\/17NOo+1s5qxIvIm0r8nJm2QH5Z1++ulp9+7dqcuL9LzLOq\/vmWeeSSalnDep8ddffz3t378\/dX0X93JtHLsjX3TRRVUn6aigJ554Ih111FEV1i+88EIVxodtkm1HvtVo2lrouNj6kBV\/5513Qs2isK7TtszMuEioT8IrWA\/aXrXF4D\/xxBOrEnX7MdnBHnahpvrMsPJywlM+qN5GsrDRnpCJEEbyg9SvfOQ3hWG7vtAn0UaT6\/r166siVo86VraGUY+wV125TWF3lI0y2iKPHvJVpX0+yD\/11FOVhLEloj+QeD\/SDn2hLliHLdJsoLtXebaRC1I+l811h4y68HM5uwe7iFNOOSVnV\/6Q90H0RbRx37596d13310oU7dn7I68UHOPSN4g4FxwwQVJQ3NxWwugBw84tuV554kbHPWyGlzXaaU577zzUr+Oi7q6hupnB3uirPquvfbahVVDO5tWOjx5ynFC7c714NN\/2WWXLbJdGbLyg5SDEayC1yuk8+GHH17IPvLII9Ojjz5aTVpPP\/10OuaYY6o8ugzO+i5EXWeffXZfm9ShH+plK8UdPsIRTKTf+c53Ukw49VV6OZWwzm0J+4yXellydXyVD1m4XHzxxQvb\/Siv3\/HlB8\/kaeKEV\/DahMcff3w6\/PDDK9Gm\/h67IwMhZqgIdSjgdAJiHae67777RJNZKGakG264oeLRE8CJA0lGrJYGk87FCwLgrbfeWiVDZ6w26s8HbyXU40Mnhu1CNkf9+UQUxeWrRxvC\/j179qQXX3yxeibw2GOPVaJhO1llMGMLRVYZvJCLNua257ipS50hx0ZY0dFEMAnMyGkb4qywq5cxeOk0EGP1iLrYtG3btmqyUjbaqF3axy721XV2SZvc4KOMrelJJ52UNmzYIJmadg5VRp+POq5EH3rooaoN4jk1ycbkATu45G0lrzy+9ouz\/9VXX208H8OPX8A\/J7qVveKKK5IJtld\/j92RGdGLwjj5+Uybd7jVB0BkAAeM6Ex8JM+qcdNNN4kuEAABaeBdfvnlFR8Y6pUYpPOVC1I3+yIdIf3qkTbzql9HOR+x0ypnYJ977rnJ5HTGGWckDkG+ia677rpqtcvL3nnnnZVojhvHxCR3ySWXiCZYwaxK1D7I3XvvvdW2Ls+Cmbbl203OCS9yV111VXK2FacjcDfxmIACd\/k5FrCiF38Qolsdym7cuFGQAl82x6CvMpb5gBX8iTW1AT+IzUiarElEnFPC1gvO9addDL4dmclfPKewX3l68rx+cXUjMr36+5A6ssbaJjDQlkOYbyGkOcRxxx0nmgDn\/FklDnzgyz8Qrf57CshpqsSBj9DJifLZTr0HsquHDpxLvCsZCDour58O9bNDvBdx3ph1w5a6LEex\/cfP7c+PGPKijQayzg69Vk\/59bMVXk7q8bBp586dKVbnyFfv1VdfXe0i4OQhjbz6LkR7tZs8OTIIT544gpU+Ex+E7KDUoSzctFW\/Bq\/fpKVMTm3boAyb2S7eRJyZ87Kn36QMG7bW627SmfNM8nFM6tXfY3dkA95sVSd8xtqCme3FJ5Vsk+r2x4rY1WZbIytslKPbCsYJgxehOup5Vh0DJibAkB02tELETkFbY1dkgmDDsPqHLW+MxI6gly6rtVWvV\/64+LfccsvCjko\/wku\/1uuzo8RrcmSTXhxX4B+EJ08\/6Hvlm2jsjtxUaRMvGldfQcx2VmJlzIxHHHGEaEX48qvEgY+Y8Q5Eq\/+hExAACXAitBJZkSrhFfoIG61+ZtrY3vWq3kpg5Q+b6xNgtDH0hVyEnJOT1vVzDGdhk4LVJMeRbOgVtwpYWX33GWlhULQJzuSCb\/WRF2l16LNIdwk5h8Hcr4z6rNr9ZCJPmyIuZKfy9TbI60d2iNEmk5++0mf1MtF2\/WTVrue3TUe\/0GP8RD+P3ZHNIgZLnfAZH1vp\/KwT20L5jzzyyMJs51wEpHjAoSHyyRmY8WBLGgFMg3VQ3sG2p+xpGsDKrQQZlAanurQDiQfl22\/x4EdH2uYafDlugalBo23aqK1RNg85t7ManrphRT7I1lWega2OXN5DSTsL+Tnup512WvIAii64y88fHukrdeF3pVjN6KUjBnCEMcFZtdm0nH5YBa7kY+xEG5Yr35QfW3v4a3cu46sji1SM9zxP3Bh1TAj8I8STR0bf6wt9YvxoA776xu7IKupH8TDEyughClkGRkPCqc12sXrpNB1KVj5ZWxqNwwsy+OKhT65TnEzULb5SlA9yzsJ2Yb1+7UH48skh7cVzfoYZEseTR0YdBrsOj4d88usE08CxnhdputUhHfIGlgGmLjbCnZ677767erKa484O9pA1AZxwwglUdSKTRnx3bPKhv67AJI\/Hluhf6YPUGASu0QZCXceEHWK+sGgr0m76gkzabCNrMQp+25CNSF\/oE+Xy\/j5kjmyQ2e6GczLM0z88edJBzhvyIq0jn3zyyYXvEPENJHL1ssrVdZLBy+umYyWI7fUnxRyE7eqPc57Otk2Th58TnvNz8MSjfPDgYfDr+ODVQ7bYetNXzwuM6I68kDeRBk9ogOkP+dII7rlNYc9ZZ50luxPFk1qFwmHFc+I86sCLlVG8F2kzivxo7yBjQltzTNixffv2hfFpNxFbeatq1Nk2pNtYMCaU0Sc5tnhjcWQVqji2PU1hr\/OpgScvL9MEbr0OA5JclBXXQFTXSQZPXi+ST44dua5e8mTIKqNsyInjySODn\/PwDQR54mTlk0Py8HPCk5dTlA85eOSOlcvW4\/RFuQjrduRlDKSQE+pr\/ZHLiOc29bJHW9VFD3nl6pTb10tGW9VBT9gTdka6n142sCVktEc5+ugJvlAaXz65nIfPDkcM+dLkow11+6XJ9CPl1ZFTvdxYHDmvsMTnDwFbYd9B2047qzvDQcFZ1BlWfNAtprKFliJQHHkpJoUzJAJWIw+NqHFWtO3l1Lbgzom2sR7ayC80NAKVguLIFQzlY5QI2G7aVjrb1fU6Py53dq+XKenlESiOvDxGRWJABJzt6mc\/50fn2QFVDlVMvepnkzPrUMomrPBcO7KniV1o165dadchoNdeey21pQkbX8WcFUJgbh35lVdeSb\/\/\/e87ka+shqXHH388daV77rkn1en2229PTbR58+ZUJ9\/PN9EPf\/jD1ERbtmxJw1Dd1kHTTe2bFJ4\/MOpKxty4\/HpGHXl5uMYJar\/aV69enUZB\/lyyLR177LGpiXrZuXpIG\/3V2Sjo\/fffT5NKe\/fuTXs70jjH3Nw6sr8wWrduXVo3BK1duzatXSFas2ZN9TvWNS3CXg5a+IcWAWNuXBbMtSP7M8NhyJ8\/rhRdc801qS1t3bo1taW2OgeRWylsJqWe5cZSceRxTWNFb+tVvs1OoC6zUruVSalnud3dOIfb3K7I4wR1vLqL9oLAUgSKIy\/FpHAKAlOHQHHkqeuyYnBBYCkCxZGXYlI4BYGpQ6A48tR12SwbXNo2KALFkfsg52d3559\/fnUVbR+xknUQAXjFPWB+7TTqCwIPVjNzgZ95+rknzFBcQdSlocWRe6DlN7WuVHnppZd6SBR2HQFXz7hayY8S\/Cmrq30GGZR1vbOedpeZiyXh5uYP94eZFLu0e6YdmTNaUZtAMcDMfkHSARx5f0f7wAMPpKOPPjrYcxMOiptfO8WvivLfJM8LcIPiBjPYBU69LuiL\/KZwZh2ZM7pc\/c0331zSbk7rbmmrRsyC0jqCsJ+7AdbFatLzRMPgluPkfmmX9LlUIOfPanx53K5LvcYbTGJ77ULArhcAKj+TjsxR4zYKjcwJYK4qtW2OO5rcfyRtFc5l5y0+KtwM6htvvDHddtttCy+Am2UsR4FbXMbgRhVbazq7YDZzjgwAs5obElEdjLhfuH4bo7T7pAzCepl5SI8KN7uaK6+8Mj344IML74eaZfxGhVtg5I2LttZx62bwlwtnzpGtrrbLzh1NjXep+wcffJDyNyKEnJ\/eyY\/0PIWjwI0Tu+rXa4AcT+YBv1Hg5uk+gpcjiZ87dr3TbOYcGRiFVh6B9957L3mtqvu4nIubHiKuvFXTUaO3VLqLG2Z+QXXzzTd33s0cWkeecJydoQEsnHBTD7l5hx12WHLhnt1QTlasQ27chBsQ5+PAbRDMiiNPeCcX8woCbRCYO0d2Nl61alVqOgv7Pa38NsDNmwxcVq0quHXt95XCbe4cOZ4Kuj0z7xTpXi8Iy+XmNV5wG6znVwq3uXNk5xFfuHtjn68OdI\/QQxpXxkiPhGZMScFtsA5dKdzmzpF1h4cJ\/qbV982eFArvuuuuzk8K6ZonKrgN1tsrgdtMO7Lvkr1ZoOk7TeDGU0Kh9GDdNHulCm6D9emhxG2mHXmw7iilCgLTh0Bx5Onrs0NvcbFg4hAojjxxXVIMKgh0R6A4cnfMSomCwMQhUBx54rqkGFQQ6I5AceTumJUSs4zAlLatOPKUdlwxuyCQI1AcOUejxAsCU4pAceQp7bhhzXa5gtsb\/X25v277+te\/nuJ+MzeHfvnLX66uAXZZwJlnnpnih+\/1el2d5CpXJF7PL+mVQaA48srgPHG1\/P3vf09uCXUN649\/\/OO0efPm5A\/8Gbp79+7qZeyf+MQnJAtNAQKtHHkK2lFM7IjAP\/\/5z2oFtvJeeOGFyWWFH\/rQh5KV2mUK69atm8urgDvCODHixZEnpitWzhDbZFfKuK7WpfK21n4BxoJ\/\/etfac+ePanrnVHKIrrpayJbdFt1coVGi0Bx5NHiORXavvjFLyY3jK5evTpdeGA1\/slPfpJOO+20yva\/\/vWv6d\/\/\/nf61Kc+VaW7ftBNX04\/+MEP0lFHHZU48tq1a7uqLPItECiO3AKkWRM59dRTqxX3wx\/+cOWwmzZtSh\/\/+MerZv7pT39K3hLhZouKcfAjVu76SuuiPXcxHxRLdNMXtGHDhhS\/QHM5H4cO2RKODoG5d+TRQTn9mqzEf\/zjH9NnPvOZ9JGPfGRRg6ym3\/jGN1KdvvrVr6aPfvSji2Qj8Z\/\/\/Ke6WXPXrl3pRz\/6UXJfc+SVcLQIFEceLZ5Trc3XTnv37q1W63pDvvSlL1VvjvD2iJy2bt2aTj755Lp48tDsF7\/4RfrVr36VvLrnrLPOWiJTGKNDoDjy6LCcek0ectn6juJrJ19h\/exnP0tf+9rX0je\/+c20atWqqcdnkhtQHHmSe2cFbbOC7tixI61bt27or5327duXvve971Ur9ZYtW5Kz+Ao2ZS6rKo48w93epWnDfu0UdflKi\/N6\/c73v\/\/96ml15JVwfAgURx4ftlOledivnTT2v\/\/9b7r99tvT7373u+RrKHG2XgIAAACSSURBVCvzk08+mXL6y1\/+QrTQiBEojjxiQKdVXa+vnbq05\/33308uMlTml7\/8Zfrud7+7hLZv3y670IgRKI48YkCnRZ33WXkg5eZHNgt\/\/vOfL\/naqS5HNif3NnvnE\/Lds5Az9yL15OVLfDQIFEceDY5FywojUKpbjEBx5MV4lFRBYCoRKI48ld1WjC4ILEbg\/wEAAP\/\/oIvsogAAAAZJREFUAwBqrt8furfYRgAAAABJRU5ErkJggg==","height":146,"width":242}}
%---
%[output:248d5970]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPIAAACSCAYAAAB\/nJf+AAAQAElEQVR4AeydCbRVxZWG9xMHUCaJyAMBH0oEg6goKDaKmEjnqSDKCi5g4bBIOwAaWAyhkZAgiAOCLWlRIS2rMQqmSQgRjcRhCS5YDmAIIkESkFEQQXkoCIjG5ivcz3qXe+6973KHc+7dLParql276tTZ5\/x3V+1TwzHf2D\/TgGkg8ho4RuyfacA0EHkNGJAj\/wjtBkwDIgZkewtMAwWggUIGcgE8HrsF00BqGjAgp6YnkzINhFoDBuRQPx5rnGkgNQ0YkFPTk0mZBkKtAQNyqB9PYOMswzRQRQMG5CrqsIRpIJoaMCBH87lZq00DVTRgQK6iDkuYBqKpAQNyNJ9bIbfa7i0NDRiQ01BaPousXLlSzjvvPDnjjDMqqW\/fvrJnz56UmoUc8q+88kpK8sj51yI+fPhwV7a6dblC9icrGjAgZ0Wt2a20Xr168qc\/\/Uk++OADeffdd93F3nzzTRdm40+zZs2E+vV6W7duFcBcu3ZtmTVrllx55ZXZuKzVWQ0NGJCroawoiPoWFLBpm4ljTfv06SOff\/65suWJJ56otOzIVGYERADvqFGjZM2aNbJhwwZR667WmWtA8OFBxJXXvXt3oVfBdeFffvnlrg1+u5El\/fHHHwvykydPdm2kJwKfMshQR0Azi45tQI7gI9+9e7f06NHDvdznnnuudO7c2VlFADJu3DhnrbHUWE5edl7+pUuXOqt63333yaZNm9xdw3\/99dedVVd5eC4zwZ9GjRq5XIDmIof+7NixQyZMmOB6CVhvfizWr18vTz\/99KFccdfo379\/5bVhfvjhhzJnzhy54447SLqyWP2f\/\/znsmDBAsf77LPPZNu2bS6va9euMn78eFcGGdrOD4UTjMafrLXSgJw11Wav4tiuNS80gN2+fbs0bdpUWrRoIVhOAL527VoHig4dOsipp57q8tq0aeMaRx6g48cAIq4AcgLV+MM1GzZs6Cx0x44dHWD37dsntI120B6u0bx588paTzvtNDnxxBNdmu45PyJY2okTJzqe\/ikvL3fRli1bit4Hcce0P04DBmSnhuj+ASAABVCmcxc9e\/Z01g5LCE2aNClpNfxgIMQPAyEECAEqVpcfBB+w5CcirCrd7BkzZjjLjbVNJG95R2rAgHykTiLFAQRYPSwUXd4tW7YIXVqfj0Wja01XmLxVq1a5e6TMyy+\/7MasyAMmLLvLDPiD3P333y+tWrWSU045pYoUlhiim073vVatWq7bT\/sop\/wqhbwEPwIkkSc0Sl0DBuTUdRUaydgxMg3r16+ftG3bVn75y1+68TPWsUmTJm78SbeVLikgu\/vuu0WtJfwBAwYcIU99Pm3evFkoS7dX64213PAZ8xIy7qX7juWmXdQFH4ur14anpL2K2267TTp16iQXXnihML7\/4osvVMTCJBoIAZCTtNCyq2gAsK5YsaJKd5hPQIABQcBJFxnywUYc3vz58wVCDvk7Djma4EPIwPMJOfJ8UjmuybWRoZu9aNEi165p06ZVfpZSGcpjcXFe0XPgupQln+uRRoZ7GzZsmCtfVlYm1En9KqPXhueXJ7+YKZRApnvHr79P8Ir5QUX13nlu+hyxuGPGjHFOt6jeT1jbHSog4zDhoaMsfp19gkceMsSNoqEBtbT6LLGk0Wh5tFoZGiDjiKFbxQPn4ceqER55yMTmWdo0UOwaCA2QGWMxNkr2QFKRSVZH7vLtSqaB3GggNEDmdpmZxDQ8utCMrSDi8MhDxsg0YBo4UgOhATLfGfk+ybxaJhTMnj3btZbuNJ9IyEPGMe2PacA0UEUDoQGyfjPkeyPdbL57MmGB1jL7iFBliBuZBkwD32kgNED+rkkWi4YGrJVh0oABOUxPw9piGkhTA6ECMlP8dCrg3LlzhQkEOLvgkZfmPVox00DBayA0QGZczHQ8nFvxiDxkCv6J2A2aBtLQQGiAzIQQ3fkBKxxL5CGTxj1aEdNANTUQPfHQABlri9XFGrMeFU81cYg0W9QgEz0VW4tNA9nXQGiArLeK1WU9KuBVHqCGR57yLDQNmAa+00DogKxbv+Ds0mbq1i+ap3wLTQOmgcMaCB2QWZ\/6yCOPCDO7dJzM7hbwyDvcbPtrGjAN+BpIHch+qSzHGQvreJkxMnF4Wb6sVW8aiKwGQgNkxr\/Ms06myVRkktVh+aaBQtNAaICMxWWVE91pVj3FKhoeecjE5lnaNFDsGggNkHkQ7B5BV5o4oPUJHnnIEDcyDZgGvtNAqICszdLdQACuEjzNz3hoFZoGIq6BUAI54jq15psGcq4BA3LOVW4XNA1kXgNFAWR23tTxNk6zzKvRajQN5FcDoQQyn6JYJAH45s2bJ7fffrs71iQdVVHXlClT3AmFnCn8wgsvCLx06opkGWt0UWggdEBmX64hQ4YIiySYY81srl69ekm6e3Zx3pCebsCJgXXq1HEHhRXF07WbLBoNhA7Iui8XINanwD5exDWPeHXIP76Tcv7JhTt37pQ1a9YkJDYDhHw50pDPC4ojB\/n5pCGfFxRHDvLzSUM+LyiOHOTnk4Z8XlAcOUjziSspLyiMJ6c8wqByykcG0jQhaSXSiSienPIIE5UlDxmIuBJpJeUFhSrHe8a7ly0KHZB1YUQuFk1wcuHDDz8sLMpIRJxSCPkypCGfFxRHDvLzSUM+LyiOHOTnk4Z8XlAcOcjPJw35vKA4cpDmE1dSXlAYT055hEHllI8MpGlC0kqkE1E8OeURMux65plnhFDreeyxx+TJJ590PGQgzSMkrUQ6Eakc7xnvW9EAma70I98umgDMbPdztIsm2CbIt+a6OyeKZQyOslk2GUQcFgb5+aShu+66S2688UYZOnSo+Pl+HDkoGc\/P9+OUhZLx\/Hw\/TlnIbytpyJcLiiMHaT5xJeUFhfHklEcYVO6GG24QemJPPfWUO9DNl6Oco1mzqui893\/8TL45r5f8pc51jv6t562uLLJB5a+99lqHLQ6YUxn8M2eeeabwXlAW0jxC0kqkfb2S9gm5wYMHC+8Z75u7WBb+hM4ic49M12ShhE4GIQ6PvESERxoHGcTDwKnFy0CZiy++WDjJkDqVB3\/\/\/v1y0kknSatWrQKJPcMgX4Y0dP7550uXLl2kTZs2aZX36wyKcx3IzycN+bygOHKQ31bSUFAZn48cpDziSsoLCuPJKY8wqBzPD5C1b99eYuVIK2n5Wg1Pl1\/\/9V\/y18\/qSY+LW8pdV50jI17dJ+trlCUtz3vwve99r\/L5NW7cWDAoTZo0cWW5ll6HkLQS6fMTvAPI8e7xnkkW\/4UGyICOhwcI4xF5yATpgpMo8EgzJgGsHTp0EJxmaolLSkqkpKRETjjhhKAqjB8xDWz6dL\/MXvqRdJ+6XM6\/9w25tGV9+dsvLpGpfc6WkT9u4eKDZq8WKGK3Vu3mHlPtElkqgMXF8gJCuiY4u4hDpPFiIxN0eawt5\/6qTHl5uRNdtWqVCwE4HmwcX4SOmeU\/vGgQLxv04F\/Wu5eKF8voMMCC9DDsj+tl7Cs7ZfCcf1bqDMAqNRj6mgMvOm3eoKY8N7CdA6\/\/SOEDbJ4BQCf08wspHhogq1KxumzrA3iVB6jhkae8ZOGCBQukc+fOzgIDXnWiUc73WpPeuHGj0PVJhyoqKuT999+XXbt2Vdbxwcd73MvHywPxsk544Z\/y2ze2yKI1O+UfWyvyQqs375SV67fn5drVvWfaun7HXkGXWpbnc+qJItCk68oEenN4O5l8fQtp37Rmpf6RU0J2zk\/Pluvb1pdrH1suC9\/fWUXuq6++4hWowjt48KB8\/fXX8uWXX1bha52xYbx3wJfRa7gLpfqnmnKhA7ICDkeX3gteQeKaRzwRMVYmP9WFFgBx27Ztkg599NFHsmPHDtm+fbu8s2az6+a1f2CpA+zYK0+R6T1L5Z27ymT+zU0rCV4+6LFrG8qEy2rIf19zsmtXPtqQ6jWD2vqLLvUFuqKZCJTqM7vp3JquXM\/pK2XsvNXuWREOfG6nPLrnCun1u52OT33MOzhw4IB88sknKb0T\/jtA+Vh69dVXeR2zSqEDMk4G9VrrWDme15oxMWuTkcG7yEQSNDV8+HACmTRpkgv5E+S1Jg\/60Y9+JDg40qGysjJpdGZb+d2ab6T7zC1yVpP6rpu37D87yG0\/\/L50a98i7brTaU+iMrQV3wFhIrkw5DVq1EgaNmwopaWlGdMfz4LnsuoTcc9q+tsV8s6H+3kFZOtnXwlpgF23bl3Xk8MBloou0GcivbZu3dpdI5t\/QgdkbpZxro6XGSMTh0eeEmPiFStWCPm4+PkBAMTlh8bGnTp1ct1qwK4eajyHlEFeeVrXscceKzVr1kyLPv5C5IYZq+WPKyscgHG04HRJtz4rd\/g51KpVy4Ep0\/o449Ta0veixhL0762Ne+T3f\/tEatSoIccff3xa70Rsm08\/\/XTJ9r9jsn2B6tbPOBgPNZbWJ3jkBdXHwgi643x37tGjh\/DNbvTo0bJ3715XpKSkJKNeaxwnOFqu\/vVSKW9ZUxirAWB3sZD+Ydy2fv16N+4LaRNz0qxZb29LeJ0\/b62TMD82Mwx6DR2QsbxYYCynEo6vMWPGCHmxStQ0O4cgjywTNPiuO2HCBFm3bp0TyaTXGhDjOMET\/eefdZAHerdzv9zuQiH+g6VgvjlhiJuZ9aYtWVeR0Wugz3zrNXRAjqdhvNYzZsxIumqJrvTy5cvllltuqVJNprzWeFDVCuMFxQrjFeUX2Wi\/s\/SZ1AMOp3379mW8Xj5LVXlBYhLffPOvanmtk91zUXqtY3Tqknz3xWHlEgF\/cHY9+uijcuedd7qZWgFicdmpeK2fX7befb7gE9K060sFLyjeyQ0bNgjOOELSYSbaGJW2JvMEH42e+19QO+57oMxLSw8KPyKpeq2T6bUovdaMgxkP++Njxr2xE0Kwvr7X+u9\/\/7vQfWZ83LFjR2EiSL9+\/WTTpk3Cj4DO8OJh6Vxr4lAir\/XBE06WexdWOLrxkqaC1\/PCVs0qPanZ8K6m4ilNR8ba2tg9N74mdDqzPo8+Ls3ffKK8+0UDSdVrnUyvRem1ZhwcO0Zm7Bv7TRgPtO+1vuiii2TJkiUCiHk6JSUlMnbsWOnWrRtJqY7XGk80XmhmF3WctFxI68whxkM+nXzyycKDql+\/vhsn+3nViedCNkptzZbXWvU8f1A7GfnjMqGbrUT604evkEfK68lZdQ+m7LVOptei9VrzXRjL7BB46A\/xWN4h9hH\/AS6zubDMTZs2leeff75SpqQkvtd6f+tr5ZlVX7mZWMzAYgqgzsba9Ol+90mJh87DrqzMIgWhgZHfzsdmGidEmhv7fp0D0uD4r4lGhvLq7GIGlt+FJo7lBIhYVtIQcTSaaGYXYN+6dav07NlT1KqzppTxNWWpkziOL0J40JfNO8nspdvkd4vekzkvLpJt\/1whN7bYLdO6lsjoCw\/IsTvfd112ysejxYsXC0vUFi5cmFAuXtlcLgS86QAAEABJREFU86LU1mXLlglLGNmeKdd6Wrt2rZtyS5jKtZPplaEd71o2KS9A5psvAOXG6Db7NGjQINjy0EMPuckemqeTPlxmgj9DhgwR6mb8zDgaUcDr\/wjwgOBjtbvunit1XxrpqPbiifLpc+Nk\/n8NlZGD+rv1qPQEEtFNN93k1iKznjWRXBjyotbWmTNnyqhRo1J6DpnU77PPPit8\/Rh7aGiWSr3J9DpixAg3tON9473LBuUcyFhOHdv64174KI2ZWc2aNRNmaQFIJRxgyAQpgbnOOLgAFOAfMGCA2+cL72NQGRTLDwY\/EkazKhfhZ1oXUauPOQu33nqrsKtHptrOe8b7FvQuHi0\/50Cm2zts2LAj2g0fpZ1zzjmyaNEiYQYSgFSCh4wWxNpidQE6PwBsqte8eXPBg4gMnmm6NACZMMhrjXLpuht1dI5C00N29MB7xnuZLco5kPVGsK5YWYAYS3iBmSnj85GljJaP9VqXlZW5HR7wXCND95ku9QUXXCB169Z1q5P4cfj8888ldq418kamgShrIG9AxrrybRjnlFpdplcOHDhQsK5soaJ8wliLHE\/plJ89e7YbIxOyigqAs2cS35ch4lw7XnnjmQaiqoG8ARnrymYBgE+VB6jxErJXEquZlJ9qCEABfCzwdR42fOKp1mdypoFkGghLft6ArF5k3TQAhbB6iaVjfIJibAvPyDRgGkiugbwBGYtL15e5vzoWpjvMtrJ8P3zvvfdcF1nzYsfIyW\/NJEwDxaOBvAE5VsW4+nFyscslixhwTNEVVqLLTNc5tpylTQOmAZG8AZnVSkzeUIcXFpoznu655x659NJLq1hjtcp8buKzkz0404BpoKoGsgHkqlcISOl3XRxcKsJnIcbI1113nZtqqdYYhxg0efJkN8lD5S00DZgGDmsgb0BWZxcOrsNNEXcGE1uQvv322+4oEOUDdjzcpaWlyrLQNGAa8DSQNyDTlcbZhYMLMLPmGMcXU9nYDC\/Wm02bTzrpJOHTFHEj04Bp4DsN5A3INAHnFU4s7UITZ0YXAAfUOjYG7Oy\/NXr0aGHqJWWNTAOmge80kHMgMxGET0kK0tiQPJoHqGMBzlxsf6EFyyBZXIE8hCMMhxh1Ug\/Xgq+rreBTBl6aZMVMA6HUQM6B7FthHFiMfxWwpPFi060GdD75wESTABg54hBe8Pvvv19Y9UR9bBhOPmCeMmWKsK4V4qA3eJQxMg0UigZyDmRVHGDCgQV4lQeosZ4sRwR05E2fPt05vgA4PwLIYlVZ7og8aYgxd+z4mW44mwhwBAiroui2M48bHmWMTAOFooG8ATnIa41iARvAA4gcxgZgAT3gJ5\/udbw50xwTw6onLDlARw55VkHp9UgjQwixtY9PbPeTjNgSV4m9rRevrQj97iDMYTd6M2\/PiQMTeN+yRXkDMhYUpxaOLIAH4eDCa826Ym4YML\/88stunSxriuEFESCn+w2A6VrzA0D3O0gePsp96623xCe2+0lEbAn00B+XCfTgXza4vb7YrP7q\/9sn3f53s\/Qc92zOd7RgPXYGyNrdt6\/Twfjx4+U3v\/mN2\/klU3pllxDeN967bFDegMzN0FWO59T6yU9+4tYPs+aYSSBYaPasRp5y8YgdQlh3zKQS8gE0Pwx0q\/kR0Ako5GHpCVEs2\/s83qOhPDfw\/JTo+VuaidKfb6glv+\/dUP6n\/FiZe1tbGXF9e9nfuoe0HTA9lLttsMk\/U2GZy47jMMyEv+Pmm292+3blup29e\/eWdu3aCVv9pHLtZHpl6SzGgveN9y4blHMgYzkBZ6KbYZeKl156yYnQhWZrIEDtGAF\/sN6AVse\/dJ\/pUrOxAEVYUUUdWGsFO3wIOa6ZDv3wonOk579fJl1anyLswtinQ6k0bXqa60WkU182y3Tp0kWYNccU2GxeJxN1s3acvbDat2+fc13yQ88Wt4Sp3EsyvfLu8Z5lk3IOZKyqfiLCaRV7c\/DoZiMTm5coTb1YbbYRojxddrruOjYuKSnJ6CFuQW25tOXJouPmIBnjmwYyrYGcA5kbwMpiGYkDOp8efPBB2MJML5\/P+Bdr7jK\/\/YNzC\/o2KVhcrDd102UH3GqhcfQQx\/oSaplMh5xggFVm3Nxg6GvCHtnqGMMplunrFVt97DUOoUt+MNGtOieJkxdHJwXPyguQVat4lQEdBNDYPbOkpMRl89kJvpIC02VW8w\/gVctMUbrdhEobN25M+6CwiooKYdnlrl27XB0c6jb5+hbCAW+DLy+V69vWl1fe+0hwjCm4FeCAXMn3nGcz3nXyG3LVlKWSzWtkou5eT66W2+Z+JJz0oTry9YYuATBncS1as1P+sbXC6RjZ376x2T2LZIerBeUfPHiwWoe4xb4DsfUW1SFuWE\/ACnABNZ5DtcjxrLGCMBMhQEz3ULCgw8aOO7BLOOgNmt6zVN65q0zm39xUiI+98hQpb1mzks5rdKw0OP6rpBT7glQ3vXfvHnc4GS9WdctmSz7ovuses1\/q1Tgg6EZ1hd4gdIguVafE4ZEmHDxnrYydt1rSfab4WtihJtVD3ILeAb1+UR7iBjB9UANs9hlmrXJs1xrZVCjIa61lEx3iluywtLKyMmEWGWEyWQ5\/69a+hXCI2NjrzhalaTe2lVSIo2uOhjjL+aWhHeVo6sh02aD7frR3a7m3vFSm9Pp+pZ7QG4QO0WU8fZPHQXurPhHpPnOLvLZZ3MFt8WSDeHz9YIOLVA9x49knegfYMEPftWyFee1aB92UP2caq4x7n6mVANwv88QTT7iN7JXnl1Mrrh5qPIeMoflhUJ6WY7WVHu5lYc1QHEZXq1YtAUzpPI8zTq3tfqw4PXP4vA2uez5l0TaBUqnvuOOOkxo1aqR8iFuyOrN8iJvwL69AxsICOMDKt80f\/OAH0qJFC7nzzjuFiSCADuJbHhNIaLASkz2YS63poLnWml9SUpIVrzVdTrYlItRrhTWkjVFpayZ0yOfAv\/3iEulz6JMgzjH8FDrOxjGWiWtQRxj0mjcgA7whQ4YIc6iZgslaYwjF4Hzq2LFjle1+ADzAJx9LXF5e7nYRIQ0B9KC51uQz7sZbjeOLEF4miF9jfnwIM1FfNuugjVFpa6b0wCmaAJohAaB+bmA7B2xAjWMsE4AOg17zBmSdaQWIeWj16tUTus+XXHKJsEMIltgnHGHatcbbzScsyvnEpyg80lh4gI4c+YA3yGvdtWtXZ\/3pvscjfm0hP4+0ks+PF48npzzCeGV8HjJQPF4s35fx4\/Hk4EG+XLw4MpCfR1rJ58eLB8kF8bUOemEvvvii7Nu3z3mglU+oZQlJJyPkoFfmzZJ\/vD5XBl\/e2H1V4IsCgL5swmvi18FJkJs3b3bvI+UgP5+0ks+PF0cO5yLvGe9itihvQFZgsTuI3px2lTVP+amEWGusNgDmByCVudZaL4pG4fFIPY9+nvII9+7dK7t373Yvmy+jcWQgTROSViKdiOLJKY8wUVnNUzm\/rfAglQkKkYH8fNJKPj9ePEguiK910FY8x0y9RVb5hKSVSCejWFnS+lVh5Jnrhed\/z6rGlc9QPz8RIgv51yCtBJ+2Br0DyBW015quMDOvmIEFmJkAwtxoeOQpyOiCM3EdK8tsLxxamueHPHC8jYcdWSIAmvr4lJDIa81YnF\/L\/v37SzxSz6afpzzCfv36yVVXXeUm2\/syGkcG0jQhaSXSiSienPIIE5XVPJXz2woPUpmgEBnIzyet5PPjxYPkgvhah7a1tLTUeZ2VT6hlCUknI+QglSOuNPinfeW1kZdJt\/ZlsuO0K9w7wLRQ5jRcffXV7trIallC0kqkta28p6R9Qg6vNe+Z\/75mOp43i8yNAGDc9oxfURzdmdjxK6Cmm4WVZdYWnmfWLAPsP\/zhD8LyRsDOXOuKigphfi55OMzorqcy1\/povNbMyeVB1a9fPxTeXsZrQRSlth6N1zro\/hPxp\/Y5W5Zt2S9MRKmu1zqZXgvaaw34li9fLqxwUUCzmcCcOXOEPIAej+hC644fTKxH5umnnxbGz2eddZZs2LABljuBkeWQ2k0vKcmO19pdzP4UhAbmD2rn7uPRxTtcGKU\/ebPIOLs44hRLioOKlSbE4ZEXpEQAO3\/+fDev+t5773Wb8VEWgOMYwUJjvXGYsYhCLTxWnziOL0K\/fpaYkZ8OLV68WObNmycLFy7M26L1VNsdpbbicOKTJD\/uqd5fJuS61N8uL26rIyt3\/Et4L1OpM5leGdr571s24nkDMpaSdcaAas2aNdKpUye3Bhkeeclulu61joc7HvpUpfJ80qJr7Y+nAa9fJw8IeQ6fZqIIFp7xTTpEV57zqhgXpVO++mUOL35Pp1zU2jpz5kwZNWqU8z+kc7\/plJk6ZqAcv2mJzP3gOBk7dmxK106m1xEjRgjvGe8b7102KG9AZuzLtEucXK1atRKs8d133+0mg5CX7Gb5\/ITl7dy5s1sptWHDBmGvLwAFn034WJyO5zOoLhTLjiSMwY1mhXIzhHw8l3nDr5Sn+pYJmzBk6vq8Z7xvQe\/i0fLzBmQarmDk+6\/fZSZPifEyv6yxVlbz6VbTdWEyCWNifhDIUz5AJt\/vrpOHDIRysehGHXO+gL+YdM57xvuWLcorkJlmCUB94lsw4129Yawzv4pYWbzWABVgq8yCBQvcooU2bdoIln3JkiWuKN1nutR4rfksxecppicyBtcuuRO0P6aBAtBA3oAMEBkb48wI2vY2nn6x3HSf+TXnB2Dr1q1uLIMs9fBdGj4h36RZmcKeSXi4IeLUgbxR2DSQuD30znimDMdWr16dWLjIcvMGZPSMYwsLS1cXy8p0TbzOgJz8INIuOVYaa43VRhaALlq0SOATkobvyxOHZxQ9DTCFl03xHnjgAUn2mTJ6d3d0Lc4bkPEi6yIHwMzMF6ws49mjuyUrHTUN8MPdvXt38WftsTCGnhXEFwruad26de4QvwYNGgg+Ed\/vQX4xU96AjBVNZ9vbYn5YhXjvgJcpsps2baq8PXhYX77hsuUTixHoVnN2NgCuFLRIpQbyBmRagAVmyiVxurw4szQNz6iwNYAl5lvxtGnThOm0erc4LBl20WvDMYmDEkdlw4YNZceOHcKXCIiptVom\/GF2W5hXIPvdJ7pQUKzXOru3b7XnUwP4MPj0GM\/KMuyi10b7WPjCVwfmDHACxLhx44RPjXSxyTcSyRuQ+TXGs0zXCeeUku+ksgdkGvA1wKYITLudMGGC9OvXz88q+njegIzm+c5L14m4kWnA1wCfFRkXw2MeAA5R4kbxNZA3INOtoqvEyqf4TTNusWqAefeMi\/FKMxef8TLWuFj1kcp95xzIdKkZBzMenjhxorArCHEl8pBJpfEmU5gawOF5zTXXuCmjHAHEwgkdLxfmHR\/9XeUcyFhixsE6Jo4NyUPm6G\/NaoiKBgAuE4EItc3st8a7YV8yVCOJw5wDmebwnZBlhlhh5k3rWIg8I0YFj3IAAARhSURBVNOAaaD6Gsg5kAEtyws5WpVfXD4zsMNH9ZtuJUwDpgHVQM6BjAODi6u3mq1+2GkQXkGR3YxpIIcayDmQ490bh2BhqePlGc80YBpIroFQADl5M03CNGAaSKSBvACZFU7Ms8bZxRphVj7R1SZtn58SPS7LMw3E10DOgcynJT4x4eiKR+QhE7+5xg2FBqwRodNAzoEcOg1Yg0wDBaABA3IBPES7BdOAAdneAdNAAWjAgByBh8inOWbA4Qz0iV1II9D8hE1klt+vfvUrd0wQ98gadS2g961b\/ShfQ\/LRQUbn5mvlEQsNyBF6YOwS6jsIWZQfoeYf0VSAyA4hgwYNOiIvFQYLKZhQNHXq1FTEC1rGgBzxx4sFw5Lx2Y44Fk7nscNTa4VVw5qTd\/vtt4taMmTIQw2Uh69x5CHlIccmeRB8rgsYkUcGHoQcpPnIEIeHrBJ7crGDaipfKfz6uQb1Ua8utOC+td5iDA3IEXrq\/pJPAKgg5bs828OynTDH7jB3Hcvdp08ft0wUufHjxwu7sbAfFmt9E902gGM1EmuBIRb5w6MMm+Tdd9997sA6rgsYyeMsauLsU85ZWpwLjDxTctlvi2syVwCeElsgA2RNE\/r3iDx1wqf3wT1B3Cd7m2ORyeNa3BfxYiUDcoSevN+19r+3s9MKG9WxrxVAY5INVgtQADBOC+Q2AQYvPxs6kA4iTukAQMhDxAEd8pwDzW4dXI\/rwkOec66xrFhITsvk5A8WxPBDQLs4BYR85CGsKeCmLtJK\/j1SlolDmkeIZeYQezZrJA3xY0AbiBcrpQTkYlVOFO+bTekAHpYLAvBslVPde8HqUV4Ji1jdOgAcPwAQ8eqWj5Wn6w9ofRDHyhRr2oBcQE8e68aOk7p9Ei8+Y0lefm4TC4clpNtMWq0q1sznI8+0Wcad8KmDuigTj5DH8tOFpwwWn1CtKUcDYdn9svQM2MIHa+3zg+J6fTYciJWh\/bQhll9MaQNyAT1tuq7sMvn4448LXWt2KeWsJM6\/GjNmjDt+lv2wABC3DZgAHV1wn4\/F41hauugAkC5yPABRB4Q843GASxmATxeb+gFYbLeaMhBWGmtNPBHx48S90E7uC1IfAT80y5cvF9qfqI5CzzMgR+AJAwjOuIoHJnjkIcOtACC2x6FLTLcacMMHbPDI4\/wkeBDllc9G8dqFVj55yqMOrZPrcV14fj3IUxYeIMP6A1jSsQTw4WHJtT4tC195dPO5LnUrkebecKSVlpYK902ZYqWiB3KxPvhs3zfgZAM9rqOAJe4TQAXkWFqfn2qcHwp6IBw9lGqZQpUzIBfqk01wX1g9tbIJxI4qC2uJ1cRqA9igyrDo6baFeulFFLs1RrcGZLRgZBqIuAYMyBF\/gNZ80wAaMCCjhQIlu63i0YABuXietd1pAWvAgFzAD9durXg0YEAunmdtd1rAGjAgF\/DDLeRbs3urqgEDclV9WMo0EEkNGJAj+dis0aaBqhr4fwAAAP\/\/yK8UTgAAAAZJREFUAwDGiNzx7VXYFQAAAABJRU5ErkJggg==","height":146,"width":242}}
%---
%[output:3c743fad]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPIAAACSCAYAAAB\/nJf+AAAQAElEQVR4AezdC4xexZUn8FKW3Tg8ApkFwssJTohwRIBF4GDECpIl0VjrsGQiIS0JeYiVIN6wwloTvKwScMIKZniMjNYsCQqWiCysDCsWxiJiBNkR0Vg8vBGxE2sZhYRgjM1LhMQ8HA1Kxr9rTqfu7ft9\/XX7u+2vu8vy6bq36tSpuufWv86pulX1veuP5V\/RQNHAjNfAu1L5VzRQNDDjNVCAPONfYXmAooGUCpBLKygamAUamM1AngWvpzxC0cBgGihAHkxPhatoYKQ1MOeA\/PDDD6cPfehDSTjsN0NmLvv1119PV155ZXrppZeGVhSZn\/\/85xNyTbByv\/Od77jslJSRl9uvsGuvvTb97Gc\/68cyljYZ3rFME1zQjbp6H6eeeurAdYl88rqeoJjOkrWZc889d+B2WgE5MnnoiYhw\/J09QceCP\/WpT6Vf\/epXSTjsosjMZa9atSpt2rRp2MXU5AHLihUranFd3Hjn69evT+ecc046+OCD+xYB8Pfdd19fnkicDG\/kGSR85pln0tatW9PnPve5tHnz5nTyyScPkm1keI488si0aNGitHbt2jRIh1IBWe2PPfbY9Nhjj1WNXGNsI+n48PcjjUsvGJ0Ci4FfhfR0559\/fkLS8eGXjvCKR3mnEfG33HJLkke6RiBPG5EZfHjlxyeM+2Z98MsX8fgQqyqvNDxRbi5Len6P5957703PPfdc+vSnP12zCCGfLlzLq4z8eaMs5SOy8b1DVfDKK6+kyy+\/PO3atSvdeOONlfWXoGx5kPqSJR4pJ+Ivu+yySpeRri7qJB2RIw\/yLK+99lo6++yz3VZehvriQ8GrnuqiThdccEFlUXQCOa86ENLGK54sMpH6qJf4JjXlyodHfOhFvdtkkCleGSjqJD9S\/4suuqjy3tSdTPEIrzwoT\/M84iJdmPN7F02dk0uGfIgMedCSJUsqTMKd+35UARn677777iTsxyx9Ij6N4uKLL07Lli2rOoWrrroqsRjiQ\/a2bdvS9ddfX1XysMMOSzfccEPV6+DBK8+WLVuSTmP58uVVWuTduXNn1cPqaVkIioi0CMV5kVEHvNddd13V+IInD6M+0XOzpNLV4Y477kgaQ65gaRPRV7\/61coazJ8\/Pz300EM1i8CisWzPP\/98evPNN6t6sdx6YDqmh4l0qPzDDz88rVmzJh1yyCGJzm6++eaqw3jggQcq3WoAuX49g2fBu3HjxqoDICfIc6uTfJ4bIOWR\/vTTT6eTTjopLViwwG3VcbjAS97tt99elc0rca9O999\/f+X5kAMUjIM0dSC3jVc8fuWTrT7qpaycvOMLL7ywslrk4pdPfjoMvXj32iyd5\/lDZq93HG3CM+jA1Fl+8nfs2JHkizTlSstJnbwP\/PJ67qbOdSbat3ZOHh7t3\/sn65RTTknaD92770cVkClFr4Bc98swUZrK4omemyI1prvuukt0RdEgKNwL5gJxhfK8FJ839irjnj8nnHDCnr8pCSn4xRdfrO7zP5QiLepAoY888kjPjirqQ4bnByplq4PG5qUIpQ+L1E0d1RWx3Hpg8nM9uG\/Tofg24kJu2LCh6nwWL15ceQTB9+CDD1agV7Zn84yRFs8dnUneiDQ4jTd4I1RnYFK\/6AQjLQ\/pH4\/21dboc1511HiVr32oj7LVIedr6syzokFc0XhWz08P3m3zHWuz73\/\/+xNyHWDCq2PQEfA4WO68Xq7jPbr2PDq1Np1r89p+1AOPPM02HWVL60UVkClML65noAwmnhsQPUOvzG3xCvXgFNCW3i9OXoqhIHXw0jX2\/MEAuJ8MaeQIp0LKUuZU8k4mD8umA\/GiUTReMtR\/qjpkAejuxz\/+cWWVvU8yg3rJjedmPeSXD1DVhdfAMoYM4Ze\/\/OWqU8CDF0gBRFpOAMiFxfPNb34zsZx5en6NF2hDpnqoj7LVIedVLwCZSjuLZ83lNa9h4cADD2xGJ+67ekngMXhvrvtRP51r79o5mdq9e8+Wy6MTusnjmtcVkEXqmfQ0eibELSVYAcgD4JuIAA0QKGsi3ma6vF4Ol0UdUL+evpk\/7smhkKnUQcOg+KYyQ\/awQvrWE3O7n3jiicpF1KGSr\/5T1WF0CqtXr05tDbGX3HhulpPeg1hTcjRsdQti+b0blhFIgU+DjPQIw+pwG1mziG8L6eSYY46p3EkgiTq0eVN0NN3vGJh0kJ6XRW57hra4fjrX3ukmnlVoWJbLOeaYYyacYBwDcp7RNWGEIkr1AG09Lt6cwj0I91CP6kH04MFHHiLPOJdlYqHyvJSmJ0euI+8gIbdMT6lR49cJDephABN3jnutfrwSecmIxh4gD\/nKmArF827fvj3l7ljE99Nhr\/I0cPrWidExCl5laPzkeja6j7Tmc4dlFwbAwjJ6H94Lkv\/qq6+urLOy3eeU60w+rm+e3rxWR52CDkIdWfqYNMp5m+\/Yc6JLLrlkwkbffNb8HedlNK+betC21bXJl997nl461+a1fdiiG21MW1OfXEabXvN0161ADgWyxMisq5dFATL1Iz31unXrkskPeYXuxUe+xXvGbvfcc08SimM9KAmPWWk9uxel4UQavkFJPU12sHbqQB655A8iY9Wez0YskPrxSjy\/jo1cY3ovkNyJZHmJXjQ5ANHkjxep0\/G8ka6edEZ3yhG6Fx88EYYMzwhYyuRRqLeJk+OOOy7RI9eURWRx8XqmhQsXhpgqzJ\/70ksvrSbQ5JGoMUUH4V15L+Sqt7I0SBN0eKMjEk9XoTO8Oi1WKDrDnFdDVh4LpXx68x7Ui9ycvAttSIdLR\/i57fLnfL2uySRbGepJH95xL\/6IZ5DoQT7vhX77ub7q00vnTT16L3k71ZlpP3Qf5fcKKyC3ARcQWGPEhWprRE2hehLuonh58rx6HL3lo48+mpBBvh606TZ5cPlQnhbxQvIpXRm96iVeOjko8gnjniINJ5BrcpFrcfgQ91I8Uq44JF5IpjRh230ehy8oysmfM9Ly+nsO99Iij\/q5Rq6VITz++OMTee7l09u71\/B1JoClwUsDKKBnNXPZ8iLPKh5pjHhZevfkkYsPKVtdpKkr+eLJQK6R+kgT18YrThpelMuVllOzDvQf6VEH7yji8lBdyVYGCr6Il+Y6yoj0kCuP5\/A8wat88cIoayKdh3z5UJ5XZ6fDQCGvV1gB2RhI70YQUkEV7pWpLR5QfUb67W9\/25ZcfWbhYpB71llnJW6ph2hlLpGdaECDQKwXKwbUJqAGeQ94WFausXfdSQVnoVD6Ru\/ovPqaMIjOGVdDH8ZPhzKRaiogc7t8z5K5Xwbp3Ddhk4\/r95GPfCR94AMfaCZV93ryAw44IH3ve99L0YNVCeXPtGlAg6B7nXVQbgEmqshX93wbl5+ciXhL+l4N0BWdhb6Fg+hcx8njGYRXSRWQXRjv6Dn01L1IOj78OXGpn3zyyfSVr3wlj65dm2DhTpNBvvFQjaHcFA0UDUxZAxWQA\/16i4lIL4E\/SuRmGU9bSXXQQQdF9LhQbx6yjY25DcYPOaOJEGmFHqu+ARc9zB49aNt5Wx\/2dQXkfRHqO6EGZ+aPtWV1zV6y0r3kGpObMTSYDx4P+vWvf73a1cN9L7R3h1PRw+zQg7atjUd7H3a4z0A2eWVyjLUFaJ8hjJfF55XlSiNxwO+h9n56EJOS+8cffzzddNNN1RjauKLQ3Z3owqSkzyjf\/\/73O5Ff3lv9vV1xxRVJ29bG97b24f\/dZyD3qxLgIjystKl642PWm6Vpgh0fS82yF1pcfWfvQg\/0\/6UvfSmdccYZnZXRRb1nqswzzzxT0+6UhgpkY2cL9gOgxsXIEzRn7yJeWqGigaKBfdPAUIG8b1UpuYsGigamqoFxQDYLze3lAguNZx0C0G\/yaqqFz\/585QmLBqZHAzUgA7EVKHZb2IFkCd8RRxyRli5dOrb5f3qqVUopGigamIwGakC2wktmi9aFQdbZuo5014WKBooGRkcDNSCbrGKNLdd84403qlq+\/PLLySkQ4qVXkeVP0UDRwEhpoAZkNbPLw0Jt42PbAM8777xksbx46YWKBvZqoPwdJQ2MA7LKWahtgUdQ+VREK4WKBkZXA61AHt3qlpoVDRQNtGmgBmTbEx2t4tNTG7UdQ9ImtMQVDRQNTK8GakA2mWU8bJY63GqhWWxxjiGxTtdnqumtZimtaGA6NTDzyqoBmUW2Hhpw80cBYucSHXXUUVV0+QxVqaH8KRoYSAPP\/Ivj02ufvTP9+795K63f9MJAeSbLVAOy7YUEOIgsVnIJ3YuPbYfBJ65Q0UDRwOAamP++eYMzT4KzBuTY2MCFtkPGOFnofvXq1enOO+9MTtPEN4kyCmvRQNFAxxqoATnKan5+cm\/8nO9sCt4SFg0UDex\/DbQCubVaJbJooGhgShrY9upbU8o3mUzjgOwgAC51k3yWMhk2GeG9eP1yAOqVXuKLBooGJqeBGpAB1Y+52flkptpB5j4\/ufZZins9OfHjuR245zzl8Sklpmhg9mvgA382DZNd1Gjrol8e8DMVDpEX53OUz1KA7n6qJL8Dzr\/4xS9OVUTJVzRQNNCigZpFjs9KLKaD8fxKoM9P7tvOs26R1zfKb9vYkHH00Uf35Hv22WfT7t27p5fmYHm\/\/\/3v01tvvVX0PA3v\/h+efm2svU+LRfZZyWcm1pdVtgPK5ycAHORnLsZq23LBpRZtBlzYi5566qm0c+fOQh3q4IUXXki2p\/r1j6Lr7tsaw6S9dwVismsWWYRxsONMhXY9GSOjiQAobz\/iprPsJtF0DK7bJrxsm2SxCx2dutKBTtrJL1bqdVVGkbv3\/f3Tu9+XfvL87goaXS0GIbwGZGNYVlgoMch9W3ykDxLaz6xDQMbcJtDENfP6fah58+alQt3p4D3veU9697vfXXQ8De0sQKyd\/9sTDhN0QhWQAdXnJecGO2ReyHIGuVd6jKFdF5ohGijV3G8a2Pbq7vS19f+\/Kv9db76Szj30xeq6iz8VkLnRftMpQCxkOXPibhtDD6MSXPY2azwM2UVG0cAoaACI\/8P\/enKsKv9q28ax6y4uKiCHYIAGWGHElbBooGhgcA0A8F\/93TPp3\/yPR5NrOT\/0h2fTvKf+1mVnVAE5XOtwpdtCrje+zmpSBBcNzFANAKztidxoAP6rv\/v12JOs\/PPj05r\/uHDsvquLCsgsMNc6d6Wb19LxdVWRIrdoYNIa2I8ZmuAFYmCOKvnUdNtFH00r\/3xBRHUaVkDutIQivGhghmsAaC3qAFSA\/bP\/+veV6+xaXP54AeCffuOsdNGivQdx5OldXY8DsmN8fGrK3Wv34ruqRJFbNDAKGgBYBJxAirjKyMSVe2nNugLvyj0u9N\/+59PSdAM46lIDMrDGT8bkrrXD6cVLj4wlLBqYiRoAVJRb2PNve7KysACLArBAi7f5nIDL2nKdX\/3rT1bgXbnHhe7yO3GzDs37GpDjLC4LNnLGuI\/0PK1cFw2MYOBfHgAAEABJREFUmgaAL4BqBhkwm2DNLezGX742NsOcPwvAogAtixvABWLxOf9QrycprAZkk1msrzO6bJYgS+hevHRxhYoG9pcGcpCymECKAqgxfg2gmkHG1wusngNYEWAiIAVabjJyL35\/Wlz17Ec1IGO0UGPZsmXJZgnjZKF78dILFQ10oQEARW2WtBdIARhIUT+gqi+gIoBEwJmDNQArXvoog9bzNGkckDFYeZWPkd2LL1Q0MBkNACYKcAJc7uo2AWp82mZJJwKpOgHp2R8+rJopBkSARMAa7nAbWOWTf6ZTK5Bn+kOV+nenAcBEUwEnC5q7uoMCFNh6gRQ4A6gbvnZaAl4EzGimWdapvrkakK3csoJrVD83TfUhS77eGgBK1AZMwMutZow\/c8uJZ7LgVBvgRDlAV+75hAOErCgC0qBeICWDvLlONSCbzHJmF6WccsopyRgZte0bxlNotDQAkAgoEVcWhTsLdIC5+OYn0+n\/89fpmP\/+6Nhnl3Bp8QQw5R3EaoYWgAoBJ2IRA5wBUMAMC+o6B+jKPZ9w5GFFEVkhu4T9NVADMlY7nGycyMfIS5YsSSw1i42nUPcaAMigfqAETBayaS2HAUxPCUwogAloK9+xnP3AGQBd+Q445SvgpNFuaByQLfrgWrPEQStWrEhr1qxJLHZbNRzjE7zyktHkEyct+OZSx5ADcrKgBNJ+oJyMxfROgPKY9x6QTj923tjE0MoGMMOtnchyFnDS6GhQDcgs7tKlS5OD9vI9yZs3b04nn3xya43lufXWW5MjdLds2VLxrFu3rgrzPxaT7Nq1q+Jj7WfiJowApHBQQA5qKQESkZ3rrd81UCLWEgEWagKTCwuUyPX\/+2+L0h2fOyrd8hcLqsmhlQ2rWSxnP62PZloNyCwugCHXg1QZX\/OnZByl28zroDdxzosSjgoBDsqBaZyIuK2IVcwB6X6YVpIuABIBJAJItHJAaxmuLHd3ZQOY5Cqj0OzVQA3I+\/KY3GsTZGTE0UCugzZu3Ji2bt2apHGv\/aJFpHUd5kANgAJjDs4cmCZ5EAuJ5B+0jkATNAggWciwlK4BEgEkaoKyWMtB38Tc4hsakJ2yyWU+55xzUtsGC4tKpCNu+\/r16xPwN9X97BDOtf7VS69Xv0MLtAHWAGoAdBBwGksi40n0FycfltAV5x6Vbv7s8RX9zSULE3rsytPSjuvPSsKge\/7TRyv3lQt7xblHV3nlP+O4eQkdeWBKyHGp003lXOvpOzv97bffbjbzod8PDchRM261MbYxccQ1Q4f4HXvssSl+bzlPn+q51j\/5x+fSHf\/3F4krfMZfbqoOPQPaXHZcAycCzvM\/enC69OOHpVWfOrwaNxo7bvjycekn\/+X4JETi0Dc+cVhCXzplXvrk\/FTRhw\/andC\/\/P1vZsxZ3NN9rvVcPzv7Rz\/6UTS9zsIakE1cmVkW5iW6b4vH00xzfvWiRYvGzXBzpZE8zzzzTNq+fXvyaxbuc5rKudZ\/\/1xKl\/2fF9Kqh18ZO0M4ZHJzjTW5qWZjWU2TPYgL+90vnpxWffaj6dJ\/95H0mTMWVHT6ifM7O1N6FM57Nk9RzrU+etre8cKFC6M5dhZWQAZGn4OMX7m9QuPYIPdqwJIKczLZ5Wdg8ODfsWNHWrVqVcUCuMjNxRdfnPyCBR4bMXQMbTPhkznX+qU3U7r1kZ3pyvt+nXb8bq\/7ArjGpoAbY0\/XwGx8Wc7LnpfKudbzpvVM7w9+8IMg0ClVQAZGM9UBYqGxbE4WiVgs0labGB\/jz\/mMi5E88krDgyJe2lTIGNe41yok+QF45Z4ZXlaXpQVc8YWKBuaCBiogx4MCNLAJI24UwwCxUP2AGIBX7vns4lpcoaKB6dDAqJRRAZlrzdX9+c9\/nrjY3N8mice3vysOvCyxUF1YXp9tCoBpo9Bc1UAFZBaYJf7Yxz6WuNhc3yaJx7c\/FQW8PikJ1QOIjX9dFyoamMsaqIA8UxSw8ZevJaS+LHABMU0UKhpIqQZkrjMXuulWuxcvfX8pjRVmjZUPxMbErgsVDRQNNIDMdeZCN91qP4E6iR8670Sv9tSGYC41MMd9CYsG5roGaha5lzIch7t27dq0vyzytld3V0su1Q+AzU67LlQ0UDSwVwMDAdn2RMsu92aZ\/r+5NS4gnn79lxJHXwM1ILO4xsLGxDnZBLG\/XOumNeZWj75aSw2LBqZXAzUg9xojGzNbvTW9VdtbWsxSuxsBEKtGoaKBkdNADchq55clbEUUurdWmpVmrd1PN939xM6qyDI2rtRQ\/hQNtGqgBmTnat1www3pmmuuGTvax5roiy66KC1fvjxJb5XSUSS3Oizy\/PfN66iUIrZoYOZroAbk2EMcJ33E4\/n85DrSXU8Hrd+01xorq0xy0UKhooF2DdSAbIzMrfajbeFac6kvvPDCtD9+xO0fnn6tqjW32hbE6qb86UoDRe4M1kANyJ6DK+0UTPuHzVzbZ8y1nu4fcStutbdRqGhgMA2MA7JsNvw7AtdsNQJu8ftCxtd2WOkcUNt5Xbl8QI77Yo1DEyUsGmjXQA3I3GhgE7azTz2Wleee6xjuuOOOdN111\/VdKbbxl78ZK+zsD79v7LpcFA0UDYzXQA3IxsjAdu+9947n3McYVj13zx2+13Z0UBRTxsehiRIOQQOzXkQNyCzxpk2b0o033jj2A27cYDSMb8nhXlsp5pwvx\/+0aZhbXT47tWmmxBUNtGugBmQWuW33E3dYvPR2MYPFAq4DDJwJxrVuGyc719q51CFx8fEHp+k+83kulFfOtS7nWgfGphxyqbnWvc61fvCnz43JPvHQt2fMedEz5fzmcq71zmltU9N2rjWX2iRXl2d2WeqJIHSic603v\/ino22dNT0KZ0HPpjqUc62n70xr7abjc61Bau8JIVxmLm+XZ3b5Lp2fa50vA61q8s4f51o\/\/uzr1Z1lmeUc6uGfwVzOtR6+Tvu102k717pCzTt\/rOiyuksoihUdxkRXjI+Nt1Gv3VTP\/Wa3Yisq348rNZQ\/RQMTaqA22WVWeX9vmjBjHbUu349DEyUsGuivgRqQY1PE\/tw0Ed+P+1e7pBYNFA3kGqgB2ViZW70\/N03E9+N92yiRP2K5LhqY\/RqoAdnjWoFlOaXJKQtBpnvTxB8O\/NeqUahooGhgEhoYB2R5u9g0Qe5E9IcDD08InxlrYaGigaKBiTXQCuSJs3XDkVvjMmPdjY6L1NmpgZECcq7iMmOdayO\/LtdFA+M1MFJAfvvwE8fXsMQUDRQNTKiBEQPywrEKF9d6TBXlomhgQg2MA7IdSWarmzSM1V0T1uYdBp+e3rksQdFA0cAAGqgB2eYJ2wud4GEZZU7D2MY4UX3CtS4z1hNparaml+eaqgZqQCbE9sLmyi7xXVN8dlJOsci0UKhoYHAN1IAcK7ucEDK4iOFwFiAPR49FytzUQA3IXOv169cnZ3btzzFy+fQ0Nxtjeeqpa6AGZBbZWDgfG8e1eOlTL6p\/zvzUzP6cJbVoYCZqoNs614A81aLsWQ4L3mt22xZJp5D04su3L5Yx8lTfRMk3VzUwDsg54ADPsTznn39+ioMGmooS\/8ADDyQH6rHeixYtav3BN1skd+3ale6\/\/\/6Er2nhA8hAjJrllPuigaKB3hqoARmIHVXrbGuAO+SQQ9IRRxyRli5dmhw4IL0pygaLDRs2pHC7lyxZ0mSp7l988cUqdF5UdVH+FA0UDQxNAzUgs5okX3XVVYIxmsyvMT744IPJnmZH+4wJ2HOxcePGtHXr1mRbJPeaO74neux\/7EM++pADyvG3u7s\/qrUch9u9juNY5bff3nuY5Fhj7+CiBmRWlTVevnx5euONN6riXn755TTorzEGOO1prjJnf8RxqRE33Oz4ww8\/XHGEW+3myAPTtB5VOlOOsB1mPctxuLP0OFwACvKzLn4Fwvj4oYceSuedd14a5NcYr7zyykqE\/NVFnz\/9zrU+8A9vJIfUF3q2Mz1s3749\/fCHP0yPP\/54Z2WU9\/en99f0TvtAY8pJNYscUpxwyXIGsaaR1hYCsbFxPz7WGslvAk1jOvvss93WaMP6O5OOpNAlnenBPMgPfvCD9K1vfauzMsr7+9P7u+2229KZZ56ZjjvuuFpbH+ZNK5AnU4BNFhaQaBzGvog1NzEGuIg8Rwfl51rjMVEmzSz1q3\/9yfTTb5yV\/vdffi05Y7vQ3UUPd88eHdx0003TC2QABDKADHIvHuia1LTerDgQcidYaCSPe\/HSUcRLCwJok2GzghYvrib2yrMUPWgDXVpj+KlZZGBlWU14ARvasmULviReenVT\/hQNFA2MlAZqQG77\/MSSrl69uqp0pFc35U\/RQNHAyGigBmSfn0xS+PyUW1+7oXwblt5FzY2jw4035u6ijLkqkz5Dt\/Tcpger80499dSx38Q2ednGV+KmrgEbkvqtkJy65L05a0BWmIMFfOe1JzkagMksYHbfay31XnGT\/6sRxRJPBxqsXbs25Z3I5CWWHKEB7\/PWW2+tlsVaqUfP4iI9QqvuTjrppGQYZTg14SfEyFjCgTSgjfvRh23btg3EPxWmGpBZXGugvcxeJB3fVApry2PFl6Wgvi3rPKzH9nmqjbfETU4DgPm73\/0uWRa7YMGCRM\/imlKefvrpZF7EMKqZVu73TQM6zrvuuit997vfTYceeui+CeuTuwbkPnydJuWNSMNjITotcA4Jd+KLTjIeGWjjWsj78VmQ18Xj4mKzINIK7bsGGD0ezkEHHbTvwvpIGAdkL9bnJi81yL34PnJK0gzVACucfxa85ZZb0uWXX55Ykhn6SHOy2jUgA6vPTCxk7lq7Fy+9Cy3t2LFjbFz83ve+t3IFuyhnLsp8\/vnnU\/614YQTTuirBm74H\/\/4xzRnvaK+2hndxBqQ44U3dz\/FfaQP83Es0zQuJtv4zTjOeG6YZcxVWeYcdIxAad6BnsXl+tA5X3bZZWP7zc1ZWLxQ3kGupdG\/rgGZP8\/6mmGLcZLQvXjpw34kyzTtd7b6ZcWKFenqq69O3L1hlzMX5XlfV1xxRbrgggsqci2OLnyKQnTNlbaE1lDq9ttvL++AgmYY1YCs7gbmy5Ytq168F6sRuBcvvQuyXJMrv3nz5gTYXZQxV2XmS2hdhx7oHLmnc7ov74A2uiE6Nqko7KKEcUBWiBfspQa5F1+oaKBoYDQ10ArkZlXLfdFA0cBoa6AAebTfT6ld0cBAGihAHkhNhaloYLQ1UIA8hfdjtneQjQVm\/C2UH+biCjKtvjIRaUPEINXHNyqLetRF3Ye9Zj\/0EPJH5XmjXl2Hcx7IXSt42PJ9E\/Y50ERkPgs97HK6lOdTow0c8SlsmGXRiQ0i1iMMU+6oyypA7vOGonfPLYg4O8GsTQ6rLA5PkHtW2PdZRwA7hdS9xRcsRc7Xq3gygk8eecVZYafsiGvm5y1EPtd5uu\/00prWEJ94xNqz+vJ5vs985jPV9kY84oTBZxNNJEcAAAPhSURBVCEJHvFI\/aShXvXDl5P81157bVIn+dzn6XGtTrl3o6xBywgZszksQO7xdgEvtgCyfk4SBSA9vpVuzvr2bV0D+\/a3v11tFcQnzVZMGxXWrFmTbA+85557qgP8V61aVe0ywmfLJmDJ36yCOGl4rHaTLq+yxbFoQos5pAVp3L5VymMrKqtHlnT3OhRpNlLojMTL42hi6erF2tutIw1ZGSaPT5BkWZfN4jlh9amnnsJSkbTQA36R6iyciO67775EV+qwadOmpE4T5SnpdQ0UINf10fNOQ0ZNBh\/4gUcorddaZh2DRuq0UXzACORcZfc5iZOGB1gd9iAvGTlf8zr\/cQBuq18AiXrNnz8\/WZ5JnkMiIq\/O4ZFHHqk6GnHN+lvRJ480yzc\/\/vGPV4t2yNe5iUfSYmknfnXO19Dj6UU6D\/UMmZ6jF2+Jb9dAAXK7XqqGff3116dYutjPjeMOcgsR17eHyCpaOj6gYoGa2woxNeNsZGAZpfUirjfg5On5NSvMS8jjXMvn2dQJhaWWhnJgN+slPSfP47nI8ZzNDRs5b36dl5HHl+vBNVCA3EdXrEQsXWTF2lxFbiBrqRFzTbm8vUQCI7cUX1CblW82bBbaPu1ecsWzgqyn68nQunXrKnbusDoZGlQRLX+a9WqyGG6QEZRb+iZvfp93EPl1ztO8HpSvmW+23hcg93izxnz55Aq2iRoy62Z8jLdJ3MYTTzwxxfiTfBNLOoImLwtskkznEDL9yiUZTd78Xv24+fJww00gtcnP8+TX8hgv53H5tZ1qTzzxRLVTqsmbp8nDS2Hp1cV9P9IRkodcx\/CjmcdROTocMj1nM30u3xcg93j7rHHsyuIqajjcbOwAY+JLYzWO5bYKNWZ5bBe0LRMgWVKTTBopi879JS82oxijkpmTsm3w555yVaXJK+xHrDurLI\/6GMO2yc9lsKJcYHnUU5nqCCw5n2v1Ak51x\/uJT3xCdEXSrrnmmrHNNgC5evXqgXayLVy4MBknN+tshpyOFUC+zTvqR8+nn3666Ip0is1Ot0qYQ38KkPu8bMAIN9FsLfcVO3CIN2stTpp7bvgXvvCFZJKJ9UTcS+Q658VPPnltFGXgI19efOLze3E5qZM8KOQ384jHJ596qR9+ofqHfDx48QW5x2tG\/Be\/+EXSqUWacqQhssiOtH7haaedluhOPvKD17U65Pd48JrVj3oCeeg8eOda+K659sDleaeuAW46bwKx4Kw\/sE1WoiEDz4WXMtm8E\/GrI4+BVzQR72xKL0CeTW+z42fJLS7LmFvLQYsOGWGxyZhKZ9CrvJAf1roX32yL\/2cAAAD\/\/9xGL1wAAAAGSURBVAMAgRfhPJLkse8AAAAASUVORK5CYII=","height":146,"width":242}}
%---
%[output:83771a39]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAPIAAACSCAYAAAB\/nJf+AAAQAElEQVR4AezdC4xexZUn8FKW3Tg8ApkFwssJTohwRIBF4GDECpIl0VjrsGQiIS0JeYiVIN6wwloTvKwScMIKZniMjNYsCQqWiCysDCsWxiJiBNkR0Vg8vBGxE2sZhYRgjM1LhMQ8HA1Kxr9rTqfu7ft9\/XX7u+2vu8vy6bq36tSpuufWv86pulX1veuP5V\/RQNHAjNfAu1L5VzRQNDDjNVCAPONfYXmAooGUCpBLKygamAUamM1AngWvpzxC0cBgGihAHkxPhatoYKQ1MOeA\/PDDD6cPfehDSTjsN0NmLvv1119PV155ZXrppZeGVhSZn\/\/85xNyTbByv\/Od77jslJSRl9uvsGuvvTb97Gc\/68cyljYZ3rFME1zQjbp6H6eeeurAdYl88rqeoJjOkrWZc889d+B2WgE5MnnoiYhw\/J09QceCP\/WpT6Vf\/epXSTjsosjMZa9atSpt2rRp2MXU5AHLihUranFd3Hjn69evT+ecc046+OCD+xYB8Pfdd19fnkicDG\/kGSR85pln0tatW9PnPve5tHnz5nTyyScPkm1keI488si0aNGitHbt2jRIh1IBWe2PPfbY9Nhjj1WNXGNsI+n48PcjjUsvGJ0Ci4FfhfR0559\/fkLS8eGXjvCKR3mnEfG33HJLkke6RiBPG5EZfHjlxyeM+2Z98MsX8fgQqyqvNDxRbi5Len6P5957703PPfdc+vSnP12zCCGfLlzLq4z8eaMs5SOy8b1DVfDKK6+kyy+\/PO3atSvdeOONlfWXoGx5kPqSJR4pJ+Ivu+yySpeRri7qJB2RIw\/yLK+99lo6++yz3VZehvriQ8GrnuqiThdccEFlUXQCOa86ENLGK54sMpH6qJf4JjXlyodHfOhFvdtkkCleGSjqJD9S\/4suuqjy3tSdTPEIrzwoT\/M84iJdmPN7F02dk0uGfIgMedCSJUsqTMKd+35UARn677777iTsxyx9Ij6N4uKLL07Lli2rOoWrrroqsRjiQ\/a2bdvS9ddfX1XysMMOSzfccEPV6+DBK8+WLVuSTmP58uVVWuTduXNn1cPqaVkIioi0CMV5kVEHvNddd13V+IInD6M+0XOzpNLV4Y477kgaQ65gaRPRV7\/61coazJ8\/Pz300EM1i8CisWzPP\/98evPNN6t6sdx6YDqmh4l0qPzDDz88rVmzJh1yyCGJzm6++eaqw3jggQcq3WoAuX49g2fBu3HjxqoDICfIc6uTfJ4bIOWR\/vTTT6eTTjopLViwwG3VcbjAS97tt99elc0rca9O999\/f+X5kAMUjIM0dSC3jVc8fuWTrT7qpaycvOMLL7ywslrk4pdPfjoMvXj32iyd5\/lDZq93HG3CM+jA1Fl+8nfs2JHkizTlSstJnbwP\/PJ67qbOdSbat3ZOHh7t3\/sn65RTTknaD92770cVkClFr4Bc98swUZrK4omemyI1prvuukt0RdEgKNwL5gJxhfK8FJ839irjnj8nnHDCnr8pCSn4xRdfrO7zP5QiLepAoY888kjPjirqQ4bnByplq4PG5qUIpQ+L1E0d1RWx3Hpg8nM9uG\/Tofg24kJu2LCh6nwWL15ceQTB9+CDD1agV7Zn84yRFs8dnUneiDQ4jTd4I1RnYFK\/6AQjLQ\/pH4\/21dboc1511HiVr32oj7LVIedr6syzokFc0XhWz08P3m3zHWuz73\/\/+xNyHWDCq2PQEfA4WO68Xq7jPbr2PDq1Np1r89p+1AOPPM02HWVL60UVkClML65noAwmnhsQPUOvzG3xCvXgFNCW3i9OXoqhIHXw0jX2\/MEAuJ8MaeQIp0LKUuZU8k4mD8umA\/GiUTReMtR\/qjpkAejuxz\/+cWWVvU8yg3rJjedmPeSXD1DVhdfAMoYM4Ze\/\/OWqU8CDF0gBRFpOAMiFxfPNb34zsZx5en6NF2hDpnqoj7LVIedVLwCZSjuLZ83lNa9h4cADD2xGJ+67ekngMXhvrvtRP51r79o5mdq9e8+Wy6MTusnjmtcVkEXqmfQ0eibELSVYAcgD4JuIAA0QKGsi3ma6vF4Ol0UdUL+evpk\/7smhkKnUQcOg+KYyQ\/awQvrWE3O7n3jiicpF1KGSr\/5T1WF0CqtXr05tDbGX3HhulpPeg1hTcjRsdQti+b0blhFIgU+DjPQIw+pwG1mziG8L6eSYY46p3EkgiTq0eVN0NN3vGJh0kJ6XRW57hra4fjrX3ukmnlVoWJbLOeaYYyacYBwDcp7RNWGEIkr1AG09Lt6cwj0I91CP6kH04MFHHiLPOJdlYqHyvJSmJ0euI+8gIbdMT6lR49cJDephABN3jnutfrwSecmIxh4gD\/nKmArF827fvj3l7ljE99Nhr\/I0cPrWidExCl5laPzkeja6j7Tmc4dlFwbAwjJ6H94Lkv\/qq6+urLOy3eeU60w+rm+e3rxWR52CDkIdWfqYNMp5m+\/Yc6JLLrlkwkbffNb8HedlNK+betC21bXJl997nl461+a1fdiiG21MW1OfXEabXvN0161ADgWyxMisq5dFATL1Iz31unXrkskPeYXuxUe+xXvGbvfcc08SimM9KAmPWWk9uxel4UQavkFJPU12sHbqQB655A8iY9Wez0YskPrxSjy\/jo1cY3ovkNyJZHmJXjQ5ANHkjxep0\/G8ka6edEZ3yhG6Fx88EYYMzwhYyuRRqLeJk+OOOy7RI9eURWRx8XqmhQsXhpgqzJ\/70ksvrSbQ5JGoMUUH4V15L+Sqt7I0SBN0eKMjEk9XoTO8Oi1WKDrDnFdDVh4LpXx68x7Ui9ycvAttSIdLR\/i57fLnfL2uySRbGepJH95xL\/6IZ5DoQT7vhX77ub7q00vnTT16L3k71ZlpP3Qf5fcKKyC3ARcQWGPEhWprRE2hehLuonh58rx6HL3lo48+mpBBvh606TZ5cPlQnhbxQvIpXRm96iVeOjko8gnjniINJ5BrcpFrcfgQ91I8Uq44JF5IpjRh230ehy8oysmfM9Ly+nsO99Iij\/q5Rq6VITz++OMTee7l09u71\/B1JoClwUsDKKBnNXPZ8iLPKh5pjHhZevfkkYsPKVtdpKkr+eLJQK6R+kgT18YrThpelMuVllOzDvQf6VEH7yji8lBdyVYGCr6Il+Y6yoj0kCuP5\/A8wat88cIoayKdh3z5UJ5XZ6fDQCGvV1gB2RhI70YQUkEV7pWpLR5QfUb67W9\/25ZcfWbhYpB71llnJW6ph2hlLpGdaECDQKwXKwbUJqAGeQ94WFausXfdSQVnoVD6Ru\/ovPqaMIjOGVdDH8ZPhzKRaiogc7t8z5K5Xwbp3Ddhk4\/r95GPfCR94AMfaCZV93ryAw44IH3ve99L0YNVCeXPtGlAg6B7nXVQbgEmqshX93wbl5+ciXhL+l4N0BWdhb6Fg+hcx8njGYRXSRWQXRjv6Dn01L1IOj78OXGpn3zyyfSVr3wlj65dm2DhTpNBvvFQjaHcFA0UDUxZAxWQA\/16i4lIL4E\/SuRmGU9bSXXQQQdF9LhQbx6yjY25DcYPOaOJEGmFHqu+ARc9zB49aNt5Wx\/2dQXkfRHqO6EGZ+aPtWV1zV6y0r3kGpObMTSYDx4P+vWvf73a1cN9L7R3h1PRw+zQg7atjUd7H3a4z0A2eWVyjLUFaJ8hjJfF55XlSiNxwO+h9n56EJOS+8cffzzddNNN1RjauKLQ3Z3owqSkzyjf\/\/73O5Ff3lv9vV1xxRVJ29bG97b24f\/dZyD3qxLgIjystKl642PWm6Vpgh0fS82yF1pcfWfvQg\/0\/6UvfSmdccYZnZXRRb1nqswzzzxT0+6UhgpkY2cL9gOgxsXIEzRn7yJeWqGigaKBfdPAUIG8b1UpuYsGigamqoFxQDYLze3lAguNZx0C0G\/yaqqFz\/585QmLBqZHAzUgA7EVKHZb2IFkCd8RRxyRli5dOrb5f3qqVUopGigamIwGakC2wktmi9aFQdbZuo5014WKBooGRkcDNSCbrGKNLdd84403qlq+\/PLLySkQ4qVXkeVP0UDRwEhpoAZkNbPLw0Jt42PbAM8777xksbx46YWKBvZqoPwdJQ2MA7LKWahtgUdQ+VREK4WKBkZXA61AHt3qlpoVDRQNtGmgBmTbEx2t4tNTG7UdQ9ImtMQVDRQNTK8GakA2mWU8bJY63GqhWWxxjiGxTtdnqumtZimtaGA6NTDzyqoBmUW2Hhpw80cBYucSHXXUUVV0+QxVqaH8KRoYSAPP\/Ivj02ufvTP9+795K63f9MJAeSbLVAOy7YUEOIgsVnIJ3YuPbYfBJ65Q0UDRwOAamP++eYMzT4KzBuTY2MCFtkPGOFnofvXq1enOO+9MTtPEN4kyCmvRQNFAxxqoATnKan5+cm\/8nO9sCt4SFg0UDex\/DbQCubVaJbJooGhgShrY9upbU8o3mUzjgOwgAC51k3yWMhk2GeG9eP1yAOqVXuKLBooGJqeBGpAB1Y+52flkptpB5j4\/ufZZins9OfHjuR245zzl8Sklpmhg9mvgA382DZNd1Gjrol8e8DMVDpEX53OUz1KA7n6qJL8Dzr\/4xS9OVUTJVzRQNNCigZpFjs9KLKaD8fxKoM9P7tvOs26R1zfKb9vYkHH00Uf35Hv22WfT7t27p5fmYHm\/\/\/3v01tvvVX0PA3v\/h+efm2svU+LRfZZyWcm1pdVtgPK5ycAHORnLsZq23LBpRZtBlzYi5566qm0c+fOQh3q4IUXXki2p\/r1j6Lr7tsaw6S9dwVismsWWYRxsONMhXY9GSOjiQAobz\/iprPsJtF0DK7bJrxsm2SxCx2dutKBTtrJL1bqdVVGkbv3\/f3Tu9+XfvL87goaXS0GIbwGZGNYVlgoMch9W3ykDxLaz6xDQMbcJtDENfP6fah58+alQt3p4D3veU9697vfXXQ8De0sQKyd\/9sTDhN0QhWQAdXnJecGO2ReyHIGuVd6jKFdF5ohGijV3G8a2Pbq7vS19f+\/Kv9db76Szj30xeq6iz8VkLnRftMpQCxkOXPibhtDD6MSXPY2azwM2UVG0cAoaACI\/8P\/enKsKv9q28ax6y4uKiCHYIAGWGHElbBooGhgcA0A8F\/93TPp3\/yPR5NrOT\/0h2fTvKf+1mVnVAE5XOtwpdtCrje+zmpSBBcNzFANAKztidxoAP6rv\/v12JOs\/PPj05r\/uHDsvquLCsgsMNc6d6Wb19LxdVWRIrdoYNIa2I8ZmuAFYmCOKvnUdNtFH00r\/3xBRHUaVkDutIQivGhghmsAaC3qAFSA\/bP\/+veV6+xaXP54AeCffuOsdNGivQdx5OldXY8DsmN8fGrK3Wv34ruqRJFbNDAKGgBYBJxAirjKyMSVe2nNugLvyj0u9N\/+59PSdAM46lIDMrDGT8bkrrXD6cVLj4wlLBqYiRoAVJRb2PNve7KysACLArBAi7f5nIDL2nKdX\/3rT1bgXbnHhe7yO3GzDs37GpDjLC4LNnLGuI\/0PK1cFw2MYOBfHgAAEABJREFUmgaAL4BqBhkwm2DNLezGX742NsOcPwvAogAtixvABWLxOf9QrycprAZkk1msrzO6bJYgS+hevHRxhYoG9pcGcpCymECKAqgxfg2gmkHG1wusngNYEWAiIAVabjJyL35\/Wlz17Ec1IGO0UGPZsmXJZgnjZKF78dILFQ10oQEARW2WtBdIARhIUT+gqi+gIoBEwJmDNQArXvoog9bzNGkckDFYeZWPkd2LL1Q0MBkNACYKcAJc7uo2AWp82mZJJwKpOgHp2R8+rJopBkSARMAa7nAbWOWTf6ZTK5Bn+kOV+nenAcBEUwEnC5q7uoMCFNh6gRQ4A6gbvnZaAl4EzGimWdapvrkakK3csoJrVD83TfUhS77eGgBK1AZMwMutZow\/c8uJZ7LgVBvgRDlAV+75hAOErCgC0qBeICWDvLlONSCbzHJmF6WccsopyRgZte0bxlNotDQAkAgoEVcWhTsLdIC5+OYn0+n\/89fpmP\/+6Nhnl3Bp8QQw5R3EaoYWgAoBJ2IRA5wBUMAMC+o6B+jKPZ9w5GFFEVkhu4T9NVADMlY7nGycyMfIS5YsSSw1i42nUPcaAMigfqAETBayaS2HAUxPCUwogAloK9+xnP3AGQBd+Q445SvgpNFuaByQLfrgWrPEQStWrEhr1qxJLHZbNRzjE7zyktHkEyct+OZSx5ADcrKgBNJ+oJyMxfROgPKY9x6QTj923tjE0MoGMMOtnchyFnDS6GhQDcgs7tKlS5OD9vI9yZs3b04nn3xya43lufXWW5MjdLds2VLxrFu3rgrzPxaT7Nq1q+Jj7WfiJowApHBQQA5qKQESkZ3rrd81UCLWEgEWagKTCwuUyPX\/+2+L0h2fOyrd8hcLqsmhlQ2rWSxnP62PZloNyCwugCHXg1QZX\/OnZByl28zroDdxzosSjgoBDsqBaZyIuK2IVcwB6X6YVpIuABIBJAJItHJAaxmuLHd3ZQOY5Cqj0OzVQA3I+\/KY3GsTZGTE0UCugzZu3Ji2bt2apHGv\/aJFpHUd5kANgAJjDs4cmCZ5EAuJ5B+0jkATNAggWciwlK4BEgEkaoKyWMtB38Tc4hsakJ2yyWU+55xzUtsGC4tKpCNu+\/r16xPwN9X97BDOtf7VS69Xv0MLtAHWAGoAdBBwGksi40n0FycfltAV5x6Vbv7s8RX9zSULE3rsytPSjuvPSsKge\/7TRyv3lQt7xblHV3nlP+O4eQkdeWBKyHGp003lXOvpOzv97bffbjbzod8PDchRM261MbYxccQ1Q4f4HXvssSl+bzlPn+q51j\/5x+fSHf\/3F4krfMZfbqoOPQPaXHZcAycCzvM\/enC69OOHpVWfOrwaNxo7bvjycekn\/+X4JETi0Dc+cVhCXzplXvrk\/FTRhw\/andC\/\/P1vZsxZ3NN9rvVcPzv7Rz\/6UTS9zsIakE1cmVkW5iW6b4vH00xzfvWiRYvGzXBzpZE8zzzzTNq+fXvyaxbuc5rKudZ\/\/1xKl\/2fF9Kqh18ZO0M4ZHJzjTW5qWZjWU2TPYgL+90vnpxWffaj6dJ\/95H0mTMWVHT6ifM7O1N6FM57Nk9RzrU+etre8cKFC6M5dhZWQAZGn4OMX7m9QuPYIPdqwJIKczLZ5Wdg8ODfsWNHWrVqVcUCuMjNxRdfnPyCBR4bMXQMbTPhkznX+qU3U7r1kZ3pyvt+nXb8bq\/7ArjGpoAbY0\/XwGx8Wc7LnpfKudbzpvVM7w9+8IMg0ClVQAZGM9UBYqGxbE4WiVgs0labGB\/jz\/mMi5E88krDgyJe2lTIGNe41yok+QF45Z4ZXlaXpQVc8YWKBuaCBiogx4MCNLAJI24UwwCxUP2AGIBX7vns4lpcoaKB6dDAqJRRAZlrzdX9+c9\/nrjY3N8mice3vysOvCyxUF1YXp9tCoBpo9Bc1UAFZBaYJf7Yxz6WuNhc3yaJx7c\/FQW8PikJ1QOIjX9dFyoamMsaqIA8UxSw8ZevJaS+LHABMU0UKhpIqQZkrjMXuulWuxcvfX8pjRVmjZUPxMbErgsVDRQNNIDMdeZCN91qP4E6iR8670Sv9tSGYC41MMd9CYsG5roGaha5lzIch7t27dq0vyzytld3V0su1Q+AzU67LlQ0UDSwVwMDAdn2RMsu92aZ\/r+5NS4gnn79lxJHXwM1ILO4xsLGxDnZBLG\/XOumNeZWj75aSw2LBqZXAzUg9xojGzNbvTW9VdtbWsxSuxsBEKtGoaKBkdNADchq55clbEUUurdWmpVmrd1PN939xM6qyDI2rtRQ\/hQNtGqgBmTnat1www3pmmuuGTvax5roiy66KC1fvjxJb5XSUSS3Oizy\/PfN66iUIrZoYOZroAbk2EMcJ33E4\/n85DrSXU8Hrd+01xorq0xy0UKhooF2DdSAbIzMrfajbeFac6kvvPDCtD9+xO0fnn6tqjW32hbE6qb86UoDRe4M1kANyJ6DK+0UTPuHzVzbZ8y1nu4fcStutbdRqGhgMA2MA7JsNvw7AtdsNQJu8ftCxtd2WOkcUNt5Xbl8QI77Yo1DEyUsGmjXQA3I3GhgE7azTz2Wleee6xjuuOOOdN111\/VdKbbxl78ZK+zsD79v7LpcFA0UDYzXQA3IxsjAdu+9947n3McYVj13zx2+13Z0UBRTxsehiRIOQQOzXkQNyCzxpk2b0o033jj2A27cYDSMb8nhXlsp5pwvx\/+0aZhbXT47tWmmxBUNtGugBmQWuW33E3dYvPR2MYPFAq4DDJwJxrVuGyc719q51CFx8fEHp+k+83kulFfOtS7nWgfGphxyqbnWvc61fvCnz43JPvHQt2fMedEz5fzmcq71zmltU9N2rjWX2iRXl2d2WeqJIHSic603v\/ino22dNT0KZ0HPpjqUc62n70xr7abjc61Bau8JIVxmLm+XZ3b5Lp2fa50vA61q8s4f51o\/\/uzr1Z1lmeUc6uGfwVzOtR6+Tvu102k717pCzTt\/rOiyuksoihUdxkRXjI+Nt1Gv3VTP\/Wa3Yisq348rNZQ\/RQMTaqA22WVWeX9vmjBjHbUu349DEyUsGuivgRqQY1PE\/tw0Ed+P+1e7pBYNFA3kGqgB2ViZW70\/N03E9+N92yiRP2K5LhqY\/RqoAdnjWoFlOaXJKQtBpnvTxB8O\/NeqUahooGhgEhoYB2R5u9g0Qe5E9IcDD08InxlrYaGigaKBiTXQCuSJs3XDkVvjMmPdjY6L1NmpgZECcq7iMmOdayO\/LtdFA+M1MFJAfvvwE8fXsMQUDRQNTKiBEQPywrEKF9d6TBXlomhgQg2MA7IdSWarmzSM1V0T1uYdBp+e3rksQdFA0cAAGqgB2eYJ2wud4GEZZU7D2MY4UX3CtS4z1hNparaml+eaqgZqQCbE9sLmyi7xXVN8dlJOsci0UKhoYHAN1IAcK7ucEDK4iOFwFiAPR49FytzUQA3IXOv169cnZ3btzzFy+fQ0Nxtjeeqpa6AGZBbZWDgfG8e1eOlTL6p\/zvzUzP6cJbVoYCZqoNs614A81aLsWQ4L3mt22xZJp5D04su3L5Yx8lTfRMk3VzUwDsg54ADPsTznn39+ioMGmooS\/8ADDyQH6rHeixYtav3BN1skd+3ale6\/\/\/6Er2nhA8hAjJrllPuigaKB3hqoARmIHVXrbGuAO+SQQ9IRRxyRli5dmhw4IL0pygaLDRs2pHC7lyxZ0mSp7l988cUqdF5UdVH+FA0UDQxNAzUgs5okX3XVVYIxmsyvMT744IPJnmZH+4wJ2HOxcePGtHXr1mRbJPeaO74neux\/7EM++pADyvG3u7s\/qrUch9u9juNY5bff3nuY5Fhj7+CiBmRWlTVevnx5euONN6riXn755TTorzEGOO1prjJnf8RxqRE33Oz4ww8\/XHGEW+3myAPTtB5VOlOOsB1mPctxuLP0OFwACvKzLn4Fwvj4oYceSuedd14a5NcYr7zyykqE\/NVFnz\/9zrU+8A9vJIfUF3q2Mz1s3749\/fCHP0yPP\/54Z2WU9\/en99f0TvtAY8pJNYscUpxwyXIGsaaR1hYCsbFxPz7WGslvAk1jOvvss93WaMP6O5OOpNAlnenBPMgPfvCD9K1vfauzMsr7+9P7u+2229KZZ56ZjjvuuFpbH+ZNK5AnU4BNFhaQaBzGvog1NzEGuIg8Rwfl51rjMVEmzSz1q3\/9yfTTb5yV\/vdffi05Y7vQ3UUPd88eHdx0003TC2QABDKADHIvHuia1LTerDgQcidYaCSPe\/HSUcRLCwJok2GzghYvrib2yrMUPWgDXVpj+KlZZGBlWU14ARvasmULviReenVT\/hQNFA2MlAZqQG77\/MSSrl69uqp0pFc35U\/RQNHAyGigBmSfn0xS+PyUW1+7oXwblt5FzY2jw4035u6ijLkqkz5Dt\/Tcpger80499dSx38Q2ednGV+KmrgEbkvqtkJy65L05a0BWmIMFfOe1JzkagMksYHbfay31XnGT\/6sRxRJPBxqsXbs25Z3I5CWWHKEB7\/PWW2+tlsVaqUfP4iI9QqvuTjrppGQYZTg14SfEyFjCgTSgjfvRh23btg3EPxWmGpBZXGugvcxeJB3fVApry2PFl6Wgvi3rPKzH9nmqjbfETU4DgPm73\/0uWRa7YMGCRM\/imlKefvrpZF7EMKqZVu73TQM6zrvuuit997vfTYceeui+CeuTuwbkPnydJuWNSMNjITotcA4Jd+KLTjIeGWjjWsj78VmQ18Xj4mKzINIK7bsGGD0ezkEHHbTvwvpIGAdkL9bnJi81yL34PnJK0gzVACucfxa85ZZb0uWXX55Ykhn6SHOy2jUgA6vPTCxk7lq7Fy+9Cy3t2LFjbFz83ve+t3IFuyhnLsp8\/vnnU\/614YQTTuirBm74H\/\/4xzRnvaK+2hndxBqQ44U3dz\/FfaQP83Es0zQuJtv4zTjOeG6YZcxVWeYcdIxAad6BnsXl+tA5X3bZZWP7zc1ZWLxQ3kGupdG\/rgGZP8\/6mmGLcZLQvXjpw34kyzTtd7b6ZcWKFenqq69O3L1hlzMX5XlfV1xxRbrgggsqci2OLnyKQnTNlbaE1lDq9ttvL++AgmYY1YCs7gbmy5Ytq168F6sRuBcvvQuyXJMrv3nz5gTYXZQxV2XmS2hdhx7oHLmnc7ov74A2uiE6Nqko7KKEcUBWiBfspQa5F1+oaKBoYDQ10ArkZlXLfdFA0cBoa6AAebTfT6ld0cBAGihAHkhNhaloYLQ1UIA8hfdjtneQjQVm\/C2UH+biCjKtvjIRaUPEINXHNyqLetRF3Ye9Zj\/0EPJH5XmjXl2Hcx7IXSt42PJ9E\/Y50ERkPgs97HK6lOdTow0c8SlsmGXRiQ0i1iMMU+6oyypA7vOGonfPLYg4O8GsTQ6rLA5PkHtW2PdZRwA7hdS9xRcsRc7Xq3gygk8eecVZYafsiGvm5y1EPtd5uu\/00prWEJ94xNqz+vJ5vs985jPV9kY84oTBZxNNJEcAAAPhSURBVCEJHvFI\/aShXvXDl5P81157bVIn+dzn6XGtTrl3o6xBywgZszksQO7xdgEvtgCyfk4SBSA9vpVuzvr2bV0D+\/a3v11tFcQnzVZMGxXWrFmTbA+85557qgP8V61aVe0ywmfLJmDJ36yCOGl4rHaTLq+yxbFoQos5pAVp3L5VymMrKqtHlnT3OhRpNlLojMTL42hi6erF2tutIw1ZGSaPT5BkWZfN4jlh9amnnsJSkbTQA36R6iyciO67775EV+qwadOmpE4T5SnpdQ0UINf10fNOQ0ZNBh\/4gUcorddaZh2DRuq0UXzACORcZfc5iZOGB1gd9iAvGTlf8zr\/cQBuq18AiXrNnz8\/WZ5JnkMiIq\/O4ZFHHqk6GnHN+lvRJ480yzc\/\/vGPV4t2yNe5iUfSYmknfnXO19Dj6UU6D\/UMmZ6jF2+Jb9dAAXK7XqqGff3116dYutjPjeMOcgsR17eHyCpaOj6gYoGa2woxNeNsZGAZpfUirjfg5On5NSvMS8jjXMvn2dQJhaWWhnJgN+slPSfP47nI8ZzNDRs5b36dl5HHl+vBNVCA3EdXrEQsXWTF2lxFbiBrqRFzTbm8vUQCI7cUX1CblW82bBbaPu1ecsWzgqyn68nQunXrKnbusDoZGlQRLX+a9WqyGG6QEZRb+iZvfp93EPl1ztO8HpSvmW+23hcg93izxnz55Aq2iRoy62Z8jLdJ3MYTTzwxxfiTfBNLOoImLwtskkznEDL9yiUZTd78Xv24+fJww00gtcnP8+TX8hgv53H5tZ1qTzzxRLVTqsmbp8nDS2Hp1cV9P9IRkodcx\/CjmcdROTocMj1nM30u3xcg93j7rHHsyuIqajjcbOwAY+JLYzWO5bYKNWZ5bBe0LRMgWVKTTBopi879JS82oxijkpmTsm3w555yVaXJK+xHrDurLI\/6GMO2yc9lsKJcYHnUU5nqCCw5n2v1Ak51x\/uJT3xCdEXSrrnmmrHNNgC5evXqgXayLVy4MBknN+tshpyOFUC+zTvqR8+nn3666Ip0is1Ot0qYQ38KkPu8bMAIN9FsLfcVO3CIN2stTpp7bvgXvvCFZJKJ9UTcS+Q658VPPnltFGXgI19efOLze3E5qZM8KOQ384jHJ596qR9+ofqHfDx48QW5x2tG\/Be\/+EXSqUWacqQhssiOtH7haaedluhOPvKD17U65Pd48JrVj3oCeeg8eOda+K659sDleaeuAW46bwKx4Kw\/sE1WoiEDz4WXMtm8E\/GrI4+BVzQR72xKL0CeTW+z42fJLS7LmFvLQYsOGWGxyZhKZ9CrvJAf1roX32yL\/2cAAAD\/\/9xGL1wAAAAGSURBVAMAgRfhPJLkse8AAAAASUVORK5CYII=","height":146,"width":242}}
%---
