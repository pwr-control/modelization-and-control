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
[model, options] = init_environment('afe_uvw_ctrl');
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
param.afe.hw.displayInfo(); %[output:7b3b50a3]
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVYAAADOCAYAAABl7SoZAAAQAElEQVR4AeydCbyN1frHn20oyhwZuuqQRPcmikJHhshQKDKcBhr+ZLiVdHOazVdUUpGhQSQkt6JrikgdY0TTVRJHhFKmkoPk77vqPW2nffbZ+5y93\/XuvR+fz3Ped7\/vWutZ6\/fu\/fO8z3rWs\/Id13+KgCKgCCgCEUUgn+g\/RUARUAQUgYgioMQaUTi1MUVAEUhYBPwGrsTqB4aeKgKKgCIQCQSUWCOBorahCCgCioAfAkqsfmDoqSKgCCgC4SEQuLQSa2Bc9KoioAgoArlGQIk119BpRUVAEVAEAiOgxBoYF72qCCgCikB2COR4XYk1R4i0gCKgCCgC4SGgxBoeXlpaEVAEFIEcEVBizREiLaAIKAKKgEg4GCixhoOWls0VAp9++qlcdtll0qBBA7niiivkkksukc6dO8uOHTtCbm\/cuHGyaNGikMpTrlq1aifpGzRokPz8889y8OBBefDBB2XDhg0htaWFFIHcIKDEmhvUtE7YCNSsWVPmzZsn77\/\/vqxevVrOPfdc8znshkKscPHFF5v20ZeWlmZqPfbYY3LqqafKv\/\/9b6levbq5lps\/x48fl2PHjuWmqtZJEASUWBPkQXtpmEeOHJEDBw7ImWeeKUePHpURI0YYixarlnOuIcOHD5c6depIs2bN5KOPPjJDgNSmTJliLN\/k5GRxLFFzM5s\/hQsXlltuuUU+\/\/xz+eqrr+SOO+4QrOiffvpJ+vbtK3Xr1pVLL71UnnjiCdMfrt91113m2k033STdunUz5bGae\/XqJa1bt5YVK1bIkiVLTD\/q168v11xzjXz22Wfy\/fffS\/v27aV\/\/\/5C\/7DMJ0+eLE2aNDHtLV68OJte6mVPIpDLTimx5hI4rRYeAitXrpQaNWpI5cqV5e9\/\/7vs3r3bkNL8+fNl27ZtglWJ7NmzR9544w3h+vbt242Fy2eIGI2U4TX+3XfflQ8++EBOP\/10mTVrFreCSqlSpaR06dICaToFV61aJVWqVDEkib4vv\/xSdu7cKS+++KKxaLmPlfvDDz84VYT+TZs2zYzlnXfeEUh++fLlhrhpg4L79++X2rVrm\/5VqFBBaHfhwoXy+OOPm77++uuvFFOJYwSUWOP44XppaFiFn3zyiWzevNlYjVhzjz76qCxbtkxatWolBQsWNNK0aVP58MMPZenSpeY61mbRokWN9cd4IDuI7fzzzzfuhDFjxsi6deu4FbY0btzY+GFnzJhh3AMQ9nfffScbN24UrFCfzydnnXWWXHjhhZltN2rUSOhPsWLFpF+\/fqbsU089Ja+++qrs2rXLlMMSx\/r2+XySL18+0\/f8+fML5J6RkSGI6L+4RkCJNa4frzcHB8k0bNjQEMyhQ4cMoTo9xQUAyTqfsx6ZfILQIGgEMhwwYEDWYn\/5jKWJtYrV6tx86aWXZPz48QJJ33PPPcaSxtWA\/xRCpBznuC449xcm3nr27ClYsx07dpTu3bv73z7pPNh4TiqoHzyCQN67ocSadwy1hTARgLx4pS9UqJDxrTKLD6Eic+fONdYivlXOubZ3717h1R81+EJ59YYkIWV8mbgEuJedUG7s2LFywQUXGL+uU45X9KuvvlqYWIMoIWksZF7f6R\/93Lp1q2BpO3Wc448\/\/mgsV3yrWKJO\/5z7ekxsBJRYE\/v5uzb69evXS8uWLY1ftV69esYFkJqaKh06dDCvyLgGkIoVK0qLFi2kTZs25jqv5LfddpuUL1\/e9BVXAaTLkddySI1zc9PvD5Ndjj7ahTCZqPIrYnQPGTLEvKrjA8UHPH36dDNZBVnXPTGpRWhW2bJl\/auZc3yzJUuWFCzvG264wZD2xx9\/bFwDpoD+SWgElFgT+vG7M3h8lPhGISvCn5jIYtIHy5DXZF7tuY9wzjWIcODAgcbf+uabb8pzzz0nECj37rzzTqEs4pT3HwnlvvjiCzN5hL61a9cK\/twiRYoIwus\/fcIPio8X6xQCHT16tAwdOlSwiCF92n\/66aeNnxQXQo8ePQRBF\/0jgoG26R\/kDzlD4lOnTs20jIk0oD\/UQSe66QOfVTyFQEQ7o8QaUTi1sXhAgMUFEO3ll18u+E8JuXIs5ngYn44h+ggosUYfY9UQYwj84x\/\/kDlz5hh3BRYv0QMxNgTtrmUElFgtPwBVrwgoAvYQiJZmJdZoIavtKgKKQMIioMSasI9eB64IKALRQkCJNVrIaruKgCLgTQRc6JUSqwsgqwpFQBFILASUWBPreetoFQFFwAUElFhdAFlVKAKKgG0E3NWvxOou3qpNEVAEEgABJdYEeMg6REVAEXAXAc8RK\/sSkdiYLOzsj4RwzjXuuQuPalMEFIEYRsBa1z1DrKRoI0lHly5dTMJg9iWaOXOmICSygFRZsz179mxrYKliRUARUARCQcAzxEp+TRIgv\/7668K+QiQfJhM7UqlSJYFw\/\/Of\/2SmjwtlcFpGEVAEFAEbCHiGWNnqgjRubIvBfkgkJyb9GlmGOPIZ4iUXpw2gVKcioAjEBgJe6KVniJUN1sgG37VrVyErO\/ktFy9eLGSMJy\/myJEjBXeBF0DTPigCioAiEAwBzxAr2298\/\/33Jl0b7gD2GRo0aJBce+21wpGN2thfKNhg9J4ioAgoAl5AwDPEijWKO4DM7OyFdOqpp0qBAgUMRrgA+EwZc0H\/KAKKgCLgj4DHzj1DrB7DRbujCCgCikCuEfAUsW7evFnY5+ixxx6Tr7\/+WjgyecU17uV6lFpREVAEFAEXEfAMsRYtWlTYJI6dN6+88kqz+RtH9h3iGvco4yI2qkoRUAQ8jYB3O+cZYj3llFOEvYYIpwok3KOMd6HUnikCikA0Efh1d7rsnTEwmioi1rZniHXPnj3GYr3++utNuJWzlJXlrM2aNZNRo0bJwYMHwx44E16TJ082YVvJyckyadIkOXbsWGY7lStXFhXFQL8D3vwO1K92tgxILiXL2xaUb3pVkg2vDgnp95r5A7d04hliLVOmjEyfPl0WLFggEOm8efOEJa4IeQLAp2DBghzCko8++kgWLlxo2qVNYmM\/\/PDDk9rAf+u20AG3daq+zZJoGMTi92zjqsWy9rGu8l6L4zLlwm1yW43iUv3Gh6XCgCVSf9bRHJ8hY7YtniFWBwiWtrLKCqJ1rpUtW1a4xj3nWqhHSJQVXSVLlhR8tLgZ1qxZE2r1qJV79913o9a2NqwIOAjEyvfMec3fMaCxsUx\/eu9lKdroFkOmZz+3RUp27C+F\/t7IGZbnj54j1uLFi8vhw4dl\/PjxsmPHDiOvvPKKsGAAYgwX0U2bNsnf\/va3zGrnnnuupKenZ35OxJMtW7ZYG7Yt3bb0ArRN3ei3JTmNOxCZFr6gUcySqT\/OniNWFgIMGzZMWOLauXNn6dChg2zfvl0GDx4sLB7w73yo57QZatlEKEdSG1vjtKU7q97hC9IlVOk9bYNkJ23GrBN\/qTlkhSCl+i4RRy55Nj3znGvcpw76bT0HN\/RmxdzRyQSUv2UaCpk6dWPl6Bli5VWfZasAxwqs1NRUef\/992XZsmUydOhQ4VWee04ZzkORpKQkExPrlCU+tnr16s5HOXDVcCnd43X5x4APXJWWL251VZ\/b4\/O6vldWbJNQZeOOfZKdlDzlV\/GXllUKCbL2ziQJJBPalTP3\/36GnCD2LYZwmz25QpZ9\/JVg4UVaMEoi3WbY7X30gWye0Ec2319PNnfwyd5FL0hGhYvEd8dUOXbfYtlXp4vsPO2ciIyfScjMH7fFE88QK8tX3377benRo4eQdIX8qw4unC9dutTco4xzPZRjvXr1ZN26dYJ\/FuG8Zs2amVWLvZMqP4zrIJ8NaOCqzLv9HFf1uT2+eNK38N56kp1M6lZH\/GVY51qCYK0FkuuTq5v7lNkzsrEghQsVktaTtsuMjT4JVCcv13CD5aV+butWLOKTEh9OllMn3SbHh10hBdbPkqIVq5nX\/MoTtknl7qOkUtOUiI+XycnMH7fFE88QKzP+3bp1k0ceeUTmzp0rjRs3zgyruOqqq4zlSjIWyoSD18UXXywNGzaUli1bSvPmzU27XAunjXgri8Vha0y2dNvSC8456Z7du5bM7lVLpn2407gR0jbto1rMSVafKZZpwTJJhkyZgCrTe2J4E1Axh8CfHfYMsTpdOuuss8yyVmbz+d8HWb58ubC0laTXTrlQjz6fzyTJTktLE9phFwKfzxdqdS2nCLiCQHKVErL+4XqSUqe8tHlu3Qk3QborevOqBDJlBn+H32y+Q6a+B96XRCJTfyw9R6z+ndPz6CDA6110Ws65VVu6bekFkXB0pzZPOsl6\/WZPBk14TvzJlMkoh0z9LdNwxu25AeaxQ0qseQRQqysCkUbAsV7PLlnIuAa8ED3gb5maCagZAyUQmeYdi\/hoIa6IlVd9Erew60CrVq3ks88+M7sOBFvSGh+PMbxR5OTzC6+18Erb0m1LL+jkVje+1zEp1U+4BbaYkC7aclP8yZTlpNlZptn1Kbfjzq69WLruSWKFEFu0aGEmnViC2r17d7NQIBiw7EDw7LPPCju6rl69Wnr27CnPPPOMrFy5MsclrcHajcd7Nl\/RbOm2pZfvT150p9QpJ\/heaYf4VzcmtgK95uMr9X\/Npz85SV7GnVPbXr\/vOWIlnpVVV3369BEmsog5bd26tbAdNveyA5QNCJOSkuS8884zRciG9eOPP5pogssuu8zEwbJyyytLWk0n9Y8iEAICZ5cqJFivKVGa2PK3TP1f8yu\/flwcMmV5aQhdzW2RuKvnOWIl1pSVUueff74Bm21ZCL1iuxbumYsB\/lStWlVYsVWkSBE5evSokLiFfAMESBPL51TRJa0OEnqMNQScia3hC7YY32te+5+TZZrX9hO5vueIFauSxQI7d+7MfC4bNmwQNhL0X9LKxoPEp1auXFn+9a9\/ZZbduHGjdOzYUb788kuzDJa2IOrMAgFOaAN\/kJsC4bupz18XGcP8P7t5bku3Lb1gG0ndZ+XfK293\/ZuUKSxm1dYD09cFXbF00vfsow9ky+tPyObh15sVULunPiIZhc8Q34jNZgXUz60elR\/OaRi0PcYTqkRy3KHq5Lcc4Cfu+iXPESvkSazpgAEDZO3atSaw\/5\/\/\/Kfcc889JjuVgxAxrazGIs4Vv+rx48eFSarevXtL3759ZezYsYLFinuAZaxOPc5xLzifOdIG\/iA3BSvaTX3+uhJRdzyN+fKLzjMrwZjYGr96n9w9d1+2K5j+drpI6a1LpcjcQWYFVP4lY6RokaImvtSsgEqdmW1d\/+9Mbs5zwjw3beZUh98yv2nb4jliBRD8o+RlXbFihcyfP98E9l900UXcylb4n5nlrhMmTJAGDRoILgQK57SklTKJJnw5bY3Zlm5besE5WrpTskxsoQvx95mynHTvjN+z7vtPQLnhM43WuBmj18UzxMrkE9ms2DkA4TX\/2muvFXYQwMfKPcpkByjpAYkm6NKli1Afoc7ZZ58tbdmMHQAAEABJREFUtKVLWrNDTq\/HMgLOxFaHpMPS9Y77ZdVd9cQ\/NIrXfJ2Acv8Je4ZYS5UqJYRLzZgxQ9q0aSO333672VGAXQVwDRCXWrp06WwRgnzxxeLXISsWQl3cAZCtLmn9Ezr8VX9+cvfMlm5bekE3Wrr9LdOUWY1lcP7XZEeBsjKwVKp803eded0Xi\/8Cjttif9xU7Rli5dUdEuRIakAs1QoVKghyww03yCeffGImsNwEJ1512XxFs6Xbll6+Q5HUnfH5e8KrvbM2n3NWQDmhUdeNfEsuaNdDvJBvIJLjBsdYEs8QqwMaO7ESAcCrvXPtu+++M6QK6TrXgh3XrVsnxL4SOeBMarEaK9BmgsHa0XuKgG0EHKt095hbzUw+hEqYFPs\/OWSK79S\/n\/5hWaQj9L+n5+4g4DliZWuWrl27Sq9evcwkFBNRKSkpJkMV7oKcYMEPO3z4cMnIyDBFQ9lM0BTUP4qABxDwJ1JI1PGX0jUI1CFTPgcTJ99A62pFhF0L7OYbCNbT+LznOWIFZvylhFJNnTpV2O8Kv2mTJk24FVRYGDB69GjBH4tbgcKkH9SVVyDxp9j0fdnSbUsvqAfTjfWJNQqJsurJIdKju9Ol8AWNxCFSSDXcmXwmtrpfVkJm96pl8g24tSSWMSPBxs39eBbPESsWJ7P5JFO58cYbjaXarFkz4Rr3gj2MRYsWyWmnnSaUd8rhUiCezvmsK68cJPToJgL4Qh0ChTwhUYRrh074TfGTQp4OkfKqz86kkeijY72SLQvfayTa1DaCI+A5YuV1n+iAmTNnCvL888\/LNddcI47V6QwH\/ylhVKy0YOUVcazEvJKwJasv9tRTT3Wq\/eV4X9Ju4cvtthyfcZ8VvYyTYHGONiQ73Vhs0RS2CIlU+xCjv0CQwST\/400EyxQrFALF8oRAHXEjHArrlXwDLCrANYD1KtH9ZxYeRFmFZ5v3HLFCirzGs7IKIWcAK6l+++03OXDgQCaQ3MNdwEoLVl598cUXwgKBSy65ROrWrWuyWmG5spKL1VZORc79V16VO+VXSXt7uvy07QtX5fCuza7q8x\/fD5s+9pxullZGU\/b8Vtgs34yEjl9rthVH2AyPWNFgsjVlslkyerjrS8KyUTbP4zXZDcHg8NdTt\/ShsJbE+tcN9xwXXrh18loeQ8v5rds8eo5YA4Gxbds2+fbbbwPdyrzWtGlTgWQRUgVCrgsXLhQWGRAlQAIXhPOaNWtm1rt3Y3m5bv4hqfzYClel0F3\/cVWf\/\/g8qTt1plSOopx+65jItd99lLAZHkJIUU6CKyqnMtG6H0h3oCWx+YuXNxZmJPsRSHck2w\/UFr\/\/zB+3xRPPEav\/K37lypXNhoL4WtkA0NkCOxy8qIfLQFde\/YkaX8g\/P7l7Zku3Lb2ga1M3+rOT7JbEZlc+lOv+Zbw6bv8+Ruvcc8R6xhlnyOzZs4VXdv73QZYtW2biUnEHhAIEbgIiCjj6fLqZYCiYaZnERMDxvaY2r2TCstqMWZeYQER41J4i1p9\/\/tmk+3vooYckPT1dsF4R\/KdMUJG4OsLjT8jm8GPZGrgt3bb0grNN3egPRVKbJ520U8G0D3eFUi1omVgYd9AB5OGmp4iVmNMXXnhB\/ve\/\/8lzzz0no0aNMkIsKz5UIgaCjXXHjh3C8lciCK6++mrd8yoYWHpPEciCgGO9ptQpL72nbZCQrdcs7ehHEU8RKwsDhg4dKk8++aTZDYDtWBzp1KmTkLQ6u4f266+\/yuOPPy7t2rUzEQE9evSQl19+WdasWSNMYpGGcN68ecIeWhB4du0kwnWbvi9bum3p5ftkUzf6w5Ws1mvapn3hNmHKx9q4Tacj9MczxErwP6TKkSWprLQi9Z8jOS0QwFrds2ePUM\/n8wmbEeJSIFk2FiwTX+xOUKdOHUO2EcJPm1EE4hIBf+uVRQVqvYb3mD1DrMSuQoTkTyXdH2n\/\/IVrlMluePv37zd7XT344INSo0YNufnmm2Xfvn2iK6\/kL\/9s+r5s6balF\/Bt6kZ\/XgTrdc\/IxvLN3gxhUcGfvtecW43lcec8uuAlPEOs\/t1kworVV5CkI1ix\/gsEKEMYFSFZTGyREYt9rsjjSopBQrQeeOABQ7bBVl7569VzRUARCIzA+ofrieN7DVxCr\/oj4DliJYi\/T58+cujQIbn88sszhbR\/BQsWzOw7oVT+K69Igl27dm1xVlXx+p8vXz4pX768Cd1yKhLG5ZRxrkHO\/O\/qpmRdEeOmbsbtpj5\/XbZ029LL2G3qjuT3rGPV4zKhXTkTlvWPAR\/IzLQNQTcetDFufsvotS2eI1YIleTWpA1kZt8RJrZYnpodYElJSXL48GH56quvTBF2E8CnilXLaisIG+G8Zs2apozzh1hZHO1uio1VKW6OT3VVivhKptxgGuHvmVyfXN2EZd1cr6J0f2OX3D13nyfG6WDDb9n5Xds8eo5YCanClwoxhgMMJIpL4OGHHzZW7tixY+X+++83eQMgV1159SeajhX15xX3zmzptqUXZG3qRn+khYktfK+kI0z7ep\/xvQaKHIi3cYeDo+eIde\/evSZciigAXv9DjQpg0OzuOmfOHGGlFpNd\/C\/m8\/lM6kHd8wqEVBSByCGQXKWEsV6ddITEvkau9dhuyXPEirX65ptvGr\/o6tWrxYkMgCi5F9twe6P3\/Idjqye2dNvSC842daM\/rxKsPtYr6QixXokYIHLAsV7jedzBMOGe54iVmX8iAJxoAOc4cOBAIQ0ZuwTQcRVFQBHwDgJZrdfhC9K90zkLPfEcsTLzzyQU4VP16tUT3AGkDWRSi3yrQ4YMyRamJUuWCK6D+vXrm6WtLBrQzQT\/CpdN35ct3bb0gr5N3eh3SxzrlWTawxdsESIHHOvVrT54RY\/niBUCxWpleSo7rV577bXy9NNPy5EjR6Rnz57C6qpA4HF94sSJMmnSJONjJck1sbC6mWAgtPSaIpADAnm4nVKnnPG9li9aQNo8t04S0Xr1HLHyPDMyMgyRco4cO3ZMyHxFPgA+ZydYuqy24j5tEClAXgBiWnVJK6j8LjZ9X7Z029IL4jZ1o9+GYL0uvLeeONarv+\/VRn\/c1uk5YiXciuTURAUQLoWP9dZbbxWC\/\/Gxcj8QSFynDsla2DBw2rRpkpKSoktaA4Gl1xQBlxBwrFcnciBRrFfPESvP+7bbbpPJkyebhCqsvhoxYoR07NjRJLsmTpUyWZe0EhhMHTJYsbqKmFb2wsLKzWlJK6s18IO5KZFcERNuv\/kPKtw6kSpvS7ctveBmU3d437MtQVdSMZZwxBn3sf075elWJeSOS0uccAv87nvNadVWOHr8y\/Jbhh9siyeJlcUB48ePl\/fee8\/4S1988UW55ZZbDFZMbnGSdUkrxFq2bFkhiYvP5zMWLomxy5UrZ0K3qINAulmXtFKX1zU3JdIrYsLpeyLqTsQx853w0riHda5lfK+VyxQxq7ZmbPRFfNUWv2V+57bFc8SKL3XUqFHC6zwTUuedd56cfvrp0q1bNwkWx1qxYkUh5eB3331nMCVdIOVZdcUyVpazIpxnXdJqKiTQH35wtoZrS7ctveBsUzf6bUmgceN7Je41tXklY73Gq+\/Vc8T6yy+\/CJNO7du3F8KtWE115513CtYrxJjdl6Rq1arGqmUHAcKtpkyZImybXbduXYFcdUlrdsjp9QRHwMrwU\/\/YCiZefa+eI1YSrTCjj+XJ6\/7WrVslf\/78cvDgQZPxKrtvgc\/nMz5YVmotX75cXnvtNfOa4fN5c0nrlVdemd1Qon4dn1TUlWSjwJZuW3qBwaZuL3\/PAlmv3+zJALKYF88Rq2OtkkTl\/PPPl5deekkaNWokkCwz\/9FAHIe328I43Nbp6OPH5py7fbSl25Ze8LWpOxa+Z+N7NpFi76TKjo0fm4QuYJYXYcy2xXPECiCNGzcWZvQBd9asWbJ06VLp379\/0D2vqJcbwdmtslkUg8TCwGvPO\/2z1fL95B6SesL3Gom+5YYLIlnHM8TKxBNxqCxJ9Rf+t2\/btq2JSaVMJAevbSkCioC3EEg94Xv1Vo9y1xvPECvdZylroUKFzCQUS1NnzpwpjrA8NVquAHSrKAKKgCIQKQQ8Q6yERpFkZcyYMUJQM9YrQf7r16+X4sWLm1ArJrEiNXBtRxFIOAR0wK4h4BliZcQQJ3Grjz76qEl2PXDgQHNs0qSJEHKFRUs5FUVAEVAEvIyAp4jVH6jffvtNWI7qf03PFQFFQBGIBQQ8RaxksWIzwEGDBglB\/oMHDzZ7Vi1evFjwsRYrViwWMNU+KgIeQkC7YgMBzxArM\/7kXyWTFeub2YqFkCuWn+7fv98sV4V4bYCkOhUBRUARCAcBzxArncYizZcvn7z88svStWtXuf766zMFHyu5AyinoggoAoqAlxHwDLESFYCVypLUQMI9ygQD09mahcTW5Akgt4BuzRIMMb0XpwjosCwjYJ1YmekntyrCeW7xYF8s0gtOnTrVRBIQXcBn3Zolt4hqPUVAEcgtAlaIFV\/pnDlzhFVVTZs2zXzdv+qqq4S9qrhHmXAGtWnTJhPrWr58efH5fJKcnCzkdSUhCxasbs0SDppaVhFQBPKCgOvESr5VZv3ZhfWtt96S1atXi\/Pqv3LlSnnzzTcF6\/Xuu+8Oa1znnHOO7Ny50wikTH4ByHbjxo3CZJjTGHle09PTnY96VATiBgEdiHcQcJ1YSQvI9ipMTpHJKisURYoUESIDnn766ay3gn4mqe6NN94orNgiB+uuXbukWrVqwmRYTluzBG1YbyoCisBfEPjpvZclO9k7Y6BkJ7vH3CpuyF867PIF14mV1VVsr0J4FSTon3CFc9wBTz75pBBiFQ4WLCbg9Z99dtiZtUWLFiYbVpUqVYJuzUIGLZXKohgkDgYDkkvJ2CZFjbzZorAgy9sWFGRzB5+EItmR46dPdZMNrw7JVr56Z4qkvT096hIOd0SjrOvE6gwCnye+T3ZSZcafXVUhQ3KvlitXTgYMGOAUDemIG6Bnz57y7bffmoTYJLrGf8tCA7ZjIUIA4ZzYWP9GI5GmLNw20B9uHS2vqf3+8h3Y\/FdM1j7WVVbd21DSupwjiy7Z8he5rUZxubpaaWl7RW1Jbt3ZSPUbH5b6s45K5deP50loIye5bv4hiabcu7E8Py+rYo1Y9+7da0jw5ptvlgoVKshZZ50ld911lxCrChkWKFAgLGDY8+qaa66Rjh07msTYfIaoL774YvHi1izvvvtuWOPTwoqAPwK\/7k43r+JYjjsGND7JyuQV\/eiJ+wXLJImv4+N\/Icqzn9siSIUBS6RM74lGSnbs79+8nucRAWvESnpAZwsWZwy4B5jUOnLkiCDO9VCOPp9PbrrpJiEKYNWqVdKvXz\/B5eDz+aRLly6SlpZm7lHG5\/OF0mTclrG5VYgt3bb08iWKhG6INOPz94x\/EiL9plcl48c8dOIaBOpvafqTptRuTxesSAX9J0YAABAASURBVCTGbaXjEVBqjViZuGLDQCzWBg0aCP7VTp06CUKUQI0aNSIwPG0iEAJM9AW67sY1W7pt6QXTvOjG+sQqhUgh1N+JNMlYoRAogtWJHq9JXsbttbGE2x9rxEpH8adOnDhR7rvvPhk\/frzMmDFDWrZsaaICunfvThEVRSDhEMA6hUyZRGJ2ndd6xyL1MpEm3IMKMmCrxPrCCy8IyazHjRsnrLrq06ePzJs3z8zmB+mz3lIE4g4ByBQSxSrFOsUyxRKFSPGFxt2A43xA1oiVSaqPP\/5Yhg0bJiVKlDBbVY8ePVpYNMA90X9RQ8Cm78uWblt6eYjBdEOoWKeQKa\/9hS9oZCaWINSijW6huiMxdww27pgbTJgdtkasrI5iAuu0007L7DJ+VyaciEnNvKgnikAcIgCJOtap86oPmTI7X+DEbH4cDjmhhmSNWEuXLm3W9k+aNMnEnbL0dOjQoXLGGWeIbhoY3e+gzUkFW7pt6eVJ+uvmdd\/xnWKd4juN11d9\/3GDQyKJNWL1+XwmbpW1+1io5A8gnpWJrHBjWBPpgelYYwsBXvUhU6xTCBVL1fGdYp3mNBq9H5sIuE6suACIVyXRCiuhCOInvd+UKVPkhhtuMNYrZWITztjotU3fly3dbuqFTCFQyBTf6d5FLwjWKZYpr\/ux7jsN9VvuJuah9smtcq4TKxNT7AbA7gCslCJ+lXhWhJSBo0aNkoMHD0Zs\/MePH5fJkyfLpZdealIJ4npIdOK2+YpmS3c09UKkiGOZGjKdMVAyA\/cnbBOs00J\/bxSx73UsNBRNzL0+fteJlV0AyA2wYMECk3uV8CoSpyBvvPGGwYsJLHMSgT+a6DoCIGoTJyEAibIKyt8qhUz57JAplimv\/CdVDPWDlot5BFwnVgcx3ACHDh0SiNa5VrZsWeMK4J5zLa9HMl2R7IWkL0Qd1KlTR9asWZPXZrV+AiEAYRIS5bzaQ6KcY6E6RMoklJJpAn0pchiqNWItXry4HD582Ky42rFjhyCvvPKKyREAAebQ75Bvk+xaE12fDJdN35ct3cH0QpAQJwJhQpxMNDnCfQL2IVH8o5AoEiqRBtN98pOJr0+JOm6eojViJfk0iwOICCAva4cOHWT79u0yePBgIRk2nYuUoCtSbWk70UEA8gokkF1OAhlmFcjREQjyeL\/KJ2WA4pojWKQQJ\/GkkGfhE75QXuMhTwQCRbiGrzQ6CGir8YSA68TK6z\/Z\/QGxWLFikpqaarZmWbZsmRDHyis795wynOdFkpKSgia6Jl+l8wNz8xjshx7tftjWnZZSXrLK52+Ok0DyZdp8eW\/F+qDyevqpklWeO9JQkEeOdZI76i6XgVetNvJ8+48E8Y3YLI4cu2+xIIe7viQ\/t3rUyA\/nNBQsrkgI399ItJObNjBWclMvEnVsjJuE7ei1La4TKzGqb7\/9tvTo0UNI78ceWA4InC9dutTco4xzPS\/HevXqCcmt8dsinNesWTOzyaZrK5lMQVgmbgo\/ajf1eUlX8rSdEo50mLJOgsndL88X5MzeL0tGyjgjJTsMEKRIo65StUIJ2XukgJHPfxSZtylDLnk2PaBc9+ouuXvuPnliZYbM2OgzS62Z3Y5VwQ0Wq33PTb9JBJ7547Z44jqxMuPfrVs3eeSRR2Tu3LnSuHHjzG1B2JYFy5XFApSJBC5eTXQdibHltg2skdzWzWu9aOpOqVNOUpsnnSRjUqoL8nSrEjK7dy0j6x+uJ3tGNg4oKXXKy+VVSsq2PRkyfMEWKdV3SabUHLLixLV0I+HgEGTM4TQTc2UTddw8KNeJFaUIOwYMHDhQmLXnfxmEJNUPPvignHnmmRSJiPh8Pk10nQVJLIEsl1z7aEt3qHodYoaE\/ckXcoZ0l23ae4JYfydch2i\/OUHCwQAMVXewNmLxXqKOm2dljVhRrqIIxAoCKX9Yww7hYvWmnLBup324UyBYZPiC9FgZjvYzyggosUYZYG0+PhE4u1Qh43KAYBFIFtdBKAQbn4joqPwRUGL1RyNBzm36vmzpjqZeh2RxHfgTbNqmfeYbFU3dRoFH\/yTquHkcVomVZCz333+\/EMeKf5VE10QG0DEVRSAWEcBH61iwbZ5bF4tD0D5HAAFrxMrCgOHDhwtpAxkHYSFHjx6Vxx9\/XLjHNZXoIGBzUsGWbjf1OhZsavNKJqKA0C2JzqP0dKtuYu41IKwRK1mufvvtN7nyyislX758wm4ChFgRa8o9rwEVT\/2x+YpmS7cNvanNk2R2r1oy7cNd8fT1CXksNjAPuXNRLmiNWFm2mj9\/fvnll18yh4hrAFJlEUHmRT1RBGIYgeQqJWRCu3LGctWogRh+kGF23RqxkmilSZMm0rt3b2FbFnZrJUdrp06ddGuWMB9iuMVtvqLZ0m1H7+9P5vrk6sZyJWrAmdD6\/U58\/7WJuW1krRErA2\/ZsqX897\/\/laeeekpuuukmc8417qkoAvGEAJZr6gmfKxNaiUSu8fQMwxmLVWJdsmSJ9O\/fX8qVKyejRo0Stml56aWXwun\/SWVpjx0JyL\/at29fwV+rOwicBJH5YNP3ZUu3Lb0A7uhOPeFzTT63hPxz+gYux7044477gQYYoDVi3b9\/v7z22mvGFbB161YzibVo0SIh4z9+1gB9DXpp27Ztwt5ZU6dOlZUrV8p5551nPtPewoULhR0L2K1g8eLFZhlt0Mb0piKQewSC1hydUl2+2UMeAl2lFRSoGL9pjVhJcl2kSBEpW7asrF69WrAymdA65ZRTchVutWnTJrMbQfny5cXn85n9rTZs2CDEx9I26Qjx6+oOAmIyNtn63tryu9nSC87+ugnFIu9AIvhb\/ccNDokk1oiVHQSOHDkid999t6xYsUKSkpKEPa+4BgGG+xDOOecc2blzpxE2CyT9IGTLxBgxsk57xM2mp6c7H\/WoCLiOQEqdcoJLYMSCLa7rVoXuIGCNWMnqP2TIEElJSZERI0YISa8ZMikDsVw5DyZsn92wYUOTcpCIAv53vPHGG80qrrp16wqJsqtVq2ZiZNEVrC2S4+IPclNsJiBm40Y3x+qvy5buaOv1H2PW80C6U5OLSNrX++SZuZ9K1vKR\/Jxo3zN+y8F+627ds0asDBACveiii+SMM86QjIwMadCggSFZ4lm5H0xILYhVSrrBJ554wrgPkpOThS8xqQiZCCMetkqVKkF3EEAHbUDMbgpWtJv6\/HUlom6vjfnyi86T1OaVZMCiHyR\/8fLGPeP\/jCJ17rVxR2pc2bXDb5nftG2xRqwsW8Vibd26tTRt2lTatWsn7HtFhECJEiXCxgU3QM+ePeXbb781O70yMUa79evXD7qDQNiK4qACX0pbw7Cl25ZecM5Od2rzJMHn+s9pGygWd5LduONuoAEGZI1Y9+7dK8z+L1iwQO69914ZO3asvPrqq8Zy9fl8Aboa\/FLFihXlmmuukY4dO0qjRo2Ez1ituoNAcNz0bh4QiEDV0Z2rG5dA2h+ZsCLQpDbhAQSsEav\/2Hll\/+6774RdBX744QdDuP73Qzn3+XxmkQFRAOyl1a9fP2EbGJ\/PpzsIZAEQH16WS659tKXbll6ADaabhQNMZrFwgDAsyseLBBt3vIwxu3FYI9bSpUubhQHDhg0z\/qUXXnjBrMDasWOH4HvNrsN6XRGINwQIv4pnl0C8Pa9QxmONWH0+n5CL9YEHHjAxrPhHiWslbWBuwq1CGayW+R0Bm74vW7ojp\/d3DMP5G4pusmARJRBPiVpCGXc4OMZSWWvECkikDfzyyy9l\/vz5QoLrqlWrmoQshw4d4raKIpAwCGCxQq6JsHAgER6qNWKFPMlsNXLkSFm2bFmmsAqLhNeJAL6tMdr0fdnSbUsvzzhU3fhb2d4Ff2s85HANddxgFG9ijVhJkMKrP77Vf\/\/73+JIampq5mKBeANbxxOzCLjacXyuvadtkHhyC7gKoAeUWSNW\/KhMUoWyGMADOMVVF2z6vmzptqWXL064uokScNwCbcbE7r5Z4Y4brOJFXCfWAwcOCHtdDR48WCDVtm3byh133CEPPvigEe5RJl4A1nEoArlBALcAmxJSt+aQFWq9AkQMievEevrpp8ttt90mffr0EVZekSoQkuUzwj3KxBCGMddVm74vW7pzozdSDza3us2EVu9aklKn\/Ali3SKxRrC5HXekcLfZjuvEyj5XpAx89NFHha1YmLzy+XzC2n+kTJkyQpmcQGFJ7EMPPSSffvppZlEn0TXLWG+44QYhJlYTXWfCoycxikBq8yRhUssh2FJ9l0gsuwhi9DGE1W3XiZVoACzUevXqyeTJk81EFRNXEGWoPcfK7dKli7Ac1qnD8tiJEyfKpEmTTIRBs2bN5NlnnzWJszXRtYPS70ebvi9bum3pBfFI6XYIFv9rxVKFzAaFkKxXLdlIjRsMY03yud1hogF41W\/fvr2Qg\/X\/\/u\/\/zBYq4fhVyYaFtUsOVv\/+Ywnv27fPXCJbFhNkZLrSRNcGEv0TCgIxUAb\/K5ED+GAhWSzZZZv2nkS0kC2RBRNW\/f57iIFhxVUXXSdW0MuX72S1v\/zyi0n7x71QpFatWnLVVVcJy2Kd8qVKlTK5WDt16iQks542bZrJ9bpp0yYhdZpTjnteSHR95ZVXOl1y\/WjT92VLty29PNxo6cYHC8liyc4+4YvFXYBAtMi2PRkyfvW+TMLFukUgXQR3AuSLENqF0N9ISbTGHan+RbOdkxkumpry0DaJrElgS2JrElwHaoo8jLgW2Nfq66+\/Fuo4eVpDSXRN+24KY3BTn78uSN3\/s5vntnTb0gu2buse37OJIJ892V5KvHX7X+T75TMEWb1kjrw+b6mRx9\/8UFj1BfHmRUr3eF0cqTN8Tea5c43jmV3GSTTl5+R+\/LysihVi5fW8Ro0aJvs\/2f7Z\/I8jX8JA5AlBQpxLly41k1yBEOM++2edffbZZs+r2rVry48\/\/mgSvUC0Th3Oq1ev7nwU6qlsTnAcEmv8O+Y+JciumQ\/L95N7GPlhXAczQYbFG4rgiggk911XR3KSDi0bSrQl8wdu6cR1YmXmH4LMjsy4R5lw8SD\/KnGxpB+k7tq1a83mghD1unXrjB8X\/y7nNWvWpIiKIqAI5BKBlDrlJJDglshJAhFypK\/lclgRq+Y6sUas51kaIoHLLbfcIoRZEW41ZcoU6du3r2AJQ64tW7aU5s2bS+PGjYXk11mq60dFQBFQBCKGQMwSK3kGxo8fLxdeeKEBw+fzCdu8vP\/++2bLa7ZmqVSpknELEJqVlpZmrt90003mmqnksT\/E3bK5Ij65d955x2O9i7vuJOSANmzYYCZ5MUAGDRpktjFKSCCiPOiYJdYo42KledIn9ujRQ9544w3hPwJcF1Y6okrjFgHSc5JVburUqcIiHfJ1xO1gLQ5MidUi+FlV4yPGyi5evLgQ68u+YFnL6GdFIC8I8J91OItx8qIrkesqsXrk6fNlP3bsmBQqVMj0iNjhfGtKAAAICUlEQVTe\/fv3m3P9EzkEEr2l7du3yzPPPCPXXnutMA\/Bku9ExyQa488XjUa1zZMRIEKB1y7nKom8yeJVp04dSU5OFpbokh\/hlFNOyVwowaoxVpg5dfSoCARDIJTvGCTaoEEDefHFFw2prl+\/XrZt2xasWb2XSwSUWHMJXCjV2HGWPAj4tLBAnTr4UrEc2FF2+vTpQrJvVqmweuyrr74yu9RSvlixYk4VPSoCAREI9zv2v\/\/9z0zesmiGCWCfzxewXb2YNwSUWPOGX9DaWKEsVLjuuutOKkesbqtWrcz23Gz5XaFCBWHvL8LBxo4dKx06dDBhYnzxT6qoH3KHQBzXCvc7xn\/WLPsm8oTFNP7LveMYJteHpsQaRchLlixpchpccsklmVqYlWVSyslz4PP5zAq0rVu3CiSLBfvuu++aepmV9EQRyAaBcL9juAII5SMskdzHPp9arNlAm6fLSqx5gi93lbEy8KfmrrbWUgRyRkC\/YzljFM0SSqzRRDdA24RR8YrPYgBuM6GAf\/X888\/no0rEEEjchvQ7Zv\/ZK7G6\/Ax8Pp9ZVoufleiAb7\/9VshvUKVKFZd7ouriFQGfT79jtp+tEquFJ9CiRQvBN0aoFUsLb7\/9diGJjIWuqMo4RUC\/Y3YfrBKrC\/g3bdpUSH3oqCpYsKCkpqbKqlWrhEkEEsM49\/SYJwQStrJ+x7z16PN5qzvaG0VAEVAEYh8BJdbYf4Y6AkVAEfAYAkqsHnsg2p3wEdAaioDXEFBi9doT0f4oAopAzCOgxBrzj1AHoAgoAl5DQInVa09E+xMaAlpKEfAwAkqsHn442jVFQBGITQSUWGPzuWmvFQFFwMMIKLF6+OHEetf+9a9\/yaWXXipXXHFFpgwdOjQPw\/JGVRZ2TJw4UchUxso5EpX794xxjxs3zv9S5jl5IUaNGiUsZ868qCdxh4ASa9w9Um8NiETfrC5z5KGHHsrsIAlo2I4m80IMnECmb731Vq7TOrKnGZnNVq5cGQOj1S7mFgEl1twip\/VyjQDWXK9evaR169ayYsUK+eyzz6Rt27Zmm5rOnTsLVh2NL1myROrXr2+s3b59+8qAAQPk+++\/F6xEjpTBWsRChKTZwwnrmBwMbO0MCXKfNrt06SKXXXaZdOvWTdhQD4txxIgR5hr5ckkwDmHec889mdvjYFki6HGELVDYNofcuc617I5Y5\/QHqVWrltCHgwcPCjlR2Yn38OHD2VXV6zGOgBJrjD9Ar3e\/e\/fuJpF35cqV5aKLLpJPP\/3UdHnPnj0ybdo0IV3i6NGjZcyYMWbLb4ht5MiRsmnTJrM304wZM4RMYBAZRGkqB\/iTlpYmGzZsEJKEf\/DBB0LqvFmzZpmSu3fvFixnymAtrlmzRtgeh\/2e3nvvPbPnGK\/37DG2a9cu2blzpyFf9oQi075p5I8\/lL\/gggv++PT7wX+MjBPS5A7WOZY640xKShK26KFfjIXxk9WMcirxh4ASa\/w9U0+NaMKECbJ582YjH3\/8sVx44YWmf40aNRIsv\/T0dJOIBqsOUkpJSZFPPvnEWLFly5Y1uyr4fD4hyQh5bE3lAH8gRggMoj733HMNUa9bt86UxFpkCxKS36Af9wP7jUGahQsXNpnG8JliHVetWlXWr18vGzduFPRlTeeItct10\/Aff\/zHyFjbtWv3xx2RQ4cOycCBA6Vly5bG38wNyL1QoUKyf\/9+PqrEIQJKrHH4UGNpSLwOs1stZAopsZni7NmzDanxeu+M5ciRI0EnfHjF7tevnyFw2sF6HXDCdeDUz3rEFcCGes71ffv2GRJs0qSJsZBxQ9SrV08gXqdMuEf6\/8orr0iZMmXk1ltvNZv4hduGlo9NBJRYY\/O5xU2vK1WqJAcOHBAIFSKaM2eOPPnkk0J+Wl7VSQTOdV6pIeECBQoIlmdGRoZwHZ8nYBB9gBWKRYmV2L9\/f8ElwL1AApnjYoBgt2\/fLnfeeaewF1nNmjWNHxffL1Z01rrlypUz0QBZrwf6jLuB8eACoM9OGSxm+l68eHHnkh7jDAEl1jh7oLE2HPyN9913nzABdfnllwubKfbo0UN4JWeSitfqhg0bCgTLKzgJwvncoUMHMzOPLxayxVUAWXLEzVCqVCnjPsgOj6uvvtpYkEwkdezYUdq3by+4CyC7GjVqGL8wn7PWRwd+Yogx6z3\/zxA21uo333wjTJ5B0hzx9+LHZSxYsv519Dx+EFBijZ9n6bmRkNwbosvaMYgTca7j22TSCYtz6tSpUqFCBUN61113naxevdr4YG898SoNmfl8PmF3UXyqCxculOeff14ee+wxY8VidXIdwS2AlYh++uHoQi\/XILZhw4YJYU\/oRZfP5zMRARBfmzZtTB+ces6RCAIsW6xc2qC\/tOfc54g++sI9\/LxY2wj\/aUCmjLVVq1Z5cjOgR8W7CCixevfZJF7PLI+YEC4IFgu4du3aAXsDmd58882CDxaiD1goyEVCyX755RfB6g5STG\/FOAL5Yrz\/2v0EQYDZfGbXozncM888U95++21jAQebtCLKgJhUn88XdnfwKeP2wJoOu7JWiBkElFhj5lFpRxUBRSBWEFBijZUnFbf91IEpAvGHgBJr\/D1THZEioAhYRkCJ1fIDUPWKgCIQfwgoscbfM42FEWkfFYG4RkCJNa4frw5OEVAEbCCgxGoDddWpCCgCcY2AEmtcP15vDU57owgkCgJKrInypHWcioAi4BoCSqyuQa2KFAFFIFEQ+H8AAAD\/\/0KqhuEAAAAGSURBVAMAJB+\/yQ+tfykAAAAASUVORK5CYII=","height":206,"width":342}}
%---
%[output:4ab8b4aa]
%   data: {"dataType":"text","outputData":{"text":"Single Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\nSingle Phase DAB: Three_phase_DAB_1200V\nNominal Power: 750000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 4.000000e-05 [H] | Internal Tank Cs: 750 [F]\n---------------------------\nSingle Phase PSFBC: PSFBC_1200V\nNominal Power: 275000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls1: 3.030303e-05 [H] | Internal Tank Ls2: 3.030303e-05 [H]\n---------------------------\nSingle Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.580123e-05 [H] | Internal Tank Cs: 4.734509e-06 [F]\n---------------------------\nInduction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:6debe77d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVYAAADOCAYAAABl7SoZAAAQAElEQVR4AezdCfxvxfw\/8PnEX+GHa20RbisVutmKQskS6idC2W\/WbMlSSqJ+SPadCCWFsi+VUpQSFYlCpSIRZZclD9v\/Pk+9r7nnez6f89m+3+\/n+\/nOfdzX98yZec\/Me94z5\/WZmTMzZ7X\/lH\/FAsUCxQLFAmO1wGqp\/CsWKBYoFigWGKsFCrGO1ZwlsWKBYoFFa4Gs4IVYM2MUZ7FAsUCxwDgsUIh1HFYsaRQLFAsUC2QWKMSaGaM4iwWKBYoFBrNAs3Qh1ma7FN9igWKBYoGhLVCIdWjTlYjFAsUCxQLNFijE2myX4lssUCxQLNDNAq3+hVhbTVQEigWKBYoFBrNAIdbB7FWkiwWKBYoFWi0wNcT6spe9LK2\/\/vpd8cQnPjH9+c9\/bjXIIAKHHnpold8DH\/jAdPXVVw8SdYbs+eefnzbffPMqPeVo0rdexpNPPnlGOnPtMU4b9KP7bOTHjmyeYxx12k958nqnRz9xyIQd6KzdSId\/jmgvTW0pl2tyS0+60h9Er6a06n6eQzpJ+8Mf\/vAq7Z5fHePOv65Pv\/eDyE0NsbYV+lvf+lZ65CMfOTIBtuUzrvBf\/OIX6a9\/\/evK5DTGK6+8cuX9JDtm86Ecd7mRz7Of\/ewZyV5xxRVpq622SghsRuA8e2gLX\/\/611dqcc0116RvfOMbK+8n3fGTn\/wk\/eAHP0h3uMMd0gYbbNCqrvoZph7ULZJG4mzWmtEYBaaOWD0M3\/\/+99Nll122Eh\/4wAcqk3lYPvOZz1TuSf9DV+UIPaMxrrvuuulmN7tZeM\/7dY899qjsfNppp6Xb3e52867PIAr4AfjKV75SRdlnn32qcmg3foQ99ALe9773JXLcs4G73e1u6Xvf+16V94Mf\/OC+sqAf5MKIdq7JI89\/EPdHPvKR5MfgCU94QrrVrW61MqrnlP0D2r\/nmcDHP\/7xBdMpou\/UEatC1bHmmmt2JaP4VfPLBoY\/TQ8SP2FkQLx6PnFv6EImMOiw8jGPeUyV1CWXXFJd\/bnqqquqxrjJJpu4nYF6nvJu0rEuR0ZvgHzomZeVPBnhAfKhADf\/iOv+UY96VKUrGb2N6DFEOnEvHMSRBvvKmx9w8xMG4vNvAj3JBEKfJtnw08vzgCPRsLkwPxAHHHAAZ1UOtq9urv9Dj8jHlY50vT545aWuk3hR1tBPPPGlQ35l5B6OL3\/5y1UovYN49AD9+FYBXf6YrpKvvOhBH+5Ar\/zbZPNyRHr1eqYWHc4555zqedx66615dcX\/\/M\/\/pHXWWacxvJ4fG\/IjLA\/ljE6UH6G73\/3uKcpHjnzo6dqkq7TSkH8WBbHGA8RGG264oUs138qYYfzKc8UfDxpi0PBW3Fb\/VQg\/YZXHij\/ivfGNb1zhWvW\/BohMcl+9z4c85CF993zWXnvtquHlvZB4mDbeeOM86cpN13qeAuiojNGTUY66HJmmcogP5MlwB8jLM+5n40rXubK5+tE7ysuh9xg9J25h7MiedXtoF3TNbUJ\/thMvIB7bxf0wV6SBmMS9973vnZ7+9KdzVj8A2nl108cfetAnF6VvXoYI498kq4xkXJWfHdwHEFp9+o2d2XuzzTZL6623Xog2XvOy3v72t083uclNKjk61vOTNz+6VEJd\/ggnRz4XadI1Dx\/UPXXEykB+nfwKBTQihjHci4fkqKOOSmT5xxBEpUcPIIaAKvc1r3kNsWpOSBwPnDiVZ\/ZHpUUDlBe5SFNFvv71r68IPYvS6FxrrbXSkiVLUsyzeqDNr+qh+KXNIwlDwPzoLj\/5hn7RkxmkHNIKyDPK\/PnPf74ifGHylDd3DlMDuRw9PvaxjyW9j1yul3sQXUexuR5TTKsgj2gvfhyb9JuNNtOUTy8\/9YuYyOywww5JW1dH7rvVibA6lFs9aSvqN9JoGnILI0NWHHGl58WTNhA\/+k1ydKUzebLicPtBqLeJvA7UhfYsvnTf\/va3V21Ib9OzKQ2jDDoBN794Vk9bMTUVftKhg2e\/X12lNQqmjlh7GSMajQeXmyzjMzi3it5vv\/0q8kCEegAqROUKNzw0TOQWR1zuQAzdNYQIk6ZGRCZIjrsXTF3ojfzhD39IhqFeYiFZv9rSy+O6R1wal6v7PFw5pDFIOfL45sGizHoYehp5+Gy4B9F1FJub33zLW94yowh+HD3YoHdEYLbajLQHQU4MSFXdaCvS6Ld9kTWCUn5uaahnbm2d\/bkDwsi4b2oDb37zm6s5YmQWcmQDUUemKujo+aB7hLdd6aROyGnL2jRyf9rTnsarAjc\/snX9K4Hr\/\/Sn6\/XCI1ymjljj1wnRBPSa2IjR9V5VDtLiF1MD3IDU9Ba5o0FwqzRh3IF63JCXDz08mOCXWBwNQt7cbZA2+ZzczTfd9KY3bYyKAOQViDzrwv2UI49Dj7hH2nSI+9m+9qPrqDb3AxntJH4M83JpL2yr3majzeR5tbmRez4NECSm5yputBfuNkSckMvrOfzimod1awNGDtH2XJuG29oyHf0QhO6Rh6vnNOrCFUF6joTppeqtRn1LRx7yAm5+ZEOGuwn96NoUbxC\/qSPWpsKrHBCmYf7ud7\/j7AttldRXIkMIRWOWP0ii\/jDwA0NXBMAdUxD5kI1\/pME96ZgvXaM346HO7WeI\/be\/\/a1vs82W\/ojGjzZF9OAQCuQ\/onQ15CYzV\/DDEzr4cWI\/Uwd6pqEDnejmx1LvMvx7XZF4jPaQJmLuJd9PWD+69pNOm8yiINa6EW5zm9tUc5j86w9BvWcSBKdihYkTqMcNfyTuIdDA6tBDCrleV0MlDdMPQTTIeo9ZfL\/isWTIL745Tv50pTM3DFMO8WYDpjVMb3RLexhdh7G5HyTE5A2y3mCuT33Iq4fVNJIRh63z3uww+kunFxBTzE\/2kjPUNuTuJSOs3nbjHvE1tTNxmvCb3\/wmxbSaH3U\/TuRym7hHtGAqiW35jQLPhvTqz5f7eAbq6avjHrrWxUe6XxTEqgKApQxDVK55I\/d++Q0NuDVeL5gQkgbm5UYQnHAT4yqHWxxxuQPRo5QXhH+vBzhk6ldvQM2p6qFIy0Pd1uDj4VCO+kM4SDnquozrPghHmfzwSJc9o7G7h0F0HcXmuT71umRzoI\/pj6VLl6bZaDPS7wfIEmmSjV4hEgkgNWHabj89uxhai+PH2T13P+2MXBOi\/QmLtarcEHPDD3jAA6qXUPzakLfjeB49k9zaUF5neqJ+JL3cVZ62tHvp2ha3n\/CpI1YPgweTkQMxTFEhMQx58pOfXO2sYSThZMUTn99zn\/vcZHJfT8VLK34qU8+IrDj8cuiNavT8hJODaAAeTOkJb4NhkAc65PwgNMX16++HgpzpAPnl5eAP4sqfu60cZMYFdrBEyUMSRCZt\/nRlT\/rwC9B1Lmyet4GwHZ2AfvTRMwrSyuWFk8ttnbeZcdsaWSJNOsWPCXcgCMe9Hyo\/WNzdIC3zksrg6p4su7M\/dz8w+tM2yWrn0gNufqDujbw8f\/Tk14SwqfiQ2zZetnkm2Vn8vM64+YUcd8AzLS0\/5m26RpxRr1NHrN0M4gHWOFUMGcTlLXoQIb9APqTmhzD5cQc8bBD3cTUUqstqUObsug1RIm79mj9AOSnlcsphKQoCCH9l0pjCT+9V45Z\/XTeyTeWItIa5srEGXo\/bZEf513USr0mWniA8x7A2ZzttoCl\/6csrf9Md8nQWnkMa7Bt+3Pzi3lU8aXIPAnVnOkgcdYokuHPkP7B+qJBIHl5304M+4R9tlN3Dr9\/rgQceuLKTIo5nTftzdX\/00UcnOmkT2ga\/QcCO6jjisK3nic7h51qX04mqy9R1pWOuaz8\/SvJqw9QQK8PHsKjp6gHyYNQN0hSvqXHxy9NVucAvf\/ikX5e1ZbGtQQknJz3x6+nIi1+TnB4GHcQFZcr98rJLm0yAbD4s6pYHfyAvbqRJL\/fylycZCDlhIcu\/Kf\/wU37lIwfhLw2QF3DX86vL1tOSXjfU40of5NUUJy8bOZBGXZafsIB4g9g60tNu2VA69XI3yZCTt\/y4xZVGyMY1wsnU7aUe+AmTVsRxjXiRrrS5yQK3tuDq\/rvf\/W61HEs88XPk+ZBtQj1\/8Zvi1eXqMsLbdO1mX3kOgrET67\/+9a9qh9Fxxx2XTjrppGqR+yAKDSv7j3\/8Ix122GFJV98wYvvtt09f+9rX0n\/+859hk5yqeOadzD+xTQzNFZB\/vPxiOw8E\/4LhLcCmxdbD228aYg5NrIjsyCOPTA996EPTTjvtVJHYH\/\/4x\/TMZz4zmbN54QtfmPzib7vttukNb3hDIj+bBjv22GPT2972tvSkJz0pnXDCCWnLLbdMz3\/+89MZZ5yRyr9UzRcbirGFoY\/hJJJVV+bXDJkMnYQXjGYBPaVi69FsOKux5yDx1YbNA5EddNBBydvKiy++OO21117JW\/MzzzwzPf7xj6\/IDfFut9126UMf+lAiP2xebfGsSzWPI98XvehF6c53vnMyCX+f+9wnfepTn0r\/\/Oc\/25JYFOGGYubW6oU1b6fXihDqYeV+OAsUWw9nt2mJNRSxmkw31Dd0PPvss9MFF1xQnXXqTaC3oZYsIbdtttkmvec970kPe9jD0oknnpj+8pe\/zIrdLr300mrKwRxKp9Op8rjxjW+c7nWve6Vzzz13QR03Vik\/i3+MIurzWOOaV5pFtRdk0pNma9M86lr9021BGnWBKD0UsVrcbZH3tiuG+be85S3T\/\/t\/\/y\/tuuuu1fo0ZNrpXEdubCDMcqDLL7981oiVLje84Q1XOdtR3ne5y13Sn\/70p\/Tb3\/7W7YKFxef94qc\/\/Wn6aZ8477zz0nk90Cud0GfBGrUovsgsMLfFHYpYm1RcbbXVUqfzX0Jtkpktv1\/96lcVqVtTN2geH\/nIR9IgeMc73pHGAdMo\/WKQ\/AYpiyUrvdArrdApyhD3ruJJ95hjjkng\/v3vf39ltyOOOCIdcT0sGocgd\/U4aP0V+WKBSbTAapOo1Fzq1KtX1hQWPbVRr3NZxrnIK7cHuyHLCy+8MIF7pEnm8hUjl4suuqg6NR\/pQpCzaSRz8nD\/+98\/wX3ve99qpcc97nGPFNh0000TOJvWC7iC7t96W6y2mYs23yuPqSBWb7QdkmFVQr2wpghucIMb1L1X3psvto1vHLDtcVgsW7YsLRsA9rcHTHnAkiVL0rXXXlsBmSG1cUO6ORAmIE2Q\/0rjdnGsscYa1VkNS5cuTUtXIMptasmCbfc2RICzaa09vMUtbpECtvaCLb93utOdqhUgVoFY4fCc5zwnvepVr6rw3ve+N5n390I1rfhnbrHgsmpN6TTbYUVVx\/95u45ErF5G2ToH3sxbM2pO030OcrNZQg\/X3\/\/+9+TBzvNBKh5AD2fun7s\/\/elPJysJxgHLlYYFUqgDWYDyrb766gmUCQy1Da\/19uyThs997nPV4d2WUyE\/hDcIwi7shdzAi8e8TGEnLyjh1a9+dbLP3FI3CD\/+ZMVVrvgRzAyosAAAEABJREFUQKQQedWvTaSbE6649CMXcaOMesmmFkw9gGV+L37xi6s9\/myoJ6xXzE9YPiUhLkSa5VosMIoFRiJWb\/z1MGD33XevTsd34Ij7HORGUbItrhUIPmdi2x9yJ68nqqdiyZUXbPwmAXUSQAQecA+6Bx4e97jHpe22264ihJwIyJAHJCCttjIhIdCj3WGHHRK8\/OUvT4AIwTY+mylcgZ9wWL58eRWHjcVFtiDNQF0HS\/D46UEjQvLbrnjRCYgWEK8tlcg3iJesOOLWgUiFkVGWrbbaqtpG6coPhNfj1e\/ZjO3YEPkCu7I7sDuwO\/ATro5APPGlU097mPuw1TBxu8UZNc1B4vcr2yY3bHhbvG42mm3\/oYjV0PvAAw9M73rXu\/oGefGGLZBe2CMe8Yjqc7kbbbRR2m233VIY9ba3vW3aeeedkx6ctZj2TXvgfvjDH6Zddtll1l+qecjAAwcePg+hBxI8nB7SpgdWuIdbHHHBUWxtdgpSW7ZsWUJ4gAgBMSLInCz1KoUBWRAXpNWWH5u2yUR4v7LqChnSIUgX0SJd1yBc4eQi\/bgiW1i6YjoByElnqxWkyx1XbcPhH\/wg4ve6qk9QH+pGHYH66lWf6h3EERek0y2vfm3VLX6T\/6hpDhK\/X9k2uWHDm+I12WSu\/YYiVmtEkYQPhfUL8uINU0B7je2iWnfdddMXv\/jFikAN+\/n9+te\/rpJ0zJ7eKlihYJmXeVdylcCAfzwM4MEAD4oHxoMF3R4u\/sI9hOKA+NJqUwHBgYcf8YHeHTJEllAnTH7CgTyIL522\/CY5HJEqB6JEsIi2TrjItKkMyFb8uGoDRi3kpSWdww8\/vDpHlD3DhmwN8oV+bKheQR2ra\/UO2oC2AN1+VLUnEE98kFZTmYrfwrLAUMTqPACEls+jtrnJizeoeRClwxzueMc7VltjvQ1+0IMelF73uteln\/\/858mJVeZ3P\/GJT1TbWW1WsGHANIA3yr12XmnEHkAPAXR7CPgL98B4CGCQh8ADCh5WxAeIEDzUkPcu3QuD5cuvG4qLC9JJi\/if+mKHbVdMLSBBJIl0uc3jCkOgdROZGkKwp512WgJ1aYRjVYLDRpCwNNmb\/QHpNtWLvEA9yg\/q+TXda2+g7WhDdABtSxuDJhLmTwYJg7jSkFZTPsVv\/i0wFLFacP\/Yxz525RyXIVcbyIs3aJG9+LIBQOPV64j4tmG617gQKZlBd14hKW+aNVQYpLGKC\/TygIGHDRl6KKHbg0mGPIgPUS7XmOLgHhdGTXOQ+P3Ktsn1G45skSli1CN1mHIQLrIFLzGbbIlsvegLsj3ooIOq9bbW45566qnp1BWw0iGva+QL6lE9AwIGdQ78hGsTEHUtnSY96n7aNWiToH0iYUCyyDZI2GiQG\/i\/8pWvTAgYxBMfpFfPp+m+ze55nH5l2+SGDV8ZL1dqAtxDEeutb33rag++eU9QeebLnOjjTEQHn5x++unJfmn+SFclizdomX08793vfnd66UtfukpU5zv+\/ve\/r3Z9IVXLqm51q1utIuMlh1UK\/RK6Rg+IzoMAHg7woICHpukBIrN8eeldrlIB83yDcLdd0bMFqxuCbBEdslXPCLmuJrIFZAvILAiXG9kiKoRcj6v9gLS1H20CtI+8\/UQbckAzf+H0AvHEh3r63e6RJtDLKI6eECSMcBFvnYSFg+cTPMvWGUurW17Fv90CQxGrdaFeGNl7jMzMV+n5OXRFr9HJ99YYeut71FFHpX\/\/+9\/pox\/9aDL32a7SqhKdTqfaqmotY4SYAnjta1+bzNlaRK4RCB9m55Vv8\/QiS40cNHLw0IQes3WdjQn5UdMcJH6\/sm1yw4b3iodskSmy1btFZEG4yBaEN9UtskWmyNZyN8QVhOse4Qpvitvkpy05b0O70sYQMCBZZAsIGLRR4Cec3iCe+CC9pnzqfp4XQMKIFJQFkCwECTcRMQIG8awdl4706vnk973qhNyw4W3xpD0fGIpYc0X1Gh3Ecr\/73S81vfXnpxHr2ZLN4w7qdvSgM1cf\/vCHpyuvvLI6JtDum0HTyeVjmJf7FffiswDC1U4BYQXZciNbxNWLcJELwkVOQbbcyBYGIdwm6yNNoAcyRcCAZJEtIF4kDNz8gIxygLjSgKZ8mvyQJigjMlUuQMB6wtAvEUsDpNeU12B+kys9MrF6ueRYPi8HuhVTuLlS124y\/H2gL9+CZ00sfzCX4ljAQw45pNrqaEcNMheGvK0A8OvpPocpAj3s3K\/ulqf0JwWmUcaty6hpDhK\/X9k2uWHDm+IZ5QxqUx0B7dvmAufXmrt96lOfmozELN8y9dXP3G1OuG95y1tSbObQ3h3XOKhebfLK73kwigRrkP04gGdozz33TOCTPWA6AhDwM57xjGoNtd48mE6DQUaDSBMQaD9EbN32ox\/96ORbVuaHfWoIjHbFB6uBmsqtrLm\/Z7n+fM\/H\/cjEairAr5+3q9\/5zndmlMFa0k9+8pNpk002qbYkzhDIPAzr9WwDtiYKNofqQ2PmSk0p2HBgqkEYaPjD7rwS3\/Y+Q4pJATuMW5dR0xwkfr+ybXLDhjfFs1RvXDb13sCUl\/Wxjt+r924RWLferQ6I6ScrEcyFWr1iCu0LX\/hC8p0qZyl0Op00iq5N5e+VnukI0Jv1wUTP3V577VWdsWz9M3iG9YRBbxh69Yb1rj1b\/cC6bdMsdhSyiWkVcI6zXjG89a1vTU9\/+tMrvOIVr0jwzne+M7GfHy4wN6zDZYVHP\/nOpszIxKpHqMDmTy3a98uj0MDtV89w26+RLZm9CiPcvG3g5je\/edJjMPwXT2XqpXY6q56i5Rd5oey8Uo6C6bOAqQRkaioBDLuDcD0DQbbkmkqPWPTwEIRh9mxMJzTlO4wf0gQdKmRsSgL0eJEteFaRMHADfzJsA+JKA6TXpy5JbxjYS2+WvQABm5boNoroN\/1xyA1FrNajGmqEAoZEhjQ+y2LpkzWl8KMf\/Sgxnl8RMiE\/yNWvme6+HoJF\/\/X1snoAXqTZYeUX7oMf\/GD1za399tsv+XCbZV6dzqpEPEj+8yFraDPufEdNc5D4\/cq2yQ0b3hZv3LbtlR4iRRxBttbcBpCt5wIhN6WBbM3NIltAHv0Q7qjlHyR+P7JIE1+wAz5AwoBkkS0cfPDB1eeduhGxHydxpQHSbLIZv7YpRzKzjaGI1ZB8xx13THqkpgAs\/te7ZCjDGeT64x\/\/OOna2\/aaD9sHLZC0NTCLubfa6r\/7w8NtHkaavnVlQ4A5WN\/g8iHD3XffPTlkW3hBscCkWADZArJFrHpv0bvlRrgwDOEaQptm0Jvz3ExKmQfRA2kCAt1hhx0SEkas+AUJgx4wEjZHzA38yXifM0h+syE7FLFaj2oCfosttqjWqiI5HxW0htVLAoq2vTAi0w\/stDJ3ax4UzEXJ3wYBc7HmuKTz2c9+Nn3zm99MPmLoMzB6sEjdmlrhCwnrrbfe2NUdNc1B4vcr2yY3bHhbvLEbd4QEc12R7dKlSxPCBSRbJ1xkQyY1\/EOkCBWxIlibHKKH677f1Qm5Tg3ZrOLVr2ybXFt4t3D+SBjYBhGzwypKzsPNUMSKNB2EYpLbUivDlJ133jmZhPfmVM\/RfIdeq2mDcZXLcMKEtd5xnqYtreVjgrlFinuaLJATrp5bTrju894t2XrZEQ3C9Zzm0wmIF9mCKYd6vHI\/vAWGItY8u06nk7xxfd7znpf0Ds8666yq13jKKackvx56tfvvv39ykMqoJGuu1vrVZz3rWbkKydSDlQPmYTudThVm88BC\/ZhgP\/NWVSEH+DNqmoPE71e2TW7Y8LZ4A5ht1kVH0RWJ6qXlvVukazlYnXDrBUG2gGwhJ1yjUWQLbYTbr\/5tcsOGt8Wrl3uu7kcm1rqiXiSZ7zTH6VfSnKfPY1uTam62Lt\/vvTTMs5pDqa+pQ6pWJ1j6ladn\/d0gW1rzuMVdLLAQLYBs7UKsE64XZkhX7xZ6TSdYDoZsISdcbmTruW4j3LHYbgEnMnZizW1h4b5dUk6i8rKr07muN5nL9OO248p8aUwz1ONYeqEx1Qm3LrdQ7s0bjVvXUdMcJH6\/sm1yw4a3xRu3bUdJbzZ0bUoT4SJTvVtAsvX52zbCRabI1nwtkjV\/652H+zbCbdIpt9uw4W3x8jzm0j1rxKoX6ZtD5ltNCfhsiDWpwxTOrgvDfdMNeqbDpNErjt0ahhQFP6kOD59GO3ipOo3lGrVM+e4yG210XmJ3mYNrrFrYYIMNUre1oaYT9GDrhGuHmRU7nnsvmUfVs9\/4nuVez\/pchY2VWK+55prq1CsrBFSQ060Y3mHY3N7m9yqYLX4MEzB9wKB2Wnnb73CXpvh6xl5sDbul1WoDv3yTAmUcty6jpjlI\/H5l2+SGDW+K5z3AuG06jvSadB013VHTFN\/usq222ip5KW03ltU30cPN52+NFMnXYX25zpAVClbpeLGtdwt2l0Gn898dZuJfV+71GneddQuv+3uW+c03RiZWw3SE6FfuHve4R9pnn32qhfl6p\/vuu29iWMN4W+asJuhVYFvx\/LoFrDpQOcjV1wKCcG1vdWygike+fmlH2dLaS6cSVixQLPBfCyxZsiQtW7asWhJmOsGLMvO3kBPuf2P816WTBXq3ENMJVicgX9MJYMrhv7EWpmsoYvV231t+pOaXDdHZ4+sX5zWveU1yjKBeJCK0TbVf05CN7ayullXVyRbp6v3qvX7pS19KyHfatrSyY78261du1DQHid+vbJvcsOFt8fq12VzIzYauo6Y5SHyyS5YsSUtWICfc6N0iXHO3sHTp0kaTItu2F2ZGo02EK\/\/GROfZcyhi9XZ\/r732Shbl+2QKMkV4VgJYEVB\/Oz9KGetkG4Sr9+tlFfK1EsGGgGnZ0jqKvUrcYoFJsQCyBS\/LQA83CJcb2cLSpUsbVUa4yFTv1guyvIfrPnq35BoTmEfPoYg19DXc\/+tf\/5omYQvZ8uXLkwMY7P4yL2QKwWaCu971rqHugrma+hi3sqOmOUj8fmXb5IYNb4q3\/fbbj9ukY0mvSddREx41zUHi9yubyyFbvddtr\/+6A5I1lah3y41s9X7JNNkCkcYLM2RrKiGAbMVtijeXfkMRq17jcccdl5zhqEdprar5Ti+tfLxPt302C2EjgF8xekQ+DmixceCcc86pNgwcf\/zx1Te5IrzX1ZTFJAEJjFufUdMcJH6\/sm1yw4Y3xVP\/47bpONJr0nXUdEdNc5D4\/cq2yQn3jsaHQp2Wp5NkORfi\/PSnP119NNT7Gksr1WUdyBbwgpUM3HWZubwfilgp6G2gk6PssHL6lANyLd3w8sqvjxUC3tB5qUR+UkHHgstSsUGxwVy3gX7zc7CTaUY7L50NG9MJerh6t5D3bvWIYT45Z2hizZV2epXzV82zOu3Kr86NbnSjpCfr18NLLjunTB3k8Yq7WKBYoFhgGAsgTsinE4JwrU4YJs1xxhkLsYZCXhikhU8AABAASURBVChZVuVM1G9\/+9vVVIGXW15yGaY7AjBky7VYoFigWGDcFkC2ee913On3m95YiTXPNKYKdOG\/\/vWvJ4uMzYPmMsVdLFAssMgssEiKO2vEmtvPVIEe6y1vecvcu7iLBYoFigWm0gJzQqxTablSqGKBYoFigS4WKMTaxTDFu1igWGBcFlh86RRiXXx1XkpcLFAsMMsWKMQ6ywYuyRcLFAssPgsUYm2pcx8ydNBLi1gJXsAWcMTdvvvum7bccst0z3ves1om6NS2BVykSVB9InXwaafddtutquv73e9+6bDDDkuzUdeFWLtU\/29+85t08MEHJ8cVOg+hi1jxngIL2J6tGL7ZZq\/5BRdckM4880xeBVNmAUTqAG+bmRzkYimoj56Ou5hTT6yOGLTzq244v1KMbEODvdn2Kn\/ta19LsTvMZgcfI3z0ox9dj1ruJ9QCw9a1E5R8Qsg6a8dd+lbaj370owktZVGLBYata+fH2pnV6XSS7faed3UuzXFiqonViTonnHBCo72OPfbY9La3vS055pCMYaDeqV6LCNbcOlTG0NB9wWRbYJS69oFK+82V0NZrP7CTcEISfRYa5kLfUera+dG22z\/xiU9MvnKy6aabprXWWmvsak8lsdpC67Qtp2CphLrVfve736Wjjz46Pf7xj08OcnBQ9gEHHJDuc5\/7VJ+W+ec\/\/1mPUu4n1ALjqmuf9nHkpOMn99xzz2oObkKLvGjVGldd2xX6sY99LHl\/4oAXJ2KN26hTSazOYEWUb3rTm9KGG244w2bOavWxQ8Tb6Vz35dgb3\/jGydD\/3HPPTVdfffWMOMVjMi0wjrr2QkMPxgcHzbfutNNOqdO5rl1MZqkXp1aj1vXPfvaz5LComOYxKtUBu+SSS8Zu0Kkk1o022ij5gKGhvK8MpNo\/pOprr\/UvHZhb+9Of\/pR8IaEWpdxOqAXGUdemhbbZZpvk\/E8P24QWdXLVmiPNRq1r50QbmfzgBz+o3qU45tSB2d6zjLsIU0msbUZyWK7hQBPptsUt4QvLAm11fe211yajFMddmmc17wamihZWSYu2bXWtM7X33nsn57puvfXWaccdd0xGrQ7YHrf1FiWxDmJEhvcGcpA4RXbhWGCNNdZIRx55ZDr77LOTpTcBLzUXTimKpv1awEdIza9aTucDqE7d63TGP+2zKInV8gpDAkODeoX4VbPUqu5f7hemBUpdz1a9TV66k1TXi5JY73SnO1Vr2OrfxbnwwgvTmmuumWZj+cXkNcPFoVGp68VRz0o5SXW9KInV8qq11167GvpZIKxSbGs0PLDkqrzAYJHpQKnr6ajHfkoxSXW9KIn1tre9bdpll13Shz70oeQzMueff37ab7\/9kmUXPpDY6Yx\/zqWfhlFkxm+BUtdjtelEJzZJdb0oiVXrsBD8xS9+cbIofOedd07Wtr7zne9M1soJL5geCyxfvjyVup6e+uxVkkmp66kmVkuqvAFseqtvX7jPxZxzzjkVqR5\/\/PHJYuFelVbCJtcCpa4nt27GrdlCqOupJtZxV2hJr1hgMVqglHlwCxRiHdxmJUaxQLFAsUBPCxRi7WmeElgsUCxQLDC4BQqxDm6zEqNYYPotUEo4kgUKsY5kvhK5WKBYoFhgpgUKsc60SfEpFigWKBYYyQKFWEcyX4lcLDBNFihlGZcFCrGOy5IlnWKBYoFigestUIj1ekOUS7FAsUCxwLgsUIh1XJYs6RQLLEwLFK1nwQKFWGfBqCXJYoFigcVtgUKsi7v+S+mLBYoFZsECfRHr3\/\/+93TGGWekQw45JO27777JFy1d3X\/ta19L11xzzSyoVpIsFigWmC0LlHRn1wI9ifWyyy5Lz3ve86qj9J761KdWR+wde+yx6Vvf+lZydeTeM57xjORjXLvvvnu6+OKLZ1fbknqxQLFAscACsEAjsf7pT39Kr371q9PDH\/7w9Itf\/CIdeOCB6ZRTTkkXXXRRQrYB96effnp67Wtfm373u9+lnXbaqYq3AMpdVCwWKBYoFpg1C8wg1l\/\/+tdp1113TX\/961\/TySefnD7\/+c8nX6xcb731kjNMc03c3\/72t6\/kyZEXL5cp7mKBYoEJsUBRY84sMINYlyxZko4++uj0pje9Kd3hDncYSBHy4g0UqQgXCxQLFAtMmQVmEKte6K1udaspK2YpTrFAsUCxwNxZYAax5llfffXV6YEPfGBaf\/31W7HFFlukvffeu5qDzdMo7mKBYoH5tEDJez4s0JNYb3azm6X9998\/3fve906+M\/PIRz6yepFlqZXvQ93gBjdI66yzTnr84x+fttlmm3TiiSdW863zUZCSZ7FAsUCxwKRYoCex3vjGN06XX355+uMf\/5hOOOGE9K53vStZdvXsZz87+UjfcccdV73Q2mCDDaqwr371q2ndddedlLIVPYoFigWKBebFAj2JFaH6eumOO+6YvP2va7jxxhunXXbZJZEhe5vb3CZtv\/32dbFyXyxQLDC3Fii5zbMFehKrHVfWp97whjfsqqYwMmQJuXctKBYoFigWWKwW6Emst7jFLdJGG22UTj311PT73\/9+ho1sZT3zzDOr3uxNbnKT9I9\/\/CNdeumlM+SKR7FAsUCxwGKyQE9iXX311dMLXvCCiiztwnrDG96QzKvCRz7ykWSK4Oyzz05PecpT0r\/\/\/e\/0zGc+s9pQsJgMWMpaLDApFih6TI4FVmtTZdmyZenwww9Pa6+9dnVWwAtf+MIEBx10UNWLfeUrX5l22GGHdO211yZbYS25akuzhBcLFAsUC0yzBVqJVeHvete7ps9+9rPpe9\/7XnUAi0NYzjrrrPTd7343PfnJT06dTifd7na3q2Se9axnibJg8bKXvaznml0ne\/35z38ea\/nOP\/\/8tPnmm1f52hY8SuJta4\/r+kd56\/6j6DBXcQ899NDKZvk667kqR+RtnTeb91vmsDedm+JKi79wefSbbsiNsy1FmnHV7tmXbh\/+8IdXtln3TRi1LUe+C\/HaF7H+61\/\/SirssMMOS29\/+9uTyv\/nP\/+Zfvazn6X\/\/Oc\/C7HcQ+vsR8V6XjYYOpF5jLjQ9We6eMDf+MY3ul0Fynf3u9+9OudilYAJuNFmzjnnnJWaXHHFFen73\/\/+yvuhHHMY6Sc\/+Un6wQ9+UG11t8SyLWvLMof5cYgfHySurtvymcTwVmL1gupFL3pRetSjHlWtVf3EJz6RfvWrX1UGfvCDH5wOOOCA6qVVmrJ\/W221VdXo4yQvV8ckKqYH4jOf+QznROMxj3lMtROO7rDPPvtU+i4U\/StlG\/4gTxCkTpQNHARkUwv\/17zmNVUHgHs2sMcee1S2Pe2006rRWj95aDNsn8t++ctfzm8n2u29Cj54whOekPJt73kdqAc\/Fp4fhfn4xz8+q\/Ugj0lEK7E6kEWXXkM95phjUjRchvPLorEcccQRk1i2seu05pprrix\/PXG\/zPXhELvV5fwC+yUOWcO+q666qi5W3RslxBQBeW5+VeAQf7beeuuV+l9yySUzUrjgggtWGd7Rk765oDqnS46mcjbZg1+eFre4eVrsoWcnrBuCjLRBCLm73e1u6bnPfW51+4c\/\/CHldpWmtPO8msonMj1zOfdR7ojDj4w0pS1eL7Dj17\/+9Upks802S+BGD7YtvnD5yE++oYt7YENpNaFNVnvSrqQTiDLm6dGBrp5\/7SgPq7vt0rQjs+7vnq6Rj6tySVuYq3uc4t6PZz766FdXcecbPYnVon\/bVHfbbbfqqwFrrLHGSn0Zz6+27ay+IqDhrAycUsc3vvGNlV9L2HDDDVeWUuNtGpYaCgkLQQ3HNIIGE356MOT0BMLP1QNklJD7c\/PTOMmME3TyQMkj0uVHN3ULwqPRh4wrGfpygzI32YOfMDLALS53gD0e8pCHVFNP4dftalhqeJqHa5N6Td4HIFphHkhpStt9QPmQhPDwUw56xr2r+6ZyC+sX9KQveW0AuOmkh8fdD67TZdXREhvSux6ff11vftF+XLWnvM6lwS70017dAx3p6gdhvfXW49UV4iFhAjYWWYrJPUp9D6KrvOYbPYnVon+L\/9daa63qBVVd2U6nk4Q5DHvazmHVuPxa+lUNaNRsYEhtGoRbhUfj5e+hBsNw4cLIcIuvcXLH8Ek+jlvkF\/Cgv+9976tupSM94OZp9KDxcg+Cbj8MkUaT\/vSDnBia5PTGkC\/dv\/KVr1RJRhlz3T1wdGcTtiEY6Xl49UA96K9\/\/euT9ITXYRUKP3KIIeqniVykIS2y7Kws9IlpA\/7CydE97E4P+pCln\/xGQdg+enwInVua0QPnboM4dKeXsiiTOE1DbmFkyIojLlkvnpQ38m2S006VnzxZcbif\/vSnV+eGcAeQddSBK9uJL13vZHTC+qlvBGxqJdq5dOjgWetX19Bpvq89idU6VnMpDNtNUWE3velN02LacRWNWNmjwWkEVkiEnTyMGpZ7jQKZIBX3Go7Gwm01hXlq7oAhrAfeg\/C0pz0tvBM3P41Wg1sZ0MWBuDT0AGInSi86cAfqfkFeEa73pxfoIdUrDP+4Nv24euA8UGTe\/OY3rzInGVMReb4eQA8ueb07ZM5dB9uxb91f+aKskS9iAbLszN7cyhPTBpFXTn777bffSgJRr+pXvGGgnfjhETd6fHp93Py0C+2Duw163nQnpyzmO7mb2oQwMsLz\/NxDvU745Yg6Ug9spK50NnKZXm46aYNkIi1pRNvrt77F71dXspOAnsRq55U5D3Or3\/nOd1bR12oAvy7CNLpb3vKWq4Qv9BtlQl6IJKAHplwajIdYLx2h8DOnpKFwg19fwyDuK6+8svoiAzfk0wju63O30QiRa94j4+YnTshwDwLlssEjHriIS1c6x31dJ\/4IwnRAkJdrPDjCQbwlS5ZwVkCu5MBQsPJc8Sf0Z0s6CQfyK4KrKRc\/MNxNQO7qRR2JX5eRDnKNfPwg0S2Xi3pg0zwv+uey6lX95nEHcQcxifOABzygImxpcvNjA+Xg7oEqqP6DF2WoAmt\/8jD5NZWBjdg9kLexSC5+cJxyV283ZDwX6iKgLFEnRgBGAlEPyios8lNP0qjXAb86+tG1Hme+7nsSK6V8LNA6VnOpy5cvr4Znr3jFK1Z+QPC2t71tItPpdIhPNTQIUEi9DJ+x4e4HeiReqPQjOy4ZPYNo7HF1KpmHbNA86G\/eTe8PSRlaSrPec\/TgIe6wU54PEvZDLa3cfxS3sigTXcBDHukZKYS7n2s8\/P3IDiITxCSOH+QgFW5+MKiu4owKUydBbNFW1K9eZaTtx1RvW50bMYV\/r6s6iZEHwlT+XvL9hPWjaz\/pzJVMK7Ey6Hvf+97qUy3mU31hwLkBpgicy2r5ld7OXCk8SfmYAomy65VqhKFfvTe7dOnSpCckvP4A6y1pgMJyaOAaOsKoQ48tl51tN1LU26CTOdQYjtbLQg8PVk52Hlr+IA09Gm5AwO7r5XNvyE8mBxtHr9nVfR7+jL+yAAAQAElEQVRuqErH8IteG\/uyc\/i7hu7auB5qyPoBzGXloX7FGRR+REwdtcXzQ022TS50Drm4jzKEf9v1N7\/5TQq9\/Dgaaouj3MrPDdofmLYwncBvFAxa3\/Jil350JTspaCVWijqX1cOhJ3LhhRemH\/\/4x9VXW\/3a3fzmNyeyKKCBgcIaFiHL+GXmf9RRRwmqoDeCRNwYvunJieMeSRnWcGs0XkZxB+LFhvhkw9+vtt6O5TGGV+E\/l1cPnYdPnnRAstwB5aIjcPP30OrhevjdA5u4shtwg+kCcbv1bJF2DGnFA\/EC7MVu7hGlBxncszN7c9PdMJU7SCPsjoTjhZZw9VrPh38\/8KMR+iAwPxg5PFfSIUOWO0fdTWe683d1z+1H248D96AIchYv1qpyQ\/SkTVuwPb82+CGKdw\/qnF2Hre96Xr10rcvO5\/0MYrXf32Erhvv9grx481mQceftQdL78ZAH\/JDIR2OJYZFeVTwcyDRkPeBkhZHh9mBFb0paZD30HirhAb3BeLGSp8lNJn+B4X4u4OFQbqRjHo7uru7z\/NmsXsZcVnmBTdhG3LAFubBb\/uKFTI4mO4oLYSN5eOmEDLyIojs78ycXuvMXTo7d2VZeef1HmvwHRRCTfNiwHj8Ihz8yMtLh7gb2pnteBrL5izn3bXB2cv5DLz0I+4uPIPWku+lOBvL6k4Y2wH7C2JNdh6lvaUjLD06brvKaJMwgVkusnAFwxhlnpMAXv\/jFZMjP6Keffnoy5\/LJT36y8vNlAfLiTVLBZksXD6Y5I40l8tAj87DHfVz5CYt7vVZ2C+LhLz3DZo3XfcBQv97LE2YOMU+T31xAed\/ylreskhVdgCfS0j6U0chGufjnQKTKisT4K0fEdw\/soNzK774J8vDilH2bwqWZ50N3Pevc7uK55y\/cPdCpnq57ugsfBHrHiEmc6BVz50Ac9ODnzbsvdnB3Q12XsBfi6hanm\/+BBx6Y8nriRmau4tgcpF6DHPkNAvXAnhGHm1\/cu4b+eX3rtPAXHmjT1VQBe4f8fF9nEKuXUUgUeYLhk6kAQyMVj1gR7g9\/+MN08MEHJ2+SzbWKN9+FGUf+Kj8fqtXd+QOb56dh1GX55TLcQQohKz0NOZYy5Q+IBz78Qz4Pl14defrKUg+v35ORNj2C8MjkeUeermQD7iHuo7zSkV74x1Ve0s6RxyenvPLOZbq55SdOHdKsx8ntEvLImX9dtp6unm99jjVkuqUhTWkLlx97sAv\/HLmMspsCOe2006qlafLIZcPNjtIEcXJ7cfMTVrdDxAtd6MNNFrjp4+peh8lVvMg7rnk+ZJpQz19cfrksXaUlLOCef8iJ06YrO9M90pjv6wxizRUyFHj\/+99ffX7FqgAfD4xwL7F23XXX5Nfs3e9+d1osPdYof7lOnwXMCRvKgrneKKFeHLgfZK6RfMHitEBPYjXfY52m9aydzszlVJ3OdTuvvMyy\/XVxmrCUelosYOQAymNaA8GCOUR+huzDTAmIW7C4LNCTWO2msqTIFIDPrtRN4+hAc0JkyNbDy32xwEKyQAw3m8gT4Zo7nr3h5kKyVNG1zQI9idVa1Yc+9KHp+OOPT+Za8wXx3Pw+9alPpe233z5N286rNsOV8Om1gDnFmN+Lq3lHxDsfpUbm5hDp0m3edT70Knl2t0BPYhXNd6ye+tSnVouJt9xyy5UntnNb37fzzjun5zznOY2HtIg\/6bAc5rzzzkvn1eDM2RyTXo6iX7FAscDkWKCVWP1Kv\/rVr06nnnpqevGLX5y8xDJUMu\/0uc99LlnjR2ZyijSYJtbgKlcd1lHm2G677VKOPIw7jy\/NwBFHHJGOyIDIISdyBD6Y1kV6SixQijGlFphBrA5X8SmWenlt3fQRwUMOOSQZKlliZVlEpzPzpVY97jTeI8McOVEizoCdLDmCcHMiRsw5ad\/\/\/vdPcN\/73jeBxdFwj3vcozqjwdrHTTfdNMHGG2+8chThRUvB+sUe6xcbzDfnzCBWc6dPecpTkk0BTS+seilMXrxeMrMVJm\/f5EJAyMW8rwO4\/VD0yvO3v\/1tevnLX74SFifXYXdMjmXLlqVlGZyhEOiVV79hXgTCGmuskcCIAKzOgFvf+tbJ9kXwg3enO90pgekZu3LgVa96VQLnPHjDfeaZZ1ZrI83TLUaw\/WIs92Iss7qeb8wgVgv9X\/CCFyQH1Oo1eTB\/\/vOfp24EpXf705\/+NJEj\/8EPfnBeynTsscemt73tbelJT3pSshsMyTz\/+c+vdo+lHv+s1c1Jc\/ny5Wl5DTnxcssnh10fAWQeCL+4RhxpQE7goUMQ9jBErQcdPefoJesh6x1Hr9gV+McUhTg9TDQVQaeccsrElKMoMv0WmEGsnU4n3e9+90tf+tKXqjlV+5ctit5kk02q4Sl3YJtttkl3uctd0oMe9KB05JFHVvJ2bc212XzlwPY7878+fHjnO9+5+sjhfe5zn2TVgmVhc62T\/NZaa62UY9myZWnZCgSJ5gSOaAH57rnnntXLQoRcJ2nhQDaIWXrSBfn2AvIFUxVBvojXVATChSDdnHCdKdor3TysX9k2uWHD2+Llus63ezZ0HTXNQeL3K9smN2x4W7z5qt8ZxBqK2MZqZ9U3v\/nN5AHccccd041udKPkyEA9WFfbWU0bfPrTn072z5MXL9KYq+ull16abGSw9a3TuW7Olx73ute90rnnnrvgvhLZdDxbEDTyBGQaxIxkkS00EXGdgLvVC8KFIN2ccB3I028Pt0n\/pjzb5IYNb4vXpMt8+c2GrqOmOUj8fmXb5IYNb4s3X\/XalVhDIVtXDfG9sDKcclSZeRvXk046qZrH22KLLVK+3TXiztUVqZqTtO42z1Nv2qlb5lFz\/8Xgzom4TsA5+SLjIF6E3WQbZAt+YCEn3CBb4U1xi9\/1FiiXRWWBVmJdCNbwUHu54yi0haDvpOiIfJFpEC+SRbqmIKAfwkW0YAoBkC6yzacRJqW8RY9igbmywFQQ6yjGQi7mlB2yAXriAfM38wEniI0730HS\/Nvf\/pbASUtgzvf\/\/u\/\/qjXLPsNj1YHRQN3ufuAQKqJFsOZtH\/e4x1UvQr0MNcWQl6tNp2HDm+KZvsrznhR3k66j6jZqmoPE71e2TW7Y8Ho8K4Lq7XI+7qeCWJ3diAiaDoIxRdBrmuJhD3tYghNPPDGBpUkBL+Tq+MIXvpByOFoxh0N5wRkKOegGnU6n2qXW6XSqbcC2Aq+33nopx7rrrrvKfR42rHvUNMW3lM0RenvttVdycn30bs3x6t3q\/dYbsU+AOF8VvBTzxQXztbZo+kAlm3Qrkzy7hfHvFt7k3+QnjdnFqvXalNds6DVqmoPE71e2TW7Y8Ho805T1Njgf91NBrNZwOrbQZ0NyI\/qMjLWeeqW5\/yhueeSw1CyHHhsgjhyIBfTmAu94xzsSHHTQQSngHnGHjDhw6qmnplNXQNqRHz36LYuHul\/ZJrlu8dk2XqTlUwmIln9TWnq2yqdc0bM1jYB482mEbnlGmt3Cu\/lHvEm6zoauo6Y5SPx+Zdvkhg1vizdfdT0VxGp51dprr1192SDW21qfalG8JVd6hd0MbDWD4W4T9MByLF26NC3NsGTJkuoDgUtWXLulP6g\/soQgT0QKQdLICCkBEs4J2T1\/MkgYIp1B9RhFHtmat9WTjV4t0kW27NmUNrI1VUD\/nGxzwhXOFk3xi1+xwCRZoG9iRVTmIB2d9stf\/rI62FovcZyF8RmLhz\/84Y3Lo+T9iEc8Im2wwQbJlk5zdob\/8repYeedd04O5bbN1q+YY9585WCXXXapht7kmuDjZNtuu23atgHmEnMghhzWzAacpxAIv7jmcSK9nMiRDQRpexE3KFkjY0CkyCeIGFFBTsDug3zJitNkm9zPvF9+38tdl0W0yrd8+fJqE0eQrflathHWlB6yBYRKZz3anHTN3\/LTywVyylPPvyntWfQbKOnZ0HXUNAeJ369sm9yw4W3xBqqMMQq3Equtor4QYEnVE5\/4xOS8AOezmjS2TtQcZPQSR9ELSR5zzDHJ4dr1dC6++OKEpNZZZ53k4BdzdIcffnh605vetHJHmDW19IDVVlstWSYmTWST5vgfUswRhOmKRGDbjMiDbJEMvPSlL63KWydqYUA+iFla0gV5thWVPRApAkK+CBZpId7o8erpArlIz49VuNuu\/cgiW\/O1dbKNnq1yKlu3vBCu+VtkSn9AsojXPK4XZ3q7wO9DH\/pQQr4gjvKDdLrlMRf+\/dhqUD1GTXOQ+P3KtskNG94Wb1DbjUu+lVjt\/X\/nO9+ZHv3oR6e3vvWtSW9K5nZiOSDEoSwaKr9h4Bdnp512SojbS6OmNCz9MZz33S090t122y0hHx+C87bXzis7vmxnveCCC5INA6YB6DefO6+ayjKMH8IE5AkIJ4gZySIh8OODjF2Bn\/AgYXGhmw5\/+MMfEjJFuICs6oQrvFv8Uf2RrbItX748LV8BJBu9W23A1IIymbsl15Yf0gQEaspHeSAIGOEiXiQM3MAfyMERRxyRjrge2rr0AtKHNl1K+OKyQE9iNfxHTN6av+51r6uG4Z1Op7KQwz\/e8573VIdc23k17LQAwtAL9gVQw\/kq8eyPt8YaMZI05I+gzTffPAkLIrVJYFp2XvmxiXIOc2VTWLp0aUJA267oHXvBh5QgJ98gXnLkm\/LLCRcxtZFtv\/q3yUU4wgWEunz58urAHKRru3UQr\/sgXmUiq0ziNZWpyQ9BgvYGSBSUOYBokW4AEQNiDrgPhJyruBAk7WpKSx4B+eagT6BJ5ya\/sFtTWD9+g8TvV7ZNbtjwtnj9lHc2ZHoSq2E5wtpss82qoXVdAcNtYaN880pP1FcKHvnIRyZH4NXzQJ5XX311sqwiD\/OyStzLL7+82s5qWVXZeZVbqLd7yYoXboB8tl1BvMgoJ11uPV3hTYTbjWxNLfjC5mz2bOslQ570RKaIV1mQLLLV00W+zg3m5gfClRHEA2mA9Op5DHIfROiak2SQZ5C0K3sh2wACzhEE7RrE7eq+joinrJFeXJF4HaGPa67nRRddVB38Tv\/AIOUvsin1JFZk5XtWvQ4xEUau0+nMij0Rq56zHle3DFS+KYpp2Xk1G\/NGg6SJcJFpE+EiWxBerw9k6wFFrEgjerannnpqOnUF6vJtOg0b3hRP20CYiBMQKRIGJAsIFxAwMgbugDAgC0g5ID2QdkB+UC\/3OO61+TrYHiwzRJY51EcdQbquQcqucZ8TNzLPEWHed4Rb3IA0Agjd1JJrINeNzp5z17xMuZ2a6lR4N39h84mexKpHaLmSX1U917qiuuHC7nrXu1aL3evhC+Xebg1lKfhJ6mUDB+\/4gQMnnPlkj69JmJaBpvpGth4qQLSmfA499NDkJaSVHr3yG2eYufhh0vMCNOAsXLC8Dx74wAcmPzLgZDWwSy1w8MEHJzBlEdCbzIGgA1ZJBPS6c2y99dYpYNdbDj8a9h7ERgAAEABJREFUOZrqYdx+OQGGGzEGcuKsE7r7IF3XIGPXIGnXnMitAMnx3Oc+N8ErX\/nKFDCt4jB4HYNxl3fQ9HoSa6fTSc961rOqNC2DYoRrr702ffSjH01OteLnwSGj11oJdvlz8sknr3Ky+8te9rIukqt6r7HGGmn11VdPV1111aoB19+Zjhhl55Vk7NbwyzcpoNO4dRk1zW7xvUw0Nw4xd2tOHtk09WqNPtSlXq2dblaV2BBh95ppHeh0OtXOs255hm26hTf5m0qKePN5tXstB4LW0wWrJAJ2t+V47WtfmwJ2veX45Cc\/mXIgcb3tQPS446rXnSOIPa4IPXrirnTLET1yVwSvVx5g+9mAFSA59MrBS8mATp4fPpgNHQZJsyexSshLKl+otIb029\/+drL8ylIrvQ0vlBwwrcdKtheckCVOwOn2veQjzGn5fo2tnQ0\/V\/fmgDVMPSgvz5C8sADDj3vnVaRdrs0W0Ftg821XzNt6KINsES00ka16MyerVwt6NHq3Volwm0YAMmSbcy6+3SwQpBdXhJgjJ01uxGqKJBCEG9eclPfZZ5+VZwcj7iDzuPIDvXTXPC53pKmnrr3koAvQFYG7QpSjW3knwb+VWCnpbbxlVT\/60Y8Scj3rrLMS0rKW1K8\/mTbodfqMb+DmN795W5QqPKYj\/CoZilaeK\/44JxbpIlbwMkuvxzrWFcFJz8iSK1MZ0uC3UNCvTQcpz6hpDhK\/LotsES14cPohW2VTh8gU2QKStdYW6bqqb70UpGsIShbx1vOX1vgw3pRmQ9dR0xwkfptskKBeOjdizLHDDjsk0FMPIo9rkC4C1kN3BQQNQd6u7gNGROOtpcFT64tYNXC7okwDePOuAfuFYYCzzz575SL9wbO\/LoZerB6xqYaf\/exnaY899qjm+oR2Op302Mc+trr3YJoPRaTmU\/73f\/83IX0wFB1m55U8CubeAt3IVm9Jz9bD19S7DU0RqHaIUJEugq0TL\/LlJ8zUQ07A4koj0ivXhW0BpB3AU\/NdmlZi1SB9mM+c6DXXXJPsgvJNLCcTCfMCwwHYwxbku9\/9bvJtKnNgiBpJOpyanw8bSteXCzqdTvr3v\/9dbU91r2fql8mVzCTtvKLPKPCSZZT4TXFHTXOQ+P3K1uWQLUL1A4pgvSCL3q2eLsIFhAtN5Qw\/pAkIFPlqqzkBI1zEq\/cL3MAfkDEg44B0pBeQPkSew1zrNhgmjXqcUdMcJH6\/sm1yw4a3xavbZq7uexKrpVQmwg3bDfsN47\/61a9Wa1rjhYMHwa4nc5yDKo0Uzd\/e8Y53THqrHiI9YG5vcQ3\/pamLb95O3nZVOS91\/\/33T+7JTfvOKzZYzEC4iBThApIFP+p2mIF7pAvOkyAvXr92Q5AQpIlEARmfdtppyRXRIt0AIgbEDNyBkHEVLxAk7Sp97dk18o0rXQL9lqHITY4FehKrOU07m\/QeDMERrXtkeoc73CF5G6+hD7tB4C9\/+Uu1uF96+TyotN1bxhHr27wo05sN01neI4w+GqflYNOy86pt3ipsMMh11DQHid+vbJtcP+HIExCptgjm6xAtwvVjDZaFueevPSNg0PZAfJDWIHbNZYMIXYMgXRFnAEEHkK3OgysCzhEE7Yq0c\/DLkceTlvOAXQGB1xG6uNKvDs+eMkBeviZ3Wx1FnDa5YcPb4kX+c33tSax6lMg0llIZmjuAxb7+8BtFYZsPHPBi33+ezhVXXFF9tNBSKuRZdl7l1inuYSxgAwnSRJ6IFAEDkgWEC8gXEQN3QBiQRcg5pAfSBvkEhtG1LQ7Cy5ETI7LMESSeXxFuICflcOeknRM6dx4W7ojnGunGdRBSjzK1lX8hhPckVvOWllvpEVpmZUWAYbdj+xSOH6L1Rp4sv0HQ6XSSl2EafcQztLdez1dWLdFCrF6eWVIVMvWrnq00LMuqhy3E+9mYNxo1zUHi9yvbJjdseFM87wkGbQtBjq4IExAoQs6BbAHxQpCxK4IOuA+QA2t+xQ3khM0tvxx0CNArx6DlG0Y+yC+\/DkLsQbiuiDhHEDUCz8HfxhLXXF4aRx11VHLNCXzDDTccpmhjjdOTWJGVN\/LWE2611VbpJS95SXI2gJOtDMFtErDIW6MlO4pmSPqwww5LNh1ceeWV1bmdTWcHjJJHt7imOSYJ7DlufUZNc5D4\/cq2yQ0b3hRP3Y\/bpoOmZ1dQ4EEPelACR3Gapgg48jCH3Ug56oSDbAI56XA7HCmHZzWHaYgcXvDV4bzigA5MDh2eHGw8G0Di8nGtk7hOnx563iO3Q2029BgkzZ7EKiHHBVpD5ldSg7XQF4kysELtvffeydt8sr3Qa+eVHobtgNbK6qUich\/4k95s77yy66rgslRsMH02MNeaw7ruHCeddFLK4bmrwy7LAB7I4TzkHNEzj2v0zuOqhx6IHrqrnnmOfnrouGGS0UqsnU4nOX3KL5893pZFKZAXRda22s5qLpRfLyBM61UDsfPKS6dnP\/vZ6be\/\/W21VdZRhKYfIi2bAAzx7bQKP1f3ZecVSxSM1QIlsbFZIJ+m4F66dGlaej1y8synVbiRbSCI2DUI2jXI29V9gByMrRBDJtRKrG3pIls9WS+2esk27bzycszwXzxLqvRSO52O25XwhtLuKUMWqxQiYJp3XkUZy7VYoFig3QJIOxDE3R5rdiVaiRVh2mW10UYbrXKISswvmRpAmje4wQ0G1tShCs4d0PvV6\/X2P4d5lU7nup1XXmodcMAByRpWZGxI4ntWlmABt89vfPCDH6xk9ttvv2RuyBxxp7MqWQ+saIlQLFAsUCwwgAVaiRWJ6S16e2kexNpV20cdTG2u1RDfVlJv9wfItxJF2iakxfdyrA5v\/Ag65OVd73pXNQ8o7w984APJ7q98bpfbJL8wMuZ\/fVJGXGkUFAt0t0AJKRYYrwV6EqulTuZRfWMKsdkRpXfoDab7448\/PiFHb+qGUWvTTTdNtsZ2e3HizIBIF+nKD2HSqT63q8fLTxgZsuJE\/HItFigWKBaYKwv0JFbbVK1bdbhBp9NJ1qra1urMTAp6kWXJiKUdZPkVFAsUCxQLLHYL9CRWu6vsjrL7iqG4kaw38u7BdMCwW1rFLygWmCcLlGyLBWbNAj2J1eErXlp99rOfTRYOO11qnXXWSeZcvaG3qN+wGwF3OuUF0azVUkm4WKBYYEFZoCexIsznPOc51aHRThL64Q9\/mLzEMidqCsAOB+tbvdSy1nRBlbxPZc0Bx5rbPqMUsQVmAatP9t1337Tlllume97znslKF52GBVaMom4fFrCr0zsjdW15p5fzs1HXPYmVnl4wOandMiZzqt6yO2Sa266o3Xffvdrq2ulMV4\/VUjAfgnMurI0IbFGw8C3QVILjjjuu8j7jjDOSPee2a9uhVHmWP1NlAUSqc2ijkjMGcJupzHEXspVYZWidqk8rWKzf6XTSdtttl774xS8mSllbap6V3CTizW9+c3JId103v1KMrFzW5Nrk4BMPNi2QtS73Xve6V7Kl133B5Ftg2Lq2qsUZFVaWWE7o+0o+QzT5JV68Gg5b11Y02dXV6XSSF+6ed3U+bku2EquM7eX3rZlXvOIVqQmWYTn1f9zKjZoevU844YTGZHwEUZme9KQnJTKGBnqnei1pxT8\/IrbyGhquuC3\/J9wCo9S1byuZzlJEX8jwA2vLpfuCybPAKHXtq8K+QOLwG1+oMCL3Qn7cpWwl1mOOOaY6K8C6VV8KaII1o5PwnZkwjpdt1rDa0aUSwj+ulpAdffTRycEv9hj7hpaet62zn\/rUp1Ksggj5cp1cC\/RV133U9d\/+9rdkc4mNJnvuuWc13zq5pV6cmo2rro2wfbnE+xOH1DgZa9wW7UmsNgggVuevytxLqyYIs7513MoNm555YETp5J2msxmtZHD4C+LtdK6bG3b+q6H\/ueeem2yrHTbvEm9uLTCOuvZCQw\/GtmnzrTvttFP1bbW5LUnJrc0Co9a1D5Uaccc0j1GpDpit7215Dxrek1jNQejdPeQhD0k+lzJo4vMlb4mYLbeG8k2rFZCqFQ\/1bbjm1kxpOGlrvnQv+Q5mgXHUtWmhbbbZJjnr1MM2mAZFeq4sMGpd6ygamTic3xSnJaPOcvWeZdxl6EmsXlohn2kbGjtL1nCgiXTHbeCS3nxZ4Lp82+raFJZRipGZeVbzbmCq6LoUyt+FYoG2utaZcn60M2ctFd1xxx2TUavDx8ddxp7Eeotb3CLtuuuuyfBIL2\/cmS+E9BjeG8iFoGvRcXALWDJ45JFHprPPPrta5WKlC3ipOXhqJcakW8AGJ\/OrltPZ6OTDk53OddOB49R9BrEaCnvLby4CdJV9TXWHHXZINgvwq4O8eONUbDbTsrzCkMDQoJ6PXzVLrer+5X5hWqDU9cKst2G0nqS6nkGshkbe8sfbf\/NPJvWR61e+8pUU\/vmVvHjDGGM+4vgwofljRxbm+V944YVpzTXXTLOx\/CLPp7hnzQIzEi51PcMkU+sxSXU9g1i93feWv+ntfzc\/8uItlBqzvGrttdeuhn4mseltW6PhgSVX5QUGi0wHSl1PRz32U4pJqusZxJoXQK\/O0iOkk\/svdLczZcsXBxZ6Lfanf6nr\/uw0DVKTVNeNxGq7p3lT68as89p8883T8573vOpQ62moAGWwELx8cYAlFj7aSlDqus1C0xM+KXXdSKxHHHFEtQvljne8Y7U7yXZP86v7779\/8tInLZB\/llR5A9j0Vt++8PLFgQVSkX2oWeq6DyNNichCqOsZxOpNuZdRFkw7\/eWQQw5J1vS95CUvSWeddVb1gb4pqZ9SjGKBYoFigVmxwAxiNa9qt9V973vf5JdBrp1OJzm70Iueq666ildBscD8WqDkXiwwwRaYQazddF1ttdXK\/uluxin+xQLFAsUCmQVWy9zFWSxQLFAsUCwwBgsUYh2DEUsSc2WBkk+xwMKwQFditdPKGtaAeVdzrLauhl9cf\/3rX6d\/\/etfC6PERctigWKBYoFZtkBXYn3Pe96TrGEN+LaVjQI+cxJ+cX3sYx+bylF7s1xTJfligWKBBWOBGcTqIIMDDzww+WJAvyAv3oIpdVF0IVmg6FossOAsMINYnaTvY4EOiu4X5MVbcKUvCi84C9gVaMOHbxc5P9V0VBTCucEveMELkh\/6888\/P9kxeOihh0bwKlejL18NqKexilC5KRYY0gIziHXIdEq0YoE5scAvf\/nL5NtHPgLn8zv5SMknVhCqH\/o5UaZkUizQxQKFWLsYpnjPrwW65W5noN4m8nQIeT5S+t73vpecp7vxxht3i178iwXmxAKFWOfEzCWTcVjAsP5Rj3pUuuaaa9Ib3\/jGlA\/jrVhx8r\/PbAxznu7JJ5+c1l9\/\/Ubk+YyjHCWN6bdAIdbpr+OpKeH222+f9tlnn7TGGmuknXfeuZpLjakA3zvy7Srfq+p0\/vupjfqyQXOyYImg+dowzmabbTbjhbmjWhYAAAI3SURBVK0T3fSAly5dmm5yk5uEaLkWC7RaoBBrq4mKwJxZoCWjjTbaKPkInJPJDPdNB8RUwMUXX1zFXrZsWXWNP\/Vlg7FEEEl\/5zvfCbHk4PP8Za10TjzxxOTQc6e6xbkZKyMUR7FADwsUYu1hnBK0cCxw6qmnpk022aQiyFzr5z\/\/+elb3\/rWDJxyyinpnve8Zy660q0ni5CvuOKKqleMxFcGFkexQB8WKMTah5GKyGRbwK5AR1p6mWXonmt705veNPlsUB1Om9fzTbV\/5moPP\/zw5HPYL3rRi5KPaNZEym2xQKsFCrG2mqgIzK4FRk\/90ksvTeZSzZOOmtp5552X9FZNOTz5yU8uJ7qNatBFGr8Q6yKt+Gkqto9Zmn\/1xYtRyvWLX\/yiejmmd\/v6178+xYuxUdIscRenBQqxLs56n5pSW9f6jW98I1kNsPrqqw9dLvOqvpZhXtXXMm5\/+9sPnVaJWCxQiLW0gfmwwNjyvOSSS5KlU1tsscVIafrOm08S+b6bedbjjjsuBU466aT0+9\/\/fqT0S+TFZYFCrIurvqeutOecc04yBbDeeuuNVLaLLrooIdTTTz89OW\/ghS98YQrsvffe6ec\/\/\/lI6ZfIi8sChVgXV30v+NI6fMXW1T322KMqi6sv8dbXmdblKuHsD3nxzM+aU3Wwy2WXXZaaID\/pZdGLs1igpwUKsfY0TwkcpwVKWsUCi8UChVgXS02XchYLFAvMmQUKsc6ZqUtGxQLFAovFAv8fAAD\/\/24VqJ8AAAAGSURBVAMAK3cr9StIxn4AAAAASUVORK5CYII=","height":206,"width":342}}
%---
%[output:32ef08ab]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVYAAADOCAYAAABl7SoZAAAQAElEQVR4AeydC7xNRfvHn63oVIqkQuhI5a8XUU65lmtC1JvI6aLbSyglb3VK5ZpEJb2RUhIJ6aL0quQWEbl0hF6FRMglty4uifr7jua0HHvvc\/Y5e++11t5Pn56zZs08a9bMb2b\/zHpm5pkCf+p\/ioAioAgoAlFFoIDof4qAIqAIKAJRRUCJNapwamaKgCKQtAg4Kq7E6gBDg4qAIqAIRAMBJdZooKh5KAKKgCLgQECJ1QGGBhUBRUARiAyB4NpKrMFx0VhFQBFQBPKMgBJrnqHTBxUBRUARCI6AEmtwXDRWEVAEFIFQCOQYr8SaI0SqoAgoAopAZAgosUaGl2orAoqAIpAjAkqsOUKkCoqAIqAIiESCgRJrJGipbp4QWLZsmVxyySVSt25dufTSS+Wiiy6Stm3byg8\/\/JDr\/F544QWZNm1arvTR+7\/\/+78j3tenTx\/59ddfZffu3dK9e3dZsWJFrvJSJUUgLwgoseYFNX0mYgSqVq0qH374ocyePVsWLFgg5cuXN\/cRZ5TLBy688EKTP++bM2eOeeqJJ56Q4447Th5\/\/HGpWLGiicvLnz\/\/\/FMOHjyYl0f1mSRBQIk1SRraS9Xcv3+\/\/Pzzz3L66afL77\/\/LgMHDjQjWka1hIlDBgwYIGlpadK4cWP54osvTBUgtTFjxpiRb506dcSORE1iiD\/HH3+83HLLLfLVV1\/JqlWr5I477hBG0b\/88ot069ZNatSoIRdffLE89dRTpjzE33333SbuxhtvlPbt2xt9Rs2dO3eWFi1ayLx582TmzJmmHLVq1ZIrr7xSli9fLlu3bpVWrVpJz549hfIxMh89erQ0aNDA5DdjxowQpdRoTyKQx0IpseYROH0sMgTmz58vVapUkbPPPlv+8Y9\/yI8\/\/mhI6aOPPpL169cLo0pkx44d8s477wjxGzZsMCNc7iFi3ogOn\/HTp0+XTz\/9VE488UR57733SAorxYoVk+LFiwukaRU\/\/\/xzOeeccwxJ8r5vvvlGNm3aJCNGjDAjWtIZ5W7bts0+IpRv3Lhxpi4ff\/yxQPKfffaZIW7yQPGnn36S6tWrm\/KVKlVKyHfq1Kny5JNPmrIeOHAANZUERkCJNYEb10tVY1S4dOlSWbNmjRk1Mprr0aOHzJ07V5o1ayYFCxY00qhRI1m4cKHMmjXLxDPaPOmkk8zoj\/pAdhBbhQoVjDlh6NChkpmZSVLEUr9+fWOHnTBhgjEPQNhbtmyRlStXCqPQQCAgZ555plSuXDkr73r16gnlOfnkk+WBBx4wus8884y8\/vrrsnnzZqPHSJzRdyAQkAIFCpiyH3PMMQK579u3TxDR\/xIaASXWhG5eb1YOkrnssssMwezdu9cQqi0pJgBI1t5nvzL5BKFB0Ahk2KtXr+xqR90z0mS0yqjVJr7yyivy4osvCiR97733mpE0pgbspxAieoQxXRB2ChNvnTp1Ekazbdq0kQ4dOjiTjwiHq88RinrjEQTyXwwl1vxjqDlEiADkxSd9SkqKsa0yiw+hIh988IEZLWJbJUzczp07hU9\/XoMtlE9vSBJSxpaJSYC0UILesGHD5Pzzzzd2XavHJ3rz5s2FiTWIEpJmhMznO+WjnOvWrRNG2vYZe92+fbsZuWJbZSRqy2fT9ZrcCCixJnf7x632S5YskaZNmxq7as2aNY0JICMjQ1q3bm0+kTENIGXKlJErrrhCWrZsaeL5JL\/tttukZMmSpqyYCiBdrnyWQ2qETaLjD5Nd9n3kC2EyUeVQMe9+7LHHzKc6NlBswOPHjzeTVZB1jUOTWizNOuOMM5yPmTC22VNOOUUYeV9\/\/fWGtL\/88ktjGjAK+iepEVBiTermj0\/lsVFiG4WsWP7ERBaTPowM+Uzm0550hDBxEGHv3r2NvXXixIny\/PPPCwRKWpcuXQRdxOo7a4Le119\/bSaPeN\/ixYsFe27hwoUF4fOfMmEHxcbL6BQCHTJkiPTr108YEUP65P\/ss88aOykmhI4dOwrCuygfKxjIm\/JB\/pAzJD527NiskTErDSgPz\/BO3k0ZuFfxFAJRLYwSa1Th1MwSAQE2F0C0tWvXFuynLLmyI+ZEqJ\/WIfYIKLHGHmN9g88QqFSpkkyePNmYKxjxsnrAZ1XQ4rqMgBKryw2gr1cEFAH3EIjVm5VYY4Ws5qsIKAJJi4ASa9I2vVZcEVAEYoWAEmuskNV8FQFFwJsIxKFUSqxxAFlfoQgoAsmFgBJrcrW31lYRUATigIASaxxA1lcoAoqA2wjE9\/1KrPHFW9+mCCgCSYCAEmsSNLJWURFQBOKLgBJrfPHWtykCikD8EHDtTUqsrkGvL1YEFIFERUCJNVFbVuulCCgCriHgOWLliGLOOOJANo5KRggTR5prSOmLFQFFwBcIeKGQniFWvLXjr7Ndu3bm7KDHH39c3nrrLSP4tIRUcd82adIkL+CmZVAEFAFFICQCniFWjtrgLKQ333xTOnfubM4h4lA2pFy5cgLhvv3221me5EPWSBMUAUVAEXAZAc8QK6de4tGdEzI5GplzivDEjsNhrtxDvBzL4TJm+npFQBHwGgIeK49niPXAgQPCwXA333yzcEAbR13MmDFDODyOIzIGDRokmAs8hp8WRxFQBBSBoxDwDLFyEufWrVuN53bMAfv375c+ffrI1Vdfba6bN2+Wbdu2HVUBjVAEFAFFwGsIeIZYGY1iDuCQNo5FPu644+TYY481eGEC4B4dE6F\/FAFFQBEQ70LgGWL1LkRaMkVAEVAEIkPAU8S6Zs0a6d27tzzxxBPy7bffmiuTV8SRFlnVVFsRUAQUAXcQ8AyxnnTSSdKlSxepV6+eNGzYUHr06GGuHEFMHGnoRAoT5oPRo0ebSbA6derIqFGj5ODBg5Fmo\/qKgCLgDQR8UQrPEGuhQoWkUqVKwnKqYEIaOpGi+sUXX8jUqVNlypQp8uGHHworDRYuXBhpNqqvCCgCikCuEfAMse7YscOMWK+99lqz3MpuZWU7a+PGjWXw4MGye\/fuXFfMKkKirI895ZRThBEvpL1o0SKbrFdFQBFQBKKOgGeI9bTTTpPx48ebkSVEyuiSLa4IfgKoecGCBblEJKtXr5bSpUtnPVO+fHlZu3Zt1r0GFAFFwPsI+K2EniFWCxxbW9llBdHauDPOOEOII83GRXJlqVYk+qqrCCgCikB+EPAcsRYpUkR+++03efHFF+WHH34w8tprrwkbBviUj7SyqampZoWBfY7VBhUrVrS3cvvtt+coZ599tiBOXe4RZ1yoMHqIM517xBkXKowe4kznHnHGhQqjhzjTuUeccaHC6CHOdO4RZ1yoMHqIM517xBkXKowe4kznHnHGhQqjh9h0wlZsXKhrMD0bxzXUczYeHcTec+XeCvfhJJiejeMa7lnS0EEIW+Heio0LdQ2mx8RysHjyuPfee+WBBx6Qjh07mt9VMD0bx5Vnwgk6iFOHeyvOeBsmLesH7lLAc8TK6LJ\/\/\/7CFte2bdtK69atZcOGDdK3b19h80CkONWsWVMyMzOF0S5CuGrVqlnZzJw5U0aMGBFWWOqFOPW4R5xxocLoIc53cY+EesYZjx6SU5wz3RnmWSSnOGe6M8yzSE5xznRnmGeRnOKc6c4wzyI5xTnTnWGeRWwcYSs2LtQ1mJ6N4xrqORuPDmLvuXJvhXsrzv5h44Lp2TiuVi\/UFR3Emc69FWd8sHAwPX48weJ5vnnz5nLBBReYZZPcB9OzcVzRCSmHfpfoIE4d7q04422Y8rktniFWPvXZtgog7MDKyMiQ2bNny9y5c6Vfv37C5BNpVodwbuTCCy+Uyy67TJo2bSpNmjSR+vXrC3G5eTbaOtOnT492lr7O77vvvvN1+aNdeO0fRyPq1z7iGWJl++r7779vPiFwuoL\/VQsz4VmzZpk0dGx8bq6BQMC4HJwzZ4589tlngk\/XQCCQm0dVRxFQBBSBPCHgGWJlxr99+\/by6KOPygcffGBGlthKkMsvv9yMXHHKgk6eaqoPeQ4B\/Ox6rlAJWKABU9ZKsW4zjbQcmunxGh5ZPL\/2Ec8Qq4XzzDPPNPYZ1p9aOwojTba24vTa6ulVEVAEcofAuIWbZMeg+rLkkZoy59tduXtItfKFgOeINV+10Yd9hYBf7Wd+ArnqY\/NkSNuKpshli6XIpM7VzMiVUayJ9Pgfv\/YRJVaPd6xELp5fP\/P80iaWVOucUzSryIQh1wFTvpM5qz0zes0qX\/aAX\/uIEmv2ltR7RcDnCDAahVTT00oKRJq9OsTVKV9U7hq\/IntSru6Z68iVYhIreZJYly9fLldccYVZJoXTlA4dOpiNAkncTlp1RSBHBL7fsU8gVWyq2FMzmqSGfGbSndWk7Ckp4rfJrJAV8liC54iV9azsuurataswkcUuqRYtWgjHYZPmMfy0OPlAwK\/2s3xUOWaPjlu4WVo+nymQKp\/6uXkR5MpkFmScG32rw84rG87PNTfP+rWPeI5Y2R3F7qsKFSoY3DmWhUX9HNdCmokM8YfVA\/hy5QDCZs2aCSNf9ccaAiyNTggEIEU++7GZMkpFmKTKbeVYLcCzfrC35rZOXtDzHLHiD4DNAps2bcrCZ8WKFeYgwXBbWjmM8LnnnpOnnnpKFixYIJ06dZL\/\/Oc\/Mn\/+fPXHmoWktwJ+nZiINYqQpVPuHLdCrPDpDpHadamMULGlQqh5LRfkir0VU0Je84jVc37tI54jVsiT3VG9evWSxYsXm62od911l+DcAdIN1YA\/\/vijpKamyrnnnmtUcIy9fft2s7HAj\/5Y6eT8uPgh2R9RuCs\/NpV54mcMWozaYMoPWTpl\/SHbqZUyxVIEEoUMEcLhbKnmx5CLP9hb7zpE4LlQNacm50YvqE6SRHqOWMEdUsTj\/7x58+Sjjz4yW1Fx7EBaKDnvvPME5y2FCxeW33\/\/XfDhiutBHLiU9pE\/VmMrG5qZ9QPjh8QPKCfhB+Y3mXhDCUMSfit3rMr7\/s2lg+KBLdTK0PTDa1JD\/Q7yGk\/+3+\/c57nJLLWx5rVF\/3qOESferDg5AMFxytVXXy2cIICNlTR0\/lKXrVu3mlUDbHm97777bLSsXLlS2rRpI998843xiIVZAZttlkKQAHnQgLEWSD7UO+Z+uUoq9fpU+k1eJf84VWRxl1SBeO6rkSKhnvF7PE3h9zpEs\/zh+kc03xMqr0fqFRUmsx4an5nnPrdr1y7Zs2ePbNy4Mc95OMsXaR\/ht8wzbotnRqzFihUTbKQTJkyQli1bGl+OnCiAYBpgMqp48eJZeLG9FccsbHvFrmonqe68807p1q2bDBs2TBixYh7AB6t9kDArDew9V\/LAlhNrYeQc7B0TVgaEz8DLKhSX5b3qSv+21SSYnsaVS2hcQvWPeLX7tXUqmp1ZLy7YJRsPnhIS65EjR4ZMK1q0qJxwwglmRc\/f5Y5fu\/Fb5jfttniGWJn9hwi54hqQkWqpUqUEuf7662Xp0qVmAisUYPxrj+erbZLRkAAAEABJREFU4cOHS926dYV80M3JHys6bgp2VOxpfOrH6jPPzfrpu\/2FAJsH0tNKmKVbulIg723nGWK1VeAk1m3btglnVdm4LVu2GFK1ZGnjnVf0WV7Vrl07wZSAYD4oW7asMRl4wR+rs7xMTjExBalO6lzNmZQ0YT75kqayPqoo\/8DnZ2dWNKvq1z7iOWLlaJabb75ZOnfubEaejD7T09ONT1XMBaEaDTssy7I4fBAH2QhmBEbBkK3X\/LEyA8tkARMhkaw7DFV\/jVcEookAk1msFOAf\/zzmm9SPeY5YaQ1IEvvp2LFjhfOuIMsGDRqQlDBiSTVhKpSHimCDy8Nj+kicEBiSXtFMZmGuitMrj3qNX\/uI54iVmX8+4dlBdcMNN5iRauPGjYU40o5C3mcRjABYa5msn\/8+a66kLi5fUvRT3ZkVeTfwHLHyuc\/qgLfeekuQl156Sa688kq55JJLss69irya3nmCkSqdlU7rnVK5UxK\/2s\/cQcudtzKZldGknJnMYvcXA4Mhv9YXBgfBRrLRLqVf+4jniJUJKuyiLKdC8BnA8qk\/\/vhDfv7552i3W1zzY0mVkurfkPv1M+\/vGiRHiFUCDATYvMI6V2rN5CsjWYiW+1iJX\/uI54g1WAOtX7\/eLDgOluaXOP6F79WouNBB\/VJmLaciAAJzv90lECnh7ALRQrjZ45P93nPE6txRxS4KBFsrR1bbI7D91miQKp9TF52Z4reix7S8fv3MiykoHsx83IK\/HSIFK96AKd8Fi45KnF\/7iOeI9dRTT5VJkyYJO6TYRYHMnTtX8MmKOSA3rZWZmWn0IWm7IwtXgnXq1JFRo0bJwYMHc5NNvnX4lxzHKelpJSU9rUS+89MMFAE3EGBU6sZ7\/fxOTxHrr7\/+avb4P\/zww7J27VrjDwBy\/PrrrwV\/AHiryglsVg4MGDBA9u3bZ1S\/+OILV9wGQqoY+xmpZoTx5G4KmaR\/\/Go\/S7bmctN85dc+4ili5cjrl19+Wf73v\/\/J888\/L4MHDzbCWtZGjRoJKwbCdWq8Wg0ZMkTwK8AEGLrkaVcU4HYwLS1NFi1aRFLMBHuUkmrM4NWM44xARpNyQd74d1T6oS+yv+80BAKeIlY2BvTr10+efvpp4wKQ41isXHfddYKnKgodSqZNm2YcQLDu1eqw1RXnFva+fPnyZjRs76N9hVQ5IiP90Kd\/ho5Uw8LrV\/tZ2EolYCJ9mS2uwarGaDaW\/dyvfcQzxMonPKTKlU95dlqx399K9g0CmAhwLcjkFmYCnLDgu5WDB1my5ewEObkNdOrmJ2xJlTzYb81VRRFIBATY4ppxaOQKkR4hp+iEbLD29Qyx8umObRWnKezxZ6+\/U4hDx1aCNa6zZs0SJrdwG4gdFu9WF110kdSoUcMcycLIlRMJmAizzxHO7jYQcuZfxvwI\/lRvH5kpBw4cMH5Ug+UF+QeLT9Y42iRZ6x6s3l7vH23O+9P0bfwEI4\/85b\/1Px8sy\/K9qv5Y6dUiniHWw8U5\/JfRKLuvunfvLlYYxYbbIIANFpJFOOcKcp06dargLJtVAhxEiBCuWrXq4Rf99ZdnMJLnRwbM+VV+3Cvywd1pIX1VYpLIzzv02XIhsU0EbPzWP\/Dfyii217RtWf5bi6o\/VsMqniNWyK9r166yd+9eqV27dpawXKpgwYKm0JH8Yf0rJoNYuw3UraqRtMphXUZth0P6168IZDRJFeyvA2O0ltWvfcRzxAqh4twat4HNmzcXK0xs8Vmfmw6ImQDPWFwDgYBx5BJLt4FsANCtqrlpGdXxIwI5lRn7K2tdY729NadyeCndc8TKkipsqfhW9RJQwcqCEwol1WDI5C6Oz\/fcaaqW1xHYMai+cTH4+BfRnczyax\/xHLHu3LnTTDyxCoDP\/1CrAtzuaKxTxfv\/kkdq6v5\/txtD3+8JBPgtnFooPrsaPVHhMIXwHLEyWp04caLZ0rpgwQKxKwOyrwoIU6eYJrGkilEqjin4\/I\/pyxI8c7\/azxK8WQ5XLw9\/WYZ1R6Xf8\/Bk6Ef82kc8R6zM\/LMCwK4GsNfevXsLJwmwuyp0M8Q2hW2qLP7nLZAqHYmwiiKgCCgCTgQ8R6zM\/P\/222\/m8EBOWMUcgNtAJrVYp\/rYY485yx+38IApa4XPfz53ECXV\/EPvV\/tZ\/muuOeQWAb\/2Ec8RKwTKqPXJJ580HqpYh\/rss8\/K\/v37pVOnTrJjx47ctkm+9RihQqZ4qMKemtEk\/J7pfL9QM1AEXEVAXx4tBDxHrFQMz1QQKWEEN394vmJXE\/exEkgUYdkIZEoYWyoznoxSM5qkxurVSZmvX+1nSdlYLlXar33Ec8TKcisW9bMq4MEHHzQ7r2699VapXr26sbGSHqqNf\/jhB7n++uvN+Visf12+fLnk5I9119UjBBK1E1Lrd+yTMsVSBDJFINRQ79N4RUARUASCIeA5YqWQt912m4wePVpwxMLuq4EDB0qbNm2MaeCRRx5B5ShhNIv54JprrjHLtTp27CivvvqqcRHI1tYpU6bIhx9+KDNmzBBcCdoMir57uyFRCBRhsbM6ULHoxPbqV\/tZbFGJe+6efqFf+4gniZXNAS+++KJ88sknwukBI0aMkFtuucV0ACa3TCDbH0ar2F8h40AgIFdccYXg1GXx4sVmBMuxLvHyx5qtaHqrCCgCSYaA54gVW+rgwYMFv6kQ5bnnnisnnniitG\/fXljjGqp9fvrpJ2EpFsuzqlSpIjfddJPgaSfe\/lglzH8NGzYMk5p8SX61n8WqpbR\/HI2sX\/uI54h1z549wsiyVatWwnKrSpUqSZcuXYTRKw5aLPR4wMK5Ci7\/8Me6bds2c6zL7bffLkuXLhUOIHzooYcM2ebkj5U84iGUPR7v8cs7IBK\/lDUe5Yxn\/4hVfYYNG2bMbTg9isY78tJHwNFt8Ryx4miFVQFbtmwRnKisW7dOcFy9e\/du4\/HKAkaa0x9r8eLFzQSX9bXKcSwFChSQkiVLml1c9rns\/lhxGaiyxvi1VRwUh\/z2gbffflsw3XHWXH7zys\/z9vfu1tVzxGpHq\/zLV6FCBXnllVekXr16hmTDrQhITU0VNhasWrXKYImdlrwY1eKDldEuQji7P1bzgP5RBBQBRSBKCHiOWKlX\/fr1hVMB+JR47733hJFpz549w555BYliEmDVACsJIGaWa9WoUUMgVz5NmjRpIuTNci7eo6IIJBUCWtm4IeAZYuWsK9auWm9W9oqN5aqrrpL09HRBJxwy2GMnT55sVhLgtIWlGoFA7P2xhiuTpikCikDyIeAZYgV6trKmpKQIS6tGjRolb731VpZwVEs4UwDPqygCioAi4AUEPEOsLKXCycrQoUOFQ9UYvfJpv2TJEilSpIhZasUklhdA0zIoAv5BQEvqBgKeIVYqD3GybrVHjx5m9xSuAjkYkEX\/LLliRIueiiKgCCgCXkbAU8TqBOqPP\/4Qtqk64zSsCCgCioAfEPAUseLFiuVSffr0kVq1aknfvn2FWX3292NjPfnkk\/2AqZZREXAbAX2\/ywh4hliZ8W\/RooXgyap06dLCrD5LrlhzynZV0iFel\/HS1ysCioAikCMCniFWSsqIlN1Sr776qtx8881y7bXXZgk2VnwHoKeiCCgCioCXEXCdWJmQYt8\/flOHDx+edXigPUTQXhnBsnLAy2Bq2RQBNxHQd3sHAVeIlU96FvKz+L9Ro0ZZo9LLL79cGjduLKSh4x2YtCSKgCKgCOQegbgTK24BmZzCG9W7774rCxYsyBqlsrRq4sSJwgj2nnvuCVsLVgzgb3XZsmVZejNnzhR2bDHxxUkC+GhlJIzTbA4lrFOnjrDxQEk7CzINKAKKQAwQiDux4r2K\/fzYUNnfn71OhQsXNhNYHCCYPc3eT5s2Tdq1ayecCmDjsL+OHDnSECfOsRn5spIALzvhThCwz+tVEfAlAlpoTyJQIN6lYhMApwAwy8\/uKkaYTsEc8PTTTwsrAUKV7dRTTzXmg7POOusIFbxb4dyaSFwPQtwcw4ILQT1BAFRUFAFFIB4IxJ1YbaUgOggP5ypMTI0bN044TgUXgSVKlJBevXpZ1aOu1apVEwgYH6w2ET8CEPV1111nTh8gP\/L20gkCtqx6VQQUgcRGwDVi3blzp2zcuNEcoVKqVCk588wz5e677xY+6bGRHnvssREhj1NcbKkcGIgza\/wMsA4WW2y4EwQ44jovMmDKWkEiKqQqKwJRQUAz8ToCrhErXqz4XOekAAsS5gEmtfbv3y+Ijc\/NFWI944wzpGzZshIIBMxpAtu3bxdGvxCtzYOwPWWAuDc\/nCV5kScnLhSkeMc3BSnWbaaEEtJPb\/eClGp2r+BjVuVsxeHsxMSAI5U4Hgmfx272c37bboprxIr9k0bg0L+6deua2Xw+4xHWrnIgYCTAlClTxvhrtUTN6ayse8XJNacGcHoAQpjdXDbvwnMGytbRHSOWbS+0FqfsGFRfQsn9\/0yT1k0vkwPFK8iuq0cYOfm21+WOYTP0SJQ1ehwKgwKEPsnVz9KpUyfBaRJfjm7VAxzdFteIlYrXq1dPmMm\/\/\/77heOuJ0yYIHj6Z1trhw4dUMm1nHfeecaPK8usMCWMGTNGunXrZnwNQK7k69YJAhlNUmVoekX5vHeTLPJNTysp4xZuEka5ua5kgin69QTOWDXD9OnTw2WdlGl+7SOuEuvLL78s2EJfeOEFYQdW165dhX\/pcmNfLVy4sCHjypUrmw4XCAQEXwOMdj\/77DN54403pFy5csYswNKsOXPmCPE33nijiTMPufgHsl3ySE1JTyshLYdmulgSfbUioAhEGwHXiJVJqi+\/\/FL69+8vRYsWNSQ4ZMgQYdMAaZIk\/5Utdrx8v3NfktT2yGryD9+RMXqnCByJgF\/7iGvEyu6nlJQUOeGEE7KQxO7KGldm8rMiEzxQtliKfL8jOYk1wZs2KtXTTPyJgGvEyhpUJpfYYrp3715ZuXKl9OvXT1j8z5pUf8KZ91InI7n61X6W91bWJyNFwK99xDViDQQCZt1q+fLlzUkB+A9gPSsTWbmxsUbaQF7Vr12+qFeLFvNy+fUzL+bA6AuyEPBrH4k7sWICYL0qjlZY\/sRuqxEjRgiz+MzoM3pFJwvZBA9gCqCKc7\/dxUVFERBRDHyPQNyJlYkpnFbjxPrKK68061dZz4rgOGXw4MGye\/du3wMbSQUsuUbyjOoqAoqAdxGIO7FiV8U3AJ6pIFKWV3366aeCvPPOOwYpJrBMIIn+jFuwKYlqe7iqfrWfHS69\/o0HAn7tI3EnVtsYmAH47IdobRxbUokjzcbl9+oHf6ysZy1TLMVsFqj62Lz8Vlmf9yUCWuhEQsA1Yi1SpIjg5o8dVzikRl577TXjI4BlV9EC2S\/+WNmZBcGmp5XMIthEd\/Li14mJaPVNzSdnBPzaR1wjVjxOsTmANau4+2vdurVs2LBB+vbtKzjDzhny3Gn4yR8rtlZ2ZOFzIJemJqwAABAASURBVP0Qwc5dvTOLZPHAlehEm7sWVS2LAP2BXXtsiw4l9Burr9f4IRB3YuVTf\/PmzaaGnMqakZFhjmbB6z\/rWPHTSqLVIZwfWb16tXCcts2D5V1r1661t569QrCT7qxmfAtAsut37JMBU74zRMuPCJMBPyp+OPzAPFuRMAXzq\/0sTJUiTcpRn7aljWlrhLa3gq8JTEh87fCPcXbJaFJOxi08\/FvL8UUeVfBrH4k7sbJG9f3335eOHTvK559\/LpyBZduU8KxZs0waOjY+v1dGx+HywL0ZDRhrYUSel3e0Oe9PebZZUVncJdVIr0bFpek5KXJKoQOy8odd8tq89VmEa390frhe9NxaX5Y7Vthmx6NSr09N29LGtDVi+wDXiTeUkPtqpEiN4nslWL+i37x\/c2mDceOn5wXVCfZcfuI4wWPPnj3G13J+8rHPyqH\/bDg3V37Lhx5x\/f+4Eysz\/u3bt5dHH31UPvjgA6lfv36Wb05OBWDkymYBdKKBTmpqquCD1eZF2OmPlXjcm2HLibUwco7GO+5uVln6t60mo9qnydR\/15TlveqakW32EYve1\/cVLpCls81oV4Q2pq2RSPtP7QvOlUmdq8nijftk48FTjE+OSPOIRB+\/H2xTx3F9JM9FS5ffMr9ptyXuxGorDPC9e\/cWbKCAgeB9qnv37nL66adbtXxfa9asKfhgZaUBQrhq1ar5zlczUATyikC8n6tzTtHDXtSez1S\/FHEC3zVijVP9BE\/mbvtjjVdd\/fYePu38Vma\/lhc7LJOjd41b4asq+LWPJDyxBgIB8aI\/Vl\/1bi1sQiAwpG1FmfPtrkOToN6fvPU74AlPrH5voEQuP3a1RK7fEXXzwA0mAeytrC5htYEHipRjEfzaR1wlVpyxPPjgg8I6VuyrOLpmZUCOaKuCIqAI5AkByJUJMsg1TxnoQ7lCwDViZWPAgAEDhHWllJQZ899\/\/12efPJJ40aQOJXERsCv9rNEaBVGriwbYz20l+vj1z7iGrHi5eqPP\/6Qhg0bSoECBYTTBFhixcw9aV5ubC1bdBDw62de7mvvXU07cmXziSVYL5oH\/NpHXCNWtq0ec8wxwmJi2\/0wDUCqbCKwcXpVBBSB2CGQ0STVrPWFYNnJBcmywyt2b0yOnF0jVhytNGjQQO68805zLAunteKj9brrrpNkPJolObrbkbX062fekbVIjDsIFidACIdbQrBspXW7dn7tI64RKw3WtGlT+e9\/\/yvPPPOMcCw1YeJIy0mw0T788MOybNmyLNWZM2cax9m1atUSTiPAY5Yf3AZmVUADiYCAr+vAWlfIFRssp1pAsL6ukEuFd5VYIcKePXtKiRIlhJMDOKbllVdeyRGKadOmmbWpOMu2ypgQRo4cKRxOyLZYnGg\/99xz4he3gbYeyXT1q\/0sGdoIGywEm9GknPE14Fad\/dpHXCPWn376Sd544w1jCli3bp2ZxIIwIUJIMlxDcpIrZoOzzjrrCDX8u+IEgsh9+\/YJ5ga2zF5yySWC1yzu09LSZNGiRaioKAKKQA4IZByywTJ61ZFrDkBlS3aNWCHBwoULC6cGLFiwQCA\/JrQKFSqU43KratWqCQ5bOELb1ge7LOthsdGyhGvcuHGSnp4ufnUbaOuVyFe\/2s+CtUkix9nRK+Qa74ktv\/YR14iVEwT2798v99xzj8ybN09SU1OFM6+IY2Tp7KhMbOEOjD3\/W7dudSZlhXHiMnr0aOEMLTxY8cxTTz1lSDont4FZmWhAEVAEgiKA7ZWNBWyJjTe5Bi2QxyNdI1bI7rHHHjOjyoEDBwpOr8EKl4GMXAlbgSAhTny1hvJ8RTqj37Jly0ogEJDq1avL9u3bjf0WorV5EXa6DWTSrEePHhJO+FcTsTqErdi4cFeriw0YPXvPlftwgo4Vp16wOGe6DVs9rjaOK\/cI4ZwEPcSpxz3ijAsVRg9xpnNPmzjjQoXRRZzp3CPOuFBh9BBnOveIMy5UGD3Emc494owLFUYPcaZzjzjjbP9wxhFGDyFshXvE3oe7ooc4dbhHnHGhwughpN9VeKbxN\/DQ+EyxccRbmT17tnzzzTcydOjQrN8UeojV4co9QjicgAl6iFOPe8QZZ8P8pulbboprxEqlIdALLrhAsJliE61bt65AsqxnJT0SKVOmjPDcli1bzGOLFy8WDipklJuZmSlsPEAIV61a1ejwZ8yYMQKZh5Ny5coZP5ZWx95ztXHhrught956q3kXYSvhniPN6nHl3gr3iL0PdUXHilMnWJwz3RkOphsszvmMMxxMN1ic8xlnOJhusDjnM85wMN1gcc5nnOFguibuUL9w6oUKB9MNFmf7R\/Z8gukGi8v+nL0Pphsszupnv2bXZeT64oJdxr9rdt1LL71UKlSoYOZObFr254kPFkd8MAmmGyzOPstvmt+2m+IasbJcihFrixYtpFGjRnLNNdcI516xQgBnuZGCct5558ktt9xillmx3Apwu3XrJjVq1BDIlWVcTZo0MY61cSUYaf6qH30EGHFEP1fNMR4IQK53jV8R81f5tY+4Rqw7d+4UZv9ZMvXvf\/9bhg0bJq+\/\/rowcg0EAjk2GBNfnPBauXJloxsIBASS5lMEhy6sOOBftUBA3QYagPSPIhADBLy4DTYG1Yw4S9eI1VlStrDyCc+pAtu2bTOE60zXcGIiwD98\/quZltgikNGknOAla87qXTYq6le\/9hHXiJWlUnz29+\/f39gvX375ZbMDi91S2F6j3kKaoSKgCEQVgfS0ElKnfFEZOOW7qOabCJm5RqyBQEDwxfrQQw+ZNaydOnUSPu9xG5h9uVUiAK11OBoBv9rPjq5J8sY8cGjUyhKsWI1a\/dpHXCNWuiJuA1ma8dFHH5ljsJmAWrlypezdu5dkFUXAKwhoOUIgUOecombUimesECpJGe0asUKeeLYaNGiQsLffCruwcHidlK2RZJX2q\/0syZopx+oyah23cHOOenlR8GsfcY1YWVPKpz+21ccff1ysZGRkZG0WyEtD6DOKgCIQXwTsqNULbgbjW\/PQb3ONWLGjMknFov7QxdOUREbA6\/azRMY+2nVLv7ikxGLU6tc+Endi\/fnnn4Wzrvr27Wt2Sl111VVyxx13SPfu3Y2Qhk6ohse\/Kov\/L774YmEjAE5XLPisKMAPKw5dmjdvLsuXLxf08SGAfp06dYxbwYMHD4bKXuMVAUUgDwiwQoDHJv3vFy5JL3En1hNPPFFuu+026dq1q7DzCleBkCz3CGnohGoZvFXhEHvSpEnGNsuOreHDhxtnK6wo4H7+\/PnSsWNHefXVV42LwKlTp8qUKVOMg5YZM2YIrgRD5a\/x8UPAr\/az+CHkrzdBrqt+KRTVQvu1j8SdWDnnCpeBOEzApyqTV4FAQHCugrC\/H51QrcP6V5Zo4XAFHXZvEWa0SpjjXgKBgOA0mxMG8BnACFb9sYKWSo4IqEKeERiaXlE+336C7Nh\/TJ7zSJQH406srAZghFqzZk3hEx2vVkxc4TsgN6BCkDhumT59uuB3FbMAvllxnM1qAkwKVapUkZtuuklwes0It3Tp0mL\/45m1a9fa27heGzZsGNf3ef1l1oTj9XLGq3yJ0D9wL7jql+OiBplf+0jciZXVAHzqt2rVyvhg\/de\/\/mU8T4Wzq+JbNbs\/Vhy34AKQVQTYZTds2GDcld1+++2ydOlSueGGG4SRLWSLi8JwLU3e8RDKEI\/3+OUdEIlfyhqPciZC\/9j19VyZvXK74PQoGpjlpY+Ao9sSd2KlwgUKHPlajsAON2J1+mNlA0G\/fv3IxvhdveiiiwTTwgknnGB8sFpfq3z+856SJUsKBGweOPSHsNU5dCv4cVVZk+Q4aP2j9RtYO+Y+WTLkX+asuWjlmZd8+G27KUcynJslyeW7WaL11VdfCc5amPHHhsqnPp\/\/EOyqVatMTitWrDBnXuEyEB+sjJQRwk5\/rEZZ\/ygCioAiEEUEXCFWZuUhQj4VatSoIczic+UeIgx1\/Ar1xpdqs2bNpGXLloKddsKECYLfVWyvmAweeeQRqV27tnFDiC8C8iVPPk3UHysIqigCikCsEYg7sTLzzxEroYb3pKETquKBQEA4egGfqxAyk1elSpUy6pUqVZLJkyebZVjjx483XrMCAW\/7Y33ppZdk2bJlpvyjRo0SPLCz1ZfRtYnUP7FCwBf5stolPT1dsDV+\/PHHviizFlIk7sSqoB9GALMFy8Fw1k3Mpk2bBPsx620ZWX\/yySdEqyQ5AjgoYk02B23OmTPHTPQmOSS+qL4Sq0vNxEoFJuHYIUYRsBlzbhc2ZBZFb9y4kWiVJEeALd\/0B041ZjUNJ28kOSS+qL4Sq0eaiWO\/Wdlgi8MI1ob1Gj0E\/JQTK2XYfp2SkmKKzeoZ1mubG\/3jaQSUWKPQPPwA+Ky3tlKyxDaGHwN8FLRt21ZYZ0s6mxvwZ\/Drr7+iliWFChUyfg1sBBN5NqzXxEOA1SzsPrQ1Y70167HT0tIEnxZs9WYHIv2C\/oUejotOPfVUgioeR0CJNZ8NxA+gXbt2xheBzYofAn4LIFb8y7JZgR8Njry\/\/PJLGTt2rDktwepzZasuNlZ2prFkjPO\/iFdJLAQw+bDTkAlKRqC2dthS+ceXSVkmXnGnya6jYsWKCf2B7dros1PRPqNX7yJQwLtF80fJGEHg8+Css87KKjA\/Aj7lWepFJJsYsJVt376d26DC+V9st2Xiiu26bHAIqqiRkSPgoScYhVavXl3++c9\/HlGqWbNmCcsICxYsKPyjykoXTtegPwwbNkxat24t9Cd8GB\/xoN54EgEl1nw2S7Vq1QRfBYw4bVZbtmwRPuGYbCCOUQafccGItXfv3lK5cmWziwzPXrNnz5ahQ4eazQ08q5JYCLDemv7CP7a2ZpiFmJSyfSgQCAimoHXr1hmSZQTLP7Y8Z5\/Rq7cRUGKNUfsw68\/oJEbZa7YJhgB9hX+ME6xaSVsdJdYYND3mARb4W8cyXPft2yd87sfgdZplUAT8E8mXDZ\/4THhSarZqY1+tUKECtyo+RECJNQaNxs4x\/Bcw80v2XPEZy2cg9yqKgBOBQCAg9evXF+ysrA5gDTPmpHPOOceppmEfIaDEGoPGOvbYY43\/gtdff12YhBo3bpy5Jz4Gr9MsEwABHLPzDy9LrViOh\/tLNowkQNWSsgpKrFFodj7j2JrKJJTNjlldJh0+\/\/xzs7yKe5um15gh4JuM8SeMO0xbYFYD4FuY\/sIEJiNYm6ZX\/yGgxOq\/NtMSKwKKgMcRUGL1eANp8RQBRcB\/CCix+q\/NtMTZENBbRcBrCCixeq1FtDyKgCLgewSUWH3fhFoBRUAR8BoCSqxeaxEtT+4QUC1FwMMIKLF6uHG0aIqAIuBPBJRY\/dluWmpFQBHwMAJKrB5uHL8XjVNzcfTNAYlWOI4m7\/XyxpMs4h85cqTglYpdUvjkdZaMer\/wwgvOqKwwPgAGDx4sbF3NitRAwiEIuG2PAAAFHElEQVSgxJpwTeqtCuHUmZ1EVjhpwZYQZyMcPWLv\/XCFTN99913jKjIv5S1XrpxxKTl\/\/vy8PK7P+AQBJVafNFQiFZPRXOfOnaVFixYyb948Wb58uVx11VXmSBKOsWFUR31nzpwptWrVEka73bp1k169esnWrVuFUSJXdBgtMkKEpMeMGWN02W\/fp08fM6IknTw55QG\/De3btzcnnTJiHDhwoPHlgG9UnElDmPfee69wAgR5M7JECFvBoQ6+dXFGbeNCXRmdU3YEv72UYffu3VK3bl3h1FVO6g31rMb7GwElVn+3n+dL36FDB+O0GcfNnPfFuV8UmlMWcE6Da7whQ4YY594c7wyxDRo0SFavXi0jRoyQCRMmGK9PEBmjRZ4NJjy7YsUKwSH0p59+Krjie++994wqpzcwckYHn6eLFi0SjkJZv369fPLJJwL58nmPu8fNmzcLpz\/g9nHJkiXSsGFDk4f9g\/75559vb83VWUfqCWmSwOickTr1TE1NFY5joVzUhfrjwQo9lcRDQIk18drUUzUaPny4rFmzxgjnfVlHNfXq1TOnJKxdu1YgH0Z1kFJ6erosXbrUjGJxtQgJBQIBwWkJzm5CVQ5ihMAgao644RSGzMxMo85oETeOODrh\/ZgfOFsK0jz++OMFr1LYTBkdcy7ZkkOEyvljvC+76z4Il3iT8V9\/nHWkrtdcc81fKSKcYcYpEU2bNhXszSRA7ikpKaInroJGYooSa2K2q29qxecwJ5NCppASB+dNmjTJHLbI572tyP79+8NO+PCJ\/cADDxgCJx9Gr70OmQ7s89mvmAI45cHG79q1y5BggwYNzAgZM0TNmjUF4rU6kV4p\/2uvvSannXaa3Hrrreb4nUjzUH1\/IqDE6s92S5hSM5nDCQsQKkQ0efJkefrppwVfpHyq4\/SZeEa1kDA+bRl5ciID8dg8AYPRIKNQRpSMEnv27CmYBEgLJpC5dSy9YcMG6dKli3DuVNWqVY0dF9svo+jsz3IKRDiThFMfcwP1wQRAmW0aI2bKXqRIERul1wRDQIk1wRrUb9XhU\/\/+++8XJqBq164t+LDt2LGj8EnOJBWf1ZdddplAsHyC89nOPaeWcrgetljIFlMBZMkVM0OxYsWM+SAUHs2bNzcjSCaS2rRpI61atRLMBZBdlSpVjF2Y++zP8w7sxBBj9jTnPSNiRqvff\/+9MHkGSXPF3osdl7owknU+o+HEQUCJNXHa0nM1wZEzRJe9YBAnYuOxbTLpxIhz7NixglPwQCBgjohesGCBscHyKQ2ZBQIB4TRbbKpTp06Vl156SZ544glhRMiok3gEswBxvJ9y2HfxXuIgtv79+wvLnngvx1EHAgGzIgDia9mypSFe+5y9soKAkS2jXPKgvORn07nyPspCGnZeRtsI\/2hAptSVo67zY2bgPSreRUCJ1bttk3wlc7nGLOGCYBkBV69ePWhpINObbrpJsMFC9EGVwkSylGzPnj3CqDuMmib5HIECPi+\/Fj9JEGA2n9n1WFaXQyDff\/99MwION5pklQFrUgOBQMTFwaaM2YPRdMQP6wO+QUCJ1TdNpQVVBBQBvyCgxOqXlkrYcmrFFIHEQ0CJNfHaVGukCCgCLiOgxOpyA+jrFQFFIPEQUGJNvDb1Q420jIpAQiOgxJrQzauVUwQUATcQUGJ1A3V9pyKgCCQ0AkqsCd283qqclkYRSBYElFiTpaW1noqAIhA3BJRY4wa1vkgRUASSBYH\/BwAA\/\/82YIvXAAAABklEQVQDAHDcEyQEIqLlAAAAAElFTkSuQmCC","height":206,"width":342}}
%---
%[output:6ac8e065]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVYAAADOCAYAAABl7SoZAAAQAElEQVR4AeydCbxN5frHn03dXCSEFMoRQql0M5Sh\/FNEQrduuKVug0ypUKdL3TSYVTJEpUEDjVJuUYlwdEkSijJnrK4pDUpy\/76v3t066+xh7XP2sPY+j4\/nrLXe8Xl\/a63fft7pWUX+p\/8UAUVAEVAE4opAEdF\/ioAioAgoAnFFQIk1rnBqYYqAIlBoEXA0XInVAYaeKgKKgCIQDwSUWOOBopahCCgCioADASVWBxh+Pp01a5ZUq1ZNJkyYkBA1V6xYIWeccYb069cvIeU7C\/3hhx+kc+fOct5558m3337rjEraOTjSXtqdtErjUBH6ojfCudci\/YC5U1d0pw08187w9DsPrbESa2hcCn0ohAfxQYC8lIkEJJl10Q5e6vHjx8uDDz4odevWJSisxKobRJGoH0Duw5AhQ6R06dIybdo0Oe6448Lq7fcIcL\/wwgulb9++wv3wu76x6qfEeggxrDReBiu8HIeChQcZYkF4kG081g7xVpz5ISNeRuIohzy8wJTBOUfKJT6UUDbpEH7Rwz101EldL7zwgrFkqYvySE8+8iOU5wy39aMD55Rh9SUdwvUVV1whmzdvloULF0rXrl0NFsRZoT5n+aHKQ0fSWCGPzW+P\/\/3vfyVUXeiAbjYvZdk8HLkmjraCL+e2rcRTF2FWuCYcmTRpkiGn008\/nUvTNrCwaamX+pFYdKMOsKLQ4cOHB3sXhNuyOXJNmlBCndRPOoRzwsCXsrkf3Jf27dvLN998k6cI0pKHvAg4uRO999575pkh3okZdThx4Jww8qMz6XneKN+W63zebrrppjw9EWc8+Z31XXPNNRQt3A9zkkF\/Cj2xcqOnTp2a65byAPMg2UAe5j59+thL4aWx8Txgzvw89Lfeeqt5WW2GcePGGYLimrKef\/55TvMIZVK2jfj++++lV69eYbvL1HX33Xfb5CYd6clnAymPcu11vI6QUpUqVWTt2rWmyJ9++km2bt0qlSpVkuLFixtSceJCIjeuhIWSH3\/8UcCQ9tl4yuJecc2Ra85pK\/hyboX2Upe95mgtI4hn8eLFQT2JGzhwYPD+cE291E+buHZKNN2caTmPpAvxToHEqJf6bTjnkPuOHTtsUNhjqPzgBF42E+U5nxksd8iPeDcOPKuEEWeFvJTBNVg6nzcImzzEIZR71VVXCfeIa8T5PGJxY31zPyiL+DSRqGoWamLlZk6ZMkWOPvpoeeONN2T9+vVyxx13GNBmzpxpjvwJF09+HopGjRrJ8uXLjXDOw4WQFyGM+Mcff5xLmTdvXi7iJZCX4qmnnuJUSIcul112mbEayWsiQvxBX9K2aNFCeIl46MlHGOWQxdkWrqNJhQoV5JVXXhGIE90pp2TJkrmykaZ+\/fpC+8EBHam7WbNmAiF5wZUCy5cvn6euQCBgSJq6KRcs0QXcsNI4khe9iCcd11Zor\/ue8XJjGZF\/z549gp60Cdy3bdtm2ko9trzPP\/\/cEIIbh0AgvG7ogU7owX3p1q2bRNKFdE6hfoRy0APhHFz5AaNsrsECEqM7HSk\/zzQ4gBc\/CDYt5diywQVMwuFg76\/NS\/3kHTlypHne0c2GUa5Nx3HBggUGQ8KdzyPPOfXxA8wPMfcDHciTKVKoiZWbyU099dRTJSsry9xTSIkHlwfKWgnO+MaNGxsi5mXcuHGjkJ+XAQsO4dwU5PhjX2J+oXnQHVHBU8gIi4+6KYcIHl4eSEiTa7dQFvrYcF4+zlu1asVByEd+yjEBcf5TvXp1035wpG6rD9fg4sQtFK7h1IG0586dK5MnTxase15cXmDS79u3z5CuxQlyvO6664gywgvLvYEw2rVrZ7q8WEkm8tAfdCPu0Kn5T\/4TTjjB\/IBRD9hT3rJly0KOv0bSzRTo+BNNF0dScwqGnNjnBd3QhTAImmMkceeHeGkHOJYoUcJkdeJGu03goT\/URTpwhzjBIdKzfChLsLdi9QU\/hDjE6kPvgWEAjoQXBinUxBqvGwxpQGBOgdTiVb5fy7Gk\/u677xor3EmkBdEZQmJ8j5cR8uYFhxBiKZP05HPeE35g7MvuLItwyMSSAgTAOCKWuDMd5\/nRLZwulOcnYVgLzMHIiUdBdOTHFsvZeR8gcIjcGhMMB2B0FKSeZOSNpY5CTazcTG4q3b4NGzYY3Gx3mm5uuXLlTBgvKMIF3UksHn7tq1ataiZBsG5DvYSk9yrubpHzBWaczks5kBDpeDE4MrbGi8ILwzWCVcwDTXtpN2H5Fax8yHTJkiXGirSWi1dcw9UL1gjdaX6geMmtxWrzcE04ONG1tOG8sNwbG2\/D7dFiZK+5b5AoZGq7rNQbLj96IaQJp5stO5ouNp09Wt3outMuxLbN9kJs2lBHm9\/ef54b7j8\/Us6hgFB5GQ9leAEjoduhIYxYng+rL7ggtnz04V1hSMCGOY\/0HujZ8K7RE3DGpft5oSZWbmanTp3MOJC72+h+kHnxeEghXm468eTnoeAlxNohHuFF5YUlnVfhJYSYeBDRxXbFKBfxUg4WJBYCXV\/04Mg1s6+W7Kyu1EFd0crlRaHtvOTutFZn0vCCUD9pwMUrrqS3QjnUVapUKTPcgv60gzCbhh87yuaacIsT11a4N5wTT34rEA04gAnxCLpyD6mbskhLvaQhLWkQ4ikvkm6ks0IZ\/LBF0sWmtUfuM0Jd6IJwjsXLuU0X7kga0vKM0g70JS3DCXYogOtQQlsxMmxeL88HJEx96Ejdtj5bPs8DOIIF+lgBF9JgpFAnP1JcZ5IUamLlRvLrzAPCuRUsF6wRe83Dc\/\/999tLM8Fl4+lGOvPzII0dO1Z4YYMZPJ64daHeUaNGCQTmpQjG1Fh65EzLNeHow4yujevZs6fwEttr99Fa0O5w97V9ebBcsWBtvLsthLtxJQxx11WnTh1hjSNx4EnXEV2xsLGkmGnmmngwct4bwrg31MW5FV5ewi2BWCuLeGa+bXlcIxa3WHWjfHSmDIQ6w+lCvFO4z9xv2mTDOWcCjftnw8IdScOz56zftjtcHhtOXvuDRRg681zzQ0zPgDC3kCfSM8Vzxxi5Ux\/K5NmwFjL5Kcddtm+u86lI0ogV66hLly6C1SAh\/k2bNs1MNNhftfxYfSGK9RQEOTrHgHgZ3Bl50W0aHgxnvDM\/kwU8UMRTDnlsesKJhyh4iUjjFmdZTCTYh85dFukoizKdZdh01ItwbeM5Jwxh+RF62Dooh\/Iol\/ToRzxpOXJNuFtsvlBpKIv8Vqif\/JRFelu3vSYd4VzbvOgE6RHOOeSNZYSVxMtPGUuXLqVYoetpTg79oS7Ks2LvAXhCIHZI5FBS88NF+TYtR\/K740gTSTewQNCTMmydlMW1FRtO+W5BP9pk03JOGOmoGx2cYYQ7xVk\/Zdi6QuW1GKMfZZCWPAhhzniuCScNaRHeZe4F9wfyveiii4QfP2b6+UEijVsfynSGUy7XmSZJIdb\/\/e9\/wvKbcGMtgLplyxZhPSI3D4n08JBepXAiAEEwZELrean5Iab7ilWEtUh4NMHKZuiCfNHSanx4BOj+Y1HzI8e5HT5gjJv7FD5n5sckhVj5FVu0aJE0bdo0LKLbt28Xxs\/CJtAIReB3BLCa6FL+fmnGY+lyYh3ZsEhH0nXv3l2ci+Mjpde40AhgSbuHHrgv1ioNnSsdQguuY8KJlUkPwMe6CEeczFiy9pDxlho1apixv+nTpwuWbsGbmP8S+NWN1vXKf+masyAI8PLSs0HoekOWsZQHOecnXyx1FIa04A6O3AeE+1IY2h2tjQknVoYA6C40aNAgrC779+8XZi0HDBggX3zxhQwbNkxGjBghn3zySdg8GqEIKAKKgF8RSCixrl69WubPn2+ceAQCgbAYlClTRrBqGSooWrSo2W549tlnm+2S7kyMqalUyzXRp3goHvoM\/PEMsHLEzRseruOaJKHEumnTJsnJyZGGDRsaImCygCEB54J1WrNq1SqzP\/6XX37hMihMSAQvHCd0OTJJaFomtYe2aJvWG98TYOFnycT7dMEFF9CslEpCiZWlFM6HioFtlsi4x2EYy2TbG7OLv\/32m9keyTo3Fm6nFB2tXBFQBBSBfCCQUGKNpA87k1irylo4xmCZuMKJL5NXjLX2799fatasGamIjIl7\/\/33M6YttiHaJouEv4+ZeJ9iQTxRaZNKrFiqWLE0hqUarFW11ywyxoMPFi7DB82bNyeZiiKgCCgCaYdAUok17dBRhRUBRUARyAcCSqz5AE2zKAKKQBojkATVlViTALJWoQgoAoULASXWwnW\/tbWKgCKQBASiEitf0ezYsaNZtI\/zi2iSBJ21CkVAEVAEYkQgucmjEiv79Vm436NHD8nOzo4opEmu+lqbIqAIKAL+QyAqsQYCAfMFS3YztGnTRiIJafzXRNVIEVAEFIHkIhCVWI866ihhTekRRxwRVTM+ZRw1kSZQBBQBRSA5CKSslqjEyjDAE088IXinsl8A+PXXX1OmsFasCCgCioDfEYhKrFih+EZ98803zZBAnz595C9\/+Yvcd999ghcZxmD93kjVTxFQBBSBZCIQlVhRBld+tWvXlkGDBgmfOn7yySeF1QIXX3yxsCX19ddfF7ViQUpFEVAEUo2AH+r3RKxORY888kjB69To0aNlxowZUrp0aRk1apTs3r3bmUzPFQFFQBEotAjETKxYqrj+wyk1FitfzYRYjz322EILojZcEVAEFAEnAp6IFWv0hRdeED5vyxcusVRx6\/fxxx8LHqvq1asnDBc4C9ZzRUARUASShoDPKopKrPhNbd++vYwZM0Y4zpkzRxhTbd26teCg2mftUXUUAUVAEUg5AlGJlc+jsNxqwYIFws6qSpUqpVxpVUARUAQUAT8jEJVYv\/\/+exk4cKDs3LkzajuwbqMm0gSKgCKgCMQFgbyFbNr1sxwsXi5vRJJDohIr+uzZs0eGDh0qjKtGEtKQXkURUAQUgVQgsGDdHtl70TAp22eOTFn8dSpUMHVGJVaGAm6\/\/XbBDwATV5GENKZU\/aMIKAKKQIoRqFKmWMo0iEqsf\/7zn42vgEjOV5xxKWuJVqwIKAKFAYG0aGNUYk2LVqiSioAioAj4CAElVh\/dDFVFEVAECoYAk1cFKyE+uZVY44OjlqIIKAIJRCDdilZiTbc7pvoqAoqA7xGIiVhZ04rbwFq1asl5550ns2fPljvvvNN4uvJ9S1VBRUARKFQInFjWx6sC7J3A7+rDDz9s3AM+99xzcvzxx0v16tWlePHi8tBDD8mBAwdsUj0qAoqAIlBwBNK4BM8W644dO2TlypXCmtaTTjpJihQpIsWKFZNrr73WOLzGUUsa46CqKwKKgCIQNwQ8E2u4GvkWFj5aw8XHGs5wg\/0ETKx5Nb0ioAgUbgQ27drnCwA8E2u5cuWkTp068sADD8j69evlt99+k2+++UYmTJhgPtlSpkyZAjeI4YYpU6YIDl8KXJgWoAgoAmmIQGao7JlYA4GAB5mVGQAAEABJREFUMHGFd6urr75aFi9ebNwIMgTA8ACWa0Eh+fzzz2XRokWCE+2ClqX5FQFFoPAhsHnXz8FGp8XkFdrif\/Wee+4xY60QIGOu+GmNh7X6ww8\/yNixY6Vr166CdUx9KoqAIqAI5AeBVJIq+nq2WPfu3SvDhg0zHq4gV1YI3HvvveYaj1d8YWDr1q2UmS9hCKBKlSrmM9v5KkAzKQKKQLoiEDe9N+0+bLGemEIHLDTGM7GSmJUBb731lkCy9nr69OlC+MKFC6VDhw4ExyyrV6+W+fPnG2s1EAhEzV+tWjWzEmHDhg0ZcdyyZUtGtMN5P7RN6fFsZtJ9qnpaA0m7La3M\/O\/fv1\/4kCBd9sGDB5vzxx57TPCANXz4cPP5lqisGCLBpk2bJCcnRxo2bCiQ5tSpUw3J9uvXL0RqMZNnWVlZkilSuXLljGmLvSfapvR4PjPpPj0\/7d0gXzSuXvDJ9GBh+TjxbLGyDIovtPLiOOthkwDhxLvjnOkinbdo0cKQJasNkMsuu8yQNh8qjJRP4xQBRSB9EYi35sPf2RAssvHJpYPnqTjxTKw4vGbyii4\/y6JQliOTWJxjta5du5ZTFUVAEVAEkooAQwA56\/aYOov8tEOaVE8TYoU4e\/fuLY888ojpsjdr1kzOOeccueuuu4TlVxs3bjS7skzLCvgHSxUrtoDFaHZFQBEoJAgMc1irxb54M+Wt9myxoulpp50mM2fOFL7amp2dLSNGjDBjoxdffLHUqlVL3n33jzEO0qsoAoqAIpALgQRc8G0rhKJZZvWnTQs4TanERKx0\/RlLxQFL\/fr15ZRTTpHNmzdL586dhQ8OlihRIqWN0coVAUWgcCEw7J2N0nPKKtNoSHVsx9rmPNV\/YiLWiRMnCoTaqFEjscJEU9WqVYXx11Q3RutXBBSBwoEAY6qQqnMIoFP941M+tmrR90ys3333ncyZM0cmT54sL730khlXXbNmjfTq1UtOPfVUs+TKFqpHRUARUAT+QCB+ZxAq3f5LH10qllSxVLNbZkl2y6rxq6iAJXkm1l9++cVUxZKq0qVLy86dO2Xfvn1y6aWXyty5cwXiNQn0jyKgCCgCcUYAQh32zkaBUOn6c00VkCrdfz+RKnp5JlZWBeBohSVVECtjrQiFQKqWeLlWUQQUAUWgoAhAnjlr95gx1DMf+I+xUAmjXAg1+5CV+uld5\/im+49eVjwTK+tYO3XqZJZUbdu2TU4++WRp166dkYoVK0rZsmVtmXpUBBQBRSBfCECcdPWxSiFTLFSubWEQaqf6FeXNHvV81fW3+tmjZ2IlA8uq6PYzpoqrwAcffFBGjx4tQ4cOFaxZ0qgoAoqAIuAVAYgUq9R088ctFcgUUnWSKWVBqNZCHdeptnBNuF\/FM7HieAU\/AT\/99JMhUYYG8Jt6xhlnyMsvvyy4\/fNrI1UvRUAR8AcCECmkCXle+juRYpUyEWV3TllNIU\/I9M1D1ild\/mwfTU5ZHcMdoxIrE1SsBnj77bfl1VdfFY54uLLy+uuvm3AIN1wlGq4IKAKFAwHbSggUcZJo2T5zclmkoYi0ycmlD3Xxs0xX35Jpqren2jbFcoxKrFiieLDCo9VXX31lnFHjl9UKflivuuoqKV++fCz1alpFQBHIEAQsgTpJlC49gmVKuJtEaToWqZtI3+x5eOw0HcmUNlmJSqwQ5osvvmgs1b59+5rjvHnzxArbWK+88koJBKL7UbWV6lERUATSCwHIE4EkGQ+FMOnKO61QwogPR6IQKRNPjJHa7n2mEKn7bkYlVj4aiFvAn3\/+Wdq3by8cv\/32W3EK8aRzF67XioAikD4IQJxMJEGOCETpJE9rgTIeSnyQQF1NhECxRN0kStceUiU83S1SV5PzXEYlVjYCXH755cEtrHYrq\/NIPOnylK4BioAi4BsEIE4EUkQgzoGzdoiTPJlIIhwhTTjypFGhCHTXQ80FAsUSLSwkChZuiUqsFSpUMDurcEAdTliCRTp34XqtCCgCyUMA0sxZu0cgRARyRJzEaa1OwkkzfdUP4pU8s1tmCWRJN14JNPJ9jUqszux4t1q6dKnwMUF8BLB+FbJ1ptFzRUARiD8CkCZiiRNiRCBNyNKOdbotTsgzEnGiqdvyDEee2S2ryh\/deHKqhEMgJmLFF2vPnj3lwIEDUq9ePfM5FcZdZ8yYEa58DVcEFIEICECWiCVMiBDCRNykCYFa4iQdAmmSP0IVZjG9HfPMdliddNmX3Fw1T9ddyTMSmt7iPBMrfgGmTJkiDz\/8sAwaNEiuv\/56810qnF4TTry3KjWVIlA4EIDwEEuaztl0S5qQJWIJE0KFMBGvpBnO4oQ43V32bIfVSb7CcSeS30rPxMpGASzVrKysXFpyTTjxuSL0QhHIQAQgSsSSJQQYijBt19xJms7Z9PyQZrbD2nSOc0Kgzskia3EmgDgz8I4mpkmeibV48eJy5JFHyqpVq3JpsmHDhuAW11wReqEIpBECkCXiJEysR8Ral38Zs9HsHHKSJfEFJUyIkHFNBMKEKK2lybklzWyHtZnpy5XS6NEJqapnYuULAdddd52wSYBPsfTv31+6du0qN954o+D1Cu9XIWvQQEUgRQhAlEg4srSEGcq6hDCxRhEv1iVNxEJE7HgmhJntwcqEUEmLQJiUQXkq6YuAZ2Klieedd57ZedW8eXMupXbt2vLGG28IXq9MgP5RBBKIACSJQJQIpIdAgghEiWBReiHLWAnzL5WKmVlxCBAyRLAwkVAWJvHZGWBlJvCWZmzRnol19+7dMmHCBPnxxx\/lhhtukMGDB8ttt90mjLFmLDrasIQjAFEiTqK0Y5aWLEMRZbjJnljJMpJ1STfcSZiPX1bRrOOEMCFXBAsTSThQWkFaIeCZWIsWLWqWV2GdtmjRQlgNANmmVWtV2YQjAEkiS7b+HFyo7ibKaFalHbPEGvVKlDSMLjQSiiwhQyxLN1na8Uvi3dYlZVGuiiIQKwKeibVUqVIyfPhwWbFihXFszZGhgSuuuEJmz54t6isgVujTIz0kiUSyKEMRZdepX5tPamB1uokyXmQJUSJeyLJT\/YrmEx5Klh6fO01WIAQ8E6uthZUB9evXl0ceeUReeeUVs1Lg3nvvNR8XtGn06E8EIEgrkYjSdr3DjVMmiiixGiFKhC44Amkialn685lSrUIjEBOxYpWy3GrAgAHSoEEDwQ\/raaedJvhkxb1g6Co0NFEIeCHJUNYkxOkco3QTJeV61RkLEHF3vwe2KGfGIyFJBHKMRpTWqtQxS6\/oazq\/IuCZWHEN2KpVK+nQoYPwRVY+07Jw4UJh2VXlypXD+mP97LPPhHzVqlWTJk2aCF8jCAXGtGnThDRWGGbANWGotJkaBqEhkazJUEQZiSRj6XaDKySJuIkSaxKBJJFoRNm2dkkzgw5JIpRJ+Sp+RkB1ixcCnomVb1yxEoCx1ZEjRxpfAUxoRVKEba7k6dGjh6xevVqys7OFLw9s3749T7YtW7bIuHHjzAQZjl0yxWMWRIk4yZJxRwSSRNpO2iKJ7HZDapGI0pJkLBYlZea5iRqgCCgCBgHPxMoGAcZWGWM1OT382bx5sxw8eNBYqnzFtWHDhmZMdseOHXlyQ7blypXLE+73AEgTYQYbsYRJdzscWZIOwZpEtu09ELKZbvLiGolEkqGsSYgz1Bil7XpTZkgFNFARUATyhYBnYs1P6XXq1BE+61K2bFnB5eCiRYvM9teKFSvmKo61sdu2bZO7775batSoYZxqT58+3eTJlTDFF5ZAneQJgSKEIZYwSRtNXQgNcS48z\/59pw7d7rEdawc\/qqbWZDQ0NT6fCGi2BCCQUGK1+jJ8cOaZZ5rtsIydli5d2kaZ4\/79+6VEiRLCpNgXX3xhhgtGjBghn3zyiYlP1R\/IEaKEMOmyWwIlDEuT+FC6QZaItSwtWWJNIliQu373tM65c+F59u87daw1qeOToRDWMEXA3wh4Jla8VzFZxdHZJK6XLFkiv\/76qzM413ndunVl2bJlgj9XK84EZcqUMV9\/bdq0qTBu26xZMzn77LNl8eLFzmTBcya4cP6SKHk1Z5Vc88Ri43ADUrVEGlTg0MkJpY4QhEkaZsARCBL\/lq\/\/vaIgj7QuLf0aFZO\/1fyfNCq3TyoV3W3kt++2i1N3xped15lwrm3akOse+\/WeZtJ9gheQQ69nyv9HJVaWWLEigAfjwQcfNA8Ls\/VWvvzyS7O9NdQurJycHOnXr59ZRUBLaTTLs7Zu3cplUFjCxSoDVhsEAw+dhHPswuQWW2njKUWPOV4W7viz3PL2HmFxO5+sOKRC8D8WKFYkXXSszs8GNhVk0o31pXfrukYub1LbbPGNVS9WVcSax+\/ptU1Z+XoWknVfbT2ZdJ\/gBST40qbwpEi0uvlIIB8LvOSSSwTLlGOjRo3MOChHdl5hkWJ1ustibSuW6vLly83OLJZerVy5UpgEc6ZlYgxnLljEEPm8efPMDi93OmeeeJ7Tpb\/00aVmpxBdfFs2ZJp9aMwTIv30rnPMukzItUn13EMZNr0eFQFFQBEAgajEykcCWfrExBOkypFfBStr1qyRgQMHmtl+CnRKzZo1pXfv3nLzzTebSSmct3Tv3l3OOussweJlvHXWrFlSpUoVM3E1ZMgQk46xVtbHkt9ZXrzPIVT2sTN2yjnlQ6aQpyXT7JZVRYkUZFQUAUXAKwJRiRULcteuXcKEE7P2zO5Dik5hqIB07koDgYC0bdtWsEQhYo5cBwIBsYSNQxfyYf0y\/ko6hhCsa0LiEiEQKVYqu45s+ZZQ6e4rmVpU9JgxCGhDkoZAVGJlKKBXr17CWCpDAhCgWwgnnaTJP0uqHFEZKxULFULlnDAVRUARUATyi0BUYsWynDx5sjDpxJAAFqVbCCddfpVIZj7I1Nn1Z0kUpKoWajLvgtalCGQ2AlGJNZOaD6nS\/bdtymZiqmc9USvVIqLHzENAW5QKBDwTK2OqTDaxZMothBOfigZ4rRNS7TVllXAkD+Op2YcmpjhXUQQUAUUgngh4Jla6+nT57TDAunXr5MMPP5SbbrrJuA9kaVU8FYt3Wc5F\/nT\/GU+Ndx1aniKgCCgCIOCZWEnslEAgIOz579atm5nYCrVBwJk+leeQqp39p9s\/tlPtVKqjdSsCiUZAy08xAvkmVqs3+\/z37t1rNgDYMD8d6frnItWOtXVM1U83SHVRBDIQAc\/EylrVjh07Cvv4nXLRRReZNamhdl75AS+sVcgVXRqfXFoX+wOEiiKgCCQUAc\/EyrZTxlNxVu2UZ599VvjmFf5WE6ppPgqHUJ3Wqo6r5gNEzZI2CKii\/kHAM7HyBYHzzz9fzjjjDPO9qzZt2gg+AipXrmw8UvmnSX9oYkmVkOyWWRxUFAFFQBFIOAKeiRVNJk6cKBdeeKHgM5XrDz74QFhqNWPGDC59JVirDAOgFBNWLK\/iXEURUAQUgUQj4JlY8ReAwxR2YeE3FSqnGIkAABAASURBVMW6dOkiEyZMkClTpgjftyLML6LWql\/uhOqRUAS0cF8i4JlYDxw4YLr8lSpVytWQ6tWrC3E4vM4VkcILtVZTCL5WrQgoAuKZWI855hhhAuu1114LLq3C0xUeq\/jAYPHixX0D54J1e4K6dKp\/fPBcTxQBRUARSAYCnon1qKOOMr5VX331VeH7VSy5wq\/qoEGDpE+fPoZ0k6GwlzqmfHT489o6tuoFLU2Tfgioxn5HwDOx0hA8XOEz9fnnnxeWXI0fP17w9s9KAeL9IAwD2K8AnFimmG4G8MNNUR0UgUKGgGdi3bZtm7BBgI0CECnLrfDLiiXrJ8zsSgB0ukOXWAGDiiKgCCQZAc\/EihOW2rVry8cffyyMrSZZT8\/VLVi726RlGEB9rBoo9E9mIKCtSCMEPBMrTlb4GGDfvn2lYcOGuba2Wks21e12DgOwfTXV+mj9ioAiUDgR8EysrAjo0aOHjBo1Su655x4zxso4K8JWV+JTDaFzGEBXA6T6bmj9ikDhRcAzsbKllQ\/8MbbqFsKJTzWMOgyQ6jug9ccbAS0vPRGISqxMVvXu3VtWrVplJq9YZuUWPwwFOIcBWA2QnrdDtVYEFIFMQCAqsdLF79y5s3FqTZefrr9bCCddKgGBWG39nRropgCLhR4VAUUg+QhEJVa6+Cyrwt9qKO9W5557rvhhKGD4OxuC6FUpUyx4rieKQNohoAqnPQJRidXZQj97t9q0+2ejqi6zMjDoH0VAEUghAp6J1c\/erRgGQMBRl1mBgooioAikEgHPxIoHq6JFi4ofvVup05VUPkJad3wQ0FIyCQHPxJpo71ZsPmjVqpVUq1ZNmjRpInPmzPGMs3W6QgaGAjiqKAKKgCKQKgQ8Eys+AVh2lQjvVjjJHjx4sLABYfXq1WbzwbBhw2T79sNeqryCA6kiXtNrOkVAEVAEEoGAZ2Kl8kR5t9q8ebMcPHjQWKp8lJAts\/h43bFjB9VGFMZWnd6sIibWSEXAXwioNhmKQEzECgYQXry9W9WpU0defPFFKVu2rODgZdGiRQLBVqxYkSojCsRqEzSuXsae6lERUAQUgZQhEDOxJlLTFStWyJlnnik4euEjhaVLl45anXPiSlcERIVLEygCikASEPAVsfI57WXLlgnOtK2EwoAJrg0bNggy67PD47AnlDpCKhXdbcIITyfZsmVLWuodCWNt0+HnMxRGfgrLpPsELyChOCPZYb4g1pycHOnXr5\/88ssvpv2Aw3ju1q1bzbX7z\/r16yUrK8vIkq2HNwZUK1\/SXNvwdDpWrlw5bXUPh7O26fDzGQ4fv4Rn0n2CFxA3X6TiOiZi3bt3r7z00kvSv3\/\/XMIMPnH5bUD58uUFS3X58uXmQ4UsvVq5cqXUr18\/YpE6vhoRHo1UBBSBFCHgmVixJu+44w55\/PHHzQx+PPWtWbOm+VDhzTffLDVq1JAbbrhBunfvLnysMFI9TmLVZVaRkNK4lCOgChQqBDwT63fffSd8ReDpp5+WoUOHCutOreDtqlSpUvkGLhAISNu2bWXhwoWCKc+R60AgELFM58SVOl6JCJVGKgKKQBIR8EyseLk69thjk6ha9KqsY2tS6vetQEFFEVAE\/ICAZ2ItXry4cQ84YsQIWbNmjXz77bdBwRn2b7\/9lrL26DBAyqDXiiMioJGFFQHPxLpz504ZO3asvPXWW9KyZUvBR6uVyy+\/XIhPJoiMr+qOq2QirnUpAoqAVwQ8E2uFChVk7ty5ZgyUcVCnEE6810rjkQ5iteXojiuLhB4VAUXADwh4JlaUZbspu6OYvGKJFUMCfAuLcOJTJbrjKlXIa70hENAgRUBiIlZ2Q11\/\/fWybt06Y73i\/BqPV5MnT046lM4VAUmvXCtUBBQBRSACAp6JFdd+U6ZMkTFjxsigQYOEffzsHsF6ZdyV5VgR6ol7lF0RwMSVrgiIO7xaoCKgCBQAAc\/Eum\/fPuErApCps74qVaqYSzYQmBP9owgUQgS0yYqAEwHPxHr00UdLyZIlzSJ+59KqtWvXCkuxEGfBiTxn4kpXBCQSYS1bEVAECoKAZ2Jlg0DPnj3Njqt27doJk1hXXnml2Yp69dVXG9ItiCL5zasrAvKLnOZTBBSBRCHgmVhRAAfX7777rtx5553SoUMHs6f\/7bffFnynEp8swWK1dTHGas\/1qAgkFQGtTBEIg0BMxEoZ+ARo06aN4DClY8eOkuz1q+jgXBGgPgJAREURUAT8hEBMxLpt2zaBTPmUCt+lqlWrliFYtrQms1Gbdu0LVqcWaxAKPVEEFAGfIOCZWJn1v+++++SEE04wE1isZZ0\/f77wDawHHnjArBhIVps27zrs3BpSRZJVr9ajCIgoBopAdAQ8EyvrVPfs2WM8\/eOYOhAICB\/7u\/XWW+Xrr7+WXbt2Ra8tTik27T5MrHEqTotRBBQBRSCuCHgmVlYFsCkAy9WtAe4EiXeHJ+raTl6dWKZYoqrQchUBRUARyDcCnomVdaqdOnWSu+++W1gJwHgrzlf4qkCzZs2EDQS4Esy3Jh4zHixeLphSl1oFodCTxCKgpSsCMSHgmVjp6o8fP142bdokbGPt2LGjIVkI9tFHHxVcByIx1Z6PxAeL+8vZdj6aoFkUAUUgwxHwTKyMq7744osyb968iJJovHJZrCeXTnR1Wr4ioAgoAjEj4JlYKfnXX3+VZ599Vlq0aCEst+K7VDhgcW5xJV0ixUmsiaxHy1YEFAFFIL8IxESszzzzjEydOlX69OkjEydONN3\/kSNHylNPPZXf+mPO5xwKUK9WMcOnGRQBRSAJCHgmVsZYZ8+eLTi4bt26tdStW1euueYamTBhghBOfBL0FWux6vrVZKCtdSgCikB+ECjiNRMuAwOBgJQpUyZXFq4DgUDSNghYi1WXWuW6DXoRLwS0HEUgDgh4JtZjjjnGkOqMGTPEjqnySZaFCxcaz1a4FYyDPlGLsBZr1ISaQBFQBBSBFCHgmViPOuooYZcVXxE4++yzhbWrTGDxNQHcCSZjg8Cm37eygpWuYQUFFUVAEfAjAp6JFeVr1Kgh06dPFyaxsrOzZfTo0WbpFe4EiU+0OIk10XVp+YUJAW2rIhBfBGIiVqrG6QpEiuvARo0aCZYs4cmWxrqGNdmQa32KgCLgEYGYidVjuQlJ5vTDmpAKtFBFQBFQBOKAQMKJdcuWLcaH68knnyynn366PP\/888Kkl1v3adOmSbVq1YLCVwki+R7Q5VZuBPU6RgQ0uSKQMAQSSqw4ZsGHKyS5evVqYUvspEmTZNGiRXkaBAGPGzdO1q9fb2Tu3Ll5vk6wYO3uYD4l1iAUeqIIKAI+QyChxPr9998LflwZjy1atKjw5QHGZz\/99NM8MGzfvl3KlfvDc1WeBI4AJVUHGHqqCCgCvkMgocTK5gE8YlWqVMk0fPfu3cKXB3CQbQJ+\/\/Pjjz8KXrJwScjKAybFWH3gHjKwDq51c8DvwOkhZgQ0gyKQDAQSSqysIChbtqxgrW7dulVuueUWYSPBBRdckKtt+\/fvlxIlSsiAAQPkiy++MNtmR4wYIZ988kmudHa51cJDQwkbNmyQTBGGQTKlLbYd2qb0eD4z6T7ZOZpcpJGii4QSK23CI9YTTzwhl156qZx66qny2GOPGXIlzgqW7dixY6Vp06aGhNl8wCaExYsX2yRiSZWAKy4+T7KysjJGKleunDFtsfdF25Qez2cm3Sc7PwNHpFoSSqx05Z9++mnzxYE33nhD2FQQaofWqlWr5PHHHxf3Z1+wbkMBdGLZP4cKTtswtwWftg1xKO6rNjn0Ksiptqkg6BWuvAklVroZ7733njz00EPCL2M4aEuWLCkQL34H8EOAM+0VK1ZI\/fr1Q2YZM+Tu4LIsa\/6n85FGprP+oXTXNv2xdDAUPn4Jy9T7RLtSKQklVr7e+vnnnwu\/9M4HCVeDrFFlGdasWbOkSpUq5jMvQ4YMESavGGvt37+\/1KxZM4jNiWWLya6Hmsund50jWz54zizJsqa\/Hg8vUVMcFAd9Bg4\/A0HiSNFJQokVi3PlypV5SLBbt25mjSprVfkaAW1nJcDMmTNN2pycHGnevDnBeQSCzROoAYpAHgQ0QBFIHQIJJdbUNUtrVgQUAUUgdQgosaYOe61ZEVAEMhQBJdYMvbGFtFnabEXAFwgosfriNqgSioAikEkIpAWxsh4Wr1h4x6pVq5bg2OWHH35Im\/vgVX+Wmj377LPGCxjewDp27Gi2+vqxoV7b5NSdz\/rwAUo\/37tY2vXhhx9KkyZNzNK\/Vq1ayWeffeZsrm\/OvbYJp0l8LJR3jNU5V111lW+fv0jg4p+kS5cuwpLNSOkSGZcWxMrW1pdeesmsdf3oo49k06ZN5jPciQQmnmV71Z8X9eWXXzbt5CVlVQU\/Iu6NE\/HULb9leW2TLZ8tzaxnZieeDYvXMZ7leG3X5s2bzdbrhx9+WPDcdskllwg7DA8cOBBPdeJSltc2sfRx6dKl8p\/\/\/EeWL18u+PQYM2ZMXHRIRiEYJqyF\/9e\/\/iUs80xGneHqSAtinTNnjrDmle2SpUqVMv5d33\/\/fcF5S7iG+Sncq\/48DOeff77Z3soOtYsuukggJH6B\/dQedPHaJtJCNuysO\/fcc\/NsZybeT+K1XWxiqVevnrD1+ogjjpCbbrpJBg4caLZk+6k96OK1TaRFuF8ckeOOO45DWgh8wBJOfJPw\/qRSad8TKzd548aNUr169SBO3Oxdu3alBbHGon\/nzp2FNb62oWvWrBFe2lR9\/sbq4T7G0ibyfvDBB0L3H38RXPtVYmkXPYqff\/5ZLr74YmHY5uqrrzbPYyAQ8FXzYmkTa8rLly8vfCQUvx6sQb\/yyit91Z5IymB0sW3+zjvvNF+UjpQ20XG+J1YeXkiUba+JBiMR5ceiPw8G7aRLw9DH4MGDpWvXrsKnxxOhW37LjKVNuIPk45NYdH\/605\/yW2XefAkIiaVdDGngJIihADyyNW7c2Iz9M06ZANXyXWQsbZo6daqwI3L+\/PlmKIA5jaFDhwrknG8FCmlG3xMrrgeLFy8uPMjOe4QlFwj4yzpw6mfPY9Uft3uXX3654LzmySefNBaRLcsvR69tYtKEdmDVObcn+6Udbj28tsvm69Chg9SuXVvIx7btb775RnDubuP9cEQ3L+8P4\/gMGdCrwH8yP\/CdOnWSr776SvCj7Ie2pJMOvidWusH4EmD80QLLA0yXBQvPhvn1GIv+jKfefPPNxsXiW2+9JXxtwY\/t8tomxry4bzgwx1dEu3btBKc8WHepnLENh6nXdpGfiR2sQc6tFClSRAIBf\/3Ye20T45LFihWzTQkeIWbiggF64gmBIp5SpTgRM64MSmPN8evJkqS2bdum7NPbscLhVf9XXnnF+Ei49tprfTkJ4my3lzZh9UyePNn4f8A5CB7MLrzwQlmwYIHUrVvXWZzH88Qn89IutMCXxezZs4VxSLr\/r776qjCZ5fWbw3hVAAAIPElEQVTzQpSRLPHSJnqA55xzjrz55ptmwpQ28Y067hP+kpOla6bUkxbEetZZZwmD6Fg83PzTTjtNWDeYLjchkv79+vUThHGstWvXCh9UZDIECw9hNQTjXn5rq5c2+U1nL\/p4bRfpunfvLtddd51Zd7xz506htxEI+Mtipc3oGu794dlDSPe3v\/1NmMBi6IbxVYYHbrvtNt9Z4ejqd0kLYg0EAsJiZdbWMVFwxx13mHEtv4Nr9QsEwus\/cuRIQbAY+IoClp1TsNQrVKhgi\/LNMRCI3ia3slg\/fEECS9Yd55frQMBbuwKBgNBrYt0kqzceeeSRlM9Eh8MwEPDWJrr9N954o5m4ok2sOw7nbD5cXX4I533hm3k8b6nSJy2INVXgaL2+QECVUATSDgEl1rS7ZaqwIqAI+B0BJVa\/3yHVTxFQBNIOASXWtLtlhUNhbaUikM4IKLGm891T3RUBRcCXCCix+vK2qFKKgCKQzggosabz3cuH7nwh165bjJY9lrTRynLGs6geP6Y4ncE5SzAuygl6o1OUZEmNxtUe643ZJZeo3WSUS\/kI50ltoFaWLwSUWPMFm2YqCAI4ZjnxxBMFV4LJXtOK\/wKc3BREf3feyy67TJYtW5aw3WSsx2S3WqNGjdxV67VPEVBi9emNKYhaOALp06ePWE\/wnBOGdTV8+HDjJNxaiyykxnrE6sKxNs6acXgTKu2qVasExyPsDOMlJy9EFUpXFs6zO45ynWkpF49dxHN0W6yUR7nkoZ7WrVvn8swPKfNlBeIoH\/d91I\/O6E4bqJM2UQ7lYeW1adNGaDvhWMzgAS54yqeuUaNGGZ+qVh88POFYhbKoB32pJ5KQF1yxqi1OV1xxhbAVO1Q+twUONuSnnFDpNSx9EFBiTZ975VlT9nvzcuINnt1qOAfBUxbbFdm1hoXFHn7IZeLEiYLrO3Z7cY67QnbduNPibLtv377yj3\/8w3jMf+qpp2T06NGSk5OTRy886uN45Z\/\/\/KesW7dOnGkpF0sVMuNYsmTJXPkpj3LJQznsbsJ1He0hIZbbPffcY3T4v\/\/7P4EQ2XrJjry3335b+AIDdQ4YMEAguB07dpBNtmzZIjg\/xjcsW6JpM24MwYhPxixZsiTomQoiHjhwoFAvWNAO2oM+prAof1577TVh1xI6NWvWzGxZBuso2TQ6gxBQYs2gm+lsCuRC95StsmyZvfXWW53R5hyHIbj1Yy855IR1BwmH6ipDSGwVxIkK3o7q1Kljvve0aNEiU5bzD9beKaecInixCgQCQtqWLVsKxOdMF+qcNKQlD\/VgneLLlXPSs48dV31co\/dPP\/1kXErilpAfj6pVqxqH02zPPHjwoNAm8p1wwgmCrwnCIVs+V4JbPIYicDLCt7hoO2nxS9q0aVPzdQDq4UfgpJNOMg5XiI8mf\/3rX81XIKiLOtFx48aN0bJpfAYhoMSaQTfTNgWLtGfPnsbxMtYZ3VHbZbZpOEKm+CfAKxOWJFYWJECcW7C4+BwJnuXpHiOTJk2Sr7\/+2p1UGDKgXkjdRkJgdNftdagjVulXX30lpLXxkB4kh7VJmDOOayt79+4VLFkmeCCzZ555xhCujUcX6xYPndEF15M2nq9SlChRwlzSVjyoMdxAOyFyPFmRzySI8sf5tQvqpO4oWTQ6wxBQYs2wG0pz6ILiBQzrD0JljJDxRfyjEm8FN350hefOnStYmVi2EJmNdx6xvhjvpDvMsAHy6aefmu88OdNxjtWIxWytRY5cV6xYkWgkpEBCZcuWzeWxni48Dk4gzpCZfg989NFHzRlder6Hxic6LBmbCMefY489VrBO9+zZEwylHgiVANrao0ePoLtDdP\/444\/NMAjx0QR\/wTYNPlvpAWD52rBwR2e+cGk0PD0QUGJNj\/sUk5YQJSSJBUhGrFC8yGM5QRrOLjLxvPikfeGFF4wFSjzhzrRYjV9++aXxpQpRQqxMtEDMpHUKpM4QAWREWrrdXDdv3tyZLM85+jEM8O9\/\/9tM+OATlLFW6kL\/PBlcAdSF+0V89mJNQ8Zcu5IZL1THH3+8PPfcc0IdOBgnvU3HD8g777xjuv6USTv+\/ve\/G2\/6Nk2kI0MJlEnZ48ePl8qVK5uhAXcePEdhBVMHpM5kmzuNXqcnAkqs6XnfImp9\/fXXm7FFuvisDGDC56677jKOwSE9riEKZr2zsrKEMIYCOOfLsL179zbfPiLcpmWM8v7775chQ4aYj+dBqox9MsvuVoaxTz5BjH9SutO33HKL3H777UK4O637GuuaMUq68\/gExZLE+oR03Wmd17i7Y+yUIQjGTikHy5mZdwjOmRZLduChySlm60mPr1LGhG0a2gQG+FpFfyauGGZgHNemiXQkP\/eAsqmDuqgTv7r412X2n\/w33HCDYPXz8T63Du60pFdJHwSUWNPnXnnWFEuI8VJmtOnG4gkeq4kCmBTCemRVAF1zm47lRO3bt5dBgwYJFi8TVc60jG0yiTNz5kzTRSY9M\/aBQF7HzoFAwDhMJg3WJjP9pA0EDqeFxKmfMtHJKVjJkCSrGdCflQOQOmmwwrt168apEWc5pKGdtBcdicMKpZ4GDRoI1iBtIiMWIsMOxJMe\/WgbVjH1BwKBoK9V9Kc84snrRVh3Sh7KRid0Ix\/1gy26cc23paZNmyYfffSRmdi79957BX3BxZ2W9Crpg4ASa\/rcK9U0Tgjs379fWHbGuleGQRg6gNBYGsU3ouJUjRZTiBFQYi3EN7+wNh3yZM1ur169zLfFmLBjvLVLly4xQ4L1iSVNRixNCJowruMplM3XF7CG41mulpUYBP4fAAD\/\/z3Oj3YAAAAGSURBVAMAHwELYBGc\/gMAAAAASUVORK5CYII=","height":206,"width":342}}
%---
%[output:98c8ccfa]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAVYAAADOCAYAAABl7SoZAAAQAElEQVR4AeydCbxN5frHn03dXCSEFMoRQql0M5Sh\/FNEQrduuKVug0ypUKdL3TSYVTJEpUEDjVJuUYlwdEkSijJnrK4pDUpy\/76v3t066+xh7XP2sPY+j4\/nrLXe8Xl\/a63fft7pWUX+p\/8UAUVAEVAE4opAEdF\/ioAioAgoAnFFQIk1rnBqYYqAIlBoEXA0XInVAYaeKgKKgCIQDwSUWOOBopahCCgCioADASVWBxh+Pp01a5ZUq1ZNJkyYkBA1V6xYIWeccYb069cvIeU7C\/3hhx+kc+fOct5558m3337rjEraOTjSXtqdtErjUBH6ojfCudci\/YC5U1d0pw08187w9DsPrbESa2hcCn0ohAfxQYC8lIkEJJl10Q5e6vHjx8uDDz4odevWJSisxKobRJGoH0Duw5AhQ6R06dIybdo0Oe6448Lq7fcIcL\/wwgulb9++wv3wu76x6qfEeggxrDReBiu8HIeChQcZYkF4kG081g7xVpz5ISNeRuIohzy8wJTBOUfKJT6UUDbpEH7Rwz101EldL7zwgrFkqYvySE8+8iOU5wy39aMD55Rh9SUdwvUVV1whmzdvloULF0rXrl0NFsRZoT5n+aHKQ0fSWCGPzW+P\/\/3vfyVUXeiAbjYvZdk8HLkmjraCL+e2rcRTF2FWuCYcmTRpkiGn008\/nUvTNrCwaamX+pFYdKMOsKLQ4cOHB3sXhNuyOXJNmlBCndRPOoRzwsCXsrkf3Jf27dvLN998k6cI0pKHvAg4uRO999575pkh3okZdThx4Jww8qMz6XneKN+W63zebrrppjw9EWc8+Z31XXPNNRQt3A9zkkF\/Cj2xcqOnTp2a65byAPMg2UAe5j59+thL4aWx8Txgzvw89Lfeeqt5WW2GcePGGYLimrKef\/55TvMIZVK2jfj++++lV69eYbvL1HX33Xfb5CYd6clnAymPcu11vI6QUpUqVWTt2rWmyJ9++km2bt0qlSpVkuLFixtSceJCIjeuhIWSH3\/8UcCQ9tl4yuJecc2Ra85pK\/hyboX2Upe95mgtI4hn8eLFQT2JGzhwYPD+cE291E+buHZKNN2caTmPpAvxToHEqJf6bTjnkPuOHTtsUNhjqPzgBF42E+U5nxksd8iPeDcOPKuEEWeFvJTBNVg6nzcImzzEIZR71VVXCfeIa8T5PGJxY31zPyiL+DSRqGoWamLlZk6ZMkWOPvpoeeONN2T9+vVyxx13GNBmzpxpjvwJF09+HopGjRrJ8uXLjXDOw4WQFyGM+Mcff5xLmTdvXi7iJZCX4qmnnuJUSIcul112mbEayWsiQvxBX9K2aNFCeIl46MlHGOWQxdkWrqNJhQoV5JVXXhGIE90pp2TJkrmykaZ+\/fpC+8EBHam7WbNmAiF5wZUCy5cvn6euQCBgSJq6KRcs0QXcsNI4khe9iCcd11Zor\/ue8XJjGZF\/z549gp60Cdy3bdtm2ko9trzPP\/\/cEIIbh0AgvG7ogU7owX3p1q2bRNKFdE6hfoRy0APhHFz5AaNsrsECEqM7HSk\/zzQ4gBc\/CDYt5diywQVMwuFg76\/NS\/3kHTlypHne0c2GUa5Nx3HBggUGQ8KdzyPPOfXxA8wPMfcDHciTKVKoiZWbyU099dRTJSsry9xTSIkHlwfKWgnO+MaNGxsi5mXcuHGjkJ+XAQsO4dwU5PhjX2J+oXnQHVHBU8gIi4+6KYcIHl4eSEiTa7dQFvrYcF4+zlu1asVByEd+yjEBcf5TvXp1035wpG6rD9fg4sQtFK7h1IG0586dK5MnTxase15cXmDS79u3z5CuxQlyvO6664gywgvLvYEw2rVrZ7q8WEkm8tAfdCPu0Kn5T\/4TTjjB\/IBRD9hT3rJly0KOv0bSzRTo+BNNF0dScwqGnNjnBd3QhTAImmMkceeHeGkHOJYoUcJkdeJGu03goT\/URTpwhzjBIdKzfChLsLdi9QU\/hDjE6kPvgWEAjoQXBinUxBqvGwxpQGBOgdTiVb5fy7Gk\/u677xor3EmkBdEZQmJ8j5cR8uYFhxBiKZP05HPeE35g7MvuLItwyMSSAgTAOCKWuDMd5\/nRLZwulOcnYVgLzMHIiUdBdOTHFsvZeR8gcIjcGhMMB2B0FKSeZOSNpY5CTazcTG4q3b4NGzYY3Gx3mm5uuXLlTBgvKMIF3UksHn7tq1ataiZBsG5DvYSk9yrubpHzBWaczks5kBDpeDE4MrbGi8ILwzWCVcwDTXtpN2H5Fax8yHTJkiXGirSWi1dcw9UL1gjdaX6geMmtxWrzcE04ONG1tOG8sNwbG2\/D7dFiZK+5b5AoZGq7rNQbLj96IaQJp5stO5ouNp09Wt3outMuxLbN9kJs2lBHm9\/ef54b7j8\/Us6hgFB5GQ9leAEjoduhIYxYng+rL7ggtnz04V1hSMCGOY\/0HujZ8K7RE3DGpft5oSZWbmanTp3MOJC72+h+kHnxeEghXm468eTnoeAlxNohHuFF5YUlnVfhJYSYeBDRxXbFKBfxUg4WJBYCXV\/04Mg1s6+W7Kyu1EFd0crlRaHtvOTutFZn0vCCUD9pwMUrrqS3QjnUVapUKTPcgv60gzCbhh87yuaacIsT11a4N5wTT34rEA04gAnxCLpyD6mbskhLvaQhLWkQ4ikvkm6ks0IZ\/LBF0sWmtUfuM0Jd6IJwjsXLuU0X7kga0vKM0g70JS3DCXYogOtQQlsxMmxeL88HJEx96Ejdtj5bPs8DOIIF+lgBF9JgpFAnP1JcZ5IUamLlRvLrzAPCuRUsF6wRe83Dc\/\/999tLM8Fl4+lGOvPzII0dO1Z4YYMZPJ64daHeUaNGCQTmpQjG1Fh65EzLNeHow4yujevZs6fwEttr99Fa0O5w97V9ebBcsWBtvLsthLtxJQxx11WnTh1hjSNx4EnXEV2xsLGkmGnmmngwct4bwrg31MW5FV5ewi2BWCuLeGa+bXlcIxa3WHWjfHSmDIQ6w+lCvFO4z9xv2mTDOWcCjftnw8IdScOz56zftjtcHhtOXvuDRRg681zzQ0zPgDC3kCfSM8Vzxxi5Ux\/K5NmwFjL5Kcddtm+u86lI0ogV66hLly6C1SAh\/k2bNs1MNNhftfxYfSGK9RQEOTrHgHgZ3Bl50W0aHgxnvDM\/kwU8UMRTDnlsesKJhyh4iUjjFmdZTCTYh85dFukoizKdZdh01ItwbeM5Jwxh+RF62Dooh\/Iol\/ToRzxpOXJNuFtsvlBpKIv8Vqif\/JRFelu3vSYd4VzbvOgE6RHOOeSNZYSVxMtPGUuXLqVYoetpTg79oS7Ks2LvAXhCIHZI5FBS88NF+TYtR\/K740gTSTewQNCTMmydlMW1FRtO+W5BP9pk03JOGOmoGx2cYYQ7xVk\/Zdi6QuW1GKMfZZCWPAhhzniuCScNaRHeZe4F9wfyveiii4QfP2b6+UEijVsfynSGUy7XmSZJIdb\/\/e9\/wvKbcGMtgLplyxZhPSI3D4n08JBepXAiAEEwZELrean5Iab7ilWEtUh4NMHKZuiCfNHSanx4BOj+Y1HzI8e5HT5gjJv7FD5n5sckhVj5FVu0aJE0bdo0LKLbt28Xxs\/CJtAIReB3BLCa6FL+fmnGY+lyYh3ZsEhH0nXv3l2ci+Mjpde40AhgSbuHHrgv1ioNnSsdQguuY8KJlUkPwMe6CEeczFiy9pDxlho1apixv+nTpwuWbsGbmP8S+NWN1vXKf+masyAI8PLSs0HoekOWsZQHOecnXyx1FIa04A6O3AeE+1IY2h2tjQknVoYA6C40aNAgrC779+8XZi0HDBggX3zxhQwbNkxGjBghn3zySdg8GqEIKAKKgF8RSCixrl69WubPn2+ceAQCgbAYlClTRrBqGSooWrSo2W549tlnm+2S7kyMqalUyzXRp3goHvoM\/PEMsHLEzRseruOaJKHEumnTJsnJyZGGDRsaImCygCEB54J1WrNq1SqzP\/6XX37hMihMSAQvHCd0OTJJaFomtYe2aJvWG98TYOFnycT7dMEFF9CslEpCiZWlFM6HioFtlsi4x2EYy2TbG7OLv\/32m9keyTo3Fm6nFB2tXBFQBBSBfCCQUGKNpA87k1irylo4xmCZuMKJL5NXjLX2799fatasGamIjIl7\/\/33M6YttiHaJouEv4+ZeJ9iQTxRaZNKrFiqWLE0hqUarFW11ywyxoMPFi7DB82bNyeZiiKgCCgCaYdAUok17dBRhRUBRUARyAcCSqz5AE2zKAKKQBojkATVlViTALJWoQgoAoULASXWwnW\/tbWKgCKQBASiEitf0ezYsaNZtI\/zi2iSBJ21CkVAEVAEYkQgucmjEiv79Vm436NHD8nOzo4opEmu+lqbIqAIKAL+QyAqsQYCAfMFS3YztGnTRiIJafzXRNVIEVAEFIHkIhCVWI866ihhTekRRxwRVTM+ZRw1kSZQBBQBRSA5CKSslqjEyjDAE088IXinsl8A+PXXX1OmsFasCCgCioDfEYhKrFih+EZ98803zZBAnz595C9\/+Yvcd999ghcZxmD93kjVTxFQBBSBZCIQlVhRBld+tWvXlkGDBgmfOn7yySeF1QIXX3yxsCX19ddfF7ViQUpFEVAEUo2AH+r3RKxORY888kjB69To0aNlxowZUrp0aRk1apTs3r3bmUzPFQFFQBEotAjETKxYqrj+wyk1FitfzYRYjz322EILojZcEVAEFAEnAp6IFWv0hRdeED5vyxcusVRx6\/fxxx8LHqvq1asnDBc4C9ZzRUARUASShoDPKopKrPhNbd++vYwZM0Y4zpkzRxhTbd26teCg2mftUXUUAUVAEUg5AlGJlc+jsNxqwYIFws6qSpUqpVxpVUARUAQUAT8jEJVYv\/\/+exk4cKDs3LkzajuwbqMm0gSKgCKgCMQFgbyFbNr1sxwsXi5vRJJDohIr+uzZs0eGDh0qjKtGEtKQXkURUAQUgVQgsGDdHtl70TAp22eOTFn8dSpUMHVGJVaGAm6\/\/XbBDwATV5GENKZU\/aMIKAKKQIoRqFKmWMo0iEqsf\/7zn42vgEjOV5xxKWuJVqwIKAKFAYG0aGNUYk2LVqiSioAioAj4CAElVh\/dDFVFEVAECoYAk1cFKyE+uZVY44OjlqIIKAIJRCDdilZiTbc7pvoqAoqA7xGIiVhZ04rbwFq1asl5550ns2fPljvvvNN4uvJ9S1VBRUARKFQInFjWx6sC7J3A7+rDDz9s3AM+99xzcvzxx0v16tWlePHi8tBDD8mBAwdsUj0qAoqAIlBwBNK4BM8W644dO2TlypXCmtaTTjpJihQpIsWKFZNrr73WOLzGUUsa46CqKwKKgCIQNwQ8E2u4GvkWFj5aw8XHGs5wg\/0ETKx5Nb0ioAgUbgQ27drnCwA8E2u5cuWkTp068sADD8j69evlt99+k2+++UYmTJhgPtlSpkyZAjeI4YYpU6YIDl8KXJgWoAgoAmmIQGao7JlYA4GAB5mVGQAAEABJREFUMHGFd6urr75aFi9ebNwIMgTA8ACWa0Eh+fzzz2XRokWCE+2ClqX5FQFFoPAhsHnXz8FGp8XkFdrif\/Wee+4xY60QIGOu+GmNh7X6ww8\/yNixY6Vr166CdUx9KoqAIqAI5AeBVJIq+nq2WPfu3SvDhg0zHq4gV1YI3HvvveYaj1d8YWDr1q2UmS9hCKBKlSrmM9v5KkAzKQKKQLoiEDe9N+0+bLGemEIHLDTGM7GSmJUBb731lkCy9nr69OlC+MKFC6VDhw4ExyyrV6+W+fPnG2s1EAhEzV+tWjWzEmHDhg0ZcdyyZUtGtMN5P7RN6fFsZtJ9qnpaA0m7La3M\/O\/fv1\/4kCBd9sGDB5vzxx57TPCANXz4cPP5lqisGCLBpk2bJCcnRxo2bCiQ5tSpUw3J9uvXL0RqMZNnWVlZkilSuXLljGmLvSfapvR4PjPpPj0\/7d0gXzSuXvDJ9GBh+TjxbLGyDIovtPLiOOthkwDhxLvjnOkinbdo0cKQJasNkMsuu8yQNh8qjJRP4xQBRSB9EYi35sPf2RAssvHJpYPnqTjxTKw4vGbyii4\/y6JQliOTWJxjta5du5ZTFUVAEVAEkooAQwA56\/aYOov8tEOaVE8TYoU4e\/fuLY888ojpsjdr1kzOOeccueuuu4TlVxs3bjS7skzLCvgHSxUrtoDFaHZFQBEoJAgMc1irxb54M+Wt9myxoulpp50mM2fOFL7amp2dLSNGjDBjoxdffLHUqlVL3n33jzEO0qsoAoqAIpALgQRc8G0rhKJZZvWnTQs4TanERKx0\/RlLxQFL\/fr15ZRTTpHNmzdL586dhQ8OlihRIqWN0coVAUWgcCEw7J2N0nPKKtNoSHVsx9rmPNV\/YiLWiRMnCoTaqFEjscJEU9WqVYXx11Q3RutXBBSBwoEAY6qQqnMIoFP941M+tmrR90ys3333ncyZM0cmT54sL730khlXXbNmjfTq1UtOPfVUs+TKFqpHRUARUAT+QCB+ZxAq3f5LH10qllSxVLNbZkl2y6rxq6iAJXkm1l9++cVUxZKq0qVLy86dO2Xfvn1y6aWXyty5cwXiNQn0jyKgCCgCcUYAQh32zkaBUOn6c00VkCrdfz+RKnp5JlZWBeBohSVVECtjrQiFQKqWeLlWUQQUAUWgoAhAnjlr95gx1DMf+I+xUAmjXAg1+5CV+uld5\/im+49eVjwTK+tYO3XqZJZUbdu2TU4++WRp166dkYoVK0rZsmVtmXpUBBQBRSBfCECcdPWxSiFTLFSubWEQaqf6FeXNHvV81fW3+tmjZ2IlA8uq6PYzpoqrwAcffFBGjx4tQ4cOFaxZ0qgoAoqAIuAVAYgUq9R088ctFcgUUnWSKWVBqNZCHdeptnBNuF\/FM7HieAU\/AT\/99JMhUYYG8Jt6xhlnyMsvvyy4\/fNrI1UvRUAR8AcCECmkCXle+juRYpUyEWV3TllNIU\/I9M1D1ild\/mwfTU5ZHcMdoxIrE1SsBnj77bfl1VdfFY54uLLy+uuvm3AIN1wlGq4IKAKFAwHbSggUcZJo2T5zclmkoYi0ycmlD3Xxs0xX35Jpqren2jbFcoxKrFiieLDCo9VXX31lnFHjl9UKflivuuoqKV++fCz1alpFQBHIEAQsgTpJlC49gmVKuJtEaToWqZtI3+x5eOw0HcmUNlmJSqwQ5osvvmgs1b59+5rjvHnzxArbWK+88koJBKL7UbWV6lERUATSCwHIE4EkGQ+FMOnKO61QwogPR6IQKRNPjJHa7n2mEKn7bkYlVj4aiFvAn3\/+Wdq3by8cv\/32W3EK8aRzF67XioAikD4IQJxMJEGOCETpJE9rgTIeSnyQQF1NhECxRN0kStceUiU83S1SV5PzXEYlVjYCXH755cEtrHYrq\/NIPOnylK4BioAi4BsEIE4EUkQgzoGzdoiTPJlIIhwhTTjypFGhCHTXQ80FAsUSLSwkChZuiUqsFSpUMDurcEAdTliCRTp34XqtCCgCyUMA0sxZu0cgRARyRJzEaa1OwkkzfdUP4pU8s1tmCWRJN14JNPJ9jUqszux4t1q6dKnwMUF8BLB+FbJ1ptFzRUARiD8CkCZiiRNiRCBNyNKOdbotTsgzEnGiqdvyDEee2S2ryh\/deHKqhEMgJmLFF2vPnj3lwIEDUq9ePfM5FcZdZ8yYEa58DVcEFIEICECWiCVMiBDCRNykCYFa4iQdAmmSP0IVZjG9HfPMdliddNmX3Fw1T9ddyTMSmt7iPBMrfgGmTJkiDz\/8sAwaNEiuv\/56810qnF4TTry3KjWVIlA4EIDwEEuaztl0S5qQJWIJE0KFMBGvpBnO4oQ43V32bIfVSb7CcSeS30rPxMpGASzVrKysXFpyTTjxuSL0QhHIQAQgSsSSJQQYijBt19xJms7Z9PyQZrbD2nSOc0Kgzskia3EmgDgz8I4mpkmeibV48eJy5JFHyqpVq3JpsmHDhuAW11wReqEIpBECkCXiJEysR8Ral38Zs9HsHHKSJfEFJUyIkHFNBMKEKK2lybklzWyHtZnpy5XS6NEJqapnYuULAdddd52wSYBPsfTv31+6du0qN954o+D1Cu9XIWvQQEUgRQhAlEg4srSEGcq6hDCxRhEv1iVNxEJE7HgmhJntwcqEUEmLQJiUQXkq6YuAZ2Klieedd57ZedW8eXMupXbt2vLGG28IXq9MgP5RBBKIACSJQJQIpIdAgghEiWBReiHLWAnzL5WKmVlxCBAyRLAwkVAWJvHZGWBlJvCWZmzRnol19+7dMmHCBPnxxx\/lhhtukMGDB8ttt90mjLFmLDrasIQjAFEiTqK0Y5aWLEMRZbjJnljJMpJ1STfcSZiPX1bRrOOEMCFXBAsTSThQWkFaIeCZWIsWLWqWV2GdtmjRQlgNANmmVWtV2YQjAEkiS7b+HFyo7ibKaFalHbPEGvVKlDSMLjQSiiwhQyxLN1na8Uvi3dYlZVGuiiIQKwKeibVUqVIyfPhwWbFihXFszZGhgSuuuEJmz54t6isgVujTIz0kiUSyKEMRZdepX5tPamB1uokyXmQJUSJeyLJT\/YrmEx5Klh6fO01WIAQ8E6uthZUB9evXl0ceeUReeeUVs1Lg3nvvNR8XtGn06E8EIEgrkYjSdr3DjVMmiiixGiFKhC44Amkialn685lSrUIjEBOxYpWy3GrAgAHSoEEDwQ\/raaedJvhkxb1g6Co0NFEIeCHJUNYkxOkco3QTJeV61RkLEHF3vwe2KGfGIyFJBHKMRpTWqtQxS6\/oazq\/IuCZWHEN2KpVK+nQoYPwRVY+07Jw4UJh2VXlypXD+mP97LPPhHzVqlWTJk2aCF8jCAXGtGnThDRWGGbANWGotJkaBqEhkazJUEQZiSRj6XaDKySJuIkSaxKBJJFoRNm2dkkzgw5JIpRJ+Sp+RkB1ixcCnomVb1yxEoCx1ZEjRxpfAUxoRVKEba7k6dGjh6xevVqys7OFLw9s3749T7YtW7bIuHHjzAQZjl0yxWMWRIk4yZJxRwSSRNpO2iKJ7HZDapGI0pJkLBYlZea5iRqgCCgCBgHPxMoGAcZWGWM1OT382bx5sxw8eNBYqnzFtWHDhmZMdseOHXlyQ7blypXLE+73AEgTYQYbsYRJdzscWZIOwZpEtu09ELKZbvLiGolEkqGsSYgz1Bil7XpTZkgFNFARUATyhYBnYs1P6XXq1BE+61K2bFnB5eCiRYvM9teKFSvmKo61sdu2bZO7775batSoYZxqT58+3eTJlTDFF5ZAneQJgSKEIZYwSRtNXQgNcS48z\/59pw7d7rEdawc\/qqbWZDQ0NT6fCGi2BCCQUGK1+jJ8cOaZZ5rtsIydli5d2kaZ4\/79+6VEiRLCpNgXX3xhhgtGjBghn3zyiYlP1R\/IEaKEMOmyWwIlDEuT+FC6QZaItSwtWWJNIliQu373tM65c+F59u87daw1qeOToRDWMEXA3wh4Jla8VzFZxdHZJK6XLFkiv\/76qzM413ndunVl2bJlgj9XK84EZcqUMV9\/bdq0qTBu26xZMzn77LNl8eLFzmTBcya4cP6SKHk1Z5Vc88Ri43ADUrVEGlTg0MkJpY4QhEkaZsARCBL\/lq\/\/vaIgj7QuLf0aFZO\/1fyfNCq3TyoV3W3kt++2i1N3xped15lwrm3akOse+\/WeZtJ9gheQQ69nyv9HJVaWWLEigAfjwQcfNA8Ls\/VWvvzyS7O9NdQurJycHOnXr59ZRUBLaTTLs7Zu3cplUFjCxSoDVhsEAw+dhHPswuQWW2njKUWPOV4W7viz3PL2HmFxO5+sOKRC8D8WKFYkXXSszs8GNhVk0o31pXfrukYub1LbbPGNVS9WVcSax+\/ptU1Z+XoWknVfbT2ZdJ\/gBST40qbwpEi0uvlIIB8LvOSSSwTLlGOjRo3MOChHdl5hkWJ1ustibSuW6vLly83OLJZerVy5UpgEc6ZlYgxnLljEEPm8efPMDi93OmeeeJ7Tpb\/00aVmpxBdfFs2ZJp9aMwTIv30rnPMukzItUn13EMZNr0eFQFFQBEAgajEykcCWfrExBOkypFfBStr1qyRgQMHmtl+CnRKzZo1pXfv3nLzzTebSSmct3Tv3l3OOussweJlvHXWrFlSpUoVM3E1ZMgQk46xVtbHkt9ZXrzPIVT2sTN2yjnlQ6aQpyXT7JZVRYkUZFQUAUXAKwJRiRULcteuXcKEE7P2zO5Dik5hqIB07koDgYC0bdtWsEQhYo5cBwIBsYSNQxfyYf0y\/ko6hhCsa0LiEiEQKVYqu45s+ZZQ6e4rmVpU9JgxCGhDkoZAVGJlKKBXr17CWCpDAhCgWwgnnaTJP0uqHFEZKxULFULlnDAVRUARUATyi0BUYsWynDx5sjDpxJAAFqVbCCddfpVIZj7I1Nn1Z0kUpKoWajLvgtalCGQ2AlGJNZOaD6nS\/bdtymZiqmc9USvVIqLHzENAW5QKBDwTK2OqTDaxZMothBOfigZ4rRNS7TVllXAkD+Op2YcmpjhXUQQUAUUgngh4Jla6+nT57TDAunXr5MMPP5SbbrrJuA9kaVU8FYt3Wc5F\/nT\/GU+Ndx1aniKgCCgCIOCZWEnslEAgIOz579atm5nYCrVBwJk+leeQqp39p9s\/tlPtVKqjdSsCiUZAy08xAvkmVqs3+\/z37t1rNgDYMD8d6frnItWOtXVM1U83SHVRBDIQAc\/EylrVjh07Cvv4nXLRRReZNamhdl75AS+sVcgVXRqfXFoX+wOEiiKgCCQUAc\/EyrZTxlNxVu2UZ599VvjmFf5WE6ppPgqHUJ3Wqo6r5gNEzZI2CKii\/kHAM7HyBYHzzz9fzjjjDPO9qzZt2gg+AipXrmw8UvmnSX9oYkmVkOyWWRxUFAFFQBFIOAKeiRVNJk6cKBdeeKHgM5XrDz74QFhqNWPGDC59JVirDAOgFBNWLK\/iXEURUAQUgUQj4JlY8ReAwxR2YeE3FSqnGIkAABAASURBVMW6dOkiEyZMkClTpgjftyLML6LWql\/uhOqRUAS0cF8i4JlYDxw4YLr8lSpVytWQ6tWrC3E4vM4VkcILtVZTCL5WrQgoAuKZWI855hhhAuu1114LLq3C0xUeq\/jAYPHixX0D54J1e4K6dKp\/fPBcTxQBRUARSAYCnon1qKOOMr5VX331VeH7VSy5wq\/qoEGDpE+fPoZ0k6GwlzqmfHT489o6tuoFLU2Tfgioxn5HwDOx0hA8XOEz9fnnnxeWXI0fP17w9s9KAeL9IAwD2K8AnFimmG4G8MNNUR0UgUKGgGdi3bZtm7BBgI0CECnLrfDLiiXrJ8zsSgB0ukOXWAGDiiKgCCQZAc\/EihOW2rVry8cffyyMrSZZT8\/VLVi726RlGEB9rBoo9E9mIKCtSCMEPBMrTlb4GGDfvn2lYcOGuba2Wks21e12DgOwfTXV+mj9ioAiUDgR8EysrAjo0aOHjBo1Su655x4zxso4K8JWV+JTDaFzGEBXA6T6bmj9ikDhRcAzsbKllQ\/8MbbqFsKJTzWMOgyQ6jug9ccbAS0vPRGISqxMVvXu3VtWrVplJq9YZuUWPwwFOIcBWA2QnrdDtVYEFIFMQCAqsdLF79y5s3FqTZefrr9bCCddKgGBWG39nRropgCLhR4VAUUg+QhEJVa6+Cyrwt9qKO9W5557rvhhKGD4OxuC6FUpUyx4rieKQNohoAqnPQJRidXZQj97t9q0+2ejqi6zMjDoH0VAEUghAp6J1c\/erRgGQMBRl1mBgooioAikEgHPxIoHq6JFi4ofvVup05VUPkJad3wQ0FIyCQHPxJpo71ZsPmjVqpVUq1ZNmjRpInPmzPGMs3W6QgaGAjiqKAKKgCKQKgQ8Eys+AVh2lQjvVjjJHjx4sLABYfXq1WbzwbBhw2T79sNeqryCA6kiXtNrOkVAEVAEEoGAZ2Kl8kR5t9q8ebMcPHjQWKp8lJAts\/h43bFjB9VGFMZWnd6sIibWSEXAXwioNhmKQEzECgYQXry9W9WpU0defPFFKVu2rODgZdGiRQLBVqxYkSojCsRqEzSuXsae6lERUAQUgZQhEDOxJlLTFStWyJlnnik4euEjhaVLl45anXPiSlcERIVLEygCikASEPAVsfI57WXLlgnOtK2EwoAJrg0bNggy67PD47AnlDpCKhXdbcIITyfZsmVLWuodCWNt0+HnMxRGfgrLpPsELyChOCPZYb4g1pycHOnXr5\/88ssvpv2Aw3ju1q1bzbX7z\/r16yUrK8vIkq2HNwZUK1\/SXNvwdDpWrlw5bXUPh7O26fDzGQ4fv4Rn0n2CFxA3X6TiOiZi3bt3r7z00kvSv3\/\/XMIMPnH5bUD58uUFS3X58uXmQ4UsvVq5cqXUr18\/YpE6vhoRHo1UBBSBFCHgmVixJu+44w55\/PHHzQx+PPWtWbOm+VDhzTffLDVq1JAbbrhBunfvLnysMFI9TmLVZVaRkNK4lCOgChQqBDwT63fffSd8ReDpp5+WoUOHCutOreDtqlSpUvkGLhAISNu2bWXhwoWCKc+R60AgELFM58SVOl6JCJVGKgKKQBIR8EyseLk69thjk6ha9KqsY2tS6vetQEFFEVAE\/ICAZ2ItXry4cQ84YsQIWbNmjXz77bdBwRn2b7\/9lrL26DBAyqDXiiMioJGFFQHPxLpz504ZO3asvPXWW9KyZUvBR6uVyy+\/XIhPJoiMr+qOq2QirnUpAoqAVwQ8E2uFChVk7ty5ZgyUcVCnEE6810rjkQ5iteXojiuLhB4VAUXADwh4JlaUZbspu6OYvGKJFUMCfAuLcOJTJbrjKlXIa70hENAgRUBiIlZ2Q11\/\/fWybt06Y73i\/BqPV5MnT046lM4VAUmvXCtUBBQBRSACAp6JFdd+U6ZMkTFjxsigQYOEffzsHsF6ZdyV5VgR6ol7lF0RwMSVrgiIO7xaoCKgCBQAAc\/Eum\/fPuErApCps74qVaqYSzYQmBP9owgUQgS0yYqAEwHPxHr00UdLyZIlzSJ+59KqtWvXCkuxEGfBiTxn4kpXBCQSYS1bEVAECoKAZ2Jlg0DPnj3Njqt27doJk1hXXnml2Yp69dVXG9ItiCL5zasrAvKLnOZTBBSBRCHgmVhRAAfX7777rtx5553SoUMHs6f\/7bffFnynEp8swWK1dTHGas\/1qAgkFQGtTBEIg0BMxEoZ+ARo06aN4DClY8eOkuz1q+jgXBGgPgJAREURUAT8hEBMxLpt2zaBTPmUCt+lqlWrliFYtrQms1Gbdu0LVqcWaxAKPVEEFAGfIOCZWJn1v+++++SEE04wE1isZZ0\/f77wDawHHnjArBhIVps27zrs3BpSRZJVr9ajCIgoBopAdAQ8EyvrVPfs2WM8\/eOYOhAICB\/7u\/XWW+Xrr7+WXbt2Ra8tTik27T5MrHEqTotRBBQBRSCuCHgmVlYFsCkAy9WtAe4EiXeHJ+raTl6dWKZYoqrQchUBRUARyDcCnomVdaqdOnWSu+++W1gJwHgrzlf4qkCzZs2EDQS4Esy3Jh4zHixeLphSl1oFodCTxCKgpSsCMSHgmVjp6o8fP142bdokbGPt2LGjIVkI9tFHHxVcByIx1Z6PxAeL+8vZdj6aoFkUAUUgwxHwTKyMq7744osyb968iJJovHJZrCeXTnR1Wr4ioAgoAjEj4JlYKfnXX3+VZ599Vlq0aCEst+K7VDhgcW5xJV0ixUmsiaxHy1YEFAFFIL8IxESszzzzjEydOlX69OkjEydONN3\/kSNHylNPPZXf+mPO5xwKUK9WMcOnGRQBRSAJCHgmVsZYZ8+eLTi4bt26tdStW1euueYamTBhghBOfBL0FWux6vrVZKCtdSgCikB+ECjiNRMuAwOBgJQpUyZXFq4DgUDSNghYi1WXWuW6DXoRLwS0HEUgDgh4JtZjjjnGkOqMGTPEjqnySZaFCxcaz1a4FYyDPlGLsBZr1ISaQBFQBBSBFCHgmViPOuooYZcVXxE4++yzhbWrTGDxNQHcCSZjg8Cm37eygpWuYQUFFUVAEfAjAp6JFeVr1Kgh06dPFyaxsrOzZfTo0WbpFe4EiU+0OIk10XVp+YUJAW2rIhBfBGIiVqrG6QpEiuvARo0aCZYs4cmWxrqGNdmQa32KgCLgEYGYidVjuQlJ5vTDmpAKtFBFQBFQBOKAQMKJdcuWLcaH68knnyynn366PP\/888Kkl1v3adOmSbVq1YLCVwki+R7Q5VZuBPU6RgQ0uSKQMAQSSqw4ZsGHKyS5evVqYUvspEmTZNGiRXkaBAGPGzdO1q9fb2Tu3Ll5vk6wYO3uYD4l1iAUeqIIKAI+QyChxPr9998LflwZjy1atKjw5QHGZz\/99NM8MGzfvl3KlfvDc1WeBI4AJVUHGHqqCCgCvkMgocTK5gE8YlWqVMk0fPfu3cKXB3CQbQJ+\/\/Pjjz8KXrJwScjKAybFWH3gHjKwDq51c8DvwOkhZgQ0gyKQDAQSSqysIChbtqxgrW7dulVuueUWYSPBBRdckKtt+\/fvlxIlSsiAAQPkiy++MNtmR4wYIZ988kmudHa51cJDQwkbNmyQTBGGQTKlLbYd2qb0eD4z6T7ZOZpcpJGii4QSK23CI9YTTzwhl156qZx66qny2GOPGXIlzgqW7dixY6Vp06aGhNl8wCaExYsX2yRiSZWAKy4+T7KysjJGKleunDFtsfdF25Qez2cm3Sc7PwNHpFoSSqx05Z9++mnzxYE33nhD2FQQaofWqlWr5PHHHxf3Z1+wbkMBdGLZP4cKTtswtwWftg1xKO6rNjn0Ksiptqkg6BWuvAklVroZ7733njz00EPCL2M4aEuWLCkQL34H8EOAM+0VK1ZI\/fr1Q2YZM+Tu4LIsa\/6n85FGprP+oXTXNv2xdDAUPn4Jy9T7RLtSKQklVr7e+vnnnwu\/9M4HCVeDrFFlGdasWbOkSpUq5jMvQ4YMESavGGvt37+\/1KxZM4jNiWWLya6Hmsund50jWz54zizJsqa\/Hg8vUVMcFAd9Bg4\/A0HiSNFJQokVi3PlypV5SLBbt25mjSprVfkaAW1nJcDMmTNN2pycHGnevDnBeQSCzROoAYpAHgQ0QBFIHQIJJdbUNUtrVgQUAUUgdQgosaYOe61ZEVAEMhQBJdYMvbGFtFnabEXAFwgosfriNqgSioAikEkIpAWxsh4Wr1h4x6pVq5bg2OWHH35Im\/vgVX+Wmj377LPGCxjewDp27Gi2+vqxoV7b5NSdz\/rwAUo\/37tY2vXhhx9KkyZNzNK\/Vq1ayWeffeZsrm\/OvbYJp0l8LJR3jNU5V111lW+fv0jg4p+kS5cuwpLNSOkSGZcWxMrW1pdeesmsdf3oo49k06ZN5jPciQQmnmV71Z8X9eWXXzbt5CVlVQU\/Iu6NE\/HULb9leW2TLZ8tzaxnZieeDYvXMZ7leG3X5s2bzdbrhx9+WPDcdskllwg7DA8cOBBPdeJSltc2sfRx6dKl8p\/\/\/EeWL18u+PQYM2ZMXHRIRiEYJqyF\/9e\/\/iUs80xGneHqSAtinTNnjrDmle2SpUqVMv5d33\/\/fcF5S7iG+Sncq\/48DOeff77Z3soOtYsuukggJH6B\/dQedPHaJtJCNuysO\/fcc\/NsZybeT+K1XWxiqVevnrD1+ogjjpCbbrpJBg4caLZk+6k96OK1TaRFuF8ckeOOO45DWgh8wBJOfJPw\/qRSad8TKzd548aNUr169SBO3Oxdu3alBbHGon\/nzp2FNb62oWvWrBFe2lR9\/sbq4T7G0ibyfvDBB0L3H38RXPtVYmkXPYqff\/5ZLr74YmHY5uqrrzbPYyAQ8FXzYmkTa8rLly8vfCQUvx6sQb\/yyit91Z5IymB0sW3+zjvvNF+UjpQ20XG+J1YeXkiUba+JBiMR5ceiPw8G7aRLw9DH4MGDpWvXrsKnxxOhW37LjKVNuIPk45NYdH\/605\/yW2XefAkIiaVdDGngJIihADyyNW7c2Iz9M06ZANXyXWQsbZo6daqwI3L+\/PlmKIA5jaFDhwrknG8FCmlG3xMrrgeLFy8uPMjOe4QlFwj4yzpw6mfPY9Uft3uXX3654LzmySefNBaRLcsvR69tYtKEdmDVObcn+6Udbj28tsvm69Chg9SuXVvIx7btb775RnDubuP9cEQ3L+8P4\/gMGdCrwH8yP\/CdOnWSr776SvCj7Ie2pJMOvidWusH4EmD80QLLA0yXBQvPhvn1GIv+jKfefPPNxsXiW2+9JXxtwY\/t8tomxry4bzgwx1dEu3btBKc8WHepnLENh6nXdpGfiR2sQc6tFClSRAIBf\/3Ye20T45LFihWzTQkeIWbiggF64gmBIp5SpTgRM64MSmPN8evJkqS2bdum7NPbscLhVf9XXnnF+Ei49tprfTkJ4my3lzZh9UyePNn4f8A5CB7MLrzwQlmwYIHUrVvXWZzH88Qn89IutMCXxezZs4VxSLr\/r776qjCZ5fWbw3hVAAAIPElEQVTzQpSRLPHSJnqA55xzjrz55ptmwpQ28Y067hP+kpOla6bUkxbEetZZZwmD6Fg83PzTTjtNWDeYLjchkv79+vUThHGstWvXCh9UZDIECw9hNQTjXn5rq5c2+U1nL\/p4bRfpunfvLtddd51Zd7xz506htxEI+Mtipc3oGu794dlDSPe3v\/1NmMBi6IbxVYYHbrvtNt9Z4ejqd0kLYg0EAsJiZdbWMVFwxx13mHEtv4Nr9QsEwus\/cuRIQbAY+IoClp1TsNQrVKhgi\/LNMRCI3ia3slg\/fEECS9Yd55frQMBbuwKBgNBrYt0kqzceeeSRlM9Eh8MwEPDWJrr9N954o5m4ok2sOw7nbD5cXX4I533hm3k8b6nSJy2INVXgaL2+QECVUATSDgEl1rS7ZaqwIqAI+B0BJVa\/3yHVTxFQBNIOASXWtLtlhUNhbaUikM4IKLGm891T3RUBRcCXCCix+vK2qFKKgCKQzggosabz3cuH7nwh165bjJY9lrTRynLGs6geP6Y4ncE5SzAuygl6o1OUZEmNxtUe643ZJZeo3WSUS\/kI50ltoFaWLwSUWPMFm2YqCAI4ZjnxxBMFV4LJXtOK\/wKc3BREf3feyy67TJYtW5aw3WSsx2S3WqNGjdxV67VPEVBi9emNKYhaOALp06ePWE\/wnBOGdTV8+HDjJNxaiyykxnrE6sKxNs6acXgTKu2qVasExyPsDOMlJy9EFUpXFs6zO45ynWkpF49dxHN0W6yUR7nkoZ7WrVvn8swPKfNlBeIoH\/d91I\/O6E4bqJM2UQ7lYeW1adNGaDvhWMzgAS54yqeuUaNGGZ+qVh88POFYhbKoB32pJ5KQF1yxqi1OV1xxhbAVO1Q+twUONuSnnFDpNSx9EFBiTZ975VlT9nvzcuINnt1qOAfBUxbbFdm1hoXFHn7IZeLEiYLrO3Z7cY67QnbduNPibLtv377yj3\/8w3jMf+qpp2T06NGSk5OTRy886uN45Z\/\/\/KesW7dOnGkpF0sVMuNYsmTJXPkpj3LJQznsbsJ1He0hIZbbPffcY3T4v\/\/7P4EQ2XrJjry3335b+AIDdQ4YMEAguB07dpBNtmzZIjg\/xjcsW6JpM24MwYhPxixZsiTomQoiHjhwoFAvWNAO2oM+prAof1577TVh1xI6NWvWzGxZBuso2TQ6gxBQYs2gm+lsCuRC95StsmyZvfXWW53R5hyHIbj1Yy855IR1BwmH6ipDSGwVxIkK3o7q1Kljvve0aNEiU5bzD9beKaecInixCgQCQtqWLVsKxOdMF+qcNKQlD\/VgneLLlXPSs48dV31co\/dPP\/1kXErilpAfj6pVqxqH02zPPHjwoNAm8p1wwgmCrwnCIVs+V4JbPIYicDLCt7hoO2nxS9q0aVPzdQDq4UfgpJNOMg5XiI8mf\/3rX81XIKiLOtFx48aN0bJpfAYhoMSaQTfTNgWLtGfPnsbxMtYZ3VHbZbZpOEKm+CfAKxOWJFYWJECcW7C4+BwJnuXpHiOTJk2Sr7\/+2p1UGDKgXkjdRkJgdNftdagjVulXX30lpLXxkB4kh7VJmDOOayt79+4VLFkmeCCzZ555xhCujUcX6xYPndEF15M2nq9SlChRwlzSVjyoMdxAOyFyPFmRzySI8sf5tQvqpO4oWTQ6wxBQYs2wG0pz6ILiBQzrD0JljJDxRfyjEm8FN350hefOnStYmVi2EJmNdx6xvhjvpDvMsAHy6aefmu88OdNxjtWIxWytRY5cV6xYkWgkpEBCZcuWzeWxni48Dk4gzpCZfg989NFHzRlder6Hxic6LBmbCMefY489VrBO9+zZEwylHgiVANrao0ePoLtDdP\/444\/NMAjx0QR\/wTYNPlvpAWD52rBwR2e+cGk0PD0QUGJNj\/sUk5YQJSSJBUhGrFC8yGM5QRrOLjLxvPikfeGFF4wFSjzhzrRYjV9++aXxpQpRQqxMtEDMpHUKpM4QAWREWrrdXDdv3tyZLM85+jEM8O9\/\/9tM+OATlLFW6kL\/PBlcAdSF+0V89mJNQ8Zcu5IZL1THH3+8PPfcc0IdOBgnvU3HD8g777xjuv6USTv+\/ve\/G2\/6Nk2kI0MJlEnZ48ePl8qVK5uhAXcePEdhBVMHpM5kmzuNXqcnAkqs6XnfImp9\/fXXm7FFuvisDGDC56677jKOwSE9riEKZr2zsrKEMIYCOOfLsL179zbfPiLcpmWM8v7775chQ4aYj+dBqox9MsvuVoaxTz5BjH9SutO33HKL3H777UK4O637GuuaMUq68\/gExZLE+oR03Wmd17i7Y+yUIQjGTikHy5mZdwjOmRZLduChySlm60mPr1LGhG0a2gQG+FpFfyauGGZgHNemiXQkP\/eAsqmDuqgTv7r412X2n\/w33HCDYPXz8T63Du60pFdJHwSUWNPnXnnWFEuI8VJmtOnG4gkeq4kCmBTCemRVAF1zm47lRO3bt5dBgwYJFi8TVc60jG0yiTNz5kzTRSY9M\/aBQF7HzoFAwDhMJg3WJjP9pA0EDqeFxKmfMtHJKVjJkCSrGdCflQOQOmmwwrt168apEWc5pKGdtBcdicMKpZ4GDRoI1iBtIiMWIsMOxJMe\/WgbVjH1BwKBoK9V9Kc84snrRVh3Sh7KRid0Ix\/1gy26cc23paZNmyYfffSRmdi79957BX3BxZ2W9Crpg4ASa\/rcK9U0Tgjs379fWHbGuleGQRg6gNBYGsU3ouJUjRZTiBFQYi3EN7+wNh3yZM1ur169zLfFmLBjvLVLly4xQ4L1iSVNRixNCJowruMplM3XF7CG41mulpUYBP4fAAD\/\/z3Oj3YAAAAGSURBVAMAHwELYBGc\/gMAAAAASUVORK5CYII=","height":206,"width":342}}
%---
