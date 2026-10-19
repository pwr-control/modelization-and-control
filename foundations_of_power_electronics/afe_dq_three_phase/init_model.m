%[text] # afe\_dq\_three\_phase: model initialization (param structure by subsystem)
% Rev. 01, 2026-09-30. Replaces the flat-workspace init_model.m (kept as init_model_legacy.m).
% Model afe_dqctrl.slx: one three-phase AFE (IGBT bridge with the simple two-thermal-port loss model, a
% local masked subsystem) on the 400 V grid emulator with LCL filter, dq current control and FHT/dq PLL
% variants. Same convention as afe_inv_psm/init_model.m: everything the model needs is in the struct
% "param", one group per subsystem:
%   param.sim       simulation settings (stop time, logging base, thermal model flags, template settings)
%   param.time      timing settings shared by all stages, and the PWM data of the stages not in this model
%   param.glb_time  timing_setup object (built from param.time, param.afe.pwm, param.inv.pwm)
%   param.grid      grid emulator inputs;              param.grid.emu     grid_three_phase_emulator object
%   param.frt       FRT test inputs;                   param.frt.data     frt_settings + grid_fault_generator data
%   param.afe       active front end: .pwm .hw_in (hardware numbers) .hw (hw_afe_three_phase_setup object)
%                   .ctrl .dq_vector_pi .pll .impl .ref .behav .igbt .mosfet .ideal_switch
%   param.inv       no inverter stage in this model: .pwm (second clock domain of the timing object, the AFE
%                   current controller runs at ts_inv) and .behav.time_gain_module_2 (clock time_inv1)
%   param.meas      ADC end scale and quantization
%   param.ctrl      control settings common to all modules (time alignment, filters, PI clip release)
%   param.thermal   heatsink inputs;                   param.thermal.heatsink  liquid_cooled_plate_2kw_setup object
%   param.dclink    DC-link stray inductance (Bode check only, not read by the model)
%   param.dev       names of the semiconductor datasets
%
% Naming rule: the leaf name of the old workspace variable is kept (kp_align -> param.ctrl.kp_align),
% except that inside param.afe the afe_ tag becomes redundant and is dropped (fpwm_afe -> param.afe.pwm.fpwm,
% afe_pwm_phase_shift_mod1 -> param.afe.behav.pwm_phase_shift_mod1). Library objects keep their name as a
% field: hwdata.afe -> param.afe.hw, afe_ctrl -> param.afe.ctrl, frt_data -> param.frt.data,
% grid_emu -> param.grid.emu, igbt.afe -> param.afe.igbt, filters -> param.ctrl.filters,
% heatsink -> param.thermal.heatsink. The AFE current controller reads its gains from psm_ctrl.kp_i / ki_i
% in the legacy init: they are param.afe.dq_vector_pi.kp_i / ki_i here.
%
% Library dependence: the library provides classes and algorithms (timing_setup, hw_afe_three_phase_setup,
% ctrl_afe_setup, grid_three_phase_emulator, device_*_setup, liquid_cooled_plate_2kw_setup...), the model's
% numbers stay in this file (param.afe.hw_in); three_phase_afe_hwdata is no longer called. The device
% datasets (device_igbt_setup...) stay in the library: they describe components, not this model. A function
% that must be frozen or changed for this model only goes in ./private with the same name: MATLAB resolves
% private functions before the path, for the files of this folder only, and the list of such overrides is
% printed at every run (see below).
%
% The last code section ("Transition: legacy workspace names") exports the old names still read by
% afe_dqctrl.slx. migrate_afe_dqctrl.m rewrites the model to param.*; once done, set
% export_legacy_names = false and delete that section.

clear;
addpath(genpath('C:\github\library'));
[model, options] = init_environment('afe_dqctrl_param');   % 'afe_dqctrl_param' once migrated
s = tf('s');
param = struct();

% functions overridden for this model: ./private takes precedence over the library
overrides = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
if ~isempty(overrides)
    fprintf('%s init: local overrides in private/: %s\n', model, strjoin({overrides.name}, ', '));
end
clear overrides
%[text] ## Simulation
param.sim.simlength = 1;                    % [s] model StopTime
param.sim.tc = 0.625e-6;                    % [s] logging base and model MaxStep (glb_time.tc); FixedStep is not used (ode23tb)
                                            %     same value as the legacy init: ts_afe/200 with fpwm = 4 kHz, double update
param.sim.load_step_time = 1.25;            % [s]
param.sim.transmission_delay = 125e-6*2;    % [s]
param.sim.sst_num_of_modules = 2;
param.sim.time_aux_power_supply_fault = 1e3;    % [s] never, in this run

% one/two modules
param.sim.number_of_modules = 1;
param.sim.enable_two_modules = param.sim.number_of_modules;

% power stage model: afe_dqctrl.slx has one bridge only (IGBT with the simple two-thermal-port loss
% model), no variant to enable: the flags set the Simscape nonlinear iterations, as in the other models
param.sim.use_mosfet_thermal_model = 0;
param.sim.use_thermal_model = 1;

param.sim.nonlinear_iterations = 3;         % Simscape solver (Solver Configuration of the grid emulator)
if (param.sim.use_mosfet_thermal_model || param.sim.use_thermal_model)
    param.sim.nonlinear_iterations = 5;
end

% white noise power used by the module behaviour settings (clock and PWM phase noise)
param.sim.wnp = 0;

% stages of the shared init template not present in afe_dqctrl.slx (inverter, PSM and IM drives, DAB,
% CLLC, PSFBC, ISOP rail, single-phase inverter, UPQC, battery, load transformer): see add_template_extras
param.sim.build_template_extras = false;
%[text] ## Timing
param.time.fpwm = 4e3;                              % [Hz] base PWM frequency
param.time.t_measure = 0.4;                         % [s] logging window (legacy init: 0.4 s, not simlength)
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
%[text] ### AFE PWM and second clock domain
param.afe.pwr_nom = 250e3;                          % [W]
param.afe.pwm.fpwm = param.time.fpwm;               % [Hz]
param.afe.pwm.trgo = 1;                             % double update
param.afe.pwm.dead_time = 3e-6;                     % [s]

% no inverter in this model: the timing object still needs the second domain (ts_inv, read by the
% AFE current controller gains and by the clock time_inv1)
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
param.grid.application_voltage = 400;               % [V] 400, 480 or 690
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
param.frt.enable_frt_2 = 0;                 % faults generated from xi_eta_pos and xi_eta_neg
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
param.afe.hw.displayInfo(); %[output:3cacc576]
clear h
%[text] ### Control (with sequences)
param.afe.ctrl = ctrl_afe_setup(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom);
param.afe.ctrl.kp_udc_ctrl = 4;
param.afe.ctrl.ki_udc_ctrl = 45;

% resonant PI gains for weak grids
% param.afe.ctrl.res_pi.kp_rpi = 0.5;
% param.afe.ctrl.res_pi.ki_rpi = 18;

% resonant PI gains for LVRT
param.afe.ctrl.res_pi.kp_rpi = 0.6;
param.afe.ctrl.res_pi.ki_rpi = 35;
%[text] ### dq current controller
% gains of the dq vector PI of the AFE current loop (blocks .../current_ctrl/dq_vector_pi, sampled at
% ts_inv); the legacy init reads them from psm_ctrl.kp_i / ki_i of the machine control
param.afe.dq_vector_pi.kp_i = 0.25;
param.afe.dq_vector_pi.ki_i = 35;
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
%[text] ## Second clock (time\_inv1)
% the "global timing" subsystem of the model has the clock of a second module (time_inv1), read with
% the same name as in afe_inv_psm
param.inv.behav.time_gain_module_2 = 1.001;
%[text] ## Power semiconductors
param.dev.used_device_igbt = 'mitsubishi_CM1200DW_34T';        % alternative: 'infineon_FF1200R17IP5'
param.dev.used_device_mosfet = 'danfoss_SKM1700MB20R4S2I4';
param.dev.used_device_ideal_switch = 'silicon_high_power_ideal_switch';

% IGBT
param.afe.igbt = device_igbt_setup(param.dev.used_device_igbt, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);

% MOSFET
param.afe.mosfet = device_mosfet_setup(param.dev.used_device_mosfet, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);

% ideal switch
param.afe.ideal_switch = device_ideal_switch_setting(param.dev.used_device_ideal_switch, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
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
% Not needed by afe_dqctrl.slx: its bridge is a local masked subsystem that takes Tambient from its mask
% (migrated to param.thermal.Tambient). Kept, commented, as a reminder for models that use the thermal
% bridges of the library still reading Tambient / DThs_init from the base workspace
% (tools/fix_library_thermal_source_refs.m; three_phase_diode_rectifier_with_thermal_model and
% full_bridge_three_level_ttype_igbt_based_model were still to be done on 2026-09-30).
% Tambient = param.thermal.Tambient;
% DThs_init = param.thermal.DThs_init;
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
figure; bode(param.dclink.sys_dclink, param.dclink.Z_Lstray_dclink, options); grid on %[output:36f93f76]
%[text] ## Template extras (not used by afe\_dqctrl.slx)
if param.sim.build_template_extras
    param = add_template_extras(param, options);
end
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
% afe_dqctrl.slx has a single power stage (three_phase_inverter_with_igbt_simple_2th): nothing to select.
% The legacy init kept, commented out, the bridge selection of afe_inv_psm with the paths
% afe_dqctrl/afe_dq_mod1/afe/three_phase_inverter_{mosfet,igbt,ideal_switch}_based...: those blocks do
% not exist in this model.
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
%[text] ## Transition: legacy workspace names
%[text] Old names still read by afe\_dqctrl.slx (block and mask parameters, library-link instance parameters, Variant Source conditions, model configuration) and by the post-processing scripts (tc, Nc, Tambient). The list is the one found in the model XML; migrate\_afe\_dqctrl.m rewrites the blocks to param.\*. Set export\_legacy\_names = false after the migration to check that nothing is left, then delete this section.
export_legacy_names = false;
if export_legacy_names
    % model configuration: StopTime = simlength, FixedStep = tc (unused, ode23tb), MaxStep = glb_time.tc;
    % tc and Nc also in plotting_results_*.m
    simlength = param.sim.simlength;
    tc = param.sim.tc;
    Nc = param.glb_time.Nc;

    % Solver Configuration of the grid emulator
    nonlinear_iterations = param.sim.nonlinear_iterations;

    % variant conditions of the PLL Variant Source blocks
    use_dq_pll_mode1 = param.afe.pll.use_dq_pll_mode1;
    use_dq_pll_mode2 = param.afe.pll.use_dq_pll_mode2;
    use_dq_pll_mode3 = param.afe.pll.use_dq_pll_mode3;
    use_dq_pll_mode4 = param.afe.pll.use_dq_pll_mode4;

    % block parameters
    adc_quantization = param.meas.adc_quantization;
    afe_pwm_phase_shift_mod1 = param.afe.behav.pwm_phase_shift_mod1;
    u_cm_comp_mod1 = param.afe.ref.u_cm_comp_mod1;
    CTRPIFF_CLIP_RELEASE = param.ctrl.CTRPIFF_CLIP_RELEASE;
    kp_align = param.ctrl.kp_align;                     % mask of .../phase_compensation_1/alignment_eval_afe1/pi
    ki_align = param.ctrl.ki_align;
    lim_up_align = param.ctrl.lim_up_align;
    lim_down_align = param.ctrl.lim_down_align;
    time_gain_afe_module_1 = param.afe.behav.time_gain_module_1;    % masks of the clocks in "global timing"
    time_gain_inv_module_2 = param.inv.behav.time_gain_module_2;
    Tambient = param.thermal.Tambient;                  % mask of the IGBT bridge; sim_results_post_process.m

    % objects and structs
    glb_time = param.glb_time;
    grid_emu = param.grid.emu;
    hwdata.afe = param.afe.hw;
    frt_data = param.frt.data;
    afe_ctrl = param.afe.ctrl;
    psm_ctrl.kp_i = param.afe.dq_vector_pi.kp_i;        % the AFE current loop reads the machine control gains
    psm_ctrl.ki_i = param.afe.dq_vector_pi.ki_i;
    filters = param.ctrl.filters;
    heatsink = param.thermal.heatsink;
    igbt.afe = param.afe.igbt;
end
%[text] ## Local functions
function param = add_template_extras(param, options)
% Parts of the shared init template not used by afe_dqctrl.slx, kept so that nothing of the
% legacy init_model.m is lost. Enabled by param.sim.build_template_extras = true.
% Same convention: one group per stage (param.inv, param.psm, param.load, param.spinv, param.dab,
% param.psfbc, param.cllc, param.isop_rail, param.upqc, param.im, param.battery, param.trafo_load),
% each with .hw, .ctrl, devices.

s = tf('s');
ts = param.glb_time.ts_inv;
dev = param.dev;

% ---------------------------------------------------------------- inverter and machine drive
param.inv.pwr_nom = 250e3;
param.inv.hw = three_phase_inverter_hwdata(param.grid.application_voltage, param.inv.pwr_nom, param.glb_time.fPWM_INV);
param.inv.behav.time_gain_module_1 = 1.0005;
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
param.inv.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
param.inv.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
param.inv.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);

param.psm.machine = psm_calculus();
param.psm.n_sys = param.psm.machine.number_of_systems;
param.psm.u_psm_scale = 2/3*param.inv.hw.udc_nom/param.psm.machine.ubez;
param.psm.u_psm_scale_ekf = sqrt(3)/2 * 2/3 * param.inv.hw.udc_nom/param.psm.machine.ubez;
param.psm.mode.use_torque_curve = 1;                                        % energy production
param.psm.mode.use_speed_control = 1 - param.psm.mode.use_torque_curve;     % drive
param.psm.mode.use_mtpa = 1;
param.psm.mode.use_psm_encoder = 0;
param.psm.mode.use_im_encoder = 1;
param.psm.mode.use_load_estimator = 0;
param.psm.mode.use_estimator_from_mb = 0;                                   % mb: model based
param.psm.mode.use_motor_speed_control_mode = 0;
param.psm.mode.motor_torque_mode = 1 - param.psm.mode.use_motor_speed_control_mode;
param.psm.mode.time_start_motor_control = 0.25;                             % [s]
param.psm.impl.use_ekf_bemf_module_1 = 1;
param.psm.impl.use_ekf_bemf_module_2 = 1;
param.psm.impl.use_observer_from_simulink_module_1 = 0;
param.psm.impl.use_observer_from_ccaller_module_1 = 0;
param.psm.impl.use_observer_from_simulink_module_2 = 0;
param.psm.impl.use_observer_from_ccaller_module_2 = 0;
param.psm.impl.use_current_controller_from_simulink_module_1 = 0;
param.psm.impl.use_current_controller_from_ccaller_module_1 = 1;
param.psm.impl.use_current_controller_from_simulink_module_2 = 0;
param.psm.impl.use_current_controller_from_ccaller_module_2 = 0;
param.psm.ctrl = ctrl_pmsm_setup(ts, param.psm.machine.omega_bez, param.psm.u_psm_scale, param.psm.machine.Jm_norm);
param.psm.ctrl.ekf = ekf_pmsm_setup(param.psm.machine.Rs_norm, param.psm.machine.Ls_norm, 1e6, ts);
param.psm.ctrl.kp_i = param.afe.dq_vector_pi.kp_i;      % legacy: psm_ctrl.kp_i = 0.25, ki_i = 35, read by the AFE current loop
param.psm.ctrl.ki_i = param.afe.dq_vector_pi.ki_i;

param.load.b = param.psm.machine.load_friction_m;
param.load.external_load_inertia = 1;       % alternative: 6*param.psm.machine.Jm_m
param.load.rpm_base = [6 7 8 9 10 11 12 13 14 15 16 17 17.8 18.0 18.2];                        % [rpm]
param.load.torque_base = 1e3/param.psm.n_sys*[50 75 100 125 150 175 200 225 275 350 450 600 750 885 885];   % [N*m]
param.load.torque_overload_factor = 1;
param.load.rpm_sim = 17.8;                  % alternatives: 3000, 15.2
param.load.omega_m_sim = param.psm.machine.omega_m_bez;
param.load.omega_sim = param.load.omega_m_sim*param.psm.machine.number_poles/2;
param.load.tau_load_sim = param.psm.machine.tau_bez/5;  % [N*m]
param.load.b_square = 0;

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
%[output:3cacc576]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe400V\nNominal Voltage: 400 V | Nominal Current: 4.009377e+02 A\nCurrent Normalization Data: 567.01 A\nVoltage Normalization Data: 326.60 V\n---------------------------\n","truncated":false}}
%---
%[output:36f93f76]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARYAAACnCAYAAADUrHMrAAAQAElEQVR4AeydCbxN1R7H\/1sUUSFlKNwrDbwKcUuh8CqlSMSjAfHI8JTq4fUaUMrUoDJWKjL0KFOPFGXoFm6EKBlCVJTppnrdyLvvftdrnc9xOufeM+x1zj7nLh\/\/u6e1\/3ut\/z7nd\/7T+q8iufaflYCVgJWAyxIoIvaflYCVgJWAyxKwwOKyQC07KwErARELLPZTUOgkYAdsXgIWWMzL2D7BSqDQScACS6F75XbAVgLmJWCBxbyM7ROsBAqdBCyweOyV2+5YCaSCBCywpMJbtGOwEvCYBCyweOyF2O5YCaSCBCywpMJbtGOwEvCYBCICFo\/13XbHSsBKwKMSsMDi0RdjqlsbNmyQyy67TBo1aiRXXnml1K1bV9q3by\/ffvtt2I8cP368LF68OKz2tLvggguOe96jjz4qP\/30k\/z888\/yz3\/+UzZt2hQWL9soeSRggSV53pVrPa1du7a8\/fbbsnz5csnKypJzzjlHHbv2gABGl1xyieLP8zIzM9XVYcOGyUknnSRPPPGE1KhRQ52L5k9ubq4cO3YsmlvtPQYlYIHFoHCTgfWRI0fk8OHDcuaZZ8rRo0dlxIgRSqNBq2Gfc9Dw4cMlIyNDrrnmGvnkk0\/U0PhST5kyRWk+DRs2FK2JqIsh\/pQoUUI6d+4sn332mWzdulXuuusuQYv68ccf5b777pP69evLpZdeKk8++aTqD+fvvvtude7222+Xbt26qfZoTb169ZIWLVrIihUrZMmSJaofV1xxhdx4442yceNG+f7776VNmzYycOBAoX9oZpMnT5amTZsqfu+\/\/36IXvqdtrtRScACS1RiS+6bVq5cKRdffLFUq1ZN\/vSnP8m+ffvUl3LhwoWye\/duQauADh48KLNmzRLOf\/3110rD4RggQgK0wYx577335IMPPpCSJUvK3LlzuZQvlS1bVsqVKyeAhm64atUqqV69ugIJnrd582bZs2ePTJw4UWk0XEfL2b9\/v75F6N\/06dPVWN59910B5D766CMFXPCg4Q8\/\/CD16tVT\/atUqZLAd9GiRTJy5EjV199++41mllyWgAUWlwWaDOzQCj799FPZvn270hr4NX\/kkUfkww8\/lObNm0uxYsUUXX311fLxxx\/LsmXL1Hm0jVNOOUX9+jNOvux8sc8\/\/3xlTo0ZM0bWrl3LpYipSZMmyg8zY8YMZR4BWN99951s2bJF0EIcx5GzzjpLLrroIh\/vxo0bC\/059dRTpX\/\/\/qrtM888I1OnTpW9e\/eqdmhiaF+O40iRIkVU30844QQB3HJycgQS+891CVhgcV2kycWQL9lVV12lvmC\/\/PKLAhQ9AkwgQEYfB25xvvKFBqAgwGDQoEGBzf5wjKaBtoLWoi++\/PLLMmHCBAGk7r33XqVJYWrhPwEQaMc+phv7\/oTjuWfPnoI2065dO+nevbv\/5eP28xvPcQ3tQUwSsMASk\/gSfLMLj+fLi0lTvHhx5VshigOgQAsWLFDaAr4V9jl36NAhwfTh0fhCMD0ACUAJXwYmEddCEe3GjRsnNWvWVH4d3Q4T5YYbbhAcywAFIIWGhPlC\/+jnV199JWha+h69PXDggNJc8K2giej+6et2G38JWGCJv8wT\/sR169bJ9ddfr\/wql19+uTKBBgwYIG3btlUmAqYRVLlyZbnuuuukZcuW6jwmSZcuXaRixYpqDJhKgA5bzBK+1Oyri35\/cPbq58EXwMBR69dEPXvIkCHKVMEHgg\/o9ddfV85awKp+nlOX0HT58uX9b1P7+GbKlCkjaF633nqrAq3169cr00g1sH\/iLgELLHEXeWIfiI8C3whfVsK\/OHJxeqIZYCZg2nAdYp9zAMHgwYOVv2X27NkyduxYAUC41qdPH6EtpNv7j5B2X3zxhXKe8rw1a9YI\/pxSpUoJhPlDn\/CD4ONBOwFARo8eLY8\/\/rigEQF68H\/22WeVnwQTqkePHgLxLPpHBAve9A\/wA5wAsWnTpvk0IyJN9Id7eCbPpg8cW3JXAhZY3JWn5eayBEiuA2gaNGgg+E8IOWuNyeVHWXYuSsAcsLjYScuq8ErgwgsvlPnz5ytzDY2H6FHhlUbyjNxzwEKqN7kSJDaRcg6xzzmuJY9obU+tBAqvBDwDLHj9sfs7duyochBI9X7jjTcEwjYGVFCD582bV3jflh25lUCSSMAzwELIkpyKmTNnCqna5DOQ3ASlp6cLgPPmm2\/6IhJJIl\/bzUIlATtYLQHPAAvZk0QGyLQkxZx8Bzz6OO7YcgzwEN7UnbdbKwErAW9KwDPAwpwNEqw6deqkJpERMmSSGElYhBqffvppwVzyphhtr6wErAT8JeAZYCGjk9moRAAwh0jdZrZsq1at1KxZ5n6Qsu3febtvJWAl4E0JeAZY0EYwh0h2Ir2cWh1FixZVUsME4pg26oT9E4UE7C1WAvGTgGeAJX5Dtk+yErASMC0BTwELM2RJHafuxpdffilscd5yjmumhWH5WwlYCbgjAc8AC3U1mHfCZLY\/\/\/nPaj4JW1K5Occ12rgzbMvFSsBKwKQEPAIsIieeeKKQvk04ORhxjTYmhWF5WwlYCbgjAc8AC8V\/0EpuueUWFW7Wqfyk81NnddSoUaqquzvDtlysBKwETErAM8ByxhlnCPU33nnnHVWwmSrypPhDzBNCCEzTZ2vJSsBKwNsS8AywaDGR2k+WLUCjz1Hch3Nc0+fs1kogySWQ0t33HLCcdtpp8uuvv6r6p5QohF577TUhYc46b1P6s2gHl0IS8BywkAg3dOhQIcW\/ffv2qmQhS0889thjQvJcCsneDsVKIGUl4BlgwdQhbR9Jk4FLOUIK+1CukBKF1DTlmm7DviUrASsBb0rAM8BC+v5bb72l6pgy6ZD6K1pk7LO2DTVOaaPP260xCVjGVgIxScAzwELEp1u3bvLwww8LS01QgpCV+qBrr71WlSZkUiJtIhkx84smT54szJKmuPKkSZOE9Wki4WHbWglYCUQmAc8Ai+42q92Rws8KfKTxQ1RqJ7Wfok+6Xbhblp6gYjthbELYlGKAd7j323ZWAskggd\/27ZScz5bKoRmDPdFdzwGL21IBRCgghY+GqBJZvatXr\/Y9Bo3IUjW1jrOVQ3LJ4YoLqkjPjDNlXNNTZFevdPl2UBPZNHVIyHfp+9DHYScZgSUisWzbtk3OPvts3z3nnHOO7Ny503fMDlqRCWJFPhN84WmSt2n+tu\/bBRkHo\/xks2XV+wKtGdZJMjtWlSkX7ZYHMkrKLW1ukTN6vyKVBi2RK+YeDcmbHw4+7\/GglAcWhEgIm20qEXWATY7HJH83eQ9\/Z6dAvadvEk1Prszx7XPNpJzc5h1MNj8ufVX2jblTaSVoJhwXOyNNAUmVsTsUqJzSuLMU\/1Njt7sTNT9PAsvGjRuFpT1ZMhOfSPfu3YVEuWhGmZaWJpRg0PeyX6NGDX0o2a0mStn7lhihus\/vNMLXVH+Tie+Fgz4Q6LUVuwVatnm\/bPk2+w+0eOMe3zug\/Y4dOyQWIqcqlvvDuveTD2T7C31l+z8ul+1tHdk37WH5cfcX4rQbKc5d0+RYv\/flp+aPyJ6Tq0Y0Ft+HPg47ngMW8llY+rJv376CIxcQaNGihbAcCNcilQlrE69du1aYDgCxX7t2bR+b0nO6ysGnmxihNX3SjPA11d9k4rtxUCMJpEX3Xy7BSI\/rqvPLCWB\/89S9gmYQDWFWR3NffvdULuVIua+WSakFj0pu\/2qSO\/RKKbpurpxS+QKljVR7YbdUG7ZC0tv+XdKv7hB1330f+jjseA5Y+PJjurD8B+OnLCWhZ8pVco1zoYjoETVcCC03b95c0Hzq1Kkj3MsWInu3Vq1aoVgkzXl++Ux21iR\/k7yRSSj+YzrUkHUPXS5VyhRXWgxtE0VEcTBpcLhi3hDNOZoX2ZFr7vmDiZOoPsbyXM8BC5EbkuX27NnjG9emTZuEQtqAgu9kwA7FuJ9\/\/nlhcbOsrCzp2bOnPPfcc0Ky3eHDh4XoENoK4OQfFQpgYw9TXAJVyhaXeb3ryLxedaT2kBWKpn+81\/ioNZD4+0oO5YWG8ZXgeMVXgvPVyQMWL\/lKohWM54AF8GDFw0GDBsmaNWvk+uuvl7\/97W9y7733CqATaqCsRZSWlibnnnuuakJhqAMHDghTAvILN6vGSfgnPT3daK9N8jfJG6GEw79h9dJKe+mQUVFw+rYcs1Z2HczhdlcpUCv55bOlAohUm5mrtoAKjlf90HD6rtt6ees5YEFYgAIJbStWrJCFCxcKJk5B5st5550nTF4sVaqUHD16VKjhQukFnG3YxfCFgoWbOW8pegkk850DmqUpgGEMaDBuRJHQSjBxcLwGaiWACs9KdfIMsKBxMJuZynEQEaFWrVoJFeTwsXCNNvqFsAYRbYjN\/\/3vf9enhZUU27VrJ5s3bxZmRGNW4bPxNUiRnVB+BLeGZ5K\/Sd6MP1L+2jwa0Cw9L3S9Q5lHmduyYVUgaRNHAwlgglZSomZjCaWV5Mc00r7nxyuR1zwDLGXLlhV8JDNmzJCWLVtK165dVUU5qsphGuGMLVeunE9WpPcvW7ZMJQPhV8nNzRXmBPXu3Vvuu+8+GTdunKCxYB4RYtY3sk+kSR+z5WWaILQlE3zhabLfpvl7te\/tzsuVtzqdLddXLy4tx66Va55aIR+u33pcSFe905lPyvbht6hwMI5XwsE5JU4XZ8R2RYSDszM6HncfMg2HTMoG3vEizwAL0R+AgC2lEdBUKlWqJNCtt94qn376qXLghhIML5yZzy+88II0atRI4EPbgsLNtMGuNUGYYCb4Wp7pUYdcC5Jdg1rnytD2dZR5VKJ4cWkx6WuZseX\/4eCTJnWRs567UnJn9JOiX62RMpdcp7QSFQ4e8IaxPhXU53Cv81mPF3kGWPSAqcRPBIhUfH3uu+++U6CiwUKf99\/SnvByx44dpX79+oLPBXCqXLlySoab\/cdu992XAObRRHlOVu9uKm1fSpf1Ex6QVYdPU0lq2sQp026g+w9OEY6eAxZKU3bq1El69eqlNA+0jw4dOgiAgbkUSu74YQhL47TF\/Klataoyh3bv3i2pGG5GrQ4lCzfOm+Rvkjdjj4Y\/0Rt\/pysmDnklgEjRCb\/Ih51XSFe5R7rvamAkekS\/oWj6zn1eI88BCwICJPCfTJs2Tah3S6X+pk2bcilfIho0evRowR+DWUVj8lcuu+wyKVOmjApXZ2RkiM1jQTKFlygvEAgkRG9wupJXAphUGbtDJaohJbSXAQaiR\/BOVfIcsBD5IQJEBu1tt92mNBXWFeIc1\/J7EYsXL5aTTz5ZLR+i22Ei4evQx6kSbsau1mMysTXJ3yRvZKH5Ax6Q1kTQQojaEMEBSNBISErTQAKYkFcCj2AEwLzQuoLM61XHFz1yIzzt\/yzdd\/9zEe974AbPAQvmDtGhN954Q6AXX3xRLWCmtQ4ts8BwM85bcl6YsBjoi8kv3NwvbZ+aOcqHz23Cyec2TDAojAAAEABJREFUz2TixxfYFAESmgCLYISsNICgieATAUQgQARQkSj+kVzH\/COS64a\/8\/\/wdBRsUvoWzwELoIAZQzgZYs4Q4eP\/\/ve\/ylei3wbXMJeoaUG4+YsvvhCiQnXr1lXO25UrVyrNhUxeQsz6Pvb9w80VTvxNzRxl9qjb9Ove7cZ479+23hhv5OAGf0Kwwejgf0tIsPORnPut9k2iSYd59fabu5ersC\/HhH5\/7fSymg28v+pVUYWA8Xto4gdM77fLC08z0bRW+aLCzHDC0\/patFvM\/mjvLeg+\/R2Ix9ZzwBJs0Dhgv\/nmm2CXfOeuvvpqldMC0AAqRIYoSUmSnZ4jxDwh9mvXru277\/4tFYWZoyao+N1vGuNd45lPjPFGFq7wzwvBVgtCFz66QIKdj+hc91FS7XfCfPAnHP7+x27uY1YH8pvULUOZR\/t+EWH2dOD1SI5N9t33oY\/DjueAxd\/EqVatmiqzh6\/lkksuUQ7Y\/GRCzRZyXm644QZZv369oMUwozkVZzfnJwdPX0vRzmEeMXN6QLN0pb0wPSAekxu9Kk7PAcvpp58u8+bNE0wWtA+IiYTUZMEcCiVIFjgbOXKktG7dWrKysmTYsGGKDxMZbbg5lNRCn0etDn01tismedMz0\/x5Riga8Hv0CP8LkxsBmEgmNyay76HGFM15TwEL6wcxx+fBBx8U6tKivUBoHswHYrZyqEGirRw8eFAISzuOI9ddd53AB2DRjl9mR9twcygJ2vNuSYDokQYYar8ALm5Hj9zqqyk+ngIWck5eeukl+fzzz2Xs2LEyatQoReSy4EMhYhRKED\/88IOa1cwyIRdffLHccccdkp2dLTbcLFH9wy8Q1Y1h3GSSN483zZ9nhEMAjK79Mv3jPRIOwHil7+GML782ngIWEuNYTvWpp55SJRAoR6npL3\/5izBTWQ8GTcZ\/djPTANB2mLzIvCL8Mg888IACm\/zCzfBD\/VQUYz3UQB7+EYTAa\/Y4ttqziZJfNO\/0rBMOyezbKqjJjYSnqb37RuammCNUkcqAz3q8yDPAQvIboMJ2+PDhyqShfIKmwAS5wHAzM5\/r1asnOpSM+VOkSBGpWLGi8tdogeK70W30OX4lTFCwCIJbz6HvbvEKxsckf5O8GYtJ\/rG8Uz258Y7LK0v3WXvlngXZcsJpFY+bvGiy7\/COF3kGWMhdwSdSpUoVVS5h+fLl4k+UT6BNKMGkpaXJr7\/+Klu3blVNmDeETwWthhAzoWaI\/dq1a6s29o+VQLwlgHmk\/S88OxzziHbJRp4BFn\/BYeaQfYu\/RBNaDNEd\/3b++xpEMJmqV68uffr0kS5dugiaSyqGm\/Uvs78M3Nw3yd8kb2Rgmj\/PiJUAGPwvA\/LC05hHGmDi3\/dYRxL8fs8BC1pF3759haU+GjRoIJqovF+sWLHgo8g7S0RoyZIlQklLtBZq5M6ZM0fQUAAkHMPsw99OQswTmP3vCQkMyAtPB04PyNyW7Ym+xdIJzwELgEJxJ8omkOimCccu6fn5DRZTiEgQbXJyctRsZgAFrSXVZjfjuGOcpsgkf5O8kYdp\/jzDbQJgSLA7o4So6nUU93b7GfHk5zlgIaSMLwUfSSSC4D4cvJhCzGCePn26UMclVcPNkcjGtk0OCWAe6dnTuw7lqAze4e8cv854coxExHPAwvpAzPUBJDB\/QkWF8MPgmCXtn+Q5MnSpefv222+rKBDnmJxIRm4qhpv5gPHLbIpM8g+f946oQrIm+UcTbo7kHdF3HZ6+69LSgv+F8PRzCzZEJQv\/Z8M7XuQ5YEFbmT17tgIHUvN1ZCgwKhQYbgZYypcvL0SVHMcRQs9k6laoUEHx0gJNlXAzTj5L6ceFauMhj1jCzZH2j\/A0\/hfC04MW75ebp+79Q3g6Ep76OxCPbcKBBccq2gfEPkQESEeD9Hbw4MHClHKqxAUTDLVtyYGhPi7XSeUHpNBqtNMWxy37qRBu5peIcZoik\/xN8kYepvnzDFMUrO\/a\/5JM0wMSAizHjh2T+fPnC1XiSNW\/5ZZbBLr22mvVJELmBgESVNjHHKJsAk5d6q0MGTIk6DuleHbnzp2F2c1XXHGFTJkyRajjUr9+fQFcWFGxWbNmghOYmdL+TDCnTBDjM8EXniZ5m+Zv+\/7\/WfvIOZBCyaZxvZqy8ak2UipzhIyc\/bGU6zFTzfwPvD+\/Y\/\/PvOn9uAMLEw0fffRRVXWfcLC\/uYNv5ZVXXhFCx2TNMqO5VatW8uyzz8qRI0fUesxcCyYUx3GE9phOrJz4r3\/9S6nJjuOo8paZmZlqRUXWKHIcx8cCEyo62u6r\/2Lvt7KI12dgV9bbsn98W+l3c0ZUnz\/fB9\/wTtyBhZDxQw89JFTiJ6ktcHwlS5YUzBpMH30NDQdAwhGrz9mtlUBhlgDmkZfHH3dgofQkiW6YOkR+dNRHbzmHn4XtP\/7xD8HHcueddypnLD4WwspeFqjtm5WAlYAkLtxMwhqJa+SaEPEh74QaKo0bNxZ8Ieeee66aiEjm7YgRI4T1mDF10Hbsi7MSsBJIiATCfmjcNRbdM\/JVqGNL3RQybc866yy5++67lX+FsDE269KlS4XqcRMnTpTOnTurW9F21I79YyVgJeBZCSQMWJgYSNq9Dg8jIcwjjl9++WXBF4OjFs0Fv0u3bt2E8DHtLFkJWAl4WwIJAxYct23atFGV3qhMjo+FdHzmBv3888+CWUS4+cILL1QzldFeyEPJT5xMQoQPJhahZtrn5uYKGbmErRs2bCiTJk0SnMH58bHXrASsBGKTQMKAhW43zvOnEF7u16+fTJgwQWbMmCEAC2YReSdk13711VeCwxewIZeF+4IRuS6YTCzLStgaTYfjTz75RFgGhFnPpPu\/\/\/77wsTEYDzsuaSWgO28hySQUGChvi1zesaPH68WI6NcAvkmbdu2lXHjxgmLlWEWAUCATH4RISYbYipVrFhRHMcRtBMmMpLTggaDsxgtyRbT9tCnz3YlZSWQMGDBf8LaP0OHDpXSpUurZLbRo0fLnDlzpFatWsIEQrII586dK8uWLZOBAwceV\/NWAv5VrVpV9uzZowhTh3sAmy1btgjzO3RzZj7v3LlTH9qtlYCVgAEJJAxY+PLjwD355JPVsJgwqDUWwsr4SiBSnG+66SZVAgHnrmoc5A+TsSigTf5L\/fr1Ze\/evXLBBRcIGbwFzW4Ows6eshIwJoFDMwZLfsSa06bI2KACGCcMWCh+jemCMxXfCeFlNIwTTzxRSIjjPIvCa6JUZX6mEFm5mD8k0eFDwflLVX\/KVDKjWY+bff9i2mhFcaCI53XYPoWeT5Mo2fTMOFMGNSwr45qeIrOvK6Hoo5uKyfa2TthE+01Th0h+tPXdKZL51utGSH8PTG8TBiyO46i8FUwTQIH5QKT5v\/rqq8LiY+3btxf8L+vWrZPTTjtNhZpx4oYSCGZQz549hdwYgIq5QkxwZEIiM5qJEEHsB85uBtRM0HvvvRfVfI5w+mKSN883yd8kbzf6vv7lR2TV\/VdJZseqsrjuDh\/1S9snHWuWlJuurCcNW7RXVOO2h+SM3q9ItZm5YdEVc49KflRx1BZ1\/eaFv4jbdP+WihKvf3EHFkwgTBrKJPBFR7MgesNsZEwZkuWo1k9kZ\/DgwcKW1Q0pjk2qfyjBML\/oxhtvVBm6OHs5hjczmQua3RyKp5fPY\/qZ7J9J\/iZ5I5NI+P+2b6f8uPRVwfTQmgdmyi+fLZViZ6T9ASycB5ZLpUFLFJgAKGXaDZRTGnfmsa5QJH135YGGmMQdWHDaAhKUSQAI8KOQzwJdc801MmrUKCG0zDrNaDLhjttxHGHmMlGgVatWSf\/+\/YUsXcfJf3ZzuPxtu9SRAGACeHw7qIns6pWu\/B1H8wCmTNtBCkiqjN0hEMCROqOO70gAlrg+Eb8Kc4PIKwFIyC3BLwLNnDlTfvjhBxk5cqRgwjz22GOCI5bcE3wsp556alz7ah+WOhIIBBO0FK2RACJoIWgfqTPixI4k7sCih4sZhC8EoOEc5hGaDNmznAN8CDnjDwFsuI4ZRVtLouqfmpRDsEpmbj3PJG\/6qPkDJgCI1kzY9wcTL2okuu+MI5kpYcCCQ5blOsi4xVlLeBigQZhk4OLIxVzSBOhgRnE9Esq1Kf2RiCs12q5+UzSYYPJ4HUxSQ+jHjyJhwEJuCclx+FGIAPXo0UOYM0TmLUQlOH9Cg0GTOb77BR+lakq\/aSefSf5u887Jc7QCIIAJDtgTlozxOV4xc4JpJgV\/chLTwm3ZJGYUCajHglaCdsKA8ZkMGDBAABDKIzz++ONC6j3XdBv2YyFyWmxKfywS9Oa9RHE0kLBNBjPHm5I006u4aywkrVEUGw2F6A0lJ\/XQ2CcVn2u00edj2ZJ0l4op\/aZtcZP8w+WNFqIBhOgN2gjEvg4Ho42QQ+KvmYTLP5bPlal7k7nv\/jKJO7AQAqa2ysMPPywLFixQVfN1JiVV+tFcKLZNG\/+OxrKP2RXL\/V6817TKHIw\/WgFf9oIIMAhGaBbQSZO6qDAvAAFQhCKedzQvDIyPBOAAQCD2IUAlWA5JsL578R0G61My991\/PHEHFv1wSiOQAIepQrYkRA4KNW6ZyazbxbpNS0vLd8EyMitDfbBjPZ\/bv1rYqd7+z\/LqPmDx2ezxUhBtzlwoS1es+wPN3HmSaJr9n\/Ol+ZmT5a76HykafG2WvNjmE0XOiO0CHev3vvza6WX5qfkjKgrGr3miyfRKiCbHF+t3KZL7i0TSOBnbUiyKNH7C2xD7hLD1WK5ek66SovgldJv4crjNM9H8Gk7fI+FQ2ylrJZDueXWhQDkdxgvECn\/nVSotJYoXl0NHispnB0QmZGVL3ed3Hkc3T90r9yzIlhlbHDULnl\/1RBFmdaKeHetz9Wc+HtuUB5ZUTennl83kB8Qk\/3bn5QrLV0BjOtSQeb3r+Ojg003En7jeIaOiVC5bXD7cdkjK3rdEUe0hK2T4O8HLX5jsu0mZwzuZ+07\/NaU8sDiOTenXLzsZtx0yKigQAmAAoHUPXS7zetWRDnlgM\/3jPQWCTDKOORX6HDGwuDlosmlZO4g8FvwrFHoiMuTmM1KVF2qxybGZ5B8L7yp5mkvD6qUV2AAyECAz\/J0dghaDTGLhz\/2JpGTuu7\/cEgYsJMYNHz5cKJtAh7Bdjx49quYJcY1zlqwECpIAQINJhflUpUxxpcGEMpEK4mWvuyeBhAEL6fnMYKZCHFXeqCbXrVs3wcHKNfeGmJqcTNviJvmb4o2phMmE9pKsb92UbOItj4QBC+sGUbjpP\/\/5j2\/MmEaACkl0vpMR7DCBkTIMZNqm+vIfplVmk\/xN8u6Q55PBB4OTt+WYtRF8evJpGsdLJmUTx2FIwoCFivkUcOrdu7dQ8JpqcUw4ZG2h\/EpQhhKOXf4jlGQK33l8MJhGmV9mh4wcFT6pxHfECQMWhsnaQf\/+97\/lmWeeUUWa2Occ1yKlbdu2CZMUCxofS60AAA30SURBVMvyH6ZVZpP8TfLmc6P5o7lgFmVuy+Z0UpDue1J0Np9OJhRYMF1Y1qNChQqqchylJFlHKJ\/+hrxkl\/8IKZpCe0FrLi3HrpVdB3MKrRwSMfCEAQvFmyh4jSnEaoc4cRcvXiyUOcDPUpAwqJlLLVvmGWFGYZtSM5fQNVXnmB0d7vIf\/EqYIJPp38hH9XnHDiPp7ib5m+SNTAL51z2ruHR9Za0rcjL5ToP1nXNuEXKJFyUMWCjyVKpUKSlfvrxkZWUJDlccuiz\/EU64mflEzIRmjhGV5rgnmuU\/EDSgZIIIoZvga3mmR5TaP\/HOOrLmmxyZ4cKUgGR+p3zW40UJAxYqyB05ckTuueceWbFihaSlpcmsWbOEczh2IxVALMt\/RPosL7TnV8xkP0zyN8kbmQTyJ9dlXq86eY7cHZLpcX9LYN8ZTzJSwoCFUgZDhgxRKxyOGDFCKPqEACmZgObCfiTEch9U\/W\/Xrp00btxYOMZnk6pzhSKRjW0rov0tf3t9k+fBJRXel1lgKUBCAEitWrXk9NNPl5ycHFWaEpAhn6WAW\/9w2XEK1\/IfmEN\/EIKLJ0zyN8kbEeTHn+xcL4NLfn1nbMlCCQMWfCJoLC1atBBWLGzdurW0bdtWiBCVLl06WeRn+5lkEiA7l\/lFgEuSdT2pupswYDl06JAQ\/WF9ofvvv1\/GjRsnU6dOVZqL4zhJJcREdNa0LW6Sv0nevItw+KO5kJ3rtXlF4fSdMXqdEgYs\/oIhhf+7774Tqsrt379fAY7\/9WD7aDwsxbphwwbfZfJiSOlnsbNbb71VrQFtl\/\/wicfu+EkAzWVAs3Tl0NWzov0ux7Brb0UCCQOWcuXKKbOHJUCwK1966SWVgcsaQ\/he6FwoIt+lY8eOgraj26D9vPLKKzJp0iShbi6rLLJ6InkxixYtUm1ZdZFVFSmHqe9L1i0yM9l3k\/xN8kYm4fIf0CxNFZXqkFFRzYoGYBKtwYTbd8bpZSqSqM45jiPUYnnggQdUDkvPnj2FvBaWVy0o3Iyzl3lFZNv695\/cmOzsbHUKZzB8ABFyZFhWhOOMjAxZvXq1amP\/WAkgAQBm3UOXH1c8CjPJC0BD\/5KREgYsCIuyCZs3b5aFCxcKBZ7OO+88YUIiaw9xPRTVqVNHqOiP1qPbMHGRrFsmMVLjZfr06SqUzRyis88+WzdT9V927tzpO07WHdO2uEn+JnnzPqPhT66LBhhAZkCemYQm86FfOUzApsWkr1VBKWZO956+SdzWcKLpO2P2GiUMWAAP0vmffvppZbpgvkBk4VLwyV9QpOyTuk8KP6n8\/tf0Phm4kydPFsydL7\/8UrhHZ+SSM6PbBdvC2wQxTcEEX3hGx7uacG84ZJK\/Sd6MLVb+jevVlAk9myra+FQbKT2nq48OZs2S7z+aIVlL5svMt5fJyNkfKzMK0AmHyvWYKflRxvDV6vqZHceL2\/RTw\/7BPv5GziUMWCjohOmDb+WJJ54QTayMqJPl9IgBCIBj2bJlQiq\/Pu+\/5TrTA6pUqSKO40i9evXkwIEDyo8D0Oi27NeoUUMfCvdZ2m7lsD08GXy74BmB9r7xkHw\/uYfsH99W+Wko0xAO0b7fzRlSELW9\/ioxQb4PvuGdhAEL\/g6ctNEkwwWTCZm28CK6xPU1a9aoMgpoOSz5AZBB7Psv\/0FbS1YC8ZTAgDyncUFEJTwTFK9xxh1YDh8+LNS6feyxxwQguOmmm+Suu+4SFiqDuEabSAWAf6Zz585CmJlw85QpU4Qqcsx0Blyo89KsWTO18iJp\/pHyt+2tBKwEwpdA3IGlZMmS0qVLF+nbt6+QeUvoGJDp27evOsc12hQ0BMyoCRMmyEUXXaSaOo4jZPEuX75cqPhPSYb09HRlFhGazszMVOdvv\/12dU7dlIA\/hNM7dOgg+AHefffdBPTAPtJtCWzatEkIHPCjxlw3\/IduPyPZ+MUdWKhzS1j4kUceEULGOG8dx1G+E\/wnVIGjTbIJMtz+EgHr0aOHmskN2GGehXuvbedNCRDRJBAxbdo04XONie\/NnsavV3EHFtAcDYWlT4ni4KjFcUsmbfyGnbgnYf6hSVE2As2MqQ2J6419shsS4MehsHx+w5VX3IGFl8AXqk2bNkINlr\/+9a9qyY9o\/CrhDtIr7fjwHTt2TIoXL666xAoFVNJTB\/ZP0kqAqnLPPfectGrVSvDtMY0kzMGkbLMiiRgZ6wj5P5cvGF86\/3PJtk8UCjVY95tcHBzRZPpS2Q5fEiaef4U8ImNkEet77NZbEgjnnQIijRo1kokTJypQWbdunbBihLdGEv\/eJARY4j9Mc09k0iSmHDY2AKmfhC+FXzIcya+\/\/rqQr0NWJRnCW7duVRMtaY8pqO+xW29IINJ3+vnnn6uAAImYBBUcx87OTwiwMH\/n4osvVlmghINXrlwpbMmaJDQcKrvWGx+743uBFkIy3s0333zcBZL5mjdvLsWKFVOztitVqiRMXyDkTYkIas8wZj6Ix91oDxIugUjfKT8OTCUh0keCpv8UkoQPJkEdiDuwEPnhSxcq25VrtEmQPCJ+LJMbmbdUt25d371ECXDK6rlMjuMoEGU1AkpDoMG89957ar6T7ybv7RTaHkX6TjGFSB0g1YF0CcexGkvcgaWwfFr51cOfUljGWxjGad9p+G\/ZAkv4sgq7JVEvTByS4bgJBx\/+lfPPP59DS0koAftOI3tpFlgik1dYrR3HUVMHMOuIDn3zzTfCHKbq1auHdb9t5D0JOI59p5G8lWQFlkjGmJC2LD2CrU6omVTvrl27qiVJEtIZ+1BXJGDfafhitMASvqzybclKA5R30I2IBlECYtWqVYJTr0mTJvqS3SaJBOw7jf5FWWCJXnb2TisBK4EQErDAEkIw9rSVgNckkEz9scCSTG\/L9tVKIEkkYIElSV6U7aaVQDJJwAJLMr0t21crgSSRgAWWJHlRXu+m7Z+VgL8ELLD4S8PuWwlYCbgiAQssrojRMrESsBLwl4AFFn9ppPA+C7hdeumlcuWVV\/ro8ccfT\/oRk4DImt3MKCfDmYJa\/oNi3OPHj\/c\/5dtn\/taoUaOEaRe+k3bHFQkUCmBxRVIpwISCVGQBa3rwwQd9o2KiJGUzfSeSYAcwmTNnTtTlJ6g9zAx06gElwXCTqosWWJLqdbnfWX7Ne\/XqpZZOWbFihWzcuFFY64k5Tixpwa86T12yZImwXhMaD+s1DRo0SCjIhZbAljZoC2gIgBS1X2kLH5bEAAS4Dk+WY7nsssukW7duqt4xGsOIESOEc9S1oRAWgHHvvfeKLlmKZgHxHE2UjqS8JzVu9LlQW7Qz+gPVqVNH6MPPP\/8s1FKZNWuWsHJEqHvt+cglYIElcpkl7R3du3dXBaeo1FerVi3ZsGGDGsvBgwdl+vTpQlmH0aNHy5gxY4SlSfhiszzLtm3bVE3XGTNmCDO2+SIDFOrmIH+4l7V2KGb1wQcfCCUH5s6dq1qySgGaE23QFlavXi2U8aRO7NKlSwXwwbyhFvDevXtlz549CnyoJUuFNsXk9z+0r1mz5u9H\/9\/4j5FxAhpcQTtDU2OcFHGnlCj9YiyMn9nntLPkjgQssLgjx6Tg8sILL\/jWaF6\/fr1vsbfGjRsLv\/w7d+5UEyb5VedLycJqn376qdJiWBebL6HjOMLkPOrNhBo0wMAXGKA655xzFFCxtC3t0RYo3cgkTRabw\/yiLjCgwXo8zAjHZ4J2xOqW69atky1btgjPCyw7wYoPnIevJv8xUqWwdevW+pKw9MzgwYOFVTHxN3EBcGPVhJRbLYHBJZAssCRQ+F57NOYAqwoAJnwpKfo9b9489aXGvNH9PXLkSL4OT0yM\/v37+0AM7WVQnumk7w\/cYgpRiFqfz87OViDQtGlTpSFhhrEOFcCj20S6pf+vvfaasCDenXfeqYpfR8rDtg9fAhZYwpdVyrdMT08X1ncCUPgizp8\/X5566ilVRwZThYJVnMekAISKFi2qioXn5OQI5\/F5ICS0AbQQNAq0hIEDBwomEdeCEWCGiQXAsLJBnz59hJrBtWvXVn4cfD9oUYH3VqhQQfIzyfzbY24xHkwgtCV9DY2JvrOAnD5nt7FLwAJL7DJMGQ6YOv369RMcsA0aNBCKfvfo0UMwSXDSYlawigIAgwmC2cIxKw5QUBxfDGCDqQRYsMXMYskT9kMJ6oYbblAaBI7Udu3aCYvZYS7xZderOXAceD\/PwE8EMARe8z8GsNBWdu3apdZYBqRwIuPvwY\/DWNBk\/O+x+7FJwAJLbPJLmrspQhXsyw1wQHog+DZwuqJxsBYxy5Y4jiMsb5KVlaV8MJgSfJkdxxGq0uNTWbRokbz44osybNgwpcWgdXAewixCS+D59EN+fxjP5Rxf7KFDhwphX57LsxzHUREhvvgtW7ZUwPP7bb4NESQ0G7QceNBf+Pka5O3wPPrCNfw8aFsQoAmYMFaWaYnFzMp7jP0fIAELLAECsYfekAAhbAAGDYh1m4L1CjC54447BB8MQBesTX7nCKWzaBxaV37t7LXIJVAk8lvsHYVdAkRziK6YlANrS7311ltKA8pPmyDKRE6K40S+lg8+Jcw+tCmTYymMvC2wFMa3bsdsJWBYAhZYAgVsj60ErARiloAFlphFaBlYCVgJBErAAkugROyxlYCVQMwSsMASswgtAyuBZJeA+\/23wOK+TC1HK4FCLwELLIX+I2AFYCXgvgQssLgvU8vRSqDQS8ACS6H\/CHhdALZ\/ySgBCyzJ+NZsn60EPC6B\/wEAAP\/\/990uwQAAAAZJREFUAwCLe97HziViZwAAAABJRU5ErkJggg==","height":167,"width":278}}
%---
