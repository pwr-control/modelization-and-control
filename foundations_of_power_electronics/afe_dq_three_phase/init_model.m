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
param.afe.hw.displayInfo(); %[output:106360cb]
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
figure; bode(param.dclink.sys_dclink, param.dclink.Z_Lstray_dclink, options); grid on %[output:2d6a3809]
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
%[output:106360cb]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe400V\nNominal Voltage: 400 V | Nominal Current: 4.009377e+02 A\nCurrent Normalization Data: 567.01 A\nVoltage Normalization Data: 326.60 V\n---------------------------\n","truncated":false}}
%---
%[output:2d6a3809]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAQAElEQVR4Aey9DbxWVZU\/viyNCwqBf3lHAsSUnyGakTiaL79wtByiTEwZE8cawLDwJwj5GqZGKjjSSCGVRRqkFEqMk43MKEYJQw0C46B5BVLecQS5hJCY\/\/vdsq6Lxd77OffZ9+U55y4\/rrvX215nr+8+9z6Lvfc5z\/vesf8MAUPAEDAEDAFDwBDIGQLvI\/vPEDAEDAFDwBAwBOqJgLk3NwJWwDT3DNj1DQFDwBAwBAwBQ6DeCFgBU2\/IrIMhYAgYAs2PgI3AEGjpCFgB09LvAMvfEDAEDAFDwBDIIQJWwORw0mzIhkDzI2AjMAQMAUOgeRGwAqZ58berGwKGgCFgCBgChkAZCFgBUwZo1qX5EbARGAKGgCFgCLRsBKyAadnzb9kbAoaAIWAIGAK5RMAKmLKmzToZAoaAIWAIGAKGQHMiYAVMc6Jv1zYEDAFDwBAwBFoSAg2YqxUwDQimhTIEDAFDwBAwBAyBpkHACpimwdmuYggYAoaAIdD8CNgICoSAFTAFmkxLxRCoLwKrVq2iAQMGUJ8+fepo+PDhtGvXrkyh4Af\/hQsXZvKHn7wW+PHjx7u+9Y3lOtkPQ8AQaLEIWAHTYqfeEjcE3kXggx\/8IM2fP5\/WrFlDK1eudMolS5a4tjF+HH300YT4fL2NGzcSipgjjjiCZs+eTYMHD26My1ZGTBuFIWAINBgCVsA0GJQWyBAoHgJyxQRFBmcIHqsnl156KdXU1LCaZsyYUbeSA586Q4BB0XL99dfTiy++SOvWrSNezeHVGFwDBD10IPCsGzJkCGEVCdeF\/qyzznJjkOOGL+StW7cS\/KdOnerGiJUn6NEHPogRGKapDQFDoAIRsAKmAifFhmQINBIC3rBvvPEGDR061H2on3jiiXTmmWe6VRAUBt\/85jfd6gxWZrBSgg95fOgvW7bMraJ861vfoldeecXFhf6ZZ55xqzjsD50zRn507tzZWVFgOKb2x7Zt2+iOO+5wq0JYrUGRtHbtWnrooYdqreSuceWVV9ZdG8oNGzbQ3LlzafTo0RBdX6zyTJgwgZ544gmn27lzJ23atMnZzj33XLrttttcH\/hg7CiQnKP9MAQMgYpHwAqYip8iG6Ah0LgI6C0kfJCjUNmyZQv16NGDevfuTVgpQWFTXV3tioGBAwdSp06dnO2EE05wA4QNxQaKIBB4LhycQz1+4JodO3Z0KzKDBg1yhcqbb75JGBvGgfHgGj179qyL2r17d2rTpo2TsQ2F4gkrK3fddZfT8Y\/zzz\/fsX379iXOA7xT2g9DwBDIDQJWwORmqgowUEuh4hFAYYACAcVIOYO98MIL3eoGVj5AU6ZMKRkGhRKcUBChBaH4QIGCVRYUQrJQgT1GWEXBdtIDDzzgVmqwuhLzN5shYAjkEwErYPI5bzZqQ6BREMCHP1Y5sCKBrZ3169cTtm6kHisY2ELClg9szz\/\/vBsL+jz55JPuTAr8UURgJccZAz\/gN3nyZDruuOPoqKOOOsALKy8gbEdhm6p169ZuewvjQz\/WH9BJCCh+IMIfrZEhYAgUC4GWVMAUa+YsG0OggRDQZ2AQ9rLLLqP+\/fvTLbfc4s7HYDWkW7du7nwJtmew9YLi4oYbbiBeHYH+qquuOsgf8SS9+uqrhL7Y3uG4eqUGepxpQYtzLdimwkoNxoVY0GOFha8NHROvIo0cOZJOP\/10OuWUUwjnd3bv3s0u1hoChkABELACpgCTaCkYAuUigCJlxYoVB2z74FFmFAGIiaIEW0EgWWSAh27BggUEgh\/8R48eXRcLPtBJgh\/6SWI\/XBPXhg+2kxYtWuRi3X\/\/\/XWPV7MP+mOFBYdysVKE66Iv7LgeZPggt3Hjxrn+vXr1IsREfPbha0Mn+8NuZAgYAoxAZbZWwFTmvNioDAFDwIMAtqSwcgPCCsvNN9\/sDhN7XE1lCBgCBUfACpiCT7ClZwgUCQFeWcHqCggrJ0XKz3LxI2BaQ8CHgBUwPlRMZwgYAoaAIWAIGAIVjYAVMBU9PTY4Q8AQaH4EbASGgCFQiQhYAVOJs2JjMgQMAUPAEDAEDIEoAlbAROExoyHQ\/AjYCAwBQ8AQMAQORsAKmIMxMY0hYAgYAoaAIWAIVDgCVsBU+AQ1\/\/BsBIaAIWAIGAKGQOUhYAVM5c2JjcgQMAQMAUPAEDAESiBQ8QVMifGb2RAwBAwBQ8AQMARaIAJWwLTASbeUDQFDwBAwBAqPQOETtAKm8FNsCRoChoAhYAgYAsVDwAqY4s2pZWQIGAKGQPMjYCMwBBoZAStgGhlgC28IGAKGgCFgCBgCDY+AFTANj6lFNAQMgeZHwEZgCBgCBUfACpiCT7ClZwgYAoaAIWAIFBEBK2CKOKuWU\/MjYCMwBAwBQ8AQaFQErIBpVHgtuCFgCBgChoAhYAg0BgJWwDQGqs0f00ZgCBgChoAhYAgUGgErYAo9vZacIWAIGAKGgCFQTAQap4ApGFbr16+nJUuWGBkGdg\/YPWD3gN0DTXoPrFu+mFY\/dEfSNfEZVrCPZZeOFTAOhvAPTPx1111Hw4cPNzIM7B6we8DuAbsHGvUewGfN\/7viYvrxVz9DmyadQ3\/91ido+9xJSdfEZxg+y8KfdPm0WAFTYt4w6UuXLqW7776bZs+eXW8aO3asu0Ksf8zHZyulk\/YsfGPlhbjy+pCZsuq1n5R9\/Oc\/\/\/mSePMYfK2M6bOzLubns2ldTJY25lPzwrg5VmPei\/oaUvbxeclL4+fLBbiG9OhfX5KxYn1Dfln12k\/KPj51zmTMcvJCH18MrYvJ0sZ8al6hcUEvia+H+0XqwX\/rujF0edftNPW4TXTDwMPpjCGX0Cuf+jZdturous8g3V\/KPh554TMMn2Xuj2OBflgBk3Eyu3fvToMGDao34ebBTXXqqacG+8Z8fLZSOmkP8RgPUm\/MvICXvD5kpqx67SdlH4\/4zZkXri\/HBRmkdTFZ2phHjJS80J9jYe4h+yjm47NpXUyWNuYxhjzkhXHymIFfFh5+KbnJa+D6IQr5ZdUf6DeIpOzjMY7mzAvXl+OCDNK6mCxtzCNGSl7oz7Ew95B95PP52DFd6Jg\/3E8f\/4\/xdOWA9tTv72+ibrc+RR3H\/Ij6\/u3fk\/wM0f2l7OMxBuRVRLICppFntUePHu7mQxu6FGy4QdFqH+i0rZRO2kO8vk59ZRk31jfkl1Wv\/aTs41GQxcZTyiZjxnxjfj6b1sVkaWM+NS\/kwrHQQvYRbPp+Yz+fTetisrQxn5e8gAGPGS2IcQrx6JNCMm4sTsgvq177SdnHp86ZjFlOXujji6F1MVnamE\/NKzQu6CXx9bq02kc1T\/+YNtZuE73yld705vNPu4Kl53fXUoeLv0GHduzlurE\/WijQ8r2nZWljviHywnUqkayAaeRZ2bNnD61du5bQhi4FW8jHZyulk\/YQ36VLFxoxYgShDY0rppdxy\/EL9df6mCxtzLdv375Z8wIWPBa0kEHg5RzHZGljPjUv3xig08TXQ5vFBr+GzEtfM4usx+DrE\/Px2UrppD3E43fLfscOng2J18HW9zQxP59N62KytDHfZL9jG6tpzcxrCEXL9kdupcNqC5Wuk54iFC6HDbrkoM8LHh9aoIM29DsnbZJHvyKSFTBFnNUMOeGP6+WXX152AZPhEs3i0rlzZ7K8mgX6si5a1PkCGPY7BhTyQ415L+7btq5utWXj2GOJfv8L6jBskitasE3U+oSz8wNUBY3UCphGnoyqqirq3bs3oQ1dCraQj89WSiftIT40lqx6GTfWJ+SXVa\/9pOzjW7duHRtOCRu5eQrNhewsry314H02rYvJ0sZ8al6hcUEvia+HVurBQ6ex0bqYLG3M5yUvnT+PHy2IcZE8+qRQ1lghv6x67SdlH586ZzJmDJ+Yn8+mdTFZ2phPzQu5cCy0kFG4YJVl4zfOIbS82tJn5qtumwg+ktCP7yXWa11MljbJc6yitVbA7J\/RhQsXUp8+fRzNmDFjv9YaQ8AQMAQMAUOgfgigcNk2\/R\/cNlHN0z+mtmdfUXco11Zb6odlzNsKmFp0tm7dStOmTaP58+c7evzxxwm6WlPy\/1n2IdkHrb4gdHK\/E\/ZSOmkP8YiTQjJuLE7IL6te+0nZx7\/55pux4ZS0yZgx55ifz6Z1MVnamE\/NC7lwLLSQfQSbvt\/Yz2fTupgsbcznJS9gwGNGC2KcQjz6pJCMG4sT8suq135S9vGpcyZjlpMX+vhiaF1MljbmU\/NC0bJ94Q9ozcRBrnAJHcrl66FFLpKg4\/uK9VoXk6VN8hyraK0VMLUzunLlStq5cydhDxTLd23btiXoak3J\/2dZxov5+GyldNIe4lMTk3FjsUJ+WfXaT8o+PnUZWMYsJy\/08cXQupgsbcyn5hUaF\/SS+HpopR48dPj9QAsZBF7qYrK0MZ+XvDjX93+wK23dTY42vN2Bfr9+Dz26agctea21a6ct2kRTluyhcY+udTRp4Wv0uftX0ZDpy2nMnNWO7vz1WgItrt6BsEFijNAGnWoNsMs5qFW5\/7PqtZ+UfXzqnMmYbqCBHzE\/n03rYrK0MV9uXq5weeRWV7TUzLuD2vbsR3woFysvOj2+HtosNvjJ+Y3J0iZ5fZ2iyKKAKUpK5eWBR83atGlT17m6urqO\/0vP02nOss2FornLt9GC1bto9n9usrxyMLctfb7wgZ9KXED4WhQYmk66\/VliOvLapwjEMtrPfHc5gWS8B59dT4tefI3+uHEHrX71NVq77c+0b98+9xQidCD43Pnrda4vx8TfF\/yLOS+0d+9ewopFXsabdZyZ89pYXXco1z1NtPAH1HrIROp0z2pq+6Xv0SHHDHJznvW6De23bt26uq8e2LBhQ91nWdEYK2AyzOjuj15JD679oPvXk\/xjlWd+7Nxqwr8Oxz+2zvLa\/y\/jSp7Plj5fd9Z+4GtCIVAfQmEBQhGhCR8gR35gH0ka0PlQOr9vlaNJg48ippkXdiGmBSN6EOgPX+1FIPAg2L\/7mY50xyfeT\/98QYc6f+hhZ9+RH29PuA7uvY99exlNemw1bdq0qaJp8+bNtG3bNtqyZUtFj7O+OGbK63+W0caffN29u2Xb7JtpT+v\/jw4ZNZsOuf4Z2vOJURWDx6xZs+q+egBfI5DhYy6XLlbA7J82VKm7d9euDe+X+\/btu58jav\/Yl+j7p79O\/3pxa0f\/9bXetPFbp2Wil28+iX43siutueXkoH\/Mx2crpZP2LHzWXKSfjCv1mg\/5ZdVrPyn7+P+Z0K8k3nqMUpYxpV7zMT+fTetisrQxn5oXxs+xGvNe1NeQso9Pyev3Xx9IoN+M\/Qj9\/JKOtPia\/k6GTpP0WTDmZJL0yJXHuyJj2rBj6f4v9nd070V96fpPtKWvn9+bJn22H438v8fSF0\/rQX\/b5zAa\/JGu9MkTutBHjjqETvhQR\/o\/PY8i\/Mu9Q4cO1L59+zoeW9IdO3YkPE7dtWtX0nTKcUe72Lgmxov4M\/9zj82pHwAAEABJREFUBw2ZtZ5e\/nPVAbF0XynLa5aj1\/2l7OOhi+Ulx+Dj0Z\/x8tlZF\/Pz2bQuJksb82hDeR31\/r30\/nnX0zuTz6RDn5tPHQZ\/mbpNeoq6jX2Iup7xubq5RYxSucV8fDati8ls+8IXvkAPPPAA3XPPPTRmzBgq6n9WwNTO7Iknnkjt2rVz\/6LAAaqamhqCrtZU9z+2mAYNGuS+DqBXr15UVVWVibCv2qpVq6hvzMdnK6WT9ix81lykn4wr9ZoP+WXVaz8p+3hgDdLjyCrLmLE+MT+fTetisrQxj5xAsTGVsnGsmF\/Mx2fTupgsbcwjJ1BsTKVsHCvmF\/Px2UrppD0LHxsbbH06HUETz+tNr99zjiuULn7gBfru716jLNi0bt3a6yfHhWswaX1MljbmMSYQx6tvy3FK9Yv5+WxaF5OljXnkBOJxHVqzmd5a8jN6\/dufom3j+hG9vPSAN+VWdet70N91jsUxfG3Mx2fTupjMNnxG4fNqwIABdMopp9R9jhWNsQKmdkY7derkXvc\/dOhQAuE1zdDVmpL\/xw0sD2D5AsZ8fLZSOmkP8b5x1Ecn48b6hfyy6rWflH08foFj4yllkzFjvjE\/n03rYrK0MZ+aF3LhWGgh+wi20P3qs2ldTJY25vOSF7DiMaMFMU4hHn3KIRQyz910Gi1Zt4s+99PN7qBwLI68vvTLqtd+UvbxqXMmY8rxaj7m57NpXUyWNuY5r7e2rnPvbHFnWx659YA35foO5cpxcyy0Ui952PjekXrwPpvWxWRpkzxiF5GsgNk\/q4MHD6Y1a9Y4Ar9fbY0hYAiUQsDsDY5AzyOr3DbXpQO7uMO+i6t3NPg1LOB7COBJojcXP0TvzLjUrbbg3S32ptz38KlUzgqYRp4ZHA7EthTa0KVgC\/n4bKV00h7iQ2PJqpdxY31Cfln12k\/KPh5PRsTGU8omY8Z8Y34+m9bFZGljPjUv5MKx0EL2EWwp96LuL2Ufn5e8gJVv\/NCBGDPJo08KIdbFHybCGR087YSnsHzx4MfXl\/aseu0nZR+fOmcyphyv5mN+PpvWxWS21bzywrurLTd+wrV0ZA\/qeNOThO8lwhcq6jGVkjku2pAvbL75gr\/PpnUxWdokj9hFJCtgGnlWsyzjxXx8tlI6aQ\/xqWnLuLFYIb+seu0n5f183T40lmV5GTg2pphNxizXzxdD62KytDGfmhdy4VhoIfsINuCIVtuh0zati8nSxnxe8gIWPGa0IMYixKNPCnHcwR\/pSthSwioMHvXWMdkPrbRB5jHG9NpPyj4+dc5kTDkuzcf8fDati8k433LEv37zvdWWwV92hcshF99Nhx13hh5KZllf09cx5uOzaV1MljbJ+8ZRBJ0VMEWYRcvBEDAECo0AbykhSV8RA71RHAFsE2FraOOkc9xL5958\/ukDDuUe1qlXPIBZKw4BK2BSp8T6GwKGgCHQRAjg8W8UM3iRHlZkmuiyub4MChd8kWI5h3JznXgLGHxuCxh84SJ\/+SK30FXanGXZh4z5+GyldNIe4lNxknFjsUJ+WfXaT8o+vkj788gPhP3y1LwwRxwLLWQfwYbrodV26LRN62KytDGfl7yABY8ZLYixCPHok0Iyrowz\/dJ+hMO9V\/9sNaGICfll1Ws\/Kfv41DmTMWVemo\/5+WxS54qWhe99LxFWXvCm3Lev+4+6N+Xq+UvNC+OXY4Dso5iPz6Z1MVnaJO8bRyXoUseQuwJm4cKFhIIFifNTQ9xCBxt8wBsZAoaAIVBEBPCo9X2X9CMc7l1cvb2IKZaX0\/b1tP2RSbTxG+fUtrfSgYdyJ5UX03pVLAK5KmC2bt1KK1ascI86jx49+iBQoUMxA5+DjM2kyHKQKubjs5XSSXuIT4VDxo3FCvll1Ws\/Kfv4IhwwbIy8MEcyLmQfxXx8Nq2LydLGfOp8IQeOhRayj2DzHWiFr89WSiftIR6xU0jG9cU5o297+uVXTia89G7Ja63dYXbpF+qv9TFZ2phPnTOOg1aOV\/OwZ50zrLjU\/PAq96bct5Y8THhfS887fkN9Jv6c2g4Y7ELLeO\/yvR1mzKfmhYtwLLSQfQRb1rzQX\/vHZGmTPOIUkXJVwODlcuPGjSs5D1l8SgYxB0PAEDAEKhwBFDHP3XSa+z6z0GPWFZ5C2cNzRcvTP3bfS4TzLfpQ7qEd7VBu2eDmpGOuChhgumrVKsLrkbFVhDMvIPDQwQafSqIs+5AxH5+tlE7aQ3wqRjJuLFbIL6te+0nZx6fuY8uY5eSFPr4YWheTpY351LxC44JeEl8PrdSDh47PDUAGaV1Mljbm85KXzpXHjxbEuEgefWJUypY1Vqc2RAtG9KCnX3jNFTIcN9Rf62OytDGfOmccBy2P1dfCzrhKOwqX7Y\/cSiha0B5WW6h0nfQUdbpnNb32obMI\/eCPVvaXso9Pzct3Teg0yWtnsWn\/mCxtktfXKYqcqwJm165dNHnyZJo6dSotWbKE5syZ4+YB20ZXXXWVs8HHKSvkR5ZlvJiPz1ZKJ+0hPhUeGTcWK+SXVa\/9pOzjU5eBZcxy8kIfXwyti8nSxnxqXqFxQS+Jr4dW6sFDp5e+tS4mSxvzeclL58\/jRwtiXCSPPimUNRb8Th9wrPsSylde30P8mDX0PC45Dq2PydLGfOqccRy0clyah53Hj6IFB3H5EWjw+k250h+xYrK0MZ+al++a0Gni66HNYoMf4wD\/mCxtkke\/IlKuChj+tmh80SK2kwYOHEj8rdEXXnihmx\/2cYL9MAQMgQACpi4aAni8Go9ZIy8uYsDnmVC4YJWFD+Xyaku5b8rNMxY29oMRyFUBc\/DwTWMIGAKGgCEgEUARg2Imz++KQeGybfo\/uG0irLbgUG63W5+ijmN+RK1POFuma3wLRsAKmEae\/Cz7kDEfn62UTtpDfGraMm4sVsgvq177SdnHp+5jy5jl5IU+vhhaF5OljfnUvELjgl4SXw+t1IOHTp4p8Om0j5R9fF7y0rn6coEOpDFC33IoayyfH94VM+6TPWj0Q6vc2Rh5fe0fk6WN+dQ54zho5bhQtKBY4W2iNzdVE17tj7Mt+F4ieSgXfTXOWheTpY351LyQC8dCC9lHsOmxs5\/PpnUxWdokz\/GL1uaugNmwYQMNGjTIvQtm3rx5NHLkSMdDB1vRJsjyMQQMAUOgHASGndyRJg0+ii6cucq98K6cGE3RB4ULton4UG7r\/3M2uUO5Nz1J9LHPN8UQ7Bo5RSBXBQzOvSxatMi9BwYHdzXBBp\/YXLz71FIfV\/ScddZZhHfLwB8vv8PTTCD4QNcQlOUgVczHZyulk\/YQn5qbjBuLFfLLqtd+UvbxqQfxZMxy8kIfXwyti8nSxnxqXqFxQS+Jr4dW6sFDJw8T+nTaR8o+Pi956Vx9uUAH0hihbzmUNVbID\/qLzuhHeFcMXng3Z9lmNwzo5RhjsrQxnzpniHP0EYfQW0t+VvcINFZe+FAuVluwTQQ\/OU43+P0\/fDati8nSxnxqXhgax0IL2UewNWVeuJ5vHEXQ5aqAQbGBogNFho9gg09oYvCY9eOPP+6eYELxg0PA11xzDa1bt46mTZtG8+fPdwSfWByy\/wwBQ8AQyAkC8l0xY+asbtZR82qLHcpt1mlIu3gF9c5VAYPVFayyoPiYMGEC4ckj8CDIl156KcEnhG\/\/\/v1pwYIFdT7nn3++c33++edp586d1LlzZ0Jl3LZtW1q5cqWz8Y+lS5e6wgePb6Pgwf5iFtqxYwe98MILtH37dvd+Al+fmI\/PVkon7SEe49i7dy9h3xd8fUnGjfUN+WXVaz8p+3jomjMvYIEx6DnXupgsbcyjTckrNC7oJeE6euxs99m0LiZLG\/No85AXMMBYGZssPPqk5CavgVghCvlJPd4Vs2T8ybRm6y761LRlB\/xNkn64hpR9PHT1zevN558mPpT72q9n0J\/7nU9t7\/ivuu8lwnU14TqMdxab9o\/J0sY82vrmlWVc9fHBGHTOWheT2YYYixcvpscee4yqq6v5Y6xwba4KGEYfqyPPPPMMoWhhHYoZ6GBjXan2iSeeoDPPPJNatWpF3bt3pzZt2tR10ZOOFZrhw4cTaNasWbRp06ZMhMIF8XFjhfrEfHy2UjppD\/GbN2+mbdu20ZYtWzLloccu42qblEN+WfXaT8ohvjnzQu5yXJBBWheTpU3yKXn5xgCdJnm9LDbtH5OlTfJ5yAtYyDFn4Svtd+ywvdvpu585ivbt20dffXw78d8kmUvWPDPN2f8so40\/+Tpt\/NqxtGnaF6lmV407lHvoTb+jqr+bUHd9XNNHelz7fdzfK59N62KytEk+U16Rv\/0ylhyv5GM+PpvWxWS2Pfzww3T55ZfTtddeS7fcckvd51rRmFwWMFxo4BAvT8hdd93lWLY5IfKDz7ng+5MibnWmu+++m2bPnu1oxIgR1LVr11wTVps6duxIXbp0yXUeeh4sr3zdl0WdL9yXlZobHrP+cLf2NGTWenr5z1X1\/v0vlddR799L7593vfteokOfm08dBn+Zuk16irqNfYi6Drm63tcDlk1BpfJqijE01DXwGcWfV2PHjq37HCsak8sC5ogjjqB7773XvYmXz8IsW7bM6WDjScKZF3zFAHywcsJv6R0\/frxzmTJlimvxY8OGDSRfgscvyIMNhBUaPOkE6tWrV92XgFVVVeWSx4E1rAzldfyhcVte+bofczdf9fh9r+Tc8Jj1jRccS+MfW0e\/X7+nXn\/DfHkdWrPZHcp9\/dufom3j+hG9vNS9s4VfOFfVrW+9rhH6\/W5MvS+vxrxeY8bGZxQ+q0CnnnoqPsIKSbksYDATOOuySDyRBB462Jhw5mXFihXuqSVUoyhuULycf\/75JFde8Gbfdu3aue0UPJ9fU1ND0HGclBb7n4iJNhQHtpCPz1ZKJ+0hPjSWrHoZN9Yn5JdVr\/2k7ONxpic2nlI2GTPmG\/Pz2bQuJksb86l5IReOhRayj2BLuRd1fyn7+LzkBax844cOxJhJHn1SKGuskF8p\/ef6t6f7LulHeELp54tXuzN6GK\/s5+PlnPGhXH4E+rCOvdwj0Chc2p59BcIdRDLmQUahiPn5bFoXk6WNeZmXGEa9WI6FNtQRNr5ftI\/PpnUxWdokr69TFDlXBQzOt+B7kEqBH\/LBo9LYduJ3x\/DKDLadsMw2dOhQAoHXxVCpa5rdEGgCBOwShkCDIoAnlOaN7E8j522mqf\/+arbY2zcQ\/f4XtPX2cwmFi3wE2t6Umw1C82oYBHJVwKCo4C0hPsMiYYAORQl8pJ75wYMHu9UYPLXExCsz0gae+6S2WCbEk01oQ7FgC\/n4bKV00h7iQ2PJqpdxY31Cfln12k\/KPh7LwLHxlLLJmDHfmJ\/PpnUxWdqYT80LuXAstJB9BFvKvaj7S9nH5yUvYOUbP3Qgxkzy6JNCWWOF\/LLqzz7+KHruptNoybpd7osgZT\/JY5uo\/bKf0PaJ\/emdJ6dR6+z1Xp8AABAASURBVK5961Zb8O6WrLnKmLE+MT+fTetisrQxn5d7kceLFvih9d1\/Ug+\/IlKuChhMAIoLFB\/gUaxIgg42+IA3amAELJwhYAgUEgF8dxIO9yI5+UWQ2CLCCgu\/3h986yET6ZDrn3GPQOOFc+hjZAg0BwK5K2AYJJxhQbEiCTq2V0qbZR8y5uOzldJJe4hPxUfGjcUK+WXVaz8p+\/jUfWwZs5y80McXQ+tisrQxn5pXaFzQS+LroZV68NDpvXuti8nSxnxe8tL58\/jRghgXyaNPCmWNFfLLqpd+KGJQzJx701z6w\/Txboto2+ybCWdbOnz9V\/T2df9BVbUFTHPmhWvLMUMGaV1Mljbm83Iv8njR6ryha4x7EdepRMpTAVOJ+JUcU5ZlvJiPz1ZKJ+0hvuTASzjIuDHXkF9WvfaTso9PXQaWMcvJC318MbQuJksb86l5hcYFvSS+HlqpBw8dL1VDBmldTJY25vOSl86Vx48WxLhIHn1SKGuskF9WPfthiwgrLDe++BW6\/8WL6c3FP6WawROpz8xX3RNFHU45373oM3XO+HpoY\/jAzrhqP59N62KytDGfmhfGyLHQQvYRbE2ZF67nG0cRdFbAFGEWLQdDwBAwBMpEANtE8ssUsdqCL1Ok21bTOS+eR\/wdSmWGt24VgUAxB2EFTDHn1bIyBAwBQyCIAIoWrLbIsy38ZYr8JBGeUMLhXnx\/0p2\/XhuMZQZDoLkQyG0Bg0eq8eWNOMSL73sYNWoU4cV1zQVk6LpyT7IcH1\/\/UjppD\/GhsWTVy7ixPiG\/rHrtJ2Ufn7qPLWOWkxf6+GJoXUyWNuZT8wqNC3pJfD20Ug8eOt5fhwzSupgsbcznJS+dK48fLYhxkTz6pFDWWCE\/rUfR8ubz730nEVZdsNrCZ1taf2aiG67sh+9QWjCiBz39wmvuO5SQZzlz5gLv\/yHj71d5m5ifz6Z1MVnamE\/NC0lwLLSQfQQbcESr7dBpm9bFZGmTvL5OUeRcFjB4o+4111xD+PJGfAcSXlA3bNgwmjx5MsFWlMmxPAwBQ8AQSEXgra3rCMXKxm+cQ5smnUM1Kxa67yTqdM9qd7al1JNE3dodSo+O6k+HHnqo+\/qB1PFYf0OgoRDIZQHDr\/xH8cJA8Jtz2cb65m5xgCp0YIvHFvPx2UrppD3E87XLbWXcWIyQX1a99pOyj089iCdjlpMX+vhiaF1MljbmU\/MKjQt6SXw9tFIPHjp9L2tdTJY25psmL3KvsddjR04gHgtayCDw2l\/qsvCIk0LyGrE4Pj+32vLLO6nVrCvdq\/2xXYQ34+INuTiU23vYeIcJ4ur+UpY8nlD64mk96JzpL9AfNuxB17JIxowFiPn5bFoXk6WN+bzcizxetMAPLd+rIR5+RaRcFjB4cy4mA2\/VRQuq75c5oo+RIWAIGAJFQQBFCwoVea4FW0Q4kIvCBS+bO7Rjr6R0J57Xm8Z9sod7cy9efJcUzDobAokI5LKAwZbRvfu\/zBFFDL4awPdljonYNEj3LPuQMR+frZRO2kN8anIybixWyC+rXvtJ2cen7mPLmOXkhT46hk+nfaTs41Pz8o0BOk3y2lls2j8mSxvzeckLWPCY0YL4rEKIR58UknFDcVC0bF\/4A1ozcZB7Zwu2irhowRbRrk\/fQoccM+iA7jpuTJY25v\/u+CNo5oVd6MKZq6icw70cB+0BA1MC7IyxMrnvbdI27R+TpY35vNyLPF60wAUtYxHi4VdEymUBg4nA1wrgCxz5RXbgoYOtkkgu6YXGFfPx2UrppD3Eh8aSVS\/jxvqE\/LLqtZ+UfXzqMrCMWU5e6OOLoXUxWdqYT80rNC7oJfH10Eo9eOh4qRoySOtisrQxn5e8dK48frQgxkXy6JNCoViuaHnkVqpbaZl3B7Xt2c+dZ8FKCz9FFOqv9TFZ2pjHnJ3SvYqWjD+ZFlfvIDylVJ88OQ7aWD\/YGVft57NpXUyWNuaRl75OfWWOhTbUF7amzAvXC40l7\/pcFTDyySM8faQJTyXBJ++T0rDjt2iGgCGQZwRQsGBraNv0f6A1ww5xKy2QeaWFixaccWnKPPHG3umX9qNXXt9D8usHmnIMdq2WjUCuChissGClBasuEyZMIBziBQ+CjKeS4NOyp9SyNwQMgTwjIAsWXmXB1hAegca7WvhMC6+0NGeuKGJwuBftSbc\/61ZkmnM8du2WhUCjFzCNASdWWZ555hlC0cLxUcxABxvrYi3eGXPmmWfWvTtm4cKFxCs6M2bMiHWtl03uSYY6xnx8tlI6aQ\/xobFk1cu4sT4hv6x67SdlH5+6jy1jlpMX+vhiaF1MljbmU\/MKjQt6SXw9tFIPHjrea4cM0rqYLG3M5yUvnSuPHy2IcZE8+pQiFCsoTLCighWWV77S262wgMfjznta\/3\/UYdT3CassIBzE1Y89h66ZVa\/9pOzjfXOGlRgc8P3Md5eXLGJkzBg+MT+fTetisrQx78srNj6fjWOh9dmhg43vF8iSfDati8nSJnl5jSLxuSxgUp9Cwrti8M6YN954w80lip5p06bR\/PnzHT3++OMEnTPaD0PAEDAEEhHQhQqvrKBgwbtZsMLy1rZ1hG0grLD0mfsOdfvOS+59La3PuCzx6k3T\/dKBXeiXXzmZUMSUc7i3aUaZq6vYYEsgkMsCRj6FxKsm9XkK6aGHHqJjjz2Wevbs6eBZuXIl7dy5kzp37uy+rKxt27YEnTMm\/sABqtCBLQ4d8\/HZSumkPcTztcttZdxYjJBfVr32k7KPTz2IJ2OWkxf6+GJoXUyWNuZT8wqNC3pJfD20Ug8eOn0va11Mljbm85KXzh\/jP\/qIQwhfgAjqsvtP9NaSnzk64l+\/STU\/vIpe\/\/an6J3JZ9LGL7YmWahgxQXnV7CSgm0grKyAuk16ijpc\/A2CXl8PcogwFj0v8M2q135S9vGxOeOvH8D3J4UO98qYGGeIYn4+m9bFZGljPpZXaIxaz7HQahvLsPnmC3afTetisrRJHrGLSLksYDAROOvC52FwBgY8dLDFCFtHy5cvpyuuuOIAt+7duxOv7MBQXV2NxtGAtntoz\/8sIvzhKQq99eJiopeXFConzI3l9XSu5rQx5wvbMqUIKx+asH0jCaslTChEQHyYFjwIqyiuz+yb3Ztua15ZTXtbH0V7e5xErS\/\/Z+rw9V9Rx6mrCY82g9p+6XsEOmzQJbSvbRf3WDCW\/CuK9tT+3QvQ3r17CVsuvvHi6wceubIfrdm6yx3u9flUqi6WV6WO2TeudevW0ZIlSxxt2LDBfY4V8UcuCxhs7+CJI1594RY62EITha2j++67j66++mo6\/PDDQ24H6ad+eBP1\/NXX3Wu48YeqCLTt9nPpnfuH0\/bafyUWIR\/OwfJ693XxjEelt405X66gmP4PFGvxDpXXfj2DQOBBOHuyY+1\/02svPUcoRHAOBfTnw9rT633Oob0fuYD+MvR22n7u9fT2lx+kv1zxI9p8xSP0l9tW01+ufZI2fHE2\/eUffkx7h95Gr50zgd44djDtOPI4eu3tVrRp06bMhA8hrCyjjfWD3eeXVa\/9pOzj\/\/SnP9G2bdtoy5YtwVwO27udvvuZo1xh9rFvL6N\/+f3aOl8Zs5y80McXQ+tisrQxnyUvXDtGHAttyA8233zB32fTupjMtvvvv5+GDx\/u6LrrrjvoM60oilwWMFhpwYoLVl6YJkyYQDfffDPBxpOD1ZYBAwa4w7mYzP\/5n\/9xFenQoUNp0KBB9Pzzz9Nll11Gr7zyCqFKlV9D0LdvXw5D4\/7Ylf5a+4eqa+0Sb1EI\/yI8ZNRs6njTk1SUnJCH5fVUxc5nt2kvkaYOd66it697io68a5W2NZz84JvULUTfeYmOnr7WEc6cMB19+2+o97efpW63Pk3dxj7k6Ohxc+j4q79LR185hXpdNI5OuPwWOvqsi6jXJy+hj35yCPXq1cvRwIEDXYst6Y4dO1KXLl2oa9eu9aZetfE4Vqx\/yC+rXvtJ2cf3rN16z5oXnlDC1w9MWvgavfznKoeBjFlOXujji6F1MVnamK9PXhiDjzgWWp8dOthC8+qzaV1MZtuoUaNo9uzZjsaOHVv3WVY0JpcFjG8S8BTSAw88cMDh2\/79+9OKFSsIRQ4m8+Mf\/3idjOW1E044gXAe5u\/+7u+oXbt27l8UOB1eU1ND\/N1KuNaKmip67YN9CXvTRSI6ZhAddtwZltcJZ+cCg7zPV1W3vqSpda3u0I4foqquB9u0b9lyVZX73p+qJm5xpqJVq1bNcu3GzLW+eU08rzfNuKw\/XfzAC\/Toqh0Vi0d982pMjFNjo5DBP9JBp556Kj7GCkmFKWBw6BarKOXMElZtUKViZQYEHrpyYuk+2J9EUYRW21iGLeTjs5XSSXuI52uX28q4sRghv6x67SdlH499+dh4StnqYtbu\/cd8Y34+m9bFZGljPjUv5MKx0EL2EWwp96LuL2Ufn5e8gJVv\/NCBGDPJo08KZY0V8suq135S9vHlzBkf7sXB3lEPriLGK4aPvLb289m0LiZLG\/Pl5JVlXPXx4bGg5X7gJV4xWdokz7GK1uaygME5F5x34bMvaPF9SPV5kR0KlAULFhBWaTCpgwcPdis1WK0BD52RIWAIGAKGQMMggJfdPXfTabRx5z73ZZANE9WitGQEclnAoPjQZ2BQeIwePbri5hJLgaFH5niwHh82ueVW3d\/nL3VZ+LoLlMnIa8RChPyy6rWflH08loFj4yllkzFjvjE\/n03rYrK0MZ+aF3LhWGgh+wg2fb+xn8+mdTFZ2pjPS17AgMeMFsQ4hXj0SSEZNxYn5JdVr\/2k7ONT5gxFDM7FIO6gKcujL72DD2Os8\/fZtC4mSxvzKXnx+DgWWtbpFramzAvX02MoipzLAgYrMDiUi5YnArzWsc1aQ8AQMAQMgcpBAEUMXnx39c9WR4uYyhmxjaQSEfAXMJU40tox4TFoFCk4mIRDuGixfQQCX+tywLtcIDc3ZdmHjPn4bKV00h7iU3GRcWOxQn5Z9dpPyj4+dR9bxiwnL\/TxxdC6mCxtzKfmFRoX9JL4emilHjx0ci\/ep9M+UvbxeclL5+rLBTqQxgh9y6GssUJ+WfXaT8o+PnXOOObYs7rSfZf0c2\/uxYvvNEbshzaLDX4S+5gsbcyn5oUxciy0kH0Emxyn9PHZtC4mS5vk5TWKxOeqgMEbePE0ERcvaLF1xAQbfIo0QZaLIWAIGAJFRUAe7sUB3yLkaTk0HQK5KmDkNhGeOMKqC1ZfmHCwFz5NB1\/pK2H\/MbTfyb1jPj5bKZ20h3i+drmtjBuLEfLLqtd+UvbxqfvYMmY5eaGPL4bWxWRpYz41r9C4oJfE10Mr9eCh0\/ey1sVkaWM+L3np\/Hn8aEGMi+TRJ4Wyxgr5ZdVrPyn7+NQ5kzGBD87F4HDvK6\/voSHTl0PlSPs55f4fPpvWxWRpYz41LwyNY6GF7CPY+H7Rdp9N62KytEkJbR5CAAAQAElEQVReX6cocq4KGBzexSrLRz7yEfId4oUOPkWZHMvDEDAEDIH6IZBPbxQxOBeD0Z90+7OEYga8kSEQQyBXBYxMZMaMGe4Nu7z6grYSV2Cy7EPGfHy2UjppD\/ESy3J4GTfWP+SXVa\/9pOzjU\/exZcxy8kIfXwyti8nSxnxqXqFxQS+Jr4dW6sFDp\/futS4mSxvzeclL58\/jRwtiXCSPPimUNVbIL6te+0nZx6fOmYyp8UERg8O9n\/nuclr435uC74vxxdC6mCxtzKfmhVw4FlrIPoKN7xdt99m0LiZLm+T1dYoi57KAwTbRnDlzaObMmXXvbsE5GFuBKcptaXnkFQEbtyGQigDe3IvDvXhz74LVu1LDWf8CI5DLAgbz0b179wNe9w9dJVKWfciYj89WSiftIT4VKxk3Fivkl1Wv\/aTs41P3sWXMcvJCH18MrYvJ0sZ8al6hcUEvia+HVurBQ6f37rUuJksb83nJS+fP40cLYlwkjz4plDVWyC+rXvtJ2cenzpmMGcKHD\/fiO5TGPbr2IDdfDK2LydLGfGpeGCTHQgvZR7Dx\/aLtPpvWxWRpk7y+TlHkXBYwOOdy5pln0rx584oyD5ZHgyBgQQwBQ6AoCOBcjO9wb1HyszzSEchlAcNbSHfdddcB52DsDMxawr4niPdYQ3zqrSPjxmKF\/LLqtZ+UfXzqPraMWU5e6OOLoXUxWdqYT80rNC7oJfH10Eo9eOj4voIM0rqYLG3M5yUvnSuPHy2IcZE8+qRQ1lghv6x67SdlH586ZzJmDB\/4vf3GJpr7pX7OTR7uhY0xd8baH1oXk6WN+dS8aofg\/v7qcUEvia+HVurBQ6f7a11MljbJI3YRKZcFDFZgcN4F514kQQdbc02UXdcQMAQMAUOg4RGQh3sXV+9o+AtYxFwikMsCBiswWG3Bk0eaBgwYQKtWraqYyciyDxnz8dlK6aQ9xKcCJOPGYoX8suq1n5R9fOo+toxZTl7o44uhdTFZ2phPzSs0Lugl8fXQSj146PTevdbFZGljPi956fx5\/GhBjIvk0SeFssYK+WXVaz8p+\/jUOZMxY\/hoPz7ciyeUHl21gxhzjqH9Y7K0MZ+aF8bBsdBC9hFseuzs57NpXUyWNslz\/ApuyxpaLgsYrLJceumldOGFF9Y9hTRhwgQCTZ06lSZPnlwWGNbJEDAEDAFDoHIR4MO9eGsvqHJHaiNrCgRyWcBgBeaZZ55xBQuDhGIGui5durDK2\/L3KWHlRq7WLFy4sO48Dd4x4+1chjLLPmTMx2crpZP2EF9GKgd0kXEPMCgh5JdVr\/2k7ONT97FlTJXKAWLMz2fTupgsbcyn5oXBcyy0kH0Em96DZz+fTetisrQxn5e8gAGPGS2IcQrx6JNCMm4sTsgvq177SdnHp86ZjFlOXjjcu2T8yfTHjTsOeHOvjhuTpc3xa9dSal7IhWOhhewj2Pje0XafTetisrRJXl+nKHIuC5g2bdo4\/HGI1zG1P\/iJpMMPP5y6detWq\/H\/P2nSJMITTDg7g9WaWbNmEQqiadOm0fz58x09\/vjjTuePYFpDwBAwBAyB5kQARczMC7vQvn37SB7ubc4x2bWbHoFcFjD4wsZ7772Xli1bVrdqghfb3XHHHXTjjTdS3759vUiiUNm4caPbeoLD4MGDacqUKbRy5UrauXMnde7c2e2rtm3b1ungk0pZ9iFjPj5bKZ20h\/imyAvXkNeHzJRVr\/2k7ONT97FlTB6rr435+WxaF5OljfnUvJADx0IL2UewNeX+fF7yAlYSmyw8+mQkr5u8htdhvzLkl1Wv\/aTs41PnTMbcn4K3ifmx7VdjBxK\/uff36\/e4v9+wISBaeS9L2cen5uW7JnSa5LWz2LR\/TJY2yevrFEV+X14TwTmYRYsW1Z2BAY+bFd+VNHr06Gha11xzjSt85BYSXozHKzvoXF1djaaOli5dSkuWLHG0bt0697gclujyTHv37nXLpnnOwTd2y2tPru7Pos4X7s2i5lZJeY09qytN+WwvwuHeB599Nener6S8cP+US\/iM4s8rfPFx3QdZwZjcFjDjx493RQjOsjDhySSssoTmaMuWLfT888\/TlVde6Qqfq666yh34xU0b6sN6bDENHz6cQNh22rRpE2Uh3EhYKUIb8oct5OOzldJJe4jfvHkzbdu2jYBJaFwxvYxbjl+ov9bHZGlj\/k9\/+lOz5gUseCxoIYPAyzmOydLGfGpevjFA50jcy3w9tFls8GsJeQELmWsW3n7HNnn\/RkrsgGuIYn7adszhe+jnl3SksXOradSDq9x1tY+UfXyRfsfuv\/9+91mFz6vrrruOP8YK1+aygEGR8uKLL7rzKnjyCN+JhBZPJmFlhmcJj1NjlQUFDiYSW0M9e\/Z0W0XwwVYTqlMUMGh3794NtSPYHLP\/x913301Y3QGNGDGCunbtmomwLdWxY0fC4eJQn5iPz1ZKJ+1Z+NC4YnoZtxy\/UH+tj8nSxjy3MbzLGa\/uE7uOz6Z1MVnamOe23Lww\/iwxYj4+m9bFZGljnttKz0vjJ8edhUf\/+pKMG+sb8suq135S9vFSFxtXyJa1f8zPZzv52O702GVdaGvtn\/Gv\/PI193de\/u2VfXy81IXGXkqfJUbMx2fTupjMNnxGPfDAA3TPPffQmDFj9n+KFa\/JZQGDaUAxgslCofHEE0+4cy14CgnFDeyg\/v3704oVK9xqCwqPXr160XHHHUe\/\/e1vYabq2m0ibB199KMfpXbt2rnVCJwOr6mpOeh7luA3aNAgAiFOVVUVZaEOHTrQ8ccfT+3btw\/6x3x8tlI6aQ\/x2O9t1apVcEylcpNxY74hP6E\/YAxaH5OlTfLNmRewkGOBDNK6mCxtkk\/JyzcG6DTJ62Wxaf+YLG2Sz0NewEKOOQtvv2P+v5ESO+AaopifzwbdmR\/tRzgXc+ihh9I501+gNp161f3thZ3\/Fof4PNyLcuzATsrMI8+zzz6bPvvZz9Lpp5\/uPu+K+COXBQzOqvCTRihinnzySVdYYBWl1CRhpQYHfrEqg\/bee+8lFCRjx46loUOHOgIvV3JKxTS7IWAIGAKGQOUgYG\/urZy5aMyR5LKAwVNIF110kVsxwSoLHofGisx9991HpQoP2HHgF49Ro+3UqZPDF08kQQcC75QN8AOHsLCqgzYUDraQj89WSiftIT40lqx6GTfWJ+SXVa\/9pOzjU9\/lIGOWkxf6+GJoXUyWNuZT8wqNC3pJfD20Ug8eOn2fal1Mljbm85KXzp\/HjxbEuEgefVIoa6yQX1a99pOyj0+dMxkzhk\/Mz2fTOhzuvens9u5w75xlm93hXt88cb\/UvJALx0IL2Uew8Ti03WfTupgsbZLX1ymKnMsCBuBjKwfFC3gUHNgqYhk6I0PAEDAEDIGWjcAp3asIL73DW3vHzn2pZYPRCNk3d8jcFjDNDVzW62OPEo93ow31gS3k47OV0kl7iA+NJatexo31Cfll1Ws\/Kft4nDuIjaeUTcaM+cb8fDati8nSxnxqXsiFY6GF7CPYUu5F3V\/KPj4veQEr3\/ihAzFmkkefFMoaK+SXVa\/9pOzjU+dMxozhE\/Pz2bSO5Q93a0\/P3XQabdy5j772rzvqztzpOUvNC7nwNdFC9hFsfG1t99m0LiZLm+T1dYoi56qAwQFdPCqN8ys+gg0+RZkcy8MQMAQMAUMghEB2Pd7cu2DMya6DvbnXwVCIH7kqYDTieHwaZ1aY5JkW7dtccpZ9yJiPz1ZKJ+0hPhUPGTcWK+SXVa\/9pOzjU\/exZcxy8kIfXwyti8nSxnxqXqFxQS+Jr4dW6sFDp\/futS4mSxvzeclL58\/jRwtiXCSPPimUNVbIL6te+0nZx6fOmYwZwyfm57NpnU\/+zqfb07CTj3LnYn6+eLU7F8N+qXkhF46FFrKPYOP7Rdt9Nq2LydImeX2dosi5KmDkAVy8ZfC2226re5mdrb4U5Za0PAyBfCBgo8wnAuM+eTTde9GxNHLeZpq7fFs+k7BROwRyVcC4Ee\/\/IYsZrMDcfPPNNGzYsIr7EsYs+5AxH5+tlE7aQ\/x+GMtuZNxYkJBfVr32k7KPT93HljHLyQt9fDG0LiZLG\/OpeYXGBb0kvh5aqQcPnd6717qYLG3M5yUvnT+PHy2IcZE8+qRQ1lghv6x67SdlH586ZzJmDJ+Yn8+mdTH57OOPcudi8ObecY+udd+hlJoXctHXhE5TzMdn07qYLG2S12MoipzbAka+ZRfnYfDWQXyLNAqbokyO5WEIhBEwiyFgCKQggHMxONz7yut7aMj05SmhrG8zIZCrAgYHdLFVhILl6quvJrzADqsvILxpF++HaSYcg5fNsg8Z8\/HZSumkPcQHB5zRIOPGuoT8suq1n5R9fOo+toxZTl7o44uhdTFZ2phPzSs0Lugl8fXQSj146PTevdbFZGljPi956fx5\/GhBjIvk0SeFssYK+WXVaz8p+\/jUOZMxY\/jE\/Hw2rYvJbOvUhmjul\/q5szCDpix3TyrFxlTKxnHRhnxh4\/tF+\/hsWheTpU3y+jpFkXNVwEjQX331Vff2XRQzTChuUORIP+MbBwGLaggYAoZAURCYeWEX+lz\/9jTq0c20ZN2uoqRV+DxyVcBgewhPGmHFxUewwaeSZi3LPmTMx2crpZP2EJ+KkYwbixXyy6rXflL28an72DJmOXmhjy+G1sVkaWM+Na\/QuKCXxNdDK\/XgoeOzHpBBWheTpY35vOSlc+XxowUxLpJHnxTKGivkl1Wv\/aTs41PnTMaM4RPz89m0LiZLG\/MTz+tNkwYfRRfOXEV4c29sbCEbx0Jbjg\/68b3E\/bUuJkub5DlW0dpcFTDvgW+cIWAIGAKGgCHQsAjIN\/fi7b0NG92iNTQCVsA0NKIqXpZ9yJiPz1ZKJ+0hXg2z3qKMG+sc8suq135S9vFF2p9HfiDsl6fmhTniWGgh+wg2XA+ttkOnbVoXk6WN+bzkBSx4zGhBjEWIR58UknFjcUJ+WfXaT8o+PnXOZMxy8kIfXwyti8nSxjznlXK4l2OhxTh9BBvfO9rus2ldTJY2yR9wnQIJVsAUaDItFUPAEDAEDIF0BFDE2Jt703Fs7AhWwDQywln2IWM+PlspnbSH+NS0ZdxYrJBfVr32k7KPL+r+fGpemCOJF2QfxXx8Nq2LydLGfF7yAlY8ZrQgPqsQ4tEnhWTcWJyQX1a99pOyj0+cM\/ddRIxdOXmhjxwXZJDWxWRpY96XF4qYSwd2cW\/uXVy9A5eJEsdCG3KELZS\/z6Z1MVnaJB8aS971La6Ake+PkU8tLVy4sO6tvjNmzMj7vNr4DQFDwBAwBBoAARzuve+Sfq6IKfdwbwMMw0J4EGhRBcyuXbto8uTJdNVVVxGeYho4cCDddddd7u2906ZNo\/nz5zvCC\/Ea6nHsLPuQMR+frZRO2kO8516ol0rGjXUM+WXVaz8p+3jex46NKWaTMcv188XQupgsbcyn5oVcjLZKQgAAEABJREFUOBZayD6CrSn355s8rz17Dkrbl3MpnbSH+IMuVE+FjBvrGvLLqtd+UvbxqXMmY5aTF\/r4YmhdTJY25mN5ndG3vXtzLw72gjAGH3EstD47dLA15e8YrofrFpFaVAGDF91169btgHns27cvrVy5knbu3EmdO3d2r5Ru27at00nHpUuXEr5\/CbRu3Tr34iPcGFlo7969hF+OmG\/Mx2crpZP2LHxsbCGbjBvygT7kl1Wv\/aSsecigUnhjXCHK2j\/m57NpXUyWNvBMKXkhX8QpFSPm47NpXUyWNvBMpcaEsccIcUrFiPn4bKV00p6Fj40\/ZJNxQz7Qh\/yy6rWflDUPGVQKb4wrRFn7x\/x8Nq2LydIGnimUF156t2T8ybRm6y735t6U3HCt0HV8Nq2LybDhM+r3v\/89rVixgtavXy8\/ygrFt6gCBjM3ZcoUqq6udttF559\/Po0ePRpq6t69O7Vp08bx+AEftExYoRk+fDiBZs2aRZs2bcpE27dvp1atWtGOHTuC\/jEfn62UTtpD\/ObNm2nbtm20ZcuW4LhiOcq45fiF+mt9TJY2yTdnXsBCjgUyaL+u7j6IydIm+ZS8fGOATpO8Xhab9o\/J0ib5POQFLOSYs\/D2O+b\/GymxA64hivn5bFoXk6VN8qXuxcP2bqfvfuYo9w\/Yj317Gf3hxVcP+PspYzVnXg8\/\/DBdfvnldO2119Itt9zCH2OFa1tUAYNtIZx7QeGCLaQnnniCxo8fn2lS7777bsLXFYBGjBhBXbt2zTVhtaljx47UpUuXXOeh58Hyytd9WdT5wn1Z1Nwsr66Ew71fPK2He3Pvy3+uqri\/ofiMwmcVaOzYsZk+4\/LoVOgCRh7YxcrJutqtn3bt2tGJJ57o5gqFzLJly9z20YYNG2j37t1Ojx\/YWkLLhBWaQYMGua8v6NWrlztJX1VVVbJFf1TiaEP+sIV8fLZSOmk\/gK8V+Do4cY+VodCYSulrQ7l\/eaCN+cLO15R+WfXaT8ohvjnzQo5yXJBBWheTpU3yKXn5xgCdJnm9LDbtH5OlTfJ5yAtYyDFn4e13zP\/3UWIHXEMU8\/PZtC4mS5vk63Mv4nDvjMv608UPvECPrtrhPgtkrObMC\/8wPemkk9yxiAEDBmBYhaRCFzD9+\/d3e4BYbUEl2qu28MBZF5x5wWximwiFyUc\/+lFCYYPtFByuqqmpqSty4GdkCBgChoAhYAhoBLIe7tX9TG4YBJqygGmYESdEwfck3XfffTRu3Dh3BmbOnDl07733EgobLLMNHTqUQODhm3Cpuq6owkPP\/LNTzMdnK6WT9hDP1y63lXFjMUJ+WfXaT8o+Hv\/qjY2nlE3GjPnG\/Hw2rYvJ0sZ8al7IhWOhhewj2EL3q8+mdTFZ2pjPS17AiseMFsQ4hXj0SSEZNxYn5JdVr\/2k7ONT50zGLCcv9PHF0LqYLG3Ml5sXXnr33E2n0Suv76FhP1ztHgRBTIzTR7DxvaPtPpvWxWRpk7y+TlHkFlXAYNLkqsyiRYuIC5XBgwe7R6uxWgMevkaGgCFgCBgChkApBFDE4FwM\/E66\/VlXzIBvOLJIPgRaXAHjA6ExdXjUDttSaEPXgS3k47OV0kl7iA+NJatexo31Cfll1Ws\/Kft4PJoYG08pm4wZ8435+WxaF5OljfnUvJALx0IL2UewpdyLur+UfXxe8gJWvvFDB2LMJI8+KZQ1Vsgvq177SdnHp86ZjBnDJ+bns2ldTJY25lPzQi5zv9SPzu9b5V56tzjw5l6+Hlr0kQQd30us17qYLG2S51hFa62AKdqMZswHj3j+5Cc\/IbQZu+TCDeeYLK9cTJUbZFHnC8nhd6so9yLyYSrqnDVUXqNObU9TPtvLFTH25l6+axqntQKmcXCti5plHzLm47OV0kl7iMcfV7zPBm3dYOvByLixbiG\/rHrtJ2Ufj\/ftNGdewEKOCzJI62KytDGfmpdvDNBp4uuhzWKDn9zPj8nSxnxe8gIWPGa0IM47xON3qznvRTkujJ9J62OytDGfOmccBy2PydfCzhhru8+mdTFZ2phPzQtj5FiDP9I1+OZe9kGLPpKg0zlrXUyWNsnLaxSJtwKmkWcTb0HES\/DQhi4FW8jHZyulk\/YQHxpLVr2MG+sT8suq135S9vF4HD42nlI2GTPmG\/Pz2bQuJksb86l5IReOhRayj2BLuRd1fyn7+Gx5+Ub6nk7GfU97IBfz8dlK6aQ9xB84gvpLMm6sd8gvq177SdnHp86ZjFlOXujji6F1MVnamE\/NS48L52L4cO+Q6cthdsTXQ+sU4gd0+vdP62KytDHfEHmJIVYUawVMxumQXyWArxPISuiHG\/IXv\/hF3VcR6L4xH5+tlE7aQzzf1LDr8WSR0a9UXogT8suq135S9vG4JqYUNvD1JfRLyQvX88XQupgsbcwjbkpe6M+xGvNe1NeQso\/HuPKQF8bpGz+wDOntd2yJ92+exAu4hijm57NpXUyWNuYxjoa+Fzf+8Tm68ZS9tGHDesLh3t+trCa+Hu4dXFOSz6Z1MVnamEd85FVEsgKmxKz26NGDTj31VMKHGl6GV1+67rrr3BVi\/WM+PlspnbRn4eubE\/xlXMghCvll1Ws\/Kft4\/FEA4DG8Q2OFXsaEDPJRzM9n07qYLG3Mp+aFHDhWDJuYj8+mdTFZ2pjPS14aPx4\/sMzCo399ScaN9Q35ZdVrPyn7+NQ5kzHLyQt9fDG0LiZLG\/OpeYXGBf2On42lrb972J2L4evh3oFNks+mdTFZ2phHXvgMw2cZ\/jYWiayAKTGbmHT5NQJ4IZ7R7LqvVTAsDAu7B+wesHug9D0w75ZL6F+uOLpZ\/nbiMwyfZSU+7nJnfl\/uRtwMA8bED9r\/NQJN3w5yX19g1zUc7B6we8DuAbsHyrkH8BnWDB+djX5JK2AaHWK7gCFgCBgChoAhYAg0NAIlC5iGvqDFMwQMAUPAEDAEDAFDIBUBK2BSEbT+hoAhYAgYAobAwQiYppERsAKmkQG28IaAIWAIGAKGgCHQ8AhYAdPwmFpEQ8AQMASaHwEbgSFQcASsgCn4BFt6hoAhYAgYAoZAERGwAqaIs2o5GQLNj4CNwBAwBAyBRkXACphGhdeCGwKGgCFgCBgChkBjIGAFTGOgajGbHwEbgSFgCBgChkChEbACptDTa8kZAoaAIWAIGALFRMAKmMaZV4tqCBgChoAhYAgYAo2IgBUwjQiuhTYEDAFDwBAwBAyB+iCQ3dcKmOxYmachYAgYAoaAIWAIVAgCVsBUyETYMAwBQ8AQMASaHwEbQX4QsAImP3NlIzUEDAFDwBAwBAyB\/QhYAbMfCGsMAUPAEGh+BGwEhoAhkBUBK2CyImV+hoAhYAgYAoaAIVAxCFgBUzFTYQMxBJofARuBIWAIGAJ5QaDFFTALFy6kPn36OBo+fDjt2rXLzZXUz5gxw+nshyFgCBgChoAhYAhUJgItqoDZunUr3XbbbTRz5kxas2YNnXnmmTRp0iSCftq0aTR\/\/nxHjz\/+uNNV5pQVeVSWmyFgCBgChoAhkA2BFlXAbNmyhdq1a0cnnniiQ6dv3760ceNGWrZsGe3cuZM6d+5MvXv3prZt29LKlSudj\/0wBAwBQ8AQMAQMgcpDoEUVMChQUKhwcVJdXU0bNmygvXv3Uvfu3alNmzZ1MwQbC+vXr6clS5YYGQZ2D9g9YPdATu+BdcsXU4jefP5p8lHN0z+mEG1\/5FZqCNo2\/R+oKYg\/z4rUtqgCplOnTnTzzTfTyJEj3RmY5cuXuxWZ2ISieLnuuusI52WMhhsOww0D+z2we6Ch74H\/d8XFxHTnqAvpx1\/9TB09Me5TBFp+w7nEtGnSOSTpr5M\/QaBOU0+jEP31W7U+AZKxJB8rLLbPnUSrf3p7Mr30bw+RpMULfkaNQfgsi33WJdqapfv7muWqzXjRwYMHu\/MvOAMzbNgwt13UqlUrtxKze\/fuupFhewkCJn3p0qV099130+zZs+tNY8eORZho\/5iPz1ZKJ+1Z+MbKC3Hl9SEzZdVrPyn7+M9\/\/vMl8eYx+FoZ02dnXczPZ9O6mCxtzKfmhXFzrNi9HPPx2bQuJksb83nJS+PH4weWWXj0ry\/JuLG+Ib+seu0nZR+fdc5+Mm0ygWaOPI9A0z93HE07o5WjqR\/eRA\/1f9XRwlPW0k9reSbYJvTaRkxXDmhPoAuOP4qGnvkxRz169KDNfzmUzhhyiaN+f38TtT37CvrJxva0d+jt1HHMj2jNKaPprnUdD5DH\/bEr\/fXLD9Irn\/o2af7R466hv191NG28\/Gf0vht+U0dbxz1LIXrf9bV+gpb93yl0WW2MTYgh9NIv5gObG9c\/PkRdJz3lqG6s+3UxWdqY\/13\/q12u+CxzfxwL9KNFFTA4rDtkyBBatWqVe\/rogQcecAd5Bw4c6FZicEZm7dq1VFNTU3dOhucaW0yDBg2i+tLnaz9Q8Ufg1FNPDfaN+fhspXTSHuIxHuTWmHkBK3l9yExZ9dpPyj4e8ZszL1xfjgsySOtisrQxjxgpeaE\/x8LcQ\/ZRzMdn07qYLG3MYwx5yAvj5DEDvyw8\/FJyk9fA9UMU8suq135S9vEYB\/Lq\/IF99LFjutAJe15wdMwf7qeeT1xP3X5yqVsF6faTS2r5S6jPH2Y4OuHNF2jAEXvolD5dXNFx7N9eRig8UGyA+AO753fXEugD\/\/QqLTjjXldIQAZ1q\/1QB8H3L7WFyt6htxH6drj4G9RhWC1dPKm2kBlRS1fQ8ZfdSP1qqe3Z78lnXzmRqk44m\/r+7d+T5nudfAZtqS2KDu34IQLPhHyzksQr1Cfm47NpXUyWNuYxDsxXEalFFTDYQkIxMXToUFegdOvWjUaPHk1SDxt8oGuICce\/FBAPbSgebCEfn62UTtpDfGgsWfUybqxPyC+rXvtJ2cejIIuNp5RNxoz5xvx8Nq2LydLGfGpeyIVjoYXsI9hS7kXdX8o+Pi95ASvf+KEDMWaSR58Uyhor5JdVr\/2kDH7M8KHUpdU++mD1Qhp+6HN02qr7aGHtqgkKlFe+0tud38BZkLe2raPDOvai1rUFAooKFBkgFB595r7jihLwKEBgB3WoLTywcgJCP9ChtTFAuDbjqnH02bQuJksb83m5F3m8aIELWsbJxzdEXrhOJVKLKmAwAXILacqUKVA5knrwTtkAP\/bs2UNY1UEbCgdbyMdnK6WT9hDfpUsXGjFiBKENjSuml3HL8Qv11\/qYLG3Mt2\/fvlnzAhY8FrSQQeDlHMdkaWM+NS\/fGKDTxNdDm8UGv5aQF7CQuSq+7ndc6vG7lbffsX21RciejdW0du4U2jTtMto46RxCkQLaNvtmqlmxkA7p0J3oY5+nDqO+77Y4uDiRhYkuSIAfk8SIdb425uezaV1Mljbmi\/o7hvx8+BZB1+IKmCJMWkPkgD+ul19+edkFTEOMoTFi4Ekzy6sxkG2cmEWdL6CVh98xFCz0+nqi38k8eI8AABAASURBVP+Ctt5+ritWNo49lt55chq9s32DK1YOufhu6njTk9TtOy\/RIdc\/Q1WX\/zNB1\/qMy9xqC3ItAhX5XizC\/PhysALGh0oD6qqqqty7ZdCGwsKG98+g1T7QaVspnbSHeH2d+soybqxvyC+rXvtJ2ce3bt06NpySNhkz5hzz89m0LiZLG\/PRvGIDFTaOhVaoD2Bh0\/cbO\/hsWheTpY35vOQFDHjMaEGMU4hHnxSScWNxQn4h\/aE1m+moPy2imh9eVVewvP+p6dS6a193ngTbPn1mvkpYUek69iHqPWw8tR0wmDhe6pxxHLTl5IU+6Mv4QwZpXUyWNuZT8\/KNATpNfD20WWzwk7nGZGmTvL5OUWQrYIoyk5aHIWAIGAIBBLDSgnMqG\/dvCYHHmRWcQ0HBgrMp4HnrJxDG1IZARSFgBUwjTwf2H+UZAd\/lYj4+WymdtId43zjqoxNxo91Cfln12k\/KPv7NN9+MjqeUUcaM+cb8fDati8nSxnxqXsiFY6GF7CPYQverz6Z1MVnamM9LXsCKx4wWxDiFePRJIRk3Fifkh7Msa2ZeQ6\/c9Am30oIXsuGQbYev\/4revu4\/6MjaFgXLIccMqjvDg+vIeD4+dc5kTFwvRDE\/n03rYrK0MZ+aF\/LgWGgh+wg2vne03WfTupgsbZLX1ymKbAVMI89klmW8mI\/PVkon7SE+NW0ZNxYr5JdVr\/2k7ONTl4FlzHLyQh9fDK2LydLGfGpeoXFBL4mvh1bqwUMnl7J9Ou0jZR+fl7x0rr5coANpjNC3HMoaS\/phpQWFClZacJbl0Ofmu60hucrS4ZTzD9jWlv0xTin7+NQ5kzFxvRDF\/Hw2rYvJ0sZ8al7Ig2Ohhewj2EL3iM+mdTFZ2iTvG0cRdPkvYIowC5aDIWAIGAIJCKBwwbYQnhZCi5UWbAnx1hAeT04Ib10NgYpEwAqYipwWG5QhYAgYAqUR4NUWFC7gOwyb5N63guIF20OlI5hHS0Yg77lbAdPIM5hlHzLm47OV0kl7iE9NW8aNxQr5ZdVrPyn7+NR9bBmznLzQxxdD62KytDGfmldoXNBL4uuhlXrw0Om9e62LydLGfF7y0vnz+NGCGBfJo08KhWJhtQXFCraJsNqC97IcMmo2dbpnNeHFcHzNUH+tj8nSxnzqnHEctDxWXws746rtPpvWxWRpYz41L4yRY6GF7CPYmjIvXM83jiLorIApwixaDoaAIVB4BFC4oGDZ+I1zCG3r\/3M2dbv1Keow6gdExwzKYf42ZEMgDQErYNLwK9k7y0GqmI\/PVkon7SG+5MBLOMi4MdeQX1a99pOyj089iCdjlpMX+vhiaF1MljbmU\/MKjQt6SXw9tFIPHjp9+FDrYrK0MZ+XvHT+PH60IMZF8uiTQhwL721BwcLbRNgawtkWrLbglfvsh1ZeDzKPK6bXflL28alzJmPKcWk+5uezaV1MljbmU\/PC+DkWWsg+gs03L\/D12bQuJkub5BG7iGQFTBFn1XIyBAyBkghUugOvuHDhwudbULhU+thtfIZAUyBgBUwjo4z9x9B+J1865uOzldJJe4jna5fbyrixGCG\/rHrtJ2Ufn7qPLWOWkxf6+GJoXUyWNuZT8wqNC3pJfD20Ug8eOn0va11Mljbm85KXzp\/HjxbEuEgefcohWbhsX\/gDKlW4hK6ZVa\/9pOzjU+dMxozhE\/Pz2bQuJksb86l5IReOhRayj2Dj+0XbfTati8nSJnl9naLIVsA08kxmWcaL+fhspXTSHuJT05ZxY7FCfln12k\/KPj51GVjGLCcv9PHF0LqqqqrguzikL\/OpeYXGBb0kvh5aqQcPnV761rqYLG3M5yUvnT+PHy2IcZE8+tSHZOGCQ7ooXPBK\/1IrLqFrZtVrPyn7+NQ5kzFj+MT8fDati8nSxnxqXsiFY6GF7CPY+H7Rdp9N62KytEleX6coshUwRZlJy8MQMARyiYCvcOEzLrlMyAZtCDQRAlbANBHQlXYZG48hYAg0LwJWuDQv\/nb1\/CNgBUwjz2GWfciYj89WSiftIT41bRk3Fivkl1Wv\/aTs41P3sWXMcvJCH18MrYvJ0sZ8al6hcUEvia+HVurBQ6f37rUuJksb83nJS+fP40cLYlwkjz4+ylq4ZImF+CG\/rHrtJ2UfnzpnMibGH6KYn8+mdTFZ2phPzQt5cCy0kH0EG98v2u6zaV1MljbJ6+sURW6mAqYo8FkehoAhYAhkQyBr4ZItmnkZAoaAFTCNfA9kOUgV8\/HZSumkPcSnpi3jxmKF\/LLqtZ+UfXzqQTwZs5y80McXQ+tisrQxn5pXaFzQS+LroZV68NDpw4daF5Oljfm85KXz5\/GjBTEukkcf0Ftb17kXz9X3cWhfLMTTFPLLqtd+UvbxqXMmY+pcpBzz89m0LiZLG\/OpeWHsHAstZB\/BxveLtvtsWldVVb8HANBfX6coshUwRZlJy8MQMAQqC4HX19M7T06jbeP6ET9VZIdzK2uKbDT5RsAKmEaevyz7kDEfn62UTtpDfGraMm4sVsgvq177SdnHp+5jy5jl5IU+vhhaF5OljfnUvELjgl4SXw+t1IOHTu\/da11Mljbm85KXzp\/HjxbEuDj+v35Dm6ZdRhvHHkv0+19Qh1Hfd1+wWOpxaFxDkou1di2hlXrNw87Xl7aseu0nZQ9PqXMmY8rxaj7m57NpXUyWNuZT88L4ORZayD6CzTdf8PXZtC4mS5vkEbuIZAVMI88qlu9Cy4V86ZiPz1ZKJ+0hnq9dbivjxmKE\/LLqtZ+UfXzqMrCMWU5e6OOLoXUxWdqYT80rNC7oJfH10Eo9eOj0vax1MVnamM9LXjp\/Hj9a0NFHHEJvLfkZvf7tT9E7k8+kQw89jDp8\/Vd0yPXPUOszLkP3ehPiarx9QUJ+WfXaT8o+PnXOZExfPqyL+flsWheTpY351Lwwbo6FFrKPYAvNq8+mdTFZ2iTvG0cRdFbAFGEWLQdDoKUgUGF54mDutun\/QDjfgu8rav1\/znarLR3H\/Ihan3B2hY3WhmMIFAsBK2CKNZ+WjSFgCDQyAihaUKxsnHSOK1zefP7pA173jy9YbOQhWHhDwBCoRaDiC5gZM2ZQnz59DiDoaseei\/+z7EPGfHy2UjppD\/Gp4Mm4sVghv6x67SdlH5+6jy1jevKqU8X8fDati8nSxnxqXhg4x0IL2UewNeX+fF7yckXLwh\/QmomDXNGC7yja020AdbzpSep0z2raMfByd1Ylhp8P75gua6yQX1a99pOyj0+dMxmz3Px9MbQuJksb86l5IReOhRayj2Bryt8xXM83jiLoKraAWbhwoStaAPKaNWtIEnQoauAD3sgQMAQMgQZHYPt62v7IJOKVlu2P3Ep0ZA9XtHT7zkt0yLlj6bDjzmjwy1pAQ8AQyIZARRYwW7dupRUrVriiZfTo0QdlAh0KGvgcZKwwRZaDVDGfA2z7cyulk\/YQvz9U2Y2MGwsS8suq135S9vGpB\/FkzHLyQh9fDK2LydLGfGpeoXFBL4mvh1bqwUOnDx9qXUyWNuYrKa9Daza7x51xpmXjF1u7w7hvLXmYDuvYi7pOeorwxYp9Jv6c2g4YTDx+tCDGRfLALIWyxgr5ZdVrPyn7+NQ5kzFj+MT8fDati8nSxnxqXsiFY6GF7CPY+H7Rdp9N62KytEleX6cockUWMJ06daJx48aVxDiLT8kg5mAIGAItDgFsCYFqnv4xoWBZM+yQd7eGaldZ3tq2zp1pQdGC97bYgdwWd3tYwjlBoD4FTJOnhJWYs846y20lYctI0oABA2jVqlVNPqb6XhD7j6H9To4V8\/HZSumkPcTztcttZdxYjJBfVr32k7KPT93HljHLyQt9fDG0LiZLG\/OpeYXGBb0kvh5aqQcPnb6XtS4mSxvzTZGXK1RWLKS1c6e8+36W\/Ydv+cmhNzdVE9VuB+E8CwqWbrWrLa0\/M5E2t\/mQO9eC3EE8ZrQgxiLEo08KybixOCG\/rHrtJ2UfnzpnMmY5eaGPL4bWxWRpYz41r9C4oJfE10Mr9eCh4\/sKMkjrYrK0SR5xikgVXcBgJebSSy+lCy+80G0nYdtowoQJBJo6dSpNnjy54uckyzJezMdnK6WT9hCfCpyMG4sV8suq135S9vGpy8AyZjl5oY8vhtbFZGljPjWv0Ligl8TXQyv14KHTS99aF5OljfmGygvvYeEtIF5VwdkVXlnZdvu59P6nptM72ze4LSGsqnStLVRQsPS8\/TfUZ+S9bmsIeYJ4fGghg8Bz\/ll49EkheY1YnJBfVr32k7KPT50zGbOcvNDHF0PrYrK0MZ+aV2hc0BO995Ovh\/Y97bscdHyPvauhA7YrodM+Ug7x6FdEqugCBiswzzzzjCtYGHwUM9B16dKFVQ3S4kBwn\/1PO+XpKacGSd6CGAIVjABWUEB4XBnFCQjbPihQQFykYEUFehy2hS\/OrqBQAXGxgtUVyG3PvsLe01LBc25DMwSyIFDRBUybNm1cDnfddZdr8WPevHlo6PDDD6du3bo5PvUHCqVp06bR\/PnzHT3++OMEXWpc628IGAJEKD6YUFiAUISA3lz8kPu+oO33f9mdRUFBAkIxgsIEBB60qXbrRxYowFYXKX3mvuNeJIeVFS5UrFghAlZGhkDREKjoAuaII46ge++9l5YtW1Z3DmbOnDl0xx130I033kh9+\/ZtkPlYuXIl7dy5kzp37kxYvmvbti1BJ4MvXbqUlixZ4mjdunVubxx7jKVox44d9MILL9D27duDfWI+PlspnbSHeIx779697ntNwNeXZNxY35BfVr32k7KPh67svDZWU80rL9BL\/\/k0bV\/z37SnVg5RzM9n07qYLG3g1y6cQzUrFhK9vITw4V8fQpHAhHeX4BzItidmuKdtoMdqhaRts2+iNTOvcWdFUCxoWnPnRfTKTZ9wjxaj0AC5d6PU6lBkgNaMPJpQeDBBx4QiBMRxX5tzM7299OHavBbVYr+a9rXt7IiOOZXaful7jvBKflDHqaup24Nvuveu4N0rR379V87+9kmfpQ1dBhHeyxK6F3FfhH4PfbZSOmkP8RhL2ffinj0k4yJWiEJ+WfXaT8o+HrrmzAs4YAx6PrUuJksb82hT8gqNC3pJuI4eO9t9Nq2LyWxD\/MWLF9Njjz1G1dXV8qOsUHxFFzAa6XvuuYeOP\/542rVrF82ePZvwOLX2KVfu3r078YoPYshJX3jKWhqy+BrqNPU0R3+9rjfhMcss9Po\/dqAP3NyPto88Mtgn5uOzldJJexY+Sx7aR8bVNimH\/LLqtZ+Uffzur3Ytibcc3wH82GPdNwe\/\/+5zaPvE\/u7L+PCFfD7CNwyH\/Hw2rYvJ0gb+nfuHU82UIYR2+7c\/RSgAshIXCmi33\/+P9M4j11HND69yqx3QPf\/oDJL04oIf0sanHqaXl\/w7vbj4CXr62eUH0B9WVdPSdTU0d10rR7\/938PpP2s60NI3PkhLO55Hr\/Y9EFQeAAAQAElEQVS5gKp7DKa3Pz+ZDrn4bvrL0Ntp+7nX09tffpD+csWPaPMVj9Bfrn2S\/nLbatrwtWdo99eeqNMdMnoOvX3h5Draddx5BNpx5HEEeu3tVrRp06aDaPPmzbRt2zbasmXLQTb2j\/n4bKV00p6F53HUp5VxY\/1Cfln12k\/KIb4U3uWMV\/eR185i0\/4xWdokn5IXxihjQfZRzMdn07qYzLZZs2bR5ZdfTtdeey3dcsst+EgrJFV0AYNC5ZprriE+yIsVmWHDhrnDu7A11Yzcta4j7a39Q4wl6aJQ29p\/3eIDpsOo71NRckIeTZ3Xvq\/9ipjWfPFRSqGnL5hDIVp43oP0L2fNdPY5Q58iTXd+4l8pRmPP\/i\/y0Z1n1ParpatPmEc+uuP47xFoVKd\/Ikm3HjmRQKMOG0dfqZpAaEfVfIGGbPw8XbLjcjr1P0+lU549hf7mv\/6Gzn3hXPrcsx+ii5YdQyOXdaGv\/u5w+urj2+nH1UfQz6vfR\/PWHEY\/fX4fPfUq0ct\/rnL0VqsO1LVr10zUq1cvGjhwIKEN9YEt5OOzldJJe4jHim7Hjh2pS5cumfLQY5dxtU3KIb+seu0nZR\/fs2dPas68kLscF2SQ1sVkaWM+NS\/fGKDTxNdDm8UGP3nvxmS2jRo1yv0jH\/\/QHzt2bFN9VDb5dSq6gNm9e7cDBAd3HVP748QTT6z9ScQ2JzTAjw0bNhwQU25P\/dv\/HkFv9P0ktT37ikIRfezz1PqMywqVE+ZI5wVdY9GHP3E+MQ3+zGcpha684hIK0eWXXURDLhhMwy\/5LN142dkH0f1f+xTFaMGYkylGz910GsXo9XvOIU3a\/5dfOZmYpl\/aj5gmnteLvnhaD0dnHXcUfbhbe\/dkxdbaX+9HV+2gJ9e+RdN\/u43GP7aOLn7gBUeDpiynbjc86wg8aNgPV9O0RZscod\/v1+8hxKiqqnLxKq3FUy2tWrWqyLGlYGV5Veb9JucUhcygQYMIdOqppzbAJ2RlhqjoAoa3dPjgLiDkA71sgy6VUBS1a9fOLUHjGfyamhqCLjUu+mNvEzHRQvYRbCEfn62UTtpDvG8c9dHJuLF+Ib+seu0nZR+f+i4HGbOcvNDHF0PrYrK0MZ+aV2hc0Evi66GVevDQyfu055FV1KkN0dtvbHLtGX3b08d6VFH392+nz\/VvT5cO7OLaiz9MNPasro7GD6qiqZ\/rTXO\/1I++8+n29JuxH6EFI3rQxm+d5gokLopQCKEAQvGDOKcf0x5DoMXVO+jOX6+jMXNW02e+u5xOuv1ZOvLap+gjk35Dn5q2rNa21hH8XIf9P\/TY96td47OV0kl7iHfBE37IuLEwIb+seu0nZR+fei\/KmOXkhT6+GFoXk6WN+dS8QuOCXhJfD63Ug4dO\/o75dNpHyiEecYpIFV3AYMvo3nvvJRzcRREzcuRIwoFe6GBrqAnB+2awzDZ06FACgYeuIeJXVVW5g8GojkPxYMPhYbTaBzptK6WT9hCvr1NfWcaN9Q35ZdVrPyn7ePzrMDaeUjYZM+Yb8\/PZtC4mSxvzqXkhF46FFrKPYNP3G\/v5bFoXk6WNeZ0XiiIQiiEULhPP600gFDO8gsQrQSh2uNDBCk+fTkfUFTgoblDYoMAZMn25W7VZ8lprtxrC+XDLY0GbVQdfxinEc6xyWxk3FiPkl1Wv\/aTs4\/Wcxcbms8mYPjvrYn4+m9bFZGljPjUvjJtjoYXsI9j43tF2n03rYrK0SV5fpyhyRRcwABmFxKJFiwgvsQOBhw62hqTBgwfXXQN8Q8a2WIaAIdDwCOhCh4scFDjP1W6JobhBEQQ\/XrXhogYrOHf+ei298vqehh+YRTQEDIGGQaBElIosYPAOlrMCXyGAl83BBp8SuZnZEDAEWigCKFqwisOrN7qoQeGCogYrNCAUM3rbqYVCZ2kbArlBoCILGKywYKUFKy742gAc4gUPgoynkuCTB5TlnmRovDEfn62UTtpDfGgsWfUybqxPyC+rXvtJ2cen7mPLmOXkhT6+GFoXk6WN+dS8QuOCXhJfD63Ug4euoffnmzIvnNXBGR0UNdiO4qJm2MlH0YPPrq87U4Ni5ukXXqNYrhKLEA\/MUkjGjcUJ+WXVaz8p+\/jUOZMxy8kLfXwxtC4mSxvznrxwqXoRx0Ib6gibvrfY12fTupgsbZLn+EVrK7KAYZCxyoKvDUDRwjoUM9DBxjprDQFDwBAoBwGs1Iz75NHuIPGS8Se7g8dYiblw5ioaMms9Tf33V8sJa30MAUOgCRCo6AKGnzTCAV7GojGeQuLYjdFmOUgV8\/HZSumkPcSn5irjxmKF\/LLqtZ+UfXzqQTwZs5y80McXQ+tisrQxn5pXaFzQS+LroZV68NDpw4daF5OljflKyguPd\/PqDM7Q4GDw1H9f755ywqoMHtnm\/Hn8aEE+PTBLoaqq0g8AIL68PmSmrHrtJ2UfnzpnMiaP1dfG\/Hw2rYvJ0sZ8al7IgWOhhewj2Ph+0XafTetisrRJXl+nKHJFFzB40ghPHOEpJJx9ATXGU0hFmUzLwxAwBBoGAazMoJjBNtPE83rRnGWb3SPbKGQa5goWxRAwBFIRqOgCBsnhrAufh8EZGPDQwZYHyrIPGfPx2UrppD3Ep2In48Zihfyy6rWflH186j62jFlOXujji6F1MVnamE\/NKzQu6CXx9fbs2SPVjvfZtC4mSxvzechr4nm93Uv+sMW08L83uxWZOx5\/qe6MDOeCFhQ63+BArMePrLFCfln12k\/KPj51zmTMGBwxP59N62KytDGfmhdy4VhoIfsIttA94rNpXUyWNsn7xlEEXUUWMDjfMnXq1JL4ZvEpGaSRHbIs48V8fLZSOmkP8alpy7ixWCG\/rHrtJ2Ufn7oMLGOWkxf6+GJoXUyWNuZT8wqNC3pJfD20Ug8eOr30rXUxWdqYz0teyB9bTE+OO829bXju8tfocz\/dTHgbMOeCFqQxQt9yKGuskF9WvfaTso9PnTMZM4ZLzM9n07qYLG3Mp+aFXDgWWsg+gi10j\/hsWheTpU3yvnEUQVeRBQxWWAYMGOC+gXrGjBkH4QwdtpPgc5DRFMVAwLIwBCoUATyejTMyeMcMXphn20oVOlE2rMIjUJEFDFDHy+SwZQQexYok6GCDD3gjQ8AQMASaGgHeWsL5GLztt6mvb9czBFo6AhVbwPDEjB49uu4NuShaQNCxvZHaBgubZR8y5uOzldJJe4hPTVDGjcUK+WXVaz8p+\/jUfWwZs5y80McXQ+tisrQxn5pXaFzQS+LroZV68NDpvXuti8nSxnxe8tL58\/jxXVA4G3PkB\/a5Q76sR4s+KYQYGm9fvJBfVr32k7KPT50zGdOXD+tifj6b1sVkaWM+NS+Mm2Ohhewj2ELz6rNpXUyWNsn7xlEEXcUXMEUA2XIwBAyBYiMwafBRhBfiDZqynP6wwb6eoNizbdnFEWg6qxUwjYx1loNUMR+frZRO2kN8atoybixWyC+rXvtJ2cenHsSTMcvJC318MbQuJksb86l5hcYFvSS+HlqpBw+dPnyodTFZ2pjPS146fx4\/WhBwufGCY90XT97+9A53uBd9Uojjoo3FgR3XRyv9IGfRaz8p+\/jUOZMx5Xg1H\/Pz2bQuJksb86l5YfwcCy1kH8Hmmxf4+mxaF5OlTfKIXUSyAqaIs2o5GQKGQLMggIO9913Sj3Cwd3H19mYZQ0u\/qOXfchCo+AIGj1TjyxtxiPexxx6jUaNG0apVq3IzQ1n2IWM+PlspnbSH+FQAZdxYrJBfVr32k7KPT93HljHLyQt9fDG0LiZLG\/OpeYXGBb0kvh5aqQcPnd6717qYLG3M5yUvnT+PHy2IcQHf\/f3b6XP929P4x9ahW9mEWBw3FiTkl1Wv\/aTs41PnTMYsJy\/08cXQupgsbcyn5hUaF\/SS+HpopR48dHrOtS4mS5vkEbuIVNEFzK5du+iaa64hfHkjvgMJb+YdNmwYTZ48mWDLw4RkWcaL+fhspXTSHuJTsZNxY7FCfln12k\/KPj51GVjGLCcv9PHF0LqYLG3Mp+YVGhf0kvh6aKUePHR66VvrYrK0MZ+XvHT+PH60IMalqurd1\/9feWZvOv2Y9jRy3mZ0LYtk3FiAkF9WvfaTso9PnTMZs5y80McXQ+tisrQxn5pXaFzQS+LroZV68NDxvQQZpHUxWdokjzhFpIouYHbv3u0wR\/HimNofJ554Yu1PIrY5wX4YAoaAIVBhCEz9XG83omE\/XO1a+2EIGAINi0BFFzBF+DLHLMt4MR+frZRO2kN86m0k48Zihfyy6rWflH186jKwjFlOXujji6F1MVnamM+aF64fIo6Fthwf9Gvo5e285AW8ZP5ZePSZeWEX2rdvnzsTA7k+JK8R6xfyy6rXflL28alzJmOWkxf6+GJoXUyWNuZT8wqNC3pJfD20Ug8euob+HUNMxC4iVXQBgy0j\/jLHefPm0ciRI8m+zLGIt6HlZAgUF4Fxn+xRW8Cso8XVO4qbpGVmCDQDAhVdwAAPFC4DBw6kJUuW0NFHH02vvvoqrVy5EqZcUJZ9yJiPz1ZKJ+0hPhU8Gfe9WAdzIb+seu0nZR+fuo8tYx6czXuamJ\/PpnUxWdqYT80LI+dYaCH7CDa9B89+PpvWxWRpYz4veQEDHjNaEOMU4tEHdEbfDnTpwC61RcxaiJlJxo11Cvll1Ws\/Kfv41DmTMcvJC318MbQuJksb86l5hcYFvSS+HlqpBw8d31eQQVoXk6VN8ohTRKroAgYHdZcvX04jRowgLmTmz59Pc+fOzc0h3iLeNJaTIWAI1A+B6Zf2o1e376l3EVO\/q5i3IdCyEKjoAgYHdWtqaqhz585UXV1Nffv2dTx0sOmpqkQZ+496T1OPM+bjs5XSSXuI12OoryzjxvqG\/LLqtZ+UfXzqPraMWU5e6OOLoXUxWdqYT80rNC7oJfH10Eo9eOj0vax1MVnamM9LXjp\/Hj9aEOMiefSR9O77YbJvJcViybghv6x67SdlH586ZzKmzEPzMT+fTetisrQxn5oXxs+x0EL2EWx8v2i7z6Z1MVnaJK+vUxS5ogsYHOJt27at2zJ68cUX6fTTT6ctW7YQdLAVZRIsD0PAECg+Amf0be8ercZL7oqfrWWYMwRyOdyKLmBwiBfvfcHh3eOOO86tvtxwww109dVXE2x5QDzLPmTMx2crpZP2EJ+KnYwbixXyy6rXflL28an72DJmOXmhjy+G1sVkaWM+Na\/QuKCXxNdDK\/XgoWvo\/fm85KXzl1iEePTRtGDMyfTbl3fQnGWl3w8j4+o4Ug75ZdVrPyn7+NQ5kzFlHpqP+flsWheTpY351Lwwfo6FFrKPYNO\/R+zns2ldTJY2yXP8orUVXcAA7MGDB9OaNWtoypQp1KlTJ1qwYAH1798fJiNDwBAwBHKHAM7DjJlj74Y5YOJMMATKQKDiC5jx48cTvkZAEr5aAF8xUEa+Td4lyz5kHXajfgAAEABJREFUzMdnK6WT9hCfCoSMG4sV8suq135S9vGp+9gyZjl5oY8vhtbFZGljPjWv0Ligl8TXQyv14KHTe\/daF5Oljfm85KXz5\/GjBTEukkcfH+GJpJ5HVpU80JslFuKH\/LLqtZ+UfXzqnMmYGH+IYn4+m9bFZGljPjUv5MGx0EL2EWx8v2i7z6Z1MVnaJK+vUxS5ogsYFCk4+4InjyZMmEAzZ84ktPhqAazGFGUSLA9DwBBoVgSa\/OITz+tdW8Csa\/Lr2gUNgSIhUNEFDIDGgV08hYQnkJ544gnC1wo888wzhOIG9kqnLPuQMR+frZRO2kN8Km4ybixWyC+rXvtJ2cen7mPLmOXkhT6+GFoXk6WN+dS8QuOCXhJfD63Ug4dO791rXUyWNubzkpfOn8ePFsS4SB59QoRVGHxXUmwrKWuskF9WvfaTso9PnTMZM4QP9DE\/n03rYrK0MZ+aV6kxww7i66GFLAk6vpdYr3UxWdokz7GK1lZ0AYMnjbp16+YwRxHz5JNP0qBBg2jDhg1OZz8MgUIgYEm0SASGf7yrO8z7yut7WmT+lrQhkIpARRcweNLooosuco9O4+Du1KlT3SPU9913nzvQW07yCxcurDtTM3z48LoX4kn9jBkzygnt7ZNlHzLm47OV0kl7iPcOth5KGTfWLeSXVa\/9pOzjU\/exZcxy8kIfXwyti8nSxnxqXqFxQS+Jr4dW6sFDp\/futS4mSxvzeclL58\/jRwtiXCSPPjHiVZjQY9VZY4X8suq1n5R9fOqcyZgxfGJ+PpvWxWRpYz41L+TCsdBC9hFsfL9ou8+mdTFZ2iSvr1MUuaILGICMFRcUL+DxRNKKFSvKfgoJ20633XabO0uDJ5vOPPNMmjRpktuOmjZtGuGsDejxxx93OlyzBZClaAgYAs2EAM7C4JFqW4Vppgmwy+YagYovYLAaIp9AAl\/uU0h4CV67du3oxBNPdJOGczUbN250XxC5c+dO954Z7D\/i3E1Dfd9Sln3ImI\/PVkon7SHeAZDwQ8aNhQn5ZdVrPyn7+NR9bBmznLzQxxdD62KytDGfmldoXNBL4uuhlXrw0OH3Ay1kEHipi8nSxnxe8tK58vjRghgDyaNPKYq93C5rrJBfVr32k7KPT50zGTOGT8zPZ9O6mCxtzKfmhVw4FlrIPoKN7xdt99m0LiZLm+T1dYoiN38BE0ESKyZz5sypWzHBqglo0aJFZW0h4RwNChUuTvD1BDhPs3fvXurevTvhzA0PBzbm0S5dupSWLFniaN26dYTlubwT8sayad7z0OO3vPbk6v4s6nzhvsyS29izurizMH\/cuCM385YlL+SfNypKXviM4s8rfMbhM6yIVNEFDABHYcErJpBTCI9e33zzzYQ3+2IlB18UiRWZLDGxxYQzM6BZs2bRpk2bMhFupGXLlhHaUB\/YQj4+WymdtIf4zZs307Zt29z5otC4YnoZtxy\/UH+tj8nSxvyf\/vSnZs0LWPBY0EIGgZdzHJOljfnUvHxjgE4TXw9tFhv8WkJewELmmoXP+jt2zOF76JTuVXTH4y8d8DdFXgPXD1HIL6te+0nZx6feizJmKCfoY34+m9bFZGljPjWvUmOGHcTXQwtZEnTy9wk2rZOytrPt\/vvvJ3xWga677rosH3G59KnoAgYFB86pzJs3ryxwV61aRQMGDHCHdjGRu3btIpyjwSoOCF9TgO2iVq1auSebdu\/eXXcdbC\/VCbXM3XffTbNnz3Y0YsQI6tq1aybCqk\/Hjh2pS5cuQf+Yj89WSiftWfisuUg\/GVfqNR\/yy6rXflL28VKnx5JFzto\/5uezaV1MljbmuY3dR6XyyxIj5uOzaV1Mljbmua30vICtHGt9efSP0Y0XHEsLVu+it1p1qPs7Ia8R6xvyy6rXflL28VIXG1fIlrV\/zM9n07qYLG3Mc1vp96Iep5SZx2fUAw88QPfccw+NGTOm9hOsmP9XZAGDrSOcc8EqyV133UUg8EywwafUlODwLw79olhB8YECZciQIYTCBsUMJhgF0sCBAwkrMTgjg9PhNTU1dedk+BpYCcKBYlCvXr2oqqoqE3Xo0IGOP\/54at++fdA\/5uOzldJJe4jHfi8Kt6x5aD8ZV9ukHPLLqtd+Ug7xzZkXcpfjggzSupgsbZJPycs3Bug0yetlsWn\/mCxtks9DXsBCjjkLX5\/fsbOPP8p90eO0RZvq\/k7Ia+D6IQr5ZdVrPymH+HfnLNvfQD1uGVPbpBzz89m0LiZLm+RT8sLYZSzIPor5+GxaF5PZhs+cs88+mz772c\/S6aefzh9jhWsrsoDBysuiRYvcdyCh+NAEG3zqOxvoM3bsWBo6dKgrUPCOmdGjR7vzNKyHDTx86xvf\/A0BQ8AQKBcBfiJpcfWOckNYP0OgRSFQkQUMZgCrJHr7B\/pUkltI+IJIjif14Fmf2uIQG1Z10IZiwRby8dlK6aQ9xIfGklUv48b6hPyy6rWflH08DiXHxlPKJmPGfGN+PpvWxWRpYz41L+TCsdBC9hFsKfei7i9lH5+SF49fxmWdbmM+PlspnbSHeD2GUjI\/kTRn2SbnKuM6ReBHyC+rXvtJ2cenzpmMGUjJqWN+PpvWxWRpYz41LwyaY6GF7CPYmvJ3DNfzjaMIuoosYLC9M3nyZMKL67D6gpWShx56qAh4Ww6GgCFgCAQRsFWYIDRmMAQOQqAiCxicVcFI+ekjHEjCaWvo8kbYAw0988+5xHx8tlI6aQ\/xfO1yWxk3FiPkl1Wv\/aTs43HuIDaeUjYZM+ZbVVVFoXn1xdC6mCxtzKfmhVw4FlrIPoLN8iJ3DkXjILHJwvvwLaXDKgze0Iu388prxPqF\/LLqtZ+UfXzqvShjlpMX+vhiaF1MljbmU\/MKjQt6SXw9tFIPHrrYfefzkX1CPPoVkSqygPEBjUcSsTLjs5nOEDAEDIGiIIBVmN++vIPsLExRZtTyaCwEclPANBYAjR0X+4+h\/U6+dszHZ4NOx5S6LDxfu9xWXiMWI+SXVa\/9pOzjU\/exZcxy8kIfXwyti8nSxnxqXqFxQS+Jr4dW6sFDF7vvfD6yj4\/PS146N18u0IE0RuhbH+p5ZBVhFQbvhckSK3TNrHrtJ2UfnzpnMmYMl5ifz6Z1MVnamE\/NC7lwLLSQfQRbaF59Nq2LydImed84iqCr2AIGbw\/EI8t4dBpPBj355JPuySHIWR+jLsIEWQ6GgCHQ8hCYfmk\/WvqnXe7dMC0ve8vYEMiGQJkFTLbg5XrhEWY8Ko0DvD6CDT7lxm\/KfnJPMnTdmI\/PVkon7SE+NJasehk31ifkl1Wv\/aTs41P3sWXMcvJCH18MrYvJ0sZ8al6hcUEvia+HVurBQ9fQ+\/N5yUvnL7EI8eiTQhPP60WTFr7mzuPE4sjrS7+seu0nZR+fOmcyphyv5mN+PpvWxWRpYz41L4yfY6GF7CPY9O8R+\/lsWheTpU3yHL9obUUWMEUD2fIxBAwBQ6C+CEw8r7d7uR0O9Na3r\/lXMAI2tAZDwAqYBoPSHyjLPmTMx2crpZP2EO8fbXatjBvrFfLLqtd+UvbxqfvYMmY5eaGPL4bWxWRpYz41r9C4oJfE10Mr9eCh03v3WheTpY35vOSl8+fxowUxLpJHnxRCrBEDqujOX6+LHuiFH19fXi+rXvtJ2cenzpmMKcer+Zifz6Z1MVnamE\/NC+PnWGgh+wg233zB12fTupgsbZJH7CKSFTBFnFXLyRAwBAqBAL7kEd9WffXPVjdUPhbHECgMAlbANPJUZtmHjPn4bKV00h7iU9OWcWOxQn5Z9dpPyj4+dR9bxiwnL\/TxxdC6mCxtzKfmFRoX9JL4emilHjx0eu9e62KytDGfl7x0\/jx+tCDGRfLok0Ic6xtD+9HRHbASs9Ybjv3QSgfIPK6YXvtJ2cenzpmMKcel+Zifz6Z1MVnamE\/NC+PnWGgh+wg237zA12fTupgsbZJH7CKSFTBFnFXLyRCoVARsXGUhgPMwpbaSygpsnQyBHCNgBUwjT16WfciYj89WSiftIT41bRk3Fivkl1Wv\/aTs41P3sWXMcvJCH18MrYvJ0sZ8al6hcUEvia+HVurBQ6f37rUuJksb83nJS+fP40cLYlwkjz4pJGPhDb0Tz+tFvq0k6Sevl1Wv\/aTs41PnTMaU49V8zM9n07qYLG3Mp+aF8XMstJB9BBvfL9rus2ldTJY2yevrFEW2AqYoM2l5ZEHAfAyB3CIw8bzebitpyPTluc3BBm4INCQCVsA0JJqeWFn2IWM+PlspnbSHeM9Q66WScWMdQ35Z9dpPyj4+dR9bxiwnL\/TxxdC6mCxtzKfmFRoX9JL4emilHjx0eu9e62KytDGfl7x0\/jx+tCDGRfLok0K+WAvGnOxCjpnz3qFenx+csuq1n5R9fOqcyZgYZ4hifj6b1sVkaWM+NS\/kwbHQQvYRbHy\/aLvPpnUxWdokr69TFNkKmKacSbuWIWAIGAKJCOAtva+8vofmLNucGMm6GwL5RsAKmEaevyz7kDEfn62UTtpDfGraMm4sVsgvq177SdnHp+5jy5jl5IU+vhhaF5OljfnUvELjgl4SXw+t1IOHTu\/da11Mljbm85KXzp\/HjxbEuEgefVIoFAvflYQiZvZ\/bnJFTMgvq177SdnHp86ZjBnDJ+bns2ldTJY25lPzQi4cCy1kH8HG94u2+2xaF5OlTfL6OnmUfWO2AsaHiukMAUPAEKhgBFDEYDsJW0lT\/\/3VCh6pDc0QaDwErIBpPGxd5Cz7kDEfn62UTtpDvBtcwg8ZNxYm5JdVr\/2k7ONT97FlzHLyQh9fDK2LydLGfGpeoXFBL4mvh1bqwUOn9+61LiZLG\/N5yUvnz+NHC2JcJI8+KZQl1nM3nUZL1u2iKUv2HPSdSaH+Wh+TpY351DnjOGhj+MDOuGo\/n03rYrK0MR\/PS4\/AL3MstH4PcvPUlHnFxhIaY170VsDkZaZsnIaAIWAIKASwEoPtJJyJsaeTFDgmFh4BK2AaeYqz7EPGfHy2UjppD\/Gpacu4sVghv6x67SdlH5+6jy1jlpMX+vhiaF1MljbmU\/MKjQt6SXw9tFIPHjq9d691MVnamM9LXjp\/Hj9aEOMiefRJIRkrFqdTG6LvfLo97du3j066\/Vl3Lgb+of5aH5OljfnUOeM4aDHOEMHOuGofn03rYrK0MZ+aF8bIsdBC9hFsTZkXrucbRxF0VsAUYRYtB0PAEGjxCDw6qj\/dd0k\/wrkYW41p8bdDiwDACphGnmbsP4b2O\/nSMR+frZRO2kM8X7vcVsaNxQj5ZdVrPyn7+NR9bBmznLzQxxdD62KytDGfmldoXNBL4uuhfU\/\/Lgedvpe1LiZLG\/N5yQsI8JjRghiLEI8+KSTjxuJIP7yxF+di4N\/thmfpkT++e+YCMpP0hy4mSxvzqXPGcdDi+iGCnTHWPj6b1sVkaWM+NS+MkWOhhewj2JoyL1zPN44i6KyAKcIsWlFvtdcAABAASURBVA6GgCFgCOxHAOdi8ITSL79ysttOwrbSnb\/2fxHk\/i7WGAK5RMAKmEaeNuw\/hvY7+dIxH5+tlE7aQzxfu9xWxo3FCPll1cNP4idlH5+6jy1jlpMX+vhiaF1MljbmU\/MKjQt6SXw9tFIPHjo5Fz6d9pGyj89LXjpXXy7QgTRG6FsOZY0V8vtYjyp69O+70LCTjyJ8ESQXMn\/cuIPkGHV\/Kfv41DmTMWO4xPx8Nq2LydLGfGpeyIVjoYXsI9gk\/tLHZ9O6mCxtkpfXKBJvBUyRZtNyMQQMAUNAITDuk0fT6\/ecQ5cO7OJWZAZNWU5DZq0ne3+MAsrE3CFgBUzJKUtzwP5jaL+TI8d8fLZSOmkP8XztclsZNxYj5JdVr\/2k7ONT97FlzHLyQh9fDK2LydLGfGpeoXFBL4mvh1bqwUOn72Wti8nSxnxe8tL58\/jRghgXyaNPCmWNFfLTenwZJM7HgM467qjaAmY9HXntUzTu0bUHnJWR\/Xx86pzJmDF8Yn4+m9bFZGljPjUv5MKx0EL2EWx8v2i7z6Z1MVnaJK+vUxTZCpiizKTlYQgYAoZABgRwRgbvjkEhM\/G8XoR3yGCLCcUMbzMtrt6RIZK5GAIlEGhksxUwjQxwln3ImI\/PVkon7SE+NW0ZNxYr5JdVr\/2k7ONT97FlzHLyQh9fDK2LydLGfGpeoXFBL4mvh1bqwUOn9+61LiZLG\/N5yUvnz+NHC2JcJI8+KZQ1VsivlB7vj5l4Xm+a+6V+tGBED5o2rK\/bZkIx85nvLnerMyhovv6z5fTAM2uJ46XOGcdBG8MHdsZV+\/lsWheTpY351LwwRo6FFrKPYGvKvHA93ziKoLMCpgizWEYOmzdvpp\/85CeEtozuFdtly5YtllfFzs7BAyvqfCFT\/G7l5XesW7tDadjJHQkFzcZvneYKmnGf7FGrO4pm\/ucOGv\/YOsJj2Tg7c870F+j+pTtqt6BepQpfqcE0ZKYi34uZQciZY+ELmBkzZtD48ePrpmXVqlU0YMAA6tOnD5111lm0detWZ1u4cKHTQY8+TtkAP7LsQ8Z8fLZSOmkP8fjjOmvWrLILGBk3BlPIL6te+0nZx+\/YsYOaMy9gIccFGaR1MVnamE\/NyzcG6DTx9dBmscFP7ufHZGljPi95AQseM1oQ5x3i8\/w7dvqAY+nGC94lHADGdhMey\/7iaT3oQ63+TA\/869LaAmY98UoNbz\/hBXp4ZBs0Z9lmV+Bgiwr4MUm8WOdrY34+m9bFZGljPi\/3Io8XLXBDW+pehF8RqdAFDAqXu+66q27edu3aRZMnT6arrrqK1qxZQwMHDiTYUcRMmzaN5s+f7+jxxx+vK2zqOpfJrF+\/nhAbbSgEbCEfn62UTtpDfGgsWfUybqxPyC+rXvtJ2cdv2LAhNpySNhkz5hzz89m0LiZLG\/OpeSEXjoUWso9gS7kXdX8p+\/i85AWsfOOHDsSYSZ7QKYGyxgr5ZdVrPykz\/77dr1Gvql1U9cIv6fI+b9ARi++if724NXFhM\/G8Xm77Cekurt7hHtvGG4FR4GAbigsc8ChyYEORw8TFDvpywcPXRou4kqBjzFmvdTFZ2pjPy73I40WL3NEyFj6+IfLCdSqRClvAYBXl\/PPPpwsvvLAO9yOOOIK6detWJ4Pp27cvrVy5knbu3EmdO3cmVLJt27Z1OtiZli5dSkuWLKk3oR9url\/84hfBvjEfn62UTtpDPN\/UsDdWXoiL+L78s+q1n5R9PK6JOYMNfH0J\/Xzj1XFifj6b1sVkaWMe10\/JC\/05VmPei\/oaUvbxGFce8sI4feMHliF9S\/gd2\/jH5+jQ116gsz64xdGNp+wlEIqb+889hECQv9j7Der99p8cbXppBc391SK6+9Hfu0IHZ25Q0KDYAaHIQcFz4j0v0a1\/OpH+7sevuu94gh7FD+iq+dto8m9qaMycF2pjrN1P6+paFEQgXAMEHjT1p0\/Qw4v+m3778g7S\/O9WVtNf2xxFz656mcAzYe6zkrwXQn1iPj6b1sVkaWMe48DvWBGpsAXM6NGjafDgwQfN2ZQpU6i6utptF6HAgR+cunfvTm3atAHrCD5gevToQaeeeqpbRRk+fDjVl6677jqEifaP+fhspXTSnoWvb07wl3Ehhyjkl1Wv\/aTs4\/GBAsBRhITGFNPLmOX6+WJoXUyWNubLyOuge5VjxbCJ+fhsWheTpY35vOSFe4HHDPzqy6N\/fUleI9Y35JdVr\/2k7OOzztnEMVcSaPrNX6EF\/3QtrZh1g6PXf\/lNt4LT7t8mUvvHvuQIPBNWd9r81wMEqnphPm393cN1tOypxwmED+d3i41qVwi9W6Rsoj3HD6WfPv9WbWGz2hU4uz96JT249oN18q4zJtAdf2hFKIA0\/6Oaj9POv72TblvdzRVNKJxAn37kTQoR7JJGPflXb4ysPtwfY\/zMd5+r3aJ77oCxQifHrmVpY37WlmMIueKzDH8bi0SFLWB8k4StIpx7QeGCLaQnnnjigPMxvj6Y9Lvvvptmz55tZBjYPWD3gN0DjXAP\/OwH3yGmR77zDfr5t8c4mnfLJSTpsfGDCfTLr5xETP9yxdHEhJWfELGPr\/3lV06ujXcw4XFzH2HLDHTd5z5GDUFfOOsjJGnYp86ihiZ8lvk+4\/KsK0QBg7Mt+BcKDuDigC4O6vomBafM27VrRyeeeKIzo5BZtmyZ2z7Ccu\/u3budHj+wtYQWhIkfNGgQGdUDA8PL7he7B+weqKB74G9O7Eshwpdg+ghvL\/bRxPN6uye2Gqr1FUkNrcNnWdGoEAUMzrZghQSrKitWrKD+\/ft75wlnXHDWBWde4IBtImwdffSjHyUUNihw8Hx+TU1NXZEDPyNDwBAwBAwBQ8AQqCwEClHAeCD1qjp16kT33XcfjRs3zp2BmTNnDt17773Uq1cvGjt2LA0dOtQRePh6gyjlv\/3bv9Hvfvc7pc2niJUs4DFy5EhavXp1PpOIjPqtt94iHO7etm1bxCtfJuTyjW98g7785S\/T4sWL8zX4yGi3b99Ot912G1177bWEf1REXHNpwqP+RZuvO++8k2644Qb6\/ve\/T3v37s3lvPgGjb8b+NwYMWIE\/eY3v\/G55FL38MMPu\/nCnH3zm98kPMGUt0QKX8Dg0C6IJwarM1ilwWrNokWLiAsVHPiFDgSe\/UPtvn376IEHHqDp06eT3HoK+edBj8fHTz75ZPr2t79Nc+fOJRQ0eRh3ljG+9tprdOONN9JTTz1F77zzTpYuufD5+c9\/Tueddx798z\/\/M6GYRkGTi4GXGOSvfvUrwhYvXoXwgx\/8oFAfiPj78x\/\/8R\/uzbYlYMiN+ZVXXqEPfehD9K1vfYv+8R\/\/kVq1apWbsZca6IIFC9w\/ePGPn5deeqkwfxe\/8IUvuPm6+OKL6YMf\/CBhN+JdLPLz8335GWrjjhQvssM5Gv7QRgsZ52pw8BcHgPUIPvvZz9JXv\/pVra44GWMfMmQIybNB+GVEbiDkjkG\/\/PLL7jHzI488kg4\/\/PBcFGZZc8PLnm666Sb6+Mc\/jlQrnrLmhXca\/c3f\/I37o4p\/KSLPSk4ua1743cM5NBTVxx13HH3gAx+o5LTce6Oy\/I5he\/qZZ54h\/O2o6IT2Dy7rfGGV7Je\/\/CVddtllhA\/8PPwjIWtuOGqAv41f+9rX3PEEHFnYD09FNlnzwuDx9QmPPvooXXTRRXTIIYdAlSuyAqZ2uvBhjm2TWrbu\/4ceesh9mGNFBi+8mzdvHmGy8S94\/FHFv3TxQV\/XoUIZFC3nnnsu4V9IPETokAPeDzBz5ky3koSCDR8SKFzYr9Jb5JE1N\/zRwTmnSs8J46tPXvDHvXjrrbfSJZdcQniHEXSVSPXNC681+NjHPkYvvvgi\/fnPf67ElNyY6pMXPuTPOeecip4nl1Ttj\/rkhYcnsM3yox\/9iP7whz8kbUfUXrrR\/69PbtjO7NevH\/3TP\/0TPfbYY4Tft0YfYJkXqE9euATm6sMf\/nAuV18w\/hZfwPDqw4QJE4CHI3yY419J+BcgFFjKhox\/4X7kIx9xb\/DNQ\/GCShx77ffff79bIkQuoN\/+9rfuDyg+IPBEFv5ViH9BdezY0f1yYv8adOihh8K9Iqm+uVVkEp5B1TcvPD13++23u3Nd+BDxhKwIVX3zwhbSG2+8QSeddBJhValSPzTqkxcKMWw344GDBx980K1U4KGCipggNYj65IW\/HdgWQ274m\/G+972vorfH6pvbUUcdRfjbiH\/c4R9Blbq6VN+8MOV4nw4+A8DnkVp8AYPzLvwyOz2BXMBAjw8KfKsnbmScm8nDHi\/GifM\/+MVDDpK6detGWJWADn9E8QTWmWee6Q7g4UBXz549qZKLtPrmhjzzQPXN64c\/\/KFbGcTBSRyixFxWYp71zQv3JwoznFtCoX300Uc3UVr1u0x98sK\/5HE+BOdEvvjFLxK2nPCBWL8rNo13ffLC3w6cn8DZOWzT4jxF7969m2agZVylvrl9+tOfpu9973uEVU4UL5X6d7G+eWFV83\/\/93+pS5cuZaBYGV1afAGTOg0ogECpcSqhP\/7oYBn4jjvucHvZlTCmhh4DnkDDL3pDx22ueLfccgthGxAfihMnTnSvA2iusTTkdbGahCV7fGhgzvAv+4aM39yx8DcD1NzjaKjrY5ud\/3bgKc4izRe2j3DMYNKkSfT1r3+dipIb\/mGLvxv4R3lD3QdNHccKmAjiOLzFZvwLA\/8SZDnv7caNG93BT+SBfwXiHTngi0B5zi2Gf0vL6\/3vfz8ddthhMUgq2tbS5quiJyPj4GzOMgJVIW5WwHgmAlsr2E7hAgZfOQAZeo977lSnn3464dwL9qzxUj8c\/MTqS+4S8Qy4qLlZXp7JrmCVzVcFT05gaDZnAWAqWG0FTGBy8DggqnE8ZowWcsA1d2q8C+eCCy5wrznH8vz111+\/\/zxM7lI5aMBFzc3yOmiqK1ph81XR0+MdnM2ZF5aKVloBs396cJAXTwfwKgtayHiMGi3k\/a65a\/CLiaeo0PLgkS9yw9MDUs\/2vLQYexFzs7zycge+O06br3dxyNNPm7M8zZZ\/rBVVwPiHaFpDwBAwBAwBQ8AQMAQORMAKmAPxMMkQMAQMAUPAEMgbAi1yvFbAtMhpt6QNAUPAEDAEDIF8I2AFTL7nz0ZvCBgChkDzI2AjMASaAQErYJoBdLukIWAIGAKGgCFgCKQhYAVMGn7W2xAwBJofARuBIWAItEAErIBpgZNuKRsChoAhYAgYAnlHwAqYvM+gjb\/5EbARGAKGgCFgCDQ5AlbANDnkdkFDwBAwBAwBQ8AQSEXACphUBJu\/v43AEDAEDAFDwBBocQhYAdPiptwSbukI7Nq1i4YPH074ni9J48ePzz00q1atom984xvEOc6YMaMuJ9YtXLiwTicZ2IHB1q1bpdp4Q8AQqFAE0guYCk3MhmUIGAJxBCZMmED4PiymKVOmxDtUuBUFyKwMbxbbAAAE00lEQVRZs2jMmDFljRTfdzZixAiaPn16Wf2tkyFgCDQtAlbANC3edjVDoKIRwIoFVmfOOussAo8VjQEDBrjVGuh4dQKrGFi9gW3UqFHEKxfwgQ1Joj\/0zMMfxDr4DRkyhEDQ47ooQuAPH+hA8AOxHT7goYMv05IlS6hv377UqVMnVgVbGR\/XQDzExRf8oRPyRmtkCDQmAhY7DQErYNLws96GQG4RuOuuu1xhgg9wFB5cnGzYsIHmzp1LF154Id1www300EMPuZWaSy+9lNAHfrfddhvNnDmTfvvb31JNTU0UAxQazzzzDK1cudLRxo0bCTp0euWVV+hb3\/oWofjAddHCtmzZMqebP38+TZs2jbp27Qp32r17N61du9Zd88QTT3Q6\/vHEE0+4AoZltBgv8gPBH\/Ghx2oTrzwhzyuvvJKwAgMbroW8wBsZAoZA5SJgBUzlzo2NzBBoVATkFtKiRYvqVi66d+9Obdq0oS1bthAKjKFDh7pCB8UACovFixe7caEgwIf+mWee6eTQj+rqaleMwB+EIgLFBvzbt29PnTt3dtfDdaGD\/8CBA914sCKyYMECOuGEE6hbt26uAMK4jjvuOGeHPwirJyikEAsyk8wRBdSgQYPY5FqsxJx\/\/vk0ePBgJ+MHVnEwBvDFJsvOEMg3AlbA5Hv+bPSGQKMi0LNnT1d88GoFCp127drV+5pY5eAYaLECUt8gKDRQ+IDA17e\/9scWF4oVWbxoH5MNAUOgchGwAqZy58ZGZgg0KwJYzdi5cyfNmzfPjQMf+Dgrgg99KLCigZUPbA9BxqoNVlGweiH18H\/yyScJ50qgRwzEQh8fwR8rPdiqQh+s8KDl1ZMXX3yRsJIj+2IlqG3btm7VSOpDPF9\/9OjRB7lg\/BjDQQZTGAKGQEUhYAVMRU2HDcYQqBwEcBj2vvvuo+9973tuC2nOnDl07733Uq9evejmm2+mkSNH0umnn04oHDBqFBEoNrDVJPVY4bjqqqsIW1EoPLAV5CscEAMEf5y3QcGCPih4sJWE+Cgs9PYR+oCwKoPVGfAxQlGGXDBOnI0B8RkgFFjLly93ecVimM0QMASaHwErYJp\/DmwEzYJAy70oCoHZs2eTr4iADjb4ACEUDitWrHCHeLF9hKIGehQZ2AqC7eSTT4bKEfqz\/v777yfeKmI9bKxDDI6J6+G60CGQ9AcPHYoLrPagUIGsCQUPdFi54XjcF3rWYTsL18VYmCAjNxwQ7tKlCyFv9DEyBAyBykXACpjKnRsbmSFgCOxHAEXJBRdc4CQuVJwgfqBAQXGDlRWhzsyiQMKK00UXXZS5jzkaAoZA8yFgBUwzYW+XNQSKggBWOXhVpbFywuoIVkmwSoNCJXQdrOCUOxbExaqRrb6E0DW9IVBZCFgBU1nzYaMxBAwBQ8AQMAQMgTACdRYrYOqgMMYQMAQMAUPAEDAE8oKAFTB5mSkbpyFgCBgChkDzI2AjqBgErICpmKmwgRgChoAhYAgYAoZAVgSsgMmKlPkZAoaAIdD8CNgIDAFDYD8CVsDsB8IaQ8AQMAQMAUPAEMgPAlbA5GeubKSGQPMjYCMwBAwBQ6BCELACpkImwoZhCBgChoAhYAgYAtkRsAImO1bm2fwI2AgMAUPAEDAEDAGHwP8PAAD\/\/zZncDMAAAAGSURBVAMAEA1\/tmQG3CIAAAAASUVORK5CYII=","height":337,"width":560}}
%---
