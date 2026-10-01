%[text] # single\_phase\_inverter (two levels): model initialization (param structure by subsystem)
% Rev. 01, 2026-09-30. Replaces the flat-workspace init_model.m (kept as init_model_legacy.m).
% Model single_phase_inverter.slx: three-phase diode rectifier (library block with thermal model; the plain
% one is commented out) fed by the 400 V grid emulator with FRT, DC link CFi, one single-phase two-level
% full bridge (ideal-switch or MOSFET-with-thermal-model variant, selected at the end of this file), output
% single-phase transformer with RL load, dq PI current control on a virtual dq frame (SOGI / FHT /
% phase-shift filter, Simulink or C caller) or resonant PI, system-identification mode, grid PLL with the
% FHT tracker of the AFE control object.
% Same convention as afe_inv_psm/init_model.m: everything the model needs is in the struct "param",
% one group per subsystem:
%   param.sim        simulation settings (stop time, logging base, thermal model flags, template settings)
%   param.time       timing settings shared by all stages, and the PWM data of the stages not in this model
%   param.spinv      single-phase inverter: .pwr_nom .pwm .hw_in (hardware numbers) .hw (hw_single_phase_inverter_setup
%                    object) .impl .ref .pres_ctrl .sogi .dqvector_pi .ctrl .igbt .mosfet .ideal_switch, plus the
%                    operating mode (system_identification_enable, frequency_set, omega_set, rpi_enable)
%   param.glb_time   timing_setup object (built from param.time and param.spinv.pwm)
%   param.grid       grid emulator inputs;              param.grid.emu     grid_three_phase_emulator object
%   param.frt        FRT test inputs;                   param.frt.data     frt_settings + grid_fault_generator data
%   param.afe        no AFE stage in this model: .ctrl only (ctrl_afe_setup object, its fht_grid tracker is read
%                    by the grid PLL of the inverter control); the AFE PWM data live in param.time
%   param.diode      diode rectifier data (param.diode.rectifier)
%   param.trafo_load output single-phase transformer inputs; param.trafo_load.output_transformer object; rload, lload
%   param.meas       ADC end scale and quantization
%   param.ctrl       control settings common to all modules (time alignment, filters, PI clip release)
%   param.thermal    heatsink inputs;                   param.thermal.heatsink  liquid_cooled_plate_2kw_setup object
%   param.dclink     DC-link stray inductance (Bode check only, not read by the model)
%   param.dev        names of the semiconductor datasets
%
% Naming rule: the leaf name of the old workspace variable is kept (kp_align -> param.ctrl.kp_align), and
% inside param.spinv the single_phase_inverter tag is dropped (single_phase_inverter_pwr_nom ->
% param.spinv.pwr_nom). Library objects keep their name as a field: hwdata.single_phase_inverter ->
% param.spinv.hw, single_phase_inverter_ctrl -> param.spinv.ctrl, sogi -> param.spinv.sogi,
% output_transformer -> param.trafo_load.output_transformer, afe_ctrl -> param.afe.ctrl, frt_data ->
% param.frt.data, grid_emu -> param.grid.emu, filters -> param.ctrl.filters, heatsink -> param.thermal.heatsink,
% diode -> param.diode. The device datasets read by the bridge and by the diode rectifier are those of the
% single-phase inverter stage: mosfet.inv -> param.spinv.mosfet, igbt.inv -> param.spinv.igbt (the rectifier
% takes its snubber and the diode Rth_JH from the IGBT dataset, as in the legacy model).
%
% Clock domains: the PWM generator and the control of this model read the INV domain of the timing object
% (glb_time.fPWM_INV, TRGO_INV_double_update, dead_time_INV, ts_inv, Ns_inv), so param.spinv.pwm is the
% inverter slot of timing_setup. The SINGLE_PHASE_INV slot of the object is not read by this model and keeps
% the legacy values (param.time.*_single_phase_inv); the AFE slot (ts_afe, Ns_afe) is read by the grid
% measurements and by the SOGI / global filters.
%
% Library dependence: the library provides classes and algorithms (timing_setup, hw_single_phase_inverter_setup,
% ctrl_single_phase_inverter_setup, ctrl_afe_setup, grid_three_phase_emulator, single_phase_transformer_setup,
% sogi_filter, device_*_setup, liquid_cooled_plate_2kw_setup...), the model's numbers stay in this file
% (param.spinv.hw_in); single_phase_inverter_hwdata is no longer called. The device datasets (device_igbt_setup...)
% stay in the library: they describe components, not this model. A function that must be frozen or changed
% for this model only goes in ./private with the same name: MATLAB resolves private functions before the
% path, for the files of this folder only, and the list of such overrides is printed at every run (see below).
%
% The last code section ("Transition: legacy workspace names") exports the old names still read by
% single_phase_inverter.slx. migrate_single_phase_inverter.m rewrites the model to param.*; once done, set
% export_legacy_names = false and delete that section.

clear;
[model, options] = init_environment('single_phase_inverter_param'); 
s = tf('s');
param = struct();

% functions overridden for this model: ./private takes precedence over the library
overrides = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
if ~isempty(overrides)
    fprintf('%s init: local overrides in private/: %s\n', model, strjoin({overrides.name}, ', '));
end
clear overrides
%[text] ## Simulation
param.sim.simlength = 1.25;                 % [s] model StopTime
param.sim.tc = 0.5e-6;                      % [s] logging base and model MaxStep (glb_time.tc); FixedStep is not used (ode23tb)
                                            %     same value as the legacy init: ts_afe/200 with fpwm = 10 kHz, single update
param.sim.load_step_time = 1.25;            % [s]
param.sim.transmission_delay = 125e-6*2;    % [s]
param.sim.sst_num_of_modules = 2;
param.sim.time_aux_power_supply_fault = 1e3;    % [s] never, in this run

% one/two modules
param.sim.number_of_modules = 1;
param.sim.enable_two_modules = param.sim.number_of_modules;

% power stage model of the full bridge (see "Enable/disable subsystems" at the end):
%   use_mosfet_thermal_model or use_thermal_model = 1 -> full_bridge_mosfet_based_with_thermal_model
%   both 0                                            -> full_bridge_ideal_switch_based
% The diode rectifier is not selected here: single_phase_inverter.slx has the thermal one active and the
% plain "three phase diode rectifier" commented out.
param.sim.use_mosfet_thermal_model = 0;
param.sim.use_thermal_model = 0;

param.sim.nonlinear_iterations = 3;         % Simscape solver (Solver Configuration of the grid emulator)
if (param.sim.use_mosfet_thermal_model || param.sim.use_thermal_model)
    param.sim.nonlinear_iterations = 5;
end

% white noise power used by the module behaviour settings (clock and PWM phase noise)
param.sim.wnp = 0;

% stages of the shared init template not present in single_phase_inverter.slx (AFE hardware, three-phase
% inverter, PSM and IM drives, DAB, CLLC, PSFBC, ISOP rail, UPQC, battery): see add_template_extras
param.sim.build_template_extras = false;
%[text] ## Timing
param.time.fpwm = 10e3;                             % [Hz] base PWM frequency
param.time.t_measure = param.sim.simlength;         % [s] logging window (legacy init: t_measure = simlength)
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;
param.time.trgo_th_generator = 0.025;

% stages not in this model, needed by the shared timing object only. The AFE slot gives ts_afe (grid
% measurements, SOGI, global filters); trgo = 0 is single update, 1 double update (timing_setup)
param.time.fpwm_afe = param.time.fpwm;
param.time.trgo_afe = 0;                            % single update
param.time.dead_time_afe = 0.3e-6;
param.time.fpwm_single_phase_inv = param.time.fpwm; % SINGLE_PHASE_INV slot, not read by this model (see header)
param.time.trgo_single_phase_inv = 1;               % double update
param.time.dead_time_single_phase_inv = 0.33e-6;
param.time.fpwm_dab = param.time.fpwm;
param.time.trgo_dab = 1;                            % double update
param.time.dead_time_dab = 2e-6;
param.time.fpwm_psfbc = param.time.fpwm;
param.time.trgo_psfbc = 0;                          % single update
param.time.dead_time_psfbc = 3e-6;
param.time.fpwm_isop_rail = param.time.fpwm;
param.time.trgo_isop_rail = 1;                      % double update
param.time.dead_time_isop_rail = 3e-6;
param.time.fpwm_cllc = param.time.fpwm;
param.time.trgo_cllc = 1;                           % double update
param.time.dead_time_cllc = 2e-6;

% tc_factor = ts_afe/tc (timing_setup: ts_afe = 1/fpwm_afe, halved with double update), so that glb_time.tc = param.sim.tc
param.time.tc_factor = 1/param.time.fpwm_afe/(1 + param.time.trgo_afe)/param.sim.tc;
%[text] ### Single-phase inverter PWM (INV slot of the timing object)
param.spinv.pwr_nom = 225e3;                        % [W]
param.spinv.pwm.fpwm = param.time.fpwm;             % [Hz] glb_time.fPWM_INV
param.spinv.pwm.trgo = 0;                           % single update (legacy trgo_inv = 0); glb_time.TRGO_INV_double_update
param.spinv.pwm.dead_time = 0.3e-6;                 % [s] glb_time.dead_time_INV
%[text] ### Global timing object
t = param.time;
param.glb_time = timing_setup(t.fpwm_afe, t.trgo_afe, param.spinv.pwm.fpwm, param.spinv.pwm.trgo, ...
    t.fpwm_single_phase_inv, t.trgo_single_phase_inv, t.fpwm_dab, t.trgo_dab, ...
    t.fpwm_psfbc, t.trgo_psfbc, t.fpwm_isop_rail, t.trgo_isop_rail, t.fpwm_cllc, t.trgo_cllc, ...
    t.t_measure, t.tc_factor, t.tc_decimation, t.delay_pwm, t.dead_time_afe, param.spinv.pwm.dead_time, ...
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
%[text] ## Grid-side control object (PLL tracker)
% No AFE in this model: the grid PLL of the inverter control (phase generator / dq_pll_ccaller /
% first_harmonic_tracker1) reads afe_ctrl.fht_grid.omega_fht, delta_fht, Ld_fht. The gains below are the
% legacy ones, not read by this model (kept so that the object is the same as in the other inits).
param.afe.ctrl = ctrl_afe_setup(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom);
param.afe.ctrl.kp_udc_ctrl = 2;
% resonant PI gains: the legacy init sets the weak-grid pair (0.5 / 18) and then the LVRT pair, which wins
param.afe.ctrl.res_pi.kp_rpi = 0.6;
param.afe.ctrl.res_pi.ki_rpi = 35;
%[text] ## Diode rectifier
% ABB 5SDF 0131Z0401 (slow recovery); the library blocks "three phase diode rectifier" and
% three_phase_diode_rectifier_with_thermal_model read Vf, Rdon and the snubber of param.spinv.igbt
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
%[text] ## Single-phase inverter
%[text] ### Hardware
% The numbers are the model's own (param.spinv.hw_in), the library provides only the setup class
% hw_single_phase_inverter_setup (normalization ubez/ibez/udc_bez, norm.Xbez). Same data as
% single_phase_inverter_hwdata of the library: the three application-voltage sets differ only in the
% nominal frequency and in the braking resistor.
uapp = param.grid.application_voltage;
if uapp == 690
    param.spinv.hw_in.f_nom = 50;                   % [Hz]
    param.spinv.hw_in.Rbrake = 4;                   % [Ohm]
elseif uapp == 480
    param.spinv.hw_in.f_nom = 60;
    param.spinv.hw_in.Rbrake = 2.5;
else
    param.spinv.hw_in.f_nom = 50;
    param.spinv.hw_in.Rbrake = 2.5;
end
param.spinv.hw_in.us_nom = uapp*1.35*0.9/sqrt(2);   % [V] rms output voltage: 90 % of the rectified line voltage, over sqrt(2)
param.spinv.hw_in.is_nom = param.spinv.pwr_nom/param.spinv.hw_in.us_nom;    % [A] rms
param.spinv.hw_in.udc_nom = 2/sqrt(3)*1.1*uapp*sqrt(2);                     % [V] DC-link
param.spinv.hw_in.CFi = 1500e-6 * 4;                % [F] DC-link capacitance; the two banks of the three-level stage
param.spinv.hw_in.CFi1 = 2*param.spinv.hw_in.CFi;
param.spinv.hw_in.CFi2 = 2*param.spinv.hw_in.CFi;
param.spinv.hw_in.RCFi = 1e-3;                      % [Ohm] ESR
param.spinv.hw_in.RCFi1 = param.spinv.hw_in.RCFi/2;
param.spinv.hw_in.RCFi2 = param.spinv.hw_in.RCFi/2;
param.spinv.hw_in.LFu = 500e-6;                     % [H] output filter inductor
param.spinv.hw_in.LFu_cm = 500e-6;                  % [H] common-mode inductor
param.spinv.hw_in.RLFu = 157*0.05*param.spinv.hw_in.LFu;        % [Ohm] 5 % of the reactance at 25 Hz
param.spinv.hw_in.RLFu_cm = 157*0.05*param.spinv.hw_in.LFu_cm;  % [Ohm]
param.spinv.hw_in.CFu = 200e-6;                     % [F] output filter capacitor
param.spinv.hw_in.RCFu = 50e-3;                     % [Ohm] damping resistor

h = param.spinv.hw_in;
param.spinv.hw = hw_single_phase_inverter_setup(uapp, param.spinv.pwr_nom, h.us_nom, h.is_nom, h.f_nom, h.udc_nom, ...
    param.glb_time.fPWM_INV, h.CFi, h.RCFi, h.CFi1, h.RCFi1, h.CFi2, h.RCFi2, h.LFu, h.RLFu, h.LFu_cm, h.RLFu_cm, ...
    h.CFu, h.RCFu, h.Rbrake);
param.spinv.hw.displayInfo();
clear h uapp
%[text] ### Operating mode
%[text] Flags marked (variant) drive Variant Source conditions in the model.
% system identification (open-loop u/f at 300 Hz) versus normal operation
param.spinv.system_identification_enable = 0;       % (variant)
param.spinv.frequency_set = 50;                     % [Hz] (variant: frequency_set ~= grid_emu.fgrid_nom selects the phase generator)
if param.spinv.system_identification_enable
    param.spinv.frequency_set = 300;
end
param.spinv.omega_set = param.spinv.frequency_set*2*pi;     % [rad/s]

param.spinv.rpi_enable = 0;                         % (variant) 1: resonant PI, 0: dq PI on the virtual dq frame
%[text] ### C-caller versus Simulink implementation, virtual dq frame method
param.spinv.impl.use_current_controller_from_ccaller_mod1 = 0;      % (variant)
param.spinv.impl.use_phase_shift_filter_from_ccaller_mod1 = 0;
param.spinv.impl.use_sogi_from_ccaller_mod1 = 1;

% alpha-beta generation for the current control (one of the five)
param.spinv.impl.use_single_phase_inverter_based_FHT = 0;           % (variant)
param.spinv.impl.use_single_phase_inverter_based_SOGI = 0;          % (variant)
param.spinv.impl.use_single_phase_inverter_based_PHSH = 0;          % (variant)
param.spinv.impl.use_single_phase_inverter_based_SOGI_ccaller = 1;  % (variant)
param.spinv.impl.use_single_phase_inverter_based_PHSH_ccaller = 0;  % (variant)

% alpha-beta generation for the system identification (one of the five)
param.spinv.impl.use_system_identification_based_FHT = 0;           % (variant)
param.spinv.impl.use_system_identification_based_SOGI = 0;          % (variant)
param.spinv.impl.use_system_identification_based_PHSH = 0;          % (variant)
param.spinv.impl.use_system_identification_based_SOGI_ccaller = 0;  % (variant)
param.spinv.impl.use_system_identification_based_PHSH_ccaller = 1;  % (variant)
%[text] ### Output current references (pu, steps)
param.spinv.ref.iph_grid_pu_ref_1 = 1/3;
param.spinv.ref.iph_grid_pu_ref_2 = 1/3;
param.spinv.ref.iph_grid_pu_ref_3 = 1/3;
param.spinv.ref.time_step_ref_1 = 0.025;            % [s]
param.spinv.ref.time_step_ref_2 = 0.5;              % [s]
param.spinv.ref.time_step_ref_3 = 1;                % [s]
%[text] ### Resonant PI
ts = param.glb_time.ts_inv;
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
clear w dr
%[text] ### SOGI
% sogi.flt.beta is read by the "pwm filtering for measure" block of the voltage acquisition, kepsilon by the
% SOGI of the dq PI current controller
param.spinv.sogi_delta = 1;
param.spinv.kepsilon = 2;
param.spinv.sogi = sogi_filter(param.spinv.omega_set, param.spinv.sogi_delta, param.spinv.kepsilon, param.glb_time.ts_afe);
%[text] ### dq PI current control
param.spinv.dqvector_pi.kp_inv = 0.5;
param.spinv.dqvector_pi.ki_inv = 45;
param.spinv.dqvector_pi.pi_ctrl = param.spinv.dqvector_pi.kp_inv + param.spinv.dqvector_pi.ki_inv/s;
param.spinv.dqvector_pi.pid_ctrl = c2d(param.spinv.dqvector_pi.pi_ctrl, ts);
param.spinv.dqvector_pi.plant = 1/(s*param.grid.emu.trafo.Ld1 + 1);
param.spinv.dqvector_pi.plantd = c2d(param.spinv.dqvector_pi.plant, ts);
param.spinv.dqvector_pi.G = param.spinv.sogi.fltd.alpha * param.spinv.dqvector_pi.pid_ctrl * param.spinv.dqvector_pi.plantd;
figure; margin(param.spinv.dqvector_pi.G, options); grid on
%[text] ### Control object (resonant PI, virtual dq, FHT load tracker)
param.spinv.ctrl = ctrl_single_phase_inverter_setup(ts, param.spinv.pres_ctrl.omega_set, ...
    param.spinv.dqvector_pi.kp_inv, param.spinv.dqvector_pi.ki_inv, param.spinv.pres_ctrl.kp_rpi, ...
    param.spinv.pres_ctrl.ki_rpi, param.spinv.pres_ctrl.delta_rpi);
clear ts
%[text] ## Output transformer and load
param.trafo_load.name = 'Load Single Phase Transformer';
param.trafo_load.pwr_nom = 225e3;                   % [W]
param.trafo_load.u1_nom = 400;                      % [V]
param.trafo_load.n1 = 50;
param.trafo_load.n2 = 1;
param.trafo_load.u2_nom = param.trafo_load.u1_nom/param.trafo_load.n1*param.trafo_load.n2;   % [V]
param.trafo_load.f_nom = param.spinv.frequency_set; % [Hz] alternative: 50
param.trafo_load.eta = 98;                          % [%]
param.trafo_load.ucc = 5;                           % [%]
param.trafo_load.i1m = 10;                          % [A]
param.trafo_load.p_iron = 2e3;                      % [W]

tl = param.trafo_load;
param.trafo_load.output_transformer = single_phase_transformer_setup(tl.name, tl.pwr_nom, tl.u1_nom, ...
    tl.u2_nom, tl.n1, tl.n2, tl.f_nom, tl.eta, tl.ucc, tl.i1m, tl.p_iron);
clear tl

% RL load on the secondary
param.trafo_load.uload = 2;                         % [V]
param.trafo_load.rload = param.trafo_load.uload / param.trafo_load.output_transformer.i2_nom;   % [Ohm]
param.trafo_load.lload = 250e-6 / param.trafo_load.output_transformer.n12^2;                     % [H]
% alternative: rload = 0.86/m12_load_trafo^2; lload = 3e-3/m12_load_trafo^2;
%[text] ## Power semiconductors
param.dev.used_device_igbt = 'mitsubishi_CM1200DW_24T';
param.dev.used_device_mosfet = 'danfoss_SKM1700MB20R4S2I4';
param.dev.used_device_ideal_switch = 'silicon_high_power_ideal_switch';

% datasets of the single-phase inverter stage, built at fPWM_INV and at the inverter DC-link voltage
% (the legacy init built igbt.inv / mosfet.inv with hwdata.inv.udc_nom of the three-phase inverter: udc
% enters only Csnubber_zvs / Rsnubber_zvs, not read by this model, so the data read here are the same)
param.spinv.igbt = device_igbt_setup(param.dev.used_device_igbt, param.glb_time.fPWM_INV, param.spinv.hw.udc_nom);
param.spinv.mosfet = device_mosfet_setup(param.dev.used_device_mosfet, param.glb_time.fPWM_INV, param.spinv.hw.udc_nom);
param.spinv.ideal_switch = device_ideal_switch_setting(param.dev.used_device_ideal_switch, param.glb_time.fPWM_INV, param.spinv.hw.udc_nom);
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;               % also read as adc11_quantization by the commented-out current acquisition
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;

param.meas.Imax_adc = 1049.835;             % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;

param.meas.Umax_adc = 1500;                 % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%[text] ## Control settings common to all modules
% local time alignment to master time (not read by this model, kept for uniformity)
param.ctrl.kp_align = 0.25;
param.ctrl.ki_align = 18;
param.ctrl.lim_up_align = 0.05;
param.ctrl.lim_down_align = -0.05;

% filters and PI clip release
param.ctrl.CTRPIFF_CLIP_RELEASE = 0.001;
param.ctrl.mavarage_filter_frequency_base_order = 2;   % 2 means 100 Hz, 1 means 50 Hz
param.ctrl.dmavg_filter_enable_time = 0.025;           % [s]
% filters.afe.SOF_50Hz.ss_c is read by the FRT voltage source of the grid emulator
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
param.thermal.Rth_mosfet_HA = param.thermal.Rth_switch_HA;      % [K/W] also the diode-to-water Rth of the rectifier mask (rth_diode_HA_m)
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;       % [K/W]
param.thermal.Tambient = 40;                                    % [degC] water temperature
param.thermal.DThs_init = 0;                                    % [K]

t = param.thermal;
param.thermal.heatsink = liquid_cooled_plate_2kw_setup(t.weight, t.no_weight, t.cp_al, t.heat_capacity_hs, ...
    t.thermal_conductivity_al, t.Rth_switch_HA, t.Rth_mosfet_HA, t.Rth_diode_HA, t.Tambient, t.DThs_init);
clear t
%[text] ### Library workaround: thermal sources read the base workspace
% Needed by single_phase_inverter.slx: its rectifier is the library block
% three_phase_diode_rectifier_with_thermal_model, whose temperature source and initial temperatures still
% read Tambient and DThs_init from the base workspace (12 references; the block has no mask parameter for
% them, so tools/fix_library_thermal_source_refs.m could not fix it on 2026-09-30). Delete these two lines
% once the library block takes them from its mask.
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
param.dclink.ZCFi = 7/s/param.spinv.hw.CFi;
param.dclink.sys_dclink = minreal(param.dclink.ZCFi/(param.dclink.ZCFi + param.dclink.Z_Lstray_dclink));
figure; bode(param.dclink.sys_dclink, param.dclink.Z_Lstray_dclink, options); grid on
%[text] ## Template extras (not used by single\_phase\_inverter.slx)
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
% full-bridge variants of ".../single phase inverter" (paths built on the model name, so that the migrated
% copy works too); the legacy init hard-coded 'single_phase_inverter/...'
bridge = [model '/single_phase_inverter/single phase inverter/'];
if param.sim.use_mosfet_thermal_model || param.sim.use_thermal_model
    set_param([bridge 'full_bridge_mosfet_based_with_thermal_model'], 'Commented', 'off');
    set_param([bridge 'full_bridge_ideal_switch_based'], 'Commented', 'on');
else
    set_param([bridge 'full_bridge_ideal_switch_based'], 'Commented', 'off');
    set_param([bridge 'full_bridge_mosfet_based_with_thermal_model'], 'Commented', 'on');
end
clear bridge
%[text] ## Transition: legacy workspace names
%[text] Old names still read by single\_phase\_inverter.slx (block and mask parameters, library-link instance parameters, Variant Source conditions, model configuration) and by the post-processing scripts. Set export\_legacy\_names = false after migrate\_single\_phase\_inverter.m.
export_legacy_names = false;
if export_legacy_names
    % model configuration (active set "Configuration1"): StopTime = simlength, FixedStep = tc (unused, ode23tb),
    % MaxStep = glb_time.tc; the inactive set "Configuration" reads an undefined Ts, left alone
    simlength = param.sim.simlength;
    tc = param.sim.tc;

    % Solver Configuration of the grid emulator
    nonlinear_iterations = param.sim.nonlinear_iterations;

    % variant conditions
    frequency_set = param.spinv.frequency_set;
    rpi_enable = param.spinv.rpi_enable;
    system_identification_enable = param.spinv.system_identification_enable;
    use_current_controller_from_ccaller_mod1 = param.spinv.impl.use_current_controller_from_ccaller_mod1;
    use_single_phase_inverter_based_FHT = param.spinv.impl.use_single_phase_inverter_based_FHT;
    use_single_phase_inverter_based_SOGI = param.spinv.impl.use_single_phase_inverter_based_SOGI;
    use_single_phase_inverter_based_PHSH = param.spinv.impl.use_single_phase_inverter_based_PHSH;
    use_single_phase_inverter_based_SOGI_ccaller = param.spinv.impl.use_single_phase_inverter_based_SOGI_ccaller;
    use_single_phase_inverter_based_PHSH_ccaller = param.spinv.impl.use_single_phase_inverter_based_PHSH_ccaller;
    use_system_identification_based_FHT = param.spinv.impl.use_system_identification_based_FHT;
    use_system_identification_based_SOGI = param.spinv.impl.use_system_identification_based_SOGI;
    use_system_identification_based_PHSH = param.spinv.impl.use_system_identification_based_PHSH;
    use_system_identification_based_SOGI_ccaller = param.spinv.impl.use_system_identification_based_SOGI_ccaller;
    use_system_identification_based_PHSH_ccaller = param.spinv.impl.use_system_identification_based_PHSH_ccaller;

    % block parameters
    CTRPIFF_CLIP_RELEASE = param.ctrl.CTRPIFF_CLIP_RELEASE;
    adc12_quantization = param.meas.adc12_quantization;
    adc11_quantization = param.meas.adc_quantization;   % commented-out acquisition_and_scaling_current (never defined by the legacy init)
    Rth_mosfet_HA = param.thermal.Rth_mosfet_HA;        % mask rth_diode_HA_m of the thermal diode rectifier
    kepsilon = param.spinv.kepsilon;
    omega_set = param.spinv.omega_set;
    iph_grid_pu_ref_1 = param.spinv.ref.iph_grid_pu_ref_1;
    iph_grid_pu_ref_2 = param.spinv.ref.iph_grid_pu_ref_2;
    iph_grid_pu_ref_3 = param.spinv.ref.iph_grid_pu_ref_3;
    time_step_ref_1 = param.spinv.ref.time_step_ref_1;
    time_step_ref_2 = param.spinv.ref.time_step_ref_2;
    time_step_ref_3 = param.spinv.ref.time_step_ref_3;
    rload = param.trafo_load.rload;
    lload = param.trafo_load.lload;

    % objects and structs (glb_time, grid_emu.fgrid_nom and frequency_set also in plotting_results_*.m;
    % spectrum.m reads tc / Nc and power_losses_calculation.m Nc / tc / decimation_tc / Ns_inv / ts_inv flat:
    % never defined by the legacy init, stale)
    glb_time = param.glb_time;
    grid_emu = param.grid.emu;
    frt_data = param.frt.data;
    afe_ctrl = param.afe.ctrl;
    filters = param.ctrl.filters;
    heatsink = param.thermal.heatsink;
    hwdata.single_phase_inverter = param.spinv.hw;
    igbt.inv = param.spinv.igbt;
    mosfet.inv = param.spinv.mosfet;
    single_phase_inverter_ctrl = param.spinv.ctrl;
    sogi = param.spinv.sogi;
    output_transformer = param.trafo_load.output_transformer;
    diode = param.diode;
end
%[text] ## Local functions
function param = add_template_extras(param, options)
% Parts of the shared init template not used by single_phase_inverter.slx, kept so that nothing of the
% legacy init_model.m is lost. Enabled by param.sim.build_template_extras = true.
% Same convention: one group per stage (param.afe hardware / PLL / references / behaviour, param.inv,
% param.psm, param.load, param.im, param.dab, param.three_phase_dab, param.psfbc, param.cllc,
% param.isop_rail, param.dcdc, param.upqc, param.battery), each with .hw, .ctrl, devices. The hardware
% data of these stages are still taken from the library data functions (three_phase_afe_hwdata...):
% move them into hw_in blocks here when a model of this kind is migrated.

s = tf('s');
ts = param.glb_time.ts_inv;
dev = param.dev;
uapp = param.grid.application_voltage;

% ---------------------------------------------------------------- AFE (hardware, PLL, references, behaviour)
param.afe.pwr_nom = 225e3;
param.afe.hw = three_phase_afe_hwdata(uapp, param.afe.pwr_nom, param.glb_time.fPWM_AFE);

param.afe.pll.use_dq_pll_fht_pll = 1;
param.afe.pll.use_dq_pll_fht_simulink_pll = 0;
param.afe.pll.use_dq_pll_mod1 = 0;
param.afe.pll.use_dq_pll_ccaller_mod1 = 0;
param.afe.pll.use_dq_pll_ccaller_mod2 = 0;
param.afe.pll.use_dq_pll_mode1 = param.afe.pll.use_dq_pll_mod1;                 % simulink dqPLL
param.afe.pll.use_dq_pll_mode2 = param.afe.pll.use_dq_pll_ccaller_mod1;         % ccaller dqPLL
param.afe.pll.use_dq_pll_mode3 = param.afe.pll.use_dq_pll_fht_simulink_pll;     % fht simulink dqPLL
param.afe.pll.use_dq_pll_mode4 = param.afe.pll.use_dq_pll_fht_pll;              % fht ccaller dqPLL
param.afe.pll.use_dq_pll_mode1_modn = 0;
param.afe.pll.use_dq_pll_mode2_modn = 0;
param.afe.pll.use_dq_pll_mode3_modn = 1;
param.afe.pll.use_dq_pll_mode4_modn = 0;

param.afe.impl.use_moving_average_from_ccaller_mod1 = 1;
param.afe.impl.use_moving_average_from_ccaller_mod2 = 0;
param.afe.impl.use_moving_average_from_ccaller_mod3 = 0;
param.afe.impl.use_moving_average_from_ccaller_mod4 = 0;

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
param.afe.ref.time_i_react_pos_ref_1 = 0;
param.afe.ref.time_i_react_pos_ref_2 = 0;
param.afe.ref.i_react_pos_ref_1 = 0;
param.afe.ref.i_react_pos_ref_2 = 0;
param.afe.ref.i_react_pos_ref_3 = 0;
if param.afe.ref.enable_i_react_pos_steps
    param.afe.ref.time_i_react_pos_ref_1 = param.frt.data.start_time_LVRT + param.frt.data.error_length + 0.335;
    param.afe.ref.time_i_react_pos_ref_2 = param.afe.ref.time_i_react_pos_ref_1 + 0.5;
    param.afe.ref.i_react_pos_ref_2 = -param.afe.ref.ixi_ref_mod1*tan(acos(0.95));   % cos(phi) = 0.95
    param.afe.ref.i_react_pos_ref_3 =  param.afe.ref.ixi_ref_mod1*tan(acos(0.95));   % cos(phi) = 0.95
end

param.afe.behav.time_gain_module_1 = 1.0;
param.afe.behav.time_gain_module_2 = 1.0;
param.afe.behav.time_gain_module_3 = 0.9988;
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

% ---------------------------------------------------------------- three-phase inverter and machine drive
param.inv.pwr_nom = 225e3;
param.inv.hw = three_phase_inverter_hwdata(uapp, param.inv.pwr_nom, param.glb_time.fPWM_INV);
param.inv.behav.time_gain_module_1 = 1.0005;        % legacy init assigns 1.0 first, then 1.0005
param.inv.behav.time_gain_module_2 = 1.0;
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
param.psm.ctrl.kp_i = 0.25;
param.psm.ctrl.ki_i = 35;

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

% ---------------------------------------------------------------- induction machine
param.im.machine = im_calculus();
param.im.u_im_scale = 2/3*param.inv.hw.udc_nom/param.im.machine.ubez;
param.im.u_im_scale_ekf = (2/3)^2 * param.inv.hw.udc_nom/param.im.machine.ubez;
param.im.ctrl = ctrl_im_setup(ts, param.im.machine.omega_bez, param.im.u_im_scale, param.im.machine.Jm_norm);
param.im.ctrl.ekf = ekf_im_setup(param.im.machine.alpha_norm, param.im.machine.beta_norm, param.im.machine.gamma_norm, ...
    param.im.machine.sigma_norm, param.im.machine.mu_norm, param.im.machine.Lm_norm, param.im.machine.Jm_norm, ts);

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
param.dab.pwr_nom = 225e3;
param.dab.fres = param.glb_time.fPWM_DAB/5;
param.dab.hw = single_phase_dab_hwdata(uapp, param.dab.pwr_nom, param.glb_time.fPWM_DAB, param.dab.fres);
param.three_phase_dab.pwr_nom = param.dab.pwr_nom;  % legacy init passes dab_pwr_nom
param.three_phase_dab.hw = three_phase_dab_hwdata(uapp, param.three_phase_dab.pwr_nom, param.glb_time.fPWM_DAB, param.dab.fres);
param.psfbc.pwr_nom = 275e3;
param.psfbc.hw = single_phase_psfbc_hwdata(uapp, param.psfbc.pwr_nom, param.glb_time.fPWM_PSFBC);
param.cllc.pwr_nom = 225e3;
param.cllc.fres = param.glb_time.fPWM_CLLC*1.2;
param.cllc.hw = single_phase_cllc_hwdata(uapp, param.dab.pwr_nom, param.glb_time.fPWM_CLLC, param.cllc.fres);  % legacy init passes dab_pwr_nom (same value)
% modification of the legacy init (active there)
param.afe.hw.CFi = 2 * param.psfbc.hw.Cdc_dc1;

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

% ---------------------------------------------------------------- devices of the other stages
param.afe.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.inv.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
param.dab.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_DAB, param.dab.hw.udc1_nom);
param.psfbc.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom);
param.cllc.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_CLLC, param.cllc.hw.udc1_nom);
param.isop_rail.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_ISOP_RAIL, param.isop_rail.hw.udc1_nom);

param.afe.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.inv.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
param.dab.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_DAB, param.dab.hw.udc1_nom);
param.psfbc.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom);
param.cllc.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_CLLC, param.cllc.hw.udc1_nom);
param.isop_rail.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_ISOP_RAIL, param.isop_rail.hw.udc1_nom);

param.afe.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.inv.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
param.dab.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_DAB, param.dab.hw.udc1_nom);
param.psfbc.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom);
param.cllc.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_CLLC, param.cllc.hw.udc1_nom);
param.isop_rail.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_ISOP_RAIL, param.isop_rail.hw.udc1_nom);

% ---------------------------------------------------------------- lithium-ion battery
param.battery.nominal_battery_voltage_1 = param.dab.hw.udc1_bez;   % alternatives: param.cllc.hw.udc1_bez, param.afe.hw.udc_nom
param.battery.nominal_battery_voltage_2 = param.dab.hw.udc2_bez;   % alternatives: param.cllc.hw.udc2_bez, param.afe.hw.udc_nom
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

% special settings for simulation (legacy init of this model: C1/50)
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_2.R0 = param.battery.pack_2.R0/2;
param.battery.pack_2.R1 = param.battery.pack_2.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/50;
param.battery.pack_2.C1 = param.battery.pack_2.C1/50;
end
