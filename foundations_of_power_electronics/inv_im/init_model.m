%[text] # inv\_im: model initialization (param structure by subsystem)
% Rev. 01, 2026-10-05. Replaces the flat-workspace init_model.m (kept as init_model_legacy.m).
% Model inv_im.slx: one three-phase two-level inverter (IGBT with advanced thermal model, MOSFET with
% thermal model or ideal switch) fed by a lithium-ion battery through the DC link, with the three-phase
% output inductor, driving the induction machine with field-oriented current control and EKF rotor flux
% observer; the shaft is held by a speed source (torque curve) or loaded by inertia and torque step
% (speed control). Same convention as afe_inv_psm/init_model.m: everything the model needs is in the
% struct "param", one group per subsystem:
%   param.sim       simulation settings (stop time, logging base, module count, power stage model choice)
%   param.time      timing settings shared by all stages, and the PWM data of the stages not in this model
%   param.glb_time  timing_setup object (built from param.time, param.afe.pwm, param.inv.pwm)
%   param.afe       no AFE stage in this model: .pwm (first clock domain of the timing object: ts_afe and
%                   the logging base tc derive from it), .behav.time_gain_module_1 (clock time_afe1),
%                   .hw_in / .hw (hw_afe_three_phase_setup: DC-link capacitors and nominal voltage, read by
%                   dc_source/dclink)
%   param.inv       machine-side inverter: .application_voltage .pwr_nom .pwm .hw_in (hardware numbers)
%                   .hw (hw_inverter_three_phase_setup) .behav .igbt .mosfet .ideal_switch
%   param.im        induction machine and its control: .machine_in (machine numbers) .machine (im_setup)
%                   .mode .impl .ctrl (ctrl_im_setup, .ctrl.ekf ekf_im_setup) and the voltage scaling
%   param.psm       wind generator of the torque curve: .machine_in .machine (pmsm_setup) .n_sys; the model
%                   reads only psm.rpm_nom and psm.tau_bez (torque_reference)
%   param.load      torque curve, speed reference, load torque, external inertia
%   param.battery   battery inputs;                    param.battery.pack_1 / .pack_2  lithium_ion_battery_setup structs
%   param.meas      ADC end scale and quantization
%   param.ctrl      control settings common to all modules (time alignment, filters, PI clip release)
%   param.thermal   heatsink inputs;                   param.thermal.heatsink  liquid_cooled_plate_2kw_setup object
%   param.dclink    DC-link stray inductance (Bode check only, not read by the model)
%   param.dev       names of the semiconductor datasets
%
% Naming rule: the leaf name of the old workspace variable is kept (kp_align -> param.ctrl.kp_align),
% except that inside param.afe, param.inv and param.im the afe_/inv_/im_ tag becomes redundant and is
% dropped (fpwm_inv -> param.inv.pwm.fpwm, inv_pwm_phase_shift_mod1 -> param.inv.behav.pwm_phase_shift_mod1,
% time_gain_afe_module_1 -> param.afe.behav.time_gain_module_1; u_im_scale_ekf keeps its name).
% Library objects keep their name as a field: hwdata.inv -> param.inv.hw, hwdata.afe -> param.afe.hw,
% im -> param.im.machine, im_ctrl -> param.im.ctrl, psm -> param.psm.machine, igbt.inv -> param.inv.igbt,
% mosfet.inv -> param.inv.mosfet, ideal_switch.inv -> param.inv.ideal_switch,
% lithium_ion_battery_1 -> param.battery.pack_1, filters -> param.ctrl.filters,
% heatsink -> param.thermal.heatsink, glb_time -> param.glb_time.
% Three references of inv_im.slx are corrected by the mapping (see migrate_inv_im.m):
%   - the EKF of the induction machine reads psm_ctrl.ekf.C_ekf: mapped to param.im.ctrl.ekf.C_ekf, the
%     same constant [1 0 0 0 0 0; 0 1 0 0 0 0] of ekf_im_setup;
%   - the two thermal bridges read weigth (typo, never defined by the legacy init) for the heatsink mass:
%     mapped to param.thermal.weight;
%   - the MOSFET bridge reads mosfet.inv.Rth_switch_JH, a property device_mosfet_setting does not have:
%     mapped to param.inv.mosfet.Rth_mosfet_JH (junction-to-heatsink resistance of the MOSFET dataset).
%
% Library dependence (2026-10-05): the library provides classes and algorithms (timing_setup,
% hw_afe_three_phase_setup, hw_inverter_three_phase_setup, im_setup, pmsm_setup, ctrl_im_setup,
% ekf_im_setup, device_*_setup, lithium_ion_battery_setup, liquid_cooled_plate_2kw_setup...), the model's
% numbers stay in this file (param.afe.hw_in, param.inv.hw_in, param.im.machine_in, param.psm.machine_in,
% the torque curve in param.load, the battery voltages); the *_hwdata, im_calculus, psm_calculus and
% torque-curve functions of the library are no longer called (the template extras still use the *_hwdata
% of the stages not in this model). The device datasets (device_igbt_setup...) stay in the library: they
% describe components, not this model. A function that must be frozen or changed for this model only goes
% in ./private with the same name: MATLAB resolves private functions before the path, for the files of
% this folder only, and the list of such overrides is printed at every run (see below).
%
% inv_im_param.slx is inv_im.slx rewritten to param.* by migrate_inv_im.m. The section "Transition:
% legacy workspace names" exports the old names still read by inv_im.slx: to run the legacy model, set
% the model name below to 'inv_im' and export_legacy_names = true. Once inv_im_param.slx is adopted,
% delete that section and inv_im.slx.

clear;
[model, options] = init_environment('inv_im_param');   % 'inv_im' with export_legacy_names = true
s = tf('s');
param = struct();

% functions overridden for this model: ./private takes precedence over the library
overrides = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
if ~isempty(overrides)
    fprintf('%s init: local overrides in private/: %s\n', model, strjoin({overrides.name}, ', '));
end
clear overrides
%[text] ## Simulation
param.sim.simlength = 10;                   % [s] model StopTime
param.sim.tc = 0.625e-6;                    % [s] logging base and model MaxStep (glb_time.tc); FixedStep is not used (ode23tb)
                                            %     same value as the legacy init: ts_afe/200 with fpwm = 4 kHz, double update
param.sim.load_step_time = 1.25;            % [s]
param.sim.transmission_delay = 125e-6*2;    % [s]
param.sim.sst_num_of_modules = 2;
param.sim.time_aux_power_supply_fault = 1e3;    % [s] never, in this run

% one/two modules
param.sim.number_of_modules = 1;
param.sim.enable_two_modules = param.sim.number_of_modules;

% power stage model: MOSFET with thermal model, IGBT with (advanced) thermal model, or ideal switch
param.sim.use_mosfet_thermal_model = 0;
param.sim.use_thermal_model = 0;

param.sim.nonlinear_iterations = 3;         % Simscape solver (Solver Configuration of the battery network)
if (param.sim.use_mosfet_thermal_model || param.sim.use_thermal_model)
    param.sim.nonlinear_iterations = 5;     % Simscape solver
end

% white noise power used by the module behaviour settings (clock and PWM phase noise)
param.sim.wnp = 0;

% stages of the shared init template not present in inv_im.slx (grid emulator and FRT, AFE control,
% PSM drive control, DAB, CLLC, PSFBC, ISOP rail, single-phase inverter, UPQC, load transformer):
% see add_template_extras at the end
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
%[text] ### Inverter PWM and first clock domain
param.inv.application_voltage = 690;                % [V] 690, 480 or 400: selects the data sets of inverter, DC link and battery
param.inv.pwr_nom = 250e3;                          % [W]
param.inv.pwm.fpwm = param.time.fpwm;               % [Hz]
param.inv.pwm.trgo = 1;                             % double update
param.inv.pwm.dead_time = 3e-6;                     % [s]

% no AFE in this model: the timing object still needs the first domain (ts_afe, from which the logging
% base tc derives, Ns_afe, and the clock time_afe1)
param.afe.pwr_nom = 250e3;                          % [W]
param.afe.pwm.fpwm = param.time.fpwm;               % [Hz]
param.afe.pwm.trgo = 1;                             % double update
param.afe.pwm.dead_time = 3e-6;                     % [s]
%[text] ### Global timing object
t = param.time;
param.glb_time = timing_setup(param.afe.pwm.fpwm, param.afe.pwm.trgo, param.inv.pwm.fpwm, param.inv.pwm.trgo, ...
    t.fpwm_single_phase_inv, t.trgo_single_phase_inv, t.fpwm_dab, t.trgo_dab, ...
    t.fpwm_psfbc, t.trgo_psfbc, t.fpwm_isop_rail, t.trgo_isop_rail, t.fpwm_cllc, t.trgo_cllc, ...
    t.t_measure, t.tc_factor, t.tc_decimation, t.delay_pwm, param.afe.pwm.dead_time, param.inv.pwm.dead_time, ...
    t.dead_time_single_phase_inv, t.dead_time_dab, t.dead_time_psfbc, t.dead_time_isop_rail, t.dead_time_cllc);
clear t
%[text] ### Clocks of the "global timing" subsystem (clock mismatch)
% time_afe1 and time_inv1, read with the same names as in afe_inv_psm
param.afe.behav.time_gain_module_1 = 1.0002;
%[text] ## DC link
% The model reads the DC-link capacitors and the nominal voltage of the AFE hardware object (block
% dc_source/dclink): the numbers are the model's own (param.afe.hw_in), the library provides only the
% setup class hw_afe_three_phase_setup. Same data sets as three_phase_afe_hwdata of the library, selected
% by the application voltage; DC-link capacitors and their ESR are the same for all.
if param.inv.application_voltage == 690
    param.afe.hw_in.name = 'afe690V';
    param.afe.hw_in.uafe_nom = 690;                 % [V] line-to-line
    param.afe.hw_in.iafe_freq = 50;                 % [Hz]
    param.afe.hw_in.udc = 1070;                     % [V] DC-link
    param.afe.hw_in.LFu = 500e-6;                   % [H] grid-side filter inductor
    param.afe.hw_in.LFu_cm = 500e-6;                % [H] common-mode inductor
    param.afe.hw_in.CFu = 200e-6;                   % [F] filter capacitor
    param.afe.hw_in.Rbrake = 4;                     % [Ohm]
elseif param.inv.application_voltage == 480
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
% DC-link capacitance; the legacy init then set hwdata.afe.CFi = 2*hwdata.psfbc.Cdc_dc1 (7.2e-3 at 690 V,
% 2.4e-3 at 480/400 V): CFi is read only by the Bode check of param.dclink, the model reads CFi1 and CFi2
param.afe.hw_in.CFi = 7.2e-3;                       % [F]
param.afe.hw_in.CFi1 = 2*param.afe.hw_in.CFi;       % [F] the two banks of the split DC link
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
param.afe.hw.displayInfo(); %[output:22e63205]
clear h
%[text] ## Inverter
%[text] ### Hardware
% Same data sets as three_phase_inverter_hwdata of the library (the 480 V set keeps the 400 V machine
% side with a 750 V DC link), class hw_inverter_three_phase_setup for the normalization.
if param.inv.application_voltage == 690
    param.inv.hw_in.name = 'inv690V_250kW';
    param.inv.hw_in.u_inv_nom = 550;                % [V] machine-side line-to-line
    param.inv.hw_in.i_inv_nom = 370;                % [A]
    param.inv.hw_in.udc = 1070;                     % [V] DC-link
    param.inv.hw_in.LFi = 230e-6;                   % [H] machine-side filter inductor
    param.inv.hw_in.LFi_cm = 230e-6;                % [H] common-mode inductor
elseif param.inv.application_voltage == 480
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
param.inv.hw_in.i_inv_freq = 15.6;                  % [Hz] nominal machine frequency of the data set
param.inv.hw_in.RLFi = 157*0.05*param.inv.hw_in.LFi;        % [Ohm]
param.inv.hw_in.RLFi_cm = 157*0.05*param.inv.hw_in.LFi_cm;  % [Ohm]

h = param.inv.hw_in;
param.inv.hw = hw_inverter_three_phase_setup(h.name, param.inv.pwr_nom, h.u_inv_nom, h.i_inv_nom, h.i_inv_freq, ...
    h.udc, param.glb_time.fPWM_INV, h.LFi, h.RLFi, h.LFi_cm, h.RLFi_cm);
param.inv.hw.displayInfo(); %[output:0e10b819]
clear h
%[text] ### Module behaviour (clock mismatch, noise, PWM phase shift)
param.inv.behav.time_gain_module_1 = 1.0005;
param.inv.behav.time_gain_module_2 = 1.001;         % clock time_inv1

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
param.dev.used_device_igbt = 'mitsubishi_CM1200DW_34T';        % alternative: 'infineon_FF1200R17IP5'
param.dev.used_device_mosfet = 'danfoss_SKM1700MB20R4S2I4';
param.dev.used_device_ideal_switch = 'silicon_high_power_ideal_switch';

param.inv.igbt = device_igbt_setup(param.dev.used_device_igbt, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
param.inv.mosfet = device_mosfet_setup(param.dev.used_device_mosfet, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
param.inv.ideal_switch = device_ideal_switch_setting(param.dev.used_device_ideal_switch, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
%[text] ## Induction machine and its control
%[text] ### Machine
% Machine data of im_calculus of the library (ABB M3BP 355MLB 6, 261 kW, 460 V, 60 Hz), class im_setup:
% nameplate, no-load test and locked-rotor test; im_setup iterates Rr until the rated torque is matched.
% Alternative data set, commented out in im_calculus: GM355L6-250kW, 400 V, 432 A, 50 Hz, 992 rpm,
% i_no_load = 135 A, locked rotor at u_nom with i_cc = 3084 A and 50 Hz.
param.im.machine_in.name = 'ABB M3BP 355MLB 6 261kW';
param.im.machine_in.pwr_nom = 261e3;                % [W]
param.im.machine_in.u_nom = 460;                    % [V] line-to-line
param.im.machine_in.i_nom = 411;                    % [A]
param.im.machine_in.freq_nom = 60;                  % [Hz]
param.im.machine_in.rpm_load = 1195;                % [rpm] at rated load
param.im.machine_in.number_poles = 6;
param.im.machine_in.eta = 0.95;                     % [pu]
param.im.machine_in.cosphi = 0.88;
param.im.machine_in.Rs = 2e-3;                      % [Ohm] stator phase resistance
param.im.machine_in.u_no_load = param.im.machine_in.u_nom;      % [V] no-load test
param.im.machine_in.i_no_load = 180;                % [A]
param.im.machine_in.f_no_load = param.im.machine_in.freq_nom;   % [Hz]
param.im.machine_in.u_cc = 77;                      % [V] locked-rotor test
param.im.machine_in.i_cc = 470;                     % [A]
param.im.machine_in.f_cc = 50;                      % [Hz]
param.im.machine_in.Jm = 5;                         % [kg m^2]
param.im.machine_in.load_friction_factor = 0.75;    % [pu]

m = param.im.machine_in;
param.im.machine = im_setup(m.name, m.pwr_nom, m.u_nom, m.i_nom, m.freq_nom, m.rpm_load, m.number_poles, ...
    m.eta, m.cosphi, m.Rs, m.u_no_load, m.i_no_load, m.f_no_load, m.u_cc, m.i_cc, m.f_cc, m.Jm, m.load_friction_factor);
param.im.machine.displayInfo(); %[output:39e0b4d4]
clear m
%[text] ### Motor voltage to Udc scaling
param.im.u_im_scale = 2/3*param.inv.hw.udc_nom/param.im.machine.ubez;
param.im.u_im_scale_ekf = (2/3)^2 * param.inv.hw.udc_nom/param.im.machine.ubez;
%[text] ### Control modes
%[text] Flags marked (variant) drive Variant Source conditions in the model.
param.im.mode.use_torque_curve = 0;                                         % (variant) energy production
param.im.mode.use_speed_control = 1 - param.im.mode.use_torque_curve;       % (variant) drive
param.im.mode.use_im_encoder = 1;
param.im.mode.use_load_estimator = 0;
param.im.mode.use_estimator_from_mb = 0;                                    % mb: model based
param.im.mode.use_motor_speed_control_mode = 0;
param.im.mode.motor_torque_mode = 1 - param.im.mode.use_motor_speed_control_mode;   % torque curve for wind application
param.im.mode.time_start_motor_control = 0.25;                              % [s]
%[text] ### C-caller versus Simulink implementation
param.im.impl.use_current_controller_from_simulink_module_1 = 1;            % (variant)
param.im.impl.use_current_controller_from_ccaller_module_1 = 0;             % (variant)
param.im.impl.use_current_controller_from_simulink_module_2 = 0;
param.im.impl.use_current_controller_from_ccaller_module_2 = 0;
%[text] ### Control with EKF based rotor flux observer
param.im.ctrl = ctrl_im_setup(param.glb_time.ts_inv, param.im.machine.omega_bez, param.im.u_im_scale, param.im.machine.Jm_norm);
param.im.ctrl.ekf = ekf_im_setup(param.im.machine.alpha_norm, param.im.machine.beta_norm, param.im.machine.gamma_norm, ... %[output:group:8dae76a6] %[output:8188f2d4]
    param.im.machine.sigma_norm, param.im.machine.mu_norm, param.im.machine.Lm_norm, param.im.machine.Jm_norm, param.glb_time.ts_inv); %[output:group:8dae76a6] %[output:8188f2d4]
%[text] ## Wind generator of the torque curve
% The torque curve of param.load is the one of the 1.6 MW wind generator of afe_inv_psm (psm_calculus of
% the library, class pmsm_setup): the torque_reference block of the model scales the speed with
% psm.rpm_nom and normalizes the curve torque with psm.tau_bez, so the curve is applied in per unit to
% the induction machine. Nothing else of this object is read by inv_im.slx.
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
param.psm.machine.displayInfo(); %[output:0e2de209]
clear m
param.psm.n_sys = param.psm.machine.number_of_systems;
%[text] ## Mechanical load, speed reference and torque curve
param.load.b = param.psm.machine.load_friction_m;   % not read by the model
param.load.external_load_inertia = 1;       % [kg m^2] alternative: 6*param.psm.machine.Jm_m

% energy production: torque curve of the 1.5 MW generic generator (was the library script
% n_sys_generic_1M5W_torque_curve), total torque divided among the n_sys systems
param.load.rpm_base = [6 7 8 9 10 11 12 13 14 15 16 17 17.8 18.0 18.2];                        % [rpm]
param.load.torque_base = 1e3/param.psm.n_sys*[50 75 100 125 150 175 200 225 275 350 450 600 750 885 885];   % [N*m]
param.load.torque_overload_factor = 1;

% drive application: speed reference and load torque
param.load.rpm_sim = 17.8;                  % alternatives: 3000, 15.2
param.load.omega_m_sim = param.psm.machine.omega_m_bez;
param.load.omega_sim = param.load.omega_m_sim*param.psm.machine.number_poles/2;
param.load.tau_load_sim = param.psm.machine.tau_bez/5;  % [N*m]
param.load.b_square = 0;
%[text] ## Battery
% The battery block of the model reads pack_1; pack_2 is built as in the legacy init, not read.
% Battery voltage: the legacy init took hwdata.dab.udc1_bez / udc2_bez, the DC voltage of the library DAB
% data set selected by the application voltage (single_phase_dab_hwdata: 1200 V for 690 V, 800 V otherwise).
if param.inv.application_voltage == 690
    param.battery.nominal_battery_voltage_1 = 1200;     % [V] alternative of the legacy init: param.afe.hw.udc_nom
    param.battery.nominal_battery_voltage_2 = 1200;     % [V]
else
    param.battery.nominal_battery_voltage_1 = 800;      % [V]
    param.battery.nominal_battery_voltage_2 = 800;      % [V]
end
param.battery.nominal_battery_power = 250e3;            % [W]
param.battery.initial_battery_soc = 0.85;
% lithium_ion_battery_setup takes the charge capacity [Ah] as third argument (the legacy init used the old
% four-argument signature, whose library version had Q = 50 Ah fixed). ASSUMPTION, as in afe_inv_psm and
% resonant_cllc: 1C rating, capacity = rated current x 1 h
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;
param.battery.charge_capacity_2 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_2;

b = param.battery;
param.battery.pack_1 = lithium_ion_battery_setup(b.nominal_battery_voltage_1, b.nominal_battery_power, ... %[output:group:23cdc646] %[output:281f8d6c]
    b.charge_capacity_1, b.initial_battery_soc, param.glb_time.ts_dab); %[output:group:23cdc646] %[output:281f8d6c]
param.battery.pack_2 = lithium_ion_battery_setup(b.nominal_battery_voltage_2, b.nominal_battery_power, ... %[output:group:2e3e4406] %[output:803f6bd8]
    b.charge_capacity_2, b.initial_battery_soc, param.glb_time.ts_dab); %[output:group:2e3e4406] %[output:803f6bd8]
clear b

% special settings for simulation
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_2.R0 = param.battery.pack_2.R0/2;
param.battery.pack_2.R1 = param.battery.pack_2.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/50;
param.battery.pack_2.C1 = param.battery.pack_2.C1/50;
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
% water in/out rise max 5 K with 2 kW total losses.
% The two thermal bridges of the model read the flat inputs below (weight, cp_al, Rth_*_HA, Tambient,
% DThs_init), not the heatsink object.
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
% Kept as in afe_inv_psm. The bridges of the library used here (three_phase_inverter_igbt_advanced_thermal_model,
% three_phase_inverter_mosfet_based_with_thermal_model) read Tambient_m / DThs_init_m since the library fix
% of 2026-09-30 (tools/fix_library_thermal_source_refs.m): the two flat names are needed only with an older
% copy of the library, and by power_losses_calculus.m (Tambient). Delete this section once not needed.
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
figure; bode(param.dclink.sys_dclink, param.dclink.Z_Lstray_dclink, options); grid on %[output:66f27b22]
%[text] ## Template extras (not used by inv\_im.slx)
if param.sim.build_template_extras %[output:group:92e2561d]
    param = add_template_extras(param, options); %[output:4aa8f07a] %[output:29845dda] %[output:51b29746]
end %[output:group:92e2561d]
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
% power stage model: MOSFET thermal, IGBT advanced thermal or ideal switch (the other two are commented out)
if param.sim.use_mosfet_thermal_model
    active_bridge = 'three_phase_inverter_mosfet_based_with_thermal_model';
elseif param.sim.use_thermal_model
    active_bridge = 'three_phase_inverter_igbt_adv_thermal_model';
else
    active_bridge = 'three_phase_inverter_ideal_switch_based_model';
end
bridge_models = {'three_phase_inverter_mosfet_based_with_thermal_model', ...
                 'three_phase_inverter_igbt_adv_thermal_model', ...
                 'three_phase_inverter_ideal_switch_based_model'};
bridge_path = [model '/inv_im_mod1/inverter/inverter/'];
commented = {'on', 'off'};                  % index 2 (off) for the active bridge
for ib = 1:numel(bridge_models)
    set_param([bridge_path bridge_models{ib}], 'Commented', ...
        commented{1 + strcmp(bridge_models{ib}, active_bridge)});
end
clear active_bridge bridge_models bridge_path commented ib

% load: torque curve (energy production, speed held by fixed_speed_setting) or inertia with load torque (drive)
if param.im.mode.use_torque_curve
    set_param([model '/fixed_speed_setting'], 'Commented', 'off');
    set_param([model '/motor_load_setting'], 'Commented', 'on');
else
    set_param([model '/fixed_speed_setting'], 'Commented', 'on');
    set_param([model '/motor_load_setting'], 'Commented', 'off');
end
%[text] ## Transition: legacy workspace names
%[text] Old names read by inv\_im.slx (block and mask parameters, library-link instance parameters, Variant Source conditions, model configuration), from the model XML on 2026-10-05: they are the left column of the map in migrate\_inv\_im.m. inv\_im\_param.slx reads param.\* only.
export_legacy_names = false;
if export_legacy_names
    % model configuration: StopTime = simlength, FixedStep = tc (unused, ode23tb), MaxStep = glb_time.tc
    simlength = param.sim.simlength;
    tc = param.sim.tc;

    % Solver Configuration of the battery network
    nonlinear_iterations = param.sim.nonlinear_iterations;

    % variant conditions
    use_torque_curve = param.im.mode.use_torque_curve;
    use_speed_control = param.im.mode.use_speed_control;
    use_current_controller_from_simulink_module_1 = param.im.impl.use_current_controller_from_simulink_module_1;
    use_current_controller_from_ccaller_module_1 = param.im.impl.use_current_controller_from_ccaller_module_1;

    % block parameters
    load_step_time = param.sim.load_step_time;
    time_start_motor_control = param.im.mode.time_start_motor_control;
    u_im_scale_ekf = param.im.u_im_scale_ekf;
    adc_quantization = param.meas.adc_quantization;
    adc12_quantization = param.meas.adc12_quantization;
    inv_pwm_phase_shift_mod1 = param.inv.behav.pwm_phase_shift_mod1;
    CTRPIFF_CLIP_RELEASE = param.ctrl.CTRPIFF_CLIP_RELEASE;
    kp_align = param.ctrl.kp_align;                     % mask of .../phase_compensation_1/alignment_ctrl/pi
    ki_align = param.ctrl.ki_align;
    lim_up_align = param.ctrl.lim_up_align;
    lim_down_align = param.ctrl.lim_down_align;
    time_gain_afe_module_1 = param.afe.behav.time_gain_module_1;    % masks of the clocks in "global timing"
    time_gain_inv_module_2 = param.inv.behav.time_gain_module_2;
    external_load_inertia = param.load.external_load_inertia;
    rpm_base = param.load.rpm_base;
    torque_base = param.load.torque_base;
    torque_overload_factor = param.load.torque_overload_factor;
    rpm_sim = param.load.rpm_sim;

    % instance parameters of the thermal bridges (Tambient and DThs_init: see the library workaround)
    cp_al = param.thermal.cp_al;
    Rth_switch_HA = param.thermal.Rth_switch_HA;
    Rth_diode_HA = param.thermal.Rth_diode_HA;
    weigth = param.thermal.weight;                      % typo of the model, never defined by the legacy init
    % the MOSFET bridge of inv_im.slx also reads mosfet.inv.Rth_switch_JH, not a property of the MOSFET
    % object: the legacy model runs with that bridge only after editing it to Rth_mosfet_JH

    % objects and structs
    glb_time = param.glb_time;
    hwdata.afe = param.afe.hw;
    hwdata.inv = param.inv.hw;
    im = param.im.machine;
    im_ctrl = param.im.ctrl;
    psm = param.psm.machine;
    psm_ctrl.ekf.C_ekf = param.im.ctrl.ekf.C_ekf;       % the IM EKF reads the PSM one: same constant
    filters = param.ctrl.filters;
    igbt.inv = param.inv.igbt;
    mosfet.inv = param.inv.mosfet;
    ideal_switch.inv = param.inv.ideal_switch;
    lithium_ion_battery_1 = param.battery.pack_1;
end
%[text] ## Local functions
function param = add_template_extras(param, options)
% Parts of the shared init template not used by inv_im.slx, kept so that nothing of the legacy
% init_model.m is lost. Enabled by param.sim.build_template_extras = true.
% Same convention: one group per stage (param.grid, param.frt, param.afe, param.psm, param.spinv,
% param.isop_rail, param.dab, param.three_phase_dab, param.psfbc, param.cllc, param.upqc, param.dcdc,
% param.diode, param.trafo_load), each with .hw, .ctrl, devices.

s = tf('s');
ts = param.glb_time.ts_inv;
dev = param.dev;

% ---------------------------------------------------------------- grid emulator
param.grid.grid_nominal_power = 1000e3;             % [W]
param.grid.application_voltage = param.inv.application_voltage;    % [V]
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

% grid impedance (the legacy ucc_factor preset is not available inside this function: 1)
param.grid.Lgrid_base = param.grid.us1/sqrt(3)*param.grid.ucc/100/2/pi/param.grid.fgrid/param.grid.grid_nominal_current;
param.grid.ucc_factor = 1;
param.grid.eq_grid_inductance = param.grid.Lgrid_base*param.grid.ucc_factor;    % [H]
param.grid.eq_grid_resistance = 2e-3;                                             % [Ohm]

g = param.grid;
param.grid.emu = grid_three_phase_emulator('Dyn11', g.delta_star, g.grid_nominal_power, g.application_voltage, ...
    g.us1, g.us2, g.fgrid, g.eq_grid_inductance, g.eq_grid_resistance, g.eta, g.ucc, g.i1m, g.p_iron, ...
    g.n1, g.n2, g.core_area, g.core_length, g.mur, ...
    g.up_xi_pu_ref, g.up_eta_pu_ref, g.un_xi_pu_ref, g.un_eta_pu_ref);

% ---------------------------------------------------------------- FRT
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

% ---------------------------------------------------------------- AFE control (with sequences)
param.afe.ctrl = ctrl_afe_setup(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom);
param.afe.ctrl.kp_udc_ctrl = 2;

% resonant PI gains for weak grids
% param.afe.ctrl.res_pi.kp_rpi = 0.5;
% param.afe.ctrl.res_pi.ki_rpi = 18;

% resonant PI gains for LVRT
param.afe.ctrl.res_pi.kp_rpi = 0.6;
param.afe.ctrl.res_pi.ki_rpi = 35;

% dqPLL selection
param.afe.pll.use_dq_pll_fht_pll = 1;
param.afe.pll.use_dq_pll_fht_simulink_pll = 0;
param.afe.pll.use_dq_pll_mod1 = 0;
param.afe.pll.use_dq_pll_ccaller_mod1 = 0;
param.afe.pll.use_dq_pll_ccaller_mod2 = 0;

param.afe.pll.use_dq_pll_mode1 = param.afe.pll.use_dq_pll_mod1;                 % simulink dqPLL
param.afe.pll.use_dq_pll_mode2 = param.afe.pll.use_dq_pll_ccaller_mod1;         % ccaller dqPLL
param.afe.pll.use_dq_pll_mode3 = param.afe.pll.use_dq_pll_fht_simulink_pll;     % fht simulink dqPLL
param.afe.pll.use_dq_pll_mode4 = param.afe.pll.use_dq_pll_fht_pll;              % fht ccaller dqPLL

param.afe.pll.use_dq_pll_mode1_modn = 0;    % simulink dqPLL
param.afe.pll.use_dq_pll_mode2_modn = 0;    % ccaller dqPLL
param.afe.pll.use_dq_pll_mode3_modn = 1;    % fht simulink dqPLL
param.afe.pll.use_dq_pll_mode4_modn = 0;    % fht ccaller dqPLL

% C-caller versus Simulink implementation
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

% module behaviour (clock mismatch, noise, PWM phase shift); time_gain_module_1 is in the main body
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

% AFE devices
param.afe.igbt = device_igbt_setup(dev.used_device_igbt, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.afe.mosfet = device_mosfet_setup(dev.used_device_mosfet, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.afe.ideal_switch = device_ideal_switch_setting(dev.used_device_ideal_switch, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);

% ---------------------------------------------------------------- PSM drive control
param.psm.u_psm_scale = 2/3*param.inv.hw.udc_nom/param.psm.machine.ubez;
param.psm.u_psm_scale_ekf = sqrt(3)/2 * 2/3 * param.inv.hw.udc_nom/param.psm.machine.ubez;
param.psm.mode.use_mtpa = 1;
param.psm.mode.use_psm_encoder = 0;
param.psm.impl.use_ekf_bemf_module_1 = 1;
param.psm.impl.use_ekf_bemf_module_2 = 1;
param.psm.impl.use_observer_from_simulink_module_1 = 0;
param.psm.impl.use_observer_from_ccaller_module_1 = 0;
param.psm.impl.use_observer_from_simulink_module_2 = 0;
param.psm.impl.use_observer_from_ccaller_module_2 = 0;
param.psm.ctrl = ctrl_pmsm_setup(ts, param.psm.machine.omega_bez, param.psm.u_psm_scale, param.psm.machine.Jm_norm);
% param.psm.ctrl.ekf = ekf_pmsm_setup(param.psm.machine.Rs_norm, param.psm.machine.Ls_norm, param.psm.machine.Jm_norm, ts);
param.psm.ctrl.ekf = ekf_pmsm_setup(param.psm.machine.Rs_norm, param.psm.machine.Ls_norm, 1e6, ts);
param.psm.ctrl.kp_i = 0.25;
param.psm.ctrl.ki_i = 35;

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
param.three_phase_dab.pwr_nom = param.dab.pwr_nom;  % legacy inv_im init passes dab_pwr_nom (afe_inv_psm: 3x)
param.three_phase_dab.hw = three_phase_dab_hwdata(uapp, param.three_phase_dab.pwr_nom, param.glb_time.fPWM_DAB, param.dab.fres);
param.psfbc.pwr_nom = 275e3;
param.psfbc.hw = single_phase_psfbc_hwdata(uapp, param.psfbc.pwr_nom, param.glb_time.fPWM_PSFBC);
param.cllc.pwr_nom = 250e3;
param.cllc.fres = param.glb_time.fPWM_CLLC*1.2;
param.cllc.hw = single_phase_cllc_hwdata(uapp, param.dab.pwr_nom, param.glb_time.fPWM_CLLC, param.cllc.fres);  % legacy init passes dab_pwr_nom (same value)

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
%   data: {"layout":"onright","rightPanelPercent":35.3}
%---
%[output:22e63205]
%   data: {"dataType":"text","outputData":{"text":"Device AFE_THREE_PHASE: afe690V\nNominal Voltage: 690 V | Nominal Current: 2.324276e+02 A\nCurrent Normalization Data: 328.70 A\nVoltage Normalization Data: 563.38 V\n---------------------------\n","truncated":false}}
%---
%[output:0e10b819]
%   data: {"dataType":"text","outputData":{"text":"Device INVERTER: inv690V_250kW\nNominal Voltage: 550 V | Nominal Current: 370 A\nCurrent Normalization Data: 523.26 A\nVoltage Normalization Data: 449.07 V\n---------------------------\n","truncated":false}}
%---
%[output:39e0b4d4]
%   data: {"dataType":"text","outputData":{"text":"Induction Machine: ABB M3BP 355MLB 6 261kW\nIM Normalization Voltage Factor: 375.6 V | IM Normalization Current Factor: 581.2 A\nRotor Resistance: 0.00274 Ohm\nMagnetization Inductance: 0.00376 H\n---------------------------\n","truncated":false}}
%---
%[output:8188f2d4]
%   data: {"dataType":"text","outputData":{"text":"IM EKF Fully controllable\nIM EKF is stable.\n","truncated":false}}
%---
%[output:0e2de209]
%   data: {"dataType":"text","outputData":{"text":"Permanent Magnet Synchronous Machine: WindGen\nPSM Normalization Voltage Factor: 365.8 V | PSM Normalization Current Factor: 486.0 A\nPer-System Direct Axis Inductance: 0.00624 H\nPer-System Quadrature Axis Inductance: 0.00756 H\n---------------------------\n","truncated":false}}
%---
%[output:281f8d6c]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAXMAAADgCAYAAADxP4bLAAAQAElEQVR4AeydC8wV55nfH2edNabgW+NwtyCh8iYEWMfBkLDFTWViJErZRbJULgkSkQhuqLAKmBLFNjKrsOGyMlo7eFGKhGLDZqmIKXWEhaX2Y0MXTC0bEMqisgFhczOWgwFxadOl\/CZ+jt9vmHPOnO87Z87MnD\/i+WbmfZ\/38vxnzn+eed533vnMDf0TAkJACAiBwiPwGdM\/ISAEhIAQKDwCIvPCn0IZIARyjoC6lwkCIvNMYFYjQkAICIHWIiAyby2+ban9gw8+sEcffTQS9lvViZdfftm+8IUvGFtvg33Ej5u5xZa4XZcvX7bvfe97dvjw4WY2VbMu2ho7dmw3u2sV6Ekf33zzTVuyZEmtanuV51hy\/hDaa6RCzjHl2DZSrpW64Dxr1qyW4tbK\/ve2bpF5bxHMYfnPf\/7z1tXVFQn7reriggUL7De\/+Y2xpQ1+2KtXr2Y3E+HHO3\/+fNu3b18P2+tZsc2bN9s999xjM2bMqFtBT\/oIsWJX3cp7oXDo0CF777337Omnn47O4WOPPdaL2vJRtF+\/fjZp0iTbvXt3pjf3fFhv9pm8dCTv\/cBLwhNB8MrwzrzPkBjpzz33XOQNsx\/XiXtC1OflqQt9vAq8TMoj1Os6SVvy0UMoTz3oeVt4seyTTv60adMMQZ+y6LLlGEEHXdIpR3mEfdIgGfQowzFbP2bfiZwttkBk6CHskxbW52lhu7TPMfUiIU7U43LlyhWD8CDyS5cu2fTp0yNP2eukrAt983JsqZM82sEm+kTfKEt+vA9hefIgi3HjxpnfKMGHOqgToV70qC+pj7RBnei6ePv0hzLobN++PbqeqB8J28AGdKoJbVOn109Z6kCftr0NzlWYR74L+uR5HdXaJB0dt9vLYwvpLtRFneSDD\/pcjwg69It89Dim\/6+99lri0x\/5CHVQF3UiEydOZGPccKOdDvojMk9xsrlY+WG5qpMHF6unsf3Zz34WeTvso7Nw4ULj4kSeeOKJSh751MfFy74LxARR+PGGDRuqehiU5YfourQ3Z86cqvroHTlyxJD+\/fsbF31P6qCenoh7TXiDeIXUcfz48ag\/o0aNshEjRkR9xwZsIR8BJ37UkBPH9WTFihW3eOohjthMndRDO88\/\/7xduHCBw0gghngfwNnP9d69e41yI0eOjPTp11NPPdXt3JLPuf\/www8jnfgf6qLOMJ1z\/8orr4RJlf3z589bmuvHC9AnyJo6PQ3cJ0+eHGHsabW21a5ZfgthOewI8Vy1apXRPjguXrw4VI0wQj9M5HpEuCa\/+tWvWogl\/f\/pT38aqkc37LAOx5r+ojhgwIDoqenAgQPRb4+0ThGReZ0zzUUJwQ4bNiwiCcIKGzdujEpt2rQpunCjg5t\/JkyYYBAVwj4\/IPYR9v2RlmPyt27d2u2C44LesWMHdRj5XKjnzp27WXP3\/1y4lHV9+kTd6EM23bU\/PXL9gwcPGhd9T+r4tLbue4Ra6AOpbLds2WIQOMcu3EDow65du6Ik+kqfeTRGF2+KY8pjk+PEjxqJCn3yp2\/fvsZ5ACfqBDf6sHbt2ihsQHmEUAh1gqPjRhWUJZ8bBfmkId4nz6ddzr2f62PHjqFmTubRwc0\/6KBLnUjXzTDX8OHDE\/tISAMdF9q6WYVRN3l+TN+p58yZMxERxnHh\/GETZUOhHwjYgCHtUBY7wRicOKYMW9rwpwzSEAiaa5Y+UJ76sDFOkt6G5586dcp4aho9erRxnVEW4fxwnk6fPt3tN0MaeehevHgx+o3RDvXR97vuuovuRMJNYs+ePeb51AtW9JP+ooQdPDVxg+ack9YpIjKvc6b9xz1z5szKYzUXMOIXrlcxb968iMAgJvZJ5weKsI9HwaPhmDFjoos2fsG5h0p5CI4yScJFSlnXR4cfKBc3W46TJNTvaR1J9aZNw\/umDxACXrn\/MCEMSIl0fqgcUyc4hDiSllbwwMHaf+SUc5s5dwhptEWb7CN+rvBsKY8eZMG5xtOGjCAgbobo00fOFTroUoYbBMRDfi3BUSBMQFu19LxP9a4fr8NvlmBH\/0h3O+l\/mr55m1OmTKF4dO1D+giEGSXe\/IPttMHNdciQITdTuv\/nvBI2IQzGzaR7rhnXA9cF6d6m\/9aoFxvIQ7hJcB5CrB07L4seQlucb\/Y7RUTmdc503AOrpR6\/oGrpkle0C65R+7AxFH6c\/Pj5MUKyPF7jRUEO1cggLJ9mnzAAhMp54+YGiaUpV0+HmyfElKTHDTRsC6+SpxDIOkmfMAt9xEvGI8W7TNKrl1bt+sH2emVbnY\/t3KieuhmCev311w3vm5tgq9pNe5NqVft5qLdhMs9Dp7PsAx4YF2H4SMuPFcETgYS8P3iaeD0Ij+Wk88NC2OeRlh99KDxWk9eI0CdmU9AHhLJOEJAZx\/UkbR2QGB4ONmFfvXrr5UNy4PnSSy9FsWf3\/CD6wYMHR+EEiJ56aDPEkbRaAtm6d8\/TD+X5kXuZJJtpi5uL6\/i5gmDD8wTpfvnLX3a1xK2HeDjPEC1PdUmK7jm77fVukt4n6g37xH7S9eP6YAcG9MHtBGOwJq2WxOugHp44IGiIulZZ8rAdDLh50x7XEMfkVRNv039rtIkNrs9vjd8cT1Jc99jvkhTW83KdshWZ1znTxP4YOOIH74\/R\/mjHIyAXqlfBBQaJIOxz0bGPsO+PyXhlCD8OLlgvn3aLJ8ujKPr0hbrYQpJz584lua7Uq8N\/OPwAeUTGBmyqW\/FNBezk0RpyvXnY7b\/jSSKYUC\/7CH3HBspjE3m0Ce4IOknifWQwk3w\/V16eNCTJZkiANslH\/GYDnvTBhZsk5xoypD3ICX1sxFbXY0v\/qZO60EEoA47UwzHibaDPcVwgYMjzs5\/9bBQnRo\/6XapdP2CFgB0YoE9Z+gTG8XaSjnmi4fx4HdTDPr8FzmFSmaQ02qV9bE3KD9PoMxKeP+LorgP+3Bw8n3pdcGZcjy22cvNmv1NEZJ7iTONxcXG7KhcKj41xr+jb3\/529KNDD50XX3wxijVCItu2bavkkc8P5YUXXohi7Bw3Kjza46mF5datW2eN\/NBq1cEPZ\/ny5YYdtEF\/V65cyW5VASP0qip8kuEeqYdYPkmO+s6MDm+TdOqEcOkPx6GQxg3V0\/r06WPPPPOMH0aDyJSlPveGsZk6UaKvP\/nJT6LZDxwj4BfvAwTDLBnyve\/uTSedW\/T8XMT7SB7njbbZp2\/0kTZ4quDmwD5CPnLfffdZI9cPbfJkEdZBewzkYx911pMku6jPcahXnhlB6Lse\/QF3QmuMl3h6uKXf\/CaGDRsWJaNPmCY6+OQP5w\/8PjmMNhz7bxH8wDGMxUdKHfCnLWTOYxp3WLZJGHOX9TsuW7yTarpJ5VuRBqH7Ix2P3Ek\/ikGDBkUv6qAX1+HHweAReQj7pNFX6kKfHzUXNGlctOj5RUpaXFwHPcR1qZf6EfaT6ve6qtVBvpejbuqaPXt2NFOEMuSzJY8tx7SFHmlsOSY9LvQTHTCN54VtxnWoj3oR9inrdblueAyeEArY0hZPQXizeLwQAPXcuHGj29RE6oz3gXr8vOChQoxO5ujTF+qiDy70gzyEfU+nH6E+faOPtEEd5NEWx5Qhn\/6QTj5pCPukUX+ShHUk6XPOSGebVJ406qcd9BD6RL3kUY40thyTTj76lPNjdBAwwHa3B5vYpwy61MFvHM+f0B7OEvohzuggtEmdLhyTjvDERHmeoLxe0jtBMidzflDMRf3444+r4ssJ5K7sJ4uTzsmvWkAZQiAFAvy4+ZGj6o\/\/hD4IgeBckE9eLYGoeKLA+8MLrKWrvMYQICzCWBDng\/OCI8d54uklDFnVqpVYPflpw0nolkUyJ3Pibu+\/\/77dfffdVTGEzH0wpKqSMoRADxAgTIAnHBbFcQi9uzAvth8dQhR4f8yDjhL0pykIcKMkNAl5hxV6yCpMS9rHUWSQHu++E52\/TMkcT4bR6WeffTbpXERpnJBwBkKUmPM\/EAFPEWxz3tWO7x7eN4\/2nC8XHucbAQai4GmR0EEj5aRbHwHH1s8N27Q4+7lt9HzW71UxNDIlc2KVPM7yOFUNHn8xgMcrHrMQXgCppq90ISAEhIAQsOwW2mJw45133jFGuWsB7wMYjH5zV2YghLU1GBRNKkfIhtBNp4vs3xe9VSschENW1wDck8RJ7UrLzDPfvHlztFgQj0K1jPXHLH+04pgYmE8tC8sC5tKlS40ZCpJZwmGWMNDvILtrAO6Bg0JOaud+JmROrJzRf14cIGzCSDUgsK3mccdBSRoQpY79+\/fbmjVrjDhoGWXRokURFGW2kfPWCXZ2go2ddC7hHjgo+oHm4E\/ryfymkYxSd3V1RXOUPXQydOjQaL0G98BvqkX\/IXfeqOMGQALHvOxQa2oSr\/gyQ6GMwg2P2RMPP\/xw9BJMGW3Epk6wsxNs7JRzOX78eOgpV5IJmdezGMLm8ZCZLJA7b\/FxUeDF482nnZpUr50i5g8cONC+853vGNsi9j9tn7Gv7HZ2go2c706xE1vzJG0hc+LgzAdlCxgQOI9nHk\/nGA\/ehWP0JEJACAiBBASUdBOBtpD5zXb1XwgIASEgBJqIgMi8iWCqKiEgBIRAuxAQmbcLebXbEQjISCGQFQIi86yQVjtCQAgIgRYiIDJvIbiqWggIASGQFQIi86yQzls76o8QEAKlQkBkXqrTKWOEgBDoVARE5p165mW3EBACpUIgh2ReKnxljBAQAkIgEwRE5pnArEaEgBAQAq1FoC6Zs+AVC1+xTkpaQb+13VbtQkAICIGeI1DGknXJHKPvuuuuaIVDXyul1paPSaBPOYkQEAJCQAhkg0BdMmf52p07d5ovilWvW+ihX09P+UJACAgBIdA8BOqSuYdZCJ2w37ymVZMQEAK3IKAEIdBDBOqSOZ7566+\/bv4BCOLmrD\/ew\/ZUTAgIASEgBFqAQF0yp03WGWe9cWLlGzduND4YAan7ByXQkQgBISAEhED7EEhF5mH3+FAEpH7o0KEoecyYMTZ27Fg7fPhwdKw\/ZUVAdgkBIZBnBBomczcm9NZfeeUV+8EPfmCKqTs62goBISAEskWgx2TO9zoJsxBumTNnjv3oRz8y4uvZdl+tCQEhIASEAAg0TOYMfkLghFeogHDLwYMHU09dpEyTRdUJASEgBDJF4Hef+yO78tV5turvLtuvjl3ItO1qjaUi89ALZ\/Dz6aefNuLmDIoSbqlWudKFgBAQAmVE4L3fXrP\/88BE+7szf5Ab8+qSOXHwqVOn2pEjRypvgS5YsCA3BqgjQkAICIGWI1CABuqSOXHwrq4uUyilAGdTXRQCQqBjEahL5njm06ZNSz31kCmK6HcsojJcCAgBIdAGBOqSOX26ePGiTZ8+3Rj4rCfooU85iRAQAq1GQPW3A4GTH11tR7M126xL5h5mYcAzrRCWqdmqMoWAEBACQqCpCNQl86a2psqEgBAQAiVD4IH7+uTCLtxKDQAAEABJREFUIpF5Lk5DKTsho4SAEMgQAZF5hmCrKSEgBIRAqxAQmbcKWdUrBISAEMgQgYbJ\/OWXX67MauHVfoQ1WnhLtBX9XrJkiSHNrFt1CQEhIAR6g8DJj671pnhLyjZE5pDqnj17bN++fTZhwoSoQyyJO3jwYFuxYkV03Mw\/3Ci2b9\/ezCpVlxAQAkKglAikJnNeHjp69KgtX77c+vbt2w2MuXPnGnnodMvoxQGe\/rZt22zo0KG9qEVFhYAQEAKtRSB5Nktr20yqPTWZJxX2tHPnzlmzXxRijfSHHnrIHnnkEW9GWyEgBIRALhDwMEteiBxQUpM5Lw+x4NaqVavsypUrlI0Eb3zlypU2c+bMpq1nTp2Ec2bMmBG1oT9CQAgIASFQG4HUZE41rJY4b968KF5O3JzlcImdQ+TkodMMWb16tdEON5A09e3fv99OnDhh165dK6Vcv37drl69WkrbwnPWCXbmwMZMrqMy2wnX7P3H369hPuzefLwwBE82ROYUYMAz\/lp\/M4mchbouXboU3TBoL42sX7\/eNm\/ebGfOnCmdnD171s6fP2+Esspon9vUCXZ2go2cz7Lbuelv\/1uFlgoZZqn0vsU7e\/futd27dxtfMmJRL2azIEx\/rNb0mjVrjEHYQYMGWdlkwIABdv\/999vAgQNLZ1t4rjrBzk6wkXNadjsffPhPKlRUSDInjv3oo49W5phDtEnC9MWKpT3YwcsPPX\/i5ghfNapW3ZAhQ2z48OHWp0+f0smdd95pd9xxR+nsip8r7Cy7nZ1gI+e17Hb+zT\/cqFDRxC\/eW9lv907qMAvxa2Ljw4YNi+aZO+ESOyfNPyWHQb0ldOqQCAEhIATyhgCzWDxe\/pkrH9rtH\/5DbrqYmszxzLdu3WrPPPNMt1krkDxp5KFDuIM5582ycO3atYY0qz7VIwSEgBDoKQJO5JT\/w5N72eRGUpN5bnrcjo6oTSEgBDoeAbzy72\/9dYQDXvkfnvyf0X5e\/qQmczxwwizMKccDdwPYJ408dBjA7N+\/v2drKwSEgBAoPAIhkWPMvxz0\/wxCZz8vkprM6TCDk4RUmFvug5\/sk0Yea6ls2LAheuUffYkQEAJCoOgIOJF7iIUZLMseH95ss3pdX0NkTmtJ88xJ87yDBw\/a6NGjOZQIASEgBAqNAET+b3\/yjoVE\/l\/\/\/UO5tKlhMs+lFeqUEBACQqCJCEDiP37juP3xn\/+9sU\/VeOQQOVuO8yYNkTlhFA+vxLfMQSd+njcD1R8hIAR6h0AnlYa4IXG88R+\/caJi+sQv3mPv\/vDrllcip6OpyZwlaTdt2mTMJz906FD0uv3GjRujOefMM\/cBUCqVCAEhIASKhMDJj64ZJI4nDolzTP8hb+LjO7+fz9AKfXRJTeaslMiaKRMnTrR+\/foZH6Q4duxYNOecAVBWOYTwvWJthYAQEAJ5RgDChsCnvfROFE6BxL2\/TuJ448seH+HJud6mJvO4FSNHjjTInHTWYoDoIXyOJUIgEwTUiBBoAAHI+1fHLkQeeEjgPrhJVUUkcfqNpCZz5pA\/+OCD0eqEFITMT58+bXjjPrc8\/gUi9CRCQAgIgXYg4OS99cBZc\/L2WHicwGeOG2gMbhbJE49jmprMKUi8\/MCBA8ZAKNMRCbWwuqHPLSf8gp5ECAgBIZA1ApA3xM1bmiF5cxySN\/3CA4fAX5r5pWhgk+2fjLyHrMJKQ2SOd97V1WUQORazZgoLbmluOWiEon0hIARaiYATt8e87\/uP\/z2Ke0PcEHoSeTMjZdnjwyseOAQOobeyn1nWnZrMmXY4bdo04+MR8Q6SRh468TwdCwEhIAR6igCkTZwbgoao8bhD4mbQMk7ctIXnHSdvZqQsuzmYWXQPHPuSJDWZJxX2NL6C0+wPOnvd2goBIVB+BGqRNnFuiBxCr0bckDdeNt62x77LTt7xq6IumRMf5wUh1mA5cuSITZ8+\/ZYPVMyfP9\/GjRsXTVOMN6BjISAEhAAIxAkbgg497XqkTR2QNh53nLgZuITISS+r5439taQumRMfJy7ORyhGjRplO3bsMI7jQvy8VkPKEwJCoNwIQNbIvhOXbeevL9uibf\/bahF2NU8blCBtBHKGpN3bhrR3fv8hI428TiVuMIpLXTL3Agx+7ty5U4toOSDaCoEORACy9hg2g49O1rw56TJj42Fb8eaHtu2dD60WYQMfhF3N04a4Q9JGlzKSZATqkjmDmqy7QqillqCDbnIzShUCQiDvCEDUyK+OXYhIGKJGCIVA1D7w6OEQBh+drClXzT5IOCRsCBpP+6O\/\/KZB2HFPG\/1qdSm9OgJ1yRyPnOmI8bBK\/BgddKs3pRwhIATahQBkizhRQ8JO1JC1EzWk7WSNDsKgI2Wr9R3yRQh7IGv\/dLhtnDHQ9i15yJIIGx2FR6qh2fP0umTe86pVUgjkDYFy9geiRZyow\/BHNaKGyJ2oIetayEDUCCSM4FkjoXeNh00aMuuRQfbwkD65XmHQSvivYTJ\/+eWXb5nNQloJsZFJQqCtCEDQiJM05AsJI5A0XnSSRx2GP9IStYdBnKwhasQ965Cs0UHkXbf18ril8YbIHNLeunVrtOyth1mY5UIaebfUrgQhIARuQQCCRqqRNESdRNKQOISOQNLUcUvlQQLeNBInarxniBqCdvG4NXlO1CLrAMwC7KYmcwY3IW2Wuw1j4+yTRh46BbA5j11Un0qAAOSKVCPpCWvfsYf\/6oSxxav22HScpCHqenBA0ogT9bLHh0fT9SBpJPSo40TtZE35eu0ovzgIpCbz4pikngqB5iEAOSMhQcdj0hBzMzxpeg3BIk7SEC\/eMgJJ40knEfWyx0cYunjTCHVJOguB1GSOB87XhFauXGmhB84+aeSh01nwydoiIgA5I0kEjZdMmKMWQcdj0tRVDwcIevBdt0cDg5AuAkEjtUiafHQRSJp66rWl\/M5EoC6Zh7AsWLDAIG1e7fc55+yTRl6oq30hkCUCECrSW4JOG4922yBXxD3pZZ+EOyDhOEn\/r\/80Lpqyt+7PRkQhEQgaEUk7mtr2BoHUZI4HzotBNOaDn74VkYOKpNkIQM5ISNB4zgjeM5LGg+4tQUO4kDMCQSOEOhDCHojHpZd9Eu6gjEi62VeE6quFQGoyJ4TCQOfq1asrUxNZhKtW5coTAiECEDMSkrPHnxf\/4rjN3342Ghwk\/oxA1EitgULqC9uoto\/3jLgHDdlCzgjkjEDKSQSNDvoIBI1Ua0fpPUFAZZqBQGoypzFfdAuPnK8OsVoi4ZaxY8cmrnNOGUl5EYBIkSRy7on3\/Papa0Z9aRCDmBHIGYFoEYgXgZyRRgia+tK0LR0hkEcEGiLz0ABCK5A6MnnyZFu4cGG3gdFQV\/vFQAAiRZpFzkyxQ6gzDQIMEEKoITkvqxGDhqgJbyAQOAKhI3jPCPWlaVs6QqDoCPSYzJcsWVIJt\/Bd0G3btmk985xdDZAo0i5yhkiRWuTs3vPpH33dds4dGq3nsfOTJU4h52XtjUHn7IyqO0KgOgINkTlveRJWQXbv3l1Z2zzNIluXL1+2WbNmVW4A3AyqdYtYPG24KIzze6QgZiRP5IwXDOkiEDOCx1wt9hwnZ3nPvz+3+isEeotAajJnNgtveW7cuDH6OMXBgwcbWtt8xYoVNnjw4KgsSwDgzXNzSDLg2LFjNmPGjEiXME6jbSXVmdc0yBlJImhmazzxn39t0za\/b4N\/8PfRB2vjA4I+59lnbDQS1sBrRtJ4ztXIGRKH0BGIGaHOvOKtfgmBsiKQmsyZzYIHziBoT8DgS0QIZamLz8xB2hzHhfSRI0fGkwt5DFEjkC3CwCACUVebseEEDTEjpy\/+7hbbkxIgUaQ35CzPOQlZpQmB\/COQmsybaQpePp75lClTbqmWcMzp06dvSS9CgpO2T7eDsPGkEQgcgdARSLqeTRAzg4IsJ4rniyyLDQgqrFEPReULgc5AIHMyJ1bOW6NDhgwxtnGYr1y5YqdOnbJwPnu1cIyX3b9\/v504ccKuXbuWqfzmg8uVL7K4lw1hh5619zG+dZL+s9H3GLL2T4cb8rfz\/igaBGRAkMX9f\/XUaPurqffan08ZaLw5uOjRQZE+Zb42tI8hn+9rmdrdKpyvX79uV69eLYUt1TDqBBuxvcx2wjVwVPw33e7j5pN5HYsItRAHJ37OPHU88bDIuXPn7MKFC+ax+R07dtiGDRuMQdFQL9xfv369bd682c6cOdNyefvoe7bitZtx7Jfesa\/9xQGDvPG0w\/6wD1kj077Uz5AVj30uepWbGRtv\/4fh0cwNvsbyw391jyHfHGaGfPGfXbPPXv9txY6zZ8\/a+fPnDVyysK9dbXSCnZ1gI9dP2e2Ea5YuXcrPPFeSmswJjUybNi3x5aDDhw8beeiktW7u3Ln2\/vvv2\/Hjx7sVGT16tDHg6bF5jpnHvmvXrm564cGaNWuM+gYNGmStkv97x03v+H9ciAYjN751wXjBJewDIRHCIAwIEvpgHQ7kr7892pD5\/\/pf2L\/52gh7+MFhDfVxwIABdv\/999vAgQMbKtcqHFpVbyfY2Qk2cn2U3U64ZtGiReHPPxf7qcm8Vm\/xGi9evFhL5ZY8ytx2223Gib8lMyGh1oAoIZvhw4dbnz59mi4fXDFjRsmEte\/YLw5fqPQsJG+f6QGRQ+jM6GhWX+6880674447mm5Xs\/rXrHo6wc5OsJHrIQM72\/p7gGvGjx9f4YK87NQlc8IbzPcmvn3kyBGbPn16Za446QjhEmanMEulmmHEyhHyCa1s2rTJksrQHgt6uZfPMXPaJ06cSNHM5ORH14yBTAYvw8FKSBzSZi41W8g7s06pISEgBIRAFQTqkjnhDmLczA0fNWpU5UUh0kIhFl6ljSh5xYoVxiwVyH\/MmDGVNHYgbF4oguRpjwW9uHmgy41i3bp1Dc1pp87eCHO+Wdzpx2+cqFQDaRM+gcTZr2RoRwgIASGQAwTqkrn3Ea97586dPSbVfv362ZYtWyovArFPGvVD4PHj8EZBPnpZCN44RI5nTnt44pA4XjjhE9IkQqAaAkoXAu1CoC6ZE+4g7PHqq68aW7zlJCEP3XYZ0ox2IfLQG2dON564SLwZ6KoOISAEWolAXTLHI+\/q6rLZs2cb29BjDvfJQ7eVnW1l3SGR441D5MseH9HKJlW3EBACQqBpCNQl86a1lOOK4kT+4r\/7kpWOyHOMv7omBIRA7xFITeaEUAilJIVYSCMPnd53KdsaGOz00AoeOUSusEq250CtCQEh0HsEUpM5IRRCKWFohf1Dhw4ZL\/W8+OKLhVvPnEHOhX\/z6wqKzFIRkVfg0I4QEAIFQiA1mVeziRkpTzzxhK1atcqYWlhNr3p6+3J4FR9CpwfLHh+u0ApASISAECgkAr0mc6zmLc5Lly4Zi2RxXARhPRV\/GYjwyjINdhbhtKmPQkAIVEGgKWTOwjP9+\/e3vn37VmkmX8l44wx60iuInCJz1ycAABAASURBVDg5+xIhIATKiUAnWJWazBncZJCTwc648Lr98uXLjZBLEUDbeuCMQej0lQ85KE4OEhIhIASKjEBqMq82AMogKKscsrphEYCAxMPZK7zZWYR+q49CQAgIgVoIpCZzKmGAkzVU\/GMRfuwLaKGTd\/HwCv1UnBwUJG1FQI0LgSYh0BCZs1gWH5VYsGBB1DxhFT4iwQJaRSB0vHIGPuk8sfKZ4wayKxECQkAIFB6B1GROzPzo0aPRRyBCqyF04uXkoRPm5W2fWLn3SV65I6GtEBACZUAgNZljLB+g4KMS7IdCGnlhWt725ZU3ekakLwSEQJEQSE3mDIDOnDnTVq5caaEHzj5p5KGTV+Mhc4T+MYOFrUQICAEhUBYEUpM5BhMrDz8cwRRFPiIBkZOHTl4lHPicOW5QXrupfgkBISAEeoRAQ2ROC3wogumIoWRM5HSjIcEj97c98co1r7wh+KQsBIRAARBITeY+DZFPvBXArm5ddCIncdYj8srBQSIEhEC5EEhN5qy7wvorrMNSNAi2vHUm6rKmI0Yw6I8QEAL1EChgfmoyZ3Bz6tSpxjosRbIzDLEMu7dPkbquvgoBISAEUiOQmsyZtbJ161bbvn27MfAZF9ZtQSd1yxkphiEWzS3PCHQ1IwSEQOYIpCZzPPOkj1P4QCh56GRuQZ0GwxCLBj7rgKXsAiCgLgqBZARSk3ly8XynKsSS7\/Oj3gkBIdA8BOqSOaETQiivvvqqsY2HV\/yYPHSb17Xe1wSZey3yyh0JbYWAECgjAnXJnNAJIZTZs2cbWw+rxLfkoZsnkMK1WCZ+8d48da2MfZFNQkAItBGBumQe9s3nmhdlCVz3zJmSKM88PJPaFwJCoGwINETmRVoCFyL3mSyakli2y1b2CAEhEEcgNZkTD2eZ27lz53arI69L4ELm3lFNSXQktBUCQqCsCKQmcwBgmVuWu2U\/FNLIC9PavR8urNXuvqh9ISAEhECrEUhN5gxusjoiy93ipXvH2CeNPHQ8PS9bxcvzcibUDyFQZgTab1tqMqerrI6YxRK4LOblUx7Hjh1rhw8fpvnUQohF8fLUcElRCAiBEiDQEJljb6uXwIW4n3\/+eduxY4cx\/fHJJ5+0hQsXdvsgBv2oJZC552sWiyOhrRAQAmVGoGEybzUYo0ePtj179hhb2po4caLduHHDiMtznEb2\/uNvK2qaX16BQjudiYCs7hAEckfmcdz37t1rQ4cOtREjRsSzqh7\/6tiFKE\/x8ggG\/RECQqADEMgtmRNuIV6+evVqmzdvnjEFstr52L9\/v504ccKuXbsWicfLB\/W\/PTr29KJur1+\/blevXi2FLbXOQSfY2Qk2co7LbCdcc+rUqWp01Lb03JI5YZaDBw9GsfPFixcbg6LVUFq\/fn20zvqZM2fs7aPvVdTGDrjdSCuynD171s6fPx+FmYpsR1Lfw7ROsLMTbOSclt1OvumwdOnSCs\/kZachMndv2WeahNtWLbQFqU+ePNl27dpVFbM1a9YYLzMNGjTIrv5Bv4relD8eZqQVWfiy0\/33328DBw4svC21zkMn2NkJNnKOy24nXLNo0aIKz+RlJzWZsy7LqlWrDGJllklcWrXQFu2ePn3aRo4cWRWzIUOG2PDhw61Pnz729qlrFb3bb789SiO9qHLnnXfaHXfcUXg76uHfCXZ2go2c57LbCdeMHz\/e8vYvNZn7N0C5K7XSCLz\/SZMmVeaW79u3z44cOWLMaknTblMGP9M0JB0hIASEQI4QSE3mffv2tf79+0ex21b2n7DKs88+a9OnT48+Tzd\/\/nxbt25dZapivbZ98FOLa9VDSvlCQAiUCYHUZM5sEjzmTZs2GaGPVoIQfzGJ4zTtnfzo0xCLXhZKg5h0hIAQaBMCTW82NZmzBsvWrVuNsMeYMWMirzmLAdBGLA7JvJFy0hUCQkAIFB2B1GTOIloMcsYHPv2YPHTaCYje\/Gwn+mpbCAiBdiKQmszb2cm0bfvgJ\/oKs4CCRAg0joBKFBOBhsmcT8Z5eIUXeZBZs2a1PI7eCLy8xt+IvnSFgBAQAkVHoCEyX7JkSbQIFnHzCRMmRLYzODl48GDjk3JRQhv\/aCZLG8FX00JACLQVgdRkzgAon41bvny5MU0x7DVzz8lDJ0zPcj8c\/FSIJUvkM2xLTQkBIVAVgdRkXrWGmxksT9vuz8aFZH6zS\/ovBISAEOgoBFKTOTNVpk6darzSz9ugjhLeeB4+G6eZLH5GtBUCQqATEUhN5oCzYMGCaDla4uXEzXk7k32+\/0keOu2S0DPPfgC0XVarXSEgBITA7xFoiMwpwoCnzy33bbuJnH45mUPkCGkSISAEhECnINAwmecVmPd+++mr\/Hnto\/olBISAEOgpAvXKNUzm4TzzcL55vYZamQ+Ru2euBbZaibTqFgJCIK8INETmELmvz+IhFmLnDICSlwcjNS0xD2dBfRACQiBrBFKTObNWIPJnnnnGmNniHWWfNPLQ8fQst+6V0+YD993JRiIEhEBWCKidXCCQmsxr9ZbPRN111121VFqaF67JojBLS6FW5UJACOQUgdRk7h744sWLK18Bwia88YULFxpz0NEhrZ3ywH192tm82hYCQkAItAWB1GQOaRMbv3TpUuUrQAyAMs\/8vffes9WrV1fWOOfjzlla42uyQORIlm2rrTwjoL4Jgc5BIDWZ43WzZrkPfNbaotc5EMpSISAEhED7EUhN5u3vavUe\/O5zD0aZipdHMOiPEBACHYhAw2TOFETCK6GQ1i7s\/qnv5ypNFyjEUumzdoSAEBACzUCgITKHtJmCyNxyD7OwTxp5zehQo3X8U99\/XikiMq9AoR0hIAQ6DIHUZM4AKKTNnHLi544T+6SRh46nt2M78Yv3tqNZtSkEhIAQaDsCt5B523vUYAc8Xt5gMakLASEgBEqFQGoyxwNnqVumJ4YeOPukkYdO1ugoZp414mpPCAiBPCKQmszpPEvdQtrMLfcBUPZJIw+drMXJnHg5knX7ak8ICIFGEZB+KxBoiMzpAKTtg5++JY08iRAQAkJACLQHgYbJvD3drN6qx8w1x7w6RsoRAkKg\/AgUmsw9xFL+0yQLM0JAzQiBwiJQaDIPUdc65iEa2hcCQqDTECg0mYee+QNax7zTrl3ZKwSEQIBAZmTOFEZWU\/RZMLNmzbLLly8HXfl0980336yswIj+2LFjuy2765p8Ls73yx4zdzu1FQJCQAgkIZAJmUPaTz31lDGFkRkwhw4divqyYsWKaBv\/c+zYMZsxY4ahixw8eNBGjx4dV7OTH129JU0JQkAICIFORCATMu\/Xr59t2bLFfAojx5MmTbLTp08neueQ+ciRI+uej+6fi9NHKeoCJgUhIARKi0DvybzJ0ODFQ\/JpqhWZp0FJOkJACHQCAm0hc+LnLMyFd46XHgJ95coVO3XqVLcvF9VbkfEBfSouhFD7QkAIdCACmZM5njfx8yFDhticOXNugfzcuXN24cIF27hxYxQz37Fjh23YsMEYFI0r+wDoZ658aCdOnLBr166VUq5fv25Xr14tpW3hOesEOzvBRs5pk+3M1bUP1+Bwxvmo3ceZkjlEPn\/+\/MhmyDrulZPBQCcDno899hiH0cDn5MmTbdeuXdFx+MfDLO+\/f8o2b95sZ86cKZ2cPXvWzp8\/b9zkymif29QJdnaCjZzPstsJ1yxdujSkolzsZ0bmTuSDBw+OBkOTiLwWIvEBUSdyynx99EibO3euDRo0qHSCfb\/85S\/ZlM628HxhYNnt7AQbOadltxOuWbRoEWbmSjIjc5+G6NtqKBBOYT46cXV0ON69e7dNnDiRw4qEZE7MHG\/g3XfftXdLJkeOHLGf\/\/zn9sYbb5TOtvBcFdXO0IZ6+51gIxiU3U64ZujQoRUuystOJmQOMR84cMD4xNyYMWMqLwQ5aUPY\/hIR4RW+XMTSurwwRFhm3bp1UbglBA0C9+Nf\/OyvjfJlFH+cW79+fWlt5Lx1gp2dYGMnncvx48dbnkg9EzLnoxVdXV3RgCYvAbmQRh4Ezjx0D71w7DpsOXbi9i1k\/tFfftPe\/eHX7b\/8xfej0A11SLYIiy3CQL+D1l8Da9as6TwydwJuxRZSx4uXTLBuGEzQsfDQNdDKayBPXjncmolnTkMSISAEhIAQaB0CIvPWYauahYAQEAKZIdAGMs\/MNjUkBISAEOgYBETmHXOqZagQEAJlRqCwZM56LUxdRNgvw0lasmRJZdom0zWr2RTqVVvrvVrZdqeHfa9lo\/eTl82Y7la0c5zWTqbtMkWX6xhJg4lj0+5tWhsPHz5sXKfYx5bjVvc9q\/rBIC\/XZiHJnIuBqVes24KwT1pWJ7AV7fAj9rn4LHWwcuVK44ceb4sLh1UlWROeaZtPPvmkLVy4MFE3Xrbdx2ltDPv5yiuvRO8nhGl5309rJzcq1ikaN25cNG2Xa\/n5559P\/BBL3mxOayPXMNcn74oU7XqthzlEvn379npqmeUXksz37t0bze8cMWJE9DLRI488YqRlhloLGmLtGX7UzLtnOhULkUHY8aYWLFgQzSP3Ofm8GXvjxo1o7Za4bt6O09ro\/YYIXn\/99ehce1oRtmntPH78uF26dMmefvrpyCzWJdqzZ090TUcJOf6T1kbWFOL6HDBgQGRNka7XqMMJf7gueZo6evSojRo1KkGjPUmFJHM+XsEaL05oQEca2yIKHhrednz9mSLbFD8PPbFx9erV9t3vfrdQZF7XzgAYHJD+\/ftb3759g9T87zZiIyR+2223VZwNbGZ+No5Y\/i2t3sOXXnrJWMab81ddK9ucQpI5EIXEF+6TV1RxO7hJcbNKYwcruBXpx5HWRsJmeK3f+MY30sCQO520dnKeV6xYkWqsJG9GprGRJ02erjZt2hTZiINCWJRrPG\/2pO0PNn3lK19Jq56ZXmHJPDOEctwQ8XMWIVu+fLkV+ccRhxjP78UXX4zGAormtcZtqXdMzHXKlClRzJyxksWLFxciZl7PLs\/npkxoZd68eZGN2EqIglCF62jbHAQKS+bc4R2CcN\/Tirh1OyAzwi61bIDI+WgHA4TEWmvppsvLRiuNjSzIxuNrkeyKo5fGTsowPoL4PjFYQhEc513S2Igt2BTayHgQN7G821e0\/hWSzP3xLgQ7KS3Mz\/M+XjWP2\/E+VrMJIideh1deFMJrxEYG1\/ixM5WNVTYhd+LnzB6IY5S340bsrHZ+q6XnxdZGbMxLnzuhH4Ukcx7b3nrrrehxlMc49kkr8gnj8ROC5vET8uKzVBBZ3CamhOGRE4YgdhfPz\/NxWhvXrl0bPZIzlY0ZPXh1zPggPc\/2ed\/S2sn55TxzvinLlmPSOc6zpLWR3yXrm2Mb9rDlmHSOJc1DoAhkfou1eKO8SDJ9+nRD2CftFsUCJbDML1MTIS7WcGdNdydrPHH3SvFaGRjEbjxXF0g+7+amtTHvdtTrX1o7Ob\/btm0z3ingPBIvL8qLuNRxAAAGQElEQVRNOq2N\/C6ZY8417TYWLTRY73znJb+QZA54zLfGc0PYJ63ogueJPQg\/FrcH+8jjmC35cQn10curhP0P+xzaGPadR3pmP5Afpud9P62dEDrr+nM++fYt5Jd327x\/aW3kPGMfUjQb3dakbd6uzcKSeRK4ShMCQkAI9AiBEhQSmZfgJMoEISAEhIDIXNeAEBACQqAECIjMS3ASZUKZEZBtQiAdAiLzdDhJSwgIASGQawRE5rk+Pe3vHFMemfrJW6n1eoMOupSpp9uTfObg8yo4U9xoh\/bS1OPlWtWvNH2opcO0U2zCNvpaS7c3eeAFbrTFluPe1Key+UJAZJ6v81Gk3mTeV14g4lVwtkxXZGpY5p1oUYMzZswwpigyVbFFTUTr94AbL2C1qg3V2z4ERObtwz43LeMN4hXisSF4inQOT5aXPXhrb+rUqdEHMPDm8OrQc+GlJtJdly1p1EF6qM8+aeQlCW16veFXaaiPeukLb0iiFy9fzQ7X44Urrxt70fc86vM8tmE+bT\/33HNGGnnoYgO2cIzwYsykSZOit5KpM56PLmnk1RPa4xwg1I3QZlI5bKBfYT7lG2kvqV6lFQ8BkXnxzllTewzB8LWbmTNnRq\/Q4\/WyyBeEwMserOTHW6ksY8oKhhAq68jwAgiCl8fyAnxowXXZ8pIPdaMPyaGLUJYlX5OMgJB4C5Iv7qALQc6ZMyciSOqjXvpCH+lbWAdtVbPD9bCLsggePuu9kBdvlxsG6Z7P\/muvvWa8nUm\/aNttoC7k7bffto8\/\/hhVoy+N2B0Viv1hbRrWaKE98AAX+hlT06EQqCBQSjKvWKedhhEgdMGjOOQZL+x5vPnneayxcffdd\/tht62TIoTsGZA\/X2hhTR1PYwsBsuY1n8HztyAhzcmTJ9vmzZtRqSm0xbomhCtQ9L6GdrAMK+kINxjInXZpJ3wzkVAHSytQjwsr\/\/kHFfCGscGXHqY+6nZd+sJ+GrvRSxJuWl4ePMCBJ4skXaUJARAQmYNCB4sTEV4oj\/NpH889BMAaMe6RxmFkiVSIjbAIdSOQ1MmTJ+OqduXKFYOM8UbDzPhxmBfu0xbeNk8PYXoj+9xgCO3QTzzjamX5FBp5fEWHLcK+39ToS1q7KZskPMFwbjwPHPzm42naCoEQAZF5iEaH7uOZ+uM8K9pBvpB1EhykQ3YQC+EFQgBOYkn6kDd61O8SesFJZbJOcxLnxkRoh366h9\/TvhTB7p7apnIgkD8RmefvnLStRzzOQ7TEpg8cOBANeIad8fACBE4oJvQcQz3fx5vE28br9rRqWzxqPGu82lAnfhzmhfuNtBWWY58PKBDGgMS5sZFWS\/DCyXcP3ff9CaU3faEuhJslISD2EXCIe+ukS4SAIyAydyQ6dAthEFphwNMhIDZLzJjYsaf59uLFi5WP80LuCxcurAz8uY5v8U4haEI4nsYgHrMvKOtpbLkxEHdmMBVPmTR0+QDH3LlzOawp3lYYHuEpAqlZ8JPMkDzBIqznE5XKBlwefPBBW7VqVTTYCYbE+13B+5LGbi8T3xKmQUgHD9bsT8LBb4KcM3TBlXXx2Zd0FgIi884637dYC4kykAeJEj5BIDafrUHIBe+agU48UdZZZ6YGenizHN9zzz2Gd0tdDCySD4ly\/MILLxhePvoIa3ezhjeEGO8MXjFhDsId6DKDI+3a194WREZZJLQj3lZ47AON2Eo5vGAGauk35Bjq+n6ID+Uefvhhe+CBBwyv3ftCeepDatntdYZbBlzXr18ffQQZPJ599lnjyQkdbjbcgLmJ0Bbnj5se7TzxxBMG\/ugh9J+bJzdGjiXlRUBkXt5zm9oySILwCmEGJAyhQLpdXV1GPnoQLjoIaRyT77NG2JLnM168PGkIuqRV6xz1oYdQP226Lnlh3zzdt9RL\/ZRFXNfTKe+69NPzIUT2KYPQd\/Kpi7Lsk4+el2efNPSRb33rW54VbSlHefIQ9kmLMlP84Ruo3Jgoi1TrO1WBEVihRzuzZ882+kYfaZO0sDxlJOVDQGRevnMqi1qMQJK3y\/RJQi+QZ4ubV\/VCIBEBkXkiLEpsBQJlqRPCJrxEOIPQBoJtePRseyLE6AmHcKPoSfk0ZQjLEJ4JY\/lpykmnGAj8fwAAAP\/\/0DdTTQAAAAZJREFUAwDO\/Dw2IGeUiQAAAABJRU5ErkJggg==","height":224,"width":371}}
%---
%[output:803f6bd8]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAXMAAADgCAYAAADxP4bLAAAQAElEQVR4AeydC8wV55nfH2edNabgW+NwtyCh8iYEWMfBkLDFTWViJErZRbJULgkSkQhuqLAKmBLFNjKrsOGyMlo7eFGKhGLDZqmIKXWEhaX2Y0MXTC0bEMqisgFhczOWgwFxadOl\/CZ+jt9vmHPOnO87Z87MnD\/i+WbmfZ\/38vxnzn+eed533vnMDf0TAkJACAiBwiPwGdM\/ISAEhIAQKDwCIvPCn0IZIARyjoC6lwkCIvNMYFYjQkAICIHWIiAyby2+ban9gw8+sEcffTQS9lvViZdfftm+8IUvGFtvg33Ej5u5xZa4XZcvX7bvfe97dvjw4WY2VbMu2ho7dmw3u2sV6Ekf33zzTVuyZEmtanuV51hy\/hDaa6RCzjHl2DZSrpW64Dxr1qyW4tbK\/ve2bpF5bxHMYfnPf\/7z1tXVFQn7reriggUL7De\/+Y2xpQ1+2KtXr2Y3E+HHO3\/+fNu3b18P2+tZsc2bN9s999xjM2bMqFtBT\/oIsWJX3cp7oXDo0CF777337Omnn47O4WOPPdaL2vJRtF+\/fjZp0iTbvXt3pjf3fFhv9pm8dCTv\/cBLwhNB8MrwzrzPkBjpzz33XOQNsx\/XiXtC1OflqQt9vAq8TMoj1Os6SVvy0UMoTz3oeVt4seyTTv60adMMQZ+y6LLlGEEHXdIpR3mEfdIgGfQowzFbP2bfiZwttkBk6CHskxbW52lhu7TPMfUiIU7U43LlyhWD8CDyS5cu2fTp0yNP2eukrAt983JsqZM82sEm+kTfKEt+vA9hefIgi3HjxpnfKMGHOqgToV70qC+pj7RBnei6ePv0hzLobN++PbqeqB8J28AGdKoJbVOn109Z6kCftr0NzlWYR74L+uR5HdXaJB0dt9vLYwvpLtRFneSDD\/pcjwg69It89Dim\/6+99lri0x\/5CHVQF3UiEydOZGPccKOdDvojMk9xsrlY+WG5qpMHF6unsf3Zz34WeTvso7Nw4ULj4kSeeOKJSh751MfFy74LxARR+PGGDRuqehiU5YfourQ3Z86cqvroHTlyxJD+\/fsbF31P6qCenoh7TXiDeIXUcfz48ag\/o0aNshEjRkR9xwZsIR8BJ37UkBPH9WTFihW3eOohjthMndRDO88\/\/7xduHCBw0gghngfwNnP9d69e41yI0eOjPTp11NPPdXt3JLPuf\/www8jnfgf6qLOMJ1z\/8orr4RJlf3z589bmuvHC9AnyJo6PQ3cJ0+eHGHsabW21a5ZfgthOewI8Vy1apXRPjguXrw4VI0wQj9M5HpEuCa\/+tWvWogl\/f\/pT38aqkc37LAOx5r+ojhgwIDoqenAgQPRb4+0ThGReZ0zzUUJwQ4bNiwiCcIKGzdujEpt2rQpunCjg5t\/JkyYYBAVwj4\/IPYR9v2RlmPyt27d2u2C44LesWMHdRj5XKjnzp27WXP3\/1y4lHV9+kTd6EM23bU\/PXL9gwcPGhd9T+r4tLbue4Ra6AOpbLds2WIQOMcu3EDow65du6Ik+kqfeTRGF2+KY8pjk+PEjxqJCn3yp2\/fvsZ5ACfqBDf6sHbt2ihsQHmEUAh1gqPjRhWUJZ8bBfmkId4nz6ddzr2f62PHjqFmTubRwc0\/6KBLnUjXzTDX8OHDE\/tISAMdF9q6WYVRN3l+TN+p58yZMxERxnHh\/GETZUOhHwjYgCHtUBY7wRicOKYMW9rwpwzSEAiaa5Y+UJ76sDFOkt6G5586dcp4aho9erRxnVEW4fxwnk6fPt3tN0MaeehevHgx+o3RDvXR97vuuovuRMJNYs+ePeb51AtW9JP+ooQdPDVxg+ack9YpIjKvc6b9xz1z5szKYzUXMOIXrlcxb968iMAgJvZJ5weKsI9HwaPhmDFjoos2fsG5h0p5CI4yScJFSlnXR4cfKBc3W46TJNTvaR1J9aZNw\/umDxACXrn\/MCEMSIl0fqgcUyc4hDiSllbwwMHaf+SUc5s5dwhptEWb7CN+rvBsKY8eZMG5xtOGjCAgbobo00fOFTroUoYbBMRDfi3BUSBMQFu19LxP9a4fr8NvlmBH\/0h3O+l\/mr55m1OmTKF4dO1D+giEGSXe\/IPttMHNdciQITdTuv\/nvBI2IQzGzaR7rhnXA9cF6d6m\/9aoFxvIQ7hJcB5CrB07L4seQlucb\/Y7RUTmdc503AOrpR6\/oGrpkle0C65R+7AxFH6c\/Pj5MUKyPF7jRUEO1cggLJ9mnzAAhMp54+YGiaUpV0+HmyfElKTHDTRsC6+SpxDIOkmfMAt9xEvGI8W7TNKrl1bt+sH2emVbnY\/t3KieuhmCev311w3vm5tgq9pNe5NqVft5qLdhMs9Dp7PsAx4YF2H4SMuPFcETgYS8P3iaeD0Ij+Wk88NC2OeRlh99KDxWk9eI0CdmU9AHhLJOEJAZx\/UkbR2QGB4ONmFfvXrr5UNy4PnSSy9FsWf3\/CD6wYMHR+EEiJ56aDPEkbRaAtm6d8\/TD+X5kXuZJJtpi5uL6\/i5gmDD8wTpfvnLX3a1xK2HeDjPEC1PdUmK7jm77fVukt4n6g37xH7S9eP6YAcG9MHtBGOwJq2WxOugHp44IGiIulZZ8rAdDLh50x7XEMfkVRNv039rtIkNrs9vjd8cT1Jc99jvkhTW83KdshWZ1znTxP4YOOIH74\/R\/mjHIyAXqlfBBQaJIOxz0bGPsO+PyXhlCD8OLlgvn3aLJ8ujKPr0hbrYQpJz584lua7Uq8N\/OPwAeUTGBmyqW\/FNBezk0RpyvXnY7b\/jSSKYUC\/7CH3HBspjE3m0Ce4IOknifWQwk3w\/V16eNCTJZkiANslH\/GYDnvTBhZsk5xoypD3ICX1sxFbXY0v\/qZO60EEoA47UwzHibaDPcVwgYMjzs5\/9bBQnRo\/6XapdP2CFgB0YoE9Z+gTG8XaSjnmi4fx4HdTDPr8FzmFSmaQ02qV9bE3KD9PoMxKeP+LorgP+3Bw8n3pdcGZcjy22cvNmv1NEZJ7iTONxcXG7KhcKj41xr+jb3\/529KNDD50XX3wxijVCItu2bavkkc8P5YUXXohi7Bw3Kjza46mF5datW2eN\/NBq1cEPZ\/ny5YYdtEF\/V65cyW5VASP0qip8kuEeqYdYPkmO+s6MDm+TdOqEcOkPx6GQxg3V0\/r06WPPPPOMH0aDyJSlPveGsZk6UaKvP\/nJT6LZDxwj4BfvAwTDLBnyve\/uTSedW\/T8XMT7SB7njbbZp2\/0kTZ4quDmwD5CPnLfffdZI9cPbfJkEdZBewzkYx911pMku6jPcahXnhlB6Lse\/QF3QmuMl3h6uKXf\/CaGDRsWJaNPmCY6+OQP5w\/8PjmMNhz7bxH8wDGMxUdKHfCnLWTOYxp3WLZJGHOX9TsuW7yTarpJ5VuRBqH7Ix2P3Ek\/ikGDBkUv6qAX1+HHweAReQj7pNFX6kKfHzUXNGlctOj5RUpaXFwHPcR1qZf6EfaT6ve6qtVBvpejbuqaPXt2NFOEMuSzJY8tx7SFHmlsOSY9LvQTHTCN54VtxnWoj3oR9inrdblueAyeEArY0hZPQXizeLwQAPXcuHGj29RE6oz3gXr8vOChQoxO5ujTF+qiDy70gzyEfU+nH6E+faOPtEEd5NEWx5Qhn\/6QTj5pCPukUX+ShHUk6XPOSGebVJ406qcd9BD6RL3kUY40thyTTj76lPNjdBAwwHa3B5vYpwy61MFvHM+f0B7OEvohzuggtEmdLhyTjvDERHmeoLxe0jtBMidzflDMRf3444+r4ssJ5K7sJ4uTzsmvWkAZQiAFAvy4+ZGj6o\/\/hD4IgeBckE9eLYGoeKLA+8MLrKWrvMYQICzCWBDng\/OCI8d54uklDFnVqpVYPflpw0nolkUyJ3Pibu+\/\/77dfffdVTGEzH0wpKqSMoRADxAgTIAnHBbFcQi9uzAvth8dQhR4f8yDjhL0pykIcKMkNAl5hxV6yCpMS9rHUWSQHu++E52\/TMkcT4bR6WeffTbpXERpnJBwBkKUmPM\/EAFPEWxz3tWO7x7eN4\/2nC8XHucbAQai4GmR0EEj5aRbHwHH1s8N27Q4+7lt9HzW71UxNDIlc2KVPM7yOFUNHn8xgMcrHrMQXgCppq90ISAEhIAQsOwW2mJw45133jFGuWsB7wMYjH5zV2YghLU1GBRNKkfIhtBNp4vs3xe9VSschENW1wDck8RJ7UrLzDPfvHlztFgQj0K1jPXHLH+04pgYmE8tC8sC5tKlS40ZCpJZwmGWMNDvILtrAO6Bg0JOaud+JmROrJzRf14cIGzCSDUgsK3mccdBSRoQpY79+\/fbmjVrjDhoGWXRokURFGW2kfPWCXZ2go2ddC7hHjgo+oHm4E\/ryfymkYxSd3V1RXOUPXQydOjQaL0G98BvqkX\/IXfeqOMGQALHvOxQa2oSr\/gyQ6GMwg2P2RMPP\/xw9BJMGW3Epk6wsxNs7JRzOX78eOgpV5IJmdezGMLm8ZCZLJA7b\/FxUeDF482nnZpUr50i5g8cONC+853vGNsi9j9tn7Gv7HZ2go2c706xE1vzJG0hc+LgzAdlCxgQOI9nHk\/nGA\/ehWP0JEJACAiBBASUdBOBtpD5zXb1XwgIASEgBJqIgMi8iWCqKiEgBIRAuxAQmbcLebXbEQjISCGQFQIi86yQVjtCQAgIgRYiIDJvIbiqWggIASGQFQIi86yQzls76o8QEAKlQkBkXqrTKWOEgBDoVARE5p165mW3EBACpUIgh2ReKnxljBAQAkIgEwRE5pnArEaEgBAQAq1FoC6Zs+AVC1+xTkpaQb+13VbtQkAICIGeI1DGknXJHKPvuuuuaIVDXyul1paPSaBPOYkQEAJCQAhkg0BdMmf52p07d5ovilWvW+ihX09P+UJACAgBIdA8BOqSuYdZCJ2w37ymVZMQEAK3IKAEIdBDBOqSOZ7566+\/bv4BCOLmrD\/ew\/ZUTAgIASEgBFqAQF0yp03WGWe9cWLlGzduND4YAan7ByXQkQgBISAEhED7EEhF5mH3+FAEpH7o0KEoecyYMTZ27Fg7fPhwdKw\/ZUVAdgkBIZBnBBomczcm9NZfeeUV+8EPfmCKqTs62goBISAEskWgx2TO9zoJsxBumTNnjv3oRz8y4uvZdl+tCQEhIASEAAg0TOYMfkLghFeogHDLwYMHU09dpEyTRdUJASEgBDJF4Hef+yO78tV5turvLtuvjl3ItO1qjaUi89ALZ\/Dz6aefNuLmDIoSbqlWudKFgBAQAmVE4L3fXrP\/88BE+7szf5Ab8+qSOXHwqVOn2pEjRypvgS5YsCA3BqgjQkAICIGWI1CABuqSOXHwrq4uUyilAGdTXRQCQqBjEahL5njm06ZNSz31kCmK6HcsojJcCAgBIdAGBOqSOX26ePGiTZ8+3Rj4rCfooU85iRAQAq1GQPW3A4GTH11tR7M126xL5h5mYcAzrRCWqdmqMoWAEBACQqCpCNQl86a2psqEgBAQAiVD4IH7+uTCLtxKDQAAEABJREFUIpF5Lk5DKTsho4SAEMgQAZF5hmCrKSEgBIRAqxAQmbcKWdUrBISAEMgQgYbJ\/OWXX67MauHVfoQ1WnhLtBX9XrJkiSHNrFt1CQEhIAR6g8DJj671pnhLyjZE5pDqnj17bN++fTZhwoSoQyyJO3jwYFuxYkV03Mw\/3Ci2b9\/ezCpVlxAQAkKglAikJnNeHjp69KgtX77c+vbt2w2MuXPnGnnodMvoxQGe\/rZt22zo0KG9qEVFhYAQEAKtRSB5Nktr20yqPTWZJxX2tHPnzlmzXxRijfSHHnrIHnnkEW9GWyEgBIRALhDwMEteiBxQUpM5Lw+x4NaqVavsypUrlI0Eb3zlypU2c+bMpq1nTp2Ec2bMmBG1oT9CQAgIASFQG4HUZE41rJY4b968KF5O3JzlcImdQ+TkodMMWb16tdEON5A09e3fv99OnDhh165dK6Vcv37drl69WkrbwnPWCXbmwMZMrqMy2wnX7P3H369hPuzefLwwBE82ROYUYMAz\/lp\/M4mchbouXboU3TBoL42sX7\/eNm\/ebGfOnCmdnD171s6fP2+Esspon9vUCXZ2go2cz7Lbuelv\/1uFlgoZZqn0vsU7e\/futd27dxtfMmJRL2azIEx\/rNb0mjVrjEHYQYMGWdlkwIABdv\/999vAgQNLZ1t4rjrBzk6wkXNadjsffPhPKlRUSDInjv3oo49W5phDtEnC9MWKpT3YwcsPPX\/i5ghfNapW3ZAhQ2z48OHWp0+f0smdd95pd9xxR+nsip8r7Cy7nZ1gI+e17Hb+zT\/cqFDRxC\/eW9lv907qMAvxa2Ljw4YNi+aZO+ESOyfNPyWHQb0ldOqQCAEhIATyhgCzWDxe\/pkrH9rtH\/5DbrqYmszxzLdu3WrPPPNMt1krkDxp5KFDuIM5582ycO3atYY0qz7VIwSEgBDoKQJO5JT\/w5N72eRGUpN5bnrcjo6oTSEgBDoeAbzy72\/9dYQDXvkfnvyf0X5e\/qQmczxwwizMKccDdwPYJ408dBjA7N+\/v2drKwSEgBAoPAIhkWPMvxz0\/wxCZz8vkprM6TCDk4RUmFvug5\/sk0Yea6ls2LAheuUffYkQEAJCoOgIOJF7iIUZLMseH95ss3pdX0NkTmtJ88xJ87yDBw\/a6NGjOZQIASEgBAqNAET+b3\/yjoVE\/l\/\/\/UO5tKlhMs+lFeqUEBACQqCJCEDiP37juP3xn\/+9sU\/VeOQQOVuO8yYNkTlhFA+vxLfMQSd+njcD1R8hIAR6h0AnlYa4IXG88R+\/caJi+sQv3mPv\/vDrllcip6OpyZwlaTdt2mTMJz906FD0uv3GjRujOefMM\/cBUCqVCAEhIASKhMDJj64ZJI4nDolzTP8hb+LjO7+fz9AKfXRJTeaslMiaKRMnTrR+\/foZH6Q4duxYNOecAVBWOYTwvWJthYAQEAJ5RgDChsCnvfROFE6BxL2\/TuJ448seH+HJud6mJvO4FSNHjjTInHTWYoDoIXyOJUIgEwTUiBBoAAHI+1fHLkQeeEjgPrhJVUUkcfqNpCZz5pA\/+OCD0eqEFITMT58+bXjjPrc8\/gUi9CRCQAgIgXYg4OS99cBZc\/L2WHicwGeOG2gMbhbJE49jmprMKUi8\/MCBA8ZAKNMRCbWwuqHPLSf8gp5ECAgBIZA1ApA3xM1bmiF5cxySN\/3CA4fAX5r5pWhgk+2fjLyHrMJKQ2SOd97V1WUQORazZgoLbmluOWiEon0hIARaiYATt8e87\/uP\/z2Ke0PcEHoSeTMjZdnjwyseOAQOobeyn1nWnZrMmXY4bdo04+MR8Q6SRh468TwdCwEhIAR6igCkTZwbgoao8bhD4mbQMk7ctIXnHSdvZqQsuzmYWXQPHPuSJDWZJxX2NL6C0+wPOnvd2goBIVB+BGqRNnFuiBxCr0bckDdeNt62x77LTt7xq6IumRMf5wUh1mA5cuSITZ8+\/ZYPVMyfP9\/GjRsXTVOMN6BjISAEhAAIxAkbgg497XqkTR2QNh53nLgZuITISS+r5439taQumRMfJy7ORyhGjRplO3bsMI7jQvy8VkPKEwJCoNwIQNbIvhOXbeevL9uibf\/bahF2NU8blCBtBHKGpN3bhrR3fv8hI428TiVuMIpLXTL3Agx+7ty5U4toOSDaCoEORACy9hg2g49O1rw56TJj42Fb8eaHtu2dD60WYQMfhF3N04a4Q9JGlzKSZATqkjmDmqy7QqillqCDbnIzShUCQiDvCEDUyK+OXYhIGKJGCIVA1D7w6OEQBh+drClXzT5IOCRsCBpP+6O\/\/KZB2HFPG\/1qdSm9OgJ1yRyPnOmI8bBK\/BgddKs3pRwhIATahQBkizhRQ8JO1JC1EzWk7WSNDsKgI2Wr9R3yRQh7IGv\/dLhtnDHQ9i15yJIIGx2FR6qh2fP0umTe86pVUgjkDYFy9geiRZyow\/BHNaKGyJ2oIetayEDUCCSM4FkjoXeNh00aMuuRQfbwkD65XmHQSvivYTJ\/+eWXb5nNQloJsZFJQqCtCEDQiJM05AsJI5A0XnSSRx2GP9IStYdBnKwhasQ965Cs0UHkXbf18ril8YbIHNLeunVrtOyth1mY5UIaebfUrgQhIARuQQCCRqqRNESdRNKQOISOQNLUcUvlQQLeNBInarxniBqCdvG4NXlO1CLrAMwC7KYmcwY3IW2Wuw1j4+yTRh46BbA5j11Un0qAAOSKVCPpCWvfsYf\/6oSxxav22HScpCHqenBA0ogT9bLHh0fT9SBpJPSo40TtZE35eu0ovzgIpCbz4pikngqB5iEAOSMhQcdj0hBzMzxpeg3BIk7SEC\/eMgJJ40knEfWyx0cYunjTCHVJOguB1GSOB87XhFauXGmhB84+aeSh01nwydoiIgA5I0kEjZdMmKMWQcdj0tRVDwcIevBdt0cDg5AuAkEjtUiafHQRSJp66rWl\/M5EoC6Zh7AsWLDAIG1e7fc55+yTRl6oq30hkCUCECrSW4JOG4922yBXxD3pZZ+EOyDhOEn\/r\/80Lpqyt+7PRkQhEQgaEUk7mtr2BoHUZI4HzotBNOaDn74VkYOKpNkIQM5ISNB4zgjeM5LGg+4tQUO4kDMCQSOEOhDCHojHpZd9Eu6gjEi62VeE6quFQGoyJ4TCQOfq1asrUxNZhKtW5coTAiECEDMSkrPHnxf\/4rjN3342Ghwk\/oxA1EitgULqC9uoto\/3jLgHDdlCzgjkjEDKSQSNDvoIBI1Ua0fpPUFAZZqBQGoypzFfdAuPnK8OsVoi4ZaxY8cmrnNOGUl5EYBIkSRy7on3\/Papa0Z9aRCDmBHIGYFoEYgXgZyRRgia+tK0LR0hkEcEGiLz0ABCK5A6MnnyZFu4cGG3gdFQV\/vFQAAiRZpFzkyxQ6gzDQIMEEKoITkvqxGDhqgJbyAQOAKhI3jPCPWlaVs6QqDoCPSYzJcsWVIJt\/Bd0G3btmk985xdDZAo0i5yhkiRWuTs3vPpH33dds4dGq3nsfOTJU4h52XtjUHn7IyqO0KgOgINkTlveRJWQXbv3l1Z2zzNIluXL1+2WbNmVW4A3AyqdYtYPG24KIzze6QgZiRP5IwXDOkiEDOCx1wt9hwnZ3nPvz+3+isEeotAajJnNgtveW7cuDH6OMXBgwcbWtt8xYoVNnjw4KgsSwDgzXNzSDLg2LFjNmPGjEiXME6jbSXVmdc0yBlJImhmazzxn39t0za\/b4N\/8PfRB2vjA4I+59lnbDQS1sBrRtJ4ztXIGRKH0BGIGaHOvOKtfgmBsiKQmsyZzYIHziBoT8DgS0QIZamLz8xB2hzHhfSRI0fGkwt5DFEjkC3CwCACUVebseEEDTEjpy\/+7hbbkxIgUaQ35CzPOQlZpQmB\/COQmsybaQpePp75lClTbqmWcMzp06dvSS9CgpO2T7eDsPGkEQgcgdARSLqeTRAzg4IsJ4rniyyLDQgqrFEPReULgc5AIHMyJ1bOW6NDhgwxtnGYr1y5YqdOnbJwPnu1cIyX3b9\/v504ccKuXbuWqfzmg8uVL7K4lw1hh5619zG+dZL+s9H3GLL2T4cb8rfz\/igaBGRAkMX9f\/XUaPurqffan08ZaLw5uOjRQZE+Zb42tI8hn+9rmdrdKpyvX79uV69eLYUt1TDqBBuxvcx2wjVwVPw33e7j5pN5HYsItRAHJ37OPHU88bDIuXPn7MKFC+ax+R07dtiGDRuMQdFQL9xfv369bd682c6cOdNyefvoe7bitZtx7Jfesa\/9xQGDvPG0w\/6wD1kj077Uz5AVj30uepWbGRtv\/4fh0cwNvsbyw391jyHfHGaGfPGfXbPPXv9txY6zZ8\/a+fPnDVyysK9dbXSCnZ1gI9dP2e2Ea5YuXcrPPFeSmswJjUybNi3x5aDDhw8beeiktW7u3Ln2\/vvv2\/Hjx7sVGT16tDHg6bF5jpnHvmvXrm564cGaNWuM+gYNGmStkv97x03v+H9ciAYjN751wXjBJewDIRHCIAwIEvpgHQ7kr7892pD5\/\/pf2L\/52gh7+MFhDfVxwIABdv\/999vAgQMbKtcqHFpVbyfY2Qk2cn2U3U64ZtGiReHPPxf7qcm8Vm\/xGi9evFhL5ZY8ytx2223Gib8lMyGh1oAoIZvhw4dbnz59mi4fXDFjRsmEte\/YLw5fqPQsJG+f6QGRQ+jM6GhWX+6880674447mm5Xs\/rXrHo6wc5OsJHrIQM72\/p7gGvGjx9f4YK87NQlc8IbzPcmvn3kyBGbPn16Za446QjhEmanMEulmmHEyhHyCa1s2rTJksrQHgt6uZfPMXPaJ06cSNHM5ORH14yBTAYvw8FKSBzSZi41W8g7s06pISEgBIRAFQTqkjnhDmLczA0fNWpU5UUh0kIhFl6ljSh5xYoVxiwVyH\/MmDGVNHYgbF4oguRpjwW9uHmgy41i3bp1Dc1pp87eCHO+Wdzpx2+cqFQDaRM+gcTZr2RoRwgIASGQAwTqkrn3Ea97586dPSbVfv362ZYtWyovArFPGvVD4PHj8EZBPnpZCN44RI5nTnt44pA4XjjhE9IkQqAaAkoXAu1CoC6ZE+4g7PHqq68aW7zlJCEP3XYZ0ox2IfLQG2dON564SLwZ6KoOISAEWolAXTLHI+\/q6rLZs2cb29BjDvfJQ7eVnW1l3SGR441D5MseH9HKJlW3EBACQqBpCNQl86a1lOOK4kT+4r\/7kpWOyHOMv7omBIRA7xFITeaEUAilJIVYSCMPnd53KdsaGOz00AoeOUSusEq250CtCQEh0HsEUpM5IRRCKWFohf1Dhw4ZL\/W8+OKLhVvPnEHOhX\/z6wqKzFIRkVfg0I4QEAIFQiA1mVeziRkpTzzxhK1atcqYWlhNr3p6+3J4FR9CpwfLHh+u0ApASISAECgkAr0mc6zmLc5Lly4Zi2RxXARhPRV\/GYjwyjINdhbhtKmPQkAIVEGgKWTOwjP9+\/e3vn37VmkmX8l44wx60iuInCJz1ycAABAASURBVDg5+xIhIATKiUAnWJWazBncZJCTwc648Lr98uXLjZBLEUDbeuCMQej0lQ85KE4OEhIhIASKjEBqMq82AMogKKscsrphEYCAxMPZK7zZWYR+q49CQAgIgVoIpCZzKmGAkzVU\/GMRfuwLaKGTd\/HwCv1UnBwUJG1FQI0LgSYh0BCZs1gWH5VYsGBB1DxhFT4iwQJaRSB0vHIGPuk8sfKZ4wayKxECQkAIFB6B1GROzPzo0aPRRyBCqyF04uXkoRPm5W2fWLn3SV65I6GtEBACZUAgNZljLB+g4KMS7IdCGnlhWt725ZU3ekakLwSEQJEQSE3mDIDOnDnTVq5caaEHzj5p5KGTV+Mhc4T+MYOFrUQICAEhUBYEUpM5BhMrDz8cwRRFPiIBkZOHTl4lHPicOW5QXrupfgkBISAEeoRAQ2ROC3wogumIoWRM5HSjIcEj97c98co1r7wh+KQsBIRAARBITeY+DZFPvBXArm5ddCIncdYj8srBQSIEhEC5EEhN5qy7wvorrMNSNAi2vHUm6rKmI0Yw6I8QEAL1EChgfmoyZ3Bz6tSpxjosRbIzDLEMu7dPkbquvgoBISAEUiOQmsyZtbJ161bbvn27MfAZF9ZtQSd1yxkphiEWzS3PCHQ1IwSEQOYIpCZzPPOkj1P4QCh56GRuQZ0GwxCLBj7rgKXsAiCgLgqBZARSk3ly8XynKsSS7\/Oj3gkBIdA8BOqSOaETQiivvvqqsY2HV\/yYPHSb17Xe1wSZey3yyh0JbYWAECgjAnXJnNAJIZTZs2cbWw+rxLfkoZsnkMK1WCZ+8d48da2MfZFNQkAItBGBumQe9s3nmhdlCVz3zJmSKM88PJPaFwJCoGwINETmRVoCFyL3mSyakli2y1b2CAEhEEcgNZkTD2eZ27lz53arI69L4ELm3lFNSXQktBUCQqCsCKQmcwBgmVuWu2U\/FNLIC9PavR8urNXuvqh9ISAEhECrEUhN5gxusjoiy93ipXvH2CeNPHQ8PS9bxcvzcibUDyFQZgTab1tqMqerrI6YxRK4LOblUx7Hjh1rhw8fpvnUQohF8fLUcElRCAiBEiDQEJljb6uXwIW4n3\/+eduxY4cx\/fHJJ5+0hQsXdvsgBv2oJZC552sWiyOhrRAQAmVGoGEybzUYo0ePtj179hhb2po4caLduHHDiMtznEb2\/uNvK2qaX16BQjudiYCs7hAEckfmcdz37t1rQ4cOtREjRsSzqh7\/6tiFKE\/x8ggG\/RECQqADEMgtmRNuIV6+evVqmzdvnjEFstr52L9\/v504ccKuXbsWicfLB\/W\/PTr29KJur1+\/blevXi2FLbXOQSfY2Qk2co7LbCdcc+rUqWp01Lb03JI5YZaDBw9GsfPFixcbg6LVUFq\/fn20zvqZM2fs7aPvVdTGDrjdSCuynD171s6fPx+FmYpsR1Lfw7ROsLMTbOSclt1OvumwdOnSCs\/kZachMndv2WeahNtWLbQFqU+ePNl27dpVFbM1a9YYLzMNGjTIrv5Bv4relD8eZqQVWfiy0\/33328DBw4svC21zkMn2NkJNnKOy24nXLNo0aIKz+RlJzWZsy7LqlWrDGJllklcWrXQFu2ePn3aRo4cWRWzIUOG2PDhw61Pnz729qlrFb3bb789SiO9qHLnnXfaHXfcUXg76uHfCXZ2go2c57LbCdeMHz\/e8vYvNZn7N0C5K7XSCLz\/SZMmVeaW79u3z44cOWLMaknTblMGP9M0JB0hIASEQI4QSE3mffv2tf79+0ex21b2n7DKs88+a9OnT48+Tzd\/\/nxbt25dZapivbZ98FOLa9VDSvlCQAiUCYHUZM5sEjzmTZs2GaGPVoIQfzGJ4zTtnfzo0xCLXhZKg5h0hIAQaBMCTW82NZmzBsvWrVuNsMeYMWMirzmLAdBGLA7JvJFy0hUCQkAIFB2B1GTOIloMcsYHPv2YPHTaCYje\/Gwn+mpbCAiBdiKQmszb2cm0bfvgJ\/oKs4CCRAg0joBKFBOBhsmcT8Z5eIUXeZBZs2a1PI7eCLy8xt+IvnSFgBAQAkVHoCEyX7JkSbQIFnHzCRMmRLYzODl48GDjk3JRQhv\/aCZLG8FX00JACLQVgdRkzgAon41bvny5MU0x7DVzz8lDJ0zPcj8c\/FSIJUvkM2xLTQkBIVAVgdRkXrWGmxksT9vuz8aFZH6zS\/ovBISAEOgoBFKTOTNVpk6darzSz9ugjhLeeB4+G6eZLH5GtBUCQqATEUhN5oCzYMGCaDla4uXEzXk7k32+\/0keOu2S0DPPfgC0XVarXSEgBITA7xFoiMwpwoCnzy33bbuJnH45mUPkCGkSISAEhECnINAwmecVmPd+++mr\/Hnto\/olBISAEOgpAvXKNUzm4TzzcL55vYZamQ+Ru2euBbZaibTqFgJCIK8INETmELmvz+IhFmLnDICSlwcjNS0xD2dBfRACQiBrBFKTObNWIPJnnnnGmNniHWWfNPLQ8fQst+6V0+YD993JRiIEhEBWCKidXCCQmsxr9ZbPRN111121VFqaF67JojBLS6FW5UJACOQUgdRk7h744sWLK18Bwia88YULFxpz0NEhrZ3ywH192tm82hYCQkAItAWB1GQOaRMbv3TpUuUrQAyAMs\/8vffes9WrV1fWOOfjzlla42uyQORIlm2rrTwjoL4Jgc5BIDWZ43WzZrkPfNbaotc5EMpSISAEhED7EUhN5u3vavUe\/O5zD0aZipdHMOiPEBACHYhAw2TOFETCK6GQ1i7s\/qnv5ypNFyjEUumzdoSAEBACzUCgITKHtJmCyNxyD7OwTxp5zehQo3X8U99\/XikiMq9AoR0hIAQ6DIHUZM4AKKTNnHLi544T+6SRh46nt2M78Yv3tqNZtSkEhIAQaDsCt5B523vUYAc8Xt5gMakLASEgBEqFQGoyxwNnqVumJ4YeOPukkYdO1ugoZp414mpPCAiBPCKQmszpPEvdQtrMLfcBUPZJIw+drMXJnHg5knX7ak8ICIFGEZB+KxBoiMzpAKTtg5++JY08iRAQAkJACLQHgYbJvD3drN6qx8w1x7w6RsoRAkKg\/AgUmsw9xFL+0yQLM0JAzQiBwiJQaDIPUdc65iEa2hcCQqDTECg0mYee+QNax7zTrl3ZKwSEQIBAZmTOFEZWU\/RZMLNmzbLLly8HXfl0980336yswIj+2LFjuy2765p8Ls73yx4zdzu1FQJCQAgkIZAJmUPaTz31lDGFkRkwhw4divqyYsWKaBv\/c+zYMZsxY4ahixw8eNBGjx4dV7OTH129JU0JQkAICIFORCATMu\/Xr59t2bLFfAojx5MmTbLTp08neueQ+ciRI+uej+6fi9NHKeoCJgUhIARKi0DvybzJ0ODFQ\/JpqhWZp0FJOkJACHQCAm0hc+LnLMyFd46XHgJ95coVO3XqVLcvF9VbkfEBfSouhFD7QkAIdCACmZM5njfx8yFDhticOXNugfzcuXN24cIF27hxYxQz37Fjh23YsMEYFI0r+wDoZ658aCdOnLBr166VUq5fv25Xr14tpW3hOesEOzvBRs5pk+3M1bUP1+Bwxvmo3ceZkjlEPn\/+\/MhmyDrulZPBQCcDno899hiH0cDn5MmTbdeuXdFx+MfDLO+\/f8o2b95sZ86cKZ2cPXvWzp8\/b9zkymif29QJdnaCjZzPstsJ1yxdujSkolzsZ0bmTuSDBw+OBkOTiLwWIvEBUSdyynx99EibO3euDRo0qHSCfb\/85S\/ZlM628HxhYNnt7AQbOadltxOuWbRoEWbmSjIjc5+G6NtqKBBOYT46cXV0ON69e7dNnDiRw4qEZE7MHG\/g3XfftXdLJkeOHLGf\/\/zn9sYbb5TOtvBcFdXO0IZ6+51gIxiU3U64ZujQoRUuystOJmQOMR84cMD4xNyYMWMqLwQ5aUPY\/hIR4RW+XMTSurwwRFhm3bp1UbglBA0C9+Nf\/OyvjfJlFH+cW79+fWlt5Lx1gp2dYGMnncvx48dbnkg9EzLnoxVdXV3RgCYvAbmQRh4Ezjx0D71w7DpsOXbi9i1k\/tFfftPe\/eHX7b\/8xfej0A11SLYIiy3CQL+D1l8Da9as6TwydwJuxRZSx4uXTLBuGEzQsfDQNdDKayBPXjncmolnTkMSISAEhIAQaB0CIvPWYauahYAQEAKZIdAGMs\/MNjUkBISAEOgYBETmHXOqZagQEAJlRqCwZM56LUxdRNgvw0lasmRJZdom0zWr2RTqVVvrvVrZdqeHfa9lo\/eTl82Y7la0c5zWTqbtMkWX6xhJg4lj0+5tWhsPHz5sXKfYx5bjVvc9q\/rBIC\/XZiHJnIuBqVes24KwT1pWJ7AV7fAj9rn4LHWwcuVK44ceb4sLh1UlWROeaZtPPvmkLVy4MFE3Xrbdx2ltDPv5yiuvRO8nhGl5309rJzcq1ikaN25cNG2Xa\/n5559P\/BBL3mxOayPXMNcn74oU7XqthzlEvn379npqmeUXksz37t0bze8cMWJE9DLRI488YqRlhloLGmLtGX7UzLtnOhULkUHY8aYWLFgQzSP3Ofm8GXvjxo1o7Za4bt6O09ro\/YYIXn\/99ehce1oRtmntPH78uF26dMmefvrpyCzWJdqzZ090TUcJOf6T1kbWFOL6HDBgQGRNka7XqMMJf7gueZo6evSojRo1KkGjPUmFJHM+XsEaL05oQEca2yIKHhrednz9mSLbFD8PPbFx9erV9t3vfrdQZF7XzgAYHJD+\/ftb3759g9T87zZiIyR+2223VZwNbGZ+No5Y\/i2t3sOXXnrJWMab81ddK9ucQpI5EIXEF+6TV1RxO7hJcbNKYwcruBXpx5HWRsJmeK3f+MY30sCQO520dnKeV6xYkWqsJG9GprGRJ02erjZt2hTZiINCWJRrPG\/2pO0PNn3lK19Jq56ZXmHJPDOEctwQ8XMWIVu+fLkV+ccRhxjP78UXX4zGAormtcZtqXdMzHXKlClRzJyxksWLFxciZl7PLs\/npkxoZd68eZGN2EqIglCF62jbHAQKS+bc4R2CcN\/Tirh1OyAzwi61bIDI+WgHA4TEWmvppsvLRiuNjSzIxuNrkeyKo5fGTsowPoL4PjFYQhEc513S2Igt2BTayHgQN7G821e0\/hWSzP3xLgQ7KS3Mz\/M+XjWP2\/E+VrMJIideh1deFMJrxEYG1\/ixM5WNVTYhd+LnzB6IY5S340bsrHZ+q6XnxdZGbMxLnzuhH4Ukcx7b3nrrrehxlMc49kkr8gnj8ROC5vET8uKzVBBZ3CamhOGRE4YgdhfPz\/NxWhvXrl0bPZIzlY0ZPXh1zPggPc\/2ed\/S2sn55TxzvinLlmPSOc6zpLWR3yXrm2Mb9rDlmHSOJc1DoAhkfou1eKO8SDJ9+nRD2CftFsUCJbDML1MTIS7WcGdNdydrPHH3SvFaGRjEbjxXF0g+7+amtTHvdtTrX1o7Ob\/btm0z3ingPBIvL8qLuNRxAAAGQElEQVRNOq2N\/C6ZY8417TYWLTRY73znJb+QZA54zLfGc0PYJ63ogueJPQg\/FrcH+8jjmC35cQn10curhP0P+xzaGPadR3pmP5Afpud9P62dEDrr+nM++fYt5Jd327x\/aW3kPGMfUjQb3dakbd6uzcKSeRK4ShMCQkAI9AiBEhQSmZfgJMoEISAEhIDIXNeAEBACQqAECIjMS3ASZUKZEZBtQiAdAiLzdDhJSwgIASGQawRE5rk+Pe3vHFMemfrJW6n1eoMOupSpp9uTfObg8yo4U9xoh\/bS1OPlWtWvNH2opcO0U2zCNvpaS7c3eeAFbrTFluPe1Key+UJAZJ6v81Gk3mTeV14g4lVwtkxXZGpY5p1oUYMzZswwpigyVbFFTUTr94AbL2C1qg3V2z4ERObtwz43LeMN4hXisSF4inQOT5aXPXhrb+rUqdEHMPDm8OrQc+GlJtJdly1p1EF6qM8+aeQlCW16veFXaaiPeukLb0iiFy9fzQ7X44Urrxt70fc86vM8tmE+bT\/33HNGGnnoYgO2cIzwYsykSZOit5KpM56PLmnk1RPa4xwg1I3QZlI5bKBfYT7lG2kvqV6lFQ8BkXnxzllTewzB8LWbmTNnRq\/Q4\/WyyBeEwMserOTHW6ksY8oKhhAq68jwAgiCl8fyAnxowXXZ8pIPdaMPyaGLUJYlX5OMgJB4C5Iv7qALQc6ZMyciSOqjXvpCH+lbWAdtVbPD9bCLsggePuu9kBdvlxsG6Z7P\/muvvWa8nUm\/aNttoC7k7bffto8\/\/hhVoy+N2B0Viv1hbRrWaKE98AAX+hlT06EQqCBQSjKvWKedhhEgdMGjOOQZL+x5vPnneayxcffdd\/tht62TIoTsGZA\/X2hhTR1PYwsBsuY1n8HztyAhzcmTJ9vmzZtRqSm0xbomhCtQ9L6GdrAMK+kINxjInXZpJ3wzkVAHSytQjwsr\/\/kHFfCGscGXHqY+6nZd+sJ+GrvRSxJuWl4ePMCBJ4skXaUJARAQmYNCB4sTEV4oj\/NpH889BMAaMe6RxmFkiVSIjbAIdSOQ1MmTJ+OqduXKFYOM8UbDzPhxmBfu0xbeNk8PYXoj+9xgCO3QTzzjamX5FBp5fEWHLcK+39ToS1q7KZskPMFwbjwPHPzm42naCoEQAZF5iEaH7uOZ+uM8K9pBvpB1EhykQ3YQC+EFQgBOYkn6kDd61O8SesFJZbJOcxLnxkRoh366h9\/TvhTB7p7apnIgkD8RmefvnLStRzzOQ7TEpg8cOBANeIad8fACBE4oJvQcQz3fx5vE28br9rRqWzxqPGu82lAnfhzmhfuNtBWWY58PKBDGgMS5sZFWS\/DCyXcP3ff9CaU3faEuhJslISD2EXCIe+ukS4SAIyAydyQ6dAthEFphwNMhIDZLzJjYsaf59uLFi5WP80LuCxcurAz8uY5v8U4haEI4nsYgHrMvKOtpbLkxEHdmMBVPmTR0+QDH3LlzOawp3lYYHuEpAqlZ8JPMkDzBIqznE5XKBlwefPBBW7VqVTTYCYbE+13B+5LGbi8T3xKmQUgHD9bsT8LBb4KcM3TBlXXx2Zd0FgIi884637dYC4kykAeJEj5BIDafrUHIBe+agU48UdZZZ6YGenizHN9zzz2Gd0tdDCySD4ly\/MILLxhePvoIa3ezhjeEGO8MXjFhDsId6DKDI+3a194WREZZJLQj3lZ47AON2Eo5vGAGauk35Bjq+n6ID+Uefvhhe+CBBwyv3ftCeepDatntdYZbBlzXr18ffQQZPJ599lnjyQkdbjbcgLmJ0Bbnj5se7TzxxBMG\/ugh9J+bJzdGjiXlRUBkXt5zm9oySILwCmEGJAyhQLpdXV1GPnoQLjoIaRyT77NG2JLnM168PGkIuqRV6xz1oYdQP226Lnlh3zzdt9RL\/ZRFXNfTKe+69NPzIUT2KYPQd\/Kpi7Lsk4+el2efNPSRb33rW54VbSlHefIQ9kmLMlP84Ruo3Jgoi1TrO1WBEVihRzuzZ882+kYfaZO0sDxlJOVDQGRevnMqi1qMQJK3y\/RJQi+QZ4ubV\/VCIBEBkXkiLEpsBQJlqRPCJrxEOIPQBoJtePRseyLE6AmHcKPoSfk0ZQjLEJ4JY\/lpykmnGAj8fwAAAP\/\/0DdTTQAAAAZJREFUAwDO\/Dw2IGeUiQAAAABJRU5ErkJggg==","height":224,"width":371}}
%---
%[output:66f27b22]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAXMAAADgCAYAAADxP4bLAAAQAElEQVR4AeydC5xUxbH\/iwQjCChwQQERF4NBgmgQiWskoDf6hxslRK6oIAIxCfi6wSsIoiIoIvIyGPEBRiLKI0qiIuEGxURBiHiJQUEFBGHDW+EKukTwFf77bdJD7bFn9sHOzNnZ4kNtV1XX6cfvnKnpqe7T\/bUD9s8QMAQMAUOg0iPwNbF\/hoAhYAgYApUeAXPmlf4WWgcMgZgjYM3LCALmzDMCs1ViCBgChkB6ETBnnl58rXRDwBAwBDKCgDnzjMBslcQTAWuVIZA7CJgzz517aT0xBAyBKoyAOfMqfPOt64aAIZA7CJgzj+e9tFYZAoaAIVAmBMyZlwkuMzYEDAFDIJ4ImDOP532xVhkChoAhUCYEyuzMy1S6GRsChoAhYAhkBAFz5hmB2SoxBAwBQyC9CJgzTy++Vno5EVi1apWcfvrpctJJJyWoV69esnfv3lKViB32L774YqnssdN1wQ8ePNhdW9ay3EX25zAQsEvLg4A58\/KgZtdkBIFjjjlG5s6dKxs2bJCVK1e6OpctW+bSdPw54YQThPJ9fdu2bRMceu3atWXWrFly\/vnnp6NaK9MQqBAEzJlXCIxWSKYR0CNpHK6vH55Rdc+ePaWwsNCr5eGHH06M8LFJZCRhcODDhg2TtWvXSkFBgfhRvh+lUweEHh0E73Vdu3YVfl1QL\/pOnTq5Nuh2Y4v8wQcfCPYTJ050beQXCXquwYYykjTT1IZAAgFz5gkojIkbAh999JF069bNObjTTjtNOnbs6EbHOMk777zTjdoZsTOCxuHhAJcvX+5G13fffbds2rTJdQn94sWL3eje26NzmSn+HHfccS4XZ+sYEdm5c6eMHj3a\/VpgFM8XxsaNG2XGjBnOhPKvuuqqRN0ot27dKnPmzJGrr74a0V3L6H\/IkCGyYMECp\/v4449l+\/btLu+CCy6QUaNGuWuwoe18WThD+2MIJEHAnHkSYEydfQSiYRacGk77\/fffl6ZNm0rz5s2FETROfv369c4xtm\/fXo499liX17p1a9cJ8nC8fCFA8N6JOoMy\/KHOhg0bupF6fn6+c9r79u0T2kY7aA91NGvWLFHq8ccfL0cddZSTCdXwRcKIe9y4cU7n\/3Tp0sWxLVq0EN8PeKe0P4ZACQiYMy8BIMuOBwI4SZwljrk8Lerevbsb9TIihiZMmFBiMXxpYMSXAymEI8ZZM\/rmS0E7bfJTEaNrQi7Tpk1zvxIYdaeytzxDoCwImDMvC1oxts31puEIGf0yUiX8sWXLFiG8ofWMbAmzEBYh7+2333awcM3ChQtdDBt7HCojfJeZ5A92Y8aMkZYtW0qDBg2KWTEihwipEMqpWbOmCwHRPq7z+mIXKYEvAkTsSY0MgYpAwJx5RaBoZaQFgWjMnEp69+4tbdq0kdtvv93F0xklN2nSxMWjCWEQnsDR3nLLLeJHzeivueaar9hTnqbNmzcL1xIC8eVGR\/DoiYGTEgcnlMMInnZRFnpG3r5udJ78r4v+\/fvLOeecI+3atRPi\/Z988ok3sdQQKDcC5szLDZ1dmE4EcNhvvvlmsdAIywNxiNSLgyZcAmmHC49u3rx5AmGH\/dVFk4\/oIWzQacKOPE3ejjqpGxtCLosWLXLtmjJlSmLJorfhekbeTGjyC4J6uZZ86kPGhr4NGjTIXZ+XlyeUSfnexteNTl9PvpEhEEIg\/c48VKvpDIEcQ4CwDSN6iJH38OHD3URsjnXTuhNjBMyZx\/jmWNMqDwJ+xM2oG2JEXXlaby3NBQTMmefCXbQ+GAJVGwHrfREC5syLQLD\/hoAhYAhUdgTMmVf2O2jtNwQMAUOgCAFz5kUg2H9DIF0IWLmGQKYQMGeeKaStHkPAEDAE0oiAOfM0gmtFGwKGgCGQKQTMmWcK6bjVY+0xBAyBnELAnHlO3U7rjCFgCFRVBMyZV9U7b\/02BAyBnEIghs48p\/C1zhgChoAhkBEEzJlnBGarxBAwBAyB9CJgzjy9+FrphoAhEEMEcrFJ5sxz8a5anwwBQ6DKIRBLZ663E2VLUQhdlbs71mFDwBAwBEqJQKycOecr4rhpO9uIakJHHjbwRoZATiJgnTIEyolAbJw55zZy+goOnL2ho\/1BRx420TyTDQFDwBCo6gjExplzHBfHaJV0Q0pjU1IZlm8IGAKGQK4hEBtnDrCrVq2S008\/XQinECOH4NGRh41RthCweg0BQyDOCMTGme\/du1fGjBkjEydOlGXLlsns2bMdboRWrrnmGpeHjVNW4J8tW7a4+qjTaJlhUfTs2XNQdZ+DghVLZPWM0aX6HOA7KtAVHXZRsXHmn3zyievMaaed5g7Cbd++vbRo0cLpunfv7lJv44QK+MPNuOmmm6RXr15GhoE9A1X0GfjvfpfKgkH\/IdtHnif\/vPv7snvOyFI9C\/gOfEgFuKIKKSI2zvwwelPuS7kRr732mowfP15mzZqVlPjF0LdvX3n88ceL2UT1Wg7xd955p1COTjt37uzaX1IbaJ8uE9lTVK\/lEK\/r9+254oorBIr20dehU11mKr22C\/GhdhgefUXjYnikB48+XX8gD171A5n4re0ys81m6dO6tpx71c3yWb\/fyMIzbnOfdf\/M6vvhPy\/cF3wHPsR9gGPwp0o4c5YzEnuHiMNHcT\/++OMlPz8\/KV122WUyYsQI6dChQzGbqF7LIb53796uHJ0OHTrUOfh27doVKzvUHl2mzo\/qtRzidf30C3nUqFECRfuo6\/G8LtPrSKN6LYd46vX1+9TwGCEaF8Oj4vA485uNpPX+NfLvuxZIn22\/lm9ve1E6dL1cGo98SU6aulnqXTpCTrmwn\/uM8jnwz6y+H\/45xalH\/Ui25Vg5861btzqHhtN9+umnpX\/\/\/m4yFEdBXnnAYsnjfffdJ3PnznU0f\/58QVeestJxTaNGjaRPnz5Cmo7yK1uZ4GB4HLprhschLODKg8e+t1+WnQ\/8RDZd21x2P3UHxUjD634jzR7c6NKarc91upR\/KkFmbJw5SxMXLVokTHiGiDxsyorpypUr5eOPP5bjjjtOmjdvLnXq1BF0ZSln\/\/79snHjRiHV1yFrvZZD\/O7du105Oi0oKJBPP\/1UF5uU12Vqo6heyyFe10\/7kdesWSMQ9rrsEI8N15HqfGSt13KIp17sdWp4bJR3Nu2UpW+uc+mzy9bJXzZ8JEvW7y6iPQnSuHteY+x1pFG9lkO8vh\/+\/vBsQNhTZirChutItR2y1ms5xIfaUdrn44udBc5xb+h\/gouF49Dr9RjpHHidnz4ku07slPg8h+pGB9HeUDt27NihuxYLPjbOnNFyp06d3EickXmUyMOmPKgRRjnqqKMSl65fvz7Bf3LGVTLmlb1y3ezVSWnQMxtlwrL9QqrtkLVeyyH+tgU7XDk6vX\/FgaL6C2XgnHVJ6\/d16jK9jjSq13KI1\/XTfmTaAWFPmakIG64j1XbIWq\/lEE+92OuUNox5JXfw+MX\/7JEej66Wrg+scKmX0V08c4fkT1jhqN39BfLNUW8IaYdJb0nX6VuE9Nrndsl\/zd8tl05bIz96cEWC6t\/4knznrldl7PMbZcMHexOOqXHjxu7ZxhF5QqH1Wg7xNWvWFOx1mpeXJ3lFhL0vN1mKDdeTahtkrddyiNf1cx0yI3Ns9+3b5\/qsy9+\/bb0w8t5WNJHpRuEv\/lpqdrhC6t38Rzn23tVS80dD3TVcT3mkXE\/q5RBPveT79KOPPpIvv\/wS01hRbJw5o25G34zKhwwZIqxggYeQe\/bs6Va5VDR6\/zyqgby6ar28smKNvLttT8Zp9eZdsnHnP9wHMhv1x63OXMMDZwHV\/8YXEqXTj6suXVrUcNT\/u3Vl5PkNEjS1eyOBfnd5Q3mk89eFdF7fpuIJW6594tUtcuY9y92XxetrN8v27dtzmhgR79y5U95\/\/\/2D\/XxnuWyfN1m2jThXtg08WXYXOfD9Nf9Nqg2YJdWGLZb93x8ge+q3PGhbQdhMnz5dWMlS0b7ocMuLjTP3HWH0vXjxYsGBex2OHR15XleWdGtRLF4va\/RLHimj9pJx8lC3hvLctW1l3nWZp9\/\/\/FT3oZ3z01ZZqT8bfU5VZ67iMeXKNhKikT9uJZ76\/\/vJ4umiM5sL1Pbk46X1icfKGd9qKu1anpAg7Ljurze3F6hGjRpuNP\/4yv1uVM1IMheJcGnD6p9KrTUL5OtPD5MDYzrK1196QOo0a+UmMpv8ap00GThDGne4OG04MPk5cOBA3EesKHbO3IdDmAD1SI0bN86xPs8JpfzDuvWjjz7afZMT\/yosLBR0+nLCMPyE5AMRImwZ8ZDqfGSt13KI56ch9jplhIGtLjcZjx3Xk2obZK3XcojX9XMdckFR7B7CXpcd4rHhOlKdj6z1Wg7x1Iu9Tg2P7VJWPE46trYbCDzdv43MWbHLhW0++ETE3xuNPToth3hdv78\/PBsQ9pSRirDhOlJth6z1Wg7xX2kHYZSpP5dv3HuBHHjhPqle+L6bwGz24KGJzFT1+TxdFzoth\/hoO4455hg58cQTJW7\/YufMa9euLZMmTXJvgPq4+fLly52OvLICSPiGb9Fu3boJBI+urOWYfZkRsAsyjEB+XpFTLwrF9GjbwMXTZy\/fkeEWpKG63Vtk\/7yxbgS+e2gb+fLdpSIXDJRjb1soTUa+JHXO7ZeGSitnkbFz5sCIs12kVrbAoyOvPHT++ecnVsnAl7UMvr1ZCUOqr0XWei2H+Hr16rkVNTrlF8GRRx6pi03K6zK1UVSv5RCv66f9yKeccopA2OuyQzw2XEeq85G1Xsshnnqx16nh0VwOF49bLzzZhQ395LTGnvul5RCv6\/f3h2cDwp4yUhE2XEeq7ZC1Xsuar164Qxr8fZHsu6+7c+KfL3tS6p3\/MxdGOeGBjVKtyJlL\/aa66CCvy9QGUb2WQ3wIj0aNGukiY8HHxpl\/8MEHbl+WklCZOHFiSSaWbwhUeQQ6tKgrb9x2tmz6cL8bpZPGGRSWEha+\/JgkVqI8dYcc0TDPOXDCKLzQkyvrwdN1H2LjzBl5szsioZXQW5royMPmcMEo6\/WsRiDeTqqvRdZ6LYf40HrVgoICsXXmh9bfGx4bRT8nh4NHs\/o1XCydsMsPf7VcXl6zyz2+oWcTHcTzrOv3MmvMIWxcISn+YMN1pNoMWetZSrhxzgTZfl9vYSnhzlnDnQNnHfiXN\/1Zqve6V3YcdWKF4eHb8pV2qPdIdJ7nQ3iwqsaXF5c0Ns4cQAiBsBQRHsetCR152MBnkvRPL11vVK\/lEB\/6uWZhhcMPK4B1Rd8XQgpQtGxdj+ex0eGDZHptF+LT9XwQdnm4dxvpPnWVW5ceqhsdRD9C7QALCBvfv2QpNpRDqm2Q0RNGcevBB54sB566SQ7s3uomMnmlnjczG3a5+ivhSK6jXXH5vFiYRd\/ZFLw\/VQjn7Qldiktclh+98yXQqVOnxGv7Je3N4i62P4ZADiPgwy5L1u9xa9Iz3VXCKM6BKEnDxgAAEABJREFU\/+uFHkIq\/o3M3JzIzDTCIrEamR9O9zm8Yv78+W4fYr4A2EL3hhtuEH6mxnlvFrF\/hkCGEPBhF1LeHsWxp7NqHDhO28fB4Wt++9xicfDqRXHxdLahKpWdM868TZs2Mm\/evMRbol26dHH38e233xbbm2Wje405VQyQeCiEjQMuxR9sdOzTm0b1Wg7xoVgkX742h5DeOYSJFzeXq86o7bYGGD1\/nfh76e9R6L7wbEDY+PsdSnHgvIW5YWi+i4MzGvcTmbxSv6d9H6n2zXx3KWVF60YHoQ+1Iy7Ph8XM3S3MzJ8FCxZIx44dhWV\/vBSkXzhar\/ZmoTXsS8xDwkOUjHAwvDwQzY\/qtRzlkZNRqOxoXchcH7KN6rUc5ZGTUahs6o0S14dso3otR3nkZBQqO9oGZK4P2Ub1Wo7yyMkoVDb1RonrQ7ZRvZajPHIyCpUdbQMy14dsld59sf+\/k46Ql68\/xW3c1f\/pHe5FJX89tskoVDYTmYUvP5bYmXDX7OHyaa2GUrPP\/YIDZ0ITB+7L12VQj5ejPHIy8tdQZiri+pBtVK\/lKI+siTfKzZnjOUtBLFMk5k3s+9lnn5UBAwYIYZRSXOpMiJ3DlCbOjh1hGPZb4O20EDFC4Ethz549xfZ4iOq1HOJ5qCgnmvLLgTcfQ3VrnS4zlV7bhfho\/ci8JQtF+6jr8bwu0+tIo3oth3jqNTz2iMcm03jUqbZfHvxRAz4C8v373pIX39ruBj+hdvBsQMWej3eWy7bHb3Z7orDFbOGbL0q1S8dL9dv+IjWu\/JXsP\/XCnPy8\/P73vxf\/VroDLyZ\/Yhdm4ZzPG264QdhYiz1ZeOuzR48eXzkDFOfOMkUcPse+cR2YDh48mEQmTJjgUv7wTZpsbxbyOeWH\/RaysZeF22uiYUNhdjwb9cetTsOjcbE9RTKBB3vlDO2cJ4zQS9rbpcHXP5Uar0xxL\/OwL0r1N+YKE5lN7lsnTX61Thp3vb5Y+yv6+coEHiW1GV\/Bm+T4jjhR7Jy5d7o4cg+U30vF56EnRv7mm2+6Nzs5ugynjyPvUhQr1yNyrmVEwciXOJztzbLd\/aRmJM0IzKeEmSCwrVGjRmJfjxCPDdeR6nxkrddyiNf1cx0y9wlbXW4yHjuuI9U2yFqv5RBPvdjrFCwg7HXZIR4brifV+char+UQr+vnOuRM4dGjbUNZNritC7tcPGWVG1FTv2vHtvVuBL75tu\/LzkGtRL+R6V\/oqdGkReKZ0X2rrHjQB8j1f9++YnjY3iwgUwryse2ybrTF8kOu8acT+RE75fEtyr4sEDwvKJWiKQkTHkjWuZImlEUMstZrOcSzThZ7ncZl3SxriCHaXdS1lP+xoR+k2hBZ67Uc4jUOXIdseGRv3f23mtSVhYPOlnNPaSDXTn9Dtr30pBw5\/SphTxRG4HWbn1psJUqyNzL1vc7V54Nf0rpvceBjNzJnhO032vLOuTQbbfEyEUsSNfkRu86DjwPw1gZDIG4IsBKlsGgi88pXr5IH3u4uu+fcIXMKjkw4cF7oSebA49aXqtie2DlzbgIjZzbX8o4ZHh15WSGr1BDIUQS8A\/drwf1SQhx3swc2yovtx8jZzxzpwi85CkHOdCs2zlyvYCFEEiVWt2CTDeRZ+kS8nVTXj6z1Wg7xrFrAXqfEZVn2pMtNxusytU1Ur+UQr+v37WENMYS9LjvEY8N1pDofWeu1HOJD7TA8NrrVLeAIPmnBoygGvmHqDbKpKAaebE8UzsisU22f\/OqHdeXiNnXdmvSbZi53yxr1PQ\/x+l7r\/KheyyGe\/nscfJoWPGxvFn2bDp9n5M0InNE4pwwxAQoPIbO6BZvDr8lKMASqGAIfbhH56+9l95SfuV0JOV4NuWbjFlLv5j8Kx6uxFrxOkr3Bf9GxkVvC+Ojre905plUMvUx197Dric3I3PeE0TdHxOHAvQ7Hjo48r0uVsmyRF4ZIsWNy1I\/0\/Rp09KWlqjChw+QnRF9LwgUbJixJtS2y1ms5xDPhib1ObQL08CdA2chq39svCyETwic4bza0+nztUvGv0\/tNreq16+I2teL+QNH74eXLz23jttTlfpe0FYAvhxR7T8iUR4qO1MshXj8X2CHH5fmwCVDuYAnE6hNMmPwkhfwCfZ+HLhmx3nzMmDHy0UcfORO+AHgpaO7cuQKxfws6l2l\/DIEcQMDHvXlxB+dN6GT7yPOEycwjGua5HQlZQggdzr7g7OnCmvSe7Ru5sMvY5zfmAHq504XYjcz1ahY\/mi7NahZ\/S2bMmCEnn3yyNGvWzKlWrlxpe7Ns3OhinT4uGYpFEi+HsHHApfiDDTFMUm2GrPVaDvGhdsQlJgoWEO3WfQzx2Oh+e5uoXsshPikeO96Tzz8oEEbbhS8\/5kbc3nFv6FHN7YGCzBuYnExfb8AjxVagHJF\/uWzee8A9A7QtVDc6iH5E24EMFhA2Qzs3d6N0jqXr+sAKiR58gQ3lkFKfJ2St13KIp17sdRqX58Ne5\/d3tYSU2LiPnxMzh0dXwmXulf8VK1ZIv379ipmm2pvl9Dr7Zf87i9wHhQ9LiA68t0waffJ3IdX5yFqv5RBfY9ubrhydHv\/pFjlyy4qU9fs6dZleRxrVaznE6\/ppP\/KJX+4QCHvKTEXYcB2ptkPWei2HeOrFXqe5igdO+PNlv3XHoZFCDTga7bmxsq+Iav\/PnfLFrBuFlOPSWN9deOsZ8o3hreQb914gvKzDaBunzUZWhZtWyxd1jhNi3cS9mzyxz72B2WTgDKnZobfbzAoHCfFhaNy4MUnCoXsZZZSvWbOme4tTp3l5eZJXRNhT5rFHiXvJ6MymNdwonYMv0EPY+DKRPUX1Wg7xun7KQ\/bhDV5o8uUmS3WZ2iaq13KIp15fPym\/+r\/88ktMY0WxG5kTAmHlih+V+xQdecnQI7wyefJkuf7666VWrVrJzL6in\/it7dLsjzcLH5Rs0M67LpADU3rJ7nv+I2ttyEa\/k9WZq3jghEOEY4YYVeOgIUbX0BcntpPPut0luy8YJl\/+7Ak3UVlt3IaD6dWz5cvuY2Rvy86yp35L94YibytmmvqcVkNuO7euO\/hiwBOr0t4ORsQ7d+4U3ozNdF99fdOnT5ebbrrpK74k24rYOXNG4IvUYc6MzIcMGSLDhw9PbG8LaExu6r1Z3nnnHbeXOW955ufnC1vf9u7dWzZt2iRbt24VvRVAixYtKMLRoHcbyz+LPiiNR77kfpqGUkY+1QbMkoa3LSxmE9VrOcTXGTxPKEenn\/X7jfugRsu2dhTHOogHKzHSfF\/qj1slbt8R9h5JQvXGrpIvb3pJorZRfb2pH8pno1ZL\/Ud2S4IvKr\/e2JXy2Y0L5aj\/flY++8ljclTfyc6Jk37a5iKpftZl0vCsi6Txt9u7EXPjolF2iOrWreuOIGSiUOdH9VoO8YxEWS6rUzaDg6JlU89FZzaXv97cXj78rLp0nb5F3v4\/SVs7aBfbczBCp+5UpPum7aJ6LYd4jQP142PYn8U5kBj9iZ0zD2HTvXt3mTZtWuLkIGyie7N897vfFb9Xy7Jly6R169ZC\/Pyiiy4Sbj7f5MTfCgsLhf1aKAN6s7CG7DqmhfBmWyqSb+bLES07fMUuqtdylK\/+rXMEXTT98sQzg2WH2sP11o5zE\/ch3XjUaNxC3L4j7D2ShGoW6as3PFGitlE9ToFdImvUqCEhnrxkxDUlkS5T20b1Wg7xydqAXper+ZOOrS1Mjl55dlPpM6tAHntjX2KvFm+n60Kn5RBPfcmI60siXaa2jeq1HOKjbTjxxBOFgST+I05UKZw5k5iMrssDHCN99mPh2xSCR1eWsngQWBpFqq9D1noth3hGNdjrlDgkD4suNxmvy9Q2Ub2WQ7yu37eHZYkQ9rrsEI8N15HqfGSt13KID7XD8GguGpfKhgeTo89d21YWrN\/\/lTXp+hngudFyiNc48FwhxwUPfhnQhzhR7Jw5cXHi4z5WTsrmWWV5aQhnzalDjN4Bm\/1YCNdA8OjSS1a6IVB1EejQoq7g0EGgpDXp2BhVDAKxc+Y44mjMHCest7WtmK6XvhRmwgnRkOqrkLVeyyFeL7HiOuSCggIXX9TlJuN1mdomqtdyiKdeX79PWXYGYa\/LDvHYcB2pzkfWei2H+FA7DI8MvM5fjtfXeTYg7qO+5yEemy8\/2i5zftpKGKn\/6MEVwpp09LnyfDARG+p7NnWxc+aMzDlsgtQDAx\/V+TxLDQFDIL4I8ILRG7ed7Tbqyp+wQl7fuj++ja1kLYs2NzbOnKWFOGxWojCBSUqIBYKn4aV5AxS7iiYdz9NlR\/VaDvHE\/Hzsz6dxiQESL4dot+5jiMeG9pPqfGSt13KINzyaJyYJwS1X8dBvjnKa0X2LtrvHJvRMoIPijofFzN0tDP\/hzU\/2H\/eOnJTwiifysAlfbVpDwBCIOwKEXIilj32+QLo+sCLuza107YvNyFyHUli5wmicUbknJkWxSYWwH91zDUuHWIuO\/eFutBWN9VEmFNVrOcRbjHijewPRY2N45CgeKibP58QT9\/34r+92b46iY3L0d0tWF3smsIGIrcf5+bCYOXcwCTHxyej71FNPldAEKDpsklzu1CNHjhR2S2Q0P3HiROFNLb4AbKMtB4\/9MQRigYAPuwz6QVMh7DLxT5tj0a7K3ojYjMw1kGxTy+haU0kjc5z2tm3bhBeMKIsliBMmTBDWqPPm2nHHHee2+qxTp47TYePptddeE1ZRMCIIEXaNGx\/a28LbRPVaDvG8kEA5OvWxt2zuNUF7iN1DtNv3L1mKDf0g1TbIWq\/lEE+92OvU8GjsXijyuOQyHvoQaSZHD9RqyGPiyPc\/msYBD9ubxd2ikv\/glGfPni1Tp04VRtieSjMyp\/QbbrhB+BLQYZZUG21xDSN3RvF+74VMpvxcy\/ZeE5nsb0l1pReP7WnfO6Sk\/pU1P9fxOOLT3e7giy4tasiZ9yyXkc+uTnmP4oAHvsL2ZsFzloJwvvqV+1Jc4jbeYT+Wq666yn0JXHPNNTJmzBhhL4WSrh8\/fryw1wKjgBDp\/Rp0flSv5RDPCJT2RFO2G2DEocsO8bpMnR\/VaznER+tH5tcLxIoKXXaI12Xq\/KheyyGeeg2PeuKxqcp4DO2c55z61P\/d40IvoefC45PtzwtvkuMvSvIrmc6PXZiFuDhxb304RQgUJjcZfTMKZ0kj4RP2MCecgj2baTGRykNBmmyjLWz58iDEwJKoEPEQ8cp9NC+q13KIp4xkFC07JOsydX5Ur+UQn6wN6HW5yXhdpraJ6rUc4qkvGelyk\/G6TG0T1Ws5xCdrA3pdbjJel6ltonoth3jqS0a63GS8LlPbRPVaDvHJ2oBel5uM12Vqm6hey\/BnnVjbTY5yzSW\/3Slv7PhcqDNE2JRElMm1UbuoXsshnjI02d4seM1SkA+zcLoQjtpTNGbOq\/p+Yy0mTnHGLVu2lKVLl7pa1q9fLzjpM7i9jwcAABAASURBVM44I+VGW864hD88DKx7JdWmyFqv5RDPqBd7ndLuI488UheblNdlaqOoXsshXtfv28Macwh7XXaIx4brSHU+stZrOcSH2mF4VO69WXge9L1G9hTVa9nz32pSN7FhV6\/H18tT70os96rhl7TvV1zSWI7MiY\/7WLlP0TFqTwUcW+USb+cLgHTSpEmCc2BzLX4aQfAllZOqDsszBAyB9CMwVJ1mxBLGTR\/uT3+llbyG2DlzRuaMwnHIUSKsQnglGeY4aZw+XwCkyNiysgUdBI+uLMSKDda9kurrkLVeyyE+zutm2XcDot26jyEeG91vbxPVaznEGx5Vb525fm5CzwQ6CLs61fbJM1c0kovb1BUc+tQ\/r5OCmOxlxESsf+7jkpbozDPdUBwwOySyxBDnCzHihlg7zqRmpttk9RkChkD2EPhFx0YytXsjufkPW+Ta53ZmryExrzl2zpyR+eLFiwXn7bHDsaPLVpzKx\/NIfZtIkS1GDBIHyfCocRCIf\/01PCoOj0s6tHKHSNeqVVuYHF1WsPdfKCdPovh7y6heyyE+NLeTLV\/k+xBKY+fM\/WZaTID6BvuVLZzt2aRJE6+21BAwBHICgdJ1gjdH2Vb3olNqS\/epq9y2uqW7smpYxc6Zs5kWE5fLly93L\/8QN2cyc\/To0XLrrbcKSw6T3Rri6cTVuYa4O6N8bG1vlpJjs8TLIeKVYJaKsCGmSartkLVeyyHeYuYl35e4xIh5NiDuo77nIR4b\/Rx4m6heyyE+2fPR7zs13RLG2ct3uA27kk2O6jJ9G0ijei2H+FA7LGYOkqUg4uZMYBIvh+AJZ7AEMdkhFWyyRTydl4W4pn379sLoHofOG55z584VaP78+cXOEi1Fc8zEEDAEYoYAo3T2SedUIw6\/WLJ+T8xamPnmxG5kDgSDBw9OjMoZZUN6pI1NlBjRR0MwjOJtbxZxJ7p7vHhjkxcjomleXp5bxokdo5NUhA3Xk2o7ZK3XcogPtcPHImOyVw3NTuzqp\/uqeYx0v31eVK\/lEG94NAYWR+BZGjwGdmosE36cJzj062avLnavKIhySP09IUXWei2H+Gg7bG8WUCoFMZJeu3atG0UzCcoeLaSscGHEnqoINtbiZSGcf5cuXcSP4nl5yMfiuR4bUk+M3Nlvoaz7ZlSEPT\/XbG+WQ3umGB6HsOD5MjxKxuObtfbLvL5N5d1te9z+Ln\/468aU+7uA6+EQvsL2ZvHes4SUV\/N5LZ+R9YIFC9xOiKxmwdEnu5Q8Ru84ccIsXMcIP5m91tveLDWFfVkgZu4ZtaQiv5dI1Daq13KIZ8TDdgvRNNt7b9AesICifQzhovum86N6LYd46jU86onHpix4tGt5QuLNUbbVfXzl\/kQ50Xvoy\/d6LYf4aDt4+dD2ZtEeNAnPCNqHS3DoCxculPz8fGF\/FX2JnuxkbxYminACfoMunDqTqHwguTbX9mZhCZUnHjb2jgjJOs\/z2CYjX0aq1JcTtYnqtRzik7UBfbTskKzL1PlRvZZDPPUlI11uMl6XqW2iei2H+GRtQK\/LTcbrMrVNVK\/lEE99yUiXm4zXZWqbqF7LIT5ZG9DrcjXv3xx9ZtUeYTuAkK2ui2u1HOIpQ5PtzaI9cQqe2Pcll1zidkFk\/xVeFGKkPnnyZNFhFvKie7PguImRUzyhFMIrtjdLDbePOw8txEQyI5Joyr4sEDbgl4qw4XpSbYes9VoO8aF25OXluc2VdLnJeF2mtonqtRziQ+0ACwh7XXaIx0b329tE9VoO8aF2GB5l36vGT46edGxtuXjmDvnrluJbAWjsuVdaDvGh++Lndrg+LhTLCVBG4jhrQOL1e5y2l9GFCEePwx80aJCbPGU5I0sc+TCwHws\/jSB4bENlZENHTPTxxx8X0mzUH7c6wcHwOHRXDI9DWMCVBY8HerYSRup+cpTrc5kq3plnES0cPo6fmDnLGb3T5gsBHQQfbSInDS1btkyS0ZIlS+SOO+6QJ598sphNVK\/lED9jxgxXjk7Hjh3rjrdj2WSy+r1el+l1pFG9lkO8rp9+IQ8fPlygaB8pP0q6TJ0X1Ws5xFOvr9+nhscdonExPA4Pj3f\/Z4oMO\/nv8uqq9W5\/lycXvSX6WeT51XKI1\/fDP6dMgkb9SLbl2DhzP4HJSpQQMbmJTUUC1rRpUznrrLOE1SzE3ZNRnz59nMMdNmyYaJuoXssh\/vbbb3fl6PT55593XSqpDdSry0T2FNVrOcTr+nkokWfOnClQtI++Dp3qMlPptV2Ip15fv08Nj+micTE8Dh+Ph8bfKXt+O1A++MuTcs3cnaKfRZ5fLYd4fT\/0c4rvwIe4D3AM\/sTGmUexYEkiI2lPeqQdtS2vzI1gJQsvIxnNEsPAMKgkz0C5ntWnb79cHurWsFzXhnDBd+BDyut\/Kvq62DhzQiI4bJw3P31GjRrlYt+M0tMxKvdAcjOI0Rvlu1VDhoPhkMvPwGWdTq2w5xzf4f1IHNLYOHMNhnbsOHdiuT169LDX8DVIxhsChoAhoBCIpTPXa8gZmU+bNk3YUwUnr9purCEQOwSsQYZAthCIjTNncpNwCs77+uuvF14WYlQOEa9i\/Xm2QLJ6DQFDwBCIOwKxceYaqM2bN7u4Fo7dE44eh6\/tjDcEDAFDwBA4iEBsnDkhFD8Bymg8SuRhc7DZ9rfMCNgFhoAhkNMIxMaZ5zTK1jlDwBAwBNKMgDnzNANsxRsChoAhkAkEYuDMM9FNq8MQMAQMgdxGwJx5bt9f650hYAhUEQRyypnrg5vZc4FzQbmPWv\/www+jMjIEDIEqhEBV6GrOOHOWLbIFgN\/TpWPHjjJy5Ej31iibWLErIcTLR9hWhZtrfTQEDIGqg0DOOPP3339f9ElDHDm3bds28acNcWoRBwhw0IU\/wKLq3GbrqSFgCOQ6AjnjzHHWH3\/8sXhHzUlDHBfHmYqcOMRxdP5mkuf5LVu2FNujnE2+jJLv7W7YVDA2KfbRz1WsC1YsEU373n5ZNBW+\/Jikot1P3SHJaOcDP5FMkvcjcUhzxpnzQhEbcvXv39\/ttrhixQo3Uk8FMo78pptuKrZHObF2o16GSS\/DoKTPwX\/3u1SgsQO6y2P\/9SNHK265QDxtH3meQP8c832Bjp14tkD\/vLtIVoSNppKc8e45I2X1zLuCtO6FGaJpybzfSjoJH5LKx2Qy72uZrCzddXGKkH9zlF0WCalwEOvWrVvlk08+SVRPCAaBG8EpQ+xLzP4vyWjMmDHSt29f4TgzbRPVaznE33nnna4cnXbu3JmmSEltoF5dJrKnqF7LIV7XT7+Qr7jiCoGiffR16FSXmUqv7UI89fr6fWp49BWNS7bx6NP1B3J7787Sp3UtuSlvp0z81nZHL7bbKDPbbHaEbkhR3i3ta8mFpzSQC7\/3HencvrV06Hq5o1ZX3CbQPy4cKe93GiT1BjwijUe+JJ\/1+428mT9UGt62UDb1ekJeOONW+eeQlxL8X7o9IU+cOlJ0Or7B1XL+681le5\/fyteGvZKSNveaISvPvtmVT32ePvvJY8X0Wg7x\/+j9iLPX6QvN+8mgdxsLPkRi8i+rzpyVJX7vFZ+iKw82TGp27dpV2HGRVSzstMgkaPv27d0InZj6xo0bpbCwUE477bRiVRCGSbWH82WXXSYjRoyQDh06uD1jvG1Ur+UQ37t3b1eOTocOHeocfLt27YqV7evQqS4zlV7bhXhdP\/1CZvIYOtTH\/KTt0WVWdDsMjxHC\/fD3JZN4\/Oe\/nyU3djpROn\/0J\/n5gZfle3OvlD7bfi0dVz8sV52w1zlq76AbXvcbgXCQzR7cKCfNOSCkju56RU4au8zlY1Pv0hECtek3Qs6+foLUO\/9nUrP1uXLKhf3k4kH3SJ3Tz5dzL+4tVw+7S1q0PzfBaxyygYd\/zkPtYPBRzInEQMiKM\/dLBem\/H0n7FB2OHRv40hJhloEDB0q3bt2cs27SpIlcffXVovXkYYOutOWm265Ro0buGCvSdNdVGcoHB47uIq0M7U13G8EhXXh8sbPAxaYJa2wrColsura5izfvK4phH9Ewzzlj76ydky5y2jhnqM65\/QTCKVcvsk03Dr78dOLh66isacadOSNof+gyzjYKHDocOzbRvJLk888\/X7gWmjBhQsJc6+ETGaVk9u\/fL4zqSfUlyFqv5RC\/e\/duV45OCwoKhElaXW4yXpepbaJ6LYd4XT\/tR16zZo1A2OuyQzw2XEeq85G1Xsshnnqx16nhsVHSiQcOnMnDDUPzxTnvWcOlcNNqwXnXu\/mPUm3cBqkz+m+y94e3yxend5MdR50ohdWPcc+GPR+HPr87duyQuP3LuDNnVDxo0KAScUhhU+K1FW1Qo0YNYVkjqS4bWeu1HOLr1avnytFpXl6eENfX5SbjdZnaJqrXcojX9dN+5FNOOUUg7HXZIR4briPV+char+UQT73Y69TwaC4VjUf1wh1Sd\/nj8uE9\/+EcOCtF6jRr5UbeJ03dnAiJ1GvX5SvPp78\/PBsQ91Hf8xCPDdeR6nxkrddyiNc4cB1yXJ6PRkW\/qHXf4sBn3Jn7TjNCZ49yQipROv30013s29taaggYAmVDgBE4TtuHT+AZfTcumngkZOJDJWUr1azjjEDWnDkj9J49e0r37t0ToZEhQ4YINHHiRGH1Q5yBs7YZAnFEACdOGIUQCmnUgRPjjmO7Y9emStigrDlzRuaLFy92ztvjhmNHF7efMDre69tKGtVrOcTrWKiPFVuMOL0x4uh9Ku19IT4MYU8ZqQgb7ieptkPWei2H+MN5PnDiTGTixBmF1+sx0q0uYQRe7Zv5bq6GOmkfqW9XiA+1Aywg7CkjFWHjy9d2Ub2WQ3yoHXH5vFjMXN1Z\/0bmuHHjEtqnn37a8bVq1RJWozjB\/hgChkAQARw4jtuHUvZtXy\/VLh0vx967WlgKGLzIlDmLQNZG5hzQPGnSJLd3io+Zz549W0aPHi233nqr+Bd74oC8npzR7YnqtRzimcDxEzk+jcuEDpNbEO3WfQzx2NB+Up2PrPVaDvGGR3MBFwjcSoeHCE6cEMq2Eee519oToZS7XpHmPQa7Miv6vvBsQLRVlx3isaE\/pDofWeu1HOJLi4euQ\/O6zFR6bRfiQ+2IW\/SA\/mXNmVO5pnvvvVd4WHjhh7cKWaKo8403BKo6AjhxHUqpc26\/RCjFYuFV\/ekQyZozx2nfcMMN4idBGan36NHDTXySF6dbo+N5ul1RvZZDfJxjgMRDIdqt+xjiscn1mChYQPQ1hIHWYZMuPDavXCafLp0lH9x1gVtWyAs9xMFZkRINpaSzHWABUYfue4jHJl14FBTE470Mi5mrO+\/3SmHS06v9a\/Y+z+uzneqfXrotUb2WQ3zo51oOhVncz\/tc+RnNr0SI+6jveYjHRvfb20T1Wg7x\/vmo88VH0uDvi+SLWTfK18efJ99Y\/LDUbNzC7WeCE2dE7uvQqS4zlV7bhXjfDp2Gf3XbAAAQAElEQVSCBYS9LjvEY1OReOh2xOXz0sjWmR+69X4C1E96kuMnQ30eOiNDoCogQAiFyUwfRiEm\/vnOAqnz04ek2rDFLrVQSlV4Esrfx6yFWQirMAHKpCcOna1rOUgCHXnl75JdaQjEHwGcNyETnLZfjQKPA\/dhlCYjX3L7n8S\/N9bCOCCQNWdO53lxaNGiRYmXhuDRkVeRxKZdfsVMeXZlrAoxQOKhEH0tCXtscj0mChYQfT1cPAo3rRGc9+4Xfy0bxl4im277vot\/s4c3o\/Fq9Y4XlhSyJ8qnfae5PVHAlzmWuMSIwQKqCDx8GaT0kxSK8vQfnU7jgofFzIs+FbwslOw1fhwuedgUmVbIf8qyM0ArBEorJBUCu7fI5x8UuBNzcNCESwofvUYOPNxTdg5q5Zz37ik\/F3nvNRf\/duGTAbMOrgkf8GuRM\/8zVemWZwiUiEDGR+aMvBmBs7Mhr+4zAQoPIbO6BZsSW15KA46R4zg5jpVjUoYDK9CV8nJnVhUmdJjcguir63SKP9iAJak2Q9Z6LYd4PbHFdchxmOCqUzQB2eLfagh04L1lCQeNkyYUgqOGCI9AH9zYSg6M6eicNqNt8gihEDLxG1qxJwp7frOxFWGUhl2ulubn93STxh4b+u9x8Gkc8KBdPBsQbdX3PMRjQ\/tJdT6y1ms5xFMv9jqNCx5fnQDVPc0On3Fn7rvJiJlX93HgXodjR0ee11VEyuETelJ1\/fr1iWL\/37\/tlb0vT3f7OvNhzTTtWzJD5K+\/z1r9me5vsvpwklDh03fJgYX3CSlyaQjnmYpwuCHi1XdNG3pUE0jrcM6QL59QSeGbL7ptY7+oc5xAR+Rf5iYo2UK24cTV0uSJfW7EXf\/mPzr9EfmXC6\/UE0ooD7FF8r59+6Q81+biNdnGg1DP1q1bEz4kLkzWnLl3rkx+ejCysZqF465Oev1htym\/\/8BmMt1d9NP7wFM3CT\/JM1lv3OriXEeo8JnRIkXOfO28R+XtZx4uFa1dskCgl19dISFaun63zCk48iv0zCct5cFPOyVoZP2h4ml43i\/l\/m\/\/Uqa2fVCm\/vtz8oefrJS\/DXxH\/tbnBdl+3Z+l2tWz5cvuYxx99N1+sqZWa3m\/Vp7s+vJI2b59uyM+9Ezqk6Ij9XKIX7t2rXsjWqevv\/664Djef\/99VyblJCNdpraJ6rUc4nX9tBd56dKlAmGvyw7x2HAdqc5H1noth3jqxV6nccBjypQpwtnB3m\/FJc2aM2fFyqRJk4TVLMTKIW4cOvIqEiA+DHrtut4qgPMEV1z+O\/na+I1uRMWoKpNUb+qH8tmo1VL\/kd1ZqT8dfZUHCqW8tH\/iB7Jl2DtSf+wqaXLfOpEibEpLjUe+LFH6YuACgY697jHxtL\/nQ+KJF2+gwvOHSp1z+8ru0y5z9HFeJ3npiLNkwT+\/I1PX1ZaRL+6S\/k\/vcNR1+hZpd3+BI\/j\/mr9bHltfW55ce0AeX7lf3vtHDWncuLEQEuDYQtKojM7neb5ly5aCTqdnnHGG8MuSn\/WUkYp8OaTKrkLacc455wgULVvX43ls6Aep15Eia72WQ7zGgeuQ44DHgAEDhBPLKtJHVURZWXPmNJ7YuI+fb9iwQeDRkVdRxItIRx99tDCyYWa8sLDQHSuny+fDwsNE3C7TVLNmTXc4RabrTWd9Jx1bW8pPdeSEejUS13doUVcOh3q2byRRGtq5uUTpgZ6tBJp3XVvx9MZtZwv04b3nCQQPPXdtW2c7tHOeXHl2U\/lWk7rywSci9y3a4ejSaWukyS2vSv6EFdLj0dVFuu3yzKo9ifh4WbDPxeejLP2P2sYBD3zFWWedpV1ILPiMO3Pi4exXXlLvS2NTUhnk8+XAtyjnf0Lw6MgzMgTKgkCz+jUE4suFL4ih\/\/pS8F8COHzIO3xssB\/7fIFcN3u11L\/xJfnOXa\/K2Oc3ypL1e8pStdkaAiUikHFnjiPlJCHCKqE13+jIw6bE1pfSgHM\/GflD8KW8LGHGJBKjetKEsohB1noth3i9XpbrkAsKCoQJnaLiSvyvy9TGUb2WQzz1+vp9yhpiCHtddojHhutIdT6y1ms5xIfakQt44MBx+AM7NZbB+TVk290HR\/hPXXWKdGlRwznyHz24Qk4d+YqMnr9OXl6zy+03nqt4+GdEPwPotBzi44yHrTPnDhYRDhXHWsQKjlsTOvKwgTcyBHIBgYMOvp4MOKuuzPlpK1k2uK1cdEptmbNil3SfukqIvROKyYW+Wh+yg0DGR+a6m2xzi+PWhE7bxIEnbsd6V1LdHmSt13KIr1ev3lcOzCX+Zgc6H8KlquBBnP2ey9u6mDxhGWLvg58tkItn7pC3\/69a4jmpKnhUts8LE9LaF8SBz6ozjwMAwTaY0hDIIAKM2om\/49SJsxOCIa6ewSZYVTmAgDnzUtxEHc\/T5lG9lkN8nGOAxMsh2q37GOKx0bFxbxPVaznEGx4bEy8CgWedavvk0m+JvHz9KfLiWzvcZOlfVq63OZXduxNzCnGZU7GYuf\/UV7JU\/wTUTY\/qtRziLczSPLE8j\/CU4RHG47STGsvCQWe7JZU3vlDolq7q5y4Zr585bRPVaznEh+4Lr\/JD2OuyQzw23F9SnY+s9VoO8aF2xCXsVAFhFg1NhfBZHZmzTLFTp05uEvTZZ58VFuOvWrWqQjpmhRgClR0BQi+TLjnZTY4uK9hb2btj7U8zAllz5hwNd4MdGydx+dlIiAUiHFLSM4cNYQFSbYus9VoO8RZmKR5mCeHRSHbJsO\/XkRt+t84tadR4R3mNsc6L6rUc4kPt4NmAsNdlh3hs9HPgbaJ6LYf4UDvi8nmxMIu\/q0Wpf72ezbWKRPeftzVhfB68kSFQ1RFod3wN+UXHRnL9b1dnBAqrpHIikLWReVw22irNbdPxPG0f1Ws5xMc5Bkg8FKLduo8hHhsd+\/Q2Ub2WQ7zhEY6Za1x8jLjXdxu7bQ66PrDCw\/2VVGOsM6N6LYd4XT\/3GZlnA8Jelx3iseE6Up2PrPVaDvHUi71OPR663GS8LlPbRPVaDvG6ft8ei5krRNlMi0212GiLnRPt2DgFjrGGQAAB9oxBbcsWQcEoikDWRuY0BCfObmjLli2TE044QTZv3iwrV64kK1ak43m6YVG9lkN8nGOAxEMh2q37GOKxyUpMNNKYdLYDLCDqiFT7FRGbTOHBpCh7vYT2dklnO8ACoo6vABBRYJMpPCJVFxPT2Q6LmSuomQBdsWKF9O3bV7xTnzt3rsyZM0fIU6bGGgKGwL8QYM8XXiyy0fm\/ALEkgUDWRuZMcrIdLce5rV+\/XthjHB4deYkWloHRBzf36tUr8aWg9Q8\/\/HAZSjxoquNoBzUH\/0b1Wg7xodhbXGKAxEMh2n2wd8n\/YkPskFRbIWu9lkO84VH6mLnGmV0aN+\/e73Zf1HqNcSq9tgvxofvCswFhr8sO8djo58DbRPVaDvGhdsTl82Ixc39Xi1ImQP15nJwkwsb37DmOjrwikzL9Z836qFGjZOrUqcJeLx07dpSRI0cK+vQf6FymppqxIXDYCEy+vFWRMy+QTR\/uP+yyrIDcQCBrI3MmQHv06CFMfHKCCKPyW265Ra6\/\/nohr6zw8kVw9NFHJw6eYKS\/bds24fSijz\/+WCif0QJfFtG4\/GuvvebWexNjC9GePXuEeCExb50f1Ws5xBNnoxyd8kVG+0pzxqMus6Lb8eabbwoU7aOux\/PpbIfhsUZK83yc2bSGnHVibbeFbibuC88GZM\/HGnnrrbfkjTfeKKuLSrt91pw5PWObW0bREyZMEPY5nzdvnrRp04asMhPOGqfoHTWhG46LY69wThLSo33ydAWM3KdPn570jEU+XDt37nSnFekzDaN6LSfjfTk+3bVrl0B8GemyQ7wuU+dH9VpOxvv6fUobIGvHwfsMFlCc8bjt3LpuC90\/\/HWje3b1vbbnY7v7UvTPd0XiMXPmTPHnFWs\/km6+pPKz6swHDx7sXuXX+5nzej+hkZIaHs3ny2D48OFupE95TK4yUo\/aheTx48e7iVjOKQwRcTpW3ZDqfGSt13KI5xcI9jqNw5mGtIcwF0S7dR9DPDb0g1TnI2u9lkM89WKvU8OjvZQWj3YtT5AebRvIo6\/vFe6DxhjZU1Sv5RCv6\/f3h2cDwt6XmyzFhutItQ2y1ms5xIfaEYfng21HOLEs5EuyqcuaM8dh85OaFSxDhgxxsW7Snj17ulF6SaCwhwunEeG4\/WSnH+kz2ieEQ0iFvcIZoetJVUIwunxG7jxMTMJkmuJwpmGm+5yqPsOjRmIzMnAqCY8pV7aR1\/6+t9xnjFJHZaKS8MhEX\/AVdgao9qBFPM6W8AjOdcGCBcKr\/YsXL3aTlkXZKf8TjiGGh+OeNWuW4Ky7du0qOHmWNk6bNk2YBGUkwAidn8usfWW1jN82IGUFKpOYJNeSKrXbvlTryfdyiCfeSL5O47LXBLF8iHbrPoZ4bOgHqc5H1noth3iNA9chGx4bBRzKggerWzhj9N1te9xWsWBd0feFZwOKlu3qifzBhvaT6ixkrddyiNc4cB1yXJ4PQlq6b3HgszYyJ4bdpEkThwEOfeHChZKfny+Mop2yjH8Is\/DTp1u3bm4SlLI5tUjrycMGXRmLN3NDILYI9GzfSM75Zl0ZOGddbNtoDUs\/Allz5qxYueSSS9ykIqPsiRMnCiP1yZMnlyrMEoJGh1mYVPU2Wg\/v9aVN+enGShhSfQ2y1ms5xMd53SxriCHarfsY4rHR\/fY2Ub2WQ7zhUb515h5vnTI6J9yy9ct6Lkyj8zT26LUc4kP3hWcDwp4yUhE2uf582DrzyBPASBxHjhonS9jEy+iMDIHDQ6DqXH3w6Lk821mx6tzyr\/Q0ayNzWsLbmExgairvahbKSxfpeJ6uI6rXcogn5udjfz6NSwyQeChEu3UfQzw2tJ9U5yNrvZZDvOGx0c27eGwOF4+BnRpLw5oi0Z0Vffmk3C9Sf59CfKgdPBsQ9pSRirDx5Wu7qF7LIT7Ujrh8Xixmru4sq1nYMdG\/sclEJrRo0aJyh1lU8cYaAlUSgandG8kXX3whtndL1bv9WR2ZsySwrCtLsnGL0hADFGKKxCZZ5sTyydL0K53tIB4KUUdJbcGG9pNqW2St13KIp\/\/Y69TwaC6HiwfLFfXOihp77peWQ7yu398fng0Ie8pIRdhwHam2Q9Z6LYf4UDvi8nxYzFzdWVaUsHSQHROV2lhDwBA4TASIn79x29nyowdXlHjU3GFWZZfHCIGMj8wJrxAXJ07OK7EQvCfysIkRRi6umesxQOKhELHLkrDHxvA4hFIc8fAOnXDL6Pnriq0\/1+0N8aFYNc8GhP2hnoc5bHL9+agUMfPw7ak4LSNy4uLEx0NEHjYVV6OVZAhUTQRw6POua+v2b+n\/9I6qCUIV6nXGR+Zgy1ua0Vfx0ceVdDxPtzGq13KIj3MMkHgoRLt1H0M8Njr26W2iei2HLpHZrgAADZlJREFUeMOj4taZcw80xsienru2rVt7nj9hhZsY1XYhPnRfeDYg7H25yVJscv35sJh50d3nVfsxY8YILwkxMudNzRkzZhTl2H9DwBBIBwJ+hM6bokyMfueuV51TT0ddpSvTrNKBQMZH5uyhQkf8KhaOjWN7SnRxpaoQAyQeCtHXku4DNrkeEwULiL7mCh6sQ2dilJ0WvVMnnr70zXVuXoi+cl8tZl7y+n+LmSf5VAAMI\/Yk2aY2BAyBCkKAUfqgH5wg8\/o2ddvnzlmxS7pO3yL5RSGYiX\/aLK9v3V9BNVkxmUYg4yPziuwgb5CyJ7ovU8fi9aoYOwO05Ngs8VCIeKfHM1mKTY7GRBPr\/8ECoq\/JcPB6bCobHuecfrLceuHJwkgdIgSDY2ei9Juj3pCLZ+6Qqf+7R556V+Tt\/6smYAHRV9\/vZCk2lQ0P317dds+H5hAaNWqUrPtZ02fFmbMzIvuysByRnQzZMZGwC7J2wqlQwYmzrNHbMLInFn\/NNdcIsfj27du700BY5shJQuybDs2fP79UW+z6ctOd8qvk8ccfd6eipLuuylC+4VH8LmUCD0brQzs3Tzj2oZ3zBOe+ZP2eoth6gVuvXv\/Gl4RYO1sFsNwRmr18h1vHvimD55BmAo\/id6DySBl35iw7ZPkhDjdE5GGTCkJG5F26dHH7n3s7dmFkMtXLpOyTzjFyHCfHNrt8+7IzIzryPXEG6LJlyyQZLVmyRO644w558skni9lE9VoO8Uz0Uo5Ox44dKxxZxxdNsvq9XpfpdaRRvZZDvK7ft2f48OECRftI+VHSZeq8qF7LIT7UDsPjDtG4ZBqP39w\/Vo7fuVT2L3tcLjvmbRl4zGK5qfl70mzTc\/LBX56U7evelIkzn3dOnj3UeTEJJ++dPTx0wcRXZcATqwQbHD9EfP6WaQtlzJxl7kvg5TW75LbJv5WZf3pDFv9tjdw+brLMnvei4\/1zGU0zjUe0fi\/zmfX+Iy5pxp15RXScfcrZZTFaFtvecr4nI3ycPXbYsG0A+6fDQ9iQNm3aVDgxhJE7pxUloz59+jiHO2zYMNE2Ub2WQ\/ztt9\/uytHp888\/T1OkpDZQry4T2VNUr+UQr+vnoUSeOXOmQNE++jp0qstMpdd2IZ56ff0+NTymi8YlDng88su75eO\/zZUaa56TD5+7U2ovGSd1n\/2pHP3CUEfIR\/1tmnP2OPwdb\/xJVr36J3lm4VKZ88dFMv6ZvzrnP\/FPW+Tht6rL+Ff3udF+96mr5MENx8l\/zd8tP56xXSbvaC3XvfR1x9\/3UUf5xd+Ol2j6ZM1LZc+PH5VRq5vIRY9tTkmUef\/2b7vyf\/TgG0V1HqTuU9+SB947WC96LYf4G1\/Y6+x1+vt\/nC57OwwRfIj7AMfgT6V05iHcCKcQoulSNGJnxM\/JRYRiQrZex40YP368cFKR0SzDYZZhUJbPwW9\/\/SuBnvrVCPndPdfJ07df7ujZwecL9Ny13xHoD\/1OkP+5tGaCkDU9d23bIrtDxN7sqYgw0E0XnynJ6LJOp4qmHv\/RSdJF+BDvT7Kdxt6ZEwtn9MdomxeNmOQMgcaxcBwPR+ydfJz68uXLhRALMXq\/JJI8wi+kEDeD+L1RvjvpyXAwHNL9DHzvtBaiqUOLuqKJeH0qGloU309Fqb4IKjoPHxIXOnxnnuaeEAtntMBoO9XhFcTEcdw+Hk4ohfDKGWecITh5nD1raAsLC92xcmluthVvCBgChkBGEYi9My8tGkyacuTcoEGDhFE8e6VPmjRJ2DKTcz9ZNQPBY1vacrNlxy8S2t+\/f39ZvXp1tpoRm3p37twpI0aMkJ\/97GfCZGpsGpblhjDXYHiIfP7558Lnn5cQX3nllSzflexUX6mdOROekIeOI+cYvTOK16timCxFB8F7+zinLKFs27at3HPPPTJnzhzBuce5velu2+9+9zvp3Lmz3H\/\/\/fLCCy8Izj3ddca9fJ71P\/\/5z+7tzbi39TDbV+Ll8+bNc4M4VrqtW7euSn5eKrUzL\/EOx9CAidquXbuKjv3zAPJrAuIFJ5r93nvvSZMmTaR+\/fpSq1Yt0TF\/8nOFSosH7w9873vfcx9SRmG80JErGOh+lBYPwoWLFy+WH\/\/4x\/rynONLiwdhVT4zv\/jFL4RBHeHZnAOjhA6ZMy8BoIrMxoFfcMEFsmnTpkSx6BiFs1576tSpMm3aNOewvvGNbzgnnjDMQYa+lxYPus9onHW+l19+ufC+ALpcorLg8dxzz8l5552Xkzj4e1oWPNhPplWrVvLLX\/5Snn322Sr5y82cuX9y0pwywiC+OWXKFDnmmGMStS1dutR9IFkHz0ocRlxM1DZs2NA9kJ9++qlA1atXT1yTC0xZ8WBF0l133SXMibCqKS4YVFQ7yoLH2rVr3S81FgY88cQTQoiByf+KakscyikLHnxeGjRoIHxm+BXLgocDBw7EoRsZbYM58wzBzaQr8X0etmiVhFP8z0I+lKy84Ui9Rx55RO68805p1qyZC7dEr6vMclnxePTRR2Xfvn0CJrwFCE6Vuf\/RtpcFD0ahP\/\/5z+Xuu++WK6+8Ugjb4cCiZVZmuSx48Hn54Q9\/KA899JB7UxtHTniyMve\/PG03Z14e1DJwTfPmzd3s\/OjRo6V3794ZqDHeVfBWJGEoHNjQoUPdctN4tzgzrWNCH8pMbfGthRALc08jR46Um2++WXLtl2xpkDdnXhqU0myzbds2FyenGkZYrJmHPyyqxBcbHsVvnuFheBRHICyZMw\/jkjHtOeecI8TJWa3CC09M7DEqz1gDYlaR4VH8hhgehkdxBJJL5syTY5ORHJZRXXjhhe5Veib32OjKx88z0oCYVWJ4FL8hhofhURyB5FIWnHnyxlSFHD6crA8m9f1ld0deaOIlEK33+bmc0l\/D49AdNjwOYQFneIBC6ciceelwMitDwBAwBGKNgDnzWN8ea5whYAiUB4GqeI0586p4163PhoAhkHMImDPPuVtqHTIEDIGqiIA586p4163P2UPAajYE0oSAOfM0AWvFGgKGgCGQSQTMmWcSbavLEDAEDIE0IWDOPE3AVr5ircWGgCFQmREwZ16Z75613RAwBAyBfyFgzvxfQFhiCBgChkBlRqAyOPPKjK+1vQwI7N27V3r16uXOcuQIPU+DBw8uQynxNOXUnBEjRrjdMekj27X6lvp++yMDvd6n5IMBBzZ4naWGQBQBc+ZRREzOOgJDhgwR9qrxxKEeWW\/UYTQAZ8wpU9ddd125SmHjtb59+8oDDzxQruvtoqqBgDnzqnGfK30vGckyou3UqZPAM9Ll+DhG7+j8qJXRLTryBgwYIH5Eiw15AMH16D2PPeR12HF6D4SeenHI2GODDsIO8vnYwKPD1hPnu7Zo0UI4PcfrkqW6fOqgPMplwymuod+kRhWMQA4UZ848B25irnVh3LhxiVALTtg76q1bt8qcOXOke\/fucsstt8iMGTPcCL5nz57CNdiNGjVKOJFo6dKlbp\/4VNjgdNmxkX3kIQ6BQMc1mzZtcsey4Yipl5S85cuXC\/zcuXPlvvvuk8aNG2PuzuTkLEr2pj\/ttNOczv9ZsGCB4My9TEp7cdYQ9pSJnl8h\/hcJ\/bzqqquEkTl51EW\/4I0MgSgC5syjiJicdQR0mGXRokWJEe3xxx8vRx11lHDmI862W7duzunjGHGyS5YscW3HOeIAOUfVKZL8Wb9+vXPM2EM4VBwv5nXr1hVOfKI+6kWHffv27V17GClzkHLr1q2FM1z5MqBdLVu2dPnYQ4yqcfCUhexJ95Fr8\/PzfZZLGaF36dJF9JFwfCHQBmdgfwyBCALmzCOAmFg5EOCQa5yvH8Xi9Dlyr6ytZ\/TryyBlZFzWMnC6fAlA8GW9PmpPGAjHfdCRR3NNNgTCCJgzD+Ni2hgjwCj3448\/lqefftq1EudHbBkHiIKRLiNiQijIfnTNqFbrsV+4cKEQh0ZPGZTFNSHCnl8AhHO4hpE\/qR9Vr127Vhjh62v5hcBRgIzatT4Z7+vnwJKoDe2nDVG9yYYACJgzBwWjSoUAE4mTJ0+Whx56yIVZZs+eLZMmTZK8vDwZPny49O\/fXzg7EydKx3CoOF7CMVrPyPeaa64RwjU4YcIlISdKGRD2xOdx3lyD8yfcQvk42WiIhWsgRuuM2uFTEV9Q9IV2EkuH\/JwBXzYrVqxw\/UpVhuVVXQTMmVfde3+4Pa\/w63GKs2bNkpBDRUceNlSME+WYPUIjhFhw8OhxuOjIa9u2LSpHXO\/1U6ZMER9O8XryvI4yfJnUR73oKEjbw6PD0fIrAKeNHCWcPzpG9L48fy16ryPkQ720xRMyfWNytVGjRkK\/ucbIEIgiYM48iojJhkAZEMBBX3jhhe4K77SdoP7grHH0jLiVutQsXxb8ErnkkktKfY0ZVj0EzJlXvXteZXrM6NePttPVaUbNjJ4ZveO0k9XDyL68baFcfk3YqDwZuqYHgZx05nTMyBAwBAyBqoSAOfOqdLetr4aAIZCzCJgzz9lbax0zBAyB9CEQv5LNmcfvnliLDAFDwBAoMwLmzMsMmV1gCBgChkD8EDBnHr97Yi0yBA4HAbu2iiJgzryK3njrtiFgCOQWAubMc+t+Wm8MAUOgiiJgzryK3vhsdNvqNAQMgfQh8P8BAAD\/\/2efCQgAAAAGSURBVAMAcBADwHEy+fEAAAAASUVORK5CYII=","height":224,"width":371}}
%---
%[output:4aa8f07a]
%   data: {"dataType":"text","outputData":{"text":"PSM EKF Fully controllable\nPSM EKF is stable.\nSingle Phase DAB: DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 3.819719e-05 [H] | Internal Tank Cs: 1.151294e-04 [F]\n---------------------------\nSingle Phase DAB: Three_phase_DAB_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 750 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 750 [A]\nInternal Tank Ls: 1.200000e-04 [H] | Internal Tank Cs: 750 [F]\n---------------------------\nSingle Phase PSFBC: PSFBC_1200V\nNominal Power: 275000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls1: 3.030303e-05 [H] | Internal Tank Ls2: 3.030303e-05 [H]\n---------------------------\nSingle Phase CLLC: CLLC_1200V\nNominal Power: 250000 [W]\nNormalization Voltage DC1: 1200 [V] | Normalization Current DC1: 250 [A]\nNormalization Voltage DC2: 1200 [V] | Normalization Current DC2: 250 [A]\nInternal Tank Ls: 2.580123e-05 [H] | Internal Tank Cs: 4.734509e-06 [F]\n---------------------------\n","truncated":false}}
%---
%[output:29845dda]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAXMAAADgCAYAAADxP4bLAAAQAElEQVR4AezdCdxv1bw\/8PWLiMQhU9G\/E2UqOZTkptQlQlHKnJQydBWZIkQDJWNuuSHkKJnJNQ9dnZSb1EtRmYpDypgoQ8TL\/3nve75P++yzf\/u3f9Pz\/H7Ps87rfJ6191rfNX332p+19ncNv7X+lf9lDWQNZA1kDUy9BtZK+V\/WQNZA1kDWwNRrIJP51D\/CXIGsgQnXQC7enGggk\/mcqDlnkjWQNZA1MF4NZDIfr35z6lkDWQNZA3OigUzmc6LmnMlkaiCXKmtg4Wggk\/nCeZa5JlkDWQOLWAOZzBfxw89VzxrIGlg4GshkPpnPMpcqayBrIGugLw1kMu9LXVk4ayBrIGtgMjWQyXwyn0suVdZA1kDWQF8a6JvM+0p9goX\/9Kc\/pac\/\/enpHve4R1cIJzfKarzrXe8q8nv4wx+efvOb3wyV9Ne+9rUirajDy172sjXS4xfhXPmvITTHHt\/73vfSAx7wgKLs6hDZ0we9lP0ibBhXndVd2vIYJq2Iq4zSLGOU6Uc+da42qW3K2\/Otk6n6leOIp\/xVGff8hXs+nhO\/tijn0bZcbdOWXpQpyqic3UCmbdpVOe2FftWnGjbJ94uWzNs8lPPOOy897nGPG5p02+Q1Cplvf\/vbq5UVcfEbRdrjTgNx7LzzzunKK68cd1ZDp49Ynvvc566RjrKrg7qsETjPHj\/96U\/TpZdeOluKL33pS7PXk34R7XjJkiXpLne5S6viej5IuZVwScizfdOb3lTymZ7LTOY9npUX9Lvf\/W4PqckI\/sMf\/pB+\/etfzxbGNb9Zjwm5uP\/9758uvvji9JOf\/CQ98pGPnJBStSsGov7qV7\/aVfj6669Pxx57bBrnqO42t7lNOv300wv9veUtb+lalnLAueeem5Qt\/HTySDLuJ8tdvTTRjh\/84AenO9\/5zqsHNtydffbZY30ODVnPS1Am8xm1P\/GJTyxeDOQS+MxnPpPWW2+9mdCULr\/88sKNPz7hyp938fkX4eF68YWFbNNIoZpmP595ynn3u9+9eFm9tJG\/ay\/wtttumzbaaKPwnnWr5VPOunyrcsqqLuTDtIC8xOVndEPGdYB8ZFxOj5ywJzzhCUX5yRhV0Rs5aUlD2vIQDuLwDzl+IA4\/YUCOfx3kTSZQzaMuDmKhU2Hvec97Vms32hF\/I2AjYdeBal7KqKwRHi4\/YVEm8dTBfTddR9xuLtL+8Ic\/XAT7ctBeDFI+9alPFX5Nf8SVr\/yVI56He+VU3m7xlZ1cQPyybDntkJEX\/7JctONNN9207F28n97TeGfDjedw1VVXpb\/85S+zcZRVmevykqe8Qye+yrfccsukvhKI8IjLJc9f+CQgk3mLp1BuRB4usilH83Ijo3Jj1ZD5CQtZn28Q9+GKV01TY+rHxCMv6cVoBPG55rfZZptxVoOGvffee88SaATKV1nE51dXD+F19SAPXggyrgPk1TPux+HWlVW+UM1PWaplVPd+dP7+979\/tZGfUTJC8dXh6yPyHKbNKGNd+SPtNq4vS+SNxA866KCE0MXTPuI5u+8F5fBsQ07b1u7oPfzCJafscc8VP2SR4JOe9KQ1zGrKyV+4OGAwpezbbbed29a4293ulm5961sX8vJVVmUuPGb+yIsuvAszt13\/K4sykS8LuecvvOw\/X9eZzGc0r+HpacuIB29UCzNiSYMg6\/rQQw+dHZXFSOCkk05KGoYXxItOzogYSXjJjeT4lUFePH6RJnnxNJbIT3gT7nrXuxYjlRiNGJG49hJsv\/32a0Rdvnx5QeTKrmwgf4IxsuynHuKVoa7SNHJSBmHdyOP5z39+KsuJWyVE8ZvQT1mH0HnSFkBZPCejN+3GiE+6\/MsYR5spp9\/mOuzjm2++edpkk03SLrvsUkSL51zctPzj2Xiu6q6Niqat07\/rMupkoyzatfa97cxXo85GmuTF58\/PtXSvvvrqVGcvR8zeU\/ovQ9riPvvZz05MUsj26KOP5pWivUtf3tLwLjDfrFixoggnKIyMDlp6yhR+3coq3nwik3kP7UeDIBYN0UM1quUHSFDD1jB8EvrE9qIIe9rTnjZr52Mf1pj4B8iLJ36EaVjikelGgMLKuOc975m8rBqdRgiuvQSIvizrWiPVKLkau09GIydhgX7qEXG49AOujVCNflyPE\/2UdRidI4fjjz9+DbOVZxjEgsCjruNoM5F2G9ezZR8nu8MOOxTkpgPS3pQZkQlrA+1TGyarjR5++OEui4lV+i9uVv3x\/MEtWfZu14gZQevAtT+2f\/5MXNWRPH\/pepfElw6\/NpA3kI13waDiWc96Fq9CD95tN\/RDT67r0LasdXHn0i+TeQ9ta2A+yTVADZH4hhtuWDQG1+BTzieda5+E3EDZRMOveh\/yiFfjixFGEKvRtVG2uE1Yd911k3KRQSDguttLoPEicPnJV\/7ku6Fa7up9OZ5yIL2y31xeV8tWvR9W50jFKA5J0F21bi996Utnv9DG0Waq+TXdx6iSjDZVfd69iEy8QFWPVpYgSJ2CuYSQ4\/ZqA94nBK48OhcjffGqkK70q3lX5ar30guTWTxv6USHK1\/vtngWCcjHdR3alrUu7lz6ZTKf0bYRh1FCFfxngpPJo9\/97ncuJxYKFp\/PVlucf\/75vFL4FTer\/michxxyyKy9Uj3V3RfGKpHstNCADsvIku4gzARIw+i\/RRJjFfGcfdk1ZaIT1yk1yYwj7IgjjkgIV9rbrjK1hP74BWJQUkfmOhLmObovg58wddOZRVqDum3LOmj6o4qXybylJo2+jTaIG215UVyDkbMRtOtqo4tRgTCo3oe8z16Nu9woXRsBGgmK2wvlkdIvfvGLwobOrxqvXF4vEFNLVaZ6Xy139b4qP5\/31bJV7wfVuWceo0lfNb5uyvWkayTCT57IfhxtRvptECaKXrLdbN7VeOpU9jOa1XGps7qXw5qu\/\/znPyfvEBkDCB0iXbkvg77JeTeM3sth\/VzH81ZORO+9KqNpfibKIL+msgqfb2Qyn3kCem+fXVXwnwlOTCjIPEa5SPe0004TVMDnq1GAxmLG3SQT+7VAk5sxMcaWGmkKA\/LiiR9hGlCQhpUQ5NqgnC95ZeDnuhti5IOYfIGU5cSVBj9hZFzX1YP\/OBAvIp2DPOiTXl0HymUVRkZYXVkH1TnCYXeWrudlRFluMz7hkZvwaCvhKnvbNjMqXfs6UB7tq47EkJOyskkjftdN0D7pk4y2EJOK2gj98+8Xvhy0d9CplOPHoCPev3KYa3Wj8\/IzcM1PGBntR0egQ+BXniPwbpH3rsmffBOaytoUb67Cxk\/mc1WTMeZjosSLbPKHSUJWCFxDAI2c31vf+tZkwo+sOPw0II2LXNjo+AfIxwRhpKnxefm9hDFhE\/JNrnyDbMgZFfJzXYaRPls6P2VXNsSEoPgFxI16CCNDVj3ud7\/7hdhIXCM7k7USk36sDvEy8gP+8qdPeuUXKJdVGBmy4oRMuMPo3MQ3PURada5wEDZImxmFrpGtTkEZupFtdGr0VSY5cbqBPulV\/ZSTnDZC\/67bwPxOtFPtXHsH1+X4zD\/yINtP+pGGMoL2HgsKor2rg2uy5fJHe1MWZWJikT+58OPvmt8kIZN5w9NApkY0XsgQY5Jgmoh7bp2cOOIKIwM6ghgNuQ\/UpWkkwfaNeEKujRuNkWyMCl1XoZFq6OHv+swzzyxWaXi5jeqEVeuhPuq16667Ch4Zyi+cRJXBZ7z8q\/rupkeyyqaM0oBusoPqHKkwC1TLJC\/gL5yce6jLSxmVVZnJgGt+wtxz3Q+i6yBC6SCjcnn4gdE0onfdZiJU2wXyEOVTbvf9wAoRzybiaO\/an3bIzxdjmHXKbVpYG0i7\/BzkR5fKHPFd8yuXXzxlCRlmHh04\/\/ATXi1rhM2nu2jJXOP2sMu2s+p1N1uah1+W7SaHiIWFrJdao3JftYVX06yG1zWSiCMPeZEJP3m45ocopcdP\/vyq9acLL3dVjqy05SE+1z3\/MsrpqWc5zL248iAnvnT4RRnJKxu\/QIRxw48rvZCVjvTEB9f8yEFZVt3oghxU062Gk+mGalx5Af+6OPyFB5RRWauy\/ISR47qvytAhXZJRv2q4+3J+dMWvinI6UfeI1y1vackXqjLl9Krlci+OcpNTlvDjL3\/tT7h7YZGXMpEPuCdTwuyej\/ATP+TDpUtlDhnX\/CKcq30oS8goj\/JKL\/yEV8sq7nxjwZA5W55Pp4D7+VbutOfvU91EH51y3asT+yL7oWvmGi+A64zBNUC3dJx1PbgOF3vMBUHmiNvaXp9Mek+fuu5jEmyxP+RB64+kkbX4bJc+gZFN2WbYZMoRL6OdBrKu2+kpS3XXwNSTuVGiWXCTiPHJ5DPMfdtJne7qySE+L8t20tBInb0xwrJ7kwb6ucq67kdbWbaqgakn81i+VB0hum8zqVNVSL5fUwNhu\/TVE6izN64ZM\/v0q4FJ07UvBjZiz13Z+q1Plp87DUw9mVvx8K9\/\/av20Ppe23TnTs1zk5P69sLKlSvTyh646KKL0kU90C2NyH9uapxzyRrIGggNTD2ZR0UGcdnYRw2mnUHwjne8I\/XCkUcemZrQK77w2bItX566XbfRSbe48oAop+uAONI+66yz0lkzsPzMel1uGdGRXHDBBQnIZJxXbH\/PepgcPdhlPQjvjCvOSMn8xhtvTL\/97W8LuB5XoUeRrgcRpDFKt9uItZd\/jGib3FHUe67TKNeHDuh6xYoVCb71rW+lL3\/5y4kL\/NwH+b\/hDW9IL3\/5y9Nhhx1WwDW8+MUvTgcffHA68MADC9jIYuPHPvvsU\/yuqx19GU\/Punj6eHWgLeKRuX6nuuU3EJlbRnXCCSekN77xjcX6Tol\/\/etfTw996EPTQx7ykAJbbbVV+uAHP5j++c9\/Cp44zOVDsLOxHyxdujQtnYFfD7Iz0nUVy5YtS8t6wFK3brBDUvq3u93tiuMK1llnnYR4f\/CDH6RBgKjL+NWvfpVAmnDDDTf0bAPKsGTJkrR0pu6gfjvuuGNxhrhrm0fAkb7W\/pKF9ddfvzCz2fa98cYbF\/VxTXd2ktoY44hgK5ze\/va3Fz+5Zv0w+Ik3u2y1VfeBF73oRcm\/N7\/5zavJR\/gkuN3KPqqyjSL9QdLoJ04b2V4yTeHdwrQPA5C55BHtsQlrNQXWhRlZOVrSS2EJ4B577JE++tGPpjjb2E4puMUtbpGMrL7yla\/UJdPg11+QF7bT6SS282rMJUuWFC951T\/uP\/nJTxZE4cHU4XWve10aBerSbvJDLvCMZzwjPfrRj05PecpTkvsykHEv3Oc+90mASEHjA6YOo1tnhbj+0Ic+lJg5PFsE3Bb0+I9\/\/CPpEOQT0IFE2ZRfudUh\/LiPetSjknZEFpC35yXNKppIXjwox735zW+eQDwdiRVPyvmJT3wiHX\/88cUoP0b\/K2fmD5RfB0YeLMFURuU2KHE\/iVBGXyNbb7110Y5HXcZRpD9IGv3EItdY4gAAEABJREFUaSPbS6YpvFuYQWu1nc73fV9k7oU45ZRTijJzHdCjMfkMvuUtb5nOOOOMZHkVuDbyQ5h\/+9vfijjj+OMALKMwZFRO37010mbjy\/7lay+5FxkR1KEsOx\/XiMVOM25d\/kgXAasrHHfccQlJw0477ZScRwH8gQyIU5de+N31rndNsGzZsrTLLrsUeMUrXpFAJ+7MD19i3I9\/\/OPJee\/CAvvuu28RJ+JKpwpfcUhoxx13TDvOAHHq4HSeXPdIHpYuXRpFW81dZ+ZrQhhIf9ttty1Izb3yryZcuaE7oI\/o0OgN6E6bvuSSS5Lzdj7wgQ+kD8yAbC\/dVbIZ6636N7WPYTMfRfqDpNFPnDayvWSawpvChtXvqOP3RebXXXdd+vGPf5z8DJnzHm51q1ulxz\/+8emOd7xj8jmLVKOAiPyBD3xgchqbeOE\/atfnNnupQ3NsHpI+17kmCMH9NALRAPJAIsgYkA2CRjhc9\/yBHHloqjOiQ35BtkgYSUOQNKJ2LwxCVjzxm9IfNkzHipCRPHiOQfJGSt0I3osH4vpKEBfBK\/vuu++e3EvLvXo0lZPunWGP0E3cAh3TN91DWf\/IHuhe3Ka0c9j8a2AhlqAvMjcydxbxWmutlTqdTqEPI1ukXvYrAlb9EcfSwVW3Y3FsEmLyMRFmhyLXiCo2EY0l0yES9bKDFx8BQxBFkAWiAPfCyIA44nbLHtECskJayAsZI2YEHWTt3gj4yU9+ckFy5KFbut38mW902NxuMlV\/sm3jlGWRvDIGKQfBM+UYTCDxal7IXRrMLcJ+9rOfJX46BfG\/+MUvJqdVso3TE33R2xZbbFEMUsTpBs\/B8\/BckD14Xp5bkL1rfogeyIojbrd02\/qrV1s9tk2zLDeK9AdJo584bWR7yTSFN4WVdTUJ12tNQiFGUQaEbmNDwP0o0h0kDS+qF9aLC8gYvNRe7oB7\/kBOHGjKE1EDwgEEBMg5iBppu+e\/7777JnJIULxy2khtk002Kcit7N\/P9SBp9BOnlyyC33bGvGLkjYgRtE7K6B3qCB6xWzkDdP\/Nb36z+B1Lee04Y\/KhN+SO5JE9fQJ\/eQB9QpOutAPwTBE9yM9z1wbKhM8f2UO0haa0hSnvsM9POt0wivQHSaOfOG1ke8k0hTeFddPbfPkvGDKfKwXGy+mFAy8hVF9Q9\/yBHHipxe9WVmQLSAIBIw0EgpihTNb8gRyI0y3dxeaP4JEy0CFy5zaRu4lQ5I5wjzzyyPTud7872eV6zjnnJM8E6FnnCJ4HlJ+Je\/kAWc9EvCb9aw+gfch7+fLlSZvRfpA9IH73\/JE9kNeemtLOYYtLAwORuU8PyxOBXdHyw7Iff+A3CnWaYGM+CfiFkGq67OQRznVflWm690KBF8SLAl4e8CJ5oaovljByIJ74TXl4sb3gXnRAxoAEjPwC7vkjDXLiQFPaOay7BpC7EXqQe9vROzJ3rjtyt\/rFROlZZ52VEH85N88VPCPPDDw\/z9EzLRM+f2QP8WzLadVda1fal3Y2COGLX5du9ltYGhiIzL\/whS8UqwZ83jo43wHuZT\/+wG9YdSFyPwPmRWJCsQPOmStlQkfc1hCHDPu5+6ZTEy1ds6yRbT1IGmEjbiQNXh7wIvV6IeJl9oKClxaqL7R7\/kAOkID4VV3pDLNN9IZiEp0uqvqJe2FNeqoLR\/DIXTs1mf+85z2vWP7ZNHrXDlasWFHsnD1yZvSO4JE7\/yhLN9fzBc8b2YM2oD0ge0D87vkje9A2oFu64a99Kof2WiV87braxo3ugbx44kda4dbpLcLauoOk0U+cNrK9ZJrCm8La6mCu5PoicyflHXHEEcmGobYgL94gFbJ00LnZdvrFZKalhta0W62CrMkMcmqiidl11lkn+bLoVTYvoRfKiwheNvDieQGrL6IwciCe+L3ymPzwhV1C5F4evXuG9kv4abUmgkfuBhFB7q4RfHX03kZ72on2ot0ge9DGQBsD7c298tmHYCJXHOiVB8IG5N2G8HVW6mM\/gjjQK48cPn8a6IvMrVrRw9vs0RbkxRukipYd2s32\/Oc\/f7XoRtRW0fAc5tREvW75BfISeUnAC+PFqb5AwsiBF0h85RgHdDZ5gmud1EsHvfTUFN4Upv097GEPK1b7GCW3sb0jPASPLIPgkTuMoo1ob9qd9nfAAQekQw45JGmroK2CduteW1VuIC+e+E3lQPagHkbtyNymMl+s4H0Go333vmCN8IG8eOJHHk36DZmq20+cNrK9ZJrCm8Kq5Z7v+77InG3c2Svs4W1BXrxRVpQds9PpFLs77fy09BHBV\/OwakF41T\/uhXlBo+Fr\/Bo9tGn4kU52F4cGlixZksqj96rtXXiq\/NMGkTsEuSN65D7I6L2SfO0twtZ+tWOje9C2tXNEH4Tvmh+yB\/LiiV+bcMkTYSNuBK4+gNgRPKIPwnfNTxjCB\/GglFy+HIEG+iLza665Ju21116z9nL2xl4gL94IylokoRPRCEu7Owv\/hfjHl0OTLXjYOo8i\/UHS6CdOG9leMk3hTWFt9IvA2d7BwAAQY5NpBokjdwQYBI\/cQVibfMkMU3aEDcgb2UOV8O3ytjzTMk1h6gXigDI0AeED4i6TPnIHhA8IH4Lw3\/ve9yamHfNj4jfl0UYHvWSawpvCmso1H2F9kTnbNxt42MtPPPHE9NjHPjaxY7\/kJS+ZtaUbsWy00UaJvA083FFUjn38kJnPSmnV\/foN\/37hrBIvkIc2aVCXDTbYgJPGUTYJD5v+IGn0E6eNbC+ZpvCmMMdQ\/PWvf+1L9z7LkaRBjvNokKDJVWd5QLcvSOQOCJ6tmhvk3u3ZN5W9W5x+\/NXDXJUNVDqrpz71qQkcPgXW4APSd68jU2fQmd373vfuufFKHRA2BOEz67zvfe8rztBB8mXC1wkE6Rvl09Ef\/\/jHYu6rW93k0dTOm8LrwvDFVVddJWii0BeZs31TbNjLVcgEogd60EEHFQcnCdN7f+5zn0v3ve990\/nnn5\/WXnvtoSuNyHUM8nQeiA5k6ERnEogX55e\/\/GXKyDqINoBcmAiZ4sJvUFeHgNjAblUTqw4as2u1G7kjjCB35PXOd74zOSgM4X3nO9+ZqLYaZk71UT\/AAToyI3sLFLhgELb\/\/vsnE7dAJzDzKjb+9zygOsqnGwT\/zGc+Mz3mMY9JXDDoe\/3rX1+cG2RFHNMSDPoMf1nih+XLlxdHMzcWeB4C+yLzcvl+\/\/vfp09\/+tNpt912S7e\/\/e3LQcW10bgwZ1tce+21hV\/THwq3PjzgPGoELg63G5F7GTqdwU5NlLZPSA1Pz52xQco6+D8daFd3utOdktHpqHWy2WabJecWsVGHeeMFL3hBQXBGtOzy2mYZ3oErrrii2MjkJFJH9jqP6Ic\/\/GGCUZdx1OkZ4YPBINOrRQ3gFEswIIRTTz01waC2fINLoBNzayZwQUcCznECnQroDJCzc\/TtK9B5Q1P98YWvkPLzmYTrgcncpKbP0Ouvv75rPfSCXQMrAR6sdeQBq1isJtCIuxG5JIY5NVF8h4N5edZZZ51iW\/skucoXOhxHuUaR\/iBp9BOnjWwvmabwbmG+Qp0EOg6916XpsDp2aOYMZMFUiTCQOyhnFd4NZkJASnatOo46bM11+fTjJ79h21+\/aTigjy6sVmPSic4OuZsrM7oG1\/zoyAjfMk2dIx3qgOXbBIQPSH\/FihXFMd6+0p2WCUb5sN9++xWjfT9E8ZGPfCSB45Kdj0+XTXnMddjAZG40zpZmosJnX7XgF154YaGgbbbZpnbkXpXvds9G32RaQfh6WqMUm4ekw7UO3UvhPiNrYBo1UJ5cDXLXppG7Acj\/1emmv1bOlM0zR67a2GRketZZa+5cvSnm9F0hbMSNwJH53nvvXXtEM9Jnz2faQfj0J4640uhVc6YdpM\/CYAQPOk6bDX292XzYK425Ch+YzFXCZ4oRjM+mnXbaKdmVCa6dxuczlS2d7CAVsikIKV955ZXFCpowwYSLtKXrUC27Po3ghXEp22ed8GmFnr\/XGuth6jaK9AdJo584bWR7yTSFN4UNo9txxEXuSLw6ekdmTQTPzmz0iYiC4JE7IP9uZR2FbgZJo584TbLIGkxGI3vr8utG+ggfjPTZ+RE+IHyQRp2ObDyEurD58BuYzBWWicKPTyBs92zo4JpSfJKQcT8IkDE7Vpheqi4Sj3Rdl8PdR9i0umbn89LE8WznjzYxbh1HPuNwld18VJzdjoBiBI\/coS5fI3jkDgieeYFbJXfpD9v+BkmjnzhtZHvJ6CRNUtOjUTvCB+RuVH\/MMcck9nyED\/x1qMxcdfqdL7++yNzsvp+C0wiiIre97W2LX7dhxzJBA66NjoXNV8VyvvOugVyAedIAckI2EOSO6JG7kX21WMjdCN17jdRj9M7+bjAlrBpnMd4boYPRuhU7lkROkh76InMmFYVnSrEMyfIq5g0P24SosHGDaUXeTDDlvPgzsQTcl8On8brpE3IU9RlF+oOk0U+cNrK9ZJrCm8JGoeNxptG27MgdiSN3pB4E38s8471G5kg9CL5f+3vbMpb11E+cNrK9ZJrCm8LKZZ6E677I3GTjq1\/96mLtuF5cw\/D5sfPOOye91Qtf+MLEP0bto66g3Z9HH330GskibqckamhMLToY91XCXyNi9sgaWKQaWLJkSfLOVgne6B2Qf1U1RvBN9nfh1Tj5fu400BeZR7E6nU6yhMjsMBu5mV67QW9xi1sUk6BGzuMYtVsnajI0ysHVcdiUoENhY+fHXu7eJ6P7Kqblvpetb9h6jCL9QdLoJ04b2V4yTeFNYcPqd9zxR132JTMEj9zBQM3OVe+xkyPryF39ELgBHLC9g\/cu7O+DlLGfOG1ke8k0hTeFqf8kYSAyr1bAely9+Vve8pbZUfuTnvSkYtLAkqBRnM1i9O0c8+rIfJhTE6v1yPdZA1kDN2kAubMROzkSuYd5xrX3vY7gkTvzDHJH6pbxWTZsi34Q\/E055KtRamAkZF4uUKfzf6N2ZyoYudtJNaw9Pcwr0rS2s5yf3Vqxnbjs71rDEu56GjFue90o0h8kjX7itJHtJdMU3hQ26W1m3GWvSx\/BI\/EYvQfBN9nffT3\/4he\/KEywCD7s78gdkH9Z13X5lsPL121ke8k0hTeFlcux5vXc+4yUzJHqj370o8LUsvXWWyfr0JlFbnazmw1VMz27JY7Wig6VUI6cNZA1MHINIPhB7O9G71BH8AZiIy\/oAk9wJGR+4403JudF2P5qnSbyZT93kuIZZ5xRnKo4qB5NYtrab4utCdhB0+kWz1ZoIwO2sUmDBm3rsLXE4yjbKNIfJI1+4rSR7SXTFN4U5rgK64\/HoftRpNlU9vlO34jWZh0k77RIphmnKbqufl3Hu6k+yB3Y3gHRx+i9rk7i9HpHesk0hdeF4Qu70qPck+IOReYqZEH9VlttlZytYlSuYla82M5vM5Ft\/\/wGgc8zi\/Ydukr0iPIAABAASURBVBWTm4Ok0xQnGowzKCYNthJb289UNI6yjSL9QdLoJ04b2V4yTeHdwrr5j+M5DJrmgGVsfeLiKNKPNBCxuTUnJFqjvc8++6RBT4909G2cHhnpN70jvWSawuvCdDDOamnilfkI65vMjcJNRu6xxx5phx12SA6St23fxKRlis4x33jjjfs+9rbu1MTLLrssXXrppckqllg\/bjOSw73Y6MTRy3c6C\/PURLZJP8LBNfcwakh32PQHSaOfOG1ke8k0hXcL0660axOAo9b7qNLrVvZJSr+pjIOeHom4rX9nDbAG3jHbK1euLE6P9DVVrX9TGcg2hdeF+cpwwNd8EHZTnn2RuVGiH6NAqK7\/4z\/+I33jG99IZ555ZjLZOcwo3MjeGvEA04pDujy08ONaQ+54XWvKxVnIpyb6VM2Yn9MsbZAzksz6n1v9OzGRaaY6wWr1DCDXKqH5gmcuBeRuBY0TJJ0eCaN+hsrAXFQtx3zf90XmJjj\/\/ve\/F2de77777unxj398MnKZz0qwoy\/UUxN9mg57NkbTsxlF+oOk0S1OXVnbyPaSaQpvCqsrzyT5jbvso0h\/kDSqcUywIneIUTEXudvvUvdM2LrZ3sHqGQfv+fWisL+X41TzaxtWlpuE677IfP31108Okn\/oQx9amFfYvkxyHHXUUQnpDLsEcVCF2CRkxO6LgTmG6+GNy84+aDlzvKyBrIHhNYDcjY6RO4sA+7sNTkHwwqq5GL3XLY\/0hX\/OOecktvFqnGm774vMLTH0Cyk2B11wwQWJu+GGGyYTAo94xCPSrrvumq6++urEbjUuRSBuppcqUfNnhgm4H1cZ5irdddZZJ+UjcHvroJeemsKbwubqOQ+az7jLPor0B0mjnzghy0KAxBE8Um+z\/t3o3fEEfpGI\/Z15xoIIo3dYOWOHj\/S5gz6nuYrXF5mXC8W8YTZaz3b22WcnZwWbHDU6d0aLCVITpfzK8abyOhd60WnASM1Ps3EXXeUXSIWXLGk+f0Z4taoInmkGDFKROwS5V+Un6X5gMi9XwoaeV73qVckvDpmAsMrFeS3MHSZMTZaW5Sfp2qSJSZJJhM8\/9j46HUf5RpH+IGn0E6eNbC+ZpvBuYZbWepkNVsah+1Gk2a3so0hbGqNIf5A0+onTRrYsY026UTaYxGQm3nTTTZMRel3HjdwhyN37iNz93JwzqCaJy0ZC5lEhZhhL3U455ZTijBbLFZlhTJyGzKS4Jk48TA\/GOvZJBFsgQrFhahzlG0X6g6TRT5w2sr1kmsK7hcU64tw+lqdh2l83\/Ta1537itJFtkjnwwAPT4YcfXvxQth919r75wR2mlyaCZ4Kx6oaFYkScNnQyIyXzcmksUzQ5QTl3vvOdy0ETcY3M3\/zmNydLIDNOz3o4Pesgvwf\/1wYsprCow6+lsb9b6LHnnnsmq2eAbR6JMdNsscUWLicCYyPziahdj0IgdJ9ZGdsWv7Ga9ZD1kNtAfRtA2iZXAcHbNGTjYg+KmdPgRU3mc6rpnFnWwJRqIBd7TQ0YlccIfc3Q+fHJZD4\/es+5Zg1kDWQNjFQDmcxHqs6cWNZA1kDWwPxoIJP5\/Og95zoKDeQ0sgayBmY1kMl8VhX5ImsgayBrYHo1kMl8ep9dLnnWQNZA1sCsBjKZz6pizQu\/Pbrbbrslv3a0ZmiTTw5byBrQLqw3dqgbOFd\/Idc3161\/DTjYy8Yo7QMcbdJ\/Kv3FyGTeRV8IfOedd04\/\/\/nPu0hk78WqAT+W8rSnPS051M12\/5NOOinNxcu6WPU9jfU+7bTTkt3v2ohNSHbDGwSMsy4LmswRspF1nRK9fHrMgPtQNHk7Vx1wf7vb3S68s7vANDBo+3BaqB9GoQ6nWm6++eYuMxagBgZtI9qHdhIqcX6VH9KJ+1G41TQWLJkj5IMOOihdd9111ToXo6iXvvSlyagqek73Hhxhxw94EOuuu67bjAWogWHaR1kdzvH3M4Zbbrll2TtfLwANDNtGwtTiwEE\/oDPuc1wWJJkbZduWfOWVV67RpCj4\/e9\/f2JCiTPRnX3u3mh8jQjZY8FpYFTtw8vutNBjjjkmGQAsOEUt4gqNoo0gb+fdOIGSmUWa41TpgiNzCtMTHnrooQmqyvvLX\/6SrrrqqrTLLrusFuT+29\/+dvKCrhaQbxaUBkbVPnzF7b\/\/\/slPkcWgYEEpapjKTHncUbWRUAPzCjPL5ZdfHl5jcRccmRtlM52wWdVp7Ne\/\/nVyJO9d7nKXNYKdWyx8jYDssWA0MIr2gchPPPHE9OEPfziPyBdMy7ipIqNoI1Y4gVSZ4vxk3Xbbbed2bFhwZD42TeWEswZmNPDXv\/41HXvssemrX\/1qYievm0CfEcv\/F7kG9t577+QX2LQPpyu+9rWvTeP+gstk3tDoKN8D4TaI5aCp0cDwBb3VrW5VnP3u668Mo7nhU88pLBQNhL082shctI9M5gul9eR6ZA1kDSxqDSw6Mmcr73Q6qc427oxi4Yu6RSzyynv+nU5uH4u8GTRWf1LbyKIj85hZ\/tKXvrTaA3Pv90vnaYnZamXJN\/Ongdw+5k\/305LzpLaRRUfmbFkW8H\/qU58qNg9pQJYimdDyc1DuMxavBnL7WLzPvm3NJ7WNLDoy98BMRjgvwXp0s83ct771rWOfbZZ3xuRrILePyX9G813CSWwjPcl8vpU2TP7Wmq9YsaJ2LbCHETPNXPfD5JXjTp8GcvuYvmc21yWepjayoMl8rh98zi9rIGsga2C+NJDJfL40n\/PNGsgaWKWB7IxCA5nMR6HFnEbWQNZA1sA8ayCT+Tw\/gJx91kDWQNbAKDSQyXwUWsxpLFQN5HplDUyNBjKZT82jygXNGsgayBroroFM5t11k0OyBrIGsgamRgOZzKfmUa1e0HyXNZA1kDVQ1kAm87I28nXWQNZA1sCUaiCT+ZQ+uFzsrIGsgayBsgZGT+bl1PN11kDWQNZA1sCcaCCT+ZyoOWeSNZA1kDUwXg1kMh+vfnPqWQNZA6PXQE6xRgOZzGuUkr2yBrIGsgamTQOZzKftieXyZg1kDWQN1Gggk3mNUrJX1sCgGsjxsgbmSwM9yfyf\/\/xn+u1vf5suuuii9PnPf75w3fOfr0LnfLMGsgayBrIGVtdALZkj6m9\/+9vpgAMOSJtvvnl6yEMekp74xCemgw8+uHDd83f\/ne98J\/3rX\/9aPdV8lzWQNZA1kDUwpxpYg8x\/9KMfpb322is95SlPST\/4wQ\/Snnvumd72trel5cuXp2984xuF637XXXdNF154YRGO6DOpz+lz6z+zHCNrIGtgQWtglsxvvPHGdNxxxxVEvvXWWxfEfc4556Q3vOENaffdd0\/bb799utvd7la47t\/ylrekc889N5199tlpyy23TPvss09605vetKCVlSuXNZA1kDUwqRqYJfM\/\/OEP6Xa3u11asWJFevWrX10Qd69CdzqddPe73z0deeSRBamL3ytODs8ayBrIGsgaGL0GZsn8Tne6U\/JL1Le\/\/e0HykU88fuPnGNkDWQNZA1kDQyrgVkyHzahHD9rIGsgayBrYP40UEvmv\/nNb9LDH\/7wdI973KMn7n\/\/+6fnPOc5ycTp\/FUj55w1kDWQNdBdA4shpJbM11tvvcJuft\/73jfd4ha3SI985CPTEUccUeBxj3tcus1tbpPWX3\/9Ypniv\/3bv6XzzjsvPeMZzyjWoC8GpeU6Zg1kDWQNTJoGasl8nXXWSVdccUX6+9\/\/nr74xS+m97znPcVqFStWTjjhhMIPmd\/73vdO7373u4vJz3ve857plFNOmbT6jb08f\/rTn9LTn\/70xi8Y4eRGWZjvfe976QEPeECR79e+9rWhki6nVfc1Vi2\/\/MjJX9yhMp\/jyO9617sKnSl\/oFq\/cRWJruhMvnTYJp\/yV7K40qiL97KXvayo1yB10TbFUy7p1KU\/qJ\/0otzqLI8mkBk0L89WPdRn0DSmOV4tmV977bXps5\/9bLGGfJNNNlmjfpYoWp746U9\/Ov3+979PJj933HHHdMkll6whmz1S8eXii8aLOY368OU1zeWncy+4F71u+ey81k\/hGvDd7343XXnllYXE9ddfXywHLm6m4I\/2bvPhkiVL0l3ucpdWJX7uc5+bkHIr4ZKQTqPu2ZZEFvxlLZn\/4x\/\/SH\/+85\/T2muv3VUBwsiQJXTXu941xbX7jNU14IX0Yq7uOz13yv+pT31qegpcKSnChor37K36jZsMzC9dfPHF6Sc\/+UlhupzNvOHiS1\/60mqh9nXomFbznNCbX\/\/618mS5wc\/+MHpzne+c+tSTlMdW1dqDgRrydx68c022yx95CMfSVddddUaxeAnjAxZ2\/kvuuiitMEGG6whu5g87IT1opbxmc98JpmDoIfLL7+cMwufzD5By5+ddZ+ZXl6jypBzrSOdTah0UU3TRLYRUkmk8ZJJrVx+BLjRRhsVcarl56kcyhNlq8vPSCvCw+UnfqBaR3J0oz4hEy4dCQ\/IX\/wIr3Oj7J6FZxJ11MFuu+22RRSjyKqujPgiH263vMplinJH3IijLsKkQ77ItOEP+a9+9auFhLblwvMA101QD89CXnQdZXGvDNLuFl\/ZyAXEL8uW0w4ZefEvy9lU6Gti0003LXsX70P5GcSziDril7\/85S+zcarlCX0SkKe8Y6BBNzYxqm85PMrJJS+e8IWEWjK\/5S1vmQ466KCiV33CE56QjjrqqMLswvTimh9lPO95zyt08ZrXvCZ96EMfSkwthUf+U6uBcqP2gtCjxl4W9pkZDZE\/PTNxaKTuwbUGXY2r0VfTNOLceeedU9PLK8010dtH\/sqhPCEtv0MOOSQFuapn3YiXnzDxyKp3OR3+0t97771XK7s4ZIUHxKMjugq\/bq40HU0R4SbzTz\/99GK0bMNcjCCl5aUPkgh5eW233XaNZZKH51CNG2m0dYMMdaYveMELUnQ61dF6r\/ToulyWKJ\/2Uo1Lrqpf8UOWXp70pCfNmn4ivufOX3j46UB1nvQVfm1cZtxb3\/rWhegwz1tZlEnZisRW\/XHPX\/gqrwXh1JK5mi1btqwgaAR06qmnphe96EUFvAjs6B\/\/+MeTzycjs5\/97GfF8sRnPetZoi5aeBH0\/GV4qb08XkSgHMR60kknuSxWBMXI5NBDDy38pBMvj2uNT0CMmhGKF5xfQMM8+uiji1sjHGnGqFP+nlsR2OOPF7lcfmWO\/LWFanTh8pFflP\/SSy9NP\/3pT5MyffjDHy6iRNnJisMzPqfJisMv5KKO5bKX9SYveYacMtKVNOpAJ6EzclFHhK2c1TgITJrIKEaRkZcyHXvssUWHJW7UUb3UT7mUr5pmP\/c6OPoRx3vmndthhx3cproviCKg4U9Vr0Tf\/\/73F3VwXUadbHQgdEcv5bqSF5+\/+rtW\/quvvjrV2cvpz3sRzyBcaYv77Gc\/u1gx1+Z563x1wp6vuFEux41IT5nCz3OpK6t4CwFdyVzl7nWvexU98OvAAAAQAElEQVSmFhObGjJcdtllCZELI3OHO9whnXbaaUnjvdWtbsUro0YD0UAFlUdc9MYPjEI1PNdeHi9EvND8QZgGfPjhh7uchZdIw0U+0akadcqX0CAEIF5A3soX9+FKXz7ujcDk72VlL1VOL5qXyPJWHZRPYO2IfB1e+tKXFqPeclwvJtmy3uLlJfe0pz1NcLGqis6Km8ofcieeeGLxiV8OojN1K5seEDR9kTvwwAMTW7draYTedT46odC78LIu6Eq6\/AeBtOUh7i677MJJoV9lRlSFZ4s\/dEX\/ROvqwD+gzOCerI7ENWKmW7u8PU9fM\/x9mRkAuC4jyi++dMphTdfyBjLDPG\/x25aV7EJAI5k7CtdpiG9\/+9uTZYeWLKq0l\/K6665zOR+Yyjw1eJ+MCu\/zk1v+nHSPFDfccEOXycvjq6e4mfnDX\/jMZfHf6gDEWdzM\/Ik0EWl51CPfmeDCZIZgXfcLZODlLecvDfkrh+tu0FZi5BVlqcoiS6Yg\/uXyl81NwqKOyMwLH+kaRQuv2lr5lSEfE5A6kxilR7h8mRYROT2ZuBNW\/RpRX\/UmT44M8BPmGujKM3M9CHxJyUNcelNXzzX8dPLIVXgvtK2DdJRZ2V3XQZ4IXHmaOma6UdZq3nVplv08mzCZDfu825a1nP80X3clcw+CacURuD5NvvCFL6QbbrghIXGfmPy9PNNc+VGXHekZtVTBX14+x5GF60mFZ10tf4yM+y2zz2Qj7YgnbSNZRBx+4cqjGmb0iTSiEwzZYV0jxfhiUNf4OtJJKMOw6Q8bXxuJL4NuaRm1G\/12Cx+X\/xFHHFEstZW+50hfnqv7MnxZuq8jcx1fmK7oP8BPmOfg2Ys\/DNqWdZg8JiluVzI3oen88v\/6r\/9abd3nHe94x\/Ta1762GOmdfPLJ+YcpBnia0cB1huVZeyMJI3JJGiGtu+66LgvwF17czPyJkc\/MZfE\/0vQyeCniBQnXiNTItBCeoz9RRqNgI6741O+WvRGhL4Aoc7UTjDpGeiEXLoJG1NX0kSPbuI7BqLKsR7KRrmujQSNstt645waiTvRMLvwNfoTFvTw8s7jvx0WQCK0pjvyM3ptkIkyd4pqrnOJX6yCsCb4Uo046QM\/KM6vGibp7Tkbv1fC29\/FcpKP9xHMOt9vzln6UwXVTWYXXYgo9a8n8j3\/8Y\/ryl7+c9thjj\/ToRz86rbXWTWKdTqc4t8WPV5x\/\/vnJBqMprPdYimw0gTCq4C\/DMKuUbZ9hIhBu7kGjdc1O6kWJSS\/+woUhp5jsdA9eGo3eS7p8+XJeBZgqlKeOxAqBOfiDmBCUrNQDXAfKphjX4R8vM5MHAirrLXTqpVU3dVTXiFt2ETzbLT950xX5ADOGMOQmj7K8iWpfGMLLevdLWyYlpUXvwssTip6VvPj3ixjVSlcaQV7hRidn9K5MvdKnq9Ar+Wg7UYde8evCw8xD\/+pdljFAMVCJ9l4Oc62NMhmF\/sPlJ4yMZ+9ZeCbajzrwl1+v502ujKayluWm\/fomli7V5G9\/+1uxs9ML0Ol0SiE3XTq3RU+dNwrdpJNeVzFBZoRsYo28RhqNOYjdSCJGsV5cLzVZ4WR93mrg\/AIIKCYCy2m6JhN5u54rlIkOYSo7t5q\/+gB\/4eRAffmxp9MZuOYnjIw8EJ6XPiZ+hVdBp6HHaljcS1se7kMeuSAZeSkjvUvn+OOPL1ZclPWuHMpDVifgrH9p9QMdR6wt9\/5JvxpfR89PWeL5um9C6DXqQLbfNuFLsTy4UFdQb+kFdNzKRtaAJPzbusoInoVnIl6v5438ySmLMjGxyL\/sx184v4WIWjK\/+c1vnjy473\/\/+7V1tknIqMBKFmvSa4Wy56wGEA3TRxC0ADPt\/IS5D7A\/Cot7L7Mf0ta4ww+ZkKvGFa+aJhl+5bwjnXG7yl5dQYIklV3eYff1wvtkF8a\/DH7s6eHnOuKHH30gQC9\/+FVdZVmxYkWx6qoaFjqSdoSFvM40\/Lieg+ch3D3Qe7lMUZ5tttlGcF+IFRwiBWm7vgmp+GUvefCLUafrbqBDiPCo7yBtQl3LOlGOM888c7U18GHWCYKNfNu40tYWtAnynklZt\/zkWX3e4vEXDsxBVhTxdw\/Cq2Xlv1BQS+bOWtFoP\/rRj65xEqKfl+NvB+gDH\/jAdNvb3nah6GKgemh0Gl98Ate53ezVyEdYOU7dC1bNAymRi7iuo\/DVNMnwi\/A6Vzg55SinVSfLjwxZccTlB675CSNT9eOPDIS5JisOORDGvwx+wsqI+CFHH2VyLctWr6UX8cKtlqMcB5mEHNez9jzKMq7LZepWHnWVl3TIi1dFuXzdZNRVHtKJ8kQ5474pXWVQlpBRH\/GkJ53w57rnL5xc2Y+\/cmyyySZJuHvyUYdq+d2TaYL48iijGk+edFCWcc8\/0lYe5ZVe+AmvlrWcxrRf15J5p9NJL37xixNTip1Sr3jFK4qlcnq5Bz3oQelVr3pVEXbAAQekTqcz7TrI5c8aGFgDzCLWqDOtsOWy6UqMbdrXq2uf+4jFdUbWwLg0UEvmMvMp5nhbmyQconWzm90smYTSA77yla9MJtmsuCCbkTWQUlqUSjDSM5Go8uyx7LKI3Zctu7H3yESe8IysgXFqoCuZy9SOTmeYsxH+4Ac\/SD\/+8Y8Tm5PJlMVuXqGfjKwBI26f9L5aq9pgo63adqsy+T5rYFQamCVzm4GOO+64woTCjNIG5MUbVWFyOlkD06qBsm22bKP1JTsfdZIvG7GysGHPRxlynnOrgVkyt7vTLk8Tm21BXry5LXL\/uXWLYT2vo3vL+NWvfpUC3eJl\/6yBrIGsgUnTwCyZl3tyvfm3vvWttP322yfH3F544YXFEaH8rSF9+ctfXhxYxHYu3qRVqm15fFmY6C3DWu3ATjvtlMoIf245jnQCH\/jAB9IHVkFnAeXO4oILLii2Q7OvZpyXdXFe1sG0vge\/+MUv2lLNnMjNknk5NxuBXv\/61yeTO9anWqoY4WyEPtvsDnU0bszeR\/i0uIM8iBixc8sEjbADJoYDQfBl4tcRHnbYYSngHkLm4IMPTjYUBcxP2NwB5i+smMh4evG7q1kPWQ\/z2Qa8tzfxyPwzXy2Z\/\/73vy\/Wl9uB1umsufSw0+mkLbbYovgVIlt3578a\/ZfAQ7jmmmuSjQWWXto9WIVNG4Fly5alZatgdU+g\/5xXj2GDFjiREnSWS5YsKc6BXjLjrr\/++sXvJzoHxPbojTfeOIFrfmBpnGMXQF3ApFwvvPnNby7OqO8l1xQ+SBr9xGkj20umKbxbmEPmPCnhTfWfzzBlU85xlWEU6Q+SRj9x2sj2kmkK7xZG76wXeEQ7mQTUkrldnXZ3nnPOObW\/62njkMkVMmQnoSKDlMFXhWVlCHvfffdN+1aAFAOOAQ44\/TDw9a9\/PQXCjxuyEb\/cUcgPqp1DP3UodwDOLrHaCGK+I0b+73jHO5JNXp6XcLI6DUvnHDX6+Mc\/PimH+0EwSBr9xGkj20umKbxbmC38ntlWW21V7G4cRDfjjtOt7KPKdxTpD5JGP3HayPaSaQrvFvaQhzykn9d1TmRrydzvej72sY9Nn\/zkJ4sTEleuXJmcbQ6uX\/e61xUHce21116J7JyUdAoyidE6F0EC0oZyRxEEH4SP\/KHaKThqeP\/990+OkUUsIC3pgnx6qSVMQsxAzD9MP0w65gIe85jHFNvbrcRg5yfDfNQrzcUQTrfMWtwpr28u\/iLRQC2ZqzvyQSR+Vejf\/\/3f02abbVbANb8XvvCF6clPfjLRiYBT4WzWCLifiIINUAgEAkZYzEC77rrr7FeDjqDcCegAdATAXzjSB4QPTUX43e9+V+wdqBJ9TPIi+SaCN8o3t8JtyqccRrZtnDayvWSawpvCymWexOtxl30U6Q+SRj9x2sj2kmkKbwqbtDbRlczXXnvthBi++c1vJiM3pAKuHX3rV1nITEKFELfRqwOlrLhxMI97W60noXzjLgPiB8Rt5L7vKnMRcocq4SN6IC9eXfliRI\/kYyTfluDr0st+WQNZA+PVQFcyj2zvdKc7FT86fNRRRyWw0628uiXk5stl93aesqMy4\/AgB\/O4R0TzVa5R5GsNv1+T4Q6bHtIGBB5kz4xzzDHHpC9+8YvJyF7njeR1COSqeXYjeMfBvve97y3W51fj1N2rj3px68LLfmR6yfaSaQpvCiuXYxKvx132UaQ\/SBr9xGkj20umKbwpbNLaxCyZOxiIjdBI3BG3\/RSUvHjMMv3EG4Ws1TQOwkdA5fTcO+hIvcr+03Q97k+8cvqIns4QPVIvj+gRPHQjeF9EfpnKyB3Y5Zlnuum6nG83mfBvI9tLpim8KSzKMKnuuMs+ivQHSaOfOG1ke8k0hTeFTVq7mCVzI3AnJLKF77777snuTqPepgI7uP8Tn\/hEMhKOeE3y4whz+JfOxBK9avpWbgiv+uf79hpA8vuWzDZhskHuUE3J6N1Eqq8ik6zIHbFDVTbfZw1kDYxOA7Nk3ul00m677Zb+53\/+Jznm9iUveUmyfvlhD3tYspGlfFaLXaEO3hd+5JFHpkc84hFFPPFHV7S5SclaUSt0fE7N4oYbUr7uroMlS5akpz71qQWYaE455ZRivbqfErzjHe+42oND7ogdyuRuUnXSdewXt\/7617\/mtpDfh9XaAL5gDVitoU\/AzSyZR1mchmjpoW3nFszbru\/kt1i\/zGVScSKcydD\/\/d\/\/LQ7nEi\/SmCbXOmxE88tf\/jJNGjQapiLuOMom3WHTl4ajXu9zn\/sUvxfr573AOm2otoUgd5Oqz3zmM9MhhxyS3vWudyWT1XV1lH6vMvaSaQrvFqacv\/3tb4tjn+vKNQl+3co+qrKNIv1B0ugnThvZXjJN4XVh+MLuz2rbnu\/7Ncg8CmQnoslOvzFos4njb7\/zne+kK664onjxrEEXTi7iTJtrx6TRpJGmEfoPf\/jDVAUzTcBILeB44MAGG2yQxgGmI+Yvpo5JTb+ujCaiHfkARu6OfWCSqdrcLYukbzZ3x0aAHxKHqG9d+hEWbi+ZpvBuYd38I89JcMddxlGkP0ga\/cRpI9tLpim8LkxbtgN0SL4befSuZF7NyY9T2CDU6ay5vb8qO5f3lN3pdIoRVDVfJC286h\/3SNIEB9s6UkHoVSCWgJ2UAT\/cETDhFwg\/bshG\/PKBQjpIMAIEJgdlKcOqISNe9Sj7j+p6FOm3SePud797sU6+OqlaR+6hMxua9ttvv4To6a1JB73K0BTeLUxHbXfzqHQ9jnS6lX1UeY0i\/UHS6CdOG9leMk3hdWFLly5NU7MDNMhuGtxb3\/rWyTklJt3K5XXvbBlmorL\/uK91DAGfaMA+DLbUB5AU+GQD5h7zD+A6IIzcWWedlc5aBWlJVz7jrs840teJmlRF7pZEco12qnnp5NQfdtppp1SeTKWDqny+zxpYzBpoPTKfVCUx8zhRkDnI5iHl5LLz1xGE8ACSYGoxR+CzqtWl5AAAEABJREFUqQxxy2D\/DTz84Q9PAaPLgB47YCQJkVc\/LpIOIG3EFZ0AF7krO8JH\/uCan7C2pO9roNca7l7lHiSNchzETn\/Ivc1KGXUENnfkzrXG\/bTTTismqerKW86vGt4UVpWdtPtxlT3qOYr0B0mjnzhtZHvJNIU3hYWeJsWdejKnSEsj7fp0XKzt\/Ny3vvWtie1WeBsg3jKClMNFOIEdd9wx7bgKQfDcMvlHx6CjgLjnhpw4UO0YIk\/laVN2Msi\/SvyIHfEh+jrCZ+bpdDrJZ7k0BoG4bbfmR\/pNcZA7Yoc25K6js8b9fe97X2KaQfBgKSQIb8qvKSzKO6nuuMs+ivQHSaOfOG1ke8k0hTeFTVq7WBBkTqkI3Vb+gHv+kwLEHAiyrnYOiB2C7BG\/jgBcgzAy5Q4g0utV1yrhB9mXiT5G9TqGXunNRXgdudvURA\/0Vy0D0wzoxMDIPUw0rhE8IHmoxs\/3WQPTqoGeZG5j0De+8Y3imFdrbv1whTN8naA4rZWexnKXOwIkVv4yQGyA9AHpQ5n0exF+ED0zDiDCKsnPO8HPPDjkHjtV2dqro3e6mRFb4z+CR97qBYgdykRvEhvRwyWXXJJ8uayRUPbIGphQDXQlc2eWn3jiicUGIkRxxBFHJMR+3XXXJWaMAw44oLgfVb3sNvWrIezd1TRtyTcSZUKBl73sZVWRJJ6wgPs1hKbMY1B7XRA\/YgvS9wyhTPbmC2z8Iofs69RTJnkkGARvVG8Uf\/nll6d+7e791KuNrPpqHzYylQnetTo\/4xnPSPe+973rqlf4BdGbNFdHsI7Yenmmm26ET\/6iiy4qfshFGkVic\/injW6GKc4o0h8kjX7itJHtJdMU3hQ2jG7HEbcrmX\/2s59NJ5xwQkLafkIuMl9vvfWS7f42C7FThv+wrs7CErRqOojcMQPsoEwoZGwisdEkZBG3UxIRDBn2c\/c2ooRMdm\/SAPKDZcuWFbt8kTrSC6J3jRyFL1269KaIq64Q\/EUzJGYEz179wQ9+MFmKidwnYfSumEbwys\/27rRPphlr3mP1jHv1tHu5ieilBchanRE4sgcjeaN70D6RPrgG\/mSM9APiSycgXelnZA0Mq4FaMjdKdubKLrvsUvwwQnl5n2Nvn\/Oc5xRbub3Mf\/zjH4cqA7JGHFaj1CUU\/jYoCVeWww8\/vDjlT1xlnf9TE5Vs9Bj35Etd+kgegRvRM9MgvCB5zwmEV2uL4LWH5cuXpxi9dyP3unyr6cV9G9leMuXwIHltG9G\/5jWvKXaghrkmyD7MVEb1OgWIMvVyETQgbORNJwHkjuQDSF8HAK4DEU4eojPgShOYgbx\/XPlBr7L1E17WWz\/xyrKDpNFPnDayvWSawpvCyvWchOtaMo+TCDUwP09WLWin00kat98KtSOyGt72HhkbdVsn7ncMjfqrcX3GV9eLb7nlloXYd7\/73RRl9XIWnqv+uDeCl8cqr6lzxv2J10\/6S5YsSQgeEDyy42633XbFb5RWlVsmd6tpkBly599Pvm1ke8k0hZfDED1o29qPzsyonrkGED4gfOAXI3y6EEdckE5VJ73ukXFARwBIG+gvgNwhCJ8bnYB3NhB+XDIgXkDHUIZ8QL7gSA9fM762lKtX+evCy\/qtC6\/z6ydOG9leMk3hTWF1ZZ9Pv1oy73Q6CYmb6OxWOCS77rrrFnLdZHr5G2WvWLEiIXJpVeWNuq+++uq06aabVoOKe2Ww1T6fmlioY07\/IPelS5cmB7Ex0yA1BF83ckfgCMGzRuxMMhdffHHyG7NzWugRZYaoAWkjcCN8oAMED8ge8YNr4A\/kkH9AGiA9kPYoioqAA8gZkHUgOodwg+SRPsS8gZ24OoToJMLlFyAfiHS41v8zf3J1HJE3V3kCUU7uKOq+ENPoVadaMreFdYsttii2UtedDnbhhRcm2679lNy0HrAVirF9H9HogScNyugMEO44yibdYdMvp+GT1E\/dOe8GYTld07bn6pEKyB2Zn3vuuWuYZKr1LKdfDYv7XjJN4U1hvjqt4Ip8BnV1fOBoBvB1Y7I2oBMEPxYCTqE0Ig64DwgH8kDXAR1pwDxAwEmWoK6jBOINBClzEXXAnAoy5+o0EHwgyJ8bnQLXpLONgFz3AXKBSMPgwCDBAYDdOgtmKOZhZwHVPUM66fYe1IXhizpeJDufqCVzo3KHHinYnnvumeywu\/baa9Mb3\/jG5NPTb3+utdZayctKlty0QmPQyEZ10lxO56bTJxEhQjFyN+fxqEc9qphwtWu33F6QuxfSc\/CSvvOd70zIwMFu86VPJDUppyb68gzoGMEqJKDbQIz0uTrTgFU5YG4p4D7gXQ\/4gZmAd70MJjXwTMvQUQTKz3UU155DQEcR0D4C2k1A+wkE8XOd0Ak6CHAdOOSQQ4rTOw+ZcS32AAssAuYPfWEFfGX4ahlF\/UaZRi2Zy4AdW0\/HXk2BTB5nnHFGuuyyy9IOO+yQ9IT3u9\/9iE41HPOr8euZJw1Gc0aHvpTGUbZRpN82DV9xD3zgAxNzglEXkkAMzArVBqStGbl\/5StfSSbA\/\/u\/\/7tod3RRp4deZWgK7xaGMMd5YmVdPfr161b2NunYHR0Iswl3r732SgHvhefFhPT85z8\/mSyG448\/Ph1fwqmnnpqcjAnxNcF1f\/LJJydfElwmpkB0NFz5BORnhVG0De0DmJ4C1fYyyL1ResAhewFfi+BrImDVXnR83DPPPDNtvPHGBQbJe1xxupK5DBG65Ynf\/\/73k8kQJgmfLD75NtlkEyKtoIeL9d9c68m9sL0iG8FtuOGGiW28TpYt3UvX6Qx2aqI01ZHtl4lg0jDuk\/tGkf4gaYjjBE72dqReXi3jWXguAe3EyF37Y9oz6mJzt0RVW\/TMpNd0wmFTeLewbv7ymxSMu4zDpu+0TPCOcpFyAGkHdBYB5G4fia8DnUCQvzmHQIyQubgIwXJDlisd0ElYkaSdcSPPKIcOwldFtLdpdhvJ3Mahn\/\/858luzzvc4Q5p5cqVyScLZRs5ta24Xt3674AJT0TdJj7Crq5KsYpFXKtaJu3UROUaFZCGTpM7qjTL6Uh32PQHSaMujlEmW7KXjy2Yy\/5bJXflR+7MMkZORx55ZLHG\/eyzz04IXhslU0ZdfhHeFBYyk+qOu+yjSH+QNPqJo32wHnCDoLlB2joJe2UOOeSQYs8MggeEDzqIj3\/848UOd50D8AuQAXFAuwTHhRhoTFLb6ErmRuM+d7xYf\/7zn9OPfvSjdNBBB6XPfe5zxaevSRfb\/MddGbZWeeh9uZYaHn300cVxqFbD6BR8tvsct3mIDLfNqYlkMyZPA4jdy9kPuSNxBM92iuDNhbzjHe9IlkOC8MmraS7RJGrAaD2gY4By56CDwI3XXHPNRBW\/lsydv8I0YgLrhS98YUKYX\/jCF5J7PZZRuQp6cdgxx1kjhK3nNDpnotl2222TnthoP\/LVS9r16ZgBMtx+T02MtCbJNfPe71b5fso\/ivQHSaOfOGRNvnu5dtxxx2RUVDbLGL37hK\/W2+gdEDxoq0geEL17JK\/jt5vZJFs1jUm\/p5vcPm7oeZxELz01hTeFTVr7qCVz56\/4mTgvjkkRJH7++ecnyxU333zzZHPPzjvvnMjYgTaKSpmM0Ukg5mp6CN0LGWYavz1alREvwrnuqzL5fuFowOgduQNb6D777FOsrkLuYGTfrbZI3khdmzLZZaKVHT6IHtn7dSjt0Vp4CwDIi9ctzeyfNTDfGqglcyNzppUY8VgeZhLyQQ96UDFKV2h2LW7GQBpoFYmOh7VpN2U0ivQHSaOfOG1kQyZG7wjeQCRG8K4RPCB5HUE3vSBsQODIHNmzzRvJI\/ky4fMTZoQfEA\/xg3S65TMK\/6g3dxTpVdOQ7rDtb5A0+onTRraXTFN4U1hVX\/N9X0vmMamoYVrf6jPUCJyJQ4FNjJpssgSKLL9BwZzDNBIwuVpNi53ci9gk43M5wrnuq+nk+8WnAcSNwBE8IHbzQHVEv2zZsrR06dKeSkLSgLC9I0b4AeSO5CHIv9oBCCMXHQBXOtILSL9nQbJA1kBJA7VkzkbOvGLVCQI\/5phjChOLTQpMMAcffHBiQ\/dykC2l19clIj\/ppJOKnaZMIzoItvEyoSNy57fYBVaWETcyQ9xOSfSCkGE\/dz\/tpyaO2143ivQHSaOfOG1ke8l0C0f0RvPWDGvnlq8he0Rv5YLJdyYc\/gYTEITfhvSjfXKRM6xcuTKtnAHyjg6Aq+0i+UC5Iyh3BvxDRhwbWLR\/HUJA2iCfgLxBWdqim97axic3SBr9xGkj20umKbwpTP0mCWuQeRTOjK0NAtYDO\/L2P\/\/zPwtbuZ2fbOgmGTX8kO\/XtazHcrIDDzxw9ufd2MadiGglShCxVSrS9mJxQ8ZELKKXjl1tbPjs7mTYy91r9O4zsgYG0YCBClMjAjdwAcSu3QPSByN94Ccc6QfERfyg8xikHOIg4kAQNMK+4oorElOQDiGA5EH7D+gEQMcQcF9GyIoL5XmDakch7ygHN8rGVd6MuddAVzJ3loFJJZNDtvHbXKN4GriHbuRiUwG\/QSAdI\/\/yqhTp2GCgA3ENbPVWryBx92B9Odd684V8auK47XWjSH+QNPqJ00a2l0xTeFOYNtYGSBoQNvJG+gHkjuQB4SN\/cB0QBmSjE+BKC6QbkA+0KVcvGcRbBlIGRB2IeYNqR4Hs8UCg3ClEZ8E1sWwHL5dMyHOlEYjOgusLnVnXpjDlKSPKG3Vr8\/x6yTSFN4VFGSbF7UrmUUBmFSPgKhw0Y8uu8JAdhWuU0el0imNVjbrzqYmj0GpOo18NWKroRze4\/cZtI4+QA0HUiDs6AS5yB0QfiA5AhwBxzw0ZrnigUyhDHhB5cqMc3DZl70cmyDfc\/yPmlbOmpug0orMIN0ge6ZehQwAdRcB9oCwbaeggyog8V86YuwLK10+9JlG2K5kja+YVD549sYrtt9++OHiLTWlUFdNhMJ9UR+KjSr8uHUcUGAlMIiyL02BtYx9H+UaR\/iBp9BOnjWwvmabwbmFOBkUMCGEcuh9FmsqOxJhDjGJ1PAHvJRhZluHLF8oHZTnZMhCHdnG32mqrZEBlXsH778THAL9AOX3X8oUoS7jSgrr3cBg\/RBwIcuYGaUcHEa5nCp5vgB69awH3AfNzzmcRB3QMzFvmEIcp96jj1pK5pYl2XF566aVpjz32SCZAN9poo4TcX\/nKVxbni3uoDuJyGNEoCuUh23IrLSe4cccJtlAN2ANzVswkgplLYzvssMPSOMo3ivQHSaOfOG1ke8k0hXcLi1PxFnP7sGHwk5\/8ZHIejrkt7TDAL8CEUobBB+hkypAWaNNl8Csj4vhKDwQxh8v8CtFRcHFIYFjuiM6B6zcT\/LZD5K1TkPeyZcsSHhk2r1HFryVzphMbgszk26Bj675e2uSniU8PysFGlN7pdIYuiwcgXV8DdnuW7eNDJ94lAQ\/BiYns9szKnEEAAAocSURBVBmnFz8QkvVwetbD6XPfFqw+K+Ntb3tbgqOOOioFzNGV8YIXvCABk1LAcd0BXxaB8OOGLBMUtDVBdaGRySdzI3ObhvQ8nU4ndTqddK973SuxXyNeR7I6m8BqFPfdKhr+PlOs\/Q4YZUY8bjciN0k6zlMTEbrPx4xtU9ZB1sFCbQN2rgeWLl1a7CXAbWBuIoDcIQifay4CzE+A64Dw4LhJcGtH5n5wws+4+XSJQjq90OeGZYn8EKGRtNUk7ptgxYr13wEjQETdROSRnnytPWdPDz+rWFxb1WLTkpU2DqrnF3A\/l7b3yDe7WQNZAwtXAyaJA0tnOoZJqmktmfspOD8mYL23X1tRYJMmiJz5xa5Q9iOk3+l0BA+EI444IukQmkwrsb6cDV8mSH0xnpqo7hlZA1kDWQPdNFBL5kh6v\/32S84yZxJB6M5o0BP5qbiHPvShxS+LbLPNNonJpVviTf42BeksrrzyyuITP0ww4drVJj77ObI3OhfmU9CI22hfONgkxObGXEOGuxBOTVS3jKyBrIGsgTYaqCVzEdmTTHQidSN1ZhGjY8txHEn62Mc+Nr3kJS9JiJ98v7Bb04aEML1UXQQdaSJ0M8ghY1I2wsIlH+Fc9xGW3ayBrIGsgYWuga5kruJGuUbmVq64Zyd3FoTfy\/NzcsWyRAEZWQNZA1kDWQPzqoFGMnfwPfPFq171qlQH60wtY5zXGuTMswayBrIGsgZSVzK3Vf8xj3lMci7LRz7ykVQHJyfa6bVQ9Wiydbfddkvs+wu1jrle\/WtAu7A+2ZcrWHrbfyo5Rh8amDpRK\/VYNbQPiDnAcVaklswV5NRTT02W\/Dkkx9ZVdugq2LHZs8dZwPlKG4E7edEk8HyVIec7mRowdxRHMtve7RjnuXhZJ1MbuVR1GmCOtkcGZ7JuWIFnEFAnOyq\/WjK3dtySQT2LxfadzuDLD0dV0EHSQchG1nVK9PLpMQPuIw\/ythvbolw+wTHCs7swNDBo+zABH6uprPLyU4oLQyO5FlUNDNpGtA\/tJNIzMLYnJu7H4daSuQnPO9zhDskvCo0j07lIEyEfdNBBqc6mj7j9eIVRVfSc7j04ZfO14UHYOOU+Y+FpYJj2UdaGeaXrr78+2cBW9s\/X06+BYdsIC4cBsaXSz372s5MVgePUSi2ZG41aenjGGWcUm3rGWYBxpI2srUe3hr2aPgXnH7OoamVx3Y+qfXjZLQxwEJ0BwOLS4sKu7SjaCPK2290Jmcws0hyn1mbJ3AjW6hSNEyw\/vOaaa5JJUD8Tx68K8uKNs4D9pk1hekInL0I1fpiQdtlll9WC3NuY5AVdLSDfLCgNjKp9+Irbf\/\/9k6NR7ZlYUEpa5JUZVRsJNTKvMLM4aTH8xuHOkrlVKVanxKoVP9eGzI1kP\/\/5z0\/NahabhZhO2KzqFOZ8GccROJ6gGh7HXVb9p\/I+F7pWA6NoH4j8xBNPTM7ezyPyWjVPteco2ogVTkARTHGO0N1uu+3cjg2zZK5RWp2CCNuCvHhjK11OOGtgwjTgfKJjjz02OYqCnbxuAn3CipyLMw8a2HvvvZNTZbUPJzG+9rWvnf2t43EVZ5bMyxn87W9\/S8wNk2ZCKZdxLq59Pnsg3LnIL+cx+Rrwu7fsoNUBj9Hc5Jc+l3CuNBD28mgnc9E+ViNzq1esoXX+iglE57P4pSGfCaNTQk4payBrIGsga2DUGliNzD\/2sY8la6vZk5\/85CcnP0BxySWXJD8Z5YcpRp35fKSnbp1OJ7GdV\/N3TrHwqn++Xzwa8Pw7ndw+Fs8T77+mk9pGZsncLwv5GTh2QEsSbeM\/+eSTi9\/\/86lw2WWX9V\/rCYwRM8t+vKJcPPeO1s1zAGWtLL7r3D4WxjMfZy0mtY2sRuY\/+9nP0j3vec\/Vzijfeuutk9MRx72sZpzKL6fNlmUBv9U6liAJ45rQmrSfgVK2jLnVQG4fc6vvacxtUtvILJl3U+o666xTnFm+UMhcPU1GOC\/BenSzzdz8YxY0k0EDuX3QQkaTBiaxjfQk86YKTXqYtebdlk96GMxHAfeTXp9cvtFqYF7ax2irkFMbswamqY0saDIf83POyWcNZA1kDUyMBjKZT8yjyAXJGsgayBoYXANrkLmJQXbkgPXmDqyq+gt3QL\/NRYNnn2NOjgZySbIGsgamWQOzZG6i00mJT33qU1NbkBdvmhWQy541kDWQNbAQNDBL5re97W3TK17xiuQ4z7YgL95CUESuQ9ZA1kDWwDRrYJbMJ7gSuWhZA1kDWQNZAz00kMm8h4JycNZA1kDWwDRoIJP5NDylXMasgayB8WpgAaSeyXwBPMRchWYN+DGS0047LTl7xyqspzzlKSmOd\/75z39e\/JqWH5wAJ4bGjwpUU\/VDLX7TEVxXw\/N91sB8aiCT+XxqP+c9Jxr45S9\/WZwGuuGGG6a3ve1t6cUvfnFyWJLML7744mRF1v\/7f\/\/PbUbWwNRqIJP51D66XPC2GvDzh0bifs929913T\/ZO3PzmN09G7H58xLn9fsS8bXpzK5dzyxpop4FM5u30lKWmVANMJn626\/rrr09+eIWZxSmZqnPttdemSy+9NA3624zSll4dmGuYbeSTkTUwFxrIZD4XWs55zJsG\/MDKoYceWphSjMpPOOGEtPnmmxflueKKK5KfSLzXve5V3Pf7R9rSK+PVr351Wm+99RIyX7p0ab9JZvmsgYE1kMl8YNUt+ohToYDNNtusGHmvvfbaCWk\/7nGPSxtssEFR9gsvvDBtsskmyS\/HFB6r\/sQIvjri9sMt55133iqplKQtvcAOO+yQ4pTOY489tiD1WeF8kTUwZg1kMh+zgnPyk6kBI\/ILLrgg+fGVW97ylqsV0qi67kiLPffcs\/ihltWEV93ceOONCYFfdNFF6fWvf326293utiokO1kDc6OBTOZzo+ecy4RpwJLElStXFqP2atEe\/ehH1x5r8brXva74Ja6qvInUU045JX3iE59IBx10UNpmm22qIvk+a2DsGliQZD52reUMpl4DJj7ZtkexJNHyxpNPPjnttddeab\/99kudTmfq9ZMrMH0ayGQ+fc8sl3hIDRhJn3vuuWkUSxKvuuqq9PKXv7wYsR922GGJbX7I4uXoWQMDaSCT+UBqy5GmWQPDLkmMulvuiMD\/8Ic\/pFe+8pV5wjMUsyjcyatkJvPJeya5RGPWwLBLEhXvn\/\/8ZzruuOPSOeeckyxRNEL\/\/Oc\/n8r48Y9\/TDQja2BONJDJfE7UnDOZJA10W5LYTxlvuOGG9JOf\/KSI8rGPfSwdfPDBa+DMM88swvOfrIG50EAm87nQcs5jXjVw\/\/vfP5mk9EvrCsJ973vfm6pLEqtyZMu4zW1uk04\/\/fQC1qa7RujdIJ9y\/Dm6ztksUg1kMl+kDz5XO2sga2BhaSCT+cJ6nrk2WQNZA4tUA5nMF+mDn49q5zyzBrIGxqeB\/w8AAP\/\/AVSMywAAAAZJREFUAwCPXcpJWSm2EAAAAABJRU5ErkJggg==","height":224,"width":371}}
%---
%[output:51b29746]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAXMAAADgCAYAAADxP4bLAAAQAElEQVR4AeydDbRU1ZXnd6ujD\/HjoSIfIv1QbLQVDRoSbI3mQyM9xlZRnDyX0YQeQRRbV4PSotiorUQFgyv4gZnQY2JDDCsqMkYMrnFgYYJNHBpp2iHBQBD5kIwgqKDiZN7vkF2cV9xbdeveqrr3Vm0W+51z9tn77HP+99auU\/uee85+f7R\/hoAhYAgYArlHYD+xf4aAIWAIGAK5R8Ccee4voQ3AEMg4Ata9uiBgzrwuMJsRQ8AQMARqi4A589ria60bAoaAIVAXBMyZ1wVmM5JNBKxXhkDjIGDOvHGupY3EEDAEmhgBc+ZNfPFt6IaAIdA4CJgzz+a1tF4ZAoaAIVARAubMK4LLhA0BQ8AQyCYC5syzeV2sV4aAIWAIVIRAxc68otZN2BAwBAwBQ6AuCJgzrwvMZsQQMAQMgdoiYM68tvha6zERWLFihZx22mly3HHHFejKK6+UDz74IFKLyCH\/8ssvR5JHzrdFfty4cU630rackv1JgICpxkHAnHkc1EynLggcfvjhMnfuXPnd734nb7zxhrO5ZMkSl9biz7HHHiu0r\/Y2bNggOPRDDjlEZs2aJeedd14tzFqbhkBVEDBnXhUYrZF6I+DPpHG4ap88s+r29nbZsWOHsuXxxx8vzPCRKVSEZHDgt912m6xatUrWrl0rOsvXWTo2IPjwIPLKu+iii4RfF9iFf+6557o++P1GlvK7774ryE+dOtX1kV8k8NFBhjZCumlsQ6CAgDnzAhSWyRoC77\/\/vlx88cXOwZ166qlyzjnnuNkxTvLuu+92s3Zm7MygcXg4wKVLl7rZ9X333Sfr1q1zQ4K\/aNEiN7tXeXiussSfHj16uFqcrcuIyJYtW+Tee+91vxaYxfOFsWbNGnnqqaecCO2PGDGiYBvmO++8I3PmzJHrrruOotNl9n\/rrbfK\/PnzHW\/79u2yceNGV3f++efLPffc43SQoe98WThB+2MIhCBgzjwEGGOnj0BxmAWnhtPevHmz9OnTR\/r16yfMoHHyq1evdo5x8ODBcvTRR7u6k08+2Q2COhwvXwgQeXWiTqCCP9js3r27m6kPGTLEOe2dO3cKfaMf9Acbffv2LbR6zDHHyMEHH+zKhGr4ImHG\/cADDzie\/hk6dKjL9u\/fX3Qc5B3T\/hgCZRAwZ14GIKvOBgI4SZwljjlOj4YNG+ZmvcyIoSlTppRthi8NhPhyIIVwxDhrZt98KfhOm\/pSxOyakMvMmTPdrwRm3aXkrc4QqAQBc+aVoJVh2UbvGo6Q2S8zVcIf69evF8IbPp+ZLWEWwiLUrVy50sGCzoIFC1wMG3kcKjN8VxnyB7nJkyfLgAED5KijjuokxYwcIqRCKKdLly4uBET\/0FN+JyWvwBcBReRJjQyBaiBgzrwaKFobNUGgOGaOkauuukoGDhwod955p4unM0vu3bu3i0cTwiA8gaOdMGGC6KwZ\/ujRo\/eRpz2f3n77bUGXEIi2WzyDh08MnJQ4OKEcZvD0i7bgM\/NW2\/CU9NfFyJEj5ayzzpIzzjhDiPd\/9NFHKmKpIRAbAXPmsaEzxVoigMNevnx5p9AIywNxiNjFQRMugXyHSx7evHnzBEIO+es6Hj7Ch5CB5xNy1PmkctjENjKEXBYuXOj6NWPGjMKSRZVBn5k3DzT5BYFddKnHHmVkGNvYsWOdfltbm9Am7auM2obn61NvZAgEIVB7Zx5k1XiGQIMhQNiGGT3EzHvixInuQWyDDdOGk2EEzJln+OJY1\/KDgM64mXVDzKjz03vraSMgYM68Ea6ijcEQaG4EbPQdCJgz7wDB\/hsChoAhkHcEzJnn\/Qpa\/w0BQ8AQ6EDAnHkHCPbfEKgVAtauIVAvBMyZ1wtps2MIGAKGQA0RMGdeQ3CtaUPAEDAE6oWAOfN6IZ01O9YfQ8AQaCgEzJk31OW0wRgChkCzImDOvFmvvI3bEDAEGgqBDDrzhsLXBmMIGAKGQF0QMGdeF5jNiCFgCBgCtUXAnHlt8bXWDQFDIIMINGKXzJk34lW1MRkChkDTIWDOvOkuuQ3YEDAEGhEBc+aNeFVtTPlFwHpuCMREwJx5TOBMzRAwBAyBLCFgzjxLV8P6YggYAoZATAQy6cz9I7g4hguCF3OMplYVBKwRQ8AQyDICmXLmL7\/8suC4AYyjt3yCRx0y5I0MAUPAEDAE9iKQGWf+7rvvyvLlywUHznmKe7u4JwePOmT2cOyvIWAIGAKGgCKQGWd+9NFHy9ixY7VfoWmATKisVRgChoAh0CwIZMaZA\/iKFSvktNNOc6EWYuQQoRV41CEThwjN0A5Em3HaMB1DwBAwBLKMQGac+QcffCCTJ0+WqVOnypIlS2T27NkON0Iro0ePdnXIOGYFfwjfPPzwwzJ37lxHL7zwgsCroAkTNQQMgWZHIAfjz4wz\/+ijjxxcp556qhByGTx4sPTv39\/xhg0b5lKVcYWIf9544w3Zvn279OjRQ\/r16yeHHnqowIuobmKGgCFgCOQCgcw481qidcwxx8jBBx9cMLF69epC\/g9\/+IOsWrWqJPFLAfLlKEM+LyyPHOTXU4Z8XlgeOcivpwz5vLA8cpBfTxnyeWF55CC\/njLk88LyyEF+PWXI54XlkYP8esqQzwvLIwf59ZQhnxeURwby6ygr+fygfJCc8kiDdHweMlAQr5jvy2geGUjLpJSVKJeiIDnlkZbSpQ4ZiLwSZSXlhaVBcsojDdNTPjKQlkkpK1EuRUFyysN3FBxJBjJN4czDcF6\/fr089NBD8sADD5SkK6+8UiBfjjLk88LyyEF+PWXI54XlkYP8esqQzwvLIwf59ZQhnxeWRw7y6ylDPi8sjxzk11OGfF5YHjnIr6cM+bywPHKQX08Z8nlBeWQgv46yks8PygfJKY80SMfnIQMF8fbw49+7WdJ\/7LHH5F\/+5V\/2+RzSR+h73\/ue\/PCHPxSeeVFW8nEJygfJKY80SMfnIQMF8fAd+JAw\/1Jvfqac+TvvvCNDhgxxD0CfeeYZGTlypMvDoy4uOOj6IRoN33AhnnvuOeeob731VgmjWbNmCeTXP\/jgg3LTTTeF6viy6ELKO+ecc9xQaEN5pVJ0IWS4sdCnDMErR8hBqos8ZYh8OUKOvmJXZeFBWi6VoluMFbpQKT3qsAlYtEFZCV1Iy0Gpjhc5yJehDPk8Px9Ft1b6apv+YAMir0RZSXmagteyZcsK93SQnPJIVY\/Ut0sZQgYir0RZSXlButQVy\/k86iij29bWxmXe5\/OEDITM8ccfL+eee677LMKD4DNm2gkiZCC\/jjLEPelj5ctoHjlIy6SU0cV34ENcxzPwJzPOnDj5woUL3TpzHnoWE3XIVIoZMfjDDjtMNm\/eLGvWrJEdO3YIPG1n165d0rVrVxkwYEAo8WUC+TKXXXaZc+Y+LyyPLqT13JDXXHONnHHGGaE2VZYUXYj81772NUGXMgSvHCEHqS7ylCHy5Qg5xotdlYUHablUii43vy+DLuTzgvJhWKELBekoT8eLHKR8UsoQ+SCKolsrfbVNv7ABkVeirKQ8TcHr4osvFhwfvCA55ZEio+TbVR4ykJZJKStRhoJ04RfL+TzqKKPLijU+k5R9QgaCd8ghhwhEWQld\/75EzieVC+KBE6RY+TKaD9P\/4he\/KPgO+pwVyowzZ4UJNyLLB4OIOmQqBY4vABwJFw0iD6\/Sdqop37NnT7n66quFtJrtZqytqnQHjAyr6FAaXs2LVWacOQ6W2Tczcn7KsIKFPES5vb3drXKJfqn2Sp533nmFGT\/5vTXxc3wrM9Mnjd9K5ZrYi2s3j7qVI7RXI63x0oM0bWM\/DlXS53Xv7epkohLdToodBXS3bt3akav8P7pxPw+VW8u2RmacucLE7HvRokUudqY8HDs86pRnqSFgCKSDwP0vrZHP\/dOv5Ii\/fyWdDpjVQAQy58x1CSEPQLXHPEkmr3Xkk1A1dFtaWty6ddJqtBe1DeyxXp40qo7KoZM3Xe17nDSt8dLXNG1jPw6V6zOz8dlLNwn0\/PWDZPwFbc6hL169TcrpluoPut26dSslElqHbtx7OrTRnFZkzpnzgGPatGnuDVCNnS9dulTgUZdTnK3bhkDuEZi9dKMwK\/+3O86Us\/u3djjzfoJT\/5tHlwkOPfcDzPkAMufMwdOPnxMzJ5YOj7qsUFqxuiR286ib5HqnNV76nKZt7MehUn3GieuM3G97j1NvkzE\/edOtFqMNvz5KHp3Gi5lHGXl1ZTLjzImHsy9LueFFkSnXRjXq0\/p5l8RuHnWTXKu0xkuf07SN\/TgU1mecOMQsvO8RLfs0Pf6CfnJstxb56W\/EhVv2ESjDwC7Pxe6+++4ykvtWo2thlj24ZMaZM\/NmrSmhFd7y2tO9vX\/hUYfMXq7lDAFDoJYIECdnVj79mydJkCNX2zj0+19aa+EWBSSFNDPOnLGzbJCwCnkct0\/wqEOGvJEhYAjEQiCyEk6ceLjGyEsp+uGWUnJhda+88orceeedYdXGj4BAppy59ldPFcJ5K8HT+iykxPnSWN+axG4edZNc67TGS5\/TtI39OKR9\/s2Gbe5BJ6EVZuRR2kL3ir8Q6XXoAXLD7DejqBRk0LWYeQGO2JlMOvPYozFFQ8AQiI3Ahu27ZcZr22TIlGWF5YfMuCtp8OHhJzhdW91SCWrVkTVnHhPHtB68JLGbdd2YlyJULa3x0qE0bWM\/ChEPx+kyA7\/okWXy+e8ulfmrd8n4C9qE0EqpGHlx+zrev+jdKo+0n+RWtxTLhJXRtQegYehE5zeUM9eHpMTa\/b1cGvHYOP+DyIeR+CbET9xGorHPrpFJL\/9BbprzW\/fzvZHGVouxKF6XzlghOOhi4s1NiLc3SYmJc9\/guHHCOPHxF\/SL7kECJNsH93SrW7AdUG2sGiGQSWfOMkWcMU6ZbSZHjRol5c4ApZ4j4dg4njg7JxXdfPPNsnbtWqnFsXHE+eoZM8d586HjA+J\/EHEI8HHo0KtvbRNkg+h3734gxEOD6srx0tTl5z9Uro\/F9Wn1mX6kZRu7YHXAAQcI\/3DSPp11fKvgbHHcz18\/yM3AceCULx3YmmituP95mHfDIHl76y4Xe6cfpYjPEm98x3kAiq5vt5SdRq\/LnDPnnE+cMBtr8dOLtz6HDx9e9gzQgQMHyrx58wqbcQ0dOtRdu5UrVyY\/Ns611PkPPw3rsb4Vx4DDZhaFs+aDyQdPP4jvPfSVwgeSDyXEBymIXrxpsCwYe6YE1ZXjpaX77KiB8sSwnjLnb0+quN9p9Rks07JdjBf3SjGN75h5t3fMnomHcz\/pnZ3kng7S5eFplOWK6Haz1\/n1MsROM+fM9RAJHLmOSvcf1zrll0rnz58vbFp\/0EEHSalj42jjtddeE2bwfMvXiz7++GPZuXOn2xM5yCYzLGbcOHHqfzriRFkybpBMvbSfMIP6fJ8WOfpgCdVHp1GoHFaNMs5qjSMreHGPjv1aHxc\/Lze23bt381EMvZ8\/mE\/hxwAAEABJREFU\/fRT+eyzz+STTz4JlSlnI6g+Dlb4Cg68cR3O0J\/MOXPdTIufXYpTpRttETtHN+pyRsIwTz75pGzcuLEutGnTJtmyZYs7MCPI5uur3hZimT\/+1Xo3K73jy61yfNdddelbUH\/S5JXDKs2+ZdF21vC68i8PkO5dxMXvS+HFoTF8ZsNkCL3ieJnQhclUyo+LFb7illtuobvVpMRtZc6ZE1ZhU63Zs2cLDp2j44I22iJGztugxNU5OorwDGiMGzeORKZMmeJS\/vAtyk1AHtJj48hDHEnGaSW9evWSqNTa2ircXPw8jKqjcj169JDu3bsLBwkoT9NPD+omo57dJN86s4\/8+h8Gyzc+369Tn5LYzaNuKawUs7A0rfHSn7RsZxGvGd8aKK+\/s0ve+rCl070MThBYMevms0g5iI488kjhV\/ahhx7aqQ10a\/E5DOqD8vAVHHJDf7NEmXPmgMOr\/Qu9I+TIw6NOiRj58uXL3aETnMnHlwCOnFi5PyMnRHNYiWPjaI8wDGcQtrS0uL0loqRdunRxN1cU2WKZMN13PxK5YuabzpET1yzWoxymS105Mt36XF+ug2G9F+vjjj7E7a54xcz\/I79ev2ufzxhY7b\/\/\/nwU96kDSyVkIC2ToouTJ18pxdXFV3BsnOtwhv5kxpnzM0pXsDDbLibqkAnDjuWHOpNXXWbshG34FuXIOIh88RdDWJul+Nw41XwAyoNOYuQ4cSjMdhK7edQNwyEKP63x0rc0bWM\/DoX1OUpb5XR52Dr+gjYXPy9uD11+4Rbzo5TRrebnMIrNrMpkxpnjYJmBs6yQY+J4AEoeoszqFmTCgGTPFmR90hm7X0c+rI20+DhyYuTtg3u5ZWNp9cPsGgK1RGD8BXt2V2R5bS3tNGvbmXHmegGYfXNEHA5ceTh2eNQpL+2UJ+PVWN+qjpwbndlLuXElsZtH3XJ4lKpPa7z0KU3b2I9D9egzyyR5F2Lx6m2FLmLX9mYpwBE7kzlnTliE0RAyIYUqXc2CTq2pGj\/v1JG3D+4ZeUaexG4edUtexzKVaY2XbqVpG\/txqB59Zl0770jwS1QdOnaZsNl+5nGu2l6dzDlzHmTqahaNfQetZtk7hHzmeEtv2BMrnBNnVp7PUVivDYHKEeAXKG+i8oyocm3TCEMgc86cjhIb1\/g5MXDy8KhrBMKRs9\/IuQOOEnPkjXBFbQyVIsBbsuioQ7f9zEEjGWXOmRMXZ+WKzso1hUddsuFWT5s4X3jMvLSdOcu2yMYdu93bnKUl961NYjePuvsiEJ2T1njpYZq2sR+H6t1nJjL3v7TWbaA2YdFumf7BV4Q3ntXBRxlDkj5HaT9PMplz5szAmYkzI1fiYejEiRML+65kAWDifHGWRHGj8mbnjEt7xhpGXLsYy6Mu\/Y5LaY2X\/qZpG\/txqN59Jn5OuIU9h1a9f4DrMs+RcPBRV7wk6bMz2EB\/MufMg7Dl4cjMmTMlSzPzoH6W4\/HAhxt32uX9pfdhe27ecjpWbwg0KgKsaoGCxgefz0pQnfH2IFD8NxfO\/I033hBeyS\/ufJrlSn\/erXtvl9tvhZ+WZ\/fvFrvrldr1DeVR1+9\/pfm0xks\/07SN\/ThU7z7P+teNJbvJr9iSAh2VSfrcod5Q\/zPnzJl9Ex\/XWDkp+7OUe2koy1dlryNvc6tXstxX65shUC8EmH3Xy1Yz2MmcMw+KmRM79\/dbycKFiRqrw5GzHzmxQWblSfse1W6QnTzqBo0jKi+t8dK\/NG1jPw5Vrc8RjRMzjygaKpakz6GN5rQic86cmTl7qpAqpuSLeVoXlLKjInuZk1LPvi3M8CHdHhd+PWj20o3uxBXefKuHPbNhCOQFgXKTG16my8tYstDPzDhztrDFYQ8ZMkQ4+o0U5wuRByx9O5R8GNHO5MmT5f3333cifBGwX\/ncuXMF4mg5eK4ywZ8osToe4EC88ZbAVCfVKHY7KXiFPOp63a84m9Z46WiatrEfh+rdZ5w1v1iD+sqsvZyzRy9Jn9FvJMqMM+fNTzbGUkdOSnhFiTpkyoH\/1FNPyQknnCB9+\/Z1ojw83b59u7DPM0sJ2Q8Znqus4R\/CKzzA4egsbswamrKmIyNggllDgJeHOI2I1V18TiD6yOeG1Cg6Aplx5syWmZmTsnKF2TizciUeilJXamiEVZYtWybf\/va3O4mxX7k\/q1+9enWn+jjHxtEAm9WTMjvwiSPf2HtCj3fz6zTPhvqljo1TueIUe2F2i2WLy3nUZQx5w4o+p4U1tvOG19ivHesOYuFYRGhswFFzYcfGJcE5LlZ2bByolyAefDL7PuWUU6T4pSFm5\/CQCWuC8Mr06dNlzJgx0rVr1zCxQD5hmCefrM6xcRz5NurHK2Ro\/xa5+tSWwKPe4h5XVemxWI0gb1hVdpRhI+ClR81Neu7NwueHiRxfUpwYVq37Oi5W+Ao7Ni7Qle7L5CGlzsg1LZ6ZMwv3j437j\/\/4Dxdr5wCKIR1x95UrV8pVV10l69atc2vUuQnUUi2PjXt57W53WsqkS07qdLwVs2klQj5Fx8aFyqqOpq2t8Y+ry6NuHrHiWqWFdR7xCsLq9gtPkCf+dVvhqDk7Nk69V3iamTCLdpFvYM7\/fOKJJ9yRcMzKoYULF3Z6nb\/42LgvfOELosfIEW8\/+eSThfj5N77xDanXsXEPL9woz67YJsQBWTIVRnGPq6I9021xX5ZgUY7Swop+pWW7Uex++cSj3DsZDy\/cVLjeHBkHga9SGuNta2sTOzZOPXaZlBg3Z3eWEYtUTWiGo+KYsUPk4UVSLiHEzcQDVVLEWLUCVXPlCu0WE\/Z8u8X1pcp51C01nnJ1aY2XfqVpG\/txKK0+h9nV5bwsJAgbT5humHwj88vOzOs9eBwta8T9wykq7QNtzJs3T5i9o8tRcczuIfLwqkk4cW44nsDr0\/hqtm9tGQLNigAOnc\/XI69uaVYIIo87c85cwyycLqTxctLimHnkEdZIkCfhbIH7mw3bRB352f1ba2Rtb7Nql3QvN1oOHfpMGk1jrxQ6aeju7UHluTT7nKbtypHao5FWn0vZZXL0b3ecKdMXb5Gn1u77+Sqlu2dUzfM3c86cWTXxcWbRPsGjLkuXhkMmxj67RpiR18ORZ2ns1hdDoHoIlG\/ppWuPkxMO\/aS8YBNLZM6ZMzNnFs5svJhYvcIqlixcr3c\/Erlh3h\/cSUH1dORJYoR51E1yrdMaL31O0zb241BafY5ilxn6F4\/s+NAVDSyKbpFKwxYz58yZfbNDInuY68ycwymgqVOnCq\/qp301CKvwUpDNyNO+EmbfEDAEFIHMOXNm5osWLRKct3YSxw6vZ8+eykol1Vf0eSDz0xEnyTH7bxVidvXsDPbSiF2nZTcJtlXoc+zrm6btuJil1ee07MbFKat6mXPm+to9D0AVNF3ZwpudvXv3VnbdUnXizMZx5Cw\/5Gdf3TpghgwBQ8AQKINA5pw5m2lNmzZNli5dKhoz5yWie++9V26\/\/XYpfnuzzPhiV+PAOeaNkAqHzOLE2eWNJ+s48rRidUns5lE39gXsUExrvB2m3YsuabwPgO24lBZeadmNi1NW9TLnzAGKuDmrVzRmTp4PBnu3VPuQik\/6niU4apw2h0hAHCaLA9eZ+PgL2gQnPv6CfnQvE2SdMAQMAUPARyCTznzcuHGFWbnOzlnhQjzd73w18h+dPkJ+vOZw59A5xooZOTNvXlYgnBLmxJPE+ZL0O4ndPOrmESv6nBbW2I5LafU5LbtxccqqXuacOQ571apV7iAJHoKyRwspK1yYsVcbyNbn\/lZ+fkUXN\/PGcbOvCo6ckEo9lxxWe1zWniFgCDQXAtV35lXAjwMk2P2N+Pj8+fNFV7Pg6Es1zza47InObN5fk16LY+OSxPlKjaFcXRK7edQth0ep+rTGS5\/StI39OJRWn9OyGwejLOtkzpmzmkVXrODQFyxYIGxpy4EV5YCcNGmSsK8LsXbWpLPvMF8A7FfOkXFQtY6NK9eXUvXso\/yjH\/1ISEvJWZ04jAyr6HcC95ThFQ2vRsMqc86c1SyXX365bN682W2UhVNmps7BE6XCLDjtDRs2uFk8l5INtaZMmSIcEVfu2DhOGmLb3Epo8eLFctddd8nTTz\/t9lGvRPf1118Xvmj4cqlED9kkdvOom0es0rxOecQryn3J6WBbt24V0iVLlhQ+c1F0uR5BlASrKJNL\/FA9KXPOnMEzE\/d3PGSfci1TX4puvvlm9\/DUD7OwpS4zftXjhiDfp08fty8xM3fCM5XQ1Vdf7RzybbfdJpXoIaunlNTbbpI+p6WbR6y4xobXlZE\/F1GwuvHGG4UjIfmlDr5KUXRVtjhNcm+hy57m+BB8SRYok848DjDM5DldaMSIEe5Qi9GjR7tX\/zlqKqw9LsSDDz4oLHk0mmU4zDIMsvo5ePTRR+Xaa68VnGhW+ojvwIeE+Zd68zPjzAmTsPyQh5dBRB0yChAbbjH7RpZvXEIxffv2FeLsyPDwlJ9COHPSsGPjuBj8EjAa4p5NGA7JcDD8mgc\/fAe+JiuUGWdeDAhLEnmQqcSLQ37MnLAL4Rfq+aZua2uTAQMGyKuvvuqaIpRCeOX0008ve2ycU7A\/hoAhYAjkGIHMOHMcNQ4b58zDinvuucfFvpl5F8\/Kw\/BmPTqv\/qNDOm3aNMHJc1QcR8ZB5LEV1obxDQFDwBDIIwKZceY+eDhbdew494kTJ8rw4cPFD7P48pr39dCnTB0rW2gHIg+v6cgGbAgYAg2NQCaduR8PZ5Y9c+ZMYX24OueGviI2OEPAEDAEYiCQGWfOrJtwCs57zJgxwhIkZtIQMXHWn8cYn6kYAoaAIdAUCGTAme+L89tvv+1WVuDYlXD0OPx9pY1jCBgChoAhkBlnTgiFODcz8SCiDhm7ZIaAIWAIGAL7IpAZZ75v14xjCBgChkB1EGiGVsyZN8NVtjEaAoZAwyNgzrzhL7EN0BAwBJoBAXPmzXCVbYzZRcB6ZghUCYFcO\/PHH39cOGJOsfDXp\/urX2pxOIXatNQQMAQMgSwgkFtnjhN\/4IEHChhyytDkyZOF3RJZDTN48GChnuWMbDXL3uEQLx\/BKyhaxhAwBAyBBkAgl86cGfnQoUMLB1FwHXipSE8oogyxc2KUwymQNSpGwMqGgCGQJwRy6cyvu+46CdpjhZOF2C2RF41w9shxMdg9MehwCupumP2mxKX7X1ojxTR76SaBFq\/eJtAv31hdOBWFDcSMlhgeSwyDRvgcrF+\/HheSGUrVmTPDxvH6BC8OOoROiJPjxAmzcBA0oZhSbXEx5ry4UOLQ0wv\/XR589tcFuv+ltR2OfW3hi+FvHl0m0Df++9vyn3+6U0ihS6a8LMPu\/olc8Xd3RT6Jhf3ajaKfXGNYNR5W119\/vfzgBz8QDoTIyvXloAx8SCkfU8+6VJy5PpBkoDhen+Dh3JEhH0CBLE4aOuyww+TUU0919Tj1pUuXyvbt255JeNsAABAASURBVCXscAouxCGLH5DHLu4uz1\/\/uYpoxvn7ycSTNsj\/+Paxjn5+RRdRUh7p89cPkkfaT5JbLv28o8FfuVB2nXixfHD2rbLtkh\/K9q\/fL6ddc5\/c\/8jMSCf9cDOzjS\/71VRKedRlrFxQ+p6X8dJP+kvfyVdKSXSxmTe8oox30qRJMmjQIPnmN7\/Z6XMSRTcM\/yRYofvaa68JPgS8s0B1d+bMoPVQCQ2D+EDAw7kj4\/PL5TlhCMdNjBxZwi2EV6IcToFcpSfEXHbZZcIFDdL7q1P7i9LZ\/VulfXBPGX9BP0fzbhgk\/\/vv+smYnivl4eH9nYNf+H4PGbXgj3Ldy3+U3Ued6PalCWoXXim71JeiPOqyB\/0111wjZ5xxRklcgsad1njpS1q284hXFKw4b7Nbt24Cga9SFF2VLU6TYEV\/8DNZoro7c\/ZXGTt2bFkMosj4jdDu9OnTBT1m9lk+nKJnz57yneEXyvBB3Z2Df++hr8i\/3XGmnHV8q4z5yZuy7r1d\/tCaOg9WHNpL2tRARBw8OBle0cAqiVW0JjIlVXdnrqNnhk6MG8dbTKeddpqwZlxlw1IeeEJa7x8l52\/MxcNSZvsQeZVPku7atUvWrFkjpEnaUd2+R7S4cAyO\/NW3til7nxR7ce3mUXcfACpgpDVeupimbezHobT6nJbdOBhlWSc1Z85Mur293S0vxMlCHPsGTZ06VVgznmXgrG+GgCFgCGQJgdScOTPzRYsWCc5bARk2bJjA4+eP8rKatrS0SL9+\/YS0mn1khr7uvZ2hTWIvrt086oYCEaEirfHStdrZpvXaUFp9TstubVBMr9XUnLmu++YtTR3+M88847Jdu3aV3r17u3wz\/iHU0ozjtjEbAoZAfARSc+aHHHKITJs2TVg+qDFzHlree++9cvvttwtvb8YfVu01k8T5SvWOh6Cl6pPYzaNuKSzK1aU1XvqVpm3sx6G0+pyW3TgYZVknNWdeDMpDDz0kJ554orDHCutCWaJYLJOlcpKfhuXGUWpmnsRujXVDh5XEbmijESqS2E2iS9eS6CfRxXZcSmI3j7pxccqqXmrOHKd98803iz4EZaY+fPhw9+CTuqwCVut+ETMvtZql1vatfUPAEMgnAqk5848++sghxkNPl+n4o29val0Hq+n+9z2iS9ON2QZsCBgCyRFIzZnrA1B96MlQ9GGo1sGrBtWijSRxvij9YZOuILkkdvOoG4RBVF5a46V\/adrGfhxKq89p2Y2DUZZ1UnPmhFV4AMpDTxz6yJEj3cNQeNRlGbRa9q3dvfrf5jbpuuiRZW7nxVras7YNAUOgMRBIzZkDHy8O8aYmLwxB5OFRl3VK8sCn3NjYx4XX+\/se0eKc+uf+6VduW130ktjNoy5jjktpjZf+pmkb+3EorT6nZTcORuE66dfU3ZnzslDYa\/wsUaQOmWpCww6MtA3F3WK3mv2J0haOnN0WcerM1tlzHafO\/umlVrtEadtkDIEwBLi3CPGxJz\/3mhL8MB3jZwOBujtzZt7MwJmJ8\/YnD0DJQ5RZ3YJMteDhi6EWx8YlifNVMjacus7Ucersm45Th\/ig8cGL2l6SPqelG3VsQXJp9jlN20FYBPFw0BCOm8kCYT3uq795dJlwb8GH9J6jPqgdeGmNN4ld+t1IVHdnruDhZHl1HweuPBw7POqUlzRlS1y2xmWLXF6DP\/TQQwVe0naT\/DSMY1uduu6wiGPng8YH74i\/f0X4EPJh40MIUYejh\/jAYjNJn9PSpd9xKc0+p2kbvLjmfyL33IX7AcJpQ9wr3DMQ9wvLYbnH+DXIHvz8IlTSe452kSctprTGm8Ru8RjyXk7NmeuKFR5+Koi1Ws3CfuVqD1vsdU6qxCbza9eudTsg8k1fD\/r4449l586dsWwefbDITef2kiXjBjn66YgT5dKBrQL\/5X\/f1DGr2nPiEY4e4gOIw1eiHEZ8yLNEw3\/4pox8ZpNcOmOFZKlfWehL0DXsPeFXcsb318qQKcvcFzwy3AM48Htf+K0sXPUH+c2Gbe5emXJJm3Dv6H009dJ+7j76fJ+Wfe5L7q05f3uSDB90lLsO9fiM+DY+\/fRT+eyzz+STTz7Zp2++XKX5OJ9DfAUH3kjG\/qXmzFmxwsoVVrMQy4Z4tR8edfXEiTDMk08+KRs3bqwLbdq0SbZs2SKcjpTU5n\/6eKsc33WXXH1qi9zx5VZ5YlhPef3GNkfzrunjyvAmnXeUQCO\/0CpD+7fsQ6f1OECgIw7cLXGp0g9SFPkPP\/xA+MDt3r27qh\/iKLbrJRMXb64X5F\/P2750qNx0xn4y8St77gWuPfcB9wQpBI975SvHirt3KrkHr\/zLA9x1GPXjFXX5rGjf+LXOfcA7KMpLmsb9HOIrbrnllnq6qEi2UnPm9I7Y+MKFC4V4OUQeHnXVJL5FuQm0zeJ9Xzh6itNsevXqJVGptbXVORlOPomqo3KEfLp37y49e\/aMbE91K7F7xoBj5Ruf7+do5FdPkG+d2Ue+\/bku8g9D+8mkS07qRDO+NVBK0bTL+wuO4uHhJ4TKcYpSEDH7+\/6F3YSZnV8fJf+za09xX0hxdJPYTaLLuCrRL8Y9Cta+jn8t\/+uXj5f\/ckYP+c6X9lx3rj\/3gd4\/pdLWiPc0tue9+YG89WFL4f6NqhtkP4rukUceKQcddJAQJvXbiKLry\/v5uJ9DfAWnjKk\/yUpad2fONyz7lZcDIIpMuTao561SzgZlFsyhDjt27BB41CkRhmlra3Pb2ba0tERKu3Tp4m6uqPK+nOlGwxjM8ohVXvsdFevjjj5Exl\/QJuOeW1v4rETVBZtiiqq7\/\/77C+TrR9X1dTQfVxdfYcfGdXhPZt6cJHTcccfJ448\/3sHp\/B8edch0rolXwh7fopz3B5GHF6+1vVrcEDxQJd3LrX0Oe3Ht5lE3CaJpjZc+p2kb+3Gokj6Pv6CfM8HDUzKV6CLvU1q6fh8aIb9fGoPg6DbCKtjGcfsEjzpkyFeDaIs2IfJl2zQBQ8AQKIvA9G+e5B62s2KqrLAJ1ByBVJy5joptbnGwPsHT+iynPBwjbENaz35iL67dPOomwTat8dLnNG1jPw5V2uez+7e6cMuYn7zpHorm7b6Mg1GWdVJ15lkGplzfkvw0LNd2qfokdvOoWwqLcnVpjZd+pWkb+3EoTp8Jt7QP7iksIc1b+C8ORlXUqXpT5syrDqk1aAg0FwI4dF44Yk27hVzSu\/bmzNPD3iwbAg2DAG+O4tR5QcmcejqXNVVnzjJFNtbiAehzzz0no0aNkhUrVqSDRIVWK40vVth8qHgSu3nUDQUiQkVa46VradrGfhzSPpNWqo\/OkKN2ujeSCbtU4tTRTSPeXukYsy6fmjPnaLg8HxsXJ75YjZshid086ibBLK3x0uc0bWM\/DlWjz3\/Rm4ei\/YR9Xc46vrWwhXOp8Es17NJGnDE3kk5qzlzfyGRzLQVUX+bROuVbaggYAvlCgBg6oRecus7U2c+Gzb\/yNZL89DY1Z64bX9Vjo61aXI4kPw2T9CeJ3TzqdsKqwkJa46WbadrGfhyqRZ9x6uMv2DNTp0\/E0\/VFI8pQLezSbrNRas6czbTYVIuNtnDodmxcs916Nt5mQgCnzp41bK97\/0tr3c6LzTT+eow1NWfO4HDigwcPliVLlsixxx4rb7\/9dlX2GqftWhMxurjrapP0LYndPOrmESv6nBbW2I5L9egzLxoReqGPhF1I62EXO41OqTlzHoAuW7ZM2IFMnfrcuXNlzpw5Ql0U4NnHZdy4cQVRVsKwpwurY1glw2oZKmt\/bBxWjAwBQyAKAjpLJy0OuUTRN5lgBFJz5jzkZAdDtqHksAi2pSUPj7rg7u7l4sT1MAu4fAFMnjxZRo8e7bbUZcZPPQ6d\/cr5ooBeeOEFgYdOEkoS50vLbpI+p6WbR6zos+EFCqWJB6Q8ECWObksTS2MVpTY1Z84DUPYm5gi3VatWyVlnneUOa4BHXanOMyMfOnSo+CthiMH37t27kxpfELRfi2PjOhmygiFgCMRCAId+bLcWd5pUrAaaSKncUFNz5jjf4cOHCw8+BwwYIMzKJ0yYIGPGjBHqSnWczbiCdj+cMmWKMMsnzIKzR4522K\/c\/4JABj50\/vnny4IFC2TmzJmBxAwL8uspo8tG97NmzQrUU3lkIS2TUubUFI6No1yKkIVUBnvYxb7P1\/riFBkIPrqvvPIKqm5jJOVTF0bIQKpLSlkpTE\/5yGEQu+jCh6dEuRQhB1Y\/+tGPCjjDUyqliz3FypdTXVKf7+fRpc\/0vViOspKv4+eT6KsuaZAd5ZH6Nsmj8+KLL4rihQxEnRJlJeWRosuYSSlDQXLKI0UGQqdYFz4yEHklykrwLu2yTDj159vTXixcY\/iQyvGLmmdqv\/71rwv3LnVhdtGFkIHIK1FW8rHSej9VOZ9nx8bxqSgiHDI7JuKE2WN83rx5MnDgwCKpaEVCJ8TJceK0OX\/+fCEUE0V7\/fr18vvf\/77TTaIXUY+m0jKp8kgplyJkIF+GG1ePjfP5QXl0Ib+OspLPD8oHySmPNEjH5yEDBfGK+b6M5pGBtExKWYlyKVKs\/GPjVJe0lC51yEDklSgrKS8sDZJTHmmYnvKRgbRMSlmJcikKklMeabHuhx9+6E7AUryQgXw5yko+PygfJKc80iAdn4cMFMTz+Ve1bZVfvNsqyzd3Ph4QGWjbtm3uDNADDzyw03F1frtBeXQhv44yxL3FREGx8mU0jxykZVI7Ni7Aq+JsmUX7dO655wqOWcWJhV955ZWCDA83ecipdX7KSUKcKKQvHuHUOVOUEEupY+OYlX\/961+X73znOzJixIh9iJkd5NdRbm1tdR8a+ubXFeeRhXw+v0L02DifH5RHF9K69vZ2Z7dbt27uyC7lh6XoQtSj+9WvflVUV\/nUhREykOoyXspKYXrKRw6ssIsufHhKlEuRYqW6yKouKeUwos98WBmvL4Oeks\/38+jSZ3SR9esoK\/l8P59EX3UZc5Ad5ZH6NslfddVV8td\/\/deCLmVkIPJKlJWUR+rbpQwFySmPFJkRHZ+bIF3qkIHIK1FWgofuxV\/7K7n\/omPlx1tO6PQZVLnLLrvMrXg7+eST3X2vfHS5Tjpe2vNJ5YJ43Fs+Vr6M5oP0WbTBITe+\/8lCPrUwCw6bWDkPJW+99VZ54oknhJSLwyxdwSHkwk8pZtvLly8PnblzYXDcxMjRJZRCeOX0008XnDzOnocsPGBVh48chBxHQbFEKirFPXKK9k3Xjo3jPgijZr0\/vnVmH2ELgLHPrikcR+djxJFxkM9LAyt8hR0bh+f0iIedOGEeVBIW4YHmokWLOs3MPfGSWb4Apk+fLmPHjnWzeF5G4qUkgOdblCPjIPLIlmwsQiU3lK0zjwBUh0izYdUxZOeM4t4fzYaXP17eFp29dJOU2ssFfJV8XeU1a5razJwHkrr6BIdOuGPIkCFCSCTqxSDWDqk88XZomc6LAAAN4UlEQVRm78ziFy5cKOq0NTYPn7zKW9roCNj48oYALxW1D+4ptv688iuXmjMnfHL55Ze75Yg44alTpwozdWbX6oQrH079NHgQQtiGtH5WxT2kjWuXvuZNNwm2aY2XPqdpG\/txKK0+F9tlueKrb22LNDsv1o0z7kbRSc2ZAyAzcRw5eWbMzKq1DM\/IEDAEmhMBZuecLdqco4836lSdOS\/\/sErFp+LVLPGGVXuthLG62B1MYjePurGB6lBMa7wdpi1mDggRKeg6MTtf996usrPzIN2IZhtOLDVnzmoWHlKyioVYtpIf6244tG1AhoAhEBkBZucWO48Ml6TmzOkiSwKLlwnCzwOlFatLYjePuknuhbTGS5\/TtI39OJRWn8Pstg\/uJeVi52G6ccafd519nHm9BsRDznPOOUfYMbFeNs2OIWAI5AcBVraw7nz20o356XSKPa27Mye8QlycODm7GkLklahDJkVMIplOK1aXxG4edSNdjBChtMZLd9K0jf04lFafS9m98gu9hHXnYeMppRum06j8ujtzZuTExTVGXpxSh0yjAm7jMgQMgegItA\/u6YRLOXQnYH\/SiZmzvwr7rDAbZ08F9l\/J27VIK1aXxG4edZPcF2mNlz6naRv7cSitPpezi0MPeyO0nG4cHPKqU\/eZOY6bQyR4SYhZOW+BPvXUU3nFz\/ptCBgCNUbg7P7d5H+u+3\/y3if719hSvpuvuzPXU4R0FQs7kLHFZCUw+sfA+TN7vigoM+Nn5s8vANr15VnbDi8ppRWrS2I3j7pJrlOM8RbMJdGlkST6SXSxHZeS2K2lLjNzxvTbHQeRdKIkdjs11ACFujvzIMzYVxhHHFRXzOPhqB4Dpzsk6sx+0qRJwgoZZvzM\/Nl32Jdnh0Y2uYdX3G49y4yXwwNI62k3j7bAyLCKfuUaFS8c+m93HBgdiAiSjYZVJpx5BNwLIjwcLT7Egl0XcdAbNmwoHCXH9gBswoXDZ2tcNvNiFzv2f4FXaLAj89prr8mSJUsqosWLF8tdd90lTz\/9dEV62Hn99deFLxq+XChXQkns5lE3j1hxPdPCOo94RcGqreVDee3\/HuxOEgNfpSi6KlucJsGqkg0BO1xMXf6n4swBgn1ZCIewLS07JhJ2oRx1aSKhE3RAibZIoZtvvtltgeuHWXg5iV0aqYfY65y0T58+wr7EzPQJz1RCV199tXPIt912mzsIoBLdW265BfNSiV1tP4ndPOrmESuuVVpY5xGvKFjNvmek+8z8t\/\/1VqfPWxRdrkcQJcEKXXwHPsR1LAN\/6u7MmVmz\/JBQSBBRh0w5bJh5o09YhXNEOZdv5cqV7pQS+KNHjxYetHLSTFhbXIgHH3xQOPzCaJbhMMswyOrn4NFHH5WfX9FFpt98WWbuU3wHPkQy8q\/uzrzScRNL51uVWbs\/29Z2CLEw0+\/atav07dvXHQxNnfJx5tTrg1etI4W4GMzsjYaIYWAY2D0Q\/R7Ad+BDskLJnXmNR8K+58wWmG2zRS6xb5w7MXJMc0LR4MGDhbMBBwwYIK+++ipsF1sjvBLl2DinYH8MAUPAEMgxApl35sXYEoLhsFVmEMzWeejJKhbkOEOUnRjhk06bNk1qdWwc9qpNv\/jFL+SXv\/yla3bp0qVy\/fXXC4d1fPrpp45nfzoj8Mc\/\/tH95J4wYYLcfffdsn79+s4CVnII8OuWzwLhyDfffNPx7E84Alu3bpX7779fuK9+8IMfCL\/uw6WzU5M7Zw50Gi9nts6sndk7fBw9MXf4pJTh+\/Lk4WWJdu\/eLTNnzpRHHnlECAe999578uKLLwrLK3mpirFkqb9Z6cuHH34ovKNw++23y5133ilZ+9mbFZxYjjto0CD57ne\/K3PmzBGce1b6FrEfdRVbt26d\/Pmf\/7ncd999cu2118pBBx1UV\/txjeXSmccdbL30eDFp3LhxBXN8eAgN8YshbLXOJZdcIjfeeKPTIcbPM4AuXbrICSec4I7WcxUN\/ocVSuAEXgyVlHIYbnzp8QtmzJgx7oOHPHrNQoQaL7roItGX4xg39x54QeAJ76233hImBUcccYRwXzFhgN9sFBUvjlZ8\/vnn5aqrrhKWQfMLMA9YmTOv8lXCibMTpN8sLzXxYeIXA\/F9tv1lRsmM6ZVXXhHCKHzQfJ2jjjrKLzZ8HidEGMAfaBBuO3fuFDADO\/L33nuv\/PM\/\/7OceOKJju\/rN3IeB37++ecLs0gdJzxwYT01h77wa48vuAMPPNA5cZVrxhRsouLFQgvCm9xXrEXPS\/jOnHkV72wc0tChQwsvLtE0H6ZFixYJq2soU0+ZDxiO\/ZRTThHy1CkdeeSRwttpzAjef\/99OeCAA7SqIVOdQfLMQwcYhhtffGAGdsww1ZnxK4aXw1S\/XmkadphhPvnkkzJjxgw5\/PDDC13g4T8vxfFOBe9g7NixQ5hldu\/eXbZs2eJiv8R\/G\/1+KgDyp0yleLHQgnsLnPbbbz9hM68\/NZXpxJx5FS\/PddddJ2ExeXXmmCOM8tlnnwkxfT5o+++\/P+wC9erVy82k\/vEf\/1F+9rOfyZe+9KVCXSNmwAzsgsZWjBsfLDADOx5uP\/fcc+5h1c9\/\/nNh5hXURqPxGDtvNxMyKR4bvwD1GRJfbps3b3ZbXPAgj4fELN8t\/hVY3EajlSvFi1VwPF+444473Jdlv379cgGJOfMMXSacGvRnf\/ZnQhyYD9\/3vvc9F+\/MUDcz05Vu3bq5t2jHjx8v3\/\/+992XY2Y6l6GO4IwIGxCSIg6coa5lsiv86lO8brrpptz8MjZnXqfbSbcQwBzf\/PwUJl8zapCGDbfKLiRLdQlRoXXYYYcVXqKjbLQvAo2Elznzfa9vVTn85GXLAXVKvOREGX5VDTVYY+ADToZb9At71llnCXFy4r1sJkf8nFl59BaaS7LR8DJnXof7l5+2zABYLkZKuQ5mc28CnMDLcIt2KQcOHCgXXnih25Zh7NixwiZwfClG024+qUbDKwVn3vg3DQ+nIB0pHyhebmJpIillrbN0LwI8BPXxASfKhttejPwczoiVUaTKB0PwYkWGz9f6Zk7Bo5HxMmfezHe3jd0QMAQaBgFz5g1zKW0ghoAhoAg0Y2rOvBmvuo3ZEDAEGg4Bc+YNd0ltQIaAIdCMCJgzb8arbmNODwGzbAjUCAFz5jUC1po1BAwBQ6CeCJgzryfaZssQMAQMgRohYM68RsDmr1nrsSFgCOQZAXPmeb561ndDwBAwBP6EgDnzPwFhiSFgCBgCeUYgD848z\/ha3ytAgN3+9Jg49mNR4vSmCprJpCgn3bA\/vY6Rg0y0o8rTQzqUryn1YMAhC8qz1BAoRsCceTEiVk4dAU4cYn8RJX+fm9Q7F6MDOGNOBrrhhhtiaIuwR80111wjjzzySCx9U2oOBMyZN8d1zv0omckya+dAbPLMdDmrkdk7PJ21MruFR92oUaNEZ7TIUAcQ6MPXPPKQ8pDjoGQIPnZxyMgjAw9CDtJ6ZMjDQ1aJMzk5MYkTb5QXlvrtY4P2aJdNotBh3KRGVUagAZozZ94AF7HRhvDAAw8IjgzCCauj5ri9OXPmyLBhw2TChAny1FNPCbP39vZ2QQe5e+65R5544gnhPEz29i6FDU6XXfTY+xtiu1146Kxbt07uu+8+wRFjl5S6pUuXOt7cuXPdKUcc8Yc8e4hz3iY2OX8TnhJ72OPMtUxKfxkfhDztw+dXCGOCGOeIESPczJw6bDEu8kaGQDEC5syLEbFy6gj4YZaFCxcWjoPTE5o41xJne\/HFFzunj2PEyS5evNj1HedIaILDLRwj5A8HX+BEkYfI43gRb21tdaf0cCIUduEhz5FizLCZKc+bN09OPvlkd6wfXwb0a8CAAYX+osOsGgffo0cPigXyx4jukCFDCnVkmKEPHTq005myfCHQB+qNDIFiBMyZFyNi5VwgwMHEOF9msBBOn2PSKu08s1\/0lZgZV9oGTpcvAYh8pfrF8oSBcNycBytSXGtlQyAYAXPmwbgYN8MIMMvdvn27PPPMM66XOD9iyzhAGMx0mRETQqGss2tmtT4f+QULFghxaPi0QVvoBBHy\/AIgnIMOM39SnVWvWrVKmOH7uvxC4Pg2Zu0+Pyyv9jlkoliG\/tOHYr6VDQEQMGcOCka5QoAwx\/Tp0+Wxxx5zYZbZs2fLtGnTpK2tTSZOnCgjR44UznfEiTIwHCqOl3CMz2fmO3r0aCFcgxPu3bu3BDlR2oCQJz6P80YH50+4hfZxssUhFnQgZuvM2smXIr6gGAv9JJYO6TMDvmyWLVvmxlWqDatrXgTMmTfvtU868qrr4xQ5Ji7IocKjDhkM40Q5Go3wCCEWHDx8HC486gYNGgTLEfrKnzFjhmg4RfnUKY82tE3sYRceDfny5OHhaPkVgNOmXEw4f3jM6LU91YWvPEI+2KUvSpQZGw9Xe\/bsKYwbHSNDoBgBc+bFiFjZEKgAARz0hRde6DTUabuC9wdnjaNnxu2xI2f5suCXyOWXXx5ZxwSbDwFz5s13zZtmxMx+dbZdq0Eza2b2zOwdpx1mh5l93L7QLr8mbFYehq7xQaAhnTkDMzIEDAFDoJkQMGfeTFfbxmoIGAINi4A584a9tDYwQ8AQqB0C2WvZnHn2ron1yBAwBAyBihEwZ14xZKZgCBgChkD2EDBnnr1rYj0yBJIgYLpNioA58ya98DZsQ8AQaCwEzJk31vW00RgChkCTImDOvEkvfBrDNpuGgCFQOwT+PwAAAP\/\/UtwE0QAAAAZJREFUAwDNYqH\/4wgg3gAAAABJRU5ErkJggg==","height":224,"width":371}}
%---
