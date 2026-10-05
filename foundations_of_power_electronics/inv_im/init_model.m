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
param.afe.hw.displayInfo();
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
param.inv.hw.displayInfo();
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
param.im.machine.displayInfo();
clear m
%[text] ### Motor voltage to Udc scaling
param.im.u_im_scale = 2/3*param.inv.hw.udc_nom/param.im.machine.ubez;
param.im.u_im_scale_ekf = (2/3)^2 * param.inv.hw.udc_nom/param.im.machine.ubez;
%[text] ### Control modes
%[text] Flags marked (variant) drive Variant Source conditions in the model.
param.im.mode.use_torque_curve = 1;                                         % (variant) energy production
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
param.im.ctrl.ekf = ekf_im_setup(param.im.machine.alpha_norm, param.im.machine.beta_norm, param.im.machine.gamma_norm, ...
    param.im.machine.sigma_norm, param.im.machine.mu_norm, param.im.machine.Lm_norm, param.im.machine.Jm_norm, param.glb_time.ts_inv);
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
param.psm.machine.displayInfo();
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
param.battery.pack_1 = lithium_ion_battery_setup(b.nominal_battery_voltage_1, b.nominal_battery_power, ...
    b.charge_capacity_1, b.initial_battery_soc, param.glb_time.ts_dab);
param.battery.pack_2 = lithium_ion_battery_setup(b.nominal_battery_voltage_2, b.nominal_battery_power, ...
    b.charge_capacity_2, b.initial_battery_soc, param.glb_time.ts_dab);
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
figure; bode(param.dclink.sys_dclink, param.dclink.Z_Lstray_dclink, options); grid on
%[text] ## Template extras (not used by inv\_im.slx)
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
