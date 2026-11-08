%[text] # Vienna rectifier (vienna\_rectifier)
%[text] Initialization of vienna\_rectifier.slx. The model is a 250 kW three-level Vienna rectifier on the 690 V grid: the grid emulator (three controlled voltage sources driven by the library block three\_phase\_voltage\_source\_with\_frt\_emulator and a Dyn11 coupling transformer made of three single-phase transformers, grid/grid\_emulator/3ph\_Dyg) feeds the LCL filter (500 uH on the converter side, 200 uF in star with 50 mOhm), the Vienna bridge of the library with ideal switches and diodes (vienna\_rectifier\_ideal\_switch\_based\_model) and a split DC link of two 14.4 mF capacitors (dclink\_for\_three\_levels). The load is a 5 Ohm resistor across the DC link: 229 kW at 1070 V.
%[text] Each leg of the Vienna bridge has a diode to the positive rail, a diode from the negative rail and a bidirectional switch to the neutral point (two IGBTs in common emitter). A leg can only put its terminal at the rail that has the sign of its current, or at the neutral point: the bridge takes power from the grid and never gives it back, and it cannot follow the voltage reference where the reference and the current have opposite signs (a few degrees around each zero crossing of the current).
%[text] The control (afe\_dq\_mod1/AFE\_control/afe\_control, triggered by the modulator TRGO every ts = 125 us) is the dq control of the two-level and T-type AFE models: acquisitions quantized to 12 bit and one step late, DDSRF-PLL on the filter capacitor voltage, PI of the DC voltage that gives the active current reference, vector PI of the converter current in the xi-eta frame, space vector modulation with common mode (C-Caller sv\_pwm\_cm), three-level PWM of the library with 3 us dead time. The Vienna leg reads the gate commands Q2 and Q3 of the T-type pattern.
%[text] All the data are in one struct param, one group per part of the model: sim, time, glb\_time, grid, frt, afe, load, meas, ctrl. The model reads param.\* only; the README lists the old names and what was changed in the model. The functions of ./private are called through the handles of vienna\_rectifier\_private.m (prv.timing\_param, prv.grid\_param, ...): the Live Editor runs a live script from a copy outside this folder and does not see ./private. The legacy init (the shared template of the AFE and inverter models) is kept as init\_model\_legacy.m for reference.
clear;
model = 'vienna_rectifier';
prv = vienna_rectifier_private();   % handles to the functions of ./private (the Live Editor does not see ./private)
prv.library_setup();                % pwr-control/library on the path (setup classes, blocks, C-Caller sources), cloned when missing

% environment (was init_environment of the library)
close all; clc; beep off;
try
    pm_addunit('percent', 0.01, '1');       % custom Simscape unit used by some library blocks
catch ME
    if ~contains(ME.message, 'percent')     % already registered in this session: fine
        rethrow(ME);
    end
end
options = bodeoptions;
options.FreqUnits = 'Hz';
param = struct();
fprintf('%s init: functions of ./private, called as prv.<name>: %s\n', model, strjoin(fieldnames(prv)', ', '));
%%
%[text] ## Simulation
param.sim.simlength = 1.25;                 % [s] model StopTime
param.sim.nonlinear_iterations = 5;         % Simscape Solver Configuration of the grid emulator (raise to 10 if the bridge does not converge)
param.sim.check_model = true;               % last section: param.* references of the model and From blocks without a Goto
param.sim.design_figures = true;            % section Design check: figures (false: numbers only)
%%
%[text] ## Timing
%[text] One clock domain: the modulator timer at 4 kHz with double update gives the control trigger (TRGO) every ts = 125 us; the step tc = ts/200 = 0.625 us is the logging base, the MaxStep of the solver, the step of the Simscape local solver and of the dead-time generators.
param.time.fpwm = 4e3;                      % [Hz] frequency of the modulator timer (block rectifier/sv_pwm/modulator1)
param.time.trgo = 1;                        % double update: control at 2*fpwm
param.time.dead_time = 3e-6;                % [s] dead time of the gate commands (rectifier/deadtime_generator_for_ttype)
param.time.t_measure = param.sim.simlength; % [s] logging window: the whole simulation
param.time.tc_factor = 200;                 % ts_afe/tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;                   % [s] PWM delay of the modulator model B (not used)
param.glb_time = prv.timing_param(param.time);
%%
%[text] ## Grid emulator
%[text] 1 MVA at 690 V (us1 = us2 = 690 V, 50 Hz), coupling transformer Dyn11 of three single-phase units: efficiency 95 percent, ucc 5 percent, iron losses 1.8 kW, 14 turns on the secondary and 24 on the delta primary (secondary at 697 V at no load), magnetizing inductance from the core geometry (0.05 m2, 2.5 m, mur 10e3) and the line no-load current from it. The class grid\_three\_phase\_emulator of the library (param.grid.emu) gives the transformer data read by the blocks ph1, ph2, ph3 and the references of the voltage source. The grid voltage rises from 0 with a 20 ms ramp and a 50 Hz second order filter at the start of the simulation.
param.grid.grid_nominal_power = 1000e3;     % [W]
param.grid.application_voltage = 690;       % [V] 690, 480 or 400: selects the transformer data set in grid_param
param.grid.delta_star = 1;                  % 1: Dyn11, 0: Yyn
param.grid.n2 = 14;                         % turns of the secondary winding (primary: floor(n2*sqrt(3)))
param.grid.core_area = 0.05;                % [m^2]
param.grid.core_length = 2.5;               % [m]
param.grid.mur = 10e3;                      % relative permeability of the core
param.grid.up_xi_pu_ref = 1;                % [pu] voltage sequence references of the emulator
param.grid.up_eta_pu_ref = 0;
param.grid.un_xi_pu_ref = 0;
param.grid.un_eta_pu_ref = 0;
param.grid.ucc_factor = 1;                  % grid inductance = ucc_factor * short-circuit inductance (not read by the model)
param.grid.eq_grid_resistance = 2e-3;       % [Ohm] (not read by the model)
param.grid = prv.grid_param(param.grid);
%%
%[text] ## FRT test (not applied)
%[text] Fault data of the grid emulator (library frt\_settings and grid\_fault\_generator, merged in param.frt.data). Both fault generators are off, as in the legacy init: the data of test 25.4 of VDE-AR-N 4110 are computed but not applied, and the Vienna control has no fault ride-through logic. The emulator reads t\_ramp for the start of the grid voltage.
param.frt.test_index = 25;                  % VDE-AR-N 4110 test: index ...
param.frt.test_subindex = 4;                % ... and subindex
param.frt.asymmetric_error_type = 1;
param.frt.enable_frt_1 = 0;                 % fault from the abc amplitudes and phase shift: off
param.frt.enable_frt_2 = 0;                 % fault from the sequence dips below: off
param.frt.start_time_LVRT = 0.75;           % [s]
param.frt.deepPOSxi = 1;                    % [pu] sequence dips of the second generator
param.frt.deepPOSeta = -0.4;
param.frt.deepNEGxi = 0.4;
param.frt.deepNEGeta = 0.4;
param.frt.data = prv.frt_param(param.frt);
%%
%[text] ## Vienna rectifier
%[text] ### Hardware
%[text] 250 kW data set of the library for the 690 V class (afe690V: udc 1070 V, LCL filter 500 uH plus 500 uH common mode, 200 uF with 50 mOhm, DC link 2 x 14.4 mF). The class hw\_afe\_three\_phase\_setup gives the normalization ubez (563.4 V) and ibez (328.7 A) of the control.
param.afe.pwr_nom = 250e3;                  % [W]
[param.afe.hw, param.afe.hw_in] = prv.afe_hw_param(param.grid.application_voltage, param.afe.pwr_nom, param.glb_time.fPWM_AFE);
%%
%[text] ### Switches and diodes of the bridge
%[text] Data set silicon\_high\_power\_ideal\_switch of the library (the numbers the legacy model had typed in the mask of the bridge: Vth 5.5 V, Ron 1.5 mOhm, diode 2 V and 0.15 mOhm, snubber 10 nF and 2.2 kOhm).
param.afe.dev = device_ideal_switch_setting('silicon_high_power_ideal_switch', param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
%%
%[text] ### DC link and load
param.afe.dclink.udc_init = param.afe.hw.udc_nom;   % [V] initial DC-link voltage, udc_init/2 on each half (legacy: udc_nom)
param.afe.dclink.Rudc_measure = 107e3;      % [Ohm] measuring divider of the block dclink
param.afe.dclink.R_np_gnd = 1e5;            % [Ohm] neutral point of the DC link to ground (afe_dq_mod1/Resistor)
param.load.R = 5;                           % [Ohm] load resistor across the DC link (block load): 229 kW at 1070 V
%%
%[text] ### Control (class ctrl\_afe\_setup)
%[text] The legacy gains: udc PI 4 and 45 (output limited to 1.65 pu), current PI 0.25 and 35 (the model read them from the PMSM control of the shared template, psm\_ctrl.kp\_i and ki\_i: now param.afe.ctrl.kp\_is\_xi, ki\_is\_xi for the xi axis and kp\_is\_eta, ki\_is\_eta for the eta axis), DDSRF-PLL 0.32 and 8 as in frt\_dqpll after the fix of the DDSRF-PLL (natural frequency 8 Hz, damping 1.0).
param.afe.ctrl = ctrl_afe_setup(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom);
param.afe.ctrl.kp_udc_ctrl = 4;
param.afe.ctrl.ki_udc_ctrl = 45;
param.afe.ctrl.kp_is_xi = 0.25;
param.afe.ctrl.ki_is_xi = 35;
param.afe.ctrl.kp_is_eta = 0.25;
param.afe.ctrl.ki_is_eta = 35;
param.afe.ctrl.pll_p_ddsrt = 0.32;
param.afe.ctrl.pll_i1_ddsrt = 8;
param.ctrl.clip_release = 1e-3;             % [1/s] decay of the current PI integrators while the voltage vector is clipped
%[text] DDSRF-PLL (grid\_voltage\_phase\_lock\_loops/DDSRF-PLL): the loop is now fed with the decoupled positive sequence (it read the negative sequence through a gain -1), its frequency integrator is clamped to 0.5 ... 1.5 pu as in frt\_dqpll, and it starts from the nominal frequency: from 0 the clamp holds it at 0.5 pu and the PLL locks after 110 ms instead of 45 ms (switched model of the design check).
param.afe.pll.integrator_init = 1;          % [pu] initial value of the frequency integrator (Unit Delay4 of pll_stageB)
%%
%[text] ### Clocks, modulator and behavioural settings
%[text] Local clock time = k t of the AFE (global timing/time\_afe1) and the PI that aligns it to the master clock (acquisitions/phase\_compensation\_1). The zero-sequence current controller is computed but its output is multiplied by u\_cm\_comp\_mod1 = 0 (single module). The power limitation of the udc controller has no source in this model (it was a From block without Goto): a Constant 0 now.
param.afe.behav.time_gain_module_1 = 1.0002;    % clock gain of the AFE (global timing/time_afe1)
param.afe.behav.pwm_phase_shift_mod1 = 0;       % [rad] phase shift of the modulator carrier (rectifier/sv_pwm)
param.afe.behav.u_cm_comp_mod1 = 0;             % gain of the zero-sequence controller output (space_vector/Gain3)
param.afe.behav.pwr_lim_enable = 0;             % udc_ctrl/Switch: 1 sets the active current reference to 0
param.ctrl.align.kp = 0.25;                     % PI of the clock alignment (phase_compensation_1/alignment_eval_afe1/pi)
param.ctrl.align.ki = 18;
param.ctrl.align.lim_up = 0.05;
param.ctrl.align.lim_down = -0.05;
%%
%[text] ## Sensors: full scale and quantization
param.meas.adc_quantization = 1/2^11;       % [pu] quantizer of the acquisitions (12 bit)
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;
param.meas.Imax_adc = 1049.835;             % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;
param.meas.Umax_adc = 1500;                 % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%%
%[text] ## Filters
%[text] Second order low pass at 50 Hz of the grid emulator (block three\_phase\_voltage\_source\_with\_frt\_emulator reads the matrices A, B, C of the continuous filter, class sof\_setup of the library; the legacy init built the whole filter bank of setup\_global\_filters).
param.ctrl.filters.afe.SOF_50Hz = sof_setup(50, param.glb_time.ts_afe);
%%
%[text] ## Design check
%[text] Operating point, voltage margin of the modulation, angle between the converter voltage and current (the zero-crossing window of the Vienna bridge), LCL resonance against fs/6, converter-current loop (closed-loop eigenvalues and margins with the one-step delay), DC-voltage loop, neutral-point voltage ripple, DDSRF-PLL and start of the udc reference. Numbers in param.afe.design; the switched simulation of the same model in Python is in the design note (repos\_doc/vienna\_rectifier of pwr-control/repositories-documentation).
param.afe.design = prv.vienna_design(param, options);
%%
%[text] ## Model setup
open_system(model);
%[text] Output types (buses) of the C-Caller blocks: the three first harmonic trackers of the DDSRF-PLL and the space vector modulator sv\_pwm\_cm. The C sources are in the library (user\_defined\_functions/ccaller), listed in the configuration of the model.
Simulink.importExternalCTypes(model, 'Names', {'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'sv_pwm_cm_output_t'});
%[text] Scopes closed at start.
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### CREATE\_POWER\_METER\_BUS
%[text] Simulink.Bus PowerMeterIecBus for the output m of the MATLAB Function block three\_phase\_power\_meter\_iec (grid/power meter iec mfunc; Symbols pane: Type = Bus: PowerMeterIecBus). Bus objects stay in the base workspace.
prv.power_meter_bus();
%%
%[text] ## Check of the model
%[text] Every param.\* read by an active block must exist, and every From block of the active part of the model must have a Goto.
if param.sim.check_model
    [missing_refs, model_refs] = prv.check_model_param_refs(model, param);
    if isempty(missing_refs)
        fprintf('%s init: all %d param.* references of the model blocks resolve\n', model, numel(model_refs));
    else
        warning('%s init: %d param.* references of the model blocks do not exist in param:', model, numel(missing_refs));
        disp(missing_refs);
    end
    lost = prv.check_from_goto(model);
    if isempty(lost)
        fprintf('%s init: every active From block reaches its Goto\n', model);
    else
        warning('%s init: %d tags read by active From blocks do not reach an active Goto:\n  %s', model, ...
            numel(lost), strjoin(lost, '\n  '));
    end
    clear missing_refs model_refs lost
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":33.8}
%---
