%[text] # Three-phase grid PLLs under grid faults (three\_phase\_dqPLLs\_analysis, model frt\_dqpll)
%[text] Initialization of frt\_dqpll.slx. The model is a PLL test bench: a 1 MVA, 690 V grid emulator (three controlled voltage sources driven by the fault generator of the library, three\_phase\_voltage\_source\_with\_frt\_emulator, and a Dyn11 coupling transformer, library block configurable\_trafo) feeds the LCL filter of a 250 kW active front end whose converter side is left open: there is no converter and no DC link, the grid voltages and currents are only measured, normalized and acquired by the AFE control (12-bit quantizer and one step delay).
%[text] The AFE control (AFE\_control1/afe\_control, triggered by the modulator TRGO every ts = 125 us) holds four phase locked loops on the same grid voltage, compared side by side: the dq-PLL in Simulink (synchronous reference frame, proportional gain scheduled during the fault), the dq-PLL of the C-Caller dqpll\_grid (fed by three per-phase first harmonic trackers), the DDSRF-PLL (decoupled double synchronous reference frame on the per-phase first harmonic trackers) and the FHT-PLL (first harmonic trackers on alpha and beta, positive sequence from their quadrature outputs); every loop ends in a double-integrator observer for the frequency. One of the four drives the rest of the control (param.afe.pll.use\_dq\_pll\_mode1..4): the grid voltage sequences, the FRT detection with the reactive current references of VDE-AR-N 4110, the grid current sequences with the reactive power PIs and the islanding detector are the library blocks afe\_uvw\_ctrl, parameter struct param.afe.uvw. The space vector modulator runs with zero references and only gives the control trigger.
%[text] FRT test: VDE-AR-N 4110 test 25.4 in its asymmetric variant 1 (phase U to 0.25 pu, phases V and W to 0.875 pu with a shift of 21.8 degrees, built from the abc amplitudes) from 0.75 s for 915 ms; the simulation stops at 1.25 s, inside the fault.
%[text] All the data are in one struct param, one group per part of the model: sim, time, glb\_time, grid, frt, afe, inv, meas, ctrl. The model reads param.\* only; the README lists the old names. The functions of ./private are called through the handles of frt\_dqpll\_private.m (prv.timing\_param, prv.grid\_param, ...): the Live Editor runs a live script from a copy outside this folder and does not see ./private. Run the whole script once (Run): a section run on its own needs prv and param in the workspace.
%[text] The old flat init is init\_model\_legacy.m (kept until the merge). It no longer runs with the library (grid\_fault\_generator is a function now) and it does not define the param.\* names that the blocks copied from afe\_uvw\_three\_phase already read.
clear;
model = 'frt_dqpll';
prv = frt_dqpll_private();      % handles to the functions of ./private (the Live Editor does not see ./private)
prv.library_setup();            % pwr-control/library on the path (setup classes, blocks, C-Caller sources), cloned when missing

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
param.sim.simlength = 1.25;                 % [s] model StopTime: the simulation ends inside the fault (0.75 s + 915 ms)
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration of the grid emulator (no semiconductors: 3 is enough)
param.sim.check_model = true;               % last section: param.* references of the model and From blocks without a Goto
param.sim.design_figures = true;            % section PLL design check: figures of the loops (false: numbers only)
%%
%[text] ## Timing
%[text] One clock domain: the modulator timer at 4 kHz with double update gives the control trigger (TRGO) every ts = 125 us; the step tc = ts/200 = 0.625 us is the logging base, the MaxStep of the solver, the step of the Simscape local solver and the time constant of the first order filters that smooth the emulator voltages (td2tc).
param.time.fpwm = 4e3;                      % [Hz] frequency of the modulator timer (block sv_pwm/modulator1)
param.time.trgo = 1;                        % double update: control at 2*fpwm
param.time.dead_time = 3e-6;                % [s] dead time of the modulator (no bridge in this model)
param.time.t_measure = param.sim.simlength; % [s] logging window: the whole simulation
param.time.tc_factor = 200;                 % ts_afe/tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;                   % [s] PWM delay of the modulator model B (not used)
param.glb_time = prv.timing_param(param.time);
%%
%[text] ## Grid emulator
%[text] 1 MVA at 690 V (us1 = us2 = 690 V, 50 Hz), coupling transformer Dyn11 with a three-limb core: efficiency 95 percent, ucc 5 percent, iron losses 1.8 kW, 14 turns on the secondary and 24 on the delta primary, magnetizing inductance from the core geometry (0.05 m2, 2.5 m, mur 10e3) and the line no-load current from it. The class grid\_three\_phase\_emulator of the library (param.grid.emu) gives the normalization of the emulator, its voltage reference and the transformer data in SI and in pu for the block configurable\_trafo (param.grid.emu.trafo.pu). The grid inductance (ucc of the transformer times ucc\_factor, with 2 mOhm) is computed as in the legacy init but no block of this model reads it: the controlled voltage sources feed the transformer directly, so the grid impedance seen by the LCL filter is the leakage of the transformer only.
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
param.grid.ucc_factor = 1;                  % grid inductance = ucc_factor * short-circuit inductance of the transformer
param.grid.eq_grid_resistance = 2e-3;       % [Ohm]
param.grid.lm0_pu = 0.5;                    % [pu] zero-sequence magnetizing inductance of the three-limb core (lm0 = lm: three single-phase units)
param.grid = prv.grid_param(param.grid);
%%
%[text] ## FRT test
%[text] Fault of the grid emulator (library frt\_settings and grid\_fault\_generator, merged in param.frt.data): test 25.4 of VDE-AR-N 4110 (75 percent dip, full load) in its asymmetric variant 1, phase U to 0.25 pu and phases V and W to 0.875 pu with a shift of 21.8 degrees, from 0.75 s for 915 ms, built from the abc amplitudes and the phase shift (enable\_frt\_1). The sequence dips deepPOS/deepNEG are for the other fault generator (enable\_frt\_2), off here. The thresholds of the FRT detection are in param.afe.uvw.frt.
param.frt.test_index = 25;                  % VDE-AR-N 4110 test: index ...
param.frt.test_subindex = 4;                % ... and subindex (25.4: 75 percent dip, two-phase, full load)
param.frt.asymmetric_error_type = 1;        % 0: variant C (phase U stays at 1 pu, V and W dip and shift), 1: variant D (U dips to 0.25 pu too)
param.frt.enable_frt_1 = 1;                 % fault from the abc amplitudes and phase shift
param.frt.enable_frt_2 = 0;                 % fault from the positive and negative sequence dips below
param.frt.start_time_LVRT = 0.75;           % [s]
param.frt.deepPOSxi = 1;                    % [pu] sequence dips of the second generator
param.frt.deepPOSeta = -0.4;
param.frt.deepNEGxi = 0.4;
param.frt.deepNEGeta = 0.4;
param.frt.data = prv.frt_param(param.frt);
%%
%[text] ## AFE
%[text] ### Hardware
%[text] 250 kW data set of the library for the 690 V class (afe690V: udc 1070 V, LCL filter 500 uH plus 500 uH common mode, 200 uF with 50 mOhm). The model has the LCL filter and the measurements only (LCL filter/LCL\_filter), the converter side is open. The class hw\_afe\_three\_phase\_setup gives the normalization ubez and ibez of the control.
param.afe.pwr_nom = 250e3;                  % [W]
[param.afe.hw, param.afe.hw_in] = prv.afe_hw_param(param.grid.application_voltage, param.afe.pwr_nom, param.glb_time.fPWM_AFE);
%%
%[text] ### Control (class ctrl\_afe\_setup)
%[text] The PLL gains and the observers come from the class: pll\_p 1 and pll\_i1 80 (the loop works on omega in pu, so its characteristic polynomial is s^2 + omega\_n pll\_p s + omega\_n pll\_i1: natural frequency 25 Hz, damping 0.99), pll\_p\_frt 0.2 during the fault (dq-PLL only), the DDSRF and FHT loops with the same gains, tau\_ddsrf = 1/omega\_n for the decoupling low passes, double-integrator observer with poles at \[-1 -4\] omega\_n (di\_obs), first harmonic trackers with delta 0.05 and poles at \[-1 -4\] omega\_n (fht\_grid). The current and DC-link gains of the class are not used by this model.
param.afe.ctrl = ctrl_afe_setup(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom);
%%
%[text] ### PLL selection
%[text] Variant Source blocks of grid\_voltage\_phase\_lock\_loops: the phase and the frequency used by the rest of the control (tags phase\_grid\_pll, omega\_grid\_pll\_pu) come from one of the four loops; exactly one flag must be 1. The four loops run and are logged in any case.
param.afe.pll.use_dq_pll_mode1 = 0;         % dq-PLL in Simulink (dq_pll, proportional gain scheduled by frt_active)
param.afe.pll.use_dq_pll_mode2 = 0;         % dq-PLL of the C-Caller dqpll_grid (dq_pll_ccaller)
param.afe.pll.use_dq_pll_mode3 = 0;         % DDSRF-PLL
param.afe.pll.use_dq_pll_mode4 = 1;         % FHT-PLL: first harmonic trackers on alpha and beta
assert(param.afe.pll.use_dq_pll_mode1 + param.afe.pll.use_dq_pll_mode2 + param.afe.pll.use_dq_pll_mode3 + ...
    param.afe.pll.use_dq_pll_mode4 == 1, '%s init: exactly one PLL must be selected (param.afe.pll.use_dq_pll_mode1..4)', model);
%%
%[text] ### Current references
%[text] Active current reference of the module and the reactive current steps after the fault (block FRT-VDE4110/positive\_reactive\_current\_reference). With the 915 ms fault the steps fall at 2.0 s and 2.5 s, after the end of the simulation.
param.afe.ref.ixi_ref_mod1 = -0.85;         % [pu] active current reference (generator sign)
param.afe.ref.enable_i_react_pos_steps = 1;
param.afe.ref.time_i_react_pos_ref_1 = 0;   % defaults
param.afe.ref.time_i_react_pos_ref_2 = 0;
param.afe.ref.i_react_pos_ref_1 = 0;
param.afe.ref.i_react_pos_ref_2 = 0;
param.afe.ref.i_react_pos_ref_3 = 0;
if param.afe.ref.enable_i_react_pos_steps
    param.afe.ref.time_i_react_pos_ref_1 = param.frt.data.start_time_LVRT + param.frt.data.error_length + 0.335;   % [s]
    param.afe.ref.time_i_react_pos_ref_2 = param.afe.ref.time_i_react_pos_ref_1 + 0.5;                             % [s]
    param.afe.ref.i_react_pos_ref_2 = -param.afe.ref.ixi_ref_mod1*tan(acos(0.95));   % [pu] cos(phi) = 0.95
    param.afe.ref.i_react_pos_ref_3 =  param.afe.ref.ixi_ref_mod1*tan(acos(0.95));   % [pu]
end
%%
%[text] ### Clocks and modulator
%[text] Local clocks time = k t (blocks global timing/time\_afe1 and time\_inv1): the AFE clock drives the modulator and the control, the second clock (gain of inverter module 2, legacy name) is not used. The correction inputs of the two clocks (tags local\_time\_corr\_afe1 and local\_time\_corr\_inv1) have no writer in this model: no time alignment.
param.afe.behav.time_gain_module_1 = 1.0002;    % clock gain of the AFE (global timing/time_afe1)
param.afe.behav.pwm_phase_shift_mod1 = 0;       % [rad] phase shift of the modulator carrier (sv_pwm)
param.inv.behav.time_gain_module_2 = 1.001;     % clock gain of global timing/time_inv1 (output not used)
%%
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;       % [pu] quantizer of the acquisitions (12 bit)
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;
param.meas.Imax_adc = 1049.835;             % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;
param.meas.Umax_adc = 1500;                 % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%%
%[text] ## Control settings common to the modules
%[text] Moving average filters of the sequences, and the second order low pass at 50 Hz of the grid emulator (block three\_phase\_voltage\_source\_with\_frt\_emulator1 reads the matrices A, B, C of the continuous filter, class sof\_setup of the library; the legacy init built the whole filter bank of setup\_global\_filters).
param.ctrl.mavarage_filter_frequency_base_order = 2;    % 2: window of 100 Hz (half period), 1: 50 Hz
param.ctrl.dmavg_filter_enable_time = 0.025;            % [s] the averages are zero before this time
param.ctrl.filters.afe.SOF_50Hz = sof_setup(50, param.glb_time.ts_afe);
%%
%[text] ## AFE control in uvw: parameters of the library blocks (afe\_uvw\_ctrl)
%[text] The library blocks grid\_voltage\_sequences, frt\_vde4110, grid\_current\_sequences\_q\_ctrl and islanding\_detector read one struct from their mask parameter p: param.afe.uvw. The fields are those of afe\_uvw\_ctrl\_param\_example.m of the library (see the README of the package masked\_models/afe\_uvw\_ctrl); the numbers come from param.afe.ctrl, param.frt.data, param.ctrl and param.grid, the same values as in afe\_uvw\_three\_phase. The blocks inherit the sample time of the triggered subsystem afe\_control: uvw.ts is its period.
param.afe.uvw = prv.afe_uvw_param(param);
%%
%[text] ## PLL design check
%[text] Loops of the three Simulink PLLs with the gains above: closed loop poles and damping of the phase loop with the normal and the fault gains, gain from the 100 Hz ripple of the eta error (negative sequence of the fault) to the phase, double-integrator observer and first harmonic trackers, resonance of the LCL filter seen from the grid with the converter side open. Numbers in param.afe.design, figures when param.sim.design\_figures is true.
param.afe.design = prv.pll_design(param, options);
%%
%[text] ## Model setup
open_system(model);
%[text] Output types (buses) of the C-Caller blocks: the first harmonic trackers of the DDSRF-PLL, of the FHT-PLL and of the C-Caller dq-PLL, and the dq-PLL dqpll\_grid. The C sources are in the library (user\_defined\_functions/ccaller), listed in the configuration of the model.
Simulink.importExternalCTypes(model, 'Names', {'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'dqpll_grid_output_t'});
%[text] Scopes closed at start.
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### CREATE\_POWER\_METER\_BUS
%[text] Simulink.Bus PowerMeterIecBus for the output m of the MATLAB Function block three\_phase\_power\_meter\_iec (Symbols pane: Type = Bus: PowerMeterIecBus). Bus objects stay in the base workspace.
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
        fprintf('%s init: every active From block has its Goto\n', model);
    else
        warning('%s init: %d tags read by active From blocks have no active Goto:\n  %s', model, numel(lost), ...
            strjoin(lost, '\n  '));
    end
    clear missing_refs model_refs lost
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":33.8}
%---
