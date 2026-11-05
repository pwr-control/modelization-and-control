%[text] # Power supply 25 V, 12 kA (ps\_25V\_12kA\_r)
%[text] Initialization of ps\_25V\_12kA\_r.slx. A 480 V, 60 Hz grid (emulator with fault generator, equivalent grid impedance, Dyn11 1750 kVA transformer made of three single-phase transformer blocks) feeds a two-level AFE of 325 kW (LCL filter, DC link 750 V, brake chopper). The DC link supplies a phase-shift full bridge at 4 kHz with a centre-tapped transformer 34:2:2 and a two-diode rectifier, which drives 12 kA into a 2.08 mOhm load (25 V). Design document: docs/ps\_25V\_12kA\_r.pdf.
%[text] All the data are in one struct param, one group per part of the model: sim, time, glb\_time, grid, afe, psfbc (phase-shift full bridge), diode, thermal, battery, load, meas. The model reads param.\* only; the README lists the old names. The functions of ./private are called through the handles of ps\_25V\_12kA\_private.m (prv.timing\_param, prv.afe\_hw\_param, ...): the Live Editor runs a live script from a copy outside this folder and does not see ./private. Run the whole script once (Run): a section run on its own needs prv and param in the workspace.
clear;
model = 'ps_25V_12kA_r';
prv = ps_25V_12kA_private();    % handles to the functions of ./private (the Live Editor does not see ./private)
prv.library_setup();            % pwr-control/library on the path (blocks, C-Caller sources), cloned when missing

% environment (was init_environment of the library)
close all; clc; beep off;
try
    pm_addunit('percent', 0.01, '1');       % custom Simscape unit used by some library blocks
catch ME
    if ~contains(ME.message, 'percent')     % already registered in this session: fine
        rethrow(ME);
    end
end
param = struct();
fprintf('%s init: functions of ./private, called as prv.<name>: %s\n', model, strjoin(fieldnames(prv)', ', '));
%%
%[text] ## Simulation
%[text] The full bridge has three variants in the model: ideal switches (default), IGBT with thermal model and MOSFET with thermal model (ZVS snubbers). The init comments out the two that are not used, together with the blocks that log their data. The AFE bridge is always the ideal-switch one.
param.sim.simlength = 1.75;                 % [s] model StopTime
param.sim.use_thermal_model = 0;            % 1: IGBT full bridge with thermal model (full_bridge_igbt_adv_thermal_model)
param.sim.use_mosfet_thermal_model = 0;     % 1: MOSFET full bridge with thermal model (has priority over the IGBT one)
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration: ideal switches
if param.sim.use_mosfet_thermal_model || param.sim.use_thermal_model
    param.sim.nonlinear_iterations = 5;     % semiconductor models with thermal network
end
param.sim.check_model = true;               % last section: param.* references and From blocks without a Goto
param.sim.design_figures = true;            % section "Design figures": operating point and loops, printed
%%
%[text] ## Timing
%[text] AFE and full bridge switch at 4 kHz with double update: their control runs at 8 kHz (ts 125 us). The step tc = ts\_afe/200 = 0.625 us is the logging base, the MaxStep of the solver and the step of the Simscape local solver. ts\_dab (12 kHz with double update) is a leftover of the legacy template: the low-pass filter and the PI of the DC transformer current compensation, and the battery block (commented out), are set with it, while the compensation runs at the full-bridge trigger (README, open points).
param.time.fpwm_afe = 4e3;                  % [Hz] switching frequency of the AFE
param.time.trgo_afe = 1;                    % double update
param.time.dead_time_afe = 3e-6;            % [s]
param.time.fpwm_psfbc = 4e3;                % [Hz] switching frequency of the full bridge
param.time.trgo_psfbc = 1;                  % double update: two triggers per period
param.time.dead_time_psfbc = 3e-6;          % [s]
param.time.fpwm_dab = 3*param.time.fpwm_psfbc;      % [Hz] only for ts_dab
param.time.trgo_dab = 1;
param.time.t_measure = param.sim.simlength; % [s] logging window
param.time.tc_factor = 200;                 % tc = ts_afe/tc_factor
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;                   % [s] (not used by this model)
param.glb_time = prv.timing_param(param.time);
%%
%[text] ## Grid
%[text] Grid emulator (three ideal voltage sources), equivalent grid impedance (short-circuit inductance of a 1750 kVA grid with ucc 5 percent, 17.5 uH, plus 2 mOhm) and a Dyn11 transformer 480/480 V, 1750 kVA, ucc 5 percent, made of three single-phase transformer blocks: delta primary coils of n1 = 24 turns, star secondary coils of n2 = 14 turns (star point grounded). The magnetizing inductance comes from the core (Lm1 = mu0\*mur\*n1\*n1\*area/length = 145 mH). The transformer data are those of one coil of the blocks, with the factor 3 of the delta primary on Ld1, Rd1 and Rfe1, as the library class after its fix of November 2026 (README).
param.grid.application_voltage = 480;           % [V] rms line-to-line, primary and secondary
param.grid.f_nom = 60;                          % [Hz]
param.grid.pwr_nom = 1750e3;                    % [VA] rated power of the transformer and of the equivalent grid
param.grid.ucc = 5;                             % [%] short-circuit voltage
param.grid.eta = 95;                            % [%] efficiency of the transformer (copper losses)
param.grid.p_iron = 1400;                       % [W] iron losses of the transformer
param.grid.n2 = 14;                             % turns of the secondary (star) coils
param.grid.n1 = floor(param.grid.n2*sqrt(3));   % turns of the primary (delta) coils: 24
param.grid.core_area = 0.05;                    % [m^2]
param.grid.core_length = 2.5;                   % [m]
param.grid.mur = 10e3;                          % [-] relative permeability: Lm1 = 145 mH
param.grid.ucc_factor = 1;                      % [-] multiplies the equivalent grid inductance (weak grid: > 1)
param.grid.eq_grid_resistance = 2e-3;           % [Ohm]
%[text] ### Fault test of the grid emulator
%[text] Both fault generators are off, as in the legacy init. The data are those of the legacy test 25.4: 75 percent dip on two phases, variant D, 915 ms from 0.75 s. Set enable\_frt\_1 = 1 to apply it from the abc amplitudes.
param.grid.frt = prv.grid_fault_param('two_phase_D', 0.75, 0.915, 0.75, 0, 0);
param.grid.emu = prv.grid_emulator_param(param.grid, param.grid.frt);
%%
%[text] ## AFE (afe\_abc\_mod1)
%[text] Two-level bridge (ideal switches), LC filter 330 uH / 360 uF in star with 50 mOhm (the transformer leakage and the grid impedance are the grid-side inductance), DC link CFi = 2.4 mF (the legacy init set it to twice the input capacitor of the full bridge, which is on the same bus), brake chopper 2.5 Ohm.
param.afe.pwr_nom = 325e3;                      % [W] rated power (bases: ibez = 614.2 A peak)
param.afe.hw_in.CFi = 2.4e-3;                   % [F] DC-link capacitor of the AFE
param.afe.hw_in.RCFi = 1e-3;                    % [Ohm] its series resistance (informative)
param.afe.hw = prv.afe_hw_param(param.grid.application_voltage, param.afe.pwr_nom, param.glb_time.fPWM_AFE, param.afe.hw_in);
%[text] ### Behaviour
param.afe.behav.time_gain_module_1 = 1.0002;            % [-] clock ratio of the local time of the module
param.afe.behav.pwm_phase_shift_mod1 = 0;               % [rad] carrier phase
param.afe.behav.u_cm_comp_mod1 = 0;                     % [-] zero-sequence factor of the modulator
param.afe.behav.time_aux_power_supply_fault = 1e3;      % [s] step of the brake command (after the end: off)
%[text] ### PLL selection (variant)
%[text] The four PLLs run in parallel; two Variant Source blocks choose which one gives the grid phase and frequency to the control. Mode 4 (FHT-PLL in C-Caller) as the legacy init; exactly one must be 1.
param.afe.variant.dq_pll_mode1 = 0;             % (variant) Simulink dq PLL
param.afe.variant.dq_pll_mode2 = 0;             % (variant) C-Caller dq PLL
param.afe.variant.dq_pll_mode3 = 0;             % (variant) FHT-PLL in Simulink
param.afe.variant.dq_pll_mode4 = 1;             % (variant) FHT-PLL in C-Caller
%[text] ### Control
%[text] Gains of the library class ctrl\_afe\_setup with the change of the legacy init (kp\_udc\_ctrl = 2). The section "Design figures" prints crossover and margins of the current and DC-link loops.
g = struct();
g.kp_is = 0.25;                 % [pu/pu] dq PI of the converter current (xi and eta)
g.ki_is = 45;                   % [1/s]
g.clip_release = 1e-3;          % [1/s] decay of the integrators while the voltage vector is clipped (CTRPIFF_CLIP_RELEASE)
g.udc_norm_ref = 1;             % [pu] DC-link voltage reference (750 V)
g.kp_udc_ctrl = 2;              % [pu/pu] DC-link voltage PI (library 0.85)
g.ki_udc_ctrl = 35;             % [1/s]
g.ixi_pos_ref_lim = 1.65;       % [pu] limit of the active current reference
g.kp_rc_pos_grid = 0.25;        % [pu/pu] PI of the positive-sequence reactive grid current
g.ki_rc_pos_grid = 35;          % [1/s]
g.ieta_pos_ref_lim = 1.0;       % [pu] limit of the reactive current reference
g.pll_p = 1;                    % PLL PI (Simulink dq PLL and FHT-PLLs)
g.pll_i1 = 80;                  % [1/s]
g.pll_p_frt = 0.2;              % PLL gain during a fault (Simulink dq PLL)
g.align.kp = 0.25;              % local time alignment to the master time (legacy kp_align, ki_align, ...)
g.align.ki = 18;                % [1/s] (the model multiplies it by ts_afe)
g.align.lim_up = 0.05;
g.align.lim_down = -0.05;
param.afe.ctrl = prv.afe_ctrl_param(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom, g);
clear g
%%
%[text] ## Phase-shift full bridge (full\_bridge\_converter)
%[text] Data set PSFBC\_800V of the library with the numbers of the legacy init: input capacitor 1.2 mF, centre-tapped transformer 34:2:2 (Lm 1 mH, leakage Ls = udc\*udc/(8\*fpwm\*P\*2.7) = 20 uH, half on the primary and half on the secondaries), output inductor 0.4 uH, output capacitor 45 mF. The bases of the control are idc2\_bez = 12 kA (output current) and idc1\_bez = P/udc = 433 A (input current).
param.psfbc.pwr_nom = 325e3;                    % [W]
h = struct();
h.udc2 = 45;                    % [V] output voltage base
h.idc2 = 12e3;                  % [A] output current base and reference of the current loop
h.n1 = 34;                      % turns of the primary
h.n2 = 2;                       % turns of each half of the centre-tapped secondary
h.Lm = 1e-3;                    % [H] magnetizing inductance (primary)
h.Rfe = 1e3;                    % [Ohm] iron losses (primary)
h.Rs1 = 1e-3;                   % [Ohm] primary winding (each secondary half: Rs1*(n2/n1)^2)
h.core_length = 1;              % [m] only for the flux density output of the transformer block
h.core_mur = 70;                % [-]
h.ls_factor = 2.7;              % leakage design factor: Ls = udc^2/(8*fpwm*pwr_nom*ls_factor)
h.Cdc_dc1 = 1.2e-3;             % [F] input capacitor CFi_dc1
h.RCdc_dc1 = 1e-3;              % [Ohm]
h.Cdc_dc2 = 45e-3;              % [F] output capacitor CFu_dc2
h.RCdc_dc2 = 1e-4;              % [Ohm]
h.Ldc_dc1 = 250e-6;             % [H] input inductor of the data set (not in the model)
h.RLdc_dc1 = 157*0.02*h.Ldc_dc1;
h.Ldc_dc2 = 0.4e-6;             % [H] output inductor
h.RLdc_dc2 = 157*0.01*h.Ldc_dc2;
param.psfbc.hw_in = h;
param.psfbc.hw = prv.psfbc_hw_param(param.afe.hw.udc_nom, param.psfbc.pwr_nom, param.glb_time.fPWM_PSFBC, h);
clear h
%[text] ### Control
%[text] Output current loop: a PI on the load current (in pu of 12 kA) gives the phase displacement of leg B (0 to 1 = 0 to pi); the reference steps to 12 kA at 25 ms with a slope of 10 times 12 kA per second. The DC transformer current compensation adds its PI output (limited at plus or minus lim\_idc\_trafo) to the duty 0.5 of both legs. It reads the DC input current of the bridge, which is positive at full load: it sits at its lower limit and caps the effective duty at 1 - 2\*lim\_idc\_trafo = 0.8, which the section "Design figures" compares with the duty the rated point needs. lim\_idc\_trafo = 0 switches it off. Values of the legacy model; they were typed in the blocks before.
param.psfbc.ctrl.kp_idc = 1;                    % [pu/pu] output current PI (legacy psfbc_ctrl.kp_idc)
param.psfbc.ctrl.ki_idc = 18;                   % [1/s]
param.psfbc.ctrl.kp_udc = 0.5;                  % [pu/pu] output voltage PI of the data set (not used by the model)
param.psfbc.ctrl.ki_udc = 18;                   % [1/s] (not used by the model)
param.psfbc.ctrl.kp_idc_trafo = 0.1;            % [pu/pu] DC transformer current compensation (typed in the model before)
param.psfbc.ctrl.ki_idc_trafo = 2;              % [1/s] (block ts = ts_dab)
param.psfbc.ctrl.lim_idc_trafo = 0.1;           % [-] limit of the duty correction; 0 switches the compensation off
param.psfbc.ctrl.tau_idc_trafo = 20e-3;         % [s] low-pass filter of the measured current (block ts = ts_dab)
%%
%[text] ## Rectifier diodes
%[text] ABB 5SDF 0131Z0401 (Simscape diode with charge model), blocks diode\_rectifier\_1 and diode\_rectifier\_2 (diode\_rectifier\_3 to 5 are commented out in the model).
d = struct();
d.Vf = 0.977;               % [V] forward voltage
d.Rdon = 22e-6;             % [Ohm] on-state resistance
d.Cj = 0;                   % [F] junction capacitance
d.Irm = -75;                % [A] peak reverse recovery current
d.didt = -30;               % [A/us] at the recovery
d.trr = 5;                  % [us] reverse recovery time
d.Qrr = 325e-6;             % [C] recovered charge
d.Ifm = 2000;               % [A] forward current of the recovery data
d.Vr = -50;                 % [V] reverse voltage of the recovery data
d.Err = 15e-3;              % [J] recovery energy
d.Rth_JC = 0.004;           % [K/W] junction to case
d.Rth_CH = 0.003;           % [K/W] case to heatsink
d.Rsnubber = 1e4;           % [Ohm]
d.Csnubber = 1e-12;         % [F]
param.diode.rectifier = d;
clear d
%%
%[text] ## Power semiconductors
%[text] Ideal switch for the AFE bridge and the brake (param.afe.ideal\_switch) and for the full bridge (param.psfbc.ideal\_switch); IGBT and MOSFET data sets for the full bridges with thermal model and for the IGBT AFE bridge (commented out in the model).
param.afe.ideal_switch = prv.device_ideal_switch_param('silicon_high_power_ideal_switch', param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.psfbc.ideal_switch = prv.device_ideal_switch_param('silicon_high_power_ideal_switch', param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom);
param.afe.igbt = prv.device_igbt_param('mitsubishi_CM1200DW_34T', param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.psfbc.igbt = prv.device_igbt_param('mitsubishi_CM1200DW_34T', param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom);
param.psfbc.mosfet = prv.device_mosfet_param('danfoss_SKM1700MB20R4S2I4', param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom);
%%
%[text] ## Heatsink
%[text] Aluminium plate, liquid cooled, for PrimePACK2 modules: water flow above 28 l/min, water in/out rise at most 5 K with 2 kW of losses. HA is the rise from the water to the heatsink surface. no\_weight = weight/10 leaves out the thermal inertia.
param.thermal.weight = 0.150;                                   % [kg]
param.thermal.no_weight = 0.150/10;                             % [kg]
param.thermal.cp_al = 900;                                      % [J/(kg K)] aluminium
param.thermal.heat_capacity_hs = param.thermal.cp_al*param.thermal.weight;   % [J/K]
param.thermal.thermal_conductivity_al = 160;                    % [W/(m K)]
param.thermal.Rth_switch_HA = 15/1000;                          % [K/W]
param.thermal.Rth_mosfet_HA = param.thermal.Rth_switch_HA;      % [K/W]
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;       % [K/W]
param.thermal.Tambient = 40;                                    % [degC] water temperature
param.thermal.DThs_init = 0;                                    % [K] initial heatsink rise
% heatsink object of the library class liquid_cooled_plate_2kw_setup, read by the masks of the bridges with
% thermal model (fields cp, no_weight, Rth_switch_HA, Rth_mosfet_HA, Rth_diode_HA, Tambient, DThs_init)
param.thermal.heatsink.name = 'liquid_cooled_plate_2kw';
param.thermal.heatsink.weight = param.thermal.weight;
param.thermal.heatsink.no_weight = param.thermal.no_weight;
param.thermal.heatsink.cp = param.thermal.cp_al;
param.thermal.heatsink.heat_capacity = param.thermal.heat_capacity_hs;
param.thermal.heatsink.thermal_conductivity = param.thermal.thermal_conductivity_al;
param.thermal.heatsink.Rth_switch_HA = param.thermal.Rth_switch_HA;
param.thermal.heatsink.Rth_mosfet_HA = param.thermal.Rth_mosfet_HA;
param.thermal.heatsink.Rth_diode_HA = param.thermal.Rth_diode_HA;
param.thermal.heatsink.Tambient = param.thermal.Tambient;
param.thermal.heatsink.DThs_init = param.thermal.DThs_init;
%%
%[text] ## Battery (battery\_1, commented out in the model)
%[text] Lithium-ion pack at the DC-link voltage, 250 kW, 1C capacity, as the legacy init (R0 and R1 halved, C1 divided by 50). The block is commented out: the data only keep it ready.
param.battery.pack_1 = prv.battery_param(param.afe.hw.udc_nom, 250e3, 250e3/param.afe.hw.udc_nom, 0.85, param.glb_time.ts_dab, false);
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/50;
%%
%[text] ## Load
param.load.u_out = 25;                          % [V] rated output voltage
param.load.i_out = 12e3;                        % [A] rated output current
param.load.R = param.load.u_out/param.load.i_out;   % [Ohm] block Resistor at the output: 2.083 mOhm
%%
%[text] ## Measurements
param.meas.adc_quantization = 1/2^11;           % [pu] 12-bit converter on +-1 pu
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;
%%
%[text] ## Design figures
%[text] Averaged models (the README explains them): operating point and duty budget of the full bridge, its output current loop, and the AFE current and DC-link voltage loops. With the legacy settings two points stand out: the DC transformer current compensation caps the duty of the full bridge at 0.8 while the rated point needs about 0.83 (the output stays near 11.6 kA, 24.2 V), and kp\_idc = 1 is close to the stability limit of the output current loop (gain margin 1.6 dB, the output filter mode at 1.35 kHz damped 6 percent). Both are left as in the legacy model; the README gives the numbers of the alternatives.
if param.sim.design_figures
    param.design = prv.design_figures(param);
end
%%
%[text] ## Model
%[text] ### C-Caller types and scopes
open_system(model);
Simulink.importExternalCTypes(model, 'Names', {'first_harmonic_tracker_output_t'});    % FHT-PLL (C-Caller)
Simulink.importExternalCTypes(model, 'Names', {'dqpll_grid_output_t'});                % dq PLL (C-Caller)
Simulink.importExternalCTypes(model, 'Names', {'sv_pwm_cm_output_t'});                 % space vector modulator
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### Full-bridge variant
%[text] The three full-bridge blocks are switched by commenting them in and out (as the legacy init). The blocks that log the data of a bridge (From blocks of its Goto tags, their To Workspace blocks and scopes) follow it: when the bridge is commented out, their From blocks would have no Goto.
fb = [model '/full_bridge_converter/full_bridge_converter_with_iso/full_bridge/'];
bridges = {'full-bridge_ideal_switch_based', 'full_bridge_igbt_adv_thermal_model', ...
    'full-bridge_mosfet_based_ with_thermal_model_zvs'};
readers = { ...
    {'afe_measurement_sim/From36', 'afe_measurement_sim/To Workspace41', 'scope/From25', 'scope/full_bridge_ideal_switch_dat'}, ...
    {'afe_measurement_sim/From15', 'afe_measurement_sim/From22', 'afe_measurement_sim/From23', 'afe_measurement_sim/From24', ...
     'afe_measurement_sim/From25', 'afe_measurement_sim/From26', 'afe_measurement_sim/From27', 'afe_measurement_sim/From28', ...
     'afe_measurement_sim/From30', 'afe_measurement_sim/To Workspace27', 'afe_measurement_sim/To Workspace28', ...
     'afe_measurement_sim/To Workspace29', 'afe_measurement_sim/To Workspace30', 'afe_measurement_sim/To Workspace31', ...
     'afe_measurement_sim/To Workspace32', 'afe_measurement_sim/To Workspace33', 'afe_measurement_sim/To Workspace34', ...
     'afe_measurement_sim/To Workspace36', 'scope/From6', 'scope/From7', 'scope/From8', 'scope/From9', 'scope/From18', ...
     'scope/From19', 'scope/From22', 'scope/From23', 'scope/bridge_device_power_losses_Q1', 'scope/bridge_device_power_losses_Q2'}, ...
    {'afe_measurement_sim/From29', 'afe_measurement_sim/To Workspace35'}};
if param.sim.use_mosfet_thermal_model
    active = 3;
elseif param.sim.use_thermal_model
    active = 2;
else
    active = 1;
end
for b = 1:numel(bridges)
    state = 'on';
    if b == active
        state = 'off';
    end
    set_param([fb bridges{b}], 'Commented', state);
    for r = 1:numel(readers{b})
        set_param([model '/' readers{b}{r}], 'Commented', state);
    end
end
fprintf('%s init: full bridge %s active\n', model, bridges{active});
clear fb bridges readers active b r state
%[text] ### Checks of the model
%[text] Every param.\* read by the active blocks must exist; then the From blocks of the active part that have no active Goto (the model does not compile with them).
if param.sim.check_model
    [missing_refs, model_refs] = prv.check_model_param_refs(model, param);
    if isempty(missing_refs)
        fprintf('%s init: all %d param.* references of the active blocks resolve\n', model, numel(model_refs));
    else
        warning('%s init: %d param.* references of the active blocks do not exist in param:\n  %s', model, ...
            numel(missing_refs), strjoin(missing_refs', '\n  '));
    end
    lost = prv.check_from_goto(model);
    if isempty(lost)
        fprintf('%s init: every From block of the active part has its Goto\n', model);
    else
        warning('%s init: %d tags of From blocks have no active Goto:\n  %s', model, numel(lost), strjoin(lost, '\n  '));
    end
    clear missing_refs model_refs lost
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":33.8}
%---
