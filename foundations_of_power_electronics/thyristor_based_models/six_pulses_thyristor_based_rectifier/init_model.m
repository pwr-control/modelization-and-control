%[text] # Six-pulse thyristor rectifier with load current control (six_pulse_thyristor_based_rectifier)
%[text] Initialization of six_pulse_thyristor_based_rectifier.slx: 400 V 50 Hz grid emulator (ideal sources) ->
%[text] Dyn11 100 kVA 400/400 V transformer (configurable_trafo, ucc 6 %, star point of the secondary grounded) ->
%[text] three 65 uH inductors -> six-pulse thyristor bridge (B6C, Lauritzen-Ma thyristor model with RC snubbers) ->
%[text] DC filter (1.2 mH split on DC+ and DC-, two 8.6 mF capacitors, one with a 0.25 Ohm damping resistor) ->
%[text] load 3.4 Ohm + 250 uH. Control at 5 kHz: the three voltages at the bridge terminals go through a
%[text] first-harmonic observer each and into the C-Caller PLL dqpll_thyr; the C-Caller sprcProcessSimulink makes the
%[text] six firing pulses (109 deg long) from the PLL phase and the firing angle; the load current PI (PI_i) gives
%[text] cos(alpha) and acos gives alpha (PI_v, the voltage PI, is not selected). Test: load current reference 20 A,
%[text] step to 100 A at 0.5 s.
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, grid_emulator_param, grid_fault_param, transformer_param, thyristor_param,
%[text] rect_ctrl_param, check_model_param_refs). The library is needed for the Simulink/Simscape blocks (grid
%[text] emulator, configurable transformer, thyristor, first-harmonic observer, power meter), for the C-Caller
%[text] sources (six_pulse_rect, dqpll_thyr) and for the Simscape components. Run it from this folder (the
%[text] functions of ./private are visible only to the files of this folder). The draft init (copy of the shared
%[text] AFE/inverter template plus the B6C data: grid emulator data for 690 V, fault data in param.frt instead of
%[text] the param.grid.frt read by the emulator mask) is kept as init_model_legacy.m until the merge.

clear;
model = 'six_pulse_thyristor_based_rectifier';

% Simulink/Simscape library blocks (blocks, not parameters) and C-Caller sources
library_path = 'C:\github\library';
if exist(library_path, 'dir')
    addpath(genpath(library_path));
else
    warning('%s init: library path %s not found, the library blocks must already be on the path', model, library_path);
end

% environment (was init_environment of the library)
close all; clc; beep off;
try
    pm_addunit('percent', 0.01, '1');       % custom Simscape unit used by the library blocks
catch ME
    if ~contains(ME.message, 'percent')     % already registered in this session: fine
        rethrow(ME);
    end
end
options = bodeoptions;
options.FreqUnits = 'Hz';
s = tf('s');
param = struct();

% local parameter functions of this model (./private)
local_functions = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
fprintf('%s init: local parameter functions in private/: %s\n', model, strjoin({local_functions.name}, ', '));
clear local_functions library_path
%[text] ## Simulation
param.sim.simlength = 1.2;                  % [s] model StopTime (legacy simlength)
param.sim.tc = 1.0e-6;                      % [s] logging base, MaxStep of ode23tb and step of the Simscape local
                                            %     solver (legacy Tc = Ts = 0.5 us = Tcontrol_rect/400; 1 us since
                                            %     Davide's run of 1 Nov 2026)
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration (grid_line/Grid/grid_emulator), 5 in the legacy
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
% one clock domain: the digital control of the rectifier at 5 kHz (PLL, firing pulses, current PI); the power
% meter blocks copied from the AFE models run at the same step
param.time.fs_rect = 5e3;                           % [Hz] legacy fPWM_RECT (Tcontrol_rect = 200 us)
param.time.tc_factor = 1/param.time.fs_rect/param.sim.tc;   % ts_rect/tc = 200 (400 in the legacy)
param.time.t_measure = 20e-3;                       % [s] logging window at tc (thyristor_data), legacy t_misura

param.glb_time = timing_param(param.time, param.sim.simlength);
%[text] ## Grid
% three-phase grid emulator (ideal voltage sources, star point grounded) at 400 V 50 Hz. Fault data of the draft
% init (test 25.4 of the template: 75 % dip on two phases, variant D, from 0.75 s for 915 ms); the draft had
% enable_frt_2 = 1, here both enables are 0 (no dip during the current step test): set the last argument to 1
% to apply it
param.grid.application_voltage = 400;               % [V] rms line-to-line (the draft had 690 V, see the README)
param.grid.f_nom = 50;                              % [Hz]
param.grid.omega_nom = 2*pi*param.grid.f_nom;       % [rad/s] legacy w_grid
param.grid.frt = grid_fault_param('two_phase_D', 0.75, 0.915, 0.75, 0, 0);
param.grid.emu = grid_emulator_param(param.grid, param.grid.frt);
%[text] ### Transformer
% configurable_trafo (one primary): delta primary, star secondary with the star point grounded. The model had
% clock3 = 0, which the component rejects with a delta primary and a star secondary (the clock number must be
% odd): 11 here (Dyn11, the vector group named in the draft init). The PLL reads the voltages at the bridge,
% so the 30 deg shift of the secondary does not change the firing.
trafo_in.Sr = 100e3;                                % [VA]
trafo_in.V1 = 400;                                  % [V] rms line-to-line, primary
trafo_in.V2 = 400;                                  % [V] primary 2 (not used: one primary)
trafo_in.V3 = 400;                                  % [V] secondary
trafo_in.fr = param.grid.f_nom;                     % [Hz]
trafo_in.r1 = 0.005; trafo_in.l1 = 0.030;           % [pu] primary coil
trafo_in.r2 = 0.005; trafo_in.l2 = 0.030;           % [pu] primary 2 (not used)
trafo_in.r3 = 0.005; trafo_in.l3 = 0.030;           % [pu] secondary coil
trafo_in.lm = 100;                                  % [pu] magnetizing inductance
trafo_in.lm0 = 0.5;                                 % [pu] zero-sequence magnetizing inductance
trafo_in.rfe = 500;                                 % [pu] core-loss resistance per limb
trafo_in.conn1 = 'delta';                           % enumerations typed in the block (checks only)
trafo_in.conn3 = 'star';
trafo_in.clock2 = 0;                                % primary 2 (not used)
trafo_in.clock3 = 11;                               % Dyn11 (model: 0, see above)
param.grid.trafo = transformer_param(trafo_in);
clear trafo_in
%[text] ## Six-pulse thyristor rectifier
%[text] ### Hardware
param.rect.hw.Lac = 65e-6;                          % [H] series inductors between the transformer and the bridge (Inductor, Inductor1, Inductor2)
param.rect.hw.LFu_dc = 1.2e-3;                      % [H] DC inductance, two halves on DC+ and DC- (LFu_dc_1, LFu_dc_2)
param.rect.hw.RLFu_dc = 2e-4;                       % [Ohm]
param.rect.hw.f0_dc = 35;                           % [Hz] design resonance of LFu_dc with CFu1_dc + CFu2_dc
param.rect.hw.CFu1_dc = 0.5/(2*pi*param.rect.hw.f0_dc)^2/param.rect.hw.LFu_dc;   % [F] 8.62 mF, damping branch
param.rect.hw.CFu2_dc = param.rect.hw.CFu1_dc;      % [F] 8.62 mF
param.rect.hw.RCFu1_dc = 0.25;                      % [Ohm] damping resistor in series with CFu1_dc
param.rect.hw.RCFu2_dc = 2.5e-3;                    % [Ohm] ESR of CFu2_dc
%[text] ### Thyristors, snubbers and gate command
% the thyristor data go to the mask of BR2/ssc_thyristor_bridge (typed there before); the gate command of the
% thyristor block is a physical signal compared with Vgt (the firing circuit with the pulse transformer is
% commented out in the model, and the strobe generator is commented through)
param.rect.thy = thyristor_param('abb_5STP26N6500_A');
param.rect.snubber.R = 10;                          % [Ohm] RC snubber across each thyristor (mask Rsnubber_m)
param.rect.snubber.C = 1e-6;                        % [F] (mask Csnubber_m)
param.rect.gate.u_on = 12;                          % [V] gate command level (gains of C-caller ctrl/shift_level)
%[text] ### Measurements and bases
% current and voltage full scale of the PI inputs (legacy margin_factor 1.15 on 90 A / 340 V), AC voltage base
% of the PLL input and of the measurements (peak phase voltage at 400 V), rms meter of the snubber currents
% (exponential mean of i^2 at tc with time constant n1*tc = one period)
param.meas.margin_factor = 1.15;
param.meas.Vout_dc_nom = 340;                       % [V]
param.meas.Iout_dc_nom = 90;                        % [A]
param.meas.I_FS = param.meas.Iout_dc_nom*param.meas.margin_factor;   % [A] 103.5 A
param.meas.V_FS = param.meas.Vout_dc_nom*param.meas.margin_factor;   % [V] 391 V
param.meas.Vnom_ac_adc = param.grid.application_voltage/sqrt(3)*sqrt(2);   % [V] 326.6 V
param.meas.rms.periods = 1;
param.meas.rms.n1 = 2*pi*param.meas.rms.periods/param.grid.omega_nom/param.glb_time.tc;   % 20000 (legacy rms.n1, 40000 at 0.5 us)
%[text] ### Control
% values of the legacy init and literals of the blocks; the gains of the C-Caller PLL and the pulse width are
% fixed in the C sources (see rect_ctrl_param)
ctrl_in.pi_i.kp = 2;                                % [pu/pu] load current PI (block "1/Ti" of PI_i)
ctrl_in.pi_i.ki = 35;                               % [1/s] (block "kp" of PI_i = ki*ts)
ctrl_in.pi_v.kp = 0.5;                              % [pu/pu] output voltage PI (not selected)
ctrl_in.pi_v.ki = 18;                               % [1/s]
ctrl_in.u_min = -1;                                 % [-] clamp of cos(alpha) (alpha up to 180 deg, see the README)
ctrl_in.u_max = 1;
ctrl_in.alpha_min = 0;                              % [rad] clamp of alpha after acos
ctrl_in.alpha_max = pi;
ctrl_in.iref_1 = 20;                                % [A] load current reference
ctrl_in.iref_2 = 100;                               % [A] after the step
ctrl_in.t_iref_step = 0.5;                          % [s]
ctrl_in.vref = 200;                                 % [V] reference of PI_v
ctrl_in.t_pulse_release = 0.01;                     % [s] pulses blocked before (Step in six_pulse rectifier)
ctrl_in.t_pll_reset = 0.01;                         % [s] PLL dqpll_thyr in reset before (Step in dqpll_thyr_ccaller)
ctrl_in.pll.kp = 1;                                 % Simulink PLL (legacy pll_p, scope only)
ctrl_in.pll.ki = 80;                                % [1/s] (legacy pll_i1)
ctrl_in.fht.delta = 0.05;                           % first-harmonic observers (literals of the blocks)
ctrl_in.fht.pole1_factor = 1;
ctrl_in.fht.pole2_factor = 4;
param.rect.ctrl = rect_ctrl_param(param.glb_time.ts_rect, param.grid.omega_nom, ctrl_in);
clear ctrl_in
%[text] ## DC load
param.load.R = 0.9*param.meas.Vout_dc_nom/param.meas.Iout_dc_nom;   % [Ohm] 3.4 Ohm: 340 V at 100 A (legacy Rload)
param.load.L = 250e-6;                              % [H] (legacy Lload)
%[text] ### Design figures (informative)
% steady state of the bridge at the two current references (continuous DC current): ideal no-load voltage
% Vdi0 = 3*sqrt(2)/pi*U2, drops of the commutation (3/pi*omega*Lc*Id), of the resistances (two phases and two
% thyristors in series) and of the thyristor threshold; overlap mu from cos(alpha) - cos(alpha + mu) =
% 2*omega*Lc*Id/(sqrt(2)*U2); extinction angle gamma = 180 deg - alpha - mu, against the turn-off time tq of
% the thyristor model (a thyristor that sees forward voltage again within tq fires again). Lc is the
% transformer leakage referred to the secondary plus Lac. DC filter: resonance of LFu_dc with the two
% capacitors, and with two phases of Lc in series (bridge conducting, outside the commutations).
hw = param.rect.hw; tr = param.grid.trafo; thy = param.rect.thy; w = param.grid.omega_nom;
d.U2 = tr.V3;                                                       % [V] rms line-to-line at the bridge, no load
d.Vdi0 = 3*sqrt(2)/pi*d.U2;                                         % [V]
d.Lc = tr.L_sc2 + hw.Lac;                                           % [H] commutation inductance per phase
d.Rc = tr.R_sc2;                                                    % [Ohm]
d.dx = 3/pi*w*d.Lc;                                                 % [Ohm] commutation drop per ampere
d.Id = [param.rect.ctrl.iref_1, param.rect.ctrl.iref_2];            % [A]
d.Vd = d.Id*param.load.R;                                           % [V] load voltage
d.Vb = d.Vd + d.Id*hw.RLFu_dc + 2*(thy.Vf + thy.Ron*d.Id) + 2*d.Rc*d.Id + d.dx*d.Id;   % [V] bridge, ideal average
d.cos_alpha = d.Vb/d.Vdi0;
d.alpha = acos(d.cos_alpha);                                        % [rad]
d.mu = acos(d.cos_alpha - 2*w*d.Lc*d.Id/(sqrt(2)*d.U2)) - d.alpha;  % [rad] overlap
d.gamma = pi - d.alpha - d.mu;                                      % [rad] extinction angle
d.tq_angle = w*thy.tq;                                              % [rad]
d.alpha_max_commutation = pi - d.mu(end) - d.tq_angle;              % [rad] largest alpha with a safe turn-off at Id(end)
d.C_dc = hw.CFu1_dc + hw.CFu2_dc;                                   % [F]
d.f_res_filter = 1/(2*pi*sqrt(hw.LFu_dc*d.C_dc));                   % [Hz]
d.f_res_bridge = 1/(2*pi*sqrt((hw.LFu_dc + 2*d.Lc)*d.C_dc));        % [Hz]
d.R_dc = param.load.R + hw.RLFu_dc + 2*thy.Ron + 2*d.Rc + d.dx;     % [Ohm] DC resistance seen by the average bridge voltage
d.k_plant = d.Vdi0/d.R_dc/param.meas.I_FS;                          % [pu of I_FS per unit of cos(alpha)] static gain
d.k_loop_p = param.rect.ctrl.pi_i.kp*d.k_plant;                     % [-] proportional loop gain at low frequency
param.rect.design = d;
fprintf('Bridge: Vdi0 = %.1f V, Lc = %.1f uH (transformer %.1f + %.0f uH), commutation drop %.1f mOhm/A, tq = %.1f deg\n', ...
    d.Vdi0, d.Lc*1e6, tr.L_sc2*1e6, hw.Lac*1e6, d.dx*1e3, d.tq_angle*180/pi);
for k = 1:numel(d.Id)
    fprintf('  Id = %5.1f A: Vd = %5.1f V, alpha = %5.1f deg (cos %.3f), overlap %4.1f deg, extinction %5.1f deg\n', ...
        d.Id(k), d.Vd(k), d.alpha(k)*180/pi, d.cos_alpha(k), d.mu(k)*180/pi, d.gamma(k)*180/pi);
end
fprintf('  safe turn-off at %.0f A needs alpha < %.1f deg; the clamp allows %.1f deg (cos(alpha) >= %.3f)\n', ...
    d.Id(end), d.alpha_max_commutation*180/pi, param.rect.ctrl.alpha_max*180/pi, param.rect.ctrl.u_min);
fprintf('DC filter: %.2f + %.2f mF, resonance %.1f Hz with LFu_dc, %.1f Hz with 2*Lc in series | current loop: static plant gain %.2f, kp*gain %.2f\n', ...
    hw.CFu1_dc*1e3, hw.CFu2_dc*1e3, d.f_res_filter, d.f_res_bridge, d.k_plant, d.k_loop_p);
clear hw tr thy w d k

%[text] ## Model setup
open_system(model);
Simulink.importExternalCTypes(model, 'Names', {'spr_p_t'});
Simulink.importExternalCTypes(model, 'Names', {'dqpll_thyr_output_t'});
%[text] ### Scopes closed at start
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### Check of the param.* references of the model
if param.sim.check_param_refs
    [missing_refs, model_refs] = check_model_param_refs(model, param);
    if isempty(missing_refs)
        fprintf('%s init: all %d param.* references of the model blocks resolve\n', model, numel(model_refs));
    else
        warning('%s init: %d param.* references of the model blocks do not exist in param:', model, numel(missing_refs));
        disp(missing_refs);
    end
    clear missing_refs model_refs
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
