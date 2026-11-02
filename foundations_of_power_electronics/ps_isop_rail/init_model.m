%[text] # Rail auxiliary power supply: four input-series output-parallel full-bridge modules (ps_isop_rail)
%[text] Initialization of ps_isop_rail.slx. The model is a 1 MW auxiliary power supply for a 3 kV DC railway
%[text] catenary, built from four identical 250 kW isolated DC/DC modules (ISOP: input series, output parallel).
%[text] The catenary is emulated by a lithium-ion battery (block battery_1/lithium_ion_battery, 4.5 kV nominal) with
%[text] a 1 mH line inductor. Its voltage is shared by the input capacitors of the four modules, which are in series
%[text] (each module sees about one quarter of the catenary voltage). Each module (isop/fb_iso_mod_1..4) is a full
%[text] bridge with ideal switches, driven with phase-shift modulation at 4 kHz, and a single-phase transformer with
%[text] turns ratio 1:1. The transformer secondaries of modules 1 and 4 are connected in parallel (AC side) and feed
%[text] the diode rectifier rectifier_1; modules 2 and 3 feed rectifier_2 in the same way. The two rectifiers are in
%[text] parallel on the DC side, each one through its own 250 uH inductor, on a common 1.2 mF output capacitor
%[text] with a 0.5 Ohm load (750 V, 1.125 MW).
%[text] Control (isop/phase_shift_full_bridge_ctrl, 8 kHz, triggered by the modulator): one PI on the output
%[text] voltage gives ONE phase shift for the four bridges (common-duty control). There is no input-voltage sharing
%[text] loop: the series input voltages balance by themselves through the parallel outputs. This works well between
%[text] the two rectifiers, but it is weak between the two modules that share a rectifier (see the README). The
%[text] model also has an output current PI and a "DC transformer current" PI: both are computed, but their outputs
%[text] are not connected (the duty d1 of the modulator stays 0.5).
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, isop_rail_hw_param, device_ideal_switch_param, battery_param, check_model_param_refs). The
%[text] library is needed only for the Simulink/Simscape blocks and for the C sources listed in the configuration
%[text] of the model. Run this script from this folder (the functions of ./private are visible only to the files
%[text] of this folder).
%[text] The old flat init is init_model_legacy.m (kept until the merge). It no longer runs with the library:
%[text] lithium_ion_battery_setup now takes the charge capacity, and the battery block reads
%[text] param.battery.nominal_battery_voltage_1 from its mask.

clear;
model = 'ps_isop_rail';

% Simulink/Simscape library blocks (blocks, not parameters) and C sources of the configuration
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
param = struct();

% local parameter functions of this model (./private)
local_functions = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
fprintf('%s init: local parameter functions in private/: %s\n', model, strjoin({local_functions.name}, ', '));
clear local_functions library_path
%[text] ## Simulation
param.sim.simlength = 1.75;                 % [s] model StopTime
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration (block battery_1/Solver Configuration)
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
% one clock domain: the four full bridges switch at fpwm with the same carriers; the modulator gives a control
% trigger at each peak of the carrier (double update) -> control at 8 kHz, ts = 125 us
param.time.fpwm = 4e3;                      % [Hz] switching frequency of the full bridges
param.time.trgo = 1;                        % double update: control at 2*fpwm
param.time.dead_time = 3e-6;                % [s] dead time of each leg
param.time.t_measure = param.sim.simlength; % [s] logging window: the whole simulation (legacy)
param.time.tc_factor = 200;                 % ts/tc: tc = 0.625 us (logging, MaxStep, Simscape local solver)
param.time.tc_decimation = 1;

% ts_dab of the shared template (3*fpwm, double update = 41.7 us): sample time of the state-of-charge estimator
% of the battery block, and coefficient of the low-pass filter and of the PI of the DC-current compensation
% (these two run in the 8 kHz control, see the README)
param.time.fpwm_dab = 3*param.time.fpwm;    % [Hz]
param.time.trgo_dab = 1;

param.glb_time = timing_param(param.time);
%[text] ## Catenary and catenary emulator (battery)
% 3 kV DC railway catenary (legacy data; EN 50163 gives 2.0 ... 3.6 kV of permanent range for 3 kV systems)
param.isop_rail.catenary_nominal_voltage = 3000;            % [V]
param.isop_rail.catenary_maximum_voltage = 4000;            % [V] (legacy value, not used)
param.isop_rail.catenary_nominal_power = 2*5.4e6;           % [W] per 12 km of line
param.isop_rail.catenary_nominal_current = param.isop_rail.catenary_nominal_power/param.isop_rail.catenary_nominal_voltage;

% lithium-ion pack as catenary emulator (block battery_1/lithium_ion_battery): 1.5 times the nominal catenary
% voltage, as in the legacy init (4.5 kV nominal, about 4.7 kV open circuit at soc 0.85). With the 1:1
% transformers the modules cannot give 750 V from the nominal catenary voltage (see the README).
% As in the legacy init, R0 and R1 are halved and C1 is divided by 200. The library block reads
% battery_voltage_init_m = param.battery.nominal_battery_voltage_1 from its mask default.
param.battery.nominal_battery_voltage_1 = 1.5*param.isop_rail.catenary_nominal_voltage;    % [V]
param.battery.nominal_battery_power = param.isop_rail.catenary_nominal_power;              % [W]
param.battery.initial_battery_soc = 0.85;
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;   % [Ah] ASSUMPTION: 1C rating
param.battery.plot_ocv = false;
b = param.battery;
param.battery.pack_1 = battery_param(b.nominal_battery_voltage_1, b.nominal_battery_power, b.charge_capacity_1, ...
    b.initial_battery_soc, param.glb_time.ts_dab, b.plot_ocv);
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;            % legacy changes
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/200;
fprintf('Battery block (after the legacy changes): R0 = %.1f mOhm, R1 = %.1f mOhm, C1 = %.1f mF\n', ...
    param.battery.pack_1.R0*1e3, param.battery.pack_1.R1*1e3, param.battery.pack_1.C1*1e3);
clear b

% inductance of the line between the emulator and the converter (block battery_1/Inductor, typed 1e-3 before)
param.battery.L_line = 1e-3;                                % [H]
%[text] ## ISOP converter: four modules
param.isop_rail.number_of_systems = 4;                      % modules in series at the input
param.isop_rail.pwr_nom = 250e3;                            % [W] nominal power of ONE module
% design input voltage of one module: nominal catenary voltage / number of modules (750 V)
param.isop_rail.application_voltage = param.isop_rail.catenary_nominal_voltage/param.isop_rail.number_of_systems;

% numbers of one module (were inside isop_rail_aux_application of the library)
hw_in.udc2 = 750;                           % [V] output voltage: turns ratio n2/n1 = udc2/application_voltage = 1
hw_in.n1 = 1;                               % primary turns (the transformer uses only the ratio)
hw_in.Lm = 1e-3;                            % [H] magnetizing inductance (primary)
hw_in.Rfe = 1e3;                            % [Ohm] iron losses (primary)
hw_in.Rs1 = 1e-3;                           % [Ohm] primary winding (secondary: Rs1/(n1/n2)^2)
hw_in.core_length = 1;                      % [m] only for the flux-density output of the transformer block
hw_in.core_mur = 70;
hw_in.ls_factor = 2.7;                      % leakage Ls = udc1^2/(8*fpwm*pwr_nom*ls_factor) = 26 uH, half per side
hw_in.Cdc_dc1 = 1.2e-3;                     % [F] input capacitor of each module (isop/fb_iso_mod_*/CFi_dc1)
hw_in.RCdc_dc1 = 1e-3;                      % [Ohm]
hw_in.Cdc_dc2 = hw_in.Cdc_dc1;              % [F] common output capacitor (isop/Cdc_dc2)
hw_in.RCdc_dc2 = 1e-4;                      % [Ohm]
hw_in.Ldc_dc1 = 1e-3;                       % [H] input inductor of the series string (isop/Ldc_dc1)
hw_in.RLdc_dc1 = 157*0.02*hw_in.Ldc_dc1;    % [Ohm]
hw_in.Ldc_dc2 = hw_in.Ldc_dc1/4;            % [H] output inductor of each rectifier (isop/Ldc_dc2, isop/Ldc_dc3)
hw_in.RLdc_dc2 = 157*0.01*hw_in.Ldc_dc2;    % [Ohm]
param.isop_rail.hw_in = hw_in;
param.isop_rail.hw = isop_rail_hw_param(param.isop_rail.application_voltage, param.isop_rail.pwr_nom, ...
    param.time.fpwm, hw_in);
clear hw_in

% load (root block Resistor, typed 0.5 before): 750 V / 0.5 Ohm = 1.125 MW = 4 x 281 kW (112 % of the modules)
param.load.rload = 0.5;                     % [Ohm]
%[text] ### Power semiconductors
% full bridges: ideal switches (library block full-bridge ideal switch based, gate threshold Vth, gate levels
% +15/-10 V typed in the modulator mask)
param.dev.used_device_ideal_switch = 'silicon_high_power_ideal_switch';
param.isop_rail.ideal_switch = device_ideal_switch_param(param.dev.used_device_ideal_switch, ...
    param.glb_time.fPWM_ISOP_RAIL, param.isop_rail.hw.udc1_nom);

% diode rectifiers (library block full_bridge_diode_rectifier, Simscape piecewise-linear diodes):
% ABB 5SDF 0131Z0401 (slow recovery), data of the legacy init; the block reads Vf, Rdon, Csnubber, Rsnubber
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
%[text] ## Control
% output voltage PI (ui_ctrl/isop_rail_ctrl/voltage control/pi_ctrl, limits 0..1 typed in the block): its
% output is the phase shift of the four bridges, in pu of pi (1 = square wave on the transformer).
% Output current PI (current control/pi_ctrl): computed, output not connected. Gains of the legacy init.
param.isop_rail.ctrl.kp_udc = 0.5;          % [pu/pu]
param.isop_rail.ctrl.ki_udc = 18;           % [1/s]
param.isop_rail.ctrl.kp_idc = 0.5;
param.isop_rail.ctrl.ki_idc = 18;
% output voltage reference in pu of udc2_bez = 750 V (block isop_rail_ctrl/Constant: its value was the
% default 1 of the block, applied from t = 0)
param.isop_rail.ctrl.udc_ref = 1;

% "DC transformer current" compensation (ui_ctrl/dc_trafo_current_measurements_and_compensation, values typed
% in the blocks before): mean of two samples of the input current of the string, low-pass filter, PI with
% reference 0. Its output is NOT connected: input 2 of the Sum in modulator/d1_m is open, so d1 stays 0.5 and
% these four values have no effect. Connected as it is, it would not work: it measures the input current of the
% string (a DC current by nature, not the DC part of the transformer current) and it would change d1 of both
% legs together, which gives no DC voltage on the transformer (see the README, open point 5).
param.isop_rail.ctrl.kp_idc_trafo = 0.1;    % [pu/pu]
param.isop_rail.ctrl.ki_idc_trafo = 2;      % [1/s] with ts = ts_dab (the block runs at 8 kHz)
param.isop_rail.ctrl.lim_idc_trafo = 0.1;   % [pu of d1] output limits +-lim
param.isop_rail.ctrl.tau_idc_trafo = 20e-3; % [s] low-pass filter, with ts = ts_dab
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;       % [pu] quantizer of the acquisitions (pu signals, 12 bit)
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;
param.meas.Imax_adc = 1049.835;             % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;
param.meas.Umax_adc = 1500;                 % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%[text] ### Design figures (informative)
% averaged model of the phase-shift full bridge (PSFB): rectified output of module k = n*D*v_k - Req*i_k, with
% D the effective duty (phase shift / pi) and Req = 4*n^2*Ls*fpwm the output resistance that models the
% duty-cycle loss of the leakage (Ls referred to the primary). Operating point: output at udc_ref, load
% rload, battery at its open-circuit voltage, modules balanced.
hw = param.isop_rail.hw; pk = param.battery.pack_1; c = param.isop_rail.ctrl; N = param.isop_rail.number_of_systems;
z = param.battery.initial_battery_soc;
d.u_ocv = pk.number_of_cells*(pk.E_1*exp(-pk.alpha*z) + pk.E0 + pk.E1*z + pk.E2*z^2 + pk.E3*z^3 + ...
    pk.Elog*log(abs(1 - z + 1e-2)));                            % [V] open-circuit voltage (cell model of the block)
d.n = hw.n2/hw.n1;
d.Ls = hw.Ld1 + hw.Ld2*(hw.n1/hw.n2)^2;                         % [H] leakage referred to the primary
d.Req = 4*d.n^2*d.Ls*hw.fpwm_base;                              % [Ohm] duty-cycle loss of one module (output side)
d.u_out = c.udc_ref*hw.udc2_bez;                                % [V]
d.i_out = d.u_out/param.load.rload;                             % [A] total load current
d.p_out = d.u_out*d.i_out;                                      % [W]
d.i_mod = d.i_out/N;                                            % [A] output current of one module
d.i_string = d.p_out/d.u_ocv;                                   % [A] input current of the string (lossless)
d.u_in = (d.u_ocv - d.i_string*(pk.R0 + pk.R1 + hw.RLdc_dc1))/N;   % [V] input voltage of one module
d.u_drop = 2*param.diode.rectifier.Vf + 2*param.isop_rail.ideal_switch.Rds_on*d.n*d.i_mod;   % [V] diodes, switches
d.D = (d.u_out + d.Req*d.i_mod + d.u_drop)/(d.n*d.u_in);        % effective duty = phase shift needed (pu of pi)
d.u_in_cat = param.isop_rail.catenary_nominal_voltage/N;        % [V] module input at the nominal catenary voltage
d.u_out_cat = d.n*d.u_in_cat - d.Req*d.i_mod - d.u_drop;        % [V] largest output at that voltage (D = 1)
% output filter: two rectifier branches (Req/2 + Ldc_dc2 each) in parallel on Cdc_dc2 and the load
L = hw.Ldc_dc2/2; C = hw.Cdc_dc2; R = param.load.rload; Rs = (d.Req/2 + hw.RLdc_dc2)/2;
d.f0_out = 1/(2*pi*sqrt(L*C));                                  % [Hz]
% voltage loop: phase shift (pu) -> output voltage (pu of udc2_bez), PI as continuous, delay 1.5 samples
f = logspace(-1, 3.5, 4000); s = 1j*2*pi*f; ts = param.glb_time.ts_isop_rail;
Zl = R./(1 + s*R*C);
G = d.n*d.u_in/hw.udc2_bez*Zl./(Zl + Rs + s*L).*exp(-1.5*ts*s);
Lv = (c.kp_udc + c.ki_udc./s).*G;
k = find(abs(Lv) < 1, 1);
d.fc_udc = f(k);                                                % [Hz] crossover of the voltage loop
d.pm_udc = 180 + angle(Lv(k))*180/pi;                           % [deg] phase margin
% input voltage sharing (common duty, outputs in parallel on the DC side): a module whose input voltage rises
% by dv gives more output current, draws more input current and discharges its capacitor; the deviation decays
% with tau = Cdc_dc1/g, g = (u_out/u_in)*(n*D/Req - i_mod/u_in) (linearized, first order, output inductors left
% out). In this model it holds between the two rectifiers (pairs 1+4 and 2+3). Inside a pair the two bridges
% are tied on the AC side through the leakage inductances: a voltage difference gives a reactive circulating
% current and almost no restoring power, so the sharing inside a pair is about 23 times slower and very
% sensitive to a leakage mismatch (see the README, open point 3).
d.g_ivs = d.u_out/d.u_in*(d.n*d.D/d.Req - d.i_mod/d.u_in);      % [S]
d.tau_ivs = hw.Cdc_dc1/d.g_ivs;                                 % [s]
param.isop_rail.design = d;
fprintf('Operating point: %.0f V, %.0f A, %.3f MW (%.0f kW and %.0f A per module) | battery %.0f V open circuit, %.0f V per module\n', ...
    d.u_out, d.i_out, d.p_out/1e6, d.p_out/N/1e3, d.i_mod, d.u_ocv, d.u_in);
fprintf('Phase-shift bridge: leakage %.1f uH -> Req = %.3f Ohm, duty-cycle loss %.0f V | phase shift needed %.3f\n', ...
    d.Ls*1e6, d.Req, d.Req*d.i_mod, d.D);
fprintf('Nominal catenary %.0f V: %.0f V per module, largest output %.0f V at this load (reference %.0f V)\n', ...
    param.isop_rail.catenary_nominal_voltage, d.u_in_cat, d.u_out_cat, d.u_out);
fprintf('Output filter %.0f Hz | voltage loop: crossover %.1f Hz, phase margin %.0f deg | input voltage sharing between the rectifiers: tau = %.2f ms (inside a pair much slower, see the README)\n', ...
    d.f0_out, d.fc_udc, d.pm_udc, d.tau_ivs*1e3);
clear hw pk c N z d L C R Rs f s ts Zl G Lv k
%[text] ## Model setup
open_system(model);
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
