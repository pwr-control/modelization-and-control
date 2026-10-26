%[text] # Three-leg interleaved buck converter
%[text] Initialization of interleaved_buck.slx (discrete blocks) and of interleaved_buck_compact.slx (compact
%[text] block interleaved_buck_3legs_mod1 of the library, built by build_interleaved_buck_compact.m): battery
%[text] on the DC link, three buck legs with one inductor each and interleaved carriers (0, 120, 240 deg),
%[text] common output capacitor, resistive load. Output voltage loop giving the total current reference, one
%[text] PI per leg on the leg current (reference i_ref/3: inherent current sharing), each leg current sampled
%[text] at the extremes of its own carrier (average value, no ripple), control step at the extremes of carrier 1
%[text] (double update, 8 kHz).
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, buck_ctrl_param, device_igbt_param, battery_param, check_model_param_refs). The library is
%[text] needed for the Simulink/Simscape blocks (modulator, bridge, DC link, battery) and for the compact block.
%[text] Run it from this folder (the functions of ./private are visible only to the files of this folder).
%[text] param.buck is also the mask struct p of the compact block (p.hw, p.pwm, p.ctrl): numeric fields only.

clear;
model = 'interleaved_buck';

% Simulink/Simscape library blocks (blocks, not parameters)
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
param.sim.simlength = 0.4;                  % [s] model StopTime
param.sim.tc = 0.625e-6;                    % [s] logging base, model MaxStep (ode23tb) and step of the
                                            %     Simscape local solver (glb_time.tc): ts/200 at 4 kHz, double update
param.sim.use_thermal_model = 1;            % IGBT bridge with the simple two-thermal-port loss model
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration
if param.sim.use_thermal_model
    param.sim.nonlinear_iterations = 5;
end
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
param.time.fpwm = 4e3;                              % [Hz] PWM frequency of each leg (12 kHz effective on the output)
param.time.trgo = 1;                                % double update: control at 2*fpwm, at the bottom and top of carrier 1
param.time.dead_time = 3e-6;                        % [s]
param.time.t_measure = 0.4;                         % [s] logging window
param.time.tc_factor = 1/param.time.fpwm/param.sim.tc/2;    % ts/tc, so that glb_time.tc = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;

% sample time of the battery state-of-charge estimator (legacy ts_dab of the shared template: 3*fpwm, double update)
param.time.fpwm_dab = 3*param.time.fpwm;            % [Hz]
param.time.trgo_dab = 1;

param.glb_time = timing_param(param.time);
%[text] ## Buck converter
%[text] ### Operating point and hardware
% 750 V battery to a 400 V DC bus, 250 kW: 625 A total, 208 A per leg, duty 0.533. Bases of the pu signals:
% ubez = u_out_nom (output voltage), ibez = total nominal current (a leg carries 1/3 pu).
param.buck.hw.n_legs = 3;
param.buck.hw.udc_nom = 750;                        % [V] DC link = battery
param.buck.hw.u_out_nom = 400;                      % [V] output
param.buck.hw.pwr_nom = 250e3;                      % [W]
param.buck.hw.ubez = param.buck.hw.u_out_nom;       % [V] base of the output voltage
param.buck.hw.ibez = param.buck.hw.pwr_nom/param.buck.hw.u_out_nom;    % [A] 625 A, base of all currents (total)

% leg inductors: one separate inductor per leg (a three-limb coupled reactor is wrong here: the three leg
% currents are DC and in phase, the core sees only the zero-sequence inductance). 1 mH: 47 A peak-peak of
% ripple per leg at D = 0.533 (22 % of the leg current), the three ripples add to 15 A peak-peak at 12 kHz
param.buck.hw.L = 1e-3;                             % [H] per leg
param.buck.hw.RL = 8e-3;                            % [Ohm] per leg: 350 W per leg at 208 A, 0.4 % of Pnom
param.buck.hw.kL = [1 1 1];                         % per-leg factors of L and RL (mismatch studies: the per-leg loops
param.buck.hw.kRL = [1 1 1];                        % keep the sharing, a common duty would not)
% output capacitor: 3.6 mF in three parts (the model has one capacitor Cout/3 with 3*RCout on each leg
% output, the compact block one capacitor Cout with RCout); 145 Hz resonance with L/3, zeta 0.24 at full load
param.buck.hw.Cout = 3.6e-3;                        % [F] total
param.buck.hw.RCout = 2e-3/3;                       % [Ohm] ESR of the total capacitor
% DC-link capacitor (block dclink_for_two_levels in the discrete model, inside the ssc block in the compact one)
param.buck.hw.Cdc = 7.2e-3;                         % [F]
param.buck.hw.Rdc = 1e-3;                           % [Ohm] ESR (compact block)
param.buck.hw.Vdc0 = param.buck.hw.udc_nom;         % [V] initial DC-link voltage (compact block)
% switches of the compact block (numerical switch + antiparallel diode, no losses): the discrete model has the
% IGBT bridge with the loss and thermal model (param.buck.igbt, param.thermal)
param.buck.hw.Vth = 2.5;                            % [V]   gate threshold
param.buck.hw.Ron = 1.5e-3;                         % [Ohm] on-state resistance (about the Vce_sat/I of the module at 200 A)
param.buck.hw.Goff = 1e-6;                          % [1/Ohm]

% design figures (informative)
hw = param.buck.hw;
d.D = hw.u_out_nom/hw.udc_nom;                                                 % duty at the operating point
d.i_leg = hw.ibez/hw.n_legs;                                                   % [A]
d.ripple_leg = hw.udc_nom*d.D*(1 - d.D)/(hw.L*param.time.fpwm);               % [A] peak-peak per leg
k = floor(hw.n_legs*d.D);                                                      % interleaving: ripple of the sum
d.ripple_tot = hw.udc_nom/(hw.L*param.time.fpwm)*(d.D - k/hw.n_legs)*((k + 1)/hw.n_legs - d.D)*hw.n_legs;   % [A] pk-pk at n_legs*fpwm
d.ripple_u_out = d.ripple_tot/(8*hw.n_legs*param.time.fpwm*hw.Cout);          % [V] peak-peak (capacitor only)
d.f0 = 1/(2*pi*sqrt(hw.L/hw.n_legs*hw.Cout));                                  % [Hz] LC resonance, common duty
d.R_load_nom = hw.u_out_nom^2/hw.pwr_nom;                                      % [Ohm]
d.zeta = sqrt(hw.L/hw.n_legs/hw.Cout)/(2*d.R_load_nom);                        % damping at nominal load
param.buck.hw.design = d;
fprintf('Buck: D = %.3f, %.0f A per leg, ripple %.0f A pk-pk per leg (%.0f %%), %.1f A pk-pk total at %.0f kHz, u_out ripple %.1f mV, f0 = %.0f Hz, zeta = %.2f at full load\n', ...
    d.D, d.i_leg, d.ripple_leg, d.ripple_leg/d.i_leg*100, d.ripple_tot, hw.n_legs*param.time.fpwm/1e3, d.ripple_u_out*1e3, d.f0, d.zeta);
clear hw d k
%[text] ### PWM
% triangle carriers, one per leg, shifted by 2*pi/n_legs (0, 120, 240 deg: the ripples of the three legs cancel at
% the fundamental and the output sees n_legs*fpwm). Polarity -1 of the library modulator: Q1 (upper switch, dc+ to
% the leg output) on while ramp <= m - dead_time*fpwm, Q2 on while ramp >= m + dead_time*fpwm: the ON pulse of Q1 is
% centred on the bottom of the carrier, the leg current is at its average value at the bottom and at the top.
% TRGO (control trigger) at the bottom and at the top of carrier 1 (double update); the TRGO of carriers 2 and 3
% sample-and-hold the currents of legs 2 and 3 and load their duties (shadow registers).
param.buck.pwm.tc = param.glb_time.tc;              % [s]   step of the modulator and of the compact block
param.buck.pwm.fpwm = param.glb_time.fPWM_AFE;      % [Hz]
param.buck.pwm.double_update = param.glb_time.TRGO_AFE_double_update;
param.buck.pwm.dead_time = param.glb_time.dead_time_AFE;    % [s]
param.buck.pwm.carrier_phase = [0 2*pi/3 4*pi/3];   % [rad] leg 1, 2, 3 (the error of the first version: pi/3, 2*pi/3)
param.buck.pwm.m_up = 0.98;                         %       limits of the duty
param.buck.pwm.m_low = 0.02;
param.buck.pwm.gate_on = 15;                        % [V]
param.buck.pwm.gate_off = -10;                      % [V]
param.buck.pwm.t_enable = 0;                        % [s]   compact block: gates off before this time
%[text] ### Control
% cascade: PI on u_out (pu of ubez) -> total current reference (pu of ibez, limited at +-i_lim, integrator
% clamped at the same limit) -> one PI per leg on the leg current sampled at the extremes of its own carrier,
% reference i_ref/n_legs -> duty of the leg (limited by the modulator at m_low..m_up, integrator clamped at the
% same limits). Gains from the crossover targets (private/buck_ctrl_param): 300 Hz for the current loops
% (fs = 8 kHz: 1.5 ts of delay on leg 1, 2.5 ts on legs 2 and 3, phase margins about 58 and 45 deg), 60 Hz for
% the voltage loop at nominal load, where the plant is the output capacitor with the load (corner 69 Hz) behind
% the closed common-mode current loop (gain 0.7 at that frequency: the capacitor reacts to a common duty), and
% about 100 Hz without load. Linear check of 2026-10-25 (claude_analysis/interleaved_buck_loop_analysis.py,
% discrete model at ts): current loops crossover 313 Hz with 59 deg of phase margin on leg 1, 42-44 deg and
% 7.5 dB of gain margin on legs 2-3; voltage loop 61 Hz / 121 deg at nominal load, 114 Hz / 94 deg at 5 % load;
% 50 ms reference ramp without overshoot (1 pu within 2 % at 80 ms), 5 % dip on a +0.2 pu load step recovered
% within 1 % in 40 ms. A 30 Hz target would halve the gains and double the dip and the recovery time.
param.buck.ctrl = buck_ctrl_param(param.buck.hw, param.glb_time.ts_afe, 300, 60);
param.buck.ctrl.ts = param.glb_time.ts_afe;         % [s]   control period (compact block)
param.buck.ctrl.mode = 0;                           %       compact block: 0 voltage loop, 1 total current reference (i_ref, t_ref, i_ref_slew)
param.buck.ctrl.u_out_ref = 1;                      % [pu]  output voltage reference (pu of ubez)
param.buck.ctrl.u_ref_slew = 20;                    % [pu/s] ramp of the reference at start: 1 pu in 50 ms
param.buck.ctrl.i_lim = 1.2;                        % [pu]  limit of the total current reference (750 A), also the integrator clamp
param.buck.ctrl.i_ref = 1;                          % [pu]  mode 1 of the compact block: total current reference
param.buck.ctrl.t_ref = 0;                          % [s]   step time of i_ref
param.buck.ctrl.i_ref_slew = 20;                    % [pu/s]
param.buck.ctrl.adc_q = 1/2^11;                     % [pu]  quantization of the acquisitions (compact block; param.meas.adc_quantization in the model)
param.buck.ctrl.k_time = 1;                         %       local clock of the compact block = master time
param.buck.ctrl.kp_align = 0;                       %       alignment PI of the local clock (compact block), off
param.buck.ctrl.ki_align = 0;
param.buck.ctrl.lim_up_align = 0;
param.buck.ctrl.lim_down_align = 0;
%[text] ### Module behaviour (clock mismatch)
param.buck.behav.time_gain_module_1 = 1;            % local clock of the module versus master time ("global timing"); 1: no mismatch
%[text] ## Power semiconductors
% 1200 V module for the 750 V link: Infineon FF1800R12IE5 (the data set of the proposal); the 1700 V
% CM1200DW-34T of the AFE models is the other data set of private/device_igbt_param
param.dev.used_device_igbt = 'infineon_FF1800R12IE5';
dev = device_igbt_param(param.dev.used_device_igbt, param.glb_time.fPWM_AFE, param.buck.hw.udc_nom);
param.dev.igbt_name = dev.name;
param.dev.igbt_type = dev.type;
param.buck.igbt = rmfield(dev, {'name', 'type'});   % numeric fields only: param.buck is the mask struct of the compact block
clear dev
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = param.buck.ctrl.adc_q;    % [pu] quantizer of the acquisitions (pu signals)
param.meas.adc16_quantization = 1/2^15;
param.meas.Imax_adc = 1049.835;                     % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;
param.meas.Umax_adc = 1500;                         % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%[text] ## Thermal model
% aluminium plate, liquid cooled, sized for PrimePACK2; liquid flow > 28 l/min. "A" (ambient) is the water:
% HA is the temperature rise from water to heatsink surface; water in/out rise max 5 K with 2 kW total losses
param.thermal.weight = 0.150;                                   % [kg]
param.thermal.no_weight = 0.150/10;                             % [kg] /10: thermal inertia not accounted
param.thermal.cp_al = 900;                                      % [J/(kg K)] specific heat, aluminium
param.thermal.heat_capacity_hs = param.thermal.cp_al * param.thermal.weight;   % [J/K]
param.thermal.thermal_conductivity_al = 160;                    % [W/(m K)] aluminium
param.thermal.Rth_switch_HA = 15/1000;                          % [K/W]
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;       % [K/W]
param.thermal.Tambient = 40;                                    % [degC] water temperature (mask of the bridge)
param.thermal.DThs_init = 0;                                    % [K]

% heatsink object read by the bridge mask (fields cp, no_weight, Rth_switch_HA, Rth_diode_HA)
param.thermal.heatsink.name = 'liquid_cooled_plate_2kw';
param.thermal.heatsink.weight = param.thermal.weight;
param.thermal.heatsink.no_weight = param.thermal.no_weight;
param.thermal.heatsink.cp = param.thermal.cp_al;
param.thermal.heatsink.heat_capacity = param.thermal.heat_capacity_hs;
param.thermal.heatsink.thermal_conductivity = param.thermal.thermal_conductivity_al;
param.thermal.heatsink.Rth_switch_HA = param.thermal.Rth_switch_HA;
param.thermal.heatsink.Rth_mosfet_HA = param.thermal.Rth_switch_HA;
param.thermal.heatsink.Rth_diode_HA = param.thermal.Rth_diode_HA;
param.thermal.heatsink.Tambient = param.thermal.Tambient;
param.thermal.heatsink.DThs_init = param.thermal.DThs_init;
%[text] ## Battery on the DC link
param.battery.nominal_battery_voltage_1 = param.buck.hw.udc_nom;  % [V] 750 V
param.battery.nominal_battery_power = param.buck.hw.pwr_nom;      % [W]
param.battery.initial_battery_soc = 0.85;
% ASSUMPTION: 1C rating, capacity = rated current x 1 h
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;   % [Ah]
param.battery.plot_ocv = false;

b = param.battery;
param.battery.pack_1 = battery_param(b.nominal_battery_voltage_1, b.nominal_battery_power, ...
    b.charge_capacity_1, b.initial_battery_soc, param.glb_time.ts_dab, b.plot_ocv);
clear b

% special settings for simulation (same as the AFE models)
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/200;
%[text] ## Load
param.load.R_load = param.buck.hw.design.R_load_nom;    % [Ohm] 0.64 Ohm: 250 kW at 400 V (block load/Resistor)
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
