%[text] # Single-phase dual active bridge, 250 kW (single\_phase\_dab)
%[text] Initialization of single_phase_dab.slx. The model is a bidirectional isolated DC/DC converter: a single-phase
%[text] dual active bridge (DAB) between two lithium-ion batteries. Side 1: battery_1 -> DC choke LFi_dc1 -> DC link
%[text] CFi_dc1 -> full bridge 1. Side 2: full bridge 2 -> DC link CFi_dc2 -> DC choke LFi_dc2 -> battery_2. The two
%[text] bridges (2 kV SiC MOSFET modules, library block full-bridge_mosfet_based_with_thermal_model_zvs) drive the
%[text] transformer single_phase_dab_transformer: one series capacitor on each side (DC blocking, resonance at
%[text] fPWM_DAB/5) and the library block single_phase_transformer (n = 1, leakage Ls/2 per side, Lm = 1 mH).
%[text] Modulation: phase shift between the two square waves at 12 kHz, 2 us dead time, both legs at 50 % duty.
%[text] Control (dab_modA/dab_ctrl/ui_ctrl, triggered twice per period, ts_dab = 41.7 us): one PI on the DC input
%[text] current of side 1 (the current of LFi_dc1, battery_1 to the DAB) plus a feed-forward of the reference;
%[text] output u in -1 ... 1, phase shift = u * phi_max (pi/2). Bench: the current reference is 0.1 pu and steps to
%[text] 1 pu at 0.15 s, through a 20 ms low-pass filter. The output voltage loop (Vout Ctrl) is in the model but
%[text] not selected (Variant Source).
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, dab_hw_param, device_mosfet_param, device_ideal_switch_param, battery_param, dab_ideal_op,
%[text] dab_harmonic_op, dab_harmonic_phi, check_model_param_refs, library_setup), called through the handles of
%[text] single_phase_dab_private.m (prv.<name>), as in ../three_phase_dab: the Live Editor does not see ./private.
%[text] The library is needed only for the Simulink/Simscape blocks and for the C sources listed in the
%[text] configuration of the model; prv.library_setup() puts it on the path.
%[text] The old flat init is init_model_legacy.m (kept until the merge). It no longer runs with the library:
%[text] lithium_ion_battery_setup now takes the charge capacity, and the battery block reads
%[text] battery_voltage_init_m from its mask.

clear;
model = 'single_phase_dab';
prv = single_phase_dab_private();   % handles to the functions of ./private (the Live Editor does not see ./private)
prv.library_setup();                % pwr-control/library on the path (blocks and C sources), cloned when missing

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
fprintf('%s init: functions of ./private, called as prv.<name>: %s\n', model, strjoin(fieldnames(prv)', ', '));
%[text] ## Simulation
param.sim.simlength = 2;                    % [s] model StopTime
param.sim.tc = 625e-9;                      % [s] logging base, MaxStep of the solver, step of the Simscape local
                                            % solver (legacy value: ts of the 4 kHz template domain / 200). At
                                            % 12 kHz: 133 steps per period, phase resolution 2.7 deg (see README)
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration (block battery_1/Solver Configuration).
                                            % Legacy: 3 because its flags use_mosfet_thermal_model and
                                            % use_thermal_model were 0 (5 when one of them is 1)
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
% one clock domain: the two bridges switch at fPWM_DAB (= frequency of the transformer); the modulator gives a
% control trigger at the peak and at the valley of its carrier (double update): control at 24 kHz, ts = 41.7 us
param.time.fpwm_dab = 12e3;                 % [Hz] legacy: 3*fpwm with fpwm = 4 kHz
param.time.trgo_dab = 1;                    % double update
param.time.dead_time_dab = 2e-6;            % [s] dead time of each leg (applied by the modulator)
param.time.tc = param.sim.tc;
param.time.t_measure = param.sim.simlength; % [s] logging window: the whole simulation (legacy)
param.time.tc_decimation = 1;
param.glb_time = prv.timing_param(param.time);
%[text] ## DAB
param.dab.pwr_nom = 250e3;                  % [W] nominal power
param.dab.application_voltage = 690;        % [V] legacy selector of single_phase_dab_hwdata: 690 -> data set DAB_1200V
param.dab.fres = param.glb_time.fPWM_DAB/5; % [Hz] resonance of the series capacitors with the series inductance
%[text] ### Hardware
% numbers of the data set DAB_1200V (were inside single_phase_dab_hwdata of the library)
hw_in.udc1 = 1200;                          % [V] nominal DC voltage of side 1 (normalization, design)
hw_in.udc2 = 1200;                          % [V] side 2
hw_in.uac1 = 1000;                          % [V] (stored, not used by the model)
hw_in.uac2 = 1000;
hw_in.idc1 = 250;                           % [A] nominal DC current of side 1: end scale of the current feedback
hw_in.idc2 = 250;
hw_in.Lm = 1e-3;                            % [H] magnetizing inductance (primary)
hw_in.n1 = 6;                               % turns: the transformer block uses only the ratio n1/n2 = 1
hw_in.n2 = 6;
hw_in.Rfe = 1e3;                            % [Ohm] iron losses (primary, in parallel with Lm)
hw_in.Rs1 = 1e-3;                           % [Ohm] primary winding (secondary: Rs1/(n1/n2)^2)
hw_in.core_length = 1;                      % [m] only for the flux-density output of the transformer block
hw_in.core_mur = 75;
hw_in.Cdc_dc1 = 3.6e-3;                     % [F] DC link of side 1 (CFi_dc1)
hw_in.Cdc1_dc1 = 2*hw_in.Cdc_dc1;           % split banks: stored, not used by the model
hw_in.Cdc2_dc1 = 2*hw_in.Cdc_dc1;
hw_in.Cdc_dc2 = 3.6e-3;                     % [F] DC link of side 2 (CFi_dc2)
hw_in.Cdc1_dc2 = 2*hw_in.Cdc_dc2;
hw_in.Cdc2_dc2 = 2*hw_in.Cdc_dc2;
hw_in.RCdc_dc1 = 1e-3;                      % [Ohm] ESR of the DC links
hw_in.RCdc1_dc1 = hw_in.RCdc_dc1/2;
hw_in.RCdc2_dc1 = hw_in.RCdc_dc1/2;
hw_in.RCdc_dc2 = 1e-3;
hw_in.RCdc1_dc2 = hw_in.RCdc_dc2/2;
hw_in.RCdc2_dc2 = hw_in.RCdc_dc2/2;
hw_in.Ldc_dc1 = 250e-6;                     % [H] DC choke of side 1 (LFi_dc1)
hw_in.RLdc_dc1 = 157*0.05*hw_in.Ldc_dc1;    % [Ohm] 1.96 mOhm
hw_in.Ldc_dc2 = 250e-6;                     % [H] DC choke of side 2 (LFi_dc2)
hw_in.RLdc_dc2 = 157*0.05*hw_in.Ldc_dc2;
param.dab.hw_in = hw_in;
param.dab.hw = prv.dab_hw_param('DAB_1200V', param.dab.pwr_nom, param.glb_time.fPWM_DAB, param.dab.fres, hw_in);
clear hw_in
%[text] ### Modulator and control
% dab_ctrl/dab_single_phase_pwm_modulator: phase shift of bridge 2 = u * phi_max (typed pi/2 before)
param.dab.mod.phi_max = pi/2;               % [rad] u = 1 gives 90 deg, the largest power
% feed-forward of the current reference (ui_ctrl/dab_ctrl/current control/Gain, default 1 of the block before):
% u = k_ff * i_ref + PI. With 1 the feed-forward asks about twice the phase shift needed (0.5 gives 45 deg at
% 1 pu, see the design figures below and the README). Kept at the legacy value.
param.dab.mod.k_ff = 1;
% PI gains (library pi_ctrl, ts = ts_dab, limits -1 ... 1 for the current PI and -2 ... 2 for the voltage PI,
% typed in the blocks). Legacy: ctrl_dab_setup(0.5, 18, 0.5, 18), then ki_udc = 35.
param.dab.ctrl.kp_idc = 0.5;                % current PI (active): [pu of u / pu of current]
param.dab.ctrl.ki_idc = 18;                 % [1/s]
param.dab.ctrl.kp_udc = 0.5;                % output voltage PI (Vout Ctrl, not selected by the Variant Source)
param.dab.ctrl.ki_udc = 35;
% bench (ui_ctrl/dab_ctrl/Step6 and discrete-time-low-pass-filter6, typed before): current reference in pu of
% Idc_FS (250 A, input current of side 1, positive from battery_1 to battery_2)
param.dab.bench.i_ref0 = 0.1;               % [pu] from t = 0
param.dab.bench.t_step = 0.15;              % [s]
param.dab.bench.i_ref1 = 1;                 % [pu] from t_step to the end
param.dab.bench.tau_ref = 0.02;             % [s] low-pass filter of the reference (ts = ts_dab)
%[text] ### Power semiconductors
% both bridges: 2 kV SiC MOSFET half-bridge module (active blocks full-bridge_mosfet_based_with_thermal_model_zvs);
% the ideal-switch bridges are in the model but commented
param.dev.used_device_mosfet = 'danfoss_SKM1700MB20R4S2I4';
param.dev.used_device_ideal_switch = 'silicon_high_power_ideal_switch';
param.dab.mosfet = prv.device_mosfet_param(param.dev.used_device_mosfet, param.glb_time.fPWM_DAB, ...
    param.dab.hw.udc1_nom);
param.dab.ideal_switch = prv.device_ideal_switch_param(param.dev.used_device_ideal_switch, ...
    param.glb_time.fPWM_DAB, param.dab.hw.udc1_nom);
%[text] ## Batteries (side 1 and side 2)
% legacy: nominal voltage = hwdata.afe.udc_nom of the 690 V AFE data set of the template (1070 V), on both sides
param.battery.nominal_battery_voltage_1 = 1070;            % [V]
param.battery.nominal_battery_voltage_2 = 1070;            % [V]
param.battery.nominal_battery_power = 250e3;               % [W]
param.battery.initial_battery_soc = 0.85;
% lithium_ion_battery_setup now takes the charge capacity [Ah]. ASSUMPTION: 1C rating, capacity = rated current
% x 1 h (234 Ah; the old library function had 50 Ah fixed)
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;
param.battery.charge_capacity_2 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_2;
param.battery.plot_ocv = false;
b = param.battery;
param.battery.pack_1 = prv.battery_param(b.nominal_battery_voltage_1, b.nominal_battery_power, ...
    b.charge_capacity_1, b.initial_battery_soc, param.glb_time.ts_dab, b.plot_ocv);
param.battery.pack_2 = prv.battery_param(b.nominal_battery_voltage_2, b.nominal_battery_power, ...
    b.charge_capacity_2, b.initial_battery_soc, param.glb_time.ts_dab, b.plot_ocv);
clear b
% legacy changes for the simulation: stiffer battery, faster RC branch
for k = 1:2
    f = sprintf('pack_%d', k);
    param.battery.(f).R0 = param.battery.(f).R0/2;
    param.battery.(f).R1 = param.battery.(f).R1/2;
    param.battery.(f).C1 = param.battery.(f).C1/50;
end
% open-circuit voltage at the initial state of charge (same expression as the cell of the library block, cells
% in series): initial voltage of the battery terminals (mask parameter battery_voltage_init_m of each battery
% block; without it both blocks read the mask default param.battery.nominal_battery_voltage_1)
for k = 1:2
    b = param.battery.(sprintf('pack_%d', k));
    z = b.soc_init;
    param.battery.(sprintf('ocv_%d', k)) = b.number_of_cells*(b.E_1*exp(-b.alpha*z) + b.E0 + b.E1*z + ...
        b.E2*z^2 + b.E3*z^3 + b.Elog*log(abs(1 - z + 1e-2)));
end
% initial voltage of the DC links CFi_dc1, CFi_dc2 (priority High in the model). NEW: the DC links start at the
% open-circuit voltage of their battery, as in ../three_phase_dab. The legacy model started them at 0 V: an
% inrush of about 3 kA through the DC chokes (250 uH - 3.6 mF, 168 Hz, lightly damped) and 1.7 kV on the DC
% links in the first 3 ms, which the current loop also sees. Set both to 0 for the legacy start.
param.dab.ic.vc_dc1 = param.battery.ocv_1;  % [V]
param.dab.ic.vc_dc2 = param.battery.ocv_2;  % [V]
fprintf(['Batteries: %.0f V nominal, open circuit %.1f V / %.1f V at soc %.2f, R0 = %.1f mOhm, R1 = %.1f mOhm, ' ...
    'C1 = %.1f mF (after the legacy changes); DC links start at %.0f V / %.0f V\n'], ...
    param.battery.nominal_battery_voltage_1, param.battery.ocv_1, param.battery.ocv_2, ...
    param.battery.initial_battery_soc, param.battery.pack_1.R0*1e3, param.battery.pack_1.R1*1e3, ...
    param.battery.pack_1.C1*1e3, param.dab.ic.vc_dc1, param.dab.ic.vc_dc2);
clear k f b z
%[text] ## Sensors quantization
param.meas.adc_quantization = 1/2^11;       % [pu] quantizer of the feedbacks (pu signals, 12 bit)
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;
%[text] ## Thermal model
% aluminium plate, liquid cooled (library liquid_cooled_plate_2kw_setup, same fields); "A" (ambient) is the
% water: HA is the temperature rise from the water to the heatsink surface
param.thermal.weight = 0.150;                               % [kg]
param.thermal.no_weight = 0.150/10;                         % [kg] /10: thermal inertia not accounted
param.thermal.cp_al = 900;                                  % [J/(kg K)] specific heat, aluminium
param.thermal.heat_capacity_hs = param.thermal.cp_al*param.thermal.weight;     % [J/K]
param.thermal.thermal_conductivity_al = 160;                % [W/(m K)] aluminium
param.thermal.Rth_switch_HA = 15/1000;                      % [K/W]
param.thermal.Rth_mosfet_HA = param.thermal.Rth_switch_HA;  % [K/W]
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;   % [K/W]
param.thermal.Tambient = 40;                                % [degC] water temperature
param.thermal.DThs_init = 0;                                % [K]
t = param.thermal;
param.thermal.heatsink = struct('name', 'liquid_cooled_plate_2kw', 'weight', t.weight, 'no_weight', t.no_weight, ...
    'cp', t.cp_al, 'heat_capacity', t.heat_capacity_hs, 'thermal_conductivity', t.thermal_conductivity_al, ...
    'Rth_switch_HA', t.Rth_switch_HA, 'Rth_mosfet_HA', t.Rth_mosfet_HA, 'Rth_diode_HA', t.Rth_diode_HA, ...
    'Tambient', t.Tambient, 'DThs_init', t.DThs_init);
clear t
%[text] ### Design figures (informative)
% (1) lossless DAB at the design voltages 1200 V / 1200 V (series inductance only). (2) bench point at 1 pu
% (250 A out of battery_1): harmonic model of the real tank (series capacitors, windings, 2 x Rds_on, Lm, Rfe),
% DC voltages = open-circuit voltages -/+ the drop on the battery resistances; u needed = phi/phi_max, against
% the feed-forward k_ff * 1. Switching currents > 0: the current charges and discharges the leg capacitances
% (ZVS); the model has no Coss, the transitions are instantaneous.
hw = param.dab.hw; ms = param.dab.mosfet; n12 = hw.n1/hw.n2; fs = param.glb_time.fPWM_DAB;
L = hw.Ld1 + hw.Ld2*n12^2;
op = prv.dab_ideal_op(hw.udc1_nom, hw.udc2_nom*n12, fs, L, param.dab.pwr_nom);
fprintf(['Design point %.0f V / %.0f V, %.0f kW: phi = %.1f deg (lossless), Pmax = %.0f kW at 90 deg, %.0f A rms, ' ...
    '%.0f A peak, switching currents %.0f / %.0f A\n'], hw.udc1_nom, hw.udc2_nom, param.dab.pwr_nom/1e3, ...
    op.phi*180/pi, op.Pmax/1e3, op.Irms, op.Ipk, op.i_sw1, op.i_sw2);
pk1 = param.battery.pack_1; pk2 = param.battery.pack_2;
i1 = param.dab.bench.i_ref1*hw.Idc_FS;                       % [A] DC current out of battery_1 at 1 pu
v1 = param.battery.ocv_1 - i1*(pk1.R0 + pk1.R1 + hw.RLdc_dc1);
i2 = i1*v1/param.battery.ocv_2;                              % [A] into battery_2 (losses left out)
v2 = param.battery.ocv_2 + i2*(pk2.R0 + pk2.R1 + hw.RLdc_dc2);
p = struct('V1', v1, 'V2', v2, 'n1', hw.n1, 'n2', hw.n2, 'fs', fs, 'R1', hw.Rs1 + 2*ms.Rds_on, 'L1', hw.Ld1, ...
    'C1', hw.Cs1, 'R2', hw.Rs2 + 2*ms.Rds_on, 'L2', hw.Ld2, 'C2', hw.Cs2, 'Lm', hw.Lm, 'Rfe', hw.Rfe);
[phi, o] = prv.dab_harmonic_phi(p, i1*v1, 499, 1000);
fprintf(['Bench at %.2f pu: %.0f A, %.0f V / %.0f V (d = %.3f), %.0f kW: phi = %.1f deg -> u = %.2f (feed-forward ' ...
    'k_ff*i_ref = %.2f), %.0f A rms, %.0f A peak, switching currents %.0f / %.0f A, magnetizing %.0f A peak\n'], ...
    param.dab.bench.i_ref1, i1, v1, v2, v2*n12/v1, i1*v1/1e3, phi*180/pi, phi/param.dab.mod.phi_max, ...
    param.dab.mod.k_ff*param.dab.bench.i_ref1, o.I1rms, o.I1pk, o.i_sw1, o.i_sw2, o.Im_pk);
param.dab.design = struct('L', L, 'ideal', op, 'bench_V1', v1, 'bench_V2', v2, 'bench_phi', phi, ...
    'bench_u', phi/param.dab.mod.phi_max, 'bench_I1rms', o.I1rms, 'bench_i_sw1', o.i_sw1, 'bench_i_sw2', o.i_sw2);
clear hw ms n12 fs L op pk1 pk2 i1 v1 i2 v2 p phi o
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
    [missing_refs, model_refs] = prv.check_model_param_refs(model, param);
    if isempty(missing_refs)
        fprintf('%s init: all %d param.* references of the model blocks resolve\n', model, numel(model_refs));
    else
        warning('%s init: %d param.* references of the model blocks do not exist in param:', model, numel(missing_refs));
        disp(missing_refs);
    end
    clear missing_refs model_refs
end
