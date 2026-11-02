%[text] # three\_phase\_dab: model initialization (param structure by subsystem)
% Rev. 02, 2026-10-29: three-phase DAB (DAB3), 1200 V -> 1200 V, 250 kW, fs = 10 kHz (numbers in
% ./private/dab3_param.m, sizing in dab3_sizing.md). Everything the model needs is in the struct "param",
% one group per subsystem:
%   param.sim       simulation settings (bench case, stop time, Simscape local solver step, nonlinear iterations)
%   param.time      timing settings shared by all stages, and the PWM data of the stages not in this model
%   param.dab3      DAB3: pwr_nom, fs, .pwm, .hw_in (the hardware numbers, kept here and not in the
%                   library), .tank, .mod, .ctrl_in, .bench, .hw (hw_dab_three_phase_setup object), .ctrl
%                   (ctrl_dab_setup object), .mosfet, .ideal_switch, .ic (initial voltages of the DC links)
%   param.glb_time  timing_setup object (built from param.time and param.dab3.pwm)
%   param.battery   battery inputs;                    param.battery.pack_1/2  lithium_ion_battery_setup structs,
%                   param.battery.ocv_1/2  open circuit voltages (battery_voltage_init_m of the two battery blocks)
%   param.meas      ADC quantization
%   param.thermal   heatsink inputs;                   param.thermal.heatsink  liquid_cooled_plate_2kw_setup object
%   param.dev       names of the semiconductor datasets
%
% Library dependence: the library provides classes and algorithms (timing_setup, hw_dab_three_phase_setup,
% ctrl_dab_setup, device_mosfet_setup, device_ideal_switch_setting, lithium_ion_battery_setup,
% liquid_cooled_plate_2kw_setup) and the blocks of three_phase_dab.slx (three_phase_dab_modulator,
% three_phase_inverter_mosfet_based_with_thermal_model, single_phase_transformer, lithium_ion_battery, pi_ctrl,
% discrete_time_low_pass_filter); the model's numbers stay in this folder. The functions of ./private (the
% numbers, the helpers, and any function of the library that must be frozen or changed for this model only,
% with the same name) are called through the handles made by dab3_private.m: prv.<name>(...). The Live Editor
% runs a live script from a temporary copy outside this folder, where ./private is out of scope ("Unrecognized
% function or variable 'library_setup'"); the handles keep the scope of the function that made them, so the
% script runs the same with Run and from the Command Window, and a section runs on its own once prv is in the
% workspace. The list of the files of ./private is printed at every run. The first call, prv.library_setup()
% (./private/library_setup.m), adds the library to the path with its subfolders: it uses the copy already on
% the path, else the folder "library" next to this repository (or the one named by the environment variable
% PWR_CONTROL_LIBRARY), and clones https://github.com/pwr-control/library when neither exists.

clear;
prv = dab3_private();       % handles to the functions of ./private (dab3_private.m): the Live Editor does not see ./private
prv.library_setup();        % pwr-control/library on the path, cloned next to this repository when missing
[model, options] = init_environment('three_phase_dab');
param = struct();
fprintf('%s init: functions of ./private, called as prv.<name>: %s\n', model, strjoin(fieldnames(prv)', ', '));

% save('sim_results_01.mat', '-v7.3', '-regexp', '_sim$', '^param$');
% then dab3_run_check: powers, switching currents and losses of the logged window against the sizing
%[text] ## Simulation
param.sim.bench_case = 'mismatch';           % 'forward' (run 01), 'reverse', 'light', 'mismatch': current
                                            % reference and batteries of ./private/dab3_param.m
param.sim.simlength = 0.3;                  % [s] model StopTime ('forward': reverse power, then reversal at
                                            % 0.1 s, forward power at 1 pu from 0.2 s)
param.sim.tc = 100e-9;                      % [s] logging base, model MaxStep and Simscape local solver step
                                            % (glb_time.tc): 1000 steps per period at 10 kHz, phase resolution
                                            % 0.36 deg (1 % of the power at 30 deg). 500 ns for quick
                                            % structural checks; the run time scales with 1/tc
param.sim.nonlinear_iterations = 5;         % Simscape solver (MOSFET bridges with thermal model)
%[text] ## Timing
param.time.fpwm = 4e3;                              % [Hz] base PWM frequency of the shared timing object
param.time.t_measure = 0.02;                        % [s] logged window: the last 20 ms of the run (200 periods)
param.time.tc_factor = 1/param.time.fpwm/param.sim.tc/2;    % ts_afe/tc, so that glb_time.tc = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;
param.time.trgo_th_generator = 0.025;

% stages not in this model, needed by the shared timing object only
param.time.fpwm_afe = param.time.fpwm;
param.time.trgo_afe = 1;                            % double update
param.time.dead_time_afe = 3e-6;
param.time.fpwm_inv = param.time.fpwm;
param.time.trgo_inv = 1;                            % double update
param.time.dead_time_inv = 3e-6;
param.time.fpwm_single_phase_inv = param.time.fpwm;
param.time.trgo_single_phase_inv = 1;               % double update
param.time.dead_time_single_phase_inv = 3e-6;
param.time.fpwm_psfbc = param.time.fpwm;
param.time.trgo_psfbc = 1;                          % double update
param.time.dead_time_psfbc = 3e-6;
param.time.fpwm_isop_rail = param.time.fpwm;
param.time.trgo_isop_rail = 1;                      % double update
param.time.dead_time_isop_rail = 3e-6;
param.time.fpwm_cllc = 3*param.time.fpwm;
param.time.trgo_cllc = 0;
param.time.dead_time_cllc = 2e-6;
%[text] ### DAB3 PWM and bench
% 1200 V -> 1200 V, 250 kW: the local numbers (pwr_nom, fs, pwm, hw_in, tank, mod, ctrl_in, bench) are in
% ./private/dab3_param.m (fs = 10 kHz, fPWM_DAB = 6 fs = 60 kHz, single update)
param.dab3 = prv.dab3_param(param.sim.bench_case);
fprintf('DAB3 bench "%s": %.2f pu from %.2f s, %.2f pu from %.2f s (to the end), soc %.2f / %.2f\n', ...
    param.dab3.bench.case, param.dab3.bench.i_ref1, param.dab3.bench.t_on, param.dab3.bench.i_ref2, ...
    param.dab3.bench.t_rev, param.dab3.bench.soc_1, param.dab3.bench.soc_2);
%[text] ### Global timing object
t = param.time;
d = param.dab3.pwm;
param.glb_time = timing_setup(t.fpwm_afe, t.trgo_afe, t.fpwm_inv, t.trgo_inv, ...
    t.fpwm_single_phase_inv, t.trgo_single_phase_inv, d.fpwm, d.trgo, ...
    t.fpwm_psfbc, t.trgo_psfbc, t.fpwm_isop_rail, t.trgo_isop_rail, t.fpwm_cllc, t.trgo_cllc, ...
    t.t_measure, t.tc_factor, t.tc_decimation, t.delay_pwm, t.dead_time_afe, t.dead_time_inv, ...
    t.dead_time_single_phase_inv, d.dead_time, t.dead_time_psfbc, t.dead_time_isop_rail, t.dead_time_cllc);
clear t d
fprintf('DAB3 timing: fs = %.1f kHz, fPWM_DAB = %.1f kHz, ts_dab = %.2f us, tc = %.0f ns (%d steps per period)\n', ...
    param.dab3.fs/1e3, param.glb_time.fPWM_DAB/1e3, param.glb_time.ts_dab*1e6, param.glb_time.tc*1e9, ...
    round(1/param.dab3.fs/param.glb_time.tc));
%[text] ## DAB3
%[text] ### Hardware
% The numbers are the model's own (param.dab3.hw_in), the library provides only the setup class
% hw_dab_three_phase_setup, which derives a series inductance Ls = udc1^2/(4 fPWM_DAB P) (and a series
% capacitor Cs from the fres argument: the DAB3 has none, the model does not use it) and the normalization data.
h = param.dab3.hw_in;
param.dab3.hw = hw_dab_three_phase_setup(h.name, param.dab3.pwr_nom, h.udc1, h.udc2, h.uac1, h.uac2, ...
    h.idc1, h.idc2, param.glb_time.fPWM_DAB, param.dab3.fs/10, ...
    h.n1, h.n2, h.Lm, h.Rfe, h.Rs1, h.core_length, h.core_mur, h.Cdc_dc1, h.Cdc1_dc1, h.Cdc2_dc1, h.Cdc_dc2, ...
    h.Cdc1_dc2, h.Cdc2_dc2, h.RCdc_dc1, h.RCdc1_dc1, h.RCdc2_dc1, h.RCdc_dc2, h.RCdc1_dc2, h.RCdc2_dc2, ...
    h.Ldc_dc1, h.Ldc_dc2, h.RLdc_dc1, h.RLdc_dc2);
clear h
% series inductance of the sizing (28 uH per phase, 14 + 14 uH) written over the one of the class.
% Same layout as the class: Ld1 = Ls1 on the primary, Ld2 = Ls2 = Ld1/n^2 on the secondary, Ls the total.
t = param.dab3.tank;
param.dab3.hw.Ld1 = t.Ld1;
param.dab3.hw.Ls1 = t.Ld1;
param.dab3.hw.Ld2 = t.Ld2;
param.dab3.hw.Ls2 = t.Ld2;
param.dab3.hw.Ls = t.L;
clear t
% the current loop normalizes the DC input current of side 1 (dab_current_input_modA) by idc1_bez (208 A)
param.dab3.hw.Idc_FS = param.dab3.hw.idc1_nom;
param.dab3.hw.displayInfo();

hw = param.dab3.hw;
n12 = hw.n1/hw.n2;
op = prv.dab3_ideal_op(hw.udc1_nom, hw.udc2_nom*n12, param.dab3.fs, hw.Ld1 + hw.Ld2*n12^2, param.dab3.pwr_nom);
lam = hw.udc1_nom/(9*param.dab3.fs);                % [Vs] peak flux linkage of the six-step phase voltage
fprintf('DAB3 series inductance: Ld1 = %.1f uH, Ld2 = %.1f uH, L = %.1f uH per phase (primary)\n', ...
    1e6*hw.Ld1, 1e6*hw.Ld2, 1e6*(hw.Ld1 + hw.Ld2*n12^2));
fprintf('         phi(Pnom) = %.1f deg (ideal), Pmax = %.0f kW at 90 deg, phi_max = %.0f deg, k_ff = %.3f\n', ...
    rad2deg(op.phi), op.Pmax/1e3, rad2deg(param.dab3.mod.phi_max), param.dab3.mod.k_ff);
fprintf('Transformer: n1 = n2 = %d, B = %.3f T peak, Im = %.1f A peak (ideal), Lm = %.2f mH, Rfe = %.1f kOhm\n', ...
    hw.n1, lam/(hw.n1*param.dab3.hw_in.core_area), lam/hw.Lm, 1e3*hw.Lm, hw.Rfe/1e3);
clear hw n12 op lam
%[text] ### Control
% PI gains of the current loop (feedback: DC input current of side 1, behind the DC choke) and of the output
% voltage loop (not active); see ./private/dab3_param.m and dab3_sizing.md
c = param.dab3.ctrl_in;
param.dab3.ctrl = ctrl_dab_setup(c.kp_udc, c.ki_udc, c.kp_idc, c.ki_idc);
clear c
%[text] ### Power semiconductors
% Both bridges: 2 kV SiC MOSFET half-bridge module (Danfoss SKM1700MB20R4S2I4, 1.04 mOhm, Coss 3.28 nF). The
% fpwm argument (ZVS snubber of the _zvs bridges only) is the switching frequency of each device, fs.
param.dev.used_device_mosfet = 'danfoss_SKM1700MB20R4S2I4';
param.dev.used_device_ideal_switch = 'silicon_high_power_ideal_switch';     % commented-out bridges of the model
param.dab3.mosfet = device_mosfet_setup(param.dev.used_device_mosfet, param.dab3.fs, param.dab3.hw.udc1_nom);
param.dab3.ideal_switch = device_ideal_switch_setting(param.dev.used_device_ideal_switch, param.dab3.fs, ...
    param.dab3.hw.udc1_nom);
% ZVS check at the nominal point (harmonic model, ./private/dab3_harmonic_op.m): the commutating current
% charges and discharges the two Coss of one leg. The library modulator applies no dead time and the bridges
% have no Coss: the model shows the switching currents, not the transitions.
hw = param.dab3.hw;
n12 = hw.n1/hw.n2;
op = prv.dab3_ideal_op(hw.udc1_nom, hw.udc2_nom*n12, param.dab3.fs, hw.Ld1 + hw.Ld2*n12^2, param.dab3.pwr_nom);
p = struct('V1', hw.udc1_nom, 'V2', hw.udc2_nom, 'n1', hw.n1, 'n2', hw.n2, 'fs', param.dab3.fs, ...
    'L1', hw.Ld1, 'R1', hw.Rs1 + param.dab3.mosfet.Rds_on, 'L2r', hw.Ld2*n12^2, ...
    'R2r', (hw.Rs2 + param.dab3.mosfet.Rds_on)*n12^2, 'Lm', hw.Lm, 'Rfe', hw.Rfe);
o = prv.dab3_harmonic_op(p, op.phi, 1800, 1001);
c_leg = 2*param.dab3.mosfet.Coss;
fprintf('Nominal point: I1 = %.0f A rms, %.0f A peak; i_sw1 = %.0f A, i_sw2 = %.0f A; transitions %.0f / %.0f ns, dead time %.0f ns\n', ...
    o.I1rms, o.I1pk, o.i_sw1, o.i_sw2, 1e9*c_leg*hw.udc1_nom/o.i_sw1, 1e9*c_leg*hw.udc2_nom/o.i_sw2, ...
    1e9*param.dab3.pwm.dead_time);
clear hw n12 op p o c_leg
%[text] ## Batteries (side 1 and side 2)
% battery_1 on side 1 and battery_2 on side 2, both 1200 V and sized on the DAB3 power; the current loop sets
% the current of battery_1 (param.dab3.bench)
param.battery.nominal_battery_voltage_1 = param.dab3.hw.udc1_nom;  % [V] 1200 V
param.battery.nominal_battery_voltage_2 = param.dab3.hw.udc2_nom;  % [V] 1200 V
param.battery.nominal_battery_power = param.dab3.pwr_nom;          % [W]
param.battery.initial_battery_soc_1 = param.dab3.bench.soc_1;      % 0.85: open circuit voltage about 1248 V
param.battery.initial_battery_soc_2 = param.dab3.bench.soc_2;
% lithium_ion_battery_setup takes the charge capacity [Ah] as third argument. ASSUMPTION: 1C rating,
% capacity = rated current x 1 h
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;
param.battery.charge_capacity_2 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_2;

% Kalman sample time of the battery blocks: in the model they read glb_time.ts_dab
b = param.battery;
param.battery.pack_1 = lithium_ion_battery_setup(b.nominal_battery_voltage_1, b.nominal_battery_power, ...
    b.charge_capacity_1, b.initial_battery_soc_1, param.glb_time.ts_dab);
param.battery.pack_2 = lithium_ion_battery_setup(b.nominal_battery_voltage_2, b.nominal_battery_power, ...
    b.charge_capacity_2, b.initial_battery_soc_2, param.glb_time.ts_dab);
clear b

% special settings for simulation: stiffer battery, faster RC branch
for k = 1:2
    f = sprintf('pack_%d', k);
    param.battery.(f).R0 = param.battery.(f).R0/2;
    param.battery.(f).R1 = param.battery.(f).R1/2;
    param.battery.(f).C1 = param.battery.(f).C1/200;
end

% open circuit voltage at the initial state of charge (same expression as the cell SSC of the library block,
% number_of_cells in series): initial voltage of the capacitor at the battery terminals (battery_voltage_init_m)
% and of the DC links CFi_dc1, CFi_dc2 (param.dab3.ic, priority High). The run starts at no load near the
% steady state, without the inrush of the empty DC links through the DC chokes (250 uH - 500 uF, lightly damped)
for k = 1:2
    b = param.battery.(sprintf('pack_%d', k));
    z = b.soc_init;
    param.battery.(sprintf('ocv_%d', k)) = b.number_of_cells*(b.E_1*exp(-b.alpha*z) + b.E0 + b.E1*z + ...
        b.E2*z^2 + b.E3*z^3 + b.Elog*log(abs(1 - z + 1e-2)));
end
param.dab3.ic.vc_dc1 = param.battery.ocv_1;                        % [V] about 1248 V at soc 0.85
param.dab3.ic.vc_dc2 = param.battery.ocv_2;
fprintf('Batteries: open circuit voltage %.0f V / %.0f V (soc %.2f / %.2f), R0 = %.0f mOhm; DC links start at the same voltage\n', ...
    param.battery.ocv_1, param.battery.ocv_2, param.battery.initial_battery_soc_1, ...
    param.battery.initial_battery_soc_2, 1e3*param.battery.pack_1.R0);
clear k f b z
%[text] ## Sensors quantization
param.meas.adc_quantization = 1/2^11;
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;
%[text] ## Thermal model
% aluminium plate, liquid cooled; "A" (ambient) is the water: HA is the temperature rise from water to
% heatsink surface; water in/out rise max 5 K with 2 kW total losses
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
%[text] ## Model setup
open_system(model);
%[text] ### Scopes closed at start
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":8.4}
%---
