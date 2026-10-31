%[text] # Single-phase three-level T-type bridge on the grid (single_phase_afe_ttype)
%[text] Initialization of single_phase_afe_ttype.slx: single-phase three-level T-type bridge (SiC MOSFET, loss and
%[text] thermal model) connected between two phases of a 690 V grid (three-phase emulator + Dyn11 1 MVA transformer)
%[text] through an L filter (250 uH on each line, 200 uF across the output), split DC link (2 x 12 mF, center tap =
%[text] neutral point of the bridge) fed by a 1200 V lithium-ion battery. Control at 8 kHz (4 kHz PWM, double update):
%[text] FHT-PLL on the three grid voltages, current amplitude reference in three steps (1/3 pu each at 25 ms, 0.5 s, 1 s),
%[text] in phase with the grid voltage; converter current -> virtual alpha-beta (SOGI C-Caller) -> dq -> vector PI
%[text] (C-Caller dqvector_pi, variants: Simulink dq_vector_pi, resonant PI) -> three-level PWM. There is no DC-link
%[text] voltage loop (the battery sets the link) and no neutral-point balancing loop (see the README).
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, afe_hw_param, afe_ctrl_param, grid_emulator_param, grid_fault_param, device_mosfet_param,
%[text] battery_param, check_model_param_refs). The library is needed for the Simulink/Simscape blocks (modulator,
%[text] T-type bridge, split DC link, grid emulator, transformer, battery, C-Caller functions) and for the MOSFET data
%[text] sets other than the embedded one. Run it from this folder (the functions of ./private are visible only to the
%[text] files of this folder). The legacy flat init (shared template, seven clock domains, DAB/PSFBC/CLLC/machines) is
%[text] kept as init_model_legacy.m for reference; it no longer matches the library (grid_fault_generator and
%[text] lithium_ion_battery_setup changed signature, the battery block reads param.battery.nominal_battery_voltage_1).

clear;
model = 'single_phase_afe_ttype';

% Simulink/Simscape library blocks (blocks, not parameters), C-Caller sources and device data sets
library_path = 'C:\github\library';
if exist(library_path, 'dir')
    addpath(genpath(library_path));
else
    warning('%s init: library path %s not found, the library blocks and data sets must already be on the path', model, library_path);
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
param.sim.simlength = 1.75;                 % [s] model StopTime
param.sim.tc = 0.625e-6;                    % [s] logging base, model MaxStep (ode23tb) and step of the Simscape
                                            %     local solver (glb_time.tc): ts/200 at fpwm = 4 kHz, double update
param.sim.use_mosfet_thermal_model = 1;     % the active bridge is the SiC MOSFET T-type with the loss and thermal
                                            % model (the ideal-switch bridge is commented out in the model): the
                                            % flag only sets the Simscape nonlinear iterations, as the legacy init
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration
if param.sim.use_mosfet_thermal_model
    param.sim.nonlinear_iterations = 5;
end
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
% one converter clock domain (bridge and control): 4 kHz PWM, double update -> ts = 125 us
param.time.fpwm = 4e3;                              % [Hz] switching frequency of the T-type bridge
param.time.trgo = 1;                                % double update: control at 2*fpwm = 8 kHz
param.time.dead_time = 3e-6;                        % [s] legacy dead_time_inv read by the modulator
param.time.t_measure = param.sim.simlength;         % [s] logging window
param.time.tc_factor = 1/param.time.fpwm/param.sim.tc/2;    % ts/tc = 200, so that glb_time.tc = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;

% sample time of the battery state-of-charge estimator (legacy ts_dab of the shared template: 3*fpwm, double update)
param.time.fpwm_dab = 3*param.time.fpwm;            % [Hz]
param.time.trgo_dab = 1;

param.glb_time = timing_param(param.time);
%[text] ## Grid
% three-phase grid emulator (ideal voltage sources) feeding a Dyn11 1 MVA 690/690 V transformer made of three
% single-phase transformer blocks; the bridge is connected between phases a and b of the secondary, the PLL reads
% the three secondary voltages. No fault applied (both enables at 0): the fault data are those of the legacy test
% 25.4 (75 % dip on two phases, variant D, from 0.75 s for 915 ms)
param.grid.application_voltage = 690;               % [V] rms line-to-line, primary and secondary of the transformer
param.grid.f_nom = 50;                              % [Hz]
param.grid.omega_nom = 2*pi*param.grid.f_nom;       % [rad/s]
param.grid.pwr_nom = 1000e3;                        % [VA] rated power of the transformer (bases of the grid currents)
param.grid.ucc = 5;                                 % [%] short-circuit voltage of the transformer
param.grid.eta = 95;                                % [%] efficiency of the transformer (copper losses)
param.grid.p_iron = 1800;                           % [W] iron losses of the transformer
param.grid.n2 = 14;                                 % turns of the secondary (star) coils of the transformer blocks
param.grid.n1 = floor(param.grid.n2*sqrt(3));       % turns of the primary (delta) coils: 24
param.grid.core_area = 0.05;                        % [m^2] core of the transformer blocks
param.grid.core_length = 2.5;                       % [m]
param.grid.mur = 10e3;                              % [-] relative permeability: Lm1 = 145 mH, i1m = 8.8 A
param.grid.R_eq = 2e-3;                             % [Ohm] equivalent grid resistance (informative, not in the model)
param.grid.frt = grid_fault_param('two_phase_D', 0.75, 0.915, 0.75, 0, 0);
param.grid.emu = grid_emulator_param(param.grid, param.grid.frt);
%[text] ## Single-phase T-type bridge
%[text] ### Hardware
% 225 kW nominal; the legacy per-unit bases of the library single-phase data set are kept (ubez 838.4 V, ibez 536.8 A,
% udc_nom 1239.4 V): the 690 V grid is 1.164 pu of ubez at the bridge terminals and 1 pu of current reference is
% 262 kW at 690 V. Split DC link 2 x 12 mF (6 mF in series), filter 250 uH + 250 uH with 200 uF across the output.
param.afe.hw = afe_hw_param(param.grid.application_voltage, 225e3, param.time.fpwm);
%[text] ### PWM
% three-level modulator of the library (single_phase_three_levels_PWM_generator_with_global_TRGO): input -1..1 =
% -udc..udc, carriers at fpwm, control trigger at the bottom and at the top of the carrier (double update), gates
% +15/-10 V, polarity 1 and limits -1..1 are the defaults of the block mask
param.afe.pwm.fpwm = param.glb_time.fPWM_AFE;       % [Hz]
param.afe.pwm.double_update = param.glb_time.TRGO_AFE_double_update;
param.afe.pwm.dead_time = param.glb_time.dead_time_AFE;    % [s]
param.afe.pwm.tc = param.glb_time.tc;               % [s]
%[text] ### Variants of the model
% current controller (Simulink dq_vector_pi, C-Caller dqvector_pi, or resonant PI rpi), virtual beta of the current
% (first harmonic tracker, SOGI, phase-shift filter, Simulink or C-Caller) and the identification subsystem (open
% loop u/f test, legacy system_identification_enable). Legacy flags: rpi_enable 0, use_current_controller_from_ccaller_mod1 1,
% use_single_phase_inverter_based_SOGI_ccaller 1, the others 0.
param.afe.variant.rpi_enable = 0;                   % 1: resonant PI on the alpha current instead of the dq vector PI
param.afe.variant.current_ctrl_ccaller = 1;         % 1: C-Caller dqvector_pi, 0: Simulink dq_vector_pi (read when rpi_enable = 0)
param.afe.variant.system_identification = 0;        % 1: open-loop u/f identification instead of the current control
param.afe.variant.beta_fht = 0;
param.afe.variant.beta_sogi = 0;
param.afe.variant.beta_phsh = 0;
param.afe.variant.beta_sogi_ccaller = 1;
param.afe.variant.beta_phsh_ccaller = 0;
if sum([param.afe.variant.beta_fht param.afe.variant.beta_sogi param.afe.variant.beta_phsh ...
        param.afe.variant.beta_sogi_ccaller param.afe.variant.beta_phsh_ccaller]) ~= 1
    error('%s init: exactly one virtual-beta variant must be active', model);
end
%[text] ### Control
% all the gains are in private/afe_ctrl_param (legacy values: current vector PI 0.5/45 with u_lim 1, resonant PI
% 0.75/45 with delta 0.025, FHT delta 0.05 with observer poles [-1 -4] w, PLL 1/80, SOGI delta 1 and kepsilon 2).
% The working frequency of the SOGI, phase-shift filters, resonant PI and impedance measure is the grid frequency
% (legacy frequency_set), 300 Hz in the identification test.
f_set = param.grid.f_nom;                           % [Hz]
if param.afe.variant.system_identification
    f_set = 300;
end
param.afe.ctrl = afe_ctrl_param(param.glb_time.ts_afe, param.grid.omega_nom, f_set);
clear f_set

% current amplitude reference: three steps (legacy iph_grid_pu_ref_1..3 at time_step_ref_1..3), summed and rate
% limited, the d axis reference of the vector PI (in phase with the grid voltage); the q reference is the Constant 0
% of the model. 1 pu = ibez = 536.8 A peak = 262 kW at 690 V.
param.afe.ref.i_pu_1 = 1/3;                         % [pu] of ibez
param.afe.ref.t_1 = 0.025;                          % [s]
param.afe.ref.i_pu_2 = 1/3;
param.afe.ref.t_2 = 0.5;
param.afe.ref.i_pu_3 = 1/3;
param.afe.ref.t_3 = 1;
param.afe.ref.slew = param.grid.omega_nom;          % [pu/s] rate limiter (the legacy block read grid_emu.omega_grid_nom)
param.afe.ref.i_pu_tot = param.afe.ref.i_pu_1 + param.afe.ref.i_pu_2 + param.afe.ref.i_pu_3;
param.afe.ref.p_tot = param.afe.ref.i_pu_tot*param.afe.hw.grid.p_1pu;   % [W] power at the final reference, unity power factor
%[text] ### Design figures (informative)
% three-level T-type between two phases: the output spans -udc..udc in steps of udc/2, each commutation is at udc/2;
% single-phase power pulsation p(t) = P*(1 - cos(2wt)) at unity power factor: the 100 Hz current P/udc splits between
% the series DC-link capacitance and the battery (no decoupling in this model, the battery sees most of it, see the
% README); the neutral point sees +-i_ac pulses of equal duration at each switching period (zero average with
% symmetric carriers), there is no balancing loop.
hw = param.afe.hw; w = param.grid.omega_nom; tr = param.grid.emu.trafo;
P = param.afe.ref.p_tot;
d.P = P;
d.i_peak = P/hw.grid.u_nom*sqrt(2);                             % [A] peak grid current at P
d.m = sqrt(hw.grid.u_peak^2 + (w*hw.LFu*d.i_peak)^2)/hw.udc_nom; % [-] modulation index at P and udc_nom
d.i_ripple_dc = P/hw.udc_nom;                                   % [A] amplitude of the 100 Hz current into the DC side
d.u_ripple_dc = d.i_ripple_dc/(2*w*hw.CFi);                     % [V] amplitude of the 100 Hz ripple of udc, series capacitance alone
d.u_ripple_dc_pu = d.u_ripple_dc/hw.udc_nom;                    % [pu]
d.e_ripple = P/w;                                               % [J] peak-to-peak energy pulsation at 2f
d.dq_np_sw = d.i_peak*0.5*hw.udc_nom/hw.udc_nom/param.time.fpwm;    % [C] worst-case charge pulse into the neutral point in one
                                                                %     switching period (duty 0.5 of the leg at the peak current)
d.du_np_sw = d.dq_np_sw/(hw.CFi1 + hw.CFi2);                    % [V] neutral-point voltage step of that pulse (C1 and C2 in parallel)
d.di_sw_max = (hw.udc_nom/2)/(4*param.time.fpwm*hw.LFu);        % [A] worst-case pk-pk switching ripple of the current (udc/2 steps)
d.L_grid = 2*tr.Ld2;                                            % [H] leakage of the two secondary coils between phases a and b
d.R_grid = 2*tr.Rd2;                                            % [Ohm]
d.f_res = sqrt((hw.LFu + d.L_grid)/(hw.LFu*d.L_grid*hw.CFu))/2/pi;  % [Hz] resonance of the L - CFu - transformer filter
d.k_i = hw.udc_nom/(hw.ibez*hw.LFu);                            % [1/s] plant gain of the current loop (pu/pu), LC seen as LFu
d.f_c_i = param.afe.ctrl.kp_i*d.k_i/2/pi;                       % [Hz] crossover estimate of the current loop
d.f_nyquist = 1/(2*param.glb_time.ts_afe);                      % [Hz]
param.afe.design = d;
fprintf('Design at %.0f kW (reference %.2f pu): %.0f A peak, m = %.2f, 100 Hz ripple %.0f A on the DC side -> %.0f V (%.1f %%) on udc with the 6 mF alone\n', ...
    P/1e3, param.afe.ref.i_pu_tot, d.i_peak, d.m, d.i_ripple_dc, d.u_ripple_dc, d.u_ripple_dc_pu*100);
fprintf('Neutral point: %.1f mC per switching period at the current peak -> %.2f V step on 2 x %.0f mF, no balancing loop | switching ripple %.0f A pk-pk\n', ...
    d.dq_np_sw*1e3, d.du_np_sw, hw.CFi1*1e3, d.di_sw_max);
fprintf('Filter: LFu %.0f uH + CFu %.0f uF + transformer %.0f uH: resonance %.0f Hz | current loop: crossover estimate %.0f Hz, Nyquist %.0f Hz\n', ...
    hw.LFu*1e6, hw.CFu*1e6, d.L_grid*1e6, d.f_res, d.f_c_i, d.f_nyquist);
clear hw w tr P d
%[text] ## Power semiconductors
% SiC MOSFET module Danfoss/Semikron SKM1700MB20R4S2I4 (1700 A, energies at 1300 V / 1000 A, 125 degC) for the eight
% switches of the T-type bridge (four per leg: two vertical, two in the bidirectional middle path). The data set
% is embedded in private/device_mosfet_param; any other data set of the library is read through device_data_load.
param.dev.used_device_mosfet = 'danfoss_SKM1700MB20R4S2I4';
dev = device_mosfet_param(param.dev.used_device_mosfet, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.dev.mosfet_name = dev.name;
param.dev.mosfet_type = dev.type;
param.afe.mosfet = rmfield(dev, {'name', 'type'});
clear dev
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;               % [pu] quantizer of the acquisitions (pu signals, 12 bit)
param.meas.adc16_quantization = 1/2^15;
param.meas.Imax_adc = 1049.835;                     % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;
param.meas.Umax_adc = 1500;                         % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
% bases of the grid-side logging (ug_pu_mod1, ig_pu_mod1 of vi_measures_grid): the legacy init used the bases of the
% three-phase AFE data set, 250 kW at 690 V (ubez = 690*sqrt(2/3) = 563.4 V, ibez = 250e3/sqrt(3)/690/0.9*sqrt(2) = 328.7 A)
param.meas.ugrid_bez = param.grid.application_voltage*sqrt(2/3);                            % [V] peak phase-to-neutral
param.meas.igrid_bez = 250e3/sqrt(3)/param.grid.application_voltage/0.9*sqrt(2);            % [A] peak
%[text] ## Thermal model
% aluminium plate, liquid cooled, sized for PrimePACK2; liquid flow > 28 l/min. "A" (ambient) is the water:
% HA is the temperature rise from water to heatsink surface; water in/out rise max 5 K with 2 kW total losses
param.thermal.weight = 0.150;                                   % [kg]
param.thermal.no_weight = 0.150/10;                             % [kg] /10: thermal inertia not accounted
param.thermal.cp_al = 900;                                      % [J/(kg K)] specific heat, aluminium
param.thermal.heat_capacity_hs = param.thermal.cp_al*param.thermal.weight;   % [J/K]
param.thermal.thermal_conductivity_al = 160;                    % [W/(m K)] aluminium
param.thermal.Rth_switch_HA = 15/1000;                          % [K/W]
param.thermal.Rth_mosfet_HA = param.thermal.Rth_switch_HA;      % [K/W] read by the bridge mask
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;       % [K/W]
param.thermal.Tambient = 40;                                    % [degC] water temperature
param.thermal.DThs_init = 0;                                    % [K] initial heatsink rise, read by the bridge mask

% heatsink object read by the bridge mask (fields cp, no_weight, Tambient)
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

% loss and junction temperature estimate of the T-type bridge at the final reference (informative, averaged model:
% sinusoidal current of amplitude I, modulation index m, unity power factor, synchronous conduction of the channels
% (v = Rds_on*i, the body diode only in the dead time); each commutation at udc/2, energies scaled linearly from the
% data set point. Vertical switch: conducts with duty m*sin, switches during its half period -> I/pi average;
% middle switch: conducts the zero state (1 - m*sin) in both half periods, its diode recovers at each vertical turn-on)
g = param.afe.mosfet; hw = param.afe.hw; I = param.afe.design.i_peak; m = param.afe.design.m; fsw = param.time.fpwm;
ksw = (hw.udc_nom/2)/g.Voff_sw_losses*(I/pi)/g.Ion_sw_losses;                       % [-] energy scaling, average over the half period
p.vert_cond = g.Rds_on*I^2*2*m/(3*pi);                                                % [W] per vertical switch
p.vert_sw = (g.Eon + g.Eoff)*ksw*fsw;                                                 % [W] per vertical switch
p.mid_cond = g.Rds_on*I^2*(1/2 - 4*m/(3*pi));                                         % [W] per middle switch (both half periods)
p.mid_rec = g.Err*ksw*fsw;                                                            % [W] per middle switch (body diode recovery)
p.leg = 2*(p.vert_cond + p.vert_sw) + 2*(p.mid_cond + p.mid_rec);                     % [W]
p.bridge = 2*p.leg;                                                                   % [W]
p.Tj_vert = param.thermal.Tambient + (p.vert_cond + p.vert_sw)*(g.Rth_mosfet_JH + param.thermal.Rth_mosfet_HA);   % [degC]
p.Tj_mid = param.thermal.Tambient + (p.mid_cond + p.mid_rec)*(g.Rth_mosfet_JH + param.thermal.Rth_mosfet_HA);
p.efficiency = param.afe.design.P/(param.afe.design.P + p.bridge + hw.RLFu*(I/sqrt(2))^2);
param.thermal.loss_estimate = p;
fprintf('T-type bridge losses at %.0f A peak, %.0f V, %.0f kHz: vertical switch %.0f W conduction + %.0f W switching, middle switch %.0f W + %.0f W; bridge %.0f W; Tj %.0f / %.0f degC; efficiency %.2f %% with the filter\n', ...
    I, hw.udc_nom, fsw/1e3, p.vert_cond, p.vert_sw, p.mid_cond, p.mid_rec, p.bridge, p.Tj_vert, p.Tj_mid, p.efficiency*100);
clear g hw I m fsw ksw p
%[text] ## Battery on the DC link (block battery_1)
% lithium-ion pack across the whole DC link: 1200 V nominal (the legacy init took hwdata.dab.udc1_bez of the 1200 V
% DAB data set selected at 690 V), 250 kW, initial state of charge 0.85; the legacy init halved R0 and R1 and
% divided C1 by 50 after the setup. The library block reads battery_voltage_init_m = param.battery.nominal_battery_voltage_1
% from its mask default (same name as in the other migrated models).
param.battery.nominal_battery_voltage_1 = 1200;                 % [V]
param.battery.nominal_battery_power = 250e3;                    % [W]
param.battery.initial_battery_soc = 0.85;
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;   % [Ah] ASSUMPTION: 1C rating
param.battery.plot_ocv = false;
b = param.battery;
param.battery.pack_1 = battery_param(b.nominal_battery_voltage_1, b.nominal_battery_power, b.charge_capacity_1, ...
    b.initial_battery_soc, param.glb_time.ts_dab, b.plot_ocv);
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;            % legacy modifications of the model
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/50;
clear b
%[text] ## Model setup
open_system(model);
Simulink.importExternalCTypes(model, 'Names', {'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'sogi_flt_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'rpi_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'phase_shift_flt_output_t'});
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
