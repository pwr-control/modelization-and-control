%[text] # Single-phase AFE with active power decoupling
%[text] Initialization of single_phase_afe.slx: full-bridge active front end between two phases of a 690 V grid
%[text] (LCL filter 250 uH + 200 uF + 250 uH), DC link of 2.4 mF feeding a current-source load, and a second full
%[text] bridge (the "ripple port") whose DC side is a 3.6 mF buffer capacitor at 2000 V and whose AC side is tied to
%[text] the DC link through two 250 uH inductors. Control at 10 kHz (5 kHz PWM, double update): DC-link voltage PI ->
%[text] amplitude of the grid current -> virtual alpha-beta (first harmonic tracker) dq vector PI -> PWM; FHT-PLL on
%[text] the three grid voltages; buffer: DC voltage PI -> DC current reference, 100 Hz ripple loop (derivative observer of
%[text] udc_pu -> quadrature pair at 2f -> dq at 2 theta -> vector PI -> current reference at 100 Hz) -> buffer current PI
%[text] with udc_pu feed-forward -> PWM of the ripple port. The ripple loop is enabled at 0.5 s.
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, afe_hw_param, afe_ctrl_param, grid_emulator_param, grid_fault_param, device_igbt_param,
%[text] battery_param, check_model_param_refs). The library is needed for the Simulink/Simscape blocks (modulators,
%[text] bridges, DC link, grid emulator, C-Caller functions) and for the IGBT data sets other than the embedded one.
%[text] Run it from this folder (the functions of ./private are visible only to the files of this folder).
%[text] The legacy flat init (shared template, seven clock domains, DAB/PSFBC/CLLC/machines) is kept as
%[text] init_model_legacy.m for single_phase_afe_ideal.slx and dcdc_buffer.slx, which still read the old names.

clear;
model = 'single_phase_afe';

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
param.sim.simlength = 1.25;                 % [s] model StopTime
param.sim.tc = 1e-6;                        % [s] logging base, model MaxStep (ode23tb) and step of the Simscape
                                            %     local solvers (glb_time.tc): ts/100 at fpwm = 5 kHz, double update
param.sim.use_thermal_model = 1;            % IGBT bridges with the simple two-thermal-port loss model
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration
if param.sim.use_thermal_model
    param.sim.nonlinear_iterations = 5;
end
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
% one converter clock domain (both full bridges and all the control loops): 5 kHz PWM, double update -> ts = 100 us
param.time.fpwm = 5e3;                              % [Hz] switching frequency of the two full bridges
param.time.trgo = 1;                                % double update: control at 2*fpwm = 10 kHz
param.time.dead_time = 1e-6;                        % [s] legacy glb_time.dead_time_INV read by the two modulators
                                                    %     (the FF1200R17IP5 has td_off + tf = 1.74 us: see the README)
param.time.t_measure = param.sim.simlength;         % [s] logging window
param.time.tc_factor = 1/param.time.fpwm/param.sim.tc/2;    % ts/tc = 100, so that glb_time.tc = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;

% sample time of the battery state-of-charge estimator (legacy ts_dab of the shared template; the battery block is
% commented out in the model)
param.time.fpwm_dab = param.time.fpwm;              % [Hz]
param.time.trgo_dab = 1;

param.glb_time = timing_param(param.time);
%[text] ## Grid
% three-phase grid emulator (ideal voltage sources, 690 V line-to-line, 50 Hz); the AFE is connected between
% phases a and b, the PLL reads the three phase voltages. No fault applied (both enables at 0): the fault data are
% those of the legacy test 25.4 (75 % dip on two phases, variant D, from 0.75 s for 915 ms)
param.grid.application_voltage = 690;               % [V] rms line-to-line
param.grid.f_nom = 50;                              % [Hz]
param.grid.omega_nom = 2*pi*param.grid.f_nom;       % [rad/s]
param.grid.pwr_nom = 1000e3;                        % [VA] grid/transformer power behind the emulator (bases, L_eq)
param.grid.ucc = 5;                                 % [%] short-circuit voltage for L_eq (informative, ideal source in the model)
param.grid.frt = grid_fault_param('two_phase_D', 0.75, 0.915, 0.75, 0, 0);
param.grid.emu = grid_emulator_param(param.grid.application_voltage, param.grid.f_nom, param.grid.pwr_nom, ...
    param.grid.ucc, param.grid.frt);
%[text] ## Single-phase AFE
%[text] ### Hardware
% 225 kW nominal; the legacy per-unit bases of the library single-phase data set are kept (ubez 838.4 V, ibez 536.8 A,
% udc_nom 1239.4 V): the 690 V grid is 1.164 pu of ubez at the AFE terminals. DC link 2.4 mF, LCL 250 uH + 200 uF + 250 uH.
param.afe.hw = afe_hw_param(param.grid.application_voltage, 225e3, param.time.fpwm);
%[text] ### Ripple port (buffer)
% H-bridge with the buffer capacitor on its DC side and the AC side tied to the DC link through two inductors: a
% boost-type ripple port, the buffer voltage must stay above udc. Numbers of the model (they were hard-coded in
% the blocks): 3.6 mF (7.2 mF/2), 2 x 250 uH with 4 mOhm, 2000 V reference, current base 750 A of the acquisition.
param.afe.buffer.C = 7.2e-3/2;                      % [F] buffer capacitor (ESR 1 uOhm in the block)
param.afe.buffer.L = 250e-6;                        % [H] each of the two inductors between the bridge and the DC link
param.afe.buffer.RL = 4e-3;                         % [Ohm] series resistance of each inductor
param.afe.buffer.L_tot = 2*param.afe.buffer.L;      % [H]
param.afe.buffer.udc_ref = 2000;                    % [V] buffer DC voltage reference (step at ctrl.buffer.t_ref, filtered)
param.afe.buffer.ibez = 750;                        % [A] base of the buffer current acquisition (gain 1/750 of the model)
param.afe.buffer.udc_pu_ref = param.afe.buffer.udc_ref/param.afe.hw.udc_nom;    % [pu] of udc_nom: 1.614 (the buffer
                                                    % voltage is acquired with the same 1/udc_nom gain as the DC link)
%[text] ### PWM
% unipolar modulation of each full bridge (library single-phase modulator, polarity -1, gates +18/-3 V), carriers at
% fpwm, control trigger at the bottom and at the top of the carrier (double update)
param.afe.pwm.fpwm = param.glb_time.fPWM_AFE;       % [Hz]
param.afe.pwm.double_update = param.glb_time.TRGO_AFE_double_update;
param.afe.pwm.dead_time = param.glb_time.dead_time_AFE;    % [s]
param.afe.pwm.tc = param.glb_time.tc;               % [s]
%[text] ### Control
% all the gains are in private/afe_ctrl_param (legacy values: udc loop 2/35 with a 20 ms reference filter, current
% vector PI 0.5/45 with u_lim 1, FHT delta 0.05 with observer poles [-1 -4] w, PLL 1/80, ripple loop vector PI 1/18
% enabled at 0.5 s, buffer voltage PI 2/18 with a 20 ms reference filter, buffer current PI 1/18)
param.afe.ctrl = afe_ctrl_param(param.glb_time.ts_afe, param.grid.omega_nom);
param.afe.ctrl.buffer.udc_pu_ref = param.afe.buffer.udc_pu_ref;     % [pu] read by the Step block of the buffer voltage loop

% variants of the model: current controller (Simulink dq_vector_pi, or C-Caller dqvector_pi) and virtual beta of
% the current (first harmonic tracker, SOGI, phase-shift filter, Simulink or C-Caller). Legacy flags
% use_current_controller_from_ccaller_mod1 = 0, use_single_phase_inverter_based_FHT = 1, the others 0.
param.afe.variant.current_ctrl_ccaller = 0;
param.afe.variant.beta_fht = 1;
param.afe.variant.beta_sogi = 0;
param.afe.variant.beta_phsh = 0;
param.afe.variant.beta_sogi_ccaller = 0;
param.afe.variant.beta_phsh_ccaller = 0;
if sum([param.afe.variant.beta_fht param.afe.variant.beta_sogi param.afe.variant.beta_phsh ...
        param.afe.variant.beta_sogi_ccaller param.afe.variant.beta_phsh_ccaller]) ~= 1
    error('%s init: exactly one virtual-beta variant must be active', model);
end
%[text] ### Design figures (informative)
% single-phase power pulsation p(t) = P*(1 - cos(2wt)) at unity power factor: the DC link sees a current ripple of
% amplitude P/udc at 2f, the capacitor alone would give a voltage ripple of amplitude P/(2w*C*udc); the energy that
% pulsates (peak-to-peak) is P/w and the ripple port must store it: dE = C_buf/2*(Vmax^2 - Vmin^2).
hw = param.afe.hw; b = param.afe.buffer; w = param.grid.omega_nom;
P = hw.pwr_nom;
d.P = P;
d.i_ripple_dc = P/hw.udc_nom;                                   % [A] amplitude of the 100 Hz current into the DC link
d.u_ripple_dc = P/(2*w*hw.CFi*hw.udc_nom);                      % [V] amplitude of the 100 Hz ripple of udc, capacitor alone
d.u_ripple_dc_pu = d.u_ripple_dc/hw.udc_nom;                    % [pu]
d.i_cdc_rms = d.i_ripple_dc/sqrt(2);                            % [A] rms 100 Hz current of the DC-link capacitor without decoupling
d.e_ripple = P/w;                                               % [J] peak-to-peak energy pulsation at 2f
d.e_buf = 0.5*b.C*b.udc_ref^2;                                  % [J] energy of the buffer at its reference
d.u_buf_max = sqrt(b.udc_ref^2 + d.e_ripple/b.C);               % [V] buffer voltage swing with the whole pulsation
d.u_buf_min = sqrt(b.udc_ref^2 - d.e_ripple/b.C);               % [V]
d.u_buf_ripple = d.u_buf_max - d.u_buf_min;                     % [V] peak-to-peak
d.i_buf_ac = d.i_ripple_dc;                                     % [A] amplitude of the 100 Hz buffer current (DC-link side)
d.i_buf_ac_pu = d.i_buf_ac/b.ibez;                              % [pu] of the buffer current base
d.m_buf = hw.udc_nom/b.udc_ref;                                 % [-] modulation index of the ripple port at udc_nom
d.u_L_buf = 2*w*b.L_tot*d.i_buf_ac;                             % [V] voltage across the buffer inductors at 2f
d.di_buf_sw_max = b.udc_ref/(8*param.time.fpwm*b.L_tot);        % [A] worst-case pk-pk switching ripple of the buffer current (unipolar, 2 fpwm)
d.di_grid_sw_max = hw.udc_nom/(8*param.time.fpwm*hw.LFu1);      % [A] worst-case pk-pk switching ripple of the converter-side current
d.c_buf_min_10pct = d.e_ripple/(0.5*((1.05*b.udc_ref)^2 - (0.95*b.udc_ref)^2));   % [F] buffer for a 10 % voltage swing
% loop gains (plants in pu, static part): current loop, DC-link voltage loop, ripple loop (envelope)
d.k_i = hw.udc_nom/(hw.ibez*hw.LFu);                            % [1/s] plant gain of the current loop (pu/pu), LCL seen as LFu
d.f_c_i = param.afe.ctrl.kp_i*d.k_i/2/pi;                       % [Hz] crossover estimate of the current loop
d.k_udc = 0.5*hw.grid.u_peak*hw.ibez/(hw.CFi*hw.udc_nom^2);     % [1/s] plant gain of the DC-link voltage loop (pu/pu)
d.f_c_udc = param.afe.ctrl.kp_udc*d.k_udc/2/pi;                 % [Hz] crossover estimate of the DC-link voltage loop
d.k_buf_i = b.udc_ref/(b.L_tot*b.ibez);                         % [1/s] plant gain of the buffer current loop (pu/pu)
d.f_c_buf_i = param.afe.ctrl.buffer.kp_i*d.k_buf_i/2/pi;        % [Hz] crossover estimate of the buffer current loop
d.g_apd = b.ibez/(2*w*hw.CFi*hw.udc_nom)*(4/5);                 % [pu/pu] ripple loop: buffer current amplitude -> detected udc ripple
                                                                %   (derivative observer gain 4/5 at 2f with poles [-1 -4] w)
param.afe.design = d;
fprintf('Design at %.0f kW: 100 Hz ripple %.0f A on the DC link, %.0f V (%.1f %%) on udc without decoupling, Cdc rms current %.0f A\n', ...
    P/1e3, d.i_ripple_dc, d.u_ripple_dc, d.u_ripple_dc_pu*100, d.i_cdc_rms);
fprintf('Buffer: %.0f J to store, %.1f mF at %.0f V (%.0f J) swings %.0f-%.0f V (%.1f %%), %.0f A peak at 100 Hz (%.2f pu of %.0f A), m = %.2f, %.0f V on the inductors, switching ripple %.0f A pk-pk\n', ...
    d.e_ripple, b.C*1e3, b.udc_ref, d.e_buf, d.u_buf_min, d.u_buf_max, d.u_buf_ripple/b.udc_ref*100, d.i_buf_ac, d.i_buf_ac_pu, ...
    b.ibez, d.m_buf, d.u_L_buf, d.di_buf_sw_max);
fprintf('Loops (static estimates): current %.0f Hz, DC-link voltage %.1f Hz, buffer current %.0f Hz, ripple loop gain %.2f pu/pu\n', ...
    d.f_c_i, d.f_c_udc, d.f_c_buf_i, d.g_apd);
clear hw b w P d
%[text] ## Power semiconductors
% 1700 V module for the 1239 V link: Infineon FF1200R17IP5 (PrimePACK 3, 1200 A, values at 125 degC), one half bridge
% per leg of both full bridges (the buffer bridge runs at 2000 V with the same data set: see the README). The data set
% is embedded in private/device_igbt_param; any other data set of the library is read through device_data_load.
param.dev.used_device_igbt = 'infineon_FF1200R17IP5';
dev = device_igbt_param(param.dev.used_device_igbt, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
param.dev.igbt_name = dev.name;
param.dev.igbt_type = dev.type;
param.afe.igbt = rmfield(dev, {'name', 'type'});
clear dev
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;               % [pu] quantizer of the acquisitions (pu signals, 12 bit)
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
param.thermal.heat_capacity_hs = param.thermal.cp_al*param.thermal.weight;   % [J/K]
param.thermal.thermal_conductivity_al = 160;                    % [W/(m K)] aluminium
param.thermal.Rth_switch_HA = 15/1000;                          % [K/W]
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;       % [K/W]
param.thermal.Tambient = 40;                                    % [degC] water temperature (mask of the bridges)
param.thermal.DThs_init = 0;                                    % [K]

% heatsink object read by the bridge masks (fields cp, no_weight, Rth_switch_HA, Rth_diode_HA)
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

% loss and junction temperature estimate of the grid-side bridge at pwr_nom (informative, same model as the bridge:
% conduction v = V0 + R*i, switching energies scaled linearly from the data set point to udc and to the current;
% sinusoidal current of amplitude I, modulation index m, unity power factor: IGBT conduction (1/(2pi) + m/8) terms,
% diode (1/(2pi) - m/8), switching energies averaged over the half period -> I/pi)
g = param.afe.igbt; hw = param.afe.hw; I = hw.grid.i_peak; m = hw.grid.m_nom; fsw = param.time.fpwm;
p.igbt_cond = g.Vce_sat*I*(1/(2*pi) + m/8) + g.Rce_on*I^2*(1/8 + m/(3*pi));          % [W] per switch
p.diode_cond = g.Vdon_diode*I*(1/(2*pi) - m/8) + g.Rdon_diode*I^2*(1/8 - m/(3*pi));   % [W] per diode
p.igbt_sw = (g.Eon + g.Eoff)*(I/pi/g.Ion_sw_losses)*(hw.udc_nom/g.Voff_sw_losses)*fsw;   % [W] per switch
p.diode_rec = g.Erec*(I/pi/g.Ion_sw_losses)*(hw.udc_nom/g.Voff_sw_losses)*fsw;           % [W] per diode
p.bridge = 4*(p.igbt_cond + p.igbt_sw + p.diode_cond + p.diode_rec);                      % [W] four switches
p.Tj_igbt = param.thermal.Tambient + (p.igbt_cond + p.igbt_sw)*(g.Rth_switch_JH + param.thermal.Rth_switch_HA);   % [degC]
p.Tj_diode = param.thermal.Tambient + (p.diode_cond + p.diode_rec)*(g.Rth_diode_JH + param.thermal.Rth_diode_HA);
p.efficiency = hw.pwr_nom/(hw.pwr_nom + p.bridge + hw.RLFu*hw.grid.i_nom^2);
param.thermal.loss_estimate = p;
fprintf('Grid bridge losses at %.0f A peak, %.0f V, %.0f kHz: per switch IGBT %.0f W conduction + %.0f W switching, diode %.0f W + %.0f W; bridge %.0f W; Tj IGBT %.0f degC, diode %.0f degC; efficiency %.2f %% with the filter\n', ...
    I, hw.udc_nom, fsw/1e3, p.igbt_cond, p.igbt_sw, p.diode_cond, p.diode_rec, p.bridge, p.Tj_igbt, p.Tj_diode, p.efficiency*100);
clear g hw I m fsw p
%[text] ## Load
% current source on the DC link: ramp from 0 at 4*P_load/udc_nom A/s, limited at P_load/udc_nom (0.25 s to full load).
% The legacy value is 275 kW: 122 % of the 225 kW rating of the AFE (see the README). The resistor R_load of the
% block load/load is commented out in the model.
param.load.pwr = 275e3;                                         % [W]
param.load.i_max = param.load.pwr/param.afe.hw.udc_nom;         % [A] 221.9 A
param.load.i_slope = 4*param.load.i_max;                        % [A/s]
param.load.R = param.afe.hw.udc_nom^2/param.load.pwr;           % [Ohm] resistive alternative (commented block)
%[text] ## Battery on the DC link (block battery_1, commented out in the model)
param.battery.nominal_voltage = param.afe.hw.udc_nom;           % [V]
param.battery.nominal_power = param.afe.hw.pwr_nom;             % [W]
param.battery.initial_soc = 0.85;
param.battery.charge_capacity = param.battery.nominal_power/param.battery.nominal_voltage;   % [Ah] ASSUMPTION: 1C rating
param.battery.plot_ocv = false;
b = param.battery;
param.battery.pack_1 = battery_param(b.nominal_voltage, b.nominal_power, b.charge_capacity, b.initial_soc, ...
    param.glb_time.ts_dab, b.plot_ocv);
clear b
%[text] ## Model setup
open_system(model);
Simulink.importExternalCTypes(model, 'Names', {'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'phase_shift_flt_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'sogi_flt_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'linear_double_integrator_observer_output_t'});
%[text] ### Scopes closed at start
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### Bus object of the IEC power meter (MATLAB Function three_phase_power_meter_iec, output type Bus: PowerMeterIecBus)
names = {'Q1p', 'Q1n', 'P1n', 'P1p', 'U1n', 'U1p', 'i_act_1n', 'i_react_1n', 'i_act_1p', 'i_react_1p'};
elems = Simulink.BusElement.empty(0, 1);
for k = 1:numel(names)
    e = Simulink.BusElement;
    e.Name = names{k};
    e.Dimensions = 1;
    e.DataType = 'double';
    e.Complexity = 'real';
    elems(k, 1) = e;
end
PowerMeterIecBus = Simulink.Bus;
PowerMeterIecBus.Elements = elems;
assignin('base', 'PowerMeterIecBus', PowerMeterIecBus);
clear names elems e k
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
