%[text] # Grid-Forming in dq reference frame
%[text] Initialization of grid_forming_dq.slx: three-phase bridge with LCL filter (AFE2 of the layout 2),
%[text] battery on the DC link, Dyg transformer and RL load on the formed grid. All parameters are local:
%[text] the numbers are in this script and the parameter functions are in ./private (timing_param,
%[text] afe_hw_param, afe_ctrl_param, three_phase_transformer_param, device_igbt_param, battery_param,
%[text] check_model_param_refs). Nothing is read from library/setup_files any more; the library is still
%[text] needed for the Simulink/Simscape blocks and for the C sources of the C-Caller blocks.
%[text] Run it from this folder (the functions of ./private are visible only to the files of this folder).

clear;
model = 'grid_forming_dq';

% Simulink/Simscape library blocks and C-Caller sources (blocks, not parameters)
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
param.sim.simlength = 1;                    % [s] model StopTime
param.sim.tc = 0.625e-6;                    % [s] logging base, model MaxStep (ode23tb) and step of the
                                            %     Simscape local solver (glb_time.tc): ts_afe/200 at 4 kHz, double update
param.sim.use_thermal_model = 1;            % IGBT bridge with the simple two-thermal-port loss model
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration (grid_formed/grid_emulator/3ph_Dyg)
if param.sim.use_thermal_model
    param.sim.nonlinear_iterations = 5;
end
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
param.time.fpwm = 4e3;                              % [Hz] AFE PWM frequency
param.time.trgo = 1;                                % double update (control at 2*fpwm)
param.time.dead_time = 3e-6;                        % [s]
param.time.t_measure = 0.4;                         % [s] logging window (legacy init: 0.4 s, not simlength)
param.time.tc_factor = 1/param.time.fpwm/param.sim.tc/2;    % ts_afe/tc, so that glb_time.tc = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;

% second clock domain of the "global timing" subsystem (time_inv1): same PWM as the AFE
param.time.fpwm_inv = param.time.fpwm;              % [Hz]
param.time.trgo_inv = 1;
param.time.dead_time_inv = 3e-6;                    % [s]

% sample time of the battery state-of-charge estimator (legacy ts_dab of the shared template: 3*fpwm, double update)
param.time.fpwm_dab = 3*param.time.fpwm;            % [Hz]
param.time.trgo_dab = 1;

param.glb_time = timing_param(param.time);
%[text] ### AFE PWM
param.afe.pwr_nom = 250e3;                          % [W]
param.afe.pwm.fpwm = param.glb_time.fPWM_AFE;       % [Hz]
param.afe.pwm.trgo = param.glb_time.TRGO_AFE_double_update;
param.afe.pwm.dead_time = param.glb_time.dead_time_AFE;     % [s]
%[text] ## Formed grid: coupling transformer and load
param.grid.grid_nominal_power = 1000e3;             % [W] transformer rating
param.grid.application_voltage = 400;               % [V] 400, 480 or 690
param.grid.grid_nominal_current = param.grid.grid_nominal_power/param.grid.application_voltage/sqrt(3);

% coupling transformer Dyn11 (block grid_formed/grid_emulator/3ph_Dyg: three single-phase transformers)
param.grid.delta_star = 1;

if param.grid.application_voltage == 690
    param.grid.us1 = 690; param.grid.us2 = 690; param.grid.fgrid = 50;
    param.grid.eta = 95; param.grid.ucc = 5;
    param.grid.p_iron = 1800;
elseif param.grid.application_voltage == 480
    param.grid.us1 = 480; param.grid.us2 = 480; param.grid.fgrid = 60;
    param.grid.eta = 95; param.grid.ucc = 5;
    param.grid.p_iron = 1400;
else
    param.grid.us1 = 400; param.grid.us2 = 400; param.grid.fgrid = 50;
    param.grid.eta = 95; param.grid.ucc = 5;
    param.grid.p_iron = 1000;
end

param.grid.n2 = 14;
param.grid.n1 = floor(param.grid.n2*sqrt(3));
param.grid.core_area = 0.05;                        % [m^2]
param.grid.core_length = 2.5;                       % [m]
param.grid.mu0 = 4*pi*1e-7;
param.grid.mur = 10e3;

% magnetizing inductance from the core geometry, no-load current from it
param.grid.Lm1 = (param.grid.n1^2 * param.grid.mu0 * param.grid.mur * param.grid.core_area) / param.grid.core_length;
param.grid.i1m = param.grid.us1/sqrt(3)/param.grid.Lm1/(2*pi*param.grid.fgrid);

% param.grid.emu keeps the name of the legacy grid emulator: the blocks read param.grid.emu.trafo.*
g = param.grid;
param.grid.emu.omega_grid_nom = 2*pi*g.fgrid;        % [rad/s]
param.grid.emu.trafo = three_phase_transformer_param('Dyn11', g.delta_star, g.grid_nominal_power, g.us1, g.us2, ...
    g.fgrid, g.eta, g.ucc, g.i1m, g.p_iron, g.n1, g.n2, g.core_area, g.core_length, g.mur);
clear g

% RL load of the formed grid (block grid_formed/grid_emulator/three_phase_rl_load). NOT read by the block
% yet: the values are hard-coded in its mask (2026-10-21: per phase R = 230^2*3/275e3*[0.9 1.1 0.95] Ohm,
% i.e. about 275 kW at 230 V with +/-10 % unbalance, L = 1 mH each). Listed here as documentation and for
% the next step (mask with two 3-element vectors).
param.load.Rs_load = 230^2*3/275e3*[0.9 1.1 0.95];  % [Ohm] per phase
param.load.Ls_load = [1e-3 1e-3 1e-3];              % [H] per phase
%[text] ## AFE
%[text] ### Hardware (LCL filter)
% Same data sets as the library (690 V, 480 V, 400 V) selected by the application voltage. The output filter
% is an LCL: LFu1 converter side, CFu (star, floating), LFu2 grid side. Default LFu1 = LFu2 = the legacy LFu
% (330 uH at 400 V), i.e. the value the model had on both inductors before the split. To recover the total
% inductance of the legacy LC filter set, after this call, LFu1 = LFu2 = legacy LFu/2 (and the RLFu* accordingly).
param.afe.hw = afe_hw_param(param.grid.application_voltage, param.afe.pwr_nom, param.glb_time.fPWM_AFE);

% LCL checks: resonance versus the control frequency, capacitor reactive power
fs = 1/param.glb_time.ts_afe;                       % [Hz] control/sampling frequency (double update)
hw = param.afe.hw;
fprintf('LCL check: f_res = %.0f Hz, fs/6 = %.0f Hz, fs/2 = %.0f Hz, Z0 = %.2f Ohm, RCFu = %.0f mOhm, Qc = %.1f %% of Pnom\n', ...
    hw.f_res, fs/6, fs/2, hw.z0_lcl, hw.RCFu*1e3, hw.q_cfu_pu*100);
% Converter-side current feedback with a 1.5*ts loop delay: the loop damps the resonance only if f_res < fs/6
% (critical frequency); above fs/6 it adds negative damping and only the passive RCFu holds the mode
% (text corrected on 2026-10-21 with the grid_follower_dq review: the earlier warning stated the opposite).
if hw.f_res > fs/6
    warning(['%s init: LCL resonance %.0f Hz above fs/6 = %.0f Hz: the converter-current loop adds negative ' ...
        'damping, only RCFu holds the mode (zeta about %.3f)'], model, hw.f_res, fs/6, hw.RCFu*hw.CFu*hw.omega_res/2);
elseif hw.f_res > 0.85*fs/6
    warning('%s init: LCL resonance %.0f Hz within 15 %% of fs/6 = %.0f Hz: no damping from the loop, only RCFu', model, hw.f_res, fs/6);
end
if hw.f_res > fs/2
    warning('%s init: LCL resonance %.0f Hz above the Nyquist frequency fs/2 = %.0f Hz', model, hw.f_res, fs/2);
end
if hw.q_cfu_pu > 0.05
    warning('%s init: filter capacitor reactive power %.1f %% of Pnom (usual range 2 - 5 %%)', model, hw.q_cfu_pu*100);
end
clear fs hw
%[text] ### Control: first harmonic tracker, limits
param.afe.ctrl = afe_ctrl_param(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom);

% dq voltage loop (block afe_control/dq_vector_pi): vector PI on the grid-side voltage u_grid_xi/eta (pu, from
% the FHT positive sequence) with references ud_ref = 1, uq_ref = 0; outputs i_xi_ref, i_eta_ref (pu), vector
% limited at ixi_pos_ref_lim. kp in pu/pu, ki in 1/s (integrator gain of the blocks ki*ts_inv = ki*ts_afe here).
% Linear check (2026-10-21, current loop without feed-forward, load as in the model): kp 0.5 / ki 18 -> dominant
% closed-loop poles at -11 and -20 1/s (slow, no overshoot); kp 1 / ki 50 would excite the LCL resonance
% (unstable 760 Hz mode): faster voltage control needs the current-loop feed-forward and the LCL damping first.
param.afe.ctrl.kp_ug_ctrl = 0.5;
param.afe.ctrl.ki_ug_ctrl = 18;                    % [1/s]
%[text] ### dq current controller
% gains of the dq vector PI of the converter-side current loop (blocks current_ctrl/dq_vector_pi/pi_d, pi_q):
% kp in pu/pu (modulation index per pu current), ki in 1/s. The integrator gain of the blocks is
% ki_i*param.glb_time.ts_inv (same value as ts_afe in this model).
param.afe.dq_vector_pi.kp_i = 0.75;
param.afe.dq_vector_pi.ki_i = 35;

% design figures of the current loop, informative (the gains above are not changed by this section):
% plant from the modulation index to the converter current with the load voltage fed forward: 1/(s*L1_pu),
% L1_pu = LFu1/zbez [s]; modulation gain udc/sqrt(3)/ubez; delay 1.5 ts (one step computation + ZOH)
d.L1_pu = param.afe.hw.tau_lfu1_pu;                                             % [s]
d.k_mod = param.afe.hw.udc_nom/sqrt(3)/param.afe.hw.ubez;                      % [pu/pu]
d.f_cross_ff = param.afe.dq_vector_pi.kp_i*d.k_mod/d.L1_pu/2/pi;                % [Hz] crossover with ideal feed-forward
d.f_zero = param.afe.dq_vector_pi.ki_i/param.afe.dq_vector_pi.kp_i/2/pi;        % [Hz] PI zero
d.f_cross_target = 300;                                                         % [Hz] example target
d.kp_i_target = 2*pi*d.f_cross_target*d.L1_pu/d.k_mod;                          % [pu/pu]
d.ki_i_target = d.kp_i_target*2*pi*d.f_cross_target/5;                          % [1/s] zero at f_cross/5
param.afe.dq_vector_pi.design = d;
fprintf('Current loop: kp_i = %.2f, ki_i = %.0f 1/s -> crossover %.0f Hz with voltage feed-forward (PI zero at %.0f Hz); %.0f Hz would need kp_i = %.2f, ki_i = %.0f\n', ...
    param.afe.dq_vector_pi.kp_i, param.afe.dq_vector_pi.ki_i, d.f_cross_ff, d.f_zero, d.f_cross_target, d.kp_i_target, d.ki_i_target);
clear d
%[text] ### Common-mode reference
param.afe.ref.u_cm_comp_mod1 = 0;                   % ugamma of the space vector modulator (0: standard SVPWM, k_offset = 0.5)
%[text] ### Module behaviour (clock mismatch, PWM phase shift)
param.afe.behav.time_gain_module_1 = 1.0002;        % local clock of the AFE versus master time ("global timing")
param.afe.behav.pwm_phase_shift_mod1 = 0;           % [rad] carrier phase of the modulator
param.inv.behav.time_gain_module_2 = 1.001;         % second clock (time_inv1), read with the name of afe_inv_psm
%[text] ## Power semiconductors
% 'mitsubishi_CM1200DW_34T' (1700 V) is the legacy choice of the shared init; the module of the proposal is
% the Infineon FF1800R12IE5P (1200 V): 'infineon_FF1800R12IE5'
param.dev.used_device_igbt = 'mitsubishi_CM1200DW_34T';
param.afe.igbt = device_igbt_param(param.dev.used_device_igbt, param.glb_time.fPWM_AFE, param.afe.hw.udc_nom);
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;               % [pu] quantizer of the acquisitions (pu signals)
param.meas.adc16_quantization = 1/2^15;
param.meas.Imax_adc = 1049.835;                     % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;
param.meas.Umax_adc = 1500;                         % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%[text] ## Control settings common to all modules
% local time alignment to master time (acquisitions/phase_compensation_1)
param.ctrl.kp_align = 0.25;
param.ctrl.ki_align = 18;
param.ctrl.lim_up_align = 0.05;
param.ctrl.lim_down_align = -0.05;

% PI clip release (leak of the integrator while the vector limiter clips)
param.ctrl.CTRPIFF_CLIP_RELEASE = 0.001;
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
param.battery.nominal_battery_voltage_1 = 750;   % [V]
param.battery.nominal_battery_power = 250e3;                      % [W]
param.battery.initial_battery_soc = 0.85;
% ASSUMPTION (not in the legacy init): 1C rating, capacity = rated current x 1 h
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;   % [Ah]
param.battery.plot_ocv = false;

b = param.battery;
param.battery.pack_1 = battery_param(b.nominal_battery_voltage_1, b.nominal_battery_power, ...
    b.charge_capacity_1, b.initial_battery_soc, param.glb_time.ts_dab, b.plot_ocv);
clear b

% special settings for simulation (same as the legacy init)
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/200;
%[text] ## DC-link stray inductance (partial loop inductance)
% same data as the library script parasitic_dclink_data.m; not read by the model, Bode check only
param.dclink.Lstray_dclink = 100e-9;                            % [H]
param.dclink.RLstray_dclink = 10e-3;                            % [Ohm]
param.dclink.C_HF_Lstray_dclink = 15e-6;                        % [F]
param.dclink.R_HF_Lstray_dclink = 22000;                        % [Ohm]
param.dclink.Z_HF_Lstray_dclink = 1/s/param.dclink.C_HF_Lstray_dclink + param.dclink.R_HF_Lstray_dclink;
param.dclink.Z_LF_Lstray_dclink = s*param.dclink.Lstray_dclink + param.dclink.RLstray_dclink;
param.dclink.Z_Lstray_dclink = param.dclink.Z_HF_Lstray_dclink*param.dclink.Z_LF_Lstray_dclink/ ...
    (param.dclink.Z_HF_Lstray_dclink + param.dclink.Z_LF_Lstray_dclink);
param.dclink.ZCFi = 7/s/param.afe.hw.CFi;
param.dclink.sys_dclink = minreal(param.dclink.ZCFi/(param.dclink.ZCFi + param.dclink.Z_Lstray_dclink));
figure; bode(param.dclink.sys_dclink, param.dclink.Z_Lstray_dclink, options); grid on
%[text] ## Model setup
open_system(model);
%[text] ### C-Caller types
ctypes = {'mavgflt_output_t', 'dsmavgflt_output_t', 'mavgflts_output_t', 'bemf_obsv_output_t', ...
    'bemf_obsv_load_est_output_t', 'dqvector_pi_output_t', 'sv_pwm_output_t', 'sv_pwm_cm_output_t', ...
    'global_state_machine_output_t', 'first_harmonic_tracker_output_t', 'dqpll_thyr_output_t', ...
    'dqpll_grid_output_t', 'rpi_output_t', 'phase_shift_flt_output_t', 'sogi_flt_output_t', ...
    'linear_double_integrator_observer_output_t'};
for k = 1:numel(ctypes)
    Simulink.importExternalCTypes(model, 'Names', ctypes(k));
end
clear ctypes k
%[text] ### Scopes closed at start
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
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
