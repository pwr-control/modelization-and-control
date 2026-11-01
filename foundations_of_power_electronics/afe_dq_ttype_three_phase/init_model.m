%[text] # Three-phase three-level T-type AFE with dq control (afe_dq_ttype)
%[text] Initialization of afe_dq_ttype.slx: three-phase three-level T-type active front end (IGBT Mitsubishi
%[text] CM1200DW-34T for the twelve switches, loss and thermal model) between a 690 V grid (three-phase emulator +
%[text] Dyn11 1 MVA transformer, star point of the secondary grounded) and a split DC link (2 x 14.4 mF, center tap =
%[text] neutral point of the bridge, floating) loaded by a 275 kW resistor across the whole link. Filter: 500 uH
%[text] three-phase inductor, 200 uF in star (floating star point), then the transformer leakage (LCL). Control at
%[text] 8 kHz (4 kHz PWM, double update): FHT-PLL (C-Caller, three other PLLs selected by variant), DC-link voltage PI
%[text] -> active current reference, dq vector PI of the converter current (no grid-voltage feed-forward) ->
%[text] alpha-beta -> space vector modulation (C-Caller sv_pwm_cm) -> three-level PWM -> dead time -> bridge. The
%[text] zero-sequence current PI is computed but reaches the modulator through u_cm_comp_mod1 = 0, and there is no
%[text] neutral-point balancing loop (see the README).
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, afe_hw_param, afe_ctrl_param, grid_emulator_param, grid_fault_param, device_igbt_param,
%[text] check_model_param_refs). The library is needed for the Simulink/Simscape blocks (modulator, T-type bridge,
%[text] split DC link, grid emulator, transformer, inductor, power meter, filters), for the C-Caller sources and for
%[text] the IGBT data sets other than the embedded one. Run it from this folder (the functions of ./private are
%[text] visible only to the files of this folder). The legacy flat init (shared template, seven clock domains,
%[text] DAB/PSFBC/CLLC/machines) is kept as init_model_legacy.m for reference; it no longer runs with the library
%[text] (grid_fault_generator is now a function with arguments).

clear;
model = 'afe_dq_ttype';

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
param.sim.tc = 0.625e-6;                    % [s] logging base, model MaxStep (ode23tb) and step of the Simscape
                                            %     local solver (glb_time.tc): ts/200 at fpwm = 4 kHz, double update
param.sim.use_thermal_model = 1;            % the active bridge is the IGBT T-type with the loss and thermal model
                                            % (the ideal-switch bridge is commented out in the model): the flag only
                                            % sets the Simscape nonlinear iterations, as the legacy init
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration (block grid/grid_emulator/Solver Configuration)
if param.sim.use_thermal_model
    param.sim.nonlinear_iterations = 10;     % legacy value; the new T-type blocks of semiconductors_bridges need 20
end
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
% one converter clock domain (bridge and control): 4 kHz PWM, double update -> ts = 125 us
param.time.fpwm = 4e3;                              % [Hz] switching frequency of the T-type bridge
param.time.trgo = 1;                                % double update: control at 2*fpwm = 8 kHz
param.time.dead_time = 3e-6;                        % [s] legacy dead_time_afe, read by deadtime_generator_for_ttype
param.time.t_measure = param.sim.simlength;         % [s] logging window
param.time.tc_factor = 1/param.time.fpwm/param.sim.tc/2;    % ts/tc = 200 (legacy tc_factor), glb_time.tc = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;

param.glb_time = timing_param(param.time);
%[text] ## Grid
% three-phase grid emulator (ideal voltage sources) feeding a Dyn11 1 MVA 690/690 V transformer made of three
% single-phase transformer blocks (star point of the secondary grounded); the AFE filter is connected to the
% secondary. No fault applied (both enables at 0): the fault data are those of the legacy test 25.4 (75 % dip on
% two phases, variant D, from 0.75 s for 915 ms)
param.grid.application_voltage = 690;               % [V] rms line-to-line, primary and secondary of the transformer
param.grid.f_nom = 50;                              % [Hz]
param.grid.omega_nom = 2*pi*param.grid.f_nom;       % [rad/s]
param.grid.pwr_nom = 1000e3;                        % [VA] rated power of the transformer
param.grid.ucc = 5;                                 % [%] short-circuit voltage of the transformer
param.grid.eta = 95;                                % [%] efficiency of the transformer (copper losses)
param.grid.p_iron = 1800;                           % [W] iron losses of the transformer
param.grid.n2 = 14;                                 % turns of the secondary (star) coils of the transformer blocks
param.grid.n1 = floor(param.grid.n2*sqrt(3));       % turns of the primary (delta) coils: 24 (ratio 1.714, not sqrt(3):
                                                    % the secondary is at 697 V, +1.0 %)
param.grid.core_area = 0.05;                        % [m^2] core of the transformer blocks
param.grid.core_length = 2.5;                       % [m]
param.grid.mur = 10e3;                              % [-] relative permeability: Lm1 = 145 mH, i1m = 8.8 A
param.grid.R_eq = 2e-3;                             % [Ohm] equivalent grid resistance (informative, not in the model)
param.grid.frt = grid_fault_param('two_phase_D', 0.75, 0.915, 0.75, 0, 0);
param.grid.emu = grid_emulator_param(param.grid, param.grid.frt);
%[text] ## Three-phase T-type AFE
%[text] ### Hardware
% 250 kW data set of the library at 690 V: ubez 563.4 V, ibez 328.7 A (10 % margin on the power), udc_nom 1070 V;
% split DC link 2 x 14.4 mF, filter 500 uH + 200 uF in star
param.afe.hw = afe_hw_param(param.grid.application_voltage, 250e3, param.time.fpwm);
%[text] ### PWM
% space vector modulation (C-Caller sv_pwm_cm: the dq PI output is scaled by sqrt(3)/2, so 1 pu = udc/sqrt(3),
% the linear limit) -> duties 0..1 -> three-level modulator of the library (three_phase_three_levels_PWM_modulator,
% bus fields mu(k), mv(k), mw(k), 2d - 1 internally, carriers at fpwm, control trigger at the bottom and at the top
% of the carrier) -> dead-time generator of the model (gates of Q1/Q3 and Q2/Q4 of each leg)
param.afe.pwm.fpwm = param.glb_time.fPWM_AFE;       % [Hz]
param.afe.pwm.double_update = param.glb_time.TRGO_AFE_double_update;
param.afe.pwm.dead_time = param.glb_time.dead_time_AFE;    % [s]
param.afe.pwm.tc = param.glb_time.tc;               % [s]
%[text] ### Behavioural settings (module 1)
param.afe.behav.time_gain_module_1 = 1.0002;        % [-] local clock gain of the module against the master time
param.afe.behav.pwm_phase_shift_mod1 = 0;           % [-] phase of the carrier (modulator input "phase")
param.afe.behav.u_cm_comp_mod1 = 0;                 % [-] gain of the zero-sequence voltage of the controller into the
                                                    %     SVM (ugamma: k_offset = (ugamma + 1)/2); legacy u_cm_comp_mod1,
                                                    %     -1 for the modules 2..4 of the hard-parallel setup
%[text] ### Variants of the model
% PLL whose phase and frequency feed the control (the four PLLs always run, Variant Source1/2 pick one):
% mode1 Simulink dq PLL, mode2 C-Caller dq PLL, mode3 Simulink FHT-PLL, mode4 C-Caller FHT-PLL (legacy
% use_dq_pll_fht_pll = 1, the others 0)
param.afe.variant.dq_pll_mode1 = 0;                 % (variant) dq_pll
param.afe.variant.dq_pll_mode2 = 0;                 % (variant) dq_pll_ccaller
param.afe.variant.dq_pll_mode3 = 0;                 % (variant) FHT-PLL_simulink
param.afe.variant.dq_pll_mode4 = 1;                 % (variant) FHT-PLL (C-Caller)
if param.afe.variant.dq_pll_mode1 + param.afe.variant.dq_pll_mode2 + param.afe.variant.dq_pll_mode3 + ...
        param.afe.variant.dq_pll_mode4 ~= 1
    error('%s init: exactly one PLL variant must be active', model);
end
%[text] ### Control
% gains of the legacy init: the dq current PI read the PMSM inverter gains (psm_ctrl.kp_i = 0.25, ki_i = 35), the
% udc PI overrode the library defaults 0.85/35 of ctrl_afe_setup with 4/45; the PLLs, the zero-sequence PI and the
% current limit are the library values
ctrl_in.kp_i = 0.25;                                % [pu/pu] dq vector PI of the converter current
ctrl_in.ki_i = 35;                                  % [1/s]
ctrl_in.clip_release = 1e-3;                        % [1/s] legacy CTRPIFF_CLIP_RELEASE (integrator decay while clipped)
ctrl_in.kp_udc = 4;                                 % [pu/pu] DC-link voltage PI
ctrl_in.ki_udc = 45;                                % [1/s]
ctrl_in.udc_pu_ref = 1;                             % [pu] of udc_nom (legacy afe_ctrl.udc_norm_ref)
ctrl_in.i_ref_lim = 1.65;                           % [pu] of ibez, limit of the active current reference (legacy ixi_pos_ref_lim)
ctrl_in.kp_is_z = 0.5;                              % [pu/pu] zero-sequence current PI
ctrl_in.ki_is_z = 18;                               % [1/s]
ctrl_in.pll.kp = 1;                                 % PLLs (legacy afe_ctrl.pll_p, pll_i1, pll_p_frt; FHT-PLL: same kp, ki)
ctrl_in.pll.ki = 80;                                % [1/s]
ctrl_in.pll.kp_frt = 0.2;
ctrl_in.align.kp = 0.25;                            % local time alignment to the master time (legacy kp_align, ...)
ctrl_in.align.ki = 18;                              % [1/s]
ctrl_in.align.lim_up = 0.05;
ctrl_in.align.lim_down = -0.05;
param.afe.ctrl = afe_ctrl_param(param.glb_time.ts_afe, param.grid.omega_nom, ctrl_in);
clear ctrl_in
%[text] ## DC load
% resistor across the whole DC link (block load): 275 kW at udc_nom (legacy literal 1070^2/275e3 in the block)
param.load.pwr_nom = 275e3;                         % [W]
param.load.R = param.afe.hw.udc_nom^2/param.load.pwr_nom;  % [Ohm] 4.16 Ohm
%[text] ### Design figures (informative)
% operating point at the load power (the converter losses are not included), grid-side unity power factor (the
% q reference of the converter current is 0, the PLL reads the voltage at the transformer secondary); the
% converter voltage adds the drop on LFu in quadrature; SVM: linear up to udc/sqrt(3) phase peak.
% LCL: LFu - CFu (star) - leakage of the transformer referred to the secondary; with the converter current fed
% back and the 1.5-sample delay of the digital control, the resonance should stay below fs/6 (fs = 1/ts) for a
% stable loop without active damping. Neutral point: centred SVM -> phase modulating signals m' = 2*u_x0/udc
% (min-max injection); the neutral-point current is sum((1 - |m'|)*i) (no balancing loop, floating np).
hw = param.afe.hw; w = param.grid.omega_nom; tr = param.grid.emu.trafo; ts = param.glb_time.ts_afe;
d.P = param.load.pwr_nom;                                           % [W]
d.u_peak = hw.ubez*tr.n2/tr.n1*sqrt(3);                             % [V] peak phase voltage at the secondary (n1/n2 = 24/14)
d.i_peak = d.P/(3/2*d.u_peak);                                      % [A] peak current
d.i_pu = d.i_peak/hw.ibez;                                          % [pu] of ibez
d.u_conv_peak = sqrt(d.u_peak^2 + (w*hw.LFu*d.i_peak)^2);           % [V] converter phase voltage
d.svm_limit = hw.udc_nom/sqrt(3);                                   % [V] linear limit of the SVM
d.u_margin = d.svm_limit/d.u_conv_peak - 1;                         % [-] voltage headroom of the current controller
d.u_grid_max_no_load = d.svm_limit/d.u_peak - 1;                    % [-] grid overvoltage that saturates the SVM at no load
d.L_grid = tr.Ld2 + tr.Ld1*(tr.n2/tr.n1)^2;                         % [H] transformer leakage referred to the secondary (per phase)
d.f_res = sqrt((hw.LFu + d.L_grid)/(hw.LFu*d.L_grid*hw.CFu))/2/pi;  % [Hz] LCL resonance
d.f_crit = 1/ts/6;                                                  % [Hz] fs/6
d.k_i = (hw.udc_nom/sqrt(3))/(hw.ibez*(hw.LFu + d.L_grid));         % [1/s] plant gain of the current loop (pu/pu), LCL as L1 + L2
d.f_c_i = param.afe.ctrl.kp_i*d.k_i/2/pi;                           % [Hz] crossover estimate of the current loop
d.k_udc = hw.norm.Pbez/(hw.CFi*hw.udc_nom^2);                       % [1/s] plant gain of the udc loop (pu of udc per pu of current)
d.a_load = 2/(param.load.R*hw.CFi);                                 % [rad/s] pole of the DC link with the resistive load
d.f_c_udc = param.afe.ctrl.kp_udc*d.k_udc/2/pi;                     % [Hz] crossover estimate of the udc loop
th = linspace(0, 2*pi, 3601); th(end) = [];
d.delta = atan(w*hw.LFu*d.i_peak/d.u_peak);                         % [rad] lag of the converter voltage on the current
m_abc = 2*d.u_conv_peak/hw.udc_nom*[sin(th - d.delta); sin(th - d.delta - 2*pi/3); sin(th - d.delta + 2*pi/3)];
m_abc = m_abc - (max(m_abc) + min(m_abc))/2;                        % centred SVM = min-max injection
i_abc = -d.i_peak*[sin(th); sin(th - 2*pi/3); sin(th + 2*pi/3)];    % current out of the bridge (rectifier)
d.m_peak = max(abs(m_abc(1,:)));                                    % [-] peak of the three-level modulating signal
d.i_np = sum((1 - abs(m_abc)).*i_abc, 1);                           % [A] neutral-point current (carrier average)
q_np = cumsum(d.i_np)*(2*pi/numel(th))/w;                           % [C]
d.du_np_pp = (max(q_np) - min(q_np))/(hw.CFi1 + hw.CFi2);           % [V] peak-to-peak np ripple (3rd harmonic), no balancing
d.di_sw_max = (hw.udc_nom/2)/(4*param.time.fpwm*hw.LFu);            % [A] worst-case pk-pk switching ripple of the current (udc/2 steps)
param.afe.design = rmfield(d, 'i_np');
fprintf('Design at %.0f kW: %.0f A peak (%.2f pu of ibez), converter voltage %.0f V peak, SVM limit %.0f V: headroom %.1f %%, grid overvoltage +%.1f %% saturates at no load\n', ...
    d.P/1e3, d.i_peak, d.i_pu, d.u_conv_peak, d.svm_limit, d.u_margin*100, d.u_grid_max_no_load*100);
fprintf('LCL: LFu %.0f uH, CFu %.0f uF, transformer %.1f uH: resonance %.0f Hz (fs/6 = %.0f Hz) | current loop crossover estimate %.0f Hz | udc loop crossover estimate %.0f Hz (load pole %.0f rad/s)\n', ...
    hw.LFu*1e6, hw.CFu*1e6, d.L_grid*1e6, d.f_res, d.f_crit, d.f_c_i, d.f_c_udc, d.a_load);
fprintf('Neutral point: peak modulating signal %.2f, np current %.0f A peak (3rd harmonic) -> %.2f V pk-pk on 2 x %.1f mF, no balancing loop | switching ripple %.0f A pk-pk\n', ...
    d.m_peak, max(abs(d.i_np)), d.du_np_pp, hw.CFi1*1e3, d.di_sw_max);
clear hw w tr ts d th m_abc i_abc q_np

%[text] ## Power semiconductors
% IGBT module Mitsubishi CM1200DW-34T (1700 V / 1200 A, energies at 1000 V / 1200 A, 125 degC, Rg = 1 Ohm) for
% the twelve switches of the T-type bridge (four per leg: two outer, two in the bidirectional middle path, the
% same data for both). The data set is embedded in private/device_igbt_param; any other data set of the library
% is read through device_data_load.
param.dev.used_device_igbt = 'mitsubishi_CM1200DW_34T';
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
param.thermal.Rth_mosfet_HA = param.thermal.Rth_switch_HA;      % [K/W]
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;       % [K/W]
param.thermal.Tambient = 40;                                    % [degC] water temperature
param.thermal.DThs_init = 0;                                    % [K] initial heatsink rise

% heatsink object read by the bridge mask (fields Rth_switch_HA, Rth_diode_HA, no_weight, cp, Tambient, DThs_init)
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

% loss and junction temperature estimate of one leg at the design point (informative, averaged over the carrier
% period, rectifier operation: power from the grid to the DC link). Conduction: state P = Q1 (current out of
% the leg) or D1 (current into the leg); state N = D4 (out) or Q4 (in); state O = Q2 + D3 (out) or Q3 + D2 (in).
% Switching, each commutation at udc/2 (energies scaled linearly in voltage and current from the data set
% point): for m > 0 the leg moves between P and O, Q1 + recovery of D3 (current out) or Q3 + recovery of D1
% (current in); for m < 0 between N and O, Q2 + recovery of D4 (out) or Q4 + recovery of D2 (in).
% Conduction drops: v = Vce_sat + Rce_on*i, vd = Vdon_diode + Rdon_diode*i.
g = param.afe.igbt; hw = param.afe.hw; dsg = param.afe.design; fsw = param.time.fpwm;
th = linspace(0, 2*pi, 3601); th(end) = [];
m3 = 2*dsg.u_conv_peak/hw.udc_nom*[sin(th - dsg.delta); sin(th - dsg.delta - 2*pi/3); sin(th - dsg.delta + 2*pi/3)];
m = m3(1,:) - (max(m3) + min(m3))/2;                                % phase u, -1..1 of udc/2
i = -dsg.i_peak*sin(th);                                            % current out of the leg
ia = abs(i);
dP = max(m, 0); dN = max(-m, 0); dO = 1 - dP - dN;
vq = g.Vce_sat*ia + g.Rce_on*ia.^2;                                 % [W] conduction power of an IGBT while on
vd = g.Vdon_diode*ia + g.Rdon_diode*ia.^2;                          % [W] of a diode
eq = fsw*(g.Eon + g.Eoff)*(hw.udc_nom/2)/g.Voff_sw_losses*ia/g.Ion_sw_losses;   % [W] switching power of an IGBT
ed = fsw*g.Erec*(hw.udc_nom/2)/g.Voff_sw_losses*ia/g.Ion_sw_losses;            % [W] recovery power of a diode
out = i > 0; pos = m > 0;
p.Q1 = mean(dP.*vq.*out) + mean(eq.*pos.*out);
p.D1 = mean(dP.*vd.*~out) + mean(ed.*pos.*~out);
p.Q2 = mean(dO.*vq.*out) + mean(eq.*~pos.*out);
p.D2 = mean(dO.*vd.*~out) + mean(ed.*~pos.*~out);
p.Q3 = mean(dO.*vq.*~out) + mean(eq.*pos.*~out);
p.D3 = mean(dO.*vd.*out) + mean(ed.*pos.*out);
p.Q4 = mean(dN.*vq.*~out) + mean(eq.*~pos.*~out);
p.D4 = mean(dN.*vd.*out) + mean(ed.*~pos.*out);
p.leg = p.Q1 + p.Q2 + p.Q3 + p.Q4 + p.D1 + p.D2 + p.D3 + p.D4;     % [W]
p.bridge = 3*p.leg;                                                 % [W]
p.filter = 3*hw.RLFu*(dsg.i_peak/sqrt(2))^2;                        % [W] copper losses of LFu
p.Tj_Q3 = param.thermal.Tambient + p.Q3*(g.Rth_switch_JH + param.thermal.Rth_switch_HA);   % [degC] middle IGBT
p.Tj_D1 = param.thermal.Tambient + p.D1*(g.Rth_diode_JH + param.thermal.Rth_diode_HA);     % [degC] outer diode
p.Tj_D2 = param.thermal.Tambient + p.D2*(g.Rth_diode_JH + param.thermal.Rth_diode_HA);     % [degC] middle diode
p.efficiency = dsg.P/(dsg.P + p.bridge + p.filter);
param.thermal.loss_estimate = p;
fprintf('T-type losses at %.0f A peak, %.0f V, %.0f kHz (rectifier): outer diode %.0f W, middle IGBT %.0f W, middle diode %.0f W, outer IGBT %.1f W; bridge %.0f W, filter %.0f W; Tj %.0f / %.0f / %.0f degC; efficiency %.2f %%\n', ...
    dsg.i_peak, hw.udc_nom, fsw/1e3, p.D1, p.Q3, p.D2, p.Q1, p.bridge, p.filter, p.Tj_D1, p.Tj_Q3, p.Tj_D2, p.efficiency*100);
clear g hw dsg fsw th m3 m i ia dP dN dO vq vd eq ed out pos p

%[text] ## Model setup
open_system(model);
Simulink.importExternalCTypes(model, 'Names', {'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'dqpll_grid_output_t'});
Simulink.importExternalCTypes(model, 'Names', {'sv_pwm_cm_output_t'});
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