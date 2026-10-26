%[text] # Three-leg interleaved buck converter
%[text] Initialization of interleaved_buck.slx (discrete blocks) and of interleaved_buck_cmp.slx (the compact block
%[text] interleaved_buck_3legs_mod1 of the library with the same battery, timing and load; build_interleaved_buck_compact.m
%[text] generates the minimal interleaved_buck_compact.slx around the same block): battery on the DC link, three buck legs
%[text] with one inductor each and interleaved carriers (0, 120, 240 deg), common output capacitor, resistive load. Output
%[text] voltage loop giving the total current reference, one PI per leg on the leg current (reference i_ref/3: inherent
%[text] current sharing), each leg current sampled at the extremes of its own carrier (average value, no ripple), control
%[text] step at the extremes of carrier 1 (double update, 6 kHz at fpwm = 3 kHz).
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, buck_ctrl_param, device_igbt_param, battery_param, check_model_param_refs). The library is
%[text] needed for the Simulink/Simscape blocks (modulator, bridge, DC link, battery), for the compact block and for the
%[text] IGBT data set (setup_files/semiconductor_devices_data, read by device_igbt_param through device_data_load).
%[text] Run it from this folder (the functions of ./private are visible only to the files of this folder).
%[text] param.buck is also the mask struct p of the compact block (p.hw, p.pwm, p.ctrl): numeric fields only.

clear;
model = 'interleaved_buck';

% Simulink/Simscape library blocks (blocks, not parameters) and device data sets
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
param.sim.simlength = 0.4;                  % [s] model StopTime
param.sim.tc = 1/(2*3e3)/200;               % [s] 0.833 us: logging base, model MaxStep (ode23tb) and step of the Simscape
                                            %     local solver (glb_time.tc): ts/200 at fpwm = 3 kHz with double update
param.sim.use_thermal_model = 1;            % IGBT bridge with the simple two-thermal-port loss model
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration
if param.sim.use_thermal_model
    param.sim.nonlinear_iterations = 5;
end
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
param.time.fpwm = 3e3;                              % [Hz] switching frequency of each leg (the output sees n_legs*fpwm = 9 kHz)
param.time.trgo = 1;                                % double update: control at 2*fpwm = 6 kHz, at the bottom and top of carrier 1
param.time.dead_time = 3e-6;                        % [s] (FF450R17ME4: td_off + tf = 1.6 us at 150 degC)
param.time.t_measure = 0.4;                         % [s] logging window
param.time.tc_factor = 1/param.time.fpwm/param.sim.tc/2;    % ts/tc = 200, so that glb_time.tc = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;

% sample time of the battery state-of-charge estimator (legacy ts_dab of the shared template: 3*fpwm, double update)
param.time.fpwm_dab = 3*param.time.fpwm;            % [Hz]
param.time.trgo_dab = 1;

param.glb_time = timing_param(param.time);
%[text] ## Buck converter
%[text] ### Operating point
% 660 V battery to a 200 V DC bus, 750 A (150 kW): 250 A per leg, duty 0.303. Bases of the pu signals:
% ubez = u_out_nom (output voltage), ibez = total nominal current (a leg carries 1/3 pu).
param.buck.hw.n_legs = 3;
param.buck.hw.udc_nom = 660;                        % [V] DC link = battery
param.buck.hw.u_out_nom = 200;                      % [V] output
param.buck.hw.i_out_nom = 750;                      % [A] output current (total of the three legs)
param.buck.hw.pwr_nom = param.buck.hw.u_out_nom*param.buck.hw.i_out_nom;    % [W] 150 kW
param.buck.hw.ubez = param.buck.hw.u_out_nom;       % [V] base of the output voltage
param.buck.hw.ibez = param.buck.hw.i_out_nom;       % [A] 750 A, base of all currents (total)
%[text] ### Sizing of the leg inductors and of the output capacitor
% Design inputs (the specifications) and the formulas; the chosen values follow, rounded from the minima.
spec.r_i_leg = 0.25;        %      max peak-peak ripple of the leg current / nominal leg current, at the nominal duty
spec.r_u_out = 0.2e-2;      %      max peak-peak ripple of the output voltage / nominal voltage (0.2 %), capacitor alone
spec.f_c_i = 225;           % [Hz] crossover target of the per-leg current loops: fs/26.7 at fs = 6 kHz, the ratio of the
                            %      4 kHz version (300 Hz at 8 kHz), so the delays (1.5 ts leg 1, 2.5 ts legs 2-3) leave
                            %      the same phase margins (about 58 and 45 deg)
spec.f_c_u = 60;            % [Hz] crossover target of the output voltage loop at nominal load
spec.k_f0 = 2;              %      the LC resonance of the common mode (L/n_legs with Cout) must stay k_f0 below f_c_i:
                            %      inside the current loops, which damp it; above them the loops cross the undamped
                            %      resonance a second time (2026-10-26 check: 14 deg of phase margin at no load with the
                            %      capacitor sized on the ripple only, 0.8 mF, f0 = 345 Hz)
hw = param.buck.hw;
n  = hw.n_legs;
D  = hw.u_out_nom/hw.udc_nom;                                               % duty at the operating point, 0.303
i_leg = hw.i_out_nom/n;                                                     % [A] 250 A
% leg inductor: triangular ripple of one leg  dI_leg = Udc*D*(1-D)/(L*fpwm)
L_min = hw.udc_nom*D*(1 - D)/(param.time.fpwm*spec.r_i_leg*i_leg);          % [H] 0.743 mH for 25 % of 250 A
% output capacitor, two criteria:
% (a) ripple: the three leg ripples add with the 120 deg shift to a triangular current at n*fpwm,
%     dI_tot(D) = n*Udc*(D - k/n)*((k+1)/n - D)/(L*fpwm), k = floor(n*D) (zero at D = k/n, 1/3 here: at the
%     nominal duty the three ripples nearly cancel), worst case over the duty (n*D - k = 1/2, e.g. with the
%     battery at a different voltage or a lower output) dI_tot_max = Udc/(4*n*L*fpwm); the capacitor takes it,
%     dU = dI_tot/(8*n*fpwm*C)  ->  C_ripple = dI_tot_max/(8*n*fpwm*dU_max)
% (b) control: f0 = 1/(2*pi*sqrt(L/n*C)) <= f_c_i/k_f0  ->  C_ctrl = n*(k_f0/(2*pi*f_c_i))^2/L
%     (b) dominates by far: with C at the ripple minimum the voltage plant gain also changes 12x between no load
%     and nominal load (load corner 1/(2*pi*R_load*C) = 750 Hz against the 60 Hz voltage loop), so the cascade
%     without a load-current feed-forward is either unstable at no load or 1 Hz slow at nominal load. With C_ctrl
%     the load corner is 80 Hz and the ripple is 0.02 % worst case.
L_choice = 0.8e-3;                                                          % [H] L_min rounded up to 0.1 mH
dI_tot_max = hw.udc_nom/(4*n*L_choice*param.time.fpwm);                     % [A] 22.9 A peak-peak at 9 kHz
C_ripple = dI_tot_max/(8*n*param.time.fpwm*spec.r_u_out*hw.u_out_nom);     % [F] 0.80 mF (0.26 mF at the nominal duty only)
C_ctrl = n*(spec.k_f0/(2*pi*spec.f_c_i))^2/L_choice;                        % [F] 7.5 mF
fprintf('Sizing: D = %.3f, %.0f A per leg; L_min = %.3f mH for %.0f %% of ripple; C_ripple = %.2f mF for %.1f %% of output ripple (worst duty), C_ctrl = %.2f mF for f0 <= f_c_i/%.0f\n', ...
    D, i_leg, L_min*1e3, spec.r_i_leg*100, C_ripple*1e3, spec.r_u_out*100, C_ctrl*1e3, spec.k_f0);
%[text] ### Hardware
% leg inductors: one separate inductor per leg (a three-limb coupled reactor is wrong here: the three leg
% currents are DC and in phase, the core sees only the zero-sequence inductance). 0.8 mH: 58 A peak-peak of
% ripple per leg at D = 0.303 (23 % of the leg current), the three ripples add to 7.6 A peak-peak at 9 kHz
% at the nominal duty (22.9 A worst case over the duty)
param.buck.hw.L = L_choice;                         % [H] per leg
param.buck.hw.RL = 6e-3;                            % [Ohm] per leg: 375 W per leg at 250 A, 0.75 % of Pnom (assumption)
param.buck.hw.kL = [1 1 1];                         % per-leg factors of L and RL (mismatch studies: the per-leg loops
param.buck.hw.kRL = [1 1 1];                        % keep the sharing, a common duty would not)
% output capacitor: max(C_ripple, C_ctrl) = 7.5 mF in three parts of 2.5 mF (the model has one capacitor Cout/3
% with 3*RCout on each leg output, the compact block one capacitor Cout with RCout); 112 Hz resonance with L/3,
% zeta 0.35 at full load; 14 mV of ripple at 9 kHz at the nominal duty, 42 mV worst case (0.02 %)
param.buck.hw.Cout = 7.5e-3;                        % [F] total
param.buck.hw.RCout = 3e-3/3;                       % [Ohm] ESR of the total capacitor (3 mOhm per part)
% DC-link capacitor (block dclink_for_two_levels in the discrete model, inside the ssc block in the compact one):
% the input current is a pulse train at n*fpwm (the ON pulses of the three legs do not overlap at D < 1/3), the
% capacitor takes 72 A rms and the voltage ripple is dU_dc = D'*(1-D')*i_leg/(n*fpwm*Cdc), D' = n*D - k: 0.3 V
param.buck.hw.Cdc = 7.2e-3;                         % [F]
param.buck.hw.Rdc = 1e-3;                           % [Ohm] ESR (compact block)
param.buck.hw.Vdc0 = param.buck.hw.udc_nom;         % [V] initial DC-link voltage (compact block)
% switches of the compact block (numerical switch + antiparallel diode, no losses): the discrete model has the
% IGBT bridge with the loss and thermal model (param.buck.igbt, param.thermal)
param.buck.hw.Vth = 2.5;                            % [V]   gate threshold
param.buck.hw.Ron = 7e-3;                           % [Ohm] on-state resistance: Vce/I of the FF450R17ME4 at 250 A, 125 degC (1.72 V)
param.buck.hw.Goff = 1e-6;                          % [1/Ohm]

% design figures (informative)
hw = param.buck.hw;
d.D = D;                                                                       % duty at the operating point
d.i_leg = i_leg;                                                               % [A]
d.L_min = L_min;                                                               % [H] from spec.r_i_leg
d.C_ripple = C_ripple;                                                         % [F] from spec.r_u_out, worst duty
d.C_ctrl = C_ctrl;                                                             % [F] from spec.f_c_i and spec.k_f0
d.ripple_leg = hw.udc_nom*D*(1 - D)/(hw.L*param.time.fpwm);                   % [A] peak-peak per leg
k = floor(n*D);                                                                % interleaving: ripple of the sum
d.ripple_tot = hw.udc_nom/(hw.L*param.time.fpwm)*(D - k/n)*((k + 1)/n - D)*n; % [A] pk-pk at n*fpwm, nominal duty
d.ripple_tot_max = hw.udc_nom/(4*n*hw.L*param.time.fpwm);                     % [A] pk-pk at n*fpwm, worst duty
d.ripple_u_out = d.ripple_tot/(8*n*param.time.fpwm*hw.Cout);                  % [V] peak-peak (capacitor only), nominal duty
d.ripple_u_out_max = d.ripple_tot_max/(8*n*param.time.fpwm*hw.Cout);          % [V] peak-peak, worst duty
Dp = n*D - k;                                                                  % duty of the DC-side pulse train at n*fpwm
d.ripple_u_dc = Dp*(1 - Dp)*i_leg/(n*param.time.fpwm*hw.Cdc);                 % [V] peak-peak on the DC link (capacitor only)
d.i_cdc_rms = i_leg*sqrt(Dp*(1 - Dp));                                        % [A] rms current of the DC-link capacitor
d.f0 = 1/(2*pi*sqrt(hw.L/n*hw.Cout));                                         % [Hz] LC resonance, common duty
d.R_load_nom = hw.u_out_nom^2/hw.pwr_nom;                                      % [Ohm]
d.zeta = sqrt(hw.L/n/hw.Cout)/(2*d.R_load_nom);                                % damping at nominal load
d.f_load = 1/(2*pi*d.R_load_nom*hw.Cout);                                      % [Hz] load corner of the voltage plant
param.buck.hw.design = d;
fprintf('Buck: D = %.3f, %.0f A per leg, ripple %.0f A pk-pk per leg (%.0f %%), %.1f A pk-pk total at %.0f kHz (%.1f A worst duty), u_out ripple %.0f mV (%.0f mV worst duty, %.3f %%), udc ripple %.2f V, f0 = %.0f Hz, zeta = %.2f at full load, load corner %.0f Hz\n', ...
    d.D, d.i_leg, d.ripple_leg, d.ripple_leg/d.i_leg*100, d.ripple_tot, n*param.time.fpwm/1e3, d.ripple_tot_max, ...
    d.ripple_u_out*1e3, d.ripple_u_out_max*1e3, d.ripple_u_out_max/hw.u_out_nom*100, d.ripple_u_dc, d.f0, d.zeta, d.f_load);
clear hw d k n D Dp i_leg L_min L_choice dI_tot_max C_ripple C_ctrl
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
% same limits). Gains from the crossover targets (private/buck_ctrl_param): spec.f_c_i = 225 Hz for the current loops
% (fs = 6 kHz: 1.5 ts of delay on leg 1, 2.5 ts on legs 2 and 3, phase margins about 58 and 45 deg; the LC resonance
% at 112 Hz is inside the loops), spec.f_c_u = 60 Hz for the voltage loop at nominal load, where the plant is the output
% capacitor with the load (corner 80 Hz) behind the closed common-mode current loop (gain 0.75 at that frequency: the
% capacitor reacts to a common duty), and about 100 Hz without load. Linear check of 2026-10-26
% (claude_analysis/interleaved_buck_loop_analysis.py, discrete model at ts): current loops crossover 234 Hz on the
% inductor alone (276 Hz in common mode, with the capacitor) with 59 deg of phase margin on leg 1, 43 deg and 7.4 dB
% of gain margin on legs 2-3 at nominal load (40 deg at no load); voltage loop 62 Hz / 123 deg at nominal load,
% 143 Hz / 73 deg at no load; 50 ms reference ramp without overshoot, 6.9 % dip on a +0.2 pu load step recovered
% within 1 % in 44 ms.
param.buck.ctrl = buck_ctrl_param(param.buck.hw, param.glb_time.ts_afe, spec.f_c_i, spec.f_c_u);
param.buck.ctrl.ts = param.glb_time.ts_afe;         % [s]   control period (compact block)
param.buck.ctrl.mode = 0;                           %       compact block: 0 voltage loop, 1 total current reference (i_ref, t_ref, i_ref_slew)
param.buck.ctrl.u_out_ref = 1;                      % [pu]  output voltage reference (pu of ubez)
param.buck.ctrl.u_ref_slew = 20;                    % [pu/s] ramp of the reference at start: 1 pu in 50 ms
param.buck.ctrl.i_lim = 1.2;                        % [pu]  limit of the total current reference (900 A, 300 A per leg), also the integrator clamp
param.buck.ctrl.i_ref = 1;                          % [pu]  mode 1 of the compact block: total current reference
param.buck.ctrl.t_ref = 0;                          % [s]   step time of i_ref
param.buck.ctrl.i_ref_slew = 20;                    % [pu/s]
param.buck.ctrl.adc_q = 1/2^11;                     % [pu]  quantization of the acquisitions (compact block; param.meas.adc_quantization in the model)
param.buck.ctrl.k_time = 1;                         %       local clock of the compact block = master time
param.buck.ctrl.kp_align = 0;                       %       alignment PI of the local clock (compact block), off
param.buck.ctrl.ki_align = 0;
param.buck.ctrl.lim_up_align = 0;
param.buck.ctrl.lim_down_align = 0;
param.buck.spec = spec;                             % the design inputs above (numeric), kept with the rest of param.buck
clear spec
%[text] ### Module behaviour (clock mismatch)
param.buck.behav.time_gain_module_1 = 1;            % local clock of the module versus master time ("global timing"); 1: no mismatch
%[text] ## Power semiconductors
% 1700 V module for the 660 V link: Infineon FF450R17ME4 (EconoDUAL 3, 450 A nominal, 900 A ICRM, IGBT4): one half
% bridge per leg at 250 A (300 A at i_lim), data set of the library (setup_files/semiconductor_devices_data/
% devices_source_dataname/infineon_FF450R17ME4.m, values at 125 degC, loaded by private/device_igbt_param through
% device_data_load). The two data sets embedded in device_igbt_param (FF1800R12IE5 of the 750 V version, CM1200DW-34T)
% stay available by name.
param.dev.used_device_igbt = 'infineon_FF450R17ME4';
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

% loss and junction temperature estimate at the operating point (informative, same model as the bridge: conduction
% v = V0 + R*i, switching energies scaled linearly from the data set point to udc and i_leg; the upper IGBT conducts for
% D of the period, the lower diode for 1 - D, ripple neglected)
g = param.buck.igbt; hw = param.buck.hw; D = hw.design.D; i_l = hw.design.i_leg;
p.igbt_cond = D*(g.Vce_sat*i_l + g.Rce_on*i_l^2);                                      % [W]
p.igbt_sw = (g.Eon + g.Eoff)*(i_l/g.Ion_sw_losses)*(hw.udc_nom/g.Voff_sw_losses)*param.time.fpwm;
p.diode_cond = (1 - D)*(g.Vdon_diode*i_l + g.Rdon_diode*i_l^2);
p.diode_rec = g.Erec*(i_l/g.Ion_sw_losses)*(hw.udc_nom/g.Voff_sw_losses)*param.time.fpwm;
p.leg = p.igbt_cond + p.igbt_sw + p.diode_cond + p.diode_rec;
p.Tj_igbt = param.thermal.Tambient + (p.igbt_cond + p.igbt_sw)*(g.Rth_switch_JH + param.thermal.Rth_switch_HA);   % [degC]
p.Tj_diode = param.thermal.Tambient + (p.diode_cond + p.diode_rec)*(g.Rth_diode_JH + param.thermal.Rth_diode_HA);
p.efficiency = hw.pwr_nom/(hw.pwr_nom + hw.n_legs*(p.leg + hw.RL*i_l^2));
param.thermal.loss_estimate = p;
fprintf('Losses per leg at %.0f A, %.0f V, %.0f kHz: IGBT %.0f W conduction + %.0f W switching, diode %.0f W conduction + %.0f W recovery = %.0f W; Tj IGBT %.0f degC, Tj diode %.0f degC with water at %.0f degC; efficiency %.1f %% with the inductors\n', ...
    i_l, hw.udc_nom, param.time.fpwm/1e3, p.igbt_cond, p.igbt_sw, p.diode_cond, p.diode_rec, p.leg, p.Tj_igbt, p.Tj_diode, param.thermal.Tambient, p.efficiency*100);
clear g hw D i_l p
%[text] ## Battery on the DC link
param.battery.nominal_battery_voltage_1 = param.buck.hw.udc_nom;  % [V] 660 V
param.battery.nominal_battery_power = param.buck.hw.pwr_nom;      % [W] 150 kW
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
param.load.R_load = param.buck.hw.design.R_load_nom;    % [Ohm] 0.267 Ohm: 150 kW at 200 V (block load/Resistor)
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
