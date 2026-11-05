%[text] # Power supply 25 V, 12 kA (ps\_25V\_12kA\_r)
%[text] Initialization of ps\_25V\_12kA\_r.slx. A 480 V, 60 Hz grid (emulator with fault generator and a Dyn11 1750 kVA transformer, both library blocks) feeds a two-level AFE of 325 kW (LCL filter, DC link 750 V, brake chopper) controlled in uvw by the library blocks afe\_uvw\_ctrl: resonant PI per phase, grid current sequences, FRT after VDE-AR-N 4110. The DC link supplies a phase-shift full bridge at 4 kHz with a centre-tapped transformer 34:2:2 and a two-diode rectifier, which drives 12 kA into a 2.08 mOhm load (25 V). Design document: docs/ps\_25V\_12kA\_r.pdf.
%[text] All the data are in one struct param, one group per part of the model: sim, time, glb\_time, grid, frt (fault test of the grid emulator), ctrl (settings common to the control blocks), afe, psfbc (phase-shift full bridge), diode, thermal, battery, load, meas, design. The model reads param.\* only; the README lists the old names. The functions of ./private are called through the handles of ps\_25V\_12kA\_private.m (prv.timing\_param, prv.afe\_hw\_param, ...): the Live Editor runs a live script from a copy outside this folder and does not see ./private. Run the whole script once (Run): a section run on its own needs prv and param in the workspace.
clear;
model = 'ps_25V_12kA_r';
prv = ps_25V_12kA_private();    % handles to the functions of ./private (the Live Editor does not see ./private)
prv.library_setup();            % pwr-control/library on the path (blocks, C-Caller sources), cloned when missing %[output:675b18a7]

% environment (was init_environment of the library)
close all; clc; beep off;
try
    pm_addunit('percent', 0.01, '1');       % custom Simscape unit used by some library blocks
catch ME
    if ~contains(ME.message, 'percent')     % already registered in this session: fine
        rethrow(ME);
    end
end
param = struct();
fprintf('%s init: functions of ./private, called as prv.<name>: %s\n', model, strjoin(fieldnames(prv)', ', ')); %[output:929bb977]
%%
%[text] ## Simulation
%[text] The full bridge has three variants in the model: ideal switches (default), IGBT with thermal model and MOSFET with thermal model (ZVS snubbers). The init comments out the two that are not used, together with the blocks that log their data. The AFE bridge is always the ideal-switch one.
param.sim.simlength = 1.75;                 % [s] model StopTime
param.sim.use_thermal_model = 0;            % 1: IGBT full bridge with thermal model (full_bridge_igbt_adv_thermal_model)
param.sim.use_mosfet_thermal_model = 0;     % 1: MOSFET full bridge with thermal model (has priority over the IGBT one)
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration: ideal switches
if param.sim.use_mosfet_thermal_model || param.sim.use_thermal_model
    param.sim.nonlinear_iterations = 5;     % semiconductor models with thermal network
end
param.sim.check_model = true;               % last section: param.* references and From blocks without a Goto
param.sim.design_figures = true;            % section "Design figures": operating point and loops, printed
%%
%[text] ## Timing
%[text] AFE and full bridge switch at 4 kHz with double update: their control runs at 8 kHz (ts 125 us). The step tc = ts\_afe/200 = 0.625 us is the logging base, the MaxStep of the solver and the step of the Simscape local solver. ts\_dab (12 kHz with double update) is a leftover of the legacy template: the low-pass filter and the PI of the DC transformer current compensation, and the battery block (commented out), are set with it, while the compensation runs at the full-bridge trigger (README, open points). The inverter domain (fpwm\_inv, trgo\_inv) of the template only gives ts\_inv to setup\_global\_filters: the model has no inverter.
param.time.fpwm_afe = 4e3;                  % [Hz] switching frequency of the AFE
param.time.trgo_afe = 1;                    % double update
param.time.dead_time_afe = 3e-6;            % [s]
param.time.fpwm_inv = 4e3;                  % [Hz] inverter domain of the template: only ts_inv for setup_global_filters
param.time.trgo_inv = 1;                    % double update
param.time.dead_time_inv = 3e-6;            % [s] (not read by the model)
param.time.fpwm_psfbc = 4e3;                % [Hz] switching frequency of the full bridge
param.time.trgo_psfbc = 1;                  % double update: two triggers per period
param.time.dead_time_psfbc = 3e-6;          % [s]
param.time.fpwm_dab = 3*param.time.fpwm_psfbc;      % [Hz] only for ts_dab
param.time.trgo_dab = 1;
param.time.t_measure = param.sim.simlength; % [s] logging window
param.time.tc_factor = 200;                 % tc = ts_afe/tc_factor
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;                   % [s] (not used by this model)
param.glb_time = prv.timing_param(param.time); %[output:5f132968]
%%
%[text] ## Grid
%[text] Grid emulator (block three\_phase\_voltage\_source\_with\_frt\_emulator1 of the library, MATLAB Function: three ideal voltage sources with the fault generator) and a Dyn11 transformer 480/480 V, 1750 kVA, ucc 5 percent: block configurable\_trafo of the library (one delta primary, star secondary with the star point grounded, three-limb core). There is no equivalent grid impedance any more, so the grid side of the AFE filter is the transformer leakage (17.5 uH star equivalent). The transformer block reads the per-unit data param.grid.emu.trafo.pu (r1, l1, r3, l3, lm, lm0, rfe, clock3) and Sr, V1, V3, fr from param.grid.emu.trafo, with the per-unit formulas of the class three\_phase\_transformer\_setup of the library. The magnetizing inductance comes from the core: Lm1 = mu0\*mur\*n1\*n1\*area/length = 145 mH on the delta coil of n1 = 24 turns (lm = 138 pu).
param.grid.application_voltage = 480;           % [V] rms line-to-line, primary and secondary
param.grid.f_nom = 60;                          % [Hz]
param.grid.pwr_nom = 1750e3;                    % [VA] rated power of the transformer
param.grid.ucc = 5;                             % [%] short-circuit voltage
param.grid.eta = 95;                            % [%] efficiency of the transformer (copper losses)
param.grid.p_iron = 1400;                       % [W] iron losses of the transformer
param.grid.n2 = 14;                             % turns of the secondary (star) coils
param.grid.n1 = floor(param.grid.n2*sqrt(3));   % turns of the primary (delta) coils: 24
param.grid.core_area = 0.05;                    % [m^2]
param.grid.core_length = 2.5;                   % [m]
param.grid.mur = 10e3;                          % [-] relative permeability: Lm1 = 145 mH
param.grid.lm0_pu = 0.5;                        % [pu] zero-sequence magnetizing inductance of the three-limb core
                                                % (lm0 = lm would be a bank of three single-phase units)
param.grid.emu = prv.grid_emulator_param(param.grid); %[output:8072d662]
%[text] ### Fault test of the grid emulator
%[text] Both fault generators are off, as in the legacy init. The data are those of the legacy test 25.4: 75 percent dip on two phases, variant D, 915 ms from 0.75 s. Set enable\_frt\_1 = 1 to apply it from the abc amplitudes, enable\_frt\_2 = 1 to apply it from the sequence references (deepPOSxi to deepNEGeta). The emulator and the FRT control of the AFE read param.frt.data; the FRT control is switched on at settle\_time when one of the two enables is 1.
param.frt.data = prv.grid_fault_param('two_phase_D', 0.75, 0.915, 0.75, 0, 0); %[output:0a0efb48]
param.frt.data.settle_time = 0.175;             % [s] FRT control of the AFE enabled from here (step FRT-VDE4110/enable)
param.frt.data.k_frt_ref = 2;                   % [pu/pu] reactive current per pu of voltage change (frt_vde4110)
%%
%[text] ## Control settings common to all modules
%[text] Local time alignment of the module to the master time (phase\_compensation\_1/alignment\_eval\_afe1/pi), moving averages of the sequence blocks, and the filters of the library: setup\_global\_filters gives param.ctrl.filters, of which the grid emulator reads the second-order low pass SOF\_50Hz on its amplitude ramps.
param.ctrl.kp_align = 0.25;                     % local time alignment PI
param.ctrl.ki_align = 18;                       % [1/s] (the model multiplies it by ts_afe)
param.ctrl.lim_up_align = 0.05;
param.ctrl.lim_down_align = -0.05;
param.ctrl.mavarage_filter_frequency_base_order = 2;   % moving average window = grid period/order (2: half a period)
param.ctrl.dmavg_filter_enable_time = 0.025;           % [s] the averages are zero before this time
param.ctrl.filters = setup_global_filters(param.glb_time.ts_afe, param.glb_time.ts_inv, ...
    param.glb_time.ts_dab, param.glb_time.tc);
%%
%[text] ## AFE (afe\_abc\_mod1)
%[text] Two-level bridge (ideal switches), LC filter 330 uH / 360 uF in star with 50 mOhm (the transformer leakage is the grid-side inductance), DC link CFi = 2.4 mF (the legacy init set it to twice the input capacitor of the full bridge, which is on the same bus), brake chopper 2.5 Ohm.
param.afe.pwr_nom = 325e3;                      % [W] rated power (bases: ibez = 614.2 A peak)
param.afe.hw_in.CFi = 2.4e-3;                   % [F] DC-link capacitor of the AFE
param.afe.hw_in.RCFi = 1e-3;                    % [Ohm] its series resistance (informative)
param.afe.hw = prv.afe_hw_param(param.grid.application_voltage, param.afe.pwr_nom, param.glb_time.fPWM_AFE, param.afe.hw_in); %[output:32b8bcfd]
%[text] ### Behaviour
param.afe.behav.time_gain_module_1 = 1.0002;            % [-] clock ratio of the local time of the module
param.afe.behav.pwm_phase_shift_mod1 = 0;               % [rad] carrier phase
param.afe.behav.time_aux_power_supply_fault = 1e3;      % [s] step of the brake command (after the end: off)
%[text] ### Control
%[text] The control runs in the triggered subsystem afe\_control at ts\_afe = 125 us: FHT-PLL in Simulink, DC-link voltage PI (udc\_ctrl) and the library blocks afe\_uvw\_ctrl (grid\_voltage\_sequences, grid\_current\_sequences\_q\_ctrl, frt\_vde4110, current\_ctrl\_uvw; islanding\_detector is commented out in the model), which read param.afe.uvw (section below). Gains of the library class ctrl\_afe\_setup, with kp\_udc\_ctrl = 2 of the legacy init and the resonant current PI kp = 0.6, ki = 35 of afe\_uvw\_three\_phase. The section "Design figures" prints the margins of the current and DC-link loops.
g = struct();
g.udc_norm_ref = 1;             % [pu] DC-link voltage reference (750 V)
g.kp_udc_ctrl = 2;              % [pu/pu] DC-link voltage PI (library 0.85)
g.ki_udc_ctrl = 35;             % [1/s]
g.ixi_pos_ref_lim = 1.65;       % [pu] limit of the active current reference
g.kp_rpi = 0.6;                 % [pu/pu] resonant PI per phase of the converter current (library 0.2)
g.ki_rpi = 35;                  % [1/s] (library 45)
g.kp_is_z = 0.5;                % [pu/pu] zero-sequence current PI
g.ki_is_z = 18;                 % [1/s]
g.kp_rc_pos_grid = 0.25;        % [pu/pu] PI of the positive-sequence reactive (eta) grid current
g.ki_rc_pos_grid = 35;          % [1/s]
g.kp_rc_neg_grid = 0.25;        % [pu/pu] PIs of the negative-sequence grid currents (xi and eta)
g.ki_rc_neg_grid = 35;          % [1/s]
g.ieta_pos_ref_lim = 1.0;       % [pu] limit of the positive-sequence reactive current reference
g.ieta_neg_ref_lim = 0.5;       % [pu] limit of the negative-sequence references
g.i_grid_pos_eta_lim = 1;       % [pu] FRT: limits of the grid current references
g.i_grid_neg_xi_lim = 0.5;
g.i_grid_neg_eta_lim = 0.5;
g.i_grid_pos_eta_red_lim = 0.1; % [pu] the same with reduced dynamic grid support
g.i_grid_neg_xi_red_lim = 0.1;
g.i_grid_neg_eta_red_lim = 0.1;
g.pll_p = 1;                    % FHT-PLL PI
g.pll_i1 = 80;                  % [1/s]
param.afe.ctrl = prv.afe_ctrl_param(param.glb_time.ts_afe, param.grid.emu.omega_grid_nom, g); %[output:10e5d3fd]
clear g
%[text] ### References
%[text] u\_cm\_comp\_mod1 is the zero-sequence factor of the space-vector modulator (space\_vector/Gain3): 0 for one module, -1 on the modules that follow a master in a hard parallel connection. The positive-sequence reactive current reference (FRT-VDE4110/positive\_reactive\_current\_reference) is 0, unity power factor. enable\_i\_react\_pos\_steps = 1 gives two steps after the fault test, as in afe\_uvw\_three\_phase: plus and minus the reactive current of cos(phi) = 0.95 at the rated active current, 0.335 s and 0.835 s after the end of the fault (2.0 s and 2.5 s with the default test: extend simlength).
param.afe.ref.u_cm_comp_mod1 = 0;               % [-] zero-sequence factor of the modulator
param.afe.ref.enable_i_react_pos_steps = 0;     % 1: two steps of the reactive current reference
param.afe.ref.i_act_pu = 0.83;                  % [pu] active current at the rated point (section Design figures)
param.afe.ref.cos_phi = 0.95;                   % power factor of the steps
param.afe.ref.time_i_react_pos_ref_1 = 0;       % [s] time of the first step
param.afe.ref.time_i_react_pos_ref_2 = 0;       % [s] time of the second step
param.afe.ref.i_react_pos_ref_1 = 0;            % [pu] before the first step
param.afe.ref.i_react_pos_ref_2 = 0;            % [pu] after the first step
param.afe.ref.i_react_pos_ref_3 = 0;            % [pu] after the second step
if param.afe.ref.enable_i_react_pos_steps
    param.afe.ref.time_i_react_pos_ref_1 = param.frt.data.end_time_LVRT + 0.335;
    param.afe.ref.time_i_react_pos_ref_2 = param.afe.ref.time_i_react_pos_ref_1 + 0.5;
    param.afe.ref.i_react_pos_ref_2 = param.afe.ref.i_act_pu*tan(acos(param.afe.ref.cos_phi));
    param.afe.ref.i_react_pos_ref_3 = -param.afe.ref.i_react_pos_ref_2;
end
%[text] ### Control in uvw: parameters of the library blocks (afe\_uvw\_ctrl)
%[text] The five MATLAB Function blocks of the library package masked\_models/afe\_uvw\_ctrl (grid\_voltage\_sequences, frt\_vde4110, grid\_current\_sequences\_q\_ctrl, current\_ctrl\_uvw and islanding\_detector) read one struct: the mask parameter p of every block is param.afe.uvw. The field names are those of afe\_uvw\_ctrl\_param\_example.m of the library (README of the package); the numbers come from param.afe.ctrl, param.frt.data and param.ctrl. The blocks inherit the sample time of the triggered subsystem afe\_control: uvw.ts must be its period. Same section as in afe\_uvw\_three\_phase, with the islanding frequency thresholds in per unit of the nominal frequency (60 Hz here).
c = param.afe.ctrl;
uvw = struct();
uvw.ts        = param.glb_time.ts_afe;              % [s] control period: integration step of all blocks
uvw.omega_nom = param.grid.emu.omega_grid_nom;      % [rad/s] nominal grid pulsation: base of omega_pu

% grid voltage sequences: xi-eta of the positive and negative sequence, moving average
uvw.vseq.avg_order    = param.ctrl.mavarage_filter_frequency_base_order;  % window = grid period/avg_order (2: half period)
uvw.vseq.omega_pu_min = 0.1;                        % [pu] limits of the measured pulsation used for the window
uvw.vseq.omega_pu_max = 2;
uvw.vseq.t_enable     = param.ctrl.dmavg_filter_enable_time;   % [s] averages are zero before this time

% grid current sequences and reactive power control: PIs on the averaged sequences
uvw.iseq.avg_order    = uvw.vseq.avg_order;
uvw.iseq.omega_pu_min = uvw.vseq.omega_pu_min;
uvw.iseq.omega_pu_max = uvw.vseq.omega_pu_max;
uvw.iseq.t_enable     = uvw.vseq.t_enable;
uvw.iseq.kp_pos       = c.kp_rc_pos_grid;           % PI of the positive-sequence eta grid current
uvw.iseq.ki_pos       = c.ki_rc_pos_grid;           % [1/s]
uvw.iseq.kp_neg       = c.kp_rc_neg_grid;           % PIs of the negative-sequence xi and eta grid currents
uvw.iseq.ki_neg       = c.ki_rc_neg_grid;           % [1/s]
uvw.iseq.lim_pos_eta  = c.ieta_pos_ref_lim;         % [pu] limits of the stack current references
uvw.iseq.lim_neg_xi   = c.ieta_pos_ref_lim;         % [pu] (the old subsystem used the positive limit here too)
uvw.iseq.lim_neg_eta  = c.ieta_neg_ref_lim;         % [pu]
uvw.iseq.err_sign     = -1;                         % error = -(reference - measured): grid current measured towards the converter

% FRT (VDE-AR-N 4110): voltage-rate observers, fault detection, power limitation, reactive current references
uvw.frt.obs_kx        = c.vr_obs.kx;                % gains of the voltage-rate observers of ctrl_afe_setup (poles [-0.1 -0.4]*omega_base)
uvw.frt.obs_kv        = c.vr_obs.kv;
uvw.frt.t_obs_release = 0.025;                      % [s] observers held at zero before this time
uvw.frt.v_pos_low     = 0.85;                       % [pu] fault when the positive sequence is below this ...
uvw.frt.v_pos_high    = 1.15;                       % [pu] ... or above this ...
uvw.frt.v_neg_max     = 0.1;                        % [pu] ... or the negative sequence is above this
uvw.frt.v_hyst        = 0.1;                        % [pu] band narrowing to leave the fault
uvw.frt.v_pos_dot_max = 2.0;                        % [pu/s] the voltage must also change faster than this
uvw.frt.v_neg_dot_max = 2.0;                        % [pu/s]
uvw.frt.t_rearm       = 0.125;                      % [s] idle time needed before a new detection
uvw.frt.t_hold_max    = 7.625;                      % [s] a fault longer than this ends in the state EXCEEDED
uvw.frt.pwrlim_v_pos  = 0.8;                        % [pu] active power limited (pwr_lim_enable) when v_pos < this ...
uvw.frt.pwrlim_v_neg  = 0.1;                        % [pu] ... or v_neg > this
uvw.frt.pwrlim_hyst   = 0.1;                        % [pu] released when v_pos > pwrlim_v_pos + hyst ...
uvw.frt.pwrlim_v_neg_release = 0.7;                 % [pu] ... and v_neg < this
uvw.frt.reddyn_enabled = 0;                         % reduced dynamic grid support (off, as in the old subsystem)
uvw.frt.reddyn_v_pos  = 0.7;                        % [pu]
uvw.frt.reddyn_v_neg  = 0.2;                        % [pu]
uvw.frt.k_frt_ref     = param.frt.data.k_frt_ref;   % [pu/pu] reactive current per voltage change
uvw.frt.t_avg_prefault = 0.5;                       % [s] window of the pre-fault voltage averages
uvw.frt.t_avg_start   = 0.05;                       % [s] the pre-fault averages start here
uvw.frt.lim_pos_eta   = c.i_grid_pos_eta_lim;       % [pu] limits of the grid current references
uvw.frt.lim_neg_xi    = c.i_grid_neg_xi_lim;
uvw.frt.lim_neg_eta   = c.i_grid_neg_eta_lim;
uvw.frt.lim_pos_eta_red = c.i_grid_pos_eta_red_lim; % [pu] the same with reduced dynamic support
uvw.frt.lim_neg_xi_red  = c.i_grid_neg_xi_red_lim;
uvw.frt.lim_neg_eta_red = c.i_grid_neg_eta_red_lim;
uvw.frt.f_lpf_ref     = 161;                        % [Hz] low pass of the references

% current control in uvw: resonant PI per phase, zero-sequence PI
uvw.ictrl.kp        = c.res_pi.kp_rpi;              % resonant PI kp + ki*s/(s^2 + 2*delta*w*s + w^2)
uvw.ictrl.ki        = c.res_pi.ki_rpi;              % [1/s]
uvw.ictrl.delta_res = 0.005;                        % damping of the resonator. The class value 0.015 was tuned
                                                    % with the forward Euler resonator of the old subsystem (poles
                                                    % outside the unit circle, gain 10.8 at omega); the library block
                                                    % uses forward-backward Euler (poles inside) and 0.005 gives
                                                    % about the same gain at omega (11.1): same current loop as before
uvw.ictrl.omega_res = c.omega_base;                 % [rad/s] resonance pulsation
uvw.ictrl.mag_lim   = 1;                            % [pu] the alpha/beta output vector is scaled down above this length
uvw.ictrl.mag_scale_max = 10;                       % largest scaling factor
uvw.ictrl.out_delay = 1;                            % 1: alpha/beta output one step late (as the old subsystem), 0: no delay
uvw.ictrl.kp_z      = c.kp_is_z;                    % zero-sequence current PI
uvw.ictrl.ki_z      = c.ki_is_z;                    % [1/s]
uvw.ictrl.u_z_lim   = 1;                            % [pu] limit of u_gamma
uvw.ictrl.iz_ref    = 0;                            % [pu] zero-sequence current reference

% islanding detector (passive): U<, U>, f<, f>, ROCOF and 5th/7th harmonic criteria with on-delays.
% Thresholds of the network protection of VDE-AR-N 4105 / 4110 (frequency in pu of the nominal); the slow U< stage
% (2.7 s) lets the converter ride through a grid fault (frt_vde4110) without an islanding trip.
uvw.island.h_avg_order  = 1;                        % harmonic averaging window = grid period/h_avg_order
uvw.island.omega_pu_min = 0.5;                      % [pu] limits of the measured pulsation used for the window
uvw.island.omega_pu_max = 1.5;
uvw.island.t_enable     = 0.1;                      % [s] no detection before this time
uvw.island.tau_f        = 0.02;                     % [s] low pass of the frequency
uvw.island.obs_poles    = [-20 -80];                % [rad/s] poles of the ROCOF observer (double integrator, as vr_obs)
Ad = [1 uvw.ts; 0 1];
L = acker(Ad', [1 0]', exp(uvw.island.obs_poles*uvw.ts))';
uvw.island.obs_kx = L(1)/uvw.ts;
uvw.island.obs_kv = L(2)/uvw.ts;
uvw.island.u_valid      = 0.5;                      % [pu] frequency and harmonic criteria only above this voltage
uvw.island.u_min        = 0.8;                      % [pu] under-voltage ...
uvw.island.t_trip_u_min = 2.7;                      % [s] ... for this time
uvw.island.u_max        = 1.15;                     % [pu] over-voltage ...
uvw.island.t_trip_u_max = 0.1;                      % [s] ... for this time
uvw.island.f_min        = 0.95;                     % [pu] under-frequency (57.0 Hz at 60 Hz, 47.5 Hz at 50 Hz) ...
uvw.island.f_max        = 1.03;                     % [pu] over-frequency (61.8 Hz at 60 Hz, 51.5 Hz at 50 Hz) ...
uvw.island.t_trip_f     = 0.1;                      % [s] ... for this time
uvw.island.rocof_max    = 2.5;                      % [Hz/s] rate of change of frequency ...
uvw.island.t_trip_rocof = 0.1;                      % [s] ... for this time
uvw.island.h_max        = 0.1;                      % [pu/pu] sqrt(u5^2 + u7^2)/u1 above this ...
uvw.island.t_trip_h     = 0.1;                      % [s] ... for this time
uvw.island.latch        = 1;                        % the output stays 1 until enable goes to 0

param.afe.uvw = uvw;
clear c uvw Ad L
%%
%[text] ## Phase-shift full bridge (full\_bridge\_converter)
%[text] Data set PSFBC\_800V of the library with the numbers of the legacy init: input capacitor 1.2 mF, centre-tapped transformer 34:2:2 (Lm 1 mH, leakage Ls = udc\*udc/(8\*fpwm\*P\*2.7) = 20 uH, half on the primary and half on the secondaries), output inductor 0.4 uH, output capacitor 45 mF. The bases of the control are idc2\_bez = 12 kA (output current) and idc1\_bez = P/udc = 433 A (input current).
param.psfbc.pwr_nom = 325e3;                    % [W]
h = struct();
h.udc2 = 45;                    % [V] output voltage base
h.idc2 = 12e3;                  % [A] output current base and reference of the current loop
h.n1 = 34;                      % turns of the primary
h.n2 = 2;                       % turns of each half of the centre-tapped secondary
h.Lm = 1e-3;                    % [H] magnetizing inductance (primary)
h.Rfe = 1e3;                    % [Ohm] iron losses (primary)
h.Rs1 = 1e-3;                   % [Ohm] primary winding (each secondary half: Rs1*(n2/n1)^2)
h.core_length = 1;              % [m] only for the flux density output of the transformer block
h.core_mur = 70;                % [-]
h.ls_factor = 2.7;              % leakage design factor: Ls = udc^2/(8*fpwm*pwr_nom*ls_factor)
h.Cdc_dc1 = 1.2e-3;             % [F] input capacitor CFi_dc1
h.RCdc_dc1 = 1e-3;              % [Ohm]
h.Cdc_dc2 = 45e-3;              % [F] output capacitor CFu_dc2
h.RCdc_dc2 = 1e-4;              % [Ohm]
h.Ldc_dc1 = 250e-6;             % [H] input inductor of the data set (not in the model)
h.RLdc_dc1 = 157*0.02*h.Ldc_dc1;
h.Ldc_dc2 = 0.4e-6;             % [H] output inductor
h.RLdc_dc2 = 157*0.01*h.Ldc_dc2;
param.psfbc.hw_in = h;
param.psfbc.hw = prv.psfbc_hw_param(param.afe.hw.udc_nom, param.psfbc.pwr_nom, param.glb_time.fPWM_PSFBC, h); %[output:01855f71]
clear h
%[text] ### Control
%[text] Output current loop: a PI on the load current (in pu of 12 kA) gives the phase displacement of leg B (0 to 1 = 0 to pi); the reference steps to 12 kA at 25 ms with a slope of 10 times 12 kA per second. The DC transformer current compensation adds its PI output (limited at plus or minus lim\_idc\_trafo) to the duty 0.5 of both legs. It reads the DC input current of the bridge, which is positive at full load: it sits at its lower limit and caps the effective duty at 1 - 2\*lim\_idc\_trafo = 0.8, which the section "Design figures" compares with the duty the rated point needs. lim\_idc\_trafo = 0 switches it off. Values of the legacy model; they were typed in the blocks before.
param.psfbc.ctrl.kp_idc = 1;                    % [pu/pu] output current PI (legacy psfbc_ctrl.kp_idc)
param.psfbc.ctrl.ki_idc = 18;                   % [1/s]
param.psfbc.ctrl.kp_udc = 0.5;                  % [pu/pu] output voltage PI of the data set (not used by the model)
param.psfbc.ctrl.ki_udc = 18;                   % [1/s] (not used by the model)
param.psfbc.ctrl.kp_idc_trafo = 0.1;            % [pu/pu] DC transformer current compensation (typed in the model before)
param.psfbc.ctrl.ki_idc_trafo = 2;              % [1/s] (block ts = ts_dab)
param.psfbc.ctrl.lim_idc_trafo = 0.1;           % [-] limit of the duty correction; 0 switches the compensation off
param.psfbc.ctrl.tau_idc_trafo = 20e-3;         % [s] low-pass filter of the measured current (block ts = ts_dab)
%%
%[text] ## Rectifier diodes
%[text] ABB 5SDF 0131Z0401 (Simscape diode with charge model), blocks diode\_rectifier\_1 and diode\_rectifier\_2 (diode\_rectifier\_3 to 5 are commented out in the model).
d = struct();
d.Vf = 0.977;               % [V] forward voltage
d.Rdon = 22e-6;             % [Ohm] on-state resistance
d.Cj = 0;                   % [F] junction capacitance
d.Irm = -75;                % [A] peak reverse recovery current
d.didt = -30;               % [A/us] at the recovery
d.trr = 5;                  % [us] reverse recovery time
d.Qrr = 325e-6;             % [C] recovered charge
d.Ifm = 2000;               % [A] forward current of the recovery data
d.Vr = -50;                 % [V] reverse voltage of the recovery data
d.Err = 15e-3;              % [J] recovery energy
d.Rth_JC = 0.004;           % [K/W] junction to case
d.Rth_CH = 0.003;           % [K/W] case to heatsink
d.Rsnubber = 1e4;           % [Ohm]
d.Csnubber = 1e-12;         % [F]
param.diode.rectifier = d;
clear d
%%
%[text] ## Power semiconductors
%[text] Ideal switch for the AFE bridge and the brake (param.afe.ideal\_switch) and for the full bridge (param.psfbc.ideal\_switch); IGBT and MOSFET data sets for the full bridges with thermal model and for the IGBT AFE bridge (commented out in the model).
param.afe.ideal_switch = prv.device_ideal_switch_param('silicon_high_power_ideal_switch', param.glb_time.fPWM_AFE, param.afe.hw.udc_nom); %[output:7a95a2c1]
param.psfbc.ideal_switch = prv.device_ideal_switch_param('silicon_high_power_ideal_switch', param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom); %[output:0592dc0d]
param.afe.igbt = prv.device_igbt_param('mitsubishi_CM1200DW_34T', param.glb_time.fPWM_AFE, param.afe.hw.udc_nom); %[output:2e283582]
param.psfbc.igbt = prv.device_igbt_param('mitsubishi_CM1200DW_34T', param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom); %[output:7ebc0036]
param.psfbc.mosfet = prv.device_mosfet_param('danfoss_SKM1700MB20R4S2I4', param.glb_time.fPWM_PSFBC, param.psfbc.hw.udc1_nom); %[output:66698fb6]
%%
%[text] ## Heatsink
%[text] Aluminium plate, liquid cooled, for PrimePACK2 modules: water flow above 28 l/min, water in/out rise at most 5 K with 2 kW of losses. HA is the rise from the water to the heatsink surface. no\_weight = weight/10 leaves out the thermal inertia.
param.thermal.weight = 0.150;                                   % [kg]
param.thermal.no_weight = 0.150/10;                             % [kg]
param.thermal.cp_al = 900;                                      % [J/(kg K)] aluminium
param.thermal.heat_capacity_hs = param.thermal.cp_al*param.thermal.weight;   % [J/K]
param.thermal.thermal_conductivity_al = 160;                    % [W/(m K)]
param.thermal.Rth_switch_HA = 15/1000;                          % [K/W]
param.thermal.Rth_mosfet_HA = param.thermal.Rth_switch_HA;      % [K/W]
param.thermal.Rth_diode_HA = param.thermal.Rth_switch_HA;       % [K/W]
param.thermal.Tambient = 40;                                    % [degC] water temperature
param.thermal.DThs_init = 0;                                    % [K] initial heatsink rise
% heatsink object of the library class liquid_cooled_plate_2kw_setup, read by the masks of the bridges with
% thermal model (fields cp, no_weight, Rth_switch_HA, Rth_mosfet_HA, Rth_diode_HA, Tambient, DThs_init)
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
%%
%[text] ## Battery (battery\_1, commented out in the model)
%[text] Lithium-ion pack at the DC-link voltage, 250 kW, 1C capacity, as the legacy init (R0 and R1 halved, C1 divided by 50). The block is commented out: the data only keep it ready.
param.battery.pack_1 = prv.battery_param(param.afe.hw.udc_nom, 250e3, 250e3/param.afe.hw.udc_nom, 0.85, param.glb_time.ts_dab, false); %[output:0fdf3891]
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/50;
%%
%[text] ## Load
param.load.u_out = 25;                          % [V] rated output voltage
param.load.i_out = 12e3;                        % [A] rated output current
param.load.R = param.load.u_out/param.load.i_out;   % [Ohm] block Resistor at the output: 2.083 mOhm
%%
%[text] ## Measurements
param.meas.adc_quantization = 1/2^11;           % [pu] 12-bit converter on +-1 pu
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;
%%
%[text] ## Design figures
%[text] Averaged models (the README explains them): operating point and duty budget of the full bridge, its output current loop, and the AFE current and DC-link voltage loops. With the legacy settings two points stand out: the DC transformer current compensation caps the duty of the full bridge at 0.8 while the rated point needs about 0.83 (the output stays near 11.6 kA, 24.2 V), and kp\_idc = 1 is close to the stability limit of the output current loop (gain margin 1.6 dB, the output filter mode at 1.35 kHz damped 6 percent). Both are left as in the legacy model; the README gives the numbers of the alternatives. The AFE current loop is the resonant PI per phase of current\_ctrl\_uvw (kp 0.6, ki 35): crossover 193 Hz, phase margin 75 deg, gain margin 17 dB; the DC-link loop crosses at 56 Hz with 71 deg. Without the grid impedance the LCL resonance is at 2.06 kHz.
if param.sim.design_figures %[output:group:70bca072]
    param.design = prv.design_figures(param); %[output:02ed2fb8]
end %[output:group:70bca072]
%%
%[text] ## Model
%[text] ### C-Caller types and scopes
open_system(model);
Simulink.importExternalCTypes(model, 'Names', {'sv_pwm_cm_output_t'});    % space vector modulator (the only C-Caller)
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### Full-bridge variant
%[text] The three full-bridge blocks are switched by commenting them in and out (as the legacy init). The blocks that log the data of a bridge (From blocks of its Goto tags, their To Workspace blocks and scopes) follow it: when the bridge is commented out, their From blocks would have no Goto.
fb = [model '/full_bridge_converter/full_bridge_converter_with_iso/full_bridge/'];
bridges = {'full-bridge_ideal_switch_based', 'full_bridge_igbt_adv_thermal_model', ...
    'full-bridge_mosfet_based_ with_thermal_model_zvs'};
readers = { ...
    {'afe_measurement_sim/From36', 'afe_measurement_sim/To Workspace41', 'scope/From25', 'scope/full_bridge_ideal_switch_dat'}, ...
    {'afe_measurement_sim/From15', 'afe_measurement_sim/From22', 'afe_measurement_sim/From23', 'afe_measurement_sim/From24', ...
     'afe_measurement_sim/From25', 'afe_measurement_sim/From26', 'afe_measurement_sim/From27', 'afe_measurement_sim/From28', ...
     'afe_measurement_sim/From30', 'afe_measurement_sim/To Workspace27', 'afe_measurement_sim/To Workspace28', ...
     'afe_measurement_sim/To Workspace29', 'afe_measurement_sim/To Workspace30', 'afe_measurement_sim/To Workspace31', ...
     'afe_measurement_sim/To Workspace32', 'afe_measurement_sim/To Workspace33', 'afe_measurement_sim/To Workspace34', ...
     'afe_measurement_sim/To Workspace36', 'scope/From6', 'scope/From7', 'scope/From8', 'scope/From9', 'scope/From18', ...
     'scope/From19', 'scope/From22', 'scope/From23', 'scope/bridge_device_power_losses_Q1', 'scope/bridge_device_power_losses_Q2'}, ...
    {'afe_measurement_sim/From29', 'afe_measurement_sim/To Workspace35'}};
if param.sim.use_mosfet_thermal_model
    active = 3;
elseif param.sim.use_thermal_model
    active = 2;
else
    active = 1;
end
for b = 1:numel(bridges)
    state = 'on';
    if b == active
        state = 'off';
    end
    set_param([fb bridges{b}], 'Commented', state);
    for r = 1:numel(readers{b})
        set_param([model '/' readers{b}{r}], 'Commented', state);
    end
end
fprintf('%s init: full bridge %s active\n', model, bridges{active}); %[output:87aefeef]
clear fb bridges readers active b r state
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
%[text] ### Checks of the model
%[text] Every param.\* read by the active blocks must exist; then the From blocks of the active part that have no active Goto. In this model they all feed scopes or To Workspace blocks: the logging subsystem data\_to\_worspace\_module\_1 of the control copied from afe\_rst\_ctrl (old PLLs and xi-eta current control), instantaneus\_power (commented out) and power\_meter\_iec (the new power meter does not write it). afe\_rst\_ctrl has the same ones in data\_to\_worspace\_module\_1; the README lists them.
if param.sim.check_model %[output:group:95f59ab5]
    [missing_refs, model_refs] = prv.check_model_param_refs(model, param); %[output:88ff59d7] %[output:2c60366b] %[output:43c50375]
    if isempty(missing_refs)
        fprintf('%s init: all %d param.* references of the active blocks resolve\n', model, numel(model_refs)); %[output:118beca0]
    else
        warning('%s init: %d param.* references of the active blocks do not exist in param:\n  %s', model, ...
            numel(missing_refs), strjoin(missing_refs', '\n  '));
    end
    lost = prv.check_from_goto(model);
    if isempty(lost)
        fprintf('%s init: every From block of the active part has its Goto\n', model);
    else
        warning('%s init: %d tags of From blocks have no active Goto:\n  %s', model, numel(lost), strjoin(lost, '\n  ')); %[output:01640732]
    end
    clear missing_refs model_refs lost
end %[output:group:95f59ab5]

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":33.8}
%---
%[output:675b18a7]
%   data: {"dataType":"text","outputData":{"text":"library_setup: library already on the path (C:\\github\\pwr-control\\library)\n","truncated":false}}
%---
%[output:929bb977]
%   data: {"dataType":"text","outputData":{"text":"ps_25V_12kA_r init: functions of .\/private, called as prv.<name>: afe_ctrl_param, afe_hw_param, battery_param, check_from_goto, check_model_param_refs, design_figures, device_ideal_switch_param, device_igbt_param, device_mosfet_param, grid_emulator_param, grid_fault_param, library_setup, psfbc_hw_param, timing_param\n","truncated":false}}
%---
%[output:5f132968]
%   data: {"dataType":"text","outputData":{"text":"Timing: AFE 4000 Hz, double update = 1, ts_afe = 125.0 us, dead time 3.0 us | full bridge 4000 Hz, double update = 1, ts_psfbc = 125.0 us, dead time 3.0 us\ntc = 0.6250 us, logging window 1.750 s (Nc = 2800000, Ns_afe = 14000), ts_dab = 41.67 us\n","truncated":false}}
%---
%[output:8072d662]
%   data: {"dataType":"text","outputData":{"text":"Transformer Dyn11_480V_480V_1750kVA: Ld1 = 26.2 uH, Rd1 = 9.87 mOhm, Ld2 = 8.7 uH, Rd2 = 3.29 mOhm, Lm1 = 144.8 mH (i1m = 15.2 A), Rfe1 = 494 Ohm\nPer unit (coil bases 0.3950 \/ 0.1317 Ohm): r1 = 0.0250, l1 = 0.0250, r3 = 0.0250, l3 = 0.0250, lm = 138.2, lm0 = 0.5, rfe = 1250, clock 11\nGrid emulator: grid_480V_1750kVA_Dyn11, 60 Hz, ugrid_bez = 391.9 V, igrid_bez = 509.1 A | no grid impedance: grid side = transformer leakage 17.5 uH (star equivalent)\n---------------------------\n","truncated":false}}
%---
%[output:0a0efb48]
%   data: {"dataType":"text","outputData":{"text":"Fault test: two_phase_D, enable_frt_1 = 0, enable_frt_2 = 0, amplitudes [0.250 0.875 0.875], phase shift -0.380 rad, from 0.750 s to 1.665 s\n","truncated":false}}
%---
%[output:32b8bcfd]
%   data: {"dataType":"text","outputData":{"text":"Hardware three-phase two-level AFE: afe480V, 325 kW, 480 V, 60 Hz | ubez = 391.9 V, ibez = 614.3 A (is_nom 434.3 A rms), 1 pu = 361.1 kW | udc_nom = 750 V\nDC link CFi = 2.4 mF | filter LFu = 330 uH (0.195 pu), CFu = 360 uF star (0.087 pu), RCFu = 50 mOhm | Rbrake = 2.5 Ohm\n---------------------------\n","truncated":false}}
%---
%[output:10e5d3fd]
%   data: {"dataType":"text","outputData":{"text":"AFE control: udc PI kp = 2.00, ki = 35, ixi limit = 1.65 | resonant current PI kp = 0.60, ki = 35, zero sequence kp = 0.50, ki = 18 | sequence PIs pos 0.25\/35, neg 0.25\/35, limits 1.00\/0.50 | PLL kp = 1.0, ki = 80\nFHT: omega = 377.0 rad\/s, delta = 0.05, Ld = [0.2155 41.4364] | di_obs: Ld = [0.2178 63.263187] | vr_obs: kx = 186.99, kv = 5618.40\n","truncated":false}}
%---
%[output:01855f71]
%   data: {"dataType":"text","outputData":{"text":"Phase-shift full bridge: PSFBC_800V, 325 kW, 750 V -> 45 V \/ 12000 A bases, 4000 Hz | transformer 34:2:2, Lm = 1.00 mH, Ls = 20.03 uH (Ld1 = 10.02 uH, Ld2 = 34.7 nH), Rs1 = 1.00 mOhm\nCin = 1.20 mF + 1.0 mOhm, Cout = 45 mF + 0.10 mOhm, Lout = 0.40 uH + 0.63 uOhm | idc1_bez = 433.3 A\n---------------------------\n","truncated":false}}
%---
%[output:7a95a2c1]
%   data: {"dataType":"text","outputData":{"text":"Ideal switch data set: silicon_high_power_ideal_switch | Rds_on = 1.50 mOhm, Vth = 5.5 V | diode 2.0 V + 0.15 mOhm | snubber 2200 Ohm + 10 nF\n","truncated":false}}
%---
%[output:0592dc0d]
%   data: {"dataType":"text","outputData":{"text":"Ideal switch data set: silicon_high_power_ideal_switch | Rds_on = 1.50 mOhm, Vth = 5.5 V | diode 2.0 V + 0.15 mOhm | snubber 2200 Ohm + 10 nF\n","truncated":false}}
%---
%[output:2e283582]
%   data: {"dataType":"text","outputData":{"text":"IGBT data set: mitsubishi_CM1200DW-34T (Si-IGBT)\nVce_sat = 2.40 V + 0.25 mOhm, Vd = 2.80 V + 0.25 mOhm | Eon+Eoff = 730 mJ, Erec = 140 mJ at 1000 V \/ 1200 A | Rth_switch_JH = 49.0 mK\/W, Rth_diode_JH = 64.0 mK\/W\n---------------------------\n","truncated":false}}
%---
%[output:7ebc0036]
%   data: {"dataType":"text","outputData":{"text":"IGBT data set: mitsubishi_CM1200DW-34T (Si-IGBT)\nVce_sat = 2.40 V + 0.25 mOhm, Vd = 2.80 V + 0.25 mOhm | Eon+Eoff = 730 mJ, Erec = 140 mJ at 1000 V \/ 1200 A | Rth_switch_JH = 49.0 mK\/W, Rth_diode_JH = 64.0 mK\/W\n---------------------------\n","truncated":false}}
%---
%[output:66698fb6]
%   data: {"dataType":"text","outputData":{"text":"MOSFET data set: danfoss_SKM1700MB20R4S2I4 (SiC MOSFET)\nRds_on = 1.04 mOhm, Vth = 5.5 V | Eon+Eoff = 185 mJ, Err = 9.7 mJ at 1300 V \/ 1000 A | Rth_mosfet_JH = 35.0 mK\/W\n---------------------------\n","truncated":false}}
%---
%[output:0fdf3891]
%   data: {"dataType":"text","outputData":{"text":"Battery: 750 V | 250 kW | 333 Ah | 207 cells | R0 = 56.25 mOhm | soc_init = 0.85\n","truncated":false}}
%---
%[output:02ed2fb8]
%   data: {"dataType":"text","outputData":{"text":"1. Full bridge at 25.0 V, 12000 A (300 kW): n*udc = 44.12 V (udc 750 V, turns 34:2), commutation dD = 0.226 (0.832 mOhm), path 39.9 uOhm + diode 0.98 V (ideal switch 1.50 mOhm)\n   duty needed 0.826, largest duty 0.800: the DC transformer current compensation sits at -0.10 (d1 = 0.40), the current loop saturates at about 11613 A, 24.19 V. lim_idc_trafo = 0 gives D up to 1 (14599 A possible).\n   power from the DC link about 298 kW (conduction 16.7 kW, iron 0.5 kW)\n2. Full-bridge output current loop: kp_idc = 1.00, ki_idc = 18 1\/s, ts = 125.0 us, plant 1.244 pu\/pu at DC, output filter 1186 Hz (Z0 2.98 mOhm, load 2.08 mOhm)\n   crossover 1105 Hz, phase margin 34.3 deg, gain margin 1.6 dB | closed loop: pair at 1349 Hz damped 0.057, slow pole -10.0 rad\/s | stable for kp_idc < 1.21\n3. AFE: 298 kW from the grid = 0.83 pu of active current (507 A peak, limit 1.65 pu) | LCL: LFu 330 uH, CFu 360 uF, grid side 17.5 uH -> 2060 Hz (fs\/6 = 1333 Hz)\n   current loop, resonant PI per phase kp = 0.60, ki = 35, delta = 0.005: crossover 193 Hz, phase margin 75.2 deg, gain margin 17.2 dB\n   DC-link loop kp = 2.00, ki = 35, C = 3.6 mF (plant 178 1\/s, current loop as a dq PI 0.60\/17.5): crossover 56 Hz, phase margin 70.8 deg, gain margin 22.7 dB\n","truncated":false}}
%---
%[output:87aefeef]
%   data: {"dataType":"text","outputData":{"text":"ps_25V_12kA_r init: full bridge full-bridge_ideal_switch_based active\n","truncated":false}}
%---
%[output:88ff59d7]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Error resolving Custom Code."}}
%---
%[output:2c60366b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: include specified in custom include directory paths string does not exist in any of the following search directories:\nC:\\github\\pwr-control\\modelization-and-control\\foundations_of_power_electronics\\ps_25V_12kA"}}
%---
%[output:43c50375]
%   data: {"dataType":"warning","outputData":{"text":"Warning: src specified in custom include directory paths string does not exist in any of the following search directories:\nC:\\github\\pwr-control\\modelization-and-control\\foundations_of_power_electronics\\ps_25V_12kA"}}
%---
%[output:118beca0]
%   data: {"dataType":"text","outputData":{"text":"ps_25V_12kA_r init: all 150 param.* references of the active blocks resolve\n","truncated":false}}
%---
%[output:01640732]
%   data: {"dataType":"warning","outputData":{"text":"Warning: ps_25V_12kA_r init: 24 tags of From blocks have no active Goto:\n  i_grid_pu_rms (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/pwr_meter)\n  id_eta_pu (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/inner_current_tracking)\n  id_xi_pu (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/inner_current_tracking)\n  isa (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/inner_current_tracking)\n  isa_ref (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/inner_current_tracking)\n  isb (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/inner_current_tracking)\n  isb_ref (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/inner_current_tracking)\n  isc (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/inner_current_tracking)\n  isc_ref (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/inner_current_tracking)\n  iz_ctrl_out (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/inner_current_tracking)\n  iz_pu (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/inner_current_tracking)\n  iz_pu_ref (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/inner_current_tracking)\n  omega_grid_dqpll_ccaller (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/dq_pll)\n  omega_grid_fht_simulink_pll (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/dq_pll)\n  omega_grid_fhtsr_dqpll (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/dq_pll)\n  omega_grid_pu_dqpll (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/dq_pll)\n  phase_grid_dqpll (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/dq_pll)\n  phase_grid_dqpll_ccaller (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/dq_pll)\n  phase_grid_fht_simulink_pll (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/dq_pll)\n  phase_grid_fhtsr_dqpll (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/dq_pll)\n  power_meter_iec (2 From in afe_measurement_sim, scope)\n  pwr_act_grid (2 From in afe_abc_mod1\/AFE_control\/afe_control, afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/pwr_meter)\n  pwr_react_grid (2 From in afe_abc_mod1\/AFE_control\/afe_control, afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/pwr_meter)\n  u_grid_pu_rms (1 From in afe_abc_mod1\/AFE_control\/afe_control\/data_to_worspace_module_1\/pwr_meter)"}}
%---
