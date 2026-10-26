%[text] # Three-leg interleaved boost converter
%[text] Initialization of interleaved_boost.slx (discrete blocks) and of interleaved_boost_cmp.slx (the compact block
%[text] interleaved_boost_3legs_mod1 of the library with the same battery, timing and load; build_interleaved_boost_compact.m
%[text] generates the minimal interleaved_boost_compact.slx around the same block): 200 V battery on the low-voltage side,
%[text] three boost legs (the half bridges of the interleaved buck used in the other direction: one inductor per leg from the
%[text] battery to the leg midpoint, the lower switch is the boost switch, the upper one the synchronous rectifier) with
%[text] interleaved carriers (0, 120, 240 deg), DC-link capacitor regulated at 660 V, resistive load on the DC link. DC-link
%[text] voltage loop giving the total input current reference, one PI per leg on the leg current (reference i_ref/3: inherent
%[text] current sharing), each leg current sampled at the extremes of its own carrier (average value, no ripple), control
%[text] step at the extremes of carrier 1 (double update, 6 kHz at fpwm = 3 kHz).
%[text] The hardware (inductors, capacitors, module, DC link) is the one of interleaved_buck_converter on purpose: same
%[text] converter, power flow reversed. What changes is the control: regulated voltage, sign of the currents, duty of the
%[text] lower switch, right-half-plane zero of the boost in the voltage loop.
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, boost_ctrl_param, device_igbt_param, battery_param, check_model_param_refs). The library is
%[text] needed for the Simulink/Simscape blocks (modulator, bridge, DC link, battery), for the compact block and for the
%[text] IGBT data set (setup_files/semiconductor_devices_data, read by device_igbt_param through device_data_load).
%[text] Run it from this folder (the functions of ./private are visible only to the files of this folder).
%[text] param.boost is also the mask struct p of the compact block (p.hw, p.pwm, p.ctrl): numeric fields only.

clear;
model = 'interleaved_boost';

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
% param.sim.tc = 1e-7;               
param.sim.tc = 1/(2*3e3)/200;               % [s] 0.833 us: logging base, model MaxStep (ode23tb) and step of the Simscape
%     local solver (glb_time.tc): ts/200 at fpwm = 3 kHz with double update
param.sim.use_thermal_model = 1;            % IGBT bridge with the simple two-thermal-port loss model
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration
if param.sim.use_thermal_model
    param.sim.nonlinear_iterations = 5;
end
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
param.time.fpwm = 3e3;                              % [Hz] switching frequency of each leg (the DC link sees n_legs*fpwm = 9 kHz)
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
%[text] ## Boost converter
%[text] ### Operating point
% 200 V battery to a 660 V DC link, 750 A of input current (150 kW): 250 A per leg, duty of the lower switch
% D = 1 - uin/udc = 0.697 (the upper switch conducts for M = 1 - D = 0.303, the duty of the buck at the same
% voltages). Bases of the pu signals: uin_nom (battery side), udc_nom (DC link, the regulated voltage),
% ibez = total nominal input current (a leg carries 1/3 pu; the DC-link current is 0.303 pu at nominal load).
param.boost.hw.n_legs = 3;
param.boost.hw.uin_nom = 200;                       % [V] battery (low-voltage side, input)
param.boost.hw.udc_nom = 660;                       % [V] DC link (regulated)
param.boost.hw.i_in_nom = 750;                      % [A] input current (total of the three inductors)
param.boost.hw.pwr_nom = param.boost.hw.uin_nom*param.boost.hw.i_in_nom;   % [W] 150 kW
param.boost.hw.ibez = param.boost.hw.i_in_nom;      % [A] 750 A, base of all currents (total)
%[text] ### Sizing of the leg inductors and of the capacitors
% The hardware is the one of the interleaved buck (same L, capacitors, DC link, module): the minima below are the
% boost figures of the same specifications, the chosen values follow (identical to the buck by design).
spec.r_i_leg = 0.25;        %      max peak-peak ripple of the leg current / nominal leg current, at the nominal duty
spec.r_u_in = 0.2e-2;       %      max peak-peak ripple of the input (battery side) voltage, capacitor alone
spec.r_u_dc = 0.2e-2;       %      max peak-peak ripple of the DC-link voltage, capacitor alone
spec.f_c_i = 225;           % [Hz] crossover target of the per-leg current loops: fs/26.7 at fs = 6 kHz (as the buck: the
                            %      delays 1.5 ts on leg 1 and 2.5 ts on legs 2-3 leave about 58 and 45 deg of phase margin)
spec.f_c_u = 40;            % [Hz] crossover target of the DC-link voltage loop at nominal load: about f_z/4, where f_z is the
                            %      right-half-plane zero of the boost (157 Hz at nominal load, see below); at 60 Hz the gain
                            %      margin on the zero drops to 5.8 dB (2026-10-26 check), at 40 Hz it is 8 dB with 83 deg
spec.k_fz = 4;              %      f_c_u <= f_z/k_fz
hw = param.boost.hw;
n  = hw.n_legs;
M  = hw.uin_nom/hw.udc_nom;                                                 % duty of the upper switch, 0.303
D  = 1 - M;                                                                 % duty of the lower (boost) switch, 0.697
i_leg = hw.i_in_nom/n;                                                      % [A] 250 A
% leg inductor: the inductor sees uin for the fraction D of the period, dI_leg = uin*D/(L*fpwm) = udc*M*(1-M)/(L*fpwm),
% the same ripple as the buck at the same voltages
L_min = hw.uin_nom*D/(param.time.fpwm*spec.r_i_leg*i_leg);                  % [H] 0.743 mH for 25 % of 250 A
L_choice = 0.8e-3;                                                          % [H] the inductor of the buck
RL_choice = 6e-3;                                                           % [Ohm]
% input capacitor (battery side): the three leg ripples add with the 120 deg shift to a triangular current at n*fpwm,
% dI_tot(M) = n*udc*(M - k/n)*((k+1)/n - M)/(L*fpwm), k = floor(n*M) (nearly zero at M = 1/3, i.e. at the nominal
% duty), worst case over the duty dI_tot_max = udc/(4*n*L*fpwm); the capacitor takes it: C = dI_tot_max/(8*n*fpwm*dU).
% The input capacitor has no control role here: the battery (R0 of a few mOhm) sits across it, so the inductors
% see the battery voltage at the control frequencies and the L/n - Cin resonance (112 Hz with the 7.5 mF of the
% buck) is damped by the battery. The 7.5 mF of the buck (where the same capacitor is the regulated output and
% its LC resonance had to stay inside the current loops) is kept as the same hardware.
dI_tot_max = hw.udc_nom/(4*n*L_choice*param.time.fpwm);                     % [A] 22.9 A peak-peak at 9 kHz
Cin_ripple = dI_tot_max/(8*n*param.time.fpwm*spec.r_u_in*hw.uin_nom);      % [F] 0.80 mF for 0.4 V
% DC-link capacitor: the upper diodes deliver a pulse train at n*fpwm (the conduction intervals of the three legs
% do not overlap at M < 1/3), duty D' = n*M - k, the capacitor takes i_leg*sqrt(D'*(1-D')) rms and the ripple is
% dU_dc = D'*(1-D')*i_leg/(n*fpwm*Cdc)
k = floor(n*M);
Dp = n*M - k;                                                               % duty of the DC-side pulse train, 0.909
Cdc_ripple = Dp*(1 - Dp)*i_leg/(n*param.time.fpwm*spec.r_u_dc*hw.udc_nom); % [F] 1.7 mF for 1.3 V
Cdc_choice = 7.2e-3;                                                        % [F] the DC link of the buck: 0.3 V of ripple
% right-half-plane zero of the boost: with the current loops closed the DC-link current is (1-d)*i_leg, and a
% current increase is obtained by first increasing d, so the plant from the input current to udc has a zero at
% w_z = (uin - 2*RL*i_leg)/(L*i_leg) (per leg; the three legs in parallel give the same value) in the right half
% plane: the voltage loop must cross below f_z/k_fz, and its gain above f_z does not roll off (gain margin)
f_z = (hw.uin_nom - 2*RL_choice*i_leg)/(2*pi*L_choice*i_leg);               % [Hz] 157 Hz at nominal load, higher at light load
f_c_u_max = f_z/spec.k_fz;                                                  % [Hz] 39 Hz
fprintf('Sizing: D = %.3f (M = %.3f), %.0f A per leg; L_min = %.3f mH for %.0f %% of ripple; Cin_ripple = %.2f mF for %.1f %% of input ripple (worst duty); Cdc_ripple = %.2f mF for %.1f %% of DC-link ripple; RHP zero %.0f Hz -> f_c_u <= %.0f Hz\n', ...
    D, M, i_leg, L_min*1e3, spec.r_i_leg*100, Cin_ripple*1e3, spec.r_u_in*100, Cdc_ripple*1e3, spec.r_u_dc*100, f_z, f_c_u_max);
if spec.f_c_u > f_c_u_max
    warning('%s init: the voltage loop crossover %.0f Hz is above f_z/%.0f = %.0f Hz', model, spec.f_c_u, spec.k_fz, f_c_u_max);
end
%[text] ### Hardware
% leg inductors: one separate inductor per leg (a three-limb coupled reactor is wrong here: the three leg
% currents are DC and in phase, the core sees only the zero-sequence inductance). 0.8 mH: 58 A peak-peak of
% ripple per leg at D = 0.697 (23 % of the leg current), the three ripples add to 8 A peak-peak at 9 kHz on the
% input capacitor at the nominal duty (22.9 A worst case over the duty)
param.boost.hw.L = L_choice;                        % [H] per leg
param.boost.hw.RL = RL_choice;                      % [Ohm] per leg: 375 W per leg at 250 A, 0.75 % of Pnom (assumption)
param.boost.hw.kL = [1 1 1];                        % per-leg factors of L and RL (mismatch studies: the per-leg loops
param.boost.hw.kRL = [1 1 1];                       % keep the sharing, a common duty would not)
% input capacitor (battery side): 7.5 mF in three parts of 2.5 mF (the model has one capacitor Cin/3 with 3*RCin on
% each leg terminal, the compact block one capacitor Cin with RCin); 15 mV of ripple at 9 kHz at the nominal duty,
% 42 mV worst case (0.02 %)
param.boost.hw.Cin = 7.5e-3;                        % [F] total
param.boost.hw.RCin = 3e-3/3;                       % [Ohm] ESR of the total capacitor (3 mOhm per part)
param.boost.hw.Vin0 = param.boost.hw.uin_nom;       % [V] initial voltage of the input capacitor (battery connected)
% DC-link capacitor (block dclink_for_two_levels in the discrete model, inside the ssc block in the compact one): the
% regulated capacitor; 75 A rms, 0.34 V of ripple at 9 kHz; the load corner of the voltage plant is
% 2/(2*pi*R_load*Cdc) = 15 Hz (the 2: at constant input current the DC-link current falls as 1/udc)
param.boost.hw.Cdc = Cdc_choice;                    % [F]
param.boost.hw.Rdc = 1e-3;                          % [Ohm] ESR (compact block)
param.boost.hw.Vdc0 = param.boost.hw.uin_nom;       % [V] initial DC-link voltage (compact block): precharged through the upper
                                                    %     diodes to the battery voltage. The dclink block of the discrete model
                                                    %     has no initial voltage: there the DC link starts at 0 V and is charged
                                                    %     by the current loops at i_lim in the first milliseconds
% switches of the compact block (numerical switch + antiparallel diode, no losses): the discrete model has the
% IGBT bridge with the loss and thermal model (param.boost.igbt, param.thermal)
param.boost.hw.Vth = 2.5;                           % [V]   gate threshold
param.boost.hw.Ron = 7e-3;                          % [Ohm] on-state resistance: Vce/I of the FF450R17ME4 at 250 A, 125 degC (1.72 V)
param.boost.hw.Goff = 1e-6;                         % [1/Ohm]

% design figures (informative)
hw = param.boost.hw;
d.D = D;                                                                       % duty of the lower switch at the operating point
d.M = M;                                                                       % duty of the upper switch (1 - D)
d.i_leg = i_leg;                                                               % [A]
d.i_dc_nom = hw.pwr_nom/hw.udc_nom;                                            % [A] 227 A on the DC link
d.L_min = L_min;                                                               % [H] from spec.r_i_leg
d.Cin_ripple = Cin_ripple;                                                     % [F] from spec.r_u_in, worst duty
d.Cdc_ripple = Cdc_ripple;                                                     % [F] from spec.r_u_dc
d.ripple_leg = hw.uin_nom*D/(hw.L*param.time.fpwm);                            % [A] peak-peak per leg
d.ripple_tot = hw.udc_nom/(hw.L*param.time.fpwm)*(M - k/n)*((k + 1)/n - M)*n; % [A] pk-pk at n*fpwm on the input capacitor, nominal duty
d.ripple_tot_max = hw.udc_nom/(4*n*hw.L*param.time.fpwm);                     % [A] pk-pk at n*fpwm, worst duty
d.ripple_u_in = d.ripple_tot/(8*n*param.time.fpwm*hw.Cin);                    % [V] peak-peak (capacitor only), nominal duty
d.ripple_u_in_max = d.ripple_tot_max/(8*n*param.time.fpwm*hw.Cin);            % [V] peak-peak, worst duty
d.Dp = Dp;                                                                     % duty of the DC-side pulse train at n*fpwm
d.ripple_u_dc = Dp*(1 - Dp)*i_leg/(n*param.time.fpwm*hw.Cdc);                 % [V] peak-peak on the DC link (capacitor only)
d.i_cdc_rms = i_leg*sqrt(Dp*(1 - Dp));                                        % [A] rms current of the DC-link capacitor
d.f_z = f_z;                                                                   % [Hz] right-half-plane zero at nominal load
d.f_c_u_max = f_c_u_max;                                                       % [Hz] f_z/k_fz
d.f0_cm = M*sqrt(n/(hw.L*hw.Cdc))/(2*pi);                                      % [Hz] resonance of the common-mode current plant (open loop), 35 Hz
d.f_in_res = 1/(2*pi*sqrt(hw.L/n*hw.Cin));                                    % [Hz] L/n with the input capacitor, damped by the battery
d.R_load_nom = hw.udc_nom^2/hw.pwr_nom;                                        % [Ohm] 2.904 Ohm on the DC link
d.f_load = 2/(2*pi*d.R_load_nom*hw.Cdc);                                       % [Hz] load corner of the voltage plant, 15 Hz
param.boost.hw.design = d;
fprintf('Boost: D = %.3f, %.0f A per leg, %.0f A on the DC link, ripple %.0f A pk-pk per leg (%.0f %%), %.1f A pk-pk on the input capacitor at %.0f kHz (%.1f A worst duty), u_in ripple %.0f mV (%.0f mV worst duty), udc ripple %.2f V (%.0f A rms in Cdc), RHP zero %.0f Hz, common-mode resonance %.0f Hz, load corner %.0f Hz\n', ...
    d.D, d.i_leg, d.i_dc_nom, d.ripple_leg, d.ripple_leg/d.i_leg*100, d.ripple_tot, n*param.time.fpwm/1e3, d.ripple_tot_max, ...
    d.ripple_u_in*1e3, d.ripple_u_in_max*1e3, d.ripple_u_dc, d.i_cdc_rms, d.f_z, d.f0_cm, d.f_load);
clear hw d k n D M Dp i_leg L_min L_choice RL_choice dI_tot_max Cin_ripple Cdc_ripple Cdc_choice f_z f_c_u_max
%[text] ### PWM
% triangle carriers, one per leg, shifted by 2*pi/n_legs (0, 120, 240 deg: the ripples of the three legs cancel at
% the fundamental and the capacitors see n_legs*fpwm). The modulator is the one of the buck (polarity -1: Q1, upper
% switch, on while ramp <= m - dead_time*fpwm, Q2 on while ramp >= m + dead_time*fpwm): the control computes the
% duty d of the LOWER switch (the boost switch) and the modulator gets m = 1 - d, so the ON pulse of Q2 is centred
% on the top of the carrier and the one of Q1 on the bottom; the leg current (rising while Q2 is on) is at its
% average value at the bottom and at the top of its own carrier. TRGO (control trigger) at the bottom and at the top
% of carrier 1 (double update); the TRGO of carriers 2 and 3 sample-and-hold the currents of legs 2 and 3 and load
% their duties (shadow registers).
param.boost.pwm.tc = param.glb_time.tc;             % [s]   step of the modulator and of the compact block
param.boost.pwm.fpwm = param.glb_time.fPWM_AFE;     % [Hz]
param.boost.pwm.double_update = param.glb_time.TRGO_AFE_double_update;
param.boost.pwm.dead_time = param.glb_time.dead_time_AFE;   % [s]
param.boost.pwm.carrier_phase = [0 2*pi/3 4*pi/3];  % [rad] leg 1, 2, 3
param.boost.pwm.m_up = 0.98;                        %       limits of the duty (of the lower switch; the modulator limits m = 1 - d
param.boost.pwm.m_low = 0.02;                       %       at the same values)
param.boost.pwm.gate_on = 15;                       % [V]
param.boost.pwm.gate_off = -10;                     % [V]
param.boost.pwm.t_enable = 0;                       % [s]   compact block: gates off before this time
%[text] ### Control
% cascade: PI on udc (pu of udc_nom) -> total input current reference (pu of ibez, limited at +-i_lim, integrator
% clamped at the same limit; a negative reference is the buck direction, DC link to battery) -> one PI per leg on
% the leg current sampled at the extremes of its own carrier, reference i_ref/n_legs -> duty d of the lower switch
% of the leg (limited at m_low..m_up, integrator clamped at the same limits) -> m = 1 - d to the modulator. Gains
% from the crossover targets (private/boost_ctrl_param): spec.f_c_i = 225 Hz for the current loops (the plant duty
% -> leg current is the integrator udc_nom/(L*ibez), the same as the buck: same gains; the common-mode resonance
% with the DC link, 35 Hz, is inside the loops and damped by them at any load), spec.f_c_u = 40 Hz for the voltage
% loop at nominal load: the plant is k_u*(1 - s/w_z)/(s + 2/(R_load*Cdc)) behind the closed common-mode current
% loop, k_u = ibez*M/(Cdc*udc_nom) = 47 1/s, right-half-plane zero at 157 Hz, load corner 15 Hz; without load the
% zero disappears (it moves as 1/i_leg) and the plant is the integrator k_u/s, the crossover stays at about 40 Hz.
% Linear check of 2026-10-26 (claude_analysis/interleaved_boost_loop_analysis.py, discrete model at ts with the
% nonlinear averaged model for the time response): current loops crossover 234 Hz on the inductor alone (240 Hz
% in common mode) with 59 deg of phase margin on leg 1, 45 deg and 7.7 dB of gain margin on legs 2-3 (the same at
% no load); voltage loop 40 Hz / 83 deg with 8.1 dB of gain margin at nominal load (the margin on the RHP zero),
% 43 Hz / 78 deg at no load; reference ramp from the precharged DC link (0.303 pu) to 1 pu in 70 ms without
% overshoot and without reaching i_lim (277 A per leg); 2.4 % dip on a +20 % load step recovered within 1 % in 31 ms.
param.boost.ctrl = boost_ctrl_param(param.boost.hw, param.glb_time.ts_afe, spec.f_c_i, spec.f_c_u);
param.boost.ctrl.ts = param.glb_time.ts_afe;        % [s]   control period (compact block)
param.boost.ctrl.mode = 0;                          %       compact block: 0 DC-link voltage loop, 1 total input current reference (i_ref, t_ref, i_ref_slew)
param.boost.ctrl.udc_ref = 1;                       % [pu]  DC-link voltage reference (pu of udc_nom)
param.boost.ctrl.udc_ref_init = param.boost.hw.uin_nom/param.boost.hw.udc_nom;  % [pu] start of the reference ramp: the DC link cannot
                                                    %       be regulated below the battery voltage (0.303 pu)
param.boost.ctrl.u_ref_slew = 10;                   % [pu/s] ramp of the reference at start: 0.303 -> 1 pu in 70 ms (the DC link stores
                                                    %       1.6 kJ at 660 V: a faster ramp with the nominal load connected saturates i_lim)
param.boost.ctrl.i_lim = 1.2;                       % [pu]  limit of the total current reference (900 A, 300 A per leg), also the integrator clamp
param.boost.ctrl.i_ref = 1;                         % [pu]  mode 1 of the compact block: total input current reference
param.boost.ctrl.t_ref = 0;                         % [s]   step time of i_ref
param.boost.ctrl.i_ref_slew = 20;                   % [pu/s]
param.boost.ctrl.adc_q = 1/2^11;                    % [pu]  quantization of the acquisitions (compact block; param.meas.adc_quantization in the model)
param.boost.ctrl.k_time = 1;                        %       local clock of the compact block = master time
param.boost.ctrl.kp_align = 0;                      %       alignment PI of the local clock (compact block), off
param.boost.ctrl.ki_align = 0;
param.boost.ctrl.lim_up_align = 0;
param.boost.ctrl.lim_down_align = 0;
param.boost.spec = spec;                            % the design inputs above (numeric), kept with the rest of param.boost
clear spec
%[text] ### Module behaviour (clock mismatch)
param.boost.behav.time_gain_module_1 = 1;           % local clock of the module versus master time ("global timing"); 1: no mismatch
%[text] ## Power semiconductors
% 1700 V module for the 660 V link: Infineon FF450R17ME4 (EconoDUAL 3, 450 A nominal, 900 A ICRM, IGBT4): one half
% bridge per leg at 250 A (300 A at i_lim), data set of the library (setup_files/semiconductor_devices_data/
% devices_source_dataname/infineon_FF450R17ME4.m, values at 125 degC, loaded by private/device_igbt_param through
% device_data_load). The two data sets embedded in device_igbt_param (FF1800R12IE5, CM1200DW-34T) stay available by name.
param.dev.used_device_igbt = 'infineon_FF450R17ME4';
dev = device_igbt_param(param.dev.used_device_igbt, param.glb_time.fPWM_AFE, param.boost.hw.udc_nom);
param.dev.igbt_name = dev.name;
param.dev.igbt_type = dev.type;
param.boost.igbt = rmfield(dev, {'name', 'type'});  % numeric fields only: param.boost is the mask struct of the compact block
clear dev
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = param.boost.ctrl.adc_q;   % [pu] quantizer of the acquisitions (pu signals)
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
% v = V0 + R*i, switching energies scaled linearly from the data set point to udc and i_leg). Boost roles: the lower
% IGBT conducts for D of the period, the upper diode for 1 - D (the upper IGBT is driven but the reverse current goes
% through its diode), ripple neglected
g = param.boost.igbt; hw = param.boost.hw; D = hw.design.D; i_l = hw.design.i_leg;
p.igbt_cond = D*(g.Vce_sat*i_l + g.Rce_on*i_l^2);                                      % [W] lower IGBT
p.igbt_sw = (g.Eon + g.Eoff)*(i_l/g.Ion_sw_losses)*(hw.udc_nom/g.Voff_sw_losses)*param.time.fpwm;
p.diode_cond = (1 - D)*(g.Vdon_diode*i_l + g.Rdon_diode*i_l^2);                        % [W] upper diode
p.diode_rec = g.Erec*(i_l/g.Ion_sw_losses)*(hw.udc_nom/g.Voff_sw_losses)*param.time.fpwm;
p.leg = p.igbt_cond + p.igbt_sw + p.diode_cond + p.diode_rec;
p.Tj_igbt = param.thermal.Tambient + (p.igbt_cond + p.igbt_sw)*(g.Rth_switch_JH + param.thermal.Rth_switch_HA);   % [degC]
p.Tj_diode = param.thermal.Tambient + (p.diode_cond + p.diode_rec)*(g.Rth_diode_JH + param.thermal.Rth_diode_HA);
p.efficiency = (hw.pwr_nom - hw.n_legs*(p.leg + hw.RL*i_l^2))/hw.pwr_nom;              % input power = nominal power
param.thermal.loss_estimate = p;
fprintf('Losses per leg at %.0f A, %.0f V, %.0f kHz: lower IGBT %.0f W conduction + %.0f W switching, upper diode %.0f W conduction + %.0f W recovery = %.0f W; Tj IGBT %.0f degC, Tj diode %.0f degC with water at %.0f degC; efficiency %.1f %% with the inductors\n', ...
    i_l, hw.udc_nom, param.time.fpwm/1e3, p.igbt_cond, p.igbt_sw, p.diode_cond, p.diode_rec, p.leg, p.Tj_igbt, p.Tj_diode, param.thermal.Tambient, p.efficiency*100);
clear g hw D i_l p
%[text] ## Battery on the low-voltage side
param.battery.nominal_battery_voltage_1 = param.boost.hw.uin_nom;  % [V] 200 V
param.battery.nominal_battery_power = param.boost.hw.pwr_nom;      % [W] 150 kW
param.battery.initial_battery_soc = 0.85;
% ASSUMPTION: 1C rating, capacity = rated current x 1 h
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;   % [Ah] 750 Ah
param.battery.plot_ocv = false;

b = param.battery;
param.battery.pack_1 = battery_param(b.nominal_battery_voltage_1, b.nominal_battery_power, ...
    b.charge_capacity_1, b.initial_battery_soc, param.glb_time.ts_dab, b.plot_ocv);
clear b

% special settings for simulation (same as the AFE models)
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/200;
%[text] ## Load on the DC link
param.load.R_load = param.boost.hw.design.R_load_nom;   % [Ohm] 2.904 Ohm: 150 kW at 660 V (block load/Resistor)
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
