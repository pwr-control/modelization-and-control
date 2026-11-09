%[text] # AFE + inverter + PMSM with high-frequency cable models (afe\_inv\_psm\_with\_hf\_models)
%[text] Initialization of afe\_inv\_psm\_with\_hf\_models.slx after its rebuild on the library blocks of the new architecture: **grid\_following\_dqctrl\_mod1** (AFE: two-level bridge, DC-link capacitor, LCL filter and dq control in one block, folder masked\_models/grid\_following of pwr-control/library) and **pmsm\_inverter\_dqctrl\_mod1** (inverter: the same power stage with an L output filter and the field-oriented control, folder masked\_models/pmsm\_inverter). A 400 V grid (emulator + Dyn11 10 MVA transformer, star point of the secondary grounded) feeds the AFE. The inverter drives one 250 kW system of a permanent magnet synchronous machine, connected at its terminals. The power cable (two lumped high-frequency sections per phase, with capacitors to ground) is connected at the same terminals with its far end open: at high frequency the machine winding is a high impedance, so the open end is the machine terminal for the ringing, while the machine itself is fed without the cable (its 30 uH do not matter at 16 Hz). The model shows the voltage at the far end of the cable and the common-mode current through the cable capacitances. The shaft turns at a fixed speed (fixed\_speed\_setting).
%[text] Each library block takes one struct as mask parameter p, with three fields: p.hw (power stage, the Simscape component afe\_ssc.afe\_two\_level\_lcl), p.pwm (carrier and gates) and p.ctrl (control). The AFE reads param.afe\_follower(1), the inverter param.inv\_mod(1). The field names are those of the library examples grid\_following\_param\_example.m and pmsm\_inverter\_param\_example.m (the Help of the blocks). The numbers are those of the legacy init (init\_model\_legacy.m, deleted in November 2026: see the git history) where the old model had them, otherwise those of the examples. The rest of the model reads param.\* too: grid emulator, cable, machine, logging, scopes.
%[text] The functions of ./private are called through the handles of long\_cable\_effects\_private.m (prv.timing\_param, prv.cable\_param, ...): the Live Editor runs a live script from a copy outside this folder and does not see ./private. Run the whole script once (Run): a section run on its own needs prv and param in the workspace. The last section checks that every param.\* of the active blocks exists and that every active From block reaches a Goto.
clear;
model = 'afe_inv_psm_with_hf_models';
prv = long_cable_effects_private();     % handles to the functions of ./private (the Live Editor does not see ./private)
prv.library_setup();                    % pwr-control/library on the path (blocks, examples), cloned when missing

% environment (was init_environment of the library)
close all; clc; beep off;
try
    pm_addunit('percent', 0.01, '1');       % custom Simscape unit used by the library machine models
catch ME
    if ~contains(ME.message, 'percent')     % already registered in this session: fine
        rethrow(ME);
    end
end
param = struct();
fprintf('%s init: functions of ./private, called as prv.<name>: %s\n', model, strjoin(fieldnames(prv)', ', '));
%%
%[text] ## Simulation
%[text] Run time: glb\_time.tc is the step of the Simscape local solver, the MaxStep of ode23tb and the logging base. With tc = 0.125 us (ts/1000) the slow mode of the cable ringing (116 kHz) has 69 steps per period and the fast mode (about 620 kHz) 13 steps: the Backward Euler solver damps the ringing much more than the cable does (design check, point 1). Halving tc doubles the run time and the logged data.
param.sim.simlength = 1.75;                 % [s] model StopTime (legacy simlength)
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration (grid/grid_emulator): ideal switches
param.sim.check_model = true;               % last section: param.* references and From/Goto of the model
param.sim.design_check = true;              % section "Design check": resonances, damping of the solver, loops
%%
%[text] ## Timing
%[text] AFE and inverter switch at 4 kHz with double update (control period 125 us). Each block makes its own carrier and trigger from its local clock (p.ctrl.k\_time and the alignment PI to the master time time\_master), so glb\_time is only for the logging and the Simscape local solver. The step tc must be the same in the two blocks (p.pwm.tc) and in the Solver Configuration block.
param.time.fpwm_afe = 4e3;                  % [Hz] switching frequency of the AFE
param.time.trgo_afe = 1;                    % double update: control at 2*fpwm = 8 kHz
param.time.dead_time_afe = 3e-6;            % [s]
param.time.fpwm_inv = 4e3;                  % [Hz] switching frequency of the inverter
param.time.trgo_inv = 1;
param.time.dead_time_inv = 3e-6;            % [s]
param.time.t_measure = 0.3;                 % [s] logging window (legacy t_measure, informative)
param.time.tc_factor = 1000;                % ts_afe/tc: glb_time.tc = 125 us/1000 = 0.125 us
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;
param.glb_time = prv.timing_param(param.time);  % the model reads param.glb_time.tc (FixedStep, MaxStep, logging)
%%
%[text] ## Grid
%[text] Three-phase grid emulator (ideal voltage sources, MATLAB Function copy of the library block) and a Dyn11 10 MVA 400/400 V transformer made of three single-phase transformer blocks, star point of the secondary grounded. Its leakage, 2.5 uH at the secondary, is the grid-side inductor of the LCL filter of the AFE: a stiff grid, 40 times the power of the converter.
param.grid.application_voltage = 400;           % [V] rms line-to-line, primary and secondary of the transformer
param.grid.f_nom = 50;                          % [Hz]
param.grid.omega_nom = 2*pi*param.grid.f_nom;   % [rad/s]
param.grid.pwr_nom = 10e6;                      % [VA] rated power of the transformer (legacy grid_nominal_power)
param.grid.ucc = 5;                             % [%] short-circuit voltage of the transformer
param.grid.eta = 95;                            % [%] efficiency of the transformer (copper losses)
param.grid.p_iron = 1000;                       % [W] iron losses of the transformer
param.grid.n2 = 14;                             % turns of the secondary (star) coils of the transformer blocks
param.grid.n1 = floor(param.grid.n2*sqrt(3));   % turns of the primary (delta) coils: 24
param.grid.core_area = 0.05;                    % [m^2] core of the transformer blocks
param.grid.core_length = 2.5;                   % [m]
param.grid.mur = 10e3;                          % [-] relative permeability: Lm1 = 145 mH
param.grid.R_eq = 2e-3;                         % [Ohm] equivalent grid resistance (informative, not in the model)
%[text] ### Fault test of the grid emulator
%[text] The fault is OFF, as in the legacy init (enable\_frt\_1 = enable\_frt\_2 = 0), and the AFE block has no fault ride-through control. The data are those of the legacy test 25.4 (75 percent dip on two phases, variant D, 915 ms from 0.75 s).
param.grid.fault_in.kind = 'two_phase_D';           % fault of the abc amplitudes (enable_frt_1)
param.grid.fault_in.dip = 0.75;                     % [pu]
param.grid.fault_in.error_length = 0.915;           % [s]
param.grid.fault_in.start_time = 0.75;              % [s]
param.grid.fault_in.enable_frt_1 = 0;               % faults generated from abc
param.grid.fault_in.enable_frt_2 = 0;               % faults generated from the positive and negative sequences
param.grid.fault_in.deepPOSxi = 1;                  % [pu] positive sequence xi during the fault: 1*(1 - deepPOSxi)
param.grid.fault_in.deepPOSeta = -0.4;              % [pu] positive sequence eta during the fault: 0 - deepPOSeta
param.grid.fault_in.deepNEGxi = 0.4;                % [pu] negative sequence xi during the fault: 0 - deepNEGxi
param.grid.fault_in.deepNEGeta = 0.4;               % [pu] negative sequence eta during the fault: 0 - deepNEGeta
param.grid.fault_in.t_ramp = 20e-3;                 % [s]
param.grid.frt = prv.grid_fault_param(param.grid.fault_in);
param.grid.emu = prv.grid_emulator_param(param.grid, param.grid.frt);
%%
%[text] ## Power cable
%[text] Per phase, two lumped sections (power\_cables\_and\_psm\_high\_frequency\_models/power\_cable\_model\_1/2/3, subsystems parasitic\_lump\_1 and parasitic\_lump\_2): series inductor and resistor, capacitor to ground at the output of each section. The input of the cable is at the inverter terminals (driver-side measurement), the far end is open (machine-side measurement, then Open Circuit blocks). Numbers of the legacy init (hf\_power\_cable\_model): 30 uH and 30 mOhm in series and 18 nF to ground for the whole cable. The second capacitor Cp2 and the resistor Rp1 of a section are commented out in the model; their values are kept. The commented subsystems afe\_abc\_inv\_psm\_mod1/parasitic\_lump\_1 and parasitic\_lump\_2 (DC bus to ground) read the same data.
param.cable_in.n = 2;                       % sections per phase
param.cable_in.L = 30e-6;                   % [H] series inductance of the whole cable (legacy Ls1, measured with 10 uF)
param.cable_in.R = 30e-3;                   % [Ohm] series resistance of the whole cable
param.cable_in.C = 18e-9;                   % [F] capacitance phase to ground of the whole cable
param.cable_in.RC = 1e-3;                   % [Ohm] series resistance of the capacitor branch
param.cable_in.Cp2 = 2e-10;                 % [F] second capacitor branch of a section (commented in the model)
param.cable_in.RCs2 = 1e-3;                 % [Ohm]
param.cable_in.Rp1 = 80e3;                  % [Ohm] resistor to ground of a section (commented in the model)
param.cable = prv.cable_param(param.cable_in);
%%
%[text] ## AFE (grid\_following\_dqctrl\_mod1)
%[text] Hardware of the old 400 V AFE (library data set afe400V with the overrides of the legacy init): converter-side inductor 330 uH positive sequence and 10 uH zero sequence, with the 5 uH leakage of the old three-phase inductor block, and 360 uF in star with 50 mOhm. The three 1 uH inductors of the old filter (between the inductor and the capacitors) become the grid-side inductor of the block (Lls2 = 1 uH, L2p = L20 = 0; L2 must stay above zero): same total inductance, the transformer leakage adds 2.5 uH. The 7.2 mF DC link of the old model is split: 3.6 mF in the AFE block and 3.6 mF in the inverter block (each block has its own capacitor; Udc+ and Udc- connect the two), 2 mOhm each (1 mOhm in parallel, as the library data set). There is no brake chopper in the new architecture.
%[text] ### Power stage
a = struct();
a.hw.Vth  = 2.5;                            % [V] gate threshold: a switch is on when its gate is above Vth
a.hw.Ron  = 1e-3;                           % [Ohm] on resistance of switch and diode
a.hw.Goff = 1e-6;                           % [1/Ohm] off conductance
a.hw.Cdc  = 3.6e-3;                         % [F] half of the 7.2 mF DC link (legacy hwdata.afe.CFi)
a.hw.Rdc  = 2e-3;                           % [Ohm] its series resistance (the two halves in parallel: 1 mOhm)
a.hw.L1p  = 330e-6;                         % [H] converter-side inductor (legacy LFu), positive sequence
a.hw.L10  = 10e-6;                          % [H] zero sequence (legacy override LFu_cm = 10 uH)
a.hw.R1   = 157*0.05*330e-6;                % [Ohm] 2.6 mOhm, legacy RLFu (5 % of the reactance at 25 Hz)
a.hw.Lls1 = 5e-6;                           % [H] leakage (default of the old three-phase inductor block)
a.hw.Cf   = 360e-6;                         % [F] filter capacitor per phase, floating star (legacy CFu)
a.hw.Rcf  = 50e-3;                          % [Ohm] its series resistance (legacy RCFu)
a.hw.L2p  = 0;                              % [H] no grid-side inductor in the block: the transformer leakage
a.hw.L20  = 0;                              %     (2.5 uH at the secondary) is the grid-side inductor
a.hw.R2   = 0;                              % [Ohm]
a.hw.Lls2 = 1e-6;                           % [H] the three 1 uH inductors of the old LCL filter (must be > 0)
a.hw.ubez = param.grid.application_voltage*sqrt(2/3);                   % [V] 326.6 V: peak phase voltage
a.hw.ibez = 250e3/sqrt(3)/param.grid.application_voltage/0.9*sqrt(2);   % [A] 567.0 A: 250 kW, 10 % margin
a.hw.udc_nom = 660;                         % [V] nominal DC-link voltage (base of udc)
%[text] ### Carrier and gates
a.pwm.tc            = param.glb_time.tc;    % [s] sample time of the block = step of the Simscape local solver
a.pwm.fpwm          = param.time.fpwm_afe;  % [Hz] PWM frequency
a.pwm.phase         = 0;                    % [rad] carrier phase
a.pwm.double_update = param.time.trgo_afe;  % 1: control at both ends of the carrier
a.pwm.dead_time     = param.time.dead_time_afe;   % [s]
a.pwm.gate_on       = 15;                   % [V] gate voltage of a switch that is on (above hw.Vth)
a.pwm.gate_off      = -10;                  % [V] gate voltage of a switch that is off
a.pwm.m_low         = 0.02;                 % [-] lower limit of the duties
a.pwm.m_up          = 0.98;                 % [-] upper limit of the duties
%[text] ### Control
%[text] Gains of the library example (same 400 V, 250 kW hardware) except the DC-link voltage PI: kp\_udc = 2 as afe\_ctrl.kp\_udc\_ctrl of the legacy init. The clock ratio k\_time = 1.0002 is the old time gain of global timing/time\_afe1; the alignment PI keeps the legacy gains 0.25 and 18 (stable up to about 4 s of simulated time, see the README). The zero-sequence PI does not act (u\_cm = 0, as u\_cm\_comp\_mod1 of the legacy init).
ts = param.glb_time.ts_afe;                 % [s] control period, 125 us
w = param.grid.emu.omega_grid_nom;          % [rad/s] nominal grid pulsation
a.ctrl.ts     = ts;
a.ctrl.ts_int = ts;                         % [s] sample time of the PI integrators

% local clock and alignment to the master time: t_loc = t_master*k_time*(1 + corr)
a.ctrl.k_time         = 1.0002;             % [-] legacy time_gain_afe_module_1
a.ctrl.kp_align       = 0.25;               % legacy kp_align
a.ctrl.ki_align       = 18;                 % [1/s] legacy ki_align
a.ctrl.lim_up_align   = 0.05;
a.ctrl.lim_down_align = -0.05;

% acquisitions
a.ctrl.adc_q = 2^-11;                       % [pu] quantization step (legacy adc_quantization)

% first harmonic tracker of the terminal voltage, positive sequence, PLL
a.ctrl.fht_omega   = w;
a.ctrl.fht_delta   = 0.05;                  % [-] damping of the resonant model
[a.ctrl.fht_l1, a.ctrl.fht_l2] = prv.fht_gains(ts, w, a.ctrl.fht_delta, [-1 -4]);   % poles [-1 -4]*omega
a.ctrl.fht_t_reset = 1e-3;                  % [s] the tracker is held at zero until this time
a.ctrl.seq_gain     = 0.5;                  % [-] gain of the positive sequence, with pll_err_sign = +1
a.ctrl.pll_err_sign = 1;
a.ctrl.u_mag_min    = 0.02;                 % [pu] lower limit of |u| in the normalization 1/|u|
a.ctrl.pll_p        = 1;                    % [pu of omega_nom per pu of error] legacy pll_p
a.ctrl.pll_i1       = 80;                   % [1/s] legacy pll_i1
a.ctrl.omega_nom    = w;

% double integrator observer on the PLL angle (frequency estimate for the log)
a.ctrl.obs_A = [1 ts; 0 1];
a.ctrl.obs_C = [1 0];
a.ctrl.obs_L = prv.observer_gains(a.ctrl.obs_A, exp(ts*[-1 -4]*w));

% DC-link voltage loop: reference step at udc_t_step, low-pass udc_tau, PI -> active current reference
a.ctrl.udc_t_step   = 1e-3;                 % [s]
a.ctrl.udc_norm_ref = 1;                    % [pu of udc_nom] 660 V
a.ctrl.udc_tau      = 20e-3;                % [s]
a.ctrl.kp_udc       = 2;                    % [pu/pu] legacy afe_ctrl.kp_udc_ctrl (example: 0.85)
a.ctrl.ki_udc       = 35;                   % [1/s]
a.ctrl.ixi_lim      = 1.65;                 % [pu] limit of the active current reference
a.ctrl.ieta_ref     = 0;                    % [pu] reactive current reference (the legacy steps came after 1.75 s)

% current loop: dq vector PI on the converter current, output = modulation (1 = udc/sqrt(3))
a.ctrl.kp_i         = 0.75;                 % [pu/pu]
a.ctrl.ki_i         = 35;                   % [1/s]
a.ctrl.u_lim        = 1;                    % [pu] radius of the voltage reference
a.ctrl.clip_release = 0.001;                % [1/s] legacy CTRPIFF_CLIP_RELEASE

% zero-sequence current loop and its path to the modulator
a.ctrl.iz_ref      = 0;                     % [pu]
a.ctrl.kp_iz       = 0.5;                   % [pu/pu] legacy kp_is_z
a.ctrl.ki_iz       = 18;                    % [1/s] legacy ki_is_z
a.ctrl.iz_lim      = 1;                     % [pu] saturation of the PI
a.ctrl.iz_out_gain = 1;                     % [-] gain from the PI output to u_gamma
a.ctrl.u_cm        = 0;                     % [-] standard SVPWM: the zero-sequence PI does not act
param.afe_follower = a;

% read by the model outside the block
param.afe.hw = a.hw;                        % bases ubez, ibez, udc_nom: scopes and the measurement Gotos
param.afe.behav.time_gain_module_1 = 1;     % global timing/time_afe1 (commented out: the block has its own clock)
fprintf('AFE: L1 %.0f uH (zero seq. %.0f uH), Cf %.0f uF, Cdc %.1f mF, ubez %.1f V, ibez %.1f A, udc_nom %.0f V | kp_i %.2f, ki_i %.0f, kp_udc %.2f, ki_udc %.0f, k_time %.4f\n', ...
    a.hw.L1p*1e6, a.hw.L10*1e6, a.hw.Cf*1e6, a.hw.Cdc*1e3, a.hw.ubez, a.hw.ibez, a.hw.udc_nom, ...
    a.ctrl.kp_i, a.ctrl.ki_i, a.ctrl.kp_udc, a.ctrl.ki_udc, a.ctrl.k_time);
clear a ts w
%%
%[text] ## Machine
%[text] Data of psm\_calculus\_with\_variants of the library at 400 V (MotorDrive): four three-phase systems on one rotor, 1 MW, 1880 A rms, 24.6 rpm, 80 poles; Ld, Lq and Jm of the whole machine. private/psm\_machine\_param gives the values of one system and its bases: Rs 15.1 mOhm, Ld 2.0 mH, Lq 3.2 mH, ubez 250.7 V, ibez 664.7 A, torque base 97.0 kNm, 16.4 Hz at 24.6 rpm. The machine block of the model is one system (250 kW).
param.psm.machine_in.name = 'MotorDrive';
param.psm.machine_in.number_of_systems = 4;
param.psm.machine_in.pwr_nom = 1000e3;             % [W]
param.psm.machine_in.i_nom = 1880;                 % [A] rms, all systems
param.psm.machine_in.rpm_nom = 24.6;               % [rpm]
param.psm.machine_in.number_poles = 80;
param.psm.machine_in.eta = 96;                     % [%] (sets Rs)
param.psm.machine_in.Ld = 0.5e-3;                  % [H] whole machine
param.psm.machine_in.Lq = 0.8e-3;                  % [H]
param.psm.machine_in.Jm = 900e3/4;                 % [kg m^2]
param.psm.machine_in.h5_percent = 0;               % [%] 5th and 7th flux harmonics (the block uses literals 0)
param.psm.machine_in.h7_percent = 0;
param.psm.machine = prv.psm_machine_param(param.psm.machine_in);
%%
%[text] ## Mechanical side
%[text] Energy production: the shaft turns at rpm\_sim (block fixed\_speed\_setting: step at 25 ms, first-order lag), and the torque reference of the inverter is the torque curve of the legacy script n\_sys\_generic\_1M5W\_torque\_curve, scaled by torque\_overload\_factor (second element of the ref input of the block).
%[text] **The curve is now used in per unit.** The script gives the torque of the 17.8 rpm generator of the 690 V models (the 1.6 MW WindGen of inv\_psm and afe\_inv\_psm\_cascaded), while this machine is the 1 MW, 24.6 rpm MotorDrive. The legacy model read the curve in rpm and in Nm per system: at its 17.8 rpm the curve asked 187.5 kNm per system, 1.93 times the torque base of MotorDrive (about 350 kW in a 250 kW system). Here the speed is in pu of 17.8 rpm and the torque in pu of 858.4 kNm (1.6 MW at 17.8 rpm), as in the 690 V models, and the shaft turns at the rated speed of this machine: 0.87 pu of torque, about 218 kW. For the legacy speed set rpm\_sim = 17.8 (then 0.26 pu of torque, about 47 kW).
param.load.rpm_base = [6 7 8 9 10 11 12 13 14 15 16 17 17.8 18.0 18.2];      % [rpm] speed axis of the curve
param.load.torque_base = 1e3*[50 75 100 125 150 175 200 225 275 350 450 600 750 885 885];   % [Nm] whole generator
param.load.rpm_curve_nom = 17.8;                        % [rpm] rated speed of the generator of the curve
param.load.torque_curve_nom = 1.6e6/(17.8/60*2*pi);     % [Nm] its rated torque: 858.4 kNm
param.load.torque_overload_factor = 1;                  % [-] scale of the curve (inv_dqctrl_psm_mod1/Constant)
param.load.rpm_sim = param.psm.machine.rpm_nom;         % [rpm] speed of the shaft: 24.6 (legacy 17.8)

% speed control (p.ctrl.mode = 1, not used here): load inertia and torque step of motor_load_setting
param.load.external_load_inertia = 1;                   % [kg m^2] motor_load_setting/Inertia
param.sim.load_step_time = 1.25;                        % [s] motor_load_setting/PS Step (legacy load_step_time)
%%
%[text] ## Inverter (pmsm\_inverter\_dqctrl\_mod1)
%[text] Hardware of the old inverter: three-phase inductor of 80 uH positive sequence and 5 uH zero sequence (legacy overrides LFi and LFi\_cm) with the 5 uH leakage of the old inductor block, and the three 1 uH inductors in series (Lls2). There is no filter capacitor: the capacitor of the block is switched off by its series resistance (Rcf = 1e5 Ohm). The bases of the measurements are those of one machine system. The angle comes from the EKF (angle\_src = 2), as in the legacy init (EKF on the back-EMF, no encoder); the encoder angle theta\_elec\_psm is wired to the block too, for angle\_src = 0.
%[text] Current PI from the rule of the library example: crossover 250 Hz, PI zero at 250/8 Hz, plant Lq + filter in pu (kp\_i about 13.8, ki\_i about 2700). There is one inverter on the machine, so no circulating current between parallel inverters (the reason for the low gains of afe\_inv\_psm\_cascaded). The inverter enables its gates at t\_enable = 0.25 s (legacy time\_start\_motor\_control), after the enable step of its ref input at 0.125 s.
mc = param.psm.machine;
v = struct();
%[text] ### Power stage
v.hw.Vth  = 2.5;                            % [V]
v.hw.Ron  = 1e-3;                           % [Ohm]
v.hw.Goff = 1e-6;                           % [1/Ohm]
v.hw.Cdc  = 3.6e-3;                         % [F] other half of the 7.2 mF DC link
v.hw.Rdc  = 2e-3;                           % [Ohm]
v.hw.L1p  = 80e-6;                          % [H] output inductor (legacy override LFi), positive sequence
v.hw.L10  = 5e-6;                           % [H] zero sequence (legacy override LFi_cm)
v.hw.R1   = 157*0.05*160e-6;                % [Ohm] 1.26 mOhm, legacy RLFi (5 % rule on the 160 uH of the data set)
v.hw.Lls1 = 5e-6;                           % [H] leakage (default of the old three-phase inductor block)
v.hw.Cf   = 1e-6;                           % [F] capacitor branch switched off ...
v.hw.Rcf  = 1e5;                            % [Ohm] ... by its series resistance (L filter)
v.hw.L2p  = 0;                              % [H] no second inductor ...
v.hw.L20  = 0;
v.hw.R2   = 0;                              % [Ohm]
v.hw.Lls2 = 1e-6;                           % [H] ... only the three 1 uH inductors of the old L filter (must be > 0)
v.hw.ubez = mc.ubez;                        % [V] 250.7 V: bases of the measurements = bases of one system
v.hw.ibez = mc.ibez;                        % [A] 664.7 A
v.hw.udc_nom = 660;                         % [V] nominal DC-link voltage
%[text] ### Carrier and gates
v.pwm.tc            = param.glb_time.tc;    % [s] = step of the Simscape local solver
v.pwm.fpwm          = param.time.fpwm_inv;  % [Hz]
v.pwm.phase         = 0;                    % [rad]
v.pwm.double_update = param.time.trgo_inv;
v.pwm.dead_time     = param.time.dead_time_inv;   % [s]
v.pwm.gate_on       = 15;                   % [V]
v.pwm.gate_off      = -10;                  % [V]
v.pwm.m_low         = 0.02;
v.pwm.m_up          = 0.98;
%[text] ### Control
v.ctrl.ts = param.glb_time.ts_inv;          % [s] control period, 125 us

% local clock and alignment to the master time (legacy gains, as the AFE)
v.ctrl.k_time         = 1.001;              % [-] the old global timing/time_inv1 used time_gain_inv_module_2 = 1.001
v.ctrl.kp_align       = 0.25;
v.ctrl.ki_align       = 18;                 % [1/s]
v.ctrl.lim_up_align   = 0.05;
v.ctrl.lim_down_align = -0.05;

% acquisitions
v.ctrl.adc_q = 2^-11;                       % [pu]

% machine model of the control: one system, pu of v.hw.ubez, v.hw.ibez, omega_b (Lmu: leakage of the machine block)
v.ctrl.omega_b = mc.omega_bez;              % [rad/s] 103.0 rad/s
v.ctrl.rs      = mc.Rs_m/mc.Xbez;           % 0.0400
v.ctrl.ld      = (mc.Ld_m + mc.Lmu)/mc.Lbez;   % 0.548
v.ctrl.lq      = (mc.Lq_m + mc.Lmu)/mc.Lbez;   % 0.876
v.ctrl.psi_m   = mc.psi_m*mc.omega_bez/mc.ubez;   % 1 with the bases of the machine

% operation
v.ctrl.mode      = 2;                       % 0 torque reference, 1 speed control, 2 torque curve (generator)
v.ctrl.mtpa      = 1;                       % 1 MTPA (legacy use_mtpa), 0 id = 0
v.ctrl.angle_src = 2;                       % 0 encoder, 1 back-EMF observer, 2 EKF (legacy use_ekf_bemf_module_1)
v.ctrl.omega_th  = 0;                       % [pu] encoder below this speed with angle_src 1 or 2 (0: never)
v.ctrl.t_enable  = 0.25;                    % [s] gates off before (legacy time_start_motor_control)
v.ctrl.k_delay   = 1.5;                     % [periods] angle advance of the voltage (delay compensation)

% torque and current references
v.ctrl.torque_lim  = 1.5;                   % [pu] limit of the torque reference
v.ctrl.torque_slew = 2;                     % [pu/s] slew rate of the torque reference
v.ctrl.i_lim       = 1.6;                   % [pu] limit of the current vector
v.ctrl.id_max      = 0.35;                  % [pu] largest |id| of the MTPA

% speed control (mode 1 only): PI and filters of the library example
v.ctrl.kp_w     = 2.5;                      % [pu/pu]
v.ctrl.ki_w     = 18;                       % [1/s]
v.ctrl.w_ref_fc = 1;                        % [Hz] filter of the speed reference
v.ctrl.w_flt_fc = 10;                       % [Hz] filter of the estimated speed (speed PI and torque curve)

% torque curve (mode 2): speed in pu of the speed of the curve, torque in pu of its rated torque (see above)
v.ctrl.curve_w = param.load.rpm_base/param.load.rpm_curve_nom;
v.ctrl.curve_T = param.load.torque_base/param.load.torque_curve_nom;

% current loop: crossover fc_i, PI zero at fc_i/8, plant L = Lq + L1 + L2 (pu), rule of the library example
fc_i = 250;                                 % [Hz]
l_tot = v.ctrl.lq + (v.hw.L1p + v.hw.Lls1 + v.hw.L2p + v.hw.Lls2)/mc.Lbez;
v.ctrl.kp_i         = 2*pi*fc_i*l_tot/mc.omega_bez;   % [pu/pu] pu of ubez per pu of ibez
v.ctrl.ki_i         = v.ctrl.kp_i*2*pi*fc_i/8;        % [1/s]
v.ctrl.u_lim        = 1;                    % [pu of vdc/sqrt(3)] radius of the voltage (linear SVPWM)
v.ctrl.clip_release = 0.001;                % [1/s]
v.ctrl.ff           = 1;                    % 1: feed-forward of back-EMF and cross-coupling
v.ctrl.dt_comp      = 1;                    % 1: dead-time compensation in the duties
v.ctrl.dt_band      = 0.01;                 % [pu] current at which the dead-time sign saturates

% observers (they all run; angle_src selects the one in use)
v.ctrl.pll_poles = [-1 -4]*mc.omega_bez;    % [rad/s] tracking loops of the encoder and of the back-EMF observer
v.ctrl.bemf_fb   = 0.2;                     % [-] correction gain of the back-EMF observer
v.ctrl.bemf_leak = 0;                       % [-]
v.ctrl.ekf_q  = [1e-6; 1e-6; 1e-7; 1e-7; 1e-6];   % EKF: process noise variances per period
v.ctrl.ekf_r  = [1e-5; 1e-5];               % measurement noise variances
v.ctrl.ekf_p0 = [1e-4; 1e-4; 1; 1; 1];      % initial variances
v.ctrl.ekf_w0 = 0;                          % [pu] initial speed of the EKF
param.inv_mod = v;

% read by the model outside the block
param.inv.hw = v.hw;                        % bases ibez, ubez: common-mode current is_uvw_cm (inv_dqctrl_psm_mod1)
param.inv.t_ref_enable = 0.125;             % [s] inv_dqctrl_psm_mod1/Step: enable of the ref input of the block
param.inv.behav.time_gain_module_1 = 1;     % global timing/time_inv1 (commented out: the block has its own clock)
fprintf('Inverter: L %.0f uH (zero seq. %.0f uH), Cdc %.1f mF, ubez %.1f V, ibez %.1f A | pu: rs %.4f, ld %.4f, lq %.4f, filter %.4f | kp_i %.2f, ki_i %.0f | k_time %.4f\n', ...
    (v.hw.L1p + v.hw.Lls1 + v.hw.Lls2)*1e6, (v.hw.L10 + v.hw.Lls1 + v.hw.Lls2)*1e6, v.hw.Cdc*1e3, v.hw.ubez, v.hw.ibez, ...
    v.ctrl.rs, v.ctrl.ld, v.ctrl.lq, (v.hw.L1p + v.hw.Lls1 + v.hw.L2p + v.hw.Lls2)/mc.Lbez, v.ctrl.kp_i, v.ctrl.ki_i, ...
    v.ctrl.k_time);
clear v mc fc_i l_tot
%%
%[text] ## Scope filters
% second-order 50 Hz low-pass filter of scopes/machine_side/LTI System and LTI System1
tau_sof = 1/(2*pi*50);                      % [s]
s = tf('s');
param.ctrl.filters.inv.SOF_50Hz.fcut = 50;  % [Hz]
param.ctrl.filters.inv.SOF_50Hz.time_constant = tau_sof;
param.ctrl.filters.inv.SOF_50Hz.ss_c = ss(1/(tau_sof*s + 1)^2);
% mean of u^2 over about one grid period at tc (scopes/cable_effects/rms), library rms_filter_setup
param.ctrl.rms_tc.omega_base = param.grid.emu.omega_grid_nom;   % [rad/s]
param.ctrl.rms_tc.ts = param.glb_time.tc;                       % [s]
param.ctrl.rms_tc.n1 = 2*pi*1/param.ctrl.rms_tc.omega_base/param.ctrl.rms_tc.ts;    % one period: 160000 steps
param.ctrl.rms_tc.n2 = 2*param.ctrl.rms_tc.n1;
param.ctrl.rms_tc.n10 = 10*param.ctrl.rms_tc.n1;
clear tau_sof s
%%
%[text] ## Checks of the parameters
%[text] The structs of the two blocks against the examples of the library (a missing field stops the compilation with a message about the inside of the block), and the timing of the blocks against the Simscape step.
issues = [prv.check_block_struct(param.afe_follower, grid_following_param_example(param.glb_time.tc), 'param.afe_follower'), ...
          prv.check_block_struct(param.inv_mod, pmsm_inverter_param_example(param.glb_time.tc), 'param.inv_mod')];
blk = {param.afe_follower, param.inv_mod};
for k = 1:numel(blk)
    p = blk{k};
    n_tc = p.ctrl.ts/p.pwm.tc;
    t_ctrl = 1/(p.pwm.fpwm*(1 + p.pwm.double_update));
    if abs(p.pwm.tc - param.glb_time.tc) > 1e-12 || abs(n_tc - round(n_tc)) > 1e-6 || abs(p.ctrl.ts - t_ctrl) > 1e-12
        issues{end+1} = sprintf(['block struct %d: tc = %g s, ts = %g s; tc must be glb_time.tc (%g s), ts an integer ' ...
            'multiple of tc and equal to 1/(fpwm*(1 + double_update))'], k, p.pwm.tc, p.ctrl.ts, param.glb_time.tc); %#ok<SAGROW>
    end
end
if isempty(issues)
    fprintf('%s init: the structs of the two blocks have the fields of the library examples, timing consistent\n', model);
else
    warning('%s init: %d point(s) in the block structs:\n  %s', model, numel(issues), strjoin(issues, '\n  '));
end
clear issues blk k p n_tc t_ctrl
%%
%[text] ## Design check
%[text] Resonances and loops with the numbers above: LCL filter of the AFE on the transformer leakage (against the limit fs/6 of the converter-current feedback), the cable modes (differential and common mode) against the numerical damping of the Backward Euler local solver at tc, the charge of the DC link at the start, the DC-link voltage loop, the current loop and the operating point of the machine, the clock alignment loops. Change a number and run this section again.
if param.sim.design_check
    param.design = prv.design_check(param);
end
%%
%[text] ## Model setup
open_system(model);
%[text] ### Scopes closed at start
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### Mechanical setting
% torque curve or torque reference: the shaft turns at a fixed speed; speed control: the load inertia
if param.inv_mod(1).ctrl.mode ~= 1
    set_param([model '/fixed_speed_setting'], 'Commented', 'off');
    set_param([model '/motor_load_setting'], 'Commented', 'on');
else
    set_param([model '/fixed_speed_setting'], 'Commented', 'on');
    set_param([model '/motor_load_setting'], 'Commented', 'off');
end
%[text] ### Bus of the grid power meter
%[text] Simulink.Bus PowerMeterIecBus for the output m of the MATLAB Function block grid/three\_phase\_power\_meter\_iec (Symbols pane: Type = Bus: PowerMeterIecBus). Bus objects stay in the base workspace.
prv.power_meter_bus();
%[text] ### Checks of the model
%[text] Every param.\* read by the active blocks must exist, and every active From block must reach an active Goto with its tag.
if param.sim.check_model
    [missing_refs, model_refs] = prv.check_model_param_refs(model, param);
    if isempty(missing_refs)
        fprintf('%s init: all %d param.* references of the active blocks resolve\n', model, numel(model_refs));
    else
        warning('%s init: %d param.* references of the active blocks do not exist in param:\n  %s', model, ...
            numel(missing_refs), strjoin(missing_refs', '\n  '));
    end
    lost = prv.check_from_goto(model);
    if isempty(lost)
        fprintf('%s init: every active From block reaches an active Goto\n', model);
    else
        warning('%s init: %d tag(s) read by active From blocks without an active Goto:\n  %s', model, ...
            numel(lost), strjoin(lost, '\n  '));
    end
    clear missing_refs model_refs lost
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":33.8}
%---
