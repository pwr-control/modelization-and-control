%[text] # Two AFE + inverter modules in parallel on one PMSM generator (afe\_inv\_psm\_n)
%[text] Initialization of afe\_inv\_psm\_n.slx with the library blocks of the new architecture: two **grid\_following\_dqctrl\_mod1** (AFE: two-level bridge, DC-link capacitor, LCL filter and dq control in one block, folder masked\_models/grid\_following of pwr-control/library) and two **pmsm\_inverter\_dqctrl\_mod1** (inverter: the same power stage with an L output filter and the field-oriented control, folder masked\_models/pmsm\_inverter). A 690 V grid (emulator + Dyn11 1 MVA transformer, star point of the secondary grounded) feeds two identical back-to-back modules in hard parallel: the two AFEs at the transformer secondary, the two inverters at the terminals of one permanent magnet synchronous generator. The machine is two of the six 267 kW systems of the 1.6 MW, 17.8 rpm, 104-pole WindGen generator, and its shaft turns at a fixed speed (fixed\_speed\_setting).
%[text] Each block takes one struct as mask parameter p, with three fields: p.hw (power stage, the Simscape component afe\_ssc.afe\_two\_level\_lcl), p.pwm (carrier and gates) and p.ctrl (control). The AFEs read param.afe\_follower(1) and param.afe\_follower(2), the inverters param.inv\_mod(1) and param.inv\_mod(2). The field names are those of the library examples grid\_following\_param\_example.m and pmsm\_inverter\_param\_example.m (the Help of the blocks); the numbers below are those of this model. The two modules differ only in the clock ratio p.ctrl.k\_time and in the zero-sequence factor p.ctrl.u\_cm of the AFE.
%[text] The functions of ./private are called through the handles of afe\_inv\_psm\_cascaded\_private.m (prv.timing\_param, prv.loop\_design\_check, ...): the Live Editor runs a live script from a copy outside this folder and does not see ./private. Run the whole script once (Run): a section run on its own needs prv and param in the workspace.
%[text] Before the first run, the model needs five fixes (README, section "Model check"): the blocks of module 2 read index (1), the four blocks must get time\_master at t\_master, the inverter ref constant must be param.inv.ref, enc must be theta\_elec\_psm directly (not a bus), and 38 From blocks (25 tags) have no active Goto. The last section checks these points and prints what is still open.
clear;
model = 'afe_inv_psm_n';
prv = afe_inv_psm_cascaded_private();   % handles to the functions of ./private (the Live Editor does not see ./private)
prv.library_setup();                    % pwr-control/library on the path (blocks, examples), cloned when missing %[output:158291a7]

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
fprintf('%s init: functions of ./private, called as prv.<name>: %s\n', model, strjoin(fieldnames(prv)', ', ')); %[output:1243faa5]
%%
%[text] ## Simulation
param.sim.simlength = 1.25;                 % [s] model StopTime
param.sim.tc = 0.625e-6;                    % [s] FixedStep of the model; the step that matters is glb_time.tc
                                            %     (MaxStep of ode23tb and step of the Simscape local solver)
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration (grid/grid_emulator): ideal switches
param.sim.check_model = true;               % last section: param.* references and connections of the blocks
param.sim.design_figures = true;            % print the design figures of the loops (section "Design figures")
param.sim.design_plots = false;             % ... and draw the open loops of the two current loops
%%
%[text] ## Timing
%[text] All four converters switch at 4 kHz with double update (control period 125 us). Each block makes its own carrier and TRGO from its local clock, so glb\_time is only for the logging, the Unit Delays of global timing and the Simscape local solver. The step tc must be the same in the four blocks (p.pwm.tc) and in the Solver Configuration block.
param.time.fpwm_afe = 4e3;                  % [Hz] switching frequency of the AFEs
param.time.trgo_afe = 1;                    % double update: control at 2*fpwm = 8 kHz
param.time.dead_time_afe = 3e-6;            % [s]
param.time.fpwm_inv = 4e3;                  % [Hz] switching frequency of the inverters
param.time.trgo_inv = 1;
param.time.dead_time_inv = 3e-6;            % [s]
param.time.t_measure = param.sim.simlength; % [s] logging window
param.time.tc_factor = 200;                 % ts_afe/tc: glb_time.tc = 125 us/200 = 0.625 us
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;
param.glb_time = prv.timing_param(param.time); %[output:36073821]
%%
%[text] ## Grid
%[text] Three-phase grid emulator (ideal voltage sources) and a Dyn11 1 MVA 690/690 V transformer made of three single-phase transformer blocks, star point of the secondary grounded. Its leakage, 77 uH at the secondary, is the grid-side inductor of the LCL filters of the two AFEs. The turns 24/14 give 697 V at the secondary (1 percent above 690 V).
param.grid.application_voltage = 690;           % [V] rms line-to-line, primary and secondary of the transformer
param.grid.f_nom = 50;                          % [Hz]
param.grid.omega_nom = 2*pi*param.grid.f_nom;   % [rad/s]
param.grid.pwr_nom = 1000e3;                    % [VA] rated power of the transformer
param.grid.ucc = 5;                             % [%] short-circuit voltage of the transformer
param.grid.eta = 95;                            % [%] efficiency of the transformer (copper losses)
param.grid.p_iron = 1800;                       % [W] iron losses of the transformer
param.grid.n2 = 14;                             % turns of the secondary (star) coils of the transformer blocks
param.grid.n1 = floor(param.grid.n2*sqrt(3));   % turns of the primary (delta) coils: 24
param.grid.core_area = 0.05;                    % [m^2] core of the transformer blocks
param.grid.core_length = 2.5;                   % [m]
param.grid.mur = 10e3;                          % [-] relative permeability: Lm1 = 145 mH
param.grid.R_eq = 2e-3;                         % [Ohm] equivalent grid resistance (informative, not in the model)
%[text] ### Fault test of the grid emulator
%[text] The fault is OFF: the new AFE block has no fault ride-through control (no negative-sequence current control, no reactive current injection on a dip), so the first runs are in normal operation. To run the legacy test 25.4 (75 percent dip on two phases, variant D, from 0.75 s, given as positive and negative sequence references), set enable\_frt\_2 = 1.
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
param.grid.emu = prv.grid_emulator_param(param.grid, param.grid.frt); %[output:818fe05d]
%%
%[text] ## AFE modules (grid\_following\_dqctrl\_mod1)
%[text] Same hardware as the AFE of the old modules: LCL filter 500 uH, 200 uF in star with 50 mOhm. The grid-side inductor of the block is only a 1 uH connection (L2p = L20 = 0, Lls2 = 1 uH; L2 must stay above zero) because the transformer leakage plays that role. The 7.2 mF DC link of the old module is split: 3.6 mF in the AFE block and 3.6 mF in the inverter block (each block has its own capacitor; Udc+ and Udc- connect the two). There is no brake chopper in the new architecture.
%[text] ### Power stage
a = struct();
a.hw.Vth  = 2.5;                            % [V] gate threshold: a switch is on when its gate is above Vth
a.hw.Ron  = 1e-3;                           % [Ohm] on resistance of switch and diode
a.hw.Goff = 1e-6;                           % [1/Ohm] off conductance
a.hw.Cdc  = 3.6e-3;                         % [F] half of the 7.2 mF DC link of the module
a.hw.Rdc  = 1e-3;                           % [Ohm] its series resistance
a.hw.L1p  = 500e-6;                         % [H] converter-side inductor (LFu of the old module), positive sequence
a.hw.L10  = 500e-6;                         % [H] zero sequence (three independent phases)
a.hw.R1   = 157*0.05*a.hw.L1p;              % [Ohm] 3.9 mOhm: 5 % of the reactance at 25 Hz (library rule)
a.hw.Lls1 = 0;                              % [H] leakage
a.hw.Cf   = 200e-6;                         % [F] filter capacitor per phase, floating star
a.hw.Rcf  = 50e-3;                          % [Ohm] its series resistance
a.hw.L2p  = 0;                              % [H] no grid-side inductor in the block: the transformer leakage
a.hw.L20  = 0;                              %     (77 uH at the secondary) is the grid-side inductor
a.hw.R2   = 0;                              % [Ohm]
a.hw.Lls2 = 1e-6;                           % [H] connection to the transformer (must be > 0)
a.hw.ubez = param.grid.application_voltage*sqrt(2/3);                   % [V] 563.4 V: peak phase voltage
a.hw.ibez = 250e3/sqrt(3)/param.grid.application_voltage/0.9*sqrt(2);   % [A] 328.7 A: 250 kW, 10 % margin
a.hw.udc_nom = 1070;                        % [V] nominal DC-link voltage (base of udc)
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
%[text] Gains of the library example (the study model grid\_follower\_dq) except the DC-link voltage PI: kp\_udc = 2 as in the old modules (crossover about 11 Hz, phase margin 75 deg with C = 7.2 mF; the example 0.85 gives 44 deg). Current PI 0.75 / 35: with the two modules on the transformer the LCL mode is at about 1.1 kHz, damped 7 percent; kp\_i above about 1.7 makes it unstable (section "Design figures").
ts = param.glb_time.ts_afe;                 % [s] control period, 125 us
w = param.grid.emu.omega_grid_nom;          % [rad/s] nominal grid pulsation
a.ctrl.ts     = ts;
a.ctrl.ts_int = ts;                         % [s] sample time of the PI integrators

% local clock and alignment to the master time (k_time per module below): t_loc = t_master*k_time*(1 + corr),
% so the loop gain grows with the time (kp_align*t). ki_align 18 of the old model aligns the carriers only after
% about 0.9 s (those of the two AFEs pass through opposition at 0.11 s); 400 aligns them within 2 us by 0.15 s.
% The loop stays stable up to about 3.6 s of simulated time (4 s with 18): see the README for longer runs.
a.ctrl.k_time         = 1;
a.ctrl.kp_align       = 0.25;
a.ctrl.ki_align       = 400;                % [1/s] (old model and library example: 18)
a.ctrl.lim_up_align   = 0.05;
a.ctrl.lim_down_align = -0.05;

% acquisitions
a.ctrl.adc_q = 2^-11;                       % [pu] quantization step (12-bit converter on +/-1 pu)

% first harmonic tracker of the terminal voltage, positive sequence, PLL
a.ctrl.fht_omega   = w;
a.ctrl.fht_delta   = 0.05;                  % [-] damping of the resonant model
[a.ctrl.fht_l1, a.ctrl.fht_l2] = prv.fht_gains(ts, w, a.ctrl.fht_delta, [-1 -4]);   % poles [-1 -4]*omega
a.ctrl.fht_t_reset = 1e-3;                  % [s] the tracker is held at zero until this time
a.ctrl.seq_gain     = 0.5;                  % [-] gain of the positive sequence, with pll_err_sign = +1
a.ctrl.pll_err_sign = 1;
a.ctrl.u_mag_min    = 0.02;                 % [pu] lower limit of |u| in the normalization 1/|u|
a.ctrl.pll_p        = 1;                    % [pu of omega_nom per pu of error] (pll_p of the old modules)
a.ctrl.pll_i1       = 80;                   % [1/s]
a.ctrl.omega_nom    = w;

% double integrator observer on the PLL angle (frequency estimate for the log)
a.ctrl.obs_A = [1 ts; 0 1];
a.ctrl.obs_C = [1 0];
a.ctrl.obs_L = prv.observer_gains(a.ctrl.obs_A, exp(ts*[-1 -4]*w));

% DC-link voltage loop: reference step at udc_t_step, low-pass udc_tau, PI -> active current reference
a.ctrl.udc_t_step   = 1e-3;                 % [s]
a.ctrl.udc_norm_ref = 1;                    % [pu of udc_nom] 1070 V
a.ctrl.udc_tau      = 20e-3;                % [s]
a.ctrl.kp_udc       = 2;                    % [pu/pu] as the old modules (example: 0.85)
a.ctrl.ki_udc       = 35;                   % [1/s]
a.ctrl.ixi_lim      = 1.65;                 % [pu] limit of the active current reference
a.ctrl.ieta_ref     = 0;                    % [pu] reactive current reference

% current loop: dq vector PI on the converter current, output = modulation (1 = udc/sqrt(3))
a.ctrl.kp_i         = 0.75;                 % [pu/pu]
a.ctrl.ki_i         = 35;                   % [1/s]
a.ctrl.u_lim        = 1;                    % [pu] radius of the voltage reference
a.ctrl.clip_release = 0.001;                % [1/s] leak of the integrators while the limiter clips

% zero-sequence current loop (u_cm per module below)
a.ctrl.iz_ref      = 0;                     % [pu]
a.ctrl.kp_iz       = 0.5;                   % [pu/pu]
a.ctrl.ki_iz       = 18;                    % [1/s]
a.ctrl.iz_lim      = 1;                     % [pu] saturation of the PI
a.ctrl.iz_out_gain = 1;                     % [-] gain from the PI output to u_gamma
a.ctrl.u_cm        = 0;                     % [-] factor of u_gamma in the modulator (0: standard SVPWM)
clear ts w
%[text] ### The two AFE modules
%[text] The clock ratios are those of the old modules (time gains of global timing/time\_afe1 and time\_afe2): each block runs its carrier on t\_master\*k\_time and its alignment loop pulls it back to the master time. A zero-sequence current can circulate through both AFEs, both DC links and both inverters (about 1.5 mH in the loop). One PI must control it, in one AFE only: u\_cm = -1 in module 2, as u\_cm\_comp\_mod2 of the old model (u\_cm = +1 would be positive feedback with the duty convention of the block).
param.afe_follower = [a, a];
param.afe_follower(1).ctrl.k_time = 1.0002;     % [-] local clock of the AFE of module 1
param.afe_follower(2).ctrl.k_time = 1.0015;     % [-] local clock of the AFE of module 2
param.afe_follower(1).ctrl.u_cm = 0;            % module 1: standard SVPWM, its zero-sequence PI does not act
param.afe_follower(2).ctrl.u_cm = -1;           % module 2: the zero-sequence PI acts on the common-mode voltage

% read by the model outside the blocks
param.afe.hw = a.hw;                            % bases ubez, ibez for the scopes (scopes/Gain2, Gain3, ...)
param.afe.behav.time_gain_module_1 = 1;         % global timing/time_afe1 and time_afe2: 1, the clock ratio is
param.afe.behav.time_gain_module_2 = 1;         % now k_time of the blocks (not applied twice)
fprintf('AFE modules: L1 %.0f uH, Cf %.0f uF, Cdc %.1f mF, ubez %.1f V, ibez %.1f A, udc_nom %.0f V | k_time %.4f / %.4f, u_cm %g / %g\n', ... %[output:group:2686a478] %[output:400786c3]
    a.hw.L1p*1e6, a.hw.Cf*1e6, a.hw.Cdc*1e3, a.hw.ubez, a.hw.ibez, a.hw.udc_nom, ... %[output:400786c3]
    param.afe_follower(1).ctrl.k_time, param.afe_follower(2).ctrl.k_time, ... %[output:400786c3]
    param.afe_follower(1).ctrl.u_cm, param.afe_follower(2).ctrl.u_cm); %[output:group:2686a478] %[output:400786c3]
clear a
%%
%[text] ## Machine
%[text] Data of psm\_calculus of the library (WindGen): six three-phase systems on one rotor; Ld, Lq and Jm of the whole machine. private/psm\_machine\_param gives the values of one system and its bases: Rs 30.1 mOhm, Ld 6.24 mH, Lq 7.56 mH, ubez 365.8 V, ibez 486.0 A, torque base 143.1 kNm, 15.43 Hz at 17.8 rpm. The machine block of the model is two systems in parallel: it divides La, Lb, Rs by number\_of\_modules and multiplies Jm and the friction by it.
param.psm.machine_in.name = 'WindGen';
param.psm.machine_in.number_of_systems = 6;
param.psm.machine_in.pwr_nom = 1600e3;             % [W]
param.psm.machine_in.i_nom = 2062;                 % [A] rms, all systems
param.psm.machine_in.rpm_nom = 17.8;               % [rpm]
param.psm.machine_in.number_poles = 104;
param.psm.machine_in.eta = 96;                     % [%] (sets Rs)
param.psm.machine_in.Ld = 1.04e-3;                 % [H] whole machine
param.psm.machine_in.Lq = 1.26e-3;                 % [H]
param.psm.machine_in.Jm = 900e3/6;                 % [kg m^2]
param.psm.machine_in.h5_percent = 0;               % [%] 5th and 7th flux harmonics (the block uses literals 0)
param.psm.machine_in.h7_percent = 0;
param.psm.machine = prv.psm_machine_param(param.psm.machine_in); %[output:14e8855f]
param.psm.number_of_modules = 2;                   % systems in parallel in the machine block = converter modules
%%
%[text] ## Mechanical side
%[text] Energy production: the shaft turns at rpm\_sim (block fixed\_speed\_setting: step at 25 ms, first-order lag of 0.1 s). The torque reference of each inverter is the torque curve of the 1.5 MW generic generator (legacy library script n\_sys\_generic\_1M5W\_torque\_curve, total torque divided among the six systems), scaled by torque\_overload\_factor (second element of param.inv.ref).
param.load.rpm_base = [6 7 8 9 10 11 12 13 14 15 16 17 17.8 18.0 18.2];      % [rpm]
param.load.torque_base = 1e3/param.psm.machine.number_of_systems* ...
    [50 75 100 125 150 175 200 225 275 350 450 600 750 885 885];              % [Nm] per system
param.load.torque_overload_factor = 1;
param.load.rpm_sim = 17.8;                          % [rpm] speed of the shaft (fixed_speed_setting)

% speed control (p.ctrl.mode = 1, not used here): load inertia and torque step of motor_load_setting
param.load.external_load_inertia = 1;               % [kg m^2] (motor_load_setting/Inertia)
param.sim.load_step_time = param.sim.simlength;     % [s] (motor_load_setting/PS Step)
%%
%[text] ## Inverter modules (pmsm\_inverter\_dqctrl\_mod1)
%[text] L output filter of 230 uH (the inverter of the old modules) to the machine terminals; the capacitor of the block is switched off by its series resistance (Rcf = 1e5 Ohm). The bases of the measurements are those of one machine system, so each inverter controls the current of one system. The angle comes from the encoder (theta\_elec\_psm), as in the old model; the back-EMF observer (angle\_src = 1) and the EKF (2) also work, each inverter with its own current.
%[text] **Current PI: kp\_i = 1.0, ki\_i = 150 (not the example values).** The current that circulates between the two inverters sees only the two output inductors (0.03 pu per module, against 1.0 pu of machine + filter for the machine current). The example gains (250 Hz on the machine, kp\_i = 16.3) make this circulating mode unstable, and kp\_i must stay near 1: with 1.0 the circulating mode has its crossover at about 520 Hz with about 52 deg of phase margin and 7.6 dB of gain margin. With kp\_i near 1 the dq cross-coupling of the machine (omega\*L, about 1 pu at rated speed) is as large as the gain, and the feed-forward of the block takes it from the references, so it does not act on a current error: the machine current loop has a slow oscillating pole. A high ki\_i makes it faster: with 150 its time constant is 30 ms (110 ms with 0.8 / 20). The torque reference follows the curve with a slew of 2 pu/s, so the current error stays below 0.01 pu (section "Design figures").
mc = param.psm.machine;
nm = param.psm.number_of_modules;
v = struct();
%[text] ### Power stage
v.hw.Vth  = 2.5;                            % [V]
v.hw.Ron  = 1e-3;                           % [Ohm]
v.hw.Goff = 1e-6;                           % [1/Ohm]
v.hw.Cdc  = 3.6e-3;                         % [F] other half of the 7.2 mF DC link of the module
v.hw.Rdc  = 1e-3;                           % [Ohm]
v.hw.L1p  = 230e-6;                         % [H] output inductor (LFi of the old modules), positive sequence
v.hw.L10  = 230e-6;                         % [H] zero sequence
v.hw.R1   = 157*0.05*v.hw.L1p;              % [Ohm] 1.8 mOhm (library rule)
v.hw.Lls1 = 0;                              % [H]
v.hw.Cf   = 1e-6;                           % [F] capacitor branch switched off ...
v.hw.Rcf  = 1e5;                            % [Ohm] ... by its series resistance (L filter)
v.hw.L2p  = 0;                              % [H] no second inductor ...
v.hw.L20  = 0;
v.hw.R2   = 0;                              % [Ohm]
v.hw.Lls2 = 1e-6;                           % [H] ... only a small leakage to the machine (must be > 0)
v.hw.ubez = mc.ubez;                        % [V] 365.8 V: bases of the measurements = bases of one system
v.hw.ibez = mc.ibez;                        % [A] 486.0 A
v.hw.udc_nom = 1070;                        % [V] nominal DC-link voltage
%[text] ### Carrier and gates
v.pwm.tc            = param.glb_time.tc;    % [s] = step of the Simscape local solver
v.pwm.fpwm          = param.time.fpwm_inv;  % [Hz]
v.pwm.phase         = 0;                    % [rad] carriers of the two inverters in phase: no PWM ripple
                                            %       between them once the clocks are aligned
v.pwm.double_update = param.time.trgo_inv;
v.pwm.dead_time     = param.time.dead_time_inv;   % [s]
v.pwm.gate_on       = 15;                   % [V]
v.pwm.gate_off      = -10;                  % [V]
v.pwm.m_low         = 0.02;
v.pwm.m_up          = 0.98;
%[text] ### Control
v.ctrl.ts = param.glb_time.ts_inv;          % [s] control period, 125 us

% local clock and alignment to the master time (k_time per module below), ki_align as the AFEs
v.ctrl.k_time         = 1;
v.ctrl.kp_align       = 0.25;
v.ctrl.ki_align       = 400;                % [1/s] (old model and library example: 18)
v.ctrl.lim_up_align   = 0.05;
v.ctrl.lim_down_align = -0.05;

% acquisitions
v.ctrl.adc_q = 2^-11;                       % [pu]

% machine model of the control: one system, pu of v.hw.ubez, v.hw.ibez, omega_b. The machine block holds nm
% systems in parallel with one leakage Lmu, so one system sees Ld_m + nm*Lmu
v.ctrl.omega_b = mc.omega_bez;              % [rad/s] 96.93 rad/s
v.ctrl.rs      = mc.Rs_m/mc.Xbez;           % 0.0400
v.ctrl.ld      = (mc.Ld_m + nm*mc.Lmu)/mc.Lbez;   % 0.805
v.ctrl.lq      = (mc.Lq_m + nm*mc.Lmu)/mc.Lbez;   % 0.975
v.ctrl.psi_m   = mc.psi_m*mc.omega_bez/mc.ubez;   % 1 with the bases of the machine

% operation
v.ctrl.mode      = 2;                       % 0 torque reference, 1 speed control, 2 torque curve (generator)
v.ctrl.mtpa      = 1;                       % 1 MTPA, 0 id = 0
v.ctrl.angle_src = 2;                       % 0 encoder (as the old model), 1 back-EMF observer, 2 EKF
v.ctrl.omega_th  = 0;                       % [pu] encoder below this speed with angle_src 1 or 2 (0: never)
v.ctrl.t_enable  = 0.25;                    % [s] gates off before: DC links charged, shaft at about 90 % speed
v.ctrl.k_delay   = 1.5;                     % [periods] angle advance of the voltage (delay compensation)

% torque and current references
v.ctrl.torque_lim  = 1.5;                   % [pu] limit of the torque reference
v.ctrl.torque_slew = 2;                     % [pu/s] slew rate of the torque reference
v.ctrl.i_lim       = 1.6;                   % [pu] limit of the current vector
v.ctrl.id_max      = 0.35;                  % [pu] largest |id| of the MTPA

% speed control (mode 1 only): PI and filters of inv_psm
v.ctrl.kp_w     = 2.5;                      % [pu/pu]
v.ctrl.ki_w     = 18;                       % [1/s]
v.ctrl.w_ref_fc = 1;                        % [Hz] filter of the speed reference
v.ctrl.w_flt_fc = 10;                       % [Hz] filter of the estimated speed (speed PI and torque curve)

% torque curve (mode 2): speed in pu of the rated speed, torque in pu of the torque base of one system
v.ctrl.curve_w = param.load.rpm_base/mc.rpm_nom;
v.ctrl.curve_T = param.load.torque_base/mc.tau_bez;

% current loop: dq PI on the output current (see the text above for the gains)
v.ctrl.kp_i         = 1.0;                  % [pu/pu] pu of ubez per pu of ibez (example: 16.3)
v.ctrl.ki_i         = 150;                  % [1/s] (example: 3195)
v.ctrl.u_lim        = 1;                    % [pu of vdc/sqrt(3)] radius of the voltage (linear SVPWM)
v.ctrl.clip_release = 0.001;                % [1/s]
v.ctrl.ff           = 1;                    % 1: feed-forward of back-EMF and cross-coupling (from the references)
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
%[text] ### The two inverter modules
param.inv_mod = [v, v];
param.inv_mod(1).ctrl.k_time = 1.0005;      % [-] local clock of the inverter of module 1 (old time_inv1)
param.inv_mod(2).ctrl.k_time = 1.001;       % [-] local clock of the inverter of module 2 (old time_inv2)

% input ref of the two blocks: [enable; scale of the torque curve] (Constant blocks in inv_dqctrl_psm_mod1/2)
param.inv.ref = [1; param.load.torque_overload_factor];

% read by the model outside the blocks
param.inv.behav.time_gain_module_1 = 1;     % global timing/time_inv1 and time_inv2 (see the AFE modules)
param.inv.behav.time_gain_module_2 = 1;
fprintf('Inverter modules: L %.0f uH, Cdc %.1f mF, ubez %.1f V, ibez %.1f A | pu: rs %.4f, ld %.4f, lq %.4f, filter %.4f | kp_i %.2f, ki_i %.1f | k_time %.4f / %.4f\n', ... %[output:group:95b6b356] %[output:05f73f26]
    v.hw.L1p*1e6, v.hw.Cdc*1e3, v.hw.ubez, v.hw.ibez, v.ctrl.rs, v.ctrl.ld, v.ctrl.lq, ... %[output:05f73f26]
    (v.hw.L1p + v.hw.Lls1 + v.hw.L2p + v.hw.Lls2)/mc.Lbez, v.ctrl.kp_i, v.ctrl.ki_i, ... %[output:05f73f26]
    param.inv_mod(1).ctrl.k_time, param.inv_mod(2).ctrl.k_time); %[output:group:95b6b356] %[output:05f73f26]
clear v mc nm
%%
%[text] ## Scope filters
% second-order 50 Hz low-pass filter of scopes/psm/LTI System and LTI System1
tau_sof = 1/(2*pi*50);                      % [s]
s = tf('s');
param.ctrl.filters.inv.SOF_50Hz.fcut = 50;  % [Hz]
param.ctrl.filters.inv.SOF_50Hz.time_constant = tau_sof;
param.ctrl.filters.inv.SOF_50Hz.ss_c = ss(1/(tau_sof*s + 1)^2);
clear tau_sof s
%%
%[text] ## Checks of the parameters
%[text] The structs of the four blocks against the examples of the library (a missing field stops the compilation with a message about the inside of the block) and the timing of the blocks against the Simscape step.
issues = [prv.check_block_struct(param.afe_follower, grid_following_param_example(param.glb_time.tc), 'param.afe_follower'), ...
          prv.check_block_struct(param.inv_mod, pmsm_inverter_param_example(param.glb_time.tc), 'param.inv_mod')];
blk = [num2cell(param.afe_follower), num2cell(param.inv_mod)];
for k = 1:numel(blk)
    p = blk{k};
    n_tc = p.ctrl.ts/p.pwm.tc;
    t_ctrl = 1/(p.pwm.fpwm*(1 + p.pwm.double_update));
    if abs(p.pwm.tc - param.glb_time.tc) > 1e-12 || abs(n_tc - round(n_tc)) > 1e-6 || abs(p.ctrl.ts - t_ctrl) > 1e-12
        issues{end+1} = sprintf(['block struct %d: tc = %g s, ts = %g s; tc must be glb_time.tc (%g s), ts an integer ' ...
            'multiple of tc and equal to 1/(fpwm*(1 + double_update))'], k, p.pwm.tc, p.ctrl.ts, param.glb_time.tc); %#ok<SAGROW>
    end
end
if isempty(issues) %[output:group:57dc18ac]
    fprintf('%s init: the structs of the four blocks have the fields of the library examples, timing consistent\n', model); %[output:7070ccfc]
else
    warning('%s init: %d point(s) in the block structs:\n  %s', model, numel(issues), strjoin(issues, '\n  '));
end %[output:group:57dc18ac]
clear issues blk k p n_tc t_ctrl
%%
%[text] ## Design figures
%[text] Discrete-time loops with the timing of the blocks (sample of the previous trigger, output held for one period), for the gains above: inverter current loop (machine current and circulating current), AFE current loop with the LCL filter (two modules, one module, difference of the two AFEs), DC-link voltage loop, zero-sequence loop, operating point at rpm\_sim. Change a gain and run this section again.
if param.sim.design_figures %[output:group:5e49b468]
    param.design = prv.loop_design_check(param, param.sim.design_plots); %[output:36024d03]
end %[output:group:5e49b468]
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
inv_modes = arrayfun(@(x) x.ctrl.mode, param.inv_mod);
if all(inv_modes ~= 1)
    set_param([model '/fixed_speed_setting'], 'Commented', 'off');
    set_param([model '/motor_load_setting'], 'Commented', 'on');
else
    set_param([model '/fixed_speed_setting'], 'Commented', 'on');
    set_param([model '/motor_load_setting'], 'Commented', 'off');
end
clear inv_modes
%[text] ### Checks of the model
%[text] Every param.\* read by the active blocks must exist. Then the connections of the four library blocks (struct of their own module, master time, inverter ref and enc inputs, logged outputs) and the From blocks without an active Goto.
if param.sim.check_model %[output:group:0131d822]
    [missing_refs, model_refs] = prv.check_model_param_refs(model, param); %[output:6b83bb23]
    if isempty(missing_refs)
        fprintf('%s init: all %d param.* references of the active blocks resolve\n', model, numel(model_refs)); %[output:3ca92eda]
    else
        warning('%s init: %d param.* references of the active blocks do not exist in param:\n  %s', model, ...
            numel(missing_refs), strjoin(missing_refs', '\n  '));
    end
    try
        issues = prv.check_model_wiring(model); %[output:860d587c] %[output:3535db85]
        if isempty(issues)
            fprintf('%s init: connections of the four library blocks as expected\n', model);
        else
            warning('%s init: %d point(s) to fix in the model before running it (README, Model check):\n  %s', ... %[output:80009551]
                model, numel(issues), strjoin(issues, '\n  ')); %[output:80009551]
        end
    catch ME
        warning('%s init: the check of the connections stopped: %s', model, ME.message);
    end
    clear missing_refs model_refs issues
end %[output:group:0131d822]


%[text] ### 
%[text] ### CREATE\\\_POWER\\\_METER\\\_BUS
%[text] Simulink.Bus for output "m" of the MATLAB Function block three\_phase\_power\_meter\_iec
%[text] (Symbols pane: Type = Bus: PowerMeterIecBus). Bus objects stay in the base workspace.
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

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":33.8}
%---
%[output:158291a7]
%   data: {"dataType":"text","outputData":{"text":"library_setup: library already on the path (C:\\github\\pwr-control\\library)\n","truncated":false}}
%---
%[output:1243faa5]
%   data: {"dataType":"text","outputData":{"text":"afe_inv_psm_n init: functions of .\/private, called as prv.<name>: check_block_struct, check_model_param_refs, check_model_wiring, fht_gains, grid_emulator_param, grid_fault_param, library_setup, loop_design_check, observer_gains, psm_machine_param, timing_param\n","truncated":false}}
%---
%[output:36073821]
%   data: {"dataType":"text","outputData":{"text":"Timing: AFE fPWM = 4000 Hz, double update = 1, ts = 125.0 us, dead time = 3.0 us | INV fPWM = 4000 Hz, double update = 1, ts = 125.0 us, dead time = 3.0 us\n        tc = 0.6250 us, logging window 1.250 s (Nc = 2000000, Ns_afe = 10000, Ns_inv = 10000)\n","truncated":false}}
%---
%[output:818fe05d]
%   data: {"dataType":"text","outputData":{"text":"Transformer Dyn11_690V_690V_1000kVA: Ld1 = 113.7 uH, Rd1 = 35.7 mOhm, Ld2 = 37.9 uH, Rd2 = 11.9 mOhm, Lm1 = 144.8 mH (i1m = 26.3 A), Rfe1 = 793 Ohm, n1\/n2 = 24\/14\nGrid emulator: grid_690V_1000kVA_Dyn11, 50 Hz, ugrid_bez = 563.4 V, igrid_bez = 381.8 A, L_eq = 76 uH (ucc 5 %, the transformer leakage is in the model)\nFault test: two_phase_D, enable_frt_1 = 0 (amplitudes [0.250 0.875 0.875], phase shift -0.380 rad), enable_frt_2 = 0 (positive sequence 0.00+0.40j, negative sequence -0.40-0.40j during the fault), from 0.750 s to 1.665 s\n---------------------------\n","truncated":false}}
%---
%[output:400786c3]
%   data: {"dataType":"text","outputData":{"text":"AFE modules: L1 500 uH, Cf 200 uF, Cdc 3.6 mF, ubez 563.4 V, ibez 328.7 A, udc_nom 1070 V | k_time 1.0002 \/ 1.0015, u_cm 0 \/ -1\n","truncated":false}}
%---
%[output:14e8855f]
%   data: {"dataType":"text","outputData":{"text":"PSM: WindGen, 6 systems, 1600 kW, 17.8 rpm, 104 poles | per system: ubez = 365.8 V, ibez = 486.0 A, tau_bez = 143061 Nm, psi_m = 3.774 Vs, f = 15.43 Hz\n     per system: Rs = 30.10 mOhm, Ld = 6.24 mH, Lq = 7.56 mH | pu: Rs = 0.0400, Ld = 0.8036, Lq = 0.9736, Ls = 0.8886\n","truncated":false}}
%---
%[output:05f73f26]
%   data: {"dataType":"text","outputData":{"text":"Inverter modules: L 230 uH, Cdc 3.6 mF, ubez 365.8 V, ibez 486.0 A | pu: rs 0.0400, ld 0.8049, lq 0.9749, filter 0.0298 | kp_i 1.00, ki_i 150.0 | k_time 1.0005 \/ 1.0010\n","truncated":false}}
%---
%[output:7070ccfc]
%   data: {"dataType":"text","outputData":{"text":"afe_inv_psm_n init: the structs of the four blocks have the fields of the library examples, timing consistent\n","truncated":false}}
%---
%[output:36024d03]
%   data: {"dataType":"text","outputData":{"text":"1. Inverter current loop: kp_i = 1, ki_i = 150 1\/s, ts = 125.0 us, filter 231 uH = 0.0298 pu\n   mode                           l\/pu    fc\/Hz   PM\/deg    GM\/dB    |z|max   kp limit\n   sum (machine), q axis        1.0047     22.5     43.7     38.2    0.9937       > 20\n   sum (machine), d axis        0.8347     25.5     47.1     36.6    0.9924       > 20\n   difference (circulating)     0.0298    527.6     52.0      7.6    0.9808       2.45\n   The difference mode sees only the filter of one module: it sets the limit of kp_i (the gains of pmsm_inverter_param_example, 250 Hz on the machine, are unstable here).\n   machine current with the dq cross-coupling (w*L at 1.00 pu speed, not in the feed-forward of the error): slowest pole -33.5 +77.2j rad\/s (time constant 30 ms); step of the q current: 2 % settling 89 ms, overshoot 27 %, peak |id| 0.29 pu (the torque reference has a slew limit, so steps do not occur)\n\n2. AFE current loop: kp_i = 0.75, ki_i = 35 1\/s, L1 = 500 uH, Cf = 200 uF, Rcf = 50 mOhm, transformer leakage 76.6 uH at the secondary\n   mode                            Lg\/uH  fres\/Hz    fc\/Hz   PM\/deg    GM\/dB         LCL mode  kp limit\n   two modules (sum)               154.1     1037    333.1     68.2      7.1    1122 Hz   6.9 %      1.72\n   one module alone                 77.6     1373    386.4     63.8      6.7    1418 Hz   4.7 %      1.63\n   difference of the two AFEs        1.0    11265    451.4     58.8      9.0    4722 Hz  84.4 %      2.13\n   A loop on the converter current with 1.5 periods of delay damps the LCL resonance only below fs\/6 = 1333 Hz: the two-module resonance is above it, its damping comes from Rcf and R1.\n\n3. DC-link voltage loop: C = 3.6 + 3.6 mF, 1 pu = 277.8 kW, plant 34.0 1\/s, kp_udc = 2, ki_udc = 35 1\/s: crossover 11.2 Hz, phase margin 74.8 deg\n\n4. Zero-sequence current loop: loop inductance 1464 uH, u_cm of the AFEs = [0 -1]\n   module 2, zero-vector time 0.1: plant 111 1\/s, crossover 10.2 Hz, phase margin 60.0 deg\n   module 2, zero-vector time 0.2: plant 222 1\/s, crossover 18.6 Hz, phase margin 71.6 deg\n   module 2, zero-vector time 0.4: plant 445 1\/s, crossover 35.9 Hz, phase margin 78.5 deg\n\n5. Operating point at 17.8 rpm: torque 125 kNm per module (0.874 pu), 233 kW per module | AFE 273 A peak (0.83 pu), modulation 0.924 | inverter id -0.122, iq -0.856 pu, |u| 445 V peak of 618 V (udc\/sqrt(3))\n","truncated":false}}
%---
%[output:6b83bb23]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Using find_system without the 'Variants' argument skips inactive Variant Subsystem blocks in the search. This behavior will change in a future release to look at all choices of the Variant Subsystem. To find blocks that are active in simulation or code generation, compile the model and use the built-in variant filters with the 'MatchFilter' option. To find all blocks use filter '@Simulink.match.allVariants' with the 'MatchFilter' option."}}
%---
%[output:3ca92eda]
%   data: {"dataType":"text","outputData":{"text":"afe_inv_psm_n init: all 60 param.* references of the active blocks resolve\n","truncated":false}}
%---
%[output:860d587c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Using find_system without the 'Variants' argument skips inactive Variant Subsystem blocks in the search. This behavior will change in a future release to look at all choices of the Variant Subsystem. To find blocks that are active in simulation or code generation, compile the model and use the built-in variant filters with the 'MatchFilter' option. To find all blocks use filter '@Simulink.match.allVariants' with the 'MatchFilter' option."}}
%---
%[output:3535db85]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Using find_system without the 'Variants' argument skips inactive Variant Subsystem blocks in the search. This behavior will change in a future release to look at all choices of the Variant Subsystem. To find blocks that are active in simulation or code generation, compile the model and use the built-in variant filters with the 'MatchFilter' option. To find all blocks use filter '@Simulink.match.allVariants' with the 'MatchFilter' option."}}
%---
%[output:80009551]
%   data: {"dataType":"warning","outputData":{"text":"Warning: afe_inv_psm_n init: 6 point(s) to fix in the model before running it (README, Model check):\n  afe_dqctrl_mod1\/grid_following_dqctrl_mod1: t_master comes from the tag time_afe_module_1, use time_master (the block makes its own local clock with p.ctrl.k_time)\n  afe_dqctrl_mod2\/grid_following_dqctrl_mod2: t_master comes from the tag time_afe_module_2, use time_master (the block makes its own local clock with p.ctrl.k_time)\n  inv_dqctrl_psm_mod1\/pmsm_inverter_dqctrl_mod1: t_master comes from the tag time_inv_module_1, use time_master (the block makes its own local clock with p.ctrl.k_time)\n  inv_dqctrl_psm_mod2\/pmsm_inverter_dqctrl_mod2: t_master comes from the tag time_inv_module_2, use time_master (the block makes its own local clock with p.ctrl.k_time)\n  outputs m and ctrl_log not connected in afe_dqctrl_mod1\/grid_following_dqctrl_mod1, afe_dqctrl_mod2\/grid_following_dqctrl_mod2, inv_dqctrl_psm_mod1\/pmsm_inverter_dqctrl_mod1, inv_dqctrl_psm_mod2\/pmsm_inverter_dqctrl_mod2: nothing of these blocks is logged (indices: pmsm_inverter_log_index.m of the library, Help of the blocks)\n  38 From blocks with 25 tags have no active Goto (the Gotos are in the commented modules afe_abc_inv_psm_mod1\/2): all_voltages_about_dclink (scopes\/module_1); ic_dclink_mod1 (afe_measurement_sim, scopes); ic_dclink_mod2 (scopes); ig_abc_pu_mod1 (afe_measurement_sim, scopes\/module_1); inverter_voltage_ee_phase_u_mod1 (inverter_measurement_sim); inverter_voltage_ee_phase_v_mod1 (inverter_measurement_sim); inverter_voltage_ee_phase_w_mod1 (inverter_measurement_sim); is_abc_pu_mod1 (afe_measurement_sim, scopes\/module_1); is_abc_pu_mod2 (scopes); is_uvw_pu_mod1 (inverter_measurement_sim\/module_1, scopes); is_uvw_pu_mod2 (inverter_measurement_sim\/module_1, scopes); iz_afe_pu_mod1 (scopes\/module_1); iz_afe_pu_mod2 (scopes); local_time_corr_afe1 (global timing); local_time_corr_afe2 (global timing); local_time_corr_inv1 (global timing); local_time_corr_inv2 (global timing); udc_mod1 (afe_measurement_sim, scopes); udc_mod2 (scopes); ug_abc_pu_mod1 (afe_measurement_sim, scopes\/module_1); us_uvw_pu_mod1 (inverter_measurement_sim\/module_1, scopes); us_uvw_pu_mod2 (inverter_measurement_sim\/module_1, scopes); vdc_pu_afe_mod1 (scopes\/module_1); vs_abc_pu_mod1 (scopes\/module_1); vs_abc_pu_mod2 (scopes)"}}
%---
