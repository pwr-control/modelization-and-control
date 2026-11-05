%[text] # Wind generator with a two-level inverter and predictive current control (mpc\_psm)
%[text] Initialization of mpc\_psm.slx. The machine is a 1.6 MW, 17.8 rpm, 104-pole permanent magnet synchronous generator with six separate three-phase systems (WindGen). The model simulates ONE system (about 267 kW): Simscape block permanent\_magnet\_synchronous\_machine, shaft driven at a fixed speed by an ideal speed source (fixed\_speed\_setting). The system is connected through a 230 uH three-phase inductor to a two-level inverter with ideal switches (library block Three-Phase Bridge (ideal), ideal\_bridges.three\_phase\_bridge) on a DC link with center tap (2 x 14.4 mF) across a 1200 V lithium-ion battery.
%[text] The control runs at 15 kHz (timer at 15 kHz, single update) in the per unit of the machine: torque reference from the torque curve of the turbine (speed to torque, generator), current reference on the q axis only (id = 0), and a finite-set model predictive control (MATLAB Function mpc, block mp control/mpc\_pu\_d) that tests the 8 states of the bridge and writes the six gate commands directly (there is no modulator). Rotor angle and magnet flux come from a 6-state extended Kalman filter (EKF) on the alpha/beta currents and the voltage of the predictive control (sensorless); the encoder angle is also there. The gate generator (dead\_time\_generator\_cmos\_simulink) adds the dead time. Variant: speed control instead of the torque curve (use\_speed\_control, with motor\_load\_setting).
%[text] The predictive control, the EKF and the bridge block were fixed in November 2026 (voltage base, delay compensation, inductance of the prediction, voltage and A6 path of the EKF, built-in passives of the bridge): see the section Control fixes of the README.
%[text] All the data are in one struct param, one group per part of the model: sim, time, glb\_time, inv, afe (one clock gain), battery, psm, load, ctrl, meas. The model reads param.\* only; the README lists the old names. The functions of ./private are called through the handles of mpc\_psm\_private.m (prv.timing\_param, prv.inv\_hw\_param, ...): the Live Editor runs a live script from a copy outside this folder and does not see ./private. Run the whole script once (Run): a section run on its own needs prv and param in the workspace.
%[text] The old flat init is init\_model\_legacy.m (kept until the merge). It no longer runs with the library: grid\_fault\_generator is now a function, lithium\_ion\_battery\_setup takes the charge capacity, and it switches three inverter blocks that the model no longer has.
clear;
model = 'mpc_psm';
prv = mpc_psm_private();        % handles to the functions of ./private (the Live Editor does not see ./private)
prv.library_setup();            % pwr-control/library on the path (blocks, C-Caller sources), cloned when missing

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
fprintf('%s init: functions of ./private, called as prv.<name>: %s\n', model, strjoin(fieldnames(prv)', ', '));
%%
%[text] ## Simulation
param.sim.simlength = 1.25;                 % [s] model StopTime
param.sim.nonlinear_iterations = 5;         % Simscape Solver Configuration (block battery/Solver Configuration):
                                            % the literal 5 of the legacy model (the legacy variable, 3, was read
                                            % by no block). Two-level bridge with ideal switches: 5 is the library
                                            % rule (semiconductors_bridges tests).
param.sim.load_step_time = 1.25;            % [s] step time of the load torque (motor_load_setting, speed control only)
param.sim.check_model = true;               % last section: param.* references and From blocks without a Goto
%%
%[text] ## Timing
%[text] One clock domain (control trigger and gate generator): timer at 15 kHz, single update, so the control step is ts = 66.7 us. The legacy init set it as 6 times fpwm = 2.5 kHz. The step tc = ts/200 = 0.333 us is the logging base, the MaxStep of the solver, the step of the Simscape local solver and the time constant of the gate filter. ts\_dab (7.5 kHz with double update, 66.7 us) is the sample time of the state-of-charge estimator of the battery block (legacy name of the shared template).
param.time.fpwm = 15e3;                     % [Hz] frequency of the control timer (inv_psm_mod1/inverter/inverter/timer).
                                            %      There is no PWM: the predictive control writes the gate commands
param.time.trgo = 0;                        % single update: control at fpwm
param.time.dead_time = 3e-6;                % [s] dead time of the gate generator
param.time.t_measure = param.sim.simlength; % [s] logging window: the whole simulation (legacy)
param.time.tc_factor = 200;                 % ts/tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;
param.time.fpwm_dab = 7.5e3;                % [Hz] only for ts_dab (legacy: 3*fpwm with fpwm = 2.5 kHz)
param.time.trgo_dab = 1;
param.glb_time = prv.timing_param(param.time);
%%
%[text] ## Inverter
%[text] ### Hardware
%[text] 250 kW data set of the library for the 690 V class (inv690V\_250kW: udc\_nom 1070 V), output inductor 230 uH (block inverter\_output\_filter/threephase inductor). DC link with center tap 2 x 14.4 mF of the AFE data set of the same class (block dclink/dclink, library dclink\_with\_center\_tap): the DC voltage goes to the control in pu of udc\_nom.
param.inv.application_voltage = 690;        % [V] voltage class of the data sets
param.inv.pwr_nom = 250e3;                  % [W]
param.inv.hw = prv.inv_hw_param(param.inv.application_voltage, param.inv.pwr_nom, param.time.fpwm);
%[text] ### Clocks
%[text] Local clock of the inverter: time = k\*t (block global timing/time\_inv1). The block reads the gain of module 2 (1.001, legacy time\_gain\_inv\_module\_2), so the control timer, and with it the whole control, runs at 15015 Hz while the control constants use ts = 66.7 us. The correction of phase\_compensation\_1 (tag local\_time\_corr\_inv1) goes into this clock, but its PI has kp = ki = 0 typed in the mask (see the README).
param.inv.behav.time_gain_module_2 = 1.001; % read by global timing/time_inv1
% clock of an AFE that this model does not have (block global timing/time_afe1, output not used)
param.afe.behav.time_gain_module_1 = 1.0002;
%[text] ### Bridge and gate generator (values typed in the blocks)
%[text] Bridge: library block Three-Phase Bridge (ideal) (ideal\_bridges.three\_phase\_bridge): six ideal switches with antiparallel diodes, gate threshold 2.5 V, Ron 1 mOhm, Goff 1e-6 S; gate order \[a+; a-; b+; b-; c+; c-\], gate levels 0/15 V. Its built-in DC capacitor and AC inductors are off (cap\_dc, ind\_ac): the model has its own DC link and output inductor. Gate generator (inverter/dead\_time\_generator\_cmos\_simulink): first-order filter with time constant glb\_time.tc on the six gates, then per leg the dead time between the two switches: the switch that turns on waits until an RC copy (time constant sqrt(2)\*dead\_time\_INV) of the other gate is below 7.5 V, ln(2)\*sqrt(2)\*3 us = 2.94 us.
%%
%[text] ## Battery across the DC link (block battery/lithium\_ion\_battery)
%[text] Lithium-ion pack across the whole DC link: 1200 V nominal (legacy: udc1\_bez of the 1200 V DAB data set of the library, hwdata.dab.udc1\_bez), 250 kW, initial state of charge 0.85. As in the legacy init, R0 and R1 are halved and C1 is divided by 50. The library block reads battery\_voltage\_init\_m = param.battery.nominal\_battery\_voltage\_1 from its mask default.
param.battery.nominal_battery_voltage_1 = 1200;                 % [V]
param.battery.nominal_battery_power = 250e3;                    % [W]
param.battery.initial_battery_soc = 0.85;
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;   % [Ah] ASSUMPTION: 1C rating
param.battery.plot_ocv = false;
b = param.battery;
param.battery.pack_1 = prv.battery_param(b.nominal_battery_voltage_1, b.nominal_battery_power, b.charge_capacity_1, ...
    b.initial_battery_soc, param.glb_time.ts_dab, b.plot_ocv);
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;            % legacy changes
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/50;
fprintf('Battery block (after the legacy changes): R0 = %.1f mOhm, R1 = %.1f mOhm, C1 = %.0f mF\n', ...
    param.battery.pack_1.R0*1e3, param.battery.pack_1.R1*1e3, param.battery.pack_1.C1*1e3);
clear b
%%
%[text] ## Permanent magnet synchronous machine
%[text] Data of psm\_calculus of the library (WindGen): six three-phase systems on one rotor; Ld, Lq, Jm of the whole machine. The per-system values and the bases are computed in private/psm\_machine\_param (pmsm\_setup): per system Rs 30.1 mOhm, Ld 6.24 mH, Lq 7.56 mH, ubez 365.8 V, ibez 486.0 A, tau\_bez 143.1 kNm, 15.43 Hz.
param.psm.machine_in.name = 'WindGen';
param.psm.machine_in.number_of_systems = 6;
param.psm.machine_in.pwr_nom = 1600e3;             % [W]
param.psm.machine_in.i_nom = 2062;                 % [A] rms, all systems
param.psm.machine_in.rpm_nom = 17.8;               % [rpm]
param.psm.machine_in.number_poles = 104;
param.psm.machine_in.eta = 96;                     % [%] (sets Rs)
param.psm.machine_in.Ld = 1.04e-3;                 % [H]
param.psm.machine_in.Lq = 1.26e-3;                 % [H]
param.psm.machine_in.Jm = 900e3/6;                 % [kg m^2]
param.psm.machine_in.h5_percent = 0;               % [%] 5th and 7th flux harmonics (the block uses literals 0)
param.psm.machine_in.h7_percent = 0;
param.psm.machine = prv.psm_machine_param(param.psm.machine_in);

% voltage scales of the observers. The predictive control gives the observers the bridge voltage of the vector
% of two steps before (the one that made the measured current), already in pu of ubez: the scales are 1. The two
% non-EKF back-EMF observers multiply it by their udc input, which is the constant 1 (block inner_ctrl/Constant).
% Before November 2026: 3/2 times a vector in pu of udc_nom (vdc = 1) times sqrt(3)/2*2/3*udc_nom/ubez for the EKF
% (0.74 times the true voltage at the battery voltage, and one step too new) or times 2/3*udc_nom/ubez for the
% non-EKF observers.
param.psm.u_psm_scale = 1;                          % back-EMF observers (not active), read as param.psm.ctrl.u_psm_scale
param.psm.u_psm_scale_ekf = 1;                      % EKF
%%
%[text] ## Mechanical side: torque curve, fixed speed, speed reference
%[text] Energy production (default): the shaft turns at rpm\_sim (block fixed\_speed\_setting: step at 0.025 s, 0.1 s lag); the torque reference is the torque curve of the 1.5 MW generic generator (legacy library script n\_sys\_generic\_1M5W\_torque\_curve, total torque divided among the systems) times torque\_overload\_factor. Drive (variant use\_speed\_control): speed reference step to omega\_m\_sim at time\_start\_motor\_control, 1 Hz filter; the load is the inertia external\_load\_inertia plus a torque step (motor\_load\_setting).
param.load.rpm_base = [6 7 8 9 10 11 12 13 14 15 16 17 17.8 18.0 18.2];        % [rpm]
param.load.torque_base = 1e3/param.psm.machine.number_of_systems* ...
    [50 75 100 125 150 175 200 225 275 350 450 600 750 885 885];                % [Nm] per system
param.load.torque_overload_factor = 1;
param.load.rpm_sim = 17.8;                          % [rpm] speed of the shaft (fixed_speed_setting)
param.load.omega_m_sim = param.psm.machine.omega_m_bez;     % [rad/s] speed reference (speed control)
param.load.external_load_inertia = 1;                       % [kg m^2] (block motor_load_setting/Inertia)
%%
%[text] ## Control
%[text] ### Mode of operation
param.psm.mode.use_torque_curve = 1;                                        % (variant) energy production
param.psm.mode.use_speed_control = 1 - param.psm.mode.use_torque_curve;     % (variant) drive
param.psm.mode.use_psm_encoder = 0;                                         % (variant) 1: encoder, 0: sensorless
param.psm.mode.time_start_motor_control = 0.25;                             % [s] step of the speed reference
%[text] ### Observer implementation
%[text] Back-EMF observer: exactly one of the three is active (Variant Source blocks of inner\_ctrl/bemf\_observer1). The predictive control always takes speed and flux from the EKF (its inputs come from the EKF block directly).
param.psm.impl.use_ekf_bemf_module_1 = 1;                                   % (variant) EKF
param.psm.impl.use_observer_from_simulink_module_1 = 0;                     % (variant) Simulink back-EMF observer
param.psm.impl.use_observer_from_ccaller_module_1 = 0;                      % (variant) C-Caller back-EMF observer
if param.psm.impl.use_ekf_bemf_module_1 + param.psm.impl.use_observer_from_simulink_module_1 + ...
        param.psm.impl.use_observer_from_ccaller_module_1 ~= 1
    error('%s init: exactly one back-EMF observer must be active', model);
end
%[text] ### Controllers and observers
%[text] Weights of the cost function of the predictive control (legacy: fields k\_kalman, k\_sw of the EKF object of the library). The switching term counts the legs that change state: with 1e-5 it breaks the ties between states that give almost the same current (above all the two zero vectors \[0 0 0\] and \[1 1 1\]) and lowers the switching from about 3.5 to 2.8 kHz per device at rated power with the same current error (2.8 A rms); the angle noise of the EKF is also lower (std 0.35 deg instead of 3.0 deg). 1e-4 gives 1.7 kHz with a current error of 3.5 A rms, 1e-3 gives 0.4 kHz with 11.6 A rms (design check of the README). Before November 2026 k\_sw (0) weighted the squared voltage of the vector.
%[text] Speed PI and observers: library defaults of ctrl\_pmsm\_setup. EKF: the inductance and resistance of the predictive control (psm\_ctrl\_param), speed as a free state. The process noise q\_ekf is 1/100 (legacy 1/10): at 15 kHz the legacy value makes the EKF so fast that the angle jumps (std 14 deg) when the machine inductance is 10 % lower than the value of the control, and is lost at -15 %; with 1/100 it still works with -25 % (angle error 12 deg) and the angle noise at rated power is less than half (std 0.35 deg instead of 0.79 deg).
ctrl_in.k_kalman = 1;                               % weight of the current error
ctrl_in.k_sw = 1e-5;                                % weight of the number of legs that change state
ctrl_in.kp_w = 2.5;                                 % [pu/pu] speed PI
ctrl_in.ki_w = 18;                                  % [1/s]
ctrl_in.iq_lim = 1.4;                               % [pu] limit of the torque reference of the speed PI
ctrl_in.emf_fb_p = 0.2;                             % gains of the two non-EKF back-EMF observers
ctrl_in.emf_p = ctrl_in.emf_fb_p*4/10;
ctrl_in.omega_th = 0;                               % [pu] speed threshold of the encoder-sensorless switch
ctrl_in.jm_ekf = 1e6;                               % inertia of the EKF model: the speed is a free state
ctrl_in.q_ekf = 1/100;                              % EKF noise (ekf_pmsm_setup): Q = ts*q_ekf*diag(q_ekf_diag),
ctrl_in.q_ekf_diag = [param.glb_time.ts_inv^2 param.glb_time.ts_inv^2 1/2 1/2 1 1];   % R = ts*r_ekf*eye(2)
ctrl_in.r_ekf = 1;
param.psm.ctrl = prv.psm_ctrl_param(param.glb_time.ts_inv, param.psm.machine.omega_bez, param.psm.machine.Jm_norm, ...
    param.psm.u_psm_scale, param.psm.machine, param.inv.hw, ctrl_in);
clear ctrl_in
%[text] Local time alignment to the master time (phase\_compensation\_1/alignment\_ctrl, output local\_time\_corr\_inv1 into global timing/time\_inv1). The PI block of the model has kp = 0 and ki = 0 typed in its mask (these names follow in a comment), so the correction is zero.
param.ctrl.kp_align = 0.25;
param.ctrl.ki_align = 18;                           % [1/s]
param.ctrl.lim_up_align = 0.05;
param.ctrl.lim_down_align = -0.05;
%[text] Filters (were part of setup\_global\_filters of the library): first-order low-pass y(k) = g1\*y(k-1) + g0\*u(k) with g0 = 2\*pi\*fcut\*ts, and the second-order filter of the scope data (continuous state space).
ts = param.glb_time.ts_inv;
param.ctrl.filters.inv.g0_1Hz = 1*ts*2*pi;          % speed filter of the torque curve (omega_pu/flt)
param.ctrl.filters.inv.g1_1Hz = 1 - param.ctrl.filters.inv.g0_1Hz;
tau_sof = 1/(2*pi*50);                              % [s] 50 Hz second-order filter (scopes, LTI System blocks)
s = tf('s');
param.ctrl.filters.inv.SOF_50Hz.fcut = 50;          % [Hz]
param.ctrl.filters.inv.SOF_50Hz.time_constant = tau_sof;
param.ctrl.filters.inv.SOF_50Hz.ss_c = ss(1/(tau_sof*s + 1)^2);
clear ts tau_sof s
%[text] ### Design figures (informative)
%[text] Per-unit chain: currents in pu of ibez of the machine (one system). The predictive control computes the voltage of each bridge state from the measured DC voltage in pu of udc\_nom (Vdc\_scale of the DC link block) times u\_scale = udc\_nom/ubez, so in pu of ubez, the base of its machine model and of the observers. Operating point: rated speed, torque curve times torque\_overload\_factor, id = 0 (the reference is on the q axis).
mc = param.psm.machine; hw = param.inv.hw; pk = param.battery.pack_1; mpc = param.psm.ctrl.mpc;
d.u_ocv = pk.number_of_cells*interp1(0:1e-3:1, pk.ocv_model, param.battery.initial_battery_soc);   % [V] battery, open circuit
d.vdc_pu_ocv = d.u_ocv/hw.udc_nom;                              % [pu] DC voltage seen by the control
d.u_phase_max_pu = d.u_ocv/sqrt(3)/mc.ubez;                     % [pu] peak phase voltage, linear range of the bridge
% current loop of the predictive control seen as a deadbeat loop on one axis: one-step gain g = (true voltage /
% predicted voltage)*(L of the prediction / true L); with the delay compensation the error obeys
% e(k+1) = (1 - g)*e(k-1) (deadbeat for g = 1, stable for 0 < g < 2). The prediction uses Lq + LFi on both axes,
% so the d axis has g = (Lq + LFi)/(Ld + LFi).
d.g_q = mpc.L_norm*mc.Lbez/(mc.Lq_m + hw.LFi);
d.g_d = mpc.L_norm*mc.Lbez/(mc.Ld_m + hw.LFi);
% operating point at rated speed
d.T_curve = interp1(param.load.rpm_base, param.load.torque_base, mc.rpm_nom);    % [Nm] torque curve, per system
d.iq_ref = -param.load.torque_overload_factor*d.T_curve/mc.tau_bez;              % [pu] torque reference = iq reference
iq = d.iq_ref*mc.ibez; we = mc.omega_bez;
L_q = mc.Lq_m + hw.LFi; R = mc.Rs_m + hw.RLFi;                  % q axis inductance and resistance seen by the bridge
ud = -we*L_q*iq;                                                % [V] steady-state voltage at the bridge (id = 0)
uq = R*iq + we*mc.psi_m;
d.u_inv = hypot(ud, uq);                                        % [V] peak phase voltage
d.u_inv_pu = d.u_inv/mc.ubez;
d.m_index = d.u_inv/(d.u_ocv/2);                                % referred to vdc/2 (linear limit 2/sqrt(3))
d.cosphi = uq*iq/(d.u_inv*abs(iq));                             % < 0: power flows to the DC link
d.P_gen = -3/2*mc.pp*mc.psi_m*iq*mc.omega_m_bez;                % [W] per system (generated)
d.i_rms = abs(iq)/sqrt(2);                                      % [A] rms
% angle delay of the chain: the EKF angle used at step k is the one of t(k-2) (current measured one step late,
% output read from the state of the previous step) and the chosen vector acts until t(k+1): 3 steps, not
% compensated
d.angle_delay_deg = 3*param.glb_time.ts_inv*mc.omega_bez*180/pi;
% dead time seen by the bridge: the switch that turns on waits until the RC copy (sqrt(2)*dead_time) of the other
% gate has fallen below half of the 15 V rail
d.dead_time_bridge = log(2)*sqrt(2)*param.glb_time.dead_time_INV;
param.psm.design = d;
fprintf('Per unit: battery %.0f V open circuit at soc %.2f = %.3f pu of udc_nom %.0f V | linear limit of the bridge %.2f pu of ubez\n', ...
    d.u_ocv, param.battery.initial_battery_soc, d.vdc_pu_ocv, hw.udc_nom, d.u_phase_max_pu);
fprintf('Predictive control: state voltages times u_scale = %.4f (pu of ubez) | one-step gain g = %.3f (q axis, deadbeat), %.3f (d axis: the error times %.3f every two steps)\n', ...
    mpc.u_scale, d.g_q, d.g_d, 1 - d.g_d);
fprintf('Rated speed: iq_ref = %.3f pu (id = 0) | %.0f kW per system, %.0f A rms, %.0f V peak (%.2f pu, m = %.2f), cos(phi) %.2f\n', ...
    d.iq_ref, d.P_gen/1e3, d.i_rms, d.u_inv, d.u_inv_pu, d.m_index, d.cosphi);
fprintf('Angle delay of the chain %.2f deg at rated speed (not compensated) | dead time seen by the bridge %.2f us\n', ...
    d.angle_delay_deg, d.dead_time_bridge*1e6);
clear mc hw pk mpc d iq we L_q R ud uq
%%
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;               % [pu] quantizer of the acquisitions (pu signals, 12 bit)
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;
param.meas.Imax_adc = 1049.835;                     % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;
param.meas.Umax_adc = 1500;                         % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%%
%[text] ## Model setup
open_system(model);
Simulink.importExternalCTypes(model, 'Names', {'bemf_obsv_output_t'});    % output bus of the C-Caller observer
%[text] Scopes closed at start.
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] Mechanical setting: the two settings are switched by commenting them in and out (as the legacy init).
if param.psm.mode.use_torque_curve
    set_param([model '/fixed_speed_setting'], 'Commented', 'off');
    set_param([model '/motor_load_setting'], 'Commented', 'on');
else
    set_param([model '/fixed_speed_setting'], 'Commented', 'on');
    set_param([model '/motor_load_setting'], 'Commented', 'off');
end
%%
%[text] ## Check of the model
%[text] Every param.\* read by an active block must exist, and every From block of the active part of the model must have a Goto.
if param.sim.check_model
    [missing_refs, model_refs] = prv.check_model_param_refs(model, param);
    if isempty(missing_refs)
        fprintf('%s init: all %d param.* references of the model blocks resolve\n', model, numel(model_refs));
    else
        warning('%s init: %d param.* references of the model blocks do not exist in param:', model, numel(missing_refs));
        disp(missing_refs);
    end
    lost = prv.check_from_goto(model);
    if isempty(lost)
        fprintf('%s init: every active From block has its Goto\n', model);
    else
        warning('%s init: %d tags read by active From blocks have no active Goto:\n  %s', model, numel(lost), ...
            strjoin(lost, '\n  '));
    end
    clear missing_refs model_refs lost
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":33.8}
%---
