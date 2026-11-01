%[text] # Wind generator with a three-level T-type inverter and predictive current control (mpc_ttype_psm)
%[text] Initialization of mpc_ttype_psm.slx. The machine is a 1.6 MW, 17.8 rpm, 104-pole permanent magnet
%[text] synchronous generator with six separate three-phase systems (WindGen). The model simulates ONE system (about
%[text] 267 kW): Simscape block permanent_magnet_synchronous_machine, shaft driven at a fixed speed by an ideal speed
%[text] source (fixed_speed_setting). The system is connected through a 230 uH three-phase inductor to a three-level
%[text] T-type inverter with ideal switches (library block ideal_switch_based_three_level_inverter_ttype) on a split
%[text] DC link (2 x 14.4 mF, neutral point at the center tap) across a 1200 V lithium-ion battery. The control runs at
%[text] 8 kHz (timer at 4 kHz with double update) in the per unit of the machine: torque reference from the torque
%[text] curve of the turbine (speed -> torque, generator), current reference on the q axis only (id = 0), and a
%[text] finite-set model predictive control (MATLAB Function mpc_ttype_gate) that tests the 27 states of the bridge
%[text] and writes the 12 gate commands directly (there is no modulator). Rotor angle, speed and magnet flux come from
%[text] a 6-state extended Kalman filter (EKF) on the alpha/beta currents and voltages (sensorless); the encoder
%[text] angle is also there. The gate commands go through a dead-time generator (interlock between Q1 and Q4 of each
%[text] leg). Variant: speed control instead of the torque curve (use_speed_control, with motor_load_setting).
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, inv_hw_param, psm_machine_param, psm_ctrl_param, battery_param, check_model_param_refs).
%[text] The library is needed only for the Simulink/Simscape blocks and the C-Caller source of the back-EMF
%[text] observer. Run this script from this folder (the functions of ./private are visible only to the files of
%[text] this folder).
%[text] The old flat init is init_model_legacy.m (kept until the merge). It no longer runs with the library:
%[text] lithium_ion_battery_setup now takes the charge capacity, and the battery block reads
%[text] param.battery.nominal_battery_voltage_1 from its mask.

clear;
model = 'mpc_ttype_psm';

% Simulink/Simscape library blocks (blocks, not parameters) and C-Caller sources
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
param = struct();

% local parameter functions of this model (./private)
local_functions = dir(fullfile(fileparts(mfilename('fullpath')), 'private', '*.m'));
fprintf('%s init: local parameter functions in private/: %s\n', model, strjoin({local_functions.name}, ', '));
clear local_functions library_path
%[text] ## Simulation
param.sim.simlength = 2.25;                 % [s] model StopTime
param.sim.nonlinear_iterations = 20;        % Simscape Solver Configuration (block battery/Solver Configuration).
                                            % The legacy model had the literal 5 there (the legacy variable, 3, was
                                            % read by no block). In a T-type leg two diodes can switch in the same
                                            % step: the library rule (semiconductors_bridges tests) is at least 10.
param.sim.load_step_time = 1.25;            % [s] step time of the load torque (motor_load_setting, speed control only)
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
% one clock domain (control trigger and gate generator): timer at 4 kHz with double update -> ts = 125 us
param.time.fpwm = 4e3;                      % [Hz] frequency of the control timer (inv_psm_mod1/inverter/timer). There
                                            %      is no PWM: the predictive control writes the gate commands
param.time.trgo = 1;                        % double update: control at 2*fpwm = 8 kHz
param.time.dead_time = 3e-6;                % [s] dead time of the gate generator (interlock Q1-Q4 of each leg)
param.time.t_measure = param.sim.simlength; % [s] logging window: the whole simulation (legacy)
param.time.tc_factor = 200;                 % ts/tc: tc = 0.625 us (logging, MaxStep, Simscape local solver, gate filters)
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;

% sample time of the battery state-of-charge estimator (legacy ts_dab of the shared template: 3*fpwm, double update)
param.time.fpwm_dab = 3*param.time.fpwm;    % [Hz]
param.time.trgo_dab = 1;

param.glb_time = timing_param(param.time);
%[text] ## Inverter
%[text] ### Hardware
% 250 kW data set of the library for the 690 V class (inv690V_250kW: udc_nom 1070 V), output inductor 230 uH
% (block inverter_output_filter/threephase inductor); split DC link 2 x 14.4 mF of the AFE data set of the same
% class (block dclink/dclink: the DC voltages go to the control in pu of udc_nom, each capacitor starts at udc_nom/2)
param.inv.application_voltage = 690;        % [V] voltage class of the data sets
param.inv.pwr_nom = 250e3;                  % [W]
param.inv.hw = inv_hw_param(param.inv.application_voltage, param.inv.pwr_nom, param.time.fpwm);
%[text] ### Clocks
% local clock of the inverter: time = k*t (block global timing/time_inv1). The block reads the gain of module 2
% (1.001, legacy time_gain_inv_module_2), so the control timer, and with it the whole control, runs at 4004 Hz
% (8008 Hz trigger) while the control constants use ts = 125 us. The correction of phase_compensation_1
% (tag local_time_corr_inv1) goes into this clock, but its PI has kp = ki = 0 typed in the mask (see the README).
param.inv.behav.time_gain_module_2 = 1.001; % read by global timing/time_inv1
% clock of an AFE that this model does not have (block global timing/time_afe1, output not used)
param.afe.behav.time_gain_module_1 = 1.0002;
%[text] ### T-type bridge and gate generator (values typed in the blocks)
% bridge: twelve ideal switches (threshold 5.5 V, Ron 1.5 mOhm) with diodes (2 V, 0.15 mOhm) and RC snubbers
% (10 nF, 2.2 kOhm); per leg Q1 (P), Q2 and Q3 (bidirectional switch to the neutral point), Q4 (N). The
% predictive control gives P = Q1 + Q2, O = Q2 + Q3, N = Q3 + Q4, gate levels 0/15 V.
% gate generator (deadtime_generator_for_ttype): first-order filter with time constant glb_time.tc on the four
% gates, then a CMOS interlock (15 V logic) between Q1 and Q4 only, with the dead time glb_time.dead_time_INV.
% Q2 and Q3 go to the bridge through the filter only (see the README: Q1-Q3 and Q2-Q4 are not interlocked).
%[text] ## Battery across the DC link (block battery/lithium_ion_battery)
% lithium-ion pack across the whole split DC link (P to N): 1200 V nominal (legacy: udc1_bez of the 1200 V DAB
% data set of the library, hwdata.dab.udc1_bez), 250 kW, initial state of charge 0.85. As in the legacy init, R0
% and R1 are halved and C1 is divided by 50. The library block reads battery_voltage_init_m =
% param.battery.nominal_battery_voltage_1 from its mask default.
param.battery.nominal_battery_voltage_1 = 1200;                 % [V]
param.battery.nominal_battery_power = 250e3;                    % [W]
param.battery.initial_battery_soc = 0.85;
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;   % [Ah] ASSUMPTION: 1C rating
param.battery.plot_ocv = false;
b = param.battery;
param.battery.pack_1 = battery_param(b.nominal_battery_voltage_1, b.nominal_battery_power, b.charge_capacity_1, ...
    b.initial_battery_soc, param.glb_time.ts_dab, b.plot_ocv);
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;            % legacy changes
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/50;
fprintf('Battery block (after the legacy changes): R0 = %.1f mOhm, R1 = %.1f mOhm, C1 = %.0f mF\n', ...
    param.battery.pack_1.R0*1e3, param.battery.pack_1.R1*1e3, param.battery.pack_1.C1*1e3);
clear b
%[text] ## Permanent magnet synchronous machine
% data of psm_calculus of the library (WindGen): six three-phase systems on one rotor; Ld, Lq, Jm of the whole
% machine. The per-system values and the bases are computed in private/psm_machine_param (pmsm_setup):
% per system Rs 30.1 mOhm, Ld 6.24 mH, Lq 7.56 mH, ubez 365.8 V, ibez 486.0 A, tau_bez 143.1 kNm, 15.43 Hz
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
param.psm.machine = psm_machine_param(param.psm.machine_in);

% voltage scales of the observers: control voltage (pu) -> machine voltage (pu of ubez)
param.psm.u_psm_scale = 2/3*param.inv.hw.udc_nom/param.psm.machine.ubez;                 % back-EMF observers (not active)
param.psm.u_psm_scale_ekf = sqrt(3)/2*2/3*param.inv.hw.udc_nom/param.psm.machine.ubez;   % EKF: udc_nom/sqrt(3)/ubez
%[text] ## Mechanical side: torque curve, fixed speed, speed reference
% energy production (default): the shaft turns at rpm_sim (block fixed_speed_setting: step at 0.025 s, 0.1 s lag);
% the torque reference is the torque curve of the 1.5 MW generic generator (legacy library script
% n_sys_generic_1M5W_torque_curve, total torque divided among the systems) times torque_overload_factor
param.load.rpm_base = [6 7 8 9 10 11 12 13 14 15 16 17 17.8 18.0 18.2];        % [rpm]
param.load.torque_base = 1e3/param.psm.machine.number_of_systems* ...
    [50 75 100 125 150 175 200 225 275 350 450 600 750 885 885];                % [Nm] per system
param.load.torque_overload_factor = 1;
param.load.rpm_sim = 17.8;                          % [rpm] speed of the shaft (fixed_speed_setting)

% drive (variant use_speed_control): speed reference step to omega_m_sim at time_start_motor_control, 1 Hz
% filter; the load is the inertia external_load_inertia plus a torque step (motor_load_setting)
param.load.omega_m_sim = param.psm.machine.omega_m_bez;     % [rad/s] speed reference
param.load.external_load_inertia = 1;                       % [kg m^2] (block motor_load_setting/Inertia)
%[text] ## Control
%[text] ### Mode of operation
param.psm.mode.use_torque_curve = 1;                                        % (variant) energy production
param.psm.mode.use_speed_control = 1 - param.psm.mode.use_torque_curve;     % (variant) drive
param.psm.mode.use_psm_encoder = 0;                                         % (variant) 1: encoder, 0: sensorless
param.psm.mode.time_start_motor_control = 0.05;                             % [s] step of the speed reference
%[text] ### Observer implementation
% back-EMF observer: exactly one of the three is active (Variant Source blocks of inner_ctrl/bemf_observer1). The
% predictive control always takes speed and flux from the EKF (its inputs come from the EKF block directly).
param.psm.impl.use_ekf_bemf_module_1 = 1;                                   % (variant) EKF
param.psm.impl.use_observer_from_simulink_module_1 = 0;                     % (variant) Simulink back-EMF observer
param.psm.impl.use_observer_from_ccaller_module_1 = 0;                      % (variant) C-Caller back-EMF observer
if param.psm.impl.use_ekf_bemf_module_1 + param.psm.impl.use_observer_from_simulink_module_1 + ...
        param.psm.impl.use_observer_from_ccaller_module_1 ~= 1
    error('%s init: exactly one back-EMF observer must be active', model);
end
%[text] ### Controllers and observers
% weights of the cost function of the predictive control (legacy: fields k_kalman, k_dc, k_sw of the EKF object
% of the library, values 1, 0, 0): only the current error counts, the neutral point and the switching are free
ctrl_in.k_kalman = 1;                               % weight of the current error
ctrl_in.k_dc = 0;                                   % weight of the neutral-point voltage
ctrl_in.k_sw = 0;                                   % weight of the change of the voltage vector
% speed PI and observers: library defaults of ctrl_pmsm_setup, EKF of the legacy init
ctrl_in.kp_w = 2.5;                                 % [pu/pu] speed PI
ctrl_in.ki_w = 18;                                  % [1/s]
ctrl_in.iq_lim = 1.4;                               % [pu] limit of the torque reference of the speed PI
ctrl_in.emf_fb_p = 0.2;                             % gains of the two non-EKF back-EMF observers
ctrl_in.emf_p = ctrl_in.emf_fb_p*4/10;
ctrl_in.omega_th = 0;                               % [pu] speed threshold of the encoder-sensorless switch
ctrl_in.jm_ekf = 1e6;                               % inertia of the EKF model: the speed is a free state
ctrl_in.q_ekf = 1/10;                               % EKF noise (ekf_pmsm_setup): Q = ts*q_ekf*diag(q_ekf_diag),
ctrl_in.q_ekf_diag = [param.glb_time.ts_inv^2 param.glb_time.ts_inv^2 1/2 1/2 1 1];   % R = ts*r_ekf*eye(2)
ctrl_in.r_ekf = 1;
param.psm.ctrl = psm_ctrl_param(param.glb_time.ts_inv, param.psm.machine.omega_bez, param.psm.machine.Jm_norm, ...
    param.psm.u_psm_scale, param.psm.machine, ctrl_in);
clear ctrl_in

% local time alignment to the master time (phase_compensation_1/alignment_ctrl, output local_time_corr_inv1 into
% global timing/time_inv1). The PI block of the model has kp = 0 and ki = 0 typed in its mask (these names follow
% in a comment), so the correction is zero.
param.ctrl.kp_align = 0.25;
param.ctrl.ki_align = 18;                           % [1/s]
param.ctrl.lim_up_align = 0.05;
param.ctrl.lim_down_align = -0.05;

% filters (were part of setup_global_filters of the library): first-order low-pass y(k) = g1*y(k-1) + g0*u(k)
% with g0 = 2*pi*fcut*ts, and the second-order filter of the scope data (continuous state space)
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
% per-unit chain: currents in pu of ibez of the machine (one system). The predictive control computes the
% voltage of each bridge state from vdc1 + vdc2 in pu of udc_nom (Vdc_scale of the DC link block) and puts it
% into a machine model in pu of ubez; the EKF gets that voltage times 3 times u_psm_scale_ekf (= udc_nom/sqrt(3)/ubez).
% Operating point: rated speed, torque curve times torque_overload_factor, id = 0 (the reference is on the q axis).
mc = param.psm.machine; hw = param.inv.hw; pk = param.battery.pack_1; ts = param.glb_time.ts_inv;
d.u_ocv = pk.number_of_cells*interp1(0:1e-3:1, pk.ocv_model, param.battery.initial_battery_soc);   % [V] battery, open circuit
d.vdc_pu_ocv = d.u_ocv/hw.udc_nom;                              % [pu] DC voltage seen by the control
d.u_phase_max_pu = d.u_ocv/sqrt(3)/mc.ubez;                     % [pu] peak phase voltage, linear range of the bridge
d.mpc_voltage_ratio = hw.udc_nom/mc.ubez;                       % true voltage / voltage used by the prediction
d.ekf_voltage_ratio = 3*param.psm.u_psm_scale_ekf*mc.ubez/hw.udc_nom;   % voltage given to the EKF / true voltage
d.ts_pu = ts*mc.omega_bez;                                      % prediction step in the pu time of the model
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
% neutral point: change of vdc1 - vdc2 (pu of udc_nom) in one step for 1 pu of neutral-point current, true value
% (capacitors CFi1 = CFi2: d(vdc1 - vdc2)/dt = inp/CFi1) and value written in the predictive control (1/1e-3*inp*ts_pu)
d.np_dv_true = mc.ibez*ts/hw.CFi1/hw.udc_nom;
d.np_dv_mpc = 1/1e-3*d.ts_pu;
% gate timing after a command step (filter time constant tc, interlock logic at 7.5 V, switch threshold 5.5 V)
tc = param.glb_time.tc; v_gate = 15; v_logic = v_gate/2; v_th = 5.5;   % values typed in the blocks
d.t_Q1Q4_on = tc*log(v_gate/(v_gate - v_logic));                % [s] Q1 or Q4 on (the other one off for long)
d.t_Q1Q4_off = tc*log(v_gate/v_logic);                          % [s] Q1 or Q4 off
d.t_Q2Q3_on = tc*log(v_gate/(v_gate - v_th));                   % [s] Q2 or Q3 on
d.t_Q2Q3_off = tc*log(v_gate/v_th);                             % [s] Q2 or Q3 off
param.psm.design = d;
fprintf('Per unit: battery %.0f V open circuit at soc %.2f = %.3f pu of udc_nom %.0f V | linear limit of the bridge %.2f pu of ubez\n', ...
    d.u_ocv, param.battery.initial_battery_soc, d.vdc_pu_ocv, hw.udc_nom, d.u_phase_max_pu);
fprintf('Predictive control: voltages in pu of udc_nom, machine model in pu of ubez: the true voltage is %.3f times the predicted one | EKF voltage = %.3f x true voltage\n', ...
    d.mpc_voltage_ratio, d.ekf_voltage_ratio);
fprintf('Rated speed: iq_ref = %.3f pu (id = 0) | %.0f kW per system, %.0f A rms, %.0f V peak (%.2f pu, m = %.2f), cos(phi) %.2f\n', ...
    d.iq_ref, d.P_gen/1e3, d.i_rms, d.u_inv, d.u_inv_pu, d.m_index, d.cosphi);
fprintf('Neutral point: 1 pu of current for one step moves vdc1 - vdc2 by %.4f pu (the predictive control assumes %.2f pu)\n', ...
    d.np_dv_true, d.np_dv_mpc);
fprintf('Gate timing: Q1/Q4 on %.3f us, off %.3f us | Q2/Q3 on %.3f us, off %.3f us (overlap Q1-Q3 and Q2-Q4: see the README)\n', ...
    d.t_Q1Q4_on*1e6, d.t_Q1Q4_off*1e6, d.t_Q2Q3_on*1e6, d.t_Q2Q3_off*1e6);
clear mc hw pk ts d iq we L_q R ud uq tc v_gate v_logic v_th
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;               % [pu] quantizer of the acquisitions (pu signals, 12 bit)
param.meas.adc12_quantization = param.meas.adc_quantization;
param.meas.adc16_quantization = 1/2^15;
param.meas.Imax_adc = 1049.835;                     % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;
param.meas.Umax_adc = 1500;                         % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%[text] ## Model setup
open_system(model);
Simulink.importExternalCTypes(model, 'Names', {'bemf_obsv_output_t'});
%[text] ### Scopes closed at start
open_scopes = find_system(model, 'BlockType', 'Scope');
for k = 1:numel(open_scopes)
    set_param(open_scopes{k}, 'Open', 'off');
end
clear open_scopes k
%[text] ### Mechanical setting
% the two mechanical settings are switched by commenting them in and out (as the legacy init)
if param.psm.mode.use_torque_curve
    set_param([model '/fixed_speed_setting'], 'Commented', 'off');
    set_param([model '/motor_load_setting'], 'Commented', 'on');
else
    set_param([model '/fixed_speed_setting'], 'Commented', 'on');
    set_param([model '/motor_load_setting'], 'Commented', 'off');
end
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
