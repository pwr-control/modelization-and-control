%[text] # Induction machine drive with field-oriented control (im_analysis_and_design)
%[text] Initialization of im_gyro_inv_model.slx (default) and im_inv_model.slx: 261 kW, 460 V, 60 Hz, six-pole
%[text] squirrel-cage induction machine (ABB M3BP 355MLB 6, block poliphase_induction_machine of the library,
%[text] Simscape model im_uvw_2) fed by a three-phase two-level inverter (4 kHz PWM, space vector modulation
%[text] C-Caller, no dead time) through a 230 uH three-phase inductor; DC link 2 x 14.4 mF across a 1200 V
%[text] lithium-ion battery. In im_gyro_inv_model the inverter is the averaged switching-function model
%[text] (gyro_inverter: controlled voltage sources on the AC side, controlled current sources on the DC side), in
%[text] im_inv_model it is the bridge of six ideal switches. Control at 8 kHz (double update), in the per unit of
%[text] the machine: speed ramp from time_start_motor_control to synchronous speed in 1 s (second-order filter
%[text] 10 Hz), speed PI -> y current reference (torque), rotor flux PI -> x current reference (field weakening
%[text] above base speed as psi_r_ref/omega^2), rotor flux observer (current model in rotor coordinates with the
%[text] encoder angle), x/y current PI with voltage-vector limit -> alpha-beta -> SVM. Variant: open-loop u/f
%[text] instead of the vector control. The load is the viscous friction of the machine block (0.75 pu of torque
%[text] at synchronous speed); the load generator is commented out.
%[text] All parameters are local: the numbers are in this script and the parameter functions are in ./private
%[text] (timing_param, inv_hw_param, im_machine_param, im_ctrl_param, battery_param, device_ideal_switch_param,
%[text] check_model_param_refs). The library is needed for the Simulink/Simscape blocks (induction machine, DC
%[text] link, inductor, battery, modulator, C-Caller sv_pwm) and for the ideal-switch data sets other than the
%[text] embedded one. Run it from this folder (the functions of ./private are visible only to the files of this
%[text] folder). The legacy flat init (shared template, seven clock domains, grid emulator, DAB/PSFBC/CLLC, PMSM)
%[text] is kept as init_model_legacy.m for reference; it no longer runs with the library (lithium_ion_battery_setup
%[text] takes the charge capacity, grid_fault_generator is a function, the battery block reads
%[text] param.battery.nominal_battery_voltage_1). inv_im_ekf.slx is an older copy of im_inv_model without the
%[text] observer its name suggests and with names no init defines: it is not migrated (see the README).

clear;
model = 'im_gyro_inv_model';                % averaged inverter (gyro_inverter)
% model = 'im_inv_model';                   % switch-level inverter (ideal switches), same parameters

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
param.sim.simlength = 3.25;                 % [s] model StopTime
param.sim.tc = 1.25e-6;                     % [s] logging base, model MaxStep (ode23tb) and step of the Simscape
                                            %     local solver (glb_time.tc): ts/100 at fpwm = 4 kHz, double update
param.sim.use_thermal_model = 0;            % no loss/thermal model in these models (averaged inverter or ideal
                                            % switches): the flag only sets the Simscape nonlinear iterations
param.sim.nonlinear_iterations = 3;         % Simscape Solver Configuration (block inverter/ctrl/dc_source/Solver Configuration)
if param.sim.use_thermal_model
    param.sim.nonlinear_iterations = 5;
end
param.sim.check_param_refs = true;          % at the end: check that every param.* read by the model exists
%[text] ## Timing
% one converter clock domain (inverter and control): 4 kHz PWM, double update -> ts = 125 us
param.time.fpwm = 4e3;                              % [Hz] switching frequency of the inverter
param.time.trgo = 1;                                % double update: control at 2*fpwm = 8 kHz
param.time.dead_time = 0;                           % [s] legacy dead_time_inv, read by the PWM generators
param.time.t_measure = param.sim.simlength;         % [s] logging window
param.time.tc_factor = 1/param.time.fpwm/param.sim.tc/2;    % ts/tc = 100 (legacy tc_factor), glb_time.tc = param.sim.tc
param.time.tc_decimation = 1;
param.time.delay_pwm = 0;

% sample time of the battery state-of-charge estimator (legacy ts_dab of the shared template: 3*fpwm, double update)
param.time.fpwm_dab = 3*param.time.fpwm;            % [Hz]
param.time.trgo_dab = 1;

param.glb_time = timing_param(param.time);
%[text] ## Inverter
%[text] ### Hardware
% 250 kW data set of the library for the 690 V class (inv690V_250kW: 550 V, 370 A rms, ubez 449.1 V, ibez 523.3 A,
% udc_nom 1070 V), output inductor 230 uH (block LFi); DC link 2 x 14.4 mF of the AFE data set of the same class
% (block dclink, which also scales the logged udc_pu with udc_nom = 1070 V while the battery sets 1200 V)
param.inv.application_voltage = 690;                % [V] voltage class of the data sets
param.inv.pwr_nom = 250e3;                          % [W]
param.inv.hw = inv_hw_param(param.inv.application_voltage, param.inv.pwr_nom, param.time.fpwm);
%[text] ### Power semiconductors (switch-level inverter of im_inv_model)
% six ideal switches with antiparallel diode and RC snubber (switch_1..switch_6 of inverter/ctrl/inverter/inverter,
% commented out in im_gyro_inv_model); the data set is embedded in private/device_ideal_switch_param
param.dev.used_device_ideal_switch = 'silicon_high_power_ideal_switch';
param.inv.ideal_switch = device_ideal_switch_param(param.dev.used_device_ideal_switch, param.glb_time.fPWM_INV, param.inv.hw.udc_nom);
%[text] ## Battery on the DC link (block battery/lithium_ion_battery)
% lithium-ion pack across the whole DC link: 1200 V nominal (the legacy init took hwdata.dab.udc1_bez of the
% 1200 V DAB data set selected at 690 V), 250 kW, initial state of charge 0.85; the legacy init halved R0 and R1
% and divided C1 by 50 after the setup. The library block reads battery_voltage_init_m =
% param.battery.nominal_battery_voltage_1 from its mask default.
param.battery.nominal_battery_voltage_1 = 1200;                 % [V]
param.battery.nominal_battery_power = 250e3;                    % [W]
param.battery.initial_battery_soc = 0.85;
param.battery.charge_capacity_1 = param.battery.nominal_battery_power/param.battery.nominal_battery_voltage_1;   % [Ah] ASSUMPTION: 1C rating
param.battery.plot_ocv = false;
b = param.battery;
param.battery.pack_1 = battery_param(b.nominal_battery_voltage_1, b.nominal_battery_power, b.charge_capacity_1, ...
    b.initial_battery_soc, param.glb_time.ts_dab, b.plot_ocv);
param.battery.pack_1.R0 = param.battery.pack_1.R0/2;            % legacy modifications of the model
param.battery.pack_1.R1 = param.battery.pack_1.R1/2;
param.battery.pack_1.C1 = param.battery.pack_1.C1/50;
clear b
%[text] ## Induction machine
% nameplate, no-load test and locked-rotor test of the machine (legacy im_calculus of the library); the equivalent
% circuit, the rotor resistance and the per-unit bases are computed in private/im_machine_param (im_setup):
% Ls 3.914 mH, Lm 3.763 mH, Lr 3.914 mH, sigma*Ls 0.295 mH, Rr 2.74 mOhm (rotor time constant 1.43 s),
% ubez 375.6 V, ibez 581.2 A, omega_bez 2*pi*60 rad/s, psi_bez 0.996 Vs, torque_nom 2186 Nm
param.im.machine_in.name = 'ABB M3BP 355MLB 6 261kW';
param.im.machine_in.pwr_nom = 261e3;                % [W]
param.im.machine_in.u_nom = 460;                    % [V] rms line-to-line
param.im.machine_in.i_nom = 411;                    % [A] rms
param.im.machine_in.freq_nom = 60;                  % [Hz]
param.im.machine_in.rpm_load = 1195;                % [rpm] at rated load
param.im.machine_in.number_poles = 6;
param.im.machine_in.eta = 0.95;                     % [pu]
param.im.machine_in.cosphi = 0.88;
param.im.machine_in.Rs = 2e-3;                      % [Ohm] stator phase resistance
param.im.machine_in.u_no_load = param.im.machine_in.u_nom;      % [V] no-load test
param.im.machine_in.i_no_load = 180;                % [A]
param.im.machine_in.f_no_load = param.im.machine_in.freq_nom;   % [Hz]
param.im.machine_in.u_cc = 77;                      % [V] locked-rotor test
param.im.machine_in.i_cc = 470;                     % [A]
param.im.machine_in.f_cc = 50;                      % [Hz]
param.im.machine_in.Jm = 5;                         % [kg m^2] machine + load inertia
param.im.machine_in.load_friction_factor = 0.75;    % [pu] viscous friction of the block: 0.75*torque_nom at omega_m_bez
param.im.machine = im_machine_param(param.im.machine_in);
%[text] ### Load generator (commented out in the models)
% the block induction motor/load generator is commented out (the machine block sees only its viscous friction):
% if it is enabled, a torque step Tload at 1 s (default time of its Step block) is applied; the ramp RampSlow
% (slope Tload/timeRamp/simlength*2 from start_load_ramp) is not connected. The legacy init did not define these
% names: the values below are placeholders.
param.im.load.Tload = 0.25*param.im.machine.torque_nom;        % [Nm] placeholder
param.im.load.timeRamp = 1;                                     % [s] placeholder
param.im.load.start_load_ramp = 1.5;                            % [s] placeholder
%[text] ## Control
%[text] ### Mode of operation
% vector control (speed, flux and current loops) or open-loop u/f (fixed frequency, voltage steps): the three
% Variant Source blocks of inverter/ctrl/im_ctrl/im_control pick usx, usy and the angle of one of them
param.im.mode.use_vector_control = 1;               % (variant)
param.im.mode.use_uf_control = 0;                   % (variant)
if param.im.mode.use_vector_control + param.im.mode.use_uf_control ~= 1
    error('%s init: exactly one of use_vector_control and use_uf_control must be 1', model);
end
param.im.mode.time_start_motor_control = 0.25;      % [s] start of the speed ramp (block speed reference/wref);
                                                    %     the flux loop and the modulator run from t = 0
%[text] ### Vector control
% gains of the legacy init (library defaults of ctrl_im_setup) and the literals of the flux PI block; the current
% PI output is in pu of udc/sqrt(3) (see the design figures), the currents in pu of ibez of the machine
ctrl_in.kp_w = 2.5;                                 % [pu/pu] speed PI (pu of omega_m_bez in, pu of isy out)
ctrl_in.ki_w = 18;                                  % [1/s]
ctrl_in.iq_lim = 1.4;                               % [pu] limit of the y current (torque) reference
ctrl_in.id_lim = 0.35;                              % [pu] field of ctrl_im_setup, read by no block
ctrl_in.kp_i = 0.25;                                % [pu/pu] x and y current PI
ctrl_in.ki_i = 18;                                  % [1/s]
ctrl_in.kp_psi = 2;                                 % [pu/pu] rotor flux PI (legacy literal of the block)
ctrl_in.ki_psi = 20;                                % [1/s] (legacy literal 20*glb_time.ts_inv)
ctrl_in.isx_lim_min = 0.05;                         % [pu] limits of the x current reference (legacy literals
ctrl_in.isx_lim_max = 0.6;                          %      0.05*im.Lm_norm and 0.6*im.Lm_norm on Lm_norm*isx)
ctrl_in.psi_r_ref = 1;                              % [pu] rotor flux reference (Constant psi_r_ref, default 1)
ctrl_in.fcut_speed_ref = 10;                        % [Hz] second-order filter of the speed reference
param.im.ctrl = im_ctrl_param(param.glb_time.ts_inv, param.im.machine.omega_bez, param.im.machine.Jm_norm, ctrl_in);
clear ctrl_in

% decay rate of the current PI integrators while the voltage vector is clipped (pi-ctrl-x, pi-ctrl-y)
param.ctrl.CTRPIFF_CLIP_RELEASE = 0.001;            % [1/s]
%[text] ### Open-loop u/f (variant use_uf_control)
% fixed stator frequency uf_omega_ref*Omega_s_bez, voltage on the y axis: step to uq_ref/5 at 25 ms and to uq_ref
% at 12 s (after the end of the simulation), rate limited at uq_ref/5 per second. The legacy init did not define
% these names (the variant was never active); uq_ref is set to the rated phase voltage of the machine in the pu of
% the control (udc/sqrt(3), see the design figures)
param.im.uf.omega_ref = 1;                          % [pu] of Omega_s_bez
param.im.uf.Omega_s_bez = param.im.machine.omega_bez;   % [rad/s]
param.im.uf.uq_ref = param.im.machine.ubez/(param.battery.nominal_battery_voltage_1/sqrt(3));   % [pu] 0.54
%[text] ### Design figures (informative)
% per-unit chain: the currents are in pu of ibez of the machine (acquisition gain 1/ibez), the current PI output
% in pu of udc/sqrt(3): the SVM C-Caller scales its input by sqrt(3)/2 and its udc input is the Constant 1 (no
% udc feed-forward), so 1 pu of control voltage is 693 V at 1200 V, 1.84 times ubez (u_im_scale of the legacy init
% was computed but read by no block). Current loop plant: transient inductance sigma*Ls + LFi, resistance
% Rs + Rr*(Lm/Lr)^2 + RLFi, no decoupling of the cross terms and of the back-EMF. Flux: rotor time constant
% Lr/Rr; the flux PI starts at t = 0 with isx at its upper limit. Speed: torque per pu of isy at psi_r_ref, plant
% J*omega_m_bez, viscous load. Steady state at synchronous speed with the friction load.
mc = param.im.machine; hw = param.inv.hw; c = param.im.ctrl; ts = param.glb_time.ts_inv;
d.udc = param.battery.nominal_battery_voltage_1;                % [V]
d.u_ctrl_bez = d.udc/sqrt(3);                                   % [V] 1 pu of the current PI output (SVM linear limit)
d.L_tr = mc.sigma + hw.LFi;                                     % [H] transient inductance seen by the current loop
d.R_tr = mc.Rs + mc.Rr*(mc.Lm/mc.Lr)^2 + hw.RLFi;               % [Ohm]
d.tau_r = mc.Lr/mc.Rr;                                          % [s] rotor time constant
d.isx_ref = c.psi_r_ref/mc.Lm_norm;                             % [pu] magnetizing current at psi_r_ref
d.T_isy = 3/2*mc.pp*mc.Lm/mc.Lr*c.psi_r_ref*mc.psi_bez*mc.ibez; % [Nm] torque per pu of isy at psi_r_ref
d.k_w = d.T_isy/(mc.Jm*mc.omega_m_bez);                         % [1/s] plant gain of the speed loop (pu/pu)
% open loops (frequency responses, w in rad/s): current PI + R-L plant + 1.5 samples of delay; flux PI + rotor
% flux lag (isx -> psi_r); speed PI + inertia with the viscous friction
L_i = @(w) (c.kp_i + c.ki_i./(1i*w)).*(d.u_ctrl_bez/mc.ibez)./(d.R_tr + 1i*w*d.L_tr).*exp(-1i*w*1.5*ts);
L_psi = @(w) (c.kp_psi + c.ki_psi./(1i*w))./(1 + 1i*w*d.tau_r);
L_w = @(w) (c.kp_w + c.ki_w./(1i*w)).*d.k_w./(1i*w + mc.load_friction/mc.Jm);
d.wc_i = fzero(@(w) abs(L_i(w)) - 1, [1 pi/ts]);                % [rad/s] crossover
d.pm_i = 180 + angle(L_i(d.wc_i))*180/pi;                       % [deg] phase margin
d.wc_psi = fzero(@(w) abs(L_psi(w)) - 1, [1e-3 1e3]);
d.pm_psi = 180 + angle(L_psi(d.wc_psi))*180/pi;
d.wc_w = fzero(@(w) abs(L_w(w)) - 1, [1e-2 1e3]);
d.pm_w = 180 + angle(L_w(d.wc_w))*180/pi - atand(d.wc_w/d.wc_i);    % current loop as a first-order lag
d.psi_r_at_start = c.isx_lim_max*mc.Lm_norm*(1 - exp(-param.im.mode.time_start_motor_control/d.tau_r));  % [pu]
d.t_mag = -d.tau_r*log(1 - c.psi_r_ref/(c.isx_lim_max*mc.Lm_norm));    % [s] from 0 to psi_r_ref at isx_lim_max
d.T_load = mc.load_friction*mc.omega_m_bez;                     % [Nm] friction torque at synchronous speed
d.T_acc = mc.Jm*mc.omega_m_bez/1;                               % [Nm] acceleration torque of the 1 s ramp
d.isy_ramp_end = (d.T_load + d.T_acc)/d.T_isy;                  % [pu] at the end of the ramp with psi_r_ref
isx = d.isx_ref*mc.ibez; isy = d.T_load/d.T_isy*mc.ibez;        % [A] steady state at synchronous speed
d.w_s = mc.omega_bez + isy/(isx*d.tau_r);                       % [rad/s] stator pulsation (rotor at omega_bez + slip)
us = (mc.Rs + hw.RLFi)*(isx + 1i*isy) + 1i*d.w_s*((mc.sigma + hw.LFi)*(isx + 1i*isy) + mc.Lm^2/mc.Lr*isx);
d.u_ss = abs(us);                                               % [V] peak phase voltage
d.u_ss_pu = d.u_ss/d.u_ctrl_bez;                                % [pu] of the control voltage
d.i_ss_rms = abs(isx + 1i*isy)/sqrt(2);                         % [A] rms
d.P_ss = d.T_load*mc.omega_m_bez;                               % [W] mechanical power at synchronous speed
param.im.design = d;
fprintf('Per unit: 1 pu of control voltage = udc/sqrt(3) = %.0f V = %.2f ubez | current loop: L = %.3f mH, crossover %.0f Hz, PM %.0f deg | Nyquist %.0f Hz\n', ...
    d.u_ctrl_bez, d.u_ctrl_bez/mc.ubez, d.L_tr*1e3, d.wc_i/2/pi, d.pm_i, 1/(2*ts));
fprintf('Flux: tau_r = %.2f s, isx = %.3f pu at psi_r_ref, loop crossover %.2f Hz, PM %.0f deg | psi_r = %.2f pu at the start of the ramp, %.2f s from 0 to psi_r_ref with isx = %.2f pu\n', ...
    d.tau_r, d.isx_ref, d.wc_psi/2/pi, d.pm_psi, d.psi_r_at_start, d.t_mag, c.isx_lim_max);
fprintf('Speed: %.0f Nm per pu of isy, crossover %.2f Hz, PM %.0f deg | end of ramp isy = %.2f pu | at synchronous speed: %.0f Nm, %.0f kW, %.0f A rms, %.0f V peak (%.2f pu of control), %.2f Hz\n', ...
    d.T_isy, d.wc_w/2/pi, d.pm_w, d.isy_ramp_end, d.T_load, d.P_ss/1e3, d.i_ss_rms, d.u_ss, d.u_ss_pu, d.w_s/2/pi);
clear mc hw c ts d isx isy us L_i L_psi L_w
%[text] ## Sensors end scale and quantization
param.meas.adc_quantization = 1/2^11;               % [pu] quantizer of the acquisitions (pu signals, 12 bit)
param.meas.adc12_quantization = param.meas.adc_quantization;    % read by the current acquisition
param.meas.adc16_quantization = 1/2^15;
param.meas.Imax_adc = 1049.835;                     % [A]
param.meas.CurrentQuantization = param.meas.Imax_adc/2^11;
param.meas.Umax_adc = 1500;                         % [V]
param.meas.VoltageQuantization = param.meas.Umax_adc/2^11;
%[text] ## Model setup
open_system(model);
Simulink.importExternalCTypes(model, 'Names', {'sv_pwm_output_t'});
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
