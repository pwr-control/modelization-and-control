function ctrl = afe_ctrl_param(ts, omega_base, f_set)
%AFE_CTRL_PARAM  Control data of single_phase_afe_ttype.slx (local replacement of ctrl_afe_setup,
%   ctrl_single_phase_inverter_setup and sogi_filter of the library for the fields the model reads).
%
%   ctrl = afe_ctrl_param(ts, omega_base, f_set)
%     ts         : [s] control sample time (ts_afe)
%     omega_base : [rad/s] nominal grid pulsation (PLL, first harmonic trackers, observer)
%     f_set      : [Hz] working frequency of the SOGI, phase-shift filters, resonant PI and impedance
%                  measure (legacy frequency_set: the grid frequency, 300 Hz in the identification test)
%
%   Fields read by single_phase_afe_ttype.slx (param.afe.ctrl.*):
%     kp_i, ki_i, u_lim, clip_release
%                       dq vector PI of the converter current (legacy dqvector_pi.kp_inv/ki_inv, copied in
%                       single_phase_inverter_ctrl.kp_inv/ki_inv by the library class): Simulink block
%                       dq_vector_pi (lim_ctrl_out, CTRPIFF_CLIP_RELEASE) or C-Caller dqvector_pi
%     rpi.kp, rpi.ki, rpi.delta
%                       resonant PI of the current (C-Caller rpi, variant rpi_enable; legacy pres_ctrl /
%                       single_phase_inverter_ctrl.kp_rpi, ki_rpi, delta_rpi)
%     f_set, omega_set  working frequency and pulsation (legacy frequency_set, omega_set)
%     fht.omega_fht, fht.delta_fht, fht.Ld_fht(1), fht.Ld_fht(2)
%                       first harmonic tracker (C-Caller first_harmonic_tracker): two instances for the
%                       PLL (grid voltages alpha, beta), one for the virtual beta of the current, one in
%                       the identification subsystem (legacy afe_ctrl.fht_grid and the undefined
%                       omega_fht1/delta_fht1/Ld_fht1)
%     sogi.kepsilon, sogi.flt_beta
%                       SOGI of the virtual beta (Simulink or C-Caller sogi_flt, omega_set) and the SOGI
%                       beta low-pass filter (tf, LTI block "pwm filtering for measure" of the voltage
%                       acquisition)
%     phsh.gain         output scaling of the phase-shift filter of the identification subsystem (legacy
%                       phase_shift_filter_gain, undefined)
%     pll.kp, pll.ki, di_obs.Adso, di_obs.Cso, di_obs.Ldso
%                       FHT-PLL stage B: PI on the q voltage plus the double-integrator observer of the phase
%
%   The observers are the same as in the library: double integrator A = [0 1; 0 0] and resonant model
%   A = [0 1; -w^2 -delta*w], C = [1 0], discretized with forward Euler, observer poles at [-1 -4]*omega_base
%   mapped with exp(p*ts). The gains are computed in closed form (two states), which gives the same result
%   as acker on the transposed pair and does not need the Control System Toolbox.

    ctrl.ts = ts;
    ctrl.omega_base = omega_base;
    ctrl.f_set = f_set;                         % [Hz]
    ctrl.omega_set = 2*pi*f_set;                % [rad/s]

    % converter current dq vector PI (legacy dqvector_pi.kp_inv 0.5, ki_inv 45, u_lim 1 of the library class,
    % CTRPIFF_CLIP_RELEASE 1e-3)
    ctrl.kp_i = 0.5;                            % [pu/pu]
    ctrl.ki_i = 45;                             % [1/s]
    ctrl.u_lim = 1.0;                           % [pu] modulation index limit of the vector (1 = udc)
    ctrl.clip_release = 1e-3;                   % [pu] hysteresis of the clip release of the block

    % resonant PI (legacy pres_ctrl: kp_rpi 0.75, ki_rpi 45, delta_rpi 0.025); the resonant model in state
    % space is informative, the C code builds it from omega_set, delta and ts
    ctrl.rpi.kp = 0.75;                         % [pu/pu]
    ctrl.rpi.ki = 45;                           % [1/s]
    ctrl.rpi.delta = 0.025;                     % [-]
    ctrl.rpi.Ares = [0 1; -ctrl.omega_set^2 -2*ctrl.rpi.delta*ctrl.omega_set];
    ctrl.rpi.Bres = [0; 1];
    ctrl.rpi.Cres = [0 1];
    ctrl.rpi.Aresd = eye(2) + ctrl.rpi.Ares*ts;
    ctrl.rpi.Bresd = ctrl.rpi.Bres*ts;
    ctrl.rpi.Cresd = ctrl.rpi.Cres;

    % first harmonic tracker (PLL stage A, virtual beta of the current, identification subsystem)
    ctrl.fht = fht_observer_param(ts, omega_base, 0.05, [-1 -4]);

    % SOGI of the virtual beta (legacy sogi_filter(omega_set, 1, 2, ts)); the beta filter is read by the
    % LTI block of the voltage acquisition
    ctrl.sogi.omega = ctrl.omega_set;
    ctrl.sogi.delta = 1;
    ctrl.sogi.kepsilon = 2;
    s = tf('s');
    w = ctrl.sogi.omega;
    ctrl.sogi.flt_alpha = ctrl.sogi.delta*w*s/(s^2 + ctrl.sogi.delta*w*s + w^2);
    ctrl.sogi.flt_beta = ctrl.sogi.delta*w^2/(s^2 + ctrl.sogi.delta*w*s + w^2);

    % phase-shift filter of the identification subsystem: output scaling (the legacy name was undefined)
    ctrl.phsh.gain = 1;

    % FHT-PLL stage B: PI (legacy pll_p_fht 1, pll_i1_fht 80) and double integrator observer of the phase
    ctrl.pll.kp = 1;
    ctrl.pll.ki = 80;                           % [1/s]
    ctrl.di_obs = double_integrator_observer_param(ts, omega_base, [-1 -4]);

    fprintf('Control: current PI kp = %.2f, ki = %.0f, u_lim = %.2f | resonant PI kp = %.2f, ki = %.0f, delta = %.3f at %.0f Hz | PLL kp = %.1f, ki = %.0f\n', ...
        ctrl.kp_i, ctrl.ki_i, ctrl.u_lim, ctrl.rpi.kp, ctrl.rpi.ki, ctrl.rpi.delta, f_set, ctrl.pll.kp, ctrl.pll.ki);
    fprintf('FHT: omega = %.1f rad/s, delta = %.2f, Ld = [%.4f %.4f] | di_obs: Ld = [%.4f %.4f] | SOGI: kepsilon = %.1f, delta = %.1f\n', ...
        ctrl.fht.omega_fht, ctrl.fht.delta_fht, ctrl.fht.Ld_fht(1), ctrl.fht.Ld_fht(2), ...
        ctrl.di_obs.Ldso(1), ctrl.di_obs.Ldso(2), ctrl.sogi.kepsilon, ctrl.sogi.delta);
end

function fht = fht_observer_param(ts, omega_fht, delta_fht, poles_norm)
    fht.omega_fht = omega_fht;                  % [rad/s] tracked pulsation
    fht.delta_fht = delta_fht;                  % [-] damping term of the resonant model
    fht.Afht = [0 1; -omega_fht^2 -delta_fht*omega_fht];
    fht.Cfht = [1 0];
    fht.Ad_fht = eye(2) + fht.Afht*ts;          % forward Euler, as the C code first_harmonic_tracker_init
    fht.poles_fht = poles_norm*omega_fht;       % [rad/s] continuous-time observer poles
    fht.polesd_fht = exp(ts*fht.poles_fht);     % discrete-time observer poles
    fht.L_fht = observer_gain_2x2(fht.Afht, fht.poles_fht);
    fht.Ld_fht = observer_gain_2x2(fht.Ad_fht, fht.polesd_fht);
end

function obs = double_integrator_observer_param(ts, omega, poles_norm)
    obs.Aso = [0 1; 0 0];
    obs.Adso = eye(2) + obs.Aso*ts;
    obs.Cso = [1 0];
    obs.poles = poles_norm*omega;               % [rad/s]
    obs.polesd = exp(obs.poles*ts);
    obs.Ldso = observer_gain_2x2(obs.Adso, obs.polesd);
    obs.ktheta = obs.Ldso(1)/ts;                % gains of the C-Caller form x' = v + kx*(u - x), v' = kv*(u - x)
    obs.komega = obs.Ldso(2)/ts;
end

function L = observer_gain_2x2(A, p)
% Observer gain L (2x1) such that eig(A - L*C) = p, with C = [1 0]; closed-form Ackermann for two states:
% det(zI - (A - L*C)) = z^2 - (a11 - l1 + a22)*z + (a11 - l1)*a22 - a12*(a21 - l2)
    a1 = -(p(1) + p(2));
    a0 = p(1)*p(2);
    l1 = A(1,1) + A(2,2) + a1;
    l2 = A(2,1) + (a0 - (A(1,1) - l1)*A(2,2))/A(1,2);
    L = [l1; l2];
end
