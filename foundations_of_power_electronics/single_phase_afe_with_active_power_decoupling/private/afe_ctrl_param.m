function ctrl = afe_ctrl_param(ts, omega_base)
%AFE_CTRL_PARAM  Control data of single_phase_afe.slx (local replacement of ctrl_afe_setup and
%   ctrl_single_phase_inverter_setup of the library for the fields the model reads).
%
%   ctrl = afe_ctrl_param(ts, omega_base)
%     ts         : [s] control sample time (ts_afe)
%     omega_base : [rad/s] nominal grid pulsation
%
%   Fields read by single_phase_afe.slx (param.afe.ctrl.*):
%     udc_pu_ref, t_udc_ref, tau_udc_ref, kp_udc, ki_udc, i_ref_lim
%                       DC-link voltage loop: step of the reference at t_udc_ref through a first-order
%                       filter tau_udc_ref, PI (gains of the legacy afe_ctrl: kp_udc_ctrl 2 overridden
%                       in the init, ki_udc_ctrl 35), limit of the current amplitude reference
%     kp_i, ki_i, u_lim, clip_release
%                       dq vector PI of the grid current (legacy single_phase_inverter_ctrl.kp_inv/ki_inv/
%                       u_lim and CTRPIFF_CLIP_RELEASE)
%     fht.omega_fht, fht.delta_fht, fht.Ld_fht(1), fht.Ld_fht(2)
%                       first harmonic tracker (C-Caller first_harmonic_tracker): three instances for
%                       the PLL (grid voltages alpha, beta) and one for the virtual beta of the current
%     sogi.kepsilon, sogi.flt_beta
%                       SOGI of the inactive variant of the virtual beta, and the SOGI beta low-pass
%                       filter (tf, LTI block "pwm filtering for measure" of the voltage acquisition)
%     pll.kp, pll.ki, di_obs.Adso, di_obs.Cso, di_obs.Ldso
%                       FHT-PLL: PI on the q voltage plus the double-integrator observer of the phase
%     di_obs.ktheta, di_obs.komega, omega_base
%                       derivative observer of udc_pu of the ripple detector (C-Caller
%                       linear_double_integrator_observer), same gains as the PLL observer divided by ts
%     apd.kp, apd.ki, apd.lim, apd.t_enable
%                       dq vector PI of the 100 Hz ripple loop (enabled at t_enable)
%     buffer.kp_u, buffer.ki_u, buffer.lim_u, buffer.tau_ref, buffer.t_ref
%                       PI of the buffer DC voltage (reference udc_pu_ref set in the init)
%     buffer.kp_i, buffer.ki_i, buffer.lim_i
%                       PI of the buffer current (feed-forward udc_pu added by the block)
%
%   The observers are the same as in the library: double integrator A = [0 1; 0 0] and resonant model
%   A = [0 1; -w^2 -delta*w], C = [1 0], discretized with forward Euler, observer poles at [-1 -4]*omega_base
%   mapped with exp(p*ts). The gains are computed in closed form (two states), which gives the same result
%   as acker on the transposed pair and does not need the Control System Toolbox.

    ctrl.ts = ts;
    ctrl.omega_base = omega_base;

    % DC-link voltage loop (legacy: afe_ctrl.udc_norm_ref 1, kp_udc_ctrl 2, ki_udc_ctrl 35, ixi_pos_ref_lim 1.65,
    % step at 1 ms, first-order filter 20 ms on the reference)
    ctrl.udc_pu_ref = 1;                        % [pu] of udc_nom
    ctrl.t_udc_ref = 1e-3;                      % [s]
    ctrl.tau_udc_ref = 20e-3;                   % [s]
    ctrl.kp_udc = 2;                            % [pu/pu]
    ctrl.ki_udc = 35;                           % [1/s]
    ctrl.i_ref_lim = 1.65;                      % [pu] of ibez, amplitude of the grid current reference

    % grid current dq vector PI (legacy dqvector_pi.kp_inv 0.5, ki_inv 45, u_lim 1, CTRPIFF_CLIP_RELEASE 1e-3)
    ctrl.kp_i = 0.5;                            % [pu/pu]
    ctrl.ki_i = 45;                             % [1/s]
    ctrl.u_lim = 1.0;                           % [pu] modulation index limit of the vector
    ctrl.clip_release = 1e-3;                   % [pu] hysteresis of the clip release of the block

    % first harmonic tracker (PLL stage A and virtual beta of the current)
    ctrl.fht = fht_observer_param(ts, omega_base, 0.05, [-1 -4]);

    % SOGI (inactive variant of the virtual beta; the beta filter is used by the voltage acquisition)
    ctrl.sogi.omega = omega_base;
    ctrl.sogi.delta = 1;
    ctrl.sogi.kepsilon = 2;
    s = tf('s');
    ctrl.sogi.flt_alpha = ctrl.sogi.delta*omega_base*s/(s^2 + ctrl.sogi.delta*omega_base*s + omega_base^2);
    ctrl.sogi.flt_beta = ctrl.sogi.delta*omega_base^2/(s^2 + ctrl.sogi.delta*omega_base*s + omega_base^2);

    % FHT-PLL stage B: PI (legacy pll_p_fht 1, pll_i1_fht 80) and double integrator observer of the phase
    ctrl.pll.kp = 1;
    ctrl.pll.ki = 80;                           % [1/s]
    ctrl.di_obs = double_integrator_observer_param(ts, omega_base, [-1 -4]);

    % 100 Hz ripple loop (active power decoupling): dq vector PI on the detected ripple, references 0
    ctrl.apd.kp = 1;                            % [pu/pu]
    ctrl.apd.ki = 18;                           % [1/s]
    ctrl.apd.lim = 1;                           % [pu] of the buffer current base
    ctrl.apd.t_enable = 0.5;                    % [s] the loop is enabled at this time (Step block)

    % buffer (ripple port) loops: DC voltage PI -> DC current reference, current PI -> modulation index
    ctrl.buffer.kp_u = 2;                       % [pu/pu]
    ctrl.buffer.ki_u = 18;                      % [1/s]
    ctrl.buffer.lim_u = 1;                      % [pu] of the buffer current base
    ctrl.buffer.tau_ref = 20e-3;                % [s] first-order filter on the voltage reference
    ctrl.buffer.t_ref = 1e-3;                   % [s] step time of the voltage reference
    ctrl.buffer.kp_i = 1;                       % [pu/pu]
    ctrl.buffer.ki_i = 18;                      % [1/s]
    ctrl.buffer.lim_i = 1;                      % [-] modulation index limit (before and after the feed-forward)

    fprintf('Control: udc loop kp = %.2f, ki = %.0f, i_ref_lim = %.2f pu | current PI kp = %.2f, ki = %.0f, u_lim = %.2f | PLL kp = %.1f, ki = %.0f\n', ...
        ctrl.kp_udc, ctrl.ki_udc, ctrl.i_ref_lim, ctrl.kp_i, ctrl.ki_i, ctrl.u_lim, ctrl.pll.kp, ctrl.pll.ki);
    fprintf('FHT: omega = %.1f rad/s, delta = %.2f, Ld = [%.4f %.4f] | di_obs: Ld = [%.4f %.4f], ktheta = %.0f, komega = %.0f\n', ...
        ctrl.fht.omega_fht, ctrl.fht.delta_fht, ctrl.fht.Ld_fht(1), ctrl.fht.Ld_fht(2), ...
        ctrl.di_obs.Ldso(1), ctrl.di_obs.Ldso(2), ctrl.di_obs.ktheta, ctrl.di_obs.komega);
    fprintf('Ripple loop: kp = %.1f, ki = %.0f, enabled at %.2f s | buffer voltage PI %.1f/%.0f, buffer current PI %.1f/%.0f\n', ...
        ctrl.apd.kp, ctrl.apd.ki, ctrl.apd.t_enable, ctrl.buffer.kp_u, ctrl.buffer.ki_u, ctrl.buffer.kp_i, ctrl.buffer.ki_i);
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
