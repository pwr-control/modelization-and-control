function ctrl = afe_ctrl_param(ts, omega_base, g)
%AFE_CTRL_PARAM  Control data of afe_dq_ttype.slx (local replacement of ctrl_afe_setup of the library for
%   the fields the model reads, plus the gains the legacy init set after the setup).
%
%   ctrl = afe_ctrl_param(ts, omega_base, g)
%     ts         : [s] control sample time (ts_afe)
%     omega_base : [rad/s] nominal grid pulsation (PLLs, first harmonic trackers, observer, rms windows)
%     g          : gains set in init_model.m (copied in ctrl):
%                  kp_i, ki_i, clip_release    dq vector PI of the converter current (pi_d, pi_q of
%                                              current_ctrl/dq_vector_pi; legacy psm_ctrl.kp_i, psm_ctrl.ki_i,
%                                              CTRPIFF_CLIP_RELEASE)
%                  kp_udc, ki_udc, udc_pu_ref, i_ref_lim
%                                              DC-link voltage PI (udc_ctrl; legacy afe_ctrl.kp_udc_ctrl,
%                                              ki_udc_ctrl, udc_norm_ref, ixi_pos_ref_lim)
%                  kp_is_z, ki_is_z            zero-sequence current PI (zero_current_ctrl; legacy
%                                              afe_ctrl.kp_is_z, ki_is_z)
%                  pll.kp, pll.ki, pll.kp_frt  PI of the PLLs (legacy afe_ctrl.pll_p, pll_i1, pll_p_frt; the
%                                              FHT-PLL reads pll.kp_fht = pll.kp and pll.ki_fht = pll.ki as
%                                              the library)
%                  align.kp, align.ki, align.lim_up, align.lim_down
%                                              PI of the local time alignment to the master time
%                                              (acquisitions/phase_compensation_1; legacy kp_align, ...)
%
%   Fields read by afe_dq_ttype.slx (param.afe.ctrl.*): the gains above and
%     fht.omega_fht, fht.delta_fht, fht.Ld_fht, fht.Ad_fht, fht.Cfht
%                       first harmonic trackers of the two FHT-PLLs (C-Caller and Simulink, stage A on
%                       the alpha and beta grid voltages; legacy afe_ctrl.fht_grid)
%     di_obs.Adso, di_obs.Cso, di_obs.Ldso
%                       double integrator observer of the phase (stage B of the PLLs)
%     rms.n1, rms.n10   samples in 1 and 10 grid periods (rms and moving averages of instantaneus_power)
%
%   The observers are the same as in the library: double integrator A = [0 1; 0 0] and resonant model
%   A = [0 1; -w^2 -delta*w], C = [1 0], discretized with forward Euler, observer poles at [-1 -4]*omega_base
%   mapped with exp(p*ts). The gains are computed in closed form (two states), which gives the same result
%   as acker on the transposed pair.

    ctrl.ts = ts;
    ctrl.omega_base = omega_base;

    % converter current dq vector PI (pi_d and pi_q: x(k+1) = x(k) + ki*ts*e(k), u(k) = kp*e(k) + x(k+1); the
    % vector |[ud uq]| is clipped at 1 and, while the clip flag of the previous sample is set, the integrator
    % is frozen and decays with clip_release*ts)
    ctrl.kp_i = g.kp_i;                         % [pu/pu]
    ctrl.ki_i = g.ki_i;                         % [1/s]
    ctrl.clip_release = g.clip_release;        % [1/s] decay rate of the integrator while clipped

    % DC-link voltage PI: output = active (xi) current reference, saturated at +-i_ref_lim, anti-windup by
    % back-calculation (x(k+1) = output - kp*e); reference udc_pu_ref (step at 1 ms, first-order filter 20 ms
    % in the model)
    ctrl.kp_udc = g.kp_udc;                     % [pu/pu]
    ctrl.ki_udc = g.ki_udc;                     % [1/s]
    ctrl.udc_pu_ref = g.udc_pu_ref;             % [pu] of udc_nom
    ctrl.i_ref_lim = g.i_ref_lim;               % [pu] of ibez

    % zero-sequence current PI (its output reaches the modulator through u_cm_comp_mod1, 0 here)
    ctrl.kp_is_z = g.kp_is_z;                   % [pu/pu]
    ctrl.ki_is_z = g.ki_is_z;                   % [1/s]

    % PLLs: Simulink dq PLL (kp, ki, kp_frt) and FHT-PLLs (kp_fht, ki_fht, same values as the library)
    ctrl.pll.kp = g.pll.kp;
    ctrl.pll.ki = g.pll.ki;                     % [1/s]
    ctrl.pll.kp_frt = g.pll.kp_frt;
    ctrl.pll.kp_fht = g.pll.kp;
    ctrl.pll.ki_fht = g.pll.ki;                 % [1/s]

    % local time alignment to the master time (PI on the time error, output limits)
    ctrl.align = g.align;

    % first harmonic tracker of the grid voltages (stage A of the FHT-PLLs)
    ctrl.fht = fht_observer_param(ts, omega_base, 0.05, [-1 -4]);

    % double integrator observer of the phase (stage B of the PLLs)
    ctrl.di_obs = double_integrator_observer_param(ts, omega_base, [-1 -4]);

    % number of samples in 1 and 10 grid periods
    ctrl.rms.n1 = 2*pi*1/omega_base/ts;
    ctrl.rms.n10 = 2*pi*10/omega_base/ts;

    fprintf('Control: current PI kp = %.2f, ki = %.0f | udc PI kp = %.2f, ki = %.0f, i_ref_lim = %.2f | zero-sequence PI kp = %.2f, ki = %.0f | PLL kp = %.1f, ki = %.0f, kp_frt = %.1f\n', ...
        ctrl.kp_i, ctrl.ki_i, ctrl.kp_udc, ctrl.ki_udc, ctrl.i_ref_lim, ctrl.kp_is_z, ctrl.ki_is_z, ...
        ctrl.pll.kp, ctrl.pll.ki, ctrl.pll.kp_frt);
    fprintf('FHT: omega = %.1f rad/s, delta = %.2f, Ld = [%.4f %.4f] | di_obs: Ld = [%.4f %.6f] | rms windows: %.0f and %.0f samples\n', ...
        ctrl.fht.omega_fht, ctrl.fht.delta_fht, ctrl.fht.Ld_fht(1), ctrl.fht.Ld_fht(2), ...
        ctrl.di_obs.Ldso(1), ctrl.di_obs.Ldso(2), ctrl.rms.n1, ctrl.rms.n10);
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
