function ctrl = afe_ctrl_param(ts, omega_base, g)
%AFE_CTRL_PARAM  Control data of the AFE of ps_25V_12kA_r.slx (local replacement of ctrl_afe_setup of the
%   library for the fields the model reads, with the gains the legacy init set after the setup; the field
%   names are those of the library class, which the model reads).
%
%   ctrl = afe_ctrl_param(ts, omega_base, g)
%     ts         : [s] control sample time (ts_afe)
%     omega_base : [rad/s] nominal grid pulsation (PLLs, first harmonic trackers, observer, rms windows)
%     g          : gains set in init_model.m (copied in ctrl):
%                  kp_is, ki_is                dq vector PI of the converter current, same gains on xi and
%                                              eta (fields kp_is_xi, ki_is_xi, kp_is_eta, ki_is_eta; blocks
%                                              current_ctrl/dq_vector_pi/pi_xi and pi_eta)
%                  clip_release                [1/s] decay rate of the integrators while the voltage vector
%                                              is clipped (legacy CTRPIFF_CLIP_RELEASE, gain clip_release*ts)
%                  kp_udc_ctrl, ki_udc_ctrl, udc_norm_ref, ixi_pos_ref_lim
%                                              DC-link voltage PI (block udc_ctrl): output = active (xi)
%                                              current reference, saturated at +-ixi_pos_ref_lim
%                  kp_rc_pos_grid, ki_rc_pos_grid, ieta_pos_ref_lim
%                                              PI of the positive-sequence reactive (eta) grid current (block
%                                              grid_current_pos_sequence_and_reactive_pwr_ctrl/
%                                              sequences_current_control/Positive-Seq. eta-Control), output
%                                              saturated at +-ieta_pos_ref_lim
%                  pll_p, pll_i1, pll_p_frt    PI of the PLLs: Simulink dq PLL (pll_p, pll_i1; pll_p_frt is the
%                                              scheduled gain during a fault) and the two FHT-PLLs (pll_p_fht =
%                                              pll_p, pll_i1_fht = pll_i1, as the library)
%                  align.kp, align.ki, align.lim_up, align.lim_down
%                                              PI of the local time alignment to the master time
%                                              (phase_compensation_1/alignment_eval_afe1/pi; legacy kp_align,
%                                              ki_align, lim_up_align, lim_down_align)
%
%   Fields read by ps_25V_12kA_r.slx (param.afe.ctrl.*): the gains above and
%     fht_grid.omega_fht, fht_grid.delta_fht, fht_grid.Ld_fht, fht_grid.Ad_fht, fht_grid.Cfht
%                       first harmonic trackers of the FHT-PLLs (C-Caller and Simulink, stage A on the alpha
%                       and beta grid voltages)
%     di_obs.Adso, di_obs.Cso, di_obs.Ldso
%                       double integrator observer of the phase (stage B of the PLLs)
%     rms.n1, rms.n10   samples in 1 and 10 grid periods (moving averages of instantaneus_power)
%
%   The observers are the same as in the library: double integrator A = [0 1; 0 0] and resonant model
%   A = [0 1; -w^2 -delta*w], C = [1 0], discretized with forward Euler, observer poles at [-1 -4]*omega_base
%   mapped with exp(p*ts). The gains are computed in closed form (two states), which gives the same result
%   as acker on the transposed pair and does not need the Control System Toolbox.

    ctrl.ts = ts;
    ctrl.omega_base = omega_base;

    % converter current dq vector PI (pi_xi and pi_eta: x(k+1) = x(k) + ki*ts*e(k), u(k) = kp*e(k) + x(k+1);
    % the vector |[u_xi u_eta]| is clipped at 1 and, while the clip flag of the previous sample is set, the
    % integrator is frozen and decays with clip_release*ts)
    ctrl.kp_is_xi = g.kp_is;                    % [pu/pu]
    ctrl.ki_is_xi = g.ki_is;                    % [1/s]
    ctrl.kp_is_eta = g.kp_is;
    ctrl.ki_is_eta = g.ki_is;
    ctrl.clip_release = g.clip_release;        % [1/s]

    % DC-link voltage PI: output = active (xi) current reference, saturated at +-ixi_pos_ref_lim
    ctrl.udc_norm_ref = g.udc_norm_ref;         % [pu] of udc_nom
    ctrl.kp_udc_ctrl = g.kp_udc_ctrl;           % [pu/pu]
    ctrl.ki_udc_ctrl = g.ki_udc_ctrl;           % [1/s]
    ctrl.ixi_pos_ref_lim = g.ixi_pos_ref_lim;   % [pu] of ibez

    % positive-sequence reactive grid current PI (its output is the eta current reference, limited)
    ctrl.kp_rc_pos_grid = g.kp_rc_pos_grid;     % [pu/pu]
    ctrl.ki_rc_pos_grid = g.ki_rc_pos_grid;     % [1/s]
    ctrl.ieta_pos_ref_lim = g.ieta_pos_ref_lim; % [pu] of ibez

    % PLLs: Simulink dq PLL (pll_p, pll_i1, pll_p_frt) and FHT-PLLs (pll_p_fht, pll_i1_fht)
    ctrl.pll_p = g.pll_p;
    ctrl.pll_i1 = g.pll_i1;                     % [1/s]
    ctrl.pll_p_frt = g.pll_p_frt;
    ctrl.pll_p_fht = g.pll_p;
    ctrl.pll_i1_fht = g.pll_i1;                 % [1/s]

    % local time alignment to the master time (PI on the time error, output limits)
    ctrl.align = g.align;

    % first harmonic tracker of the grid voltages (stage A of the FHT-PLLs)
    ctrl.fht_grid = fht_observer_param(ts, omega_base, 0.05, [-1 -4]);

    % double integrator observer of the phase (stage B of the PLLs)
    ctrl.di_obs = double_integrator_observer_param(ts, omega_base, [-1 -4]);

    % number of samples in 1 and 10 grid periods
    ctrl.rms.n1 = 2*pi*1/omega_base/ts;
    ctrl.rms.n10 = 2*pi*10/omega_base/ts;

    fprintf('AFE control: current PI kp = %.2f, ki = %.0f | udc PI kp = %.2f, ki = %.0f, ixi limit = %.2f | reactive current PI kp = %.2f, ki = %.0f, ieta limit = %.2f | PLL kp = %.1f, ki = %.0f, kp_frt = %.1f\n', ...
        ctrl.kp_is_xi, ctrl.ki_is_xi, ctrl.kp_udc_ctrl, ctrl.ki_udc_ctrl, ctrl.ixi_pos_ref_lim, ...
        ctrl.kp_rc_pos_grid, ctrl.ki_rc_pos_grid, ctrl.ieta_pos_ref_lim, ctrl.pll_p, ctrl.pll_i1, ctrl.pll_p_frt);
    fprintf('FHT: omega = %.1f rad/s, delta = %.2f, Ld = [%.4f %.4f] | di_obs: Ld = [%.4f %.6f] | rms windows: %.1f and %.1f samples\n', ...
        ctrl.fht_grid.omega_fht, ctrl.fht_grid.delta_fht, ctrl.fht_grid.Ld_fht(1), ctrl.fht_grid.Ld_fht(2), ...
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
