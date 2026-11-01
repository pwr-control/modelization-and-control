function ctrl = im_ctrl_param(ts, omega_base, Jm_norm, g)
%IM_CTRL_PARAM  Field-oriented control data of the induction machine of im_analysis_and_design (local
%   replacement of ctrl_im_setup and of the filter filters.inv.SOF_10Hz of setup_global_filters of the
%   library, plus the gains that were literals in the blocks of the legacy model).
%
%   ctrl = im_ctrl_param(ts, omega_base, Jm_norm, g)
%     ts         : [s] control sample time (ts_inv)
%     omega_base : [rad/s] electrical base pulsation of the machine (observer poles)
%     Jm_norm    : [s] normalized inertia of the machine (stored, as ctrl_im_setup)
%     g          : gains set in init_model.m (copied in ctrl):
%                  kp_w, ki_w, iq_lim            speed PI (Speed control): output = torque (isy) reference,
%                                                saturated at +-iq_lim, anti-windup by back-calculation
%                  kp_i, ki_i                    x and y current PI (current control/pi-ctrl-x, pi-ctrl-y)
%                  id_lim                        legacy field of ctrl_im_setup (0.35, read by no block)
%                  kp_psi, ki_psi,               rotor flux PI (magnetization current reference/pi, literals
%                  isx_lim_min, isx_lim_max      2, 20, 0.05*Lm_norm and 0.6*Lm_norm in the legacy model):
%                                                output = Lm_norm*isx reference, saturated at
%                                                [isx_lim_min, isx_lim_max]*Lm_norm
%                  psi_r_ref                     [pu] rotor flux reference below base speed (Constant
%                                                psi_r_ref, default value 1 of the block in the legacy model)
%                  fcut_speed_ref                [Hz] corner of the second-order filter of the speed reference
%                                                (legacy filters.inv.SOF_10Hz)
%
%   Fields read by the models (param.im.ctrl.*): kp_w, ki_w, iq_lim, kp_inv_d, ki_inv_d, kp_inv_q, ki_inv_q,
%   kp_psi, ki_psi, isx_lim_min, isx_lim_max, psi_r_ref, SOF_10Hz.ss_d, di_obs.Adso, di_obs.Cso, di_obs.Ldso,
%   di_obs.komega, di_obs.ktheta.
%
%   PI form of the model (all three loops, backward-Euler integral): u(k) = kp*e(k) + ki*ts*e(k) + x(k),
%   x(k+1) = u(k) - kp*e(k) (speed and flux PI: back-calculation on the saturated output) or x(k+1) = x(k) +
%   ki*ts*e(k) (current PI: frozen and decaying with CTRPIFF_CLIP_RELEASE while the voltage vector exceeds
%   sqrt(3)/2; the vector is scaled to length 1 above 1).
%   Double integrator observer of the encoder angle (induction motor/mechanical data and data_analysis):
%   A = [0 1; 0 0], C = [1 0], forward Euler, poles [-1 -4]*omega_base mapped with exp(p*ts); gains in
%   closed form (two states), the same result as acker of ctrl_im_setup. The extended observer edi_obs of
%   ctrl_im_setup is read by no block and is not built.

    ctrl.ts = ts;
    ctrl.omega_base = omega_base;
    ctrl.Jm_norm = Jm_norm;

    % speed PI (pu of omega_m_bez in, pu of ibez out)
    ctrl.kp_w = g.kp_w;
    ctrl.ki_w = g.ki_w;                         % [1/s]
    ctrl.iq_lim = g.iq_lim;                     % [pu]
    ctrl.id_lim = g.id_lim;                     % [pu] (read by no block)

    % current PI, same gains on the two axes (field names of ctrl_im_setup)
    ctrl.kp_i = g.kp_i;
    ctrl.ki_i = g.ki_i;                         % [1/s]
    ctrl.kp_id = ctrl.kp_i;
    ctrl.ki_id = ctrl.ki_i;
    ctrl.kp_iq = ctrl.kp_i;
    ctrl.ki_iq = ctrl.ki_i;
    ctrl.kp_inv_d = ctrl.kp_i;                  % read by pi-ctrl-x
    ctrl.ki_inv_d = ctrl.ki_i;
    ctrl.kp_inv_q = ctrl.kp_i;                  % read by pi-ctrl-y
    ctrl.ki_inv_q = ctrl.ki_i;

    % rotor flux PI and flux reference
    ctrl.kp_psi = g.kp_psi;
    ctrl.ki_psi = g.ki_psi;                     % [1/s]
    ctrl.isx_lim_min = g.isx_lim_min;           % [pu] of ibez
    ctrl.isx_lim_max = g.isx_lim_max;           % [pu] of ibez
    ctrl.psi_r_ref = g.psi_r_ref;               % [pu] of psi_bez = ubez/omega_bez

    % double integrator observer of the encoder angle
    ctrl.di_obs = double_integrator_observer_param(ts, omega_base, [-1 -4]);

    % second-order filter of the speed reference: H(s) = 1/(tau*s + 1)^2, ZOH at ts (sof_setup of the library)
    ctrl.SOF_10Hz = sof_param(g.fcut_speed_ref, ts);

    fprintf('IM control: speed PI kp = %.2f, ki = %.0f, iq_lim = %.2f | current PI kp = %.2f, ki = %.0f | flux PI kp = %.1f, ki = %.0f, isx in [%.2f %.2f] pu, psi_r_ref = %.2f\n', ...
        ctrl.kp_w, ctrl.ki_w, ctrl.iq_lim, ctrl.kp_i, ctrl.ki_i, ctrl.kp_psi, ctrl.ki_psi, ctrl.isx_lim_min, ...
        ctrl.isx_lim_max, ctrl.psi_r_ref);
    fprintf('di_obs: Ld = [%.4f %.6f] (komega = %.0f, ktheta = %.0f) | speed reference filter: %.0f Hz second order\n', ...
        ctrl.di_obs.Ldso(1), ctrl.di_obs.Ldso(2), ctrl.di_obs.komega, ctrl.di_obs.ktheta, ctrl.SOF_10Hz.fcut);
end

function obs = double_integrator_observer_param(ts, omega, poles_norm)
    obs.Aso = [0 1; 0 0];
    obs.Adso = eye(2) + obs.Aso*ts;
    obs.Cso = [1 0];
    obs.poles = poles_norm*omega;               % [rad/s]
    obs.polesd = exp(obs.poles*ts);
    obs.Ldso = observer_gain_2x2(obs.Adso, obs.polesd);
    obs.komega = obs.Ldso(2)/ts;
    obs.ktheta = obs.Ldso(1)/ts;
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

function f = sof_param(fcut, ts)
    f.fcut = fcut;                              % [Hz]
    f.ts = ts;                                  % [s]
    f.time_constant = 1/(2*pi*fcut);            % [s]
    s = tf('s');
    sof_c = 1/(f.time_constant*s + 1)^2;
    f.ss_c = ss(sof_c);
    f.ss_d = ss(c2d(sof_c, ts, 'zoh'));         % read by the LTI System block of the speed reference
end
