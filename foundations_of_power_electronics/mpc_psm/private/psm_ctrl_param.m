function ctrl = psm_ctrl_param(ts, omega_base, Jm_norm, u_psm_scale, mc, hw, g)
%PSM_CTRL_PARAM  Control data of the PSM of mpc_psm: data and weights of the finite-set predictive current
%   control, speed PI, double integrator observer, back-EMF observer gains and the extended Kalman filter (EKF)
%   matrices (local copy of ctrl_pmsm_setup and ekf_pmsm_setup of the library, same formulas and field names,
%   with the fixes of November 2026 described below and in the README).
%
%   ctrl = psm_ctrl_param(ts, omega_base, Jm_norm, u_psm_scale, mc, hw, g)
%     ts          : [s] control sample time (ts_inv)
%     omega_base  : [rad/s] electrical base pulsation of the machine (observer poles, EKF time scale)
%     Jm_norm     : [s] normalized inertia of the machine (stored, as ctrl_pmsm_setup)
%     u_psm_scale : voltage scale of the two non-EKF back-EMF observers (stored in ctrl.u_psm_scale)
%     mc          : machine data (param.psm.machine): Rs_m, Lq_m, bases ubez, Xbez, Lbez
%     hw          : inverter data (param.inv.hw): LFi, RLFi (output inductor), udc_nom (base of the DC voltage
%                   given to the control)
%     g           : gains set in init_model.m (copied in ctrl):
%                   k_kalman, k_sw             weights of the cost function of the predictive control (block
%                                              mp control/mpc_pu_d): current error and number of legs that
%                                              change state (switching). Legacy names: they were fields of the
%                                              EKF object (psm_ctrl.ekf.k_kalman, k_sw), now in ctrl.mpc; before
%                                              November 2026 k_sw weighted the squared voltage of the vector
%                   kp_w, ki_w, iq_lim         speed PI (inverter_control/Speed control), output = torque (q
%                                              current) reference in pu, saturated at +-iq_lim (variant
%                                              use_speed_control)
%                   emf_fb_p, emf_p            gains of the two non-EKF back-EMF observers (Schroedl type)
%                   omega_th                   [pu] speed threshold of the encoder-sensorless switch
%                   jm_ekf                     inertia used by the EKF model (1e6: speed modelled as constant)
%                   q_ekf, q_ekf_diag, r_ekf   EKF noise: Qkalman = ts*q_ekf*diag(q_ekf_diag),
%                                              Rkalman = ts*r_ekf*eye(2) (legacy: q_ekf = 1/10,
%                                              diag [ts^2 ts^2 1/2 1/2 1 1], r_ekf = 1)
%
%   Fields read by mpc_psm.slx (param.psm.ctrl.*): mpc.u_scale, mpc.L_norm, mpc.R_norm, mpc.k_kalman, mpc.k_sw,
%   kp_w, ki_w, iq_lim, emf_fb_p, emf_p, omega_th, omega_base, u_psm_scale, di_obs.komega, di_obs.ktheta,
%   ekf.A1_tilde_ekf ... ekf.A6_tilde_ekf, ekf.A6_pred_ekf, ekf.Bd_ekf, ekf.C_ekf, ekf.Qkalman, ekf.Rkalman.
%   The current PI gains and the MTPA limit of ctrl_pmsm_setup (kp_i, ki_i, id_lim, ...) are read by no block
%   of this model (the predictive control replaces the current PI) and are not built (the legacy init set
%   kp_i = 0.25, ki_i = 35).
%
%   Predictive control (MATLAB Function mpc of mp control/mpc_pu_d): the current and the DC voltage come one
%   step late, so it first computes the current of now with the vector of the last step (delay compensation).
%   Then, for each of the 8 states of the two-level bridge, it computes the alpha/beta voltage from the measured
%   DC voltage in pu of udc_nom times u_scale = udc_nom/ubez (pu of ubez, the base of the machine model),
%   predicts the current of the next step with L_norm, R_norm, ts*omega_bez and the back-EMF of the EKF, and
%   takes the state with the lowest cost
%     J = k_kalman*|i_ref - i_pred|^2 + k_sw*(number of legs that change state)
%   It gives the observers the voltage (pu of ubez) of the vector of the last step; the Unit Delay after the
%   block makes it the vector of two steps before: the vector that made the current measured one step before.
%     L_norm, R_norm  q axis of the machine plus the output inductor, pu (the EKF uses the same two values: its
%                     flux is then the "active flux" psi_m + (Ld - Lq)*id, on the d axis for any id)
%   Before November 2026: vectors from the constant vdc = 1 without u_scale (3.4 times too small at the battery
%   voltage), no delay compensation, machine Ls = (Ld + Lq)/2 and Rs, 7 vectors with one zero vector.
%
%   Double integrator observer (encoder angle -> angle and speed): A = [0 1; 0 0], C = [1 0], forward Euler,
%   poles [-1 -4]*omega_base mapped with exp(p*ts); gains in closed form (two states), the same result as acker
%   in ctrl_pmsm_setup. The extended observer edi_obs of ctrl_pmsm_setup is read by no block and is not built.
%   EKF (block EKF_bemf_observer/extended_kalman_observer): state x = [i_alpha i_beta psi_alpha psi_beta
%   omega torque_load] in pu. Two copies of the subsystem A_ekf build the matrix from the Jacobian
%   coefficients below. The copy of the covariance uses F = I + ts*omega_bez*(A1 + A2*i_alpha + A3*i_beta +
%   A4*psi_alpha + A5*psi_beta + A6*omega) (the Jacobian). The copy of the state prediction
%   x(k|k) = F_pred*x + ts*omega_bez*B*u reads A6_pred_ekf = zeros(6) on its A6 path: the products omega*psi
%   are already there through the A4/A5 paths, so F_pred*x = x + ts*omega_bez*f(x, u) (the model itself; the
%   full Jacobian would count omega*psi twice). Before November 2026 that path had A6 without omega_bez (as in
%   inv_psm): an extra 1 % of omega*psi and a speed estimate 1 % low.
%   The speed output of the EKF block comes from the double integrator observer on the EKF angle (block
%   extended_kalman_observer/double_integrator_observer, gains di_obs): the speed state of the EKF is about
%   0.4 % low at rated speed, while the angle rotates at the true speed.

    ctrl.ts = ts;
    ctrl.omega_base = omega_base;               % [rad/s] read by the encoder double integrator observer
    ctrl.Jm_norm = Jm_norm;
    ctrl.u_psm_scale = u_psm_scale;

    % predictive current control: model data (per unit) and weights of the cost function
    ctrl.mpc.L_norm = (mc.Lq_m + hw.LFi)/mc.Lbez;           % q axis of the machine + output inductor
    ctrl.mpc.R_norm = (mc.Rs_m + hw.RLFi)/mc.Xbez;
    ctrl.mpc.u_scale = hw.udc_nom/mc.ubez;                  % pu of udc_nom -> pu of ubez
    ctrl.mpc.k_kalman = g.k_kalman;             % current error (legacy name)
    ctrl.mpc.k_sw = g.k_sw;                     % number of legs that change state (switching)

    % speed PI (pu of omega_m_bez in, pu of tau_bez out), variant use_speed_control
    ctrl.kp_w = g.kp_w;
    ctrl.ki_w = g.ki_w;                         % [1/s]
    ctrl.iq_lim = g.iq_lim;                     % [pu]

    % back-EMF observers (Simulink and C-Caller versions, not active by default) and encoder switch
    ctrl.emf_fb_p = g.emf_fb_p;
    ctrl.emf_p = g.emf_p;
    ctrl.omega_th = g.omega_th;                 % [pu]

    % double integrator observer (encoder angle, and inside the three back-EMF observers)
    ctrl.di_obs = double_integrator_observer_param(ts, omega_base, [-1 -4]);

    % extended Kalman filter: the same inductance and resistance as the predictive control (legacy: machine Rs
    % and Ls = (Ld + Lq)/2, without the output inductor)
    ctrl.ekf = ekf_param(ctrl.mpc.R_norm, ctrl.mpc.L_norm, g.jm_ekf, ts, g);

    fprintf('PSM control: predictive control L = %.4f pu, R = %.4f pu, u_scale = %.4f | weights k_kalman = %g, k_sw = %g\n', ...
        ctrl.mpc.L_norm, ctrl.mpc.R_norm, ctrl.mpc.u_scale, ctrl.mpc.k_kalman, ctrl.mpc.k_sw);
    fprintf('speed PI kp = %.2f, ki = %.0f, iq_lim = %.2f | di_obs: poles %.0f and %.0f Hz, komega = %.0f, ktheta = %.0f | EKF: R = %.4f pu, L = %.4f pu, jm = %g\n', ...
        ctrl.kp_w, ctrl.ki_w, ctrl.iq_lim, -ctrl.di_obs.poles(1)/2/pi, -ctrl.di_obs.poles(2)/2/pi, ...
        ctrl.di_obs.komega, ctrl.di_obs.ktheta, ctrl.mpc.R_norm, ctrl.mpc.L_norm, g.jm_ekf);
end

function obs = double_integrator_observer_param(ts, omega, poles_norm)
    obs.Aso = [0 1; 0 0];
    obs.Asod = eye(2) + obs.Aso*ts;
    obs.Cso = [1 0];
    obs.poles = poles_norm*omega;               % [rad/s]
    obs.polesd = exp(obs.poles*ts);
    obs.Ldso = observer_gain_2x2(obs.Asod, obs.polesd);
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

function e = ekf_param(rs, ls, jm, ts, g)
% Matrices of ekf_pmsm_setup. Continuous model in pu (time scaled by omega_bez in the block):
%   di/dt   = -rs/ls*i + u/ls + (1/ls)*omega*J'*psi      (J = [0 -1; 1 0])
%   dpsi/dt = omega*J*psi
%   domega/dt = (psi_alpha*i_beta - psi_beta*i_alpha - torque_load)/jm,  dtorque_load/dt = 0
% written as A1*x + sum_j (A_j*x_j)*x, with the coefficient matrices of the Jacobian:
% A2 (x i_alpha), A3 (x i_beta), A4 (x psi_alpha), A5 (x psi_beta), A6 (x omega).
    e.Rs_norm_ekf = rs;
    e.Lalpha_norm_ekf = ls;
    e.Lbeta_norm_ekf = ls;
    e.ts = ts;

    e.A1_tilde_ekf = [-rs/ls 0 0 0 0 0; ...
                      0 -rs/ls 0 0 0 0; ...
                      0 0 0 0 0 0; ...
                      0 0 0 0 0 0; ...
                      0 0 0 0 0 -1/jm; ...
                      0 0 0 0 0 0];
    e.A2_tilde_ekf = [0 0 0 0 0 0; ...
                      0 0 0 0 0 0; ...
                      0 0 0 0 0 0; ...
                      0 0 0 0 0 0; ...
                      0 0 0 -1/jm 0 0; ...
                      0 0 0 0 0 0];
    e.A3_tilde_ekf = [0 0 0 0 0 0; ...
                      0 0 0 0 0 0; ...
                      0 0 0 0 0 0; ...
                      0 0 0 0 0 0; ...
                      0 0 1/jm 0 0 0; ...
                      0 0 0 0 0 0];
    e.A4_tilde_ekf = [0 0 0 0 0 0; ...
                      0 0 0 0 -1/ls 0; ...
                      0 0 0 0 0 0; ...
                      0 0 0 0 1 0; ...
                      0 1/jm 0 0 0 0; ...
                      0 0 0 0 0 0];
    e.A5_tilde_ekf = [0 0 0 0 1/ls 0; ...
                      0 0 0 0 0 0; ...
                      0 0 0 0 -1 0; ...
                      0 0 0 0 0 0; ...
                      -1/jm 0 0 0 0 0; ...
                      0 0 0 0 0 0];
    e.A6_tilde_ekf = [0 0 0 1/ls 0 0; ...
                      0 0 -1/ls 0 0 0; ...
                      0 0 0 -1 0 0; ...
                      0 0 1 0 0 0; ...
                      0 0 0 0 0 0; ...
                      0 0 0 0 0 0];
    e.B_tilde_ekf = [1/ls 0; 0 1/ls; 0 0; 0 0; 0 0; 0 0];
    e.C_ekf = [1 0 0 0 0 0; 0 1 0 0 0 0];
    e.Bd_ekf = e.B_tilde_ekf*ts;
    e.A_tilde_ekf = e.A1_tilde_ekf + e.A2_tilde_ekf + e.A3_tilde_ekf + e.A4_tilde_ekf + ...
        e.A5_tilde_ekf + e.A6_tilde_ekf;
    % A6 path of the copy of A_ekf of the state prediction: zero, because F_pred*x must be x + ts*f(x, u) and
    % the products omega*psi already come from the A4/A5 paths (the covariance copy keeps A6_tilde_ekf)
    e.A6_pred_ekf = zeros(6);

    % noise covariances (the legacy values "work pretty nice both generator and driver")
    e.Qkalman = ts*g.q_ekf*diag(g.q_ekf_diag);
    e.Rkalman = ts*g.r_ekf*eye(2);
end
