function ctrl = psm_ctrl_param(ts, omega_base, Jm_norm, u_psm_scale, mc, g)
%PSM_CTRL_PARAM  Control data of the PSM of mpc_ttype_psm: weights of the finite-set predictive current
%   control, speed PI, double integrator observer, back-EMF observer gains and the extended Kalman filter
%   (EKF) matrices (local copy of ctrl_pmsm_setup and ekf_pmsm_setup of the library, same formulas and field
%   names; the same EKF as in inv_psm).
%
%   ctrl = psm_ctrl_param(ts, omega_base, Jm_norm, u_psm_scale, mc, g)
%     ts          : [s] control sample time (ts_inv)
%     omega_base  : [rad/s] electrical base pulsation of the machine (observer poles, EKF time scale)
%     Jm_norm     : [s] normalized inertia of the machine (stored, as ctrl_pmsm_setup)
%     u_psm_scale : voltage scale of the two non-EKF back-EMF observers (stored in ctrl.u_psm_scale)
%     mc          : machine data (param.psm.machine): Rs_norm and Ls_norm for the EKF model
%     g           : gains set in init_model.m (copied in ctrl):
%                   k_kalman, k_dc, k_sw   weights of the cost function of the predictive control (block
%                                          mp control/mpc_pu_d): current error, neutral-point voltage,
%                                          voltage change. Legacy names: they were fields of the EKF object
%                                          (psm_ctrl.ekf.k_kalman, k_dc, k_sw), now in ctrl.mpc
%                   kp_w, ki_w, iq_lim     speed PI (inverter_control/Speed control), output = torque (q
%                                          current) reference in pu, saturated at +-iq_lim (variant
%                                          use_speed_control)
%                   emf_fb_p, emf_p        gains of the two non-EKF back-EMF observers (Schroedl type)
%                   omega_th               [pu] speed threshold of the encoder-sensorless switch
%                   jm_ekf                 inertia used by the EKF model (1e6: speed modelled as constant)
%                   q_ekf, r_ekf           EKF noise: Qkalman = ts*q_ekf*diag(q_ekf_diag), Rkalman = ts*r_ekf*eye(2)
%                   q_ekf_diag             (legacy: q_ekf = 1/10, diag [ts^2 ts^2 1/2 1/2 1 1], r_ekf = 1)
%
%   Fields read by mpc_ttype_psm.slx (param.psm.ctrl.*): mpc.k_kalman, mpc.k_dc, mpc.k_sw, kp_w, ki_w, iq_lim,
%   emf_fb_p, emf_p, omega_th, omega_base, u_psm_scale, di_obs.komega, di_obs.ktheta, ekf.A1_tilde_ekf ...
%   ekf.A6_tilde_ekf, ekf.Bd_ekf, ekf.C_ekf, ekf.Qkalman, ekf.Rkalman.
%   The current PI gains and the MTPA limit of ctrl_pmsm_setup (kp_i, ki_i, id_lim, ...) are read by no block
%   of this model (the predictive control replaces the current PI) and are not built.
%
%   Predictive control (MATLAB Function mpc_ttype_gate): for each of the 27 states [-1 0 1]^3 of the T-type
%   bridge it computes the alpha/beta voltage vdc_tot/3*(...) with vdc_tot = vdc1 + vdc2 in pu of udc_nom
%   (Vdc_scale of the DC link block), predicts the current one step ahead with the machine model in pu
%   (ts*omega_bez, Rs_norm, Lalpha_norm = Lbeta_norm = Ls_norm, back-EMF from the EKF speed and flux) and
%   takes the state with the lowest cost
%     J = k_kalman*|i_ref - i_pred|^2 + k_sw*|u - u_prev|^2 + k_dc*(vdc1 - vdc2 + inp*ts/1e-3)^2
%   (inp: current of the neutral point of that state). See the README for the voltage base and the delays.
%
%   Double integrator observer (encoder angle -> angle and speed): A = [0 1; 0 0], C = [1 0], forward Euler,
%   poles [-1 -4]*omega_base mapped with exp(p*ts); gains in closed form (two states), the same result as acker
%   in ctrl_pmsm_setup. The extended observer edi_obs of ctrl_pmsm_setup is read by no block and is not built.
%   EKF (block EKF_bemf_observer/extended_kalman_observer): state x = [i_alpha i_beta psi_alpha psi_beta
%   omega torque_load] in pu. Two copies of the subsystem A_ekf build the matrix from the Jacobian
%   coefficients below. The copy of the covariance uses F = I + ts*omega_bez*(A1 + A2*i_alpha + A3*i_beta +
%   A4*psi_alpha + A5*psi_beta + A6*omega). The copy of the state prediction x(k|k) = F*x + ts*omega_bez*B*u
%   has no omega_bez on the A6 path (as in inv_psm): F*x then has the right rotation and back-EMF terms plus
%   an extra 1/omega_bez (about 1 %), so the speed estimate is about 1 % low (see the README, open points).

    ctrl.ts = ts;
    ctrl.omega_base = omega_base;               % [rad/s] read by the encoder double integrator observer
    ctrl.Jm_norm = Jm_norm;
    ctrl.u_psm_scale = u_psm_scale;

    % predictive current control: weights of the cost function
    ctrl.mpc.k_kalman = g.k_kalman;             % current error (legacy name)
    ctrl.mpc.k_dc = g.k_dc;                     % neutral-point voltage
    ctrl.mpc.k_sw = g.k_sw;                     % change of the voltage vector (switching)

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

    % extended Kalman filter
    ctrl.ekf = ekf_param(mc.Rs_norm, mc.Ls_norm, g.jm_ekf, ts, g);

    fprintf('PSM control: predictive control weights k_kalman = %g, k_dc = %g, k_sw = %g | speed PI kp = %.2f, ki = %.0f, iq_lim = %.2f\n', ...
        ctrl.mpc.k_kalman, ctrl.mpc.k_dc, ctrl.mpc.k_sw, ctrl.kp_w, ctrl.ki_w, ctrl.iq_lim);
    fprintf('di_obs: poles %.0f and %.0f Hz, komega = %.0f, ktheta = %.0f | EKF: Rs = %.4f pu, Ls = %.4f pu, jm = %g\n', ...
        -ctrl.di_obs.poles(1)/2/pi, -ctrl.di_obs.poles(2)/2/pi, ctrl.di_obs.komega, ctrl.di_obs.ktheta, ...
        mc.Rs_norm, mc.Ls_norm, g.jm_ekf);
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

    % noise covariances (the legacy values "work pretty nice both generator and driver")
    e.Qkalman = ts*g.q_ekf*diag(g.q_ekf_diag);
    e.Rkalman = ts*g.r_ekf*eye(2);
end
