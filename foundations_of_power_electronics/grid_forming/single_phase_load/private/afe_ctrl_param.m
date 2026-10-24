function ctrl = afe_ctrl_param(ts, omega_base)
%AFE_CTRL_PARAM  Control data of the AFE read by grid_forming_dq.slx (local replacement of ctrl_afe_setup).
%
%   ctrl = afe_ctrl_param(ts, omega_base)
%     ts         : [s] control sample time (ts_afe)
%     omega_base : [rad/s] nominal grid pulsation
%
%   Fields read by grid_forming_dq.slx:
%     fht_grid.omega_fht, fht_grid.delta_fht, fht_grid.Ld_fht(1), fht_grid.Ld_fht(2)
%                       first harmonic tracker of the grid voltage (C-Caller first_harmonic_tracker)
%     ixi_pos_ref_lim   [pu] saturation of the current reference of the voltage magnitude loop
%     kp_udc_ctrl, ki_udc_ctrl
%                       gains of the legacy |ug|^2 magnitude loop (block u_grid_mag_ctrl, commented out);
%                       the dq voltage loop gains kp_ug_ctrl, ki_ug_ctrl are set in init_model.
%
%   The FHT is the same observer as in the library: A = [0 1; -w^2 -delta*w], C = [1 0], discretized
%   with forward Euler, observer poles at [-1 -4]*omega_base mapped with exp(p*ts). The gains are computed
%   in closed form (two states), which gives the same result as acker on the transposed pair.

    ctrl.ts = ts;
    ctrl.omega_base = omega_base;

    % first harmonic tracker of the grid voltage
    ctrl.fht_grid = fht_observer_param(ts, omega_base, 0.05, [-1 -4]);

    % limits of the voltage magnitude loop (same value as the library: ixi_pos_ref_lim = 1.65)
    ctrl.ixi_pos_ref_lim = 1.65;                % [pu]

    % gains of the legacy |ug|^2 magnitude loop (block afe_control/u_grid_mag_ctrl, commented out in the model
    % since 2026-10-21 and replaced by the dq voltage loop kp_ug_ctrl/ki_ug_ctrl set in init_model). The block
    % still reads these names: kept at the legacy values so that nothing resolves to NaN if it is re-enabled.
    ctrl.kp_udc_ctrl = 4;
    ctrl.ki_udc_ctrl = 45;                      % [1/s]
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

function L = observer_gain_2x2(A, p)
% Observer gain L (2x1) such that eig(A - L*C) = p, with C = [1 0]; closed-form Ackermann for two states:
% det(zI - (A - L*C)) = z^2 - (a11 - l1 + a22)*z + (a11 - l1)*a22 - a12*(a21 - l2)
    a1 = -(p(1) + p(2));
    a0 = p(1)*p(2);
    l1 = A(1,1) + A(2,2) + a1;
    l2 = A(2,1) + (a0 - (A(1,1) - l1)*A(2,2))/A(1,2);
    L = [l1; l2];
end
