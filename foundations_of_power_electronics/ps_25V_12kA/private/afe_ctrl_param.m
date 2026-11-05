function ctrl = afe_ctrl_param(ts, omega_base, g)
%AFE_CTRL_PARAM  Control data of the AFE of ps_25V_12kA_r.slx (local replacement of ctrl_afe_setup of the
%   library for the fields the model and the section "AFE control in uvw" of the init read; the field names
%   are those of the library class).
%
%   ctrl = afe_ctrl_param(ts, omega_base, g)
%     ts         : [s] control sample time (ts_afe)
%     omega_base : [rad/s] nominal grid pulsation (resonant PI, PLL, first harmonic tracker, observers)
%     g          : gains set in init_model.m (copied in ctrl):
%                  udc_norm_ref, kp_udc_ctrl, ki_udc_ctrl, ixi_pos_ref_lim
%                                    DC-link voltage PI (block udc_ctrl): output = active (xi) current
%                                    reference, saturated at +-ixi_pos_ref_lim
%                  kp_rpi, ki_rpi    resonant PI per phase of the converter current, kp + ki*s/(s^2 + w^2)
%                                    (block current_ctrl_uvw, through param.afe.uvw.ictrl; field res_pi)
%                  kp_is_z, ki_is_z  PI of the zero-sequence converter current (current_ctrl_uvw)
%                  kp_rc_pos_grid, ki_rc_pos_grid, kp_rc_neg_grid, ki_rc_neg_grid, ieta_pos_ref_lim,
%                  ieta_neg_ref_lim  PIs of the grid current sequences: positive-sequence eta and
%                                    negative-sequence xi/eta (block grid_current_sequences_q_ctrl), outputs
%                                    limited at ieta_pos_ref_lim / ieta_neg_ref_lim
%                  i_grid_pos_eta_lim, i_grid_neg_xi_lim, i_grid_neg_eta_lim, and the same with _red_lim
%                                    limits of the grid current references of the FRT control (block
%                                    frt_vde4110; _red_lim with reduced dynamic grid support)
%                  pll_p, pll_i1     PI of the FHT-PLL in Simulink (fields pll_p_fht, pll_i1_fht)
%
%   Fields read by ps_25V_12kA_r.slx (param.afe.ctrl.*): udc_norm_ref, kp_udc_ctrl, ki_udc_ctrl,
%   ixi_pos_ref_lim, pll_p_fht, pll_i1_fht and
%     fht_grid.omega_fht, fht_grid.Ld_fht, fht_grid.Ad_fht, fht_grid.Cfht
%                       first harmonic tracker of the FHT-PLL (stage A on the alpha and beta grid voltages)
%     di_obs.Adso, di_obs.Cso, di_obs.Ldso
%                       double integrator observer of the phase (stage B of the PLL)
%   instantaneus_power (commented out in the model) reads rms.n1 and rms.n10. The section "AFE control in
%   uvw" of the init reads res_pi.kp_rpi, res_pi.ki_rpi, kp_is_z, ki_is_z, omega_base, the sequence PIs and
%   limits, the FRT limits and vr_obs.kx, vr_obs.kv (voltage-rate observers of frt_vde4110).
%
%   The observers are the same as in the library: double integrator A = [0 1; 0 0] (di_obs with poles
%   [-1 -4]*omega_base, vr_obs with [-0.1 -0.4]*omega_base) and resonant model A = [0 1; -w^2 -delta*w],
%   C = [1 0] (fht_grid, poles [-1 -4]*omega_base), discretized with forward Euler, poles mapped with
%   exp(p*ts). The gains are computed in closed form (two states), which gives the same result as acker on
%   the transposed pair and does not need the Control System Toolbox.

    ctrl.ts = ts;
    ctrl.omega_base = omega_base;
    ctrl.omega_diobs = omega_base;

    % DC-link voltage PI: output = active (xi) current reference, saturated at +-ixi_pos_ref_lim
    ctrl.udc_norm_ref = g.udc_norm_ref;         % [pu] of udc_nom
    ctrl.kp_udc_ctrl = g.kp_udc_ctrl;           % [pu/pu]
    ctrl.ki_udc_ctrl = g.ki_udc_ctrl;           % [1/s]
    ctrl.ixi_pos_ref_lim = g.ixi_pos_ref_lim;   % [pu] of ibez

    % converter current: resonant PI per phase (kp + ki*s/(s^2 + w^2), the damping is set in the uvw section)
    % and zero-sequence PI
    ctrl.res_pi.kp_rpi = g.kp_rpi;              % [pu/pu]
    ctrl.res_pi.ki_rpi = g.ki_rpi;              % [1/s]
    ctrl.kp_is_z = g.kp_is_z;                   % [pu/pu]
    ctrl.ki_is_z = g.ki_is_z;                   % [1/s]

    % grid current sequences: PI of the positive-sequence eta current and of the negative-sequence xi/eta
    % currents, output limits (stack current references)
    ctrl.kp_rc_pos_grid = g.kp_rc_pos_grid;     % [pu/pu]
    ctrl.ki_rc_pos_grid = g.ki_rc_pos_grid;     % [1/s]
    ctrl.kp_rc_neg_grid = g.kp_rc_neg_grid;
    ctrl.ki_rc_neg_grid = g.ki_rc_neg_grid;
    ctrl.ieta_pos_ref_lim = g.ieta_pos_ref_lim; % [pu] of ibez
    ctrl.ieta_neg_ref_lim = g.ieta_neg_ref_lim;

    % FRT: limits of the grid current references (normal and reduced dynamic grid support)
    ctrl.i_grid_pos_eta_lim = g.i_grid_pos_eta_lim;         % [pu]
    ctrl.i_grid_neg_xi_lim = g.i_grid_neg_xi_lim;
    ctrl.i_grid_neg_eta_lim = g.i_grid_neg_eta_lim;
    ctrl.i_grid_pos_eta_red_lim = g.i_grid_pos_eta_red_lim;
    ctrl.i_grid_neg_xi_red_lim = g.i_grid_neg_xi_red_lim;
    ctrl.i_grid_neg_eta_red_lim = g.i_grid_neg_eta_red_lim;

    % FHT-PLL (Simulink): PI of the phase loop
    ctrl.pll_p_fht = g.pll_p;
    ctrl.pll_i1_fht = g.pll_i1;                 % [1/s]

    % first harmonic tracker of the grid voltages (stage A of the FHT-PLL)
    ctrl.fht_grid = fht_observer_param(ts, omega_base, 0.05, [-1 -4]);

    % double integrator observer of the phase (stage B of the PLL)
    ctrl.di_obs = double_integrator_observer_param(ts, omega_base, [-1 -4]);
    ctrl.di_obs.ktheta = ctrl.di_obs.Ldso(1)/ts;    % gains of the form x' = v + kx*(u - x), v' = kv*(u - x)
    ctrl.di_obs.komega = ctrl.di_obs.Ldso(2)/ts;

    % voltage-rate observers of the FRT control (positive and negative sequence amplitudes)
    ctrl.vr_obs = double_integrator_observer_param(ts, omega_base, [-0.1 -0.4]);
    ctrl.vr_obs.kx = ctrl.vr_obs.Ldso(1)/ts;
    ctrl.vr_obs.kv = ctrl.vr_obs.Ldso(2)/ts;

    % number of samples in 1 and 10 grid periods (instantaneus_power, commented out in the model)
    ctrl.rms.n1 = 2*pi*1/omega_base/ts;
    ctrl.rms.n10 = 2*pi*10/omega_base/ts;

    fprintf('AFE control: udc PI kp = %.2f, ki = %.0f, ixi limit = %.2f | resonant current PI kp = %.2f, ki = %.0f, zero sequence kp = %.2f, ki = %.0f | sequence PIs pos %.2f/%.0f, neg %.2f/%.0f, limits %.2f/%.2f | PLL kp = %.1f, ki = %.0f\n', ...
        ctrl.kp_udc_ctrl, ctrl.ki_udc_ctrl, ctrl.ixi_pos_ref_lim, ctrl.res_pi.kp_rpi, ctrl.res_pi.ki_rpi, ...
        ctrl.kp_is_z, ctrl.ki_is_z, ctrl.kp_rc_pos_grid, ctrl.ki_rc_pos_grid, ctrl.kp_rc_neg_grid, ...
        ctrl.ki_rc_neg_grid, ctrl.ieta_pos_ref_lim, ctrl.ieta_neg_ref_lim, ctrl.pll_p_fht, ctrl.pll_i1_fht);
    fprintf('FHT: omega = %.1f rad/s, delta = %.2f, Ld = [%.4f %.4f] | di_obs: Ld = [%.4f %.6f] | vr_obs: kx = %.2f, kv = %.2f\n', ...
        ctrl.fht_grid.omega_fht, ctrl.fht_grid.delta_fht, ctrl.fht_grid.Ld_fht(1), ctrl.fht_grid.Ld_fht(2), ...
        ctrl.di_obs.Ldso(1), ctrl.di_obs.Ldso(2), ctrl.vr_obs.kx, ctrl.vr_obs.kv);
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
