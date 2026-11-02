function ctrl = afe_ctrl_param(ts, omega_base, g)
%AFE_CTRL_PARAM  Control data of the AFE of afe_inv_psm_n.slx, the same for both modules (local replacement of
%   ctrl_afe_setup of the library for the fields the model reads, plus the gains the legacy init set after
%   the setup). The field names are those of ctrl_afe_setup, so the model reads param.afe.ctrl.<library name>.
%
%   ctrl = afe_ctrl_param(ts, omega_base, g)
%     ts         : [s] control sample time (ts_afe)
%     omega_base : [rad/s] nominal grid pulsation (PLLs, first harmonic trackers, resonant controllers,
%                  observers, rms windows)
%     g          : gains and limits set in init_model.m (copied in ctrl):
%                  udc_norm_ref, kp_udc_ctrl, ki_udc_ctrl
%                        DC-link voltage PI (udc_ctrl): reference in pu of udc_nom, output = active (xi)
%                        current reference
%                  kp_rpi, ki_rpi, delta_res
%                        proportional-resonant controller of the converter current (current_ctrl/
%                        inner_current_ctrl, one per phase): resonant part at omega_base
%                  kp_rc_pos_grid, ki_rc_pos_grid, kp_rc_neg_grid, ki_rc_neg_grid
%                        PI of the grid current sequences (grid_current_sequences_and_reactive_pwr_ctrl):
%                        positive sequence eta (reactive current), negative sequence xi and eta
%                  kp_is_z, ki_is_z
%                        zero-sequence current PI (zero_current_ctrl): its output reaches the modulator through
%                        param.afe.behav.u_cm_comp_mod1/2
%                  pll_p, pll_i1, pll_p_frt
%                        PI of the dq PLLs (the FHT-PLLs read pll_p_fht = pll_p, pll_i1_fht = pll_i1, as the
%                        library)
%                  ixi_pos_ref_lim, ieta_pos_ref_lim, ieta_neg_ref_lim, i_grid_pos_eta_lim, i_grid_neg_xi_lim,
%                  i_grid_neg_eta_lim, i_grid_pos_eta_red_lim, i_grid_neg_eta_red_lim, i_grid_neg_xi_red_lim
%                        [pu] limits of the current references (normal and reduced during FRT)
%                  fht.delta, fht.poles, di_obs.poles, vr_obs.poles
%                        damping of the resonant model of the first harmonic tracker and the observer poles
%                        (normalized to omega_base)
%
%   Fields read by afe_inv_psm_n.slx (param.afe.ctrl.*): the gains and limits above (not delta_res and the
%   poles) and
%     omega_base
%     res_pi.Aresd, res_pi.Bresd, res_pi.Cresd, res_pi.kp_rpi, res_pi.ki_rpi
%                       resonant controller in discrete state space (forward Euler, as the library)
%     fht_grid.omega_fht, fht_grid.delta_fht, fht_grid.Ld_fht, fht_grid.Ad_fht, fht_grid.Cfht
%                       first harmonic trackers of the FHT-PLLs (stage A on the alpha and beta grid voltages)
%     di_obs.Adso, di_obs.Cso, di_obs.Ldso
%                       double integrator observer of the phase (stage B of the PLLs)
%     vr_obs.kv, vr_obs.kx
%                       voltage rate observer (C-Caller form)
%     rms.n1, rms.n10   samples in 1 and 10 grid periods
%
%   The observers are the same as in the library: double integrator A = [0 1; 0 0] and resonant model
%   A = [0 1; -w^2 -delta*w], C = [1 0], discretized with forward Euler, poles p*omega_base mapped with
%   exp(p*ts). The gains are computed in closed form (two states), which gives the same result as acker on
%   the transposed pair.

    ctrl.ts = ts;
    ctrl.omega_base = omega_base;
    ctrl.omega_diobs = omega_base;

    % DC-link voltage PI (output = active current reference, saturated at +-ixi_pos_ref_lim, anti-windup by
    % back-calculation)
    ctrl.udc_norm_ref = g.udc_norm_ref;         % [pu] of udc_nom
    ctrl.kp_udc_ctrl = g.kp_udc_ctrl;           % [pu/pu]
    ctrl.ki_udc_ctrl = g.ki_udc_ctrl;           % [1/s]

    % proportional-resonant controller of the converter current: u = kp*e + ki*r, with r the output of the
    % resonator s/(s^2 + 2*delta*w*s + w^2) driven by e; discrete state space with forward Euler (library
    % resonant_pi: Aresd = I + Ares*ts, Bresd = Bres*ts, Cresd = Cres)
    ctrl.res_pi = resonant_pi_param(ts, omega_base, g.kp_rpi, g.ki_rpi, g.delta_res);

    % PI of the grid current sequences
    ctrl.kp_rc_pos_grid = g.kp_rc_pos_grid;     % [pu/pu]
    ctrl.ki_rc_pos_grid = g.ki_rc_pos_grid;     % [1/s]
    ctrl.kp_rc_neg_grid = g.kp_rc_neg_grid;
    ctrl.ki_rc_neg_grid = g.ki_rc_neg_grid;

    % zero-sequence current PI
    ctrl.kp_is_z = g.kp_is_z;                   % [pu/pu]
    ctrl.ki_is_z = g.ki_is_z;                   % [1/s]

    % PLLs: dq PLL (pll_p, pll_i1, pll_p_frt) and FHT-PLLs (pll_p_fht, pll_i1_fht, same values as the library)
    ctrl.pll_p = g.pll_p;
    ctrl.pll_i1 = g.pll_i1;                     % [1/s]
    ctrl.pll_p_frt = g.pll_p_frt;
    ctrl.pll_p_fht = g.pll_p;
    ctrl.pll_i1_fht = g.pll_i1;                 % [1/s]

    % limits of the current references [pu of ibez]
    ctrl.ixi_pos_ref_lim = g.ixi_pos_ref_lim;
    ctrl.ieta_pos_ref_lim = g.ieta_pos_ref_lim;
    ctrl.ieta_neg_ref_lim = g.ieta_neg_ref_lim;
    ctrl.i_grid_pos_eta_lim = g.i_grid_pos_eta_lim;
    ctrl.i_grid_neg_xi_lim = g.i_grid_neg_xi_lim;
    ctrl.i_grid_neg_eta_lim = g.i_grid_neg_eta_lim;
    ctrl.i_grid_pos_eta_red_lim = g.i_grid_pos_eta_red_lim;
    ctrl.i_grid_neg_eta_red_lim = g.i_grid_neg_eta_red_lim;
    ctrl.i_grid_neg_xi_red_lim = g.i_grid_neg_xi_red_lim;

    % first harmonic tracker of the grid voltages (stage A of the FHT-PLLs)
    ctrl.fht_grid = fht_observer_param(ts, omega_base, g.fht.delta, g.fht.poles);

    % double integrator observer of the phase (stage B of the PLLs) and voltage rate observer
    ctrl.di_obs = double_integrator_observer_param(ts, ctrl.omega_diobs, g.di_obs.poles);
    vr = double_integrator_observer_param(ts, ctrl.omega_diobs, g.vr_obs.poles);
    ctrl.vr_obs.Aso = vr.Aso;
    ctrl.vr_obs.Adso = vr.Adso;
    ctrl.vr_obs.Cso = vr.Cso;
    ctrl.vr_obs.Ldso = vr.Ldso;
    ctrl.vr_obs.kv = vr.Ldso(2)/ts;
    ctrl.vr_obs.kx = vr.Ldso(1)/ts;

    % number of samples in 1 and 10 grid periods
    ctrl.rms.n1 = 2*pi*1/omega_base/ts;
    ctrl.rms.n10 = 2*pi*10/omega_base/ts;

    fprintf('AFE control: udc PI kp = %.2f, ki = %.0f | PR current kp = %.2f, ki = %.0f, delta = %.3f | sequence PI kp = %.2f, ki = %.0f | zero-sequence PI kp = %.2f, ki = %.0f | PLL kp = %.1f, ki = %.0f, kp_frt = %.1f\n', ...
        ctrl.kp_udc_ctrl, ctrl.ki_udc_ctrl, ctrl.res_pi.kp_rpi, ctrl.res_pi.ki_rpi, ctrl.res_pi.delta_res, ...
        ctrl.kp_rc_pos_grid, ctrl.ki_rc_pos_grid, ctrl.kp_is_z, ctrl.ki_is_z, ctrl.pll_p, ctrl.pll_i1, ctrl.pll_p_frt);
    fprintf('FHT: omega = %.1f rad/s, delta = %.2f, Ld = [%.4f %.4f] | di_obs: Ld = [%.4f %.6f] | vr_obs: kx = %.1f, kv = %.1f | rms windows: %.0f and %.0f samples\n', ...
        ctrl.fht_grid.omega_fht, ctrl.fht_grid.delta_fht, ctrl.fht_grid.Ld_fht(1), ctrl.fht_grid.Ld_fht(2), ...
        ctrl.di_obs.Ldso(1), ctrl.di_obs.Ldso(2), ctrl.vr_obs.kx, ctrl.vr_obs.kv, ctrl.rms.n1, ctrl.rms.n10);
end

function r = resonant_pi_param(ts, omega, kp, ki, delta)
    r.kp_rpi = kp;                              % [pu/pu]
    r.ki_rpi = ki;                              % [1/s]
    r.delta_res = delta;                        % [-] damping of the resonator
    r.Ares = [0 1; -omega^2 -2*delta*omega];
    r.Bres = [0; 1];
    r.Cres = [0 1];
    r.Aresd = eye(2) + r.Ares*ts;               % forward Euler
    r.Bresd = r.Bres*ts;
    r.Cresd = r.Cres;
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
