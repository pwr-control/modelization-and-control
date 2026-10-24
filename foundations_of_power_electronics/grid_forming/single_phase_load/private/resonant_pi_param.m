function res = resonant_pi_param(ts, omega_res, ki_rpi, delta_res)
%RESONANT_PI_PARAM  Resonant term of the dq current controller of grid_forming_dq (local replacement of the
%   resonant_pi method of ctrl_afe_setup of the library, with an exact discretization).
%
%   res = resonant_pi_param(ts, omega_res, ki_rpi, delta_res)
%     ts        : [s] sample time of the blocks (rate of the triggered subsystem afe_control: ts_afe)
%     omega_res : [rad/s] resonance pulsation (2*omega_grid: negative sequence seen from the positive dq frame)
%     ki_rpi    : [1/s] resonant gain kr
%     delta_res : [-] damping of the resonant model (0: ideal resonant, discrete poles on the unit circle)
%
%   Blocks current_ctrl/dq_vector_pi/pi_d/rpi_phase_d and pi_q/rpi_phase_q: per axis, in parallel with the PI,
%     x(k+1) = Aresd*x(k) + Bresd*ki_rpi*e(k),  y(k) = Cresd*x(k)
%   Fields read by grid_forming_dq.slx: ki_rpi, Aresd, Bresd, Cresd.
%
%   Continuous-time term  R(s) = ki_rpi*s/(s^2 + 2*delta_res*omega_res*s + omega_res^2), state space
%     Ares = [0 1; -omega_res^2 -2*delta_res*omega_res], Bres = [0; 1], Cres = [0 1]  (same model as the library).
%   Discretization: exact (zero-order hold on e, matrix exponential of [Ares Bres; 0 0]*ts). The discrete poles
%   are exp(ts*eig(Ares)): with delta_res = 0 they sit on the unit circle at +/- omega_res*ts, the resonance is
%   exactly at omega_res (same poles as the C implementation of grid_forming_ctrl_ver_ii, a = 2*sin(omega_res*ts/2)).
%   The forward Euler form of the library (Aresd = eye(2) + Ares*ts) is not used on purpose: its poles have
%   modulus sqrt((1 - 2*delta*w*ts)^2 + (w*ts)^2) > 1 unless delta > w*ts/2, i.e. the resonator is unstable by
%   itself (at 100 Hz and ts = 125 us: |z| = 1.0031 with delta = 0, 1.0019 with delta = 0.015, +15 ... +25 1/s)
%   and the current loop, which damps the +2*omega_grid mode by a few 1/s only, cannot recover it (see the
%   linear check in init_model).

    res.ts = ts;                                            % [s]
    res.omega_res = omega_res;                              % [rad/s]
    res.f_res = omega_res/2/pi;                             % [Hz]
    res.ki_rpi = ki_rpi;                                    % [1/s]
    res.delta_res = delta_res;                              % [-]

    % continuous-time model (per axis)
    res.Ares = [0 1; -omega_res^2 -2*delta_res*omega_res];
    res.Bres = [0; 1];
    res.Cres = [0 1];

    % exact discretization (ZOH on the input)
    M = expm([res.Ares res.Bres; 0 0 0]*ts);
    res.Aresd = M(1:2, 1:2);
    res.Bresd = M(1:2, 3);
    res.Cresd = res.Cres;

    % checks (not read by the model)
    res.polesd = eig(res.Aresd);                            % exp(ts*eig(Ares))
    res.polesd_abs = max(abs(res.polesd));                  % 1 for delta_res = 0
    res.f_polesd = max(abs(angle(res.polesd)))/ts/2/pi;     % [Hz] resonance of the discrete term
    res.polesd_abs_fwd_euler = max(abs(eig(eye(2) + res.Ares*ts)));    % library form, for comparison
    if delta_res > 0
        res.peak_gain = ki_rpi/(2*delta_res*omega_res);     % [pu/pu] |R(j*omega_res)|
    else
        res.peak_gain = inf;
    end

    fprintf(['Resonant term: f_res = %.1f Hz, ki_rpi = %.0f 1/s, delta_res = %.3f: discrete poles |z| = %.6f at ' ...
        '%.2f Hz, peak gain %.1f (forward Euler would give |z| = %.4f, unstable)\n'], ...
        res.f_res, res.ki_rpi, res.delta_res, res.polesd_abs, res.f_polesd, res.peak_gain, res.polesd_abs_fwd_euler);
end
