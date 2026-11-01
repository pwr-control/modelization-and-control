function ctrl = rect_ctrl_param(ts, omega_nom, c)
%RECT_CTRL_PARAM  Control data of the six-pulse thyristor rectifier (local, the numbers of the legacy init
%   and the literals typed in the blocks of six_pulse_thyristor_based_rectifier.slx).
%
%   ctrl = rect_ctrl_param(ts, omega_nom, c)
%     ts         [s] control step (param.glb_time.ts_rect)
%     omega_nom  [rad/s] nominal grid pulsation
%     c          numbers of the init (ctrl_in):
%       c.pi_i.kp, c.pi_i.ki    load current PI (block PI_i): u = cos(alpha) = kp*e + x, x += ki*ts*e, e in pu
%                               of I_FS; the block named "1/Ti" is kp, the block named "kp" is ki*ts
%       c.pi_v.kp, c.pi_v.ki    output voltage PI (block PI_v, same structure, e in pu of V_FS; its output is
%                               not selected by Manual Switch, so it is not used)
%       c.u_min, c.u_max        clamp of the PI output cos(alpha) (Saturation1 of PI_i and PI_v); the
%                               integrator follows the clamp (x = sat(v) - kp*e)
%       c.alpha_min, c.alpha_max  [rad] clamp of the firing angle after acos (Saturation of PI_i and PI_v)
%       c.iref_1, c.iref_2      [A] load current reference before and after the step (block Step)
%       c.t_iref_step           [s] time of the reference step
%       c.vref                  [V] output voltage reference of PI_v (block Step1)
%       c.t_pulse_release       [s] end of the pulse block of the firing generator (Step in six_pulse
%                               rectifier, input Block of sprcProcessSimulink)
%       c.t_pll_reset           [s] end of the reset of the C-Caller PLL dqpll_thyr (Step inside
%                               dqpll_thyr_ccaller)
%       c.pll.kp, c.pll.ki      Simulink PLL of pll_stageB (comparison only: its phase goes to a scope)
%       c.fht.delta, c.fht.pole1_factor, c.fht.pole2_factor
%                               first-harmonic observers on the three measured voltages (library block
%                               first_harmonic_tracker/harmonic_observer): damping of the model, observer
%                               poles at -pole1_factor*omega_nom and -pole2_factor*omega_nom
%
%   Fields read by the model: pi_i.kp, pi_i.ki, pi_v.kp, pi_v.ki, u_min, u_max, alpha_min, alpha_max,
%   iref_1, iref_2, t_iref_step, vref, t_pulse_release, t_pll_reset, pll.kp, pll.ki, fht.omega_base,
%   fht.delta, fht.pole1, fht.pole2.
%   Fixed in the C sources of the library, not in param (informative copies in ctrl.c_code): gains of the
%   C-Caller PLL dqpll_thyr (KP_DQPLL_THYR 0.25, KI1_DQPLL_THYR 80, KI2_DQPLL_THYR 314.159 = omega base;
%   after a reset the PLL restarts from omega = 0 and gamma = 0) and the pulse width of the firing
%   generator six_pulse_rect (PWIDTH_PU 1.9 rad = 109 deg).

    ctrl.ts = ts;

    ctrl.pi_i.kp = c.pi_i.kp;                       % [pu/pu]
    ctrl.pi_i.ki = c.pi_i.ki;                       % [1/s]
    ctrl.pi_v.kp = c.pi_v.kp;
    ctrl.pi_v.ki = c.pi_v.ki;
    ctrl.u_min = c.u_min;                           % [-] cos(alpha)
    ctrl.u_max = c.u_max;
    ctrl.alpha_min = c.alpha_min;                   % [rad]
    ctrl.alpha_max = c.alpha_max;

    ctrl.iref_1 = c.iref_1;                         % [A]
    ctrl.iref_2 = c.iref_2;                         % [A]
    ctrl.t_iref_step = c.t_iref_step;               % [s]
    ctrl.vref = c.vref;                             % [V]
    ctrl.t_pulse_release = c.t_pulse_release;       % [s]
    ctrl.t_pll_reset = c.t_pll_reset;               % [s]

    ctrl.pll.kp = c.pll.kp;
    ctrl.pll.ki = c.pll.ki;                         % [1/s]

    ctrl.fht.omega_base = omega_nom;                % [rad/s]
    ctrl.fht.delta = c.fht.delta;
    ctrl.fht.pole1 = c.fht.pole1_factor*omega_nom;  % [rad/s]
    ctrl.fht.pole2 = c.fht.pole2_factor*omega_nom;  % [rad/s]

    % constants of the C sources (informative, see the header)
    ctrl.c_code.dqpll_thyr_kp = 0.25;
    ctrl.c_code.dqpll_thyr_ki1 = 80;                % [1/s]
    ctrl.c_code.dqpll_thyr_ki2 = 314.159;           % [rad/s]
    ctrl.c_code.pulse_width = 1.9;                  % [rad]

    fprintf('Control: current PI kp %.3g, ki %.3g 1/s | cos(alpha) in [%.3f, %.3f], alpha in [%.1f, %.1f] deg | Iref %.0f A -> %.0f A at %.2f s | pulses released at %.3f s, PLL reset until %.3f s\n', ...
        ctrl.pi_i.kp, ctrl.pi_i.ki, ctrl.u_min, ctrl.u_max, ctrl.alpha_min*180/pi, ctrl.alpha_max*180/pi, ...
        ctrl.iref_1, ctrl.iref_2, ctrl.t_iref_step, ctrl.t_pulse_release, ctrl.t_pll_reset);
end
