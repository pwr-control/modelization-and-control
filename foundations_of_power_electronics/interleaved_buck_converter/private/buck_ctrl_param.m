function ctrl = buck_ctrl_param(hw, ts, f_c_i, f_c_u)
%BUCK_CTRL_PARAM  Gains of the two loops of the interleaved buck from the crossover targets.
%
%   ctrl = buck_ctrl_param(hw, ts, f_c_i, f_c_u)
%     hw      param.buck.hw: udc_nom, ubez, ibez, L, RL, Cout, n_legs, pwr_nom
%     ts      [s]  control period (ts_afe: 1/(2*fpwm) with double update)
%     f_c_i   [Hz] crossover target of the per-leg current loops (PI zero at f_c_i/5)
%     f_c_u   [Hz] crossover target of the output voltage loop at nominal load (PI zero at f_c_u/5)
%
%   Fields: kp_i [pu/pu], ki_i [1/s], kp_u [pu/pu], ki_u [1/s] (the integrator of the blocks is
%   x(k+1) = x(k) + ki*ts*e(k), y = kp*e(k) + x(k+1), so the blocks read ki*ts) and the struct design with
%   the figures below.
%
%   Per-leg current loop (blocks dcdc_ctrl/buck_control/current_ctrl/pi/PId, one PI per leg on the leg current in
%   pu of ibez with the reference i_ref/n_legs): the plant from the duty d_k of leg k to the leg current, with the
%   output voltage as a disturbance, is L di_k/dt = d_k*udc - u_out - RL*i_k, i.e. in pu an integrator of gain
%   k_i = udc_nom/(L*ibez) [1/s per unit duty]. kp_i = 2*pi*f_c_i/k_i, ki_i = kp_i*2*pi*f_c_i/5.
%   Delays: leg 1 one step (the control uses the samples of the previous trigger) plus the ZOH of the duty, 1.5 ts;
%   legs 2 and 3 one more step (the sample is taken at their own carrier extreme, up to 2/3 ts before the control
%   step, and the duty is loaded at their next extreme, the two add up to one ts), 2.5 ts. Phase margin estimate
%   90 - atan(1/5) - 360*f_c_i*delay deg.
%   Output voltage loop (blocks voltage_ctrl/pi_u, output = total current reference in pu of ibez, limited at
%   +-i_lim): the plant seen by the voltage PI at s = j*w_u is T_i(s)*Zc(s), where Zc = k_u/(s + a) is the output
%   capacitor with the nominal resistive load (k_u = ibez/(Cout*ubez) [1/s per pu of current], a = 1/(R_load*Cout):
%   69 Hz at nominal load, so the plant is NOT an integrator below that) and T_i is the closed common-mode current
%   loop: with the same duty on all legs the output capacitor reacts and the leg current per unit duty is
%   k_i/(s + RL/L + n_legs*k_v*Zc), k_v = ubez/(L*ibez), well below the gain of the single inductor at the
%   frequencies of the voltage loop (|T_i| about 0.7 at a few tens of Hz with f_c_i = 300 Hz). kp_u from
%   |PI*T_i*Zc| = 1 at w_u, ki_u = kp_u*w_u/5; the crossover rises without load (plant k_u/s) up to about
%   kp_u*k_u*|T_i|.

    k_i = hw.udc_nom/(hw.L*hw.ibez);                 % [1/s] duty -> leg current (pu of ibez)
    k_u = hw.ibez/(hw.Cout*hw.ubez);                 % [1/s] total current (pu) -> u_out (pu of ubez), capacitor alone
    k_v = hw.ubez/(hw.L*hw.ibez);                    % [1/s] u_out (pu) -> slope of the leg current (pu)
    R_load = hw.ubez^2/hw.pwr_nom;                   % [Ohm] nominal resistive load
    a = 1/(R_load*hw.Cout);                          % [1/s] load corner
    w_i = 2*pi*f_c_i;
    w_u = 2*pi*f_c_u;

    % current loops
    ctrl.kp_i = w_i/k_i;                             % [pu/pu]
    ctrl.ki_i = ctrl.kp_i*w_i/5;                     % [1/s]  zero at f_c_i/5

    % voltage loop: plant at j*w_u = closed common-mode current loop x capacitor with the load
    s_u = 1i*w_u;
    Zc = k_u/(s_u + a);
    G_cm = k_i/(s_u + hw.RL/hw.L + hw.n_legs*k_v*Zc);              % leg current per unit duty, common duty
    L_cm = (ctrl.kp_i + ctrl.ki_i/s_u)*G_cm*exp(-1.5*ts*s_u);       % current loop of leg 1 (1.5 ts)
    T_i = L_cm/(1 + L_cm);
    ctrl.kp_u = 1/(abs(T_i*Zc)*sqrt(1 + 1/25));      % [pu/pu] |PI(j w_u)| = kp_u*sqrt(1 + 1/25) with the zero at w_u/5
    ctrl.ki_u = ctrl.kp_u*w_u/5;                     % [1/s]

    d.f_c_i = f_c_i;
    d.f_c_u = f_c_u;
    d.k_i = k_i;
    d.k_u = k_u;
    d.f_zero_i = ctrl.ki_i/ctrl.kp_i/2/pi;           % [Hz]
    d.f_zero_u = ctrl.ki_u/ctrl.kp_u/2/pi;           % [Hz]
    d.delay_leg1 = 1.5*ts;                           % [s]
    d.delay_leg23 = 2.5*ts;                          % [s]
    d.pm_leg1 = 90 - atand(1/5) - 360*f_c_i*d.delay_leg1;      % [deg] estimate
    d.pm_leg23 = 90 - atand(1/5) - 360*f_c_i*d.delay_leg23;    % [deg]
    d.f_load = a/2/pi;                               % [Hz] load corner of the voltage plant at nominal load
    d.T_i_at_f_c_u = abs(T_i);                       % inner loop gain at the voltage crossover
    d.pm_u = 180 + angle((ctrl.kp_u + ctrl.ki_u/s_u)*T_i*Zc)*180/pi;   % [deg] estimate at nominal load
    d.f_c_u_noload = ctrl.kp_u*k_u*abs(T_i)/2/pi;    % [Hz] estimate without load
    d.f_nyquist = 1/(2*ts);                          % [Hz]
    ctrl.design = d;

    fprintf('Current loop (per leg): kp_i = %.3f pu/pu, ki_i = %.0f 1/s -> crossover %.0f Hz, PI zero %.0f Hz, phase margin about %.0f deg (leg 1, 1.5 ts) / %.0f deg (legs 2-3, 2.5 ts)\n', ...
        ctrl.kp_i, ctrl.ki_i, f_c_i, d.f_zero_i, d.pm_leg1, d.pm_leg23);
    fprintf('Voltage loop: kp_u = %.3f pu/pu, ki_u = %.1f 1/s -> crossover %.0f Hz at nominal load (load corner %.0f Hz, inner loop gain %.2f there), phase margin about %.0f deg, PI zero %.0f Hz, about %.0f Hz without load\n', ...
        ctrl.kp_u, ctrl.ki_u, f_c_u, d.f_load, d.T_i_at_f_c_u, d.pm_u, d.f_zero_u, d.f_c_u_noload);
    if f_c_i > d.f_nyquist/10
        warning('buck_ctrl_param: current loop crossover %.0f Hz above fs/20 = %.0f Hz, the delays leave little margin', f_c_i, d.f_nyquist/10);
    end
    if d.f_c_u_noload > f_c_i/2.5
        warning('buck_ctrl_param: voltage loop crossover without load (%.0f Hz) less than a factor 2.5 below the current loops (%.0f Hz)', d.f_c_u_noload, f_c_i);
    end
end
