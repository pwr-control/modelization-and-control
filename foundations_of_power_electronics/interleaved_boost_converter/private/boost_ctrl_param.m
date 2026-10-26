function ctrl = boost_ctrl_param(hw, ts, f_c_i, f_c_u)
%BOOST_CTRL_PARAM  Gains of the two loops of the interleaved boost from the crossover targets.
%
%   ctrl = boost_ctrl_param(hw, ts, f_c_i, f_c_u)
%     hw      param.boost.hw: uin_nom, udc_nom, ibez, L, RL, Cdc, n_legs, pwr_nom
%     ts      [s]  control period (ts_afe: 1/(2*fpwm) with double update)
%     f_c_i   [Hz] crossover target of the per-leg current loops (PI zero at f_c_i/5)
%     f_c_u   [Hz] crossover target of the DC-link voltage loop at nominal load (PI zero at f_c_u/5)
%
%   Fields: kp_i [pu/pu], ki_i [1/s], kp_u [pu/pu], ki_u [1/s] (the integrator of the blocks is
%   x(k+1) = x(k) + ki*ts*e(k), y = kp*e(k) + x(k+1), so the blocks read ki*ts) and the struct design with
%   the figures below. Same closed-form design as buck_ctrl_param of the interleaved buck, with the boost plant.
%
%   Conventions: leg currents positive from the battery into the legs (boost direction), d_k = duty of the LOWER
%   switch of leg k (the boost switch; the modulator gets m_k = 1 - d_k). Operating point at nominal load:
%   i_leg = pwr_nom/(n_legs*uin_nom), M = 1 - D = (uin_nom - RL*i_leg)/udc_nom.
%
%   Per-leg current loop (blocks dcdc_ctrl/boost_control/current_ctrl/pi/PId, one PI per leg on the leg current in
%   pu of ibez with the reference i_ref/n_legs): L di_k/dt = uin - RL*i_k - (1 - d_k)*udc, so the plant from d_k to
%   the leg current, with udc as a disturbance, is in pu the integrator k_i = udc_nom/(L*ibez) [1/s per unit duty],
%   the same as the buck (same hardware, same voltages). kp_i = 2*pi*f_c_i/k_i, ki_i = kp_i*2*pi*f_c_i/5. Delays:
%   leg 1 one step (the control uses the samples of the previous trigger) plus the ZOH of the duty, 1.5 ts; legs 2
%   and 3 one more step (sample at their own carrier extreme up to 2/3 ts earlier, duty loaded at their next extreme),
%   2.5 ts. Phase margin estimate 90 - atan(1/5) - 360*f_c_i*delay deg. With the same duty on the three legs the DC
%   link reacts (common mode): the plant becomes
%       G_cm = [k_i*(s + a1) + n*b*e]/[(s + RL/L)*(s + a1) + n*b*c],
%   a1 = 1/(R_load*Cdc), b = M*udc_nom/(L*ibez), c = M*ibez/(Cdc*udc_nom), e = i_leg/(Cdc*udc_nom), with the
%   resonance f0 = M*sqrt(n/(L*Cdc))/(2*pi) (35 Hz), which must stay inside the current loops (f0 <= f_c_i/2) so that
%   the loops damp it at any load.
%   DC-link voltage loop (blocks voltage_ctrl/pi_u, output = total input current reference in pu of ibez, limited
%   at +-i_lim): with the current loops closed the DC-link current of a leg is (1 - d_k)*i_k and a current increase
%   is obtained by first increasing d_k, so the plant from the total current to udc (pu) is
%       G_ui = k_u*(1 - s/w_z)/(s + 2*a1),   k_u = ibez*(M - i_leg*RL/udc_nom)/(Cdc*udc_nom),
%       w_z = (uin_nom - 2*RL*i_leg)/(L*i_leg)  (right-half-plane zero, 157 Hz at nominal load, as 1/i_leg at
%   lighter load), 2*a1 because at constant input current the DC-link current falls as 1/udc (power balance).
%   The crossover must stay below f_z/4: the gain of the loop does not roll off above the zero (the zero cancels
%   the pole), so the gain margin is set by kp_u*k_u/w_z. The plant seen by the voltage PI at s = j*w_u is
%   T_i*(c - e/G_cm)/(s + a1), T_i the closed common-mode current loop of leg 1 (1.5 ts), the same expression as
%   G_ui once the loop is ideal. kp_u from |PI*plant| = 1 at w_u, ki_u = kp_u*w_u/5; without load the zero
%   disappears and the plant is the integrator k_u/s: the crossover stays at about kp_u*k_u/(2*pi).

    n = hw.n_legs;
    i_leg = hw.pwr_nom/(n*hw.uin_nom);               % [A] per leg at nominal load (lossless first guess, then with RL)
    for it = 1:20
        i_leg = hw.pwr_nom/(n*(hw.uin_nom - hw.RL*i_leg));
    end
    M = (hw.uin_nom - hw.RL*i_leg)/hw.udc_nom;       % duty of the upper switch at nominal load (1 - D)
    R_load = hw.udc_nom^2/hw.pwr_nom;                % [Ohm] nominal resistive load on the DC link
    k_i = hw.udc_nom/(hw.L*hw.ibez);                 % [1/s] duty -> leg current (pu of ibez)
    a1 = 1/(R_load*hw.Cdc);                          % [1/s] load corner, open loop
    b  = M*hw.udc_nom/(hw.L*hw.ibez);                % [1/s] udc (pu) -> slope of the leg current (pu)
    c  = M*hw.ibez/(hw.Cdc*hw.udc_nom);              % [1/s] leg current (pu) -> slope of udc (pu)
    e  = i_leg/(hw.Cdc*hw.udc_nom);                  % [1/s] duty -> slope of udc (pu), the term of the RHP zero
    k_u = hw.ibez*(M - i_leg*hw.RL/hw.udc_nom)/(hw.Cdc*hw.udc_nom);   % [1/s] total current (pu) -> udc (pu), low frequency
    w_z = (hw.uin_nom - 2*hw.RL*i_leg)/(hw.L*i_leg); % [rad/s] right-half-plane zero at nominal load
    w_i = 2*pi*f_c_i;
    w_u = 2*pi*f_c_u;

    % current loops
    ctrl.kp_i = w_i/k_i;                             % [pu/pu]
    ctrl.ki_i = ctrl.kp_i*w_i/5;                     % [1/s]  zero at f_c_i/5

    % voltage loop: plant at j*w_u = closed common-mode current loop x DC link (with the RHP zero)
    s_u = 1i*w_u;
    G_cm = (k_i*(s_u + a1) + n*b*e)/((s_u + hw.RL/hw.L)*(s_u + a1) + n*b*c);    % leg current per unit duty, common duty
    L_cm = (ctrl.kp_i + ctrl.ki_i/s_u)*G_cm*exp(-1.5*ts*s_u);                     % current loop of leg 1 (1.5 ts)
    T_i = L_cm/(1 + L_cm);
    G_ui = T_i*(c - e/G_cm)/(s_u + a1);              % total current reference (pu) -> udc (pu)
    ctrl.kp_u = 1/(abs(G_ui)*sqrt(1 + 1/25));        % [pu/pu] |PI(j w_u)| = kp_u*sqrt(1 + 1/25) with the zero at w_u/5
    ctrl.ki_u = ctrl.kp_u*w_u/5;                     % [1/s]

    % margins of the voltage loop on the closed-form model (sweep): gain crossover and phase crossover of
    % PI_u*T_i*(c - e/G_cm)/(s + a1) with the same expressions as above
    f_sw = logspace(0, log10(1/(2*ts)), 600);
    s_sw = 1i*2*pi*f_sw;
    G_cm_sw = (k_i*(s_sw + a1) + n*b*e)./((s_sw + hw.RL/hw.L).*(s_sw + a1) + n*b*c);
    L_cm_sw = (ctrl.kp_i + ctrl.ki_i./s_sw).*G_cm_sw.*exp(-1.5*ts*s_sw);
    T_i_sw = L_cm_sw./(1 + L_cm_sw);
    L_u_sw = (ctrl.kp_u + ctrl.ki_u./s_sw).*T_i_sw.*(c - e./G_cm_sw)./(s_sw + a1);
    mag_sw = abs(L_u_sw);
    ph_sw = unwrap(angle(L_u_sw))*180/pi;
    i_gc = find(mag_sw >= 1, 1, 'last');                       % gain crossover (last frequency with |L| >= 1)
    i_pc = find(ph_sw(i_gc:end) <= -180, 1, 'first') + i_gc - 1;   % phase crossover above it
    pm_u = 180 + ph_sw(i_gc);
    if isempty(i_pc)
        gm_u = Inf;
    else
        gm_u = -20*log10(mag_sw(i_pc));
    end

    d.f_c_i = f_c_i;
    d.f_c_u = f_c_u;
    d.i_leg = i_leg;                                 % [A]
    d.M = M;
    d.k_i = k_i;
    d.k_u = k_u;
    d.f_zero_i = ctrl.ki_i/ctrl.kp_i/2/pi;           % [Hz]
    d.f_zero_u = ctrl.ki_u/ctrl.kp_u/2/pi;           % [Hz]
    d.delay_leg1 = 1.5*ts;                           % [s]
    d.delay_leg23 = 2.5*ts;                          % [s]
    d.pm_leg1 = 90 - atand(1/5) - 360*f_c_i*d.delay_leg1;      % [deg] estimate
    d.pm_leg23 = 90 - atand(1/5) - 360*f_c_i*d.delay_leg23;    % [deg]
    d.f_z = w_z/2/pi;                                % [Hz] right-half-plane zero at nominal load
    d.f_load = 2*a1/2/pi;                            % [Hz] load corner of the voltage plant at nominal load (2/(R*Cdc))
    d.T_i_at_f_c_u = abs(T_i);                       % inner loop gain at the voltage crossover
    d.f_c_u_check = f_sw(i_gc);                      % [Hz] gain crossover of the closed-form loop (sweep)
    d.pm_u = pm_u;                                   % [deg] phase margin at nominal load (sweep)
    d.gm_u = gm_u;                                   % [dB]  gain margin at nominal load (sweep: the margin on the RHP zero)
    if ~isempty(i_pc)
        d.f_gm_u = f_sw(i_pc);                       % [Hz] phase crossover
    else
        d.f_gm_u = Inf;
    end
    d.f_c_u_noload = ctrl.kp_u*hw.ibez*M/(hw.Cdc*hw.udc_nom)/2/pi;   % [Hz] estimate without load (integrator plant, no zero)
    d.f_nyquist = 1/(2*ts);                          % [Hz]
    d.f0 = M*sqrt(n/(hw.L*hw.Cdc))/(2*pi);           % [Hz] resonance of the common-mode current plant (open loop)
    d.zeta_f0 = a1/(2*2*pi*d.f0);                    % damping of the resonance by the nominal load
    ctrl.design = d;

    fprintf('Current loop (per leg): kp_i = %.3f pu/pu, ki_i = %.0f 1/s -> crossover %.0f Hz, PI zero %.0f Hz, phase margin about %.0f deg (leg 1, 1.5 ts) / %.0f deg (legs 2-3, 2.5 ts); common-mode resonance %.0f Hz (zeta %.2f at nominal load)\n', ...
        ctrl.kp_i, ctrl.ki_i, f_c_i, d.f_zero_i, d.pm_leg1, d.pm_leg23, d.f0, d.zeta_f0);
    fprintf('Voltage loop: kp_u = %.3f pu/pu, ki_u = %.1f 1/s -> crossover %.0f Hz at nominal load (RHP zero %.0f Hz, load corner %.0f Hz, inner loop gain %.2f there), phase margin %.0f deg, gain margin %.1f dB at %.0f Hz (the RHP zero), PI zero %.0f Hz, about %.0f Hz without load\n', ...
        ctrl.kp_u, ctrl.ki_u, f_c_u, d.f_z, d.f_load, d.T_i_at_f_c_u, d.pm_u, d.gm_u, d.f_gm_u, d.f_zero_u, d.f_c_u_noload);
    if f_c_i > d.f_nyquist/10
        warning('boost_ctrl_param: current loop crossover %.0f Hz above fs/20 = %.0f Hz, the delays leave little margin', f_c_i, d.f_nyquist/10);
    end
    if d.f0 > f_c_i/2
        warning('boost_ctrl_param: common-mode resonance of the current plant (%.0f Hz) above f_c_i/2 = %.0f Hz: the current loops do not damp it at light load', d.f0, f_c_i/2);
    end
    if f_c_u > d.f_z/4
        warning('boost_ctrl_param: voltage loop crossover %.0f Hz above f_z/4 = %.0f Hz (right-half-plane zero of the boost at nominal load): gain margin %.1f dB', ...
            f_c_u, d.f_z/4, d.gm_u);
    end
    if d.f_c_u_noload > f_c_i/2
        warning('boost_ctrl_param: voltage loop crossover without load (%.0f Hz, estimate) less than a factor 2 below the current loops (%.0f Hz)', d.f_c_u_noload, f_c_i);
    end
end
