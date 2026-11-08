function d = vienna_design(param, options)
%VIENNA_DESIGN  Design check of vienna_rectifier: operating point, LCL filter, control loops, neutral point, start.
%
%   d = vienna_design(param, options)   prints the numbers and returns them in a struct (param.afe.design). With
%                                       param.sim.design_figures = true it also draws one figure. options: the
%                                       bodeoptions of the init (frequency unit for the figure: Hz or rad/s).
%
%   Linear, small-signal checks with the data of param. The switched model of the design note (Python, the same
%   numbers) gives the start-up, the harmonics and the neutral-point ripple with the switching; these checks give
%   the steady state and the loops.
%     1. operating point with the load resistor: phasors of the LCL filter at the secondary of the transformer
%        (leakage of the transformer referred to the star secondary, no-load voltage n2/n1 of the coil voltage),
%        converter current in phase with the capacitor voltage (iq = 0, as the control does), converter voltage in pu
%        of udc/sqrt(3): the linear range of the space vector modulation is |u| <= 1 (the current PI clips |u| at 1);
%        angle delta between the converter voltage and the converter current
%     2. LCL resonance against fs/6 (fs = 1/ts): a loop on the converter current with 1.5 periods of delay damps a
%        resonance below fs/6 and takes damping away above it; damping of the LCL mode without and with the loop
%     3. converter-current loop: complex model in the alpha-beta frame, the dq PI of the model is
%        kp + ki ts z/(z - exp(j w ts)) there; the sample is one control period old, the voltage is held over the
%        period (zero-order hold of the LCL filter). Closed-loop poles, crossover and phase margin for the positive
%        and the negative sequence, gain margin, the kp that makes the loop unstable
%     4. DC-voltage loop: the two DC-link halves in series, load pole 2/(R C), udc sample one period old, the
%        closed current loop (d axis) between the PI output and the current; crossover and phase margin
%     5. Vienna constraint and neutral point over one period of the operating point: angle per phase where the pole
%        reference and the current have opposite signs (there the leg cannot follow the reference: the pole stays at
%        the neutral point), with the min-max zero sequence of the modulator and with a zero sequence that follows
%        the sign of the current; neutral-point current (1 - |m|) i and the ripple of vC2 - vC1 it gives
%     6. DDSRF-PLL: linear loop with the decoupling low-pass filter (tau = 1/w): natural frequency, damping,
%        crossover and phase margin
%     7. start: the DC link starts at udc_init with the load connected and the udc reference rises from 0 (Step at
%        1 ms, low-pass 20 ms with zero initial state). The udc PI (udc_ctrl) is in velocity form with
%        back-calculation: it stores x = out - kp e also while its output is clamped, so out(k) = out(k-1) +
%        kp (e(k) - e(k-1)) + ki ts e(k). It starts at its lower limit with a positive stored state, and its output
%        turns positive when the error rises above -x/(kp + ki ts), while udc is still above its reference: with the
%        current loop, which has no feed-forward of the grid voltage, this gives the start-up current peak (see the
%        design note: a lower limit 0 with conditional integration removes it)

    if nargin < 2 || isempty(options)
        hz = true;
    else
        try
            hz = strcmpi(options.FreqUnits, 'Hz');
        catch
            hz = true;
        end
    end
    hw = param.afe.hw;
    c  = param.afe.ctrl;
    tr = param.grid.emu.trafo;
    ts = param.glb_time.ts_afe;
    w  = param.grid.emu.omega_grid_nom;
    udc0 = hw.udc_nom*c.udc_norm_ref;                       % [V] DC voltage of the operating point
    Ku = udc0/sqrt(3);                                      % [V] phase peak per pu of the control voltage
    g  = Ku/hw.ibez;                                        % [V/A] from the current error (A) to the voltage (V)

    % LCL filter at the secondary of the transformer
    L1 = hw.LFu + 1e-6;                                     % [H] LFu + the 1 uH Inductor blocks of LCL_filter
    R1 = hw.RLFu;
    Cf = hw.CFu;
    Rc = hw.RCFu;
    k21 = (tr.n2/tr.n1)^2;
    L2 = tr.Ld2 + tr.Ld1*k21;                               % [H] leakage of the transformer referred to the secondary
    R2 = tr.Rd2 + tr.Rd1*k21;                               % [Ohm]
    u1_coil = param.grid.emu.Vemu_ref_norm*param.grid.emu.us_primary_nom*sqrt(2/3);   % [V] phase peak of the sources
    if tr.delta_star == 1
        u1_coil = sqrt(3)*u1_coil;                          % the delta coil sees the line voltage
    end
    E_pk = u1_coil*tr.n2/tr.n1;                             % [V] no-load phase peak at the secondary
    C1 = hw.CFi1;
    C2 = hw.CFi2;
    C_ser = C1*C2/(C1 + C2);                                % [F] seen by udc
    C_np = C1 + C2;                                         % [F] seen by the neutral point
    R_load = param.load.R;
    R_meas = param.afe.dclink.Rudc_measure;
    d.lcl = struct('L1', L1, 'R1', R1, 'Cf', Cf, 'Rc', Rc, 'L2', L2, 'R2', R2, 'E_pk', E_pk);

    % ======================================================================== 1. operating point
    P_dc = udc0^2/R_load + udc0^2/(2*R_meas);               % [W] load and the two measuring resistors
    Z1 = R1 + 1j*w*L1;
    Z2 = R2 + 1j*w*L2;
    Yc = 1/(Rc + 1/(1j*w*Cf));
    I1 = P_dc/(1.5*E_pk);
    for it = 1:60                                           % ug on the real axis, i1 = I1 in phase with it
        a = 1 + Z2*Yc;
        b = Z2*I1;
        qa = abs(a)^2;
        qb = 2*real(conj(a)*b);
        qc = abs(b)^2 - E_pk^2;
        Ug = (-qb + sqrt(qb^2 - 4*qa*qc))/(2*qa);           % |a Ug + b| = E_pk
        vconv = Ug - Z1*I1;
        I1 = I1*P_dc/(1.5*real(vconv)*I1);
    end
    I2 = I1 + Ug*Yc;
    u_pu = abs(vconv)/Ku;
    delta = -angle(vconv);                                  % [rad] the converter voltage lags the current
    d.op = struct('P', P_dc, 'ug', Ug, 'i1', I1, 'i2', abs(I2), 'pf_grid', cos(angle(I2)), 'vconv', abs(vconv), ...
        'u_pu', u_pu, 'margin', 1 - u_pu, 'delta_deg', delta*180/pi);
    fprintf(['%s design check\n1. Operating point: load %.2f Ohm at %.0f V -> %.1f kW; E = %.1f V peak at the secondary ' ...
        '(no load), capacitor voltage %.1f V (%.3f pu), converter current %.1f A peak (%.3f pu of %.1f A), grid ' ...
        'current %.1f A (power factor %.3f at the capacitor)\n   converter voltage %.1f V = %.3f pu of udc/sqrt(3) ' ...
        '(margin %.1f %% to the linear limit of the modulation), lagging the current by delta = %.2f deg\n'], ...
        'vienna_rectifier', R_load, udc0, P_dc/1e3, E_pk, Ug, Ug/hw.ubez, I1, I1/hw.ibez, hw.ibez, abs(I2), ...
        d.op.pf_grid, abs(vconv), u_pu, 100*(1 - u_pu), delta*180/pi);

    % ======================================================================== 2. LCL resonance
    A = [-(R1 + Rc)/L1, -1/L1, Rc/L1; 1/Cf, 0, -1/Cf; Rc/L2, 1/L2, -(Rc + R2)/L2];
    B = [1/L1; 0; 0];
    f_res = sqrt((L1 + L2)/(L1*L2*Cf))/2/pi;
    [f_ol, z_ol] = top_mode_s(eig(A));
    d.res = struct('f_res', f_res, 'fs6', 1/(6*ts), 'zeta_passive', z_ol);
    fprintf(['2. LCL resonance: L1 = %.1f uH, Cf = %.0f uF (Rc = %.0f mOhm), L2 = %.1f uH (transformer leakage at ' ...
        'the secondary): %.0f Hz, fs/6 = %.0f Hz; passive damping of the mode %.1f %% (%.0f Hz)\n'], L1*1e6, ...
        Cf*1e6, Rc*1e3, L2*1e6, f_res, 1/(6*ts), 100*z_ol, f_ol);

    % ======================================================================== 3. converter-current loop
    if c.kp_is_xi ~= c.kp_is_eta || c.ki_is_xi ~= c.ki_is_eta
        fprintf('   (the xi and eta gains differ: the check uses the xi gains)\n');
    end
    kp = c.kp_is_xi;
    ki = c.ki_is_xi;
    E4 = expm([A*ts, B*ts; zeros(1, 4)]);
    Ad = E4(1:3, 1:3);
    Bd = E4(1:3, 4);
    [Acl, Bcl] = current_cl(kp, ki, Ad, Bd, g, w, ts);
    p = eig(Acl);
    rad = max(abs(p));
    [f_cl, z_cl] = top_mode_z(p, ts);                       % LCL mode (alpha-beta, the physical frequency)
    s_dq = log(p)/ts - 1j*w;                                % poles in the dq frame
    s_dq = s_dq(abs(s_dq) < 0.5*f_cl*2*pi);                 % the slow ones (not the LCL mode)
    [~, k] = max(real(s_dq));
    s_slow = s_dq(k);
    Lcur = @(z) current_open(z, kp, ki, Ad, Bd, g, w, ts);
    [fc_p, pm_p, gm_p] = margins_c(Lcur, ts, w/2/pi, 1);
    [fc_n, pm_n, gm_n] = margins_c(Lcur, ts, w/2/pi, -1);
    kp_lim = [];
    for kk = linspace(0.02, 3, 750)
        if max(abs(eig(current_cl(kk, ki, Ad, Bd, g, w, ts)))) >= 1
            kp_lim = kk;
            break
        end
    end
    d.cur = struct('kp', kp, 'ki', ki, 'radius', rad, 'f_lcl', f_cl, 'zeta_lcl', z_cl, 's_slow', s_slow, ...
        'fc_pos', fc_p, 'pm_pos', pm_p, 'gm_pos', gm_p, 'fc_neg', fc_n, 'pm_neg', pm_n, 'gm_neg', gm_n, 'kp_lim', kp_lim);
    fprintf(['3. Converter-current loop (kp %.3g, ki %.3g 1/s, ts %.1f us, 1 pu of voltage = %.1f V): pole radius ' ...
        '%.4f%s\n   positive sequence: crossover %s Hz, phase margin %s deg, gain margin %s dB | negative sequence: ' ...
        'crossover %s Hz, phase margin %s deg, gain margin %s dB\n   LCL mode with the loop %.0f Hz, damping %.1f %% ' ...
        '(passive %.1f %%) | slowest pole in dq %.1f %+.1fj rad/s (time constant %.1f ms) | unstable from kp = %s\n'], ...
        kp, ki, ts*1e6, Ku, rad, flag(rad), num(fc_p, '%.1f'), num(pm_p, '%.1f'), num(gm_p, '%.1f'), num(fc_n, '%.1f'), ...
        num(pm_n, '%.1f'), num(gm_n, '%.1f'), f_cl, 100*z_cl, 100*z_ol, real(s_slow), imag(s_slow), ...
        -1e3/real(s_slow), num(kp_lim, '%.2f', '> 3'));
    if f_res > 1/(6*ts)
        fprintf(['   The resonance is above fs/6: the loop takes a little damping away from the LCL mode, Rc keeps it ' ...
            'stable (%.1f %% left)\n'], 100*z_cl);
    end

    % ======================================================================== 4. DC-voltage loop
    K = 1.5*real(vconv)*hw.ibez/(C_ser*udc0*hw.udc_nom);    % d(udc_pu)/dt per pu of active current [1/s]
    a_load = 2/C_ser*(1/R_load + 1/(2*R_meas));             % [rad/s] pole of the resistive load
    ea = exp(-a_load*ts);
    Gu = @(z) K*(1 - ea)/a_load./(z - ea);
    Tdd = @(z) current_dd(z, Acl, Bcl, w, ts);              % d-axis closed current loop
    Lu = @(z) (c.kp_udc_ctrl + c.ki_udc_ctrl*ts*z./(z - 1)).*Tdd(z).*Gu(z)./z;
    [fc_u, pm_u, gm_u] = margins_r(Lu, ts);
    d.udc = struct('C', C_ser, 'K', K, 'a', a_load, 'fc', fc_u, 'pm', pm_u, 'gm', gm_u, 'tau_RC', R_load*C_ser);
    fprintf(['4. DC-voltage loop (kp %.3g, ki %.3g 1/s, limit +-%.2f pu): C = %.1f mF (two halves in series), ' ...
        'plant %.1f 1/s per pu, load pole %.1f rad/s: crossover %s Hz, phase margin %s deg, gain margin %s dB\n'], ...
        c.kp_udc_ctrl, c.ki_udc_ctrl, c.ixi_pos_ref_lim, C_ser*1e3, K, a_load, num(fc_u, '%.1f'), num(pm_u, '%.1f'), ...
        num(gm_u, '%.1f'));

    % ======================================================================== 5. Vienna constraint, neutral point
    th = (0:3599)/3600*2*pi;
    ph = [0; -2*pi/3; 2*pi/3];
    m0 = 2/sqrt(3)*u_pu*cos(th - delta + ph);               % modulation of the three phases (1 = udc/2)
    ii = I1*cos(th + ph);                                   % [A] converter currents
    v0 = -0.5*(max(m0) + min(m0));                          % min-max zero sequence of the SVM (k_offset 0.5)
    m_svm = m0 + v0;
    [m_vi, ok_vi] = vienna_zero_sequence(m0, ii);
    mis_svm = 360*mean(m_svm(1, :).*ii(1, :) < 0);
    mis_vi = 360*mean(m_vi(1, :).*ii(1, :) < 0);
    m_eff = m_svm.*(m_svm.*ii >= 0);                        % opposite signs: the pole stays at np
    i_np = sum((1 - abs(m_eff)).*ii);
    v_np = cumsum(i_np)*(2*pi/w/numel(th))/C_np;
    v_np = v_np - mean(v_np);
    X = fft(i_np)/numel(th)*2;
    d.np = struct('mis_svm_deg', mis_svm, 'mis_vienna_deg', mis_vi, 'vienna_feasible', mean(ok_vi), ...
        'i_np_mean', mean(i_np), 'i_np_h3', abs(X(4)), 'i_np_peak', max(abs(i_np)), 'unb_pp', 2*(max(v_np) - min(v_np)));
    fprintf(['5. Vienna constraint: pole reference and current of a phase with opposite signs over %.2f deg per ' ...
        'period (2 delta, min-max zero sequence) -> %.2f deg with a zero sequence that follows the current sign ' ...
        '(possible over %.0f %% of the period)\n   neutral-point current: mean %.2f A, third harmonic %.1f A, peak ' ...
        '%.1f A -> ripple of vC2 - vC1 %.1f V peak to peak (C1 + C2 = %.1f mF), no drift in steady state\n'], ...
        mis_svm, mis_vi, 100*mean(ok_vi), mean(i_np), abs(X(4)), max(abs(i_np)), d.np.unb_pp, C_np*1e3);

    % ======================================================================== 6. DDSRF-PLL
    tau = c.tau_ddsrf;
    wn = sqrt(w*c.pll_i1_ddsrt);
    zeta = w*c.pll_p_ddsrt/(2*wn);
    pp = roots([tau, 1, w*c.pll_p_ddsrt, w*c.pll_i1_ddsrt]);
    Lp = @(s) w*(c.pll_p_ddsrt*s + c.pll_i1_ddsrt)./s.^2./(tau*s + 1);
    f = logspace(-1, 3, 20000);
    Lf = Lp(1j*2*pi*f);
    k = find(abs(Lf(1:end-1)) >= 1 & abs(Lf(2:end)) < 1, 1);
    [~, k1] = max(real(pp));                                % slowest closed-loop pole
    [~, k2] = max(abs(imag(pp)));                           % complex pair (if any)
    d.pll = struct('wn', wn, 'zeta', zeta, 'poles', pp, 'fc', f(k), 'pm', 180 + angle(Lf(k))*180/pi);
    fprintf(['6. DDSRF-PLL (kp %.3g, ki %.3g, decoupling filter tau = %.2f ms): without the filter wn = %.1f rad/s ' ...
        '(%.1f Hz), damping %.2f; with it crossover %.1f Hz, phase margin %.1f deg, closed-loop poles %.1f rad/s ' ...
        '(time constant %.0f ms) and %.0f rad/s (damping %.2f)\n'], c.pll_p_ddsrt, c.pll_i1_ddsrt, tau*1e3, wn, ...
        wn/2/pi, zeta, d.pll.fc, d.pll.pm, real(pp(k1)), -1e3/real(pp(k1)), abs(pp(k2)), -real(pp(k2))/abs(pp(k2)));

    % ======================================================================== 7. start
    u_diode = sqrt(3)*E_pk;
    t_step = 1e-3;
    tau_ref = 20e-3;
    if u_diode < udc0
        t_cross = t_step - tau_ref*log(1 - u_diode/udc0);   % the udc reference passes the diode level
    else
        t_cross = [];
    end
    kpi = c.kp_udc_ctrl + c.ki_udc_ctrl*ts;
    e0 = 0 - param.afe.dclink.udc_init/hw.udc_nom;          % [pu] first error of the udc PI (reference 0)
    out0 = min(max(kpi*e0, -c.ixi_pos_ref_lim), c.ixi_pos_ref_lim);
    x0 = out0 - c.kp_udc_ctrl*e0;                           % state stored by the back-calculation
    e_pos = -x0/kpi;                                        % error at which the output turns positive
    d.start = struct('u_diode', u_diode, 't_cross', t_cross, 'udc_init', param.afe.dclink.udc_init, ...
        'tau_RC', R_load*C_ser, 'e0', e0, 'out0', out0, 'x0', x0, 'e_pos', e_pos);
    fprintf(['7. Start: DC link at %.0f V with the load (R C = %.0f ms), grid voltage from 0 (ramp %.0f ms and filter), ' ...
        'diode level sqrt(3) E = %.0f V (%.3f pu)\n'], param.afe.dclink.udc_init, 1e3*R_load*C_ser, ...
        1e3*param.frt.data.t_ramp, u_diode, u_diode/hw.udc_nom);
    if out0 <= -c.ixi_pos_ref_lim && x0 > 0 && e_pos < 0
        fprintf(['   udc PI (velocity form, back-calculation): the reference starts at 0, error %.2f pu, output at ' ...
            'the lower limit %.2f pu, stored state %+.2f pu: the output turns positive when the error rises above ' ...
            '%.2f pu, while udc is still above its reference (start-up current peak: see the design note; a lower ' ...
            'limit 0 with conditional integration removes it)\n'], e0, out0, x0, e_pos);
    else
        fprintf('   udc PI: no positive output while udc is above its reference (error %.2f pu at the start)\n', e0);
    end

    % ======================================================================== figure
    if isfield(param.sim, 'design_figures') && param.sim.design_figures
        figure('Name', 'vienna_rectifier: design check');
        fx = logspace(0, log10(0.4999/ts), 3000);
        fu = 1;
        if ~hz
            fu = 2*pi;
        end
        subplot(2, 2, 1);
        Lpos = Lcur(exp(1j*2*pi*fx*ts));
        Lneg = Lcur(exp(-1j*2*pi*fx*ts));
        semilogx(fx*fu, 20*log10(abs(Lpos)), fx*fu, 20*log10(abs(Lneg)));
        grid on; ylabel('|L| [dB]'); title('Current loop (alpha-beta)');
        legend('positive sequence', 'negative sequence', 'Location', 'southwest');
        subplot(2, 2, 2);
        Lux = Lu(exp(1j*2*pi*fx*ts));
        semilogx(fx*fu, 20*log10(abs(Lux)));
        grid on; ylabel('|L| [dB]'); title('DC-voltage loop');
        if hz
            xlabel('f [Hz]');
        else
            xlabel('\omega [rad/s]');
        end
        subplot(2, 2, 3);
        deg = th*180/pi;
        plot(deg, m_svm(1, :), deg, m_vi(1, :), deg, ii(1, :)/hw.ibez);
        grid on; xlim([0 360]); xlabel('angle [deg]');
        title('Phase u: pole reference and current');
        legend('m, min-max zero sequence', 'm, zero sequence on the current sign', 'i [pu]', 'Location', 'southwest');
        subplot(2, 2, 4);
        plot(deg, i_np, deg, 2*v_np*10);
        grid on; xlim([0 360]); xlabel('angle [deg]');
        title('Neutral point');
        legend('i_{np} [A]', '10 (vC2 - vC1) [V]', 'Location', 'southwest');
    end
end

% -------------------------------------------------------------------------------------------- current loop
function [Acl, Bcl] = current_cl(kp, ki, Ad, Bd, g, w, ts)
%CURRENT_CL  closed loop in alpha-beta, states [i1; vc; i2; i1 sampled at the previous trigger; integrator of the
%   previous trigger]: u(k) = (kp + ki ts) (r - s) + rho X(k-1), X(k) = rho X(k-1) + ki ts (r - s), rho = exp(j w ts)
    rho = exp(1j*w*ts);
    Acl = zeros(5);
    Acl(1:3, 1:3) = Ad;
    Acl(1:3, 4) = -Bd*g*(kp + ki*ts);
    Acl(1:3, 5) = Bd*g*rho;
    Acl(4, 1) = 1;
    Acl(5, 4) = -ki*ts;
    Acl(5, 5) = rho;
    Bcl = [Bd*g*(kp + ki*ts); 0; ki*ts];
end

function L = current_open(z, kp, ki, Ad, Bd, g, w, ts)
%CURRENT_OPEN  open loop in alpha-beta: PI (rotating integrator) * one period of delay * LCL (u -> i1)
    L = zeros(size(z));
    for k = 1:numel(z)
        G = [1 0 0]*((z(k)*eye(3) - Ad)\Bd)*g;
        L(k) = (kp + ki*ts*z(k)/(z(k) - exp(1j*w*ts)))*G/z(k);
    end
end

function T = current_dd(z, Acl, Bcl, w, ts)
%CURRENT_DD  d-axis to d-axis closed current loop at the dq frequency of z: (T(f + f0) + conj(T(-f + f0)))/2
    T = zeros(size(z));
    rho = exp(1j*w*ts);
    C = [1 0 0 0 0];
    for k = 1:numel(z)
        Tp = C*((z(k)*rho*eye(5) - Acl)\Bcl);
        Tn = C*((conj(z(k))*rho*eye(5) - Acl)\Bcl);
        T(k) = (Tp + conj(Tn))/2;
    end
end

% -------------------------------------------------------------------------------------------- Vienna
function [m, ok] = vienna_zero_sequence(m0, i)
%VIENNA_ZERO_SEQUENCE  zero sequence closest to the min-max one that gives every pole the sign of its current
%   (within |m| <= 1); where none exists the min-max one is kept (ok = false)
    n = size(m0, 2);
    m = zeros(size(m0));
    ok = false(1, n);
    for k = 1:n
        lo = -1 - min(m0(:, k));
        hi = 1 - max(m0(:, k));
        for x = 1:3
            if i(x, k) > 0
                lo = max(lo, -m0(x, k));
            elseif i(x, k) < 0
                hi = min(hi, -m0(x, k));
            end
        end
        v0 = -0.5*(max(m0(:, k)) + min(m0(:, k)));
        if lo <= hi
            ok(k) = true;
            v0 = min(max(v0, lo), hi);
        end
        m(:, k) = m0(:, k) + v0;
    end
end

% -------------------------------------------------------------------------------------------- tools
function [fc, pm, gm] = margins_c(Lfun, ts, f0, sgn)
%MARGINS_C  crossover [Hz], phase margin [deg], gain margin [dB] of a complex-coefficient loop for one sequence:
%   sgn = 1 positive frequencies above f0 (the integrator of the PI sits at +f0), sgn = -1 negative frequencies
    if sgn > 0
        f = logspace(log10(1.02*f0), log10(0.4999/ts), 40000);
    else
        f = logspace(-1, log10(0.4999/ts), 40000);
    end
    L = Lfun(exp(1j*2*pi*sgn*f*ts));
    mag = abs(L);
    ph = sgn*unwrap(angle(L))*180/pi;                       % phase seen from the frequency axis of the sequence
    fc = []; pm = []; gm = [];
    k = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1);
    if ~isempty(k)
        fc = f(k);
        pm = mod(ph(k) + 180 + 180, 360) - 180;
    else
        k = 1;
    end
    q = floor((ph + 180)/360);                              % changes where the phase passes -180 deg (+- n 360)
    k = k - 1 + find(q(k:end-1) ~= q(k+1:end), 1);          % first one after the crossover
    if ~isempty(k)
        gm = -20*log10(mag(k));
    end
end

function [fc, pm, gm] = margins_r(Lfun, ts)
%MARGINS_R  first crossover [Hz], phase margin [deg], gain margin [dB] (first -180 deg crossing) of L(z)
    f = logspace(-1, log10(0.4999/ts), 40000);
    L = Lfun(exp(1j*2*pi*f*ts));
    mag = abs(L);
    ph = unwrap(angle(L))*180/pi;
    fc = []; pm = []; gm = [];
    k = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1);
    if ~isempty(k)
        fc = f(k);
        pm = mod(180 + ph(k) + 180, 360) - 180;
    end
    k = find(ph(1:end-1) > -180 & ph(2:end) <= -180, 1);
    if ~isempty(k)
        gm = -20*log10(mag(k));
    end
end

function [f, zeta] = top_mode_s(p)
%TOP_MODE_S  frequency [Hz] and damping of the continuous complex pole with the highest frequency
    f = 0; zeta = 1;
    for k = 1:numel(p)
        if imag(p(k)) > 1e-6 && abs(p(k))/2/pi > f
            f = abs(p(k))/2/pi;
            zeta = -real(p(k))/abs(p(k));
        end
    end
end

function [f, zeta] = top_mode_z(p, ts)
%TOP_MODE_Z  frequency [Hz] and damping of the discrete pole with the highest frequency (the LCL mode)
    f = 0; zeta = 1;
    for k = 1:numel(p)
        s = log(p(k))/ts;
        if abs(imag(s))/2/pi > f
            f = abs(imag(s))/2/pi;
            zeta = -real(s)/abs(s);
        end
    end
end

function s = num(x, fmt, empty)
    if nargin < 3
        empty = 'n/a';
    end
    if isempty(x)
        s = empty;
    else
        s = sprintf(fmt, x);
    end
end

function s = flag(rad)
    s = '';
    if rad >= 1
        s = '  UNSTABLE';
    end
end
