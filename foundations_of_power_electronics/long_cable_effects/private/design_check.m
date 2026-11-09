function d = design_check(param)
%DESIGN_CHECK  Design figures of afe_inv_psm_with_hf_models: power cable modes against the damping of the
%   Simscape solver, AFE current loop with the LCL filter, inverter current loop, DC-link voltage loop,
%   operating point, start-up and clock alignment.
%
%   d = design_check(param)   prints the tables and returns the numbers in the struct d
%
%   1. Power cable (param.cable, n lumped sections per phase). In the model the machine is connected at the
%      inverter terminals and the far end of the cable is open (Open Circuit blocks after the machine-side
%      measurement): at high frequency the machine winding is a high impedance, so the open end is the
%      machine terminal of the ringing. Seen from the bridge:
%        normal mode  : inverter filter (L1 + Lls1 + L2 + Lls2) in series with the cable, capacitors to the
%                       star point (= ground for a balanced excitation)
%        common mode  : zero-sequence path from the inverter bridge through the inverter filter, the cable
%                       capacitances to ground, the grounded star point of the transformer, its leakage, the
%                       AFE filter (zero sequence, the star of Cf is floating) and the AFE bridge
%      For each mode: frequency, physical damping, damping of the Backward Euler local solver at tc
%      (x(k+1) = (I - tc*A)\x(k): a pole s becomes z = 1/(1 - s*tc)), steps per period. Step response at the
%      open end for the switching of one leg (udc_nom): exact (zero-order hold) and Backward Euler at tc.
%   2. AFE current loop with the LCL filter on the transformer leakage, exact discrete-time loop with the
%      timing of the block (sample of trigger k-1 used at trigger k, output held for one period ts).
%   3. Inverter current loop on one machine system + filter (q and d axis), and the dq cross-coupling at the
%      speed of the shaft.
%   4. DC-link voltage loop (C = Cdc of the AFE block + Cdc of the inverter block).
%   5. Operating point at param.load.rpm_sim (torque curve times param.load.torque_overload_factor).
%   6. Start-up: charge of the DC link through the diodes during the voltage ramp of the grid emulator, DC-link
%      voltage reference against the diode voltage, enable of the inverter and torque ramp.
%   7. Clock alignment loops of the two blocks (t_loc = t_master*k_time*(1 + corr)): carrier phase error
%      against the master time and stability limit of the loop.
%   Small-signal and linear except 7: limits, dead time and the PWM sampling are left out.

    a  = param.afe_follower(1);
    v  = param.inv_mod(1);
    mc = param.psm.machine;
    tr = param.grid.emu.trafo;
    cb = param.cable;
    h  = param.glb_time.tc;
    n21 = tr.n2/tr.n1;
    Lt  = tr.Ld2 + tr.Ld1*n21^2;                            % [H] transformer leakage at the secondary
    Rt  = tr.Rd2 + tr.Rd1*n21^2;                            % [Ohm] (positive and zero sequence: Dyn bank)
    udc = v.hw.udc_nom;

    % ================================================================ 1. power cable
    Ls_dm = v.hw.L1p + v.hw.Lls1 + v.hw.L2p + v.hw.Lls2;
    Rs_dm = v.hw.R1 + v.hw.R2 + v.hw.Ron;
    Ls_cm = (v.hw.L10 + v.hw.Lls1 + v.hw.L20 + v.hw.Lls2) + (a.hw.L10 + a.hw.Lls1 + a.hw.L20 + a.hw.Lls2) + Lt;
    Rs_cm = v.hw.R1 + v.hw.R2 + a.hw.R1 + a.hw.R2 + Rt + v.hw.Ron + a.hw.Ron;
    fprintf(['1. Power cable: %d sections per phase, per section %.1f uH, %.1f mOhm, %.2f nF to ground (%.2f mOhm), ' ...
        'Z0 %.1f Ohm, f %.0f kHz; far end open; local solver Backward Euler, tc = %.3f us\n'], cb.n, cb.Ls1*1e6, ...
        cb.Rs1*1e3, cb.Cp1*1e9, cb.RCs1*1e3, cb.Z0, cb.f1/1e3, h*1e6);
    fprintf('   source inductance seen by the cable: normal mode %.1f uH (inverter filter), common mode %.1f uH (both filters + transformer)\n', ...
        Ls_dm*1e6, Ls_cm*1e6);
    fprintf('   %-14s %10s %12s %12s %10s %14s\n', 'mode', 'f/kHz', 'zeta phys.', 'zeta BE', 'f BE/kHz', 'steps/period');
    modes = {'normal', Ls_dm, Rs_dm; 'common', Ls_cm, Rs_cm};
    d.cable = struct('mode', {}, 'f', {}, 'zeta', {}, 'zeta_be', {}, 'f_be', {}, 'steps', {});
    for m = 1:2
        [A, ~] = ladder(modes{m, 2}, modes{m, 3}, cb);
        s = eig(A);
        s = s(imag(s) > 0);
        [~, k] = sort(imag(s));
        s = s(k);
        sb = log(1./(1 - s*h))/h;                           % Backward Euler pole, back to the s plane
        for k = 1:numel(s)
            f = abs(s(k))/2/pi;
            r = struct('mode', modes{m, 1}, 'f', f, 'zeta', -real(s(k))/abs(s(k)), ...
                'zeta_be', -real(sb(k))/abs(sb(k)), 'f_be', imag(sb(k))/2/pi, 'steps', 1/(f*h));
            d.cable(end+1) = r;
            fprintf('   %-14s %10.1f %12.5f %12.4f %10.1f %14.1f\n', sprintf('%s %d', modes{m, 1}, k), f/1e3, ...
                r.zeta, r.zeta_be, r.f_be/1e3, r.steps);
        end
    end
    zr = [d.cable.zeta_be]./[d.cable.zeta];
    fprintf(['   The solver damps the ringing %.0f to %.0f times more than the cable: the decay of the ringing in the ' ...
        'simulation is set by tc, not by the cable data (physical Q of a section %.0f).\n'], min(zr), max(zr), cb.Q1);

    % switching of one leg (step udc on one phase): line-to-line at the far end = udc*(normal-mode step
    % response), common-mode part of the phase voltage = udc/3*(common-mode step response); the three phases
    % carry the same zero-sequence current, total to ground = 3*i0
    nt = round(500e-6/h);                                   % first 500 us
    [A, B, Cv, Ci] = ladder(Ls_dm, Rs_dm, cb);
    [ydm, ydm_be] = steps(A, B, [Cv; Ci], h, nt);
    [A, B, Cv, Ci] = ladder(Ls_cm, Rs_cm, cb);
    [ycm, ycm_be] = steps(A, B, [Cv; Ci], h, nt);
    d.step.ull_peak    = udc*max(ydm(1, :));                % [V] line-to-line at the far end
    d.step.ull_peak_be = udc*max(ydm_be(1, :));
    d.step.u0_peak     = udc/3*max(ycm(1, :));              % [V] common-mode voltage at the far end
    d.step.u0_peak_be  = udc/3*max(ycm_be(1, :));
    uph    = 2/3*udc*ydm(1, :) + udc/3*ycm(1, :);           % [V] phase to ground at the far end
    uph_be = 2/3*udc*ydm_be(1, :) + udc/3*ycm_be(1, :);
    d.step.uph_peak    = max(uph);
    d.step.uph_peak_be = max(uph_be);
    d.step.icm_peak    = udc*max(abs(ycm(2, :)));           % [A] total current to ground (3*i0)
    d.step.icm_peak_be = udc*max(abs(ycm_be(2, :)));
    sl_dm = min(-real(eig(ladder(Ls_dm, Rs_dm, cb))));
    sl_cm = min(-real(eig(ladder(Ls_cm, Rs_cm, cb))));
    d.step.t_decay      = log(20)/min(sl_dm, sl_cm);        % [s] 5 % of the ringing, physical damping
    d.step.t_decay_be   = max(decay_time(ydm_be(1, :)), decay_time(ycm_be(1, :)))*h;
    fprintf(['   switching of one leg (step of %.0f V) - peak at the far end, exact / Backward Euler at tc:\n' ...
        '     line-to-line %.0f / %.0f V (%.2f / %.2f times the step), common-mode voltage %.0f / %.0f V, ' ...
        'change of the phase-to-ground voltage %.0f / %.0f V\n     current to ground (three phases) %.1f / %.1f A peak; the ringing decays ' ...
        'to 5 %% in %.1f ms with the cable losses, in %.1f us with the solver\n'], udc, d.step.ull_peak, ...
        d.step.ull_peak_be, d.step.ull_peak/udc, d.step.ull_peak_be/udc, d.step.u0_peak, d.step.u0_peak_be, ...
        d.step.uph_peak, d.step.uph_peak_be, d.step.icm_peak, d.step.icm_peak_be, d.step.t_decay*1e3, ...
        d.step.t_decay_be*1e6);
    fprintf(['   The switching period is %.0f us: with the cable losses alone the ringing of one edge would still be ' ...
        'there at the next edges.\n'], 1e6/v.pwm.fpwm);

    % ================================================================ 2. AFE current loop with the LCL filter
    ha = a.hw;
    ca = a.ctrl;
    ts = ca.ts;
    Ku = ha.udc_nom/sqrt(3);                                % [V] per unit of modulation
    L1 = ha.L1p + ha.Lls1;
    Lg = ha.L2p + ha.Lls2 + Lt;
    Rg = ha.R2 + Rt;
    [Ad, Bd] = lcl_d(L1, ha.R1, ha.Cf, ha.Rcf, Lg, Rg, ts);
    f_res = sqrt((L1 + Lg)/(L1*Lg*ha.Cf))/2/pi;
    Zr = sqrt(L1*Lg/(L1 + Lg)/ha.Cf);
    [fc, pm, gm] = margins(@(z) afe_open(z, ca.kp_i, ca.ki_i, Ad, Bd, Ku/ha.ibez, ts), ts, 6000);
    p = afe_poles(ca.kp_i, ca.ki_i, Ad, Bd, Ku/ha.ibez, ts);
    [f_mode, z_mode] = top_mode(p, ts, f_res);
    kp_lim = kp_limit(@(kp) afe_poles(kp, ca.ki_i, Ad, Bd, Ku/ha.ibez, ts), ca.kp_i, 4*ca.kp_i);
    d.afe = struct('Lg', Lg, 'f_res', f_res, 'Q_rcf', Zr/ha.Rcf, 'fc', fc, 'pm', pm, 'gm', gm, 'f_mode', f_mode, ...
        'zeta_mode', z_mode, 'rad', max(abs(p)), 'kp_lim', kp_lim);
    fprintf(['\n2. AFE current loop: kp_i = %.3g, ki_i = %.3g 1/s, ts = %.1f us, L1 = %.0f uH, Cf = %.0f uF, ' ...
        'Rcf = %.0f mOhm, grid side %.1f uH (Lls2 + transformer leakage %.1f uH)\n'], ca.kp_i, ca.ki_i, ts*1e6, ...
        L1*1e6, ha.Cf*1e6, ha.Rcf*1e3, Lg*1e6, Lt*1e6);
    fprintf(['   LCL resonance %.0f Hz (fs/6 = %.0f Hz, Nyquist %.0f Hz), damping of Rcf alone Q = %.1f | loop: ' ...
        'crossover %s Hz, PM %s deg, GM %s dB | closed-loop LCL mode %.0f Hz (above the Nyquist frequency: seen ' ...
        'aliased by the control), damping %.1f %% | |z|max %.4f, kp limit %s%s\n'], f_res, 1/(6*ts), 1/(2*ts), ...
        Zr/ha.Rcf, num(fc, '%.1f'), num(pm, '%.1f'), num(gm, '%.1f'), f_mode, 100*z_mode, max(abs(p)), ...
        num(kp_lim, '%.2f', sprintf('> %.2f', 4*ca.kp_i)), flag(max(abs(p))));
    fprintf(['   The resonance is above fs/6: the converter-current loop does not damp it, Rcf does (the 1 uH of ' ...
        'Lls2 and the 2.5 uH of the transformer are the grid side).\n']);

    % ================================================================ 3. inverter current loop
    c  = v.ctrl;
    hv = v.hw;
    wb = c.omega_b;
    zb = hv.ubez/hv.ibez;
    lb = zb/wb;
    l1 = (hv.L1p + hv.Lls1)/lb;
    r1 = hv.R1/zb;
    l2 = (hv.L2p + hv.Lls2)/lb;
    r2 = hv.R2/zb;
    ts = c.ts;
    axes_ = {'q axis', c.rs + r1 + r2, c.lq + l1 + l2; 'd axis', c.rs + r1 + r2, c.ld + l1 + l2};
    fprintf('\n3. Inverter current loop: kp_i = %.3g, ki_i = %.3g 1/s, ts = %.1f us, filter %.0f uH = %.4f pu\n', ...
        c.kp_i, c.ki_i, ts*1e6, (hv.L1p + hv.Lls1 + hv.L2p + hv.Lls2)*1e6, l1 + l2);
    d.inv = struct('axis', {}, 'l', {}, 'fc', {}, 'pm', {}, 'gm', {}, 'rad', {}, 'kp_lim', {});
    for k = 1:2
        [r, l] = axes_{k, 2:3};
        [fc, pm, gm] = margins(@(z) inv_open(z, c.kp_i, c.ki_i, r, l, wb, ts), ts);
        p = inv_poles(c.kp_i, c.ki_i, r, l, wb, ts);
        kp_lim = kp_limit(@(kp) inv_poles(kp, c.ki_i, r, l, wb, ts), c.kp_i, 4*c.kp_i);
        d.inv(k) = struct('axis', axes_{k, 1}, 'l', l, 'fc', fc, 'pm', pm, 'gm', gm, 'rad', max(abs(p)), 'kp_lim', kp_lim);
        fprintf('   %s: l = %.3f pu, crossover %s Hz, PM %s deg, GM %s dB, |z|max %.4f, kp limit %s%s\n', ...
            axes_{k, 1}, l, num(fc, '%.1f'), num(pm, '%.1f'), num(gm, '%.1f'), max(abs(p)), ...
            num(kp_lim, '%.1f', sprintf('> %.1f', 4*c.kp_i)), flag(max(abs(p))));
    end
    w_pu = param.load.rpm_sim/mc.rpm_nom;
    Lsum = diag([c.ld, c.lq]) + (l1 + l2)*eye(2);
    rsum = c.rs + r1 + r2;
    F = inv_mimo(c.kp_i, c.ki_i, rsum, Lsum, w_pu, wb, ts);
    s = log(complex(eig(F)))/ts;
    [~, k] = max(real(s));
    [t2, over, idpk] = inv_mimo_step(c.kp_i, c.ki_i, rsum, Lsum, w_pu, wb, ts);
    d.inv_mimo = struct('speed', w_pu, 'pole', s(k), 'rad', max(abs(eig(F))), 't_settle', t2, 'overshoot', over, ...
        'id_peak', idpk);
    fprintf(['   with the dq cross-coupling at %.2f pu speed: slowest pole %.0f %+.0fj rad/s, step of the q current ' ...
        '(feed-forward from the reference): 2 %% in %.1f ms, overshoot %.0f %%, peak |id| %.2f pu%s\n'], w_pu, ...
        real(s(k)), abs(imag(s(k))), t2*1e3, 100*over, idpk, flag(max(abs(eig(F)))));

    % ================================================================ 4. DC-link voltage loop
    ts = ca.ts;
    ug_pu = n21*sqrt(3)*param.grid.emu.us_primary_nom/param.grid.application_voltage;   % secondary [pu]
    Pbez  = 1.5*ha.ubez*ha.ibez;
    C_dc  = ha.Cdc + hv.Cdc;
    K_dc  = Pbez*ug_pu/(C_dc*ha.udc_nom^2);                 % d(udc_pu)/dt per pu of active current
    [fc, pm] = margins(@(z) (ca.kp_udc + ca.ki_udc*ts*z./(z - 1))./z.^2*K_dc*ts./(z - 1), ts);
    d.udc = struct('C', C_dc, 'K', K_dc, 'fc', fc, 'pm', pm);
    fprintf(['\n4. DC-link voltage loop: C = %.1f + %.1f mF, 1 pu = %.1f kW, plant %.1f 1/s, kp_udc = %.3g, ' ...
        'ki_udc = %.3g 1/s: crossover %s Hz, phase margin %s deg (reference filter %.0f ms, outside the loop)\n'], ...
        ha.Cdc*1e3, hv.Cdc*1e3, Pbez/1e3, K_dc, ca.kp_udc, ca.ki_udc, num(fc, '%.1f'), num(pm, '%.1f'), ca.udc_tau*1e3);
    fprintf('   no zero-sequence loop (u_cm = %g): the common-mode current through the cable capacitances is not controlled\n', ...
        ca.u_cm);

    % ================================================================ 5. operating point
    T_pu = param.load.torque_overload_factor*torque_curve(w_pu, c.curve_w, c.curve_T);
    T_pu = min(T_pu, c.torque_lim);
    T = T_pu*mc.tau_bez;
    P = T*w_pu*mc.omega_m_bez;
    u_ph = ug_pu*ha.ubez;                                   % [V] peak phase voltage at the secondary
    ig = P/(1.5*u_ph);
    m_afe = abs(u_ph + 1j*param.grid.emu.omega_grid_nom*(L1 + Lg)*ig)/Ku;
    [id, iq] = mtpa(-T_pu, c);                              % generator: the torque brakes the rotation
    lq_t = c.lq + l1 + l2;
    ld_t = c.ld + l1 + l2;
    ud = c.rs*id - w_pu*lq_t*iq;
    uq = c.rs*iq + w_pu*(ld_t*id + c.psi_m);
    u_inv = hypot(ud, uq)*hv.ubez;
    d.op = struct('speed', w_pu, 'T_pu', T_pu, 'P', P, 'ig', ig, 'm_afe', m_afe, 'id', id, 'iq', iq, 'u_inv', u_inv);
    fprintf(['\n5. Operating point at %.1f rpm (%.2f pu, %.1f Hz): torque %.1f kNm (%.3f pu of one system), %.0f kW ' ...
        '| AFE %.0f A peak (%.2f pu), modulation %.3f | inverter id %.3f, iq %.3f pu, |u| %.0f V peak of %.0f V ' ...
        '(udc/sqrt(3))\n'], param.load.rpm_sim, w_pu, w_pu*mc.freq_bez, T/1e3, T_pu, P/1e3, ig, ig/ha.ibez, m_afe, ...
        id, iq, u_inv, hv.udc_nom/sqrt(3));

    % ================================================================ 6. start-up
    tau_g = param.grid.emu.sof_50Hz.time_constant;          % amplitude filter of the emulator, 1/(tau*s + 1)^2
    u_rect = sqrt(3)*u_ph;                                  % [V] peak line-to-line at the secondary
    i_ch = C_dc*u_rect/(tau_g*exp(1));                      % C*max(du/dt) of the ramp
    r_rect = u_rect/ha.udc_nom;
    t_x = [];
    if r_rect < ca.udc_norm_ref
        t_x = ca.udc_t_step + ca.udc_tau*log(ca.udc_norm_ref/(ca.udc_norm_ref - r_rect));
    end
    t_T = c.t_enable + T_pu/c.torque_slew;
    d.start = struct('u_rect', u_rect, 'i_charge', i_ch, 't_ref_above_diodes', t_x, 't_torque', t_T);
    fprintf(['\n6. Start-up: the grid voltage rises with 1/(%.1f ms*s + 1)^2 from t = 0: the diodes of the AFE charge the ' ...
        'DC link (%.1f mF) to %.0f V with up to %.0f A (%.2f pu); no pre-charge circuit\n'], tau_g*1e3, C_dc*1e3, ...
        u_rect, i_ch, i_ch/ha.ibez);
    if ~isempty(t_x)
        fprintf(['   the DC-link reference (step at %.0f ms, low-pass %.0f ms, from 0) stays below the diode voltage ' ...
            '(%.2f pu) until %.0f ms: before, the PI asks for current into the grid to pull udc down (look at udc and ' ...
            'the AFE current in the first %.0f ms)\n'], ca.udc_t_step*1e3, ca.udc_tau*1e3, r_rect, t_x*1e3, ...
            ceil(t_x*1e3/10)*10);
    end
    fprintf(['   inverter: ref enable at %.3f s, gates on at %.3f s, torque ramp %.1f pu/s: %.3f pu reached at %.2f s ' ...
        '(StopTime %.2f s)\n'], param.inv.t_ref_enable, c.t_enable, c.torque_slew, T_pu, t_T, param.sim.simlength);

    % ================================================================ 7. clock alignment
    fprintf('\n7. Clock alignment (t_loc = t_master*k_time*(1 + corr), PI once per control period):\n');
    blk = {'AFE', a; 'inverter', v};
    d.align = struct('block', {}, 'k_time', {}, 'e_max_deg', {}, 't_aligned', {}, 't_unstable', {});
    for k = 1:2
        b = blk{k, 2};
        [e_max, t_al, t_un] = align_sim(b.ctrl, b.ctrl.ts, b.pwm.fpwm, param.sim.simlength);
        d.align(k) = struct('block', blk{k, 1}, 'k_time', b.ctrl.k_time, 'e_max_deg', e_max, 't_aligned', t_al, ...
            't_unstable', t_un);
        fprintf(['   %-9s k_time %.4f, kp %.3g, ki %.3g 1/s: largest carrier phase error %.0f deg, within 5 deg ' ...
            'from %s s; loop unstable after %.2f s%s\n'], blk{k, 1}, b.ctrl.k_time, b.ctrl.kp_align, b.ctrl.ki_align, ...
            e_max, num(t_al, '%.2f', 'never (in the run)'), t_un, ...
            iff(t_un < param.sim.simlength, '  (BEFORE the end of the run)', ''));
    end
    fprintf(['   Before the alignment the carriers of the AFE and of the inverter slide against each other: the ' ...
        'common-mode voltage at the cable changes pattern over time.\n']);
end

% ------------------------------------------------------------------------------------------------ cable
function [A, B, Cv, Ci] = ladder(Lsrc, Rsrc, cb)
%LADDER  one phase of the cable driven by a source voltage through Lsrc, Rsrc; far end open.
%   states [i_1 .. i_n; vc_1 .. vc_n]: i_k current of the series inductor of section k (into the section),
%   vc_k voltage of its capacitor (without RCs1); node voltage v_k = vc_k + RCs1*(i_k - i_k+1), i_n+1 = 0.
%   Outputs: Cv*x far-end voltage to ground, Ci*x current at the input of the cable.
    n = cb.n;
    L = cb.Ls1;
    R = cb.Rs1;
    C = cb.Cp1;
    Rc = cb.RCs1;
    A = zeros(2*n);
    B = zeros(2*n, 1);
    for k = 1:n
        Lk = L + (k == 1)*Lsrc;
        Rk = R + (k == 1)*Rsrc;
        row = zeros(1, 2*n);
        row(k) = -Rk - Rc;                                  % - R_k*i_k - (v_k: Rc*i_k)
        row(n + k) = -1;                                    % - vc_k
        if k < n
            row(k + 1) = Rc;                                % - (v_k: -Rc*i_k+1)
        end
        if k == 1
            B(1) = 1/Lk;                                    % + source voltage
        else
            row(n + k - 1) = row(n + k - 1) + 1;            % + v_k-1 = vc_k-1 + Rc*(i_k-1 - i_k)
            row(k - 1) = row(k - 1) + Rc;
            row(k) = row(k) - Rc;
        end
        A(k, :) = row/Lk;
        A(n + k, k) = 1/C;
        if k < n
            A(n + k, k + 1) = -1/C;
        end
    end
    Cv = zeros(1, 2*n);
    Cv(2*n) = 1;
    Cv(n) = Rc;
    Ci = zeros(1, 2*n);
    Ci(1) = 1;
end

function [y, y_be] = steps(A, B, C, h, nt)
%STEPS  unit step response: exact (zero-order hold over h) and Backward Euler at h, nt samples
    nx = size(A, 1);
    E = expm([A*h, B*h; zeros(1, nx + 1)]);
    Ad = E(1:nx, 1:nx);
    Bd = E(1:nx, nx + 1);
    M = inv(eye(nx) - h*A);
    x = zeros(nx, 1);
    xb = x;
    y = zeros(size(C, 1), nt);
    y_be = y;
    for k = 1:nt
        x = Ad*x + Bd;
        xb = M*(xb + h*B);
        y(:, k) = C*x;
        y_be(:, k) = C*xb;
    end
end

function k = decay_time(y)
%DECAY_TIME  last sample at which the ringing (y - final value) is above 5 % of the step (final value of the
%   far-end voltage = 1 for a unit step), in samples
    k = find(abs(y - 1) > 0.05, 1, 'last');
    if isempty(k)
        k = 0;
    end
end

% ------------------------------------------------------------------------------------------------ plants
function [a, b] = rl_d(r, l, wb, ts)
%RL_D  i(k+1) = a*i(k) + b*u(k) of l/wb*di/dt = u - r*i (pu), zero-order hold over ts.
    a = exp(-r*wb*ts/l);
    if r > 1e-12
        b = (1 - a)/r;
    else
        b = wb*ts/l;
    end
end

function L = inv_open(z, kp, ki, r, l, wb, ts)
    [a, b] = rl_d(r, l, wb, ts);
    L = (kp + ki*ts*z./(z - 1))./z.*b./(z - a);
end

function p = inv_poles(kp, ki, r, l, wb, ts)
%INV_POLES  states [i; i sampled at the previous trigger; integrator]
    [a, b] = rl_d(r, l, wb, ts);
    p = eig([a, -b*(kp + ki*ts), b; 1, 0, 0; 0, -ki*ts, 1]);
end

function [Ph, Ga] = rl_dq(r, L, w, wb, ts)
%RL_DQ  i(k+1) = Ph*i(k) + Ga*u(k) of L/wb*di/dt = u - (r + w*J*L)*i in the rotor dq frame (pu), ZOH over ts
    J = [0 -1; 1 0];
    A = -wb*(L\(r*eye(2) + w*J*L));
    B = wb*inv(L);
    E = expm([A*ts, B*ts; zeros(2, 4)]);
    Ph = E(1:2, 1:2);
    Ga = E(1:2, 3:4);
end

function F = inv_mimo(kp, ki, r, L, w, wb, ts)
%INV_MIMO  closed loop of the dq current PI with the cross-coupling, states [i; i(k-1); integrator] (2 each)
    [Ph, Ga] = rl_dq(r, L, w, wb, ts);
    I2 = eye(2);
    Z = zeros(2);
    F = [Ph, -Ga*(kp + ki*ts), Ga; I2, Z, Z; Z, -ki*ts*I2, I2];
end

function [t2, over, idpk] = inv_mimo_step(kp, ki, r, L, w, wb, ts)
%INV_MIMO_STEP  step of the q current reference (1 pu) with the feed-forward of the block (from the reference):
%   2 % settling time, overshoot of iq, peak |id|
    J = [0 -1; 1 0];
    [Ph, Ga] = rl_dq(r, L, w, wb, ts);
    ref = [0; 1];
    uff = (r*eye(2) + w*J*L)*ref;
    n = round(0.1/ts);
    i = zeros(2, 1);
    i_old = i;
    x = i;
    y = zeros(2, n);
    for k = 1:n
        e = ref - i_old;
        x = x + ki*ts*e;
        u = kp*e + x + uff;
        i_old = i;
        i = Ph*i + Ga*u;
        y(:, k) = i;
    end
    err = sqrt(y(1, :).^2 + (y(2, :) - 1).^2);
    t2 = find(err > 0.02, 1, 'last')*ts;
    if isempty(t2)
        t2 = 0;
    end
    over = max(y(2, :)) - 1;
    idpk = max(abs(y(1, :)));
end

function [Ad, Bd] = lcl_d(L1, R1, Cf, Rcf, Lg, Rg, ts)
%LCL_D  states [i1; vc; i2], input the converter voltage, zero-order hold over ts
    A = [-(R1 + Rcf)/L1, -1/L1, Rcf/L1; 1/Cf, 0, -1/Cf; Rcf/Lg, 1/Lg, -(Rcf + Rg)/Lg];
    B = [1/L1; 0; 0];
    E = expm([A*ts, B*ts; zeros(1, 4)]);
    Ad = E(1:3, 1:3);
    Bd = E(1:3, 4);
end

function L = afe_open(z, kp, ki, Ad, Bd, g, ts)
    L = zeros(size(z));
    C1 = [1 0 0];
    for k = 1:numel(z)
        G = C1*((z(k)*eye(3) - Ad)\Bd)*g;
        L(k) = (kp + ki*ts*z(k)/(z(k) - 1))*G/z(k);
    end
end

function p = afe_poles(kp, ki, Ad, Bd, g, ts)
%AFE_POLES  states [i1; vc; i2; i1 sampled at the previous trigger (pu); integrator]
    A = zeros(5);
    A(1:3, 1:3) = Ad;
    A(1:3, 4) = -Bd*g*(kp + ki*ts);
    A(1:3, 5) = Bd*g;
    A(4, 1) = 1;
    A(5, 4) = -ki*ts;
    A(5, 5) = 1;
    p = eig(A);
end

% ------------------------------------------------------------------------------------------------ control
function T = torque_curve(w, cw, cT)
%TORQUE_CURVE  torque_curve of the control of pmsm_inverter_dqctrl: linear interpolation, linear to zero below
%   cw(1), cT(end) above cw(end)
    n = numel(cw);
    if w <= cw(1)
        T = cT(1)*w/cw(1);
    elseif w >= cw(n)
        T = cT(n);
    else
        T = interp1(cw, cT, w);
    end
end

function [id, iq] = mtpa(T, c)
%MTPA  current references [pu] of the torque T [pu]: torque_to_current of the control of the block (MTPA with
%   |id| <= id_max, or id = 0; current vector limit i_lim with priority to iq)
    id = 0;
    iq = T/c.psi_m;
    dl = c.lq - c.ld;
    if c.mtpa ~= 0 && abs(dl) > 1e-6
        a = c.psi_m/(2*dl);
        for n = 1:4
            id = a - sign(a)*sqrt(a*a + iq*iq);
            id = min(max(id, -c.id_max), c.id_max);
            iq = T/(c.psi_m - dl*id);
        end
    end
    id = min(max(id, -c.i_lim), c.i_lim);
    if id*id + iq*iq > c.i_lim^2
        iq = sign(iq)*sqrt(c.i_lim^2 - id*id);
    end
end

function [e_max, t_al, t_un] = align_sim(c, ts, fpwm, t_end)
%ALIGN_SIM  alignment PI of the block, once per control period: the local trigger j comes at t_loc = j*ts,
%   i.e. t_master = j*ts/(k_time*(1 + corr)). Carrier phase error 360*fpwm*(t_master - t_loc) [deg]; time after
%   which it stays within 5 deg; time after which the linearized loop is unstable (gain kp*k_time*t).
    N = floor(t_end/ts);
    corr = 0;
    x = 0;
    e_deg = zeros(1, N);
    tm = zeros(1, N);
    for j = 1:N
        t_m = j*ts/(c.k_time*(1 + corr));
        e = t_m - j*ts;
        P = c.kp_align*e;
        y = min(max(P + c.ki_align*ts*e + x, c.lim_down_align), c.lim_up_align);
        x = y - P;
        corr = y;
        e_deg(j) = 360*fpwm*e;
        tm(j) = t_m;
    end
    e_wrap = abs(mod(e_deg + 180, 360) - 180);
    e_max = max(e_wrap);
    k = find(e_wrap > 5, 1, 'last');
    if isempty(k)
        t_al = 0;
    elseif k < N
        t_al = tm(k + 1);
    else
        t_al = [];
    end
    t_un = min(1/c.kp_align, 2/(2*c.kp_align + c.ki_align*ts))/c.k_time;
end

% ------------------------------------------------------------------------------------------------ tools
function [fc, pm, gm] = margins(Lfun, ts, n)
%MARGINS  first crossover [Hz], phase margin [deg], gain margin [dB] (first -180 deg crossing) of L(z)
    if nargin < 3
        n = 40000;
    end
    f = logspace(-1, log10(0.4999/ts), n);
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

function [f, zeta] = top_mode(p, ts, f_ref)
%TOP_MODE  frequency [Hz] and damping of the complex pole with the largest angle (the LCL mode). A mode above
%   the Nyquist frequency 1/(2*ts) is aliased in z: its frequency is taken as the alias closest to f_ref
%   (the resonance of the filter), sigma = -log|z|/ts is the same for all the aliases.
    f = 0; zeta = 1; best = -1;
    ws = 2*pi/ts;
    for k = 1:numel(p)
        if imag(p(k)) > 1e-9 && angle(p(k)) > best
            best = angle(p(k));
            sigma = -log(abs(p(k)))/ts;
            wa = angle(p(k))/ts + ws*(-3:3);
            [~, j] = min(abs(abs(wa) - 2*pi*f_ref));
            w = abs(wa(j));
            f = w/2/pi;
            zeta = sigma/hypot(sigma, w);
        end
    end
end

function kp_lim = kp_limit(poles_of_kp, kp0, kp_max)
%KP_LIMIT  smallest kp from kp0 (the gain of the init) up to kp_max with a closed-loop pole on or outside the
%   unit circle ([] if none)
    kp_lim = [];
    for kp = linspace(kp0, kp_max, 800)
        if max(abs(poles_of_kp(kp))) >= 1
            kp_lim = kp;
            return
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

function s = iff(c, a, b)
    if c
        s = a;
    else
        s = b;
    end
end
