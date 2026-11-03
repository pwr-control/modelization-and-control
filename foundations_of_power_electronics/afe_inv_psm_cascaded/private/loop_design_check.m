function d = loop_design_check(param, show_plots)
%LOOP_DESIGN_CHECK  Design figures of the control loops of afe_inv_psm_n (two modules in hard parallel).
%
%   d = loop_design_check(param)          prints the tables and returns the numbers in the struct d
%   d = loop_design_check(param, true)    also draws the open loops of the two current loops
%
%   Exact discrete-time loops with the timing of the two library blocks: the sample taken at trigger k-1 is
%   used at trigger k, the output computed at trigger k is applied from t(k) to t(k+1) (zero-order hold of
%   one control period ts). Module 1 is analysed: the two modules have the same gains (only the clock ratio
%   k_time and the zero-sequence factor u_cm differ).
%     1. inverter current loop (pmsm_inverter_dqctrl, dq PI on the output current, pu of the machine):
%        sum mode = machine current (plant: machine + filter of one system), difference mode = current that
%        circulates between the two inverters (plant: the filter of one module only, 230 uH)
%     2. AFE current loop (grid_following_dqctrl, dq PI on the converter current) with the LCL filter: the two
%        modules on the transformer (sum mode, each module sees twice the leakage of the transformer), one
%        module alone, difference of the two AFEs (only L2 of the blocks between the two capacitor banks)
%     3. DC-link voltage loop (C = Cdc of the AFE block + Cdc of the inverter block of the module)
%     4. zero-sequence current loop (path through both AFEs, both DC links and both inverters; one PI, in the
%        AFE whose u_cm is not 0)
%     5. operating point at param.load.rpm_sim (torque curve times param.inv.ref(2))
%   Small-signal, linear: limits, dead time and the sampling of the PWM are left out. Same model as the
%   Python design check of the README (design_check_cascade.py).

    if nargin < 2
        show_plots = false;
    end
    a  = param.afe_follower(1);
    v  = param.inv_mod(1);
    mc = param.psm.machine;
    tr = param.grid.emu.trafo;
    ts = v.ctrl.ts;

    % ================================================================ 1. inverter current loop
    c  = v.ctrl;
    hv = v.hw;
    wb = c.omega_b;
    zb = hv.ubez/hv.ibez;
    lb = zb/wb;
    l1 = (hv.L1p + hv.Lls1)/lb;
    r1 = hv.R1/zb;
    l2 = (hv.L2p + hv.Lls2)/lb;
    r2 = hv.R2/zb;
    inv_modes = {'sum (machine), q axis',    c.rs + r1 + r2, c.lq + l1 + l2; ...
                 'sum (machine), d axis',    c.rs + r1 + r2, c.ld + l1 + l2; ...
                 'difference (circulating)', r1 + r2,        l1 + l2};
    fprintf('1. Inverter current loop: kp_i = %.3g, ki_i = %.3g 1/s, ts = %.1f us, filter %.0f uH = %.4f pu\n', ...
        c.kp_i, c.ki_i, ts*1e6, (hv.L1p + hv.Lls1 + hv.L2p + hv.Lls2)*1e6, l1 + l2);
    fprintf('   %-26s %8s %8s %8s %8s %9s %10s\n', 'mode', 'l/pu', 'fc/Hz', 'PM/deg', 'GM/dB', '|z|max', 'kp limit');
    d.inv = struct('mode', {}, 'l', {}, 'fc', {}, 'pm', {}, 'gm', {}, 'rad', {}, 'zeta', {}, 'kp_lim', {});
    for k = 1:size(inv_modes, 1)
        [r, l] = inv_modes{k, 2:3};
        [fc, pm, gm] = margins(@(z) inv_open(z, c.kp_i, c.ki_i, r, l, wb, ts), ts);
        [rad, zeta] = worst(inv_poles(c.kp_i, c.ki_i, r, l, wb, ts), ts);
        kp_lim = kp_limit(@(kp) inv_poles(kp, c.ki_i, r, l, wb, ts), 20);
        d.inv(k) = struct('mode', inv_modes{k, 1}, 'l', l, 'fc', fc, 'pm', pm, 'gm', gm, 'rad', rad, ...
            'zeta', zeta, 'kp_lim', kp_lim);
        fprintf('   %-26s %8.4f %8s %8s %8s %9.4f %10s%s\n', inv_modes{k, 1}, l, num(fc, '%.1f'), num(pm, '%.1f'), ...
            num(gm, '%.1f'), rad, num(kp_lim, '%.2f', '> 20'), flag(rad));
    end
    fprintf(['   The difference mode sees only the filter of one module: it sets the limit of kp_i (the gains of ' ...
        'pmsm_inverter_param_example, 250 Hz on the machine, are unstable here).\n']);
    % machine current with the dq cross-coupling: at rated speed w*L is about 1 pu, as large as kp_i, and the
    % feed-forward of the block is computed from the references, so it does not act on a current error
    w_pu = param.load.rpm_sim/mc.rpm_nom;
    Lsum = diag([c.ld, c.lq]) + (l1 + l2)*eye(2);
    rsum = c.rs + r1 + r2;
    F = inv_mimo(c.kp_i, c.ki_i, rsum, Lsum, w_pu, wb, ts);
    s = log(complex(eig(F)))/ts;
    [~, k] = max(real(s));
    [t2, over, idpk] = inv_mimo_step(c.kp_i, c.ki_i, rsum, Lsum, w_pu, wb, ts);
    d.inv_mimo = struct('speed', w_pu, 'pole', s(k), 'tau', -1/real(s(k)), 't_settle', t2, 'overshoot', over, ...
        'id_peak', idpk);
    fprintf(['   machine current with the dq cross-coupling (w*L at %.2f pu speed, not in the feed-forward of the ' ...
        'error): slowest pole %.1f %+.1fj rad/s (time constant %.0f ms); step of the q current: 2 %% settling %.0f ms, ' ...
        'overshoot %.0f %%, peak |id| %.2f pu (the torque reference has a slew limit, so steps do not occur)\n'], ...
        w_pu, real(s(k)), abs(imag(s(k))), -1e3/real(s(k)), t2*1e3, over*100, idpk);

    % ================================================================ 2. AFE current loop with the LCL filter
    ha = a.hw;
    ca = a.ctrl;
    Ku = ha.udc_nom/sqrt(3);                                % [V] per unit of modulation
    n21 = tr.n2/tr.n1;
    Lt  = tr.Ld2 + tr.Ld1*n21^2;                            % [H] transformer leakage at the secondary
    Rt  = tr.Rd2 + tr.Rd1*n21^2;                            % [Ohm]
    L2  = ha.L2p + ha.Lls2;
    afe_modes = {'two modules (sum)',          L2 + 2*Lt, ha.R2 + 2*Rt; ...
                 'one module alone',           L2 + Lt,   ha.R2 + Rt; ...
                 'difference of the two AFEs', L2,        ha.R2};
    fprintf(['\n2. AFE current loop: kp_i = %.3g, ki_i = %.3g 1/s, L1 = %.0f uH, Cf = %.0f uF, Rcf = %.0f mOhm, ' ...
        'transformer leakage %.1f uH at the secondary\n'], ca.kp_i, ca.ki_i, (ha.L1p + ha.Lls1)*1e6, ha.Cf*1e6, ...
        ha.Rcf*1e3, Lt*1e6);
    fprintf('   %-28s %8s %8s %8s %8s %8s %16s %9s\n', 'mode', 'Lg/uH', 'fres/Hz', 'fc/Hz', 'PM/deg', 'GM/dB', ...
        'LCL mode', 'kp limit');
    d.afe = struct('mode', {}, 'Lg', {}, 'f_res', {}, 'fc', {}, 'pm', {}, 'gm', {}, 'f_mode', {}, 'zeta_mode', {}, ...
        'rad', {}, 'kp_lim', {});
    L1 = ha.L1p + ha.Lls1;
    for k = 1:size(afe_modes, 1)
        [Lg, Rg] = afe_modes{k, 2:3};
        [Ad, Bd] = lcl_d(L1, ha.R1, ha.Cf, ha.Rcf, Lg, Rg, ts);
        f_res = sqrt((L1 + Lg)/(L1*Lg*ha.Cf))/2/pi;
        [fc, pm, gm] = margins(@(z) afe_open(z, ca.kp_i, ca.ki_i, Ad, Bd, Ku/ha.ibez, ts), ts, 6000);
        p = afe_poles(ca.kp_i, ca.ki_i, Ad, Bd, Ku/ha.ibez, ts);
        [f_mode, z_mode] = top_mode(p, ts);
        kp_lim = kp_limit(@(kp) afe_poles(kp, ca.ki_i, Ad, Bd, Ku/ha.ibez, ts), 2.5);
        rad = max(abs(p));
        d.afe(k) = struct('mode', afe_modes{k, 1}, 'Lg', Lg, 'f_res', f_res, 'fc', fc, 'pm', pm, 'gm', gm, ...
            'f_mode', f_mode, 'zeta_mode', z_mode, 'rad', rad, 'kp_lim', kp_lim);
        fprintf('   %-28s %8.1f %8.0f %8s %8s %8s %7.0f Hz %5.1f %% %9s%s\n', afe_modes{k, 1}, Lg*1e6, f_res, ...
            num(fc, '%.1f'), num(pm, '%.1f'), num(gm, '%.1f'), f_mode, 100*z_mode, num(kp_lim, '%.2f', '> 2.5'), flag(rad));
    end
    fprintf(['   A loop on the converter current with 1.5 periods of delay damps the LCL resonance only below ' ...
        'fs/6 = %.0f Hz: the two-module resonance is above it, its damping comes from Rcf and R1.\n'], 1/(6*ts));

    % ================================================================ 3. DC-link voltage loop
    ug_pu = n21*sqrt(3)*param.grid.emu.us_primary_nom/param.grid.application_voltage;   % secondary [pu]
    Pbez  = 1.5*ha.ubez*ha.ibez;
    C_dc  = ha.Cdc + hv.Cdc;
    K_dc  = Pbez*ug_pu/(C_dc*ha.udc_nom^2);                 % d(udc_pu)/dt per pu of active current
    [fc, pm] = margins(@(z) (ca.kp_udc + ca.ki_udc*ts*z./(z - 1))./z.^2*K_dc*ts./(z - 1), ts);
    d.udc = struct('C', C_dc, 'K', K_dc, 'fc', fc, 'pm', pm);
    fprintf(['\n3. DC-link voltage loop: C = %.1f + %.1f mF, 1 pu = %.1f kW, plant %.1f 1/s, kp_udc = %.3g, ' ...
        'ki_udc = %.3g 1/s: crossover %s Hz, phase margin %s deg\n'], ha.Cdc*1e3, hv.Cdc*1e3, Pbez/1e3, K_dc, ...
        ca.kp_udc, ca.ki_udc, num(fc, '%.1f'), num(pm, '%.1f'));

    % ================================================================ 4. zero-sequence current loop
    hw2 = [a.hw, param.afe_follower(end).hw];
    L_loop = 0;
    for h = [hw2, v.hw, param.inv_mod(end).hw]
        L_loop = L_loop + h.L10 + h.Lls1 + h.L20 + h.Lls2;
    end
    u_cm = arrayfun(@(x) x.ctrl.u_cm, param.afe_follower);
    act = find(u_cm ~= 0);
    fprintf('\n4. Zero-sequence current loop: loop inductance %.0f uH, u_cm of the AFEs = %s\n', L_loop*1e6, mat2str(u_cm));
    d.zero = struct('L', L_loop, 'u_cm', u_cm, 'fc', [], 'pm', []);
    if isempty(act)
        fprintf('   no AFE acts on the zero sequence (u_cm = 0): the circulating zero-sequence current is not controlled\n');
    elseif numel(act) > 1
        fprintf('   %d AFEs act on the same zero-sequence loop: their integrators fight, keep one u_cm ~= 0\n', numel(act));
    else
        z = param.afe_follower(act).ctrl;
        if z.u_cm*z.iz_out_gain > 0
            fprintf('   u_cm*iz_out_gain > 0 in module %d: POSITIVE feedback (the loop needs u_cm < 0)\n', act);
        end
        for t0 = [0.1 0.2 0.4]
            Kz = ha.udc_nom*t0*abs(z.u_cm*z.iz_out_gain)/(2*L_loop*ha.ibez);
            [fc, pm] = margins(@(zz) (z.kp_iz + z.ki_iz*ts*zz./(zz - 1))./zz*Kz*ts./(zz - 1), ts);
            d.zero.fc(end+1) = fc;
            d.zero.pm(end+1) = pm;
            fprintf('   module %d, zero-vector time %.1f: plant %.0f 1/s, crossover %s Hz, phase margin %s deg\n', act, ...
                t0, Kz, num(fc, '%.1f'), num(pm, '%.1f'));
        end
    end

    % ================================================================ 5. operating point
    w_pu = param.load.rpm_sim/mc.rpm_nom;
    T_pu = param.inv.ref(2)*interp1([0, c.curve_w], [0, c.curve_T], min(w_pu, c.curve_w(end)));
    T = T_pu*mc.tau_bez;
    P = T*w_pu*mc.omega_m_bez;
    u_ph = ug_pu*ha.ubez;                                   % [V] peak phase voltage at the secondary
    ig = P/(1.5*u_ph);
    m_afe = abs(u_ph + 1j*param.grid.emu.omega_grid_nom*L1*ig)/Ku;
    [id, iq] = mtpa(-T_pu, c);                              % generator: the torque brakes the rotation
    lq_t = c.lq + l1 + l2;
    ld_t = c.ld + l1 + l2;
    ud = c.rs*id - w_pu*lq_t*iq;
    uq = c.rs*iq + w_pu*(ld_t*id + c.psi_m);
    u_inv = hypot(ud, uq)*hv.ubez;
    d.op = struct('T_pu', T_pu, 'P', P, 'ig', ig, 'm_afe', m_afe, 'id', id, 'iq', iq, 'u_inv', u_inv);
    fprintf(['\n5. Operating point at %.1f rpm: torque %.0f kNm per module (%.3f pu), %.0f kW per module | AFE %.0f A ' ...
        'peak (%.2f pu), modulation %.3f | inverter id %.3f, iq %.3f pu, |u| %.0f V peak of %.0f V (udc/sqrt(3))\n'], ...
        param.load.rpm_sim, T/1e3, T_pu, P/1e3, ig, ig/ha.ibez, m_afe, id, iq, u_inv, hv.udc_nom/sqrt(3));

    if show_plots
        f = logspace(0, log10(0.4999/ts), 2000);
        zf = exp(1j*2*pi*f*ts);
        figure('Name', 'afe_inv_psm_n: current loops');
        subplot(1, 2, 1);
        for k = 1:size(inv_modes, 1)
            semilogx(f, 20*log10(abs(inv_open(zf, c.kp_i, c.ki_i, inv_modes{k, 2}, inv_modes{k, 3}, wb, ts))));
            hold on;
        end
        grid on; xlabel('f [Hz]'); ylabel('|L| [dB]'); title('Inverter current loop');
        legend(inv_modes(:, 1), 'Location', 'southwest');
        subplot(1, 2, 2);
        for k = 1:size(afe_modes, 1)
            [Ad, Bd] = lcl_d(L1, ha.R1, ha.Cf, ha.Rcf, afe_modes{k, 2}, afe_modes{k, 3}, ts);
            semilogx(f, 20*log10(abs(afe_open(zf, ca.kp_i, ca.ki_i, Ad, Bd, Ku/ha.ibez, ts))));
            hold on;
        end
        grid on; xlabel('f [Hz]'); ylabel('|L| [dB]'); title('AFE current loop');
        legend(afe_modes(:, 1), 'Location', 'southwest');
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
%INV_MIMO  closed loop of the dq current PI with the cross-coupling, states [i; i(k-1); integrator] (2 each):
%   the sample of trigger k-1 is used at trigger k, x(k) = x(k-1) + ki*ts*e, u = kp*e + x (error dynamics)
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
    n = round(0.5/ts);
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

function [rad, zeta] = worst(p, ts)
%WORST  largest pole radius, smallest damping of the complex poles
    rad = max(abs(p));
    zeta = 1;
    for k = 1:numel(p)
        if abs(imag(p(k))) > 1e-9
            s = log(p(k))/ts;
            zeta = min(zeta, -real(s)/abs(s));
        end
    end
end

function [f, zeta] = top_mode(p, ts)
%TOP_MODE  frequency [Hz] and damping of the complex pole with the highest frequency (the LCL mode)
    f = 0; zeta = 1;
    for k = 1:numel(p)
        if imag(p(k)) > 1e-9
            s = log(p(k))/ts;
            if abs(s)/2/pi > f
                f = abs(s)/2/pi;
                zeta = -real(s)/abs(s);
            end
        end
    end
end

function kp_lim = kp_limit(poles_of_kp, kp_max)
%KP_LIMIT  smallest kp (0.05 ... kp_max) with a closed-loop pole on or outside the unit circle ([] if none)
    kp_lim = [];
    for kp = linspace(0.05, kp_max, 800)
        if max(abs(poles_of_kp(kp))) >= 1
            kp_lim = kp;
            return
        end
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
