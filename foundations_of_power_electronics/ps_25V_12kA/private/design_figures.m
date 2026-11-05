function fig = design_figures(param)
%DESIGN_FIGURES  Design figures of ps_25V_12kA_r.slx from the data of the init (printed; fig returns the
%   numbers). Averaged models without switching: estimates to compare with the simulation.
%
%   fig = design_figures(param)
%
%   1. Full bridge at the rated output (param.load.u_out, param.load.i_out): secondary voltage at full duty,
%      duty lost in the commutation of the centre-tapped rectifier, voltage drops, duty needed; largest duty
%      with the DC transformer current compensation at its limit, and the output at that duty.
%   2. Output current loop of the full bridge: crossover, phase and gain margins, closed-loop poles, largest
%      stable kp_idc.
%   3. AFE: power drawn from the DC link, grid current, LCL resonance, converter current loop and DC-link
%      voltage loop (crossover, phase and gain margins).
%
%   Models
%   - Full bridge, averaged: source n*udc*D (n = n2/n1) behind the equivalent resistance of the commutation
%     Rc = n*udc*dD/Io, the resistances of the path (diode, secondary and primary windings, two switches,
%     output inductor) and the diode threshold, then Ldc_dc2, the output capacitor Cdc_dc2 + RCdc_dc2 and the
%     load R. Commutation of the centre-tapped rectifier: both diodes conduct, the secondary is shorted and
%     udc drives the primary current through Ld1 plus the two secondary leakages in parallel referred to the
%     primary (Ld1/2): the primary current changes by 2*n*Io in 1.5*Ld1*2*n*Io/udc, twice per period, so
%     dD = 4*1.5*Ld1*fpwm*n*Io/udc.
%   - Modulator (library full_bridge_zvs_pwm_modulator): both legs get the duty d1 (0.5 plus the output of the
%     DC transformer current compensation, limited at +-lim_idc_trafo) and leg B is shifted by
%     phase_displacement*pi: the bridge applies +-udc for D = min(phase_displacement, 2*min(d1, 1 - d1)) of
%     each half period. The compensation reads the DC input current of the bridge, which is positive in its
%     samples (centre of the pulses), so with load it sits at -lim_idc_trafo: d1 = 0.5 - lim_idc_trafo and the
%     largest D is 1 - 2*lim_idc_trafo.
%   - Output current loop: the PI of the model (pi_ctrl of the library, kp_idc + ki_idc*ts/(1 - 1/z)) runs at
%     ts_psfbc on the load current, which reaches it through a Unit Delay (one sample); zero-order hold of the
%     phase displacement; the plant above discretized exactly (expm).
%   - AFE: converter-side current loop on the LCL filter (LFu with RLFu, CFu + RCFu in star, grid side =
%     leakage of the transformer + equivalent grid inductance with their resistances), 1 pu of controller
%     output = udc/sqrt(3) peak phase voltage (gain sqrt(3)/2 into the space vector modulator, which gives
%     2/3*udc per unit), 1.5 samples of delay. DC-link loop: the closed current loop and the capacitance
%     CFi + Cdc_dc1 (the input capacitor of the full bridge is on the same bus), plant
%     1.5*ubez*ibez/(C*udc^2) per unit of active current.
%   Plain MATLAB (expm, eig), no toolbox.

    fig = struct();

    %% 1. full bridge at the rated output
    ld  = param.load;
    hw  = param.psfbc.hw;
    dio = param.diode.rectifier;
    c   = param.psfbc.ctrl;
    fs  = param.glb_time.fPWM_PSFBC;
    udc = hw.udc1_nom*param.afe.ctrl.udc_norm_ref;      % [V] DC link
    n   = hw.n2/hw.n1;                                  % [-] each secondary half / primary
    Io  = ld.i_out;                                     % [A]
    Ip  = n*Io;                                         % [A] primary current
    Vs  = n*udc;                                        % [V] secondary voltage at full duty
    dD  = 4*1.5*hw.Ld1*fs*Ip/udc;                       % [-] duty lost in the commutation
    Rc  = Vs*dD/Io;                                     % [Ohm] its equivalent output resistance
    [Rsw, sw_name] = switch_resistance(param, Ip);      % [Ohm] one switch of the bridge
    Rpath = dio.Rdon + hw.Rs2 + (hw.Rs1 + 2*Rsw)*n^2 + hw.RLdc_dc2;   % [Ohm] referred to the output
    D_need = (ld.u_out + dio.Vf + Io*(Rpath + Rc))/Vs;
    d1 = 0.5 - c.lim_idc_trafo;                         % compensation at its lower limit
    D_max = 2*min(d1, 1 - d1);
    Io_max = (Vs*D_max - dio.Vf)/(ld.R + Rpath + Rc);   % [A] output at the largest duty
    D_op = min(D_need, D_max);
    Io_op = min(Io, Io_max);
    P_cond = dio.Vf*Io_op + Rpath*Io_op^2;              % [W] conduction losses
    P_fe = udc^2*D_op/hw.Rfe;                           % [W] iron losses of the HF transformer
    P_dc = ld.R*Io_op^2 + P_cond + P_fe;                % [W] drawn from the DC link

    fig.fb = struct('n', n, 'Vs', Vs, 'dD', dD, 'Rc', Rc, 'Rpath', Rpath, 'D_need', D_need, 'd1', d1, ...
        'D_max', D_max, 'Io_max', Io_max, 'P_cond', P_cond, 'P_fe', P_fe, 'P_dc', P_dc);
    fprintf('1. Full bridge at %.1f V, %.0f A (%.0f kW): n*udc = %.2f V (udc %.0f V, turns %d:%d), commutation dD = %.3f (%.3f mOhm), path %.1f uOhm + diode %.2f V (%s %.2f mOhm)\n', ...
        ld.u_out, Io, ld.u_out*Io/1e3, Vs, udc, hw.n1, hw.n2, dD, Rc*1e3, Rpath*1e6, dio.Vf, sw_name, Rsw*1e3);
    if D_need <= D_max && c.lim_idc_trafo == 0
        fprintf('   duty needed %.3f, largest duty %.3f (DC transformer current compensation off): margin %.3f\n', ...
            D_need, D_max, D_max - D_need);
    elseif D_need <= D_max
        fprintf('   duty needed %.3f, largest duty %.3f (d1 = %.2f with the DC transformer current compensation at its limit): margin %.3f\n', ...
            D_need, D_max, d1, D_max - D_need);
    else
        fprintf(['   duty needed %.3f, largest duty %.3f: the DC transformer current compensation sits at -%.2f (d1 = %.2f), ' ...
            'the current loop saturates at about %.0f A, %.2f V. lim_idc_trafo = 0 gives D up to 1 (%.0f A possible).\n'], ...
            D_need, D_max, c.lim_idc_trafo, d1, Io_max, Io_max*ld.R, (Vs - dio.Vf)/(ld.R + Rpath + Rc));
    end
    fprintf('   power from the DC link about %.0f kW (conduction %.1f kW, iron %.1f kW)\n', P_dc/1e3, P_cond/1e3, P_fe/1e3);

    %% 2. output current loop of the full bridge
    ts = param.glb_time.ts_psfbc;
    L = hw.Ldc_dc2;  C = hw.Cdc_dc2;  esr = hw.RCdc_dc2;  R = ld.R;
    Rs = Rc + Rpath;
    k = R/(R + esr);                                    % node voltage v = k*(vC + esr*iL)
    A = [-(Rs + k*esr)/L, -k/L; (1 - k*esr/R)/C, -k/(R*C)];
    B = [Vs/L; 0];
    Cm = [k*esr/R, k/R]/hw.idc2_bez;                    % load current in pu (measurement of the model)
    G0 = Cm*(-A\B);
    M = expm([A, B; zeros(1, 3)]*ts);
    Ad = M(1:2, 1:2);  Bd = M(1:2, 3);
    kp = c.kp_idc;  ki = c.ki_idc;
    f = logspace(0, log10(0.999/(2*ts)), 6000);
    Lfb = zeros(size(f));
    for q = 1:numel(f)
        z = exp(1j*2*pi*f(q)*ts);
        Lfb(q) = (kp + ki*ts/(1 - 1/z))*(Cm*((z*eye(2) - Ad)\Bd))/z;
    end
    [fc, pm, gm] = margins(f, Lfb);
    [p_res, p_slow] = cl_poles(cl_matrix(Ad, Bd, Cm, kp, ki, ts), ts);
    kp_lim = kp_limit(Ad, Bd, Cm, ki, ts);
    f0 = 1/(2*pi*sqrt(L*C));
    fig.fb_loop = struct('G0', G0, 'f0', f0, 'fc', fc, 'pm', pm, 'gm', gm, 'f_res', p_res(1), 'zeta_res', p_res(2), ...
        'slow_pole', p_slow, 'kp_limit', kp_lim);
    fprintf(['2. Full-bridge output current loop: kp_idc = %.2f, ki_idc = %.0f 1/s, ts = %.1f us, plant %.3f pu/pu at DC, output ' ...
        'filter %.0f Hz (Z0 %.2f mOhm, load %.2f mOhm)\n'], kp, ki, ts*1e6, G0, f0, sqrt(L/C)*1e3, R*1e3);
    fprintf('   crossover %.0f Hz, phase margin %.1f deg, gain margin %.1f dB | closed loop: pair at %.0f Hz damped %.3f, slow pole %.1f rad/s | stable for kp_idc < %.2f\n', ...
        fc, pm, gm, p_res(1), p_res(2), p_slow, kp_lim);

    %% 3. AFE
    a = param.afe.hw;  ac = param.afe.ctrl;  tsa = param.glb_time.ts_afe;
    emu = param.grid.emu;  tr = emu.trafo;
    i_act = P_dc/(1.5*a.ubez*a.ibez);                   % [pu] active current at 1 pu of voltage
    K = udc/sqrt(3)/a.ubez;                             % converter voltage per pu of controller output
    if tr.delta_star == 1
        k1 = 3;                                         % delta primary coil
    else
        k1 = 1;
    end
    L1 = a.LFu;  R1 = a.RLFu;  Cf = a.CFu;  Rcf = a.RCFu;
    L2 = tr.Lsigma + emu.eq_grid_inductance;            % [H] star equivalent per phase
    R2 = tr.Rd1/k1 + tr.Rd2*tr.n12^2 + emu.eq_grid_resistance;
    fres = sqrt((L1 + L2)/(L1*L2*Cf))/(2*pi);
    Zb = a.ubez/a.ibez;
    fa = logspace(0, log10(0.999/(2*tsa)), 6000);
    w = 2*pi*fa;  z = exp(1j*w*tsa);
    Zc = Rcf + 1./(1j*w*Cf);  Z2 = 1j*w*L2 + R2;
    G = K*Zb./(1j*w*L1 + R1 + Zc.*Z2./(Zc + Z2));
    Li = (ac.kp_is_xi + ac.ki_is_xi*tsa./(1 - 1./z)).*G.*exp(-1j*w*1.5*tsa);
    [fci, pmi, gmi] = margins(fa, Li);
    Cdc = a.CFi + hw.Cdc_dc1;
    kdc = 1.5*a.ubez*a.ibez/(Cdc*udc^2);                % [1/s] per pu of active current
    Lu = (ac.kp_udc_ctrl + ac.ki_udc_ctrl*tsa./(1 - 1./z)).*(Li./(1 + Li))*kdc./(1j*w);
    [fcu, pmu, gmu] = margins(fa, Lu);
    fig.afe = struct('P_dc', P_dc, 'i_act', i_act, 'K', K, 'L2', L2, 'fres', fres, 'fc_i', fci, 'pm_i', pmi, ...
        'gm_i', gmi, 'Cdc', Cdc, 'kdc', kdc, 'fc_udc', fcu, 'pm_udc', pmu, 'gm_udc', gmu);
    fprintf('3. AFE: %.0f kW from the grid = %.2f pu of active current (%.0f A peak, limit %.2f pu) | LCL: LFu %.0f uH, CFu %.0f uF, grid side %.1f uH -> %.0f Hz (fs/6 = %.0f Hz)\n', ...
        P_dc/1e3, i_act, i_act*a.ibez, ac.ixi_pos_ref_lim, L1*1e6, Cf*1e6, L2*1e6, fres, 1/tsa/6);
    fprintf('   current loop kp = %.2f, ki = %.0f: crossover %.0f Hz, phase margin %.1f deg, gain margin %.1f dB\n', ...
        ac.kp_is_xi, ac.ki_is_xi, fci, pmi, gmi);
    fprintf('   DC-link loop kp = %.2f, ki = %.0f, C = %.1f mF (plant %.0f 1/s): crossover %.0f Hz, phase margin %.1f deg, gain margin %.1f dB\n', ...
        ac.kp_udc_ctrl, ac.ki_udc_ctrl, Cdc*1e3, kdc, fcu, pmu, gmu);
end

function [R, name] = switch_resistance(param, Ip)
% on-state resistance of one switch of the full bridge in use (at the primary current Ip)
    if param.sim.use_mosfet_thermal_model
        R = param.psfbc.mosfet.Rds_on;
        name = 'MOSFET';
    elseif param.sim.use_thermal_model
        R = param.psfbc.igbt.Rce_on + param.psfbc.igbt.Vce_sat/Ip;
        name = 'IGBT';
    else
        R = param.psfbc.ideal_switch.Rds_on;
        name = 'ideal switch';
    end
end

function [fc, pm, gm] = margins(f, L)
% crossover (last crossing of |L| = 1 going down), phase margin there, gain margin at the crossings of
% -180 deg (+-k*360)
    mag = abs(L);
    ph = unwrap(angle(L));
    i = find(mag(1:end-1) >= 1 & mag(2:end) < 1, 1, 'last');
    if isempty(i)
        fc = NaN;  pm = NaN;
    else
        r = (1 - mag(i))/(mag(i+1) - mag(i));
        fc = f(i) + r*(f(i+1) - f(i));
        pm = 180 + (ph(i) + r*(ph(i+1) - ph(i)))*180/pi;
    end
    q = floor((ph + pi)/(2*pi));                        % changes where the phase crosses -180 deg + k*360
    j = find(diff(q) ~= 0);
    gm = Inf;
    for m = 1:numel(j)
        jj = j(m);
        target = 2*pi*max(q(jj), q(jj+1)) - pi;
        r = (target - ph(jj))/(ph(jj+1) - ph(jj));
        mg = mag(jj) + r*(mag(jj+1) - mag(jj));
        gm = min(gm, -20*log10(mg));
    end
end

function Acl = cl_matrix(Ad, Bd, Cm, kp, ki, ts)
% closed loop, states [plant (2), delayed measurement, integrator]: e = r - y(k-1),
% u = kp*e + ki*ts*e + x, x(k+1) = x + ki*ts*e (pi_ctrl of the library without saturation)
    Acl = zeros(4);
    Acl(1:2, 1:2) = Ad;
    Acl(1:2, 3) = -(kp + ki*ts)*Bd;
    Acl(1:2, 4) = Bd;
    Acl(3, 1:2) = Cm;
    Acl(4, 3) = -ki*ts;
    Acl(4, 4) = 1;
end

function [p_res, p_slow] = cl_poles(Acl, ts)
% least damped complex pair [frequency in Hz, damping] and slowest pole (real part in rad/s)
    s = log(eig(Acl))/ts;
    cplx = s(abs(imag(s)) > 1e-6 & imag(s) > 0);
    if isempty(cplx)
        p_res = [NaN, NaN];
    else
        zeta = -real(cplx)./abs(cplx);
        [zm, m] = min(zeta);
        p_res = [abs(cplx(m))/(2*pi), zm];
    end
    [~, m] = min(abs(s));
    p_slow = real(s(m));
end

function kp_lim = kp_limit(Ad, Bd, Cm, ki, ts)
% largest kp with all closed-loop poles inside the unit circle (bisection, ki fixed)
    lo = 0;  hi = 50;
    for it = 1:60
        mid = (lo + hi)/2;
        if max(abs(eig(cl_matrix(Ad, Bd, Cm, mid, ki, ts)))) < 1
            lo = mid;
        else
            hi = mid;
        end
    end
    kp_lim = lo;
end
