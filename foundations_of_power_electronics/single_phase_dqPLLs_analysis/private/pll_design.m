function d = pll_design(param, options)
%PLL_DESIGN  Design check of the three single-phase PLLs of single_phase_pll_analysis: numbers in d, figures on request.
%
%   d = pll_design(param)             numbers only (and figures when param.sim.design_figures is true)
%   d = pll_design(param, options)    options: bodeoptions of the init for the Bode plots, or [] (Octave)
%
%   Park PLL (block single_phase_pll). The quadrature signal beta is the inverse Park transform of xi and eta
%   filtered by the first-order low passes tau (uxi_pll_flt, ueta_pll_flt); the direct Park transform of (u, beta)
%   at the PLL angle gives eta. With the PLL locked at omega this quadrature generator is a SOGI:
%       beta/u = omega wf / (s^2 + wf s + omega^2),  wf = 1/tau  (SOGI gain k = wf/omega)
%   and eta, linearized around the lock, is the phase error through the phase detector (averaged over a period)
%       Kpd(s) = V (s + wf) / (2 s + wf)        V: amplitude of u in pu, eta is not normalized
%   The block pi-ctrl (kp, ki, output saturated at sat_neg..sat_pos with back-calculation) adds to the nominal
%   frequency (Constant 1 pu); theta_k+1 = theta_k + omega_n ts omega_pu. Open loop, continuous:
%       L(s) = omega_n V (kp s + ki) (s + wf) / (s^2 (2 s + wf))
%   With ki = 0 (model) the loop is of type 1: s^2 + (wf + K)/2 s + K wf/2 with K = omega_n kp V, and a frequency
%   step d_omega leaves the phase error d_omega/K [rad]. The output phase_pll_hat is the estimate of the
%   double-integrator observer, which follows theta with no error at constant frequency.
%
%   FHT-PLL and DDSRF-PLL. The first harmonic tracker (FHT) is a Luenberger observer of the damped oscillator
%   x' = [0 1; -w^2 -delta w] x, forward Euler model Ad = I + A ts, gains Ld for the poles exp(ts [-1 -4] w)
%   (ctrl_afe_setup, fht_grid); alpha = x1 and beta = -x2/w are the estimates of step k (one step ahead).
%   d.fht: frequency response of alpha and beta at omega_n and at the harmonics, the positive sequence of
%   (alpha, beta) (gain and phase) and the negative sequence that remains (|alpha - j beta|/2, it puts a ripple at
%   2 omega_n on eta). The DDSRF-PLL reads (alpha, beta) of one FHT; the FHT-PLL filters alpha and beta with two
%   more FHTs and takes -1/2 (u - j v) (positive sequence), so its phase error is about twice the error of one
%   FHT. d.fht_undamped: the same numbers for the FHT on the undamped oscillator (delta = 0) with the exact
%   discretization Ad = expm(A ts), the option param.afe.pll.fht_undamped of the init.
%   Stage B (pll_stageB): eta of the positive sequence over its magnitude (1/|u| saturated at 1/sqrt(0.02)), PI in
%   omega pu (forward Euler, integrator clamped to [0.5 1.5] pu in the DDSRF-PLL):
%       T(s) = omega_n (kp s + ki) / (s^2 + omega_n kp s + omega_n ki),  wn = sqrt(omega_n ki),
%       zeta = omega_n kp / (2 wn)
%   d.fht_pll, d.ddsrf_pll: wn, zeta, poles, gain of T at 2 omega_n (ripple of eta from the negative sequence).
%   The trackers do not see theta: they are outside the loop and only delay the response (they set no margin).
%   The decoupling network of the DDSRF-PLL (four low passes wf = 1/tau_ddsrf, rotations by 2 theta) is inside
%   the loop; on a small phase error of a positive-sequence input it is the real transfer function
%       Deff(s) = wf (s^3 + 2 wf s^2 + 4 w^2 s + 4 w^2 wf) / (s^4 + 4 wf s^3 + 4 (wf^2 + w^2) s^2 + 8 w^2 wf s
%                 + 4 w^2 wf^2)
%   (d.ddsrf_pll.Deff, crossover and phase margin with it in wc_net, pm_net), and it cancels the negative sequence
%   in steady state (no 2 omega_n term). Design note: repos_doc/single_phase_lock_loop (rev. 01), linear models.
%   d.obs: discrete and continuous poles of the double-integrator observer (omega output) and its delay.
%   Figures: Bode of the closed phase loops (Park PLL at 1 pu and at the sag of the test, FHT-PLL, DDSRF-PLL),
%   phase response to a 0.1 rad jump, frequency response of the FHT outputs.

    if nargin < 2
        options = [];
    end
    use_options = ~isempty(options) && ~isstruct(options);     % bodeoptions object of MATLAB (Octave: none)
    c = param.afe.ctrl;
    pk = param.afe.pll.park;
    wn0 = param.grid.emu.omega_grid_nom;
    ts = param.glb_time.ts_afe;
    d.omega_n = wn0;
    d.ts = ts;
    s = tf('s');

    %% Park PLL: SOGI-equivalent quadrature generator and phase loop, at 1 pu and at the sag of the test signal
    d.park.wf = 1/pk.tau;
    d.park.k_sogi = d.park.wf/wn0;
    d.park.Gq = wn0*d.park.wf/(s^2 + d.park.wf*s + wn0^2);         % beta/u
    v_sag = min([param.test.a0, param.test.a_sag, param.test.a_rec, 1]);
    d.park.nominal = park_loop(pk, wn0, 1, s);
    d.park.sag = park_loop(pk, wn0, v_sag, s);

    %% first harmonic tracker: frequency response of alpha and beta (forward Euler model of the class)
    f = c.fht_grid;
    d.fht = fht_response(f.Ad_fht, f.Ld_fht, f.Cfht, f.omega_fht, wn0, ts);
    A0 = [0 1; -f.omega_fht^2 0];                                   % undamped oscillator, exact discretization
    Ad0 = expm(A0*ts);
    Ld0 = acker(Ad0', f.Cfht', exp(ts*[-1 -4]*f.omega_fht))';
    d.fht_undamped = fht_response(Ad0, Ld0, f.Cfht, f.omega_fht, wn0, ts);
    d.fht_undamped.Ad_fht = Ad0;
    d.fht_undamped.Ld_fht = Ld0;

    %% stage B loops
    d.fht_pll = pll_loop(c.pll_p_fht, c.pll_i1_fht, wn0, ts, s);
    d.ddsrf_pll = pll_loop(c.pll_p_ddsrt, c.pll_i1_ddsrt, wn0, ts, s);
    d.ddsrf_pll.tau_lpf = c.tau_ddsrf;
    wf = 1/c.tau_ddsrf;                                         % decoupling network in the loop
    d.ddsrf_pll.Deff = wf*(s^3 + 2*wf*s^2 + 4*wn0^2*s + 4*wn0^2*wf)/ ...
        (s^4 + 4*wf*s^3 + 4*(wf^2 + wn0^2)*s^2 + 8*wn0^2*wf*s + 4*wn0^2*wf^2);
    Lnet = wn0*(c.pll_p_ddsrt*s + c.pll_i1_ddsrt)/s^2*d.ddsrf_pll.Deff;
    d.ddsrf_pll.T_net = minreal(feedback(Lnet, 1));
    [~, d.ddsrf_pll.pm_net, ~, d.ddsrf_pll.wc_net] = margin(Lnet);
    [~, d.fht_pll.pm, ~, d.fht_pll.wc] = margin(wn0*(c.pll_p_fht*s + c.pll_i1_fht)/s^2);

    %% double-integrator observer (theta -> omega_hat): x+ = Adso x + Ldso (theta - Cso x), omega_hat = x(2)
    Ao = c.di_obs.Adso - c.di_obs.Ldso*c.di_obs.Cso;
    d.obs.poles_d = eig(Ao);
    d.obs.poles_c = log(d.obs.poles_d)/ts;
    d.obs.delay_ms = sum(-1./real(d.obs.poles_c))*1e3;

    %% summary
    fprintf('PLL design: omega_n = %.1f rad/s, ts = %.0f us\n', wn0, ts*1e6);
    fprintf('  Park PLL: QSG = SOGI with k = %.2f (tau = %.2f ms), kp = %.2f, ki = %.1f\n', ...
        d.park.k_sogi, pk.tau*1e3, pk.kp, pk.ki);
    park_line('1 pu', d.park.nominal);
    park_line(sprintf('%.2g pu', v_sag), d.park.sag);
    fprintf('  FHT (delta %.2f, forward Euler) at %.0f Hz: alpha %.4f / %+.2f deg, beta %.4f / %+.2f deg (ideal -90), positive sequence %.4f / %+.2f deg, negative %.4f\n', ...
        f.delta_fht, wn0/2/pi, abs(d.fht.alpha), angle(d.fht.alpha)*180/pi, abs(d.fht.beta), angle(d.fht.beta)*180/pi, ...
        abs(d.fht.pos), angle(d.fht.pos)*180/pi, abs(d.fht.neg));
    fprintf('      FHT-PLL stage A (three FHTs): positive sequence %.4f / %+.2f deg, negative %.4f | |alpha|/|u| at DC %.2f, 3rd %.2f, 5th %.2f, 7th %.2f\n', ...
        abs(d.fht.pos3), angle(d.fht.pos3)*180/pi, abs(d.fht.neg3), d.fht.alpha_h(1), d.fht.alpha_h(2), d.fht.alpha_h(3), d.fht.alpha_h(4));
    fprintf('      undamped FHT, exact discretization: positive sequence %.4f / %+.3f deg, negative %.5f (FHT-PLL: %+.3f deg)\n', ...
        abs(d.fht_undamped.pos), angle(d.fht_undamped.pos)*180/pi, abs(d.fht_undamped.neg), angle(d.fht_undamped.pos3)*180/pi);
    fprintf('  FHT-PLL   kp = %.2f, ki = %.0f: wn = %.1f rad/s (%.1f Hz), zeta = %.2f, crossover %.0f rad/s, PM %.0f deg, 100 Hz ripple gain %.3f, discrete poles |z| = %s\n', ...
        c.pll_p_fht, c.pll_i1_fht, d.fht_pll.wn, d.fht_pll.wn/2/pi, d.fht_pll.zeta, d.fht_pll.wc, d.fht_pll.pm, ...
        d.fht_pll.ripple_gain, mat2str(abs(d.fht_pll.poles_d)', 4));
    fprintf('  DDSRF-PLL kp = %.2f, ki = %.0f: wn = %.1f rad/s (%.1f Hz), zeta = %.2f without the network; with the decoupling network (LPF tau %.2f ms) in the loop: crossover %.0f rad/s, PM %.0f deg\n', ...
        c.pll_p_ddsrt, c.pll_i1_ddsrt, d.ddsrf_pll.wn, d.ddsrf_pll.wn/2/pi, d.ddsrf_pll.zeta, c.tau_ddsrf*1e3, ...
        d.ddsrf_pll.wc_net, d.ddsrf_pll.pm_net);
    fprintf('  omega observer poles %s rad/s, delay %.1f ms\n', mat2str(real(d.obs.poles_c)', 4), d.obs.delay_ms);

    %% figures
    if isfield(param.sim, 'design_figures') && param.sim.design_figures
        w = logspace(0, 4, 400);
        figure('Name', 'PLL phase loops');
        if use_options
            bode(d.park.nominal.T, d.park.sag.T, d.fht_pll.T, d.ddsrf_pll.T_net, w, options);
        else
            bode(d.park.nominal.T, d.park.sag.T, d.fht_pll.T, d.ddsrf_pll.T_net, w);
        end
        grid on
        legend('Park PLL, 1 pu', sprintf('Park PLL, %.2g pu', v_sag), 'FHT-PLL', 'DDSRF-PLL', 'Location', 'southwest');
        title('Closed phase loops theta/theta_g (averaged models, trackers not included)');

        figure('Name', 'PLL phase step');
        step(0.1*d.park.nominal.T, 0.1*d.park.sag.T, 0.1*d.fht_pll.T, 0.1*d.ddsrf_pll.T_net, 0.3);
        grid on
        legend('Park PLL, 1 pu', sprintf('Park PLL, %.2g pu', v_sag), 'FHT-PLL', 'DDSRF-PLL', 'Location', 'southeast');
        title('Phase after a 0.1 rad jump of the input phase (averaged models)');
        ylabel('theta [rad]');

        figure('Name', 'First harmonic tracker');
        if use_options
            bode(d.fht.sys, logspace(1, log10(pi/ts), 400), options);
        else
            bode(d.fht.sys, logspace(1, log10(pi/ts), 400));
        end
        grid on
        title(sprintf('FHT: alpha (out 1) and beta (out 2) over u (delta %.2f, poles [-1 -4] omega_n)', f.delta_fht));
    end
end

function L = park_loop(pk, wn0, V, s)
%PARK_LOOP  Averaged phase loop of the Park PLL at input amplitude V [pu].
    wf = 1/pk.tau;
    L.V = V;
    L.K = wn0*pk.kp*V;
    L.Kpd = V*(s + wf)/(2*s + wf);                              % eta over the phase error, averaged
    L.L = minreal(wn0*(pk.kp*s + pk.ki)/s^2*L.Kpd);             % PI on eta, then theta = omega_n/s omega_pu
    [~, L.pm, ~, L.wc] = margin(L.L);
    L.T = minreal(feedback(L.L, 1));
    L.poles_c = pole(L.T);
    p = L.poles_c(abs(imag(L.poles_c)) > 0);
    if isempty(p)
        L.wn = sqrt(prod(abs(L.poles_c(1:min(2, end)))));
        L.zeta = 1;
    else
        L.wn = abs(p(1));
        L.zeta = -real(p(1))/abs(p(1));
    end
    L.err_per_Hz = 2*pi/L.K*180/pi;                            % [deg] phase error per Hz of frequency step (ki = 0)
    L.ripple_gain = abs(freqresp(L.T, 2*wn0));
end

function park_line(label, L)
    fprintf('      at %s: crossover %.0f rad/s, PM %.0f deg, closed loop poles %s rad/s (wn %.0f rad/s, zeta %.2f), phase error %.2f deg per Hz of frequency step\n', ...
        label, L.wc, L.pm, mat2str(L.poles_c.', 4), L.wn, L.zeta, L.err_per_Hz);
end

function r = fht_response(Ad, Ld, C, w, wn0, ts)
%FHT_RESPONSE  Outputs alpha = x1 and beta = -x2/w of the FHT over the input, at omega_n and at the harmonics.
    Acl = Ad - Ld*C;
    r.sys = ss(Acl, Ld, [C; 0 -1/w], [0; 0], ts);
    r.poles_c = log(eig(Acl))/ts;
    H = squeeze(freqresp(r.sys, wn0));
    r.alpha = H(1);
    r.beta = H(2);
    r.pos = (r.alpha + 1j*r.beta)/2;                            % u = cos: alpha + j beta = pos e^jwt + conj(neg) e^-jwt
    r.neg = (r.alpha - 1j*r.beta)/2;
    % FHT-PLL: alpha, beta through two more FHTs (beta = +x2/w there), u_sr = -1/2 (u - j v)
    hx1 = H(1);
    hx2 = -H(2);                                                % x2/w over the input
    a = r.alpha;
    b = r.beta;
    usr_a = -(hx1*a + hx2*b)/2;
    usr_b = -(-hx2*a + hx1*b)/2;
    r.pos3 = -(usr_a + 1j*usr_b)/2;                             % the gain -1 of stage B takes back the sign
    r.neg3 = (usr_a - 1j*usr_b)/2;
    h = [1e-3 3 5 7];                                           % DC (1e-3 omega_n) and odd harmonics
    Hh = freqresp(r.sys, h*wn0);
    r.alpha_h = abs(squeeze(Hh(1, 1, :))).';
    r.beta_h = abs(squeeze(Hh(2, 1, :))).';
end

function L = pll_loop(kp, ki, wn0, ts, s)
%PLL_LOOP  Continuous and discrete data of the stage B phase loop with gains kp [pu/pu] and ki [1/s].
    L.kp = kp;
    L.ki = ki;
    L.wn = sqrt(wn0*ki);
    L.zeta = wn0*kp/(2*L.wn);
    L.T = wn0*(kp*s + ki)/(s^2 + wn0*kp*s + wn0*ki);            % theta/theta_g
    L.poles_c = roots([1 wn0*kp wn0*ki]);
    L.ripple_gain = abs(freqresp(L.T, 2*wn0));                  % |T(j 2 omega_n)| [rad/pu]
    % discrete loop as in pll_stageB: i_k = i_k-1 + ki ts e_k, theta_k+1 = theta_k + omega_n ts (kp e_k + i_k),
    % e_k = theta_g,k - theta_k (stage A reads the phase of the previous step)
    z = tf('z', ts);
    C = kp + ki*ts*z/(z - 1);                                   % e -> omega_pu
    P = wn0*ts/(z - 1);                                         % omega_pu -> theta
    L.T_d = feedback(C*P, 1);
    L.poles_d = pole(L.T_d);
end
