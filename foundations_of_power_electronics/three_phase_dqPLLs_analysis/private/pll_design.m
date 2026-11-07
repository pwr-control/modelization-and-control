function d = pll_design(param, options)
%PLL_DESIGN  Design check of the PLL loops of frt_dqpll and of the grid stage: numbers in d, figures on request.
%
%   d = pll_design(param)             numbers only (and figures when param.sim.design_figures is true)
%   d = pll_design(param, options)    options: bodeoptions of the init for the Bode plots, or [] (Octave)
%
%   Phase loop of the three Simulink PLLs (dq_pll, DDSRF-PLL, FHT-PLL, blocks pll_stageB). The eta component of
%   the positive sequence, divided by the voltage magnitude (Vgrid_unorm: e = sin(theta_g - theta) in pu, 1/|u|
%   saturated at 50), goes to a PI in omega pu: kp = pll_p, ki = pll_i1 [1/s], forward Euler integrator (bounded
%   at +-2 pu in dq_pll, free in the other two); theta_k = theta_k-1 + omega_n ts omega_pu_k, the output theta is
%   one step late (Unit Delay2) and the observer di_obs estimates omega from it. Linearized continuous loop:
%       theta' = omega_n (kp e + ki int e),  e = theta_g - theta
%       T(s) = omega_n (kp s + ki) / (s^2 + omega_n kp s + omega_n ki)
%       wn = sqrt(omega_n ki),  zeta = omega_n kp / (2 wn)
%   d.normal, d.frt: the loop with pll_p and with pll_p_frt (the dq_pll switches to it while frt_active is 1, ki
%   unchanged), each with wn, zeta, the continuous and discrete closed-loop poles (ts, one step delay) and the gain
%   of T at 2 omega_n: the 100 Hz ripple that an asymmetric fault puts on e (negative over positive sequence, un/up)
%   reaches the phase with this gain [rad/pu]. d.frt_alt: the fault gain with ki scaled by (pll_p_frt/pll_p)^2,
%   which keeps zeta. d.obs: discrete and continuous poles of the double-integrator observer and its delay (sum of
%   the time constants). d.fht: poles of the first harmonic tracker designed on the forward Euler model
%   (ctrl_afe_setup) and the poles the same gains give with the Taylor-2 discretization of the C code
%   (first_harmonic_tracker.c). d.lcl: resonance of the LCL filter seen from the grid with the converter side
%   open: leakage of the transformer (configurable_trafo, pu r1+r3 and l1+l3 on us2^2/Sr) plus the 1 uH typed in
%   the Inductor blocks in series with CFu and RCFu (LFu is open-ended and carries no current).
%   Figures: Bode of T for the three gain sets with the 100 Hz mark, phase response to a 0.1 rad jump, and the
%   grid-to-capacitor voltage transfer of the LCL stage.

    if nargin < 2
        options = [];
    end
    use_options = ~isempty(options) && ~isstruct(options);     % bodeoptions object of MATLAB (Octave: none)
    c = param.afe.ctrl;
    wn0 = param.grid.emu.omega_grid_nom;
    ts = param.glb_time.ts_afe;
    d.omega_n = wn0;
    d.ts = ts;

    %% phase loops
    d.normal  = pll_loop(c.pll_p, c.pll_i1, wn0, ts);
    d.frt     = pll_loop(c.pll_p_frt, c.pll_i1, wn0, ts);
    d.frt_alt = pll_loop(c.pll_p_frt, c.pll_i1*(c.pll_p_frt/c.pll_p)^2, wn0, ts);

    %% double-integrator observer (theta -> omega_hat): x+ = Adso x + Ldso (theta - Cso x), omega_hat = x(2)
    Ao = c.di_obs.Adso - c.di_obs.Ldso*c.di_obs.Cso;
    d.obs.poles_d = eig(Ao);
    d.obs.poles_c = log(d.obs.poles_d)/ts;
    d.obs.delay_ms = sum(-1./real(d.obs.poles_c))*1e3;

    %% first harmonic tracker: gains designed on the forward Euler model, run with the Taylor-2 model of the C code
    f = c.fht_grid;
    Ad_fe = eye(2) + f.Afht*ts;
    Ad_t2 = eye(2) + f.Afht*ts + f.Afht^2*ts^2/2;
    d.fht.poles_c_design = [-1 -4]*f.omega_fht;
    d.fht.poles_c_fe = log(eig(Ad_fe - f.Ld_fht*f.Cfht))/ts;
    d.fht.poles_c_t2 = log(eig(Ad_t2 - f.Ld_fht*f.Cfht))/ts;

    %% LCL filter from the grid, converter side open
    pu = param.grid.emu.trafo.pu;
    Zb = param.grid.us2^2/param.grid.grid_nominal_power;
    d.lcl.L_series = (pu.l1 + pu.l3)*Zb/wn0 + 1e-6;         % [H] transformer leakage + 1 uH of the Inductor blocks
    d.lcl.R_series = (pu.r1 + pu.r3)*Zb;                     % [Ohm] transformer copper
    d.lcl.C = param.afe.hw.CFu;
    d.lcl.R_c = param.afe.hw.RCFu;
    d.lcl.f_res = 1/(2*pi*sqrt(d.lcl.L_series*d.lcl.C));
    d.lcl.zeta = (d.lcl.R_series + d.lcl.R_c)/2*sqrt(d.lcl.C/d.lcl.L_series);
    s = tf('s');
    d.lcl.H = (1 + s*d.lcl.R_c*d.lcl.C)/(1 + s*(d.lcl.R_series + d.lcl.R_c)*d.lcl.C + s^2*d.lcl.L_series*d.lcl.C);

    %% summary
    fprintf('PLL design: omega_n = %.1f rad/s, ts = %.0f us\n', wn0, ts*1e6);
    fprintf('  normal gains kp = %.2f, ki = %.0f: wn = %.1f rad/s (%.1f Hz), zeta = %.2f, 100 Hz ripple gain %.3f rad/pu, discrete poles |z| = %s\n', ...
        c.pll_p, c.pll_i1, d.normal.wn, d.normal.wn/2/pi, d.normal.zeta, d.normal.ripple_gain, mat2str(abs(d.normal.poles_d)', 4));
    fprintf('  fault gains  kp = %.2f, ki = %.0f: wn = %.1f rad/s (%.1f Hz), zeta = %.2f, 100 Hz ripple gain %.3f rad/pu\n', ...
        c.pll_p_frt, c.pll_i1, d.frt.wn, d.frt.wn/2/pi, d.frt.zeta, d.frt.ripple_gain);
    fprintf('  alternative  kp = %.2f, ki = %.1f: wn = %.1f rad/s (%.1f Hz), zeta = %.2f, 100 Hz ripple gain %.3f rad/pu\n', ...
        c.pll_p_frt, d.frt_alt.ki, d.frt_alt.wn, d.frt_alt.wn/2/pi, d.frt_alt.zeta, d.frt_alt.ripple_gain);
    fprintf('  omega observer poles %s rad/s, delay %.1f ms | FHT poles designed %s, with Taylor-2 %s rad/s\n', ...
        mat2str(real(d.obs.poles_c)', 4), d.obs.delay_ms, mat2str(d.fht.poles_c_design, 4), mat2str(real(d.fht.poles_c_t2)', 4));
    fprintf('  LCL from the grid (converter open): L = %.1f uH, R = %.1f mOhm, C = %.0f uF with %.0f mOhm: resonance %.0f Hz, zeta = %.3f\n', ...
        d.lcl.L_series*1e6, d.lcl.R_series*1e3, d.lcl.C*1e6, d.lcl.R_c*1e3, d.lcl.f_res, d.lcl.zeta);

    %% figures
    if isfield(param.sim, 'design_figures') && param.sim.design_figures
        w = logspace(0, 4, 400);
        figure('Name', 'PLL phase loop');
        if use_options
            bode(d.normal.T, d.frt.T, d.frt_alt.T, w, options);
        else
            bode(d.normal.T, d.frt.T, d.frt_alt.T, w);
        end
        grid on
        legend(sprintf('kp %.2f, ki %.0f', c.pll_p, c.pll_i1), sprintf('fault kp %.2f, ki %.0f', c.pll_p_frt, c.pll_i1), ...
            sprintf('fault kp %.2f, ki %.1f', c.pll_p_frt, d.frt_alt.ki), 'Location', 'southwest');
        title('PLL: closed loop theta/theta_g (100 Hz ripple of e at 2 omega_n)');

        figure('Name', 'PLL phase step');
        step(0.1*d.normal.T, 0.1*d.frt.T, 0.1*d.frt_alt.T, 0.3);
        grid on
        legend('normal', 'fault gains', 'fault, ki scaled', 'Location', 'southeast');
        title('PLL phase after a 0.1 rad jump of the grid phase');
        ylabel('theta [rad]');

        figure('Name', 'LCL from the grid');
        if use_options
            bode(d.lcl.H, logspace(1, 5, 400), options);
        else
            bode(d.lcl.H, logspace(1, 5, 400));
        end
        grid on
        title(sprintf('LCL stage, converter side open: capacitor over grid voltage (f_{res} = %.0f Hz, zeta = %.3f)', ...
            d.lcl.f_res, d.lcl.zeta));
    end
end

function L = pll_loop(kp, ki, wn0, ts)
%PLL_LOOP  Continuous and discrete data of the phase loop with gains kp [pu/pu] and ki [1/s].
    s = tf('s');
    L.kp = kp;
    L.ki = ki;
    L.wn = sqrt(wn0*ki);
    L.zeta = wn0*kp/(2*L.wn);
    L.T = wn0*(kp*s + ki)/(s^2 + wn0*kp*s + wn0*ki);     % theta/theta_g
    L.poles_c = roots([1 wn0*kp wn0*ki]);
    L.ripple_gain = abs(freqresp(L.T, 2*wn0));            % |T(j 2 omega_n)| [rad/pu]
    % discrete loop as in pll_stageB: i_k = i_k-1 + ki ts e_k, theta_k = theta_k-1 + omega_n ts (kp e_k + i_k),
    % e_k = theta_g,k - theta_k-1 (the stage A reads the delayed phase)
    z = tf('z', ts);
    C = kp + ki*ts*z/(z - 1);                             % e -> omega_pu
    P = wn0*ts/(z - 1);                                   % omega_pu -> theta
    Lo = C*P/z;                                           % loop with the one step delay of the phase output
    L.T_d = feedback(Lo, 1);
    L.poles_d = pole(L.T_d);
end
