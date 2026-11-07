function out = plotting_results_plls(varargin)
%PLOTTING_RESULTS_PLLS  Compare the three grid PLLs of frt_dqpll offline on the logged grid voltage.
%
%   plotting_results_plls                        after a run: reads time_ts_afe_sim, ugd_abc_grid_sim,
%                                                phase_grid_fhtsr_dqpll_sim, omega_grid_fhtsr_dqpll_sim and
%                                                param from the base workspace
%   plotting_results_plls('sim_results_01.mat')  reads them from a .mat file saved after a run
%   out = plotting_results_plls(...)             returns the runs, the reference angle and the metrics
%
%   The function runs the PLL algorithms of the model on the logged capacitor voltage ugd_abc_grid_sim,
%   step by step as the Simulink blocks and the library C code do it (control period ts, one-step delays,
%   clamps): the dq-PLL with constant gains and with the gain scheduled in the fault (frt_active is not
%   logged: the fault window of param.frt.data is used), the FHT-PLL, the DDSRF-PLL as built (stage B fed
%   with the negative-sequence outputs through the gain -1), the DDSRF-PLL with the positive-sequence feed
%   (zero start, as the model starts) and an improved DDSRF-PLL (positive feed, frequency integrator started
%   at 1 pu and clamped to [0.5 1.5] pu, kp 0.32, ki 8). The FHT-PLL branch is checked against the logged
%   phase_grid_fhtsr_dqpll_sim. The phase errors are measured against the angle of the voltage itself: a
%   linear fit of the alpha-beta angle before the fault (the emulator runs at a constant frequency).
%
%   Figures: (1) voltage and phase errors with zooms at the fault start and end, (2) estimated frequency,
%   (3) start-up and the 100 Hz ripple inside the fault, (4) the DDSRF-PLL variants with their sequence outputs.
%   The metrics are printed as a table.
%
%   Design note of these PLLs: repos_doc/three_phase_plls_comparison of pwr-control/repositories-documentation
%   (its script py/make_figures.py does the same on the .mat file, plus the analytic figures of the note).

% ------------------------------------------------------------------ data
if nargin >= 1 && ischar(varargin{1})
    S = load(varargin{1}, 'time_ts_afe_sim', 'ugd_abc_grid_sim', 'phase_grid_fhtsr_dqpll_sim', 'omega_grid_fhtsr_dqpll_sim', 'param');
    t = S.time_ts_afe_sim(:)'; u = S.ugd_abc_grid_sim; ph_log = S.phase_grid_fhtsr_dqpll_sim(:)'; om_log = S.omega_grid_fhtsr_dqpll_sim(:)';
    param = S.param;
else
    t = evalin('base', 'time_ts_afe_sim'); t = t(:)'; u = evalin('base', 'ugd_abc_grid_sim');
    ph_log = evalin('base', 'phase_grid_fhtsr_dqpll_sim'); ph_log = ph_log(:)'; om_log = evalin('base', 'omega_grid_fhtsr_dqpll_sim'); om_log = om_log(:)';
    param = evalin('base', 'param');
end
if size(u, 1) ~= 3, u = u'; end
N = numel(t);
P = pll_param(param);
t_f0 = param.frt.data.start_time_LVRT; t_f1 = param.frt.data.end_time_LVRT;

% ------------------------------------------------------------------ reference angle
[al, be] = clarke(u(1, :), u(2, :), u(3, :));
ang = unwrap(atan2(be, al));
m = t > 0.30 & t < min(0.74, t_f0 - 0.01);
pf = polyfit(t(m), ang(m), 1);
th_ref = wrap(pf(1) * t + pf(2));
fprintf('reference angle: %.4f Hz (%.5f pu of omega_n), theta0 = %.4f rad (linear fit of the alpha-beta angle on %.2f-%.2f s)\n', pf(1)/2/pi, pf(1)/P.wn, pf(2), 0.30, min(0.74, t_f0 - 0.01));

% ------------------------------------------------------------------ runs
frt = double(t >= t_f0 & t < t_f1);
kp_sched = P.pll_p * ones(1, N); kp_sched(frt > 0.5) = P.pll_p_frt;
runs = struct();
runs.dq        = run_dq_pll(u, P, P.pll_p * ones(1, N));
runs.dq_sched  = run_dq_pll(u, P, kp_sched);
runs.fht       = run_fht_pll(u, t, P);
o = ddsrf_opts(); o.feed = 'neg'; o.sign = -1;                       runs.ddsrf_model = run_ddsrf_pll(u, t, P, o);
o = ddsrf_opts(); o.feed = 'pos'; o.sign = +1;                       runs.ddsrf_pos   = run_ddsrf_pll(u, t, P, o);
o = ddsrf_opts(); o.feed = 'pos'; o.sign = +1; o.i1_init = 1; o.clamp = [0.5 1.5]; o.kp = 0.32; o.ki = 8;
                                                                     runs.ddsrf_imp   = run_ddsrf_pll(u, t, P, o);
labels = struct('dq', 'dq-PLL (kp 1, ki 80)', 'dq_sched', 'dq-PLL as in the model (kp 1 -> 0.2 in the fault)', 'fht', 'FHT-PLL', ...
    'ddsrf_model', 'DDSRF-PLL as built (negative feed)', 'ddsrf_pos', 'DDSRF-PLL, positive feed (kp 1, ki 80, zero start)', ...
    'ddsrf_imp', 'DDSRF-PLL improved (kp 0.32, ki 8, omega init 1 pu)');
names = fieldnames(runs);

% check of the FHT-PLL branch against the log
e_chk = abs(wrap(ph_log - runs.fht.phase));
fprintf('check: offline FHT-PLL vs logged phase, max |diff| %.2e rad; omega max |diff| %.2e pu\n', max(e_chk), max(abs(om_log - runs.fht.omega)));

% ------------------------------------------------------------------ metrics
pre = t > 0.5 & t < t_f0 - 0.01; mid = t > t_f0 + 0.45 & t < t_f1 - 0.015;
win_in = t >= t_f0 & t < t_f0 + 0.3; win_out = t >= t_f1 & t < t_f1 + 0.3;
fprintf('\n%-50s %8s %7s %9s %9s %10s %10s %9s %9s\n', 'variant', 'offset', 'lock', 'in peak', 'in settle', 'ripple pp', 'w rip pp', 'out peak', 'out settle');
fprintf('%-50s %8s %7s %9s %9s %10s %10s %9s %9s\n', '', '[deg]', '[ms]', '[deg]', '[ms]', '[deg]', '[pu]', '[deg]', '[ms]');
for i = 1:numel(names)
    r = runs.(names{i}); e = wrap(r.phase - th_ref); w = r.omega;
    off = mean(e(pre)); ee = e - off;
    k = find(t < t_f0 - 0.01 & abs(ee) > pi/180, 1, 'last'); lock = 0; if ~isempty(k), lock = t(k); end
    k = find(t >= t_f0 & t < t_f1 & abs(ee) > pi/180, 1, 'last'); s_in = 0; if ~isempty(k), s_in = t(k) - t_f0; end
    k = find(t >= t_f1 & abs(ee) > pi/180, 1, 'last'); s_out = 0; if ~isempty(k), s_out = t(k) - t_f1; end
    mt = struct('offset_deg', off*180/pi, 'lock_s', lock, 'in_peak_deg', max(abs(ee(win_in)))*180/pi, 'in_settle_s', s_in, ...
        'ripple_pp_deg', (max(ee(mid)) - min(ee(mid)))*180/pi, 'omega_ripple_pp', max(w(mid)) - min(w(mid)), ...
        'out_peak_deg', max(abs(ee(win_out)))*180/pi, 'out_settle_s', s_out);
    runs.(names{i}).metrics = mt;
    fprintf('%-50s %8.2f %7.0f %9.2f %9.0f %10.3f %10.4f %9.2f %9.0f\n', labels.(names{i}), mt.offset_deg, 1e3*mt.lock_s, mt.in_peak_deg, ...
        1e3*mt.in_settle_s, mt.ripple_pp_deg, mt.omega_ripple_pp, mt.out_peak_deg, 1e3*mt.out_settle_s);
end
fprintf('(offset: mean phase error before the fault; lock: last time |e - offset| > 1 deg before the fault; settle: last time > 1 deg after the fault start / end)\n');

% ------------------------------------------------------------------ figures
col = struct('dq', [0 0.447 0.741], 'dq_sched', [0.301 0.745 0.933], 'fht', [0.85 0.325 0.098], 'ddsrf_model', [0.45 0.45 0.45], ...
    'ddsrf_pos', [0.466 0.674 0.188], 'ddsrf_imp', [0.635 0.078 0.184]);
zooms = {[0 t(end)], [t_f0 - 0.01, t_f0 + 0.2], [t_f1 - 0.015, t_f1 + 0.2]};
ttl = {'whole record', 'fault start', 'fault end'};
% 1: voltage and phase errors
figure('Name', 'PLLs: phase error');
for j = 1:3
    subplot(2, 3, j); hold on; grid on;
    plot(t, u(1, :), t, u(2, :), t, u(3, :)); xlim(zooms{j}); title(['grid voltage, ' ttl{j}]); ylabel('u_{abc} [pu]');
    subplot(2, 3, 3 + j); hold on; grid on;
    for i = 1:numel(names)
        r = runs.(names{i}); plot(t, (wrap(r.phase - th_ref) - r.metrics.offset_deg*pi/180)*180/pi, 'Color', col.(names{i}));
    end
    xlim(zooms{j}); if j > 1, ylim([-30 30]); end; title(['phase error, ' ttl{j} ' (offset removed)']); ylabel('[deg]'); xlabel('time [s]');
end
subplot(2, 3, 4); legend(cellfun(@(n) labels.(n), names, 'UniformOutput', false), 'Location', 'southoutside', 'FontSize', 7);
% 2: frequency
figure('Name', 'PLLs: frequency');
for j = 1:3
    subplot(1, 3, j); hold on; grid on;
    for i = 1:numel(names), r = runs.(names{i}); plot(t, r.omega, 'Color', col.(names{i})); end
    xlim(zooms{j}); if j > 1, ylim([0.85 1.15]); else, ylim([-1.2 1.4]); end
    title(['estimated frequency, ' ttl{j}]); ylabel('\omega [pu]'); xlabel('time [s]');
end
legend(cellfun(@(n) labels.(n), names, 'UniformOutput', false), 'Location', 'southoutside', 'FontSize', 7);
% 3: start-up and ripple
figure('Name', 'PLLs: start-up and 100 Hz ripple');
subplot(2, 2, 1); hold on; grid on;
for i = 1:numel(names), r = runs.(names{i}); plot(t, wrap(r.phase - th_ref)*180/pi, 'Color', col.(names{i})); end
xlim([0 0.35]); title('start (phase error)'); ylabel('[deg]');
subplot(2, 2, 3); hold on; grid on;
for i = 1:numel(names), r = runs.(names{i}); plot(t, r.omega, 'Color', col.(names{i})); end
xlim([0 0.35]); title('start (frequency)'); ylabel('\omega [pu]'); xlabel('time [s]');
tz = [t_f0 + 0.65, t_f0 + 0.69];
subplot(2, 2, 2); hold on; grid on;
for i = 1:numel(names), r = runs.(names{i}); plot(t, (wrap(r.phase - th_ref) - r.metrics.offset_deg*pi/180)*180/pi, 'Color', col.(names{i})); end
xlim(tz); ylim([-20 20]); title('inside the fault: 100 Hz ripple (offset removed)'); ylabel('[deg]');
subplot(2, 2, 4); hold on; grid on;
for i = 1:numel(names), r = runs.(names{i}); plot(t, r.omega, 'Color', col.(names{i})); end
xlim(tz); ylim([0.7 1.3]); title('inside the fault: frequency'); ylabel('\omega [pu]'); xlabel('time [s]');
legend(cellfun(@(n) labels.(n), names, 'UniformOutput', false), 'Location', 'southoutside', 'FontSize', 7);
% 4: DDSRF variants with their sequence outputs
figure('Name', 'DDSRF-PLL variants');
dd = {'ddsrf_model', 'ddsrf_pos', 'ddsrf_imp'};
subplot(3, 1, 1); hold on; grid on;
for i = 1:3, r = runs.(dd{i}); plot(t, wrap(r.phase - th_ref)*180/pi, 'Color', col.(dd{i})); end
title('DDSRF-PLL: phase error (as built, positive feed with zero start, improved)'); ylabel('[deg]'); legend(cellfun(@(n) labels.(n), dd, 'UniformOutput', false), 'FontSize', 7);
subplot(3, 1, 2); hold on; grid on;
for i = 1:3, r = runs.(dd{i}); plot(t, r.omega, 'Color', col.(dd{i})); end
ylabel('\omega [pu]'); title('estimated frequency');
subplot(3, 1, 3); hold on; grid on;
for i = 1:3, r = runs.(dd{i}); plot(t, hypot(r.pos_xi, r.pos_eta), 'Color', col.(dd{i})); plot(t, hypot(r.neg_xi, r.neg_eta), '--', 'Color', col.(dd{i})); end
ylabel('[pu]'); xlabel('time [s]'); title('sequence outputs of the DDSRF block: |u_{pos}| (solid), |u_{neg}| (dashed)');

out = struct('t', t, 'th_ref', th_ref, 'runs', runs, 'labels', labels, 'P', P);
end

% ====================================================================== parameters of the blocks
function P = pll_param(param)
c = param.afe.ctrl;
P.ts = param.glb_time.ts_afe;
P.wn = param.grid.emu.omega_grid_nom;
P.pll_p = c.pll_p; P.pll_i1 = c.pll_i1; P.pll_p_frt = c.pll_p_frt;
P.pll_p_ddsrt = c.pll_p_ddsrt; P.pll_i1_ddsrt = c.pll_i1_ddsrt;
P.pll_p_fht = c.pll_p_fht; P.pll_i1_fht = c.pll_i1_fht;
P.tau_ddsrf = c.tau_ddsrf;
P.Adso = c.di_obs.Adso; P.Ldso = c.di_obs.Ldso(:); P.Cso = c.di_obs.Cso;
P.omega_fht = c.fht_grid.omega_fht; P.delta_fht = c.fht_grid.delta_fht; P.Ld_fht = c.fht_grid.Ld_fht(:);
P.t_reset = 0.0125;          % Step block that holds the first harmonic trackers in reset at the start
end

% ====================================================================== elementary blocks (Stateflow charts)
function [al, be] = clarke(a, b, c)
al = 2/3 * (a - 0.5*b - 0.5*c); be = (b - c) / sqrt(3);
end
function [xi, eta] = rot_pos(al, be, g)      % alphabeta2xieta_pos
xi = al.*cos(g) + be.*sin(g); eta = -al.*sin(g) + be.*cos(g);
end
function [xi, eta] = rot_neg(al, be, g)      % alphabeta2xieta_neg
xi = al.*cos(g) - be.*sin(g); eta = al.*sin(g) + be.*cos(g);
end
function y = wrap(x)                          % bounder: -pi + mod(u + 3 pi, 2 pi)
y = -pi + mod(x + 3*pi, 2*pi);
end
function vn = vnorm_of(al, be)                % 1/sqrt(max(alpha^2 + beta^2, 0.02))
vn = 1 ./ sqrt(max(al.^2 + be.^2, 0.02));
end

% ---------------------------------------------------------------------- first harmonic tracker (library C code)
function f = fht_init(P)
a = -P.omega_fht^2; b = -P.delta_fht*P.omega_fht; ts = P.ts;
f.a11 = 1 + 0.5*a*ts*ts;           f.a12 = ts + 0.5*b*ts*ts;
f.a21 = a*ts + 0.5*a*b*ts*ts;      f.a22 = 1 + b*ts + 0.5*(a + b*b)*ts*ts;
f.l1 = P.Ld_fht(1); f.l2 = P.Ld_fht(2);
f.s1 = 0; f.s2 = 0; f.out = 0;
end
function [f, u_hat, u_hat_dot] = fht_step(f, u, reset)
% first_harmonic_tracker_process: the innovation uses the output stored at the previous call; the output is the
% state 1 before the update, u_hat_dot the state 2 after the update
if reset, f.s1 = 0; f.s2 = 0; f.out = 0; end
err = u - f.out;
s1 = f.l1*err + f.a11*f.s1 + f.a12*f.s2;
s2 = f.l2*err + f.a21*f.s1 + f.a22*f.s2;
f.out = f.s1; f.s1 = s1; f.s2 = s2;
u_hat = f.out; u_hat_dot = f.s2;
end

% ---------------------------------------------------------------------- discrete low pass of the library (no direct feedthrough)
function f = lpf_init(ts, tau)
f.g = ts/tau; f.x1 = 0; f.y = 0;
end
function [f, y] = lpf_output(f)               % y[k] = ts/tau x[k-1] + (1 - ts/tau) y[k-1]
f.y = f.g*f.x1 + (1 - f.g)*f.y; y = f.y;
end
function f = lpf_push(f, x)
f.x1 = x;
end

% ---------------------------------------------------------------------- stage B: PI loop, phase integrator, double integrator observer
function sb = stageB_init(P, kp, ki, sgn, clamp, i1_init)
sb.kp = kp; sb.ki = ki; sb.sign = sgn; sb.clamp = clamp; sb.i1 = i1_init; sb.theta = 0;
sb.x = [0; i1_init*P.wn];                      % observer state [theta_hat; omega_hat]
end
function [sb, phase_out, omega_out, e] = stageB_step(sb, P, u_eta, vnorm, kp)
e = sb.sign * u_eta * vnorm;
i1 = sb.i1 + sb.ki*P.ts*e;
if ~isempty(sb.clamp), i1 = min(max(i1, sb.clamp(1)), sb.clamp(2)); end
sb.i1 = i1;
w_pu = kp*e + i1;
theta_new = wrap(sb.theta + P.ts*P.wn*w_pu);
phase_out = sb.theta;                          % Unit Delay: the phase computed at the previous step
eo = wrap(phase_out - wrap(P.Cso*sb.x));       % observer: e = wrap(theta - wrap(theta_hat))
omega_out = sb.x(2) / P.wn;                    % omega_hat from the delayed state
sb.x = P.Adso*sb.x + P.Ldso*eo;
sb.theta = theta_new;
end

% ====================================================================== the three PLLs
function r = run_dq_pll(u, P, kp_sched)
N = size(u, 2); r.phase = zeros(1, N); r.omega = zeros(1, N); r.e = zeros(1, N);
sb = stageB_init(P, P.pll_p, P.pll_i1, +1, [-2 2], 0);           % integrator clamped to +-2 as the block
for k = 1:N
    [al, be] = clarke(u(1, k), u(2, k), u(3, k)); vn = vnorm_of(al, be);
    [~, eta] = rot_pos(al, be, sb.theta);
    [sb, r.phase(k), r.omega(k), r.e(k)] = stageB_step(sb, P, eta, vn, kp_sched(k));
end
end

function r = run_fht_pll(u, t, P)
N = size(u, 2); r.phase = zeros(1, N); r.omega = zeros(1, N); r.e = zeros(1, N); r.u_sr_a = zeros(1, N); r.u_sr_b = zeros(1, N);
fa = fht_init(P); fb = fht_init(P);
sb = stageB_init(P, P.pll_p_fht, P.pll_i1_fht, -1, [], 0);       % gain -1 in front of the PI, no clamp
for k = 1:N
    rst = t(k) < P.t_reset;
    [al, be] = clarke(u(1, k), u(2, k), u(3, k));
    [fa, ua, da] = fht_step(fa, al, rst); [fb, ub, db] = fht_step(fb, be, rst);
    va = da / P.omega_fht; vb = db / P.omega_fht;
    usa = -0.5*(ua + vb); usb = -0.5*(ub - va);                   % minus the positive sequence
    vn = vnorm_of(usa, usb);
    [~, eta] = rot_pos(usa, usb, sb.theta);
    [sb, r.phase(k), r.omega(k), r.e(k)] = stageB_step(sb, P, eta, vn, sb.kp);
    r.u_sr_a(k) = usa; r.u_sr_b(k) = usb;
end
end

function o = ddsrf_opts()
o = struct('feed', 'neg', 'sign', -1, 'kp', [], 'ki', [], 'prefilter', true, 'i1_init', 0, 'clamp', [], 'tau', []);
end
function r = run_ddsrf_pll(u, t, P, o)
N = size(u, 2); r.phase = zeros(1, N); r.omega = zeros(1, N); r.e = zeros(1, N);
r.pos_xi = zeros(1, N); r.pos_eta = zeros(1, N); r.neg_xi = zeros(1, N); r.neg_eta = zeros(1, N);
kp = o.kp; if isempty(kp), kp = P.pll_p_ddsrt; end
ki = o.ki; if isempty(ki), ki = P.pll_i1_ddsrt; end
tau = o.tau; if isempty(tau), tau = P.tau_ddsrf; end
f = {fht_init(P), fht_init(P), fht_init(P)};
lp = {lpf_init(P.ts, tau), lpf_init(P.ts, tau), lpf_init(P.ts, tau), lpf_init(P.ts, tau)};
sb = stageB_init(P, kp, ki, o.sign, o.clamp, o.i1_init);
for k = 1:N
    rst = t(k) < P.t_reset;
    if o.prefilter
        [f{1}, a] = fht_step(f{1}, u(1, k), rst); [f{2}, b] = fht_step(f{2}, u(2, k), rst); [f{3}, c] = fht_step(f{3}, u(3, k), rst);
    else
        a = u(1, k); b = u(2, k); c = u(3, k);
    end
    [al, be] = clarke(a, b, c); vn = vnorm_of(al, be);
    th = sb.theta; th2 = wrap(2*th);
    y = zeros(1, 4);
    for i = 1:4, [lp{i}, y(i)] = lpf_output(lp{i}); end           % pos_xi, pos_eta, neg_xi, neg_eta at this step
    [pxi, peta] = rot_pos(al, be, th); [nxi, neta] = rot_neg(al, be, th);
    [dpx, dpe] = rot_pos(y(3), y(4), th2);                         % decoupling: filtered negative, rotated by 2 theta
    [dnx, dne] = rot_neg(y(1), y(2), th2);                         % decoupling: filtered positive, rotated by -2 theta
    lp{1} = lpf_push(lp{1}, pxi - dpx); lp{2} = lpf_push(lp{2}, peta - dpe);
    lp{3} = lpf_push(lp{3}, nxi - dnx); lp{4} = lpf_push(lp{4}, neta - dne);
    if strcmp(o.feed, 'neg'), u_eta = y(4); else, u_eta = y(2); end
    [sb, r.phase(k), r.omega(k), r.e(k)] = stageB_step(sb, P, u_eta, vn, kp);
    r.pos_xi(k) = y(1); r.pos_eta(k) = y(2); r.neg_xi(k) = y(3); r.neg_eta(k) = y(4);
end
end
