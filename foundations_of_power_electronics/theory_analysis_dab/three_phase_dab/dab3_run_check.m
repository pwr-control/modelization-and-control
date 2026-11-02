%[text] # dab3_run\_check: a logged run of three\_phase\_dab.slx against the sizing
% Rev. 01, 2026-10-29. Reads the *_sim variables of the logged window and param (after the run, or
% load sim_results_xx.mat) and prints:
%   - DC powers and efficiency between the two DC links (dab_*_modA: voltage of the DC link capacitor and
%     current of the DC choke where it enters the DC link, positive into the converter);
%   - the operating point measured on the edges of the phase voltages: phase shift, switching current of
%     every leg on the rising and falling edges, transition time 2 Coss V / i_sw against the dead time;
%   - the same point from the harmonic model (./private/dab3_harmonic_op.m) at the measured DC voltages and
%     phase shift;
%   - the loss breakdown. The local solver (Backward Euler, step tc) dissipates L/2 (delta i)^2 per step in
%     every inductance: with the fast currents of the leakage inductances this numerical damping is of the
%     order of the physical losses (it scales with tc). The switch model conducts in the off state (leakage
%     conductance). The ploss outputs of the bridges add Eon and Eoff of the dataset at every edge (hard
%     switching); the electrical circuit has no switching loss;
%   - the DC offset of the magnetizing currents and the DC side (low-frequency oscillation, 6 fs ripple).
% Signals: i1 out of bridge 1 into the transformer, i2 out of the transformer into bridge 2, u1 and u2
% phase-to-star voltages of the bridges; inverter_k_dab_devices_modA: per switch [ploss, v, i, T_JH, T_HA].
% It needs the logging step equal to the solver step (param.time.tc_decimation = 1). Helpers:
% ./private/dab3_edges.m, ./private/dab3_isw.m, called through the handles of dab3_private.m.

prv = dab3_private();                                       % ./private through handles (dab3_private.m)
hw = param.dab3.hw;
dev = param.dab3.mosfet;
fs = param.dab3.fs;
tc = param.glb_time.tc;                                     % [s] solver step
td = param.dab3.pwm.dead_time;                              % [s]
c_leg = 2*dev.Coss;                                         % [F] one leg: two Coss
n12 = hw.n1/hw.n2;
if param.glb_time.decimation_tc ~= 1
    warning('dab3_run_check: logging decimation %d, the numerical damping needs the solver step', ...
        param.glb_time.decimation_tc);
end
t = time_tc_sim(:);
V1 = dab_voltage_input_modA_sim(:, 1);
I1 = dab_current_input_modA_sim(:, 1);
V2 = dab_voltage_output_modA_sim(:, 1);
I2 = dab_current_output_modA_sim(:, 1);
u1 = u1_dab_transformer_modA_sim;
i1 = i1_dab_transformer_modA_sim;
u2 = u2_dab_transformer_modA_sim;
i2 = i2_dab_transformer_modA_sim;
Tw = numel(t)*tc;                                           % [s] logged window
Np = round(1/(fs*tc));                                      % samples per period

%[text] ## DC powers
Pin1 = mean(V1.*I1);                                        % [W] into the converter from side 1
Pin2 = mean(V2.*I2);                                        % [W] into the converter from side 2
% energy stored in the DC link capacitors during the window (the window may end at another point of a
% low-frequency oscillation than where it starts)
dEc = (hw.Cdc_dc1*(mean(V1(end-Np+1:end))^2 - mean(V1(1:Np))^2) + ...
    hw.Cdc_dc2*(mean(V2(end-Np+1:end))^2 - mean(V2(1:Np))^2))/(2*Tw);
if Pin1 >= 0
    eta = -Pin2/Pin1;
else
    eta = -Pin1/Pin2;
end
fprintf('Run %.3f-%.3f s (%d periods): side 1 %.1f V, %.1f A, %.2f kW in; side 2 %.1f V, %.1f A, %.2f kW in\n', ...
    t(1), t(end), round(Tw*fs), mean(V1), mean(I1), Pin1/1e3, mean(V2), mean(I2), Pin2/1e3);
fprintf('  losses between the DC links %.0f W (%.0f W into the DC link capacitors), eta = %.2f %%\n', ...
    Pin1 + Pin2 - dEc, dEc, 100*eta);

%[text] ## Operating point on the edges
% leg k of a bridge switches when phase k jumps by 2V/3 (the other two phases by -V/3)
ed1 = prv.dab3_edges(u1, mean(V1));
ed2 = prv.dab3_edges(u2, mean(V2));
% phase shift: from each edge of bridge 1 to the next edge of bridge 2 of the same leg and direction. The
% edges sit on the grid of tc and 1/fPWM_DAB is not a whole number of steps: the delay of the single edges
% differs by one step from leg to leg, the mean over all the edges is the phase shift.
dn = NaN(size(ed2.n));
for k = 1:numel(ed2.n)
    j = find(ed1.leg == ed2.leg(k) & ed1.dir == ed2.dir(k) & ed1.n <= ed2.n(k), 1, 'last');
    if ~isempty(j)
        dn(k) = ed2.n(k) - ed1.n(j);
    end
end
dn = dn(~isnan(dn));
phk = 2*pi*fs*tc*dn;
phk(phk > pi) = phk(phk > pi) - 2*pi;                       % bridge 2 leading: reverse power
phi = mean(phk);
fprintf('Phase shift %.2f deg (single edges %.2f to %.2f deg, steps of tc = %.2f deg)\n', rad2deg(phi), ...
    rad2deg(min(phk)), rad2deg(max(phk)), 360*fs*tc);
% switching current: current into the leg node at a rising edge, out of it at a falling edge (> 0: ZVS
% possible). Bridge 1: i1 leaves the leg node; bridge 2: i2 enters it. Mean of the samples around the edge.
sw1 = prv.dab3_isw(ed1, -i1);
sw2 = prv.dab3_isw(ed2, i2);
fprintf('Switching currents [A], rising / falling / mean per leg, then all edges (minimum):\n');
for b = 1:2
    if b == 1
        sw = sw1; V = mean(V1);
    else
        sw = sw2; V = mean(V2);
    end
    fprintf('  bridge %d:', b);
    for k = 1:3
        fprintf('  %.1f / %.1f / %.1f', sw.rise(k), sw.fall(k), sw.leg(k));
    end
    fprintf(';  %.1f (min %.1f) -> transition %.0f ns, dead time %.0f ns\n', sw.all, sw.min, ...
        1e9*c_leg*V/sw.all, 1e9*td);
end

%[text] ## Harmonic model at the measured point
p = struct('V1', mean(V1), 'V2', mean(V2), 'n1', hw.n1, 'n2', hw.n2, 'fs', fs, ...
    'L1', hw.Ld1, 'R1', hw.Rs1 + dev.Rds_on, 'L2r', hw.Ld2*n12^2, ...
    'R2r', (hw.Rs2 + dev.Rds_on)*n12^2, 'Lm', hw.Lm, 'Rfe', hw.Rfe);
o = prv.dab3_harmonic_op(p, phi);
I1rms = sqrt(mean(i1.^2));
I2rms = sqrt(mean(i2.^2));
fprintf('                         run          harmonic model\n');
fprintf('  P side 1 / kW      %8.1f       %8.1f\n', Pin1/1e3, o.P1/1e3);
fprintf('  P side 2 / kW      %8.1f       %8.1f\n', -Pin2/1e3, o.P2/1e3);
fprintf('  I1 / A rms         %8.1f       %8.1f   (phases %.1f %.1f %.1f)\n', mean(I1rms), o.I1rms, I1rms);
fprintf('  I2 / A rms         %8.1f       %8.1f\n', mean(I2rms), o.I2rms);
fprintf('  I1 / A peak        %8.1f       %8.1f\n', max(abs(i1(:))), o.I1pk);
fprintf('  i_sw1 / A          %8.1f       %8.1f\n', sw1.all, o.i_sw1);
fprintf('  i_sw2 / A          %8.1f       %8.1f\n', sw2.all, o.i_sw2);

%[text] ## Losses
% transformers, from the equations of the library SSC (saturable_single_phase_transformer) as the solver
% steps them: e = u1 - Rs1 i1 - Ld1 (i1(n) - i1(n-1))/tc on the magnetizing branch, i_fe = e/Rfe
p_cu = 0; p_fe = 0; p_num_ld = 0; p_num_lm = 0;
im_dc = zeros(1, 3); im_pk = zeros(1, 3);
for k = 1:3
    di1 = diff(i1(:, k));
    di2 = diff(i2(:, k));
    e = u1(2:end, k) - hw.Rs1*i1(2:end, k) - hw.Ld1*di1/tc;
    im = i1(2:end, k) - e/hw.Rfe - i2(2:end, k)/n12;        % magnetizing current, primary
    p_cu = p_cu + hw.Rs1*mean(i1(:, k).^2) + hw.Rs2*mean(i2(:, k).^2);
    p_fe = p_fe + mean(e.^2)/hw.Rfe;
    p_num_ld = p_num_ld + hw.Ld1/(2*tc)*mean(di1.^2) + hw.Ld2/(2*tc)*mean(di2.^2);
    p_num_lm = p_num_lm + hw.Lm/(2*tc)*mean(diff(im).^2);
    im_dc(k) = mean(im);
    im_pk(k) = max(abs(im));
end
% switches: on-state (Rds_on) and off-state conduction of the electrical model, ploss outputs of the blocks
p_on = zeros(1, 2); p_off = zeros(1, 2); p_blk = zeros(1, 2);
for b = 1:2
    if b == 1
        D = inverter_1_dab_devices_modA_sim; V = mean(V1);
    else
        D = inverter_2_dab_devices_modA_sim; V = mean(V2);
    end
    v = D(:, 2:5:end);
    is = D(:, 3:5:end);
    on = v < 0.1*V;
    p_on(b) = sum(mean(v.*is.*on));
    p_off(b) = sum(mean(v.*is.*~on));
    p_blk(b) = sum(mean(D(:, 1:5:end)));
end
p_sum = p_cu + p_fe + p_num_ld + p_num_lm + sum(p_on) + sum(p_off);     % ESR of the DC links not included:
                                                            % their current is not logged (a few W)
fprintf('Losses [W]: windings %.0f, cores %.0f, conduction %.0f + %.0f, off-state conduction %.0f + %.0f\n', ...
    p_cu, p_fe, p_on, p_off);
fprintf('            numerical damping (Backward Euler, tc = %.0f ns): leakage inductances %.0f, Lm %.0f\n', ...
    1e9*tc, p_num_ld, p_num_lm);
fprintf('            sum %.0f W against %.0f W between the DC links; physical (windings, cores, conduction) %.0f W,\n', ...
    p_sum, Pin1 + Pin2 - dEc, p_cu + p_fe + sum(p_on));
fprintf('            eta %.2f %% without numerical damping and off-state conduction (no switching losses)\n', ...
    100*(1 - (p_cu + p_fe + sum(p_on))/max(abs(Pin1), abs(Pin2))));
fprintf('  ploss outputs of the bridges (conduction + Eon + Eoff at every edge): %.0f W + %.0f W\n', p_blk);

%[text] ## Magnetizing current and DC side
fprintf('Magnetizing currents: DC offset %.1f / %.1f / %.1f A, peak %.1f / %.1f / %.1f A (harmonic model %.1f A)\n', ...
    im_dc, im_pk, o.Im_pk);
f = (0:numel(t)-1)'/Tw;
names = {'V1', 'I1', 'V2', 'I2'};
sigs = [V1 I1 V2 I2];
for k = 1:4
    x = sigs(:, k);
    X = abs(fft(x - mean(x)))/numel(x)*2;
    lf = find(f > 0 & f < 5e3);
    [a_lf, j_lf] = max(X(lf));
    hf = find(abs(f - 6*fs) < 0.5*fs);
    xm = movmean(x, Np);
    fprintf('  %s: mean %.2f, over one period %.2f to %.2f; largest component below 5 kHz %.2f at %.0f Hz; %.3f at %.0f kHz\n', ...
        names{k}, mean(x), min(xm(Np:end-Np)), max(xm(Np:end-Np)), a_lf, f(lf(j_lf)), max(X(hf)), 6*fs/1e3);
end
clear hw dev fs tc td c_leg n12 t V1 I1 V2 I2 u1 i1 u2 i2 Tw Np Pin1 Pin2 dEc eta ed1 ed2 dn phk k j phi sw1 sw2
clear b sw V p o I1rms I2rms p_cu p_fe p_num_ld p_num_lm im_dc im_pk di1 di2 e im p_on p_off p_blk D v is on
clear p_sum f names sigs x X lf a_lf j_lf hf xm
