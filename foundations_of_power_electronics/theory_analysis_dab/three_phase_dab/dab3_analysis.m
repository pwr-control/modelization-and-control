%[text] # dab3_analysis: steady state of the three-phase DAB (harmonic superposition)
% Rev. 01, 2026-10-29: DAB3 1200 V -> 1200 V, 250 kW, fs = 10 kHz. Reads param (init_model.m or a saved run).
% Per phase, T equivalent referred to the primary (./private/dab3_harmonic_op.m): Rs1 + Rds_on and Ld1 on the
% primary, Lm // Rfe, Rs2 + Rds_on and Ld2 of the secondary referred to the primary; six-step voltages of both
% bridges, bridge 2 lags bridge 1 by phi. Prints the nominal point (currents, switching currents, ZVS
% transition times, loss estimate) and the map over the load and the voltage ratio d = V2 n1/(n2 V1);
% figures: power against phi, waveforms at the nominal point, ZVS map. The numbers are those of
% dab3_sizing.md.
% Losses: conduction and windings from the rms currents, core from Rfe, switching from the energies of the
% dataset scaled linearly with V*I: turn-off at the switching current (an upper bound with ZVS, Coss acts as
% a snubber), turn-on only where the switching current has the wrong sign (hard switching).

close all
prv = dab3_private();                                       % ./private through handles (dab3_private.m)
hw = param.dab3.hw;
dev = param.dab3.mosfet;
fs = param.dab3.fs;
n12 = hw.n1/hw.n2;
L = hw.Ld1 + hw.Ld2*n12^2;                                  % [H] series inductance per phase, primary
td = param.dab3.pwm.dead_time;                              % [s]
c_leg = 2*dev.Coss;                                         % [F] one leg: two Coss
eoff = dev.Eoff/(dev.Voff_sw_losses*dev.Ion_sw_losses);     % [J/(V A)]
eon = dev.Eon/(dev.Voff_sw_losses*dev.Ion_sw_losses);       % [J/(V A)]
p = struct('V1', hw.udc1_nom, 'V2', hw.udc2_nom, 'n1', hw.n1, 'n2', hw.n2, 'fs', fs, ...
    'L1', hw.Ld1, 'R1', hw.Rs1 + dev.Rds_on, 'L2r', hw.Ld2*n12^2, ...
    'R2r', (hw.Rs2 + dev.Rds_on)*n12^2, 'Lm', hw.Lm, 'Rfe', hw.Rfe);
% losses [W]: conduction, turn-off, hard turn-on, windings, core (6 switchings of each kind per bridge and period)
loss = @(o, V1, V2) [3*dev.Rds_on*(o.I1rms^2 + o.I2rms^2), ...
    6*fs*eoff*(V1*max(o.i_sw1, 0) + V2*max(o.i_sw2, 0)), ...
    6*fs*eon*(V1*max(-o.i_sw1, 0) + V2*max(-o.i_sw2, 0)), ...
    3*(hw.Rs1*o.I1rms^2 + hw.Rs2*o.I2rms^2), ...
    2*V1^2/(3*hw.Rfe)];

%[text] ## Nominal point
op = prv.dab3_ideal_op(p.V1, p.V2*n12, fs, L, param.dab3.pwr_nom);
o = prv.dab3_harmonic_op(p, op.phi);
ls = loss(o, p.V1, p.V2);
fprintf('Nominal point, %.0f V / %.0f V, L = %.1f uH: phi = %.2f deg (ideal), P1 = %.1f kW, P2 = %.1f kW\n', ...
    p.V1, p.V2, 1e6*L, rad2deg(op.phi), o.P1/1e3, o.P2/1e3);
fprintf('  phase currents: %.1f A rms / %.1f A peak (side 1), %.1f A rms / %.1f A peak (side 2)\n', ...
    o.I1rms, o.I1pk, o.I2rms, o.I2pk);
fprintf('  switching currents: bridge 1 %.1f A, bridge 2 %.1f A -> transitions %.0f ns / %.0f ns (dead time %.0f ns)\n', ...
    o.i_sw1, o.i_sw2, 1e9*c_leg*p.V1/o.i_sw1, 1e9*c_leg*p.V2/o.i_sw2, 1e9*td);
fprintf('  magnetizing current %.1f A peak, flux linkage %.2f mVs -> B = %.3f T\n', ...
    o.Im_pk, 1e3*o.lam_pk, o.lam_pk/(hw.n1*param.dab3.hw_in.core_area));
fprintf('  DC currents %.1f A / %.1f A, capacitor ripple %.1f A rms / %.1f A rms at %.0f kHz\n', ...
    o.Idc1, o.Idc2, o.Icap1_rms, o.Icap2_rms, 6*fs/1e3);
fprintf(['  losses: conduction %.0f W, turn-off %.0f W (upper bound), hard turn-on %.0f W, windings %.0f W, ' ...
    'core %.0f W; total %.2f kW, eta = %.2f %% (%.2f %% without turn-off)\n'], ls, sum(ls)/1e3, ...
    100*(1 - sum(ls)/o.P1), 100*(1 - (sum(ls) - ls(2))/o.P1));

figure;
subplot(3, 1, 1);
plot(rad2deg(o.theta), o.ia, 'k', rad2deg(o.theta), o.ja*n12, 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);
grid on; ylabel('A'); legend('i_{1a}', 'i_{2a}'); title('DAB3, nominal point: phase currents');
subplot(3, 1, 2);
plot(rad2deg(o.theta), o.ia - o.ja, 'k', 'LineWidth', 1.5);
grid on; ylabel('A'); title('magnetizing + iron current, primary');
subplot(3, 1, 3);
plot(rad2deg(o.theta), o.idc1, 'k', rad2deg(o.theta), o.idc2, 'Color', [0.6 0.6 0.6], 'LineWidth', 1.5);
grid on; ylabel('A'); xlabel('\omega t / deg'); legend('i_{dc1}', 'i_{dc2}'); title('bridge DC currents');

%[text] ## Power against phase shift
phi = deg2rad(0:2.5:120);
k = p.V1*p.V2*n12/(2*pi*fs*L);
a = abs(phi);
P_id = k*sign(phi).*((a <= pi/3).*a.*(2/3 - a/(2*pi)) + (a > pi/3).*(a - a.^2/pi - pi/18));
P_h = zeros(size(phi));
I_h = zeros(size(phi));
for j = 1:numel(phi)
    oj = prv.dab3_harmonic_op(p, phi(j), 600, 301);
    P_h(j) = oj.P2;
    I_h(j) = oj.I1rms;
end
figure;
subplot(2, 1, 1);
plot(rad2deg(phi), P_id/1e3, 'k--', rad2deg(phi), P_h/1e3, 'k', 'LineWidth', 1.5); grid on
hold on; plot(rad2deg(param.dab3.mod.phi_max)*[1 1], [0 1.1*max(P_id)/1e3], 'r:'); hold off
ylabel('P / kW'); legend('ideal', 'with Lm, Rfe, R (side 2)', 'phi_{max}', 'Location', 'southeast');
title('DAB3: power against phase shift');
subplot(2, 1, 2);
plot(rad2deg(phi), I_h, 'k', 'LineWidth', 1.5); grid on
xlabel('\phi / deg'); ylabel('I_{1} / A rms');

%[text] ## Map: load and voltage ratio
% V1 fixed, V2 = d V1 n2/n1; ZVS of a bridge when its switching current charges the leg within the dead time
d_list = [0.85 0.9 0.95 1 1.05 1.1 1.15];
load_list = [0.1 0.25 0.5 0.75 1];
fprintf('\n   d   load   phi/deg  I1/A rms  i_sw1/A  i_sw2/A   ZVS1  ZVS2  loss/W   eta/%%\n');
for d = d_list
    for x = load_list
        pd = p;
        pd.V2 = d*p.V1/n12;
        opd = prv.dab3_ideal_op(pd.V1, pd.V2*n12, fs, L, x*param.dab3.pwr_nom);
        od = prv.dab3_harmonic_op(pd, opd.phi, 1800, 1001);
        ld = loss(od, pd.V1, pd.V2);
        z1 = od.i_sw1 >= c_leg*pd.V1/td;
        z2 = od.i_sw2 >= c_leg*pd.V2/td;
        fprintf('%5.2f %5.2f %8.2f %9.1f %8.1f %8.1f %6d %5d %7.0f %7.2f\n', d, x, rad2deg(opd.phi), od.I1rms, ...
            od.i_sw1, od.i_sw2, z1, z2, sum(ld), 100*(1 - sum(ld)/od.P1));
    end
end

d_grid = 0.8:0.025:1.2;
x_grid = 0.05:0.05:1;
isw1 = zeros(numel(x_grid), numel(d_grid));
isw2 = isw1;
for jd = 1:numel(d_grid)
    for jx = 1:numel(x_grid)
        pd = p;
        pd.V2 = d_grid(jd)*p.V1/n12;
        opd = prv.dab3_ideal_op(pd.V1, pd.V2*n12, fs, L, x_grid(jx)*param.dab3.pwr_nom);
        od = prv.dab3_harmonic_op(pd, opd.phi, 600, 301);
        isw1(jx, jd) = od.i_sw1 - c_leg*pd.V1/td;            % > 0: ZVS of bridge 1 within the dead time
        isw2(jx, jd) = od.i_sw2 - c_leg*pd.V2/td;
    end
end
figure;
contour(d_grid, 100*x_grid, isw1, [0 0], 'k', 'LineWidth', 2); hold on
contour(d_grid, 100*x_grid, isw2, [0 0], 'k--', 'LineWidth', 2); hold off
grid on; xlabel('d = V_2 n_1 / (n_2 V_1)'); ylabel('load / %');
legend('bridge 1', 'bridge 2', 'Location', 'north');
title(sprintf('DAB3 ZVS boundaries (dead time %.0f ns, C_{leg} = %.2f nF): ZVS above the lines', ...
    1e9*td, 1e9*c_leg));
clear hw dev fs n12 L td c_leg eoff eon p loss op o ls phi k a P_id P_h I_h j oj d_list load_list d x pd opd od ld z1 z2
clear d_grid x_grid isw1 isw2 jd jx
