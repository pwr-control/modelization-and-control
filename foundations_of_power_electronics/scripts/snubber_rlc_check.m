%% snubber_rlc_check.m
% RC snubber verification for a high-power three-phase diode bridge.
% Companion to snubber_ponte_diodi.tex.
%
%   1. commutation inductance, overlap angle, di/dt at turn-off
%   2. reverse recovery peak current from Qrr and softness factor
%   3. snap-off transient of the series RLC (2*Lc, R, C) -> VRM
%   4. Mohan normalisation (Cbase, Rbase)
%   5. loss estimate
%   6. R sweep at fixed C, C sweep with R = Z0(C)
%
% The ideal snap-off (all of Irr diverted into the snubber at t = 0,
% capacitor discharged) is the conservative reference case used by
% McMurray (1972) and Mohan. Set alpha_deg > 0 to evaluate a thyristor
% bridge at a given firing angle (recovery part only; dv/dt, firing
% discharge and voltage-step losses are reported as separate figures).

clear; close all; clc;

%% 1. Input data ---------------------------------------------------------
Vll      = 690;       % line-to-line rms voltage [V]
f        = 50;        % grid frequency [Hz]
Id       = 4000;      % DC current [A]
Sn       = 4.5e6;     % transformer rating [VA]
uk       = 0.06;      % transformer short-circuit voltage [p.u.]
Lext     = 0;         % extra line/busbar inductance per phase [H]
Qrr_ref  = 2.0e-3;    % recovery charge from datasheet [C] ...
didt_ref = 7.7e6;     % ... at this di/dt [A/s], Tvj,max
scaleQrr = true;      % scale Qrr ~ sqrt(di/dt) around the reference point
S        = 0.3;       % softness factor tb/ta
C        = 4.7e-6;    % snubber capacitance [F]
R        = 3.3;       % snubber resistance [ohm]
VRRM     = 2800;      % device voltage class [V]
kVRM     = 0.5;       % allowed repetitive peak as a fraction of VRRM
alpha_deg = 0;        % firing angle: 0 = diode bridge, > 0 = thyristor bridge

%% 2. Commutation --------------------------------------------------------
w     = 2*pi*f;
Vpk   = sqrt(2)*Vll;
Zb    = Vll^2/Sn;
Lc    = uk*Zb/w + Lext;                  % commutation inductance per phase [H]
L     = 2*Lc;                            % loop inductance seen by the transient
alpha = alpha_deg*pi/180;
mu    = acos(cos(alpha) - 2*w*Lc*Id/Vpk) - alpha;   % overlap angle [rad]
Vcom  = Vpk*sin(alpha + mu);             % commutation voltage at turn-off [V]
didt  = Vcom/L;                          % di/dt at turn-off [A/s]

%% 3. Reverse recovery ---------------------------------------------------
if scaleQrr
    Qrr = Qrr_ref*sqrt(didt/didt_ref);
else
    Qrr = Qrr_ref;
end
Irr = sqrt(2*Qrr*didt/(1+S));
ta  = Irr/didt;
trr = ta*(1+S);
EL  = 0.5*L*Irr^2;                       % = Qrr*Vcom/(1+S)

%% 4. Snap-off transient -------------------------------------------------
Z0   = sqrt(L/C);
zeta = R/(2*Z0);
f0   = 1/(2*pi*sqrt(L*C));
[t, vD, iL, vC] = snap_transient(Vcom, Irr, L, C, R);
[VRM, kmax] = max(vD);
tVRM = t(kmax);

%% 5. Mohan normalisation ------------------------------------------------
Cbase = L*(Irr/Vcom)^2;
Rbase = Vcom/Irr;

%% 6. Losses -------------------------------------------------------------
% diode bridge: one recovery event per device per cycle; the blocking
% voltage evolves continuously, so no step-induced losses
Erec = EL + 0.5*C*Vcom^2;                % energy dissipated in R per event [J]
Pfol = 0.5*R*(C*w*Vpk)^2;                % waveform-following term, rough bound [W]
PR   = Erec*f + Pfol;
if alpha > 0
    Vfire  = Vpk*sin(alpha);             % device voltage at firing
    Efire  = 0.5*C*Vfire^2;              % capacitor discharge through R at firing
    Idisch = Vfire/R;                    % peak discharge current into the device
    PR     = PR + Efire*f;               % step losses from other legs NOT included
end

%% 7. Report -------------------------------------------------------------
fprintf('--- Commutation ---------------------------------------------\n');
fprintf('Lc = %.1f uH  (2Lc = %.1f uH)\n', Lc*1e6, L*1e6);
fprintf('mu = %.2f deg,  Vcom = %.0f V,  di/dt = %.2f A/us\n', ...
        mu*180/pi, Vcom, didt*1e-6);
fprintf('--- Reverse recovery ----------------------------------------\n');
fprintf('Qrr = %.2f mC,  Irr = %.0f A,  ta = %.1f us,  trr = %.1f us\n', ...
        Qrr*1e3, Irr, ta*1e6, trr*1e6);
fprintf('EL = 0.5*(2Lc)*Irr^2 = %.3f J   (Qrr*Vcom/(1+S) = %.3f J)\n', ...
        EL, Qrr*Vcom/(1+S));
fprintf('--- Snubber -------------------------------------------------\n');
fprintf('C = %.2f uF,  R = %.2f ohm,  Z0 = %.2f ohm,  zeta = %.2f,  f0 = %.1f kHz\n', ...
        C*1e6, R, Z0, zeta, f0*1e-3);
fprintf('R*Irr = %.0f V,  Irr*Z0 = %.0f V\n', R*Irr, Irr*Z0);
if VRM <= kVRM*VRRM, status = 'OK'; else, status = 'FAIL'; end
fprintf('VRM = %.0f V at t = %.1f us  (limit %.0f V = %.2f*VRRM)  -> %s\n', ...
        VRM, tVRM*1e6, kVRM*VRRM, kVRM, status);
fprintf('Mohan: Cbase = %.2f uF, Rbase = %.2f ohm  ->  C/Cbase = %.2f, R/Rbase = %.2f\n', ...
        Cbase*1e6, Rbase, C/Cbase, R/Rbase);
fprintf('--- Losses --------------------------------------------------\n');
fprintf('Energy per recovery event in R = %.2f J  ->  %.0f W\n', Erec, Erec*f);
fprintf('Total P_R = %.0f W per snubber\n', PR);
if alpha > 0
    fprintf('Firing: V = %.0f V, discharge peak = %.0f A, %.2f J -> %.0f W\n', ...
            Vfire, Idisch, Efire, Efire*f);
end

%% 8. R sweep at fixed C -------------------------------------------------
Rvec  = linspace(0.25, 4*Z0, 60);
VRM_R = zeros(size(Rvec));
for n = 1:numel(Rvec)
    [~, v] = snap_transient(Vcom, Irr, L, C, Rvec(n));
    VRM_R(n) = max(v);
end
[VRM_opt, nopt] = min(VRM_R);
fprintf('R sweep: min VRM = %.0f V at R = %.2f ohm (%.2f*Z0)\n', ...
        VRM_opt, Rvec(nopt), Rvec(nopt)/Z0);

%% 9. C sweep with R = Z0(C) --------------------------------------------
Cvec  = logspace(log10(0.5e-6), log10(20e-6), 30);
VRM_C = zeros(size(Cvec));
for n = 1:numel(Cvec)
    [~, v] = snap_transient(Vcom, Irr, L, Cvec(n), sqrt(L/Cvec(n)));
    VRM_C(n) = max(v);
end
Prec_C = (EL + 0.5*Cvec*Vcom^2)*f;

%% 10. Plots -------------------------------------------------------------
figure('Name', 'Snap-off transient');
subplot(2,1,1);
plot(t*1e6, vD, 'LineWidth', 1.5); hold on; grid on;
plot([t(1) t(end)]*1e6, Vcom*[1 1], 'k--');
plot([t(1) t(end)]*1e6, kVRM*VRRM*[1 1], 'r--');
ylabel('v_D [V]');
title(sprintf('C = %.1f uF, R = %.2f ohm: V_{RM} = %.0f V', C*1e6, R, VRM));
legend('v_D', 'V_{com}', 'limit', 'Location', 'best');
subplot(2,1,2);
plot(t*1e6, iL, t*1e6, vC/R, 'LineWidth', 1.2); grid on;
xlabel('t [us]'); ylabel('[A]');
legend('i_L = i_{snubber}', 'v_C / R', 'Location', 'best');

figure('Name', 'Sweeps');
subplot(1,2,1);
plot(Rvec, VRM_R, 'LineWidth', 1.5); hold on; grid on;
plot(Rvec, Vcom + Rvec*Irr, 'r--');
plot([Z0 Z0], [min(VRM_R)*0.9 max(VRM_R)], 'k:');
xlabel('R [ohm]'); ylabel('V_{RM} [V]');
title(sprintf('R sweep, C = %.1f uF', C*1e6));
legend('V_{RM}', 'V_{com} + R I_{rr}', 'Z_0', 'Location', 'northwest');
subplot(1,2,2);
yyaxis left;  semilogx(Cvec*1e6, VRM_C, 'LineWidth', 1.5); ylabel('V_{RM} [V]');
yyaxis right; semilogx(Cvec*1e6, Prec_C, 'LineWidth', 1.2); ylabel('P_{rec} [W]');
grid on; xlabel('C [uF]'); title('C sweep, R = Z_0(C)');

%% Local function --------------------------------------------------------
function [t, vD, iL, vC] = snap_transient(Vcom, I0, L, C, R)
% Series RLC after ideal snap-off: source Vcom, loop inductance L,
% snubber R-C in place of the diode. State x = [iL; vC],
% iL(0) = I0, vC(0) = 0, diode voltage vD = vC + R*iL.
    A    = [-R/L, -1/L; 1/C, 0];
    b    = [Vcom/L; 0];
    w0   = 1/sqrt(L*C);
    zeta = R/2*sqrt(C/L);
    tend = max(8/(max(zeta, 0.05)*w0), 4*2*pi/w0);
    opts = odeset('RelTol', 1e-8, 'AbsTol', 1e-9, 'MaxStep', tend/2000);
    [t, x] = ode45(@(t, x) A*x + b, [0 tend], [I0; 0], opts);
    iL = x(:,1);
    vC = x(:,2);
    vD = vC + R*iL;
end
