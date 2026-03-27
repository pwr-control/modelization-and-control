%% Diffusione Termica 1D: Sbarra di Alluminio
%  BC sinistra  (x=0): flusso imposto  q_in = 25 W  (Neumann)
%  BC destra    (x=L): temperatura fissa T_sink = 75 °C (Dirichlet)
%
%  Geometria: 3 mm x 30 mm x 50 mm (sezione A = 9e-5 m^2)
%
%  Metodo: differenze finite esplicite (FTCS) con ghost-node per
%          la condizione di Neumann al bordo sinistro.
%

clear; clc; close all;

%% ── 1. Parametri Fisici ─────────────────────────────────────────────────
L     = 0.050;          % Lunghezza sbarra              [m]
alpha = 98e-6;          % Diffusività termica Al        [m²/s]
k     = 237;            % Conducibilità termica Al      [W/(m·K)]
A     = 3e-3 * 30e-3;   % Sezione: 3 mm × 30 mm         [m²]  = 9e-5 m²

% Condizioni al contorno
q_in   = 25;        % Flusso termico imposto a x=0      [W]
T_sink = 75;        % Temperatura pozzo a x=L           [°C]
T0     = 25;        % Temperatura iniziale (ambiente)   [°C]

%% ── 2. Discretizzazione Spaziale ────────────────────────────────────────
Nx = 200;                    % Nodi interni + bordi
dx = L / (Nx - 1);           % Passo spaziale
x  = linspace(0, L, Nx);     % Griglia [m]

% Coefficiente CFL — stabilità per esplicito: lambda < 0.5
% Usiamo safety factor 0.45
lambda = 0.45;
dt     = lambda * dx^2 / alpha;   % Passo temporale [s]

T_final = 120;               % Durata simulazione [s] — abbondante per il regime
Nt      = ceil(T_final / dt);

fprintf('Parametri solver:\n');
fprintf('  dx = %.4f mm | dt = %.6f s | Nt = %d | lambda = %.4f\n', ...
        dx*1e3, dt, Nt, lambda);

%% ── 3. Soluzione Analitica a Regime (verifica) ──────────────────────────
% A regime: d²T/dx² = 0  →  T(x) = a·x + b
% T'(0) = -q_in/(k·A)   →  a = -q_in/(k·A)
% T(L)  = T_sink         →  b = T_sink - a·L
a_ss = -q_in / (k * A);
b_ss = T_sink - a_ss * L;
T_ss = a_ss * x + b_ss;       % Profilo lineare a regime

fprintf('  Temp. a x=0 a regime: %.2f °C\n', T_ss(1));
fprintf('  Temp. a x=L a regime: %.2f °C  (atteso: %.0f °C)\n\n', T_ss(end), T_sink);

%% ── 4. Inizializzazione ─────────────────────────────────────────────────
u     = T0 * ones(1, Nx);   % Condizione iniziale uniforme
u(end) = T_sink;             % Applica subito la Dirichlet a destra

% Matrici per registrare la storia temporale
t_hist = zeros(1, Nt);
T_x0_hist = zeros(1, Nt);   % T(x=0) nel tempo
T_xmid_hist = zeros(1, Nt); % T(x=L/2) nel tempo

%% ── 5. Setup Grafico ────────────────────────────────────────────────────
fig = figure('Name', 'Diffusione Termica 1D', ...
             'Position', [100 100 1000 520], ...
             'Color', [0.9 0.9 0.9]);

% Subplot 1: Profilo spaziale
ax1 = subplot(1,2,1);
hProf = plot(ax1, x*1e3, u, '-','color', [0.25 0.25 0.25], 'LineWidth', 2); hold on;
hSS   = plot(ax1, x*1e3, T_ss, '--','color', [0.75 0.75 0.75], 'LineWidth', 2);
xlabel(ax1, 'Posizione x [mm]');
ylabel(ax1, 'Temperatura [°C]');
title(ax1, 'Profilo Termico');
legend(ax1, 'Transitorio', 'Regime (analitico)', 'Location', 'SouthEast');
grid(ax1, 'on');
ylim(ax1, [T0-5, T_ss(1)+10]);
xlim(ax1, [0, L*1e3]);

% Subplot 2: Storia temporale
ax2 = subplot(1,2,2);
hH0   = plot(ax2, NaN, NaN, '-','color', [0.25 0.25 0.25], 'LineWidth', 2); hold on;
hHmid = plot(ax2, NaN, NaN, '-','color', [0.50 0.50 0.50], 'LineWidth', 2);
hSS0  = yline(ax2, T_ss(1), '--','color', [0.75 0.75 0.75], 'LineWidth', 2);
xlabel(ax2, 'Tempo [s]');
ylabel(ax2, 'Temperatura [°C]');
title(ax2, 'Convergenza al Regime');
legend(ax2, 'T(x=0)', 'T(x=L/2)', 'T_{regime}(x=0)', 'Location', 'SouthEast');
grid(ax2, 'on');
ylim(ax2, [T0-5, T_ss(1)+10]);

sgtitle(fig, sprintf('Sbarra Al 3×30×50 mm | q_{in}=%.0f W | T_{sink}=%.0f °C', ...
                     q_in, T_sink), 'FontSize', 12, 'FontWeight', 'bold');

%% ── 6. Loop Temporale ───────────────────────────────────────────────────
disp('Avvio simulazione...');
plot_interval = max(1, floor(Nt / 300)); % ~300 aggiornamenti grafico totali

for n = 1:Nt

    u_new = u;

    % ── A. Nodi interni: schema FTCS standard ────────────────────────────
    u_new(2:end-1) = u(2:end-1) + lambda * ...
                     (u(3:end) - 2*u(2:end-1) + u(1:end-2));

    % ── B. BC sinistra — Neumann: flusso imposto q_in  (ghost node) ──────
    %  -k·A·(dT/dx)|_{x=0} = q_in
    %  Con ghost node: u_ghost = u(2) + 2·dx·q_in/(k·A)
    %  → u_new(1) = u(1) + lambda·(u(2) - 2·u(1) + u_ghost)
    %             = u(1) + lambda·(2·u(2) - 2·u(1) + 2·dx·q_in/(k·A))
    u_ghost  = u(2) + 2 * dx * q_in / (k * A);
    u_new(1) = u(1) + lambda * (u(2) - 2*u(1) + u_ghost);

    % ── C. BC destra — Dirichlet: temperatura fissa ───────────────────────
    u_new(end) = T_sink;

    % ── D. Aggiorna stato ─────────────────────────────────────────────────
    u = u_new;

    % ── E. Registra storia temporale ──────────────────────────────────────
    t_hist(n)      = n * dt;
    T_x0_hist(n)   = u(1);
    T_xmid_hist(n) = u(ceil(Nx/2));

    % ── F. Aggiorna grafico ───────────────────────────────────────────────
    if mod(n, plot_interval) == 0
        set(hProf, 'YData', u);

        idx = 1:n;
        set(hH0,   'XData', t_hist(idx),   'YData', T_x0_hist(idx));
        set(hHmid, 'XData', t_hist(idx),   'YData', T_xmid_hist(idx));

        sgtitle(fig, sprintf('Sbarra Al 3×30×50 mm | t = %.1f s / %.0f s', ...
                             n*dt, T_final), ...
                'FontSize', 12, 'FontWeight', 'bold');
        drawnow limitrate;
    end
end

%% ── 7. Verifica Convergenza ─────────────────────────────────────────────
err_max = max(abs(u - T_ss));
fprintf('Simulazione completata.\n');
fprintf('  Errore max vs. regime analitico: %.4f °C\n', err_max);

% Grafico finale con overlay soluzione analitica ben visibile
set(hProf, 'YData', u, 'Color', [0.5 0.5 0.5]);
set(hH0,   'XData', t_hist, 'YData', T_x0_hist);
set(hHmid, 'XData', t_hist, 'YData', T_xmid_hist);
sgtitle(fig, sprintf('REGIME RAGGIUNTO | Sbarra Al 3×30×50 mm | Errore max: %.3f °C', err_max), ...
        'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.25 0.25 0.25]);
drawnow;
