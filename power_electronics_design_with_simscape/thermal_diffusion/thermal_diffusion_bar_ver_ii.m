%% 1D Thermal Diffusion: Aluminium Bar
%  Left BC   (x=0): imposed heat flux  q_in = 25 W  (Neumann)
%  Right BC  (x=L): fixed temperature  T_sink = 80 °C (Dirichlet)
%
%  Geometry: 3 mm x 30 mm x 50 mm (cross-section A = 9e-5 m^2)
%
%  Method: explicit finite differences (FTCS) with ghost node for
%          the Neumann condition at the left boundary.
%

clear; clc; close all;

%% ── 1. Physical Parameters ──────────────────────────────────────────────
L     = 0.050;          % Bar length                    [m]
alpha = 98e-6;          % Thermal diffusivity of Al     [m²/s]
k     = 237;            % Thermal conductivity of Al    [W/(m·K)]
A     = 3e-3 * 30e-3;   % Cross-section: 3 mm × 30 mm   [m²]  = 9e-5 m²

% Boundary conditions
q_in   = 25;        % Imposed heat flux at x=0          [W]
T_sink = 80;        % Sink temperature at x=L           [°C]
T0     = 25;        % Initial temperature (ambient)     [°C]

%% ── 2. Spatial Discretisation ───────────────────────────────────────────
Nx = 200;                    % Internal nodes + boundaries
dx = L / (Nx - 1);           % Spatial step
x  = linspace(0, L, Nx);     % Grid [m]

% CFL coefficient — explicit stability condition: lambda < 0.5
% Safety factor 0.45 applied
lambda = 0.45;
dt     = lambda * dx^2 / alpha;   % Time step [s]

T_final = 100;               % Simulation duration [s] — sufficient to reach steady state
Nt      = ceil(T_final / dt);

fprintf('Solver parameters:\n');
fprintf('  dx = %.4f mm | dt = %.6f s | Nt = %d | lambda = %.4f\n', ...
        dx*1e3, dt, Nt, lambda);

%% ── 3. Analytical Steady-State Solution (verification) ─────────────────
% At steady state: d²T/dx² = 0  →  T(x) = a·x + b
% T'(0) = -q_in/(k·A)   →  a = -q_in/(k·A)
% T(L)  = T_sink         →  b = T_sink - a·L
a_ss = -q_in / (k * A);
b_ss = T_sink - a_ss * L;
T_ss = a_ss * x + b_ss;       % Linear steady-state profile

fprintf('  Temperature at x=0 (steady state): %.2f °C\n', T_ss(1));
fprintf('  Temperature at x=L (steady state): %.2f °C  (expected: %.0f °C)\n\n', T_ss(end), T_sink);

%% ── 4. Initialisation ───────────────────────────────────────────────────
u     = T0 * ones(1, Nx);   % Uniform initial condition
u(end) = T_sink;             % Enforce Dirichlet BC on the right immediately

% Arrays to record the time history
t_hist = zeros(1, Nt);
T_x0_hist = zeros(1, Nt);   % T(x=0) over time
T_xmid_hist = zeros(1, Nt); % T(x=L/2) over time

%% ── 5. Plot Setup ───────────────────────────────────────────────────────
fig = figure('Name', '1D Thermal Diffusion', ...
             'Position', [100 100 1000 520], ...
             'Color', [0.9 0.9 0.9]);

% Subplot 1: Spatial profile
ax1 = subplot(1,2,1);
hProf = plot(ax1, x*1e3, u, '-','color', [0.25 0.25 0.25], 'LineWidth', 2); hold on;
hSS   = plot(ax1, x*1e3, T_ss, '--','color', [0.75 0.75 0.75], 'LineWidth', 2);
xlabel(ax1, 'Position x [mm]');
ylabel(ax1, 'Temperature [°C]');
title(ax1, 'Thermal Profile');
legend(ax1, 'Transient', 'Steady state (analytical)', 'Location', 'SouthEast');
grid(ax1, 'on');
ylim(ax1, [T0-5, T_ss(1)+10]);
xlim(ax1, [0, L*1e3]);

% Subplot 2: Time history
ax2 = subplot(1,2,2);
hH0   = plot(ax2, NaN, NaN, '-','color', [0.25 0.25 0.25], 'LineWidth', 2); hold on;
hHmid = plot(ax2, NaN, NaN, '-','color', [0.50 0.50 0.50], 'LineWidth', 2);
hSS0  = yline(ax2, T_ss(1), '--','color', [0.75 0.75 0.75], 'LineWidth', 2);
xlabel(ax2, 'Time [s]');
ylabel(ax2, 'Temperature [°C]');
title(ax2, 'Convergence to Steady State');
legend(ax2, 'T(x=0)', 'T(x=L/2)', 'T_{steady}(x=0)', 'Location', 'SouthEast');
grid(ax2, 'on');
ylim(ax2, [T0-5, T_ss(1)+10]);

sgtitle(fig, sprintf('Al bar 3×30×50 mm | q_{in}=%.0f W | T_{sink}=%.0f °C', ...
                     q_in, T_sink), 'FontSize', 12, 'FontWeight', 'bold');

%% ── 6. Time-Stepping Loop ───────────────────────────────────────────────
disp('Starting simulation...');
plot_interval = max(1, floor(Nt / 300)); % ~300 total plot updates

for n = 1:Nt

    u_new = u;

    % ── A. Interior nodes: standard FTCS scheme ──────────────────────────
    u_new(2:end-1) = u(2:end-1) + lambda * ...
                     (u(3:end) - 2*u(2:end-1) + u(1:end-2));

    % ── B. Left BC — Neumann: imposed heat flux q_in  (ghost node) ───────
    %  -k·A·(dT/dx)|_{x=0} = q_in
    %  Ghost node: u_ghost = u(2) + 2·dx·q_in/(k·A)
    %  → u_new(1) = u(1) + lambda·(u(2) - 2·u(1) + u_ghost)
    %             = u(1) + lambda·(2·u(2) - 2·u(1) + 2·dx·q_in/(k·A))
    u_ghost  = u(2) + 2 * dx * q_in / (k * A);
    u_new(1) = u(1) + lambda * (u(2) - 2*u(1) + u_ghost);

    % ── C. Right BC — Dirichlet: fixed temperature ────────────────────────
    u_new(end) = T_sink;

    % ── D. Update state ───────────────────────────────────────────────────
    u = u_new;

    % ── E. Record time history ────────────────────────────────────────────
    t_hist(n)      = n * dt;
    T_x0_hist(n)   = u(1);
    T_xmid_hist(n) = u(ceil(Nx/2));

    % ── F. Update plot ────────────────────────────────────────────────────
    if mod(n, plot_interval) == 0
        set(hProf, 'YData', u);

        idx = 1:n;
        set(hH0,   'XData', t_hist(idx),   'YData', T_x0_hist(idx));
        set(hHmid, 'XData', t_hist(idx),   'YData', T_xmid_hist(idx));

        sgtitle(fig, sprintf('Al bar 3×30×50 mm | t = %.1f s / %.0f s', ...
                             n*dt, T_final), ...
                'FontSize', 12, 'FontWeight', 'bold');
        drawnow limitrate;
    end
end

%% ── 7. Convergence Check ────────────────────────────────────────────────
err_max = max(abs(u - T_ss));
fprintf('Simulation complete.\n');
fprintf('  Max error vs. analytical steady state: %.4f °C\n', err_max);

% Final plot with clearly visible analytical solution overlay
set(hProf, 'YData', u, 'Color', [0.5 0.5 0.5]);
set(hH0,   'XData', t_hist, 'YData', T_x0_hist);
set(hHmid, 'XData', t_hist, 'YData', T_xmid_hist);
set(gca, 'xlim', [0 T_final]);
sgtitle(fig, sprintf('STEADY STATE REACHED | Al bar 3×30×50 mm | Max error: %.3f °C', err_max), ...
        'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.25 0.25 0.25]);
drawnow;
