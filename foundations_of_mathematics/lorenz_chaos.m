%% Lorenz System - Traiettorie e sensibilità alle condizioni iniziali
% Davide Bagnara - Caos e ODE
% Sistema: dx/dt = sigma*(y-x)
%          dy/dt = x*(rho-z) - y
%          dz/dt = x*y - beta*z

clear; close all; clc;

%% Parametri classici di Lorenz (regime caotico)
sigma = 10;
rho   = 28;
beta  = 8/3;

lorenz = @(t, s) [
    sigma * (s(2) - s(1));
    s(1) * (rho - s(3)) - s(2);
    s(1) * s(2) - beta * s(3)
];

%% Tre condizioni iniziali quasi identiche (differenza 1e-8 su x)
x0_base = [1.0; 1.0; 1.0];
delta   = 1e-8;

ic = [x0_base, ...
      x0_base + [delta; 0; 0], ...
      x0_base + [2*delta; 0; 0]];

%% Integrazione con ode45
tspan   = [0 50];
options = odeset('RelTol', 1e-10, 'AbsTol', 1e-12);

colors = [0.00 0.45 0.74;   % blu
          0.85 0.33 0.10;   % rosso
          0.47 0.67 0.19];  % verde

sol = cell(1, 3);
for k = 1:3
    sol{k} = ode45(lorenz, tspan, ic(:,k), options);
end

% Campionamento uniforme per i plot 2D
t_plot = linspace(tspan(1), tspan(2), 8000);
S = cell(1,3);
for k = 1:3
    S{k} = deval(sol{k}, t_plot);
end

%% ---------------------------------------------------------------
%  Figura 1: Attrattore di Lorenz 3D (prima traiettoria)
%% ---------------------------------------------------------------
figure('Name','Attrattore di Lorenz','Color','k','Position',[100 100 800 650]);
ax = axes('Color','k','XColor','w','YColor','w','ZColor','w');
hold on;

% Colora la traiettoria per tempo (gradiente)
n  = size(S{1}, 2);
cm = parula(n);
for i = 1:n-1
    plot3(S{1}(1,i:i+1), S{1}(2,i:i+1), S{1}(3,i:i+1), ...
          'Color', cm(i,:), 'LineWidth', 0.6);
end

xlabel('x', 'Color','w'); ylabel('y', 'Color','w'); zlabel('z', 'Color','w');
title('Attrattore di Lorenz  (\sigma=10, \rho=28, \beta=8/3)', ...
      'Color','w', 'FontSize', 13);
grid on; ax.GridColor = [0.3 0.3 0.3];
view(30, 25);
cb = colorbar; cb.Color = 'w';
ylabel(cb, 'Tempo [s]', 'Color','w');
colormap(ax, parula);
clim([t_plot(1) t_plot(end)]);

%% ---------------------------------------------------------------
%  Figura 2: Confronto traiettorie - proiezione XY, XZ
%% ---------------------------------------------------------------
figure('Name','Confronto proiezioni','Position',[950 100 900 600]);
labels = {'\delta = 0', '\delta = 10^{-8}', '\delta = 2\times10^{-8}'};

subplot(1,2,1); hold on; grid on;
for k = 1:3
    plot(S{k}(1,:), S{k}(2,:), 'Color', colors(k,:), ...
         'LineWidth', 0.7, 'DisplayName', labels{k});
end
xlabel('x'); ylabel('y');
title('Proiezione X-Y');
legend('Location','best'); legend('boxoff');

subplot(1,2,2); hold on; grid on;
for k = 1:3
    plot(S{k}(1,:), S{k}(3,:), 'Color', colors(k,:), ...
         'LineWidth', 0.7, 'DisplayName', labels{k});
end
xlabel('x'); ylabel('z');
title('Proiezione X-Z');
legend('Location','best'); legend('boxoff');

sgtitle('Attrattore di Lorenz: tre condizioni iniziali \Deltax = 10^{-8}', ...
        'FontSize', 13);

%% ---------------------------------------------------------------
%  Figura 3: Divergenza nel tempo - norma della differenza
%% ---------------------------------------------------------------
figure('Name','Sensibilità condizioni iniziali','Position',[100 750 900 400]);

diff12 = vecnorm(S{1} - S{2});   % distanza tra traiettoria 1 e 2
diff13 = vecnorm(S{1} - S{3});

semilogy(t_plot, diff12, 'Color', colors(2,:), 'LineWidth', 1.5, ...
         'DisplayName', '||\Deltax(t)||  (\delta=10^{-8})');
hold on;
semilogy(t_plot, diff13, 'Color', colors(3,:), 'LineWidth', 1.5, ...
         'DisplayName', '||\Deltax(t)||  (\delta=2\times10^{-8})');

% Stima esponente di Lyapunov: fit lineare su scala log nei primi 15s
mask = t_plot < 15;
p = polyfit(t_plot(mask), log(diff12(mask)), 1);
lambda_est = p(1);
semilogy(t_plot(mask), exp(polyval(p, t_plot(mask))), 'w--', ...
         'LineWidth', 1.2, 'DisplayName', ...
         sprintf('Fit \\lambda \\approx %.2f s^{-1}', lambda_est));

xlabel('Tempo [s]'); ylabel('||\Deltax(t)||  (scala log)');
title('Sensibilità alle condizioni iniziali - Divergenza esponenziale');
legend('Location','southeast'); legend('boxoff');
grid on;

fprintf('\n--- Stima esponente di Lyapunov massimo ---\n');
fprintf('  lambda_1 ~ %.4f s^-1  (letteratura: ~0.906)\n\n', lambda_est);
