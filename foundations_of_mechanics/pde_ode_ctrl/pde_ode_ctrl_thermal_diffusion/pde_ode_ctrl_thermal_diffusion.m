%% Simulazione Accoppiata 1D: PDE (Sbarra) + ODE (Riscaldatore)
clear; clc; close all;

%% 1. Parametri Fisici
L = 2.0;            % Lunghezza sbarra [m]
alpha = 1e-4;       % Diffusività termica sbarra [m^2/s]
k = 200;            % Conducibilità termica [W/(m*K)]
A = 0.01;           % Area sezione sbarra [m^2]

% Parametri Riscaldatore (ODE)
Mc = 500;           % Capacità termica del riscaldatore [J/K] (Massa * calore specifico)
Power = 2000;       % Potenza in ingresso [Watt] (Accendiamo il riscaldatore)

%% 2. Discretizzazione (Griglia)
Nx = 150;            % Numero di nodi spaziali
dx = L / (Nx - 1);  % Passo spaziale
x = linspace(0, L, Nx);

% Calcolo passo temporale per stabilità (CFL condition)
% Per il metodo esplicito serve: alpha * dt / dx^2 < 0.5
dt = 0.1 * (dx^2) / alpha; 
T_final = 6000;      % Durata simulazione [s]
Nt = ceil(T_final / dt);

% Coefficiente lambda per la PDE
lambda = alpha * dt / dx^2;

%% 3. Inizializzazione
u = 20 * ones(1, Nx); % Temperatura iniziale ambiente (20°C)
u_new = u;            % Vettore temporaneo per il passo successivo

% Prepariamo il grafico
figure(1);
hPlot = plot(x, u, 'LineWidth', 2);
axis([0 L 15 100]); % Assi fissi: da 0 a 1m, da 15°C a 100°C
grid on;
xlabel('Posizione x [m]');
ylabel('Temperatura [°C]');
title('Sistema Accoppiato: ODE (x=0) + PDE (x>0)');
legend('Dist.    Temp. Sbarra', 'Location', 'NorthEast');

%% 4. Loop Temporale (Solver)
disp('Avvio simulazione...');

for n = 1:Nt
    
    % --- A. Risoluzione ODE (Riscaldatore al nodo 1) ---
    % Equazione: dT/dt = (Power + Calore_Perso_Verso_Sbarra) / Mc
    % Calore perso = k*A * (du/dx) -> approssimato con (u(2)-u(1))/dx
    
    gradiente_bordo = (u(2) - u(1)) / dx; 
    
    % Aggiornamento Eulero Esplicito per la ODE
    dT_dt = (Power + k*A * gradiente_bordo) / Mc; 
    u_new(1) = u(1) + dt * dT_dt;
    
    
    % --- B. Risoluzione PDE (Nodi interni della sbarra) ---
    % Equazione del Calore standard discretizzata
    % u_new(i) = u(i) + lambda * (u(i+1) - 2u(i) + u(i-1))
    
    % Metodo vettorizzato (più veloce del for loop in Matlab)
    u_new(2:end-1) = u(2:end-1) + lambda * (u(3:end) - 2*u(2:end-1) + u(1:end-2));
    
    
    % --- C. Condizione al Bordo Destro (Isolamento) ---
    % Neumann: derivata zero -> u(end) uguale a u(end-1)
    u_new(end) = u_new(end-1);
    
    
    % --- Aggiornamento e Visualizzazione ---
    u = u_new; % Aggiorna lo stato corrente
    
    % Disegna ogni 50 passi (per non rallentare troppo)
    if mod(n, 50) == 0
        set(hPlot, 'YData', u);
        title(sprintf('Tempo: %.2f s | Temp. Riscaldatore: %.1f °C', n*dt, u(1)));
        set(gca, 'ylim', [0 1000]);
        drawnow;
        pause(0.01)
    end
end

disp('Simulazione completata.');