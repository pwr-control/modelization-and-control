clear; close all; clc;

%% 1. Parametri Fisici
L = 1.0;              
c = 40;               % Velocità onda (ridotta leggermente per visualizzare meglio)
T_sim = 2.0;          % Durata simulazione (s)

% Calcolo Frequenza Propria
f1 = c / (2*L);       % Frequenza fondamentale
omega_res = 2*pi*f1;  

% --- Tuning Oscillatore ---
% 1.0 = Risonanza (l'onda cresce molto)
% 0.8 = Fuori fase (l'onda resta piccola e batte)
tuning = 1.05;         
omega_osc = omega_res * tuning;

%% 2. Discretizzazione (FDM)
Nx = 150;              % Punti spaziali
dx = L / (Nx - 1);
x = linspace(0, L, Nx);

% Calcolo dt basato su CFL per stabilità
CFL = 0.5;           
dt = CFL * dx / c;    
Nt = round(T_sim / dt); 

% Parametro Courant al quadrato
r2 = (c * dt / dx)^2; 

%% 3. Inizializzazione Campi
u       = zeros(1, Nx); % u al tempo n
u_prev  = zeros(1, Nx); % u al tempo n-1
u_next  = zeros(1, Nx); % u al tempo n+1

% Stato Oscillatore (ODE)
z = 1.0;    % Posizione iniziale oscillatore (molla estesa)
vz = 0;     % Velocità iniziale

% --- PARAMETRO CRITICO ---
% Per vedere l'effetto, la forza deve vincere la tensione della corda.
% Dividiamo per dx per simulare una densità di forza puntiforme.
coupling_strength = 5000 / dx; 

idx_source = round(Nx/2); % La forza agisce al centro

%% 4. Loop Temporale
figure('Color', 'w', 'Position', [100 100 800 600]);

for n = 1:Nt
    
    % --- A. Step ODE (Oscillatore Armonico) ---
    % Integrazione Simplettica (Semi-implicit Euler)
    az = -(omega_osc^2) * z; 
    vz = vz + az * dt;
    z  = z + vz * dt;
    
    % --- B. Step PDE (Corda) ---
    for i = 2:Nx-1
        % Laplaciano discreto
        d2u = u(i+1) - 2*u(i) + u(i-1);
        
        % Termine di Forza: esiste solo nel punto centrale
        F_term = 0;
        if i == idx_source
            % La forza è proporzionale alla posizione dell'oscillatore z
            F_term = coupling_strength * z * dt^2;
        end
        
        % Schema FDM standard
        u_next(i) = 2*u(i) - u_prev(i) + r2 * d2u + F_term;
    end
    
    % Aggiornamento array (Scambio buffer)
    u_prev = u;
    u = u_next;
    
    % --- C. Visualizzazione (ogni 20 step) ---
    if mod(n, 10) == 0
        % Subplot 1: La Corda
        subplot(3,1, [1 2]); 
        plot(x, u, 'b-', 'LineWidth', 2); hold on;
        % Disegno il "pistone" dell'oscillatore che spinge
        plot(x(idx_source), u(idx_source), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
        % Disegno la posizione "target" dell'oscillatore fantasma (per confronto)
        plot(x(idx_source), z, 'g*', 'MarkerSize', 5); 
        hold off;
        
        % Limiti fissi per vedere la crescita dell'ampiezza
        ylim([-3 3]); 
        title(sprintf('Corda Eccitata (t = %.3f s) - Tuning: %.1f', n*dt, tuning));
        grid on; ylabel('Ampiezza u(x)');
        legend('Corda', 'Punto di contatto', 'Posizione Oscillatore Z', 'Location', 'northeast');

        % Subplot 2: L'Oscillatore (Input)
        subplot(3,1,3);
        plot(n*dt, z, 'k.'); hold on;
        xlim([0 T_sim]); ylim([-1.5 1.5]);
        xlabel('Tempo (s)'); ylabel('Input z(t)');
        grid on;
        
        drawnow;
    end
end