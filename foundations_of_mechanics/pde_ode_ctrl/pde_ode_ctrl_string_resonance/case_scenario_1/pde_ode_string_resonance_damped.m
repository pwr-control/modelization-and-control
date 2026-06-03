clear; close all; clc;

%% 1. Parametri Fisici
L = 1.0;              
c = 40;               
T_sim = 2.0;

% --- Parametri di Smorzamento ---
% gamma = 0   -> Nessun attrito (crescita infinita in risonanza)
% gamma = 1.0 -> Smorzamento leggero (aria)
% gamma = 5.0 -> Smorzamento forte (olio/miele)
gamma = 1.25; 

modo = 8;

% Calcolo Frequenza Propria
f1 = modo * c / (2*L);       
omega_res = 2*pi*f1;  

% Tuning Oscillatore
tuning = 1.0;         
omega_osc = omega_res * tuning;

%% 2. Discretizzazione
Nx = 150;              
dx = L / (Nx - 1);
x = linspace(0, L, Nx);

CFL = 0.2;           
dt = CFL * dx / c;    
Nt = round(T_sim / dt); 

r2 = (c * dt / dx)^2; % Numero di Courant al quadrato

% --- Coefficienti per lo Smorzamento ---
% Derivazione: u_tt + gamma*u_t = ...
% Discretizzando u_t come (u_next - u_prev) / (2*dt)
alpha = gamma * dt / 2;
denom = 1 + alpha;      % Fattore di divisione per u_next

%% 3. Inizializzazione
u       = zeros(1, Nx); 
u_prev  = zeros(1, Nx); 
u_next  = zeros(1, Nx); 

z = 0.0;    % Partiamo da zero
vz = 1.0;   % Diamo una velocità iniziale all'oscillatore
az = 0;

% Forza
coupling_strength = 0.25e6 / dx; 
idx_source = round(Nx/2/modo); 

%% 4. Loop Temporale
figure('Color', 'w', 'Position', [100 100 800 600]);

for n = 1:Nt
    
    % --- A. Oscillatore (Sorgente) ---
    % Nota: anche l'oscillatore potrebbe avere smorzamento, 
    % ma qui lo lasciamo ideale per fornire energia costante.
    az = -(omega_osc^2) * z; 
    vz = vz + az * dt;
    z  = z + vz * dt;
        
    % --- B. PDE (Corda Smorzata) ---
    for i = 2:Nx-1
        d2u = u(i+1) - 2*u(i) + u(i-1);
        
        F_term = 0;
        if i == idx_source
            F_term = coupling_strength * z * dt^2;
        end
        
        % --- FORMULA DI AGGIORNAMENTO CON SMORZAMENTO ---
        % Senza smorzamento era: u_next = 2u - u_prev + ...
        % Con smorzamento diventa:
        
        term_inerziale = 2*u(i) - u_prev(i)*(1 - alpha);
        term_elastico  = r2 * d2u;
        
        u_next(i) = (term_inerziale + term_elastico + F_term) / denom;
    end
    
    % Aggiornamento array
    u_prev = u;
    u = u_next;
    
    Amax = 6;
    % --- C. Visualizzazione ---
    if mod(n, 10) == 0 % Disegno meno frequente per velocità
        subplot(3,1, [1 2]); 
        plot(x, u, 'b-', 'LineWidth', 2); hold on;
        plot(x(idx_source), u(idx_source), 'ro', 'MarkerFaceColor', 'r');
        plot(x(idx_source), z, 'g*', 'MarkerSize', 5);
        hold off;
        
        % Fisso i limiti per vedere se l'ampiezza si stabilizza
        ylim([-Amax/modo Amax/modo]); 
        title(sprintf('Corda Smorzata (gamma=%.1f) - t=%.2fs', gamma, n*dt));
        grid on; ylabel('u(x)');
        
        subplot(3,1,3);
        plot(n*dt, u(idx_source), 'm.'); hold on; % Traccio l'ampiezza del centro
        xlim([0 T_sim]); ylim([-Amax/modo Amax/modo]);
        xlabel('Tempo (s)'); ylabel('Ampiezza al Centro');
        grid on;
        
        drawnow;
    end
end