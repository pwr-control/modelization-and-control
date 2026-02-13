clear; close all; clc;
videoFile = VideoWriter('damped_string_excitation.mp4', 'MPEG-4');
videoFile.FrameRate = 30; % Imposta i fotogrammi al secondo
open(videoFile);
    
grey = [0.5 0.5 0.5];
greyL = [0.85 0.85 0.85];

%% 1. Parameters
L = 1.0;              
c = 40;               
T_sim = 1;

% --- damping
gamma = 1.25; 

modo = 8;

% omega res
f1 = modo * c / (2*L);       
omega_res = 2*pi*f1;  

% external excitation frequency
tuning = 1.0;         
omega_osc = omega_res * tuning;

%% 2. finite elements
Nx = 150;              
dx = L / (Nx - 1);
x = linspace(0, L, Nx);

CFL = 0.2;           
dt = CFL * dx / c;    
Nt = round(T_sim / dt); 

r2 = (c * dt / dx)^2; % Courant square number

%% u_t as (u_next - u_prev) / (2*dt)
alpha = gamma * dt / 2;
denom = 1 + alpha;   

%% 3. Init
u       = zeros(1, Nx); 
u_prev  = zeros(1, Nx); 
u_next  = zeros(1, Nx); 

z = 0.0;    % init position
vz = 1.0;   % init speed
az = 0;

% strength
coupling_strength = 0.25e6 / dx; 
idx_source = round(Nx/2/modo); 

%% 4. time loop

figure('Color', greyL, 'Position', [100 100 800 600]);

for n = 1:Nt
    
    % --- external disturbance ---

    az = -(omega_osc^2) * z; 
    vz = vz + az * dt;
    z  = z + vz * dt;
        
    % --- damped string ---
    for i = 2:Nx-1
        d2u = u(i+1) - 2*u(i) + u(i-1);
        
        F_term = 0;
        if i == idx_source
            F_term = coupling_strength * z * dt^2;
        end
               
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
        plot(x, u, 'color', grey, 'LineWidth', 2); hold on;
        plot(x(idx_source), u(idx_source), 'ko', 'MarkerFaceColor', grey);
        plot(x(idx_source), z, 'ko', 'MarkerSize', 5);
        hold off;
        
        % Fisso i limiti per vedere se l'ampiezza si stabilizza
        ylim([-Amax/modo Amax/modo]); 
        title(sprintf('Damped String (gamma=%.1f) - t=%.2fs', gamma, n*dt),'FontSize', 14);
        grid on; ylabel('u(x) /m','FontSize', 14); xlabel('x /m','FontSize', 14);
        
        subplot(3,1,3);
        plot(n*dt, u(idx_source), '.', 'Color',grey); hold on; % Traccio l'ampiezza del centro
        xlim([0 T_sim]); ylim([-Amax/modo Amax/modo]);
        xlabel('t /s','FontSize', 14); ylabel('u(t) /m','FontSize', 14);
        grid on;
        
        drawnow;
        frame = getframe(gcf); % Cattura la finestra corrente
        writeVideo(videoFile, frame); % Scrive il frame nel video
    end

end

% 3. Chiusura del file
close(videoFile);
fprintf('Video salvato con successo!\n');