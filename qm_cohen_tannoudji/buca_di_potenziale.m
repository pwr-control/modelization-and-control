clear; clc; close all;

%% 1. Parametri Fisici e di Simulazione
hbar = 1;           % Costante di Planck ridotta (unità normalizzate)
m = 1;              % Massa della particella (unità normalizzate)
L = 5;              % Semilarghezza del dominio di calcolo (-L, L)
N = 1000;           % Numero di punti della griglia (discretizzazione)
x = linspace(-L, L, N)'; % Vettore posizione
dx = x(2) - x(1);   % Passo della griglia

%% 2. Definizione del Potenziale V(x)
% Esempio: Buca di potenziale finita (Finite Square Well)
V0 = 50;            % Profondità della buca (o altezza delle barriere)
width = 2;          % Semilarghezza della buca
V = zeros(N, 1);    

% Definiamo la buca: 0 dentro, V0 fuori
V(abs(x) > width) = V0; 

% --- ALTERNATIVA: Oscillatore Armonico (decommentare per provare) ---
% k = 10; V = 0.5 * k * x.^2; 
% --------------------------------------------------------------------

%% 3. Costruzione della Matrice Hamiltoniana (H)
% L'Hamiltoniana è H = T + V
% T (Energia Cinetica) è approssimata con differenze finite (matrice tridiagonale)

% Coefficiente t = -hbar^2 / (2*m*dx^2)
t = -hbar^2 / (2*m*dx^2);

% Creazione matrice sparsa per efficienza
e = ones(N, 1);
% Matrice derivata seconda (Laplaciano discreto 1D) moltiplicata per coefficienti
T = spdiags([e -2*e e], -1:1, N, N); 
T = -t * T; % Nota: il segno meno in t e il segno nella matrice si combinano

% Matrice Energia Potenziale (diagonale)
U = spdiags(V, 0, N, N);

% Hamiltoniana Totale
H = T + U;

% Condizioni al contorno: La funzione d'onda deve essere 0 agli estremi (Infinite Wall box esterno)
% Questo è implicito se non colleghiamo il punto 1 con N.

%% 4. Soluzione dell'Equazione agli Autovalori
n_modes = 5; % Numero di stati energetici da calcolare
sigma = 'sm'; % 'sm' cerca gli autovalori più piccoli (Smallest Magnitude)

% eigs è ottimizzato per matrici sparse
[psi, E] = eigs(H, n_modes, sigma); 

% Ordiniamo gli autovalori dal più basso al più alto
eigenvalues = diag(E);
[eigenvalues, ind] = sort(eigenvalues);
psi = psi(:, ind);

%% 5. Normalizzazione e Plotting
figure('Name', 'Stati Energetici Particella', 'Color', 'w');
plot(x, V, 'k-', 'LineWidth', 2, 'DisplayName', 'Potenziale V(x)');
hold on;
grid on;

scale_factor = 0.5; % Fattore di scala per visualizzare le funzioni d'onda
colors = lines(n_modes);

for n = 1:n_modes
    % Normalizzazione della funzione d'onda (integrale |psi|^2 = 1)
    norm_const = sqrt(trapz(x, abs(psi(:,n)).^2));
    psi_norm = psi(:,n) / norm_const;
    
    % Shiftiamo la funzione d'onda per disegnarla sopra il suo livello energetico
    y_plot = scale_factor * psi_norm + eigenvalues(n);
    
    % Plot del livello energetico (linea tratteggiata orizzontale)
    yline(eigenvalues(n), '--', 'Color', [0.5 0.5 0.5], 'HandleVisibility', 'off');
    
    % Plot della funzione d'onda
    plot(x, y_plot, 'LineWidth', 1.5, 'Color', colors(n,:), ...
        'DisplayName', sprintf('n=%d (E=%.2f)', n, eigenvalues(n)));
end

title('Soluzione Eq. Schrödinger: Buca di Potenziale');
xlabel('Posizione x');
ylabel('Energia / Ampiezza \psi');
ylim([-10, V0 + 10]); % Zoom sull'area di interesse
legend('Location', 'best');
hold off;

fprintf('Livelli energetici calcolati:\n');
disp(eigenvalues);