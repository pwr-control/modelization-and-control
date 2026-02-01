clear; clc; close all;

%% 1. Definizione delle Costanti Fisiche per l'Aria
% Costanti empiriche standard per l'aria (Townsend breakdown)
% A e B dipendono dal gas. Per l'aria secca:
A = 15;        % [cm^-1 * Torr^-1] Costante di saturazione ionizzazione
B = 365;       % [V * cm^-1 * Torr^-1] Costante legata all'energia di ionizzazione
gamma = 0.01;  % Coefficiente di seconda ionizzazione (varia tra 0.01 e 0.1 per l'aria)

% Pressione atmosferica standard
P_atm_Torr = 760; % 1 atm in Torr (mmHg)

%% 2. Definizione del vettore Distanza
% Creiamo un vettore logaritmico per la distanza d
% Da 1 micrometro (1e-4 cm) a 10 cm
d_cm = logspace(-4, 1, 1000); 

% Calcolo del prodotto (p * d)
% p in Torr, d in cm
pd = P_atm_Torr .* d_cm; 

%% 3. Calcolo della Tensione di Rottura (Legge di Paschen)
% Formula: V = (B * pd) / (ln(A * pd) - ln(ln(1 + 1/gamma)))

% Termine costante al denominatore: C = ln(ln(1 + 1/gamma))
C = log(log(1 + 1/gamma));

% Calcolo Vb (Breakdown Voltage)
numerator = B .* pd;
denominator = log(A .* pd) - C;

% Filtriamo i valori non fisici (dove il denominatore è <= 0)
% Questo accade per pd estremamente piccoli dove la formula di Townsend non regge
valid_indices = denominator > 0;
pd = pd(valid_indices);
d_cm = d_cm(valid_indices);
Vb = numerator(valid_indices) ./ denominator(valid_indices);

%% 4. Trovare il Minimo di Paschen
[V_min, min_index] = min(Vb);
d_min_cm = d_cm(min_index);
pd_min = pd(min_index);

fprintf('--- Risultati Minimo di Paschen ---\n');
fprintf('Tensione minima di breakdown: %.2f V\n', V_min);
fprintf('Distanza critica (a 1 atm):   %.2f micrometri\n', d_min_cm * 10000);
fprintf('Prodotto (p*d) al minimo:     %.3f Torr*cm\n', pd_min);

%% 5. Plotting
figure('Color', 'w');

% Plot principale (Log-Log)
loglog(d_cm * 10000, Vb, 'LineWidth', 2, 'Color', 'b'); 
hold on;

% Evidenziare il minimo
plot(d_min_cm * 10000, V_min, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
text(d_min_cm * 10000, V_min * 1.3, ...
    sprintf('  Minimo: %.0f V @ %.1f \\mum', V_min, d_min_cm*10000), ...
    'FontSize', 10, 'FontWeight', 'bold');

% Linea di riferimento classica 3 kV/mm (per confronto sul lato destro)
% V = 3000 V/mm * d
d_lin = d_cm(d_cm > 0.01); % Solo per d > 100 um
V_lin = 30000 * d_lin;     % 30 kV/cm
loglog(d_lin * 10000, V_lin, '--k', 'LineWidth', 1.5);
text(1000, 2000, 'Approx. 3 kV/mm', 'FontSize', 9, 'Rotation', 25);

% Formattazione Grafico
grid on; grid minor;
xlabel('Distanza tra le piastre [\mum] (a 1 atm)', 'FontSize', 12);
ylabel('Tensione di Rottura [V]', 'FontSize', 12);
title('Curva di Paschen per l''Aria (1 atm)', 'FontSize', 14);

% Limiti assi per migliore visualizzazione
xlim([1, 100000]); % Da 1 um a 10 cm (100,000 um)
ylim([100, 100000]); 

legend('Curva di Paschen Teorica', 'Minimo di Paschen', 'Ref. Lineare (3 kV/mm)', ...
    'Location', 'northwest');

annotation('textbox', [0.15, 0.1, 0.3, 0.1], 'String', ...
    {'Nota: A sinistra del minimo', 'la rigidità aumenta drasticamente.'}, ...
    'FitBoxToText', 'on', 'BackgroundColor', 'white');