clear; clc; close all;

%% 1. Define Physical Constants for Air
% Standard empirical constants for air (Townsend breakdown mechanism)
% A and B depend on the gas. For dry air:
A = 15;        % [cm^-1 * Torr^-1] Saturation ionization constant
B = 365;       % [V * cm^-1 * Torr^-1] Constant related to ionization energy
gamma = 0.01;  % Second ionization coefficient (approx. 0.01 to 0.1 for air)

% Standard Atmospheric Pressure
P_atm_Torr = 760; % 1 atm in Torr (mmHg)

%% 2. Define Distance Vector
% Create a logarithmic vector for distance 'd'
% From 1 micrometer (1e-4 cm) to 10 cm
d_cm = logspace(-4, 1, 1000); 

% Calculate the product (p * d)
% p in Torr, d in cm
pd = P_atm_Torr .* d_cm; 

%% 3. Calculate Breakdown Voltage (Paschen's Law)
% Formula: V = (B * pd) / (ln(A * pd) - ln(ln(1 + 1/gamma)))

% Constant term in the denominator: C = ln(ln(1 + 1/gamma))
C = log(log(1 + 1/gamma));

% Calculate Vb (Breakdown Voltage)
numerator = B .* pd;
denominator = log(A .* pd) - C;

% Filter out non-physical values (where denominator <= 0)
% This happens for extremely small pd where Townsend's formula breaks down
valid_indices = denominator > 0;
pd = pd(valid_indices);
d_cm = d_cm(valid_indices);
Vb = numerator(valid_indices) ./ denominator(valid_indices);

%% 4. Find Paschen Minimum
[V_min, min_index] = min(Vb);
d_min_cm = d_cm(min_index);
pd_min = pd(min_index);

fprintf('--- Paschen Minimum Results ---\n');
fprintf('Minimum Breakdown Voltage: %.2f V\n', V_min);
fprintf('Critical Distance (at 1 atm): %.2f micrometers\n', d_min_cm * 10000);
fprintf('Product (p*d) at minimum:     %.3f Torr*cm\n', pd_min);

%% 5. Plotting
figure('Color', 'w');

% Main Plot (Log-Log)
loglog(d_cm * 10000, Vb, 'LineWidth', 2, 'Color', 'k'); 
hold on;

gray = [0.5, 0.5 0.5];

% Highlight the Minimum
plot(d_min_cm * 10000, V_min, 'ko', 'MarkerFaceColor', gray, 'MarkerSize', 8);
text(d_min_cm * 10000, V_min * 0.75, ...
    sprintf('  Min: %.0f V @ %.1f \\mum', V_min, d_min_cm*10000), ...
    'FontSize', 12, 'FontWeight', 'bold');

% Reference Line: Classical 3 kV/mm (Linear Approximation)
% V = 3000 V/mm * d
d_lin = d_cm(d_cm > 0.01); % Only for d > 100 um
V_lin = 30000 * d_lin;     % 30 kV/cm
loglog(d_lin * 10000, V_lin, '--k', 'LineWidth', 1.5);
text(1000, 2000, 'Approx. 3 kV/mm', 'FontSize', 12, 'Rotation', 25);

% Graph Formatting
grid on; grid minor;
xlabel('Plate Distance [\mum] (at 1 atm)', 'FontSize', 12);
ylabel('Breakdown Voltage [V]', 'FontSize', 12);
title('Paschen Curve for Air (1 atm)', 'FontSize', 12);

% Axis limits for better visualization
xlim([1, 100000]); % From 1 um to 10 cm (100,000 um)
ylim([100, 100000]); 

legend('Theoretical Paschen Curve', 'Paschen Minimum', 'Linear Ref. (3 kV/mm)', ...
    'FontSize', 10, 'Location', 'northwest');

annotation('textbox', [0.08, 0.12, 0.3, 0.1], 'String', ...
    {'Note: Left of the minimum,', 'dielectric strength rises sharply.'}, ...
    'FitBoxToText', 'on', 'BackgroundColor', 'white');

print -depsc pascen_air.eps;