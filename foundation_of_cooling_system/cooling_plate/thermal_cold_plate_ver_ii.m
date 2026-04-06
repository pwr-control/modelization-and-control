%% =========================================================================
%  thermal_cold_plate.m
%  Thermal resistance calculation: Al plate + X pin-fin cold plate
%
%  System:
%    - 40 x 160 x 5 mm aluminium plate (heat source on top)
%    - X pin-fin cold plate below (fins: 10 mm deep, 2 mm wide,
%      2 mm Al walls, pitch 4 mm)
%    - Coolant: 30% ethylene-glycol / water mixture
%
%  Two approaches:
%    1) Manufacturer data (X) + power-law flow scaling  [recommended]
%    2) Analytical fin model (Gnielinski + fin efficiency)   [cross-check]
%
%  Author : Shannon
%  Date   : 2026-04
%% =========================================================================

clear; clc; close all;

%% -------------------------------------------------------------------------
%  1. GEOMETRY
%% -------------------------------------------------------------------------
L   = 0.160;    % [m]  plate length (along fins / flow direction)
W   = 0.040;    % [m]  plate width  (across fins)
t   = 0.003;    % [m]  plate thickness (heat-flow axis)
A   = L * W;    % [m^2] base area

% Pin-fin geometry
H_f = 0.005;    % [m]  fin depth
w_f = 0.003;    % [m]  fin wall thickness
w_c = 0.003;    % [m]  channel width
p   = w_f + w_c; % [m]  pitch
N_f = round(W / p); % number of fins

fprintf('=== GEOMETRY ===\n');
fprintf('Base area A     = %.1f cm^2\n', A*1e4);
fprintf('Number of fins  = %d\n', N_f);

% Wetted area breakdown
A_fin_total = N_f * 2 * H_f * L;    % [m^2]  both sides of all fins
A_base      = N_f * w_c * L;        % [m^2]  channel bottoms (base exposed)
A_plain     = A;                     % [m^2]  plain base (no fins)
area_factor = (A_fin_total + A_base) / A_plain;

fprintf('Fin area (both sides) = %.1f cm^2\n', A_fin_total*1e4);
fprintf('Base (channel bottom) = %.1f cm^2\n', A_base*1e4);
fprintf('Area enhancement      = %.1fx vs plain base\n', area_factor);

%% -------------------------------------------------------------------------
%  2. MATERIAL PROPERTIES
%% -------------------------------------------------------------------------
% Aluminium (6xxx series)
k_Al   = 160;    % [W/(m·K)]
rho_Al = 2700;   % [kg/m^3]
cp_Al  = 900;    % [J/(kg·K)]

% EG 30% vol. coolant at ~50°C
rho_f = 1040;    % [kg/m^3]
cp_f  = 3800;    % [J/(kg·K)]
mu_f  = 8.2e-4;  % [Pa·s]   dynamic viscosity
k_f   = 0.45;    % [W/(m·K)]
Pr    = mu_f * cp_f / k_f;

fprintf('\n=== FLUID PROPERTIES (EG30%% @ ~50°C) ===\n');
fprintf('Density          rho = %.0f kg/m^3\n', rho_f);
fprintf('Specific heat    cp  = %.0f J/(kg·K)\n', cp_f);
fprintf('Dyn. viscosity   mu  = %.2f mPa·s\n', mu_f*1e3);
fprintf('Conductivity     k_f = %.2f W/(m·K)\n', k_f);
fprintf('Prandtl number   Pr  = %.2f\n', Pr);

%% -------------------------------------------------------------------------
%  3. CONDUCTION RESISTANCE THROUGH THE ALUMINIUM PLATE
%% -------------------------------------------------------------------------
R_cond = t / (k_Al * A);   % [K/W]

fprintf('\n=== CONDUCTION RESISTANCE (Al plate) ===\n');
fprintf('R_cond = t / (k_Al * A) = %.2f mK/W\n', R_cond*1e3);

%% -------------------------------------------------------------------------
%  4. MANUFACTURER DATA — X COLD PLATE
%% -------------------------------------------------------------------------
R_cp_ref  = 18e-3;   % [K/W]  X reference value
Q_ref     = 18.0;    % [l/min] at which R_cp_ref is specified

% Power-law scaling: R_cp(Q) = R_cp_ref * (Q_ref / Q)^n
% Exponent n ~ 0.65–0.75 for turbulent pin-fin flow
n_exp     = 0.70;    % midpoint exponent (recommended)
n_lo      = 0.65;    % conservative bound
n_hi      = 0.75;    % optimistic bound

Q_user    = 24.0;    % [l/min] actual operating flow rate

R_cp_24     = R_cp_ref * (Q_ref / Q_user)^n_exp;
R_cp_24_lo  = R_cp_ref * (Q_ref / Q_user)^n_lo;
R_cp_24_hi  = R_cp_ref * (Q_ref / Q_user)^n_hi;

R_tot_18 = R_cond + R_cp_ref;
R_tot_24 = R_cond + R_cp_24;

fprintf('\n=== X COLD PLATE (manufacturer data + scaling) ===\n');
fprintf('R_cp  @ %.0f l/min (reference) = %.2f mK/W\n', Q_ref, R_cp_ref*1e3);
fprintf('R_cp  @ %.0f l/min  (n=%.2f)   = %.2f mK/W\n', Q_user, n_exp, R_cp_24*1e3);
fprintf('  conservative (n=%.2f)         = %.2f mK/W\n', n_lo, R_cp_24_lo*1e3);
fprintf('  optimistic   (n=%.2f)         = %.2f mK/W\n', n_hi, R_cp_24_hi*1e3);
fprintf('\nTotal R_th (Al + cold plate):\n');
fprintf('  @ %.0f l/min = %.2f mK/W\n', Q_ref,  R_tot_18*1e3);
fprintf('  @ %.0f l/min = %.2f mK/W\n', Q_user, R_tot_24*1e3);

%% -------------------------------------------------------------------------
%  5. ANALYTICAL FIN MODEL (cross-check)
%% -------------------------------------------------------------------------
% Hydraulic diameter of one rectangular micro-channel
A_chan = w_c * H_f;                          % [m^2] channel cross-section
D_h   = 4 * A_chan / (2*(w_c + H_f));        % [m]   hydraulic diameter

fprintf('\n=== ANALYTICAL FIN MODEL (cross-check) ===\n');
fprintf('Channel cross-section  = %.2f mm^2\n', A_chan*1e6);
fprintf('Hydraulic diameter D_h = %.2f mm\n',   D_h*1e3);

% Loop over two flow rates
Q_list = [18, 24];   % [l/min]

for qi = 1:length(Q_list)
    Q_lpm  = Q_list(qi);
    Q_m3s  = Q_lpm / 1000 / 60;              % [m^3/s]
    v_mean = Q_m3s / (N_f * A_chan);          % [m/s] mean velocity per channel
    Re     = rho_f * v_mean * D_h / mu_f;    % Reynolds number

    % Nusselt number — Gnielinski correlation (3000 < Re < 5e6, 0.5 < Pr < 2000)
    if Re > 3000
        f  = (0.790*log(Re) - 1.64)^(-2);   % Petukhov friction factor
        Nu = (f/8)*(Re - 1000)*Pr / ...
             (1 + 12.7*sqrt(f/8)*(Pr^(2/3) - 1));
    else
        % Fully developed laminar flow in rectangular duct (aspect ratio ~1:5)
        Nu = 7.54;
        f  = 96 / Re;
    end

    h_conv = Nu * k_f / D_h;    % [W/(m^2·K)]  convective coefficient

    % Fin efficiency (straight fin, uniform cross-section, adiabatic tip)
    %   m = sqrt(2*h / (k_Al * w_f))
    %   eta_f = tanh(m * H_f) / (m * H_f)
    m_fin   = sqrt(2 * h_conv / (k_Al * w_f));
    eta_fin = tanh(m_fin * H_f) / (m_fin * H_f);

    % Effective heat transfer area
    A_eff = eta_fin * A_fin_total + A_base;   % [m^2]

    % Convective resistance over effective area
    R_conv_fin = 1 / (h_conv * A_eff);        % [K/W]

    % Total resistance (Al plate + fins)
    R_tot_analytical = R_cond + R_conv_fin;

    fprintf('\n  Q = %.0f l/min:\n', Q_lpm);
    fprintf('    v_mean = %.3f m/s\n',    v_mean);
    fprintf('    Re     = %.0f  (%s)\n',  Re, regime_label(Re));
    fprintf('    Nu     = %.1f\n',        Nu);
    fprintf('    h      = %.0f W/(m^2·K)\n', h_conv);
    fprintf('    m_fin  = %.1f 1/m\n',    m_fin);
    fprintf('    eta_f  = %.3f\n',        eta_fin);
    fprintf('    A_eff  = %.1f cm^2\n',   A_eff*1e4);
    fprintf('    R_conv_fin      = %.2f mK/W\n', R_conv_fin*1e3);
    fprintf('    R_tot_analytic  = %.2f mK/W\n', R_tot_analytical*1e3);
end

%% -------------------------------------------------------------------------
%  6. THERMAL CAPACITANCE AND TIME CONSTANT
%% -------------------------------------------------------------------------
V_Al   = L * W * t;                      % [m^3]  aluminium volume
C_th   = rho_Al * cp_Al * V_Al;          % [J/K]  thermal capacitance
tau    = R_cond * C_th;                  % [s]    time constant (cond. path)

fprintf('\n=== DYNAMIC MODEL (Al plate only) ===\n');
fprintf('C_th = rho*cp*V = %.2f J/K\n', C_th);
fprintf('tau  = R_cond * C_th = %.3f s\n', tau);

%% -------------------------------------------------------------------------
%  7. PLOTS
%% -------------------------------------------------------------------------

% --- 7a. R_th vs flow rate (manufacturer scaling) -------------------------
Q_range  = linspace(10, 35, 200);   % [l/min]
R_cp_ref = R_tot_analytical;
R_cp_Q   = R_cp_ref * (Q_ref ./ Q_range).^n_exp;
R_tot_Q  = R_cond + R_cp_Q;

figure('Name','R_th vs Flow Rate','NumberTitle','off');
plot(Q_range, R_tot_Q*1e3, 'b-', 'LineWidth', 2); hold on;
plot([Q_ref, Q_user], [R_tot_18, R_tot_24]*1e3, 'ro', ...
     'MarkerSize', 8, 'MarkerFaceColor', 'r');
text(Q_ref  + 0.5, R_tot_18*1e3 + 0.8, ...
     sprintf('%.1f mK/W', R_tot_18*1e3), 'FontSize', 9);
text(Q_user + 0.5, R_tot_24*1e3 + 0.8, ...
     sprintf('%.1f mK/W', R_tot_24*1e3), 'FontSize', 9);
grid on; grid minor;
xlabel('Flow rate [l/min]', 'FontSize', 11);
ylabel('R_{th,tot} [mK/W]',  'FontSize', 11);
title('Total Thermal Resistance vs Flow Rate', 'FontSize', 12);
legend('X data + Al plate  (n = 0.70)', 'Design points', ...
       'Location', 'northeast');
xlim([10 35]); ylim([5 35]);

return
% --- 7b. Contribution breakdown bar chart ---------------------------------
figure('Name','Resistance Breakdown','NumberTitle','off');
cats   = {'18 l/min', '24 l/min'};
R_bar  = [R_cond*1e3, R_cp_ref*1e3;
          R_cond*1e3, R_cp_24*1e3];
b = bar(R_bar, 'stacked');
b(1).FaceColor = [0.85 0.60 0.20];   % orange: Al conduction
b(2).FaceColor = [0.20 0.50 0.80];   % blue:   cold plate
set(gca, 'XTickLabel', cats, 'FontSize', 11);
ylabel('R_{th} [mK/W]', 'FontSize', 11);
title('Thermal Resistance Breakdown', 'FontSize', 12);
legend('Al plate conduction', 'X cold plate', 'Location', 'northeast');
grid on; ylim([0 30]);

% Annotate totals
totals = sum(R_bar, 2);
for k = 1:2
    text(k, totals(k) + 0.4, sprintf('%.1f', totals(k)), ...
         'HorizontalAlignment', 'center', 'FontSize', 10, 'FontWeight', 'bold');
end

% --- 7c. Fin efficiency vs h_conv -----------------------------------------
h_vec   = linspace(1000, 35000, 300);
m_vec   = sqrt(2 * h_vec / (k_Al * w_f));
eta_vec = tanh(m_vec * H_f) ./ (m_vec * H_f);

figure('Name','Fin Efficiency','NumberTitle','off');
plot(h_vec/1e3, eta_vec*100, 'k-', 'LineWidth', 2); hold on;
% Mark the two operating points
for qi = 1:length(Q_list)
    Q_m3s  = Q_list(qi) / 1000 / 60;
    v_tmp  = Q_m3s / (N_f * A_chan);
    Re_tmp = rho_f * v_tmp * D_h / mu_f;
    f_tmp  = (0.790*log(Re_tmp) - 1.64)^(-2);
    Nu_tmp = (f_tmp/8)*(Re_tmp-1000)*Pr / ...
             (1 + 12.7*sqrt(f_tmp/8)*(Pr^(2/3)-1));
    h_tmp  = Nu_tmp * k_f / D_h;
    m_tmp  = sqrt(2*h_tmp / (k_Al * w_f));
    e_tmp  = tanh(m_tmp * H_f) / (m_tmp * H_f);
    plot(h_tmp/1e3, e_tmp*100, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    text(h_tmp/1e3 + 0.2, e_tmp*100 + 1.5, ...
         sprintf('%.0f l/min: \\eta_f=%.2f', Q_list(qi), e_tmp), 'FontSize', 9);
end
grid on; grid minor;
xlabel('Convective coefficient h [kW/(m^2·K)]', 'FontSize', 11);
ylabel('Fin efficiency \eta_f [%]',              'FontSize', 11);
title('Fin Efficiency vs Convective Coefficient', 'FontSize', 12);
xlim([0 35]); ylim([0 100]);

fprintf('\nDone. Three figures generated.\n');

%% =========================================================================
%  LOCAL HELPER FUNCTION
%% =========================================================================
function s = regime_label(Re)
    if Re > 10000
        s = 'turbulent';
    elseif Re > 4000
        s = 'transitional';
    else
        s = 'laminar';
    end
end
