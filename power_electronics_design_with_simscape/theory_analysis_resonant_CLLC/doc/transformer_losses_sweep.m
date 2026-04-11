% ==========================================================================
% MF Transformer Loss Estimation — Core & Winding Losses vs Frequency
% ==========================================================================
% Specification:
%   - Rated power  : 250 kW
%   - Voltage ratio: 800 V / 800 V  (1:1)
%   - Frequency range: 1 kHz – 200 kHz (sweep)
%   - Core materials: N87 ferrite, 3C95 ferrite, Nanocrystalline (Vitroperm)
%
% Models used:
%   - Core losses  : Steinmetz equation  P_fe = k * f^alpha * Bpk^beta * V_core
%   - Winding losses: P_cu = FR * R_dc * I_rms^2
%     where FR = 1 + (pi^2/48) * (d/delta)^4  (simplified Dowell, round wire)
%     and skin depth delta = sqrt(rho / (pi * mu0 * f))
%
% References:
%   Steinmetz parameters from manufacturer datasheets (TDK, Ferroxcube, VAC)
%   Dowell model: Dowell, P.L., IEE Proc., 1966
% ==========================================================================

clear; close all; clc;

% --------------------------------------------------------------------------
% 1. TRANSFORMER SPECIFICATIONS
% --------------------------------------------------------------------------
P_rated   = 250e3;          % [W]   rated power
V_primary = 800;            % [V]   primary voltage (rms)
FP        = 1.0;            % [-]   power factor (assumed unity)
I_rms     = P_rated / (V_primary * FP);   % [A] primary rms current ~312 A

fprintf('=== MF Transformer — 250 kW, 800V/800V ===\n');
fprintf('Primary rms current: %.1f A\n\n', I_rms);

% --------------------------------------------------------------------------
% 2. FREQUENCY SWEEP
% --------------------------------------------------------------------------
f_vec = logspace(3, 5.3, 300);   % [Hz]  1 kHz to ~200 kHz

% --------------------------------------------------------------------------
% 3. CORE MATERIAL PARAMETERS  (Steinmetz: P_v = k * f^alpha * Bpk^beta)
%    Units: P_v in W/m^3, f in Hz, Bpk in T
%    B_peak optimum is chosen as a function of frequency (see section 5)
% --------------------------------------------------------------------------
materials(1).name  = 'N87 (TDK)';
materials(1).k     = 6.5e-3;       % Steinmetz coefficient
materials(1).alpha = 1.70;         % frequency exponent
materials(1).beta  = 2.70;         % flux density exponent
materials(1).Bsat  = 0.39;         % [T]  saturation flux density @ 25°C
materials(1).color = [0.20 0.45 0.75];   % plot color (blue)
materials(1).f_max = 100e3;        % [Hz] practical upper limit

materials(2).name  = '3C95 (Ferroxcube)';
materials(2).k     = 1.5e-3;
materials(2).alpha = 1.60;
materials(2).beta  = 2.50;
materials(2).Bsat  = 0.41;
materials(2).color = [0.85 0.33 0.10];   % plot color (orange)
materials(2).f_max = 500e3;

materials(3).name  = 'Nanocrystalline (Vitroperm 500F)';
materials(3).k     = 2.0e-4;       % much lower losses
materials(3).alpha = 1.50;
materials(3).beta  = 2.00;
materials(3).Bsat  = 1.20;         % [T]  much higher Bsat
materials(3).color = [0.13 0.63 0.30];   % plot color (green)
materials(3).f_max = 200e3;

n_mat = length(materials);

% --------------------------------------------------------------------------
% 4. CORE VOLUME ESTIMATION
%    V_core is estimated from Faraday's law and an assumed window utilization.
%    A simple area-product approach: Ap = V_core * A_window
%    For a given frequency, fewer turns are needed → smaller core.
%    Here we use a simplified scaling: V_core ∝ 1/f  (constant flux linkage)
%    Anchor point: V_core = 2.5e-3 m^3 at f = 10 kHz
% --------------------------------------------------------------------------
V_core_ref = 2.5e-3;    % [m^3]  reference core volume at f_ref
f_ref      = 10e3;      % [Hz]   reference frequency
V_core_vec = V_core_ref * (f_ref ./ f_vec);  % [m^3]  scaled core volume
V_core_vec = max(V_core_vec, 0.3e-3);        % [m^3]  practical lower bound

% --------------------------------------------------------------------------
% 5. OPTIMAL PEAK FLUX DENSITY vs FREQUENCY
%    The optimal B_peak minimises total losses (core + copper).
%    As a practical approximation, B_opt scales as f^(-0.45) (empirical fit).
%    Anchor: B_opt = 80 mT at 10 kHz for ferrite, 400 mT for nanocrystalline.
% --------------------------------------------------------------------------
B_opt_ferrite_ref  = 80e-3;   % [T]  at f_ref = 10 kHz
B_opt_nano_ref     = 400e-3;  % [T]  at f_ref = 10 kHz

% --------------------------------------------------------------------------
% 6. WINDING (COPPER) LOSS MODEL
%    Simplified Dowell model for round litz wire:
%      FR = 1 + (pi^2/48) * (d_strand / delta)^4
%    where d_strand is chosen as 1.5 * delta(f) to keep FR low.
%    R_dc is estimated from geometry; scaled with 1/V_core for winding length.
% --------------------------------------------------------------------------
rho_cu  = 1.72e-8;   % [Ohm·m]  copper resistivity at 80°C (derated)
mu0     = 4*pi*1e-7; % [H/m]

% Skin depth vs frequency
delta_vec = sqrt(rho_cu ./ (pi * mu0 * f_vec));   % [m]

% Litz strand diameter: chosen as 1.5 * delta  (practical design rule)
d_strand_vec = 1.5 * delta_vec;

% Dowell FR factor for litz (valid for d/delta < 2)
FR_vec = 1 + (pi^2/48) * (d_strand_vec ./ delta_vec).^4;

% DC resistance: scales with winding volume / conductor cross-section
% R_dc_ref at f_ref, assuming fill factor k_cu = 0.35
k_cu        = 0.35;   % copper fill factor in window
R_dc_ref    = 0.20e-3;  % [Ohm]  total winding DC resistance at f_ref
% Scale: fewer turns at higher f → shorter wire → lower R_dc
R_dc_vec    = R_dc_ref * (f_ref ./ f_vec);
R_dc_vec    = max(R_dc_vec, 0.05e-3);   % practical lower bound

% AC copper losses
P_cu_vec = FR_vec .* R_dc_vec .* I_rms^2;   % [W]

% --------------------------------------------------------------------------
% 7. CORE LOSS CALCULATION FOR EACH MATERIAL
% --------------------------------------------------------------------------
P_fe  = zeros(n_mat, length(f_vec));
B_opt = zeros(n_mat, length(f_vec));

for m = 1:n_mat
    if m < 3   % ferrite materials
        B_opt(m,:) = B_opt_ferrite_ref * (f_ref ./ f_vec).^0.45;
    else       % nanocrystalline
        B_opt(m,:) = B_opt_nano_ref    * (f_ref ./ f_vec).^0.45;
    end
    % Clamp B_peak to 60% of Bsat (safety margin)
    B_opt(m,:) = min(B_opt(m,:), 0.60 * materials(m).Bsat);

    % Steinmetz core loss density [W/m^3]
    P_v = materials(m).k .* f_vec.^materials(m).alpha .* B_opt(m,:).^materials(m).beta;

    % Total core losses
    P_fe(m,:) = P_v .* V_core_vec;   % [W]
end

% Total losses per material
P_total = P_fe + P_cu_vec;   % [W]  (copper losses are material-independent)

% Efficiency
eta = P_rated ./ (P_rated + P_total) * 100;   % [%]

% --------------------------------------------------------------------------
% 8. PLOTS
% --------------------------------------------------------------------------
f_kHz = f_vec / 1e3;   % convert to kHz for x-axis labels

% --- Figure 1: Core losses vs frequency for all materials -----------------
figure('Name','Core Losses vs Frequency','NumberTitle','off','Position',[100 100 820 500]);
hold on; grid on; box on;
for m = 1:n_mat
    semilogy(f_kHz, P_fe(m,:), 'Color', materials(m).color, ...
        'LineWidth', 2, 'DisplayName', materials(m).name);
end
xline(12,  '--k', '12 kHz',  'LabelVerticalAlignment','bottom','FontSize',9);
xline(100, '--k', '100 kHz', 'LabelVerticalAlignment','bottom','FontSize',9);
xlabel('Switching frequency  f  [kHz]', 'FontSize', 12);
ylabel('Core losses  P_{fe}  [W]',      'FontSize', 12);
title('Core losses vs frequency — 250 kW, 800 V/800 V transformer', 'FontSize', 13);
legend('Location','northwest','FontSize',10);
xlim([1 200]); ylim([10 1e5]);
set(gca,'XScale','log','FontSize',11);

% --- Figure 2: Copper losses and skin depth --------------------------------
figure('Name','Winding Losses & Skin Depth','NumberTitle','off','Position',[120 120 820 500]);

yyaxis left
semilogx(f_kHz, P_cu_vec, 'b-', 'LineWidth', 2);
xlabel('Switching frequency  f  [kHz]', 'FontSize', 12);
ylabel('Copper losses  P_{cu}  [W]',    'FontSize', 12);
ylim([0 max(P_cu_vec)*1.3]);

yyaxis right
semilogx(f_kHz, delta_vec*1e3, 'r--', 'LineWidth', 1.8);
ylabel('Skin depth  \delta  [mm]', 'FontSize', 12);
ylim([0 max(delta_vec*1e3)*1.3]);

xline(12,  '--k', '12 kHz',  'LabelVerticalAlignment','bottom','FontSize',9);
xline(100, '--k', '100 kHz', 'LabelVerticalAlignment','bottom','FontSize',9);
title('Winding losses and skin depth vs frequency', 'FontSize', 13);
legend({'P_{cu} (copper losses)','Skin depth \delta'}, ...
    'Location','northeast','FontSize',10);
grid on; box on; set(gca,'FontSize',11);

% --- Figure 3: Total losses (core + copper) per material ------------------
figure('Name','Total Losses vs Frequency','NumberTitle','off','Position',[140 140 820 500]);
hold on; grid on; box on;
for m = 1:n_mat
    semilogy(f_kHz, P_total(m,:), 'Color', materials(m).color, ...
        'LineWidth', 2, 'DisplayName', materials(m).name);
end
semilogy(f_kHz, P_cu_vec, 'k:', 'LineWidth', 1.5, 'DisplayName', 'P_{cu} only (reference)');
xline(12,  '--k', '12 kHz',  'LabelVerticalAlignment','bottom','FontSize',9);
xline(100, '--k', '100 kHz', 'LabelVerticalAlignment','bottom','FontSize',9);
xlabel('Switching frequency  f  [kHz]', 'FontSize', 12);
ylabel('Total losses  P_{fe} + P_{cu}  [W]', 'FontSize', 12);
title('Total transformer losses vs frequency', 'FontSize', 13);
legend('Location','northwest','FontSize',10);
xlim([1 200]); ylim([50 1e5]);
set(gca,'XScale','log','FontSize',11);

% --- Figure 4: Efficiency vs frequency ------------------------------------
figure('Name','Efficiency vs Frequency','NumberTitle','off','Position',[160 160 820 500]);
hold on; grid on; box on;
for m = 1:n_mat
    eta_m = P_rated ./ (P_rated + P_total(m,:)) * 100;
    semilogx(f_kHz, eta_m, 'Color', materials(m).color, ...
        'LineWidth', 2, 'DisplayName', materials(m).name);
end
xline(12,  '--k', '12 kHz',  'LabelVerticalAlignment','bottom','FontSize',9);
xline(100, '--k', '100 kHz', 'LabelVerticalAlignment','bottom','FontSize',9);
xlabel('Switching frequency  f  [kHz]', 'FontSize', 12);
ylabel('Efficiency  \eta  [%]',          'FontSize', 12);
title('Transformer efficiency vs frequency', 'FontSize', 13);
legend('Location','southwest','FontSize',10);
xlim([1 200]); ylim([98.5 100]);
set(gca,'XScale','log','FontSize',11);

% --- Figure 5: Optimal B_peak vs frequency --------------------------------
figure('Name','Optimal Bpeak vs Frequency','NumberTitle','off','Position',[180 180 820 500]);
hold on; grid on; box on;
for m = 1:n_mat
    semilogx(f_kHz, B_opt(m,:)*1e3, 'Color', materials(m).color, ...
        'LineWidth', 2, 'DisplayName', [materials(m).name ...
        sprintf('  (B_{sat} = %.0f mT)', materials(m).Bsat*1e3)]);
end
xline(12,  '--k', '12 kHz',  'LabelVerticalAlignment','bottom','FontSize',9);
xline(100, '--k', '100 kHz', 'LabelVerticalAlignment','bottom','FontSize',9);
xlabel('Switching frequency  f  [kHz]',   'FontSize', 12);
ylabel('Optimal peak flux density  [mT]', 'FontSize', 12);
title('Optimal B_{peak} vs frequency (60% B_{sat} clamp)', 'FontSize', 13);
legend('Location','northeast','FontSize',10);
xlim([1 200]);
set(gca,'XScale','log','FontSize',11);

% --------------------------------------------------------------------------
% 9. SUMMARY TABLE AT KEY FREQUENCIES
% --------------------------------------------------------------------------
f_key = [12e3, 20e3, 50e3, 100e3];
fprintf('\n%-35s  %8s  %8s  %8s  %8s\n', 'Loss component / material', ...
    '12 kHz', '20 kHz', '50 kHz', '100 kHz');
fprintf('%s\n', repmat('-',1,75));

% Copper losses (material-independent)
idx = arrayfun(@(fk) find(f_vec >= fk, 1), f_key);
fprintf('%-35s', 'Copper losses P_cu [W]');
fprintf('  %8.0f', P_cu_vec(idx));
fprintf('\n');

% Core + total losses per material
for m = 1:n_mat
    fprintf('%-35s', ['Core  P_fe [W]  — ' materials(m).name]);
    fprintf('  %8.0f', P_fe(m, idx));
    fprintf('\n');
    fprintf('%-35s', ['Total P_tot [W] — ' materials(m).name]);
    fprintf('  %8.0f', P_total(m, idx));
    fprintf('\n');
    eta_row = P_rated ./ (P_rated + P_total(m, idx)) * 100;
    fprintf('%-35s', ['Efficiency [%%]  — ' materials(m).name]);
    fprintf('  %8.3f', eta_row);
    fprintf('\n');
    fprintf('%s\n', repmat('·',1,75));
end

fprintf('\nNote: copper losses assume litz wire with d_strand = 1.5 * delta(f).\n');
fprintf('Core volume scaled as V_core ∝ 1/f  (anchor: %.1f L at %g kHz).\n', ...
    V_core_ref*1e3, f_ref/1e3);
fprintf('Steinmetz model (iGSE recommended for non-sinusoidal DAB currents).\n');
