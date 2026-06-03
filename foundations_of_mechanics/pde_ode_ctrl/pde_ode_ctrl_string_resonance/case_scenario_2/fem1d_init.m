%% FEM1D_INIT.M
% Inizializzazione co-simulazione FEM 1D barra vibrante + LQR
% Barra incastrata-incastrata, 50 elementi, disturbo al 2° modo, controllo LQR
%
% Eseguire questo script PRIMA di aprire/simulare fem1d_simulink.slx
% (oppure impostarlo come InitFcn callback del modello)

clear; clc;

%% --- Parametri fisici barra ---
L   = 1.0;      % lunghezza [m]
E   = 210e9;    % modulo di Young [Pa]  (acciaio)
rho = 7800;     % densità [kg/m³]
A   = 1e-4;     % sezione trasversale [m²]
xi  = 0.005;    % smorzamento modale (0.5% - molto basso, risonanza evidente)

%% --- Mesh FEM ---
n_el  = 50;             % numero di elementi
n_dof = n_el - 1;       % DOF interni (BC incastro-incastro: nodi 1 e n_el+1 vincolati)
le    = L / n_el;       % lunghezza elemento

%% --- Assemblaggio matrici globali (vibrazione assiale) ---
% Elemento lineare a 2 nodi:
%   k_e = (EA/le)*[ 1 -1; -1  1]
%   m_e = (rho*A*le/6)*[2 1; 1 2]

n_tot = n_el + 1;       % nodi totali
K_full = zeros(n_tot);
M_full = zeros(n_tot);

EA = E * A;
rhoAle = rho * A * le;

k_e = (EA/le)  * [ 1 -1; -1  1];
m_e = (rhoAle/6) * [ 2  1;  1  2];

for e = 1:n_el
    idx = [e, e+1];
    K_full(idx,idx) = K_full(idx,idx) + k_e;
    M_full(idx,idx) = M_full(idx,idx) + m_e;
end

% Applicazione BC: elimina nodo 1 e nodo n_tot (incastri)
free_dof = 2:n_tot-1;   % = nodi interni = n_dof DOF
K = K_full(free_dof, free_dof);
M = M_full(free_dof, free_dof);

%% --- Analisi modale ---
[Phi, Omega2] = eig(K, M);
omega2 = diag(Omega2);
[omega2, idx_sort] = sort(omega2);
Phi = Phi(:, idx_sort);

omega_n = sqrt(omega2);         % pulsazioni naturali [rad/s]
f_n     = omega_n / (2*pi);    % frequenze naturali [Hz]

fprintf('=== Frequenze naturali (prime 5) ===\n');
for i=1:5
    fprintf('  Modo %d:  %.4f Hz  (%.2f rad/s)\n', i, f_n(i), omega_n(i));
end

% Normalizzazione rispetto alla massa (Phi' * M * Phi = I)
for i = 1:n_dof
    mn = Phi(:,i)' * M * Phi(:,i);
    Phi(:,i) = Phi(:,i) / sqrt(mn);
end

%% --- Matrice di smorzamento (Rayleigh proporzionale) ---
% C = alpha*M + beta*K  tale che xi_i = (alpha/(2*omega_i) + beta*omega_i/2)
% Usiamo xi costante per i modi 1 e 2
om1 = omega_n(1); om2 = omega_n(2);
A_ray = 0.5 * [1/om1, om1; 1/om2, om2];
ab    = A_ray \ [xi; xi];
alpha_r = ab(1);
beta_r  = ab(2);
C = alpha_r * M + beta_r * K;

fprintf('\nRayleigh: alpha=%.6f, beta=%.6f\n', alpha_r, beta_r);

%% --- Punti di applicazione forze ---
% Disturbo: anti-nodo del 2° modo → nodo a L/4 (indice interno = n_el/4)
% Controllo: nodo centrale = n_el/2
node_dist = round(n_el/4);      % nodo interno per disturbo
node_ctrl = round(n_el/2);      % nodo interno per controllo (L/2)

fprintf('\nNodo disturbo : %d (x = %.3f m)\n', node_dist, node_dist*le);
fprintf('Nodo controllo: %d (x = %.3f m)\n', node_ctrl, node_ctrl*le);

% Vettori di ingresso (forza → accelerazione)
B_dist = zeros(n_dof,1);  B_dist(node_dist) = 1;
B_ctrl = zeros(n_dof,1);  B_ctrl(node_ctrl) = 1;

% Nodo di osservazione: stesso del controllo
node_obs = node_ctrl;

%% --- Frequenza di eccitazione (2° modo) ---
omega_exc = omega_n(2);     % disturbo esattamente al 2° modo
f_exc     = f_n(2);
F_amp     = 1.0;            % ampiezza forza disturbo [N]

fprintf('\nEccitazione: f = %.4f Hz\n', f_exc);

%% --- Spazio di stato (sistema completo, 2*n_dof stati) ---
% x = [u; u_dot],   x_dot = [u_dot; M\(f - K*u - C*u_dot)]
%
% A_ss = [0,  I;  -M\K, -M\C]
% B_ss = [0; M\B_ctrl]    (ingresso di controllo)
% Bd_ss= [0; M\B_dist]    (ingresso disturbo)

In = eye(n_dof);
Minv = M \ In;

A_ss = [ zeros(n_dof),  In;
        -Minv*K,        -Minv*C];

B_ss  = [zeros(n_dof,1); Minv*B_ctrl];
Bd_ss = [zeros(n_dof,1); Minv*B_dist];

% Output: spostamento nodo osservato
C_ss = zeros(1, 2*n_dof);
C_ss(1, node_obs) = 1;

fprintf('\nDimensione sistema stato: %d x %d\n', size(A_ss));

%% --- Progetto LQR ---
% Pesatura: penalizza spostamento nodo controllato + energia di controllo
% Q pesa tutti i DOF di spostamento; R pesa la forza di controllo

% Q: peso unità su tutti gli spostamenti + velocità scalate
q_u   = 1e6;    % peso spostamenti
q_v   = 1e2;    % peso velocità
Q_lqr = blkdiag(q_u * In, q_v * In);

R_lqr = 1.0;    % costo forza di controllo [1/N²]

% Soluzione Riccati
[K_lqr, S_lqr, eig_cl] = lqr(A_ss, B_ss, Q_lqr, R_lqr);

fprintf('\n=== LQR progettato ===\n');
fprintf('Norma guadagno K_lqr: %.4f\n', norm(K_lqr));
fprintf('Autovalori CL (primi 4, parte reale):\n');
eig_cl_sorted = sort(eig_cl, 'ComparisonMethod','real');
for i=1:min(4,length(eig_cl_sorted))
    fprintf('  lambda_%d = %.4f + %.4fi\n', i, ...
        real(eig_cl_sorted(i)), imag(eig_cl_sorted(i)));
end

%% --- Parametri simulazione ---
T_sim    = 5.0;         % durata [s]
dt       = 1e-5;        % passo fisso Simulink [s]  (>> 1/f_max)
t_ctrl_on = 1.0;        % il controllo si attiva dopo 1 s (vedi switch in Simulink)

fprintf('\nSimulazione: T=%.1f s, dt=%.1e s\n', T_sim, dt);
fprintf('Controllo attivo da t=%.1f s\n', t_ctrl_on);

%% --- Salva workspace per Simulink ---
save('fem1d_workspace.mat', ...
    'M','K','C','n_dof','Phi','omega_n','f_n', ...
    'A_ss','B_ss','Bd_ss','C_ss', ...
    'K_lqr','Q_lqr','R_lqr', ...
    'B_dist','B_ctrl','node_dist','node_ctrl','node_obs', ...
    'omega_exc','f_exc','F_amp', ...
    'T_sim','dt','t_ctrl_on', ...
    'L','le','n_el','xi','alpha_r','beta_r');

fprintf('\nWorkspace salvato in fem1d_workspace.mat\n');

%% --- Plot modi di vibrazione ---
x_nodes = (1:n_dof) * le;  % coordinate nodi interni

figure('Name','Modi di vibrazione','NumberTitle','off');
for i=1:4
    subplot(2,2,i);
    plot(x_nodes, Phi(:,i), 'b-o', 'MarkerSize',3,'LineWidth',1.5);
    hold on;
    xline(node_dist*le,'r--','Label','Disturbo');
    xline(node_ctrl*le,'g--','Label','Controllo');
    grid on;
    xlabel('x [m]'); ylabel('\phi_i [m^{-1/2}]');
    title(sprintf('Modo %d:  f = %.2f Hz', i, f_n(i)));
end
sgtitle('Forme modali - Barra incastrata-incastrata');

%% --- Verifica posizionamento: componente modale del disturbo sul modo 2 ---
phi2_dist = Phi(node_dist, 2);
phi2_ctrl = Phi(node_ctrl, 2);
fprintf('\nComponente modo 2 al nodo disturbo : %.6f\n', phi2_dist);
fprintf('Componente modo 2 al nodo controllo: %.6f\n', phi2_ctrl);
if abs(phi2_ctrl) < 1e-6
    warning('Il nodo di controllo è un nodo del modo 2! Scegliere posizione diversa.');
end

fprintf('\n=== Init completato. Aprire fem1d_simulink.slx ===\n');
