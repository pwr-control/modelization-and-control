%% FEM1D_POSTPROCESS.M
% Post-processing risultati simulazione FEM 1D + LQR
% Eseguire DOPO la simulazione di fem1d_simulink.slx

%% Carica workspace se necessario
if ~exist('n_dof','var')
    load('fem1d_workspace.mat');
end

%% Verifica dati simulazione
if ~exist('tout','var') || ~exist('y_obs_out','var')
    error('Eseguire prima la simulazione di fem1d_simulink.slx');
end

t  = tout;
y  = y_obs_out;    % spostamento nodo osservato
fc = f_ctrl_out;   % forza di controllo

dt_sim  = mean(diff(t));
Fs      = 1/dt_sim;
idx_on  = find(t >= t_ctrl_on, 1);

%% ===== FIGURE 1: Risposta temporale =====
figure('Name','Risposta temporale','NumberTitle','off','Position',[100 100 1000 600]);

subplot(3,1,1);
plot(t, y*1e6, 'b', 'LineWidth',1.0);
hold on;
xline(t_ctrl_on,'r--','LQR ON','LabelVerticalAlignment','bottom','LineWidth',1.5);
grid on; xlabel('t [s]'); ylabel('u_{obs} [µm]');
title(sprintf('Spostamento nodo osservato (x = %.2f m) — disturbo a f_2 = %.3f Hz', ...
    node_obs*le, f_exc));
legend('Risposta','Controllo ON');

subplot(3,1,2);
plot(t, fc, 'r', 'LineWidth',1.0);
hold on;
xline(t_ctrl_on,'r--','','LineWidth',1.5);
grid on; xlabel('t [s]'); ylabel('F_{ctrl} [N]');
title('Forza di controllo LQR');

% Energia cinetica (stima scalare sul nodo obs)
subplot(3,1,3);
y_dot_approx = gradient(y, dt_sim);
E_kin = 0.5 * y_dot_approx.^2;  % proporzionale, massa non nota qui
semilogy(t, E_kin + eps, 'k', 'LineWidth',1.0);
hold on;
xline(t_ctrl_on,'r--','LQR ON','LineWidth',1.5);
grid on; xlabel('t [s]'); ylabel('E_{cin,rel} [a.u.]');
title('Energia cinetica relativa al nodo osservato');

sgtitle('Co-simulazione FEM 1D + LQR — 50 elementi, disturbo 2° modo');

%% ===== FIGURE 2: Analisi in frequenza =====
figure('Name','Analisi FFT','NumberTitle','off','Position',[100 120 1000 500]);

% FFT pre-controllo
seg_pre = y(1:idx_on-1);
N_pre   = length(seg_pre);
Y_pre   = abs(fft(seg_pre)) / N_pre;
f_ax    = (0:N_pre-1) * Fs / N_pre;
idx_half= 1:floor(N_pre/2);

% FFT post-controllo
seg_post= y(idx_on:end);
N_post  = length(seg_post);
Y_post  = abs(fft(seg_post)) / N_post;
f_ax_p  = (0:N_post-1) * Fs / N_post;
idx_halfp= 1:floor(N_post/2);

subplot(1,2,1);
semilogy(f_ax(idx_half), Y_pre(idx_half)*1e6 + 1e-10, 'b');
hold on;
for i=1:5
    xline(f_n(i),'k:',sprintf('f_%d',i),'LineWidth',0.8);
end
xlim([0, max(f_n(5)*1.2, f_exc*2)]);
grid on; xlabel('f [Hz]'); ylabel('|Y| [µm]');
title('Spettro PRIMA del controllo');

subplot(1,2,2);
semilogy(f_ax_p(idx_halfp), Y_post(idx_halfp)*1e6 + 1e-10, 'r');
hold on;
for i=1:5
    xline(f_n(i),'k:',sprintf('f_%d',i),'LineWidth',0.8);
end
xlim([0, max(f_n(5)*1.2, f_exc*2)]);
grid on; xlabel('f [Hz]'); ylabel('|Y| [µm]');
title('Spettro DOPO il controllo LQR');

sgtitle('Contenuto in frequenza dello spostamento al nodo osservato');

%% ===== FIGURE 3: Forme modali + posizionamento =====
figure('Name','Modi e posizionamento','NumberTitle','off','Position',[100 140 800 400]);
x_nodes = (1:n_dof)*le;

subplot(1,2,1);
plot(x_nodes, Phi(:,1), 'b-o', 'MarkerSize',3, 'LineWidth',1.5); hold on;
plot(x_nodes, Phi(:,2), 'r-s', 'MarkerSize',3, 'LineWidth',1.5);
xline(node_dist*le, 'k--', 'Disturbo');
xline(node_ctrl*le, 'g--', 'Controllo');
grid on; xlabel('x [m]'); ylabel('\phi [m^{-1/2}]');
title('Modi 1 e 2'); legend('\phi_1','\phi_2');

subplot(1,2,2);
bar([abs(Phi(node_dist,1:4)); abs(Phi(node_ctrl,1:4))]');
set(gca,'XTickLabel',{'Modo 1','Modo 2','Modo 3','Modo 4'});
legend('Nodo disturbo','Nodo controllo');
grid on; ylabel('|\phi_i|'); title('Osservabilità/controllabilità modale');

%% ===== Statistiche =====
amp_pre  = rms(y(1:idx_on-1));
amp_post = rms(y(idx_on:end));
att_dB   = 20*log10(amp_pre / (amp_post + eps));
fc_rms   = rms(fc(idx_on:end));

fprintf('\n=== Statistiche post-processing ===\n');
fprintf('RMS spostamento PRE-controllo : %.4f µm\n', amp_pre*1e6);
fprintf('RMS spostamento POST-controllo: %.4f µm\n', amp_post*1e6);
fprintf('Attenuazione LQR              : %.1f dB\n', att_dB);
fprintf('RMS forza di controllo        : %.4f N\n', fc_rms);
fprintf('Frequenza eccitazione         : %.4f Hz  (modo 2)\n', f_exc);
