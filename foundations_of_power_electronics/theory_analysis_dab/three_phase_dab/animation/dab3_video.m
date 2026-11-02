

close all; clc;
clear; 
load ../sim_results/sim_results_1;

% -----------------------------------------------------------------------
%  PARAMETRI UTENTE
% -----------------------------------------------------------------------
tratto1 = 1;
tratto2 = 2;
tratto3 = 3;

colore1 = [1.0 0.0 0.0];
colore2 = [0.0 1.0 0.0];
colore3 = [0.0 0.0 1.0];

N1c = floor(glb_time.Nc/2);
N2c = floor(glb_time.Nc);
N3c = floor(glb_time.Nc - 4/glb_time.fPWM_AFE/glb_time.tc);

t1c = time_tc_sim(N1c);
t2c = time_tc_sim(N2c);

t3c = time_tc_sim(N3c);
t4c = time_tc_sim(N2c);

t_start     = time_tc_sim(1);
t_end       = time_tc_sim(end);

fontsize_title = 18;
fontsize_legend = 18;
fontsize_axis = 16;

video_file  = 'dab3.mp4';

% -----------------------------------------------------------------------
%  GRIGLIA TEMPORALE FINE (per il segnale dentro ogni frame)
% -----------------------------------------------------------------------
f0          = fpwm_dab/6;          % frequenza armonica [Hz]
T0          = 1/f0;          % periodo [s]
N_periods   = 4;             % periodi visibili nella finestra

pts_per_period = T0 * N_periods/glb_time.tc;       % punti per periodo (risoluzione grafica)
Fs_fine        = 1 / glb_time.tc;                    % "sample rate" grafico
dt_fine        = 1 / Fs_fine;


tempo = time_tc_sim;
segnale_1 = u1_dab_transformer_modA_sim(:,1);
segnale_2 = u2_dab_transformer_modA_sim(:,1);
segnale_3 = i1_dab_transformer_modA_sim(:,1);

% -----------------------------------------------------------------------
%  VETTORE DEI FRAME (istanti del "centro" della finestra scorrevole)
% -----------------------------------------------------------------------
fps       = 100;          % mantieni fluidità
slowdown  = 5;            % fattore di rallentamento
dt_frame  = 1 / (fps * slowdown);   % 2ms invece di 10ms 
win_half   = N_periods * T0 / 2;        % metà larghezza finestra

% Il centro della finestra parte da win_half (così la finestra è sempre piena)
t_centers  = (t_start + win_half : dt_frame : t_end - win_half)';

if isempty(t_centers)
    error('t_end troppo piccolo per contenere almeno un frame. Aumenta t_end.');
end

N_frames = numel(t_centers);
fprintf('Generazione di %d frame a %d fps → durata video %.2f s\n', ...
        N_frames, fps, N_frames/fps);

% -----------------------------------------------------------------------
%  SETUP FIGURA
% -----------------------------------------------------------------------
fig = figure('Color','k','Position',[100 100 1280 720]);
ax  = axes('Parent',fig,'Color','k','XColor','w','YColor','w', ...
           'FontSize',fontsize_axis,'LineWidth',tratto2);
hold(ax,'on'); grid(ax,'on');
ax.GridColor       = [0.3 0.3 0.3];
ax.GridAlpha       = 0.8;
ax.XTickLabelMode  = 'manual';

% Limiti Y: ampiezza massima raggiungibile + 10%
V_max = max(segnale_1);
ylim(ax, [-V_max  V_max]);
xlabel(ax,'Time  [s]','Color','w','FontSize',fontsize_axis);
ylabel(ax,'Voltage  [V]','Color','w','FontSize',fontsize_axis);

% linea del segnale
h_line_1  = plot(ax, NaN, NaN, 'Color',colore1, 'LineWidth',tratto3);
h_line_2  = plot(ax, NaN, NaN, 'Color',colore2, 'LineWidth',tratto3);
h_line_3  = plot(ax, NaN, NaN, 'Color',colore3, 'LineWidth',tratto3);

% testo ampiezza istantanea (angolo in alto a sinistra)
h_amp   = text(ax, 0.02, 0.90, '', 'Units','normalized', ...
               'Color',colore1, 'FontSize',fontsize_legend, 'FontWeight','bold');

% testo tempo corrente
h_time  = text(ax, 0.02, 0.80, '', 'Units','normalized', ...
               'Color',colore1, 'FontSize',fontsize_legend);

% linea verticale al centro (tempo corrente)
h_vline = xline(ax, 0, '--', 'Color',colore1, 'LineWidth',1.2, 'Alpha',0.7);

title(ax, sprintf('Three Phase DAB - Current Phase U'), 'Color','w','FontSize',14);

% -----------------------------------------------------------------------
%  WRITER VIDEO
% -----------------------------------------------------------------------
vw = VideoWriter(video_file, 'MPEG-4');
vw.FrameRate = fps;
vw.Quality   = 95;
open(vw);

% -----------------------------------------------------------------------
%  LOOP FRAME
% -----------------------------------------------------------------------
for k = 1:N_frames
    tc    = t_centers(k);
    t_win = [tc - win_half,  tc + win_half];

    % estrai il segmento di segnale nella finestra
    mask  = (tempo >= t_win(1)) & (tempo <= t_win(2));
    tempo_seg = tempo(mask);
    segnale_1_seg = segnale_1(mask);
    segnale_2_seg = segnale_2(mask);
    segnale_3_seg = segnale_3(mask);

    % aggiorna XLim in microsecondi
    xlim(ax, t_win);

    % xtick: un tick per periodo
    t_ticks = (ceil(t_win(1)/T0)*T0 : T0 : t_win(2));
    ax.XTick      = t_ticks;
    ax.XTickLabel = arrayfun(@(v) sprintf('%.3f', v), t_ticks, ...
                             'UniformOutput', false);

    % aggiorna linea segnale
    set(h_line_1,  'XData', tempo_seg, 'YData', segnale_1_seg);
    set(h_line_2,  'XData', tempo_seg, 'YData', segnale_2_seg);
    set(h_line_3,  'XData', tempo_seg, 'YData', segnale_3_seg);

    % aggiorna linea verticale centrale
    h_vline.Value = tc;

    % ampiezza al tempo corrente
    % A_now = dAdt * tc;
    % set(h_amp,  'String', sprintf('A(t) = %.4f pu', A_now));
    % set(h_time, 'String', sprintf('t = %.3f ms',   tc*1e3));

    % cattura e scrivi frame
    frame = getframe(fig);
    writeVideo(vw, frame);
end

close(vw);
close(fig);

fprintf('✓ Video salvato in: %s\n', fullfile(pwd, video_file));
