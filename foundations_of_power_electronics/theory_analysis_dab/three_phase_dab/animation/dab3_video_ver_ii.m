

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

fontsize_title = 14;
fontsize_legend = 14;
fontsize_axis = 12;

fps         = 100;                      % frame al secondo del video
slowdown    = 5;                        % fattore di rallentamento
dt_frame    = 1 / (fps * slowdown);     % 2ms invece di 10ms
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
segnale_4 = dab_voltage_output_modA_sim;
segnale_5 = dab_current_output_modA_sim;

% -----------------------------------------------------------------------
%  VETTORE DEI FRAME (istanti del "centro" della finestra scorrevole)
% -----------------------------------------------------------------------
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

% --- subplot 211: tensioni primario e secondario ---
ax1 = subplot(2,1,1,'Parent',fig);
set(ax1,'Color','k','XColor','w','YColor','w', ...
        'FontSize',fontsize_axis,'LineWidth',tratto2);
hold(ax1,'on'); grid(ax1,'on');
ax1.GridColor      = [0.3 0.3 0.3];
ax1.GridAlpha      = 0.8;
ax1.XTickLabelMode = 'manual';

V_max = max([abs(segnale_1); abs(segnale_2)]) * 1.15;
ylim(ax1, [-V_max  V_max]);
ylabel(ax1,'Voltage  [V]','Color','w','FontSize',fontsize_axis);
title(ax1,'Three Phase DAB – Transformer Voltages (Phase U)', ...
      'Color','w','FontSize',fontsize_title);

h_line_1 = plot(ax1, NaN, NaN, 'Color',colore1, 'LineWidth',tratto3);
h_line_2 = plot(ax1, NaN, NaN, 'Color',colore2, 'LineWidth',tratto3);
legend(ax1, {'u1 (primary)','u2 (secondary)'}, ...
       'TextColor','w','Color','none','EdgeColor',[0.4 0.4 0.4], ...
       'FontSize',fontsize_legend,'Location','northeast');
h_vline1 = xline(ax1, 0, '--', 'Color','w', 'LineWidth',1.2, 'Alpha',0.5);

% --- subplot 212: corrente ---
ax2 = subplot(2,1,2,'Parent',fig);
set(ax2,'Color','k','XColor','w','YColor','w', ...
        'FontSize',fontsize_axis,'LineWidth',tratto2);
hold(ax2,'on'); grid(ax2,'on');
ax2.GridColor      = [0.3 0.3 0.3];
ax2.GridAlpha      = 0.8;
ax2.XTickLabelMode = 'manual';

I_max = max(abs(segnale_3)) * 1.15;
ylim(ax2, [-I_max  I_max]);
xlabel(ax2,'Time  [s]','Color','w','FontSize',fontsize_axis);
ylabel(ax2,'Current  [A]','Color','w','FontSize',fontsize_axis);
title(ax2,'Transformer Current (Phase U)', ...
      'Color','w','FontSize',fontsize_title);

h_line_3 = plot(ax2, NaN, NaN, 'Color',colore3, 'LineWidth',tratto3);
legend(ax2, {'i1 (primary)'}, ...
       'TextColor','w','Color','none','EdgeColor',[0.4 0.4 0.4], ...
       'FontSize',fontsize_legend,'Location','northeast');
h_vline2 = xline(ax2, 0, '--', 'Color','w', 'LineWidth',1.2, 'Alpha',0.5);

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

    % aggiorna XLim e XTick su entrambe le subplot
    t_ticks   = (ceil(t_win(1)/T0)*T0 : T0 : t_win(2));
    tick_lbls = arrayfun(@(v) sprintf('%.3f', v), t_ticks, 'UniformOutput', false);

    xlim(ax1, t_win);
    ax1.XTick      = t_ticks;
    ax1.XTickLabel = tick_lbls;

    xlim(ax2, t_win);
    ax2.XTick      = t_ticks;
    ax2.XTickLabel = tick_lbls;

    % aggiorna linee segnale
    set(h_line_1, 'XData', tempo_seg, 'YData', segnale_1_seg);
    set(h_line_2, 'XData', tempo_seg, 'YData', segnale_2_seg);
    set(h_line_3, 'XData', tempo_seg, 'YData', segnale_3_seg);

    % aggiorna linee verticali centrali
    h_vline1.Value = tc;
    h_vline2.Value = tc;

    % cattura e scrivi frame
    frame = getframe(fig);
    writeVideo(vw, frame);
end

close(vw);
close(fig);

fprintf('✓ Video salvato in: %s\n', fullfile(pwd, video_file));
