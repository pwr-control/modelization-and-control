%% growing_harmonic_video.m
% Animazione di un'armonica con ampiezza linearmente crescente.
% La finestra temporale scorre (sliding window) e mostra sempre N_periods periodi.
% Output: file .mp4 su disco.
%
% Davide Bagnara – Leitner SpA
% -----------------------------------------------------------------------

clear; close all; clc;

% -----------------------------------------------------------------------
%  PARAMETRI UTENTE
% -----------------------------------------------------------------------
f0          = 10e3;          % frequenza armonica [Hz]
T0          = 1/f0;          % periodo [s]
N_periods   = 3;             % periodi visibili nella finestra
dAdt        = 1.0;           % tasso di crescita ampiezza [pu/s]  A(t) = dAdt * t

t_start     = 0;             % inizio simulazione [s]
t_end       = 2;             % fine simulazione [s]  → 50 periodi a 10 kHz

fps         = 25;            % frame al secondo del video
video_file  = 'growing_harmonic.mp4';

% -----------------------------------------------------------------------
%  GRIGLIA TEMPORALE FINE (per il segnale dentro ogni frame)
% -----------------------------------------------------------------------
pts_per_period = 500;                    % punti per periodo (risoluzione grafica)
Fs_fine        = pts_per_period * f0;    % "sample rate" grafico
dt_fine        = 1 / Fs_fine;

t_fine = (t_start : dt_fine : t_end)';  % vettore tempo globale

% segnale completo: x(t) = A(t) * sin(2*pi*f0*t)
A_full = dAdt * t_fine;                 % ampiezza istantanea
x_full = A_full .* sin(2*pi*f0*t_fine);

% -----------------------------------------------------------------------
%  VETTORE DEI FRAME (istanti del "centro" della finestra scorrevole)
% -----------------------------------------------------------------------
win_half   = N_periods * T0 / 2;        % metà larghezza finestra
dt_frame   = 1 / fps;                   % passo temporale tra frame

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
           'FontSize',13,'LineWidth',1.2);
hold(ax,'on'); grid(ax,'on');
ax.GridColor       = [0.3 0.3 0.3];
ax.GridAlpha       = 0.8;
ax.XTickLabelMode  = 'manual';

% Limiti Y: ampiezza massima raggiungibile + 10%
A_max = dAdt * t_end * 1.1;
ylim(ax, [-A_max  A_max]);
xlabel(ax,'Tempo  [μs]','Color','w','FontSize',13);
ylabel(ax,'Ampiezza  [pu]','Color','w','FontSize',13);

% linea del segnale
h_line  = plot(ax, NaN, NaN, 'Color',[0.2 0.8 1.0], 'LineWidth',1.8);

% testo ampiezza istantanea (angolo in alto a sinistra)
h_amp   = text(ax, 0.02, 0.90, '', 'Units','normalized', ...
               'Color',[1 0.8 0.2], 'FontSize',14, 'FontWeight','bold');

% testo tempo corrente
h_time  = text(ax, 0.02, 0.80, '', 'Units','normalized', ...
               'Color',[0.7 0.7 0.7], 'FontSize',12);

% linea verticale al centro (tempo corrente)
h_vline = xline(ax, 0, '--', 'Color',[1 0.4 0.4], 'LineWidth',1.2, 'Alpha',0.7);

title(ax, sprintf('Armonica %.0f kHz  –  crescita ampiezza: dA/dt = %.1f pu/s', ...
      f0/1e3, dAdt), 'Color','w','FontSize',14);

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
    mask  = (t_fine >= t_win(1)) & (t_fine <= t_win(2));
    t_seg = t_fine(mask);
    x_seg = x_full(mask);

    % aggiorna XLim in microsecondi
    xlim(ax, t_win * 1e6);

    % xtick: un tick per periodo
    t_ticks = (ceil(t_win(1)/T0)*T0 : T0 : t_win(2));
    ax.XTick      = t_ticks * 1e6;
    ax.XTickLabel = arrayfun(@(v) sprintf('%.3f', v*1e6), t_ticks, ...
                             'UniformOutput', false);

    % aggiorna linea segnale
    set(h_line,  'XData', t_seg*1e6, 'YData', x_seg);

    % aggiorna linea verticale centrale
    h_vline.Value = tc * 1e6;

    % ampiezza al tempo corrente
    A_now = dAdt * tc;
    set(h_amp,  'String', sprintf('A(t) = %.4f pu', A_now));
    set(h_time, 'String', sprintf('t = %.3f ms',   tc*1e3));

    % cattura e scrivi frame
    frame = getframe(fig);
    writeVideo(vw, frame);
end

close(vw);
close(fig);

fprintf('✓ Video salvato in: %s\n', fullfile(pwd, video_file));
