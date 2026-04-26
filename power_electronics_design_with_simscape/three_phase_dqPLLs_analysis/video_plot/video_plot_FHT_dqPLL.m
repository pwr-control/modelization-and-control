% =========================================================
%  video_plot_comparison.m
%  Finestra sinistra:  subplot 1 → tensioni trifase (Icc = 20 Inom)
%                      subplot 2 → correnti trifase (Icc = 20 Inom)
%  Finestra destra:  subplot 1 → tensioni trifase (Icc = 2 Inom)
%                      subplot 2 → correnti trifase (Icc = 2 Inom)
%  Author: 
% =========================================================
clear; close all; clc

font_size_legend = 14;
font_size_labels = 14;
font_size_title = 14;

load ..\sim_results_FHTSR_dqPLL_2.mat;
tc_eq = glb_time.ts_afe;
tc = tc_eq;
t1 = 0.7;
t2 = 1.1;
N1 = floor(t1/tc);
N2 = floor(t2/tc);
N = N2 - N1 + 1;


vg_u = ugd_abc_grid_sim(N1:N2,1);
vg_v = ugd_abc_grid_sim(N1:N2,2);
vg_w = ugd_abc_grid_sim(N1:N2,3);

phase_grid_fht = phase_grid_fhtsr_dqpll_sim(N1:N2);
omega_grid_fht = omega_grid_fhtsr_dqpll_sim(N1:N2);

u_fht_alpha = u_fht_alpha_sim(N1:N2);
v_fht_alpha = v_fht_alpha_sim(N1:N2);

u_fht_beta = u_fht_beta_sim(N1:N2);
v_fht_beta = v_fht_beta_sim(N1:N2);

time = time_ts_afe_sim(N1:N2);


%% Parametri
sim_duration   = t2-t1;
video_duration = 10.0;
fps            = 30;
output_file    = 'video_plot_FHTSR_dqPLL.mp4';

%% Colori
col_u  = [0.25  0.72  1.00];
col_v  = [1.00  0.55  0.10];
col_w  = [0.20  0.85  0.45];
col_P1p = [1.00  0.85  0.20];
col_P1n = [1.00  0.45  0.20];
col_Q1p = [0.75  0.30  1.00];
col_Q1n = [0.30  0.85  0.85];

%% Limiti assi (precalcolati)
lim = @(x) deal(min(x)*1.15, max(x)*1.15);
[yphase_min, yphase_max] = lim(phase_grid_fht);
[ypu_min, ypu_max] = lim([vg_u; vg_v; vg_w]);

fix = @(a,b) deal(a-(a==b), b+(a==b));
[yi_min,   yphase_max  ] = fix(yphase_min,   yphase_max  );
[yv_min,   ypu_max  ] = fix(ypu_min,   ypu_max  );

n_frames = round(fps * video_duration);

%% Video writer
vw = VideoWriter(output_file, 'MPEG-4');
vw.FrameRate = fps;
vw.Quality   = 95;
open(vw);

%% ---- Figura SINISTRA: tensioni + correnti ----
W = 900;  H = 720;          % dimensioni di ciascuna finestra
fig_L = figure('Color','k', 'Position',[50,  100, W, H], ...
               'MenuBar','none','ToolBar','none');

ax_v = subplot(3,1,1,'Parent',fig_L);
setup_ax(ax_v);
hl_vu_g = plot(ax_v,NaN,NaN,'Color',col_u,'LineWidth',1.5);
hl_vv_g = plot(ax_v,NaN,NaN,'Color',col_v,'LineWidth',1.5);
hl_vw_g = plot(ax_v,NaN,NaN,'Color',col_w,'LineWidth',1.5);
hc_v_g  = xline(ax_v,0,'r--','LineWidth',0.8,'Alpha',0.7);
legend(ax_v,{'v_{g,u}','v_{g,v}','v_{g,w}'},'TextColor','w', ...
       'Color','none','EdgeColor',[0.4 0.4 0.4],'Location','northeast','FontSize',font_size_legend);
title(ax_v,'Grid Voltages','Color','w','FontSize',font_size_title,'FontWeight','normal');
ylabel(ax_v,'v_g  [V]','Color','w','FontSize',font_size_labels);
xlim(ax_v,[t1,t2]); ylim(ax_v,[ypu_min,ypu_max]);
set(ax_v,'XTickLabel',{});
ht_v_g = make_text(ax_v,'w');

ax_phase = subplot(3,1,[2 3],'Parent',fig_L);
setup_ax(ax_phase);
hl_phase_g = plot(ax_phase,NaN,NaN,'Color',col_u,'LineWidth',1.5);
hl_omega_g = plot(ax_phase,NaN,NaN,'Color',col_v,'LineWidth',1.5);
hc_phase_g  = xline(ax_phase,0,'r--','LineWidth',0.8,'Alpha',0.7);
legend(ax_phase,{'\vartheta_{g}','\omega_{g}'},'TextColor','w', ...
       'Color','none','EdgeColor',[0.4 0.4 0.4],'Location','northeast','FontSize',font_size_legend);
title(ax_phase,'Grid Phase and Pulsation [pu]','Color','w','FontSize',font_size_title,'FontWeight','normal');
ylabel(ax_phase,'phase  [rad]','Color','w','FontSize',font_size_labels);
xlabel(ax_phase,'Time  [s]','Color','w','FontSize',font_size_labels);
xlim(ax_phase,[t1,t2]); ylim(ax_phase,[yphase_min,yphase_max]);
ht_phase_g = make_text(ax_phase,'w');

%% ---- Figura DESTRA: tensioni + correnti ----
W = 900;  H = 720;          % dimensioni di ciascuna finestra
fig_R = figure('Color','k', 'Position',[50+W+10, 100, W, H], ...
               'MenuBar','none','ToolBar','none');

ax_v = subplot(2,1,1,'Parent',fig_R);
setup_ax(ax_v);
hl_u_fht_alpha = plot(ax_v,NaN,NaN,'Color',col_u,'LineWidth',1.5);
hl_v_fht_alpha = plot(ax_v,NaN,NaN,'Color',col_v,'LineWidth',1.5);
hc_uv_alpha  = xline(ax_v,0,'r--','LineWidth',0.8,'Alpha',0.7);
legend(ax_v,{'u_{\alpha}','v_{\alpha}'},'TextColor','w', ...
       'Color','none','EdgeColor',[0.4 0.4 0.4],'Location','northeast','FontSize',font_size_legend);
title(ax_v,'FHT DQ PLL Components','Color','w','FontSize',font_size_title,'FontWeight','normal');
ylabel(ax_v,'[pu]','Color','w','FontSize',font_size_labels);
xlim(ax_v,[t1,t2]); ylim(ax_v,[ypu_min,ypu_max]);
set(ax_v,'XTickLabel',{});
ht_uv_alpha = make_text(ax_v,'w');

ax_v = subplot(2,1,2,'Parent',fig_R);
setup_ax(ax_v);
hl_u_fht_beta = plot(ax_v,NaN,NaN,'Color',col_u,'LineWidth',1.5);
hl_v_fht_beta = plot(ax_v,NaN,NaN,'Color',col_v,'LineWidth',1.5);
hc_uv_beta  = xline(ax_v,0,'r--','LineWidth',0.8,'Alpha',0.7);
legend(ax_v,{'u_{\beta}','v_{\beta}'},'TextColor','w', ...
       'Color','none','EdgeColor',[0.4 0.4 0.4],'Location','northeast','FontSize',font_size_legend);
title(ax_v,'FHT DQ PLL Components','Color','w','FontSize',font_size_title,'FontWeight','normal');
ylabel(ax_v,'[pu]','Color','w','FontSize',font_size_labels);
xlabel(ax_v,'Time  [s]','Color','w','FontSize',font_size_labels);
xlim(ax_v,[t1,t2]); ylim(ax_v,[ypu_min,ypu_max]);
set(ax_v,'XTickLabel',{});
ht_uv_beta = make_text(ax_v,'w');

%% ---- Render ----
fprintf('Rendering %d frames...  ', n_frames);
t_prog = tic;

for k = 1:n_frames
    t_now = (k/n_frames) * sim_duration;
    idx   = min(round(t_now/tc)+1, N);
    t_str = sprintf('t = %.4f s', time(idx));

    % --- aggiorna figura sinistra ---
    set(hl_vu_g,'XData',time(1:idx),'YData',vg_u(1:idx));
    set(hl_vv_g,'XData',time(1:idx),'YData',vg_v(1:idx));
    set(hl_vw_g,'XData',time(1:idx),'YData',vg_w(1:idx));
    set(hc_v_g, 'Value',time(idx)); set(ht_v_g,'String',t_str);

    set(hl_phase_g,'XData',time(1:idx),'YData',phase_grid_fht(1:idx));
    set(hl_omega_g,'XData',time(1:idx),'YData',omega_grid_fht(1:idx));
    set(hc_phase_g, 'Value',time(idx)); set(ht_phase_g,'String',t_str);

    % --- aggiorna figura destra ---
    set(hl_u_fht_alpha,'XData',time(1:idx),'YData',u_fht_alpha(1:idx));
    set(hl_v_fht_alpha,'XData',time(1:idx),'YData',v_fht_alpha(1:idx));
    set(hc_uv_alpha, 'Value',time(idx)); set(ht_uv_alpha,'String',t_str);

    set(hl_u_fht_beta,'XData',time(1:idx),'YData',u_fht_beta(1:idx));
    set(hl_v_fht_beta,'XData',time(1:idx),'YData',v_fht_beta(1:idx));
    set(hc_uv_beta, 'Value',time(idx)); set(ht_uv_beta,'String',t_str);

    % --- cattura e affianca i due frame ---
    fr_L = getframe(fig_L);
    fr_R = getframe(fig_R);

    % Ridimensiona alla stessa altezza se necessario
    hL = size(fr_L.cdata,1);
    hR = size(fr_R.cdata,1);
    if hL ~= hR
        fr_R.cdata = imresize(fr_R.cdata, [hL, size(fr_R.cdata,2)]);
    end

    combined = [fr_L.cdata, fr_R.cdata];   % affianca orizzontalmente
    writeVideo(vw, combined);

    if mod(k, round(n_frames/20)) == 0
        fprintf('%.0f%%  ', 100*k/n_frames);
    end
end

fprintf('\nDone in %.1f s\n', toc(t_prog));
close(vw);
close(fig_L); close(fig_R);
fprintf('Video salvato: %s\n', output_file);

%% ---- Funzioni locali ----
function setup_ax(ax)
    set(ax, 'Color','k','XColor','w','YColor','w', ...
            'GridColor','w','GridAlpha',0.15, ...
            'XGrid','on','YGrid','on','FontSize',12);
    hold(ax,'on');
end

function h = make_text(ax, col)
    h = text(ax, 0.02, 0.93, '', ...
             'Units','normalized','Color',col, ...
             'FontSize',12,'FontName','Consolas', ...
             'VerticalAlignment','top');
end