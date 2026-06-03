clear
close all
clc

load sim_results_DP650B1700_4kHz_520A_cosphi8p;

tratto1 = 2;
tratto2 = 2;
tratto3 = 2;
tratto4 = 3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
colore4 = [0 0 0];
t1c = t_ts_inv_sim_mod1(end) - glb_time.Nc*glb_time.tc;
t2c = t_ts_inv_sim_mod1(end);
t1s = t_ts_inv_sim_mod1(end) - glb_time.Ns_inv*glb_time.ts_inv;
t2s = t_ts_inv_sim_mod1(end);


fontsize_title = 12;
fontsize_labels = 13;
options = bodeoptions;
options.FreqUnits = 'Hz';

t_tc_sim = t1c:glb_time.tc:t2c-glb_time.tc;

figure;
subplot 311
plot(t_tc_sim,iuvw_psm_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
title('Phase U motor current in A: 520\,A - 15\,Hz - cos(phi) = 0.8.','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{u}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
ylabel('$i/\mathrm{A}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'ylim',[-750 750]);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 312
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,igbt_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 1500]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,igbt_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 100]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{Q1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('DP650B1700 IGBT Q1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 313
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,diode_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 750]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,diode_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 75]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{D1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('DP650B1700 Diode D1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

print('thermal_behavior_Q1','-depsc');
movefile('thermal_behavior_Q1.eps', 'figures/sim_results_DP650B1700_4kHz_520A_cosphi8p');
print('thermal_behavior_Q1','-dpdf');
movefile('thermal_behavior_Q1.pdf', 'figures/sim_results_DP650B1700_4kHz_520A_cosphi8p');

% print('thermal_behavior_Q1','-depsc');
% movefile('thermal_behavior_Q1.eps', 'figures/sim_results_FF1200R17_4kHz_520A_cosphi8p');
% print('thermal_behavior_Q1','-dpdf');
% movefile('thermal_behavior_Q1.pdf', 'figures/sim_results_FF1200R17_4kHz_520A_cosphi8p');

clear

load sim_results_FF1200R17_4kHz_520A_cosphi8p;

tratto1 = 2;
tratto2 = 2;
tratto3 = 2;
tratto4 = 3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
colore4 = [0 0 0];
t1c = t_ts_inv_sim_mod1(end) - glb_time.Nc*glb_time.tc;
t2c = t_ts_inv_sim_mod1(end);
t1s = t_ts_inv_sim_mod1(end) - glb_time.Ns_inv*glb_time.ts_inv;
t2s = t_ts_inv_sim_mod1(end);


fontsize_title = 12;
fontsize_labels = 13;
options = bodeoptions;
options.FreqUnits = 'Hz';

t_tc_sim = t1c:glb_time.tc:t2c-glb_time.tc;

figure;
subplot 311
plot(t_tc_sim,iuvw_psm_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
title('Phase U motor current in A: 520\,A - 15\,Hz - cos(phi) = 0.8.','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{u}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
ylabel('$i/\mathrm{A}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'ylim',[-750 750]);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 312
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,igbt_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 1500]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,igbt_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 100]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{Q1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('FF1200R17 IGBT Q1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 313
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,diode_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 750]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,diode_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 75]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{D1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('FF1200R17 Diode D1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

% print('thermal_behavior_Q1','-depsc');
% movefile('thermal_behavior_Q1.eps', 'figures/sim_results_DP650B1700_4kHz_520A_cosphi8p');
% print('thermal_behavior_Q1','-dpdf');
% movefile('thermal_behavior_Q1.pdf', 'figures/sim_results_DP650B1700_4kHz_520A_cosphi8p');

print('thermal_behavior_Q1','-depsc');
movefile('thermal_behavior_Q1.eps', 'figures/sim_results_FF1200R17_4kHz_520A_cosphi8p');
print('thermal_behavior_Q1','-dpdf');
movefile('thermal_behavior_Q1.pdf', 'figures/sim_results_FF1200R17_4kHz_520A_cosphi8p');

clear

load sim_results_DP650B1700_4kHz_370A_cosphi8p;

tratto1 = 2;
tratto2 = 2;
tratto3 = 2;
tratto4 = 3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
colore4 = [0 0 0];
t1c = t_ts_inv_sim_mod1(end) - glb_time.Nc*glb_time.tc;
t2c = t_ts_inv_sim_mod1(end);
t1s = t_ts_inv_sim_mod1(end) - glb_time.Ns_inv*glb_time.ts_inv;
t2s = t_ts_inv_sim_mod1(end);


fontsize_title = 12;
fontsize_labels = 13;
options = bodeoptions;
options.FreqUnits = 'Hz';

t_tc_sim = t1c:glb_time.tc:t2c-glb_time.tc;

figure;
subplot 311
plot(t_tc_sim,iuvw_psm_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
title('Phase U motor current in A: 370\,A - 15\,Hz - cos(phi) = 0.8.','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{u}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
ylabel('$i/\mathrm{A}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'ylim',[-750 750]);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 312
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,igbt_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 1500]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,igbt_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 100]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{Q1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('DP650B1700 IGBT Q1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 313
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,diode_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 750]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,diode_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 75]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{D1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('DP650B1700 Diode D1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

print('thermal_behavior_Q1','-depsc');
movefile('thermal_behavior_Q1.eps', 'figures/sim_results_DP650B1700_4kHz_370A_cosphi8p');
print('thermal_behavior_Q1','-dpdf');
movefile('thermal_behavior_Q1.pdf', 'figures/sim_results_DP650B1700_4kHz_370A_cosphi8p');

% print('thermal_behavior_Q1','-depsc');
% movefile('thermal_behavior_Q1.eps', 'figures/sim_results_FF1200R17_4kHz_520A_cosphi8p');
% print('thermal_behavior_Q1','-dpdf');
% movefile('thermal_behavior_Q1.pdf', 'figures/sim_results_FF1200R17_4kHz_520A_cosphi8p');

clear

load sim_results_FF1200R17_4kHz_370A_cosphi8p;

tratto1 = 2;
tratto2 = 2;
tratto3 = 2;
tratto4 = 3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
colore4 = [0 0 0];
t1c = t_ts_inv_sim_mod1(end) - glb_time.Nc*glb_time.tc;
t2c = t_ts_inv_sim_mod1(end);
t1s = t_ts_inv_sim_mod1(end) - glb_time.Ns_inv*glb_time.ts_inv;
t2s = t_ts_inv_sim_mod1(end);


fontsize_title = 12;
fontsize_labels = 13;
options = bodeoptions;
options.FreqUnits = 'Hz';

t_tc_sim = t1c:glb_time.tc:t2c-glb_time.tc;

figure;
subplot 311
plot(t_tc_sim,iuvw_psm_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
title('Phase U motor current in A: 370\,A - 15\,Hz - cos(phi) = 0.8.','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{u}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
ylabel('$i/\mathrm{A}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'ylim',[-750 750]);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 312
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,igbt_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 1500]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,igbt_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 100]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{Q1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('FF1200R17 IGBT Q1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 313
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,diode_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 750]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,diode_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 75]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{D1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('FF1200R17 Diode D1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

% print('thermal_behavior_Q1','-depsc');
% movefile('thermal_behavior_Q1.eps', 'figures/sim_results_DP650B1700_4kHz_520A_cosphi8p');
% print('thermal_behavior_Q1','-dpdf');
% movefile('thermal_behavior_Q1.pdf', 'figures/sim_results_DP650B1700_4kHz_520A_cosphi8p');

print('thermal_behavior_Q1','-depsc');
movefile('thermal_behavior_Q1.eps', 'figures/sim_results_FF1200R17_4kHz_370A_cosphi8p');
print('thermal_behavior_Q1','-dpdf');
movefile('thermal_behavior_Q1.pdf', 'figures/sim_results_FF1200R17_4kHz_370A_cosphi8p');

clear

load sim_results_FF650R17IE4_4kHz_370A_cosphi8p.mat;

tratto1 = 2;
tratto2 = 2;
tratto3 = 2;
tratto4 = 3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
colore4 = [0 0 0];
t1c = t_ts_inv_sim_mod1(end) - glb_time.Nc*glb_time.tc;
t2c = t_ts_inv_sim_mod1(end);
t1s = t_ts_inv_sim_mod1(end) - glb_time.Ns_inv*glb_time.ts_inv;
t2s = t_ts_inv_sim_mod1(end);


fontsize_title = 12;
fontsize_labels = 13;
options = bodeoptions;
options.FreqUnits = 'Hz';

t_tc_sim = t1c:glb_time.tc:t2c-glb_time.tc;

figure;
subplot 311
plot(t_tc_sim,iuvw_psm_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
title('Phase U motor current in A: 370\,A - 15\,Hz - cos(phi) = 0.8.','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{u}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
ylabel('$i/\mathrm{A}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'ylim',[-750 750]);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 312
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,igbt_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 1500]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,igbt_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 100]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{Q1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('FF650R17IE4 IGBT Q1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 313
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,diode_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 750]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,diode_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 75]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{D1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('FF650R17IE4 Diode D1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

print('thermal_behavior_Q1','-depsc');
movefile('thermal_behavior_Q1.eps', 'figures/sim_results_FF650R17IE4_4kHz_370A_cosphi8p');
print('thermal_behavior_Q1','-dpdf');
movefile('thermal_behavior_Q1.pdf', 'figures/sim_results_FF650R17IE4_4kHz_370A_cosphi8p');

clear

load sim_results_FF650R17IE4_4kHz_520A_cosphi8p.mat;

tratto1 = 2;
tratto2 = 2;
tratto3 = 2;
tratto4 = 3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
colore4 = [0 0 0];
t1c = t_ts_inv_sim_mod1(end) - glb_time.Nc*glb_time.tc;
t2c = t_ts_inv_sim_mod1(end);
t1s = t_ts_inv_sim_mod1(end) - glb_time.Ns_inv*glb_time.ts_inv;
t2s = t_ts_inv_sim_mod1(end);


fontsize_title = 12;
fontsize_labels = 13;
options = bodeoptions;
options.FreqUnits = 'Hz';

t_tc_sim = t1c:glb_time.tc:t2c-glb_time.tc;

figure;
subplot 311
plot(t_tc_sim,iuvw_psm_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
title('Phase U motor current in A: 520\,A - 15\,Hz - cos(phi) = 0.8.','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{u}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
ylabel('$i/\mathrm{A}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'ylim',[-750 750]);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 312
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,igbt_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 1500]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,igbt_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 100]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{Q1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('FF650R17IE4 IGBT Q1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 313
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(t_tc_sim,diode_ploss_invQ1_sim,'-','LineWidth',tratto1,'Color',colore1);
set(gca,'ylim',[0 750]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold on
yyaxis right;
plot(t_tc_sim,diode_JH_temp_invQ1_sim,'-','LineWidth',tratto1,'Color',colore2);
set(gca,'ylim',[0 75]);
ylabel('$p/\mathrm{W}$','Interpreter','latex','FontSize', fontsize_labels);
hold off
legend('$p_\mathrm{Q1}$','$\Delta T_\mathrm{D1}^{\mathrm{jh}}$','Location','northeast',...
    'Interpreter','latex','FontSize',fontsize_labels);
title('FF650R17IE4 Diode D1 Power Loss and Junction-HeatSink Temperature','Interpreter','latex','FontSize',fontsize_title);
ylabel('$T/\mathrm{C}$','Interpreter','latex','FontSize', fontsize_labels);
xlabel('$t/\mathrm{s}$','Interpreter','latex','FontSize', fontsize_labels);
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

print('thermal_behavior_Q1','-depsc');
movefile('thermal_behavior_Q1.eps', 'figures/sim_results_FF650R17IE4_4kHz_520A_cosphi8p');
print('thermal_behavior_Q1','-dpdf');
movefile('thermal_behavior_Q1.pdf', 'figures/sim_results_FF650R17IE4_4kHz_520A_cosphi8p');