close all
clc

tratto1=2.2;
tratto2=2.2;
tratto3=1;
colore1 = [0.5 0.5 0.5];
colore2 = [0.8 0.8 0.8];
t1c = t_tc_sim(end) - Nc*tc;
t2c = t_tc_sim(end);
t1s = t_ts_afe_sim(end) - Ns_afe*ts_afe;
t2s = t_ts_afe_sim(end);
% t1s = 0;
% t2s = 1.5;

t1c_zoom = t_tc_sim(end) - 5/50;
t2c_zoom = t_tc_sim(end);
t3c_zoom = t_tc_sim(end) - 50/fPWM_AFE;
t4c_zoom = t_tc_sim(end);
t1s_zoom = t_ts_afe_sim(end) - 5/50;
t2s_zoom = t_ts_afe_sim(end);

fontsize_plotting = 12;

figure;
subplot 211
plot(t_tc_sim,ig_abc_sim(:,1),'-k','LineWidth',tratto1);
hold on
plot(t_tc_sim,ig_abc_sim(:,2),'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,ig_abc_sim(:,3),'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Grid Currents','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{g}^{a}$','$i_{g}^{b}$','$i_{g}^{c}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-750 750]);
set(gca,'xlim',[t1s t2s]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(1); chH(2); chH(3)]);
subplot 212
plot(t_tc_sim,vg_abc_sim(:,1),'-k','LineWidth',tratto1);
hold on
plot(t_tc_sim,vg_abc_sim(:,2),'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,vg_abc_sim(:,3),'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Grid Voltages','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{g}^{a}$','$u_{g}^{b}$','$u_{g}^{c}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-750 750]);
set(gca,'xlim',[t1s t2s]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(1); chH(2); chH(3)]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('grid_voltages_currents','-depsc');

figure;
subplot 211
plot(t_tc_sim,udc_dclink_sim,'-','LineWidth',tratto2,'Color',colore1);
% hold on
% plot(t_tc_sim,ic_dclink_sim,'-','LineWidth',tratto2,'Color',colore1);
% hold off
title('DClink Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[1.5 1.54]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,udc_dclink_sim,'-','LineWidth',tratto2,'Color',colore2);
title('DClink Current in AC',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[1.5 1.54]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('dclink_voltage_current','-depsc');

return
figure;
subplot 211
plot(t_tc_sim,udc_dclink_sim,'-k','LineWidth',tratto1);
% hold on
% plot(t_tc_sim,ic_dclink_sim,'-','LineWidth',tratto2,'Color',colore1);
% hold off
title('DClink Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,idc_sim,'-','LineWidth',tratto2,'Color',colore1);
title('DClink Current',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('dclink_voltage_current','-depsc');

figure;
subplot 211
plot(t_ts_afe_sim,P1p_sim./1e3,'-k','LineWidth',tratto1);
hold on
plot(t_ts_afe_sim,P1n_sim./1e3,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Grid Active Power - positive and negative sequence','Interpreter','latex','FontSize',fontsize_plotting);
legend('$P_1^p$','$P_1^n$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$kW$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_ts_afe_sim,Q1p_sim./1e3,'-k','LineWidth',tratto1);
hold on
plot(t_ts_afe_sim,Q1n_sim./1e3,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Grid Reactive Power - positive and negative sequence','Interpreter','latex','FontSize',fontsize_plotting);
legend('$Q_1^p$','$Q_1^n$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
ylabel('$kVA$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('grid_power_sequences','-depsc');

figure;
subplot 211
plot(t_ts_afe_sim,u_grid_pos_xi_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_afe_sim,u_grid_pos_eta_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Grid voltages - positive sequence','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{\xi}^p$','$u_{\eta}^p$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_ts_afe_sim,u_grid_neg_xi_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_afe_sim,u_grid_neg_eta_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Grid voltages - negative sequence','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{\xi}^n$','$u_{\eta}^n$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('grid_voltage_sequences','-depsc');

figure;
subplot 211
plot(t_ts_afe_sim,i_grid_pos_xi_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_afe_sim,i_grid_pos_eta_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Grid voltages - positive sequence','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{\xi}^p$','$i_{\eta}^p$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_ts_afe_sim,i_grid_neg_xi_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_afe_sim,i_grid_neg_eta_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Grid voltages - negative sequence','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{\xi}^n$','$i_{\eta}^n$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('grid_current_sequences','-depsc');



figure;
subplot 211
plot(t_ts_afe_sim,P1p_sim./1e3,'-k','LineWidth',tratto1);
hold on
plot(t_ts_afe_sim,P1n_sim./1e3,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Grid Active Power - positive and negative sequence','Interpreter','latex','FontSize',fontsize_plotting);
legend('$P_1^p$','$P_1^n$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$kW$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_ts_afe_sim,Q1p_sim./1e3,'-k','LineWidth',tratto1);
hold on
plot(t_ts_afe_sim,Q1n_sim./1e3,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Grid Reactive Power - positive and negative sequence','Interpreter','latex','FontSize',fontsize_plotting);
legend('$Q_1^p$','$Q_1^n$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
ylabel('$kVA$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_8','-depsc');

dclink_terminal_current_afe_rms = sqrt(mean(ileg_dc_afe_sim.^2))

figure;
plot(t_tc_sim,is_abc_sim(:,1),'-k','LineWidth',tratto1);
hold on
plot(t_tc_sim,is_abc_sim(:,2),'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,is_abc_sim(:,3),'-','LineWidth',tratto2,'Color',colore2);
hold off
title('AFE Current in A','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_r$','$i_s$','$i_t$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
% print('sim_results_fig_10','-depsc');

figure;
subplot 211
plot(t_ts_afe_sim,ia_line_rms_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_afe_sim,ib_line_rms_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_ts_afe_sim,ic_line_rms_sim,'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Grid Current (rms)','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_a^{rms}$','$i_b^{rms}$','$i_c^{rms}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_ts_afe_sim,va_line_rms_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_afe_sim,vb_line_rms_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_ts_afe_sim,vc_line_rms_sim,'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Grid Voltage (rms)','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_a^{rms}$','$u_b^{rms}$','$u_c^{rms}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
ylabel('$V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_8','-depsc');

