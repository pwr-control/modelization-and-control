close all
clc

tratto1=2.5;
tratto2=2.5;
tratto3=2;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1 = 0.7;
t2 = 1.1;


fontsize_plotting = 14;

figure;
% subplot(3,1, [1 2]); 
% subplot(2,1,1); 
subplot(3,1,1); 
plot(time_ts_afe_sim,ugd_abc_grid_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_ts_afe_sim,ugd_abc_grid_sim(:,2),'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(time_ts_afe_sim,ugd_abc_grid_sim(:,3),'-','LineWidth',tratto1,'Color',colore3);
hold off
title('Grid Voltages','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{g}^{a}$','$u_{g}^{b}$','$u_{g}^{c}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize',fontsize_plotting);
ylabel('pu','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1.2 1.2]);
set(gca,'xlim',[t1 t2]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(1); chH(2); chH(3)]);
% subplot(3,1,3);
subplot(3,1,[2 3]);
% subplot(2,1,2);
plot(time_ts_afe_sim,phase_grid_fhtsr_dqpll_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_ts_afe_sim,omega_grid_fhtsr_dqpll_sim,'-','LineWidth',tratto1,'Color',colore2);
hold off
title('FHTSR dqPLL outputs','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\vartheta_{g}$','$\omega_{g}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('pu','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-pi*1.1 pi*1.1]);
set(gca,'xlim',[t1 t2]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(1); chH(2)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('figure_fhtsr_dqPLL_1','-depsc');

figure;
% subplot(3,1, [1 2]); 
subplot(2,1,1); 
% subplot(3,1,1); 
plot(time_ts_afe_sim,u_fht_alpha_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_ts_afe_sim,v_fht_alpha_sim,'-','LineWidth',tratto1,'Color',colore2);
hold off
title('FHT-SR Voltage Outputs','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{\alpha}^{fht}$','$v_{\alpha}^{fht}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize',fontsize_plotting);
ylabel('pu','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1.2 1.2]);
set(gca,'xlim',[t1 t2]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(1); chH(2)]);
% subplot(3,1,3);
% subplot(3,1,[2 3]);
subplot(2,1,2);
plot(time_ts_afe_sim,u_fht_beta_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_ts_afe_sim,v_fht_beta_sim,'-','LineWidth',tratto1,'Color',colore2);
hold off
title('FHT-SR Voltage Outputs','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{\beta}^{fht}$','$v_{\beta}^{fht}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('pu','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1.2 1.2]);
set(gca,'xlim',[t1 t2]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(1); chH(2)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('figure_fhtsr_dqPLL_2','-depsc');