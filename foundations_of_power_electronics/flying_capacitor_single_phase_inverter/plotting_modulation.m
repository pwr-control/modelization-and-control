close all
clc

tratto1=1;
tratto2=2;
tratto3=2;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
% t1c = t_tc_sim(end) - Nc*tc;
t1c = t_tc_sim(end) - 2*tPWM;
tc_end = t_tc_sim(end);
fontsize_plotting = 12;
Vfc1_base = Udc/5;
Vfc2_base = 2*Vfc1_base;
Vfc3_base = 3*Vfc1_base;
Vfc4_base = 4*Vfc1_base;
DVfc = 100;


% figure;
% plot(t_tc_sim,gate_cmd_51_sim,'-','LineWidth',tratto2,'Color',colore1);
% hold on
% plot(t_tc_sim,gate_cmd_41_sim,'-','LineWidth',tratto2,'Color',colore1);
% hold on
% plot(t_tc_sim,gate_cmd_31_sim,'-','LineWidth',tratto2,'Color',colore1);
% hold on
% plot(t_tc_sim,gate_cmd_21_sim,'-','LineWidth',tratto2,'Color',colore1);
% hold on
% plot(t_tc_sim,gate_cmd_11_sim,'-','LineWidth',tratto2,'Color',colore1);
% hold off
% title('Modulation Strategy','Interpreter','latex','FontSize',fontsize_plotting);
% legend('$m_{51}$','$m_{41}$','$m_{31}$','$m_{21}$','$m_{51}$','Location','best',...
%     'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
% ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'xlim',[t1c tc_end]);
% set(gca,'ylim',[-0.2 15.2]);
% grid on
% h=gcf;
% % set(h,'PaperOrientation','landscape');
% % set(h,'PaperUnits','normalized');
% % set(h,'PaperPosition', [0 0 1 1]);
% % print('sim_results_fig_1','-depsc');
% % print('sim_results_fig_1','-djpeg');
figure;
subplot 511
plot(t_tc_sim,gate_cmd_51_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Modulation Strategy','Interpreter','latex','FontSize',fontsize_plotting);
legend('$m_{51}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[-0.2 15.2]);
grid on
subplot 512
plot(t_tc_sim,gate_cmd_41_sim,'-','LineWidth',tratto2,'Color',colore1);
% title('Modulation Strategy','Interpreter','latex','FontSize',fontsize_plotting);
legend('$m_{41}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[-0.2 15.2]);
grid on
subplot 513
plot(t_tc_sim,gate_cmd_31_sim,'-','LineWidth',tratto2,'Color',colore1);
% title('Modulation Strategy','Interpreter','latex','FontSize',fontsize_plotting);
legend('$m_{31}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[-0.2 15.2]);
grid on
subplot 514
plot(t_tc_sim,gate_cmd_21_sim,'-','LineWidth',tratto2,'Color',colore1);
% title('Modulation Strategy','Interpreter','latex','FontSize',fontsize_plotting);
legend('$m_{21}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[-0.2 15.2]);
grid on
subplot 515
plot(t_tc_sim,gate_cmd_11_sim,'-','LineWidth',tratto2,'Color',colore1);
% title('Modulation Strategy','Interpreter','latex','FontSize',fontsize_plotting);
legend('$m_{11}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[-0.2 15.2]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('sim_results_fig_1','-depsc');
print('sim_results_fig_1','-dpdf');

figure;
subplot 511
plot(t_tc_sim,gate_cmd_52_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Modulation Strategy','Interpreter','latex','FontSize',fontsize_plotting);
legend('$m_{52}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[-0.2 15.2]);
grid on
subplot 512
plot(t_tc_sim,gate_cmd_42_sim,'-','LineWidth',tratto2,'Color',colore1);
% title('Modulation Strategy','Interpreter','latex','FontSize',fontsize_plotting);
legend('$m_{42}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[-0.2 15.2]);
grid on
subplot 513
plot(t_tc_sim,gate_cmd_32_sim,'-','LineWidth',tratto2,'Color',colore1);
% title('Modulation Strategy','Interpreter','latex','FontSize',fontsize_plotting);
legend('$m_{32}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[-0.2 15.2]);
grid on
subplot 514
plot(t_tc_sim,gate_cmd_22_sim,'-','LineWidth',tratto2,'Color',colore1);
% title('Modulation Strategy','Interpreter','latex','FontSize',fontsize_plotting);
legend('$m_{22}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[-0.2 15.2]);
grid on
subplot 515
plot(t_tc_sim,gate_cmd_12_sim,'-','LineWidth',tratto2,'Color',colore1);
% title('Modulation Strategy','Interpreter','latex','FontSize',fontsize_plotting);
legend('$m_{12}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[-0.2 15.2]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('sim_results_fig_2','-depsc');
print('sim_results_fig_2','-dpdf');

