clear all
close all
clc

load sim_results_1.mat

tratto1=2.25;
tratto2=2.25;
tratto3=2.25;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = time(end) - Nc*tc;
tc_end = time(end);
fontsize_plotting = 12;


figure;
subplot 211
plot(time,i0_status_quo_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(time,i0_status_with_db_mod_sim,'-','LineWidth',tratto2,'Color',colore3);
hold off
title('DriverBoard Output Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{0}^{sq}$','$i_{0}^{dbm}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-6 6]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(time,u_out_status_quo_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(time,u_out_with_db_mod_sim,'-','LineWidth',tratto2,'Color',colore3);
hold off
title('OPA350 Output Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{out}^{sq}$','$u_{out}^{dbm}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('ntc_conversion_1','-depsc');
% print('sim_results_fig_1','-djpeg');

load sim_results_2.mat

figure;
subplot 211
plot(time,i0_status_quo_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(time,i0_status_with_db_mod_sim,'-','LineWidth',tratto2,'Color',colore3);
hold off
title('DriverBoard Output Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{0}^{sq}$','$i_{0}^{dbm}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-6 6]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(time,u_out_status_quo_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(time,u_out_with_db_mod_sim,'-','LineWidth',tratto2,'Color',colore3);
hold off
title('OPA350 Output Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{out}^{sq}$','$u_{out}^{dbm}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('ntc_conversion_2','-depsc');
% print('sim_results_fig_1','-djpeg');


