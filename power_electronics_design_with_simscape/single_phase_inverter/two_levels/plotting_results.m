% clear all
close all
clc

tratto1=2.5;
tratto2=2.5;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = time_tc_sim(end) - Nc*tc/2000;
t2c = time_tc_sim(end);
t1s = time_tc_sim(end) - (Ns_inv*ts_inv/10);
t2s = time_tc_sim(end);
t3s = time_tc_sim(end) - (Ns_inv*ts_inv/10);
t4s = time_tc_sim(end);
t5s = time_tc_sim(end) - (Ns_inv*ts_inv/20);
t6s = time_tc_sim(end);
t3c = time_tc_sim(end) - (Nc*tc/1000);
t4c = time_tc_sim(end);
t5c = time_tc_sim(end) - (Nc*tc/2000);
t6c = time_tc_sim(end);
fontsize_plotting = 14;

figure(1);
subplot 211
plot(time_tc_sim,ig_abc_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_tc_sim,ig_abc_sim(:,2),'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(time_tc_sim,ig_abc_sim(:,3),'-','LineWidth',tratto1,'Color',colore3);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Grid Phase Currents','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{r}^{g}$','$i_{s}^{g}$','$i_{t}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 212
plot(time_tc_sim,ug_abc_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_tc_sim,ug_abc_sim(:,2),'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(time_tc_sim,ug_abc_sim(:,3),'-','LineWidth',tratto1,'Color',colore3);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Grid Phase Voltages','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{r}^{g}$','$u_{s}^{g}$','$u_{t}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('grid_quantities','-depsc');
movefile('grid_quantities.eps', 'figures');

figure(2);
subplot 211
plot(time_tc_sim,i_line_abc_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_tc_sim,i_line_abc_sim(:,2),'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(time_tc_sim,i_line_abc_sim(:,3),'-','LineWidth',tratto1,'Color',colore3);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Line Phase Currents','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{r}^{l}$','$i_{s}^{l}$','$i_{t}^{l}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 212
plot(time_tc_sim,u_line_abc_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_tc_sim,u_line_abc_sim(:,2),'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(time_tc_sim,u_line_abc_sim(:,3),'-','LineWidth',tratto1,'Color',colore3);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Line Phase Voltages','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{r}^{l}$','$u_{s}^{l}$','$u_{t}^{l}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('line_quantities','-depsc');
movefile('line_quantities.eps', 'figures');