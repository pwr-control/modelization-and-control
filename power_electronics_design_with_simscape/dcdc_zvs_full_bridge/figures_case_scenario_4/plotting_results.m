clear all
close all
clc
load sim_results_4;

tratto1=2.5;
tratto2=2.5;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = time_tc_sim(end) - Nc*tc/1250;
t2c = time_tc_sim(end);
t1s = time_ts_dab_sim(end) - Ns_dab*ts_dab;
t2s = time_ts_dab_sim(end);
t3s = time_ts_dab_sim(end) - Ns_dab*ts_dab/10;
t4s = time_ts_dab_sim(end);
t5s = time_ts_dab_sim(end) - Ns_dab*ts_dab/20;
t6s = time_ts_dab_sim(end);
t3c = time_tc_sim(end) - Nc*tc/1000;
t4c = time_tc_sim(end);
t5c = time_tc_sim(end) - Nc*tc/2000;
t6c = time_tc_sim(end);

fontsize_plotting = 14;

figure(1);
subplot 311
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,i1_dab_transformer_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,u1_dab_transformer_modA_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[800 1650]);
hold off
title('Transformer primary current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{1}^{ac}$','$u_{1}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
subplot 312
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,i2_dab_transformer_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[-400 0]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,u2_dab_transformer_modA_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Transformer secondary 1 current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{2}^{ac}$','$u_{2}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
subplot 313
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,i3_dab_transformer_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[-400 0]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,u3_dab_transformer_modA_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[1200 1400]);
hold off
title('Transformer secondary 2 current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{3}^{ac}$','$u_{3}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('transformer_current_voltage','-depsc');
% movefile('transformer_current_voltage.eps', 'figures');


figure(2);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,current_battery_1_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[240 250]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,voltage_battery_1_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('DC input current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{1}^{dc}$','$u_{1}^{dc}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
% set(gca,'ylim',[760 770]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,-current_battery_2_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[295 305]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,voltage_battery_2_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[600 610]);
hold off
title('DC output current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{2}^{dc}$','$u_{2}^{dc}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dc_inout_voltage_current','-depsc');
% movefile('dc_inout_voltage_current.eps', 'figures');


figure(3);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,2),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-20 850]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,21),'-','LineWidth',tratto1,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Full-bridge: Q1 Uce and Uge','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ce}^{Q_1}$','$u_{ge}^{Q_1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[-20 25]);
grid on
subplot 212
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,3),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1600 1000]);
legend('$i_{Q_1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
title('Full-bridge: Q1 current','Interpreter','latex','FontSize',fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('full_bridge_Q1','-depsc');
% movefile('full_bridge_Q1.eps', 'figures');

figure(4);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,7),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-20 850]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,22),'-','LineWidth',tratto1,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Full-bridge: Q2 Uce and Uge','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ce}^{Q_2}$','$u_{ge}^{Q_2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[-20 25]);
grid on
subplot 212
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,8),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1600 1000]);
legend('$i_{Q_2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
title('Full-bridge: Q2 current','Interpreter','latex','FontSize',fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('full_bridge_Q2','-depsc');
% movefile('full_bridge_Q2.eps', 'figures');


figure(5);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,12),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-20 850]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,23),'-','LineWidth',tratto1,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Full-bridge: Q3 Uce and Uge','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ce}^{Q_3}$','$u_{ge}^{Q_3}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[-20 25]);
grid on
subplot 212
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,13),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1600 1000]);
legend('$i_{Q_3}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
title('Full-bridge: Q3 current','Interpreter','latex','FontSize',fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('full_bridge_Q3','-depsc');
% movefile('full_bridge_Q3.eps', 'figures');
% 
figure(6);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,17),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-20 850]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,24),'-','LineWidth',tratto1,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Full-bridge: Q4 Uce and Uge','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ce}^{Q_4}$','$u_{ge}^{Q_4}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[-20 25]);
grid on
subplot 212
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,18),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1600 1000]);
legend('$i_{Q_4}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
title('Full-bridge: Q4 current','Interpreter','latex','FontSize',fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('full_bridge_Q4','-depsc');
% movefile('full_bridge_Q4.eps', 'figures');


figure(7);
subplot 211
yyaxis left;
ax = gca;
ax.YColor = [0 0 0]; 
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[500 700]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,6),'-','LineWidth',tratto1,'Color',colore2);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
grid on
legend('$p_{Q_1}$','$p_{Q_1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
% set(gca,'ylim',[500 700]);
title('Full-bridge: Q1/Q2 power loss','Interpreter','latex','FontSize',fontsize_plotting);
grid on
subplot 212
yyaxis left;
ax = gca;
ax.YColor = [0 0 0]; 
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,11),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[500 700]);
set(gca,'xlim',[t1s t2s]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,full_bridge_inverter_1_device_data_sim(:,16),'-','LineWidth',tratto1,'Color',colore2);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
grid on
legend('$p_{Q_3}$','$p_{Q_4}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
% set(gca,'ylim',[500 700]);
title('Full-bridge: Q3/Q4 power loss','Interpreter','latex','FontSize',fontsize_plotting);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('full_bridge_device_losses','-depsc');
% movefile('full_bridge_device_losses.eps', 'figures')


