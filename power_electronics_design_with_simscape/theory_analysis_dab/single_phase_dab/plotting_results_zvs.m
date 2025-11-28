clear all
close all
clc

load sim_result_1.mat;

tratto1=2.5;
tratto2=2.5;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
time_offset = 0.0;
t1c = time_tc_sim(end) - Nc*tc - time_offset;
t2c = time_tc_sim(end) - time_offset;
t3c = time_tc_sim(end) - Nc*tc/200 - time_offset;
t4c = time_tc_sim(end) - time_offset;
fontsize_plotting = 14;

Q1_ploss = dab_inverter_1_device_data_sim(:,1);
Q1_voltage = dab_inverter_1_device_data_sim(:,2);
Q1_current = dab_inverter_1_device_data_sim(:,3);
ic_zvs_Q1 = dab_inverter_1_device_data_sim(:,21);
gate_cmd_Q1 = dab_inverter_1_device_data_sim(:,25);

Q2_ploss = dab_inverter_1_device_data_sim(:,6);
Q2_voltage = dab_inverter_1_device_data_sim(:,7);
Q2_current = dab_inverter_1_device_data_sim(:,8);
ic_zvs_Q2 = dab_inverter_1_device_data_sim(:,22);
gate_cmd_Q2 = dab_inverter_1_device_data_sim(:,26);

Q5_ploss = dab_inverter_2_device_data_sim(:,1);
Q5_voltage = dab_inverter_2_device_data_sim(:,2);
Q5_current = dab_inverter_2_device_data_sim(:,3);
ic_zvs_Q5 = dab_inverter_2_device_data_sim(:,21);
gate_cmd_Q5 = dab_inverter_2_device_data_sim(:,25);

Q6_ploss = dab_inverter_2_device_data_sim(:,6);
Q6_voltage = dab_inverter_2_device_data_sim(:,7);
Q6_current = dab_inverter_2_device_data_sim(:,8);
ic_zvs_Q6 = dab_inverter_2_device_data_sim(:,22);
gate_cmd_Q6 = dab_inverter_2_device_data_sim(:,26);

figure(1);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,Q1_current,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1000 1000]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,Q1_voltage,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[800 1650]);
hold off
title('DAB Q1-Mosfet: device current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{d}^{Q1}$','$u_{ds}^{Q1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,ic_zvs_Q1,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[-400 0]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,gate_cmd_Q1,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[1200 1400]);
hold off
title('DAB Q1-Mosfet: snubber current and gate command','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{c}^{Q1}$','$u_{gs}^{Q1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_Q1_zvs_data','-depsc');
movefile('dab_Q1_zvs_data.eps', 'figures');

figure(2);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,Q2_current,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1000 1000]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,Q2_voltage,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[800 1650]);
hold off
title('DAB Q2-Mosfet: device current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{d}^{Q2}$','$u_{ds}^{Q2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,ic_zvs_Q2,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[-400 0]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,gate_cmd_Q2,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[1200 1400]);
hold off
title('DAB Q1-Mosfet: snubber current and gate command','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{c}^{Q2}$','$u_{gs}^{Q2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_Q2_zvs_data','-depsc');
movefile('dab_Q2_zvs_data.eps', 'figures');

figure(3);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,Q5_current,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1000 1000]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,Q5_voltage,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[800 1650]);
hold off
title('DAB Q5-Mosfet: device current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{d}^{Q5}$','$u_{ds}^{Q5}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,ic_zvs_Q5,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[-400 0]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,gate_cmd_Q5,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[1200 1400]);
hold off
title('DAB Q5-Mosfet: snubber current and gate command','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{c}^{Q5}$','$u_{gs}^{Q5}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_Q5_zvs_data','-depsc');
movefile('dab_Q5_zvs_data.eps', 'figures');

figure(4);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,Q6_current,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1000 1000]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,Q6_voltage,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[800 1650]);
hold off
title('DAB Q6-Mosfet: device current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{d}^{Q6}$','$u_{ds}^{Q6}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,ic_zvs_Q6,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[-400 0]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,gate_cmd_Q6,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[1200 1400]);
hold off
title('DAB Q5-Mosfet: snubber current and gate command','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{c}^{Q6}$','$u_{gs}^{Q6}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_Q6_zvs_data','-depsc');
movefile('dab_Q6_zvs_data.eps', 'figures');