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

Q3_ploss = dab_inverter_1_device_data_sim(:,11);
Q3_voltage = dab_inverter_1_device_data_sim(:,12);
Q3_current = dab_inverter_1_device_data_sim(:,13);
ic_zvs_Q3 = dab_inverter_1_device_data_sim(:,23);
gate_cmd_Q3 = dab_inverter_1_device_data_sim(:,27);

Q4_ploss = dab_inverter_1_device_data_sim(:,16);
Q4_voltage = dab_inverter_1_device_data_sim(:,17);
Q4_current = dab_inverter_1_device_data_sim(:,18);
ic_zvs_Q4 = dab_inverter_1_device_data_sim(:,24);
gate_cmd_Q4 = dab_inverter_1_device_data_sim(:,28);

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

Q7_ploss = dab_inverter_2_device_data_sim(:,11);
Q7_voltage = dab_inverter_2_device_data_sim(:,12);
Q7_current = dab_inverter_2_device_data_sim(:,13);
ic_zvs_Q7 = dab_inverter_2_device_data_sim(:,23);
gate_cmd_Q7 = dab_inverter_2_device_data_sim(:,27);

Q8_ploss = dab_inverter_2_device_data_sim(:,16);
Q8_voltage = dab_inverter_2_device_data_sim(:,17);
Q8_current = dab_inverter_2_device_data_sim(:,18);
ic_zvs_Q8 = dab_inverter_2_device_data_sim(:,24);
gate_cmd_Q8 = dab_inverter_2_device_data_sim(:,28);

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
plot(time_tc_sim,Q3_current,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1000 1000]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,Q3_voltage,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[800 1650]);
hold off
title('DAB Q3-Mosfet: device current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{d}^{Q3}$','$u_{ds}^{Q3}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,ic_zvs_Q3,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[-400 0]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,gate_cmd_Q3,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[1200 1400]);
hold off
title('DAB Q5-Mosfet: snubber current and gate command','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{c}^{Q3}$','$u_{gs}^{Q3}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_Q3_zvs_data','-depsc');
movefile('dab_Q3_zvs_data.eps', 'figures');

figure(4);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,Q4_current,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1000 1000]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,Q4_voltage,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[800 1650]);
hold off
title('DAB Q4-Mosfet: device current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{d}^{Q4}$','$u_{ds}^{Q4}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,ic_zvs_Q4,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[-400 0]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,gate_cmd_Q4,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[1200 1400]);
hold off
title('DAB Q4-Mosfet: snubber current and gate command','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{c}^{Q4}$','$u_{gs}^{Q4}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_Q4_zvs_data','-depsc');
movefile('dab_Q4_zvs_data.eps', 'figures');


figure(5);
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

figure(6);
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


figure(7);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,Q7_current,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1000 1000]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,Q7_voltage,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[800 1650]);
hold off
title('DAB Q7-Mosfet: device current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{d}^{Q7}$','$u_{ds}^{Q7}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,ic_zvs_Q7,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[-400 0]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,gate_cmd_Q7,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[1200 1400]);
hold off
title('DAB Q7-Mosfet: snubber current and gate command','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{c}^{Q7}$','$u_{gs}^{Q7}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_Q7_zvs_data','-depsc');
movefile('dab_Q7_zvs_data.eps', 'figures');

figure(8);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,Q8_current,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1000 1000]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,Q8_voltage,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[800 1650]);
hold off
title('DAB Q8-Mosfet: device current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{d}^{Q8}$','$u_{ds}^{Q8}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,ic_zvs_Q8,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[-400 0]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,gate_cmd_Q8,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[1200 1400]);
hold off
title('DAB Q8-Mosfet: snubber current and gate command','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{c}^{Q8}$','$u_{gs}^{Q8}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_Q8_zvs_data','-depsc');
movefile('dab_Q8_zvs_data.eps', 'figures');


figure(9);
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,Q1_ploss,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_tc_sim,Q2_ploss,'--','LineWidth',tratto1,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1000 1000]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,Q3_ploss,'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(time_tc_sim,Q4_ploss,'--','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[800 1650]);
hold off
title('DAB full-bridge primary side: power losses','Interpreter','latex','FontSize',fontsize_plotting);
legend('$p_{loss}^{Q1}$','$p_{loss}^{Q2}$','$p_{loss}^{Q3}$','$p_{loss}^{Q4}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,Q5_ploss,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_tc_sim,Q6_ploss,'--','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
% set(gca,'ylim',[-400 0]);
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,Q7_ploss,'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(time_tc_sim,Q8_ploss,'--','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[1200 1400]);
hold off
title('DAB full-bridge secondary side: power losses','Interpreter','latex','FontSize',fontsize_plotting);
legend('$p_{loss}^{Q1}$','$p_{loss}^{Q2}$','$p_{loss}^{Q3}$','$p_{loss}^{Q4}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3c t4c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_ploss_data','-depsc');
movefile('dab_ploss_data.eps', 'figures');