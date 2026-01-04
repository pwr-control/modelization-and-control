close all
clc

clear all
load sim_results_80Hz_10kHz_1;

tratto1=1;
tratto2=2;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2b = [0.35 0.35 0.35];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];

N1c = floor(Nc/50/(frequency_set/f_grid));
N2c = floor(Nc);
t1c = time_tc_sim(N2c) - N1c*tc;
t2c = time_tc_sim(N2c);

N3c = floor(Nc/1000/(frequency_set/f_grid));
N4c = floor(Nc);
t3c = time_tc_sim(N4c) - N3c*tc;
t4c = time_tc_sim(N4c);

N1s = floor(Nc/10/(frequency_set/f_grid));
N2s = floor(Nc);
t1s = time_tc_sim(N2s) - N1s*tc;
t2s = time_tc_sim(N2s);

N3s = floor(Nc/10);
N4s = floor(Nc);
t3s = time_tc_sim(N4s) - N3s*tc;
t4s = time_tc_sim(N4s);

N5s = floor(Nc/40);
N6s = floor(Nc);
t5s = time_tc_sim(N6s) - N5s*tc;
t6s = time_tc_sim(N6s);

fontsize_plotting = 14;

%% figure(1)
figure(1);
subplot 211
plot(time_tc_sim,ig_abc_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,ig_abc_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,ig_abc_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Grid Phase Currents');
legend('$i_{r}^{g}$','$i_{s}^{g}$','$i_{t}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
subplot 212
plot(time_tc_sim,ug_abc_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,ug_abc_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,ug_abc_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Grid Phase Voltages');
legend('$u_{r}^{g}$','$u_{s}^{g}$','$u_{t}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('grid_quantities','-depsc');
movefile('grid_quantities.eps', 'figures/sim_results_80Hz_10kHz_1');


%% figure (2)
% figure(2);
% subplot 211
% plot(time_tc_sim,i_line_abc_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
% hold on
% plot(time_tc_sim,i_line_abc_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
% hold on
% plot(time_tc_sim,i_line_abc_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
% hold off
% ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% title('Line Phase Currents');
% legend('$i_{r}^{l}$','$i_{s}^{l}$','$i_{t}^{l}$','Location','northwestoutside',...
%     'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'xlim',[t3s t4s]);
% grid on
% subplot 212
% plot(time_tc_sim,u_line_abc_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
% hold on
% plot(time_tc_sim,u_line_abc_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
% hold on
% plot(time_tc_sim,u_line_abc_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
% hold off
% ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% title('Line Phase Voltages');
% legend('$u_{r}^{l}$','$u_{s}^{l}$','$u_{t}^{l}$','Location','northwestoutside',...
%     'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'xlim',[t3s t4s]);
% grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('line_quantities','-depsc');
% movefile('line_quantities.eps', 'figures/sim_results_80Hz_10kHz_1');

% %% figure (3)
% figure(3);
% subplot 211
% plot(time_tc_sim,inverter_current_output_sim,'-','LineWidth',tratto3,'Color',colore1);
% ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% title('Inverter Output Current');
% legend('$i_{out}^{inv}$','Location','northwestoutside',...
%     'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'xlim',[t1s t2s]);
% grid on
% subplot 212
% plot(time_tc_sim,inverter_voltage_output_sim,'-','LineWidth',tratto2,'Color',colore1);
% ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% title('Inverter Output Voltage');
% legend('$u_{out}^{inv}$','Location','northwestoutside',...
%     'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'xlim',[t1s t2s]);
% grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('inverter_outputs','-depsc');
% movefile('inverter_outputs.eps', 'figures/sim_results_80Hz_10kHz_1');

% %% figure (4)
% x1 = time_tc_sim(N1s:N2s);
% y1 = inverter_current_output_sim(N1s:N2s);
% x2 = time_tc_sim(N3c:N4c);
% y2 = inverter_current_output_sim(N3c:N4c);
% figure(4);
% plot(x1,y1,'-','LineWidth',tratto3,'Color',colore1);
% ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% title('Inverter Output Current');
% legend('$i_{out}^{inv}$','Location','northwestoutside',...
%     'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
% grid on
% set(gca,'xlim',[t1s t2s]);
% ax2 = axes('Position',[0.7,0.735,0.225,0.225]);
% plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
% grid on
% set(gca,'xlim',[t3c t4c]);
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('inverter_output_current','-depsc');
% movefile('inverter_output_current.eps', 'figures/sim_results_80Hz_10kHz_1');
% 
% %% figure (5)
% x1 = time_tc_sim(N1s:N2s);
% y1 = inverter_voltage_output_sim(N1s:N2s);
% x2 = time_tc_sim(N1c:N2c);
% y2 = inverter_voltage_output_sim(N1c:N2c);
% figure(5);
% plot(x1,y1,'-','LineWidth',tratto2,'Color',colore1);
% ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% title('Inverter Output Current');
% legend('$u_{out}^{inv}$','Location','northwestoutside',...
%     'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
% grid on
% set(gca,'xlim',[t1s t2s]);
% ax2 = axes('Position',[0.7,0.735,0.225,0.225]);
% plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
% grid on
% set(gca,'xlim',[t1c t2c]);
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('inverter_output_voltage','-depsc');
% movefile('inverter_output_voltage.eps', 'figures/sim_results_80Hz_10kHz_1');

%% figure (4)
figure(4);
subplot 211
x1 = time_tc_sim(N1s:N2s);
y1 = inverter_current_output_sim(N1s:N2s);
x2 = time_tc_sim(N3c:N4c);
y2 = inverter_current_output_sim(N3c:N4c);
plot(x1,y1,'-','LineWidth',tratto3,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter Output Current');
legend('$i_{out}^{inv}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
grid on
set(gca,'xlim',[t1s t2s]);
ax2 = axes('Position',[0.7,0.735,0.225,0.225]);
plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
grid on
set(gca,'xlim',[t1c t2c]);
subplot 212
x1 = time_tc_sim(N1s:N2s);
y1 = inverter_voltage_output_sim(N1s:N2s);
x2 = time_tc_sim(N1c:N2c);
y2 = inverter_voltage_output_sim(N1c:N2c);
plot(x1,y1,'-','LineWidth',tratto2,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter Output Current');
legend('$u_{out}^{inv}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
grid on
set(gca,'xlim',[t1s t2s]);
ax2 = axes('Position',[0.7,0.275,0.225,0.225]);
plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
grid on
set(gca,'xlim',[t1c t2c]);
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('inverter_output_current','-depsc');
movefile('inverter_output_current.eps', 'figures/sim_results_80Hz_10kHz_1');



%% figure (6)
figure(6);
subplot 211
plot(time_tc_sim,mosfet_ploss_invQ1_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter MOSFET Q1 Power Losses');
legend('$p_{Q1}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5s t6s]);
grid on
subplot 212
plot(time_tc_sim,mosfet_JH_temp_invQ1_sim,'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,mosfet_HA_temp_invQ1_sim,'-','LineWidth',tratto3,'Color',colore2);
hold off
ylabel('$T/degC$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter MOSFET Q1 Temperatures');
legend('$T_{JH}$','$T_{HA}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5s t6s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('inverter_Q1_power_losses','-depsc');
movefile('inverter_Q1_power_losses.eps', 'figures/sim_results_80Hz_10kHz_1');

%% figure (7)
figure(7);
subplot 211
plot(time_tc_sim,mosfet_ploss_invQ2_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter MOSFET Q2 Power Losses');
legend('$p_{Q2}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5s t6s]);
grid on
subplot 212
plot(time_tc_sim,mosfet_JH_temp_invQ2_sim,'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,mosfet_HA_temp_invQ2_sim,'-','LineWidth',tratto3,'Color',colore2);
hold off
ylabel('$T/degC$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter MOSFET Q2 Temperatures');
legend('$T_{JH}$','$T_{HA}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5s t6s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('inverter_Q2_power_losses','-depsc');
movefile('inverter_Q2_power_losses.eps', 'figures/sim_results_80Hz_10kHz_1');


%% figure (8)
% x1 = time_tc_sim(N1s:N2s);
% y1 = mosfet_current_invQ1_sim(N1s:N2s);
% x2 = time_tc_sim(N1c:N2c);
% y2 = mosfet_current_invQ1_sim(N1c:N2c);
% figure(8);
% plot(x1,y1,'-','LineWidth',tratto2,'Color',colore1);
% ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% title('Inverter MOSFET Q1 Current');
% legend('$i_{Q1}^{inv}$','Location','northwestoutside',...
%     'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
% grid on
% set(gca,'xlim',[t1s t2s]);
% ax2 = axes('Position',[0.7,0.735,0.225,0.225]);
% plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
% grid on
% set(gca,'xlim',[t1c t2c]);
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('inverter_mosfet_Q1_current','-depsc');
% movefile('inverter_mosfet_Q1_current.eps', 'figures/sim_results_80Hz_10kHz_1');

%% figure (9)
% x1 = time_tc_sim(N1s:N2s);
% y1 = mosfet_voltage_invQ1_sim(N1s:N2s);
% x2 = time_tc_sim(N1c:N2c);
% y2 = mosfet_voltage_invQ1_sim(N1c:N2c);
% figure(9);
% plot(x1,y1,'-','LineWidth',tratto2,'Color',colore1);
% ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% title('Inverter MOSFET Q1 Voltage');
% legend('$u_{Q1}^{inv}$','Location','northwestoutside',...
%     'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
% grid on
% set(gca,'xlim',[t1s t2s]);
% ax2 = axes('Position',[0.7,0.735,0.225,0.225]);
% plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
% grid on
% set(gca,'xlim',[t1c t2c]);
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('inverter_mosfet_Q1_voltage','-depsc');
% movefile('inverter_mosfet_Q1_voltage.eps', 'figures/sim_results_80Hz_10kHz_1');


%% figure (10)
x1 = time_tc_sim(N1s:N2s);
y1 = mosfet_current_invQ1_sim(N1s:N2s);
x2 = time_tc_sim(N1c:N2c);
y2 = mosfet_current_invQ1_sim(N1c:N2c);
figure(10);
subplot 211
plot(x1,y1,'-','LineWidth',tratto2,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter MOSFET Q1 Current');
legend('$i_{Q1}^{inv}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
grid on
set(gca,'xlim',[t1s t2s]);
ax2 = axes('Position',[0.7,0.735,0.225,0.225]);
plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
grid on
set(gca,'xlim',[t3c t4c]);
x1 = time_tc_sim(N1s:N2s);
y1 = mosfet_voltage_invQ1_sim(N1s:N2s);
x2 = time_tc_sim(N1c:N2c);
y2 = mosfet_voltage_invQ1_sim(N1c:N2c);
subplot 212
plot(x1,y1,'-','LineWidth',tratto2,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter MOSFET Q1 Voltage');
legend('$u_{Q1}^{inv}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
grid on
set(gca,'xlim',[t1s t2s]);
ax2 = axes('Position',[0.7,0.275,0.225,0.225]);
plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
grid on
set(gca,'xlim',[t3c t4c]);
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('inverter_mosfet_Q1_iu','-depsc');
movefile('inverter_mosfet_Q1_iu.eps', 'figures/sim_results_80Hz_10kHz_1');

%% figure (11)
x1 = time_tc_sim(N1s:N2s);
y1 = dclink_voltage_sim(N1s:N2s);
x2 = time_tc_sim(N1c:N2c);
y2 = dclink_voltage_sim(N1c:N2c);
figure(11);
subplot 211
plot(x1,y1,'-','LineWidth',tratto2,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
title('DClink Voltage');
legend('$u_{dc}^{C}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
grid on
set(gca,'xlim',[t1s t2s]);
ax2 = axes('Position',[0.7,0.735,0.225,0.225]);
plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
grid on
set(gca,'xlim',[t1c t2c]);
x1 = time_tc_sim(N1s:N2s);
y1 = CFi_dc_current_sim(N1s:N2s);
x2 = time_tc_sim(N1c:N2c);
y2 = CFi_dc_current_sim(N1c:N2c);
subplot 212
plot(x1,y1,'-','LineWidth',tratto2,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('DClink Capacitor Current');
legend('$i_{ac}^{C}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
grid on
set(gca,'xlim',[t1s t2s]);
ax2 = axes('Position',[0.7,0.275,0.225,0.225]);
plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
grid on
set(gca,'xlim',[t1c t2c]);
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dclink_ui','-depsc');
movefile('dclink_ui.eps', 'figures/sim_results_80Hz_10kHz_1');


%% figure (12)
figure(12);
subplot 311
plot(time_tc_sim,-diode_bridge_rectifier_data_sim(:,2),'-','LineWidth',tratto3,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Diode Rectifier Current');
legend('$i_{d}^{r}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
subplot 312
plot(time_tc_sim,diode_bridge_rectifier_data_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
title('Diode Rectifier Voltage');
legend('$u_{d}^{r}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
subplot 313
plot(time_tc_sim,diode_bridge_rectifier_data_sim(:,5),'-','LineWidth',tratto3,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
title('Diode Rectifier Power Loss');
legend('$p_{d}^{r}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('diode_rectifier','-depsc');
movefile('diode_rectifier.eps', 'figures/sim_results_80Hz_10kHz_1');

%% figure (13)
figure(13);
subplot 211
plot(time_tc_sim,grid_metering_sim(:,4)./1e3,'-','LineWidth',tratto3,'Color',colore1);
ylabel('$p/kW$','Interpreter','latex','FontSize', fontsize_plotting);
title('Grid Active Power');
legend('$p_{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5s t6s]);
grid on
subplot 212
plot(time_tc_sim,grid_metering_sim(:,1)./1e3,'-','LineWidth',tratto3,'Color',colore1);
ylabel('$q/kVAr$','Interpreter','latex','FontSize', fontsize_plotting);
title('Grid Reactive Power');
legend('$q_{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5s t6s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('grid_power','-depsc');
movefile('grid_power.eps', 'figures/sim_results_80Hz_10kHz_1');

%% figure (14)
figure(14);
subplot 211
x2 = time_ts_inv_sim;
y2 = id_pu_sim;
plot(time_ts_inv_sim,id_pu_ref_sim,'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_ts_inv_sim,id_pu_sim,'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_ts_inv_sim,ud_ctrl_out_sim,'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('$pu$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter Tracking Performance D-axis');
legend('$i_{d}^{ref}$','$i_{d}$','$u_{d}^{ctrl}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[time_ts_inv_sim(1) time_ts_inv_sim(end)]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(3); chH(2); chH(1)])
ax2 = axes('Position',[0.7,0.735,0.225,0.225]);
plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore2b);
grid on
set(gca,'xlim',[t5s t6s]);
subplot 212
y2 = iq_pu_sim;
plot(time_ts_inv_sim, iq_pu_ref_sim,'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_ts_inv_sim, iq_pu_sim,'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_ts_inv_sim, uq_ctrl_out_sim,'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('$pu$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter Tracking Performance Q-axis');
legend('$i_{q}^{ref}$','$i_{q}$','$u_{q}^{ctrl}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[time_ts_inv_sim(1) time_ts_inv_sim(end)]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(3); chH(2); chH(1)])
ax2 = axes('Position',[0.7,0.275,0.225,0.225]);
plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore2b);
grid on
set(gca,'xlim',[t5s t6s]);
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('inverter_current_control_1','-depsc');
movefile('inverter_current_control_1.eps', 'figures/sim_results_80Hz_10kHz_1');

%% figure (15)
figure(15);
subplot 211
plot(time_ts_inv_sim, i_alpha_pu_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('$pu$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter Output Current \alpha');
legend('$i_{\alpha}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
set(gca,'ylim',[-1.25 1.25]);
grid on
subplot 212
plot(time_ts_inv_sim, i_beta_pu_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('$pu$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter Output Current \beta');
legend('$i_{\beta}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
set(gca,'ylim',[-1.25 1.25]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('inverter_current_control_2','-depsc');
movefile('inverter_current_control_2.eps', 'figures/sim_results_80Hz_10kHz_1');

%% spectrum phase grid R
N1=Nc-floor(200e-3/tc);
N2=Nc;
signal = ig_abc_sim(N1:N2,1);
time = time_tc_sim(N1:N2);

sig_fft = signal;
Nfft = length(sig_fft)-1;
u1 = Nfft*tc;
f_sig=fft(sig_fft,Nfft);
Xrange=[f_sig(1)/Nfft f_sig(2:Nfft/2)'/(Nfft/2)];
freq=[0:1/u1:Nfft/2/u1-1/u1]';

figure; 
subplot 211
plot(time,signal,'-','LineWidth',tratto2,'Color',colore1);
title('Grid Phase R Current')
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$i_{r}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
legend
set(gca,'xlim',[time(1) time(end)]);
grid on
subplot 212
plot(freq,abs(Xrange),'-','LineWidth',tratto2,'Color',colore1);
xlim([10 850]);
grid;
xlabel('$f/Hz$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$F(i_{r}^{g})$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
title('Grid Phase R Current Spectrum')
print('spectrum_grid_current_r','-depsc');
movefile('spectrum_grid_current_r.eps', 'figures/sim_results_80Hz_10kHz_1');

%% spectrum phase grid S
signal = ig_abc_sim(N1:N2,2);

sig_fft = signal;
Nfft = length(sig_fft)-1;
u1 = Nfft*tc;
f_sig=fft(sig_fft,Nfft);
Xrange=[f_sig(1)/Nfft f_sig(2:Nfft/2)'/(Nfft/2)];
freq=[0:1/u1:Nfft/2/u1-1/u1]';

figure; 
subplot 211
plot(time,signal,'-','LineWidth',tratto2,'Color',colore1);
title('Grid Phase S Current')
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$i_{s}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
legend
set(gca,'xlim',[time(1) time(end)]);
grid on
subplot 212
plot(freq,abs(Xrange),'-','LineWidth',tratto2,'Color',colore1);
xlim([10 850]);
grid;
xlabel('$f/Hz$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$F(i_{s}^{g})$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
title('Grid Phase S Current Spectrum')
print('spectrum_grid_current_s','-depsc');
movefile('spectrum_grid_current_s.eps', 'figures/sim_results_80Hz_10kHz_1');

%% spectrum phase grid T
signal = ig_abc_sim(N1:N2,3);

sig_fft = signal;
Nfft = length(sig_fft)-1;
u1 = Nfft*tc;
f_sig=fft(sig_fft,Nfft);
Xrange=[f_sig(1)/Nfft f_sig(2:Nfft/2)'/(Nfft/2)];
freq=[0:1/u1:Nfft/2/u1-1/u1]';

figure; 
subplot 211
plot(time,signal,'-','LineWidth',tratto2,'Color',colore1);
title('Grid Phase T Current')
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$i_{t}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
legend
set(gca,'xlim',[time(1) time(end)]);
grid on
subplot 212
plot(freq,abs(Xrange),'-','LineWidth',tratto2,'Color',colore1);
xlim([10 850]);
grid;
xlabel('$f/Hz$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$F(i_{s}^{g})$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
title('Grid Phase T Current Spectrum')
print('spectrum_grid_current_t','-depsc');
movefile('spectrum_grid_current_t.eps', 'figures/sim_results_80Hz_10kHz_1');