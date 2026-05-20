close all
clc

clear
load ../sim_results/sim_results_7;

tratto1=1;
tratto2=2;
tratto3=3;

colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];

N1c = floor(glb_time.Nc/2);
N2c = floor(glb_time.Nc);
N3c = floor(glb_time.Nc - 5/glb_time.fPWM_AFE/glb_time.tc);

t1c = time_tc_sim(N1c);
t2c = time_tc_sim(N2c);

t3c = time_tc_sim(N3c);
t4c = time_tc_sim(N2c);

fontsize_title = 18;
fontsize_legend = 18;
fontsize_axis = 16;

%% figure(1)
figure(1);
subplot 211
plot(time_tc_sim,ig_abc_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,ig_abc_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,ig_abc_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
title('Grid Currents','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{g}^\mathrm{a}$','$i_\mathrm{g}^\mathrm{b}$','$i_\mathrm{g}^\mathrm{c}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
% xlabel('$Time - [s]$','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[-650 650]);
grid on
subplot 212
plot(time_tc_sim,ug_abc_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,ug_abc_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,ug_abc_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
title('Grid Voltages','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{g}^\mathrm{a}$','$u_\mathrm{g}^\mathrm{b}$','$u_\mathrm{g}^\mathrm{c}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[-500 500]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('grid_quantities','-depsc');
movefile('grid_quantities.eps', '../figures/sim_results_7');

%% figure(2)
figure(2);
subplot 211
plot(time_tc_sim,hwdata.afe.ibez * is_abc_mod1_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,hwdata.afe.ibez * is_abc_mod1_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,hwdata.afe.ibez * is_abc_mod1_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
title('Stack Currents','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{s}^\mathrm{a}$','$i_\mathrm{s}^\mathrm{b}$','$i_\mathrm{s}^\mathrm{c}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
% xlabel('$Time - [s]$','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[-650 650]);
grid on
subplot 212
plot(time_tc_sim, udc_dclink_mod1_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
title('AFE DClink Voltage','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{dc}^\mathrm{dc1}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[740 760]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('afe_stack_quantities','-depsc');
movefile('afe_stack_quantities.eps', '../figures/sim_results_7');

%% figure(3)
figure(3);
subplot 211
plot(time_tc_sim,i1_psfbc_transformer_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
title('Full Bridge output AC current','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{ac}^\mathrm{ac1}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
% xlabel('$Time - [s]$','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
set(gca,'ylim',[-1200 1200]);
grid on
subplot 212
plot(time_tc_sim, u1_psfbc_transformer_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
title('Full Bridge output AC voltage','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{ac}^\mathrm{ac1}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
set(gca,'ylim',[-950 950]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('fbc_quantities_1','-depsc');
movefile('fbc_quantities_1.eps', '../figures/sim_results_7');

%% figure(4)
figure(4);
subplot 211
plot(time_tc_sim,i2_psfbc_transformer_modA_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
title('HF-TR Secondary output AC current','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{ac}^\mathrm{ac2}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
% xlabel('$Time - [s]$','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
set(gca,'ylim',[-2500 14500]);
grid on
subplot 212
plot(time_tc_sim, u2_psfbc_transformer_modA_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
title('HF-TR Secondary output AC voltage','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{ac}^\mathrm{ac2}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
set(gca,'ylim',[-75 75]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('fbc_quantities_2','-depsc');
movefile('fbc_quantities_2.eps', '../figures/sim_results_7');

%% figure(5)
figure(5);
subplot 211
plot(time_tc_sim,psfbc_current_output_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
title('Output current','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{dc}^\mathrm{out}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
% xlabel('$Time - [s]$','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[11.75e3 12.25e3]);
grid on
subplot 212
plot(time_tc_sim, psfbc_voltage_output_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
title('Output voltage','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{dc}^\mathrm{out}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[24.5 25.5]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('output_quantities','-depsc');
movefile('output_quantities.eps', '../figures/sim_results_7');

%% figure(6)
figure(6);
subplot 211
plot(time_tc_sim,CFu_dc2_current_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
title('Output Capacitor current','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{ac}^\mathrm{Cdc2}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
% xlabel('$Time - [s]$','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
set(gca,'ylim',[-850 850]);
grid on
subplot 212
plot(time_tc_sim, icu_CFu_mod1_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
title('AFE Grid Filter Capacitor current','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{ac}^\mathrm{CFu}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[-125 125]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('capacitors_currents','-depsc');
movefile('capacitors_currents.eps', '../figures/sim_results_7');


%% figure(7)
figure(7);
subplot 211
plot(time_tc_sim,diode_rectifiers_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,diode_rectifiers_sim(:,3),'-','LineWidth',tratto3,'Color',colore2);
hold off
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
title('Diode Rectifier 1 and 2 Currents','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{d}^\mathrm{d1}$','$i_\mathrm{d}^\mathrm{d2}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
% xlabel('$Time - [s]$','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
set(gca,'ylim',[-2500 14500]);
grid on
subplot 212
plot(time_tc_sim,diode_rectifiers_sim(:,2),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,diode_rectifiers_sim(:,4),'-','LineWidth',tratto3,'Color',colore2);
hold off
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
title('Diode Rectifier 1 and 2 Voltages','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{d}^\mathrm{d1}$','$u_\mathrm{d}^\mathrm{d2}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
set(gca,'ylim',[-150 20]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('diode_rectifier','-depsc');
movefile('diode_rectifier.eps', '../figures/sim_results_7');

%% figure(8)
figure(8);
subplot 211
plot(time_tc_sim,full_bridge_igbt_data_sim(:,3),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,full_bridge_igbt_data_sim(:,11),'-','LineWidth',tratto3,'Color',colore2);
hold off
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
title('Full Bridge Device Currents','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{ed}^\mathrm{Q1}$','$i_\mathrm{ed}^\mathrm{Q2}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
% xlabel('$Time - [s]$','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
set(gca,'ylim',[-1000 1000]);
grid on
subplot 212
plot(time_tc_sim,full_bridge_igbt_data_sim(:,19),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,full_bridge_igbt_data_sim(:,27),'-','LineWidth',tratto3,'Color',colore2);
hold off
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
title('Full Bridge Device Currents','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{ed}^\mathrm{Q3}$','$i_\mathrm{ed}^\mathrm{Q4}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
set(gca,'ylim',[-1000 1000]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('full_bridge_devices_quantities','-depsc');
movefile('full_bridge_devices_quantities.eps', '../figures/sim_results_7');

%% figure(9)
figure(9);
subplot 211
plot(time_tc_sim,full_bridge_igbt_data_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,full_bridge_igbt_data_sim(:,6),'-','LineWidth',tratto3,'Color',colore2);
hold off
ylabel('Power Losses - [W]','Interpreter','latex','FontSize', fontsize_axis);
title('Full Bridge Device Power Losses','Interpreter','latex','FontSize',fontsize_title);
legend('$p_\mathrm{loss}^\mathrm{Q1}$','$p_\mathrm{loss}^\mathrm{D1}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
% xlabel('$Time - [s]$','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[0 800]);
grid on
subplot 212
plot(time_tc_sim,full_bridge_igbt_data_sim(:,9),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,full_bridge_igbt_data_sim(:,15),'-','LineWidth',tratto3,'Color',colore2);
hold off
ylabel('Power Losses - [W]','Interpreter','latex','FontSize', fontsize_axis);
title('Full Bridge Device Power Losses','Interpreter','latex','FontSize',fontsize_title);
legend('$p_\mathrm{loss}^\mathrm{Q2}$','$p_\mathrm{loss}^\mathrm{D2}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[0 800]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('full_bridge_devices_power_losses_1','-depsc');
movefile('full_bridge_devices_power_losses_1.eps', '../figures/sim_results_7');

%% figure(10)
figure(10);
subplot 211
plot(time_tc_sim,full_bridge_igbt_data_sim(:,17),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,full_bridge_igbt_data_sim(:,23),'-','LineWidth',tratto3,'Color',colore2);
hold off
ylabel('Power Losses - [W]','Interpreter','latex','FontSize', fontsize_axis);
title('Full Bridge Device Power Losses','Interpreter','latex','FontSize',fontsize_title);
legend('$p_\mathrm{loss}^\mathrm{Q3}$','$p_\mathrm{loss}^\mathrm{D3}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
% xlabel('$Time - [s]$','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[0 800]);
grid on
subplot 212
plot(time_tc_sim,full_bridge_igbt_data_sim(:,25),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,full_bridge_igbt_data_sim(:,31),'-','LineWidth',tratto3,'Color',colore2);
hold off
ylabel('Power Losses - [W]','Interpreter','latex','FontSize', fontsize_axis);
title('Full Bridge Device Power Losses','Interpreter','latex','FontSize',fontsize_title);
legend('$p_\mathrm{loss}^\mathrm{Q4}$','$p_\mathrm{loss}^\mathrm{D4}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[0 800]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('full_bridge_devices_power_losses_2','-depsc');
movefile('full_bridge_devices_power_losses_2.eps', '../figures/sim_results_7');