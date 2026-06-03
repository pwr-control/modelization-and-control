close all
clc

tratto1=1;
tratto2=2;
tratto3=2;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = t_tc_sim(end) - Nc*tc;
tc_end = t_tc_sim(end);
fontsize_plotting = 12;
Vfc1_base = Udc/5;
Vfc2_base = 2*Vfc1_base;
Vfc3_base = 3*Vfc1_base;
Vfc4_base = 4*Vfc1_base;
DVfc = 100;


figure;
subplot 211
plot(t_tc_sim,inverter_current_output_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Output Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{out}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-6 6]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,inverter_voltage_output_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Output Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{out}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('output_current_voltage','-depsc');
% print('sim_results_fig_1','-djpeg');

figure;
subplot 211
plot(t_tc_sim,inverter_current_stack_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Stack Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{s}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-6 6]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,inverter_voltage_stack_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Stack Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{s}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_1','-depsc');
% print('sim_results_fig_1','-djpeg');
print('stack_current_voltage','-depsc');

figure;
plot(t_tc_sim,inverter_output_filter_capacitor_current_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Output Filter Capacitor Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{CFu}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('output_filter_capacitor_current','-depsc');
% print('sim_results_fig_1','-djpeg');




figure;
subplot 211
plot(t_tc_sim,mosfet_51_power_losses_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,mosfet_41_power_losses_sim,'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,mosfet_31_power_losses_sim,'-','LineWidth',tratto1,'Color',colore3);
hold on
plot(t_tc_sim,mosfet_21_power_losses_sim,'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(t_tc_sim,mosfet_11_power_losses_sim,'-','LineWidth',tratto3,'Color',colore2);
hold off
title('Mosfet Power Losses','Interpreter','latex','FontSize',fontsize_plotting);
legend('$p_{mos}^{51}$','$p_{mos}^{41}$','$p_{mos}^{31}$','$p_{mos}^{21}$','$p_{mos}^{11}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[1.75 2.5]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_ts_sim,temperature_mosfet_51_JH_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_ts_sim,temperature_mosfet_51_HA_sim,'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(t_ts_sim,temperature_mosfet_51_HA_sim+temperature_mosfet_51_JH_sim+Tambient,'-','LineWidth',tratto2,'Color',colore3);
hold off
title('Mosfet Junction and HeatSink Temperature','Interpreter','latex','FontSize',fontsize_plotting);
legend('$T_{JH}$','$T_{HA}$','$T_{J}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$T/Celsius$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[0 120]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('mosfet_51_power_losses','-depsc');
% print('sim_results_fig_1','-djpeg');

figure;
subplot 311
plot(t_ts_sim,Pout_avg_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Output Power','Interpreter','latex','FontSize',fontsize_plotting);
legend('$P_{out}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[800 900]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 312
plot(t_ts_sim,vout_rms_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Output Voltage and Current in RMS','Interpreter','latex','FontSize',fontsize_plotting);
legend('$U_{out}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$U/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[250 300]);
grid on
subplot 313
plot(t_ts_sim,iout_rms_sim,'-','LineWidth',tratto2,'Color',colore2);
title('Output Voltage and Current in RMS','Interpreter','latex','FontSize',fontsize_plotting);
legend('$I_{out}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$I/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
set(gca,'ylim',[2.5 3.5]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_1','-depsc');
% print('sim_results_fig_1','-djpeg');

figure;
plot(t_tc_sim,current_hf_capacitor_1_sim,'-','LineWidth',tratto2,'Color',colore1);
title('High Frequecy Capacitor Stage 1 Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{Chf}^1$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-7.5 7.5]);
% set(gca,'xlim',[t1c tc_end]);
set(gca,'xlim',[1.246-0.01 1.246]);
grid on
print('current_hf_capacitor_1_sim','-depsc');

figure;
subplot 211
plot(t_tc_sim,voltage_flying_capacitor_1_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 1 Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Cfc}^1$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[Vfc1_base-DVfc Vfc1_base+DVfc]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,current_flying_capacitor_1_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 1 Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{Cfc}^1$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-7.5 7.5]);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_1','-depsc');
% print('sim_results_fig_1','-djpeg');
print('flying_capacitor_1_sim','-depsc');


figure;
subplot 211
plot(t_tc_sim,voltage_flying_capacitor_2_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 2 Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Cfc}^2$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[Vfc1_base-DVfc Vfc1_base+DVfc]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,current_flying_capacitor_2_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 2 Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{Cfc}^2$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-7.5 7.5]);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_1','-depsc');
% print('sim_results_fig_1','-djpeg');
print('flying_capacitor_2_sim','-depsc');


figure;
subplot 211
plot(t_tc_sim,voltage_flying_capacitor_3_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 3 Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Cfc}^3$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[Vfc2_base-DVfc Vfc2_base+DVfc]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,current_flying_capacitor_3_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 3 Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{Cfc}^3$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-7.5 7.5]);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_1','-depsc');
% print('sim_results_fig_1','-djpeg');
print('flying_capacitor_3_sim','-depsc');

figure;
subplot 211
plot(t_tc_sim,voltage_flying_capacitor_4_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 4 Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Cfc}^4$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[Vfc2_base-DVfc Vfc2_base+DVfc]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,current_flying_capacitor_4_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 4 Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{Cfc}^4$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-7.5 7.5]);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_1','-depsc');
% print('sim_results_fig_1','-djpeg');
print('flying_capacitor_4_sim','-depsc');

figure;
subplot 211
plot(t_tc_sim,voltage_flying_capacitor_51_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 5-1 Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Cfc}^{51}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[Vfc3_base-DVfc Vfc3_base+DVfc]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,current_flying_capacitor_51_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 5-1 Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{Cfc}^{51}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-7.5 7.5]);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_1','-depsc');
% print('sim_results_fig_1','-djpeg');
print('flying_capacitor_51_sim','-depsc');

figure;
subplot 211
plot(t_tc_sim,voltage_flying_capacitor_52_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 5-2 Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Cfc}^{52}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[Vfc3_base-DVfc Vfc3_base+DVfc]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,current_flying_capacitor_52_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 5-2 Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{Cfc}^{52}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-7.5 7.5]);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_1','-depsc');
% print('sim_results_fig_1','-djpeg');
print('flying_capacitor_52_sim','-depsc');


figure;
subplot 311
plot(t_tc_sim,voltage_flying_capacitor_51_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 5-1 Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Cfc}^{51}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[Vfc3_base-DVfc Vfc3_base+DVfc]);
set(gca,'xlim',[t1c tc_end]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 312
plot(t_tc_sim,voltage_flying_capacitor_52_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 5-2 Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Cfc}^{52}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[Vfc3_base-DVfc Vfc3_base+DVfc]);
set(gca,'xlim',[t1c tc_end]);
grid on
subplot 313
plot(t_tc_sim,voltage_flying_capacitor_5_sim,'-','LineWidth',tratto2,'Color',colore1);
title('Flying Capacitor Stage 5-1 Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Cfc}^{5}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[Vfc3_base-DVfc Vfc3_base+DVfc]);
set(gca,'xlim',[t1c tc_end]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_1','-depsc');
% print('sim_results_fig_1','-djpeg');
print('flying_capacitor_5_sim','-depsc');

figure;
subplot 211
plot(t_tc_sim,voltage_cm_device_51_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,voltage_cm_device_41_sim,'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(t_tc_sim,voltage_cm_device_31_sim,'-','LineWidth',tratto2,'Color',colore3);
hold off
title('Components common mode voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{51}^{cm}$','$u_{41}^{cm}$','$u_{31}^{cm}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[Vfc4_base-DVfc Vfc4_base+DVfc]);
% set(gca,'xlim',[t1c tc_end]);
set(gca,'xlim',[1.23 1.25]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,voltage_cm_device_52_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,voltage_cm_device_42_sim,'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(t_tc_sim,voltage_cm_device_32_sim,'-','LineWidth',tratto2,'Color',colore3);
hold off
title('Components common mode voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{52}^{cm}$','$u_{42}^{cm}$','$u_{32}^{cm}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
% set(gca,'xlim',[t1c tc_end]);
set(gca,'xlim',[1.23 1.25]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('cms_voltage_fig_1','-depsc');
% print('sim_results_fig_1','-djpeg');

figure;
subplot 211
plot(t_tc_sim,voltage_cm_device_21_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,voltage_cm_device_11_sim,'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Components common mode voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{21}^{cm}$','$u_{11}^{cm}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[Vfc4_base-DVfc Vfc4_base+DVfc]);
% set(gca,'xlim',[t1c tc_end]);
set(gca,'xlim',[1.23 1.25]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,voltage_cm_device_22_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,voltage_cm_device_12_sim,'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Components common mode voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{22}^{cm}$','$u_{12}^{cm}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
% set(gca,'xlim',[t1c tc_end]);
set(gca,'xlim',[1.23 1.25]);
grid on
h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_1','-depsc');
print('cms_voltage_fig_2','-depsc');

N1 = 1;
N2 = length(t_tc_sim);
voltage_cm_device_51_rms = sqrt(mean(voltage_cm_device_51_sim(N1:N2).^2))
voltage_cm_device_41_rms = sqrt(mean(voltage_cm_device_41_sim(N1:N2).^2))
voltage_cm_device_31_rms = sqrt(mean(voltage_cm_device_31_sim(N1:N2).^2))
voltage_cm_device_21_rms = sqrt(mean(voltage_cm_device_21_sim(N1:N2).^2))
voltage_cm_device_11_rms = sqrt(mean(voltage_cm_device_11_sim(N1:N2).^2))
voltage_cm_device_12_rms = sqrt(mean(voltage_cm_device_12_sim(N1:N2).^2))
voltage_cm_device_22_rms = sqrt(mean(voltage_cm_device_22_sim(N1:N2).^2))
voltage_cm_device_32_rms = sqrt(mean(voltage_cm_device_32_sim(N1:N2).^2))
voltage_cm_device_42_rms = sqrt(mean(voltage_cm_device_42_sim(N1:N2).^2))
voltage_cm_device_52_rms = sqrt(mean(voltage_cm_device_52_sim(N1:N2).^2))

current_hf_capacitor_1_rms = sqrt(mean(current_hf_capacitor_1_sim(N1:N2).^2))
current_flying_capacitor_1_rms = sqrt(mean(current_flying_capacitor_1_sim(N1:N2).^2))
current_flying_capacitor_2_rms = sqrt(mean(current_flying_capacitor_2_sim(N1:N2).^2))
current_flying_capacitor_3_rms = sqrt(mean(current_flying_capacitor_3_sim(N1:N2).^2))
current_flying_capacitor_4_rms = sqrt(mean(current_flying_capacitor_4_sim(N1:N2).^2))
current_flying_capacitor_51_rms = sqrt(mean(current_flying_capacitor_51_sim(N1:N2).^2))
current_flying_capacitor_52_rms = sqrt(mean(current_flying_capacitor_52_sim(N1:N2).^2))


figure;
plot(t_tc_sim,voltage_flying_capacitor_1_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,voltage_flying_capacitor_2_sim,'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,voltage_flying_capacitor_3_sim,'-','LineWidth',tratto1,'Color',colore3);
hold on
plot(t_tc_sim,voltage_flying_capacitor_4_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,voltage_flying_capacitor_5_sim,'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Flying Capacitor Voltages','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{Cfc}^{1}$','$u_{Cfc}^{2}$','$u_{Cfc}^{3}$',...
    '$u_{Cfc}^{4}$','$u_{Cfc}^{5}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c tc_end]);
grid on
print('overall_fc_voltages','-depsc');

figure;
plot(t_tc_sim,device_voltage_51_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,device_voltage_41_sim,'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,device_voltage_31_sim,'-','LineWidth',tratto1,'Color',colore3);
hold on
plot(t_tc_sim,device_voltage_21_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,device_voltage_11_sim,'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Devide voltages','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ds}^{51}$','$u_{ds}^{41}$','$u_{ds}^{31}$',...
    '$u_{ds}^{21}$','$u_{ds}^{11}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'xlim',[t1c tc_end]);
set(gca,'xlim',[1.23 1.25]);
grid on
print('device_voltage_fig_1','-depsc');

figure;
plot(t_tc_sim,device_voltage_52_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,device_voltage_42_sim,'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,device_voltage_32_sim,'-','LineWidth',tratto1,'Color',colore3);
hold on
plot(t_tc_sim,device_voltage_22_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,device_voltage_12_sim,'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Devide voltages','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ds}^{52}$','$u_{ds}^{42}$','$u_{ds}^{32}$',...
    '$u_{ds}^{22}$','$u_{ds}^{12}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'xlim',[t1c tc_end]);
set(gca,'xlim',[1.23 1.25]);
grid on
print('device_voltage_fig_2','-depsc');



