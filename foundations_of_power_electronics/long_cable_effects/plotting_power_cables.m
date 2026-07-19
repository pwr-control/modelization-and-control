close all
clc
clear 
load sim_results_2


tratto1 = 1;
tratto2 = 2;
tratto3 = 3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = 1.5 - 1e-3;
t2c = 1.5;

fontsize_title = 14;
fontsize_legend = 14;
fontsize_axis = 14;

figure;
plot(t_tc_sim,iz_power_cable_drive_side_sim,'-k','LineWidth',tratto1);
title('Zero sequence inverter output current','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{z}^\mathrm{inv}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[-50 50]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('sim_results_fig_1','-depsc');
movefile('sim_results_fig_1.eps', 'figures');

figure;
plot(t_tc_sim,udc_pgnd_sim,'-','LineWidth',tratto1,'Color',colore1);
title('DClink Voltage p(+) - GND','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{dc}^\mathrm{p}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'ylim',[-250 1000]);
set(gca,'xlim',[t1c t2c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('sim_results_fig_1','-depsc');
print('sim_results_fig_2','-depsc');
movefile('sim_results_fig_2.eps', 'figures');


