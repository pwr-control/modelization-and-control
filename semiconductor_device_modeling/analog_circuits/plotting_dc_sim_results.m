clear all
close all
clc
beep off

load sim_results_dc_m.mat;

tratto1=2;
tratto2=2;
tratto3=3;
tratto4=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
colore4 = [0 0 0];

options = bodeoptions;
options.FreqUnits = 'Hz';

t1 = time_sim(end) - N*ts;
t2 = time_sim(end);

fontsize_plotting = 12;

figure;
subplot 211
plot(time_sim,um_sim,'-','LineWidth',tratto3,'Color',colore1);
title('Voltage Phase Measure: source of measure ','Interpreter','latex','FontSize',fontsize_plotting);
legend('$U_m$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1 t2]);
set(gca,'ylim',[0 1500]);
grid on
subplot 212
plot(time_sim,uin_sim,'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_sim,uout1_sim,'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_sim,uout2_sim,'-','LineWidth',tratto3,'Color',colore3);
hold off
title('Voltage Phase Measure: output voltages chain ','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{in}$','$u_{out}^{1}$','$u_{out}^{2}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1 t2]);
set(gca,'ylim',[-0.05 5]);
grid on
print('voltage_dclink_measure_chain_fig1','-depsc');

figure;
subplot 211
plot(time_sim,uout2_sim,'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_sim,uout3_sim,'--','LineWidth',tratto3,'Color',colore3);
hold on
plot(time_sim,uout_p_sim,'-','LineWidth',tratto3,'Color',colore2);
hold off
title('Voltage Phase Measure: output voltages chain ','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{uout}^{2}$','$u_{out}^{3}$','$u_{out}^{p}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1 t2]);
set(gca,'ylim',[-0.05 5]);
grid on
subplot 212
plot(time_sim,iout3_sim,'-','LineWidth',tratto3,'Color',colore1);
title('Voltage Phase Measure: current source to MainBoard','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{out}^{3}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1 t2]);
set(gca,'ylim',[0 40e-3]);
grid on
print('voltage_dclink_measure_chain_fig2','-depsc');