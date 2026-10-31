close all
clc

tratto1=1;
tratto2=2;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2b = [0.35 0.35 0.35];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];

t1c = time_tc_sim(end) - param.glb_time.Nc*param.glb_time.tc*param.glb_time.decimation_tc;
t2c = time_tc_sim(end);


fontsize_plotting = 14;

%% figure(1)
figure(1);
subplot 221
plot(time_tc_sim,inverter_current_output_sim,'-','LineWidth',tratto2,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Invertert Output Current');
legend('$i_{s}^{ac}$','Location','northwest',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
subplot 222
plot(time_tc_sim,inverter_voltage_output_sim,'-','LineWidth',tratto2,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
title('Invertert Output Voltage');
legend('$u_{s}^{ac}$','Location','northwest',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
subplot 223
plot(time_tc_sim,idc1_m_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(time_tc_sim,idc2_m_sim,'-','LineWidth',tratto2,'Color',colore2);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('DClink Currents');
legend('$i_{C1}^{dc}$','$i_{C2}^{dc}$','Location','northwest',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
subplot 224
plot(time_tc_sim,mosfet_ploss_invQ1_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(time_tc_sim,mosfet_ploss_invQ2_sim,'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(time_tc_sim,mosfet_ploss_invQ3_sim,'--','LineWidth',tratto2,'Color',colore1);
hold on
plot(time_tc_sim,mosfet_ploss_invQ4_sim,'--','LineWidth',tratto2,'Color',colore2);
hold off
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
title('Phase Device Power Losses');
legend('$p_{Q1}$','$p_{Q2}$','$p_{Q3}$','$p_{Q4}$','Location','northwest',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('single_phase_afe_ttype','-depsc');
movefile('single_phase_afe_ttype.eps', 'figures');


