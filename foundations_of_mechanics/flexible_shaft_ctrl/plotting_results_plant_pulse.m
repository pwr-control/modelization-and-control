close all
clc

tratto1=2.2;
tratto2=2.2;
tratto3=1;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = t_tc_sim(end) - Nc*tc;
t2c = t_tc_sim(end);

fontsize_plotting = 12;

figure;
plot(t_tc_sim,torque_m_pu_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,omega_m_pu_sim,'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,omega_l_pu_sim,'-','LineWidth',tratto1,'Color',colore3);
hold off
title('System pulse response','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\tau_m$','$\omega_m$','$\omega_l$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-0.25 1.25]);
set(gca,'xlim',[0 30]);
grid on
print('plant_pulse_responce','-depsc');

