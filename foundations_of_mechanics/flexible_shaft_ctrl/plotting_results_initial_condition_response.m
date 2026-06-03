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
subplot 211
plot(t_tc_sim,torque_theta_pu_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,omega_m_pu_sim,'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,omega_l_pu_sim,'-','LineWidth',tratto1,'Color',colore3);
hold off
title('System evolution from non-equilibrium point','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\tau_\theta$','$\omega_m$','$\omega_l$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-2.25 1.25]);
set(gca,'xlim',[0 30]);
grid on
subplot 212
plot(t_tc_sim,torque_theta_pu_sf_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,omega_m_pu_sf_sim,'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,omega_l_pu_sf_sim,'-','LineWidth',tratto1,'Color',colore3);
hold off
title('System evolution from non-equilibrium point with state feedback','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\tau_\theta$','$\omega_m$','$\omega_l$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-2.25 1.25]);
set(gca,'xlim',[0 30]);
grid on
print('init_condition_responce_plant','-depsc');

