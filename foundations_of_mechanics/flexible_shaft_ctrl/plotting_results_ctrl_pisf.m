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
plot(t_tc_sim,omega_m_pu_ref_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,omega_m_pu_sim,'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,omega_l_pu_sim,'-','LineWidth',tratto1,'Color',colore3);
hold off
title('Speeds of the Plant in case of Advanced Speed Ctrl','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\omega_m^{ref}$','$\omega_m$','$\omega_l$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c t2c]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(3); chH(2); chH(1)])
subplot 212
plot(t_tc_sim,torque_m_pu_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,torque_theta_pu_sim,'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,torque_disturbance_sim,'-','LineWidth',tratto1,'Color',colore3);
hold off
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
title('Torque applied in case of Advanced Speed Ctrl','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\tau_m$','$\tau_\theta$','$\tau_d$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
% set(gca,'ylim',[-1.5 1.5]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
print('speed_ctrl_performance_pisf','-depsc');

figure;
subplot 211
plot(t_tc_sim,omega_m_pu_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,omega_m_pu_hat_sim,'-','LineWidth',tratto1,'Color',colore2);
hold off
title('Observer performance - $\hat{\omega}_m$','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\omega_m$','$\hat{\omega}_m$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c t2c]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(1); chH(2)])
subplot 212
plot(t_tc_sim,omega_l_pu_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,omega_l_pu_hat_sim,'-','LineWidth',tratto1,'Color',colore2);
hold off
title('Observer performance - $\hat{\omega}_l$','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\omega_l$','$\hat{\omega}_l$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c t2c]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(1); chH(2)])
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])


figure;
subplot 211
plot(t_tc_sim,torque_theta_pu_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,torque_theta_pu_hat_sim,'-','LineWidth',tratto1,'Color',colore2);
hold off
title('Observer performance - $\hat{\tau}_\theta$','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\tau_\theta$','$\hat{\tau}_\theta$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c t2c]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(1); chH(2)])
subplot 212
plot(t_tc_sim,omega_l_pu_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,omega_l_pu_hat_sim,'-','LineWidth',tratto1,'Color',colore2);
hold off
title('Observer performance - $\hat{\omega}_l$','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\omega_l$','$\hat{\omega}_l$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c t2c]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(1); chH(2)])
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
print('observer_performance_pisf','-depsc');



figure;
plot(t_tc_sim,omega_l_pu_pictrl_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,omega_l_pu_pisf_sim,'-','LineWidth',tratto1,'Color',colore2);
hold off
title('Control strategy comparison on ${\omega}_l$','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\omega_l^{pi}$','$\omega_l^{asc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c t2c]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(1); chH(2)])
print('pi_versus_pisf','-depsc');










