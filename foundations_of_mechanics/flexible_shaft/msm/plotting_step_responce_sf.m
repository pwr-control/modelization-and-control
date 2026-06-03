clear all
close all
clc

load("sim_results_2.mat");

tratto1 = 3;
tratto2 = 3;
tratto3 = 3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = time_tc_msm_sim(end) - Nc*Tc;
t2c = t1c + 1;

fontsize_plotting = 14;

figure;
subplot 211
plot(time_tc_msm_sim,omega_1_msm_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_tc_msm_sim,omega_2_msm_sim,'-','LineWidth',tratto1,'Color',colore2);
hold off
title('System pulse response','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\omega_1$','$\omega_2$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-0.5 0.5]);
set(gca,'xlim',[t1c t2c]);
grid on
subplot 212
plot(time_tc_msm_sim,tau_theta_12_msm_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_tc_msm_sim,tau_m_msm_sim,'-','LineWidth',tratto1,'Color',colore3);
hold off
title('System pulse response','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\tau_{12}$','$\tau_m$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-0.5 1.25]);
set(gca,'xlim',[t1c t2c]);
grid on
print('plant_msm_pulse_responce_sf','-djpeg');

