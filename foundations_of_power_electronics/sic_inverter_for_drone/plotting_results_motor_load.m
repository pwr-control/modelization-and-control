close all
clc

tratto1=2.2;
tratto2=2.2;
tratto3=1;
colore1 = [0.5 0.5 0.5];
colore2 = [0.8 0.8 0.8];
% t1c = t_tc_sim(end) - Nc*tc;
t1c = t_tc_sim(1);
t2c = t_tc_sim(end);
% t1s = t_tsample_sim(end) - Nsample*tsample;
t1s = t_tsample_sim(1);
t2s = t_tsample_sim(end);

tz = 4.5;

t1c_zoom = tz - 0.008;
t2c_zoom = tz;
t3c_zoom = tz - 20/fPWM_INV;
t4c_zoom = tz;
t1s_zoom = tz - 5/1250;
t2s_zoom = tz;

fontsize_plotting = 12;



figure;
subplot 211
plot(t_tsample_sim,omega_ref_pu_sim,'-k','LineWidth',tratto1);
hold on
plot(t_tsample_sim,omega_pu_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Speed tracking in p.u - 1 p.u. = 2291 RPM.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\omega_m^{ref}$','$\omega_m$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tsample_sim,torque_load_pu_pmsm,'-k','LineWidth',tratto1);
title('Load torque in p.u. - 1 p.u. = 64 Nm.',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$\tau_m$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('sim_results_fig_1','-depsc');
print('sim_results_fig_1','-djpeg');

figure;
subplot 211
plot(t_tsample_sim,iq_ref_pu_sim,'-k','LineWidth',tratto1);
hold on
plot(t_tsample_sim,iq_pu_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Q-Current tracking in p.u. - 1 p.u. = 64 A.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_q^{ref}$','$i_q$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tsample_sim,id_ref_pu_sim,'-k','LineWidth',tratto1);
hold on
plot(t_tsample_sim,id_pu_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('D-Current tracking in p.u. - 1 p.u. = 64 A.',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_d^{ref}$','$i_d$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
print('sim_results_fig_2','-depsc');
print('sim_results_fig_2','-djpeg');

figure;
plot(t_tc_sim,iuvw_sim(:,1),'-k','LineWidth',tratto1);
hold on
plot(t_tc_sim,iuvw_sim(:,2),'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,iuvw_sim(:,3),'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Inverter Current in A','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_u$','$i_v$','$i_w$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
print('sim_results_fig_3','-depsc');
print('sim_results_fig_3','-djpeg');

figure;
subplot 211
plot(t_tsample_sim,phi_alpha_pmsm_sim,'-k','LineWidth',tratto1);
hold on
plot(t_tsample_sim,phi_alpha_hat_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('$\alpha$-Flux Estimation in Vs','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\psi_{\alpha}^{m}$','$\hat{\psi}_{\alpha}^{m}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$\psi/Vs$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tsample_sim,phi_beta_pmsm_sim,'-k','LineWidth',tratto1);
hold on
plot(t_tsample_sim,phi_beta_hat_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('$\beta$-Flux Estimation in Vs','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\psi_{\beta}^{m}$','$\hat{\psi}_{\beta}^{m}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$\psi/Vs$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
print('sim_results_fig_4','-depsc');
print('sim_results_fig_4','-djpeg');

figure;
subplot 211
plot(t_tc_sim,udc_sim,'-k','LineWidth',tratto1);
title('DClink Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$v/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
subplot 212
plot(t_tc_sim,idc_sim,'-k','LineWidth',tratto1);
title('Battery Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
print('sim_results_fig_5','-depsc');
print('sim_results_fig_5','-djpeg');

figure;
subplot 211
plot(t_tc_sim,idclink_capacitor_sim,'-k','LineWidth',tratto1);
title('DClink Capacitor Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_c^{dclink}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
subplot 212
plot(t_tc_sim,udc_sim,'-k','LineWidth',tratto1);
title('DClink Capacitor Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$v/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
print('sim_results_fig_6','-depsc');
print('sim_results_fig_6','-djpeg');

figure;
subplot 211
plot(t_tc_sim,mosfet_JH_temp_sim,'-k','LineWidth',tratto1);
hold on
plot(t_tc_sim,mosfet_HA_temp_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,mosfet_JH_temp_sim+mosfet_HA_temp_sim+Tambient,'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Single Mosfet JH, HA and Absolute Junction Temperature','Interpreter','latex','FontSize',fontsize_plotting);
legend('$T_{jh}$','$T_{ha}$','$T_{j}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$T/K$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[0 150]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
subplot 212
plot(t_tc_sim,mosfet_power_losses_sim,'-k','LineWidth',tratto1);
title('Single Mosfet Power Losses','Interpreter','latex','FontSize',fontsize_plotting);
legend('$P_{loss}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$P/W$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
print('sim_results_fig_7','-depsc');
print('sim_results_fig_7','-djpeg');

figure;
subplot 211
plot(t_tc_sim,iabc_load_sim(:,1),'-k','LineWidth',tratto1);
hold on
plot(t_tc_sim,iabc_load_sim(:,2),'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,iabc_load_sim(:,3),'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Motor Load Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{a}$','$i_{b}$','$i_{c}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
subplot 212
plot(t_tc_sim,uab_bc_ca_load_sim(:,1),'-k','LineWidth',tratto1);
hold on
plot(t_tc_sim,uab_bc_ca_load_sim(:,2),'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,uab_bc_ca_load_sim(:,3),'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Motor Load Voltage (Ph-Ph)','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ab}$','$u_{bc}$','$u_{ca}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c_zoom t2c_zoom]);
grid on
print('sim_results_fig_8','-depsc');
print('sim_results_fig_8','-djpeg');

