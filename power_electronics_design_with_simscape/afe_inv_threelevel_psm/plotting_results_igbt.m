close all
clc

tratto1=2.2;
tratto2=2.2;
tratto3=1;
colore1 = [0.5 0.5 0.5];
colore2 = [0.8 0.8 0.8];
t1c = t_tc_sim(end) - Nc*tc;
t2c = t_tc_sim(end);
t1s = t_ts_afe_sim(end) - Ns_afe*ts_afe;
t2s = t_ts_afe_sim(end);
% t1s = 0;
% t2s = 1.5;

t1c_zoom = t_tc_sim(end) - 5/50;
t2c_zoom = t_tc_sim(end);
t3c_zoom = t_tc_sim(end) - 50/fPWM_AFE;
t4c_zoom = t_tc_sim(end);
t1s_zoom = t_ts_afe_sim(end) - 5/50;
t2s_zoom = t_ts_afe_sim(end);

fontsize_plotting = 16;

figure;
plot(t_tc_sim,ue_top_inv_sim,'-k','LineWidth',tratto1);
title('Inverter TopEmitter to Ground Voltage Distribution','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_e^{top}$','best','Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1500 1500]);
set(gca,'xlim',[t2s-6e-3 t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('TopEmitterUe','-depsc');

return

figure;
subplot 211
plot(t_ts_inv_sim,iq_ref_pu_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_inv_sim,iq_pu_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Q-Current tracking in p.u.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_q^{ref}$','$i_q$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
subplot 212
colororder({'k','k'})
yyaxis left;
plot(t_ts_inv_sim,omega_pu_sim.*omega_m_bez,'-','LineWidth',tratto2,'Color',colore1);
ylabel('$\omega /rpm$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
yyaxis right;
plot(t_ts_inv_sim,torque_load_pu_pmsm.*tau_bez*1e-3,'-k','LineWidth',tratto1);
ylabel('$\tau /kNm$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Generator speed and torque in p.u.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\omega_m$','$\tau_m$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
% set(gca,'ylim',[-1.5 1.5]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
% print('inverter_generator_quantities','-depsc');


figure;
subplot 211
plot(t_ts_inv_sim,psi_r_alpha_pmsm_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_inv_sim,psi_r_alpha_hat_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('$\alpha$ permanent magnet flux from observer and machine',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$\psi_{\alpha}^{m}$','$\hat{\psi}_{\alpha}^{m}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$\psi/Vs$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
set(gca,'ylim',[-5.5 5.5]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_ts_inv_sim,psi_r_beta_pmsm_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_inv_sim,psi_r_beta_hat_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('$\beta$ permanent magnet flux from observer and machine',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$\psi_{\beta}^{m}$','$\hat{\psi}_{\beta}^{m}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$\psi/Vs$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
set(gca,'ylim',[-5.5 5.5]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
% print('inverter_fluxes_observer','-depsc');


figure;
subplot 211
plot(t_ts_inv_sim,omega_ref_pu_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_inv_sim,omega_pu_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Speed tracking in p.u.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\omega_m^{ref}$','$\omega_m$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_ts_inv_sim,torque_load_pu_pmsm,'-k','LineWidth',tratto1);
title('Load torque in p.u.',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$\tau_m$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_1','-depsc');

figure;
subplot 211
plot(t_ts_inv_sim,iq_ref_pu_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_inv_sim,iq_pu_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Q-Current tracking in p.u.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_q^{ref}$','$i_q$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_ts_inv_sim,id_ref_pu_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_inv_sim,id_pu_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('D-Current tracking in p.u.',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_d^{ref}$','$i_d$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p.u.$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
% print('sim_results_fig_2','-depsc');

figure;
plot(t_tc_sim,iuvw_sim(:,1),'-k','LineWidth',tratto1);
hold on
plot(t_tc_sim,iuvw_sim(:,2),'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim,iuvw_sim(:,3),'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Motor Current in A','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_u$','$i_v$','$i_w$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
% print('sim_results_fig_3','-depsc');

figure;
subplot 211
plot(t_ts_inv_sim,psi_r_alpha_pmsm_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_inv_sim,psi_r_alpha_hat_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('$\alpha$-Flux Estimation in Vs','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\psi_{\alpha}^{m}$','$\hat{\psi}_{\alpha}^{m}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$\psi/Vs$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_ts_inv_sim,psi_r_beta_pmsm_sim,'-k','LineWidth',tratto1);
hold on
plot(t_ts_inv_sim,psi_r_beta_hat_sim,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('$\beta$-Flux Estimation in Vs','Interpreter','latex','FontSize',fontsize_plotting);
legend('$\psi_{\beta}^{m}$','$\hat{\psi}_{\beta}^{m}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$\psi/Vs$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
chH = get(gca,'Children');
set(gca,'Children',[chH(2); chH(1)])
% print('sim_results_fig_4','-depsc');

figure;
subplot 211
plot(t_tc_sim,udc_sim,'-k','LineWidth',tratto1);
title('DClink Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$v/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 212
plot(t_tc_sim,idc_sim,'-k','LineWidth',tratto1);
title('DClink DC Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
% print('sim_results_fig_5','-depsc');

figure;
subplot 211
plot(t_tc_sim,idc_sim,'-k','LineWidth',tratto1);
title('DClink DC plus Capacitor Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{dc}^{dclink}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 212
plot(t_tc_sim,udc_sim,'-k','LineWidth',tratto1);
title('DClink Capacitor Voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$v/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
% print('sim_results_fig_6','-depsc');
% 
N1 = Nc-floor(t1c_zoom/tc);
dclink_terminal_current_inverter_rms = sqrt(mean(ileg_dc_inv_sim.^2))
dclink_terminal_current_inverter_rms = sqrt(mean(ileg_dc_inv_sim(N1:Nc).^2))

figure;
plot(t_tc_sim,ileg_dc_inv_sim,'-k','LineWidth',tratto1);
title('DC current leg - positive current means IGBT current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{dc}^{leg}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1s t2s]);
grid on
% print('sim_results_fig_9','-depsc');

