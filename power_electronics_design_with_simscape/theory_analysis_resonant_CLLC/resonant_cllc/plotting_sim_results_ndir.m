close all
clc

tratto1=1;
tratto2=2;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t2c = time_tc_sim(end);
t1c = t2c - 3*glb_time.tPWM_CLLC;
t0c = t2c - glb_time.Nc*glb_time.tc*glb_time.decimation_tc;
% t3c = time_tc_sim(end) - glb_time.Nc*glb_time.tc*glb_time.decimation_tc/10;
% t4c = time_tc_sim(end);
% t5c = time_tc_sim(end) - glb_time.Nc*glb_time.tc/100*glb_time.decimation_tc;
% t6c = time_tc_sim(end);
% t7c = time_tc_sim(end) - glb_time.Nc*glb_time.tc*glb_time.decimation_tc/1000;
% t8c = time_tc_sim(end);
fontsize_plotting = 14;

figure;
subplot 221
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,cllc_voltage_output_modA_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(time_tc_sim,cllc_voltage_input_modA_sim,'--','LineWidth',tratto2,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[0 1500]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,cllc_current_output_modA_sim,'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(time_tc_sim,cllc_current_input_modA_sim,'--','LineWidth',tratto2,'Color',colore2);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-400 0]);
title('CLLC output current and voltage.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{2}^{dc}$','$u_{1}^{dc}$','$i_{2}^{dc}$','$i_{1}^{dc}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t0c t2c]);
grid on

subplot 222
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,mosfet_zvs_plosses_cllc_Q1_sim,'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(time_tc_sim,mosfet_zvs_plosses_cllc_Q2_sim,'--','LineWidth',tratto2,'Color',colore1);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[0 100]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,mosfet_zvs_plosses_cllc_Q5_sim,'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(time_tc_sim,mosfet_zvs_plosses_cllc_Q6_sim,'--','LineWidth',tratto2,'Color',colore2);
hold off
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[0 100]);
title('CLLC device power losses.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$p_{loss}^{Q1}$','$p_{loss}^{Q2}$','$p_{loss}^{Q5}$','$p_{loss}^{Q6}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t0c t2c]);
grid on

% subplot 211
% colororder({'k','k'})
% yyaxis left;
% plot(time_tc_sim,mosfet_zvs_uds_cllc_Q2_sim,'-','LineWidth',tratto2,'Color',colore1);
% hold on
% plot(time_tc_sim,mosfet_zvs_uds_cllc_Q5_sim,'--','LineWidth',tratto2,'Color',colore1);
% ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[0 1500]);
% hold on
% yyaxis right;
% ax = gca;
% ax.YColor = [0.5 0.5 0.5]; 
% plot(time_tc_sim,mosfet_zvs_ids_cllc_Q2_sim,'-','LineWidth',tratto2,'Color',colore2);
% hold on
% plot(time_tc_sim,mosfet_zvs_ids_cllc_Q5_sim,'--','LineWidth',tratto2,'Color',colore2);
% hold off
% ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-750 750]);
% title('CLLC output current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
% legend('$u_{2}^{dc}$','$u_{1}^{dc}$','$i_{2}^{dc}$','$i_{1}^{dc}$','Location','northwestoutside',...
%     'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'xlim',[t1c t2c]);
% grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,i1_cllc_transformer_modA_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-750 750]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,u1_cllc_transformer_modA_sim,'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,u2_cllc_transformer_modA_sim,'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1500 1500]);
title('CLLC resonant tank quantities - positive direction.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{1}^{ac}$','$u_{1}^{ac}$','$u_{2}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('CLLC_quantities_ndir','-depsc');
movefile('CLLC_quantities_ndir.eps', 'figures');
