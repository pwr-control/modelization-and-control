close all
clc

tratto1=2.2;
tratto2=2.2;
tratto3=1;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
% t1c = t_tc_sim(end) - Nc*tc;
% t1c = t_tc_sim(1);
% t2c = t_tc_sim(end);
t1c = 0.475;
t2c = 0.525;

fontsize_plotting = 12;

u01 = 1/3*(uout1_uvw_sim(:,1)+uout1_uvw_sim(:,2)+uout1_uvw_sim(:,3));
u01_rms = sqrt(mean(u01.^2))

figure;
plot(t_tc_sim,uout2_uvw_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,uout2_uvw_sim(:,2),'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,uout2_uvw_sim(:,3),'-','LineWidth',tratto1,'Color',colore3);
hold on
plot(t_tc_sim,u_nom*ones(1,Nc),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,u_nom*0.92*ones(1,Nc),'-','LineWidth',tratto1,'Color',colore3);
hold off
title('Output 1 Voltages',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_u$','$u_v$','$u_w$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
set(gca,'ylim',[140 175]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('sim_results_fig_6','-depsc');
print('sim_results_fig_6','-dpdf');

% return

figure;
subplot 211
plot(t_tc_sim,iout1_uvw_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,iout1_uvw_sim(:,2),'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,iout1_uvw_sim(:,3),'-','LineWidth',tratto1,'Color',colore3);
hold off
title('Output 1 Currents','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_u$','$i_v$','$i_w$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c t2c]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,uout1_uvw_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,uout1_uvw_sim(:,2),'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,uout1_uvw_sim(:,3),'-','LineWidth',tratto1,'Color',colore3);
hold off
title('Output 1 Voltages',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_u$','$u_v$','$u_w$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('sim_results_fig_1','-depsc');
print('sim_results_fig_1','-dpdf');

figure;
subplot 211
plot(t_tc_sim,iout2_uvw_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,iout2_uvw_sim(:,2),'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,iout2_uvw_sim(:,3),'-','LineWidth',tratto1,'Color',colore3);
hold off
title('Output 2 Currents','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_u$','$i_v$','$i_w$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c t2c]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,uout2_uvw_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,uout2_uvw_sim(:,2),'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,uout2_uvw_sim(:,3),'-','LineWidth',tratto1,'Color',colore3);
hold off
title('Output 2 Voltages',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_u$','$u_v$','$u_w$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('sim_results_fig_2','-depsc');
print('sim_results_fig_2','-dpdf');

figure;
subplot 211
plot(t_tc_sim,i_CFu_sim,'-','LineWidth',tratto1,'Color',colore1);
title('Output Filter Capacitor Current','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_c$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c t2c]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,i_CFi_sim,'-','LineWidth',tratto1,'Color',colore1);
title('DClink Capacitor Current',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_c$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('sim_results_fig_3','-depsc');
print('sim_results_fig_3','-dpdf');


figure;
subplot 211
plot(t_tc_sim,is_uvw_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(t_tc_sim,is_uvw_sim(:,2),'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,is_uvw_sim(:,3),'-','LineWidth',tratto1,'Color',colore3);
hold off
title('Inverter Currents','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_s^u$','$i_s^v$','$i_s^w$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t1c t2c]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
subplot 212
plot(t_tc_sim,udc_sim,'-','LineWidth',tratto1,'Color',colore1);
title('DClink Voltage',...
    'Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-250 250]);
set(gca,'xlim',[t1c t2c]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('sim_results_fig_4','-depsc');
print('sim_results_fig_4','-dpdf');



figure;
subplot 211
plot(t_tc_sim,u01,'-','LineWidth',tratto1,'Color',colore1);
title('Common mode voltage - inpout at output transformer','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{ct}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-115 115]);
set(gca,'xlim',[t1c t2c]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('sim_results_fig_5','-depsc');
print('sim_results_fig_5','-dpdf');

% figure;
% subplot 211
% plot(t_tc_sim,idc_center_tap_sim,'-','LineWidth',tratto1,'Color',colore1);
% title('Inverter Center Tap Current','Interpreter','latex','FontSize',fontsize_plotting);
% legend('$i_{ct}$','Location','best',...
%     'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
% ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-115 115]);
% set(gca,'xlim',[t1c t2c]);
% grid on
% % h=gcf;
% % set(h,'PaperOrientation','landscape');
% % set(h,'PaperUnits','normalized');
% % set(h,'PaperPosition', [0 0 1 1]);
% print('sim_results_fig_5','-depsc');
% print('sim_results_fig_5','-dpdf');











