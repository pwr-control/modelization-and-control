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
% t1s = t_ts_afe_sim(end) - Ns_afe*ts_afe;
% t2s = t_ts_afe_sim(end);

t1s = 1;
t2s = t1s + 10/fPWM_AFE;

t1c_zoom = t_tc_sim(end) - 5/50;
t2c_zoom = t_tc_sim(end);
t3c_zoom = t_tc_sim(end) - 50/fPWM_AFE;
t4c_zoom = t_tc_sim(end);
t1s_zoom = t_ts_afe_sim(end) - 5/50;
t2s_zoom = t_ts_afe_sim(end);

fontsize_plotting = 12;

figure;
plot(t_tc_sim,udc_pos_sim,'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(t_tc_sim,udc_neg_sim,'-','LineWidth',tratto1,'Color',colore3);
hold on
plot(t_tc_sim,udc_pos_sim-udc_neg_sim,'-','LineWidth',tratto1,'Color',colore1);
hold off
title('DClink voltage measures respect to the ground and differential.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{dc}^{+}$','$u_{dc}^{-}$','$u_{dc}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1250 1250]);
set(gca,'xlim',[t1s t2s]);
grid on
chH = get(gca,'Children');
set(gca,'Children',[chH(3); chH(1); chH(2)]);
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('dclink_voltage_1','-depsc');

figure;
plot(t_tc_sim,udc_pos_sim,'-','LineWidth',tratto1,'Color',colore1);
title('DClink (+) voltage measures respect to the ground.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{dc}^{+}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-250 1250]);
set(gca,'xlim',[t1s t2s]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(3); chH(1); chH(2)]);
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('dclink_voltage_2','-depsc');

figure;
plot(t_tc_sim,udc_neg_sim,'-','LineWidth',tratto1,'Color',colore1);
title('DClink (-) voltage measures respect to the ground.','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{dc}^{-}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex');
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1250 250]);
set(gca,'xlim',[t1s t2s]);
grid on
% chH = get(gca,'Children');
% set(gca,'Children',[chH(3); chH(1); chH(2)]);
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('dclink_voltage_3','-depsc');

udc_neg_sim_rms = sqrt(mean(udc_neg_sim.^2))
udc_pos_sim_rms = sqrt(mean(udc_pos_sim.^2))