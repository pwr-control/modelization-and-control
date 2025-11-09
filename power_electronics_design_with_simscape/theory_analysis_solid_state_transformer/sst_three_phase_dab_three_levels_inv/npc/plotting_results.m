close all
clc

tratto1=2.5;
tratto2=2.5;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
time_offset = 0.0;
t1c = time_tc_sim(end) - Nc*tc/1000 - time_offset;
t2c = time_tc_sim(end) - time_offset;
t1s = time_ts_dab_modA_sim(end) - Ns_dab*ts_dab;
t2s = time_ts_dab_modA_sim(end);
t3s = time_ts_dab_modA_sim(end) - Ns_dab*ts_dab/10;
t4s = time_ts_dab_modA_sim(end);
fontsize_plotting = 14;

figure;
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,dab_current_output_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1250 500]);
hold on
yyaxis right;
plot(time_tc_sim,dab_voltage_output_modA_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[0 1750]);
hold off
title('DAB output current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{2}^{dc}$','$u_{2}^{dc}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,dab_current_input_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-500 0]);
yyaxis right;
plot(time_tc_sim,dab_voltage_input_modA_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[0 1750]);
hold off
title('DAB input current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{1}^{dc}$','$u_{1}^{dc}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_ui_output_input','-depsc');
movefile('dab_ui_output_input.eps', 'figures');

figure;
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,i2_dab_transformer_modA_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-500 500]);
yyaxis right;
plot(time_tc_sim,u2_dab_transformer_modA_sim(:,1),'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(time_tc_sim,u2_dab_transformer_modA_sim(:,2),'--','LineWidth',tratto2,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1250 1250]);
hold off
title('Transformer-DAB output current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{2u}^{ac}$','$u_{2u}^{ac}$','$u_{2v}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,i1_dab_transformer_modA_sim(:,1),'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-500 500]);
yyaxis right;
plot(time_tc_sim,u1_dab_transformer_modA_sim(:,1),'-','LineWidth',tratto2,'Color',colore2);
hold on
plot(time_tc_sim,u1_dab_transformer_modA_sim(:,2),'--','LineWidth',tratto2,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-1250 1250]);
hold off
title('Transformer-DAB input current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{1u}^{ac}$','$u_{1u}^{ac}$','$u_{1v}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dab_transformer_voltage_current','-depsc');
movefile('dab_transformer_voltage_current.eps', 'figures');

figure;
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,current_battery_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-1000 0]);
yyaxis right;
plot(time_tc_sim,voltage_battery_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[0 1550]);
hold off
title('DC grid (battery) current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{g}^{dc}$','$u_{g}^{dc}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,ac_grid_current_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
set(gca,'ylim',[-2000 2000]);
yyaxis right;
plot(time_tc_sim,ac_grid_voltage_sim,'-','LineWidth',tratto2,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[-850 850]);
hold off
title('AC grid current and voltage - phase U','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{gu}^{ac}$','$u_{gu}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('dc_ac_grid_voltage_current','-depsc');
movefile('dc_ac_grid_voltage_current.eps', 'figures');

