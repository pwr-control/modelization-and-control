close all
clc

tratto1=2.5;
tratto2=2.5;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = time_tc_sim(end) - Nc*tc;
t2c = time_tc_sim(end);
t3c = time_tc_sim(end) - Nc*tc/100;
t4c = time_tc_sim(end);
t5c = time_tc_sim(end) - Nc*tc/1000;
t6c = time_tc_sim(end);
t7c = time_tc_sim(end) - Nc*tc/10000;
t8c = time_tc_sim(end);
fontsize_plotting = 14;

figure;
subplot 211
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,dab_voltage_output_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_tc_sim,dab_voltage_input_modA_sim,'-','LineWidth',tratto1,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[0 1000]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,dab_current_output_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
hold on
plot(time_tc_sim,dab_current_input_modA_sim,'-','LineWidth',tratto1,'Color',colore2);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'ylim',[0 1000]);
title('CLLC output current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{2}^{dc}$','$u_{1}^{dc}$','$i_{2}^{dc}$','$i_{1}^{dc}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
subplot 212
colororder({'k','k'})
yyaxis left;
plot(time_tc_sim,i1_dab_transformer_modA_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1250 1000]);
hold on
yyaxis right;
ax = gca;
ax.YColor = [0.5 0.5 0.5]; 
plot(time_tc_sim,u1_dab_transformer_modA_sim,'-','LineWidth',tratto1,'Color',colore2);
hold on
plot(time_tc_sim,u2_dab_transformer_modA_sim,'-','LineWidth',tratto1,'Color',colore3);
hold off
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[500 1850]);
title('CLLC output current and voltage','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{1}^{ac}$','$u_{1}^{ac}$','$u_{2}^{ac}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t7c t8c]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('CLLC_ac_quantities','-depsc');
movefile('CLLC_ac_quantities.eps', 'figures');


