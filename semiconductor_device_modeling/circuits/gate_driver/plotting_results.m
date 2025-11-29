clear all
close all
clc

beep off

tratto1=2;
tratto2=2;
tratto3=2;
tratto4=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
colore4 = [0 0 0];

fontsize_plotting = 12;

options = bodeoptions;
options.FreqUnits = 'Hz';

load sim_results.mat

t1 = time_sim(1);
t2 = time_sim(end);

t3 = 1.85e-4;
t4 = 1.89e-4;

t5 = 1.35e-4;
t6 = 1.39e-4;

figure;
subplot 211
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(time_sim,uge_b_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
yyaxis right;
plot(time_sim,uce_b_sim,'-','LineWidth',tratto1,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Double Pulse Test - Simulation','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ge}^{b}$','$u_{ce}^{b}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1 t2]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
grid on
subplot 212
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(time_sim,uce_b_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
yyaxis right;
plot(time_sim,ie_b_sim,'-','LineWidth',tratto1,'Color',colore2);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Double Pulse Test - Simulation','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ce}^{b}$','$i_{e}^{b}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1 t2]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(1); chH(2)])
grid on
print('sim_result_fig_1','-depsc');

figure;
subplot 211
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(time_sim,uge_b_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
yyaxis right;
plot(time_sim,uce_b_sim,'-','LineWidth',tratto1,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Double Pulse Test - Simulation - IGBT Bottom Turn-Off','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ge}^{b}$','$u_{ce}^{b}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3 t4]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
grid on
subplot 212
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(time_sim,uce_b_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
yyaxis right;
plot(time_sim,ie_b_sim,'-','LineWidth',tratto1,'Color',colore2);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Double Pulse Test - Simulation - IGBT Bottom Turn-Off','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ce}^{b}$','$i_{e}^{b}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3 t4]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(1); chH(2)])
grid on
print('sim_result_fig_2','-depsc');

figure;
subplot 211
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(time_sim,uge_b_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
yyaxis right;
plot(time_sim,uce_b_sim,'-','LineWidth',tratto1,'Color',colore2);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Double Pulse Test - Simulation - IGBT Bottom Turn-On','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ge}^{b}$','$u_{ce}^{b}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5 t6]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(2); chH(1)])
grid on
subplot 212
colors = [colore1; colore2];
colororder(colors)
yyaxis left;
plot(time_sim,uce_b_sim,'-','LineWidth',tratto1,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
hold on
yyaxis right;
plot(time_sim,ie_b_sim,'-','LineWidth',tratto1,'Color',colore2);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
hold off
title('Double Pulse Test - Simulation - IGBT Bottom Turn-On','Interpreter','latex','FontSize',fontsize_plotting);
legend('$u_{ce}^{b}$','$i_{e}^{b}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t5 t6]);
% chH = get(gca,'Children');
% set(gca,'Children',[chH(1); chH(2)])
grid on
print('sim_result_fig_3','-depsc');