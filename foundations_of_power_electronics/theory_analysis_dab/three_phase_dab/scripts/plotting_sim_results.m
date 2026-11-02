close all
clc

% clear
% load ../sim_results/sim_results_1;

tratto1=1;
tratto2=2;
tratto3=3;

colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];

N1c = floor(param.glb_time.Nc/2);
N2c = floor(param.glb_time.Nc);
N3c = floor(param.glb_time.Nc - 4/param.dab3.fs/param.glb_time.tc);   % last 4 periods of the transformer

t1c = time_tc_sim(N1c);
t2c = time_tc_sim(N2c);

t3c = time_tc_sim(N3c);
t4c = time_tc_sim(N2c);

fontsize_title = 18;
fontsize_legend = 18;
fontsize_axis = 16;

%% figure(1)
figure(1);
subplot 311
plot(time_tc_sim,i1_dab_transformer_modA_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,i1_dab_transformer_modA_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,i1_dab_transformer_modA_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
title('DAB Currents','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{dab}^\mathrm{u}$','$i_\mathrm{dab}^\mathrm{v}$','$i_\mathrm{dab}^\mathrm{w}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
% xlabel('$Time - [s]$','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
% set(gca,'ylim',[-650 650]);
grid on
subplot 312
plot(time_tc_sim,u1_dab_transformer_modA_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,u1_dab_transformer_modA_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,u1_dab_transformer_modA_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
title('DAB Secondary Side Voltages','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{dab}^\mathrm{u1}$','$u_\mathrm{dab}^\mathrm{v1}$','$u_\mathrm{dab}^\mathrm{w1}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
% set(gca,'ylim',[-500 500]);
grid on
subplot 313
plot(time_tc_sim,u2_dab_transformer_modA_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,u2_dab_transformer_modA_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,u2_dab_transformer_modA_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
title('DAB Secondary Side Voltages','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{dab}^\mathrm{u2}$','$u_\mathrm{dab}^\mathrm{v2}$','$u_\mathrm{dab}^\mathrm{w3}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca,'xlim',[t3c t4c]);
% set(gca,'ylim',[-500 500]);
grid on
% h=gcf;
% set(h,'PaperOrientation','landscape');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
print('dab_ac_quantities','-depsc');
movefile('dab_ac_quantities.eps', '../figures/sim_results_1');