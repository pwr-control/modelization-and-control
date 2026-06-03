close all
clc

tratto1=1.5;
tratto2=2;
tratto3=1;
colore1 = [0.5 0.5 0.5];
colore2 = [0.8 0.8 0.8];
t1c = time_tc(end) - 40/fPWM_ZCS_MAX;
t2c = time_tc(end) - 5/fPWM_ZCS_MAX;
t0c = time_tc(end) - Nc*tc;
t0s = time_ts(end) - Ns*ts;

figure;
subplot 211
plot(time_tc,m_gA,'-k','LineWidth',tratto1);
hold on
plot(time_tc,is./20,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Gate command $Q1$ and $i_s$ in per unit','Interpreter','latex');
legend('$g_{Q1}$','$i_{s}$','Location','best','Interpreter','latex','FontSize',12);
xlabel('$s$','Interpreter','latex');
ylabel('');
set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t2c time_tc(end)]);
grid on
subplot 212
plot(time_tc,m_gB,'-k','LineWidth',tratto1);
hold on
plot(time_tc,is./20,'-','LineWidth',tratto2,'Color',colore1);
hold off
title('Gate command $Q2$ and $i_s$ in per unit','Interpreter','latex');
legend('$g_{Q2}$','$i_{s}$','Location','best','Interpreter','latex','FontSize',12);
xlabel('$s$','Interpreter','latex');
ylabel('');
set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t2c time_tc(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print('without_comp_fig_1','-depsc');


% figure;
% subplot 211
% plot(time_ts,grid_vi_ts(:,1),'-k','LineWidth',tratto2);
% hold on
% plot(time_ts,grid_vi_ts(:,2),'-','LineWidth',tratto2,'Color',colore1); 
% hold on
% plot(time_ts,grid_vi_ts(:,3),'-','LineWidth',tratto2,'Color',colore2); 
% hold off
% title('Grid Voltage','Interpreter','latex');
% legend('$v_a$','$v_b$','$v_c$','Location','best','Interpreter','latex','FontSize',12);
% xlabel('$s$','Interpreter','latex');
% ylabel('$V$','Interpreter','latex');
% % set(gca,'ylim',[0 650]);
% set(gca,'xlim',[time_ts(end)-100e-3 time_ts(end)]);
% grid on
% subplot 212
% plot(time_ts,grid_omega_pll./2/pi,'-k','LineWidth',tratto2);
% title('Grid Frequency Estimation from PLL','Interpreter','latex');
% legend('$f_{grid}^{pll}$','Location','best','Interpreter','latex','FontSize',12);
% xlabel('s','Interpreter','latex');
% ylabel('Hz','Interpreter','latex');
% % set(gca,'ylim',[0 650]);
% set(gca,'xlim',[time_ts(end)-100e-3 time_ts(end)]);
% grid on
% % print('without_comp_fig_2','-depsc');
 
figure;
subplot 211
plot(time_tc,iout_meas,'-k','LineWidth',tratto2);
title('Load current - without compensation','Interpreter','latex');
legend('$i_{out}$','Location','best','Interpreter','latex','FontSize',12);
xlabel('$s$','Interpreter','latex');
ylabel('$A$','Interpreter','latex');
set(gca,'ylim',[85 115]);
set(gca,'xlim',[t0c time_tc(end)]);
grid on
subplot 212
plot(time_tc,vout_meas,'-k','LineWidth',tratto2);
title('Load voltage - without compensation','Interpreter','latex');
legend('$v_{out}$','Location','best','Interpreter','latex','FontSize',12);
xlabel('$s$','Interpreter','latex');
ylabel('$V$','Interpreter','latex');
set(gca,'ylim',[5 20]);
set(gca,'xlim',[t0c time_tc(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print(gcf, '-dpdf', 'fig1_without_compensation.pdf');
% print('without_comp_fig_4','-depsc');

figure;
plot(time_ts, i_ctrl_out,'-k','LineWidth',tratto2);
title('Switching Frequency','Interpreter','latex');
legend('$f_{sw}$', 'Location','best','Interpreter','latex','FontSize',12);
xlabel('$s$','Interpreter','latex');
ylabel('$100kHz$','Interpreter','latex');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0c time_tc(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print('without_comp_fig_5','-depsc');


% figure;
% plot(time_ts,grid_phase_est,time_ts,grid_emu_phase,'LineWidth',tratto1); 
% title('PLL Phase - Grid Phase');
% legend('\theta_{pll}','\theta_{grid}','Location','best');
% xlabel('s');
% ylabel('');
% % set(gca,'ylim',[0 650]);
% set(gca,'xlim',[time_ts(end)-100e-3 time_ts(end)]);
% grid on
% % print('without_comp_fig_3','-depsc');

figure;
subplot 211
plot(time_ts,iout_ref_pu,time_ts,iout_pu,'LineWidth',tratto1); 
title('Load current tracking in per unit - with compensation');
legend('i_{out}^{ref}','i_{out}','Location','best');
xlabel('s');
ylabel('pu');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
subplot 212
plot(time_ts, i_ctrl_out, time_ts,iout_pu,'LineWidth',tratto1); 
title('Current control output');
legend('v_{ctrl}^{out}', 'i_{out}^{pu}','Location','best');
xlabel('s');
ylabel('pu');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print(gcf, '-dpdf', 'fig2_without_compensation.pdf');
% print('without_comp_fig_5','-depsc');


figure;
plot(time_tc,is,'LineWidth',tratto1); 
title('Internal resonance current');
legend('i_{s}','Location','best');
xlabel('s');
ylabel('A');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t1c time_tc(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print(gcf, '-dpdf', 'fig2_without_compensation.pdf');
% print('without_comp_fig_6','-depsc');

figure;
plot(time_tc,vin_s,'LineWidth',tratto1); 
title('Internal resonance current');
legend('v_{s}^{in}','Location','best');
xlabel('s');
ylabel('V');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t1c time_tc(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print('without_comp_fig_7','-depsc');



return

figure;
subplot 211
plot(time_ts,h6_amplitude,'LineWidth',tratto1); 
title('H6 harmonic amplitude estimation - without compensation');
legend('i_{out}^{6th}','Location','NorthWest');
xlabel('s');
ylabel('pu');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
subplot 212
plot(time_ts,h6_freq,'LineWidth',tratto1); 
title('H6 harmonic frequency estimation - without compensation');
legend('f^{6th}','Location','NorthWest');
xlabel('s');
ylabel('Hz');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print(gcf, '-dpdf', 'fig3_without_compensation.pdf');

figure;
subplot 211
plot(time_ts,h2_amplitude,'LineWidth',tratto1); 
title('H2 harmonic amplitude estimation - without compensation');
legend('i_{out}^{2th}','Location','NorthWest');
xlabel('s');
ylabel('pu');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
subplot 212
plot(time_ts,h2_freq,'LineWidth',tratto1); 
title('H2 harmonic frequency estimation - without compensation');
legend('f^{2th}','Location','NorthWest');
xlabel('s');
ylabel('Hz');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print(gcf, '-dpdf', 'fig3_without_compensation.pdf');

figure;
subplot 211
plot(time_ts,h4_amplitude,'LineWidth',tratto1); 
title('H4 harmonic amplitude estimation - without compensation');
legend('i_{out}^{4th}','Location','NorthWest');
xlabel('s');
ylabel('pu');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
subplot 212
plot(time_ts,h4_freq,'LineWidth',tratto1); 
title('H4 harmonic frequency estimation - without compensation');
legend('f^{4th}','Location','NorthWest');
xlabel('s');
ylabel('Hz');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print(gcf, '-dpdf', 'fig3_without_compensation.pdf');

figure;
subplot 211
plot(time_ts,h8_amplitude,'LineWidth',tratto1); 
title('H8 harmonic amplitude estimation - without compensation');
legend('i_{out}^{8th}','Location','NorthWest');
xlabel('s');
ylabel('pu');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
subplot 212
plot(time_ts,h8_freq,'LineWidth',tratto1); 
title('H8 harmonic frequency estimation - without compensation');
legend('f^{8th}','Location','NorthWest');
xlabel('s');
ylabel('Hz');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print(gcf, '-dpdf', 'fig3_without_compensation.pdf');

figure;
subplot 211
plot(time_ts,h10_amplitude,'LineWidth',tratto1); 
title('H10 harmonic amplitude estimation - without compensation');
legend('i_{out}^{10th}','Location','NorthWest');
xlabel('s');
ylabel('pu');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
subplot 212
plot(time_ts,h10_freq,'LineWidth',tratto1); 
title('H10 harmonic frequency estimation - without compensation');
legend('f^{10th}','Location','NorthWest');
xlabel('s');
ylabel('Hz');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print(gcf, '-dpdf', 'fig3_without_compensation.pdf');

figure;
subplot 211
plot(time_ts,h12_amplitude,'LineWidth',tratto1); 
title('H12 harmonic amplitude estimation - without compensation');
legend('i_{out}^{12th}','Location','NorthWest');
xlabel('s');
ylabel('pu');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
subplot 212
plot(time_ts,h12_freq,'LineWidth',tratto1); 
title('H12 harmonic frequency estimation - without compensation');
legend('f^{12th}','Location','NorthWest');
xlabel('s');
ylabel('Hz');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print(gcf, '-dpdf', 'fig3_without_compensation.pdf');

figure;
subplot 211
plot(time_ts,h14_amplitude,'LineWidth',tratto1); 
title('H14 harmonic amplitude estimation - without compensation');
legend('i_{out}^{14th}','Location','NorthWest');
xlabel('s');
ylabel('pu');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
subplot 212
plot(time_ts,h14_freq,'LineWidth',tratto1); 
title('H14 harmonic frequency estimation - without compensation');
legend('f^{14th}','Location','NorthWest');
xlabel('s');
ylabel('Hz');
% set(gca,'ylim',[0 650]);
set(gca,'xlim',[t0s time_ts(end)]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
% print(gcf, '-dpdf', 'fig3_without_compensation.pdf');
