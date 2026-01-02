% clear all
close all
clc

tratto1=1;
tratto2=2;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];

N1c = floor(Nc/200/(freq/f_grid));
N2c = floor(Nc);
t1c = time_tc_sim(N2c) - N1c*tc;
t2c = time_tc_sim(N2c);

N3c = floor(Nc/1000/(freq/f_grid));
N4c = floor(Nc);
t3c = time_tc_sim(N4c) - N3c*tc;
t4c = time_tc_sim(N4c);

N1s = floor(Nc/10/(freq/f_grid));
N2s = floor(Nc);
t1s = time_tc_sim(N2s) - N1s*tc;
t2s = time_tc_sim(N2s);

N3s = floor(Nc/10);
N4s = floor(Nc);
t3s = time_tc_sim(N4s) - N3s*tc;
t4s = time_tc_sim(N4s);


fontsize_plotting = 14;

figure(1);
subplot 211
plot(time_tc_sim,ig_abc_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,ig_abc_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,ig_abc_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Grid Phase Currents');
legend('$i_{r}^{g}$','$i_{s}^{g}$','$i_{t}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
subplot 212
plot(time_tc_sim,ug_abc_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,ug_abc_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,ug_abc_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Grid Phase Voltages');
legend('$u_{r}^{g}$','$u_{s}^{g}$','$u_{t}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('grid_quantities','-depsc');
movefile('grid_quantities.eps', 'figures');

figure(2);
subplot 211
plot(time_tc_sim,i_line_abc_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,i_line_abc_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,i_line_abc_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Line Phase Currents');
legend('$i_{r}^{l}$','$i_{s}^{l}$','$i_{t}^{l}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
subplot 212
plot(time_tc_sim,u_line_abc_sim(:,1),'-','LineWidth',tratto3,'Color',colore1);
hold on
plot(time_tc_sim,u_line_abc_sim(:,2),'-','LineWidth',tratto3,'Color',colore2);
hold on
plot(time_tc_sim,u_line_abc_sim(:,3),'-','LineWidth',tratto3,'Color',colore3);
hold off
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Line Phase Voltages');
legend('$u_{r}^{l}$','$u_{s}^{l}$','$u_{t}^{l}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t3s t4s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('line_quantities','-depsc');
movefile('line_quantities.eps', 'figures');

figure(3);
subplot 211
plot(time_tc_sim,inverter_current_output_sim,'-','LineWidth',tratto3,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter Output Current');
legend('$i_{out}^{inv}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
subplot 212
plot(time_tc_sim,inverter_voltage_output_sim,'-','LineWidth',tratto2,'Color',colore1);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter Output Voltage');
legend('$u_{out}^{inv}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
set(gca,'xlim',[t1s t2s]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('inverter_outputs','-depsc');
movefile('inverter_outputs.eps', 'figures');

x1 = time_tc_sim(N1s:N2s);
y1 = inverter_current_output_sim(N1s:N2s);
x2 = time_tc_sim(N3c:N4c);
y2 = inverter_current_output_sim(N3c:N4c);
figure(4);
plot(x1,y1,'-','LineWidth',tratto3,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter Output Current');
legend('$i_{out}^{inv}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
grid on
set(gca,'xlim',[t1s t2s]);
ax2 = axes('Position',[0.7,0.7,0.25,0.25]);
plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
grid on
set(gca,'xlim',[t3c t4c]);
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('inverter_output_current','-depsc');
movefile('inverter_output_current.eps', 'figures');

x1 = time_tc_sim(N1s:N2s);
y1 = inverter_voltage_output_sim(N1s:N2s);
x2 = time_tc_sim(N1c:N2c);
y2 = inverter_voltage_output_sim(N1c:N2c);
figure(5);
plot(x1,y1,'-','LineWidth',tratto2,'Color',colore1);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
title('Inverter Output Current');
legend('$i_{out}^{inv}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
grid on
set(gca,'xlim',[t1s t2s]);
ax2 = axes('Position',[0.7,0.7,0.25,0.25]);
plot(ax2,x2,y2,'-','LineWidth',tratto2,'Color',colore1);
grid on
set(gca,'xlim',[t1c t2c]);
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('inverter_output_voltage','-depsc');
movefile('inverter_output_voltage.eps', 'figures');