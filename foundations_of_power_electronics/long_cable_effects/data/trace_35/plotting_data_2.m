clear
close all
clc

load trace_35.mat;

tc = 1/125e6;
Nc = length(udc_pgnd);
time = [0:tc:(Nc-1)*tc];

tratto1=1;
tratto2=2;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = time(end) - 1e-3;
t2c = time(end);
fontsize_title = 14;
fontsize_legend = 14;
fontsize_axis = 14;


figure;
plot(time,udc_pgnd,'-','LineWidth',tratto1,'Color',colore1);
title('DClink Voltage p(+) - GND','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{dc}^\mathrm{p-gnd}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca, 'xlim',[t1c t2c]);
set(gca,'ylim',[-250 1000]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('trace_35_b','-depsc');
print('trace_35_b','-djpeg');
