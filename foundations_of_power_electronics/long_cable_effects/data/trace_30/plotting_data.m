clear
close all
clc

load trace_30.mat;

tc = 1/125e6;
Nc = length(Var2);
time = [0:tc:(Nc-1)*tc];

tratto1=1;
tratto2=2;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = time(end) - Nc*tc;
t2c = time(end);
fontsize_title = 18;
fontsize_legend = 18;
fontsize_axis = 16;


figure;
plot(time, Var2);
title('DClink Voltage p(+) - GND','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{dc}^\mathrm{p-gnd}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
grid on
print('trace_30','-depsc');
print('trace_30','-djpeg');
