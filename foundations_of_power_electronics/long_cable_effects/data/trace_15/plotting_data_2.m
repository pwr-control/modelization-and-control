clear
% close all
clc

load trace_15.mat;

tc = 1/1.25e6;
Nc = length(Var2);
time = [0:tc:(Nc-1)*tc];

init_offset = 1e3;
time = time(1e3:Nc);
Var2 = Var2(1e3:Nc);
Var3 = Var3(1e3:Nc);
Var4 = Var4(1e3:Nc);

tratto1 = 1;
tratto2 = 2;
tratto3 = 3;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = time(end) - 20e-3;
t2c = time(end);

fontsize_title = 14;
fontsize_legend = 14;
fontsize_axis = 14;


figure; 
plot(time,Var3,'-k','LineWidth',tratto1);
title('Zero sequence inverter output current','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{z}^\mathrm{inv}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca, 'xlim',[t1c t2c]);
set(gca,'ylim',[-50 50]);
grid on
h=gcf;
set(h,'PaperOrientation','landscape');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);
print('trace_15_b','-depsc');
print('trace_15_b','-djpeg');
