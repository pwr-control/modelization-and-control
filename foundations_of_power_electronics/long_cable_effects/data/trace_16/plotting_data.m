clear
close all
clc

load trace_16.mat;

tc = 1/625e3;
Nc = length(Var2);
time = [0:tc:(Nc-1)*tc];

init_offset = 1e3;
time = time(1e3:Nc);
Var2 = Var2(1e3:Nc);
Var3 = Var3(1e3:Nc);
Var4 = Var4(1e3:Nc);

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
subplot 311
plot(time, -Var2);
title('Phase Current (INV) $\mathrm{LD_2}$','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{inv}^\mathrm{z}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
% xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca, 'xlim',[time(1) time(end)]);
% set(gca, 'ylim',[-50 50]);
grid on
subplot 312
plot(time, Var3);
title('Zero Current (INV) $\mathrm{LD_2}$','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{afe}^{z}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
% xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca, 'xlim',[time(1) time(end)]);
set(gca, 'ylim',[-50 50]);
grid on
subplot 313
plot(time, Var4);
title('Grid Voltage $\mathrm{LD_2}$','Interpreter','latex','FontSize',fontsize_title);
legend('$u_\mathrm{grid}^\mathrm{u}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
ylabel('Voltage - [V]','Interpreter','latex','FontSize', fontsize_axis);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
set(gca, 'xlim',[time(1) time(end)]);
grid on
print('trace_16','-depsc');
print('trace_16','-djpeg');
