clear
close all
clc

load trace_22.mat;

tc = 1/250e3;
Nc = length(Var2);
time = [0:tc:(Nc-1)*tc];

init_offset = 1e3;
time = time(1e3:Nc);
Var2 = Var2(1e3:Nc);
% Var3 = Var3(1e3:Nc);

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
% subplot 211
plot(time, -Var2);
title('INV current LD 1','Interpreter','latex','FontSize',fontsize_title);
legend('$i_\mathrm{inv}^\mathrm{u}$','Location','northeastoutside',...
    'Interpreter','latex','FontSize',fontsize_legend);
ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
grid on
% subplot 212
% plot(time, Var3);
% title('AFE current LD 1','Interpreter','latex','FontSize',fontsize_title);
% legend('$i_\mathrm{afe}^\mathrm{u}$','Location','northeastoutside',...
%     'Interpreter','latex','FontSize',fontsize_legend);
% ylabel('Current - [A]','Interpreter','latex','FontSize', fontsize_axis);
% xlabel('Time - [s]','Interpreter','latex','FontSize', fontsize_axis);
% grid on
print('trace_22','-depsc');
print('trace_22','-djpeg');
