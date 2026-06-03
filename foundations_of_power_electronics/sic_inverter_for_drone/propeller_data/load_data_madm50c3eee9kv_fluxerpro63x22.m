clc
close all


% n = [0 200 400 600 839 981 1115 1246 1368 1484 1599 1703 1807 1893 2008 ...
%     2102 2192 2269 2291];
% thrust = [0 300 600 900 11397 15739 20516 25724 31815 36234 43571 48121 ...
%     55617 59131 67183 74168 81164 87937 92573];
% torque = [0 1 2 4 7.88 10.96 14.86 17.98 21.83 24.95 29.94 34.16 39.01 ...
%     42.63 49.56 54.42 59.13 62.02 63.90];
% input_power = [0 200 400 750 1090 1472 2328 3219 4071 5052 6114 7471 8968 9986 11586 ...
%     13861 15868 19102 19494];
% shaft_power = [0 100 250 500 697 1472 1730 2360 3107 3993 4991 6168 7367 8365 10498 ...
%     11991 13572 15073 15334];

n = [839 981 1115 1246 1368 1484 1599 1703 1807 1893 2008 ...
    2102 2192 2269 2291];
thrust = [11397 15739 20516 25724 31815 36234 43571 48121 ...
    55617 59131 67183 74168 81164 87937 92573];
torque = [7.88 10.96 14.86 17.98 21.83 24.95 29.94 34.16 39.01 ...
    42.63 49.56 54.42 59.13 62.02 63.90];
input_power = [1090 1472 2328 3219 4071 5052 6114 7471 8968 9986 11586 ...
    13861 15868 19102 19494];
shaft_power = [697 1472 1730 2360 3107 3993 4991 6168 7367 8365 10498 ...
    11991 13572 15073 15334];
idc = [2.74 3,7 5,85 8.09 10.23 12.7 15.38 18.79 22.56 25.18 29.22 34.95 ...
    40.04 48.19 49.14];

p_bez = 20e3;
q_bez = 93000;
tau_bez = 64;

tratto1=3;
colore1 = [0.15 0.15 0.15];
colore2 = [0.35 0.35 0.35];
colore3 = [0.5 0.5 0.5];
colore4 = [0.8 0.8 0.8];
fontsize_plotting = 12;

figure;
plot(n,thrust./q_bez,'--','LineWidth',tratto1,'Color',colore1);
hold on 
plot(n,torque./tau_bez,'-','LineWidth',tratto1,'Color',colore2);
hold on 
plot(n,input_power./p_bez,'-','LineWidth',tratto1,'Color',colore3);
hold on 
plot(n,shaft_power./p_bez,'-','LineWidth',tratto1,'Color',colore4);
hold off
title('','Interpreter','latex','FontSize',fontsize_plotting);
legend('$Q$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
% xlabel('$t/s$','Interpreter','latex');
ylabel('$Q/kgf$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[n(1) n(end)]);
grid on
