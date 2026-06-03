close all;

N1=Nc-floor(200e-3/tc);
N2=Nc;
segnale = mosfet_power_losses_zvs_sim(N1:N2);
time = t(N1:N2);

tratto1=2;
tratto2=2;
tratto3=2;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
t1c = time(1);
t2c = time(end);
f1 = 0;
f2 = 50*2500;
fontsize_plotting = 12;


figure; 
subplot 211
plot(time,segnale,'-','LineWidth',tratto2,'Color',colore1);
title('ZVS Mosfet Power Loss','Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$p/W$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$p_{loss}^{Q1}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on

segnale = mosfet_JH_temp_zvs_sim(N1:N2);

subplot 212
plot(time,segnale,'-','LineWidth',tratto2,'Color',colore1);
title('ZVS Mosfet Junction-Heatsink Temperature','Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$T/K$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$T_{JH}^{Q1}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
print('zvs_mosfet_power_loss','-depsc');

