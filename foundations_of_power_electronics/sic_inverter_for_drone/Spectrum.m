clc
close all

N1 = Nc-floor(0.1/tc);
N2 = Nc;
sig_fft = iuvw_inverter_sim(N1:N2,2);
Nfft = length(sig_fft);
u1 = Nfft*tc;
f_sig = fft(sig_fft,Nfft);
Xrange = [f_sig(1)/Nfft f_sig(2:floor(Nfft/2))'/(floor(Nfft/2))];
freq = [0:1/u1:Nfft/2/u1-1/u1]';

figure;
plot(t_tc_sim(N1:N2),iuvw_inverter_sim(N1:N2,1),'-k','LineWidth',tratto1);
hold on
plot(t_tc_sim(N1:N2),iuvw_inverter_sim(N1:N2,2),'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim(N1:N2),iuvw_inverter_sim(N1:N2,3),'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Motor Current in A','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_u$','$i_v$','$i_w$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t_tc_sim(end)-4/1135 t_tc_sim(end)]);
grid on

figure; 
subplot 211
plot(freq,abs(Xrange),'LineWidth',1.8);
xlim([0 4000]);
% ylim([0 45]);
grid;
xlabel('Hz');
ylabel('A');
title('Inverter Current Phase Spectrum');
subplot 212
plot(freq,abs(Xrange),'LineWidth',1.8);
xlim([4000 40000]);
% ylim([0 0.75]);
grid;
xlabel('Hz');
ylabel('A');
title('Inverter Current Phase Spectrum');
print('inverter_current_phase_spectrum','-dpdf');


