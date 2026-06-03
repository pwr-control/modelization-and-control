clc
close all

tratto1=2.2;
tratto2=2.2;
tratto3=1;
colore1 = [0.25 0.25 0.25];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
fontsize_plotting = 12;

N1 = Nc-floor(0.0125/tc);
N2 = Nc;
sig_fft = i_CFu_sim(N1:N2);
Nfft = length(sig_fft);
u1 = Nfft*tc;
f_sig = fft(sig_fft,Nfft);
Xrange = [f_sig(1)/Nfft f_sig(2:floor(Nfft/2))'/(floor(Nfft/2))];
freq = [0:1/u1:Nfft/2/u1-1/u1]';

figure;
plot(t_tc_sim(N1:N2),i_CFu_sim(N1:N2),'-k','LineWidth',tratto1);
title('Output Filter Capacitor Current in A','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{CFu}^u$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t_tc_sim(N1) t_tc_sim(N2)]);
grid on
print('output_filter_capacitor_current','-depsc');
print('output_filter_capacitor_current','-dpdf');

figure; 
subplot 211
plot(freq,abs(Xrange),'-k','LineWidth',tratto1);
xlim([0 2000]);
% ylim([0 45]);
grid;
xlabel('f/Hz');
ylabel('i/A');
title('Output Filter Capacitor Current Spectrum in Ampere');
subplot 212
plot(freq,abs(Xrange),'-k','LineWidth',tratto1);
xlim([2000 20000]);
% ylim([0 0.75]);
grid;
xlabel('f/Hz');
ylabel('i/A');
title('Output Filter Capacitor Current Spectrum in Ampere');
print('output_filter_capacitor_current_spectrum','-depsc');
print('output_filter_capacitor_current_spectrum','-dpdf');


