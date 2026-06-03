clc
close all

N1 = Nc-floor(0.0125/tc);
N2 = Nc;
sig_fft = iout2_uvw_sim(N1:N2,2);
Nfft = length(sig_fft);
u1 = Nfft*tc;
f_sig = fft(sig_fft,Nfft);
Xrange = [f_sig(1)/Nfft f_sig(2:floor(Nfft/2))'/(floor(Nfft/2))];
freq = [0:1/u1:Nfft/2/u1-1/u1]';

figure;
plot(t_tc_sim(N1:N2),iout2_uvw_sim(N1:N2,1),'-k','LineWidth',tratto1);
hold on
plot(t_tc_sim(N1:N2),iout2_uvw_sim(N1:N2,2),'-','LineWidth',tratto2,'Color',colore1);
hold on
plot(t_tc_sim(N1:N2),iout2_uvw_sim(N1:N2,3),'-','LineWidth',tratto2,'Color',colore2);
hold off
title('Output Currents in Ampere','Interpreter','latex','FontSize',fontsize_plotting);
legend('$i_{out}^u$','$i_{out}^v$','$i_{out}^w$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
% set(gca,'ylim',[-1.5 1.5]);
set(gca,'xlim',[t_tc_sim(N1) t_tc_sim(N2)]);
grid on
print('output_phase_currents','-depsc');
print('output_phase_currents','-dpdf');

figure; 
subplot 211
plot(freq,abs(Xrange)./i_nom*100,'-k','LineWidth',tratto1);
xlim([0 2000]);
% ylim([0 45]);
grid;
xlabel('f/Hz');
ylabel('i/%');
title('Output Phase Current Spectrum in percent');
subplot 212
plot(freq,abs(Xrange)./i_nom*100,'-k','LineWidth',tratto1);
xlim([2000 20000]);
% ylim([0 0.75]);
grid;
xlabel('f/Hz');
ylabel('i/%');
title('Output Phase Current Spectrum in percent');
print('output_phase_current_spectrum','-depsc');
print('output_phase_current_spectrum','-dpdf');


