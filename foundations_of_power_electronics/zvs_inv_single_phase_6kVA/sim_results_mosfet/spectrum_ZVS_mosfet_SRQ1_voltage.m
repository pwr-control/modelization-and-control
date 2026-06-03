close all;

N1=Nc-floor(200e-3/tc);
N2=Nc;
segnale = u_device_Q1_mosfet_rect_sim(N1:N2);
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

sig_fft=segnale;
Nfft=length(sig_fft)+1;
u1=Nfft*tc;
f_sig=fft(sig_fft,Nfft);
Xrange=[f_sig(1)/Nfft f_sig(2:Nfft/2)'/(Nfft/2)];
freq=[0:1/u1:Nfft/2/u1-1/u1]';

figure; 
subplot 211
plot(time,segnale,'-','LineWidth',tratto2,'Color',colore1);
title('ZVS Device SRQ1 Voltage','Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$u_{sr1}^{zvs}$','Location','best',...
    'Interpreter','latex','FontSize',fontsize_plotting);
set(gca,'xlim',[t1c t2c]);
grid on
subplot 212
plot(freq,abs(Xrange),'-','LineWidth',tratto2,'Color',colore1);
xlim([f1 f2]);
grid;
title('ZVS Device SRQ1 Voltage Spectrum','Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$f/Hz$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$u/V$','Interpreter','latex','FontSize', fontsize_plotting);
print('ZVS_device_SRQ1_voltage_spectrum','-depsc');

ZVS_device_SRQ1_voltage_rms = sqrt(mean(segnale.^2))


