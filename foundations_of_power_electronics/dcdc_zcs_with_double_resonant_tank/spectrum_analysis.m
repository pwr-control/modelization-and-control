
sig_fft=iout_meas;

figure; 
subplot 211
plot(time_tc,sig_fft,'-k','LineWidth',2);
title('Output Current','Interpreter','latex');
ylabel('$A$','Interpreter','latex');
xlabel('$s$','Interpreter','latex');
set(gca,'xlim',[time_tc(1) time_tc(end)]);
set(gca,'ylim',[85 115]);
grid on
subplot 212
plot(time_tc,vout_meas,'-k','LineWidth',2);
title('Output Voltage','Interpreter','latex');
ylabel('$V$','Interpreter','latex');
xlabel('$s$','Interpreter','latex');
set(gca,'xlim',[time_tc(1) time_tc(end)]);
set(gca,'ylim',[5 20]);
grid on
print('vi_for_spectrum','-depsc');


Nfft=length(sig_fft)+1;
u1=Nfft*tc
f_sig=fft(sig_fft,Nfft);
Xrange=[f_sig(1)/Nfft f_sig(2:Nfft/2)'/(Nfft/2)];
freq=[0:1/u1:Nfft/2/u1-1/u1]';

figure; 
subplot 211
bar(freq,abs(Xrange),'k'); 
xlim([50 350]);
set(gca,'ylim',[0 8]);
grid;
xlabel('$Hz$','Interpreter','latex');
ylabel('$A$','Interpreter','latex');
title('Output Current Spectrum','Interpreter','latex');
subplot 212
bar(freq,abs(Xrange),'k'); 
xlim([350 1000]);
set(gca,'ylim',[0 2]);
grid;
xlabel('$Hz$','Interpreter','latex');
ylabel('$A$','Interpreter','latex');
title('Output Current Spectrum','Interpreter','latex');
print('output_current_spectrum','-depsc');
