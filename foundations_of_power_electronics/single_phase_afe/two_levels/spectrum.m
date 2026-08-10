close all;
N1=Nc-floor(200e-3/tc);
N2=Nc;
signal = ig_abc_sim(N1:N2,1);
time = time_tc_sim(N1:N2);

sig_fft = signal;
Nfft = length(sig_fft)-1;
u1 = Nfft*tc;
f_sig=fft(sig_fft,Nfft);
Xrange=[f_sig(1)/Nfft f_sig(2:Nfft/2)'/(Nfft/2)];
freq=[0:1/u1:Nfft/2/u1-1/u1]';

figure; 
subplot 211
plot(time,signal,'-','LineWidth',tratto1,'Color',colore1);
title('Grid Phase R Current')
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$i_{r}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
legend
set(gca,'xlim',[time(1) time(end)]);
grid on
subplot 212
plot(freq,abs(Xrange),'-','LineWidth',tratto1,'Color',colore1);
xlim([10 850]);
grid;
xlabel('$f/Hz$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$F(i_{r}^{g})$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
title('Grid Phase R Current Spectrum')
print('spectrum_grid_current_r','-depsc');
movefile('spectrum_grid_current_r.eps', 'figures');


signal = ig_abc_sim(N1:N2,2);

sig_fft = signal;
Nfft = length(sig_fft)-1;
u1 = Nfft*tc;
f_sig=fft(sig_fft,Nfft);
Xrange=[f_sig(1)/Nfft f_sig(2:Nfft/2)'/(Nfft/2)];
freq=[0:1/u1:Nfft/2/u1-1/u1]';

figure; 
subplot 211
plot(time,signal,'-','LineWidth',tratto1,'Color',colore1);
title('Grid Phase S Current')
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$i_{s}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
legend
set(gca,'xlim',[time(1) time(end)]);
grid on
subplot 212
plot(freq,abs(Xrange),'-','LineWidth',tratto1,'Color',colore1);
xlim([10 850]);
grid;
xlabel('$f/Hz$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$F(i_{s}^{g})$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
title('Grid Phase S Current Spectrum')
print('spectrum_grid_current_s','-depsc');
movefile('spectrum_grid_current_s.eps', 'figures');

signal = ig_abc_sim(N1:N2,3);

sig_fft = signal;
Nfft = length(sig_fft)-1;
u1 = Nfft*tc;
f_sig=fft(sig_fft,Nfft);
Xrange=[f_sig(1)/Nfft f_sig(2:Nfft/2)'/(Nfft/2)];
freq=[0:1/u1:Nfft/2/u1-1/u1]';

figure; 
subplot 211
plot(time,signal,'-','LineWidth',tratto1,'Color',colore1);
title('Grid Phase T Current')
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$i_{t}^{g}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
legend
set(gca,'xlim',[time(1) time(end)]);
grid on
subplot 212
plot(freq,abs(Xrange),'-','LineWidth',tratto1,'Color',colore1);
xlim([10 850]);
grid;
xlabel('$f/Hz$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$F(i_{s}^{g})$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
title('Grid Phase T Current Spectrum')
print('spectrum_grid_current_t','-depsc');
movefile('spectrum_grid_current_t.eps', 'figures');
