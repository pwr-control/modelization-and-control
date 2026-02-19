close all;

tratto1=1;
tratto2=2;
tratto3=3;
colore1 = [0.25 0.25 0.25];
colore2b = [0.35 0.35 0.35];
colore2 = [0.5 0.5 0.5];
colore3 = [0.75 0.75 0.75];
fontsize_plotting = 14;

N1=1;
N2=Nc;
signal = iuvw_psm_sim(N1:N2,2);
time = [0:tc:(Nc-1)*tc];

sig_fft = signal;
Nfft = length(sig_fft)-1;
u1 = Nfft*tc;
f_sig=fft(sig_fft,Nfft);
Xrange=[f_sig(1)/Nfft f_sig(2:Nfft/2)'/(Nfft/2)];
freq=[0:1/u1:Nfft/2/u1-1/u1]';

figure; 
subplot 211
plot(time,signal,'-','LineWidth',tratto1,'Color',colore1);
title('Motor Phase U Current')
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$i_{u}^{m}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
legend
set(gca,'xlim',[time(1) time(end)]);
grid on
subplot 212
plot(freq,abs(Xrange),'-','LineWidth',tratto1,'Color',colore1);
xlim([100 12e3]);
grid;
xlabel('$f/Hz$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$F(i_{u}^{m})$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
title('Motor Phase U Current Spectrum')
% print('spectrum_motor_current_u','-depsc');
% movefile('spectrum_motor_current_u.eps', 'figures');


signal = us_uvw_sim_mod1(N1:N2,1);
time = time_tc_sim;

sig_fft = signal;
Nfft = length(sig_fft)-1;
u1 = Nfft*tc;
f_sig=fft(sig_fft,Nfft);
Xrange=[f_sig(1)/Nfft f_sig(2:Nfft/2)'/(Nfft/2)];
freq=[0:1/u1:Nfft/2/u1-1/u1]';

figure; 
subplot 211
plot(time,signal,'-','LineWidth',tratto1,'Color',colore1);
title('Motor Phase U Current')
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$i_{u}^{m}$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
xlabel('$t/s$','Interpreter','latex','FontSize', fontsize_plotting);
legend
set(gca,'xlim',[time(1) time(end)]);
grid on
subplot 212
plot(freq,abs(Xrange),'-','LineWidth',tratto1,'Color',colore1);
xlim([100 24e3]);
grid;
xlabel('$f/Hz$','Interpreter','latex','FontSize', fontsize_plotting);
ylabel('$i/A$','Interpreter','latex','FontSize', fontsize_plotting);
legend('$F(i_{u}^{m})$','Location','northwestoutside',...
    'Interpreter','latex','FontSize',fontsize_plotting);
title('Motor Phase U Current Spectrum')
% print('spectrum_motor_current_u','-depsc');
% movefile('spectrum_motor_current_u.eps', 'figures');

