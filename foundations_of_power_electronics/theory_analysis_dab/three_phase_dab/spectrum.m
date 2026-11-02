% spectrum: phase current of the transformer (phase u, side 1) in the logged window of a run
% Reads the logged variables of three_phase_dab.slx (time_tc_sim, i1_dab_transformer_modA_sim) and param.
% Harmonics of the six-step DAB3: h = 6k +- 1 of fs (no triplen harmonics with the floating star points).
close all;
fs = param.dab3.fs;
Tc = param.glb_time.tc*param.glb_time.decimation_tc;
N2 = numel(time_tc_sim);
n_per = floor((time_tc_sim(end) - time_tc_sim(1))*fs/2);            % whole periods in the last half of the window
N1 = max(N2 - round(n_per/fs/Tc) + 1, 1);
segnale = i1_dab_transformer_modA_sim(N1:N2, 1);
time = time_tc_sim(N1:N2);

figure;
plot(time, segnale);
title('Transformer current, phase u, side 1')
ylabel('A');
xlabel('sec');
set(gca, 'xlim', [time(1) time(end)]);
grid on

Nfft = length(segnale);
u1 = Nfft*Tc;
f_sig = fft(segnale, Nfft);
Xrange = [f_sig(1)/Nfft f_sig(2:floor(Nfft/2)).'/(Nfft/2)];
freq = (0:numel(Xrange)-1)'/u1;

figure;
subplot 211
bar(freq/fs, abs(Xrange));
xlim([0 26]);
grid;
xlabel('h = f/f_s');
ylabel('A');
title('Transformer current spectrum (amplitude), phase u, side 1')
subplot 212
bar(freq/fs, abs(Xrange));
xlim([0.5 1.5]);
grid;
xlabel('h = f/f_s');
ylabel('A');
title('Fundamental')
print('spectrum_i1_dab_transformer', '-depsc');
