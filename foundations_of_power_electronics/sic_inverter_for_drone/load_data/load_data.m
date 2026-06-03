clc

% % Misura tra una fase e due fasi cortocircuitate.
% % La Ls è dell’ordine di 130 uH - 160 uH (ruotando il rotore) alla frequenza di 1.25 kHz. A 30 kHz la Ls si riduce del 10%.
% % Rs = 106 mOhm @ 1.25 kHz
% % Rs = 760 mOhm @ 5 kHz
% % Rs = 9.06 Ohm @ 30 kHz
% % Provato con tutte le combinazioni di fare e ruotando sempre il rotore.

%% motor-load resiste load 
Rload_eq = 2;
Lload_eq = 10e-6;

%% RL load 
R_RLload = 10e-3;
L_RLload = 360e-6;