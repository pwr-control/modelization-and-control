clear all;
close all;
clc

%% application voltage
V_application_voltage = 480;
fnom = 60; % frequenza nominale della linea trifase

%% calcolo impedanza di linea
Icc = 50e3; % corrente di corto circuito
Vnom = V_application_voltage; % tensione nominale della linea trifase (quindi fase-fase)

Lline = Vnom/sqrt(3)/(2*pi*fnom*Icc); % impedenza di linea

%% calcolo parametri del circuito equivalente trafo 
P_trafo_nom = 1250e3;
Vtrafo_nom_1 = Vnom; % tensione nominale primario
Vtrafo_nom_2 = Vnom; % tensione nominale secondario
f_trafo_nom = fnom; % frequenza nominale
Ucc = 0.05; % tensione di corto circuito

Vtrafo_phase_1 = Vtrafo_nom_1/sqrt(3);
Vtrafo_phase_2 = Vtrafo_nom_2/sqrt(3);

Itrafo_nom_1 = P_trafo_nom/sqrt(3)/Vtrafo_nom_1;
Itrafo_nom_2 = P_trafo_nom/sqrt(3)/Vtrafo_nom_2;

I_magnetizzante = 0.01 * Itrafo_nom_1;
Lm = Vtrafo_phase_1/I_magnetizzante/(2*pi*f_trafo_nom);

%% induttanza equivalente trafo
Lsigma = Ucc*Vtrafo_phase_1/(2*pi*f_trafo_nom*Itrafo_nom_1)
% Ld1 = Lsigma/2;
% Ld2 = Lsigma/2*(n2/n1)^2;






