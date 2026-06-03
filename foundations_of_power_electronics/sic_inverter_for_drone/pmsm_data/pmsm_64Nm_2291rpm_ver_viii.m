clc

% % Misura tra una fase e due fasi cortocircuitate.
% % La Ls è dell’ordine di 130 uH - 160 uH (ruotando il rotore) alla frequenza di 1.25 kHz. A 30 kHz la Ls si riduce del 10%.
% % Rs = 106 mOhm @ 1.25 kHz
% % Rs = 760 mOhm @ 5 kHz
% % Rs = 9.06 Ohm @ 30 kHz
% % Provato con tutte le combinazioni di fare e ruotando sempre il rotore.


%% PMSM paramerters
number_poles = 62;
pp = number_poles/2;
omega_m_bez = 2291; %rpm
omega_bez = omega_m_bez/60*2*pi*pp;

tau_bez = 64;
ibez = 64;

phi_m = 2/3*tau_bez/(number_poles/2)/ibez;
ubez = omega_bez*phi_m % ph peak nominal back EMF
% ubez = 200 % ph peak nominal back EMF

imax = ibez; %A
La = 80e-6;
Lb = 5e-6;
Ld = 3/2*(La-Lb)
Lq = 3/2*(La+Lb)
Lalpha = Ld;
Lbeta = Lq;
Ls = Lq;
Rs = 0.3;
Lmu = 5e-6;
Jm = 0.015; %kgm^2

%% per-unit model
Xbez=ubez/ibez;
Lbez=Xbez/omega_bez;
Rs_norm = Rs/Xbez;
Ld_norm = Ld/Lbez;
Lq_norm = Lq/Lbez;
Ls_norm = Ls/Lbez;
Lalpha_norm = Lalpha/Lbez;
Lbeta_norm = Lbeta/Lbez;
La_norm = La/Lbez;
Lb_norm = Lb/Lbez;
phi_bez = ubez/omega_bez;
phi_m_norm = phi_m/phi_bez;

torque_load_bearing = 5; %N*m
b = torque_load_bearing/(omega_m_bez/60*2*pi);
b_square = torque_load_bearing/(omega_m_bez/60*2*pi)^2;


%% check parameter motor and dclink voltage
omega_e = omega_bez;
omega_m = omega_e/(number_poles/2);
psi = phi_m;
id = 0;
iq = ibez;
torque = 3/2*number_poles/2*psi*iq
power_shaft = torque*omega_m;
vg = sqrt((omega_e*psi+omega_e*Ld*id+Rs*iq)^2+(omega_e*Lq*iq)^2)
udc_min = sqrt(3)*sqrt((omega_e*psi+omega_e*Ld*id+Rs*iq)^2+(omega_e*Lq*iq)^2)
u_motor_rms = sqrt(3/2)*sqrt((omega_e*psi+omega_e*Ld*id+Rs*iq)^2+(omega_e*Lq*iq)^2)
RPM_V = omega_m_bez/u_motor_rms;


