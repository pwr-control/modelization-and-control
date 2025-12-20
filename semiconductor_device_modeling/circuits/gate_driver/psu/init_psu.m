clear all
close all
clc
beep off
options = bodeoptions;
options.FreqUnits = 'Hz';

ts = 1e-8;

Vin = 15;
if Vin >= 24
    N1 = 12;
    Lm = 750e-6;
    Ld = 1.2e-6;
else
    N1 = 6;
    Lm = 450e-6;
    Ld = 0.9e-6;
end

N2 = 12;

fws = 510e3;
Cres = 4.7e-6
% Cres = 22e-9
Cf = 4.7e-6
% Cin = 680e-9;
Cin = 4.7e-6;

Vgs =  16;
Qgate = 7e-6;
fPWM = 4e3;
Pgate = Vgs*Qgate*fPWM
Vout = Vgs+12;
Rload = Vout^2/Pgate

Vz = 16;
Rz = 810;

%%
s = tf('s');

L1 = 11e-6;
Rp1 = 12e3;
C1 = 4.7e-6*3;
ZC1 = 1/s/C1;
Zp1 = Rp1*ZC1/(Rp1+ZC1);
H1 = minreal(Zp1/(s*L1+Zp1));

L2 = 120e-6;
Rp2 = 61e3;
C2 = 22e-6;
RC2 = 0.1;
ZC2 = 1/s/C2+RC2;
Zp2 = Rp2*ZC2/(Rp2+ZC2);
H2 = minreal(Zp2/(s*L2+Zp2));

figure; bode(H1,H2,options); grid on


% open_system('psu');