
SKM1700MB20R4S2I4
% Rds_on = 1.04e-3; % Rds [Ohm]
% Vdon_diode = 4; % V
% Vgamma = 4; % V
% Rdon_diode = 1.85e-3; % V
% Eon = 77e-3; % J @ Tj = 125°C
% Eoff = 108e-3; % J @ Tj = 125°C
% Eerr = 9.7e-3; % J @ Tj = 125°C
% Voff_sw_losses = 1300; % V
% Ion_sw_losses = 1000; % A
% 
% JunctionTermalMass = 2; % J/K
% Rtim = 0.01;
% Rth_mosfet_JC = 19/1000; % K/W
% Rth_mosfet_CH = 6/1000; % K/W
% Rth_mosfet_JH = Rtim + Rth_mosfet_JC + Rth_mosfet_CH % K/W
% Lstray_module = 12e-9;
% 
% Irr = 475;
% Csnubber = Irr^2*Lstray_module/Vdc_nom^2
% Rsnubber = 1/(Csnubber*fPWM_INV)/5
FF650R17IE4
Vth = 5.5; % [V]
Rce_on = 0.15e-3; % Rce_on [Ohm]
Vce_sat = 2.35; % Vce_sat [V]
Vdon_diode = 2; % [V]
Rdon_diode = 0.15e-3; % [Ohm]
Eon = 300e-3; % J @ Tj = 125°C
Eoff = 205e-3; % J @ Tj = 125°C
Erec = 135e-3; % J @ Tj = 125°C
Voff_sw_losses = 900; % V
Ion_sw_losses = 650; % A

JunctionTermalMass = 2; % J/K
Rtim = 0.01;
Rth_switch_JC = 36/1000; % K/W
Rth_switch_CH = 14/1000; % K/W
Rth_switch_JH = Rtim + Rth_switch_JC + Rth_switch_CH % K/W
Rth_diode_JC = 72/1000; % K/W
Rth_diode_CH = 27/1000; % K/W
Rth_diode_JH = Rtim + Rth_diode_JC + Rth_diode_CH % K/W
Lstray_module = 18e-9;

Irr = 475;
Csnubber = Irr^2*Lstray_module/Vdc_nom^2
Rsnubber = 1/(Csnubber*fPWM_INV)/5