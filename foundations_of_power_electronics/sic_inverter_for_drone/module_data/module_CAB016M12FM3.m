%% CCB016M12FM3 data

Ron = 21.6e-3; % Rds [Ohm]
Vgamma = 5.5; % V

%% energy switching at Rg_ext = 4 Ohm
% Eon = 1.16e-3; % J @ Tj = 125°C
% Eoff = 0.54e-3; % J @ Tj = 125°C
% Eerr = 0.075e-3; % J @ Tj = 125°C
% Voff_sw_losses = 600; % V
% Ion_sw_losses = 80; % A

%% energy switching at Rg_ext = 8 Ohm
Eon = 1.56e-3; % J @ Tj = 125°C
Eoff = 0.84e-3; % J @ Tj = 125°C
Eerr = 0.075e-3; % J @ Tj = 125°C
Voff_sw_losses = 600; % V
Ion_sw_losses = 80; % A

JunctionTermalMass = 0.025; % J/K
Rtim = 0.5;
Rth_mosfet_JH = 0.642+Rtim; % K/W

