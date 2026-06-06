
%% grid_emulator setup
vp_xi_pu = 1;
vn_xi_pu = 0;
vn_eta_pu = 0;

%% grid emulator output transformer
Ptrafo = 1250e3;
I0 = 5; % no load current
Vcc_perc = 5.84; %cc voltage percente
Vline1 = 20e3; % primary voltage
Vline2 = 680; % secondary voltage
Vphase1 = Vline1/sqrt(3);
Vphase2 = Vline2/sqrt(3);
f_grid = 50;
w_grid = f_grid*2*pi;
omega_grid_emulator_nom = w_grid;
omega_grid_nom = w_grid;

I2nom_trafo = Ptrafo/Vline2/sqrt(3);
Ld2_trafo = Vphase2/(100/Vcc_perc)/I2nom_trafo/(w_grid); %leakage inductace
Rd2_trafo = 0.1*Ptrafo/3/I2nom_trafo^2; 
Lmu2_trafo = Vphase2/I0/(w_grid); %magentization inductance
Piron = 1.4e3;
Rm2_trafo = Vphase2^2/(Piron/3);
psi_trafo = Lmu2_trafo*I0*sqrt(2);


