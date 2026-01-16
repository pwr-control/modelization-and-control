
%% grid_emulator setup
% vp_xi_pu = 0.85;
% vn_xi_pu = 0.1;
% vn_eta_pu = 0.05;

vp_xi_pu = 1;
vn_xi_pu = 0;
vn_eta_pu = 0;

%% grid emulator output transformer
Ptrafo = 1600e3;
I0 = 5; % no load current
Vcc_perc = 6.6; %cc voltage percente
Vline1 = 690; % primary voltage
Vline2 = 690; % secondary voltage
Vphase1 = Vline1/sqrt(3);
Vphase2 = Vline2/sqrt(3);
% f_grid = 47.5;
% f_grid = 48.2;
f_grid = 50;
% f_grid = 51.5;
w_grid = f_grid*2*pi;
omega_grid_emulator_nom = w_grid;
omega_grid_nom = w_grid;
Inom_trafo=Ptrafo/Vline2/sqrt(3);
Ld2_trafo= Vphase2/(100/Vcc_perc)/Inom_trafo/(w_grid); %leakage inductace
Rd2_trafo = 0.05*Ptrafo/3/Inom_trafo^2; 
Lmu2_trafo= Vphase2/I0/(w_grid); %magentization inductance
Piron = 1.4e3;
Rm2_trafo = Vphase2^2/(Piron/3);
psi_trafo = Lmu2_trafo*I0*sqrt(2);

%% grid emulator others data
Vdc_bez = 1070; % DClink voltage reference
kp_vgrid = 10;
ki_vgrid = 45;
k_ff = 1;

%% voltage reference grid emulator
Igrid_phase_normalization_factor = 270*sqrt(2);
Vgrid_phase_normalization_factor = Vphase2*sqrt(2);
I_phase_normalization_factor = Igrid_phase_normalization_factor; % misura della corrente 
V_phase_normalization_factor = Vgrid_phase_normalization_factor; % misura della tensione dopo il trafo
ugrid_factor = 1;

Vemu_ref = 400; % tensione di fase rms di riferimento in uscita al trafo
Vemu_ref_norm = Vemu_ref * sqrt(2) / V_phase_normalization_factor;



