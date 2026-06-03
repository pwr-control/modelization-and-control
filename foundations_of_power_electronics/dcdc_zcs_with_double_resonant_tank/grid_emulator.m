

%% grid emulator output transformer

Ptrafo = 120e3;
I0 = 4; %no load current
Vcc_perc = 5.6; %cc voltage percente
Vline1 = 400; % primary voltage
Vline2 = 400; % secondary voltage
Vphase1=Vline1/sqrt(3);
Vphase2=Vline2/sqrt(3);
% f_grid = 47.5;
% f_grid = 48.2;
f_grid = 50;
% f_grid = 51.5;
w_grid = f_grid*2*pi;
omega_grid_emu_nom = w_grid;
omega_grid_nom = w_grid;
Inom_trafo=Ptrafo/Vphase2;
Ld2_trafo= Vphase2/(100/Vcc_perc)/Inom_trafo/(w_grid); %leakage inductace
Rd2_trafo = 0.02*Ptrafo/3/Inom_trafo^2; 
Lmu2_trafo= Vphase2/I0/(w_grid); %magentization inductance
Piron = 1.4e3;
Rm2_trafo = Vphase2^2/(Piron/3);
phi_trafo = Lmu2_trafo*I0*sqrt(2);

%% grid emulator others data
Vdc_ups=760; % DClink voltage reference
kp_vgrid = 10;
ki_vgrid = 45;
k_ff = 1;

%% voltage reference grid emulator
Igrid_phase_normalization_factor = 370*sqrt(2);
Vgrid_phase_normalization_factor = Vphase2*sqrt(2);
I_phase_normalization_factor = Igrid_phase_normalization_factor; % misura della corrente 
V_phase_normalization_factor = Vgrid_phase_normalization_factor; % misura della tensione dopo il trafo

Vups_ref = 230; % tensione di fase rms di riferimento in uscita al trafo
Vups_ref_norm = Vups_ref * sqrt(2) / V_phase_normalization_factor;

u_flt = 161;
g0 = u_flt * ts * 2*pi;
g1 = 1 - g0;

%% filters
fcut = 10;
fof = 1/(s/(2*pi*fcut)+1);
[nfof dfof] = tfdata(fof,'v');
[nfofd dfofd]=tfdata(c2d(fof,ts),'v');
fof_z = tf(nfofd,dfofd,ts,'Variable','z');
[A,B,C,D] = tf2ss(nfofd,dfofd);
LVRT_flt_ss = ss(A,B,C,D,ts);
[A,B,C,D] = tf2ss(nfof,dfof);
LVRT_flt_ss_c = ss(A,B,C,D);

%% rms calc
rms_perios = 10;
n10 = rms_perios/f_grid/ts;


