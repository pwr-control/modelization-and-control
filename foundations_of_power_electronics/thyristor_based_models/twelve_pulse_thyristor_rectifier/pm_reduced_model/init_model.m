close all
clear all
clc

opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';

%% sampling time
s=tf('s');
fPWM_RECT = 5e3; % PWM frequency 
tPWM_RECT = 1/fPWM_RECT;
fclock=25e6;
Tcontrol_rect=tPWM_RECT; % Sampling time of the control 
z_rect=tf('z',Tcontrol_rect);

%% Main Parameter for Simulation
margin_factor = 1.15;
Vdc_min = 400*0.9*1.35;
Vdc_nom = 400*1.35;
Vdc_max = 400*1.1*1.35;

Vout_dc_nom = 340;
Iout_dc_nom = 90;
I_FS = Iout_dc_nom * margin_factor;
V_FS = Vout_dc_nom * margin_factor;

%% Simulation data sampling time
simlength = 5;
Tc=1e-5;
Ts=Tc/1;
t_misura=simlength;
Nc=ceil(t_misura/Tc);

%% HW components
% Trafo
V1b = 400;
Pb = 50e3;
I1b = Pb/(V1b*sqrt(3));
V2b = 200;
I2b = Pb/(V2b*sqrt(3)*2);
Z1b = V1b/I1b;
Z2b = V2b/I2b;
r1pu = ((Pb/3/2*0.05)/I1b^2)/Z1b
l1pu = (0.025*V1b/sqrt(3)/I1b)/Z1b
r2pu = ((Pb/3/2*0.05)/I2b^2)/Z2b
l2pu = (0.025*V2b/sqrt(3)/I2b)/Z2b



% filtro uscita in DC
LFu_dc = 1.2e-3; %
RLFu_dc = 2e-4;
CFu_dc = 20e-3;
RCFu_dc = 0.25;
f0 = 1/(2*pi*sqrt(LFu_dc*CFu_dc))

Ldf2 = 250e-6; %
RLdf2 = 20e-3;
Cdf2 = 1/Ldf2/(2*pi*100)^2;
RCdf2 = 0.025;

Ldf3 = 250e-6; %
RLdf3 = 20e-3;
Cdf3 = 1/Ldf3/(2*pi*150)^2;
RCdf3 = 0.025;

Ldf5 = 250e-6; %
RLdf5 = 20e-3;
Cdf5 = 1/Ldf5/(2*pi*250)^2;
RCdf5 = 0.025;

Ldf7 = 250e-6; %
RLdf7 = 20e-3;
Cdf7 = 1/Ldf7/(2*pi*350)^2;
RCdf7 = 0.025;

% carico uscita
Rload = 1.2/4; % valore da usare per carico inverter
Lload = 250e-4;

%% grid pll
pll_i1 = 80;
pll_p = 1;
Vnom_ac_adc = 400/sqrt(3)*sqrt(2);

%% device parameters
Ron=1e-3;
Vfdiode = 0.7;
% 
Csnubber=0.15e-6;
Rsnubber=2.2e4;

%% grid data
kgrid = 1;
Vline1 = 400; % primary voltage
Vphase1=Vline1/sqrt(3);
f_grid=50;
w_grid = f_grid*2*pi;
RLgrid = 2e-3;
Lgrid = 60e-6;

%% LPF
wn = 2*pi*1;
lpf = (wn^2/(s^2+2*wn*s+wn^2));
[num, den] = tfdata(lpf,'v');
[A,B,C,D] = tf2ss(num, den);
lpf_ss = ss(A,B,C,D);
lpf_ss_init = [0,50/lpf_ss.c(2)];

% %% 
model = 'tp_thyr_rect_pm_rm';
open_system(model);
Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
Simulink.importExternalCTypes(model,'Names',{'twvpr_py_t'});
Simulink.importExternalCTypes(model,'Names',{'twvpr_pd_t'});
Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});


















