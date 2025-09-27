close all
clear all
clc
beep off

opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';

%% pwm
fPWM_INV = 35e3; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM_INV; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
fclock=25e6;
half_phase_pulses = fclock/fPWM_DAB/2;

%% sampling time
s=tf('s');
ts_inv = tPWM_INV; % Sampling time of the control 
z_inv=tf('z',ts_inv);
ts_dab = tPWM_DAB*2; % Sampling time of the control 
z_dab=tf('z',ts_dab);
% dead_time = 3e-6;
dead_time = 0;
m_dead_time = dead_time/tPWM_DAB;

ts_afe = ts_inv;

ts_transmission = 2e-3;
angle_corr = 0/180*pi;

%% Main Parameter for Simulation
margin_factor = 1.25;
Vout_ac_rms_nom = 230;
Iout_ac_rms_nom = 30;
Vout_dc_nom = 400;
Iout_dc_nom = 30;
I_FS = Iout_ac_rms_nom * sqrt(2) * margin_factor;
V_FS = Vout_dc_nom * margin_factor;

ctrl_phase_out_lim_up = 1;
ctrl_phase_out_lim_down = 0;
ctrl_v_lim_up = Vout_dc_nom/V_FS;
ctrl_v_lim_down = 0;
ctrl_i_lim_up = Iout_dc_nom/I_FS;
ctrl_i_lim_down = 0;

%% Simulation data sampling time
simlength = 3;
tc = ts_inv/200;
ts_sim = tc;
time_fault=1.5e3;
t_misura=simlength/5;
Nc = ceil(t_misura/tc);
Ns_inv = ceil(t_misura/ts_inv);
Ns_afe = ceil(t_misura/ts_afe);
Ns_dab = ceil(t_misura/ts_dab);

%% HF trafo
pn_trafo = 8e3;
fn_trafo = 25e3;

v1_trafo = 100;
rd1_trafo = 1e-3;
ld1_trafo = 1e-8;
n1 = 5;

v2_trafo = 450;
n_trafo = v2_trafo/v1_trafo;
rd2_trafo = rd1_trafo*n_trafo^2;;
ld2_trafo = ld1_trafo*n_trafo^2;

w1_trafo=1;
w2_trafo=v2_trafo/v1_trafo;

n_v = 1;
n_i = -n_v;

I0 = 2;
rlm_trafo = 1e-3;
Piron = 25;
rfe_trafo = v1_trafo^2/Piron;
lm_trafo = v1_trafo/I0/(2*pi*fn_trafo);

%% HW components

% filtro uscita in DC
LFu_dc = 2*25e-6; %
RLFu_dc = 1e-3;
CFu_dc = 2500e-6;
RCFu_dc = 12e3;
RCFu_dc_internal = 1e-3;

% LC interno DAB
m = 7;
Cs1 = 250e-6;
Ls1 = 1.5e-6;
Cs2 = Cs1/m^2;
Ls2 = Ls1*m^2;

p_tranf = v1_trafo^2/(2*pi*fPWM_DAB*Ls1)*pi/4
RLs = 0.5e-3;
req = 1/(2*pi*fPWM_DAB*Cs1) + 2*pi*fPWM_DAB*Ls1;
fres= 1/2/pi/sqrt(Cs1*Ls1)
fsr = fPWM_DAB/fres



% filtro uscita in AC
LFu_ac = 125e-6;
CFu_ac = 50e-6;
RLFu_ac = 2e-3;
RCFu_ac = 2e-3;
f_res_fcuac = 1/2/pi/sqrt(CFu_ac*2*LFu_ac);

% filtro ingresso in DC 
CFi_dc = 2*125e-6;
RCFi_dc_internal = 1e-3;


% carico uscita
Rload = 8; % valore da usare per carico inverter
% Rload = 500/40; % valore da usare per carico inverter
% Rload = 1000; % valore da usare per carico DC
% Lload = 0;
% Lload = 250e-3;
Lload = 50e-6;
% Lload = 250e-6;
% Cload = 10;
% Cload = inf;
% Cload = 1/(2*pi*50)^2/Lload;
% Cload = 1/(2*pi*50)^2/Lload;
% Lload = 250e-3;
% Cload = inf;
Zload = sqrt(Rload^2+(2*pi*50*Lload)^2);
Vload_est = Zload*Iout_ac_rms_nom;


%% double integrator observer
A = [0 1; 0 0];
Aso = eye(2) + A*ts_inv;
Cso = [1 0];
p2place = exp([-100 -500]*2*pi*ts_inv);
K = (acker(Aso',Cso',p2place))';
kg = K(1);
kw = K(2);
kalman_theta = kg;
kalman_omega = kw;

%% control dab
kp_i_dab = 2;
ki_i_dab = 18;
kp_v_dab = 8;
ki_v_dab = 45;

Vout_dc_ref = Vout_dc_nom;

%% grid pll
pll_i1 = 40;
pll_p = 0.25;
Vnom_ac_adc = 400/sqrt(3)*sqrt(2);


%% inverter PI control and phase shift
kp_v_inv = 0.2;
ki_v_inv = 12;
kp_i_inv = 0.5;
ki_i_inv = 18;

freq = 50;
delta = 0.02;
res = 10^(22/20) * s/(s^2 + 4*delta*pi*freq*s + (2*pi*freq)^2);
resd_tf = c2d(res,ts_inv);
figure; bode(res,opts); grid on

[resn, resd]=tfdata(res,'v');
[Ares,Bres,Cres,Dres] = tf2ss(resn,resd);

% resd = c2d(res,ts_inv,'tustin');
[resnd, resdd]=tfdata(c2d(res,ts_inv),'v');
% [resnd, resdd]=tfdata(c2d(res,ts_inv,'tustin'),'v');
[Aresd,Bresd,Cresd,Dresd] = tf2ss(resnd,resdd);
% Aresd
% Bresd=Bresd
% Cresd = Cresd/ts_inv;
% Dresd = Dresd/ts_inv;

%% rms calc
frequency_grid = 50;
rms_perios = 10;
n10 = rms_perios/frequency_grid/ts_afe;

%% kalman observer
f = 50;
w = 2*pi*f;
Ahn = [cos(w*ts_inv) sin(w*ts_inv); -sin(w*ts_inv) cos(w*ts_inv)];
Chn = [1 0];
Bhn = [0 0]';
sys_hn = ss(Ahn,Bhn,Chn,0);
% kalman filter parameters
% q1kalman = 1;
% q2kalman = ts_inv;
% Rkalman = 1/ts_inv;
q1kalman = 1;
q2kalman = 1;
Rkalman = 1;
Qkalman = [q1kalman 0; 0 q2kalman];

%%
gate_nominal_voltage = 15;
Vdon_diode = 0.35;
Rdon_diode = 3e-3;

%%
phase_shift_mod11 = 0;
phase_shift_mod12 = 0;

fcut_4Hz_flt = 4;
g0_4Hz = fcut_4Hz_flt * ts_inv * 2*pi;
g1_4Hz = 1 - g0_4Hz;


flt_dq = 2/(s/(2*pi*50)+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
figure; bode(flt_dq_d); grid on
[num50 den50]=tfdata(flt_dq_d,'v');


%% Lithium Ion Battery
number_of_cells = 30; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 1000; %Hr*A

Vbattery_nom = 110;
Pbattery_nom = 10e3;
Ibattery_nom = Pbattery_nom/Vbattery_nom;
Rmax = Vbattery_nom^2/(Pbattery_nom*0.1);
Rmin = Vbattery_nom^2/(Pbattery_nom);

E_1 = -1.031;
% E0 = 3.685;
E0 = 3.485;
% E1 = 0.15;
E1 = 0.2156;
E2 = 0;
E3 = 0;
Elog = -0.05;
alpha = 35;

R0 = 0.0035;
R1 = 0.0035;
% R0 = 0.15;
% R1 = 0.15;
C1 = 0.5;
M = 125;


q1Kalman = ts_inv^2;
q2Kalman = ts_inv^1;
q3Kalman = 0;
rKalman = 1;

Zmodel = (0:1e-3:1);
ocv_model = E_1*exp(-Zmodel*alpha) + E0 + E1*Zmodel + E2*Zmodel.^2 +...
    E3*Zmodel.^3 + Elog*log(1-Zmodel+ts_inv);
figure; 
plot(Zmodel,ocv_model,'LineWidth',2);
xlabel('state of charge [p.u.]');
ylabel('open circuit voltage [V]');
title('open circuit voltage(state of charge)');
grid on


%% FF23MR12W1M1P_B11 data

Ron = 7.38e-3; % Rds [Ohm]
Vgamma = 4.35; % V

%% energy switching at Rg_ext = 5.10 Ohm
Eon = 2.7e-3; % J @ Tj = 125°C
Eoff = 1.3e-3; % J @ Tj = 125°C
Eerr = 0.5e-3; % J @ Tj = 125°C
Voff_sw_losses = 600; % V
Ion_sw_losses = 200; % A

Tambient = 40;
DThs_init = 0
JunctionTermalMass = 0.025; % J/K
Rtim = 0.5;
Rth_mosfet_JH = 0.46+Rtim; % K/W

%% ALPHA LT13070
weigth = 0.6/6; % kg
cp_al = 880; % J/K/kg
heat_capacity = cp_al*weigth % J/K
thermal_conducibility = 204; % W/(m K)
Rcond = 0.015/thermal_conducibility/(0.0065*0.03)
Rth_mosfet_HA = 0.1*6+Rcond % K/W

%% single phase pll
freq = 50;
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)'

freq_filter = 50;
tau_f = 1/2/pi/freq_filter;
Hs = 1/(s*tau_f+1);
Hd = c2d(Hs,ts_inv);