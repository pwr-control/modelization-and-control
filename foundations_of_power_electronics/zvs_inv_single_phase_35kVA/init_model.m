close all
clear all
clc
beep off

opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';

%% pwm
fPWM_INV = 10e3; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_ZVS = 20e3; % PWM frequency 
tPWM_ZVS = 1/fPWM_ZVS;
fclock=25e6;
half_phase_pulses = fclock/fPWM_ZVS/2;

%% sampling time
s=tf('s');
ts_inv = tPWM_INV; % Sampling time of the control 
z_inv=tf('z',ts_inv);
ts_zvs = tPWM_ZVS*2; % Sampling time of the control 
z_zvs=tf('z',ts_zvs);
% dead_time = 3e-6;
dead_time = 0;
m_dead_time = dead_time/tPWM_ZVS;

ts_afe = ts_inv;

ts_transmission = 2e-3;
angle_corr = 0/180*pi;

%% Main Parameter for Simulation
margin_factor = 1.15;
Vdc_min = 400*0.9*1.35;
Vdc_nom = 400*1.35;
Vdc_max = 400*1.1*1.35;

Vout_ac_rms_nom = 500;
Iout_ac_rms_nom = 55;
Vout_dc_nom = 820;
Iout_dc_nom = 35;
I_FS = Iout_ac_rms_nom * sqrt(2) * margin_factor;
V_FS = Vout_dc_nom * margin_factor;

ctrl_phase_out_lim_up = 1;
ctrl_phase_out_lim_down = 0;
ctrl_v_lim_up = Vout_dc_nom/V_FS;
ctrl_v_lim_down = 0;
ctrl_i_lim_up = Iout_dc_nom/I_FS;
ctrl_i_lim_down = 0;

%% Simulation data sampling time
simlength = 3.2;
tc = ts_inv/200;
ts_sim = tc;
time_fault=1.5e3;
t_misura=simlength;
Nc = ceil(t_misura/tc);
Ns_inv = ceil(t_misura/ts_inv);
Ns_afe = ceil(t_misura/ts_afe);
Ns_zvs = ceil(t_misura/ts_zvs);

%% HW components

% % filtro ingresso DC
% LFi_dc = 5e-3;
% RLFi_dc = 2e-4;
% CFi_dc = 5e-3;
% RCFi_dc = 0.5e-3;
LFi_dc = 10e-6;
RLFi_dc = 2e-4;
CFi_dc = 240e-6;
RCFi_dc = 0.5e-3;

% %% DClink filter 
% RLrect = 2e-3;
% Lrect = 5e-3;
% C1rect = 10e-3;
% C2rect = C1rect;
% C4rect = 4*20e-6;
% RC4rect = 1e-3;
% RC1rect = 6.8e3;
% RC2rect = RC1rect;

% LC interno ZVS
Cs = 180e-6;
Ls = 11.7e-6;
% Cs = 18e-6;
% Ls = 117e-6;
RLs = 2e-3;
req = 1/(2*pi*fPWM_ZVS*Cs) + 2*pi*fPWM_ZVS*Ls;
fres= 1/2/pi/sqrt(Cs*Ls);
fsr = fPWM_ZVS/fres;

% filtro uscita in DC
LFu_dc = 250e-6; %
RLFu_dc = 2e-4;
CFu_dc = 3.3e-3;
% CFu_dc = 2.3e-3;
RCFu_dc = 1.2e3;
RCFu_dc_internal = 1e-3;

% filtro uscita in AC
LFu_ac = 125e-6*2;
CFu_ac = 160e-6;
% CFu_ac = 400e-6;
RLFu_ac = 2e-3;
RCFu_ac = 500e-3;
f_res_fcuac = 1/2/pi/sqrt(CFu_ac*LFu_ac);
Lp = 250e-6/2;

% carico uscita
Rload = 8; % valore da usare per carico inverter
% Rload = 500/40; % valore da usare per carico inverter
% Rload = 1000; % valore da usare per carico DC
% Lload = 0;
% Lload = 250e-3;
Lload = 50e-6;
% Lload = 2500e-6;
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

%% control zvs
kp_i_zvs = 1;
ki_i_zvs = 18;
kp_v_zvs = 1;
ki_v_zvs = 18;

Vout_dc_ref = Vout_dc_nom;

%% grid pll
pll_i1 = 80;
pll_p = 1;
Vnom_ac_adc = 400/sqrt(3)*sqrt(2);


%% inverter PI control
% kp_v_inv = 0.1;
% ki_v_inv = 12.5;
% kp_i_inv = 0.5;
% ki_i_inv = 18;
% 
% kp_v_inv = 0.1;
% ki_v_inv = 12.5;
% kp_i_inv = 0.05;
% ki_i_inv = 1.8;

kp_v_inv = 0.1;
ki_v_inv = 12.5;
kp_i_inv = 0.1;
ki_i_inv = 18;

%% device parameters
Ron=1e-3;
Vfdiode = 0.7;
Csnubber=0.15e-6;
Rsnubber=2.2e4;
RCzvs = 1; % eq. HF resistance
Czvs = 10e-9;

%% HF trafo
pn_trafo = 50e3;
fn_trafo = 20e3;

v1_trafo = 400;
rd1_trafo = 1e-3;
ld1_trafo = 5e-7;

v2_trafo = v1_trafo*23/21;
rd2_trafo = rd1_trafo;
ld2_trafo = ld1_trafo;

w1_trafo=1;
w2_trafo=v2_trafo/v1_trafo;
n_v = 1;
n_i = -n_v;

I0 = 2;
rlm_trafo = 1e-3;
Piron = 1e2;
rfe_trafo = v1_trafo^2/Piron;
lm_trafo = v1_trafo/I0/(2*pi*fn_trafo);
im = Vdc_nom/sqrt(2)/(2*pi*lm_trafo*fn_trafo);


%% grid data
kgrid = 1;
Vline1 = 400; % primary voltage
Vphase1 = Vline1/sqrt(3);
Vphase2 = Vline1;
frequency_grid = 50;
omega_grid = frequency_grid*2*pi;
RLgrid = 2e-3;
Lgrid = 60e-6;
%% rms calc
rms_perios = 10;
n10 = rms_perios/frequency_grid/ts_afe;

%% grid_emulator setup
% vp_xi_pu = 0.85;
% vn_xi_pu = 0.1;
% vn_eta_pu = 0.05;

up_xi_pu = 1;
un_xi_pu = 0;
un_eta_pu = 0;
Ugrid_phase_normalization_factor = Vphase2*sqrt(2);
ugrid_factor = 1;

%% funzioni di trasferimento per modello lineare e analisi nel dominio delle frequenze.
% funzione di trasferimento tra uscita dei PI e carico
z_LFu_dc = RLFu_dc + s*LFu_dc;
z_CFu_dc = 1/(s*CFu_dc)+RCFu_dc_internal;
zp_CFu_dc = minreal((RCFu_dc * z_CFu_dc)/(RCFu_dc + z_CFu_dc));

ctrl_i_zvs = (kp_i_zvs + ki_i_zvs/s); 
ctrl_i_d_zvs = c2d(ctrl_i_zvs,ts_zvs);
ctrl_v_zvs = (kp_v_zvs + ki_v_zvs/s); 
ctrl_v_zvs_d = c2d(ctrl_v_zvs,ts_zvs);

Gv_zvs = ctrl_v_zvs * zp_CFu_dc;
% Gi_zvs = ctrl_i_zvs * 1/(z_LFu_dc + zp_CFu_dc);
Gi_zvs = ctrl_i_zvs * 1/(z_LFu_dc);

Gv_zvs_d = c2d(Gv_zvs,ts_zvs);
Gi_zvs_d = c2d(Gi_zvs,ts_zvs);

% fig1 = figure;
% % bode(Gv_zvs,Gv_zvs_d,opts);
% bode(Gv_zvs_d,Gi_zvs_d,opts);
% title('Voltage-Current loop bode diagram');
% grid on
% fig2 = figure;
% set(cstprefs.tbxprefs,'FrequencyUnits','Hz');
% margin(Gv_zvs_d);
% grid on
% fig3 = figure;
% set(cstprefs.tbxprefs,'FrequencyUnits','Hz');
% margin(Gi_zvs_d);
% grid on

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

% open_system('two_parallel_half_model_simscape_ver_ii');
open_system('two_parallel_full_model_simscape_ver_ii');
% open_system('two_parallel_full_model_simscape');
% open_system('two_parallel_half_model_simscape');





