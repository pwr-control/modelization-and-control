
close all
clear all
clc

opts = bodeoptions('cstprefs');
opts.FreqUnits = 'Hz';
opts.Xlim = [14.8e3 150e3];
% opts.PhaseVisible = 'off';


scale = 1;
Iout_ref_SI = 100*scale;

%% control and simulation sampling time
fPWM_ZCS_MAX = 48e3; % maximum PWM frequency
s=tf('s');
ts = 1e-4; % Sampling time of the control 
z_inv=tf('z',ts);
% dead_time = 2e-6;
dead_time = 0;
m_dead_time = dead_time/ts;

simlength = 10; % simulation time length
tc=1e-7; % Sampling time of the simulation
t_misura_tc = 0.1; % stored data time length for tc sampling
t_misura_ts = simlength; % stored data time length for ts sampling
Nc=ceil(t_misura_tc/tc); % stored data maximum number of points for tc sampling
Ns=ceil(t_misura_ts/ts); % stored data maximum number of points for ts sampling

%% grid_emulator setup
grid_emulator; 
vp_xi_pu = 0.9;
vn_xi_pu = 0.05;
vn_eta_pu = 0.05;

%% Control scaling factor
margin_factor = 1.25;
Vdc_min = 400*0.9*1.35;
Vdc_nom = 400*1.35;
Vdc_max = 400*1.1*1.35;

Vout_dc_nom = 10;
Iout_dc_nom = 100;
I_FS = Iout_dc_nom * margin_factor;
V_FS = Vout_dc_nom * margin_factor;

%% Control gains and limits
ctrl_phase_out_lim_up = 1;
ctrl_phase_out_lim_down = 0;
ctrl_v_lim_up = Vout_dc_nom/V_FS;
ctrl_v_lim_down = 0;
ctrl_i_lim_up = Iout_dc_nom/I_FS;
ctrl_i_lim_down = 0;

% zcs current and voltage loop gains
kp_i_zcs = 0.25;
ki_i_zcs = 80;
kp_v_zcs = 0.25;
ki_v_zcs = 80;

% zcs output voltage reference
Vout_dc_ref = Vout_dc_nom;


%% output filter dimensioning
CFu = 8*33e-6;
% RCFu = 1e-3;
RCFu = 10e-3;

%% load parameters
% Rload = 10*scale/Iout_ref_SI;
Rload = 10/Iout_ref_SI;
% Rload = 5/Iout_ref_SI;
Lload = 25e-6;
% Lload = 250e-6;

%% harmonics compensator parameters
k_h2_comp = -12;
k_h4_comp = -12;
k_h6_comp = -12;
k_h8_comp = -12;
k_h10_comp = -12;
k_h12_comp = -12;
k_h14_comp = -12;

% k_h2_comp = -8;
% k_h4_comp = -8;
% k_h6_comp = -8;
% k_h8_comp = -8;
% k_h10_comp = -8;
% k_h12_comp = -8;
% k_h14_comp = -8;

% k_h2_comp = 0;
% k_h4_comp = 0;
% k_h6_comp = 0;
% k_h8_comp = 0;
% k_h10_comp = 0;
% k_h12_comp = 0;
% k_h14_comp = 0;

%% device parameters
C1 = 15e-6;
RC1 = 15e-4;
C2 = C1;
RC2 = RC1;

%% IGBT/MOSFET device parameters
Rs=1e-3;
Vgamma = 0.7;
% 
Csnubber=1e-12;
Rsnubber=1e6;
%
% Csnubber=0.15e-6;
% Rsnubber=2.2e4;

%% grid impedance
LsigmaTR1 = 60e-6;
Rd_TR1 = 6e-3;

%% high frequency transformer data
pn_trafo = 1.25e3;
fn_trafo = 50e3;
n1=16;
n2=1;

v1_trafo = 400;
Rd1_trafo = 1e-3;
L1sigma = 12e-6;
% Ld1_trafo = L1sigma/2;
Ld1_trafo = L1sigma;

v2_trafo = v1_trafo*n2/n1;
Rd2_trafo = Rd1_trafo*(n2/n1)^2;
% Ld2_trafo = Ld1_trafo*(n2/n1)^2;
Ld2_trafo = 0;

w1_trafo=1;
w2_trafo=v2_trafo/v1_trafo;
n_v = 1;
n_i = -n_v;

I0 = 0.0625;
Rlm_trafo = 1e-3;
Piron = 35;
R0_trafo = v1_trafo^2/Piron;
Lm_trafo = v1_trafo/I0/(2*pi*fn_trafo)



%% resonance circuit components
Cs = 4*22e-9;
RCs = 1e-4;
Ls = 9.4e-6;
Lp = 1.35e-3;
RLs = 1e-4;
RLp = 1e-4;
req = 1/(2*pi*fPWM_ZCS_MAX*Cs) + 2*pi*fPWM_ZCS_MAX*Ls
fres= 1/2/pi/sqrt(Cs*(Ls+L1sigma))
fsr = fPWM_ZCS_MAX/fres

Rload_p = Rload*n1^2*8/pi^2;
Z0 = s*Lm_trafo*Rload_p/(s*Lm_trafo+Rload_p)
Zls = s*(Ls+L1sigma);
Zp = s*Lp/(s*Cs)/(s*Lp+1/(s*Cs));
Zres = Zls + Zp;
Hres = (Z0/(Zres+Z0));

figure; bode(Hres,opts); grid on
Fh = gcf;                                                   % Handle To Current Figure
Kids = Fh.Children;                                         % Children
AxAll = findobj(Kids,'Type','Axes');                        % Handles To Axes
Ax1 = AxAll(1);                                             % First Set Of Axes
LinesAx1 = findobj(Ax1,'Type','Line')                    % Handle To Lines
LinesAx1.LineWidth = 2;                                  % Set ‘LineWidth’
LinesAx1.Color = 'k';                                    % Set 'LineWidth'
Ax2 = AxAll(2);                                             % Second Set Of Axes
LinesAx2 = findobj(Ax2,'Type','Line');                      % Handle To Lines
LinesAx2.LineWidth = 2;                                  % Set ‘LineWidth’
LinesAx2.Color = 'k';                                    % Set 'LineWidth'
% print -deps bode_res_block
%% rms filter
rms_perios = 10;
freq_grid = f_grid;
n = rms_perios/freq_grid/ts;

%% kalman observer
% data
f = 300;
w = 2*pi*f;
Ahn = [cos(w*ts) sin(w*ts); -sin(w*ts) cos(w*ts)];
Chn = [1 0];
Bhn = [0 0]';
sys_hn = ss(Ahn,Bhn,Chn,0);
% kalman filter parameters
q1kalman = ts/10;
q2kalman = ts/10;
Qkalman = [q1kalman 0; 0 q2kalman];
Rkalman = 1/ts;


%% double integrator observer grid
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso = [-0.4 -0.1]*omega_rso;
Lrso = acker(Arso',Crso',polesrso)';
Adrso = eye(2) + Arso*ts;
polesdrso = exp(ts*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)'
%% double integrator observer harmonics
Arsoh = [0 1; 0 0];
Crsoh = [1 0];
omega_rsoh = 2*pi*w;
polesrsoh = [-0.4 -0.1]*omega_rsoh;
Lrsoh = acker(Arsoh',Crsoh',polesrsoh)';
Adrsoh = eye(2) + Arsoh*ts;
polesdrsoh = exp(ts*polesrsoh);
Ldrsoh = acker(Adrsoh',Crsoh',polesdrsoh)'

%% PLL DDSRF
pll_i1 = 80;
pll_p = 1;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts);

%% C-Caller
open_system('cll_zcs_ver_v.slx');

