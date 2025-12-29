%[text] ## Settings for simulink model initialization and data analysis
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 1;
transmission_delay = 125e-6*2;
s=tf('s');

model = 'dcdc_cllc';
rpi_enable = 0;
rpi_ccaller = 0;
%[text] ### Settings voltage application
application400 = 0;
application690 = 0;
application480 = 1;

n_modules = 1;
%[text] ## Settings and initialization
fPWM = 40e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*5; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

fPWM_LLC = 150e3;
fPWM_LLC_pu = fPWM_LLC/fPWM_DAB %[output:076b441a]

TRGO_double_update = 0;
if TRGO_double_update
    ts_afe = 1/fPWM_AFE/2;
    ts_inv = 1/fPWM_INV/2;
    ts_dab = 1/fPWM_DAB/2;
else
    ts_afe = 1/fPWM_AFE;
    ts_inv = 1/fPWM_INV;
    ts_dab = 1/fPWM_DAB;
end
ts_battery = ts_dab;
tc = ts_dab/100;

z_dab=tf('z',ts_dab);
z_afe=tf('z',ts_afe);

% t_misura = simlength - 0.2;
t_misura = 0.2;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

Pnom = 275e3;
ubattery = 750;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 750;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:02159090]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:5d197f8f]
%[text] ### AFE simulation sampling time
dead_time_DAB = 0.2e-6;
dead_time_AFE = 0;
dead_time_INV = 0;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;
%[text] ## Grid Emulator Settings
grid_emulator;
%[text] ### Nominal DClink voltage seting
if (application690 == 1)
    Vdc_bez = 1070; % DClink voltage reference
elseif (application480 == 1)
    Vdc_bez = 750; % DClink voltage reference
else
    Vdc_bez = 660; % DClink voltage reference
end
%[text] ### ADC quantizations
adc12_quantization = 1/2^11;
adc16_quantization = 1/2^15;
%[text] ### DAB dimensioning
LFi_dc = 400e-6;
RLFi_dc = 50e-3;
%[text] #### DClink, and dclink-brake parameters
Vdc_ref = Vdc_bez; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = 250e3;
Rbrake = 4;
CFi_dc1 = 900e-6*4;
RCFi_dc1_internal = 1e-3;
CFi_dc2 = 900e-6*4;
RCFi_dc2_internal = 1e-3;
%[text] #### Tank LC and HF-Transformer parameters
% LLC
fres = fPWM_DAB;
% Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/8)
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/4) %[output:53fa3074]
Cs = 1/Ls/(2*pi*fres)^2 %[output:215718df]

m1 = 12;
m2 = 12;
m12 = m1/m2;

Ls1 = Ls/2;
Cs1 = Cs*2;
Cs2 = Cs1/m12^2;
Ls2 = Ls1*m12^2;

lm_trafo = 1e-3;
rfe_trafo = 1e3;
rd1_trafo = 5e-3;
ld1_trafo = Ls1;
rd2_trafo = rd1_trafo/m12^2;
ld2_trafo = ld1_trafo/m12^2;
%[text] #### DClink Lstray model
Lstray_module = 100e-9;
RLstray_dclink = 10e-3;
C_HF_Lstray_dclink = 15e-6;
R_HF_Lstray_dclink = 22000;
Z_HF_Lstray_dclink = 1/s/C_HF_Lstray_dclink + R_HF_Lstray_dclink;
Z_LF_Lstray_dclink = s*Lstray_module + RLstray_dclink;
Z_Lstray_dclink = Z_HF_Lstray_dclink*Z_LF_Lstray_dclink/(Z_HF_Lstray_dclink+Z_LF_Lstray_dclink);
ZCFi = 7/s/CFi_dc1;
sys_dclink = minreal(ZCFi/(ZCFi+Z_Lstray_dclink));
% figure; bode(sys_dclink,Z_Lstray_dclink,options); grid on
%[text] ### LCL switching filter
if (application690 == 1)
    LFu1_AFE = 0.5e-3;
    RLFu1_AFE = 157*0.05*LFu1_AFE;
    LFu1_AFE_0 = LFu1_AFE;
    RLFu1_AFE_0 = RLFu1_AFE/3;
    CFu_AFE = (100e-6*2);
    RCFu_AFE = (50e-3);
else
    LFu1_AFE = 0.33e-3;
    RLFu1_AFE = 157*0.05*LFu1_AFE;
    LFu1_AFE_0 = LFu1_AFE;
    RLFu1_AFE_0 = RLFu1_AFE/3;
    CFu_AFE = (185e-6*2);
    RCFu_AFE = (50e-3);
end
%%
%[text] ### Single phase inverter control
flt_dq = 2/(s/(2*pi*50)+1)^2;
flt_dq_d = c2d(flt_dq,ts_inv);
% figure; bode(flt_dq_d); grid on
% [num50 den50]=tfdata(flt_dq_d,'v');
iph_grid_pu_ref = 3.75;
%[text] ### DAB Control parameters
kp_i_dab = 0.25;
ki_i_dab = 18;
kp_v_dab = 0.25;
ki_v_dab = 45;

%%
%[text] ### AFE current control parameters
%[text] #### Resonant PI
Vac_FS = V_phase_normalization_factor %[output:4f58960b]
Iac_FS = I_phase_normalization_factor %[output:9d2fb161]

kp_rpi = 0.25;
ki_rpi = 18;
kp_afe = 0.25;
ki_afe = 18;
delta = 0.025;
res_nom = s/(s^2 + 2*delta*omega_grid_nom*s + (omega_grid_nom)^2);
res_min = s/(s^2 + 2*delta*omega_grid_min*s + (omega_grid_min)^2);
res_max = s/(s^2 + 2*delta*omega_grid_max*s + (omega_grid_max)^2);

Ares_nom = [0 1; -omega_grid_nom^2 -2*delta*omega_grid_nom];
Ares_min = [0 1; -omega_grid_min^2 -2*delta*omega_grid_min];
Ares_max = [0 1; -omega_grid_max^2 -2*delta*omega_grid_max];
Bres = [0; 1];
Cres = [0 1];
Aresd_nom = eye(2) + Ares_nom*ts_afe;
Aresd_min = eye(2) + Ares_min*ts_afe;
Aresd_max = eye(2) + Ares_max*ts_afe;
Bresd = Bres*ts_afe;
Cresd = Cres;
%%
%[text] ### Grid Normalization Factors
Vgrid_phase_normalization_factor = Vphase2*sqrt(2);
pll_i1 = 80;
pll_p = 1;
pll_p_frt = 0.2;
Vmax_ff = 1.1;
Igrid_phase_normalization_factor = 250e3/Vphase2/3/0.9*sqrt(2);
ixi_pos_ref_lim = 1.6;
ieta_pos_ref_lim = 1.0;
ieta_neg_ref_lim = 0.5;
%%
%[text] ### Double Integrator Observer for PLL
Arso = [0 1; 0 0];
Crso = [1 0];
omega_rso = 2*pi*50;
polesrso_pll = [-1 -4]*omega_rso;
Lrso_pll = acker(Arso',Crso',polesrso_pll)';
Adrso_pll = eye(2) + Arso*ts_afe;
polesdrso_pll = exp(ts_afe*polesrso_pll);
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:0bd29de1]

%[text] ### PLL DDSRF
use_advanced_pll = 0;
use_dq_pll_ccaller = 0;
pll_i1_ddsrt = pll_i1/2;
pll_p_ddsrt = pll_p/2;
omega_f = 2*pi*50;
ddsrf_f = omega_f/(s+omega_f);
ddsrf_fd = c2d(ddsrf_f,ts_afe);
%%
%[text] ### First Harmonic Tracker for Ugrid cleaning
omega0 = 2*pi*50;
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:915851da]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:46c059b7]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:7459e256]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:4579d87c]
%[text] ### 
%%
%[text] ### Reactive current control gains
kp_rc_grid = 0.35;
ki_rc_grid = 35;

kp_rc_pos_grid = 0.35;
ki_rc_pos_grid = 35;
kp_rc_neg_grid = 0.35;
ki_rc_neg_grid = 35;
%%
%[text] ### Settings for First Order Low Pass Filters
%[text] #### LPF 50Hz in state space (for initialization)
fcut = 50;
fof = 1/(s/(2*pi*fcut)+1);
[nfof, dfof] = tfdata(fof,'v');
[nfofd, dfofd]=tfdata(c2d(fof,ts_afe),'v');
fof_z = tf(nfofd,dfofd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(nfofd,dfofd);
LVRT_flt_ss = ss(A,B,C,D,ts_afe);
[A,B,C,D] = tf2ss(nfof,dfof);
LVRT_flt_ss_c = ss(A,B,C,D);
%[text] #### LPF 161Hz
fcut_161Hz_flt = 161;
g0_161Hz = fcut_161Hz_flt * ts_afe * 2*pi;
g1_161Hz = 1 - g0_161Hz;
%%
%[text] #### LPF 500Hz
fcut_500Hz_flt = 500;
g0_500Hz = fcut_500Hz_flt * ts_afe * 2*pi;
g1_500Hz = 1 - g0_500Hz;
%%
%[text] #### LPF 75Hz
fcut_75Hz_flt = 75;
g0_75Hz = fcut_75Hz_flt * ts_afe * 2*pi;
g1_75Hz = 1 - g0_75Hz;
%%
%[text] #### LPF 50Hz
fcut_50Hz_flt = 50;
g0_50Hz = fcut_50Hz_flt * ts_afe * 2*pi;
g1_50Hz = 1 - g0_50Hz;
%%
%[text] #### LPF 10Hz
fcut_10Hz_flt = 10;
g0_10Hz = fcut_10Hz_flt * ts_afe * 2*pi;
g1_10Hz = 1 - g0_10Hz;
%%
%[text] #### LPF 4Hz
fcut_4Hz_flt = 4;
g0_4Hz = fcut_4Hz_flt * ts_afe * 2*pi;
g1_4Hz = 1 - g0_4Hz;
%%
%[text] #### LPF 1Hz
fcut_1Hz_flt = 1;
g0_1Hz = fcut_1Hz_flt * ts_afe * 2*pi;
g1_1Hz = 1 - g0_1Hz;
%%
%[text] #### LPF 0.2Hz
fcut_0Hz2_flt = 0.2;
g0_0Hz2 = fcut_0Hz2_flt * ts_afe * 2*pi;
g1_0Hz2 = 1 - g0_0Hz2;
%%
%[text] ### Settings for RMS calculus
rms_perios = 1;
n1 = rms_perios/f_grid/ts_afe;
rms_perios = 10;
n10 = rms_perios/f_grid/ts_afe;
%%
%[text] ### Online time domain sequence calculator
w_grid = 2*pi*f_grid;
apf = (s/w_grid-1)/(s/w_grid+1);
[napfd, dapfd]=tfdata(c2d(apf,ts_afe),'v');
apf_z = tf(napfd,dapfd,ts_afe,'Variable','z');
[A,B,C,D] = tf2ss(napfd,dapfd);
ap_flt_ss = ss(A,B,C,D,ts_afe);
% figure;
% bode(ap_flt_ss,options);
% grid on
%%
%[text] ### Single phase pll
freq = 50;
kp_pll = 314;
ki_pll = 3140;

Arso = [0 1; 0 0];
Crso = [1 0];

polesrso = [-5 -1]*2*pi*10;
Lrso = acker(Arso',Crso',polesrso)';

Adrso = eye(2) + Arso*ts_inv;
polesdrso = exp(ts_inv*polesrso);
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:74e07718]

freq_filter = 50;
tau_f = 1/2/pi/freq_filter;
Hs = 1/(s*tau_f+1);
Hd = c2d(Hs,ts_inv);
%[text] ### Lithium Ion Battery
typical_cell_voltage = 3.6;
number_of_cells = floor(ubattery/typical_cell_voltage)-1; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 50; %Hr*A

Vbattery_nom = ubattery;
Pbattery_nom = Pnom;
Ibattery_nom = Pbattery_nom/Vbattery_nom;
Rmax = Vbattery_nom^2/(Pbattery_nom*0.1);
Rmin = Vbattery_nom^2/(Pbattery_nom);

E_1 = -1.031;
E0 = 3.485;
E1 = 0.2156;
E2 = 0;
E3 = 0;
Elog = -0.05;
alpha = 35;

R0 = 0.035;
R1 = 0.035;
C1 = 0.5;
M = 125;

q1Kalman = ts_inv^2;
q2Kalman = ts_inv^1;
q3Kalman = 0;
rKalman = 1;

Zmodel = (0:1e-3:1);
ocv_model = E_1*exp(-Zmodel*alpha) + E0 + E1*Zmodel + E2*Zmodel.^2 +...
    E3*Zmodel.^3 + Elog*log(1-Zmodel+ts_inv);
figure;  %[output:0825f5ce]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:0825f5ce]
xlabel('state of charge [p.u.]'); %[output:0825f5ce]
ylabel('open circuit voltage [V]'); %[output:0825f5ce]
title('open circuit voltage(state of charge)'); %[output:0825f5ce]
grid on %[output:0825f5ce]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:4e40d541] %[output:563fc97c] %[output:6761ce4b]
%[text] #### DEVICES settings
% danfoss_SKM1700MB20R4S2I4; % SiC-Mosfet full leg
wolfspeed_CAB760M12HM3;

dab_mosfet.Vth = Vth;                                  % [V]
dab_mosfet.Rds_on = Rds_on;                            % [Ohm]
dab_mosfet.Vdon_diode = Vdon_diode;                    % [V]
dab_mosfet.Vgamma = Vgamma;                            % [V]
dab_mosfet.Rdon_diode = Rdon_diode;                    % [Ohm]
dab_mosfet.Eon = Eon;                                  % [J] @ Tj = 125°C
dab_mosfet.Eoff = Eoff;                                % [J] @ Tj = 125°C
dab_mosfet.Eerr = Eerr;                                % [J] @ Tj = 125°C
dab_mosfet.Voff_sw_losses = Voff_sw_losses;            % [V]
dab_mosfet.Ion_sw_losses = Ion_sw_losses;              % [A]
dab_mosfet.JunctionTermalMass = JunctionTermalMass;    % [J/K]
dab_mosfet.Rtim = Rtim;                                % [K/W]
dab_mosfet.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
dab_mosfet.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
dab_mosfet.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
dab_mosfet.Lstray_module = Lstray_module;              % [H]
dab_mosfet.Irr = Irr;                                  % [A]
dab_mosfet.Csnubber = Csnubber;                        % [F]
dab_mosfet.Rsnubber = Rsnubber;                        % [Ohm]
dab_mosfet.Csnubber_zvs = 4.5e-9;                      % [F]
dab_mosfet.Rsnubber_zvs = 5e-3;                        % [Ohm]

danfoss_SKM1400MLI12BM7; % 3L-NPC Si-IGBT
igbt.inv.Vth = Vth;                                  % [V]
igbt.inv.Vce_sat = Vce_sat;                          % [V]
igbt.inv.Rce_on = Rce_on;                            % [Ohm]
igbt.inv.Vdon_diode = Vdon_diode;                    % [V]
igbt.inv.Rdon_diode = Rdon_diode;                    % [Ohm]
igbt.inv.Eon = Eon;                                  % [J] @ Tj = 125°C
igbt.inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
igbt.inv.Erec = Erec;                                % [J] @ Tj = 125°C
igbt.inv.Voff_sw_losses = Voff_sw_losses;            % [V]
igbt.inv.Ion_sw_losses = Ion_sw_losses;              % [A]
igbt.inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
igbt.inv.Rtim = Rtim;                                % [K/W]
igbt.inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
igbt.inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
igbt.inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
igbt.inv.Lstray_module = Lstray_module;              % [H]
igbt.inv.Irr = Irr;                                  % [A]
igbt.inv.Csnubber = Csnubber;                        % [F]
igbt.inv.Rsnubber = Rsnubber;                        % [Ohm]
igbt.inv.Csnubber_zvs = 4.5e-9;                      % [F]
igbt.inv.Rsnubber_zvs = 5e-3;                        % [Ohm]

wolfspeed_CAB760M12HM3; % SiC Mosfet fpr 3L - NPC (two in parallel modules)
parallel_factor = 1.75;
inv_mosfet.Vth = Vth;                                           % [V]
inv_mosfet.Rds_on = Rds_on/parallel_factor;                     % [Ohm]
inv_mosfet.Vdon_diode = Vdon_diode/parallel_factor;             % [V]
inv_mosfet.Vgamma = Vgamma/parallel_factor;                     % [V]
inv_mosfet.Rdon_diode = Rdon_diode/parallel_factor;             % [Ohm]
inv_mosfet.Eon = Eon/parallel_factor;                           % [J] @ Tj = 125°C
inv_mosfet.Eoff = Eoff/parallel_factor;                         % [J] @ Tj = 125°C
inv_mosfet.Eerr = Eerr/parallel_factor;                         % [J] @ Tj = 125°C
inv_mosfet.Voff_sw_losses = Voff_sw_losses;                     % [V]
inv_mosfet.Ion_sw_losses = Ion_sw_losses;                       % [A]
inv_mosfet.JunctionTermalMass = JunctionTermalMass;             % [J/K]
inv_mosfet.Rtim = Rtim;                                         % [K/W]
inv_mosfet.Rth_mosfet_JC = Rth_mosfet_JC/parallel_factor;       % [K/W]
inv_mosfet.Rth_mosfet_CH = Rth_mosfet_CH/parallel_factor;       % [K/W]
inv_mosfet.Rth_mosfet_JH = Rth_mosfet_JH/parallel_factor;       % [K/W]
inv_mosfet.Lstray_module = Lstray_module/parallel_factor;       % [H]
inv_mosfet.Irr = Irr/parallel_factor;                           % [A]
inv_mosfet.Csnubber = Csnubber;                                 % [F]
inv_mosfet.Rsnubber = Rsnubber;                                 % [Ohm]
inv_mosfet.Csnubber_zvs = 4.5e-9;                               % [F]
inv_mosfet.Rsnubber_zvs = 5e-3;                                 % [Ohm]
%[text] ## C-Caller Settings
open_system(model);
Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
Simulink.importExternalCTypes(model,'Names',{'rpi_output_t'});

%[text] ## Remove Scopes Opening Automatically
open_scopes = find_system(model, 'BlockType', 'Scope');
for i = 1:length(open_scopes)
    set_param(open_scopes{i}, 'Open', 'off');
end

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);
% 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":35.8}
%---
%[output:076b441a]
%   data: {"dataType":"textualVariable","outputData":{"name":"fPWM_LLC_pu","value":"   0.750000000000000"}}
%---
%[output:02159090]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.583333333333334e+02"}}
%---
%[output:5d197f8f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:53fa3074]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     1.278409090909091e-06"}}
%---
%[output:215718df]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     4.953480089180958e-07"}}
%---
%[output:4f58960b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:9d2fb161]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:0bd29de1]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.038750793402628"],["9.678128161416666"]]}}
%---
%[output:915851da]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:46c059b7]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:7459e256]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000025000000000"],["-2.467401100272340","0.999607300918301"]]}}
%---
%[output:4579d87c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.038877209088174"],["6.791521528499614"]]}}
%---
%[output:74e07718]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.009392782979394"],["0.491161532854001"]]}}
%---
%[output:0825f5ce]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARMAAACmCAYAAAD52WvKAAAAAXNSR0IArs4c6QAAH6RJREFUeF7tXQ2QV9V1P0aTuiJRFrBxXciCLmriVMTEUAazWMfPFrR0pny0hgIluHXdNIFZ+XKQmQALZc0UNC11CEPa7i5hSiOYpAyhgaqIGlFqIopmWSiCFRYRRWlLsp3z8Pxz9\/7ve+\/e9+57993lvBlnZfd+nPu75\/zuued+ndfd3d0N\/DECjAAjkBKB85hMUiLI2RkBRiBAgMmEFYERYASsIMBkYgVGLoQRYASYTAx1YNeuXTB58mRobW2FkSNHGuaOTi6X\/fHHH8PKlSthxowZUFlZaaUuLHPOnDlBWc3NzVBRUQFY74EDB2DChAlW6ggrZP369bBz585SvVGVPfbYY3DHHXdAbW1trEwmaWML+yQB4bR582aoqqqCtWvXasmiwle3Tpvpjh8\/DtOnT4eHHnrIup6GyclkYrMHLZe1bNmywNDXrFmTGZkcOnQIpk6dCg8++GCmZELKPXHixNh6kHRWrVqlZcAmaU2658033wxwGTt2bGCQul9RyATlRf05fPiwFnnrti8qXSHJhDoSgcCPvADqqJMnTwa\/37FjR9moQaM7\/v36668vGSL9\/uGHHw5+h2UvXbo0VLHDZBC9BywfR3mSB\/PgCFZdXR38Hkc1\/GbOnBkoJJVJhit7IuK\/0VOYO3dukF8eGVUKKxNPHIZYblNTEzQ0NMCePXt6yIkGGlY31rN69epApjFjxsD27dtLRi+O5ligiK9s9EQuVDelFfuP+n7YsGHBKCvLqUqL3qIoP5IBeWCyIYTJIP9eVYbcVupjlY6KekhGjhiG6SiWhX+nMqMwl2UVPeYsvWgVqRSOTGSDE5WQjPSll14KFLh\/\/\/6Bkg0ePDhQGHGUHTduXA93HhURpydiB4WN+vIoKhrqvn37StMcIhOSh1xycUSgerGTUV7RC4giEzSKKM8EcWlvbw+IET\/EAfOoSEuFIeaRMcNpDuK\/ZMkSaGlpKZVL+FJb0PAJX7HtYThRW8RRUky7devWHp6ITDyYtqamJiB+IgoyGjmtiCmREOEiGgD1Mf1N7os4zySsj2WdwDrlPm9ra+uBPXk\/JAPpKOal36kwJ3ugvty0aVMPHE28wV7pmYSNYNjpjY2NZfN9Mf3u3bvLlJIMjkiARsAo9xg7cPbs2Uo3W+WZUGdi\/CGqA008kzgyobJWrFgR6IEYxzHBMGyaoxrdMX6D3hbFD8R6iNjJOEUcZGJHnGi0lUdtbIuqb8JGYBXxiINEmKuvik+JsSTCRTXNiepj8kwOHjyoJHoyWmq\/6LmqPAlMF4a5TFSiTmA\/yIRpgzCiyiicZyKPxiIgcWSycePGwD0UP5oidHV1RRqcmCeOaMhwaRQSyUQmDLFcm2RCSovtoxGMYismGMpkQgqNRrRw4UJYtGhRUD56MUgmoqGKOJFi09SU2o2jLAaQRQ8SyUSehomkovKk0KDQG4kiTnl6STLoEJY8dYwik6g+lsvBf4teI5G0iEuYd4Tyy30pYkM6LRs4DZhkO+RZIu5ZfoUjE5NRFcGJ8kxE4GTW1yUMecVG1zNRudY2yUQcwQcOHFia4qhG9ihClslEVF7EVxytTTwTEfuooKQYeyD3XkVSYXGmOM8kzHhseCaqPo4iE3kwlIkmrWcit\/Wc90x0YiY0StGc2CRmEjbXFjtC7gTVaIDlqDwTeTTB0YPmzLfddluPUYpcXZJJVqa41RxxdBcDbzoYkrchk4mqrRSAFGMm1JajR4+Wpj1xMRPyamSSipJBnD6RMVL\/U7BVXPnJM2ZC7RH7WJ7SyYQhx4ow0E0kqiITMWYiY84xEw0\/SzQScSVDZP2+ffsGbq\/swsat5uiQCYpospojTnPw\/8Mi\/eQ10EoJRe3DyERsi2pfizw\/F\/ei6GCIUxf8aOUJSUNc4UHs0evBT5xCZbGaI66YiLKjy44fYYb9jQRGnoqcVgzSYj6T1RwVIYctDcet5pBOyGQi9wviKwe45b4+Z1dzECxUyOXLl5dt8pE7QDWXjeKaIq3ha3Bir0ti6jmpvL08N1H1hg5Igzl6mbqbBG1gZTVmQsYuL5WSoDh9mDVrFsybN09rN6HcQCYTG12evAx5MMCSTHYC563cyVtanJxJMfd+BywqC7qgSCYqwhD3MNjaHl6cbmdJGIFzG4GSZyKv40fBopqeIFGsW7cO6uvrYcGCBUoyEfcuYPlRO1DP7W7h1jMC\/iHQg0xwTwHuLYjyGpB05HToii1evBimTJkS7PLUmcq4cMP86x6WmBHwBwErMRN55QObH3fSkuaCo0aN6nE+ZujQof6gx5IyAgVHYNu2bTBkyJBcpCyb5mCtaU6pRgVZcZqDH50BUa36IJl0dHTk0vg0lfgiJ7bRF1l9kZMxVVtOD89EjhzHeReqImUyEQkk6lQpleWLQvkiJyt+miEjPK8v\/Z+nnJHTHBcB0zwbn0bN9u\/fn5v7mEZOzOuLrL7I6ROmedqTdsxEFXhNq+Sq\/Hk2Po38rPhp0FPnZUztY5qnPYWSibxUnGTKkwSaPBufRD7Kw4qfBj0mE\/voqUvM055C95nkRR4yBHk2Pk2HMpmkQY\/JxD56BSITDI7if653pjKZ2FczX4jPFzk5ZhKzmqMbE9FNl9QkmEySIheezxcj9UVOJhMNMhEv7Y1SadPTvibmwWRigpZeWl+M1Bc5mUxiyERPLbNPxWRiH2NfjNQXOZlMmEysWikrvlU4g8IYU\/uY5jk4a+8zsd9M99HnNG1ixU+DHq\/m2EfPvT0xmSTsVSaThMBFZGNM7WPKnokHB\/1Y8e0rPmNqH1MmEyYTq1rli5H6IqdP8Z2qu78Jh3\/8Hav6FFaYcpojnu6lG8vDbk+zLWWeTJpGdlb8NOhxzMQ+euUltr34DjzQtheOP3pLHtVBGZmIL8WNHz8+uIpx\/vz5gO+Y5nHTNZOJ\/X73hfh8kdMXz8Q5mYg7XPFxISITJBmdax3TmgKTSVoEy\/P7YqS+yMlkotZR5TSHHnueNm0abNiwIbgkuqGhoccTlPZV\/myJTCb2kfXFSH2Rk8nEgEwwqfiaHP47r5vkmUyYTOwjYL9EH4jP+TTHPuxmJTKZmOGlk9oHxfdltCe8fcB02ZZOWLZlv7sArI5yZpmGycQ+uj4oPpOJ\/X53TiZxj3FlfWkSk4l9pWIyOTcxdU4mCDteJN3Z2Qn4yDR99Lu6ujpoa2uD5uZmqKioSNxLGOTFT6yDA7CJ4YzMyGRiH1cfMHVOJmGXH9HvGxsbYeXKlbEv\/0V1HwV3Z86cyWRiX8\/LSvRB8XmaY18RnJMJbVrDx8fXrl0LtbW1QC\/23XjjjXDPPffAk08+mdgzIVLCck+dOsVkYl+HmEwY0wCBf3r+CDSuf919AFZeGm5tbYVhw4ZpvSMc1Zc4vcGp0oEDB8qmUjzNycYK2DOxj6sPmI57\/GV45lcn3JOJffjP7l3ZsWNH4I2o4jJEJvgT30gt8nfo0CGorq4usogl2XyR1Rc5Ediiy3rrrbfCh6Ob4MyAq3snmaBXsnr16h4GKMdNeDXHPj\/5MIpyzMR+v1d+62dBoc4O+mHl8rOg1EybF0lHeSY+PFzui4H6ZKSMqT1COXj8NAz\/9nNuyUR8eHzjxo1BfGPkyJGAXkVNTQ1MmDDBSouZTKzAqFWIL0bqi5w+EDTFS5x6JuLS8NatW0tB0qzfyyGr4GmOFj8YJfLFSH2Rs+hk8sxbJ2Dcd18OdOSCY2\/Au9+\/30hfkiYOvc9k1KhRMGLECGhqaoLly5fD7t27ob29HdasWZPpq39MJkm7MjyfL0bqi5xFJhNxejO48kI40f4N6PzFC\/aVSlGi8goC2TuZO3dukBWXh3HKk+XHZGIfXV+M1Bc5i0omy7d0QvOW\/SUFemXB78OYL30B8opB8u30CW2XFT8hcBHZGNNkmO7qeB\/ufmx3KTN6JJv+6gbAn3kOzmVkEredfuHChTzN4Qejkml9TC4mE31YcTrT0LY32JQmfkgg6JG4iEGWyCTutDAKh5dLpz3gFwdXnkwaJ0vU31nx06CnzsuYRmMaRiCYS\/RGxFLytCdtz8S+6qhLzLPxadrEip8GPSYTHfSQPFZs7QzO2IR9YSTi1DPRaVweaZhM7KPsC\/H5Iif2kG1ZkTjWPXcYXux8v2zqIk9jBve7EB6bdG3gjcR9edqT0TTH5g7YMBDybHxcR\/A0Jw1C5nltG6i5BPo5ksqKpHFJxQVw3\/dehYPvnQb8d9yHpLHg7qFwU80lWgRSmGlOXMOy\/juTiX2Ekyq+fUmiS\/RFzjjPBAkCCeDRnx6A7W8cj\/Q0ZEQwn4nnEddHedoTLw3H9UbI33uL4idsfibZfML02T1vwo1fuBKaNu6DzmMfGxEGgkdTlLu+OADq6wYZexy6HeCcTMTnQUnoPFZysK48G6\/bIap0Pim+L7IWRU6aerz7wf8Gwc+Oox9pT0nCPI2RQy+FP\/\/K5ZmRRhHCBpHPg4r3s9LDXLw0fLbbiqL4OoToi6x5yElEcdFnzofFP+6AX6UgCsKepibXfK4PNNwyOPi1TnBUp+\/SpslzcNZeGuaDfj27NQ\/FT6tIlN8XWdPIKQYy\/+2Xx+Cp\/zwaNF83yBmFNZHFVZddBPfXDYILL\/gU\/Pr9IzBkyBBbXZRZOU7JBFuFXsjmzZvL7oDFqY58m7xtFPJsfBrZ0yh+mnqT5PVFVpWcRBL\/9+tuWP\/zd2BXx9kdnzZIQvQgMOg5fXQ13DCor5Zn4QumedpTaABWviCJnwdlzyQJkanyiF7ET\/d2wUsHT8J\/HT8NHUc\/hMMnz1iphqYZSBKDKi+EGaOrobLPp7WIQkcAJpNylHg1R0dzFGl8USYUPQ9ZiSB+090N\/\/zCEXh+\/\/sA3fY8CDE+EXgUn5BE\/VcHwWcrLrBGErrqkAemurJEpSuEZ2KjIUnKyLPxSeSjPL4oU1IyEb2HnR0nAC\/cOdj1sbXphYw9ehJnzpyBoQMvhusH9YWFf3QlHD7xP7mThK5O+NL\/edqTMgA7ffp0GDx4cOaH+lQdl2fjdRVHlc4XZSIyOf+Sy4NmdAPA8x0n4D9yIAfyIPDnzbX94I+HXwYYxKRNXTKuvmHKAdiePaic5qj2mahe30tjjGF5mUziUSWvAUfzff\/9UTCtePngySCjrcCkSgoxDlEzoAL+5IbfhSEDzj4Ra2MplMkkvu9NU+RpT9oxE3zzBld5+NrGs91pU\/FppD7QdRp+8stj8NqRD4NdlXmSAwYp\/+DqSvjyJ+c\/wrwHU2U2SW8TU5N6k6T1RVbnZKLyTKqqqkpLxSrwxRcAw3bLyuWqDg7m2fgkSkR5wpSJjBB\/PvPWe\/BC58lgB2VexEBTi2su7wN\/eN1AqBvWD3Dr96BB2W3ZToOjmNcXA7U9mNjCT1VOnvYUGjNBwXS9EPF5DHzlbs6cOYAXUsvPYojp8K1h143X6USaUmCs4d9f74J\/ffndIJvNZcy4KcXn+1fAF6suhr8cfUWioKQvRuqLnEwmasvRnuboGB6mIe9j0qRJZZdP4wPoS5YsgZaWltCrH\/NkUmoTEsaaZ98O4g5ZxBzEWAOcBzC8ui\/Mvr0GTnx0dk+FjXhDVP\/4YqS+yMlkkgOZ0FQnbJqjsxEuSzJB0nj7xGlY+pP9xqc8RfjEZcyhAy+Cr998BVz8O\/nvddAleF+M1Bc5mUxyIBOqAklj586dkUvLdOcsbs8Xn89AMsHP1sPluKPyH144AZv3fhhre1WfPUsIX7riQri9tg98vt\/ZHZP0e7GAoj9c7aOsjGmsimonwIfL6fP6qQudlR+aDsmxFRueCcU58FUz1U1WdHDrO396NXz6\/E8lmmbwKKqt19oJGVNtqLQT2rAn3cqsnBqWTxTjEjJ+8qFA9Fjww8Asxk\/otUAxGJu28bhTs6F9bxmJxF28qwsYpWPFN0UsPj1jGo+RaYq09mRSn9EdsFEXJIUtDYsEIi8Nqw4PJm08eiA\/+sUxmP\/DN0vtRwJZNfFauPmqS00w0UrLiq8Fk1EixtQILq3ESe1Jq3ApkbZnkqTwJHmSNB6JRJzSIIksvbcW7rpuQBIRtPKw4mvBZJSIMTWCSytxEnvSKliRyPrScFJBKJ9p41VEQk8jppUlKj8rvn10GVP7mJraUxoJekxzFi1aBI2NjTB79mzYs2dPWblFe+pCfuHMdlyEySSNapnnZTIxxywuhxMyiRMqr7+bNB5v3qpv3RuIJr+xmrW8rPj2EWZM7WNqYk9pa\/d2moNeyfBvP+eESLBSVvy0qleenzG1j6lTMol6wLwo0xw5ToKvvme9JV3uZlZ8+4rPmNrH1CmZhDUH947U1dWVnbex3Xydxn9\/12H46x+8EVS97i+ug7G\/N9C2GLHlseLHQmScgDE1hiw2g449xRaimUB7mlOkpy4qv\/UzZ9MbwpUVX1PDDJIxpgZgaSYtJJnobJHXbF9ksrjG\/+jVY3Df2leDMnAJeHQGG9J02sGKr4OSWRrG1AwvndRx9qRThm6a0PtMVEvDra2tTqc5roOuIqis+Loqpp+OMdXHSjelUzLRFTKrdFGNF8nEpVeCbWfFt68BjKl9TJ2TiXwID8\/XtLe3a9+8lgaSqMaPe\/zl4B6SvPeUqNrDip+ml9V5GVP7mDolk7Cb0nTuKLEBRVjjRa\/kwVsGw6KxV9qoLnEZrPiJoQvNyJjax9QpmYSt2rhezfnGD16Hf9x1xHnglbqbFd++4jOm9jF1SibYHPRCVq1aVbqNnjay4Y1orh4uL8JyMAdg7Ss7Y5otps7JBJuHcZOpU6fC4cOHg9a6fLgcLzzCKwbw+2H9cPhqbb9se0CjdB5FNUAyTMKYGgKmkbwQZKIhZyZJVI1vfeEINLS\/XpgpDgrCim+\/+xlT+5g6JZO8YiNhsKkaX6RVHI6Z2Fd4xjQ7TJ2SCTYLz+HU1NSUPaKVXZN\/W7LceHEVp\/6rg2DxvVflIUZsHTyKxkJknIAxNYYsNoNTMinaqeG2F9+BB9rO3lnieqOa2HOs+LF6bJyAMTWGLDaDUzKJlS7jBHLjaYqD1R5\/9JaMa9cvnhVfHyvdlIypLlL66Xotmci306vO+siNL9qSMM\/v9RXZNCWTiSli8emdkAkFXrO8Axb3r3R2dgZ7VcJOIYuNF+MlD90xBB66oyYevZxSsOLbB5oxtY+pEzKx34zoEpFM2trayp4QFRsv7i8pUrwEW8aKb19jGFP7mDonkywP+olTnbhpzj3ffQWefuu9AGEXVzNGdS0rvn3FZ0ztY+qUTPI66KfzcPnXN74DL719Ong4fPOUavtIpyiRH9lOAV5IVsbUHqaFeLg8r4N+cQ+Xi\/GS0VdeCpseuMEe0hZK4lHUAohSEYypfUydeibYnKwO+pk8XF7k4CvHTOwrPWOaDabOyQSblcVBP5Ol4aJuVqMu51HUvvIzpvYxLQSZ2G+WXonUeNz1ioRSxOArj6J6fWmaisnEFLH49EwmHR1QxMN9Ytex4scrsmkKxtQUsfj05zyZbP\/5a6WnP4sYfGXPJF6Jk6RgMkmCWnQeJhOBTP52wjVw31cut49yyhJZ8VMCqMjOmNrHlMlEIJPHJ10Lk778OfsopyyRFT8lgEwm9gFUlOicTHCr++TJk8tEy+vh8sXrdxby2gGOmWSr\/0zQ9vF1SiZhO1PtN1NdIjb+uln\/EryPg1+Rrh1gMslWC5hM7OPrnEwWLVoECxcuhMrKSvutiylRJJMiPLYVJi4rvn3VYEztY+qUTLA54lUB9psXH30+ce+aIFFRV3JQNlZ8+5rBmNrH1CmZuL62sea6m+Dk7csCVDHwigHYIn6s+PZ7hTG1j6lTMrHfHLMSB990F3w4uinIVLQ7TDhmYtaXpqmZTEwRi0\/PZMJkEq8lBil8MVJf5PRpmuucTMQDeWPHjoWmpiZYsGABzJs3D2praw3U2Dxp1d3fhNPXjGPPxBy60By+GKkvcjKZqFXtvO7u7m7xT0QkVVVVMH78eFi3bh3Mnz8fNm3aBDt37iy7ZtGizgdFXfa1v4MzA64J\/r+oy8I+KZNPsjKZ2LYmAKeeiXg5UldXV4lMkGTyWDK+7Gt\/D2cGXA1FXhb2yUB9kpXJpJeRCTYHX\/TDB8unTZsGGzZsgPr6emhoaICRI0cGN8tn+Q24fwP85qIBhV4W9slAfZKVycS+ZTn1TKg58pb6pUuX5vJcKL2TM2ZYP9h4\/3D76FoqkRXfEpBCMYypfUwLQSb2m6VXIpFJ0d7JkaVnxdfrT5NUjKkJWnppmUwK+LQFk4me8qZJxWSSBj11XudkIt\/VimLiEnFzczNUVFTYb7FQInkmRd6w5lMcwidZmUzsm5ZTMhGXhinYSr\/DpoYRCp7nmTt3boBG2FUFMkmp0hGZFO3RLfZM7Cs6Y5o9pk7JJMm7OXiT\/ZIlS6ClpSU4aUyrQTLxYNmzZs2K3PxGZFLkPSY+jfY+ycqeiX1ycUom2BzVO8BIEDU1NVorOmHvCMuko4IOyaToe0x8MlCfZGUy6WVkEnVqmJoad+NaGPGIUyEsS7XczGRiX6F8MVJf5PSJoJ17JmnUWfculLAb3ZBMRl91aXBiuMgfK7793mFM7WPqLZmYTIXC3hpGMvnMwWfhub\/5M\/vIWiyRH9m2COYnRTGm9jAtxMPl2JwkS8NIJHV1dcGW+7BP561hJJOib1jD9uXJ+GlVzBdZfZHTp\/7PE9PIU8PiOZywFRoK2Mq32dO+FDxtjN+ECRPKSCosZnLR7u8F3gl\/jAAjkB6Bjo6O9IVolFBGJkmWhjXq4SSMACPQyxEoIxNsL3ohmzdvhrVr1waXIeGS7tSpU4NdsFmfGu7leHPzGIFei4CSTLC1Osu4vRYVbhgjwAgYIxBKJsYlcQZGgBE4pxEoDJmInlBra2vkqlCePSbe6xJ2p4uYBq+7pOlh0eQkeVy\/2qiDKU23V69eHYjtQid05BQ3ebrq+yg90znCYktPC0Em4jb7ffv2QVtbWy4nlONAFDsC04rnj0TDFM8bISm2t7fDmjVrcnsRUUdOsa0YE0MjdWGgurKKmx9RP+gu4qxPrav6NazvifDwJ8YSUeY87kmO01v6O8U68d95DHCFIBOxE3CPS9xhQF0w06bDkQkND4kBlXjOnDkwadKkSK9J5\/xRWrnk\/CZyYtqnnnoKPvjgg9i22JYTy9ORFXVg8eLFMGXKlMxfQwhro46cmFckPd3d31ngKpeJpP3EE0\/AnXfeCY888ggsX748cywLQyadnZ0Bu7t2wcVOEQ8s4u+RTEaNGhV52NGFQunKScv++HQJKlccMWah9DqyEpnQ9MbFNEdHTsKHpuh5XW1q0i84uFF\/Z\/1MDZNJRM+YKBSNui6maLpy0i5lPKip42WZKK1uWh1ZaUCZOHFiQNyil4BXXOTx6cgpHwkp2jQHcTonyYTmmr5Oc1x4JGRUOi552GnwvOMmOrKSkZLnlKdBmGIqTsldyBlHrHnKVAjPxOcALM2b8SeOoi4+3aAmySYba54y68oqkrMLz0RHTtkzcSFnXN+dc2RCBonXPhZteU1cHhRHcTq0OGLEiGB3ML4zRF\/cfS9xCpDk73FyikTnkkxoOkhnuVSYyue4XOmEDqZFXxo+J8kkiQFxHkaAESgOAoWY5hQHDpaEEWAEkiLAZJIUOc7HCDACPRBgMmGFYAQYASsIMJlYgZELYQQYASYT1gFGgBGwggCTiRUYzQrR3eCWxTkfOvyFS9m6G9bC3kEya7Wd1LRcm9XyO5Wf13O4dlApRilMJg76wSWZoLHs2LHD6Ma8opFJ1kcWXJxSdqCG1qtkMrEO6W8LFDc90UiKVyzQhq2ZM2cGRi2mw9zoMQwbNgymT58Oe\/bsKb3dTCeX8UpNShf2GoBYJo2yWBbVHTbyivfK0ME1IpO+ffsGsslegZhH3GCGZ4Fee+01ePrpp0sPrlFaTDdmzBjAMukqUJXM8pUDMrFhHX369IFt27YFWBGmcrfKXl4UQTKZJDMKJpNkuMXmki+lEZ\/5kO\/qEE91iofF8B0Z+Q1nrJgIaPbs2cp7Kmgqs2LFisDw8VAfGi\/lCxvZRQMTz0h1dXUFJEREIpdH1zTQO9Mko\/yigbgbs3\/\/\/gFZIhmiXOLfqqure8gsgq0iE7qvmMrE8mSSZTKJVdnUCZhMUkOoLiDqhquoaY5oLCKZYC1ofGQoUVviZYMTz4xEXT4V9oiafOYkSn7xb1geEQv+lPPJd4GIFwuFeQ4qMomqg3qHySQjRReKZTLJEGMx2ClOK2SjopvPSBRKqyITdOXFT3WHhnwUXucgZdgLi1iXbMCi\/KoH22iqIZNTFLnIF5hjvaogq4pMampqSocsw4iOySRDRf+kaCaT7DEOagi7kUse9aM8E90b6LLwTMSpUZRHIXsmUYae5JayOM9EJqwwzyTq7hGOmSQzCiaTZLjF5pJHwrCYiep+Dyy8ubkZomImYlxEFR\/Ak8ymMRPRwNDjoGkVyqNDJpSH4iCyZ6IbM8EbwcJekFSRCf4Or9aUp4JiJ6niSISzHORlMolVb2UCJpNkuGnlEl13cZpDqxY4HWhsbAyCjRhExCDpvHnzYMOGDdDS0lIyDvwf8R5aWs2JuiYwbGUkbplXnHLJqzlIcGh4okchHsHHacmMGTNgy5YtARmuXLkSRM+EPDS8agLT4uPap06dUq7mhO0jUZEJ3me7ffv24BoIERNVjIauuUCifOWVV5SkzWSipd5liZhMkuHGuVIiEBWjiSo6LmaSUqwgO5NJMhSZTJLhxrkSICDvpwnbExJHJrhMTZ4L3sAuez8JRCtlIRl5B6w5ikwm5phxDkaAEVAg8P+pgRL+JKdazAAAAABJRU5ErkJggg==","height":166,"width":275}}
%---
%[output:4e40d541]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:563fc97c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:6761ce4b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
