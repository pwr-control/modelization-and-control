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
fPWM = 20e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_CLLC = fPWM; % PWM frequency 
tPWM_CLLC = 1/fPWM_CLLC;


TRGO_double_update = 0;
if TRGO_double_update
    ts_afe = 1/fPWM_AFE/2;
    ts_inv = 1/fPWM_INV/2;
    ts_cllc = 1/fPWM_CLLC/2;
else
    ts_afe = 1/fPWM_AFE;
    ts_inv = 1/fPWM_INV;
    ts_cllc = 1/fPWM_CLLC;
end
ts_battery = ts_cllc;
tc = ts_cllc/400;

z_cllc=tf('z',ts_cllc);
z_afe=tf('z',ts_afe);

% t_misura = simlength - 0.2;
t_misura = 0.2;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_cllc = ceil(t_misura/ts_cllc);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

Pnom = 275e3;
ubattery = 750;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 750;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:3e478930]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:3ff417bf]
%[text] ### AFE simulation sampling time
dead_time_CLLC = 200e-9;
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
RLFi_dc = 1e-3;
LFu_dc = 400e-6;
RLFu_dc = 1e-3;
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
fres = 24e3;
% fres = 10e3;
Ls = (Vdab1_dc_nom^2/(2*pi*fres)/Pnom*pi/4) %[output:431cb1b8]
Cs = 1/Ls/(2*pi*fres)^2 %[output:695452a6]

m1 = 12;
m2 = 12;
m12 = m1/m2;

Ls1 = Ls/2;
Cs1 = Cs*2;
Cs2 = Cs1/m12^2;
Ls2 = Ls1*m12^2;

lm_trafo = Ls1*12;
rfe_trafo = 1e3;
rd1_trafo = 1e-3;
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
Vac_FS = V_phase_normalization_factor %[output:5672b5d3]
Iac_FS = I_phase_normalization_factor %[output:59bf7388]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:935eeccc]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:8f4f28b7]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:8104d681]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:5ca7637f]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:5ad1539c]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:1367780f]

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
figure;  %[output:5788559b]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:5788559b]
xlabel('state of charge [p.u.]'); %[output:5788559b]
ylabel('open circuit voltage [V]'); %[output:5788559b]
title('open circuit voltage(state of charge)'); %[output:5788559b]
grid on %[output:5788559b]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:6a2ab4c8] %[output:71a1324f] %[output:85ad6fd2]
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
%[output:3e478930]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.583333333333334e+02"}}
%---
%[output:3ff417bf]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:431cb1b8]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     2.556818181818182e-05"}}
%---
%[output:695452a6]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     9.906960178361916e-06"}}
%---
%[output:5672b5d3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:59bf7388]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:935eeccc]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.094978193477487"],["23.498490613050649"]]}}
%---
%[output:8f4f28b7]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:8104d681]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:5ca7637f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000062500000000"],["-6.168502750680849","0.999018252295753"]]}}
%---
%[output:5ad1539c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.097193022720434"],["16.978803821249034"]]}}
%---
%[output:1367780f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.023362734125580"],["1.219272005551275"]]}}
%---
%[output:5788559b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATUAAAC6CAYAAADPjtq9AAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ1wV9WVP37UkmpRg7RCYxKsYcXtFj9WN01lYzdVtp0FdjvTgWSnZYF12RZxtiWTEHTashohrLqI2C2jKct2S3B2CivZzhQ1tu7atLajwrjWFkuIIcYPDGXAQrdf7JyH9+\/937x337333ffuvcl5M51K\/vfjvN895\/fOuR\/nnnHq1KlTQA8hQAgQAuMEgTOI1MbJSNJrEAKEQIQAkRopAiFACIwrBIjUChjOl156CZYsWQLz5s2D9vZ26z0eOXIEli1bBtXV1bB+\/XqoqKiw3gffYFdXF\/T29sLWrVuhrq4u177Exk+ePAmrV6+G6dOn54Jlni\/zwx\/+EFpaWqIuli9friW\/S8xFTJi+LVq0CBYuXJgnZEZtE6kZwaZXyQWpYZ979uyBW265RU9YhdKigeXZlygOI4bt27dDfX19qrS6sj388MNQU1Oj1HZq51wBRsZDQ0PQ3d0NlZWVOtXBJ1JDwVEeHAuTd9F6cYPCRGoGoPleJW8S5Q0MscjTCxWx1jEmXRyQ0Do6OkCVMHX0YLyRmu7HRQerrGWDJzVU8i1btkQ4YEjCh0TM+G644YZIUfFZt25dmcvM18fwkIVvzCCw7vHjx6NwS2xfBJ8ZBfs7Mw7RuFg5DEFQdhaKsHIjIyNlIYoYXuKPGIKxrz7+m4WfbW1tkXe2b9++qI3Zs2eXfU2ZceFv4rvy4bEKrvfddx\/ceeedY\/pi8sTJwPpHPPFhGPDjwodpPOYMB\/TQWBjP\/ib2lSRD0t\/3799fCg2ZXNhHkixxhifKwvSJjRd7Z\/x3HHEm1cfpBKbLU6dOLeHNYybiyuPG9Oqaa66JdAYf9LD4d25sbIz+fvTo0ZK+iPrIy6z7wchKVDr1gyY10SUXv7TMMNngx\/3O5oamTJkSEQMzGDZoqESoAKOjo1KPRNa26M3wpMaMU1QSZkwo+4033lg2ZyYjNSSq4eFhLVlRnvvvv7\/0QVDBleEmvptImqIsPE5IuEjO2BYbI\/69cb6G98zYGKxcubL0YYr7nZGziKmObKgHMlnE8DHtw4PExH+I0uqLuIm6LI4RjwP\/keP1geky9i3Kix8FnO9jH0FR30UdKXoed0KQmuyrzYgpbu6HhUo333zzmMl1mYHIBjEttEjy1Pgvnyz0STOYJCVOWpjg5bn11lsjY2OeG74LT+74dxFrlfBT9DrQI2N98fNKccTBL0LwYQ7KgobHeyi8Ryl6P0neRJxs+HGRfZhwQUQWcsX9xv+NEXjSnJqIg2jEaR8aLC96a8xTjPvIif2JOvzYY4+VheIMS\/ZBSdN5HRKyXTZYTy1OYUXj37RpU9kqHf+7GKYxYJnbLnogMlJL+2qpkJpsItg2qeG7MQJHY25tbQWmrLq4JnlqzPu6+uqrS15j3IckjtTYdAKv7EhkOIEvkpoYImEdRnpJnlqcbEmkliSLuOoX91Hi323+\/PlSTy1tPi+N1Bi548dDxDmO1MT+kkhNJBw2VUKkZpuKY7yHNI8Cv7S8kscZGC+maBDjyVPD92QGN2fOHDhw4EAp9NT1gEVSE3GL8wp1PDV+TNK8GWaoSR8mmWwqnppMjV16ajNnziyLOpi3zbb42PDUxHcnUsuB1Ji3wYcqSXNq7OuiMqeWpAhp3pjYNj8HkTSnJpt45d198SvP5jvYHIkYfsaFkOIQ8CGYuGdKxStIm4vESWmcz0FvmV8MMZlTE+fvZCFQ3NySOE+aJJtITGmhMY9pmjetO6cmjqFsTBipoTw4\/8tCR1n4aTKnxq8Mp9lDTiav1Gyw4Sd7O5VVOpwjuuOOO6IqstVPfqVQx1NjsuiufibNAYmrn7xnhf+NIRiuyMatfrIVTYaLbMWWlYlbiVPBla00i309++yz0XwMPmxVbfLkyRHJ4cMWB1A2NjZJq59YnskX50XKVv3Yhw9JleEgk40RCU6aM0JgE+hsjGXbPWSrlyqejcrqJ8Nc\/Ijyq7Sox\/hxZvqRtMjF1xF1ChcTxNCeH6MJu\/qJyo5P3C56ETBx64ESJacUks1T2Wif2siOgO5+J94T093Aml3a8dtC3FYf2dvqjluRyOXmqbGXTjoOgr\/39PTkeqyHSK1IVVLrK26bDb+dJK0V1Btc2HBxRCtNtpB+F7csoeziqrfsfXz+uORCauhGr127NsIk6Yweus+Dg4Na5990lYZITRex\/MuLIRYfXqr0HvLZT5X3K7KMOF3Cbz6XyTEhz34imdTW1kaklRR+8nM2uopd5MBTX4QAIRAWAtY9NXRrt23bBrfddlu08hVHauxr29DQEO0Mp5AiLKUhaQkBnxGwSmpIVp2dnbB48eIoJY1soYAHRSQ5\/rdLL73UZ\/xINkKAEEhAoK+vD2bMmFE4PiVSi5vrSJMGVyx37dpVKha3uxt\/TMsdxUitubl5TMoXJLWBgYE0Ubz8PWTZEVCS361aEf5m+JeR2qpVq2DNmjVKif+QwO66665oFSrpSfLUkED5vjD8xLJxuZlCHtiQZSdSMzMom7VIf8zQtBp+iiLwpCYSGe\/VyTaIhjywBw8edOJ+m6nC2Fokvy0kzdoJHX9Xtjsm\/CwqJbTqMLsCRlU+WbnQlZLkt6EF5m2Ejr8r2y3z1MR9K+KRIvPhMa\/pChhzid+pGbpSkvw2tMC8jdDxd2W7ieEnv48sjyNMqkPtChhV+chTs4FUPm2ETgqhy+\/KdlPn1JIS6uWjhmNbdQWMjfcLXSlJfhtaYN5G6Pi7st1UUuOHhK143nPPPdq34ZgOrStgTOXl64WulCS\/DS0wbyN0\/F3ZbiqpydKhmA+Xek1XwKhLmFwydKUk+W1ogXkboePvynZjSU3cRJt2i5L5sKXXdAVMumTpJUJXSpI\/fYzzLBE6\/q5sV3qiII\/7D3WVwBUwunLGlQ9dKUl+G1pg3kbo+Luy3TGkxt+paD4c9mq6AsbGG4SulCS\/DS0wbyN0\/F3Zbq7HpMyH852aroCxIXvoSkny29AC8zZCx9+V7Vo90G4+fMk1XQFj411CV0qS34YWmLcROv6ubDd19dN8SOzUdAWMDelDV0qS34YWmLcROv6ubJdIzVznUmuGrpQkf+oQ51ogdPyJ1BLUwxUwNrQ1dKUk+W1ogXkboePvynbJUzPXudSaoSslyZ86xLkWCB1\/IjXy1HI1EJPGQzcqkt9k1O3VIVIjUrOnTZZaIlKwBKRhM6HjX3XDZ2D4e98wfHvzahR+mmOXWjN0pST5U4c41wIh49\/z49dgRc+LcOTej+WKUVzjiaTGLkPp7e0FvOR06dKlsHHjRigyQwcK7MqFtTESISslvj\/Jb0MLzNsIGX\/vSI2\/sq6mpgZ6enpg\/fr1sHv3bujv74\/+u6Kiwny0NGoSqWmAZbloyEZFpGxZGTSb847U+EtSRkdHS6Q2PDwc3SBVpLdGpKapTRaLE6lZBNOgqZDx947UEH9M5z0yMgILFiyARx55JAo\/V6xYEYWi7e3tBkNkVoVIzQw3G7VCNiry1GxogHkbXXsGoWvPQb\/m1PB18D7OlpaW0pu5uIiFSM1csbLWJFLLimC2+iHj7y2pZRsSO7WJ1OzgaNJKyEZFnprJiNurM+FJLekmd4SYSM2eoum2RKSmi5jd8iHj7x2pifcSxA2VrRTfLMRdvnx57FwdkZpdQ9FpLWSjIk9NZ6Ttl\/V2oaC2thYWLlxYemO87HhwcDAiH\/xv3N5x3333GSOC5Ll27dqoPpJk3AIEkZoxvJkrEqllhjBTAyHjjxtvkdi82XzLb+moq6srDQx\/RR5u9cDtHVu3bjUeOAw7kTiRKPEhUjOGMpeKIRsVeWq5qIRyo\/MfeA6eOnDUH1JDydkN7ezyFTFMzOqpIUFu27YNbrvtNti0aZOU1PDHvr4+ZUB9KYj7+qqqqnwRR1sOkl8bMqsVQsW\/qakJjt3UBb9\/z0V+kRqODn9VHj+HlvVSYzyx0NnZCYsXLwb0BGmhwKotWGuMPDVrUBo1FCr+Q0d+BVfe+YPonb0JP41GQKOSeK8oqxq3WEBzahrAWi4aqlExGEh+ywqh2NyEJDURG\/LUFLWl4GJECgUDLnQXKv5sPu3ME2\/Cm1\/7dOEgJmbpEE8TMMlmz54N3d3dUFlZaU1YIjVrUFptKFSjIk\/NqhpoNca2cmClsw+\/CG984\/Na9W0UjiU1tk9t5cqV8MQTT0RzXzjhvXr1amhoaCjb5mFDCFkbFH7mjXBy+0Rq7rDHnkPDnw87qysnwbGv\/zUMDAwUDmIiqa1atQrWrFkDO3fujLZd4H61rAsEJm9HpGaCmp06oRmV+NYkvx09UGmFJzQsv\/vzV8Fnb7raH1Jj+dRwxbOxsTE61P7QQw9F2TqGhoash5\/kqamoTfFliBSKx5zvMQT8kcy+88KbsHrXSyXR\/+uWq6Dh0gucHXGUpvPevHkzzJ07F3CjLRIbph0qMkEkokSemjvDCsGoZOiQ\/Pnqzv7XT0B919NlnTx3+0egpnJS9DdXtkt3FOQ47mRUOYKr0DThrwCSZhH0zDBPGi4I8A\/OoWHIif\/PHiK1BHBdAaM51rHFyahsoGjeBuFvjh1fE4lsw6ODsP1Hr45pMI7MvCQ1lbOfNrd00JyaHeWz3QqRgm1E9dpzhT+SWERkew5G5zfjHhmZeUVqSTv9+Zcqel6NPDU9Q7BZ2pVR2XoHkl8NSSSwTU8Mwf7Xf5lIYtgSIzL232mtu7Ld1C0dfJaOtJfI43dXwNh4FzIqGyiat0H4j8UOCezex1+GgcMnpATGiOujH7wAHmieZTQIrmyXFgqMhkutEhmVGk55lZqo+CNx4fPVJw\/BT0beSiUvhj96YjddMQVuuaG6bMLfdHyck5pKtlt8uTyOSclAcwWM6UDy9SaqUdnAzkYb4xl\/Rlx7h4\/DQ\/8zDEO\/OD0PpvoggWXxwlT6cWW75KmpjI5hmfFsVIaQFFotdPy\/v+8luOSSS+DBp4Zh36Hj2sTFQsjqCyfBZ+qnw5\/MON+KB6Y6iERqCUi5AkZ14GTlQjcqkt+GFiS3gZ7VKQD41\/5X4JmXjxmRFiMu\/H\/0vG79s2qoeNdZhZJX0hu6sl2pp4bZbTs6Okoy072fekpOpKCHl+3SLvFHwsIQb2Pfy\/DET49Er5a0PULlvdmmViSuf2iqgXeffaYXxOXj1FEiqWE6IEw\/xNIMsTm3+vp6uqFdRQsDzLIgvpZLUlCEWFrMtvz8nNXJ3\/wOvvn0q7DXMCwUBedJ66+ufD98fFZlcFk6xHfyylOjzbc2TCq81DETmdQYYf3+1Cn4z71vWPGuGJ6MsHBu64rp58Hf\/2kVnHnGGamelm1StqPV6q14RWooNnlq6oOXVDJ0pRwP8p91\/rRoeHBj6a69b8Ah3C2vuVIo0wSesP649ny4\/ZOXwvAvToeeWZ\/Q8feO1HBAaE4tm1qGrpS+ys+8qleO\/l90HvHl0ZOZ56ySwkH0rmZcVAGLrr0YPnDBaaJCwmJzZtk0RF7bV\/xV39lLUlMVPs9yroCx8U6hK2VR8vNzVf0HjkYT6kOjJ616VHGhIHpWn77m\/XDuOX6sFo638N+V7dI+NRvsldBGUaSQ1ytkkZ8R1evHfw09P3oVfv7GiUhMm6FfHFFdUjkJPls\/Haad\/244dOgQfHT2O5dx54VTXu1mwT8vmXTaJVJLQMsVMDqDNxHm1BhJvXH817Djx69Fc1R5kRQL8aL\/v3ASVE+pgPkfngqXX3xuKfxTGZ\/QSSF0+V3ZrvTilerq6sIz3YrK6goYFaNJK+OrUvLh3nOHjsHjLx4pzUvl4UmJJIX\/vrb2fGi+7mI456wztYgqDXP+d1\/xV32H0OV3ZbuJ4WdcGiLafKuqjqfLFaWUbNL6p6\/9Mpo4x71TeXpRcSQ1a9p50HLdxXDhe95VIqkiJtNlI1IU\/npaoV46dPm9I7U46HE1dMeOHdKLV3gylB1+F0kzqawrYNRVL7mkiVIyL6rqwkmw5b8Pwf+OvBVtQyiapK6pmQzXT\/sd1M2o9oakdMfEBH\/dPvIsH7r8rmxXy1PD26W2bt0KSTnW8Baqzs7O6J5QLIMk2N\/fHxvC4mmFnp6e1PDWFTA2lJUdSMa2Tvz6d1EqmME3T28\/yCvMY3Lz+6dwTmrOZRfARy69oPRaKtsSQjcqkt+GFpu34cp2pXNq+DpZbmOX3ROKhDc4OJh65MoVMLKhZN7Uzw+fgG89+7r1DZ1i3zxB4ereFdPOg7lXTIFzzi6fj7Id7hEpmBu0jZqh4+\/KdnPd0iHz1PDEwpYtW0pjv337dsBzpeLjChhGXHhzDoZ\/WQ4j8+\/EExT+fda0c6H1xlo4+Zvfl3lRkTf39qFoGwZi0kboRkXym4y6vTqubDcXUuMTTsaRFbssuaGhIbr5HUPR1tbW2NC2KGCSrv5SGWKeqJCkvvQXH4TRt35D+6RUwMuxDJFajuAqNF2U7Yqi5EJqrBNGbu3t7bFeGCsnkhwvJAKDT19fnwKMekVGjv0WvvL4m\/DMK\/KModMnnw3T3ns23FR3LjTUVESd4N\/SnuHhYaiqqkor5u3vJL\/boQkV\/6amphJwAwMDhYOYK6nJyIp\/U1auubl5DPnlwfb\/9OggrPvOwViw0ev6x\/mXwZVV7818KJk8hcL1uaxDwt8t\/nnYrsobad0mJZv4x87ElEVYvq2tDTZs2FC2YiqWw\/AT59jiFiVsAYPh5U9efQtaup8vwwVJ7C+vfB8sbfhAZhITASejUlHB\/MoQ\/vlhq9KyLdtV6YsvU0ZqNu79FNtgc2pxhLdkyRIYGRkB2VYRW8BceecPyi6mULmMVRdMIrWsiNmtT6RmF0\/d1mzZrm6\/Wp6abuM2ymcF5qmfH4X5X32uJAqS2eZFs+D6y97Zs2VDzrg2yKjyQlatXcJfDae8SmW1XVO5cp1TMxWKr5cFGJHQdn3uSmisu9CGWEptkFEpwZRbIcI\/N2iVGs5iu0odJBQqkRpbqTx8+DDcfffd0RzXvn37xlQL5d5PntCK9M54wMiosqhm9rqEf3YMs7TgnNSyCJ9nXRNgRELb\/fmrrC8CqLwzGZUKSvmVIfzzw1alZRPbVWk3rcy4Cz9xlRMXBfBx5aEx0Mmo0tQv398J\/3zxTWvdK1LjTwSIgvscfiKh3dLzYulIE3poRSwIJA0uGVWa2uf7O+GfL75prXtFaknC4jxbY2Oj9HRA2ovq\/q4DDB92Xv\/BC2D3iqt0u7NanozKKpzajRH+2pBZraBjuzY71go\/0zbf2hSMtaUDTOUXv1sKO13No\/EYkFHloRHqbRL+6ljlUVLHdm32r0VqKkkibQqHbakCs\/m7Q\/Cl3gNR967DToYBGZVtbdBrj\/DXw8t2aVXbtd2vNJ9a3JaOpBRBtgXT8dTExYG9t38kL3G02iWj0oLLemHC3zqkWg16RWpakudcWAWYbT8YgS\/8x88iSR5ongXN116cs1RqzZNRqeGUVynCPy9k1dpVsV21lvRKJYaf4llNWcJHvS71SqsAw8+l+eKl4VuSUemNte3ShL9tRPXaU7FdvRbVSseSWlLKINUU3Gpdq5VKA6bnx6\/Bip4XvfPSiNTUxjfPUkRqeaKb3naa7aa3YFZC60C7j6uf8x94LtqXhhttffLSiNTMFNJmLSI1m2jqt+UVqSUlbZTlPdN\/ZbUaMmD4fWk3X\/8B6PrUTLVGCypFRlUQ0AndEP5u8feK1BAKDDU7OjqArXYiobW0tEDRFxrLgLnj2wPwz30vRyPnyzYOXo3IqNwaFeHvFn\/vSA3hSEr4WCRUScD4uo2DSK1I7ZD3RaTmdiy8JDW3kJzuXYXUfNrGQaTmg9acloFIze1YeEVq4nYOl9AkAfPvT78Ktz7800g0XCBg19S5lFXsm4zK7WgQ\/m7x94rUEAo8vF5bWxvdy+nySQLG51VPhhcZlUvNIU\/NLfrqRxxty6l9TMqH1EP8fNoXP14Dt3\/y9N2gvj1Eam5HhPB3i793nppbON7pPQ4YfsOtj6ue5Kn5oT1Eam7HgUgtAf84YFjoiVWO3PsxtyMn6Z2Myu3QEP5u8XdOaiFdvMLOevqQCFKmNmRUbo2K8HeLv3NSK\/r1+T1wsnk6ERh+Pm37sj+CP\/\/Di4oWXbk\/MiplqHIpSPjnAqtyo96RWp5ZOvAYVmdnJyxevBjq6uqi0wv9\/f2wfv16qKioKANNBKZrzyB07TkYlfF1KwfNqSnrfa4FidRyhTe1ca9IregsHbKD8iIwocyn4YiTUaXqfa4FCP9c4U1t3CtSS9p8m1eWDh1PDa+\/wxDUx6wc4iiTUaXqfa4FCP9c4U1t3CtSKypLB38VX1KacB4Yfj7tpiumwI6\/\/XAqsC4LkFG5RJ88Zbfoe7b5FsEoMksHI7f29vYx1+8hqeHT19cHz7zyK\/i7na9F\/\/7Kxy+CebPOcz1u0v6Hh4ehqqrKaxllwpH8bocuVPybmppKwA0MDBQOovQ2qaKydCTN4SEavKf2vf1H4FNf2xeB5POmWzaK5KkVrs9lHRL+bvH3KvzMGwpxzg7Js62tDTZs2BCthvIPD0wI5z152cmo8tYkefuEv1v8JxSpIdSqXiAPDFsk8H3TLXlqbo2J8PcD\/wlHaqqwM2D4RYJ5H54K2\/7mQ6pNOCtHnoIz6KOOCX+3+BOpJeDPgOHvI\/A1KaT4CmRUbo2K8HeLP5GaBqmFsEhAnoJbgyL83eNPpJZCaiGdJKA5HfcGRaTmfgy8IzV2e5QIjaskkaGtfJJRuTcqCj\/djoFXpMY2wy5atMibdN6hpBvi1YiMyq1REf5u8feO1FatWgVr1qwZs2+saJgYMIzU2ufOgPa5tUWLYdQfGZURbNYqEf7WoDRqyCtSwzfAY1KDg4OAR5dcPgjMvz36LMz\/6nORGKEsElD46VJrTvdNpOZ2DLwiNf6guQ9zakRqbpSTSMEN7uNlockrUnM7lOW9IzCf6Pw24GUr+PieGJLm1PzRHiJlt2NBpJaAPwLzoVXfgqcOHA0ihxqRmltDIvz9wd87UmOZM3p7e2HevHmwdOlS2LhxI9xzzz1QWVlZGHIIzOSl34wSQ4Zy5nO8hA\/k6RSm5rEdhY6\/V6TGpwKqqamBnp6e6P6A3bt3J94lkNfw137oOjh2U1fUPF6ygpethPKErpQkv1tNCx1\/r0iNTw00OjpaIjVMWnfXXXcV6q3xpBbKmU\/y1NySAeHvB\/5ekRpC0tXVBSMjI7BgwQJ45JFHovBzxYoVUSha5DaP6us+AW9d3xaNEpFascoauqdA8herL2Jv3pEaCigelVq3bl3hJwyqbvgMnLh6aXArnygwGZVboyL83eLvJam5heR079M\/+QX41eXzidQcDAaRggPQuS5Dx59ILUF\/3vfZf4HfXnR5cNs5yFNzSwiEv3v8idQSSe1r8NuL\/oBIzYGOhu4pkPwOlIbr0jtS4\/epMTmXL19e6CIB9htidg5afXNrTIS\/H\/h7RWqM0KZPn14iMfY3hAv3rFVUVBSCHCO15msvjlY\/Q3rIU3A7WoS\/W\/y9IjXxCjsGDd4AVfQ+NUZqoW3noDkdtwZF+LvH3ytSQzhwOwc7ScC8Mty7VltbW+i2DiI1d8pJno477McDKXtFarLUQ\/wwY2rvXbt25TryjNRCys5Bczq5qoRy40TKylDlUtArUsvyhuKG3e3bt0N9ff2YJsXLjJPuPiBSyzIa2eoSKWTDL2vt0PEfF6SGHt7atWvhy1\/+cpTJAwkOQ9bu7u4xmT3iwts4JWCkduTej2XVkcLrh66UJH\/hKlPWYej4e0lqmNK7o6OjBLTuMSkWxuJZUdFbU00XjqRWXTkpSg4Z2hO6UpL8bjUudPy9IzX0sNCbYl4WIygkJ9UD7RhitrW1wYYNG8Zc4ILtb9mypaQ1SWEqkZo7wwrdqEh+d7qDPXtFaja2dPA52RYuXFiGrvgbkmdrayts3bp1DPkhqZ395s\/g6bVz3Y6QQe+Yqqmqqsqgph9VSH634xAq\/k1NTSXgBgYGCgfxjFOnTp2K6zWLpxa3eVf2ZjICRFL7XOMl0LngssLBydoheQpZEcxWn\/DPhl\/W2l55auxlTObUZASVBBKr09zcPGbuDUktpLs++Xd0NahZlZHVJ\/ltIWnWDuFvhluip2bSnCqhieGtbJUUSe09z34dzhn6volIVIcQIAQcIuBV+GmCg7j3jLWBiwAzZ84E\/tZ3viyeMY2bTzORgeoQAoTAxEbAqqc2saGktycECAEfECBS82EUSAZCgBCwhgCRmjUoqSFCgBDwAQEiNR9GgWQgBAgBawgQqVmDkhoiBAgBHxDwltT41VEXacR1BkdFVjE9Ot6fWmQGYdn7qMjP15cdf9PBzVZZVfn5cklZYWzJpNOOqvx8BhzfbYK9v4scjF6SGn8QHpVv9erV0NDQUGhySlWlVJUVNzLjg0fGVPfzqcqQpZyq\/KwPJvszzzzjxTYcVfnF5AqqCRWyYKtSV1V+\/kOCR+98tgme0PB8t24iDBXcZGW8JDUxbTgqYH9\/vzeejei18CnOVWVVLZd1gNPq62KNcj\/\/\/PPwwgsvxCYqSOvP9u+q8qOX8+STTyonY7AtZ1J7OvLzabzwv\/FRTS5R1PtgP\/wxSfx30dmyvSQ1Mdea7MRBkYMV15eprL4opY78zAAx9EH547KvFD0eqvIjGR8+fBj6+vpg37594Ev4qSq\/qkdXNP5p\/VH4+TZCohfjM6mZyOrT++jIjwra2NgIU6ZMSUwplabktn9XlR\/L3X\/\/\/aWQ2ZePiqr8iBufZj8pVZdtfLO2R6T2NoKqX6+sgNuoryurLM2SDXl021CVnw\/ffFooUJVfnEPz5cOiKr8ory+knKZvRGpvI6Q6z5AGaBG\/68jqiyGZzAmKST2xDR\/O7Krir0oeReiMKf78HJqPuhSHHZHa26iENH+gKquvSqgqv2iISRmNiyYFVfn5zDBs9ZC\/rLtouVl\/qvK1Jw36AAAEA0lEQVTHeWojIyNeLp7xWBKpcWio7t1xpYyikS9ZsgRQyfj9Q\/yAxnk6vuxVS8I6SSF9Cj9xHFTl58v5gr2O\/Hx+Q18WOtLsj0gtDSH6nRAgBAiBFAS83NJBo0YIEAKEgCkCRGqmyFE9QoAQ8BIBIjUvh4WEIgQIAVMEiNRMkaN6hAAh4CUCRGpeDgsJRQgQAqYIEKmZIjdB6\/EplHTS3\/i4A56l8slzEzG\/jSSUo02hqzaRWkAjqJPZQ6esDgSmm4h9JbWenp7cN7DK7rXVwZ7KqiFApKaGkxeldIhKp6zOy4nHjVTrEqmthrjLulXxo3LqCBCpqWNVSEkxQy4L8fisp2w3\/PDwMLCTDCicrCy2u2zZsijtDj6yUIjPBsGX5WVICtmS7nNFUjt+\/Hj0v97e3jHnRvm2sU+WWJCd7Zw8eXJUj8nNn9DA98b63d3dUFlZWXbCQBZaigQtyijbtS+StOwjQp5aIaZT6oRIrVi8U3vjs0mIxsAbDjaE2U\/Z1188uhRXlmUPlh1zErPyillFZOGnmF2WL\/vggw9GpMQurUZSYGcXsU\/+omu+3ujoaETcK1euLGU+5n+vqKiIcBgaGopIDR8kb0yeWF9fH5Edn1yRH4A4UsNMrTxxJp2vJFJLVWVnBYjUnEEf37GY94svJfMGRPLhy6JHx2fnxTZVz3XGkZyMJJJ+0yEBlH3Hjh0RSSGpiYfnxbZ44tq\/fz\/w82QyLymO1HgSk5G\/zvuQp1askRGpFYu3Um\/8wWU+fBJJjQ\/BMFTCh2Wj5ctiyNnS0jKm77jVS9Gz0SE1GenKSIB5nejJ4TNnzhw4duxYLKnF3e\/Ay\/zYY49BR0fHmHeNy5MfR2pYkaXI5jN71NXVlbVJpKakyk4KEak5gV29Uz5M2717d+muBvS+eA9GFn7GeWpJEojt6JCaLNSTkQCSLu\/hieFnFk9NhjR5aup6GFJJIjXPRivO+AcHByPvQQwpca7p7rvvjuaOsB4\/ZyWbU2NzX4sWLRpzQ5fNOTWeIHfu3Bkhzbwg0ZNsbW2N5ttYrjM2RxYXfurMqbGFC4ZT2pxa0rwfzt2JUwEsRGbzevh73LWHFH4Wa2REasXindqbuPrJr8AxA506dWoUmuHkO05s47N58+bo32yCXCyLZfjVT9nG2aTVT2wjbZ8av\/qJ5flJ9yRS48NPDLfXrFkTvQuG0vjEJaRkoTeWx\/dCLzZu9RPrJ13RluSpIaGKl7OIoSiPEZNh7969EamJCx9Eaqlqb7UAkZpVOKkxFwiY7p1Lm1Oz9S5EaraQVGuHSE0NJyrlEQJx215MUnMTqXk0qBZFIVKzCCY1VQwCYnhsmppbPPspzvvZeBs6+2kDRb02\/h+9bcvvjwZHmAAAAABJRU5ErkJggg==","height":186,"width":309}}
%---
%[output:6a2ab4c8]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:71a1324f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:85ad6fd2]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
