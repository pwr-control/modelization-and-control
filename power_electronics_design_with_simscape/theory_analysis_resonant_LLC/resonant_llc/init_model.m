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

model = 'dcdc_llc';
rpi_enable = 0;
rpi_ccaller = 0;
%[text] ### Settings voltage application
application400 = 0;
application690 = 0;
application480 = 1;

n_modules = 1;
%[text] ## Settings and initialization
fPWM = 4e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*5; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

fPWM_LLC_pu = 0.65;
fPWM_LLC = fPWM_LLC_pu*fPWM_DAB %[output:7c09ef44]

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

t_misura = simlength - 0.2;
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

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:076b441a]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:02159090]
%[text] ### AFE simulation sampling time
dead_time_DAB = 1e-6;
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
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/4) %[output:5d197f8f]
Cs = 1/Ls/(2*pi*fres)^2 %[output:53fa3074]

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
Vac_FS = V_phase_normalization_factor %[output:215718df]
Iac_FS = I_phase_normalization_factor %[output:4f58960b]

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
%[output:7c09ef44]
%   data: {"dataType":"textualVariable","outputData":{"name":"fPWM_LLC","value":"       13000"}}
%---
%[output:076b441a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.583333333333334e+02"}}
%---
%[output:02159090]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:5d197f8f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     1.278409090909091e-05"}}
%---
%[output:53fa3074]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     4.953480089180958e-06"}}
%---
%[output:215718df]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:4f58960b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:0bd29de1]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.345132058575099"],["81.455860923503209"]]}}
%---
%[output:915851da]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:46c059b7]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:7459e256]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000250000000000"],["-24.674011002723397","0.996073009183013"]]}}
%---
%[output:4579d87c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.388772090881737"],["67.915215284996137"]]}}
%---
%[output:74e07718]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.091119986272030"],["4.708907792220440"]]}}
%---
%[output:0825f5ce]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAd8AAAEhCAYAAAA+mKoxAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ+QHdV15o8T7NUAIWiEMB5kIQEjm9hlCWFcYxkbYwWz3kQiobKlGW2tnfHYO1tGZncl1YwEQRUMSBqVRJYCvNE6g4KT0kgbBxYpsRdjBVPGQg4WMLGzEA36a2kACwkZY4RTuLR7Wr7DnTv95+v3uvv1ff29qlSM5ut\/v3Pe+d65ffv2O06dOnVK+CEBEiABEiABEiiMwDtovoWx5oFIgARIgARIICBA82UikAAJkAAJkEDBBGi+BQPn4UiABEiABEiA5sscIAESIAESIIGCCdB8CwbOw6UncPz4cenp6Qk2HBwclNbW1vQ7AbcYGRmR7u5uueKKK2Tt2rXS0tICblmfrMhrRM7UcPjyl78sixYtQjYptWZgYEA2btwYnGNvb6\/09\/enOt+tW7fKypUrZc2aNaXkode3a9eu3L8fqaBRHEuA5ssEKT2BIo0pyny1uF199dXS0dGRC6+oa8z7uFEXU0sx1+L\/+OOPpzK2WrZJGwA9xuLFi8c2a0bzbbYfS2lj7KOe5utj1HjOhRE4efKkrFixQrZv3y6bN28uzHy14y7iuGEgTSFfsGABbKSmM0xjbLVsU0vgjfmmOTf3OGXvfE2eHjp0iN1vLUnSgG1ovg2A3shD2sNveh72MJrd9c2ZM0duv\/324FS1CNtDsKZLGx4envB3ex\/XX3+9fOELXwg0bW1tsmnTJmlvbw+9fNvkXL3bFerfzTD0\/Pnz5a677pLZs2cHRcc2raT96PC1Oe7u3buD89OPGXZetWqVfOUrXwmM13xcFvrvhqltzqbg23pTwM2+bDOwr\/Hee++VdevWhR738OHDwfmNjo6OnZMdQ5ujMr\/vvvvk\/vvvF3N9yt9lbdiZ4fwwozFxtY9rrte9LhPradOmjf2AcPlt27YtGMY1Hzs\/3P0l\/ehx8zFuX3F5GNch20z0nM25u4bufr9stmYfS5culR07doh+f0zs7GvWfzPHsGObxCUsDxtZa3jseAI034pkiFtw7cs2BSSswLpFU\/ejxmeM1\/17mDnEGZf+LercTKGcMmXKuHu+xnztc1CTCzNL24Dd\/WRlvmGdlVsI3aIcxVX\/Pcp8+\/r6ZMmSJRPYx5md\/m3q1Kly9OjR4MdFmCHqMW2TcM89Ki\/McZ9++ulQI33wwQfH7rPa+Wabi2u+7r7M36MMOC5ndZuDBw9Gmrx9Tq7xuj+QjPF9\/OMfl+9973vjqkaYgYZ9v1zzVE3YOeq\/m+Mk7dvmUvbuvCKlFr5Mmi+Mym+hKS72L39TuPTK7K5PuxvzpbaLm10o7F\/8drFWgzOdmdmHObbbYRmiYX+3C8m1114bab52Z5B2P0nmq92+fpKGf+M6c+3Gjx07NoGJ3a0pp1mzZo27RmTY2R0SN+xNPLXLdeOu56L3P8M6cmW5cOHC4HrtThmZhIYMIbsa978NE\/NDQc8\/6dgm98Kux\/yb\/kjTa44adrY5mnxyY\/roo48GJh7WyUbt1x39MN2+vY+4Y5vO2OR\/Epcshtf9rnJ+nT3N16941Xy2Ub+KTfHSojN37tzQmb625sCBA6HdjJ6YvQ\/ttszM5KQJU0m\/2KPMzS5Gevy0+8nKfO1jq5Hqxy72UUXRNp8vfvGLsPmGjRSEHVfPwx1Wj+osVasmYs7DZht2PHf4Pc58o4bb3W3iutiwH27utZlbGq6Jmx8cUSaZlJ92fO19RMXV7aINK2O+Ubcb7Jn8di6b76U95G8Kgc0l7FZHzQWDG+ZOgOabO+JyHKBo87Uf1UkqbmlNU4mGPXqE7sc2FrdQ677tR42Qzlc19iQl\/W99rMXt\/N3in9Z8XY5ud+yaflbmazI4yvR1BniY+brD10mdrw\/mGzbSYuLq5l9U52vvI+q7QfMtR93M8yxovnnSLdG+ax12dodHzT20qC4ibJgwyXzjhovtbkxxancQZb7ofnQ4zzVGMxzvmq8aHDKRxTUmuzN0h+7VrJKGnbUrTzIvdx+1DjvbaRrVTbqp7Bqp2wWaa7ZHQMz1mNxxtwkbdk76CuU97Gx+qJkRgyjzDRsxMIzczjdqgpw75B037BzGhcPOSdlSrr\/TfMsVj9zOJu8JV3HmlWS+cecWdj80ynyT9qNDdOb+rQsaMV\/dJmy2s9mXO2PVXpwizYQrM\/xob6PHveGGG4KuPOyjnMKuD51wpfs0P0hc04+ajGRvY2v0mHfffbfccccdEyaH6Tau+eq\/RU3eMtea9GMvbEg2aeTB5hh1jXHGaZvdTTfdFJlbcfvQcwibiIVOuLK5JI385FZcuOOaCNB8a8Lm70boo0b2Y0JJjxqFTeJKM+ysNOOGNJMmNNkrXsXtR4\/jPpZy6623yrPPPjs2wSis87ULc9SkMXvf7r3oMHO2TcjeVv+3Md+w437ta18bt1KTLvxh31+u5VEj20RtMwh7DC3qESeXq30PWvep3NavXy\/Lly8PcNgjGGbWetSjS0nP58Y9aqTHQjvCqHu1OvoRZmxR3b4yCnvMK6x7jvrhpv\/urqgVde\/c7AMZofG3cjXfmdN8my+mNV9R0szSmnfMDQshEDa87c5oj3rO2j7BWhbZKOQCm\/Ag9o8l8yMjbAZ00qVzkY0kQuX7uzfmqwVBn3PUxQfCCkhYx5P0i7l84WjsGdF8G8u\/3qPHDbvHDZeHHbeW5SXrPf+qbh827KwskhamCfvB1CxrcVchF7wwX2QSiA49LVu2TG6++ebIVZSqENB6rpHmWw+9cmzrDsHqWaU1Xt2GawUXG0\/3dlAa49Uz5Y+lYuOVxdG8MF+9t6LJpZ+ozleLxerVq2XDhg25vvUmC+jcBwmQAAmQQLUJlN589Zf8bbfdJl1dXYEBR5mvMei8XzlX7XTh1ZMACZAACWRBoPTmq\/dD9KOrvMTd83Xvm8TNSs0CHPdBAiRAAiRAArUSKLX56lDyAw88ILfccovogv1x5qtdsU7xN2\/fcf\/bBXTxxRfXyozbkQAJkAAJlICAvh1q5syZJTiT9KdQavO1XySeNNvZvfQkvZrvvn370hOr2BbkhAWcnMgJI4CpmE\/Nz6m05hs2a9OEI+n9nqpLmoDF5G7+5MauMBsV8wnjSE7khBHAVD7nU2nNN00nax5FmjdvnuiSfua\/dbp+f39\/aBR9DhqWltmoyAnjSE7khBHAVMyn5ufkrfkag9VZ0LpQfdzC9mFhZHJjyb1\/\/35v76lgV5iNipwwjuREThgBTOVzHffGfLFQ4Cqfg4ZfZf1KFkuMITmRE0YAUzGfME4+13GaLxbjyqpYBLDQkxM5YQQwFfMJ40TzxTiVSuVz0IoEySKA0SYncsIIYCrmE8bJ5zrOzheLcWVVLAJY6MmJnDACmIr5hHGi+WKcSqXyOWhFgmQRwGiTEzlhBDAV8wnj5HMdZ+eLxbiyKhYBLPTkRE4YAUzFfMI40XwxTqVS+Ry0IkGyCGC0yYmcMAKYivmEcfK5jrPzxWJcWRWLABZ6ciInjACmYj5hnGi+GKdSqXwOWpEgWQQw2uREThgBTMV8wjj5XMfZ+WIxrqyKRQALPTmRE0YAUzGfME40X4xTqVQ+B61IkCwCGG1yIieMAKZiPmGcfK7j7HyxGFdWxSKAhZ6cyAkjgKmYTxgnmi\/GqVQqn4NWJEgWAYw2OZETRgBTMZ8wTj7XcXa+WIwrq2IRwEJPTuSEEcBUzCeME80X41Qqlc9BKxIkiwBGm5zICSOAqZhPGCef6zg7XyzGlVWxCGChJydywghgKuYTxonmi3EqlcrnoBUJkkUAo01O5IQRwFTMJ4yTz3WcnS8W48qqWASw0JMTOWEEMBXzCeNE88U4lUrlc9CKBMkigNEmJ3LCCGAq5hPGyec6zs4Xi3FlVSwCWOjJiZwwApiK+YRxovlinEql8jloRYJkEcBokxM5YQQwFfMJ4+RzHWfni8W4sioWASz05EROGAFMxXzCONF8MU6lUvkctCJBsghgtMmJnDACmIr5hHHyuY6z88ViXFkViwAWenIiJ4wApmI+YZxovhinUql8DlqRIFkEMNrkRE4YAUzFfMI4+VzH2fliMa6sikUACz05kRNGAFMxnzBONF+MU6lUPgetSJAsAhhtciInjACmYj5hnHyu4+x8sRhXVsUigIWenMgJI4CpmE8YJ5ovxqlUKp+DViRIFgGMNjmRE0YAUzGfME4+13F2vliMK6tiEcBCT07khBHAVMwnjBPNF+NUKpXPQSsSJIsARpucyAkjgKmYTxgnn+s4O18sxpVVsQhgoScncsIIYCrmE8aJ5otxKpXK56AVCZJFAKNNTuSEEcBUzCeMk891nJ0vFuPKqlgEsNCTEzlhBDAV8wnjRPPFOJVK5XPQigTJIoDRJidywghgKuYTxsnnOs7OF4txZVUsAljoyYmcMAKYivmEcaL5YpxKpfI5aEWCZBHAaJMTOWEEMBXzCePkcx3PtPM9fvy49PT0yPDwMEbu16rZs2fLQw89lGqbesU+B63ea0+zPYsARoucyAkjgKmYTxgnn+t4Lubb398vHR0dEL1du3bJwMAAzReiVbyIRQBjTk7khBHAVMynZE5PvHBCbvjKFvnp1\/9zsriECppvCYNSplNiEcCiQU7khBHAVMynZE5DT70kNw49J8fvuiZZXEJFpuZbwuuLPCWfhyuK5MwigNEmJ3LCCGAq5lMyJ5qvxci+59vb2ys6\/FzWD80XiwyLADlhBDAV84mcMALJqoFHDsjAI\/vZ+dqo9B7uxo0bg39qa2uTTZs2SXt7ezLNAhU0Xww2iyU5YQQwFfOJnDACySqabwwjd\/Zzmbphmm9ycquCxZKcMAKYivlEThiBZBXNN5lRoBgZGZHu7m4ZHR0tRTdM88UCx2JJThgBTMV8IieMQLJKJ1vpfV9OuEpmNaYwjxcNDg5Ka2trii2zk9J8MZYsluSEEcBUzCdywggkq2i+yYwmdL76D5s3b4afBQYPMSbTLruvr0\/WrVsXea+Z5otRZbEkJ4wApmI+kRNGIFlF841hdPLkSVmxYoVs3749UC1YsEDWrl0rLS0tyWRrVJhj7t69O3aiF80XA8xiSU4YAUzFfCInjECyauF9z8gTe09w2NlGZc92zrvLdUNkhrT139n5JidwkoLFMonQ6b+TEzlhBDAV8ymZE83XYmTPbi6iy3XDo8e\/7bbbpKurK1iykuabnMBJChaBJEI0X4wQOZFTGgLJ2taljwUiTrgSkRdeeEFuuukmWbVqFXw\/N8u1nbdu3RoEY+7cubznm5y7kILmC2Fi54thIidyAgnEyw4df1Pm3PEkzddgMp1vI16soJOsHnjgAbnlllvk8OHDkPma896xY0cmCdGMO1GW06ZNa8ZLy\/SayAnDSU7khBGIVs2fP1\/eOu998vpVfTRf13wb8UpBHWa++uqrg46bs53rTe+3t2fni7EkJ3LCCGAq5lM8J32j0cKvPkPzxdIpP1Xce4SjHmnibGcsHiwC5IQRwFTMJ3LCCMSrzGQrVfGebxZEM9oHO9+MQHIWLwySpoKhIidywghEq+z7vb\/xxivyyp\/\/+3p32ZDtm\/KVgjTf7HKJxRJjSU7khBHAVMynaE72kPOk57fJ6Df\/DINaMlVTmi\/CmMPOCCU+v4pRIidyQglgOppvNCd7yPnsJ9bJoX\/8Fga1ZCqab8kCUrbTYRHAIkJO5IQRwFTMp3BO9pDz9NZJ8tr9\/0H27duHQS2ZiuZbsoCU7XRYBLCIkBM5YQQwFfMpnJPd9W770uXy2U\/PpflGpZQufLFy5crgzzrz+ODBg7Jz587c13hOSnEOOycROv13FgFywghgKuYTOWEEJqrse71XXXKubLvxcvG5jufa+eqzt\/r+Xn3D0JIlS0QX35g9e3bwsoW2trbgvxv18TloRTJjscRokxM5YQQwFfNpPCfbeHW4+d7Oy+SqS8+l+Yalk73a1axZs6SnpycwW10Eg+\/zxb6AZVCxCGBRICdywghgKubT25xs49V\/va\/rMum68oJA4HMTlVvnS\/PFvmRlV7EIYBEiJ3LCCGAq5tNpTq7x6n1e7XjNh+YbkU96v1fv79rDzqYL7uzslEWLFmGZmIPK56DlgCNylywCGG1yIieMAKaqej7prOahp16SgUf2jwG7t\/P9svgj7xkH0Oc6nlvnawjpEPPixYvHAVuzZk1Djdf34Qrs65uNqupFAKVIThgpciKnJALa7S7Z8pyoAevHvsfrbkvzTaJZwr\/7HLQicbJYYrTJiZwwApiqivnkmq4xXh1qVgMO+\/hcx3PvfLFUK17lc9CKpFXFIlALX3LCqJETObkE1HTXPbJfnth7YuxPcd2uvb3PdTw3841705ANr7e3tyGPHPkcNOzrm42KxRLjSE7khBHAVFXIp7944oj0PbhnHBA13Wve1yr\/bf5Fkd0uzRfIIZ1wtWXLFhkcHJTW1tZgC2PKOuFq4cKFDXvml+YLBJCLbGCQyImcYAKYsFnNN6zLNcPL\/\/YD58mXrn4vZLqGos91PPfO1zzba6ec\/Zzvnj17RBfjeOihh7CszEjlc9AyQgDtplmLAHTxKUTkhMEip2px0klT+n\/usLKhgA4vR1HzuY7TfLHvQmVVLJZY6MmJnDACmMrnfNLu9ocHfyZ\/+eTo2Ixl+6rVcK+9bIp8+ZrpqbrcMHI034h8Shp21ud8zbPAd999N5aVGal8DlpGCKDd+FwEoAvMSEROGEhyaj5O2tm+\/stfyYoH94ybNOUa7idntcrS38Xu5WKUuMJVLKew53z1BQu6zKQa7z333CObNm2S9vZ2lHcmOpovhpHFkpwwApiK+eQ\/JzXbbf90VL79z69Emq1epXa4uihG54cvqLvD5bAzljdeqGi+WJhYLMkJI4CpmE\/+cDKLXOgZLxl6LtZojdl+7JJzpevK94xbAhK74tpUPtfx3O751oayuK18DlpxlPhKQZQ1TQUjRU7l5WQmRw099aJ8f++J0Pu17lCyMVvtcqMWwsCuuDaVz3U8V\/MdGRmR7u7u4LWC7kdfLWg\/glQb+tq38jlotV91+i1ZLDFm5EROGAFMlXc+qdG+8NM35L\/vOJjY0ZquVv\/\/H3+0TW64\/N3BRTTCbF16Ptfx3Mz35MmTwTO88+bNG3uet6urS9zXC2KpmL3K56BlTyN6j3kXgSKvJc9jkRNGl5yK5WRPhjr06unHfpCPGqtOkPqjue8OTLYMRht23j7X8dzM136loE6u0md5Z8yYEbxQQSdhDQ0Nydq1a6WlpQXJhcw1PgctcxgxO2SxxGiTEzlhBDBV2nxSU92174T89Q9elLQmq2fUPe9C+cM555emo8UocbZzKCfXfHVm84EDB4KlJO1FNszKVyjsrHQ0X4xk2iKA7bX5VOSExZScaudk7slu+eFLcujYSWi42BzNdK6faJ8sy6+dEXTAZe5oMUo030hO2u3qxzXcRx99NHjPLztfNMUap2OxxNiTEzlhBKJV9pDwnz70I3nlzTNSGazu2ZjsVZdOlr5Pz\/Cuk03L0OcmKrdhZ4Vo3\/fV4WY1440bN0pbW1tDnu21A+tz0NImaD16mgpGj5zICSFgDHbvK2\/IN3a\/LD85\/mZqgzUmO33yJPnUZVPkw9PPaYouFuHnanyu47maby0wi9rG56AVxUiPQ1PBaJMTORkCuryidqB\/\/6Oj8q0fv5LqHqxNMRgWnjxJLjn\/zOAtP+ZT1slPWAZkq\/K5judmvu49Xxs57\/lmm4B57o2mgtElp2pwsoeG\/+KJw\/LsT35es7naHezci86Rz8+7cOxe7K9+9qLMnDkTg1phFc03JPg03+b4RtBUsDiSU\/Nw0s71gnPeJYM7j8g\/H3m9bnNVMroYxXUfOE9az3xnsPqTmfAURY35hOUTzdfipLOaV65cmUiut7c3mIjVqI\/PQSuSGYsARpucys\/J7lq\/vmtU\/nH\/z4KTfmLvCezkQ1RmCFiHhz\/\/sQvlvLPfFajUYOv5MJ8wej7X8YYMO2NY81X5HLR8yYzfO4sARpucGs\/JPIrzLy\/\/Qh4ePipy6lRdxmqGhU3n+qn3T5Er\/\/\/wcBGP6TCfsHzyuY7nZr4YusapfA5akdRYBDDa5JQvJzOJ6W+feVkee\/543R2rbazatb7\/PWfJkk9OH7uIRk9qYj5h+eRzHaf5YjGurIpFAAs9OdXGyXSr5555hgw+cUT2Hn2jrnus5izs4eD3tk6SP5hzvrS88zfHnoNttLkm0WI+JRE6\/Xea7685mUlWw8PDieT4YoVERKUQsAhgYSCn8ZyMqarJ\/d2Pjsr\/+fErmXSrbsd60ZQWWfihqfK+C84qZDgYy4b6VcwnjCHNF+NUKpXPQSsSJIsARrtKnMwQ8HeePyb\/+5mfBoDSrCccR9TuWD8y87flsx1tTWWqWDbx+XqUk891nMPOaJQrqquSqdQTYt852Z2qWRwiD1Od2iLyoYumBI\/efPiic8aQl30YuJ7cqGVb3\/OplmuuZRuabwy1sEeP1qxZE7zdqJEfn4NWJDcWAYx2WTkZU51y9jvlb3a\/LD88UP\/jNS4RsxLTxVPPlMsuOEs+88HzIrvVsnLColycipww1j7X8Vw7XzXeLVu2yODgoJi3F5n7wp2dnQ01YJ+DhqVlNioWAYxjUZzMYy7GVHWN4KcPvib7XzmZ2dCvXrE7YelDF\/7WBFNNWigijFxRnLColVdFTlhsfK7juZkvV7jCkqfsKhYBLEL1crIXgNh96DX5znPHgkX39VPPIhBhXWpgrpMnyfQpLfKJ9nOlY+bbKy7lPfxbLycsGv6ryAmLIc03hBPNF0uesqtYBLAIRXEyk5NeOPqG\/N0\/HZUXfvpGsMOsJiiZs7M7VX1m9d99cKpcfF5L6SYrMZ\/qyyds6+qoaL4Rseaws\/9fAhbLiTE0Q76tZ50h3\/rxMXl8z3E5+eabcvSkBGaX1ccd+tV3tL5XO9bWSWOHyLtTzepazH6YTxhRcsI40XxjOHHCFZZEZVVVoQi4Q747nj8mh46dNtE8O1R9RvXKGefIxeedOc5Qg2Fhy2DLmhu1nFcV8qkWLu425IRRpPlinEql8jloRYL0tQjYj84cOHZS\/tful+XQsWwnJdlDvm+99ZZcPPVs0dWUZr37LPnDOed7253mmV++5lOeTML2TU4YcZ\/reO4Trho9qzkqhD4HDUvLbFRlKQJ2d6pmuuP54\/LModdy6U7tzlMnJqmh6jOp7eefFTnkWxZO2UQ9v72QE8aWnDBOPtfx3MxX0blDzps3b5aOjg6Mas4qn4OWM5pxu8+zCNjd6Uuv\/VL+ateLcvDYyUIMtf3dZ8l1H5gi73\/3WeM61Foen9Ed5MmpyHjnfSxywgiTE8bJ5zqeq\/na+AYGBmTjxo3BP7W1tcmmTZukvb0dI5yDyueg5YAjcpdpioDbnX53z6tjizpkfe\/U7k6D\/z15UrC+7+9eNkXOetfbC+i7urzYpeGU1zn4sF9ywqJEThgnn+t4YebrGvGuXbvGLb6Boc5O5XPQsqOQvCctAkd+NTkYbn39l7+S+79\/RPa8\/ItCutOZU8+US85rCd5Iox81d31Jea3dafLV1q5gscTYkRM5YQQwlc91vDDztTtf9I1GJ0+elBUrVsj27duDSPT29kp\/f39oVFxtkt7noGFpmawyw74\/\/fm\/yj88f1wOHc9vQpLpTi89\/8ygOz1n0hlePzLj0qWpJOebKsiJnDACmMrnOp6r+dY71Kzb60cNN2lZSv37smXL5Oabb4aGs30OGpKWZgj4R0del2\/++GiwWlJWKyXZz5\/q4zIXTv43svjK95S+O0W41aqhqWDkyImcMAKYyuc6npv5xq1whWGdqLLN2P3ryMiIrF69WjZs2DC2jnTccXwOmn1duoLSO3\/zHXLnN\/fV\/Uyqbaofu3Ry8OaZd\/ziqJw6a2rQpTbrs6e15qO9HU0Fo0hO5IQRwFQ+1\/HczBdDh6uSzFzvIas52y9xaDbz1W72gV2j8tT+n6XuYs2bZ36n7Wz59O9MkUunngktPchiieUoOZETRgBTMZ8wTjRfjFPNKjN8vWDBAlm7dq20tLRM2Jf7WFPSfWUfgqZd7Teeflm+vms0kZ3pSrVb7bj4XJk5pSWTyUksAonoAwE5kRNGAFMxnzBOPtTxqCvxpvPVC1ATHh0dDTVg929xWt2XBs18duzYgUW6ANXuI2\/Kxh+cEP3\/UZ+2c86QD184SX7\/srPlPb91RiDTf8vjc\/jwYZk2bVoeu26qfZITFk5yIieMQLRq\/vz54\/64b9++enfZkO29Ml+9r9vX1yfr1q1LnFSVpC3bL6a\/+sGL8l+2Ph+aBNrV\/vFH2+TDF\/120M0W+eEvcIw2OZETRgBTMZ8wTmWr49hZn1blZr55vFIwzX3dpAlYZQia3sMdeGS\/DD310oSYqeHe23lZ4WbrngiLAPZ1IidywghgKuYTxqkMdRw704mqUpuvPbvZPMerq2O5z\/qav82bN08WLVokcVqDoJFBU9NVw1XjtT9quNu+dHnwT2WZWcwigH21yImcMAKYivmEcWpkHcfOMFqVufmGvUIw7PBxC2YYvbtwhj3hyvytq6srWC86Tht2\/EYETU33+3tPyI1Dz00w3TJ0uWGcWASwrxg5kRNGAFMxnzBOjajj2JklqzI3X3PIpEeDkk8tX0XRQVPjXTL03LhHhEynW5Yul+Zbe86xWGLsyImcMAKYqug6jp0VpsrNfLHDN05VZNDUeBd+9ZnguVr9lOV+LkKfxRKhxEeNMErkRE4oAUxXZB3HzghX0XxxVjUrW5c+NratD92ufaE0Xyzs5EROGAFMxXzCONF8f83JHmqeNWuW9PT0yPDwcCjFpEUwMPS1q4oImk6qsu\/vbvnCh4LVpXz6sAhg0SIncsIIYCrmE8apiDqOnUl6FTvf9MygLVzj1VnMRT+jC51ogohFAKNITuSEEcBUzCeME80X41QqVZ5Bs428W5wAAAAZLUlEQVTXp\/u7YQFiEcDSlpzICSOAqZhPGKc86zh2BrWrcut8zRB0FYed7Xu8vna8JqVYBLAvFzmRE0YAUzGfME40X4xToEr73t0Uu04lzSNo7qzm\/utmSv91M1KdV9nELAJYRMiJnDACmIr5hHHKo45jR65flVvnG3dqukzk0NBQ5BuK6r+s5D3kEbSbtj4vf\/2DF4ODL\/nke+UrCy9NPpGSK1gEsACREzlhBDAV8wnjlEcdx45cv6ph5pvm3bv1X+bEPWQdNPc+77N\/8tE8TrvwfbIIYMjJiZwwApiK+YRxyrqOY0fNRtUQ80163V82lxa\/lyyDpsPNc+54Mjig7xOsXGosAlg2khM5YQQwFfMJ45RlHceOmJ0qN\/ONm3ClL0fYtGlT4msBs7vMfDvfDY8ekDu\/dfolCfd1XSZdV16Q56kXum8WAQw3OZETRgBTMZ8wTjTfCE5Rbxsybx\/C8OajyipobtfbLMPNhjqLAJZ\/5EROGAFMxXzCOGVVx7GjZavKrfPV0wwbXjYdcWdnZ\/D6v0Z9sgrawvueGXtZgu+PFYXFgkUAy1ByIieMAKZiPmGcsqrj2NGyVeVmvnFvNWqW2c5213vVJefKthtPv4u3mT4sAlg0yYmcMAKYivmEcaL5hnBKMt9mmO3c7F2vhpVFACsC5EROGAFMxXzCONF8Qzi593ttydatW2Xnzp1eP+drd70fv3SyPPylOVi2eKZiEcACRk7khBHAVMwnjBPNN4KTDi8vX7583MzmkZER6e7ulvXr10tHRwdGOAdVvUGrQtfLzhdPPBZLjBU5kRNGAFPVW8exo+Sjyu2erzldNeDFixePO\/vNmzc31Hj1ZOoJ2hMvnJCFX30muKZmvddrAsZiiX3xyImcMAKYivmEcaqnjmNHyE+Vu\/nmd+r17bmeoF3\/1Wfley+8GpyAPlqkC2s064dFAIssOZETRgBTMZ8wTvXUcewI+alyM19zz7erq6vhXW4YvlqD1uzP9bqsWASwLx85kRNGAFMxnzBOtdZxbO\/5qnIz37jZzvleErb3WoNmr+HcbKtZhZFjEcDyiZzICSOAqZhPGKda6zi293xVuZmvnnYZZjVH4as1aGailQ41N9tqVjTf2r9sLJYYO3IiJ4wApqq1jmN7z1eVm\/nGre2slzR79mwZHByU1tbWfK8wYu+1BM2eaKXrN2vn2+wfFksswuREThgBTMV8wjjVUsexPeevys188z\/1+o5QS9AGHjkgA4+cfoFCMy4lyc639pxiscTYkRM5YQQwVS11HNtz\/iqaL8i4ahOtDBYWSyxByImcMAKYivmEcaL5\/pqTPclq1qxZ0tPTI8PDw6EUfRt2ruKQswaORQArAuREThgBTMV8wjjRfDFOpVKlDdqqbS\/Ivd\/9SaWGnGm+eMqyWGKsyImcMAKYKm0dx\/ZajCrXYedmep\/vnDueFB16rsosZw47p\/sC0lQwXuREThgBTEXzjeDULO\/zte\/3\/tf5F8mq37sYy4wmULFYYkEkJ3LCCGAq5hPGieYbwinplYJDQ0PevNXInuXc7MtJuqFkEcCKADmRE0YAUzGfME403xrM16f3+VZtYQ07nCwCWBEgJ3LCCGAq5hPGieYbwqlZ3udrDzk3+xuMwtKdRQArAuREThgBTMV8wjjRfCM4NcP7fG3zrcrCGux8sS8+OZFTegLYFjRfjBPNN4aT7+\/zNUPOeolVu9+r18wigBUBciInjACmYj5hnGi+GKdSqdCgVfURIxMsFgEsbcmJnDACmIr5hHFC6zi2t2JVuT7nW+ylpDsaEjR7yLkqL1JwKbIIYHlFTuSEEcBUzCeME1LHsT0Vr6L5xjCv4osUaL61fQlZLDFu5EROGAFMRfPFOJVKhQTNvt97\/K5rSnX+RZ0MiyVGmpzICSOAqZhPGCekjmN7Kl7FzjeGedXv9yoaFgHsS0lO5IQRwFTMJ4wTzRfjVCpVUtDs+739182U\/utmlOr8izoZFgGMNDmRE0YAUzGfME5JdRzbS2NUuXa+IyMj0t3dLaOjoxOuruyvFLRfIVjF53tNwFgEsC8mOZETRgBTMZ8wTjTfEE5mhau2tjbp7+\/HSBaoSgra0r\/5F\/nLJ0\/\/aKD5ziwwMn4eisUSixs5kRNGAFMl1XFsL41R5db5xr1YoTGXOv6oSUGr8nrONikWSyxbyYmcMAKYivmEcUqq49heGqPKzXxN59vV1SUdHR2NubqYo8YFrerrOdN806criyXGjJzICSOAqWi+EZx0aclGv70oKoRxQbPv91Z5spWyY7HEigA5kRNGAFMxnzBONN8QTmbYeXh4OJRimSdcDT31ktw49Fxw3lW+30vzxQoAOZETTgBT0nwxTjRfjFOuKjPMvX379uA4vb29sRO94oLGxTXeDhWLAJa25EROGAFMxXzCONF8MU65qnR4Wz86s9p03Z2dnbJo0aLQ48YFjYtr0HzTJiuLJUaMnMgJI4CpaL4xnLZu3SorV64MFJs3b5aDBw\/Kzp07Ze3atdLS0oIRrkFlm3HY5nFBa136WLDJVZecK9tuvLyGozfPJiyWWCzJiZwwApiK+YRxovlGcFID1AU2+vr6ZMmSJUFXqvd6V6xYIXk+\/4s85hQVNC6uMT6YLAJYESAncsIIYCrmE8aJ5hvCyTbAWbNmSU9PT2C++thRnrOg1fA3btwoCxYsiO2uNWjms2PHjrH\/vf251+VPv\/NK8N\/\/84YL5IoLJ2FZ0KSqw4cPy7Rp05r06rK7LHLCWJITOWEEolXz588f98d9+\/bVu8uGbJ\/bc76NMl9D0XTdUcPbUb+YONmKnW8t30R2Khg1ciInjACmYucbwUnv9+r9XXvY2XTBcZOhMOzxKl1XWo+7bt06aW9vnyBOMt\/prZPk2T\/5aBan4vU+WCyx8JETOWEEMBXzCeNE843hpEPMixcvHqdYs2ZN5CxkDHmyKmloOyxoXNlqIlcWgeRcUwU5kRNGAFMxnzBONF+MU64qe3Yz8lKHJPOt+spWJlgsAljakhM5YQQwFfMJ40TzxTjlqnIX2UAmXLk36u2ZzjrkrEPPVf+wCGAZQE7khBHAVMwnjBPNN4aT\/Zyvkenzvo1+2UJY0AYeOSADj+wPTrPqy0qy88W+\/ORETukIYGqaL8aJ5hvBSY13y5YtMjg4KK2trYEKWX0Kw16fKixofI3gRKYsAliekRM5YQQwFfMJ40TzDeEUt9BF0mQoDHt9qrCgcVlJmm+tWcViiZEjJ3LCCGAqmm8TmC9nOocnO4slVgTIiZwwApiK+YRxovlGcNIOd\/ny5bJp06axZ23LOuzMd\/jSfLGvOzmRUz0EsG1pvhgnmm9M5xv1Pl97E13v+aGHHsJoZ6Ryg2Z3vpxs9TZkFgEs4ciJnDACmIr5hHGi+WKcSqVyg8aZzuzo6klQFkuMHjmRE0YAU9F8MU6lUrlB45rONN96EpSmgtEjJ3LCCGAqmm8Mp7DnfItYXjIpdFHmyzWdx5NjsUzKpNN\/JydywghgKuYTxonmG8HJp+d8W5c+FlzFVZecK9tuvByLfAVULAJYkMmJnDACmIr5hHGi+YZw8uk5X3uyFdd0ZueLfe3JiZxqIYBtQ\/PFONF8PTdfPmYUnegsAlgRICdywghgKuYTxonm6\/mwM2c603yxrzo5kVO9BLDtab4YJ5pvDCcfJlwNPfWS3Dj0XHAVfJsRh1Oxrz05kVMtBLBtaL4YJ5ovxqlUKjtofKECO7p6k5PFEiNITuSEEcBUNF+MU6lUNF8sHCyW5IQRwFTMJ3LCCGAqmi\/GqVQqO2h8zIidb73JSVPBCJITOWEEMBXNF+NUKpUJGh8zig8LiyWWtuREThgBTMV8wjjRfDFOpVKFme99XZdJ15UXlOo8G30yLAJYBMiJnDACmIr5hHGi+WKcSqUyQbNnOvNtRhNDxCKApS05kRNGAFMxnzBONF+MU6lUJmh8xpfDzlkkJoslRpGcyAkjgKlovhinUqlM0JYMPS+bn3oxOLfjd11TqnMsw8mwWGJRICdywghgKuYTxonmi3EqlcoEjc\/4svPNIjFZLDGK5EROGAFMRfPFOJVKZYI2544nRWc8821G4eFhscTSlpzICSOAqZhPGCeaL8apVCoN2nd\/+H9FzVc\/NF+abz0JymKJ0SMncsIIYCqaL8apVCrXfPkqQZpvPQlKU8HokRM5YQQwFc0X41QqlQbt699+WhZ+9ZngvGi+NN96EpSmgtEjJ3LCCGAqmi\/GqVQq13z5jC\/Nt54Epalg9MiJnDACmIrmi3EqlUqD1vs\/\/kEGHtkfnBfNl+ZbT4LSVDB65EROGAFMRfPFOJVKpUH7zJ1\/L7rClX74Hl+abz0JSlPB6JETOWEEMBXNF+NUKpUG7YPL\/lae2HtCprdOCsyXn4kEWCyxrCAncsIIYCrmE8aJ5otxKpWK5ouFg0WAnDACmIr5RE4YAUxF88U4lUqlQTvxB4PBOfEZ3+jQsFhiaUtO5IQRwFTMJ4wTzRfjVCqVbb76GkF9nSA\/HHauNQdYLDFy5EROGAFMRfPFOJVKNeODH5HXPj0QnBOf8WXnW29y0lQwguREThgBTEXzxTiVSjX9I5+R16\/qC85Ju17tfvlh51trDtBUMHLkRE4YAUxF88U4lUo17ZP\/Ud6Y+\/ngnPiMLzvfepOTpoIRJCdywghgKpovxqlUKtt8+Ywvzbfe5KSpYATJiZwwApiK5otxKpXqgj+6Q\/51+seCczp+1zWlOrcynQyLJRYNciInjACmYj5hnGi+GKdSqc7\/7J\/LW+e9jwtsJESFRQBLW3IiJ4wApmI+YZxovhinUqlovlg4WATICSOAqZhP5IQRwFQ0X4xTqVStSx8LzocLbMSHhcUSS1tyIieMAKZiPmGcaL4Yp1KpaL5YOFgEyAkjgKmYT+SEEcBUNF+MU6lUxny5wAY73ywSk6aCUSQncsIIYCqaL8apVCqaLxYOFktywghgKuYTOWEEMBXNF+NUKpUxXy6wwc43i8SkqWAUyYmcMAKYiuaLcUqtOn78uPT09Mjw8HCw7YIFC2Tt2rXS0tIyYV8nT56UFStWyPbt28f+1tvbK\/39\/aHHpfli4WCxJCeMAKZiPpETRgBT0XwxTqlUxkznzZsnixYtEvPfbW1toYaqRr1s2TK5+eabpb29PfFYxny5uhU738RkAQQ0FQCSiJATOWEEMBXNF+NUt2rr1q2yc+fO0O53ZGREVq9eLRs2bJDW1tbEYxnz5epWNN\/EZAEENBUAEs0Xg0ROMCeaL4yqPmGc+e7atUsGBgZkcHAQNt\/prZNEO19+ognQVLDsICdywghgKuYTxonmi3GqS2Xu\/3Z2dgbD0O5HjXnlypVj\/zx79uxYI9bO9zfeeEXO+Xa\/7Nixo65za+aNDx8+LNOmTWvmS8zk2sgJw0hO5IQRiFbNnz9\/3B\/37dtX7y4bsv07Tp06daohR05xUHO\/VzeJmnClXe\/o6OjY393\/dg+n5svVrZKDwF\/gyYxUQU7khBHAVMwnjBM7X4xTTSrEeMN2rPeA+\/r6ZN26daETsNR8P\/X+VvnGf5pd03lVZSMWASzS5EROGAFMxXzCONF8MU6pVUkznON2mDQBS82Xq1slh8Tn5E6+uuwU5ISxJCdywghgKp\/zqdTDzklDxyY8aR9L0u1ovs2f3NgVZqPyuQhkQwDbCzmRE0YAU\/mcT6U1X3eBDRMKM5FKF9rQRTW6urqko6Nj7Dlgs8hG3IIcxnzPfPp+edeh72NRpooESIAESKB0BDjhqnQh4QmRAAmQAAmQQDkJlLbzLScunhUJkAAJkAAJ1E+A5ls\/Q+6BBEiABEiABFIRoPmmwkUxCZAACZAACdRPgOZbP0PugQRIgARIgARSEaD5psJFMQmQAAmQAAnUT4DmWz9D7oEESIAESIAEUhGonPnqwh0bN24MIG3evDl4RrjKH10JrLu7O1gXO+nZaJudvld506ZN0LuTm4FvGk7met3FX5qBA3INaVi5z\/NX6TuZhpOtrdp3Ly7nzHfMrPeA5GdZNJUyX\/u1g3v27En1CsKyBCzL87DNYeHChcGiJfPmzYt8a5T9LmV9i9SWLVvgVzhmed5F7ysNJ\/vczJu21qxZE8q06Oso4nhpWLnLxyatx17E+Rd1jDSczA+U\/v7+oFmo0ncPMV5dWMnHH22VMl\/t3PSjSezzL6asCoRb7PTHydDQUOSbo+zjVqlQ1sJJC+ayZcvkxIkTEvUazKziWKb9pGGVtP56ma4r63NJy8l+SUyVvntR3M1IwBVXXCGHDh0Karpvo5iVMd+o9Z+jOr2sv2xl3J89EtDa2iruf8edc5UKQC2c9IfelVdeKQ8\/\/HDkaEIZc6Lec0rDSjs4ezSl3mP7tH0aTmGdb1W5mRgfOXIk+J+6zHBPTw\/Nt8zJH9bpaoGcMWNGZYYE3fi4nW6aTgR96UWZcwI9t7SclOMDDzwgS5culdtuu61y5muPnsTllJrvgQMHgjBUbR5G2pwy9UuHWHt7ewOz4UfE\/WHiE5PKdb72jXma7\/hhZtR8tWjec889lZlwlaZQapG888475XOf+5xMmzYt9j66T4UCPdc0rMw9cXO\/Trddvnx5JfIqDSczxLp+\/fpgaDXNCBUaN191NF8PIsdh54lBSjP0ZbaumvHqdafhpNrHH3983LyCKt3aSMPKHXau0uxwcsrGNGi+2XDMfS92p8sJVyJup5s04aqqsyzTcLIfx7ITuipDhWlYuflWpe9kGk5V\/pGSZAo03yRCJfk7HzUaH4g0jztUaUjQTdc0nOxtq9TJmetOw8otnFUaTk3DKWzYuSrD80nWQfNNIlSiv3ORjfHBiHvQ30yI0ckdUR2dj8\/X1ZKOKKeqm69efxpW9iIbVVs8Ig0n\/WGyePHiIL2qxinu+0rzraWacRsSIAESIAESqCiBysx2rmh8edkkQAIkQAIlJEDzLWFQeEokQAIkQALNTYDm29zx5dWRAAmQAAmUkADNt4RB4SmRAAmQAAk0NwGab3PHl1dHAiRAAiRQQgI03xIGhaeUP4G9e\/fK5MmTRV8ogXz0kYZXX31VLrnkEkRek8Y8zjV79uxUr2r05SUX5vqKelSm6OPVFHRuVFkCNN\/Khr66F552MYciniWsx0Dr2bbILLBf6VnUcX1hUxQPHqc8BGi+5YkFz6QgAmU037TnZKPyxWBovgUlOA\/jBQGarxdh4kmmJWCvnKTbmqHcPXv2jK0UpP9uVuhyV\/AyQ6NTpkwJ3hc6PDwcnIJZo9l+xZu9\/7hhbPsY9tCrebuPucY1a9aEvuYySmfMd+HChXL77bcHu3GHdu0VktzjKKtly5bJJz7xiWB7w+rYsWPS3d0to6OjwSa33nqrbNu2TdatWyft7e3Bv0VdU1i8XPPV\/\/75z38e\/J++Ks\/mG7Z92Pt\/k94J7MsPk7T5Tb3\/BGi+\/seQV+AQCFtT2S78bpcZtXC97nbt2rWi+7Nf2G2MvbOzc8wk4146Yc7H7E9fAO6+HSqp83X19nq\/+gNBTfKKK64Iztfsf8uWLcG9YzXRvr6+caZp78\/8wJg+ffrY9u41mv8+evRo8Mo\/87pENXnzbtmk9b\/DzFff42t+bIRxtUNL8+VXvZkI0HybKZq8loBAUhFPMjrdh13oXfMNe\/tT3EsUwrovVx93TkkvaHAX3tfzT+r47L8b83V\/TOzcuXPMjHWftrnqf69evVo2bNgwbtJa3NBymPlqV21+MJhjqE5\/NLijCDRffsGbiQDNt5miyWsZI2AP0bqzh6OMzh2aXbBgQWjn6w7\/2tjDhoyjjme\/uCLOfJMmfIUZbdi\/uUPx7tC6drD6snYznKz\/33S1rqFrN20W+nfTLur1iWHmG3cMM7Rt9k\/z5Re8mQjQfJspmryWCQRsw7Hv+9rdlTFT9z6s6fzczle3dTu2OPRRxho3FG7vr17z1X2Ze7fmx0FY55vGfJ9++mkxw9ro41o0X35BSeBtAjRfZkMlCNgGZjo7HdrU+6MrVqyQefPmjZvkZBusa75x93fDYBYx7Oze07WPqUYZN4Rshp1t8w3rMu1hZ+18075TNo9h56QfQknD75VIfl5kKQnQfEsZFp5UPQTC7vna3ac9ASls4pDphM2ws56LbdBm\/zpEa4Zlw+67mmvIasKV3Wna94Hnzp07YUKVa772tuZc9fx08lSY+aITrnQf5p5t0r32eidcmdsCZoa6uQ57opmbNzTfer5J3DZPAjTfPOly3w0jYL+oXE\/CHlK2HxPSYdhrr7123ONEarrXX3+9rFq1KjAnvffoGrLphs0jSHoMYwpRFx33WA46CWzlypVjuw8bQjb3SV3TcY+9fv364DEhnWRlrt\/ufPUgLkP3USP3cSvdJuoxKTPaoP\/f\/GAJe9TI3j7MOO377RqnOXPmyLPPPhv8AHB\/JJlrcEcFGpaUPDAJWARovkwHEiABiICaYdgMZ2jjXz8T7Jqv\/d\/oftLo2PmmoUVtkQRovkXS5rFIwBMC7n1t0+Xaz\/WmvRRkwlXafSbpab5JhPj3RhGg+TaKPI9LAiUn4K76FfUIEXoZ7osOHnzwwWBT+3EmdF+Iji9WQChR0ygC\/w8+T0yM23VspQAAAABJRU5ErkJggg==","height":289,"width":479}}
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
