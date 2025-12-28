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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARMAAACmCAYAAAD52WvKAAAAAXNSR0IArs4c6QAAH41JREFUeF7tXQ2QV9V1P9aYcSFU+WxcV7Kou2pqqmK0hKJgrcVkBjTMtHx0EgIMQeq6aQqz8uUgnQALZW0K5oNYZLAzu6tMaQQzrSWkUBSpUYQ2BQO6LBSRCosrgpAMcTvn4fnn7v3f+9697933cZfzZpyV3ftx7u+e87vnnvt1SVdXVxfwxwgwAoxAQgQuYTJJiCBnZwQYgQABJhNWBEaAEXCCAJOJExi5EEaAEWAysdSBnTt3wqRJk6C5uRmGDRtmmTs8uVz22bNnYeXKlTB9+nTo16+fk7qwzDlz5gRlNTY2QkVFBWC9hw4dgvHjxzupQ1fIs88+Czt27CjVG1bZk08+CaNHj4aamppImWzSRhb2SQLCadOmTVBZWQlr1641kkWFr2mdLtOdPHkSpk2bBo8++qhzPdXJyWTisgcdl7Vs2bLA0NesWZMamRw5cgSmTJkCjzzySKpkQso9YcKEyHqQdFatWmVkwDZpbbrnwIEDAS5jxowJDNL0KwqZoLyoP0ePHjUib9P2haUrJJlQRyIQ+JEXQB116tSp4Pfbtm0rGzVodMe\/33LLLSVDpN8\/9thjwe+w7KVLl2oVWyeD6D1g+TjKkzyYB0ewqqqq4Pc4quE3Y8aMQCGpTDJc2RMR\/42ewty5c4P88sioUliZeKIwxHIbGhqgrq4O9uzZ001ONFBd3VjP6tWrA5lGjRoFW7duLRm9OJpjgSK+stETuVDdlFbsP+r72traYJSV5VSlRW9RlB\/JgDww2RB0Msi\/V5Uht5X6WKWjoh6SkSOGOh3FsvDvVGYY5rKsosecphetIpXCkYlscKISkpG+\/vrrgQL3798\/ULLBgwcHCiOOsmPHju3mzqMi4vRE7CDdqC+PoqKh7t+\/vzTNITIhecglF0cEqhc7GeUVvYAwMkGjCPNMEJfW1taAGPFDHDCPirRUGGIeGTOc5iD+S5YsgaamplK5hC+1BQ2f8BXbrsOJ2iKOkmLazZs3d\/NEZOLBtNXV1QHxE1GQ0chpRUyJhAgX0QCoj+lvcl9EeSa6PpZ1AuuU+7ylpaUb9uT9kAyko5iXfqfCnOyB+nLjxo3dcLTxBnukZ6IbwbDT6+vry+b7Yvpdu3aVKSUZHJEAjYBh7jF24OzZs5Vutsozoc7E+ENYB9p4JlFkQmWtWLEi0AMxjmODoW6aoxrdMX6D3hbFD8R6iNjJOEUcZGJHnGi0lUdtbIuqb3QjsIp4xEFC5+qr4lNiLIlwUU1zwvqYPJPDhw8riZ6Mltoveq4qTwLT6TCXiUrUCewHmTBdEEZYGYXzTOTRWAQkikw2bNgQuIfiR1OEjo6OUIMT80QRDRkujUIimciEIZbrkkxIabF9NIJRbMUGQ5lMSKHRiBYuXAiLFi0KykcvBslENFQRJ1JsmppSu3GUxQCy6EEimcjTMJFUVJ4UGhR6I2HEKU8vSQYTwpKnjmFkEtbHcjn4b9FrJJIWcdF5Ryi\/3JciNqTTsoHTgEm2Q54l4p7mVzgysRlVEZwwz0QETmZ9U8KQV2xMPROVa+2STMQRfODAgaUpjmpkDyNkmUxE5UV8xdHaxjMRsQ8LSoqxB3LvVSSlizNFeSY643Hhmaj6OIxM5MFQJpqknonc1oveMzGJmdAoRXNim5iJbq4tdoTcCarRAMtReSbyaIKjB82Z77vvvm6jFLm6JJOsTFGrOeLoLgbeTDAkb0MmE1VbKQApxkyoLcePHy9Ne6JiJuTVyCQVJoM4fSJjpP6nYKu48pNlzITaI\/axPKWTCUOOFWGgm0hURSZizETGnGMmBn6WaCTiSobI+n369AncXtmFjVrNMSETFNFmNUec5uD\/6yL95DXQSglF7XVkIrZFta9Fnp+Le1FMMMSpC3608oSkIa7wIPbo9eAnTqHSWM0RV0xE2dFlx48ww\/5GAiNPRU4rBmkxn81qjoqQdUvDUas5pBMymcj9gvjKAW65ry\/a1RwECxVy+fLlZZt85A5QzWXDuKZIa\/gGnNjjkth6TipvL8tNVD2hA5Jgjl6m6SZBF1g5jZmQsctLpSQoTh9mzZoF8+bNM9pNKDeQycRFl8cvQx4MsCSbncBZK3f8lhYnZ1zMvd8Bi8qCLiiSiYowxD0MrraHF6fbWRJG4OJGoOSZyOv4YbCopidIFOvWrYOZM2fCggULlGQi7l3A8sN2oF7c3cKtZwT8Q6AbmeCeAtxbEOY1IOnI6dAVW7x4MUyePDnY5WkylcnDDfOve1hiRsAfBJzETOSVD2x+1ElLmgsOHz682\/mYa6+91h\/0WFJGoOAIbNmyBYYMGZKJlGXTHKw1ySnVsCArTnPwozMgqlUfJJO2trZMGp+kEl\/kxDb6IqsvcjKmasvp5pnIkeMo70JVpEwmIoGEnSqlsnxRKF\/kZMVPMmTo8\/rS\/1nKGTrNySNgmmXjk6jZwYMHM3Mfk8iJeX2R1Rc5fcI0S3syjpmoAq9JlVyVP8vGJ5GfFT8Jeuq8jKl7TLO0Jy2ZyEvFcaY8caDJsvFx5KM8rPhJ0GMycY+eusQs7Um7zyQr8pAhyLLxSTqUySQJekwm7tErEJlgcBT\/y3tnKpOJezXzhfh8kZNjJhGrOaYxEdN0cU2CySQucvp8vhipL3IymRiQiXhpb5hK2572tTEPJhMbtMzS+mKkvsjJZBJBJmZqmX4qJhP3GPtipL7IyWTCZOLUSlnxncIZFMaYusc0y8HZeJ+J+2bmH31O0iZW\/CTo8WqOe\/Tytycmk5i9ymQSE7iQbIype0zZM\/HgoB8rvnvFZ0zdY8pkwmTiVKt8MVJf5PQpvpM7mYine+nGct3taU613qPj8qz4rnueA7DuEc32+omymIn4Uty4ceOCqxjnz58P+I5pFjddZ8mkSTqPySQJehyAdY9eAQOw4g5XfFyIyARJxuRax6QgMZkkRbA8vy\/E54ucPM1R66hyNYcee546dSqsX78+uCS6rq6u2xOU7lX+QolMJu6R9cVIfZGTycSCTDCp+Joc\/jurm+SZTJhM3CPgvkRfiC9Le+J9JjH1zBdl8mkUZUxjKmNINiYTXhp2qlW+GKkvcvpC0C+91QkPrtgMJ374Z071SVdYmWcS9RhX2pcmZcmkSRBmxU+CHq\/muEevvMSWnx+Dh1v2wckn7smiOlBOc\/Ai6fb2dsBHpumj340cORJaWlqgsbERKioqYguJQV78xDo4ABsbztCMvhCfL3L64pkse7Edlr14MD8y0V1+RL+vr6+HlStXRr78F6bdFNydMWMGk0k6\/NGtVF+M1Bc5mUzUSqvdtIaPj69duxZqamqAXuy7\/fbb4YEHHoDnn38+tmdCpITlnjlzhsmEyaSEAJOJW2UoxDQHmyQvDTc3N0Ntba3RO8JhkOD0BqdKhw4dKptK8TTHrTJRab4YqS9y+uKZYLwECSXXmEk6Kn2BoLZt2xZ4I6q4DJEJ\/sQ3Uov8HTlyBKqqqoosYkk2X2T1RU4Etuiy3nvvvXB6RAOcH3BDzyQT9EpWr17dzQDluAmv5rjnJ19GfF\/k9MEzOXzyHNz6nVcCZcrVM5GfBSX1dnmRdJhn4sPD5az4Fy\/p+UAmuMdk7PffyJdMxIfHN2zYEMQ3hg0bBuhVVFdXw\/jx451oEZOJExiNCvGF+HyR0wcyefAHu+E\/DryfP5nQ6eDNmzeXgqRpv5dDVsHTHCN+sErki5H6ImfRyUT0Sj514pfw3jMPWelL3MTapeHhw4fD0KFDoaGhAZYvXw67du2C1tZWWLNmTaqv\/jGZxO1KfT5fjNQXOYtMJmKsZHC\/y6Gz9VvQ\/otX3SuVokTlDljRC0HvZO7cuUFWXB7GKU+aH5OJe3R9MVJf5CwqmXx3y2H4m5+8XVIgDLxmaU98ajim7bLixwQuJBtjGg\/TXYdPwZ989\/VSZvRINv7lbYA\/cyWTqO30Cxcu5GkOPxgVT+sjcjGZmMOK05m6ln3w0tud3TKJRIJ\/yIVMok4Lo2B4uXTSA35RcGXZ+ChZwv7Oip8EPXVexjQcUx2BYC6ZRKikLO1JeQVBFne96mDLsvFJzIEVPwl6TCYm6CF5PPHTQ\/DMzqPa5DoSKQSZmDQyzTRMJu7R9YX4fJEzjQAsEkfra8fgpQPvl01dRI1A8sCPYiJR2pKlPZU8E5NpjssdsOyZRKmBu7\/7YqS+yJmETJA0+ve+DCb+w3\/B4ffPAf476kMC+fa9n4N7bugXTGdsvlzIxEbANNNm2fgk7bgYFD8JPnHy9hRMkSDQ6Ff+7DD8dF9HqKch44T5Bve9HJ6ceJM1cagwz9KeeGk4jtbzak5M1MKz+UQmL+85AHf8\/nXQsGE\/tB0\/a0UYiAJ5GKM\/PwAeHnWNE+IoJJmIz4OSgFms5GBdWTJpEovwSfF9kbUoctLU48TpX8MzO9+FtuMfGU9JdJ7GF6uvgG98qTI10ihC2CD0eVDxflZ6mIuXhi90W1EU34QQfZE1CzmJKD769W\/gR9uPwFvvxScKwp6mJrW\/1xvq\/3hw8Gvb2IZJP8ZJk+XgbLw0zAf9undlFoofR3lUeXyRNamcRBT\/trcDNu55L4DCNMgZhjWRxXWDesFDd1dBxWWXwm8+eBeGDBniqotSKydXMsFWoReyadOmsjtgcaoj3ybvGoUsG59E9qSKn6Ru27y+yKqSkwji\/Mdd8Nxrx2DHJzs+XZCE6EFg0HPy8Eq443NXGHkWvmCapT1pA7DyBUn8PCh7JrYkpksvLoduefMkvHboA\/jfk+eg7fhpOHrqvJNqaJqBJHFNv8th2h9dDQM+82kjojARgMmkHCVezTHRHEUaX5Qpq\/gOEcTHXV3Q\/Oox2HmwE6DLzTRDhF8miRl3VcGVvS5zRhKm6uBL\/xfCMzEF1XW6LBufRHZflCkumYjewyttnbD9rU443HHWSQxChTuSxPnz5+HagZ+BL1T1gUVjroOjnb\/KnCRMdcKX\/s\/SnrTPgw4ePDj1Q32qjsuy8aaKo0rnizIRmVx6xVWlZvznwQ9g24H3UyeHICbR98KOzRE1feGrtw6CmkG9gl2fqtUO3zDlAGx3y1BOc1T7TFSv7yUxRl1eJpNoVMlrQIPc\/95H0Pzqu7Dr0Kkgo6vApM57IIKo7l8BX71tEFw3sJcz74HJJLrvbVNkaU\/GMRN88wZXefjaxgvd6VLxaaQ+1HEO\/nXvCfifo6eh\/cTZTMkBg5R49uPO6isCr0HnPdgqs016l5ja1BsnrS+y5k4mKs+ksrKytFSsAl98AVC3W1YuV3VwMMvGx1EiyqNTJjJC\/Pny252AUwrcQZmV10Cew42f7Q1f+cJAGFXbF3Dr9zXXpLdlOwmOYl5fDNT1YOIKP1U5WdqTNmaCgpl6IeLzGPjK3Zw5cwAvpJafxRDT4VvDeTfepBNpStEFAD978yT88xv\/F2RzuYwZNaX4XP8K+PxVvWH6XVWxgpK+GKkvcjKZqC3HeJpjYniYhryPiRMnll0+jQ+gL1myBJqamrRXP2bJpNQmJIynX34H8C7NNGIO4nImXAJwS1UfmH1fNXxw9sKeirS3XvtipL7IyWSSAZnQVEc3zTHZCJcmmSBpvNP5K1j6L23WpzxF+MRlzCEDe8E3R1wNfS7\/VCbEYErqPk4fmEzi9G54njTtSa7ZuWeCFSBp7NixI3RpmS5jwu354vMZ2Hj8XD1cjjsqf\/RqJ2zadzqypyp\/9wIhfPHqy+G+mt5Q3ffCZij6vVhA0R+u9lFWxjRSRY0T4MPl9GX13G4qZGKy8kPTITm24oJJKc6Bb62qbrKig1t\/9+c3wGWX\/k6saQaPosZ6bZyQMTWGyjihC3syrczJqWH5RDEuIeMnHwpEjwU\/DMxi\/IReCxSDsUkbj08j1rXuKyORqIt3TQGjdKz4tohFp2dMozGyTZHUnmzqs7oDNuyCJN3SsEgg8tKw6vBg3MajB\/KTX5yA+T8+UGo\/EsiqCTfBXddfaYOJUVpWfCOYrBIxplZwGSWOa09GhUuJjD2TOIXHyROn8Ugk4pQGSWTpgzXw5ZsHxBHBKA8rvhFMVokYUyu4jBLHsSejghWJUomZxBUG89k2XkUkps8AJJGTFT8Jeuq8jKl7TG3tKYkE3aY5+PhWfX09zJ49G\/bs2VNWbtGeupBfOHMdFwkDlhU\/idoxmbhHT11iLmSSVeOi6rFp\/LOvHYOZzfuCIpFIdi\/4UlTxzv7OZOIMylJBjKl7TG3sKWnt3k5z0Cu59Tuv5EIkWCkrflLVK8\/PmLrHNFcyCXvZryjTHDlOgh5J2lvS5W5mxXev+Iype0xzJRNdc3DvyMiRI8vO27huvknj8SHnv3rul0HV675xM4z5g4GuxYgsjxU\/EiLrBIypNWSRGUzsKbIQwwTG05wiPXXR76\/\/PbfpDeHKim+oYRbJGFMLsAyTFpJMTLbIG7YvNFlU43Fj2tee\/u+gDFwCHpHChjSTdrDim6Bkl4YxtcPLJHWUPZmUYZpGe5+Jamm4ubk512lO3kFXEVRWfFMVM0\/HmJpjZZoyVzIxFTKtdGGNF8kkT68E286K714DGFP3mOZOJvIhPDxf09raanzzWhJIwho\/9ntvBPeQZL2nRNUeVvwkvazOy5i6xzRXMtHdlGZyR4kLKHSNF72SR+4ZHLyrkufHiu8efcbUPaa5kolu1Sbv1ZxvPfcm\/OPOd3MPvFJ3s+K7V3zG1D2muZIJNge9kFWrVpVuo6eNbHgjWl4PlxdhOZgDsO6VnTFNF9PcyQSbh3GTKVOmwNGjR4PW5vlwOV54hFcM4PfjmbfC3TV90+0Bg9J5FDUAyTIJY2oJmEHyQpCJgZypJFE1Hl+sq2t9szBTHBSEFd999zOm7jHNlUyyio3oYFM1vkirOBwzca\/wjGl6mOZKJtgsPIdTXV1d9ohWek3+bcly48VVnJl3XwOLH7w+CzEi6+BRNBIi6wSMqTVkkRlyJZOinRpu+fkxeLjlwp0leW9UE3uOFT9Sj60TMKbWkEVmyJVMIqVLOYHceJriYLUnn7gn5drNi2fFN8fKNCVjaoqUeboeSyby7fSqsz5y44u2JMzze3NFtk3JZGKLWHT6XMiEAq9p3gGL+1fa29uDvSq6U8hi48V4yaOjh8Cjo6uj0csoBSu+e6AZU\/eY5kIm7psRXiKSSUtLS9kTomLjxf0lRYqXYMtY8d1rDGPqHtPcySTNg37iVCdqmvPA93fD9rfeDxDO42rGsK5lxXev+Iype0xzJZOsDvqZPFz+zQ3H4PV3zgUPh2+aXOUe6QQl8iPbCcDTZGVM3WFaiIfLszroF\/VwuRgvGXHdlbDx4dvcIe2gJB5FHYAoFcGYusc0V88Em5PWQT+bh8uLHHzlmIl7pWdM08E0dzLBZqVx0M9mabiom9Woy3kUda\/8jKl7TAtBJu6bZVYiNR53vSKhFDH4yqOoWV\/apmIysUUsOj2TSVsbFPFwn9h1rPjRimybgjG1RSw6\/UVPJltf21t6+rOIwVf2TKKVOE4KJpM4qIXnYTIRyOTvx98IX\/vDq9yjnLBEVvyEACqyM6buMWUyEcjkexNvgol3fNY9yglLZMVPCCCTiXsAFSXmTia41X3SpEllomX1cPniZ3cU8toBjpmkq\/9M0O7xzZVMdDtT3TdTXSI2\/uZZ\/xS8j4Nfka4dYDJJVwuYTNzjmzuZLFq0CBYuXAj9+vVz37qIEkUyKcJjWzpxWfHdqwZj6h7TXMkEmyNeFeC+edHR584H1wSJirqSg7Kx4rvXDMbUPaa5kkne1zZW33wnnPrTZQGqGHjFAGwRP1Z8973CmLrHNFcycd8cuxIH3\/llOD2iIchUtDtMOGZi15e2qZlMbBGLTs9kwmQSrSUWKXwxUl\/k9GmamzuZiAfyxowZAw0NDbBgwQKYN28e1NTUWKixfdLKr3wbzt04lj0Te+i0OXwxUl\/kZDJRq9olXV1dXeKfiEgqKyth3LhxsG7dOpg\/fz5s3LgRduzYUXbNokOdD4oa9PUfwPkBNwb\/X9RlYZ+UySdZmUxcWxNArp6JeDlSR0dHiUyQZLJYMh709R\/C+QE3QJGXhX0yUJ9kZTLpYWSCzcEX\/fDB8qlTp8L69eth5syZUFdXB8OGDQtulk\/zG\/DQevi414BCLwv7ZKA+ycpk4t6ycvVMqDnylvqlS5dm8lwovZMzqrYvbHjoVvfoOiqRFd8RkEIxjKl7TAtBJu6bZVYikUnR3smRpWfFN+tPm1SMqQ1aZmmZTAr4tAWTiZnyJknFZJIEPXXe3MlEvqsVxcQl4sbGRqioqHDfYqFE8kyKvGHNpziET7Iymbg3rVzJRFwapmAr\/Q6bqiMUPM8zd+7cAA3dVQUySanSEZkU7dEt9kzcKzpjmj6muZJJnHdz8Cb7JUuWQFNTU3DSmFaDZOLBsmfNmhW6+Y3IpMh7THwa7X2SlT0T9+SSK5lgc1TvACNBVFdXG63o6N4RlklHBR2SSdH3mPhkoD7JymTSw8gk7NQwNTXqxjUd8YhTISxLtdzMZOJeoXwxUl\/k9Imgc\/dMkqiz6V0ouhvdkExGXH9lcGK4yB8rvvveYUzdY+otmdhMhXRvDSOZfPrwy\/DK3\/6Fe2QdlsiPbDsE85OiGFN3mBbi4XJsTpylYSSSkSNHBlvudZ\/JW8NIJkXfsIbty5Lxk6qYL7L6IqdP\/Z8lpqGnhsVzOLoVGgrYyrfZ074UPG2M3\/jx48tIShcz6bXr6cA74Y8RYASSI9DW1pa8EIMSysgkztKwQT2chBFgBHo4AmVkgu1FL2TTpk2wdu3a4DIkXNKdMmVKsAs27VPDPRxvbh4j0GMRUJIJttZkGbfHosINYwQYAWsEtGRiXRJnYAQYgYsagcKQiegJNTc3h64KZdlj4r0uujtdxDR43SVND4smJ8mT96uNJpjSdHv16tWB2HnohImc4ibPvPo+TM9MjrC40tNCkIm4zX7\/\/v3Q0tKSyQnlKBDFjsC04vkj0TDF80ZIiq2trbBmzZrMXkQ0kVNsK8bE0EjzMFBTWcXNj6gfdBdx2qfWVf2q63siPPyJsUSUOYt7kqP0lv5OsU78dxYDXCHIROwE3OMSdRjQFMyk6XBkQsNDYkAlnjNnDkycODHUazI5f5RULjm\/jZyY9oUXXoAPP\/wwsi2u5cTyTGRFHVi8eDFMnjw59dcQdG00kRPziqRnuvs7DVzlMpG0n3rqKbj\/\/vvh8ccfh+XLl6eOZWHIpL29PWD3vF1wsVPEA4v4eyST4cOHhx52zEOhTOWkZX98ugSVK4oY01B6E1mJTGh6k8c0x0ROwoem6FldbWrTLzi4UX+n\/UwNk0lIz9goFI26eUzRTOWkXcp4UNPEy7JRWtO0JrLSgDJhwoSAuEUvAa+4yOIzkVM+ElK0aQ7idFGSCc01fZ3m5OGRkFGZuOS60+BZx01MZCUjJc8pS4OwxVSckuchZxSxZilTITwTnwOwNG\/GnziK5vGZBjVJNtlYs5TZVFaRnPPwTEzklD2TPOSM6ruLjkzIIPHax6Itr4nLg+IoTocWhw4dGuwOxneG6Iu67yVKAeL8PUpOkejyJBOaDtJZLhWm8jmuvHTCBNOiLw1flGQSx4A4DyPACBQHgUJMc4oDB0vCCDACcRFgMomLHOdjBBiBbggwmbBCMAKMgBMEmEycwMiFMAKMAJMJ6wAjwAg4QYDJxAmMdoWYbnBL45wPHf7CpWzTDWu6d5DsWu0mNS3XprX8TuVn9RyuG1SKUQqTSQ79kCeZoLFs27bN6sa8opFJ2kcW8jilnIMaOq+SycQ5pL8tUNz0RCMpXrFAG7ZmzJgRGLWYDnOjx1BbWwvTpk2DPXv2lN5uppPLeKUmpdO9BiCWSaMslkV160Ze8V4ZOrhGZNKnT59ANtkrEPOIG8zwLNDevXth+\/btpQfXKC2mGzVqFGCZdBWoSmb5ygGZ2LCO3r17w5YtWwKsCFO5W2UvL4wgmUziGQWTSTzcInPJl9KIz3zId3WIpzrFw2L4joz8hjNWTAQ0e\/Zs5T0VNJVZsWJFYPh4qA+Nl\/LpRnbRwMQzUh0dHQEJEZHI5dE1DfTONMkov2gg7sbs379\/QJZIhiiX+LeqqqpuMotgq8iE7iumMrE8mWSZTCJVNnECJpPEEKoLCLvhKmyaIxqLSCZYCxofGUrYlnjZ4MQzI2GXT+keUZPPnITJL\/4NyyNiwZ9yPvkuEPFiIZ3noCKTsDqod5hMUlJ0oVgmkxQxFoOd4rRCNiq6+YxEobQqMkFXXvxUd2jIR+FNDlLqXljEumQDFuVXPdhGUw2ZnMLIRb7AHOtVBVlVZFJdXV06ZKkjOiaTFBX9k6KZTNLHOKhBdyOXPOqHeSamN9Cl4ZmIU6Mwj0L2TMIMPc4tZVGeiUxYOs8k7O4RjpnEMwomk3i4ReaSR0JdzER1vwcW3tjYCGExEzEuoooP4Elm25iJaGDocdC0CuUxIRPKQ3EQ2TMxjZngjWC6FyRVZIK\/w6s15amg2EmqOBLhLAd5mUwi1VuZgMkkHm5GuUTXXZzm0KoFTgfq6+uDYCMGETFIOm\/ePFi\/fj00NTWVjAP\/R7yHllZzwq4J1K2MRC3zilMueTUHCQ4NT\/QoxCP4OC2ZPn06vPjiiwEZrly5EkTPhDw0vGoC0+Lj2mfOnFGu5uj2kajIBO+z3bp1a3ANhIiJKkZD11wgUe7evVtJ2kwmRupdlojJJB5unCshAmExmrCio2ImCcUKsjOZxEORySQebpwrBgLyfhrdnpAoMsFlavJc8AZ22fuJIVopC8nIO2DtUWQysceMczACjIACgf8HkQkF\/pv4XdcAAAAASUVORK5CYII=","height":166,"width":275}}
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
