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
tc = ts_cllc/100;

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
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     1.065340909090909e-05"}}
%---
%[output:695452a6]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     4.127900074317465e-06"}}
%---
%[output:5672b5d3]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     3.919183588453085e+02"}}
%---
%[output:59bf7388]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     5.091168824543142e+02"}}
%---
%[output:935eeccc]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.076483869223994"],["18.982392004991411"]]}}
%---
%[output:8f4f28b7]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:8104d681]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:5ca7637f]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000050000000000"],["-4.934802200544680","0.999214601836603"]]}}
%---
%[output:5ad1539c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.077754418176347"],["13.583043056999228"]]}}
%---
%[output:1367780f]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.018721899663332"],["0.977712707506129"]]}}
%---
%[output:5788559b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARMAAACmCAYAAAD52WvKAAAAAXNSR0IArs4c6QAAH6BJREFUeF7tXQ2QV9V1P8aYuCBRFrBxXciCLmriVMRUKYOC4xg1KejQmfLRMRQoQeK6qYHh20FmAiyENVPQtNTZMKTt7ipTGsF8WEIDNSKSiFITUUiWhSJYYQmiBNqSbOc8PP\/cvf9737v3vfs+7u55M87K7v0493fP+d1zz\/26qLOzsxP4YwQYAUYgIQIXMZkkRJCzMwKMQIAAkwkrAiPACDhBgMnECYxcCCPACDCZWOrArl27YPLkydDc3AwjRoywzB2eXC777NmzsGbNGpgxYwZUVlY6qQvLnD9\/flBWQ0MDVFRUANZ76NAhmDBhgpM6dIU888wzsHPnzlK9YZU9+eSTcM8990BtbW2kTDZpIwv7KAHhtGXLFqiqqoL169cbyaLC17ROl+lOnjwJ06dPh3nz5jnXU52cTCYue9BxWStXrgwMvampKTUyOXLkCEydOhUeeeSRVMmElHvixImR9SDprF271siAbdLadM+BAwcCXMaOHRsYpOlXFDJBeVF\/jh49akTepu0LS1dIMqGORCDwIy+AOur06dPB73fs2FE2atDojn+\/6aabSoZIv3\/ssceC32HZK1as0Cq2TgbRe8DycZQneTAPjmDV1dXB73FUw2\/mzJmBQlKZZLiyJyL+Gz2FBQsWBPnlkVGlsDLxRGGI5c6dOxfq6upg7969XeREA9XVjfWsW7cukGnMmDGwffv2ktGLozkWKOIrGz2RC9VNacX+o74fOnRoMMrKcqrSorcoyo9kQB6YbAg6GeTfq8qQ20p9rNJRUQ\/JyBFDnY5iWfh3KjMMc1lW0WNO04tWkUrhyEQ2OFEJyUhfffXVQIH79esXKNmgQYMChRFH2XHjxnVx51ERcXoidpBu1JdHUdFQ9+\/fX5rmEJmQPOSSiyMC1YudjPKKXkAYmaBRhHkmiEtra2tAjPghDphHRVoqDDGPjBlOcxD\/5cuXQ2NjY6lcwpfagoZP+Ipt1+FEbRFHSTHt1q1bu3giMvFg2pqamoD4iSjIaOS0IqZEQoSLaADUx\/Q3uS+iPBNdH8s6gXXKfd7S0tIFe\/J+SAbSUcxLv1NhTvZAfbl58+YuONp4g93SM9GNYNjp9fX1ZfN9Mf2ePXvKlJIMjkiARsAw9xg7cM6cOUo3W+WZUGdi\/CGsA208kygyobJWr14d6IEYx7HBUDfNUY3uGL9Bb4viB2I9ROxknCIOMrEjTjTayqM2tkXVN7oRWEU84iChc\/VV8SkxlkS4qKY5YX1Mnsnhw4eVRE9GS+0XPVeVJ4HpdJjLRCXqBPaDTJguCCOsjMJ5JvJoLAISRSabNm0K3EPxoylCR0dHqMGJeaKIhgyXRiGRTGTCEMt1SSaktNg+GsEotmKDoUwmpNBoREuWLIGlS5cG5aMXg2QiGqqIEyk2TU2p3TjKYgBZ9CCRTORpmEgqKk8KDQq9kTDilKeXJIMJYclTxzAyCetjuRz8t+g1EkmLuOi8I5Rf7ksRG9Jp2cBpwCTbIc8ScU\/zKxyZ2IyqCE6YZyICJ7O+KWHIKzamnonKtXZJJuIIPmDAgNIURzWyhxGyTCai8iK+4mht45mI2IcFJcXYA7n3KpLSxZmiPBOd8bjwTFR9HEYm8mAoE01Sz0Rua4\/3TExiJjRK0ZzYJmaim2uLHSF3gmo0wHJUnok8muDoQXPmu+++u8soRa4uySQrU9Rqjji6i4E3EwzJ25DJRNVWCkCKMRNqy\/Hjx0vTnqiYCXk1MkmFySBOn8gYqf8p2Cqu\/GQZM6H2iH0sT+lkwpBjRRjoJhJVkYkYM5Ex55iJgZ8lGom4kiGyfp8+fQK3V3Zho1ZzTMgERbRZzRGnOfj\/ukg\/eQ20UkJRex2ZiG1R7WuR5+fiXhQTDHHqgh+tPCFpiCs8iD16PfiJU6g0VnPEFRNRdnTZ8SPMsL+RwMhTkdOKQVrMZ7OaoyJk3dJw1GoO6YRMJnK\/IL5ygFvu6x67moNgoUKuWrWqbJOP3AGquWwY1xRpDd+AE7tdElvPSeXtZbmJqjt0QBLM0cs03SToAiunMRMydnmplATF6cPs2bNh4cKFRrsJ5QYymbjo8vhlyIMBlmSzEzhr5Y7f0uLkjIu59ztgUVnQBUUyURGGuIfB1fbw4nQ7S8II9GwESp6JvI4fBotqeoJEsWHDBpg1axYsXrxYSSbi3gUsP2wHas\/uFm49I+AfAl3IBPcU4N6CMK8BSUdOh67YsmXLYMqUKcEuT5OpTB5umH\/dwxIzAv4g4CRmIq98YPOjTlrSXHDkyJFdzscMGTLEH\/RYUkag4Ahs27YNBg8enImUZdMcrDXJKdWwICtOc\/CjMyCqVR8kk7a2tkwan6QSX+TENvoiqy9yMqZqy+nimciR4yjvQlWkTCYigYSdKqWyfFEoX+RkxU8yZOjz+tL\/WcoZOs3JI2CaZeOTqNnBgwczcx+TyIl5fZHVFzl9wjRLezKOmagCr0mVXJU\/y8YnkZ8VPwl66ryMqXtMs7QnLZnIS8VxpjxxoMmy8XHkozys+EnQYzJxj566xCztSbvPJCvykCHIsvFJOpTJJAl6TCbu0SsQmWBwFP\/Le2cqk4l7NfOF+HyRk2MmEas5pjER03RxTYLJJC5y+ny+GKkvcjKZGJCJeGlvmErbnva1MQ8mExu0zNL6YqS+yMlkEkEmZmqZfiomE\/cY+2KkvsjJZMJk4tRKWfGdwhkUxpi6xzTLwdl4n4n7ZuYffU7SJlb8JOjxao579PK3JyaTmL3KZBITuJBsjKl7TNkz8eCgHyu+e8VnTN1jymTCZOJUq3wxUl\/k9Cm+kzuZiKd76cZy3e1pTrXeo+PyrPiue54DsO4Rzfb6ibKYifhS3Pjx44OrGBctWgT4jmkWN11nyaRJOo\/JJAl6HIB1j566xKovPgpHf\/CtTKorIxNxhys+LkRkgiRjcq1jUqmZTJIiWJ7fF+LzRU5fpjktP3sXHm7ZByefuNO9UilKVK7m0GPP06ZNg40bNwaXRNfV1XV5gjIt6ZhM3CPri5H6IieTiVpHtUvD4mtymDWrm+SZTJhM3CPgvkQfiK8Qnol76M1LZDIxx8o0pQ+K78toT5j7gCmTiScXSvugTD4pPpOJ6dBgnm7lC+2w8oWD+cVMoh7jSvvSJPZMzJXFNKUvxOeLnL4QX+5kgkDhRdLt7e2Aj0zTR78bPXo0tLS0QENDA1RUVJjqc1k6DPLiJ9aB\/2YyiQ2pNqMvRuqLnEwmhgFY3eVH9Pv6+npYs2ZN5Mt\/YSZBwd2ZM2cymbjnjrISfTFSX+RkMjEkE9q0ho+Pr1+\/Hmpra4Fe7Lvlllvg\/vvvh+eeey62Z0KkhOWeOXOGyYTJpIQAk4lbZdiw6yg8+uzb+cVMqDny0nBzczMMHTrU6B3hMEhweoNTpUOHDpVNpXia41aZOACbDp6+eCbjnnoNfvrrU\/mTSRrdgAS1Y8eOwBtRxWWITPAnvpFa5O\/IkSNQXV1dZBFLsvkiqy9yIrBFl\/Wuu+6CD0fNhfP9r+ueZIJeybp167oYoBw34QCse37yZfrgi5y+eCaVX\/9JoEy5bqeXnwUl9XZ5kXSYZ+LDw+Ws+D2X9Hwgk8Mnz8Gwb7ycL5mID49v2rQpiG+MGDEC0KuoqamBCRMmONEiJhMnMBoV4gvx+SKnD2RC8ZJcPRNxaXjr1q2lIGna7+WQVfA0x4gfrBL5YqS+yFl0Mvnpr07BuG+\/FujIx0+8De999yErfYmbWHufyciRI2H48OEwd+5cWLVqFezZswdaW1uhqakp1Vf\/mEzidqU+ny9G6oucRSYTcXozqPJSONX6NWj\/xW73SqUoUXlqWPZOFixYEGTF5WGc8qT5MZm4R9cXI\/VFzqKSyap\/a4eGHx0sKdDri\/8Uxnz+s5BVDJJvp49pu6z4MYELycaYxsP0lfb34b41e0qZ0SPZ\/NWbAX9mOTiH3rQmPmLOMZOuHc2KH0\/xw3IxpuaY4nSmrmVfsClN\/JBA0CPJIwZZIpOo08IoHF4unfSAXxRcWTJplCys+EkQss\/LZBKOmY5AMJfojYilZGlPxp6JvWrEy5Fl4+NJeCEXK34S9NR5GdOuuCB5rN7aDv\/0yjEt2DoSydUzca8a8UpkMomHW3fwonoymSBxfHfXUdh98P2yqYs8jRnU91J4ctINgTcS9WVpT1bTHJc7YHUgZNn4qI7oDgbqkxfVE8gESePyio\/Dg995Aw7\/5hzgv6M+JI2F9w2BEYMvNyKQwkxzohqW9t+ZTNwj7IuR+iJnFEEjQSABPPHjQ7D97ZOhnobc25jPxvOI0pYs7YmXhqN6Q\/P37qL4MZufSjafMH1p7wG45bPXwLxN++HgibNWhIHg0RTl3s\/1h6+OHmjtcZh2QO5kIj4PSkJnsZKDdWXZeNMOUaXzSfF9kbUoctLU470P\/jcIfrYd\/63xlETnadw25HJ48Laq1EijCGGD0OdBxftZ6WEuXhq+0G1FUXwTQvRF1izkJKLo9YmLYdkP2uDXCYiCsKepyfWf7g11dw4Kfm0SHDXpu6RpshycjZeGedNa127NQvGTKhLl90XWJHKKgcwf\/fIEPP+fx4PmmwY5w7Amsrj2yl7w0B0D4dJLPga\/e\/8YDB482FUXpVZOrmSCrUIvZMuWLWV3wOJUR75N3jUKWTY+iexJFD9JvXHy+iKrSk4iif\/7XSc88\/N3YVfbhR2fLkhC9CAw6Dl91NVw88BPGXkWvmCapT1pA7DyBUn8PCh7JnGITJVH9CJ+vK8DXj18Gv7r5DloO\/4hHD193kk1NM1AkhhYeSnMGFUNlb0vMSIKEwGYTMpR4tUcE81RpPFFmVD0LGQlgvh9Zyf88+5j8MrB9wE63XkQYnwi8Cg+IgmcduC+DdHLiNmlVtmywNRKIE3iQngmLhoSp4wsGx9HPsrjizLFJRPRe9jZdgrwwp3DHWedTS9k7NGTOH\/+PAwZcBncNLAPLPmza+Doqf\/JnCRMdcKX\/s\/SnpQB2OnTp8OgQYNSP9Sn6rgsG2+qOKp0vigTkcnFl18VNKMTAF45eAr+40D65EAeBP68vbYvPDDsSqi9slew61O12uEbphyA7WoZymmOap+J6vW9JMaoy8tkEo0qeQ1okPv\/+7fBtOK1w6eDjK4CkyopxDhETf8K+POb\/wgG97\/wRKyLpVAmk+i+t02RpT0Zx0zwzRtc5eFrGy90p0vFp5H6UMc5+OEvT8Cbxz6E9hNnMyUHDFLeeV0l3Fpz4fyHznuwVWab9C4xtak3TlpfZM2dTFSeSVVVVWmpWAW++AKgbresXK7q4GCWjY+jRJRHp0xkhPgT4wy7298PdlBm5TXQ1OL6q3rDl24cAKOH9gXc+j1wYHpbtpPgKOb1xUBdDyau8FOVk6U9aWMmKJipFyI+j4Gv3M2fPx\/wQmr5WQwxHb41nHfjTTqRphQYa\/j3tzrgX197L8jmchkzakrxmX4V8Lmqy+CvR10dKyjpi5H6IieTidpyjKc5JoaHacj7mDRpUtnl0\/gA+vLly6GxsVF7w32WTEptQsJoeumdIO6QRsxBjDXARQDDqvvAnLtr4NTZC3sqXMQbwvrHFyP1RU4mkwzIhKY6ummOyUa4NMkESeOdU+dgxQ8PWp\/yFOETlzGHDOgFX7n9arjsk9nvdTAleF+M1Bc5mUwyIBOqAklj586doUvLdOcsbs8Xn89AMsHP1cPluKPyH3afgi37Poy0vapPXSCEz199KXyhtjd8pu+FHZP0e7GAoj9c7aOsjGmkihonwIfL6fP6qQuTlR+aDsmxFReeCcU58FUz1U1WdHDrW39xHVxy8cdiTTN4FDXWa+OEjKkxVMYJXdiTaWVOTg3LJ4pxCRk\/+VAgeiz4YWAW4yf0WqAYjE3aeFxBqWvdV0YiURfvmgJG6VjxbRGLTs+YRmNkmyKpPdnUZ3UHbNgFSbqlYZFA5KVh1eHBuI1HD+T7vzgBi753oNR+JJC1E2+A26+9wgYTo7Ss+EYwWSViTK3gMkoc156MCpcSGXsmcQqPkydO45FIxCkNksiKB2rhvhv7xxHBKA8rvhFMVokYUyu4jBLHsSejghWJnC8NxxWE8tk2XkUk9DRiUlnC8rPiu0eXMXWPqa09JZGgyzRn6dKlUF9fD3PmzIG9e\/eWlVu0py7kF85cx0WYTJKoln1eJhN7zKJy5EImUUJl9XebxuPNW7Oa9wWiyW+spi0vK757hBlT95ja2FPS2r2d5qBXMuwbL+dCJFgpK35S1SvPz5i6xzRXMgl7wLwo0xw5ToKvvqe9JV3uZlZ894rPmLrHNFcy0TUH946MHj267LyN6+abNB7fZP2bZ98Oqt7wVzfC2D8e4FqMyPJY8SMhsk7AmFpDFpnBxJ4iCzFMYDzNKdJTF5Vf\/0lu0xvClRXfUMMskjGmFmAZJi0kmZhskTdsX2iyqMZ\/\/40T8OD6N4IycAl4VAob0kzawYpvgpJdGsbUDi+T1FH2ZFKGaRrtfSaqpeHm5uZcpzl5B11FUFnxTVXMPB1jao6VacpcycRUyLTShTVeJJM8vRJsOyu+ew1gTN1jmjuZyIfw8HxNa2ur8c1rSSAJa\/y4p14L7iHJek+Jqj2s+El6WZ2XMXWPaa5korspzeSOEhdQ6BoveiWP3DkIlo69xkV1sctgxY8NnTYjY+oe01zJRLdqk\/dqzteefQv+cdex3AOv1N2s+O4VnzF1j2muZILNQS9k7dq1pdvoaSMb3oiW18PlRVgO5gCse2VnTNPFNHcyweZh3GTq1Klw9OjRoLV5PlyOFx7hFQP4fW\/WMLijtm+6PWBQOo+iBiBZJmFMLQEzSF4IMjGQM5UkqsY37z4Gda1vFWaKg4Kw4rvvfsbUPaa5kklWsREdbKrGF2kVh2Mm7hWeMU0P01zJBJuF53BqamrKHtFKr8l\/KFluvLiKM+uOgbDsgWuzECOyDh5FIyGyTsCYWkMWmSFXMinaqeGWn70LD7dcuLMk741qYs+x4kfqsXUCxtQassgMuZJJpHQpJ5AbT1McrPbkE3emXLt58az45liZpmRMTZEyT9dtyUS+nV511kdufNGWhHl+b67ItimZTGwRi06fC5lQ4DXNO2Bx\/0p7e3uwV0V3CllsvBgvmXfPYJh3T000ehmlYMV3DzRj6h7TXMjEfTPCS0QyaWlpKXtCVGy8uL+kSPESbBkrvnuNYUzdY5o7maR50E+c6kRNc+7\/9uvw4q9+EyCcx9WMYV3Liu9e8RlT95jmSiZZHfQzebj8K5vehVffORc8HL5lSrV7pBOUyI9sJwBPk5UxdYdpIR4uz+qgX9TD5WK8ZNQ1V8Dmh292h7SDkngUdQCiVARj6h7TXD0TbE5aB\/1sHi4vcvCVYybulZ4xTQfT3MkEm5XGQT+bpeGiblajLudR1L3yM6buMS0EmbhvllmJ1Hjc9YqEUsTgK4+iZn1pm4rJxBax6PRMJm1tUMTDfWLXseJHK7JtCsbUFrHo9D2eTLb\/\/M3S059FDL6yZxKtxHFSMJnEQS08D5OJQCZ\/O+F6ePC2q9yjnLBEVvyEACqyM6buMWUyEcjkqUk3wKQ\/+bR7lBOWyIqfEEAmE\/cAKkrMnUxwq\/vkyZPLRMvq4fJlz+ws5LUDHDNJV\/+ZoN3jmyuZ6Hamum+mukRs\/I2z\/yV4Hwe\/Il07wGSSrhYwmbjHN3cyWbp0KSxZsgQqKyvdty6iRJFMivDYlk5cVnz3qsGYusc0VzLB5ohXBbhvXnT0+dQDTUGioq7koGys+O41gzF1j2muZJL3tY01N94Kp7+wMkAVA68YgC3ix4rvvlcYU\/eY5kom7ptjV+KgW++DD0fNDTIV7Q4TjpnY9aVtaiYTW8Si0zOZMJlEa4lFCl+M1Bc5fZrm5k4m4oG8sWPHwty5c2Hx4sWwcOFCqK2ttVBj+6RVX3wUzl0\/jj0Te+i0OXwxUl\/kZDJRq9pFnZ2dneKfiEiqqqpg\/PjxsGHDBli0aBFs3rwZdu7cWXbNokOdD4q68st\/B+f7Xx\/8f1GXhX1SJp9kZTJxbU0AuXom4uVIHR0dJTJBksliyfjKL\/89nO9\/HRR5WdgnA\/VJViaTbkYm2Bx80Q8fLJ82bRps3LgRZs2aBXV1dTBixIjgZvk0v\/4PbYTf9+pf6GVhnwzUJ1mZTNxbVq6eCTVH3lK\/YsWKTJ4LpXdyxgztC5seGuYeXUclsuI7AlIohjF1j2khyMR9s8xKJDIp2js5svSs+Gb9aZOKMbVByywtk0kBn7ZgMjFT3iSpmEySoKfOmzuZyHe1opi4RNzQ0AAVFRXuWyyUSJ5JkTes+RSH8ElWJhP3ppUrmYhLwxRspd9hU3WEgud5FixYEKChu6pAJilVOiKToj26xZ6Je0VnTNPHNFcyifNuDt5kv3z5cmhsbAxOGtNqkEw8WPbs2bNDN78RmRR5j4lPo71PsrJn4p5cciUTbI7qHWAkiJqaGqMVHd07wjLpqKBDMin6HhOfDNQnWZlMuhmZhJ0apqZG3bimIx5xKoRlqZabmUzcK5QvRuqLnD4RdO6eSRJ1Nr0LRXejG5LJqGuvCE4MF\/ljxXffO4ype0y9JRObqZDurWEkk08cfgle\/uZfukfWYYn8yLZDMD8qijF1h2khHi7H5sRZGkYiGT16dLDlXveZvDWMZFL0DWvYviwZP6mK+SKrL3L61P9ZYhp6alg8h6NboaGArXybPe1LwdPG+E2YMKGMpHQxk157vhN4J\/wxAoxAcgTa2tqSF2JQQhmZxFkaNqiHkzACjEA3R6CMTLC96IVs2bIF1q9fH1yGhEu6U6dODXbBpn1quJvjzc1jBLotAkoywdaaLON2W1S4YYwAI2CNgJZMrEviDIwAI9CjESgMmYieUHNzc+iqUJY9Jt7rorvTRUyD113S9LBocpI8eb\/aaIIpTbfXrVsXiJ2HTpjIKW7yzKvvw\/TM5AiLKz0tBJmI2+z3798PLS0tmZxQjgJR7AhMK54\/Eg1TPG+EpNja2gpNTU2ZvYhoIqfYVoyJoZHmYaCmsoqbH1E\/6C7itE+tq\/pV1\/dEePgTY4kocxb3JEfpLf2dYp347ywGuEKQidgJuMcl6jCgKZhJ0+HIhIaHxIBKPH\/+fJg0aVKo12Ry\/iipXHJ+Gzkx7fPPPw8ffPBBZFtcy4nlmciKOrBs2TKYMmVK6q8h6NpoIifmFUnPdPd3GrjKZSJpP\/3003DvvffC448\/DqtWrUody8KQSXt7e8DuebvgYqeIBxbx90gmI0eODD3smIdCmcpJy\/74dAkqVxQxpqH0JrISmdD0Jo9pjomchA9N0bO62tSmX3Bwo\/5O+5kaJpOQnrFRKBp185iimcpJu5TxoKaJl2WjtKZpTWSlAWXixIkBcYteAl5xkcVnIqd8JKRo0xzEqUeSCc01fZ3m5OGRkFGZuOS60+BZx01MZCUjJc8pS4OwxVSckuchZxSxZilTITwTnwOwNG\/GnziK5vGZBjVJNtlYs5TZVFaRnPPwTEzklD2TPOSM6rseRyZkkHjtY9GW18TlQXEUp0OLw4cPD3YH4ztD9EXd9xKlAHH+HiWnSHR5kglNB+kslwpT+RxXXjphgmnRl4Z7JJnEMSDOwwgwAsVBoBDTnOLAwZIwAoxAXASYTOIix\/kYAUagCwJMJqwQjAAj4AQBJhMnMHIhjAAjwGTCOsAIMAJOEGAycQKjXSGmG9zSOOdDh79wKdt0w5ruHSS7VrtJTcu1aS2\/U\/lZPYfrBpVilMJkkkM\/5EkmaCw7duywujGvaGSS9pGFPE4p56CGzqtkMnEO6R8KFDc90UiKVyzQhq2ZM2cGRi2mw9zoMQwdOhSmT58Oe\/fuLb3dTCeX8UpNSqd7DUAsk0ZZLIvq1o284r0ydHCNyKRPnz6BbLJXIOYRN5jhWaA333wTXnzxxdKDa5QW040ZMwawTLoKVCWzfOWATGxYR+\/evWHbtm0BVoSp3K2ylxdGkEwm8YyCySQebpG55EtpxGc+5Ls6xFOd4mExfEdGfsMZKyYCmjNnjvKeCprKrF69OjB8PNSHxkv5dCO7aGDiGamOjo6AhIhI5PLomgZ6Z5pklF80EHdj9uvXLyBLJEOUS\/xbdXV1F5lFsFVkQvcVU5lYnkyyTCaRKps4AZNJYgjVBYTdcBU2zRGNRSQTrAWNjwwlbEu8bHDimZGwy6d0j6jJZ07C5Bf\/huURseBPOZ98F4h4sZDOc1CRSVgd1DtMJikpulAsk0mKGIvBTnFaIRsV3XxGolBaFZmgKy9+qjs05KPwJgcpdS8sYl2yAYvyqx5so6mGTE5h5CJfYI71qoKsKjKpqakpHbLUER2TSYqK\/lHRTCbpYxzUoLuRSx71wzwT0xvo0vBMxKlRmEcheyZhhh7nlrIoz0QmLJ1nEnb3CMdM4hkFk0k83CJzySOhLmaiut8DC29oaICwmIkYF1HFB\/Aks23MRDQw9DhoWoXymJAJ5aE4iOyZmMZM8EYw3QuSKjLB3+HVmvJUUOwkVRyJcJaDvEwmkeqtTMBkEg83o1yi6y5Oc2jVAqcD9fX1QbARg4gYJF24cCFs3LgRGhsbS8aB\/yPeQ0urOWHXBOpWRqKWecUpl7yagwSHhid6FOIRfJyWzJgxA1544YWADNesWQOiZ0IeGl41gWnxce0zZ84oV3N0+0hUZIL32W7fvj24BkLERBWjoWsukChff\/11JWkzmRipd1kiJpN4uHGuhAiExWjCio6KmSQUK8jOZBIPRSaTeLhxrhgIyPtpdHtCosgEl6nJc8Eb2GXvJ4ZopSwkI++AtUeRycQeM87BCDACCgT+H8YEDP652SYOAAAAAElFTkSuQmCC","height":166,"width":275}}
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
