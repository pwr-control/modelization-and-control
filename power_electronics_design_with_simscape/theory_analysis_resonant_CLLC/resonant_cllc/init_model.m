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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAARMAAACmCAYAAAD52WvKAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXXuQVkV2P2OIMhtFGHVXRnBBHquuuyCoi6iFW5YS4zKiyRYzmIohaKj4oFJCeMjqFhaPAcEqAriLFFKkEmY0WQlQ2ZQiGlcESx2BlFkVFYbh6SJI0BW2iuykzmXP2NPTfbv73v76uz1z7j88vu6+5\/z69O+ePt19uqK1tbUVcjwnT56EGTNmwMaNGzu0MmnSJJg+fTp89NFHMGHCBDh48GCHMmPGjIH6+nrAdiZOnAg7d+5sV4Z+379\/v7GNysrKdnV1slVXV8Pq1avhggsuSN6Jz6pVq5I\/ZRnw\/Y8\/\/jg88cQTHXTUtVNVVZXog7g0NTUl78IHMRg+fLiyPdJT1OHNN9+E8ePHA+GIbSxYsABWrFgBa9euhREjRrT9WwZWxhV\/X7ZsGSxcuLCdHlhu2rRp8NBDD3XAnvTr06dPhz7G3y666CI4cuRIop+qDL5zyJAhCbaIiSy7zi7ove+++y7MnDmzTTX6\/xdeeCHBQH7mz5+f\/JeqjtwW1SUc5bbSbBbr7N27V\/meQYMGtWuK+lBun\/r0ueeeS9q56aab4PXXX29XDPUZN24cUBlTG+LvKhnxd3qPqW0RF3o\/1dHRRUVeMiGwqKMRTOoIfKk4kJBMSEixs0TFdQMHBz4RErVB7xYNVlRU9bsIzK233qolExE413ZMZILkiQ+RsM6gjx07ppSPyO\/o0aMdMKE6SMrY7uDBg9u1gWRleq9IwtgGYU\/9iX0s9zvKgsQnkqKIdU1NTTtyFe0ECRYxkT8G+D5qQ7QL2ZjlMvK\/CRMiPpt3E\/Gp9KH\/Q4xlshdlE3Eke5L7dNOmTQmZiDZs0pmwJzk2bNjQoY20d6PcKA\/ZvwkX1UdNRSi5yUTHWtQZKPSwYcPavsqi0YhlmpublV8bFFpsA7+G9KWTOwYHsfiYGFU3WEVwRYPWMbNKjjTPxJZMxHfjoMZHNF5dJ4vGeP\/991uTicqTU70X5ZD10335sSwSAckhYqt6n+yhpQ0s+Ystf+1tPGPVh0jWjbwNmZSIQHVEZ7JPHVnq+lX2cmQyEeXQvVscEzQuVTMGEReZvFSkj7pERybiV8zUWVnJhL78RE627YgDRTY8bJOmOS5kQh4cGg4+4hTHF5nIOMrei0xivsiEiF9HYjiNU5GJPF0yeSZpU5aikInKE5bJkcroPBOxjSjJJOs0R3bHaQ6qY3kyCBcySZueiF9LIg\/8U4yhEJnYtkNTCxwEMi4ymeCAlQeFynWUB5r45Zanivhe0zRHFbvQTQtIn6zTHJXLTzEkObZAZWVZZMIknUUPlXAk25HrqKY5KqzF\/yv1NIc+POTR6chE5dERRrJnIraRdZqjwiXYNKfUAViKxagGo8kzSZNNFU\/QkYmpHfy66YLQafKTweJ7VQFYeYDRlA+DcvSIbYiGoArAyoFQeu\/dd9+deE2qB3FS6WcbgMU25WCxHPPSvVckZSyD71yyZAnMmTOnQ7AYf5fJhOpg3E43DTN9vFRTAFn+tHiOLgCbRgTi4J08ebLWttLaQN1VgVvbAKyok8kzp\/7LPc3RGbXIkqJXceedd8J9992XVJMHkPhVFQ1BFzAzkQm2keZCmwKcYgwmrR18jyz7Y489Bjt27FCu5lDcSDQ0XRBZbFuO5ajIRhxUYl38O5GJ6r0rV65si1mhIY0aNapdfEbUHwf18uXL4dlnn23TD\/vIhJHKKFVTEJW7Tqt8YgwHdULcFi1aBFOnTk3gUK3K0eDXxViULKroU3GRAevYfrF1sQ70TlWY6LwxJDYV9irvRvchwv\/HgK+IsQkXGw8a263o37+\/dmkYO2rdunU6rJP\/R2PApUVcclS5rjjIMGj44YcfJuXvuOMOePLJJ5WR+9QX8Y9lQ0A1nZJX7HTTFlFoMf6DWwb4KR0CIrGTl6Ga+pgkoDotLS1tHyNdHa1nguyIjJRGJrqoN72Mfh8wYAA8\/\/zzMHTo0OSnvn37JvtP+IkDgbRpXtr0TKUd2hTaFnlJcSAQp5S6\/Smyh5WmHX0AHn744WTPS9qTi0yIcPAFOs+EvBdaybjmmmvgnXfe0e4riLPbOr\/U8jQONXYlEtEWbIyz86Naeg3lmJoLkaB0LuSfOWaCxjV79myoq6tLXphGJiJkyJZbt25lMim9HfEbGIGgCGQmEyQFfHDjS1rMRNSGvm61tbVGlykoCvwyRoARyI1AJjLBedSaNWtg1qxZgGdmbMiE5t0osW7r9GWXXZZbIW6AEWAEvkbg75eth8l\/9r0gkCSrOWnr5CopcFqDS4e4D8C0moP1bYgEyyGZ7N69O4jieV\/CsuZFsGN9xtQvpg1vH4YHG96HY0\/90G\/DmtYqdu3a1UoH6GyCM6pAHLWtOrBGRIJtm1Zw2JhK0+ex4BqLnLF8+IKTiXhqmFZnXJbtTJ4JejG42UY3tRGHDxsTkwl7pv5sIDiZiJ4JqqE7Dq9TUSYT8kRwlYeOv8s5SnS7PZlM\/BlSjCTN\/e+3\/4OTCcZMsuwX8Kv2mdZiMqY9e\/ZA\/\/79SwGD9zZjkTUWObGDYpB1wYvNsODFPeFiJnmTI\/m0fCYTn2h+3VYMhh\/LACVUY8CUySSS1ZwYjCkmw2cy8f8hKQyZ2JzN8a0+eya+ET3TXizEF4ucsWDKZMKeiXdGiWWQxiInk4naRHOnIHCxfNMyMnsmLmjal41lkMYiJ5OJhkxCBWBN6Qp4NceeHFxLxjJIY5GTyaTMZGKTroA9E1easCsfyyCNRc5YyAS30uNek2Db6UN4JrbpCphM7MjBtVQsgzQWOWMhk5rl22HLJ8c7F5nYpitgMnGlCbvysQzSWORkMinTNMclXYEqBcHmzZvtRkzgUph6Aa\/EjOGJRdZY5MQ+L6qst9xyS5tJnrhtAfz+Gxd2Hs\/EJV0BeyaloaZYvvixyBmLZ1L1yKuJQQWLmeDZHLrr13c6Rdd0BUwmTCZ83smPDbQcOwVD52wLSyZjx45txTwjdNGSTd6RrOryPpOsyOWrF8sXPxY5Y\/BMap7eAVs+\/rw8ZEK3p+G0xCWfiYuZM5m4oOWvbCyDNBY5i04moldy1lefwWc\/\/7E\/Y0ppqYI8kxBkYtKIpzkmhLL9HssgjUXOopPJ0ldb4KcbP0mMpcdL06H5vbeyGY5jrTYyoURG5cwcz2Ti2HuWxWMZpLHIWWQyEb2SS6u6w4ln7wmWV7nd2Rzx\/lFLO\/VajMnEK5xtjcUySGORs6hkIhIJyrjjJ9fDzddcGY5MQuyAtR0iTCa2SLmVi2WQxiJnEclky8fHoebp7W2GseGBq+HGgT2DZi\/MdG+Omynbl2YyscfKpWQsgzQWOYtGJrRtnmyCiAT\/HXJMaVMQpN2lI19kbbp3R7xAOS3fbEjFXQajqiwbfl4EO9ZnTO0xxSnNwhf3wNq3D7dVwhgJEgn+SU\/IMdXmmYhXd9bU1MCMGTNAt+cEl4\/xwf0ppis\/xeszKisrU9sNqbh9t6lLsuHnRZDJJAuCSCIPNbyfHOATHyQQjJHIT8gx1W6aIw78Xbt2JReSr1u3zqizSC5yYfmi8rSLy0MqblTKUIDJJC+CTCa2CCKB\/GvTYZj7n3s6VEESWVZ7RRIfUT0hx1RuMiHPBL0U3KsiPyrPZOTIkcqLy0MqbtuRunJMJnkRZDJJQzCZxrzUDGvfOqQsNv663rCs9nJjJ4QcU8ppzrhx44A8iCVLlmgFRo9kxYoVYLp3B3e+0hWkaZd8hVTc2AvsmeSFyLl+VyZoJI\/\/Ofgl\/Oy1fR2mMASkKiZiAjnkmGoXgKWBjkSydOlSWL16NQwaNMgkbzId0l0Bim01Nja2bdFPmxJxCgIj1JkKFPW4vKxMLHKi3HllPXjiNPzbe1\/Ae4d\/B00HTmn7tbpHN1hx18WAf9o8YgoCKh\/qylUvS8O6Mze06iNOa9LO54RkUZuOSSvTlb+iebHralNH9Dre3HMc\/vnNQ1qvQ\/Q+bhrYC\/7htn7tVmWyYh5yTHkhE92F50wmWU3Ab71YiC8WObF3dLIicTS+cxi2fPS5kTiwHZy6\/Ol3L4QHRvX1Qh6y5QQlE8xnIgqgu1RcLCNOVYgwdMvIqmmObkoUUvG8w7UzGH5eDHzXjwnTN3Z+BMcqesLK1\/dbkQYRx6W9usPSuiug4g9E4hvDspLJ5MmTW+vq6oAO+ulWZUQh5U1rYgCWfsM2aXWHArXYBm9aK7X5dGw\/lkFaRDnR03hmy374731fWJOGSByLf\/wdOKfbWSXxOmwsKeQHuqK+vr61X79+yVItTlcaGhqgvr4ecINZ6Cek4nl1K6Lhxx6LKBemSBjvHfgSfv6rfdDy+SnAf9s+tNv0rqHfhJ\/+aIBttWDlQo6pisbGxtbm5uZkN6su9hFK85CK59WpXIafRe5YZC2VnEQO77acgGffOOBMGKKncV3\/8+GvRlTD\/\/3vIYghxWTIMVWxbdu2VsqutmnTJvCdB9bF+EMq7iKXqmypDD+vXDHLmgdTJIy+vbrD1F\/sgo8+\/W1mwkD8bhjQEx68uS+ce0437fQkj6yl6GNdmyHHVLKaQzENDKLa7i0pBSAhFc8rfyzGhHrGImuanORd4A11b3z8eSayIA8D\/7xxYC8Yf93FMPIy9TZ0k33EgmnIMeVladgEvO3vIRW3lSn2OEQsZIJk0bJvH6z\/uBU+PJzNs6C+wjgGrppcekFlsl+jFCsnTCYdR4b2bE5VVZV2vLmmIMBYzPjx45P20paemUzyUpy6fhEMH8kCs6U3\/uHIvHzq1UVzCnridATjF73PPyf4akkRMLXBLOSYykQmLikI6FzOokWLkqViPjVsYwJ+y5TS8Gn6cXa3s2DuL3fD3qMnM09BRM+CYhcTRl4C3zzv7OQnMU+HX4TcWyslpu7S6GsEJRN505op0ZFKbFMKAlotMoEUUnGTLKbfYzGmPNMcJAocwL8+9CWs+NV+2POZP6LAaciV1efCvLGDYP\/nZ97TFTA12ZXv30OOqdwxk7QUBKrt9GlghVQ8b6fFbvjkUezY\/wW8\/P5RaPZAFOQ9IFH0reoOf319NXyrxznWXkXsmOa1qVLUDzmmcpGJKQUBkcno0aNh5cqVsHPnTo6ZlMJihDbJm8DpxtJX98GuT38LJ0+dSj2ZaiuSGKv40fcvgu\/2Ptfr1IPJxLYn7MtFQyakki4FAZFJS0tLuxQEaWdzZJg2b95sj1zAknmPoGcVFY+ubz94Ct7efwrw74e+OJ38mffBI+69z+sGl\/b8Y7j9O38Cwy\/pnrRre\/Q97\/uxfrkwzSJ7UWUtawqCLAf9ZPA5BUEWc2xfBz2KT458Bet3HoHdR77KHcSk1tGbOH36NFx20bnwvT7nwYSR1XD2H51lPfXIr5l9C+yZ2GNlWzI6zyRtGz56LXT2BwFA4pk3bx4sXrwY5OXnkIrbdoaunI3hU1zi8InfwfNNn8KunPsnZFloP8W3L6iEmwf3gj8f9q3kXIm86mEja148fNSPRU7UNRZZQ46pTDETlxQEMtGYMq2FygqVx\/hxwOLZjIoeF8OSV1rgk998lTSXZ++EKA+RAQYyb7\/qQvi7UX2VJGGrQyyGH4ucTCZqy8t0PahrCgJx01osKQiQMF7+4Cj8+\/bfeCMKkSRuGNgT6q7t3dYrpdxDEcsgjUVOJhMNmdD1oKb7b2y\/gnnKhXTJSE7KAt5y9GRmz0Ikie\/3OQ\/uvb46yWGBTylJwhbrWAZpLHIymRjIBH\/uCikIdJcY6Qam7E1MH90\/mXLEcgQ9JsNnMrH9PNiXC\/mBzrSd3l4Vt5KlUtyGQCiYOf+uQXBed\/3Rc9KIDd+tb21KM6Y2KLmVKdWYUkmhvTfHTWQ\/pX0qTispeDO8nDmLiGNZ3RWZpyFs+H76XGyFMfWPqc8xZZIuUwDW1GjW330prvJEkEDwlClOU3zEMdjws\/ayvh5j6h9TX2PKRrJMS8OuKQhIkLQ7c7CMD8W3fHwc0BuhB4ljxT1Xwg\/6n2+Dh3UZNnxrqKwLMqbWUFkX9DGmbF+WiUxcUhCQIERATU1N2mxueRWXieTGAT0hz1QmDUQ2fFsTsy\/HmNpjZVsy75iyfQ+WS6Y5aff\/2jSWthGN6tNKEf574cKFymtH8yguEonpZngbnUxl2PBNCLn\/zpi6Y2aqkWdMmdqWf0+uusDLx\/HJkgM2LQUBvQzLzJ49G\/AuHSQe32QSmkhQLzZ8V1Mzl2dMzRi5lghKJrRpDYXEgY4exKpVqzqcm1EpYUpBQHUwuxo+w4YNg2nTpnklEwy2Dp2zLWkfPZIdP7neFe9M5dnwM8GWWokx9Y9pUDIRPRObq0F1pKJLK4BB1zVr1sCsWbOSI+YmMpHbN6Ug+NsXDrfl6njm7ouTo\/MhnqIeQVfpHoussciJGBdV1rKmILjhhhta815vkbZKg97LqFGjkvyvpVjNqXrk1WT8\/OUPesM\/jrs8BI8k7+CvqH+oGVP\/mAb1TMRpTlZVdNvwKZ6CGdbkRxX0dVG8XNMb0oMNP6u16Osxpv4xdRlTed+ee2mYlnwxeItXjKY9Pj2Tp17eC3N+uTt53YYHroYbB2a7TCkrgGz4WZFjMvGPnL7FwpOJawoCUtUXmZTbK+FpTmmGAxO0f1wLTyb+VT7Toq3i5fZKmExKYwFMJv5xtR1TPt6caZrj48WqNmwVp6BryKVgWV42fP9WwJj6x9R2TPl4c3QpCMQNauWIlXAA1ofZqdtgMvGPLZNJCqY1y7cnGdHQK0Ey8XECOEsXsuFnQS29DmPqH9OgZOLjelBfENgoTlMcPMS34cGrfb3auR02fGfIjBUYUyNEzgVsxpRzo5oKmWImLikI5L0meRJKF2WKwwFYX+bXvh0mE\/+4Fp5MbFMQyHcNm\/akmBQXpzihzuDoupcN37\/hM6b+MTWNKZ9vzOSZyALYpCCgOnjob+vWrVBfXw+VlZXtmkpTXNxbUu4pDnsmPk3w67aYTPzjGpRM8l4PapOCQIQoK5k0vH0YHmx4P2kKvZJyBV5JFzZ8\/4bPmPrHNCiZ5DmbY5uCgCAy3c2TpvhfPLMTXvngGJNJBnuLZZDGImdMnmk0ZEJ2jaSiS0FAZShegv9WTXHw\/1Fx+cEUBAdPnIYxa\/YnP1X36AYb7+2TYUj5rVLUI+gqLWORNRY5EeOiylrWFAR5pzkIrOnMjQ2REJmo7hoW4yWYXX766H5+mSFDa\/wVzQCaoQpj6h\/TsngmOAWZMmUKPProo8r8rGlqpt0EaFrBEdvVKS7GS8q561WUlQ3fv+Ezpv4xLQuZoBpICg0NDdppiDitwb9jygETWdhMgahdneJFWhImWdnw\/Rs+Y+of07KSCQ5+Uw5Y2xQEgwcPhokTJ4KcHEmXHlKneBEO9sndzIbv3\/AZU\/+Ylo1MXLwI\/2qrUxCI8ZKltZfDPdf1LsWrndtkw3eGzFiBMTVC5FwgKJmIAdgsV104a5dSQaV4kbbQc8zEZ293bIvJxD++QckE95nI2979q2TXokrxtW8dgocaP0gaKErwFWVhw7frU5dSjKkLWnZlg5MJimXaUGYner5SKsWLGHxlMsnXz7raTCb+cS0LmaAatqs5\/lU+06JK8aKkHJB1ZsP3bwWMqX9My0omNqs5Liq7pCuQFS\/iZjXSnQ3fxQrsyjKmdji5lCobmaQdwnNRQCxrm65A5ZmIZLK87gqou\/birGJ4r8eG7x1SjkP5h9Q6SbuPV7elIMAt8RMmTIBFixYlt++V6klLVyCzaBF3vrJnUirL4KB2KZAN6pmIS8OqW\/Z8KmhKVyArTsFXlOHYUz\/0KUruttgzyQ1hhwYYU\/+YBiWTPCkIXFS3SVegI5NyXmmh05EN36X37coypnY4uZTqlGRCAKTtshVTEPz+GxfCidsWJNWGX9Idnrm7OPESlKmoR9BVhhaLrLHIWeT+L2sKglCeCRl5WroCmUVpWbgoaQfEgcpfUZfvo11ZxtQOJ5dSndozSUtXICoubqMv2koOdiYbvotJ25VlTO1wcikVlEx8JEdKU05cvTGlKxAVL1rOV1lHNnwXk7Yry5ja4eRSKiiZlHqak5auQAZFVBzP4+C5HHyKtpLDnomLOduXZTKxx8q2ZKciE1ulsZyoeFHP5JA+bPguPWtXljG1w8mlVFAyKfU0J6viQ+dsA9wBW4Q7clQ6sOG79KxdWcbUDieXUkHJpNTTnKyKF\/WAH3smLj3qVpbJxA0vm9LByaRo+UyKfMCPycTGhLOVYTLJhltareBkkgQ5jx1L8rXW1tbCuHHj\/Gtl0SIpLi4LF+H2Pp7mWHSehyJMJh5AlJooC5mgDLb5TIh4KFH0mDFjUjPa42nkmTNnJmqmlSXFi3zAjz0T\/wbPmJYO07KSiSmfiTwlMu0dETep4UXlM2bMAMw1i9dkyA8pvuDFZljw4p7kZ\/ZM8htaLF\/8WOTEHolF1rKRSdZ8Jmn15N9sLi4v+rJwTMYUk6yxDNCYMC0LmeTJZ5JGECrPZOTIkcq4DCnOZJLfGxFbiGWQxiInk4naPivy5jOxCdwSUeHl5mk5U4hMir4sHJMxxSQrk4nfjwi2VhbPJIsaNheSo9fS2NjYdkugKdOamHrg7JY3YNuT92QRreR1+Li8f4gZ0\/yYRpmCwIZIVPtXTCkI\/uudXwPufsVn\/QND4aaBvfIjXIIW+CvqH1TG1D+mhfdMTCs4BEkWMvmnl96Fmqe3J00UMfUA6caG79\/wGVP\/mBaeTFzuJFZNczB2Ul9fD7hULD6o+NzntsKDDe8n\/13UZeGY4hAxycpk0sXIRN6wRuoPGTIkiYvQXpK6urq2LPeU\/xXLmjatTfrZK217TIp0HajczWz4\/g2fMfWPaeE9E\/8qn2kRFb997n8A7oBlz8QfyrEM0ljkjMnbC0omRUtBcNWUX8CWT45DETPSi8ObDd8f2XEcyj+W1GJQMilaCgImE\/+GFQvxxSIneyZqG2270c+\/Cbu3iCza42\/+pdBJkfgr6t6vtjWYTGyRsi\/XZT2Tfldd13ZXTlEzrDGZ2Buya0kmE1fEzOULTyauKQjwfM748eMTzWnVp6qqqgMSIpkU8a4cjpmYjTdPCSaTPOip6wYlE9cArGsKAvkAoenU8PGxqxJUirwsHNOcOSZZmUwiJxMfAVhTCoLm5mZl\/hIZOmRRIpMi736NaYDGJCuTCZMJ6MjENbesSCbsmfgzrFgGaSxyxkTQwac5mBZg7969sHXr1tT0iyrzTktBQGQyevRoWLlyJWCax7SYCZOJPwKJMb7DZOK\/\/4OSydixY1sxhSIO8rSUiio1TSeH6feWlpZ2KQjSzubQNKfHS9PhrK8+g82bN\/tH2EOLfFzeA4hSE4xpfkzLmoKAyGTEiBFJQmlTDlhS10QkWC7LqWEikyJeCRrj1z4ml5w9k\/xkoopD7t6923\/DihYrspCJbQoCfB+SU79+\/drSNOLqzrx582Dx4sUgLw\/TNKfoW+ljGqAxycpk4n\/Ml2WaM3jwYOt7c1xSEMjejinTGnomTCZ+jSqWQRqLnDERdFAyEfeZzJ8\/33gBV5YUBOKmNVMKAiSTou9+jcmYYpKVycTvRwRbC0omPvaZ+IKApjlMJr4QPdNOLIM0FjljwrTLk0nRt9LHZEwxycpk4vcjEtwzyXvVhU\/1yTNhMvGJKnsmftGMy9sri2dic\/9NKTpFbJPIpOi7X2P62sckK3sm\/kdYWcgE1XDZZ+Jf7TPBIgzAMpn4RTeWQRqLnDERdOHJxDUFAQ2NtDtzaH7HZOKXSGIyfCYT\/31fFjKxnea4piAgeKheU1MTrF69GgYNGtQBOVQc7825cWBP\/6h6bjFkJ+UVPRZZY5EzdGAzT\/+HxLTdXcOy0HheZ926dUZd0lIQUGWaQuG\/Fy5cqCWTUFt\/jUoZCoTspK4iK2Oat6c71g+JqZccsCYyQa9n9uzZgHfp4A5YJhP\/RpPWYkiDyqNZLHKyZ6Lu5dxkYjM9QrLBZ9iwYTBt2jQmkzwjLkPdWAZpLHIymWjIxHU7vdiMzclhDLquWbMGZs2aBXjE3EQmGcYKV2EEGIEUBEKFDto8ExsPw5VIsDxOa0aNGpVcFWpazWGLYAQYgXgRaDfNsd1nYpuCQHcoEOHC7G5IMPwwAoxA50AgE5m4pCAQYWLPpHMYDWvBCKgQcJ7mZElBQC9mMmEjZAQ6LwLt9pnY5DPpvFCwZowAI5AHgdxLw3leznUZAUag8yDAZNJ5+pI1YQTKikBhyASDuitWrEjAKMJKD11ritdypKWaRHlF2aurq7Vnj0rV0y6ykgyuF6T5kt1FVjk+F9IuXOQUy5aj\/9P6hvoZd5+XevW0EGQiLknv2rXL+roNXwYutyMOtJqamuQ+oZEjRyrz48pHCfDfjY2NbfcElUpGFSmYZBVlQTlnzpwJIeNkLrjK2w9CBu9d5CTCw7uncLCG7n8bItm4cWOQD3QhyETMWB+SSXUdIRsukl1DQ4PVbYchjR7lzyIrDoApU6bA8ePHoba21phE3BchusiadiWKL3l89L+sU+j+T9NhwoQJMHz4cMBL8IjsSold2clEl9JA5wmUEgxqW968Z7uZTzW4Sy1vFlmRvK+99lpYv3691uMqhdwuspoOj5ZCviz9r\/JMslyz61ufAwcOJE1WVlYmV9h0KTIR53TyxV2+gTa1J3siLl\/JrBv6TDLpfneVlc5KPfLII8lJ7pCk7SIrkklzc3OiduhYmoucKB99EHE6MWnSpGTgFuWRya6UchXGM+kMZIIDYOnSpUEDsC6Gj0Y\/d+5cuPfee6FPnz6psaBSGJ2LrBTToaAr1p06dWoQbF3kpODrokWLkpiJixdbCozlNrskmdAXslyrDGInuLjjVK8cRILvdpEVy7722mvJl7McOLvIKk9zQsobi5w2ZNSlyAQBEac1RQnAivchmwKw5Yzgy1OwNFnFJWzREEO55i6yynqEtAsXOctJekwmCgRRkw8BAAAAZElEQVRiXhoO6X6rjMdlGVOsH\/JLT+91kVX+ooacPrjIqZrmhJqOMZloEIhp0xoFB3G6oPvaF2WDlShruckE35+2GUyWVdy0FnozmIuc4l3aoeU0EUqXm+aYAOHfGQFGoPgI\/D9ZCpr+ZR2paQAAAABJRU5ErkJggg==","height":166,"width":275}}
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
