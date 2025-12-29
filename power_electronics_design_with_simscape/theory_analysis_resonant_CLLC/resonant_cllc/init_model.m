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

fPWM_LLC = 15e3;
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

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:02159090]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:5d197f8f]
%[text] ### AFE simulation sampling time
dead_time_DAB = 3e-6;
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
%   data: {"dataType":"textualVariable","outputData":{"name":"fPWM_LLC_pu","value":"0.7500"}}
%---
%[output:02159090]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"458.3333"}}
%---
%[output:5d197f8f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"937.5000"}}
%---
%[output:53fa3074]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"1.2784e-05"}}
%---
%[output:215718df]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"4.9535e-06"}}
%---
%[output:4f58960b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"391.9184"}}
%---
%[output:9d2fb161]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"509.1169"}}
%---
%[output:0bd29de1]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.3451"],["81.4559"]]}}
%---
%[output:915851da]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.0001"],["-9.8696","-0.0016"]]}}
%---
%[output:46c059b7]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.0156"],["2.7166"]]}}
%---
%[output:7459e256]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.0000","0.0003"],["-24.6740","0.9961"]]}}
%---
%[output:4579d87c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.3888"],["67.9152"]]}}
%---
%[output:74e07718]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.0911"],["4.7089"]]}}
%---
%[output:0825f5ce]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAXgAAADiCAYAAABEF9w3AAAAAXNSR0IArs4c6QAAIABJREFUeF7tfQt0Vsd17saBK5TEvkaOLCQwKK3Bcb1a+VEqTOLlm6au04dwn0Hi3iUuxpjVEofWvASO48u6BiRAzSV+pJTIKrQLQeuaBTRd4dqshDghSR3Zpk1qG5xEjkGIyGDqOAbdOOWufej8GY3O+c\/M+c\/Mmcc+a7HsX2fOzOxv7\/2dffa8xl28ePEi0EUIEAKEACHgHQLjiOC90ykJRAgQAoRAhAARPBkCIUAIEAKeIkAE76liSSxCgBAgBIjgPbaBs2fPwqJFiyIJe3p6oKamRou0e\/bsgTVr1sDGjRth3rx5URv4N7zY7zwbjpPr\/PnzsH79eliwYAHMmDEjz+YS6zp+\/DgsXLgQ7rvvPik5s\/Txm9\/8Jhw+fBhWr16tRSaG5dGjR6P6d+3aBbNnz5ZuK0730g9rKog4d3R0QENDgzbcNHU992qJ4HOHlCrU7fQiwVdXV0cO3d\/fD729vcYIvqurC5CAZV6ejHRU+oh1z58\/H5YsWaKNqFgb\/MtZxYJ161qlL3xZ7Ncjjzxi1B6y9lXnc0TwiuiiU2\/bti16CiMEnlCYsd9\/\/\/1w6NAhwKhILCNGTLzzsojwlltugcsvvzyKpvBKcz7WrtgnkQjPnDkTRZwswsXIkNUtWwd+BYikwDs59gGjeXa1tLRAZ2cnIAnjxYjuhz\/8YYkY48iPYTE4OBg9x+PEy\/Xoo4\/Cpk2b4MCBA6U2Uaa5c+dGpC\/+nf+iYLpEHW3ZsgXw97Rp00r9FfvA64HdQ\/lYdC3qlul+6tSpiX3hcUcBGF5oO0ju7Gpqaorwwgu\/yljEnUb+DFuGA6sH9Si2zd\/j3aKczfK6HxgYiHxDtHlmL6Is2AeGo2iTd9xxR0lOxOSuu+6Ce+65Z8xXIrM1sc04\/Si6uhfFieAV1MiTO\/8Y+6wVHSbNOdl9RhwiobD7ovGKkQpPqDzJX3XVVaNSNIzgGWmyep9\/\/vlRpFyujkoJHutmODHc+BcbvgxOnDgRvYhYP8WXBZIWSz0lETwjGx4rHsc4chseHgZ8uZbrg6hrpjuRSHndJ\/Vx+vTpo0ictwfxHpLv5s2bYeXKlSVyF+1HNOWkPiXpPY7gRXJnbbAXS5LNsxdVki7Z86LNY98+\/\/nPwxe+8IVRL+fbbrsNnn322diAJO7FYSo9qUAfhRQlgpeEnRlibW1tKfJkkQkz5v3790dEyX5j1SyKZNE4RmUiKbBolhEwPodfBnzkF5cbZUaMxMS+JPiIikVBWB9Gf2L9GDWp1pFG8Bghp322i9GVWJ69SOPIE3GYOXPmqBeXTIqG1ck\/j1GwSNiiLtl9hhOL8D\/3uc9F0Sq7H\/dlwpuWTIpGTMkk\/U6yH3GMRbRPxIlhLRJ00leiWF4kzqeffnqUzfMv37jUVdLLnNk82iTrN3vhMP3iVwj\/dcZ\/BcalmlDn+IzJtJ0knRgrRgQvCXUcaYlOzYydJ2Pe8LApMdrmo2X8f4xcWRTJO2QcwYvOwtIgTKSkFA1fv2odeRA8jxuLbpmzYt\/jBoZ5HMUXVzmCFyNMxBG\/bESckwhcNA8kHdZnMZ+elG7B\/pUj+KR0lEjwSdFy0hce\/1JjA6einCwoSSL4uDriviDTXjril4AY4cfZPN+nOP2zNBXfHz5lldZ3Sdd3uhgRvKT64iKEJIJPMswkgse\/JxGPmM7gu6tKziyCr5TgxZdd2u84iNkzDz74YPR1wXLZSZGwKsGLzs3\/roTg+RRC0oBp3DgN+xrjn5GN2NPSIcx+xNkvcbZjmuBFm2MpGzEVlhfB82M+RPC00EmS3qE0GJRnikZsPI6wyxE8HxWxCJ8njcWLF8fm4Hlnkq2DpYH4tJE4QJv0Ow5kMWrlv1AqTdGIYw\/8J37WFI1qugXL8y8+NujLE7xIQGI6JC1Fk2a8eaZoxLQjk4ON3yRF8Oyrlt0X+yQSPuoqS4omDgsieCL4NB8ZdV\/XIKvM52rS\/OS4z3b2yZ40yMoTPE9EvLDlZoCwcmkEj+WSZmbgPYanWCZpsJnhJOZ5eQLHetvb26OByLhP+KQBceyDzCAri6ZF8kgajEzCEevBi83Iiksz8LNPsJ6tW7fCww8\/PEYucaYSqyttkBXz3WnjJbKDrGkELzpaOZuP67fMIKv4JUM5+IIIHh141apV0fS2uEUp4rSqcrNIlBg6h8J5T5PkyU01gmfixOWZ8XNdJgefVgfe5wkX+4tfBp\/85CfHzGhgTs6TQjmCLzfPW3aaJBvI420GyfP2228vzVBBMrn77rth6dKlpVSQ+ILBaZIrVqwoO02SJ9K4lF0cGcaNx2Db2Ef2hcWm0z722GPwxBNPABuP4F9c4kubvbzK4YvtlJsmKX5lJC1KS8qf82NESQQvvnwRD5yeywY\/sQ\/ieAj+jW+T16e4mI4f0+LviakocXwqBypwogrjOXiZ2QRIWDjNTdfqPV2aSYuGdLVL9aohwJOeOEVV\/LpJqpkRCL5IXbNTNbTMl2Yvd2w5bnaYzOpomgd\/SW\/GCR4jH4yC8UqK4PF+Y2Oj1PJv8+aX3CIRvE3aKN+XpHRb2qIyvlaVlazuIFN8T9PSXTJbUaAv0kpWwwSPilu3bh20tbVFJB9H8Cy6wjIqe2IUb5ajB9Vkogwb+hxqH+LyvGmrQkWsWJSI6R3XbNV2vceNw8juk0N70fxcu0YjeHyr4nXzzTcn5uDj3t4qUZXthkv9IwQIAULAFALGCB7fyDt27IAHHnggWoqeNMgqRkVpURLWhf\/oIgQIAUKgSARwCjH+s+kyRvCYksGZDfgpmzaLRgSI5ezFwSwkdtyb41vf+pZNmFJfCAFCIEAEmpubo\/2CbCJ5IwSfNGiCNiCTV0sadGVT1RDUKVOmeG1S+BLDedC+y0py+mXGoelThs9MatgIwccNTiWlaNgsG35+M85PjtswiBG8baDqUGAospKcOqynuDpJn8Vhjy1bQfBoBH19faV9uMWFTkkEHorxoKJwXQCOYeCJRTiF1NeL5PRLs6Ho01YuKoTg8zJhW0HNSz6+ngsXLsCpU6egvr4eJk6cqKMJK+okOa1QQ26dCEWftnIREXxupqy3olAcheTUa0emaw9Fn0TwGizLVlA1iAqhOArJqcN6iqszFH3aykUUwRdn+0oth+IoJKeSWVhfOBR9EsFrMEVbQdUgKkXwOkAtsM5QiC8UOW3lIorgC3RylaZDcRSSU8Uq7C8bij6J4DXYoq2gahCVIngdoBZYZyjEF4qctnIRRfAFOrlK06E4CsmpYhX2lw1Fn0TwGmzRVlA1iEoRvA5QC6wzFOILRU5buYgi+AKdXKXpUByF5FSxCvvLhqJPIngNtmgrqBpEpQheB6gF1hkK8YUip61cRBF8gU6u0nQojkJyqliF\/WVD0ScRvAZbtBVUDaJSBK8D1ALrDIX4QpHTVi6SiuDL7eee5CNNTU2wd+9erS5kK6g6hA7FUUhOHdZTXJ2h6NNWLpIm+OXLl8PatWtB5kRzPLFpw4YN0R7uOi9bQdUhcyiOQnLqsJ7i6gxFn7ZykRTBF2ce5Vu2FVQdeIXiKCSnDusprs5Q9GkrF0kRPEvRDA8Px56sVJT52AqqDjxCcRSSU4f1FFdnKPq0lYukCB7N4\/z589DR0QEHDhyIrGXjxo0wb9684iwHAGwFVQcooTgKyanDeoqrMxR92spF0gTPmwgTBv\/W0tJSOmrPtBnZCqoOHEJxFJJTh\/UUV2co+rSVizIRPDMXPqpvaGgwnr6xFVQd7hSKo5CcOqynuDpD0aetXFQRwfNmw2bOdHd3Q01NjRGLshVUHcKH4igkpw7rKa7OUPRpKxdVRPAUwZtznFAcheQ0Z1MmWgpFn14RPOXgTbjG6DZCcRSS07xt6WwxFH06T\/A0i0anG6TXHYqjkJzptuBSiVD06TTB0zz44l0qFEchOYu3tTx7EIo+nSb4PBWeZ122gpqnjKyuUByF5NRhPcXVGYo+beUiqUFWjOBpL5rinARbDsVRSM5i7Szv1kPRp\/MEv2jRIjh69Ki0\/mk3SWmopAqG4igkp5Q5OFMoFH06TfC2WpOtoOrAKxRHITl1WE9xdYaiz6+9eg4+8an\/BU99phVmz55dHOBCy1IpGmt6K3SECN5WzWTvVyiEQHJmtxEbn+x7bgiW9r0E\/\/SJaiL4vBREBJ8XkvbUQ8Rnjy7y6Eko+iSCz8NaKIKH+vp6mDhxogY07agyFEIgOe2wt7x60XXwB9B1cIAieFVAu7q6okdWr1495lGK4FXRtL88EZ\/9OlLpYSj69Irg9+zZA2vWrIn0vGvXrui\/fX19uW8bzAh8yZIlRPAXLsCpU6cogldhF4vLhkJ8ocjpDcFjRD04OAgPPfQQrFu3Dtra2qJBhXKRdhY\/w60R1q9fD9\/97nej+imCJ4LPYke2PhMK8YUiJw6wYh7e6UFWfsHT1KlToxOeGMHnvV0wfiXgNTAwQCkaWuhkK09n7lcoxBeKnH+47Sh8+ZWz\/hI8plMwiu\/p6al4P3h8keDXAX4lbN++nQieCD4zkdr6YCjEF4qcLY+9AF\/\/3jm3CR6dBSPrI0eOjErRzJw5E3Cla2tray7ntOKL4vbbb09N\/bAc\/bJly6C5uRkmT54c\/fPxQkcZGhqCuro6qK6u9lHESCaS0y\/VhqBP9Mvf+9tT8MOzF9wneDQ\/fj94Zo55HcKNqZ4dO3bAAw88EBGZzCwa1ocFCxZAe3u7Xx7yn9KMjIzA8PAw1NbWQlVVlZcyolAkp1+qDUGfvX\/\/RXh06IZIcU7n4E2YHj9Dh28PD\/feunXrqC6wF83mzZthypQpXkfwOOh8+vTpSEaf58GTnCa8zFwbIejzH77+Mtz3xTeJ4LOYlUwEj1M1bdr\/IYucac+EksskOdMswa37IeiTTZF0PoJnB3+k7SqZNG89i2kSwV9CLQRHITmzeIjdz\/hut5h3v\/Hhb0RKuOydN+Af\/+c1VgWbypuNYQpl9+7do2bLMOJng6x5z4lPMmFayWq3c2fpne+EwDAhObNYh33P8AT\/3hf+Gp7c+CfuEjwjclx0JKZE+GmSZ86cgQ0bNkBvb69WjRDBa4W3kMqJ+AqBXVujPutTjN6v+L+ro5X9NqWLlSJ4InhtfpBasc+OwgtPcqaaglMFfNUnkjuuXsW57yz3Pn\/+fLcJHgWRSdGwufLirJe8LZMi+LwRLb4+XwlBRJbkLN7WsvZAJPdpNRPhL39jHHhB8AhK3Dx49mmC91asWBGlZ2bMmJEVQ6nniOClYHKqEBGfU+pK7axv+kRyn\/v4C9GiJryQ3F\/89K0lTnQ6RZOqTcMFiOANA26gOd8IIQkyktOAMeXYBBL6O\/\/vZzBn0z+XamXkzge9RPA5gk4EnyOYllRFxGeJInLqhuv6ZJE6mwrJYLltxiTY9yc3llCylYuUBlmT0jNMyqamplw2G5O1LVtBle2\/SjnXHUVWVpJTFik3yrmsz6deOA33\/M2\/jQKaj9r5G7ZykRLB47Jj3CJ4zpw5MHfu3NJ2wXlvNiZruraCKtt\/lXIuOwrJORYB0qeKVZgrKw6gspaR2B9tvR4+cu2VsZ2xlYuUCJ7fDx4HUHFBU2NjY7SDJAqo41Sncqq1FVQd5kiEoAPV4uokfRaHvdjysdPvwPInXylNeeTvI7Hv\/9ObosFUF7moIoLH6ZB4IAcufMr7wA8Z9RPBy6DkVhkiPrf0ldZbG\/X5tVfPAe4fw+awizIgmX\/8hg9A5+\/LzwK0lYuUCB6B4Lch4KP2\/fv3R\/vEd3Z2Gtuv3FZQ04w+y30bHSWLHGnPkJxpCLl13wZ9vnb2AnySW5QUhyCS+h3XXwX3fXRaarQe97ytXKRM8OJqViT8bdu2QUNDg5G57zy4toKqwwVtcBQdcol1kpwmUDbXhml9Yg4do3P8b1KEjtKzlMsjrR+C266dVDEgtnKRMsFXjESOFdgKao4ilqoy7Sg6ZJCpk+SUQcmdMrr0iQT+4wvvQsfe42WJnCGFhH7NpInw2T++Dq69+r25A2grFykRvDjIyqNEOfjcbWZUhbocRW+v1WsnOdUxs\/mJSvWJRD7y7n\/A\/zn0WmpUzuOAhH7nL30Alv63a6I\/pw2SVoqh9wSf56HbsmDbCqps\/1XKVeooKm0VWZbkLBL9\/NuW0SdbTHTw387A\/qM\/gtffvFDaCiCtR4y4\/+jmOmif3aCdyJP6YysXSUXwcXvPxAma50EfaYrF+7aCKtN31TIyjqJap43lSU4btZK9T0yfP62aFB01+cxLZ2Dvi2okzqdZPvyLV8Kq32yEcePGFUbmcWjYykVSBM8EKpeiyW4C2Z+0FdTsEiU\/ScSnA9Xi6vRRnxiJvz3yLjx++AS8dua8VG5c1ACLyD\/2oRpY9uvTjaRX8rACW7lIieDzACLPOmwFNU8ZWV0+EkIcTiSnDuupvE6WRsENt7Y9ewJe\/dE7SqkUvgeMxD9y7SS476PXQPWE91gVjWdBy1YukiJ42bNYaS+aLKYh9wwRnxxOrpSyTZ+MwP\/pO2\/AF\/91OIKx3DTDNJzZrJXrrq6C3585AWpra2FmQ\/wy\/7S6XLjvNMHbCrCtoOrAyzZC0CEj1kly5o8srtxEwn3y+dPw5VfOVkzeWAEjcPzvn31sOlSNvyw2Cg9Fn7ZykVQEn7\/J5VOjraDmI93oWkJxFJJT3now6sZ\/UydVwWefeQ2+\/8b53MgbK8J54x++9kqYP6s+qjfLVMNQ9GkrF2UieNyDZs2aNaMscePGjdGmYyYvW0HVgUEojkJyXrIeceog\/k1l+mCSDTKSRvK+vv59sHDOFDjz9k8j8s5C4Gm2Hoo+beUiZYKXOZM1Tel53bcV1Lzk4+sJxVF8lpORNup193OD8PR3TsP48ePh1I\/flZ73Xc62ePKe1fhf4X8018P4yy5NJ8S2dRB4mq37rE9edlu5SIngxX1oRAFxX5qenh6oqalJ03su920FNRfhhEpCcRTX5ORJ+60L78LnD79eIutKBil59fPEPb1mInz0uhqou6KqFHUXRd4ydu6aPmVkiitjKxcRwWfVqOHnQnEUW+RkxH3x4kX4238+Bd\/8\/r9HGs8jTcJMp+GK8VEEj+kStrT+xmsuj24XEW3rMGlb9KlDNu8ieBSIUjS6TSW+\/lAcRaecbDbJf1y8CPuPDsMzL5+BixfzJW2enBlxt86aDNNrqkuKRfLWKWcxFhq23XoRwTMV0iCreRciQojHnEXax07\/BL5y7E04euLHuUfacaSNOxL+4U11pdy2ap6b9Gneh3S26AXBsxx8a2ur8RkzLuW9dBhSKITwzHdOQfXP3o4Wxrzyxk\/h0Mtn4XvD2VdNpumCz2\/fPnMS\/MFNdTB4bqSUItGVKglFn6HI6QXBo7OIG4\/t2rULZs+eneZHWu7bCqoOYV12FBZl\/8vJH8PhY2\/Cy0M\/0RJli5E2\/m7+4KXZJJeNG2dVbttlfarYdyhy2spFSoOsomL5VA2d6KRi9uplbXAUNluD5bJPvzUCB\/5lGF54XU9ahKHER9nX1b0vWnxzy7QrRuW01REt9gkb9GkCgVDk9JLgeQPBKZIoJE2T1OM2Oh2FRdhnfvJT2PPtIfju4NvaIuy4KPvmaVdA++x6mPCey6LBxwkjb0J9fT386B1\/ZpOIVqFTn3osMFutocjpJcGz81hR9WkR\/Pnz56GjowMOHDgQWUq5vePFNFBS3baCms0Vyj8l6yiMrPG\/1RMugyeOnCzNy85zih\/fWz7Cxr\/jIpu7mmph0nsnKEfZsnLqwNhknSSnSbT1t2UrFymnaMQZNLI5eHwZ4LV69WpIG6zFNgYGBqKy5S5bQc3TnBhhR4Rw+kfwzMDP4MXBC0YjbJybfVfT1XD15f8lalfXwCPWTcSXp\/UUX1co+rSVi5QIPo2YVcyJJ3zxObzX2NiYOlPHVlBlcWDk\/YMz5+Hvvj0URdo6omwxwr7pmsuhbVY9vK\/qPSXCtmU1ZCiEQHLKeokb5WzlIiWCzwvqclsesFROW1tb6uwcW0HlcYpOf8dTbr5yaQl7HsvXRcK+oeH9cPecKTBxwmXaI+y8bCCpHiI+3QibrT8UfdrKRcYJnuXtW1paoLOzE6qrf77CD00v7nCRpJ0qbQMVCfwvnnktmredhch54sb9RvAgYXZd\/V6A\/ldeh1uuuyY629LXKxRCIDn9smDbuIiha5zgWcNI9IODg2NI\/vjx47Bw4ULYsmVLFMGLv3mzYKAuW7YMmpubYfLkydE\/E9fgW5d2APzXwZ\/AQ\/\/4fakmce+R+svHw+\/+Si381g0fKD0jk9NGQhgaGoK6uroxL0Wpxh0pRHI6oijJboagT\/TL\/v5+WLlyJciOSUrCV3GxwggeiXvVqlWwadMmmDFjRllBkvL14mybBQsWQHt7e8WgpFWw\/6W3Yd0zbyQWQyK\/ZcpEuPfXLh1Rhr8rvUZGRmB4eDha4VlVVVVpddY+T3Jaq5pMHQtBnzt37oQdO3ZE+DhN8Jg+Wb58Oaxdu3YMKSNhb9iwAbq7u6W2C0Zylt1eOGnQlRH85s2bYcqUKdoj+JbHXoD+k5dmsPAXRuB331oPvzLl\/TC78f2ZHCHtIRybOH36dCSjzykakjPNEty6H4I+MYLft28fbN261V+CTyNsPgpnA6k4v12cCinWg79XrFgBvb29Y14qJvJemIbpOvgD6HtuaAypP9p6PXzkWjMHCVPO1i1iS+st6TMNIbfum+CiLIhIpWjEVEhSQ+UWL4kLnfhBVqy\/r6+vlI+X3e9GN6i4JH\/u4y+MIfb9f3qT1rngcfgSIWQxb3ufIX3aq5ssPdPNRVn6hM9IETyrvFyKJmsHKnlOJ6gYtXcdHCh1D9MwL3761kq6W9GzRAgVwWfdw6RP61RSUYd0clElHVMi+Eoa0vGsDlAxJYNRO1uEhMReRMQu4kWEoMOCiquT9Fkc9jpa1sFFefRTiuDZ3PTFixfD9u3b4ejRo7FtNzU1Ob3ZmK3kjmATIeRh7vbUQfq0Rxd59MRpgs8DAB115AlqHLkXmZKhCL7e69lCRPA6GKG4OvPkojylkIrg82wwz7ryAhXJfWnfS6XVp0Xn2+MwIkLI03KKr4v0WbwO8uxBXlyUZ5+wLiWCj9tGgO+QqymaR7\/yOnxm\/6uRKDaSO6Vo8jb74usjgi9eB3n2wAuCTwIEp0CuX78ecCVp2qpU20DF6P3Gh79RIncbBlQpgj8VHfjh84IuIvg8maD4urwmeIRXnMtuAvJKQeXz7rbMlknCjQjBhEWZa4P0aQ5rEy1VykW6+qiUoinXCdWtCvIQqFJQ\/\/zvX4Ed3xiMuvLZP74OFtzakEe3tNRBhKAF1sIqJX0WBr2WhivlIi2dUs3Bl+tE0u6QujrOvhrmz5+faf8HMTVj04wZStFQikan35isO5QXmRcEX26QNe1MVh1GVQmouHEY27MdyV1my14dMsjWGYqjkJyyFuFGuVD0WQkX6dRkbikanZ1MqjsrqHz0\/uFfvBIOLL2piO4rtRmKo5CcSmZhfeFQ9JmVi3QrUJng2aZhc+bMic5MLbczpO7OZwWVj95x1oypHSErwSMURyE5K7ES+54NRZ9ZuUi3xpQJPu7wjaJIPguoLkbvaAShOArJqdvlzdYfij6zcJEJTSgRfJ4HfuQhXBZQXYzeieDzsBa76giF+EKRMwsXmbBIZYJftGhRdEgHnpfKXyig7AlNeQmWBdSa+78cNW\/ritUkbEJxFJIzL++wo55Q9JmFi0xoSIngsUN79uyB3bt3j9o1ks2uaW1tjfLypi5VUC8CwFX\/SfCPtV0PbbPMHNCdBx6hOArJmYe12FNHKPpU5SJTGlImeOwYE4bv5MaNG42SO98P2YNuWXrGteidUjSm3MFcO6EQXyhyekXw5tygfEsqoPLH7y3\/jenwwG\/\/gi1iSPUjFEchOaXMwZlCoehThYtMKk8pgmezZdra2sbk4E12mrWlAmrH3uPwV8+eiB51ZWokj2kojkJyFuFJ+toMRZ8qXKQP7bE1KxG8q2eyurYtQZwBhOIoJKdJ99ffVij69ILg2SDrwMBANJOm6EsWVJ7gn7y3CX79QzVFd125\/VAcheRUNg2rHwhFn7JcZFpZyhE8TpN07UzWL3ztJKx66liErQv7zlAET5uNmSYCXe0RwetCVq5eJYKXq9JcKdm3psuzZxiaoTgKyWnOf0y0FIo+ZbnIBOZ8G94TPJ+e+bOPTYfP\/I5bs2eI4E27hJn2QiG+UOR0muDZQqbFixfD9u3bnUrR8NMjXZw9QwRvhnBNtxIK8YUip9MEb9r4ZduTAdWH9AziEYqjkJyy1u9GuVD0KcNFRWhMOUXj2nbBbO8ZV\/Z9TzKCUByF5CyCBvS1GYo+vSF4l7YL5vPvru09I7pcKI5Ccuoj2yJqDkWfXhC8a9sFdx38AXQdHHB6eiTl4IugJf1thkJ8ocjpDcG7tF2wL\/l3ysHrJ1zTLYRCfKHI6QXBoxO4sl2wqyc3UQ7+Apw6RQudTL9wdLVHBK8LWbl6lQdZsVrd2wXz9Tc0NEBvby\/MmDFjjETl3po8wa++sxFW3\/lBOUQsLRWKo5Cclhpgxm6Fok9vIviMepZ+7Pjx47Bq1SrYtGlTROpxXwyssnKg9j03BEv7XoqKujz\/nXLw0qbjVMFQiC8UOYngM7qfSPh8NeVA5c9ePfsXH83Yuj2PheIoJKc9NpdHT0LRJxF8RmvBCP7IkSPQ2dkJ1dXVo2opB+qND38DME3j4ulNcVCF4igkZ0ZHsfSxUPRJBK9ogBi5L1y4EAYHByHpSD4G6rJly6C5uRkmT54c\/Rt861341c7nohabp78f9i75ZcXW7SuOjjI0NAR1dXVjXnT29TZ7j0jO7NjZ+GQI+kS\/7O\/vh5UrVyZyVVG6yTTIarKzjOi3bNky5hQpcbB3wYIF0N7eDv0nL8C9Tw1F3bz3166EJc1XmuwxXVH9AAAOp0lEQVSylrZGRkZgeHgYamtroaqqSksbNlRKctqghfz6EII+d+7cCTt27IhAkz0fOj+Ey9ekTPB8ZC1W3dTUBD09PVBTk++BGnGrZ7FtRvCbN2+GKVOmlCL47kOvQ\/ehS8fzfbtjFjRcMd4UntrawS0iTp8+Hck4ceJEbe0UXTHJWbQG8m0\/BH1iBL9v3z7YunWr2wTP9qHBqYumTnQS977hzS8p7+XbACvKHEouk+TMl2CLri0UfXqRgzdxJqs4awaBW7FiRexc+DSC92WAlQi+aJrKv\/1QiC8UOb0geBZNt7W1jcmH5+kCYm49bZCVv+\/bClaGayiOQnLm6UnF1xWKPr0geDSXctMWTZtTHKi+rWAlgjdtVWbaC4X4QpHTC4JnJzvZfOg2v4OkDytYieDNEK7pVkIhvlDk9ILgTTtBWntxoPYeOQnLnzwWPfrip2+NFjr5cIXiKCSnD9b6cxlC0ScRvAa7jQPVpy2CechCcRSSU4OjFFhlKPr0iuAxD79mzZrIbHCAE6++vr7Y7QR02lYcqGyLAteP6BNxC8VRSE6dHmO+7lD06Q3B46Ij3D7goYcegnXr1gGbUZO0GEmnScWB6ssZrETw9V4v6AqF+EKR0wuC5+fBT506FTo6OkoEj\/PXN2zYAN3d3bmvZE16SYigfu3VczD38Rei4q6fwUoETwSvMzgyVTcRvCmk49tR2qqgHMEj2WIUr2OrgiwE79MMGpQ\/FEchOYslhLxbD0WfXkTwqHw2D55P0cycORPwrNbW1laYN29e3jaSWJ8Iqq9TJIngjZmUsYZCIb5Q5PSG4NEDxJWm+LeNGzcaJXe+H2wlq68zaIjgjfGusYZCIb5Q5PSK4I15QUpDIqhE8LZoJns\/QiEEkjO7jdj4JBG8Bq2IoPo6g4YieA3GU3CVRPAFKyDn5r0ieH4ePMOpiI3ueVAbZt4IOAcer9V3NsLqOz+YswqLrY4IoVj8826d9Jk3osXW5w3BI7nv3r171GwZtkdNkYOsPMH7NkWSIvhinVdH60TwOlAtrk4vCJ4ROR72MXv27FFoFj1N8vC\/10HXwYGoT75NkSSCL85xdbVMBK8L2WLq9Z7gi17o9Oq46fCpPS9H2vVpkzFmrkQIxTiurlZJn7qQLaZeLwgeoUNBxBOWbEjRrO+vgq9\/71y0eyQSvG8XEYJfGiV9+qVPLwg+bT94XmV4APfevXu1apEHlQheK9TGKifiMwa1kYZC0acXBG\/EIhQa4UH97b87Hz3p2y6SlKJRMAiHioZCfKHISQSvwfkYqF2PPQFLnr4YtdA2a3K00ZhvVyiOQnL6Zbmh6NMrgo+bB1\/kVgVL\/\/fjgCkavHycIolyheIoJCcRvIsIeEPwNs6D5wnexymSRPAuunz5PtOLzC+dekHwts6DJ4L3x1mI+PzRZUiBCRG8BrtloNbM\/Qx8\/7LpUQs+zoEPyVGI4DU4SoFVhqJPLwge7cTGFM3bH1kF737gOm\/nwBPBF8hQmpoOhfhCkdMbgmckzw7dZvZf5CArEbwmFiqg2lAIgeQswLg0NukVwWvESalqBuq53+uJnvN1DjxF8Epm4URhIngn1CTdSSJ4aajkC4oE7+sceCJ4eZtwpSQRvCuakusnEbwcTkqlENTWez4Fb\/1mV\/Scj\/vAM0CIEJRMw\/rCpE\/rVaTUQSJ4JbjkCiOon\/jUOsAcPF6+LnKiCF7OHlwqRQTvkrbS+0oEn46RcgmR4H2dIkkEr2wa1j9ABG+9ipQ6SASvBJdcYTEH7+sqViJ4OXtwqRQRvEvaSu9r8AQvbjXc0tICnZ2dUF1dPQY9Bha70dDQAL29vTBjxoxRZUWCpwg+3RBtL0HEZ7uG1PoXij6DJvjz589DR0cHzJkzB+bNmwfsNxI3Hv8nXriYamBgIPYeX5ZP0fh60AeTNxRHITnVCNT20qHoM2iCjzNCJPEjR47ERvFdXV3Q2NgYvQzKXXwETwRvu6vL9S8UQiA55ezBlVJE8IKmkgieRfdtbW1jDvYWlc0TvM+LnCgH74qby\/eTCF4eKxdKEsFzWip3hmvcsYBJ2yDwKZrGiW\/DU\/f+MkyePNkFe1DuIxLC0NAQ1NXVxY5bKFdo6QMkp6WKyditEPSJftnf3w8rV66EXbt2pQamGaHM9Ni4ixcvXjoKydDFInRsLm6Q9fjx47Bw4ULYsmVLBJT4m+8mH8FPfHkfLGmeBO3t7YYkMdvMyMgIDA8PQ21tLVRVXTrcxMeL5PRLqyHoc+fOnbBjx45IcUETfBq5J5k25uTxEgdkeYL\/7zdMgOUfm+ptBI+DzmhECxYsiMYnfL1ITr80G4I+MYLft28fbN26NVyCT5s5U86skwZdeYL3eRUrYmNrji9vOiI580a02PpIn8XibyxFgyQ9ODiYOPedwYAGgWV7enqgpqYmIrYVK1akzoN\/4JaRaDdJX6+TJ09GOb5ly5ZBc3Ozr2ICyemXakPTZ5ApmriBUzTjpqamiMiPHTsGfX19JfIXFzolgXbixImI9A5NuB0wBz\/+jVf88g6ShhAgBJxBAAOvzZs3w9SpU63ps7EIXpfESPL4jy5CgBAgBIpEAIndJnJHLJwn+CIVSm0TAoQAIWAzAkTwNmuH+kYIEAKEQAUIEMFXAB49SggQAoSAzQgQwdusHeobIUAIEAIVIEAEXwF49CghQAgQAjYj4DTB44Zla9asifBN2q\/GZvDj+oZrALZt2xbdKjenli+XtF++zbLLyslkELectlk2sW+ysorTiW2bU52GuaycbPsRXBfjou2Ww0F2J9w0LPO67yzBo5GsWrUKNm3aFGHB\/l88FCQvoEzUwy\/ywrUB\/IIvvn1xJ078vXv37tLiMBN9raQNWTlFmfFl7tqLXFZWcaU3b98u2LSsnOwlhtuO4F5TrtluGrljcGaTjTpL8CLJ2fbmzEKA\/J47Ktsmu0YGqnIiKSxfvhzOnTsHra2tqecEZMFe1zOysqION2zYAN3d3dEKbtcuFTn5YMw1243TC3tpXXnlpZX0H\/\/4x62xUWcJXtyALGlDMlccJenUK3YKVjk5XHKSLHKibmfNmhVt6CSDhy06V5G13AE4tsiT1A8VOeMi+KSDf2yXm\/UPZXrzzTejdBN\/cp0N\/Xea4PlTn2SP+bMB9Lg+xEXssl8lsvv82CC7qpz48sJdNO+\/\/35Yt26dkwTPH16TpFNmv6gjmTEYG3TJ+qCqU1b+wIEDsGTJktSjOW2StVxfbBwnIoK3xHpUnYR1G4nhkUceid2MzRLRRnVDRU4su379+miLZFwCblt0lIaviqxswgAbWC23yV5au6bvq8gpnu8gbi5ouu95tkcEnyOalKKBaIDKJXJH9at8zqPzHz58OIrwbHSeNHNWkVVM0bgkbyhyquo7rbyJ+85G8GJKRjadYQLUrG3wMqQNsro8+0BWTn7aHY+pS5\/1srLiy4zfUTVN\/1ltTNdzsnK6\/CJLw87Gl7KzBB\/yNEmXPt\/jnEJ2Sh3\/rI3Ok+bweF9WVnHw0bXUhayccSmapPMeZPC1qYyNNuoswaNiQ1roxH+xJEW2Li2MSVoUkzRYbqPzyJKLrKz8QicXFwDJysmf9+CinEl6t9FGnSZ4WQejcoQAIUAIhIgAEXyIWieZCQFCIAgEiOCDUDMJSQgQAiEiQAQfotZJZkKAEAgCASL4INRMQhIChECICBDBh6h1kpkQIASCQIAIPgg1Vy6kuBCnXI26F+nw0wlbWlqgs7MTqqurU4UU55qnPmC4AJtm2NTUpHXrZ34vGBX8DMNBzeWAABF8DiCGUIVNBK\/SF143LhA89he3ZjBxubyDpQl8fGiDCN4HLeYkg3iiENsSgF+YwqJLjJhx8y\/cEZBdeNDB3LlzR\/2dHX7AR41YPi1yTFoMwy9uw3riFnclycH+jgdNsB0bxWiZbxfr5+9j28PDw3Do0CE4evRo1Dbe53F48MEHYf\/+\/dFBNHhQh4rc4v5KKjtMxr280gg87X5OZkXVFIgAEXyB4NvUdNqGUXzUjP1GUsNViCza5Dc+Yzs\/sm1y41b4ldu\/X9yKIe43v28Lj2M5Oe644w5YtGgRTJs2LUrriHKI7fAvBJQzbnM3fqtmVl9\/f3+0u2fcDpjl5I4jeP4UK3GZf9rXSRqBp923yT6pL9kQIILPhpt3T6Uts05Li\/B7A4kEH\/csO6Vp7dq1UaTLrqR+8ORXri\/l9nDJEuXy7YqEGCcD\/5I4c+bMqA3EUMYkufFeHMGLh2EkvSCyyEYE750bjxGICN5\/HUtLyKcnxBRKEqny+4+wfUVEghfTKqxDcfuQJOXJ+T1qyhF8OdKSJUH+UGjsK0tViXXHHbPHv+ief\/750qHwvBKS9l9JStHwOfkk+WRl4\/tBBC\/tGs4WJIJ3VnX6Os4THJ+H59MijNjZi+DEiROlg8\/jCF72WLYiCR5lWLhwIQwODpZy++UieBmCl5U7KYIfGBgYNehKBK\/P7n2smQjeR63mJJO4BSwjeEyj4CHYfHolLUWDRNnT05N6oHSRKRocHBUJtdIUjazclKLJyWipmlEIEMGTQUQIpA2E8mkRLIuDlZg6wBkpLOrGGSb84KI4yMoPypbLlec5yMoT5+LFi0f1G+\/xETESPB9xs9RSUoqG1Y0RPz9oKw6yysqdNMjKZgqVO1yd6Y+1xXTCBpTj1glQisZ\/5yeC91\/H0hKKuWc+Dy+SOA4gzp8\/P6obSWXLli3RIGFrayvMmzevtFc\/I0dx6mLaYp5ye4anDfiKbTE5xBeTSPD4m5\/yiH3Hg913794dfX08\/fTTo14A\/IuRTRfFaZJf\/epXobu7O\/paUZE7juC\/9KUvRRjj0YV48dNC48YEWIoJ8cUX2sGDB6OXT5rsMgvFpA2JClqDABG8NaqgjviAQFxeXlYumVk0snXJlKMIXgYlt8sQwbutP+p9gQjEDQiXm+ee1lUi+DSE6L4qAkTwqohReUKAQ0Bc+VrJgeDiXjRxKaE8wKe9aPJA0Y06\/j+wm6iFOsQUwgAAAABJRU5ErkJggg==","height":0,"width":0}}
%---
%[output:4e40d541]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"13.2000"}}
%---
%[output:563fc97c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"0.0075"}}
%---
%[output:6761ce4b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"0.0075"}}
%---
