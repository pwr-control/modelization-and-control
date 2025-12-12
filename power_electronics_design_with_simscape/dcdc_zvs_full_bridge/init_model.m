%[text] ## dabGeneral Settings
%[text] ### Simulink model initialization
close all
clear all
clc
beep off
pm_addunit('percent', 0.01, '1');
options = bodeoptions;
options.FreqUnits = 'Hz';
% simlength = 3.75;
simlength = 1.5;
transmission_delay = 125e-6*2;
s=tf('s');

model = 'dcdc_zvs_full_bridge';
use_thermal_model = 1;


%[text] ### Voltage application
application400 = 0;
application690 = 0;
application480 = 1;

% number of modules (electrical drives)
n_modules = 1;
%[text] ### PWM and sampling time and data length storage
fPWM = 5e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM; % PWM frequency 
tPWM_DAB = 1/fPWM_DAB;
half_phase_pulses = 1/fPWM_DAB/2;

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
tc = ts_dab/400;

z_dab=tf('z',ts_dab);
z_inv=tf('z',ts_inv);
z_afe=tf('z',ts_afe);

% t_misura = simlength - 0.98;
t_misura = 0.6;
Nc = ceil(t_misura/tc);
Ns_battery = ceil(t_misura/ts_battery);
Ns_dab = ceil(t_misura/ts_dab);
Ns_afe = ceil(t_misura/ts_afe);
Ns_inv = ceil(t_misura/ts_inv);

Pnom = 175e3;
ubattery1 = 750;
ubattery2 = 540;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery1;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = ubattery2;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:573defed]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:9c485e2a]
%[text] ### dead\_time and delays
dead_time_DAB = 3e-6;
dead_time_AFE = 0;
dead_time_INV = 0;
delay_pwm = 0;
delayAFE_modB=2*pi*fPWM_AFE*delay_pwm; 
delayAFE_modA=0;
delayAFE_modC=0;
%[text] ### Grid emulator initialization
grid_emulator;
%[text] ### Nominal DClink voltage seting as function of the voltage application
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
%[text] ## HW design and settings
%[text] ### DCDC/ZVS
%[text] #### Input filter or inductance at stage 1
LFi_dc = 1000e-6;
RLFi_dc = 5e-3;
%[text] #### DClink input stage or capacitor at stage 1
Vdc_ref = Vdc_bez; % DClink voltage reference
Rprecharge = 1; % Resistance of the DClink pre-charge circuit
Pload = Pnom;
Rbrake = 4;
CFi_dc1 = 900e-6*4;
RCFi_dc1_internal = 1e-3;
CFi_dc2 = 900e-6*4;
RCFi_dc2_internal = 1e-3;
%[text] #### Tank LC and HF-Transformer parameters
magnetics_design_transformer; %[output:95f45fb2]


Ls_dab = Lsigma;

f0 = fPWM_DAB/5;
Cs_dab = 1/Ls_dab/(2*pi*f0)^2 %[output:7c0c4964]
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
%[text] ## Rectifier stage and output filter
LFu_dc2 = 550e-6/2;
RLFu_dc2 = 0.5e-3;
CFu_dc2 = 3.6e-3;
RCFu_dc2_internal = 1e-3;
%%
%[text] ## Control system design and settings
%[text] ### DCDC Control parameters
kp_i_dab = 0.05;
ki_i_dab = 5;
kp_v_dab = 1;
ki_v_dab = 45;
%%
%[text] ## Settings for user functions: filters, moving average, rms
%[text] ### Low Pass Filters
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
%[text] #### RMS filter
rms_perios = 1;
n1 = rms_perios/f_grid/ts_afe;
rms_perios = 10;
n10 = rms_perios/f_grid/ts_afe;
%%
%[text] #### Butterworth filter
omega_c = 2*pi*5;
P1 = s^2 + 0.7654*omega_c*s + omega_c^2;
P2 = s^2 + 1.8478*omega_c*s + omega_c^2; 
Hb_flt = omega_c^4/P1/P2; 
%[text] ### 
%%
%[text] ### 
%[text] ## Lithium Ion Battery 1
typical_cell_voltage = 3.6;
number_of_cells_1 = floor(ubattery1/typical_cell_voltage)-1; % nominal is 100
number_of_cells_2 = floor(ubattery2/typical_cell_voltage)-1; % nominal is 100

% stato of charge init
soc_init = 0.85; 

R = 8.3143;
F = 96487;
T = 273.15+40;
Q = 50; %Hr*A

Vbattery1_nom = ubattery1;
Pbattery1_nom = Pnom;
Ibattery1_nom = Pbattery1_nom/Vbattery1_nom;
Rmax_battery1 = Vbattery1_nom^2/(Pbattery1_nom*0.1);
Rmin_battery1 = Vbattery1_nom^2/(Pbattery1_nom);

Vbattery2_nom = ubattery2;
Pbattery2_nom = Pnom;
Ibattery2_nom = Pbattery2_nom/Vbattery2_nom;
Rmax_battery2 = Vbattery2_nom^2/(Pbattery2_nom*0.1);
Rmin_battery2 = Vbattery2_nom^2/(Pbattery2_nom);

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
figure;  %[output:1e84e4e7]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:1e84e4e7]
xlabel('state of charge [p.u.]'); %[output:1e84e4e7]
ylabel('open circuit voltage [V]'); %[output:1e84e4e7]
title('open circuit voltage(state of charge)'); %[output:1e84e4e7]
grid on %[output:1e84e4e7]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink
heatsink_liquid_2kW; %[output:7183148b] %[output:285fa001] %[output:2af6aaca]
%[text] #### Semiconductor device: SiC MOSFET SKM1700MB20R4S2I4
danfoss_SKM1700MB20R4S2I4; % SiC-Mosfet full leg
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

danfoss_SKM1700MB20R4S2I4; % SiC-Mosfet full leg
mosfet.dab.Vth = Vth;                                  % [V]
mosfet.dab.Rds_on = Rds_on;                            % [V]
mosfet.dab.g_fs = g_fs;                                % [A/V]
mosfet.dab.Vdon_diode = Vdon_diode;                    % [V]
mosfet.dab.Rdon_diode = Rdon_diode;                    % [Ohm]
mosfet.dab.Eon = Eon;                                  % [J] @ Tj = 125°C
mosfet.dab.Eoff = Eoff;                                % [J] @ Tj = 125°C
mosfet.dab.Erec = Eerr;                                % [J] @ Tj = 125°C
mosfet.dab.Voff_sw_losses = Voff_sw_losses;            % [V]
mosfet.dab.Ion_sw_losses = Ion_sw_losses;              % [A]
mosfet.dab.JunctionTermalMass = JunctionTermalMass;    % [J/K]
mosfet.dab.Rtim = Rtim;                                % [K/W]
mosfet.dab.Rth_switch_JC = Rth_mosfet_JC;              % [K/W]
mosfet.dab.Rth_switch_CH = Rth_mosfet_CH;              % [K/W]
mosfet.dab.Rth_switch_JH = Rth_mosfet_JH;              % [K/W]
mosfet.dab.Lstray_module = Lstray_module;              % [H]
mosfet.dab.Lstray_s = Lstray_s;                        % [H]
mosfet.dab.Lstray_d = Lstray_d;                        % [H]
mosfet.dab.RLs = RLs;                                  % [Ohm]
mosfet.dab.RLd = RLd;                                  % [Ohm]
mosfet.dab.Irr = Irr;                                  % [A]
mosfet.dab.Ciss = Ciss;                                % [F]
mosfet.dab.Coss = Coss;                                % [F]
mosfet.dab.Crss = Crss;                                % [F]
mosfet.dab.Cgd = Cgd;                                  % [F]
mosfet.dab.Cgs = Cgs;                                  % [F]
mosfet.dab.Cds = Cds;                                  % [F]
mosfet.dab.Rgate_internal = Rgate_internal;            % [Ohm]
mosfet.dab.Csnubber = 2*Eon/Voff_sw_losses^2;          % [F]
mosfet.dab.Rsnubber = 1;                               % [Ohm]
% Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2
% Rsnubber = 1/(Csnubber*fPWM_DAB)/5

%[text] #### Semiconductor device: Si IGBT Danfoss 600A-1700
danfoss_IGBT_SEMiX604GB17E4s; % two IGBT modules in parallel
par_modules = 2; % number of paralleled modules

dab.igbt.Vth = Vth;                                             % [V]
dab.igbt.Rce_on = Rce_on;                           % [Ohm]
dab.igbt.Vce_sat = Vce_sat;                         % [V]
dab.igbt.Vdon_diode = Vdon_diode;                   % [V]
dab.igbt.Rdon_diode = Rdon_diode;                   % [Ohm]
dab.igbt.Eon = Eon/par_modules;                                 % [J]
dab.igbt.Eoff = Eoff/par_modules;                               % [J]
dab.igbt.Erec = Erec/par_modules;                               % [J]
dab.igbt.Voff_sw_losses = Voff_sw_losses;                       % [V]
dab.igbt.Ion_sw_losses = Ion_sw_losses;                         % [A]
dab.igbt.JunctionTermalMass = JunctionTermalMass;               % [J/K]
dab.igbt.Rth_switch_JC = Rth_switch_JC/par_modules;             % [K/W]
dab.igbt.Rth_switch_CH = Rth_switch_CH/par_modules;             % [K/W]
dab.igbt.Rth_switch_JH = Rth_switch_JH/par_modules;             % [K/W]
dab.igbt.Rth_diode_JC = Rth_diode_JC/par_modules;               % [K/W]
dab.igbt.Rth_diode_CH = Rth_diode_CH/par_modules;               % [K/W]
dab.igbt.Rth_diode_JH = Rth_diode_JH/par_modules;               % [K/W]
dab.igbt.Lstray_module = Lstray_module/par_modules;             % [H]
dab.igbt.Irr = Irr;                                             % [A]
dab.igbt.Cies = Cies*par_modules;                               % [F]
dab.igbt.Cres = Cres*par_modules;                               % [F]
dab.igbt.Rgate_internal = Rgate_internal;                       % [Ohm]
dab.igbt.td_on = td_on;                                         % [s]
dab.igbt.trise = trise;                                         % [s]
dab.igbt.td_off = td_off;                                       % [s]
dab.igbt.tfall = tfall;                                         % [s]
dab.igbt.Csnubber = Csnubber*100;                               % [F]
dab.igbt.Rsnubber = Rsnubber/100;                               % [Ohm]
dab.igbt.filter_time_constant = tc*10;                          % [s]
%[text] ## ZVS constraints 
idab_zvs_min = 2*mosfet.dab.Coss*Vdab1_dc_nom/dead_time_DAB        % [A] %[output:22ea45fb]
%[text] ## C-Caller Settings
open_system(model);
% Simulink.importExternalCTypes(model,'Names',{'mavgflt_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'bemf_obsv_load_est_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqvector_pi_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'sv_pwm_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'global_state_machine_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'first_harmonic_tracker_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqpll_thyr_output_t'});
% Simulink.importExternalCTypes(model,'Names',{'dqpll_grid_output_t'});
%[text] ### Simulation settings
% scenario = 1; nominal conditions
% scenario = 2; voltage input at minimum, voltage output at minimum
% scenario = 3; voltage input at maximum, voltage output at maximum
% maximum input current = 270A
% maximum output current = 270A

delta_load = 40;
Uin_min = 500;
Uin_max = 900;
Uin_nom = 750;
Uout_min = 400;
Uout_max = 900;
Uout_nom = 600;
u_cell = 3.6;

Iout_dab_nom = 300;
Iout_dab_max = 375;

Iin_dab_nom = 240;
Iin_dab_max = 300;

scenario = 1;
Iout_dab_sim = Iout_dab_nom;
    number_of_cells_1_sim = floor(Uin_nom/u_cell);
    number_of_cells_2_sim = floor((Uout_nom-delta_load)/u_cell);
    i_out_dab_ref_1 = Iout_dab_sim;
    i_out_dab_pu_ref_1 = i_out_dab_ref_1 / Idc_FS;
%[text] ## Remove Scopes Opening Automatically
% open_scopes = find_system(model, 'BlockType', 'Scope');
% for i = 1:length(open_scopes)
%     set_param(open_scopes{i}, 'Open', 'off');
% end

% shh = get(0,'ShowHiddenHandles');
% set(0,'ShowHiddenHandles','On');
% hscope = findobj(0,'Type','Figure','Tag','SIMULINK_SIMSCOPE_FIGURE');
% close(hscope);
% set(0,'ShowHiddenHandles',shh);
% 

%[text] ## Enable/Disable Subsystems

% if use_thermal_model
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge ideal switch based', 'Commented', 'on');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge mosfet based with thermal model', 'Commented', 'off');
%      set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge ideal switch based', 'Commented', 'on');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge mosfet based with thermal model', 'Commented', 'off');
% else
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge ideal switch based', 'Commented', 'off');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge1/full-bridge mosfet based with thermal model', 'Commented', 'on');
%      set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge ideal switch based', 'Commented', 'off');
%     set_param('single_phase_dab/dab_modA/dcdc_with_galvanic_isolation/H-Bridge2/full-bridge mosfet based with thermal model', 'Commented', 'on');
% end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":38.1}
%---
%[output:573defed]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"     4.050925925925926e+02"}}
%---
%[output:9c485e2a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"     9.375000000000000e+02"}}
%---
%[output:95f45fb2]
%   data: {"dataType":"text","outputData":{"text":"--- INITIAL ELECTRICAL PARAMETERS ---\nNominal Power (Sn): 260.00 kVA\nNominal Primary Voltage (Vn): 650.00 V\nNominal Primary Current (I1n): 400.00 V\nNominal Secondary Current (I2n): 280.00 V\nNominal Frequency: 5.00 kHz\n----------------------------------------------------\nCore Section Area (S_Fe): 292.7928 cm^2\nPrimary Turns (n1): 5\nSecondary Turns (n2): 4\nPrimary Copper Area (A_Cu1): 133.33 mm^2\nPrimary Copper Band Length: 13.33 cm\nSecondary Copper Band Length (L_b2): 18.67 cm\nCore Height (AM-NC-412 AMMET): 29.00 cm\nCore Width (AM-NC-412 AMMET): 6.00 cm\nCore Length (AM-NC-412 AMMET): 95.00 cm\nCore Dept (AM-NC-412 AMMET): 48.80 cm\nSpecific Core Loss (AM-NC-412 AMMET): 2.67 W\/kg\n----------------------------------------------------\n--- LOSS ESTIMATION ---\nCore Mass (M_Fe): 216.96 kg\nCore Loss (P_Fe): 578.56 W\nCopper Loss (P_Cu): 176.44 W\nTotal Losses per Phase (P_tot): 755.00 W\n----------------------------------------------------\nEstimated Efficiency (Eta, cos(phi)=0.95): 99.70 %\n----------------------------------------------------\n--- LEAKAGE INDUCTANCE AND REACTANCE ESTIMATION ---\nCalculated Leakage Inductance method 1 (Ld_calc): 0.000031 H (31.42 uH)\nCalculated Leakage Inductance method 2 (Lsigma): 0.000017 H (17.03 uH)\nEffective Leakage Inductance method 1 (Ld_eff): 0.000041 H (40.84 uH)\nEffective Leakage Inductance method 2 (Lsigma_eff): 0.000022 H (22.14 uH)\nCalculated Magnetizing Inductance (lm_trafo): 0.004841 H (4841.24 uH)\nEstimated Short Circuit Voltage (Vcc): 78.96 %\n----------------------------------------------------\n","truncated":false}}
%---
%[output:7c0c4964]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs_dab","value":"   0.001487043826097"}}
%---
%[output:1e84e4e7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAYgAAADsCAYAAABuUTHeAAAAAXNSR0IArs4c6QAAIABJREFUeF7tfX+UV1d1707kRQYTJRNidDKSwWRIXcZOfqwoEmyaxYqJyzXYaltmeK+hhLCwEtergAxDzEt5jx8DgqsxiUpxwqKtzNAujYH11Lw4Lw9NCdWCjDWijJlMcDKJEAhiEqLPSrtvsoczZ+6Pc+\/3nnv3uWfff5Lhe+4+e3\/2Pvtzz+9zzpw5cwbkEQQEAUFAEBAENATOEYKQmBAEBAFBQBAIQ0AIQuJCEBAEBAFBIBQBIQjPA+PEiROwcOHCAIXu7m6or6+3gsjOnTuhs7MT1q9fD3Pnzg3qwH\/Dh\/7Os+Iwu06fPg1r166F+fPnQ3Nzc57VRcoaGBiABQsWwCc\/+UkjO7PouG\/fPtizZw90dHRYsYmw7O\/vD+Tv2LEDZsyYYVxXmO+NX7ZUEHFeuXIlNDQ0WMPNkuqFihWCKBRuqYwQsJ00dIKoq6sLEsL+\/fth27ZthRHEhg0bABO4CflS0kqjI8qeN28eLF682FqiozpUck8TybZ9nUYXtSzqdd999xUaD1l1Les9IQgLyGNS2LJlSyAZv1DUhESNZenSpdDX1wf4VaaX0b\/Y1MZPX6TXXXcdXHDBBcHXHD5JjZfq1XXSE+nx48eDL176wsYvU5JtKgN7IXpSUZME6oC9CXpaW1uhq6sLMInjQ4nyyJEjo4k1LHkSFiMjI8F7Kk6qXffffz9s3LgRdu\/ePVon2jRnzpyANPR\/V3s05Ev00aZNmwD\/njp16qi+ug6qH+g3tI++7nXfku8bGxsjdVFxRwMIL4wdJAd6WlpaArzwwV4hffEnkQdhSziQHPSjXrf6m9p04mJW9f3Q0FDQNvSYp3jRbUEdCEc9Jm+++eZROxGTj3zkI3DHHXeM66VSrOl1hvnHQjpwWqQQRM7uU8lBFU3dcr3BJTVu+p0Sj56Q6Hc9+PUvJTUhqyRx0UUXjRliIoKgpEtyDxw4MCapx8molSBQNuFEuKnEiGQyPDwcEBnpqZMNJj0aOosiCEpWKlYqjmHJ8dixY4DkHKeD7mvynZ6IVd9H6XjZZZeNIQE1HvTfMHl\/9rOfhU9\/+tOj5KDHjx7uUTpF+T2MIHRyoDqImKJinoguypf0vh7zqNsXv\/hF+PKXvzyG3D\/wgQ\/Ad7\/73dAPmjDiKWp4NecUU6g4IYgc4aZAvvjii0e\/fOnLiBrDrl27gkRLf2P19BVLvQH8KtSTCn1NUwLH97Bnon55ho0NUyPAxEY9GfWLjr7CUB5+fery8astrYwkgsAv9KRhB\/3rTi9PRByWfBGH6dOnjyE+kyEmkqm+j1\/hesLXfUm\/E07Uw\/j85z8ffC3T72E9IzX8TIaY9CGlqL+j4kefY9LjE3EirPUEH9VL1cvriffRRx8dE\/MqeYcNvUV9DFDMY0yS3kRY5F\/sBam9Q7UXGjZUhj7Hd4ocdswx5VgXJQSRI8RhSU9PCtRY1GSuBi6qo3\/tq1\/r+P\/45UxfsWqDDiMIvbHRMA6ZHTXEpMpPKyMPglBxo69rauyoe9jEuoqjTnxxBKF\/4SKO2LPScY4iAD2EMGmRzvp8QtRwEeoXRxBRw2k6QUR9rUf1MFVSpIln3U76qIkiiDAZYT3YJNLSeyJ6DyMs5lWdwvxPw2yqPuqQW5LuOaYHJ0UJQeTotrAvlCiCiArsKILAf49KXPpwjGpS2uROPYhaCUIny6S\/w9xA79x9991B74bG8qO+xNMShJ4c1L9rIQh1CCRqwjlsnop6g+o7pj2GpOEcih999VFY7BRNEHrM0ZCTPpSXF0Goc15CEPEJUAgiR4KwMcSkqxeW8OMIQv0qox6GmnQWLVoUOgehNkZTGTSMpQ576RPcUX+HuUH\/alZ7SLUOMelzL+oQRdYhprTDRVheJU6aNFcJQk9g+nBO0hBTUnjnOcSkD5uSHTR\/FdWDoF41\/a7rpBMG+irLEFMYFkIQQhBJbSTX321NUpt0t6PWp4cNO9CQQ9QktUoQaiJTwYpbgUPlkggCy0WtjMHfCE+9TNRkPeGkj3OrBIByb7vttmAiN2wIImpBAepgMklNX\/N68omazI3CEeXgQyviwoZJ1NU\/KOfee++FNWvWjLNLXylGspImqXG8P2m+yHSSOokg9IYYF\/NheptMUus9KZmDEILIlQBMhOW9zFVNjml7EKRv2Dg7DjeYzEEkycDf1YSN+mLP5M477xy3ooSShJpU4ggibp2\/6TJXmghVkykm3xtvvHF0hRAmo9tvvx2WLFkyOpSlExQuc12+fHnsMlc1EYcNOYYl07D5KKwbdaQeHi2HfuCBB+DBBx8Emo9RiU8nfSK\/OHyxnrhlrnovJ2pTY9T8gTpHFkUQOnkjHri8miaPUQd9Pgj\/Ta1T9ae+GVOd01N\/04fS9Pk5k7Ze9TKlDzFhI1+xYkWwTj1sd6u+PjpuOacLzkr6GnPBBh90VJOmvsRY711F4UEJCInY1i5nH3wRZiN9HOBvYavzTHbnyz6I5OgplSBMlvVhQsX16lVpYEIQyUHJpUTUcGHSpkRV\/zQ7qbnY7YIeScN1JkepYFuUndSMh5iwd4ANCJ+oHgT+3tTUZHSOjQuBLQThgpde0zFsnDtpV7JuHX2l4vBUmvOL3EGpPE3D5qFMz4mSs5jM\/FZaDwK\/AFavXg3t7e0BSYQRBDlx5syZlSEIM7dIKUFAEBAEykegNILAL2l8rr322sg5iLBuZJruffnwigaCgCAgCLiLQCkEgV3D7du3w1133RWcqRM1Sa13z6W77m6gieaCgCDgHgKlEAQOKeESQxyTTVrFpENKcxZhk9bvfOc73fOAaCwICALeI4AnO0+bNo0dDoUTRNTqA0TGZIIpbtIaCWJwcJAdyLYU8s1exFFsthVNvOT65meu9hZOEHoYxvUgaJWTutEJNypFnbzIFWRbTc83e4UgbEUSP7m+xTZXe9kRBJJCT0\/P6IUs+ka5uF4GV5BtNT\/f7BWCsBVJ\/OT6Fttc7S2dIPIMTa4g52mjKuvpp59mOW5py16UKzbbRJePbN\/8zDV3CUHwaROpNfGtEQlBpA4RZ1\/wLbaFIAoIVa4g2zLdt0YkBGErkvjJ9S22ueYu6UHwaxvGGvnWiIQgjEPD+YK+xbYQRAEhyxVkW6b71oiEIGxFEj+5vsU219wlPQh+bcNYI98akRCEcWg4X9C32BaCKCBkuYJsy3TfGpEQhK1I4ifXt9jmmrukB8GvbRhr5FsjEoIwDg3nC\/oW20IQBYQsV5Btme5bIxKCsBVJ\/OT6Fttcc5f0IPi1DWONfGtEQhDGoeF8Qd9iWwiigJDlCrIt031rREIQtiKJn1zfYptr7pIeBL+2YayRb41ICMI4NJwv6FtsC0EUELJcQbZlum+NSAjCViTxk+tbbHPNXdKD4Nc2jDXyrREJQRiHhvMFfYttIYgCQpYryLZM960RCUHYiiR+cn2Lba65S3oQ\/NqGsUa+NSIhCOPQcL6gb7EtBFFAyHIF2ZbpvjUiIQhbkcRPrm+xzTV3SQ+CX9sw1si3RiQEYRwazhf0LbYrQRAnTpyAhQsXQn9\/v3EAtrS0wEMPPWRcvpaCXEGuxaa4d31rREIQtiKJn1zfYptr7krVg0CCWLZsGaxatQqam5sTo2pgYADWrVsH27ZtSyybRwGuIOdhW5gM3xqREIStSOIn17fY5pq7UhEEvzAaqxFXkG3h5lsjEoKwFUn85PoW21xzVyqCoCEmDKfu7m6or69nFVlcQbYFkm+NSAjCViTxk+tbbHPNXakIAsPo9OnTsHLlSti9e3cQVTt27IAZM2awiDCuINsCx7dGJARhK5L4yfUttrnmrtQEoYbSvn37YN68ecE\/tba2QldXF9TV1ZUWbVxBtgWIb41ICMJWJPGT61tsc81dNREEhZXaq2hoaAgmpU0msfMOS64g520nyfOtEQlB2IokfnJ9i22uuSsXglDDi1Yubd68ufA5Cq4g22p+vjUiIQhbkcRPrm+xzTV35UIQ0oMop4H51oiEIMqJszJq9S22K0kQMgdRRtM5W6dvjUgIotx4K7J232K7MgShr2Jav349zJ07t8jYiayLK8i2wPGtEQlB2IokfnJ9i22uuSvVEBPtgzh27FhpE9FxocwVZFvNz7dGJARhK5L4yfUpth\/\/2Un46P\/shaN\/93F2jkhFEOy01xQSguDuodr18ylxEFpic+1xw1lCz\/efhyU9h+DE525ip2YqgpCzmHj5TxIHL3\/Y0kb8bAtZHnIrRRBymiuPoJLhFj5+sK2JEIRthMuVv+GRIdjwyNPu9yDKhTG5dhliSsbI9RKSLF33oJn+PvlZCMIsJhJLbdiwISjT0dERWlYIIhFC5wv4lDhkDmKa8\/FqYoAQhAlKCWVoz8XixYuFIF7HSpJlDoHlgAjxswNOqkFFnKDGeQjnJ6lrwKCmV3Hvxdq1a+HJJ58MTo6VHsRrcEriqCmsnHlZ\/OyMqzIpKgSRCbazL+3cuTP4Y2hoSIaYFCwlcdQYWI68Ln52xFEZ1fzjLx2EPYdfrF4PAhN3Z2dnAAveC4FPT09Prsd+49La1atXwz333ANbt24VghCCgGnT\/BibJlcLQWTMvI68NueBH8DjT52sFkHghPHIyEiQuDGBt7e3B8M\/SRPJaX2G8m688UYj2TJJnRZd98pLsnTPZ1k09snPV695Ao6ceLU6BKFumGtsbAxumCOCyPO4b5S1fft2uOuuu4KLiJLIBwmCnr6+vixx6dQ7w8PDgPj79IjNfnjbBz\/Pnj0bfjdpCpz64GurMyszSR1HELjaCBN5HndWq0NYarPA2+vuvffecS1FehDVTx4+fVnKEFP1hxLxHKY5X\/hBtQgCrcHkvXfv3jFDTNOnTwfcad3W1mblhFeTHsTg4GD1s+TrFkqy9MPV4ufq+pn2QFSqB0HuUu+DoH+zefy3EMTYhiKJo7qJQ7VM\/FxdP9ME9bmvvAAvfOlP2Rma6rA+dtprCskQE3cP1a6fJMvaMXRBgg9+xolpnKDGZ8KxH8PRv1\/CzjVCEOxcYq6QD41IR0NsNo8Pl0v64GfqPaCfzn98Ixz53jfZuSwTQdDFQf39\/bEGxR2LYQMJ6UHYQJWXTB8Sh5Bi9U8JoCO+0dezLp8MP9r8MeA4f5qJINAonKTu7e0ds1qJiIMmqZPmDPJOPUIQeSPKT54QBD+f2NCoyn5Wh5am1k+E+9veBbd98NrqEAQRAZ6JhJvj1Edd5nr8+HFYt25dcD1pEY8QRBEol1tHlRNHFLJic7kxl3ft9UsfGxW56xPXwKwrJgPX3JWpByEEkXfIZJMniSMbbq69JX52zWPj9cVeAz40KY3\/v\/0vroLW3784+PdKEYTpEBPtlQjb1GbD5VxBtmErypTEYQtZXnLFz7z8kVYbJIc7ew4F5y3R89XFLXDTlfWjf3PNXZl6EGRV2D4IPLQPh53wt+XLlwfDS83NzWkxzVSeK8iZjDF4SRKHAUgVKCJ+dteJnV8fgC3fGR41AOcccFgJ\/6s+XHNXTQTBzW1cQbaFkyQOW8jykit+5uWPJG3Cegz4ThQ5VHKIKQmkMn4XgigD9WLrlGRZLN5l1eain58ceQn+64P\/FpzMqj60Ugkno6Merrkrcw8ibHiJjG9pacnlsL60wckV5LR2mJZ3sRGZ2hZVTmyuFUE33nfFzwd\/\/iv4H7t+NmZ+gRCO6zHoXuCauzIRBF4Bikd8z5w5E+bMmTN63Lftw\/qSQpsryEl6Z\/3dlUaU1b6w98TmPNHkK4urn7F3sOn\/DME\/fO+5UPCQFL6y8D1wwRsnjJtniEOba+7KRBDqcd84AY0b4pqamoITXLFnkfetcqZhzBVkU\/3TluPaiNLakaa82JwGLXfLcvEzHsf9j\/ufh3\/4l3BCQISRFD52zSVw94fP3keTFnmuuSsXgsDlrHhfNG6cy\/PCoKqAnNYO0\/JcGpGpvnmUE5vzQJG\/jDL8jL2DYy\/9Blbvfip0yEhFDUnh6395NTRdVJcLmJUiCEREPUZD7TXs2rUruCeiq6sruAWuyIcryLYwKKMR2bLFVK7YbIqU2+Vs+xl7BvVvmgArvzaQSAbUS\/j4H7wDrmo4P9j5nPfDNXdl6kEgOPpuaiSMLVu2QENDQ6F7H1RHcQU572AiebYbkS29a5ErNteCnjvv5uVnWlG04ZGn4ecnXjUmg6kXToTPfmw61J33hlRzCVkR5pq7MhNEViBsvscVZFs259WIbOlnQ67YbANVfjLT+jkLEVDPAP\/7v+ZcAS2NFxRCBmFoc81dmQhCn6RWDZY5iOIaW9pGVJxm9moSm+1hy0lylJ+RCB49dBwePngUjrz46rg9B3E24LwB9gz+++zLoPmtk0ojA68JQj3Ntb7+7FkjRQQfVxa2ZbskS1vI8pLrk5+pJ\/Dlvp\/AwaNnUpMA9QqQCFZ9aBo0TJ7IigiiIotr7krVg4jbHKcaXvRFQVQ3V5BtpRufEgdhKDbbiqbi5CIJ\/Pq3v4O\/6XvGeF5A147OMsID7z41+7LRn\/UzjoqzqraauOauVARBEMQNMdUGU21vcwW5Nqui35ZkaQtZXnJd8zMSwMDRV+DrB4\/CM8dPZ+oFUE8A\/\/sHzRfCHTdcCpMn\/ZfAMa6SQFxUcc1dmQiCV\/M5qw1XkG3h5VriyAMHsTkPFLPJUM8YeujgUeg7dDwQpB5jnVYyzQtcM\/XNsPCGS0df\/\/dfPgfTpk1LK87Z8lxzVyqCML2LWs5iKiZOJVkWg3PZtRTlZyKAHz77K\/jmj14Ihn\/STgSHDQXhfEDTlDr4q9mXwYRzzwl6AFhXXE+gKJvL9i3VXwmC4AJmlB5cQbaFm2+NCHEUm7NFE24Mw4T8f396Ar524Bc1f\/mrQz1IAFe+7U3wkZa3BnLz2Ejmm5+55q5UPYhsoVncW1xBtoWAb41ICGJ8JFHif+Gl30D3Pz+by1c\/1ULDP\/j1f\/sNl0L963MAKjlIbOeDANfcVRNB4BlMnZ2dYxBav359cGhfGQ9XkG1hIQRhC9ny5dJwz4mX\/z9s6TsMz758bi5f\/fqX\/zvqJwZj\/1POPy+Qz2UC2LfY5pq7MhMEkkNvb++Yex9ojqKtra0UkuAKsq1041sjcr0HoU7yHnr+ZfjGj47B08eyr\/IJiytK8Djs8\/uNF8Ct754SJH0a9+dEAnHtwrfY5pq7MhGEfg6T6mjZKGeLDsbL9a0RcSUIGuZB\/bbtfRb2P3MqcFatE7yqx9XE\/3tvfxPcMasRJk54rVfhStJP0zJ8i20hiDTRkbEsV5AzmpP4mm+NqEiCUL\/29x85Bd8+dDzX8X09seMX\/3saL4Db3vf24IA4WuWDxCB+TmwKzhfgmrsy9SDQGzLEVH5MSuJI5wNMupR4fzj8K\/jmk\/ks5dS1UL\/2cYz\/vdPeAjdNP3vsTNpxfvFzOj+7WLpyBEEkIZPU5YWj74lD\/crH\/3\/q2Cvw\/WdOwZEadu\/GeVNN\/O9uOB\/mv78BJmlf+zaiwXc\/28CUm8xKEUTZk9FRzuUKsq1grGLiUJP+86d+DV87cBR+\/NxLuY\/phw3xXHHJJLj59y4CTP6cxvar6OekNuGbzVxzV+YhJv3gvh07dsCMGTOS\/G71d64g2zLalUZESf\/Mf26iwrH8A0dOWRnPD0v6l188CRbc0ACT686e45O0i9eWv7LKdcXPWe0Le883m7nmrswEoTpV3Q8hN8rl2UziZZXRiNQvfNTu+8\/8EvYcfhGGXjhdyFf+q79+FT5w5SXw5+97O5x77jlBnSZHNxTnlfxrKsPP+VuRTqJvNleaIFTX49Wj2Lvo7u4GuQ8iXaNIWzqvRqQm\/cO\/eDmYvB34xSvWEj4l9eC\/F06EK946CT527SXwjgvPntETNZGbl81psS6zvNhcJvrF1F1pgqD7qBFKk4P6Tp8+DStXroTdu3cH6MfdH6EPZcX1ULiCbCvEwhIHDZ\/Qip0LJr4BvvK95+Anz71cWMLHlTtXXDwJrm96y5iduWlX78jQw2sICEHYakF85HLNXZmHmGoZVkJCwaejowOSJryxnqGhoaBs0sMV5CS9TX+nL\/3HDp+A7z39Szg8chKOnYZU1y6a1qWu2JlaXweXXvhGaL\/+bXDuOWeHdVBW0eP5kixNPeh2Od\/8zDV3ZSKIuJ3UWcJSJQz9ffytqanJ6OgOriCbYIK7cS9583nwhT0\/h6eOvpLrLlx9SAf\/ntV8IfzZdZeMJny1jIm+ZZXxLXFID6KsSCu2Xq65KxNB5AldHNnQUNTMmTMrQRD4tY1LN9f878FcCKDhzRNgwoQJwTg+fvH\/t\/c1QMPkNwbuoR5A0V\/4ecaGDDHJEJPteOIiXwgixBM0d9Ha2gpdXV1QV1c3plTYBUVxp8VyAhkT88gvfw3rvjGY+sYtSu6zrrgQ3tNwPnzoqiljkj6BJF\/TXJq3XT3Ez3bx5SCdU+5S8Si9B4HKIFGMjIyMI4mBgQFYsGABbNq0Kdhjof+tOxZBpqevr69wv4+c+i389bdfgP3PvhpbN37543PdpRPhL657C5z3hnOA\/i2N0sPDw9DY2JjmFefLis3Ou9DIAB\/8PHv27DFYDA4OGmFTZCEWBIGJf8WKFbBx40Zobm6OtT9uvqJoFqZJ4zt7DkX2EujSlRW3TBs9djkvB8uXZV5I8pYjfubtnzy0Kzp3meqciSBw6GfZsmWwatWqcQkdk\/26detg8+bNxvsg0hwRHjdpXSTIOKk85ws\/GIczEsINl0+Gjlteu3A9j6WdUc6UxGEa5m6XEz+77T8T7YvMXSb6UJncCcIk2au9AJqIxv0N+lJWXRb+vXz5cti2bVtoT6MIkJEY7uw9NGZpKZLATVfWw6dmX2aVEHTHSuJIE+rulhU\/u+s7U82LyF2muqjlUhGEvmktqsK4jW\/4jr5RTp2kxjp6enpG5yPSnPlkE2QcTtKHkpAYdn3imkJJQcVcEkeWkHfvHfGzez5Lq7HN3JVWl8wEQS\/GDTHVokyt79oEuX7pY6PqITE8\/Ilr4LL6ibWqXNP7kjhqgs+Zl8XPzrgqs6I2c1dmpQAgVQ+iloqKeNcGyPpcA5LDwc+8vwhzEuuQxJEIUSUKiJ8r4cZYI2zkrjxQS0UQtC9h0aJFsHXrVujv7w\/VweQ8pjyU12XkDfLf7RuBv\/rHnwbVlD2cFIaXJA4bUcRPpviZn0\/y1ijv3JWXfqkIIq9KbcnJE2Q84O6TvT8ZJYf7294Fs66YbEv1THIlcWSCzbmXxM\/OuSy1wnnmrtSVx7wgBBECTs\/3n4clPYfY9hxIZUkceTYFvrLEz3x9k5dmlSKIsCMwVKBcHmLC1UpXr3mCPTmggpI48mqevOWIn3n7Jw\/tKkUQUYDg8tW1a9fC\/PnzE3dE5wGqLiMPkJEcaIc0LmHlNqyk2iyJw0YU8ZMpfubnk7w1yiN35a0Tyst9iEnfx2BD6SiZtYL8qX\/6KWx\/YiQQj5ve7v7w2bOdirTDtC5JHKZIuV1O\/Oy2\/0y0rzV3mdSRpUzuBJHlqI0sioe9UwvI+tASl6WscdhI4sgrcnjLET\/z9k8e2tWSu\/KoP0pG7gQRdTKrTSNIdlaQkRzwXCW6O6HM3dFpcJLEkQYtd8uKn931nanmWXOXqfys5TIRRNwkddyd0VmVNH0vK8jqqiW8VvOB9neZVllqOUkcpcJfWOXi58KgLq2irLnLtsKZCMK2UlnlZwHZxaElwkcSR9ZIces98bNb\/sqibZbclaWetO9kJgj9OtC4U1nTKpW1fBaQ1aM0uK9a0nGRxJE1Utx6T\/zslr+yaJsld2WpJ+07mQki7OKeskkiLcgu9x7Q0ZI40oa7m+XFz276LY3WaXNXGtm1lM1EEHlfGFSLAeq7aUF2ufcgBJFX1PCXIwTB30e1apg2d9Van+n7mQli4cKFwQU\/eFe0+phcGGSqXNpyaUBW73fgdEJrGpslcaRBy92y4md3fWeqeZrcZSozj3KZCAIr3rlzJ\/T29kJ3d\/fo1aK0uqmtrQ3mzp2bh36pZKQBWe09bPzodLhj1qWp6uJQWBIHBy\/Y10H8bB\/jsmtIk7uK1DUzQaCSYTfMrV+\/vhRyQH3SgDzngR\/A40+dDLB2bXKaAkQSR5FNpby6xM\/lYV9UzWlyV1E6YT01EUSRiprUZQqyOjk96\/LJsGvJNSbi2ZWRxMHOJVYUEj9bgZWVUNPcVbTSmQiCViu1t7ePm4Mo2gC1PlOQ1Y1xeKQGzkG4+EjicNFr6XUWP6fHzLU3THNX0XZlIgjX76Sm4SVXJ6dliGla0e2k1PqEIEqFv5DKK0UQiBhOUg8NDQUrmbg8JiBXZXgJMZfEwSXy7OohfraLLwfpJrmrDD0z9yBwmauLd1JveGQINjzydIC1q5PT0oOQHkQZyaLIOn0jxUoRRJGBkqYuE5CrMrwkPYg0keF2Wd+SpY+xbZK7yojiTD2IMhQ1qTMJ5CoNL\/nYiMRmk1ZQjTK+kWJS7irLq6kIgjbCLVq0CLZu3ercEJO6egmP9MajvV1+fGtEQhAuR2s63X2L7UoQRDoXF186CeQ\/\/tJB2HP4xUrMP0iyLD6+yqrRt2TpY2wn5a6yYi9VD0JV0rXjvl0\/uTUsQCRxlNVsiq1X\/Fws3mXUVjmCcO24b\/XspY5bpkHHLU1lxEGudUriyBVOtsLEz2xdk5tilSIIV4\/7rl\/6WGWGl3zshovNueUj9oJ8I8XKEYRrx31XaXkrtW7fGpEQBPu8npuCvsV2pQgCo8Cl476rtrxVCEI2yuWWiZkKEoLg4ZjMk9SovivHfavzD19d3AI3XVnPA\/0atfCtEUkPosaAceh132K7cj0IjrEWBbK6\/8H14zVU3H1rREIQHFudHZ18i20hiIxxpPZSGhoaYNu2bdDc3BwqLQpk9XKgE5+7KaMm\/F7zrREJQfCLQVsa+RbbQhAZImlgYABWrFgBGzduDEghbN5DFRsF8tVrngCch3A730KQAAAPWElEQVT9eG8dQt8akRBEhkbk6Cu+xbYQRA6BqhOGLjIMZHWC+g+nXwhf+\/jVOWjCQ4RvjUgIgkfcFaGFb7EtBJFDVGEPYu\/evdDV1QV1dXXjJCYRRFU2yJHhvjUiIYgcGpEjInyL7coRBH7NL1iwAEZGRsaFXEtLC3R3d0N9fT6rhdS6duzYEXnNaRjIVbr\/QYaY5JIkR\/J7zWoKQdQMYS4CMi1zpXOYcNK4yBvliCg2bdoUShJhBFHFDXLSg5B9ELm0fsZChCB4OCcTQZR5J3XYGVAEJRIEPX19fcH\/tm4fhpFTv4XrLp0If\/tRt4\/31kNmeHgYGhsbeURSQVqIzQUBXXI1Pvh59uzZY1AeHBwsGfXx1WciCOpBtLe3Rw732LBUP0FWr0PvQagT1Hj3A94BUaXHt68smYOoUvTG2+JbbFduDiJpwjiPUNZXLeGeiOXLl0fuhdBBVndQV2mDnAwxyRBTHu2LswwhCB7eydSDoJvl+vv7Q63Ic5JaP84jzSR1VXdQC0EIQfBIH\/a0EIKwh20ayZkIIk0FRZbVexBV3UEtBCEEUWS7KqMuIYgyUM9pDoKH6uO1iCKIqu2gFoIQguDaBvPSSwgiLyRrk1NTDwLnITo7OwMNcOgHn56ensiNbLWpmvy2ShBVPeJbRcG3RoS2i83J7aAKJXzzc+UmqXG5KW6Su+eee2D16tVAK5rilqHaDtwogsDVS7iKqWqPb41ICKJqERxtj2+xXSmCUPdB4Dr8lStXjhIErjxat24dbN68Obed1KbNQgW56iuYJFmaRoX75XxLlj7GtjcEgauOsBeR51Ebpk1cBVldwXTwM+8PTnKt2iOJo2oeDbdH\/Fx9P1eKINBdtA9CHWKaPn064F3VbW1tMHfu3MK9qoJc9RVMPn5lic2FN6nSKvSNFCtHEBg5nK8crfIZTNRqfWtEQhCl5evCK\/YttitJEIVHTUKFKsj1Sx8LSs+6fDLsWnINN1Vz0ce3RiQEkUvYOCHEt9gWgiggLAlkdYnrjoXvgVvfPaWA2ouvwrdGJARRfIyVVaNvsV1JglD3QVAgxR2FYTvYCGR1BVNVl7hKsrQdTXzk+5YsfYztyhFE2P3QdEZT2ZPU6iVBVV3B5GMjEpv5kJZtTXwjxUoRBBEBXhY0Y8aMMbHCYZmrEITt5luefN8Sh5BiebFWZM3eEASHjXI+rGCSxFFk8y23LiHFcvEvovZKEQQCFnY3A5chpqvXPAE4UV3VQ\/ooYCVxFNF0y69D\/Fy+D2xrUCmCSLoPQgUT74Z46KGHbOMbyEeQ\/9+\/\/hiQIPCp8hJX6UEUElIsKhGCYOEGq0pUiiCsIlWDcJ0gOm6ZBh23NNUgkferkjh4+ycv7cTPeSHJV44QRAG+0QmiyktcpQdRQEAxqUIIgokjLKpRSYII2wexfv36Us5hoiGmtTv3wpKeQ4Erq3gPtRqjkjgstlhGosXPjJxhSZXKEQTXfRBCEJYimIlYSZZMHGFZDd\/8XCmC4LwP4qplX4XHnzoZhO+Jz91kOYzLFe9bI5JhtXLjrcjafYttIYgCogtBJoKo+hJXSZYFBBSTKnxLlj7GdqUIAh3IdYjpzbd\/JdgDUfUlrj42IrGZCWMVoIZvpFg5giCS6OzsHBMuZU5SN131Xjj1wQ2BPkIQBbTiEqrwLXEIKZYQZCVUWUmCKAHH2CoR5JN\/1B2UqfoeCEkc3KLPnj5Civaw5SJZCKIAT0x974fgpVkrgpqqvgdCCKKAgGJShRAEE0dYVEMIwiK4JLrxD\/8cXrn29uDPqu+BEIIoIKCYVCEEwcQRFtUQgrAIrhBEAeAyqUKSJRNHWFbDNz8LQVgOKBT\/tj9ZA7+ZekNQU9X3QEgPooCAYlKFb8nSx9gWgiigsb31ti\/Bb6dcWfljvglKSRwFBBWDKsTPDJxgWQUhCMsAo\/gpH\/8n+N2kKUIQBWBdVhWSLMtCvth6ffOzEEQB8VW\/9LGgFh\/2QPjYDRebC2hETKoQguDhiHPOnDlzhocqtWtBBNF+\/duCZa5Vf3xrREIQVY\/os\/b5FtvSgyggtokgfNgkJ8mygIBiUoVvydLH2BaCKKCxEUH4sEnOx0YkNhfQiJhU4RspCkEogaffad3a2gpdXV1QV1c3Ljz37dsH8+bNG\/33hoYG2LZtGzQ3N48rSwThwyY5SZZMMlkBaviWLH2MbSGI1xvS6dOnYeXKlTBz5szg5jn6GxN\/R0fHuOaGp8YODQ2F\/qYXFoIoIFuVXIUky5IdUFD1vvlZCCImsJAE9u7dG9qL2LBhAzQ1NRldY0oEcfAz7w+Wulb98a0R+fhlKTZXvRW\/Zp8QRAaC0HsbSaGCBOHDRUGEgxBEUkRU43fxczX8GGeFEEQEOjQf0dbWNq6XoM9VoIi4+yaEIKrfkCRZVt\/HPvaahCBC4pp6CPhT2CT1wMAALFiwADZt2gQzZswA\/e+wOYgJL\/wUzn98I\/T19VW+JQ0PD0NjY2Pl7VQNFJv9cLcPfp49e\/YYZw4ODrJzbmkb5ZLIIQopnJPAJ2xCG3sQvuyi9vErS2xmlz+sKeRbT1F6EEooJa1ciou6uElrJAhfNslxntiyljUYT+aJzfkiwDVh5mvlWWlc7S2lB4FJfmRkJHLvA8GGeyCwbHd3N9TX1wP+vXz58th9EEIQtkKYh1yuDckmOmKzTXR5yObq48IJImziGV3U0tISEMHhw4ehp6dnlDz0jXI7duwI5iPCHuxBTDrwIJx35J95eF20EAQEAUHAEAGZgzAESooJAoKAICAIlI9A4T2I8k0WDQQBQUAQEARMEBCCMEFJyggCgoAg4CECQhAeOl1MFgQEAUHABAEhCBOUpIwgIAgIAh4iIAThodPFZEFAEBAETBCoBEHgabCdnZ2BvXFnNZkAwrEM7gXZsmVLoFrcMl+1XNy9GRxt1HUytZneS3uwIzcMTO3Vl4nHxQM3G7P6mI7Ywb1Trsd1nE\/SnFxdlG+dJwgMnhUrVsDGjRsDzOj\/wy4UKgrUPOtRNwviHhF146Baj35kOv7d29s7uskwT51syzK1WbcfPxJc\/EAwtVc\/gUCNfdfi3dRmIkQ8Wgf3P7kc10nkgB+B3OLXeYLQEyNHFq4loapnT1GCaG9vj9wsSHW5nDzS2oxJZNmyZXDy5EkIOxW4FvyLeNfUXvTpunXrYPPmzcHJAi4\/aWxWP\/pcjuswfxEBTp48Ofj51ltvNbr7pijfO08Q+uF9cYf5FQVqXvVE3b5Ht\/HF1eNqQ8piM\/r8+uuvh4cffnj0psK8fGBbThp74y7Wsq1nnvLT2BzWg4i6XCxPHYuShfa9+OKLwdCZetNmUfUn1VMJglBvnEtzRWkSOGX\/HtZjMO0hmZ53VbaNev1pbUYi3L59OyxduhRWr17tLEGovcIoH1NsI2Ymc1LcfEv6pPUxld+9ezcsXrzY6PphrrZH6cV1Dk0IgnEkpW1IZAomkvvuuy\/yUEPGJo\/eUW6SMBGftWvXwvz584N7MTh+gSVhncbHtBiDJqaTDq9Mqrus39PYrN8Box\/gWZYNedcrBJE3oq\/LkyGmscC6TA5oSZrhB0wWe\/bsCb4ouTawpLBPY68+xCQ2z02C15nfufrS+R6EPqRkOgTjSuSo9iRNUldlhYepzerSUNWfrg1DmNqLhKiedJwUD5xj3NTmqpBiki+EIJIQyvi7LHN9DThXhxvC3G66BFJ9l2sDMwlrU3v1CVuXh1tMbQ4bYoq7E8YEb45luMav8z0IdLavG+XU3lPU17SrG6miNo5FLULg2sBMk5GpvepGOdc3jZnarN4J47rNMklt2iKknCAgCAgCggBrBCrRg2CNsCgnCAgCgoCjCAhBOOo4UVsQEAQEAdsICEHYRljkCwKCgCDgKAJCEI46TtQWBAQBQcA2AkIQthEW+YKAICAIOIqAEISjjuOotr6RK05H25u81OWgra2t0NXVBXV1dYmw6XsNEl8ouAAtDW1pabF6lLt6\/lEa\/AqGQ6qzjIAQhGWAfRLPiSDS6KL6yAWCQH3xeJEinqqcIFsEVlWsQwiiil61aJN+oxkda6FuZqKvW\/xixwP08BROevBClDlz5oz5d7okRf1qxfJJX65RG6jUjZMoJ2yzYJQd9O94OQ2dmKp\/rav1onz1d6z72LFj0NfXB\/39\/UHd+LuKw9133w27du0KLrnCi37S2K2fPZbmhNcw8ksigKTfLYaaiGaAgBAEAye4okLSwXLqVzvahEkRd77S1656kCCdvkqntobthI6720M\/WiTsb\/XcIhXjODtuvvlmWLhwIUydOjUYltLt0OtRCQXtDDssUT16neTt378\/OG037BTaOLvDCEK9SU8\/miKpd5REAEm\/uxK7omc2BIQgsuHm5VtJx1kkDeuo52bpBBH2Lt0Ut2rVquBLm54oPdTkGadL3BlGWb6y1Xr1hBpmg0oyx48fH3MAH9oYZTf+FkYQ+gU6UQSTxTYhCC+b+qjRQhB++z+19erwij4EFJWU1TN36CwdnSD0YSFSLOzsnah5AvWcpjiCiEt6pkmUvtRHRkYCVWmoTZcddk2oSpQHDhwA7AHoT9SZQ1FDTOqcRJR9prapughBpG4ilXpBCKJS7izOGDVBqvMQ6rAOEQMRyfDwMND9wmEEYXqVZJkEgTYsWLAAkBhobiOuB2FCEKZ2R\/UghoaGxkxaC0EU1w6qXpMQRNU9bNk+\/dhmIggcBlq2bBmow0NJQ0yYaLu7u6G+vj5W6zKHmHByWU\/ItQ4xmdotQ0yWg1nEj0NACEKCwhiBpIlkdVgHy+JkLw594Iog+urHFT7q5Kw+Sa1OasfNFeQ5Sa0m3kWLFo3RG39Tv8iRINQvfhoaixpiItnY41AnvfVJalO7oyapqTejkrA6b4N6kP+oLvIJTciH7RORISbj5lHJgkIQlXSrPaP0sXd1HkInAZyAnTdvXqAMJqVNmzYFk6xtbW0wd+7c0Xs8KLnqS0+TNoPF3ROQNGGu10V26MSmEwT+rS5ZRd2bmpqgt7c36P08+uijYwhETcy03BeXuX7nO9+BzZs3B72lNHaHEcS3vvWtAGO8fhUfdVlv2JwIDZEhvkiIjzzySEBeSbabbDS0F3kiuQwEhCDKQF3q9BqBsHkJU0BMVjGZyjIpJz0IE5SqW0YIorq+FcsYIBA2oR63zyFJZSGIJITk9zwREILIE02RJQiEIKDvvKYhtSxg6WcxhQ1pZZGrvyNnMeWBovsy\/gO4PuPCqI8DuAAAAABJRU5ErkJggg==","height":0,"width":0}}
%---
%[output:7183148b]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:285fa001]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:2af6aaca]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
%[output:22ea45fb]
%   data: {"dataType":"textualVariable","outputData":{"name":"idab_zvs_min","value":"   1.640000000000000"}}
%---
