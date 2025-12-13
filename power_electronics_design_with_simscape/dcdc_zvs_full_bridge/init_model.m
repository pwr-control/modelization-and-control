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
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs_dab","value":"     1.407238661699136e-04"}}
%---
%[output:1e84e4e7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAW0AAADcCAYAAAC74PBGAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ1wX1WVP7hVCUKFtNUSS5siAeqilQ\/dGMHCVGFxJ8VxZ22T\/ahpx+1qqaOmkzTBWacjbdoORSiFJULslHUbyqxlad11EYOwYqm6FCpioUjIxhCqpaW2YHBkze559Yabm\/d13zvvvnvv\/7wZx5L\/veee8zv3\/N55591370mjo6OjwBcjwAgwAoyAEwicxKTthJ9YSUaAEWAEAgSYtHkiMAKMACPgEAJM2g45i1VlBBgBRoBJ29M58Oyzz0JLSws0NjZCe3s7uZVHjhyBpUuXwsyZM2HdunVQVVVFPoYscP369bBr1y7YsmUL1NXVFTqWKnxkZARWrVoFNTU1hWBZpDF79uyB5ubmYIhly5Zp6V8m5iomYr4tWrQIFi5cWCRk1stm0rbeRdkULIO0ccz7778frr322mxKx\/RSCaTIsVQ1BPFt27YN6uvrE23T1W379u0wa9asVLITB5caiJvN4OAg9PT0QHV1tU53sIm0UXHUB32RxRYtwy1vzKRtuYNcUa\/om4RMIIhJkU8RKuY6ZKGLAxJ2R0cHpL0h6MwH30hb9+apg5VLbZm0M3gLg7i7uzvoiY\/M8iO7IJfLL788CES8urq6xj3Syf2xfCHKCyLgse\/x48eDcoAqX1VXBL34uwh+lTxEO3xERt3Fo7JoNzw8PO4RWi1\/4I9YIhBZG\/63KI+0tbUF2fW+ffsCGXPnzh2XDQnywN9UW+XyTRpcb775Zrj++usnjCX0CdNBjI944iUwkP0ilxFkzAUOmGGLMpP4mzpWlA5Rfz9w4MBY6ULohWNE6RI2VVVdxHwS\/hI243+H3Rii+mO5S8zladOmjeEtY6biKuMm5tXFF18czBm8MEOWbZ43b17w96NHj47NF3U+yjrr3hAzhLYTXZi0Nd2kPjKqmZIgHjG5w34XtdkpU6YExCcIQUxKDBKc4IcPH47NKONko1lyNiqTtiAfNQgEWaDuH\/vYx8bVrONIG4l4aGhIS1fU55Zbbhm74aXBVeCm2qbeFFRdZJzwhoI3H5QlfCTbjfVSObMWPlixYsXYjTfsd3HzUTHV0Q3nQZwuankj6caKxCvfaJP6q7ipc1n1kYyDfBOX54OYyzi2qi\/e9LDeLm7y6nxX54jp9yia1GCsOZO2BtRxWZcg3rDaqyDPz3zmMxNe3sURQNwkTXr0jcq05cwl7tE8iRCigjTqxaesz+c\/\/\/mATETmjbbINy\/8u4p1mvKImjViRi3Gkuu6YcQov+SUH8NRFyQWOcOUnwjU7DUqGwzTDW+ecTdefOEaVxII+03+m7hBRdW0VRzUUEi6kWJ7NdsWmX7YTVwdT53DDzzwwLhSkcBS3DCT5rxGKDvdlElbw31hAamS26ZNm8atcpB\/V8sIYmjxWKlmkHGknZR1pCHtuBdN1KSNtokbFJLVypUrQQSjLq5RmbbIni+66KKxrD\/sRhlG2qLcJU8HJGp8QaiStvoIj30EqUdl2mG6RZF2lC7qqomwm65s24IFC2Iz7aR6ehJpi5sX3hxVnMNIWx0virTVkBSlPCbtE8gwaWuQdhGZtjy8GvA+ZdpopyCUyy67DJ577rmx0ogurippq7iFZfU6mbbsk6RsVBBR1I03Trc0mXbc9Cwz0z733HPHPTWKpyWxBJQi01ZtZ9Jm0tag6zea6mQfSUEpatpREz0pm1YzF7kGqBJbWFYVRSiYAatZmqg3ihqlWh4JK3GoAMslAnXNcBpck94F4EsvrKfi0478sjVLTVutn8c9oofVdtX3FFG6qcSbVLqRMU16GtKtaas+jPOJIG3UB9+\/iNJGXHkkS01bXlmTFA+ZAtrBTpxpZ3BamlUOWKP96le\/GkiPWz0ir7TQybSF2rqrR6JqsOrqETkzxn9jiQBXtIStHhErQgQucSteRJuwlQxpcBUrddSx9u7dG9RD8RKrEiZPnhyQOF7i5SPqJnwTtXoE2wv9wp4C4lZNYF8d3QRR4ks5QXjiBZ3wcdxywLjVH2ky0zSrRwTm6o1fXuWC8xiTDzE\/ol6iy33UOYUvK9XSk+wjXj1ScqaNExuvsK\/1VMepS8gy8KyxLnF1YmNK8ECxCOiu95Uzad0PVNgV0QiELQWNw0vXb75iX0qmLcCP+qwWf+\/t7TXyeTS1Y5m0qRHNLy9sGaa83DBpBJyP+OK0jE\/ok3Rz6Xd1SSvqrq4airOHb54lZdr4OLZ69epg9Ki9HPAxbGBgQGufBFsmL5O2LZ54Qw+1BCCXP9Jo6\/LeI2nsM9lGLefJH5fF6SF8yHuPlLB6BEmttrY2IOWo8ohc29QNMJMTkMdiBBgBRsA0AkbLI\/h4tHXrVrjuuuuCN\/xhpC2ymoaGhuALNH40NT0leDxGgBGwGQFjpI1kvGbNGli8eHGwtWbci0gZMJXE5d\/OPvtsm7Fl3RgBRsADBPr6+mD27NnWWGKMtMO+IkMUkvb4FaTd1NQ0YetKJO3+\/n5rwKRWxHf7EC\/fbfTdPvYhddQnyzNG2qoqUZk2vnBobW2Fzs7OICPH8gi2DdtD1\/eA8N0+DvjkAHWhhe\/z1Db7rCBtlajlrDzuQw3bwKQOsOeff96qxzJq+1Ce7zb6bl8l+NA2nimNtCkIwDYwKWySZXDAUyNqXh770Dzm1CPaxjNM2tQeJpTHAU8IZkmi2IclAU84LJO2x2ASmhaI4oCnRtS8PPahecypR2TSJkTUNjAJTWPSpgazJHlM2iUBTzisbTzD5RFC51KL4oCnRtS8PPahecypR2TSJkTUNjAJTeNMmxrMkuQxaZcEPOGwtvEMZ9qEzqUWxQFPjah5eexD85hTj8ikTYiobWASmsaZNjWYJclj0i4JeMJhbeMZzrQJnUstigOeGlHz8tiH5jGnHpFJmxBR28AkNI0zbWowS5LHpF0S8ITD2sYznGkTOpdaFAc8NaLm5bEPzWNOPSKTNiGitoFJaBpn2tRgliSPSbsk4AmHtY1nONMmdC61KA54akTNy2MfmsecekQnSTvsjL0kYPAE9XvvvTepWa7fbQMzlzEhnTngqRE1L499aB5z6hFt45lUmba6dWoSKLi16tq1a4PTq4u8bAOT2lYOeGpEzctjH5rHnHpE23gmFWlTg0AlzzYwqewScjjgqRE1L499aB5z6hFt45lUpC3KIzNnzoR169ZBVVUVNS6Z5NkGZiYjYjpxwFMjal4e+9A85tQj2sYzqUgbQdi+fTt0dHSM4dHV1RWcll7mZRuY1FhwwFMjal4e+9A85tQj2sYzqUlbBgLPbOzu7g7+hC8cw85vpAYuTJ5tYFLbzAFPjah5eexD85hTj2gbz2QibQGKuqok6WR138Gkto8DnhpR8\/LYh+Yxpx7RK9KWwRErRjZu3AjV1dXUuIXKsw1MaqM54KkRNS+PfWgec+oRbeMZ0ky7sbHR6ItK28Ckniwc8NSImpfHPjSPOfWItvGMNmljRt3S0gLDw8MBNjU1NcF67Lq6OmqsEuXZBmaiwpoNOOA1AbOwOfvQQqdoqmQbz6Qi7bAvIrdt2wb19fWa5tM2tw1MWuv4YF9qPMuQx6RdBuq0Y9rGM1qkjSTd3t5Oi0gOabaBmcOU0K4c8NSImpfHPjSPOfWItvFMatJubW2Fzs7OVGUQ\/oydZtpwwNPgWKYU9mGZ6NOM7SxpL126FPbt25caBd4wKjVUkQ054PNjWLYE9mHZHsg3\/uCR16D+bzpg+D++lk8QYe9UmTbheKSibLsDkhoHXNOmxrMMeUzaZaBON2bvTw7C8t79cOTGK+iE5pTEpJ0TwCK7c8AXia4Z2exDMzgXNQqTNjGynGkTA1qCON9JzXf7cMr4bOP6+wdg\/f3Pc6ZNxQ1M2lRIlifH54D3ndDErPHZh0zaxNzApE0MaAnifA54Ju0SJhTxkEzaGQDFHQXxClsfzqSdAVDLujBpW+aQDOr47EMmbc0JsWfPHmhuboao3QOZtDUBtbC5zwHPmbaFE05TpfYdz8Idjwy5X9MeGRmBVatWwa5duwA3iVqyZAncdNNNQLnDH346v3r16gBi3N+EM23N2eZIcyZtRxwVo6bPPlxw6+PwyHNH3SZtQdgNDQ0wa9Ys6O3tDXb227lzJ+zevZtslz8si9TW1sLAwACXR9yP60gLfA54zrTdn7hekLZ8Mvvhw4fHSHtoaCg4gZ0i28bP4Ldu3QrXXXcdbNq0KZa08ce+vj73Z0eIBYjpjBkzvLRNGOW7jb7bh3701cb58+fD0U\/0BFPV+Y9rMAvGrVmvueYauO+++4LyyPLly4NSSd4NpTCTX7NmDSxevDjY54RfRM72mrQ503bfvb76ED9hf\/\/1j\/pB2miFeEkophzVQb\/qft1CftjLSH4RyQFvOwK+EpqMu682ekfapoKFM23OtE3NtSLG8ZXQKoG0RT3bi\/JIEZM7SiaTNpO2yflGPRaTNjWiZuTJWfakl56BX9\/1D2YGTjGK9oZRYafYqOOYOoKMyyMpPGx5E99JzXf7cHr5aOM\/7vwFbH7ol0H0nLL3GzD00D9bE0napI2ai+V4CxcuHDNk+\/btwfI8fBGJ\/8blfzfffHOhhjJpFwqvEeE+BnwllA58tvGRXxyFBbc9Hpg4s\/pkOPaNv4b+\/n4j8ZBmEG3Slpf8yYf5itNqcMkfLgXE5X944G+RF5N2keiakc2kbQbnIkfxyYdyWQQJe+fnLoTLL3mP26QtMu3u7m4Qh\/uqn5tzpk0TIj4FQxQivtvou30+lUdkwka7kLAvPed0sC051M60RfDJS\/PkGraccVdXV9OwV4QU28CkNpYDnhpR8\/LYh+Yx1x0RyfreJ34Nq7\/93FjXe\/5+Lnz0\/BP8ZRvPZCZtXWCKaG8bmNQ2csBTI2peHvvQPOY6I6rZtSiJ4P+LyzaeYdLW8bDhthzwhgEvYDj2YQGg5hSJRN3\/0gh88vYnxkkKI2xvMm31a0hhOZ7A3tPTA0WXRWy9A+acSxO6c8BTI2peHvvQPOZRIyJZX9u7P9i1T76iyNpWntHOtMU67RUrVsCDDz4Y7BGCmxrhVq2485+8DLBod9n22EJtLwc8NaLm5bEPzWMuj4hEjWc84gG96pVE1l6RdmtrK3R2dsKOHTuC7VORqE2+gLQVTOrpyQFPjah5eexDs5gjSQ8cHoEbvjswIaMWmlz67tNhc9OcYA12msu25FA70xb7aeOKkXnz5gUny9x5553Bbn+Dg4NcHkkzC1K24YBPCZTFzdiHxToHSfrAr16FTQ8ORpI0apA2qw7T1nnSFkZt3rwZrrrqquBDGiRu3JYVD0Ooqqoq1kuSdNvApDacA54aUfPy2Ie0mEfVpcNKHx9+9+nQ9IEzg7XWeS7beEY7085jPHVf28Ckto8DnhpR8\/LYh9kwR3J+7fd\/gJX\/+kxsBi2kYyZNRdKqxrbxDJN2tjllpBcHvBGYCx2EfRgPL5IzXviy8JdHXktN0Njns\/POgqv\/dGrq2nRWRztP2mn2HuElf1mnx\/h+HPA0OJYphX14An0k5\/0HX4Vbvz8Igy+\/Fvx3mku8LLz6gqnw2Y+cVThBh+nkLGlHnSgjG2m6rm0bmGkmoU4bDngdtOxsW0k+RCL+2fArcPvDJ7Y0VddDJ3lIlDjarpwNJ510onXaFR5JsvP8bhvPaJdHojLtPKBk7WsbmFntiOpXSQFPjZ0t8nzyIW5ZiiTa88MX4PHBY1oZs\/CHIGGsP7df9cYBHzaQc9ScsY1ntEnblmBAPWwDkxobnwK+Um9MLvlQlCy++aMXYU\/\/0UykLGfHM884GVZeWQu1U6qsyJizxqdtPJOKtNOcVoOA8GfsWadFeD+XAj6r5b7baIt9gpAPHvsd3PXoizB4ZCQzKcvEfNk5Z0DjOSfB+WfPCogZx7E5a84yT50k7SyGmuhjG5jUNtsS8NR2yfJ8t9GEfYKQ97\/4KvznUy\/Bc4d+m4uQ1Wx5+RVnwZzpp465TSVlEzYWOQeTZNvGM6ky7SSjyvrdNjCpcfA9GBAv323Ma58g5L2Dx6Dv6SPwP4fzZchybRnLF2dVnxwsnZt88qTgpyxZcl4bqeOGWp5tPJOZtPF0mo6OjjF8urq6jG4WxTVt6qlZjjzfAz7KPrlcgTXkgZdoyFgmXnzZ99E5U+DimZMjs2QKr\/vuQy9IGw\/2xe1ZxTasouZdX18fHOxr6rINTGq7fQ8G3zJtQcT4\/6Ojo3DPY7+Cp194GQ6NnFinTHFhJowZ8swpVfDn75kC75txWuYMmUIf33wYholtPKOdafPHNVRTPVkOk3YyRkW3UMkWa8bffvIQwCjkrhvLuouyBBLypXVnwMJLpgMuVc5SrigaE1W+7\/PUedJGh3GmbSYsfA+GsrM0Qcg\/f\/EV2PXTQ8Fn1Dpf66WZBTWTJ8GkSZOC7Pi9M06DJQ018OY\/eVOh5Yo0elG28X2eekHa6HCuaVNO+3BZvgcDNWnLWfFTw6\/Ad556ibRWrGbG4kXe7KlV8KmLp4cSMfuw+DgpegRvSLtooNLItw3MNDrrtOGABxBf4SEhP33wFXj8l8cLyYjRL3KJ4rzpb4O\/\/bMz4fRT3pyrZsw+1Jnxdra1jWe0a9o2wWobmNTY+Bbw4sMLkREf\/e3v4fa+AzD0yolyAXVpQiXi2qlV0Pi+aXDeO982Lisu8oMQ33wYNsd9t9E2nmHSpmZaQnkuBINcknj9D6Pwrb2\/CrJjXD1RBAmrRHz+mW+Dv7zwnXDm29+aKyMmdNs4US74MK\/tvtvoPGmL5X0zZ840flKNOrlsAzPv5Ff7lxUMgoj\/MDoavKB7+uCrhZUkVBLGjz3ec+ap8BfvnQpv+uNWby6soIjyfVk+pJ6LcfJ8t9E2nsmUaYdt08of19CHCVUwCBIeBQgy4YeeOVJYOUKgINeHMRvGF3XvOO0t47Jh1Ot\/f\/MizJ79xm5v9CiWK5HKh+VaET+67zZ6QdphLsTVJHfffTcf7EsYXWHBIH\/A8a7T3wr\/8uMX4cfP\/8YoCWM2vGDuO2DO9Ddqw3LGrAOB7wHvu33oa99t9IK0wzJtPJ19y5YtUFdXFxmzcr+4HQFV+VFtbQNTh6zktnJdGP99z2MHg6VqI6+9Bo+9QPMlnaqbnAnjkrUrzquGqae+5Y0VFNUnZzVHq5\/vAe+7fUzaWtOdpLF2eUTUtHF08Rl7Gk1GRkZgzZo1sHjx4oDYMTPfvXt3aF0cP5Hv7e1NrJm7Qtpi2VrefYrjcJZJGD9xXnTJ9HFf09laF\/ad1Hy3j0k7DfvRttEmbarhMZteu3YtbNy4EdQzJZHQBwYGEvcxsY20kZyffOE4fOdnL2kftRSWCb\/++utw9rRTYfa0U4KTputnv32sma0krDs\/fCc13+1j0tad8fnbl0bacZk2fibf3d09Zt22bdsAN6NSr7JJG\/ct\/uI9z2gRtLrHBGbE4qq0fYorIeCZtPOTVNkSyuYZ1X7jpC2fghNGxlhGWbVqFTQ0NARbvWKpZOXKlaH1cgQTr76+PiN+HT72Onz9x0dh1\/5XYsfD\/SbOPG0SfHhWFVww\/a3Bv\/HCv+tcQ0NDMGPGDJ0uzrX13Ubf7cMJ56uN8+fPH4un\/v5+a2LLOGkLywV541auYVm0aKeSuIycqTsgvhxccNvjodtriq0yNzfNCVSjLFtwlmZNnGRWhH2YGTprOprimbQGa5M21dascWQsKy\/aNTU1TSD3IsEUKzref\/2jE7BEYt68aA5ces7paXHO1I4DPhNsVnViH1rljkzKFMkzWRRKTdphy\/zUARsbGyNXfKhkj\/La2tpgw4YN45YJqu2wPII17rCVKkWBiS8Ur717\/7jMGol608Lz4SN1Z2TBOVMfDvhMsFnViX1olTsyKVMUz2RSBgBSk7Zc1mhtbYXOzs7YNdlhCqnEL2raYYTe0tICw8PDELf+mxpMzK5\/8OzLsGL702Pqm8qqw\/DigM86re3pxz60xxdZNaHmmax6iH7apJ13QMr+1GBiKUT+0OV7X7gYLpLO16PUPY0sDvg0KNndhn1ot3\/SaEfNM2nGjGuTirTFS8NDhw7BDTfcEJQr9u3bN0Fu3FeOeRUN608FpvqiEbPrnZ+7kPSlYhb7OeCzoGZXH\/ahXf7Iog0Vz2QZO6xPKtKmGoxaDgWYthI2YsUBTz1jzMtjH5rHnHpECp6h1KmiSRsJ+9re\/WMfx2CG\/cSXP0SJby5ZHPC54LOiM\/vQCjfkUsJ50pY\/jlGRcK088oV7noG79gwHZthG2Jxp54ozazozaVvjisyKOE\/aUZZjnXvevHmxH8pkRi2iYx4wcVkffjAjCNuGGrZqJgc89YwxL499aB5z6hHz8Ay1LiiPrDwStwFUEYqjzDxgVn\/p+2OEbeJDmSwYcMBnQc2uPuxDu\/yRRZs8PJNlvKQ+ZKTt0iEIC259fKyOjRl20V82Jjkh6ncO+KzI2dOPfWiPL7Jq4jxpx9W0o3bjywpWUr8sYKplEZtePHJ5JMnj7v3OpO2ez1SNs\/BMkVaTZdpFKhklOwuYIsu2ZS12HG4c8GXMKtox2Ye0eJYhLQvPFKlnJtJWPzuP2xu7SOV1wez9yUFY3rs\/UOlLH50FX\/74ia1dbb044G31THq92IfpsbK1pS7PFG2HNmlH7c6X9rQZSoN0wJTXZLuQZSNOHPCUs6UcWezDcnCnHFWHZyjHjZKlTdpUW7NSGKcDplzLxgwbM23bLw542z2UrB\/7MBkj21vo8IwJW7RJO2p\/67gtVIsyRAdMl2rZAi8O+KJmjjm57ENzWBc1kg7PFKWDLFebtLEzlkI6OjpArBZBwm5uboaurq7giDBTV1owsTQiDjO49N2nw87lF5pSMdc4HPC54LOiM\/vQCjfkUiItz+QaRKNzJtJG+VF7Y2uMnbtpWjCvue0J+MEvXg7Gs3ldtgoIB3zuKVK6APZh6S7IrUBansk9UEoBmUk7pfxCm6UBU86ybdxfJA4gDvhCp48R4exDIzAXOkganilUAUW4NmlHvYg0qbQYKw2Y8jK\/9qtmQ\/tVtWWommlMDvhMsFnViX1olTsyKZOGZzIJzthJm7RxHNwcqra21mj9Osy+NGDKLyBt3WMkyncc8BlntUXd2IcWOSOjKml4JqPoTN20SdulrVldfQEpPMkBn2lOW9WJfWiVOzIp4zxpZ7K6oE5JYMprs3GPEaxpu3RxwLvkrXBd2Yfu+zCJZ0xbqJ1pm1YwbrwkMOXd\/Ji0bfLcG7r4Tmq+24ee9N3GJJ4xHVmpSNvFg31dL41UQjBUgo2+E1ol+NBJ0jZ9J0k7XhyYcmnEtVUjXNNOOwPsb8ekbb+PkjT0grRd2OXvzkdegLYdBwJ\/uPRBjTyBOOCTwsn+39mH9vsoSUPnSduVXf7kpX42H3QQN2E44JPCyf7f2Yf2+yhJQ+dJ24Vd\/nyoZ1dCrbASbGTSTqJE+393nrRd2OVPJm1XSyOVQGiVYCOTtv2knKSh86SNBtq+y9\/CO34KD+w\/7HQ9uxIIrRJsZNJOokT7f\/eCtBFmm3f586GeXQmEVgk2MmnbT8pJGnpD2kmGmvg9DExf6tmVQGiVYCOTtgkmKHYMJu0U+MpZ\/Ny5c6Gnpweqq6sn9AwDU16ffWvTHGj6wPQUI9rZhAPeTr\/oaMU+1EHLzrZM2gl+wReda9asgcWLF0NdXV1QP9+9ezesW7cOqqqqxvUOA\/N7+4\/Ap+7Y53w9uxKy0EqwkUnbTiLW0YpJWwetP9bO165dCxs3bpyQbYeB6Us9uxIIrRJsZNLWDHgLmzNpazpFN9Ou\/tL3gxFcOgsyChIOeM3JYmFz9qGFTtFUiUk7JWDyvt3iAGG1qwqm\/BJy66cvgMb3TUs5mp3NOODt9IuOVuxDHbTsbOsFaYvT11WI414aZnWHIO\/29naor6+fUNPGP\/T19QV\/Hz72OjRuHQr+\/fVPToeL3+XW\/tkqRkNDQzBjxoys0DnRz3cbfbcPJ5mvNs6fP38shvr7+62Jp1Rbs8raChJdtGiRkePGovY6QZ3UO+D6+wdg\/f3PB+q6\/CWkwJuzNGviJLMi7MPM0FnT0flMu+iDfVX5uPyvra0NNmzYEKwmkS8VTJ9eQqKdHPDWxG1mRdiHmaGzpqPzpI1I4svBgYEBwJJFEVfary1VMH16CcmkXcTMMi+TSds85tQjOk\/ath7sK7+EdPXQA3WyccBTh595eexD85hTj+g8aVMDkkeeDKb8JaQP9WzOtPPMDHv6Mmnb44usmjBpZ0UupB+TNiGYJYnyndR8t68SkgsvSFus6Ni1axc0NjbCkiVL4Kabbgr9arFILpDBlE9eP3LjFUUOa0w2B7wxqAsbiH1YGLTGBDtP2vISvFmzZkFvb2+wL8jOnTsj9wgpCt0w0p5ZfTK4erwY17SLminlyWXSLg97qpGdJ215Sd7hw4fHSBsX2EftEUIFnipHBtO3lSOV8NhZCTYyaRcV\/ebkOk\/aCNX69etheHgYrrnmGrjvvvuC8sjy5cuDUklRywDDXCTAlFeO\/FPzHFh4ibvbscp2csCbC8yiRmIfFoWsOblekDbCpX7K3tXVZeQLSdlVAkyf9tBm0jYXjCZGYtI2gXKxY3hD2sXClE66AFP+fB3r2VjX9uHigHffi+xD933IpE3oQwFmx789C93\/dWKjKCZtQoANiPKd1Hy3rxLeSzBpExKBANO3PUcERBzwhJOlJFHsw5KAJxzWC9KW12kLbJYtW2b0JSSOK8B8\/\/WPAr6M9OHgA65pE0abBaKYtC1wQk4VnCdtQdg1NTVjJC3+htiEneWYE7PI7gjmQ\/\/9c0DSxuvjF0yFby55b1HDGZfLAW8ccvIB2YfkkBoX6DxpR23NijvzlbVOW6zR9mWjKC6PGI\/LwgZk0i4MWmOCnSdtRAqX+4kvIcUJ6bh2u7a21uiyPwTzru\/uhQW3PR448NamOdD0AT\/WaKM9HPDG4rKwgdiHhUFrTLDKMmTKAAAKfElEQVTzpB23NauMIh49du+99xYKLIK5ZvtuWN67PxjHl939ONMudNoYFc6kbRTuQgZznrQLQSWjUATz6jX\/Dr0\/ORhI8Gm5H2faGSeFZd2YtC1zSAZ1mLQzgBbVBcG8oPVb8MhzR4MPanzZKIozbcJJUrIoJu2SHUAwvDekjUeOdXR0jEFS1mfsTNoEs7JEEb6Tmu\/2VcIToRekjS8d8WVkT08PVFdXg6hz19fXG12rjWAe\/URPQDm+rdGuhGCoBBuZtEvMCIiGdp60bVryV3vBB+HYlesD19z4V+fBpz9UQ+QmO8RwwNvhhzxasA\/zoGdHX+dJG2G0JdOe+cGr4ZVL2wLP+rZGuxKy0EqwkUnbDuLNo4UXpI0A2FDTlknbt+V+lUBolWAjk3YeurSjrzekbQOcNR\/\/Irx2\/oJAFSZtGzyir4PvpOa7fZVw42XS1o\/ryB5nLuiA351zZfC7b2u0KyEYKsFGJm3CgC9JFJM2IfDv+Lvb4fWp53m5RrsSCK0SbGTSJgz4kkQxaRMCz6RNCGZJonwnNd\/tq4QbL5M2ITn4eAK7DA8HPOFkKUkU+7Ak4AmHZdImBFOQNu7shzv8+XZxwLvvUfah+z5k0ib0oa\/7aAuIOOAJJ0tJotiHJQFPOCyTNiGYgrR920ebSZtwkpQsikm7ZAcQDM+kTQCiECFI28c12pXwgqcSbGTSJgz4kkRVNGnjJlPNzc1j0G\/btg1wkyn1wqPLWlpaYHh4OPgJD1QQm1PJbZm0S5rFhMP6Tmq+21cJN96KJW3caGr16tXwla98JdgZEAkc9zAJI+Ow48zCeEKQto8f1lRCMFSCjUzahHf4kkRVLGmreIvtXNvb2ydk27ivycDAQOI2r4K0j9x4RUnuLHZYDvhi8TUhnX1oAuVix2DS\/iO+WAJpa2uDDRs2QF1d3TjUMQPv7u5OLKMgaft4Yo0wnAO+2GA0IZ19aALlYsdg0v7\/bVRHRkZg1apV0NDQMOH0dvU3LJWsXLkStmzZMoHckbR9PPyASbvYIDQpnUnbJNrFjFXxpC1IuaamJrH8gS6II3gk7UkvPQM\/Wn1VMd4qWerQ0BDMmDGjZC2KHd53G323D2eHrzbOnz9\/bPL39\/cXGwga0k8aHR0d1Wifq2kcAUcJFn2ampom1L6RtH39GhLx4Cwt13SzojP70Ao35FKiYjPttIStHmcWt8oESdvHE2vEDLNtsuSa+RGdfbfRd\/vQrb7baJt9xjJtde21iGFcq33uuedCa2srdHZ2BnVruS2WUcLq2dgfSfuUvd+Atwz+sAg+YZmMACPACAQIVGx5hP3PCDACjAAjkA8BY5l2PjW5NyPACDACjAAiwKTN84ARYAQYAYcQYNJ2yFmsKiPACDACTNo8BxgBRoARcAgBJm2HnMWqMgKMACPgJGnLSwKXLVuW6stKm12dxh6xzn3Xrl2BKY2NjbBu3Tqoqqqy2bRAtzT2yUbE7Utjq7FpbZTbRW05bKONae2Tt1\/2ITaFL3A\/pNra2gnbbpThK+dIW94dECd91B4mZYCZZcy09uDOh3gtXLgw9tP+LDoU2SetfUIHcXN67LHHItfnF6lvFtlpbVR3tky7m2UWnSj7pLVPvtni9guux6ZM2LiBXVdXF5N2lomFE2Pt2rWwcePGYF9unPi7d+92JutUbc5qjyt269qHdj355JPw1FNPhe4AmWXOFN0nrY2YhT788MPOPRnq2CfvkY\/\/xgu3X3bxkvdJQv05087oRfWAhLjP3DMOYbRbVntcCQgd+wQ54GM12he2ba9R56QcLK2NeEM6dOgQ9PX1wb59+yJPZEo5rLFmae1Lm5EbU5xwIC6P5ABTzTBdJ+0s9rhks459GBjz5s2DKVOmRO61nmPqFNY1rY3Y7pZbbhkr+7hy401rHwIsiBtvSlHHCRbmiAIFM2nnADftXT\/HEEa76toTt7+4UcVTDpbWPrl04NqLyLQ2qjVsV26+ae1T7XHlppRmKjNpp0Epok3a+lqOIYx21bHHlSCXAUxrn3paEcqI2yzMqJMSBktrY1rys8k21CWtfSpJuzhfo7Bn0s4xK32rm6W1x9UASGufSvRRR9HlmDqFdU1ro7ztsFhdkfYwkMKUTyE4rX1hmfbw8LCziwRkaJi0U0yUuCZp14zmHMZY9yh75IkSlom6slY7jX0uk7bIRltaWgBJSl6frAa7jIUr\/tOxD0tAHR0dgTtdWoeeFOxM2kkI8e+MACPACDACoQg493EN+5ERYAQYgUpGgEm7kr3PtjMCjIBzCDBpO+cyVpgRYAQqGQEm7Ur2PtvOCDACziHApO2cy1hhRoARqGQEmLQr2fuO2C5vS6uz3aeNX+SJrUuL\/HBIXlbo06fkjkzXwtVk0i4cYrcH0NlNUKetDipZPyyylbR7e3sL\/+BE3Oiampqgvr5eB25uazkCTNqWO6hs9XSIWKetjl3q599p+zJprwIm7bSzxZ12TNru+KowTdVTcUQJQj6FRHy9NzQ0BOLLP1Qori3KXbp0abANKV5xj+ry7nByW1mHqJKCXA6Q2yBpHz9+PPgfnvij9pdl45hik3ux18bkyZODfkJv+atUtBv79\/T0BPu6R+mgOk29Aak6xn1FqN6E4m6SnGkXFi6lC2bSLt0F5Ssg7z6nBrtMDKgpnkYisjd1N76wtg0NDcFpH3E794kxRVt1J8O48oh6Gozc9o477ghId8uWLVBXVxfs0S32wsAxW1tbobOzM\/hN7nf48OHgxrRixYqxk0rk3\/GIN8RhcHAwIG288OaEm\/1jKSJO3zDSxlNR5BtD1H4dTNrlx4oNGjBp2+CFknVQ93mW1YnL5lRyldtiRi6fMIQyo\/ZvUAk9jMTlE1Fk\/eIIUofkUPe77747IGEkbXXDqrgd7A4cOABynTouyw0jbZmk425uOvZwpl1yUBU4PJN2geC6JFre6EcuI6ikLZcI8FEeL3HCjNwWSyLNzc0TIAhb\/aESrw5px91U4khOPDWIg5Ivu+wyOHbsWChpq\/pgX1nnBx54YGyTJNngsDMFw0gb+4gjueSdAPEJQL6YtF2KqOJ0ZdIuDltnJctlhJ07d46dwYnZs5yBxpVHwjLtKEDKyLTxpiJn72p5JE+mHed4zrSdDQtrFGfStsYV5SkSlsENDAwE2Z9a8sBa7w033BDUbrGfXDOOq2mL2vOiRYsmnGhNWdOWbwA7duwIQBVZrPoksHLlyqDeLfa2FjXqsPKITk1bvJQUOKnlHLmUomIo3zCxdq6WqkQJR9TV8fd169aB2pbLI+XFU9EjM2kXjbAD8tXVI\/IKBkFA06ZNC0oH+HIPX5zhtXnz5uC\/xQs4tS22kVePxH0YE7V6RC1F4EoN9ZJXbuBv8ku9KNLGv+PLRLGqBF9Ioi1Y6sEr7BAGURrC8hHahU8hYatHsH9YaUTYopI21rTxhqEe9quWSmSMhA5PPPFEQNrqi1UmbQcCL6OKTNoZgeNulY1A1rXjSTVtKlSZtKmQtE8Ok7Z9PmGNLEQgbFlklqPCmLQtdK5jKjFpO+YwVrccBNTyTdajwtS9R9S6O4V1vPcIBYr2yvg\/kzhmxXQ\/Qy4AAAAASUVORK5CYII=","height":0,"width":0}}
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
