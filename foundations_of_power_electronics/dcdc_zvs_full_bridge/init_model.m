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
heatsink_liquid_2kW;
%[text] #### Semiconductor device: SiC MOSFET SKM1700MB20R4S2I4
danfoss_SKM1700MB20R4S2I4; % SiC-Mosfet full leg
dab_mosfet.Vth = Vth;                                  % [V]
dab_mosfet.Rds_on = Rds_on;                            % [Ohm]
dab_mosfet.Vdon_diode = Vdon_diode;                    % [V]
dab_mosfet.Vgamma = Vgamma;                            % [V]
dab_mosfet.Rdon_diode = Rdon_diode;                    % [Ohm]
dab_mosfet.Eon = Eon;                                  % [J] @ Tj = 125°C
dab_mosfet.Eoff = Eoff;                                % [J] @ Tj = 125°C
dab_mosfet.Eerr = Err;                                % [J] @ Tj = 125°C
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
mosfet.dab.Erec = Err;                                % [J] @ Tj = 125°C
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
idab_zvs_min = 2*mosfet.dab.Coss*Vdab1_dc_nom/dead_time_DAB        % [A] %[output:5b4d7b44]
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
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs_dab","value":"   0.001407238661699"}}
%---
%[output:1e84e4e7]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAU4AAADJCAYAAABMkdu9AAAQAElEQVR4AeydCdxNdf7Hv5f+k0FClAohhNI2WcpSJkUkNDVh2qZFohX1NNSkxa7NEpUWLWhXpmgjoSGVUJQ9azW2TFGS+T\/vX36385zn3nPPfZ67nHvv18v3Oef81u\/vc8753O9vOd9fif\/pP0VAEVAEFIG4ECgh+k8RUAQUAUUgLgSUOOOCSxMrAoqAIiCSFOJUYBUBRUARyGYElDiz+e5q2xQBRSApCChxJgVWLVQRUASyGYHMIc5svgsR2vbuu+9KrVq1ZNy4cRFiix+0ZMkSOeGEE6Rv377FLyxGCT\/88IN069ZNTj\/9dPnuu+9ipE5ONDjSXtqdnBqSUyr6ojfCud9agoC5U1d0pw08187wTD1X4szUO5dgvSE0iA2C46VLcPEFiktlXVTMSzt27Fi57777pGHDhgRFlXh1gwiS9QPHfRg8eLCUL19epkyZIocddlhUvYMeAe5nnXWW9OnTR7gfQdc3ln45SZxYWTzsVnj4AYoHFeJAeFBtPNYK8Vac+SEbXjbiKIc8vKCUwTlHyiU+klA26RB+kaM9VNRJXc8995yxRKmL8khPPvIjlOcMt\/WjA+eUYfUlHcL1hRdeKOvXr5d58+ZJ9+7dhfTEWaE+Z\/nEu8tDR9JYIY\/Nb4\/\/+c9\/JFJd6IBuNi9l2TwcuSaOtoIv57atxFMXYVa4JhyZMGGCIZ\/jjz+eS9M2dLdpqZf6kXh0ow6wotBhw4aFeweE27I5ck2aSEKd1E86hHPCwJeyuR\/cl06dOsm3335bqAjSkoe8CDi5E73zzjvmmSHeiRl1OHHgnDDyozPped4o35brfN6uueaaQj0JZzz5nfVddtllFC3cD3OSwX9yjji5ka+88sr+W\/bbgQeUB+W3KzHk0bt3b3spvBQ2ngfImZ+H+qabbjIvo80wZswYUwbXPPjPPvssp4WEMinbRvz3v\/+V6667Lmp3lrruuOMOm9ykIz35bCDlUa69TtQR0qlWrZqsXLnSFLlr1y7ZuHGjHHnkkVK6dGlDGk5cSOTGlbBI8uOPPwoY0j4bT1ncK645cs05bQVfzq3QXuqy1xytZQOxLFiwIKwncQMGDAjfH66pl\/ppE9dOiaWbMy3nXroQ7xRIinqp34ZzDnlv2bLFBkU9RsoPTuBlM1Ge85nB8obciHfjwLNKGHFWyEsZXIOl83mDkMlDHEK5F198sXCPuEaczyMWM9Yz94OyiM9UySni5GZNmjRJDjroIHnttddk9erVcuutt5p7N336dHPkT7R48nPTmzZtKosXLzbCOQ8PQl6EMOIfffRRLuWDDz4oQKwE8tA\/8cQTnArp0OX88883Vh95TUSEP+hL2tatWwsvCQ81+QijHLI428J1LDn00EPlxRdfFIgR3SmnbNmyBbKRplGjRkL7wQEdqbtly5YC4fjBlQIrV65cqK5QKGRImLopFyzRBdywsjiSF72IJx3XVmiv+57x8mLZkH\/Hjh2CnrQJ3Ddt2mTaSj22vC+++MK88G4cQqHouqEHOqEH96VHjx7ipQvpnEL9COWgB8I5uPIDRdlcgwUkRXfXKz\/PNDiAF4Rv01KOLRtcwCQaDvb+2rzUT94RI0aY5x3dbBjl2nQc586dazAk3Pk88pxTHz+w\/NByP9CBPJkqOUWc3Cxu2rHHHis1a9Y09wzS4cHkgbG\/8s74Zs2aGaLlZVu7dq2Qn4cdCwzh3BTk+GNfUn5heZAdUeFTyAaLjbophwgeTh44SJFrt1AW+thwXi7O27Zty0HIR37KMQEJ\/lO7dm3TfnCkbqsP1+DixC0SrtHUgZRnzZolEydOFKxzXkxeUNLv3r3bkKrFCfK74ooriDLCC8m9gRA6duxouqRYOSYy\/w+6EZd\/av6T\/4gjjjA\/UNQD9pS3aNGiiOOfXrqZAh1\/YuniSGpOwZAT+7ygG7oQBgFz9BJ3foiVdoBjmTJlTFYnbrTbBOb\/oS7SgTvECA5ez3J+lnBvw+oLfghxiNUH659uOkfCs1FyijgTdQMhBQjKKZDW\/vKz9mBJ++233zZWtJMoi9NoCIfxNV42yJkXmBc+njJJTz7nPeEHxL7MzrIIhyzsS88LzjgelrQzHedF0S2aLpQXJGHYCczByIlHcXTkxxTL13kfIGiI2hoLdNcxKopTT7rz5hRxcrO4aXTL1qxZY7C33V26oZUqVTJhvIAIF3T3sFj4ta5Ro4aZZMA6jfSSkd6vuLstzheUcTI\/5UAypOPB58jYFi8CLwTXCFYtDyztpd2EFVWw0iHLTz75xFiB1vLwi2u0esEaobvLDxAvsbU4bR6uCQcnun42nBeSe2Pjbbg9WozsNfcNkoQsbZeSeqPlRy+ENNF0s2XH0sWms0erG11r2oXYttlehE0b6Wjz2\/vPc8P950fI2VWPlJfxSLr\/GAE98ocY4nk+rL7ggtjy0Yd3hS67DXMesf7pmfCuYck74zLtPKeIk5vVtWtXMw7j7ta5H1ReLB5CiJWbSjz5uem8ZFgrxCO8iLyQpPMrvGQQDw8autiuEuUifsrBAuQXnq4penDkmtlLS2ZWV+qgrljl8iLQdl5id1qrM2l4AaifNODiF1fSW6Ec6ipXrpwZDkF\/2kGYTcOPGWVzTbjFiWsr3BvOiSe\/FYgEHMCEeARduYfUTVmkpV7SkJY0CPGU56Ub6axQBj9cXrrYtPbIfUaoC10QzrFYObfpoh1JQ1qeUdqBvqSlu2+76lxHEtqKEWHz+nk+IFnqQ0fqtvXZ8nkewBEs0McKuJAGI4Q6+RHiOpMlp4iTG8WvKw8A51awPLAm7DUPxz333GMvzQSSjaeb58zPgzJ69GjhhQxn8Hni1oV6H3zwQYGg3EVEumZMi6U5zjiuCUcfZkRtXK9evYSX1F67j9YCdoe7r+3LgeWJBWrj3W0h3I0rYYi7rgYNGghr\/IgDT7p26IqFjCXETC3XxIOR894Qxr2hLs6t8HISbgnCWknEM3Nsy+MasbjFqxvlozNlINQZTRfincJ95n7TJhvOORNU3D8bFu1IGp49Z\/223dHy2HDy2h8kwtCZ55ofWix7wtxCHq9niueOMWqnPpTJs2EtXPJTjrvsTLtOCXFi6Vx66aWCBSAR\/k2ZMsUM6ttfqKJYcBGKjRoE+TnHYHjY3Yl5kW0abrwz3pmfwXgeGOIphzw2PeHEQwS8JKRxi7MsBurtQ+Uui3SURZnOMmw66kW4tvGcE4awPAc9bB2UQ3mUS3r0I560HLkm3C02X6Q0lEV+K9RPfsoiva3bXpOOcK5tXnSC1AjnHHLGssHK4eWmjIULF1Ks0DU0J\/l\/qIvyrNh7AJ4QhB2yyE9qfpgo36blSH53HGm8dAMLBD0pw9ZJWVxbseGU7xb0o002LeeEkY660cEZRrhTnPVThq0rUl6LMfpRBmnJgxDmjOeacNKQFuH95V5wfyDXs88+W\/hxY6acHxzSuPWhTGc45XKd6ZJ04vzf\/\/4nLFWJNu4BgBs2bBDW5nGjEK8HhfQquYMABMCQBi3mpeXHle4lVg3WHuGxBCuZoQXyxUqr8dERoHuORcyPGOe2e88YM\/cpes7si0k6cfKLNH\/+fGnRokVU9DZv3iyMZUVNkKIIbn6sX\/gUqZKaajKkFqweflCdgpWHdeOnCaQjPeX4Sa9pIiOAJYxR47wPnFurMnKu7AxNKnEywcAYDJZCNGJk9o91eIx91KlTx4zDTZ06VbBUsxNybZUioAhkOgJJJU666Jj2jRs3jorTnj17hBnA\/v37y5dffilDhw6V4cOHy6efflooD900lVoFxoMVD8Uj154BJgwLkUOKA5JGnMuXL5fZs2cbhxGhUChqsypUqCBYpXTlS5YsaT6NO+WUU8ynfZEy0TXIBqFt2dAO24bf27PafMpqwzP1qO0J7n0888wzuT1plaQR57p162TOnDnSpEkTYyExME+X3bk4m5YvW7bMfKv9888\/cxkWBv\/DF3qiCCgCikCAEEgacbLswGltsJ6L5STugWQmZPhEi5m6X3\/91XzKx5ovFikHCKeEq\/Lee+8lvMx0FqjtSSf6sevOtvsTu8XJTZE04vRSm69sWKvJujDGQJkYwmErk0OMdfbr10\/q1q3rVYTG5RYC2lpFIFAIpIw4sTSxQmm9XdZgr1lQizcYLFS6961atSKZiiKgCCgCgUQgZcQZyNarUoqAIqAIFAEBT+Jkm4MuXbqYmW6+3oglRahfsygCaUNAK1YEioqAJ3GyCJ3Z7p49e0peXp6nkKaoSmg+RUARUAQyCQFP4gyFQmaLAdZNtW\/fXryENJnUcNVVEVAEFIGiIuBJnAceeKAwUXPAAQfELJ+9ZGIm0gSKQLYjoO3LCQQ8iZNu+mOPPSZ8Mmndwv3yyy85AYw2UhFQBBSBaAh4EidWJA43Xn\/9ddNlZ8vcP\/3pT3L33XcL34syBhqtYA1XBBQBRSBbEfAkThrN9+P169eXgQMHCnvNPP7448Js+znnnGN2VXz11VdFrVCQUlEEkoWAlhs0BGISp1Ph\/\/u\/\/xM+hRw5cqRMmzZN2D8E1\/\/bt293JtNzRUARUASyGoG4iBNLk+\/N8WSExcm2BhDnIYccktUgaeMUAUVAEXAiEJM4sSafe+45YX8RtiDA0uRb8o8\/\/lj4jPKkk04SuvPOQvVcEVAEAo+AKlgMBDyJE2ccnTp1klGjRgnHmTNnCmOa7dq1MxteFaNezaoIKAKKQMYi4Emc+MRkORIbrfFlELvZZWxLVXFFQBFQBBKEgCdxsq0ve1Bv3bo1ZnVYpzETaQJFQBHIagRypXGexAkIO3bskCFDhgjjml5CGtKrKAKKgCKQ7Qh4Eidd9VtuuUX4Dp2JIS8hTbaDpe1TBBQBRQAEPInzj3\/8o\/lW3cu5hzOOAlUUAUVAEUgoAgEszJM4A6ivqqQIKAKKQNoRUOJM+y1QBRQBRSDTEFDizLQ7pvoqAopAAhAoXhFKnMXDT3MrAopAChFYt+0n2VO9mUxa8I1wnsKqC1TlmzhZ04lbuXr16snpp58uM2bMkNtuu814SipQol4oAoqAIpAkBOau2iG7Tr5Cek1aFnzixO\/mAw88YNzHPfPMM3L44YdL7dq1pXTp0nL\/\/ffL3r17kwSTFqsIKAKKQPAQiGJxFlR0y5YtsnTpUmFN51FHHSUlSpSQUqVKyeWXX24cGuMIpGAOvVIEFAFFIPEIpLN77myNL+J0ZnCesxcRPjqdYXquCCgCikC2I+CLOCtVqiQNGjSQe++9V1avXi2\/\/vqrfPvttzJu3DizpUaFChWyHSdtnyKgCAQMgeoVS6VNI1\/EGQqFhIkhvCNdcsklsmDBAuNmji463XcsTx8tiJmECSi7KVzMxJpAEVAEcg6Bddt2B6LNvogTTcuWLSt33nmnGeucP3++OeKnM1HWJhNQkyZNElzYUZ+KIqAIKAJuBNZv+ykcFHiLc+fOnTJ06FDjIQnyZIb9rrvuMtd4TMJD+9H+0AAAEABJREFU\/MaNG8MNKsrJF198IRAy23IUJb\/mUQQUgdxBIJ2kCcq+LU5m1t944w2BRMnINVsHc5w3b5507tyZ4CLJDz\/8IKNHj5bu3bsL46lehdSqVcvM5LM9MZKpsmHDhqxoh8Vf27Mm0PczG+4P7\/6HS1YaeqheIX3jmyjgiziZOd+zZ4+wURsEN2jQIHP+yCOPCB6Uhg0bZrbXoMCiCF30atWqSePGjWNmZ3KqZs2akulStWrVjG+D8x5oe4L9TGbD\/Xn\/46Wyr3QlwxHNaqd3QtoXcTJpww6XvChG6\/1\/WARPOPHuuP1JYh6WL18us2fPNtZmKBSKmV4TKAKKQG4iwFdDtuXNji5vT9Ny9EWcODRmcoguOZM4aMqRMUnOsTpXrvzNhOY6Hlm3bp3MmTNHmjRpIpjir7zyiiHRvn37xlNMYtNqaYqAIhA4BCZ9tNnoVGLXFmleOwOIE2K84YYb5KGHHjIE17JlSzn11FPl9ttvF5YnrV271nxVZFoV55\/WrVubtaF0wZHzzz\/fDAOMGDEizpI0uSKgCGQrAnNW7pA5q3aY5pXYFXsPNJMwiX98WZzUf9xxx8n06dOFXS\/z8vJk+PDhxlI855xzpF69evL222+TTEURUAQUgYQiwGeW101eZspkNr3Ul6+Z83T+8U2cdM0Zy8TBR6NGjeSYY46R9evXS7du3YQN3cqUKZOQdmBpYoUmpLBAFaLKKAKKQLwIQJrnPbww7AmJsc0DtnwVbzEJT++bOMePHy8QZtOmTcUK3eoaNWoI458J10wLVAQUgZxGgO65kzSxNsd0rR8ITHwR5\/fffy8zZ86UiRMnyvPPP2\/GNVesWCHXXXedHHvssWZJUiBao0ooAopAxiOAlTn0rbXiJs3Pbj81MG3zRZw\/\/\/yzUZglR+XLl5etW7fK7t275bzzzpNZs2YJxGoS6J9UI6D1KQJZg4AlzBPv\/bcMfWtNuF15bWpKkEgTxXwRJ7PqOPJgyRHEyVgnQgGQpiVWrlUUAUVAEfCLAGTJNhjnjVkobsKka\/56z5Mkr00Nv8WlLJ0v4mQdZ9euXc2So02bNsnRRx8tHTt2NFKlShWpWLFiyhTWihQBRSCzEXCTJdtg2KVGtAzCzNtvZaZ7vSb6RBJfxElGlh3RLWdME1dy9913n4wcOVKGDBkiWKOkUckOBLQVikAiEbBECUFW7D3TWJacu8mya6MqgoVJtzwvgFamExNfxIljD75T37VrlyFJuu54MTrhhBPkhRdeEJx0OAvVc0VAEchNBCBJZsPpfkOOTqIkzIkKlmXzo8sLM+WQJcegWphOvTn3JE4mgJhNf\/PNN+Wll14SjnhIsvLqq6+acAiVwlQUAUUgdxCAJCFDBJK045TMhnNNuBsNyBLLEpKELF\/vdZJw7U4X9GtP4sSSxAMSHpG+\/vpr4\/oNv5xW8MN58cUXS+XKlYPeTtUv3Qho\/RmLAATptCIhSKclaUnS2fW2jXUS5ev5Ez2QJaSZiWRp28TRkzghxMmTJxtLs0+fPub4wQcfiBU+s7zoooskFFKvRoCpoghkKgJOchz61lqBDJ0E6bQiIxEk7XaT5Lb7W5llRJYoM6UbTltiiSdxsikbbuN++ukn6dSpk3D87rvvxCnEky5WRRqvCCgC6UXASY4QIxKJHFlDSTfbiyAZm8RqhBSxJLOZJCPdNU\/iZKH7BRdcEP7E0n5q6TwST7pIhWuYIpBcBLR0JwJOYnRajayP\/NOotWY222k5epEj5WJBugmSrjbC2CSkCXlmkyVJu\/2IJ3Eeeuih5ssg3L1FE5Yokc5PZZpGEVAEioYApIhAdgjWIoLFCDHaMUdLjE6rkXzRanWTI2TotCDdBEn6aGXlUrgncTqBwDvSwoULzU6XfKPO+k3I1JlGzxUBRSB+BCA2O\/niRYoQJGSJkA6hO03+aLVCdEiH+mXN7LUlxmjkmKsWZDT8ooX7Jk58cfbq1Uv27t0rJ510knE+zLjntGnTopWt4YpAJiKQMJ0hNMQPKVpLMR5SRFFIEYHwEIgRcRIjXesBrSuZ9ZKkoWuNkF+laAj4Ik6+S2dDtQceeEAGDhwoV155pfHSjlNjwokvWvWaSxHIPAQgQ8QPIWIlFpcU7ThjXpuahvwgRcjQTshwDlkiECOixJjc58oXcbIQHkuzZs2aBbThmnDiC0TohSKQYQhAhIglw6nLfhDnBIt7LLE4hAg0WImWECE6L1K044x5bWqY7jakSH7KUUkPAr6Is3Tp0sIWwcuWLSugJXtq8506n2AWiNALRSAACECEiCVDxgQtGdIl9iLDAe9uySfONUIexhERyorVLAgNsaSYt99KxBp0W4qWEInLU1KMBW2g4n0RJx7er7jiCmERPFtl9OvXT7p37y5XX3214DUJ70mBapUqk7UIQF7InJU7DKlBbH7JELK0s83kKyoZui1ENyHSdbakaAmRPGopZs9j6Ys4ae7pp59uvhxq1aoVl1K\/fn157bXXBK9JJkD\/KAJFQAASRCBCBEJDIDkEqxCha2yX3HDuHDcsChmiKpYhYq1DyA3rD3n0\/CrGU49zHNGSIfGWECFDhHIoUyU3EPBFnNu3b5dx48bJjz\/+KFdddZUMGjRIbr75ZmGMMzdg0lb6RQASRCBBBBJEIEEEEkQgPzcROsmQPAhWIUKZfnSAwJBIZIhliGARRiNEyBP505Gl0r53d8z2aoK0IeCLOEuWLGmWH2Fdtm7dWphNh0zTprVWnDIEICwEEkQgMwQSRCBBpMOEDZIqIsxzjRtChhAhAikiTusQIkSwDBGINWUAakVZiYAv4ixXrpwMGzZMlixZYhwXc6TrfuGFF8qMGTNEv1XPnGcDEkScJOgeI4QI47UIN+3c6wsESMtKUa3CvP0TKV0bVTFWIWToq3JNpAgkCAFfxGnrYmadLYIfeughefHFF81M+1133WU2b7Np9Jg6BCBABBJEsAQRLEEEAkQsCbotQtIg7jHConSNjyh3gDiJMG+\/VYg1iGAFOi1CrtUqTN2z4r8mTekHAd\/EiVXJcqT+\/ftL48aNBT+cxx13nOCTE\/dzfirTNN4IQIKIkwQTYQ1SpnfNv8c6rUEnETIhgkCCCMTnJMKpl1UVJxHm7bcKsQYRyv29Fj1TBDIbAV\/Eieu4tm3bSufOnYUdLdlGY968ecKypKpVq0b1x\/n5558L+WrVqiXNmzcXvMlHgmvKlClCGisMA+C6LlLaTAqDsJBIRMg6wWRbg2AFYSFOEszbbw16ESEkiJAGoVuMQIIIZVK+iiKQiwj4Ik4WuDOTztjmiBEjzLfqTBh5AcZnmOTp2bOnLF++XPLy8gTP8Zs3by6UbcOGDTJmzBgzAYXjkEzxuOQkRWsZQoZIrG4xX6bE0yUGNMgKcZIgZAaxIViCiLUGnRahkwTz9luD5IUEEcqlDhVFIAkIZF2RvoiTBfCMbTLG6ReB9evXy759+4ylyddFTZo0MWOiW7ZsKVQEZFqpUqVC4UEJgCARO37oJEa7hMaOE0KGiB\/dISvESYR5Pq1BiNIKBIhAgAhl+qlf0ygCikDREPBFnEUpukGDBsK2GxUrVhRc0s2fP9\/skFmlSpUCxbE2dNOmTXLHHXdInTp1jNPkqVOnmjwFEqb4AqLEioQkmVxBmEiBPL2IEdJCnGRoCQ5rEMES\/OT6GmZbAaxDtQZTfHO1OkWgmAgkjTitXnTvTzzxRPO5JmOX5cuXt1HmuGfPHilTpoww6fTll1+a7vzw4cPl008\/NfHuP4yD8o18MmTuohXyj8kL5bgBs423bKzISCTJDDILpPFxiLsuhC9NIMNX\/1ZFkIfalZe+TUsZaVpptyBHltwuCLozPMExW0Tbs0aCfC+z4f7w7iOWE9J59EWceD9iMoijU1muP\/nkE\/nll1+cwQXOGzZsKIsWLRL8eVpxJqhQoYKMHj1aWrRoIYybtmzZUk455RRZsGCBM1n4nDFQvlhKpJQ8+HAZMe8nYRH3Ix\/tEOeaRGs90oXGWsRC\/HxAC3mnz6ky4epGckO7hkYuaF7ffEnlVy8m1fymzYR02p6acd3\/VN\/TbLg\/vPtImAzSeFLCq26WIDGjzi\/pfffdZ35Rme228tVXX5nPLyN9RTRnzhzp27evmYWnDn4pWL60ceNGLsPCEidm6ZmtDwfmn6TCcYjtjtMNpwueX635D1kyZmiJkq50Xv6ESvPa5YU4k0j\/KAKKQM4i4EmcbMLGZmznnnuuYFlybNq0qRmH5MiXQ1iUWI1uBFnbiaW5ePFi82URS5OWLl0qTDI50zLxhLMQLFqImq2H6d670znzJOKc8UsIk+64LQ9SZDwSq5IjRGnj9KgIKAI5gIDPJnoSJ5uwsTSIiR1IkyOmspUVK1bIgAEDzGy5u766devKDTfcINdff72Z9ME5yLXXXisnn3yyYLEy3vnuu+9KtWrVzMTQ4MGDTTrGOlkfSn53mYm4tlammzCtdYmlmYh6tAxFQBHIXgQ8iRMLcNu2bcKEDrPezI5Dek6hK086N0ShUEg6dOggWJIQLUeuQ6GQWELGYQj5sF4Z\/yQdXXzruo64RAqked2kZcZBLeViYea1qWlmt9W6BBEVRUAR8IOAJ3HSVWdHS8Yy6bJDcG4hnHR+KktnGkuadpYc0sTKzGtTI51qad2KgCKQgQh4EieW4cSJE4VJHbrsq1evDn\/dg3WIEE66ILcd0qRr7iZNyDPIeqtuioAiEEwEPIkzmCrHrxUz5gg5IUssTY5cqygCioAiEC8CvoiTMU0mc1hS5BbCiY+34lSln7NyR4ExTSXNVCGv9SgC2YuAL+KkK06XnK45smrVKvnwww\/lmmuuMe7lWHpUdIiSl5Mu+nWTl4UryMufCFJLMwyHnigCikAREfBFnO6yQ6GQ8M15jx49hImjSAvg3XnScU33HPKkbpYZIZyrKAKKgCJQHASKRJy2Qr4z37lzp1ngbsOCcoQwmRBCH6xMFrRzrqIIKAKKQHER8EWcrNXs0qWL8B25U84++2yzJjPSl0PFVayY+YX1mrYMuuj2XI+KgCKgCBQXAV\/EyWeRjGfijNgpTz\/9tLDnEP42i6tIIvNjbTqXHmkXPZHoalmKgCLgizjxAH\/GGWfICSecYPYbat++vfCNOh5X8GgUNBid1uboLvWDpp7qowgoAhmOgC\/ipI3jx4+Xs846S\/CZyfX7778vLEWaNm0al4ERt7WZyE8pA9NIVUQRUATSioAv4uR7dRxy8BURfjPR+NJLL5Vx48bJpEmThP2FCAuCMJNu9VBr0yKhR0VAEUgkAr6Ic+\/evcbJ8JFHHlmg7tq1awtxODQuEJGmC6xN50y6WptpuhFarSKQ5Qj4Is6DDz5YmCB6+eWXw0uP8JSExyM2cCtdunQgYII4rSLNji64RYcND9xRFVIEFIGMQ8AXcR544IHGt+ZLL70k7B\/EkiT8ag4cOFB69+5tSDUILR\/21mYJYr0AABAASURBVJqwGl0bHR4+1xNFQBFQBBKJgC\/ipEI8JOEz89lnnxWWJI0dO1bw1s5MO\/HpFqxNuwSJHSa1m57uO6L1KwLZi4Av4mT7XhbAsxAeomQ5En45sUSDAg3EaXVpVruCPc3RozZbEVAEkomAL+LEyUf9+vXl448\/FsY2k6lQUct2dtN1fLOoKGo+RUAR8IOAL+LEiQebrfXp00eaNGlS4NNLa4n6qSxZabA2tZueLHS1XEVAEXAj4Is4mVHv2bOnPPjgg3LnnXeaMU7GORE+xSTeXXAqryFOW5920y0SCT9qgYqAIrAfAV\/EySeXbKDG2KZbCCd+f3lpOcxdtSNcr3bTw1DoiSKgCCQJAU\/iZDKILX6XLVsmdMlZhuQWwkmXJP18FTt35XaTDvdxOptuoNA\/ioAikEQEPImTLni3bt2M02K65HTN3UI46ZKoo2fRdNPt+Gb1CqU802pk8BBQjRSBTETAkzjpgrPsCH+bkbwjnXbaaZLurjrEaYG\/tU1Ne6pHRUARUASShoAncTprDap3JOcyJKe+eq4IKAKKQLIQ8EWcmeAdScc3k\/WIZGC5qrIikGQEfBEnHpBwWJws70isEW3btq3UqlVLmjdvLjNnzvTVbLrpOr7pCypNpAgoAglEwBdxJtM7Er48Bw0aJKwTXb58uVkjOnToUNm8eXPMZkKcNpGu37RI6FERUASSjYAv4uSbdJYlJcM70vr162Xfvn3G0mTvIr5MwlXdli1bYrZd12\/GhEgTJAwBLUgR+B0BX8RJ8mR5R2rQoIFMnjxZKlasKHwHP3\/+fIFAq1SpQrWeous3PeHRSEVAEUgSAr6Jk\/qxBJPlHWnJkiVy4oknCt\/Ds5dR+fLlqbKQMA66Zs0aQez4ZuU\/irkmLFNkw4YNGaezF7bant+eSS+M0hmXDfeHdx8pRAppCIiLOJOpH7tmLlq0SPD5aSVSfatXr5aaNWtKyYN\/d1Tc+rjDTRjhmSLsEJopuvrRU9tTsyjPYMryZMP94d1HIvFCqsPSTpxz5syRvn37ys8\/\/2zazi8KwwIbN24019H+FJgY0m0yosGk4YqAIpAEBNJOnJUrVxYszcWLF5v9jFiatHTpUmnUqJFnc50TQ54JNVIRUAQUgQQj4Js4d+7cKc8\/\/7z069evgLB0iLii6lW3bl2zn9H1118vderUkauuukquvfZaYU8jrzJ1YsgLHY3LZQS07clHwBdx0o2+9dZb5dFHHzVLhxKpVigUkg4dOsi8efOE8QuOXIdCIc9q1m3\/ycSrYw8Dg\/5RBBSBFCLgizi\/\/\/57wQv8k08+KUOGDBEWrFvBW1K5cuVSqLII45tISivVyhQBRUAR2I+AL+LES9IhhxyyP0v6D07S1C+G0n8\/VIMcQECbWAABX8RZunRp4z5u+PDhsmLFCvnuu+\/CghPjX3\/9tUChyb5wTgypx\/dko63lKwKKgBsBX8S5detWGT16tLzxxhvSpk0bwUenlQsuuECIdxeczOt123aHi8crUvhCTxQBRUARSAECvoiT7YFnzZplJm+YwHEK4cSnQNdwFeu37Z8YqlhKlDjDsOiJIpBhCGSuur6Ik+bxHTmfRTI5xBIkuuzsRUQ48akUnVFPJdpalyKgCLgR8E2cfAZ55ZVXyqpVqwQrc9u2bWb95cSJE91lJvWaiSGESqrlW5wcVRQBRUARSCUCvogTn5mTJk2SUaNGycCBAwUHHHy\/jPXJuCfLlVKltCVN6qte8Y8cVBQBRUARsAik5OiLOHfv3i14gYcsnVpVq1bNXLJA3pyk4M\/6\/QvfqUpn1EFBRRFQBFKNgC\/iPOigg6Rs2bLm6x7n0qOVK1cKS5WQVCnutDhTVafWowgoAoqAEwFfxMkC+F69epkvhjp27ChMEl100UVmjPOSSy4xpOosNJnnc1duDxffvHZkn53hBHqiCCgCikACEHAX4Ys4yYQD47fffltuu+026dy5s3HG8eabbwpOh4lPtVTXiaFUQ671KQKKwH4EfBMn6cuVKyft27cXPBl16dJFUr1+Ex2s13d17gEaKoqAIpAOBHwT56ZNmwSyZI8gNlSrV6+eIVA+uUyV4vtKVwpXpUuRwlDoiSKgCKQYAV\/Eyaz53XffLUcccYSZIGIt5+zZs4U9iO69915hxj0Veu8r\/bujkeq6FCkVkGsdioAiEAEBX8TJOs0dO3aYLS7w2B4KhYRdKG+66Sb55ptvhMXwEcpOapAuRUoqvFq4IqAIeCDgiziZVWfRO5anuyzczRHvDk\/G9d5K9ZJRrJapCCgCikBcCPgiTtZpdu3aVe644w5hJp3xTj67xCt8y5YthQXyuJqLq+ZYiSPEF+yql4qQQoMUAUVAEUg+Ar6Ik6742LFjZd26dcJnll3yZ9QhUQj04YcfFlzLIclW104OVa+oXpGSjbWWrwgoAtER8EWcjGtOnjxZPvjgA0+JXk1iYpwWZ2JK1FIUAUVAEYgfAV\/ESbG\/\/PKLPP3009K6dWthORIbquHgw\/kJJumSKWGLs0JRu+nJ1E7LVgQUgVxBwDdxPvXUU\/LKK69I7969Zfz48aZ7PmLECHniiSdSgtW6\/c6LqUzXcIKCiiKgCKQLAV\/EyRjnjBkzBAfG7dq1k4YNG8pll10m48aNE8KJT3YDnMRZXddwJhtuLV8RUAQ8ECjhEReOYoF7KBSSChUqhMM44ToUChmXc1ynSqrnTw6lqi4f9WgSRUARyDEEfBHnwQcfbEhz2rRpYsc02TJj3rx5xjMSbueSjZtzZ8tqOsaZbLi1fEVAEfBAwBdxHnjggcJXQniBP+WUU4S1m0wQ4Q0ed3OpWgBv26EWp0VCj4qAIpAOBHwRJ4rVqVNHpk6dKkwS5eXlyciRI83SJNzNER9JNmzYYByDHH300XL88cfLs88+K1iq7rRTpkyRWrVqhQVXde4F9U4\/nLlAnG6M9FoRUASCg4Bv4kRlnHpAlLiWa9q0qWCJEh5J+JoIxyCQ4PLly4V1oBMmTJD58+cXSg7BjhkzRlavXm1k1qxZUV3WKWkWgk8DFAFFIMUIxEWc8ejGBm84B4FkS5YsKbijg3Q\/++yzQsVs3rxZKlX63WVcoQT5AbolcD4I+l8RUAQCgUDSiLNC\/gw8n2keeeSRpqHbt283WwtXqVLFXNs\/P\/74o\/DpJp9wMhyAJcuQgLtLb5cjzcu3WNesWSOZLljZKW9DEnHT9gT7mcyG+2OH8yx3pPOYNOKkW1+xYkXB2ty4caPceOONwuz7mWeeWaC9e\/bskTJlykj\/\/v3lyy+\/NGtFhw8fLp9++mk4nSVNAi4853Rht81Ml6pVq2ZFO+x90PbUDPT9zIb7Y4fy4IF0S9KIk4bxmeZjjz0m5513nhx77LHyyCOPGPIkzgqW6ejRo6VFixaGZJmxZ+Z+wYIFNkmBY\/UsWfzu\/gEp0MgMvND2BPumZdv9STfaSSNOutpPPvmkcUP32muvCTPxkZYtLVu2TB599FFx+\/rEOo0EzqjBd4Rn363pnolH2paJehfW+bfVENqe33CIhk+6w7Pp\/tCWdEvSiJMxlXfeeUfuv\/9+oZsQraFly5YViJXF9CyuxwPTkiVLpFGjRuEs1SuWkm33t5LPbj9VNrz\/jJl5t2a7Hn9biaA4KA659AyEySFNJ0kjTrbU+OKLL4QugvPXlu\/bWaPJMqV3331XqlWrZhwkDx48WJgcYqyzX79+Urdu3UKQQKCFAjVAEVAEFIEUI5A04sRiXLp0aSHrsEePHmaNJms1cVFHe5lJnz59ukk7Z84cadWqFcEquYuAtlwRCDQCSSPOQLdalVMEFAFFoBgIKHEWAzzNqggoArmJgBJnbt73nGy1NloRSBQCSpyJQlLLUQQUgZxBIPDEyXpQvCrhXalevXqC45AffvghI26QX91ZhsV+TrQRT1LsIspnqEFrpN\/2OPXGhyu7BQTtnsXTlg8\/\/FCaN29u1g+3bdtWPv\/8c2cTA3Hutz0432EnB94lVrFcfPHF5pPnQDTCpxL4wLj00kuFZYs+syQ8WeCJk08vn3\/+ebPW86OPPjJbFLP3UcKRSEKBfnXnxXzhhRdMG3kpWZHAD4T7o4AkqBhXkX7bYwvlU1vW8fIFmQ0LytFvW9avX28+A37ggQcEL1\/nnnuu8DUcuyKYtgTkj9\/2sARw4cKF8u9\/\/1sWL14s+I4YNWpUQFrhrQYGBuu9\/\/nPfwpLHb1TJzc28MQ5c+ZMYc0n30SXK1fO+Pd87733BOcgyYWm+KX71Z2H4IwzzjDfOvN11dlnny2QDr+sxdcicSX4bQ81Qix8EXbaaacV+syW+HSL37bwQcZJJ50kfAZ8wAEHyDXXXCMDBgwwnwenuw3O+v22x+bh\/tjzww47zJ4G+sg7zzJG\/F\/wnqRT2UATJzd37dq1Urt27TBG3GQ2hwPEcGAAT+LRvVu3bsL6VtuMFStWCC+pl79TmzZVx3jag07vv\/++0D3HTwHXQZJ42kIP4KeffpJzzjlHGEa55JJLzI92KBQKTJPiaQ9rpytXrizs4ID\/CNZaX3TRRYFpi5ciGE58un3bbbeZrXy80iY7LtDEyQMLSfJZZrKBSHT58ejOA0Eb6YowLDFo0CDp3r27sNdTovUqannxtIfxWXYKwDr7wx\/+UNQqk5YvnrYwzIDDGbrqeO9q1qyZGWdnrDBpCkp8JcfTHoa5+HJv9uzZpqvOuPqQIUME8o2v1txOHWjixDVd6dKlhYfXeZuwxkKh4PziO3Wz5\/Hqjm\/OCy64QHCM8vjjjxsLx5YVhKPf9jBJQRuw0CJ9NptJbbG6du7cWerXry9gwCfE3377reCo28an+4heft4Txszp0tMLwE8uP9Zdu3aVr7\/+WvCXm+52ZFL9gSZOuqp8y84YoAWVh5auBlaaDQviMR7dGc+8\/vrrjfu9N954Q\/CUH7Q2+W0PQyjcLxxT46OgY8eOgrMXLLV0zoI68fTbFvIweYJFx7mVEiVKSCgUnB9uv+1hbLBUqVK2GeEjxEtcOEBPYiJQImaKNCdgFpMBYSwyfhVZttOhQwfP\/Y7SrHK4er+6v\/jii+b7\/Msvvzxwkw7hxuSf+GkPVszEiRON3wG89eD56qyzzpK5c+dKw4YN80sJxn8\/bUFT\/CbMmDFDGAuke\/7SSy8Jk0WxtnohbyrFR3vMuPmpp54qr7\/+upl8pD3sBcZ9wS9uKvXN9LoCT5wnn3yyMHiN5cJNP+6444S1dJkAvJfuffv2FYSxpZUrVwqb1TH5gJWGsJKAsaggtdNPe4Kkr5cufttCumuvvVauuOIKYTxw69atQu8gFAqOxUk70TPae8JzhpDur3\/9qzBBxFAK7aH7fvPNNwfKgkbPoEvgiTMUCgmLdFlzxuD8rbfeasaagg4s+oVC0XUfMWKEIIzX4gEf68wpWNmHHnooxQRGQqHY7XErizWD538sUXdcOq9DIX9tCYVCQg+H9YOsdnjooYcEP0k+AAAHIUlEQVTSPqMbCbdQyF976JZfffXVZmKI9rDONprT8Ej1BCGM94J9yXi20qVP4IkzXcBovYqAIhAbgVxNocSZq3de260IKAJFRkCJs8jQaUZFQBHIVQSUOHP1zmu7FYGgIpABeilxZsBNUhUVAUUgWAgocQbrfqg2ioAikAEIKHFmwE1SFRUBRaC4CCQ2vxJnYvEMXGnjxo0zC+39KBZPWj\/l2TR8dYMjYLxA4THJhsc6smgbnWKlS2U8\/iz5QIHPYpP1CSnlUj7CeSrbp3X5Q0CJ0x9OmqoYCOAtqXr16oJ\/zlQvhMfpCF6niqF+oaznn3++LFq0KGmfkLKwm09UmzZtWqhuDQgGAkqcwbgPxdICTz29e\/cWux0C54RhHQ0bNkxwJWatPb64wPrDasLTPN7M8T4VKe2yZcsEz0B8CspLTF6IKJKyfFnDp7CU60xLubjII56j2+KkPMolD\/W0a9euwNYUkC5biRBH+fjHpH50RnfaQJ20iXIoDyutffv2QtsJx+IFD3BhuwjqevDBB41TYqsPbtbwfERZ1IO+1OMl5AVXrGKL04UXXij4VYiUz21Bgw35KSdSeg0LLgL7iTO4CqpmsRHAaQMvH1si8Gkq3ntwTcc3yXyiioWE4w3IY\/z48YJvST7v5Bz\/n3x6506L9\/k+ffrI3\/\/+d7NlxBNPPCEjR46UOXPmFFKILSXwhvSPf\/xDVq1aJc60lIulCVlxdFuclEe55KEcPm\/EPyTtoSIsrzvvvNPo8Oc\/\/1kgPL6v5vPbN998U9hyhDr79+8vENiWLVvIJhs2bBC8hONQGf8GtBnfoGDEPkiffPJJ2DUcRDtgwAChXrCgHbQHfUxhMf68\/PLLwqeL6NSyZUszNALWMbJpdAYjoMSZwTfPqTrkQfeRb9\/5Bv6mm25yRptzPPrgKxOHEJAP1hkkG6krC+HwTTCejXA51qBBA7Nh2fz5801Zzj9Ya8ccc4zgOi4UCglp27RpIxCbM12kc9KQljzUg3WJA2TOSY8zCnxhco3eu3btMv5Z8fXJj0ONGjWMR3a+wd63b5\/QJvIdccQRgmMYwiFT9uTB9yTEjScgNpCj7aTFIm\/RooXZHoN6IPmjjjrKeEQiPpb85S9\/MdueUBd1ouPatWtjZdP4DEZAiTODb55VHYuyV69exjM51hXdRdultWk4QpY4FMEtGpYgVhIvOXFuwWJivx22V6D7ikyYMEG++eYbd1KhS0+9kLaNhKDoTtvrSEesyq+\/\/lpIa+MhNUgMa5EwZxzXVnbu3ClYokygQFZPPfWUIVQbjy7W9yQ6owt+XG08W7CUKVPGXNJW3BUyHEA7IWpcyZHPJIjxx7m1C3VSd4wsGp3hCCSTODMcmsxRny4iLvew3iBMxugY38OpsLMV+Makqzpr1izBSsQyhaicaew51hPjjXRX6dYjn332mdmozKaxR6w+LF5r7XHkukqVKjZJxCMkU7FixQLbNtDFxgMRxBgx0\/7Ahx9+2JzR5WbzPvaisWRrIhx\/DjnkEMG63LFjRziUeiBMAmhrz549wz5E0f3jjz82wxTExxKca9s0OD3GgsdytWHRjs580dJoeDARUOIM5n2JSyuIEBLEgiMjViRbKWD5QArOLizxvNikfe6554wFSTzhzrRYfV999ZVxQAwRQpxMZEC8pHUKpE0XHrIhLd1irlu1auVMVugc\/eim\/+tf\/zITKjjWZayTutC\/UAZXAHXhzxQH11jDkC3XrmTGDdzhhx8uzzzzjFAHHvdJb9PxA\/HWW2+Zrjll0o6\/\/e1vZksJm8brSFefMil77NixUrVqVdN1d+fBfRtWLHVA2kxmudPodWYgoMSZGffJU8srr7zSjO3RBWdmnQmV22+\/3XjJh9S4hgiYNa5Zs6YQRledc7YivuGGGwSnyYTbtIwR3nPPPTJ48GCzuyOkydgjs9RuZRh7ZK9rHPzS3b3xxhvllltuEcLdad3XWMeMEdLdxrEuliDWI6TqTuu8xqckY5cMETB2STlYvsxcQ2DOtFiiA\/Inf5jtJj0OfxmTtWloExjgrBj9mRhiGIBxVJvG60h+7gFlUwd1USeY4pCa2XPyX3XVVYLVzg6Tbh3caUmvElwEMo44gwtl+jTDkmG8khlhuplsh4DVg0ZMumD9MatO19mmY7lNp06dZODAgYLFykSQMy1ji0ySTJ8+3XRhSc+MdyhU2PN5KBQyXsVJg7XITLkzLSRN\/ZSJTk7ByoUEWQ2A\/sy8Q9qkwYru0aMHp0ac5ZCGdtJedCQOK5J6GjduLFhztImMWHgMCxBPevSjbVi11B8KhcLOitGf8ognrx9h3SV5KBud0I181A+26MY1G6RNmTJFPvroIzNxdtdddwn6gos7LelVgouAEmdw741qliAE9uzZIyzLYt0nwxR07SEslg6x0VmCqtFicgiB\/wcAAP\/\/qDNNZAAAAAZJREFUAwB0tSIzichHaAAAAABJRU5ErkJggg==","height":337,"width":560}}
%---
%[output:5b4d7b44]
%   data: {"dataType":"textualVariable","outputData":{"name":"idab_zvs_min","value":"   1.640000000000000"}}
%---
