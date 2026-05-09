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
open_system(model); %[output:758e3eaf]
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
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAXkAAADjCAYAAABgiWSsAAAQAElEQVR4AeydCdxNdf7Hv1cpyfbYaqIm21iyVsIfSYg0SobJ1iplpmVKeEZlKFRKpbEUjUILLaMwUiLZGqFBirJWlnYUWcry9\/7ldzvPde9z73meu5xz79fL9znn\/H7f3\/Y5537O93zP73x\/BY7oP0VAEVAEFIG0RaCA6D9FQBFQBBSBtEVAST5tT60OTBFIEAJara8QUJL31enSzioCioAi4A4BJXl3ePlO+5tvvpFmzZoZYT9RA3jqqaekYsWKwjZRbTjrZSzJGJezzXD7e\/bska5duyYc33BtxyvNjoHzx1g4dlM355yybN2US6Tu6tWrpU6dOjJnzpxENuOLupXkfXGa\/NdJfvCx\/\/DzPz6ICYLih80PPP81xlbD888\/Lx9\/\/LGMGjVKypYtm2uhvPQRkgLHPn365Fp3fjKXLFkiSL9+\/WTEiBH5qcozZWvVqiV\/+ctf5K677pJkXg+eAcDRESV5Bxix7NofKj88K\/wQbVlLbq+\/\/rqx8NAJJR5rhZKHQE7USx1ckOjzo7Z1ocM++ZGEfPSsOPvkLOOsnzZoizR0ItVh+4vlzD661E9b1MGxU6jn4YcfNklsw+mEK0856mRLYatDGuLEiXwre\/fulZtuuskQ1e7du+WKK64IWnC0TVnEOVbKgjl1koe88MILxiInjTx0wIZy5CPURzoCFpMnT5ZzzjlHKlSoQJKQBk7oIpSlDuqL1MfQcTrHTxkqnjp1qrmeqCe0DWd\/0Q2VUH3bJ\/Roy7bBubrjjjtIPk5C6wjX5k8\/\/WT6yLgRxmUrAgPaJR1x4mjzSEPQIw1hH30w5fywT59tveyTZsXZZocOHaREiRLy4IMPCrjZMpm2LZBpA87PeLlQ+EFg9TjrIc15cZHXu3dvQzrsQzy33nqrIQB+LJ06dZItW7aQZYT6qIP6TcLRP\/yo+dEd3TX\/2Q9tw2Qc\/cOFTv7R3eB\/6oukjxL1I+wjeamDcnmV2rVry5lnninLli0zuFDPhg0bpGjRotK4cWND0oyBdCvhcLJ54baMyTlGzkP37t2Dlt2gQYOC54jyAwYMyHFeIBn0KUc+Qn0QEfsffvih0b\/wwgulSJEihkggSee5pSzn\/rvvvqPIcUIbWJvODM4lfXem2X3qieX6sfrhrjf6xLho2+rltg1XR7hzMXr06Bx4Dh482JxbyoMB7dp2nDg600jneMeOHeIsA6acH\/KsgBFY2WO2XDP2ui9cuLCUK1fOPGlt3ryZ7IyUdCX5hJxMLmwEcmK7adMm4RGXxp555hnzI2cfadiwoUACCPtcpOxzEbNPOcqTRj71IZRFILtp06YJOlgkpEGCbJ3CDwhr0qlP3ei8+eabbMKK1V+1apWcdtppkpc6wlZ8NLFXr15BXOjL8OHDj6bm\/I9ro379+rJr1y75+uuvDRlA+FjF5IEnJSgPBmBjcWefPCv8mMeNGyfgaMfF\/oIFC8xNw+JIXRDN4sWLg+1Zfdog39bJduLEiYI+dZNPu\/Th7bffNjcKi2\/lypVRF54otm3bZvrBeaUM547zzbmjHvpl22zZsqXQF9qgbfTpK\/n0HV3KUDn1vPjii0Kb1Gf1aQc9+oag6xR7vaGDLm1QF20yvl5Hz5VtgzppgxtWuDooR3naAQfcVE7ytG3YfHtuaZc+h5bnfHP92rYYN+Pnmvzll1\/MDdTWSR3sW13Kcc3SD9qjX3YcXDsYTIzjjDPOMOeQa8yWzbStkryLM84PFfUuXboE\/a9cuFxo\/Lj5kZOPWOuOC4190iiPsI8FwiMmFi0XKWlOgeysC8CSiDPf7nPx8mNy6vPD5aIPR662nFM\/r3XYuvK6bdOmjfkBQnT8iCECsAoEAgKe4Aq+1A\/xgzv7FkP2Iwm4Q1gQBjcxHvfB3OqHGzNPEBANOrgetm\/fzq5xBXGuIBn6SOK+ffuEfPSpnzRuNliOnE\/OK2UYI+cCQkcnVOy5YssTAq4mCDhUzx7bsTMW6qcd2rP5oVurf8MNN5inDfKvvfZac\/Oj\/5AhabmJrYOxoMe5mD9\/voAtvm\/SEM4duFscSEMYOxhwPWJ9O3Ek34rzmrRt2jqpl32ra88f54P6wAIr3ubr9jcElOR\/wyKpe1gdXPhO4ceQ1E6kuDEICiLnB41AmBBtvLoFafLjhxAhJSxVt3XTJ6xL53mC3GrUqHFcVRARNxZIl3GhAPHgV47kGsG1QB+5CdE\/2qJNyuYmfrp+GDsYIJxfJz65jTGWPAgeA8F5fjgHnAtuYNzIwNPeiHOtM00zleRdnFhrUfOYyOMiRe3jMBYcFgxpCI\/bXGQI+6RRHmGfx262+RUuXl4uOR+dsZYgDrax1B9rHTwxYEFRZzz6j0WIywb3Bz9Ma8mBI3hipYEv7YE3uLNvMWQ\/kqCPOwASwPfuPA+UCTdmniisFX3qqaeKfdQnnTLRxJIZfvmZM2caVxtkTJ2R6gBHSMjOzkEP\/Uht2bFTLpKOM93qWxcGebhpaIPxQYak5Sa2Dtsm2HJTQtjPrSx5dkyPPvqoYPlDypxb8iKJbZPfDucOYd\/qhzt\/Ns9ucSXxu2jVqpVp16Zn2lZJ3sUZhzAQLlC2EClWIlU4H4c5xlrBUkXYx7JjH0uGHzXkRXkrWDmQBGXdCESJG4MfLY\/61EefaIO2YqkrWh3kQ8bONuh\/LHXTFyzqSLq4AKh369atwuM4pIOAJ2Uoz5jA2+LOPnnhhLrAYdGiRSYb7MEdYd8kHv0Tbky0dTQr+J++cUA6fbACueGagyRpz974cK9xo6Id2kMfS546LGmxTxn6iBXvPEaftkgLFfBmRsu5555rXC0co28l0vWDu4trz9knynJ94LYJbSfcsa2DcrQH\/pwLrglwDFcmXBpYUJ5tuHxnGvg5+80xY7A6tEv7FkvqtWKvN24u6Mc6TnTTUZTkXZxVyAfLjIvcWYy0UFfLLbfcYl7AoccPylpqWDJYrlzA5CHsk0Yex24Ffy4\/RFuO9pi\/7aa+aHXgSqCftMGWMdMOx+GEG0xu+bYMP17qQ5cyNh08acMeswV30jgPHDuFNG4SNq1YsWLinI0BPpQlH4sQyxALnzpJQ5gNQl\/YR+hDqPsEfax0SMbeBHA1oU8fRowYYWYNcSzy61+woy7ynX0klzzbJtvp06eb8ligWKIWH3QR3ERcK+hyjLBPWrjzTT9feeUVUye6CFi7uT6og+uXcpRHwBMfO\/vRhJk84IYedfDUxjE3Cqx60kMltE3GyPlx6tE++DnTOCYdg+nJJ58U+\/Tg1Mm0fSV5l2ecHyoXqdMHyA84tBoe960ePlznD5ALGB+xrYN90qgDPfQpS1ukQcDosuU4nHBho4NQnnrQo17qR9gnnXxn\/eghkeogj7LUQf1sGTP1UIZ8+kYeW45tO6RZHdJDxdZLXZRx5tMG5a04+2zL0Rf2KUfbVpeyiD2mD\/aYerDEL7vsMjO9DiJHDxcAxENdVugTfSMfoaw9L5aA7U2DMvSFPqFrhX6Rh7Bv0+mPU59yNWvWFLa0SdvOfNu2M4260CeN+sMJeeigi9i6rS79IJ2+2bTQLX2hHHoIeFodypHGljTwoa\/oU84eo0MaBE8+x7SNDumkoUsdPOXwtMOTETcC+r9ixQqycghtUo8VjlGwdVI\/x5ksSvKZfPYzeOwQX+jjvnUjYG1bsskNIurAVWat7tx0Nc8dAvYGiouGfVwxuIt4EnA+8bmrNTO1U0bys2bNEl6IRHpx079\/fxMLhZOLRPI5Zthp0+HGEQGsUdwOzip53LfWoDM90j6uCKxNPtyJdC1HKqvpkRHgBppfN1Pk2jMrJyUkj69x6NCh8vPPP4dF+8CBA+bDktdee83MUOBRjMc5HsHCFvBQIgRBf9l6qFvalQgIQPScLytuzxsWP24G3AkQU4RmNDkPCIAnuNpz4xcOyMNQE1ok6STPC68HHnhA2rZtK7wcCzc6PkRhul5WVla4bE1TBBQBRUARiBGBAjHqxUXtyJEjgtXDHbp169YR6+RG8P3335svDStVqiT4TidNmiSHDh2KWEYzFAFFQBFQBI5HIKmWPI9bzETgY5GTTjrp+N4cS\/nhhx+kZMmS8vjjjwvT08aMGWPilM+ePfuYRs4NPnuVijneYSgeiodeA8m\/BnBF52Sn1B8ljeR37txp5qzedtttUqZMmVxHju8dy7169eoSCASMJY\/lP3fu3IjlrN8unbcMPp3HZ8eWKeNkvP4c66bguzLGEItkyjhbtGjBUD0lSSN5vmhkvitTzrAwmAPL1DPmzDIn1onKvHnz5KWXXgom4eZBmD4VTNQdRUARUAQUgagIJI3ksc5x19i7Ph+gMPWMebChHyzs379fnn76aVmzZo1A7suXL5d33nlH7BeGUUeVpgq5Pcmk05AzZZycs0wZa6aMk3PqNUkayUcbONY8MUGYa3zJJZfIjTfeKMQv4cXrnXfeKX379pULLrggWjWa708EtNeKgCKQIARSRvJY9jNmzAjGZceaZ04sM29OOOEE6dy5s1llBsufYFPt2rUz\/vkE4aDVKgKKgCKQlgikjOTTEk0dlCKgCCgCHkMgbUneYzhrdxQBRUARSAkCSvIpgV0bVQQUAUUgOQgoyScHZ21FEVAEPI9AenZQST49z6uOShFQBBQBg4CSvIFB\/ygCioAikJ4IKMmn53nVUXkDAe2FIpByBJTkU34KtAOKgCKgCCQOASX5xGGrNSsCioAikHIEopL8vn37hIBhM2fOFDdCmZSPLg07oENSBBQBRcANAlFJfvfu3XL\/\/ffL4MGDZdiwYTEJupRx0xHVVQQUAUVAEYg\/AlFJniZZx5KokCz4EYugSxnKqigCioAioAikDoECEqXt4sWLmxWaqlWrFkXzt2x0J06c+FuC7ikCioAioAikBIGoljxL8fXo0cPEcsdC\/\/bbb6N2tGDBgmb5vqiKqqAIKAKKgCKQUASiknypUqVk6NChwgIf\/\/znP6VBgwbCElexEn5Ce6+VKwKKQCoQ0DZ9hEBUkie2e5MmTQSCX7lypbz11lvCoh64Y5TwfXSmtauKgCKQkQhEJXknKhB+lSpVJDs7W1jIY+HChXLVVVfJK6+8Ig0bNjT7O3fudBbRfUVAEVAEFIEUIlAgP22XLl1aqlatKmeddZbgh9+xY4ccPHgwP1Vq2QxFQIetCCgCiUHANcnz4vWll16SK6+8UmrWrCm33XabnHbaaTJ58mR58803pUyZMonpqdaqCCgCioAi4BqBqCR\/5MgR2bp1q4wZM0bwzeOH50VsjRo15OWXX5YVK1aYF7P16tUT3Dmue6AFFAFFQBFQBBKGQFSSx3Lv1q2bPPXUU9KsWTP597\/\/7Q9iTxhkWrEioAgoAv5BICrJFy1aVJguqRa7f06q9lQRUAQUAYtAVJInds1dd90la9assWWiblevXi3t2rWLqqcKioAioAh4CIG07EpUkmfUzJhZvHhxzFEo0aUMZVUUvY7Y9wAAEABJREFUAUVAEVAEUodAVJIPBAJC\/JoXX3wxpgiURKpElzKpG5a2rAgoAoqAIgACUUmeKZFTpkyRWKJPOnUoQwMqikAmI6BjzxwEJi\/7Sna1Hy91h\/xXFm3Y5ZmBRyV5z\/RUO6IIKAKKgA8Q+GLHfk\/1UkneU6dDO6MIKAKKQHwRUJKPL56Jr01bUAQUAU8i4DUL3oKkJG+R0K0ioAgoAnFC4KySheJUU\/6rcU3yv\/zyi0yaNEmaN28u5513nixdulTuueceM73y0KFD+e+R1qAIKAKKgCIQNwQKiLira8KECfLss8\/Kn\/\/8ZylRooSJPnn++efLoEGD5JlnnnFXmWorAoqAIpAmCHyxY58nR+LKkmcpwHfeeccEJPvTn\/4khQsXlhNPPNFEpBw+fLjMmTNH0InnSPni9pprrjGxc+JZr9alCCgCikCiEPCtu+bAgQOyZ88eIZ5NKDglS5aUvXv3CjqheXk9JgLmuHHjzAIlea1DyykCikD8EdAaj0dgy7Gpk14ieHrpypLnK9Zy5coZ0nX639mfP3++lCpVKuwNgIbyInPnzpW1a9dKixYt8lJcyygCioAikDQEvti5P2ltuWnIFcmffPLJcuutt5o48t27d5ft27fLY489ZkIQs+brHXfcIaeccoqb9iPqbtu2zUS\/pD1uLhEVj2VUrFhRNm\/enNZCXP90HyPjy5RxZtJY0\/2cnl3zAvnCWvJZhY6xkjc2rkieLrMa1LRp0wSSb9WqlbHeWed15syZUrduXVTyLczgYZGSyy+\/POY6N23aJBUqVEhrKV++fPqOz3HuMmWcXK+ZMtZ0H+fzr88O8l7jylnBfS\/suCZ5Ol2sWDG5\/vrr5aGHHhJeuLIEYNmyZcmKi8yYMUN+\/PFHM4MnLhVqJYqAIqAIJBCBxRt3BWtvXKlEcN8LO65InlWiOnfuLBdeeGFEIZ8ZOPjp8zrA9957z8y7r1q1quCGmTp1qjz88MPStWvXvFap5RQBRUARSBgCizfsNHUX2PudNKnsY5JnBs3tt98u+OaZQvnAAw+Y6ZS8GN23b5906tRJLr30UvnHP\/6RrznzPB3gfrHSoUMH6devnxDC2CAZ0x9VUgQUAUUg8Qgs2rBLFm381ZI\/8btPE9+gyxZcWfI\/\/\/yzTJgwQe69917529\/+Jk2aNJGmTZvKwIEDpX\/\/\/rJy5UrjYhk6dKiZM++yL6quCCgCioCvEOBl68NvbQ72+aQvFgf3vbJTwE1H+DDpyy+\/FCz60HJVqlSRb775RtAhnznzoTp5Pcay79WrV16LazlFQBFQBGJCwK0SMeStFd\/kqC\/e95Y8H0ExF5458U6fO\/uLFi0yX8AyhXL9+vVm3y1gqq8IKAKKgF8QGPbWZzLsmBXPB1CjulT3ZNddWfIQeJ8+fcw8eeLV4Je\/+eabpVGjRsKUx549e8ry5cuN6+bKK6\/05IC1U4qAIqAI5AcBXDTHEXzn6gLR56feRJV1RfJ0gnnyb7zxhjzyyCNSu3ZtM08e\/zxfp7Zs2VLIZ3YNs2zQV1EEMhoBHXxaIQDBXz5mRQ4Lvkv933luRo0TdNckT+EiRYoIhM4LV2bYdOvWTXgp+8QTT5iZN4Q+QE9FEVAEFIF0QAByv2XyWrN+K\/uMCct9+l\/rSXbrszn0rLgm+Y8++shEnQydK880x7ffflvwz3t2tNoxRUARUARiRAAy58Xq5aNXGHJn3xbtUv90WXlvI8+6aGw\/2boieSJMjhgxQrDUmS\/PLBp88ldffbVJw5LPysqiXpWEIaAVKwKKQCIRgNzxudcd8l\/BerezZ2jTWu+jPfqSlT6GiiuSJ1Y8Uyh5wdq+fXv5\/e9\/b4RjiH78+PFy8ODB0Db0WBFQBBQBzyIAqfNBE4QOsSN21oztNNMjcc1gvXvti1bbx0hbVyQfCATMSlBUxmIhWPLfffcdh8I8ecIC79z56+e9JlH\/KAKKgCLgMQQgdQT3i3XF8DKVY9Jtd7Has1tXEMh9+i31PP1y1fY53BaSD5ceNg1XDNb7yJEj5bPPPpPKlSvLf\/7zHxNM7NNPPxXcOSeccELYspqoCCgCikAqEIC4sdRxwVhSx1rHcne6YugbxI6\/HWLHas8++lLVb5Y743CKK5LHes\/OzjZL\/i1evFjatm1rvnAlxDChDggghnXvbED3FQFFQBFIFgKW0LHKIXFL6ljquGDCkTrEnn3MYofY8bf7ndideLsieQqeccYZZr1Vpk1i2T\/\/\/PPy\/vvvy9KlS02MeXRUFAFFIM0R8MDwIHTIHAkldI5Jj0TqTmsdYs9OA4s90ilxRfLEpmnXrp2sXr06WF\/BggWlTJky8vnnnwt56AQzdUcRUAQUgXwgAJEjEDYCeWOdl+w9z0xr5BghL5TQaRYrHULHOrcuGEid43Sy1hlrJImJ5G0ceebCr1u3TphNEzpPvkePHmaOPC6dSI1puiKgCCgC4RCAyPGbQ9aQNkSO4DtHSEPID0fm1BlK6Dsea27mskPoEH2mkDpYOCUmkucLV+bDs94qK0Bdd911gm\/eKUOGDDFrsqpP3gmv7seOgGqmMwLbfzwooUQOaUPk1irHb06aJfLcyJwpjRA3BI6FroQe+eqJieQJTNa8eXO54oorhDAGV111lVx22WU55JJLLjEfREVuSnMUAUUgnRFwkjhEDWFD4lji7SZuNe4VJ5GjkxuRW8s8HJkzpRGCJy9TLfRYr6WoJM+KT\/PmzTPL8RF4jLVX7fJ8M2fONOl2ix76sTaueoqAIuAPBCBwxLpUmI7oJPFQa5w8S+KUizRKiDzUKscyx2+OQOSIknkkBKOnRyX5PXv2yNixY2XYsGFRBT30ozebFA1tRBFQBGJEACK2BA45Q9KItcSxxhFriTMdET0sccpGagYSR9pVLyIQNYRtSdy6WEKtcrXMI6GZt\/SoJM\/MmSlTpsiCBQuiCnro560rWkoRUATijQAEjORG4JGscDck7rTGLZFbEsciH9SytJAO0UPiEH+8x6r1hUcgKsmHK0bogpdeeknuvvtuue+++8x6ruqmCYeUpikCiUMA8kYsgTtdKJEscKxzyBuJZoXTc8jYEjgEDVEjWONOEnda4+hB5JT3naRhh12T\/KpVq6RNmzZCxMndu3fLxo0b5c4775TWrVsLYYhF\/ykCikC+EIC4EUveEDLkjFjyttZ3JBdKrAQOiUPKCOSNQOBY35bELYGThx6iJJ6vU5zUwq5Intg0xK0h4uTChQuF\/UmTJpmvXZl5QxhidJI6Am1MEfAJAhA3Yskby3vQnO9MONvcyBtyh+gRN+QdyQJ3Ejj7kDcCeSMQOOTvE1i1m1EQcEXyhBr++uuvpVmzZuIMRMYUS6ZQkodOlDY1WxFIGwQgbWTRhl0CCSOQMgJxXz761wUnIlneM9buMeViIW9Ag3wRS+DZrSsYXzfWNwJpW4lkgVOeulQyAwFXJM\/XrAhumlB49u\/fL4cPH5ZAIBCapceKgK8QgLQRJ3FjdcdC3OhA9AjEjVBXNAAgXiSUvLGwLXlb9wkkbgk8u\/XZZtYK1jdCHdHa0vzMQsAVyfM1K5En77nnHpk1a5YwXRL3zJIlS2TAgAHSoEEDKV26dGYhqKP1BQIQLZJX4nZOGcwrceMKyQ6xvGdcW15yI2\/KKHn74hLzbCddkTyjIKQBIYUJaVC7dm2pXr264KNv1KiR9O7dWy15Sfg\/beAoAhA2AmkjWM4IljQSzVWCTn6J263VjVWeHWJ5n1HsxKOj0f+KQOIQcEXyfO3KIiEdO3aUFStWmBDDhBles2aNDBw4UIhxk7iuas3pjgCkjXywbb\/xU0Pa1k0CKUPczCYJ9W\/bD3TQoQyCtY1QXzTccHEgkDaC9Zx9zOKGmHGXIKEWd6jLhHJqdUdDW\/OTjYArksc1M2HCBDn\/\/POFiJTMsClUqFBwScBkd17b8zYCECzitLQjkbaTuG+a+pWZcQJpW2vbEjf1xTJqSBuxpA0BhxI3vu1YibtJ5RKxNKs6ioDnEDAkH2uv+Jp12rRpgg\/+j3\/8o1k8BMLv1KmTvPHGG8ZHL\/ovbRGAYJHcSDuStQ1hI3klbUCFtJFwxI2ljYQjbqxxJPuYq6RL\/dMF0qYu6lVRBNIZAVckb4GA7IkpP3v2bFm+fLlcc801Jq4NkSl10RCLkre3kLWV3EjbukeclrbTPRJK2rG6SEAHkkVCSdt+Au8k7VCLG9JGLHFD2gj1UbeKIqAI\/IpAnkieorhu5s+fL8y0IbzB3r17hbnyhQsXJlslyQg4CdtJ2ljPCBY2YkmbrZXcSJt6Yx0KBIuEkjZkjEDayMp7G4WdUYIOpG2DWSlpx4p8KvS0Tb8g4IrkiU\/z6quvypVXXik1a9aU2267TYoWLSovv\/yyceFA+PryNX6nHoJFLGnz4Yz1aUPYCEQdamU7SRtfNoKFjVBfrD2EsJH8kjbuEQTSRqgz1j6oniKgCOQPAVckz0dQvHitUaOGIXZm2AwdOtRMo3R+ARupSywjyI2hSpUqgrBPWjj9\/v37S8WKFYNSp06dHGvLhivjlzSIFrHkDQljbSORiJs8PoG37hEIG6GeWMYNsVrJjbStlR3NPaKkHQvqqqMIpB4BVySPL54XrxB7vXr1coQ2iDaUI0eOyOjRo800yw8\/\/FA++OADU4Q08szBsT+4gnD\/vPbaa7Jp0yYjBEarVavWMQ1\/bCBgS+SQtCVwrG8kksVNuWgjtITNNhxpT\/9rPUEsabO1Yqf+WfcIhI2k2MqONmTNVwQUgTwg4IrkA4GAK2IXxz\/CE0PUTL0k1k2xYsWEr2fXrVsnP\/30k0NTzPGuXbskKysrR7qXDyBmLHLcKZC5daFYIicvFss7EmlDyOM6nC4QtdPK5jgcaUPYCPV5GTftmyKgCCQWAVckn5+uEBIBy7x+\/fqmGj6sYtrlBRdcYKx7k3jsD+ESvv\/+e7npppukUqVKQhmiXR46dOiYhjc2ELsldSxzrHXcKZB5uB5CuAhWMwJxI6EWdzjSRv+8coWE8uHq1jRFQBFQBMIhkDSSdzbep08fqVu3rvClLHHonXnsE8mSm8Ljjz8uGzZskDFjxpg5+UzZJD+c4L\/fvHmzOCUR+4tXrZf+U1ZIq0f\/KxB7OFLnU3WEWSJMB8QCJ0bJa91OF6RPw0KCNCy9T5ByJ+yUQz98GbXvW7dujaqTiDEnu85MGSe4ZspY032c8A8SjptSneaK5LG+WSyEExbacdLIQyc0L\/R4+PDh8umnn0r37t3l5ptvli1btuRQwfeO5U5cnEAgYCx5bgZz587Noec8wHdfoUIFSZScUPx38vK6gLSbuFXGLt0lfHpv28e6xtK2VvlHg5oKMrFnfbm9bS3p2KS6NK5TJd99K1++fL7rSBQ+8aw3U8YJZpky1nQfJ\/yDWE7w0rZALJ1h6uS8efPMV61MoSR+zcyZM8Up\/\/73v2XGjBlCyOFwda5du1auv\/56sWE2efAAABAASURBVB9LFSxY0MyrZ8rlrqP+d2cZ2mJ5QZvGi1mE6Zo2LVlb65KxVrtt10ns+MUheIgeP7jV0a0ioAj4DYH0629MJI+PfOzYsTJq1ChD0hMmTDBfuA4bNiy4nT59urA6FG6WcDCdfvrpsmPHDuFmgG+dGwfz6yHus88+O0cRbhRPP\/20cedA7nxV+84775hlB3MoJvgAgufFKS4Z2xTknt26gnkBaond5ulWEVAEFAGvIRATyTN1csqUKcKL0yZNmsizzz4rCxYsyCG4Upj3zqIi4QbJTBmmXmLt\/+EPfxDCFBPB8qGHHjIfVM2ZM8esOIWlz5ezN954o9xwww3mxStryPbt21d4SRuu7kSk8UIV6x2ip34nuWe3znlTIl9FEVAEFAEvIhATyduOQ\/bjx483Hz\/ZNDdbvpJlRg2Lf69fv164ceCXpI6WLVsKYRLKli1rpml27tzZfEWLn2vRokXSrl27pMSqh9QheKf1nn3Mcs9WcudUqbhDQLUVgZQiEJXksawh2NWrVxtXTbNmzYJfofI22SnkoZ\/SEeWjcQj+1slrxRK8td6V3PMBqhZVBBSBlCIQleRLlSolEydOlGrVqgn7vHgl1HA4IQ+dlI4oj41D8JC7neMOwU\/\/az1Rgs8joFpMEVAEPIFAVJInJg0vU5kNwz4uG1wq4YQ8dDwxMped4ItUhGKW4Nly7DXR\/igCioAiECsCUUneWRHBxPCVX3jhhRJOyEPHWcYP+5A7Vjx9hdix4NlyrKIIKAKKgJ8RcEXyzGnn46Xs7GyxwiwYXDlEqLz44ouPC1HgdXBw0xCOgH5C7KM6V9fQAYChoggoAmmBwK8kH+NQCCzWvHlzYQUoK6wKNW7cOHnmmWeEYGMFCriqMsaWE6P2xY79wotWW3uX+r8T\/ZjJoqFbRUARSAcE4sbI5cqVk+3btwtxZ\/wCzOKNu8S+aCVcr75k9cuZ034qAopArAi4Inm+VMXnzjTJUHn77beF\/JNPPjnWtlOqhxWfw03TpXpK+6ONKwI+Q0C76xMEXJE84X87duwoDRs2PE4IcdCjRw8pXry4L4ZuX7TSWdw0+OPZV1EEFAFFIJ0QcEXyzIFnLnzoHHliy7AgCOEI\/AAOVjwzaugr5K5uGpBQUQQUgXREwBXJMweeOfMfffSRiVvDXPlChQoJESNJI5iYH0ByWvHZrSv4ocsZ0UcdpCKgCMQfAVckD4mPHDlS7rrrLnHGjScWDW6cadOmxb+Hca4x1IrvUv\/0OLeg1SkCioAi4B0EXJE867QS633EiBEmQiTDKFasmLAICDJ58mTPz65hRg39RtSKBwUVRUARSGcEXJH8wYMH5fDhw4LLJhQUYsLv3btXDhw4EJqVuuOQlrHirasGX7xa8SEA6aEioAikHQKuSJ6ZM8yHf\/HFF4VFPywaTJ2cP3++CWDGIiA23WtbSB6hX40rlWCjoggoAopAWiPgiuSZA48\/fsWKFWYBDxu\/5vzzz5fnnntOWNyDr2K9itjDb20Odq1L\/d8F93VHEVAEFIFjCKTdxhXJM\/oqVaoIqzuNGTNGWrRoIRdddJEMGDBA+BiqTp06qHhSsOCdX7c2qayWvCdPlHZKEVAE4oqAK5InbEHbtm3N2qtNmzaVgQMHyv333y8dOnQQXsDGtWdxrsz5wrXLBWrFxxlerU4RUAQ8ikABN\/1iXnz9+vWFpfuYTummbKp1Jy\/90nRBX7gaGPRPEhHQphSBVCLgiuSZPUNnBw8eLN26dZO77747hxDawDl\/Hl0viNNVc1ZWIS90SfugCCgCikBSEHBF8vSocOHCJtQwUyY59oM4XTX99AtXP5wy7aMioAjECQFXJI\/fncVCHnjgAQkn5KETp77FrRqnqyYtXrjGDRmtSBFQBNIdgagkz3z4hQsXCl+7ss8XrzNnzpRwQh46XgJNXTVeOhvaF0VAEUg2AlFJfs+ePTJhwgT56quvhP2xY8cKvvdwQh46yR5Ebu1B8ja\/ceUsu6tbRUARUAQyAoFjJB95rGXKlJHx48dL9erVhf0pU6aYCJQLFiw4bkseOpFrS36O0x+vX7kmH39tURFQBFKLQFSSD+3e1q1bpXPnztK3b1+TxQpRrVq1kt69exuXjkn00J\/FG3aa3jB1Uv3xBgr9owgoAhmEgCuSJ\/jYkCFDjEV\/xx13GJhYSOTxxx8Xwg0\/+uijQhAzk+GBP7hq7FeuOnXSAydEu5BWCOhg\/IGAK5L\/4YcfZMuWLXLjjTcKgcoYIguJ1KxZ03z5ysIhvKAl3QsCydt+qD\/eIqFbRUARyCQEXJH8iSeeKCeddJL89NNPx2H0888\/yy+\/\/HJceioT1B+fSvS1bUVAEfACAq5IPisrS5o0aSJ\/\/\/vfZdasWYI\/Hnn\/\/fcFN06DBg2kdOnS4pV\/6o\/3ypmIsR+qpggoAnFHwBXJBwIBuf322+Xqq68WPnxq2LChIN27d5dzzz3XvHwNBAJx72ReKsRVo\/74vCCnZRQBRSCdEHBF8gy8YMGC0rNnTyGmPBY8smbNGhORskiRIqh4QiB52xH1x1skdKsIKAKZhkCBvA6YF67MiUcg\/rzWE64cES6JWc9TQsWKFc3TAsekh9MPl\/arP\/7XHJ0f\/ysO+lcRUAQyD4E8k3wioVq3bp3885\/\/lNGjR5upmXxdO2LECCE91nbVHx8rUqqnCCgC6YyAJ0l++fLlUrVqVWGlqUAgIOedd56Zm8\/0zVhPhvrjY0VK9RQBRcCJQLrte5LkiVU\/atQoYcomLpoPPvhAdu\/ebYg\/0gnArbN582ZBFq9aH1Q7p5SYNNL9Lnxt7PcxxNL\/TBknWGTKWNN9nPAPEiQeD+24JnkWBXn22WdzLBZiFw\/BrUJ+vMa3evVqqVu3rnnRe\/HFF+c6PXPTpk1SoUIFI0dOLRPsQu1KvzNpNs\/P2\/Lly6fNWHI7D5kyTjDIlLGm+zjhHyRIPB7aKeCmL4QsYNFufOV8\/OSmbF50a9WqJatWrZK33npL3n33XSHKZSz1OF+6nqkrQcUCmeokEgGtWxFIIQKuSH7Hjh3m5eeTTz4pw4cPP27hEObOx2PRkKFDh8rEiRODsGDxtGjRQnjkCybmsqMvXXMBR7MUAUUgoxBwRfInn3yycZkQ2iCRKFWuXFkmT55sfOm0s3btWmEKZbNmzTiMKl\/s3B9VRxUUAUVAEcgEBFyRfPHixaVr167CdMZvv\/02Yfh06NBBWrZsKZdffrnwMuOaa64x4Y3btGkTtU0+gkJQTN\/Ik4xORRFQBBSB6Ai4InmIHTcKUxyJUwMBOwVLm1g20ZvNXYOPq\/r06SO8eOVlxrJly8zLV9JzLyliCR49\/dIVFFQUAUUgkxFwRfIlS5aUkSNHyty5c2XJkiXHyauvvirEl08loIs37go237hSieC+7igCioAikIkIWJKPaew2lEHZsmUlKytLAoGA2XKMEOIAnZgqS5DSFzv2BWtmNajgge4oAoqAIpCBCLgiefDhoySW+qtRo4bxm\/Oh0rXXXmsWDdmzZw8qKZUtO3596QrBIyntjDauCCgCikCKEXBF8nx9ylJ\/27dvl+eee86sDnXKKaeY+PKQ\/WOPPSbopHJMGs4glehr2xmFgA7WFwi4InmW9uNlaP\/+\/c2sF1wzBQoUkOrVq8u9995rwg+jk6qRO1+6nlmyUKq6oe0qAoqAIuAZBFyR\/KFDhyIu8cfceb6C5avYVI3OSfJnlTwlVd3QdhUBRUAR8AwCrkiel61Y7VOmTJH9+3\/1fTMS4tUwtZKPmJiBQ1oqRGfWpAL1eLapdSkCikC8EXBF8kSF5KXrtm3bpFWrVibEQY8ePUwo4E8++UT69etnIkfGu5N5qe8sddfkBTYtowgoAmmGgCuSZ+xMkyQKJXPiiV9DBMoXXnhBpk2bZl7EopMqccasUZJP1VnQdhUBRcBLCLgmeTp\/+PBhQXjpyleouHHYJy+VEiZmTSq7o20rAoqAIpByBFyT\/EcffSTEkLnqqqvkoYcekgcffFDatm0r7dq1E9w4qRoRL10R2teYNaCgoggoAoqAiCuS37dvnwkxXL9+fSF+zcKFC2XRokXCHPlq1arJ4MGD5cCBAynHVWPWpPwUaAcUAf8ikGY9d0XyfO36\/fffC8vzFSlSJAgFMeT56nXLli3yww8\/BNOTuWOteNpUfzwoqCgCioAi4NKSL1q0qJx55plmvdVQ8H766Sdh+iRfwIbmJePYOX3yTF0NKhmQaxuKgCLgAwRcWfJ88NS9e3d55JFHhBk1+OAJLTxr1iwZMmSIYM3j0iGNsMSpGr9a8qlCXtsNj4CmKgKpQ8AVyeOqIaTBqlWrZMCAAdK0aVNp2LCh3HLLLcI8+Ztuuskck9axY8ekjkqnTyYVbm1MEVAEfIKAK5InnPD8+fOFhTyiCXo+wUC7qQgoAopA2iLgiuQtCgQhe+mll4QPoe677z6ZM2eO4Kax+anYZlj0yVRArG0qAoqADxFwTfK4atq0aSNPPPGEeQG7ceNGufPOO6V169bCHHpJwb\/DhUsHW9Xok0EodEcRUAQUAXfz5JkDz\/J\/V199tTBHnv1JkybJ0qVL5YorrpARI0akZJ784cKlgqfyLI0+GcRCdxQBRUARCFrysUDBHPivv\/5amjVrJsSSt2WYNnnJJZcIeejY9FRsz9LAZKmAXdtUBBQBjyLgiuSJQonwUVToeAg9TDybQCAQmpXw44OlqwXb0DnyQSh0RxFQBBQBd+4aPnYiTs0999wjzI1nTVdcOEuWLDFTKhs0aCClS\/\/mH08FvmrJpwJ1bTMzEdBR+wEBV5Y8A7ruuuuka9eukp2dLbVr1zZL\/+Gjb9SokRBrPhBIhSVfVew\/JXmLhG4VAUVAERB3ljyAEVq4Z8+eZj3X999\/X5A1a9bIwIEDxRnPBt1kixJ8shHX9hQBRcDrCBTIawd58coCIgjEn9d64lHOzq7REMPxQDOldWjjioAiEGcE8kzyce5Hnqsj+qRznnyeK9KCioAioAikIQK+J3nnOdE48k40dF8RUAQUgTz45L0GGpZ8bn3SPEVAEVAEMhkB31vyW3buD56\/xpVKBPd1RxFQBBQBRUAteb0GFAFFQBEIQSC9DpNqyfPx1P333y\/VqlWTSpUqCR9WRQpqRtz6ihUripU6derI6tWrc0Vfp1DmCo9mKgKKQAYikFSS\/9e\/\/iXr1683wc1YZASSHzx4sIlm6cSer2j37t0rr732WjB2PdEva9Wq5VQz+3axEA6U5EFBRRFQBBSB3xBIGsmzBuz\/\/vc\/87WsnVvfqlUr+e677+Szzz77rUdH99DdtWuXZGVlHT2K7b8SfGw4qVZKENBGFYGUIZA0ki9cuLAJRXzxxRcHB7tu3TqzX6JECbO1f3DrsNQgywni1qlfv74Q0vjQoUNWJbj94tiLV\/0QKgiJ7igCioAiEEQgaSQfCASEAGcnn3yyQNasLEWgs86dO0tUv1kZAAAP10lEQVT58uWDHWKHcMXoPv7447JhwwYZM2aMPPXUUzJ79myyc4idQrnk\/fdl8+bNaS1bt25N6\/HZ85cp42S8mTLWdB+nfXeYg5w8cpA0krfjJWIlsedffPFFwUd\/4403SiAQsNlmi+8dy7169eomD0uelafmzp1r8u0fS\/Acd7q0mVSoUCGthZuhHWM6bzNlnJzDTBlruo9z06ZN5v0hXOQ1SSrJz5kzR2699VbBDTN16lS54IILDImHgjJv3jzB0rfpR44cEaRo0aI26bjtWRmwIlSLFi2OG3c6JmTKODl3mTLWTBkn59RrkjSSx88+YcIEufvuu+Wqq67KsbJUKCgsQPL0008L0S0h9+XLl8s777wjrC0bqmuPRz44IDjd0j46pduWsabbmMKNJ1PGydgzZayZNE7G6iX5jeQT3Ksff\/xRPv\/8c+nTp08OMrbz37HyWVbwm2++Edw5uHFuuOEGM5\/+zjvvlL59+xrL39nNs0oWkh2PNZeV9zaSre8+Zx6X7GOTbn99fFQcFAe9BpJ7DTg5ygv7SSP5M844w8yPD73g7Pz3li1byvz586Vs2bLGyueFLP579BctWiTt2rUL69oBRMierYoioAgoAopATgSSRvI5m9UjRUARSAMEdAg+QEBJ3gcnSbuoCCgCikBeEVCSzytyWk4RUAQUAR8goCTvg5OUSV3UsSoCikB8EVCSjy+eWpsioAgoAp5CQEneU6dDO6MIKAKKQHwRSH+Sjy9eWpsioAgoAr5CwLck\/8svv8jDDz9sFiBhEZLevXsfF5feV2fiaGd3794tjKNKlSpC\/J7hw4cL4zyaleM\/XwHPmDFDGjZsaD4sI7bP888\/b0I\/5FD08EGsY3UOYdu2bUJ4aj6cc6Z7ed\/NOFlAhzUWiLxau3ZtSddzyjX9xBNPCGPkq98mTZoI1zPXtZfPZSx9+\/bbb6VTp05RFziKpa546fiW5F9++WX54IMPhKBlK1askAIFCghRK\/16odBv+s8PgHEtWLBACOfAOENPNnH5H3nkERk5cqRZhIUonePGjRPSQ3W9eOxmrLb\/4PLQQw\/Jxo0bbZLnt27GCTkMGDBArr32WiEENwH8nn322bQ8p3z0CKlPmzbNnM\/HHntMIH0iznrkpLruBl\/0v\/HGG8ZI4\/y5riCBBXxJ8sTBmTlzpnTp0kXKlSsnp5xyirRv317ee+89swhJAvFKWNVfffWV8IXvNddcI8WKFTMLpvCV71tvvSUsouJseOXKlXL++ecbOeGEE6RmzZrGol+2bJlTzbP7bsZqBzFhwgTzxXO9evVskue3bsbJuSe89uWXX26++D7nnHPklVdeMefW8wM92kE3Yy1YsKCceOKJcvDgwaMlRVggiONChQqZYz\/++eKLL4Qv80899VQhnLqXxuBLkmdpQB6Deay1YJ522mnmouFis2l+2rJCFoT9+9\/\/Ptht9rdv334cyRPgbdCgQYb0UGaBFeJ1c8Pj2OviZqyMhZsaLppbbrnFcz8g+hdJ3IyTpzCu4euvv97Ea2rUqJEsXrxYTjrppEjVeyrdzVgbN24szZs3F8KH8xsmMu3NN98sZ555pqfG5KYzGFoPPPCAcI167WblS5L\/+uuvhYVFnCehePHixqJ3pvlpnzHxyOfsc6QxFSlSxFj76K5du1YI5sYP5KKLLiLJ8+JmrNzMR40aZUJU++UmZk+Ac5w2LdI5ZZxvv\/22\/OUvfzEL5YwePVpwT2Ed2rJe3roZ66xZs8wCQDyN43579NFHhfdPvJPw8hj92jdfkjwWDz8WJ+iQPiGKsYad6X7ZZ0y4aZz9ZUz4ogOBnIuqoIMP969\/\/avg3sFthV8+t3j7lPGKxDpWfNq8a2DxGF7OeaX\/sfYj1nHa+po2bSoXXniheUI799xzpUGDBvLxxx\/bbE9v3YwVn\/yll14qnNdAIGCizrLvlxuap09EmM75kuTxeeHDw0Vhx4QlwTqyp59+uk3y1Zab1oEDB+TLL78M9vvzzz83SyOGkj8uHF7QMV5ePLPPe4lgQY\/vxDpW3kXwEhqrlsd6ZmPgu2bRGUJWe3yYEus4GcfZZ58tP\/\/8s1kak2Mr+K\/tvpe3bsYaOg4MM1wcfjFSQvvv9WNfkjwXFFPNJk6cKDt37jQyduxY4+fj5ZXXQQ\/XP0Ixs1LWc889J\/v27TNrubJ\/xRVXHOeHnj59ulSqVEkefPDBoNsmXJ1eTYt1rLilmGVCuGnkww8\/NNNGse55vPfq+Gy\/Yh0n+hdddJHgesMPzxMMPnqOse7J97q4GSvrRuCqsYsCMeZPP\/1UmArs9XH6sX++JHmA7t69uzDTghdUCHPL8U2T50fhyeSuu+4y8+KxWCH3P\/3pT2JXw3rqqaeka9euwswipmjxI6lataqZVcNcYwQdP4zdzVj9MJ5IfXQzTl7cDR061Ny4eWrp16+fWUXtD3\/4Q6TqPZXuZqwYaLxotYsCYawMHjxY\/DJWTwEfQ2d8S\/K4J7Kzs+WTTz4x8o9\/\/EOw\/GIYs2dVsrKyzHzh9evXC1Zrz549xT6u9+rVS7BqGSPzirFsQwUdzw4upGOxjtVZjLGDAQvMONO9vO9mnA0bNpQ333zTrHCGG84vVrzFP9ax4p5xLgrEmBm7rcfPWz5i5BsXtl4Zh4PkvdIl7YcioAgoAopAvBBQko8XklqPIqAIKAIeREBJ3oMnRbukCPgFAe2n9xFQkvf+OdIeKgKKgCKQZwSU5PMMnRZMJQJEpGzfvr0wHe+bb76JuSurV682HxyxjblQEhWZ\/89MKbfjcttFZmkxW4u22HLstg7V9wcCSvL+OE+e6CXxY2IlBEgDXcq46nyMyu+++64JrUzgslKlSsVYyh9qxD959dVXJZHjIpAW4SL88L2BP86ad3upJO\/dc6M9ywUBYr1AVGXKlDFRG3NRTVjWoUOHzI0m3g0kY1yBQED4cDD0a+p4j0XrSz0CSvKpPwee6gExcW677Tbh4zLnYixY5IQTIKzAZZddJrhIINp77rlH0OMDHj5yIR8r3uqytR9p2brRR+6\/\/37zcVc4ACDQSZMmma8gnXWjS30sGENbti+kO4WQF8zFpiwflxGvnDhAVgd3TZs2bUzER\/QIFWHzFi5cKC1atDAfmjHfmQ91GBP5tE3UxIEDB5qYK4yJPMbCmJjvPX78eKFftEEZdMCUfARdypAXTXDfDBs2zMQp55xQP7HY+So2tCztEfuGrc2jvzxRxdqeLafb9EEgA0g+fU5WokdCfO8hQ4aYZnCHQCaQH4uT8GEOj\/bnnXeeWbEIVwJkRhRB9CBGPs2HwIjBw6Im6FKGr5MhGQiLD2b40IcyfNbOV560axp1\/JkyZYqwaAblqZvY+nwRTKRC6sOlQf2snkRfHEUFUuXmAsHzYQpE98ILL5gPjdAjJg6x2ukjfeGDMxZsgTi3bNkigwYNEr7G5CbCjYYbHAtcUBYhWmTp0qVl8uTJxqXCx2mMhTERioJ8bjLouh03ZUKFD8AYK5\/\/c7MA46VLl4aq6bEiEBYBJfmwsGRmIlE8IUiCZRGLBMuRMLBYtXy2zqM9hAhR89UiLwexctFDn\/jggUDABNkivhC6lOFL1ZUrVwpx7yEpQgZTpn\/\/\/mZ1L24kTsR5Qpg6daoQW502qJuQFVjVfB1Jfbg0qN\/2xVkecub4jjvuMIvK\/N\/\/\/Z8hbYiYdPpOHvUxVsJH8CIX8qduSL5Dhw5StmxZqVu3rkCw9ImyCGE06A\/5lJs\/f75wA2JMCE83jB9dN+NGP5xwg2UNAdpjS3RKcAinq2mKQCgCSvKhiGTwMQR39dVXCy8zIRYsRtwyrEIFMYZCg+uBODqElIAor7vuuojuFyxwSBaCYkYHcuWVVwrRQwmp7KwbQiVIW506dYLJEHrlypUllkVhiN7JjYEbjK2gV69eQhgMjol4iD+afYRxs0W4aZQvX16w\/rkhXXTRRWb9UfKsYMUTVoNj+k593Lg4Rgi7a0nezbgpG06Ia8NNljy2HLNIBy4t0lTij0A61agkn05nMw5jIc738uXLzYIVuF0IY4z\/OdSlAgnjfiEMMEvVsSoO0SGdxBraHSxi3CNY2lZwhXCzcOpiURN2t0CBnJcn6U69SPs8MdCPvKyqREzzTp06ya5du+TPf\/6zMMuFJfkkl3\/0E\/KNpBLruCOVD5dOuO1wN95wupqW2Qjk\/BVlNhYZP3pWpuIFJS4bFunAX47fnZjuO3bsyIEPPmeCwxFBEBcCETGxavFB51A8doAVTqx8iAm3A4KbhmiLWO7H1MwGHzsEjb\/fJBz9Q72Eo6Weo4e5\/kdn8+bNwnhQxNeOC+a+++7jMFfhJsTLS\/R5msFip9+RCmHF0w5PD1aHfW40HNMXyscybvTDCTgwBvK42fJ0cHoM6yZQJpYnH+pVSV8ElOTT99y6HhkuEVYi4kUivmaEl4ssLWgXdMCahsytiwNLHLLGL401DwlhBdvGcffwRIA1y3THESNGCPUyA4QnhAoVKphFy60+W1wpuHJ44btixQqhfl7CQpb4\/dHJTXCxcFP417\/+Zcqyij5y8cUX51bM5OFqWbVqleBLp5+8nOUmx7gZh1Fy\/MG1ww2OdxdED2Vc4GB1g+OOYdyOanPs8kKXl69gyRoKYMLMIJQYJzdlXDe4jDgv3AwheGLSE5IaPQQddCnDsUpmIKAknxnnOaZRYrXyMhRShSgRiAWfO3k1atQwL1WZkodleu+99wqzVHg5+uSTT5pY6FjhzNAJBAJCSGCIHILG1w1pUx\/1Uge+\/t69e5vl7kI7eN111wluEtxFWNSsC8oNgptCqG7oMWTHrBxmufBEwsvhAQMGCPuhuqHH3bp1E94F4Kpp3769WZmLF6m4ot57771QdQEXnni4gTGFlPKXXHKJWEvb7biPa+BoAss7zp49W3jh+\/TTTwv44pc\/mmVmOnXs2NG81Obm2LdvX7NeKk8QrBHbo0cP1IzwdIEuM5JMgv7JCASU5DPiNMc+SEiU6YtYpQiEgLVKDbzM\/M9\/\/iNY7bhbmjdvLvjW0aMML1Vff\/11Ydohs1+YhkgeLz0pTz3URxoWL8SL5UleqPBU8be\/\/c3E1cddgSUO+Vo96sS6jVQeXWagEHOfPjIFMxAICDNqmFbJ1tbFzcjWxRMLTzL0cdmyZUJMf9bRxTpmvLTLDcSWxTpmnzTKsA4AN5PDhw+b5f\/IczNu9EOFpwsseHBgLPTD6tAfez5IIw8ddLkBk2\/HxjlDlzR0VTIDASV5355n7bgXENi7d69Amjyx8ATEuwCsflY54qaYlz7iGsKtYm8geakjWhncObxn4X1CNF3N9zcCSvL+Pn\/a+xQjgOWPFf\/ZZ5+ZYGnMTqJLuLJym3GDTiRhxhJuFdwrkXTym86NhC93mSGV37q0vLcR+H8AAAD\/\/3i89MwAAAAGSURBVAMAhs3EeBZvbN4AAAAASUVORK5CYII=","height":227,"width":377}}
%---
%[output:5b4d7b44]
%   data: {"dataType":"textualVariable","outputData":{"name":"idab_zvs_min","value":"   1.640000000000000"}}
%---
%[output:758e3eaf]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Cannot find library called 'litium_ion_battery'.  "}}
%---
