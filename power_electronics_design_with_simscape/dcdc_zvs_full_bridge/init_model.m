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
LFi_dc = 400e-6;
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
m1 = n1;
m2 = n2;
m12 = m1/m2;

Ls_dab = L_d_eff;

f0 = fPWM_DAB/5;
Cs_dab = 1/Ls_dab/(2*pi*f0)^2 %[output:3bd175ac]

Ls1_dab = Ls_dab;
Ls2_dab = Ls1_dab;

mu0 = 1.256637e-6;
mur = 5000;
lm_trafo = mu0*mur*n1^2*S_Fe/L_core_length * 1e-2;
rfe_trafo = V1^2/P_Fe;
rd1_trafo = P_Cu/I1^2/2;
ld1_trafo = Ls1_dab;
rd2_trafo = rd1_trafo/m12^2;
ld2_trafo = 0;
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
LFu_dc2 = 550e-6;
RLFu_dc2 = 0.5e-3;
CFu_dc2 = 3.6e-3;
RCFu_dc2_internal = 1e-3;
%%
%[text] ## Control system design and settings
%[text] ### DCDC Control parameters
kp_i_dab = 0.2;
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
figure;  %[output:226c4982]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:226c4982]
xlabel('state of charge [p.u.]'); %[output:226c4982]
ylabel('open circuit voltage [V]'); %[output:226c4982]
title('open circuit voltage(state of charge)'); %[output:226c4982]
grid on %[output:226c4982]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink
heatsink_liquid_2kW; %[output:85a19b40] %[output:4142069f] %[output:851aa2d0]
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
dab.igbt.Rce_on = Rce_on/par_modules;                           % [Ohm]
dab.igbt.Vce_sat = Vce_sat/par_modules;                         % [V]
dab.igbt.Vdon_diode = Vdon_diode/par_modules;                   % [V]
dab.igbt.Rdon_diode = Rdon_diode/par_modules;                   % [Ohm]
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
idab_zvs_min = 2*mosfet.dab.Coss*Vdab1_dc_nom/dead_time_DAB        % [A] %[output:3b26f23c]
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

Iin_dab_sim = Iin_dab_nom;
scenario = 1;

if scenario == 2
    number_of_cells_1_sim = floor(Uin_min/u_cell);
    number_of_cells_2_sim = floor((Uout_min-delta_load)/u_cell);
    i_in_dab_ref_1 = Iin_dab_sim;
    i_in_dab_pu_ref_1 = i_in_dab_ref_1 / Idc_FS;
elseif scenario == 3
    number_of_cells_1_sim = floor(Uin_max/u_cell);
    number_of_cells_2_sim = floor((Uout_max-delta_load)/u_cell);
    i_in_dab_ref_1 = Iin_dab_sim * Uout_nom/Uout_max;
    i_in_dab_pu_ref_1 = i_in_dab_ref_1 / Idc_FS;
else
    number_of_cells_1_sim = floor(Uin_nom/u_cell);
    number_of_cells_2_sim = floor((Uout_nom-delta_load)/u_cell);
    i_in_dab_ref_1 = Iin_dab_sim;
    i_in_dab_pu_ref_1 = i_in_dab_ref_1 / Idc_FS;
end
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
%   data: {"dataType":"text","outputData":{"text":"--- INITIAL ELECTRICAL PARAMETERS ---\nNominal Power (Sn): 347.82 kVA\nNominal Primary Voltage (Vn): 682.00 V\nNominal Primary Current (I1n): 510.00 V\nNominal Secondary Current (I2n): 194.00 V\nNominal Frequency: 5.00 kHz\n----------------------------------------------------\nCore Section Area (S_Fe): 384.0090 cm^2\nPrimary Turns (n1): 4\nSecondary Turns (n2): 8\nPrimary Copper Area (A_Cu1): 170.00 mm^2\nPrimary Copper Band Length: 17.00 cm\nSecondary Copper Band Length (L_b2): 12.93 cm\nCore Height (AM-NC-412 AMMET): 29.00 cm\nCore Width (AM-NC-412 AMMET): 6.00 cm\nCore Length (AM-NC-412 AMMET): 95.00 cm\nCore Dept (AM-NC-412 AMMET): 64.00 cm\nSpecific Core Loss (AM-NC-412 AMMET): 2.67 W\/kg\n----------------------------------------------------\n--- LOSS ESTIMATION ---\nCore Mass (M_Fe): 284.55 kg\nCore Loss (P_Fe): 758.80 W\nCopper Loss (P_Cu): 259.49 W\nTotal Losses per Phase (P_tot): 1018.29 W\n----------------------------------------------------\nEstimated Efficiency (Eta, cos(phi)=0.95): 99.69 %\n----------------------------------------------------\n--- LEAKAGE INDUCTANCE AND REACTANCE ESTIMATION ---\nCalculated Leakage Inductance (Ld_calc): 0.000016 H (15.77 uH)\nEffective Leakage Inductance (Ld_eff): 0.000021 H (20.50 uH)\nLeakage Reactance (Xd): 0.644 Ohm\nEstimated Short Circuit Voltage (Vcc): 48.16 %\n----------------------------------------------------\n","truncated":false}}
%---
%[output:3bd175ac]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs_dab","value":"   0.001235598149108"}}
%---
%[output:226c4982]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAWwAAADbCAYAAABJJ6vAAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ10V8WVv662NdRSDeJHZEP8CGqLxY\/qplQWLWs9bjfQdj8gtFsKtEtPEc\/ZhgKJPaeHVUjgSBcFPUtrZGm3BM62qLDtWWtT261SWisftUqLGlOMkYpBBG3clZXd+3T+nUzee\/+Z95+ZN+\/mvnN6KvnPu3Pvb+b+5r775t054fjx48eBL0aAEWAEGIHgETiBCTv4MWIFGQFGgBGIEGDC5onACDACjEBBEGDCLshAsZqMACPACDBhE5wDTz31FMyePRsaGxth8eLF1i08dOgQzJ07F2pra6G9vR2qqqqs9yELXLFiBWzbtg3Wr18P9fX1TvtShQ8MDMCSJUugpqbGCZYujdmxYwfMnDkz6mLevHlG+ueJuYqJmG8zZsyA6dOnu4QseNlM2MEPkbmCeRA29vnAAw\/AjTfeaK5wmTtU8nDZl6qKIL2NGzdCQ0NDWdtMddu8eTOMHTtWS3bZzqUGYqHZv38\/dHR0QHV1tcntEBJho+KoD45FFluMDA+8MRN24ANUBPVcLxAyeSAeLp8eVLxNiMIUByTrlpYW0F0MTOYCNcI2XThNsCpSWyZsw9FCB163bl10Fz4my4\/pgliuueaayAnxamtrG\/QYJ9+PKQuRUhDOjvcePXo0SgGo8lVVhcOLvwvHV4lDtMPHYtRdPB6Ldn19fYMem9WUB\/6IaQERreG\/RUpk0aJFUVS9Z8+eSMaECRMGRUGCOPA31VY5ZaOD6+233w633nrrkL6EPnE6iP4RT7wEBvK4yKkDGXOBA0bWIrUk\/qb2laRD0t\/37dtXSlcIvbCPJF3ipqmqi5hPYryEzfjvuEUh6X5McYm5PHr06BLeMmYqrjJuYl5dccUV0ZzBCyNj2ebJkydHfz98+HBpvqjzUdbZdDE0dOvCNGfCNhgq9TFRjZAE6YiJHfe7yMWOGjUqIj1BBmJCooPg5O7v70+NJNNko0lyFCoTtiAe1QEEUaDu11133aAcdRphIwn39vYa6Yr6rFmzprTY6eAqcFNtUxcEVRcZJ1xMcOFBWWKMZLsxPypH1GIMFixYUFp0434XC4+KqYluOA\/SdFFTGuUWVSRdeZEtd7+KmzqX1TGScZAXcHk+iLmMfav64oKH+XWxwKvzXZ0jvt+bGNCC16ZM2Jpwp0VbgnTjcq2COD\/\/+c8PeVGX5vxpE7Tc425ShC1HLGmP4+XIIMlBk15yyvrcdNNNEZGIiBttkRcu\/LuKtU5KRI0WMZIWfcl53DhSlF9oyo\/eqAuSihxZyk8CatSaFAXG6YYLZ9qiiy9X09IAcb\/JfxOLU1IOW8VBdYNyiyi2V6NsEeHHLeBqf+ocfvDBBwelhwSWYrEsN+c13bjwzZiwNYcwzhlVYrvjjjsG7WaQf1dTB6Jb8SipRo5phF0u2tAh7LSXSrYJG20TixMS1cKFC0E4oimuSRG2iJovv\/zyUrQft0jGEbZIcclTAUkaXwaqhK0+tuM9gtCTIuw43ZIIO0kXdXdE3IIr2zZ16tTUCLtc\/rwcYYuFCxdGFec4wlb7SyJs1R1F+o4J+y1kmLA1CdtFhC13rTo7pQgb7RRkMmnSJHjmmWdK6RBTXFXCVnGLi+ZNImx5TMpFoYKEkhbdNN10Iuy0qZlnhD1u3LhBT4viKUls87QRYau2M2EzYWtS9R+bmUQd5RxS5LCTJnm5KFqNWOScn0pqcdFUEplg5KtGZyK\/KHKSakokLq2hgiunBdQ9wTq4lsv94wsuzJ\/iU478YjVLDlvNl6c9lsflctX3Ekm6qaRbLl0jY1ruKcg0h62OYdqYCMJGffB9i0hnpKVEsuSw5R005fzB2JkLegNH2IYDp7ObAXOyt9xySyQ5bZeIvKPCJMIWKpvuEknKuaq7ROSIGP8b0wK4cyVul4jY+SFwSdvZItrE7VjQwVXsyFH72rlzZ5T\/xEvsPhg5cmRE4HiJF42omxibpF0i2F7oFxf9p+2OwHtNdBMkiS\/gBNmJl3FijNO2\/KXt8tCJSHV2iQjM1UVf3s2C8xgDDzE\/kl6Yy\/eocwpfTKrpJnmMeJdIzhE2Tmy84r7EUwdO3SpmyLHemqflhb0pwR2lImC6n1eOoE0\/PuGhSEYgbrtnGl6m40YV+1wibAF+0uey+HtnZ6eXz55tDiwTtk007ciK22opbyks1wvORXxJmsdn8eV0K9Lv6rZV1F3dHZRmDy+cOUXY+Bi2dOnSqPek+gz4+NXT02NU+yCEycuEHcIoDNZBfeyXUx462ha5loiOfT7bqCk8+cOxND3EGHItkRx2iSCp1dXVRYSclBKR85mmDuZzAnJfjAAjwAj4RMBrSgQfizZs2AA333xz9DY\/jrBFRDNx4sTo6zJ+JPU5HbgvRoARCBkBb4SNRLxs2TKYNWtWVCIz7aWjDJhK4PJv5513XsjYsm6MACNQcAS6urrg3HPPDcYKb4Qd94UYolCuTq8g7KampiElKJGwu7u7gwHTtiJsn21E\/cvjMfSPuc0eQxs\/b4StgpgUYeMLhubmZmhtbY0icUyJYNu4OrihgWlzoqAsts82ov7l8Rj6x9xmj6GNXxCErZK0HI2nfYgRGpg2JwrKevbZZ4N6HGP7zBHgMTTHLKQ7QuOY3AjbxqCEBqYNm2QZ7Oy2EfUvj8fQP+Y2ewyNY5iwbY6uZVns7JYBzUEcj2EOoFvskgmbMJgWTYtEsbPbRtS\/PB5D\/5jb7JEJ2yKaoYFp0TQmbNtg5iSPCTsn4C11GxrHcErE0sC6EMPO7gJVvzJ5DP3ibbs3JmyLiIYGpkXTOMK2DWZO8piwcwLeUrehcQxH2JYG1oUYdnYXqPqVyWPoF2\/bvTFhW0Q0NDAtmsYRtm0wc5LHhJ0T8Ja6DY1jOMK2NLAuxLCzu0DVr0weQ7942+6NCdsioqGBadE0jrBtg5mTPCbsnIC31G1oHMMRtqWBdSGGnd0Fqn5l8hj6xdt2b0zYFhENDUyLpnGEbRvMnOQxYecEvKVuQ+MYjrAtDawLMezsLlD1K5PH0C\/etnsrJGHHnYtXDhg86fzee+8t16yi30MDsyJjYm5mZ7eNqH95PIb+MbfZY2gcoxVhq+VPywGC5VGXL18enTTt8goNTNu2srPbRtS\/PB5D\/5jb7DE0jtEibJsA2JQVGpg2bUNZ7Oy2EfUvj8fQP+Y2ewyNY7QIW6REamtrob29HaqqqmxikllWaGBmNiThRnZ224j6l8dj6B9zmz2GxjFahI0AbN68GVpaWkpYtLW1Raea53mFBqZtLNjZbSPqXx6PoX\/MbfYYGsdoE7YMAp6xuG7duuhP+HIx7rxFm6AlyQoNTNs2s7PbRtS\/PB5D\/5jb7DE0jslE2AIQdfdIuRPQbQKJskID07Z97Oy2EfUvj8fQP+Y2ewyNYyoibBkYsTNk1apVUF1dbROzRFmhgWnbaHZ224j6l8dj6B9zmz2GxjEVEbYaYTc2Nnp9KRkamDYnCspiZ7eNqH95PIb+MbfZY2gcY0zYGEnPnj0b+vr6Ilxqamqi\/db19fU2cdKSFRqYWkobNGJnNwAr0KY8hoEOjKZaoXGMFmHHfem4ceNGaGho0DTbTbPQwLRtJTu7bUT9y+Mx9I+5zR5D4xgjwkaCXrx4sU08KpIVGpgVGRNzMzu7bUT9y+Mx9I+5zR5D4xhtwm5ubobW1lat1Ad\/mm5nyrCz28ExTyk8hnmiX3nfhSXsuXPnwp49e7QR4OJP2lAlNmRnrxzDvCXwGOY9Atn733\/odWj4dAv0ff+fswuxfKdWhG25T2viQlv9rBn2tiB2dtuI+pfHY+gfc1s9dj56AOZ37oVDX7vWlsiK5TBhVwyhOwHs7O6w9SWZx9AX0vb7YcK2jClH2JYB9SyOOpkhnNRtpGzfigd6YMUDz3KEbYsXmLBtIZmPHMrOLhClbiNl+5iwLfMCE7ZlQD2Lo+zsTNieJ5OD7piwM4CKlQHxitv\/zYSdAdCAbmHCDmgwMqpCeQyZsA0nxY4dO2DmzJmQVAWQCdsQ0MCaU3Z2jrADm2wZ1Fly71Pw9Z\/2Fj+HPTAwAEuWLIFt27YBFnyaM2cOrF69GmxW6sPP4ZcuXRrBjPVKOMLOMOMCv4UJO\/AB0lCP8hhOvXMXPPzM4WITtiDriRMnwtixY6GzszOq0Ld161bYvn27tWp9mAqpq6uDnp4eToloOE4Rm1B2do6wizgjB+tMgrDlE9T7+\/tLhN3b2xudlG4jysZP2zds2AA333wz3HHHHamEjT92dXUVf3bEWICYjhkzhqRtaBR1+4aDjVTHcMqUKXD44x2R7xX+wxmMfrG86rRp0+D++++PUiLz58+P0iOVFofCCH7ZsmUwa9asqG4Jv3Q8lyxhc4Rd\/KGlOob4Wfqlt\/6MBmGjFeKFoJhytg7lVettC\/lxLx75pWOxHZ6qs8ujQt1GqvaRI2xfVMERNkfYvuaai36oEhr1HL3IX5NIibiY2EkymbCZsH3ON9t9MWHbRtS9PDm6Puml38KL3\/yC+041ezAu\/hR3+ozal69jwzglojnKgTajTmYIO3UbKdr3T9\/rhtVdv4u8ZsTOe6D3x98KxoOMCRs1F1vupk+fXjJk8+bN0RY8fOmI\/41b\/G6\/\/XanhjJhO4XXuXCKzq6CRt1GavbJ0XVt9clw5J5PQXd3t3Nf0O3AmLDlbX3ywbvilBnc1ofb\/XCLHx7O6\/JiwnaJrnvZ1Jw9DjHqNlKyTyZrHMvdX\/kQXPPB9xWbsEWEvW7dOhAH8aqfkHOEbYfsKDnDcCQzTonY8QMfUh5++jBMvWtXqautX7wMrr7gVAgtKDSOsIVF8vY7OWctR9rV1dVOsQ4NTNvGMmHbRtS\/PB5D\/5ib9IhR9fd\/\/RK03vdU6bZvz7kEbhh\/evTv0DgmM2GbgOKqbWhg2raTnd02ov7l8Rj6x1y3RzUFgjlrTIPIV2gcw4StO7o5tGNnzwF0y13yGFoG1IK4nz79Mky7a\/cgSUjWmAbB\/ydH2OpXjsJAPCm9o6MDXKdCRH+hrX4W5tIgEezsthH1L4\/H0D\/mcT1iNH1j596o+p58JRF1qBxjHGGLfdgLFiyAH\/3oR1HNDyxQhOVWsYKfvNXP9VAxYbtG2K186mSG6FG3MWT7kKTxTEY8TFe9yhE1KcJubm6G1tZW2LJlS1QCFUna58vGUMG0TW8hO4MNW6nbx4RtY5aYyfjNgddg0Xf3DYmkhRRdog6VY4wjbFEPG3eGTJ48OToR5u67746q9u3fv59TImbzK7U1dUKjbh8TtkVniBGFEfThgTfgK\/c9nUjQeJspSctdhfYUb0zYwpi1a9fC9ddfH30kg6SNpVXxIIOqqiq3oyRJDw1M24ZTJzTq9jFh2\/MIJGe8MMXxyDOHQfw7rgck6EkXnAZf\/mhd9LP6ItFEq9A4JjNhmxjtqm1oYNq2kzqhUbePCTubRyAZv3n8ONy06Tew\/+XXU8lZEHLtaSfD2qaLKyZoVePQOIYJO9uc8nIXdUKjbh8TdrqbyFHzc4deT01ryJIwYv7ry86EWR+qsU7Q5Ahbp5YIb+uzw+fUCY26fUzYUIqOXxk4Bje\/\/TWhurUuyVtEKmPahDPguotHRZ+K+74KG2EnnQQjA+g7jx0amLYnE3VCo27fcCHsE997djT1\/+fYm3DHQ\/uh56UB7WhZ+AySM6Y1bpl2AZw24h0V5Z1t+mFoHGOcEkmKsG2CpCsrNDB19dZtR53QqNtHibBF+uKHv+mH+3a9qJVbVue5iJg\/fP6psODaWhjxzhODIeYknwyNY4wJW5dsfLQLDUzbNlMnNOr2FYmwBSHft+dF+OGT\/dFU1k1dxBEzRsuNE0bD9e87PXhSTvPb0DhGi7B1TplBo\/nTdLuUTZ3QqNsXEmEjIR8HgDUP7Yd9B17LFCHL6Qv8byTlpvHvgg9fcl70UyXb5+x6jj1phSRse+bblRQamHat48+abeOZhzwfi5KIju955HnYuf9IRdGxTLxIyB++4DRouvKsREL2YV8e4yb6DI1jtCLsPAEr0uOKbZyoOwN1+yqNsLGoPkate194Ff7zyX545sU\/VBQZqxEy5pI\/dsloGF9zSuYImfoYkiFsPFWmpaWlxFFtbW1eCz9hx6GByYRthgB1Z48jbPkLPUxRYFS8y0JUHJeumHxhNfzt5WeWBsVFyoL6GIbGMZkibDyEF0usilKqIsfd0NAQHcLr6woNTNt2U3cGavYJMsav9L614wV4tOcVGHj9dTg48Mf9yJXMEUG4mKpoOO9U+PSfvbWdTk5jVCI\/y73UxlDFIDSOMSZs\/nAmy7TOdg91ZyiCfXJEjF\/jbX7sQLTPWOeTad1Rl4n43NOroOmqs6Hmve\/KlYh1dS\/CGOraEteu8ISNRnGEXckU0L+XujPkZZ9Mwm\/875tRveRfPPtKNDC2ifjYsWNw3uhT4E+rT4amK88u7aRwkZ7Qn1n2WuY1hvYsSJdEgrDRRM5hu58y1J3Bpn0yCR978zj8+2MH4JGn3zpdxCYJy+kHTE1cdNa7o4h41LvfERsR27TR\/Ywz74G6fWQI23xo7d8RGpi2LaTuDOXsEySM\/3\/KySfCv+14Afb9\/jXnJFw7qgouOecUuOH9b52cXUk0XM5G23PGtzzq9oXGMcY5bN8TIq2\/0MC0jQ1FZxBb1ZCEd+57Dn77ykmAuWHbUbAaCeO\/G84\/FT59ld8XdRTHUJ7n1O0LjWOYsG2zrEV5RXEGEQnveu4I\/NdTL8NTv\/+DkyhYJeHa6iq44Iwq+ORlbreuVTKkRRnDrDZSt6\/whC228NXW1no\/YUadVKGBmXXSJ92XlzMIAj48cAw6H30Bnnj+VWcErJLw+aNHwF9cXA2XnPOeitMRtscji7y8xjCLrlnuoW5faByTKcKOK7XKH85kme7p99hyBkHAh157Azb\/8gA80eePgHF3BL6Y+6tLRsOJf3LCIBK2ZZ995O1JpG4jdftIEHbcdMZdI5s2beJDeO35OiQ5gyDgo69jBHwAftV71FsEjB2NO3MEfO7qMVF5TDlCNjWdurMjHtRtpG4fCcKOi7DxFPX169dDfX19ot\/K96VV9lPlJ7UNDUxTwpLbCxL+1fNH4QdP9kcfZ3QffBX6jhyrRGzsvfKHGtjg\/TWnwN9ccSaMPuWdFRGwqaLUnZ0J23RGhNc+NI4xTomIHDZCKz5N14F5YGAAli1bBrNmzYpIHSPy7du3x+bB8bP3zs7Osjny0MCMw0HemvYfjx+EJ\/tetb4jQiXg888YAR8bfzqMO\/PdXglYZx7IbZiwTRELrz31MQyNY4wJ29aUwSh6+fLlsGrVKlDPgEQy7+npKVuXJDQwkZxXd\/0Onn7xD5mLvwt8kYTFV3LnjR4BjR8YDfVnjCjBX8neYFtjWKkc6s7OEXalMyT\/+0PjmNwIOy3Cxk\/f161bVxqtjRs3AhaWUq88wURyfvalAVj1YI8ROcvR8CcuOwOmXDRqEAmjXNGGOqFRt48JO3\/CrVSDPDkmTnfvhC2fXhNHxJg6WbJkCUycODEq14rpkYULF8bmxxFMvLq6uiodF637MZ\/89V8chm1739plkXTVjDwJzn7PSfCR80fAn587Al44egyuOOdkrT7kRr29vTBmzBjj+4pyA3X7cByo20jVvilTppTcqLu7OxiX8k7YwnJB3FiONS56Fu1UApeR87H6YcT7+htvQsOKn8cOGkbDky44DaZ\/8Cy4+oJTrQ4s9QiUun0cYVt1h1yE+eAYE8OMCdtWedU0IpYNEO2ampqGELtLMJGodz93FD674ddD8ESS3vrFyyqqMaEzSNQJjbp9TNg6szzsNi45Jovl2oQdt5VP7bCxsTFxZ4dK9Chv0aJFsHLlykFbAdV2mBLBnHbcjhRXYGK9ixs37QW5ApwvkpYxpU5o1O1jws5CSWHd44pjslqpTdhyKqO5uRlaW1tT91zHKaSSvshhx5H57Nmzoa+vD9L2d9sGEwn6Ozt\/D7d+\/485KyTqtTMutp7u0Bkw6oRG3T4mbJ1ZHnYb2xxTqbXGhF1phzbvtwkmkvXUu3aVomok6jUzLory03ld1AmNun1M2Hl5jr1+bXKMDa20CFu8IDx48CDcdtttUYpiz549Q\/pP+3rRhrKqDFtgxpH17q98yIXKRjKpExp1+5iwjaZ7kI1tcYwt47QI21ZntuXYADNUsmZntz1b8pFHfVGibp8NjrE584Y1Yatk\/ZmGGlj9dxfaxLciWdSdgbp9vOhWNP2DuLnwhC1\/+KIiWqSUCJL1jZ17S18p4h5q3KoX0kWd0Kjbx4Qdkjdl06XwhJ1kNua1J0+enPoRTDbIku+qBEwsSzq\/c28kHF8whpCzVi2lTmjU7WPCtu3x\/uVVwjEutLWWEkkr5uRCcZRZCZjVX3ooaLJmZ3c1a\/zKpb4oUbevEo5xMdOsEXaRDjCYeueuUioEI+tQK99Rdwbq9vGi64Ky\/MosPGGn5bCTquq5gjgLmHIq5OrzT4Wt88PKW8tYUSc06vYxYbvyfH9ys3CMS+2sRdgulUySnQXMS2\/9WfRxTB6fmptiRJ3QqNvHhG0648Nrn4VjXFqRibDVT8nTalu7VN4UTDm6vvvv3w+fvOwMl+pVLJs6oVG3jwm7YhfIXYApx7hW2Jiwk6rs6Z4SY9MgEzDlPdeh7gpRsaFOaNTtY8K26e35yDLhGB8aGhO2rfKqNowzAVOOrrFGyKeuOtuGCk5lUCc06vYxYTt1Dy\/CTTjGh0LGhJ1UnzqtDKorQ0zAFDtDihJds7O7mjV+5VJflKjbZ8IxPmaWMWGjUpj+aGlpAbErBMl65syZ0NbWFh3r5evSBRPTIfiyEa9\/mDQG2j9R70vFivqh7gzU7eNFt6LpH8TNuhzjS9lMhI3KJdW29qU49qMLprzvGj8\/t32UlyubqRMadfuYsF15hj+5uhzjS6PMhO1LwbR+dMCUo+sipUPY2UOYYZXrQH1Rom6fDsdUPkv0JRgTdtJLR\/0u7bXUAVN+2Xhn08XQdOVZ9hRwLIm6M1C3jxddxw7iQbwOx3hQo9SFMWHjnVjoqa6uzmu+Og4UHTCL+LJR2Eqd0Kjbx4Ttk8rc9KXDMW56jpdqTNhFKq8qp0NC\/ww9bnioExp1+5iwfVKZm74KT9huYMkmtRyYePo5ntOIV8hFnpKsp05o1O1jws7m1yHdVY5jfOtqHGH7VjCtv3JgFjkdws4e0kzLrgv1RYm6feU4JvvMyHanFmEX8RDeoqdDmLCzTejQ7qJOaNTtKyRhh+YEQp80MOV0yOLrz4XF19eFakaiXtSdgbp9vOgWzuWGKEyCsItQrW\/hd\/bBPdufjwagSB\/LyDOGOqFRt48JmwnbNgJaKRG506JU6yt6\/pqd3fZUz0ce9UWJun2Fj7CLUK2PQv6aCTsfgrXdK3VCo25f4Qm7CNX65Px1UdMhTNi2qTMfedQJjbp9hSdsnPahV+ubetduePjplyMPLeL+a0Et1J2Bun286OazSNrslQRhIyAhV+ujkL9mZ7fpdvnJor4oUbePDGHn5wJ\/7DkOTDl\/fc2402DLFy4NQdVMOlB3Bur28aKbadoHdRMTtsZwyNH7hAkToKOjA6qrq4fcGQcmlfw1O7vGRClAE+qLEnX7mLDLOBm+1Fy2bBnMmjUL6uvro3z59u3bob29HaqqqgbdHQfmll0vwue+9UTUrsgvHJmwC8DGGipSJzTq9jFha0xyuQlG28uXL4dVq1YNibLjwKSSv2bCNpwogTanTmjU7WPCNnQskwibyv5rARF1Z6BuHy+6hs4eYHMmbM1Bketui8N+1VtVMGXC\/s68CfCRC4fmvTW7D6IZdUKjbh8TdhBuVJESJAhbnJKuIpH2gjAraoK4Fy9eDA0NDUNy2PiHrq6u6O99R45B44be6L+\/\/smz4IpzTs7abRD39fb2wpgxY4LQxYUS1O1DzKjbSNW+KVOmlKZ8d3e3i+mfSaZxLRFBoDNmzPByRFhS7RK0Vl39VjzQAyseeDYCougvHDk6yzSfg7uJ+lMEdfsKH2G7PoRXlY8vHRctWgQrV66Mdo3IlwompReOTNjBcW8mhagTGnX7Ck\/YOGvxRWBPTw9gmsLFpfsVpQpm9ZceitQp4vmNcThSdwbq9vGi64Id\/MosPGGHegiv\/MKxqAcWqFOROqFRt48J2y+5uuit8ITtApSsMmUwi37gLkfYWWdB2PdRX5So28eEbdG\/kgibwgtHjs4sTpQcRVEnNOr2kSBssXNj27Zt0NjYCHPmzIHVq1fHfo3o0ldkMMULR+zv0NeuddmtN9nUnYG6fbzoenMVZx0VnrDlbXZjx46Fzs7OqM7H1q1bE2t+uEIzjrBrq0+OamBTuKgTGnX7mLCL74WFJ2x5211\/f3+JsHEDfVLND1fDJoNJbYcIO7urWeNXLvVFibp9hSdsnO4rVqyAvr4+mDZtGtx\/\/\/1RSmT+\/PlResTVVr84NxNgyjtENnx2PDR+YLRfr3TUG3VnoG4fL7qOHMOjWBKEjXipn6e3tbV5+fJRHisBprxD5M6mi6HpyrM8Dqm7rqgTGnX7mLDd+YYvyWQI2xdgaf0IMOVP0ot8hqNqK3VCo24fE3YILFGZDkzYleE36G4B5pe\/uw86Hnk++o0J2yLAjkUxYTsG2IN46mPIhG1xEgkwqdUQERBRdwbq9nGEbdHZcxJFgrDlfdgCx3nz5nl94Yj9CjAvvfVngC8eqdQQYcLOyTsddEt9UaJuX+EJW5B1TU1NiaDF33C+x5296MAPIpEI5o9\/+SQgYeM19QOj4V8\/O95Vd97lUncG6vZxhO3dZax3WHjCTiqvmnb2onUU3xYowBR7sKkUfeII29WM8S+X+qJE3b7CEzZOedzSJ75wFCeZ497suro6r1v7EMxv\/mAnTL1rV+SJlLb0cXTmn1xd9Eid0KjbV3jCTiuvKk94PC7s3nvvdeEDJZlXi2y5AAAKSUlEQVQI5rLN22F+597ob1SKPnGE7XTaeBVOndCo21d4wvY628t0hmDesOx70PnogaglpS19HGGHNNOy60Kd0Kjbx4Sdfe4PuRPBHN\/8XXj4mcNAqegTR9gWJ0nOoqgTGnX7yBA2HhPW0tJScoe8Pk0fOefb0ZY+JuycmSlD99SdnZ+SMkyKwG4hQdj4ghFfPHZ0dEB1dTWIvHZDQ4PXvdgI5uGPd0RDTG0PNjt7YJ6bUR3qixJ1+wpP2CFt66sbfxUc+eiKyJXWzLgIPnXV2RndKszbqDsDdft40Q3Tr0y0Kjxho7GhRNi1V90Ar169KMKf2h5sdnYTtwq3LfVFibp9JAgb3SOEHLZM2NS29DFhh0vCJppRJzTq9pEhbJNJ66ptzV\/+I7x+0dRIPBO2K5TdyaXu7Lzoups7viQzYVtE+uxprfDf518XSaS2B5ud3eJEyVEU9UWJun1M2Bad54zP\/AscO\/1Cklv6mLAtTpQcRVEnNOr2MWFbdB4mbItg5iCKurPzopvDpLLcJRO2RUApnpQuw0Od0Kjbx4Rt0dlzEsWEbRF4Qdh46C5W6qN2USc06vYxYRffI5mwLY4h1TrYAiLqhEbdPiZsi86ekygmbIvAC8KmVgebCdviJMlZFPVFibp9TNgWHUgQNsU92BydWZwoOYqiTmjU7RvWhI0Fo2bOnFlyn40bNwIWjFIvPG5s9uzZ0NfXF\/2EhyGIQlNyWybsHJnIQtfUnZ0XXQuTJGcRw5awsWjU0qVL4atf\/WpU4Q\/JG2uSxBFx3BFkceMmCJviRzPs7Dl7qqXuqS9K1O0btoStzn9RknXx4sVDomysU9LT01O2VKsg7ENfu9aSe4UlhrozULePF92w\/CmLNkzYb6OGaY9FixbBypUrob6+fhCWGHmvW7eubOoECZviwQX80jGLa4V5D\/VFibp9TNj\/Xwp1YGAAlixZAhMnThxyyrr6G6ZHFi5cCOvXrx9C7EjYFA8uYMIOk3yzaEWd0KjbN+wJWxByTU1N2ZQHOkgauSNhn\/TSb+HnS6\/P4kvB39Pb2wtjxowJXs+sClK3D3GhbiNV+6ZMmVKa1t3d3VmnuPX7Tjh+\/Phx61ITBKaRb5IO4p6mpqYhuW4kbKpfOXL+09esdNsP9QiUun3DNsLWJWv1CLK03SRI2BRPmhEUEtpksU1t1O1DvKjbyPbZ9op0ed4ibHVvtVAL92KPGzcOmpubobW1NcpTy20xdRKXv8b7kbBH7LwH3rn\/Eb+ocW+MACMwbBAYtimRYTPCbCgjwAgwAg4Q8BZhO9CdRTICjAAjMKwQYMIeVsPNxjICjECREWDCLvLose6MACMwrBBgwh5Ww83GMgKMQJERYMIu8uix7owAIzCsECgkYcvb\/ubNm6f1xWSoo6pji9jDvm3btsiMxsZGaG9vh6qqqlDNGqSXjo3yDWl1ZkI0WNc+uV1SyeAQ7UOddG2USygX3TfFWGBto7q6uiFlNPIYq8IRtlzlDyd9Uk2SPMA07VPXFqxeiNf06dNTP9U37d9He10bhS5icXrssccS99\/70Fu3D1371OqUuhUpdfVw2U7XRnmhxZIKRfZNmayxEF1bWxsTdpZJhpNi+fLlsGrVqqiuNk787du3FyriFHZntaVINpvaiLY9\/vjj8MQTT8RWcswyZ1zeo2sfRp4\/+clPCvk0aGKjXOMe\/xsvLKFctEuueYS6c4SdcQTVww3SPl3P2IW327LaUiRHMLFREAM+SqONcaV3vQ2OZke69uFCdPDgQejq6oI9e\/YknqKk2a3XZro26kbiXpW30BmnRCoAUY0ui0zYWWwpmr0mNqJjTJ48GUaNGpVYK72CqePkVl37sN2aNWtKaZ4iLbq6NiLAgrRxUUo6AtDJQDgUyoRdAbi6q30FXXi71dSWtNrg3pQ27EjXRjllUKSXjrr2qTnrIi28ujaqNhVpUUqb1kzYhk4vN9fNp1XQhbdbTWwpkoNnGS\/1lCGUkVb4y9sglelIdwx1SS8Uu7KOId4nctZFnbPqGDBhVzArKeXJdG0p8sTXtVEliKTj4yqYOk5u1bVPLhssdlDoHuLhRHEDobo2xkXYfX19hdwQIMPDhG0wWeKa6u4JrbAbL7cn2SJPkrjos0h7sXVsLCpho9669sntijR+JjZi6qelpSUazqLtNU9yeCZsL1TInTACjAAjQAuBwn04Qwt+toYRYAQYAX0EmLD1seKWjAAjwAjkigATdq7wc+eMACPACOgjwIStjxW3ZAQYAUYgVwSYsHOFnzsvh4BcqdCk+luIH22ISnYu95fLO1GofGlYbo4Mp9+ZsIfTaBvaalJkyqStiRpZ96CHStidnZ3O9yWLRa6pqQkaGhpM4Oa2gSPAhB34AOWpngkJm7Q1sUn9QlD3XibsJcCErTtbitOOCbs4Y+VEU\/VwBJF2kAvRi488ent7Yfbs2YBfr+GV1hblzp07N6pMh1fa47lcMEhuK+uQlEaQUwByGyTso0ePRv\/Dgx\/U+2XZ2Keodyw+NR85cmR0n9Bb\/ngJ7cb7Ozo6ohK\/STqoA6YuPqqOaR+aqAtQ2gLJEbYTVwlCKBN2EMOQnxJyUSLV0WVSQA2xIL2I2tQCTXFtJ06cGBV9TyvmJPoUbdUCV2kpEfVQALntN77xjYhw169fD\/X19VG5VvGZNPbZ3NwMra2t0W\/yff39\/dGitGDBglLBevl3POUHcdi\/f39E2HjhwoT1MzD9kKZvHGFjcXx5UUj6lJsJOz8fCalnJuyQRiMHXdSyn7IKaVGcSqxyW4zE5UMmUGbS570qmccRuFwUX9YvjRxNCA5137RpU0TASNhqHRNVltzvvn37QM5Lp0W3cYQtE3TawmZiD0fYOTiSpy6ZsD0BHXI3cv0HOXWgEracFsDHd7zEIQNyW0yDzJw5c4jJcbs8VNI1Iey0BSWN4MTTgjgjc9KkSXDkyJFYwlb1wXtlnR988MFS7QzZ4LgjpeIIG+8R1e3kAlEY+csXE3bIHuRPNyZsf1gXoic5dbB169bS8WsYNcuRZ1pKJC7CTjI+jwgbFxQ5aldTIpVE2GmDzBF2IVwgaCWZsIMeHvfKxUVuPT09UdSnpjkwt3vbbbdFuVq8T84Rp+WwRa55xowZQw4ytZnDlsl\/y5YtEXgielWfABYuXBjlt0WpU5GTjkuJmOSwxQtIgZOawpHTJyqG8mKJuXI1PSXSNiKPjr+3t7eD2pZTIu79Jq8emLDzQj6QftVdIvJOBUE+o0ePjtIF+CIPX5LhtXbt2ujf4mWb2hbbyLtE0j56SdoloqYfcEeGesk7NPA3+QVeEmHj3\/HFodg9gi8f0RZM7+AVV4tbpIMwZYR24dNH3C4RvD\/phO2kCBsXC\/WcRzU9ImMkdNi9e3dE2OpLVCbsQJzLgRpM2A5AZZG0Eci6N7xcDtsWakzYtpAMTw4TdnhjwhoFhkDc1scsp8UwYQc2sAVUhwm7gIPGKvtFQE3ZZD0tRq0loubZbVjFtURsoBiujP8D4LFcyA+zF4UAAAAASUVORK5CYII=","height":219,"width":364}}
%---
%[output:85a19b40]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:4142069f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:851aa2d0]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
%[output:3b26f23c]
%   data: {"dataType":"textualVariable","outputData":{"name":"idab_zvs_min","value":"   1.640000000000000"}}
%---
