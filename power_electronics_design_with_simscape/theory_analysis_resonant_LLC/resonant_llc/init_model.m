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
application690 = 1;
application480 = 0;

n_modules = 2;
%[text] ## Settings and initialization
fPWM = 5e3;
fPWM_AFE = fPWM; % PWM frequency 
tPWM_AFE = 1/fPWM_AFE;
fPWM_INV = fPWM; % PWM frequency 
tPWM_INV = 1/fPWM_INV;
fPWM_DAB = fPWM*5; % PWM frequency 
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
ubattery = 1250;
margin_factor = 1.25;
Vdab1_dc_nom = ubattery;
Idab1_dc_nom = Pnom/Vdab1_dc_nom;
Vdab2_dc_nom = 1500;
Idab2_dc_nom = Pnom/Vdab2_dc_nom;

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:7c09ef44]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:076b441a]
%[text] ### AFE simulation sampling time
dead_time_DAB = 0;
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
% single phase DAB
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/8) %[output:02159090]
fres = 2.5*fPWM_DAB;
Cs = 1/Ls/(2*pi*fres)^2 %[output:5d197f8f]

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
Vac_FS = V_phase_normalization_factor %[output:53fa3074]
Iac_FS = I_phase_normalization_factor %[output:215718df]

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
danfoss_SKM1700MB20R4S2I4;

dab.Vth = Vth;                                  % [V]
dab.Rds_on = Rds_on;                            % [Ohm]
dab.Vdon_diode = Vdon_diode;                    % [V]
dab.Vgamma = Vgamma;                            % [V]
dab.Rdon_diode = Rdon_diode;                    % [Ohm]
dab.Eon = Eon;                                  % [J] @ Tj = 125°C
dab.Eoff = Eoff;                                % [J] @ Tj = 125°C
dab.Eerr = Eerr;                                % [J] @ Tj = 125°C
dab.Voff_sw_losses = Voff_sw_losses;            % [V]
dab.Ion_sw_losses = Ion_sw_losses;              % [A]
dab.JunctionTermalMass = JunctionTermalMass;    % [J/K]
dab.Rtim = Rtim;                                % [K/W]
dab.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
dab.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
dab.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
dab.Lstray_module = Lstray_module;              % [H]
dab.Irr = Irr;                                  % [A]
dab.Csnubber = Csnubber;                        % [F]
dab.Rsnubber = Rsnubber;                        % [Ohm]
% dab.Csnubber = Irr^2*Lstray_module/Vdab2_dc_nom^2
% dab.Rsnubber = 1/(Csnubber*fPWM_DAB)/5

% infineon_FF1000UXTR23T2M1;
% inv.Vth = Vth;                                  % [V]
% inv.Rds_on = Rds_on;                            % [Ohm]
% inv.Vdon_diode = Vdon_diode;                    % [V]
% inv.Vgamma = Vgamma;                            % [V]
% inv.Rdon_diode = Rdon_diode;                    % [Ohm]
% inv.Eon = Eon;                                  % [J] @ Tj = 125°C
% inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
% inv.Eerr = Eerr;                                % [J] @ Tj = 125°C
% inv.Voff_sw_losses = Voff_sw_losses;            % [V]
% inv.Ion_sw_losses = Ion_sw_losses;              % [A]
% inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
% inv.Rtim = Rtim;                                % [K/W]
% inv.Rth_mosfet_JC = Rth_mosfet_JC;              % [K/W]
% inv.Rth_mosfet_CH = Rth_mosfet_CH;              % [K/W]
% inv.Rth_mosfet_JH = Rth_mosfet_JH;              % [K/W]
% inv.Lstray_module = Lstray_module;              % [H]
% inv.Irr = Irr;                                  % [A]
% inv.Csnubber = Csnubber;                        % [F]
% inv.Rsnubber = Rsnubber;                        % [Ohm]

infineon_FF2400RB12IP7;
inv_t.Vth = Vth;                                  % [V]
inv_t.Vce_sat = Vce_sat;                          % [V]
inv_t.Rce_on = Rce_on;                            % [Ohm]
inv_t.Vdon_diode = Vdon_diode;                    % [V]
inv_t.Rdon_diode = Rdon_diode;                    % [Ohm]
inv_t.Eon = Eon;                                  % [J] @ Tj = 125°C
inv_t.Eoff = Eoff;                                % [J] @ Tj = 125°C
inv_t.Erec = Erec;                                % [J] @ Tj = 125°C
inv_t.Voff_sw_losses = Voff_sw_losses;            % [V]
inv_t.Ion_sw_losses = Ion_sw_losses;              % [A]
inv_t.JunctionTermalMass = JunctionTermalMass;    % [J/K]
inv_t.Rtim = Rtim;                                % [K/W]
inv_t.Rth_switch_JC = Rth_switch_JC;              % [K/W]
inv_t.Rth_switch_CH = Rth_switch_CH;              % [K/W]
inv_t.Rth_switch_JH = Rth_switch_JH;              % [K/W]
inv_t.Lstray_module = Lstray_module;              % [H]
inv_t.Irr = Irr;                                  % [A]
inv_t.Csnubber = Csnubber;                        % [F]
inv_t.Rsnubber = Rsnubber;                        % [Ohm]
% inv_t.Csnubber = Irr^2*Lstray_module/Vdc_bez^2
% inv_t.Rsnubber = 1/(Csnubber*fPWM_INV)/5

infineon_FF1800R12IE5;
inv.Vth = Vth;                                  % [V]
inv.Vce_sat = Vce_sat;                          % [V]
inv.Rce_on = Rce_on;                            % [Ohm]
inv.Vdon_diode = Vdon_diode;                    % [V]
inv.Rdon_diode = Rdon_diode;                    % [Ohm]
inv.Eon = Eon;                                  % [J] @ Tj = 125°C
inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
inv.Erec = Erec;                                % [J] @ Tj = 125°C
inv.Voff_sw_losses = Voff_sw_losses;            % [V]
inv.Ion_sw_losses = Ion_sw_losses;              % [A]
inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
inv.Rtim = Rtim;                                % [K/W]
inv.Rth_switch_JC = Rth_switch_JC;              % [K/W]
inv.Rth_switch_CH = Rth_switch_CH;              % [K/W]
inv.Rth_switch_JH = Rth_switch_JH;              % [K/W]
inv.Lstray_module = Lstray_module;              % [H]
inv.Irr = Irr;                                  % [A]
inv.Csnubber = Csnubber;                        % [F]
inv.Rsnubber = Rsnubber;                        % [Ohm]
% inv.Csnubber = Irr^2*Lstray_module/Vdc_bez^2
% inv.Rsnubber = 1/(Csnubber*fPWM_INV)/5
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
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:076b441a]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:02159090]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     1.420454545454546e-05"}}
%---
%[output:5d197f8f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     4.565127250189170e-07"}}
%---
%[output:53fa3074]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:215718df]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:0bd29de1]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["2.831309534039184e-01"],["6.766822226281998e+01"]]}}
%---
%[output:915851da]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Afht","rows":2,"type":"double","value":[["0","1.000000000000000e+00"],["-9.869604401089359e+04","-1.570796326794897e+01"]]}}
%---
%[output:46c059b7]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Lfht","rows":2,"type":"double","value":[["1.555088363526948e+03"],["2.716608611399846e+05"]]}}
%---
%[output:7459e256]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000e+00","2.000000000000000e-04"],["-1.973920880217872e+01","9.968584073464102e-01"]]}}
%---
%[output:4579d87c]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["3.110176727053895e-01"],["5.433217222799691e+01"]]}}
%---
%[output:74e07718]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["7.338637605205141e-02"],["3.802432508328568e+00"]]}}
%---
%[output:0825f5ce]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAATcAAAC7CAYAAAAAJ9klAAAAAXNSR0IArs4c6QAAIABJREFUeF7tXQ+QFsWVf+gmMckCSowgQqKBxJjLsRIInEiIhqsNl0sMR84NblKLJ3issBpR0AtHcSe3x0UXlqJq5SABTqgEkFTIQnIVNEfuQoJGgsUfYyCCHl4A2SwSdSkXEnWv3uD70tvbPdMz0zPd8+2bqlRkv+nXr3+v+zfvdb\/u7tPV1dUF\/DACjAAjUGYI9GFyKzOLcnMYAUYgQIDJjTsCI8AIlCUCTG4pzHry5EmoqamBY8eOwcqVK6G6ujqFNLOiK1asgCVLlsCYMWNg9erVUFlZGRTEv+Mza9YsM0EJ33Jdv07txx9\/HOrr62Hu3LmxMLjnnntg0qRJsW2XtFwS2KltVDZJX9PZLYk+WZbZv38\/1NXVwcSJE6G5uTlVVUxuKeDzhdxwoLW2tsYe2Emarhokedav0vnMmTMwY8YMOHHiBGzevBkGDRoU2TQqs3v37lgfpqTlIhXSvCD2MXylb9++sH79eqiqqoolsijkRvgePHgwUTtFUJjcYnURP1\/Ok1x8JDfybCZPnmz8tU9KUknLJe05RG6vvvpqqsFeFHKjKASjkzj2VOHrlNxkd1sOtWjQYiPxQe8EH\/k9\/Bu9q\/q6iYb98pe\/DPfee28gx+QrKH85hwwZUvIOVJ4b6YE6oieBISsZSRwYcv06L1DulPjVprB02bJlMGfOHEDvgx5Vm8jV79+\/fzfPRtXh6d2Ojo5ApNheseNh+6Lql+0r6ybjgSHlkSNHAjuLHVvWSe70OnIX+wTqTuVkm8rtlMtRf0N9aRqC8BbxITzpN5PwMWwMyL\/p+j7VJ78v4iTa+q677oI77rgD0Maq\/qJrP06BkC1Ilx07dpRkDBs2LPCgqT8mtaeuv8b9QDgjN7kjqDqLDLLYOOpw+DcRUPEd6ly6ulSDVywvDypZR\/y3POem0hmNPGXKlB4DQyS4gQMHKufvbJCbytsQ\/0bzVKrBJJOwqE8YubW1tQVzYPIjkkGYfWlg6nRSEY5IJjrZKPe+++7TktRDDz1U+oiKuictFzYHGDUGDhw40AND1Ydd\/OjIeBOOpmMgDDecA1ONCbKpDjvxwxJlTyRQW95qQG6qL5kpS2LDdu7cafp68J4IEBlfHGxkEAJa\/LoQOPQ3GkSi0ekd+ht5O1g3DQARZN0XVvTCcPJeJFKZsEiGSmex86kGtzxwRH3CyI0WFEzCUpJD2MpfR5GoxS++jIHoOYbVT+WoLeIXH2XggxPH6D3I7+DfUIdFixaVPlz0jkzU+FFAOfjQXJSKzMnehD8OIvookmwaByiL5u5k\/MU+IOstesZR3ofpGDAZ6OL4pfEk471r167A4xfHgKgDtmXEiBEB6Ye1\/\/nnny\/ZTeddp7EnLsqpPryxCObtl7uR24IFC2KtGmGHaWxsjE1uuvhf7oD0JRBBlBtOYYyq8dSRt2zZ0mOFMWoxwGRuJSwsNQ2dSO8kYWkccpMHG2Eie0hyGKobJOLHJIxcdaG46qOEWIhTEdOmTSsNJJV9cSBff\/31wTtyyC17IPIHLMq+soeiihRMIgPd1IfpGKCPTticWxSRih9X0W5hY0DXfiI38WOC\/y07FLSKH9eetNpv8sGOIryA3J555hn4yle+EhBAnHSGtOQmE4BsJCI32bUXGx5GbtSx6KtlalgELYr8dO+IxqSlbJMvUdbkJuqwdOlSePTRR4O5EdmTlcMe2XNQYanqiLowKMwm4iDEvhFFbvgOpnFg+CuTsi4aCSMpmYjFwZOU3ERPSZQne9L0m86jDiM32SlQrRaryFTuc+PGjdNO8VD7deQWliJEiwMm9qQxY43c5I5gMhEaxZphv5t+teJ4bmErKyaGlUk96sseh9xEj0Q3B5M1uclfb1zsEMMP3QBJ4rmJbZHDYPriq0hS57nJXoLYt0y8Fpm0sH+LAzlsqsIkLE2yEmk6BvL03LAumieVvdKk5Kby3MLsSba1Rm4kUP7ayl\/DNISm6pA4t2Jrzk10\/03mqUw8M3m+CV1t0VjixLQ85yaTLemkWl3DjrN48WK49dZbg9VVwkTUUZ4\/VIWFUUvn8kdMfF9FSCLZxKlfJkTMyaL2k51Qdpw5N9VKJ2KumnNT2Vb2nrFumnMjvMOITOW5yfNb4vxhlDeV9ZybjAFGOHLyd9Q7aCPCKIrc5Pk7dBbEv8lzqDp7ynNuaRwt7WqpTHRRAycu8elWZeKupoWFErqvj7giE7a7QLdaSgNUtcKpCktlL0\/GSiZGFZYm5ELlwjqEiLv8XlQoKZKUilyp\/m984xvQ0tISELX8iB8huQ\/gb\/369YPjx4+X0jaiVtfEAahaiNHVL6ctYN2zZ88G1D0Mf7E+\/G9qz7p165SrrGGrpSZjwGRBQfTKZd3D+o1MbqLnppOjC0vFDyGVTWJP66ulUcQkd\/q4W1x08pPmuclEqyK4sBVHU3JTkZI4OOMsKKCsKD3l37Fjjhw5ElatWlXK7VOtVsaZVojjUaDOYXlutKChqp88KjGXighA5a1jXWiz7du398hzM+0ncr+MikTEj5c8F4j6iB61OOelKofEL5OVyTiJapspuaG+YbJMp2ZEzFTtJ48b\/1\/eKaEK\/5PY02SqIYqz8PfYeW60iGCyzYUME7YKq\/KO5FQQ216jCTD8TrYIRIVlJsRAGibZoZBt63qfdB\/taURuST03OddJZXKUjY9qw7cuxOt9Xaf8Whw2nWCyc0REJMne0vJD1G2LbNoTxz3ufEiyh1ZEwXjOTZcZHRZyrl27NtiCFOa5YUN0pzIwubntsFnXrhoQcYlN9t7STEBn3d5yl2\/DnhTtjR492nifsA7XbuQme2hJOxoqePfdd8PXv\/51uPPOO7XkhmDg3shDhw4Fk8j4cAha7kOA28cI5IOAcvtVnPmOsFCT9lPqPDdi6alTpwZhKTH\/4MGDlaytWn3LByauhRFgBJIigItSLp4SuWHCLO7lo20TSZXBiUVMA8BN1XSKQpxtXVi+oaEhkCGeWYXENm\/ePHjqqaeSqsblGAFGwAECY8eOhaampmDlPc\/H6t5SCjORnJCYTFZL5cZimenTpwdJrSK5\/eIXv4Da2toApCuuuCJPjLiujBHAD9by5cvZthnj7EI82RYP1yg0uemSXhFUVairSitBGfPnz4c1a9Z0O1GVyM0FSC46RW+q8+jRo\/DII48Ec7MVFRW9qell31aX4zbTI4+iPDfdnNuECRN6pIa4BKnse6DjBp49exZeeuklGDp0KJObY1vYrt7luDXKc0vaYBW54Yosel+67HbdaqlLkJK2n8uZIcDkZoZTEd9yOW4zJTebxnAJks12sKyeCDC5lW+vcDlumdzKt18VpmVMboUxVWxFmdwMIHMJkoF6\/EoKBJjcUoDneVGX45Y9N887R29Qj8mtfK3M5GZgW5cgGajHr6RAgMktBXieF3U5brWem+oU2IULF8Lw4cOVJ3hkjbFLkLJuW2+Xz+RWvj3A5bhVkhsm1+JWJzxyBM+6p9QN2k5Fe0HzNIlLkPJsZ2+si8mtfK3uctz2IDfavE6JtHJeGv5706ZN3W4uz8M0LkHKo329uQ4mt\/K1vstx24Pc5MRbmdzinMRr02QuQbLZDpbVEwEmt\/LtFS7HbWzPDQ+QxAMoaYdBXmZxCVJebeyt9TC5la\/lXY7bWHNudDmJi9NOXYJUvl3Pj5Yxuflhhyy0cDlujVZLqdFJT+a1AZpLkGzozzL0CDC5lW\/vcDluOYm3fPtVYVrG5FYYU8VWlMnNADKXIBmox6+kQIDJLQV4nhd1OW61q6Um9xXkOffmEiTP+0\/h1WNyK7wJtQ1wOW61Cwr19fU9Ts+ly2+R1EaMGAE1NTWQV0KvS5DKt+v50TImNz\/skIUWLsetNhVEdwOVmPeGq6e4ewGTerN+XIKUddt6u3wmt\/LtAS7HbWQSrwy7mMR74MABaGxsDAgu68clSFm3rbfLZ3Ir3x7gctxGJvHKsMueG3ptTG7l2znzaBmTWx4ou6lj809\/BXPnzYMntnzLze1XKu8sas5t3LhxMGPGDNCFr7ahdPkFsN0WltcdASa38u0RX3h4L+x6\/hU4cM+H\/SA3hFo88oigx3sHN2\/eHFy5h9uw9uzZk9sGeia38h0ATG7la1svyc03uJncfLOIPX2Y3Oxh6ZskJre3LRJ24giTm2\/d1p4+TG72sPRN0rWNT8L\/nT7rT1iqCklF0MTw1BaYVCfKo9BXlM3kZgtp\/+QwuflnE1saeUdu4rFGmMt25MgRaG5uhv3790NdXR00NTVBdXW1rfYHcrBOfHTzeExuVuH2ShiTm1fmsKqMV+RGHhTtPMBQce3ataXz2+TDK20ggXVs374dJk2aFOTNhXluGzZsgNGjR9uolmV4ggCTmyeGsKzGidfeACQ3fLxYLZVP4kVvbf78+bBmzZpglVT+d1o88FjzOXPmQENDA7S1tUWSG9aH3uO0adPSVs3lPUHg3Llz0N7eHvSviooKT7RiNdIisOY\/n4IVLwz0h9zkOxSQ7KZPnw6LFy+Gqqoq6+SGniA+s2bNApMFBQyJR40aFQwEfsoDgc7OzuDDhnO5TG7lYVNsxbIf7IOmJzv9ITfURL4EBufD6Eo\/m2EpeoEtLS2wbNkyqKysNCI33A2Bg4Cf8kGAw9LysaXYEkoD8SYsJeVogh8XEsTVU5srpUiUS5Ys6WFZ1Ym\/vKBQngMAW8XkVn62xfQPmm\/zjtxcwG0SlrLn5sIy2dbJ5JYtvnlLR2KbvfFgsO0KnwtePwX7FlyXe8SlPBVEnGOTgcnyaj8mt7y7oR\/1Mbn5YQcbWiCxbfzlS\/DgY0dL4vpvvR1+9tP\/8Z\/c+FJmG12AZYgIMLmVR39AYrtpxd5gRwI96LX1e\/z+4OSgvOfKS54bzrG1trYaoTx37txgdTPPh+fc8kQ737qY3PLF23ZtSGZPvPAKzNpwsJvoDwy4CP5x1Dm4f\/ZtbsmNtJJTP2wDkVQek1tS5Pwvx+Tmv41UGiKpLdvxIqx78kSPn5HYts0aCSee2we1tbV+kJuvMDO5+WqZ9HoxuaXHME8J2w60w62P\/EpZJZJay9RrYPzwi4PfXY7bICyN2igvt8JmOoipUVyCZKojv5cMASa3ZLjlVQo9tIZNB+HnR86vfqoeJLWZnxoCf\/3n7wf8b3pcjlu+lDmvHsL1aBFgcvOncyCRvfFWF3zt0UOlVI4w7WRPTX6Xyc3Ati5BMlCPX0mBAJNbCvBSFEUiO3PuTbh\/y3NGREZVIaH90+eHwd9ce1lk7S7HrdZzo+ONOjo6Sg1Q7RyIbJ2lF1yCZKkJLEaDAJNbtl0DSeytri5Y+uMX4ejLnbGIDDVDMiMPTQw5TbR2OW5jXcpM26XyvGneh9jdxIj8TnIEmNySY0clkcCQeHD18ieHTscmMFEDlPOZqwdA881Xp1bMK3KTTwWRW2dz43wc5FyCFEdPfjc+AkxuZpghgXX+8U345s+OwXNtr6ciMDHERCK7e+IHuy0EmGkU\/ZbLcZvqUuY8jx1yCVK0CfmNNAgwuUEpqx\/Dxkf3nAz+TXsz02Arktj44ZfA3RM\/AO+88IJMiEylp8txm+pSZjymKK\/HJUh5tbG31lPu5EbbkU6+dg6+89RL8MKpTvjt789226Zkw\/YYTg695CL4208MhBuvHhCIjDtHZkMPUYbLcctzbratyfJiI1BUchP3UP744MvQuu93QdttelzyXBj++\/phF8M\/TLoKurrck1eUsb0jN1SYV0ujzMa\/20LAN3Ij0sIVxh8eaIfHD74cEElWpCWGjuh5Db\/sPfD344fAe991oRfeVxo7e0luaRqURVmXIGXRHpb5JwSyJjfRw8JaDxzvgB0HT8OR9tczCQ9VtqXwEL2uOz49FPpddP6uCNdhY9b90OW45R0KWVuX5UcikITcRMLqAoAnn38FNu05CW++1ZUbYYnkhB7X2Kv6wy2fHATvuPCCXkFckYb1YW+pqCTtM8Xr8\/CIcV8el18AXzAoVz1EcsPr4PDBVUMMB\/f\/9nwSeRYT8GF4kkeFpDXhw5cEk\/QXXtCHSStmJ3Q5bpWem3y2m8udCYSlS5Bi2pNffxsB0btCctq2vx1+\/dIZJ2Qle1kfGfheqBk1EC7v\/y4mrAx7rMtxGxmW4tHf9fX13Zo\/efLk3L06lyBlaPtCiRbJ6vDvXg9WB9HDcuFZEXCih\/WxyyuhZvRAeH\/lO5mwPOlZLsdtJLmJGNHuhRMnTihvhc8ST5cgZdkuV7JFomo\/8wfY\/uwpeOqFVwHnr\/IOAUUMRLLq0wdgctVlMPGjA6AP\/qMXTMC76g9Z1ety3EaSm8pz42PGs+oKyeR2I6qOP8APn2mHp198zTlRyaEg\/vuvPn4pVH\/sfUGWPP2eZEEhGVJcKm8EvCI38s52795dwoHn3PLrEjJR7Tz8e\/jJb047Df10ntWV73s3fOkTA+FDl7679EqS1AYmt\/z6V941eUVuNlZLxcuWx4wZA6tXrw5ulFc9qmRh1ZyeS5DSdAgiqzPn3oAdh04DZrJjQqjL0E81X4VR3+c+\/n749IcvgWsuf2+wNSgJUSXBisktCWrFKONy3EaGpXEhlK\/+E2+uV8nC9\/GJuk3LJUgqvYm0MGsd72nMI4M9zBbiXBW+9xdX9Q9CwEvfnlwXQ8S4Ns36fSa3rBF2J9\/luLVKbqrjkqIucUbymzRpElRXV4dawBVISGI\/evZUsA0n6+03Ko8K\/4Z5VjPGXwEdZ99MFf656+LhNTO5+WqZ9Hq5GreouVVyU0ER5rkhGc6ZMwcOHToEx48fD4rr0kwIpA0bNgAmGGf1IJl97buHrRHZ4H7nt9nggwmhY67sD5OvvQz6v7siCPvyDP+ywiytXCa3tAj6WR6nuI4dO1Z+V\/vRvFvYTVk0vzd16tQgLCXPb\/DgwT3y6Ijc0Ix1dXUwbdo0axbFrPh\/\/q9T8PTxP92UHSWcSOvyvhXw8UHvgi9eUwnvuPB8uoJIaFFy+HeAc+fOQXt7O+D5gBUVf\/oYMDbFRmDdunWwfv36oBFOb5zPCkZcMGhoaICWlhaoqqqKrEb3PpFbU1MTjBo1KhgIaZ\/Hfn0K\/u7bh0PF0BlZcz5zBQzp\/87cJtnTtq1I5Ts7O6GtrQ3wQ8jkViTLheuKzsvWrVth+fLlfpBb1I3zUXNocnOjji2X39fVbyt2xzAQ\/3fTir1KyyCZfWr4JTCv+komspzGGYelOQHtoBpb4zaJ6spjxqdPnw6LFy9WelryaqhYKYWZCxYsKC0QhJGbiijRc5s\/fz6sWbOmm3dmAyQktdkbDyrn0\/A0h1s+eXnppuwkYHKZZAgwuSXDrQilbIzbpO0skZu8WT5MYNgOBfkCGRMylOfcJkyY0CM1JC1I5K3JZ3st+NyH4J6\/\/GBS\/LicBQSY3CyA6KmItOM2TbNie24mlYlEKS8oyORH3h6uquATtVqaZGISCe3axie7qY7h57ZZIzn0NDFoxu8wuWUMsEPxXpGbQxxCq04KkorYvvnVjwXnc\/HjBwJMbn7YIQstko5bG7oEnpuYkjFlyhSoqakJ8lN0T1h6hw2lVDKSgoQemxiKorc2fvjFWanJchMgwOSWALSCFEk6bm00L\/MkXhtKooy4IKkWD9qX3FA6TdWWXiwnPQJMbukx9FVC3HFrsx1lTW7iPNsDXxgGd974AZvYsSxLCDC5WQLSQzFekZs8wa\/Cqwhh6YB7\/rukOi4e7FtwnYemZ5UQASa38u0HXpFbGMxxE3htmiwOSHO\/9xys3XV+ryo+SGx5Hd9js829RRaTW\/laOs64tY1C7LA06ggj2wqSPFOQ5NXRb331z+BLn7gsK7VYrgUEmNwsgOipCNNxm4X6scnNlfdmCtIX\/30f\/Ozw7wOsOBzNosvYl8nkZh9TXySajtss9I1NbmE7DrJQMK7nJs61bZt1LYwffkmWarFsCwgwuVkA0VMRhSE3G0eQJ7WBCUj\/+qMXYOmPX2SvLSnIjsoxuTkCPodqTcZtVmoot1+FJfFG3YmQlaJRIMlzbQ\/fcg3gZnh+\/EeAyc1\/GyXVMGrcJpVrUi52WGoiNIt3okCSyY13ImRhhWxkMrllg6sPUqPGbZY6askNV0Xx8mW6uYruL125cmXkfQdZKBwF0hce3ls6ymj8sIth2+yRWajBMjNAgMktA1A9ERk1brNUU0luukUDuoZv5syZkbdV2VY6CiRxIYHz2myjn608Jrds8XUpPWrcZqmbds6NzliTK5ePLMpSOVF2GEgbf3kyOIQSH07\/yMsi9uphcrOHpW+SvCQ38TRdETAf89zEkBSvwWu941rfbMz6hCDA5Fa+3cMrcgu7gQpNIM\/F5WWWMJC657bxkUZ52cRWPUxutpD0T45X5Ibw0OKBfJy4y0UFHUg\/P\/JK6bIXDkn969wmGjG5maBUzHe8IzeEUXU6iIvTQMikOpDEkPT6YRfDD3iVtHCjgMmtcCYzVthLcjPWPqcXVSDJuW33f\/ZKuP+zV+WkEVdjCwEmN1tI+ifHK3KLe89oXnCqQBJDUtSDE3fzsobdepjc7OLpkzSvyE1192iWYBGZ7t69O6hGd22gCqQHH\/tfePCxo0E5nm\/L0krZymZyyxZfl9K9IjdaUGhsbITNmzd3uxjZNkjyymwYsapA4hQQ2xZxI4\/JzQ3uedTqFbnlecw47nhoaGiAlpaW0u32usMwZZB4o3weXTOfOpjc8sHZRS1ekZsLAKjOOJ4bz7e5tJTdupnc7OLpk7ReT27ivFvUjfMbNmyA0aNHw3f3niptuRrcr4IvgPGpR8fUhcktJmAFeR2dFbz\/uLa2Fnbu3AmYSpbn492lzLq9q\/QFQHDq6urgiX6fhT3HzgZYjbriIvjmFD67Lc+OY7Ouc+fOQXt7ezC\/W1FRYVM0y3KIwLp162D9+vWBBs7IzWH7e1StmofDl4jcmpqaYNSoUTD52y+VbpK\/d+IQuHfiUJ+awbrEQKCzsxPa2tqCLzuTWwzgPH8VPbetW7fC8uXL\/SK3PM5zU23CjyI3\/AK89Z5LQbxwmfPbPO\/lEepxWFps+4Vp792cW17nuckJw2Gb9kWQZHLj89uKPTiY3Iptv8KQG61Y5nWem5x6ErWggJ7bd579IyfvltF4YHIrI2NKTfHKc4vaoeDDeW7ztp+Gx379cgAjb5Yv\/sBgciu+DXUt8IrcinCe2x1b20v3JfBm+eIPDCa34tuwEOSGSvp+ntuI5sMlLHkxofgDg8mt+DYsDLmhor6e5\/bt1sfh84\/8toQlLyYUf2AwuRXfhoUiN9\/gptj93\/5jG2BYSs\/p5ht9U5X1iYkAk1tMwAr0uldzbr7iRiD9y+qtcOcPTwVq8jFHvlornl5MbvHwKtLbTG4G1iKQLp66HI6erQxK3PiRAfC9+iqD0vyKzwgwuflsnXS6MbkZ4Kcity31VXDDRwYYlOZXfEaAyc1n66TTjcnNAD8C6ZXJa0pvP3zLNXDLJ3nDvAF8Xr\/C5Oa1eVIpx+RmAB+CNHXGXfBa9YOltzkNxAC4ArzC5FYAIyVU0Ttyk+81kNvl4oo\/BKnmrgfgzPj7SupwGkjCHudZMSY3zwxiUR3vyA1PBNmzZ0\/mdyjEwRBBmrLoUTj70ZuCYrxSGgc9v99lcvPbPmm084rcojbOp2lomrIyufGe0jRo+lWWyc0ve9jUxktyW7BgAVRXV9tsZypZCNJNK\/bBG5deHchhcksFp1eFmdy8ModVZbwit6iN81ZbHkOYTG68YT4GeJ6\/yuTmuYFSqOcVuWE7aOP8ypUrvfHeEKTPbe4swcxpICl6nGdFmdw8M4hFdbwitzzvLY2DoUxunAYSBz2\/32Vy89s+abTzitzSNCTLsjK5cRpIlmjnK5vJLV+886yNyc0AbZHcOA3EALACvcLkViBjxVTVS3ITw9MxY8bA6tWrYeHChTB8+HCYNWtWzCamf53JLT2GvkpgcvPVMun18o7ccEFh3rx5wYWqu3btCu4cRHLDldSamhrQXR6THgq9BJHcOA0kS6Tzl83klj\/medXoFbnJ1+3JN8Drrv3LGiyR3BpuGAqLbhqedZUsPycEmNxyAtpBNV6Rm3z7lUxuJrdf4fat1tbWAMqofah4CXNdXR10dHSUoFdd7yeSG+e4OeilGVbJ5JYhuI5Fe0VuUZ6bfBO9jJ38u0yO8vv4Oz5R83giuXEaiOMea7l6JjfLgHokzityQ1x0c244B7dkyRLQJfeq7jzFv02fPh0WL14MVVU9T81FMpw0aVJksjCTm0c91rIqTG6WAfVInHfkhtioknn79u0bLDKoSEqHJ4adDQ0N0NLS0qMceolz5syBQ4cOwfHjxwMRYTfO0w4F9tw86r0WVGFyswCipyK8JDdbWKFnhk9zc3MPkfIJJGH7WkXP7XcPfcqWeizHAwSY3DwwQgYq4Pg+duwY1NbWBhkXOP+e59Onq6urS1WhynOjfLfKyvMXtEQ9Sc6F03l6RG4XvH4KGgY9C9OmTYuqnn8vCALotW\/ZsgVuvvlmGDSIj40viNki1Vy3bl0Q6eHjDbmJc25iCIqT\/6tWrTIKTZMQG4Kgm6MTya31q5fzIIjsWsV5Ab\/ut912G6xduzb3r3txUCqepjiWn376aVi+fLkf5EahIXY21XluUaulVB5NgYm\/YV6eKq0EPbf58+fDmjVruhEYDoARzYeh4tRvoPLnDxXP0qwxI9BLERg7dixs3Lgx99b3CEujVjej8tyiyE9soW7ObcKECcrUECQ4\/B8\/jAAjUBwEcK4t7\/k2RKcHuUV5bmF5a6qEXDIBpY\/I5eW5Pd1qaXFMyZoyAoyADwgoFxR0h1UiMYXlufnQINaBEWAEGAGl52ZyWKUIHbqbuBLCDyPACDACPiGgTQXxSUnWhRFgBBiBuAgwucVFjN9nBBiBQiCgJTfV4kCS7VeFQIGVZAQYgbJDIHRBYe7cud2Xj8QkAAAGCElEQVRSMlwsKNDq7e7duwPweTW1uH2Q+g+2wNZRWMVFo\/doHpWBkRUS2lQQXa5Z1BFGthUV96bKxzHZrovlZYeAnB8Z1Y9Mj8LKTmOWbAMB0TnJ+6pQZRIvHiWuu3E+KonXBiAkQ7XPFOvHbTpRux9s6sGy0iGg+ihFJYubHoWVTjMunSUC5KnjnvQTJ05oOSUrHbz23HTbs3RHKGUFEstNh4DqnL8wLzzOUVjpNOPSWSLw\/e9\/H6677rqgijCHKSsdvJ5zU3lpUV\/8rIBiuckR0NkMvTPVbWpxjsJKrhWXzAsB1cctj7q9Xi1lcsujC2RfR1xyU2kUduhp9i3gGtIg4B25pWmMrbIcltpC0q2cuGGpSlv22N3aME3tTG4K9HhBIU2X8qds3AWFOEdh+dNK1kSHAJObBhlOBSmPQRMnFSTuUVjlgVD5toLJTWNbTuItn06vS+LVeXa4wkbn93HydnH7AZNbcW3HmjMCjICHCPDGeQ+NwioxAoxAegSY3NJjyBIYAUbAQwSY3Dw0CqvECDAC6RFgckuPIUtgBBgBDxFgcvPQKJjfh5cUP\/DAA8baJSljLFx4UTyGPu4l3Sgm6jSQJDplWQZTkVpbW4Mq8jrVQs4QyKveLHF0IZvJzQXqIXVSxx48eDA0NzcbaZekjJFgxUtxrm5U1VFEcsMTLVycQkMHxjY1NSnvEE5qw95SjsnNM0snIaokZZI2m8ktKXLxyzG5xcdMLMHklg6\/RKXlI9zpVFoUJiauiqfViuERvkchIRKbroyYNGsaVsll6DRmOVQKkyffoCa2gzy3kSNHwqpVqwL85OPrVXWJSbx0oAKW27FjRwkLlDVjxgygU5snTpwIHR0dIHrBsm5RobVM5rQl8Pbbb4elS5cG8qOwDdtbG+ahM7klGl6lQkxu6fCLXVrVYcUBRANU7PTyAKPBMnr06CB0VXluSCJIHuvXr4eqqiqgemfOnNnt6HixAVgPkkVYmSjPTVUP6rJp0ybYvHlzMJeId98SWZHuqAeGfqr2yzLpXl2R8GQMRIKk92TcsC5sz549ewLdBg0a1MOeKnKrq6sL3iOcdPf8kjAmt9jDxEoBJjcrMJoLwYEwb9680sCQS8qDVLd1JYwQ5b2ZVIdIMvJA1nkJcpkocouaU1PpIGKCusqHkcqYqDDUbbZHIkIPDj8Cqrp1WBFmOnKTPxJhuDC5mY8Pm28yudlE00CWGBbJF\/Bg8bD5M11IJXs7OgINOxMtrAwSBE1qhw1ik7k\/FflF1U2hH3lgKiJT6SXqs2jRoiBkVYWB4uEMsgl1YWlLS0vgEdMTdvw+k5vBwMjgFSa3DECNEhk2f6UiCHG+jeaIFi5cGJxLrwrlKExS6aG7nlE3OGWPzoTcdJcLoT5R5DZs2LBu82ZIaDIxpSE3mo+TsdFtzGdyi+rN\/v7O5ObYNkRmBw8eDEJVGtzkYejCxbCwNCr0VTXZ1NszIbewSfIocmtra+sRtqvC0sbGxm7zZGk8t7AuYEpuYSE\/e25uBhmTmxvcu9Uqdv5x48Z1C59UXgq9jySi8tx0hBgWOtmac0syJycS665du2Dnzp3d8spIN5o7U7XDZM4tjAB13macOTc0qio3UTWvp1rckLsir5amG5xMbunwi11a5SGJX\/3Kyspu5KZafaQwlVIs5DIU\/omrpVET51gmq9XSKPISf0fPrb6+vrQbQAzhKSR\/4oknQPbcwlZLaW5TRShhHhdhIibxkj3wN3G11GSRCMtQMjDZMOycOia32MOrWwEmt3T4JSotz4nJ82D0O\/2dBjxVhgMCb40SyUsug5Pdupy1MKWjcuOiPDOULS98iO2LCkt1eh85cqSUsnHgwIEe5Ib1ynOZuKK5d+9eEL2ysBw8FS46z622thY2bNgQ5LnJ9lMt3Mj1IuFim8jbU5Vhcks0vEqFmNzS4celPUbAxgmwOnLLY0sUk1u6zsXklg4\/Lu0JAnLSMoXmlDysStA1UZ3JzQQlP99hcvPTLqxVAgTElBksLm77SiAuKCLKxNM5Bg4cCGLeX1K5YeX4VBA7qP4\/c\/H8KP7dVe4AAAAASUVORK5CYII=","height":150,"width":249}}
%---
%[output:4e40d541]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"     1.320000000000000e+01"}}
%---
%[output:563fc97c]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"     7.500000000000000e-03"}}
%---
%[output:6761ce4b]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"     7.500000000000000e-03"}}
%---
