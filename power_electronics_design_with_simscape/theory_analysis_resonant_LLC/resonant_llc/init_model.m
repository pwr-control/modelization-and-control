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
tc = ts_dab/10;

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

Idc_FS = max(Idab1_dc_nom,Idab2_dc_nom) * margin_factor %[output:1dd96712]
Vdc_FS = max(Vdab1_dc_nom,Vdab2_dc_nom) * margin_factor %[output:174c5e41]
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
Ls = (Vdab1_dc_nom^2/(2*pi*fPWM_DAB)/Pnom*pi/8) %[output:6e4cc00d]
fres = fPWM_DAB;
Cs = 1/Ls/(2*pi*fres)^2 %[output:01f62bda]

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
Vac_FS = V_phase_normalization_factor %[output:231dc037]
Iac_FS = I_phase_normalization_factor %[output:1bc16b1f]

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
Ldrso_pll = acker(Adrso_pll',Crso',polesdrso_pll)' %[output:26989b55]

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
Afht = [0 1; -omega0^2 -0.05*omega0] % impianto nel continuo %[output:75c58cb8]
Cfht = [1 0];
poles_fht = [-1 -4]*omega0;
Lfht = acker(Afht',Cfht',poles_fht)' % guadagni osservatore nel continuo %[output:6a06269e]
Ad_fht = eye(2) + Afht*ts_afe % impianto nel discreto %[output:0c76ff32]
polesd_fht = exp(ts_afe*poles_fht);
Ld_fht = Lfht*ts_afe % guadagni osservatore nel discreto %[output:1ba8c347]
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
Ldrso = acker(Adrso',Crso',polesdrso)' %[output:16217f48]

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
figure;  %[output:28c3084b]
plot(Zmodel,ocv_model,'LineWidth',2); %[output:28c3084b]
xlabel('state of charge [p.u.]'); %[output:28c3084b]
ylabel('open circuit voltage [V]'); %[output:28c3084b]
title('open circuit voltage(state of charge)'); %[output:28c3084b]
grid on %[output:28c3084b]

%[text] ## Power semiconductors modelization, IGBT, MOSFET,  and snubber data
%[text] #### HeatSink settings
heatsink_liquid_2kW; %[output:2330891c] %[output:55c0aace] %[output:6d4591c7]
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
%[output:1dd96712]
%   data: {"dataType":"textualVariable","outputData":{"name":"Idc_FS","value":"   275"}}
%---
%[output:174c5e41]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vdc_FS","value":"        1875"}}
%---
%[output:6e4cc00d]
%   data: {"dataType":"textualVariable","outputData":{"name":"Ls","value":"     1.420454545454546e-05"}}
%---
%[output:01f62bda]
%   data: {"dataType":"textualVariable","outputData":{"name":"Cs","value":"     2.853204531368232e-06"}}
%---
%[output:231dc037]
%   data: {"dataType":"textualVariable","outputData":{"name":"Vac_FS","value":"     5.633826408401311e+02"}}
%---
%[output:1bc16b1f]
%   data: {"dataType":"textualVariable","outputData":{"name":"Iac_FS","value":"     3.818376618407357e+02"}}
%---
%[output:26989b55]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso_pll","rows":2,"type":"double","value":[["0.283130953403918"],["67.668222262819981"]]}}
%---
%[output:75c58cb8]
%   data: {"dataType":"matrix","outputData":{"columns":2,"exponent":"4","name":"Afht","rows":2,"type":"double","value":[["0","0.000100000000000"],["-9.869604401089358","-0.001570796326795"]]}}
%---
%[output:6a06269e]
%   data: {"dataType":"matrix","outputData":{"columns":1,"exponent":"5","name":"Lfht","rows":2,"type":"double","value":[["0.015550883635269"],["2.716608611399846"]]}}
%---
%[output:0c76ff32]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"Ad_fht","rows":2,"type":"double","value":[["1.000000000000000","0.000200000000000"],["-19.739208802178720","0.996858407346410"]]}}
%---
%[output:1ba8c347]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ld_fht","rows":2,"type":"double","value":[["0.311017672705390"],["54.332172227996914"]]}}
%---
%[output:16217f48]
%   data: {"dataType":"matrix","outputData":{"columns":1,"name":"Ldrso","rows":2,"type":"double","value":[["0.073386376052051"],["3.802432508328568"]]}}
%---
%[output:28c3084b]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnQ10XsV55x8SWln5IEYgJCTHMQEZaE\/XYJdaGBKaAvFue2S2SVNJ7FmrrpK6BwjeA7Ykh4RdGoIt2Wbr8rF1qeLKu\/VH0kKD+rGUsISGiPiwBrRpQoJNsI0lpAiMAwTjhER7nuuOGF3dj5l553nvfa\/+9xwf23pnnpn7e+ad+euZr1MmJycnCQ8IgAAIgAAIgAAIVBCBUyBgKshbqCoIgAAIgAAIgEBAAAIGDQEEQAAEQAAEQKDiCEDAVJzLUGEQAAEQAAEQAAEIGLQBEAABEAABEACBiiMAAVNxLkOFQQAEQAAEQAAEIGDQBkAABEAABEAABCqOAARMxbkMFQYBEAABEAABEICAQRsAARAAARAAARCoOAIQMBXnMlQ4rwSOHj1KnZ2dQfX6+\/uppqZGrKr79++nVatW0ZIlS2jjxo1UXV0tVpZuuJzvaPJCisNnP\/tZam1tNcmS6zS9vb20bdu2oI6rV6+m7u5uq\/ru2bOH1q9fTxs2bMglD36\/b3\/72+LfDytoSFyxBCBgKtZ1qHjeCJRzcI8TMDxAXHHFFdTc3CyCJ+4dpcuNexmXAZEH0Mcee8xKHLjksXUAl3HttddOZSuigCma4LT1MdL7JQAB45cnrIFAJgSOHz9OPT09NDg4SDt37iybgOHITznKjYKqBsOWlhZjMaIiFDbiwCWPSyNQAsambuFy8h6BUe308OHDiMK4NBLkmUYAAgYNIpcE9FA6V1APievRh4suuoi++MUvBu\/AA5k+naKiBcPDwzM+121cc8019OlPfzpI09DQQNu3b6empqZILrpQCKcPRyf4czWldOWVV9Kdd95JixYtCjpufeBPs8NTUarcffv2BfXjR00h3XrrrfQnf\/IngXhRT5gF\/1wx1QWOGjT19GoQVLb0AVV\/x7vvvpv6+voiyz1y5EhQv9HR0ak66T7UOTLze+65h7785S+Tej\/mH2at2KmpuajBWvlVL1e9b\/i9lK\/nzZs3JcLC\/B588MFgSkY9evsI20sTjuH2mGQrqR0mRWp0JlxnVfewKAp\/v3S2ysZNN91EjzzyCPH3R\/lOf2f+mSpD920al6h2mMtOCJXKPQEImNy7aHZVMDxo6W+vOuGoQSo88LAdFg9KvIQ\/jxpgkwZ\/\/iyubmqwOeOMM6atgVECRq8DC4UowaGLmLAdXwIm6jf88GASHtjiuPLP4wRMV1cX3XDDDTPYJwkG\/qy2tpYmJiYCgRYlKrhMfaAN1z2uXahyn3rqqUgxcv\/990+tO9Hbmz5AhwVM2Jb6PE7EJLVZznPo0KFYoaTXKSxewiJTiYePfOQj9M1vfnNa5xElQqK+X2EBwmmi6sg\/V+Wk2da55D1KNLt63Mp+WwiYyvZf4WqvOmj9N1DV+fPL6tEH\/i1bdYz6AKF3tvpvnvqAxyJBRQiUDVV2+Dd9BTnqc70zvvrqq2MFjP4bqq2dNAHDUSd+0qZykiJEHBV65ZVXZjDRowbMaeHChdPe0WQKKTy9pdgrf3K0Jex3rguvB4mKDDHLFStWBO+rR2xMFjabTAeF04T\/r5goscX1Tytbtb2o91E\/Y6HL7xw3haRzVO0p7NOHH344EEJREZU4u+EonIo66TaSylYRGtX+07j4mCorXMeHF3IiAAHjhA2ZpAjE\/XamBgDuuBcvXhy5A0dPc\/Dgwcjfqrneug3+rV\/tGEpbhJv2m2OcQNA7dC7f1o4vAaOXzWKEH33AjBtY9AH8M5\/5jLGAiYpYRZXL9QhPkcVFODgtD8SqHjrbqPLCU2lJAiZu6iycJymaEiV+w++mpifDQkiJtjihkdY+df\/qNuL8Go7mKFZKwMRNHeo77PS2rL6X+vSd6id0LlHTllL9CewWmwAETLH9W3FvV24Bo29DThsgbIUHw4\/aVm1qRx+cw4Md29a3UZtEYDiNvvCV\/89bdsMRqPAAaitgwhzDUZqwcPIlYFRjjxNOvDMrSsCEp6LSIjCVIGCiIn7Kr+H2FxeB0W3EfTcgYCquiy1UhSFgCuXOyn8Z1ymk8FSHWlMQ99tsVMg\/TcAkTf3oUQH2Av+WGidgTO1waD4sLtTUWljAsEgwWRwZHtz1CEV4Go4H\/LQpJI4OpQmAsA3XKSS9dcdFNcLfgLAYCUcj1DvrkTj1PqrthPNETSGlffOkp5CU2FWRqzgBExW5UozCEZi4Rdfh6aukKaQoLphCSmst+NyUAASMKSmkKwsB6UW8SQIgTcAk1S1qfUicgEmzw+F2tZ4lDN1EwHCeqF1IylZ4J4l+AJzNIl41laDn4XI\/8YlPBNGhqIc5Rb2f6SJetqlEXVg4xS1w1fPoabjMrVu30u233z5jwTHnCQsY\/lncgmD1rmmCOWp6JS0CpnOMe8ck8aELhhtvvDG2bSXZ4DpELe41XcSrc0mLQJalo0EhhSAAAVMINxbvJUy3UetboNO2UUctDLaZQmLKSdMTaYtk9ZN5k+xwOeEtt1\/4whfomWeemVq0GhWB0Qe3uIXIuu3w2pwogaMP5Hpe\/rcSMFHl3nfffdNOlOXD9fT1Ni7bqHUhog+oUVvs47Zvh7nqa3LYJnPbvHkzrV27NsChR9LUbrK4bdlp57ckbaPmskwjE3FrVzgKFyUO4qJOzChqC3tUFCdO\/PLPwyf\/xq0lUjZMIoXF69HwRhIEIGAkqMKmKIG0HR+ihcN4yQSipqrCO83izuHRC3c5yK7kys9SA7rgVEItamdSGh4cZJdGCJ\/bECiUgOEOjc+g4MO1ojrApIPNbKAhbbYEIGCy5V9q6UlTaElTX1HlulwlUGr9Z2v+qCkkZpF2+GOU6CzK3VWztS3k5b0LI2DSFvWpz5ctWxZccqb+z18+2wvT8uK82VoPCJjK93z4lwl+I1vxwnlwt05520J4atdGvHBNITjL66+il1YYAcPzvPzl4CcuAhN2Jv9GMTQ0VNbbfIveoPB+IAACIAACIFAOAoUQMPzb3G233Ubt7e2BiPEpYPjIef6DBwRAAARAAAQqlQBfz8F\/ivQUQsBwJIUfPgkyaQ2M7jgVwm5rawumlKIeFi7r1q2jvXv3FsnneBcQAAEQAIFZRmDp0qW0adOmQomYihcwPAc+MDBAt9xySxApMREwav0Lt1\/99uJwe1bbD9npjY2Ns6y5y74ui0I+gwNs\/XMGW\/9MlUWwBVs5AnKWVbtNuzFdrgYylitewPCUEZ8xwaeGpu1CYoSm4oXTKgFTNKfLNCU7q2Brx8smNdja0LJLC7Z2vGxSg60NLbu0RWVb0QImaieDcmuU6LDdeVRUp9s1fZnUYCvDFcJbjivYgq0sATnrRe1vK1rAhN2dFoHhaA2fPpk0baTbLKrT5b4m5pb5tmie+uvo6KAFCxaYZ0TKVAJgm4rIOQHYOqNLzQi2qYicExR1LCu0gFERF96dtHDhwuBmYHUcuGoJSUeuF9Xpzt8Cjxnfeusteumll+jss8+mOXPmeLQMU2Ar1wbAFmzlCMhZLupYVigB49v9RXW6b04u9jAQuFAzywO2ZpxcUoGtCzWzPGBrxsklVVHHMgiYhNZQVKe7fAF850Fn5ZvoO\/bAFmzlCMhZRruVY1vUsQwCBgJG7luTYBmdlRx2sAVbOQJyltFu5dhCwMixza3lojo9D8DRWcl5AWzBVo6AnGW0Wzm2RR3LEIFBBEbuW4MIDNhmQkCuUAyyYCtHQM4yBIwc29xaLqrT8wAcA4GcF8AWbOUIyFlGu5Vju+exf6X1q1ZQ0Q5lRQQGERi5bw0iMGCbCQG5QjHIgq0cARnLh4++RRfd\/gTN\/btOCBgZxPm0igiMnF8wEICtHAE5y2i3YCtHQMby4weO0Yp7n4aAkcGbX6sQMHK+wUAAtnIE5Cyj3YKtHAEZyxAwMlxzbxUCRs5FGAjAVo6AnGW0W7CVIyBjGQJGhmvurULAyLkIAwHYyhGQs4x2C7ZyBGQsQ8DIcM29VQgYORdhIABbOQJyltFuwVaOgIxlCBgZrrm3CgEj5yIMBGArR0DOMtot2MoRkLG868kxun7Xs1jEK4M3v1YhYOR8g4EAbOUIyFlGuwVbOQIylpWAOe2fu2n3X\/4ZNTc3yxSUgVWcA5MAHQJGrkViIABbOQJyltFuwVaOgIxlRGBkuObeKgSMnIswEICtHAE5y2i3YCtHQMZy70MvUO9DBzGFJIM3v1YhYOR8g4EAbOUIyFlGuwVbOQIyliFgZLjm3ioEjJyLMBCArRwBOctot2ArR0DGMgSMDNfcW4WAkXMRBgKwlSMgZxntFmzlCMhYhoCR4Zp7qxAwci7CQAC2cgTkLKPdgq0cARnLtw4eoLsffRFrYGTwZmN1\/\/791NXVRX19fdTU1BRZCQgYOd9gIABbOQJyltFuwVaOgIzllnuepm89fwwCRgZv+a0eP36cenp6aN++fbR9+3YImPK7gDAQyEEHW7CVIyBnGe1Whi0EjAzXzKxyZKW3tzcoHxGYbNyAzkqOO9iCrRwBOctotzJsL7r9CTp89C1EYGTwltfq0aNH6bbbbqP29vZAxJgImDVr1tDSpUunKlpfX0\/8B487Ae6sxsbGqK6ujqqrq90NIecMAmAr1yjAFmzlCPizzH0r\/+Hnt79yPPh77t910s6dO3ESrz\/M5be0Z8+eoNDFixcbr4EJ17Kjo4NWrlxZ\/soXqMQTJ07QxMQE1dbWUlVVVYHeLPtXAVs5H4At2MoR8Gd5x44dNDAwQL94z5n02sdPzjZAwPjjm4klXrjLTr3lllvoyJEjxgJm06ZN1NjYiAiMR6\/xOqTx8fEgkjVnzhyPlmEKbOXaANiCrRwBf5ZVBIYX735p38lfECFg\/PHNxBJPGV1xxRVBCA27kDJxwVShmO+W4w+2YCtHQM4y2q1\/to8fOEYr7n06MPy+x\/voK3\/2XzGF5B+zvEVe+9LZ2UnDw8MzCoubF8Q2ajm\/oLMCWzkCcpbRbsFWjoB\/y9fvepb4Mkd+cBu1f76ZWUQEJjP0QcEYCOT4gy3YyhGQs4x265+t2kL9rjdfDgQMFvH6Z5yJRQiYTLBPFYrOSo4\/2IKtHAE5y2i3ftny1mneQs3PqS\/\/IJhCgoDxyzjX1jCFJOcedFZgK0dAzjLaLdjKEfBrWV\/\/csUHxml44HMQMH4R59saBIycfzAQgK0cATnLaLdgK0fAr2U1fcRWb1lygu75wnUQMH4R59saBIycfzAQgK0cATnLaLdgK0fAn2V9+mh+zRz686tOoWuvvRYCxh\/i\/FuCgJHzEQYCsJUjIGcZ7RZs5Qj4s8w7j3gHEj8PXncxnfry9yFg\/OGtDEsQMHJ+wkAAtnIE5Cyj3YKtHAE\/ljn6wme\/8N8cfXnm85dSUceyUyYnJyf9YCuelaI6PQ+ewkAg5wWwBVs5AnKW0W79sO196AXqfehgYOyu1gvoPy09GwLGD9rKsgIBI+cvdFZgK0dAzjLaLdjKESjdcnjtC0df+CnqWIYITEKbKarTS\/+alG4BA0HpDOMsgC3YyhGQs4x2WxpbFi+87oXvP+LnnvYLqf2SegiY0rBWbm4IGDnfobMCWzkCcpbRbsFWjkBplrd8\/RB96R9\/GBi57Ny5NHj9xVMGizqWIQKDCExp3xrH3BgIHMEZZANbA0iOScDWEZxBNrA1gBST5G+fGqfP\/K\/vBZ+qhbt6UggYd7YVm7OoTs+DQ9BZyXkBbMFWjoCcZbRbe7Y8bfTQ916m7vv3T4kX3jbNIgYCxp5noXJAwMi5E50V2MoRkLOMdgu2cgTsLLN42fXkS1M7jqIiL8piUccyTCEltJmiOt3uayKTGgOBDFe2CrZgK0dAzjLarR1bvqiRRQw\/LF7ubruQLj9vbqSRoo5lEDAQMHbfGk+p0Vl5AhlhBmzBVo6AnGW023S2LFgef\/4Y3fBvp+wq8RI1baRbg4BJZ1u4FEV1eh4chc5KzgtgC7ZyBOQso90ms+XbpW\/Y\/exU1IVT9\/z7c6jr4wtSnVLUsQwRGERgUhu\/RAJ0VhJUT9oEW7CVIyBnGe02nq1+s7SKunQvP2fqnJc0r0DApBEq4OdFdXoeXIXOSs4LYAu2cgTkLKPdTmfL00V8LQBfzKg\/\/3np2XTz1Qtm7DRK8kxRxzJEYBCBkeuREiyjs5LDDrZgK0dAzjLaLQXTQ0q4qBN1FXFeqJu21iXOOxAwcu02t5aL6vQ8AEdnJecFsAVbOQJylmd7u9VvkdYplyJclJ2ijmWIwCACI9cjIQIDtpkQkCt0tg+ycmRn59otXpjL00ThaAtz5nuMeJ1L+FA6Fx9AwLhQK0Oe48ePU09PDw0ODgalrV69mrq7u2NL3rNnD61fvz74vKWlhTZu3EjV1dWR6Yvq9DK4JbUIDASpiJwTgK0zutSMYJuKyDnBbGHLouVrwz+i\/m+NzGDFYqXt1+vp2t8424twQQTGuTmWJ2Nvb29QEIuWo0ePUmdnJ7W1tVFra+uMCrAg4fT9\/f2BaGHh09DQECt4IGDkfDhbOis5gvGWwVaOOtiCrQsBFi1\/+\/Q4DTwxGpk97SA6lzL1PEUdywo3haQLmrDTOfoyNDQ0FXUJ\/z+cvqhOL\/XL4CM\/BgIfFKNtgC3YyhGQs1y0dps0PcQUlWjhv31MEyV5pqhjWaEEjIrAcDSmubnZKAKzbNmyyGgNZy6q0+W6IHPLReuszN9cPiXYyjEGW7CNI8CLcHlaiKeH1BH\/4bTlFC2IwMi1Ve+WOfKybdu21HUt+\/fvp1WrVtHo6Cjt3LkzUuioyikBs2bNGlq6dOlUnevr64n\/4HEnwAPB2NgY1dXVxa5Bcrc+u3OCrZz\/wRZsmcDoa28HIuVnb\/+c\/uyx0chFuJyu4bRT6ez3n0rdy\/nclmrxSIvyDvet\/Ec9IyMjtG7dutQxT867MpYLFYFhRCxkWJxELc7lKaPdu3cHa2BqamqCtPzELfpVAiaMvqOjg1auXCnjkVli9cSJEzQxMUG1tbVUVVU1S966PK8JtnKcwXb2smXR8pXvvEbfG\/8p7Rs5eYli1MOi5Xd\/9f206OwqWtI4Rw5YguUdO3bQwMDAjBRpv7RnUtkSCi2cgOEIS1dXF\/X19VFTU9MUGrVbSZ8yikurMikBs2nTJmpsbJyyhQhMCS3u37KyP8bHx4NI1pw52XzJS3+LfFoAWzm\/gO3sYfv4gVfpb5\/6EX3zh68FEZe4h6eFLjt3Ln3q4togwsICJusnHIHZu3cvbd26FRGYrB2TVr6+04ijLOopRcAUTbWmMSzH51hLIEcZbMFWjoCc5SzbLS+4\/clPf053P3o4djpIvbkSLO2XnNzqLL0A1wfxoq7nzCQCoxbbDg8PW\/lm0aJF9MADD0zLo08DKZEStzU6agopbrqJCymq062gCyXOsrMSeqXcmAVbOVeAbeWzZbHy4+M\/oz\/\/lyOpYoXflgXKby6soZuu+lDw8pUgWMJeKupYlqmAidstFPUVUZGVsIAJH2SnH06nPmtvb59arKsW+3IZOMhOrjNKs4yBII2Q++dg684uLSfYphFy\/1yCLYuVE2\/\/gv70kUPGYuWDp8+hruUL6ENlXHTrTs0sJwSMGSejVGnbnW0EjFGBjomK6nRHHF6zSXRWXitYwcbAVs55YJs\/trwbiKMi\/Pd\/f+QQHfjRm1ZiZWVzA539gaqKmQ5y8UBRx7JMIjAcGeE\/+hoVF6dI5ymq06W5mdjHQGBCyS0N2LpxM8kFtiaU3NKYsuWoystv\/DQ4cyXqDqGo0lngcGTlo02n06UfnkuXnzfXrZIVmquoY1kmAkZfA5N2d1GW7aWoTs+SqSrbtLPKQ10rrQ5gK+cxsC0P2x+9SUFE5ekXX6N\/\/t4r9OKrb8UeDheukRIrfBEiP5Wy0FaObHHXc2YiYJSj9PUovPB2+\/bt07Y+SzrUxDYEjAkltzQYCNy4meQCWxNKbmnA1o1bXC4WKfznhVeO01f+7xi9MPFG4pZl3Y5aTNv66\/X0kfNOh1BJcE1Rx7JMBYziHd6VlJeoTFGd7rcLcrOGgcCNm0kusDWh5JYGbN248bQPC47\/8diL9K+jbxhP\/agICv\/NZ62orcv6z91qNLtyFXUsy4WA0ZuSftQ\/\/zzLM1iK6vQ8fHUxEMh5AWzBVo5AvGUVTXn3u06hO\/7ph1bTPmxVHQB31a+cFWxZVotzK3Hbchb8k8os6liWOwGjOyHuULpyNY6iOr1c\/JLKwSAr5wWwBVspAuqSwv0\/epPuf3o8EBmmC2lVndQalasuPIOWzD9t6lyVs95D9NJLL9HZZ5+N07k9O7CoY1nuBAwiMJ5bbk7NYZCVcwzYgm0pBFQk5QfjP6GnDr\/mLFLUtM+nltTRqe96V+oaFbTbUryWnBcCRo5tsKW6p6eHBgcHg1LSDpgTrMo000V1ern4IQKTDWkMBHLci8SW16W8ceJtuucbLwbAbCMpnEdFUz59+Tw6472\/NBVNcZn2KRJbuRboZrmoY1mmERh9FxK7Jcv1LlHNoqhOd\/sK+M2FzsovT90a2IKtmurhv\/\/3d1+m4SOvW69JCU\/5fPLiOjqP53mEtiaj3cq126KOZZkIGH3XUV6iLRAwcl+eKMvorOR4g+3sYMsRFH7e\/sUkbXn4YElRFM7MB70pkaIiKC6RFFf6aLeu5NLzQcCkMzJOceDAAbrxxhvp1ltvnbqjKC1z3F1IaflK+byoTi+Fia+86Kx8kZxpB2yLw5ZFyvuq3k1\/+a0ROvTKcadpHhUx4b9\/6\/wa+rXG99OVF9RMQSqnSEnyDNqtXLst6liWaQTGx2WOci4v7umFksxMbaOzMiVlnw5s7ZmZ5vDJVk3z\/PDl4\/TA0+PEf9ucOKvXWYkQjqJcsfB0aj7n5FH5lXQKrU+2pv6cLekgYDx6OnxwnanpRYsWUfg2atO8LumK6nQXFr7zoLPyTfQde2CbLVt1foma4nnmyOv07EtvOO3mCYsUFijn1lbT7y2urziBkuYVtNs0Qu6fF3UsyyQC4+6G8uYsqtPLSzG6NHRWcl4AW3m2P6s6feqskn\/4zgT903dfpslJt508qrZqRw\/\/\/ftL6umcM6unXiQv0zxyZInQbuXoFnUsg4BJaDNFdbrc18TcMjorc1a2KcHWllh0eo6gNM6torsePUx8cJvr9E5YoCw4o5o+ufisqbNR+PPZIFDSvIJ2m0bI\/fOijmUQMBAw7t+KEnKisyoBXkpWsE0GpNaecKq\/5+jJv74cZHA5B0UXJ\/xvnuLhyMnaqxdMHYUPgWLW1tFuzTi5pIKAcaFW4XmK6vQ8uAWdlZwXZjNb\/fyTiTd+Sl9\/9pVASPiInrDHln7ofdQ0d5JWLJlPP3qT6PLzTi6WxVM6gdncbkunl2yhqGMZIjCIwEh\/dyLto7OSw15EtuGFsd976Q36zsgbdPAV9507UdETnsppv4QXyJ4SeapsEdnKtUQ7y2Brx8smNQSMDa2CpC2q0\/PgHnRWcl6oRLbq\/h2mctb7fzlYd+JDnKjpG57aYXFy9YVn0Jnv++UAvkv0pBLZyrU0v5bB1i9P3VpRx7LcRGD27NlD69evD5jzlQKHDh2ioaEh2rhxI1VXv7MaP+zi8D1Kq1evJj5fJu5RjuTPeVt2f38\/1dS8c6jTbHC63NfE3DI6K3NWtinzxlaJExYQ\/+\/I6\/R\/fnDUy6JYJU7478vOnUuL559G59e9d9qCWN+LY\/PG1rZt5Dk92Mp5BwJGji3xnUijo6PU1dVFN9xwQyBAWFzwBY8NDQ2JgoTz8sN51PkybW1t1NraOqPG6qbrzZs3BycAs2hKEklFdbqgK41No7MyRmWdsJxs9TUnL\/34BH3juaNe1pzo4oRFyEXz3k98YaCaStI\/twZUQoZysi2hmhWZFWzl3FbUsSzzCIwSHSxAFi5cSJ2dnYEYYYGhrg9IipKEXa4LmvBnLFgOHjyYKIgQgZH7EumW0VnJcfbBVl9zwgLiy0MjtO\/Qa0GlS9mto95aRUY4cnLFwhpq+EBVSTcZy9GcbtkH23LVtdLKAVs5j0HACLH1KWB0WyyA9EdNNS1btiwyOhP1ekV1upArrcyis7LCZZU4iW1YmDx1+DV62NNOHV2cqDUnH206nS798NypyInvKR0rMB4So916gBhjAmzl2BZ1LMs8AsMuU1M5+hSSisbETQdFRV62bdtGcbdbKwGzfPlyuu+++2h4eNh4DcyaNWto6dKlU0XW19cT\/8HjToA7q7GxMaqrq0tc4+RewuzKOfra29Rw2qnEf+9\/6Rj90olj9NjoqfTUyHFvURO2z8\/Z7z+VFta9h37l7PfR+XXvCX42v+bkOrVKFyhprQbtNo2Q++dg684unJP7Vv6jnpGREVq3bl2wvjT8y72\/UstvKRcChl9bX1yrMGzYsME4WqLyqPU04cW\/SsAcPnx4auFuXFplK6pO\/FlHRwetXLmy\/N4qUIknTpygiYkJqq2tpaqqqgK9md9X0YXJS6+\/HYiH70\/8lP7lhTcDscI\/479LfXRxckHtL9MVHz4pTLg8ftTnpZZT6fnRbuU8CLb+2O7YsYMGBgZmGISA8cdYxBIv1OVITl9fHzU1NU2VETWFFJc2LGA2bdpEjY2NU7YQgSnddeyP8fHxIJI1Z86c0g1WmAU1lXNyh87xIIJx9Cc\/o\/5vHaGDr7zlTZjotxTzv3\/7V2vovb\/87oAW\/x\/CxK7hzPZ2a0fLLjXY2vFKSh2OwOzdu5e2bt2KCIw\/xDKWkhb+csRlwYIFU1EdFjB33HEHbdmyJXIrdVHnDWXI21kt6nz3dGHyViASfvbzX9Dd33iRDvDRrZ4WwSoBwn\/zQliezmn5d7XBWpOz3kNBdGvJ+R+cleLQriXapS5qu7WjIJMabGW4stWijmWZTyGphbe8JiXpiTvfRd91pKIscVuvw+ImacdSkZ0u9zUxt1ypnZV+pskvJifpnm+8SN8f+4mIMOGFsL\/W+H7644+e3D6TTg\/1AAAgAElEQVSsREvaOpNKZWveerJLCbZy7MFWji0EjBzbYBHv7t27px0qp5\/psmLFitgzYcIH2emLeNVn7e3tUwuX9HUtcQt+1asW1emCrjQ2nbfOSj8Jdt7pc+jex16kZ196Q0yYfLi2mm74zfk0\/tpPjYWJKdy8sTWtdyWkA1s5L4GtHNuijmW5icCos190F+oRk+eeey448O6BBx6Q83LIclGdXjaACQWVq7OaLkyq6N5vvEjPCkZMFpxRTZ++vJFef+vn3oWJqd\/Kxda0PkVKB7Zy3gRbObZFHcsgYBLaTFGdLvc1Mbfso7N6\/MCxoMB5p1f9W8REbirnQ2dU0x9c2kBv\/ewXmQkTU7o+2JqWNdvSga2cx8FWjm1Rx7LMBQy7LG0Kia8FUGfF8Erqcj1FdXq5+CWVk3bYmloM+1dPjNCTB0+eAPviq7xj5+RakFIefWcOR0xWNjfQibffESZsO22dSSnlS+fFQCBHGGzBVo6AnOWijmW5EDDstqgzV9SedRYvd911F23fvn3a1mg5d5+0XFSnS3OLs68vgN33wiv098+8RKNvTAZbhn0LExYgq5Y1VkTExLc\/MMj6JvqOPbAFWzkCcpaLOpblRsDIuc7dclGd7k4kPSdP6zTOraK7v3GYnht\/00vUJHyWybW\/cTZNTr4TJankaEk6UfsUGGTtmZnmAFtTUvbpwNaemWmOoo5lEDAJLaCoTjdt9OF0+s3D3zzwKg09f6xkgaKLk2suOosuqHtvRVzq58qwHPkwEMhRBluwlSMgZ7moY1kuBAwfKLdq1SoaHR2d4cFFixZN214t5+KZlovqdBOGHEmZpEna\/eRYML3jcgOxfuPwkvmn0VUXnhHYuvy8uYSBwMQLbmnA1o2bSS6wNaHklgZs3biZ5CrqWJa5gNGP+FfnvfC5Leoyx6jt1SYO85GmqE7X2aioCh9j\/4UHD1gJFSVQfuv8Gvr4r5wRnAarnrRpHXRWPlpotA2wBVs5AnKW0W7l2BZ1LMtcwKgD65RQ0Y\/7Z+i7du2i8MWMcm6ebrmITlcLaR8\/8GogVkwiKyxG+Lj6X\/\/QB6jprPcEUzxpAiXNR+is0gi5fw627uzScoJtGiH3z8HWnV1aziKOZfzOuRMwvOPo4MGDxIIm6V6jNIf5+LwoTleipfehFxIFC4sSPr6eIyqfXFwXICxVqMT5AZ2VjxaKCIwcRbAF23ITkCuvKGNZmFDmAoYrpN9JpIuWhx9+mIaGhhCBcWzXvI4lSbSwOLnyghr63YvqvERVbKoJAWNDyy4t2NrxskkNtja07NKCrR0vm9QQMDa0LNPq62D40DoWNNu2bSO+lLHcZ7\/oVa9Ep3O0hUXLrifHZnhBTQV1Lz9HNLpi4n50ViaU3NKArRs3k1xga0LJLQ3YunEzyVWJY5nJe+UiAmNS0SzSVJLTWbhcv+vZGVNEumiRmg5y8Q06KxdqZnnA1oyTSyqwdaFmlgdszTi5pKqksczm\/TIXMOFFvOEICEdj+vv7qaamxua9vKStBKcnCZcHr7tYbA1LqYDRWZVKMD4\/2IKtHAE5y2i3cmwrYSxzeXsImARqeXY6C5dHvn+Ubv6bH0x7A46y5Fm4qMqis3L5uprlAVszTi6pwNaFmlkesDXj5JIqz2OZy\/uoPJkJGN5ttH79+tS6r169OtiRlMWTV6ezeFlx79PT7g+qFOECASPfkjEQyDEGW7CVIyBnOa9jWalvnJmAURVPmkIq9eVKzZ83p6vt0Cxe1MPC5e62C4PTbSvpwUAg5y2wBVs5AnKW0W7l2OZtLPP1ppkLGF8vImEnT05n8bLryZeo96GDU696d9sFxBcbVuKDzkrOa2ALtnIE5Cyj3cqxzdNY5vMtIWASaObF6eEpI4668Fbo9kvqfbaFstpCZyWHG2zBVo6AnGW0Wzm2eRnLfL9hJgJGTRsNDw+nvs9sv8wxSrxUwiLdNMeis0oj5P452LqzS8sJtmmE3D8HW3d2aTkhYNIIFfDzrJ0eJV6e+fylhSCNzkrOjWALtnIE5Cyj3cqxzXosk3qzTCIwPl9GneI7ODgYmDXdtbR\/\/37q6uqivr4+ampqiqxSlk4vsnhh2OisfH4LptsCW7CVIyBnGe1Wjm2WY5ncW+XgMkf1clHbqjds2EB8tUDSo9+jpKam2traEvMp0bNv377Eqwqycnr4cDpe81KUyIvyJTorua812IKtHAE5y2i3cmyzGsvk3uik5VxEYFi87N69e9qJu6ZiJAxIFzRx8NSFkfx5HiMwd379EN3+jz8Mql9E8YIIjOzXGgOBHF+wBVs5AnKWIWCE2Pq8SsDkTBlOc9ttt1F7e3twaaSJgFmzZg0tXbp0ikB9fT3xH4nnq09P0JqvHghMN5x2Kt3\/R7+W2+sASnl\/HgjGxsaorq6OqqurSzGFvCECYCvXJMAWbOUI+LPMfSv\/Uc\/IyAitW7eOdu7cSc3Nzf4KythS5hEYXwJG3WDd0tJCGzdujB0UOdrDz+LFi43XwIR91NHRQStXrvTuutHX3qaWgSNT4uW\/XXUmLWmc472cPBg8ceIETUxMUG1tLVVVVeWhSoWpA9jKuRJswVaOgD\/LO3bsoIGBgRkGIWD8MZ6y5HsKaXR0NFLE8MJdduott9xCR44cMRYwmzZtosbGxqn6SkRgeN3Lmq\/up72H3gjK2fqp8+hTF9cK0M6HSV6HND4+HkSy5swppkjLijTYypEHW7CVI+DPcjgCs3fvXtq6dSsiMP4QT7fkuog3XJ+k3UUcpbniiiuCEFrediH1PvTC1Cm7l507lwavv1gKdS7sYi2BnBvAFmzlCMhZRruVY4s1MHJsvVpWC3T7+\/uppqZmynbS4XlxYbVyOr3mpkeDulbapYyuzkNn5UouPR\/YpjNyTQG2ruTS84FtOiPXFOUcy1zr6JIvN2tg0rY+x72cvutIbY9uaGhIvcE6TxGYlnuepm89fyx4Rd4uzSKm6A86KzkPgy3YyhGQs4x2K8cWAkaOLYWnj2wWGoUPstMX8arPeMdReOV1XgTMrifH6PpdzwZ0Z8PUkWpG6KzkvlBgC7ZyBOQso93KsYWAkWM7zbLaTcQ\/5EjK9u3bY0\/Kla5SOZyuoi+zZeoIAka61eKUY0nCGGTl6IKtHNtyjGVytY+3nPkUUtJLs5hh8OH1LOUCJe103nl00e1PBK\/z2Y\/Np9tazi3Xq2VeDjorOReALdjKEZCzjHYrx1Z6LJOrebLl3AkYPQKT5U3UjE3S6fpdR7Mt+sJs0VnJfeXBFmzlCMhZRruVYys5lsnVOt1yLgRMnqaNdGSSTn\/8wDFace\/TQXF3fup8+oNLG9K9VaAU6KzknAm2YCtHQM4y2q0cW8mxTK7W6ZYzFzAmx\/+nv4ZMCimn65c1zsboCyIwMu1VWcVAIMcXbMFWjoCcZamxTK7GZpYzFzBm1cwmlZTT9Z1HfZ9ook9fPi+bF8ywVAwEcvDBFmzlCMhZRruVYys1lsnV2MwyBEwCJymnz9adRzpqdFZmX1CXVGDrQs0sD9iacXJJBbYu1MzySI1lZqXLpYKAKbOA0de+XH7eXHrwumJfGRCHF52V3JcabMFWjoCcZbRbObYQMHJsc2tZwumz8dTdKAejs5Jr9mALtnIE5Cyj3cqxlRjL5GprbjnzCEzSIt64e43MX6+0lL6drp\/7wot3+dqA2fqgs5LzPNiCrRwBOctot3JsfY9lcjW1swwBU8YpJF3AzJY7jzCFZPeF9JEaA4EPitE2wBZs5QjIWYaA8cw2fP9RnPnVq1enXszouWpT5nw7XV+8O5ujLwwYA4FUqwVbObJgC7aSBORs+x7L5GpqZznXERi7V\/Gf2qfT9ejLbLq0EREY\/+0yzSLEYRoh98\/B1p1dWk6wTSPk\/rnPscy9Fv5zZi5g\/L+SP4s+nd770AvU+9DBoHK884h3IM3mB52VnPfBFmzlCMhZRruVY+tzLJOrpb1lCJgEZj6djumj6aDRWdl\/WU1zgK0pKft0YGvPzDQH2JqSsk\/ncyyzL10uRyYCRt95tHDhQurs7KTh4eHIt8zyQkdfTtenj9ovqad72i+U82iFWEZnJecosAVbOQJyltFu5dj6GsvkauhmORMB41bV8ufy5XR9+mi27z5SXkRnJdeewRZs5QjIWUa7lWPrayyTq6GbZQiYMkwhYfpoJmR0Vm5fWJNcYGtCyS0N2LpxM8kFtiaU3NJAwLhxS82lppOKOoWE6aPoJoDOKvWr4ZwAbJ3RpWYE21REzgnA1hldakYImFREfhOwsLn55pvpc5\/7HDU1Nfk1bmjNh9MfP\/Aqrbj3maBE7D56Bzw6K8NG6JAMbB2gGWYBW0NQDsnA1gGaYRYfY5lhUWVNluspJIa+a9cu2rhxI1VXV0eCOX78OPX09NDg4GDwedLBd+FoT0tLS6JtH07H9BEiMGX9RuOQQFHcGGTl8IKtHFsfY5lc7dwt517A9Pb2Un9\/P9XU1ES+JX\/OT3d3NymB0tbWRq2trdPSK6GzbNmy4DP1\/4aGhtiTfkt1Og6vi2+Y6Kzcv7RpOcE2jZD752Drzi4tJ9imEXL\/vNSxzL1k2Zy5FjAsTkZHRxOjJGE8uqBJQ8fXGQwNDcXaL9XpuoDhrdO8hRrPSQLorORaAtiCrRwBOctot3JsSx3L5GpWmuXMBUzSIl6Ojmzfvt14DUzSzdZRmEwFzJo1a2jp0qVTJurr64n\/pD2f6n+WvvX8sSDZt9deTHwDNZ53BMzY2BjV1dXFTg+ClRsBHgjA1o1dWi6wTSPk\/jnYurML5+TvP\/9Rz8jICK1bt4527txJzc3N\/grK2FLmAobfP256R033mDDiyMu2bdsobV2LspU03aTSKNUaLr+jo4NWrlyZWq0ld528OqDhtFNpsGNeavrZlODEiRM0MTFBtbW1VFVVNZteXfxdwVYOMdiCrRwBf5Z37NhBAwMDMwxCwPhjPGUpaqrIRGBEVcVk2kkJJs6ftEBYCZhNmzZRY2PjVHEmERiePmre\/HSQ5+Yr59HNV35QgFzlmmQfjI+PB5GsOXMQmfLpSbD1SXO6LbAFWzkC\/iyHIzB79+6lrVu3IgLjD\/FJS0nTPia7kML12b9\/P3V1dVFfX1\/k1JOpeGG7pcwb4vLG5JaC+W7f36R37IEt2MoRkLOMdivHtpSxTK5WpVvOfAopTcCk7UIKI2BHxeUx2Xmk2yvF6dg+DQFT+tfTzQIGAjduJrnA1oSSWxqwdeNmkquUsczEflZpMhcw4fUvOoi0RbacVt91lCZQTKaXfAgYbJ9Ob87orNIZuaYAW1dy6fnANp2RawqwdSWXng8CJp2RcwqGu3bt2mk7jngqaNWqVbR58+bEVdPhg+z0Rbzqs\/b2doq79TrptmtXpz9+4BituPfk+hdsn45uFuisnL8uqRnBNhWRcwKwdUaXmhFsUxE5J3Ady5wLLFPGzCMw6j2jdvxkvWLa1elY\/5LeetFZpTNyTQG2ruTS84FtOiPXFGDrSi49n+tYlm452xS5ETDZYogu3dXpWP+S7k10VumMXFOArSu59Hxgm87INQXYupJLz+c6lqVbzjZF5gJGn+bJ2wE7rk6vuenRwKuXnTuXBq+\/OFsP57R0dFZyjgFbsJUjIGcZ7VaOretYJlcjP5YzFzC2p+f6eW0zKy5O1xfwbu\/4Vbpm0Vlmhc2yVOis5BwOtmArR0DOMtqtHFuXsUyuNv4sZy5g+FVMdhv5e2VzSy5O1xfwPnjdxXT5eXPNC5xFKdFZyTkbbMFWjoCcZbRbObYuY5lcbfxZzlzAJN2FxK+ZtEvIH4ZoSy5OV+tf2OIzn78U9x\/FOAmdlVzrBVuwlSMgZxntVo6ty1gmVxt\/ljMXMP5exb8lF6djAa+ZH9BZmXFySQW2LtTM8oCtGSeXVGDrQs0sj8tYZmY521QQMAn8bZ2OA+zMGzM6K3NWtinB1paYeXqwNWdlmxJsbYmZp7cdy8wtZ5syEwGjL9yNO2BOYamkKSRdwHQvX0Ddy8\/J1rs5Lh2dlZxzwBZs5QjIWUa7lWMLASPHNreWbZ2uH2CH9S\/JbkVnJdfswRZs5QjIWUa7lWNrO5bJ1cSv5UwiMOFXCN+HlHQ\/kt\/XT7Zm63R9Ae\/ROz9WzqpWXFnorORcBrZgK0dAzjLarRxb27FMriZ+LedCwERdsqimmdra2qi1tdXvWxtas3X6Rbc\/QTyNNL9mTrADCU88AXRWcq0DbMFWjoCcZbRbOba2Y5lcTfxazlzAJB1kx9B37dpFGzdupOrqar9vbmDNxulYwGsAVEuCzsqOl01qsLWhZZcWbO142aQGWxtadmltxjI7y9mmzr2A4ehMf38\/1dTUlJ2UjdNxA7Wde9BZ2fGySQ22NrTs0oKtHS+b1GBrQ8surc1YZmc529SZC5ik9S5Zn9Br43TcQG3XkNFZ2fGySQ22NrTs0oKtHS+b1GBrQ8surc1YZmc529SZCxh+fYa7du1a2r59OzU1NQVE9u\/fT6tWraLNmzdTVpc82jgdC3jtGjI6KzteNqnB1oaWXVqwteNlkxpsbWjZpbUZy+wsZ5s6FwJGiZhrr712Go2dO3dmJl70OpnUAyfw2jVkdFZ2vGxSg60NLbu0YGvHyyY12NrQsksLAWPHqxCpbZxec9OjwTtfdu5cGrz+4kK8v+RLoLOSowu2YCtHQM4y2q0cW5uxTK4W\/i3nJgLj\/9VKt2jqdCzgtWeNzsqemWkOsDUlZZ8ObO2ZmeYAW1NS9ulMxzJ7y9nmgIBJ4G\/qdF3APHjdxXT5eXOz9WoFlI7OSs5JYAu2cgTkLKPdyrE1HcvkaiBjeVYJGLXjaXBwMKC5evVq6u7ujiVr6vS\/emKUbvrqDwI7EDBmDRWdlRknl1Rg60LNLA\/YmnFySQW2LtTM8piOZWbW8pNqVgkYPlOGHxYtJif9mjodC3jtGzQ6K3tmpjnA1pSUfTqwtWdmmgNsTUnZpzMdy+wtZ5tjVgmYMGpd0ES5wdTp6goBLOA1b8zorMxZ2aYEW1ti5unB1pyVbUqwtSVmnt50LDO3mI+UuRAw6syX0dHRGVQWLVokchJv0hUGqhImTtevEPit82vob1Yvyodnc14LdFZyDgJbsJUjIGcZ7VaOrclYJle6nOXMBYxal9LQ0JC4HsUnAo68bNu2jVpaWhLvWVJOX7NmDS1dunSqCvX19cR\/+Pn2wTfoE3\/xneDfN185j26+8oM+q1pYW9xZjY2NUV1dXSb3XBUWLBGBrZx3wRZs5Qj4s8x9K\/9Rz8jICK1bt45MzjTzVwt5S5kLGJNIiBSGqFuw9bKUgAmX39HRQStXrgx+vG\/kLfqj+082lL\/4RD0taZwjVd1C2T1x4gRNTExQbW0tVVVVFerdsn4ZsJXzANiCrRwBf5Z37NhBAwMDMwxCwPhjHFhSEZj29vayn7rLU1ddXV3U19c3dYVBlIDZtGkTNTY2Tn2kR2C2PPIibXnkSPDZt9deTPNrIGBMmgj7fXx8PIhkzZkDZibMTNOArSkp+3Rga8\/MNAfYmpJKTxeOwOzdu5e2bt2KCEw6OvsUHOnI4tbptHJN5g2xA8ne35wD891u3Exyga0JJbc0YOvGzSQX2JpQcktjMpa5Wc42V26mkIaHhyNJ+FzEq+86Mll7Y+J0tQOJIy\/PfP7SbL1ZQaWjs5JzFtiCrRwBOctot3JsTcYyudLlLGcuYORebabl8EF2pot44+YN9R1I2EJt50l0Vna8bFKDrQ0tu7Rga8fLJjXY2tCySwsBY8erEKnTnK5fIdC9fAF1Lz+nEO9djpdAZyVHGWzBVo6AnGW0Wzm2aWOZXMmylnMTgdmzZw+tX78+eFuOeBw6dIiGhoYStznLoiFKc7ouYHj6CAt4zT2CzsqclW1KsLUlZp4ebM1Z2aYEW1ti5unTxjJzS\/lKmQsBo7Yz846gG264ITgPhte+9PT0UDnPhwm7Js3pvQ+9QL0PHQyy4Q4ku4aNzsqOl01qsLWhZZcWbO142aQGWxtadmnTxjI7a\/lJnbmA0c+BWbhwIXV2dgYCprm5OYiAZLE7Sbknzemf3DZMj\/7gaJD86J0fy49XK6Am6KzknAS2YCtHQM4y2q0c27SxTK5kWcsQMAl805yOLdTujROdlTu7tJxgm0bI\/XOwdWeXlhNs0wi5f542lrlbzjZn5gKGX5\/Xv\/B6F30KSUVj2traqLW1NRNKaU6vuenRoF7YgWTvHnRW9sxMc4CtKSn7dGBrz8w0B9iakrJPlzaW2VvMR45cCBhGEXVs\/4YNGzITL3qd4rZRKwHTfkk93dN+YT48WiG1QGcl5yiwBVs5AnKW0W7l2ELAyLHNreUkp2MLdWluQ2dVGr+k3GALtnIE5Cyj3cqxhYCRY5tby6YCBjuQ7F2IzsqemWkOsDUlZZ8ObO2ZmeYAW1NS9ukgYOyZWeXQz4FRGbO+OTPJ6dhCbeXeGYnRWZXGDxEYOX5gC7bZEJArFQJGjm2wiHf37t3U399PNTU1QUlqe3VeF\/Fev+tZ2vXk2Mm6Ygu1deuAgLFGZpwBbI1RWScEW2tkxhnA1hiVdUIIGGtkZhn0c2D47Bf9yfM5MNhCbebfuFTorErjhyiBHD+wBdtsCMiVCgEjxLZSBYy6hRpbqN0aBgSMGzeTXGBrQsktDdi6cTPJBbYmlNzSQMC4cTPKxXDXrl1L27dvp6ampiBPnqeQcAu1kVsTE6GzKp0holtyDMEWbMtPQK5ECBghtkqoDA8Pp5bA9yM98MADqel8JYhzui5gcAu1G20IGDduJrnA1oSSWxqwdeNmkgtsTSi5pYGAceNW0bninK6fAcMH2PFBdnjsCKCzsuNlkxpsbWjZpQVbO142qcHWhpZdWggYO16FSG0iYHAGjJur0Vm5cTPJBbYmlNzSgK0bN5NcYGtCyS0NBIwbN+NcUefA5PUqAf0MmGc+fynNr5lj\/J5IeJIAOiu5lgC2YCtHQM4y2q0cWwgYObYVdw6M2kLNSHAGjFvDQGflxs0kF9iaUHJLA7Zu3Exyga0JJbc0EDBu3FJzVeI2apwBk+rW1ATorFIROScAW2d0qRnBNhWRcwKwdUaXmhECJhWRW4JSBUx4F1NLSwtt3LiRqqurIyukT1WlpY1zurqFGmfAuPmcc6GzcmeXlhNs0wi5fw627uzScoJtGiH3zyFg3Nml5nS9SuD48ePU09NDy5Yto9bWVlL\/b2hooO7u7hnl6if7ssDhvHFpOXOU0\/Ut1Lz7iHch4bEngM7KnplpDrA1JWWfDmztmZnmAFtTUvbpIGDsmVnl8LWIl+0MDQ1FRmHCnyWlNREwOAPGysXTEqOzcmeXlhNs0wi5fw627uzScoJtGiH3zyFg3NmVNWeSKImKwKjoTVQl0yIwOAPG3bXorNzZpeUE2zRC7p+DrTu7tJxgm0bI\/XMIGHd2Zctpcv3A\/v37adWqVTQ6Oko7d+6k8AWSemWV09esWUNLly4NPvrr775Nf\/3dnwX\/\/sofXkCXn3d62d6vSAVxZzU2NkZ1dXWx65WK9L7lfBewlaMNtmArR8CfZe5b+Y96RkZGaN26daljnr8alMfSKZOTk5PlKUq2FLX+hUuJW8QbXmvT29sbVCpqvQz\/XAkYveZvXbCC3rrgmuBHgx3zqOG0U2VfrKDWT5w4QRMTE1RbW0tVVVUFfctsXgts5biDLdjKEfBneceOHTQwMDDDYNov7f5qUB5LhRAwJuIlvOCX8XI0pquri\/r6+qYukdSxKwGzadMmamxsDD667sEJOvjW+4J\/j95xaXm8VMBS2B\/j4+NUX19Pc+bgIECfLgZbnzSn2wJbsJUj4M9yOAKzd+9e2rp1KyIw\/hD7sZS280iVUoqA0VUrzoDx4zfMd\/vhGGUFbMFWjoCcZbRbObZYAyPHtiTLPA3E61mSzn5RBURNISXljXL6Rbc\/QbyVGmfAlOQ2nANTGr7E3BgI5OCCLdjKEZCzDAEjx9bZcvgQO2Vo0aJF1N\/fHywO5bNe2tvbpxbrsuDZtm1bkNTlIDscYufsrmkZMRD44YgIjBxHsAXb8hKQKw0CRo5tbi2Hna4fYoczYEpzGwRMafyScoMt2MoRkLOMdivHFgJGjm1uLYed\/viBY7Ti3qeD+j543cV0+Xlzc1v3vFcMnZWch8AWbOUIyFlGu5VjCwEjxza3liFg5FyDzgps5QjIWUa7BVs5AnKWIWDk2ObWctjpvQ+9QL0PHUQExoPHMBB4gBhjAmzBVo6AnGW0Wzm2EDBybHNrOUnAPPP5S2l+Dc4vcXUeOitXcun5wDadkWsKsHUll54PbNMZuaaAgHElV8H5wk5XZ8DwKx2982MV\/GbZVx2dlZwPwBZs5QjIWUa7lWMLASPHNreW4wQMR144AoPHnQA6K3d2aTnBNo2Q++dg684uLSfYphFy\/xwCxp1dxeYMOx2H2PlzJTorfyzDlsAWbOUIyFlGu5VjCwEjxza3lsNOxyF2\/lyFzsofSwgYOZZgC7blIyBXEgSMHNvcWtad3rDwIuIIDD\/\/5cr5dOvvnJvbeldCxSBg5LwEtmArR0DOMtqtHFsIGDm2ubWsO\/3tMy+YOsQOp\/CW7jJ0VqUzjLMAtmArR0DOMtqtHFsIGDm2ubUcJ2Duab+Q2i+pz229K6Fi6KzkvAS2YCtHQM4y2q0cWwgYOba5taw7\/YV3L6Drdz0b1BXXCJTuMnRWpTNEBEaOIdiCbfkJyJUIASPHNreWIWDkXAMBA7ZyBOQso92CrRwBOcsQMHJsc2tZd\/r\/fOEDtOvJsaCuOIW3dJdhICidIaIEcgzBFmzLT0CuRAgYOba5taw7\/Uv7quhbzx8Lrg\/AIXaluwwCpnSGGGTlGIIt2JafgFyJEDBybHNrGQJGzjUQMGArR0DOMtot2MoRkLMMASPHNreWdaf\/8dcn6fDRt+iyc+fS4Nz+fasAABCJSURBVPUX57bOlVIxDARyngJbsJUjIGcZ7VaOLQSMHNvcWtad\/ttfOR7UEwLGj7vQWfnhGGUFbMFWjoCcZbRbObYQMHJsc2tZOb33ni\/T6ocng3ry+S98Dgye0gigsyqNX1JusAVbOQJyltFu5dhCwMixza3lKAGDU3j9uAudlR+OiMDIcQRbsC0vAbnSIGDk2JZk+ejRo9TZ2UnDw8OBnZaWFtq4cSNVV1dH2lWO5A8XLVpE\/f39VFNTk5j2+i\/eS7wLiR+cwluSu6YyQ8D44YhBVo4j2IJteQnIlQYBI8fW2fLx48epp6eHli1bRq2traT+39DQQN3d3TPs7t+\/n1atWkWbN2+m5uZm2rNnDw0NDcUKHuV0XcDgDBhnd03LCAHjhyMGWTmOYAu25SUgVxoEjBxbr5aTRAl\/dvDgwUhxE1UJ5fRFHXfQYz+uC5LgGgE\/7oKA8cMRg6wcR7AF2\/ISkCsNAkaOrVfLcQImHK0xKVQ5\/cyrrqMD71sSZNl29Sl0yYULqL4elzmaMIxLwwJmbGyM6urqYqf7SrE\/m\/OCrZz3wRZs5Qj4s8x9K\/9Rz8jICK1bt4527twZzD4U5TllcnLy5PaaAjxqPUxbW1swpaQ\/SsAsX76c7rvvvmDNjOkamDcX\/yH9dP5lgbm5f9dJHR0dtHLlygIQy+4VTpw4QRMTE1RbW0tVVSfXF+HxQwBs\/XCMsgK2YCtHwJ\/lHTt20MDAwAyDEDD+GHu1pAQKG41axKs+P3z48NTC3d7eXhodHU1dA\/PG5V309pnnB9cI\/PlVpwTRF0RgSnMf+2N8fDzgOGfOnNKMIfcMsQ62Mo0C7VaGK1sFW39swxGYvXv30tatWxGB8YfYn6U08aK+HPqCX\/4ZL+rt6uqivr4+ampqmlEhNYWkCxjcg+THb1gD44djlBWwBVs5AnKW0W7l2GINjBzbkiyn7TzSjXPEZcGCBVPTSyxg7rjjDtqyZUvkVmrl9Nc+3ku\/eM+ZOIW3JE9Nz4zOyiPMkCmwBVs5AnKW0W7l2ELAyLEtyXLaNJBunJ3I6dXZL\/xvfqK2XPPPldOP\/cf+IB2uESjJVdMyo7PyxzJsCWzBVo6AnGW0Wzm2EDBybJ0thw+xU4bU4lw+zI6njdrb26dWXusH2Zkcetf26RuJIzCB0Fm+gLqXn+NcX2R8hwA6K7nWALZgK0dAzjLarRxbCBg5trm1zE6HgJFxDzorGa5sFWzBVo6AnGW0Wzm2EDBybHNrmZ3++zfeRryIlx9cI+DPVeis\/LEMWwJbsJUjIGcZ7VaOLQSMHNvcWman\/17PPcTnwPCDU3j9uQqdlT+WEDByLMEWbMtHQK4kCBg5trm1HF7ECwHjz1UQMP5YYpCVYwm2YFs+AnIlQcDIsc2t5bCAwUWO\/lwFAeOPJQZZOZZgC7blIyBXEgSMHNvcWmant3d00o9\/5+6gjkfv\/Fhu61ppFYOAkfMY2IKtHAE5y2i3cmwhYOTY5tayvgaGrxHAKbz+XIXOyh9LRAnkWIIt2JaPgFxJEDBybHNrWZ9CgoDx6yYIGL88dWtgC7ZyBOQso93KsYWAkWObW8u6gMEpvH7dhM7KL08IGDmeYAu25SEgVwoEjBzb3FqGgJFzDQQM2MoRkLOMdgu2cgTkLEPAyLHNrWVdwLRfUh8cZIfHDwEMBH44RlkBW7CVIyBnGe1Wji0EjBzb3FrWBcyf\/v75tLK5Ibd1rbSKobOS8xjYgq0cATnLaLdybCFg5Njm1rIuYHCRo183obPyy1O3BrZgK0dAzjLarRxbCBg5trm1rAsY3IPk103orPzyhICR4wm2YFseAnKlQMDIsc2tZV3A4BoBv26CgPHLE4OsHE+wBdvyEJArBQJGjm1uLUPAyLkGAgZs5QjIWUa7BVs5AnKWIWDk2ObWsnL6f\/jSP1D38nOID7PD44cABgI\/HKOsgC3YyhGQs4x2K8cWAkaObW4tF9XpeQCOzkrOC2ALtnIE5Cyj3cqxLepYdsrk5OSkHLbKtlxUp+fBK+is5LwAtmArR0DOMtqtHNuijmUQMAltpqhOl\/uamFtGZ2XOyjYl2NoSM08PtuasbFOCrS0x8\/RFHcsqXsAcPXqUOjs7aXh4OPBmS0sLbdy4kaqrqxO9u3\/\/furq6qK+vj5qamqKTFtUp5s3e7mUBw8epIGBAero6KAFCxbIFTQLLYOtnNPBFmzlCMhZLupYVtEC5vjx49TT00PLli2j1tZWUv9vaGig7u7u2Nag0u3bt4+2b98OASP3vYm1XNQvVAYoZxQJtnJeAFuwlSMgZ7mo7baiBUyUu\/fs2UNDQ0OJURh2Zm9vb5AdERi5L02S5aJ+obKhOb1UsJXzAtiCrRwBOctFbbezTsDwlNNtt91G7e3tgYgxETBr1qyhpUuXyrWuWWh5ZGSE1q1bR2Dr3\/lg65+psgi2YCtHQM6yarc7d+6k5uZmuYLKbLlQAkath2lrawumlOIiNPzzxYsXp66BOXLkSDDI7t27t8xuQXEgAAIgAAIg4I8A\/xK+adMmmjdvnj+jGVsqjIBR61qYZ9wiXl64ywtHb7nlFmJxkraIl21xOv6DBwRAAARAAAQqlQALlyKJF\/ZDIQSMiXjhl+UpoyuuuCIIoZnsQqrUhop6gwAIgAAIgEDRCVS8gDHdeRTebq07tmjzgkVvtHg\/EAABEAABEKh4AcNRldHRUaOzX3R3IwKDxg8CIAACIAAClUugogVMXFRl0aJF1N\/fHxxmx+fE8I6j8MprCJjKbbSoOQiAAAiAAAhUtICB+0AABEAABEAABGYnAQiY2el3vDUIgAAIgAAIVDQBCJiKdh8qDwIgAAIgAAKzkwAETILfeYHwtm3bghTYqeT2BeG1RqtWrQoWWqddtKnz5vusku6pcqtNsXLZsFVvHr4\/rFhE\/LyNDdfwOjz0E8k+sGGrp0V\/UFrbVt\/7qPWgpVnONjcETAx\/dV8SLwZ+7rnngjNk+N81NTXZeqyCStcHyxUrVky7eDP8GuE7rPj\/u3fvBvMYf9uw1U0w1\/Xr19OGDRtiT6uuoCbmvao2XMNHOGBjQLI7bNgqYciX8vIGDPQH7k1dcR8cHCzcL+IQMDHtQl32yF+goqpX96+EWc5wh86icNeuXUZb3jEYpP8mq58kbcKWB4Wbb76Zjh07RknXbZh5t5ipbNosp73jjjtoy5Yt+MXGoDnYstXbN\/oDA8ARSVQUa8mSJXT48GFSgtDNWv5yQcBE+CQcZkfY3a3h6lEsjlyF\/59kFR1WMnMXtizKL7nkEvra175Gy5YtQwQmArEN13DU0O1bMnty2bCNisAMDQ0Z\/fIze4imvylf4sgPHynS2dkJAZOOrPJTREVcuPNfsGABOn0L94ajAja\/sboeUGhRvYpOastW3QN20003BbexQ8BEu9+GKwuYgwcPBoawVi7962TDlq3pUx+rV68OBl88bgTCgtDNSv5yIQKTEIHRFzxBwNg3XtsOS5XAA8Ndd92FRbwJyG3Y8kDwpS99iTo6OoLL3PhwRwgYPwKG1xOphbvsk7Vr16LdxrRbmzarpj42b94crIGxid7a91TFzwEBU3wfT70hppD8ONsmZAzxYsfchi2nfeyxx4LfYDEdmszZhmt4CglswdbuW1y+1BAw5WOdi5L0iAsW8bq5JDxllLbQFDsNzDnbsNW3p+slICw\/k7cN13B7Rj+R3H5t2EIcmvcFJikhYEwoFSgNtlGX7kybbZMIv9vxtmGrW0aUIJmzDdfwoIBpDn9so6aQMD1n10foqSFg3NlVbE4cZFe665IOrlKLIHlqIy5KgIPB4n1gyhYCxq4d23DVD7LDYWvpnG3YsiC89tprA6Ngm842KQUETGn8kBsEQAAEQAAEQAAEvBHALiRvKGEIBEAABEAABECgXAQgYMpFGuWAAAiAAAiAAAh4IwAB4w0lDIEACIAACIAACJSLAARMuUijHBAAARAAARAAAW8EIGC8oYQhEAABEAABEACBchGAgCkXaZQDAg4Enn\/+eTr99NONbzvm7ZKvvvoqnXvuuQ6lmWVRW94XLVpE\/f39xnWrlAs61fuVa+tuucsz8zJSgUD+CUDA5N9HqOEsJWB7MFo5znooRYSUkrecTYAFBT\/lvDywUtiU0w8oCwTSCEDApBHC5yCQEYE8ChjbOunoKmWQhoDJqMGjWBCwJAABYwkMyUHAJwH9JFe2q6ZlnnvuualTSPnn6kTi8InFaprjjDPOoM7OThoeHg6qp+45UkfjDw4OBj83mfbRy9CnUfjkZL59WT0bNmyg1tbWGTji0ikBs2LFCvriF78Y5AtP0+inr4bLYVY333wzffSjHw3yq3d55ZVXaNWqVTQ6Ohpk+cIXvkAPPvgg9fX1UVNTU\/CzuHeK8mVYwPD\/X3\/99eCP4ph0j1T4Hh8uI+pnlSjufLZ92AKBUglAwJRKEPlBwJFA1L1E+uAZjnbEXXDHxW\/cuDG4aZpFDE99NDc3kxJHbW1tU0Ij6cJMVR9lr7q6Ohh477rrLtq+fXsgBtIiMOH0+p02LLJYaCxZsiSor7K\/e\/fuYC0NC5Gurq5pwkO3p0Ta\/Pnzp\/KH31H9f2JiIqjzvHnzqKenJxBKakoo7d6tKAGzbds2UoItiqveBCBgHL8QyAYClgQgYCyBITkI+CKQNhCmiYXwb\/ZhARN1+3fSZY5RUzzh9El1SrsoMnxBH9c\/bVpJ\/1wJmLAgGxoamhI0bFMXKPz\/O+64g7Zs2TJtsXHSNFGUgOHojhJdqgxOF7WIGQLG1zcEdkAgmQAEDFoICGRIQJ9uCU\/vxImF8DRLS0tLZAQmPJWjv2bU9E9cefqlm0kCJm0RcZRYifpZeFotPE2mIkz8PlFCRLfJUR11IWDYzXHTQFEChvPqi3qThBcETIZfKBQ9qwhAwMwqd+Nl80pAH7T1dTD6b\/lKkITXpagIRDgCw3nDkYOk948TJ0nTWrq9UgUM21JrWZTAiorA2AiYp556itQUVU1NjZH7IWCMMCERCGROAAImcxegAiDwDgFdBKgIA09T8HoRXsuxbNmyaQtndZESFjBJ612imJdjCim8xkUvk8VG0nSQmkLSBUxUtEOfQuIIzNq1a6fW8Ji0NYkppDQxmTaVZlJvpAGB2UYAAma2eRzvmxsCUWtg9CiIvqg1ajGqisioKSR+MV3kKPu8oFdNf0StQ1FAfC3i1SMe+rqYxYsXz1ikGxYwel5VV64fL8iNEjCmi3jZhlrDkrb2qNRFvGqKT+0cU++hL14ON0IImNx8LVGRCiIAAVNBzkJVi0dADW5qC7A+PaRvgeYplauvvnraVmkWLtdccw3deuutUxGGsKhRURm1vZoJqoE1jmbSlmPThcVR261N1sCEy968eXOwzoUX7qr31yMw\/A5hhuFt1OGt5Jwnbgu4inrx30r0RW2j1vNHvZe+\/oj9dNFFF9EzzzwTiKiw0FTvEI5OFa+1441AwC8BCBi\/PGENBEAgYwIsKKJ2HplWy2QNjKkt03SIwJiSQjoQeIcABAxaAwiAQMUSCK\/zUdEW\/dwX25eDgLElhvQgkA0BCJhsuKNUEAABTwTCpxMnnZJrUmT4csX7778\/yCZ1NxIuczTxCtKAwEwCEDBoFSAAAiAAAiAAAhVH4P8DAHCEoEUuvnEAAAAASUVORK5CYII=","height":155,"width":258}}
%---
%[output:2330891c]
%   data: {"dataType":"textualVariable","outputData":{"name":"heat_capacity","value":"  13.199999999999999"}}
%---
%[output:55c0aace]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_switch_HA","value":"   0.007500000000000"}}
%---
%[output:6d4591c7]
%   data: {"dataType":"textualVariable","outputData":{"name":"Rth_mosfet_HA","value":"   0.007500000000000"}}
%---
