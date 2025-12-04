clear all
close all
clc

tc = 1e-6;
simlength = 0.2;

% Power semiconductors modelization, IGBT, MOSFET,  and snubber data

% HeatSink settings
heatsink_liquid_2kW;

% DEVICES settings (MOSFET)
% wolfspeed_CAB006M12GM3;
% wolfspeed_CAB760M12HM3;
infineon_FF1000UXTR23T2M1;

mosfet.inv.Vth = Vth;                                  % [V]
mosfet.inv.Rds_on = Rds_on;                            % [V]
mosfet.inv.g_fs = g_fs;                                % [A/V]
mosfet.inv.Vdon_diode = Vdon_diode;                    % [V]
mosfet.inv.Rdon_diode = Rdon_diode;                    % [Ohm]
mosfet.inv.Eon = Eon;                                  % [J] @ Tj = 125°C
mosfet.inv.Eoff = Eoff;                                % [J] @ Tj = 125°C
mosfet.inv.Erec = Eerr;                                % [J] @ Tj = 125°C
mosfet.inv.Voff_sw_losses = Voff_sw_losses;            % [V]
mosfet.inv.Ion_sw_losses = Ion_sw_losses;              % [A]
mosfet.inv.JunctionTermalMass = JunctionTermalMass;    % [J/K]
mosfet.inv.Rtim = Rtim;                                % [K/W]
mosfet.inv.Rth_switch_JC = Rth_mosfet_JC;              % [K/W]
mosfet.inv.Rth_switch_CH = Rth_mosfet_CH;              % [K/W]
mosfet.inv.Rth_switch_JH = Rth_mosfet_JH;              % [K/W]
mosfet.inv.Lstray_module = Lstray_module;              % [H]
mosfet.inv.Ls = Ls;                                    % [H]
mosfet.inv.Ld = Ld;                                    % [H]
mosfet.inv.RLs = RLs;                                  % [Ohm]
mosfet.inv.RLd = RLd;                                  % [Ohm]
mosfet.inv.Irr = Irr;                                  % [A]
mosfet.inv.Ciss = Ciss;                                % [F]
mosfet.inv.Coss = Coss;                                % [F]
mosfet.inv.Crss = Crss;                                % [F]
mosfet.inv.Cgd = Cgd;                                  % [F]
mosfet.inv.Cgs = Cgs;                                  % [F]
mosfet.inv.Cds = Cds;                                  % [F]
mosfet.inv.Rgate_internal = Rgate_internal;            % [Ohm]
mosfet.inv.Csnubber = 2*Eon/Voff_sw_losses^2;          % [F]
mosfet.inv.Rsnubber = 1;                               % [Ohm]
% inv.Csnubber = (mosfet.inv.Irr)^2*Lstray_module/Vdc_bez^2
% inv.Rsnubber = 1/(mosfet.inv.Csnubber*fPWM_INV)/5

m = 0.5;
freq = 20e3;
L = 500e-6;
C = 200e-6;

Rgate_turn_on = 0.1;
Rgate_turn_off = 1;

open_system igbt_device_analysis.slx
