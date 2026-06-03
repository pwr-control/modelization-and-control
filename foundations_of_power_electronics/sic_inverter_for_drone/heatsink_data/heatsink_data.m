clc
% %% SK DC 7 1 117 SA
% volume_base = 117*62.5*3.5; %mm^3
% volume_fins = 16.5*3^2*20*8; %mm^3
% volume = volume_base + volume_fins;
% density_al = 2.7e-3; %g/mm^3
% weigth = volume*density_al*1e-3; % kg
% cp_al = 880; % J/K/kg
% heat_capacity = cp_al*weigth; % J/K
% Rth_mosfet_HA = 0.6*6; % K/W

%% ALPHA LT13070
weigth = 0.326/6; % kg
cp_al = 880; % J/K/kg
heat_capacity = cp_al*weigth % J/K
thermal_conducibility = 204; % W/(m K)
Rcond = 0.015/thermal_conducibility/(0.0065*0.03)
Rth_mosfet_HA = 0.25*6+Rcond % K/W

