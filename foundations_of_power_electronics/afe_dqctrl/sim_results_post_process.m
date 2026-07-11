clear all
clc
close all
load sim_results_1.mat


Ploss_switch_inv = mean(igbt_ploss_invQ1_sim);
fprintf('Inverter Switch Power Losses: %.2f W\n', Ploss_switch_inv);
Ploss_diode_inv = mean(diode_ploss_invQ1_sim);
fprintf('Inverter Diode Power Losses: %.2f W\n', Ploss_diode_inv);
Ploss_module_inv = 2*Ploss_diode_inv + 2*Ploss_switch_inv;
fprintf('Inverter Module Power Losses: %.2f W\n', Ploss_module_inv);
T_JH_switch_inv = mean(igbt_JH_temp_invQ1_sim);
fprintf('Inverter Switch Junction-HeatSink Temperature: %.2f K\n', T_JH_switch_inv);
T_HA_switch_inv = mean(igbt_HA_temp_invQ1_sim);
fprintf('Inverter Switch HeatSink-Water Temperature: %.2f K\n', T_HA_switch_inv);
T_JH_diode_inv = mean(diode_JH_temp_invQ1_sim);
fprintf('Inverter Diode Junction-HeatSink Temperature: %.2f K\n', T_JH_diode_inv);
T_HA_diode_inv = mean(diode_HA_temp_invQ1_sim);
fprintf('Inverter Diode HeatSink-Water Temperature: %.2f K\n', T_HA_diode_inv);
Tj_max_switch = max(igbt_JH_temp_invQ1_sim) + max(igbt_HA_temp_invQ1_sim) + Tambient;
Tj_max_diode = max(diode_JH_temp_invQ1_sim) + max(diode_HA_temp_invQ1_sim) + Tambient;
fprintf('Maximum IGBT Junction Temperature: %.2f K\n', Tj_max_switch);
fprintf('Maximum DIODE Junction Temperature: %.2f K\n', Tj_max_diode);

DeltaT_JH_switch = max(igbt_JH_temp_invQ1_sim) - min(igbt_JH_temp_invQ1_sim);
fprintf('Junction Temperature ripple: %.2f K\n', DeltaT_JH_switch);






