% clear
close all
clc

load sim_results_FF1200R17_4kHz_370A_RthHA36;

ploss_igbtQ1 = mean(igbt_ploss_invQ1_sim);
ploss_diodeQ1 = mean(diode_ploss_invQ1_sim);
ploss_module = 2*ploss_igbtQ1 + 2*ploss_diodeQ1;
deltaJH_igbtQ1 = mean(igbt_JH_temp_invQ1_sim);
deltaJH_diodeQ1 = mean(diode_JH_temp_invQ1_sim);
deltaHA_igbtQ1 = mean(igbt_HA_temp_invQ1_sim);
deltaHA_diodeQ1 = mean(diode_HA_temp_invQ1_sim);
rippleJH_igbtQ1 = max(igbt_JH_temp_invQ1_sim) - min(igbt_JH_temp_invQ1_sim);
rippleJH_diodeQ1 = max(diode_JH_temp_invQ1_sim) - min(diode_JH_temp_invQ1_sim);
TjMAX_igbt = Tambient + max(igbt_JH_temp_invQ1_sim) + max(igbt_JH_temp_invQ1_sim);
TjMAX_diode = Tambient + max(diode_JH_temp_invQ1_sim) + max(diode_JH_temp_invQ1_sim);
Tj_design_igbt = Tambient + mean(igbt_JH_temp_invQ1_sim) + mean(igbt_JH_temp_invQ1_sim);
Tj_design_diode = Tambient + mean(diode_JH_temp_invQ1_sim) + mean(diode_JH_temp_invQ1_sim);

disp('----------------------------------------------------');
fprintf(igbt.inv.data);
fprintf('\n');
fprintf('Power Losses IGBT: %.2f W\n', ploss_igbtQ1);
fprintf('Power Losses Diode: %.2f W\n', ploss_diodeQ1);
fprintf('Power Losses Module: %.2f W\n', ploss_module);
fprintf('Junction-HeatSink Delta Temperature IGBT: %.2f K\n', deltaJH_igbtQ1);
fprintf('Junction-HeatSink Delta Temperature Diode: %.2f K\n', deltaJH_diodeQ1);
fprintf('HeatSink-Water Delta Temperature IGBT: %.2f K\n', deltaHA_igbtQ1);
fprintf('HeatSink-Water Delta Temperature Diode: %.2f K\n', deltaHA_diodeQ1);
fprintf('IGBT Swing Temperature: %.2f K\n', rippleJH_igbtQ1);
fprintf('Diode Swing Temperature: %.2f K\n', rippleJH_diodeQ1);
fprintf('IGBT Max Tj: %.2f K\n', TjMAX_igbt);
fprintf('Diode Max Tj: %.2f K\n', TjMAX_diode);
fprintf('IGBT design Tj: %.2f K\n', Tj_design_igbt);
fprintf('Diode design Tj: %.2f K\n', Tj_design_diode);
disp('----------------------------------------------------'); 

return

clear

load sim_results_370A_4kHz_FF650R17_newRthHA;

ploss_igbtQ1 = mean(igbt_ploss_invQ1_sim);
ploss_diodeQ1 = mean(diode_ploss_invQ1_sim);
ploss_module = 2*ploss_igbtQ1 + 2*ploss_diodeQ1;
deltaJH_igbtQ1 = mean(igbt_JH_temp_invQ1_sim);
deltaJH_diodeQ1 = mean(diode_JH_temp_invQ1_sim);
deltaHA_igbtQ1 = mean(igbt_HA_temp_invQ1_sim);
deltaHA_diodeQ1 = mean(diode_HA_temp_invQ1_sim);
rippleJH_igbtQ1 = max(igbt_JH_temp_invQ1_sim) - min(igbt_JH_temp_invQ1_sim);
rippleJH_diodeQ1 = max(diode_JH_temp_invQ1_sim) - min(diode_JH_temp_invQ1_sim);
TjMAX_igbt = Tambient + max(igbt_JH_temp_invQ1_sim) + max(igbt_JH_temp_invQ1_sim);
TjMAX_diode = Tambient + max(diode_JH_temp_invQ1_sim) + max(diode_JH_temp_invQ1_sim);
Tj_design_igbt = Tambient + mean(igbt_JH_temp_invQ1_sim) + mean(igbt_JH_temp_invQ1_sim);
Tj_design_diode = Tambient + mean(diode_JH_temp_invQ1_sim) + mean(diode_JH_temp_invQ1_sim);

disp('----------------------------------------------------');
fprintf(igbt.inv.data);
fprintf('\n');
fprintf('Power Losses IGBT: %.2f W\n', ploss_igbtQ1);
fprintf('Power Losses Diode: %.2f W\n', ploss_diodeQ1);
fprintf('Power Losses Module: %.2f W\n', ploss_module);
fprintf('Junction-HeatSink Delta Temperature IGBT: %.2f K\n', deltaJH_igbtQ1);
fprintf('Junction-HeatSink Delta Temperature Diode: %.2f K\n', deltaJH_diodeQ1);
fprintf('HeatSink-Water Delta Temperature IGBT: %.2f K\n', deltaHA_igbtQ1);
fprintf('HeatSink-Water Delta Temperature Diode: %.2f K\n', deltaHA_diodeQ1);
fprintf('IGBT Swing Temperature: %.2f K\n', rippleJH_igbtQ1);
fprintf('Diode Swing Temperature: %.2f K\n', rippleJH_diodeQ1);
fprintf('IGBT Max Tj: %.2f K\n', TjMAX_igbt);
fprintf('Diode Max Tj: %.2f K\n', TjMAX_diode);
fprintf('IGBT design Tj: %.2f K\n', Tj_design_igbt);
fprintf('Diode design Tj: %.2f K\n', Tj_design_diode);
disp('----------------------------------------------------'); 

clear

load sim_results_370A_4kHz_FF1200R17_newRthHA_2;

ploss_igbtQ1 = mean(igbt_ploss_invQ1_sim);
ploss_diodeQ1 = mean(diode_ploss_invQ1_sim);
ploss_module = 2*ploss_igbtQ1 + 2*ploss_diodeQ1;
deltaJH_igbtQ1 = mean(igbt_JH_temp_invQ1_sim);
deltaJH_diodeQ1 = mean(diode_JH_temp_invQ1_sim);
deltaHA_igbtQ1 = mean(igbt_HA_temp_invQ1_sim);
deltaHA_diodeQ1 = mean(diode_HA_temp_invQ1_sim);
rippleJH_igbtQ1 = max(igbt_JH_temp_invQ1_sim) - min(igbt_JH_temp_invQ1_sim);
rippleJH_diodeQ1 = max(diode_JH_temp_invQ1_sim) - min(diode_JH_temp_invQ1_sim);
TjMAX_igbt = Tambient + max(igbt_JH_temp_invQ1_sim) + max(igbt_JH_temp_invQ1_sim);
TjMAX_diode = Tambient + max(diode_JH_temp_invQ1_sim) + max(diode_JH_temp_invQ1_sim);
Tj_design_igbt = Tambient + mean(igbt_JH_temp_invQ1_sim) + mean(igbt_JH_temp_invQ1_sim);
Tj_design_diode = Tambient + mean(diode_JH_temp_invQ1_sim) + mean(diode_JH_temp_invQ1_sim);

disp('----------------------------------------------------');
fprintf(igbt.inv.data);
fprintf('\n');
fprintf('Power Losses IGBT: %.2f W\n', ploss_igbtQ1);
fprintf('Power Losses Diode: %.2f W\n', ploss_diodeQ1);
fprintf('Power Losses Module: %.2f W\n', ploss_module);
fprintf('Junction-HeatSink Delta Temperature IGBT: %.2f K\n', deltaJH_igbtQ1);
fprintf('Junction-HeatSink Delta Temperature Diode: %.2f K\n', deltaJH_diodeQ1);
fprintf('HeatSink-Water Delta Temperature IGBT: %.2f K\n', deltaHA_igbtQ1);
fprintf('HeatSink-Water Delta Temperature Diode: %.2f K\n', deltaHA_diodeQ1);
fprintf('IGBT Swing Temperature: %.2f K\n', rippleJH_igbtQ1);
fprintf('Diode Swing Temperature: %.2f K\n', rippleJH_diodeQ1);
fprintf('IGBT Max Tj: %.2f K\n', TjMAX_igbt);
fprintf('Diode Max Tj: %.2f K\n', TjMAX_diode);
fprintf('IGBT design Tj: %.2f K\n', Tj_design_igbt);
fprintf('Diode design Tj: %.2f K\n', Tj_design_diode);
disp('----------------------------------------------------'); 