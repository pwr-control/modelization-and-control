clear all
close all
clc

load sim_results_1.mat;


u1_dab_transformer_rms = sqrt(mean(u1_dab_transformer_modA_sim.^2))
u1_dab_transformer_max = max(u1_dab_transformer_modA_sim)
i1_dab_transformer_rms = sqrt(mean(i1_dab_transformer_modA_sim.^2))

u2_dab_transformer_rms = sqrt(mean(u2_dab_transformer_modA_sim.^2))
i2_dab_transformer_rms = sqrt(mean(i2_dab_transformer_modA_sim.^2))

u3_dab_transformer_rms = sqrt(mean(u3_dab_transformer_modA_sim.^2))
i3_dab_transformer_rms = sqrt(mean(i3_dab_transformer_modA_sim.^2))

i_in = mean(current_battery_1_sim)
i_out = mean(current_battery_2_sim)
u_in = mean(voltage_battery_1_sim)
u_out = mean(voltage_battery_2_sim)

i_ctrl_out_dab_avg = mean(i_ctrl_out_dab_sim)
i_ref_pu = mean(u_ctrl_out_dab_sim)
i_pu = mean(i_out_pu_dab_sim)

Q1_rms_current = sqrt(mean(full_bridge_inverter_1_device_data_sim(:,3).^2))
Q1_mean_ploss = mean(full_bridge_inverter_1_device_data_sim(:,1))
Q3_rms_current = sqrt(mean(full_bridge_inverter_1_device_data_sim(:,13).^2))
Q3_mean_ploss = mean(full_bridge_inverter_1_device_data_sim(:,11))

diode_rectifier1_peak_voltage = max(diode_rectifier_bridge_1_sim(:,1))
diode_rectifier1_rms_current = sqrt(mean(diode_rectifier_bridge_1_sim(:,2).^2))

% diode_rectifier2_peak_voltage = max(diode_rectifier_bridge_2_sim(:,1))
% diode_rectifier2_rms_current = sqrt(mean(diode_rectifier_bridge_2_sim(:,2).^2))