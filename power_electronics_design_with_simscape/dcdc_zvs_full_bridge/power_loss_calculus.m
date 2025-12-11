% clear all
% close all
% clc
% 
% load sim_result_2.mat;

clc

i1_dab_transformer_rms = sqrt(mean(i1_dab_transformer_modA_sim.^2))
i2_dab_transformer_rms = sqrt(mean(i2_dab_transformer_modA_sim.^2))
u1_dab_transformer_rms = sqrt(mean(u1_dab_transformer_modA_sim.^2))
u2_dab_transformer_rms = sqrt(mean(u2_dab_transformer_modA_sim.^2))

S1 = i1_dab_transformer_rms*u1_dab_transformer_rms
S2 = i2_dab_transformer_rms*u2_dab_transformer_rms

ib1 = mean(current_battery_1_sim)
ib2 = mean(current_battery_2_sim)
ub1 = mean(voltage_battery_1_sim)
ub2 = mean(voltage_battery_2_sim)
Pout1 = ib1*ub1
Pin2 = ib2*ub2

return 
idc_grid = mean(current_battery_sim);
udc_grid = mean(voltage_battery_sim);
p_dc_grid = mean(current_battery_sim.*voltage_battery_sim)

p_loss_dab1_Q1 = mean(inverter_1_dab_devices_data_modA_sim(:,1));
p_loss_dab1_Q2 = mean(inverter_1_dab_devices_data_modA_sim(:,6));
p_loss_dab2_Q1 = mean(inverter_2_dab_devices_data_modA_sim(:,1));
p_loss_dab2_Q2 = mean(inverter_2_dab_devices_data_modA_sim(:,6));

p_loss_afe_Q1 = mean(inverter_device_data_modA_sim(:,1));
p_loss_afe_Q2 = mean(inverter_device_data_modA_sim(:,6));

p_loss_dab = 2*(p_loss_dab1_Q1+p_loss_dab1_Q2) + 2*(p_loss_dab2_Q1+p_loss_dab2_Q2)
p_loss_inv = 4*(p_loss_afe_Q1+p_loss_afe_Q2)

ac_grid_voltage_rms = sqrt(mean(ac_grid_voltage_sim.^2));
ac_grid_current_rms = sqrt(mean(ac_grid_current_sim.^2));

p_ac_grid = mean(ac_grid_voltage_sim.*ac_grid_current_sim)

p_dc_est = p_ac_grid - 2*p_loss_dab -2*p_loss_inv

% ploss_calc = p_ac_grid + p_dc_grid
ploss_calc = 2*p_loss_dab + 2*p_loss_inv

efficiency = 1 - abs(ploss_calc/p_dc_grid)

p_dab_modA_input = mean(dab_voltage_input_modA_sim.*dab_current_input_modA_sim)
