% clear all
% close all
% clc
% 
% load sim_result_2.mat;

clc


u1_dab_transformer_rms = sqrt(mean(u1_dab_transformer_modA_sim.^2))
i1_dab_transformer_rms = sqrt(mean(i1_dab_transformer_modA_sim.^2))
i1_dab_transformer_max = max(i1_dab_transformer_modA_sim)

u2_dab_transformer_rms = sqrt(mean(u2_dab_transformer_modA_sim.^2))
i2_dab_transformer_rms = sqrt(mean(i2_dab_transformer_modA_sim.^2))
i2_dab_transformer_max = max(i2_dab_transformer_modA_sim)

u3_dab_transformer_rms = sqrt(mean(u3_dab_transformer_modA_sim.^2))
i3_dab_transformer_rms = sqrt(mean(i3_dab_transformer_modA_sim.^2))
i3_dab_transformer_max = max(i3_dab_transformer_modA_sim)


S1 = i1_dab_transformer_rms*u1_dab_transformer_rms
S2 = i2_dab_transformer_rms*u2_dab_transformer_rms
S3 = i3_dab_transformer_rms*u3_dab_transformer_rms

ib1 = mean(current_battery_1_sim)
ib2 = mean(current_battery_2_sim)
ub1 = mean(voltage_battery_1_sim)
ub2 = mean(voltage_battery_2_sim)

Pout1 = ib1*ub1
Pin2 = ib2*ub2

i_ctrl_out_dab_avg = mean(i_ctrl_out_dab_sim)
i_ref_pu = mean(u_ctrl_out_dab_sim)
i_pu = mean(i_out_pu_dab_sim)

p_loss_Q1 = mean(dab_inverter_1_device_data_sim(:,1));
p_loss_Q2 = mean(dab_inverter_1_device_data_sim(:,6));
p_loss_Q3 = mean(dab_inverter_1_device_data_sim(:,11));
p_loss_Q4 = mean(dab_inverter_1_device_data_sim(:,16));

p_loss_dab = p_loss_Q1+p_loss_Q2+p_loss_Q3+p_loss_Q4

