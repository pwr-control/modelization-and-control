clc

idc_1 = mean(current_battery_1_sim)
udc_1 = mean(voltage_battery_1_sim)
idc_2 = mean(current_battery_2_sim)
udc_2 = mean(voltage_battery_2_sim)

% idc_1 = mean(cllc_current_input_modA_sim)
% udc_1 = mean(cllc_voltage_input_modA_sim)
% idc_2 = mean(cllc_current_output_modA_sim)
% udc_2 = mean(cllc_voltage_output_modA_sim)

p1 = idc_1*udc_1
p2 = idc_2*udc_2


i1_tr = sqrt(mean(i1_cllc_transformer_modA_sim.^2))
i2_tr = sqrt(mean(i2_cllc_transformer_modA_sim.^2))
u1_tr = sqrt(mean(u1_cllc_transformer_modA_sim.^2))
u2_tr = sqrt(mean(u2_cllc_transformer_modA_sim.^2))

p1_tr = mean(i1_cllc_transformer_modA_sim.*u1_cllc_transformer_modA_sim)
p2_tr = mean(i2_cllc_transformer_modA_sim.*u2_cllc_transformer_modA_sim)

p_balance_dc = p1 + p2
p_balance_ac = p1_tr - p2_tr

p_loss_cllc_1_Q1 = mean(inverter_1_cllc_devices_data_modA_sim(:,1))
p_loss_cllc_1_Q2 = mean(inverter_1_cllc_devices_data_modA_sim(:,6))
p_loss_cllc_1_Q3 = mean(inverter_1_cllc_devices_data_modA_sim(:,11))
p_loss_cllc_1_Q4 = mean(inverter_1_cllc_devices_data_modA_sim(:,16))
p_loss_cllc_2_Q1 = mean(inverter_2_cllc_devices_data_modA_sim(:,1))
p_loss_cllc_2_Q2 = mean(inverter_2_cllc_devices_data_modA_sim(:,6))
p_loss_cllc_2_Q3 = mean(inverter_2_cllc_devices_data_modA_sim(:,11))
p_loss_cllc_2_Q4 = mean(inverter_2_cllc_devices_data_modA_sim(:,16))

p_loss_cllc = (p_loss_cllc_1_Q1 + p_loss_cllc_1_Q2 + ...
    p_loss_cllc_1_Q3 + p_loss_cllc_1_Q4) + ...
    (p_loss_cllc_2_Q1 + p_loss_cllc_2_Q2 + ...
    p_loss_cllc_2_Q3 + p_loss_cllc_2_Q4)
