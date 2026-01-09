clc

Vdc_nom = 400*1.35;
T = 0.8;
N1 = floor(Nc) - floor(T/tc);
N2 = floor(Nc);
N1s = floor(Ns_inv) - floor(T/ts_inv);
N2s = floor(Ns_inv);
p_loss_inv_Q1 = mosfet_ploss_invQ1_sim(N1:N2);
p_loss_inv_Q2 = mosfet_ploss_invQ2_sim(N1:N2);
% figure; plot(p_loss_inv_Q1); grid on
p_loss_inv_Q1_m = mean(p_loss_inv_Q1)
p_loss_inv_Q2_m = mean(p_loss_inv_Q2)

p_loss_inv_module = (p_loss_inv_Q1_m+p_loss_inv_Q2_m)
p_loss_inv_total = 2*p_loss_inv_module

output_current_rms = sqrt(mean(inverter_current_output_sim(N1:N2).^2))
output_voltage_rms = sqrt(mean(inverter_voltage_output_sim(N1:N2).^2))
ud_ctrl = mean(ud_ctrl_out_sim(N1s:N2s))
uq_ctrl = mean(uq_ctrl_out_sim(N1s:N2s))
output_voltage_first_h_rms = Vdc_nom * sqrt(ud_ctrl^2 + uq_ctrl^2) / sqrt(2)

CFi_ac_current = sqrt(mean(CFi_dc_current_sim(N1:N2).^2))
Pgrid = mean(grid_metering_sim(N1:N2,4))
Id_rms = sqrt(mean(mosfet_current_invQ1_sim(N1:N2).^2))
Ud_max = max(abs(mosfet_voltage_invQ1_sim(N1:N2)))
DTjh_Q1 = max(abs(mosfet_JH_temp_invQ1_sim(N1:N2)))

Idr1_rms = sqrt(mean(diode_bridge_rectifier_data_sim(N1:N2,2).^2))
Idr2_rms = sqrt(mean(diode_bridge_rectifier_data_sim(N1:N2,7).^2))
Idr3_rms = sqrt(mean(diode_bridge_rectifier_data_sim(N1:N2,12).^2))
Idr4_rms = sqrt(mean(diode_bridge_rectifier_data_sim(N1:N2,17).^2))
Idr5_rms = sqrt(mean(diode_bridge_rectifier_data_sim(N1:N2,22).^2))
Idr6_rms = sqrt(mean(diode_bridge_rectifier_data_sim(N1:N2,27).^2))
Pdr1_avg = mean(diode_bridge_rectifier_data_sim(N1:N2,5))
Pdr2_avg = mean(diode_bridge_rectifier_data_sim(N1:N2,10))
Pdr3_avg = mean(diode_bridge_rectifier_data_sim(N1:N2,15))
Pdr4_avg = mean(diode_bridge_rectifier_data_sim(N1:N2,20))
Pdr5_avg = mean(diode_bridge_rectifier_data_sim(N1:N2,25))
Pdr6_avg = mean(diode_bridge_rectifier_data_sim(N1:N2,30))
Pdr_avg = Pdr1_avg+Pdr2_avg+Pdr3_avg+Pdr4_avg+Pdr5_avg+Pdr6_avg