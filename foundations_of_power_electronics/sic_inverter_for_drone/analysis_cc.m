clc
close all

kp_id = 0.15;
ki_id = 35;

reg_cc_inv_d = ki_id/s+kp_id;
regd_cc_inv_d=c2d(reg_cc_inv_d,ts_inv);
% p_cc_inv_d = ubez/(s*Ld_norm + Rs_norm);
% p_cc_inv_d = motorc_m_scale/(s*Ld_norm + Rs_norm);
p_cc_inv_d = 1/(s*Ld + Rs);
pd_cc_inv_d = c2d(p_cc_inv_d,ts_inv);
inv_cc_plantd_d = minreal(pd_cc_inv_d*regd_cc_inv_d);
figure; margin(inv_cc_plantd_d); 
set(cstprefs.tbxprefs,'FrequencyUnits','Hz');
legend('id-cc');
grid on;

kp_id = 0.1;
ki_id = 45;

reg_cc_inv_d = ki_id/s+kp_id;
regd_cc_inv_d=c2d(reg_cc_inv_d,ts_inv);
% p_cc_inv_d = ubez/(s*Ld_norm + Rs_norm);
% p_cc_inv_d = motorc_m_scale/(s*Ld_norm + Rs_norm);
p_cc_inv_d = 1/(s*Ld + Rs);
pd_cc_inv_d = c2d(p_cc_inv_d,ts_inv);
inv_cc_plantd_d = minreal(pd_cc_inv_d*regd_cc_inv_d);
figure; margin(inv_cc_plantd_d); 
set(cstprefs.tbxprefs,'FrequencyUnits','Hz');
legend('id-cc');
grid on;

kp_id = 0.15;
ki_id = 2;

reg_cc_inv_d = ki_id/s+kp_id;
regd_cc_inv_d=c2d(reg_cc_inv_d,ts_inv);
% p_cc_inv_d = ubez/(s*Ld_norm + Rs_norm);
% p_cc_inv_d = motorc_m_scale/(s*Ld_norm + Rs_norm);
p_cc_inv_d = 1/(s*Ld + Rs);
pd_cc_inv_d = c2d(p_cc_inv_d,ts_inv);
inv_cc_plantd_d = minreal(pd_cc_inv_d*regd_cc_inv_d);
figure; margin(inv_cc_plantd_d); 
set(cstprefs.tbxprefs,'FrequencyUnits','Hz');
legend('id-cc');
grid on;