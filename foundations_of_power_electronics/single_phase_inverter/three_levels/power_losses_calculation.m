clc

T = 0.8;
N1 = floor(Nc) - floor(T/tc);
N2 = floor(Nc);
p_loss_inv_Q1 = mosfet_ploss_invQ1_sim(N1:N2);
p_loss_inv_Q2 = mosfet_ploss_invQ2_sim(N1:N2);
% figure; plot(p_loss_inv_Q1); grid on
p_loss_inv_Q1_m = mean(p_loss_inv_Q1)
p_loss_inv_Q2_m = mean(p_loss_inv_Q2)

p_loss_inv_leg = 2*(p_loss_inv_Q1_m+p_loss_inv_Q2_m)
p_loss_inv_total = 2*p_loss_inv_leg
