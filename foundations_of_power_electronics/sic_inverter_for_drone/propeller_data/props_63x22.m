function [torque] = props_63x22(n)

kt = 0.00515;
rho_air = 1.225;
D = 1.6;
n_thr = 190/60;
torque = kt*rho_air*D^4*n./60*sqrt((n./60)^2 + n_thr^2);
end

