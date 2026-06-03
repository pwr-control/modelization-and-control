function [k] = intepolation(omega, k1, k2)

omega_sup = 0.5;
omega_inf = 0.2;
k = (omega - omega_sup)/(omega_inf-omega_sup)*k1 - ...
    (omega - omega_inf)/(omega_inf-omega_sup)*k2;
end

