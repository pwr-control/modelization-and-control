function psm = psm_calculus()
    
    psm_sys = 6;
    psm_pwr = 1600e3;
    psm_i = 2062;
    psm_rpm = 17.8;
    psm_np = 104;
    psm_eta = 96;
    psm_Lq = 1.26e-3;
    psm_Ld = 1.04e-3;
    psm_Jm = 900e3/6;
    
    psm = pmsm_setup('WindGen', psm_sys, psm_pwr, psm_i, psm_rpm, psm_np, psm_eta, psm_Lq, psm_Ld, psm_Jm, 0, 0);
    displayInfo(psm);
end