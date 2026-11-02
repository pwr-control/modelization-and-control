function hw = dab_hw_param(name, pwr_nom, fpwm, fres, h)
%DAB_HW_PARAM  Hardware data of the single-phase DAB (local copy of hw_dab_single_phase_setup of the library,
%   same field names and formulas; the numbers are in init_model.m, param.dab.hw_in).
%
%   hw = dab_hw_param(name, pwr_nom, fpwm, fres, h)
%     name    : name of the data set (only printed)
%     pwr_nom : [W] nominal power
%     fpwm    : [Hz] frequency of the transformer (= switching frequency of the bridges, fPWM_DAB)
%     fres    : [Hz] resonance frequency of the series capacitors with the series inductance
%     h       : struct with the numbers (param.dab.hw_in): udc1, udc2, uac1, uac2, idc1, idc2, n1, n2, Lm, Rfe,
%               Rs1, core_length, core_mur, Cdc_dc1, Cdc1_dc1, Cdc2_dc1, Cdc_dc2, Cdc1_dc2, Cdc2_dc2, RCdc_dc1,
%               RCdc1_dc1, RCdc2_dc1, RCdc_dc2, RCdc1_dc2, RCdc2_dc2, Ldc_dc1, Ldc_dc2, RLdc_dc1, RLdc_dc2
%
%   Derived (as in the class):
%     Ls  = udc1^2/(4*pi*fpwm*pwr_nom)     total series inductance referred to the primary (leakage + external).
%                                         Lossless DAB with phase shift phi: P = udc1*udc2'*phi*(1 - |phi|/pi)/(2*pi*fpwm*Ls),
%                                         so this Ls gives Pmax = pi/2*pwr_nom at phi = 90 deg with udc2' = udc1
%                                         ("version with some margin" in the class; its commented /8 version gives
%                                         Pmax = pwr_nom)
%     Cs  = 1/(Ls*(2*pi*fres)^2)          total series capacitor (DC blocking), split in Cs1 (primary) and Cs2
%     Ld1 = Ls/2, Ld2 = Ld1/n12^2         leakage of the transformer block, half per side (n12 = n1/n2)
%     Cs1 = 2*Cs, Cs2 = Cs1*n12^2         series capacitors of the blocks Capacitor (primary) and Capacitor1
%                                         (secondary) of dab_modA/dcdc_with_galvanic_isolation/single_phase_dab_transformer
%     Rs2 = Rs1/n12^2                     winding resistance of the secondary
%     idc1_bez = idc1 ... udc2_bez = udc2 normalization bases; Idc_FS = max(idc1, idc2), Vdc_FS = max(udc1, udc2):
%                                         end scales of the current and voltage feedbacks of the control
%
%   Fields read by single_phase_dab.slx: udc2_nom, Idc_FS, Vdc_FS, Cdc_dc1, RCdc_dc1, Cdc_dc2, RCdc_dc2, Ldc_dc1,
%   RLdc_dc1, Ldc_dc2, RLdc_dc2, Cs1, Cs2, Lm, n1, n2, Rs1, Rs2, Ld1, Ld2, Rfe, core_length, core_mur.

    hw.name = name;
    hw.pwr_nom = pwr_nom;
    hw.udc1_nom = h.udc1;
    hw.udc2_nom = h.udc2;
    hw.uac1_nom = h.uac1;
    hw.uac2_nom = h.uac2;
    hw.idc1_nom = h.idc1;
    hw.idc2_nom = h.idc2;
    hw.fpwm_base = fpwm;
    hw.fres_base = fres;
    hw.n1 = h.n1;
    hw.n2 = h.n2;
    hw.Lm = h.Lm;
    hw.Rfe = h.Rfe;

    n12 = hw.n1/hw.n2;
    hw.Ls = hw.udc1_nom^2/hw.fpwm_base/hw.pwr_nom/4/pi;         % [H] (the class keeps /8 as a comment)
    hw.Cs = 1/hw.Ls/(2*pi*hw.fres_base)^2;                     % [F]

    hw.Rs1 = h.Rs1;
    hw.Rs2 = hw.Rs1/n12^2;
    hw.Ld1 = hw.Ls/2;
    hw.Ld2 = hw.Ld1/n12^2;
    hw.Cs1 = hw.Cs*2;
    hw.Cs2 = hw.Cs1*n12^2;
    hw.Ls1 = hw.Ld1;
    hw.Ls2 = hw.Ld2;

    hw.core_length = h.core_length;
    hw.core_mur = h.core_mur;

    hw.Cdc_dc1 = h.Cdc_dc1;
    hw.Cdc1_dc1 = h.Cdc1_dc1;
    hw.Cdc2_dc1 = h.Cdc2_dc1;
    hw.Cdc_dc2 = h.Cdc_dc2;
    hw.Cdc1_dc2 = h.Cdc1_dc2;
    hw.Cdc2_dc2 = h.Cdc2_dc2;
    hw.RCdc_dc1 = h.RCdc_dc1;
    hw.RCdc1_dc1 = h.RCdc1_dc1;
    hw.RCdc2_dc1 = h.RCdc2_dc1;
    hw.RCdc_dc2 = h.RCdc_dc2;
    hw.RCdc1_dc2 = h.RCdc1_dc2;
    hw.RCdc2_dc2 = h.RCdc2_dc2;

    hw.Ldc_dc1 = h.Ldc_dc1;
    hw.Ldc_dc2 = h.Ldc_dc2;
    hw.RLdc_dc1 = h.RLdc_dc1;
    hw.RLdc_dc2 = h.RLdc_dc2;

    hw.idc1_bez = hw.idc1_nom;
    hw.idc2_bez = hw.idc2_nom;
    hw.udc1_bez = hw.udc1_nom;
    hw.udc2_bez = hw.udc2_nom;
    hw.Idc_FS = max(hw.idc1_bez, hw.idc2_bez);
    hw.Vdc_FS = max(hw.udc1_bez, hw.udc2_bez);

    fprintf(['DAB hardware %s: %.0f kW, %.0f V / %.0f V, n1:n2 = %d:%d, Ls = %.1f uH (Ld1 = %.1f uH, Ld2 = %.1f uH), ' ...
        'Cs = %.1f uF (Cs1 = Cs2 = %.1f uF, fres = %.2f kHz), Lm = %.2f mH, DC links %.1f mF, DC chokes %.0f uH\n'], ...
        hw.name, hw.pwr_nom/1e3, hw.udc1_nom, hw.udc2_nom, hw.n1, hw.n2, hw.Ls*1e6, hw.Ld1*1e6, hw.Ld2*1e6, ...
        hw.Cs*1e6, hw.Cs1*1e6, hw.fres_base/1e3, hw.Lm*1e3, hw.Cdc_dc1*1e3, hw.Ldc_dc1*1e6);
end
