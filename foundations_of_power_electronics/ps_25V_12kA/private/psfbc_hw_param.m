function hw = psfbc_hw_param(udc1, pwr_nom, fpwm, hw_in)
%PSFBC_HW_PARAM  Hardware data of the phase-shift full bridge of ps_25V_12kA_r.slx (local replacement of
%   single_phase_psfbc_hwdata + hw_psfbc_single_phase_setup of the library: same formulas and field names,
%   the numbers of the data set PSFBC_800V come from the init in hw_in).
%
%   hw = psfbc_hw_param(udc1, pwr_nom, fpwm, hw_in)
%     udc1     [V]  nominal input voltage (the DC link of the AFE)
%     pwr_nom  [W]  nominal power
%     fpwm     [Hz] switching frequency of the full bridge
%     hw_in    numbers of the converter:
%       .udc2                  [V]   output voltage base (udc2_nom = udc2_bez)
%       .idc2                  [A]   output current base (idc2_nom = idc2_bez): reference of the current loop
%       .n1, .n2               turns of the centre-tapped HF transformer (primary n1, each secondary n2)
%       .Lm                    [H]   magnetizing inductance (primary)
%       .Rfe                   [Ohm] iron-loss resistance (primary)
%       .Rs1                   [Ohm] primary winding resistance (each secondary: Rs1/(n1/n2)^2)
%       .core_length, .core_mur      only for the flux-density output of the transformer block
%       .ls_factor             leakage design factor: Ls = udc1^2/(8*fpwm*pwr_nom*ls_factor), referred to the
%                              primary, half on the primary (Ld1) and half on each secondary (Ld2 = Ld1/n12^2)
%       .Cdc_dc1, .RCdc_dc1    [F], [Ohm] input capacitor of the full bridge (CFi_dc1) and its series resistance
%       .Cdc_dc2, .RCdc_dc2    [F], [Ohm] output capacitor
%       .Ldc_dc1, .RLdc_dc1    [H], [Ohm] input inductor of the data set (not in the model)
%       .Ldc_dc2, .RLdc_dc2    [H], [Ohm] output inductor
%
%   Fields read by ps_25V_12kA_r.slx: Cdc_dc1, RCdc_dc1 (CFi_dc1), Cdc_dc2, RCdc_dc2, Ldc_dc2, RLdc_dc2, the
%   transformer data n1, n2, Lm, Rfe, Rs1, Rs2, Ld1, Ld2, core_length, core_mur, mu0 (block
%   three_windings_single_phase_transformer: m1 = n1, m2 = m3 = n2, ld2 = ld3 = Ld2, rd2 = rd3 = Rs2), and the
%   bases of the control idc1_bez (input current, read as hwdata.dab.idc1_bez by the legacy model), idc2_bez,
%   udc2_bez, idc2_nom (current reference and rate limit).
%
%   The leakage formula is the one of the DAB data sets of the library: with Ls the maximum power of a DAB
%   (V1*V2/(8*f*Ls)) is ls_factor times pwr_nom. In this phase-shift full bridge the leakage is not a power
%   transfer element: it gives the duty-cycle loss Req = 4*(n2/n1)^2*Ls*fpwm on the output and the soft
%   switching (see the README).

    hw.name = 'PSFBC_800V';
    hw.pwr_nom = pwr_nom;
    hw.udc1_nom = udc1;
    hw.udc2_nom = hw_in.udc2;
    hw.uac1_nom = hw.udc1_nom;                      % library data set: uac1 = udc1, uac2 = udc2
    hw.uac2_nom = hw.udc2_nom;
    hw.idc1_nom = pwr_nom/hw.udc1_nom;              % [A] input current at pwr_nom (433 A at 325 kW / 750 V)
    hw.idc2_nom = hw_in.idc2;                       % [A] output current base of the data set (12 kA)
    hw.fpwm_base = fpwm;

    % centre-tapped transformer (primary n1, two secondaries n2 each)
    hw.n1 = hw_in.n1;
    hw.n2 = hw_in.n2;
    hw.Lm = hw_in.Lm;
    hw.Rfe = hw_in.Rfe;
    n12 = hw.n1/hw.n2;
    hw.ls_factor = hw_in.ls_factor;
    hw.Ls = hw.udc1_nom^2/fpwm/pwr_nom/8/hw.ls_factor;     % [H] total leakage referred to the primary
    hw.Rs1 = hw_in.Rs1;
    hw.Rs2 = hw.Rs1/n12^2;
    hw.Ld1 = hw.Ls/2;
    hw.Ld2 = hw.Ld1/n12^2;
    hw.Ls1 = hw.Ld1;
    hw.Ls2 = hw.Ld2;
    hw.core_length = hw_in.core_length;
    hw.core_mur = hw_in.core_mur;
    hw.mu0 = 4*pi*1e-7;                                    % [H/m] read by the transformer block (Bc output)

    % capacitors and inductors
    hw.Cdc_dc1 = hw_in.Cdc_dc1;
    hw.Cdc1_dc1 = 2*hw.Cdc_dc1;     hw.Cdc2_dc1 = 2*hw.Cdc_dc1;        % banks (not read by the model)
    hw.Cdc_dc2 = hw_in.Cdc_dc2;
    hw.Cdc1_dc2 = 2*hw.Cdc_dc2;     hw.Cdc2_dc2 = 2*hw.Cdc_dc2;
    hw.RCdc_dc1 = hw_in.RCdc_dc1;
    hw.RCdc1_dc1 = hw.RCdc_dc1/2;   hw.RCdc2_dc1 = hw.RCdc_dc1/2;
    hw.RCdc_dc2 = hw_in.RCdc_dc2;
    hw.RCdc1_dc2 = hw.RCdc_dc2/2;   hw.RCdc2_dc2 = hw.RCdc_dc2/2;      % library: Cdc_dc2/2 (typo, not read)
    hw.Ldc_dc1 = hw_in.Ldc_dc1;
    hw.RLdc_dc1 = hw_in.RLdc_dc1;
    hw.Ldc_dc2 = hw_in.Ldc_dc2;
    hw.RLdc_dc2 = hw_in.RLdc_dc2;

    % bases of the control
    hw.idc1_bez = hw.idc1_nom;
    hw.idc2_bez = hw.idc2_nom;
    hw.udc1_bez = hw.udc1_nom;
    hw.udc2_bez = hw.udc2_nom;
    hw.Idc_FS = max(hw.idc1_bez, hw.idc2_bez);
    hw.Vdc_FS = max(hw.udc1_bez, hw.udc2_bez);

    fprintf('Phase-shift full bridge: %s, %.0f kW, %.0f V -> %.0f V / %.0f A bases, %.0f Hz | transformer %d:%d:%d, Lm = %.2f mH, Ls = %.2f uH (Ld1 = %.2f uH, Ld2 = %.1f nH), Rs1 = %.2f mOhm\n', ...
        hw.name, hw.pwr_nom/1e3, hw.udc1_nom, hw.udc2_nom, hw.idc2_nom, fpwm, hw.n1, hw.n2, hw.n2, hw.Lm*1e3, ...
        hw.Ls*1e6, hw.Ld1*1e6, hw.Ld2*1e9, hw.Rs1*1e3);
    fprintf('Cin = %.2f mF + %.1f mOhm, Cout = %.0f mF + %.2f mOhm, Lout = %.2f uH + %.2f uOhm | idc1_bez = %.1f A\n', ...
        hw.Cdc_dc1*1e3, hw.RCdc_dc1*1e3, hw.Cdc_dc2*1e3, hw.RCdc_dc2*1e3, hw.Ldc_dc2*1e6, hw.RLdc_dc2*1e6, hw.idc1_bez);
    fprintf('---------------------------\n');
end
