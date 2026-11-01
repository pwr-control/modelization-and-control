function hw = isop_rail_hw_param(udc1, pwr_nom, fpwm, hw_in)
%ISOP_RAIL_HW_PARAM  Hardware data of ONE module of ps_isop_rail (local replacement of
%   isop_rail_aux_application + hw_isop_rail_setup of the library: same formulas and field names, the
%   numbers come from the init in hw_in).
%
%   hw = isop_rail_hw_param(udc1, pwr_nom, fpwm, hw_in)
%     udc1     [V]  design input voltage of one module (catenary voltage / number of modules)
%     pwr_nom  [W]  nominal power of one module
%     fpwm     [Hz] switching frequency of the full bridge
%     hw_in    numbers of the module:
%       .udc2                  [V]   output voltage (sets the turns ratio n2/n1 = udc2/udc1)
%       .n1                    primary turns (the transformer block uses only the ratio)
%       .Lm                    [H]   magnetizing inductance (primary)
%       .Rfe                   [Ohm] iron-loss resistance (primary)
%       .Rs1                   [Ohm] primary winding resistance
%       .core_length, .core_mur      only for the flux-density output Bc of the transformer block
%       .ls_factor             leakage design factor: Ls = udc1^2/(8*fpwm*pwr_nom*ls_factor)
%       .Cdc_dc1, .RCdc_dc1    [F], [Ohm] input capacitor of one module (and its series resistance)
%       .Cdc_dc2, .RCdc_dc2    [F], [Ohm] common output capacitor
%       .Ldc_dc1, .RLdc_dc1    [H], [Ohm] input inductor of the series string
%       .Ldc_dc2, .RLdc_dc2    [H], [Ohm] output inductor of each diode rectifier
%
%   Fields read by ps_isop_rail.slx: Cdc_dc1, RCdc_dc1 (CFi_dc1 of each module), Cdc_dc2, RCdc_dc2,
%   Ldc_dc1, RLdc_dc1, Ldc_dc2, RLdc_dc2, the transformer data Lm, n1, n2, Rs1, Rs2, Ld1, Ld2, Rfe,
%   core_length, core_mur, mu0, and the bases idc1_bez, idc2_bez, udc2_bez, idc2_nom of the control.
%
%   The leakage formula is the one of the DAB data sets of the library: with Ls the maximum power of a DAB
%   (V1*V2/(8*f*Ls)) is ls_factor times pwr_nom. In this phase-shift full bridge the leakage is not a power
%   transfer element: it only gives the duty-cycle loss and the soft switching (see the README).

    hw.name = 'PS_FBC_750V';
    hw.pwr_nom = pwr_nom;
    hw.udc1_nom = udc1;
    hw.udc2_nom = hw_in.udc2;
    hw.uac1_nom = hw.udc1_nom*4/pi;                 % first harmonic of the square wave (informative)
    hw.uac2_nom = hw.udc2_nom*4/pi;
    hw.idc1_nom = pwr_nom/hw.udc1_nom;
    hw.idc2_nom = pwr_nom/hw.udc2_nom;
    hw.fpwm_base = fpwm;

    % transformer
    hw.n1 = hw_in.n1;
    hw.n2 = hw_in.n1*hw.udc2_nom/hw.udc1_nom;
    hw.Lm = hw_in.Lm;
    hw.Rfe = hw_in.Rfe;
    n12 = hw.n1/hw.n2;
    hw.ls_factor = hw_in.ls_factor;
    hw.Ls = hw.udc1_nom^2/fpwm/pwr_nom/8/hw.ls_factor;     % total leakage referred to the primary
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

    % bases of the control (one module)
    hw.idc1_bez = hw.idc1_nom;
    hw.idc2_bez = hw.idc2_nom;
    hw.udc1_bez = hw.udc1_nom;
    hw.udc2_bez = hw.udc2_nom;
    hw.Idc_FS = max(hw.idc1_bez, hw.idc2_bez);
    hw.Vdc_FS = max(hw.udc1_bez, hw.udc2_bez);

    fprintf('ISOP module: %.0f kW, %.0f V -> %.0f V (n2/n1 = %.3f), %.0f Hz | Lm = %.2f mH, Ld1 = Ld2 = %.2f uH (Ls = %.2f uH) | Cin = %.2f mF, Cout = %.2f mF, Lin = %.2f mH, Lout = %.0f uH\n', ...
        hw.pwr_nom/1e3, hw.udc1_nom, hw.udc2_nom, hw.n2/hw.n1, fpwm, hw.Lm*1e3, hw.Ld1*1e6, hw.Ls*1e6, ...
        hw.Cdc_dc1*1e3, hw.Cdc_dc2*1e3, hw.Ldc_dc1*1e3, hw.Ldc_dc2*1e6);
end
