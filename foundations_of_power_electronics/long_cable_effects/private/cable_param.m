function cable = cable_param(c)
%CABLE_PARAM  Lumped high-frequency model of the power cable between the inverter and the machine of
%   afe_inv_psm_with_hf_models.slx (legacy struct hf_power_cable_model of init_model_legacy.m).
%
%   cable = cable_param(c), with c = param.cable_in:
%     c.n         number of lumped sections per phase (blocks power_cable_model_1/2/3, each with
%                 parasitic_lump_1 and parasitic_lump_2: two sections)
%     c.L, c.R    [H], [Ohm] series inductance and resistance of the whole cable, per phase (L from a
%                 measurement with 10 uF, as the legacy comment says)
%     c.C, c.RC   [F], [Ohm] capacitance of the whole cable from one phase to ground, and the series
%                 resistance of that branch
%     c.Cp2, c.RCs2  [F], [Ohm] second capacitor branch of a section, at its input (Cp2 is commented out in
%                 the model)
%     c.Rp1       [Ohm] resistor from the output of a section to ground (commented out in the model)
%
%   One section (parasitic_lump_k): u1 -> inductor Ls1 (series resistance Rs1) -> u2; capacitor Cp1 (series
%   resistance RCs1) from u2 to ground; Cp2 (RCs2) from u1 to ground; Rp1 from u2 to ground. The legacy init
%   divided L, R, C and RC by n and kept RCs2, Cp2 and Rp1 per section: same numbers here.
%
%   Fields read by afe_inv_psm_with_hf_models.slx (power_cables_and_psm_high_frequency_models/power_cable_model_*
%   and the commented afe_abc_inv_psm_mod1/parasitic_lump_1/2): Ls1, Rs1, Cp1, RCs1, Cp2, RCs2, Rp1.
%   f1 and f2 are the resonance of one section with Cp1 and with Cp2 (informative, legacy f1 and f2).

    cable.n = c.n;
    cable.L_total = c.L;                                    % [H] per phase
    cable.R_total = c.R;                                    % [Ohm]
    cable.C_total = c.C;                                    % [F] phase to ground
    cable.Ls1 = c.L/c.n;                                    % [H] series inductance of one section
    cable.Rs1 = c.R/c.n;                                    % [Ohm] its series resistance
    cable.Cp1 = c.C/c.n;                                    % [F] capacitor to ground at the output of a section
    cable.RCs1 = c.RC/c.n;                                  % [Ohm] its series resistance (legacy: divided by n)
    cable.Cp2 = c.Cp2;                                      % [F] second capacitor branch (commented in the model)
    cable.RCs2 = c.RCs2;                                    % [Ohm]
    cable.Rp1 = c.Rp1;                                      % [Ohm] resistor to ground (commented in the model)

    cable.f1 = 1/(2*pi*sqrt(cable.Ls1*cable.Cp1));          % [Hz] one section with Cp1
    cable.f2 = 1/(2*pi*sqrt(cable.Ls1*cable.Cp2));          % [Hz] one section with Cp2
    cable.Z0 = sqrt(cable.Ls1/cable.Cp1);                   % [Ohm] characteristic impedance of a section
    cable.Q1 = cable.Z0/(cable.Rs1 + cable.RCs1);           % [-] quality factor of a section (series losses only)

    fprintf('Cable: %d sections per phase, per section Ls1 = %.1f uH, Rs1 = %.1f mOhm, Cp1 = %.2f nF (RCs1 = %.2f mOhm) | f1 = %.0f kHz, Z0 = %.1f Ohm, Q = %.0f\n', ...
        cable.n, cable.Ls1*1e6, cable.Rs1*1e3, cable.Cp1*1e9, cable.RCs1*1e3, cable.f1/1e3, cable.Z0, cable.Q1);
end
