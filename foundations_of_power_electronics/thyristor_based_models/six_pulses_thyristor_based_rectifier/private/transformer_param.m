function tr = transformer_param(t)
%TRANSFORMER_PARAM  Data of the coupling transformer of six_pulse_thyristor_based_rectifier.slx: block
%   grid_line/Grid/Dyg_TR/configurable_trafo (Simscape component configurable_transformer of the library,
%   one primary, three-phase secondary), between the grid emulator and the thyristor bridge.
%
%   tr = transformer_param(t), with t = the numbers of the init (param.grid.trafo_in):
%     t.Sr                 [VA] rated apparent power of the secondary (three-phase)
%     t.V1, t.V2, t.V3     [V] rated line-to-line voltages of primary 1, primary 2 (not used: one primary)
%                          and of the secondary
%     t.fr                 [Hz] rated frequency
%     t.r1, t.l1, t.r2, t.l2, t.r3, t.l3   [pu] resistance and leakage inductance of each coil
%     t.lm, t.lm0, t.rfe   [pu] magnetizing inductance (positive/negative and zero sequence) and core-loss
%                          resistance per limb, on the base of the primary-1 coil
%     t.conn1, t.conn3     'delta' | 'star': connection of primary 1 and of the secondary (enumerations
%                          typed in the block: conn1 = delta, secondary = star; here only for the checks)
%     t.clock2, t.clock3   clock numbers of primary 2 and of the secondary against primary 1 (IEC 60076-1:
%                          the secondary voltages lag primary 1 by clock*30 deg)
%
%   Fields read by the block: Sr, V1, V2, V3, fr, r1, l1, r2, l2, r3, l3, lm, lm0, rfe, clock2, clock3.
%   Per-unit bases as in configurable_transformer.ssc: a star coil V^2/Sr, a delta coil 3*V^2/Sr; lm, lm0
%   and rfe on the base of the primary-1 coil. The same parity rule as the asserts of the component is
%   checked here: primary 1 delta and secondary star (or the opposite) need an odd clock number, the same
%   connection on both sides an even one. The model had clock3 = 0 with a delta primary and a star
%   secondary, which the component rejects at compile time.

    tr = t;
    wb = 2*pi*t.fr;

    % parity rule of the clock number (configurable_transformer.ssc)
    if abs(t.clock3 - round(t.clock3)) > 1e-9 || t.clock3 < 0 || t.clock3 > 11
        error('transformer_param:clock', 'clock3 = %g must be an integer 0 ... 11', t.clock3);
    end
    if strcmp(t.conn1, t.conn3) ~= (mod(t.clock3, 2) == 0)
        error('transformer_param:clock', ['primary 1 %s, secondary %s: the clock number must be %s ' ...
            '(clock3 = %d)'], t.conn1, t.conn3, ternary(strcmp(t.conn1, t.conn3), 'even', 'odd'), t.clock3);
    end
    tr.vector_group = sprintf('%s%s%d', ternary(strcmp(t.conn1, 'delta'), 'D', 'Y'), ...
        ternary(strcmp(t.conn3, 'delta'), 'd', 'yn'), t.clock3);   % e.g. Dyn11 (star point of the secondary grounded)

    % coil bases and physical values
    Zc1 = coil_base(t.conn1, t.V1, t.Sr);                  % [Ohm] primary-1 coil
    Zc3 = coil_base(t.conn3, t.V3, t.Sr);                  % [Ohm] secondary coil
    tr.R1 = t.r1*Zc1;  tr.L1 = t.l1*Zc1/wb;                % [Ohm], [H] per coil
    tr.R3 = t.r3*Zc3;  tr.L3 = t.l3*Zc3/wb;
    tr.Lm1 = t.lm*Zc1/wb;                                  % [H] magnetizing inductance per primary coil
    tr.Rfe1 = t.rfe*Zc1;                                   % [Ohm] core-loss resistance per limb

    % coil voltages and turns ratio N3/N1
    V1c = coil_voltage(t.conn1, t.V1);
    V3c = coil_voltage(t.conn3, t.V3);
    tr.n31 = V3c/V1c;

    % short-circuit impedance referred to the secondary, per phase of the equivalent star (what the bridge
    % sees as commutation impedance, together with the series inductors of the rectifier)
    if strcmp(t.conn3, 'star')
        k3 = 1;
    else
        k3 = 1/3;                                          % delta coil -> star equivalent
    end
    tr.R_sc2 = (tr.R3 + tr.R1*tr.n31^2)*k3;                % [Ohm]
    tr.L_sc2 = (tr.L3 + tr.L1*tr.n31^2)*k3;                % [H]
    tr.ucc = (t.l1 + t.l3)*100;                            % [%] short-circuit voltage
    tr.ur = (t.r1 + t.r3)*100;                             % [%] copper losses at rated current
    tr.I2_nom = t.Sr/sqrt(3)/t.V3;                         % [A] rms rated secondary current
    tr.i_mag_pct = 100/t.lm;                               % [%] magnetizing current
    tr.p_iron = 3*V1c^2/tr.Rfe1;                           % [W] iron losses at rated voltage

    fprintf('Transformer %s %.0f kVA %.0f/%.0f V: ucc %.1f %%, ur %.1f %%, referred to the secondary L_sc2 = %.1f uH, R_sc2 = %.1f mOhm per phase, I2_nom = %.1f A, iron %.0f W\n', ...
        tr.vector_group, t.Sr/1e3, t.V1, t.V3, tr.ucc, tr.ur, tr.L_sc2*1e6, tr.R_sc2*1e3, tr.I2_nom, tr.p_iron);
end

function Z = coil_base(conn, V, Sr)
    if strcmp(conn, 'delta')
        Z = 3*V^2/Sr;
    else
        Z = V^2/Sr;
    end
end

function Vc = coil_voltage(conn, V)
    if strcmp(conn, 'delta')
        Vc = V;
    else
        Vc = V/sqrt(3);
    end
end

function s = ternary(c, a, b)
    if c
        s = a;
    else
        s = b;
    end
end
