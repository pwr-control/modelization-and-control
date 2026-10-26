function trafo = three_phase_transformer_param(t)
%THREE_PHASE_TRANSFORMER_PARAM  Equivalent circuit of a two-winding three-phase transformer from its nameplate.
%
%   trafo = three_phase_transformer_param(t) returns the coil data used by the models made of single-phase
%   transformers (grid/grid_emulator/3ph_Dyg/saturable_transformer_Dyn, commented out) and, in trafo.pu, the
%   per-unit data read by the Simscape language blocks three_phase_transformer.transformer_Dd / _Yyn / _Dyn of
%   the library (blocks T1 and T2 of power_system.slx). Fields of t:
%     name        : label
%     connection  : vector group: 'Dd0', 'Dd6', 'Yyn0', 'Yyn6', 'Dyn11', 'Dyn1', 'Dyn5', 'Dyn7' (also 'Yy..',
%                   'Dy..', 'Yd..' for the coil data only). First letter: primary, second: secondary, number:
%                   lag of the secondary in steps of 30 deg (clock number of the library blocks)
%     pwr_nom     : [VA] rated three-phase power
%     u1_nom      : [V] rated line-to-line voltage of the primary (the side named first in connection)
%     u2_nom      : [V] rated line-to-line voltage of the secondary
%     f_nom       : [Hz]
%     eta         : [%] efficiency at rated load: copper losses = (1 - eta/100)*pwr_nom, half per winding
%     ucc         : [%] short-circuit voltage: leakage, half per winding
%     p_iron      : [W] iron losses, total
%     i1m         : [A] magnetizing current of the PRIMARY COIL at its rated voltage (the line voltage for a
%                   delta coil, the phase voltage for a star coil). Empty: computed from i0_pu
%     i0_pu       : [pu] magnetizing current in pu of the rated current (used when i1m is empty)
%     lm0_pu      : [pu] zero-sequence magnetizing inductance of the library blocks. Empty: equal to lm
%                   (bank of three single-phase units, or five-limb core); 0.3 - 1 for a three-limb core
%     n1, n2, core_area [m^2], core_length [m], mur, enable_sat : coils and core of the saturable single-phase
%                   transformer blocks (optional: the geometry fields are only reported)
%
%   Coil data (fields read by the single-phase transformer blocks): n1, n2, Rd1, Ld1, Rd2, Ld2, Lm1, Rfe1, psi
%   (phi_sat = 1.5*psi), core_area, core_length, mu0, mur, enable_sat. A delta coil carries the line current
%   divided by sqrt(3) at the line voltage: its resistance and leakage are 3 x the star-equivalent values,
%   its magnetizing data (Lm1, Rfe1, psi) are referred to the line voltage (the library class
%   three_phase_transformer_setup used the phase voltage for both connections, 2026-10-26).
%
%   Per-unit data (trafo.pu, parameters of the library blocks): Sr, V1, V2, fr, r1 = r2 = (1 - eta/100)/2,
%   l1 = l2 = ucc/200, lm = 1/i0_pu (rated over magnetizing current, line quantities), lm0, rfe = Sr/p_iron,
%   clock (the clock number of connection) and block (the library block of this connection). The blocks
%   take the coil bases themselves (3*V^2/Sr for a delta coil), so the same pu numbers describe the same
%   transformer whatever the connection.

    tok = regexp(t.connection, '^([DY])(d|y|yn)(\d+)$', 'tokens', 'once');
    if isempty(tok)
        error('three_phase_transformer_param: connection ''%s'' not understood (Dd0, Dd6, Yyn0, Yyn6, Dyn11, Dyn1, Dyn5, Dyn7)', ...
            t.connection);
    end
    delta_prim = strcmp(tok{1}, 'D');
    delta_sec = strcmp(tok{2}, 'd');
    clock = str2double(tok{3});
    if (delta_prim == delta_sec && ~any(clock == [0 6])) || (delta_prim ~= delta_sec && ~any(clock == [1 5 7 11]))
        error('three_phase_transformer_param: clock number %d is not possible for the connection %s', clock, t.connection);
    end
    if delta_prim && ~delta_sec
        block = 'transformer_Dyn';
    elseif ~delta_prim && ~delta_sec
        block = 'transformer_Yyn';
    elseif delta_prim && delta_sec
        block = 'transformer_Dd';
    else
        block = '';                                                         % Yd: no library block
    end

    omega = 2*pi*t.f_nom;
    trafo.name = t.name;
    trafo.connection = t.connection;
    trafo.delta_star = double(delta_prim);                                  % legacy flag: 1 delta primary, 0 star
    trafo.pwr_nom = t.pwr_nom;
    trafo.u1_nom = t.u1_nom;
    trafo.i1_nom = t.pwr_nom/sqrt(3)/t.u1_nom;                              % [A] line current
    trafo.u2_nom = t.u2_nom;
    trafo.i2_nom = t.pwr_nom/sqrt(3)/t.u2_nom;
    trafo.f_nom = t.f_nom;
    trafo.ucc = t.ucc;
    trafo.eta = t.eta;
    trafo.p_iron = t.p_iron;
    trafo.n12 = t.u1_nom/t.u2_nom;                                          % line-to-line ratio

    % coil voltages and coil base factors (3 for a delta coil, 1 for a star coil)
    if delta_prim
        trafo.u1_coil = t.u1_nom;  k1 = 3;
    else
        trafo.u1_coil = t.u1_nom/sqrt(3);  k1 = 1;
    end
    if delta_sec
        trafo.u2_coil = t.u2_nom;  k2 = 3;
    else
        trafo.u2_coil = t.u2_nom/sqrt(3);  k2 = 1;
    end
    trafo.n_coil = trafo.u1_coil/trafo.u2_coil;                             % coil turns ratio N1/N2

    % magnetizing current of the primary coil and in pu of the rated current
    i1_coil_nom = trafo.i1_nom/sqrt(k1);                                    % [A] delta coil: line current / sqrt(3)
    if isfield(t, 'i1m') && ~isempty(t.i1m)
        trafo.i1m = t.i1m;
    else
        trafo.i1m = t.i0_pu*i1_coil_nom;
    end
    trafo.i0_pu = trafo.i1m/i1_coil_nom;

    % leakage from the short-circuit voltage: star equivalent, primary side, half per winding; copper losses
    % (1 - eta)*pwr_nom, half per winding; the secondary coil is referred with the line ratio
    trafo.Lsigma = t.u1_nom*t.ucc/100/sqrt(3)/trafo.i1_nom/omega;          % [H] total, star equivalent
    trafo.Ld1 = k1*trafo.Lsigma/2;                                          % [H] primary coil
    trafo.Ld2 = k2*trafo.Lsigma/2/trafo.n12^2;                              % [H] secondary coil
    Rd_star = 0.5*(1 - t.eta/100)*t.pwr_nom/3/trafo.i1_nom^2;              % [Ohm] per winding, star equivalent
    trafo.Rd1 = k1*Rd_star;                                                 % [Ohm] primary coil
    trafo.Rd2 = k2*Rd_star/trafo.n12^2;                                     % [Ohm] secondary coil

    % magnetizing branch of the primary coil
    trafo.Lm1 = trafo.u1_coil/trafo.i1m/omega;                              % [H]
    trafo.Lm2 = trafo.Lm1/trafo.n_coil^2;                                   % [H] seen from the secondary coil
    trafo.Rfe1 = trafo.u1_coil^2/(t.p_iron/3);                              % [Ohm] per limb, primary coil
    trafo.psi = sqrt(2)*trafo.u1_coil/omega;                                % [Vs] peak flux linkage of the primary coil

    % coils and core of the saturable single-phase transformer blocks
    trafo.mu0 = 4*pi*1e-7;
    if all(isfield(t, {'n1', 'n2', 'core_area', 'core_length', 'mur'}))
        trafo.n1 = t.n1;
        trafo.n2 = t.n2;
        trafo.core_area = t.core_area;
        trafo.core_length = t.core_length;
        trafo.mur = t.mur;
        trafo.Lm1_geometry = t.n1^2*trafo.mu0*t.mur*t.core_area/t.core_length;  % [H] what the saturable blocks use
        trafo.Bc_peak = trafo.psi/t.n1/t.core_area;                         % [T] peak flux density at rated voltage
    else
        trafo.n1 = NaN;  trafo.n2 = NaN;  trafo.core_area = NaN;  trafo.core_length = NaN;  trafo.mur = NaN;
        trafo.Lm1_geometry = NaN;  trafo.Bc_peak = NaN;
    end
    if isfield(t, 'enable_sat')
        trafo.enable_sat = t.enable_sat;
    else
        trafo.enable_sat = 0;
    end

    % per-unit data set of the Simscape language blocks (+three_phase_transformer of the library)
    pu.Sr = t.pwr_nom;                                                      % [VA]
    pu.V1 = t.u1_nom;                                                       % [V] line-to-line
    pu.V2 = t.u2_nom;                                                       % [V] line-to-line
    pu.fr = t.f_nom;                                                        % [Hz]
    pu.r1 = (1 - t.eta/100)/2;                                              % [pu] half of the copper losses
    pu.r2 = pu.r1;
    pu.l1 = t.ucc/100/2;                                                    % [pu] half of the leakage
    pu.l2 = pu.l1;
    pu.lm = 1/trafo.i0_pu;                                                  % [pu] rated current / magnetizing current
    if isfield(t, 'lm0_pu') && ~isempty(t.lm0_pu)
        pu.lm0 = t.lm0_pu;                                                  % [pu] three-limb core
    else
        pu.lm0 = pu.lm;                                                     % bank of single-phase units
    end
    pu.rfe = t.pwr_nom/t.p_iron;                                            % [pu]
    pu.clock = clock;
    pu.block = block;
    trafo.pu = pu;

    if isempty(block)
        block_label = '';
    else
        block_label = [', library block ', block];
    end
    fprintf('Three phase transformer: %s (%s%s), %.0f kVA, %.0f/%.0f V, %.0f Hz\n', t.name, t.connection, ...
        block_label, t.pwr_nom/1e3, t.u1_nom, t.u2_nom, t.f_nom);
    fprintf('Primary coil: Rd1 = %.2f mOhm | Ld1 = %.1f uH | Lm1 = %.4f H (geometry %.4f H) | Rfe1 = %.0f Ohm | psi = %.2f Vs (%.2f T)\n', ...
        trafo.Rd1*1e3, trafo.Ld1*1e6, trafo.Lm1, trafo.Lm1_geometry, trafo.Rfe1, trafo.psi, trafo.Bc_peak);
    fprintf('Secondary coil: Rd2 = %.2f mOhm | Ld2 = %.1f uH | coil ratio N1/N2 = %.3f (turns %g/%g)\n', ...
        trafo.Rd2*1e3, trafo.Ld2*1e6, trafo.n_coil, trafo.n1, trafo.n2);
    fprintf('Per unit: r = %.4f | l = %.4f (each winding) | lm = %.1f | lm0 = %.1f | rfe = %.0f | clock %d\n', ...
        pu.r1, pu.l1, pu.lm, pu.lm0, pu.rfe, pu.clock);
    fprintf('---------------------------\n');
end
