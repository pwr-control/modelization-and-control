function trafo = three_phase_transformer_param(name, delta_star, pwr_nom, u1_nom, u2_nom, f_nom, eta, ucc, ...
    i1m, p_iron, n1, n2, core_area, core_length, mur, enable_sat)
%THREE_PHASE_TRANSFORMER_PARAM  Equivalent circuit of the grid emulator transformer (local replacement of
%   three_phase_transformer_setup of the library, same field names, coil-referred magnetizing data).
%
%   trafo = three_phase_transformer_param(name, delta_star, pwr_nom, u1_nom, u2_nom, f_nom, eta, ucc, ...
%               i1m, p_iron, n1, n2, core_area, core_length, mur, enable_sat)
%     delta_star : 1 for Dyn (delta primary, star secondary), 0 for Yyn
%     pwr_nom    : [W]      u1_nom, u2_nom : [V] line-to-line    f_nom : [Hz]
%     eta        : [%] efficiency (copper losses = (1 - eta/100)*pwr_nom, half per winding)
%     ucc        : [%] short-circuit voltage (leakage, half per winding)
%     i1m        : [A] magnetizing current of the PRIMARY COIL at its rated voltage (delta coil: u1_nom,
%                  star coil: u1_nom/sqrt(3)); the magnetizing line current is sqrt(3)*i1m for the delta
%     p_iron     : [W] iron losses, total
%     n1, n2     : turns of the primary and secondary coil of each limb
%     core_area [m^2], core_length [m], mur : core geometry of the saturable single-phase transformer blocks
%     enable_sat : 1 enables the arctangent saturation of the blocks (en_sat), 0 linear
%
%   Fields read by power_system.slx (block grid/grid_emulator/3ph_Dyg: three saturable single-phase
%   transformers, one per limb, primary coils in delta, secondary coils in star with the neutral to ground):
%     n1, n2, Rd1, Rd2, Ld1, Ld2, Rfe1, core_area, core_length, mu0, mur, psi (phi_sat = 1.5*psi), enable_sat
%   and, by the commented variants: Lm1 (ideal_transformer_Dyn).
%   The magnetizing inductance of the active blocks is computed by the blocks themselves from the core
%   geometry: Lm = n1^2*mu0*mur*core_area/core_length (0.145 H with the default data); Lm1 here is the same
%   number when i1m is derived from it (see init_model).
%
%   Differences with the library class (coil-referred data, 2026-10-26): the primary coil of a Dyn is across
%   the LINE voltage, therefore Rfe1 and psi are computed from the coil voltage u1_nom (the library used the
%   phase voltage for both connections: Rfe1 three times too small, i.e. 3*p_iron of losses in the delta,
%   and psi = flux linkage at u1_nom/sqrt(3), so that phi_sat = 1.5*psi was below the rated flux of the
%   delta coil). With enable_sat = 0 (default) the flux limit is not used and only Rfe1 changes.
%
%   trafo.pu: the same transformer in per unit for the Simscape language blocks
%   three_phase_transformer.transformer_Dyn / transformer_Yyn / transformer_Dd of the library (Sr, V1, V2,
%   fr, r1, l1, r2, l2, lm, lm0, rfe, clock): lm0 = lm, a bank of three single-phase units like 3ph_Dyg;
%   set lm0 < lm (0.3 - 1 pu) for a three-limb core.

    omega = 2*pi*f_nom;
    trafo.name = name;
    trafo.delta_star = delta_star;
    trafo.pwr_nom = pwr_nom;
    trafo.u1_nom = u1_nom;
    trafo.i1_nom = pwr_nom/sqrt(3)/u1_nom;                                  % [A] line current
    trafo.u2_nom = u2_nom;
    trafo.i2_nom = pwr_nom/sqrt(3)/u2_nom;
    trafo.f_nom = f_nom;
    trafo.ucc = ucc;
    trafo.eta = eta;
    trafo.i1m = i1m;
    trafo.n12 = u1_nom/u2_nom;                                              % line-to-line ratio
    trafo.n1 = n1;
    trafo.n2 = n2;
    if delta_star == 1
        trafo.u1_coil = u1_nom;                                             % [V] delta coil
        trafo.connection = 'Dyn11';
    else
        trafo.u1_coil = u1_nom/sqrt(3);                                     % [V] star coil
        trafo.connection = 'Yyn0';
    end
    trafo.u2_coil = u2_nom/sqrt(3);                                         % [V] star secondary in both cases

    % leakage from the short-circuit voltage: total, star equivalent, primary side; half per winding
    trafo.Lsigma = u1_nom*ucc/100/sqrt(3)/trafo.i1_nom/omega;              % [H]
    if delta_star == 1
        trafo.Ld1 = 3/2*trafo.Lsigma;                                       % [H] delta coil: 3 x star equivalent
    else
        trafo.Ld1 = 1/2*trafo.Lsigma;                                       % [H] star coil
    end
    trafo.Ld2 = trafo.Lsigma/2/trafo.n12^2;                                 % [H] secondary star coil
    % copper losses (1 - eta)*pwr_nom, half per winding, star equivalent; the delta coil carries i1/sqrt(3)
    % and dissipates the same power with 3 x the star-equivalent resistance
    Rd_star = 0.5*((1 - eta/100)*pwr_nom/3/trafo.i1_nom^2);                % [Ohm]
    if delta_star == 1
        trafo.Rd1 = 3*Rd_star;
    else
        trafo.Rd1 = Rd_star;
    end
    trafo.Rd2 = Rd_star/trafo.n12^2;                                        % [Ohm]
    % magnetizing branch of the primary coil
    trafo.Lm1 = trafo.u1_coil/i1m/omega;                                    % [H]
    trafo.Lm2 = trafo.Lm1*(n2/n1)^2;                                        % [H] seen from the secondary coil
    trafo.p_iron = p_iron;
    trafo.Rfe1 = trafo.u1_coil^2/(p_iron/3);                                % [Ohm] per limb, primary coil
    trafo.psi = sqrt(2)*trafo.u1_coil/omega;                                % [Vs] peak flux linkage of the primary coil
    trafo.core_area = core_area;
    trafo.core_length = core_length;
    trafo.mu0 = 4*pi*1e-7;
    trafo.mur = mur;
    trafo.Lm1_geometry = n1^2*trafo.mu0*mur*core_area/core_length;          % [H] what the saturable blocks use
    trafo.Bc_peak = trafo.psi/n1/core_area;                                 % [T] peak flux density at rated voltage
    trafo.enable_sat = enable_sat;

    % per-unit data set for the Simscape language blocks of the library (+three_phase_transformer)
    pu.Sr = pwr_nom;                                                        % [VA]
    pu.V1 = u1_nom;                                                         % [V] line-to-line
    pu.V2 = u2_nom;                                                         % [V] line-to-line
    pu.fr = f_nom;                                                          % [Hz]
    pu.r1 = (1 - eta/100)/2;                                                % [pu] half of the copper losses
    pu.r2 = pu.r1;
    pu.l1 = ucc/100/2;                                                      % [pu] half of the leakage
    pu.l2 = pu.l1;
    if delta_star == 1
        pu.lm = trafo.i1_nom/(sqrt(3)*i1m);                                 % [pu] rated current / magnetizing line current
        pu.clock = 11;
    else
        pu.lm = trafo.i1_nom/i1m;
        pu.clock = 0;
    end
    pu.lm0 = pu.lm;                                                         % bank of single-phase units (3ph_Dyg)
    pu.rfe = pwr_nom/p_iron;                                                % [pu]
    trafo.pu = pu;

    fprintf('Three Phase Transformer: %s (%s), %.0f kVA, %.0f/%.0f V, %.0f Hz\n', name, trafo.connection, ...
        pwr_nom/1e3, u1_nom, u2_nom, f_nom);
    fprintf('Primary coil: Rd1 = %.2f mOhm | Ld1 = %.1f uH | Lm1 = %.4f H (geometry %.4f H) | Rfe1 = %.0f Ohm | psi = %.2f Vs (%.2f T)\n', ...
        trafo.Rd1*1e3, trafo.Ld1*1e6, trafo.Lm1, trafo.Lm1_geometry, trafo.Rfe1, trafo.psi, trafo.Bc_peak);
    fprintf('Secondary coil: Rd2 = %.2f mOhm | Ld2 = %.1f uH | n1/n2 = %d/%d\n', trafo.Rd2*1e3, trafo.Ld2*1e6, n1, n2);
    fprintf('Per unit: r = %.4f | l = %.4f (each winding) | lm = %.1f | rfe = %.0f\n', pu.r1, pu.l1, pu.lm, pu.rfe);
    fprintf('---------------------------\n');
end
