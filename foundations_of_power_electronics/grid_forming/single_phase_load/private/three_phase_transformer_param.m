function trafo = three_phase_transformer_param(name, delta_star, pwr_nom, u1_nom, u2_nom, f_nom, eta, ucc, ...
    i1m, p_iron, n1, n2, core_area, core_length, mur)
%THREE_PHASE_TRANSFORMER_PARAM  Equivalent circuit of the coupling transformer (local replacement of
%   three_phase_transformer_setup of the library, same formulas and field names).
%
%   delta_star : 1 for Dyn (delta primary), 0 for Yyn
%   pwr_nom    : [W]      u1_nom, u2_nom : [V] line-to-line    f_nom : [Hz]
%   eta        : [%] efficiency (copper losses)    ucc : [%] short-circuit voltage
%   i1m        : [A] primary line no-load (magnetizing) current   p_iron : [W] iron losses, total
%   n1, n2     : turns    core_area [m^2], core_length [m], mur : core geometry (flux and Lm check)
%
%   Fields read by grid_forming_dq.slx (block grid_formed/grid_emulator/3ph_Dyg, three single-phase
%   transformers): Lm1, n1, n2, Rd1, Rd2, Ld1, Ld2, Rfe1. They are the data of one coil: a delta primary
%   coil (delta_star = 1) sees the line voltage and carries the line current/sqrt(3), so its resistance and
%   leakage are 3 x the star-equivalent values and its magnetizing branch is referred to the line voltage.

    trafo.name = name;
    trafo.delta_star = delta_star;
    trafo.pwr_nom = pwr_nom;
    trafo.u1_nom = u1_nom;
    trafo.i1_nom = pwr_nom/sqrt(3)/u1_nom;
    trafo.u2_nom = u2_nom;
    trafo.i2_nom = pwr_nom/sqrt(3)/u2_nom;
    trafo.f_nom = f_nom;
    trafo.ucc = ucc;
    trafo.eta = eta;
    trafo.i1m = i1m;
    trafo.n12 = u1_nom/u2_nom;
    trafo.n1 = n1;
    trafo.n2 = n2;

    % primary coil: delta (k1 = 3) at the line voltage, star (k1 = 1) at the phase voltage
    omega = 2*pi*f_nom;
    if delta_star == 1
        k1 = 3;  trafo.u1_coil = u1_nom;
    else
        k1 = 1;  trafo.u1_coil = u1_nom/sqrt(3);
    end
    trafo.i1m_coil = i1m/sqrt(k1);                                          % [A] magnetizing current of the coil

    % leakage from the short-circuit voltage and copper losses, half per winding, star equivalents
    trafo.Lsigma = u1_nom*ucc/100/sqrt(3)/trafo.i1_nom/omega;               % [H] total, primary side
    Rd_star = 0.5*((1 - eta/100)*pwr_nom/3/trafo.i1_nom^2);                 % [Ohm] per winding
    trafo.Ld1 = k1*trafo.Lsigma/2;                                          % [H] primary coil
    trafo.Rd1 = k1*Rd_star;                                                 % [Ohm] primary coil
    trafo.Lm1 = trafo.u1_coil/trafo.i1m_coil/omega;                         % [H] magnetizing, primary coil
    trafo.Ld2 = trafo.Lsigma/2/trafo.n12^2;                                 % [H] secondary coil (star)
    trafo.Rd2 = Rd_star/trafo.n12^2;                                        % [Ohm]
    trafo.Lm2 = trafo.Lm1/(trafo.u1_coil/(u2_nom/sqrt(3)))^2;               % [H] seen from the secondary coil
    trafo.p_iron = p_iron;
    trafo.Rfe1 = trafo.u1_coil^2/(p_iron/3);                                % [Ohm] iron losses per limb, primary coil
    trafo.psi = sqrt(2)*trafo.u1_coil/omega;                                % [Vs] peak flux linkage of the primary coil
    trafo.core_area = core_area;
    trafo.core_length = core_length;
    trafo.mu0 = 4*pi*1e-7;
    trafo.mur = mur;

    fprintf('Three Phase Transformer: %s\n', name);
    fprintf('Primary Side Leakage Inductance: %.2e H | Secondary Side Leakage Inductance: %.2e H\n', trafo.Ld1, trafo.Ld2);
    fprintf('Primary Side Magnetization Inductance: %.2e H | Rfe1 = %.1f Ohm\n', trafo.Lm1, trafo.Rfe1);
    fprintf('---------------------------\n');
end
