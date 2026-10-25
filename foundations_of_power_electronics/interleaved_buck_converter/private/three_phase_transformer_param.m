function trafo = three_phase_transformer_param(name, delta_star, pwr_nom, u1_nom, u2_nom, f_nom, eta, ucc, ...
    i1m, p_iron, n1, n2, core_area, core_length, mur)
%THREE_PHASE_TRANSFORMER_PARAM  Equivalent circuit of the coupling transformer (local replacement of
%   three_phase_transformer_setup of the library, same formulas and field names).
%
%   delta_star : 1 for Dyn (delta primary), 0 for Yyn
%   pwr_nom    : [W]      u1_nom, u2_nom : [V] line-to-line    f_nom : [Hz]
%   eta        : [%] efficiency (copper losses)    ucc : [%] short-circuit voltage
%   i1m        : [A] primary magnetizing current   p_iron : [W] iron losses
%   n1, n2     : turns    core_area [m^2], core_length [m], mur : core geometry (flux and Lm check)
%
%   Fields read by grid_forming_dq.slx (block grid_formed/grid_emulator/3ph_Dyg, three single-phase
%   transformers): Lm1, n1, n2, Rd1, Rd2, Ld1, Ld2, Rfe1.

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

    % leakage from the short-circuit voltage, split between the two windings
    trafo.Lsigma = u1_nom*ucc/100/sqrt(3)/trafo.i1_nom/(2*pi*f_nom);        % [H] total, primary side
    if delta_star == 1
        trafo.Ld1 = 3/2*trafo.Lsigma;                                       % [H] Dy: delta winding
    else
        trafo.Ld1 = 1/2*trafo.Lsigma;                                       % [H] Yy
    end
    trafo.Rd1 = 0.5*((1 - eta/100)*pwr_nom/3/trafo.i1_nom^2);              % [Ohm] copper losses, half per winding
    trafo.Lm1 = u1_nom/sqrt(3)/i1m/(2*pi*f_nom);                           % [H] magnetizing, primary side
    trafo.Ld2 = trafo.Lsigma/2/trafo.n12^2;                                 % [H]
    trafo.Rd2 = trafo.Rd1/trafo.n12^2;                                      % [Ohm]
    trafo.Lm2 = trafo.Lm1/trafo.n12^2;                                      % [H]
    trafo.p_iron = p_iron;
    trafo.Rfe1 = (u1_nom/sqrt(3))^2/(p_iron/3);                             % [Ohm] iron losses, primary side
    trafo.psi = trafo.Lm1*i1m*sqrt(2);                                      % [Vs] core flux
    trafo.core_area = core_area;
    trafo.core_length = core_length;
    trafo.mu0 = 4*pi*1e-7;
    trafo.mur = mur;

    fprintf('Three Phase Transformer: %s\n', name);
    fprintf('Primary Side Leakage Inductance: %.2e H | Secondary Side Leakage Inductance: %.2e H\n', trafo.Ld1, trafo.Ld2);
    fprintf('Primary Side Magnetization Inductance: %.2e H | Rfe1 = %.1f Ohm\n', trafo.Lm1, trafo.Rfe1);
    fprintf('---------------------------\n');
end
