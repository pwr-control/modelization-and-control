function emu = grid_emulator_param(g)
%GRID_EMULATOR_PARAM  Data of the three-phase grid emulator of ps_25V_12kA_r.slx: block
%   three_phase_voltage_source_with_frt_emulator1 (library three_phase_voltage_with_frt_emulator_mfcn, MATLAB
%   Function), three controlled voltage sources, then the Dyn11 transformer 3ph_Dyg_ssc/configurable_trafo
%   (library, one delta primary, star secondary with the star point grounded, three-limb core).
%
%   emu = grid_emulator_param(g), with g = param.grid:
%     g.application_voltage : [V] rms line-to-line (690, 480, 400): primary and secondary of the transformer
%     g.f_nom               : [Hz]
%     g.pwr_nom             : [VA] rated power of the transformer
%     g.ucc, g.eta, g.p_iron: [%] short-circuit voltage, [%] efficiency, [W] iron losses of the transformer
%     g.n1, g.n2            : turns of the primary (delta) and secondary (star) coils (legacy n2 = 14,
%                             n1 = floor(n2*sqrt(3)))
%     g.core_area, g.core_length, g.mur : [m^2], [m], [-] core: magnetizing inductance of the primary coil
%                             Lm1 = n1^2*mu0*mur*core_area/core_length and line no-load current i1m, as the
%                             legacy init
%     g.lm0_pu              : [pu] zero-sequence magnetizing inductance of the three-limb core (lm0 of the
%                             block; lm0 = lm would be a bank of three single-phase units)
%
%   Local replacement of grid_three_phase_emulator of the library (object with the transformer class
%   three_phase_transformer_setup): same numbers and the same field names where the model reads them.
%   Fields read by ps_25V_12kA_r.slx (param.grid.emu.*):
%     emulator    fgrid_nom, us_secondary_nom, Vemu_ref_norm, up_xi_pu_ref, up_eta_pu_ref, un_xi_pu_ref,
%                 un_eta_pu_ref (its fault data are param.frt.data, its filter param.ctrl.filters.afe.SOF_50Hz)
%     transformer trafo.pwr_nom (Sr), trafo.u1_nom (V1), trafo.u2_nom (V3), trafo.f_nom (fr) and the per-unit
%                 data trafo.pu.r1, l1, r3, l3, lm, lm0, rfe, clock3
%     AFE control omega_grid_nom; ugrid_bez and igrid_bez are read by instantaneus_power (commented out in
%                 the model)
%
%   Bases: ugrid_bez = application_voltage*sqrt(2/3) is the peak phase-to-neutral voltage (391.9 V at
%   480 V). igrid_bez is the fixed value of the library class (360 A rms -> 509.1 A peak at 480 V and 400 V,
%   270 A rms at 690 V), not the current of the transformer (i1bez = 2977 A peak at 1750 kVA) nor the
%   current base of the converter (ibez = 614.2 A at 325 kW).
%   The model had an equivalent grid impedance between the sources and the transformer (17.5 uH + 2 mOhm,
%   blocks eq_grid_impedance_1..3) up to November 2026; the new grid emulator has none, so the grid side of
%   the AFE filter is the transformer leakage only.

    V = g.application_voltage;
    emu.name = sprintf('grid_%dV_%dkVA_Dyn11', V, round(g.pwr_nom/1e3));
    emu.application_voltage = V;
    emu.delta_star = 1;                                     % Dyn transformer (1), Yyn (0)
    emu.us_primary_nom = V;                                 % [V] rms line-to-line, primary and secondary
    emu.us_secondary_nom = V;
    emu.us_nom = V;
    emu.fgrid_nom = g.f_nom;                                % [Hz]
    emu.omega_grid_nom = 2*pi*g.f_nom;                      % [rad/s]
    emu.pwr_nom = g.pwr_nom;                                % [VA]
    emu.is_primary_nom = g.pwr_nom/sqrt(3)/V;               % [A] rms phase current at pwr_nom
    emu.is_secondary_nom = emu.is_primary_nom;
    emu.is_nom = emu.is_primary_nom;

    % bases of the transformer (primary and secondary, same voltage)
    emu.u1bez = V*sqrt(2/3);                                % [V] peak phase-to-neutral
    emu.i1bez = emu.is_primary_nom*sqrt(2);                 % [A] peak
    emu.u2bez = V*sqrt(2/3);
    emu.i2bez = emu.is_secondary_nom*sqrt(2);

    % normalization of the grid quantities in the model (fixed values of the library class)
    switch V
        case 690
            emu.igrid_bez = 270*sqrt(2);                    % [A] peak
            emu.udc_nom = 1070;                             % [V] (informative)
        case 480
            emu.igrid_bez = 360*sqrt(2);
            emu.udc_nom = 750;
        otherwise
            emu.igrid_bez = 360*sqrt(2);
            emu.udc_nom = 680;
    end
    emu.ugrid_bez = V*sqrt(2/3);                            % [V] peak phase-to-neutral

    % voltage reference of the emulator: 1 pu, positive sequence on xi, no negative sequence
    emu.ugrid_factor = 1;
    emu.Vemu_ref = V/sqrt(3)*emu.ugrid_factor;              % [V] rms phase-to-neutral
    emu.Vemu_ref_norm = emu.Vemu_ref*sqrt(2)/emu.ugrid_bez; % [pu] = 1
    emu.up_xi_pu_ref = 1;
    emu.up_eta_pu_ref = 0;
    emu.un_xi_pu_ref = 0;
    emu.un_eta_pu_ref = 0;

    % transformer Dyn11 of the emulator (block configurable_trafo)
    emu.trafo = three_phase_transformer_param(g, emu.delta_star);

    fprintf('Grid emulator: %s, %.0f Hz, ugrid_bez = %.1f V, igrid_bez = %.1f A | no grid impedance: grid side = transformer leakage %.1f uH (star equivalent)\n', ...
        emu.name, g.f_nom, emu.ugrid_bez, emu.igrid_bez, emu.trafo.Lsigma*1e6);
    fprintf('---------------------------\n');
end

function trafo = three_phase_transformer_param(g, delta_star)
% Same formulas and field names as the library class three_phase_transformer_setup (Dyn11: delta_star = 1,
% clock 11). SI data of one coil: leakage Lsigma from ucc and copper losses (1 - eta)*pwr_nom split half per
% winding, both star equivalents on the primary side; a delta primary coil (k1 = 3) sees the line voltage
% and carries the line current/sqrt(3), so Ld1 = 3*Lsigma/2 and Rd1 = 3*Rd_star (star coil: k1 = 1),
% Ld2 = Lsigma/2/n12^2 and Rd2 = Rd_star/n12^2 for the star secondary. Magnetizing branch of the primary coil
% at its own voltage: Lm1 from the core, Rfe1 from the iron losses per limb, psi the peak flux linkage.
% Per unit (struct pu, read by configurable_trafo): coil bases of the Simscape components, Zcoil1 = k1*Zbase1
% (Zbase1 = u1_nom^2/pwr_nom), Zcoil2 = Zbase2, so that r1 = r3 = (1 - eta/100)/2, l1 = l3 = ucc/200,
% lm = i1_nom/i1m, rfe = pwr_nom/p_iron; lm0 = g.lm0_pu.
    V = g.application_voltage;
    omega = 2*pi*g.f_nom;
    mu0 = 4*pi*1e-7;
    Lm1_core = g.n1^2*mu0*g.mur*g.core_area/g.core_length;  % [H] magnetizing inductance of the core

    trafo.name = sprintf('Dyn11_%dV_%dV_%dkVA', V, V, round(g.pwr_nom/1e3));
    trafo.vector_group = 'Dyn11';
    trafo.clock = 11;                                       % the secondary lags the primary by 11*30 deg
    trafo.delta_star = delta_star;
    trafo.pwr_nom = g.pwr_nom;                              % [VA]
    trafo.u1_nom = V;                                       % [V] rms line-to-line
    trafo.i1_nom = g.pwr_nom/sqrt(3)/V;                     % [A] rms
    trafo.u2_nom = V;
    trafo.i2_nom = trafo.i1_nom;
    trafo.f_nom = g.f_nom;
    trafo.eta = g.eta;                                      % [%]
    trafo.ucc = g.ucc;                                      % [%]
    if delta_star == 1
        k1 = 3;  trafo.u1_coil = V;                         % delta primary coil: line voltage, line current/sqrt(3)
    else
        k1 = 1;  trafo.u1_coil = V/sqrt(3);                 % star primary coil: phase voltage, line current
    end
    trafo.i1m_coil = trafo.u1_coil/Lm1_core/omega;          % [A] magnetizing current of the primary coil
    trafo.i1m = sqrt(k1)*trafo.i1m_coil;                    % [A] line no-load current
    trafo.n12 = trafo.u1_nom/trafo.u2_nom;
    trafo.n1 = g.n1;
    trafo.n2 = g.n2;
    trafo.Lsigma = trafo.u1_nom*trafo.ucc/100/sqrt(3)/trafo.i1_nom/omega;   % [H] star equivalent, primary side
    Rd_star = 0.5*((1 - trafo.eta/100)*trafo.pwr_nom/3/trafo.i1_nom^2);     % [Ohm] per winding, star equivalent
    trafo.Ld1 = k1*trafo.Lsigma/2;                          % [H] primary coil
    trafo.Rd1 = k1*Rd_star;                                 % [Ohm] primary coil
    trafo.Lm1 = trafo.u1_coil/trafo.i1m_coil/omega;         % [H] = Lm1_core
    trafo.Ld2 = trafo.Lsigma/2/trafo.n12^2;                 % [H] secondary coil (star)
    trafo.Rd2 = Rd_star/trafo.n12^2;                        % [Ohm]
    trafo.Lm2 = trafo.Lm1/(trafo.u1_coil/(trafo.u2_nom/sqrt(3)))^2;         % [H] seen from the secondary coil
    trafo.p_iron = g.p_iron;                                % [W] total of the three limbs
    trafo.Rfe1 = trafo.u1_coil^2/(trafo.p_iron/3);          % [Ohm] primary coil
    trafo.psi = sqrt(2)*trafo.u1_coil/omega;                % [Vs] peak flux linkage of the primary coil
    trafo.core_area = g.core_area;                          % [m^2]
    trafo.core_length = g.core_length;                      % [m]
    trafo.mu0 = mu0;
    trafo.mur = g.mur;

    % per unit on the coil bases (as the Simscape components of the library)
    trafo.Zbase1 = trafo.u1_nom^2/trafo.pwr_nom;            % [Ohm] line quantities, primary side
    trafo.Zbase2 = trafo.u2_nom^2/trafo.pwr_nom;            % [Ohm] secondary side
    trafo.Zcoil1 = k1*trafo.Zbase1;                         % [Ohm] primary coil: 3*Zbase1 for delta
    trafo.Zcoil2 = trafo.Zbase2;                            % [Ohm] secondary coil (star)
    trafo.Lsigma_pu = omega*trafo.Lsigma/trafo.Zbase1;      % = ucc/100
    trafo.Rd1_pu = trafo.Rd1/trafo.Zcoil1;                  % = (1 - eta/100)/2
    trafo.Ld1_pu = omega*trafo.Ld1/trafo.Zcoil1;            % = ucc/200
    trafo.Lm1_pu = omega*trafo.Lm1/trafo.Zcoil1;            % = i1_nom/i1m
    trafo.Rfe1_pu = trafo.Rfe1/trafo.Zcoil1;                % = pwr_nom/p_iron
    trafo.Rd2_pu = trafo.Rd2/trafo.Zcoil2;                  % = (1 - eta/100)/2
    trafo.Ld2_pu = omega*trafo.Ld2/trafo.Zcoil2;            % = ucc/200
    trafo.i1m_pu = trafo.i1m/trafo.i1_nom;                  % = 1/Lm1_pu
    trafo.lm0_pu = g.lm0_pu;                                % zero-sequence magnetizing inductance
    trafo.pu = struct('Sr', trafo.pwr_nom, 'V1', trafo.u1_nom, 'V3', trafo.u2_nom, 'fr', trafo.f_nom, ...
        'r1', trafo.Rd1_pu, 'l1', trafo.Ld1_pu, 'r3', trafo.Rd2_pu, 'l3', trafo.Ld2_pu, ...
        'lm', trafo.Lm1_pu, 'lm0', trafo.lm0_pu, 'rfe', trafo.Rfe1_pu, 'clock3', trafo.clock);

    fprintf('Transformer %s: Ld1 = %.1f uH, Rd1 = %.2f mOhm, Ld2 = %.1f uH, Rd2 = %.2f mOhm, Lm1 = %.1f mH (i1m = %.1f A), Rfe1 = %.0f Ohm\n', ...
        trafo.name, trafo.Ld1*1e6, trafo.Rd1*1e3, trafo.Ld2*1e6, trafo.Rd2*1e3, trafo.Lm1*1e3, trafo.i1m, trafo.Rfe1);
    fprintf('Per unit (coil bases %.4f / %.4f Ohm): r1 = %.4f, l1 = %.4f, r3 = %.4f, l3 = %.4f, lm = %.1f, lm0 = %.1f, rfe = %.0f, clock %d\n', ...
        trafo.Zcoil1, trafo.Zcoil2, trafo.pu.r1, trafo.pu.l1, trafo.pu.r3, trafo.pu.l3, trafo.pu.lm, trafo.pu.lm0, ...
        trafo.pu.rfe, trafo.pu.clock3);
end
