function emu = grid_emulator_param(g, fault)
%GRID_EMULATOR_PARAM  Data of the three-phase grid emulator of afe_dq_ttype.slx (block
%   three_phase_voltage_source_with_frt_emulator of the library feeding the Dyn11 transformer 3ph_Dyg made
%   of three single-phase transformer blocks, star point of the secondary grounded) and of its fault
%   generator.
%
%   emu = grid_emulator_param(g, fault), with g = param.grid:
%     g.application_voltage : [V] rms line-to-line (690, 480, 400): primary and secondary of the transformer
%     g.f_nom               : [Hz]
%     g.pwr_nom             : [VA] rated power of the transformer
%     g.ucc, g.eta, g.p_iron: [%] short-circuit voltage, [%] efficiency, [W] iron losses of the transformer
%     g.n1, g.n2            : turns of the single-phase transformer blocks (legacy n2 = 14, n1 = floor(n2*sqrt(3)))
%     g.core_area, g.core_length, g.mur : [m^2], [m], [-] core of the blocks: they give the magnetizing
%                             inductance Lm1 = n1^2*mu0*mur*core_area/core_length and the magnetizing
%                             current i1m, as the legacy init
%     g.R_eq                : [Ohm] equivalent grid resistance (informative, not in the model)
%     fault                 : struct of the fault test (see grid_fault_param), stored in emu.frt
%
%   Local replacement of grid_three_phase_emulator (object with the transformer class
%   three_phase_transformer_setup) + frt_settings + grid_fault_generator + the SOF_50Hz entry of
%   setup_global_filters: same numbers and the same field names where the model reads them. Fields read
%   by afe_dq_ttype.slx:
%     fgrid_nom, us_primary_nom, omega_grid_nom, Vemu_ref_norm, up_xi_pu_ref, up_eta_pu_ref, un_xi_pu_ref,
%     un_eta_pu_ref, ugrid_bez, igrid_bez, sof_50Hz.A, sof_50Hz.B, sof_50Hz.C, trafo.Lm1, trafo.n1,
%     trafo.n2, trafo.Rd1, trafo.Rd2, trafo.Ld1, trafo.Ld2, trafo.Rfe1, frt.* (grid_fault_param)
%
%   Bases: ugrid_bez = application_voltage*sqrt(2/3) is the peak phase-to-neutral voltage (563.4 V at
%   690 V). igrid_bez is the fixed value of the library class (270 A rms -> 381.8 A peak at 690 V,
%   360 A rms at 480 V and 400 V), not the current of the transformer (i2bez = 1183 A peak at 1 MVA) nor
%   the current base of the converter (ibez = 328.7 A at 250 kW): the model multiplies by igrid_bez the
%   grid currents normalized with ibez in instantaneus_power (power-meter scope only, see the README).

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

    % voltage reference of the emulator: 1 pu of ugrid_bez, positive sequence on xi, no negative sequence
    emu.ugrid_factor = 1;
    emu.Vemu_ref = V/sqrt(3)*emu.ugrid_factor;              % [V] rms phase-to-neutral
    emu.Vemu_ref_norm = emu.Vemu_ref*sqrt(2)/emu.ugrid_bez; % [pu] = 1
    emu.up_xi_pu_ref = 1;
    emu.up_eta_pu_ref = 0;
    emu.un_xi_pu_ref = 0;
    emu.un_eta_pu_ref = 0;

    % equivalent grid impedance (informative: the emulator sources are ideal, the transformer is the only
    % impedance between the sources and the converter filter)
    emu.ucc = g.ucc;                                                            % [%]
    emu.L_eq = V/sqrt(3)*g.ucc/100/emu.omega_grid_nom/emu.is_nom;               % [H] legacy Lgrid_base
    emu.R_eq = g.R_eq;                                                          % [Ohm]

    % transformer Dyn11 of the emulator (coil data of the three single-phase transformer blocks)
    emu.trafo = three_phase_transformer_param(g, emu.delta_star);

    % second-order filter 1/(tau*s + 1)^2 at 50 Hz on the amplitude ramps of the emulator (legacy
    % filters.afe.SOF_50Hz.ss_c): controllable canonical realization, continuous time
    tau = 1/(2*pi*50);
    emu.sof_50Hz.fcut = 50;                                 % [Hz]
    emu.sof_50Hz.tau = tau;                                 % [s]
    emu.sof_50Hz.A = [-2/tau, -1/tau^2; 1, 0];
    emu.sof_50Hz.B = [1; 0];
    emu.sof_50Hz.C = [0, 1/tau^2];
    emu.sof_50Hz.D = 0;

    % fault test of the emulator
    emu.frt = fault;

    fprintf('Grid emulator: %s, %.0f Hz, ugrid_bez = %.1f V, igrid_bez = %.1f A, L_eq = %.0f uH (ucc %.0f %%, the transformer leakage is in the model)\n', ...
        emu.name, g.f_nom, emu.ugrid_bez, emu.igrid_bez, emu.L_eq*1e6, g.ucc);
    fprintf('Fault test: %s, enable_frt_1 = %d, enable_frt_2 = %d, amplitudes [%.3f %.3f %.3f], phase shift %.3f rad, from %.3f s to %.3f s\n', ...
        fault.kind, fault.enable_frt_1, fault.enable_frt_2, fault.Amplitude_U, fault.Amplitude_V, fault.Amplitude_W, ...
        fault.Phase_shift, fault.start_time_LVRT, fault.end_time_LVRT);
    fprintf('---------------------------\n');
end

function trafo = three_phase_transformer_param(g, delta_star)
% Same formulas and field names as the library class three_phase_transformer_setup (Dyn: delta_star = 1):
% leakage Lsigma from ucc on the primary, Ld1 = 3/2*Lsigma for a Dy transformer (1/2 for Yy), Ld2 =
% Lsigma/2/n12^2, copper losses (1 - eta)*pwr_nom split half per winding, Lm1 from the magnetizing current
% at the phase voltage, Rfe1 from the iron losses per phase. The magnetizing current comes from the core
% geometry as in the legacy init: Lm1 = n1^2*mu0*mur*core_area/core_length, i1m = u1/sqrt(3)/Lm1/omega.
    V = g.application_voltage;
    omega = 2*pi*g.f_nom;
    mu0 = 4*pi*1e-7;
    Lm1_core = g.n1^2*mu0*g.mur*g.core_area/g.core_length;  % [H] magnetizing inductance of the core

    trafo.name = sprintf('Dyn11_%dV_%dV_%dkVA', V, V, round(g.pwr_nom/1e3));
    trafo.delta_star = delta_star;
    trafo.pwr_nom = g.pwr_nom;                              % [VA]
    trafo.u1_nom = V;                                       % [V] rms line-to-line
    trafo.i1_nom = g.pwr_nom/sqrt(3)/V;                     % [A] rms
    trafo.u2_nom = V;
    trafo.i2_nom = trafo.i1_nom;
    trafo.f_nom = g.f_nom;
    trafo.eta = g.eta;                                      % [%]
    trafo.ucc = g.ucc;                                      % [%]
    trafo.i1m = V/sqrt(3)/Lm1_core/omega;                   % [A] magnetizing current at the phase voltage
    trafo.n12 = trafo.u1_nom/trafo.u2_nom;
    trafo.n1 = g.n1;
    trafo.n2 = g.n2;
    trafo.Lsigma = trafo.u1_nom*trafo.ucc/100/sqrt(3)/trafo.i1_nom/omega;   % [H]
    if delta_star == 1
        trafo.Ld1 = 3/2*trafo.Lsigma;                       % [H] Dy transformer
    else
        trafo.Ld1 = 1/2*trafo.Lsigma;                       % [H] Yy transformer
    end
    trafo.Rd1 = 0.5*((1 - trafo.eta/100)*trafo.pwr_nom/3/trafo.i1_nom^2);  % [Ohm]
    trafo.Lm1 = trafo.u1_nom/sqrt(3)/trafo.i1m/omega;       % [H] = Lm1_core
    trafo.Ld2 = trafo.Lsigma/2/trafo.n12^2;                 % [H]
    trafo.Rd2 = trafo.Rd1/trafo.n12^2;                      % [Ohm]
    trafo.Lm2 = trafo.Lm1/trafo.n12^2;                      % [H]
    trafo.p_iron = g.p_iron;                                % [W]
    trafo.Rfe1 = (trafo.u1_nom/sqrt(3))^2/(trafo.p_iron/3); % [Ohm]
    trafo.psi = trafo.Lm1*trafo.i1m*sqrt(2);                % [Vs]
    trafo.core_area = g.core_area;                          % [m^2]
    trafo.core_length = g.core_length;                      % [m]
    trafo.mu0 = mu0;
    trafo.mur = g.mur;

    fprintf('Transformer %s: Ld1 = %.1f uH, Rd1 = %.1f mOhm, Ld2 = %.1f uH, Rd2 = %.1f mOhm, Lm1 = %.1f mH (i1m = %.1f A), Rfe1 = %.0f Ohm, n1/n2 = %d/%d\n', ...
        trafo.name, trafo.Ld1*1e6, trafo.Rd1*1e3, trafo.Ld2*1e6, trafo.Rd2*1e3, trafo.Lm1*1e3, trafo.i1m, trafo.Rfe1, trafo.n1, trafo.n2);
end
