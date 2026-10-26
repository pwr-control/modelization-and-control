function emu = grid_emulator_param(g)
%GRID_EMULATOR_PARAM  Data of the three-phase grid emulator of power_system (local replacement of the
%   library class grid_three_phase_emulator, same field names).
%
%   emu = grid_emulator_param(g), with g = param.grid:
%     g.name, g.connection, g.grid_nominal_power [W], g.application_voltage [V]
%     g.us1, g.us2 [V] transformer rated line-to-line voltages, g.fgrid [Hz]
%     g.eq_grid_inductance [H], g.eq_grid_resistance [Ohm]   series impedance of the voltage sources
%     g.eta, g.ucc, g.i1m, g.p_iron, g.lm0_pu, g.n1, g.n2, g.core_area, g.core_length, g.mur, g.enable_sat
%                                                           transformer data (three_phase_transformer_param)
%     g.up_xi_pu_ref, g.up_eta_pu_ref, g.un_xi_pu_ref, g.un_eta_pu_ref   [pu] positive and negative
%                                                           sequence voltage references of the emulator
%     g.ugrid_factor                                        [pu] amplitude factor of the emulated voltage
%     g.f_sof                                               [Hz] cut-off of the second order filter on the
%                                                           voltage references of the emulator block
%
%   Fields read by power_system.slx (block grid/grid_emulator/three_phase_voltage_source_with_frt_emulator,
%   the eq_grid_impedance_* inductors, the 3ph_Dyg transformer and the IEC power meters):
%     fgrid_nom, omega_grid_nom, us_primary_nom, Vemu_ref_norm, up_xi_pu_ref, up_eta_pu_ref, un_xi_pu_ref,
%     un_eta_pu_ref, eq_grid_inductance, eq_grid_resistance, trafo.*, sof_50Hz.ss_c.A, .B, .C
%
%   The emulator block builds the three phase voltages from the sequence references (pu of the peak phase
%   voltage us_primary_nom*sqrt(2/3)), filters the reference steps of the FRT with the state space
%   (A, B, C) of a second order low-pass 1/(tau*s + 1)^2 and ramps them at Vemu_ref_norm/t_ramp.
%   Not kept from the library class: kp_vgrid, ki_vgrid, k_ff (gains of a closed-loop emulator that this
%   model does not use).

    emu.name = g.name;
    emu.pwr_nom = g.grid_nominal_power;
    emu.application_voltage = g.application_voltage;
    emu.us_primary_nom = g.us1;                                             % [V] line-to-line
    emu.is_primary_nom = g.grid_nominal_power/sqrt(3)/g.us1;                % [A]
    emu.us_secondary_nom = g.us2;
    emu.is_secondary_nom = g.grid_nominal_power/sqrt(3)/g.us2;
    emu.n1 = g.n1;
    emu.n2 = g.n2;
    emu.fgrid_nom = g.fgrid;                                                % [Hz]
    emu.omega_grid_nom = 2*pi*g.fgrid;                                      % [rad/s]
    emu.eq_grid_inductance = g.eq_grid_inductance;                          % [H]
    emu.eq_grid_resistance = g.eq_grid_resistance;                          % [Ohm]

    % transformer of the emulator (block T1 of 3ph_Dyg): coil data and per-unit data from the nameplate
    emu.trafo = three_phase_transformer_param(struct('name', g.name, 'connection', g.connection, ...
        'pwr_nom', g.grid_nominal_power, 'u1_nom', g.us1, 'u2_nom', g.us2, 'f_nom', g.fgrid, 'eta', g.eta, ...
        'ucc', g.ucc, 'i1m', g.i1m, 'p_iron', g.p_iron, 'lm0_pu', g.lm0_pu, 'n1', g.n1, 'n2', g.n2, ...
        'core_area', g.core_area, 'core_length', g.core_length, 'mur', g.mur, 'enable_sat', g.enable_sat));
    emu.delta_star = emu.trafo.delta_star;                                  % legacy flag: 1 delta primary

    % normalization: peak phase voltage and peak phase current of the two sides of the transformer
    emu.u1bez = emu.us_primary_nom*sqrt(2/3);                               % [V]
    emu.i1bez = emu.is_primary_nom*sqrt(2);                                 % [A]
    emu.u2bez = emu.us_secondary_nom*sqrt(2/3);
    emu.i2bez = emu.is_secondary_nom*sqrt(2);

    % sequence references of the emulated voltage
    emu.up_xi_pu_ref = g.up_xi_pu_ref;
    emu.up_eta_pu_ref = g.up_eta_pu_ref;
    emu.un_xi_pu_ref = g.un_xi_pu_ref;
    emu.un_eta_pu_ref = g.un_eta_pu_ref;
    emu.ugrid_factor = g.ugrid_factor;

    % bases of the application (same table as the library: 690 V / 480 V / 400 V converters)
    switch g.application_voltage
        case 690
            emu.igrid_bez = 270*sqrt(2);                                    % [A]
            emu.udc_nom = 1070;                                             % [V]
        case 480
            emu.igrid_bez = 360*sqrt(2);
            emu.udc_nom = 750;
        otherwise
            emu.igrid_bez = 360*sqrt(2);
            emu.udc_nom = 680;
    end
    emu.ugrid_bez = g.application_voltage*sqrt(2/3);                        % [V] peak phase voltage
    emu.Vemu_ref = g.application_voltage/sqrt(3)*g.ugrid_factor;           % [V] rms phase voltage reference
    emu.Vemu_ref_norm = emu.Vemu_ref*sqrt(2)/emu.ugrid_bez;                 % [pu] = ugrid_factor

    % second order low-pass of the voltage references, continuous state space (gains Am, Bm, Cm and an
    % Integrator inside the emulator block): x' = A x + B u, y = C x, H(s) = 1/(tau s + 1)^2
    tau = 1/(2*pi*g.f_sof);
    emu.sof_50Hz.fcut = g.f_sof;                                            % [Hz]
    emu.sof_50Hz.time_constant = tau;                                       % [s]
    emu.sof_50Hz.ss_c.A = [0 1; -1/tau^2 -2/tau];
    emu.sof_50Hz.ss_c.B = [0; 1];
    emu.sof_50Hz.ss_c.C = [1/tau^2 0];
    emu.sof_50Hz.ss_c.D = 0;

    fprintf('Three phase grid emulator: %s\n', emu.name);
    fprintf('Nominal Voltage: %d V | Nominal Current: %.0f A | f = %.0f Hz | Z_grid = %.1f mOhm + j%.1f mOhm\n', ...
        emu.us_secondary_nom, emu.is_secondary_nom, emu.fgrid_nom, emu.eq_grid_resistance*1e3, ...
        emu.omega_grid_nom*emu.eq_grid_inductance*1e3);
    fprintf('Current Normalization Data: %.2f A | Voltage Normalization Data: %.2f V | Vemu_ref_norm = %.2f\n', ...
        emu.igrid_bez, emu.ugrid_bez, emu.Vemu_ref_norm);
    fprintf('---------------------------\n');
end
