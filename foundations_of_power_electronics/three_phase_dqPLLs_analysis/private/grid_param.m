function grid = grid_param(grid)
%GRID_PARAM  Grid emulator data of frt_dqpll: transformer data set by application voltage, derived quantities and the
%   grid_three_phase_emulator object of the library.
%
%   grid = grid_param(grid), with grid = param.grid holding the inputs: grid_nominal_power [W], application_voltage
%   [V] (690, 480 or 400), delta_star (1: Dyn11, 0: Yyn), n2 (turns of the secondary), core_area [m^2], core_length [m],
%   mur, up_xi_pu_ref ... un_eta_pu_ref [pu], ucc_factor, eq_grid_resistance [Ohm], lm0_pu [pu].
%   Fields added: grid_nominal_current [A]; us1, us2 [V], fgrid [Hz], eta [%], ucc [%], p_iron [W] of the data set;
%   n1, mu0, Lm1 [H], i1m [A]; Lgrid_base, eq_grid_inductance [H]; emu (grid_three_phase_emulator object).
%
%   Same numbers and formulas as the legacy init and as afe_uvw_three_phase: the magnetizing inductance comes from
%   the core geometry, Lm1 = n1^2 mu0 mur A / l (144.8 mH), and the line no-load current from it, i1m = sqrt(3) us1 /
%   (omega Lm1): the delta primary coil sees the line voltage (library fix of 2026-11-04, three_phase_transformer_setup
%   applies the factor 3 of the delta to Rd1, Lm1, Rfe1). The grid inductance is the short-circuit inductance of the
%   transformer (ucc in pu of us1^2/S at fgrid) times ucc_factor; no block of frt_dqpll.slx reads it (the controlled
%   voltage sources feed the transformer directly). The emulator object gives the normalization of the grid
%   quantities (igrid_bez, ugrid_bez), the voltage reference (Vemu_ref_norm) and the transformer in SI and in pu
%   (emu.trafo, emu.trafo.pu for the block configurable_trafo of 3ph_Dyg_ssc); lm0_pu is the zero-sequence
%   magnetizing inductance of the three-limb core.

    grid.grid_nominal_current = grid.grid_nominal_power/grid.application_voltage/sqrt(3);

    % coupling transformer data set
    switch grid.application_voltage
        case 690
            grid.us1 = 690; grid.us2 = 690; grid.fgrid = 50;
            grid.eta = 95; grid.ucc = 5;
            grid.p_iron = 1800;
        case 480
            grid.us1 = 480; grid.us2 = 480; grid.fgrid = 60;
            grid.eta = 95; grid.ucc = 5;
            grid.p_iron = 1400;
        otherwise
            grid.us1 = 400; grid.us2 = 400; grid.fgrid = 50;
            grid.eta = 95; grid.ucc = 5;
            grid.p_iron = 1000;
    end
    grid.n1 = floor(grid.n2*sqrt(3));
    grid.mu0 = 4*pi*1e-7;
    grid.Lm1 = (grid.n1^2 * grid.mu0 * grid.mur * grid.core_area) / grid.core_length;      % [H] magnetizing inductance of one delta coil
    grid.i1m = sqrt(3)*grid.us1/grid.Lm1/(2*pi*grid.fgrid);                                 % [A] line no-load current

    % grid impedance (computed as in the legacy init, not read by the model)
    grid.Lgrid_base = grid.us1/sqrt(3)*grid.ucc/100/2/pi/grid.fgrid/grid.grid_nominal_current;  % [H]
    grid.eq_grid_inductance = grid.Lgrid_base*grid.ucc_factor;                                   % [H]

    grid.emu = grid_three_phase_emulator('Dyn11', grid.delta_star, grid.grid_nominal_power, grid.application_voltage, ...
        grid.us1, grid.us2, grid.fgrid, grid.eq_grid_inductance, grid.eq_grid_resistance, grid.eta, grid.ucc, grid.i1m, ...
        grid.p_iron, grid.n1, grid.n2, grid.core_area, grid.core_length, grid.mur, ...
        grid.up_xi_pu_ref, grid.up_eta_pu_ref, grid.un_xi_pu_ref, grid.un_eta_pu_ref, grid.lm0_pu);

    pu = grid.emu.trafo.pu;
    fprintf('Grid emulator: %.0f kVA, %.0f V, %.0f Hz, Vemu_ref_norm = %.3f | transformer Dyn11: Lm1 = %.1f mH, i1m = %.1f A, pu r1+r3 = %.4f, l1+l3 = %.4f, lm = %.1f, lm0 = %.1f, rfe = %.0f\n', ...
        grid.grid_nominal_power/1e3, grid.us2, grid.fgrid, grid.emu.Vemu_ref_norm, grid.Lm1*1e3, grid.i1m, ...
        pu.r1 + pu.r3, pu.l1 + pu.l3, pu.lm, pu.lm0, pu.rfe);
end
