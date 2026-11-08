function grid = grid_param(grid)
%GRID_PARAM  Grid emulator data of vienna_rectifier: transformer data set by application voltage, derived quantities
%   and the grid_three_phase_emulator object of the library.
%
%   grid = grid_param(grid), with grid = param.grid holding the inputs: grid_nominal_power [W], application_voltage
%   [V] (690, 480 or 400), delta_star (1: Dyn11, 0: Yyn), n2 (turns of the secondary), core_area [m^2], core_length [m],
%   mur, up_xi_pu_ref ... un_eta_pu_ref [pu], ucc_factor, eq_grid_resistance [Ohm].
%   Fields added: grid_nominal_current [A]; us1, us2 [V], fgrid [Hz], eta [%], ucc [%], p_iron [W] of the data set;
%   n1, mu0, Lm1 [H], i1m [A]; Lgrid_base, eq_grid_inductance [H]; emu (grid_three_phase_emulator object).
%
%   Fields read by vienna_rectifier.slx: emu.Vemu_ref_norm, emu.fgrid_nom, emu.us_primary_nom, emu.up_xi_pu_ref ...
%   emu.un_eta_pu_ref (block grid/grid_emulator/three_phase_voltage_source_with_frt_emulator), emu.trafo.Ld1, Ld2,
%   Lm1, Rd1, Rd2, Rfe1, n1, n2 (the three single-phase transformer blocks ph1, ph2, ph3 of grid/grid_emulator/3ph_Dyg,
%   delta primary, star secondary grounded) and emu.omega_grid_nom (DDSRF-PLL of the AFE control).
%
%   Same numbers and formulas as the legacy init and as frt_dqpll: the magnetizing inductance comes from the core
%   geometry, Lm1 = n1^2 mu0 mur A / l (144.8 mH), and the line no-load current from it, i1m = sqrt(3) us1 / (omega
%   Lm1): the delta primary coil sees the line voltage (library fix of 2026-11-04, three_phase_transformer_setup
%   applies the factor 3 of the delta to Rd1, Lm1, Rfe1). The coils have 24 and 14 turns (ratio 1.714, not sqrt(3)):
%   the secondary is at 697 V at no load (+1.0 %). The grid inductance is the short-circuit inductance of the
%   transformer (ucc in pu of us1^2/S at fgrid) times ucc_factor; no block of vienna_rectifier.slx reads it (the
%   controlled voltage sources feed the transformer directly, so the grid impedance seen by the LCL filter is the
%   leakage of the transformer only). The single-phase units make a bank of three cores: the zero-sequence
%   magnetizing inductance is lm (no lm0_pu argument, the pu data of configurable_trafo are not read here).

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
        grid.up_xi_pu_ref, grid.up_eta_pu_ref, grid.un_xi_pu_ref, grid.un_eta_pu_ref);

    tr = grid.emu.trafo;
    fprintf('Grid emulator: %.0f kVA, %.0f V, %.0f Hz, Vemu_ref_norm = %.3f | transformer Dyn11 (coils %d/%d): Lm1 = %.1f mH, i1m = %.1f A, Ld1 = %.1f uH, Ld2 = %.1f uH, Rd1 = %.1f mOhm, Rd2 = %.1f mOhm, Rfe1 = %.0f Ohm\n', ...
        grid.grid_nominal_power/1e3, grid.us2, grid.fgrid, grid.emu.Vemu_ref_norm, tr.n1, tr.n2, grid.Lm1*1e3, ...
        grid.i1m, tr.Ld1*1e6, tr.Ld2*1e6, tr.Rd1*1e3, tr.Rd2*1e3, tr.Rfe1);
end
