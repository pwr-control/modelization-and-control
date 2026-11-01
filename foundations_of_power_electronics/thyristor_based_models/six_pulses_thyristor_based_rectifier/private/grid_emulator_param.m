function emu = grid_emulator_param(g, fault)
%GRID_EMULATOR_PARAM  Data of the three-phase grid emulator of six_pulse_thyristor_based_rectifier.slx
%   (block three_phase_voltage_source_with_frt_emulator of the library, original Simulink diagram, in
%   grid_line/Grid/grid_emulator: three ideal voltage sources, star point grounded) and of its fault
%   generator. The transformer between the emulator and the bridge is the configurable_trafo block (see
%   transformer_param).
%
%   emu = grid_emulator_param(g, fault), with g = param.grid:
%     g.application_voltage : [V] rms line-to-line voltage of the emulator (primary of the transformer)
%     g.f_nom               : [Hz]
%     fault                 : struct of the fault test (see grid_fault_param), stored in emu.frt
%
%   Fields read by the model (mask of the emulator block): us_nom (nominal_voltage_m, rms line-to-line),
%   fgrid_nom (nominal_frequency_m), Vemu_ref_norm, up_xi_pu_ref, up_eta_pu_ref, un_xi_pu_ref,
%   un_eta_pu_ref, sof_50Hz.A, sof_50Hz.B, sof_50Hz.C; the fault fields are read from param.grid.frt.
%   Local replacement of grid_three_phase_emulator + the SOF_50Hz entry of setup_global_filters of the
%   library (the draft init built the library object, which has neither us_nom nor sof_50Hz, and for
%   690 V instead of 400 V).

    V = g.application_voltage;
    emu.name = sprintf('grid_%dV_%dHz', V, g.f_nom);
    emu.us_nom = V;                                         % [V] rms line-to-line
    emu.fgrid_nom = g.f_nom;                                % [Hz]
    emu.omega_grid_nom = 2*pi*g.f_nom;                      % [rad/s]
    emu.ugrid_bez = V*sqrt(2/3);                            % [V] peak phase-to-neutral (326.6 V at 400 V)

    % voltage reference of the emulator: 1 pu, positive sequence on xi, no negative sequence
    emu.ugrid_factor = 1;
    emu.Vemu_ref = V/sqrt(3)*emu.ugrid_factor;              % [V] rms phase-to-neutral
    emu.Vemu_ref_norm = emu.Vemu_ref*sqrt(2)/emu.ugrid_bez; % [pu] = 1
    emu.up_xi_pu_ref = 1;
    emu.up_eta_pu_ref = 0;
    emu.un_xi_pu_ref = 0;
    emu.un_eta_pu_ref = 0;

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

    fprintf('Grid emulator: %s, ugrid_bez = %.1f V (ideal sources, the transformer is the only impedance)\n', ...
        emu.name, emu.ugrid_bez);
    fprintf('Fault test: %s, enable_frt_1 = %d, enable_frt_2 = %d, amplitudes [%.3f %.3f %.3f], phase shift %.3f rad, from %.3f s to %.3f s\n', ...
        fault.kind, fault.enable_frt_1, fault.enable_frt_2, fault.Amplitude_U, fault.Amplitude_V, fault.Amplitude_W, ...
        fault.Phase_shift, fault.start_time_LVRT, fault.end_time_LVRT);
end
