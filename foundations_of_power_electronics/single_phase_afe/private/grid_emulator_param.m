function emu = grid_emulator_param(application_voltage, f_nom, pwr_nom, ucc, fault)
%GRID_EMULATOR_PARAM  Data of the three-phase grid emulator of single_phase_afe.slx (block
%   three_phase_voltage_source_with_frt_emulator of the library) and of its fault generator.
%
%   emu = grid_emulator_param(application_voltage, f_nom, pwr_nom, ucc, fault)
%     application_voltage : [V] rms line-to-line (690, 480, 400)
%     f_nom               : [Hz]
%     pwr_nom             : [W] short-circuit/transformer power behind the emulator (bases of the grid
%                           currents and equivalent inductance, informative: the emulator of the model is
%                           an ideal source without series impedance)
%     ucc                 : [%] short-circuit voltage used for the equivalent grid inductance
%     fault               : struct of the fault test (see grid_fault_param), stored in emu.frt
%
%   Local replacement of grid_three_phase_emulator (object with the transformer model) + frt_settings +
%   grid_fault_generator + the SOF_50Hz entry of setup_global_filters: same numbers and the same field
%   names where the model reads them. Fields read by single_phase_afe.slx:
%     fgrid_nom, omega_grid_nom, us_nom, Vemu_ref_norm, up_xi_pu_ref, up_eta_pu_ref, un_xi_pu_ref,
%     un_eta_pu_ref, u2bez, sof_50Hz.A, sof_50Hz.B, sof_50Hz.C, frt.* (grid_fault_param)
%
%   Bases: u2bez = application_voltage*sqrt(2/3) is the peak phase-to-neutral voltage (563.4 V at 690 V),
%   the base of the grid voltages acquired by the PLL; i2bez the peak phase current at pwr_nom.

    emu.name = sprintf('grid_%dV_%dkVA', application_voltage, round(pwr_nom/1e3));
    emu.application_voltage = application_voltage;
    emu.us_nom = application_voltage;                       % [V] rms line-to-line (legacy us_primary_nom)
    emu.fgrid_nom = f_nom;                                  % [Hz]
    emu.omega_grid_nom = 2*pi*f_nom;                        % [rad/s]
    emu.pwr_nom = pwr_nom;                                  % [VA]
    emu.is_nom = pwr_nom/sqrt(3)/application_voltage;       % [A] rms phase current at pwr_nom

    % bases of the grid quantities (secondary side of the legacy transformer object = the grid itself)
    emu.u2bez = application_voltage*sqrt(2/3);              % [V] peak phase-to-neutral
    emu.i2bez = emu.is_nom*sqrt(2);                         % [A] peak
    emu.ugrid_bez = emu.u2bez;
    emu.igrid_bez = emu.i2bez;

    % voltage reference of the emulator: 1 pu of u2bez, positive sequence on xi, no negative sequence
    emu.ugrid_factor = 1;
    emu.Vemu_ref = application_voltage/sqrt(3)*emu.ugrid_factor;    % [V] rms phase-to-neutral
    emu.Vemu_ref_norm = emu.Vemu_ref*sqrt(2)/emu.u2bez;             % [pu] = 1
    emu.up_xi_pu_ref = 1;
    emu.up_eta_pu_ref = 0;
    emu.un_xi_pu_ref = 0;
    emu.un_eta_pu_ref = 0;

    % equivalent grid impedance (informative: the emulator of the model has no series impedance)
    emu.ucc = ucc;                                                              % [%]
    emu.L_eq = application_voltage/sqrt(3)*ucc/100/emu.omega_grid_nom/emu.is_nom;   % [H]
    emu.R_eq = 2e-3;                                                            % [Ohm]

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

    fprintf('Grid emulator: %s, %.0f Hz, u2bez = %.1f V, i2bez = %.1f A, L_eq = %.0f uH (ucc %.0f %%, not in the model)\n', ...
        emu.name, f_nom, emu.u2bez, emu.i2bez, emu.L_eq*1e6, ucc);
    fprintf('Fault test: %s, enable_frt_1 = %d, enable_frt_2 = %d, amplitudes [%.3f %.3f %.3f], phase shift %.3f rad, from %.3f s to %.3f s\n', ...
        fault.kind, fault.enable_frt_1, fault.enable_frt_2, fault.Amplitude_U, fault.Amplitude_V, fault.Amplitude_W, ...
        fault.Phase_shift, fault.start_time_LVRT, fault.end_time_LVRT);
    fprintf('---------------------------\n');
end
