function frt = grid_fault_param(f)
%GRID_FAULT_PARAM  Grid fault (FRT) settings of the grid emulator of power_system: local replacement of
%   frt_settings + grid_fault_generator + asymmetric_error of the library, same field names, the
%   grid-code test catalogue as a table instead of a switch.
%
%   frt = grid_fault_param(f), with f = param.grid.frt_in:
%     f.test_index, f.test_subindex    test of the catalogue below (e.g. 25 and 4: 75 % dip, two-phase, full load)
%     f.asymmetric_error_type          two-phase tests: 0 variant C, 1 variant D (VDE/FGW TG3)
%     f.enable_frt_1                   1: fault built from the phase amplitudes and the phase shift of the
%                                      catalogue (Amplitude_U/V/W, Phase_shift: abc)
%     f.enable_frt_2                   1: fault built from the sequence components deepPOS*/deepNEG* (xi/eta)
%     f.start_time_LVRT                [s] start of the fault; the end is start + error_length of the test
%     f.t_ramp                         [s] ramp of the phase voltages at the start and at the end of the fault
%     f.deepPOSxi, f.deepPOSeta, f.deepNEGxi, f.deepNEGeta   [pu] positive and negative sequence components
%                                      of the emulated voltage during the fault (enable_frt_2)
%
%   Fields read by power_system.slx (mask of grid/grid_emulator/three_phase_voltage_source_with_frt_emulator):
%     enable_frt_1, enable_frt_2, start_time_LVRT, end_time_LVRT, deepPOSxi, deepPOSeta, deepNEGxi,
%     deepNEGeta, Amplitude_U, Amplitude_V, Amplitude_W, Phase_shift, t_ramp
%   Informative fields: error_length, P_load_norm, Q_load_norm, k_lvrt (load and grid-support factor of the
%   test, for the converter models that use the same catalogue), description.
%
%   Two-phase tests: the remaining voltage rho and the phase shift delta_phi of phases V and W come from the
%   phasor construction of asymmetric_error (variant C: the faulted phases move along the imaginary axis,
%   U = 1; variant D: along the real axis, U = 1 - dip as well); Amplitude_V = Amplitude_W = rho,
%   Phase_shift = delta_phi.
%
%   Catalogue (index.subindex): three-phase tests give the remaining voltage of the three phases, two-phase
%   tests the dip passed to asymmetric_error (remaining voltage 1 - dip). Test 25.3 does not exist (empty in
%   the library). Test 0 is the no-fault setting.

    %        index sub  kind   value  error_length  P_load  Q_load  k_lvrt  description
    catalogue = {
        0,    0,  '3ph', 1.00,  0,          1.0,  0.10, 2, 'no fault';
        25,   1,  '3ph', 0.25,  760e-3,     1.0,  0,    2, '75 % dip, three-phase, full load';
        25,   2,  '3ph', 0.25,  760e-3,     0.5,  0,    2, '75 % dip, three-phase, partial load';
        25,   4,  '2ph', 0.75,  915e-3,     1.0,  0,    2, '75 % dip, two-phase, full load';
        25,   5,  '2ph', 0.75,  250e-3,     0.5,  0,    2, '75 % dip, two-phase, partial load';
        50,   1,  '3ph', 0.50,  1982e-3,    1.0,  0,    2, '50 % dip, three-phase, full load';
        50,   2,  '3ph', 0.50,  1982e-3,    0.5,  0,    2, '50 % dip, three-phase, partial load';
        50,   3,  '2ph', 0.50,  2305e-3,    1.0,  0,    2, '50 % dip, two-phase, full load';
        50,   4,  '2ph', 0.50,  2305e-3,    0.5,  0,    2, '50 % dip, two-phase, partial load';
        50,   5,  '3ph', 0.50,  1982e-3,    1.0,  0,    2, '50 % dip, three-phase, full load, reduced dynamic grid support';
        50,   6,  '2ph', 0.50,  2305e-3,    1.0,  0,    2, '50 % dip, two-phase, full load, reduced dynamic grid support';
        75,   1,  '3ph', 0.75,  2796e-3,    1.0,  0,    2, '25 % dip, three-phase, full load';
        75,   2,  '3ph', 0.75,  2796e-3,    0.5,  0,    2, '25 % dip, three-phase, partial load';
        75,   3,  '3ph', 0.75,  2796e-3,    0.5,  0.33, 2, '25 % dip, three-phase, partial load, max. under-excited';
        75,   4,  '3ph', 0.75,  2796e-3,    0.5, -0.33, 2, '25 % dip, three-phase, partial load, max. over-excited';
        75,   5,  '3ph', 0.75,  2796e-3,    0.5,  0,    4, '25 % dip, three-phase, partial load, k = 4';
        75,   6,  '2ph', 0.25,  3000e-3,    1.0,  0,    2, '25 % dip, two-phase, full load';
        75,   7,  '2ph', 0.25,  3000e-3,    0.5,  0,    2, '25 % dip, two-phase, partial load';
        75,   8,  '2ph', 0.25,  3000e-3,    0.5,  0,    4, '25 % dip, two-phase, partial load, k = 4';
        80,   1,  '3ph', 0.80,  3000e-3,    1.0,  0.10, 2, '20 % dip, three-phase, full load, reduced dynamic grid support';
        80,   2,  '2ph', 0.20,  3000e-3,    1.0,  0,    2, '20 % dip, two-phase, full load, reduced dynamic grid support';
        85,   1,  '3ph', 0.88,  60000e-3,   1.0,  0,    2, '12 % dip, three-phase, full load';
        110,  1,  '2ph', -0.15, 5000e-3,    1.0,  0,    2, '15 % rise, two-phase, full load';
        110,  2,  '2ph', -0.15, 5000e-3,    0.5,  0,    2, '15 % rise, two-phase, partial load';
        110,  3,  '3ph', 1.16,  60000e-3,   0.5,  0,    2, '16 % rise, three-phase, partial load';
        115,  1,  '3ph', 1.16,  5000e-3,    1.0,  0,    2, '16 % rise, three-phase, full load';
        115,  2,  '3ph', 1.16,  5000e-3,    0.5,  0,    2, '16 % rise, three-phase, partial load'};

    row = find([catalogue{:, 1}] == f.test_index & [catalogue{:, 2}] == f.test_subindex, 1);
    if isempty(row)
        error('grid_fault_param:unknownTest', 'Test %d.%d is not in the catalogue.', f.test_index, f.test_subindex);
    end
    [kind, value, error_length, P_load_norm, Q_load_norm, k_lvrt, description] = catalogue{row, 3:9};

    if strcmp(kind, '3ph')
        Amplitude_U = value;
        Amplitude_V = value;
        Amplitude_W = value;
        Phase_shift = 0;
    else
        [rho, delta_phi] = asymmetric_error(value, f.asymmetric_error_type);
        if f.asymmetric_error_type == 0
            Amplitude_U = 1;
        else
            Amplitude_U = 1 - value;
        end
        Amplitude_V = rho;
        Amplitude_W = rho;
        Phase_shift = delta_phi;
    end

    % settings
    frt.enable_frt_1 = f.enable_frt_1;
    frt.enable_frt_2 = f.enable_frt_2;
    frt.test_index = f.test_index;
    frt.test_subindex = f.test_subindex;
    frt.asymmetric_error_type = f.asymmetric_error_type;
    frt.description = description;

    % sequence components of the fault (type 2)
    frt.deepPOSxi = f.deepPOSxi;
    frt.deepPOSeta = f.deepPOSeta;
    frt.deepNEGxi = f.deepNEGxi;
    frt.deepNEGeta = f.deepNEGeta;

    % timing and phase quantities of the fault (type 1)
    frt.start_time_LVRT = f.start_time_LVRT;                                % [s]
    frt.error_length = error_length;                                        % [s]
    frt.end_time_LVRT = f.start_time_LVRT + error_length;                   % [s]
    frt.t_ramp = f.t_ramp;                                                  % [s]
    frt.Amplitude_U = Amplitude_U;                                          % [pu]
    frt.Amplitude_V = Amplitude_V;
    frt.Amplitude_W = Amplitude_W;
    frt.Phase_shift = Phase_shift;                                          % [rad] of phases V and W
    frt.P_load_norm = P_load_norm;                                          % [pu] load of the test
    frt.Q_load_norm = Q_load_norm;
    frt.k_lvrt = k_lvrt;                                                    % dynamic grid support factor

    fprintf('Grid fault: test %d.%d (%s), %.3f s from t = %.3f s, abc amplitudes [%.3f %.3f %.3f] pu, phase shift %.1f deg (type 1: %d), sequences pos [%.2f %.2f] neg [%.2f %.2f] pu (type 2: %d)\n', ...
        frt.test_index, frt.test_subindex, frt.description, frt.error_length, frt.start_time_LVRT, ...
        frt.Amplitude_U, frt.Amplitude_V, frt.Amplitude_W, frt.Phase_shift*180/pi, frt.enable_frt_1, ...
        frt.deepPOSxi, frt.deepPOSeta, frt.deepNEGxi, frt.deepNEGeta, frt.enable_frt_2);
end

function [rho, delta_phi] = asymmetric_error(voltage_dip, error_type)
% remaining voltage and phase shift of the two faulted phases: before the fault the phasor V - W has
% magnitude sqrt(3) (phase voltages of 1 pu), during the fault variant C keeps the real part of the phase
% V phasor and scales the imaginary part by (1 - dip) (two-phase fault without earth), variant D keeps the
% imaginary part and scales the real part (two-phase-to-earth type)
    if error_type == 0
        im = sqrt(3)/2*(1 - voltage_dip);
        re = -1/2;
    else
        im = sqrt(3)/2;
        re = -1/2*(1 - voltage_dip);
    end
    [theta, rho] = cart2pol(re, im);
    delta_phi = theta - 2/3*pi;
end
