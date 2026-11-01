function frt = grid_fault_param(kind, dip, error_length, start_time, enable_frt_1, enable_frt_2)
%GRID_FAULT_PARAM  Fault data of the grid emulator (local replacement of frt_settings +
%   grid_fault_generator + asymmetric_error of the library, reduced to the fault kinds of this model).
%
%   frt = grid_fault_param(kind, dip, error_length, start_time, enable_frt_1, enable_frt_2)
%     kind          : 'none' | 'three_phase' | 'two_phase_C' | 'two_phase_D'
%                     three_phase: the three amplitudes drop to 1 - dip;
%                     two_phase_C: phases V and W drop (variant C: U stays at 1, V and W move
%                     symmetrically); two_phase_D: variant D (U drops to 1 - dip too), as the function
%                     asymmetric_error of the library
%     dip           : [pu] depth of the dip (0.75: 25 % of the voltage left)
%     error_length  : [s]
%     start_time    : [s] start of the fault
%     enable_frt_1  : 1 applies the fault from the abc amplitudes (Amplitude_U/V/W, Phase_shift)
%     enable_frt_2  : 1 applies the fault from the xi/eta positive and negative sequence references
%                     (deepPOSxi, deepPOSeta, deepNEGxi, deepNEGeta)
%
%   Fields read by the emulator block: enable_frt_1, enable_frt_2, start_time_LVRT, end_time_LVRT,
%   deepPOSxi, deepPOSeta, deepNEGxi, deepNEGeta, Amplitude_U, Amplitude_V, Amplitude_W, t_ramp,
%   Phase_shift. The draft init (copy of the shared template) selected test 25.4 (75 % dip, two
%   phases, variant D, from 0.75 s for 915 ms) with enable_frt_2 = 1: same numbers with
%   grid_fault_param('two_phase_D', 0.75, 0.915, 0.75, 0, 1). Same function as in afe_dq_ttype_three_phase.

    frt.kind = kind;
    frt.dip = dip;
    frt.enable_frt_1 = enable_frt_1;
    frt.enable_frt_2 = enable_frt_2;

    switch kind
        case 'none'
            frt.Amplitude_U = 1; frt.Amplitude_V = 1; frt.Amplitude_W = 1;
            frt.Phase_shift = 0;
        case 'three_phase'
            frt.Amplitude_U = 1 - dip; frt.Amplitude_V = 1 - dip; frt.Amplitude_W = 1 - dip;
            frt.Phase_shift = 0;
        case 'two_phase_C'
            [rho, delta_phi] = asymmetric_dip(dip, 0);
            frt.Amplitude_U = 1; frt.Amplitude_V = rho; frt.Amplitude_W = rho;
            frt.Phase_shift = delta_phi;
        case 'two_phase_D'
            [rho, delta_phi] = asymmetric_dip(dip, 1);
            frt.Amplitude_U = 1 - dip; frt.Amplitude_V = rho; frt.Amplitude_W = rho;
            frt.Phase_shift = delta_phi;
        otherwise
            error('grid_fault_param:kind', 'unknown fault kind %s', kind);
    end

    % sequence references of the second fault generator (legacy values)
    frt.deepPOSxi = 1;
    frt.deepPOSeta = -0.4;
    frt.deepNEGxi = 0.4;
    frt.deepNEGeta = 0.4;

    frt.start_time_LVRT = start_time;                       % [s]
    frt.error_length = error_length;                        % [s]
    frt.end_time_LVRT = start_time + error_length;          % [s]
    frt.t_ramp = 20e-3;                                     % [s] ramp of the amplitudes
    frt.k_lvrt = 2;                                         % reactive current gain of the grid support (informative)
end

function [rho, delta_phi] = asymmetric_dip(voltage_dip, variant)
% Library function asymmetric_error: amplitude and phase of the two faulted phases. Variant C (0):
% the faulted phases move toward each other; variant D (1): the faulted phases keep the imaginary part.
    if variant == 0
        im = sqrt(3)/2*(1 - voltage_dip);
        re = -1/2;
    else
        im = sqrt(3)/2;
        re = -1/2*(1 - voltage_dip);
    end
    [theta, rho] = cart2pol(re, im);
    delta_phi = theta - 2/3*pi;
end
