function frt = grid_fault_param(f)
%GRID_FAULT_PARAM  Fault test of the grid emulator of afe_inv_psm_n.slx (local replacement of frt_settings +
%   grid_fault_generator + asymmetric_error of the library, reduced to the fault kinds below).
%
%   frt = grid_fault_param(f), with f = param.grid.fault_in:
%     f.kind          'none' | 'three_phase' | 'two_phase_C' | 'two_phase_D'
%                     three_phase: the three amplitudes drop to 1 - dip; two_phase_C: phases V and W drop
%                     (variant C: U stays at 1, V and W move symmetrically); two_phase_D: variant D (U drops
%                     to 1 - dip too), as the function asymmetric_error of the library
%     f.dip           [pu] depth of the dip (0.75: 25 % of the voltage left)
%     f.error_length  [s] length of the fault
%     f.start_time    [s] start of the fault
%     f.enable_frt_1  1 applies the fault from the abc amplitudes (Amplitude_U/V/W, Phase_shift)
%     f.enable_frt_2  1 applies the fault from the positive and negative sequence references below
%     f.deepPOSxi, f.deepPOSeta, f.deepNEGxi, f.deepNEGeta
%                     [pu] fault of kind 2: the emulator SUBTRACTS these values from the pre-fault sequence
%                     references (param.grid.emu.up_xi_pu_ref = 1, the others 0): during the fault the positive
%                     sequence is up_xi*(1 - deepPOSxi) + j*(up_eta - deepPOSeta), the negative sequence
%                     (un_xi - deepNEGxi) + j*(un_eta - deepNEGeta)
%     f.t_ramp        [s] ramp of the amplitudes at the start and at the end of the fault
%
%   Fields read by the emulator block (three_phase_voltage_source_with_frt_emulator, mask): enable_frt_1,
%   enable_frt_2, start_time_LVRT, end_time_LVRT, deepPOSxi, deepPOSeta, deepNEGxi, deepNEGeta,
%   Amplitude_U, Amplitude_V, Amplitude_W, t_ramp, Phase_shift.
%   The legacy init selected test 25.4 (75 % dip on two phases, variant D, 915 ms from 0.75 s) with
%   enable_frt_1 = 0 and enable_frt_2 = 1: the emulator applies the sequence references, not the abc
%   amplitudes. The reactive current gain and the settle time of the FRT control (legacy frt_data.k_frt_ref,
%   frt_data.settle_time) are control data: they are in param.afe.frt.

    frt.kind = f.kind;
    frt.dip = f.dip;
    frt.enable_frt_1 = f.enable_frt_1;
    frt.enable_frt_2 = f.enable_frt_2;

    switch f.kind
        case 'none'
            frt.Amplitude_U = 1; frt.Amplitude_V = 1; frt.Amplitude_W = 1;
            frt.Phase_shift = 0;
        case 'three_phase'
            frt.Amplitude_U = 1 - f.dip; frt.Amplitude_V = 1 - f.dip; frt.Amplitude_W = 1 - f.dip;
            frt.Phase_shift = 0;
        case 'two_phase_C'
            [rho, delta_phi] = asymmetric_dip(f.dip, 0);
            frt.Amplitude_U = 1; frt.Amplitude_V = rho; frt.Amplitude_W = rho;
            frt.Phase_shift = delta_phi;
        case 'two_phase_D'
            [rho, delta_phi] = asymmetric_dip(f.dip, 1);
            frt.Amplitude_U = 1 - f.dip; frt.Amplitude_V = rho; frt.Amplitude_W = rho;
            frt.Phase_shift = delta_phi;
        otherwise
            error('grid_fault_param:kind', 'unknown fault kind %s', f.kind);
    end

    % sequence references of the second fault generator (enable_frt_2)
    frt.deepPOSxi = f.deepPOSxi;
    frt.deepPOSeta = f.deepPOSeta;
    frt.deepNEGxi = f.deepNEGxi;
    frt.deepNEGeta = f.deepNEGeta;

    frt.start_time_LVRT = f.start_time;                     % [s]
    frt.error_length = f.error_length;                      % [s]
    frt.end_time_LVRT = f.start_time + f.error_length;      % [s]
    frt.t_ramp = f.t_ramp;                                  % [s]
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
