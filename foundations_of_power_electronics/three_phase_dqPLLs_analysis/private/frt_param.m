function data = frt_param(frt)
%FRT_PARAM  Fault data of the grid emulator of frt_dqpll: frt_settings of the library plus the waveform data of
%   grid_fault_generator, merged in one struct (param.frt.data).
%
%   data = frt_param(frt), with frt = param.frt holding test_index, test_subindex, asymmetric_error_type,
%   enable_frt_1, enable_frt_2, start_time_LVRT, deepPOSxi, deepPOSeta, deepNEGxi, deepNEGeta.
%   data: the fields of frt_settings (enable flags, sequence dips, timing of the shared template, settle_time 0.175 s
%   and k_frt_ref 2 of the grid support) plus Amplitude_U/V/W, Phase_shift, error_length, end_time_LVRT, t_ramp
%   (20 ms), P_load_norm, Q_load_norm and k_lvrt of grid_fault_generator. The block
%   three_phase_voltage_source_with_frt_emulator1 reads the amplitudes, the phase shift, the times and the dips;
%   FRT-VDE4110/enable reads settle_time; param.afe.uvw.frt.k_frt_ref reads k_frt_ref.
%
%   grid_fault_generator is a function of the library since October 2026 (the legacy init called it as a script
%   and stopped there with "Not enough input arguments"). The library version of test 25.4 lasts 915 ms (the private
%   copy of afe_uvw_three_phase has 250 ms): with start_time_LVRT = 0.75 s the fault ends at 1.665 s, after the end
%   of the simulation (1.25 s).

    data = frt_settings(frt.test_index, frt.test_subindex, frt.asymmetric_error_type, frt.enable_frt_1, ...
        frt.enable_frt_2, frt.start_time_LVRT, frt.deepPOSxi, frt.deepPOSeta, frt.deepNEGxi, frt.deepNEGeta);
    gen = grid_fault_generator(data, frt.start_time_LVRT);
    for fn = fieldnames(gen)'
        data.(fn{1}) = gen.(fn{1});
    end

    fprintf('FRT: test %d.%d (asymmetric type %d, abc generator %d, sequence generator %d), amplitudes U V W = %.3g %.3g %.3g pu, phase shift %.1f deg, from %.3f s for %.0f ms (end %.3f s), ramp %.0f ms\n', ...
        data.test_index, data.test_subindex, data.asymmetric_error_type, data.enable_frt_1, data.enable_frt_2, ...
        data.Amplitude_U, data.Amplitude_V, data.Amplitude_W, data.Phase_shift*180/pi, data.start_time_LVRT, ...
        data.error_length*1e3, data.end_time_LVRT, data.t_ramp*1e3);
end
