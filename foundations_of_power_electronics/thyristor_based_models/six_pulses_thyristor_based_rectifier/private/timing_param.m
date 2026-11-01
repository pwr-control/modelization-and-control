function glb_time = timing_param(t, simlength)
%TIMING_PARAM  Timing data of six_pulse_thyristor_based_rectifier (local, replaces the flat names of the
%   legacy init and timing_setup of the library).
%
%   glb_time = timing_param(t, simlength), with t = param.time:
%     t.fs_rect       [Hz] rate of the digital control of the rectifier: PLL, firing pulse generator,
%                     current PI (legacy fPWM_RECT = 5 kHz, Tcontrol_rect = 1/fPWM_RECT)
%     t.tc_factor     ts_rect/tc: tc is the logging base, MaxStep of the global solver and the step of the
%                     Simscape local solver (legacy Tc = Ts = Tcontrol_rect/400)
%     t.t_measure     [s] window of the logging at tc (thyristor data, To Workspace thyristor_data;
%                     legacy t_misura)
%     simlength       [s] StopTime: window of the logging at ts_rect (power meter)
%
%   Fields read by six_pulse_thyristor_based_rectifier.slx: ts_rect, tc, Nc, Ns_rect.
%   Legacy flat names: Tcontrol_rect -> ts_rect, Tc and Ts -> tc, Nc -> Nc. The power meter blocks
%   (copied from the AFE models) read glb_time.ts_afe and glb_time.Ns_afe in the draft init: they read
%   ts_rect and Ns_rect now (one clock domain).

    glb_time.fs_rect = t.fs_rect;                           % [Hz]
    glb_time.ts_rect = 1/t.fs_rect;                         % [s] 200 us at 5 kHz
    glb_time.tc = glb_time.ts_rect/t.tc_factor;             % [s] 0.5 us
    glb_time.t_measure = t.t_measure;                       % [s]
    glb_time.Nc = ceil(t.t_measure/glb_time.tc);            % samples of the logging at tc (last t_measure)
    glb_time.Ns_rect = ceil(simlength/glb_time.ts_rect);    % samples of the logging at ts_rect (whole run)

    % firing resolution of the pulse generator: one control step in electrical degrees at 50 Hz
    glb_time.firing_step_deg_50Hz = 360*50*glb_time.ts_rect;

    % discrete-time variable for the design checks
    glb_time.z_rect = tf('z', glb_time.ts_rect);

    fprintf('Timing: control at %.0f Hz (ts_rect = %.1f us, %.1f deg at 50 Hz), tc = %.3f us, Nc = %d (last %.0f ms at tc), Ns_rect = %d\n', ...
        glb_time.fs_rect, glb_time.ts_rect*1e6, glb_time.firing_step_deg_50Hz, glb_time.tc*1e6, glb_time.Nc, ...
        t.t_measure*1e3, glb_time.Ns_rect);
end
