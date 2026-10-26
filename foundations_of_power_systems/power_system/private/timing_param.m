function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of power_system (local replacement of timing_setup of the library).
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm, t.trgo        [Hz] PWM frequency of the (virtual) converter clock domain and double
%                           update flag (1: two control steps per PWM period). There is no converter in
%                           this model: ts_afe is only the sample time of the discrete blocks of the
%                           analysis (moving average filters, To Workspace).
%     t.t_measure           [s] logging window of the To Workspace blocks
%     t.tc_factor           ts_afe/tc: tc is the step of the Simscape local solver (grid/grid_emulator/
%                           Solver Configuration) and the sample time of the IEC power meters
%     t.tc_decimation       decimation of the logging at tc
%     t.delay_pwm           [s] PWM delay of the modulator model B (unused here, kept for the name)
%     t.dead_time           [s] (unused here, kept for the name)
%
%   Fields read by power_system.slx: ts_afe, Ns_afe, tc. The other fields keep the names of the
%   library object timing_setup so that the scripts written for it keep working.

    % converter clock domain (legacy name AFE)
    glb_time.fPWM_AFE = t.fpwm;
    glb_time.tPWM_AFE = 1/t.fpwm;
    glb_time.TRGO_AFE_double_update = t.trgo;
    glb_time.ts_afe = sample_time(glb_time.tPWM_AFE, t.trgo);
    glb_time.dead_time_AFE = t.dead_time;
    glb_time.minimum_pulse_time_AFE = 1e-6;
    glb_time.delayAFE_modB = 2*pi*t.fpwm*t.delay_pwm;

    % Simscape local solver step and logging base
    glb_time.tc = glb_time.ts_afe/t.tc_factor;
    glb_time.decimation_tc = t.tc_decimation;
    glb_time.t_measure = t.t_measure;
    glb_time.Nc = ceil(t.t_measure/glb_time.tc/t.tc_decimation);
    glb_time.Ns_afe = ceil(t.t_measure/glb_time.ts_afe);

    fprintf('Timing: fPWM = %.0f Hz, double update = %d, ts_afe = %.1f us, tc = %.3f us, Nc = %d, Ns_afe = %d\n', ...
        glb_time.fPWM_AFE, glb_time.TRGO_AFE_double_update, glb_time.ts_afe*1e6, glb_time.tc*1e6, ...
        glb_time.Nc, glb_time.Ns_afe);
end

function ts = sample_time(tpwm, double_update)
    if double_update
        ts = tpwm/2;
    else
        ts = tpwm;
    end
end
