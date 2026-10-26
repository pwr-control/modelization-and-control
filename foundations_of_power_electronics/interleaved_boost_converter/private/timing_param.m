function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of interleaved_boost (local replacement of timing_setup of the library).
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm, t.trgo             PWM frequency of the boost legs [Hz] and double update flag (1: two control
%                                steps per carrier period, at the bottom and at the top of carrier 1)
%     t.fpwm_dab, t.trgo_dab     sample time ts_dab of the battery state-of-charge estimator (legacy name of
%                                the shared template, no DAB here)
%     t.t_measure                [s] logging window
%     t.tc_factor                ts_afe/tc: tc is the logging base and the Simscape local solver step
%     t.tc_decimation            decimation of the logging at tc
%     t.delay_pwm                [s] PWM delay used by the modulator model B (0 here)
%     t.dead_time                [s]
%
%   Fields read by interleaved_boost.slx: fPWM_AFE, TRGO_AFE_double_update, dead_time_AFE, ts_afe, ts_dab, tc,
%   decimation_tc, Nc. The names keep the "AFE" of the library object (the clock domain of the converter) so
%   that the blocks and the scripts written for it keep working; there is one converter clock domain only.

    % converter domain (the boost legs)
    glb_time.fPWM_AFE = t.fpwm;
    glb_time.tPWM_AFE = 1/t.fpwm;
    glb_time.TRGO_AFE_double_update = t.trgo;
    glb_time.ts_afe = sample_time(glb_time.tPWM_AFE, t.trgo);
    glb_time.dead_time_AFE = t.dead_time;
    glb_time.minimum_pulse_time_AFE = 1e-6;
    glb_time.delayAFE_modB = 2*pi*t.fpwm*t.delay_pwm;

    % battery estimator sample time (legacy ts_dab of the shared template)
    glb_time.fPWM_DAB = t.fpwm_dab;
    glb_time.tPWM_DAB = 1/t.fpwm_dab;
    glb_time.TRGO_DAB_double_update = t.trgo_dab;
    glb_time.ts_dab = sample_time(glb_time.tPWM_DAB, t.trgo_dab);

    % logging base, also MaxStep of the global solver and step of the Simscape local solver
    glb_time.tc = glb_time.ts_afe/t.tc_factor;
    glb_time.decimation_tc = t.tc_decimation;
    glb_time.t_measure = t.t_measure;
    glb_time.Nc = ceil(t.t_measure/glb_time.tc/t.tc_decimation);
    glb_time.Ns_afe = ceil(t.t_measure/glb_time.ts_afe);

    % discrete-time variable for the design checks
    glb_time.z_afe = tf('z', glb_time.ts_afe);

    fprintf('Timing: fPWM = %.0f Hz, double update = %d, ts = %.1f us, tc = %.4f us, Nc = %d, Ns = %d\n', ...
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
