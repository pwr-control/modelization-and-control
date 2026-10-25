function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of grid_forming_dq (local replacement of timing_setup of the library).
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm, t.trgo             AFE PWM frequency [Hz] and double update flag (1: two updates per period)
%     t.fpwm_inv, t.trgo_inv     second clock domain (block "global timing"/time_inv1), same convention
%     t.fpwm_dab, t.trgo_dab     sample time ts_dab of the battery estimator (legacy name, no DAB here)
%     t.t_measure                [s] logging window
%     t.tc_factor                ts_afe/tc: tc is the logging base and the Simscape local solver step
%     t.tc_decimation            decimation of the logging at tc
%     t.delay_pwm                [s] PWM delay used by the modulator model B
%     t.dead_time, t.dead_time_inv   [s]
%
%   Fields read by grid_forming_dq.slx: fPWM_AFE, TRGO_AFE_double_update, dead_time_AFE, ts_afe, ts_inv,
%   ts_dab, tc, decimation_tc, Nc, Ns_afe. The other fields keep the names of the library object so that
%   the scripts written for it keep working.

    % AFE domain
    glb_time.fPWM_AFE = t.fpwm;
    glb_time.tPWM_AFE = 1/t.fpwm;
    glb_time.TRGO_AFE_double_update = t.trgo;
    glb_time.ts_afe = sample_time(glb_time.tPWM_AFE, t.trgo);
    glb_time.dead_time_AFE = t.dead_time;
    glb_time.minimum_pulse_time_AFE = 1e-6;
    glb_time.delayAFE_modB = 2*pi*t.fpwm*t.delay_pwm;

    % second clock domain (time_inv1 of the "global timing" subsystem)
    glb_time.fPWM_INV = t.fpwm_inv;
    glb_time.tPWM_INV = 1/t.fpwm_inv;
    glb_time.TRGO_INV_double_update = t.trgo_inv;
    glb_time.ts_inv = sample_time(glb_time.tPWM_INV, t.trgo_inv);
    glb_time.dead_time_INV = t.dead_time_inv;
    glb_time.minimum_pulse_time_INV = 1e-6;
    glb_time.delayINV_modB = 2*pi*t.fpwm_inv*t.delay_pwm;

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
    glb_time.Ns_inv = ceil(t.t_measure/glb_time.ts_inv);

    % discrete-time variables for the design checks
    glb_time.z_afe = tf('z', glb_time.ts_afe);
    glb_time.z_inv = tf('z', glb_time.ts_inv);

    fprintf('Timing: fPWM_AFE = %.0f Hz, double update = %d, ts_afe = %.1f us, tc = %.4f us, Nc = %d, Ns_afe = %d\n', ...
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
