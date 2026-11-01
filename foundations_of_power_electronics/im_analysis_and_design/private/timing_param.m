function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of im_analysis_and_design (local replacement of timing_setup of the library).
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm, t.trgo             PWM frequency of the inverter [Hz] and double update flag
%                                (1: two control steps per carrier period)
%     t.fpwm_dab, t.trgo_dab     sample time ts_dab of the battery state-of-charge estimator (legacy name
%                                of the shared template: the battery block reads glb_time.ts_dab)
%     t.t_measure                [s] logging window
%     t.tc_factor                ts_inv/tc: tc is the logging base, the MaxStep of the solver and the step of
%                                the Simscape local solver
%     t.tc_decimation            decimation of the logging at tc
%     t.delay_pwm                [s] PWM delay used by the modulator model B (0 here)
%     t.dead_time                [s]
%
%   Fields read by im_gyro_inv_model.slx and im_inv_model.slx: fPWM_INV, dead_time_INV, ts_inv, ts_dab, tc,
%   decimation_tc, Nc, Ns_inv. The legacy init had seven clock domains (afe, inv, single_phase_inv, dab,
%   psfbc, isop_rail, cllc), all at 4 kHz with double update except DAB and CLLC: the models used the INV
%   domain for the modulator and the control, ts_afe and Ns_afe (same values) for two logging blocks and
%   ts_dab for the battery. One converter clock domain here, named INV; the two logging blocks read ts_inv
%   and Ns_inv.

    % converter domain: inverter, modulator and all the control loops
    glb_time.fPWM_INV = t.fpwm;
    glb_time.tPWM_INV = 1/t.fpwm;
    glb_time.TRGO_INV_double_update = t.trgo;
    glb_time.ts_inv = sample_time(glb_time.tPWM_INV, t.trgo);
    glb_time.dead_time_INV = t.dead_time;
    glb_time.minimum_pulse_time_INV = 1e-6;
    glb_time.delayINV_modB = 2*pi*t.fpwm*t.delay_pwm;

    % battery estimator sample time (legacy ts_dab of the shared template)
    glb_time.fPWM_DAB = t.fpwm_dab;
    glb_time.tPWM_DAB = 1/t.fpwm_dab;
    glb_time.TRGO_DAB_double_update = t.trgo_dab;
    glb_time.ts_dab = sample_time(glb_time.tPWM_DAB, t.trgo_dab);

    % logging base, also MaxStep of the global solver and step of the Simscape local solver
    glb_time.tc = glb_time.ts_inv/t.tc_factor;
    glb_time.decimation_tc = t.tc_decimation;
    glb_time.t_measure = t.t_measure;
    glb_time.Nc = ceil(t.t_measure/glb_time.tc/t.tc_decimation);
    glb_time.Ns_inv = ceil(t.t_measure/glb_time.ts_inv);

    % discrete-time variable for the design checks
    glb_time.z_inv = tf('z', glb_time.ts_inv);

    fprintf('Timing: fPWM = %.0f Hz, double update = %d, ts = %.1f us, tc = %.4f us, dead time = %.1f us, Nc = %d, Ns = %d, ts_dab = %.2f us\n', ...
        glb_time.fPWM_INV, glb_time.TRGO_INV_double_update, glb_time.ts_inv*1e6, glb_time.tc*1e6, ...
        glb_time.dead_time_INV*1e6, glb_time.Nc, glb_time.Ns_inv, glb_time.ts_dab*1e6);
end

function ts = sample_time(tpwm, double_update)
    if double_update
        ts = tpwm/2;
    else
        ts = tpwm;
    end
end
