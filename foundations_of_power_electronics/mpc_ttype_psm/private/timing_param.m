function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of mpc_ttype_psm (local copy of timing_setup of the library, one clock domain).
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm, t.trgo             frequency of the PWM timer of the inverter [Hz] and double update flag
%                                (1: two control steps per timer period). The model has no PWM: the timer
%                                (block inv_psm_mod1/inverter/timer) only gives the trigger of the control,
%                                and the predictive control writes the gate commands directly.
%     t.fpwm_dab, t.trgo_dab     only for ts_dab, the sample time of the state-of-charge estimator of the
%                                battery block (legacy name of the shared template)
%     t.t_measure                [s] logging window (the To Workspace blocks keep the last t_measure seconds)
%     t.tc_factor                ts_inv/tc: tc is the logging base, the MaxStep of the solver, the step of the
%                                Simscape local solver and the time constant of the gate filters
%     t.tc_decimation            decimation of the logging at tc
%     t.delay_pwm                [s] PWM delay of the modulator model B (0, not read by this model)
%     t.dead_time                [s] dead time of the gate generator (deadtime_generator_for_ttype)
%
%   Fields read by mpc_ttype_psm.slx: fPWM_INV, TRGO_INV_double_update, dead_time_INV, ts_inv, ts_dab, tc,
%   Nc, Ns_inv. The legacy init had seven clock domains (afe, inv, single_phase_inv, dab, psfbc, isop_rail,
%   cllc). The model uses only the INV domain, plus ts_dab for the battery and one logging block that read
%   Ns_afe (same value as Ns_inv: same frequency, same double update); that block now reads Ns_inv.

    % converter domain: control trigger, gate generator and all the control loops
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

    fprintf('Timing: timer %.0f Hz, double update = %d, ts = %.1f us, tc = %.3f us, dead time = %.1f us, logging window %.3f s (Nc = %d, Ns = %d), ts_dab = %.2f us\n', ...
        glb_time.fPWM_INV, glb_time.TRGO_INV_double_update, glb_time.ts_inv*1e6, glb_time.tc*1e6, ...
        glb_time.dead_time_INV*1e6, glb_time.t_measure, glb_time.Nc, glb_time.Ns_inv, glb_time.ts_dab*1e6);
end

function ts = sample_time(tpwm, double_update)
    if double_update
        ts = tpwm/2;
    else
        ts = tpwm;
    end
end
