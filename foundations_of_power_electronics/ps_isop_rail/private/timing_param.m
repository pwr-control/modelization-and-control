function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of ps_isop_rail (local copy of timing_setup of the library, one clock domain).
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm, t.trgo             switching frequency of the four full bridges [Hz] (phase-shift modulator
%                                isop/phase_shift_full_bridge_ctrl/full_bridge_zvs_pwm_modulator) and double
%                                update flag (1: the modulator gives two control triggers per period)
%     t.dead_time                [s] dead time of each leg of the full bridges
%     t.fpwm_dab, t.trgo_dab     only for ts_dab: sample time of the state-of-charge estimator of the battery
%                                block and coefficient of the filter and of the PI of the DC-current
%                                compensation (legacy name of the shared template, see the README)
%     t.t_measure                [s] logging window (the To Workspace blocks keep the last t_measure seconds)
%     t.tc_factor                ts_isop_rail/tc: tc is the logging base, the MaxStep of the solver, the step
%                                of the Simscape local solver and the time constant of the modulator filters
%     t.tc_decimation            decimation of the logging at tc
%
%   Fields read by ps_isop_rail.slx: fPWM_ISOP_RAIL, TRGO_ISOP_RAIL_double_update, dead_time_ISOP_RAIL,
%   ts_isop_rail, Ns_isop_rail, ts_dab, Ns_dab, tc, decimation_tc, Nc.
%   The legacy init had seven clock domains (afe, inv, single_phase_inv, dab, psfbc, isop_rail, cllc). The
%   model used three names for the same clock of the full bridges: PSFBC (modulator, current PI), ISOP_RAIL
%   (voltage PI) and AFE (one logging block). Same frequency, same double update and same dead time, so they
%   are now one domain, ISOP_RAIL.

    % converter domain: modulator of the four full bridges, control trigger, voltage and current PI
    glb_time.fPWM_ISOP_RAIL = t.fpwm;
    glb_time.tPWM_ISOP_RAIL = 1/t.fpwm;
    glb_time.TRGO_ISOP_RAIL_double_update = t.trgo;
    glb_time.ts_isop_rail = sample_time(glb_time.tPWM_ISOP_RAIL, t.trgo);
    glb_time.dead_time_ISOP_RAIL = t.dead_time;
    glb_time.minimum_pulse_time_ISOP_RAIL = 1e-6;

    % battery estimator and DC-current compensation (legacy ts_dab of the shared template)
    glb_time.fPWM_DAB = t.fpwm_dab;
    glb_time.tPWM_DAB = 1/t.fpwm_dab;
    glb_time.TRGO_DAB_double_update = t.trgo_dab;
    glb_time.ts_dab = sample_time(glb_time.tPWM_DAB, t.trgo_dab);

    % logging base, also MaxStep of the global solver and step of the Simscape local solver
    glb_time.tc = glb_time.ts_isop_rail/t.tc_factor;
    glb_time.decimation_tc = t.tc_decimation;
    glb_time.t_measure = t.t_measure;
    glb_time.Nc = ceil(t.t_measure/glb_time.tc/t.tc_decimation);
    glb_time.Ns_isop_rail = ceil(t.t_measure/glb_time.ts_isop_rail);
    glb_time.Ns_dab = ceil(t.t_measure/glb_time.ts_dab);

    fprintf('Timing: full bridges at %.0f Hz, double update = %d, control ts = %.1f us, tc = %.3f us, dead time = %.1f us, logging window %.3f s (Nc = %d, Ns = %d), ts_dab = %.2f us\n', ...
        glb_time.fPWM_ISOP_RAIL, glb_time.TRGO_ISOP_RAIL_double_update, glb_time.ts_isop_rail*1e6, glb_time.tc*1e6, ...
        glb_time.dead_time_ISOP_RAIL*1e6, glb_time.t_measure, glb_time.Nc, glb_time.Ns_isop_rail, glb_time.ts_dab*1e6);
end

function ts = sample_time(tpwm, double_update)
    if double_update
        ts = tpwm/2;
    else
        ts = tpwm;
    end
end
