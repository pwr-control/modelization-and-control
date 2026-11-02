function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of single_phase_dab (local copy of the DAB part of timing_setup of the library).
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm_dab, t.trgo_dab   [Hz] switching frequency of the two full bridges, which is also the frequency
%                              of the transformer (modulator dab_modA/dab_ctrl/dab_single_phase_pwm_modulator),
%                              and double update flag (1: the modulator gives two control triggers per
%                              period, TRGO_DAB_1, at the peak and at the valley of its carrier)
%     t.dead_time_dab          [s] dead time of each leg (applied by the modulator)
%     t.tc                     [s] logging base, MaxStep of the solver, step of the Simscape local solver
%                              (block battery_1/Solver Configuration) and time constant of the three filters
%                              of the modulator
%     t.t_measure              [s] logging window: the To Workspace blocks keep the last t_measure seconds
%     t.tc_decimation          decimation of the logging at tc
%
%   Fields read by single_phase_dab.slx: fPWM_DAB, TRGO_DAB_double_update, dead_time_DAB, ts_dab, tc,
%   decimation_tc, Nc. The legacy init built the timing object of the shared template with seven clock
%   domains (afe, inv, single_phase_inv, dab, psfbc, isop_rail, cllc); this model uses only the DAB domain
%   and tc, so the other domains are gone. Same names and formulas as timing_setup.

    glb_time.fPWM_DAB = t.fpwm_dab;
    glb_time.tPWM_DAB = 1/t.fpwm_dab;
    glb_time.TRGO_DAB_double_update = t.trgo_dab;
    glb_time.ts_dab = sample_time(glb_time.tPWM_DAB, t.trgo_dab);    % period of the control (triggered)
    glb_time.dead_time_DAB = t.dead_time_dab;

    % logging base, also MaxStep of the global solver and step of the Simscape local solver
    glb_time.tc = t.tc;
    glb_time.decimation_tc = t.tc_decimation;
    glb_time.t_measure = t.t_measure;
    glb_time.Nc = ceil(t.t_measure/glb_time.tc/t.tc_decimation);
    glb_time.Ns_dab = ceil(t.t_measure/glb_time.ts_dab);

    fprintf(['Timing: bridges and transformer at %.1f kHz, double update = %d, control ts = %.2f us, ' ...
        'dead time = %.2f us, tc = %.0f ns (%.0f steps per period), logging window %.3f s (Nc = %d)\n'], ...
        glb_time.fPWM_DAB/1e3, glb_time.TRGO_DAB_double_update, glb_time.ts_dab*1e6, ...
        glb_time.dead_time_DAB*1e6, glb_time.tc*1e9, glb_time.tPWM_DAB/glb_time.tc, glb_time.t_measure, ...
        glb_time.Nc);
end

function ts = sample_time(tpwm, double_update)
    if double_update
        ts = tpwm/2;
    else
        ts = tpwm;
    end
end
