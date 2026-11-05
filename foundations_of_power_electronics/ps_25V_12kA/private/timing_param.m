function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of ps_25V_12kA (local replacement of timing_setup of the library, two converter
%   clock domains).
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm_afe, t.trgo_afe       [Hz] PWM frequency of the two-level AFE bridge and double update flag
%                                  (1: two control steps per carrier period)
%     t.dead_time_afe              [s] dead time of the AFE legs (modulator mask)
%     t.fpwm_psfbc, t.trgo_psfbc   [Hz] switching frequency of the phase-shift full bridge and double update
%                                  flag (modulator full_bridge_zvs_pwm_modulator: two triggers per period)
%     t.dead_time_psfbc            [s] dead time of the full-bridge legs
%     t.fpwm_dab, t.trgo_dab       only for ts_dab: sample time typed in the low-pass filter and in the PI of
%                                  the DC transformer current compensation (they run at ts_psfbc, see the
%                                  README) and in the state-of-charge estimator of the battery block
%                                  (commented out in the model); legacy name of the shared template
%     t.t_measure                  [s] logging window (the To Workspace blocks keep the last t_measure seconds)
%     t.tc_factor                  ts_afe/tc: tc is the logging base, the MaxStep of the solver, the step of
%                                  the Simscape local solver and the time constant of the modulator filters
%     t.tc_decimation              decimation of the logging at tc
%     t.delay_pwm                  [s] PWM delay of the modulator model B (0, not used by this model)
%
%   Fields read by ps_25V_12kA_r.slx: fPWM_AFE, dead_time_AFE, ts_afe (AFE control, logging at ts_afe),
%   fPWM_PSFBC, TRGO_PSFBC_double_update, dead_time_PSFBC, ts_psfbc (current PI of the full bridge), ts_dab,
%   tc, decimation_tc, Nc, Ns_afe, Ns_dab.
%   The legacy init had seven clock domains (afe, inv, single_phase_inv, dab, psfbc, isop_rail, cllc), all at
%   4 kHz with double update except dab and cllc at 12 kHz. The gains of the AFE current PI read
%   glb_time.ts_inv: the inverter domain had the same clock as the AFE, so those gains read
%   param.glb_time.ts_afe now. Same formulas as timing_setup: ts = tPWM/2 with double update,
%   tc = ts_afe/tc_factor, Nc = ceil(t_measure/tc/decimation), Ns = ceil(t_measure/ts).

    % AFE domain: two-level bridge, modulator, all the AFE control loops
    glb_time.fPWM_AFE = t.fpwm_afe;
    glb_time.tPWM_AFE = 1/t.fpwm_afe;
    glb_time.TRGO_AFE_double_update = t.trgo_afe;
    glb_time.ts_afe = sample_time(glb_time.tPWM_AFE, t.trgo_afe);
    glb_time.dead_time_AFE = t.dead_time_afe;
    glb_time.minimum_pulse_time_AFE = 1e-6;
    glb_time.delayAFE_modB = 2*pi*t.fpwm_afe*t.delay_pwm;

    % phase-shift full bridge domain: modulator, output current PI
    glb_time.fPWM_PSFBC = t.fpwm_psfbc;
    glb_time.tPWM_PSFBC = 1/t.fpwm_psfbc;
    glb_time.TRGO_PSFBC_double_update = t.trgo_psfbc;
    glb_time.ts_psfbc = sample_time(glb_time.tPWM_PSFBC, t.trgo_psfbc);
    glb_time.dead_time_PSFBC = t.dead_time_psfbc;
    glb_time.minimum_pulse_time_PSFBC = 1e-6;

    % ts_dab of the shared template (DC transformer current compensation, battery estimator)
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
    glb_time.Ns_psfbc = ceil(t.t_measure/glb_time.ts_psfbc);
    glb_time.Ns_dab = ceil(t.t_measure/glb_time.ts_dab);

    fprintf('Timing: AFE %.0f Hz, double update = %d, ts_afe = %.1f us, dead time %.1f us | full bridge %.0f Hz, double update = %d, ts_psfbc = %.1f us, dead time %.1f us\n', ...
        glb_time.fPWM_AFE, glb_time.TRGO_AFE_double_update, glb_time.ts_afe*1e6, glb_time.dead_time_AFE*1e6, ...
        glb_time.fPWM_PSFBC, glb_time.TRGO_PSFBC_double_update, glb_time.ts_psfbc*1e6, glb_time.dead_time_PSFBC*1e6);
    fprintf('tc = %.4f us, logging window %.3f s (Nc = %d, Ns_afe = %d), ts_dab = %.2f us\n', ...
        glb_time.tc*1e6, glb_time.t_measure, glb_time.Nc, glb_time.Ns_afe, glb_time.ts_dab*1e6);
end

function ts = sample_time(tpwm, double_update)
    if double_update
        ts = tpwm/2;
    else
        ts = tpwm;
    end
end
