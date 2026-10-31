function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of afe_dq_ttype (local replacement of timing_setup of the library).
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm, t.trgo             PWM frequency of the three-level T-type bridge [Hz] and double update flag
%                                (1: two control steps per carrier period)
%     t.t_measure                [s] logging window
%     t.tc_factor                ts_afe/tc: tc is the logging base and the Simscape local solver step
%     t.tc_decimation            decimation of the logging at tc
%     t.delay_pwm                [s] PWM delay used by the modulator model B (0 here)
%     t.dead_time                [s] dead time of the T-type bridge (deadtime_generator_for_ttype)
%
%   Fields read by afe_dq_ttype.slx: fPWM_AFE, TRGO_AFE_double_update, dead_time_AFE, ts_afe, tc, Nc,
%   Ns_afe. The legacy init had seven clock domains (afe, inv, single_phase_inv, dab, psfbc, isop_rail,
%   cllc); the model used the AFE domain everywhere except the gains of the current PI, which read
%   glb_time.ts_inv: the inverter domain had the same frequency (fpwm_inv = fpwm_afe, double update), so
%   those gains read param.glb_time.ts_afe now and there is one clock domain.
%   Same formulas as timing_setup: ts = tPWM/2 with double update, tc = ts/tc_factor,
%   Nc = ceil(t_measure/tc/decimation), Ns_afe = ceil(t_measure/ts).

    % converter domain: the three-level bridge and all the control loops
    glb_time.fPWM_AFE = t.fpwm;
    glb_time.tPWM_AFE = 1/t.fpwm;
    glb_time.TRGO_AFE_double_update = t.trgo;
    glb_time.ts_afe = sample_time(glb_time.tPWM_AFE, t.trgo);
    glb_time.dead_time_AFE = t.dead_time;
    glb_time.minimum_pulse_time_AFE = 1e-6;
    glb_time.delayAFE_modB = 2*pi*t.fpwm*t.delay_pwm;

    % logging base, also MaxStep of the global solver and step of the Simscape local solver
    glb_time.tc = glb_time.ts_afe/t.tc_factor;
    glb_time.decimation_tc = t.tc_decimation;
    glb_time.t_measure = t.t_measure;
    glb_time.Nc = ceil(t.t_measure/glb_time.tc/t.tc_decimation);
    glb_time.Ns_afe = ceil(t.t_measure/glb_time.ts_afe);

    % discrete-time variable for the design checks
    glb_time.z_afe = tf('z', glb_time.ts_afe);

    fprintf('Timing: fPWM = %.0f Hz, double update = %d, ts = %.1f us, tc = %.4f us, dead time = %.1f us, Nc = %d, Ns = %d\n', ...
        glb_time.fPWM_AFE, glb_time.TRGO_AFE_double_update, glb_time.ts_afe*1e6, glb_time.tc*1e6, ...
        glb_time.dead_time_AFE*1e6, glb_time.Nc, glb_time.Ns_afe);
end

function ts = sample_time(tpwm, double_update)
    if double_update
        ts = tpwm/2;
    else
        ts = tpwm;
    end
end
