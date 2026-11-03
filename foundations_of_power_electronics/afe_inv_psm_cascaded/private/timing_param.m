function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of afe_inv_psm_n (local replacement of timing_setup of the library): two clock
%   domains, the AFE and the inverter of each module.
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm_afe, t.trgo_afe     PWM frequency of the AFE [Hz] and double update flag (1: two control steps
%                                per carrier period)
%     t.fpwm_inv, t.trgo_inv     the same for the inverter
%     t.dead_time_afe            [s] dead time of the PWM generator of the AFE
%     t.dead_time_inv            [s] dead time of the PWM generator of the inverter
%     t.t_measure                [s] logging window (the To Workspace blocks keep the last t_measure seconds)
%     t.tc_factor                ts_afe/tc: tc is the logging base, the MaxStep of the solver and the step of
%                                the Simscape local solver
%     t.tc_decimation            decimation of the logging at tc
%     t.delay_pwm                [s] PWM delay of the modulator model B (0 here)
%
%   Fields read by afe_inv_psm_n.slx (logging, Unit Delays of global timing, Simscape local solver): tc,
%   decimation_tc, Nc, ts_afe, Ns_afe. The PWM and the control of the four library blocks read their own
%   p.pwm and p.ctrl (param.afe_follower, param.inv_mod), whose periods init_model.m checks against these.
%   The legacy init had seven clock domains (afe, inv, single_phase_inv, dab, psfbc, isop_rail, cllc); the
%   model uses only the AFE and the INV domains. Same formulas as timing_setup: ts = tPWM/2 with double
%   update, tc = ts_afe/tc_factor, Nc = ceil(t_measure/tc/decimation), Ns = ceil(t_measure/ts).

    % AFE domain: grid-side bridge, its modulator and the AFE control
    glb_time.fPWM_AFE = t.fpwm_afe;
    glb_time.tPWM_AFE = 1/t.fpwm_afe;
    glb_time.TRGO_AFE_double_update = t.trgo_afe;
    glb_time.ts_afe = sample_time(glb_time.tPWM_AFE, t.trgo_afe);
    glb_time.dead_time_AFE = t.dead_time_afe;
    glb_time.minimum_pulse_time_AFE = 1e-6;
    glb_time.delayAFE_modB = 2*pi*t.fpwm_afe*t.delay_pwm;

    % INV domain: machine-side bridge, its modulator and the machine control
    glb_time.fPWM_INV = t.fpwm_inv;
    glb_time.tPWM_INV = 1/t.fpwm_inv;
    glb_time.TRGO_INV_double_update = t.trgo_inv;
    glb_time.ts_inv = sample_time(glb_time.tPWM_INV, t.trgo_inv);
    glb_time.dead_time_INV = t.dead_time_inv;
    glb_time.minimum_pulse_time_INV = 1e-6;
    glb_time.delayINV_modB = 2*pi*t.fpwm_inv*t.delay_pwm;

    % logging base, also MaxStep of the global solver and step of the Simscape local solver
    glb_time.tc = glb_time.ts_afe/t.tc_factor;
    glb_time.decimation_tc = t.tc_decimation;
    glb_time.t_measure = t.t_measure;
    glb_time.Nc = ceil(t.t_measure/glb_time.tc/t.tc_decimation);
    glb_time.Ns_afe = ceil(t.t_measure/glb_time.ts_afe);
    glb_time.Ns_inv = ceil(t.t_measure/glb_time.ts_inv);

    fprintf('Timing: AFE fPWM = %.0f Hz, double update = %d, ts = %.1f us, dead time = %.1f us | INV fPWM = %.0f Hz, double update = %d, ts = %.1f us, dead time = %.1f us\n', ...
        glb_time.fPWM_AFE, glb_time.TRGO_AFE_double_update, glb_time.ts_afe*1e6, glb_time.dead_time_AFE*1e6, ...
        glb_time.fPWM_INV, glb_time.TRGO_INV_double_update, glb_time.ts_inv*1e6, glb_time.dead_time_INV*1e6);
    fprintf('        tc = %.4f us, logging window %.3f s (Nc = %d, Ns_afe = %d, Ns_inv = %d)\n', ...
        glb_time.tc*1e6, glb_time.t_measure, glb_time.Nc, glb_time.Ns_afe, glb_time.Ns_inv);
end

function ts = sample_time(tpwm, double_update)
    if double_update
        ts = tpwm/2;
    else
        ts = tpwm;
    end
end
