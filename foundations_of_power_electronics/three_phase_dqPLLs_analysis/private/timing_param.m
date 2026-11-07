function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of frt_dqpll (local copy of timing_setup of the library, AFE clock domain only).
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm, t.trgo       frequency of the modulator timer [Hz] and double update flag (1: two control steps per
%                          timer period, 0: one). The modulator (sv_pwm/modulator1) runs with zero references: it
%                          gives the control trigger TRGO of AFE_control1 and the clock of the acquisitions
%     t.dead_time          [s] dead time of the modulator (there is no bridge in this model)
%     t.t_measure          [s] logging window
%     t.tc_factor          ts_afe/tc: tc is the logging base, the MaxStep of the solver, the step of the Simscape
%                          local solver (grid_emulator/Solver Configuration) and the time constant of the first
%                          order filters td2tc on the emulator voltages
%     t.tc_decimation      decimation of the logging at tc
%     t.delay_pwm          [s] PWM delay of the modulator model B (0)
%
%   Fields read by frt_dqpll.slx: fPWM_AFE, TRGO_AFE_double_update, dead_time_AFE, ts_afe, tc. The legacy init
%   built the seven clock domains of the shared template (afe, inv, single_phase_inv, dab, psfbc, isop_rail, cllc)
%   with timing_setup of the library; the model uses the AFE domain only, with the same values: ts_afe = 125 us,
%   tc = ts_afe/200 = 0.625 us. Nc and Ns_afe are the sample counts of the logging window (plotting scripts).

    glb_time.fPWM_AFE = t.fpwm;
    glb_time.tPWM_AFE = 1/t.fpwm;
    glb_time.TRGO_AFE_double_update = t.trgo;
    if t.trgo
        glb_time.ts_afe = glb_time.tPWM_AFE/2;
    else
        glb_time.ts_afe = glb_time.tPWM_AFE;
    end
    glb_time.dead_time_AFE = t.dead_time;
    glb_time.minimum_pulse_time_AFE = 1e-6;
    glb_time.delayAFE_modB = 2*pi*t.fpwm*t.delay_pwm;

    % logging base, also MaxStep of the global solver and step of the Simscape local solver
    glb_time.tc = glb_time.ts_afe/t.tc_factor;
    glb_time.decimation_tc = t.tc_decimation;
    glb_time.t_measure = t.t_measure;
    glb_time.Nc = ceil(t.t_measure/glb_time.tc/t.tc_decimation);
    glb_time.Ns_afe = ceil(t.t_measure/glb_time.ts_afe);

    fprintf('Timing: modulator %.0f Hz, double update = %d, ts_afe = %.2f us, tc = %.4f us, dead time = %.1f us, logging window %.3f s (Nc = %d, Ns_afe = %d)\n', ...
        glb_time.fPWM_AFE, glb_time.TRGO_AFE_double_update, glb_time.ts_afe*1e6, glb_time.tc*1e6, ...
        glb_time.dead_time_AFE*1e6, glb_time.t_measure, glb_time.Nc, glb_time.Ns_afe);
end
