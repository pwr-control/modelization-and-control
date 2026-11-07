function glb_time = timing_param(t)
%TIMING_PARAM  Timing data of single_phase_pll_analysis: one control period for the three PLLs.
%
%   glb_time = timing_param(t), with t = param.time:
%     t.fpwm, t.trgo   frequency of the modulator timer of the shared template [Hz] and double update flag (1: two
%                      control steps per timer period, 0: one). There is no modulator in this model: the two numbers
%                      only give the control period ts, as in the AFE and inverter models the PLLs come from
%     t.t_measure      [s] logging window (the To Workspace blocks keep the last Ns_afe samples)
%
%   Fields read by single_phase_pll_analysis.slx:
%     ts_afe   period of the FHT-PLL and of the DDSRF-PLL (blocks copied from the AFE control of frt_dqpll)
%     ts_inv   period of the Park PLL (block single_phase_pll, from the single-phase inverter control), of the
%              test signal (Digital Clock) and fixed step of the solver; same value as ts_afe
%     Ns_afe   samples of the logging window, MaxDataPoints of the To Workspace blocks
%   The legacy init built the seven clock domains of the shared template (afe, inv, single_phase_inv, dab, psfbc,
%   isop_rail, cllc) with timing_setup of the library; the model uses ts_afe and ts_inv only, both 125 us.

    glb_time.fPWM = t.fpwm;
    glb_time.tPWM = 1/t.fpwm;
    glb_time.TRGO_double_update = t.trgo;
    if t.trgo
        ts = glb_time.tPWM/2;
    else
        ts = glb_time.tPWM;
    end
    glb_time.ts_afe = ts;
    glb_time.ts_inv = ts;
    glb_time.t_measure = t.t_measure;
    glb_time.Ns_afe = ceil(t.t_measure/ts) + 1;     % samples of [0, t_measure], both ends included

    fprintf('Timing: control period ts_afe = ts_inv = %.1f us (%.0f Hz timer, double update = %d), logging %.3f s (Ns_afe = %d)\n', ...
        ts*1e6, t.fpwm, t.trgo, t.t_measure, glb_time.Ns_afe);
end
