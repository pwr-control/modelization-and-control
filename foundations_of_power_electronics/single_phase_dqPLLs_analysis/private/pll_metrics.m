function m = pll_metrics(t, theta_g, omega_g, phase, omega, test)
%PLL_METRICS  Phase and frequency errors of one PLL on a test of single_phase_pll_analysis.
%
%   m = pll_metrics(t, theta_g, omega_g, phase, omega, test)
%     t, theta_g, omega_g   time [s], angle of the fundamental [rad] and its pulsation [pu] (block test_signal)
%     phase, omega          outputs of the PLL: angle [rad] and pulsation [pu]
%     test                  param.test of the run (event times, prv.test_param)
%   The error e = phase - theta_g is wrapped to (-180, 180] deg. Fields of m (deg, s, pu):
%     err                  e for every sample [deg]
%     lock_time            first time after which |e| stays below 5 deg until t_event (case 1 starts 180 deg
%                          away from the PLLs)
%     t_event              first event of the test (sag, phase jump, frequency step, DC offset), Inf when none
%     pre_mean, pre_pp     mean and peak-to-peak of e in the 0.1 s before t_event (in the last 0.2 s when
%                          there is no event)
%     peak                 largest |e - pre_mean| from t_event on
%     settle               time from t_event until e stays within 1 deg of its final mean (NaN: never)
%     end_mean, end_pp     mean and peak-to-peak of e in the last 0.2 s
%     w_end_mean, w_end_pp mean and peak-to-peak of omega - omega_g in the last 0.2 s [pu]
%   Called by run_test_cases.m and plotting_results_plls.m.

    t = t(:);
    e = angle(exp(1j*(phase(:) - theta_g(:))))*180/pi;
    m.err = e;
    t_end = t(end);

    m.t_event = min([test.t_sag, test.t_jump, test.t_fstep, test.t_dc]);
    before = find(t < m.t_event);                       % the start, up to the first event
    out = find(abs(e(before)) >= 5, 1, 'last');
    if isempty(out)
        m.lock_time = t(1);
    elseif out < numel(before)
        m.lock_time = t(out + 1);
    else
        m.lock_time = NaN;
    end

    if isfinite(m.t_event)
        pre = t >= m.t_event - 0.1 & t < m.t_event;
    else
        pre = t >= t_end - 0.2;
    end
    m.pre_mean = mean(e(pre));
    m.pre_pp = max(e(pre)) - min(e(pre));

    fin = t >= t_end - 0.2;
    m.end_mean = mean(e(fin));
    m.end_pp = max(e(fin)) - min(e(fin));
    dw = omega(:) - omega_g(:);
    m.w_end_mean = mean(dw(fin));
    m.w_end_pp = max(dw(fin)) - min(dw(fin));

    if isfinite(m.t_event)
        post = t >= m.t_event;
        m.peak = max(abs(e(post) - m.pre_mean));
        away = find(post & abs(e - m.end_mean) > 1, 1, 'last');
        if isempty(away)
            m.settle = 0;
        elseif away < numel(t) && t(away) < t_end - 0.2
            m.settle = t(away + 1) - m.t_event;
        else
            m.settle = NaN;
        end
    else
        m.peak = NaN;
        m.settle = NaN;
    end
end
