function results = run_test_cases(cases, mat_file)
%RUN_TEST_CASES  Simulate single_phase_pll_analysis on the test signals of private/test_param.m and compare the PLLs.
%
%   results = run_test_cases                  cases 1 to 6
%   results = run_test_cases(cases)           e.g. [2 3]
%   results = run_test_cases(cases, file)     also save results to file (e.g. 'sim_results_cases.mat')
%
%   Runs init_model in the base workspace when param is not there, then for each case sets param.test
%   (test_param of ./private), simulates the model and reads the logs of the To Workspace blocks
%   (data_logging, Array format, one row per control period ts):
%     sig_log         [t u theta_g omega_g]   test signal: time, voltage [pu], angle of the fundamental, omega [pu]
%     park_pll_log    [phase omega]           Park PLL (block single_phase_pll: phase_pll_hat, omega_pll_hat)
%     fht_pll_log     [phase omega]           FHT-PLL
%     ddsrf_pll_log   [phase omega]           DDSRF-PLL
%   results(k): id, name, test, t, u, theta_g, omega_g and, for park, fht and ddsrf, the fields phase, omega and the
%   metrics of pll_metrics (./private): lock time, error before the event, peak, settling time, final error and
%   ripple, final frequency error. One table per case is printed. plotting_results_plls(file, id) plots a saved
%   case. param in the base workspace is left with the test case of the init.

    if nargin < 1 || isempty(cases)
        cases = 1:6;
    end
    if nargin < 2
        mat_file = '';
    end
    model = 'single_phase_pll_analysis';
    if ~evalin('base', 'exist(''param'', ''var'') && isstruct(param) && isfield(param, ''test'')')
        evalin('base', 'init_model');
    end
    load_system(model);
    param0 = evalin('base', 'param');
    pll_names = {'park', 'fht', 'ddsrf'};
    pll_logs = {'park_pll_log', 'fht_pll_log', 'ddsrf_pll_log'};
    pll_titles = {'Park PLL', 'FHT-PLL', 'DDSRF-PLL'};

    results = struct([]);
    restore = onCleanup(@() assignin('base', 'param', param0));
    for k = 1:numel(cases)
        id = cases(k);
        param = param0;
        [param.test, param.sim.test_name] = test_param(id, param.grid.emu.omega_grid_nom);
        param.sim.test_case = id;
        assignin('base', 'param', param);
        fprintf('\n%s: case %d, %s (%.2f s)\n', model, id, param.sim.test_name, param.sim.simlength);
        tic;
        out = sim(model, 'ReturnWorkspaceOutputs', 'on');
        fprintf('  simulated in %.1f s\n', toc);
        sig = out.get('sig_log');
        r.id = id;
        r.name = param.sim.test_name;
        r.test = param.test;
        r.t = sig(:, 1);
        r.u = sig(:, 2);
        r.theta_g = sig(:, 3);
        r.omega_g = sig(:, 4);
        fprintf('  %-10s %8s %9s %8s %8s %8s %9s %8s %11s\n', 'PLL', 'lock [s]', 'pre [deg]', 'pp [deg]', ...
            'peak', 'settle', 'end [deg]', 'pp [deg]', 'omega err');
        for j = 1:numel(pll_names)
            y = out.get(pll_logs{j});
            p.phase = y(:, 1);
            p.omega = y(:, 2);
            p.metrics = pll_metrics(r.t, r.theta_g, r.omega_g, p.phase, p.omega, r.test);
            r.(pll_names{j}) = p;
            mt = p.metrics;
            fprintf('  %-10s %8.3f %9.2f %8.2f %8.2f %8.3f %9.2f %8.2f %11.2e\n', pll_titles{j}, mt.lock_time, ...
                mt.pre_mean, mt.pre_pp, mt.peak, mt.settle, mt.end_mean, mt.end_pp, mt.w_end_mean);
        end
        if isempty(results)
            results = r;
        else
            results(end + 1) = r; %#ok<AGROW>
        end
        clear r p
    end
    if ~isempty(mat_file)
        save(mat_file, 'results');
        fprintf('\n%s: results of cases %s saved to %s\n', model, mat2str(cases), mat_file);
    end
end
