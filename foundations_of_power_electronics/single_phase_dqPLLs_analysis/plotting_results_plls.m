function plotting_results_plls(source, ids)
%PLOTTING_RESULTS_PLLS  Plots of the three PLLs of single_phase_pll_analysis: input, phase error, frequency.
%
%   plotting_results_plls                  last run of the model (logs and param in the base workspace)
%   plotting_results_plls(file)            every case of a file saved by run_test_cases
%   plotting_results_plls(file, ids)       the cases ids of that file, e.g. [1 3]
%   plotting_results_plls(results, ids)    the same, from the output of run_test_cases
%
%   One figure per case with three panels: the test signal u [pu]; the phase error of each PLL, phase - theta_g
%   wrapped to +-180 deg; the pulsation of each PLL and of the input [pu]. The legend gives the final error and its
%   peak-to-peak ripple (pll_metrics of ./private); the command window gets the whole table.
%   Logs of the model (To Workspace blocks of data_logging, one row per control period):
%     sig_log [t u theta_g omega_g], park_pll_log, fht_pll_log, ddsrf_pll_log [phase omega]
%   After a run with the Run button they are in the base workspace (or in out, when the model returns a
%   Simulink.SimulationOutput).

    if nargin < 1 || isempty(source)
        results = from_base();
    elseif ischar(source) || isstring(source)
        s = load(source, 'results');
        results = s.results;
    else
        results = source;
    end
    if nargin >= 2 && ~isempty(ids)
        results = results(ismember([results.id], ids));
    end
    names = {'park', 'fht', 'ddsrf'};
    titles = {'Park PLL', 'FHT-PLL', 'DDSRF-PLL'};
    colors = [0 0.447 0.741; 0.850 0.325 0.098; 0.466 0.674 0.188];
    for k = 1:numel(results)
        r = results(k);
        figure('Name', sprintf('PLLs, case %d', r.id));
        tl = tiledlayout(3, 1, 'TileSpacing', 'compact');
        title(tl, sprintf('Case %d: %s', r.id, r.name), 'Interpreter', 'none');

        nexttile;
        plot(r.t, r.u, 'k');
        grid on
        ylabel('u [pu]');

        nexttile;
        hold on
        leg = cell(1, numel(names));
        mts = cell(1, numel(names));
        for j = 1:numel(names)
            p = r.(names{j});
            mts{j} = pll_metrics(r.t, r.theta_g, r.omega_g, p.phase, p.omega, r.test);
            plot(r.t, mts{j}.err, 'Color', colors(j, :));
            leg{j} = sprintf('%s: end %.2f deg, ripple %.2f deg', titles{j}, mts{j}.end_mean, mts{j}.end_pp);
        end
        hold off
        grid on
        ylim([-30 30]);
        ylabel('phase error [deg]');
        legend(leg, 'Location', 'best');

        nexttile;
        hold on
        plot(r.t, r.omega_g, 'k--');
        for j = 1:numel(names)
            plot(r.t, r.(names{j}).omega, 'Color', colors(j, :));
        end
        hold off
        grid on
        ylim([0.9 1.1]);
        ylabel('\omega [pu]');
        xlabel('t [s]');
        legend([{'input'}, titles], 'Location', 'best');

        fprintf('Case %d: %s\n', r.id, r.name);
        fprintf('  %-10s %8s %9s %8s %8s %8s %9s %8s %11s\n', 'PLL', 'lock [s]', 'pre [deg]', 'pp [deg]', 'peak', ...
            'settle', 'end [deg]', 'pp [deg]', 'omega err');
        for j = 1:numel(names)
            mt = mts{j};
            fprintf('  %-10s %8.3f %9.2f %8.2f %8.2f %8.3f %9.2f %8.2f %11.2e\n', titles{j}, mt.lock_time, ...
                mt.pre_mean, mt.pre_pp, mt.peak, mt.settle, mt.end_mean, mt.end_pp, mt.w_end_mean);
        end
    end
end

function results = from_base()
%FROM_BASE  The last run of the model, from the base workspace.
    logs = {'sig_log', 'park_pll_log', 'fht_pll_log', 'ddsrf_pll_log'};
    v = struct();
    if evalin('base', 'exist(''sig_log'', ''var'')')
        for k = 1:numel(logs)
            v.(logs{k}) = evalin('base', logs{k});
        end
    elseif evalin('base', 'exist(''out'', ''var'') && isa(out, ''Simulink.SimulationOutput'')')
        out = evalin('base', 'out');
        for k = 1:numel(logs)
            v.(logs{k}) = out.get(logs{k});
        end
    else
        error('plotting_results_plls:nolog', 'no log of single_phase_pll_analysis in the base workspace: run the model first');
    end
    param = evalin('base', 'param');
    results.id = param.sim.test_case;
    results.name = param.sim.test_name;
    results.test = param.test;
    results.t = v.sig_log(:, 1);
    results.u = v.sig_log(:, 2);
    results.theta_g = v.sig_log(:, 3);
    results.omega_g = v.sig_log(:, 4);
    names = {'park', 'fht', 'ddsrf'};
    for k = 1:numel(names)
        y = v.(logs{k + 1});
        p.phase = y(:, 1);
        p.omega = y(:, 2);
        results.(names{k}) = p;
    end
end
