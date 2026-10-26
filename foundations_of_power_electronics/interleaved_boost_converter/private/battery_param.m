function data = battery_param(nominal_voltage, nominal_power, charge_capacity, soc_init, ts, plot_ocv)
%BATTERY_PARAM  Lithium-ion pack on the DC link (local replacement of lithium_ion_battery_setup of the
%   library, same formulas and field names).
%
%   data = battery_param(nominal_voltage, nominal_power, charge_capacity, soc_init, ts, plot_ocv)
%     nominal_voltage  : [V]      nominal_power : [W]      charge_capacity : [Ah]
%     soc_init         : [pu] initial state of charge
%     ts               : [s] sample time of the state-of-charge estimator of the block
%     plot_ocv         : true to plot the open-circuit voltage curve
%
%   Fields read by the block battery/lithium-ion_battery of grid_forming_dq.slx: Q, number_of_cells, E_1,
%   E0, E1, E2, E3, Elog, M, R0, R1, C1, alpha, soc_init, q1Kalman, q2Kalman, q3Kalman, rKalman.

    data.ubattery = nominal_voltage;
    data.pnom = nominal_power;
    data.nominal_current = data.pnom/data.ubattery;
    data.nominal_efficiency = 0.95;

    data.typical_cell_voltage = 3.6;
    data.number_of_cells = floor(nominal_voltage/data.typical_cell_voltage) - 1;

    data.soc_init = soc_init;

    data.R = 8.3143;
    data.F = 96487;
    data.T = 273.15 + 40;
    data.Q = charge_capacity;                               % [Ah]

    data.Vbattery_nom = data.ubattery;
    data.Pbattery_nom = data.pnom;
    data.Ibattery_nom = data.Pbattery_nom/data.Vbattery_nom;
    data.Rmax = data.Vbattery_nom^2/(data.Pbattery_nom*0.1);
    data.Rmin = data.Vbattery_nom^2/(data.Pbattery_nom);

    % open-circuit voltage model of the cell: E_1*exp(-alpha*z) + E0 + E1*z + E2*z^2 + E3*z^3 + Elog*log(1 - z)
    data.E_1 = -1.031;
    data.E0 = 3.485;
    data.E1 = 0.2156;
    data.E2 = 0;
    data.E3 = 0;
    data.Elog = -0.05;
    data.alpha = 35;

    data.R0 = 1/2*data.pnom*(1 - data.nominal_efficiency)/data.nominal_current^2;   % [Ohm]
    data.R1 = data.R0;                                                               % [Ohm]
    data.C1 = 0.5;                                                                   % [F]
    data.M = 125;

    data.q1Kalman = ts^2;
    data.q2Kalman = ts^1;
    data.q3Kalman = 0;
    data.rKalman = 1;

    z = (0:1e-3:1);
    data.ocv_model = data.E_1*exp(-z*data.alpha) + data.E0 + data.E1*z + data.E2*z.^2 + ...
        data.E3*z.^3 + data.Elog*log(1 - z + ts);
    if plot_ocv
        figure;
        plot(z, data.ocv_model, 'LineWidth', 2);
        xlabel('state of charge [p.u.]');
        ylabel('open circuit voltage [V]');
        title('open circuit voltage(state of charge)');
        grid on
    end

    fprintf('Battery: %.0f V | %.0f kW | %.0f Ah | %d cells | R0 = %.2f mOhm | soc_init = %.2f\n', ...
        data.ubattery, data.pnom/1e3, data.Q, data.number_of_cells, data.R0*1e3, data.soc_init);
end
