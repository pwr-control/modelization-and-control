function bus = power_meter_bus()
%POWER_METER_BUS  Simulink.Bus PowerMeterIecBus for the output m of the MATLAB Function block
%   three_phase_power_meter_iec (grid/power meter iec mfunc; Symbols pane: Type = Bus: PowerMeterIecBus).
%
%   bus = power_meter_bus()   the bus object; it is also assigned in the base workspace, where the model looks for
%                             it (bus objects do not live in a struct).
%
%   Same code as the section CREATE_POWER_METER_BUS of afe_uvw_three_phase/init_model.m and as frt_dqpll (the
%   legacy init of vienna_rectifier did not build it): ten real double scalars, Q1p, Q1n, P1n, P1p, U1n, U1p,
%   i_act_1n, i_react_1n, i_act_1p, i_react_1p.

    names = {'Q1p', 'Q1n', 'P1n', 'P1p', 'U1n', 'U1p', ...
        'i_act_1n', 'i_react_1n', 'i_act_1p', 'i_react_1p'};

    elems = Simulink.BusElement.empty(0, 1);
    for k = 1:numel(names)
        e            = Simulink.BusElement;
        e.Name       = names{k};
        e.Dimensions = 1;
        e.DataType   = 'double';
        e.Complexity = 'real';
        elems(k, 1)  = e;
    end

    bus          = Simulink.Bus;
    bus.Elements = elems;
    assignin('base', 'PowerMeterIecBus', bus);
end
