% CREATE_POWER_METER_BUS
% Crea l'oggetto Simulink.Bus da assegnare all'uscita "m" del
% MATLAB Function block three_phase_power_meter_iec.
% Nel Symbols pane impostare Type = Bus: PowerMeterIecBus.

names = {'Q1p','Q1n','P1n','P1p','U1n','U1p', ...
         'i_act_1n','i_react_1n','i_act_1p','i_react_1p'};

elems = Simulink.BusElement.empty(0,1);
for k = 1:numel(names)
    e            = Simulink.BusElement;
    e.Name       = names{k};
    e.Dimensions = 1;
    e.DataType   = 'double';
    e.Complexity = 'real';
    elems(k,1)   = e;
end

PowerMeterIecBus          = Simulink.Bus;
PowerMeterIecBus.Elements = elems;

assignin('base','PowerMeterIecBus',PowerMeterIecBus);
