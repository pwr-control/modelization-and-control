function dev = device_igbt_param(device, fpwm, udc)
%DEVICE_IGBT_PARAM  IGBT data set of the two full bridges (local replacement of device_igbt_setup /
%   device_igbt_setting of the library).
%
%   dev = device_igbt_param(device, fpwm, udc)
%     device : name of the data set
%              - 'infineon_FF1200R17IP5' (1700 V / 1200 A PrimePACK 3, the module of this model, values
%                at 125 degC): embedded below, so the init does not depend on the library data sets;
%              - any other data set of library/setup_files/semiconductor_devices_data (script in
%                devices_source_dataname or its alias, e.g. 'infineon_IGBT_FF1200XTR17T2P5'): read with
%                device_data_load of the library, which must be on the path (init_model adds it).
%     fpwm   : [Hz] switching frequency, used for the ZVS snubber
%     udc    : [V]  DC-link voltage, used for the ZVS snubber
%
%   Fields read by the mask of the two full_bridge_with_igbt_simple_2th blocks of single_phase_afe.slx:
%     Vce_sat, Vdon_diode, Vth, Eon, Eoff, Erec, Ion_sw_losses, Voff_sw_losses, Rce_on, Rdon_diode,
%     Rth_switch_JH, Rth_diode_JH, JunctionTermalMass, Csnubber_zvs, Rsnubber_zvs
%   Same names and derived quantities as the library class device_igbt_setting. The output keeps every
%   field of the data set.

    switch lower(device)
        case {'infineon_ff1200r17ip5', 'infineon_igbt_ff1200r17ip5'}
            p = infineon_FF1200R17IP5();
        otherwise
            p = library_dataset(device);
    end

    dev = p;
    dev.Rth_switch_JH = p.Rtim + p.Rth_switch_JC + p.Rth_switch_CH;     % [K/W]
    dev.Rth_diode_JH = p.Rtim + p.Rth_diode_JC + p.Rth_diode_CH;        % [K/W]
    dev.Csnubber_zvs = p.Irr^2*p.Lstray_module/udc^2;                   % [F]
    dev.Rsnubber_zvs = 1/(dev.Csnubber_zvs*fpwm)/5;                     % [Ohm]

    fprintf('IGBT data set: %s (%s)\n', dev.name, dev.type);
    fprintf('Vce_sat = %.2f V + %.2f mOhm | Eon+Eoff = %.0f mJ, Erec = %.0f mJ at %.0f V / %.0f A | Rth_switch_JH = %.1f mK/W, Rth_diode_JH = %.1f mK/W\n', ...
        dev.Vce_sat, dev.Rce_on*1e3, (dev.Eon + dev.Eoff)*1e3, dev.Erec*1e3, dev.Voff_sw_losses, dev.Ion_sw_losses, ...
        dev.Rth_switch_JH*1e3, dev.Rth_diode_JH*1e3);
    fprintf('---------------------------\n');
end

function p = library_dataset(device)
% Data set script of library/setup_files/semiconductor_devices_data, read with device_data_load (the script runs
% in the workspace of that function and its variables become the fields of p). The names of the data set,
% device_name and device_type, become name and type as in the embedded data sets; the fields required by the
% bridge mask are checked here, so that a data set in another format fails at init time and not in the model.
    if exist('device_data_load', 'file') ~= 2
        error('device_igbt_param:library', ...
              ['Data set "%s" is not embedded in device_igbt_param and device_data_load of the library is not on the ' ...
               'path: add library/setup_files to the path (init_model adds C:\\github\\library).'], device);
    end
    p = device_data_load(device);
    p.name = p.device_name;
    p.type = p.device_type;
    p = rmfield(p, {'device_name', 'device_type'});
    required = {'Vth', 'Rce_on', 'Vce_sat', 'Vdon_diode', 'Rdon_diode', 'Eon', 'Eoff', 'Erec', 'Voff_sw_losses', ...
                'Ion_sw_losses', 'JunctionTermalMass', 'Rtim', 'Rth_switch_JC', 'Rth_switch_CH', 'Rth_diode_JC', ...
                'Rth_diode_CH', 'Lstray_module', 'Irr', 'Csnubber', 'Rsnubber'};
    missing = required(~isfield(p, required));
    if ~isempty(missing)
        error('device_igbt_param:missingField', 'Data set %s: missing field(s) %s', p.name, strjoin(missing, ', '));
    end
end

function p = infineon_FF1200R17IP5()
% FF1200R17IP5, 1700 V / 1200 A dual IGBT PrimePACK 3 (data of the library script
% semiconductor_devices_data/devices_source_dataname/infineon_FF1200R17IP5.m, energies at Tj = 125 degC)
    p.name = 'infineon_FF1200R17IP5';
    p.type = 'Si-IGBT';
    p.Vth = 5.5;                            % [V]
    p.Rce_on = 0.15e-3;                     % [Ohm]
    p.Vce_sat = 2.15;                       % [V]
    p.Vdon_diode = 1.75;                    % [V]
    p.Rdon_diode = 0.15e-3;                 % [Ohm]
    p.Eon = 400e-3;                         % [J] at Tj = 125 degC
    p.Eoff = 400e-3;                        % [J]
    p.Erec = 245e-3;                        % [J]
    p.Voff_sw_losses = 900;                 % [V] test voltage of the energies
    p.Ion_sw_losses = 1200;                 % [A] test current of the energies
    p.JunctionTermalMass = 1.0;             % [J/K]
    p.Rtim = 4.0/1000;                      % [K/W] thermal interface material
    p.Rth_switch_JC = 24.3/1000;            % [K/W]
    p.Rth_switch_CH = 19.6/1000;            % [K/W]
    p.Rth_diode_JC = 43.9/1000;             % [K/W]
    p.Rth_diode_CH = 22.9/1000;             % [K/W]
    p.Lstray_module = 18e-9;                % [H]
    p.Irr = 1100;                           % [A]
    p.Cies = 54e-9;                         % [F]
    p.Cres = 1.7e-9;                        % [F]
    p.Rgate_internal = 2.3;                 % [Ohm]
    p.td_on = 0.6e-6;                       % [s]
    p.trise = 0.11e-6;                      % [s]
    p.td_off = 1.25e-6;                     % [s]
    p.tfall = 0.49e-6;                      % [s]
    p.Csnubber = 12e-12;                    % [F]
    p.Rsnubber = 2200;                      % [Ohm]
end
