function dev = device_igbt_param(device, fpwm, udc)
%DEVICE_IGBT_PARAM  IGBT data set of the bridge (local replacement of device_igbt_setup / device_igbt_setting).
%
%   dev = device_igbt_param(device, fpwm, udc)
%     device : name of the data set
%              - 'infineon_FF450R17ME4' (1700 V / 450 A EconoDUAL 3, the module of the 200 V -> 660 V, 750 A
%                version) or any other data set of library/setup_files/semiconductor_devices_data (script
%                in devices_source_dataname or its alias, e.g. 'infineon_IGBT_FF450R17ME4'): read with
%                device_data_load of the library, which must be on the path (init_model adds it);
%              - 'mitsubishi_CM1200DW_34T' (1700 V, as in the legacy init) or 'infineon_FF1800R12IE5'
%              (1200 V, the module of the 750 V -> 400 V buck): the two data sets embedded below.
%     fpwm   : [Hz] switching frequency, used for the ZVS snubber
%     udc    : [V]  DC-link voltage, used for the ZVS snubber
%
%   Fields read by the mask of afe_dq_mod1/afe/three_phase_inverter_with_igbt_simple_2th:
%     Vce_sat, Vdon_diode, Vth, Eon, Eoff, Erec, Ion_sw_losses, Voff_sw_losses, Rce_on, Rdon_diode,
%     Rth_switch_JH, Rth_diode_JH, JunctionTermalMass, Csnubber_zvs, Rsnubber_zvs
%   Same names and derived quantities as the library class device_igbt_setting. The output keeps every field of
%   the data set (the library scripts add tables, the Foster networks and an igbt_dyn_param struct in dev.extra
%   style, here left as fields): numeric, except name and type, which the init removes before storing the struct
%   in param.boost (mask struct of the compact block).

    switch lower(device)
        case {'mitsubishi_cm1200dw_34t', 'mitsubishi_igbt_cm1200dw_34t'}
            p = mitsubishi_CM1200DW_34T();
        case {'infineon_ff1800r12ie5', 'infineon_igbt_ff1800r12ie5', 'infineon_ff1800r12ie5p'}
            p = infineon_FF1800R12IE5();
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

function p = mitsubishi_CM1200DW_34T()
% CM1200DW-34T, 1700 V / 1200 A dual IGBT (data of the library script, Rg = 1 Ohm energies)
    p.name = 'mitsubishi_CM1200DW-34T';
    p.type = 'Si-IGBT';
    p.Vth = 6.0;                            % [V]
    p.Rce_on = 0.25e-3;                     % [Ohm]
    p.Vce_sat = 2.40;                       % [V]
    p.Vdon_diode = 2.80;                    % [V]
    p.Rdon_diode = 0.25e-3;                 % [Ohm]
    p.Eon = 420e-3;                         % [J] at Tj = 125 degC, Rg = 1 Ohm
    p.Eoff = 310e-3;                        % [J]
    p.Erec = 140e-3;                        % [J]
    p.Voff_sw_losses = 1000;                % [V] test voltage of the energies
    p.Ion_sw_losses = 1200;                 % [A] test current of the energies
    p.JunctionTermalMass = 0.65;            % [J/K]
    p.Rtim = 1.0/1000;                      % [K/W] thermal interface material
    p.Rth_switch_JC = 28.0/1000;            % [K/W]
    p.Rth_switch_CH = 20.0/1000;            % [K/W]
    p.Rth_diode_JC = 43.0/1000;             % [K/W]
    p.Rth_diode_CH = 20.0/1000;             % [K/W]
    p.Lstray_module = 18e-9;                % [H]
    p.Qrr = 72e-6;                          % [C]
    p.trr = 300e-9;                         % [s]
    p.Irr = 2*p.Qrr/p.trr;                  % [A]
    p.Cies = 330e-9;                        % [F]
    p.Coes = 8.7e-9;                        % [F]
    p.Cres = 2.8e-9;                        % [F]
    p.Rgate_internal = 0.67;                % [Ohm]
    p.td_on = 0.8e-6;                       % [s]
    p.trise = 0.20e-6;                      % [s]
    p.td_off = 0.80e-6;                     % [s]
    p.tfall = 0.60e-6;                      % [s]
    p.Csnubber = 12e-12;                    % [F]
    p.Rsnubber = 2200;                      % [Ohm]
end

function p = infineon_FF1800R12IE5()
% FF1800R12IE5(P), 1200 V / 1800 A dual IGBT (data of the library script)
    p.name = 'infineon_FF1800R12IE5';
    p.type = 'Si-IGBT';
    p.Vth = 5.5;                            % [V]
    p.Rce_on = 0.15e-3;                     % [Ohm]
    p.Vce_sat = 2.00;                       % [V]
    p.Vdon_diode = 1.75;                    % [V]
    p.Rdon_diode = 0.15e-3;                 % [Ohm]
    p.Eon = 195e-3;                         % [J] at Tj = 125 degC
    p.Eoff = 260e-3;                        % [J]
    p.Erec = 145e-3;                        % [J]
    p.Voff_sw_losses = 600;                 % [V]
    p.Ion_sw_losses = 1800;                 % [A]
    p.JunctionTermalMass = 0.2;             % [J/K]
    p.Rtim = 0.01;                          % [K/W]
    p.Rth_switch_JC = 17.3/1000;            % [K/W]
    p.Rth_switch_CH = 11.4/1000;            % [K/W]
    p.Rth_diode_JC = 28.3/1000;             % [K/W]
    p.Rth_diode_CH = 13.2/1000;             % [K/W]
    p.Lstray_module = 18e-9;                % [H]
    p.Irr = 1150;                           % [A]
    p.Cies = 54e-9;                         % [F]
    p.Cres = 1.7e-9;                        % [F]
    p.Rgate_internal = 2.3;                 % [Ohm]
    p.td_on = 0.6e-6;                       % [s]
    p.trise = 0.11e-6;                      % [s]
    p.td_off = 1.25e-6;                     % [s]
    p.tfall = 0.49e-6;                      % [s]
    p.Csnubber = 12e-12;                    % [F]
    p.Rsnubber = 2200;                      % [Ohm]
    p.gate_charge = 8.65e-6;                % [C]
end
