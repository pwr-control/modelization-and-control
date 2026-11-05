function dev = device_igbt_param(device, fpwm, udc)
%DEVICE_IGBT_PARAM  IGBT data set of the bridges of ps_25V_12kA_r.slx (local replacement of device_igbt_setup /
%   device_igbt_setting of the library).
%
%   dev = device_igbt_param(device, fpwm, udc)
%     device : name of the data set
%              - 'mitsubishi_CM1200DW_34T' (1700 V / 1200 A dual IGBT, the module of this model, energies
%                at 125 degC with Rg = 1 Ohm): embedded below, so the init does not depend on the library
%                data sets;
%              - any other data set of library/setup_files/semiconductor_devices_data (script in
%                devices_source_dataname or its alias, e.g. 'infineon_FF1200R17IP5'): read with
%                device_data_load of the library, which must be on the path (prv.library_setup of init_model adds it).
%     fpwm   : [Hz] switching frequency, used for the ZVS snubber
%     udc    : [V]  DC-link voltage, used for the ZVS snubber
%
%   Fields read by the masks of the IGBT bridges of ps_25V_12kA_r.slx, full_bridge_igbt_adv_thermal_model
%   (phase-shift full bridge, enabled by param.sim.use_thermal_model) and three_phase_inverter_igbt_adv_thermal_model
%   (AFE, commented out in the model): Vth, Vce_sat, Rce_on, Eon, Eoff, Erec, Irr, Ion_sw_losses, Voff_sw_losses,
%   Vdon_diode, Rdon_diode, Csnubber, Rsnubber, JunctionTermalMass, Lstray_module, Cies, Rth_switch_JH,
%   Rth_diode_JH, Rth_diode_CH (the AFE block reads Rth_diode_CH as its diode-to-heatsink resistance).
%   The design document of the model (docs/ps_25V_12kA_r.pdf) names the CM1200DW-24T; the legacy init used the
%   34T data set, kept here (the 24T data set of the library is available through device_data_load).
%   Same function as in afe_dq_ttype_three_phase, except the error message that points to prv.library_setup.
%   Same names and derived quantities as the library class device_igbt_setting. The output keeps every
%   field of the data set.

    switch lower(device)
        case {'mitsubishi_cm1200dw_34t', 'mitsubishi_igbt_cm1200dw_34t'}
            p = mitsubishi_CM1200DW_34T();
        otherwise
            p = library_dataset(device);
    end

    dev = p;
    dev.Rth_switch_JH = p.Rtim + p.Rth_switch_JC + p.Rth_switch_CH;     % [K/W]
    dev.Rth_diode_JH = p.Rtim + p.Rth_diode_JC + p.Rth_diode_CH;        % [K/W]
    dev.Csnubber_zvs = p.Irr^2*p.Lstray_module/udc^2;                   % [F]
    dev.Rsnubber_zvs = 1/(dev.Csnubber_zvs*fpwm)/5;                     % [Ohm]

    fprintf('IGBT data set: %s (%s)\n', dev.name, dev.type);
    fprintf('Vce_sat = %.2f V + %.2f mOhm, Vd = %.2f V + %.2f mOhm | Eon+Eoff = %.0f mJ, Erec = %.0f mJ at %.0f V / %.0f A | Rth_switch_JH = %.1f mK/W, Rth_diode_JH = %.1f mK/W\n', ...
        dev.Vce_sat, dev.Rce_on*1e3, dev.Vdon_diode, dev.Rdon_diode*1e3, (dev.Eon + dev.Eoff)*1e3, dev.Erec*1e3, ...
        dev.Voff_sw_losses, dev.Ion_sw_losses, dev.Rth_switch_JH*1e3, dev.Rth_diode_JH*1e3);
    fprintf('---------------------------\n');
end

function p = library_dataset(device)
% Data set script of library/setup_files/semiconductor_devices_data, read with device_data_load (the script runs
% in the workspace of that function and its variables become the fields of p). The names of the data set,
% device_name and device_type, become name and type as in the embedded data set; the fields required by the
% bridge mask are checked here, so that a data set in another format fails at init time and not in the model.
    if exist('device_data_load', 'file') ~= 2
        error('device_igbt_param:library', ...
              ['Data set "%s" is not embedded in device_igbt_param and device_data_load of the library is not on the ' ...
               'path: run prv.library_setup of init_model, which puts pwr-control/library on the path.'], device);
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
% CM1200DW-34T, 1700 V / 1200 A dual IGBT (data of the library script
% semiconductor_devices_data/devices_source_dataname/mitsubishi_CM1200DW_34T.m, energies at Tj = 125 degC
% with Rg = 1 Ohm; the script keeps the Rg = 0 Ohm set commented: Eon 140 mJ, Eoff 310 mJ, Erec 220 mJ)
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
    p.Irr = 2*p.Qrr/p.trr;                  % [A] 480 A
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
