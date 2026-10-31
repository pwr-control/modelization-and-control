function dev = device_mosfet_param(device, fpwm, udc)
%DEVICE_MOSFET_PARAM  MOSFET data set of the three-level T-type bridge (local replacement of
%   device_mosfet_setup / device_mosfet_setting of the library).
%
%   dev = device_mosfet_param(device, fpwm, udc)
%     device : name of the data set
%              - 'danfoss_SKM1700MB20R4S2I4' (SiC MOSFET module, 1700 A, values at 125 degC): embedded
%                below, so the init does not depend on the library data sets;
%              - any other data set of library/setup_files/semiconductor_devices_data (script in
%                devices_source_dataname or its alias): read with device_data_load of the library, which
%                must be on the path (init_model adds it).
%     fpwm   : [Hz] switching frequency, used for the ZVS snubber
%     udc    : [V]  DC-link voltage, used for the ZVS snubber
%
%   Fields read by the mask of the block mosfet_based_three_level_inverter_ttype of single_phase_afe_ttype.slx:
%     Vth, Rds_on, Eon, Eoff, Err, Ion_sw_losses, Voff_sw_losses, Vdon_diode, Rdon_diode, Rth_mosfet_JH,
%     Csnubber, Rsnubber, JunctionTermalMass
%   Same names and derived quantities as the library class device_mosfet_setting. The output keeps every
%   field of the data set.

    switch lower(device)
        case {'danfoss_skm1700mb20r4s2i4', 'skm1700mb20r4s2i4'}
            p = danfoss_SKM1700MB20R4S2I4();
        otherwise
            p = library_dataset(device);
    end

    dev = p;
    if ~isfield(dev, 'Rth_mosfet_JH')
        dev.Rth_mosfet_JH = p.Rtim + p.Rth_mosfet_JC + p.Rth_mosfet_CH;   % [K/W]
    end
    dev.Csnubber_zvs = p.Irr^2*p.Lstray_module/udc^2;                   % [F]
    dev.Rsnubber_zvs = 1/(dev.Csnubber_zvs*fpwm)/5;                     % [Ohm]

    fprintf('MOSFET data set: %s (%s)\n', dev.name, dev.type);
    fprintf('Rds_on = %.2f mOhm, Vth = %.1f V | Eon+Eoff = %.0f mJ, Err = %.1f mJ at %.0f V / %.0f A | Rth_mosfet_JH = %.1f mK/W\n', ...
        dev.Rds_on*1e3, dev.Vth, (dev.Eon + dev.Eoff)*1e3, dev.Err*1e3, dev.Voff_sw_losses, dev.Ion_sw_losses, ...
        dev.Rth_mosfet_JH*1e3);
    fprintf('---------------------------\n');
end

function p = library_dataset(device)
% Data set script of library/setup_files/semiconductor_devices_data, read with device_data_load (the script runs
% in the workspace of that function and its variables become the fields of p). The names of the data set,
% device_name and device_type, become name and type as in the embedded data sets; the fields required by the
% bridge mask are checked here, so that a data set in another format fails at init time and not in the model.
    if exist('device_data_load', 'file') ~= 2
        error('device_mosfet_param:library', ...
              ['Data set "%s" is not embedded in device_mosfet_param and device_data_load of the library is not on the ' ...
               'path: add library/setup_files to the path (init_model adds C:\\github\\library).'], device);
    end
    p = device_data_load(device);
    p.name = p.device_name;
    p.type = p.device_type;
    p = rmfield(p, {'device_name', 'device_type'});
    required = {'Vth', 'Rds_on', 'Vdon_diode', 'Rdon_diode', 'Eon', 'Eoff', 'Err', 'Voff_sw_losses', 'Ion_sw_losses', ...
                'JunctionTermalMass', 'Rtim', 'Rth_mosfet_JC', 'Rth_mosfet_CH', 'Lstray_module', 'Irr', 'Csnubber', 'Rsnubber'};
    missing = required(~isfield(p, required));
    if ~isempty(missing)
        error('device_mosfet_param:missingField', 'Data set %s: missing field(s) %s', p.name, strjoin(missing, ', '));
    end
end

function p = danfoss_SKM1700MB20R4S2I4()
% SKM1700MB20R4S2I4, SiC MOSFET half-bridge module (data of the library script
% semiconductor_devices_data/devices_source_dataname/danfoss_SKM1700MB20R4S2I4.m, energies at Tj = 125 degC)
    p.name = 'danfoss_SKM1700MB20R4S2I4';
    p.type = 'SiC MOSFET';
    p.Vth = 5.5;                            % [V]
    p.Rds_on = 1.04e-3;                     % [Ohm]
    p.g_fs = 900;                           % [A/V]
    p.Vdon_diode = 4;                       % [V]
    p.Vgamma = 4;                           % [V]
    p.Rdon_diode = 1.85e-3;                 % [Ohm]
    p.Eon = 77e-3;                          % [J] at Tj = 125 degC
    p.Eoff = 108e-3;                        % [J]
    p.Err = 9.7e-3;                         % [J]
    p.Voff_sw_losses = 1300;                % [V] test voltage of the energies
    p.Ion_sw_losses = 1000;                 % [A] test current of the energies
    p.JunctionTermalMass = 1;               % [J/K]
    p.Rtim = 0.01;                          % [K/W] thermal interface material
    p.Rth_mosfet_JC = 19/1000;              % [K/W]
    p.Rth_mosfet_CH = 6/1000;               % [K/W]
    p.Rth_mosfet_JH = p.Rtim + p.Rth_mosfet_JC + p.Rth_mosfet_CH;   % [K/W]
    p.Lstray_module = 12e-9;                % [H]
    p.Lstray_d = p.Lstray_module/2;         % [H]
    p.RLd = 0;                              % [Ohm]
    p.Lstray_s = p.Lstray_module/2;         % [H]
    p.RLs = 0;                              % [Ohm]
    p.Ciss = 184e-9;                        % [F]
    p.Coss = 3.28e-9;                       % [F]
    p.Crss = 0.14e-9;                       % [F]
    p.Cgs = p.Ciss - p.Crss;                % [F]
    p.Cgd = p.Crss;                         % [F]
    p.Cds = p.Coss - p.Crss;                % [F]
    p.Rgate_internal = 0.6*1.75;            % [Ohm]
    p.Irr = 475;                            % [A]
    p.Csnubber = 12e-12;                    % [F]
    p.Rsnubber = 2200;                      % [Ohm]
end
