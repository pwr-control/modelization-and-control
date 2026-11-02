function dev = device_mosfet_param(device, fpwm, udc)
%DEVICE_MOSFET_PARAM  MOSFET data set of the two full bridges of single_phase_dab.slx (local copy of
%   device_mosfet_setup / device_mosfet_setting of the library, same field names and formulas).
%
%   dev = device_mosfet_param(device, fpwm, udc)
%     device : name of the data set
%              - 'danfoss_SKM1700MB20R4S2I4' (2 kV SiC MOSFET half-bridge module): embedded below, so the init
%                does not depend on the data sets of the library;
%              - any other data set of library/setup_files/semiconductor_devices_data: read with device_data_load
%                of the library, which must be on the path (init_model puts the library on the path).
%     fpwm   : [Hz] switching frequency, used only for the ZVS snubber resistor
%     udc    : [V]  DC-link voltage, used only for the ZVS snubber capacitor
%
%   ZVS snubber of the library (as in device_mosfet_setting):
%     Csnubber_zvs = Irr^2*Lstray_module/udc^2,   Rsnubber_zvs = 1/(Csnubber_zvs*fpwm)/5
%   The blocks full-bridge_mosfet_based_with_thermal_model_zvs put Csnubber_zvs in series with Rsnubber_zvs
%   across each MOSFET (see the README: with these numbers the resistor is too large for a ZVS capacitor).
%
%   Fields read by the bridges of dab_modA/dcdc_with_galvanic_isolation/full-bridge 1 and full-bridge 2
%   (full-bridge_mosfet_based_with_thermal_model_zvs, active, and full-bridge mosfet based with thermal model,
%   commented): Rth_mosfet_JH, Rds_on, Vth, Eon, Eoff, Voff_sw_losses, Ion_sw_losses, Vdon_diode,
%   JunctionTermalMass, Csnubber_zvs, Rsnubber_zvs (and Rds_on again for Rdon_diode_m, as in the legacy model).

    switch lower(device)
        case 'danfoss_skm1700mb20r4s2i4'
            p.device_name = 'danfoss_SKM1700MB20R4S2I4';
            p.device_type = 'SiC MOSFET';
            p.Vth = 5.5;                                % [V]
            p.Rds_on = 1.04e-3;                         % [Ohm]
            p.g_fs = 900;                               % [A/V]
            p.Vdon_diode = 4;                           % [V]
            p.Vgamma = 4;                               % [V]
            p.Rdon_diode = 1.85e-3;                     % [Ohm]
            p.Eon = 77e-3;                              % [J] at Tj = 125 degC, 1300 V, 1000 A
            p.Eoff = 108e-3;                            % [J] at Tj = 125 degC
            p.Err = 9.7e-3;                             % [J] at Tj = 125 degC
            p.Voff_sw_losses = 1300;                    % [V]
            p.Ion_sw_losses = 1000;                     % [A]
            p.JunctionTermalMass = 1;                   % [J/K]
            p.Rtim = 0.01;                              % [K/W]
            p.Rth_mosfet_JC = 19/1000;                  % [K/W]
            p.Rth_mosfet_CH = 6/1000;                   % [K/W]
            p.Rth_mosfet_JH = p.Rtim + p.Rth_mosfet_JC + p.Rth_mosfet_CH;   % [K/W]
            p.Lstray_module = 12e-9;                    % [H]
            p.Lstray_d = p.Lstray_module/2;             % [H]
            p.RLd = 0;                                  % [Ohm]
            p.Lstray_s = p.Lstray_module/2;             % [H]
            p.RLs = 0;                                  % [Ohm]
            p.Ciss = 184e-9;                            % [F]
            p.Coss = 3.28e-9;                           % [F]
            p.Crss = 0.14e-9;                           % [F]
            p.Cgs = p.Ciss - p.Crss;                    % [F]
            p.Cgd = p.Crss;                             % [F]
            p.Cds = p.Coss - p.Crss;                    % [F]
            p.Rgate_internal = 0.6*1.75;                % [Ohm]
            p.Irr = 475;                                % [A]
            p.Csnubber = 12e-12;                        % [F]
            p.Rsnubber = 2200;                          % [Ohm]
        otherwise
            if exist('device_data_load', 'file') ~= 2
                error('device_mosfet_param:library', ...
                      ['Data set "%s" is not embedded in device_mosfet_param and device_data_load of the library ' ...
                       'is not on the path: put the library on the path (init_model does it with library_setup).'], device);
            end
            p = device_data_load(device);
    end

    fields = {'Vth', 'Rds_on', 'g_fs', 'Vdon_diode', 'Vgamma', 'Rdon_diode', 'Eon', 'Eoff', 'Err', ...
        'Voff_sw_losses', 'Ion_sw_losses', 'JunctionTermalMass', 'Rtim', 'Rth_mosfet_JC', 'Rth_mosfet_CH', ...
        'Rth_mosfet_JH', 'Lstray_module', 'Lstray_d', 'RLd', 'Lstray_s', 'RLs', 'Ciss', 'Coss', 'Crss', 'Cgs', ...
        'Cgd', 'Cds', 'Rgate_internal', 'Irr', 'Csnubber', 'Rsnubber'};
    dev.name = p.device_name;
    dev.type = p.device_type;
    for k = 1:numel(fields)
        if isfield(p, fields{k})
            dev.(fields{k}) = p.(fields{k});
        else
            dev.(fields{k}) = NaN;                  % optional field missing in the data set
        end
    end
    dev.Csnubber_zvs = dev.Irr^2*dev.Lstray_module/udc^2;       % [F]
    dev.Rsnubber_zvs = 1/(dev.Csnubber_zvs*fpwm)/5;             % [Ohm]

    fprintf(['MOSFET data set: %s | Rds_on = %.2f mOhm, Vth = %.1f V | body diode %.1f V + %.2f mOhm | ' ...
        'Eon/Eoff = %.0f/%.0f mJ at %.0f V, %.0f A | Coss = %.2f nF | ZVS snubber %.2f nF + %.0f Ohm\n'], ...
        dev.name, dev.Rds_on*1e3, dev.Vth, dev.Vdon_diode, dev.Rdon_diode*1e3, dev.Eon*1e3, dev.Eoff*1e3, ...
        dev.Voff_sw_losses, dev.Ion_sw_losses, dev.Coss*1e9, dev.Csnubber_zvs*1e9, dev.Rsnubber_zvs);
end
