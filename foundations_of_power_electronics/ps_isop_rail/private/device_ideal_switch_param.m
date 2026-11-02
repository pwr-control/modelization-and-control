function dev = device_ideal_switch_param(device, fpwm, udc)
%DEVICE_IDEAL_SWITCH_PARAM  Ideal-switch data set of the four full bridges of ps_isop_rail.slx (local copy
%   of device_ideal_switch_setting of the library, same names and formulas).
%
%   dev = device_ideal_switch_param(device, fpwm, udc)
%     device : name of the data set
%              - 'silicon_high_power_ideal_switch': embedded below, so the init does not depend on the
%                library data sets;
%              - any other data set of library/setup_files/semiconductor_devices_data: read with
%                device_data_load of the library, which must be on the path (init_model adds it).
%     fpwm   : [Hz] switching frequency, used for the ZVS snubber (informative)
%     udc    : [V]  DC-link voltage, used for the ZVS snubber (informative)
%
%   Fields read by the library block full-bridge ideal switch based (isop/fb_iso_mod_1..4/full_bridge/
%   full-bridge_ideal_switch_based): Rds_on, Vth, Vdon_diode, Rdon_diode, Csnubber, Rsnubber.
%   Same function as in inv_psm.

    switch lower(device)
        case 'silicon_high_power_ideal_switch'
            p.device_name = 'ideal_power_switch';
            p.Vth = 5.5;                        % [V] gate threshold of the controlled switch
            p.Rds_on = 1.5e-3;                  % [Ohm]
            p.Vdon_diode = 2;                   % [V]
            p.Vgamma = p.Vdon_diode;            % [V]
            p.Rdon_diode = 0.15e-3;             % [Ohm]
            p.Irr = 900;                        % [A]
            p.Csnubber = 10e-9;                 % [F]
            p.Rsnubber = 2200;                  % [Ohm]
            p.Lstray_module = 18e-9;            % [H]
        otherwise
            if exist('device_data_load', 'file') ~= 2
                error('device_ideal_switch_param:library', ...
                      ['Data set "%s" is not embedded in device_ideal_switch_param and device_data_load of the library ' ...
                       'is not on the path: add library/setup_files to the path (init_model adds C:\\github\\library).'], device);
            end
            p = device_data_load(device);
    end

    dev.device_name = p.device_name;
    dev.Vth = p.Vth;                                            % [V]
    dev.Rds_on = p.Rds_on;                                      % [Ohm]
    dev.Vdon_diode = p.Vdon_diode;                              % [V]
    dev.Vgamma = p.Vgamma;                                      % [V]
    dev.Rdon_diode = p.Rdon_diode;                              % [Ohm]
    dev.Csnubber = p.Csnubber;                                  % [F]
    dev.Rsnubber = p.Rsnubber;                                  % [Ohm]
    dev.Irr = p.Irr;                                            % [A]
    dev.Lstray_module = p.Lstray_module;                        % [H]
    dev.Csnubber_zvs = dev.Irr^2*dev.Lstray_module/udc^2;       % [F]
    dev.Rsnubber_zvs = 1/(dev.Csnubber_zvs*fpwm)/5;             % [Ohm]

    fprintf('Ideal switch data set: %s | Rds_on = %.2f mOhm, Vth = %.1f V | diode %.1f V + %.2f mOhm | snubber %.0f Ohm + %.0f nF\n', ...
        device, dev.Rds_on*1e3, dev.Vth, dev.Vdon_diode, dev.Rdon_diode*1e3, dev.Rsnubber, dev.Csnubber*1e9);
end
