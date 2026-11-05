function hw = afe_hw_param(application_voltage, pwr_nom, fpwm, hw_in)
%AFE_HW_PARAM  Hardware data of the three-phase two-level AFE of ps_25V_12kA_r.slx: DC link, LC filter on
%   the grid side, brake resistor, per-unit bases.
%
%   hw = afe_hw_param(application_voltage, pwr_nom, fpwm, hw_in)
%     application_voltage : [V] rms line-to-line grid voltage: 690, 480 or 400 (any other value: 400 V set)
%     pwr_nom             : [W] nominal power of the converter
%     fpwm                : [Hz] switching frequency (stored in fpwm_base)
%     hw_in.CFi, hw_in.RCFi : [F], [Ohm] DC-link capacitor of this model and its series resistance. The data
%                           sets of the library have 7.2 mF; the legacy init replaced CFi with 2*Cdc_dc1 of the
%                           PSFB data set (2.4 mF) after the setup, and the model reads CFi.
%
%   Local replacement of three_phase_afe_hwdata + hw_afe_three_phase_setup of the library: same data sets,
%   same formulas and same field names, so that the per-unit scaling of the model does not change. Fields
%   read by ps_25V_12kA_r.slx: ubez, ibez, udc_nom, CFi, LFu, LFu_cm, RLFu, RLFu_cm, CFu, RCFu, Rbrake.
%
%   Bases (convention of the library data set):
%     ubez = us_nom*sqrt(2/3)              peak phase-to-neutral grid voltage (391.9 V at 480 V)
%     is_nom = pwr_nom/sqrt(3)/us_nom/0.9  rms current with a 10 % margin on the power (434.3 A at 325 kW)
%     ibez = is_nom*sqrt(2)                peak current base (614.2 A); 1 pu of active current at 1 pu of
%                                          voltage is 3/2*ubez*ibez = 361.1 kW
%     udc_nom                              DC link (750 V at 480 V), base of udc_pu
%
%   DC link: one capacitor CFi (block dclink_for_two_levels/dclink: Cdc = CFi, measurement divider 1 MOhm
%   typed in the block; the input capacitor Cdc_dc1 of the full bridge is in parallel on the same bus).
%   Filter (block LCL_filter): three-phase inductor LFu (positive sequence) / LFu_cm (zero sequence) with
%   RLFu, RLFu_cm, then CFu in star with RCFu in series (floating star point), then the secondary of the grid
%   transformer. Brake chopper (block brake): Rbrake across the DC link through two ideal switches.

    switch application_voltage
        case 690
            hw.name = 'afe690V';
            hw.us_nom = 690;                % [V] rms line-to-line
            hw.f_nom = 50;                  % [Hz]
            hw.udc_nom = 1070;              % [V]
            LFu = 500e-6; LFu_cm = 500e-6;  % [H]
            hw.CFu = 200e-6;                % [F]
            hw.Rbrake = 4;                  % [Ohm]
        case 480
            hw.name = 'afe480V';
            hw.us_nom = 480;
            hw.f_nom = 60;
            hw.udc_nom = 750;
            LFu = 330e-6; LFu_cm = 330e-6;
            hw.CFu = 360e-6;
            hw.Rbrake = 2.5;
        otherwise
            hw.name = 'afe400V';
            hw.us_nom = 400;
            hw.f_nom = 50;
            hw.udc_nom = 660;
            LFu = 330e-6; LFu_cm = 330e-6;
            hw.CFu = 360e-6;
            hw.Rbrake = 2.5;
    end
    hw.application_voltage = application_voltage;
    hw.pwr_nom = pwr_nom;                                   % [W]
    hw.fpwm_base = fpwm;                                    % [Hz]

    % bases (hw_afe_three_phase_setup)
    hw.is_nom = pwr_nom/sqrt(3)/hw.us_nom/0.9;              % [A] rms, 10 % margin on the power
    hw.ubez = hw.us_nom*sqrt(2/3);                          % [V] peak phase-to-neutral
    hw.ibez = hw.is_nom*sqrt(2);                            % [A] peak
    hw.udc_bez = hw.udc_nom;                                % [V]
    hw.omega_bez = 2*pi*hw.f_nom;                           % [rad/s]
    hw.norm.Xbez = hw.ubez/hw.ibez;                         % [Ohm]
    hw.norm.Lbez = hw.norm.Xbez/hw.omega_bez;               % [H]
    hw.norm.Cbez = 1/(hw.norm.Xbez*hw.omega_bez);           % [F]
    hw.norm.Pbez = 3/2*hw.ubez*hw.ibez;                     % [W] power at 1 pu of voltage and current

    % DC link (block dclink_for_two_levels/dclink: one capacitor, no ESR in the block)
    hw.CFi = hw_in.CFi;                                     % [F]
    hw.RCFi = hw_in.RCFi;                                   % [Ohm] (informative)
    hw.CFi1 = 2*hw.CFi;                                     % [F] banks of the library data set (informative)
    hw.CFi2 = 2*hw.CFi;
    hw.RCFi1 = hw.RCFi/2;                                   % [Ohm] (informative)
    hw.RCFi2 = hw.RCFi/2;

    % grid filter (block LCL_filter: three-phase inductor, CFu in star, then the transformer secondary)
    hw.LFu = LFu;                                           % [H] positive-sequence inductance
    hw.LFu_cm = LFu_cm;                                     % [H] zero-sequence inductance
    hw.RLFu = 157*0.05*LFu;                                 % [Ohm] 5 % of the reactance at 25 Hz (library rule)
    hw.RLFu_cm = 157*0.05*LFu_cm;                           % [Ohm]
    hw.RCFu = 50e-3;                                        % [Ohm] series resistance of each capacitor

    fprintf('Hardware three-phase two-level AFE: %s, %.0f kW, %.0f V, %.0f Hz | ubez = %.1f V, ibez = %.1f A (is_nom %.1f A rms), 1 pu = %.1f kW | udc_nom = %.0f V\n', ...
        hw.name, pwr_nom/1e3, hw.us_nom, hw.f_nom, hw.ubez, hw.ibez, hw.is_nom, hw.norm.Pbez/1e3, hw.udc_nom);
    fprintf('DC link CFi = %.1f mF | filter LFu = %.0f uH (%.3f pu), CFu = %.0f uF star (%.3f pu), RCFu = %.0f mOhm | Rbrake = %.1f Ohm\n', ...
        hw.CFi*1e3, hw.LFu*1e6, hw.LFu/hw.norm.Lbez, hw.CFu*1e6, hw.CFu/hw.norm.Cbez, hw.RCFu*1e3, hw.Rbrake);
    fprintf('---------------------------\n');
end
