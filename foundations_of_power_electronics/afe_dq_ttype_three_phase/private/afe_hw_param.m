function hw = afe_hw_param(application_voltage, pwr_nom, fpwm)
%AFE_HW_PARAM  Hardware data of the three-phase three-level T-type AFE of afe_dq_ttype.slx: split DC link,
%   LC filter on the grid side, per-unit bases.
%
%   hw = afe_hw_param(application_voltage, pwr_nom, fpwm)
%     application_voltage : [V] rms line-to-line grid voltage: 690, 480 or 400 (any other value: 400 V set)
%     pwr_nom             : [W] nominal power of the converter
%     fpwm                : [Hz] switching frequency (stored in fpwm_base)
%
%   Local replacement of three_phase_afe_hwdata + hw_afe_three_phase_setup of the library: same data sets,
%   same formulas and same field names, so that the per-unit scaling of the model does not change. Fields
%   read by afe_dq_ttype.slx: ubez, ibez, udc_nom, CFi1, CFi2, Rudc_measure, LFu, LFu_cm, RLFu, RLFu_cm,
%   CFu, RCFu.
%
%   Bases (convention of the library data set):
%     ubez = us_nom*sqrt(2/3)              peak phase-to-neutral grid voltage (563.4 V at 690 V)
%     is_nom = pwr_nom/sqrt(3)/us_nom/0.9  rms current with a 10 % margin on the power (232.4 A at 250 kW)
%     ibez = is_nom*sqrt(2)                peak current base (328.7 A); 1 pu of active current at 1 pu of
%                                          voltage is 3/2*ubez*ibez = 277.8 kW
%     udc_nom                              whole DC link (1070 V at 690 V), base of udc_pu; each bank of the
%                                          split link is at udc_nom/2
%
%   DC link: two capacitor banks in series (block dclink_for_three_levels: Cdc1 = CFi1, Cdc2 = CFi2, the
%   center tap is the neutral point np of the T-type bridge), 2 x 14.4 mF = 7.2 mF in series (CFi), each
%   bank with a 107 kOhm measurement divider (Rudc_measure). Filter (block LCL_filter): three-phase
%   inductor LFu (positive sequence) / LFu_cm (zero sequence) with RLFu, RLFu_cm, then CFu in star with
%   RCFu in series (floating star point), then the secondary of the grid transformer. The legacy init
%   overwrote CFi with 2*Cdc_dc1 of the PSFBC data set after the setup: the model reads CFi1 and CFi2, not
%   CFi, so that line had no effect and is not repeated here.

    switch application_voltage
        case 690
            hw.name = 'afe690V';
            hw.us_nom = 690;                % [V] rms line-to-line
            hw.f_nom = 50;                  % [Hz]
            hw.udc_nom = 1070;              % [V]
            LFu = 500e-6; LFu_cm = 500e-6;  % [H]
            hw.CFu = 200e-6;                % [F]
            hw.Rbrake = 4;                  % [Ohm] (not in the model)
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

    % split DC link (block dclink_for_three_levels)
    hw.CFi = 7.2e-3;                                        % [F] series of the two banks
    hw.RCFi = 1e-3;                                         % [Ohm] (informative, the block has no ESR)
    hw.CFi1 = 2*hw.CFi;                                     % [F] upper bank, 14.4 mF
    hw.CFi2 = 2*hw.CFi;                                     % [F] lower bank, 14.4 mF
    hw.RCFi1 = hw.RCFi/2;                                   % [Ohm] (informative)
    hw.RCFi2 = hw.RCFi/2;                                   % [Ohm] (informative)
    hw.Rudc_measure = 107e3;                                % [Ohm] measurement divider of the block, across each bank
    hw.udc1_init = hw.udc_nom/2;                            % [V] initial voltage of each bank (the block reads udc_nom/2)
    hw.udc2_init = hw.udc_nom/2;                            % [V]

    % grid filter (block LCL_filter: three-phase inductor, CFu in star, then the transformer secondary)
    hw.LFu = LFu;                                           % [H] positive-sequence inductance
    hw.LFu_cm = LFu_cm;                                     % [H] zero-sequence inductance
    hw.RLFu = 157*0.05*LFu;                                 % [Ohm] 5 % of the reactance at 25 Hz (library rule)
    hw.RLFu_cm = 157*0.05*LFu_cm;                           % [Ohm]
    hw.RCFu = 50e-3;                                        % [Ohm] series resistance of each capacitor

    fprintf('Hardware three-phase T-type AFE: %s, %.0f kW, %.0f V, %.0f Hz | ubez = %.1f V, ibez = %.1f A (is_nom %.1f A rms), 1 pu = %.1f kW | udc_nom = %.0f V\n', ...
        hw.name, pwr_nom/1e3, hw.us_nom, hw.f_nom, hw.ubez, hw.ibez, hw.is_nom, hw.norm.Pbez/1e3, hw.udc_nom);
    fprintf('DC link 2 x %.1f mF (%.1f mF in series) | filter LFu = %.0f uH (%.3f pu), CFu = %.0f uF star (%.3f pu), RCFu = %.0f mOhm\n', ...
        hw.CFi1*1e3, hw.CFi*1e3, hw.LFu*1e6, hw.LFu/hw.norm.Lbez, hw.CFu*1e6, hw.CFu/hw.norm.Cbez, hw.RCFu*1e3);
    fprintf('---------------------------\n');
end
