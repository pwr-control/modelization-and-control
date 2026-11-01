function hw = inv_hw_param(application_voltage, pwr_nom, fpwm)
%INV_HW_PARAM  Hardware data of the three-phase three-level T-type inverter of mpc_ttype_psm: output inductor,
%   split DC link, per-unit bases of the inverter data set.
%
%   hw = inv_hw_param(application_voltage, pwr_nom, fpwm)
%     application_voltage : [V] rms line-to-line voltage class of the data sets: 690, 480 or 400 (any other
%                           value: 400 V set)
%     pwr_nom             : [W] nominal power of the inverter
%     fpwm                : [Hz] frequency of the control timer (stored in fpwm_base)
%
%   Local copy of three_phase_inverter_hwdata + hw_inverter_three_phase_setup of the library for the
%   inverter, and of the DC-link part of three_phase_afe_hwdata (the legacy model read the DC link of the
%   AFE data set: hwdata.afe.CFi1, hwdata.afe.CFi2, hwdata.afe.udc_nom). Same data sets, same formulas, same
%   field names. Fields read by mpc_ttype_psm.slx:
%     LFi, LFi_cm, RLFi, RLFi_cm      block inv_psm_mod1/inverter_output_filter/threephase inductor: Lp, L0,
%                                     Rp, R0
%     CFi1, CFi2                      block inv_psm_mod1/dclink/dclink (dclink_for_three_levels): Cdc1_m
%                                     (upper capacitor, P to neutral point), Cdc2_m (lower capacitor)
%     udc_nom                         same block: Vdc_scale (the DC voltages go to the control in pu of
%                                     udc_nom) and the initial voltage udc_nom/2 of each capacitor
%     Rudc_measure                    same block: measuring resistor across each capacitor (literal 1e6 in
%                                     the legacy model)
%
%   Bases of the data set (not used by the control, which works in the bases of the machine):
%     ubez = us_nom*sqrt(2/3)    peak phase voltage (449.1 V for the 690 V set, us_nom 550 V)
%     ibez = is_nom*sqrt(2)      peak current (523.3 A, is_nom 370 A rms)
%
%   DC link: two banks in series (2 x 14.4 mF = 7.2 mF), the center tap is the neutral point of the T-type
%   bridge, 1 MOhm measuring resistor across each bank. The battery of the model sits across the whole link
%   (P to N), so udc is set by the battery; nothing but the bridge and the measuring resistors acts on the
%   neutral point.

    switch application_voltage
        case 690
            hw.name = 'inv690V_250kW';
            hw.us_nom = 550;                % [V] rms line-to-line, nominal output voltage
            hw.is_nom = 370;                % [A] rms, nominal output current
            hw.udc_nom = 1070;              % [V]
            LFi = 230e-6; LFi_cm = 230e-6;  % [H]
        case 480
            hw.name = 'inv480V_250kW';
            hw.us_nom = 400;                % as in the library data set (400 V, not 480 V)
            hw.is_nom = 470;
            hw.udc_nom = 750;
            LFi = 160e-6; LFi_cm = 160e-6;
        otherwise
            hw.name = 'inv400V_250kW';
            hw.us_nom = 400;
            hw.is_nom = 470;
            hw.udc_nom = 660;
            LFi = 160e-6; LFi_cm = 160e-6;
    end
    hw.application_voltage = application_voltage;
    hw.pwr_nom = pwr_nom;                                   % [W]
    hw.f_nom = 15.6;                                        % [Hz] (field f_nom of the library data set)
    hw.fpwm_base = fpwm;                                    % [Hz]

    % bases (hw_inverter_three_phase_setup)
    hw.ubez = hw.us_nom*sqrt(2/3);                          % [V] peak phase-to-neutral
    hw.ibez = hw.is_nom*sqrt(2);                            % [A] peak

    % output inductor (block threephase inductor): positive and zero sequence
    hw.LFi = LFi;                                           % [H]
    hw.LFi_cm = LFi_cm;                                     % [H]
    hw.RLFi = 157*0.05*LFi;                                 % [Ohm] 5 % of the reactance at 25 Hz (library rule)
    hw.RLFi_cm = 157*0.05*LFi_cm;                           % [Ohm]

    % split DC link (DC-link data of the AFE data set of the same voltage class, as the legacy model)
    hw.CFi = 7.2e-3;                                        % [F] series of the two banks
    hw.RCFi = 1e-3;                                         % [Ohm] (informative, the block has no ESR)
    hw.CFi1 = 2*hw.CFi;                                     % [F] upper bank (P to neutral point), 14.4 mF
    hw.CFi2 = 2*hw.CFi;                                     % [F] lower bank (neutral point to N), 14.4 mF
    hw.RCFi1 = hw.RCFi/2;                                   % [Ohm] (informative)
    hw.RCFi2 = hw.RCFi/2;                                   % [Ohm] (informative)
    hw.Rudc_measure = 1e6;                                  % [Ohm] measuring resistor of the block, across each bank

    fprintf('Hardware inverter: %s, %.0f kW, %.0f V, %.0f A rms | ubez = %.1f V, ibez = %.1f A | udc_nom = %.0f V\n', ...
        hw.name, pwr_nom/1e3, hw.us_nom, hw.is_nom, hw.ubez, hw.ibez, hw.udc_nom);
    fprintf('Output inductor LFi = %.0f uH (RLFi = %.2f mOhm) | split DC link 2 x %.1f mF (%.1f mF in series), Rudc_measure %.0f kOhm\n', ...
        hw.LFi*1e6, hw.RLFi*1e3, hw.CFi1*1e3, hw.CFi*1e3, hw.Rudc_measure/1e3);
end
